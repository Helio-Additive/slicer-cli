//! Library entrypoint for slicing a model to G-code using a BambuStudio-format
//! `project_settings.config` JSON.
//!
//! This is the in-process equivalent of the `slicer-cli slice --settings <json>`
//! path: it loads an STL (or 3MF) model, materializes a BambuStudio settings JSON
//! into the Rust config structs, builds the [`Print`] pipeline, runs
//! `Print::process()`, and exports G-code.
//!
//! Used by the `helio-slicer-cli` crate for its `--engine rust` and `compare`
//! commands so the Rust engine runs as a library rather than a subprocess.

use anyhow::{Context, Result};
use log::info;
use std::fs;
use std::path::Path;

use crate::geometry::Point3F;
use crate::print::{Print, PrintObject};
use crate::print_config::{PrintConfig, PrintObjectConfig};
use crate::region_config::PrintRegionConfig;
use crate::stl::read_stl_file as load_stl;

/// Slice a model file to G-code using a BambuStudio `project_settings.config` JSON.
///
/// * `input` — path to the model. Currently only STL is supported (3MF returns an
///   error; the `slicer-cli` binary handles 3MF separately via its own loader).
/// * `settings_json` — path to a BambuStudio-format settings JSON (the same JSON the
///   C++ `slicer_cli` binary accepts via `--config`).
/// * `output` — path to write the generated G-code to.
pub fn slice_to_gcode(input: &Path, settings_json: &Path, output: &Path) -> Result<()> {
    info!("Slicing (in-process): {:?}", input);

    // Load mesh — only STL is wired for the library entrypoint.
    let is_3mf = input
        .extension()
        .map_or(false, |e| e.eq_ignore_ascii_case("3mf"));
    if is_3mf {
        anyhow::bail!(
            "3MF input is not supported by the in-process Rust engine entrypoint \
             (use the slicer-cli binary's 3mf path); got {:?}",
            input
        );
    }

    info!("Loading STL...");
    let mut mesh = load_stl(input).with_context(|| format!("Failed to load STL: {:?}", input))?;
    info!("Loaded {} triangles", mesh.triangle_count());

    // Load BambuStudio project_settings.config JSON.
    info!("Loading BambuStudio settings from {:?}", settings_json);
    let settings_str = fs::read_to_string(settings_json)
        .with_context(|| format!("Failed to read settings: {:?}", settings_json))?;
    let mut settings_value: serde_json::Value = serde_json::from_str(&settings_str)
        .with_context(|| format!("Failed to parse settings JSON: {:?}", settings_json))?;
    patch_filament_overrides_in_json(&mut settings_value);
    let raw_settings_json = Some(settings_value.clone());
    let (print_config, object_config, region_config) = load_bambustudio_settings(&settings_value)?;

    // Match the C++ `slicer_cli` (the parity reference): it slices a bare STL
    // AS-IS in XY — it does NOT bed-center it (that's a GUI-only behavior). The
    // earlier XY auto-centering shifted every body coordinate by ~bed_center,
    // which is invisible to material/move-count metrics but breaks byte-parity
    // of the G-code coordinates. So apply NO XY translate; only drop the model
    // onto the bed surface (Z=0), which the CLI also does (no-op when the STL
    // already sits at Z>=0).
    {
        let bbox = mesh.bounding_box();
        let dz = -bbox.min.z;
        if dz != 0.0 {
            mesh.translate(Point3F { x: 0.0, y: 0.0, z: dz });
            info!("Placed model on bed surface: dz={:.3} (no XY centering, matching C++ slicer_cli)", dz);
        }
    }

    // C++ BambuStudio slices the mesh AFTER a center-store-then-instance-place round trip:
    // the ModelVolume mesh is stored centered on its bounding box (PrintObject volume), and the
    // instance/object trafo translates it back at slice time (PrintObjectSlice.cpp:60,
    // `params2.trafo = params2.trafo * volume.get_matrix()`, applied per-vertex). In exact math
    // that round trip is identity, but in f32 it QUANTIZES: f32 is coarser away from the origin,
    // so a vertex at f32(0.3) becomes f32(f32(0.3 - center_z) + center_z) = 0.299999237 once
    // center_z is large (Benchy center_z = 24 -> ~21 ULPs lost). rust slices the mesh in its raw
    // STL frame and SKIPS this round trip, so geometry that sits at exactly f32(layer-midpoint)
    // stays bit-coincident with the slice plane. The Benchy cabin floor is a flat plate at exactly
    // f32(0.3) == slice_z(li=1); coincident slicing hits the degenerate on-plane facet case and
    // emits ~8 spurious interior hole-loops -> the bottom-bridge/internal-solid cascade (R63-R65).
    // Replicate C++'s round trip so the floor lands 7.75e-7 below the plane exactly as C++
    // slices it. Must be done in f32 (our mesh is f64; an f64 round trip is a no-op).
    mesh.quantize_f32_center_roundtrip();

    // FRAME_PAIR: faithful C++ coordinate frame. C++ slices through
    // trafo_centered() (mesh − center_offset.xy, Print.hpp:376) and at export
    // subtracts m_origin = center_offset (PrintObject.cpp:108 / point_to_gcode),
    // netting gcode = raw − 2·center_offset for slicer_cli single-instance. rust
    // sliced+exported in the raw frame. Apply the slice-time XY center here (AFTER
    // the Z round-trip, so R65's floor is untouched) and stash the center as the
    // export origin. Gated until verified.
    let mut frame_origin = (0.0, 0.0);
    if std::env::var("FRAME_PAIR").is_ok() {
        frame_origin = mesh.slice_center_xy();
        info!("FRAME_PAIR slice-centered by ({:.4},{:.4}); export origin = same",
            frame_origin.0, frame_origin.1);
    }

    // Create PrintObject — slicing happens internally during Print::process().
    info!("Creating PrintObject...");
    let print_object = PrintObject::with_config(mesh, object_config);

    // Create Print and add object.
    info!("Creating Print...");
    let mut print = Print::new();
    *print.config_mut() = print_config;
    print.set_default_region_config(region_config);
    print.raw_settings = raw_settings_json;
    // FRAME_PAIR export origin (= the slice center applied above).
    print.gcode_origin = frame_origin;
    print.set_status_callback(|percent, message| {
        info!("Progress: {}% - {}", percent, message);
    });
    print.add_object(print_object);

    // Run Print::process() pipeline.
    info!("Running Print::process() pipeline...");
    print
        .process(None, false)
        .with_context(|| "Failed to process print")?;

    // Export G-code using Print::export_gcode().
    info!("Exporting G-code...");
    print
        .export_gcode(output)
        .with_context(|| format!("Failed to export G-code to {:?}", output))?;

    info!("Output written to: {:?}", output);
    info!(
        "Layers: {}",
        print.objects().first().map(|o| o.layers().len()).unwrap_or(0)
    );

    Ok(())
}

/// Load BambuStudio project_settings.config JSON and create config structs.
///
/// Uses `set_deserialize()` methods on config structs to iterate ALL keys
/// from the JSON, rather than cherry-picking specific keys. This ensures
/// every setting from the project is applied.
fn load_bambustudio_settings(
    json: &serde_json::Value,
) -> Result<(PrintConfig, PrintObjectConfig, PrintRegionConfig)> {
    let mut print_config = PrintConfig::default();
    let mut object_config = PrintObjectConfig::default();
    let mut region_config = PrintRegionConfig::default();

    // Iterate all keys from the JSON and dispatch to config structs.
    let mut recognized = 0u32;
    let mut unrecognized = 0u32;
    if let Some(obj) = json.as_object() {
        for (key, value) in obj {
            // Extract string value (scalar or first element of array).
            let str_val = match value {
                serde_json::Value::String(s) => s.clone(),
                serde_json::Value::Array(arr) => match arr.first() {
                    Some(serde_json::Value::String(s)) => s.clone(),
                    _ => continue,
                },
                _ => continue,
            };

            // Try each config struct — keys may apply to multiple structs.
            let mut found = false;
            if print_config.set_deserialize(key, &str_val) {
                found = true;
            }
            if object_config.set_deserialize(key, &str_val) {
                found = true;
            }
            if region_config.set_deserialize(key, &str_val) {
                found = true;
            }

            if found {
                recognized += 1;
            } else {
                unrecognized += 1;
            }
        }
    }

    info!(
        "Config keys: {} recognized, {} unrecognized (gcode templates, UI settings, etc.)",
        recognized, unrecognized
    );

    // === Special handling: bed temperature from curr_bed_type ===
    apply_bed_temperature(&mut print_config, json);

    // === Special handling: bed dimensions from printable_area ===
    apply_bed_dimensions(&mut print_config, json);

    // === Special handling: brim_type → brim_width logic ===
    apply_brim_logic(&mut print_config, json);

    // === Filament overrides: filament_retraction_* takes precedence over machine retraction_* ===
    apply_filament_overrides(&mut print_config, json);

    // === Extruder offset: "extruder_offset": ["XxY"] e.g. "0x2" ===
    apply_extruder_offset(&mut print_config, json);

    info!(
        "Settings loaded: layer_height={}, perimeters={}, infill={:.0}%, pattern={:?}",
        print_config.layer_height,
        object_config.perimeters,
        object_config.fill_density * 100.0,
        object_config.fill_pattern
    );
    info!(
        "Bed: {}x{}mm, nozzle={}mm, bed_temp={}C, nozzle_temp={}C",
        print_config.bed_size_x,
        print_config.bed_size_y,
        print_config.nozzle_diameter,
        print_config.bed_temperature,
        print_config.extruder_temperature
    );

    Ok((print_config, object_config, region_config))
}

/// Helper: get a string value from JSON (scalar or first element of array).
fn get_json_str(json: &serde_json::Value, key: &str) -> Option<String> {
    match json.get(key) {
        Some(serde_json::Value::String(s)) => Some(s.clone()),
        Some(serde_json::Value::Array(arr)) => {
            arr.first().and_then(|v| v.as_str()).map(|s| s.to_string())
        }
        _ => None,
    }
}

/// Helper: parse a float from JSON string (strip trailing %).
fn get_json_f64(json: &serde_json::Value, key: &str) -> Option<f64> {
    get_json_str(json, key).and_then(|s| s.trim_end_matches('%').parse::<f64>().ok())
}

/// Parse extruder_offset from JSON and set on config.
/// BambuStudio format: "extruder_offset": ["XxY"] e.g. "0x2" means X=0mm, Y=2mm.
fn apply_extruder_offset(config: &mut PrintConfig, json: &serde_json::Value) {
    if let Some(offset_str) = get_json_str(json, "extruder_offset") {
        let parts: Vec<&str> = offset_str.splitn(2, 'x').collect();
        if parts.len() == 2 {
            if let (Ok(ox), Ok(oy)) = (parts[0].parse::<f64>(), parts[1].parse::<f64>()) {
                config.extruder_offset_x = ox;
                config.extruder_offset_y = oy;
            }
        }
    }
}

/// Apply bed temperature based on curr_bed_type and plate-specific temp keys.
fn apply_bed_temperature(config: &mut PrintConfig, json: &serde_json::Value) {
    let bed_type = get_json_str(json, "curr_bed_type").unwrap_or_default();
    config.bed_temperature = match bed_type.as_str() {
        "Cool Plate" | "cool_plate" => get_json_f64(json, "cool_plate_temp").unwrap_or(35.0) as u32,
        "Engineering Plate" | "eng_plate" => {
            get_json_f64(json, "eng_plate_temp").unwrap_or(45.0) as u32
        }
        "Textured PEI Plate" | "textured_plate" => {
            get_json_f64(json, "textured_plate_temp").unwrap_or(55.0) as u32
        }
        "Hot Plate" | "hot_plate" | _ => {
            get_json_f64(json, "hot_plate_temp").unwrap_or(55.0) as u32
        }
    };
    config.first_layer_bed_temperature = match bed_type.as_str() {
        "Cool Plate" | "cool_plate" => {
            get_json_f64(json, "cool_plate_temp_initial_layer").unwrap_or(35.0) as u32
        }
        "Engineering Plate" | "eng_plate" => {
            get_json_f64(json, "eng_plate_temp_initial_layer").unwrap_or(45.0) as u32
        }
        "Textured PEI Plate" | "textured_plate" => {
            get_json_f64(json, "textured_plate_temp_initial_layer").unwrap_or(55.0) as u32
        }
        "Hot Plate" | "hot_plate" | _ => {
            get_json_f64(json, "hot_plate_temp_initial_layer").unwrap_or(55.0) as u32
        }
    };
}

/// Parse bed dimensions from printable_area array.
fn apply_bed_dimensions(config: &mut PrintConfig, json: &serde_json::Value) {
    if let Some(serde_json::Value::Array(arr)) = json.get("printable_area") {
        let mut max_x: f64 = 256.0;
        let mut max_y: f64 = 256.0;
        for v in arr {
            if let Some(s) = v.as_str() {
                let parts: Vec<&str> = s.split('x').collect();
                if parts.len() == 2 {
                    if let (Ok(x), Ok(y)) = (parts[0].parse::<f64>(), parts[1].parse::<f64>()) {
                        if x > max_x {
                            max_x = x;
                        }
                        if y > max_y {
                            max_y = y;
                        }
                    }
                }
            }
        }
        config.bed_size_x = max_x;
        config.bed_size_y = max_y;
    }
}

/// Apply brim logic: auto_brim/no_brim → 0, otherwise use brim_width from JSON.
///
/// When `brim_type` is absent from the config, fall back to BambuStudio's
/// default of `auto_brim` (PrintConfig.cpp:1480).
fn apply_brim_logic(config: &mut PrintConfig, json: &serde_json::Value) {
    let brim_type = match json.get("brim_type") {
        Some(serde_json::Value::String(s)) => s.clone(),
        _ => "auto_brim".to_string(),
    };
    if brim_type == "auto_brim" || brim_type == "no_brim" {
        config.brim_width = 0.0;
    } else if let Some(serde_json::Value::String(w)) = json.get("brim_width") {
        if let Ok(v) = w.parse::<f64>() {
            config.brim_width = v;
        }
    }
}

/// Patch raw settings JSON so filament overrides are reflected in the values
/// that the template processor reads. BambuStudio's PlaceholderParser applies
/// filament config on top of machine config.
fn patch_filament_overrides_in_json(json: &mut serde_json::Value) {
    let overrides: &[(&str, &str)] = &[
        ("filament_retraction_length", "retraction_length"),
        ("filament_retraction_speed", "retraction_speed"),
        ("filament_deretraction_speed", "deretraction_speed"),
        ("filament_z_hop", "z_hop"),
    ];

    if let Some(obj) = json.as_object_mut() {
        for &(filament_key, machine_key) in overrides {
            if let Some(filament_val) = obj.get(filament_key).cloned() {
                if let Some(arr) = filament_val.as_array() {
                    if let Some(first) = arr.first().and_then(|v| v.as_str()) {
                        if first != "nil" {
                            let new_arr: Vec<serde_json::Value> = arr
                                .iter()
                                .map(|v| {
                                    if let Some(s) = v.as_str() {
                                        if s == "nil" {
                                            obj.get(machine_key)
                                                .and_then(|mv| mv.as_array())
                                                .and_then(|ma| ma.first())
                                                .cloned()
                                                .unwrap_or(v.clone())
                                        } else {
                                            v.clone()
                                        }
                                    } else {
                                        v.clone()
                                    }
                                })
                                .collect();
                            obj.insert(machine_key.to_string(), serde_json::Value::Array(new_arr));
                        }
                    }
                }
            }
        }
    }
}

/// Apply filament-level retraction/speed overrides.
fn apply_filament_overrides(config: &mut PrintConfig, json: &serde_json::Value) {
    let get_filament_val = |key: &str| -> Option<f64> {
        match json.get(key) {
            Some(serde_json::Value::Array(arr)) => arr
                .first()
                .and_then(|v| v.as_str())
                .filter(|s| *s != "nil")
                .and_then(|s| s.parse::<f64>().ok()),
            Some(serde_json::Value::String(s)) if s != "nil" => s.parse::<f64>().ok(),
            _ => None,
        }
    };

    if let Some(v) = get_filament_val("filament_retraction_length") {
        config.retract_length = v;
    }
    if let Some(v) = get_filament_val("filament_retraction_speed") {
        config.retract_speed = v;
    }
    if let Some(v) = get_filament_val("filament_deretraction_speed") {
        config.deretract_speed = v;
    }
    if let Some(v) = get_filament_val("filament_z_hop") {
        config.retract_lift = v;
    }
}
