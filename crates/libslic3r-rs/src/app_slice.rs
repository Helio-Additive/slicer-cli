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
use std::io::Read;
use std::path::Path;

use crate::geometry::Point3F;
use crate::model::FacetsAnnotation;
use crate::print::{Print, PrintObject};
use crate::print_config::config_def::DynamicPrintConfig;
use crate::print_config::{PrintConfig, PrintObjectConfig};
use crate::region_config::PrintRegionConfig;
use crate::stl::read_stl_file as load_stl;
use crate::triangle_mesh::{Triangle, TriangleMesh};

/// PrintApply.cpp:1339-1435. Only these global options can raise the flag
/// without an override in a model object, part or layer range.
fn profile_can_enable_scarf(config: &DynamicPrintConfig) -> bool {
    use crate::config::{ConfigOptionEnumsGeneric, ConfigOptionResolver};
    (config.opt_bool("enable_circle_compensation") && config.opt_bool("apply_scarf_seam_on_circles"))
        || config.option("seam_slope_type").and_then(|opt| opt.get_int().ok()).is_some_and(|kind| kind != 0)
        || config.option("filament_scarf_seam_type")
            .and_then(|opt| opt.as_any().downcast_ref::<ConfigOptionEnumsGeneric>())
            .is_some_and(|opt| opt.values.iter().any(|&kind| kind != 0))
}

/// PrintApply.cpp 1323-1325 normalizes against the previous print, which has
/// no objects or used filaments on a fresh CLI slice.
fn normalized_model_config(mut config: DynamicPrintConfig) -> Result<DynamicPrintConfig> {
    config.normalize_fdm_1()?;
    config.normalize_fdm_2(0, 0, None)?;
    Ok(config)
}

/// Apply the flag before building the config block and typed processor config.
fn apply_model_scarf_flag(
    settings: &mut serde_json::Value,
    config: &DynamicPrintConfig,
    model: &crate::model::Model,
) {
    let has_scarf = config.opt_bool("has_scarf_joint_seam") || crate::print_apply::model_uses_scarf_seam(model, config);
    if has_scarf || settings.get("has_scarf_joint_seam").is_some() {
        settings["has_scarf_joint_seam"] = serde_json::Value::String(u8::from(has_scarf).to_string());
    }
}

const FEATURE_FILAMENT_KEYS: [&str; 3] = ["wall_filament", "sparse_infill_filament", "solid_infill_filament"];

/// PrintApply.cpp 1045-1055,1091 builds a region per part. The merged 3MF
/// mesh can use their assignments when all parts resolve to the same
/// three physical feature slots. Mixed slots need per-layer component ordering.
fn common_model_feature_slots(model: &crate::model::Model, config: &DynamicPrintConfig) -> Option<[usize; 3]> {
    use crate::gcode::wipe_tower::{cfg_bools, cfg_floats};

    // PrintApply.cpp 1649. Embedded settings from an unmarked 3MF may have
    // been ignored by the native-format importer.
    let count = cfg_floats(config, "filament_diameter").len();
    if count <= 1 {
        return None;
    }
    let mut base = crate::print::faithful::static_config_defaults("PrintRegionConfig");
    for key in FEATURE_FILAMENT_KEYS {
        *base.opt_int_mut(key) = config.opt_int(key);
    }
    let mixed = cfg_bools(config, "filament_is_mixed");
    let mut selected = None;
    let mut variant = Vec::new();
    for object in &model.objects {
        if !object.printable
            || object.instances.is_empty()
            || object.instances.iter().any(|instance| !instance.printable)
            || !object.layer_config_ranges.is_empty()
        {
            return None;
        }
        for volume in &object.volumes {
            if !volume.is_model_part() || !volume.printable() || volume.is_mm_painted() {
                return None;
            }
            // PrintObject.cpp 3742-3806 resolves the part over its object,
            // material and global feature settings, then clamps the result.
            let region = crate::print_apply::region_config_from_model_volume(
                &base, None, volume, object, Some(model), count, &mut variant,
            );
            let slots = FEATURE_FILAMENT_KEYS.map(|key| region.opt_int(key));
            if slots.iter().any(|&s| s <= 0 || s as usize > count || mixed.get(s as usize - 1) == Some(&true)) {
                return None;
            }
            let slots = slots.map(|s| s as usize);
            if selected.is_some_and(|prev| prev != slots) {
                return None;
            }
            selected = Some(slots);
        }
    }
    selected
}

/// bbs_3mf.cpp:2263-2331. The importer decides whether to read embedded
/// settings and custom tool changes based on the project's generator.
fn load_bbs_model_and_config(path: &Path) -> Result<(crate::model::Model, DynamicPrintConfig)> {
    use crate::config::{ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
    use crate::format::bbs_3mf::{load_bbs_3mf, LoadStrategy};

    let backup = tempfile::tempdir().context("Creating the 3MF model backup directory")?;
    let mut model = crate::model::Model::new();
    model.set_backup_path(backup.path().to_str().context("3MF backup path is not UTF-8")?);
    let mut config = DynamicPrintConfig::new();
    let mut substitutions = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Enable);
    let loaded = load_bbs_3mf(
        path.to_str().context("3MF model path is not UTF-8")?, &mut config, &mut substitutions,
        &mut model, &mut Vec::new(), &mut Vec::new(), None, &mut crate::semver::Semver::new(), None,
        LoadStrategy::LoadModel | LoadStrategy::LoadConfig, None, 0, None, None, None, None,
    )?;
    anyhow::ensure!(loaded, "Failed to load the model settings from {}", path.display());
    model.set_backup_path("detach");
    Ok((model, config))
}

/// Slice a model file to G-code using a BambuStudio `project_settings.config` JSON.
///
/// * `input` — path to the model. Currently only STL is supported (3MF returns an
///   error; the `slicer-cli` binary handles 3MF separately via its own loader).
/// * `settings_json` — path to a BambuStudio-format settings JSON (the same JSON the
///   C++ `slicer_cli` binary accepts via `--config`).
/// * `output` — path to write the generated G-code to.
pub fn slice_to_gcode(input: &Path, settings_json: &Path, output: &Path) -> Result<()> {
    slice_to_gcode_with_legacy_gcode(input, settings_json, output, LegacyGcode::Normalize)
}

/// Slice an STL with the caller's per-job G-code template policy.
pub fn slice_to_gcode_with_legacy_gcode(
    input: &Path, settings_json: &Path, output: &Path, legacy_gcode: LegacyGcode<'_>,
) -> Result<()> {
    let (print, __proc_s) = slice_stl_print(input, settings_json, legacy_gcode)?;
    let __timing = crate::probe_enabled("SLICE_PHASE_TIMING");

    // Export G-code using Print::export_gcode().
    info!("Exporting G-code...");
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("Failed to create output directory {:?}", parent))?;
    }
    let __t_exp = std::time::Instant::now();
    print
        .export_gcode(output)
        .with_context(|| format!("Failed to export G-code to {:?}", output))?;
    if __timing {
        let __exp_s = __t_exp.elapsed().as_secs_f64();
        eprintln!(
            "--- top-level (s): process {:.3} + export_gcode {:.3} = {:.3} ---",
            __proc_s,
            __exp_s,
            __proc_s + __exp_s
        );
    }

    info!("Output written to: {:?}", output);
    info!(
        "Layers: {}",
        print.objects().first().map(|o| o.layers().len()).unwrap_or(0)
    );

    Ok(())
}

/// The part of [`slice_to_gcode`] before the G-code export: load and place the STL, build the Print and run
/// `Print::process()`. Returns the processed Print (for tests that read its objects' layers and support layers).
pub fn slice_stl_to_print(input: &Path, settings_json: &Path) -> Result<Print> {
    slice_stl_print(input, settings_json, LegacyGcode::Normalize).map(|(print, _)| print)
}

fn slice_stl_print(input: &Path, settings_json: &Path, legacy_gcode: LegacyGcode<'_>) -> Result<(Print, f64)> {
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
    // main.cpp:9438-9593: the check of the settings' values before anything slices (FRONT_END_CONFIG_VALIDATE_PIN).
    let config = crate::print::full_print_config_from_settings(&settings_value);
    check_config_values(&config)?;
    legacy_gcode.apply(&mut settings_value);
    if crate::faithful_gate("PRINT_APPLY_SCARF_SEAM") && profile_can_enable_scarf(&config) {
        use crate::model::{Model, ModelMesh, ModelVolume, ModelVolumeType};
        // STL.cpp:36-39, Model.cpp:605-621,3416-3466. get_extruders uses
        // the unpainted facets of the part to include the default extruder.
        // No convex hull is needed to read filament assignments.
        let mut model = Model::new();
        let object = model.add_object_new();
        let mut part = ModelVolume::new_shared(ModelMesh::from_triangle_mesh(&mesh).into(), ModelVolumeType::ModelPart);
        part.set_model_object(Some(object.id()));
        object.volumes.push(part);
        apply_model_scarf_flag(&mut settings_value, &normalized_model_config(config)?, &model);
    }
    let config_block_settings_json = config_block_settings_before_patch(&settings_value);
    engine_settings_as_the_pin_reads(&mut settings_value);
    patch_filament_overrides_in_json(&mut settings_value);
    let raw_settings_json = Some(settings_value.clone());
    let (print_config, object_config, region_config) = load_bambustudio_settings(&settings_value)?;

    // R809 STL_DESKTOP_PLACE (faithful ON; `=0` restores the as-is frame of the
    // pre-R809 driver): `slicer_cli` places a model file on the plate the way
    // the desktop does on load (cli_model_load.cpp `desktop_place_on_bed`).
    let frame_origin_legacy = (0.0, 0.0);
    let desktop_place = if crate::faithful_gate("STL_DESKTOP_PLACE") {
        Some(desktop_place_stl(&mesh, &settings_value))
    } else {
        None
    };
    if let Some(place) = &desktop_place {
        info!(
            "Placed on the plate: instance shift ({}, {}) scaled, z {:.6}",
            place.shift.0, place.shift.1, place.trafo16[11]
        );
        mesh = place.placed_mesh.clone();
    } else {
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
    // R87 FRAME_UNIFY: the Z+24 round-trip is reproduced by C++'s params2.trafo
    // applied in the unified slice transform (the shim), so the separate mesh-bake
    // must be SKIPPED to avoid double-counting. R65 floor survives via the trafo.
    if !crate::faithful_gate("FRAME_UNIFY") {
        mesh.quantize_f32_center_roundtrip();
    }

    // FRAME_PAIR: subtract the slice center_offset from the mesh so verts bit-match
    // C++'s centered slice frame (verified exact in f32). The centered slice frame
    // IS C++'s gcode frame (slicer_cli single-instance) — export origin stays 0.
    if std::env::var("FRAME_PAIR").is_ok() {
        let co = mesh.slice_center_xy();
        info!("FRAME_PAIR slice-centered by ({:.4},{:.4})", co.0, co.1);
    }

    }
    let frame_origin = if desktop_place.is_some() { (0.0, 0.0) } else { frame_origin_legacy };

    // Gate TREE_SUPPORT_ORGANIC_PIN (opt_in_gate, R933): the Model of the placed object for the support step's
    // faithful Print (print.rs, `assemble_faithful_print`). Off: no Model is built and the live Print's stays empty.
    let tree_pin_model = match &desktop_place {
        // Only where print.rs's gated support block will run (a tree support type with supports or a raft): with
        // the gate on, Benchy (no supports) ran away in R933 (>10 min, 11 GB) with the Model built; not localized.
        Some(place)
            if crate::opt_in_gate("TREE_SUPPORT_ORGANIC_PIN")
                && object_config.support_type == crate::print_config::SupportType::Tree
                && (object_config.enable_support || object_config.enforce_support_layers > 0 || object_config.raft_layers > 0) =>
        {
            let name = input.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
            Some(live_model_from_desktop_placement(place, &settings_value, &name))
        }
        _ => None,
    };
    // Probe TREE_PIN_MODEL_PROBE (diagnostics only, R934): builds the same Model for any placed STL, times it and
    // drops it, to localize the R933 runaway; the Print never sees it, so the output does not change.
    if crate::probe_enabled("TREE_PIN_MODEL_PROBE") {
        if let Some(place) = &desktop_place {
            let t0 = std::time::Instant::now();
            {
                // TriangleMesh.cpp:47-59 (fill_initial_stats) step by step, timed.
                use crate::normal_utils::{indexed_triangle_set, Vec3crd, Vec3f};
                let mut its = indexed_triangle_set::default();
                its.vertices = place.local_mesh.vertices().iter().map(|p| Vec3f::new(p.x as f32, p.y as f32, p.z as f32)).collect();
                its.indices = (0..place.local_mesh.triangle_count())
                    .map(|i| {
                        let t = place.local_mesh.triangle_indices(i);
                        Vec3crd::new(t[0] as i32, t[1] as i32, t[2] as i32)
                    })
                    .collect();
                eprintln!("TREE_PIN_MODEL_PROBE: its {} vertices {} faces at {:.3} s", its.vertices.len(), its.indices.len(), t0.elapsed().as_secs_f64());
                let v = crate::triangle_mesh::its_volume(&its);
                eprintln!("TREE_PIN_MODEL_PROBE: its_volume {} at {:.3} s", v, t0.elapsed().as_secs_f64());
                let its_diag = crate::triangle_set_sampling::indexed_triangle_set { vertices: its.vertices.clone(), indices: its.indices.clone() };
                let mut nb = Vec::new();
                let st = crate::mesh_diagnostics::its_quick_diagnostics(&its_diag, Some(&mut nb));
                eprintln!("TREE_PIN_MODEL_PROBE: its_quick_diagnostics open_edges {} at {:.3} s", st.open_edges, t0.elapsed().as_secs_f64());
            }
            let model = live_model_from_desktop_placement(place, &settings_value, "probe");
            eprintln!("TREE_PIN_MODEL_PROBE: Model of {} triangles built in {:.3} s", place.local_mesh.triangle_count(), t0.elapsed().as_secs_f64());
            drop(model);
        }
    }

    // Create PrintObject — slicing happens internally during Print::process().
    info!("Creating PrintObject...");
    let mut print_object = PrintObject::with_config(mesh, object_config);
    if let Some(place) = desktop_place {
        // The volume-local mesh goes through the exact composed f32 slicing
        // transform (SLICE_VOLUME_TRAFO, R805), as the 3MF path's volumes do.
        print_object.paint_tb_trafo = Some(place.trafo16);
        print_object.raw_local_mesh = Some(place.local_mesh);
        print_object.slice_center_override = Some(place.center_mm);
        print_object.occlusion_place = Some((place.volume_offset, place.object_translation));
    }

    // Create Print and add object.
    info!("Creating Print...");
    let mut print = Print::new();
    // main.cpp:1456 — print.apply(model, config). Applies the resolved config +
    // default region config; objects are added separately below.
    print.apply(print_config, region_config);
    print.raw_settings = raw_settings_json;
    print.config_block_settings = config_block_settings_json;
    if let Some(model) = tree_pin_model {
        // After `apply`, which clears the live Print's model (print.rs).
        print.set_model(model);
    }
    // FRAME_PAIR export origin (= the slice center applied above).
    print.gcode_origin = frame_origin;
    print.set_status_callback(|percent, message| {
        info!("Progress: {}% - {}", percent, message);
    });
    print.add_object(print_object);

    // Run Print::process() pipeline.
    // main.cpp:1486 — Print::validate() before slicing; abort on a hard error
    // (empty `.string` means valid). Warnings (is_warning) are non-fatal.
    let validation = validate_and_emit_warnings(&print);
    if !validation.string.is_empty() && !validation.is_warning {
        return Err(validation_refusal(validation));
    }

    info!("Running Print::process() pipeline...");
    let __t_proc = std::time::Instant::now();
    print
        .process(None, false)
        .with_context(|| "Failed to process print")?;
    let __proc_s = __t_proc.elapsed().as_secs_f64();
    Ok((print, __proc_s))
}

/// A hard error of `Print::validate()`, carried to the front end so that it refuses the job the way
/// `slicer_cli` does (main.cpp:11539-11636): `Validation error: <message>` and the tip on stderr, and the
/// return code the official CLI maps the exception type to (BambuStudio.cpp 7094-7111 at 5873b5f).
#[derive(Debug, Clone)]
pub struct ValidationRefusal {
    pub exception: crate::print_base::StringObjectException,
}

impl std::fmt::Display for ValidationRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "validation error: {}", self.exception.string)
    }
}

impl std::error::Error for ValidationRefusal {}

impl ValidationRefusal {
    /// main.cpp:11541-11545 (stderr): the engine's sentence, then the tip.
    pub fn stderr_text(&self) -> String {
        format!(
            "Validation error: {}\n\nTip: You may need to specify proper config files:\n  --machine, --filament, and --process options",
            self.exception.string
        )
    }

    /// main.cpp:166-185 `string_exception_tag`.
    pub fn tag(&self) -> &'static str {
        use crate::print_base::StringExceptionType as T;
        match self.exception.r#type {
            T::StringExceptNotDefined => "STRING_EXCEPT_NOT_DEFINED",
            T::StringExceptFilamentNotMatchBedType => "STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE",
            T::StringExceptFilamentsDifferentTemp => "STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP",
            T::StringExceptObjectCollisionInSeqPrint => "STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT",
            T::StringExceptObjectCollisionInLayerPrint => "STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT",
            T::StringExceptLayerHeightExceedsLimit => "STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT",
            T::StringExceptCount => "STRING_EXCEPT_COUNT",
        }
    }

    /// main.cpp:465-484 `emit_validation_event` (Bambu engine): the event's JSON, keys in nlohmann's
    /// (sorted) order: event, hypertext (if any), message, opt_key (if any), params (if any), tag.
    pub fn event_payload(&self) -> String {
        let e = &self.exception;
        let mut m = serde_json::Map::new();
        m.insert("event".into(), (if e.is_warning { "validation_warning" } else { "validation_error" }).into());
        if !e.hypetext.is_empty() {
            m.insert("hypertext".into(), e.hypetext.clone().into());
        }
        m.insert("message".into(), e.string.clone().into());
        if !e.opt_key.is_empty() {
            m.insert("opt_key".into(), e.opt_key.clone().into());
        }
        if !e.params.is_empty() {
            m.insert("params".into(), e.params.clone().into());
        }
        m.insert("tag".into(), self.tag().into());
        // serde_json's Map without `preserve_order` is a BTreeMap: the keys come out sorted, as
        // nlohmann::json's std::map does.
        serde_json::Value::Object(m).to_string()
    }

    /// diagnostic_output.hpp:14-18 `write_event_now`: the record written to stdout.
    pub fn event_record(&self) -> String {
        format!("\n[[SLICER_EVENT]] {}\n", self.event_payload())
    }

    /// main.cpp:11635-11636 (`set_outcome_failure(outcome, validate_error, message); return 1;`): the process
    /// exits with 1; the CLI_* code goes into result.json.
    pub fn exit_code(&self) -> i32 {
        1
    }

    /// main.cpp:11546-11560: `CLI_VALIDATE_ERROR` (-51), or the code of the exception type
    /// (Utils.hpp:52, 62-65): what slicer_cli writes as result.json's return code.
    pub fn return_code(&self) -> i32 {
        use crate::print_base::StringExceptionType as T;
        match self.exception.r#type {
            T::StringExceptFilamentNotMatchBedType => -61,
            T::StringExceptFilamentsDifferentTemp => -62,
            T::StringExceptObjectCollisionInSeqPrint => -63,
            T::StringExceptObjectCollisionInLayerPrint => -64,
            _ => -51,
        }
    }
}

/// main.cpp:11483-11494, 11528-11531 (Bambu engine): `Print::validate` with the warnings out-parameter; every
/// warning is joined with a newline into the first (BambuStudio.cpp 7531-7536), which is emitted as the
/// validation event on stdout (main.cpp:465-484, `validation_warning` or `validation_error` by its
/// `is_warning`) and followed by `Validation warning: <text>`. Gate FRONT_END_VALIDATE_WARNINGS_PIN
/// (faithful, R948); off, `Print::validate` without warnings.
fn validate_and_emit_warnings(print: &Print) -> crate::print_base::StringObjectException {
    if !crate::faithful_gate("FRONT_END_VALIDATE_WARNINGS_PIN") {
        return print.validate();
    }
    let mut validation_warnings: Vec<crate::print_base::StringObjectException> = Vec::new();
    let validation_result = print.validate_with_warnings(Some(&mut validation_warnings));
    let mut validation_warning = crate::print_base::StringObjectException::default();
    for item in &validation_warnings {
        if validation_warning.string.is_empty() {
            validation_warning = item.clone();
        } else {
            if !validation_warning.string.ends_with('\n') {
                validation_warning.string.push('\n');
            }
            validation_warning.string += &item.string;
        }
    }
    if !validation_warning.string.is_empty() {
        use std::io::Write;
        let text = validation_warning.string.clone();
        let record = ValidationRefusal { exception: validation_warning }.event_record();
        let mut out = std::io::stdout();
        let _ = out.write_all(record.as_bytes());
        let _ = writeln!(out, "Validation warning: {text}");
        let _ = out.flush();
    }
    validation_result
}

/// The error a hard validation error becomes. Gate FRONT_END_VALIDATE_REFUSAL_PIN (faithful, R947): a
/// [`ValidationRefusal`] the front end prints and maps as `slicer_cli` does; off, the earlier plain
/// `validation error: <message>` (a slicing error, -100).
fn validation_refusal(validation: crate::print_base::StringObjectException) -> anyhow::Error {
    if crate::faithful_gate("FRONT_END_VALIDATE_REFUSAL_PIN") {
        return ValidationRefusal { exception: validation }.into();
    }
    anyhow::anyhow!("validation error: {}", validation.string)
}

/// `--flag-name` of a setting, as main.cpp:9524-9526 / 9574-9577 write it (`--` and the key with `_` as `-`).
fn setting_flag(key: &str) -> String {
    format!("--{}", key.replace('_', "-"))
}

/// A refusal of the configuration check (main.cpp:9549-9593): the settings `DynamicPrintConfig::validate(true)`
/// (PrintConfig.cpp:8969-9007) finds invalid that the desktop's own check (`validate(false)`) does not find the
/// same way: the `config_refused` event on stdout, `Param values in 3mf/config error: <sentence>` on stderr, exit
/// 1 and CLI_INVALID_VALUES_IN_3MF (Utils.hpp) as the return code.
#[derive(Debug, Clone)]
pub struct ConfigRefusal {
    /// The refused settings and why, in key order (`std::map`).
    pub settings: std::collections::BTreeMap<String, String>,
    /// main.cpp:9554-9582: the sentence.
    pub sentence: String,
}

impl std::fmt::Display for ConfigRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "config refused: {}", self.sentence)
    }
}

impl std::error::Error for ConfigRefusal {}

impl ConfigRefusal {
    /// main.cpp:9554-9582 (Bambu engine; no unknown enum values here): the engine's sentence, each refused
    /// setting as `key: why`, then the flags that override them.
    pub fn new(settings: std::collections::BTreeMap<String, String>) -> Self {
        let mut sentence = String::from("BambuStudio cannot print this file as it is: ");
        let mut first = true;
        for (key, why) in &settings {
            sentence += &format!("{}{}: {}", if first { "" } else { "; " }, key, why);
            first = false;
        }
        sentence += ".";
        if !settings.is_empty() {
            let flags: Vec<String> = settings.keys().map(|k| setting_flag(k)).collect();
            sentence += &format!(" Give {} a value in range to override it.", flags.join(", "));
        }
        sentence += " --allow-substitution lets the engine use its own default for a value it does not know instead.";
        ConfigRefusal { settings, sentence }
    }

    /// main.cpp:9583-9587: the event's JSON, keys in nlohmann's (sorted) order.
    pub fn event_payload(&self) -> String {
        let mut items = serde_json::Map::new();
        for (key, why) in &self.settings {
            items.insert(key.clone(), why.clone().into());
        }
        let mut m = serde_json::Map::new();
        m.insert("event".into(), "config_refused".into());
        m.insert("message".into(), self.sentence.clone().into());
        m.insert("settings".into(), serde_json::Value::Object(items));
        m.insert("tag".into(), "InvalidValues".into());
        m.insert("unknown_values".into(), serde_json::Value::Array(Vec::new()));
        serde_json::Value::Object(m).to_string()
    }

    /// diagnostic_output.hpp:14-18 `write_event_now`: the record written to stdout.
    pub fn event_record(&self) -> String {
        format!("\n[[SLICER_EVENT]] {}\n", self.event_payload())
    }

    /// main.cpp:9588: the line on stderr.
    pub fn stderr_text(&self) -> String {
        format!("Param values in 3mf/config error: {}", self.sentence)
    }

    /// main.cpp:9589-9590 (`set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence); return 1;`).
    pub fn exit_code(&self) -> i32 {
        1
    }

    /// main.cpp:9589: the code result.json carries.
    pub fn return_code(&self) -> i32 {
        crate::utils::CLI_INVALID_VALUES_IN_3MF
    }
}

/// The front end's check of the settings' values (main.cpp:9438-9593): `DynamicPrintConfig::validate(true)`
/// (PrintConfig.cpp:8969-9007, `validate(const FullPrintConfig&, true)` :10641-10916) over the settings as the
/// file states them; a finding the desktop's check (`validate(false)`) makes the same way is the file's own and
/// only noted (the `InvalidValuesNoted` warning, main.cpp:9512-9547: stdout event, `Warning: <sentence>` on
/// stderr, main.cpp:7894-7897); any other (the spiral-vase checks) refuses the job ([`ConfigRefusal`]). This
/// program takes no setting on its command line, so the settings are both the checked and the file's ones
/// (`with_overrides` and `file_only` of main.cpp are the same configuration). Gate FRONT_END_CONFIG_VALIDATE_PIN
/// (faithful, R949); off, no check.
fn check_config_values(config: &DynamicPrintConfig) -> Result<()> {
    if !crate::faithful_gate("FRONT_END_CONFIG_VALIDATE_PIN") {
        return Ok(());
    }
    let found = config.validate(true).map_err(|e| anyhow::anyhow!("{}", e))?;
    let mut validity: std::collections::BTreeMap<String, String> = std::collections::BTreeMap::new();
    let mut noted: std::collections::BTreeMap<String, String> = std::collections::BTreeMap::new();
    if !found.is_empty() {
        let desktop = config.validate(false).map_err(|e| anyhow::anyhow!("{}", e))?;
        for (key, why) in found {
            if desktop.get(&key) == Some(&why) {
                noted.insert(key, why);
            } else {
                validity.insert(key, why);
            }
        }
    }
    if !noted.is_empty() && validity.is_empty() {
        // main.cpp:9512-9547: the plate is 1 for this program's one-plate default call (`checked > 0 ? checked : 1`).
        let mut sentence = String::from("Invalid values found in the 3MF/config: ");
        let mut items = serde_json::Map::new();
        let mut flags = String::new();
        for (key, why) in &noted {
            sentence += &format!("{}{}: {}", if items.is_empty() { "" } else { "; " }, key, why);
            items.insert(key.clone(), why.clone().into());
            flags += &format!("{}{}", if flags.is_empty() { "" } else { ", " }, setting_flag(key));
        }
        sentence += &format!(
            ". The run slices with them as they are, and the slice's own check of each object against its nozzle and \
             plate still applies. Give {} a value in range to change it.",
            flags
        );
        let mut m = serde_json::Map::new();
        m.insert("event".into(), "warning".into());
        m.insert("message".into(), sentence.clone().into());
        m.insert("plate_id".into(), 1.into());
        m.insert("settings".into(), serde_json::Value::Object(items));
        m.insert("tag".into(), "InvalidValuesNoted".into());
        use std::io::Write;
        let mut out = std::io::stdout();
        let _ = out.write_all(format!("\n[[SLICER_EVENT]] {}\n", serde_json::Value::Object(m)).as_bytes());
        let _ = out.flush();
        eprintln!("Warning: {sentence}");
    }
    if !validity.is_empty() {
        return Err(ConfigRefusal::new(validity).into());
    }
    Ok(())
}

/// Treatment of the legacy placeholder for a single slice (main.cpp 540–610,
/// 11558–11559). Library entry points use the silent normalization mode.
#[derive(Clone, Copy)]
pub enum LegacyGcode<'a> {
    Preserve,
    Normalize,
    NormalizeAndReport(&'a dyn Fn(&[&str], usize)),
}

pub const LEGACY_FILAMENT_TOKEN: &str = "initial_no_support_filament_id";
pub const BOUND_EXTRUDER_TOKEN: &str = "initial_no_support_extruder";

impl LegacyGcode<'_> {
    /// Rewrite whole identifiers in native key order and return their total count.
    pub fn apply(self, settings: &mut serde_json::Value) -> usize {
        if matches!(self, Self::Preserve) || !crate::faithful_gate("GCODE_TOKEN_ALIAS") {
            return 0;
        }
        let (count, keys) = normalize_legacy_gcode_tokens(settings);
        if count > 0 {
            if let Self::NormalizeAndReport(report) = self {
                report(&keys, count);
            }
        }
        count
    }
}

fn normalize_legacy_gcode_tokens(settings: &mut serde_json::Value) -> (usize, Vec<&'static str>) {
    // main.cpp kGcodeStringKeys and kGcodeStringsKeys (the Bambu engine's own).
    const KEYS: &[&str] = &[
        "machine_start_gcode", "machine_end_gcode", "before_layer_change_gcode",
        "layer_change_gcode", "change_filament_gcode", "time_lapse_gcode",
        "machine_pause_gcode", "printing_by_object_gcode", "template_custom_gcode",
        "wrapping_detection_gcode", "filament_start_gcode", "filament_end_gcode",
    ];
    fn is_ident(c: u8) -> bool {
        c.is_ascii_alphanumeric() || c == b'_'
    }
    fn rewrite_text(text: &mut String) -> usize {
        if !text.contains(LEGACY_FILAMENT_TOKEN) {
            return 0;
        }
        let v = text.as_str();
        let mut count = 0;
        let bytes = v.as_bytes();
        let mut out = String::with_capacity(v.len());
        let mut pos = 0;
        while pos < v.len() {
            let Some(found) = v[pos..].find(LEGACY_FILAMENT_TOKEN).map(|i| i + pos) else {
                out.push_str(&v[pos..]);
                break;
            };
            let after = found + LEGACY_FILAMENT_TOKEN.len();
            let left_ok = found == 0 || !is_ident(bytes[found - 1]);
            let right_ok = after >= v.len() || !is_ident(bytes[after]);
            if left_ok && right_ok {
                out.push_str(&v[pos..found]);
                out.push_str(BOUND_EXTRUDER_TOKEN);
                count += 1;
                pos = after;
            } else {
                out.push_str(&v[pos..found + 1]);
                pos = found + 1;
            }
        }
        *text = out;
        count
    }
    let mut count = 0;
    let mut keys = Vec::new();
    if let Some(map) = settings.as_object_mut() {
        for &key in KEYS {
            let rewrites = match map.get_mut(key) {
                Some(serde_json::Value::String(text)) => rewrite_text(text),
                Some(serde_json::Value::Array(items)) => items.iter_mut().map(|item| match item {
                    serde_json::Value::String(text) => rewrite_text(text),
                    _ => 0,
                }).sum(),
                _ => 0,
            };
            if rewrites > 0 {
                keys.push(key);
                count += rewrites;
            }
        }
    }
    (count, keys)
}

/// What `desktop_place_stl` works out for one model file.
struct DesktopPlacement {
    /// The volume's own mesh: the file's vertices minus the bounding-box centre,
    /// in f32 (ModelVolume::center_geometry_after_creation, Model.cpp:3513).
    local_mesh: TriangleMesh,
    /// `trafo_centered() * volume.get_matrix()` (PrintObjectSlice.cpp), row-major.
    trafo16: [f64; 16],
    /// The print instance's shift, scaled (PrintApply.cpp + PrintObject.cpp:114).
    shift: (i64, i64),
    /// `unscale(shift)`: what the G-code export adds to every slice coordinate.
    center_mm: (f64, f64),
    /// The mesh in plate coordinates, for everything that is not the slicer.
    placed_mesh: TriangleMesh,
    /// The volume matrix's translation, then `trafo_centered()`'s.
    volume_offset: [f64; 3],
    object_translation: [f64; 3],
}

/// The Model `slicer_cli` holds for one placed model file (gate `TREE_SUPPORT_ORGANIC_PIN`, R933): what the pin's
/// loader and `desktop_place_on_bed` leave in it (cli_model_load.cpp 280-378 on main; Model.cpp:605-621
/// `Model::add_object(name, path, mesh)`): one ModelObject named after the file, one model-part ModelVolume whose
/// mesh is stored minus its bounding-box centre (ModelVolume::center_geometry_after_creation, Model.cpp:3513, the
/// f32 vertices `desktop_place_stl` computed) with the volume offset the two `center_around_origin` calls leave
/// (Model.cpp:1870), and one ModelInstance at the bed centre lifted by `ensure_on_bed` (Model.cpp:1883). The
/// object's config is empty, as an STL carries no per-object settings. `print_objects_from_model_object`
/// (PrintApply.cpp) and the PrintObject constructor (PrintObject.cpp:82-114) then reach the same instance shift and
/// `m_center_offset` as `desktop_place_stl`, so the faithful object's layers are in the live layers' frame.
fn live_model_from_desktop_placement(place: &DesktopPlacement, settings: &serde_json::Value, name: &str) -> crate::model::Model {
    use crate::geometry::geometry::Vec3d;
    use crate::model::{Model, ModelMesh, ModelVolumeType};
    use crate::normal_utils::{indexed_triangle_set, Vec3crd, Vec3f};
    let mut its = indexed_triangle_set::default();
    its.vertices = place.local_mesh.vertices().iter().map(|p| Vec3f::new(p.x as f32, p.y as f32, p.z as f32)).collect();
    its.indices = (0..place.local_mesh.triangle_count())
        .map(|i| {
            let t = place.local_mesh.triangle_indices(i);
            Vec3crd::new(t[0] as i32, t[1] as i32, t[2] as i32)
        })
        .collect();
    let (bed_x, bed_y) = bed_box_center(settings);
    let mut model = Model::new();
    let o = model.add_object_new();
    o.name = name.to_string();
    {
        // Model.cpp:1450-1461: already centred (center_geometry_after_creation ran in desktop_place_stl).
        let v = o.add_volume_mesh(ModelMesh::from_its(its), ModelVolumeType::ModelPart, false);
        v.name = name.to_string();
        v.set_offset(&Vec3d::new(place.volume_offset[0], place.volume_offset[1], place.volume_offset[2]));
    }
    o.add_instance().set_offset(&Vec3d::new(bed_x, bed_y, place.object_translation[2]));
    o.invalidate_bounding_box();
    model
}

/// The bounding box of the `printable_area` points (cli_model_load.cpp `bed_box`;
/// 200 x 200 when the setting is absent).
fn bed_box_center(settings: &serde_json::Value) -> (f64, f64) {
    let (mut xl, mut yl, mut xh, mut yh) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    let mut defined = false;
    let mut take = |text: &str| {
        for point in text.split(',') {
            if let Some((x, y)) = point.trim().split_once('x') {
                if let (Ok(x), Ok(y)) = (x.trim().parse::<f64>(), y.trim().parse::<f64>()) {
                    xl = xl.min(x);
                    xh = xh.max(x);
                    yl = yl.min(y);
                    yh = yh.max(y);
                    defined = true;
                }
            }
        }
    };
    match settings.get("printable_area") {
        Some(serde_json::Value::Array(points)) => {
            for point in points {
                if let Some(text) = point.as_str() {
                    take(text);
                }
            }
        }
        Some(serde_json::Value::String(text)) => take(text),
        _ => {}
    }
    if !defined {
        return (100.0, 100.0);
    }
    // BoundingBoxBase::center: (min + max) / 2.
    ((xl + xh) / 2.0, (yl + yh) / 2.0)
}

/// Place one model file on the plate as `slicer_cli` does.
///
/// Port of the single-object, single-volume path of `desktop_place_on_bed`
/// (cli_model_load.cpp 280-378 on main) with the libslic3r calls it makes
/// (Model.cpp at 56a57d0), every step in the arithmetic the C++ uses, because
/// the residues decide slice-plane rounding:
///
/// 1. `ModelVolume::center_geometry_after_creation` (3513): the mesh is stored
///    minus its bounding-box centre, subtracted in f32; the volume offset takes
///    the centre in f64.
/// 2. `center_around_origin(false)` then `center_around_origin()` (1870): each
///    moves the volume offset by minus the centre of the f64 bounding box of the
///    transformed mesh (`transformed_bounding_box`, 1760).
/// 3. `ensure_on_bed()` (1883) with the new instance: the instance's z offset is
///    minus `get_instance_min_z` (3021).
/// 4. The instance's xy offset is the bed centre.
/// 5. `print_objects_from_model_object` (PrintApply.cpp): the instance shift is
///    `Point::new_scale` of that offset, which truncates (Point.hpp:185).
/// 6. `PrintObject` (PrintObject.cpp:82-114): `m_center_offset` is `new_scale` of
///    the raw bounding box's centre and is added to the shift;
///    `trafo_centered()` (Print.hpp) takes it off again in mm.
fn desktop_place_stl(mesh: &TriangleMesh, settings: &serde_json::Value) -> DesktopPlacement {
    let sf = crate::libslic3r::SCALING_FACTOR;
    let verts = mesh.vertices();

    // 1. f32 bounding box of the file's vertices; its centre in f64.
    let mut lo = [f32::MAX; 3];
    let mut hi = [f32::MIN; 3];
    for v in verts {
        let p = [v.x as f32, v.y as f32, v.z as f32];
        for a in 0..3 {
            lo[a] = lo[a].min(p[a]);
            hi[a] = hi[a].max(p[a]);
        }
    }
    let mut vo = [0.0f64; 3];
    for a in 0..3 {
        vo[a] = (lo[a] as f64 + hi[a] as f64) / 2.0;
    }
    let c32 = [vo[0] as f32, vo[1] as f32, vo[2] as f32];
    // translate(-(float)shift): v + (-c) in f32.
    let local: Vec<[f32; 3]> = verts
        .iter()
        .map(|v| {
            [
                v.x as f32 + (-c32[0]),
                v.y as f32 + (-c32[1]),
                v.z as f32 + (-c32[2]),
            ]
        })
        .collect();

    // f64 bounding box of `translation * v.cast<double>()`.
    let bbox = |t: &[f64; 3]| -> ([f64; 3], [f64; 3]) {
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for p in &local {
            for a in 0..3 {
                let x = p[a] as f64 + t[a];
                lo[a] = lo[a].min(x);
                hi[a] = hi[a].max(x);
            }
        }
        (lo, hi)
    };

    // 2. center_around_origin(false), center_around_origin().
    for _ in 0..2 {
        let (lo, hi) = bbox(&vo);
        for a in 0..3 {
            let shift = -((lo[a] + hi[a]) / 2.0);
            vo[a] += shift;
        }
    }

    // 3. ensure_on_bed: z_offset = -get_min_z(); the instance offset was zero.
    let (lo, hi) = bbox(&vo);
    let min_z = lo[2] + 0.0;
    let z_offset = -min_z;
    let inst_z = if z_offset != 0.0 { 0.0 + z_offset } else { 0.0 };

    // 4./5. The instance at the bed centre; its shift truncates.
    let (bed_x, bed_y) = bed_box_center(settings);
    let mut shift = ((bed_x / sf) as i64, (bed_y / sf) as i64);

    // 6. m_center_offset from raw_bounding_box (instance matrix without its
    //    translation, so the box above), added to the shift.
    let co = (
        (((lo[0] + hi[0]) / 2.0) / sf) as i64,
        (((lo[1] + hi[1]) / 2.0) / sf) as i64,
    );
    shift.0 += co.0;
    shift.1 += co.1;

    // trafo_centered() * volume.get_matrix(): pure translations.
    let tx = vo[0] + (-(co.0 as f64 * sf));
    let ty = vo[1] + (-(co.1 as f64 * sf));
    let tz = vo[2] + inst_z;
    let trafo16 = [
        1.0, 0.0, 0.0, tx,
        0.0, 1.0, 0.0, ty,
        0.0, 0.0, 1.0, tz,
        0.0, 0.0, 0.0, 1.0,
    ];
    let center_mm = (shift.0 as f64 * sf, shift.1 as f64 * sf);

    let tris: Vec<crate::triangle_mesh::Triangle> = (0..mesh.triangle_count())
        .map(|i| {
            let idx = mesh.triangle_indices(i);
            crate::triangle_mesh::Triangle::new(idx[0], idx[1], idx[2])
        })
        .collect();
    let local_mesh = TriangleMesh::from_parts(
        local
            .iter()
            .map(|p| Point3F { x: p[0] as f64, y: p[1] as f64, z: p[2] as f64 })
            .collect(),
        tris.clone(),
    );
    let placed_mesh = TriangleMesh::from_parts(
        local
            .iter()
            .map(|p| Point3F {
                x: p[0] as f64 + tx + center_mm.0,
                y: p[1] as f64 + ty + center_mm.1,
                z: p[2] as f64 + tz,
            })
            .collect(),
        tris,
    );

    DesktopPlacement {
        local_mesh,
        trafo16,
        shift,
        center_mm,
        placed_mesh,
        volume_offset: vo,
        object_translation: [-(co.0 as f64 * sf), -(co.1 as f64 * sf), inst_z],
    }
}

/// Slice a 3MF model file to G-code using the Rust engine.
///
/// Unlike [`slice_to_gcode`] (STL, which needs an explicit BambuStudio settings
/// JSON), a BambuStudio 3MF carries its own `Metadata/project_settings.config`.
/// This entrypoint reads that embedded config (or an explicit `settings_override`
/// when one is supplied) and slices the merged mesh.
///
/// This mirrors the internal `slicer-cli` binary's proven 3MF slice path so the
/// in-process host (`helio-slicer-cli --engine rust`) and the standalone binary
/// produce identical G-code.
///
/// The geometry loader merges objects. The shared region can carry the
/// wall, sparse and solid filament assignments common to every part.
/// Distinct assignments between parts require separate slicing regions.
///
////// * `input` — path to a `.3mf` file.
/// * `settings_override` — optional BambuStudio settings JSON that takes
///   precedence over the 3MF's embedded config.
/// * `output` — path to write the generated G-code to.
pub fn slice_3mf_to_gcode(
    input: &Path,
    settings_override: Option<&Path>,
    output: &Path,
) -> Result<()> {
    slice_3mf_to_gcode_with_legacy_gcode(input, settings_override, output, LegacyGcode::Normalize)
}

/// Slice a 3MF with the caller's per-job G-code template policy.
pub fn slice_3mf_to_gcode_with_legacy_gcode(
    input: &Path, settings_override: Option<&Path>, output: &Path, legacy_gcode: LegacyGcode<'_>,
) -> Result<()> {
    info!("Slicing 3MF (in-process): {:?}", input);

    // Load the merged mesh + embedded settings + per-instance identify_ids.
    info!("Loading 3MF...");
    let (loaded, scoped) = load_3mf_model(input, true)
        .with_context(|| format!("Failed to load 3MF: {:?}", input))?;
    let Loaded3mf {
        mut mesh,
        settings_json: embedded_settings,
        identify_ids,
        mmu_facets,
        seam_facets,
        negative_mesh,
        paint_segments,
        paint_raw_vertices,
        paint_instance_matrix,
        negative_segments,
    } = loaded;
    info!("Loaded {} triangles", mesh.triangle_count());
    // R703 — negative volumes are loaded but NOT yet subtracted; the per-layer
    // `diff_ex` that mirrors C++ `slices_to_regions` (:403-421) is the next step.
    // Reported here so the load side is verifiable on its own.
    if negative_mesh.triangle_count() > 0 {
        info!(
            "Loaded {} negative-volume triangles (not yet subtracted)",
            negative_mesh.triangle_count()
        );
    }
    // Painted multi-material: decode the annotation over the merged mesh to
    // learn which extruder slots are painted (campaign B layer 3). The painted
    // regions are declared on the Print below; splitting the layer surfaces
    // into them (apply_mm_segmentation, layer 4) is not wired yet, so the
    // toolpaths are still single-material.
    // Painted-MMU decode happens AFTER bed-centering below so the extracted
    // sub-meshes share the sliced mesh's frame (layer zs / XY).

    // Settings source: an explicit override wins, else the 3MF's embedded config.
    let settings_str = if let Some(path) = settings_override {
        info!("Loading BambuStudio settings from {:?}", path);
        fs::read_to_string(path).with_context(|| format!("Failed to read settings: {:?}", path))?
    } else if let Some(embedded) = embedded_settings {
        info!("Using settings embedded in the 3MF");
        embedded
    } else {
        anyhow::bail!(
            "3MF {:?} has no embedded Metadata/project_settings.config and no \
             settings override was provided",
            input
        );
    };

    let mut settings_value: serde_json::Value =
        serde_json::from_str(&settings_str).with_context(|| "Failed to parse 3MF settings JSON")?;
    // main.cpp:9438-9593: the check of the settings' values before anything slices (FRONT_END_CONFIG_VALIDATE_PIN).
    let config = crate::print::full_print_config_from_settings(&settings_value);
    check_config_values(&config)?;
    legacy_gcode.apply(&mut settings_value);
    let needs_scarf = crate::faithful_gate("PRINT_APPLY_SCARF_SEAM")
        && (scoped.scarf || profile_can_enable_scarf(&config) || config.opt_bool("has_scarf_joint_seam"));
    // Validate global role assignments against the actual imported model as
    // well: a range, plate tool change or part may select a different tool.
    let route_features = crate::faithful_gate("FEATURE_FILAMENT_ORDER")
        && crate::faithful_gate("SCOPED_MODEL_FILAMENT")
        && crate::faithful_gate("REGION_CONFIG_WALL_FILAMENT")
        && crate::faithful_gate("REGION_CONFIG_INFILL_FILAMENTS")
        && crate::faithful_gate("REGION_CONFIG_FILAMENT_CLAMP");
    let global_slots = FEATURE_FILAMENT_KEYS.map(|key| config.opt_int(key).max(1));
    let separate_roles = global_slots.iter().any(|&slot| slot != global_slots[0]);
    let needs_filament = (scoped.filament || (route_features && separate_roles)) && mmu_facets.is_empty()
        && ["filament_diameter", "filament_colour"].iter().any(|key|
            settings_value[*key].as_array().is_some_and(|values| values.len() > 1));
    let mut model_slots = None;
    if needs_scarf || needs_filament {
        let (model, embedded) = load_bbs_model_and_config(input)?;
        let full = normalized_model_config(if settings_override.is_some() {
            config
        } else {
            crate::print_config::config_def::full_print_config_of(&embedded)?
        })?;
        if needs_scarf {
            apply_model_scarf_flag(&mut settings_value, &full, &model);
        }
        if needs_filament {
            model_slots = common_model_feature_slots(&model, &full);
            if model_slots.is_some_and(|[wall, sparse, solid]| wall != sparse || wall != solid)
                && (crate::gcode::wipe_tower::cfg_bools(&full, "filament_is_mixed").contains(&true)
                    || model.plates_custom_gcodes.values().any(|plate| {
                        plate.gcodes.iter().any(|item| item.gcode_type == crate::custom_g_code::Type::ToolChange)
                    }))
            {
                model_slots = None;
            }
        }
    }
    let config_block_settings_json = config_block_settings_before_patch(&settings_value);
    engine_settings_as_the_pin_reads(&mut settings_value);
    patch_filament_overrides_in_json(&mut settings_value);
    let raw_settings_json = Some(settings_value.clone());
    let (print_config, object_config, mut region_config) = load_bambustudio_settings(&settings_value)?;
    let route_features = route_features && !print_config.flush_into_objects
        && !object_config.enable_support && object_config.enforce_support_layers == 0
        && object_config.raft_layers == 0;
    let model_slots = model_slots.filter(|slots| slots.iter().all(|&slot| slot <= print_config.num_filaments()));
    if let Some([wall, sparse, solid]) = model_slots {
        if route_features || (wall == sparse && wall == solid) {
            region_config.wall_filament = wall;
            region_config.sparse_infill_filament = sparse;
            region_config.solid_infill_filament = solid;
        }
    }

    // Drop the model onto the bed surface (Z=0), and XY-center it on the bed.
    //
    // R430/R447 — the XY centering was WRONG for parity and is now GONE. C++ `slicer_cli` honours the placement baked into
    // the 3MF's build-item / component transforms and the plate origin; it does
    // not re-center (auto-centering is GUI behavior — the same trap the bare-STL
    // path above documents). Measured on Majora: with centering the rust
    // toolpaths sit (+4.20, -15.14) mm off C++ while the object size matches to
    // 0.04% (a pure translation), which collapses the per-layer silhouette IoU;
    // with `THREEMF_NO_CENTER=1` the extrusion bbox matches C++ EXACTLY
    // (center delta 0.000/0.001) and wall-line IoU jumps 6.97% -> 95.42%.
    //
    // This was gated until R447: dropping the centering used to break painted
    // segmentation (painted_cube fell from 50 tool changes to 0). That turned out
    // to be an unrelated EdgeGrid bug — our `create_from_contours` reset the bbox
    // instead of merging into the pre-set one (C++ EdgeGrid.cpp:145-151), so a
    // painted line on the object's own silhouette fell outside the grid and was
    // clipped away. With that fixed, no-centering is unconditional.
    {
        let bbox = mesh.bounding_box();
        let dz = -bbox.min.z;
        mesh.translate(Point3F { x: 0.0, y: 0.0, z: dz });
        info!(
            "Placed model on bed surface: dz={:.3} (no XY centering, matching C++ slicer_cli)",
            dz
        );
    }

    // Decode the painted-MMU annotation over the (now bed-centered) mesh:
    // which extruder slots are painted + one sub-mesh per painted slot
    // (C++: ModelVolume::mmu_segmentation_facets → get_facets, MMS.cpp:2226).
    let mut painting_extruders: Vec<u8> = Vec::new();
    let mut painted_submeshes: Vec<(u8, crate::normal_utils::indexed_triangle_set)> = Vec::new();
    let mut painted_submeshes_strict: Vec<(u8, crate::normal_utils::indexed_triangle_set)> =
        Vec::new();
    // R786/R787 (gate MMS_PAINT_FRAME) — rebuild the mesh vertices the way
    // NATIVE's painted-facet projection sees them (MMS.cpp:2303):
    //   tr = Transform3f(print_object.trafo() * volume.get_matrix())
    //   facet[p] = tr * volume_local_vertex
    // where trafo() = instance matrix with the XY translation ZEROED
    // (PrintApply.cpp:148-152; Z translation kept) and the placement rides
    // center_offset = Point::new_scale(raw_bounding_box().center().xy)
    // (PrintObject.cpp:88, Model.cpp:1559-73: bbox over MODEL parts under the
    // fully-untranslated instance matrix ∘ volume matrix), subtracted from the
    // scaled painted-line ints (MMS.cpp:2373). The eigen shim applies the exact
    // f32 affine (rotation matmul = the R85 wall); paint_frame_offset carries
    // the integer center_offset to the projection.
    let mut paint_frame_offset: Option<(i64, i64)> = None;
    let mut instance_shift: Option<(i64, i64)> = None;
    let mut paint_tb_trafo: Option<[f64; 16]> = None;
    let mut paint_tc: Option<[f64; 12]> = None;
    let mut paint_trafo_z: Option<f64> = None;
    let paint_pair: Option<([f64; 16], [f64; 16])> = if crate::faithful_gate("MMS_PAINT_FRAME")
        && !mmu_facets.is_empty()
        && paint_segments.len() == 1
        && paint_segments[0].0 == 0
        && paint_raw_vertices.len() == mesh.vertices().len()
    {
        // b∘a with translation at [9..12] (a applied first).
        fn compose12(a: &[f64; 12], b: &[f64; 12]) -> [f64; 12] {
            [
                b[0] * a[0] + b[1] * a[3] + b[2] * a[6],
                b[0] * a[1] + b[1] * a[4] + b[2] * a[7],
                b[0] * a[2] + b[1] * a[5] + b[2] * a[8],
                b[3] * a[0] + b[4] * a[3] + b[5] * a[6],
                b[3] * a[1] + b[4] * a[4] + b[5] * a[7],
                b[3] * a[2] + b[4] * a[5] + b[5] * a[8],
                b[6] * a[0] + b[7] * a[3] + b[8] * a[6],
                b[6] * a[1] + b[7] * a[4] + b[8] * a[7],
                b[6] * a[2] + b[7] * a[5] + b[8] * a[8],
                b[0] * a[9] + b[1] * a[10] + b[2] * a[11] + b[9],
                b[3] * a[9] + b[4] * a[10] + b[5] * a[11] + b[10],
                b[6] * a[9] + b[7] * a[10] + b[8] * a[11] + b[11],
            ]
        }
        let b = &paint_instance_matrix;
        // trafo(): XY translation zeroed, Z kept (PrintApply.cpp:151-152).
        let mut b0 = *b;
        b0[9] = 0.0;
        b0[10] = 0.0;
        // raw_bounding_box: instance matrix with NO translation at all
        // (get_matrix(true)) ∘ volume matrix, f64 over the f32-stored verts.
        let mut bnt = b0;
        bnt[11] = 0.0;
        let (mut bxl, mut byl, mut bxh, mut byh) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        // Bed-drop the loader applied to the PLACED mesh (translate to z=0):
        // native keeps trafo()'s Z, so the paint mesh needs the same drop for
        // its layer-z assignment. Derived from the placed (b ∘ vol) min z.
        let mut placed_minz = f64::MAX;
        for &(off, n, ref cvol) in &paint_segments {
            let m = compose12(cvol, &bnt);
            let pm = compose12(cvol, b);
            for v in &paint_raw_vertices[off..off + n] {
                let (x, y, z) = (v.x as f32 as f64, v.y as f32 as f64, v.z as f32 as f64);
                let tx = m[0] * x + m[1] * y + m[2] * z + m[9];
                let ty = m[3] * x + m[4] * y + m[5] * z + m[10];
                bxl = bxl.min(tx);
                bxh = bxh.max(tx);
                byl = byl.min(ty);
                byh = byh.max(ty);
                let pz = pm[6] * x + pm[7] * y + pm[8] * z + pm[11];
                placed_minz = placed_minz.min(pz);
            }
        }
        let dz = if placed_minz.is_finite() { -placed_minz } else { 0.0 };
        let (cx, cy) = ((bxl + bxh) * 0.5, (byl + byh) * 0.5);
        // Point::new_scale = coord_t(val / SCALING_FACTOR): TRUNCATION.
        let ox = (cx / crate::libslic3r::SCALING_FACTOR) as i64;
        let oy = (cy / crate::libslic3r::SCALING_FACTOR) as i64;
        paint_frame_offset = Some((ox, oy));
        // PrintApply.cpp:150 `shift = Point::new_scale(trafo[12], trafo[13])`,
        // then PrintObject.cpp:114 `shift += m_center_offset`.
        instance_shift = Some((
            (b[9] / crate::libslic3r::SCALING_FACTOR) as i64 + ox,
            (b[10] / crate::libslic3r::SCALING_FACTOR) as i64 + oy,
        ));
        if crate::probe_enabled("PFDBG") {
            eprintln!("PFDBG center_offset=({},{}) center=({:.6},{:.6}) dz={:.6}", ox, oy, cx, cy, dz);
        }
        // Native composes the pair IN F32: trafo().cast<float>() * get_matrix()
        // .cast<float>() (MMS.cpp:2303). The bed drop rides trafo()'s Z (native
        // has none; dz reconciles our z=0 frame). Only the pair is exported —
        // the selector runs on the VOLUME-LOCAL mesh (splits happen in local
        // f32, THEN the split verts are transformed, exactly like native).
        let a16: [f64; 16] = [
            b0[0], b0[1], b0[2], b0[9],
            b0[3], b0[4], b0[5], b0[10],
            b0[6], b0[7], b0[8], b0[11] + dz,
            0.0, 0.0, 0.0, 1.0,
        ];
        let c = &paint_segments[0].2;
        let b16: [f64; 16] = [
            c[0], c[1], c[2], c[9],
            c[3], c[4], c[5], c[10],
            c[6], c[7], c[8], c[11],
            0.0, 0.0, 0.0, 1.0,
        ];
        // R788 — topbot volume_trafo = trafo_centered() * get_matrix()
        // (MMS.cpp:1345): trafo_centered = Translate(-unscale(center_offset)) ∘
        // trafo() (Print.hpp:376), composed in f64 like native's Transform3d
        // product; the slab shim then applies prescale ∘ trafo cast to f32
        // (make_trafo_for_slicing) over the LOCAL strict verts with voff 0.
        let mut tc = b0;
        tc[9] -= ox as f64 * crate::libslic3r::SCALING_FACTOR;
        tc[10] -= oy as f64 * crate::libslic3r::SCALING_FACTOR;
        paint_trafo_z = Some(tc[11]);
        tc[11] += dz;
        if crate::probe_enabled("OCCDBG") {
            eprintln!("OCCDBG-R tc b0z={:e} dz={:e} tc={:?} vol={:?}", b0[11], dz, tc, c);
        }
        let vt = compose12(c, &tc);
        paint_tc = Some(tc);
        paint_tb_trafo = Some([
            vt[0], vt[1], vt[2], vt[9],
            vt[3], vt[4], vt[5], vt[10],
            vt[6], vt[7], vt[8], vt[11],
            0.0, 0.0, 0.0, 1.0,
        ]);
        Some((a16, b16))
    } else {
        None
    };
    // R805 — raw volume-local mesh for exact-frame slicing (SLICE_VOLUME_TRAFO).
    let raw_local_mesh: Option<TriangleMesh> = if paint_pair.is_some() {
        let raw32: Vec<Point3F> = paint_raw_vertices
            .iter()
            .map(|v| Point3F { x: v.x as f32 as f64, y: v.y as f32 as f64, z: v.z as f32 as f64 })
            .collect();
        let tris: Vec<crate::triangle_mesh::Triangle> = (0..mesh.triangle_count())
            .map(|i| {
                let idx = mesh.triangle_indices(i);
                crate::triangle_mesh::Triangle::new(idx[0], idx[1], idx[2])
            })
            .collect();
        Some(TriangleMesh::from_parts(raw32, tris))
    } else {
        None
    };
    if !mmu_facets.is_empty() {
        // R787 — with the paint frame active, the selector operates on the
        // VOLUME-LOCAL mesh (f32 file verts): facet splits are computed in
        // local f32 exactly like native TriangleSelector, and the resulting
        // (possibly split) verts are then pushed through the f32 pair
        // transform — native's split-then-transform order.
        let sel_mesh = if paint_pair.is_some() {
            let raw32: Vec<Point3F> = paint_raw_vertices
                .iter()
                .map(|v| Point3F {
                    x: v.x as f32 as f64,
                    y: v.y as f32 as f64,
                    z: v.z as f32 as f64,
                })
                .collect();
            let tris: Vec<crate::triangle_mesh::Triangle> = (0..mesh.triangle_count())
                .map(|i| {
                    let idx = mesh.triangle_indices(i);
                    crate::triangle_mesh::Triangle::new(idx[0], idx[1], idx[2])
                })
                .collect();
            TriangleMesh::from_parts(raw32, tris)
        } else {
            mesh.clone()
        };
        let paint_transform_its = |its: &mut crate::triangle_mesh::indexed_triangle_set| {
            if let Some((a16, b16)) = &paint_pair {
                let mut flat: Vec<f32> = Vec::with_capacity(its.vertices.len() * 3);
                for v in &its.vertices {
                    flat.push(v.x);
                    flat.push(v.y);
                    flat.push(v.z);
                }
                let out = crate::triangle_mesh_slicer::transform_verts_affine_f32_pair(a16, b16, &flat);
                for (v, ch) in its.vertices.iter_mut().zip(out.chunks_exact(3)) {
                    v.x = ch[0];
                    v.y = ch[1];
                    v.z = ch[2];
                }
            }
        };
        let mut selector = crate::triangle_selector::TriangleSelector::new(sel_mesh, 0.0);
        selector.deserialize(
            &mmu_facets.data,
            false,
            crate::triangle_selector::EnforcerBlockerType::EXTRUDER_MAX,
            crate::triangle_selector::EnforcerBlockerType::NONE,
            crate::triangle_selector::EnforcerBlockerType::NONE,
        );
        for state in selector.used_states() {
            let mut its = selector.get_facets(state);
            if !its.indices.is_empty() {
                paint_transform_its(&mut its);
                painted_submeshes.push((state.0 as u8, its));
            }
        }
        // Ascending-extruder pairing drives the painted region order.
        painting_extruders = painted_submeshes.iter().map(|(e, _)| *e).collect();
        // R768 — strict per-state sub-meshes for top/bottom painted propagation
        // (C++ get_facets_strict over states 0..num_extruders incl. state 0 =
        // the unpainted rest, MMS.cpp:1354-1356).
        for state_i in 0..=(print_config.num_filaments() as u8) {
            let state = crate::triangle_selector::EnforcerBlockerType(state_i as i8);
            let its = selector.get_facets_strict(state);
            if !its.indices.is_empty() {
                // R788 — native keeps strict meshes LOCAL and hands
                // slice_mesh_slabs volume_trafo = trafo_centered() * get_matrix()
                // (MMS.cpp:1345-1355); the tb ctx carries that trafo instead.
                painted_submeshes_strict.push((state_i, its));
            }
        }
        info!(
            "3MF painted multi-material: {} painted facets, extruders {:?} — \
             segmenting per-layer painted regions (Tier-1)",
            mmu_facets.facet_count(),
            painting_extruders
        );
    }

    info!("Creating PrintObject...");
    let mut print_object = PrintObject::with_config(mesh, object_config);
    // The first instance's identify_id drives the `; OBJECT_ID:` comments.
    if let Some(&id) = identify_ids.first() {
        print_object.label_id = id;
    }
    print_object.painted_submeshes = painted_submeshes;
    print_object.paint_frame_offset = paint_frame_offset;
    print_object.instance_shift = instance_shift;
    print_object.paint_tb_trafo = paint_tb_trafo;
    print_object.raw_local_mesh = raw_local_mesh;
    // R805 — negative volumes through the exact per-volume frame.
    print_object.negative_volumes_raw = if let Some(tc) = paint_tc {
        negative_segments
            .iter()
            .map(|(verts, tris, vol_tf)| {
                let raw32: Vec<Point3F> = verts
                    .iter()
                    .map(|v| Point3F { x: v.x as f32 as f64, y: v.y as f32 as f64, z: v.z as f32 as f64 })
                    .collect();
                let tri: Vec<crate::triangle_mesh::Triangle> =
                    tris.iter().map(|t| crate::triangle_mesh::Triangle::new(t[0], t[1], t[2])).collect();
                let vt = compose12_rows(vol_tf, &tc);
                let t16: [f64; 16] = [
                    vt[0], vt[1], vt[2], vt[9],
                    vt[3], vt[4], vt[5], vt[10],
                    vt[6], vt[7], vt[8], vt[11],
                    0.0, 0.0, 0.0, 1.0,
                ];
                (TriangleMesh::from_parts(raw32, tri), t16)
            })
            .collect()
    } else {
        Vec::new()
    };
    // The volumes one by one, for the seam placer's occlusion mesh
    // (SeamPlacer.cpp:577-606): the part with its matrix, the negative volumes
    // with theirs, and trafo_centered().
    if let (Some(tc), Some(local)) = (paint_tc, print_object.raw_local_mesh.as_ref()) {
        let mut volumes: Vec<(TriangleMesh, [f64; 12], bool)> =
            vec![(local.clone(), paint_segments[0].2, false)];
        for (verts, tris, vol_tf) in negative_segments.iter() {
            let raw32: Vec<Point3F> = verts
                .iter()
                .map(|v| Point3F { x: v.x as f32 as f64, y: v.y as f32 as f64, z: v.z as f32 as f64 })
                .collect();
            let tri: Vec<crate::triangle_mesh::Triangle> =
                tris.iter().map(|t| crate::triangle_mesh::Triangle::new(t[0], t[1], t[2])).collect();
            volumes.push((TriangleMesh::from_parts(raw32, tri), *vol_tf, true));
        }
        // `tc` carries the port's bed drop in Z on top of trafo()'s own; the
        // oracle's trafo_centered() has trafo()'s Z alone.
        let mut tc_occlusion = tc;
        tc_occlusion[11] = paint_trafo_z.unwrap_or(tc[11]);
        // gather_enforcers_blockers (SeamPlacer.cpp:645-668): the painted seam
        // facets of the volume, in its own coordinates, through
        // `trafo_centered() * get_matrix()` (the product in double).
        if !seam_facets.is_empty() {
            let mut selector = crate::triangle_selector::TriangleSelector::new(local.clone(), 0.0);
            selector.deserialize(
                &seam_facets.data,
                false,
                crate::triangle_selector::EnforcerBlockerType::EXTRUDER_MAX,
                crate::triangle_selector::EnforcerBlockerType::NONE,
                crate::triangle_selector::EnforcerBlockerType::NONE,
            );
            let m = compose12_rows(&paint_segments[0].2, &tc_occlusion);
            let through = |mut its: crate::triangle_mesh::indexed_triangle_set| {
                for v in its.vertices.iter_mut() {
                    let (x, y, z) = (v.x as f64, v.y as f64, v.z as f64);
                    v.x = (m[0] * x + m[1] * y + m[2] * z + m[9]) as f32;
                    v.y = (m[3] * x + m[4] * y + m[5] * z + m[10]) as f32;
                    v.z = (m[6] * x + m[7] * y + m[8] * z + m[11]) as f32;
                }
                its
            };
            let enforcers = through(selector.get_facets(crate::triangle_selector::EnforcerBlockerType::ENFORCER));
            let blockers = through(selector.get_facets(crate::triangle_selector::EnforcerBlockerType::BLOCKER));
            print_object.seam_facets = Some((enforcers, blockers));
        }
        print_object.occlusion_volumes = Some((volumes, tc_occlusion));
    }
    print_object.painted_submeshes_strict = painted_submeshes_strict;
    print_object.num_total_filaments = print_config.num_filaments();
    // R704 — negative volumes travel with the object and are subtracted per
    // layer in PrintObject::slice (gate NEGATIVE_VOLUMES). They are NOT merged
    // into `mesh`: C++ keeps them as separate ModelVolumes and does the
    // subtraction in 2D per layer (slices_to_regions, PrintObjectSlice.cpp:403).
    print_object.set_negative_mesh(negative_mesh);

    info!("Creating Print...");
    let mut print = Print::new();
    // main.cpp:1456 — print.apply(model, config). Applies the resolved config +
    // default region config; objects are added separately below.
    print.apply(print_config, region_config);
    print.region_filaments_resolved = route_features && model_slots.is_some();
    // Declare one region per painted extruder (PrintApply.cpp:1062-1078 shape)
    // BEFORE add_object so they share into PrintObjectRegions::all_regions.
    if !painting_extruders.is_empty() {
        print.install_painted_regions(&painting_extruders);
    }
    print.raw_settings = raw_settings_json;
    print.config_block_settings = config_block_settings_json;
    print.set_status_callback(|percent, message| {
        info!("Progress: {}% - {}", percent, message);
    });
    print.add_object(print_object);

    // main.cpp:1486 — Print::validate() before slicing; abort on a hard error
    // (empty `.string` means valid). Warnings (is_warning) are non-fatal.
    let validation = validate_and_emit_warnings(&print);
    if !validation.string.is_empty() && !validation.is_warning {
        return Err(validation_refusal(validation));
    }

    info!("Running Print::process() pipeline...");
    let __timing = crate::probe_enabled("SLICE_PHASE_TIMING");
    let __t_proc = std::time::Instant::now();
    print
        .process(None, false)
        .with_context(|| "Failed to process print")?;
    let __proc_s = __t_proc.elapsed().as_secs_f64();

    info!("Exporting G-code...");
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("Failed to create output directory {:?}", parent))?;
    }
    let __t_exp = std::time::Instant::now();
    print
        .export_gcode(output)
        .with_context(|| format!("Failed to export G-code to {:?}", output))?;
    if __timing {
        let __exp_s = __t_exp.elapsed().as_secs_f64();
        eprintln!(
            "--- top-level (s): process {:.3} + export_gcode {:.3} = {:.3} ---",
            __proc_s,
            __exp_s,
            __proc_s + __exp_s
        );
    }

    info!("Output written to: {:?}", output);
    info!(
        "Layers: {}",
        print.objects().first().map(|o| o.layers().len()).unwrap_or(0)
    );

    Ok(())
}

/// Result of parsing a `3D/3dmodel.model` XML: the merged printable mesh plus
/// the painted-MMU per-triangle annotation (empty when the model is unpainted).
#[derive(Debug)]
pub struct Parsed3mfModel {
    pub mesh: TriangleMesh,
    /// R703 — see [`Loaded3mf::negative_mesh`].
    pub negative_mesh: TriangleMesh,
    /// Painted multi-material state per merged-mesh triangle — the pragmatic
    /// equivalent of C++ `ModelVolume::mmu_segmentation_facets` (Model.hpp:961).
    pub mmu_facets: FacetsAnnotation,
    /// Painted seam state per merged-mesh triangle (`ModelVolume::seam_facets`).
    pub seam_facets: FacetsAnnotation,
    /// R786 — per printable instantiation: (merged-vertex offset, vertex count,
    /// composed loader transform row-major 3x4). Together with the RAW vertices
    /// (below) this reconstructs native's paint frame: local = f32(raw) −
    /// f32(volume bbox center); tr = transform ∘ translate(center).
    pub paint_segments: Vec<(usize, usize, [f64; 12])>,
    /// R786 — RAW (untransformed) vertices aligned with the merged mesh's
    /// vertex indices (only printable segments are populated).
    pub paint_raw_vertices: Vec<Point3F>,
    /// R787 — the build-item (instance) matrix for the painted object; identity
    /// when absent. paint_segments transforms are the VOLUME chain only.
    pub paint_instance_matrix: [f64; 12],
    /// R805 — per negative volume: volume-local vertices, triangles, composed volume transform.
    pub negative_segments: Vec<(Vec<Point3F>, Vec<[u32; 3]>, [f64; 12])>,
}

/// Everything the Tier-1 loader extracts from a `.3mf` archive.
#[derive(Debug)]
pub struct Loaded3mf {
    /// Merged printable mesh (see [`parse_3mf_model_xml`]).
    pub mesh: TriangleMesh,
    /// `Metadata/project_settings.config` contents, when present.
    pub settings_json: Option<String>,
    /// Per-instance `identify_id`s (drive `; OBJECT_ID:` comments).
    pub identify_ids: Vec<usize>,
    /// Painted-MMU annotation, merged-mesh triangle order.
    pub mmu_facets: FacetsAnnotation,
    /// Painted-seam annotation, merged-mesh triangle order.
    pub seam_facets: FacetsAnnotation,
    /// R703 — merged NEGATIVE-volume mesh (`subtype="negative_part"` in
    /// `Metadata/model_settings.config`), transforms applied exactly as for the
    /// printable mesh. Empty when the model declares none.
    ///
    /// C++ slices these (`model_volume_needs_slicing`, `PrintObjectSlice.cpp:110`
    /// returns true for `NEGATIVE_VOLUME`) and subtracts them from every
    /// preceding non-negative region in `slices_to_regions` (`:403-421`,
    /// `diff_ex` gated on `overlap_in_xy`). Kept as a separate mesh here so the
    /// subtraction can happen per layer in 2D, which is what C++ does — a 3D
    /// mesh boolean would NOT be the faithful shape.
    pub negative_mesh: TriangleMesh,
    /// R786 — see [`Parsed3mfModel::paint_segments`].
    pub paint_segments: Vec<(usize, usize, [f64; 12])>,
    /// R786 — see [`Parsed3mfModel::paint_raw_vertices`].
    pub paint_raw_vertices: Vec<Point3F>,
    /// R787 — the build-item (instance) matrix for the painted object; identity
    /// when absent. paint_segments transforms are the VOLUME chain only.
    pub paint_instance_matrix: [f64; 12],
    /// R805 — per negative volume: volume-local vertices, triangles, composed volume transform.
    pub negative_segments: Vec<(Vec<Point3F>, Vec<[u32; 3]>, [f64; 12])>,
}

/// Load a [`TriangleMesh`] from a `.3mf` ZIP by parsing `3D/3dmodel.model`.
///
/// Also extracts `Metadata/project_settings.config` (returned as a JSON string
/// when present) and the per-instance `identify_id`s from
/// `Metadata/model_settings.config` (used for `; OBJECT_ID:` G-code comments).
///
/// All printable (`type="model"`) objects/instances are merged into one mesh
/// (build-item and component transforms are applied). Non-model objects
/// (`type="other"` — BambuStudio negative/modifier volumes) are skipped:
/// merging them as positive solids is wrong, and true boolean subtraction
/// is Tier-2 (needs ModelVolume).
pub fn load_3mf(path: &Path) -> Result<Loaded3mf> {
    Ok(load_3mf_model(path, false)?.0)
}

struct ScopedModelSettings {
    scarf: bool,
    filament: bool,
}

// bbs_3mf.cpp 2676-2701 supplies object slot one on import. An ordinary
// object with only that assignment can use the existing geometry pass.
fn has_scoped_filament_setting(xml: &str) -> bool {
    use quick_xml::{events::Event, Reader};
    let mut reader = Reader::from_str(xml);
    loop {
        match reader.read_event() {
            Ok(Event::Empty(tag) | Event::Start(tag)) if tag.name().as_ref() == b"metadata" => {
                let attr = |name: &str| tag.try_get_attribute(name).ok().flatten()
                    .and_then(|a| a.unescape_value().ok().map(|s| s.into_owned()));
                let Some(key) = attr("key") else { continue };
                if FEATURE_FILAMENT_KEYS.contains(&key.as_str()) {
                    return true;
                }
                if key == "extruder" {
                    let slot = attr("value").and_then(|value| value.trim().parse::<i32>().ok());
                    if !matches!(slot, Some(0 | 1)) {
                        return true;
                    }
                }
            }
            Ok(Event::Eof) | Err(_) => return false,
            _ => {}
        }
    }
}

fn load_3mf_model(path: &Path, inspect_settings: bool) -> Result<(Loaded3mf, ScopedModelSettings)> {
    use zip::ZipArchive;

    let file =
        fs::File::open(path).with_context(|| format!("Failed to open 3MF file: {:?}", path))?;
    let mut archive =
        ZipArchive::new(file).with_context(|| format!("Failed to read 3MF ZIP: {:?}", path))?;

    // Try to extract settings
    let settings_json = if let Ok(mut entry) = archive.by_name("Metadata/project_settings.config") {
        let mut buf = String::new();
        entry.read_to_string(&mut buf).ok();
        Some(buf)
    } else {
        None
    };

    let mut model_settings = String::new();
    if let Ok(mut entry) = archive.by_name("Metadata/model_settings.config") {
        entry.read_to_string(&mut model_settings).ok();
    }
    let identify_ids = parse_identify_ids_from_model_settings(&model_settings);
    let negative_ids = parse_negative_part_ids_from_model_settings(&model_settings);

    // Ordinary 3MFs need only the existing geometry pass. Consult the full
    // importer for potential scarf or filament overrides. Account for XML
    // character references and the importer's case-insensitive entry names.
    let may_override = |xml: &str| {
        let decoded = quick_xml::escape::unescape(xml);
        decoded.map_or(true, |text| {
            ["enable_circle_compensation", "apply_scarf_seam_on_circles", "seam_slope_type"]
                .iter().any(|key| text.contains(key))
        })
    };
    let check_scarf = inspect_settings && crate::faithful_gate("PRINT_APPLY_SCARF_SEAM");
    let check_filament = inspect_settings
        && crate::faithful_gate("SCOPED_MODEL_FILAMENT")
        && crate::faithful_gate("REGION_CONFIG_WALL_FILAMENT")
        && crate::faithful_gate("REGION_CONFIG_INFILL_FILAMENTS")
        && crate::faithful_gate("REGION_CONFIG_FILAMENT_CLAMP");
    let mut scoped = ScopedModelSettings {
        scarf: check_scarf && may_override(&model_settings),
        filament: check_filament && has_scoped_filament_setting(&model_settings),
    };
    if (check_scarf && !scoped.scarf) || (check_filament && !scoped.filament) {
        use crate::format::bbs_3mf::{BBS_MODEL_CONFIG_FILE, LAYER_CONFIG_RANGES_FILE};
        let config_entries: Vec<String> = archive.file_names().filter(|name| {
            let normalized = name.replace('\\', "/");
            (check_scarf && !scoped.scarf && normalized.eq_ignore_ascii_case(LAYER_CONFIG_RANGES_FILE))
                || (*name != BBS_MODEL_CONFIG_FILE && normalized.eq_ignore_ascii_case(BBS_MODEL_CONFIG_FILE))
        }).map(str::to_owned).collect();
        for name in config_entries {
            let mut xml = String::new();
            archive.by_name(&name)?.read_to_string(&mut xml)?;
            scoped.scarf |= check_scarf && may_override(&xml);
            if check_filament && !scoped.filament && name.replace('\\', "/").eq_ignore_ascii_case(BBS_MODEL_CONFIG_FILE) {
                scoped.filament = has_scoped_filament_setting(&xml);
            }
        }
    }

    // Read the 3D model XML
    let model_xml = {
        let mut entry = archive
            .by_name("3D/3dmodel.model")
            .with_context(|| "3MF file missing 3D/3dmodel.model")?;
        let mut buf = String::new();
        entry
            .read_to_string(&mut buf)
            .with_context(|| "Failed to read 3D/3dmodel.model")?;
        buf
    };

    // Parse vertices, triangles, and painted-MMU state from the XML
    let parsed = parse_3mf_model_xml_with_negatives(&model_xml, &negative_ids)?;
    Ok((Loaded3mf {
        mesh: parsed.mesh,
        settings_json,
        identify_ids,
        mmu_facets: parsed.mmu_facets,
        seam_facets: parsed.seam_facets,
        negative_mesh: parsed.negative_mesh,
        paint_segments: parsed.paint_segments,
        paint_raw_vertices: parsed.paint_raw_vertices,
        paint_instance_matrix: parsed.paint_instance_matrix,
        negative_segments: parsed.negative_segments,
    }, scoped))
}

/// R703 — the `<object id>`s that `Metadata/model_settings.config` marks
/// `subtype="negative_part"`.
///
/// BambuStudio writes one `<part id="N" subtype="...">` per volume, and that id
/// is the same one `3D/3dmodel.model` uses for `<object id="N">` — verified on
/// Majora, whose model declares 1 `normal_part` (object 1) and 6 `negative_part`
/// (objects 2-7), with object 8 the container referencing components 1..7.
///
/// Only `negative_part` is returned. `modifier_part` and anything unrecognised
/// stay excluded, matching the existing conservative behaviour: a modifier
/// merged as positive solid would be worse than omitting it, and modifiers need
/// the Tier-2 ModelVolume work.
pub fn parse_negative_part_ids_from_model_settings(xml: &str) -> std::collections::HashSet<u32> {
    use quick_xml::events::Event;
    use quick_xml::Reader;

    let mut out = std::collections::HashSet::new();
    let mut reader = Reader::from_str(xml);
    reader.trim_text(true);
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                if e.name().as_ref() == b"part" {
                    let mut id: Option<u32> = None;
                    let mut subtype = String::new();
                    for attr in e.attributes().flatten() {
                        match attr.key.as_ref() {
                            b"id" => {
                                id = String::from_utf8_lossy(&attr.value).parse::<u32>().ok();
                            }
                            b"subtype" => {
                                subtype = String::from_utf8_lossy(&attr.value).to_string();
                            }
                            _ => {}
                        }
                    }
                    if subtype == "negative_part" {
                        if let Some(i) = id {
                            out.insert(i);
                        }
                    }
                }
            }
            Ok(Event::Eof) => break,
            Err(_) => break,
            _ => {}
        }
        buf.clear();
    }
    out
}

/// Parse identify_ids from Metadata/model_settings.config XML.
/// Returns a list of identify_ids in the order they appear in the plate's
/// model_instance elements.
///
/// C++ reference: _BBS_3MF_Importer::_handle_end_model_instance() line 4666
/// sets obj_inst_map[object_id] = (instance_id, identify_id)
fn parse_identify_ids_from_model_settings(xml: &str) -> Vec<usize> {
    use quick_xml::events::Event;
    use quick_xml::Reader;

    let mut ids: Vec<usize> = Vec::new();
    let mut reader = Reader::from_str(xml);
    reader.trim_text(true);

    // Track current model_instance state
    let mut in_model_instance = false;
    let mut current_identify_id: Option<usize> = None;

    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let name_bytes = e.name().as_ref().to_vec();
                let name = std::str::from_utf8(&name_bytes).unwrap_or("").to_string();
                if name == "model_instance" {
                    in_model_instance = true;
                    current_identify_id = None;
                }
                if in_model_instance && name == "metadata" {
                    let mut key = String::new();
                    let mut value = String::new();
                    for attr in e.attributes().flatten() {
                        let k = std::str::from_utf8(attr.key.as_ref())
                            .unwrap_or("")
                            .to_string();
                        let v = std::str::from_utf8(&attr.value).unwrap_or("").to_string();
                        match k.as_str() {
                            "key" => key = v,
                            "value" => value = v,
                            _ => {}
                        }
                    }
                    if key == "identify_id" {
                        if let Ok(id) = value.parse::<usize>() {
                            current_identify_id = Some(id);
                        }
                    }
                }
            }
            Ok(Event::Empty(ref e)) => {
                let name_bytes = e.name().as_ref().to_vec();
                let name = std::str::from_utf8(&name_bytes).unwrap_or("").to_string();
                if in_model_instance && name == "metadata" {
                    let mut key = String::new();
                    let mut value = String::new();
                    for attr in e.attributes().flatten() {
                        let k = std::str::from_utf8(attr.key.as_ref())
                            .unwrap_or("")
                            .to_string();
                        let v = std::str::from_utf8(&attr.value).unwrap_or("").to_string();
                        match k.as_str() {
                            "key" => key = v,
                            "value" => value = v,
                            _ => {}
                        }
                    }
                    if key == "identify_id" {
                        if let Ok(id) = value.parse::<usize>() {
                            current_identify_id = Some(id);
                        }
                    }
                }
            }
            Ok(Event::End(ref e)) => {
                let name_bytes = e.name().as_ref().to_vec();
                let name = std::str::from_utf8(&name_bytes).unwrap_or("").to_string();
                if name == "model_instance" {
                    if let Some(id) = current_identify_id.take() {
                        ids.push(id);
                    }
                    in_model_instance = false;
                }
            }
            Ok(Event::Eof) => break,
            Err(_) => break,
            _ => {}
        }
        buf.clear();
    }
    ids
}

/// Parse a 3MF 3dmodel.model XML string to extract vertices and triangles.
///
/// The XML format is:
///   <mesh>
///     <vertices>
///       <vertex x="..." y="..." z="..." />
///     </vertices>
///     <triangles>
///       <triangle v1="..." v2="..." v3="..." />
///     </triangles>
///   </mesh>
///
/// Tier-1 scope: only meshes stored inline in `3D/3dmodel.model` are read.
/// A 3MF that stores its mesh in external `/3D/Objects/*.model` parts (the 3MF
/// production extension, referenced via `<component p:path=...>`) yields no
/// inline mesh and returns a descriptive error.
///
/// Painted-MMU per-triangle attributes (`paint_color` /
/// `slic3rpe:mmu_segmentation`) are captured into
/// [`Parsed3mfModel::mmu_facets`], indexed by merged-mesh triangle order.
pub fn parse_3mf_model_xml(xml: &str) -> Result<Parsed3mfModel> {
    parse_3mf_model_xml_with_negatives(xml, &std::collections::HashSet::new())
}

/// R703 — as [`parse_3mf_model_xml`], but `negative_ids` names the `<object id>`s
/// that `Metadata/model_settings.config` marks `subtype="negative_part"`. Those
/// are instantiated into a SEPARATE mesh (same transforms) instead of being
/// dropped, so the slicer can subtract them per layer the way C++ does.
///
/// In BambuStudio 3MFs the model_settings `<part id="N">` and the 3dmodel
/// `<object id="N">` share the same id space (verified on Majora: object 1 =
/// `normal_part`, objects 2-7 = the six `negative_part`s, object 8 = the
/// container whose components are 1..7), so the set is used directly.
pub fn parse_3mf_model_xml_with_negatives(
    xml: &str,
    negative_ids: &std::collections::HashSet<u32>,
) -> Result<Parsed3mfModel> {
    use quick_xml::events::Event;
    use quick_xml::Reader;

    // 3MF structure: <resources> has <object> elements with meshes,
    // <build> has <item> elements with transforms referencing objects.
    // Objects can also have <components> that reference other objects.

    // Collect all objects (meshes) by ID
    struct MeshData {
        vertices: Vec<Point3F>,
        triangles: Vec<[u32; 3]>,
        /// Per-triangle painted-MMU string (`paint_color` in BambuStudio 3MFs,
        /// `slic3rpe:mmu_segmentation` in PrusaSlicer ones), parallel to
        /// `triangles`. `None` = unpainted (base filament).
        paints: Vec<Option<String>>,
        /// Per-triangle painted-seam string (`paint_seam`, bbs_3mf.cpp:296),
        /// parallel to `triangles`.
        seam_paints: Vec<Option<String>>,
        components: Vec<(u32, [f64; 12])>, // (objectid, transform)
        /// `<object type="...">` — defaults to "model" per the 3MF core spec.
        /// BambuStudio stores negative/modifier volumes as `type="other"`
        /// objects referenced via `<component>` (their real role lives in
        /// Metadata/model_settings.config `subtype`, e.g. `negative_part`).
        /// Tier-1 merges only printable `model` geometry and SKIPS the rest —
        /// unioning a negative part as positive solid is worse than omitting
        /// it, and true boolean subtraction needs the Tier-2 ModelVolume work.
        is_model: bool,
    }

    let mut objects: std::collections::HashMap<u32, MeshData> = std::collections::HashMap::new();
    // `has_external_components` is set when a <component> carries a `p:path`
    // attribute, i.e. the mesh lives in an external `/3D/Objects/*.model` part
    // (the 3MF production extension). The Tier-1 reader only parses the single
    // `3D/3dmodel.model`, so such a 3MF yields no inline mesh — we detect it to
    // emit a clear error instead of a cryptic "0 vertices".
    let mut has_external_components = false;
    let mut build_items: Vec<(u32, [f64; 12])> = Vec::new(); // (objectid, transform)

    let identity: [f64; 12] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];

    fn parse_transform(s: &str) -> [f64; 12] {
        let vals: Vec<f64> = s
            .split_whitespace()
            .filter_map(|v| v.parse::<f64>().ok())
            .collect();
        if vals.len() >= 12 {
            // 3MF stores the affine transform COLUMN-major: the 12 values are
            // 4 columns of 3 rows (3 basis-vector columns + a translation
            // column). See BambuStudio's get_transform_from_3mf_specs_string
            // (Format/3mf.cpp:224-231), which fills a 4x3 and transposes.
            // `apply_transform`/`compose_transforms` below use a row-major 3x3
            // plus translation, so transpose the rotation block here. (A pure
            // translation or identity is unchanged by this, which is why
            // STL-sourced / benchy 3MFs sliced correctly before this fix; a
            // real rotation like Majora's Mask did not.)
            [
                vals[0], vals[3], vals[6], // row 0
                vals[1], vals[4], vals[7], // row 1
                vals[2], vals[5], vals[8], // row 2
                vals[9], vals[10], vals[11], // translation
            ]
        } else {
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        }
    }

    fn apply_transform(p: &Point3F, t: &[f64; 12]) -> Point3F {
        Point3F {
            x: t[0] * p.x + t[1] * p.y + t[2] * p.z + t[9],
            y: t[3] * p.x + t[4] * p.y + t[5] * p.z + t[10],
            z: t[6] * p.x + t[7] * p.y + t[8] * p.z + t[11],
        }
    }

    fn compose_transforms(a: &[f64; 12], b: &[f64; 12]) -> [f64; 12] {
        // a applied first, then b: result = b * a
        [
            b[0] * a[0] + b[1] * a[3] + b[2] * a[6],
            b[0] * a[1] + b[1] * a[4] + b[2] * a[7],
            b[0] * a[2] + b[1] * a[5] + b[2] * a[8],
            b[3] * a[0] + b[4] * a[3] + b[5] * a[6],
            b[3] * a[1] + b[4] * a[4] + b[5] * a[7],
            b[3] * a[2] + b[4] * a[5] + b[5] * a[8],
            b[6] * a[0] + b[7] * a[3] + b[8] * a[6],
            b[6] * a[1] + b[7] * a[4] + b[8] * a[7],
            b[6] * a[2] + b[7] * a[5] + b[8] * a[8],
            b[0] * a[9] + b[1] * a[10] + b[2] * a[11] + b[9],
            b[3] * a[9] + b[4] * a[10] + b[5] * a[11] + b[10],
            b[6] * a[9] + b[7] * a[10] + b[8] * a[11] + b[11],
        ]
    }

    let mut reader = Reader::from_str(xml);
    reader.trim_text(true);
    let mut buf = Vec::new();
    let mut current_object_id: Option<u32> = None;
    let mut in_mesh = false;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                let local_name = e.local_name();
                match local_name.as_ref() {
                    b"object" => {
                        let mut id = 0u32;
                        // 3MF core spec: the `type` attribute defaults to "model".
                        let mut is_model = true;
                        for attr in e.attributes().flatten() {
                            match attr.key.local_name().as_ref() {
                                b"id" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        id = s.parse().unwrap_or(0);
                                    }
                                }
                                b"type" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        is_model = s == "model";
                                    }
                                }
                                _ => {}
                            }
                        }
                        current_object_id = Some(id);
                        let entry = objects.entry(id).or_insert_with(|| MeshData {
                            vertices: Vec::new(),
                            triangles: Vec::new(),
                            paints: Vec::new(),
                            seam_paints: Vec::new(),
                            components: Vec::new(),
                            is_model,
                        });
                        entry.is_model = is_model;
                    }
                    b"mesh" => {
                        in_mesh = true;
                    }
                    b"vertex" if in_mesh => {
                        let mut x = 0.0f64;
                        let mut y = 0.0f64;
                        let mut z = 0.0f64;
                        for attr in e.attributes().flatten() {
                            match attr.key.local_name().as_ref() {
                                b"x" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        x = s.parse().unwrap_or(0.0);
                                    }
                                }
                                b"y" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        y = s.parse().unwrap_or(0.0);
                                    }
                                }
                                b"z" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        z = s.parse().unwrap_or(0.0);
                                    }
                                }
                                _ => {}
                            }
                        }
                        if let Some(id) = current_object_id {
                            objects
                                .entry(id)
                                .and_modify(|m| m.vertices.push(Point3F { x, y, z }));
                        }
                    }
                    b"triangle" if in_mesh => {
                        let mut v1 = 0u32;
                        let mut v2 = 0u32;
                        let mut v3 = 0u32;
                        let mut paint: Option<String> = None;
                        let mut seam_paint: Option<String> = None;
                        for attr in e.attributes().flatten() {
                            match attr.key.local_name().as_ref() {
                                b"v1" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        v1 = s.parse().unwrap_or(0);
                                    }
                                }
                                b"v2" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        v2 = s.parse().unwrap_or(0);
                                    }
                                }
                                b"v3" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        v3 = s.parse().unwrap_or(0);
                                    }
                                }
                                // Painted-MMU per-triangle state: BambuStudio
                                // writes `paint_color` (bbs_3mf.cpp:297),
                                // PrusaSlicer `slic3rpe:mmu_segmentation`
                                // (local name `mmu_segmentation`). Same hex
                                // FacetsAnnotation encoding either way.
                                b"paint_seam" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        if !s.is_empty() {
                                            seam_paint = Some(s.to_owned());
                                        }
                                    }
                                }
                                b"paint_color" | b"mmu_segmentation" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        if !s.is_empty() {
                                            paint = Some(s.to_owned());
                                        }
                                    }
                                }
                                _ => {}
                            }
                        }
                        if let Some(id) = current_object_id {
                            objects.entry(id).and_modify(|m| {
                                m.triangles.push([v1, v2, v3]);
                                m.paints.push(paint);
                                m.seam_paints.push(seam_paint);
                            });
                        }
                    }
                    b"component" => {
                        let mut obj_id = 0u32;
                        let mut transform = identity;
                        for attr in e.attributes().flatten() {
                            match attr.key.local_name().as_ref() {
                                b"objectid" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        obj_id = s.parse().unwrap_or(0);
                                    }
                                }
                                b"transform" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        transform = parse_transform(s);
                                    }
                                }
                                // `p:path` → the referenced object is in an external
                                // model part (production extension), not this XML.
                                b"path" => {
                                    has_external_components = true;
                                }
                                _ => {}
                            }
                        }
                        if let Some(parent_id) = current_object_id {
                            objects
                                .entry(parent_id)
                                .and_modify(|m| m.components.push((obj_id, transform)));
                        }
                    }
                    b"item" => {
                        let mut obj_id = 0u32;
                        let mut transform = identity;
                        for attr in e.attributes().flatten() {
                            match attr.key.local_name().as_ref() {
                                b"objectid" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        obj_id = s.parse().unwrap_or(0);
                                    }
                                }
                                b"transform" => {
                                    if let Ok(s) = std::str::from_utf8(&attr.value) {
                                        transform = parse_transform(s);
                                    }
                                }
                                _ => {}
                            }
                        }
                        build_items.push((obj_id, transform));
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) => match e.local_name().as_ref() {
                b"mesh" => {
                    in_mesh = false;
                }
                b"object" => {
                    current_object_id = None;
                }
                _ => {}
            },
            Ok(Event::Eof) => break,
            Err(e) => return Err(anyhow::anyhow!("Error parsing 3MF XML: {}", e)),
            _ => {}
        }
        buf.clear();
    }

    // Resolve build items → instantiate objects with transforms
    let mut all_vertices: Vec<Point3F> = Vec::new();
    let mut all_triangles: Vec<Triangle> = Vec::new();
    // Painted-MMU strings parallel to `all_triangles` (merged-mesh order).
    let mut all_paints: Vec<Option<String>> = Vec::new();
    // Painted-seam strings, parallel to `all_triangles`. They ride in the same
    // order as `all_paints`, so they are collected next to it.
    let mut all_seam_paints: Vec<Option<String>> = Vec::new();
    // R703 — negative-volume geometry, kept apart from the printable mesh.
    let mut neg_vertices: Vec<Point3F> = Vec::new();
    let mut neg_triangles: Vec<Triangle> = Vec::new();
    // R786 — paint-frame reconstruction data (see Parsed3mfModel fields).
    let mut paint_segments: Vec<(usize, usize, [f64; 12])> = Vec::new();
    let mut paint_raw_vertices: Vec<Point3F> = Vec::new();
    let mut paint_instance: Option<[f64; 12]> = None;
    let mut negative_segments: Vec<(Vec<Point3F>, Vec<[u32; 3]>, [f64; 12])> = Vec::new();

    #[allow(clippy::too_many_arguments)]
    fn instantiate_object(
        obj_id: u32,
        transform: &[f64; 12],
        objects: &std::collections::HashMap<u32, MeshData>,
        all_vertices: &mut Vec<Point3F>,
        all_triangles: &mut Vec<Triangle>,
        all_paints: &mut Vec<Option<String>>,
        all_seam_paints: &mut Vec<Option<String>>,
        identity: &[f64; 12],
        negative_ids: &std::collections::HashSet<u32>,
        neg_vertices: &mut Vec<Point3F>,
        neg_triangles: &mut Vec<Triangle>,
        paint_segments: &mut Vec<(usize, usize, [f64; 12])>,
        paint_raw_vertices: &mut Vec<Point3F>,
        build_tf: &[f64; 12],
        vol_tf: &[f64; 12],
        paint_instance: &mut Option<[f64; 12]>,
        negative_segments: &mut Vec<(Vec<Point3F>, Vec<[u32; 3]>, [f64; 12])>,
    ) {
        if let Some(mesh_data) = objects.get(&obj_id) {
            // Non-printable objects (`type != "model"`, e.g. `type="other"`
            // negative/modifier volumes). See MeshData::is_model.
            //
            // R703 — one that model_settings marks `negative_part` is collected
            // into the negative mesh (same transform) rather than dropped; a
            // modifier (or anything unlabelled) is still skipped, since merging
            // it as positive solid would be wrong and modifiers need the Tier-2
            // ModelVolume work.
            if !mesh_data.is_model {
                if negative_ids.contains(&obj_id) && !mesh_data.vertices.is_empty() {
                    let v_offset = neg_vertices.len() as u32;
                    for v in &mesh_data.vertices {
                        neg_vertices.push(apply_transform(v, transform));
                    }
                    for tri in mesh_data.triangles.iter() {
                        neg_triangles.push(Triangle::new(
                            tri[0] + v_offset,
                            tri[1] + v_offset,
                            tri[2] + v_offset,
                        ));
                    }
                    // R805 — keep the volume-LOCAL vertices + composed volume
                    // transform so the negative can be sliced through the exact
                    // f32 frame (NEGVOL_EXACT_FRAME), like paint_segments.
                    negative_segments.push((mesh_data.vertices.clone(), mesh_data.triangles.clone(), *vol_tf));
                }
                return;
            }
            // If object has a mesh, instantiate it
            if !mesh_data.vertices.is_empty() {
                let v_offset = all_vertices.len() as u32;
                // R786/R787 — record the raw verts + the VOLUME-chain transform
                // (component matrices only; native's ModelVolume::get_matrix()),
                // and the build-item transform once (ModelInstance::get_matrix()).
                paint_segments.push((v_offset as usize, mesh_data.vertices.len(), *vol_tf));
                if paint_instance.is_none() {
                    *paint_instance = Some(*build_tf);
                }
                for v in &mesh_data.vertices {
                    all_vertices.push(apply_transform(v, transform));
                    paint_raw_vertices.push(*v);
                }
                for (i, tri) in mesh_data.triangles.iter().enumerate() {
                    all_triangles.push(Triangle::new(
                        tri[0] + v_offset,
                        tri[1] + v_offset,
                        tri[2] + v_offset,
                    ));
                    all_paints.push(mesh_data.paints.get(i).cloned().flatten());
                    all_seam_paints.push(mesh_data.seam_paints.get(i).cloned().flatten());
                }
            }
            // If object has components, recurse with composed transforms
            for &(comp_id, ref comp_transform) in &mesh_data.components {
                let composed = compose_transforms(comp_transform, transform);
                let vol_composed = compose_transforms(comp_transform, vol_tf);
                instantiate_object(
                    comp_id,
                    &composed,
                    objects,
                    all_vertices,
                    all_triangles,
                    all_paints,
                    all_seam_paints,
                    identity,
                    negative_ids,
                    neg_vertices,
                    neg_triangles,
                    paint_segments,
                    paint_raw_vertices,
                    build_tf,
                    &vol_composed,
                    paint_instance,
                    negative_segments,
                );
            }
        }
    }

    if build_items.is_empty() {
        // Fallback: no <build> section, just collect all printable meshes
        for (_, mesh_data) in &objects {
            if mesh_data.is_model && !mesh_data.vertices.is_empty() {
                let v_offset = all_vertices.len() as u32;
                paint_segments.push((v_offset as usize, mesh_data.vertices.len(), identity));
                paint_raw_vertices.extend_from_slice(&mesh_data.vertices);
                all_vertices.extend_from_slice(&mesh_data.vertices);
                for (i, tri) in mesh_data.triangles.iter().enumerate() {
                    all_triangles.push(Triangle::new(
                        tri[0] + v_offset,
                        tri[1] + v_offset,
                        tri[2] + v_offset,
                    ));
                    all_paints.push(mesh_data.paints.get(i).cloned().flatten());
                    all_seam_paints.push(mesh_data.seam_paints.get(i).cloned().flatten());
                }
            }
        }
    } else {
        for &(obj_id, ref transform) in &build_items {
            instantiate_object(
                obj_id,
                transform,
                &objects,
                &mut all_vertices,
                &mut all_triangles,
                &mut all_paints,
                &mut all_seam_paints,
                &identity,
                negative_ids,
                &mut neg_vertices,
                &mut neg_triangles,
                &mut paint_segments,
                &mut paint_raw_vertices,
                transform,
                &identity,
                &mut paint_instance,
                &mut negative_segments,
            );
        }
    }

    if all_vertices.is_empty() || all_triangles.is_empty() {
        if has_external_components {
            return Err(anyhow::anyhow!(
                "3MF uses the production extension (mesh stored in external \
                 /3D/Objects/*.model parts). The Tier-1 Rust 3MF reader only \
                 supports single-file 3MFs (mesh inline in 3D/3dmodel.model); \
                 slice this one with --engine native."
            ));
        }
        return Err(anyhow::anyhow!(
            "No mesh data found in 3MF XML ({} vertices, {} triangles)",
            all_vertices.len(),
            all_triangles.len()
        ));
    }

    // Fold the per-triangle paint strings (merged-mesh triangle order) into a
    // FacetsAnnotation — the same storage `ModelVolume::mmu_segmentation_facets`
    // uses in C++ (Model.hpp:961). Consumed by the Tier-2 multi-material
    // segmentation; carried here so the painting survives the pragmatic load.
    let mut mmu_facets = FacetsAnnotation::default();
    for (idx, paint) in all_paints.iter().enumerate() {
        if let Some(s) = paint {
            mmu_facets.set_triangle_from_string(idx as i32, s);
        }
    }

    // `ModelVolume::seam_facets` (Model.hpp), in the same storage.
    let mut seam_facets = FacetsAnnotation::default();
    for (idx, paint) in all_seam_paints.iter().enumerate() {
        if let Some(s) = paint {
            seam_facets.set_triangle_from_string(idx as i32, s);
        }
    }

    info!(
        "Parsed 3MF: {} vertices, {} triangles ({} painted; from {} objects, {} build items)",
        all_vertices.len(),
        all_triangles.len(),
        mmu_facets.facet_count(),
        objects.len(),
        build_items.len()
    );
    if !neg_triangles.is_empty() {
        info!(
            "Parsed 3MF negative volumes: {} vertices, {} triangles ({} objects)",
            neg_vertices.len(),
            neg_triangles.len(),
            negative_ids.len(),
        );
    }
    let paint_instance_matrix = paint_instance.unwrap_or(identity);
    Ok(Parsed3mfModel {
        paint_segments,
        paint_raw_vertices,
        paint_instance_matrix,
        negative_segments,
        mesh: TriangleMesh::from_parts(all_vertices, all_triangles),
        negative_mesh: TriangleMesh::from_parts(neg_vertices, neg_triangles),
        mmu_facets,
        seam_facets,
    })
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
    // PrintConfig.cpp:4680-4687,6090-6100. An unset profile slot inherits
    // the model filament when Print::apply builds the effective region.
    if crate::faithful_gate("REGION_CONFIG_INFILL_FILAMENTS") {
        region_config.sparse_infill_filament = 0;
        region_config.solid_infill_filament = 0;
    }

    // Iterate all keys from the JSON and dispatch to config structs.
    let mut recognized = 0u32;
    let mut unrecognized = 0u32;
    if let Some(obj) = json.as_object() {
        for (key, value) in obj {
            let str_val = match value {
                serde_json::Value::String(s) => s.clone(),
                // Config.cpp:1030-1080 joins the profile's array before deserialization.
                serde_json::Value::Array(arr) if crate::print_config::profile_machine_limit_rows_enabled(key)
                    || crate::print_config::processor_filament_material_rows_enabled(key)
                    || crate::print_config::processor_bed_area_enabled(key)
                    || (matches!(key.as_str(), "physical_extruder_map" | "filament_map") && crate::faithful_gate("PROCESSOR_TEMPERATURE_TARGETS"))
                    || (matches!(key.as_str(), "extruder_offset" | "filament_map") && crate::faithful_gate("PROCESSOR_FILAMENT_OFFSETS")) => {
                    let Some(items) = arr.iter().map(|v| v.as_str()).collect::<Option<Vec<_>>>() else { continue };
                    items.join(",")
                }
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

    // === Per-filament arrays (multi-material) ===
    // `set_deserialize` consumes only element 0 of array-valued keys (the
    // scalar config is filament 0); capture the FULL arrays here so the
    // multi-material chain knows the real filament count/colours.
    apply_filament_arrays(&mut print_config, json);

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

/// Capture the full per-filament arrays (`filament_colour`,
/// `filament_diameter`, `filament_density`) from the raw settings JSON.
///
/// The scalar `filament_*` fields keep element 0 (set by `set_deserialize`);
/// these vectors carry all filaments so `PrintConfig::num_filaments()` and the
/// multi-material chain (painted regions → tool ordering) see the real count.
fn apply_filament_arrays(config: &mut PrintConfig, json: &serde_json::Value) {
    let get_str_array = |key: &str| -> Vec<String> {
        match json.get(key) {
            Some(serde_json::Value::Array(arr)) => arr
                .iter()
                .filter_map(|v| v.as_str().map(|s| s.to_owned()))
                .collect(),
            _ => Vec::new(),
        }
    };
    let get_f64_array = |key: &str| -> Vec<f64> {
        match json.get(key) {
            Some(serde_json::Value::Array(arr)) => arr
                .iter()
                .filter_map(|v| v.as_str().and_then(|s| s.parse::<f64>().ok()))
                .collect(),
            _ => Vec::new(),
        }
    };

    config.filament_colours = get_str_array("filament_colour");
    config.filament_diameters = get_f64_array("filament_diameter");
    config.filament_densities = get_f64_array("filament_density");
    // PrintConfig.cpp:2385 (coInts). Read as f64 then truncated, because the 3MF
    // stores every value as a JSON string ("100") and get_f64_array is the
    // existing reader for that shape. R617.
    config.filament_adhesiveness_categories = get_f64_array("filament_adhesiveness_category")
        .into_iter()
        .map(|v| v as i32)
        .collect();
    // Flattened NxN inter-filament flush volumes (row-major, matrix[old*N+new]),
    // consumed by the psWipeTower phase for per-tool-change purge volumes.
    config.flush_volumes_matrix = get_f64_array("flush_volumes_matrix");
    // Per-filament prime volumes — drive the wipe-tower reserved depth.
    config.filament_prime_volumes = get_f64_array("filament_prime_volume");
    config.filament_prime_volumes_nc = get_f64_array("filament_prime_volume_nc");
    // `filament_map` (coInts, 1-based) and `nozzle_volume` (per extruder), read by
    // the G-code processor (GCodeProcessor.cpp:1975-1977, 2074-2078).
    // Either a JSON array or one comma-separated string.
    let get_f64_list = |key: &str| -> Vec<f64> {
        match json.get(key) {
            Some(serde_json::Value::String(s)) => {
                s.split(',').filter_map(|v| v.trim().parse::<f64>().ok()).collect()
            }
            _ => get_f64_array(key),
        }
    };
    if !crate::faithful_gate("PROCESSOR_FILAMENT_OFFSETS") && !crate::faithful_gate("PROCESSOR_TEMPERATURE_TARGETS") {
        config.filament_map = get_f64_list("filament_map").into_iter().map(|v| v as i32).collect();
    }
    config.nozzle_volumes = get_f64_list("nozzle_volume");
    // PrintConfig.hpp:1747-1758: the twelve ConfigOptionFloats of the auto circle compensation take
    // the whole JSON array, joined with ',' and deserialized as ConfigBase::load_from_json does for an
    // array value (Config.cpp:1030-1080, parse_str_arr + set_deserialize); the loop above handed
    // `set_deserialize` only element 0.
    for key in crate::print_config::CIRCLE_COMPENSATION_FLOATS_KEYS {
        if let Some(serde_json::Value::Array(arr)) = json.get(key) {
            let items: Vec<&str> = arr.iter().filter_map(|v| v.as_str()).collect();
            if items.len() == arr.len() {
                config.set_deserialize(key, &items.join(","));
            }
        }
    }
    // PrintConfig.hpp:1745 `filament_velocity_adaptation_factor` (ConfigOptionFloats, one value per
    // filament; GCode.cpp:6779/6809): the whole JSON array, as the keys above (Config.cpp:1030-1080).
    if let Some(serde_json::Value::Array(arr)) = json.get("filament_velocity_adaptation_factor") {
        let items: Vec<&str> = arr.iter().filter_map(|v| v.as_str()).collect();
        if items.len() == arr.len() {
            config.set_deserialize("filament_velocity_adaptation_factor", &items.join(","));
        }
    }
    // PrintConfig.hpp:1536 `printer_extruder_id` (coInts) and 1519 `extruder_type` (coEnums): the
    // whole JSON array joined with ',' (Config.cpp:1030-1080); 1538 `printer_extruder_variant`
    // (coStrings): each item quoted with escape_string_cstyle and joined with ';' (Config.cpp:922-934,
    // 1061-1063). GCodeProcessor::apply_config copies all three into its result
    // (GCodeProcessor.cpp:2038-2040).
    for key in ["printer_extruder_id", "extruder_type"] {
        if let Some(serde_json::Value::Array(arr)) = json.get(key) {
            let items: Vec<&str> = arr.iter().filter_map(|v| v.as_str()).collect();
            if items.len() == arr.len() {
                config.set_deserialize(key, &items.join(","));
            }
        }
    }
    if let Some(serde_json::Value::Array(arr)) = json.get("printer_extruder_variant") {
        let items: Vec<String> = arr
            .iter()
            .filter_map(|v| v.as_str())
            .map(|s| format!("\"{}\"", crate::config::escape_string_cstyle(s)))
            .collect();
        if items.len() == arr.len() {
            config.set_deserialize("printer_extruder_variant", &items.join(";"));
        }
    }
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
    let Some(points_json) = json.get("printable_area").and_then(|v| v.as_array()) else { return };
    let mut legacy_points = Vec::new();
    let points = if crate::faithful_gate("PROCESSOR_PRINTABLE_GEOMETRY") {
        // The profile loader already parsed this option as ConfigOptionPoints.
        &config.printable_area
    } else {
        for value in points_json {
            if let Some((x, y)) = value.as_str().and_then(|text| text.split_once('x')) {
                if let (Ok(x), Ok(y)) = (x.parse(), y.parse()) {
                    legacy_points.push((x, y));
                }
            }
        }
        &legacy_points
    };
    let (mut size_x, mut size_y) = (256.0_f64, 256.0_f64);
    for &(x, y) in points {
        size_x = size_x.max(x);
        size_y = size_y.max(y);
    }
    config.bed_size_x = size_x;
    config.bed_size_y = size_y;
    if legacy_points.len() >= 3 {
        config.printable_area = legacy_points;
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

/// The settings the CONFIG_BLOCK writes: the pin's block is `print.full_print_config()`
/// (GCode.cpp:2686-2720), the job's settings as loaded, where the machine keys keep the machine's
/// values in its variant layout and the filament overrides stay in their `filament_*` keys. The port's
/// `patch_filament_overrides_in_json` rewrites `retraction_length`, `z_hop_types`, ... in the
/// settings the placeholder parser and the engine read; this keeps the unpatched copy for the
/// block. Gate CONFIG_BLOCK_UNPATCHED_SETTINGS (faithful; `=0` writes the patched settings).
fn config_block_settings_before_patch(settings: &serde_json::Value) -> Option<serde_json::Value> {
    if crate::faithful_gate("CONFIG_BLOCK_UNPATCHED_SETTINGS") {
        Some(settings.clone())
    } else {
        None
    }
}

/// The values the pin's engine reads: slicer_cli's front end pads, clamps and aligns its config
/// before `Print::apply` (main.cpp:9807-9935 pad, 9944-9950 clamp, 11128 align), and
/// `Print::apply` reduces the variant values to the printer's extruders before the print's
/// configs are set from them (PrintApply.cpp:1437-1453 printer/print options, 1455-1483 the
/// filament options). The same steps as `GCode::config_block_pin` (g_code.rs, the verified
/// helpers `front_end_*` / `print_apply_*`), applied to the settings the Rust engine reads; a key
/// whose value a step changed is written back in the settings' own form (an array of the
/// elements for a vector option, a string otherwise). One faithful gate per step:
/// ENGINE_FRONT_END_PAD_PIN, ENGINE_FRONT_END_CLAMP_PIN, ENGINE_FRONT_END_ALIGN_PIN,
/// ENGINE_PRINT_APPLY_EXTRUDERS_PIN, ENGINE_PRINT_APPLY_FILAMENTS_PIN (the last needs the one
/// before it, as PrintApply.cpp:1455 follows 1453).
///
/// FIDELITY-NOTE: `front_end_set_default_temps` (main.cpp:1355-1366) is not applied: it writes
/// removed options the pin's engine has no definition for. Runs before
/// `patch_filament_overrides_in_json`, whose overrides the pin applies on the reduced config.
fn engine_settings_as_the_pin_reads(settings: &mut serde_json::Value) {
    use crate::config::{ConfigBase, ConfigOptionResolver};
    use crate::g_code::{
        front_end_align_per_filament_config_vectors, front_end_clamp_master_extruder_id,
        front_end_pad_vectors_to_extruder_count, front_end_project_filament_count,
        full_print_config_from_settings, print_apply_update_filament_values_to_printer_extruders,
        print_apply_update_values_to_printer_extruders,
    };
    let pad = crate::faithful_gate("ENGINE_FRONT_END_PAD_PIN");
    let clamp = crate::faithful_gate("ENGINE_FRONT_END_CLAMP_PIN");
    let align = crate::faithful_gate("ENGINE_FRONT_END_ALIGN_PIN");
    let extruders = crate::faithful_gate("ENGINE_PRINT_APPLY_EXTRUDERS_PIN");
    let filaments = extruders && crate::faithful_gate("ENGINE_PRINT_APPLY_FILAMENTS_PIN");
    if !(pad || clamp || align || extruders) {
        return;
    }
    let loaded = full_print_config_from_settings(settings);
    let mut cfg = loaded.clone();
    // main.cpp:9678: the roster, read before the padding.
    let project_filaments = front_end_project_filament_count(&cfg.base);
    if pad {
        front_end_pad_vectors_to_extruder_count(&mut cfg.base);
    }
    if clamp {
        front_end_clamp_master_extruder_id(&mut cfg.base);
    }
    if align {
        front_end_align_per_filament_config_vectors(&mut cfg.base, project_filaments);
    }
    if extruders {
        print_apply_update_values_to_printer_extruders(&mut cfg);
        // PrintApply.cpp:1453 `m_ori_full_print_config = new_full_config;`
        let ori = cfg.clone();
        if filaments {
            print_apply_update_filament_values_to_printer_extruders(&mut cfg, &ori);
        }
    }
    let Some(map) = settings.as_object_mut() else {
        return;
    };
    for key in ConfigBase::keys(&cfg) {
        let after = ConfigBase::opt_serialize(&cfg, &key).ok();
        if after.is_none() || after == ConfigBase::opt_serialize(&loaded, &key).ok() {
            continue;
        }
        let Some(opt) = ConfigOptionResolver::optptr(&cfg, &key) else {
            continue;
        };
        let was_string = matches!(map.get(&key), Some(serde_json::Value::String(_)));
        let elems: Option<Vec<String>> = opt.as_vector_base().and_then(|v| v.vserialize().ok());
        let value = match elems {
            Some(elems) if !was_string => {
                serde_json::Value::Array(elems.into_iter().map(serde_json::Value::String).collect())
            }
            _ => serde_json::Value::String(after.unwrap_or_default()),
        };
        // Diagnostic only (ENGINE_SETTINGS_PROBE=1): the keys the steps changed.
        if crate::probe_enabled("ENGINE_SETTINGS_PROBE") {
            eprintln!("ENGINE_SETTINGS key={} before={:?} after={}", key, map.get(&key), value);
        }
        map.insert(key, value);
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
        ("filament_z_hop_types", "z_hop_types"),
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
    // R807: `filament_wipe_distance` overrides the machine `wipe_distance`
    // (PrintConfig.cpp filament override list; Benchy's filament sets 1 over
    // the machine's 2, Majora has none and uses 2).
    if let Some(v) = get_filament_val("filament_wipe_distance") {
        config.wipe_distance_profile = Some(v);
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

    // R632: `z_hop_type` had NEVER been read from the config at all — it sat at
    // its `ZHopType::Auto` default (print_config.rs:1041). Majora happens to BE
    // "Auto Lift", so it looked right; Benchy's machine profile says "Auto Lift"
    // too, but its FILAMENT overrides it to "Spiral Lift", and C++ reads the
    // filament-resolved value: `ZHopType(FILAMENT_CONFIG(z_hop_types))`, where
    // `FILAMENT_CONFIG(OPT)` is `m_config.OPT.get_at(filament index)`
    // (GCode.cpp:1272). So on Benchy C++ never consults `is_through_overhang` at
    // all — every travel is a plain SpiralLift, which is why its 2,029 `G17`
    // spread evenly across all 300 layers while our overhang area is
    // concentrated in a handful.
    let z_hop_str = |key: &str| -> Option<String> {
        match json.get(key) {
            Some(serde_json::Value::Array(arr)) => arr
                .first()
                .and_then(|v| v.as_str())
                .filter(|s| *s != "nil")
                .map(|s| s.to_string()),
            Some(serde_json::Value::String(s)) if s != "nil" => Some(s.clone()),
            _ => None,
        }
    };
    // Filament first, machine second — the filament preset overrides
    // (PrintConfig.cpp:64 registers `filament_z_hop_types` as the override key).
    if let Some(s) = z_hop_str("filament_z_hop_types").or_else(|| z_hop_str("z_hop_types")) {
        // PrintConfig.cpp:475-480 — the four config key strings.
        config.z_hop_type = match s.as_str() {
            "Normal Lift" => crate::print_config::ZHopType::Normal,
            "Slope Lift" => crate::print_config::ZHopType::Slope,
            "Spiral Lift" => crate::print_config::ZHopType::Spiral,
            "Auto Lift" => crate::print_config::ZHopType::Auto,
            // PrintConfig.cpp:4442 — the option's own C++ default is zhtSpiral.
            _ => crate::print_config::ZHopType::Spiral,
        };
    }
}

/// R805 — row-major 3x4 affine composition (same as the paint block's compose12).
fn compose12_rows(a: &[f64; 12], b: &[f64; 12]) -> [f64; 12] {
    [
        b[0] * a[0] + b[1] * a[3] + b[2] * a[6],
        b[0] * a[1] + b[1] * a[4] + b[2] * a[7],
        b[0] * a[2] + b[1] * a[5] + b[2] * a[8],
        b[3] * a[0] + b[4] * a[3] + b[5] * a[6],
        b[3] * a[1] + b[4] * a[4] + b[5] * a[7],
        b[3] * a[2] + b[4] * a[5] + b[5] * a[8],
        b[6] * a[0] + b[7] * a[3] + b[8] * a[6],
        b[6] * a[1] + b[7] * a[4] + b[8] * a[7],
        b[6] * a[2] + b[7] * a[5] + b[8] * a[8],
        b[0] * a[9] + b[1] * a[10] + b[2] * a[11] + b[9],
        b[3] * a[9] + b[4] * a[10] + b[5] * a[11] + b[10],
        b[6] * a[9] + b[7] * a[10] + b[8] * a[11] + b[11],
    ]
}

#[cfg(test)]
mod circle_compensation_settings_tests {
    use super::*;

    // PrintApply.cpp 1091 creates distinct regions when two parts' feature
    // settings differ. The flattened mesh may select a slot only by consensus.
    #[test]
    fn feature_filaments_must_agree_across_parts() {
        use crate::model::{Model, ModelMesh, ModelVolumeType};

        let full = crate::print::full_print_config_from_settings(&serde_json::json!({
            "filament_diameter": ["1.75", "1.75", "1.75"],
            "filament_is_mixed": ["0", "0", "1"],
        }));
        let mut model = Model::new();
        let obj = model.add_object_new();
        obj.add_instance();
        obj.config.set("extruder", 2);
        obj.add_volume_mesh(ModelMesh::new(), ModelVolumeType::ModelPart, false);
        obj.add_volume_mesh(ModelMesh::new(), ModelVolumeType::ModelPart, false);
        assert_eq!(common_model_feature_slots(&model, &full), Some([2; 3]));

        model.objects[0].volumes[1].config.set("extruder", 1);
        assert_eq!(common_model_feature_slots(&model, &full), None);
        model.objects[0].volumes[1].config.set("extruder", 2);
        model.objects[0].volumes[1].config.set("solid_infill_filament", 1);
        assert_eq!(common_model_feature_slots(&model, &full), None);

        let second = model.add_object_new();
        second.add_instance();
        second.config.set("extruder", 1);
        second.add_volume_mesh(ModelMesh::new(), ModelVolumeType::ModelPart, false);
        model.objects[0].volumes[1].config.set("solid_infill_filament", 2);
        assert_eq!(common_model_feature_slots(&model, &full), None);
        model.objects[1].config.set("extruder", 2);
        assert_eq!(common_model_feature_slots(&model, &full), Some([2; 3]));

        for object in &mut model.objects {
            for part in &mut object.volumes {
                part.config.set("solid_infill_filament", 1);
            }
        }
        assert_eq!(common_model_feature_slots(&model, &full), Some([2, 2, 1]));

        // A unanimous virtual slot still requires mixed-component ordering.
        for obj in &mut model.objects {
            obj.config.set("extruder", 3);
            for part in &mut obj.volumes {
                part.config.set("extruder", 3);
                part.config.set("solid_infill_filament", 3);
            }
        }
        assert_eq!(common_model_feature_slots(&model, &full), None);
    }

    /// GCodeProcessor.cpp:1577-1590, 2072, 2089-2091. Reuse each processor
    /// through the same profile changes and resets as the native generator.
    #[test]
    fn printable_geometry_matches_native() {
        use crate::gcode::g_code_processor::GCodeProcessor;

        let custom = serde_json::json!({
            "printable_height": "248.125123",
            "printable_area": ["-0.25x3", "200x-1.375", "189x199", "0x190.125"],
            "bed_exclude_area": ["1x1", "2x1", "1x2"],
            "wrapping_exclude_area": ["0.125x1", "2x1.3333333333", "1x2"],
        });
        let empty = serde_json::json!({
            "printable_height": "0",
            "printable_area": [], "bed_exclude_area": [], "wrapping_exclude_area": [],
        });
        let options = |profile: &serde_json::Value| {
            let (loaded, _, _) = load_bambustudio_settings(profile).unwrap();
            let mut direct = PrintConfig::default();
            let mut by_key = PrintConfig::default();
            for (key, value) in profile.as_object().unwrap() {
                let text = match value.as_array() {
                    Some(points) => points.iter().map(|p| p.as_str().unwrap()).collect::<Vec<_>>().join(","),
                    None => value.as_str().unwrap().to_owned(),
                };
                direct.set_deserialize(key, &text);
                by_key.apply_key_value(key, &text);
            }
            [loaded, direct, by_key]
        };
        let mut processors = [GCodeProcessor::new(), GCodeProcessor::new(), GCodeProcessor::new()];
        let rows = include_str!("../tests/data/g_code_processor/printable_geometry.txt");
        assert_eq!(rows.lines().count(), 6);
        for row in rows.lines() {
            let mut fields = row.split_whitespace();
            let state = fields.next().unwrap();
            let height = u32::from_str_radix(fields.next().unwrap(), 16).unwrap();
            match state {
                "reset" | "reset_again" => processors.iter_mut().for_each(|p| p.reset()),
                "default" | "default_again" => {
                    let config = PrintConfig::default();
                    for p in &mut processors { p.apply_config(&config); }
                }
                "profile" | "empty" => {
                    let configs = options(if state == "profile" { &custom } else { &empty });
                    for (p, config) in processors.iter_mut().zip(&configs) {
                        p.apply_config(config);
                    }
                }
                _ => panic!("unknown native state {state}"),
            }
            let mut read_area = || {
                let count = fields.next().unwrap().parse::<usize>().unwrap();
                (0..count).map(|_| {
                    let x = u64::from_str_radix(fields.next().unwrap(), 16).unwrap();
                    let y = u64::from_str_radix(fields.next().unwrap(), 16).unwrap();
                    (x, y)
                }).collect::<Vec<_>>()
            };
            let expected = [read_area(), read_area(), read_area()];
            assert!(fields.next().is_none(), "{state}");
            for p in &processors {
                let result = p.result();
                assert_eq!(result.printable_height.to_bits(), height, "{state}");
                let areas = [&result.printable_area, &result.bed_exclude_area, &result.wrapping_exclude_area];
                for (area, native) in areas.into_iter().zip(&expected) {
                    let bits: Vec<_> = area.iter().map(|(x, y)| (x.to_bits(), y.to_bits())).collect();
                    assert_eq!(&bits, native, "{state}");
                }
            }
        }
    }

    /// PrintConfig.cpp 4680-4687, 6090-6100 and PrintObject.cpp 3742-3806.
    /// Compare the profile slots and the applied region of an unpainted part.
    #[test]
    fn global_infill_filament_slots_match_native() {
        let rows = include_str!("../tests/data/print/region_infill_filaments.txt");
        assert_eq!(rows.lines().count(), 72);
        for row in rows.lines() {
            let fields: Vec<_> = row.split_whitespace().collect();
            let count: usize = fields[0].parse().unwrap();
            let mut profile = serde_json::json!({
                "filament_diameter": vec!["1.75"; count],
                "filament_colour": vec!["#FFFFFF"; count],
                "filament_map": vec!["1"; count],
            });
            let options = [("sparse_infill_filament", fields[1]), ("solid_infill_filament", fields[2])];
            for (key, value) in options {
                if value != "default" {
                    profile[key] = serde_json::Value::String(value.to_owned());
                }
            }
            let (config, _, region) = load_bambustudio_settings(&profile).unwrap();
            let mut slots = vec![region.sparse_infill_filament, region.solid_infill_filament];
            let mut print = Print::new();
            print.apply(config, region);
            let applied = print.get_print_region(0).config();
            slots.extend([applied.sparse_infill_filament, applied.solid_infill_filament]);
            let expected: Vec<usize> = fields[3..].iter().map(|n| n.parse().unwrap()).collect();
            assert_eq!(slots, expected, "profile {profile}");
        }
    }

    #[test]
    fn processor_temperature_maps_follow_profile_reapplication() {
        use crate::gcode::g_code_processor::GCodeProcessor;
        let mut processor = GCodeProcessor::new();
        processor.initialize("targets.gcode");
        // PrintConfig.cpp:3008-3012,3025-3029; GCodeProcessor.cpp:2004,
        // 2074-2077,5449-5480. Each typed profile replaces both maps.
        let mut old = serde_json::to_value(PrintConfig::default()).unwrap();
        assert!(old.get("physical_extruder_map").is_none());
        old["filament_map"] = serde_json::json!([]);
        let legacy: PrintConfig = serde_json::from_value(old).unwrap();
        let profiles = [
            (serde_json::json!({"physical_extruder_map": ["1","0"], "filament_map": ["1","2","1"]}), 1, true),
            (serde_json::json!({"physical_extruder_map": [], "filament_map": ["1"]}), 0, false),
            (serde_json::json!({"filament_map": []}), 0, false),
            (serde_json::json!({}), 0, true),
        ];
        for (i, (profile, physical, assigned)) in profiles.iter().enumerate() {
            let (config, _, _) = load_bambustudio_settings(profile).unwrap();
            let mut restored = PrintConfig::default();
            restored.apply_key_value_map(&config.to_key_value_map());
            for cfg in [&config, &restored] {
                processor.apply_config(cfg);
                processor.process_buffer("M104 S33 I0\n");
                processor.process_buffer(&format!("M104 S218 T{physical}\nG1 X{} F600\n", i*2+1));
                assert_eq!(processor.result().moves.last().unwrap().temperature, if *assigned { 218.0 } else { 33.0 });
                processor.process_buffer("G1 X0\n");
            }
        }
        processor.apply_config(&legacy);
        processor.process_buffer("M104 S33 I0\nM104 S218 T0\nG1 X10 F600\n");
        assert_eq!(processor.result().moves.last().unwrap().temperature, 33.0);
    }

    #[test]
    fn printable_geometry_keeps_older_profiles_and_explicit_empty_areas() {
        let mut old = serde_json::to_value(PrintConfig::default()).unwrap();
        assert!(old.get("bed_exclude_area").is_none());
        assert!(old.get("wrapping_exclude_area").is_none());
        old["printable_height"] = serde_json::json!(256.0);
        old["printable_area"] = serde_json::json!([[0, 0], [256, 0], [0, 256]]);
        let mut config: PrintConfig = serde_json::from_value(old).unwrap();
        assert_eq!(config.printable_height, 256.0);
        assert_eq!(config.printable_area, [(0.0, 0.0), (256.0, 0.0), (0.0, 256.0)]);
        assert_eq!(config.bed_exclude_area, [(0.0, 0.0)]);
        assert!(config.wrapping_exclude_area.is_empty());
        config.bed_exclude_area.clear();
        config.wrapping_exclude_area = vec![(1.25, -2.5)];
        let restored: PrintConfig = serde_json::from_value(serde_json::to_value(config).unwrap()).unwrap();
        assert!(restored.bed_exclude_area.is_empty());
        assert_eq!(restored.wrapping_exclude_area, [(1.25, -2.5)]);
    }

    /// GCodeProcessor.cpp:2081–2087. Reapplying a typed profile replaces
    /// the layer-detection flags before the first tagged layer is processed.
    #[test]
    fn layer_tag_profiles_match_native() {
        use crate::gcode::g_code_processor::GCodeProcessor;

        let program = "; CHANGE_LAYER\nG1 X1 Y2 Z0.2 F1200\nG1 X2 E1\n; CHANGE_LAYER\nG1 Z0.4\nG1 X3 E2\n";
        let profiles = (0..4).map(|flags| {
            let spiral = if flags & 1 != 0 { "1" } else { "0" };
            let scarf = if flags & 2 != 0 { "1" } else { "0" };
            let json = serde_json::json!({"spiral_mode": spiral, "has_scarf_joint_seam": scarf});
            let (loaded, _, _) = load_bambustudio_settings(&json).unwrap();
            let mut direct = PrintConfig::default();
            for (key, value) in [("spiral_mode", spiral), ("has_scarf_joint_seam", scarf)] {
                direct.apply_key_value(key, value);
            }
            [loaded, direct]
        }).collect::<Vec<_>>();
        let rows = include_str!("../tests/data/g_code_processor/layer_tag_profiles.txt");
        assert_eq!(rows.lines().count(), 16);
        for row in rows.lines() {
            let words = row.split_whitespace().collect::<Vec<_>>();
            let previous = words[0].parse::<usize>().unwrap();
            let current = words[1].parse::<usize>().unwrap();
            let layer_count = words[2].parse::<usize>().unwrap();
            assert_eq!(words.len(), 3 + 3 * layer_count);
            let expected = words[3..].chunks_exact(3).map(|data| {
                (u32::from_str_radix(data[0], 16).unwrap(),
                 (data[1].parse::<usize>().unwrap(), data[2].parse::<usize>().unwrap()))
            }).collect::<Vec<_>>();
            for (old, new) in profiles[previous].iter().zip(&profiles[current]) {
                let mut processor = GCodeProcessor::new();
                processor.reset();
                processor.apply_config(old);
                processor.apply_config(new);
                processor.process_buffer(program);
                let actual = processor.result().spiral_vase_layers.iter()
                    .map(|(height, ids)| (height.to_bits(), *ids)).collect::<Vec<_>>();
                assert_eq!(actual, expected, "{row}");
            }
        }
    }

    #[test]
    fn layer_tag_profiles_round_trip_settings() {
        let default = PrintConfig::default();
        let old = serde_json::to_value(&default).unwrap();
        assert!(old.get("has_scarf_joint_seam").is_none());
        let mut config: PrintConfig = serde_json::from_value(old).unwrap();
        assert!(!config.has_scarf_joint_seam);
        assert!(config.set_deserialize("has_scarf_joint_seam", "1"));
        let saved = serde_json::to_value(&config).unwrap();
        let mut restored: PrintConfig = serde_json::from_value(saved).unwrap();
        assert!(restored.has_scarf_joint_seam);
        restored.apply_key_value_map(&default.to_key_value_map());
        assert!(!restored.has_scarf_joint_seam);
    }

    /// PrintConfig.cpp:3000-3006, 3624-3628, 3682-3688 and GCodeProcessor.cpp:2008-2027.
    #[test]
    fn filament_material_info_matches_native() {
        use crate::gcode::g_code_processor::GCodeProcessor;

        let vectors = include_str!("../tests/data/g_code_processor/filament_material_info.txt");
        let keys = ["required_nozzle_HRC", "temperature_vitrification", "filament_cost"];
        let mut proc = GCodeProcessor::new();
        proc.reset();
        assert_eq!(vectors.lines().count(), 18);
        for line in vectors.lines() {
            let fields: Vec<_> = line.split_whitespace().collect();
            let count: usize = fields[0].parse().unwrap();
            let expected: Vec<(i32, i32, u32)> = fields[4..].chunks_exact(3).map(|row| (
                row[0].parse().unwrap(), row[1].parse().unwrap(), u32::from_str_radix(row[2], 16).unwrap(),
            )).collect();
            let mut profile = serde_json::json!({ "filament_diameter": vec!["1.75"; count] });
            let mut direct = PrintConfig::default();
            direct.filament_diameters = vec![1.75; count];
            let mut by_key = direct.clone();
            for (key, value) in keys.iter().zip(&fields[1..4]) {
                if *value != "-" {
                    profile[*key] = serde_json::json!(value.split(',').collect::<Vec<_>>());
                    assert!(direct.set_deserialize(key, value), "{key}");
                    assert!(by_key.apply_key_value(key, value), "{key}");
                }
            }
            let (loaded, _, _) = load_bambustudio_settings(&profile).unwrap();
            for config in [direct, by_key, loaded] {
                proc.apply_config(&config);
                let r = proc.result();
                assert_eq!(r.required_nozzle_hrc.len(), count, "{line}");
                assert_eq!(r.filament_vitrification_temperature.len(), count, "{line}");
                assert_eq!(r.filament_costs.len(), count, "{line}");
                for (i, row) in expected.iter().enumerate() {
                    assert_eq!((r.required_nozzle_hrc[i], r.filament_vitrification_temperature[i], r.filament_costs[i].to_bits()), *row, "{line}");
                }
            }
        }
    }

    #[test]
    fn filament_material_info_reads_older_scalar_config() {
        use crate::gcode::g_code_processor::GCodeProcessor;

        let mut config = PrintConfig::default();
        config.filament_diameters = vec![1.75; 2];
        config.filament_cost = 17.25;
        let serialized = serde_json::to_value(&config).unwrap();
        for name in ["filament_costs", "required_nozzle_hrc", "temperature_vitrification"] {
            assert!(serialized.get(name).is_none());
        }
        let config: PrintConfig = serde_json::from_value(serialized).unwrap();
        let mut proc = GCodeProcessor::new();
        proc.apply_config(&config);
        assert_eq!(proc.result().filament_costs, vec![17.25; 2]);
        assert_eq!(proc.result().required_nozzle_hrc, vec![0; 2]);
        assert_eq!(proc.result().filament_vitrification_temperature, vec![100; 2]);
    }

    /// GCodeProcessor.cpp:2016-2022, 6164-6200. Exercise the JSON settings
    /// loader and the stored travel vertices for both assignments of T0/T1.
    #[test]
    fn profile_extruder_offsets_reach_recorded_move_positions() {
        use crate::gcode::g_code_processor::{EMoveType, GCodeProcessor};
        let vectors = include_str!("../tests/data/g_code_processor/filament_offsets.txt");
        let mut cases = 0;
        for line in vectors.lines().filter(|line| line.starts_with("moves ")) {
            let fields: Vec<_> = line.split_whitespace().collect();
            let expected: Vec<_> = fields[4..].iter().map(|s| u32::from_str_radix(s, 16).unwrap()).collect();
            for offsets in [serde_json::json!(fields[1].split(',').collect::<Vec<_>>()), fields[1].into()] {
                let profile = serde_json::json!({
                    "filament_diameter": ["1.75", "1.75"],
                    "extruder_offset": offsets,
                    "filament_map": fields[2].split(',').collect::<Vec<_>>(),
                    "master_extruder_id": fields[3],
                });
                let (config, _, _) = load_bambustudio_settings(&profile).unwrap();
                let mut processor = GCodeProcessor::new();
                processor.reset();
                processor.apply_config(&config);
                processor.initialize("offset.gcode");
                processor.process_buffer("T0\nG1 X40 Y20 F600\nT1\nG1 X50 Y30 F600\n");
                processor.finalize(false);
                let positions: Vec<_> = processor.result().moves.iter()
                    .filter(|m| m.move_type == EMoveType::Travel)
                    .flat_map(|m| [m.position[0].to_bits(), m.position[1].to_bits()]).collect();
                assert_eq!(positions, expected, "{line}");
            }
            cases += 1;
        }
        assert_eq!(cases, 12);
    }

    /// PrintConfig.cpp:1085-1108 and GCodeProcessor.cpp:2042-2051.
    /// The last three vector fields are native load, unload and T0/T1/T0 print times.
    #[test]
    fn ams_profile_times_reach_the_processor() {
        use crate::gcode::g_code_processor::{ETimeMode, GCodeProcessor};
        let base = serde_json::json!({
            "gcode_flavor": "marlin2",
            "machine_max_speed_x": ["500", "500"],
            "machine_max_acceleration_x": ["9000", "9000"],
            "machine_max_acceleration_travel": ["9000", "9000"],
            "machine_max_jerk_x": ["9", "9"],
            "filament_diameter": ["1.75", "1.75"],
            "filament_map": ["1", "1"]
        });
        let vectors = include_str!("../tests/data/g_code_processor/ams_times.txt");
        let mut cases = 0;
        for line in vectors.lines() {
            let fields: Vec<_> = line.split_whitespace().collect();
            let mut settings = vec![
                ("default_ams_type", fields[1]),
                ("machine_load_filament_time", "5.5"),
                ("machine_unload_filament_time", "8.25"),
            ];
            if fields[0] != "0" {
                settings.extend([("ams_filament_load_time_ams", "31.5"), ("ams_filament_unload_time_ams_lite", "46.75")]);
            }
            if fields[0] == "2" {
                settings.extend([
                    ("ams_filament_unload_time_ams", "41.25"),
                    ("ams_filament_load_time_ams_lite", "32.125"),
                    ("ams_filament_load_time_n3f_s", "33.75"),
                    ("ams_filament_unload_time_n3f_s", "43.5"),
                ]);
            }
            let mut profile = base.clone();
            let (mut direct, _, _) = load_bambustudio_settings(&base).unwrap();
            for (key, value) in settings {
                profile[key] = value.into();
                direct.apply_key_value(key, value);
            }
            let (loaded, _, _) = load_bambustudio_settings(&profile).unwrap();
            let expected: Vec<_> = fields[2..].iter().map(|s| u32::from_str_radix(s, 16).unwrap()).collect();
            for config in [loaded, direct] {
                let mut p = GCodeProcessor::new();
                p.reset();
                p.apply_config(&config);
                p.initialize("ams.gcode");
                p.process_buffer("T0\nG1 X10 F600\nT1\nG1 X0 F600\nT0\nG1 X10 F600\n");
                p.finalize(false);
                let times = [
                    p.time_processor().filament_load_times,
                    p.time_processor().filament_unload_times,
                    p.get_time(ETimeMode::Normal),
                ];
                assert_eq!(times.map(f32::to_bits).as_slice(), expected, "{line}");
            }
            cases += 1;
        }
        assert_eq!(cases, 18);
    }

    /// GCodeProcessor.cpp:2030-2036, 2055-2066. Modern Marlin has separate
    /// travel accelerations in the two modes. Legacy Marlin uses extrusion's.
    #[test]
    fn profile_motion_limits_reach_the_time_estimator() {
        use crate::gcode::g_code_processor::{ETimeMode, GCodeProcessor};
        let vectors = include_str!("../tests/data/g_code_processor/motion_limits.txt");
        let mut cases = 0;
        for line in vectors.lines().filter(|line| line.starts_with("time ")) {
            let fields: Vec<_> = line.split_whitespace().collect();
            let json = serde_json::json!({
                "gcode_flavor": fields[1],
                "machine_max_speed_x": ["500", "500"],
                "machine_max_acceleration_x": ["9000", "9000"],
                "machine_max_acceleration_travel": fields[2].split(',').collect::<Vec<_>>(),
                "machine_max_acceleration_extruding": "9000,9000",
                "machine_max_jerk_x": ["9", "9"],
                "machine_min_travel_rate": ["0", "0"]
            });
            let (config, _, _) = load_bambustudio_settings(&json).unwrap();
            let mut processor = GCodeProcessor::new();
            processor.reset();
            processor.apply_config(&config);
            processor.enable_stealth_time_estimator(true);
            processor.initialize("motion.gcode");
            processor.process_buffer("G90\nG1 X1000 F30000\nG1 X0 F30000\n");
            processor.finalize(false);
            for (i, mode) in [ETimeMode::Normal, ETimeMode::Stealth].into_iter().enumerate() {
                assert_eq!(processor.get_time(mode).to_bits(), u32::from_str_radix(fields[3 + i], 16).unwrap(), "{line}");
            }
            cases += 1;
        }
        assert_eq!(cases, 6);
    }

    /// GCodeProcessor.cpp:2030-2036. A profile's second mode limits both X
    /// travels in the pin's end-to-end timing vectors (gen.cpp --axis-limits).
    #[test]
    fn profile_axis_limits_reach_the_time_estimator() {
        use crate::gcode::g_code_processor::{ETimeMode, GCodeProcessor};
        let vectors = include_str!("../tests/data/g_code_processor/axis_limits.txt");
        let mut cases = 0;
        for line in vectors.lines().filter(|line| line.starts_with("time ")) {
            let fields: Vec<_> = line.split_whitespace().collect();
            let json = serde_json::json!({
                "gcode_flavor": "marlin",
                "machine_max_speed_x": fields[1].split(',').collect::<Vec<_>>(),
                "machine_max_acceleration_x": ["9000", "9000"],
                "machine_max_acceleration_travel": "9000",
                "machine_max_acceleration_extruding": "9000",
                "machine_max_jerk_x": ["9", "9"],
                "machine_min_travel_rate": ["0", "0"]
            });
            let (config, _, _) = load_bambustudio_settings(&json).unwrap();
            let mut processor = GCodeProcessor::new();
            processor.reset();
            processor.apply_config(&config);
            processor.enable_stealth_time_estimator(true);
            processor.initialize("axis.gcode");
            processor.process_buffer("G90\nG1 X1000 F30000\nG1 X0 F30000\n");
            processor.finalize(false);
            for (i, mode) in [ETimeMode::Normal, ETimeMode::Stealth].into_iter().enumerate() {
                assert_eq!(processor.get_time(mode).to_bits(), u32::from_str_radix(fields[2 + i], 16).unwrap(), "{line}");
            }
            cases += 1;
        }
        assert_eq!(cases, 3);
    }

    /// R950: the auto circle compensation options of a settings file reach the typed configs as the pin's
    /// `ConfigBase::load_from_json` gives them (Config.cpp:1030-1080: a JSON array is joined with ',' and
    /// deserialized as a ConfigOptionFloats; a string as it is).
    #[test]
    fn circle_compensation_options_are_read_whole() {
        let json = serde_json::json!({
            "enable_circle_compensation": "1",
            "circle_compensation_manual_offset": "0.05",
            "counter_coef_2": ["0.008", "0.009", "0.01"],
            "hole_limit_max": ["0.2"],
        });
        let (pc, oc, _rc) = load_bambustudio_settings(&json).unwrap();
        assert!(oc.enable_circle_compensation);
        assert_eq!(oc.circle_compensation_manual_offset, 0.05);
        assert_eq!(pc.counter_coef_2, vec![0.008, 0.009, 0.01]);
        assert_eq!(pc.hole_limit_max, vec![0.2]);
        // Not in the file: the pin's default (PrintConfig.cpp:6951-6954).
        assert_eq!(pc.counter_coef_1, vec![0.0]);
        assert_eq!(pc.circle_compensation_floats("counter_coef_2"), vec![0.008, 0.009, 0.01]);
    }

    /// R953: `filament_velocity_adaptation_factor` (PrintConfig.hpp:1745, ConfigOptionFloats, default
    /// {1.0}, PrintConfig.cpp:3294-3301) reaches the typed config whole, and `get_at` gives the first
    /// value past the end (Config.hpp:681-685), as FILAMENT_CONFIG reads it (GCode.cpp:1570, 6779, 6809).
    #[test]
    fn filament_velocity_adaptation_factor_is_read_whole() {
        let (pc, _oc, _rc) = load_bambustudio_settings(&serde_json::json!({})).unwrap();
        assert_eq!(pc.filament_velocity_adaptation_factor, vec![1.0]);
        assert_eq!(pc.filament_velocity_adaptation_factor_at(3), 1.0);
        let json = serde_json::json!({
            "filament_velocity_adaptation_factor": ["1", "0.8", "1.25"],
        });
        let (pc, _oc, _rc) = load_bambustudio_settings(&json).unwrap();
        assert_eq!(pc.filament_velocity_adaptation_factor, vec![1.0, 0.8, 1.25]);
        assert_eq!(pc.filament_velocity_adaptation_factor_at(1), 0.8);
        assert_eq!(pc.filament_velocity_adaptation_factor_at(7), 1.0);
        let mut pc2 = pc.clone();
        assert!(pc2.set_deserialize("filament_velocity_adaptation_factor", "0.5,2"));
        assert_eq!(pc2.filament_velocity_adaptation_factor, vec![0.5, 2.0]);
        // A nil item fails the (non-nullable) deserialize: the values stay.
        assert!(pc2.apply_key_value("filament_velocity_adaptation_factor", "nil"));
        assert_eq!(pc2.filament_velocity_adaptation_factor, vec![0.5, 2.0]);
    }

    /// R954: `printer_extruder_id`, `printer_extruder_variant` and `extruder_type` (PrintConfig.hpp:1536,
    /// 1538, 1519) of a settings file reach the typed config as the pin's ConfigBase::load_from_json
    /// reads them over FullPrintConfig's defaults (Config.cpp:885-1080), and `extruder_types()` is
    /// GCodeProcessor.cpp:2040's `cast<ExtruderType>`. Vectors: tests/data/machine_variant_fields
    /// (gen.cpp runs the pin's load_from_json on each document).
    #[test]
    fn machine_variant_fields_are_read_as_the_pin() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/machine_variant_fields/vectors.txt");
        let text = std::fs::read_to_string(path).expect("vectors");
        let mut want = String::new();
        let mut got = String::new();
        let mut cases = 0;
        for line in text.lines() {
            if let Some(doc) = line.strip_prefix("JSON ") {
                let json: serde_json::Value = serde_json::from_str(doc).expect("json");
                let (pc, _oc, _rc) = load_bambustudio_settings(&json).unwrap();
                let ids: Vec<String> = pc.printer_extruder_id.iter().map(|v| v.to_string()).collect();
                let ets: Vec<String> = pc.extruder_types().iter().map(|v| (*v as i32).to_string()).collect();
                got.push_str(&format!(
                    "ID {}\nVAR {}\nET {}\n",
                    ids.join(","),
                    crate::config::escape_strings_cstyle(&pc.printer_extruder_variant),
                    ets.join(",")
                ));
                cases += 1;
            } else if line.starts_with("ID ") || line.starts_with("VAR ") || line.starts_with("ET ") {
                want.push_str(line);
                want.push('\n');
            }
        }
        assert_eq!(cases, 14);
        assert_eq!(got, want);
    }
}
