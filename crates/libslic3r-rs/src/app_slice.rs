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
use crate::print_config::{PrintConfig, PrintObjectConfig};
use crate::region_config::PrintRegionConfig;
use crate::stl::read_stl_file as load_stl;
use crate::triangle_mesh::{Triangle, TriangleMesh};

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
    let frame_origin = (0.0, 0.0);

    // Create PrintObject — slicing happens internally during Print::process().
    info!("Creating PrintObject...");
    let print_object = PrintObject::with_config(mesh, object_config);

    // Create Print and add object.
    info!("Creating Print...");
    let mut print = Print::new();
    // main.cpp:1456 — print.apply(model, config). Applies the resolved config +
    // default region config; objects are added separately below.
    print.apply(print_config, region_config);
    print.raw_settings = raw_settings_json;
    // FRAME_PAIR export origin (= the slice center applied above).
    print.gcode_origin = frame_origin;
    print.set_status_callback(|percent, message| {
        info!("Progress: {}% - {}", percent, message);
    });
    print.add_object(print_object);

    // Run Print::process() pipeline.
    // main.cpp:1486 — Print::validate() before slicing; abort on a hard error
    // (empty `.string` means valid). Warnings (is_warning) are non-fatal.
    let validation = print.validate();
    if !validation.string.is_empty() && !validation.is_warning {
        anyhow::bail!("validation error: {}", validation.string);
    }

    info!("Running Print::process() pipeline...");
    let __timing = crate::probe_enabled("SLICE_PHASE_TIMING");
    let __t_proc = std::time::Instant::now();
    print
        .process(None, false)
        .with_context(|| "Failed to process print")?;
    let __proc_s = __t_proc.elapsed().as_secs_f64();

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
/// Tier-1 scope: all objects are merged into a single mesh and sliced with one
/// material. Painted MMU segmentation / per-object filaments are NOT yet applied
/// (multicolour parity is a separate milestone).
///
/// * `input` — path to a `.3mf` file.
/// * `settings_override` — optional BambuStudio settings JSON that takes
///   precedence over the 3MF's embedded config.
/// * `output` — path to write the generated G-code to.
pub fn slice_3mf_to_gcode(
    input: &Path,
    settings_override: Option<&Path>,
    output: &Path,
) -> Result<()> {
    info!("Slicing 3MF (in-process): {:?}", input);

    // Load the merged mesh + embedded settings + per-instance identify_ids.
    info!("Loading 3MF...");
    let Loaded3mf {
        mut mesh,
        settings_json: embedded_settings,
        identify_ids,
        mmu_facets,
        negative_mesh,
    } = load_3mf(input).with_context(|| format!("Failed to load 3MF: {:?}", input))?;
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
    patch_filament_overrides_in_json(&mut settings_value);
    let raw_settings_json = Some(settings_value.clone());
    let (print_config, object_config, region_config) = load_bambustudio_settings(&settings_value)?;

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
    if !mmu_facets.is_empty() {
        let mut selector =
            crate::triangle_selector::TriangleSelector::new(mesh.clone(), 0.0);
        selector.deserialize(
            &mmu_facets.data,
            false,
            crate::triangle_selector::EnforcerBlockerType::EXTRUDER_MAX,
            crate::triangle_selector::EnforcerBlockerType::NONE,
            crate::triangle_selector::EnforcerBlockerType::NONE,
        );
        for state in selector.used_states() {
            let its = selector.get_facets(state);
            if !its.indices.is_empty() {
                painted_submeshes.push((state.0 as u8, its));
            }
        }
        // Ascending-extruder pairing drives the painted region order.
        painting_extruders = painted_submeshes.iter().map(|(e, _)| *e).collect();
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
    print_object.num_total_filaments = print_config.num_filaments();

    info!("Creating Print...");
    let mut print = Print::new();
    // main.cpp:1456 — print.apply(model, config). Applies the resolved config +
    // default region config; objects are added separately below.
    print.apply(print_config, region_config);
    // Declare one region per painted extruder (PrintApply.cpp:1062-1078 shape)
    // BEFORE add_object so they share into PrintObjectRegions::all_regions.
    if !painting_extruders.is_empty() {
        print.install_painted_regions(&painting_extruders);
    }
    print.raw_settings = raw_settings_json;
    print.set_status_callback(|percent, message| {
        info!("Progress: {}% - {}", percent, message);
    });
    print.add_object(print_object);

    // main.cpp:1486 — Print::validate() before slicing; abort on a hard error
    // (empty `.string` means valid). Warnings (is_warning) are non-fatal.
    let validation = print.validate();
    if !validation.string.is_empty() && !validation.is_warning {
        anyhow::bail!("validation error: {}", validation.string);
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

    // Parse identify_ids from Metadata/model_settings.config
    // C++ equivalent: ModelInstance::loaded_id from _BBS_3MF_Importer::_handle_end_model_instance()
    // The identify_id is what get_labeled_id() returns → used for ; OBJECT_ID: comments
    let identify_ids = if let Ok(mut entry) = archive.by_name("Metadata/model_settings.config") {
        let mut buf = String::new();
        entry.read_to_string(&mut buf).ok();
        parse_identify_ids_from_model_settings(&buf)
    } else {
        Vec::new()
    };

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

    // R703 — which <object id>s are negative volumes. BambuStudio records the
    // role in model_settings, not in 3dmodel.model (which only says
    // type="other"), so the printable/negative/modifier distinction needs both.
    let negative_ids = if let Ok(mut entry) = archive.by_name("Metadata/model_settings.config") {
        let mut buf = String::new();
        entry.read_to_string(&mut buf).ok();
        parse_negative_part_ids_from_model_settings(&buf)
    } else {
        std::collections::HashSet::new()
    };

    // Parse vertices, triangles, and painted-MMU state from the XML
    let parsed = parse_3mf_model_xml_with_negatives(&model_xml, &negative_ids)?;
    Ok(Loaded3mf {
        mesh: parsed.mesh,
        settings_json,
        identify_ids,
        mmu_facets: parsed.mmu_facets,
        negative_mesh: parsed.negative_mesh,
    })
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
    // R703 — negative-volume geometry, kept apart from the printable mesh.
    let mut neg_vertices: Vec<Point3F> = Vec::new();
    let mut neg_triangles: Vec<Triangle> = Vec::new();

    #[allow(clippy::too_many_arguments)]
    fn instantiate_object(
        obj_id: u32,
        transform: &[f64; 12],
        objects: &std::collections::HashMap<u32, MeshData>,
        all_vertices: &mut Vec<Point3F>,
        all_triangles: &mut Vec<Triangle>,
        all_paints: &mut Vec<Option<String>>,
        identity: &[f64; 12],
        negative_ids: &std::collections::HashSet<u32>,
        neg_vertices: &mut Vec<Point3F>,
        neg_triangles: &mut Vec<Triangle>,
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
                }
                return;
            }
            // If object has a mesh, instantiate it
            if !mesh_data.vertices.is_empty() {
                let v_offset = all_vertices.len() as u32;
                for v in &mesh_data.vertices {
                    all_vertices.push(apply_transform(v, transform));
                }
                for (i, tri) in mesh_data.triangles.iter().enumerate() {
                    all_triangles.push(Triangle::new(
                        tri[0] + v_offset,
                        tri[1] + v_offset,
                        tri[2] + v_offset,
                    ));
                    all_paints.push(mesh_data.paints.get(i).cloned().flatten());
                }
            }
            // If object has components, recurse with composed transforms
            for &(comp_id, ref comp_transform) in &mesh_data.components {
                let composed = compose_transforms(comp_transform, transform);
                instantiate_object(
                    comp_id,
                    &composed,
                    objects,
                    all_vertices,
                    all_triangles,
                    all_paints,
                    identity,
                    negative_ids,
                    neg_vertices,
                    neg_triangles,
                );
            }
        }
    }

    if build_items.is_empty() {
        // Fallback: no <build> section, just collect all printable meshes
        for (_, mesh_data) in &objects {
            if mesh_data.is_model && !mesh_data.vertices.is_empty() {
                let v_offset = all_vertices.len() as u32;
                all_vertices.extend_from_slice(&mesh_data.vertices);
                for (i, tri) in mesh_data.triangles.iter().enumerate() {
                    all_triangles.push(Triangle::new(
                        tri[0] + v_offset,
                        tri[1] + v_offset,
                        tri[2] + v_offset,
                    ));
                    all_paints.push(mesh_data.paints.get(i).cloned().flatten());
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
                &identity,
                negative_ids,
                &mut neg_vertices,
                &mut neg_triangles,
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
    Ok(Parsed3mfModel {
        mesh: TriangleMesh::from_parts(all_vertices, all_triangles),
        negative_mesh: TriangleMesh::from_parts(neg_vertices, neg_triangles),
        mmu_facets,
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
