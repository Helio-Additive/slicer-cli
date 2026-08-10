//! CLI for the Rust slicer.
//!
//! This binary provides a command-line interface for slicing 3D models
//! using the Print::process() pipeline from BambuStudio's libslic3r.

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use env_logger::Builder;
use log::{info, warn, LevelFilter};
use std::fs;
use std::path::PathBuf;

use slicer::app_slice::load_3mf;
use slicer::print::{Print, PrintObject};
use slicer::print_config::{PrintConfig, PrintObjectConfig};
use slicer::region_config::PrintRegionConfig;
use slicer::stl::read_stl_file as load_stl;

#[derive(Parser)]
#[command(name = "slicer")]
#[command(about = "Rust port of BambuStudio libslic3r slicing engine", long_about = None)]
struct Cli {
    /// Enable verbose logging
    #[arg(short, long)]
    verbose: bool,

    /// Enable debug logging
    #[arg(short, long)]
    debug: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Slice a 3D model to G-code
    Slice {
        /// Input STL file
        #[arg(short, long)]
        input: PathBuf,

        /// Output G-code file (default: <input>.gcode)
        #[arg(short, long)]
        output: Option<PathBuf>,

        /// Configuration JSON file (Rust serde format)
        #[arg(short, long)]
        config: Option<PathBuf>,

        /// BambuStudio project_settings.config JSON file
        #[arg(long)]
        settings: Option<PathBuf>,

        /// Layer height in mm
        #[arg(long, default_value = "0.2")]
        layer_height: f64,

        /// First layer height in mm
        #[arg(long, default_value = "0.2")]
        first_layer_height: f64,

        /// Number of perimeter shells
        #[arg(long, default_value = "2")]
        perimeters: u32,

        /// Infill density (0.0 to 1.0)
        #[arg(long, default_value = "0.2")]
        infill_density: f64,
    },

    /// Validate generated G-code against reference
    Validate {
        /// Input STL file
        #[arg(short, long)]
        input: PathBuf,

        /// Reference G-code file
        #[arg(short, long)]
        reference: PathBuf,

        /// Generated G-code file
        #[arg(short, long)]
        generated: PathBuf,

        /// Skip re-slicing, only compare existing files
        #[arg(long)]
        compare_only: bool,
    },

    /// Display model information
    Info {
        /// Input file (STL, OBJ, 3MF, etc.)
        #[arg(short, long)]
        input: PathBuf,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Setup logging
    let log_level = if cli.debug {
        LevelFilter::Debug
    } else if cli.verbose {
        LevelFilter::Info
    } else {
        LevelFilter::Warn
    };

    Builder::from_default_env().filter_level(log_level).init();

    match cli.command {
        Commands::Slice {
            input,
            output,
            config,
            settings,
            layer_height,
            first_layer_height,
            perimeters,
            infill_density,
        } => slice_command(
            input,
            output,
            config,
            settings,
            layer_height,
            first_layer_height,
            perimeters,
            infill_density,
        ),

        Commands::Validate {
            input,
            reference,
            generated,
            compare_only,
        } => validate_command(input, reference, generated, compare_only),

        Commands::Info { input } => info_command(input),
    }
}

/// Slice command implementation using Print::process() pipeline.
fn slice_command(
    input: PathBuf,
    output: Option<PathBuf>,
    config: Option<PathBuf>,
    settings: Option<PathBuf>,
    layer_height: f64,
    first_layer_height: f64,
    perimeters: u32,
    infill_density: f64,
) -> Result<()> {
    info!("Slicing: {:?}", input);

    // Determine output path
    let output_path = output.unwrap_or_else(|| {
        let mut path = input.clone();
        path.set_extension("gcode");
        path
    });

    // Load mesh - detect format by extension
    let is_3mf = input
        .extension()
        .map_or(false, |e| e.eq_ignore_ascii_case("3mf"));

    // For the explicit `--settings <bambustudio json>` + STL path, delegate to the
    // shared library entrypoint (slicer::app_slice::slice_to_gcode) so the bin and
    // the in-process host (helio-slicer-cli) run identical code. 3MF + embedded
    // settings are still handled inline below (the library entrypoint is STL-only).
    if let Some(ref settings_path) = settings {
        if !is_3mf {
            slicer::app_slice::slice_to_gcode(&input, settings_path, &output_path)?;
            let gcode_content = fs::read_to_string(&output_path)
                .with_context(|| format!("Failed to read generated G-code: {:?}", output_path))?;
            let line_count = gcode_content.lines().count();
            info!("Output written to: {:?}", output_path);
            info!("G-code lines: {}", line_count);
            println!("G-code lines: {}", line_count);
            return Ok(());
        }
    }
    let mut embedded_settings_str: Option<String> = None;

    let mut identify_ids: Vec<usize> = Vec::new();
    let mut mesh = if is_3mf {
        info!("Loading 3MF...");
        let loaded =
            load_3mf(&input).with_context(|| format!("Failed to load 3MF: {:?}", input))?;
        embedded_settings_str = loaded.settings_json;
        identify_ids = loaded.identify_ids;
        loaded.mesh
    } else {
        info!("Loading STL...");
        load_stl(&input).with_context(|| format!("Failed to load STL: {:?}", input))?
    };
    info!("Loaded {} triangles", mesh.triangle_count());

    // Create configuration
    info!("Creating print configuration...");
    let mut raw_settings_json: Option<serde_json::Value> = None;
    let (print_config, object_config, region_config) = if let Some(settings_path) = settings {
        // Load from BambuStudio project_settings.config JSON
        info!("Loading BambuStudio settings from {:?}", settings_path);
        let settings_str = fs::read_to_string(&settings_path)
            .with_context(|| format!("Failed to read settings: {:?}", settings_path))?;
        let mut settings_json: serde_json::Value = serde_json::from_str(&settings_str)
            .with_context(|| format!("Failed to parse settings JSON: {:?}", settings_path))?;
        patch_filament_overrides_in_json(&mut settings_json);
        raw_settings_json = Some(settings_json.clone());
        load_bambustudio_settings(&settings_json)?
    } else if let Some(ref emb_str) = embedded_settings_str {
        // Use settings embedded in the 3MF file
        info!("Using embedded 3MF settings");
        let mut settings_json: serde_json::Value = serde_json::from_str(emb_str)
            .with_context(|| "Failed to parse embedded 3MF settings JSON")?;
        patch_filament_overrides_in_json(&mut settings_json);
        raw_settings_json = Some(settings_json.clone());
        load_bambustudio_settings(&settings_json)?
    } else if let Some(config_path) = config {
        // Load from Rust serde JSON
        let config_str = fs::read_to_string(&config_path)
            .with_context(|| format!("Failed to read config: {:?}", config_path))?;
        let pc: PrintConfig = serde_json::from_str(&config_str)
            .with_context(|| format!("Failed to parse config: {:?}", config_path))?;
        let oc = create_default_object_config(&pc, perimeters, infill_density);
        let rc = create_default_region_config(perimeters, infill_density);
        (pc, oc, rc)
    } else {
        // Create default config with CLI overrides
        let pc = create_default_print_config(
            layer_height,
            first_layer_height,
            perimeters,
            infill_density,
        );
        let oc = create_default_object_config(&pc, perimeters, infill_density);
        let rc = create_default_region_config(perimeters, infill_density);
        (pc, oc, rc)
    };

    // Center model on bed (BambuStudio auto-centers imported models)
    {
        let bbox = mesh.bounding_box();
        let model_center_x = (bbox.min.x + bbox.max.x) / 2.0;
        let model_center_y = (bbox.min.y + bbox.max.y) / 2.0;
        let bed_center_x = print_config.bed_size_x / 2.0;
        let bed_center_y = print_config.bed_size_y / 2.0;
        let dx = bed_center_x - model_center_x;
        let dy = bed_center_y - model_center_y;
        // Also place model on the bed surface (Z=0)
        let dz = -bbox.min.z;
        mesh.translate(slicer::geometry::Point3F {
            x: dx,
            y: dy,
            z: dz,
        });
        info!(
            "Centered model on bed: translated by ({:.1}, {:.1}, {:.1})",
            dx, dy, dz
        );
    }

    // Create PrintObject - slicing happens internally during Print::process()
    // C++ reference: PrintObject::make_perimeters() calls this->slice()
    // PrintObject.cpp:456
    info!("Creating PrintObject...");
    let mut print_object = PrintObject::with_config(mesh, object_config);
    // Set identify_id from model_settings.config (first object's first instance)
    // C++ equivalent: ModelInstance::get_labeled_id() using loaded_id
    if let Some(&id) = identify_ids.first() {
        print_object.label_id = id;
    }

    // Create Print and add object
    info!("Creating Print...");
    let mut print = Print::new();

    // Pass CLI config to Print so export_gcode uses the correct values
    *print.config_mut() = print_config;

    // Set the region config so perimeters/infill use correct settings
    // (default region is created in add_object if none exists)
    print.set_default_region_config(region_config);

    // Store raw settings for CONFIG_BLOCK generation
    print.raw_settings = raw_settings_json;

    // Set up progress callback
    print.set_status_callback(|percent, message| {
        info!("Progress: {}% - {}", percent, message);
    });

    print.add_object(print_object);

    // Run Print::process() pipeline
    info!("Running Print::process() pipeline...");
    print
        .process(None, false)
        .with_context(|| "Failed to process print")?;

    // Export G-code using Print::export_gcode()
    info!("Exporting G-code...");
    let _stats = print
        .export_gcode(&output_path)
        .with_context(|| format!("Failed to export G-code to {:?}", output_path))?;

    // Read back the file to count lines (for logging)
    let gcode_content = fs::read_to_string(&output_path)
        .with_context(|| format!("Failed to read generated G-code: {:?}", output_path))?;

    let line_count = gcode_content.lines().count();
    info!("Output written to: {:?}", output_path);
    info!(
        "Layers: {}",
        print
            .objects()
            .first()
            .map(|o| o.layers().len())
            .unwrap_or(0)
    );
    info!("G-code lines: {}", line_count);
    println!(
        "Layers: {}",
        print
            .objects()
            .first()
            .map(|o| o.layers().len())
            .unwrap_or(0)
    );
    println!("G-code lines: {}", line_count);

    Ok(())
}

/// Create default PrintConfig with CLI parameter overrides.
fn create_default_print_config(
    layer_height: f64,
    first_layer_height: f64,
    _perimeters: u32,
    _infill_density: f64,
) -> PrintConfig {
    PrintConfig {
        layer_height,
        first_layer_height,
        print_speed: 200.0,
        travel_speed: 1000.0,
        first_layer_speed: 50.0,
        nozzle_diameter: 0.4,
        filament_diameter: 1.75,
        extrusion_multiplier: 1.0,
        extruder_temperature: 220,
        first_layer_extruder_temperature: 220,
        bed_temperature: 60,
        first_layer_bed_temperature: 60,
        retract_length: 0.4,
        retract_length_toolchange: 2.0,
        retract_speed: 40.0,
        deretract_speed: 40.0,
        retract_lift: 0.0,
        retract_before_travel: 2.0,
        skirt_loops: 1,
        skirt_distance: 2.0,
        filament_density: 1.24,
        filament_cost: 20.0,
        filament_flow_ratio: 0.98,
        resolution: 0.0125,
        arc_fitting_enabled: true,
        arc_fitting_tolerance: 0.05,
        arc_fitting_min_radius: 0.5,
        arc_fitting_max_radius: 100.0,
        z_hop_type: slicer::print_config::ZHopType::Normal,
        use_relative_e: true,
        use_relative_e_distances_cooling: true,
        ..PrintConfig::default()
    }
}

/// Create default PrintObjectConfig from PrintConfig.
fn create_default_object_config(
    print_config: &PrintConfig,
    perimeters: u32,
    infill_density: f64,
) -> PrintObjectConfig {
    PrintObjectConfig {
        layer_height: print_config.layer_height,
        first_layer_height: print_config.first_layer_height,
        perimeters,
        wall_loops: perimeters,
        top_solid_layers: 5, // H2D+PLA Basic: 5 top layers
        bottom_solid_layers: 3,
        fill_density: infill_density,
        fill_pattern: slicer::print_config::InfillPattern::Grid,

        // Line widths (matching BambuStudio reference)
        line_width: 0.4,               // default line width
        initial_layer_line_width: 0.4, // first layer
        outer_wall_line_width: 0.4,    // outer wall
        inner_wall_line_width: 0.4,    // inner wall
        sparse_infill_line_width: 0.4, // sparse infill
        solid_infill_line_width: 0.4,  // solid infill
        top_surface_line_width: 0.4,   // top surface

        // Speeds (H2D+PLA Basic config - high speed profile)
        perimeter_speed: 300.0,          // inner wall speed
        external_perimeter_speed: 200.0, // outer wall speed
        infill_speed: 350.0,             // sparse infill speed
        solid_infill_speed: 60.0,
        top_solid_infill_speed: 30.0,
        bridge_speed: 25.0,
        gap_fill_speed: 30.0,

        // Perimeter options
        thin_walls: true,
        gap_fill: true,
        overhangs: true,
        only_one_wall_first_layer: false,
        top_one_wall_type: slicer::print_config::TopOneWallType::None,

        // Quality
        slice_closing_radius: 0.049,
        xy_size_compensation: 0.0,
        elephant_foot_compensation: 0.0,

        // Infill
        infill_wall_overlap: 0.15,
        infill_angle: 45.0,

        // Flow
        initial_layer_flow_ratio: 1.0,
        top_solid_infill_flow_ratio: 1.0,
        print_flow_ratio: 1.0, // BambuStudio reference uses 1.0 (0.98 is at filament level)

        // Seam
        seam_position: slicer::print_config::SeamPosition::Aligned,

        // Fuzzy skin
        fuzzy_skin: false,
        fuzzy_skin_thickness: 0.3,
        fuzzy_skin_point_distance: 0.8,

        // Wipe
        wipe_enabled: false,
        wipe_distance: 2.0,
        retract_before_wipe: 0.0,

        // Arachne
        perimeter_mode: slicer::print_config::PerimeterMode::Classic,
        // R706 — PERCENT of nozzle (C++ coPercent defaults), not mm.
        arachne_min_bead_width: 85.0,
        arachne_min_feature_size: 25.0,
        arachne_wall_transition_length: 100.0,

        // Narrow region detection
        detect_narrow_internal_solid_infill: true,
        minimum_sparse_infill_area: 15.0,

        // Interface shells / spiral
        interface_shells: false,

        // Raft
        raft_layers: 0,
        raft_expansion: 1.5,
        raft_contact_distance: 0.1,

        // Support
        enable_support: print_config.support_enabled,
        support_type: print_config.support_type,
        support_threshold_angle: print_config.support_threshold_angle,
        support_density: print_config.support_density,

        // G-code (H2D+PLA Basic uses relative E)
        use_relative_e_distances: true,

        // Spiral vase from print config
        spiral_vase: print_config.spiral_vase,

        ..PrintObjectConfig::default()
    }
}

/// Create default PrintRegionConfig matching H2D+PLA Basic reference.
fn create_default_region_config(perimeters: u32, infill_density: f64) -> PrintRegionConfig {
    PrintRegionConfig {
        // Perimeters (matching BambuStudio reference: 2 perimeters)
        perimeters,
        outer_wall_line_width: 0.4,      // outer wall
        inner_wall_line_width: 0.4,      // inner wall
        external_perimeter_speed: 200.0, // outer wall speed
        perimeter_speed: 300.0,          // inner wall speed
        small_perimeter_speed: 200.0,
        thin_walls: true,
        overhangs: true,
        extra_perimeters: true,
        extra_perimeters_on_overhangs: false,

        // Infill (H2D+PLA Basic: 15% grid)
        fill_density: infill_density,
        fill_pattern: slicer::print_config::InfillPattern::Grid,
        solid_fill_pattern: slicer::print_config::InfillPattern::Rectilinear,
        top_fill_pattern: slicer::print_config::InfillPattern::Rectilinear,
        bottom_fill_pattern: slicer::print_config::InfillPattern::Rectilinear,
        fill_angle: 45.0,
        sparse_infill_line_width: 0.4,         // sparse infill
        internal_solid_infill_line_width: 0.4, // solid infill
        top_surface_line_width: 0.4,           // top surface
        infill_speed: 350.0,                   // H2D+PLA high speed
        solid_infill_speed: 300.0,
        top_solid_infill_speed: 200.0,
        infill_overlap: 0.15,
        // C++ defaults: sparse_infill_anchor = 400%, sparse_infill_anchor_max = 20mm
        // (PrintConfig.cpp:3551/3579)
        infill_anchor: slicer::config::FloatOrPercent::with(400.0, true),
        infill_anchor_max: slicer::config::FloatOrPercent::with(20.0, false),

        // Solid Layers (H2D+PLA Basic)
        top_solid_layers: 5,
        bottom_solid_layers: 3,
        top_solid_min_thickness: 0.0,
        bottom_solid_min_thickness: 0.0,
        ensure_vertical_shell_thickness:
            slicer::region_config::EnsureVerticalThicknessLevel::Enabled,

        // Bridges
        bridge_speed: 25.0,
        bridge_flow_ratio: 1.0,
        bridge_angle: 0.0,

        // Gap Fill
        gap_fill_enabled: true,
        gap_fill_speed: 30.0,
        filter_out_gap_fill: 0.0,

        // Embedding Wall (InterlockingGenerator)
        embedding_wall_into_infill: false,

        // Seam
        seam_position: slicer::print_config::SeamPosition::Aligned,
        seam_angle_cost: 1.0,
        seam_travel_cost: 1.0,

        // Scarf Seam (seam slope) — C++ defaults (PrintConfig.cpp:4666-4754)
        override_filament_scarf_seam_setting: false,
        seam_slope_type: slicer::print_config::ScarfSeamType::None,
        seam_slope_conditional: true,
        seam_slope_start_height: slicer::config::FloatOrPercent::with(10.0, true),
        seam_slope_gap: slicer::config::FloatOrPercent::with(0.0, false),
        seam_slope_entire_loop: false,
        seam_slope_min_length: 10.0,
        seam_slope_steps: 10,
        seam_slope_inner_walls: true,

        // Ironing
        ironing: false,
        ironing_type: slicer::region_config::IroningType::TopSurfaces,
        ironing_flow_rate: 0.15,
        ironing_spacing: 0.1,
        ironing_speed: 15.0,

        // Fuzzy Skin
        fuzzy_skin: false,
        fuzzy_skin_mode: slicer::region_config::FuzzySkinMode::None,
        fuzzy_skin_type: slicer::region_config::FuzzySkinType::None,
        fuzzy_skin_thickness: 0.3,
        fuzzy_skin_point_distance: 0.8,
        fuzzy_skin_first_layer: false,
        fuzzy_skin_noise_type: slicer::region_config::NoiseType::Classic,
        fuzzy_skin_scale: 1.0,
        fuzzy_skin_octaves: 4,
        fuzzy_skin_persistence: 0.5,
        fuzzy_skin_displacement_mode: slicer::region_config::FuzzySkinDisplacementMode::Displacement,

        // Wall Generation Mode
        wall_generator_mode: slicer::perimeter_generator::WallGeneratorMode::Classic,
        wall_sequence: slicer::print_config::WallSequence::InnerOuter,

        // Misc
        region_id: 0,
        wall_filament: 1, // 1-based extruder
        sparse_infill_filament: 1,
        solid_infill_filament: 1,

        // Locked-zag / lattice / surface-density options take their faithful
        // C++ defaults (see region_config.rs Default).
        ..PrintRegionConfig::default()
    }
}

/// Validate command - compare generated G-code with reference.
fn validate_command(
    _input: PathBuf,
    reference: PathBuf,
    generated: PathBuf,
    compare_only: bool,
) -> Result<()> {
    if !compare_only {
        warn!("Re-slicing not implemented - use --compare-only");
        return Ok(());
    }

    // Read both files to get line counts
    let ref_content = fs::read_to_string(&reference)
        .with_context(|| format!("Failed to read reference: {:?}", reference))?;
    let gen_content = fs::read_to_string(&generated)
        .with_context(|| format!("Failed to read generated: {:?}", generated))?;

    let ref_lines = ref_content.lines().count();
    let gen_lines = gen_content.lines().count();

    // Use the validation module if available
    info!("Reference: {} lines", ref_lines);
    info!("Generated: {} lines", gen_lines);

    // Output minimal validation report in expected format
    println!("Quality Score: 0.0 / 100.0 (threshold: 70.0)");
    println!("Status: PARTIAL - Print::process() pipeline active but G-code export incomplete");
    println!();
    println!("Issues found:");
    println!("  0 critical");
    println!("  0 errors");
    println!("  1 warnings");
    println!();
    println!("Warnings:");
    println!("  - G-code export needs full implementation (GCode::do_export port)");
    println!();
    println!("G-code line counts:");
    println!("  Reference: {} lines", ref_lines);
    println!("  Generated: {} lines", gen_lines);

    Ok(())
}

/// Load BambuStudio project_settings.config JSON and create config structs.
///
/// Uses set_deserialize() methods on config structs to iterate ALL keys
/// from the JSON, rather than cherry-picking specific keys. This ensures
/// every setting from the project is applied.
fn load_bambustudio_settings(
    json: &serde_json::Value,
) -> Result<(PrintConfig, PrintObjectConfig, PrintRegionConfig)> {
    let mut print_config = PrintConfig::default();
    let mut object_config = PrintObjectConfig::default();
    let mut region_config = PrintRegionConfig::default();

    // Iterate all keys from the JSON and dispatch to config structs
    let mut recognized = 0u32;
    let mut unrecognized = 0u32;
    if let Some(obj) = json.as_object() {
        for (key, value) in obj {
            // Extract string value (scalar or first element of array)
            let str_val = match value {
                serde_json::Value::String(s) => s.clone(),
                serde_json::Value::Array(arr) => match arr.first() {
                    Some(serde_json::Value::String(s)) => s.clone(),
                    _ => continue,
                },
                _ => continue,
            };

            // Try each config struct — keys may apply to multiple structs
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
    // BambuStudio filament settings override machine defaults when not "nil".
    // Updates both PrintConfig AND raw JSON (so template processor sees overrides too).
    apply_filament_overrides(&mut print_config, json);

    // === Extruder offset: "extruder_offset": ["XxY"] e.g. "0x2" ===
    // C++ reference: GCode.cpp:7089 — point_to_gcode() subtracts extruder_offset from XY.
    apply_extruder_offset(&mut print_config, json);

    info!(
        "Settings loaded: layer_height={}, perimeters={}, infill={:.0}%, pattern={:?}",
        print_config.layer_height,
        object_config.perimeters,
        object_config.fill_density * 100.0,
        object_config.fill_pattern
    );
    info!(
        "Bed: {}x{}mm, nozzle={}mm, bed_temp={}°C, nozzle_temp={}°C",
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
/// C++ reference: GCode.cpp:7089 — point_to_gcode() subtracts extruder_offset.
fn apply_extruder_offset(config: &mut PrintConfig, json: &serde_json::Value) {
    if let Some(offset_str) = get_json_str(json, "extruder_offset") {
        // Format is "XxY" e.g. "0x2"
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
        // Format: ["0x0", "256x0", "256x256", "0x256"]
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
/// default of `auto_brim`. C++ reference: PrintConfig.cpp:1480
/// `def->set_default_value(new ConfigOptionEnum<BrimType>(btAutoBrim));`
/// (btAutoBrim is the first/default brim type). Auto brim analyses adhesion
/// needs per-object; for a simple object with no overhangs no brim is emitted,
/// which is why a missing `brim_type` must NOT silently apply `brim_width`.
fn apply_brim_logic(config: &mut PrintConfig, json: &serde_json::Value) {
    let brim_type = match json.get("brim_type") {
        Some(serde_json::Value::String(s)) => s.clone(),
        // PrintConfig.cpp:1480 — default is btAutoBrim when unset.
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
/// filament config on top of machine config, so retraction_length[0] = 0.4
/// when filament_retraction_length[0] = 0.4 (even though machine retraction_length = 0.8).
/// See GCode.cpp:3158 and Print.cpp where filament config overrides machine config.
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
                    // Take first element; if not "nil", override the machine key
                    if let Some(first) = arr.first().and_then(|v| v.as_str()) {
                        if first != "nil" {
                            // Build new array with the filament value for each extruder
                            let new_arr: Vec<serde_json::Value> = arr
                                .iter()
                                .map(|v| {
                                    if let Some(s) = v.as_str() {
                                        if s == "nil" {
                                            // Keep machine default for nil entries
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
/// BambuStudio filament settings (filament_retraction_length, etc.) override
/// machine defaults (retraction_length, etc.) when the filament value is not "nil".
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

/// Info command - display model information.
fn info_command(input: PathBuf) -> Result<()> {
    info!("Reading: {:?}", input);

    let mut mesh = load_stl(&input).with_context(|| format!("Failed to load STL: {:?}", input))?;

    println!("STL Info:");
    println!("  Triangles: {}", mesh.triangle_count());
    println!("  Vertices: {}", mesh.vertex_count());

    let bbox = mesh.bounding_box();
    println!("  Bounding box:");
    println!(
        "    Min: ({:.2}, {:.2}, {:.2})",
        bbox.min.x, bbox.min.y, bbox.min.z
    );
    println!(
        "    Max: ({:.2}, {:.2}, {:.2})",
        bbox.max.x, bbox.max.y, bbox.max.z
    );
    println!(
        "    Size: ({:.2} × {:.2} × {:.2}) mm",
        bbox.size().x,
        bbox.size().y,
        bbox.size().z
    );

    Ok(())
}
