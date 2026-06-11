//! CLI for the Rust slicer.
//!
//! This binary provides a command-line interface for slicing 3D models
//! using the Print::process() pipeline from BambuStudio's libslic3r.

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use env_logger::Builder;
use log::{info, warn, LevelFilter};
use std::fs;
use std::io::Read as IoRead;
use std::path::PathBuf;

use slicer::geometry::Point3F;
use slicer::print::{Print, PrintObject};
use slicer::print_config::{PrintConfig, PrintObjectConfig};
use slicer::region_config::PrintRegionConfig;
use slicer::stl::read_stl_file as load_stl;
use slicer::{Triangle, TriangleMesh};

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

/// Load a TriangleMesh from a .3mf ZIP file by parsing the 3D/3dmodel.model XML.
///
/// Also optionally extracts Metadata/project_settings.config if present,
/// returning it as a JSON string.
fn load_3mf(path: &std::path::Path) -> Result<(TriangleMesh, Option<String>, Vec<usize>)> {
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

    // Parse vertices and triangles from the XML
    let mesh = parse_3mf_model_xml(&model_xml)?;
    Ok((mesh, settings_json, identify_ids))
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
fn parse_3mf_model_xml(xml: &str) -> Result<TriangleMesh> {
    use quick_xml::events::Event;
    use quick_xml::Reader;

    // 3MF structure: <resources> has <object> elements with meshes,
    // <build> has <item> elements with transforms referencing objects.
    // Objects can also have <components> that reference other objects.

    // Collect all objects (meshes) by ID
    struct MeshData {
        vertices: Vec<Point3F>,
        triangles: Vec<[u32; 3]>,
        components: Vec<(u32, [f64; 12])>, // (objectid, transform)
    }

    let mut objects: std::collections::HashMap<u32, MeshData> = std::collections::HashMap::new();
    let mut build_items: Vec<(u32, [f64; 12])> = Vec::new(); // (objectid, transform)

    let identity: [f64; 12] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];

    fn parse_transform(s: &str) -> [f64; 12] {
        let vals: Vec<f64> = s
            .split_whitespace()
            .filter_map(|v| v.parse::<f64>().ok())
            .collect();
        if vals.len() >= 12 {
            [
                vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8],
                vals[9], vals[10], vals[11],
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
                        for attr in e.attributes().flatten() {
                            if attr.key.local_name().as_ref() == b"id" {
                                if let Ok(s) = std::str::from_utf8(&attr.value) {
                                    id = s.parse().unwrap_or(0);
                                }
                            }
                        }
                        current_object_id = Some(id);
                        objects.entry(id).or_insert_with(|| MeshData {
                            vertices: Vec::new(),
                            triangles: Vec::new(),
                            components: Vec::new(),
                        });
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
                                _ => {}
                            }
                        }
                        if let Some(id) = current_object_id {
                            objects
                                .entry(id)
                                .and_modify(|m| m.triangles.push([v1, v2, v3]));
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

    fn instantiate_object(
        obj_id: u32,
        transform: &[f64; 12],
        objects: &std::collections::HashMap<u32, MeshData>,
        all_vertices: &mut Vec<Point3F>,
        all_triangles: &mut Vec<Triangle>,
        identity: &[f64; 12],
    ) {
        if let Some(mesh_data) = objects.get(&obj_id) {
            // If object has a mesh, instantiate it
            if !mesh_data.vertices.is_empty() {
                let v_offset = all_vertices.len() as u32;
                for v in &mesh_data.vertices {
                    all_vertices.push(apply_transform(v, transform));
                }
                for tri in &mesh_data.triangles {
                    all_triangles.push(Triangle::new(
                        tri[0] + v_offset,
                        tri[1] + v_offset,
                        tri[2] + v_offset,
                    ));
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
                    identity,
                );
            }
        }
    }

    if build_items.is_empty() {
        // Fallback: no <build> section, just collect all meshes
        for (_, mesh_data) in &objects {
            if !mesh_data.vertices.is_empty() {
                let v_offset = all_vertices.len() as u32;
                all_vertices.extend_from_slice(&mesh_data.vertices);
                for tri in &mesh_data.triangles {
                    all_triangles.push(Triangle::new(
                        tri[0] + v_offset,
                        tri[1] + v_offset,
                        tri[2] + v_offset,
                    ));
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
                &identity,
            );
        }
    }

    if all_vertices.is_empty() || all_triangles.is_empty() {
        return Err(anyhow::anyhow!(
            "No mesh data found in 3MF XML ({} vertices, {} triangles)",
            all_vertices.len(),
            all_triangles.len()
        ));
    }

    info!(
        "Parsed 3MF: {} vertices, {} triangles (from {} objects, {} build items)",
        all_vertices.len(),
        all_triangles.len(),
        objects.len(),
        build_items.len()
    );
    Ok(TriangleMesh::from_parts(all_vertices, all_triangles))
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
    let mut embedded_settings_str: Option<String> = None;

    let mut identify_ids: Vec<usize> = Vec::new();
    let mut mesh = if is_3mf {
        info!("Loading 3MF...");
        let (m, settings_opt, ids) =
            load_3mf(&input).with_context(|| format!("Failed to load 3MF: {:?}", input))?;
        embedded_settings_str = settings_opt;
        identify_ids = ids;
        m
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
        arachne_min_bead_width: 0.34,
        arachne_min_feature_size: 0.25,
        arachne_wall_transition_length: 0.4,

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
        infill_anchor: 2.5,
        infill_anchor_max: 12.0,

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
