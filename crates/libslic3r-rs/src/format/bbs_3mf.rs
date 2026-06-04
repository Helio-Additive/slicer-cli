//! BambuStudio 3MF file format handler
//!
//! C++ Reference:
//! - Format/bbs_3mf.hpp
//! - Format/bbs_3mf.cpp
//!
//! Handles loading and saving BambuStudio's extended 3MF format,
//! which includes plate data, print configuration, thumbnails, and more.

use crate::{Error, Result};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::path::Path;

// ---------------------------------------------------------------------------
// Constants (from bbs_3mf.hpp)
// ---------------------------------------------------------------------------

pub const PLATE_THUMBNAIL_SMALL_WIDTH: u32 = 128;
pub const PLATE_THUMBNAIL_SMALL_HEIGHT: u32 = 128;

pub const GCODE_FILE_FORMAT: &str = "Metadata/plate_{}.gcode";
pub const THUMBNAIL_FILE_FORMAT: &str = "Metadata/plate_{}.png";
pub const NO_LIGHT_THUMBNAIL_FILE_FORMAT: &str = "Metadata/plate_no_light_{}.png";
pub const TOP_FILE_FORMAT: &str = "Metadata/top_{}.png";
pub const PICK_FILE_FORMAT: &str = "Metadata/pick_{}.png";
pub const PATTERN_CONFIG_FILE_FORMAT: &str = "Metadata/plate_{}.json";
pub const EMBEDDED_PRINT_FILE_FORMAT: &str = "Metadata/process_settings_{}.config";
pub const EMBEDDED_FILAMENT_FILE_FORMAT: &str = "Metadata/filament_settings_{}.config";
pub const EMBEDDED_PRINTER_FILE_FORMAT: &str = "Metadata/machine_settings_{}.config";

pub const BBL_DESIGNER_MODEL_TITLE_TAG: &str = "Title";
pub const BBL_DESIGNER_PROFILE_ID_TAG: &str = "DesignProfileId";
pub const BBL_DESIGNER_PROFILE_TITLE_TAG: &str = "ProfileTitle";
pub const BBL_DESIGNER_MODEL_ID_TAG: &str = "DesignModelId";

pub const BRIM_POINTS_FORMAT_VERSION: i32 = 1;

// Export stages
pub const EXPORT_STAGE_OPEN_3MF: i32 = 0;
pub const EXPORT_STAGE_CONTENT_TYPES: i32 = 1;
pub const EXPORT_STAGE_ADD_THUMBNAILS: i32 = 2;
pub const EXPORT_STAGE_ADD_RELATIONS: i32 = 3;
pub const EXPORT_STAGE_ADD_MODELS: i32 = 4;
pub const EXPORT_STAGE_ADD_LAYER_RANGE: i32 = 5;
pub const EXPORT_STAGE_ADD_SUPPORT: i32 = 6;
pub const EXPORT_STAGE_ADD_CUSTOM_GCODE: i32 = 7;
pub const EXPORT_STAGE_ADD_PRINT_CONFIG: i32 = 8;
pub const EXPORT_STAGE_ADD_PROJECT_CONFIG: i32 = 9;
pub const EXPORT_STAGE_ADD_CONFIG_FILE: i32 = 10;
pub const EXPORT_STAGE_ADD_SLICE_INFO: i32 = 11;
pub const EXPORT_STAGE_ADD_GCODE: i32 = 12;
pub const EXPORT_STAGE_ADD_AUXILIARIES: i32 = 13;
pub const EXPORT_STAGE_FINISH: i32 = 14;

// Import stages
pub const IMPORT_STAGE_RESTORE: i32 = 0;
pub const IMPORT_STAGE_OPEN: i32 = 1;
pub const IMPORT_STAGE_READ_FILES: i32 = 2;
pub const IMPORT_STAGE_EXTRACT: i32 = 3;
pub const IMPORT_STAGE_LOADING_OBJECTS: i32 = 4;
pub const IMPORT_STAGE_LOADING_PLATES: i32 = 5;
pub const IMPORT_STAGE_FINISH: i32 = 6;
pub const IMPORT_STAGE_ADD_INSTANCE: i32 = 7;
pub const IMPORT_STAGE_UPDATE_GCODE: i32 = 8;
pub const IMPORT_STAGE_CHECK_MODE_GCODE: i32 = 9;
pub const UPDATE_GCODE_RESULT: i32 = 10;
pub const IMPORT_LOAD_CONFIG: i32 = 11;
pub const IMPORT_LOAD_MODEL_OBJECTS: i32 = 12;
pub const IMPORT_STAGE_MAX: i32 = 13;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Save strategy flags for BBS 3MF export
/// Format/bbs_3mf.hpp: SaveStrategy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SaveStrategy(u32);

impl SaveStrategy {
    pub const DEFAULT: SaveStrategy = SaveStrategy(0);
    pub const FULL_PATH_SOURCES: SaveStrategy = SaveStrategy(1);
    pub const ZIP64: SaveStrategy = SaveStrategy(1 << 1);
    pub const PRODUCTION_EXT: SaveStrategy = SaveStrategy(1 << 2);
    pub const SECURE_CONTENT_EXT: SaveStrategy = SaveStrategy(1 << 3);
    pub const WITH_GCODE: SaveStrategy = SaveStrategy(1 << 4);
    pub const SILENCE: SaveStrategy = SaveStrategy(1 << 5);
    pub const SKIP_STATIC: SaveStrategy = SaveStrategy(1 << 6);
    pub const SKIP_MODEL: SaveStrategy = SaveStrategy(1 << 7);
    pub const WITH_SLICE_INFO: SaveStrategy = SaveStrategy(1 << 8);
    pub const SKIP_AUXILIARY: SaveStrategy = SaveStrategy(1 << 9);
    pub const USE_LOADED_ID: SaveStrategy = SaveStrategy(1 << 10);
    pub const SHARE_MESH: SaveStrategy = SaveStrategy(1 << 11);

    pub const SPLIT_MODEL: SaveStrategy = SaveStrategy(0x1000 | Self::PRODUCTION_EXT.0);
    pub const ENCRYPTED: SaveStrategy =
        SaveStrategy(Self::SECURE_CONTENT_EXT.0 | Self::SPLIT_MODEL.0);
    pub const BACKUP: SaveStrategy = SaveStrategy(
        0x10000 | Self::WITH_GCODE.0 | Self::SILENCE.0 | Self::SKIP_STATIC.0 | Self::SPLIT_MODEL.0,
    );

    pub fn contains(self, other: SaveStrategy) -> bool {
        (self.0 & other.0) == other.0
    }

    pub fn union(self, other: SaveStrategy) -> SaveStrategy {
        SaveStrategy(self.0 | other.0)
    }
}

/// Load strategy flags for BBS 3MF import
/// Format/bbs_3mf.hpp: LoadStrategy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoadStrategy(u32);

impl LoadStrategy {
    pub const DEFAULT: LoadStrategy = LoadStrategy(0);
    pub const ADD_DEFAULT_INSTANCES: LoadStrategy = LoadStrategy(1);
    pub const CHECK_VERSION: LoadStrategy = LoadStrategy(2);
    pub const LOAD_MODEL: LoadStrategy = LoadStrategy(4);
    pub const LOAD_CONFIG: LoadStrategy = LoadStrategy(8);
    pub const LOAD_AUXILIARY: LoadStrategy = LoadStrategy(16);
    pub const SILENCE: LoadStrategy = LoadStrategy(32);
    pub const IMPERIAL_UNITS: LoadStrategy = LoadStrategy(64);

    pub const RESTORE: LoadStrategy = LoadStrategy(
        0x10000
            | Self::LOAD_MODEL.0
            | Self::LOAD_CONFIG.0
            | Self::LOAD_AUXILIARY.0
            | Self::SILENCE.0,
    );

    pub fn contains(self, other: LoadStrategy) -> bool {
        (self.0 & other.0) == other.0
    }

    pub fn union(self, other: LoadStrategy) -> LoadStrategy {
        LoadStrategy(self.0 | other.0)
    }
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Thumbnail data (PNG image data)
/// Format/bbs_3mf.hpp
#[derive(Debug, Clone)]
pub struct ThumbnailData {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

impl ThumbnailData {
    pub fn new() -> Self {
        ThumbnailData {
            width: 0,
            height: 0,
            pixels: Vec::new(),
        }
    }
}

/// Triangle color data
/// Format/bbs_3mf.hpp
#[derive(Debug, Clone)]
pub struct TriangleColor {
    pub pid: i32,
    pub indices: [i32; 3],
}

impl TriangleColor {
    pub fn new() -> Self {
        TriangleColor {
            pid: -1,
            indices: [-1, -1, -1],
        }
    }
}

/// Filament info for a plate
#[derive(Debug, Clone)]
pub struct FilamentInfo {
    pub id: i32,
    pub tray_id: i32,
    pub color: String,
    pub filament_type: String,
    pub setting_id: String,
}

impl FilamentInfo {
    pub fn new() -> Self {
        FilamentInfo {
            id: 0,
            tray_id: 0,
            color: String::new(),
            filament_type: String::new(),
            setting_id: String::new(),
        }
    }
}

/// Dynamic print configuration (key-value pairs)
/// Format/bbs_3mf.hpp
#[derive(Debug, Clone)]
pub struct DynamicPrintConfig {
    pub values: HashMap<String, String>,
}

impl DynamicPrintConfig {
    pub fn new() -> Self {
        DynamicPrintConfig {
            values: HashMap::new(),
        }
    }

    pub fn set(&mut self, key: &str, value: &str) {
        self.values.insert(key.to_string(), value.to_string());
    }

    pub fn get(&self, key: &str) -> Option<&str> {
        self.values.get(key).map(|s| s.as_str())
    }
}

/// Configuration substitution context
#[derive(Debug, Clone)]
pub struct ConfigSubstitutionContext {
    pub substitutions: Vec<(String, String, String)>, // key, old_value, new_value
}

impl ConfigSubstitutionContext {
    pub fn new() -> Self {
        ConfigSubstitutionContext {
            substitutions: Vec::new(),
        }
    }
}

/// Plate data: stores all information about a single print plate
/// Format/bbs_3mf.hpp: PlateData
#[derive(Debug, Clone)]
pub struct PlateData {
    pub plate_index: i32,
    pub objects_and_instances: Vec<(i32, i32)>,
    pub obj_inst_map: HashMap<i32, (i32, i32)>,
    pub printer_model_id: String,
    pub nozzle_diameters: String,
    pub gcode_file: String,
    pub gcode_file_md5: String,
    pub thumbnail_file: String,
    pub no_light_thumbnail_file: String,
    pub plate_thumbnail: ThumbnailData,
    pub top_file: String,
    pub pick_file: String,
    pub pattern_bbox_file: String,
    pub gcode_prediction: String,
    pub gcode_weight: String,
    pub first_layer_time: String,
    pub plate_name: String,
    pub slice_filaments_info: Vec<FilamentInfo>,
    pub skipped_objects: Vec<usize>,
    pub config: DynamicPrintConfig,
    pub is_support_used: bool,
    pub is_sliced_valid: bool,
    pub toolpath_outside: bool,
    pub is_label_object_enabled: bool,
    pub timelapse_warning_code: i32,
    pub filament_maps: Vec<i32>,
    pub filament_change_sequence: Vec<u32>,
    pub limit_filament_maps: Vec<i32>,
    pub locked: bool,
}

impl PlateData {
    pub fn new() -> Self {
        PlateData {
            plate_index: -1,
            objects_and_instances: Vec::new(),
            obj_inst_map: HashMap::new(),
            printer_model_id: String::new(),
            nozzle_diameters: String::new(),
            gcode_file: String::new(),
            gcode_file_md5: String::new(),
            thumbnail_file: String::new(),
            no_light_thumbnail_file: String::new(),
            plate_thumbnail: ThumbnailData::new(),
            top_file: String::new(),
            pick_file: String::new(),
            pattern_bbox_file: String::new(),
            gcode_prediction: String::new(),
            gcode_weight: String::new(),
            first_layer_time: String::new(),
            plate_name: String::new(),
            slice_filaments_info: Vec::new(),
            skipped_objects: Vec::new(),
            config: DynamicPrintConfig::new(),
            is_support_used: false,
            is_sliced_valid: false,
            toolpath_outside: false,
            is_label_object_enabled: false,
            timelapse_warning_code: 0,
            filament_maps: Vec::new(),
            filament_change_sequence: Vec::new(),
            limit_filament_maps: Vec::new(),
            locked: false,
        }
    }

    /// Create a PlateData with specific plate_id, objects, and lock state
    pub fn with_plate_id(plate_id: i32, obj_to_inst_list: &[(i32, i32)], lock_state: bool) -> Self {
        let mut pd = PlateData::new();
        pd.plate_index = plate_id;
        pd.locked = lock_state;
        pd.objects_and_instances = obj_to_inst_list.to_vec();
        pd
    }

    /// Get gcode prediction string
    pub fn get_gcode_prediction_str(&self) -> &str {
        &self.gcode_prediction
    }

    /// Get gcode weight string
    pub fn get_gcode_weight_str(&self) -> &str {
        &self.gcode_weight
    }
}

/// Packing temporary data for 3MF export
/// Format/bbs_3mf.hpp: PackingTemporaryData
#[derive(Debug, Clone)]
pub struct PackingTemporaryData {
    pub thumbnail_3mf: String,
    pub printer_thumbnail_middle_3mf: String,
    pub printer_thumbnail_small_3mf: String,
}

impl PackingTemporaryData {
    pub fn new() -> Self {
        PackingTemporaryData {
            thumbnail_3mf: String::new(),
            printer_thumbnail_middle_3mf: String::new(),
            printer_thumbnail_small_3mf: String::new(),
        }
    }
}

/// Volume color information map
pub type VolumeColorInfoMap = HashMap<i32, super::obj::VolumeColorInfo>;

/// Export progress callback type
pub type Export3mfProgressFn = Option<Box<dyn Fn(i32, i32, i32, &mut bool) + Send + Sync>>;

/// Import progress callback type
pub type Import3mfProgressFn = Option<Box<dyn Fn(i32, i32, i32, &mut bool) + Send + Sync>>;

/// Parameters for storing a BBS 3MF file
/// Format/bbs_3mf.hpp: StoreParams
#[derive(Debug)]
pub struct StoreParams {
    pub path: String,
    pub model: Option<Model>,
    pub plate_data_list: Vec<PlateData>,
    pub export_plate_idx: i32,
    pub config: Option<DynamicPrintConfig>,
    pub thumbnail_data: Vec<ThumbnailData>,
    pub no_light_thumbnail_data: Vec<ThumbnailData>,
    pub top_thumbnail_data: Vec<ThumbnailData>,
    pub pick_thumbnail_data: Vec<ThumbnailData>,
    pub calibration_thumbnail_data: Vec<ThumbnailData>,
    pub strategy: SaveStrategy,
}

impl StoreParams {
    pub fn new() -> Self {
        StoreParams {
            path: String::new(),
            model: None,
            plate_data_list: Vec::new(),
            export_plate_idx: -1,
            config: None,
            thumbnail_data: Vec::new(),
            no_light_thumbnail_data: Vec::new(),
            top_thumbnail_data: Vec::new(),
            pick_thumbnail_data: Vec::new(),
            calibration_thumbnail_data: Vec::new(),
            strategy: SaveStrategy::ZIP64,
        }
    }
}

/// Simple model for BBS 3MF I/O
#[derive(Debug, Clone)]
pub struct Model {
    pub objects: Vec<ModelObject>,
    pub metadata: HashMap<String, String>,
}

impl Model {
    pub fn new() -> Self {
        Model {
            objects: Vec::new(),
            metadata: HashMap::new(),
        }
    }
}

/// Simple model object
#[derive(Debug, Clone)]
pub struct ModelObject {
    pub name: String,
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<[i32; 3]>,
}

impl ModelObject {
    pub fn new() -> Self {
        ModelObject {
            name: String::new(),
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }
}

/// RAII guard for saving object mesh (backup support)
/// Format/bbs_3mf.hpp: SaveObjectGaurd
pub struct SaveObjectGaurd {
    _object_name: String,
}

impl SaveObjectGaurd {
    pub fn new(object_name: &str) -> Self {
        // In C++, the constructor triggers a backup save. Here we note the intent.
        SaveObjectGaurd {
            _object_name: object_name.to_string(),
        }
    }
}

impl Drop for SaveObjectGaurd {
    fn drop(&mut self) {
        // In C++, the destructor triggers cleanup. No-op in Rust port since
        // backup management is handled externally.
    }
}

// ---------------------------------------------------------------------------
// Internal 3MF XML constants
// ---------------------------------------------------------------------------

const MODEL_FILE: &str = "3D/3dmodel.model";
const CONTENT_TYPES_FILE: &str = "[Content_Types].xml";
const RELATIONSHIPS_FILE: &str = "_rels/.rels";

// ---------------------------------------------------------------------------
// Public API functions
// ---------------------------------------------------------------------------

/// Load a BBS 3MF file
/// Format/bbs_3mf.cpp: load_bbs_3mf
///
/// Reads a .3mf ZIP archive containing the 3D model data, plate configurations,
/// print settings, thumbnails, and other BambuStudio-specific metadata.
pub fn load_bbs_3mf(
    path: &str,
    config: &mut DynamicPrintConfig,
    config_substitutions: &mut ConfigSubstitutionContext,
    model: &mut Model,
    plate_data_list: &mut Vec<PlateData>,
    strategy: LoadStrategy,
) -> Result<bool> {
    let zip_path = Path::new(path);
    if !zip_path.exists() {
        return Err(Error::IO(format!("File not found: {}", path)));
    }

    let file = std::fs::File::open(zip_path)
        .map_err(|e| Error::IO(format!("Failed to open 3MF file: {}", e)))?;
    let mut archive = zip::ZipArchive::new(file)
        .map_err(|e| Error::IO(format!("Failed to read ZIP archive: {}", e)))?;

    // Read the 3D model file
    if let Ok(mut model_file) = archive.by_name(MODEL_FILE) {
        let mut content = String::new();
        model_file
            .read_to_string(&mut content)
            .map_err(|e| Error::IO(format!("Failed to read model file: {}", e)))?;
        parse_model_xml(&content, model)?;
    }

    // Read configuration files from Metadata/
    for i in 0..archive.len() {
        let mut file = archive
            .by_index(i)
            .map_err(|e| Error::IO(format!("Failed to read archive entry: {}", e)))?;
        let name = file.name().to_string();

        if name.starts_with("Metadata/") && name.ends_with(".config") {
            let mut content = String::new();
            file.read_to_string(&mut content)
                .map_err(|e| Error::IO(format!("Failed to read config: {}", e)))?;
            parse_config_content(&content, config);
        } else if name.starts_with("Metadata/plate_") && name.ends_with(".json") {
            let mut content = String::new();
            file.read_to_string(&mut content)
                .map_err(|e| Error::IO(format!("Failed to read plate data: {}", e)))?;
            if let Ok(plate) = parse_plate_json(&content) {
                plate_data_list.push(plate);
            }
        }
    }

    Ok(true)
}

/// Store a BBS 3MF file
/// Format/bbs_3mf.cpp: store_bbs_3mf
pub fn store_bbs_3mf(store_params: &StoreParams) -> Result<bool> {
    if store_params.path.is_empty() {
        return Err(Error::IO("Empty path for 3MF export".to_string()));
    }

    let file = std::fs::File::create(&store_params.path)
        .map_err(|e| Error::IO(format!("Failed to create 3MF file: {}", e)))?;
    let mut zip = zip::ZipWriter::new(file);

    let options =
        zip::write::FileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    // Write content types
    zip.start_file(CONTENT_TYPES_FILE, options)
        .map_err(|e| Error::IO(format!("Failed to write content types: {}", e)))?;
    write!(
        zip,
        r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"#
    )
    .map_err(|e| Error::IO(format!("Write error: {}", e)))?;

    // Write relationships
    zip.start_file(RELATIONSHIPS_FILE, options)
        .map_err(|e| Error::IO(format!("Failed to write relationships: {}", e)))?;
    write!(
        zip,
        r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"#
    )
    .map_err(|e| Error::IO(format!("Write error: {}", e)))?;

    // Write 3D model
    if let Some(model) = &store_params.model {
        zip.start_file(MODEL_FILE, options)
            .map_err(|e| Error::IO(format!("Failed to write model: {}", e)))?;
        let xml = generate_model_xml(model);
        zip.write_all(xml.as_bytes())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }

    // Write plate data
    for (i, plate) in store_params.plate_data_list.iter().enumerate() {
        let filename = format!("Metadata/plate_{}.json", i + 1);
        zip.start_file(&filename, options)
            .map_err(|e| Error::IO(format!("Failed to write plate data: {}", e)))?;
        let json = generate_plate_json(plate);
        zip.write_all(json.as_bytes())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }

    // Write config
    if let Some(config) = &store_params.config {
        zip.start_file("Metadata/project_settings.config", options)
            .map_err(|e| Error::IO(format!("Failed to write config: {}", e)))?;
        let config_str = generate_config_content(config);
        zip.write_all(config_str.as_bytes())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }

    zip.finish()
        .map_err(|e| Error::IO(format!("Failed to finalize ZIP: {}", e)))?;

    Ok(true)
}

/// Release plate data list (clears the vector)
/// Format/bbs_3mf.cpp: release_PlateData_list
pub fn release_plate_data_list(plate_data_list: &mut Vec<PlateData>) {
    plate_data_list.clear();
}

/// Get gcode prediction string from a PlateData
/// Format/bbs_3mf.hpp: PlateData::get_gcode_prediction_str
pub fn get_gcode_prediction_str(plate: &PlateData) -> &str {
    &plate.gcode_prediction
}

/// Run backup UI tasks (no-op in current Rust implementation)
/// Format/bbs_3mf.cpp: run_backup_ui_tasks
///
/// In the C++ implementation, this delegates to _BBS_Backup_Manager::run_ui_tasks().
/// In the Rust port, backup management is handled externally.
pub fn run_backup_ui_tasks() {
    // No-op: backup UI tasks are managed externally in the Rust architecture
}

/// Check if restore data exists at the given path
/// Format/bbs_3mf.cpp: has_restore_data
pub fn has_restore_data(path: &str) -> (bool, String) {
    if path.is_empty() {
        return (false, "<lock>".to_string());
    }

    let lock_file = format!("{}/lock.txt", path);
    if Path::new(&lock_file).exists() {
        // Lock file exists, check if the process is still running
        if let Ok(pid_str) = std::fs::read_to_string(&lock_file) {
            let pid_str = pid_str.trim();
            // In production, we'd check if the process with this PID is still running.
            // For now, if the lock file exists, assume no restore data available.
            return (false, pid_str.to_string());
        }
    }

    // Check for model file
    let model_file = format!("{}/3D/3dmodel.model", path);
    if Path::new(&model_file).exists() {
        return (true, String::new());
    }

    (false, String::new())
}

// ---------------------------------------------------------------------------
// Internal XML/JSON helpers
// ---------------------------------------------------------------------------

/// Parse the 3D model XML content
fn parse_model_xml(content: &str, model: &mut Model) -> Result<()> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut reader = Reader::from_str(content);
    reader.trim_text(true);

    let mut current_object: Option<ModelObject> = None;
    let mut in_vertices = false;
    let mut in_triangles = false;
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                match e.name().as_ref() {
                    b"object" => {
                        let mut obj = ModelObject::new();
                        for attr in e.attributes().flatten() {
                            if attr.key.as_ref() == b"name" {
                                obj.name = String::from_utf8_lossy(&attr.value).to_string();
                            }
                        }
                        current_object = Some(obj);
                    }
                    b"vertices" => {
                        in_vertices = true;
                    }
                    b"triangles" => {
                        in_triangles = true;
                    }
                    b"vertex" => {
                        if in_vertices {
                            if let Some(ref mut obj) = current_object {
                                let mut x = 0.0f32;
                                let mut y = 0.0f32;
                                let mut z = 0.0f32;
                                for attr in e.attributes().flatten() {
                                    match attr.key.as_ref() {
                                        b"x" => {
                                            x = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0.0)
                                        }
                                        b"y" => {
                                            y = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0.0)
                                        }
                                        b"z" => {
                                            z = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0.0)
                                        }
                                        _ => {}
                                    }
                                }
                                obj.vertices.push([x, y, z]);
                            }
                        }
                    }
                    b"triangle" => {
                        if in_triangles {
                            if let Some(ref mut obj) = current_object {
                                let mut v1 = 0i32;
                                let mut v2 = 0i32;
                                let mut v3 = 0i32;
                                for attr in e.attributes().flatten() {
                                    match attr.key.as_ref() {
                                        b"v1" => {
                                            v1 = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0)
                                        }
                                        b"v2" => {
                                            v2 = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0)
                                        }
                                        b"v3" => {
                                            v3 = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0)
                                        }
                                        _ => {}
                                    }
                                }
                                obj.indices.push([v1, v2, v3]);
                            }
                        }
                    }
                    b"metadata" => {
                        // Read metadata key-value pairs
                        let mut key = String::new();
                        for attr in e.attributes().flatten() {
                            if attr.key.as_ref() == b"name" {
                                key = String::from_utf8_lossy(&attr.value).to_string();
                            }
                        }
                        if !key.is_empty() {
                            if let Ok(Event::Text(txt)) = reader.read_event_into(&mut buf) {
                                let value = String::from_utf8_lossy(txt.as_ref()).to_string();
                                model.metadata.insert(key, value);
                            }
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) => match e.name().as_ref() {
                b"object" => {
                    if let Some(obj) = current_object.take() {
                        model.objects.push(obj);
                    }
                }
                b"vertices" => in_vertices = false,
                b"triangles" => in_triangles = false,
                _ => {}
            },
            Ok(Event::Eof) => break,
            Err(e) => {
                return Err(Error::IO(format!("XML parse error: {}", e)));
            }
            _ => {}
        }
        buf.clear();
    }

    Ok(())
}

/// Generate 3D model XML for export
fn generate_model_xml(model: &Model) -> String {
    let mut xml = String::new();
    xml.push_str(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
"#,
    );

    for (i, obj) in model.objects.iter().enumerate() {
        xml.push_str(&format!(
            "    <object id=\"{}\" type=\"model\" name=\"{}\">\n",
            i + 1,
            obj.name
        ));
        xml.push_str("      <mesh>\n        <vertices>\n");
        for v in &obj.vertices {
            xml.push_str(&format!(
                "          <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>\n",
                v[0], v[1], v[2]
            ));
        }
        xml.push_str("        </vertices>\n        <triangles>\n");
        for t in &obj.indices {
            xml.push_str(&format!(
                "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
                t[0], t[1], t[2]
            ));
        }
        xml.push_str("        </triangles>\n      </mesh>\n    </object>\n");
    }

    xml.push_str("  </resources>\n  <build>\n");
    for (i, _) in model.objects.iter().enumerate() {
        xml.push_str(&format!("    <item objectid=\"{}\"/>\n", i + 1));
    }
    xml.push_str("  </build>\n</model>\n");

    xml
}

/// Parse config content (simple key = value format)
fn parse_config_content(content: &str, config: &mut DynamicPrintConfig) {
    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(pos) = line.find('=') {
            let key = line[..pos].trim();
            let value = line[pos + 1..].trim();
            config.set(key, value);
        }
    }
}

/// Generate config content for export
fn generate_config_content(config: &DynamicPrintConfig) -> String {
    let mut content = String::new();
    content.push_str("# BambuStudio project configuration\n");
    let mut keys: Vec<&String> = config.values.keys().collect();
    keys.sort();
    for key in keys {
        if let Some(value) = config.values.get(key) {
            content.push_str(&format!("{} = {}\n", key, value));
        }
    }
    content
}

/// Parse plate JSON data
fn parse_plate_json(content: &str) -> Result<PlateData> {
    let json: serde_json::Value = serde_json::from_str(content)
        .map_err(|e| Error::IO(format!("Failed to parse plate JSON: {}", e)))?;

    let mut plate = PlateData::new();

    if let Some(idx) = json.get("plate_index").and_then(|v| v.as_i64()) {
        plate.plate_index = idx as i32;
    }
    if let Some(name) = json.get("plate_name").and_then(|v| v.as_str()) {
        plate.plate_name = name.to_string();
    }
    if let Some(pred) = json.get("gcode_prediction").and_then(|v| v.as_str()) {
        plate.gcode_prediction = pred.to_string();
    }
    if let Some(weight) = json.get("gcode_weight").and_then(|v| v.as_str()) {
        plate.gcode_weight = weight.to_string();
    }
    if let Some(locked) = json.get("locked").and_then(|v| v.as_bool()) {
        plate.locked = locked;
    }
    if let Some(objects) = json.get("objects").and_then(|v| v.as_array()) {
        for obj in objects {
            if let (Some(oid), Some(iid)) = (
                obj.get("object_id").and_then(|v| v.as_i64()),
                obj.get("instance_id").and_then(|v| v.as_i64()),
            ) {
                plate.objects_and_instances.push((oid as i32, iid as i32));
            }
        }
    }

    Ok(plate)
}

/// Generate plate JSON for export
fn generate_plate_json(plate: &PlateData) -> String {
    let mut objects = Vec::new();
    for (oid, iid) in &plate.objects_and_instances {
        objects.push(serde_json::json!({
            "object_id": oid,
            "instance_id": iid
        }));
    }

    let json = serde_json::json!({
        "plate_index": plate.plate_index,
        "plate_name": plate.plate_name,
        "gcode_prediction": plate.gcode_prediction,
        "gcode_weight": plate.gcode_weight,
        "locked": plate.locked,
        "objects": objects
    });

    serde_json::to_string_pretty(&json).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_save_strategy_flags() {
        let s = SaveStrategy::ZIP64;
        assert!(s.contains(SaveStrategy::ZIP64));
        assert!(!s.contains(SaveStrategy::WITH_GCODE));

        let combined = SaveStrategy::ZIP64.union(SaveStrategy::WITH_GCODE);
        assert!(combined.contains(SaveStrategy::ZIP64));
        assert!(combined.contains(SaveStrategy::WITH_GCODE));
    }

    #[test]
    fn test_load_strategy_flags() {
        let s = LoadStrategy::LOAD_MODEL.union(LoadStrategy::LOAD_CONFIG);
        assert!(s.contains(LoadStrategy::LOAD_MODEL));
        assert!(s.contains(LoadStrategy::LOAD_CONFIG));
        assert!(!s.contains(LoadStrategy::SILENCE));
    }

    #[test]
    fn test_plate_data_new() {
        let pd = PlateData::new();
        assert_eq!(pd.plate_index, -1);
        assert!(!pd.locked);
    }

    #[test]
    fn test_plate_data_with_id() {
        let pd = PlateData::with_plate_id(1, &[(0, 0), (1, 0)], true);
        assert_eq!(pd.plate_index, 1);
        assert!(pd.locked);
        assert_eq!(pd.objects_and_instances.len(), 2);
    }

    #[test]
    fn test_dynamic_config() {
        let mut config = DynamicPrintConfig::new();
        config.set("layer_height", "0.2");
        assert_eq!(config.get("layer_height"), Some("0.2"));
        assert_eq!(config.get("missing"), None);
    }

    #[test]
    fn test_parse_model_xml() {
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1" type="model" name="TestCube">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0"/>
          <vertex x="1" y="0" z="0"/>
          <vertex x="0" y="1" z="0"/>
        </vertices>
        <triangles>
          <triangle v1="0" v2="1" v3="2"/>
        </triangles>
      </mesh>
    </object>
  </resources>
  <build>
    <item objectid="1"/>
  </build>
</model>"#;

        let mut model = Model::new();
        assert!(parse_model_xml(xml, &mut model).is_ok());
        assert_eq!(model.objects.len(), 1);
        assert_eq!(model.objects[0].name, "TestCube");
        assert_eq!(model.objects[0].vertices.len(), 3);
        assert_eq!(model.objects[0].indices.len(), 1);
    }

    #[test]
    fn test_release_plate_data_list() {
        let mut list = vec![PlateData::new(), PlateData::new()];
        assert_eq!(list.len(), 2);
        release_plate_data_list(&mut list);
        assert!(list.is_empty());
    }
}
