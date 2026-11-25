//! BambuStudio 3MF file format handler — faithful port.
//!
//! C++ Reference:
//! - `Format/bbs_3mf.hpp`
//! - `Format/bbs_3mf.cpp`
//!
//! Handles BambuStudio's extended 3MF format, which includes plate data,
//! print configuration, thumbnails and more.
//!
//! Porting status: the data model (`PlateData`, `SaveStrategy`, `LoadStrategy`,
//! `StoreParams`, stage constants), the file/tag/attribute constants, the
//! helper functions, the reader (`_BBS_3MF_Importer` with its `ObjectImporter`
//! and `MaterialColorResources`), the writer (`_BBS_3MF_Exporter`) and the entry
//! points (`load_bbs_3mf`, `store_bbs_3mf`, `bbs_3mf_get_thumbnail`,
//! `is_bbs_project_3mf`, `load_gcode_3mf_from_stream`) follow bbs_3mf.cpp at the
//! pin. miniz, expat and boost::property_tree are replaced by the stand-ins of
//! the section "miniz / expat / boost::property_tree stand-ins" (see the
//! FIDELITY-NOTEs there). The GUI's background backup thread
//! (`_BBS_Backup_Manager`) is not part of the command line engine.
//!
//! This reader is not the loader of the slicing path (`app_slice::load_3mf`); the
//! G-code of the fixtures does not depend on it.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::io::{Read, Seek, Write as IoWrite};
use std::sync::Mutex;

use crate::print_config::config_def::DynamicPrintConfig;
use crate::format::obj::VolumeColorInfo;
use crate::format::objparser;
use crate::gcode::g_code_processor::SliceWarning;
use crate::gcode::thumbnail_data::{PlateBBoxData, ThumbnailData};
use crate::geometry::geometry::{Transform3d, Vec3d};
use crate::locales_utils::general_format;
use crate::geometry::geometry::Transformation;
use crate::model::{Model, ModelInstance, ModelVolume, ModelVolumeType};
use crate::normal_utils::indexed_triangle_set;
use crate::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleInfo};
use crate::normal_utils::Vec3f;
use crate::preset::Preset;
use crate::project_task::{BBLProfile, BBLProject, FilamentInfo};
use crate::utils::decode_path;

// ===========================================================================
// bbs_3mf.hpp
// ===========================================================================

// bbs_3mf.hpp:22-23
pub const PLATE_THUMBNAIL_SMALL_WIDTH: u32 = 128;
pub const PLATE_THUMBNAIL_SMALL_HEIGHT: u32 = 128;

// bbs_3mf.hpp:25-34 — boost::format patterns ("%1%" is the plate index).
pub const GCODE_FILE_FORMAT: &str = "Metadata/plate_%1%.gcode";
pub const THUMBNAIL_FILE_FORMAT: &str = "Metadata/plate_%1%.png";
pub const NO_LIGHT_THUMBNAIL_FILE_FORMAT: &str = "Metadata/plate_no_light_%1%.png";
pub const TOP_FILE_FORMAT: &str = "Metadata/top_%1%.png";
pub const PICK_FILE_FORMAT: &str = "Metadata/pick_%1%.png";
//pub const PATTERN_FILE_FORMAT: &str = "Metadata/plate_%1%_pattern_layer_0.png"; // bbs_3mf.hpp:30 (commented out)
pub const PATTERN_CONFIG_FILE_FORMAT: &str = "Metadata/plate_%1%.json";
pub const EMBEDDED_PRINT_FILE_FORMAT: &str = "Metadata/process_settings_%1%.config";
pub const EMBEDDED_FILAMENT_FILE_FORMAT: &str = "Metadata/filament_settings_%1%.config";
pub const EMBEDDED_PRINTER_FILE_FORMAT: &str = "Metadata/machine_settings_%1%.config";

// bbs_3mf.hpp:36-39
pub const BBL_DESIGNER_MODEL_TITLE_TAG: &str = "Title";
pub const BBL_DESIGNER_PROFILE_ID_TAG: &str = "DesignProfileId";
pub const BBL_DESIGNER_PROFILE_TITLE_TAG: &str = "ProfileTitle";
pub const BBL_DESIGNER_MODEL_ID_TAG: &str = "DesignModelId";

/// BBS: define assistant struct to store temporary variable during exporting 3mf
/// bbs_3mf.hpp:43-51  class PackingTemporaryData
#[derive(Debug, Clone, Default)]
pub struct PackingTemporaryData {
    /// bbs_3mf.hpp:46  std::string _3mf_thumbnail;
    pub _3mf_thumbnail: String,
    /// bbs_3mf.hpp:47  std::string _3mf_printer_thumbnail_middle;
    pub _3mf_printer_thumbnail_middle: String,
    /// bbs_3mf.hpp:48  std::string _3mf_printer_thumbnail_small;
    pub _3mf_printer_thumbnail_small: String,
}

impl PackingTemporaryData {
    /// bbs_3mf.hpp:50  PackingTemporaryData() {}
    pub fn new() -> Self {
        Self::default()
    }
}

/// bbs_3mf.hpp:103  using LayerFilaments = std::unordered_map<std::vector<unsigned int>,
///   std::vector<std::pair<int, int>>, GCodeProcessorResult::FilamentSequenceHash>;
/// (Rust uses the default hasher — identical map semantics; see
/// `gcode::g_code_processor::filament_sequence_hash` for the C++ hash functor.)
pub type LayerFilaments = HashMap<Vec<u32>, Vec<(i32, i32)>>;

/// BBS: per-AMS filament load/unload time, exported so the print start flow can replace the
/// estimated time with the actual AMS time according to the real AMS mapping.
/// bbs_3mf.hpp:55-61  struct AmsLoadUnloadTimeInfo
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AmsLoadUnloadTimeInfo {
    pub ams_type: String,
    pub load_time: f64,
    pub unload_time: f64,
}

/// Mixed (virtual) filament used by a plate. Mixed filaments are virtual slots that get
/// resolved to their physical components before g-code statistics, so they never appear in
/// slice_filaments_info. They are recorded here separately so a plate's mixed-color usage
/// can be recovered from slice_info.
/// bbs_3mf.hpp:67-73  struct PlateMixedFilamentInfo
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlateMixedFilamentInfo {
    /// 1-based virtual filament slot id
    pub id: i32,
    pub type_: String,
    /// blended display color, "#RRGGBB"
    pub color: String,
    /// 1-based physical component ids, comma separated, e.g. "1,3"
    pub components: String,
}

/// GCodeProcessor.hpp:261-266 `struct GCodeProcessorResult::PausePrintInfo`.
/// FIDELITY-NOTE: declared here because `gcode::g_code_processor` (another unit) does not carry it
/// yet; the fields and defaults are the C++ ones.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PausePrintInfo {
    pub percent: i32,
    /// min
    pub remaining_time: i32,
    pub layer_id: u32,
}

/// The fields of ProjectTask.hpp:40-71 `struct FilamentInfo` the crate's
/// `project_task::FilamentInfo` predates (`directly_used`, `total_load_time`, `total_unload_time`).
/// FIDELITY-NOTE: `project_task.rs` is another unit; until it carries them, the reader and the
/// writer of slice_info.config keep them beside each `FilamentInfo` in
/// [`PlateData::slice_filaments_extra`], index for index.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilamentInfoExtra {
    /// ProjectTask.hpp:60 `bool directly_used{true};`
    pub directly_used: bool,
    /// ProjectTask.hpp:61 `double total_load_time{0.0};`
    pub total_load_time: f64,
    /// ProjectTask.hpp:62 `double total_unload_time{0.0};`
    pub total_unload_time: f64,
}

impl Default for FilamentInfoExtra {
    fn default() -> Self {
        Self { directly_used: true, total_load_time: 0.0, total_unload_time: 0.0 }
    }
}

/// BBS: define plate data list related structures
/// bbs_3mf.hpp:75-163  struct PlateData
#[derive(Clone)]
pub struct PlateData {
    /// bbs_3mf.hpp:74  int plate_index;
    pub plate_index: i32,
    /// bbs_3mf.hpp:75  std::vector<std::pair<int, int>> objects_and_instances;
    pub objects_and_instances: Vec<(i32, i32)>,
    /// bbs_3mf.hpp:76  std::map<int, std::pair<int, int>> obj_inst_map;
    pub obj_inst_map: BTreeMap<i32, (i32, i32)>,
    /// bbs_3mf.hpp:77
    pub printer_model_id: String,
    /// bbs_3mf.hpp:78
    pub nozzle_diameters: String,
    /// bbs_3mf.hpp:79
    pub nozzle_volume_types: String,
    /// bbs_3mf.hpp:80
    pub gcode_file: String,
    /// bbs_3mf.hpp:81
    pub gcode_file_md5: String,
    /// bbs_3mf.hpp:82
    pub thumbnail_file: String,
    /// bbs_3mf.hpp:83
    pub no_light_thumbnail_file: String,
    /// bbs_3mf.hpp:84  ThumbnailData plate_thumbnail;
    pub plate_thumbnail: ThumbnailData,
    /// bbs_3mf.hpp:85
    pub top_file: String,
    /// bbs_3mf.hpp:86
    pub pick_file: String,
    // bbs_3mf.hpp:87-88  pattern_thumbnail / pattern_file (commented out in C++)
    /// bbs_3mf.hpp:89
    pub pattern_bbox_file: String,
    /// bbs_3mf.hpp:90
    pub gcode_prediction: String,
    /// bbs_3mf.hpp:91
    pub gcode_weight: String,
    /// bbs_3mf.hpp:92
    pub first_layer_time: String,
    /// bbs_3mf.hpp:93
    pub plate_name: String,
    /// bbs_3mf.hpp:94  std::vector<FilamentInfo> slice_filaments_info;
    pub slice_filaments_info: Vec<FilamentInfo>,
    /// See [`FilamentInfoExtra`]: the newer `FilamentInfo` fields of `slice_filaments_info`, index
    /// for index (empty when no entry carries them).
    pub slice_filaments_extra: Vec<FilamentInfoExtra>,
    /// bbs_3mf.hpp:115 `std::vector<PlateMixedFilamentInfo> mixed_filaments_info;`
    pub mixed_filaments_info: Vec<PlateMixedFilamentInfo>,
    /// bbs_3mf.hpp:95  std::vector<size_t> skipped_objects;
    pub skipped_objects: Vec<usize>,
    /// bbs_3mf.hpp:118 `std::string default_ams_type;`
    pub default_ams_type: String,
    /// bbs_3mf.hpp:119 `std::vector<AmsLoadUnloadTimeInfo> ams_list;`
    pub ams_list: Vec<AmsLoadUnloadTimeInfo>,
    /// bbs_3mf.hpp:96  DynamicPrintConfig config;
    pub config: DynamicPrintConfig,
    /// bbs_3mf.hpp:97  bool is_support_used {false};
    pub is_support_used: bool,
    /// bbs_3mf.hpp:98  bool is_sliced_valid = false;
    pub is_sliced_valid: bool,
    /// bbs_3mf.hpp:99  bool toolpath_outside {false};
    pub toolpath_outside: bool,
    /// bbs_3mf.hpp:100  bool is_label_object_enabled {false};
    pub is_label_object_enabled: bool,
    /// bbs_3mf.hpp:125 `bool support_material_on_wipe_tower {false};`
    pub support_material_on_wipe_tower: bool,
    /// bbs_3mf.hpp:101  int timelapse_warning_code = 0; // 1<<0 sprial vase, 1<<1 by object
    pub timelapse_warning_code: i32,
    /// bbs_3mf.hpp:102  std::vector<int> filament_maps;   // 1 base
    pub filament_maps: Vec<i32>,
    /// bbs_3mf.hpp:104  LayerFilaments layer_filaments;
    pub layer_filaments: LayerFilaments,
    /// bbs_3mf.hpp:105  std::vector<unsigned int> filament_change_sequence;
    pub filament_change_sequence: Vec<u32>,
    /// bbs_3mf.hpp:106  std::vector<unsigned int> nozzle_change_sequence;
    pub nozzle_change_sequence: Vec<u32>,
    /// bbs_3mf.hpp:107  std::vector<int> optimal_assignment;
    pub optimal_assignment: Vec<i32>,
    /// bbs_3mf.hpp:134 `std::vector<GCodeProcessorResult::PausePrintInfo> pause_printing;`
    pub pause_printing: Vec<PausePrintInfo>,
    /// bbs_3mf.hpp:108  std::optional<MultiNozzleUtils::LayeredNozzleGroupResult> nozzle_group_result;
    pub nozzle_group_result: Option<LayeredNozzleGroupResult>,
    /// bbs_3mf.hpp:109-114  Hexadecimal number,
    /// the 0th digit corresponds to extruder 1
    /// the 1th digit corresponds to extruder 2
    /// ...  and so on.
    /// 0 means can be print on this extruder, 1 means cannot
    pub limit_filament_maps: Vec<i32>,
    /// bbs_3mf.hpp:116  std::vector<GCodeProcessorResult::SliceWarning> warnings;
    pub warnings: Vec<SliceWarning>,
    /// bbs_3mf.hpp:118-119  喷嘴信息列表，用于多喷嘴打印
    /// std::vector<MultiNozzleUtils::NozzleInfo> nozzles_info;
    pub nozzles_info: Vec<NozzleInfo>,
    /// bbs_3mf.hpp:128  bool locked;
    pub locked: bool,
}

impl PlateData {
    /// bbs_3mf.hpp:57-62
    /// `PlateData(int plate_id, std::set<std::pair<int, int>> &obj_to_inst_list, bool lock_state)`
    pub fn with_plate_id(
        plate_id: i32,
        obj_to_inst_list: &BTreeSet<(i32, i32)>,
        lock_state: bool,
    ) -> Self {
        let mut this = PlateData::new();
        this.plate_index = plate_id;
        this.locked = lock_state;
        // bbs_3mf.hpp:59-61
        this.objects_and_instances.clear();
        for it in obj_to_inst_list.iter() {
            this.objects_and_instances.push((it.0, it.1));
        }
        this
    }

    /// bbs_3mf.hpp:63-66  PlateData() : plate_index(-1), locked(false)
    pub fn new() -> Self {
        PlateData {
            plate_index: -1,
            objects_and_instances: Vec::new(),
            obj_inst_map: BTreeMap::new(),
            printer_model_id: String::new(),
            nozzle_diameters: String::new(),
            nozzle_volume_types: String::new(),
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
            slice_filaments_extra: Vec::new(),
            mixed_filaments_info: Vec::new(),
            skipped_objects: Vec::new(),
            default_ams_type: String::new(),
            ams_list: Vec::new(),
            config: DynamicPrintConfig::default(),
            is_support_used: false,
            is_sliced_valid: false,
            toolpath_outside: false,
            is_label_object_enabled: false,
            support_material_on_wipe_tower: false,
            timelapse_warning_code: 0,
            filament_maps: Vec::new(),
            layer_filaments: LayerFilaments::new(),
            filament_change_sequence: Vec::new(),
            nozzle_change_sequence: Vec::new(),
            optimal_assignment: Vec::new(),
            pause_printing: Vec::new(),
            nozzle_group_result: None,
            limit_filament_maps: Vec::new(),
            warnings: Vec::new(),
            nozzles_info: Vec::new(),
            locked: false,
        }
    }

    /// bbs_3mf.hpp:121-123  std::string get_gcode_prediction_str()
    pub fn get_gcode_prediction_str(&self) -> String {
        self.gcode_prediction.clone()
    }

    /// bbs_3mf.hpp:125-127  std::string get_gcode_weight_str()
    pub fn get_gcode_weight_str(&self) -> String {
        self.gcode_weight.clone()
    }
}

impl Default for PlateData {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for PlateData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PlateData")
            .field("plate_index", &self.plate_index)
            .field("objects_and_instances", &self.objects_and_instances)
            .field("obj_inst_map", &self.obj_inst_map)
            .field("gcode_file", &self.gcode_file)
            .field("locked", &self.locked)
            .finish()
    }
}

/// BBS: encrypt
/// bbs_3mf.hpp:132-151  enum class SaveStrategy
/// (kept as a transparent bit-flag newtype so the C++ flag arithmetic —
/// `SplitModel = 0x1000 | ProductionExt` etc. — carries over verbatim)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SaveStrategy(pub u32);

#[allow(non_upper_case_globals)]
impl SaveStrategy {
    pub const Default: SaveStrategy = SaveStrategy(0); // bbs_3mf.hpp:134
    pub const FullPathSources: SaveStrategy = SaveStrategy(1); // bbs_3mf.hpp:135
    pub const Zip64: SaveStrategy = SaveStrategy(1 << 1); // bbs_3mf.hpp:136
    pub const ProductionExt: SaveStrategy = SaveStrategy(1 << 2); // bbs_3mf.hpp:137
    pub const SecureContentExt: SaveStrategy = SaveStrategy(1 << 3); // bbs_3mf.hpp:138
    pub const WithGcode: SaveStrategy = SaveStrategy(1 << 4); // bbs_3mf.hpp:139
    pub const Silence: SaveStrategy = SaveStrategy(1 << 5); // bbs_3mf.hpp:140
    pub const SkipStatic: SaveStrategy = SaveStrategy(1 << 6); // bbs_3mf.hpp:141
    pub const SkipModel: SaveStrategy = SaveStrategy(1 << 7); // bbs_3mf.hpp:142
    pub const WithSliceInfo: SaveStrategy = SaveStrategy(1 << 8); // bbs_3mf.hpp:143
    pub const SkipAuxiliary: SaveStrategy = SaveStrategy(1 << 9); // bbs_3mf.hpp:144
    pub const UseLoadedId: SaveStrategy = SaveStrategy(1 << 10); // bbs_3mf.hpp:145
    pub const ShareMesh: SaveStrategy = SaveStrategy(1 << 11); // bbs_3mf.hpp:146

    /// bbs_3mf.hpp:148  SplitModel = 0x1000 | ProductionExt,
    pub const SplitModel: SaveStrategy = SaveStrategy(0x1000 | Self::ProductionExt.0);
    /// bbs_3mf.hpp:149  Encrypted = SecureContentExt | SplitModel,
    pub const Encrypted: SaveStrategy =
        SaveStrategy(Self::SecureContentExt.0 | Self::SplitModel.0);
    /// bbs_3mf.hpp:150  Backup = 0x10000 | WithGcode | Silence | SkipStatic | SplitModel,
    pub const Backup: SaveStrategy = SaveStrategy(
        0x10000 | Self::WithGcode.0 | Self::Silence.0 | Self::SkipStatic.0 | Self::SplitModel.0,
    );

    /// bbs_3mf.hpp:159-163  inline bool operator & (SaveStrategy & lhs, SaveStrategy rhs)
    /// `((lhs & rhs)) == rhs`
    pub fn contains(self, rhs: SaveStrategy) -> bool {
        (self.0 & rhs.0) == rhs.0
    }
}

/// bbs_3mf.hpp:153-157  inline SaveStrategy operator | (SaveStrategy lhs, SaveStrategy rhs)
impl std::ops::BitOr for SaveStrategy {
    type Output = SaveStrategy;
    fn bitor(self, rhs: SaveStrategy) -> SaveStrategy {
        SaveStrategy(self.0 | rhs.0)
    }
}

/// bbs_3mf.hpp:165-167  enum { brim_points_format_version = 1 };
pub const BRIM_POINTS_FORMAT_VERSION: i32 = 1;

/// bbs_3mf.hpp:169-181  enum class LoadStrategy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoadStrategy(pub u32);

#[allow(non_upper_case_globals)]
impl LoadStrategy {
    pub const Default: LoadStrategy = LoadStrategy(0); // bbs_3mf.hpp:171
    pub const AddDefaultInstances: LoadStrategy = LoadStrategy(1); // bbs_3mf.hpp:172
    pub const CheckVersion: LoadStrategy = LoadStrategy(2); // bbs_3mf.hpp:173
    pub const LoadModel: LoadStrategy = LoadStrategy(4); // bbs_3mf.hpp:174
    pub const LoadConfig: LoadStrategy = LoadStrategy(8); // bbs_3mf.hpp:175
    pub const LoadAuxiliary: LoadStrategy = LoadStrategy(16); // bbs_3mf.hpp:176
    pub const Silence: LoadStrategy = LoadStrategy(32); // bbs_3mf.hpp:177
    pub const ImperialUnits: LoadStrategy = LoadStrategy(64); // bbs_3mf.hpp:178

    /// bbs_3mf.hpp:180  Restore = 0x10000 | LoadModel | LoadConfig | LoadAuxiliary | Silence,
    pub const Restore: LoadStrategy = LoadStrategy(
        0x10000 | Self::LoadModel.0 | Self::LoadConfig.0 | Self::LoadAuxiliary.0 | Self::Silence.0,
    );

    /// bbs_3mf.hpp:189-193  inline bool operator & (LoadStrategy & lhs, LoadStrategy rhs)
    pub fn contains(self, rhs: LoadStrategy) -> bool {
        (self.0 & rhs.0) == rhs.0
    }
}

/// bbs_3mf.hpp:183-187  inline LoadStrategy operator | (LoadStrategy lhs, LoadStrategy rhs)
impl std::ops::BitOr for LoadStrategy {
    type Output = LoadStrategy;
    fn bitor(self, rhs: LoadStrategy) -> LoadStrategy {
        LoadStrategy(self.0 | rhs.0)
    }
}

// bbs_3mf.hpp:195-209 — BBS export 3mf progress stages
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

// bbs_3mf.hpp:211-224 — import stages
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

//BBS export 3mf progress
/// bbs_3mf.hpp:227  typedef std::function<void(int export_stage, int current, int total, bool& cancel)> Export3mfProgressFn;
pub type Export3mfProgressFn = Box<dyn Fn(i32, i32, i32, &mut bool)>;
/// bbs_3mf.hpp:228  typedef std::function<void(int import_stage, int current, int total, bool& cancel)> Import3mfProgressFn;
pub type Import3mfProgressFn = Box<dyn Fn(i32, i32, i32, &mut bool)>;

/// bbs_3mf.hpp:230  typedef std::vector<PlateData*> PlateDataPtrs;
/// (`PlateData` values are owned directly in Rust.)
pub type PlateDataPtrs = Vec<PlateData>;

/// bbs_3mf.hpp:232  typedef std::map<int, PlateData*> PlateDataMaps;
pub type PlateDataMaps = BTreeMap<i32, PlateData>;

/// 3MF color data: Stores color information for all volumes.
/// key: Volume index (ID index in ModelObject)
/// bbs_3mf.hpp:236  typedef std::unordered_map<int, VolumeColorInfo> VolumeColorInfoMap;
pub type VolumeColorInfoMap = HashMap<i32, VolumeColorInfo>;

/// bbs_3mf.hpp:238-258  struct StoreParams
/// (C++ stores raw pointers; the Rust port owns the values — `Option<T>` for
/// nullable pointers, `Vec<T>` for pointer vectors.)
pub struct StoreParams {
    /// bbs_3mf.hpp:240  const char* path;
    pub path: String,
    /// bbs_3mf.hpp:241  Model* model = nullptr;
    pub model: Option<Model>,
    /// bbs_3mf.hpp:242  PlateDataPtrs plate_data_list;
    pub plate_data_list: PlateDataPtrs,
    /// bbs_3mf.hpp:243  int export_plate_idx = -1;
    pub export_plate_idx: i32,
    /// bbs_3mf.hpp:244  std::vector<Preset*> project_presets;
    pub project_presets: Vec<Preset>,
    /// bbs_3mf.hpp:245  DynamicPrintConfig* config;
    pub config: Option<DynamicPrintConfig>,
    /// bbs_3mf.hpp:246  std::vector<ThumbnailData*> thumbnail_data;
    pub thumbnail_data: Vec<ThumbnailData>,
    /// bbs_3mf.hpp:247  std::vector<ThumbnailData*> no_light_thumbnail_data;
    pub no_light_thumbnail_data: Vec<ThumbnailData>,
    /// bbs_3mf.hpp:248  std::vector<ThumbnailData*> top_thumbnail_data;
    pub top_thumbnail_data: Vec<ThumbnailData>,
    /// bbs_3mf.hpp:249  std::vector<ThumbnailData*> pick_thumbnail_data;
    pub pick_thumbnail_data: Vec<ThumbnailData>,
    /// bbs_3mf.hpp:250  std::vector<ThumbnailData*> calibration_thumbnail_data;
    pub calibration_thumbnail_data: Vec<ThumbnailData>,
    /// bbs_3mf.hpp:251  SaveStrategy strategy = SaveStrategy::Zip64;
    pub strategy: SaveStrategy,
    /// bbs_3mf.hpp:252  Export3mfProgressFn proFn = nullptr;
    pub pro_fn: Option<Export3mfProgressFn>,
    /// bbs_3mf.hpp:253  std::vector<PlateBBoxData*> id_bboxes;
    pub id_bboxes: Vec<PlateBBoxData>,
    /// bbs_3mf.hpp:254  BBLProject* project = nullptr;
    pub project: Option<BBLProject>,
    /// bbs_3mf.hpp:255  BBLProfile* profile = nullptr;
    pub profile: Option<BBLProfile>,
    /// bbs_3mf.hpp:257 `bool qualified_assembly = false;` — written into
    /// Metadata/assembly_step.json when the saved project is a qualified assembly.
    pub qualified_assembly: bool,
}

impl StoreParams {
    /// bbs_3mf.hpp:257  StoreParams() {}
    pub fn new() -> Self {
        StoreParams {
            path: String::new(),
            model: None,
            plate_data_list: PlateDataPtrs::new(),
            export_plate_idx: -1,
            project_presets: Vec::new(),
            config: None,
            thumbnail_data: Vec::new(),
            no_light_thumbnail_data: Vec::new(),
            top_thumbnail_data: Vec::new(),
            pick_thumbnail_data: Vec::new(),
            calibration_thumbnail_data: Vec::new(),
            strategy: SaveStrategy::Zip64,
            pro_fn: None,
            id_bboxes: Vec::new(),
            project: None,
            profile: None,
            qualified_assembly: false,
        }
    }
}

impl Default for StoreParams {
    fn default() -> Self {
        Self::new()
    }
}

// bbs_3mf.hpp:320-324  class SaveObjectGaurd — the RAII guard of the GUI backup thread
// (`_BBS_Backup_Manager`), which is not part of the command line engine.

/// Compatibility shim: in C++ `ConfigSubstitutionContext` lives in
/// `libslic3r/Config.hpp` (rule + ConfigSubstitutions). The reflective config
/// layer is not yet ported, so the existing minimal context (used by
/// `format::amf` and `format::three_mf`) is kept here until `config.rs` gains
/// the faithful type.
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

impl Default for ConfigSubstitutionContext {
    fn default() -> Self {
        Self::new()
    }
}

// ===========================================================================
// bbs_3mf.cpp
// ===========================================================================

// bbs_3mf.cpp:57-61 — Slightly faster than sprintf("%.9g"), but there is an
// issue with the karma floating point formatter,
// https://github.com/boostorg/spirit/pull/586
// where the exported string is one digit shorter than it should be to
// guarantee lossless round trip. The code is left here for the ocasion boost
// guys improve.
pub const EXPORT_3MF_USE_SPIRIT_KARMA_FP: i32 = 0;

// bbs_3mf.cpp:63
pub const WRITE_ZIP_LANGUAGE_ENCODING: i32 = 1;

/// bbs_3mf.cpp:65-98  struct ZipUnicodePathExtraField
/// @see https://commons.apache.org/proper/commons-compress/apidocs/src-html/org/apache/commons/compress/archivers/zip/AbstractUnicodeExtraField.html
pub struct ZipUnicodePathExtraField;

impl ZipUnicodePathExtraField {
    /// bbs_3mf.cpp:68-83  static std::string encode(std::string const& u8path, std::string const& path)
    pub fn encode(u8path: &str, path: &str) -> Vec<u8> {
        let mut extra: Vec<u8> = Vec::new();
        if u8path != path {
            // 0x7075 - for Unicode filenames
            extra.push(0x75); // bbs_3mf.cpp:72
            extra.push(0x70); // bbs_3mf.cpp:73
            // bbs_3mf.cpp:74  boost::uint16_t len = 5 + u8path.length();
            let len: u16 = (5 + u8path.len()) as u16;
            extra.push((len & 0xff) as u8); // bbs_3mf.cpp:75
            extra.push((len >> 8) as u8); // bbs_3mf.cpp:76
            // bbs_3mf.cpp:77  auto crc = mz_crc32(0, path, path.length());
            let mut crc = flate2::Crc::new();
            crc.update(path.as_bytes());
            let crc = crc.sum();
            extra.push(0x01); // version 1  bbs_3mf.cpp:78
            // bbs_3mf.cpp:79  Little Endian
            extra.extend_from_slice(&crc.to_le_bytes());
            extra.extend_from_slice(u8path.as_bytes()); // bbs_3mf.cpp:80
        }
        extra
    }

    /// bbs_3mf.cpp:84-97  static std::string decode(std::string const& extra, std::string const& path = {})
    pub fn decode(extra: &[u8], path: &str) -> String {
        // bbs_3mf.cpp:85-86
        let mut p = 0usize;
        let e = extra.len();
        // bbs_3mf.cpp:87
        while p + 4 < e {
            // bbs_3mf.cpp:88 — `((boost::uint16_t)p[2]) | ((boost::uint16_t)p[3] << 8)`
            // NOTE: `char` is signed in the C++ build, so both bytes
            // sign-extend through the integer conversions; replicated exactly.
            let b2 = extra[p + 2] as i8 as i32; // sign-extended
            let b3 = extra[p + 3] as i8 as i32; // sign-extended
            let len = ((b2 & 0xffff) | (b3 << 8)) as u16 as usize;
            // bbs_3mf.cpp:89
            if extra[p] == 0x75 && extra[p + 1] == 0x70 && len >= 5 && p + 4 + len < e
                && extra[p + 4] == 0x01
            {
                // bbs_3mf.cpp:90  return std::string(p + 9, p + 4 + len);
                return String::from_utf8_lossy(&extra[p + 9..p + 4 + len]).into_owned();
            } else {
                // bbs_3mf.cpp:93
                p += 4 + len;
            }
        }
        // bbs_3mf.cpp:96
        decode_path(path)
    }
}

// VERSION NUMBERS
// 0 : .3mf, files saved by older slic3r or other applications. No version definition in them.
// 1 : Introduction of 3mf versioning. No other change in data saved into 3mf files.
// 2 : Volumes' matrices and source data added to Metadata/Slic3r_PE_model.config file, meshes transformed back to their coordinate system on loading.
// WARNING !! -> the version number has been rolled back to 1
//               the next change should use 3
pub const VERSION_BBS_3MF: u32 = 1; // bbs_3mf.cpp:106
/// Allow loading version 2 file as well. bbs_3mf.cpp:108
pub const VERSION_BBS_3MF_COMPATIBLE: u32 = 2;
/// definition of the metadata name saved into .model file  bbs_3mf.cpp:109
pub const BBS_3MF_VERSION1: &str = "bamboo_slicer:Version3mf";
/// compatible with prusa currently  bbs_3mf.cpp:110
pub const BBS_3MF_VERSION: &str = "BambuStudio:3mfVersion";

// Painting gizmos data version numbers
// 0 : initial version of fdm, seam, mm
pub const FDM_SUPPORTS_PAINTING_VERSION: u32 = 0; // bbs_3mf.cpp:113
pub const SEAM_PAINTING_VERSION: u32 = 0; // bbs_3mf.cpp:114
pub const MM_PAINTING_VERSION: u32 = 0; // bbs_3mf.cpp:115

pub const BBS_FDM_SUPPORTS_PAINTING_VERSION: &str = "BambuStudio:FdmSupportsPaintingVersion"; // bbs_3mf.cpp:117
pub const BBS_SEAM_PAINTING_VERSION: &str = "BambuStudio:SeamPaintingVersion"; // bbs_3mf.cpp:118
pub const BBS_MM_PAINTING_VERSION: &str = "BambuStudio:MmPaintingVersion"; // bbs_3mf.cpp:119
pub const BBL_MODEL_ID_TAG: &str = "model_id"; // bbs_3mf.cpp:120
pub const BBL_MODEL_NAME_TAG: &str = "Title"; // bbs_3mf.cpp:121
pub const BBL_ORIGIN_TAG: &str = "Origin"; // bbs_3mf.cpp:122
pub const BBL_DESIGNER_TAG: &str = "Designer"; // bbs_3mf.cpp:123
pub const BBL_DESIGNER_USER_ID_TAG: &str = "DesignerUserId"; // bbs_3mf.cpp:124
pub const BBL_DESIGN_ID_TAG: &str = "DesignId"; // bbs_3mf.cpp:125
//pub const BBL_DESIGNER_MODEL_ID_TAG: &str = "DesignModelId"; // bbs_3mf.cpp:126 (commented out; lives in bbs_3mf.hpp:39)
pub const BBL_DESIGNER_COVER_FILE_TAG: &str = "DesignerCover"; // bbs_3mf.cpp:127
pub const BBL_DESCRIPTION_TAG: &str = "Description"; // bbs_3mf.cpp:128
pub const BBL_COPYRIGHT_TAG: &str = "CopyRight"; // bbs_3mf.cpp:129
pub const BBL_COPYRIGHT_NORMATIVE_TAG: &str = "Copyright"; // bbs_3mf.cpp:130
pub const BBL_LICENSE_TAG: &str = "License"; // bbs_3mf.cpp:131
pub const BBL_REGION_TAG: &str = "Region"; // bbs_3mf.cpp:132
pub const BBL_MODIFICATION_TAG: &str = "ModificationDate"; // bbs_3mf.cpp:133
pub const BBL_CREATION_DATE_TAG: &str = "CreationDate"; // bbs_3mf.cpp:134
pub const BBL_APPLICATION_TAG: &str = "Application"; // bbs_3mf.cpp:135
pub const BBL_MAKERLAB_TAG: &str = "MakerLab"; // bbs_3mf.cpp:136
pub const BBL_MAKERLAB_VERSION_TAG: &str = "MakerLabVersion"; // bbs_3mf.cpp:137

pub const BBL_MAKERLAB_NAME: &str = "MakerLab"; // bbs_3mf.cpp:139
pub const BBL_MAKERLAB_REGION: &str = "MakerLabRegion"; // bbs_3mf.cpp:140
pub const BBL_MAKERLAB_ID: &str = "MakerLabFileId"; // bbs_3mf.cpp:141

pub const BBL_PROFILE_TITLE_TAG: &str = "ProfileTitle"; // bbs_3mf.cpp:144
pub const BBL_PROFILE_COVER_TAG: &str = "ProfileCover"; // bbs_3mf.cpp:145
pub const BBL_PROFILE_DESCRIPTION_TAG: &str = "ProfileDescription"; // bbs_3mf.cpp:146
pub const BBL_PROFILE_USER_ID_TAG: &str = "ProfileUserId"; // bbs_3mf.cpp:147
pub const BBL_PROFILE_USER_NAME_TAG: &str = "ProfileUserName"; // bbs_3mf.cpp:148

pub const MODEL_FOLDER: &str = "3D/"; // bbs_3mf.cpp:150
pub const MODEL_EXTENSION: &str = ".model"; // bbs_3mf.cpp:151
/// << this is the only format of the string which works with CURA  bbs_3mf.cpp:152
pub const MODEL_FILE: &str = "3D/3dmodel.model";
pub const MODEL_RELS_FILE: &str = "3D/_rels/3dmodel.model.rels"; // bbs_3mf.cpp:153
//BBS: add metadata_folder
pub const METADATA_DIR: &str = "Metadata/"; // bbs_3mf.cpp:155
pub const ACCESOR_DIR: &str = "accesories/"; // bbs_3mf.cpp:156
pub const GCODE_EXTENSION: &str = ".gcode"; // bbs_3mf.cpp:157
pub const THUMBNAIL_EXTENSION: &str = ".png"; // bbs_3mf.cpp:158
pub const CALIBRATION_INFO_EXTENSION: &str = ".json"; // bbs_3mf.cpp:159
pub const CONTENT_TYPES_FILE: &str = "[Content_Types].xml"; // bbs_3mf.cpp:160
pub const RELATIONSHIPS_FILE: &str = "_rels/.rels"; // bbs_3mf.cpp:161
pub const THUMBNAIL_FILE: &str = "Metadata/plate_1.png"; // bbs_3mf.cpp:162
pub const THUMBNAIL_FOR_PRINTER_FILE: &str = "Metadata/bbl_thumbnail.png"; // bbs_3mf.cpp:163
pub const PRINTER_THUMBNAIL_SMALL_FILE: &str = "/Auxiliaries/.thumbnails/thumbnail_small.png"; // bbs_3mf.cpp:164
pub const PRINTER_THUMBNAIL_MIDDLE_FILE: &str = "/Auxiliaries/.thumbnails/thumbnail_middle.png"; // bbs_3mf.cpp:165
pub const _3MF_COVER_FILE: &str = "/Auxiliaries/.thumbnails/thumbnail_3mf.png"; // bbs_3mf.cpp:166
//pub const PRINT_CONFIG_FILE: &str = "Metadata/Slic3r_PE.config"; // bbs_3mf.cpp:167 (commented out)
//pub const MODEL_CONFIG_FILE: &str = "Metadata/Slic3r_PE_model.config"; // bbs_3mf.cpp:168 (commented out)
pub const BBS_PRINT_CONFIG_FILE: &str = "Metadata/print_profile.config"; // bbs_3mf.cpp:169
pub const BBS_PROJECT_CONFIG_FILE: &str = "Metadata/project_settings.config"; // bbs_3mf.cpp:170
pub const BBS_MODEL_CONFIG_FILE: &str = "Metadata/model_settings.config"; // bbs_3mf.cpp:171
pub const BBS_MODEL_CONFIG_RELS_FILE: &str = "Metadata/_rels/model_settings.config.rels"; // bbs_3mf.cpp:172
pub const SLICE_INFO_CONFIG_FILE: &str = "Metadata/slice_info.config"; // bbs_3mf.cpp:173
pub const FILAMENT_SEQUENCE_FILE: &str = "Metadata/filament_sequence.json"; // bbs_3mf.cpp:174
pub const BBS_LAYER_HEIGHTS_PROFILE_FILE: &str = "Metadata/layer_heights_profile.txt"; // bbs_3mf.cpp:175
pub const LAYER_CONFIG_RANGES_FILE: &str = "Metadata/layer_config_ranges.xml"; // bbs_3mf.cpp:176
pub const BRIM_EAR_POINTS_FILE: &str = "Metadata/brim_ear_points.txt"; // bbs_3mf.cpp:177
/*pub const SLA_SUPPORT_POINTS_FILE: &str = "Metadata/Slic3r_PE_sla_support_points.txt"; // bbs_3mf.cpp:178 (commented out)
pub const SLA_DRAIN_HOLES_FILE: &str = "Metadata/Slic3r_PE_sla_drain_holes.txt"; // bbs_3mf.cpp:179 (commented out)*/
pub const CUSTOM_GCODE_PER_PRINT_Z_FILE: &str = "Metadata/custom_gcode_per_layer.xml"; // bbs_3mf.cpp:180
pub const AUXILIARY_DIR: &str = "Auxiliaries/"; // bbs_3mf.cpp:181
pub const PROJECT_EMBEDDED_PRINT_PRESETS_FILE: &str = "Metadata/print_setting_"; // bbs_3mf.cpp:182
pub const PROJECT_EMBEDDED_SLICE_PRESETS_FILE: &str = "Metadata/process_settings_"; // bbs_3mf.cpp:183
pub const PROJECT_EMBEDDED_FILAMENT_PRESETS_FILE: &str = "Metadata/filament_settings_"; // bbs_3mf.cpp:184
pub const PROJECT_EMBEDDED_PRINTER_PRESETS_FILE: &str = "Metadata/machine_settings_"; // bbs_3mf.cpp:185
pub const CUT_INFORMATION_FILE: &str = "Metadata/cut_information.xml"; // bbs_3mf.cpp:186

pub const AUXILIARY_STR_LEN: u32 = 12; // bbs_3mf.cpp:188
pub const METADATA_STR_LEN: u32 = 9; // bbs_3mf.cpp:189

pub const MODEL_TAG: &str = "model"; // bbs_3mf.cpp:192
pub const RESOURCES_TAG: &str = "resources"; // bbs_3mf.cpp:193
pub const COLOR_GROUP_TAG: &str = "m:colorgroup"; // bbs_3mf.cpp:194
pub const COLOR_TAG: &str = "m:color"; // bbs_3mf.cpp:195
pub const OBJECT_TAG: &str = "object"; // bbs_3mf.cpp:196
pub const MESH_TAG: &str = "mesh"; // bbs_3mf.cpp:197
pub const MESH_STAT_TAG: &str = "mesh_stat"; // bbs_3mf.cpp:198
pub const VERTICES_TAG: &str = "vertices"; // bbs_3mf.cpp:199
pub const VERTEX_TAG: &str = "vertex"; // bbs_3mf.cpp:200
pub const TRIANGLES_TAG: &str = "triangles"; // bbs_3mf.cpp:201
pub const TRIANGLE_TAG: &str = "triangle"; // bbs_3mf.cpp:202
pub const COMPONENTS_TAG: &str = "components"; // bbs_3mf.cpp:203
pub const COMPONENT_TAG: &str = "component"; // bbs_3mf.cpp:204
pub const BUILD_TAG: &str = "build"; // bbs_3mf.cpp:205
pub const ITEM_TAG: &str = "item"; // bbs_3mf.cpp:206
pub const METADATA_TAG: &str = "metadata"; // bbs_3mf.cpp:207
pub const FILAMENT_TAG: &str = "filament"; // bbs_3mf.cpp:208
pub const SLICE_WARNING_TAG: &str = "warning"; // bbs_3mf.cpp:209
pub const WARNING_MSG_TAG: &str = "msg"; // bbs_3mf.cpp:210
pub const FILAMENT_ID_TAG: &str = "id"; // bbs_3mf.cpp:211
pub const FILAMENT_TYPE_TAG: &str = "type"; // bbs_3mf.cpp:212
pub const FILAMENT_COLOR_TAG: &str = "color"; // bbs_3mf.cpp:213
pub const FILAMENT_USED_M_TAG: &str = "used_m"; // bbs_3mf.cpp:214
pub const FILAMENT_USED_G_TAG: &str = "used_g"; // bbs_3mf.cpp:215
pub const FILAMENT_USED_FOR_SUPPORT: &str = "used_for_support"; // bbs_3mf.cpp:216
pub const FILAMENT_USED_FOR_OBJECT: &str = "used_for_object"; // bbs_3mf.cpp:217
pub const FILAMENT_TRAY_INFO_ID_TAG: &str = "tray_info_idx"; // bbs_3mf.cpp:218
pub const LAYER_FILAMENT_LISTS_TAG: &str = "layer_filament_lists"; // bbs_3mf.cpp:219
pub const LAYER_FILAMENT_LIST_TAG: &str = "layer_filament_list"; // bbs_3mf.cpp:220
pub const FILAMENT_NOZZLE_GROUP_ID_TAG: &str = "group_id"; // bbs_3mf.cpp:221
pub const FILAMENT_NOZZLE_DIAMETER_TAG: &str = "nozzle_diameter"; // bbs_3mf.cpp:222
pub const FILAMENT_NOZZLE_VOLUME_TYPE_TAG: &str = "volume_type"; // bbs_3mf.cpp:223
pub const NOZZLE_TAG: &str = "nozzle"; // bbs_3mf.cpp:224

pub const CONFIG_TAG: &str = "config"; // bbs_3mf.cpp:227
pub const VOLUME_TAG: &str = "volume"; // bbs_3mf.cpp:228
pub const PART_TAG: &str = "part"; // bbs_3mf.cpp:229
pub const PLATE_TAG: &str = "plate"; // bbs_3mf.cpp:230
pub const INSTANCE_TAG: &str = "model_instance"; // bbs_3mf.cpp:231
//BBS
pub const ASSEMBLE_TAG: &str = "assemble"; // bbs_3mf.cpp:233
pub const ASSEMBLE_ITEM_TAG: &str = "assemble_item"; // bbs_3mf.cpp:234
pub const SLICE_HEADER_TAG: &str = "header"; // bbs_3mf.cpp:235
pub const SLICE_HEADER_ITEM_TAG: &str = "header_item"; // bbs_3mf.cpp:236

// text_info
pub const TEXT_INFO_TAG: &str = "text_info"; // bbs_3mf.cpp:239
pub const TEXT_ATTR: &str = "text"; // bbs_3mf.cpp:240
pub const FONT_NAME_ATTR: &str = "font_name"; // bbs_3mf.cpp:241
pub const FONT_VERSION_ATTR: &str = "font_version"; // bbs_3mf.cpp:242
pub const FONT_INDEX_ATTR: &str = "font_index"; // bbs_3mf.cpp:243
pub const FONT_SIZE_ATTR: &str = "font_size"; // bbs_3mf.cpp:244
pub const THICKNESS_ATTR: &str = "thickness"; // bbs_3mf.cpp:245
pub const EMBEDED_DEPTH_ATTR: &str = "embeded_depth"; // bbs_3mf.cpp:246
pub const ROTATE_ANGLE_ATTR: &str = "rotate_angle"; // bbs_3mf.cpp:247
pub const TEXT_GAP_ATTR: &str = "text_gap"; // bbs_3mf.cpp:248
pub const BOLD_ATTR: &str = "bold"; // bbs_3mf.cpp:249
pub const ITALIC_ATTR: &str = "italic"; // bbs_3mf.cpp:250
pub const SURFACE_TYPE: &str = "surface_type"; // bbs_3mf.cpp:251
pub const SURFACE_TEXT_ATTR: &str = "surface_text"; // bbs_3mf.cpp:252
pub const KEEP_HORIZONTAL_ATTR: &str = "keep_horizontal"; // bbs_3mf.cpp:253
pub const HIT_MESH_ATTR: &str = "hit_mesh"; // bbs_3mf.cpp:254
pub const HIT_POSITION_ATTR: &str = "hit_position"; // bbs_3mf.cpp:255
pub const HIT_NORMAL_ATTR: &str = "hit_normal"; // bbs_3mf.cpp:256

// BBS: encrypt
pub const RELATIONSHIP_TAG: &str = "Relationship"; // bbs_3mf.cpp:259
pub const PID_ATTR: &str = "pid"; // bbs_3mf.cpp:260
pub const PINDEX_ATTR: &str = "pindex"; // bbs_3mf.cpp:261
pub const P1_ATTR: &str = "p1"; // bbs_3mf.cpp:262
pub const P2_ATTR: &str = "p2"; // bbs_3mf.cpp:263
pub const P3_ATTR: &str = "p3"; // bbs_3mf.cpp:264
pub const PUUID_ATTR: &str = "p:UUID"; // bbs_3mf.cpp:265
pub const PUUID_LOWER_ATTR: &str = "p:uuid"; // bbs_3mf.cpp:266
pub const PPATH_ATTR: &str = "p:path"; // bbs_3mf.cpp:267
pub const OBJECT_UUID_SUFFIX: &str = "-61cb-4c03-9d28-80fed5dfa1dc"; // bbs_3mf.cpp:268
pub const OBJECT_UUID_SUFFIX2: &str = "-71cb-4c03-9d28-80fed5dfa1dc"; // bbs_3mf.cpp:269
pub const SUB_OBJECT_UUID_SUFFIX: &str = "-81cb-4c03-9d28-80fed5dfa1dc"; // bbs_3mf.cpp:270
pub const COMPONENT_UUID_SUFFIX: &str = "-b206-40ff-9872-83e8017abed1"; // bbs_3mf.cpp:271
pub const BUILD_UUID: &str = "2c7c17d8-22b5-4d84-8835-1976022ea369"; // bbs_3mf.cpp:272
pub const BUILD_UUID_SUFFIX: &str = "-b1ec-4553-aec9-835e5b724bb4"; // bbs_3mf.cpp:273
pub const TARGET_ATTR: &str = "Target"; // bbs_3mf.cpp:274
pub const RELS_TYPE_ATTR: &str = "Type"; // bbs_3mf.cpp:275

pub const UNIT_ATTR: &str = "unit"; // bbs_3mf.cpp:277
pub const NAME_ATTR: &str = "name"; // bbs_3mf.cpp:278
pub const COLOR_ATTR: &str = "color"; // bbs_3mf.cpp:279
pub const TYPE_ATTR: &str = "type"; // bbs_3mf.cpp:280
pub const ID_ATTR: &str = "id"; // bbs_3mf.cpp:281
pub const X_ATTR: &str = "x"; // bbs_3mf.cpp:282
pub const Y_ATTR: &str = "y"; // bbs_3mf.cpp:283
pub const Z_ATTR: &str = "z"; // bbs_3mf.cpp:284
pub const V1_ATTR: &str = "v1"; // bbs_3mf.cpp:285
pub const V2_ATTR: &str = "v2"; // bbs_3mf.cpp:286
pub const V3_ATTR: &str = "v3"; // bbs_3mf.cpp:287
pub const OBJECTID_ATTR: &str = "objectid"; // bbs_3mf.cpp:288
pub const TRANSFORM_ATTR: &str = "transform"; // bbs_3mf.cpp:289
// BBS
pub const OFFSET_ATTR: &str = "offset"; // bbs_3mf.cpp:291
pub const PRINTABLE_ATTR: &str = "printable"; // bbs_3mf.cpp:292
pub const INSTANCESCOUNT_ATTR: &str = "instances_count"; // bbs_3mf.cpp:293
pub const CUSTOM_SUPPORTS_ATTR: &str = "paint_supports"; // bbs_3mf.cpp:294
pub const CUSTOM_FUZZY_SKIN_ATTR: &str = "paint_fuzzy_skin"; // bbs_3mf.cpp:295
pub const CUSTOM_SEAM_ATTR: &str = "paint_seam"; // bbs_3mf.cpp:296
pub const MMU_SEGMENTATION_ATTR: &str = "paint_color"; // bbs_3mf.cpp:297
// BBS
pub const FACE_PROPERTY_ATTR: &str = "face_property"; // bbs_3mf.cpp:299

pub const KEY_ATTR: &str = "key"; // bbs_3mf.cpp:301
pub const VALUE_ATTR: &str = "value"; // bbs_3mf.cpp:302
pub const FIRST_TRIANGLE_ID_ATTR: &str = "firstid"; // bbs_3mf.cpp:303
pub const LAST_TRIANGLE_ID_ATTR: &str = "lastid"; // bbs_3mf.cpp:304
pub const SUBTYPE_ATTR: &str = "subtype"; // bbs_3mf.cpp:305
pub const LOCK_ATTR: &str = "locked"; // bbs_3mf.cpp:306
pub const BED_TYPE_ATTR: &str = "bed_type"; // bbs_3mf.cpp:307
pub const PRINT_SEQUENCE_ATTR: &str = "print_sequence"; // bbs_3mf.cpp:308
pub const FIRST_LAYER_PRINT_SEQUENCE_ATTR: &str = "first_layer_print_sequence"; // bbs_3mf.cpp:309
pub const OTHER_LAYERS_PRINT_SEQUENCE_ATTR: &str = "other_layers_print_sequence"; // bbs_3mf.cpp:310
pub const OTHER_LAYERS_PRINT_SEQUENCE_NUMS_ATTR: &str = "other_layers_print_sequence_nums"; // bbs_3mf.cpp:311
pub const SPIRAL_VASE_MODE: &str = "spiral_mode"; // bbs_3mf.cpp:312
pub const FILAMENT_MAP_MODE_ATTR: &str = "filament_map_mode"; // bbs_3mf.cpp:313
pub const FILAMENT_MAP_ATTR: &str = "filament_maps"; // bbs_3mf.cpp:314
pub const LIMIT_FILAMENT_MAP_ATTR: &str = "limit_filament_maps"; // bbs_3mf.cpp:315
pub const FILAMENT_VOL_MAP_ATTR: &str = "filament_volume_maps"; // bbs_3mf.cpp:316
pub const GCODE_FILE_ATTR: &str = "gcode_file"; // bbs_3mf.cpp:317
pub const THUMBNAIL_FILE_ATTR: &str = "thumbnail_file"; // bbs_3mf.cpp:318
pub const NO_LIGHT_THUMBNAIL_FILE_ATTR: &str = "thumbnail_no_light_file"; // bbs_3mf.cpp:319
pub const TOP_FILE_ATTR: &str = "top_file"; // bbs_3mf.cpp:320
pub const PICK_FILE_ATTR: &str = "pick_file"; // bbs_3mf.cpp:321
pub const PATTERN_FILE_ATTR: &str = "pattern_file"; // bbs_3mf.cpp:322
pub const PATTERN_BBOX_FILE_ATTR: &str = "pattern_bbox_file"; // bbs_3mf.cpp:323
pub const OBJECT_ID_ATTR: &str = "object_id"; // bbs_3mf.cpp:324
pub const INSTANCEID_ATTR: &str = "instance_id"; // bbs_3mf.cpp:325
pub const IDENTIFYID_ATTR: &str = "identify_id"; // bbs_3mf.cpp:326
pub const PLATERID_ATTR: &str = "plater_id"; // bbs_3mf.cpp:327
pub const PLATER_NAME_ATTR: &str = "plater_name"; // bbs_3mf.cpp:328
pub const PLATE_IDX_ATTR: &str = "index"; // bbs_3mf.cpp:329
pub const PRINTER_MODEL_ID_ATTR: &str = "printer_model_id"; // bbs_3mf.cpp:330
pub const EXTRUDER_TYPE_ATTR: &str = "extruder_type"; // bbs_3mf.cpp:331
pub const NOZZLE_VOLUME_TYPE_ATTR: &str = "nozzle_volume_type"; // bbs_3mf.cpp:332
pub const NOZZLE_TYPE_ATTR: &str = "nozzle_types"; // bbs_3mf.cpp:333
pub const NOZZLE_DIAMETERS_ATTR: &str = "nozzle_diameters"; // bbs_3mf.cpp:334
pub const SLICE_PREDICTION_ATTR: &str = "prediction"; // bbs_3mf.cpp:335
pub const SLICE_WEIGHT_ATTR: &str = "weight"; // bbs_3mf.cpp:336
pub const FIRST_LAYER_TIME_ATTR: &str = "first_layer_time"; // bbs_3mf.cpp:337
pub const TIMELAPSE_TYPE_ATTR: &str = "timelapse_type"; // bbs_3mf.cpp:338
pub const OUTSIDE_ATTR: &str = "outside"; // bbs_3mf.cpp:339
pub const SUPPORT_USED_ATTR: &str = "support_used"; // bbs_3mf.cpp:340
pub const LABEL_OBJECT_ENABLED_ATTR: &str = "label_object_enabled"; // bbs_3mf.cpp:341
pub const ENABLE_FILAMENT_DYNAMIC_MAP_ATTR: &str = "enable_filament_dynamic_map"; // bbs_3mf.cpp:342
pub const HAS_FILAMENT_SWITCHER_ATTR: &str = "has_filament_switcher"; // bbs_3mf.cpp:343
pub const SKIPPED_ATTR: &str = "skipped"; // bbs_3mf.cpp:344

pub const OBJECT_TYPE: &str = "object"; // bbs_3mf.cpp:346
pub const VOLUME_TYPE: &str = "volume"; // bbs_3mf.cpp:347
pub const PART_TYPE: &str = "part"; // bbs_3mf.cpp:348

pub const NAME_KEY: &str = "name"; // bbs_3mf.cpp:350
pub const VOLUME_TYPE_KEY: &str = "volume_type"; // bbs_3mf.cpp:351
pub const PART_TYPE_KEY: &str = "part_type"; // bbs_3mf.cpp:352
pub const MATRIX_KEY: &str = "matrix"; // bbs_3mf.cpp:353
pub const SOURCE_FILE_KEY: &str = "source_file"; // bbs_3mf.cpp:354
pub const SOURCE_OBJECT_ID_KEY: &str = "source_object_id"; // bbs_3mf.cpp:355
pub const SOURCE_VOLUME_ID_KEY: &str = "source_volume_id"; // bbs_3mf.cpp:356
pub const SOURCE_OFFSET_X_KEY: &str = "source_offset_x"; // bbs_3mf.cpp:357
pub const SOURCE_OFFSET_Y_KEY: &str = "source_offset_y"; // bbs_3mf.cpp:358
pub const SOURCE_OFFSET_Z_KEY: &str = "source_offset_z"; // bbs_3mf.cpp:359
pub const SOURCE_IN_INCHES: &str = "source_in_inches"; // bbs_3mf.cpp:360
pub const SOURCE_IN_METERS: &str = "source_in_meters"; // bbs_3mf.cpp:361

pub const MESH_SHARED_KEY: &str = "mesh_shared"; // bbs_3mf.cpp:363

pub const MESH_STAT_FACE_COUNT: &str = "face_count"; // bbs_3mf.cpp:365
pub const MESH_STAT_EDGES_FIXED: &str = "edges_fixed"; // bbs_3mf.cpp:366
pub const MESH_STAT_DEGENERATED_FACETS: &str = "degenerate_facets"; // bbs_3mf.cpp:367
pub const MESH_STAT_FACETS_REMOVED: &str = "facets_removed"; // bbs_3mf.cpp:368
pub const MESH_STAT_FACETS_RESERVED: &str = "facets_reversed"; // bbs_3mf.cpp:369
pub const MESH_STAT_BACKWARDS_EDGES: &str = "backwards_edges"; // bbs_3mf.cpp:370

// Store / load of TextConfiguration
pub const TEXT_DATA_ATTR: &str = "text"; // bbs_3mf.cpp:373
// TextConfiguration::EmbossStyle
pub const STYLE_NAME_ATTR: &str = "style_name"; // bbs_3mf.cpp:375
pub const FONT_DESCRIPTOR_ATTR: &str = "font_descriptor"; // bbs_3mf.cpp:376
pub const FONT_DESCRIPTOR_TYPE_ATTR: &str = "font_descriptor_type"; // bbs_3mf.cpp:377

// TextConfiguration::FontProperty
pub const CHAR_GAP_ATTR: &str = "char_gap"; // bbs_3mf.cpp:380
pub const LINE_GAP_ATTR: &str = "line_gap"; // bbs_3mf.cpp:381
pub const LINE_HEIGHT_ATTR: &str = "line_height"; // bbs_3mf.cpp:382
pub const BOLDNESS_ATTR: &str = "boldness"; // bbs_3mf.cpp:383
pub const SKEW_ATTR: &str = "skew"; // bbs_3mf.cpp:384
pub const PER_GLYPH_ATTR: &str = "per_glyph"; // bbs_3mf.cpp:385
pub const HORIZONTAL_ALIGN_ATTR: &str = "horizontal"; // bbs_3mf.cpp:386
pub const VERTICAL_ALIGN_ATTR: &str = "vertical"; // bbs_3mf.cpp:387
pub const COLLECTION_NUMBER_ATTR: &str = "collection"; // bbs_3mf.cpp:388

pub const FONT_FAMILY_ATTR: &str = "family"; // bbs_3mf.cpp:390
pub const FONT_FACE_NAME_ATTR: &str = "face_name"; // bbs_3mf.cpp:391
pub const FONT_STYLE_ATTR: &str = "style"; // bbs_3mf.cpp:392
pub const FONT_WEIGHT_ATTR: &str = "weight"; // bbs_3mf.cpp:393

// Store / load of EmbossShape
pub const OLD_SHAPE_TAG: &str = "slic3rpe:shape"; // bbs_3mf.cpp:396
pub const SHAPE_TAG: &str = "BambuStudioShape"; // bbs_3mf.cpp:397
pub const SHAPE_SCALE_ATTR: &str = "scale"; // bbs_3mf.cpp:398
pub const UNHEALED_ATTR: &str = "unhealed"; // bbs_3mf.cpp:399
pub const SVG_FILE_PATH_ATTR: &str = "filepath"; // bbs_3mf.cpp:400
pub const SVG_FILE_PATH_IN_3MF_ATTR: &str = "filepath3mf"; // bbs_3mf.cpp:401

// EmbossProjection
pub const DEPTH_ATTR: &str = "depth"; // bbs_3mf.cpp:404
pub const USE_SURFACE_ATTR: &str = "use_surface"; // bbs_3mf.cpp:405
// pub const FIX_TRANSFORMATION_ATTR: &str = "transform"; // bbs_3mf.cpp:406 (commented out)

pub const BBS_VALID_OBJECT_TYPES_COUNT: usize = 2; // bbs_3mf.cpp:408
// bbs_3mf.cpp:409-413
pub const BBS_VALID_OBJECT_TYPES: [&str; 2] = ["model", "other"];
// bbs_3mf.cpp:415-420
pub const BBS_INVALID_OBJECT_TYPES: [&str; 3] = ["solidsupport", "support", "surface"];

// The constants of bbs_3mf.cpp at the pin that the earlier port of this file predates.
pub const ASSEMBLY_TREE_FILE: &str = "Metadata/assembly_tree.json"; // bbs_3mf.cpp:175
pub const ASSEMBLY_STEP_JSON_FILE: &str = "Metadata/assembly_step.json"; // bbs_3mf.cpp:176
pub const ASSEMBLY_MODEL_FILE: &str = "Metadata/assembly_model.json"; // bbs_3mf.cpp:177
pub const MIXED_FILAMENT_TAG: &str = "mixed_filament"; // bbs_3mf.cpp:228
pub const MIXED_FILAMENT_COMPONENTS_TAG: &str = "components"; // bbs_3mf.cpp:229
pub const PAUSE_LIST_TAG: &str = "pause_list"; // bbs_3mf.cpp:230
pub const PAUSE_TAG: &str = "pause"; // bbs_3mf.cpp:231
pub const PAUSE_INDEX_ATTR: &str = "index"; // bbs_3mf.cpp:232
pub const PAUSE_LAYER_ATTR: &str = "layer"; // bbs_3mf.cpp:233
pub const PAUSE_PERCENT_ATTR: &str = "percent"; // bbs_3mf.cpp:234
pub const PAUSE_REMAINING_TIME_ATTR: &str = "remaining_time"; // bbs_3mf.cpp:235
pub const FILAMENT_DIRECTLY_USED_ATTR: &str = "directly_used"; // bbs_3mf.cpp:245
pub const FILAMENT_TOTAL_LOAD_TIME_TAG: &str = "total_load_time"; // bbs_3mf.cpp:246
pub const FILAMENT_TOTAL_UNLOAD_TIME_TAG: &str = "total_unload_time"; // bbs_3mf.cpp:247
pub const DEFAULT_AMS_TYPE_ATTR: &str = "default_ams_type"; // bbs_3mf.cpp:255
pub const AMS_LIST_TAG: &str = "ams_list"; // bbs_3mf.cpp:256
pub const AMS_ITEM_TAG: &str = "ams"; // bbs_3mf.cpp:257
pub const AMS_TYPE_ATTR: &str = "ams_type"; // bbs_3mf.cpp:258
pub const AMS_LOAD_TIME_ATTR: &str = "load_time"; // bbs_3mf.cpp:259
pub const AMS_UNLOAD_TIME_ATTR: &str = "unload_time"; // bbs_3mf.cpp:260
pub const VOLUMEID_ATTR: &str = "volume_id"; // bbs_3mf.cpp:329
pub const PAUSE_COUNT_ATTR: &str = "pause_count"; // bbs_3mf.cpp:375
pub const SUPPORT_MATERIAL_ON_WIPE_TOWER_ATTR: &str = "support_material_on_wipe_tower"; // bbs_3mf.cpp:381
pub const MULTI_NOZZLE_DIAMETER_MODE_ATTR: &str = "multi_nozzle_diameter_mode"; // bbs_3mf.cpp:384
pub const PART_GUID_KEY: &str = "uuid"; // bbs_3mf.cpp:405
pub const ASSEMBLY_SRC_GUID_KEY: &str = "assembly_src_guid"; // bbs_3mf.cpp:406
/// GCodeProcessor.hpp:27 `#define NOT_GENERATE_TIMELAPSE "not_generate_timelapse"`
pub const NOT_GENERATE_TIMELAPSE: &str = "not_generate_timelapse";

/// bbs_3mf.cpp:422-438  template<typename T> struct hex_wrap + std::operator<<
/// `ostr << setw(sizeof(_Arg) * 2) << std::hex << wrap.t` with fill '0'.
pub fn hex_wrap<T: std::fmt::LowerHex>(t: T) -> String {
    format!("{:0width$x}", t, width = std::mem::size_of::<T>() * 2)
}

/// bbs_3mf.cpp:440-445  class version_error : public Slic3r::FileIOError
#[derive(Debug, Clone)]
pub struct VersionError {
    /// version_error(const std::string& what_arg) / (const char* what_arg)
    pub what: String,
}

impl VersionError {
    pub fn new(what_arg: impl Into<String>) -> Self {
        VersionError {
            what: what_arg.into(),
        }
    }
}

impl std::fmt::Display for VersionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.what)
    }
}

impl std::error::Error for VersionError {}

// ---------------------------------------------------------------------------
// XML attribute helpers (bbs_3mf.cpp:447-493)
// ---------------------------------------------------------------------------
// expat hands attributes as a flat `const char**` array of key/value pairs with
// `attributes_size == 2 * count`; the Rust drivers collect `(key, value)`
// pairs (same adaptation as `format::three_mf`), so the
// `attributes_size % 2 != 0` guard is unrepresentable here.

/// bbs_3mf.cpp:447-458
/// C++: `const char* bbs_get_attribute_value_charptr(const char** attributes, unsigned int attributes_size, const char* attribute_key)`
pub fn bbs_get_attribute_value_charptr<'a>(
    attributes: &'a [(String, String)],
    attribute_key: &str,
) -> Option<&'a str> {
    // bbs_3mf.cpp:449-450
    if attributes.is_empty() {
        return None;
    }
    // bbs_3mf.cpp:452-455
    for (key, value) in attributes {
        if key == attribute_key {
            return Some(value.as_str());
        }
    }
    // bbs_3mf.cpp:457
    None
}

/// bbs_3mf.cpp:460-464
/// C++: `std::string bbs_get_attribute_value_string(...)`
pub fn bbs_get_attribute_value_string(
    attributes: &[(String, String)],
    attribute_key: &str,
) -> String {
    // bbs_3mf.cpp:462-463
    bbs_get_attribute_value_charptr(attributes, attribute_key)
        .unwrap_or("")
        .to_string()
}

/// bbs_3mf.cpp:466-472
/// C++: `float bbs_get_attribute_value_float(...)` —
/// `fast_float::from_chars(text, text + strlen(text), value)`; on parse
/// failure the value stays 0.0f.
pub fn bbs_get_attribute_value_float(attributes: &[(String, String)], attribute_key: &str) -> f32 {
    // bbs_3mf.cpp:468
    let mut value = 0.0f32;
    // bbs_3mf.cpp:469-470
    if let Some(text) = bbs_get_attribute_value_charptr(attributes, attribute_key) {
        if let Some(v) = fast_float_from_chars_f32(text) {
            value = v;
        }
    }
    // bbs_3mf.cpp:471
    value
}

/// bbs_3mf.cpp:474-479
/// C++: `bool bbs_has_attribute_value_int(...)`
pub fn bbs_has_attribute_value_int(attributes: &[(String, String)], attribute_key: &str) -> bool {
    // bbs_3mf.cpp:476-478
    bbs_get_attribute_value_charptr(attributes, attribute_key).is_some()
}

/// bbs_3mf.cpp:481-487
/// C++: `int bbs_get_attribute_value_int(...)` —
/// `boost::spirit::qi::parse(text, text + strlen(text), qi::int_, value)`;
/// on parse failure the value stays 0.
pub fn bbs_get_attribute_value_int(attributes: &[(String, String)], attribute_key: &str) -> i32 {
    // bbs_3mf.cpp:483
    let mut value = 0i32;
    // bbs_3mf.cpp:484-485 — qi::int_ parses [+-]?digits from the start (no
    // whitespace skipping with plain qi::parse).
    if let Some(text) = bbs_get_attribute_value_charptr(attributes, attribute_key) {
        let bytes = text.as_bytes();
        let mut i = 0usize;
        let mut negative = false;
        if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
            negative = bytes[i] == b'-';
            i += 1;
        }
        // qi::int_ (extract_int) stops before the digit that would overflow int and keeps what it
        // has; plain qi::parse needs no full match, so "2147483648" gives 214748364.
        let limit: i64 = if negative { -(i32::MIN as i64) } else { i32::MAX as i64 };
        let mut parsed: i64 = 0;
        let mut any = false;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            let next = parsed * 10 + (bytes[i] - b'0') as i64;
            if next > limit {
                break;
            }
            parsed = next;
            any = true;
            i += 1;
        }
        if any {
            value = (if negative { -parsed } else { parsed }) as i32;
        }
    }
    // bbs_3mf.cpp:486
    value
}

/// bbs_3mf.cpp:489-493
/// C++: `bool bbs_get_attribute_value_bool(...)` —
/// `(text != nullptr) ? (bool)::atoi(text) : true`
pub fn bbs_get_attribute_value_bool(attributes: &[(String, String)], attribute_key: &str) -> bool {
    // bbs_3mf.cpp:491-492
    match bbs_get_attribute_value_charptr(attributes, attribute_key) {
        Some(text) => atoi(text) != 0,
        None => true,
    }
}

// ---------------------------------------------------------------------------
// C runtime helpers (atoi/atof used throughout bbs_3mf.cpp; same adaptation as
// `format::three_mf` / `format::amf`)
// ---------------------------------------------------------------------------

/// C `atof(nptr)` == `strtod(nptr, NULL)`.
fn atof(s: &str) -> f64 {
    objparser::strtod(s.as_bytes(), 0).0
}

/// C `atoi(nptr)`: skip leading C whitespace, optional sign, longest run of
/// decimal digits; `0` when no conversion is performed.
fn atoi(s: &str) -> i32 {
    let bytes = s.as_bytes();
    let n = bytes.len();
    let mut i = 0;
    while i < n && matches!(bytes[i], b' ' | b'\t' | b'\n' | 0x0B | 0x0C | b'\r') {
        i += 1;
    }
    let mut negative = false;
    if i < n && (bytes[i] == b'+' || bytes[i] == b'-') {
        negative = bytes[i] == b'-';
        i += 1;
    }
    let mut value: i64 = 0;
    let mut any = false;
    while i < n && bytes[i].is_ascii_digit() {
        any = true;
        value = value
            .saturating_mul(10)
            .saturating_add((bytes[i] - b'0') as i64);
        i += 1;
    }
    if !any {
        return 0;
    }
    let value = if negative { -value } else { value };
    value.clamp(i32::MIN as i64, i32::MAX as i64) as i32
}

/// bbs_3mf.cpp:495-502
/// C++: `void add_vec3(std::stringstream &stream, const Slic3r::Vec3f &tr)` —
/// `stream << tr(r)` uses the default ostream float formatting (`%g`,
/// 6 significant digits), matched by `locales_utils::general_format`.
pub fn add_vec3(stream: &mut String, tr: &Vec3f) {
    for r in 0..3usize {
        // bbs_3mf.cpp:498
        stream.push_str(&general_format(tr[r] as f64, 6));
        // bbs_3mf.cpp:499-500
        if r != 2 {
            stream.push(' ');
        }
    }
}

/// bbs_3mf.cpp:504-512
/// C++: `template<typename T> void add_vector(std::stringstream &stream, const std::vector<T> &values)`
/// (instantiated with integer element types in bbs_3mf.cpp; `Display` matches
/// the ostream `operator<<` for those).
pub fn add_vector<T: std::fmt::Display>(stream: &mut String, values: &[T]) {
    for i in 0..values.len() {
        // bbs_3mf.cpp:508
        stream.push_str(&values[i].to_string());
        // bbs_3mf.cpp:509-510
        if i != values.len() - 1 {
            stream.push(' ');
        }
    }
}

/// bbs_3mf.cpp:514-535
/// C++: `std::vector<int> parse_int_list(const std::string& value)`
pub fn parse_int_list(value: &str) -> Vec<i32> {
    // bbs_3mf.cpp:516-518
    let mut out: Vec<i32> = Vec::new();
    if value.is_empty() {
        return out;
    }

    // bbs_3mf.cpp:520-521  boost::split(tokens, value, boost::is_any_of(" ,"), boost::token_compress_on);
    let tokens: Vec<&str> = value
        .split(|c| c == ' ' || c == ',')
        .filter(|s| !s.is_empty())
        .collect();
    out.reserve(tokens.len()); // bbs_3mf.cpp:522
    for t in tokens {
        // bbs_3mf.cpp:524-525 (empty tokens are filtered above)
        // bbs_3mf.cpp:526-529 — boost::lexical_cast<int> requires the whole
        // token to parse; failures are swallowed.
        if let Ok(v) = t.parse::<i32>() {
            out.push(v);
        }
    }

    // bbs_3mf.cpp:532-533
    out.sort();
    out.dedup();
    out
}

/// bbs_3mf.cpp:537-546
/// C++: `std::string join_int_list_comma(const std::vector<int>& values)`
pub fn join_int_list_comma(values: &[i32]) -> String {
    let mut stream = String::new();
    for i in 0..values.len() {
        // bbs_3mf.cpp:541
        stream.push_str(&values[i].to_string());
        // bbs_3mf.cpp:542-543
        if i + 1 < values.len() {
            stream.push(',');
        }
    }
    stream
}

/// bbs_3mf.cpp:548-564
/// C++: `Slic3r::Vec3f get_vec3_from_string(const std::string &pos_str)`
pub fn get_vec3_from_string(pos_str: &str) -> Vec3f {
    // bbs_3mf.cpp:550-552
    let mut pos = Vec3f::new(0.0, 0.0, 0.0);
    if pos_str.is_empty() {
        return pos;
    }

    // bbs_3mf.cpp:554-555  boost::split(values, pos_str, boost::is_any_of(" "), boost::token_compress_on);
    let values: Vec<&str> = pos_str.split(' ').filter(|s| !s.is_empty()).collect();

    // bbs_3mf.cpp:557-558
    if values.len() != 3 {
        return pos;
    }

    // bbs_3mf.cpp:560-561
    for i in 0..3usize {
        pos[i] = atof(values[i]) as f32;
    }

    pos
}

/// bbs_3mf.cpp:566-593
/// C++: `Slic3r::Transform3d bbs_get_transform_from_3mf_specs_string(const std::string& mat_str)`
pub fn bbs_get_transform_from_3mf_specs_string(mat_str: &str) -> Transform3d {
    // check: https://3mf.io/3d-manufacturing-format/ or https://github.com/3MFConsortium/spec_core/blob/master/3MF%20Core%20Specification.md
    // to see how matrices are stored inside 3mf according to specifications
    // bbs_3mf.cpp:570
    let mut ret = Transform3d::identity();

    // bbs_3mf.cpp:572-574 — empty string means default identity matrix
    if mat_str.is_empty() {
        return ret;
    }

    // bbs_3mf.cpp:576-577
    let mat_elements_str: Vec<&str> = mat_str.split(' ').filter(|s| !s.is_empty()).collect();

    // bbs_3mf.cpp:579-582 — invalid data, return identity matrix
    let size = mat_elements_str.len();
    if size != 12 {
        return ret;
    }

    // bbs_3mf.cpp:584-591 — matrices are stored into 3mf files as 4x3,
    // we need to transpose them
    let mut i = 0usize;
    for c in 0..4usize {
        for r in 0..3usize {
            ret[(r, c)] = atof(mat_elements_str[i]);
            i += 1;
        }
    }
    ret
}

/// bbs_3mf.cpp:595-616
/// C++: `Slic3r::Vec3d bbs_get_offset_from_3mf_specs_string(const std::string& vec_str)`
pub fn bbs_get_offset_from_3mf_specs_string(vec_str: &str) -> Vec3d {
    // bbs_3mf.cpp:597
    let mut ofs2ass = Vec3d::new(0.0, 0.0, 0.0);

    // bbs_3mf.cpp:599-601 — empty string means default zero offset
    if vec_str.is_empty() {
        return ofs2ass;
    }

    // bbs_3mf.cpp:603-604
    let vec_elements_str: Vec<&str> = vec_str.split(' ').filter(|s| !s.is_empty()).collect();

    // bbs_3mf.cpp:606-609 — invalid data, return zero offset
    let size = vec_elements_str.len();
    if size != 3 {
        return ofs2ass;
    }

    // bbs_3mf.cpp:611-613
    for i in 0..3usize {
        ofs2ass[i] = atof(vec_elements_str[i]);
    }

    ofs2ass
}

/// bbs_3mf.cpp:618-635
/// C++: `float bbs_get_unit_factor(const std::string& unit)`
pub fn bbs_get_unit_factor(unit: &str) -> f32 {
    // bbs_3mf.cpp:620-634
    if unit == "micron" {
        0.001f32
    } else if unit == "centimeter" {
        10.0f32
    } else if unit == "inch" {
        25.4f32
    } else if unit == "foot" {
        304.8f32
    } else if unit == "meter" {
        1000.0f32
    } else {
        // default "millimeters" (see specification)
        1.0f32
    }
}

/// bbs_3mf.cpp:637-649
/// C++: `bool bbs_is_valid_object_type(const std::string& type)`
pub fn bbs_is_valid_object_type(type_: &str) -> bool {
    // if the type is empty defaults to "model" (see specification)
    // bbs_3mf.cpp:640-641
    if type_.is_empty() {
        return true;
    }

    // bbs_3mf.cpp:643-646
    for i in 0..BBS_VALID_OBJECT_TYPES_COUNT {
        if type_ == BBS_VALID_OBJECT_TYPES[i] {
            return true;
        }
    }

    // bbs_3mf.cpp:648
    false
}

/// boost::filesystem `path::lexically_normal()` of a generic ('/'-separated) path: "." elements
/// dropped, "name/.." pairs removed, separators collapsed.
///
/// FIDELITY-NOTE: a trailing separator is dropped (boost keeps it as a trailing "."); the only
/// use, `bbs_join_path_within_dir`, compares a directory with a path inside it.
fn lexically_normal(p: &str) -> String {
    let absolute = p.starts_with('/');
    let mut parts: Vec<&str> = Vec::new();
    for part in p.split('/') {
        if part.is_empty() || part == "." {
            continue;
        }
        if part == ".." {
            if let Some(last) = parts.last() {
                if *last != ".." {
                    parts.pop();
                    continue;
                }
            }
            if absolute {
                continue;
            }
        }
        parts.push(part);
    }
    let joined = parts.join("/");
    if absolute {
        format!("/{}", joined)
    } else if joined.is_empty() {
        ".".to_string()
    } else {
        joined
    }
}

/// bbs_3mf.cpp:694-705 `static std::string bbs_join_path_within_dir(const std::string& base_dir, const std::string& rel_path)`
fn bbs_join_path_within_dir(base_dir: &str, rel_path: &str) -> String {
    let joined = format!("{}/{}", base_dir, rel_path);
    let base_str = lexically_normal(&base_dir.replace('\\', "/"));
    let joined_str = lexically_normal(&joined.replace('\\', "/"));
    let inside = joined_str.len() >= base_str.len()
        && joined_str.as_bytes()[..base_str.len()] == *base_str.as_bytes()
        && (joined_str.len() == base_str.len() || joined_str.as_bytes()[base_str.len()] == b'/');
    if inside {
        joined
    } else {
        String::new()
    }
}

/// bbs_3mf.cpp:707-729 `static bool bbs_encoded_has_dotdot_component(const std::string& encoded)`:
/// encode_path() calls WideCharToMultiByte without WC_NO_BEST_FIT_CHARS, so on systems with a
/// best-fit code page (e.g. CP1252) U+FF0E/U+FF0F map to '.'/'/'. Reject a ".." component.
fn bbs_encoded_has_dotdot_component(encoded: &str) -> bool {
    let b = encoded.as_bytes();
    if b.is_empty() {
        return true;
    }
    if b[0] == b'/' || b[0] == b'\\' {
        return true;
    }
    let mut i = 0usize;
    while i <= b.len() {
        let j = b[i..].iter().position(|&c| c == b'/' || c == b'\\').map(|p| p + i).unwrap_or(b.len());
        if j - i == 2 && b[i] == b'.' && b[i + 1] == b'.' {
            return true;
        }
        if j == b.len() {
            break;
        }
        i = j + 1;
    }
    false
}

/// bbs_3mf.cpp:731-736 `static bool bbs_auxiliary_subpath_safe_for_encoding(const std::string& subpath)`
fn bbs_auxiliary_subpath_safe_for_encoding(subpath: &str) -> bool {
    if subpath.is_empty() {
        return false;
    }
    !bbs_encoded_has_dotdot_component(&crate::utils::encode_path(subpath))
}

/// bbs_3mf.cpp:738-764 `static std::string bbs_auxiliary_sanitize_after_encoding(const std::string& subpath)`:
/// drop encoded "." / ".." components and keep the remaining relative path.
fn bbs_auxiliary_sanitize_after_encoding(subpath: &str) -> String {
    if subpath.is_empty() {
        return String::new();
    }
    let encoded = crate::utils::encode_path(subpath);
    if encoded.is_empty() || encoded.starts_with('/') || encoded.starts_with('\\') {
        return String::new();
    }

    // boost::split(..., is_any_of("/\\"), token_compress_on)
    let parts: Vec<&str> = encoded.split(|c| c == '/' || c == '\\').collect();

    let mut out = String::new();
    for part in parts {
        if part.is_empty() || part == "." || part == ".." {
            continue;
        }
        if !out.is_empty() {
            out.push('/');
        }
        out.push_str(part);
    }
    if out.is_empty() || bbs_encoded_has_dotdot_component(&out) {
        return String::new();
    }
    crate::utils::decode_path(&out)
}

/// bbs_3mf.cpp:766-781 `static bool bbs_encoded_path_within_dir(const std::string& encoded_dir, const std::string& encoded_path)`
fn bbs_encoded_path_within_dir(encoded_dir: &str, encoded_path: &str) -> bool {
    if encoded_path.len() <= encoded_dir.len() {
        return false;
    }
    if !encoded_path.as_bytes().starts_with(encoded_dir.as_bytes()) {
        return false;
    }
    let sep = encoded_path.as_bytes()[encoded_dir.len()];
    if sep != b'/' && sep != b'\\' {
        return false;
    }
    let rest = &encoded_path[encoded_dir.len() + 1..];
    !rest.is_empty() && !bbs_encoded_has_dotdot_component(rest)
}

/// bbs_3mf.cpp:786 — headroom below MAX_PATH (260) for the auxiliary files.
#[cfg_attr(not(windows), allow(dead_code))]
const BBS_AUXILIARY_MAX_PATH_LEN: usize = 240;

/// bbs_3mf.cpp:788-825 `static std::string bbs_auxiliary_shorten_subpath_for_max_path(const std::string& dir, const std::string& subpath)`:
/// on Windows, shortens the file name of a too long path (keeping the extension and appending a
/// CRC of the original sub path); elsewhere the sub path unchanged.
fn bbs_auxiliary_shorten_subpath_for_max_path(dir: &str, subpath: &str) -> String {
    #[cfg(windows)]
    {
        let full: Vec<u16> = format!("{}/{}", dir, subpath).encode_utf16().collect();
        if full.len() <= BBS_AUXILIARY_MAX_PATH_LEN {
            return subpath.to_string();
        }

        let wsub: Vec<u16> = subpath.encode_utf16().collect();
        let mut name_pos = wsub.iter().rposition(|&c| c == b'/' as u16 || c == b'\\' as u16).map(|p| p + 1).unwrap_or(0);
        if name_pos > wsub.len() {
            name_pos = 0;
        }
        let mut ext_pos = wsub.iter().rposition(|&c| c == b'.' as u16).unwrap_or(wsub.len());
        if ext_pos <= name_pos {
            ext_pos = wsub.len();
        }

        let stem = &wsub[name_pos..ext_pos];
        let ext = &wsub[ext_pos..];
        let mut crc = flate2::Crc::new();
        crc.update(subpath.as_bytes());
        let suffix: Vec<u16> = format!("_{:08x}", crc.sum()).encode_utf16().chain(ext.iter().cloned()).collect();

        let fixed_len = full.len() - stem.len() - ext.len();
        if fixed_len + suffix.len() > BBS_AUXILIARY_MAX_PATH_LEN {
            return subpath.to_string();
        }

        let mut keep = stem.len().min(BBS_AUXILIARY_MAX_PATH_LEN - fixed_len - suffix.len());
        // Do not split a UTF-16 surrogate pair.
        if keep > 0 && stem[keep - 1] >= 0xD800 && stem[keep - 1] <= 0xDBFF {
            keep -= 1;
        }

        let mut out: Vec<u16> = wsub[..name_pos].to_vec();
        out.extend_from_slice(&stem[..keep]);
        out.extend_from_slice(&suffix);
        String::from_utf16_lossy(&out)
    }
    #[cfg(not(windows))]
    {
        let _ = dir;
        subpath.to_string()
    }
}

/// Base class with error messages management
/// bbs_3mf.cpp:736-751  class _BBS_3MF_Base
#[derive(Debug, Default)]
pub struct Bbs3mfBase {
    /// bbs_3mf.cpp:738-739  mutable boost::mutex mutex; mutable std::vector<std::string> m_errors;
    m_errors: Mutex<Vec<String>>,
}

impl Bbs3mfBase {
    pub fn new() -> Self {
        Self::default()
    }

    /// bbs_3mf.cpp:742  void add_error(const std::string& error) const
    pub fn add_error(&self, error: String) {
        self.m_errors.lock().unwrap().push(error);
    }

    /// bbs_3mf.cpp:743  void clear_errors() { m_errors.clear(); }
    pub fn clear_errors(&self) {
        self.m_errors.lock().unwrap().clear();
    }

    /// The errors recorded so far (`m_errors`).
    pub fn errors(&self) -> Vec<String> {
        self.m_errors.lock().unwrap().clone()
    }

    /// bbs_3mf.cpp:746-750  void log_errors()
    pub fn log_errors(&self) {
        for error in self.m_errors.lock().unwrap().iter() {
            // bbs_3mf.cpp:749  BOOST_LOG_TRIVIAL(error) << error;
            log::error!("{}", error);
        }
    }
}

// ===========================================================================
// miniz / expat / boost::property_tree stand-ins
// ===========================================================================
//
// FIDELITY-NOTE (archives): the C++ reads and writes the archive with miniz
// (`mz_zip_archive`, miniz_extension.cpp `open_zip_reader` / `open_zip_writer`);
// the port uses the `zip` crate behind the same function names. An entry is
// located as miniz does with flags 0: case-insensitive, first match. The writer
// stages every entry in memory and writes the archive on finalize; the entry
// names, their order and their bytes are the C++ ones, the compressed bytes are
// not (deflate of the `zip` crate, not tdefl).
//
// FIDELITY-NOTE (XML): expat is replaced by quick-xml driven as a SAX parser
// (`expat_parse`): start/end/character callbacks in document order, an empty
// element delivered as a start and an end, attribute values entity-decoded and
// whitespace-normalised as XML 1.0 §3.3.3 requires, CR/CRLF in text turned into
// LF. The messages of malformed documents are expat's message texts for the
// common cases (mismatched tag, no element found, invalid token).

/// `mz_zip_archive_file_stat`: the members bbs_3mf.cpp reads.
#[derive(Debug, Clone, Default)]
pub struct MzZipArchiveFileStat {
    /// `mz_uint32 m_file_index;`
    pub m_file_index: u32,
    /// `char m_filename[MZ_ZIP_MAX_ARCHIVE_FILENAME_SIZE];` (the raw bytes of the name)
    pub m_filename: String,
    /// `mz_uint64 m_uncomp_size;`
    pub m_uncomp_size: u64,
    /// `mz_bool m_is_utf8;`
    ///
    /// FIDELITY-NOTE: the `zip` crate 0.6 does not expose the general purpose flags; a name
    /// that is valid UTF-8 counts as flagged (for ASCII names the two agree: miniz decodes the
    /// unicode-path extra field only when it is present).
    pub m_is_utf8: bool,
}

/// A seekable byte source (a file, or the `std::istream` of `load_gcode_3mf_from_stream`).
pub trait ReadSeek: Read + Seek {}
impl<T: Read + Seek> ReadSeek for T {}

/// `mz_zip_archive` opened for reading.
pub struct MzZipArchive {
    archive: zip::ZipArchive<Box<dyn ReadSeek>>,
    stats: Vec<MzZipArchiveFileStat>,
    extras: Vec<Vec<u8>>,
}

impl MzZipArchive {
    /// `mz_zip_reader_init` over a seekable source.
    fn from_reader(reader: Box<dyn ReadSeek>) -> Option<MzZipArchive> {
        let mut archive = zip::ZipArchive::new(reader).ok()?;
        let mut stats = Vec::with_capacity(archive.len());
        let mut extras = Vec::with_capacity(archive.len());
        for i in 0..archive.len() {
            let f = archive.by_index_raw(i).ok()?;
            let raw = f.name_raw().to_vec();
            let is_utf8 = std::str::from_utf8(&raw).is_ok();
            stats.push(MzZipArchiveFileStat {
                m_file_index: i as u32,
                m_filename: String::from_utf8_lossy(&raw).into_owned(),
                m_uncomp_size: f.size(),
                m_is_utf8: is_utf8,
            });
            extras.push(f.extra_data().to_vec());
        }
        Some(MzZipArchive { archive, stats, extras })
    }

    /// `archive.m_total_files`
    pub fn m_total_files(&self) -> u32 {
        self.stats.len() as u32
    }
}

/// miniz_extension.cpp `open_zip_reader(mz_zip_archive*, const std::string&)`.
pub fn open_zip_reader(path: &str) -> Option<MzZipArchive> {
    let file = std::fs::File::open(crate::utils::encode_path(path)).ok()?;
    MzZipArchive::from_reader(Box::new(file))
}

/// miniz_extension.cpp `close_zip_reader(mz_zip_archive*)` — the archive is dropped.
pub fn close_zip_reader(archive: MzZipArchive) {
    drop(archive);
}

/// `mz_zip_reader_get_num_files`
pub fn mz_zip_reader_get_num_files(archive: &MzZipArchive) -> u32 {
    archive.m_total_files()
}

/// `mz_zip_reader_file_stat`
pub fn mz_zip_reader_file_stat(archive: &MzZipArchive, file_index: u32) -> Option<MzZipArchiveFileStat> {
    archive.stats.get(file_index as usize).cloned()
}

/// `mz_zip_reader_locate_file(archive, name, nullptr, 0)`: flags 0 compare without case.
pub fn mz_zip_reader_locate_file(archive: &MzZipArchive, name: &str) -> i32 {
    for (i, st) in archive.stats.iter().enumerate() {
        if st.m_filename.len() == name.len() && st.m_filename.eq_ignore_ascii_case(name) {
            return i as i32;
        }
    }
    -1
}

/// `mz_zip_reader_get_extra`: the central directory extra field of an entry.
pub fn mz_zip_reader_get_extra(archive: &MzZipArchive, file_index: u32) -> Vec<u8> {
    archive.extras.get(file_index as usize).cloned().unwrap_or_default()
}

/// `mz_zip_reader_extract_to_mem`: the uncompressed bytes of an entry.
pub fn mz_zip_reader_extract_to_mem(archive: &mut MzZipArchive, file_index: u32) -> Option<Vec<u8>> {
    let mut f = archive.archive.by_index(file_index as usize).ok()?;
    let mut buf = Vec::with_capacity(f.size() as usize);
    f.read_to_end(&mut buf).ok()?;
    Some(buf)
}

/// `mz_zip_reader_extract_file_to_mem`: locates `filename` (flags 0), then extracts it.
pub fn mz_zip_reader_extract_file_to_mem(archive: &mut MzZipArchive, filename: &str) -> Option<Vec<u8>> {
    let index = mz_zip_reader_locate_file(archive, filename);
    if index < 0 {
        return None;
    }
    mz_zip_reader_extract_to_mem(archive, index as u32)
}

/// `mz_zip_reader_extract_to_file`
pub fn mz_zip_reader_extract_to_file(archive: &mut MzZipArchive, file_index: u32, dst: &str) -> bool {
    match mz_zip_reader_extract_to_mem(archive, file_index) {
        Some(buf) => std::fs::write(dst, buf).is_ok(),
        None => false,
    }
}

/// One entry staged into an [`MzZipWriter`].
#[derive(Debug, Clone)]
pub struct MzZipWriterEntry {
    pub name: String,
    pub data: Vec<u8>,
    /// `MZ_NO_COMPRESSION` (stored) rather than a deflate level.
    pub stored: bool,
    /// The unicode path extra field written with the entry.
    pub extra: Vec<u8>,
}

/// `mz_zip_archive` opened for writing: the entries in the order they are added.
pub struct MzZipWriter {
    path: String,
    pub entries: Vec<MzZipWriterEntry>,
}

/// miniz_extension.cpp `open_zip_writer(mz_zip_archive*, const std::string&)`.
pub fn open_zip_writer(path: &str) -> Option<MzZipWriter> {
    // the C++ creates the file here
    std::fs::File::create(crate::utils::encode_path(path)).ok()?;
    Some(MzZipWriter { path: path.to_string(), entries: Vec::new() })
}

/// `mz_zip_writer_init_heap`: an archive kept in memory.
pub fn mz_zip_writer_init_heap() -> MzZipWriter {
    MzZipWriter { path: String::new(), entries: Vec::new() }
}

/// `mz_zip_writer_add_mem(archive, name, data, size, level)`
pub fn mz_zip_writer_add_mem(archive: &mut MzZipWriter, name: &str, data: &[u8], stored: bool) -> bool {
    archive.entries.push(MzZipWriterEntry { name: name.to_string(), data: data.to_vec(), stored, extra: Vec::new() });
    true
}

/// `mz_zip_writer_add_from_zip_reader(main, &archive, 0)` for every entry of a heap archive.
pub fn mz_zip_writer_add_from_heap(main: &mut MzZipWriter, archive: MzZipWriter) -> bool {
    main.entries.extend(archive.entries);
    true
}

/// `mz_zip_writer_finalize_archive` + miniz_extension.cpp `close_zip_writer`: writes the
/// staged entries to the file opened by [`open_zip_writer`].
pub fn mz_zip_writer_finalize_archive(archive: &mut MzZipWriter) -> bool {
    if archive.path.is_empty() {
        return true;
    }
    let file = match std::fs::File::create(crate::utils::encode_path(&archive.path)) {
        Ok(f) => f,
        Err(_) => return false,
    };
    let mut zip = zip::ZipWriter::new(file);
    for e in &archive.entries {
        let method = if e.stored { zip::CompressionMethod::Stored } else { zip::CompressionMethod::Deflated };
        let options = zip::write::FileOptions::default()
            .compression_method(method)
            .large_file(e.data.len() as u64 >= 0xFFFF_FFFF);
        let started = if e.extra.is_empty() {
            zip.start_file(e.name.as_str(), options)
        } else {
            zip.start_file_with_extra_data(e.name.as_str(), options).and_then(|_| {
                zip.write_all(&e.extra).map_err(zip::result::ZipError::Io)?;
                zip.end_extra_data().map(|_| ())
            })
        };
        if started.is_err() || zip.write_all(&e.data).is_err() {
            return false;
        }
    }
    zip.finish().is_ok()
}

/// One SAX event of [`expat_parse`].
pub enum SaxEvent<'a> {
    /// `XML_StartElementHandler(name, attributes)`
    Start(&'a str, &'a [(String, String)]),
    /// `XML_EndElementHandler(name)`
    End(&'a str),
    /// `XML_CharacterDataHandler(s, len)`
    Characters(&'a str),
}

/// XML 1.0 §3.3.3 attribute-value normalisation of a raw (undecoded) attribute value: a literal
/// TAB, LF or CR (and a CRLF pair) becomes one space; character references are kept.
fn normalize_raw_attribute(raw: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(raw.len());
    let mut i = 0;
    while i < raw.len() {
        let c = raw[i];
        if c == b'\r' {
            out.push(b' ');
            if i + 1 < raw.len() && raw[i + 1] == b'\n' {
                i += 1;
            }
        } else if c == b'\t' || c == b'\n' {
            out.push(b' ');
        } else {
            out.push(c);
        }
        i += 1;
    }
    out
}

/// XML 1.0 §2.11 end-of-line handling of character data.
fn normalize_line_ends(s: &str) -> std::borrow::Cow<'_, str> {
    if !s.contains('\r') {
        return std::borrow::Cow::Borrowed(s);
    }
    std::borrow::Cow::Owned(s.replace("\r\n", "\n").replace('\r', "\n"))
}

/// The start tag's attributes as expat hands them to the handlers.
fn expat_attributes(e: &quick_xml::events::BytesStart) -> Option<Vec<(String, String)>> {
    let mut atts = Vec::new();
    for attr in e.attributes() {
        let attr = attr.ok()?;
        let key = String::from_utf8_lossy(attr.key.as_ref()).into_owned();
        let raw = normalize_raw_attribute(&attr.value);
        let raw_str = String::from_utf8_lossy(&raw).into_owned();
        let value = quick_xml::escape::unescape(&raw_str).ok()?.into_owned();
        atts.push((key, value));
    }
    Some(atts)
}

/// The outcome of [`expat_parse`] that is not a success.
#[derive(Debug, Clone)]
pub struct ExpatError {
    /// `XML_ErrorString(XML_GetErrorCode(parser))`; "parsing aborted" when a handler stopped the
    /// parser (`XML_StopParser`, `XML_ERROR_ABORTED`).
    pub message: String,
    /// `XML_GetCurrentLineNumber(parser)`
    pub line: usize,
    /// The parser was stopped by a handler rather than by a malformed document.
    pub aborted: bool,
}

/// `XML_Parse` of a complete document: calls `handler` for every event in document order and
/// stops as soon as it returns `false` (`XML_StopParser`).
pub fn expat_parse(data: &[u8], handler: &mut dyn FnMut(SaxEvent) -> bool) -> Result<(), ExpatError> {
    use quick_xml::events::Event;
    let mut reader = quick_xml::Reader::from_reader(data);
    reader.check_end_names(true);
    reader.trim_text(false);
    reader.expand_empty_elements(false);
    let mut buf = Vec::new();
    let mut depth: usize = 0;
    let mut seen_root = false;
    let line_at = |pos: usize| -> usize { data[..pos.min(data.len())].iter().filter(|&&b| b == b'\n').count() + 1 };
    let err = |message: &str, line: usize| ExpatError { message: message.to_string(), line, aborted: false };
    let stop = |line: usize| ExpatError { message: "parsing aborted".to_string(), line, aborted: true };
    loop {
        let pos_before = reader.buffer_position();
        let ev = reader.read_event_into(&mut buf);
        match ev {
            Ok(Event::Start(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let atts = match expat_attributes(&e) {
                    Some(a) => a,
                    None => return Err(err("not well-formed (invalid token)", line_at(pos_before))),
                };
                if depth == 0 && seen_root {
                    return Err(err("junk after document element", line_at(pos_before)));
                }
                depth += 1;
                seen_root = true;
                if !handler(SaxEvent::Start(&name, &atts)) {
                    return Err(stop(line_at(pos_before)));
                }
            }
            Ok(Event::Empty(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let atts = match expat_attributes(&e) {
                    Some(a) => a,
                    None => return Err(err("not well-formed (invalid token)", line_at(pos_before))),
                };
                if depth == 0 && seen_root {
                    return Err(err("junk after document element", line_at(pos_before)));
                }
                seen_root = true;
                if !handler(SaxEvent::Start(&name, &atts)) {
                    return Err(stop(line_at(pos_before)));
                }
                if !handler(SaxEvent::End(&name)) {
                    return Err(stop(line_at(pos_before)));
                }
            }
            Ok(Event::End(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                depth = depth.saturating_sub(1);
                if !handler(SaxEvent::End(&name)) {
                    return Err(stop(line_at(pos_before)));
                }
            }
            Ok(Event::Text(t)) => {
                let raw = String::from_utf8_lossy(t.as_ref()).into_owned();
                if depth == 0 {
                    if raw.trim_matches(|c| c == ' ' || c == '\t' || c == '\n' || c == '\r').is_empty() {
                        continue;
                    }
                    return Err(err("not well-formed (invalid token)", line_at(pos_before)));
                }
                let text = match quick_xml::escape::unescape(&raw) {
                    Ok(t) => t.into_owned(),
                    Err(_) => return Err(err("undefined entity", line_at(pos_before))),
                };
                let text = normalize_line_ends(&text).into_owned();
                if !text.is_empty() && !handler(SaxEvent::Characters(&text)) {
                    return Err(stop(line_at(pos_before)));
                }
            }
            Ok(Event::CData(t)) => {
                let text = String::from_utf8_lossy(t.as_ref()).into_owned();
                let text = normalize_line_ends(&text).into_owned();
                if depth > 0 && !text.is_empty() && !handler(SaxEvent::Characters(&text)) {
                    return Err(stop(line_at(pos_before)));
                }
            }
            Ok(Event::Decl(_)) | Ok(Event::Comment(_)) | Ok(Event::PI(_)) | Ok(Event::DocType(_)) => {}
            Ok(Event::Eof) => {
                if depth > 0 || !seen_root {
                    return Err(err("no element found", line_at(data.len())));
                }
                return Ok(());
            }
            Err(quick_xml::Error::EndEventMismatch { .. }) => {
                return Err(err("mismatched tag", line_at(reader.buffer_position())));
            }
            Err(_) => {
                return Err(err("not well-formed (invalid token)", line_at(reader.buffer_position())));
            }
        }
        buf.clear();
    }
}

/// `boost::property_tree::ptree` as read_xml / write_xml use it: the data and the children in
/// order (attributes under `<xmlattr>`, comments under `<xmlcomment>`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PTree {
    pub data: String,
    pub children: Vec<(String, PTree)>,
}

impl PTree {
    /// `ptree::empty()`: no children.
    pub fn empty(&self) -> bool {
        self.children.is_empty()
    }

    /// `ptree::get_child_optional(path)` with '.' separated keys (first match of each key).
    pub fn get_child(&self, path: &str) -> Option<&PTree> {
        let mut cur = self;
        for key in path.split('.') {
            cur = &cur.children.iter().find(|(k, _)| k == key)?.1;
        }
        Some(cur)
    }

    /// `ptree::find(key)` among the direct children.
    pub fn find(&self, key: &str) -> Option<&PTree> {
        self.children.iter().find(|(k, _)| k == key).map(|(_, v)| v)
    }

    /// `ptree::get<std::string>(path)` — the data at `path`.
    pub fn get_string(&self, path: &str) -> Option<String> {
        self.get_child(path).map(|c| c.data.clone())
    }

    /// `ptree::get<T>(path)` through the stream translator: `istream >> value`, then only
    /// trailing white space may follow.
    pub fn get_value<T: std::str::FromStr>(&self, path: &str) -> Option<T> {
        let data = self.get_child(path)?.data.clone();
        let t = data.trim_start_matches(|c: char| c.is_ascii_whitespace());
        let t = t.trim_end_matches(|c: char| c.is_ascii_whitespace());
        t.parse::<T>().ok()
    }

    /// `ptree::get<T>(path, default_value)`
    pub fn get_or<T: std::str::FromStr>(&self, path: &str, default_value: T) -> T {
        self.get_value(path).unwrap_or(default_value)
    }

    /// `ptree::add(path, value)`: walks to the parent of the last key (the first child of each
    /// key, created when missing) and appends a new child.
    pub fn add(&mut self, path: &str, value: &str) -> &mut PTree {
        let keys: Vec<&str> = path.split('.').collect();
        let mut cur: &mut PTree = self;
        for key in &keys[..keys.len() - 1] {
            let idx = match cur.children.iter().position(|(k, _)| k == key) {
                Some(i) => i,
                None => {
                    cur.children.push((key.to_string(), PTree::default()));
                    cur.children.len() - 1
                }
            };
            cur = &mut cur.children[idx].1;
        }
        cur.children.push((keys[keys.len() - 1].to_string(), PTree { data: value.to_string(), children: Vec::new() }));
        let last = cur.children.len() - 1;
        &mut cur.children[last].1
    }

    /// `ptree::put(path, value)`: sets the data of the first child at `path`, creating it.
    pub fn put(&mut self, path: &str, value: &str) -> &mut PTree {
        let mut cur: &mut PTree = self;
        for key in path.split('.') {
            let idx = match cur.children.iter().position(|(k, _)| k == key) {
                Some(i) => i,
                None => {
                    cur.children.push((key.to_string(), PTree::default()));
                    cur.children.len() - 1
                }
            };
            cur = &mut cur.children[idx].1;
        }
        cur.data = value.to_string();
        cur
    }
}

/// `pt::read_xml(istream, ptree)` with the default flags (rapidxml without whitespace trimming:
/// text and CDATA are appended to the element's data, comments are kept, the declaration is
/// skipped). `None` when the document is malformed (the C++ throws `xml_parser_error`).
pub fn ptree_read_xml(text: &[u8]) -> Option<PTree> {
    use quick_xml::events::Event;
    let mut reader = quick_xml::Reader::from_reader(text);
    reader.check_end_names(true);
    reader.trim_text(false);
    let mut buf = Vec::new();
    let mut stack: Vec<(String, PTree)> = vec![(String::new(), PTree::default())];
    let push_element = |stack: &mut Vec<(String, PTree)>, e: &quick_xml::events::BytesStart| -> Option<()> {
        let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
        let mut node = PTree::default();
        let mut attrs = PTree::default();
        for attr in e.attributes() {
            let attr = attr.ok()?;
            let key = String::from_utf8_lossy(attr.key.as_ref()).into_owned();
            let raw = String::from_utf8_lossy(&attr.value).into_owned();
            let value = quick_xml::escape::unescape(&raw).ok()?.into_owned();
            attrs.children.push((key, PTree { data: value, children: Vec::new() }));
        }
        if !attrs.children.is_empty() {
            node.children.push(("<xmlattr>".to_string(), attrs));
        }
        stack.push((name, node));
        Some(())
    };
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(e)) => push_element(&mut stack, &e)?,
            Ok(Event::Empty(e)) => {
                push_element(&mut stack, &e)?;
                let (name, node) = stack.pop()?;
                stack.last_mut()?.1.children.push((name, node));
            }
            Ok(Event::End(_)) => {
                let (name, node) = stack.pop()?;
                if stack.is_empty() {
                    return None;
                }
                stack.last_mut()?.1.children.push((name, node));
            }
            Ok(Event::Text(t)) => {
                if stack.len() > 1 {
                    let raw = String::from_utf8_lossy(t.as_ref()).into_owned();
                    let v = quick_xml::escape::unescape(&raw).ok()?.into_owned();
                    stack.last_mut()?.1.data.push_str(&v);
                }
            }
            Ok(Event::CData(t)) => {
                if stack.len() > 1 {
                    stack.last_mut()?.1.data.push_str(&String::from_utf8_lossy(t.as_ref()));
                }
            }
            Ok(Event::Comment(t)) => {
                let v = String::from_utf8_lossy(t.as_ref()).into_owned();
                stack.last_mut()?.1.children.push(("<xmlcomment>".to_string(), PTree { data: v, children: Vec::new() }));
            }
            Ok(Event::Decl(_)) | Ok(Event::PI(_)) | Ok(Event::DocType(_)) => {}
            Ok(Event::Eof) => break,
            Err(_) => return None,
        }
        buf.clear();
    }
    if stack.len() != 1 {
        return None;
    }
    Some(stack.pop()?.1)
}

/// boost xml_parser_utils.hpp `encode_char_entities`.
fn ptree_encode_char_entities(s: &str) -> String {
    if s.is_empty() {
        return String::new();
    }
    if s.bytes().all(|b| b == b' ') {
        let mut r = "&#32;".to_string();
        r.push_str(&" ".repeat(s.len() - 1));
        return r;
    }
    let mut r = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '<' => r.push_str("&lt;"),
            '>' => r.push_str("&gt;"),
            '&' => r.push_str("&amp;"),
            '"' => r.push_str("&quot;"),
            '\'' => r.push_str("&apos;"),
            _ => r.push(c),
        }
    }
    r
}

/// boost xml_parser_write.hpp `write_xml_element` without indentation (the default settings).
fn ptree_write_xml_element(out: &mut String, key: &str, pt: &PTree, indent: i32) {
    let mut has_attrs_only = pt.data.is_empty();
    for (k, _) in &pt.children {
        if k != "<xmlattr>" {
            has_attrs_only = false;
            if k != "<xmltext>" {
                break;
            }
        }
    }
    if pt.data.is_empty() && pt.children.is_empty() {
        if indent >= 0 {
            out.push('<');
            out.push_str(key);
            out.push_str("/>");
        }
    } else {
        if indent >= 0 {
            out.push('<');
            out.push_str(key);
            if let Some(attribs) = pt.find("<xmlattr>") {
                for (k, v) in &attribs.children {
                    out.push(' ');
                    out.push_str(k);
                    out.push_str("=\"");
                    out.push_str(&ptree_encode_char_entities(&v.data));
                    out.push('"');
                }
            }
            if has_attrs_only {
                out.push_str("/>");
            } else {
                out.push('>');
            }
        }
        if !pt.data.is_empty() {
            out.push_str(&ptree_encode_char_entities(&pt.data));
        }
        for (k, v) in &pt.children {
            if k == "<xmlattr>" {
                continue;
            } else if k == "<xmlcomment>" {
                out.push_str("<!--");
                out.push_str(&v.data);
                out.push_str("-->");
            } else if k == "<xmltext>" {
                out.push_str(&ptree_encode_char_entities(&v.data));
            } else {
                ptree_write_xml_element(out, k, v, indent + 1);
            }
        }
        if indent >= 0 && !has_attrs_only {
            out.push_str("</");
            out.push_str(key);
            out.push('>');
        }
    }
}

/// `pt::write_xml(ostream, ptree)` with the default settings.
pub fn ptree_write_xml(pt: &PTree) -> String {
    let mut out = String::from("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    ptree_write_xml_element(&mut out, "", pt, -1);
    out
}

/// The text a ptree stores for `put(path, double)`: the stream translator prints with
/// `max_digits10` (17) significant digits.
fn ptree_double(v: f64) -> String {
    general_format(v, 17)
}

/// The text a ptree stores for `put(path, float)`: `max_digits10` (9) significant digits.
fn ptree_float(v: f32) -> String {
    general_format(v as f64, 9)
}

/// `std::ostream << double` with the stream's precision (default 6), `%g`.
fn stream_double(v: f64, precision: usize) -> String {
    general_format(v, precision)
}

/// `std::ostream << float` with the stream's precision (default 6): the float is promoted.
fn stream_float(v: f32, precision: usize) -> String {
    general_format(v as f64, precision)
}

/// `std::ostream << std::fixed << std::setprecision(p) << v`
fn stream_fixed(v: f64, precision: usize) -> String {
    format!("{:.*}", precision, v)
}

/// `std::ostream << std::boolalpha << b`
fn stream_boolalpha(b: bool) -> &'static str {
    if b {
        "true"
    } else {
        "false"
    }
}

/// `std::ostream << b` without boolalpha.
fn stream_bool(b: bool) -> &'static str {
    if b {
        "1"
    } else {
        "0"
    }
}

/// `std::istringstream(value) >> std::boolalpha >> b`: "true" / "false"; any other text leaves
/// the target unchanged on failure (C++11: a failed extraction stores false).
fn istream_boolalpha(value: &str, target: &mut bool) {
    let t = value.trim_start();
    if t.starts_with("true") {
        *target = true;
    } else {
        // "false", or a failed extraction (which stores false since C++11)
        *target = false;
    }
}

/// `std::stringstream(str) >> value` repeatedly: the leading integers of a white space separated
/// list (stops at the first token that is not an integer).
fn get_vector_from_string(s: &str) -> Vec<i32> {
    let mut results = Vec::new();
    let b = s.as_bytes();
    let mut i = 0;
    loop {
        while i < b.len() && (b[i] as char).is_ascii_whitespace() {
            i += 1;
        }
        if i >= b.len() {
            break;
        }
        let start = i;
        if b[i] == b'+' || b[i] == b'-' {
            i += 1;
        }
        let digits = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        if i == digits {
            break;
        }
        match s[start..i].parse::<i64>() {
            Ok(v) if v >= i32::MIN as i64 && v <= i32::MAX as i64 => results.push(v as i32),
            _ => break,
        }
    }
    results
}

/// `boost::algorithm::iequals` for ASCII.
fn iequals(a: &str, b: &str) -> bool {
    a.len() == b.len() && a.eq_ignore_ascii_case(b)
}

/// `boost::algorithm::istarts_with`
fn istarts_with(s: &str, prefix: &str) -> bool {
    s.len() >= prefix.len() && s.as_bytes()[..prefix.len()].eq_ignore_ascii_case(prefix.as_bytes())
}

/// `boost::algorithm::iends_with`
fn iends_with(s: &str, suffix: &str) -> bool {
    s.len() >= suffix.len() && s.as_bytes()[s.len() - suffix.len()..].eq_ignore_ascii_case(suffix.as_bytes())
}

/// Eigen `isApprox` of two 4x4 matrices: `(a - b).norm() <= prec * min(a.norm(), b.norm())`.
fn transform_is_approx(a: &Transform3d, b: &Transform3d, prec: f64) -> bool {
    (a - b).norm() <= prec * a.norm().min(b.norm())
}

/// Eigen `Vec3d::isApprox(other)` with the default precision (1e-12):
/// `(a - b).squaredNorm() <= prec^2 * min(a.squaredNorm(), b.squaredNorm())`.
fn vec3d_is_approx(a: &Vec3d, b: &Vec3d) -> bool {
    let prec = 1e-12_f64;
    (a - b).norm_squared() <= prec * prec * a.norm_squared().min(b.norm_squared())
}

/// `std::istringstream(uuid) >> std::hex >> backup_id`: the leading hexadecimal digits.
fn istream_hex_int(s: &str) -> Option<i32> {
    let t = s.trim_start();
    let t = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")).unwrap_or(t);
    let end = t.find(|c: char| !c.is_ascii_hexdigit()).unwrap_or(t.len());
    if end == 0 {
        return None;
    }
    // operator>>(int&) with std::hex reads into a long and fails when the value does not fit
    let v = i64::from_str_radix(&t[..end], 16).ok()?;
    if v > i32::MAX as i64 {
        return None;
    }
    Some(v as i32)
}

/// `fast_float::from_chars(first, last, float&)` with the general format: an optional '-', the
/// digits with an optional '.', an optional exponent (or "inf"/"infinity"/"nan"), as a prefix of
/// `text` without leading white space, rounded to the nearest float. `None` when no number starts
/// the text (the target is left as it was).
fn fast_float_from_chars_f32(text: &str) -> Option<f32> {
    let b = text.as_bytes();
    let mut i = 0;
    let neg = if i < b.len() && b[i] == b'-' {
        i += 1;
        true
    } else {
        false
    };
    let rest = &text[i..];
    let lower: String = rest.chars().take(8).collect::<String>().to_ascii_lowercase();
    if lower.starts_with("inf") || lower.starts_with("nan") {
        let v = if lower.starts_with("nan") { f32::NAN } else { f32::INFINITY };
        return Some(if neg { -v } else { v });
    }
    let int_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    let mut digits = i - int_start;
    if i < b.len() && b[i] == b'.' {
        i += 1;
        let frac_start = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        digits += i - frac_start;
    }
    if digits == 0 {
        return None;
    }
    let mut end = i;
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let mut j = i + 1;
        if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
            j += 1;
        }
        let exp_start = j;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_start {
            end = j;
        }
    }
    let mut s = text[int_start..end].to_string();
    if s.starts_with('.') {
        s.insert(0, '0');
    }
    let v: f32 = s.parse().ok()?;
    Some(if neg { -v } else { v })
}

// ===========================================================================
// bbs_3mf.cpp:198-214 — tag helpers
// ===========================================================================

/// bbs_3mf.cpp:204-208 `static const char* local_tag_name(const char* name)`: the name without
/// its namespace prefix.
fn local_tag_name(name: &str) -> &str {
    match name.rfind(':') {
        Some(p) => &name[p + 1..],
        None => name,
    }
}

/// bbs_3mf.cpp:210-213 `static bool is_texture_resource_tag(const char* name)`
fn is_texture_resource_tag(name: &str) -> bool {
    let local_name = local_tag_name(name);
    local_name == "texture2d" || local_name == "texture2dgroup"
}

/// An exception escaping the C++ reader or writer (`Slic3r::FileIOError`, a configuration
/// exception of `set_deserialize`, a ptree exception): the load or save is abandoned.
#[derive(Debug, Clone)]
pub struct Bbs3mfException(pub String);

impl std::fmt::Display for Bbs3mfException {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for Bbs3mfException {}

impl From<crate::config::ConfigurationError> for Bbs3mfException {
    fn from(e: crate::config::ConfigurationError) -> Self {
        Bbs3mfException(e.to_string())
    }
}

/// bbs_3mf.cpp:829-915 `void PlateData::parse_filament_info(GCodeProcessorResult *result)` —
/// see [`PlateData::parse_filament_info`].
impl PlateData {
    /// bbs_3mf.cpp:829-915
    ///
    /// FIDELITY-NOTE: `GCodeProcessorResult` of the crate is another unit; the members this
    /// function reads are passed in as [`ParseFilamentInfoInput`].
    pub fn parse_filament_info(&mut self, result: Option<&ParseFilamentInfoInput>) {
        let result = match result {
            Some(r) => r,
            None => return,
        };
        let m_filament_diameters = &result.filament_diameters;
        let m_filament_densities = &result.filament_densities;
        let get_used_filament_from_volume = |volume: f64, extruder_id: i32| -> (f64, f64) {
            let koef = 0.001;
            let d = 0.5 * m_filament_diameters[extruder_id as usize] as f64;
            let section_area = std::f64::consts::PI * (d * d);
            (
                if section_area < crate::libslic3r::EPSILON { 0.0 } else { koef * volume / section_area },
                volume * m_filament_densities[extruder_id as usize] as f64 * 0.001,
            )
        };
        let directly_used_filaments: BTreeSet<u32> = result.directly_used_filaments.iter().cloned().collect();
        let has_direct_usage_hint = !result.directly_used_filaments.is_empty() || result.has_used_mixed_filaments;

        for (&first, &volume) in result.total_volumes_per_extruder.iter() {
            let (used_filament_m, used_filament_g) = get_used_filament_from_volume(volume, first);
            let filament_info = result.used_filaments.iter().find(|item| item.0 == first);
            let mut info = FilamentInfo::default();
            info.id = first;
            info.used_g = used_filament_g as f32;
            info.used_m = used_filament_m as f32;
            let mut extra = FilamentInfoExtra::default();
            if let Some(t) = result.load_time_per_filament.get(&first) {
                extra.total_load_time = *t;
            }
            if let Some(t) = result.unload_time_per_filament.get(&first) {
                extra.total_unload_time = *t;
            }
            extra.directly_used = !has_direct_usage_hint || directly_used_filaments.contains(&(first as u32));

            if let Some(nozzles_for_filament) = result.nozzles_for_filament.get(&first) {
                if !nozzles_for_filament.is_empty() {
                    info.group_id.reserve(nozzles_for_filament.len());
                    let mut diameters: Vec<f64> = Vec::new();
                    let mut volume_types: BTreeSet<i32> = BTreeSet::new();
                    for nozzle in nozzles_for_filament {
                        info.group_id.push(nozzle.group_id);
                        let (d, _) = crate::locales_utils::string_to_double_decimal_point(&nozzle.diameter);
                        diameters.push(d);
                        volume_types.insert(nozzle.volume_type as i32);
                    }
                    info.group_id.sort();
                    info.group_id.dedup();
                    // std::set<double>: the smallest
                    if let Some(min) = diameters.iter().cloned().fold(None, |acc: Option<f64>, d| match acc {
                        None => Some(d),
                        Some(a) => Some(if d < a { d } else { a }),
                    }) {
                        info.nozzle_diameter = min;
                    }
                    use crate::config::ConfigEnum;
                    if volume_types.len() > 1 {
                        info.nozzle_volume_type =
                            crate::print_config::config_def::get_nozzle_volume_type_string(crate::print_config::enums::NozzleVolumeType::NvtHybrid);
                    } else if let Some(&vt) = volume_types.iter().next() {
                        info.nozzle_volume_type = crate::print_config::config_def::get_nozzle_volume_type_string(
                            crate::print_config::enums::NozzleVolumeType::from_int(vt),
                        );
                    }
                }
            }

            if let Some(fi) = filament_info {
                info.used_for_support = fi.1;
                info.used_for_object = fi.2;
            }

            self.slice_filaments_info.push(info);
            self.slice_filaments_extra.push(extra);
        }

        if let Some(g) = &result.layered_nozzle_group_result {
            self.nozzle_group_result = Some(g.clone());
        }

        self.warnings = result.warnings.clone();
    }
}

/// The members of `GCodeProcessorResult` that bbs_3mf.cpp:829-915 `PlateData::parse_filament_info`
/// reads.
#[derive(Clone, Default)]
pub struct ParseFilamentInfoInput {
    /// `result->filament_diameters`
    pub filament_diameters: Vec<f32>,
    /// `result->filament_densities`
    pub filament_densities: Vec<f32>,
    /// `result->print_statistics.total_volumes_per_extruder` (ordered as the C++ std::map)
    pub total_volumes_per_extruder: BTreeMap<i32, f64>,
    /// `result->print_statistics.load_time_per_filament`
    pub load_time_per_filament: BTreeMap<i32, f64>,
    /// `result->print_statistics.unload_time_per_filament`
    pub unload_time_per_filament: BTreeMap<i32, f64>,
    /// `result->used_filaments`: (filament_id, use_for_support, use_for_object)
    pub used_filaments: Vec<(i32, bool, bool)>,
    /// `result->directly_used_filaments`
    pub directly_used_filaments: Vec<u32>,
    /// `!result->used_mixed_filaments.empty()`
    pub has_used_mixed_filaments: bool,
    /// `result->nozzle_group_result->get_nozzles_for_filament(id)` per filament
    pub nozzles_for_filament: BTreeMap<i32, Vec<NozzleInfo>>,
    /// `std::dynamic_pointer_cast<LayeredNozzleGroupResult>(result->nozzle_group_result)`
    pub layered_nozzle_group_result: Option<LayeredNozzleGroupResult>,
    /// `result->warnings`
    pub warnings: Vec<SliceWarning>,
}

// ===========================================================================
// bbs_3mf.cpp:923-1055 class MaterialColorResources
// ===========================================================================

/// bbs_3mf.cpp:1024-1029 `struct Composite`
#[derive(Debug, Clone, Default)]
struct Composite {
    matid: i32,
    matindices: Vec<i32>,
    values: Vec<Vec<f32>>,
}

/// bbs_3mf.cpp:1030-1034 `struct Multi`
#[derive(Debug, Clone, Default)]
struct Multi {
    pids: Vec<i32>,
    pindices: Vec<Vec<i32>>,
}

/// bbs_3mf.cpp:1035 `enum class Parent`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MaterialParent {
    None,
    BaseMaterials,
    CompositeMaterials,
    MultiProperties,
}

/// basematerials, compositematerials and multiproperties of a standard 3MF, read into the
/// color group map as one color per entry, so triangles bound to them resolve like the ones
/// bound to an m:colorgroup. An empty string marks an entry without color. Composites and
/// multiproperties may reference resources of another model file, so they are resolved
/// once every file is parsed.
/// bbs_3mf.cpp:928-1055
#[derive(Debug, Clone)]
pub struct MaterialColorResources {
    m_composites: BTreeMap<i32, Composite>,
    m_multis: BTreeMap<i32, Multi>,
    m_has_textures: bool,
    m_parent: MaterialParent,
    m_parent_id: i32,
}

impl Default for MaterialColorResources {
    fn default() -> Self {
        Self {
            m_composites: BTreeMap::new(),
            m_multis: BTreeMap::new(),
            m_has_textures: false,
            m_parent: MaterialParent::None,
            m_parent_id: -1,
        }
    }
}

impl MaterialColorResources {
    /// bbs_3mf.cpp:931
    pub fn has_textures(&self) -> bool {
        self.m_has_textures
    }

    /// bbs_3mf.cpp:933-961
    pub fn start_element(&mut self, name: &str, attributes: &[(String, String)], group_colors: &mut BTreeMap<i32, Vec<String>>) {
        let local_name = local_tag_name(name);
        if local_name == "basematerials" {
            self.m_parent = MaterialParent::BaseMaterials;
            self.m_parent_id = bbs_get_attribute_value_int(attributes, ID_ATTR);
        } else if local_name == "base" && self.m_parent == MaterialParent::BaseMaterials {
            group_colors.entry(self.m_parent_id).or_default().push(bbs_get_attribute_value_string(attributes, "displaycolor"));
        } else if local_name == "compositematerials" {
            self.m_parent = MaterialParent::CompositeMaterials;
            self.m_parent_id = bbs_get_attribute_value_int(attributes, ID_ATTR);
            let composite = self.m_composites.entry(self.m_parent_id).or_insert_with(|| Composite { matid: -1, ..Default::default() });
            composite.matid = bbs_get_attribute_value_int(attributes, "matid");
            composite.matindices = Self::parse_numbers::<i32>(&bbs_get_attribute_value_string(attributes, "matindices"));
        } else if local_name == "composite" && self.m_parent == MaterialParent::CompositeMaterials {
            let values = Self::parse_numbers::<f32>(&bbs_get_attribute_value_string(attributes, "values"));
            self.m_composites.entry(self.m_parent_id).or_insert_with(|| Composite { matid: -1, ..Default::default() }).values.push(values);
        } else if local_name == "multiproperties" {
            self.m_parent = MaterialParent::MultiProperties;
            self.m_parent_id = bbs_get_attribute_value_int(attributes, ID_ATTR);
            self.m_multis.entry(self.m_parent_id).or_default().pids =
                Self::parse_numbers::<i32>(&bbs_get_attribute_value_string(attributes, "pids"));
        } else if local_name == "multi" && self.m_parent == MaterialParent::MultiProperties {
            let pindices = Self::parse_numbers::<i32>(&bbs_get_attribute_value_string(attributes, "pindices"));
            self.m_multis.entry(self.m_parent_id).or_default().pindices.push(pindices);
        } else if is_texture_resource_tag(name) {
            self.m_has_textures = true;
        }
    }

    /// bbs_3mf.cpp:963-969
    pub fn end_element(&mut self, name: &str) {
        let local_name = local_tag_name(name);
        if local_name == "basematerials" || local_name == "compositematerials" || local_name == "multiproperties" {
            self.m_parent = MaterialParent::None;
        }
    }

    /// bbs_3mf.cpp:971-976 (`std::map::insert` keeps the entries already present)
    pub fn merge(&mut self, other: &MaterialColorResources) {
        for (k, v) in &other.m_composites {
            self.m_composites.entry(*k).or_insert_with(|| v.clone());
        }
        for (k, v) in &other.m_multis {
            self.m_multis.entry(*k).or_insert_with(|| v.clone());
        }
        self.m_has_textures |= other.m_has_textures;
    }

    /// bbs_3mf.cpp:978-1021
    pub fn resolve(&self, group_colors: &mut BTreeMap<i32, Vec<String>>) {
        // An out-of-range index takes the first entry, as it does for triangles.
        let color_at = |group_colors: &BTreeMap<i32, Vec<String>>, pid: i32, index: i32| -> Option<String> {
            let group = group_colors.get(&pid)?;
            if group.is_empty() {
                return None;
            }
            let color = &group[if index >= 0 && (index as usize) < group.len() { index as usize } else { 0 }];
            if color.is_empty() {
                None
            } else {
                Some(color.clone())
            }
        };
        // A composite mixes the display colors of its base materials by their ratios.
        for (id, composite) in &self.m_composites {
            let mut colors = Vec::new();
            for values in &composite.values {
                let mut mixed = [0.0f32; 3];
                let mut total = 0.0f32;
                let mut i = 0;
                while i < values.len() && i < composite.matindices.len() {
                    let color = color_at(group_colors, composite.matid, composite.matindices[i]);
                    if color.is_none() || values[i] <= 0.0 {
                        i += 1;
                        continue;
                    }
                    let rgba = crate::color::convert_color_string_to_rgba(color.as_deref().unwrap());
                    for c in 0..3 {
                        mixed[c] += rgba[c] * values[i];
                    }
                    total += values[i];
                    i += 1;
                }
                colors.push(if total > 0.0 {
                    crate::color::encode_color_rgb(&crate::color::ColorRGB::new(mixed[0] / total, mixed[1] / total, mixed[2] / total))
                } else {
                    String::new()
                });
            }
            group_colors.insert(*id, colors);
        }
        // Each layer of a multiproperties entry holds one property. Texture layers have no
        // color and are dropped; an upper color layer covers the lower ones.
        for (id, multi) in &self.m_multis {
            let mut colors = Vec::new();
            for pindices in &multi.pindices {
                let mut top_color: Option<String> = None;
                for layer in 0..multi.pids.len() {
                    if let Some(color) =
                        color_at(group_colors, multi.pids[layer], if layer < pindices.len() { pindices[layer] } else { 0 })
                    {
                        top_color = Some(color);
                    }
                }
                colors.push(top_color.unwrap_or_default());
            }
            group_colors.insert(*id, colors);
        }
    }

    /// bbs_3mf.cpp:1037-1048: space separated numbers, as in matindices, values, pids and
    /// pindices (`std::strtod` until it stops consuming).
    fn parse_numbers<T: FromF64>(s: &str) -> Vec<T> {
        let mut numbers = Vec::new();
        let b = s.as_bytes();
        let mut begin = 0usize;
        loop {
            let (number, consumed) = objparser::strtod(&b[begin..], 0);
            if consumed == 0 {
                break;
            }
            numbers.push(T::from_f64(number));
            begin += consumed;
        }
        numbers
    }
}

/// `static_cast<T>(double)` for the element types of `parse_numbers`.
trait FromF64 {
    fn from_f64(v: f64) -> Self;
}
impl FromF64 for i32 {
    fn from_f64(v: f64) -> Self {
        v as i32
    }
}
impl FromF64 for f32 {
    fn from_f64(v: f64) -> Self {
        v as f32
    }
}

// ===========================================================================
// bbs_3mf.cpp:1075-1709 class _BBS_3MF_Importer
// ===========================================================================

/// bbs_3mf.cpp:1077 `typedef std::pair<std::string, int> Id; // BBS: encrypt`
type Id = (String, i32);

/// bbs_3mf.cpp:1079-1095 `struct Component`
#[derive(Debug, Clone)]
struct Component {
    object_id: Id,
    transform: Transform3d,
}

impl Component {
    /// bbs_3mf.cpp:1090-1094
    fn new(object_id: Id, transform: Transform3d) -> Self {
        Self { object_id, transform }
    }
}

/// bbs_3mf.cpp:1099-1135 `struct Geometry`
#[derive(Debug, Clone, Default)]
struct Geometry {
    vertices: Vec<Vec3f>,
    triangles: Vec<crate::triangle_mesh::Vec3i>,
    custom_supports: Vec<String>,
    custom_fuzzy_skin: Vec<String>,
    custom_seam: Vec<String>,
    mmu_segmentation: Vec<String>,
    // BBS
    face_properties: Vec<String>,
    triangle_colors: Vec<crate::format::obj::TriangleColor>,
}

impl Geometry {
    /// bbs_3mf.cpp:1111
    fn empty(&self) -> bool {
        self.vertices.is_empty() || self.triangles.is_empty()
    }

    /// bbs_3mf.cpp:1114-1123 — backup & restore
    #[allow(dead_code)]
    fn swap(&mut self, o: &mut Geometry) {
        std::mem::swap(&mut self.vertices, &mut o.vertices);
        std::mem::swap(&mut self.triangles, &mut o.triangles);
        std::mem::swap(&mut self.custom_supports, &mut o.custom_supports);
        std::mem::swap(&mut self.custom_fuzzy_skin, &mut o.custom_fuzzy_skin);
        std::mem::swap(&mut self.custom_seam, &mut o.custom_seam);
        std::mem::swap(&mut self.mmu_segmentation, &mut o.mmu_segmentation);
        std::mem::swap(&mut self.face_properties, &mut o.face_properties);
        std::mem::swap(&mut self.triangle_colors, &mut o.triangle_colors);
    }

    /// bbs_3mf.cpp:1125-1134
    fn reset(&mut self) {
        self.vertices.clear();
        self.triangles.clear();
        self.custom_supports.clear();
        self.custom_fuzzy_skin.clear();
        self.custom_seam.clear();
        self.mmu_segmentation.clear();
        self.face_properties.clear();
        self.triangle_colors.clear();
    }
}

/// bbs_3mf.cpp:1137-1169 `struct CurrentObject`
#[derive(Debug, Clone)]
struct CurrentObject {
    /// ID of the object inside the 3MF file, 1 based.
    id: i32,
    /// Index of the ModelObject in its respective Model, zero based.
    model_object_idx: i32,
    geometry: Geometry,
    /// `ModelObject* object` — the index of the object in the model.
    object: Option<usize>,
    components: Vec<Component>,
    name: String,
    uuid: String,
    pid: i32,
    pindex: i32,
}

impl CurrentObject {
    /// bbs_3mf.cpp:1155
    fn new() -> Self {
        let mut o = CurrentObject {
            id: -1,
            model_object_idx: -1,
            geometry: Geometry::default(),
            object: None,
            components: Vec::new(),
            name: String::new(),
            uuid: String::new(),
            pid: -1,
            pindex: -1,
        };
        o.reset();
        o
    }

    /// bbs_3mf.cpp:1157-1168
    fn reset(&mut self) {
        self.id = -1;
        self.model_object_idx = -1;
        self.geometry.reset();
        self.object = None;
        self.components.clear();
        //BBS: sub object id
        self.uuid.clear();
        self.name.clear();
        self.pid = -1;
        self.pindex = -1;
    }
}

/// bbs_3mf.cpp:1171-1175 `struct CurrentConfig`
#[derive(Debug, Clone, Copy)]
struct CurrentConfig {
    object_id: i32,
    volume_id: i32,
}

/// bbs_3mf.cpp:1177-1182 `struct CurrentInstance`
#[derive(Debug, Clone, Copy)]
struct CurrentInstance {
    object_id: i32,
    instance_id: i32,
    identify_id: i32,
}

/// bbs_3mf.cpp:1184-1194 `struct Instance` — `ModelInstance* instance` is the object and
/// instance indices in the model.
#[derive(Debug, Clone)]
struct ImporterInstance {
    object_idx: usize,
    instance_idx: usize,
    transform: Transform3d,
}

/// bbs_3mf.cpp:1196-1206 `struct Metadata`
#[derive(Debug, Clone)]
struct Metadata {
    key: String,
    value: String,
}

impl Metadata {
    fn new(key: &str, value: &str) -> Self {
        Self { key: key.to_string(), value: value.to_string() }
    }
}

/// bbs_3mf.cpp:1212-1238 `struct ObjectMetadata::VolumeMetadata`
#[derive(Debug, Clone)]
struct VolumeMetadata {
    //BBS: refine the part logic
    first_triangle_id: u32,
    last_triangle_id: u32,
    subobject_id: i32,
    metadata: Vec<Metadata>,
    mesh_stats: crate::triangle_mesh::RepairedMeshErrors,
    part_type: ModelVolumeType,
    text_info: crate::model::TextInfo,
    shape_configuration: Option<crate::emboss_shape::EmbossShape>,
}

impl VolumeMetadata {
    /// bbs_3mf.cpp:1223-1229
    fn with_triangles(first_triangle_id: u32, last_triangle_id: u32, type_: ModelVolumeType) -> Self {
        Self {
            first_triangle_id,
            last_triangle_id,
            subobject_id: -1,
            metadata: Vec::new(),
            mesh_stats: crate::triangle_mesh::RepairedMeshErrors::default(),
            part_type: type_,
            text_info: crate::model::TextInfo::default(),
            shape_configuration: None,
        }
    }

    /// bbs_3mf.cpp:1231-1237
    fn with_subobject(sub_id: i32, type_: ModelVolumeType) -> Self {
        Self {
            first_triangle_id: 0,
            last_triangle_id: 0,
            subobject_id: sub_id,
            metadata: Vec::new(),
            mesh_stats: crate::triangle_mesh::RepairedMeshErrors::default(),
            part_type: type_,
            text_info: crate::model::TextInfo::default(),
            shape_configuration: None,
        }
    }
}

/// bbs_3mf.cpp:1210-1244 `struct ObjectMetadata`
#[derive(Debug, Clone, Default)]
struct ObjectMetadata {
    metadata: Vec<Metadata>,
    volumes: Vec<VolumeMetadata>,
}

/// bbs_3mf.cpp:1248-1256 `struct CutObjectInfo::Connector`
#[derive(Debug, Clone, Copy)]
struct CutConnectorInfo {
    volume_id: i32,
    type_: i32,
    radius: f32,
    height: f32,
    r_tolerance: f32,
    h_tolerance: f32,
}

/// bbs_3mf.cpp:1246-1259 `struct CutObjectInfo`
#[derive(Debug, Clone)]
struct CutObjectInfo {
    id: crate::object_id::CutObjectBase,
    connectors: Vec<CutConnectorInfo>,
}

/// bbs_3mf.cpp:1504-1510 `struct PendingVolumeAssemble`
#[derive(Debug, Clone)]
struct PendingVolumeAssemble {
    model_object_index: i32,
    volume_id: i32,
    transform: Transform3d,
}

/// `m_curr_plater`: a plate being read from model_settings.config (`new PlateData()`), or one
/// of `m_plater_data` selected by the slice info's plate index.
enum CurrPlater {
    New(Box<PlateData>),
    InMap(i32),
}

/// What `_extract_from_archive` reads of the importer: the errors and the restore paths.
#[derive(Clone)]
struct ExtractContext {
    base: std::sync::Arc<Bbs3mfBase>,
    m_backup_path: String,
    m_origin_file: String,
}

/// Which handlers an XML parser of the importer calls (`XML_SetElementHandler`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum XmlHandlers {
    /// `_handle_start_model_xml_element` / `_handle_end_model_xml_element` (the model files are
    /// parsed by `_extract_model_from_archive`, which installs these handlers itself)
    #[allow(dead_code)]
    Model,
    /// `_handle_start_config_xml_element` / `_handle_end_config_xml_element`
    Config,
    /// `_handle_start_relationships_element` / `_handle_end_relationships_element`
    Relationships,
}

/// bbs_3mf.cpp:1278-1418 `struct ObjectImporter`: reads one object file of the Production
/// Extension.
struct ObjectImporter {
    object_list: BTreeMap<Id, CurrentObject>,
    current_object: Option<Box<CurrentObject>>,
    object_path: String,
    zip_path: String,
    /// `top_importer`: what the object importer reads of it
    top: ExtractContext,
    top_load_restore: bool,
    /// `object_xml_parser != nullptr`
    object_xml_parser: bool,
    obj_parse_error: bool,
    obj_parse_error_message: String,
    //local parsed datas
    obj_curr_metadata_name: String,
    obj_curr_characters: String,
    object_unit_factor: f32,
    object_current_color_group: i32,
    object_group_id_to_color: BTreeMap<i32, Vec<String>>,
    object_material_colors: MaterialColorResources,
    is_bbl_3mf: bool,
}

impl ObjectImporter {
    /// bbs_3mf.cpp:1298-1303
    fn new(top: ExtractContext, top_load_restore: bool, file_path: &str, obj_path: &str) -> Self {
        ObjectImporter {
            object_list: BTreeMap::new(),
            current_object: None,
            object_path: obj_path.to_string(),
            zip_path: file_path.to_string(),
            top,
            top_load_restore,
            object_xml_parser: false,
            obj_parse_error: false,
            obj_parse_error_message: String::new(),
            obj_curr_metadata_name: String::new(),
            obj_curr_characters: String::new(),
            object_unit_factor: 1.0,
            object_current_color_group: -1,
            object_group_id_to_color: BTreeMap::new(),
            object_material_colors: MaterialColorResources::default(),
            is_bbl_3mf: false,
        }
    }

    /// bbs_3mf.cpp:1310-1316
    fn _destroy_object_xml_parser(&mut self) {
        if self.object_xml_parser {
            self.object_xml_parser = false;
        }
    }

    /// bbs_3mf.cpp:1318-1326
    fn _stop_object_xml_parser(&mut self, msg: &str) {
        self.obj_parse_error = true;
        self.obj_parse_error_message = msg.to_string();
    }

    /// bbs_3mf.cpp:1328
    fn object_parse_error(&self) -> bool {
        self.obj_parse_error
    }

    /// bbs_3mf.cpp:1329-1335
    fn object_parse_error_message(&self, expat_message: &str) -> String {
        if self.obj_parse_error {
            // The error was signalled by the user code, not the expat parser.
            if self.obj_parse_error_message.is_empty() {
                "Invalid 3MF format".to_string()
            } else {
                self.obj_parse_error_message.clone()
            }
        } else {
            // The error was signalled by the expat parser.
            expat_message.to_string()
        }
    }

    /// bbs_3mf.cpp:1339-1369
    fn extract_object_model(&mut self) -> bool {
        let mut archive = match open_zip_reader(&self.zip_path) {
            Some(a) => a,
            None => {
                self.top.base.add_error(format!("Unable to open the zipfile {}", self.zip_path));
                return false;
            }
        };

        let top = self.top.clone();
        let object_path = self.object_path.clone();
        let restore = self.top_load_restore;
        if !_BBS_3MF_Importer::_extract_from_archive(&top, &mut archive, &object_path, &mut |archive, stat| self._extract_object_from_archive(archive, stat), restore) {
            let error_msg = format!("Archive does not contain a valid model for {}", self.object_path);
            self.top.base.add_error(error_msg);
            close_zip_reader(archive);
            return false;
        }

        close_zip_reader(archive);

        if self.obj_parse_error {
            //already add_error inside
            log::error!("Found error while extrace object {}\n", self.object_path);
            return false;
        }
        true
    }

    /// bbs_3mf.cpp:6070-6075
    fn _handle_object_start_model(&mut self, attributes: &[(String, String)]) -> bool {
        self.object_unit_factor = bbs_get_unit_factor(&bbs_get_attribute_value_string(attributes, UNIT_ATTR));
        true
    }

    /// bbs_3mf.cpp:6077-6081
    fn _handle_object_end_model(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6083-6087
    fn _handle_object_start_resources(&mut self, _attributes: &[(String, String)]) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6089-6093
    fn _handle_object_end_resources(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6095-6124
    fn _handle_object_start_object(&mut self, attributes: &[(String, String)]) -> bool {
        // reset current object data
        self.current_object = None;

        let object_type = bbs_get_attribute_value_string(attributes, TYPE_ATTR);

        if bbs_is_valid_object_type(&object_type) {
            if self.current_object.is_none() {
                self.current_object = Some(Box::new(CurrentObject::new()));
            }
            let current_object = self.current_object.as_mut().unwrap();

            current_object.id = bbs_get_attribute_value_int(attributes, ID_ATTR);
            current_object.name = bbs_get_attribute_value_string(attributes, NAME_ATTR);

            current_object.uuid = bbs_get_attribute_value_string(attributes, PUUID_ATTR);
            if current_object.uuid.is_empty() {
                current_object.uuid = bbs_get_attribute_value_string(attributes, PUUID_LOWER_ATTR);
            }
            if bbs_has_attribute_value_int(attributes, PID_ATTR) {
                current_object.pid = bbs_get_attribute_value_int(attributes, PID_ATTR);
            }
            if bbs_has_attribute_value_int(attributes, PINDEX_ATTR) {
                current_object.pindex = bbs_get_attribute_value_int(attributes, PINDEX_ATTR);
            }
        }

        true
    }

    /// bbs_3mf.cpp:6126-6187
    fn _handle_object_end_object(&mut self) -> bool {
        let mut current_object = match self.current_object.take() {
            Some(o) if o.id != -1 => o,
            _ => {
                self.top.base.add_error(format!("Found invalid object for {}", self.object_path));
                return false;
            }
        };
        if self.is_bbl_3mf && current_object.uuid.ends_with(OBJECT_UUID_SUFFIX) && self.top_load_restore {
            let mut _need_replace = false;
            let mut backup_id = 0;
            if let Some(v) = istream_hex_int(&current_object.uuid) {
                backup_id = v;
                _need_replace = current_object.id != backup_id;
                current_object.id = backup_id;
            }
            //if (need_replace)
            {
                for index in 0..current_object.components.len() {
                    let temp_id = ((index as i32 + 1) << 16) | backup_id;
                    let component_id = current_object.components[index].object_id.clone();
                    let new_path = component_id.0.clone();
                    let new_id: Id = (new_path, temp_id);
                    if let Some(old) = self.object_list.remove(&component_id) {
                        let mut new_object = CurrentObject::new();
                        new_object.geometry = old.geometry;
                        new_object.id = temp_id;
                        new_object.model_object_idx = old.model_object_idx;
                        new_object.name = old.name;
                        new_object.uuid = old.uuid;

                        self.object_list.insert(new_id, new_object);
                    } else {
                        self.top.base.add_error(format!("can not find object for component, id={}", component_id.1));
                        return false;
                    }

                    current_object.components[index].object_id.1 = temp_id;
                }
            }
        }
        let id: Id = (self.object_path.clone(), current_object.id);
        if !self.object_list.contains_key(&id) {
            self.object_list.insert(id, *current_object);
        } else {
            self.top.base.add_error(format!("Found object with duplicate id for {}", self.object_path));
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:6189-6193
    fn _handle_object_start_color_group(&mut self, attributes: &[(String, String)]) -> bool {
        self.object_current_color_group = bbs_get_attribute_value_int(attributes, ID_ATTR);
        true
    }

    /// bbs_3mf.cpp:6195-6199
    fn _handle_object_end_color_group(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6201-6207
    fn _handle_object_start_color(&mut self, attributes: &[(String, String)]) -> bool {
        let color = bbs_get_attribute_value_string(attributes, COLOR_ATTR);
        let colors = self.object_group_id_to_color.entry(self.object_current_color_group).or_default();
        colors.push(color);
        true
    }

    /// bbs_3mf.cpp:6209-6213
    fn _handle_object_end_color(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6215-6221
    fn _handle_object_start_mesh(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current geometry
        if let Some(o) = self.current_object.as_mut() {
            o.geometry.reset();
        }
        true
    }

    /// bbs_3mf.cpp:6223-6227
    fn _handle_object_end_mesh(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6229-6235
    fn _handle_object_start_vertices(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current vertices
        if let Some(o) = self.current_object.as_mut() {
            o.geometry.vertices.clear();
        }
        true
    }

    /// bbs_3mf.cpp:6237-6241
    fn _handle_object_end_vertices(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6243-6253
    fn _handle_object_start_vertex(&mut self, attributes: &[(String, String)]) -> bool {
        // appends the vertex coordinates
        // missing values are set equal to ZERO
        let f = self.object_unit_factor;
        if let Some(o) = self.current_object.as_mut() {
            o.geometry.vertices.push(Vec3f::new(
                f * bbs_get_attribute_value_float(attributes, X_ATTR),
                f * bbs_get_attribute_value_float(attributes, Y_ATTR),
                f * bbs_get_attribute_value_float(attributes, Z_ATTR),
            ));
        }
        true
    }

    /// bbs_3mf.cpp:6255-6259
    fn _handle_object_end_vertex(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6261-6269
    fn _handle_object_start_triangles(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current triangles
        if let Some(o) = self.current_object.as_mut() {
            o.geometry.triangles.clear();
            o.geometry.triangle_colors.clear();
        }
        true
    }

    /// bbs_3mf.cpp:6271-6275
    fn _handle_object_end_triangles(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6277-6331
    fn _handle_object_start_triangle(&mut self, attributes: &[(String, String)]) -> bool {
        // we are ignoring the following attributes:
        // p1
        // p2
        // p3
        // pid
        // see specifications

        // appends the triangle's vertices indices
        // missing values are set equal to ZERO
        let is_bbl_3mf = self.is_bbl_3mf;
        if let Some(current_object) = self.current_object.as_mut() {
            current_object.geometry.triangles.push(crate::triangle_mesh::Vec3i::new(
                bbs_get_attribute_value_int(attributes, V1_ATTR),
                bbs_get_attribute_value_int(attributes, V2_ATTR),
                bbs_get_attribute_value_int(attributes, V3_ATTR),
            ));

            current_object.geometry.custom_supports.push(bbs_get_attribute_value_string(attributes, CUSTOM_SUPPORTS_ATTR));
            current_object.geometry.custom_fuzzy_skin.push(bbs_get_attribute_value_string(attributes, CUSTOM_FUZZY_SKIN_ATTR));
            current_object.geometry.custom_seam.push(bbs_get_attribute_value_string(attributes, CUSTOM_SEAM_ATTR));
            current_object.geometry.mmu_segmentation.push(bbs_get_attribute_value_string(attributes, MMU_SEGMENTATION_ATTR));
            // BBS
            current_object.geometry.face_properties.push(bbs_get_attribute_value_string(attributes, FACE_PROPERTY_ATTR));
            if !is_bbl_3mf {
                let mut color_info = crate::format::obj::TriangleColor::new();
                if bbs_has_attribute_value_int(attributes, PID_ATTR) {
                    color_info.pid = bbs_get_attribute_value_int(attributes, PID_ATTR);
                } else {
                    color_info.pid = current_object.pid;
                }
                if color_info.pid >= 0 {
                    let color_attrs = [P1_ATTR, P2_ATTR, P3_ATTR];
                    for idx in 0..3 {
                        if bbs_has_attribute_value_int(attributes, color_attrs[idx]) {
                            color_info.indices[idx] = bbs_get_attribute_value_int(attributes, color_attrs[idx]);
                        } else if current_object.pindex >= 0 {
                            if idx == 0 {
                                color_info.indices[idx] = current_object.pindex;
                            } else {
                                color_info.indices[idx] = color_info.indices[0];
                            }
                        }
                    }
                    modify_import_color(&mut color_info.indices);
                    current_object.geometry.triangle_colors.push(color_info);
                }
            }
        }
        true
    }

    /// bbs_3mf.cpp:6333-6337
    fn _handle_object_end_triangle(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6339-6345
    fn _handle_object_start_components(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current components
        if let Some(o) = self.current_object.as_mut() {
            o.components.clear();
        }
        true
    }

    /// bbs_3mf.cpp:6347-6351
    fn _handle_object_end_components(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6353-6374
    fn _handle_object_start_component(&mut self, attributes: &[(String, String)]) -> bool {
        let object_id = bbs_get_attribute_value_int(attributes, OBJECTID_ATTR);
        let transform = bbs_get_transform_from_3mf_specs_string(&bbs_get_attribute_value_string(attributes, TRANSFORM_ATTR));

        if let Some(o) = self.current_object.as_mut() {
            let id: Id = (self.object_path.clone(), object_id);
            o.components.push(Component::new(id, transform));
        }

        true
    }

    /// bbs_3mf.cpp:6376-6380
    fn _handle_object_end_component(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:6382-6392
    fn _handle_object_start_metadata(&mut self, attributes: &[(String, String)]) -> bool {
        self.obj_curr_metadata_name.clear();

        let name = bbs_get_attribute_value_string(attributes, NAME_ATTR);
        if !name.is_empty() {
            self.obj_curr_metadata_name = name;
        }

        true
    }

    /// bbs_3mf.cpp:6394-6400
    fn _handle_object_end_metadata(&mut self) -> bool {
        if self.obj_curr_metadata_name == BBS_3MF_VERSION || self.obj_curr_metadata_name == BBS_3MF_VERSION1 {
            self.is_bbl_3mf = true;
        }
        true
    }

    /// bbs_3mf.cpp:6401-6440
    fn _handle_object_start_model_xml_element(&mut self, name: &str, attributes: &[(String, String)]) {
        if !self.object_xml_parser {
            return;
        }

        let mut res = true;

        self.object_material_colors.start_element(name, attributes, &mut self.object_group_id_to_color);

        if MODEL_TAG == name {
            res = self._handle_object_start_model(attributes);
        } else if RESOURCES_TAG == name {
            res = self._handle_object_start_resources(attributes);
        } else if OBJECT_TAG == name {
            res = self._handle_object_start_object(attributes);
        } else if COLOR_GROUP_TAG == name {
            res = self._handle_object_start_color_group(attributes);
        } else if COLOR_TAG == name {
            res = self._handle_object_start_color(attributes);
        } else if MESH_TAG == name {
            res = self._handle_object_start_mesh(attributes);
        } else if VERTICES_TAG == name {
            res = self._handle_object_start_vertices(attributes);
        } else if VERTEX_TAG == name {
            res = self._handle_object_start_vertex(attributes);
        } else if TRIANGLES_TAG == name {
            res = self._handle_object_start_triangles(attributes);
        } else if TRIANGLE_TAG == name {
            res = self._handle_object_start_triangle(attributes);
        } else if COMPONENTS_TAG == name {
            res = self._handle_object_start_components(attributes);
        } else if COMPONENT_TAG == name {
            res = self._handle_object_start_component(attributes);
        } else if METADATA_TAG == name {
            res = self._handle_object_start_metadata(attributes);
        }

        if !res {
            self._stop_object_xml_parser("");
        }
    }

    /// bbs_3mf.cpp:6442-6480
    fn _handle_object_end_model_xml_element(&mut self, name: &str) {
        if !self.object_xml_parser {
            return;
        }

        let mut res = true;

        self.object_material_colors.end_element(name);

        if MODEL_TAG == name {
            res = self._handle_object_end_model();
        } else if RESOURCES_TAG == name {
            res = self._handle_object_end_resources();
        } else if OBJECT_TAG == name {
            res = self._handle_object_end_object();
        } else if COLOR_GROUP_TAG == name {
            res = self._handle_object_end_color_group();
        } else if COLOR_TAG == name {
            res = self._handle_object_end_color();
        } else if MESH_TAG == name {
            res = self._handle_object_end_mesh();
        } else if VERTICES_TAG == name {
            res = self._handle_object_end_vertices();
        } else if VERTEX_TAG == name {
            res = self._handle_object_end_vertex();
        } else if TRIANGLES_TAG == name {
            res = self._handle_object_end_triangles();
        } else if TRIANGLE_TAG == name {
            res = self._handle_object_end_triangle();
        } else if COMPONENTS_TAG == name {
            res = self._handle_object_end_components();
        } else if COMPONENT_TAG == name {
            res = self._handle_object_end_component();
        } else if METADATA_TAG == name {
            res = self._handle_object_end_metadata();
        }

        if !res {
            self._stop_object_xml_parser("");
        }
    }

    /// bbs_3mf.cpp:6482-6485
    fn _handle_object_xml_characters(&mut self, s: &str) {
        self.obj_curr_characters.push_str(s);
    }

    /// bbs_3mf.cpp:6508-6573
    fn _extract_object_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat) -> bool {
        if stat.m_uncomp_size == 0 {
            self.top.base.add_error(format!("Found invalid size for {}", self.object_path));
            return false;
        }

        self.object_xml_parser = true;

        let data = match mz_zip_reader_extract_to_mem(archive, stat.m_file_index) {
            Some(d) => d,
            None => {
                self.top.base.add_error(format!("Error while extracting model data from zip archive for {}", self.object_path));
                return false;
            }
        };
        let result = expat_parse(&data, &mut |ev| {
            match ev {
                SaxEvent::Start(name, atts) => ObjectImporterXmlCallbacks::_handle_object_start_model_xml_element(Some(&mut *self), name, atts),
                SaxEvent::End(name) => ObjectImporterXmlCallbacks::_handle_object_end_model_xml_element(Some(&mut *self), name),
                SaxEvent::Characters(s) => ObjectImporterXmlCallbacks::_handle_object_xml_characters(Some(&mut *self), s),
            }
            !self.object_parse_error()
        });
        if let Err(e) = result {
            let error_buf = format!(
                "Error ({}) while parsing '{}' at line {}",
                self.object_parse_error_message(&e.message),
                stat.m_filename,
                e.line
            );
            let error_message = format!("{} for {}", error_buf, self.object_path);
            self.top.base.add_error(error_message);
            return false;
        }

        true
    }
}

/// bbs_3mf.cpp:1075-1709 `class _BBS_3MF_Importer : public _BBS_3MF_Base`
#[allow(non_camel_case_types)]
pub struct _BBS_3MF_Importer {
    base: std::sync::Arc<Bbs3mfBase>,
    // Version of the 3mf file
    m_version: u32,
    m_check_version: bool,
    m_load_model: bool,
    m_load_aux: bool,
    m_load_config: bool,
    // backup & restore
    m_load_restore: bool,
    m_backup_path: String,
    m_origin_file: String,
    // Semantic version of Bambu Studio, that generated this 3MF.
    m_bambuslicer_generator_version: Option<crate::semver::Semver>,
    m_fdm_supports_painting_version: u32,
    m_seam_painting_version: u32,
    m_mm_painting_version: u32,
    m_model_id: String,
    m_contry_code: String,
    m_designer: String,
    m_designer_id: String,
    m_design_id: String,
    m_designer_user_id: String,
    #[allow(dead_code)]
    m_designer_cover: String,
    model_info: crate::model::ModelInfo,
    #[allow(dead_code)]
    project_info: BBLProject,
    m_profile_title: String,
    m_profile_cover: String,
    m_profile_description: String,
    m_profile_user_id: String,
    m_profile_user_name: String,

    /// `m_xml_parser != nullptr`
    m_xml_parser: bool,
    // Error code returned by the application side of the parser. In that case the expat may not reliably deliver the error state
    // after returning from XML_Parse() function, thus we keep the error state here.
    m_parse_error: bool,
    m_parse_error_message: String,
    m_unit_factor: f32,
    m_has_unit: bool,
    m_curr_object: Option<Box<CurrentObject>>,
    m_current_objects: BTreeMap<Id, CurrentObject>,
    m_index_paths: BTreeMap<i32, String>,
    m_objects: BTreeMap<Id, i32>,
    m_instances: Vec<ImporterInstance>,
    m_curr_config: CurrentConfig,
    m_objects_metadata: BTreeMap<i32, ObjectMetadata>,
    m_cut_object_infos: BTreeMap<i32, CutObjectInfo>,
    m_layer_heights_profiles: BTreeMap<i32, Vec<f64>>,
    m_layer_config_ranges: BTreeMap<i32, crate::model::TLayerConfigRanges>,
    m_brim_ear_points: BTreeMap<i32, Vec<crate::brim_ears_point::BrimPoint>>,
    m_path_to_emboss_shape_files: BTreeMap<String, String>,
    m_curr_metadata_name: String,
    m_curr_characters: String,
    m_name: String,
    m_sub_model_path: String,

    m_start_part_path: String,
    m_thumbnail_path: String,
    m_thumbnail_middle: String,
    m_thumbnail_small: String,
    m_sub_model_paths: Vec<String>,

    /// `std::map<int, ModelVolume*> m_shared_meshes`: (object index, volume index)
    m_shared_meshes: BTreeMap<i32, (usize, usize)>,

    //BBS: plater related structures
    m_is_bbl_3mf: bool,
    m_parsing_slice_info: bool,
    m_plater_data: BTreeMap<i32, PlateData>,
    m_curr_plater: Option<CurrPlater>,
    m_curr_instance: CurrentInstance,

    m_current_color_group: i32,
    m_group_id_to_color: BTreeMap<i32, Vec<String>>,
    m_material_colors: MaterialColorResources,

    // Store the color data for each volume(volume object id->VolumeColorInfo).
    // Note: This mapping is populated in _generate_volumes_new.
    m_volume_color_data: HashMap<i32, VolumeColorInfo>,

    m_pending_volume_assemble: Vec<PendingVolumeAssemble>,

    /// An exception thrown inside an XML handler (it escapes `XML_Parse` in the C++).
    m_pending_exception: Option<Bbs3mfException>,
}

impl Default for _BBS_3MF_Importer {
    fn default() -> Self {
        Self::new()
    }
}

impl _BBS_3MF_Importer {
    /// bbs_3mf.cpp:1711-1722
    pub fn new() -> Self {
        _BBS_3MF_Importer {
            base: std::sync::Arc::new(Bbs3mfBase::new()),
            m_version: 0,
            m_check_version: false,
            m_load_model: false,
            m_load_aux: false,
            m_load_config: false,
            m_load_restore: false,
            m_backup_path: String::new(),
            m_origin_file: String::new(),
            m_bambuslicer_generator_version: None,
            m_fdm_supports_painting_version: 0,
            m_seam_painting_version: 0,
            m_mm_painting_version: 0,
            m_model_id: String::new(),
            m_contry_code: String::new(),
            m_designer: String::new(),
            m_designer_id: String::new(),
            m_design_id: String::new(),
            m_designer_user_id: String::new(),
            m_designer_cover: String::new(),
            model_info: crate::model::ModelInfo::default(),
            project_info: BBLProject::default(),
            m_profile_title: String::new(),
            m_profile_cover: String::new(),
            m_profile_description: String::new(),
            m_profile_user_id: String::new(),
            m_profile_user_name: String::new(),
            m_xml_parser: false,
            m_parse_error: false,
            m_parse_error_message: String::new(),
            m_unit_factor: 1.0,
            m_has_unit: false,
            m_curr_object: None,
            m_current_objects: BTreeMap::new(),
            m_index_paths: BTreeMap::new(),
            m_objects: BTreeMap::new(),
            m_instances: Vec::new(),
            m_curr_config: CurrentConfig { object_id: -1, volume_id: -1 },
            m_objects_metadata: BTreeMap::new(),
            m_cut_object_infos: BTreeMap::new(),
            m_layer_heights_profiles: BTreeMap::new(),
            m_layer_config_ranges: BTreeMap::new(),
            m_brim_ear_points: BTreeMap::new(),
            m_path_to_emboss_shape_files: BTreeMap::new(),
            m_curr_metadata_name: String::new(),
            m_curr_characters: String::new(),
            m_name: String::new(),
            m_sub_model_path: String::new(),
            m_start_part_path: String::new(),
            m_thumbnail_path: String::new(),
            m_thumbnail_middle: String::new(),
            m_thumbnail_small: String::new(),
            m_sub_model_paths: Vec::new(),
            m_shared_meshes: BTreeMap::new(),
            m_is_bbl_3mf: false,
            m_parsing_slice_info: false,
            m_plater_data: BTreeMap::new(),
            m_curr_plater: None,
            m_curr_instance: CurrentInstance { object_id: -1, instance_id: -1, identify_id: 0 },
            m_current_color_group: -1,
            m_group_id_to_color: BTreeMap::new(),
            m_material_colors: MaterialColorResources::default(),
            m_volume_color_data: HashMap::new(),
            m_pending_volume_assemble: Vec::new(),
            m_pending_exception: None,
        }
    }

    fn add_error(&self, error: impl Into<String>) {
        self.base.add_error(error.into());
    }

    /// bbs_3mf.cpp:1068-1072 `log_errors()`
    pub fn log_errors(&self) {
        self.base.log_errors();
    }

    /// The errors recorded so far.
    pub fn errors(&self) -> Vec<String> {
        self.base.errors()
    }

    fn extract_context(&self) -> ExtractContext {
        ExtractContext { base: self.base.clone(), m_backup_path: self.m_backup_path.clone(), m_origin_file: self.m_origin_file.clone() }
    }

    fn curr_plater_mut(&mut self) -> Option<&mut PlateData> {
        match self.m_curr_plater.as_mut()? {
            CurrPlater::New(p) => Some(p.as_mut()),
            CurrPlater::InMap(k) => {
                let k = *k;
                self.m_plater_data.get_mut(&k)
            }
        }
    }

    /// bbs_3mf.cpp:1523
    pub fn version(&self) -> u32 {
        self.m_version
    }

    /// bbs_3mf.cpp:1526
    pub fn get_color_group_map(&self) -> &BTreeMap<i32, Vec<String>> {
        &self.m_group_id_to_color
    }

    /// bbs_3mf.cpp:1529 — True when the file declares textures, which the color import ignores.
    pub fn has_texture_resources(&self) -> bool {
        self.m_material_colors.has_textures()
    }

    /// bbs_3mf.cpp:1532 — True when the root model declares its unit; without it the reader
    /// falls back to millimeters.
    pub fn has_unit(&self) -> bool {
        self.m_has_unit
    }

    /// bbs_3mf.cpp:1535
    pub fn get_volume_color_data(&self) -> &HashMap<i32, VolumeColorInfo> {
        &self.m_volume_color_data
    }

    /// bbs_3mf.cpp:1752-1821
    #[allow(clippy::too_many_arguments)]
    pub fn load_model_from_file(
        &mut self,
        filename: &str,
        model: &mut Model,
        plate_data_list: &mut PlateDataPtrs,
        project_presets: &mut Vec<Preset>,
        config: &mut DynamicPrintConfig,
        config_substitutions: &mut crate::config::ConfigSubstitutionContext,
        strategy: LoadStrategy,
        is_bbl_3mf: Option<&mut bool>,
        file_version: &mut crate::semver::Semver,
        pro_fn: Option<&Import3mfProgressFn>,
        project: Option<&mut BBLProject>,
        plate_id: i32,
    ) -> Result<bool, Bbs3mfException> {
        self.m_version = 0;
        self.m_fdm_supports_painting_version = 0;
        self.m_seam_painting_version = 0;
        self.m_mm_painting_version = 0;
        self.m_check_version = strategy.contains(LoadStrategy::CheckVersion);
        //BBS: auxiliary data
        self.m_load_model = strategy.contains(LoadStrategy::LoadModel);
        self.m_load_aux = strategy.contains(LoadStrategy::LoadAuxiliary);
        self.m_load_restore = strategy.contains(LoadStrategy::Restore);
        self.m_load_config = strategy.contains(LoadStrategy::LoadConfig);
        self.m_unit_factor = 1.0;
        self.m_has_unit = false;
        self.m_curr_object = None;
        self.m_current_objects.clear();
        self.m_index_paths.clear();
        self.m_objects.clear();
        self.m_instances.clear();
        self.m_curr_config.object_id = -1;
        self.m_curr_config.volume_id = -1;
        self.m_objects_metadata.clear();
        self.m_layer_heights_profiles.clear();
        self.m_layer_config_ranges.clear();
        self.m_brim_ear_points.clear();
        self.m_curr_metadata_name.clear();
        self.m_curr_characters.clear();
        //BBS: plater data init
        self.m_plater_data.clear();
        self.m_curr_instance.object_id = -1;
        self.m_curr_instance.instance_id = -1;
        self.m_curr_instance.identify_id = 0;
        self.base.clear_errors();

        // restore
        if self.m_load_restore {
            self.m_backup_path = filename[..filename.len().saturating_sub(5)].to_string();
            model.set_backup_path(&self.m_backup_path);
            let origin = format!("{}/origin.txt", model.get_backup_path());
            if std::path::Path::new(&origin).exists() {
                let mut s = String::new();
                match crate::utils::load_string_file(std::path::Path::new(&origin), &mut s) {
                    Ok(()) => self.m_origin_file = s,
                    Err(e) => log::warn!("{}", e),
                }
            }
            let _ = crate::utils::save_string_file(
                std::path::Path::new(&format!("{}/lock.txt", model.get_backup_path())),
                &crate::utils::get_current_pid().to_string(),
            );
        } else {
            self.m_backup_path = model.get_backup_path();
        }
        let result = self._load_model_from_file(filename, model, plate_data_list, project_presets, config, config_substitutions, pro_fn, project, plate_id)?;
        if let Some(b) = is_bbl_3mf {
            *b = self.m_is_bbl_3mf;
        }
        if let Some(v) = &self.m_bambuslicer_generator_version {
            *file_version = v.clone();
        }
        // save for restore
        if result && self.m_load_aux && !self.m_load_restore {
            let _ = crate::utils::save_string_file(std::path::Path::new(&format!("{}/origin.txt", model.get_backup_path())), filename);
        }
        if self.m_load_restore && !result {
            // not clear failed backup data for later analyze
            model.set_backup_path("detach");
        }
        Ok(result)
    }

    /// bbs_3mf.cpp:1823-1860
    pub fn get_thumbnail(&mut self, filename: &str, data: &mut Vec<u8>) -> bool {
        let mut archive = match open_zip_reader(filename) {
            Some(a) => a,
            None => {
                self.add_error(format!("Unable to open the file {}", filename));
                return false;
            }
        };

        // BBS: load relationships
        if !self._extract_xml_from_archive_path(&mut archive, RELATIONSHIPS_FILE, XmlHandlers::Relationships, None, None) {
            return false;
        }
        if self.m_thumbnail_middle.is_empty() {
            self.m_thumbnail_middle = self.m_thumbnail_path.clone();
        }
        let ctx = self.extract_context();
        if !self.m_thumbnail_middle.is_empty() {
            let path = self.m_thumbnail_middle.clone();
            return Self::_extract_from_archive(&ctx, &mut archive, &path, &mut |archive, stat| {
                match mz_zip_reader_extract_to_mem(archive, stat.m_file_index) {
                    Some(d) => {
                        *data = d;
                        true
                    }
                    None => false,
                }
            }, false);
        }
        Self::_extract_from_archive(&ctx, &mut archive, THUMBNAIL_FILE, &mut |archive, stat| {
            match mz_zip_reader_extract_to_mem(archive, stat.m_file_index) {
                Some(d) => {
                    *data = d;
                    true
                }
                None => false,
            }
        }, false)
    }

    /// bbs_3mf.cpp:1872-2049
    pub fn load_gcode_3mf_from_stream(
        &mut self,
        data: Box<dyn ReadSeek>,
        model: &mut Model,
        plate_data_list: &mut PlateDataPtrs,
        config: &mut DynamicPrintConfig,
        _file_version: &mut crate::semver::Semver,
    ) -> Result<bool, Bbs3mfException> {
        // archive.m_pRead = mz_zip_read_istream; archive.m_pIO_opaque = &data;
        let stream = match MzZipIstream::new(data) {
            Some(s) => s,
            None => return Ok(false),
        };
        // mz_zip_reader_init(&archive, size, 0)
        let mut archive = match MzZipArchive::from_reader(Box::new(stream)) {
            Some(a) => a,
            None => return Ok(false),
        };

        // BBS: load relationships
        if !self._extract_xml_from_archive_path(&mut archive, RELATIONSHIPS_FILE, XmlHandlers::Relationships, None, None) {
            return Ok(false);
        }
        if self.m_start_part_path.is_empty() {
            return Ok(false);
        }

        //extract model files
        let ctx = self.extract_context();
        let start = self.m_start_part_path.clone();
        if !Self::_extract_from_archive(&ctx, &mut archive, &start, &mut |archive, stat| self._extract_model_from_archive(model, archive, stat), false) {
            self.add_error("Archive does not contain a valid model");
            return Ok(false);
        }

        if !self.m_designer.is_empty() {
            model.design_info = Some(std::sync::Arc::new(crate::model::ModelDesignInfo {
                designer_id: self.m_designer_id.clone(),
                designer_user_id: self.m_designer_user_id.clone(),
                designer: self.m_designer.clone(),
            }));
        }

        model.profile_info = Some(std::sync::Arc::new(crate::model::ModelProfileInfo {
            profile_tile: self.m_profile_title.clone(),
            profile_cover: self.m_profile_cover.clone(),
            profile_description: self.m_profile_description.clone(),
            profile_user_id: self.m_profile_user_id.clone(),
            profile_user_name: self.m_profile_user_name.clone(),
        }));
        model.design_id = self.m_design_id.clone();

        let mut info = crate::model::ModelInfo::default();
        info.load(&self.model_info);

        if self.m_thumbnail_middle.is_empty() {
            self.m_thumbnail_middle = self.m_thumbnail_path.clone();
        }
        if self.m_thumbnail_small.is_empty() {
            self.m_thumbnail_small = self.m_thumbnail_path.clone();
        }
        if self.m_thumbnail_small.starts_with('/') {
            self.m_thumbnail_small.remove(0);
        }
        if self.m_thumbnail_middle.starts_with('/') {
            self.m_thumbnail_middle.remove(0);
        }
        info.metadata_items.entry("Thumbnail".to_string()).or_insert_with(|| self.m_thumbnail_middle.clone());
        info.metadata_items.entry("Poster".to_string()).or_insert_with(|| self.m_thumbnail_middle.clone());
        model.model_info = Some(std::sync::Arc::new(info));

        // we then loop again the entries to read other files stored in the archive
        let num_entries = mz_zip_reader_get_num_files(&archive);
        let mut has_filament_sequence = false;
        let mut filament_sequence_stat = MzZipArchiveFileStat::default();
        for i in 0..num_entries {
            if let Some(stat) = mz_zip_reader_file_stat(&archive, i) {
                let name = stat.m_filename.replace('\\', "/");

                log::info!("extract {}th file {}, total={}\n", i + 1, name, num_entries);

                if iequals(&name, BBS_PROJECT_CONFIG_FILE) {
                    // extract slic3r print config file
                    let mut config_substitutions =
                        crate::config::ConfigSubstitutionContext::new(crate::config::ForwardCompatibilitySubstitutionRule::Disable);
                    self._extract_project_config_from_archive(&mut archive, &stat, config, &mut config_substitutions, model);
                } else if iequals(&name, BBS_MODEL_CONFIG_FILE) {
                    // extract slic3r model config file
                    if !self._extract_xml_from_archive(&mut archive, &stat, XmlHandlers::Config, Some(model)) {
                        self.add_error("Archive does not contain a valid model config");
                        return Ok(false);
                    }
                } else if self._is_svg_shape_file(&name) {
                    self._extract_embossed_svg_shape_file(&name, &mut archive, &stat, model);
                } else if iequals(&name, SLICE_INFO_CONFIG_FILE) {
                    self.m_parsing_slice_info = true;
                    //extract slice info from archive
                    self._extract_xml_from_archive(&mut archive, &stat, XmlHandlers::Config, Some(model));
                    self.m_parsing_slice_info = false;
                } else if iequals(&name, FILAMENT_SEQUENCE_FILE) {
                    has_filament_sequence = true;
                    filament_sequence_stat = stat.clone();
                }
            }
        }
        if let Some(e) = self.m_pending_exception.take() {
            return Err(e);
        }

        // The sequences are attached to plates created by BBS_MODEL_CONFIG_FILE, so they can only
        // be applied once the loop above is done. Entry order is not guaranteed here: this stream
        // may come from an archive repacked by the printer when printing from storage.
        if has_filament_sequence {
            self._extract_filament_sequence_from_archive(&mut archive, &filament_sequence_stat);
        }

        //BBS: load the plate info into plate_data_list
        plate_data_list.clear();
        plate_data_list.reserve(self.m_plater_data.len());
        for _ in 0..self.m_plater_data.len() {
            plate_data_list.push(PlateData::new());
        }
        let n = self.m_plater_data.len() as i32;
        let keys: Vec<i32> = self.m_plater_data.keys().cloned().collect();
        for key in keys {
            if key <= 0 || key > n {
                self.add_error("invalid plate index");
                return Ok(false);
            }
            let src = self.m_plater_data.get(&key).unwrap().clone();
            let plate = &mut plate_data_list[(key - 1) as usize];
            plate.locked = src.locked;
            plate.plate_index = src.plate_index - 1;
            plate.obj_inst_map = src.obj_inst_map.clone();
            plate.gcode_file = src.gcode_file.clone();
            plate.gcode_prediction = src.gcode_prediction.clone();
            plate.gcode_weight = src.gcode_weight.clone();
            plate.toolpath_outside = src.toolpath_outside;
            plate.is_support_used = src.is_support_used;
            plate.is_label_object_enabled = src.is_label_object_enabled;
            plate.support_material_on_wipe_tower = src.support_material_on_wipe_tower;
            plate.skipped_objects = src.skipped_objects.clone();
            plate.slice_filaments_info = src.slice_filaments_info.clone();
            plate.slice_filaments_extra = src.slice_filaments_extra.clone();
            plate.nozzles_info = src.nozzles_info.clone();
            plate.printer_model_id = src.printer_model_id.clone();
            plate.nozzle_diameters = src.nozzle_diameters.clone();
            plate.nozzle_volume_types = src.nozzle_volume_types.clone();
            plate.warnings = src.warnings.clone();
            plate.thumbnail_file = src.thumbnail_file.clone();
            plate.filament_maps = src.filament_maps.clone();
            if plate.thumbnail_file.is_empty() {
                plate.thumbnail_file = plate.gcode_file.clone();
                plate.thumbnail_file = plate.thumbnail_file.replace(".gcode", ".png");
            }
            //plate->pattern_file = it->second->pattern_file;
            plate.no_light_thumbnail_file = src.no_light_thumbnail_file.clone();
            plate.top_file = src.top_file.clone();
            // bbs_3mf.cpp:2028-2029 assign the `bool` of `empty()` to a std::string: the string
            // holds the one character of that value (0 or 1)
            plate.pick_file = String::from(char::from(src.pick_file.is_empty() as u8));
            plate.pattern_bbox_file = String::from(char::from(src.pattern_bbox_file.is_empty() as u8));
            plate.config = src.config.clone();
            plate.filament_change_sequence = src.filament_change_sequence.clone();
            plate.nozzle_change_sequence = src.nozzle_change_sequence.clone();
            plate.optimal_assignment = src.optimal_assignment.clone();

            if !plate.thumbnail_file.is_empty() {
                let path = plate.thumbnail_file.clone();
                let ctx = self.extract_context();
                let pixels = &mut plate_data_list[(key - 1) as usize].plate_thumbnail.pixels;
                Self::_extract_from_archive(&ctx, &mut archive, &path, &mut |archive, stat| {
                    match mz_zip_reader_extract_to_mem(archive, stat.m_file_index) {
                        Some(d) => {
                            *pixels = d;
                            true
                        }
                        None => false,
                    }
                }, false);
            }

            let plate = &plate_data_list[(key - 1) as usize];
            log::info!(
                "plate {}, thumbnail_file={}, no_light_thumbnail_file={}",
                key,
                plate.thumbnail_file,
                plate.no_light_thumbnail_file
            );
            log::info!("top_thumbnail_file={}, pick_thumbnail_file={}", plate.top_file, plate.pick_file);
        }

        close_zip_reader(archive);

        Ok(true)
    }

    /// bbs_3mf.cpp:2051-2057
    fn _destroy_xml_parser(&mut self) {
        if self.m_xml_parser {
            self.m_xml_parser = false;
        }
    }

    /// bbs_3mf.cpp:2059-2067
    fn _stop_xml_parser(&mut self, msg: &str) {
        self.m_parse_error = true;
        self.m_parse_error_message = msg.to_string();
    }

    /// bbs_3mf.cpp:1541
    fn parse_error(&self) -> bool {
        self.m_parse_error
    }

    /// bbs_3mf.cpp:1542-1548
    fn parse_error_message(&self, expat_message: &str) -> String {
        if self.m_parse_error {
            // The error was signalled by the user code, not the expat parser.
            if self.m_parse_error_message.is_empty() {
                "Invalid 3MF format".to_string()
            } else {
                self.m_parse_error_message.clone()
            }
        } else {
            // The error was signalled by the expat parser.
            expat_message.to_string()
        }
    }

    /// bbs_3mf.cpp:2070-2831
    #[allow(clippy::too_many_arguments)]
    fn _load_model_from_file(
        &mut self,
        filename: &str,
        model: &mut Model,
        plate_data_list: &mut PlateDataPtrs,
        project_presets: &mut Vec<Preset>,
        config: &mut DynamicPrintConfig,
        config_substitutions: &mut crate::config::ConfigSubstitutionContext,
        pro_fn: Option<&Import3mfProgressFn>,
        project: Option<&mut BBLProject>,
        plate_id: i32,
    ) -> Result<bool, Bbs3mfException> {
        let mut cb_cancel = false;
        let progress = |stage: i32, current: i32, total: i32, cancel: &mut bool| {
            if let Some(f) = pro_fn {
                f(stage, current, total, cancel);
            }
        };
        //BBS progress point
        // prepare restore
        if self.m_load_restore {
            log::info!("import 3mf IMPORT_STAGE_RESTORE\n");
            if pro_fn.is_some() {
                progress(IMPORT_STAGE_RESTORE, 0, 1, &mut cb_cancel);
                if cb_cancel {
                    return Ok(false);
                }
            }
        }

        //BBS progress point
        log::info!("import 3mf IMPORT_STAGE_OPEN, m_load_restore={}\n", self.m_load_restore);
        if pro_fn.is_some() {
            progress(IMPORT_STAGE_OPEN, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        let mut archive = match open_zip_reader(filename) {
            Some(a) => a,
            None => {
                self.add_error(format!("Unable to open the file {}", filename));
                return Ok(false);
            }
        };

        let num_entries = mz_zip_reader_get_num_files(&archive);

        self.m_name = std::path::Path::new(filename).file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();

        //BBS progress point
        log::info!("import 3mf IMPORT_STAGE_READ_FILES\n");
        if pro_fn.is_some() {
            progress(IMPORT_STAGE_READ_FILES, 0, 3, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }
        // BBS: load relationships
        if !self._extract_xml_from_archive_path(&mut archive, RELATIONSHIPS_FILE, XmlHandlers::Relationships, None, None) {
            return Ok(false);
        }
        if self.m_start_part_path.is_empty() {
            return Ok(false);
        }
        // BBS: load sub models (Production Extension)
        let mut sub_rels = self.m_start_part_path.clone();
        let pos = sub_rels.rfind('/').map(|p| p + 1).unwrap_or(0);
        sub_rels.insert_str(pos, "_rels/");
        sub_rels.push_str(".rels");
        if sub_rels.starts_with('/') {
            sub_rels = sub_rels[1..].to_string();
        }

        //check whether sub relation file is exist or not
        let sub_index = mz_zip_reader_locate_file(&archive, &sub_rels);
        if sub_index == -1 {
            //no submodule files found, use only one 3dmodel.model
        } else {
            self._extract_xml_from_archive_path(&mut archive, &sub_rels, XmlHandlers::Relationships, None, None);

            // bbs_3mf.cpp:2176-2215 — FIDELITY-NOTE: the object files are read one after the
            // other (tbb::parallel_for in the C++); each importer is independent and the merge
            // below runs in the same order either way.
            let mut object_importers: Vec<ObjectImporter> = Vec::new();
            for path in self.m_sub_model_paths.clone() {
                object_importers.push(ObjectImporter::new(self.extract_context(), self.m_load_restore, filename, &path));
            }

            let mut object_load_result = true;
            for importer in object_importers.iter_mut() {
                let result = importer.extract_object_model();
                object_load_result &= result;
            }

            if !object_load_result {
                log::error!("loading sub-objects error\n");
                return Ok(false);
            }

            //merge these objects into one
            for obj_importer in object_importers {
                for (k, v) in obj_importer.object_list {
                    self.m_current_objects.entry(k).or_insert(v);
                }
                for (k, src) in obj_importer.object_group_id_to_color {
                    let dst = self.m_group_id_to_color.entry(k).or_default();
                    dst.extend(src);
                }
                self.m_material_colors.merge(&obj_importer.object_material_colors);
            }
            // BBS: load root model
            if pro_fn.is_some() {
                progress(IMPORT_STAGE_READ_FILES, 2, 3, &mut cb_cancel);
                if cb_cancel {
                    return Ok(false);
                }
            }
        }

        //extract model files
        let ctx = self.extract_context();
        let start = self.m_start_part_path.clone();
        if !Self::_extract_from_archive(&ctx, &mut archive, &start, &mut |archive, stat| self._extract_model_from_archive(model, archive, stat), false) {
            self.add_error("Archive does not contain a valid model");
            return Ok(false);
        }

        if !self.m_designer.is_empty() {
            model.design_info = Some(std::sync::Arc::new(crate::model::ModelDesignInfo {
                designer_id: self.m_designer_id.clone(),
                designer_user_id: self.m_designer_user_id.clone(),
                designer: self.m_designer.clone(),
            }));
        }

        model.profile_info = Some(std::sync::Arc::new(crate::model::ModelProfileInfo {
            profile_tile: self.m_profile_title.clone(),
            profile_cover: self.m_profile_cover.clone(),
            profile_description: self.m_profile_description.clone(),
            profile_user_id: self.m_profile_user_id.clone(),
            profile_user_name: self.m_profile_user_name.clone(),
        }));
        model.design_id = self.m_design_id.clone();

        let mut info = crate::model::ModelInfo::default();
        info.load(&self.model_info);
        if !self.m_thumbnail_small.is_empty() {
            info.metadata_items.entry("Thumbnail_Small".to_string()).or_insert_with(|| self.m_thumbnail_small.clone());
        }
        if !self.m_thumbnail_middle.is_empty() {
            info.metadata_items.entry("Thumbnail_Middle".to_string()).or_insert_with(|| self.m_thumbnail_middle.clone());
        }
        model.model_info = Some(std::sync::Arc::new(info));

        //got project id
        if let Some(project) = project {
            project.project_model_id = self.m_model_id.clone();
            project.project_country_code = self.m_contry_code.clone();
        }

        //BBS: version check
        let mut dont_load_config = !self.m_load_config;
        if self.m_bambuslicer_generator_version.is_some() {
            // bbs_3mf.cpp:2263-2266: the version comparison is commented out
        } else {
            self.m_bambuslicer_generator_version = crate::semver::Semver::parse("0.0.0.0");
            dont_load_config = true;
        }

        // we then loop again the entries to read other files stored in the archive
        for i in 0..num_entries {
            if let Some(stat) = mz_zip_reader_file_stat(&archive, i) {
                //BBS progress point
                if pro_fn.is_some() {
                    progress(IMPORT_STAGE_EXTRACT, i as i32, num_entries as i32, &mut cb_cancel);
                    if cb_cancel {
                        return Ok(false);
                    }
                }

                let name = stat.m_filename.replace('\\', "/");

                log::info!("extract {}th file {}, total={}", i + 1, name, num_entries);

                if name.contains("/../") {
                    log::warn!("find file path including /../, not valid, skip it\n");
                    continue;
                }

                if iequals(&name, BBS_LAYER_HEIGHTS_PROFILE_FILE) {
                    // extract slic3r layer heights profile file
                    self._extract_layer_heights_profile_config_from_archive(&mut archive, &stat);
                } else if iequals(&name, LAYER_CONFIG_RANGES_FILE) {
                    // extract slic3r layer config ranges file
                    self._extract_layer_config_ranges_from_archive(&mut archive, &stat, config_substitutions)?;
                } else if iequals(&name, BRIM_EAR_POINTS_FILE) {
                    // extract slic3r config file
                    self._extract_brim_ear_points_from_archive(&mut archive, &stat)?;
                }
                //BBS: disable SLA related files currently
                //BBS: project setting file
                if !dont_load_config && iequals(&name, BBS_PROJECT_CONFIG_FILE) {
                    // extract slic3r print config file
                    self._extract_project_config_from_archive(&mut archive, &stat, config, config_substitutions, model);
                } else if iequals(&name, CUT_INFORMATION_FILE) {
                    // extract object cut info
                    self._extract_cut_information_from_archive(&mut archive, &stat, config_substitutions)?;
                }
                //BBS: project embedded presets
                else if !dont_load_config && istarts_with(&name, PROJECT_EMBEDDED_PRINT_PRESETS_FILE) {
                    // extract slic3r layer config ranges file
                    self._extract_project_embedded_presets_from_archive(&mut archive, &stat, project_presets, model, crate::preset::Type::TypePrint, false);
                } else if !dont_load_config && istarts_with(&name, PROJECT_EMBEDDED_SLICE_PRESETS_FILE) {
                    // extract slic3r layer config ranges file
                    self._extract_project_embedded_presets_from_archive(&mut archive, &stat, project_presets, model, crate::preset::Type::TypePrint, true);
                } else if !dont_load_config && istarts_with(&name, PROJECT_EMBEDDED_FILAMENT_PRESETS_FILE) {
                    // extract slic3r layer config ranges file
                    self._extract_project_embedded_presets_from_archive(&mut archive, &stat, project_presets, model, crate::preset::Type::TypeFilament, true);
                } else if !dont_load_config && istarts_with(&name, PROJECT_EMBEDDED_PRINTER_PRESETS_FILE) {
                    // extract slic3r layer config ranges file
                    self._extract_project_embedded_presets_from_archive(&mut archive, &stat, project_presets, model, crate::preset::Type::TypePrinter, true);
                } else if !dont_load_config && iequals(&name, CUSTOM_GCODE_PER_PRINT_Z_FILE) {
                    // extract slic3r layer config ranges file
                    self._extract_custom_gcode_per_print_z_from_archive(&mut archive, &stat, model)?;
                } else if iequals(&name, BBS_MODEL_CONFIG_FILE) {
                    // extract slic3r model config file
                    if !self._extract_xml_from_archive(&mut archive, &stat, XmlHandlers::Config, Some(model)) && self.m_is_bbl_3mf {
                        if let Some(e) = self.m_pending_exception.take() {
                            return Err(e);
                        }
                        self.add_error("Archive does not contain a valid model config");
                        return Ok(false);
                    }
                } else if self._is_svg_shape_file(&name) {
                    self._extract_embossed_svg_shape_file(&name, &mut archive, &stat, model);
                } else if !dont_load_config && iequals(&name, SLICE_INFO_CONFIG_FILE) {
                    self.m_parsing_slice_info = true;
                    //extract slice info from archive
                    self._extract_xml_from_archive(&mut archive, &stat, XmlHandlers::Config, Some(model));
                    self.m_parsing_slice_info = false;
                } else if !dont_load_config && iequals(&name, FILAMENT_SEQUENCE_FILE) {
                    self._extract_filament_sequence_from_archive(&mut archive, &stat);
                } else if !dont_load_config && iequals(&name, ASSEMBLY_TREE_FILE) {
                    self._extract_assembly_tree_from_archive(&mut archive, &stat, model);
                } else if !dont_load_config && iequals(&name, ASSEMBLY_STEP_JSON_FILE) {
                    self._extract_assembly_steps_json_from_archive(&mut archive, &stat, model);
                } else if !dont_load_config && iequals(&name, ASSEMBLY_MODEL_FILE) {
                    self._extract_assembly_model_from_archive(&mut archive, &stat, model);
                } else if istarts_with(&name, AUXILIARY_DIR) {
                    // extract auxiliary directory to temp directory, do nothing for restore
                    if self.m_load_aux && !self.m_load_restore {
                        self._extract_auxiliary_file_from_archive(&mut archive, &stat, model);
                    }
                } else if !dont_load_config && istarts_with(&name, METADATA_DIR) && iends_with(&name, GCODE_EXTENSION) {
                    //load gcode files
                    self._extract_file_from_archive(&mut archive, &stat);
                } else if !dont_load_config && istarts_with(&name, METADATA_DIR) && iends_with(&name, THUMBNAIL_EXTENSION) {
                    //BBS parsing pattern thumbnail and plate thumbnails
                    self._extract_file_from_archive(&mut archive, &stat);
                } else if !dont_load_config && istarts_with(&name, METADATA_DIR) && iends_with(&name, CALIBRATION_INFO_EXTENSION) {
                    //BBS parsing pattern config files
                    self._extract_file_from_archive(&mut archive, &stat);
                } else {
                    log::info!("{} skipped, already parsed or a directory or not supported\n", name);
                }
                if let Some(e) = self.m_pending_exception.take() {
                    return Err(e);
                }
            }
        }

        close_zip_reader(archive);

        if !self.m_is_bbl_3mf {
            // if the 3mf was not produced by BambuStudio and there is more than one instance,
            // split the object in as many objects as instances
            log::info!("found 3mf from other vendor, split as instance");
            let objects: Vec<(Id, i32)> = self.m_objects.iter().map(|(k, v)| (k.clone(), *v)).collect();
            for (object_first, object_second) in objects {
                if object_second >= model.objects.len() as i32 {
                    self.add_error(format!("3rd 3mf, invalid object, id: {}", object_first.1));
                    return Ok(false);
                }
                let model_object_idx = object_second as usize;
                if model.objects[model_object_idx].instances.len() > 1 {
                    let current_object_name = match self.m_current_objects.get(&object_first) {
                        Some(o) => o.name.clone(),
                        None => {
                            self.add_error(format!("3rd 3mf, can not find object, id {}", object_first.1));
                            return Ok(false);
                        }
                    };
                    let mut object_id_list: Vec<Component> = Vec::new();
                    Self::_generate_current_object_list(&mut object_id_list, object_first.clone(), &self.m_current_objects);

                    let mut volumes: Vec<VolumeMetadata> = Vec::new();

                    for k in 0..object_id_list.len() {
                        let object_id = &object_id_list[k].object_id;
                        volumes.push(VolumeMetadata::with_subobject(object_id.1, ModelVolumeType::ModelPart));
                    }

                    // A single-object file is temporarily named after the filename. Put the
                    // 3MF object name back before cloning, so every split instance keeps it.
                    // The volume name is taken from the object name while the mesh is built.
                    if !current_object_name.is_empty() {
                        model.objects[model_object_idx].name = current_object_name;
                    }

                    // for each instance after the 1st, create a new model object containing only that instance
                    // and copy into it the geometry
                    while model.objects[model_object_idx].instances.len() > 1 {
                        let source = model.objects[model_object_idx].clone();
                        let last_instance = source.instances.last().unwrap().clone();
                        {
                            let new_model_object = model.add_object_clone(&source);
                            new_model_object.clear_instances();
                            new_model_object.add_instance_copy(&last_instance);
                        }
                        model.objects[model_object_idx].delete_last_instance();
                        let new_idx = model.objects.len() - 1;
                        if !self._generate_volumes_new(model, new_idx, &object_id_list, &volumes, config_substitutions)? {
                            return Ok(false);
                        }
                    }
                }
            }
        }

        log::info!("process group colors, size {}\n", self.m_group_id_to_color.len());
        type ColorKey = (i32, i32);
        let mut color_group_index_to_extruder_id_map: BTreeMap<ColorKey, i32> = BTreeMap::new();
        let mut color_to_extruder_id_map: BTreeMap<String, i32> = BTreeMap::new();
        let mut extruder_id = 0;
        for (group_id, colors) in &self.m_group_id_to_color {
            for (color_index, color_string) in colors.iter().enumerate() {
                let mapped_extruder = match color_to_extruder_id_map.get(color_string) {
                    None => {
                        extruder_id += 1;
                        color_to_extruder_id_map.insert(color_string.clone(), extruder_id);
                        extruder_id
                    }
                    Some(e) => *e,
                };
                color_group_index_to_extruder_id_map.insert((*group_id, color_index as i32), mapped_extruder);
            }
        }

        log::info!("begin to assemble objects, size {}\n", self.m_objects.len());
        //only load objects in plate_id
        let mut current_plate_key: Option<i32> = None;
        if plate_id > 0 && plate_id <= self.m_plater_data.len() as i32 {
            if self.m_plater_data.contains_key(&plate_id) {
                current_plate_key = Some(plate_id);
            }
        }
        let objects: Vec<(Id, i32)> = self.m_objects.iter().map(|(k, v)| (k.clone(), *v)).collect();
        for (object_first, object_second) in objects {
            if object_second >= model.objects.len() as i32 {
                self.add_error(format!("invalid object, id: {}", object_first.1));
                return Ok(false);
            }
            if let Some(key) = current_plate_key {
                let current_plate_data = self.m_plater_data.get(&key).unwrap();
                if !current_plate_data.obj_inst_map.contains_key(&object_first.1) {
                    //not in current plate, skip
                    log::info!("could not find object {} in plate {}, skip it\n", object_first.1, plate_id);
                    continue;
                }
            }
            let model_object_idx = object_second as usize;

            // m_layer_heights_profiles are indexed by a 1 based model object index.
            if let Some(profile) = self.m_layer_heights_profiles.remove(&(object_second + 1)) {
                model.objects[model_object_idx].layer_height_profile.set(&profile);
                self.m_layer_heights_profiles.insert(object_second + 1, Vec::new());
            }

            // m_layer_config_ranges are indexed by a 1 based model object index.
            if let Some(ranges) = self.m_layer_config_ranges.get_mut(&(object_second + 1)) {
                model.objects[model_object_idx].layer_config_ranges = std::mem::take(ranges);
            }

            if let Some(points) = self.m_brim_ear_points.get_mut(&(object_second + 1)) {
                model.objects[model_object_idx].brim_points = std::mem::take(points);
            }

            let mut object_id_list: Vec<Component> = Vec::new();
            Self::_generate_current_object_list(&mut object_id_list, object_first.clone(), &self.m_current_objects);

            let mut volumes: Vec<VolumeMetadata> = Vec::new();
            let volumes_ref: Vec<VolumeMetadata>;

            if let Some(obj_metadata) = self.m_objects_metadata.get(&object_first.1) {
                // config data has been found, this model was saved using slic3r pe

                // apply object's name and config data
                for metadata in &obj_metadata.metadata {
                    let model_object = &mut model.objects[model_object_idx];
                    if metadata.key == "name" {
                        model_object.name = metadata.value.clone();
                    }
                    //BBS: add module name
                    else if metadata.key == "module" {
                        model_object.module_name = metadata.value.clone();
                    } else {
                        model_object.config.set_deserialize(&metadata.key, &metadata.value, config_substitutions, false)?;
                    }
                }

                // select object's detected volumes
                volumes_ref = obj_metadata.volumes.clone();
            } else {
                // config data not found, this model was not saved using slic3r pe

                // add the entire geometry as the single volume to generate
                for k in 0..object_id_list.len() {
                    let object_id = &object_id_list[k].object_id;
                    volumes.push(VolumeMetadata::with_subobject(object_id.1, ModelVolumeType::ModelPart));
                }

                if let Some(current_object) = self.m_current_objects.get(&object_first) {
                    let model_object = &mut model.objects[model_object_idx];
                    // get name
                    if !current_object.name.is_empty() {
                        model_object.name = current_object.name.clone();
                    } else {
                        model_object.name = format!("Object_{}", object_second + 1);
                    }

                    // get color
                    let object_pid = current_object.pid;
                    if object_pid >= 0 {
                        let mut object_pindex = current_object.pindex;
                        if let Some(group) = self.m_group_id_to_color.get(&object_pid) {
                            if !group.is_empty() {
                                if object_pindex < 0 || object_pindex >= group.len() as i32 {
                                    object_pindex = 0;
                                }
                                let key: ColorKey = (object_pid, object_pindex);
                                if let Some(extruder) = color_group_index_to_extruder_id_map.get(&key) {
                                    model_object.config.set_key_value("extruder", Box::new(crate::config::ConfigOptionInt::with_value(*extruder)));
                                }
                            }
                        }
                    }
                }

                // select as volumes
                volumes_ref = std::mem::take(&mut volumes);
            }

            if !self._generate_volumes_new(model, model_object_idx, &object_id_list, &volumes_ref, config_substitutions)? {
                return Ok(false);
            }

            // Apply cut information for object if any was loaded
            // m_cut_object_ids are indexed by a 1 based model object index.
            if let Some(cut_object_info) = self.m_cut_object_infos.get(&(object_second + 1)).cloned() {
                model.objects[model_object_idx].cut_id = cut_object_info.id.clone();

                for connector in &cut_object_info.connectors {
                    // volume_id and type come straight out of Metadata/cut_information.xml, i.e. from
                    // untrusted file content. They must be range-checked before use: assert() is compiled
                    // out in release builds, so a crafted or corrupted 3MF would index ModelObject::volumes
                    // out of bounds and write a CutInfo through the resulting garbage pointer.
                    if connector.volume_id < 0 || connector.volume_id >= model.objects[model_object_idx].volumes.len() as i32 {
                        self.add_error(format!("Invalid cut connector volume_id {}", connector.volume_id));
                        continue;
                    }
                    if connector.type_ < 0 || connector.type_ > crate::model::CutConnectorType::Undef as i32 {
                        self.add_error(format!("Invalid cut connector type {}", connector.type_));
                        continue;
                    }
                    model.objects[model_object_idx].volumes[connector.volume_id as usize].cut_info = crate::model::ModelVolumeCutInfo::new(
                        cut_connector_type_from_int(connector.type_),
                        connector.radius,
                        connector.height,
                        connector.r_tolerance,
                        connector.h_tolerance,
                        true,
                    );
                }
            }
        }
        // BBS: replay deferred per-volume assemble matrices collected while parsing <assemble_item volume_id="..."> in model_settings.config..
        for entry in &self.m_pending_volume_assemble {
            if entry.model_object_index < 0 || entry.model_object_index >= model.objects.len() as i32 {
                continue;
            }
            let mo = &mut model.objects[entry.model_object_index as usize];
            if entry.volume_id < 0 || entry.volume_id >= mo.volumes.len() as i32 {
                continue;
            }
            mo.volumes[entry.volume_id as usize].set_assemble_from_transform(&entry.transform);
        }
        self.m_pending_volume_assemble.clear();

        // If instances contain a single volume, the volume offset should be 0,0,0
        // This equals to say that instance world position and volume world position should match
        // Correct all instances/volumes for which this does not hold
        for obj_id in 0..model.objects.len() {
            let o = &mut model.objects[obj_id];
            if o.volumes.len() == 1 {
                let first_inst_trafo = o.instances[0].get_transformation().clone();
                let vol_trafo = o.volumes[0].get_transformation().clone();
                let world_vol_offset = (first_inst_trafo.clone() * vol_trafo.clone()).get_offset();
                let world_inst_offset = first_inst_trafo.get_offset();

                if !vec3d_is_approx(&world_vol_offset, &world_inst_offset) {
                    for inst_id in 0..o.instances.len() {
                        let inst_trafo = o.instances[inst_id].get_transformation().clone();
                        let offset = (inst_trafo * vol_trafo.clone()).get_offset();
                        o.instances[inst_id].set_offset(&offset);
                    }
                    o.volumes[0].set_offset(&Vec3d::zeros());
                }
            }
        }

        for (object_idx, o) in model.objects.iter_mut().enumerate() {
            for (volume_idx, v) in o.volumes.iter_mut().enumerate() {
                if v.source.input_file.is_empty() && v.type_() == ModelVolumeType::ModelPart {
                    v.source.input_file = filename.to_string();
                    if v.source.volume_idx == -1 {
                        v.source.volume_idx = volume_idx as i32;
                    }
                    if v.source.object_idx == -1 {
                        v.source.object_idx = object_idx as i32;
                    }
                }
            }
        }

        let max_filament_id = {
            use crate::config::ConfigOptionResolver;
            match config.option("filament_settings_id").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionStrings>()) {
                Some(opt) => opt.values.len() as i32,
                None => i32::MAX,
            }
        };
        for mo in model.objects.iter_mut() {
            let extruder_opt = mo.config.option("extruder").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionInt>()).map(|o| o.value);
            let mut extruder_id = 0;
            if let Some(v) = extruder_opt {
                extruder_id = v;
            }

            if extruder_id == 0 || extruder_id > max_filament_id {
                mo.config.set_key_value("extruder", Box::new(crate::config::ConfigOptionInt::with_value(1)));
            }

            if mo.volumes.len() == 1 {
                mo.volumes[0].config.erase("extruder");
            } else {
                for mv in mo.volumes.iter_mut() {
                    let vol_extruder_opt =
                        mv.config.option("extruder").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionInt>()).map(|o| o.value);
                    let v = match vol_extruder_opt {
                        None => continue,
                        Some(v) => v,
                    };

                    if v == 0 {
                        mv.config.erase("extruder");
                    } else if v > max_filament_id {
                        mv.config.set_key_value("extruder", Box::new(crate::config::ConfigOptionInt::with_value(1)));
                    }
                }
            }
        }

        //BBS progress point
        log::info!(
            "import 3mf IMPORT_STAGE_LOADING_PLATES, m_plater_data size {}, m_backup_path {}\n",
            self.m_plater_data.len(),
            self.m_backup_path
        );
        if pro_fn.is_some() {
            progress(IMPORT_STAGE_LOADING_PLATES, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        //BBS: load the plate info into plate_data_list
        plate_data_list.clear();
        plate_data_list.reserve(self.m_plater_data.len());
        for _ in 0..self.m_plater_data.len() {
            plate_data_list.push(PlateData::new());
        }
        let n = self.m_plater_data.len() as i32;
        let keys: Vec<i32> = self.m_plater_data.keys().cloned().collect();
        let join = |backup: &str, f: &str| -> String {
            if self.m_load_restore || f.is_empty() {
                f.to_string()
            } else {
                bbs_join_path_within_dir(backup, f)
            }
        };
        for key in keys {
            if key <= 0 || key > n {
                self.add_error("invalid plate index");
                return Ok(false);
            }
            let src = self.m_plater_data.get(&key).unwrap();
            let dst = &mut plate_data_list[(key - 1) as usize];
            dst.locked = src.locked;
            dst.plate_index = src.plate_index - 1;
            dst.plate_name = src.plate_name.clone();
            dst.obj_inst_map = src.obj_inst_map.clone();
            dst.gcode_file = join(&self.m_backup_path, &src.gcode_file);
            dst.gcode_prediction = src.gcode_prediction.clone();
            dst.gcode_weight = src.gcode_weight.clone();
            dst.toolpath_outside = src.toolpath_outside;
            dst.is_support_used = src.is_support_used;
            dst.is_label_object_enabled = src.is_label_object_enabled;
            dst.support_material_on_wipe_tower = src.support_material_on_wipe_tower;
            dst.slice_filaments_info = src.slice_filaments_info.clone();
            dst.slice_filaments_extra = src.slice_filaments_extra.clone();
            dst.nozzles_info = src.nozzles_info.clone();
            dst.skipped_objects = src.skipped_objects.clone();
            dst.warnings = src.warnings.clone();
            dst.thumbnail_file = join(&self.m_backup_path, &src.thumbnail_file);
            //plate_data_list[it->first-1]->pattern_file = ...
            dst.no_light_thumbnail_file = join(&self.m_backup_path, &src.no_light_thumbnail_file);
            dst.top_file = join(&self.m_backup_path, &src.top_file);
            dst.pick_file = join(&self.m_backup_path, &src.pick_file);
            dst.pattern_bbox_file = join(&self.m_backup_path, &src.pattern_bbox_file);
            dst.config = src.config.clone();
            dst.filament_maps = src.filament_maps.clone();
            dst.filament_change_sequence = src.filament_change_sequence.clone();
            dst.nozzle_change_sequence = src.nozzle_change_sequence.clone();
            dst.optimal_assignment = src.optimal_assignment.clone();
            dst.nozzle_volume_types = src.nozzle_volume_types.clone();
            dst.nozzle_diameters = src.nozzle_diameters.clone();

            log::info!("plate {}, thumbnail_file={}, no_light_thumbnail_file={}", key, dst.thumbnail_file, dst.no_light_thumbnail_file);
            log::info!("top_thumbnail_file={}, pick_thumbnail_file={}", dst.top_file, dst.pick_file);

            //update the arrange order
            let obj_inst_map: Vec<(i32, (i32, i32))> = dst.obj_inst_map.iter().map(|(k, v)| (*k, *v)).collect();
            for (obj_id, (inst_index, loaded)) in obj_inst_map {
                let index_iter = match self.m_index_paths.get(&obj_id) {
                    None => {
                        log::warn!("can not find object from plate's obj_map, id={}, skip this object", obj_id);
                        continue;
                    }
                    Some(p) => p.clone(),
                };
                let temp_id: Id = (index_iter.clone(), obj_id);
                let obj_index = match self.m_objects.get(&temp_id) {
                    None => {
                        log::warn!("can not find object from plate's obj_map, ID <{}, {}>, skip this object", index_iter, obj_id);
                        continue;
                    }
                    Some(i) => *i,
                };

                if obj_index as usize >= model.objects.len() || obj_index < 0 {
                    log::warn!("invalid object id {}\n", obj_index);
                    continue;
                }
                let obj = &mut model.objects[obj_index as usize];
                if inst_index as usize >= obj.instances.len() || inst_index < 0 {
                    log::warn!("invalid instance id {}\n", inst_index);
                    continue;
                }
                obj.instances[inst_index as usize].loaded_id = loaded as usize;
            }
        }

        if plate_id > 0 && plate_id <= self.m_plater_data.len() as i32 {
            //remove the no need objects
            let mut delete_ids: Vec<usize> = Vec::new();
            for (index, obj) in model.objects.iter().enumerate() {
                if obj.volumes.is_empty() {
                    //remove this model objects
                    delete_ids.push(index);
                }
            }

            for index in delete_ids.iter().rev() {
                model.delete_object(*index);
            }
        }
        log::info!("import 3mf IMPORT_STAGE_FINISH\n");
        if pro_fn.is_some() {
            progress(IMPORT_STAGE_FINISH, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }
        // NOTE: we do NOT deserialize m_assembly_steps_tree_data here, even though ModelObject::name is finalized by this point.
        Ok(true)
    }

    /// bbs_3mf.cpp:2833-2835
    fn _is_svg_shape_file(&self, name: &str) -> bool {
        name.starts_with(MODEL_FOLDER) && name.ends_with(".svg")
    }

    /// bbs_3mf.cpp:2837-2896
    fn _extract_from_archive(
        ctx: &ExtractContext,
        archive: &mut MzZipArchive,
        path: &str,
        extract: &mut dyn FnMut(&mut MzZipArchive, &MzZipArchiveFileStat) -> bool,
        restore: bool,
    ) -> bool {
        let mut path2 = path.to_string();
        if path2.starts_with('/') {
            path2 = path2[1..].to_string();
        }
        // try utf8 encoding
        let mut index = mz_zip_reader_locate_file(archive, &path2);
        if index < 0 {
            // try native encoding
            let native_path = crate::utils::encode_path(&path2);
            index = mz_zip_reader_locate_file(archive, &native_path);
        }
        if index < 0 {
            // try unicode path extra
            for i in 0..archive.m_total_files() {
                let extra = mz_zip_reader_get_extra(archive, i);
                let n = extra.len().min(1024);
                if n > 0 && path2 == ZipUnicodePathExtraField::decode(&extra[..n], "") {
                    index = i as i32;
                    break;
                }
            }
        }
        let stat = if index >= 0 { mz_zip_reader_file_stat(archive, index as u32) } else { None };
        let stat = match stat {
            Some(s) => s,
            None => {
                if restore {
                    let mut paths = vec![format!("{}{}", ctx.m_backup_path, path)];
                    if !ctx.m_origin_file.is_empty() {
                        paths.push(ctx.m_origin_file.clone());
                    }
                    for path2 in &paths {
                        let mut result = false;
                        if std::path::Path::new(path2).exists() {
                            if let Some(mut archive2) = open_zip_reader(path2) {
                                result = Self::_extract_from_archive(ctx, &mut archive2, path, extract, false);
                                close_zip_reader(archive2);
                            }
                        }
                        if result {
                            return result;
                        }
                    }
                }
                let error_buf = format!("File {} not found from archive", path);
                ctx.base.add_error(error_buf);
                return false;
            }
        };
        if !extract(archive, &stat) {
            return false;
        }
        true
    }

    /// bbs_3mf.cpp:2898-2903 `_extract_xml_from_archive(archive, path, start_handler, end_handler)`
    fn _extract_xml_from_archive_path(
        &mut self,
        archive: &mut MzZipArchive,
        path: &str,
        handlers: XmlHandlers,
        model: Option<&mut Model>,
        _unused: Option<()>,
    ) -> bool {
        let ctx = self.extract_context();
        let mut model = model;
        Self::_extract_from_archive(&ctx, archive, path, &mut |archive, stat| {
            let m = model.as_deref_mut();
            self._extract_xml_from_archive(archive, stat, handlers, m)
        }, false)
    }

    /// bbs_3mf.cpp:2905-2948 `_extract_xml_from_archive(archive, stat, start_handler, end_handler)`
    fn _extract_xml_from_archive(
        &mut self,
        archive: &mut MzZipArchive,
        stat: &MzZipArchiveFileStat,
        handlers: XmlHandlers,
        model: Option<&mut Model>,
    ) -> bool {
        // XML_GetBuffer() takes an int; reject sizes that would be truncated, otherwise the buffer
        // is allocated from the truncated size while the extraction writes the full 64-bit size.
        if stat.m_uncomp_size == 0 || stat.m_uncomp_size > i32::MAX as u64 {
            self.add_error("Found invalid size");
            return false;
        }

        self._destroy_xml_parser();

        self.m_xml_parser = true;
        self.m_parse_error = false;
        self.m_parse_error_message.clear();

        let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
            Some(b) => b,
            None => {
                self.add_error("Error while reading config data to buffer");
                return false;
            }
        };

        let mut model = model;
        let result = expat_parse(&buffer, &mut |ev| {
            match ev {
                SaxEvent::Start(name, atts) => match handlers {
                    XmlHandlers::Model => ImporterXmlCallbacks::_handle_start_model_xml_element(Some(&mut *self), model.as_deref_mut(), name, atts),
                    XmlHandlers::Config => ImporterXmlCallbacks::_handle_start_config_xml_element(Some(&mut *self), model.as_deref_mut(), name, atts),
                    XmlHandlers::Relationships => ImporterXmlCallbacks::_handle_start_relationships_element(Some(&mut *self), name, atts),
                },
                SaxEvent::End(name) => match handlers {
                    XmlHandlers::Model => ImporterXmlCallbacks::_handle_end_model_xml_element(Some(&mut *self), model.as_deref_mut(), name),
                    XmlHandlers::Config => ImporterXmlCallbacks::_handle_end_config_xml_element(Some(&mut *self), name),
                    XmlHandlers::Relationships => ImporterXmlCallbacks::_handle_end_relationships_element(Some(&mut *self), name),
                },
                SaxEvent::Characters(s) => ImporterXmlCallbacks::_handle_xml_characters(Some(&mut *self), s),
            }
            !self.parse_error()
        });
        if let Err(e) = result {
            let error_buf = format!("Error ({}) while parsing xml file at line {}", e.message, e.line);
            self.add_error(error_buf);
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:2950-3015
    fn _extract_model_from_archive(&mut self, model: &mut Model, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat) -> bool {
        if stat.m_uncomp_size == 0 {
            self.add_error("Found invalid size");
            return false;
        }

        self._destroy_xml_parser();

        self.m_xml_parser = true;
        self.m_parse_error = false;
        self.m_parse_error_message.clear();

        let data = match mz_zip_reader_extract_to_mem(archive, stat.m_file_index) {
            Some(d) => d,
            None => {
                self.add_error("Error while extracting model data from zip archive");
                return false;
            }
        };
        let result = expat_parse(&data, &mut |ev| {
            match ev {
                SaxEvent::Start(name, atts) => ImporterXmlCallbacks::_handle_start_model_xml_element(Some(&mut *self), Some(&mut *model), name, atts),
                SaxEvent::End(name) => ImporterXmlCallbacks::_handle_end_model_xml_element(Some(&mut *self), Some(&mut *model), name),
                SaxEvent::Characters(s) => ImporterXmlCallbacks::_handle_xml_characters(Some(&mut *self), s),
            }
            !self.parse_error()
        });
        if let Err(e) = result {
            let error_buf = format!("Error ({}) while parsing '{}' at line {}", self.parse_error_message(&e.message), stat.m_filename, e.line);
            self.add_error(error_buf);
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:3017-3071
    fn _extract_cut_information_from_archive(
        &mut self,
        archive: &mut MzZipArchive,
        stat: &MzZipArchiveFileStat,
        _config_substitutions: &mut crate::config::ConfigSubstitutionContext,
    ) -> Result<(), Bbs3mfException> {
        if stat.m_uncomp_size > 0 {
            let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
                Some(b) => b,
                None => {
                    self.add_error("Error while reading cut information data to buffer");
                    return Ok(());
                }
            };

            let objects_tree = ptree_read_xml(&buffer).ok_or_else(|| Bbs3mfException("cut_information.xml: xml parser error".into()))?;

            let objects = objects_tree.get_child("objects").ok_or_else(|| Bbs3mfException("No such node (objects)".into()))?;
            for (_, object_tree) in &objects.children {
                let obj_idx = object_tree.get_or::<i32>("<xmlattr>.id", -1);
                if obj_idx <= 0 {
                    self.add_error("Found invalid object id");
                    continue;
                }

                if self.m_cut_object_infos.contains_key(&obj_idx) {
                    self.add_error("Found duplicated cut_object_id");
                    continue;
                }

                let mut cut_id = crate::object_id::CutObjectBase::new();
                let mut connectors: Vec<CutConnectorInfo> = Vec::new();

                for (key, obj_cut_info) in &object_tree.children {
                    if key == "cut_id" {
                        let cut_id_tree = obj_cut_info;
                        let bad = |p: &str| Bbs3mfException(format!("No such node ({})", p));
                        cut_id = crate::object_id::CutObjectBase::from_full(
                            crate::object_id::ObjectID::new(cut_id_tree.get_value::<usize>("<xmlattr>.id").ok_or_else(|| bad("<xmlattr>.id"))?),
                            cut_id_tree.get_value::<usize>("<xmlattr>.check_sum").ok_or_else(|| bad("<xmlattr>.check_sum"))?,
                            cut_id_tree.get_value::<usize>("<xmlattr>.connectors_cnt").ok_or_else(|| bad("<xmlattr>.connectors_cnt"))?,
                        );
                    }
                    if key == "connectors" {
                        let cut_connectors_tree = obj_cut_info;
                        for (ckey, connector_tree) in &cut_connectors_tree.children {
                            if ckey != "connector" {
                                continue;
                            }
                            let bad = |p: &str| Bbs3mfException(format!("No such node ({})", p));
                            let connector = CutConnectorInfo {
                                volume_id: connector_tree.get_value::<i32>("<xmlattr>.volume_id").ok_or_else(|| bad("<xmlattr>.volume_id"))?,
                                type_: connector_tree.get_value::<i32>("<xmlattr>.type").ok_or_else(|| bad("<xmlattr>.type"))?,
                                radius: connector_tree.get_or::<f32>("<xmlattr>.radius", 0.0),
                                height: connector_tree.get_or::<f32>("<xmlattr>.height", 0.0),
                                r_tolerance: connector_tree.get_value::<f32>("<xmlattr>.r_tolerance").ok_or_else(|| bad("<xmlattr>.r_tolerance"))?,
                                h_tolerance: connector_tree.get_value::<f32>("<xmlattr>.h_tolerance").ok_or_else(|| bad("<xmlattr>.h_tolerance"))?,
                            };
                            connectors.push(connector);
                        }
                    }
                }

                let cut_info = CutObjectInfo { id: cut_id, connectors };
                self.m_cut_object_infos.insert(obj_idx, cut_info);
            }
        }
        Ok(())
    }

    /// bbs_3mf.cpp:3073-3084
    ///
    /// FIDELITY-NOTE: `ConfigBase::load_from_gcode_string_legacy` is not in the crate; the call
    /// site (bbs_3mf.cpp:2318-2321) is commented out at the pin, so nothing reaches this.
    #[allow(dead_code)]
    fn _extract_print_config_from_archive(
        &mut self,
        archive: &mut MzZipArchive,
        stat: &MzZipArchiveFileStat,
        config: &mut DynamicPrintConfig,
        config_substitutions: &mut crate::config::ConfigSubstitutionContext,
        _archive_filename: &str,
    ) {
        if stat.m_uncomp_size > 0 {
            let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
                Some(b) => b,
                None => {
                    self.add_error("Error while reading config data to buffer");
                    return;
                }
            };
            // ConfigBase::load_from_gcode_string_legacy(config, buffer.data(), config_substitutions):
            // "; key = value" lines.
            let text = String::from_utf8_lossy(&buffer);
            for line in text.lines() {
                if let Some(rest) = line.strip_prefix("; ") {
                    if let Some(eq) = rest.find(" = ") {
                        let key = &rest[..eq];
                        let value = &rest[eq + 3..];
                        use crate::config::ConfigBase;
                        let _ = config.set_deserialize(key, value, config_substitutions, false);
                    }
                }
            }
        }
    }

    //BBS: extract project config from json files
    /// bbs_3mf.cpp:3087-3117
    fn _extract_project_config_from_archive(
        &mut self,
        archive: &mut MzZipArchive,
        stat: &MzZipArchiveFileStat,
        config: &mut DynamicPrintConfig,
        config_substitutions: &mut crate::config::ConfigSubstitutionContext,
        model: &mut Model,
    ) {
        if stat.m_uncomp_size > 0 {
            let temp_path = model.get_backup_path();

            let dest_file = format!("{}/{}", temp_path, "_temp_3.config");
            let dest_zip_file = crate::utils::encode_path(&dest_file);
            let res = mz_zip_reader_extract_to_file(archive, stat.m_file_index, &dest_zip_file);
            log::info!("extract  {} from 3mf {}, ret {}\n", dest_file, stat.m_filename, res as i32);
            if !res {
                self.add_error("Error while extract project config file to file");
                return;
            }
            let mut key_values: BTreeMap<String, String> = BTreeMap::new();
            let mut reason = String::new();
            use crate::config::ConfigBase;
            let ret = config.load_from_json_with_context(&dest_file, config_substitutions, true, &mut key_values, &mut reason);
            if ret != 0 {
                self.add_error(format!("Error load config from json:{}", reason));
                return;
            }
            log::info!("load project config file successfully from {}\n", dest_file);

            // Heal any gradient-curve slots corrupted by the legacy "|" separator collision
            // (see FilamentMixer::sanitize_mixed_gradient_curve_array). The 3MF JSON itself
            // is safe (";" + C-style escape), but older projects saved through the buggy
            // export_selections/load_selections path may already carry single-point entries
            // that fail MakerWorld's "curve needs >= 2 points" check.
            if let Ok(Some(curve_opt)) = crate::config::ConfigBaseTyped::option_typed_mut::<crate::config::ConfigOptionStrings>(&mut **config, "filament_mixed_gradient_curve", false) {
                sanitize_mixed_gradient_curve_array(&mut curve_opt.values);
            }
        }
    }

    //BBS: extract project embedded presets
    /// bbs_3mf.cpp:3120-3215
    fn _extract_project_embedded_presets_from_archive(
        &mut self,
        archive: &mut MzZipArchive,
        stat: &MzZipArchiveFileStat,
        project_presets: &mut Vec<Preset>,
        _model: &mut Model,
        type_: crate::preset::Type,
        use_json: bool,
    ) {
        if stat.m_uncomp_size > 0 {
            let dest_file = format!("{}/{}", self.m_backup_path, "_temp_2.config");
            let dest_zip_file = crate::utils::encode_path(&dest_file);
            let res = mz_zip_reader_extract_to_file(archive, stat.m_file_index, &dest_zip_file);
            log::info!("extract  {} from 3mf {}, ret {}\n", dest_file, stat.m_filename, res as i32);
            if !res {
                self.add_error("Error while extract auxiliary file to file");
                return;
            }
            //load presets
            let mut config = DynamicPrintConfig::new();
            let mut key_values: BTreeMap<String, String> = BTreeMap::new();
            let mut reason = String::new();
            use crate::config::{ConfigBase, ConfigOptionResolver};
            let config_substitutions = if use_json {
                config.load_from_json(&dest_file, crate::config::ForwardCompatibilitySubstitutionRule::Enable, &mut key_values, &mut reason)
            } else {
                match config.load_from_ini(&dest_file, crate::config::ForwardCompatibilitySubstitutionRule::Enable) {
                    Ok(s) => s,
                    Err(e) => {
                        // the C++ load_from_ini throws; the exception escapes the loader
                        self.m_pending_exception = Some(Bbs3mfException(e.to_string()));
                        return;
                    }
                }
            };
            if !reason.is_empty() {
                log::error!("load project embedded config from  {} failed\n", dest_file);
                //skip this file
                return;
            }
            let preset_name: String;
            if type_ == crate::preset::Type::TypePrint {
                match config.option("print_settings_id").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionString>()) {
                    None => {
                        log::error!("can not found print_settings_id from  {}\n", dest_file);
                        //skip this file
                        return;
                    }
                    Some(print_name) => preset_name = print_name.value.clone(),
                }
            } else if type_ == crate::preset::Type::TypeFilament {
                match config.option("filament_settings_id").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionStrings>()) {
                    None => {
                        log::error!("can not found filament_settings_id from  {}\n", dest_file);
                        //skip this file
                        return;
                    }
                    Some(filament_names) => preset_name = filament_names.values[0].clone(),
                }
            } else if type_ == crate::preset::Type::TypePrinter {
                match config.option("printer_settings_id").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionString>()) {
                    None => {
                        log::error!("can not found printer_settings_id from  {}\n", dest_file);
                        //skip this file
                        return;
                    }
                    Some(print_name) => preset_name = print_name.value.clone(),
                }
            } else {
                log::error!("invalid type  {:?} from file {}\n", type_, dest_file);
                //skip this file
                return;
            }

            let mut preset = Preset::new(type_, &preset_name, false);
            preset.file = dest_file.clone();
            preset.config = config;
            preset.loaded = true;
            preset.is_project_embedded = true;
            preset.is_external = true;
            preset.is_dirty = false;

            let version_str = key_values.entry(crate::preset::BBL_JSON_KEY_VERSION.to_string()).or_default().clone();
            match crate::semver::Semver::parse(&version_str) {
                Some(version) => preset.version = version,
                None => {
                    preset.version = match &self.m_bambuslicer_generator_version {
                        Some(v) => v.clone(),
                        None => crate::semver::Semver::new(),
                    }
                }
            }
            if !config_substitutions.is_empty() {
                preset.loading_substitutions = Some(config_substitutions);
            }

            log::info!("create one project embedded preset: {} from {}, type {:?}\n", preset_name, dest_file, type_);
            project_presets.push(preset);
        }
    }

    /// bbs_3mf.cpp:3217-3296
    fn _extract_auxiliary_file_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat, model: &mut Model) {
        log::info!("stat.m_uncomp_size is {}", stat.m_uncomp_size);
        if stat.m_uncomp_size > 0 {
            let mut dest_file = if stat.m_is_utf8 {
                stat.m_filename.clone()
            } else {
                let extra = mz_zip_reader_get_extra(archive, stat.m_file_index);
                let n = extra.len().min(1024);
                ZipUnicodePathExtraField::decode(&extra[..n], &stat.m_filename)
            };

            if dest_file.is_empty() || dest_file.contains("..") || dest_file.starts_with('/') || dest_file.starts_with('\\') {
                return;
            }

            let temp_path = model.get_auxiliary_file_temp_path();
            let dir = std::path::PathBuf::from(&temp_path);

            match dest_file.find(AUXILIARY_DIR) {
                Some(found) => dest_file = dest_file[found + AUXILIARY_STR_LEN as usize..].to_string(),
                None => return,
            }

            if dest_file.is_empty() || dest_file.contains("..") || dest_file.starts_with('/') || dest_file.starts_with('\\') {
                log::error!("Invalid sub path");
                return;
            }

            if !bbs_auxiliary_subpath_safe_for_encoding(&dest_file) {
                let safe_dest = bbs_auxiliary_sanitize_after_encoding(&dest_file);
                if safe_dest.is_empty() || !bbs_auxiliary_subpath_safe_for_encoding(&safe_dest) {
                    log::error!("Invalid sub path: unsafe after local path encoding");
                    return;
                }
                log::warn!("Auxiliary sub path rewritten for local encoding safety: {} -> {}", dest_file, safe_dest);
                dest_file = safe_dest;
            }

            let short_dest = bbs_auxiliary_shorten_subpath_for_max_path(&temp_path, &dest_file);
            if short_dest != dest_file {
                log::warn!("Auxiliary sub path shortened to fit MAX_PATH: {} -> {}", dest_file, short_dest);
                dest_file = short_dest;
            }

            if dest_file.contains('/') || dest_file.contains('\\') {
                let src_path = std::path::PathBuf::from(&dest_file);
                if let Some(parent_path) = src_path.parent() {
                    let parent_full_path = dir.join(parent_path);
                    if !parent_full_path.exists() {
                        let _ = std::fs::create_dir_all(&parent_full_path);
                    }
                }
            }

            let final_path = dir.join(&dest_file);
            let dest_zip_file = crate::utils::encode_path(&final_path.to_string_lossy());

            if !bbs_encoded_path_within_dir(&crate::utils::encode_path(&temp_path), &dest_zip_file) {
                log::error!("Invalid sub path: escapes extraction dir after local path encoding");
                return;
            }

            let res = mz_zip_reader_extract_to_file(archive, stat.m_file_index, &dest_zip_file);
            log::info!("extract  {} from 3mf {}, ret {}\n", dest_file, stat.m_filename, res as i32);

            if !res {
                self.add_error("Error while extract auxiliary file to file");
            }
        }
    }

    /// bbs_3mf.cpp:3298-3326
    fn _extract_file_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat) {
        if stat.m_uncomp_size > 0 {
            let src_file = crate::utils::decode_path(&stat.m_filename);

            if src_file.contains("..") {
                self.add_error("invalid file name");
                return;
            }

            let base_path = std::path::PathBuf::from(&self.m_backup_path);
            let dest_path = base_path.join(&src_file);

            if let Some(parent) = dest_path.parent() {
                if !parent.as_os_str().is_empty() && !parent.exists() {
                    let _ = std::fs::create_dir_all(parent);
                }
            }

            let dest_zip_file = crate::utils::encode_path(&dest_path.to_string_lossy());
            let res = mz_zip_reader_extract_to_file(archive, stat.m_file_index, &dest_zip_file);

            log::info!("extract  {} from 3mf {}, ret {}\n", dest_path.display(), stat.m_filename, res as i32);

            if !res {
                self.add_error("Error while extract file to temp directory");
            }
        }
    }

    /// bbs_3mf.cpp:3328-3388
    fn _extract_layer_heights_profile_config_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat) {
        if stat.m_uncomp_size > 0 {
            let buffer = match mz_zip_reader_extract_to_mem(archive, stat.m_file_index) {
                Some(b) => b,
                None => {
                    self.add_error("Error while reading layer heights profile data to buffer");
                    return;
                }
            };
            let mut buffer = String::from_utf8_lossy(&buffer).into_owned();

            if buffer.ends_with('\n') {
                buffer.pop();
            }

            let objects: Vec<&str> = buffer.split('\n').collect();

            for object in objects {
                let object_data: Vec<&str> = object.split('|').collect();
                if object_data.len() != 2 {
                    self.add_error("Error while reading object data");
                    continue;
                }

                let object_data_id: Vec<&str> = object_data[0].split('=').collect();
                if object_data_id.len() != 2 {
                    self.add_error("Error while reading object id");
                    continue;
                }

                let object_id = atoi(object_data_id[1]);
                if object_id == 0 {
                    self.add_error("Found invalid object id");
                    continue;
                }

                if self.m_layer_heights_profiles.contains_key(&object_id) {
                    self.add_error("Found duplicated layer heights profile");
                    continue;
                }

                let object_data_profile: Vec<&str> = object_data[1].split(';').collect();
                if object_data_profile.len() <= 4 || object_data_profile.len() % 2 != 0 {
                    self.add_error("Found invalid layer heights profile");
                    continue;
                }

                let mut profile: Vec<f64> = Vec::with_capacity(object_data_profile.len());

                for value in object_data_profile {
                    profile.push(atof(value));
                }

                self.m_layer_heights_profiles.insert(object_id, profile);
            }
        }
    }

    /// bbs_3mf.cpp:3390-3446
    fn _extract_layer_config_ranges_from_archive(
        &mut self,
        archive: &mut MzZipArchive,
        stat: &MzZipArchiveFileStat,
        config_substitutions: &mut crate::config::ConfigSubstitutionContext,
    ) -> Result<(), Bbs3mfException> {
        if stat.m_uncomp_size > 0 {
            let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
                Some(b) => b,
                None => {
                    self.add_error("Error while reading layer config ranges data to buffer");
                    return Ok(());
                }
            };

            let objects_tree = ptree_read_xml(&buffer).ok_or_else(|| Bbs3mfException("layer_config_ranges.xml: xml parser error".into()))?;

            let objects = objects_tree.get_child("objects").ok_or_else(|| Bbs3mfException("No such node (objects)".into()))?;
            for (_, object_tree) in &objects.children {
                let obj_idx = object_tree.get_or::<i32>("<xmlattr>.id", -1);
                if obj_idx <= 0 {
                    self.add_error("Found invalid object id");
                    continue;
                }

                if self.m_layer_config_ranges.contains_key(&obj_idx) {
                    self.add_error("Found duplicated layer config range");
                    continue;
                }

                let mut config_ranges = crate::model::TLayerConfigRanges::new();

                for (key, range_tree) in &object_tree.children {
                    if key != "range" {
                        continue;
                    }
                    let bad = |p: &str| Bbs3mfException(format!("No such node ({})", p));
                    let min_z = range_tree.get_value::<f64>("<xmlattr>.min_z").ok_or_else(|| bad("<xmlattr>.min_z"))?;
                    let max_z = range_tree.get_value::<f64>("<xmlattr>.max_z").ok_or_else(|| bad("<xmlattr>.max_z"))?;

                    // get Z range information
                    let mut config = DynamicPrintConfig::new();

                    for (okey, option) in &range_tree.children {
                        if okey != "option" {
                            continue;
                        }
                        let opt_key = option.get_string("<xmlattr>.opt_key").ok_or_else(|| bad("<xmlattr>.opt_key"))?;
                        let value = option.data.clone();

                        use crate::config::ConfigBase;
                        config.set_deserialize(&opt_key, &value, config_substitutions, false)?;
                    }

                    config_ranges
                        .entry((ordered_float::OrderedFloat(min_z), ordered_float::OrderedFloat(max_z)))
                        .or_default()
                        .assign_config_data(&config);
                }

                if !config_ranges.is_empty() {
                    self.m_layer_config_ranges.insert(obj_idx, config_ranges);
                }
            }
        }
        Ok(())
    }

    /// bbs_3mf.cpp:3448-3524
    fn _extract_brim_ear_points_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat) -> Result<(), Bbs3mfException> {
        if stat.m_uncomp_size > 0 {
            let buffer = match mz_zip_reader_extract_to_mem(archive, stat.m_file_index) {
                Some(b) => b,
                None => {
                    self.add_error("Error while reading brim ear points data to buffer");
                    return Ok(());
                }
            };
            let mut buffer = String::from_utf8_lossy(&buffer).into_owned();

            if buffer.ends_with('\n') {
                buffer.pop();
            }

            let mut objects: Vec<String> = buffer.split('\n').map(|s| s.to_string()).collect();

            // Info on format versioning - see bbs_3mf.hpp
            let mut version = 0;
            let key = "brim_points_format_version=";
            if !objects.is_empty() && objects[0].contains(key) {
                objects[0] = objects[0][key.len()..].to_string(); // removes the string
                version = std_stoi(&objects[0]).map_err(Bbs3mfException)?;
                objects.remove(0); // pop the header
            }

            for object in &objects {
                let object_data: Vec<&str> = object.split('|').collect();

                if object_data.len() != 2 {
                    self.add_error("Error while reading object data");
                    continue;
                }

                let object_data_id: Vec<&str> = object_data[0].split('=').collect();
                if object_data_id.len() != 2 {
                    self.add_error("Error while reading object id");
                    continue;
                }

                let object_id = atoi(object_data_id[1]);
                if object_id == 0 {
                    self.add_error("Found invalid object id");
                    continue;
                }

                if self.m_brim_ear_points.contains_key(&object_id) {
                    self.add_error("Found duplicated brim ear points");
                    continue;
                }

                let object_data_points: Vec<&str> = object_data[1].split(' ').collect();

                let mut brim_ear_points: Vec<crate::brim_ears_point::BrimPoint> = Vec::new();
                let at = |i: usize| -> f64 { object_data_points.get(i).map(|s| atof(s)).unwrap_or(0.0) };
                if version == 0 {
                    let mut i = 0;
                    while i < object_data_points.len() {
                        brim_ear_points.push(crate::brim_ears_point::BrimPoint::from_coords(
                            at(i) as f32,
                            at(i + 1) as f32,
                            at(i + 2) as f32,
                            at(i + 3) as f32,
                            -1,
                        ));
                        i += 4;
                    }
                } else if version == 1 {
                    let mut i = 0;
                    while i < object_data_points.len() {
                        brim_ear_points.push(crate::brim_ears_point::BrimPoint::from_coords(
                            at(i) as f32,
                            at(i + 1) as f32,
                            at(i + 2) as f32,
                            at(i + 3) as f32,
                            at(i + 4) as i32,
                        ));
                        i += 5;
                    }
                }

                if !brim_ear_points.is_empty() {
                    self.m_brim_ear_points.insert(object_id, brim_ear_points);
                }
            }
        }
        Ok(())
    }

    /// bbs_3mf.cpp:3693-3716
    fn _extract_embossed_svg_shape_file(&mut self, filename: &str, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat, _model: &mut Model) {
        let file = match mz_zip_reader_extract_to_mem(archive, stat.m_file_index) {
            Some(f) => String::from_utf8_lossy(&f).into_owned(),
            None => {
                self.add_error("Error while reading svg shape for emboss");
                return;
            }
        };

        // store for case svg is loaded before volume
        self.m_path_to_emboss_shape_files.insert(filename.to_string(), file);

        // find embossed volume, for case svg is loaded after volume
        // FIDELITY-NOTE: `ModelVolume::emboss_shape` of the crate keeps whether the shape is set,
        // not the shape; the svg data stays with the importer (`m_path_to_emboss_shape_files`).
    }

    /// bbs_3mf.cpp:3718-3767
    fn _extract_filament_sequence_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat) {
        if stat.m_uncomp_size == 0 {
            self.add_error("Error while reading layer config ranges data to buffer");
            return;
        }

        let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
            Some(b) => b,
            None => {
                self.add_error("Error while reading filament sequence data to buffer");
                return;
            }
        };

        let parse = || -> Result<Vec<(i32, Vec<u32>, Vec<u32>, Vec<i32>)>, String> {
            let j: serde_json::Value = serde_json::from_slice(&buffer).map_err(|e| e.to_string())?;
            let mut out = Vec::new();
            for (&plate_idx, _) in self.m_plater_data.iter() {
                let key = format!("plate_{}", plate_idx);
                let jk = j.get(&key).cloned().unwrap_or(serde_json::Value::Null);
                let mut filament_key = "filament_sequence";
                if jk.get(filament_key).is_none() {
                    filament_key = "sequence";
                }
                let as_u32 = |v: &serde_json::Value| -> Result<u32, String> {
                    v.as_u64().map(|x| x as u32).ok_or_else(|| "[json.exception.type_error.302] type must be number".to_string())
                };
                let mut filament_sequence: Vec<u32> = Vec::new();
                let mut nozzle_sequence: Vec<u32> = Vec::new();
                let mut optimal_assignment: Vec<i32> = Vec::new();
                if let Some(arr) = jk.get(filament_key) {
                    for item in iter_json(arr) {
                        filament_sequence.push(as_u32(item)?.wrapping_sub(1)); // data stored in file is 1 based, change to 0 based when loading
                    }
                }
                if let Some(arr) = jk.get("nozzle_sequence") {
                    for item in iter_json(arr) {
                        nozzle_sequence.push(as_u32(item)?);
                    }
                }
                if let Some(arr) = jk.get("optimal_assignment") {
                    for item in iter_json(arr) {
                        optimal_assignment.push(item.as_i64().ok_or_else(|| "[json.exception.type_error.302] type must be number".to_string())? as i32);
                    }
                }
                out.push((plate_idx, filament_sequence, nozzle_sequence, optimal_assignment));
            }
            Ok(out)
        };
        match parse() {
            Ok(items) => {
                for (plate_idx, filament_sequence, nozzle_sequence, optimal_assignment) in items {
                    let plater_data = self.m_plater_data.get_mut(&plate_idx).unwrap();
                    plater_data.filament_change_sequence = filament_sequence;
                    plater_data.nozzle_change_sequence = nozzle_sequence;
                    if !optimal_assignment.is_empty() {
                        plater_data.optimal_assignment = optimal_assignment;
                    }
                }
            }
            Err(e) => {
                self.add_error(format!("Error while parsing filament sequence JSON: {}", e));
            }
        }
    }

    /// bbs_3mf.cpp:3770-3785
    fn _extract_assembly_tree_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat, model: &mut Model) {
        if stat.m_uncomp_size == 0 {
            self.add_error("Error while reading assembly tree data to buffer");
            return;
        }

        let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
            Some(b) => b,
            None => {
                self.add_error("Error while reading assembly tree data to buffer");
                return;
            }
        };

        model.set_assembly_tree_json_str(String::from_utf8_lossy(&buffer).into_owned());
    }

    /// bbs_3mf.cpp:3787-3803
    fn _extract_assembly_steps_json_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat, model: &mut Model) {
        if stat.m_uncomp_size == 0 {
            return;
        }
        let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
            Some(b) => String::from_utf8_lossy(&b).into_owned(),
            None => {
                self.add_error("Error while reading assembly.json from archive");
                return;
            }
        };
        // NOTE: do NOT deserialize into m_assembly_steps_tree_data here. This handler is invoked from the zip iteration loop in _load_model_from_file, which runs *before* the "assemble objects"
        // Remember version and qualified_assembly, then strip the flag so a reload
        // does not look like an assembly-guide edit.
        model.note_loaded_assembly_steps_meta(crate::format::assembly_steps_json::assembly_steps_json_file_meta(&buffer));
        let buffer = crate::format::assembly_steps_json::assembly_steps_json_with_qualified_assembly(&buffer, false);
        model.set_assembly_steps_json_str(buffer);
    }

    /// bbs_3mf.cpp:3805-3818
    fn _extract_assembly_model_from_archive(&mut self, archive: &mut MzZipArchive, stat: &MzZipArchiveFileStat, model: &mut Model) {
        if stat.m_uncomp_size == 0 {
            return;
        }
        let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
            Some(b) => b,
            None => {
                self.add_error("Error while reading assembly_model.json from archive");
                return;
            }
        };
        // Keep only the raw JSON here. The independent assembly model is rebuilt lazily on first entry to
        // the assembly view (meshes are rebound from the prepare model by part_guid), see Plater.
        model.set_assembly_model_json_str(String::from_utf8_lossy(&buffer).into_owned());
    }

    /// bbs_3mf.cpp:3821-3905
    fn _extract_custom_gcode_per_print_z_from_archive(
        &mut self,
        archive: &mut MzZipArchive,
        stat: &MzZipArchiveFileStat,
        model: &mut Model,
    ) -> Result<(), Bbs3mfException> {
        use crate::custom_g_code::{Item, Mode, Type, MULTI_AS_SINGLE_MODE, SINGLE_EXTRUDER_MODE};
        //BBS: add plate tree related logic
        if stat.m_uncomp_size > 0 {
            let buffer = match mz_zip_reader_extract_file_to_mem(archive, &stat.m_filename) {
                Some(b) => b,
                None => {
                    self.add_error("Error while reading custom Gcodes per height data to buffer");
                    return Ok(());
                }
            };

            let main_tree = ptree_read_xml(&buffer).ok_or_else(|| Bbs3mfException("custom_gcode_per_layer.xml: xml parser error".into()))?;

            let front = match main_tree.children.first() {
                Some(f) => f,
                None => return Ok(()),
            };
            if front.0 != "custom_gcodes_per_layer" {
                return Ok(());
            }

            let bad = |p: &str| Bbs3mfException(format!("No such node ({})", p));
            let extract_code = |model: &mut Model, plate_id: i32, code_tree: &PTree| -> Result<(), Bbs3mfException> {
                for (key, tree) in &code_tree.children {
                    if key == "mode" {
                        let mode = tree.get_string("<xmlattr>.value").ok_or_else(|| bad("<xmlattr>.value"))?;
                        model.plates_custom_gcodes.entry(plate_id - 1).or_insert_with(crate::custom_g_code::Info::new).mode =
                            if mode == SINGLE_EXTRUDER_MODE {
                                Mode::SingleExtruder
                            } else if mode == MULTI_AS_SINGLE_MODE {
                                Mode::MultiAsSingle
                            } else {
                                Mode::MultiExtruder
                            };
                    }
                    if key == "layer" {
                        let print_z = tree.get_value::<f64>("<xmlattr>.top_z").ok_or_else(|| bad("<xmlattr>.top_z"))?;
                        let extruder = tree.get_value::<i32>("<xmlattr>.extruder").ok_or_else(|| bad("<xmlattr>.extruder"))?;
                        let color = tree.get_string("<xmlattr>.color").ok_or_else(|| bad("<xmlattr>.color"))?;

                        let type_;
                        let extra;
                        let attr_tree = tree.find("<xmlattr>").ok_or_else(|| bad("<xmlattr>"))?;
                        if attr_tree.find("type").is_none() {
                            // It means that data was saved in old version (2.2.0 and older) of PrusaSlicer
                            // read old data ...
                            let gcode = tree.get_string("<xmlattr>.gcode").ok_or_else(|| bad("<xmlattr>.gcode"))?;
                            // ... and interpret them to the new data
                            type_ = if gcode == "M600" {
                                Type::ColorChange
                            } else if gcode == "M601" {
                                Type::PausePrint
                            } else if gcode == "tool_change" {
                                Type::ToolChange
                            } else {
                                Type::Custom
                            };
                            extra = if type_ == Type::PausePrint {
                                color.clone()
                            } else if type_ == Type::Custom {
                                gcode
                            } else {
                                String::new()
                            };
                        } else {
                            let t = tree.get_value::<i32>("<xmlattr>.type").ok_or_else(|| bad("<xmlattr>.type"))?;
                            type_ = custom_gcode_type_from_int(t);
                            extra = tree.get_string("<xmlattr>.extra").ok_or_else(|| bad("<xmlattr>.extra"))?;
                        }
                        model
                            .plates_custom_gcodes
                            .entry(plate_id - 1)
                            .or_insert_with(crate::custom_g_code::Info::new)
                            .gcodes
                            .push(Item::new_with_extra(print_z, type_, extruder, color, extra));
                    }
                }
                Ok(())
            };

            model.plates_custom_gcodes.clear();

            let mut has_plate_info = false;
            for (key, element) in &front.1.children {
                if key == "plate" {
                    has_plate_info = true;

                    let mut plate_id = -1;
                    let code_tree = element;
                    for (ckey, code) in &code_tree.children {
                        if ckey == "plate_info" {
                            plate_id = code.get_value::<i32>("<xmlattr>.id").ok_or_else(|| bad("<xmlattr>.id"))?;
                        }
                    }
                    if plate_id == -1 {
                        continue;
                    }

                    extract_code(model, plate_id, code_tree)?;
                }
            }

            if !has_plate_info {
                let plate_id = 1;
                let code_tree = &front.1;
                extract_code(model, plate_id, code_tree)?;
            }
        }
        Ok(())
    }

    /// bbs_3mf.cpp:3907-3950
    fn _handle_start_model_xml_element(&mut self, model: Option<&mut Model>, name: &str, attributes: &[(String, String)]) {
        if !self.m_xml_parser {
            return;
        }

        let mut res = true;

        self.m_material_colors.start_element(name, attributes, &mut self.m_group_id_to_color);

        if MODEL_TAG == name {
            res = self._handle_start_model(attributes);
        } else if RESOURCES_TAG == name {
            res = self._handle_start_resources(attributes);
        } else if OBJECT_TAG == name {
            res = self._handle_start_object(attributes);
        } else if COLOR_GROUP_TAG == name {
            res = self._handle_start_color_group(attributes);
        } else if COLOR_TAG == name {
            res = self._handle_start_color(attributes);
        } else if MESH_TAG == name {
            res = self._handle_start_mesh(attributes);
        } else if VERTICES_TAG == name {
            res = self._handle_start_vertices(attributes);
        } else if VERTEX_TAG == name {
            res = self._handle_start_vertex(attributes);
        } else if TRIANGLES_TAG == name {
            res = self._handle_start_triangles(attributes);
        } else if TRIANGLE_TAG == name {
            res = self._handle_start_triangle(attributes);
        } else if COMPONENTS_TAG == name {
            res = self._handle_start_components(attributes);
        } else if COMPONENT_TAG == name {
            res = self._handle_start_component(attributes);
        } else if BUILD_TAG == name {
            res = self._handle_start_build(attributes);
        } else if ITEM_TAG == name {
            res = match model {
                Some(m) => self._handle_start_item(m, attributes),
                None => true,
            };
        } else if METADATA_TAG == name {
            res = self._handle_start_metadata(attributes);
        }

        if !res {
            self._stop_xml_parser("");
        }
    }

    /// bbs_3mf.cpp:3952-3994
    fn _handle_end_model_xml_element(&mut self, model: Option<&mut Model>, name: &str) {
        if !self.m_xml_parser {
            return;
        }

        let mut res = true;

        self.m_material_colors.end_element(name);

        if MODEL_TAG == name {
            res = match model {
                Some(m) => self._handle_end_model(m),
                None => true,
            };
        } else if RESOURCES_TAG == name {
            res = self._handle_end_resources();
        } else if OBJECT_TAG == name {
            res = self._handle_end_object();
        } else if COLOR_GROUP_TAG == name {
            res = self._handle_end_color_group();
        } else if COLOR_TAG == name {
            res = self._handle_end_color();
        } else if MESH_TAG == name {
            res = self._handle_end_mesh();
        } else if VERTICES_TAG == name {
            res = self._handle_end_vertices();
        } else if VERTEX_TAG == name {
            res = self._handle_end_vertex();
        } else if TRIANGLES_TAG == name {
            res = self._handle_end_triangles();
        } else if TRIANGLE_TAG == name {
            res = self._handle_end_triangle();
        } else if COMPONENTS_TAG == name {
            res = self._handle_end_components();
        } else if COMPONENT_TAG == name {
            res = self._handle_end_component();
        } else if BUILD_TAG == name {
            res = self._handle_end_build();
        } else if ITEM_TAG == name {
            res = self._handle_end_item();
        } else if METADATA_TAG == name {
            res = self._handle_end_metadata();
        }

        if !res {
            self._stop_xml_parser("");
        }
    }

    /// bbs_3mf.cpp:3996-3999
    fn _handle_xml_characters(&mut self, s: &str) {
        self.m_curr_characters.push_str(s);
    }

    /// bbs_3mf.cpp:4001-4049
    fn _handle_start_config_xml_element(&mut self, model: Option<&mut Model>, name: &str, attributes: &[(String, String)]) {
        if !self.m_xml_parser {
            return;
        }

        let mut res = true;

        if CONFIG_TAG == name {
            res = self._handle_start_config(attributes);
        } else if OBJECT_TAG == name {
            res = self._handle_start_config_object(attributes);
        } else if VOLUME_TAG == name {
            res = self._handle_start_config_volume(attributes);
        } else if PART_TAG == name {
            res = self._handle_start_config_volume(attributes);
        } else if MESH_STAT_TAG == name {
            res = self._handle_start_config_volume_mesh(attributes);
        } else if METADATA_TAG == name {
            res = self._handle_start_config_metadata(attributes);
        } else if PLATE_TAG == name {
            res = self._handle_start_config_plater(attributes);
        } else if INSTANCE_TAG == name {
            res = self._handle_start_config_plater_instance(attributes);
        } else if FILAMENT_TAG == name {
            res = self._handle_start_config_filament(attributes);
        } else if PAUSE_TAG == name {
            res = self._handle_start_config_pause(attributes);
        } else if MIXED_FILAMENT_TAG == name {
            res = self._handle_start_config_mixed_filament(attributes);
        } else if SLICE_WARNING_TAG == name {
            res = self._handle_start_config_warning(attributes);
        } else if NOZZLE_TAG == name {
            res = self._handle_start_config_nozzle(attributes);
        } else if AMS_ITEM_TAG == name {
            res = self._handle_start_config_ams_item(attributes);
        } else if ASSEMBLE_TAG == name {
            res = self._handle_start_assemble(attributes);
        } else if ASSEMBLE_ITEM_TAG == name {
            res = match model {
                Some(m) => self._handle_start_assemble_item(m, attributes),
                None => true,
            };
        } else if TEXT_INFO_TAG == name {
            res = self._handle_start_text_info_item(attributes);
        } else if OLD_SHAPE_TAG == name {
            res = self._handle_start_shape_configuration(attributes);
        } else if SHAPE_TAG == name {
            res = self._handle_start_shape_configuration(attributes);
        }
        if !res {
            self._stop_xml_parser("");
        }
    }

    /// bbs_3mf.cpp:4051-4085
    fn _handle_end_config_xml_element(&mut self, name: &str) {
        if !self.m_xml_parser {
            return;
        }

        let mut res = true;

        if CONFIG_TAG == name {
            res = self._handle_end_config();
        } else if OBJECT_TAG == name {
            res = self._handle_end_config_object();
        } else if VOLUME_TAG == name {
            res = self._handle_end_config_volume();
        } else if PART_TAG == name {
            res = self._handle_end_config_volume();
        } else if MESH_STAT_TAG == name {
            res = self._handle_end_config_volume_mesh();
        } else if METADATA_TAG == name {
            res = self._handle_end_config_metadata();
        } else if PLATE_TAG == name {
            res = self._handle_end_config_plater();
        } else if FILAMENT_TAG == name {
            res = self._handle_end_config_filament();
        } else if NOZZLE_TAG == name {
            res = self._handle_end_config_nozzle();
        } else if INSTANCE_TAG == name {
            res = self._handle_end_config_plater_instance();
        } else if ASSEMBLE_TAG == name {
            res = self._handle_end_assemble();
        } else if ASSEMBLE_ITEM_TAG == name {
            res = self._handle_end_assemble_item();
        }

        if !res {
            self._stop_xml_parser("");
        }
    }

    /// bbs_3mf.cpp:4087-4094
    fn _handle_start_model(&mut self, attributes: &[(String, String)]) -> bool {
        self.m_unit_factor = bbs_get_unit_factor(&bbs_get_attribute_value_string(attributes, UNIT_ATTR));
        if self.m_sub_model_path.is_empty() {
            self.m_has_unit = !bbs_get_attribute_value_string(attributes, UNIT_ATTR).is_empty();
        }

        true
    }

    /// bbs_3mf.cpp:4096-4137
    fn _handle_end_model(&mut self, model: &mut Model) -> bool {
        // BBS: Production Extension
        if !self.m_sub_model_path.is_empty() {
            return true;
        }

        // The sub-model files are merged before the root model is read, so every material
        // resource is known now.
        self.m_material_colors.resolve(&mut self.m_group_id_to_color);

        // deletes all non-built or non-instanced objects
        let objects: Vec<i32> = self.m_objects.values().cloned().collect();
        for object_second in objects {
            if object_second >= model.objects.len() as i32 {
                self.add_error("Unable to find object");
                return false;
            }
            if model.objects[object_second as usize].instances.is_empty() {
                // `m_model->delete_object(model_object)`: the instances of the objects behind it
                // keep pointing at their objects
                let idx = object_second as usize;
                model.delete_object(idx);
                for inst in self.m_instances.iter_mut() {
                    if inst.object_idx > idx {
                        inst.object_idx -= 1;
                    }
                }
            }
        }

        //construct the index maps
        for (k, _) in &self.m_current_objects {
            self.m_index_paths.entry(k.1).or_insert_with(|| k.0.clone());
        }

        if !self.m_is_bbl_3mf {
            // if the 3mf was not produced by BambuStudio and there is only one object,
            // set the object name to match the filename
            if model.objects.len() == 1 {
                model.objects[0].name = self.m_name.clone();
            }
        }

        // applies instances' matrices
        for instance in &self.m_instances {
            if instance.object_idx < model.objects.len() && instance.instance_idx < model.objects[instance.object_idx].instances.len() {
                // apply the transform to the instance
                Self::_apply_transform(&mut model.objects[instance.object_idx].instances[instance.instance_idx], &instance.transform);
            }
        }

        true
    }

    /// bbs_3mf.cpp:4139-4143
    fn _handle_start_resources(&mut self, _attributes: &[(String, String)]) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4145-4149
    fn _handle_end_resources(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4151-4190
    fn _handle_start_object(&mut self, attributes: &[(String, String)]) -> bool {
        // reset current object data
        self.m_curr_object = None;

        let object_type = bbs_get_attribute_value_string(attributes, TYPE_ATTR);

        if bbs_is_valid_object_type(&object_type) {
            if self.m_curr_object.is_none() {
                self.m_curr_object = Some(Box::new(CurrentObject::new()));
                // create new object (it may be removed later if no instances are generated from it)
            }
            let o = self.m_curr_object.as_mut().unwrap();

            o.id = bbs_get_attribute_value_int(attributes, ID_ATTR);
            o.name = bbs_get_attribute_value_string(attributes, NAME_ATTR);

            o.uuid = bbs_get_attribute_value_string(attributes, PUUID_ATTR);
            if o.uuid.is_empty() {
                o.uuid = bbs_get_attribute_value_string(attributes, PUUID_LOWER_ATTR);
            }

            if bbs_has_attribute_value_int(attributes, PID_ATTR) {
                o.pid = bbs_get_attribute_value_int(attributes, PID_ATTR);
            }
            if bbs_has_attribute_value_int(attributes, PINDEX_ATTR) {
                o.pindex = bbs_get_attribute_value_int(attributes, PINDEX_ATTR);
            }
        }

        true
    }

    /// bbs_3mf.cpp:4192-4299
    fn _handle_end_object(&mut self) -> bool {
        if !self.m_load_model {
            self.m_curr_object = None;
            return true;
        }
        let mut curr = match self.m_curr_object.take() {
            Some(o) if o.id != -1 => o,
            _ => {
                self.add_error("Found invalid object");
                return false;
            }
        };
        if self.m_is_bbl_3mf && curr.uuid.ends_with(OBJECT_UUID_SUFFIX) && self.m_load_restore {
            // Adjust backup object/volume id
            let mut backup_id = 0;
            let mut _need_replace = false;
            if let Some(v) = istream_hex_int(&curr.uuid) {
                backup_id = v;
                _need_replace = curr.id != backup_id;
                curr.id = backup_id;
            }
            if !curr.components.is_empty() {
                let mut first_id = curr.components[0].object_id.clone();
                first_id.1 = 0;
                // m_current_objects.lower_bound(first_id)
                let mut current_keys: Vec<Id> = self.m_current_objects.range(first_id..).map(|(k, _)| k.clone()).collect();
                current_keys.reverse();
                let mut new_map: BTreeMap<Id, CurrentObject> = BTreeMap::new();
                for index in 0..curr.components.len() {
                    let component = &mut curr.components[index];
                    let mut new_id = component.object_id.clone();
                    new_id.1 = ((index as i32 + 1) << 16) | backup_id;
                    let current_key = current_keys.last().cloned();
                    match current_key {
                        Some(ck) if new_id.0.is_empty() || new_id.0 == ck.0 => {
                            let mut obj = self.m_current_objects.remove(&ck).unwrap();
                            current_keys.pop();
                            obj.id = new_id.1;
                            new_map.insert(new_id.clone(), obj);
                        }
                        _ => {
                            self.add_error(format!("can not find object for component, id={}", component.object_id.1));
                            return false;
                        }
                    }
                    component.object_id.1 = new_id.1;
                }
                for (k, v) in new_map {
                    self.m_current_objects.entry(k).or_insert(v);
                }
            }
        }

        let id: Id = (self.m_sub_model_path.clone(), curr.id);
        if !self.m_current_objects.contains_key(&id) {
            self.m_current_objects.insert(id, *curr);
        } else {
            self.add_error("Found object with duplicate id");
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:4301-4305
    fn _handle_start_color_group(&mut self, attributes: &[(String, String)]) -> bool {
        self.m_current_color_group = bbs_get_attribute_value_int(attributes, ID_ATTR);
        true
    }

    /// bbs_3mf.cpp:4307-4311
    fn _handle_end_color_group(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4313-4319
    fn _handle_start_color(&mut self, attributes: &[(String, String)]) -> bool {
        let color = bbs_get_attribute_value_string(attributes, COLOR_ATTR);
        let colors = self.m_group_id_to_color.entry(self.m_current_color_group).or_default();
        colors.push(color);
        true
    }

    /// bbs_3mf.cpp:4321-4325
    fn _handle_end_color(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4327-4333
    fn _handle_start_mesh(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current geometry
        if let Some(o) = self.m_curr_object.as_mut() {
            o.geometry.reset();
        }
        true
    }

    /// bbs_3mf.cpp:4335-4339
    fn _handle_end_mesh(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4341-4347
    fn _handle_start_vertices(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current vertices
        if let Some(o) = self.m_curr_object.as_mut() {
            o.geometry.vertices.clear();
        }
        true
    }

    /// bbs_3mf.cpp:4349-4353
    fn _handle_end_vertices(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4355-4365
    fn _handle_start_vertex(&mut self, attributes: &[(String, String)]) -> bool {
        // appends the vertex coordinates
        // missing values are set equal to ZERO
        let f = self.m_unit_factor;
        if let Some(o) = self.m_curr_object.as_mut() {
            o.geometry.vertices.push(Vec3f::new(
                f * bbs_get_attribute_value_float(attributes, X_ATTR),
                f * bbs_get_attribute_value_float(attributes, Y_ATTR),
                f * bbs_get_attribute_value_float(attributes, Z_ATTR),
            ));
        }
        true
    }

    /// bbs_3mf.cpp:4367-4371
    fn _handle_end_vertex(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4373-4381
    fn _handle_start_triangles(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current triangles
        if let Some(o) = self.m_curr_object.as_mut() {
            o.geometry.triangles.clear();
            o.geometry.triangle_colors.clear();
        }
        true
    }

    /// bbs_3mf.cpp:4383-4387
    fn _handle_end_triangles(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4407-4459
    fn _handle_start_triangle(&mut self, attributes: &[(String, String)]) -> bool {
        // we are ignoring the following attributes:
        // p1
        // p2
        // p3
        // pid
        // see specifications

        // appends the triangle's vertices indices
        // missing values are set equal to ZERO
        let is_bbl_3mf = self.m_is_bbl_3mf;
        if let Some(current_object) = self.m_curr_object.as_mut() {
            current_object.geometry.triangles.push(crate::triangle_mesh::Vec3i::new(
                bbs_get_attribute_value_int(attributes, V1_ATTR),
                bbs_get_attribute_value_int(attributes, V2_ATTR),
                bbs_get_attribute_value_int(attributes, V3_ATTR),
            ));

            current_object.geometry.custom_supports.push(bbs_get_attribute_value_string(attributes, CUSTOM_SUPPORTS_ATTR));
            current_object.geometry.custom_fuzzy_skin.push(bbs_get_attribute_value_string(attributes, CUSTOM_FUZZY_SKIN_ATTR));
            current_object.geometry.custom_seam.push(bbs_get_attribute_value_string(attributes, CUSTOM_SEAM_ATTR));
            current_object.geometry.mmu_segmentation.push(bbs_get_attribute_value_string(attributes, MMU_SEGMENTATION_ATTR));
            // BBS
            current_object.geometry.face_properties.push(bbs_get_attribute_value_string(attributes, FACE_PROPERTY_ATTR));
            if !is_bbl_3mf {
                let mut color_info = crate::format::obj::TriangleColor::new();
                if bbs_has_attribute_value_int(attributes, PID_ATTR) {
                    color_info.pid = bbs_get_attribute_value_int(attributes, PID_ATTR);
                } else {
                    color_info.pid = current_object.pid;
                }
                if color_info.pid >= 0 {
                    let color_attrs = [P1_ATTR, P2_ATTR, P3_ATTR];
                    for idx in 0..3 {
                        if bbs_has_attribute_value_int(attributes, color_attrs[idx]) {
                            color_info.indices[idx] = bbs_get_attribute_value_int(attributes, color_attrs[idx]);
                        } else if current_object.pindex >= 0 {
                            if idx == 0 {
                                color_info.indices[idx] = current_object.pindex;
                            } else {
                                color_info.indices[idx] = color_info.indices[0];
                            }
                        }
                    }
                    modify_import_color(&mut color_info.indices);
                    current_object.geometry.triangle_colors.push(color_info);
                }
            }
        }
        true
    }

    /// bbs_3mf.cpp:4461-4465
    fn _handle_end_triangle(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4467-4473
    fn _handle_start_components(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current components
        if let Some(o) = self.m_curr_object.as_mut() {
            o.components.clear();
        }
        true
    }

    /// bbs_3mf.cpp:4475-4479
    fn _handle_end_components(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4481-4503
    fn _handle_start_component(&mut self, attributes: &[(String, String)]) -> bool {
        let path = crate::utils::xml_unescape(&bbs_get_attribute_value_string(attributes, PPATH_ATTR));
        let object_id = bbs_get_attribute_value_int(attributes, OBJECTID_ATTR);
        let transform = bbs_get_transform_from_3mf_specs_string(&bbs_get_attribute_value_string(attributes, TRANSFORM_ATTR));

        let sub = self.m_sub_model_path.clone();
        if let Some(o) = self.m_curr_object.as_mut() {
            let id: Id = (if sub.is_empty() { path } else { sub }, object_id);
            o.components.push(Component::new(id, transform));
        }

        true
    }

    /// bbs_3mf.cpp:4505-4509
    fn _handle_end_component(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4511-4515
    fn _handle_start_build(&mut self, _attributes: &[(String, String)]) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4517-4521
    fn _handle_end_build(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4523-4538
    fn _handle_start_item(&mut self, model: &mut Model, attributes: &[(String, String)]) -> bool {
        // we are ignoring the following attributes
        // thumbnail
        // partnumber
        // pid
        // pindex
        // see specifications

        let object_id = bbs_get_attribute_value_int(attributes, OBJECTID_ATTR);
        let path = bbs_get_attribute_value_string(attributes, PPATH_ATTR);
        let transform = bbs_get_transform_from_3mf_specs_string(&bbs_get_attribute_value_string(attributes, TRANSFORM_ATTR));
        let printable = bbs_get_attribute_value_bool(attributes, PRINTABLE_ATTR);

        !self.m_load_model || self._create_object_instance(model, &path, object_id, &transform, printable, 1)
    }

    /// bbs_3mf.cpp:4540-4544
    fn _handle_end_item(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4546-4556
    fn _handle_start_metadata(&mut self, attributes: &[(String, String)]) -> bool {
        self.m_curr_characters.clear();

        let name = bbs_get_attribute_value_string(attributes, NAME_ATTR);
        if !name.is_empty() {
            self.m_curr_metadata_name = name;
        }

        true
    }

    /// bbs_3mf.cpp:4564-4660
    fn _handle_end_metadata(&mut self) -> bool {
        use crate::utils::xml_unescape;
        let name = self.m_curr_metadata_name.clone();
        let chars = self.m_curr_characters.clone();
        if name == BBS_3MF_VERSION || name == BBS_3MF_VERSION1 {
            //m_is_bbl_3mf = true;
            self.m_version = atoi(&chars) as u32;
        } else if name == BBL_APPLICATION_TAG {
            // Generator application of the 3MF.
            // SLIC3R_APP_KEY - SLIC3R_VERSION
            if chars.starts_with("BambuStudio-") {
                self.m_is_bbl_3mf = true;
                self.m_bambuslicer_generator_version = crate::semver::Semver::parse(&chars[12..]);
            }
        //TODO: currently use version 0, no need to load&&save this string
        } else if name == BBL_MODEL_ID_TAG {
            self.m_model_id = xml_unescape(&chars);
        } else if name == BBL_MODEL_NAME_TAG {
            log::trace!("design_info, load_3mf found model name = {}", chars);
            self.model_info.model_name = xml_unescape(&chars);
        } else if name == BBL_ORIGIN_TAG {
            log::trace!("design_info, load_3mf found model name = {}", chars);
            self.model_info.origin = xml_unescape(&chars);
        } else if name == BBL_DESIGNER_TAG {
            log::trace!("design_info, load_3mf found designer = {}", chars);
            self.m_designer = xml_unescape(&chars);
        } else if name == BBL_DESIGNER_USER_ID_TAG {
            log::trace!("design_info, load_3mf found designer_user_id = {}", chars);
            self.m_designer_user_id = xml_unescape(&chars);
        } else if name == BBL_DESIGNER_MODEL_ID_TAG {
            log::trace!("design_info, load_3mf found designer_model_id = {}", chars);
            self.m_designer_id = xml_unescape(&chars);
        } else if name == BBL_DESIGN_ID_TAG {
            log::trace!("design_info, load_3mf found design_id = {}", chars);
            self.m_design_id = xml_unescape(&chars);
        } else if name == BBL_DESIGNER_COVER_FILE_TAG {
            log::trace!("design_info, load_3mf found designer_cover = {}", chars);
            self.model_info.cover_file = xml_unescape(&chars);
        } else if name == BBL_DESCRIPTION_TAG {
            log::trace!("design_info, load_3mf found description = {}", chars);
            self.model_info.description = xml_unescape(&chars);
        } else if name == BBL_LICENSE_TAG {
            log::trace!("design_info, load_3mf found license = {}", chars);
            self.model_info.license = xml_unescape(&chars);
        } else if name == BBL_COPYRIGHT_TAG {
            log::trace!("design_info, load_3mf found CopyRight = {}", chars);
            self.model_info.copyright = xml_unescape(&chars);
        } else if name == BBL_COPYRIGHT_NORMATIVE_TAG {
            log::trace!("design_info, load_3mf found Copyright = {}", chars);
            self.model_info.copyright = xml_unescape(&chars);
        } else if name == BBL_REGION_TAG {
            log::trace!("design_info, load_3mf found region = {}", chars);
            self.m_contry_code = xml_unescape(&chars);
        } else if name == BBL_PROFILE_TITLE_TAG {
            log::trace!("design_info, load_3mf found profile_title = {}", chars);
            self.m_profile_title = xml_unescape(&chars);
        } else if name == BBL_PROFILE_COVER_TAG {
            log::trace!("design_info, load_3mf found profile_cover = {}", chars);
            self.m_profile_cover = xml_unescape(&chars);
        } else if name == BBL_PROFILE_DESCRIPTION_TAG {
            log::trace!("design_info, load_3mf found profile_description = {}", chars);
            self.m_profile_description = xml_unescape(&chars);
        } else if name == BBL_PROFILE_USER_ID_TAG {
            log::trace!("design_info, load_3mf found profile_user_id = {}", chars);
            self.m_profile_user_id = xml_unescape(&chars);
        } else if name == BBL_PROFILE_USER_NAME_TAG {
            log::trace!("design_info, load_3mf found profile_user_name = {}", chars);
            self.m_profile_user_name = xml_unescape(&chars);
        } else if name == BBL_CREATION_DATE_TAG || name == BBL_MODIFICATION_TAG {
            // nothing
        }
        if !name.is_empty() {
            self.model_info.metadata_items.insert(name, xml_unescape(&chars));
        }

        true
    }

    /// bbs_3mf.cpp:4666-4694
    fn _handle_start_shape_configuration(&mut self, attributes: &[(String, String)]) -> bool {
        let object_id = self.m_curr_config.object_id;
        let object = match self.m_objects_metadata.get_mut(&object_id) {
            Some(o) => o,
            None => {
                self.add_error("Can not assign volume mesh to a valid object");
                return false;
            }
        };
        if object.volumes.is_empty() {
            self.add_error("Can not assign mesh to a valid volume");
            return false;
        }
        let volume = object.volumes.last_mut().unwrap();
        volume.shape_configuration = read_emboss_shape(attributes);
        let shape = match volume.shape_configuration.as_mut() {
            Some(s) => s,
            None => return false,
        };

        // Fill svg file content into shape_configuration
        let svg = match shape.svg_file.as_mut() {
            Some(s) => s,
            None => return true, // do not contain svg file
        };

        let path = svg.path_in_3mf.clone();
        if path.is_empty() {
            return true; // do not contain svg file
        }

        match self.m_path_to_emboss_shape_files.get(&path) {
            None => true, // svg file is not loaded yet
            Some(data) => {
                svg.file_data = Some(data.clone());
                true
            }
        }
    }

    /// bbs_3mf.cpp:4696-4797
    fn _create_object_instance(
        &mut self,
        model: &mut Model,
        path: &str,
        object_id: i32,
        transform: &Transform3d,
        printable: bool,
        recur_counter: u32,
    ) -> bool {
        const MAX_RECURSIONS: u32 = 10;

        // escape from circular aliasing
        if recur_counter > MAX_RECURSIONS {
            self.add_error("Too many recursions");
            return false;
        }

        let id: Id = (path.to_string(), object_id);
        if !self.m_current_objects.contains_key(&id) {
            self.add_error(format!("can not find object id {} to builditem", object_id));
            return false;
        }

        match self.m_objects.get(&id).cloned() {
            None => {
                //add object
                let object_index = model.objects.len() as i32;
                model.add_object_new();
                self.m_objects.insert(id.clone(), object_index);
                let is_bbl_3mf = self.m_is_bbl_3mf;
                let current_object = self.m_current_objects.get_mut(&id).unwrap();
                current_object.model_object_idx = object_index;
                current_object.object = Some(object_index as usize);
                let uuid = current_object.uuid.clone();

                let instance = model.objects[object_index as usize].add_instance();
                instance.printable = printable;
                let instance_idx = model.objects[object_index as usize].instances.len() - 1;

                self.m_instances.push(ImporterInstance { object_idx: object_index as usize, instance_idx, transform: *transform });

                if is_bbl_3mf && uuid.ends_with(OBJECT_UUID_SUFFIX) {
                    if let Some(backup_id) = istream_hex_int(&uuid) {
                        let objects = std::mem::take(&mut model.objects);
                        model.set_object_backup_id(&objects[object_index as usize], backup_id);
                        model.objects = objects;
                    }
                }
            }
            Some(object_second) => {
                //add instance
                let instance = model.objects[object_second as usize].add_instance();
                instance.printable = printable;
                let instance_idx = model.objects[object_second as usize].instances.len() - 1;

                self.m_instances.push(ImporterInstance { object_idx: object_second as usize, instance_idx, transform: *transform });
            }
        }

        true
    }

    /// bbs_3mf.cpp:4799-4807
    fn _apply_transform(instance: &mut ModelInstance, transform: &Transform3d) {
        let t = Transformation::from_transform(*transform);
        // invalid scale value, return
        let s = t.get_scaling_factor();
        if !(s.x != 0.0 && s.y != 0.0 && s.z != 0.0) {
            return;
        }

        instance.set_transformation(&t);
    }

    /// bbs_3mf.cpp:4809-4813
    fn _handle_start_config(&mut self, _attributes: &[(String, String)]) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4815-4819
    fn _handle_end_config(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4821-4838
    fn _handle_start_config_object(&mut self, attributes: &[(String, String)]) -> bool {
        if self.m_parsing_slice_info {
            return true;
        }
        let object_id = bbs_get_attribute_value_int(attributes, ID_ATTR);
        if self.m_objects_metadata.contains_key(&object_id) {
            self.add_error(format!("Duplicated object id: {} in model_settings.config", object_id));
            return false;
        }

        // Added because of github #3435, currently not used by PrusaSlicer
        // int instances_count_id = bbs_get_attribute_value_int(attributes, num_attributes, INSTANCESCOUNT_ATTR);

        self.m_objects_metadata.insert(object_id, ObjectMetadata::default());
        self.m_curr_config.object_id = object_id;
        true
    }

    /// bbs_3mf.cpp:4840-4844
    fn _handle_end_config_object(&mut self) -> bool {
        self.m_curr_config.object_id = -1;
        true
    }

    /// bbs_3mf.cpp:4846-4874
    fn _handle_start_config_volume(&mut self, attributes: &[(String, String)]) -> bool {
        let object_id = self.m_curr_config.object_id;
        let object = match self.m_objects_metadata.get_mut(&object_id) {
            Some(o) => o,
            None => {
                self.add_error(format!("can not find object for part, id {}", object_id));
                return false;
            }
        };

        self.m_curr_config.volume_id = object.volumes.len() as i32;

        let first_triangle_id = bbs_get_attribute_value_int(attributes, FIRST_TRIANGLE_ID_ATTR) as u32;
        let last_triangle_id = bbs_get_attribute_value_int(attributes, LAST_TRIANGLE_ID_ATTR) as u32;

        //BBS: refine the part type logic
        let subtype_str = bbs_get_attribute_value_string(attributes, SUBTYPE_ATTR);
        let type_ = ModelVolume::type_from_string(&subtype_str);
        let part_guid_str = bbs_get_attribute_value_string(attributes, PART_GUID_KEY);

        let subbject_id = bbs_get_attribute_value_int(attributes, ID_ATTR);

        if last_triangle_id > 0 {
            object.volumes.push(VolumeMetadata::with_triangles(first_triangle_id, last_triangle_id, type_));
        } else {
            object.volumes.push(VolumeMetadata::with_subobject(subbject_id, type_));
        }
        let volume_metadata = object.volumes.last_mut().unwrap();
        if !part_guid_str.is_empty() {
            volume_metadata.metadata.push(Metadata::new(PART_GUID_KEY, &part_guid_str));
        }
        true
    }

    /// bbs_3mf.cpp:4876-4899
    fn _handle_start_config_volume_mesh(&mut self, attributes: &[(String, String)]) -> bool {
        let object_id = self.m_curr_config.object_id;
        let volume_id = self.m_curr_config.volume_id;
        let object = match self.m_objects_metadata.get_mut(&object_id) {
            Some(o) => o,
            None => {
                self.add_error(format!("can not find object for mesh_stats, id {}", object_id));
                return false;
            }
        };
        if volume_id == -1 || (object.volumes.len() as i64 - 1) < volume_id as i64 {
            self.add_error("can not find part for mesh_stats");
            return false;
        }

        let volume = &mut object.volumes[volume_id as usize];

        let edges_fixed = bbs_get_attribute_value_int(attributes, MESH_STAT_EDGES_FIXED);
        let degenerate_facets = bbs_get_attribute_value_int(attributes, MESH_STAT_DEGENERATED_FACETS);
        let facets_removed = bbs_get_attribute_value_int(attributes, MESH_STAT_FACETS_REMOVED);
        let facets_reversed = bbs_get_attribute_value_int(attributes, MESH_STAT_FACETS_RESERVED);
        let backwards_edges = bbs_get_attribute_value_int(attributes, MESH_STAT_BACKWARDS_EDGES);

        volume.mesh_stats = crate::triangle_mesh::RepairedMeshErrors {
            edges_fixed,
            degenerate_facets,
            facets_removed,
            facets_reversed,
            backwards_edges,
        };

        true
    }

    /// bbs_3mf.cpp:4901-4905
    fn _handle_end_config_volume(&mut self) -> bool {
        self.m_curr_config.volume_id = -1;
        true
    }

    /// bbs_3mf.cpp:4907-4911
    fn _handle_end_config_volume_mesh(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:4913-5174
    fn _handle_start_config_metadata(&mut self, attributes: &[(String, String)]) -> bool {
        use crate::config::{ConfigOptionBool, ConfigOptionEnum, ConfigOptionInt, ConfigOptionInts};
        use crate::print_config::enums::{BedType, FilamentMapMode, PrintSequence};
        //std::string type = bbs_get_attribute_value_string(attributes, num_attributes, TYPE_ATTR);
        let key = bbs_get_attribute_value_string(attributes, KEY_ATTR);
        let value = bbs_get_attribute_value_string(attributes, VALUE_ATTR);
        if key.is_empty() {
            return true;
        }

        if self.m_curr_plater.is_none() && !self.m_parsing_slice_info {
            let object_id = self.m_curr_config.object_id;
            let volume_id = self.m_curr_config.volume_id;
            let object = match self.m_objects_metadata.get_mut(&object_id) {
                Some(o) => o,
                None => {
                    self.add_error(format!("Cannot find object for metadata, id {}", object_id));
                    return false;
                }
            };
            if volume_id == -1 {
                object.metadata.push(Metadata::new(&key, &value));
            } else if (volume_id as usize) < object.volumes.len() {
                object.volumes[volume_id as usize].metadata.push(Metadata::new(&key, &value));
            }
        } else {
            //plater
            // FIDELITY-NOTE: the C++ dereferences `m_curr_plater` without a check in most branches
            // (undefined behaviour when a slice info carries those keys outside a known plate);
            // the port skips them.
            if key == PLATERID_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.plate_index = atoi(&value);
                }
            } else if key == PLATER_NAME_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.plate_name = crate::utils::xml_unescape(&value);
                }
            } else if key == LOCK_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    istream_boolalpha(&value, &mut p.locked);
                }
            } else if key == BED_TYPE_ATTR {
                let mut bed_type = BedType::BtPC;
                ConfigOptionEnum::<BedType>::from_string(&value, &mut bed_type);
                if let Some(p) = self.curr_plater_mut() {
                    p.config.set_key_value("curr_bed_type", Box::new(ConfigOptionEnum::<BedType>::with_value(bed_type)));
                }
            } else if key == PRINT_SEQUENCE_ATTR {
                let mut print_sequence = PrintSequence::ByLayer;
                ConfigOptionEnum::<PrintSequence>::from_string(&value, &mut print_sequence);
                if let Some(p) = self.curr_plater_mut() {
                    p.config.set_key_value("print_sequence", Box::new(ConfigOptionEnum::<PrintSequence>::with_value(print_sequence)));
                }
            } else if key == FIRST_LAYER_PRINT_SEQUENCE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.config.set_key_value("first_layer_print_sequence", Box::new(ConfigOptionInts::from_values(get_vector_from_string(&value))));
                }
            } else if key == OTHER_LAYERS_PRINT_SEQUENCE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    let mut values = get_vector_from_string(&value);
                    if values.len() > 2 && values[0] > 0 && values[0] < i32::MAX && values[1] == i32::MAX {
                        values[1] = i32::MAX - 1;
                    }
                    p.config.set_key_value("other_layers_print_sequence", Box::new(ConfigOptionInts::from_values(values)));
                }
            } else if key == OTHER_LAYERS_PRINT_SEQUENCE_NUMS_ATTR {
                match std_stoi(&value) {
                    Ok(v) => {
                        if let Some(p) = self.curr_plater_mut() {
                            p.config.set_key_value("other_layers_print_sequence_nums", Box::new(ConfigOptionInt::with_value(v)));
                        }
                    }
                    Err(e) => {
                        // std::stoi throws through the expat callback
                        self.m_pending_exception = Some(Bbs3mfException(e));
                        return false;
                    }
                }
            } else if key == SPIRAL_VASE_MODE {
                let mut spiral_mode = false;
                istream_boolalpha(&value, &mut spiral_mode);
                if let Some(p) = self.curr_plater_mut() {
                    p.config.set_key_value("spiral_mode", Box::new(ConfigOptionBool::with_value(spiral_mode)));
                }
            } else if key == FILAMENT_MAP_MODE_ATTR {
                let mut map_mode = FilamentMapMode::FmmAutoForFlush;
                // handle old versions, only load manual params
                if value != "Auto" {
                    ConfigOptionEnum::<FilamentMapMode>::from_string(&value, &mut map_mode);
                    if let Some(p) = self.curr_plater_mut() {
                        p.config.set_key_value("filament_map_mode", Box::new(ConfigOptionEnum::<FilamentMapMode>::with_value(map_mode)));
                    }
                }
            } else if key == FILAMENT_MAP_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    let mut filament_map = get_vector_from_string(&value);
                    for v in filament_map.iter_mut() {
                        if *v < 1 {
                            *v = 1;
                        }
                    }
                    p.config.set_key_value("filament_map", Box::new(ConfigOptionInts::from_values(filament_map.clone())));
                    p.filament_maps = filament_map;
                }
            } else if key == FILAMENT_VOL_MAP_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    let mut filament_volume_map = get_vector_from_string(&value);
                    for v in filament_volume_map.iter_mut() {
                        // 保留合法的新喷嘴类型，仅将越界值回退为标准流量。
                        if *v < crate::print_config::enums::NozzleVolumeType::NvtStandard as i32 || *v >= crate::print_config::enums::NozzleVolumeType::NvtCount as i32 {
                            *v = 0;
                        }
                    }
                    p.config.set_key_value("filament_volume_map", Box::new(ConfigOptionInts::from_values(filament_volume_map)));
                }
            } else if key == GCODE_FILE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.gcode_file = value;
                }
            } else if key == THUMBNAIL_FILE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.thumbnail_file = value;
                }
            } else if key == NO_LIGHT_THUMBNAIL_FILE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.no_light_thumbnail_file = value;
                }
            } else if key == TOP_FILE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.top_file = value;
                }
            } else if key == PICK_FILE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.pick_file = value;
                }
            } else if key == PATTERN_BBOX_FILE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.pattern_bbox_file = value;
                }
            } else if key == INSTANCEID_ATTR {
                self.m_curr_instance.instance_id = atoi(&value);
            } else if key == IDENTIFYID_ATTR {
                self.m_curr_instance.identify_id = atoi(&value);
            } else if key == OBJECT_ID_ATTR {
                self.m_curr_instance.object_id = atoi(&value);
            } else if key == PLATE_IDX_ATTR {
                let plate_index = atoi(&value);
                if self.m_plater_data.contains_key(&plate_index) {
                    self.m_curr_plater = Some(CurrPlater::InMap(plate_index));
                }
            } else if key == SLICE_PREDICTION_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.gcode_prediction = value;
                }
            } else if key == SLICE_WEIGHT_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.gcode_weight = value;
                }
            } else if key == DEFAULT_AMS_TYPE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.default_ams_type = value;
                }
            } else if key == OUTSIDE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    istream_boolalpha(&value, &mut p.toolpath_outside);
                }
            } else if key == SUPPORT_USED_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    istream_boolalpha(&value, &mut p.is_support_used);
                }
            } else if key == LABEL_OBJECT_ENABLED_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    istream_boolalpha(&value, &mut p.is_label_object_enabled);
                }
            } else if key == SUPPORT_MATERIAL_ON_WIPE_TOWER_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    istream_boolalpha(&value, &mut p.support_material_on_wipe_tower);
                }
            } else if key == ENABLE_FILAMENT_DYNAMIC_MAP_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    let mut enable_filament_dynamic_map = false;
                    istream_boolalpha(&value, &mut enable_filament_dynamic_map);
                    p.config.set_key_value("enable_filament_dynamic_map", Box::new(ConfigOptionBool::with_value(enable_filament_dynamic_map)));
                }
            } else if key == HAS_FILAMENT_SWITCHER_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    let mut has_filament_switcher = false;
                    istream_boolalpha(&value, &mut has_filament_switcher);
                    p.config.set_key_value("has_filament_switcher", Box::new(ConfigOptionBool::with_value(has_filament_switcher)));
                }
            } else if key == PRINTER_MODEL_ID_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.printer_model_id = value;
                }
            } else if key == NOZZLE_VOLUME_TYPE_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.nozzle_volume_types = value;
                }
            } else if key == NOZZLE_DIAMETERS_ATTR {
                if let Some(p) = self.curr_plater_mut() {
                    p.nozzle_diameters = value;
                }
            }
        }

        true
    }

    /// bbs_3mf.cpp:5176-5180
    fn _handle_end_config_metadata(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:5182-5213
    fn _handle_start_config_filament(&mut self, attributes: &[(String, String)]) -> bool {
        if let Some(p) = self.curr_plater_mut() {
            let id = bbs_get_attribute_value_string(attributes, FILAMENT_ID_TAG);
            let type_ = bbs_get_attribute_value_string(attributes, FILAMENT_TYPE_TAG);
            let color = bbs_get_attribute_value_string(attributes, FILAMENT_COLOR_TAG);
            let used_m = bbs_get_attribute_value_string(attributes, FILAMENT_USED_M_TAG);
            let used_g = bbs_get_attribute_value_string(attributes, FILAMENT_USED_G_TAG);
            let filament_id = bbs_get_attribute_value_string(attributes, FILAMENT_TRAY_INFO_ID_TAG);
            let group_id = bbs_get_attribute_value_string(attributes, FILAMENT_NOZZLE_GROUP_ID_TAG);
            let nozzle_diameter = bbs_get_attribute_value_string(attributes, FILAMENT_NOZZLE_DIAMETER_TAG);
            let volume_type = bbs_get_attribute_value_string(attributes, FILAMENT_NOZZLE_VOLUME_TYPE_TAG);
            let mut filament_info = FilamentInfo::default();
            filament_info.id = atoi(&id) - 1;
            filament_info.type_ = type_;
            filament_info.color = color;
            filament_info.used_m = atof(&used_m) as f32;
            filament_info.used_g = atof(&used_g) as f32;
            filament_info.filament_id = filament_id;
            filament_info.group_id = parse_int_list(&group_id);
            filament_info.nozzle_diameter = crate::locales_utils::string_to_double_decimal_point(&nozzle_diameter).0;
            filament_info.nozzle_volume_type = volume_type;
            let total_load_time = bbs_get_attribute_value_string(attributes, FILAMENT_TOTAL_LOAD_TIME_TAG);
            let total_unload_time = bbs_get_attribute_value_string(attributes, FILAMENT_TOTAL_UNLOAD_TIME_TAG);
            let mut extra = FilamentInfoExtra::default();
            extra.total_load_time = if total_load_time.is_empty() { 0.0 } else { atof(&total_load_time) };
            extra.total_unload_time = if total_unload_time.is_empty() { 0.0 } else { atof(&total_unload_time) };
            let directly_used = bbs_get_attribute_value_string(attributes, FILAMENT_DIRECTLY_USED_ATTR);
            extra.directly_used = if directly_used.is_empty() { true } else { directly_used == "true" || directly_used == "1" };
            p.slice_filaments_info.push(filament_info);
            p.slice_filaments_extra.push(extra);
        }
        true
    }

    /// bbs_3mf.cpp:5215-5219
    fn _handle_end_config_filament(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:5221-5231
    fn _handle_start_config_pause(&mut self, attributes: &[(String, String)]) -> bool {
        if let Some(p) = self.curr_plater_mut() {
            let pause_info = PausePrintInfo {
                percent: atoi(&bbs_get_attribute_value_string(attributes, PAUSE_PERCENT_ATTR)),
                remaining_time: atoi(&bbs_get_attribute_value_string(attributes, PAUSE_REMAINING_TIME_ATTR)),
                layer_id: atoi(&bbs_get_attribute_value_string(attributes, PAUSE_LAYER_ATTR)) as u32,
            };
            p.pause_printing.push(pause_info);
        }
        true
    }

    /// bbs_3mf.cpp:5233-5248
    fn _handle_start_config_mixed_filament(&mut self, attributes: &[(String, String)]) -> bool {
        if let Some(p) = self.curr_plater_mut() {
            let id = bbs_get_attribute_value_string(attributes, FILAMENT_ID_TAG);
            let type_ = bbs_get_attribute_value_string(attributes, FILAMENT_TYPE_TAG);
            let color = bbs_get_attribute_value_string(attributes, FILAMENT_COLOR_TAG);
            let components = bbs_get_attribute_value_string(attributes, MIXED_FILAMENT_COMPONENTS_TAG);
            let mixed_info = PlateMixedFilamentInfo { id: atoi(&id), type_, color, components };
            p.mixed_filaments_info.push(mixed_info);
        }
        true
    }

    /// bbs_3mf.cpp:5250-5267
    fn _handle_start_config_warning(&mut self, attributes: &[(String, String)]) -> bool {
        if let Some(p) = self.curr_plater_mut() {
            let msg = bbs_get_attribute_value_string(attributes, WARNING_MSG_TAG);
            let lvl_str = bbs_get_attribute_value_string(attributes, "level");
            let mut sw = SliceWarning::default();
            sw.msg = msg;
            sw.level = atoi(&lvl_str);

            p.warnings.push(sw);
        }
        true
    }

    /// bbs_3mf.cpp:5269-5273
    fn _handle_end_config_warning(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:5275-5300
    fn _handle_start_config_nozzle(&mut self, attributes: &[(String, String)]) -> bool {
        if let Some(p) = self.curr_plater_mut() {
            // id="0" extruder_id="1" nozzle_diameter="0.4" volume_type="nvtNormal"
            let id = bbs_get_attribute_value_string(attributes, "id");
            let extruder_id = bbs_get_attribute_value_string(attributes, "extruder_id");
            let nozzle_diameter = bbs_get_attribute_value_string(attributes, "nozzle_diameter");
            let volume_type = bbs_get_attribute_value_string(attributes, "volume_type");

            use crate::config::ConfigEnum;
            let volume_type_str_to_enum = <crate::print_config::enums::NozzleVolumeType as ConfigEnum>::get_enum_values();

            let mut nozzle_info = NozzleInfo::default();
            nozzle_info.group_id = atoi(&id);
            nozzle_info.extruder_id = atoi(&extruder_id) - 1;
            nozzle_info.diameter = nozzle_diameter;

            // FIDELITY-NOTE: `MultiNozzleUtils::NozzleInfo::volume_type` of the crate is the older
            // `extruder::NozzleVolumeType` (four values); a newer type maps through its integer.
            nozzle_info.volume_type = match volume_type_str_to_enum.get(&volume_type) {
                Some(v) => crate::extruder::NozzleVolumeType::from_i32(*v),
                None => crate::extruder::NozzleVolumeType::NvtStandard,
            };

            p.nozzles_info.push(nozzle_info);
        }
        true
    }

    /// bbs_3mf.cpp:5302-5306
    fn _handle_end_config_nozzle(&mut self) -> bool {
        // do nothing
        true
    }

    /// bbs_3mf.cpp:5308-5318
    fn _handle_start_config_ams_item(&mut self, attributes: &[(String, String)]) -> bool {
        if let Some(p) = self.curr_plater_mut() {
            let ams_info = AmsLoadUnloadTimeInfo {
                ams_type: bbs_get_attribute_value_string(attributes, AMS_TYPE_ATTR),
                load_time: bbs_get_attribute_value_float(attributes, AMS_LOAD_TIME_ATTR) as f64,
                unload_time: bbs_get_attribute_value_float(attributes, AMS_UNLOAD_TIME_ATTR) as f64,
            };
            p.ams_list.push(ams_info);
        }
        true
    }

    /// bbs_3mf.cpp:5320-5327
    fn _handle_start_config_plater(&mut self, _attributes: &[(String, String)]) -> bool {
        if !self.m_parsing_slice_info {
            self.m_curr_plater = Some(CurrPlater::New(Box::new(PlateData::new())));
        }

        true
    }

    /// bbs_3mf.cpp:5329-5339
    fn _handle_end_config_plater(&mut self) -> bool {
        match self.m_curr_plater.take() {
            None => {
                self.add_error("_handle_end_config_plater: don't find plate created before");
                false
            }
            Some(CurrPlater::New(p)) => {
                // std::map::emplace does not replace an entry of the same plate index
                self.m_plater_data.entry(p.plate_index).or_insert(*p);
                true
            }
            Some(CurrPlater::InMap(_)) => {
                // the plate is already in the map under its own plate index
                true
            }
        }
    }

    /// bbs_3mf.cpp:5341-5351
    fn _handle_start_config_plater_instance(&mut self, _attributes: &[(String, String)]) -> bool {
        if self.m_curr_plater.is_none() {
            self.add_error("_handle_start_config_plater_instance: don't find plate created before");
            return false;
        }

        //do nothing
        true
    }

    /// bbs_3mf.cpp:5353-5373
    fn _handle_end_config_plater_instance(&mut self) -> bool {
        if self.m_curr_plater.is_none() {
            self.add_error("_handle_end_config_plater_instance: don't find plate created before");
            return false;
        }
        if self.m_curr_instance.object_id == -1 || self.m_curr_instance.instance_id == -1 {
            //add_error("invalid object id/instance id");
            //skip this instance
            self.m_curr_instance.object_id = -1;
            self.m_curr_instance.instance_id = -1;
            self.m_curr_instance.identify_id = 0;
            return true;
        }

        let ci = self.m_curr_instance;
        if let Some(p) = self.curr_plater_mut() {
            p.obj_inst_map.entry(ci.object_id).or_insert((ci.instance_id, ci.identify_id));
        }
        self.m_curr_instance.object_id = -1;
        self.m_curr_instance.instance_id = -1;
        self.m_curr_instance.identify_id = 0;
        true
    }

    /// bbs_3mf.cpp:5375-5378
    fn _handle_start_assemble(&mut self, _attributes: &[(String, String)]) -> bool {
        true
    }

    /// bbs_3mf.cpp:5380-5384
    fn _handle_end_assemble(&mut self) -> bool {
        //do nothing
        true
    }

    /// bbs_3mf.cpp:5386-5427
    fn _handle_start_assemble_item(&mut self, model: &mut Model, attributes: &[(String, String)]) -> bool {
        if !self.m_load_model {
            return true;
        }

        let mut object_id = bbs_get_attribute_value_int(attributes, OBJECT_ID_ATTR);
        let instance_id = bbs_get_attribute_value_int(attributes, INSTANCEID_ATTR);

        let index_iter = match self.m_index_paths.get(&object_id) {
            None => {
                self.add_error(format!("can not find object for assemble item, id= {}", object_id));
                return false;
            }
            Some(p) => p.clone(),
        };
        let temp_id: Id = (index_iter, object_id);
        match self.m_objects.get(&temp_id) {
            None => {
                self.add_error(format!("can not find object for assemble item, id= {}", object_id));
                return false;
            }
            Some(i) => object_id = *i,
        }

        let transform = bbs_get_transform_from_3mf_specs_string(&bbs_get_attribute_value_string(attributes, TRANSFORM_ATTR));
        let ofs2ass = bbs_get_offset_from_3mf_specs_string(&bbs_get_attribute_value_string(attributes, OFFSET_ATTR));
        // BBS: optional volume index. When the attribute is missing the entry targets the instance (legacy behaviour). When present it carries a per-volume assemble matrix.
        let has_volume_attr = bbs_get_attribute_value_charptr(attributes, VOLUMEID_ATTR).is_some();
        let volume_id = if has_volume_attr { bbs_get_attribute_value_int(attributes, VOLUMEID_ATTR) } else { -1 };
        if object_id >= 0 && (object_id as usize) < model.objects.len() {
            if has_volume_attr {
                // BBS: model_settings.config is parsed before _generate_volumes_new fills ModelObject::volumes
                if volume_id >= 0 {
                    self.m_pending_volume_assemble.push(PendingVolumeAssemble { model_object_index: object_id, volume_id, transform });
                }
            } else {
                let mo = &mut model.objects[object_id as usize];
                if instance_id < 0 || instance_id >= mo.instances.len() as i32 {
                    log::warn!("invalid instance id {}\n", instance_id);
                } else {
                    mo.instances[instance_id as usize].set_assemble_from_transform(&transform);
                    mo.instances[instance_id as usize].set_offset_to_assembly(&ofs2ass);
                }
            }
        }
        true
    }

    /// bbs_3mf.cpp:5429-5432
    fn _handle_end_assemble_item(&mut self) -> bool {
        true
    }

    /// bbs_3mf.cpp:5434-5526
    fn _handle_start_text_info_item(&mut self, attributes: &[(String, String)]) -> bool {
        use crate::text_configuration::{HorizontalAlign, VerticalAlign};
        use crate::utils::xml_unescape;
        let object_id = self.m_curr_config.object_id;
        let volume_id = self.m_curr_config.volume_id;
        let object = match self.m_objects_metadata.get_mut(&object_id) {
            Some(o) => o,
            None => {
                self.add_error(format!("can not find object for text_info, id {}", object_id));
                return false;
            }
        };
        if volume_id == -1 || (object.volumes.len() as i64 - 1) < volume_id as i64 {
            self.add_error("can not find part for text_info");
            return false;
        }

        let volume = &mut object.volumes[volume_id as usize];
        let mut text_info = crate::model::TextInfo::default();
        text_info.m_text = xml_unescape(&bbs_get_attribute_value_string(attributes, TEXT_ATTR));
        text_info.m_font_name = xml_unescape(&bbs_get_attribute_value_string(attributes, FONT_NAME_ATTR));
        text_info.text_configuration.style.prop.face_name = Some(text_info.m_font_name.clone());
        text_info.m_font_version = bbs_get_attribute_value_string(attributes, FONT_VERSION_ATTR);
        text_info.text_configuration.style.name = bbs_get_attribute_value_string(attributes, STYLE_NAME_ATTR);
        if (atof(&text_info.m_font_version) as f32) > 2.2f32 {
            text_info.text_configuration.style.prop.boldness = Some(bbs_get_attribute_value_float(attributes, BOLDNESS_ATTR));
            text_info.text_configuration.style.prop.skew = Some(bbs_get_attribute_value_float(attributes, SKEW_ATTR));
        }
        text_info.m_curr_font_idx = bbs_get_attribute_value_int(attributes, FONT_INDEX_ATTR);

        text_info.m_font_size = bbs_get_attribute_value_float(attributes, FONT_SIZE_ATTR);
        text_info.m_thickness = bbs_get_attribute_value_float(attributes, THICKNESS_ATTR);
        text_info.m_embeded_depth = bbs_get_attribute_value_float(attributes, EMBEDED_DEPTH_ATTR);
        text_info.m_rotate_angle = bbs_get_attribute_value_float(attributes, ROTATE_ANGLE_ATTR);
        text_info.m_text_gap = bbs_get_attribute_value_float(attributes, TEXT_GAP_ATTR);
        // Persist FontProp::line_gap (font points), same as face_name on text_configuration.
        // Missing attribute must not become an explicit 0 that later overwrites a loaded style.
        if bbs_has_attribute_value_int(attributes, LINE_GAP_ATTR) {
            let line_gap = bbs_get_attribute_value_int(attributes, LINE_GAP_ATTR);
            if line_gap != 0 {
                text_info.text_configuration.style.prop.line_gap = Some(line_gap);
            }
        }
        // FontProp::align, same names as EmbossStyleManager. Missing attribute keeps the default center.
        {
            let h = bbs_get_attribute_value_string(attributes, HORIZONTAL_ALIGN_ATTR);
            if h == "left" || h == "0" {
                text_info.text_configuration.style.prop.align.0 = HorizontalAlign::Left;
            } else if h == "right" || h == "2" {
                text_info.text_configuration.style.prop.align.0 = HorizontalAlign::Right;
            } else if !h.is_empty() {
                text_info.text_configuration.style.prop.align.0 = HorizontalAlign::Center;
            }

            let v = bbs_get_attribute_value_string(attributes, VERTICAL_ALIGN_ATTR);
            if v == "top" || v == "0" {
                text_info.text_configuration.style.prop.align.1 = VerticalAlign::Top;
            } else if v == "bottom" || v == "2" {
                text_info.text_configuration.style.prop.align.1 = VerticalAlign::Bottom;
            } else if !v.is_empty() {
                text_info.text_configuration.style.prop.align.1 = VerticalAlign::Center;
            }
        }

        text_info.m_bold = bbs_get_attribute_value_int(attributes, BOLD_ATTR) != 0;
        text_info.m_italic = bbs_get_attribute_value_int(attributes, ITALIC_ATTR) != 0;
        if bbs_has_attribute_value_int(attributes, SURFACE_TYPE) {
            text_info.m_surface_type = bbs_get_attribute_value_int(attributes, SURFACE_TYPE);
        } else {
            //old version
            text_info.m_is_surface_text = bbs_get_attribute_value_int(attributes, SURFACE_TEXT_ATTR) != 0;
            text_info.m_keep_horizontal = bbs_get_attribute_value_int(attributes, KEEP_HORIZONTAL_ATTR) != 0;

            let is_surface_text = text_info.m_is_surface_text;
            let is_keep_horizontal = text_info.m_keep_horizontal;
            // Model.hpp TextInfo::TextType { HORIZONAL, SURFACE, SURFACE_HORIZONAL }
            let convert_text_type = |is_surface_text: bool, is_keep_horizontal: bool, text_type: &mut i32| {
                if is_surface_text && is_keep_horizontal {
                    *text_type = 2; // SURFACE_HORIZONAL
                } else if is_surface_text {
                    *text_type = 1; // SURFACE
                } else if is_keep_horizontal {
                    *text_type = 0; // HORIZONAL
                } else {
                    *text_type = 0; // HORIZONAL
                }
            };
            convert_text_type(is_surface_text, is_keep_horizontal, &mut text_info.m_surface_type);
        }

        // FIDELITY-NOTE: `TextInfo::m_rr` (the raycast result: hit mesh, position and normal) is not
        // carried by the crate's TextInfo; HIT_MESH_ATTR, HIT_POSITION_ATTR and HIT_NORMAL_ATTR are
        // read and dropped.
        let _hit_mesh = bbs_get_attribute_value_int(attributes, HIT_MESH_ATTR);
        let hit_pos = bbs_get_attribute_value_string(attributes, HIT_POSITION_ATTR);
        if !hit_pos.is_empty() {
            let _ = get_vec3_from_string(&hit_pos);
        }
        let hit_normal = bbs_get_attribute_value_string(attributes, HIT_NORMAL_ATTR);
        if !hit_normal.is_empty() {
            let _ = get_vec3_from_string(&hit_normal);
        }

        volume.text_info = text_info;
        true
    }

    /// bbs_3mf.cpp:5528-5531
    #[allow(dead_code)]
    fn _handle_end_text_info_item(&mut self) -> bool {
        true
    }

    /// bbs_3mf.cpp:5547-5561
    fn _handle_start_relationships_element(&mut self, name: &str, attributes: &[(String, String)]) {
        if !self.m_xml_parser {
            return;
        }

        let mut res = true;

        if RELATIONSHIP_TAG == name {
            res = self._handle_start_relationship(attributes);
        }

        self.m_curr_characters.clear();
        if !res {
            self._stop_xml_parser("");
        }
    }

    /// bbs_3mf.cpp:5563-5572
    fn _handle_end_relationships_element(&mut self, _name: &str) {
        if !self.m_xml_parser {
            return;
        }

        let res = true;

        if !res {
            self._stop_xml_parser("");
        }
    }

    /// bbs_3mf.cpp:5574-5590
    fn _handle_start_relationship(&mut self, attributes: &[(String, String)]) -> bool {
        let path = bbs_get_attribute_value_string(attributes, TARGET_ATTR);
        let type_ = bbs_get_attribute_value_string(attributes, RELS_TYPE_ATTR);
        if type_.starts_with("http://schemas.microsoft.com/3dmanufacturing/") && type_.ends_with("3dmodel") {
            if self.m_start_part_path.is_empty() {
                self.m_start_part_path = path;
            } else {
                self.m_sub_model_paths.push(path);
            }
        } else if type_.starts_with("http://schemas.openxmlformats.org/") && type_.ends_with("thumbnail") {
            if path.ends_with(".png") {
                self.m_thumbnail_path = path;
            }
        } else if type_.starts_with("http://schemas.bambulab.com/") && type_.ends_with("cover-thumbnail-middle") {
            self.m_thumbnail_middle = path;
        } else if type_.starts_with("http://schemas.bambulab.com/") && type_.ends_with("cover-thumbnail-small") {
            self.m_thumbnail_small = path;
        }
        true
    }

    /// bbs_3mf.cpp:5592-5617
    fn _generate_current_object_list(sub_objects: &mut Vec<Component>, object_id: Id, current_objects: &BTreeMap<Id, CurrentObject>) {
        let mut id_list: std::collections::VecDeque<(Component, Transform3d)> = std::collections::VecDeque::new();
        id_list.push_back((Component::new(object_id, Transform3d::identity()), Transform3d::identity()));

        while let Some(current_item) = id_list.pop_front() {
            let current_id = current_item.0.clone();
            if let Some((key, current_object)) = current_objects.get_key_value(&current_id.object_id) {
                //found one
                if !current_object.components.is_empty() {
                    for comp in &current_object.components {
                        id_list.push_back((comp.clone(), crate::model::eigen_mul_affine(&current_item.1, &comp.transform)));
                    }
                } else if !current_object.geometry.empty() {
                    sub_objects.push(Component::new(key.clone(), current_item.1));
                }
            }
        }
    }

    /// bbs_3mf.cpp:5619-5874
    fn _generate_volumes_new(
        &mut self,
        model: &mut Model,
        object_idx: usize,
        sub_objects: &[Component],
        volumes: &[VolumeMetadata],
        config_substitutions: &mut crate::config::ConfigSubstitutionContext,
    ) -> Result<bool, Bbs3mfException> {
        if !model.objects[object_idx].volumes.is_empty() {
            self.add_error("object already built with parts");
            return Ok(false);
        }

        //unsigned int geo_tri_count = (unsigned int)geometry.triangles.size();
        let mut renamed_volumes_count: u32 = 0;

        for index in 0..sub_objects.len() {
            //find the volume metadata firstly
            let sub_comp = sub_objects[index].clone();
            let object_id = sub_comp.object_id.clone();
            let sub_object = match self.m_current_objects.get(&object_id) {
                Some(o) => o,
                None => {
                    self.add_error(format!("sub_objects can not be found, id={}", object_id.1));
                    return Ok(false);
                }
            };

            let default_volume_data = VolumeMetadata::with_subobject(sub_object.id, ModelVolumeType::ModelPart);
            let mut volume_data: Option<&VolumeMetadata> = None;
            if index < volumes.len() && volumes[index].subobject_id == sub_object.id {
                volume_data = Some(&volumes[index]);
            } else {
                for volume_iter in volumes {
                    if volume_iter.subobject_id == sub_object.id {
                        volume_data = Some(volume_iter);
                        break;
                    }
                }
            }

            let mut volume_matrix_to_object = Transform3d::identity();
            let mut has_transform = false;
            let mut shared_mesh_id = object_id.1;
            if let Some(vd) = volume_data {
                let mut found_count = 0;
                // extract the volume transformation from the volume's metadata, if present
                for metadata in &vd.metadata {
                    if metadata.key == MATRIX_KEY {
                        volume_matrix_to_object = crate::geometry::geometry::transform3d_from_string(&metadata.value);
                        has_transform = !transform_is_approx(&volume_matrix_to_object, &Transform3d::identity(), 1e-10);
                        found_count += 1;
                    } else if metadata.key == MESH_SHARED_KEY {
                        //add the shared mesh logic
                        shared_mesh_id = atoi(&metadata.value);
                        found_count += 1;
                        log::info!("shared_mesh_id {}", shared_mesh_id);
                    }

                    if found_count >= 2 {
                        break;
                    }
                }
            } else {
                //create a volume_data
                volume_data = Some(&default_volume_data);
            }
            let volume_data = volume_data.unwrap();

            let mut shared_volume: Option<(usize, usize)> = None;
            log::debug!("subobject_id {}, shared_mesh_id {}", sub_object.id, shared_mesh_id);
            if shared_mesh_id != -1 {
                if let Some(v) = self.m_shared_meshes.get(&shared_mesh_id) {
                    shared_volume = Some(*v);
                    log::info!("found shared mesh, id {}, mesh {:?}", shared_mesh_id, v);
                }
            } else {
                //for some cases, object point to this shared mesh already loaded, treat that one as the root
                if let Some(v) = self.m_shared_meshes.get(&sub_object.id) {
                    shared_volume = Some(*v);
                    log::info!("already loaded copy-share mesh before, id {}, mesh {:?}", sub_object.id, v);
                }
            }

            let triangles_count = sub_object.geometry.triangles.len();
            if triangles_count == 0 {
                self.add_error(format!("found no trianges in the object {}", sub_object.id));
                return Ok(false);
            }
            let mut flipped = false;
            let volume_idx: usize;
            if shared_volume.is_none() {
                // splits volume out of imported geometry
                let mut its = indexed_triangle_set::default();
                its.indices = sub_object.geometry.triangles.clone();
                for face in &its.indices {
                    for k in 0..3 {
                        let tri_id = face[k];
                        if tri_id < 0 || tri_id >= sub_object.geometry.vertices.len() as i32 {
                            self.add_error(format!("invalid vertex id in object {}", sub_object.id));
                            return Ok(false);
                        }
                    }
                }

                its.vertices = sub_object.geometry.vertices.clone();

                // BBS
                // FIDELITY-NOTE: the crate's indexed_triangle_set has no `properties` (FaceProperty
                // per face); the face_property strings read from the model are not attached.

                let mut triangle_mesh = crate::model::ModelMesh::from_its_with_errors(its, volume_data.mesh_stats);

                // BBS: no need to multiply the instance matrix into the volume
                if triangle_mesh.volume() < 0.0 {
                    triangle_mesh.flip_triangles();
                    flipped = true;
                }

                let is_text = !volume_data.text_info.m_text.is_empty();
                let modify_to_center_geometry = !is_text; //text do not modify_to_center_geometry
                model.objects[object_idx].add_volume_mesh(triangle_mesh, ModelVolumeType::ModelPart, modify_to_center_geometry);
                volume_idx = model.objects[object_idx].volumes.len() - 1;

                if shared_mesh_id != -1 {
                    //for some cases the shared mesh is in other plate and not loaded in cli slicing
                    //we need to use the first one in the same plate instead
                    self.m_shared_meshes.insert(shared_mesh_id, (object_idx, volume_idx));
                } else {
                    self.m_shared_meshes.insert(sub_object.id, (object_idx, volume_idx));
                }
            } else {
                //create volume to use shared mesh
                let (so, sv) = shared_volume.unwrap();
                let shared = model.objects[so].volumes[sv].clone();
                model.objects[object_idx].add_volume_with_shared_mesh(&shared, ModelVolumeType::Invalid);
                volume_idx = model.objects[object_idx].volumes.len() - 1;
                log::info!("create volume using shared_mesh {:?}", (so, sv));
            }
            let object_name = model.objects[object_idx].name.clone();
            let volume = &mut model.objects[object_idx].volumes[volume_idx];
            // stores the volume matrix taken from the metadata, if present
            if has_transform {
                volume.source.transform = Transformation::from_transform(volume_matrix_to_object);
            }

            volume.calculate_convex_hull();

            // Save the color data of this volume（TriangleColor）
            if !self.m_is_bbl_3mf && sub_object.geometry.triangle_colors.len() == triangles_count {
                let mut vol_color_info = VolumeColorInfo::new();
                vol_color_info.pid = sub_object.pid;
                vol_color_info.pindex = sub_object.pindex;
                vol_color_info.triangle_colors = sub_object.geometry.triangle_colors.clone();
                // flip_triangles() swapped the second and third corner of every face.
                if flipped {
                    for color in vol_color_info.triangle_colors.iter_mut() {
                        color.indices.swap(1, 2);
                    }
                }
                self.m_volume_color_data.insert(volume.id().id as i32, vol_color_info);
            }

            //set transform from 3mf
            let comp_transformatino = Transformation::from_transform(sub_comp.transform);
            let t = comp_transformatino * volume.get_transformation().clone();
            volume.set_transformation(&t);
            if shared_volume.is_some() {
                let shift = volume.mesh().get_init_shift();
                if !vec3d_is_approx(&shift, &Vec3d::zeros()) {
                    volume.translate(&shift);
                }
            }

            // recreate custom supports, seam and mmu segmentation from previously loaded attribute
            {
                volume.supported_facets.reserve(triangles_count as i32);
                volume.fuzzy_skin_facets.reserve(triangles_count as i32);
                volume.seam_facets.reserve(triangles_count as i32);
                volume.mmu_segmentation_facets.reserve(triangles_count as i32);
                for i in 0..triangles_count {
                    if !sub_object.geometry.custom_supports[i].is_empty() {
                        volume.supported_facets.set_triangle_from_string(i as i32, &sub_object.geometry.custom_supports[i]);
                    }
                    if !sub_object.geometry.custom_fuzzy_skin[i].is_empty() {
                        volume.fuzzy_skin_facets.set_triangle_from_string(i as i32, &sub_object.geometry.custom_fuzzy_skin[i]);
                    }
                    if !sub_object.geometry.custom_seam[i].is_empty() {
                        volume.seam_facets.set_triangle_from_string(i as i32, &sub_object.geometry.custom_seam[i]);
                    }
                    if !sub_object.geometry.mmu_segmentation[i].is_empty() {
                        volume.mmu_segmentation_facets.set_triangle_from_string(i as i32, &sub_object.geometry.mmu_segmentation[i]);
                    }
                }
                volume.supported_facets.shrink_to_fit();
                volume.fuzzy_skin_facets.shrink_to_fit();
                volume.seam_facets.shrink_to_fit();
                volume.mmu_segmentation_facets.shrink_to_fit();
                volume.mmu_segmentation_facets.touch();
            }

            volume.set_type(volume_data.part_type);
            if volume_data.shape_configuration.is_some() {
                // FIDELITY-NOTE: the crate's ModelVolume keeps whether the emboss shape is set.
                volume.emboss_shape = Some(());
            }
            if !volume_data.text_info.m_text.is_empty() {
                volume.set_text_info(volume_data.text_info.clone());
            }

            // apply the remaining volume's metadata
            for metadata in &volume_data.metadata {
                if metadata.key == NAME_KEY {
                    volume.name = metadata.value.clone();
                }
                //for old format
                else if metadata.key == VOLUME_TYPE_KEY || metadata.key == PART_TYPE_KEY {
                    volume.set_type(ModelVolume::type_from_string(&metadata.value));
                } else if metadata.key == SOURCE_FILE_KEY {
                    volume.source.input_file = metadata.value.clone();
                } else if metadata.key == SOURCE_OBJECT_ID_KEY {
                    volume.source.object_idx = atoi(&metadata.value);
                } else if metadata.key == SOURCE_VOLUME_ID_KEY {
                    volume.source.volume_idx = atoi(&metadata.value);
                } else if metadata.key == SOURCE_OFFSET_X_KEY {
                    volume.source.mesh_offset[0] = atof(&metadata.value);
                } else if metadata.key == SOURCE_OFFSET_Y_KEY {
                    volume.source.mesh_offset[1] = atof(&metadata.value);
                } else if metadata.key == SOURCE_OFFSET_Z_KEY {
                    volume.source.mesh_offset[2] = atof(&metadata.value);
                } else if metadata.key == SOURCE_IN_INCHES {
                    volume.source.is_converted_from_inches = metadata.value == "1";
                } else if metadata.key == SOURCE_IN_METERS {
                    volume.source.is_converted_from_meters = metadata.value == "1";
                } else if metadata.key == PART_GUID_KEY {
                    volume.set_part_guid(&metadata.value);
                } else if metadata.key == ASSEMBLY_SRC_GUID_KEY {
                    volume.set_assembly_src_guid(&metadata.value);
                } else if metadata.key == MATRIX_KEY || metadata.key == MESH_SHARED_KEY {
                    continue;
                } else {
                    volume.config.set_deserialize(&metadata.key, &metadata.value, config_substitutions, false)?;
                }
            }

            // this may happen for 3mf saved by 3rd part softwares
            if volume.name.is_empty() {
                volume.name = object_name.clone();
                if renamed_volumes_count > 0 {
                    volume.name += &format!("_{}", renamed_volumes_count + 1);
                }
                renamed_volumes_count += 1;
            }
        }

        Ok(true)
    }

    /// bbs_3mf.cpp:5876-6031 — the volumes of one indexed triangle set split by triangle ranges
    /// (files of the older format; `_load_model_from_file` builds volumes with
    /// `_generate_volumes_new`).
    #[allow(dead_code)]
    fn _generate_volumes(
        &mut self,
        model: &mut Model,
        object_idx: usize,
        geometry: &Geometry,
        volumes: &[VolumeMetadata],
        config_substitutions: &mut crate::config::ConfigSubstitutionContext,
    ) -> Result<bool, Bbs3mfException> {
        if !model.objects[object_idx].volumes.is_empty() {
            self.add_error("Found invalid volumes count");
            return Ok(false);
        }

        let geo_tri_count = geometry.triangles.len() as u32;
        let mut renamed_volumes_count: u32 = 0;

        for volume_data in volumes {
            if geo_tri_count <= volume_data.first_triangle_id
                || geo_tri_count <= volume_data.last_triangle_id
                || volume_data.last_triangle_id < volume_data.first_triangle_id
            {
                self.add_error("Found invalid triangle id");
                return Ok(false);
            }

            let mut volume_matrix_to_object = Transform3d::identity();
            let mut has_transform = false;
            // extract the volume transformation from the volume's metadata, if present
            for metadata in &volume_data.metadata {
                if metadata.key == MATRIX_KEY {
                    volume_matrix_to_object = crate::geometry::geometry::transform3d_from_string(&metadata.value);
                    has_transform = !transform_is_approx(&volume_matrix_to_object, &Transform3d::identity(), 1e-10);
                    break;
                }
            }

            // splits volume out of imported geometry
            let mut its = indexed_triangle_set::default();
            its.indices = geometry.triangles[volume_data.first_triangle_id as usize..=volume_data.last_triangle_id as usize].to_vec();
            let triangles_count = its.indices.len();
            if triangles_count == 0 {
                self.add_error("An empty triangle mesh found");
                return Ok(false);
            }

            {
                let mut min_id = its.indices[0][0];
                let mut max_id = min_id;
                for face in &its.indices {
                    for k in 0..3 {
                        let tri_id = face[k];
                        if tri_id < 0 || tri_id >= geometry.vertices.len() as i32 {
                            self.add_error("Found invalid vertex id");
                            return Ok(false);
                        }
                        min_id = min_id.min(tri_id);
                        max_id = max_id.max(tri_id);
                    }
                }
                its.vertices = geometry.vertices[min_id as usize..=max_id as usize].to_vec();

                // BBS — FIDELITY-NOTE: face properties are not attached (see _generate_volumes_new).

                // rebase indices to the current vertices list
                for face in its.indices.iter_mut() {
                    for k in 0..3 {
                        face[k] -= min_id;
                    }
                }
            }

            let mut triangle_mesh = crate::model::ModelMesh::from_its_with_errors(its, volume_data.mesh_stats);

            if !self.m_is_bbl_3mf {
                // if the 3mf was not produced by BambuStudio and there is only one instance,
                // bake the transformation into the geometry to allow the reload from disk command
                // to work properly
                let object = &mut model.objects[object_idx];
                if object.instances.len() == 1 {
                    let m = object.instances[0].get_transformation().matrix();
                    triangle_mesh.transform(&m, false);
                    object.instances[0].set_transformation(&Transformation::new());
                    //FIXME do the mesh fixing?
                }
            }
            if triangle_mesh.volume() < 0.0 {
                triangle_mesh.flip_triangles();
            }

            let object_name = model.objects[object_idx].name.clone();
            let volume = model.objects[object_idx].add_volume_mesh(triangle_mesh, ModelVolumeType::ModelPart, true);
            // stores the volume matrix taken from the metadata, if present
            if has_transform {
                volume.source.transform = Transformation::from_transform(volume_matrix_to_object);
            }
            volume.calculate_convex_hull();

            // recreate custom supports, seam and mmu segmentation from previously loaded attribute
            volume.supported_facets.reserve(triangles_count as i32);
            volume.fuzzy_skin_facets.reserve(triangles_count as i32);
            volume.seam_facets.reserve(triangles_count as i32);
            volume.mmu_segmentation_facets.reserve(triangles_count as i32);
            for i in 0..triangles_count {
                let index = volume_data.first_triangle_id as usize + i;
                if !geometry.custom_supports[index].is_empty() {
                    volume.supported_facets.set_triangle_from_string(i as i32, &geometry.custom_supports[index]);
                }
                if !geometry.custom_fuzzy_skin[index].is_empty() {
                    volume.fuzzy_skin_facets.set_triangle_from_string(i as i32, &geometry.custom_fuzzy_skin[index]);
                }
                if !geometry.custom_seam[index].is_empty() {
                    volume.seam_facets.set_triangle_from_string(i as i32, &geometry.custom_seam[index]);
                }
                if !geometry.mmu_segmentation[index].is_empty() {
                    volume.mmu_segmentation_facets.set_triangle_from_string(i as i32, &geometry.mmu_segmentation[index]);
                }
            }
            volume.supported_facets.shrink_to_fit();
            volume.fuzzy_skin_facets.shrink_to_fit();
            volume.seam_facets.shrink_to_fit();
            volume.mmu_segmentation_facets.shrink_to_fit();

            volume.set_type(volume_data.part_type);

            // apply the remaining volume's metadata
            for metadata in &volume_data.metadata {
                if metadata.key == NAME_KEY {
                    volume.name = metadata.value.clone();
                }
                //for old format
                else if metadata.key == VOLUME_TYPE_KEY || metadata.key == PART_TYPE_KEY {
                    volume.set_type(ModelVolume::type_from_string(&metadata.value));
                } else if metadata.key == SOURCE_FILE_KEY {
                    volume.source.input_file = metadata.value.clone();
                } else if metadata.key == SOURCE_OBJECT_ID_KEY {
                    volume.source.object_idx = atoi(&metadata.value);
                } else if metadata.key == SOURCE_VOLUME_ID_KEY {
                    volume.source.volume_idx = atoi(&metadata.value);
                } else if metadata.key == SOURCE_OFFSET_X_KEY {
                    volume.source.mesh_offset[0] = atof(&metadata.value);
                } else if metadata.key == SOURCE_OFFSET_Y_KEY {
                    volume.source.mesh_offset[1] = atof(&metadata.value);
                } else if metadata.key == SOURCE_OFFSET_Z_KEY {
                    volume.source.mesh_offset[2] = atof(&metadata.value);
                } else if metadata.key == SOURCE_IN_INCHES {
                    volume.source.is_converted_from_inches = metadata.value == "1";
                } else if metadata.key == SOURCE_IN_METERS {
                    volume.source.is_converted_from_meters = metadata.value == "1";
                } else if metadata.key == PART_GUID_KEY {
                    volume.set_part_guid(&metadata.value);
                } else if metadata.key == ASSEMBLY_SRC_GUID_KEY {
                    volume.set_assembly_src_guid(&metadata.value);
                } else {
                    volume.config.set_deserialize(&metadata.key, &metadata.value, config_substitutions, false)?;
                }
            }

            // this may happen for 3mf saved by 3rd part softwares
            if volume.name.is_empty() {
                volume.name = object_name.clone();
                if renamed_volumes_count > 0 {
                    volume.name += &format!("_{}", renamed_volumes_count + 1);
                }
                renamed_volumes_count += 1;
            }
        }

        Ok(true)
    }
}

/// The `static void XMLCALL` callbacks of `_BBS_3MF_Importer`: expat calls them with the importer
/// as its user data (`XML_SetUserData`), and they forward to the member handlers.
struct ImporterXmlCallbacks;

impl ImporterXmlCallbacks {
    /// bbs_3mf.cpp:5533-5538
    fn _handle_start_relationships_element(user_data: Option<&mut _BBS_3MF_Importer>, name: &str, attributes: &[(String, String)]) {
        if let Some(importer) = user_data {
            importer._handle_start_relationships_element(name, attributes);
        }
    }

    /// bbs_3mf.cpp:5540-5545
    fn _handle_end_relationships_element(user_data: Option<&mut _BBS_3MF_Importer>, name: &str) {
        if let Some(importer) = user_data {
            importer._handle_end_relationships_element(name);
        }
    }

    /// bbs_3mf.cpp:6033-6038
    fn _handle_start_model_xml_element(
        user_data: Option<&mut _BBS_3MF_Importer>,
        model: Option<&mut Model>,
        name: &str,
        attributes: &[(String, String)],
    ) {
        if let Some(importer) = user_data {
            importer._handle_start_model_xml_element(model, name, attributes);
        }
    }

    /// bbs_3mf.cpp:6040-6045
    fn _handle_end_model_xml_element(user_data: Option<&mut _BBS_3MF_Importer>, model: Option<&mut Model>, name: &str) {
        if let Some(importer) = user_data {
            importer._handle_end_model_xml_element(model, name);
        }
    }

    /// bbs_3mf.cpp:6047-6052
    fn _handle_xml_characters(user_data: Option<&mut _BBS_3MF_Importer>, s: &str) {
        if let Some(importer) = user_data {
            importer._handle_xml_characters(s);
        }
    }

    /// bbs_3mf.cpp:6054-6059
    fn _handle_start_config_xml_element(
        user_data: Option<&mut _BBS_3MF_Importer>,
        model: Option<&mut Model>,
        name: &str,
        attributes: &[(String, String)],
    ) {
        if let Some(importer) = user_data {
            importer._handle_start_config_xml_element(model, name, attributes);
        }
    }

    /// bbs_3mf.cpp:6061-6066
    fn _handle_end_config_xml_element(user_data: Option<&mut _BBS_3MF_Importer>, name: &str) {
        if let Some(importer) = user_data {
            importer._handle_end_config_xml_element(name);
        }
    }
}

/// The `static void XMLCALL` callbacks of `ObjectImporter` (bbs_3mf.cpp:1415-1417).
struct ObjectImporterXmlCallbacks;

impl ObjectImporterXmlCallbacks {
    /// bbs_3mf.cpp:6487-6492
    fn _handle_object_start_model_xml_element(user_data: Option<&mut ObjectImporter>, name: &str, attributes: &[(String, String)]) {
        if let Some(importer) = user_data {
            importer._handle_object_start_model_xml_element(name, attributes);
        }
    }

    /// bbs_3mf.cpp:6494-6499
    fn _handle_object_end_model_xml_element(user_data: Option<&mut ObjectImporter>, name: &str) {
        if let Some(importer) = user_data {
            importer._handle_object_end_model_xml_element(name);
        }
    }

    /// bbs_3mf.cpp:6501-6506
    fn _handle_object_xml_characters(user_data: Option<&mut ObjectImporter>, s: &str) {
        if let Some(importer) = user_data {
            importer._handle_object_xml_characters(s);
        }
    }
}

/// bbs_3mf.cpp:4558-4562 `inline static void check_painting_version(unsigned int loaded_version, unsigned int highest_supported_version, const std::string &error_msg)`
/// (its callers in `_handle_end_metadata` are commented out at the pin).
#[allow(dead_code)]
fn check_painting_version(loaded_version: u32, highest_supported_version: u32, error_msg: &str) -> Result<(), VersionError> {
    if loaded_version > highest_supported_version {
        return Err(VersionError::new(error_msg));
    }
    Ok(())
}

/// bbs_3mf.cpp:1862-1870 `static size_t mz_zip_read_istream(void *pOpaque, mz_uint64 file_ofs, void *pBuf, size_t n)`:
/// the read callback of an archive over a `std::istream` — seek to `file_ofs`, read up to the
/// buffer's size, the count read (0 when the seek fails).
fn mz_zip_read_istream(is: &mut dyn ReadSeek, file_ofs: u64, buf: &mut [u8]) -> usize {
    if is.seek(std::io::SeekFrom::Start(file_ofs)).is_err() {
        return 0;
    }
    let mut read = 0usize;
    while read < buf.len() {
        match is.read(&mut buf[read..]) {
            Ok(0) => break,
            Ok(n) => read += n,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(_) => break,
        }
    }
    read
}

/// `mz_zip_archive` with `m_pRead = mz_zip_read_istream`: a reader over a stream that reads at an
/// explicit offset through [`mz_zip_read_istream`] (bbs_3mf.cpp:1876-1877).
struct MzZipIstream {
    is: Box<dyn ReadSeek>,
    pos: u64,
    size: u64,
}

impl MzZipIstream {
    fn new(mut is: Box<dyn ReadSeek>) -> Option<Self> {
        // data.seekg(0, std::istream::end); mz_uint64 size = data.tellg(); data.seekg(0, std::istream::beg);
        let size = is.seek(std::io::SeekFrom::End(0)).ok()?;
        is.seek(std::io::SeekFrom::Start(0)).ok()?;
        Some(Self { is, pos: 0, size })
    }
}

impl Read for MzZipIstream {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = mz_zip_read_istream(&mut *self.is, self.pos, buf);
        self.pos += n as u64;
        Ok(n)
    }
}

impl Seek for MzZipIstream {
    fn seek(&mut self, pos: std::io::SeekFrom) -> std::io::Result<u64> {
        let p = match pos {
            std::io::SeekFrom::Start(p) => p as i64,
            std::io::SeekFrom::End(d) => self.size as i64 + d,
            std::io::SeekFrom::Current(d) => self.pos as i64 + d,
        };
        if p < 0 {
            return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, "negative position"));
        }
        self.pos = p as u64;
        Ok(self.pos)
    }
}

/// bbs_3mf.cpp:4389-4405 `void modify_import_color(int *color_arr, int size)`
pub fn modify_import_color(color_arr: &mut [i32]) {
    let mut first_bigger_than_0 = -1;
    for &c in color_arr.iter() {
        if c >= 0 {
            first_bigger_than_0 = c;
            break;
        }
    }
    if first_bigger_than_0 >= 0 {
        for c in color_arr.iter_mut() {
            if *c < 0 {
                *c = first_bigger_than_0;
            }
        }
    }
}

/// `nlohmann::json` iteration: the elements of an array, the values of an object, or the value
/// itself for a scalar (a range-for over a json scalar visits the scalar once; null visits nothing).
fn iter_json(v: &serde_json::Value) -> Vec<&serde_json::Value> {
    match v {
        serde_json::Value::Array(a) => a.iter().collect(),
        serde_json::Value::Object(o) => o.values().collect(),
        serde_json::Value::Null => Vec::new(),
        other => vec![other],
    }
}

/// `static_cast<CustomGCode::Type>(int)`
fn custom_gcode_type_from_int(t: i32) -> crate::custom_g_code::Type {
    use crate::custom_g_code::Type;
    match t {
        0 => Type::ColorChange,
        1 => Type::PausePrint,
        2 => Type::ToolChange,
        3 => Type::Template,
        4 => Type::Custom,
        _ => Type::Unknown,
    }
}

/// FilamentMixer.cpp:262-315 `parse_gradient_curve(s).empty()`: fewer than two valid points.
///
/// FIDELITY-NOTE: the same reproduction as `preset_bundle.rs` keeps privately (FilamentMixer's
/// gradient-curve functions are not exported by the crate): '|'-separated segments
/// (`std::getline`), each of 2 or 4 comma-separated fields whose first two parse with `std::stod`.
fn gradient_curve_is_empty(s: &str) -> bool {
    if s.is_empty() {
        return true;
    }
    let stod_ok = |t: &str| {
        let r = crate::config::strtod_c(t.as_bytes());
        r.consumed > 0 && !r.erange
    };
    let mut points = 0usize;
    // std::getline(ss, segment, '|'): no trailing empty segment.
    let mut segments: Vec<&str> = s.split('|').collect();
    if s.ends_with('|') {
        segments.pop();
    }
    for segment in segments {
        if segment.is_empty() {
            continue;
        }
        let fields: Vec<&str> = segment.split(',').collect();
        if fields.len() != 2 && fields.len() != 4 {
            continue;
        }
        if stod_ok(fields[0]) && stod_ok(fields[1]) {
            points += 1;
        }
    }
    points < 2
}

/// FilamentMixer.cpp:583-597 `void sanitize_mixed_gradient_curve_array(std::vector<std::string>& vals)`
fn sanitize_mixed_gradient_curve_array(vals: &mut [String]) {
    for v in vals.iter_mut() {
        if v.is_empty() {
            continue;
        }
        // parse_gradient_curve returns empty for both "empty input" and "<2 valid points";
        // we already skipped empty, so an empty result means a corrupted single-point slot.
        if gradient_curve_is_empty(v) {
            v.clear();
        }
    }
}

/// bbs_3mf.cpp:10316-10346 `std::optional<EmbossShape> read_emboss_shape(const char **attributes, unsigned int num_attributes)`
fn read_emboss_shape(attributes: &[(String, String)]) -> Option<crate::emboss_shape::EmbossShape> {
    let scale = bbs_get_attribute_value_float(attributes, SHAPE_SCALE_ATTR) as f64;
    let unhealed = bbs_get_attribute_value_int(attributes, UNHEALED_ATTR);
    let is_healed = unhealed != 1;

    let mut projection = crate::emboss_shape::EmbossProjection::default();
    projection.depth = bbs_get_attribute_value_float(attributes, DEPTH_ATTR) as f64;
    // libslic3r.h `is_approx(value, test_value, precision = EPSILON)`
    if (projection.depth - 0.0).abs() < crate::libslic3r::EPSILON {
        projection.depth = 10.0;
    }

    let use_surface = bbs_get_attribute_value_int(attributes, USE_SURFACE_ATTR);
    if use_surface == 1 {
        projection.use_surface = true;
    }

    let mut fix_tr_mat: Option<Transform3d> = None;
    let fix_tr_mat_str = bbs_get_attribute_value_string(attributes, TRANSFORM_ATTR);
    if !fix_tr_mat_str.is_empty() {
        fix_tr_mat = Some(bbs_get_transform_from_3mf_specs_string(&fix_tr_mat_str));
    }

    let file_path = bbs_get_attribute_value_string(attributes, SVG_FILE_PATH_ATTR);
    let file_path_3mf = bbs_get_attribute_value_string(attributes, SVG_FILE_PATH_IN_3MF_ATTR);

    // MayBe: store also shapes to not store svg
    // But be carefull curve will be lost -> scale will not change sampling
    // shapes could be loaded from SVG
    let mut es = crate::emboss_shape::EmbossShape::default();
    // final shape could be calculated from shapes
    es.final_shape.is_healed = is_healed;
    es.scale = scale;
    es.projection = projection;
    es.fix_3mf_tr = fix_tr_mat.map(|m| crate::geometry::Transform3D { matrix: <[f64; 16]>::try_from(m.as_slice()).unwrap() });
    let mut svg = crate::emboss_shape::SvgFile::default();
    svg.path = file_path;
    svg.path_in_3mf = file_path_3mf;
    es.svg_file = Some(svg);
    Some(es)
}

/// `CutConnectorType(int)` (Model.hpp:248-255, 0..=Undef).
fn cut_connector_type_from_int(t: i32) -> crate::model::CutConnectorType {
    use crate::model::CutConnectorType;
    match t {
        0 => CutConnectorType::Plug,
        1 => CutConnectorType::Dowel,
        2 => CutConnectorType::Snap,
        3 => CutConnectorType::Thread,
        _ => CutConnectorType::Undef,
    }
}

/// `std::stoi(s)`: leading white space, a sign, decimal digits; an error without a digit or
/// beyond int (`std::invalid_argument` / `std::out_of_range`).
fn std_stoi(s: &str) -> Result<i32, String> {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && (b[i] == b' ' || (b'\t'..=b'\r').contains(&b[i])) {
        i += 1;
    }
    let mut neg = false;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        neg = b[i] == b'-';
        i += 1;
    }
    let start = i;
    let mut v: i64 = 0;
    while i < b.len() && b[i].is_ascii_digit() {
        v = v.saturating_mul(10).saturating_add((b[i] - b'0') as i64);
        i += 1;
    }
    if i == start {
        return Err("stoi".to_string());
    }
    let v = if neg { -v } else { v };
    if v < i32::MIN as i64 || v > i32::MAX as i64 {
        return Err("stoi".to_string());
    }
    Ok(v as i32)
}

/// libslic3r_version.h `SLIC3R_APP_KEY` (version.inc `set(SLIC3R_APP_KEY "BambuStudio")`).
const SLIC3R_APP_KEY: &str = "BambuStudio";
/// `std::numeric_limits<float>::max_digits10`
const FLOAT_MAX_DIGITS10: usize = 9;
/// `std::numeric_limits<double>::max_digits10`
const DOUBLE_MAX_DIGITS10: usize = 17;

/// bbs_3mf.cpp:7537-7546 `static void reset_stream(std::stringstream &stream)`: empties the
/// stream and sets its precision to `max_digits10` of float, the precision the callers' numbers
/// are then written with (`stream_double(v, FLOAT_MAX_DIGITS10)`).
///
/// https://en.cppreference.com/w/cpp/types/numeric_limits/max_digits10
/// Conversion of a floating-point value to text and back is exact as long as at least max_digits10
/// were used (9 for float, 17 for double). The default value of std::stream precision is 6 digits.
fn reset_stream(stream: &mut String) -> usize {
    stream.clear();
    FLOAT_MAX_DIGITS10
}

// ===========================================================================
// bbs_3mf.cpp:6576-6720 class _BBS_3MF_Exporter
// ===========================================================================

/// bbs_3mf.cpp:6578-6592 `struct BuildItem`
#[derive(Debug, Clone)]
struct BuildItem {
    path: String,
    id: u32,
    transform: Transform3d,
    printable: bool,
}

/// bbs_3mf.cpp:6612-6620 `struct ObjectData`. `ModelObject const * object` is the object index;
/// `VolumeToObjectIDMap volumes_objectID` (keyed by `const ModelVolume*`) is keyed by the volume
/// index of that object.
#[derive(Debug, Clone)]
struct ObjectData {
    object: usize,
    backup_id: i32,
    object_id: i32,
    sub_path: String,
    share_mesh: bool,
    volumes_object_id: BTreeMap<usize, i32>,
}

impl ObjectData {
    fn new(object: usize, backup_id: i32) -> Self {
        Self { object, backup_id, object_id: 0, sub_path: String::new(), share_mesh: false, volumes_object_id: BTreeMap::new() }
    }
}

/// bbs_3mf.cpp:6623 `typedef std::map<ModelObject const *, ObjectData> ObjectToObjectDataMap;`
///
/// FIDELITY-NOTE: the C++ map is ordered by the address of the objects, which follows the order
/// they were allocated in (the model order for a model loaded or built in one go); the port
/// orders by the object index.
type ObjectToObjectDataMap = BTreeMap<usize, ObjectData>;

/// bbs_3mf.cpp:6576 `class _BBS_3MF_Exporter : public _BBS_3MF_Base`
#[allow(non_camel_case_types)]
pub struct _BBS_3MF_Exporter {
    base: Bbs3mfBase,
    m_fullpath_sources: bool,
    m_zip64: bool,
    // save with Production Extention
    m_production_ext: bool,
    // not save mesh and other big static contents
    m_skip_static: bool,
    // the object save is from backup store
    m_from_backup_save: bool,
    // save object per file with Production Extention
    m_split_model: bool,
    // whether to save gcode for normal save
    m_save_gcode: bool,
    // skip model when exporting .gcode.3mf
    m_skip_model: bool,
    // skip normal axuiliary files
    m_skip_auxiliary: bool,
    // whether to use loaded id for identify_id
    m_use_loaded_id: bool,
    // whether to share mesh between objects
    m_share_mesh: bool,
    // stamp qualified_assembly into assembly_step.json
    m_qualified_assembly: bool,
    m_thumbnail_middle: String,
    m_thumbnail_small: String,
    /// `std::map<void const *, std::pair<ObjectData*, ModelVolume const *>> m_shared_meshes`:
    /// the mesh address to (object index, volume index)
    m_shared_meshes: BTreeMap<usize, (usize, usize)>,
    /// `std::map<ModelVolume const *, std::pair<std::string, int>> m_volume_paths`: keyed by
    /// (object index, volume index)
    m_volume_paths: BTreeMap<(usize, usize), (String, i32)>,
}

impl Default for _BBS_3MF_Exporter {
    fn default() -> Self {
        Self::new()
    }
}

impl _BBS_3MF_Exporter {
    pub fn new() -> Self {
        _BBS_3MF_Exporter {
            base: Bbs3mfBase::new(),
            m_fullpath_sources: false,
            m_zip64: true,
            m_production_ext: false,
            m_skip_static: false,
            m_from_backup_save: false,
            m_split_model: false,
            m_save_gcode: false,
            m_skip_model: false,
            m_skip_auxiliary: false,
            m_use_loaded_id: false,
            m_share_mesh: false,
            m_qualified_assembly: false,
            m_thumbnail_middle: PRINTER_THUMBNAIL_MIDDLE_FILE.to_string(),
            m_thumbnail_small: PRINTER_THUMBNAIL_SMALL_FILE.to_string(),
            m_shared_meshes: BTreeMap::new(),
            m_volume_paths: BTreeMap::new(),
        }
    }

    fn add_error(&self, error: impl Into<String>) {
        self.base.add_error(error.into());
    }

    /// bbs_3mf.cpp:1068-1072 `log_errors()`
    pub fn log_errors(&self) {
        self.base.log_errors();
    }

    /// The errors recorded so far.
    pub fn errors(&self) -> Vec<String> {
        self.base.errors()
    }

    /// bbs_3mf.cpp:6707-6719 `static int convert_instance_id_to_resource_id(const Model& model, int obj_id, int instance_id)`
    pub fn convert_instance_id_to_resource_id(model: &Model, obj_id: i32, _instance_id: i32) -> i32 {
        let mut resource_id = 1i32;

        for i in 0..obj_id as usize {
            resource_id += model.objects[i].volumes.len() as i32 + 1;
        }

        resource_id += model.objects[obj_id as usize].volumes.len() as i32;

        resource_id
    }

    /// bbs_3mf.cpp:6722-6769
    pub fn save_model_to_file(&mut self, store_params: &mut StoreParams) -> Result<bool, Bbs3mfException> {
        self.base.clear_errors();
        self.m_fullpath_sources = store_params.strategy.contains(SaveStrategy::FullPathSources);
        self.m_zip64 = store_params.strategy.contains(SaveStrategy::Zip64);
        self.m_production_ext = store_params.strategy.contains(SaveStrategy::ProductionExt);

        self.m_skip_static = store_params.strategy.contains(SaveStrategy::SkipStatic);
        self.m_split_model = store_params.strategy.contains(SaveStrategy::SplitModel);
        self.m_save_gcode = store_params.strategy.contains(SaveStrategy::WithGcode);
        self.m_skip_model = store_params.strategy.contains(SaveStrategy::SkipModel);
        self.m_skip_auxiliary = store_params.strategy.contains(SaveStrategy::SkipAuxiliary);
        self.m_share_mesh = store_params.strategy.contains(SaveStrategy::ShareMesh);
        self.m_from_backup_save = store_params.strategy.contains(SaveStrategy::Backup);

        self.m_use_loaded_id = store_params.strategy.contains(SaveStrategy::UseLoadedId);
        self.m_qualified_assembly = store_params.qualified_assembly;

        let model = match store_params.model.as_mut() {
            Some(m) => m,
            None => return Ok(false),
        };
        if let Some(info) = &model.model_info {
            if let Some(v) = info.metadata_items.get("Thumbnail_Small") {
                self.m_thumbnail_small = v.clone();
            }
            if let Some(v) = info.metadata_items.get("Thumbnail_Middle") {
                self.m_thumbnail_middle = v.clone();
            }
        }
        let filename = store_params.path.clone();
        let _ = std::fs::remove_file(format!("{}.tmp", filename));

        let result = self._save_model_to_file(
            &format!("{}.tmp", filename),
            model,
            &mut store_params.plate_data_list,
            &mut store_params.project_presets,
            store_params.config.as_ref(),
            &store_params.thumbnail_data,
            &store_params.no_light_thumbnail_data,
            &store_params.top_thumbnail_data,
            &store_params.pick_thumbnail_data,
            store_params.pro_fn.as_ref(),
            &store_params.calibration_thumbnail_data,
            &store_params.id_bboxes,
            store_params.project.as_ref(),
            store_params.export_plate_idx,
        )?;
        if result {
            if let Err(ec) = std::fs::rename(format!("{}.tmp", filename), &filename) {
                self.add_error(format!("Failed to rename file: {}", ec));
                let _ = std::fs::remove_file(format!("{}.tmp", filename));
                return Ok(false);
            }
            if !store_params.strategy.contains(SaveStrategy::Silence) {
                let _ = crate::utils::save_string_file(std::path::Path::new(&format!("{}/origin.txt", model.get_backup_path())), &filename);
            }
        }
        Ok(result)
    }

    /// bbs_3mf.cpp:6771-6823 — backup mesh-only
    pub fn save_object_mesh(&mut self, temp_path: &str, model: &mut Model, object_idx: usize, obj_id: i32) -> bool {
        self.m_production_ext = true;
        self.m_from_backup_save = true;

        let filename = format!("3D/Objects/object_{}.model", obj_id);
        let filepath = format!("{}/{}", temp_path, filename);
        let filepath_tmp = format!("{}.tmp", filepath);
        let _ = std::fs::remove_file(&filepath_tmp);
        let mut archive = match open_zip_writer(&filepath_tmp) {
            Some(a) => a,
            None => {
                self.add_error(format!("Unable to open the file {}", filepath_tmp));
                log::error!("Unable to open the file\n");
                return false;
            }
        };

        let mut objects_data = ObjectToObjectDataMap::new();
        let mut data = ObjectData::new(object_idx, obj_id);
        let mut volume_count: i32 = 0;
        for (vi, _volume) in model.objects[object_idx].volumes.iter().enumerate() {
            volume_count += 1;
            data.volumes_object_id.insert(vi, (volume_count << 16) | obj_id);
        }
        objects_data.insert(object_idx, data);

        self._add_model_file_to_archive(&filename, &mut archive, model, &mut objects_data, None, None);

        if !mz_zip_writer_finalize_archive(&mut archive) {
            let _ = std::fs::remove_file(&filepath_tmp);
            return false;
        }
        let _ = std::fs::rename(&filepath_tmp, &filepath);
        true
    }

    /// bbs_3mf.cpp:6826-7314
    #[allow(clippy::too_many_arguments)]
    fn _save_model_to_file(
        &mut self,
        filename: &str,
        model: &mut Model,
        plate_data_list: &mut PlateDataPtrs,
        project_presets: &mut Vec<Preset>,
        config: Option<&DynamicPrintConfig>,
        thumbnail_data: &[ThumbnailData],
        no_light_thumbnail_data: &[ThumbnailData],
        top_thumbnail_data: &[ThumbnailData],
        pick_thumbnail_data: &[ThumbnailData],
        pro_fn: Option<&Export3mfProgressFn>,
        calibration_data: &[ThumbnailData],
        id_bboxes: &[PlateBBoxData],
        project: Option<&BBLProject>,
        export_plate_idx: i32,
    ) -> Result<bool, Bbs3mfException> {
        let mut temp_data = PackingTemporaryData::new();

        let mut cb_cancel = false;
        let progress = |stage: i32, current: i32, total: i32, cancel: &mut bool| {
            if let Some(f) = pro_fn {
                f(stage, current, total, cancel);
            }
        };

        //BBS progress point
        log::info!(
            "before open zip writer, m_skip_static {}, m_save_gcode {}, m_use_loaded_id {}",
            self.m_skip_static,
            self.m_save_gcode,
            self.m_use_loaded_id
        );
        if pro_fn.is_some() {
            progress(EXPORT_STAGE_OPEN_3MF, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        let mut archive = match open_zip_writer(filename) {
            Some(a) => a,
            None => {
                self.add_error(format!("Unable to open the file {}", filename));
                log::error!("Unable to open the file\n");
                return Ok(false);
            }
        };

        // `close_lock`: the file is removed unless the archive is finalized
        struct CloseLock<'a> {
            filename: Option<&'a str>,
        }
        impl Drop for CloseLock<'_> {
            fn drop(&mut self) {
                if let Some(f) = self.filename {
                    let _ = std::fs::remove_file(f);
                }
            }
        }
        let mut lock = CloseLock { filename: Some(filename) };

        //BBS progress point
        log::info!("before add _add_content_types_file_to_archive\n");
        if pro_fn.is_some() {
            progress(EXPORT_STAGE_CONTENT_TYPES, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        // Adds content types file ("[Content_Types].xml";).
        // The content of this file is the same for each BambuStudio 3mf.
        if !self._add_content_types_file_to_archive(&mut archive) {
            return Ok(false);
        }

        //BBS progress point
        log::info!("before add thumbnails, count {}", thumbnail_data.len());
        log::info!("top&&pick thumbnails, count {}", top_thumbnail_data.len());

        //BBS: add thumbnail for each plate
        if !self.m_skip_static {
            let mut thumbnail_status = vec![false; plate_data_list.len()];
            let mut no_light_thumbnail_status = vec![false; plate_data_list.len()];
            let mut top_thumbnail_status = vec![false; plate_data_list.len()];
            let mut pick_thumbnail_status = vec![false; plate_data_list.len()];

            if !thumbnail_data.is_empty() && thumbnail_data.len() > plate_data_list.len() {
                log::error!("thumbnail_data size {} > plate count {}", thumbnail_data.len(), plate_data_list.len());
                return Ok(false);
            }
            if !no_light_thumbnail_data.is_empty() && no_light_thumbnail_data.len() > plate_data_list.len() {
                log::error!("no_light_thumbnail_data size {} > plate count {}", no_light_thumbnail_data.len(), plate_data_list.len());
                return Ok(false);
            }
            if !top_thumbnail_data.is_empty() && top_thumbnail_data.len() > plate_data_list.len() {
                log::error!("top_thumbnail_data size {} > plate count {}", top_thumbnail_data.len(), plate_data_list.len());
                return Ok(false);
            }
            if !pick_thumbnail_data.is_empty() && pick_thumbnail_data.len() > plate_data_list.len() {
                log::error!("pick_thumbnail_data size {} > plate count {}", pick_thumbnail_data.len(), plate_data_list.len());
                return Ok(false);
            }
            if top_thumbnail_data.len() != pick_thumbnail_data.len() {
                log::error!("top_thumbnail_data size {} != pick_thumbnail_data size {}", top_thumbnail_data.len(), pick_thumbnail_data.len());
                return Ok(false);
            }

            if pro_fn.is_some() {
                progress(EXPORT_STAGE_ADD_THUMBNAILS, 0, plate_data_list.len() as i32, &mut cb_cancel);
                if cb_cancel {
                    return Ok(false);
                }
            }

            for (index, td) in thumbnail_data.iter().enumerate() {
                if td.is_valid() {
                    if !self._add_thumbnail_file_to_archive(&mut archive, td, "Metadata/plate", index as i32, true) {
                        return Ok(false);
                    }

                    log::info!("add thumbnail {}'s data into 3mf", index + 1);
                    thumbnail_status[index] = true;
                }
            }

            for (index, td) in no_light_thumbnail_data.iter().enumerate() {
                if td.is_valid() {
                    if !self._add_thumbnail_file_to_archive(&mut archive, td, "Metadata/plate_no_light", index as i32, false) {
                        return Ok(false);
                    }

                    log::info!("add no light thumbnail {}'s data into 3mf", index + 1);
                    no_light_thumbnail_status[index] = true;
                }
            }
            // Adds the file Metadata/top_i.png and Metadata/pick_i.png
            for index in 0..top_thumbnail_data.len() {
                if top_thumbnail_data[index].is_valid() {
                    log::info!("add top thumbnail {}'s data into 3mf", index + 1);
                    if !self._add_thumbnail_file_to_archive(&mut archive, &top_thumbnail_data[index], "Metadata/top", index as i32, false) {
                        return Ok(false);
                    }
                    top_thumbnail_status[index] = true;
                }

                if pick_thumbnail_data[index].is_valid() {
                    log::info!("add pick thumbnail {}'s data into 3mf", index + 1);
                    if !self._add_thumbnail_file_to_archive(&mut archive, &pick_thumbnail_data[index], "Metadata/pick", index as i32, false) {
                        return Ok(false);
                    }
                    pick_thumbnail_status[index] = true;
                }
            }

            for i in 0..plate_data_list.len() {
                let plate_data = &plate_data_list[i];
                let exists = |p: &str| !p.is_empty() && std::path::Path::new(p).exists();

                if !thumbnail_status[i] && exists(&plate_data.thumbnail_file) {
                    let dst_in_3mf = format!("Metadata/plate_{}.png", i + 1);
                    log::info!("add thumbnail {} from file {}", i + 1, plate_data.thumbnail_file);

                    if !self._add_file_to_archive(&mut archive, &dst_in_3mf, &plate_data.thumbnail_file) {
                        log::error!("add thumbnail {} from file {} failed\n", i + 1, plate_data.thumbnail_file);
                        return Ok(false);
                    }
                }

                if !no_light_thumbnail_status[i] && exists(&plate_data.no_light_thumbnail_file) {
                    let dst_in_3mf = format!("Metadata/plate_no_light_{}.png", i + 1);
                    log::info!("add no light thumbnail {} from file {}", i + 1, plate_data.no_light_thumbnail_file);

                    if !self._add_file_to_archive(&mut archive, &dst_in_3mf, &plate_data.no_light_thumbnail_file) {
                        log::error!("add no light thumbnail {} from file {} failed\n", i + 1, plate_data.no_light_thumbnail_file);
                        return Ok(false);
                    }
                }

                if !top_thumbnail_status[i] && exists(&plate_data.top_file) {
                    let dst_in_3mf = format!("Metadata/top_{}.png", i + 1);

                    log::info!("add top thumbnail {} from file {}", i + 1, plate_data.top_file);
                    if !self._add_file_to_archive(&mut archive, &dst_in_3mf, &plate_data.top_file) {
                        log::error!("add top thumbnail {} failed", i + 1);
                        return Ok(false);
                    }
                    top_thumbnail_status[i] = true;
                }

                if !pick_thumbnail_status[i] && exists(&plate_data.pick_file) {
                    let dst_in_3mf = format!("Metadata/pick_{}.png", i + 1);

                    log::info!("add pick thumbnail {} from file {}", i + 1, plate_data.pick_file);
                    if !self._add_file_to_archive(&mut archive, &dst_in_3mf, &plate_data.pick_file) {
                        log::error!("add pick thumbnail {} failed", i + 1);
                        return Ok(false);
                    }
                    pick_thumbnail_status[i] = true;
                }
            }
            if pro_fn.is_some() {
                progress(EXPORT_STAGE_ADD_THUMBNAILS, plate_data_list.len() as i32, plate_data_list.len() as i32, &mut cb_cancel);
                if cb_cancel {
                    return Ok(false);
                }
            }
        }

        log::info!("before add calibration thumbnails, count {}\n", calibration_data.len());
        //BBS add calibration thumbnail for each plate
        if !self.m_skip_static && !calibration_data.is_empty() {
            // Adds the file Metadata/calibration_p[X].png.
            for (index, cd) in calibration_data.iter().enumerate() {
                if pro_fn.is_some() {
                    progress(EXPORT_STAGE_ADD_THUMBNAILS, index as i32, calibration_data.len() as i32, &mut cb_cancel);
                    if cb_cancel {
                        return Ok(false);
                    }
                }

                if cd.is_valid() && !self._add_calibration_file_to_archive(&mut archive, cd, index as i32) {
                    return Ok(false);
                }
            }
        }

        log::info!("before add calibration boundingbox, count {}\n", id_bboxes.len());
        if !self.m_skip_static && !id_bboxes.is_empty() {
            // Adds the file Metadata/calibration_p[X].png.
            for (index, bb) in id_bboxes.iter().enumerate() {
                // BBS: save bounding box to json
                if bb.is_valid() && !self._add_bbox_file_to_archive(&mut archive, bb, index as i32) {
                    return Ok(false);
                }
            }
        }

        //BBS progress point
        log::info!("before add models\n");
        if pro_fn.is_some() {
            progress(EXPORT_STAGE_ADD_MODELS, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        // Adds model file ("3D/3dmodel.model").
        // This is the one and only file that contains all the geometry (vertices and triangles) of all ModelVolumes.
        let mut objects_data = ObjectToObjectDataMap::new();
        //if (!m_skip_model)
        {
            if !self._add_model_file_to_archive(filename, &mut archive, model, &mut objects_data, pro_fn, project) {
                return Ok(false);
            }

            // Adds layer height profile file ("Metadata/Slic3r_PE_layer_heights_profile.txt").
            // All layer height profiles of all ModelObjects are stored here, indexed by 1 based index of the ModelObject in Model.
            // The index differes from the index of an object ID of an object instance of a 3MF file!
            if !self._add_layer_height_profile_file_to_archive(&mut archive, model) {
                return Ok(false);
            }

            // Adds layer config ranges file ("Metadata/Slic3r_PE_layer_config_ranges.txt").
            // All layer height profiles of all ModelObjects are stored here, indexed by 1 based index of the ModelObject in Model.
            // The index differes from the index of an object ID of an object instance of a 3MF file!
            if !self._add_layer_config_ranges_file_to_archive(&mut archive, model) {
                return Ok(false);
            }

            if !self._add_brim_ear_points_file_to_archive(&mut archive, model) {
                return Ok(false);
            }

            // BBS progress point
            log::info!("before add custom gcodes\n");
            if pro_fn.is_some() {
                progress(EXPORT_STAGE_ADD_CUSTOM_GCODE, 0, 1, &mut cb_cancel);
                if cb_cancel {
                    return Ok(false);
                }
            }

            // Adds custom gcode per height file ("Metadata/Prusa_Slicer_custom_gcode_per_print_z.xml").
            // All custom gcode per height of whole Model are stored here
            if !self._add_custom_gcode_per_print_z_file_to_archive(&mut archive, model, config) {
                return Ok(false);
            }

            // BBS progress point
            log::info!("before add project_settings\n");
            if pro_fn.is_some() {
                progress(EXPORT_STAGE_ADD_PRINT_CONFIG, 0, 1, &mut cb_cancel);
                if cb_cancel {
                    return Ok(false);
                }
            }

            // Adds slic3r print config file ("Metadata/Slic3r_PE.config").
            // This file contains the content of FullPrintConfing / SLAFullPrintConfig.
            if let Some(config) = config {
                // BBS: change to json format
                if !self._add_project_config_file_to_archive(&mut archive, config, model) {
                    return Ok(false);
                }
            }

            // BBS progress point
            log::info!("before add project embedded settings\n");
            if pro_fn.is_some() {
                progress(EXPORT_STAGE_ADD_CONFIG_FILE, 0, 1, &mut cb_cancel);
                if cb_cancel {
                    return Ok(false);
                }
            }

            // BBS: add project config
            if !project_presets.is_empty() {
                // BBS: add project embedded preset files
                self._add_project_embedded_presets_to_archive(&mut archive, model, project_presets);

                log::info!("finished add project embedded settings, size {}\n", project_presets.len());
                if pro_fn.is_some() {
                    progress(EXPORT_STAGE_ADD_PROJECT_CONFIG, 0, 1, &mut cb_cancel);
                    if cb_cancel {
                        return Ok(false);
                    }
                }
            }
        }

        // add plate_N.gcode.md5 to file
        if !self.m_skip_static && self.m_save_gcode {
            for plate_data in plate_data_list.iter_mut() {
                if !plate_data.gcode_file.is_empty() && plate_data.is_sliced_valid && std::path::Path::new(&plate_data.gcode_file).exists() {
                    // the MD5 of the file, "%02X" per byte (bbs_3mf.cpp:7182-7197, the same loop as
                    // utils.cpp `bbl_calc_md5`)
                    let mut md5_str = String::new();
                    if crate::utils::bbl_calc_md5(&plate_data.gcode_file, &mut md5_str).is_err() {
                        return Err(Bbs3mfException(format!("boost::filesystem::file_size: {}", plate_data.gcode_file)));
                    }
                    plate_data.gcode_file_md5 = md5_str;
                    let target_file = format!("Metadata/plate_{}.gcode.md5", plate_data.plate_index + 1);
                    if !mz_zip_writer_add_mem(&mut archive, &target_file, plate_data.gcode_file_md5.as_bytes(), false) {
                        log::error!("store  gcode md5 to 3mf's {},  length {}, failed\n", target_file, plate_data.gcode_file_md5.len());
                        return Ok(false);
                    }
                }
            }
        }

        // Adds gcode files ("Metadata/plate_1.gcode, plate_2.gcode, ...)
        // Before _add_model_config_file_to_archive, because we modify plate_data
        if !self.m_skip_static && self.m_save_gcode && !self._add_gcode_file_to_archive(&mut archive, model, plate_data_list, pro_fn) {
            log::error!("_add_gcode_file_to_archive failed\n");
            return Ok(false);
        }

        // Adds slic3r model config file ("Metadata/Slic3r_PE_model.config").
        // This file contains all the attributes of all ModelObjects and their ModelVolumes (names, parameter overrides).
        // As there is just a single Indexed Triangle Set data stored per ModelObject, offsets of volumes into their respective Indexed Triangle Set data
        // is stored here as well.
        let (save_gcode, use_loaded_id) = (self.m_save_gcode, self.m_use_loaded_id);
        if !self._add_model_config_file_to_archive(&mut archive, model, plate_data_list, &objects_data, export_plate_idx, save_gcode, use_loaded_id) {
            log::error!("_add_model_config_file_to_archive failed\n");
            return Ok(false);
        }

        if !self._add_cut_information_file_to_archive(&mut archive, model) {
            log::error!("_add_cut_information_file_to_archive failed\n");
            return Ok(false);
        }

        //BBS progress point
        log::info!("before add sliced info to 3mf\n");
        if pro_fn.is_some() {
            progress(EXPORT_STAGE_ADD_SLICE_INFO, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        // Adds sliced info of plate file ("Metadata/slice_info.config")
        // This file contains all sliced info of all plates
        let config = match config {
            Some(c) => c,
            // `*config` dereferences a null pointer in the C++
            None => return Err(Bbs3mfException("store_bbs_3mf: no config".into())),
        };
        if !self._add_slice_info_config_file_to_archive(&mut archive, model, plate_data_list, &objects_data, config)? {
            log::error!("_add_slice_info_config_file_to_archive failed\n");
            return Ok(false);
        }

        if !self._add_filament_sequence_file_to_archive(&mut archive, plate_data_list) {
            log::error!("_add_filament_sequence_file_to_archive failed\n");
            return Ok(false);
        }

        if !self._add_assembly_tree_file_to_archive(&mut archive, model) {
            log::error!("_add_assembly_tree_file_to_archive failed\n");
            return Ok(false);
        }
        if !self._add_assembly_stpes_json_file_to_archive(&mut archive, model) {
            log::error!("_add_assembly_stpes_json_file_to_archive failed\n");
            return Ok(false);
        }
        if !self._add_assembly_model_file_to_archive(&mut archive, model) {
            log::error!("_add_assembly_model_file_to_archive failed\n");
            return Ok(false);
        }
        //BBS progress point
        log::info!("before add auxiliary dir to 3mf\n");
        if pro_fn.is_some() {
            progress(EXPORT_STAGE_ADD_AUXILIARIES, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        if !self.m_skip_static {
            let aux_dir = model.get_auxiliary_file_temp_path();
            if !self._add_auxiliary_dir_to_archive(&mut archive, &aux_dir, &mut temp_data) {
                log::error!("_add_auxiliary_dir_to_archive failed\n");
                return Ok(false);
            }
        }

        //BBS progress point
        log::info!("before add relation file to 3mf\n");
        if pro_fn.is_some() {
            progress(EXPORT_STAGE_ADD_RELATIONS, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        // Adds relationships file ("_rels/.rels").
        // The content of this file is the same for each BambuStudio 3mf.
        // The relationshis file contains a reference to the geometry file "3D/3dmodel.model", the name was chosen to be compatible with CURA.
        if !self._add_relationships_file_to_archive(&mut archive, "", &[], &[], temp_data.clone(), export_plate_idx) {
            log::error!("_add_relationships_file_to_archive failed\n");
            return Ok(false);
        }

        {
            // Writes the zip central directory and flushes everything staged above.
            if !mz_zip_writer_finalize_archive(&mut archive) {
                self.add_error("Unable to finalize the archive");
                log::error!("Unable to finalize the archive\n");
                return Ok(false);
            }
        }

        //BBS progress point
        log::info!("finished exporting 3mf\n");
        if pro_fn.is_some() {
            progress(EXPORT_STAGE_FINISH, 0, 1, &mut cb_cancel);
            if cb_cancel {
                return Ok(false);
            }
        }

        lock.filename = None;

        Ok(true)
    }

    /// The archive of the last [`store_bbs_3mf_to_entries`] (its entries in the order written).
    fn _add_file_to_archive(&mut self, archive: &mut MzZipWriter, path_in_zip: &str, src_file_path: &str) -> bool {
        const NOCOMP_EXTS: [&str; 6] = [".png", ".jpg", ".mp4", ".jpeg", ".zip", ".3mf"];
        let nocomp = NOCOMP_EXTS.iter().any(|ext| path_in_zip.ends_with(ext));
        // WRITE_ZIP_LANGUAGE_ENCODING: mz_zip_writer_add_file(archive, path_in_zip, encode_path(src), ...)
        let result = match std::fs::read(crate::utils::encode_path(src_file_path)) {
            Ok(data) => mz_zip_writer_add_mem(archive, path_in_zip, &data, nocomp),
            Err(_) => false,
        };
        if !result {
            self.add_error(format!("Unable to add file {} to archive", src_file_path));
            log::error!("Unable to add file {} to archive {}\n", src_file_path, path_in_zip);
        } else {
            log::info!("add file {} to archive {}\n", src_file_path, path_in_zip);
        }
        result
    }

    /// bbs_3mf.cpp:7338-7358
    fn _add_content_types_file_to_archive(&mut self, archive: &mut MzZipWriter) -> bool {
        let mut stream = String::new();
        stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        stream.push_str("<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n");
        stream.push_str(" <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n");
        stream.push_str(" <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n");
        stream.push_str(" <Default Extension=\"png\" ContentType=\"image/png\"/>\n");
        stream.push_str(" <Default Extension=\"gcode\" ContentType=\"text/x.gcode\"/>\n");
        stream.push_str("</Types>");

        if !mz_zip_writer_add_mem(archive, CONTENT_TYPES_FILE, stream.as_bytes(), false) {
            self.add_error("Unable to add content types file to archive");
            log::error!("Unable to add content types file to archive\n");
            return false;
        }

        true
    }

    /// `tdefl_write_image_to_png_file_in_memory_ex(pixels, w, h, 4, &size, level, 1)`: an RGBA
    /// PNG of the rows bottom up.
    ///
    /// FIDELITY-NOTE: the PNG is encoded by `png_read_write::encode_png` (flate2), not by tdefl:
    /// the same image, other compressed bytes.
    fn write_image_to_png_in_memory(pixels: &[u8], width: usize, height: usize) -> Option<Vec<u8>> {
        let row = width * 4;
        if pixels.len() < row * height {
            return None;
        }
        let mut flipped: Vec<u8> = Vec::with_capacity(row * height);
        for y in (0..height).rev() {
            flipped.extend_from_slice(&pixels[y * row..(y + 1) * row]);
        }
        crate::png_read_write::encode_png(width, height, crate::png_read_write::PNG_COLOR_TYPE_RGB_ALPHA, &flipped)
    }

    /// bbs_3mf.cpp:7360-7424
    fn _add_thumbnail_file_to_archive(
        &mut self,
        archive: &mut MzZipWriter,
        thumbnail_data: &ThumbnailData,
        local_path: &str,
        index: i32,
        generate_small_thumbnail: bool,
    ) -> bool {
        let mut res = false;

        if let Some(png_data) =
            Self::write_image_to_png_in_memory(&thumbnail_data.pixels, thumbnail_data.width as usize, thumbnail_data.height as usize)
        {
            let thumbnail_name = format!("{}_{}.png", local_path, index + 1);
            res = mz_zip_writer_add_mem(archive, &thumbnail_name, &png_data, true);
        }

        if !res {
            self.add_error("Unable to add thumbnail file to archive");
            log::error!("Unable to add thumbnail file to archive\n");
        }

        if generate_small_thumbnail && thumbnail_data.is_valid() {
            //generate small size of thumbnail
            let sw_small = PLATE_THUMBNAIL_SMALL_WIDTH as i32;
            let sh_small = PLATE_THUMBNAIL_SMALL_HEIGHT as i32;
            let mut small_pixels = vec![0u8; (sw_small * sh_small * 4) as usize];
            /* step width and step height */
            let sw = thumbnail_data.width as i32 / sw_small;
            let sh = thumbnail_data.height as i32 / sh_small;
            let clampped_width = sw * sw_small;
            let clampped_height = sh * sh_small;

            let w = thumbnail_data.width as i32;
            let mut i = 0;
            while i < clampped_height {
                let mut j = 0;
                while j < clampped_width {
                    let (mut r, mut g, mut b, mut a) = (0i32, 0i32, 0i32, 0i32);
                    for m in 0..sh {
                        for n in 0..sw {
                            let base = (4 * ((i + m) * w + j + n)) as usize;
                            r += thumbnail_data.pixels[base] as i32;
                            g += thumbnail_data.pixels[base + 1] as i32;
                            b += thumbnail_data.pixels[base + 2] as i32;
                            a += thumbnail_data.pixels[base + 3] as i32;
                        }
                    }
                    // std::clamp(0, avg, 255) (see std_clamp)
                    let clamp = |v: i32| -> i32 { std_clamp(0, v, 255) };
                    r = clamp(r / sw / sh);
                    g = clamp(g / sw / sh);
                    b = clamp(b / sw / sh);
                    a = clamp(a / sw / sh);
                    let idx = (4 * (i / sw * sw_small + j / sh)) as usize;
                    small_pixels[idx] = r as u8;
                    small_pixels[idx + 1] = g as u8;
                    small_pixels[idx + 2] = b as u8;
                    small_pixels[idx + 3] = a as u8;
                    j += sw;
                }
                i += sh;
            }
            let small_png_data = Self::write_image_to_png_in_memory(&small_pixels, sw_small as usize, sh_small as usize);
            if let Some(small_png_data) = small_png_data {
                let thumbnail_name = format!("{}_{}_small.png", local_path, index + 1);
                res = mz_zip_writer_add_mem(archive, &thumbnail_name, &small_png_data, true);
            }

            if !res {
                self.add_error("Unable to add small thumbnail file to archive");
                log::error!("Unable to add small thumbnail file to archive\n");
            }
        }

        res
    }

    /// bbs_3mf.cpp:7426-7444 — the body is commented out at the pin: returns false.
    fn _add_calibration_file_to_archive(&mut self, _archive: &mut MzZipWriter, _thumbnail_data: &ThumbnailData, _index: i32) -> bool {
        false
    }

    /// bbs_3mf.cpp:7446-7461
    fn _add_bbox_file_to_archive(&mut self, archive: &mut MzZipWriter, id_bboxes: &PlateBBoxData, index: i32) -> bool {
        let mut j = serde_json::Value::Null;
        id_bboxes.to_json(&mut j);
        let out = serde_json::to_string(&j).unwrap_or_default();

        let json_file_name = PATTERN_CONFIG_FILE_FORMAT.replace("%1%", &(index + 1).to_string());
        if !mz_zip_writer_add_mem(archive, &json_file_name, out.as_bytes(), false) {
            self.add_error("Unable to add json file to archive");
            log::error!("Unable to add json file to archive\n");
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:7463-7534
    fn _add_relationships_file_to_archive(
        &self,
        archive: &mut MzZipWriter,
        from: &str,
        targets: &[String],
        types: &[&str],
        data: PackingTemporaryData,
        export_plate_idx: i32,
    ) -> bool {
        use crate::utils::xml_escape;
        let mut stream = String::new();
        stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        stream.push_str("<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n");
        if from.is_empty() {
            stream.push_str(&format!(
                " <Relationship Target=\"/{}\" Id=\"rel-1\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>\n",
                MODEL_FILE
            ));

            if export_plate_idx < 0 {
                //use cover image if have
                if data._3mf_thumbnail.is_empty() {
                    stream.push_str(" <Relationship Target=\"/Metadata/plate_1.png\" Id=\"rel-2\" Type=\"http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail\"/>\n");
                } else {
                    stream.push_str(&format!(
                        " <Relationship Target=\"/{}\" Id=\"rel-2\" Type=\"http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail\"/>\n",
                        xml_escape(&data._3mf_thumbnail, false)
                    ));
                }

                if data._3mf_printer_thumbnail_middle.is_empty() {
                    stream.push_str(" <Relationship Target=\"/Metadata/plate_1.png\" Id=\"rel-4\" Type=\"http://schemas.bambulab.com/package/2021/cover-thumbnail-middle\"/>\n");
                } else {
                    stream.push_str(&format!(
                        " <Relationship Target=\"/{}\" Id=\"rel-4\" Type=\"http://schemas.bambulab.com/package/2021/cover-thumbnail-middle\"/>\n",
                        xml_escape(&data._3mf_printer_thumbnail_middle, false)
                    ));
                }

                if data._3mf_printer_thumbnail_small.is_empty() {
                    stream.push_str("<Relationship Target=\"/Metadata/plate_1_small.png\" Id=\"rel-5\" Type=\"http://schemas.bambulab.com/package/2021/cover-thumbnail-small\"/>\n");
                } else {
                    stream.push_str(&format!(
                        " <Relationship Target=\"/{}\" Id=\"rel-5\" Type=\"http://schemas.bambulab.com/package/2021/cover-thumbnail-small\"/>\n",
                        xml_escape(&data._3mf_printer_thumbnail_small, false)
                    ));
                }
            } else {
                //always use plate thumbnails
                let thumbnail_file_str = format!("Metadata/plate_{}.png", export_plate_idx + 1);
                stream.push_str(&format!(
                    " <Relationship Target=\"/{}\" Id=\"rel-2\" Type=\"http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail\"/>\n",
                    xml_escape(&thumbnail_file_str, false)
                ));

                let thumbnail_file_str = format!("Metadata/plate_{}.png", export_plate_idx + 1);
                stream.push_str(&format!(
                    " <Relationship Target=\"/{}\" Id=\"rel-4\" Type=\"http://schemas.bambulab.com/package/2021/cover-thumbnail-middle\"/>\n",
                    xml_escape(&thumbnail_file_str, false)
                ));

                let thumbnail_file_str = format!("Metadata/plate_{}_small.png", export_plate_idx + 1);
                stream.push_str(&format!(
                    " <Relationship Target=\"/{}\" Id=\"rel-5\" Type=\"http://schemas.bambulab.com/package/2021/cover-thumbnail-small\"/>\n",
                    xml_escape(&thumbnail_file_str, false)
                ));
            }
        } else if targets.is_empty() {
            return false;
        } else {
            let mut i = 0;
            for path in targets {
                for type_ in types {
                    i += 1;
                    stream.push_str(&format!(" <Relationship Target=\"/{}\" Id=\"rel-{}\" Type=\"{}\"/>\n", xml_escape(path, false), i, type_));
                }
            }
        }
        stream.push_str("</Relationships>");

        if !mz_zip_writer_add_mem(archive, if from.is_empty() { RELATIONSHIPS_FILE } else { from }, stream.as_bytes(), false) {
            self.add_error("Unable to add relationships file to archive");
            log::error!("Unable to add relationships file to archive\n");
            return false;
        }

        true
    }

    /*
     * BBS: Production Extension (SplitModel)
     *   save sub model if objects_data is not empty
     *   not collect build items in sub model
     */
    /// bbs_3mf.cpp:7553-7883
    fn _add_model_file_to_archive(
        &mut self,
        filename: &str,
        archive: &mut MzZipWriter,
        model: &mut Model,
        objects_data: &mut ObjectToObjectDataMap,
        pro_fn: Option<&Export3mfProgressFn>,
        project: Option<&BBLProject>,
    ) -> bool {
        use crate::utils::xml_escape;
        let sub_model = !objects_data.is_empty();
        let write_object = sub_model || !self.m_split_model;

        log::info!("filename {}, m_split_model {},  sub_model {}", filename, self.m_split_model, sub_model);

        // WRITE_ZIP_LANGUAGE_ENCODING
        let zip_filename = filename.to_string();
        // mz_zip_writer_add_staged_open: the entry is written by mz_zip_writer_add_staged_finish
        let mut staged = String::new();

        {
            let mut stream = String::new();
            reset_stream(&mut stream);
            stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
            stream.push_str(&format!(
                "<{} unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\" xmlns:BambuStudio=\"http://schemas.bambulab.com/package/2021\"",
                MODEL_TAG
            ));
            if self.m_production_ext {
                stream.push_str(" xmlns:p=\"http://schemas.microsoft.com/3dmanufacturing/production/2015/06\" requiredextensions=\"p\"");
            }
            stream.push_str(">\n");

            let mut origin = String::new();
            let mut name = String::new();
            let mut user_name = String::new();
            let mut user_id = String::new();
            let mut design_cover = String::new();
            let mut license = String::new();
            let mut description = String::new();
            let mut copyright = String::new();
            let mut model_id = String::new();
            let mut region_code = String::new();
            let mut profile_title = String::new();
            let mut profile_cover = String::new();
            let mut profile_description = String::new();
            if let Some(di) = &model.design_info {
                user_name = di.designer.clone();
                user_id = di.designer_user_id.clone();
                log::trace!("design_info, save_3mf found designer = {}", user_name);
                log::trace!("design_info, save_3mf found designer_user_id = {}", user_id);
            }

            if let Some(project) = project {
                model_id = project.project_model_id.clone();
                region_code = project.project_country_code.clone();
            }

            if let Some(mi) = &model.model_info {
                design_cover = mi.cover_file.clone();
                license = mi.license.clone();
                description = mi.description.clone();
                copyright = mi.copyright.clone();
                name = mi.model_name.clone();
                origin = mi.origin.clone();
                log::trace!("design_info, save_3mf found designer_cover = {}", design_cover);
            }

            if let Some(pi) = &model.profile_info {
                profile_title = pi.profile_tile.clone();
                profile_cover = pi.profile_cover.clone();
                profile_description = pi.profile_description.clone();
            }
            // remember to use metadata_item_map to store metadata info
            let mut metadata_item_map: BTreeMap<String, String> = BTreeMap::new();
            if !sub_model {
                // update metadat_items
                if let Some(mi) = &model.model_info {
                    metadata_item_map = mi.metadata_items.clone();
                }

                metadata_item_map.insert(BBL_MODEL_NAME_TAG.to_string(), xml_escape(&name, false));
                metadata_item_map.insert(BBL_ORIGIN_TAG.to_string(), xml_escape(&origin, false));
                metadata_item_map.insert(BBL_DESIGNER_TAG.to_string(), xml_escape(&user_name, false));
                metadata_item_map.insert(BBL_DESIGNER_USER_ID_TAG.to_string(), user_id.clone());
                metadata_item_map.insert(BBL_DESIGNER_COVER_FILE_TAG.to_string(), xml_escape(&design_cover, false));
                metadata_item_map.insert(BBL_DESCRIPTION_TAG.to_string(), xml_escape(&description, false));
                metadata_item_map.insert(BBL_COPYRIGHT_NORMATIVE_TAG.to_string(), xml_escape(&copyright, false));
                metadata_item_map.insert(BBL_LICENSE_TAG.to_string(), xml_escape(&license, false));
                metadata_item_map.insert(BBL_PROFILE_TITLE_TAG.to_string(), xml_escape(&profile_title, false));
                metadata_item_map.insert(BBL_PROFILE_COVER_TAG.to_string(), xml_escape(&profile_cover, false));
                metadata_item_map.insert(BBL_PROFILE_DESCRIPTION_TAG.to_string(), xml_escape(&profile_description, false));

                /* save model info */
                if !model_id.is_empty() {
                    metadata_item_map.insert(BBL_MODEL_ID_TAG.to_string(), model_id.clone());
                    metadata_item_map.insert(BBL_REGION_TAG.to_string(), region_code.clone());
                }

                let date = crate::time::utc_timestamp(crate::time::get_current_time_utc());
                // keep only the date part of the string
                let date = date[..date.len().min(10)].to_string();
                metadata_item_map.insert(BBL_CREATION_DATE_TAG.to_string(), date.clone());
                metadata_item_map.insert(BBL_MODIFICATION_TAG.to_string(), date);
                metadata_item_map.insert(BBL_APPLICATION_TAG.to_string(), format!("{}-{}", SLIC3R_APP_KEY, crate::semver::SLIC3R_VERSION));
            }
            metadata_item_map.insert(BBS_3MF_VERSION.to_string(), VERSION_BBS_3MF.to_string());

            if !model.mk_name.is_empty() {
                metadata_item_map.insert(BBL_MAKERLAB_TAG.to_string(), xml_escape(&model.mk_name, false));
                log::info!("saved mk_name {}", model.mk_name);
            }
            if !model.mk_version.is_empty() {
                metadata_item_map.insert(BBL_MAKERLAB_VERSION_TAG.to_string(), xml_escape(&model.mk_version, false));
                log::info!("saved mk_version {}", model.mk_version);
            }

            /*for ml*/
            if model.mk_name.is_empty() && !model.makerlab_name.is_empty() {
                metadata_item_map.insert(BBL_MAKERLAB_NAME.to_string(), model.makerlab_name.clone());
                metadata_item_map.insert(BBL_MAKERLAB_REGION.to_string(), model.makerlab_region.clone());
                metadata_item_map.insert(BBL_MAKERLAB_ID.to_string(), model.makerlab_id.clone());
            }

            if !model.md_name.is_empty() {
                for i in 0..model.md_name.len() {
                    log::info!("saved metadata_name {}, metadata_value {}", model.md_name[i], model.md_value[i]);
                    metadata_item_map.insert(model.md_name[i].clone(), xml_escape(&model.md_value[i], false));
                }
            }

            // store metadata info
            for (k, v) in &metadata_item_map {
                log::info!("bbs_3mf: save key= {}, value = {}", k, v);
                stream.push_str(&format!(" <{} name=\"{}\">{}</{}>\n", METADATA_TAG, k, xml_escape(v, false), METADATA_TAG));
            }

            stream.push_str(&format!(" <{}>\n", RESOURCES_TAG));
            staged.push_str(&stream);
        }

        // Instance transformations, indexed by the 3MF object ID (which is a linear serialization of all instances of all ModelObjects).
        let mut build_items: Vec<BuildItem> = Vec::new();

        // The object_id here is a one based identifier of the first instance of a ModelObject in the 3MF file, where
        // all the object instances of all ModelObjects are stored and indexed in a 1 based linear fashion.
        // Therefore the list of object_ids here may not be continuous.
        let mut object_id: u32 = 1;
        let mut object_index: u32 = 0;

        let mut cb_cancel = false;
        let mut object_paths: Vec<String> = Vec::new();
        let sub_model_object = objects_data.keys().next().cloned();
        for obj_idx in 0..model.objects.len() {
            if sub_model && Some(obj_idx) != sub_model_object {
                continue;
            }

            if let Some(f) = pro_fn {
                f(EXPORT_STAGE_ADD_MODELS, object_index as i32, model.objects.len() as i32, &mut cb_cancel);
                object_index += 1;
                if cb_cancel {
                    return false;
                }
            }

            // Index of an object in the 3MF file corresponding to the 1st instance of a ModelObject.
            let object_key: usize;

            if !sub_model {
                // For backup, use backup id as object id
                let backup_id = {
                    let objects = std::mem::take(&mut model.objects);
                    let id = model.get_object_backup_id(&objects[obj_idx]);
                    model.objects = objects;
                    id
                };
                if self.m_from_backup_save {
                    object_id = backup_id as u32;
                }
                objects_data.entry(obj_idx).or_insert_with(|| ObjectData::new(obj_idx, backup_id));
                object_key = obj_idx;

                if self.m_split_model {
                    let filename = format!("3D/Objects/object_{}.model", backup_id);
                    objects_data.get_mut(&object_key).unwrap().sub_path = format!("/{}", filename);
                    object_paths.push(filename);
                }

                let mut volume_id = object_id;
                let mut volume_count: u32 = 0;
                let nvol = model.objects[obj_idx].volumes.len();
                for vi in 0..nvol {
                    volume_count += 1;
                    if self.m_share_mesh {
                        let mesh_key = model.objects[obj_idx].volumes[vi].get_mesh_shared_ptr().map(|m| std::sync::Arc::as_ptr(m) as usize).unwrap_or(0);
                        if let Some(&(so, sv)) = self.m_shared_meshes.get(&mesh_key) {
                            let shared_volume = &model.objects[so].volumes[sv];
                            let volume = &model.objects[obj_idx].volumes[vi];
                            if shared_volume.supported_facets.equals(&volume.supported_facets)
                                && shared_volume.fuzzy_skin_facets.equals(&volume.fuzzy_skin_facets)
                                && shared_volume.seam_facets.equals(&volume.seam_facets)
                                && shared_volume.mmu_segmentation_facets.equals(&volume.mmu_segmentation_facets)
                            {
                                let data = objects_data.get(&so).unwrap();
                                let entry = (data.sub_path.clone(), *data.volumes_object_id.get(&sv).unwrap());
                                self.m_volume_paths.entry((obj_idx, vi)).or_insert(entry);
                                let object_data = objects_data.get_mut(&object_key).unwrap();
                                object_data.volumes_object_id.entry(vi).or_insert(0);
                                object_data.share_mesh = true;
                                continue;
                            }
                        }
                        self.m_shared_meshes.entry(mesh_key).or_insert((obj_idx, vi));
                    }
                    let backup_id = objects_data.get(&object_key).unwrap().backup_id;
                    if self.m_from_backup_save {
                        volume_id = (volume_count << 16) | backup_id as u32;
                    }
                    objects_data.get_mut(&object_key).unwrap().volumes_object_id.entry(vi).or_insert(volume_id as i32);
                    volume_id += 1;
                }

                if !self.m_from_backup_save {
                    object_id = volume_id;
                }
                objects_data.get_mut(&object_key).unwrap().object_id = object_id as i32;
            } else {
                object_key = *objects_data.keys().next().unwrap();
            }

            if self.m_skip_model {
                continue;
            }

            if write_object {
                // Store geometry of all ModelVolumes contained in a single ModelObject into a single 3MF indexed triangle set object.
                // object_it->second.volumes_objectID will contain the offsets of the ModelVolumes in that single indexed triangle set.
                // object_id will be increased to point to the 1st instance of the next ModelObject.
                let data = objects_data.get(&object_key).unwrap().clone();
                if !self._add_object_to_model_stream(&mut staged, model, &data) {
                    self.add_error("Unable to add object to archive");
                    log::error!("Unable to add object to archive\n");
                    return false;
                }
            }

            if sub_model {
                break;
            }

            let id = objects_data.get(&object_key).unwrap().object_id as u32;
            for instance in &model.objects[obj_idx].instances {
                let t = instance.matrix();
                // instance_id is just a 1 indexed index in build_items.
                build_items.push(BuildItem { path: String::new(), id, transform: t, printable: instance.printable });
            }

            if !self.m_from_backup_save {
                object_id += 1;
            }
        }

        {
            let mut stream = String::new();
            reset_stream(&mut stream);

            if !self.m_skip_model && !sub_model {
                for obj_idx in 0..model.objects.len() {
                    let data = objects_data.entry(obj_idx).or_insert_with(|| ObjectData::new(obj_idx, 0)).clone();
                    self._add_object_components_to_stream(&mut stream, model, &data);
                }
            }

            stream.push_str(&format!(" </{}>\n", RESOURCES_TAG));

            // Store the transformations of all the ModelInstances of all ModelObjects, indexed in a linear fashion.
            if !self._add_build_to_model_stream(&mut stream, &build_items) {
                self.add_error("Unable to add build to archive");
                log::error!("Unable to add build to archive\n");
                return false;
            }

            stream.push_str(&format!("</{}>\n", MODEL_TAG));

            staged.push_str(&stream);

            // mz_zip_writer_add_staged_finish
            if !mz_zip_writer_add_mem(archive, if sub_model { &zip_filename } else { MODEL_FILE }, staged.as_bytes(), false) {
                self.add_error("Unable to add model file to archive");
                log::error!("Unable to add model file to archive\n");
                return false;
            }
        }

        if self.m_skip_model || write_object {
            return true;
        }

        // write model rels
        self._add_relationships_file_to_archive(
            archive,
            MODEL_RELS_FILE,
            &object_paths,
            &["http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"],
            PackingTemporaryData::new(),
            -1,
        );

        if !self.m_from_backup_save {
            // FIDELITY-NOTE: one object file after the other (tbb::parallel_for in the C++, whose
            // entries reach the archive in the order the tasks finish).
            for i in 0..objects_data.len() {
                let key = i;
                let item = match objects_data.get(&key) {
                    Some(d) => d.clone(),
                    None => continue,
                };
                let mut objects_data2 = ObjectToObjectDataMap::new();
                objects_data2.insert(key, item);
                let mut heap = mz_zip_writer_init_heap();
                self._add_model_file_to_archive(&object_paths[i], &mut heap, model, &mut objects_data2, None, project);
                *objects_data.get_mut(&key).unwrap() = objects_data2.into_values().next().unwrap();
                mz_zip_writer_add_from_heap(archive, heap);
            }
        }

        true
    }

    /// bbs_3mf.cpp:7885-7909
    fn _add_object_to_model_stream(&mut self, context: &mut String, model: &Model, object_data: &ObjectData) -> bool {
        // backup: make _add_mesh_to_object_stream() reusable
        let mut flush = |buf: &mut String, force: bool| -> bool {
            if (force && !buf.is_empty()) || buf.len() >= 65536 * 16 {
                context.push_str(buf);
                buf.clear();
            }
            true
        };
        if !self._add_mesh_to_object_stream(&mut flush, model, object_data) {
            self.add_error("Unable to add mesh to archive");
            log::error!("Unable to add mesh to archive\n");
            return false;
        }

        // Move all components to main model
        //_add_object_components_to_stream(stream, object_data);

        true
    }

    /// bbs_3mf.cpp:7911-7954
    fn _add_object_components_to_stream(&self, stream: &mut String, model: &Model, object_data: &ObjectData) {
        use crate::utils::xml_escape;
        let object = &model.objects[object_data.object];

        stream.push_str(&format!("  <{} id=\"{}", OBJECT_TAG, object_data.object_id));
        if self.m_production_ext {
            stream.push_str(&format!(
                "\" {}=\"{}{}",
                PUUID_ATTR,
                hex_wrap(object_data.backup_id as u32),
                if object_data.share_mesh { OBJECT_UUID_SUFFIX2 } else { OBJECT_UUID_SUFFIX }
            ));
        }
        stream.push_str("\" type=\"model\">\n");

        stream.push_str(&format!("   <{}>\n", COMPONENTS_TAG));

        for index in 0..object.volumes.len() {
            let volume = &object.volumes[index];
            let mut volume_id = *object_data.volumes_object_id.get(&index).unwrap_or(&0);
            let mut ppath = object_data.sub_path.clone();
            if let Some((p, id)) = self.m_volume_paths.get(&(object_data.object, index)) {
                ppath = p.clone();
                volume_id = *id;
            }
            //add the transform of the volume
            if ppath.is_empty() {
                stream.push_str(&format!("    <{} objectid=\"{}", COMPONENT_TAG, volume_id));
            } else {
                stream.push_str(&format!("    <{} p:path=\"{}\" objectid=\"{}", COMPONENT_TAG, xml_escape(&ppath, false), volume_id));
            }
            if self.m_production_ext {
                stream.push_str(&format!(
                    "\" {}=\"{}{}",
                    PUUID_ATTR,
                    hex_wrap((index as u32).wrapping_add((object_data.backup_id as u32) << 16)),
                    COMPONENT_UUID_SUFFIX
                ));
            }
            let transf = volume.matrix();
            stream.push_str(&format!("\" {}=\"", TRANSFORM_ATTR));
            for c in 0..4 {
                for r in 0..3 {
                    stream.push_str(&stream_double(transf[(r, c)], FLOAT_MAX_DIGITS10));
                    if r != 2 || c != 3 {
                        stream.push(' ');
                    }
                }
            }
            stream.push_str("\"/>\n");
        }

        stream.push_str(&format!("   </{}>\n", COMPONENTS_TAG));

        stream.push_str(&format!("  </{}>\n", OBJECT_TAG));
    }

    //BBS: change volume to seperate objects
    /// bbs_3mf.cpp:7977-8207
    fn _add_mesh_to_object_stream(&mut self, flush: &mut dyn FnMut(&mut String, bool) -> bool, model: &Model, object_data: &ObjectData) -> bool {
        let mut output_buffer = String::new();

        // Round-trippable float, shortest possible: sprintf(buf, "%.9g", f)
        let format_coordinate = |f: f32| -> String { general_format(f as f64, 9) };

        let object = &model.objects[object_data.object];

        let mut _vertices_count: u32 = 0;
        for index in 0..object.volumes.len() {
            let volume = &object.volumes[index];

            let volume_id = *object_data.volumes_object_id.get(&index).unwrap_or(&0);
            if self.m_share_mesh && volume_id == 0 {
                continue;
            }

            let its = &volume.mesh().its;
            if its.vertices.is_empty() {
                self.add_error("Found invalid mesh");
                log::error!("Found invalid mesh\n");
                return false;
            }

            let type_ = if volume.type_() == ModelVolumeType::ModelPart { "model" } else { "other" };

            output_buffer.push_str("  <");
            output_buffer.push_str(OBJECT_TAG);
            output_buffer.push_str(" id=\"");
            output_buffer.push_str(&volume_id.to_string());
            if self.m_production_ext {
                output_buffer.push_str(&format!(
                    "\" {}=\"{}{}",
                    PUUID_ATTR,
                    hex_wrap((index as u32).wrapping_add((object_data.backup_id as u32) << 16)),
                    SUB_OBJECT_UUID_SUFFIX
                ));
            }
            output_buffer.push_str("\" type=\"");
            output_buffer.push_str(type_);
            output_buffer.push_str("\">\n");
            output_buffer.push_str("   <");
            output_buffer.push_str(MESH_TAG);
            output_buffer.push_str(">\n    <");
            output_buffer.push_str(VERTICES_TAG);
            output_buffer.push_str(">\n");

            _vertices_count += its.vertices.len() as u32;

            for v in &its.vertices {
                //don't save the volume's matrix into vertex data
                //add the shared mesh logic
                output_buffer.push_str("     <");
                output_buffer.push_str(VERTEX_TAG);
                output_buffer.push_str(" x=\"");
                output_buffer.push_str(&format_coordinate(v.x));
                output_buffer.push_str("\" y=\"");
                output_buffer.push_str(&format_coordinate(v.y));
                output_buffer.push_str("\" z=\"");
                output_buffer.push_str(&format_coordinate(v.z));
                output_buffer.push_str("\"/>\n");
                if !flush(&mut output_buffer, false) {
                    return false;
                }
            }

            output_buffer.push_str("    </");
            output_buffer.push_str(VERTICES_TAG);
            output_buffer.push_str(">\n    <");
            output_buffer.push_str(TRIANGLES_TAG);
            output_buffer.push_str(">\n");

            //BBS: as we stored matrix seperately, not multiplied into vertex
            //we don't need to consider this left hand case specially
            let is_left_handed = false;

            for i in 0..its.indices.len() {
                {
                    let idx = &its.indices[i];
                    output_buffer.push_str(&format!(
                        "     <{} v1=\"{}\" v2=\"{}\" v3=\"{}\"",
                        TRIANGLE_TAG,
                        idx[if is_left_handed { 2 } else { 0 }],
                        idx[1],
                        idx[if is_left_handed { 0 } else { 2 }]
                    ));
                }

                let custom_supports_data_string = volume.supported_facets.get_triangle_as_string(i as i32);
                if !custom_supports_data_string.is_empty() {
                    output_buffer.push(' ');
                    output_buffer.push_str(CUSTOM_SUPPORTS_ATTR);
                    output_buffer.push_str("=\"");
                    output_buffer.push_str(&custom_supports_data_string);
                    output_buffer.push('"');
                }

                let custom_fuzzy_skin_string = volume.fuzzy_skin_facets.get_triangle_as_string(i as i32);
                if !custom_fuzzy_skin_string.is_empty() {
                    output_buffer.push(' ');
                    output_buffer.push_str(CUSTOM_FUZZY_SKIN_ATTR);
                    output_buffer.push_str("=\"");
                    output_buffer.push_str(&custom_fuzzy_skin_string);
                    output_buffer.push('"');
                }

                let custom_seam_data_string = volume.seam_facets.get_triangle_as_string(i as i32);
                if !custom_seam_data_string.is_empty() {
                    output_buffer.push(' ');
                    output_buffer.push_str(CUSTOM_SEAM_ATTR);
                    output_buffer.push_str("=\"");
                    output_buffer.push_str(&custom_seam_data_string);
                    output_buffer.push('"');
                }

                let mmu_painting_data_string = volume.mmu_segmentation_facets.get_triangle_as_string(i as i32);
                if !mmu_painting_data_string.is_empty() {
                    output_buffer.push(' ');
                    output_buffer.push_str(MMU_SEGMENTATION_ATTR);
                    output_buffer.push_str("=\"");
                    output_buffer.push_str(&mmu_painting_data_string);
                    output_buffer.push('"');
                }

                // BBS — FIDELITY-NOTE: `its.properties` (FaceProperty) is not carried by the crate's
                // indexed_triangle_set, so no face_property attribute is written.

                output_buffer.push_str("/>\n");

                if !flush(&mut output_buffer, false) {
                    return false;
                }
            }
            output_buffer.push_str("    </");
            output_buffer.push_str(TRIANGLES_TAG);
            output_buffer.push_str(">\n   </");
            output_buffer.push_str(MESH_TAG);
            output_buffer.push_str(">\n");
            output_buffer.push_str("  </");
            output_buffer.push_str(OBJECT_TAG);
            output_buffer.push_str(">\n");
        }

        // Force flush.
        flush(&mut output_buffer, true)
    }

    /// bbs_3mf.cpp:8209-8218 `static void add_transformation(std::stringstream &stream, const Transform3d &tr)`
    /// (the stream's precision is `max_digits10` of float, set by `reset_stream`).
    pub fn add_transformation(stream: &mut String, tr: &Transform3d) {
        for c in 0..4 {
            for r in 0..3 {
                stream.push_str(&stream_double(tr[(r, c)], FLOAT_MAX_DIGITS10));
                if r != 2 || c != 3 {
                    stream.push(' ');
                }
            }
        }
    }

    /// bbs_3mf.cpp:8220-8247
    fn _add_build_to_model_stream(&self, stream: &mut String, build_items: &[BuildItem]) -> bool {
        use crate::utils::xml_escape;
        // This happens for empty projects
        if build_items.is_empty() {
            stream.push_str(&format!(" <{}/>\n", BUILD_TAG));
            return true;
        }

        stream.push_str(&format!(" <{}", BUILD_TAG));
        if self.m_production_ext {
            stream.push_str(&format!(" {}=\"{}\"", PUUID_ATTR, BUILD_UUID));
        }
        stream.push_str(">\n");

        for item in build_items {
            stream.push_str(&format!("  <{} {}=\"{}", ITEM_TAG, OBJECTID_ATTR, item.id));
            if self.m_production_ext {
                stream.push_str(&format!("\" {}=\"{}{}", PUUID_ATTR, hex_wrap(item.id), BUILD_UUID_SUFFIX));
            }
            if !item.path.is_empty() {
                stream.push_str(&format!("\" {}=\"{}", PPATH_ATTR, xml_escape(&item.path, false)));
            }
            stream.push_str(&format!("\" {}=\"", TRANSFORM_ATTR));
            Self::add_transformation(stream, &item.transform);
            stream.push_str(&format!("\" {}=\"{}\"/>\n", PRINTABLE_ATTR, stream_bool(item.printable)));
        }

        stream.push_str(&format!(" </{}>\n", BUILD_TAG));

        true
    }

    /// bbs_3mf.cpp:8249-8282
    fn _add_layer_height_profile_file_to_archive(&mut self, archive: &mut MzZipWriter, model: &Model) -> bool {
        let mut out = String::new();

        let mut count: u32 = 0;
        for object in &model.objects {
            count += 1;
            let layer_height_profile = object.layer_height_profile.get();
            if layer_height_profile.len() >= 4 && layer_height_profile.len() % 2 == 0 {
                out.push_str(&format!("object_id={}|", count));

                // Store the layer height profile as a single semicolon separated list.
                for (i, v) in layer_height_profile.iter().enumerate() {
                    if i == 0 {
                        out.push_str(&format!("{:.6}", v));
                    } else {
                        out.push_str(&format!(";{:.6}", v));
                    }
                }

                out.push('\n');
            }
        }

        if !out.is_empty() && !mz_zip_writer_add_mem(archive, BBS_LAYER_HEIGHTS_PROFILE_FILE, out.as_bytes(), false) {
            self.add_error("Unable to add layer heights profile file to archive");
            log::error!("Unable to add layer heights profile file to archive\n");
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:8284-8341
    fn _add_layer_config_ranges_file_to_archive(&mut self, archive: &mut MzZipWriter, model: &Model) -> bool {
        let mut out = String::new();
        let mut tree = PTree::default();

        let mut object_cnt: u32 = 0;
        for object in &model.objects {
            object_cnt += 1;
            let ranges = &object.layer_config_ranges;
            if !ranges.is_empty() {
                let obj_tree = tree.add("objects.object", "");

                obj_tree.put("<xmlattr>.id", &object_cnt.to_string());

                // Store the layer config ranges.
                for (range_first, config) in ranges {
                    let range_tree = obj_tree.add("range", "");

                    // store minX and maxZ
                    range_tree.put("<xmlattr>.min_z", &ptree_double(range_first.0 .0));
                    range_tree.put("<xmlattr>.max_z", &ptree_double(range_first.1 .0));

                    // store range configuration
                    use crate::config::ConfigBase;
                    for opt_key in config.keys() {
                        let value = config.get().opt_serialize(&opt_key).unwrap_or_default();
                        let opt_tree = range_tree.add("option", &value);
                        opt_tree.put("<xmlattr>.opt_key", &opt_key);
                    }
                }
            }
        }

        if !tree.empty() {
            out = ptree_write_xml(&tree);

            // Post processing("beautification") of the output string for a better preview
            out = out.replace("><object", ">\n <object");
            out = out.replace("><range", ">\n  <range");
            out = out.replace("><option", ">\n   <option");
            out = out.replace("></range>", ">\n  </range>");
            out = out.replace("></object>", ">\n </object>");
            // OR just
            out = out.replace("><", ">\n<");
        }

        if !out.is_empty() && !mz_zip_writer_add_mem(archive, LAYER_CONFIG_RANGES_FILE, out.as_bytes(), false) {
            self.add_error("Unable to add layer heights profile file to archive");
            log::error!("Unable to add layer heights profile file to archive\n");
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:8343-8375
    fn _add_brim_ear_points_file_to_archive(&mut self, archive: &mut MzZipWriter, model: &Model) -> bool {
        let mut out = String::new();

        let mut count: u32 = 0;
        for object in &model.objects {
            count += 1;
            let brim_points = &object.brim_points;
            if !brim_points.is_empty() {
                out.push_str(&format!("object_id={}|", count));

                // Store the layer height profile as a single space separated list.
                for (i, p) in brim_points.iter().enumerate() {
                    out.push_str(&format!(
                        "{}{:.6} {:.6} {:.6} {:.6} {}",
                        if i == 0 { "" } else { " " },
                        p.pos[0] as f64,
                        p.pos[1] as f64,
                        p.pos[2] as f64,
                        p.head_front_radius as f64,
                        p.volume_idx
                    ));
                }
                out.push('\n');
            }
        }

        if !out.is_empty() {
            // Adds version header at the beginning:
            out = format!("brim_points_format_version={}\n{}", BRIM_POINTS_FORMAT_VERSION, out);

            if !mz_zip_writer_add_mem(archive, BRIM_EAR_POINTS_FILE, out.as_bytes(), false) {
                self.add_error("Unable to add brim ear points file to archive");
                return false;
            }
        }
        true
    }

    /// bbs_3mf.cpp:8466-8486 — the print config as "; key = value" lines (the call site is
    /// commented out at the pin in favour of `_add_project_config_file_to_archive`).
    #[allow(dead_code)]
    fn _add_print_config_file_to_archive(&mut self, archive: &mut MzZipWriter, config: &DynamicPrintConfig) -> bool {
        use crate::config::ConfigBase;
        let mut out = format!("; {}\n\n", crate::utils::header_slic3r_generated());

        for key in config.keys() {
            if key != "compatible_printers" {
                out.push_str(&format!("; {} = {}\n", key, config.opt_serialize(&key).unwrap_or_default()));
            }
        }

        if !out.is_empty() && !mz_zip_writer_add_mem(archive, BBS_PRINT_CONFIG_FILE, out.as_bytes(), false) {
            self.add_error("Unable to add print config file to archive");
            log::error!("Unable to add print config file to archive\n");
            return false;
        }

        true
    }

    //BBS: add project config file logic for new json format
    /// bbs_3mf.cpp:8489-8497
    fn _add_project_config_file_to_archive(&mut self, archive: &mut MzZipWriter, config: &DynamicPrintConfig, model: &mut Model) -> bool {
        use crate::config::ConfigBase;
        let temp_path = model.get_backup_path();
        let temp_file = format!("{}/{}", temp_path, "_temp_1.config");
        let mut export_config = config.clone();
        crate::print_config::config_def::split_nozzle_stats_for_export(&mut export_config);
        let _ = export_config.save_to_json(&temp_file, "project_settings", "project", crate::semver::SLIC3R_VERSION);
        self._add_file_to_archive(archive, BBS_PROJECT_CONFIG_FILE, &temp_file)
    }

    //BBS: add project embedded preset files
    /// bbs_3mf.cpp:8500-8539
    fn _add_project_embedded_presets_to_archive(&mut self, archive: &mut MzZipWriter, model: &mut Model, project_presets: &mut [Preset]) -> bool {
        use crate::config::ConfigBase;
        let _out = format!("; {}\n\n", crate::utils::header_slic3r_generated());
        let mut print_count = 0;
        let mut filament_count = 0;
        let mut printer_count = 0;
        let temp_path = model.get_backup_path();

        for preset in project_presets.iter_mut() {
            preset.file = format!("{}/{}", temp_path, "_temp_1.config");
            let version = preset.version.to_string();
            let _ = preset.config.save_to_json(&preset.file, &preset.name, "project", &version);

            let dest_file;
            if preset.r#type == crate::preset::Type::TypePrint {
                dest_file = EMBEDDED_PRINT_FILE_FORMAT.replace("%1%", &(print_count + 1).to_string());
                print_count += 1;
            } else if preset.r#type == crate::preset::Type::TypeFilament {
                dest_file = EMBEDDED_FILAMENT_FILE_FORMAT.replace("%1%", &(filament_count + 1).to_string());
                filament_count += 1;
            } else if preset.r#type == crate::preset::Type::TypePrinter {
                dest_file = EMBEDDED_PRINTER_FILE_FORMAT.replace("%1%", &(printer_count + 1).to_string());
                printer_count += 1;
            } else {
                continue;
            }

            let file = preset.file.clone();
            self._add_file_to_archive(archive, &dest_file, &file);
        }

        true
    }

    /// bbs_3mf.cpp:8612-8981
    #[allow(clippy::too_many_arguments)]
    fn _add_model_config_file_to_archive(
        &mut self,
        archive: &mut MzZipWriter,
        model: &Model,
        plate_data_list: &PlateDataPtrs,
        objects_data: &ObjectToObjectDataMap,
        export_plate_idx: i32,
        save_gcode: bool,
        use_loaded_id: bool,
    ) -> bool {
        use crate::print_config::enums::{BedType, FilamentMapMode, PrintSequence};
        use crate::utils::xml_escape;

        // Store mesh transformation in full precision, as the volumes are stored transformed and they need to be transformed back
        // when loaded as accurately as possible.
        let prec = DOUBLE_MAX_DIGITS10;
        let mut stream = String::new();
        stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        stream.push_str(&format!("<{}>\n", CONFIG_TAG));

        if !self.m_skip_model {
            for (_, object_data) in objects_data {
                let obj = &model.objects[object_data.object];
                // Output of instances count added because of github #3435, currently not used by PrusaSlicer
                stream.push_str(&format!("  <{} {}=\"{}\">\n", OBJECT_TAG, ID_ATTR, object_data.object_id));

                // stores object's name
                if !obj.name.is_empty() {
                    stream.push_str(&format!("    <{} {}=\"name\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, VALUE_ATTR, xml_escape(&obj.name, false)));
                }

                //BBS: store object's module name
                if !obj.module_name.is_empty() {
                    stream.push_str(&format!(
                        "    <{} {}=\"module\" {}=\"{}\"/>\n",
                        METADATA_TAG,
                        KEY_ATTR,
                        VALUE_ATTR,
                        xml_escape(&obj.module_name, false)
                    ));
                }

                // stores object's config data
                for key in obj.config.keys() {
                    use crate::config::ConfigBase;
                    stream.push_str(&format!(
                        "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                        METADATA_TAG,
                        KEY_ATTR,
                        key,
                        VALUE_ATTR,
                        obj.config.get().opt_serialize(&key).unwrap_or_default()
                    ));
                }

                stream.push_str(&format!("    <{} {}=\"{}\"/>\n", METADATA_TAG, MESH_STAT_FACE_COUNT, obj.facets_count()));

                for (vi, volume) in obj.volumes.iter().enumerate() {
                    let it = match object_data.volumes_object_id.get(&vi) {
                        Some(v) => *v,
                        None => continue,
                    };
                    // stores volume's stable cross-model part GUID (assembly view independent model).
                    // Assign one to every model part now so the identity persists across reloads.
                    if volume.is_model_part() {
                        volume.ensure_part_guid(false);
                    }

                    // stores volume's offsets
                    stream.push_str(&format!("    <{} ", PART_TAG));
                    let mut volume_id = it;
                    if self.m_share_mesh && volume_id == 0 {
                        volume_id = self.m_volume_paths.get(&(object_data.object, vi)).map(|p| p.1).unwrap_or(0);
                    }
                    stream.push_str(&format!("{}=\"{}\" ", ID_ATTR, volume_id));

                    stream.push_str(&format!("{}=\"{}\"", SUBTYPE_ATTR, ModelVolume::type_to_string(volume.type_())));
                    if !volume.part_guid().is_empty() {
                        stream.push_str(&format!(" {}=\"{}\"", PART_GUID_KEY, xml_escape(&volume.part_guid(), false)));
                    }
                    stream.push_str(">\n");

                    // stores volume's name
                    if !volume.name.is_empty() {
                        stream.push_str(&format!(
                            "      <{} {}=\"{}\" {}=\"{}\"/>\n",
                            METADATA_TAG,
                            KEY_ATTR,
                            NAME_KEY,
                            VALUE_ATTR,
                            xml_escape(&volume.name, false)
                        ));
                    }

                    // stores volume's local matrix
                    stream.push_str(&format!("      <{} {}=\"{}\" {}=\"", METADATA_TAG, KEY_ATTR, MATRIX_KEY, VALUE_ATTR));
                    let matrix = crate::model::eigen_mul_affine(&volume.matrix(), &volume.source.transform.matrix());
                    for r in 0..4 {
                        for c in 0..4 {
                            stream.push_str(&stream_double(matrix[(r, c)], prec));
                            if r != 3 || c != 3 {
                                stream.push(' ');
                            }
                        }
                    }
                    stream.push_str("\"/>\n");

                    // stores volume's source data
                    {
                        let file_path = get_dealed_platform_path(&volume.source.input_file);
                        let input_file = xml_escape(
                            &if self.m_fullpath_sources {
                                volume.source.input_file.clone()
                            } else {
                                file_path.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default()
                            },
                            false,
                        );
                        let prefix = format!("      <{} {}=\"", METADATA_TAG, KEY_ATTR);
                        if !volume.source.input_file.is_empty() {
                            stream.push_str(&format!("{}{}\" {}=\"{}\"/>\n", prefix, SOURCE_FILE_KEY, VALUE_ATTR, input_file));
                            stream.push_str(&format!("{}{}\" {}=\"{}\"/>\n", prefix, SOURCE_OBJECT_ID_KEY, VALUE_ATTR, volume.source.object_idx));
                            stream.push_str(&format!("{}{}\" {}=\"{}\"/>\n", prefix, SOURCE_VOLUME_ID_KEY, VALUE_ATTR, volume.source.volume_idx));
                            stream.push_str(&format!(
                                "{}{}\" {}=\"{}\"/>\n",
                                prefix,
                                SOURCE_OFFSET_X_KEY,
                                VALUE_ATTR,
                                stream_double(volume.source.mesh_offset[0], prec)
                            ));
                            stream.push_str(&format!(
                                "{}{}\" {}=\"{}\"/>\n",
                                prefix,
                                SOURCE_OFFSET_Y_KEY,
                                VALUE_ATTR,
                                stream_double(volume.source.mesh_offset[1], prec)
                            ));
                            stream.push_str(&format!(
                                "{}{}\" {}=\"{}\"/>\n",
                                prefix,
                                SOURCE_OFFSET_Z_KEY,
                                VALUE_ATTR,
                                stream_double(volume.source.mesh_offset[2], prec)
                            ));
                        }
                        if volume.source.is_converted_from_inches {
                            stream.push_str(&format!("{}{}\" {}=\"1\"/>\n", prefix, SOURCE_IN_INCHES, VALUE_ATTR));
                        } else if volume.source.is_converted_from_meters {
                            stream.push_str(&format!("{}{}\" {}=\"1\"/>\n", prefix, SOURCE_IN_METERS, VALUE_ATTR));
                        }
                    }

                    // stores volume's config data
                    for key in volume.config.keys() {
                        use crate::config::ConfigBase;
                        stream.push_str(&format!(
                            "      <{} {}=\"{}\" {}=\"{}\"/>\n",
                            METADATA_TAG,
                            KEY_ATTR,
                            key,
                            VALUE_ATTR,
                            volume.config.get().opt_serialize(&key).unwrap_or_default()
                        ));
                    }

                    // bbs_3mf.cpp:8722-8724 `to_xml(stream, *es, *volume, archive, m_fullpath_sources)`
                    // FIDELITY-NOTE: the crate's ModelVolume keeps whether an emboss shape is set, not
                    // the shape, so no BambuStudioShape element is written.

                    let text_info = volume.get_text_info();
                    if !text_info.m_text.is_empty() {
                        _add_text_info_to_archive(&mut stream, text_info);
                    }

                    // stores mesh's statistics
                    let stats = volume.mesh().stats().repaired_errors;
                    stream.push_str(&format!("      <{} ", MESH_STAT_TAG));
                    stream.push_str(&format!("{}=\"{}\" ", MESH_STAT_FACE_COUNT, volume.mesh().facets_count()));
                    stream.push_str(&format!("{}=\"{}\" ", MESH_STAT_EDGES_FIXED, stats.edges_fixed));
                    stream.push_str(&format!("{}=\"{}\" ", MESH_STAT_DEGENERATED_FACETS, stats.degenerate_facets));
                    stream.push_str(&format!("{}=\"{}\" ", MESH_STAT_FACETS_REMOVED, stats.facets_removed));
                    stream.push_str(&format!("{}=\"{}\" ", MESH_STAT_FACETS_RESERVED, stats.facets_reversed));
                    stream.push_str(&format!("{}=\"{}\"/>\n", MESH_STAT_BACKWARDS_EDGES, stats.backwards_edges));

                    stream.push_str(&format!("    </{}>\n", PART_TAG));
                }

                stream.push_str(&format!("  </{}>\n", OBJECT_TAG));
            }
        }

        // `std::boolalpha` is sticky on the stream: every bool below is written as true / false

        //BBS: store plate related logic
        let mut gcode_paths: Vec<String> = Vec::new();
        for plate_data in plate_data_list.iter() {
            let instance_size = plate_data.objects_and_instances.len();

            stream.push_str(&format!("  <{}>\n", PLATE_TAG));
            //plate index
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, PLATERID_ATTR, VALUE_ATTR, plate_data.plate_index + 1));
            stream.push_str(&format!(
                "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                METADATA_TAG,
                KEY_ATTR,
                PLATER_NAME_ATTR,
                VALUE_ATTR,
                xml_escape(&plate_data.plate_name, false)
            ));
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, LOCK_ATTR, VALUE_ATTR, stream_boolalpha(plate_data.locked)));
            {
                use crate::config::ConfigOptionResolver;
                let bed_type_opt = plate_data.config.option("curr_bed_type");
                let bed_type_names = crate::config::ConfigOptionEnum::<BedType>::get_enum_names();
                if let Some(opt) = bed_type_opt {
                    let v = opt.get_int().unwrap_or(0);
                    if v >= 0 && bed_type_names.len() > v as usize {
                        stream.push_str(&format!(
                            "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                            METADATA_TAG,
                            KEY_ATTR,
                            BED_TYPE_ATTR,
                            VALUE_ATTR,
                            bed_type_names[v as usize]
                        ));
                    }
                }

                let print_sequence_opt = plate_data.config.option("print_sequence");
                let print_sequence_names = crate::config::ConfigOptionEnum::<PrintSequence>::get_enum_names();
                if let Some(opt) = print_sequence_opt {
                    let v = opt.get_int().unwrap_or(0);
                    if v >= 0 && print_sequence_names.len() > v as usize {
                        stream.push_str(&format!(
                            "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                            METADATA_TAG,
                            KEY_ATTR,
                            PRINT_SEQUENCE_ATTR,
                            VALUE_ATTR,
                            print_sequence_names[v as usize]
                        ));
                    }
                }

                let ints = |key: &str| -> Option<Vec<i32>> {
                    plate_data.config.option(key).and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionInts>()).map(|o| o.values.clone())
                };
                let join_space = |values: &[i32]| -> String { values.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(" ") };

                if let Some(values) = ints("first_layer_print_sequence") {
                    stream.push_str(&format!(
                        "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                        METADATA_TAG,
                        KEY_ATTR,
                        FIRST_LAYER_PRINT_SEQUENCE_ATTR,
                        VALUE_ATTR,
                        join_space(&values)
                    ));
                }

                if let Some(values) = ints("other_layers_print_sequence") {
                    stream.push_str(&format!(
                        "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                        METADATA_TAG,
                        KEY_ATTR,
                        OTHER_LAYERS_PRINT_SEQUENCE_ATTR,
                        VALUE_ATTR,
                        join_space(&values)
                    ));
                }

                if let Some(opt) = plate_data
                    .config
                    .option("other_layers_print_sequence_nums")
                    .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionInt>())
                {
                    stream.push_str(&format!(
                        "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                        METADATA_TAG,
                        KEY_ATTR,
                        OTHER_LAYERS_PRINT_SEQUENCE_NUMS_ATTR,
                        VALUE_ATTR,
                        opt.value
                    ));
                }

                if let Some(spiral_mode_opt) = plate_data.config.option("spiral_mode") {
                    stream.push_str(&format!(
                        "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                        METADATA_TAG,
                        KEY_ATTR,
                        SPIRAL_VASE_MODE,
                        VALUE_ATTR,
                        stream_boolalpha(spiral_mode_opt.get_bool().unwrap_or(false))
                    ));
                }

                //filament map related
                let filament_map_mode_opt = plate_data.config.option("filament_map_mode");
                let filament_map_mode_names = crate::config::ConfigOptionEnum::<FilamentMapMode>::get_enum_names();
                if let Some(opt) = filament_map_mode_opt {
                    let v = opt.get_int().unwrap_or(0);
                    if v >= 0 && filament_map_mode_names.len() > v as usize {
                        stream.push_str(&format!(
                            "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                            METADATA_TAG,
                            KEY_ATTR,
                            FILAMENT_MAP_MODE_ATTR,
                            VALUE_ATTR,
                            filament_map_mode_names[v as usize]
                        ));
                    }
                }

                // filament map override global settings only when group mode overrides the global settings
                if filament_map_mode_opt.is_some() {
                    if let Some(values) = ints("filament_map") {
                        stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, FILAMENT_MAP_ATTR, VALUE_ATTR, join_space(&values)));
                    }
                }

                if filament_map_mode_opt.is_some() {
                    if let Some(values) = ints("filament_volume_map") {
                        stream.push_str(&format!(
                            "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                            METADATA_TAG,
                            KEY_ATTR,
                            FILAMENT_VOL_MAP_ATTR,
                            VALUE_ATTR,
                            join_space(&values)
                        ));
                    }
                }
            }

            if save_gcode {
                stream.push_str(&format!(
                    "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                    METADATA_TAG,
                    KEY_ATTR,
                    GCODE_FILE_ATTR,
                    VALUE_ATTR,
                    xml_escape(&plate_data.gcode_file, false)
                ));
            }
            if !plate_data.gcode_file.is_empty() {
                gcode_paths.push(plate_data.gcode_file.clone());
            }
            let exists = |p: &str| !p.is_empty() && std::path::Path::new(p).exists();
            if plate_data.plate_thumbnail.is_valid() || exists(&plate_data.thumbnail_file) {
                let thumbnail_file_in_3mf = THUMBNAIL_FILE_FORMAT.replace("%1%", &(plate_data.plate_index + 1).to_string());
                stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, THUMBNAIL_FILE_ATTR, VALUE_ATTR, thumbnail_file_in_3mf));
            }

            if !plate_data.no_light_thumbnail_file.is_empty() {
                let no_light_thumbnail_file_in_3mf = NO_LIGHT_THUMBNAIL_FILE_FORMAT.replace("%1%", &(plate_data.plate_index + 1).to_string());
                stream.push_str(&format!(
                    "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                    METADATA_TAG,
                    KEY_ATTR,
                    NO_LIGHT_THUMBNAIL_FILE_ATTR,
                    VALUE_ATTR,
                    no_light_thumbnail_file_in_3mf
                ));
            }

            if !plate_data.top_file.is_empty() {
                let top_file_in_3mf = TOP_FILE_FORMAT.replace("%1%", &(plate_data.plate_index + 1).to_string());
                stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, TOP_FILE_ATTR, VALUE_ATTR, top_file_in_3mf));
            }

            if !plate_data.pick_file.is_empty() {
                let pick_file_in_3mf = PICK_FILE_FORMAT.replace("%1%", &(plate_data.plate_index + 1).to_string());
                stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, PICK_FILE_ATTR, VALUE_ATTR, pick_file_in_3mf));
            }

            if !plate_data.pattern_bbox_file.is_empty() {
                let pattern_bbox_file_in_3mf = PATTERN_CONFIG_FILE_FORMAT.replace("%1%", &(plate_data.plate_index + 1).to_string());
                stream.push_str(&format!(
                    "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                    METADATA_TAG,
                    KEY_ATTR,
                    PATTERN_BBOX_FILE_ATTR,
                    VALUE_ATTR,
                    pattern_bbox_file_in_3mf
                ));
            }

            if !self.m_skip_model && instance_size > 0 {
                for j in 0..instance_size {
                    stream.push_str(&format!("    <{}>\n", INSTANCE_TAG));
                    let mut obj_id = plate_data.objects_and_instances[j].0;
                    let inst_id = plate_data.objects_and_instances[j].1;
                    let mut identify_id: i32 = 0;
                    let mut obj: Option<usize> = None;
                    if obj_id < 0 || obj_id as usize >= model.objects.len() {
                        log::warn!("invalid object id {}\n", obj_id);
                    } else {
                        obj = Some(obj_id as usize);
                    }

                    match obj {
                        Some(o) if inst_id < 0 || inst_id as usize >= model.objects[o].instances.len() => {
                            log::warn!("invalid instance id {}\n", inst_id);
                        }
                        Some(o) => {
                            let inst = &model.objects[o].instances[inst_id as usize];
                            if use_loaded_id && inst.loaded_id > 0 {
                                identify_id = inst.loaded_id as i32;
                                if identify_id as u32 & 0xFF00_0000 != 0 {
                                    log::info!("identify_id {}, too big, limit the high bits to 0\n", identify_id);
                                    identify_id &= 0x00FF_FFFF;
                                }
                            } else {
                                identify_id = inst.id().id as i32;
                            }
                        }
                        None => {}
                    }
                    // `objects_data.find(obj)->second.object_id` (the C++ dereferences `end()` for an invalid object)
                    obj_id = obj.and_then(|o| objects_data.get(&o)).map(|d| d.object_id).unwrap_or(0);

                    stream.push_str(&format!("      <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, OBJECT_ID_ATTR, VALUE_ATTR, obj_id));
                    stream.push_str(&format!("      <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, INSTANCEID_ATTR, VALUE_ATTR, inst_id));
                    stream.push_str(&format!("      <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, IDENTIFYID_ATTR, VALUE_ATTR, identify_id));
                    stream.push_str(&format!("    </{}>\n", INSTANCE_TAG));
                }
            }
            stream.push_str(&format!("  </{}>\n", PLATE_TAG));
        }

        // write model rels
        if save_gcode {
            self._add_relationships_file_to_archive(
                archive,
                BBS_MODEL_CONFIG_RELS_FILE,
                &gcode_paths,
                &["http://schemas.bambulab.com/package/2021/gcode"],
                PackingTemporaryData::new(),
                export_plate_idx,
            );
        }

        if !self.m_skip_model {
            //BBS: store assemble related info
            stream.push_str(&format!("  <{}>\n", ASSEMBLE_TAG));
            for (_, object_data) in objects_data {
                let obj = &model.objects[object_data.object];
                for instance_idx in 0..obj.instances.len() {
                    if obj.instances[instance_idx].is_assemble_initialized() {
                        stream.push_str(&format!("   <{} {}=\"{}\" ", ASSEMBLE_ITEM_TAG, OBJECT_ID_ATTR, object_data.object_id));
                        stream.push_str(&format!("{}=\"{}\" {}=\"", INSTANCEID_ATTR, instance_idx, TRANSFORM_ATTR));
                        let assemble_trans = obj.instances[instance_idx].get_assemble_transformation().matrix();
                        for c in 0..4 {
                            for r in 0..3 {
                                stream.push_str(&stream_double(assemble_trans[(r, c)], prec));
                                if r != 2 || c != 3 {
                                    stream.push(' ');
                                }
                            }
                        }

                        stream.push_str("\" ");

                        stream.push_str(&format!("{}=\"", OFFSET_ATTR));
                        let ofs2ass = obj.instances[instance_idx].get_offset_to_assembly();
                        stream.push_str(&format!(
                            "{} {} {}",
                            stream_double(ofs2ass[0], prec),
                            stream_double(ofs2ass[1], prec),
                            stream_double(ofs2ass[2], prec)
                        ));
                        stream.push_str("\" />\n");
                    }
                }
                // BBS: per-volume assemble transformation. Stored as additional <assemble_item>
                for (volume_idx, mv) in obj.volumes.iter().enumerate() {
                    if !mv.is_assemble_initialized() {
                        continue;
                    }
                    stream.push_str(&format!("   <{} {}=\"{}\" ", ASSEMBLE_ITEM_TAG, OBJECT_ID_ATTR, object_data.object_id));
                    stream.push_str(&format!("{}=\"{}\" {}=\"", VOLUMEID_ATTR, volume_idx, TRANSFORM_ATTR));
                    let volume_assemble_trans = mv.get_assemble_transformation().matrix();
                    for c in 0..4 {
                        for r in 0..3 {
                            stream.push_str(&stream_double(volume_assemble_trans[(r, c)], prec));
                            if r != 2 || c != 3 {
                                stream.push(' ');
                            }
                        }
                    }
                    stream.push_str("\" />\n");
                }
            }
            stream.push_str(&format!("  </{}>\n", ASSEMBLE_TAG));
        }

        stream.push_str(&format!("</{}>\n", CONFIG_TAG));

        if !mz_zip_writer_add_mem(archive, BBS_MODEL_CONFIG_FILE, stream.as_bytes(), false) {
            log::error!("Unable to add model config file to archive\n");
            self.add_error("Unable to add model config file to archive");
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:8983-9044
    fn _add_cut_information_file_to_archive(&mut self, archive: &mut MzZipWriter, model: &Model) -> bool {
        let mut out = String::new();
        let mut tree = PTree::default();

        let mut object_cnt: u32 = 0;
        for object in &model.objects {
            object_cnt += 1;
            let obj_tree = tree.add("objects.object", "");

            obj_tree.put("<xmlattr>.id", &object_cnt.to_string());

            // Store info for cut_id
            let cut_id_tree = obj_tree.add("cut_id", "");

            // store cut_id atributes
            cut_id_tree.put("<xmlattr>.id", &object.cut_id.id().id.to_string());
            cut_id_tree.put("<xmlattr>.check_sum", &object.cut_id.check_sum().to_string());
            cut_id_tree.put("<xmlattr>.connectors_cnt", &object.cut_id.connectors_cnt().to_string());

            for (volume_idx, volume) in object.volumes.iter().enumerate() {
                if volume.is_cut_connector() {
                    let connectors_tree = obj_tree.add("connectors.connector", "");
                    connectors_tree.put("<xmlattr>.volume_id", &volume_idx.to_string());
                    connectors_tree.put("<xmlattr>.type", &(volume.cut_info.connector_type as i32).to_string());
                    connectors_tree.put("<xmlattr>.radius", &ptree_float(volume.cut_info.radius));
                    connectors_tree.put("<xmlattr>.height", &ptree_float(volume.cut_info.height));
                    connectors_tree.put("<xmlattr>.r_tolerance", &ptree_float(volume.cut_info.radius_tolerance));
                    connectors_tree.put("<xmlattr>.h_tolerance", &ptree_float(volume.cut_info.height_tolerance));
                }
            }
        }

        if !tree.empty() {
            out = ptree_write_xml(&tree);

            // Post processing("beautification") of the output string for a better preview
            out = out.replace("><object", ">\n <object");
            out = out.replace("><cut_id", ">\n  <cut_id");
            out = out.replace("></cut_id>", ">\n  </cut_id>");
            out = out.replace("><connectors", ">\n  <connectors");
            out = out.replace("></connectors>", ">\n  </connectors>");
            out = out.replace("><connector", ">\n   <connector");
            out = out.replace("></connector>", ">\n   </connector>");
            out = out.replace("></object>", ">\n </object>");
            // OR just
            out = out.replace("><", ">\n<");
        }

        if !out.is_empty() && !mz_zip_writer_add_mem(archive, CUT_INFORMATION_FILE, out.as_bytes(), false) {
            self.add_error("Unable to add cut information file to archive");
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:9046-9316
    fn _add_slice_info_config_file_to_archive(
        &mut self,
        archive: &mut MzZipWriter,
        model: &Model,
        plate_data_list: &PlateDataPtrs,
        _objects_data: &ObjectToObjectDataMap,
        config: &DynamicPrintConfig,
    ) -> Result<bool, Bbs3mfException> {
        use crate::config::{ConfigOption, ConfigOptionResolver};
        use crate::print_config::config_def::{get_ams_load_times, get_ams_time_types, get_ams_type_name, get_ams_unload_times, multi_nozzle_diameter_mode_enabled};
        use crate::utils::xml_escape;

        // Store mesh transformation in full precision, as the volumes are stored transformed and they need to be transformed back
        // when loaded as accurately as possible.
        // `std::fixed << std::setprecision(2)`: every floating point value is written with two decimals
        let fixed = |v: f64| stream_fixed(v, 2);
        let mut stream = String::new();
        stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        stream.push_str(&format!("<{}>\n", CONFIG_TAG));

        // save slice header for debug
        stream.push_str(&format!("  <{}>\n", SLICE_HEADER_TAG));
        stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", SLICE_HEADER_ITEM_TAG, KEY_ATTR, "X-BBL-Client-Type", VALUE_ATTR, "slicer"));
        stream.push_str(&format!(
            "    <{} {}=\"{}\" {}=\"{}\"/>\n",
            SLICE_HEADER_ITEM_TAG,
            KEY_ATTR,
            "X-BBL-Client-Version",
            VALUE_ATTR,
            crate::utils::convert_to_full_version(crate::semver::SLIC3R_VERSION)
        ));
        stream.push_str(&format!("  </{}>\n", SLICE_HEADER_TAG));

        for plate_data in plate_data_list.iter() {
            if !plate_data.is_sliced_valid {
                continue;
            }
            stream.push_str(&format!("  <{}>\n", PLATE_TAG));
            //plate index
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, PLATE_IDX_ATTR, VALUE_ATTR, plate_data.plate_index + 1));

            let mut timelapse_type = match config.opt_enum::<crate::print_config::enums::TimelapseType>("timelapse_type") {
                Ok(t) => t as i32,
                // the C++ dereferences a missing option
                Err(e) => return Err(Bbs3mfException(e.to_string())),
            };
            for w in &plate_data.warnings {
                if w.msg == NOT_GENERATE_TIMELAPSE {
                    timelapse_type = -1;
                    break;
                }
            }

            let mut extruder_types: Vec<i32> = Vec::new();
            if let Some(opt) = config.option("extruder_type").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionEnumsGeneric>()) {
                extruder_types = opt.values.clone();
            }
            let mut nozzle_volume_types: Vec<i32> = Vec::new();
            if let Some(opt) = config.option("nozzle_volume_type").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionEnumsGeneric>()) {
                nozzle_volume_types = opt.values.clone();
            }

            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"", METADATA_TAG, KEY_ATTR, EXTRUDER_TYPE_ATTR, VALUE_ATTR));
            add_vector(&mut stream, &extruder_types);
            stream.push_str("\"/>\n");

            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"", METADATA_TAG, KEY_ATTR, NOZZLE_VOLUME_TYPE_ATTR, VALUE_ATTR));
            add_vector(&mut stream, &nozzle_volume_types);
            stream.push_str("\"/>\n");

            let nozzle_diameter_option =
                config.option("nozzle_diameter").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionFloatsNullable>());
            let mut nozzle_diameters_str = String::new();
            if let Some(opt) = nozzle_diameter_option {
                nozzle_diameters_str = opt.serialize().unwrap_or_default();
            }
            // slice_filaments_info is ascending by slot and already carries each used filament's nozzle diameter.
            if multi_nozzle_diameter_mode_enabled(config) && nozzle_diameter_option.is_some() && !plate_data.slice_filaments_info.is_empty() {
                let diameter = plate_data.slice_filaments_info[0].nozzle_diameter;
                if diameter > 0.0 {
                    nozzle_diameters_str = crate::config::ConfigOptionFloatsNullable::with_size(nozzle_diameter_option.unwrap().values.len(), diameter)
                        .serialize()
                        .unwrap_or_default();
                }
            }

            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, PRINTER_MODEL_ID_ATTR, VALUE_ATTR, plate_data.printer_model_id));
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, NOZZLE_DIAMETERS_ATTR, VALUE_ATTR, nozzle_diameters_str));
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, TIMELAPSE_TYPE_ATTR, VALUE_ATTR, timelapse_type));
            stream.push_str(&format!(
                "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                METADATA_TAG,
                KEY_ATTR,
                SLICE_PREDICTION_ATTR,
                VALUE_ATTR,
                plate_data.get_gcode_prediction_str()
            ));
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, SLICE_WEIGHT_ATTR, VALUE_ATTR, plate_data.get_gcode_weight_str()));
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, PAUSE_COUNT_ATTR, VALUE_ATTR, plate_data.pause_printing.len()));
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, FIRST_LAYER_TIME_ATTR, VALUE_ATTR, plate_data.first_layer_time));
            // `std::boolalpha` from here on (sticky on the stream)
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, OUTSIDE_ATTR, VALUE_ATTR, stream_boolalpha(plate_data.toolpath_outside)));
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, SUPPORT_USED_ATTR, VALUE_ATTR, stream_boolalpha(plate_data.is_support_used)));
            stream.push_str(&format!(
                "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                METADATA_TAG,
                KEY_ATTR,
                LABEL_OBJECT_ENABLED_ATTR,
                VALUE_ATTR,
                stream_boolalpha(plate_data.is_label_object_enabled)
            ));
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, SUPPORT_MATERIAL_ON_WIPE_TOWER_ATTR, VALUE_ATTR, stream_boolalpha(false)));
            if let Some(g) = &plate_data.nozzle_group_result {
                stream.push_str(&format!(
                    "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                    METADATA_TAG,
                    KEY_ATTR,
                    ENABLE_FILAMENT_DYNAMIC_MAP_ATTR,
                    VALUE_ATTR,
                    stream_boolalpha(g.is_support_dynamic_nozzle_map())
                ));
            } else {
                stream.push_str(&format!("    <{} {}=\"{}\" {}=\"{}\"/>\n", METADATA_TAG, KEY_ATTR, ENABLE_FILAMENT_DYNAMIC_MAP_ATTR, VALUE_ATTR, stream_boolalpha(false)));
            }
            {
                let has_filament_switcher = if config.has("has_filament_switcher") { config.opt_bool("has_filament_switcher") } else { false };
                stream.push_str(&format!(
                    "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                    METADATA_TAG,
                    KEY_ATTR,
                    HAS_FILAMENT_SWITCHER_ATTR,
                    VALUE_ATTR,
                    stream_boolalpha(has_filament_switcher)
                ));
            }
            stream.push_str(&format!(
                "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                METADATA_TAG,
                KEY_ATTR,
                MULTI_NOZZLE_DIAMETER_MODE_ATTR,
                VALUE_ATTR,
                if multi_nozzle_diameter_mode_enabled(config) { "1" } else { "0" }
            ));
            let mut filament_maps = plate_data.filament_maps.clone();
            if filament_maps.is_empty() {
                if let Some(opt) = config.option("filament_map").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionInts>()) {
                    filament_maps = opt.values.clone();
                }
            }
            stream.push_str(&format!("    <{} {}=\"{}\" {}=\"", METADATA_TAG, KEY_ATTR, FILAMENT_MAP_ATTR, VALUE_ATTR));
            add_vector(&mut stream, &filament_maps);
            stream.push_str("\"/>\n");

            if !plate_data.limit_filament_maps.is_empty() {
                stream.push_str(&format!("    <{} {}=\"{}\" {}=\"", METADATA_TAG, KEY_ATTR, LIMIT_FILAMENT_MAP_ATTR, VALUE_ATTR));
                add_vector(&mut stream, &plate_data.limit_filament_maps);
                stream.push_str("\"/>\n");
            }

            // AMS type the preset assumes for the time estimation, written as the canonical AMS type name
            {
                let mut default_ams_type = -1;
                if let Some(opt) = config.option("default_ams_type").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionInt>()) {
                    default_ams_type = opt.value;
                }
                if default_ams_type >= 0 {
                    let default_ams_type_str = get_ams_type_name(default_ams_type);
                    stream.push_str(&format!(
                        "    <{} {}=\"{}\" {}=\"{}\"/>\n",
                        METADATA_TAG,
                        KEY_ATTR,
                        DEFAULT_AMS_TYPE_ATTR,
                        VALUE_ATTR,
                        xml_escape(&default_ams_type_str, false)
                    ));
                }
            }

            for &(obj_id, inst_id) in &plate_data.objects_and_instances {
                let mut identify_id: i32;
                if obj_id < 0 || obj_id as usize >= model.objects.len() {
                    log::warn!("invalid object id {}\n", obj_id);
                    continue;
                }
                let obj = &model.objects[obj_id as usize];

                if inst_id < 0 || inst_id as usize >= obj.instances.len() {
                    log::warn!("invalid instance id {}\n", inst_id);
                    continue;
                }
                let inst = &obj.instances[inst_id as usize];
                if !inst.printable {
                    continue;
                }
                if self.m_use_loaded_id && inst.loaded_id > 0 {
                    identify_id = inst.loaded_id as i32;
                    if identify_id as u32 & 0xFF00_0000 != 0 {
                        log::info!("identify_id {}, too big, limit the high bits to 0\n", identify_id);
                        identify_id &= 0x00FF_FFFF;
                    }
                } else {
                    identify_id = inst.id().id as i32;
                }
                let skipped = plate_data.skipped_objects.iter().any(|&s| s as i64 == identify_id as i64);
                stream.push_str(&format!(
                    "    <{} {}=\"{}\" {}=\"{}\" {}=\"{}\" />\n",
                    OBJECT_TAG,
                    IDENTIFYID_ATTR,
                    identify_id,
                    NAME_ATTR,
                    xml_escape(&obj.name, false),
                    SKIPPED_ATTR,
                    if skipped { "true" } else { "false" }
                ));
            }

            for (fi, it) in plate_data.slice_filaments_info.iter().enumerate() {
                let extra = plate_data.slice_filaments_extra.get(fi).cloned().unwrap_or_default();
                let group_id_value = join_int_list_comma(&it.group_id);
                stream.push_str(&format!(
                    "    <{} {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\"/>\n",
                    FILAMENT_TAG,
                    FILAMENT_ID_TAG,
                    it.id + 1,
                    FILAMENT_TRAY_INFO_ID_TAG,
                    it.filament_id,
                    FILAMENT_TYPE_TAG,
                    it.type_,
                    FILAMENT_COLOR_TAG,
                    it.color,
                    FILAMENT_USED_M_TAG,
                    fixed(it.used_m as f64),
                    FILAMENT_USED_G_TAG,
                    fixed(it.used_g as f64),
                    FILAMENT_NOZZLE_GROUP_ID_TAG,
                    group_id_value,
                    FILAMENT_NOZZLE_DIAMETER_TAG,
                    fixed(it.nozzle_diameter),
                    FILAMENT_NOZZLE_VOLUME_TYPE_TAG,
                    it.nozzle_volume_type,
                    FILAMENT_USED_FOR_OBJECT,
                    stream_boolalpha(it.used_for_object),
                    FILAMENT_USED_FOR_SUPPORT,
                    stream_boolalpha(it.used_for_support),
                    FILAMENT_DIRECTLY_USED_ATTR,
                    stream_boolalpha(extra.directly_used),
                    FILAMENT_TOTAL_LOAD_TIME_TAG,
                    fixed(extra.total_load_time),
                    FILAMENT_TOTAL_UNLOAD_TIME_TAG,
                    fixed(extra.total_unload_time)
                ));
            }

            // Mixed (virtual) filaments used by this plate. These are resolved to physical
            // components before g-code statistics, so they are not present in the <filament>
            // list above and are recorded separately here.
            for it in &plate_data.mixed_filaments_info {
                stream.push_str(&format!(
                    "    <{} {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\"/>\n",
                    MIXED_FILAMENT_TAG,
                    FILAMENT_ID_TAG,
                    it.id,
                    FILAMENT_TYPE_TAG,
                    it.type_,
                    FILAMENT_COLOR_TAG,
                    it.color,
                    MIXED_FILAMENT_COMPONENTS_TAG,
                    it.components
                ));
            }

            for it in &plate_data.warnings {
                stream.push_str(&format!(
                    "    <{} msg=\"{}\" level=\"{}\" error_code =\"{}\"  />\n",
                    SLICE_WARNING_TAG, it.msg, it.level, it.error_code
                ));
            }

            if let Some(g) = &plate_data.nozzle_group_result {
                // 获取使用的所有逻辑喷嘴
                let used_nozzle_list = g.get_used_nozzles_in_extruder(-1);
                for used_nozzle in &used_nozzle_list {
                    stream.push_str(&format!("    <{} {}/>\n", NOZZLE_TAG, used_nozzle.serialize()));
                }
            }

            // Per-AMS load/unload time list. Grouped with the other list tags near the end of the plate.
            // Each AMS type has its own pair of timing options; the type is reported under the
            // canonical name from get_ams_type_name. Types with both times zero are skipped.
            {
                let ams_load_times = get_ams_load_times(config);
                let ams_unload_times = get_ams_unload_times(config);

                let mut ams_items: Vec<(String, f64, f64)> = Vec::new();
                for &ams_type in get_ams_time_types().iter() {
                    let ams_idx = ams_type as usize;
                    let load_time = ams_load_times[ams_idx];
                    let unload_time = ams_unload_times[ams_idx];
                    if load_time <= 0.0 && unload_time <= 0.0 {
                        continue;
                    }
                    let name = get_ams_type_name(ams_type);
                    if name.is_empty() {
                        continue;
                    }
                    ams_items.push((name, load_time, unload_time));
                }
                if !ams_items.is_empty() {
                    stream.push_str(&format!("    <{}>\n", AMS_LIST_TAG));
                    for (name, load_time, unload_time) in &ams_items {
                        stream.push_str(&format!(
                            "      <{} {}=\"{}\" {}=\"{}\" {}=\"{}\"/>\n",
                            AMS_ITEM_TAG,
                            AMS_TYPE_ATTR,
                            xml_escape(name, false),
                            AMS_LOAD_TIME_ATTR,
                            fixed(*load_time),
                            AMS_UNLOAD_TIME_ATTR,
                            fixed(*unload_time)
                        ));
                    }
                    stream.push_str(&format!("    </{}>\n", AMS_LIST_TAG));
                }
            }

            if !plate_data.layer_filaments.is_empty() {
                stream.push_str(&format!("    <{}>\n", LAYER_FILAMENT_LISTS_TAG));
                // FIDELITY-NOTE: the C++ iterates an unordered_map (its own hash order); the port
                // writes the lists ordered by their filament sequence.
                let mut entries: Vec<(&Vec<u32>, &Vec<(i32, i32)>)> = plate_data.layer_filaments.iter().collect();
                entries.sort_by(|a, b| a.0.cmp(b.0));
                for (sequence, ranges) in entries {
                    // key
                    let mut key_stream = String::new();
                    add_vector(&mut key_stream, sequence);

                    // value
                    let mut value_stream = String::new();
                    for (i, r) in ranges.iter().enumerate() {
                        value_stream.push_str(&r.0.to_string());
                        value_stream.push(' ');
                        value_stream.push_str(&r.1.to_string());
                        if i != ranges.len() - 1 {
                            value_stream.push(',');
                        }
                    }

                    stream.push_str(&format!(
                        "      <{} filament_list=\"{}\" layer_ranges=\"{}\" />\n",
                        LAYER_FILAMENT_LIST_TAG, key_stream, value_stream
                    ));
                }
                stream.push_str(&format!("    </{}>\n", LAYER_FILAMENT_LISTS_TAG));
            }

            if !plate_data.pause_printing.is_empty() {
                stream.push_str(&format!("    <{}>\n", PAUSE_LIST_TAG));
                for (pause_index, pause_info) in plate_data.pause_printing.iter().enumerate() {
                    stream.push_str(&format!(
                        "      <{} {}=\"{}\" {}=\"{}\" {}=\"{}\" {}=\"{}\" />\n",
                        PAUSE_TAG,
                        PAUSE_INDEX_ATTR,
                        pause_index + 1,
                        PAUSE_LAYER_ATTR,
                        pause_info.layer_id,
                        PAUSE_PERCENT_ATTR,
                        pause_info.percent,
                        PAUSE_REMAINING_TIME_ATTR,
                        pause_info.remaining_time
                    ));
                }
                stream.push_str(&format!("    </{}>\n", PAUSE_LIST_TAG));
            }

            stream.push_str(&format!("  </{}>\n", PLATE_TAG));
        }
        stream.push_str(&format!("</{}>\n", CONFIG_TAG));

        if !mz_zip_writer_add_mem(archive, SLICE_INFO_CONFIG_FILE, stream.as_bytes(), false) {
            self.add_error("Unable to add model config file to archive");
            log::error!("store  slice-info to 3mf,  length {}, failed\n", stream.len());
            return Ok(false);
        }

        Ok(true)
    }

    /// bbs_3mf.cpp:9317-9379
    fn _add_gcode_file_to_archive(
        &mut self,
        archive: &mut MzZipWriter,
        _model: &Model,
        plate_data_list: &mut PlateDataPtrs,
        pro_fn: Option<&Export3mfProgressFn>,
    ) -> bool {
        let mut result = true;
        let mut cb_cancel = false;

        let mut plate_data_list2: Vec<usize> = Vec::new();
        for i in 0..plate_data_list.len() {
            if let Some(f) = pro_fn {
                f(EXPORT_STAGE_ADD_GCODE, i as i32, plate_data_list.len() as i32, &mut cb_cancel);
                if cb_cancel {
                    return false;
                }
            }

            let plate_data = &plate_data_list[i];
            if !plate_data.gcode_file.is_empty() && plate_data.is_sliced_valid && std::path::Path::new(&plate_data.gcode_file).exists() {
                plate_data_list2.push(i);
            }
        }

        // FIDELITY-NOTE: one plate after the other (tbb::parallel_for in the C++).
        for i in plate_data_list2 {
            let plate_data = &mut plate_data_list[i];
            let src_gcode_file = plate_data.gcode_file.clone();
            let gcode_in_3mf = GCODE_FILE_FORMAT.replace("%1%", &(plate_data.plate_index + 1).to_string());

            plate_data.gcode_file = gcode_in_3mf.clone();
            let mut heap = mz_zip_writer_init_heap();
            {
                if !std::path::Path::new(&src_gcode_file).exists() {
                    log::error!("Gcode is missing, filename = {}", src_gcode_file);
                    result = false;
                }
                let data = std::fs::read(&src_gcode_file).unwrap_or_default();
                mz_zip_writer_add_mem(&mut heap, &gcode_in_3mf, &data, false);
            }
            mz_zip_writer_add_from_heap(archive, heap);
            log::info!("store  {} to 3mf {}\n", src_gcode_file, gcode_in_3mf);
        }
        result
    }

    /// bbs_3mf.cpp:9381-9436
    fn _add_custom_gcode_per_print_z_file_to_archive(&mut self, archive: &mut MzZipWriter, model: &Model, config: Option<&DynamicPrintConfig>) -> bool {
        use crate::custom_g_code::{Mode, Type, MULTI_AS_SINGLE_MODE, MULTI_EXTRUDER_MODE, SINGLE_EXTRUDER_MODE};
        //BBS: add plate tree related logic
        let mut out = String::new();
        let mut has_custom_gcode = false;
        let mut tree = PTree::default();
        {
            let main_tree = tree.add("custom_gcodes_per_layer", "");
            for (plate, custom_gcodes) in &model.plates_custom_gcodes {
                has_custom_gcode = true;
                let plate_tree = main_tree.add("plate", "");
                let plate_idx_tree = plate_tree.add("plate_info", "");
                plate_idx_tree.put("<xmlattr>.id", &(plate + 1).to_string());

                // store data of custom_gcode_per_print_z
                for code in &custom_gcodes.gcodes {
                    let code_tree = plate_tree.add("layer", "");
                    code_tree.put("<xmlattr>.top_z", &ptree_double(code.print_z));
                    code_tree.put("<xmlattr>.type", &(code.gcode_type as i32).to_string());
                    code_tree.put("<xmlattr>.extruder", &code.extruder.to_string());
                    code_tree.put("<xmlattr>.color", &code.color);
                    code_tree.put("<xmlattr>.extra", &code.extra);

                    //BBS
                    let opt_string = |key: &str| -> String { config.map(|c| c.opt_string(key).clone()).unwrap_or_default() };
                    let gcode = if code.gcode_type == Type::PausePrint {
                        opt_string("machine_pause_gcode")
                    } else if code.gcode_type == Type::Template {
                        opt_string("template_custom_gcode")
                    } else if code.gcode_type == Type::ToolChange {
                        "tool_change".to_string()
                    } else {
                        code.extra.clone()
                    };
                    code_tree.put("<xmlattr>.gcode", &gcode);
                }

                let mode_tree = plate_tree.add("mode", "");
                // store mode of a custom_gcode_per_print_z
                mode_tree.put(
                    "<xmlattr>.value",
                    if custom_gcodes.mode == Mode::SingleExtruder {
                        SINGLE_EXTRUDER_MODE
                    } else if custom_gcodes.mode == Mode::MultiAsSingle {
                        MULTI_AS_SINGLE_MODE
                    } else {
                        MULTI_EXTRUDER_MODE
                    },
                );
            }
        }
        if has_custom_gcode {
            out = ptree_write_xml(&tree);

            // Post processing("beautification") of the output string
            out = out.replace("><", ">\n<");
        }

        if !out.is_empty() && !mz_zip_writer_add_mem(archive, CUSTOM_GCODE_PER_PRINT_Z_FILE, out.as_bytes(), false) {
            self.add_error("Unable to add custom Gcodes per print_z file to archive");
            log::error!("Unable to add custom Gcodes per print_z file to archive\n");
            return false;
        }

        true
    }

    /// bbs_3mf.cpp:9438-9496
    fn _add_auxiliary_dir_to_archive(&mut self, archive: &mut MzZipWriter, aux_dir: &str, data: &mut PackingTemporaryData) -> bool {
        let mut result = true;

        if aux_dir.is_empty() {
            //no accessory directories
            return result;
        }

        let dir = std::path::PathBuf::from(aux_dir);
        if !dir.exists() {
            //no accessory directories
            return result;
        }

        let mut directories: std::collections::VecDeque<std::path::PathBuf> = std::collections::VecDeque::new();
        directories.push_back(dir.clone());
        let root_dir_len = dir.to_string_lossy().len() + 1;
        //boost file access
        while let Some(front) = directories.pop_front() {
            let iterator = match std::fs::read_dir(&front) {
                Ok(it) => it,
                Err(_) => continue,
            };
            // FIDELITY-NOTE: directory_iterator's order is the file system's; the entries are
            // visited sorted by name here.
            let mut entries: Vec<std::path::PathBuf> = iterator.filter_map(|e| e.ok().map(|e| e.path())).collect();
            entries.sort();
            for dir_entry in entries {
                if dir_entry.is_dir() {
                    directories.push_back(dir_entry);
                    continue;
                }
                if dir_entry.is_file() && !self.m_skip_auxiliary {
                    let src_file = dir_entry.to_string_lossy().into_owned();
                    let mut dst_in_3mf = src_file.clone();
                    dst_in_3mf.replace_range(0..root_dir_len.min(dst_in_3mf.len()), AUXILIARY_DIR);
                    let dst_in_3mf = dst_in_3mf.replace('\\', "/");
                    if &_3MF_COVER_FILE[1..] == dst_in_3mf {
                        data._3mf_thumbnail = dst_in_3mf.clone();
                    } else if !self.m_thumbnail_small.is_empty() && self.m_thumbnail_small.get(1..) == Some(dst_in_3mf.as_str()) {
                        data._3mf_printer_thumbnail_small = dst_in_3mf.clone();
                        if self.m_thumbnail_middle == self.m_thumbnail_small {
                            data._3mf_printer_thumbnail_middle = dst_in_3mf.clone();
                        }
                    } else if !self.m_thumbnail_middle.is_empty() && self.m_thumbnail_middle.get(1..) == Some(dst_in_3mf.as_str()) {
                        data._3mf_printer_thumbnail_middle = dst_in_3mf.clone();
                    }
                    result &= self._add_file_to_archive(archive, &dst_in_3mf, &src_file);
                }
            }
        }

        result
    }

    /// bbs_3mf.cpp:9499-9537
    fn _add_filament_sequence_file_to_archive(&mut self, archive: &mut MzZipWriter, plate_data_list: &PlateDataPtrs) -> bool {
        // nlohmann::json objects keep their keys ordered (std::map)
        let mut j: BTreeMap<String, BTreeMap<String, serde_json::Value>> = BTreeMap::new();

        for (idx, plate_data) in plate_data_list.iter().enumerate() {
            let plate_idx = format!("plate_{}", idx + 1);
            let sequence: Vec<u32> = plate_data.filament_change_sequence.iter().map(|v| v.wrapping_add(1)).collect(); // to 1 based idx

            let enable_dynamic_map = plate_data.nozzle_group_result.as_ref().map(|g| g.is_support_dynamic_nozzle_map()).unwrap_or(false);
            let seq_key = if enable_dynamic_map { "filament_sequence" } else { "sequence" };
            let entry = j.entry(plate_idx).or_default();
            entry.insert(seq_key.to_string(), serde_json::json!(sequence));
            entry.insert("nozzle_sequence".to_string(), serde_json::json!(plate_data.nozzle_change_sequence));

            let optimal_assignment = plate_data.optimal_assignment.clone();
            entry.insert("optimal_assignment".to_string(), serde_json::json!(optimal_assignment));
        }

        if j.is_empty() {
            return true;
        }

        let sequence_str = serde_json::to_string(&j).unwrap_or_default();

        if !mz_zip_writer_add_mem(archive, FILAMENT_SEQUENCE_FILE, sequence_str.as_bytes(), false) {
            self.add_error("Unable to add model config file to archive");
            log::error!("store  slice-info to 3mf,  length {}, failed\n", sequence_str.len());
            return false;
        }
        true
    }

    /// bbs_3mf.cpp:9539-9553
    fn _add_assembly_tree_file_to_archive(&mut self, archive: &mut MzZipWriter, model: &Model) -> bool {
        // Project save: serialize the in-memory tree to JSON once at the I/O boundary.No other site should be calling AssemblyTreeData::to_json_string() during a session.
        let tree = model.get_assembly_tree_data();
        if tree.empty() || !crate::format::assembly_tree_json::SHOW_ORIGIN_STEP_TREE.load(std::sync::atomic::Ordering::SeqCst) {
            return true;
        }

        let tree_str = tree.to_json_string();
        if !mz_zip_writer_add_mem(archive, ASSEMBLY_TREE_FILE, tree_str.as_bytes(), false) {
            self.add_error("Unable to add assembly tree file to archive");
            log::error!("store assembly tree to 3mf, length {}, failed\n", tree_str.len());
            return false;
        }
        true
    }

    /// bbs_3mf.cpp:9555-9568
    fn _add_assembly_stpes_json_file_to_archive(&mut self, archive: &mut MzZipWriter, model: &Model) -> bool {
        let steps_json_str = model.get_assembly_steps_json_str();
        if steps_json_str.is_empty() {
            return true;
        }
        let payload = crate::format::assembly_steps_json::assembly_steps_json_with_qualified_assembly(steps_json_str, self.m_qualified_assembly);
        if !mz_zip_writer_add_mem(archive, ASSEMBLY_STEP_JSON_FILE, payload.as_bytes(), false) {
            self.add_error("Unable to add assembly.json to archive");
            log::error!("store assembly.json to 3mf, length {}, failed\n", steps_json_str.len());
            return false;
        }
        true
    }

    /// bbs_3mf.cpp:9570-9584
    fn _add_assembly_model_file_to_archive(&mut self, archive: &mut MzZipWriter, model: &Model) -> bool {
        // The independent assembly model object graph, pre-serialized by the GUI (Plater) into the prepare
        // model before export. Empty when the project never entered the assembly view.
        let assembly_model_json_str = model.get_assembly_model_json_str();
        if assembly_model_json_str.is_empty() {
            return true;
        }
        if !mz_zip_writer_add_mem(archive, ASSEMBLY_MODEL_FILE, assembly_model_json_str.as_bytes(), false) {
            self.add_error("Unable to add assembly_model.json to archive");
            log::error!("store assembly_model.json to 3mf, length {}, failed\n", assembly_model_json_str.len());
            return false;
        }
        true
    }
}

/// `std::clamp(v, lo, hi)`: `v < lo ? lo : hi < v ? hi : v`. bbs_3mf.cpp:7398-7401 calls it as
/// `std::clamp(0, r / sw / sh, 255)`, which gives the average of the bytes (never above 255).
fn std_clamp(v: i32, lo: i32, hi: i32) -> i32 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}

/// bbs_3mf.cpp:8541-8598 `void _add_text_info_to_archive(std::stringstream& stream, const TextInfo& text_info)`
fn _add_text_info_to_archive(stream: &mut String, text_info: &crate::model::TextInfo) {
    use crate::text_configuration::{HorizontalAlign, VerticalAlign};
    use crate::utils::{xml_escape, xml_escape_double_quotes_attribute_value};
    // the model config stream's precision (max_digits10 of double)
    let p = DOUBLE_MAX_DIGITS10;
    stream.push_str(&format!("      <{} ", TEXT_INFO_TAG));

    stream.push_str(&format!("{}=\"{}\" ", TEXT_ATTR, xml_escape_double_quotes_attribute_value(&text_info.m_text)));
    stream.push_str(&format!("{}=\"{}\" ", FONT_NAME_ATTR, xml_escape(&text_info.m_font_name, false)));
    stream.push_str(&format!("{}=\"{}\" ", FONT_VERSION_ATTR, text_info.m_font_version));
    stream.push_str(&format!("{}=\"{}\" ", STYLE_NAME_ATTR, xml_escape_double_quotes_attribute_value(&text_info.text_configuration.style.name)));
    stream.push_str(&format!("{}=\"{}\" ", BOLDNESS_ATTR, stream_float(text_info.text_configuration.style.prop.boldness.unwrap_or(0.0), p)));
    stream.push_str(&format!("{}=\"{}\" ", SKEW_ATTR, stream_float(text_info.text_configuration.style.prop.skew.unwrap_or(0.0), p)));

    stream.push_str(&format!("{}=\"{}\" ", FONT_INDEX_ATTR, text_info.m_curr_font_idx));

    stream.push_str(&format!("{}=\"{}\" ", FONT_SIZE_ATTR, stream_float(text_info.m_font_size, p)));
    stream.push_str(&format!("{}=\"{}\" ", THICKNESS_ATTR, stream_float(text_info.m_thickness, p)));
    stream.push_str(&format!("{}=\"{}\" ", EMBEDED_DEPTH_ATTR, stream_float(text_info.m_embeded_depth, p)));
    stream.push_str(&format!("{}=\"{}\" ", ROTATE_ANGLE_ATTR, stream_float(text_info.m_rotate_angle, p)));
    stream.push_str(&format!("{}=\"{}\" ", TEXT_GAP_ATTR, stream_float(text_info.m_text_gap, p)));
    if let Some(line_gap) = text_info.text_configuration.style.prop.line_gap {
        stream.push_str(&format!("{}=\"{}\" ", LINE_GAP_ATTR, line_gap));
    }
    {
        let h = text_info.text_configuration.style.prop.align.0;
        let mut h_name = "center";
        if h == HorizontalAlign::Left {
            h_name = "left";
        } else if h == HorizontalAlign::Right {
            h_name = "right";
        }
        stream.push_str(&format!("{}=\"{}\" ", HORIZONTAL_ALIGN_ATTR, h_name));
        let v = text_info.text_configuration.style.prop.align.1;
        let mut v_name = "middle";
        if v == VerticalAlign::Top {
            v_name = "top";
        } else if v == VerticalAlign::Bottom {
            v_name = "bottom";
        }
        stream.push_str(&format!("{}=\"{}\" ", VERTICAL_ALIGN_ATTR, v_name));
    }

    stream.push_str(&format!("{}=\"{}\" ", BOLD_ATTR, if text_info.m_bold { 1 } else { 0 }));
    stream.push_str(&format!("{}=\"{}\" ", ITALIC_ATTR, if text_info.m_italic { 1 } else { 0 }));
    let temp_version = if text_info.m_font_version.is_empty() { 0.0f32 } else { atof(&text_info.m_font_version) as f32 };
    if temp_version < 2.0f32 {
        stream.push_str(&format!("{}=\"{}\" ", SURFACE_TEXT_ATTR, if text_info.m_is_surface_text { 1 } else { 0 }));
        stream.push_str(&format!("{}=\"{}\" ", KEEP_HORIZONTAL_ATTR, if text_info.m_keep_horizontal { 1 } else { 0 }));
    } else {
        stream.push_str(&format!("{}=\"{}\" ", SURFACE_TYPE, text_info.m_surface_type));
    }

    // FIDELITY-NOTE: `TextInfo::m_rr` is not carried (see the reader): the raycast result is
    // written as its defaults (mesh -1, zero vectors).
    stream.push_str(&format!("{}=\"{}\" ", HIT_MESH_ATTR, -1));

    stream.push_str(&format!("{}=\"", HIT_POSITION_ATTR));
    add_vec3(stream, &Vec3f::zeros());
    stream.push_str("\" ");

    stream.push_str(&format!("{}=\"", HIT_NORMAL_ATTR));
    add_vec3(stream, &Vec3f::zeros());
    stream.push_str("\" ");

    stream.push_str("/>\n");
}

/// bbs_3mf.cpp:8600-8610 `boost::filesystem::path get_dealed_platform_path(std::string path_str)`
fn get_dealed_platform_path(path_str: &str) -> std::path::PathBuf {
    #[cfg(not(windows))]
    {
        let translated_input = path_str.replace('\\', "/");
        std::path::PathBuf::from(translated_input)
    }
    #[cfg(windows)]
    {
        std::path::PathBuf::from(path_str)
    }
}

// bbs_3mf.cpp:9587-9596 `static void handle_legacy_project_loaded(unsigned int version_project_file, DynamicPrintConfig& config)`
// — its only call (bbs_3mf.cpp:10026) is commented out at the pin.
#[allow(dead_code)]
fn handle_legacy_project_loaded(_version_project_file: u32, config: &mut DynamicPrintConfig) {
    use crate::config::{ConfigBase, ConfigBaseTyped, ConfigOptionResolver};
    if !config.has("brim_object_gap") {
        let elephant_foot = config
            .option("elefant_foot_compensation")
            .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionFloat>())
            .map(|o| o.value);
        if let Some(v) = elephant_foot {
            // Conversion from older PrusaSlicer which applied brim separation equal to elephant foot compensation.
            if let Ok(Some(opt_brim_separation)) = config.option_typed_mut::<crate::config::ConfigOptionFloat>("brim_object_gap", true) {
                opt_brim_separation.value = v;
            }
        }
        let _ = ConfigBase::keys(&**config);
    }
}

// bbs_3mf.cpp:9598-9992 `class _BBS_Backup_Manager` and the backup interface around it
// (save_object_mesh, delete_object_mesh, backup_soon, remove_backup, set_backup_interval,
// set_backup_callback, stop_backup, run_backup_ui_tasks, has_restore_data, put_other_changes,
// clear_other_changes, has_other_changes, SaveObjectGaurd): the GUI's background backup thread.
// FIDELITY-NOTE: not part of the command line engine; `_BBS_3MF_Exporter::save_object_mesh` (the
// mesh-only backup it calls) is ported above.

//BBS: add plate data list related logic
/// bbs_3mf.cpp:9994-10028 `bool load_bbs_3mf(const char* path, DynamicPrintConfig* config, ...)`
#[allow(clippy::too_many_arguments)]
pub fn load_bbs_3mf(
    path: &str,
    config: &mut DynamicPrintConfig,
    config_substitutions: &mut crate::config::ConfigSubstitutionContext,
    model: &mut Model,
    plate_data_list: &mut PlateDataPtrs,
    project_presets: &mut Vec<Preset>,
    is_bbl_3mf: Option<&mut bool>,
    file_version: &mut crate::semver::Semver,
    pro_fn: Option<&Import3mfProgressFn>,
    strategy: LoadStrategy,
    project: Option<&mut BBLProject>,
    plate_id: i32,
    color_group_map: Option<&mut BTreeMap<i32, Vec<String>>>,
    volume_color_data: Option<&mut VolumeColorInfoMap>,
    has_textures: Option<&mut bool>,
    has_unit: Option<&mut bool>,
) -> Result<bool, Bbs3mfException> {
    // All import should use "C" locales for number formatting.
    let mut importer = _BBS_3MF_Importer::new();
    let mut is_bbl = false;
    let want_bbl = is_bbl_3mf.is_some();
    let res = importer.load_model_from_file(
        path,
        model,
        plate_data_list,
        project_presets,
        config,
        config_substitutions,
        strategy,
        if want_bbl { Some(&mut is_bbl) } else { None },
        file_version,
        pro_fn,
        project,
        plate_id,
    );
    importer.log_errors();
    let res = res?;
    if let Some(b) = is_bbl_3mf {
        *b = is_bbl;
    }

    // If color mapping information is requested, return
    if let Some(m) = color_group_map {
        *m = importer.get_color_group_map().clone();
    }

    // If volume color data is requested, return
    if let Some(v) = volume_color_data {
        *v = importer.get_volume_color_data().clone();
    }

    if let Some(t) = has_textures {
        *t = importer.has_texture_resources() && !(want_bbl && is_bbl);
    }

    if let Some(u) = has_unit {
        *u = importer.has_unit();
    }

    //BBS: remove legacy project logic currently
    //handle_legacy_project_loaded(importer.version(), *config);
    Ok(res)
}

/// bbs_3mf.cpp:10030-10037 `std::string bbs_3mf_get_thumbnail(const char *path)`
pub fn bbs_3mf_get_thumbnail(path: &str) -> Vec<u8> {
    let mut importer = _BBS_3MF_Importer::new();
    let mut data = Vec::new();
    let res = importer.get_thumbnail(path, &mut data);
    if !res {
        importer.log_errors();
    }
    data
}

/// bbs_3mf.cpp:10039-10051 `bool is_bbs_project_3mf(const std::string &path)` — True if the 3mf
/// contains BambuStudio project files.
pub fn is_bbs_project_3mf(path: &str) -> bool {
    let archive = match open_zip_reader(path) {
        Some(a) => a,
        None => return false,
    };

    let is_project =
        mz_zip_reader_locate_file(&archive, BBS_MODEL_CONFIG_FILE) != -1 || mz_zip_reader_locate_file(&archive, BBS_PROJECT_CONFIG_FILE) != -1;

    close_zip_reader(archive);
    is_project
}

/// bbs_3mf.cpp:10053-10060 `bool load_gcode_3mf_from_stream(std::istream &data, ...)`
pub fn load_gcode_3mf_from_stream(
    data: Box<dyn ReadSeek>,
    config: &mut DynamicPrintConfig,
    model: &mut Model,
    plate_data_list: &mut PlateDataPtrs,
    file_version: &mut crate::semver::Semver,
) -> Result<bool, Bbs3mfException> {
    let mut importer = _BBS_3MF_Importer::new();
    let res = importer.load_gcode_3mf_from_stream(data, model, plate_data_list, config, file_version);
    importer.log_errors();
    res
}

/// bbs_3mf.cpp:10062-10077 `bool store_bbs_3mf(StoreParams& store_params)`
pub fn store_bbs_3mf(store_params: &mut StoreParams) -> Result<bool, Bbs3mfException> {
    // All export should use "C" locales for number formatting.
    if store_params.path.is_empty() || store_params.model.is_none() {
        return Ok(false);
    }

    let mut exporter = _BBS_3MF_Exporter::new();
    let res = exporter.save_model_to_file(store_params);
    if !matches!(res, Ok(true)) {
        exporter.log_errors();
    }

    res
}


//BBS: release plate data list
/// bbs_3mf.cpp:9194-9204  void release_PlateData_list(PlateDataPtrs& plate_data_list)
pub fn release_plate_data_list(plate_data_list: &mut PlateDataPtrs) {
    //clear
    // bbs_3mf.cpp:9197-9200 — C++ `delete`s each owned pointer; the Rust Vec
    // owns its `PlateData` values and drops them on clear.
    plate_data_list.clear();
    // bbs_3mf.cpp:9203  return;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_save_strategy_flags() {
        // bbs_3mf.hpp:148-150 derived flag values
        assert_eq!(SaveStrategy::SplitModel.0, 0x1000 | (1 << 2));
        assert_eq!(
            SaveStrategy::Encrypted.0,
            (1 << 3) | 0x1000 | (1 << 2)
        );
        let s = SaveStrategy::Zip64;
        assert!(s.contains(SaveStrategy::Zip64));
        assert!(!s.contains(SaveStrategy::WithGcode));

        let combined = SaveStrategy::Zip64 | SaveStrategy::WithGcode;
        assert!(combined.contains(SaveStrategy::Zip64));
        assert!(combined.contains(SaveStrategy::WithGcode));
    }

    #[test]
    fn test_load_strategy_flags() {
        let s = LoadStrategy::LoadModel | LoadStrategy::LoadConfig;
        assert!(s.contains(LoadStrategy::LoadModel));
        assert!(s.contains(LoadStrategy::LoadConfig));
        assert!(!s.contains(LoadStrategy::Silence));
        // bbs_3mf.hpp:180
        assert!(LoadStrategy::Restore.contains(LoadStrategy::LoadAuxiliary));
        assert_eq!(LoadStrategy::Restore.0, 0x10000 | 4 | 8 | 16 | 32);
    }

    #[test]
    fn test_plate_data_new() {
        // bbs_3mf.hpp:63
        let pd = PlateData::new();
        assert_eq!(pd.plate_index, -1);
        assert!(!pd.locked);
        assert!(pd.objects_and_instances.is_empty());
    }

    #[test]
    fn test_plate_data_with_id() {
        // bbs_3mf.hpp:57-62
        let set: BTreeSet<(i32, i32)> = [(0, 0), (1, 0)].into_iter().collect();
        let pd = PlateData::with_plate_id(1, &set, true);
        assert_eq!(pd.plate_index, 1);
        assert!(pd.locked);
        assert_eq!(pd.objects_and_instances, vec![(0, 0), (1, 0)]);
    }

    #[test]
    fn test_zip_unicode_path_extra_field_roundtrip() {
        // bbs_3mf.cpp:66-98. Expected values printed by the pin's own struct
        // (a generator that includes bbs_3mf.cpp and calls encode/decode).
        let extra = ZipUnicodePathExtraField::encode("päth", "p?th");
        assert_eq!(
            extra,
            vec![
                0x75, 0x70, 0x0a, 0x00, 0x01, 0xb5, 0x88, 0x76, 0x6d, 0x70, 0xc3, 0xa4, 0x74,
                0x68
            ]
        );
        // bbs_3mf.cpp:92 tests `p + 4 + len < e` (strict), so a field that ends
        // exactly at the end of the extra data is not taken and decode falls
        // back to decode_path(path) (bbs_3mf.cpp:99); the pin prints "p?th".
        assert_eq!(ZipUnicodePathExtraField::decode(&extra, "p?th"), "p?th");
        // With any byte after the field it is taken; the pin prints "päth".
        let mut padded = extra.clone();
        padded.push(0);
        assert_eq!(ZipUnicodePathExtraField::decode(&padded, "p?th"), "päth");
        padded.extend_from_slice(&[0, 0, 0]);
        assert_eq!(ZipUnicodePathExtraField::decode(&padded, "p?th"), "päth");
        // Identical paths produce no extra field.
        assert!(ZipUnicodePathExtraField::encode("path", "path").is_empty());
        // No unicode entry -> falls back to decode_path(path).
        assert_eq!(ZipUnicodePathExtraField::decode(&[], "plain"), "plain");
    }

    #[test]
    fn test_hex_wrap() {
        // bbs_3mf.cpp:422-438
        assert_eq!(hex_wrap(0x1au8), "1a");
        assert_eq!(hex_wrap(0x1au32), "0000001a");
        assert_eq!(hex_wrap(0xdeadbeefu32), "deadbeef");
    }

    #[test]
    fn test_attribute_helpers() {
        let attributes = vec![
            ("id".to_string(), "42".to_string()),
            ("x".to_string(), "1.5".to_string()),
            ("printable".to_string(), "0".to_string()),
        ];
        assert_eq!(
            bbs_get_attribute_value_string(&attributes, "id"),
            "42".to_string()
        );
        assert_eq!(bbs_get_attribute_value_int(&attributes, "id"), 42);
        assert_eq!(bbs_get_attribute_value_float(&attributes, "x"), 1.5);
        assert!(bbs_has_attribute_value_int(&attributes, "x"));
        assert!(!bbs_has_attribute_value_int(&attributes, "missing"));
        // bbs_3mf.cpp:492 — missing attribute defaults to true
        assert!(bbs_get_attribute_value_bool(&attributes, "missing"));
        assert!(!bbs_get_attribute_value_bool(&attributes, "printable"));
    }

    #[test]
    fn test_parse_int_list_and_join() {
        // bbs_3mf.cpp:514-546
        assert_eq!(parse_int_list("3, 1 2,3"), vec![1, 2, 3]);
        assert_eq!(parse_int_list(""), Vec::<i32>::new());
        assert_eq!(parse_int_list("a 2 b"), vec![2]);
        assert_eq!(join_int_list_comma(&[1, 2, 3]), "1,2,3");
        assert_eq!(join_int_list_comma(&[]), "");
    }

    #[test]
    fn test_get_vec3_from_string() {
        // bbs_3mf.cpp:548-564
        let v = get_vec3_from_string("1 2.5 -3");
        assert_eq!(v[0], 1.0);
        assert_eq!(v[1], 2.5);
        assert_eq!(v[2], -3.0);
        let v = get_vec3_from_string("1 2");
        assert_eq!((v[0], v[1], v[2]), (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_bbs_get_transform_from_3mf_specs_string() {
        // bbs_3mf.cpp:566-593 — 4x3 column-major input transposed into 4x4
        let t = bbs_get_transform_from_3mf_specs_string(
            "1 0 0 0 1 0 0 0 1 10 20 30",
        );
        assert_eq!(t[(0, 0)], 1.0);
        assert_eq!(t[(1, 1)], 1.0);
        assert_eq!(t[(2, 2)], 1.0);
        assert_eq!(t[(0, 3)], 10.0);
        assert_eq!(t[(1, 3)], 20.0);
        assert_eq!(t[(2, 3)], 30.0);
        // invalid -> identity
        let t = bbs_get_transform_from_3mf_specs_string("1 2 3");
        assert_eq!(t, Transform3d::identity());
    }

    #[test]
    fn test_bbs_get_offset_from_3mf_specs_string() {
        // bbs_3mf.cpp:595-616
        let v = bbs_get_offset_from_3mf_specs_string("1 2 3");
        assert_eq!((v[0], v[1], v[2]), (1.0, 2.0, 3.0));
        let v = bbs_get_offset_from_3mf_specs_string("");
        assert_eq!((v[0], v[1], v[2]), (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_bbs_get_unit_factor() {
        // bbs_3mf.cpp:618-635
        assert_eq!(bbs_get_unit_factor("micron"), 0.001);
        assert_eq!(bbs_get_unit_factor("centimeter"), 10.0);
        assert_eq!(bbs_get_unit_factor("inch"), 25.4);
        assert_eq!(bbs_get_unit_factor("foot"), 304.8);
        assert_eq!(bbs_get_unit_factor("meter"), 1000.0);
        assert_eq!(bbs_get_unit_factor("millimeter"), 1.0);
        assert_eq!(bbs_get_unit_factor(""), 1.0);
    }

    #[test]
    fn test_bbs_is_valid_object_type() {
        // bbs_3mf.cpp:637-649 — BBS accepts "model" and "other"
        assert!(bbs_is_valid_object_type(""));
        assert!(bbs_is_valid_object_type("model"));
        assert!(bbs_is_valid_object_type("other"));
        assert!(!bbs_is_valid_object_type("support"));
        assert!(!bbs_is_valid_object_type("surface"));
    }

    #[test]
    fn test_release_plate_data_list() {
        // bbs_3mf.cpp:9194-9204
        let mut list = vec![PlateData::new(), PlateData::new()];
        assert_eq!(list.len(), 2);
        release_plate_data_list(&mut list);
        assert!(list.is_empty());
    }

    #[test]
    fn test_bbs_3mf_base_errors() {
        // bbs_3mf.cpp:736-751
        let base = Bbs3mfBase::new();
        base.add_error("e1".to_string());
        base.add_error("e2".to_string());
        assert_eq!(base.m_errors.lock().unwrap().len(), 2);
        base.clear_errors();
        assert!(base.m_errors.lock().unwrap().is_empty());
    }
}
