//! BambuStudio 3MF file format handler — faithful port.
//!
//! C++ Reference:
//! - `Format/bbs_3mf.hpp`
//! - `Format/bbs_3mf.cpp`
//!
//! Handles BambuStudio's extended 3MF format, which includes plate data,
//! print configuration, thumbnails and more.
//!
//! Porting status: the public data model (`PlateData`, `SaveStrategy`,
//! `LoadStrategy`, `StoreParams`, stage constants), every file/tag/attribute
//! constant, and all standalone helper functions of `bbs_3mf.cpp` are ported
//! 1:1 below. The miniz/expat-driven archive classes
//! (`_BBS_3MF_Importer`, `_BBS_3MF_Exporter`) and the threaded
//! `_BBS_Backup_Manager` are not yet ported; every blocked symbol is kept as a
//! `BLOCKED(...)` comment at its exact C++ line, following the conventions
//! established in `format::three_mf` / `format::amf`.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::Mutex;

use crate::calib::DynamicPrintConfig;
use crate::format::obj::VolumeColorInfo;
use crate::format::objparser;
use crate::gcode::g_code_processor::SliceWarning;
use crate::gcode::thumbnail_data::{PlateBBoxData, ThumbnailData};
use crate::geometry::geometry::{Transform3d, Vec3d};
use crate::locales_utils::general_format;
use crate::model::Model;
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

/// BBS: define plate data list related structures
/// bbs_3mf.hpp:55-129  struct PlateData
#[derive(Debug, Clone)]
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
    /// bbs_3mf.hpp:95  std::vector<size_t> skipped_objects;
    pub skipped_objects: Vec<usize>,
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
            skipped_objects: Vec::new(),
            config: DynamicPrintConfig::default(),
            is_support_used: false,
            is_sliced_valid: false,
            toolpath_outside: false,
            is_label_object_enabled: false,
            timelapse_warning_code: 0,
            filament_maps: Vec::new(),
            layer_filaments: LayerFilaments::new(),
            filament_change_sequence: Vec::new(),
            nozzle_change_sequence: Vec::new(),
            optimal_assignment: Vec::new(),
            nozzle_group_result: None,
            limit_filament_maps: Vec::new(),
            warnings: Vec::new(),
            nozzles_info: Vec::new(),
            locked: false,
        }
    }

    // bbs_3mf.hpp:72  void parse_filament_info(GCodeProcessorResult *result);
    // BLOCKED(bbs_3mf.cpp:653-727 `PlateData::parse_filament_info`): requires
    // the full `GCodeProcessorResult` struct (print_statistics
    // .total_volumes_per_extruder joined with filament_diameters/densities,
    // used_filaments: Vec<FilamentUseInfo>, the polymorphic
    // nozzle_group_result shared_ptr, warnings) which is not yet ported —
    // only its nested POD types exist in `gcode::g_code_processor`.

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
pub type Export3mfProgressFn = Box<dyn FnMut(i32, i32, i32, &mut bool)>;
/// bbs_3mf.hpp:228  typedef std::function<void(int import_stage, int current, int total, bool& cancel)> Import3mfProgressFn;
pub type Import3mfProgressFn = Box<dyn FnMut(i32, i32, i32, &mut bool)>;

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
        }
    }
}

impl Default for StoreParams {
    fn default() -> Self {
        Self::new()
    }
}

// bbs_3mf.hpp:320-324  class SaveObjectGaurd
// BLOCKED(bbs_3mf.cpp:9296-9304 `SaveObjectGaurd::SaveObjectGaurd/~SaveObjectGaurd`):
// RAII guard pushing/popping on `_BBS_Backup_Manager` (see below) — blocked on
// the backup manager port.

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
        value = atof(text) as f32;
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
        let mut parsed: i64 = 0;
        let mut any = false;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            any = true;
            parsed = parsed
                .saturating_mul(10)
                .saturating_add((bytes[i] - b'0') as i64);
            i += 1;
        }
        if any {
            let parsed = if negative { -parsed } else { parsed };
            value = parsed.clamp(i32::MIN as i64, i32::MAX as i64) as i32;
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

// bbs_3mf.cpp:653-727  void PlateData::parse_filament_info(GCodeProcessorResult *result)
// BLOCKED: see the note inside `impl PlateData` above — needs the full
// `GCodeProcessorResult` (print_statistics.total_volumes_per_extruder,
// filament_diameters/densities, used_filaments, the polymorphic
// nozzle_group_result and warnings), which has not been ported yet.

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

    /// bbs_3mf.cpp:746-750  void log_errors()
    pub fn log_errors(&self) {
        for error in self.m_errors.lock().unwrap().iter() {
            // bbs_3mf.cpp:749  BOOST_LOG_TRIVIAL(error) << error;
            log::error!("{}", error);
        }
    }
}

// ===========================================================================
// BLOCKED(bbs_3mf.cpp:753-5956 `class _BBS_3MF_Importer : public _BBS_3MF_Base`):
// the complete BBS 3MF reader — miniz zip extraction + expat SAX parsing of
// 3D/3dmodel.model (+ per-object .model files via the nested ObjectImporter),
// Metadata/model_settings.config, project_settings.config (JSON ->
// DynamicPrintConfig via load_from_json), slice_info.config, layer-height
// profiles / layer config ranges / brim ear points / custom per-layer gcode,
// painting data, cut information, embedded presets and auxiliary files.
// Porting it faithfully requires the reflective DynamicPrintConfig layer
// (calib::DynamicPrintConfig is still a placeholder), the Preset/PresetBundle
// JSON loaders and ModelObject volume reconstruction
// (`_generate_volumes_new`), so the whole class is deferred — no simplified
// substitute is provided (a lossy reader would silently break gcode parity).
// Symbols: load_model_from_file (1708), get_thumbnail,
// load_gcode_3mf_from_stream (1520), _destroy_xml_parser (1689),
// _extract_from_archive (2437), _extract_xml_from_archive (2505),
// _extract_model_from_archive (2548), _extract_cut_information_from_archive
// (2615), _extract_project_config_from_archive (2685),
// _extract_project_embedded_presets_from_archive (2710),
// _extract_auxiliary_file_from_archive (2807), _extract_file_from_archive
// (2865), _extract_layer_heights_profile_config_from_archive (2895),
// _extract_layer_config_ranges_from_archive (2957),
// _extract_brim_ear_points_from_archive (3015),
// _extract_sla_support_points_from_archive (3093),
// _extract_sla_drain_holes_from_archive (3175),
// _extract_filament_sequence_from_archive (3285),
// _extract_custom_gcode_per_print_z_from_archive (3337), the
// _handle_start/end_* expat handler family (3508-4990),
// _create_object_instance (4196), _apply_transform (4299),
// _generate_current_object_list (4992), _generate_volumes_new (5019),
// _generate_volumes (5265), and the nested ObjectImporter (5432-5955).
// ===========================================================================

// BLOCKED(bbs_3mf.cpp:1510-1518 `mz_zip_read_istream`): miniz stream-read
// callback for the gcode-3mf stream loader; meaningless without miniz.

// ===========================================================================
// BLOCKED(bbs_3mf.cpp:5957-8757 `class _BBS_3MF_Exporter : public _BBS_3MF_Base`):
// the complete BBS 3MF writer — miniz zip writer staging the model stream
// ([Content_Types].xml, _rels/.rels, 3D/3dmodel.model + per-object models,
// Metadata/* config and slice-info files, thumbnails, gcode payloads,
// auxiliary dirs). Deferred together with the importer for the same
// DynamicPrintConfig / Preset / ModelObject reasons.
// Symbols: save_model_to_file (6099), _add_content_types_file_to_archive
// (6693), _add_calibration_file_to_archive (6781), _add_bbox_file_to_archive
// (6801), _add_relationships_file_to_archive (6818), _add_model_file_to_archive,
// _add_object_to_model_stream (7238), _add_object_components_to_stream (7264),
// coordinate_policy_fixed/scientific (7311-7328), _add_mesh_to_object_stream
// (7330), add_transformation (7562), _add_build_to_model_stream (7573),
// _add_layer_height_profile_file_to_archive (7602),
// _add_layer_config_ranges_file_to_archive (7637),
// _add_brim_ear_points_file_to_archive (7696),
// _add_sla_support_points_file_to_archive (7731),
// _add_sla_drain_holes_file_to_archive (7767),
// _add_print_config_file_to_archive (7819),
// _add_project_config_file_to_archive (7842),
// _add_project_embedded_presets_to_archive (7851),
// _add_text_info_to_archive (7892), _add_model_config_file_to_archive (7945),
// _add_cut_information_file_to_archive (8289),
// _add_slice_info_config_file_to_archive (8352), _add_gcode_file_to_archive
// (8536), _add_custom_gcode_per_print_z_file_to_archive (8600),
// _add_auxiliary_dir_to_archive (8659), _add_filament_sequence_file_to_archive
// (8716), and reset_stream (6892).
// ===========================================================================

// bbs_3mf.cpp:8758-8767  static void handle_legacy_project_loaded(unsigned int version_project_file, DynamicPrintConfig& config)
// BLOCKED: requires the reflective `DynamicPrintConfig`
// (`config.has("brim_object_gap")` /
// `config.option<ConfigOptionFloat>("elefant_foot_compensation", false)`);
// `calib::DynamicPrintConfig` is still a placeholder without that API.

// ===========================================================================
// BLOCKED(bbs_3mf.cpp:8770-9127 `class _BBS_Backup_Manager`): backup
// background thread to dispatch tasks and coperate with ui thread — a
// boost::thread + condition_variable timer loop holding a temp Model and a
// task queue (AddObject/RemoveObject/Backup/RemoveBackup). A detached
// background thread is not wasm-safe and the tasks need Model backup paths /
// ObjectBase ids that are not yet ported. Symbols: get (8773),
// set_post_callback (8778), run_ui_tasks (8783), push_object_gaurd (8795),
// pop_object_gaurd (8799), add_object_mesh (8806), remove_object_mesh (8820),
// backup_soon (8824), remove_backup (8831), set_interval (8857),
// put_other_changes (8867), clear_other_changes (8874), has_other_changes
// (8882), Task/timer (8896-8951), push_task (8953), process_ui_task (8970),
// process_task (9013), operator() (9050), delay_task (9086).
// ===========================================================================

//BBS: add plate data list related logic
// bbs_3mf.cpp:9131-9157  bool load_bbs_3mf(const char* path, DynamicPrintConfig* config, ...)
// BLOCKED: thin wrapper over `_BBS_3MF_Importer::load_model_from_file` (see
// the importer block above).

// bbs_3mf.cpp:9159-9166  std::string bbs_3mf_get_thumbnail(const char *path)
// BLOCKED: wrapper over `_BBS_3MF_Importer::get_thumbnail`.

// bbs_3mf.cpp:9168-9175  bool load_gcode_3mf_from_stream(std::istream &data, ...)
// BLOCKED: wrapper over `_BBS_3MF_Importer::load_gcode_3mf_from_stream`.

// bbs_3mf.cpp:9177-9191  bool store_bbs_3mf(StoreParams& store_params)
// BLOCKED: wrapper over `_BBS_3MF_Exporter::save_model_to_file`.

//BBS: release plate data list
/// bbs_3mf.cpp:9194-9204  void release_PlateData_list(PlateDataPtrs& plate_data_list)
pub fn release_plate_data_list(plate_data_list: &mut PlateDataPtrs) {
    //clear
    // bbs_3mf.cpp:9197-9200 — C++ `delete`s each owned pointer; the Rust Vec
    // owns its `PlateData` values and drops them on clear.
    plate_data_list.clear();
    // bbs_3mf.cpp:9203  return;
}

// backup interface
// bbs_3mf.cpp:9208-9215  void save_object_mesh(ModelObject& object)
// bbs_3mf.cpp:9217-9221  void delete_object_mesh(ModelObject& object)
// bbs_3mf.cpp:9223-9226  void backup_soon()
// bbs_3mf.cpp:9228-9231  void remove_backup(Model& model, bool removeAll)
// bbs_3mf.cpp:9233-9236  void set_backup_interval(long interval)
// bbs_3mf.cpp:9238-9241  void set_backup_callback(std::function<void(int)> callback)
// bbs_3mf.cpp:9243-9246  void run_backup_ui_tasks()
// bbs_3mf.cpp:9281-9284  void put_other_changes()
// bbs_3mf.cpp:9286-9289  void clear_other_changes(bool backup)
// bbs_3mf.cpp:9291-9294  bool has_other_changes(bool backup)
// bbs_3mf.cpp:9296-9304  SaveObjectGaurd ctor/dtor
// BLOCKED: all delegate to the `_BBS_Backup_Manager` singleton (see block
// above).

// bbs_3mf.cpp:9248-9279  bool has_restore_data(std::string & path, std::string& origin)
// BLOCKED: needs `get_process_name(pid)` (Utils.hpp — platform process
// inspection, native-only; not added to keep the crate wasm-safe) to decide
// whether the `lock.txt` holder is still alive.

// bbs_3mf.cpp:9306-9455 — anonymous namespace tail: `bimap_cvt` (9310),
// boost::bimap tables for EmbossStyle/FontProperty serialization, and
// `to_xml(...)` / `read_emboss_shape(...)` for TextConfiguration/EmbossShape
// volumes.
// BLOCKED: writes SVG payloads into the open miniz archive and round-trips
// `TextConfiguration` / `EmbossShape` which are not wired into
// `model::ModelVolume` yet; deferred together with the exporter.

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
        // bbs_3mf.cpp:66-98
        let extra = ZipUnicodePathExtraField::encode("päth", "p?th");
        assert!(!extra.is_empty());
        assert_eq!(extra[0], 0x75);
        assert_eq!(extra[1], 0x70);
        assert_eq!(ZipUnicodePathExtraField::decode(&extra, "p?th"), "päth");
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
