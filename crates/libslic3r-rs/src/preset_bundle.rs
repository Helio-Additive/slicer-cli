//! Port of libslic3r/PresetBundle.hpp and PresetBundle.cpp (Bambu Studio at the pin; no
//! override-layer copy).
//!
//! `PresetBundle` holds the preset collections of `preset.rs` (prints, filaments, printers and
//! the SLA ones), the physical printers, the selected filament of every slot, the project config
//! and the vendor profiles. The functions follow the C++ in its order, each with its lines.
//!
//! Mappings that hold for the whole file (on top of those of `preset.rs`):
//! - A thrown `std::runtime_error` is an `Err(ConfigurationError)`; `catch` blocks match on it.
//! - A `Preset*` into a collection is its index in `m_presets` (`calibrate_printer`,
//!   `calibrate_filaments`, the presets an AMS tray installs); a `DynamicPrintConfig*` into
//!   `filament_ams_list` is its key there (`unknowns` of `sync_ams_list`).
//! - `this->option<ConfigOptionFloats>(key)` and the other typed lookups of a vector class return
//!   the option when its type code matches, which includes the nullable twin of the class (the C++
//!   `static_cast`s it); `vec_ref` / `vec_mut` here do the same.
//! - `BOOST_LOG_TRIVIAL` lines are not reproduced, `CNumericLocalesSetter` does nothing (the port
//!   never formats numbers through the locale), `L(s)` is `crate::i18n::translate`.
//! - FIDELITY-NOTE: `NozzleInventory` (`nozzle_inventory.rs`) works on `NozzlePrintConfig`, a
//!   stand-in for the fourteen keys of `DynamicPrintConfig` it reads and writes. The C++ inventory
//!   holds a reference to `project_config`; here it holds a mirror that `PresetBundle` fills from
//!   `project_config` before each call and writes back after it (`nozzle_call`). The history is the
//!   `AppConfig` of `load_selections`, which is borrowed here: it is shared with the inventory for
//!   the duration of that call only (`load_selections`).
//! - FIDELITY-NOTE: a json value of the wrong type (`json::type_error`) and a number out of range
//!   (`json::out_of_range`) are nlohmann exceptions, not `std::runtime_error`s, so the
//!   `catch (const std::runtime_error&)` blocks of the C++ (load_system_presets_from_json and the
//!   like) let them through; here they are `Err(ConfigurationError)` like every other error and
//!   those blocks collect them.

use crate::app_config::AppConfig;
use crate::config::{
    escape_strings_cstyle, unescape_strings_cstyle, ConfigBase, ConfigBaseTyped, ConfigOption, ConfigOptionBool,
    ConfigOptionInt, ConfigOptionResolver, ConfigOptionResolverTyped, ConfigOptionString, ConfigOptionStrings, ConfigOptionType,
    ConfigSubstitutions, ConfigurationError, ConfigurationErrorKind, DynamicConfig, ForwardCompatibilitySubstitutionRule, PrinterTechnology,
};
use crate::enum_bitmask::EnumBitmask;
use crate::locales_utils::string_to_double_decimal_point;
use crate::nozzle_inventory::{NozzleInventory, NozzlePrintConfig, PrinterPresetSelection};
use crate::preset::{
    boost_make_preferred, boost_path_append, extend_default_config_length, is_compatible_with_printer, preset_utils, Preset,
    PresetCollection, PresetConfigSubstitutions, PresetConfigSubstitutionsSource, PresetSelectCompatibleType, PresetWithVendorProfile,
    PresetsConfigSubstitutions, PrinterModel, PrinterPresetCollection, PrinterVariant, PhysicalPrinter, PhysicalPrinterCollection, Type,
    VendorMap, VendorProfile, BBL_JSON_KEY_FILAMENT_ID, BBL_JSON_KEY_INHERITS, BBL_JSON_KEY_NAME, BBL_JSON_KEY_SETTING_ID,
    BBL_JSON_KEY_TYPE, BBL_JSON_KEY_VERSION, PRESET_CUSTOM_VENDOR, PRESET_FILAMENT_NAME, PRESET_IOT_FILAMENT_TYPE,
    PRESET_IOT_PRINTER_TYPE, PRESET_IOT_PRINT_TYPE, PRESET_PRINTER_NAME, PRESET_PRINT_NAME, PRESET_SLA_MATERIALS_NAME,
    PRESET_SLA_PRINT_NAME, PRESET_SYSTEM_DIR, PRESET_USER_DIR,
};
use crate::print_config::config_def::{
    filament_options_with_variant, get_default_nozzle_volume_type, get_extruder_ams_count, is_filament_extruder_override_key,
    is_single_ext_multi_nozzle, multi_nozzle_diameter_mode_enabled, nozzle_volume_type_from_string, print_options_with_variant,
    printer_options_with_variant_1, printer_options_with_variant_2, save_extruder_ams_count_to_string, static_print_config_defaults,
    DynamicPrintConfig,
};
use crate::print_config::enums::{NozzleVolumeType, PrimeVolumeMode};
use crate::semver::{Semver, SLIC3R_VERSION};
use crate::utils::format_diameter_to_str_default as format_diameter_to_str;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

type CfgResult<T> = Result<T, ConfigurationError>;

/// `throw Slic3r::RuntimeError(msg)`.
fn runtime_error(msg: impl Into<String>) -> ConfigurationError {
    ConfigurationError::with_kind(ConfigurationErrorKind::RuntimeError, msg)
}

/// PresetBundle.cpp:45 `#define L(s) Slic3r::I18N::translate(s)`
#[allow(non_snake_case)]
fn L(s: &str) -> String {
    crate::i18n::translate(s)
}

// ============================================================================
// PresetBundle.hpp:18-24 constants
// ============================================================================

pub const DEFAULT_USER_FOLDER_NAME: &str = "default";
pub const BUNDLE_STRUCTURE_JSON_NAME: &str = "bundle_structure.json";

pub const VALIDATE_PRESETS_SUCCESS: i32 = 0;
pub const VALIDATE_PRESETS_PRINTER_NOT_FOUND: i32 = 1;
pub const VALIDATE_PRESETS_FILAMENTS_NOT_FOUND: i32 = 2;
pub const VALIDATE_PRESETS_MODIFIED_GCODES: i32 = 3;

// ============================================================================
// Typed option access (see the module notes)
// ============================================================================

/// `config.option<T>(key)` of a vector class with the type code `ty`: its values.
fn vec_ref<'a, T: 'static>(config: &'a DynamicConfig, key: &str, ty: ConfigOptionType) -> Option<&'a Vec<T>> {
    match config.optptr(key) {
        Some(o) if o.r#type() == ty => o.vector_values_any().and_then(|v| v.downcast_ref::<Vec<T>>()),
        _ => None,
    }
}

/// `config.option<T>(key, create)` of a vector class with the type code `ty`: its values.
fn vec_mut<'a, T: 'static>(config: &'a mut DynamicConfig, key: &str, ty: ConfigOptionType, create: bool) -> CfgResult<Option<&'a mut Vec<T>>> {
    Ok(match config.optptr_mut(key, create)? {
        Some(o) if o.r#type() == ty => o.vector_values_any_mut().and_then(|v| v.downcast_mut::<Vec<T>>()),
        _ => None,
    })
}

fn strings_ref<'a>(config: &'a DynamicConfig, key: &str) -> Option<&'a Vec<String>> {
    vec_ref::<String>(config, key, ConfigOptionType::CoStrings)
}
fn strings_mut<'a>(config: &'a mut DynamicConfig, key: &str, create: bool) -> CfgResult<Option<&'a mut Vec<String>>> {
    vec_mut::<String>(config, key, ConfigOptionType::CoStrings, create)
}
fn ints_ref<'a>(config: &'a DynamicConfig, key: &str) -> Option<&'a Vec<i32>> {
    vec_ref::<i32>(config, key, ConfigOptionType::CoInts)
}
fn ints_mut<'a>(config: &'a mut DynamicConfig, key: &str, create: bool) -> CfgResult<Option<&'a mut Vec<i32>>> {
    vec_mut::<i32>(config, key, ConfigOptionType::CoInts, create)
}
fn floats_ref<'a>(config: &'a DynamicConfig, key: &str) -> Option<&'a Vec<f64>> {
    vec_ref::<f64>(config, key, ConfigOptionType::CoFloats)
}
fn floats_mut<'a>(config: &'a mut DynamicConfig, key: &str, create: bool) -> CfgResult<Option<&'a mut Vec<f64>>> {
    vec_mut::<f64>(config, key, ConfigOptionType::CoFloats, create)
}
fn bools_ref<'a>(config: &'a DynamicConfig, key: &str) -> Option<&'a Vec<u8>> {
    vec_ref::<u8>(config, key, ConfigOptionType::CoBools)
}
fn bools_mut<'a>(config: &'a mut DynamicConfig, key: &str, create: bool) -> CfgResult<Option<&'a mut Vec<u8>>> {
    vec_mut::<u8>(config, key, ConfigOptionType::CoBools, create)
}
fn enums_ref<'a>(config: &'a DynamicConfig, key: &str) -> Option<&'a Vec<i32>> {
    vec_ref::<i32>(config, key, ConfigOptionType::CoEnums)
}
fn enums_mut<'a>(config: &'a mut DynamicConfig, key: &str, create: bool) -> CfgResult<Option<&'a mut Vec<i32>>> {
    vec_mut::<i32>(config, key, ConfigOptionType::CoEnums, create)
}

/// `config.option<ConfigOptionString>(key)->value`, or None.
fn string_ref<'a>(config: &'a DynamicConfig, key: &str) -> Option<&'a String> {
    config.option_typed::<ConfigOptionString>(key).map(|o| &o.value)
}

/// `config.opt_string(key)` (Config.hpp:2805): the C++ dereferences the option unchecked.
fn opt_string<'a>(config: &'a DynamicConfig, key: &str) -> &'a String {
    config.opt_string(key)
}

/// `config.opt_string(key, idx)` (Config.hpp:2809): `get_at(idx)` of the strings, unchecked.
fn opt_string_at(config: &DynamicConfig, key: &str, idx: usize) -> String {
    let values = strings_ref(config, key).unwrap_or_else(|| panic!("opt_string({key}, {idx}): no such option"));
    if idx < values.len() {
        values[idx].clone()
    } else {
        values[0].clone()
    }
}

/// `boost::algorithm::split(out, s, boost::algorithm::is_any_of(seps))` (no token compression):
/// an empty input gives one empty piece.
fn boost_split(s: &str, seps: &[char]) -> Vec<String> {
    s.split(|c: char| seps.contains(&c)).map(|p| p.to_string()).collect()
}

/// `boost::algorithm::join(values, sep)`
fn boost_join(values: &[String], sep: &str) -> String {
    values.join(sep)
}

/// `std::to_string(double)`: `%f`.
fn std_to_string_f64(v: f64) -> String {
    format!("{:.6}", v)
}

/// `boost::lexical_cast<double>(std::string)`: the whole text has to be a number (no leading or
/// trailing space); `bad_lexical_cast` otherwise, a `std::bad_cast` that the callers do not catch.
fn lexical_cast_double(s: &str) -> CfgResult<f64> {
    let r = crate::config::strtod_c(s.as_bytes());
    if s.is_empty() || r.consumed != s.len() || s.as_bytes()[0].is_ascii_whitespace() || r.erange {
        return Err(runtime_error("bad lexical cast: source type value could not be interpreted as target"));
    }
    Ok(r.value)
}

/// `boost::iends_with(s, suffix)` for an ASCII suffix.
fn iends_with(s: &str, suffix: &str) -> bool {
    s.len() >= suffix.len() && s.as_bytes()[s.len() - suffix.len()..].eq_ignore_ascii_case(suffix.as_bytes())
}

/// `boost::filesystem::exists(p)`.
fn fs_exists(p: &str) -> bool {
    !p.is_empty() && std::path::Path::new(p).exists()
}

/// `boost::filesystem::is_directory(p)`.
fn fs_is_directory(p: &str) -> bool {
    std::path::Path::new(p).is_dir()
}

/// The text of `boost::filesystem::filesystem_error` (boost 1.90): `boost::filesystem::<op>:
/// <message> [system:<errno>]: "<path>"`.
fn boost_fs_error(op: &str, e: &std::io::Error, path: &str) -> ConfigurationError {
    let code = e.raw_os_error().unwrap_or(0);
    let mut message = std::io::Error::from_raw_os_error(code).to_string();
    if let Some(i) = message.find(" (os error") {
        message.truncate(i);
    }
    runtime_error(format!("boost::filesystem::{}: {} [system:{}]: \"{}\"", op, message, code, path))
}

/// `fs::create_directory(p)` (the throwing form): false when it exists already.
fn fs_create_directory(p: &str) -> CfgResult<bool> {
    match std::fs::create_dir(p) {
        Ok(()) => Ok(true),
        Err(_) if fs_is_directory(p) => Ok(false),
        Err(e) => Err(boost_fs_error("create_directory", &e, p)),
    }
}

/// `path.filename().string()` of filesystem v3.
fn path_filename(p: &str) -> String {
    let is_sep = |c: char| c == '/' || (cfg!(windows) && c == '\\');
    match p.rfind(is_sep) {
        Some(i) => p[i + 1..].to_string(),
        None => p.to_string(),
    }
}

/// `boost::filesystem::directory_iterator(dir)`: the entries in the order the file system gives
/// them (readdir, as boost reads them); a directory that cannot be read throws.
fn directory_entries(dir: &str) -> CfgResult<Vec<String>> {
    let rd = std::fs::read_dir(dir).map_err(|e| boost_fs_error("directory_iterator::construct", &e, dir))?;
    let mut out = Vec::new();
    for entry in rd.flatten() {
        out.push(entry.path().to_string_lossy().into_owned());
    }
    Ok(out)
}

// ============================================================================
// PresetBundle.hpp:28-170 structs
// ============================================================================

/// PresetBundle.hpp:28-34 `struct AMSMapInfo`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AMSMapInfo {
    pub ams_id: String,
    pub slot_id: String,
    pub nozzle_diameter: String,
    pub nozzle_volume_type: Option<NozzleVolumeType>,
}

/// PresetBundle.hpp:35-50 `struct AMSComboInfo`
#[derive(Debug, Clone, Default)]
pub struct AMSComboInfo {
    pub ams_filament_colors: Vec<String>,
    pub ams_multi_color_filment: Vec<Vec<String>>,
    pub ams_filament_presets: Vec<String>,
    pub ams_names: Vec<String>,
}

impl AMSComboInfo {
    /// PresetBundle.hpp:40-45
    pub fn clear(&mut self) {
        self.ams_filament_colors.clear();
        self.ams_multi_color_filment.clear();
        self.ams_filament_presets.clear();
        self.ams_names.clear();
    }

    /// PresetBundle.hpp:46-48
    pub fn empty(&self) -> bool {
        self.ams_names.is_empty()
    }
}

/// PresetBundle.hpp:51-54 `struct MergeFilamentInfo`
#[derive(Debug, Clone, Default)]
pub struct MergeFilamentInfo {
    pub merges: Vec<Vec<i32>>,
}

impl MergeFilamentInfo {
    /// PresetBundle.hpp:53
    pub fn is_empty(&self) -> bool {
        self.merges.is_empty()
    }
}

/// PresetBundle.hpp:56-86 `struct FilamentBaseInfo`
#[derive(Debug, Clone, PartialEq)]
pub struct FilamentBaseInfo {
    pub filament_name: String,
    pub filament_id: String,
    pub filament_type: String,
    pub vendor: String,
    pub nozzle_temp_range_low: i32,
    pub nozzle_temp_range_high: i32,
    pub temperature_vitrification: i32,
    pub is_support: bool,
    pub is_system: bool,
    pub filament_printable: i32,
    pub setting_id: String,
    m_filament_extruder_compatibility: i32,
}

impl Default for FilamentBaseInfo {
    fn default() -> Self {
        FilamentBaseInfo {
            filament_name: String::new(),
            filament_id: String::new(),
            filament_type: String::new(),
            vendor: String::new(),
            nozzle_temp_range_low: 220,
            nozzle_temp_range_high: 220,
            temperature_vitrification: i32::MAX,
            is_support: false,
            is_system: true,
            filament_printable: 3,
            setting_id: String::new(),
            m_filament_extruder_compatibility: 0,
        }
    }
}

impl FilamentBaseInfo {
    /// PresetBundle.hpp:70-78
    pub fn get_extruder_compatibility(&self, extruder_id: i32) -> i32 {
        const BITS_PER_EXTRUDER: i32 = 3;
        const EXTRUDER_MASK: i32 = (1 << BITS_PER_EXTRUDER) - 1; // 0x7
        const MAX_EXTRUDER_COUNT: i32 = 32 / BITS_PER_EXTRUDER; // 10

        if extruder_id < 0 || extruder_id >= MAX_EXTRUDER_COUNT {
            return 0;
        }
        (self.m_filament_extruder_compatibility >> (BITS_PER_EXTRUDER * extruder_id)) & EXTRUDER_MASK
    }

    /// PresetBundle.hpp:80
    pub fn set_filament_extruder_compatibility(&mut self, value: i32) {
        self.m_filament_extruder_compatibility = value;
    }
    /// PresetBundle.hpp:81
    pub fn get_filament_extruder_compatibility(&self) -> i32 {
        self.m_filament_extruder_compatibility
    }
}

/// PresetBundle.hpp:88-104 `struct ExtruderNozzleStat`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExtruderNozzleStat {
    extruder_nozzle_counts: Vec<BTreeMap<NozzleVolumeType, i32>>,
}

impl ExtruderNozzleStat {
    /// PresetBundle.hpp:92
    pub fn new(nozzle_counts: Vec<BTreeMap<NozzleVolumeType, i32>>) -> Self {
        ExtruderNozzleStat { extruder_nozzle_counts: nozzle_counts }
    }

    /// PresetBundle.hpp:98
    pub fn get_raw_stat(&self) -> Vec<BTreeMap<NozzleVolumeType, i32>> {
        self.extruder_nozzle_counts.clone()
    }
    /// PresetBundle.hpp:99
    pub fn set_raw_stat(&mut self, data: &[BTreeMap<NozzleVolumeType, i32>]) {
        self.extruder_nozzle_counts = data.to_vec();
    }
}

/// PresetBundle.hpp:106-148 `struct ExtruderFullStat`: per extruder, per nozzle slot, the pair
/// (diameter, volume type name).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExtruderFullStat {
    extruder_nozzle_stats: Vec<Vec<(String, String)>>,
}

impl ExtruderFullStat {
    /// PresetBundle.hpp:110
    pub fn new(stats: Vec<Vec<(String, String)>>) -> Self {
        ExtruderFullStat { extruder_nozzle_stats: stats }
    }
    /// PresetBundle.hpp:111
    pub fn get_raw_stat(&self) -> &Vec<Vec<(String, String)>> {
        &self.extruder_nozzle_stats
    }
    /// PresetBundle.hpp:112
    pub fn set_raw_stat(&mut self, data: &[Vec<(String, String)>]) {
        self.extruder_nozzle_stats = data.to_vec();
    }
    /// PresetBundle.hpp:113
    pub fn get_extruder_count(&self) -> i32 {
        self.extruder_nozzle_stats.len() as i32
    }
    /// PresetBundle.hpp:114-118
    pub fn get_extruder_nozzle_count(&self, extruder_id: usize) -> i32 {
        if extruder_id >= self.extruder_nozzle_stats.len() {
            return 0;
        }
        self.extruder_nozzle_stats[extruder_id].len() as i32
    }
    /// PresetBundle.hpp:119-123
    pub fn get_extruder_stats(&self, extruder_id: usize) -> Vec<(String, String)> {
        if extruder_id >= self.extruder_nozzle_stats.len() {
            return Vec::new();
        }
        self.extruder_nozzle_stats[extruder_id].clone()
    }
    /// PresetBundle.hpp:124
    pub fn resize(&mut self, extruder_count: usize) {
        self.extruder_nozzle_stats.resize(extruder_count, Vec::new());
    }
    /// PresetBundle.hpp:125-129
    pub fn set_extruder_stats(&mut self, extruder_id: usize, stats: &[(String, String)]) {
        if extruder_id >= self.extruder_nozzle_stats.len() {
            self.extruder_nozzle_stats.resize(extruder_id + 1, Vec::new());
        }
        self.extruder_nozzle_stats[extruder_id] = stats.to_vec();
    }
    /// PresetBundle.hpp:130-133
    pub fn clear_extruder(&mut self, extruder_id: usize) {
        if extruder_id < self.extruder_nozzle_stats.len() {
            self.extruder_nozzle_stats[extruder_id].clear();
        }
    }
    /// PresetBundle.hpp:134
    pub fn clear(&mut self) {
        self.extruder_nozzle_stats.clear();
    }
    /// PresetBundle.hpp:135
    pub fn empty(&self) -> bool {
        self.extruder_nozzle_stats.is_empty()
    }
}

/// PresetBundle.hpp:165-171 `struct ExtruderAmsSelection`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExtruderAmsSelection {
    pub initialized: bool,
    pub device_id: String,
    pub ams4: Vec<String>,
    pub ams1: Vec<String>,
}

/// PresetBundle.hpp:198-203 `struct PresetBundle::PresetPreferences`
#[derive(Debug, Clone, Default)]
pub struct PresetPreferences {
    // name of a preferred printer model
    pub printer_model_id: String,
    // name of a preferred printer variant
    pub printer_variant: String,
    // name of a preferred filament preset
    pub filament: String,
    // name of a preferred sla_material preset
    pub sla_material: String,
}

/// PresetBundle.hpp:277-283 `struct PresetBundle::AmsSyncSubstitution`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct AmsSyncSubstitution {
    pub slot: usize,
    // the tray it came from, labelled as the machine reports it
    pub tray: String,
    // the material that tray holds, by its display name
    pub wanted: String,
    // what went into the slot; empty when nothing could be applied at all
    pub applied: String,
}

/// PresetBundle.hpp:345-351 `struct PresetBundle::ObsoletePresets`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ObsoletePresets {
    pub prints: Vec<String>,
    pub sla_prints: Vec<String>,
    pub filaments: Vec<String>,
    pub sla_materials: Vec<String>,
    pub printers: Vec<String>,
}

/// PresetBundle.hpp:375-381 `enum PresetBundle::LoadConfigBundleAttribute`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadConfigBundleAttribute {
    SaveImported,
    ResetUserProfile,
    LoadSystem,
    LoadVendorOnly,
    LoadFilamentOnly,
}

impl From<LoadConfigBundleAttribute> for u32 {
    fn from(a: LoadConfigBundleAttribute) -> u32 {
        a as u32
    }
}

/// PresetBundle.hpp:382 `using LoadConfigBundleAttributes = enum_bitmask<LoadConfigBundleAttribute>;`
pub type LoadConfigBundleAttributes = EnumBitmask<LoadConfigBundleAttribute>;

// ============================================================================
// PresetBundle.cpp:57-101 statics
// ============================================================================

/// PresetBundle.cpp:59-95 `static std::vector<std::string> s_project_options`
pub static S_PROJECT_OPTIONS: &[&str] = &[
    "flush_volumes_vector",
    "flush_volumes_matrix",
    // BBS
    "filament_colour",
    "filament_colour_type",
    "filament_multi_colour",
    "wipe_tower_x",
    "wipe_tower_y",
    "wipe_tower_rotation_angle",
    "curr_bed_type",
    "flush_multiplier",
    "flush_multiplier_fast",
    "nozzle_volume_type",
    "filament_map_mode",
    "filament_map",
    "filament_volume_map",
    "filament_nozzle_map",
    "filament_diameter_map",
    "multi_nozzle_diameter_mode",
    "extruder_nozzle_stats",
    "extruder_nozzle_stats_new",
    "prime_volume_mode",
    "extruder_nozzle_count",
    "extruder_nozzle_volume_type",
    "extruder_nozzle_diameter",
    "extruder_nozzle_slot",
    "enable_filament_dynamic_map",
    "filament_is_mixed",
    "filament_mixed_components",
    "filament_mixed_sublayer_ratios",
    "filament_mixed_gradient",
    "filament_mixed_gradient_range",
    "filament_mixed_gradient_curve",
    "filament_mixed_gradient_per_part",
    "has_filament_switcher",
];

//BBS: add BBL as default
/// PresetBundle.cpp:98
pub const BBL_BUNDLE: &str = "BBL";
/// PresetBundle.cpp:99
pub const BBL_DEFAULT_PRINTER_MODEL: &str = "Bambu Lab X1 Carbon";
/// PresetBundle.cpp:100
pub const BBL_DEFAULT_PRINTER_VARIANT: &str = "0.4";
/// PresetBundle.cpp:101
pub const BBL_DEFAULT_FILAMENT: &str = "Generic PLA";

/// PresetBundle.cpp:1371-1372 `static std::set<std::string> gcodes_key_set` (in its set order).
pub static GCODES_KEY_SET: &[&str] = &[
    "before_layer_change_gcode",
    "change_filament_gcode",
    "filament_end_gcode",
    "filament_start_gcode",
    "layer_change_gcode",
    "machine_end_gcode",
    "machine_pause_gcode",
    "machine_start_gcode",
    "printing_by_object_gcode",
    "template_custom_gcode",
    "time_lapse_gcode",
    "wrapping_detection_gcode",
];

fn s_project_options() -> Vec<String> {
    S_PROJECT_OPTIONS.iter().map(|s| s.to_string()).collect()
}

// ============================================================================
// The bridge to NozzleInventory (see the module notes)
// ============================================================================

/// The keys of `config` that `NozzleInventory` reads, as its stand-in holds them.
fn nozzle_print_config_of(config: &DynamicConfig) -> NozzlePrintConfig {
    let bool_of = |key: &str| config.option_typed::<ConfigOptionBool>(key).map(|o| o.value);
    NozzlePrintConfig {
        default_nozzle_volume_type: enums_ref(config, "default_nozzle_volume_type").cloned(),
        extruder_max_nozzle_count: ints_ref(config, "extruder_max_nozzle_count").cloned(),
        extruder_nozzle_count: ints_ref(config, "extruder_nozzle_count").cloned(),
        extruder_nozzle_diameter: floats_ref(config, "extruder_nozzle_diameter").cloned(),
        extruder_nozzle_slot: ints_ref(config, "extruder_nozzle_slot").cloned(),
        extruder_nozzle_stats: strings_ref(config, "extruder_nozzle_stats").cloned(),
        extruder_nozzle_stats_new: strings_ref(config, "extruder_nozzle_stats_new").cloned(),
        extruder_nozzle_volume_type: enums_ref(config, "extruder_nozzle_volume_type").cloned(),
        handle_hotend_as_extruder: bool_of("handle_hotend_as_extruder"),
        multi_nozzle_diameter_mode: bool_of("multi_nozzle_diameter_mode"),
        nozzle_diameter: floats_ref(config, "nozzle_diameter").cloned(),
        nozzle_volume_type: enums_ref(config, "nozzle_volume_type").cloned(),
        printer_model: string_ref(config, "printer_model").cloned(),
        printer_variant: string_ref(config, "printer_variant").cloned(),
    }
}

/// The keys `NozzleInventory::export_to` writes (NozzleInventory.cpp:785-807, each with
/// `option<T>(key, true)`), from the stand-in into `config`.
fn write_nozzle_keys(src: &NozzlePrintConfig, config: &mut DynamicConfig) -> CfgResult<()> {
    if let Some(v) = &src.nozzle_volume_type {
        if let Some(dst) = enums_mut(config, "nozzle_volume_type", true)? {
            *dst = v.clone();
        }
    }
    if let Some(v) = &src.extruder_nozzle_stats {
        if let Some(dst) = strings_mut(config, "extruder_nozzle_stats", true)? {
            *dst = v.clone();
        }
    }
    if let Some(v) = &src.extruder_nozzle_stats_new {
        if let Some(dst) = strings_mut(config, "extruder_nozzle_stats_new", true)? {
            *dst = v.clone();
        }
    }
    if let Some(v) = &src.extruder_nozzle_count {
        if let Some(dst) = ints_mut(config, "extruder_nozzle_count", true)? {
            *dst = v.clone();
        }
    }
    if let Some(v) = &src.extruder_nozzle_volume_type {
        if let Some(dst) = enums_mut(config, "extruder_nozzle_volume_type", true)? {
            *dst = v.clone();
        }
    }
    if let Some(v) = &src.extruder_nozzle_diameter {
        if let Some(dst) = floats_mut(config, "extruder_nozzle_diameter", true)? {
            *dst = v.clone();
        }
    }
    if let Some(v) = &src.extruder_nozzle_slot {
        if let Some(dst) = ints_mut(config, "extruder_nozzle_slot", true)? {
            *dst = v.clone();
        }
    }
    Ok(())
}

/// What `NozzleInventory::select_printer(const PresetCollection &printers)` reads from the printers.
fn printer_selection(printers: &PresetCollection) -> PrinterPresetSelection {
    let selected = printers.get_selected_preset();
    PrinterPresetSelection {
        selected_preset_name: selected.name.clone(),
        preset_base_name: printers.get_preset_base(selected).map(|p| p.name.clone()),
        edited_preset_config: nozzle_print_config_of(&printers.get_edited_preset().config),
    }
}

/// `NozzleVolumeType(int)` of the inventory's integer enum.
fn nvt_of_inventory(t: crate::nozzle_inventory::NozzleVolumeType) -> NozzleVolumeType {
    NozzleVolumeType::from_i32(t.0)
}

// ============================================================================
// class PresetBundle (PresetBundle.hpp:173-429)
// ============================================================================

/// PresetBundle.hpp:173-429 `class PresetBundle`
pub struct PresetBundle {
    pub block_filament_fallback_substitution: bool,
    pub incompatible_nozzle_diameter_slots: Vec<usize>,
    pub ams_sync_substitutions: Vec<AmsSyncSubstitution>,

    pub prints: PresetCollection,
    pub sla_prints: PresetCollection,
    pub filaments: PresetCollection,
    pub sla_materials: PresetCollection,
    pub printers: PrinterPresetCollection,
    pub physical_printers: PhysicalPrinterCollection,
    pub filament_presets: Vec<String>,
    pub filament_ams_list: BTreeMap<i32, DynamicPrintConfig>,
    pub ams_multi_color_filment: Vec<Vec<String>>,

    pub extruder_ams_counts: Vec<BTreeMap<i32, i32>>,
    pub extruder_ams_selections: Vec<ExtruderAmsSelection>,
    /// `Preset const *calibrate_printer`: an index into `printers` (or the edited preset, `usize::MAX - 1`).
    pub calibrate_printer: Option<usize>,
    /// `std::set<Preset const *> calibrate_filaments`: indices into `filaments`.
    pub calibrate_filaments: BTreeSet<usize>,

    pub project_config: DynamicPrintConfig,
    pub nozzle_inventory: NozzleInventory,
    /// The stand-in of `project_config` the inventory writes into (see the module notes).
    nozzle_mirror: Arc<Mutex<NozzlePrintConfig>>,

    pub vendors: VendorMap,

    pub obsolete_presets: ObsoletePresets,
}

/// The index that stands for the edited preset of a collection where an index stands for a
/// `Preset*` (`calibrate_printer`, `find_preset_index`).
pub const EDITED_PRESET_IDX: usize = usize::MAX - 1;

impl PresetBundle {
    pub const BBL_BUNDLE: &'static str = BBL_BUNDLE;
    pub const BBL_DEFAULT_PRINTER_MODEL: &'static str = BBL_DEFAULT_PRINTER_MODEL;
    pub const BBL_DEFAULT_PRINTER_VARIANT: &'static str = BBL_DEFAULT_PRINTER_VARIANT;
    pub const BBL_DEFAULT_FILAMENT: &'static str = BBL_DEFAULT_FILAMENT;

    /// PresetBundle.hpp:303-304 `PresetCollection& materials(PrinterTechnology pt)`
    pub fn materials(&self, pt: PrinterTechnology) -> &PresetCollection {
        if pt == PrinterTechnology::PtFFF {
            &self.filaments
        } else {
            &self.sla_materials
        }
    }
    /// PresetBundle.hpp:303
    pub fn materials_mut(&mut self, pt: PrinterTechnology) -> &mut PresetCollection {
        if pt == PrinterTechnology::PtFFF {
            &mut self.filaments
        } else {
            &mut self.sla_materials
        }
    }

    /// PresetBundle.hpp:358-359 `bool has_defauls_only() const`
    pub fn has_defauls_only(&self) -> bool {
        self.prints.has_defaults_only() && self.filaments.has_defaults_only() && self.printers.has_defaults_only()
    }

    /// Fill the inventory's mirror from `project_config` (NozzleInventory reads the
    /// multi-diameter mode from it).
    fn sync_nozzle_mirror(&self) {
        let cfg = nozzle_print_config_of(&self.project_config);
        *self.nozzle_mirror.lock().unwrap_or_else(|e| e.into_inner()) = cfg;
    }

    /// A call of the inventory with `project_config` as its mirror: the mirror is filled before
    /// and its nozzle keys written back after.
    fn nozzle_call<R>(&mut self, f: impl FnOnce(&mut NozzleInventory) -> R) -> CfgResult<R> {
        self.sync_nozzle_mirror();
        let r = f(&mut self.nozzle_inventory);
        let mirror = self.nozzle_mirror.lock().unwrap_or_else(|e| e.into_inner()).clone();
        write_nozzle_keys(&mirror, &mut self.project_config)?;
        Ok(r)
    }

    /// `nozzle_inventory.select_printer(printers)`
    fn nozzle_select_printer(&mut self) -> CfgResult<()> {
        let selection = printer_selection(&self.printers);
        self.nozzle_call(|inv| inv.select_printer(&selection))
    }

    /// `nozzle_inventory.open_project(printers, file_keys)` (NozzleInventory.hpp:153), the call the
    /// project loader makes on the bundle's inventory, through the bridge of the module notes: the
    /// keys of `file_keys` are handed over as the stand-in holds them, and the nozzle keys the
    /// inventory writes land in `project_config`.
    pub fn nozzle_open_project(&mut self, file_keys: &DynamicPrintConfig) -> CfgResult<()> {
        let selection = printer_selection(&self.printers);
        let keys = nozzle_print_config_of(file_keys);
        self.nozzle_call(|inv| inv.open_project(&selection, &keys))
    }

    /// PresetBundle.cpp:103-307 `static DynamicPrintConfig PresetBundle::construct_full_config(Preset &in_printer_preset, Preset &in_print_preset, const DynamicPrintConfig &project_config, std::vector<Preset> &in_filament_presets, bool apply_extruder, std::optional<std::vector<int>> filament_maps_new, std::optional<std::vector<int>> filament_volume_maps_new)`
    ///
    /// The presets are not const in C++: `compatible_printers_condition()`, `inherits()` and the
    /// other accessors create their option when it is missing.
    pub fn construct_full_config(
        in_printer_preset: &mut Preset,
        in_print_preset: &mut Preset,
        project_config: &DynamicPrintConfig,
        in_filament_presets: &mut [Preset],
        apply_extruder: bool,
        filament_maps_new: Option<Vec<i32>>,
        filament_volume_maps_new: Option<Vec<i32>>,
    ) -> CfgResult<DynamicPrintConfig> {
        let mut out = DynamicPrintConfig::new();
        out.apply(static_print_config_defaults("FullPrintConfig"), false)?;
        out.apply(&in_printer_preset.config, false)?;
        out.apply(&in_print_preset.config, false)?;
        out.apply(project_config, false)?;
        out.apply(&in_filament_presets[0].config, false)?;

        let num_filaments = in_filament_presets.len();

        let mut filament_maps: Vec<i32> = ints_ref(&out, "filament_map").expect("filament_map").clone();
        let mut filament_volume_maps: Vec<i32> = vec![NozzleVolumeType::NvtStandard as i32; num_filaments];

        let filament_volume_map_opt = ints_ref(&out, "filament_volume_map").cloned();
        if let Some(m) = filament_maps_new {
            filament_maps = m;
        }
        if let Some(m) = filament_volume_maps_new {
            filament_volume_maps = m;
        } else if let Some(opt) = &filament_volume_map_opt {
            if opt.len() == num_filaments {
                filament_volume_maps = opt.clone();
            }
        }

        // in some middle state, they may be different
        if filament_maps.len() != num_filaments {
            filament_maps.resize(num_filaments, 1);
        }
        if filament_volume_maps.len() != num_filaments {
            filament_volume_maps.resize(num_filaments, NozzleVolumeType::NvtStandard as i32);
        }

        // Collect the "compatible_printers_condition" and "inherits" values over all presets (print, filaments, printers) into a single vector.
        let mut compatible_printers_condition: Vec<String> = Vec::new();
        let mut compatible_prints_condition: Vec<String> = Vec::new();
        let mut inherits: Vec<String> = Vec::new();
        let mut filament_ids: Vec<String> = Vec::new();
        let mut print_compatible_printers: Vec<String> = Vec::new();
        compatible_printers_condition.push(in_print_preset.compatible_printers_condition_mut().clone());

        if let Some(compatible_printers) = strings_ref(&in_print_preset.config, "compatible_printers") {
            print_compatible_printers = compatible_printers.clone();
        }
        // BBS: add logic for settings check between different system presets
        let print_inherits = in_print_preset.inherits_mut().clone();
        inherits.push(print_inherits);

        //BBS: update printer config related with variants
        let mut nozzle_volume_types: Vec<Vec<NozzleVolumeType>> = Vec::new();
        let mut extruder_count: i32 = 1;
        let mut extruder_volume_type_count: i32 = 1;
        let mut different_extruder = false;
        if apply_extruder {
            different_extruder = out.support_different_extruders(&mut extruder_count);
            extruder_volume_type_count = out.get_extruder_nozzle_volume_count(extruder_count, &mut nozzle_volume_types);

            if extruder_count > 1 || different_extruder {
                out.update_values_to_printer_extruders(None, extruder_count, extruder_volume_type_count, &nozzle_volume_types, &printer_options_with_variant_2, "printer_extruder_id", "printer_extruder_variant", 2, 0, NozzleVolumeType::NvtStandard)?;
                out.update_values_to_printer_extruders(None, extruder_count, extruder_volume_type_count, &nozzle_volume_types, &printer_options_with_variant_1, "printer_extruder_id", "printer_extruder_variant", 1, 0, NozzleVolumeType::NvtStandard)?;
                // update print config related with variants
                out.update_values_to_printer_extruders(None, extruder_count, extruder_volume_type_count, &nozzle_volume_types, &print_options_with_variant, "print_extruder_id", "print_extruder_variant", 1, 0, NozzleVolumeType::NvtStandard)?;
            }
        }

        if num_filaments <= 1 {
            // BBS: update filament config related with variants
            let mut filament_config = in_filament_presets[0].config.clone();
            if apply_extruder && (extruder_count > 1 || different_extruder) {
                filament_config.update_values_to_printer_extruders(Some(&out), extruder_count, extruder_volume_type_count, &nozzle_volume_types, &filament_options_with_variant, "", "filament_extruder_variant", 1, filament_maps[0] as u32, NozzleVolumeType::from_i32(filament_volume_maps[0]))?;
            }
            out.apply(&filament_config, false)?;
            compatible_printers_condition.push(in_filament_presets[0].compatible_printers_condition_mut().clone());
            compatible_prints_condition.push(in_filament_presets[0].compatible_prints_condition_mut().clone());
            let filament_inherits = in_filament_presets[0].inherits_mut().clone();
            inherits.push(filament_inherits);
            filament_ids.push(in_filament_presets[0].filament_id.clone());

            let index_size = strings_ref(&out, "filament_extruder_variant").expect("filament_extruder_variant").len();
            let filament_self_indice = ints_mut(&mut out, "filament_self_index", true)?.expect("filament_self_index");
            filament_self_indice.resize(index_size, 1);
        } else {
            let mut filament_temp_configs: Vec<DynamicPrintConfig> = vec![DynamicPrintConfig::new(); num_filaments];
            for i in 0..num_filaments {
                filament_temp_configs[i] = in_filament_presets[i].config.clone();
                if apply_extruder && (extruder_count > 1 || different_extruder) {
                    filament_temp_configs[i].update_values_to_printer_extruders(Some(&out), extruder_count, extruder_volume_type_count, &nozzle_volume_types, &filament_options_with_variant, "", "filament_extruder_variant", 1, filament_maps[i] as u32, NozzleVolumeType::from_i32(filament_volume_maps[i]))?;
                }
            }

            // loop through options and apply them to the resulting config.
            let mut filament_variant_count: Vec<usize> = vec![1; num_filaments];
            for key in in_filament_presets[0].config.keys() {
                if key == "compatible_prints" || key == "compatible_printers" {
                    continue;
                }
                // Get a destination option.
                let opt_dst = out.option_mut(&key, false)?.expect("opt_dst");
                if opt_dst.is_scalar() {
                    // Get an option, do not create if it does not exist.
                    if let Some(opt_src) = filament_temp_configs[0].option(&key) {
                        opt_dst.set(opt_src)?;
                    }
                } else {
                    // BBS
                    let opt_vec_dst = opt_dst.as_vector_base_mut().expect("ConfigOptionVectorBase");
                    if apply_extruder {
                        // Setting a vector value from all filament_configs.
                        let filament_opts: Vec<&dyn ConfigOption> =
                            (0..num_filaments).map(|i| filament_temp_configs[i].option(&key).expect("filament option")).collect();
                        opt_vec_dst.set_from_options(&filament_opts)?;
                    } else {
                        for i in 0..num_filaments {
                            let filament_option = filament_temp_configs[i].option(&key).expect("filament option");
                            if i == 0 {
                                opt_vec_dst.set(filament_option)?;
                            } else {
                                opt_vec_dst.append(filament_option)?;
                            }

                            if key == "filament_extruder_variant" {
                                filament_variant_count[i] = filament_option.as_vector_base().expect("vector").size();
                            }
                        }
                    }
                }
            }

            let index_size = strings_ref(&out, "filament_extruder_variant").expect("filament_extruder_variant").len();
            let filament_self_indice = ints_mut(&mut out, "filament_self_index", true)?.expect("filament_self_index");
            if apply_extruder {
                filament_self_indice.resize(index_size, 1);
                let mut i = 0;
                while i < num_filaments && i < filament_self_indice.len() {
                    filament_self_indice[i] = (i + 1) as i32;
                    i += 1;
                }
            } else {
                // append filament_self_index
                filament_self_indice.resize(index_size, 1);
                let mut k = 0usize;
                for i in 0..num_filaments {
                    for _j in 0..filament_variant_count[i] {
                        filament_self_indice[k] = (i + 1) as i32;
                        k += 1;
                    }
                }
            }
        }

        // These value types clash between the print and filament profiles. They should be renamed.
        out.erase("compatible_prints");
        out.erase("compatible_prints_condition");
        out.erase("compatible_printers");
        out.erase("compatible_printers_condition");
        out.erase("inherits");
        // BBS: add logic for settings check between different system presets
        out.erase("different_settings_to_system");

        for key in ["support_filament", "support_interface_filament"] {
            let opt = out.option_typed_mut::<ConfigOptionInt>(key, false)?.expect("ConfigOptionInt");
            opt.value = opt.value.clamp(0, num_filaments as i32);
        }

        let filamnet_preset_names: Vec<String> = in_filament_presets.iter().map(|p| p.name.clone()).collect();
        out.option_typed_mut::<ConfigOptionString>("print_settings_id", true)?.expect("print_settings_id").value = in_print_preset.name.clone();
        *strings_mut(&mut out, "filament_settings_id", true)?.expect("filament_settings_id") = filamnet_preset_names;
        out.option_typed_mut::<ConfigOptionString>("printer_settings_id", true)?.expect("printer_settings_id").value = in_printer_preset.name.clone();
        *strings_mut(&mut out, "filament_ids", true)?.expect("filament_ids") = filament_ids;
        *ints_mut(&mut out, "filament_map", true)?.expect("filament_map") = filament_maps;
        *ints_mut(&mut out, "filament_volume_map", true)?.expect("filament_volume_map") = filament_volume_maps;

        let add_if_some_non_empty = |out: &mut DynamicPrintConfig, values: Vec<String>, key: &str| {
            let nonempty = values.iter().any(|v| !v.is_empty());
            if nonempty {
                out.set_key_value(key, Box::new(ConfigOptionStrings::from_values(values)));
            }
        };
        add_if_some_non_empty(&mut out, compatible_printers_condition, "compatible_machine_expression_group");
        add_if_some_non_empty(&mut out, compatible_prints_condition, "compatible_process_expression_group");
        add_if_some_non_empty(&mut out, inherits, "inherits_group");
        // BBS: add logic for settings check between different system presets
        //add_if_some_non_empty(std::move(different_settings), "different_settings_to_system");
        add_if_some_non_empty(&mut out, print_compatible_printers, "print_compatible_printers");

        set_printer_technology_fff(&mut out)?;
        Ok(out)
    }
}

/// `out.option<ConfigOptionEnumGeneric>("printer_technology", true)->value = ptFFF;`: the option
/// is a ConfigOptionEnum<PrinterTechnology> (type coEnum), which the C++ writes through the
/// generic class.
fn set_printer_technology_fff(out: &mut DynamicPrintConfig) -> CfgResult<()> {
    let opt = out.option_mut("printer_technology", true)?.expect("printer_technology");
    if opt.r#type() == ConfigOptionType::CoEnum {
        opt.set_int(PrinterTechnology::PtFFF as i32)?;
    }
    Ok(())
}

impl ExtruderNozzleStat {
    /// PresetBundle.cpp:309-321 `int ExtruderNozzleStat::get_extruder_nozzle_count(int extruder_id, std::optional<NozzleVolumeType> volume_type) const`
    pub fn get_extruder_nozzle_count(&self, extruder_id: i32, volume_type: Option<NozzleVolumeType>) -> i32 {
        if extruder_id < 0 || extruder_id as usize >= self.extruder_nozzle_counts.len() {
            return 0;
        }
        let counts = &self.extruder_nozzle_counts[extruder_id as usize];
        match volume_type {
            None | Some(NozzleVolumeType::NvtHybrid) => counts.values().fold(0, |sum, c| sum + *c),
            Some(t) => counts.get(&t).copied().unwrap_or(0),
        }
    }

    /// PresetBundle.cpp:323-334 `void ExtruderNozzleStat::on_printer_model_change_cli(const std::vector<int>& nozzle_volume_type, const std::vector<int>& max_nozzle_count)`
    pub fn on_printer_model_change_cli(&mut self, nozzle_volume_type: &[i32], max_nozzle_count: &[i32]) {
        self.extruder_nozzle_counts.resize(max_nozzle_count.len(), BTreeMap::new());
        for eid in 0..self.extruder_nozzle_counts.len() {
            let mut r#type = NozzleVolumeType::NvtStandard;
            if eid >= nozzle_volume_type.len() {
                // eid out of bounds, use standard flow
            } else {
                r#type = NozzleVolumeType::from_i32(nozzle_volume_type[eid]);
            }
            self.set_extruder_nozzle_count(eid as i32, r#type, max_nozzle_count[eid], true);
        }
    }

    /// PresetBundle.cpp:337-347 `void ExtruderNozzleStat::on_volume_type_switch(int extruder_id, NozzleVolumeType type)`
    pub fn on_volume_type_switch(&mut self, extruder_id: i32, r#type: NozzleVolumeType) {
        if r#type != NozzleVolumeType::NvtHybrid {
            let current_count = self.get_extruder_nozzle_count(extruder_id, Some(r#type));
            if extruder_id as usize >= self.extruder_nozzle_counts.len() {
                self.extruder_nozzle_counts.resize(extruder_id as usize + 1, BTreeMap::new());
            }
            self.extruder_nozzle_counts[extruder_id as usize].clear();
            self.extruder_nozzle_counts[extruder_id as usize].insert(r#type, current_count);
        }
    }

    /// PresetBundle.cpp:349-356 `void ExtruderNozzleStat::set_extruder_nozzle_count(int extruder_id, NozzleVolumeType type, int count, bool clear)`
    pub fn set_extruder_nozzle_count(&mut self, extruder_id: i32, r#type: NozzleVolumeType, count: i32, clear: bool) {
        if extruder_id as usize >= self.extruder_nozzle_counts.len() {
            self.extruder_nozzle_counts.resize(extruder_id as usize + 1, BTreeMap::new());
        }
        if clear {
            self.extruder_nozzle_counts[extruder_id as usize].clear();
        }
        self.extruder_nozzle_counts[extruder_id as usize].insert(r#type, count);
    }
}

impl Default for PresetBundle {
    fn default() -> Self {
        PresetBundle::new()
    }
}

impl Clone for PresetBundle {
    /// PresetBundle.cpp:433-436 `PresetBundle::PresetBundle(const PresetBundle &rhs) { *this = rhs; }`
    ///
    /// FIDELITY-NOTE: the members `operator=` leaves alone start as `PresetBundle()` makes them
    /// (the C++ default-constructs them; the collections and the project config are overwritten
    /// either way).
    fn clone(&self) -> Self {
        let mut b = PresetBundle::new();
        b.assign(self);
        b
    }
}

impl PresetBundle {
    /// PresetBundle.cpp:359-431 `PresetBundle::PresetBundle()`
    pub fn new() -> PresetBundle {
        let full = static_print_config_defaults("FullPrintConfig");
        let sla_full = static_print_config_defaults("SLAFullPrintConfig");
        let project_config = DynamicPrintConfig::new();
        let nozzle_mirror = Arc::new(Mutex::new(NozzlePrintConfig::default()));
        let mut b = PresetBundle {
            block_filament_fallback_substitution: false,
            incompatible_nozzle_diameter_slots: Vec::new(),
            ams_sync_substitutions: Vec::new(),
            prints: PresetCollection::new(Preset::TYPE_PRINT, Preset::print_options(), full, "Default Setting").expect("prints"),
            sla_prints: PresetCollection::new(Preset::TYPE_SLA_PRINT, Preset::sla_print_options(), sla_full, "Default Setting").expect("sla_prints"),
            filaments: PresetCollection::new(Preset::TYPE_FILAMENT, Preset::filament_options(), full, "Default Filament").expect("filaments"),
            sla_materials: PresetCollection::new(Preset::TYPE_SLA_MATERIAL, Preset::sla_material_options(), sla_full, "Default Setting")
                .expect("sla_materials"),
            printers: PrinterPresetCollection::new(Preset::TYPE_PRINTER, Preset::printer_options(), full, "Default Printer").expect("printers"),
            physical_printers: PhysicalPrinterCollection::new(PhysicalPrinter::printer_options()),
            filament_presets: Vec::new(),
            filament_ams_list: BTreeMap::new(),
            ams_multi_color_filment: Vec::new(),
            extruder_ams_counts: Vec::new(),
            extruder_ams_selections: Vec::new(),
            calibrate_printer: None,
            calibrate_filaments: BTreeSet::new(),
            project_config,
            nozzle_inventory: NozzleInventory::new(nozzle_mirror.clone()),
            nozzle_mirror,
            vendors: VendorMap::new(),
            obsolete_presets: ObsoletePresets::default(),
        };
        // The following keys are handled by the UI, they do not have a counterpart in any StaticPrintConfig derived classes,
        // therefore they need to be handled differently. As they have no counterpart in StaticPrintConfig, they are not being
        // initialized based on PrintConfigDef(), but to empty values (zeros, empty vectors, empty strings).
        //
        // "compatible_printers", "compatible_printers_condition", "inherits",
        // "print_settings_id", "filament_settings_id", "printer_settings_id", "printer_settings_id"
        // "printer_model", "printer_variant", "default_print_profile", "default_filament_profile"

        // Create the ID config keys, as they are not part of the Static print config classes.
        {
            let p = b.prints.default_preset_mut(0);
            p.config.optptr_mut("print_settings_id", true).expect("print_settings_id");
            p.compatible_printers_condition_mut();
            p.inherits_mut();
        }
        {
            let p = b.filaments.default_preset_mut(0);
            *strings_mut(&mut p.config, "filament_settings_id", true).expect("filament_settings_id").expect("filament_settings_id") = vec![String::new()];
            p.compatible_printers_condition_mut();
            p.inherits_mut();
            // Set all the nullable values to nils.
            let default_config = &mut p.config;
            for opt_key in default_config.keys() {
                let opt = default_config.optptr_mut(&opt_key, false).expect("optptr").expect("optptr");
                let is_override_key = is_filament_extruder_override_key(&opt_key);
                if !is_override_key || !opt.nullable() {
                    continue;
                }
                // deserialize("nil", ForwardCompatibilitySubstitutionRule::Disable): the rule converts to `append = false`.
                let _ = opt.deserialize("nil", false);
            }
        }
        {
            let p = b.sla_materials.default_preset_mut(0);
            p.config.optptr_mut("sla_material_settings_id", true).expect("sla_material_settings_id");
            p.compatible_printers_condition_mut();
            p.inherits_mut();
        }
        {
            let p = b.sla_prints.default_preset_mut(0);
            p.config.optptr_mut("sla_print_settings_id", true).expect("sla_print_settings_id");
            *p.config.opt_string_mut("filename_format", true) = "[input_filename_base].sl1".to_string();
            p.compatible_printers_condition_mut();
            p.inherits_mut();
        }

        //this->printers.add_default_preset(Preset::sla_printer_options(), static_cast<const SLAMaterialConfig &>(SLAFullPrintConfig::defaults()), "- default SLA -");
        //this->printers.preset(1).printer_technology_ref() = ptSLA;
        for i in 0..1 {
            // The following ugly switch is to avoid printers.preset(0) to return the edited instance, as the 0th default is the current one.
            let preset = b.printers.default_preset_mut(i);
            for key in ["printer_settings_id", "printer_model", "printer_variant", "thumbnail_size"] {
                preset.config.optptr_mut(key, true).expect("printer key");
            }
            preset.config.optptr_mut("default_print_profile", true).expect("default_print_profile");
            strings_mut(&mut preset.config, "default_filament_profile", true).expect("default_filament_profile");
            // default_sla_material_profile
            preset.inherits_mut();
        }

        // Re-activate the default presets, so their "edited" preset copies will be updated with the additional configuration values above.
        b.prints.select_preset(0);
        b.sla_prints.select_preset(0);
        b.filaments.select_preset(0);
        b.sla_materials.select_preset(0);
        b.printers.select_preset(0);

        b.project_config.apply_only(full, &s_project_options(), false).expect("project_config");
        // This is a project-only option, intentionally kept out of the static PrintConfig layout.
        b.project_config.set_key_value("multi_nozzle_diameter_mode", Box::new(ConfigOptionBool::with_value(false)));
        b
    }

    /// PresetBundle.cpp:438-463 `PresetBundle& PresetBundle::operator=(const PresetBundle &rhs)`
    pub fn assign(&mut self, rhs: &PresetBundle) -> &mut PresetBundle {
        self.prints.assign(&rhs.prints);
        self.sla_prints.assign(&rhs.sla_prints);
        self.filaments.assign(&rhs.filaments);
        self.sla_materials.assign(&rhs.sla_materials);
        self.printers.base.assign(&rhs.printers.base);
        // physical_printers = rhs.physical_printers (the default operator=)
        self.physical_printers.m_printers = rhs.physical_printers.m_printers.clone();
        self.physical_printers.m_default_config = rhs.physical_printers.m_default_config.clone();
        self.physical_printers.m_idx_selected = rhs.physical_printers.m_idx_selected;
        self.physical_printers.m_selected_preset = rhs.physical_printers.m_selected_preset.clone();
        self.physical_printers.m_dir_path = rhs.physical_printers.m_dir_path.clone();

        self.filament_presets = rhs.filament_presets.clone();
        self.project_config = rhs.project_config.clone();
        // The map of vendors is copied, so the copy owns its VendorProfile objects.
        self.vendors = rhs.vendors.iter().map(|(k, v)| (k.clone(), Arc::new((**v).clone()))).collect();
        self.obsolete_presets = rhs.obsolete_presets.clone();
        self.extruder_ams_counts = rhs.extruder_ams_counts.clone();
        self.extruder_ams_selections = rhs.extruder_ams_selections.clone();
        self.nozzle_inventory.assign(&rhs.nozzle_inventory);

        // Adjust Preset::vendor pointers to point to the copied vendors map.
        self.prints.update_vendor_ptrs_after_copy(&self.vendors);
        self.sla_prints.update_vendor_ptrs_after_copy(&self.vendors);
        self.filaments.update_vendor_ptrs_after_copy(&self.vendors);
        self.sla_materials.update_vendor_ptrs_after_copy(&self.vendors);
        self.printers.update_vendor_ptrs_after_copy(&self.vendors);

        self
    }

    /// PresetBundle.cpp:465-484 `void PresetBundle::reset(bool delete_files)`
    pub fn reset(&mut self, delete_files: bool) {
        // Clear the existing presets, delete their respective files.
        self.vendors.clear();
        self.prints.reset(delete_files);
        self.sla_prints.reset(delete_files);
        self.filaments.reset(delete_files);
        self.sla_materials.reset(delete_files);
        self.printers.reset(delete_files);
        // BBS: filament_presets is load from project config, not handled here
        //this->filament_presets.clear();
        if self.filament_presets.is_empty() {
            self.filament_presets.push(self.filaments.get_selected_preset_name());
        }
        self.obsolete_presets.prints.clear();
        self.obsolete_presets.sla_prints.clear();
        self.obsolete_presets.filaments.clear();
        self.obsolete_presets.sla_materials.clear();
        self.obsolete_presets.printers.clear();
        self.extruder_ams_selections.clear();
    }

    /// PresetBundle.cpp:486-512 `void PresetBundle::setup_directories()`
    pub fn setup_directories(&self) -> CfgResult<()> {
        let data_dir = crate::log_sink::data_dir();
        //BBS: change directoties by design
        let paths = [
            data_dir.clone(),
            boost_path_append(&data_dir, "ota"),
            boost_path_append(&data_dir, PRESET_SYSTEM_DIR),
            boost_path_append(&data_dir, PRESET_USER_DIR),
            // Store the print/filament/printer presets at the same location as the upstream Slic3r.
        ];
        for path in paths {
            let subdir = boost_make_preferred(path);
            // create_directory() returns false for a directory that exists and throws when it fails.
            if !fs_is_directory(&subdir) && !fs_create_directory(&subdir)? {
                if fs_is_directory(&subdir) {
                    // creating directory failed, maybe created by other instance, go on!
                } else {
                    return Err(runtime_error(format!("Unable to create directory {}", subdir)));
                }
            }
        }
        Ok(())
    }
}

/// PresetBundle.cpp:514-533 `static void copy_dir(const boost::filesystem::path& from_dir, const boost::filesystem::path& to_dir)`:
/// recursively copy all files and dirs in from_dir to to_dir. The boost calls throw.
fn copy_dir(from_dir: &str, to_dir: &str) -> CfgResult<()> {
    if !fs_is_directory(from_dir) {
        return Ok(());
    }
    // i assume to_dir.parent surely exists
    if !fs_is_directory(to_dir) {
        fs_create_directory(to_dir)?;
    }
    let entries = directory_entries(from_dir)?;
    for entry in entries {
        let target = boost_path_append(to_dir, &path_filename(&entry));
        if !fs_is_directory(&entry) {
            let mut em = String::new();
            let cfr = crate::utils::copy_file(&entry, &target, &mut em, false);
            if cfr != crate::utils::CopyFileResult::Success {
                // Error when copying files (logged only)
            }
        } else {
            copy_dir(&entry, &target)?;
        }
    }
    Ok(())
}

impl PresetBundle {
    /// PresetBundle.cpp:535-554 `void PresetBundle::copy_files(const std::string& from)`
    pub fn copy_files(&self, from: &str) -> CfgResult<()> {
        let data_dir = crate::log_sink::data_dir();
        // list of searched paths based on current directory system in setup_directories()
        // do not copy cache and snapshots
        let from_data_dir = from.to_string();
        //BBS: change directoties by design
        let from_dirs = [
            //from_data_dir / "vendor",
            // Store the print/filament/printer presets at the same location as the upstream Slic3r.
            boost_path_append(&from_data_dir, PRESET_PRINT_NAME),
            boost_path_append(&from_data_dir, PRESET_FILAMENT_NAME),
            boost_path_append(&from_data_dir, PRESET_PRINTER_NAME),
        ];
        // copy recursively all files
        for from_dir in &from_dirs {
            copy_dir(from_dir, &boost_path_append(&boost_path_append(&data_dir, "old"), &path_filename(from_dir)))?;
        }
        Ok(())
    }

    /// PresetBundle.cpp:556-591 `std::pair<PresetsConfigSubstitutions, std::string> PresetBundle::load_presets(AppConfig &config, ForwardCompatibilitySubstitutionRule substitution_rule, const PresetPreferences& preferred_selection)`
    pub fn load_presets(
        &mut self,
        config: &mut AppConfig,
        substitution_rule: ForwardCompatibilitySubstitutionRule,
        preferred_selection: &PresetPreferences,
    ) -> CfgResult<(PresetsConfigSubstitutions, String)> {
        // First load the vendor specific system presets.
        //BBS: change system config to json
        let (substitutions, errors_cummulative) = self.load_system_presets_from_json(substitution_rule)?;

        // BBS load preset from user's folder, load system default if
        // BBS: change directories by design
        let dir_user_presets = config.get_key("preset_folder");
        if dir_user_presets.is_empty() {
            self.load_user_presets(DEFAULT_USER_FOLDER_NAME, substitution_rule)?;
        } else {
            self.load_user_presets(&dir_user_presets, substitution_rule)?;
        }

        self.update_multi_material_filament_presets(usize::MAX)?;
        self.update_compatible(PresetSelectCompatibleType::Never)?;

        self.load_selections(config, preferred_selection)?;

        self.set_calibrate_printer("");

        // Load support recommended params from JSON
        crate::support_recommended_params::load_support_recommended_params();

        Ok((substitutions, errors_cummulative))
    }

    /// The collection of a preset type, for the functions that switch over print, printer and
    /// filament (PresetBundle.cpp:597-612).
    fn collection_of_type_mut(&mut self, r#type: Type) -> Option<&mut PresetCollection> {
        match r#type {
            Type::TypePrint => Some(&mut self.prints),
            Type::TypePrinter => Some(&mut self.printers.base),
            Type::TypeFilament => Some(&mut self.filaments),
            _ => None,
        }
    }

    /// PresetBundle.cpp:593-615 `Preset* PresetBundle::get_preset_differed_for_save(Preset& preset)`:
    /// BBS: add function to generate differed preset for save (the C++ caller frees it).
    pub fn get_preset_differed_for_save(&mut self, preset: &Preset) -> CfgResult<Option<Preset>> {
        match self.collection_of_type_mut(preset.r#type) {
            Some(c) => c.get_preset_differed_for_save(preset),
            // invalid type, return directly
            None => Ok(None),
        }
    }

    /// PresetBundle.cpp:617-637 `int PresetBundle::get_differed_values_to_update(Preset& preset, std::map<std::string, std::string>& key_values)`
    pub fn get_differed_values_to_update(&mut self, preset: &Preset, key_values: &mut BTreeMap<String, String>) -> CfgResult<i32> {
        match self.collection_of_type_mut(preset.r#type) {
            Some(c) => c.get_differed_values_to_update(preset, key_values),
            // invalid type, return directly
            None => Ok(-1),
        }
    }

    /// PresetBundle.cpp:639-650 `Semver PresetBundle::get_vendor_profile_version(std::string vendor_name)`:
    /// BBS: get vendor's current version.
    pub fn get_vendor_profile_version(&self, vendor_name: &str) -> Semver {
        let mut result_ver = Semver::new();

        if let Some(vendor_profile) = self.vendors.get(vendor_name) {
            result_ver = vendor_profile.config_version.clone();
        }

        result_ver
    }

    /// PresetBundle.cpp:652-700 `std::optional<FilamentBaseInfo> PresetBundle::get_filament_by_filament_id(const std::string& filament_id, const std::string& printer_name, bool only_system) const`
    pub fn get_filament_by_filament_id(&self, filament_id: &str, printer_name: &str, only_system: bool) -> Option<FilamentBaseInfo> {
        if filament_id.is_empty() {
            return None;
        }

        // basic filament info should be same in the parent preset and child preset
        // so just match the filament id is enough

        for filament_preset in self.filaments.iter() {
            let config = &filament_preset.config;
            if filament_preset.filament_id == filament_id {
                if only_system && !filament_preset.is_system {
                    continue;
                }
                let mut info = FilamentBaseInfo { filament_id: filament_id.to_string(), ..Default::default() };
                info.is_system = filament_preset.is_system;
                info.filament_name = filament_preset.alias.clone();
                if config.has("filament_is_support") {
                    info.is_support = bools_ref(config, "filament_is_support").expect("filament_is_support")[0] != 0;
                }
                if config.has("filament_type") {
                    info.filament_type = strings_ref(config, "filament_type").expect("filament_type")[0].clone();
                }
                if config.has("filament_vendor") {
                    info.vendor = strings_ref(config, "filament_vendor").expect("filament_vendor")[0].clone();
                }
                if config.has("nozzle_temperature_range_high") {
                    info.nozzle_temp_range_high = ints_ref(config, "nozzle_temperature_range_high").expect("nozzle_temperature_range_high")[0];
                }
                if config.has("nozzle_temperature_range_low") {
                    info.nozzle_temp_range_low = ints_ref(config, "nozzle_temperature_range_low").expect("nozzle_temperature_range_low")[0];
                }
                if config.has("temperature_vitrification") {
                    info.temperature_vitrification = ints_ref(config, "temperature_vitrification").expect("temperature_vitrification")[0];
                }

                info.setting_id = filament_preset.setting_id.clone();
                if !printer_name.is_empty() {
                    let compatible_printers = strings_ref(config, "compatible_printers").expect("compatible_printers").clone();
                    let found = compatible_printers.iter().any(|p| p == printer_name);
                    if found && config.has("filament_printable") {
                        info.filament_printable = ints_ref(config, "filament_printable").expect("filament_printable")[0];
                        if config.has("filament_extruder_compatibility") {
                            info.set_filament_extruder_compatibility(
                                ints_ref(config, "filament_extruder_compatibility").expect("filament_extruder_compatibility")[0],
                            );
                        }
                        return Some(info);
                    }
                } else {
                    return Some(info);
                }
            }
        }
        None
    }

    /// PresetBundle.cpp:702-735 `PresetsConfigSubstitutions PresetBundle::load_project_embedded_presets(std::vector<Preset*> project_presets, ForwardCompatibilitySubstitutionRule substitution_rule)`:
    /// BBS: load project embedded presets.
    pub fn load_project_embedded_presets(
        &mut self,
        project_presets: &mut [Preset],
        substitution_rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<PresetsConfigSubstitutions> {
        // First load the vendor specific system presets.
        let mut substitutions = PresetsConfigSubstitutions::new();
        let mut errors_cummulative = String::new();

        if let Err(err) = self.prints.load_project_embedded_presets(project_presets, PRESET_PRINT_NAME, &mut substitutions, substitution_rule) {
            errors_cummulative += &err.what();
        }
        if let Err(err) = self.filaments.load_project_embedded_presets(project_presets, PRESET_FILAMENT_NAME, &mut substitutions, substitution_rule) {
            errors_cummulative += &err.what();
        }
        if let Err(err) = self.printers.load_project_embedded_presets(project_presets, PRESET_PRINTER_NAME, &mut substitutions, substitution_rule) {
            errors_cummulative += &err.what();
        }

        //this->update_multi_material_filament_presets();
        //this->update_compatible(PresetSelectCompatibleType::Never);
        if !errors_cummulative.is_empty() {
            return Err(runtime_error(errors_cummulative));
        }

        //this->load_selections(config, "");

        Ok(substitutions)
    }

    /// PresetBundle.cpp:737-753 `std::vector<Preset*> PresetBundle::get_current_project_embedded_presets()`:
    /// BBS: get current project embedded presets (copies of them here).
    pub fn get_current_project_embedded_presets(&self) -> CfgResult<Vec<Preset>> {
        let mut project_presets: Vec<Preset> = self.prints.get_project_embedded_presets()?;

        let filament_presets = self.filaments.get_project_embedded_presets()?;
        if !filament_presets.is_empty() {
            project_presets.extend(filament_presets);
        }
        let printer_presets = self.printers.get_project_embedded_presets()?;
        if !printer_presets.is_empty() {
            project_presets.extend(printer_presets);
        }

        Ok(project_presets)
    }

    /// PresetBundle.cpp:755-815 `void PresetBundle::reset_project_embedded_presets()`:
    /// BBS: reset project embedded presets.
    pub fn reset_project_embedded_presets(&mut self) -> CfgResult<()> {
        let mut prefer_printer = String::new();
        {
            let current_printer = self.printers.get_selected_preset();
            if let Some(inherits) = current_printer.config.option("inherits") {
                prefer_printer = inherits.as_any().downcast_ref::<ConfigOptionString>().expect("ConfigOptionString").value.clone();
            }
        }
        //first printer, then filament, then print
        let printer_reselect = self.printers.reset_project_embedded_presets();
        let filament_reselect = self.filaments.reset_project_embedded_presets();
        let print_reselect = self.prints.reset_project_embedded_presets();

        if printer_reselect {
            if !prefer_printer.is_empty() {
                self.printers.select_preset_by_name(&prefer_printer, true, false);
            } else {
                let idx = self.printers.first_visible_idx();
                self.printers.select_preset(idx);
            }

            //this->update_multi_material_filament_presets();
            self.update_compatible(PresetSelectCompatibleType::Never)?;
        } else if filament_reselect || print_reselect {
            self.update_compatible(PresetSelectCompatibleType::Never)?;
        }

        //this->update_multi_material_filament_presets();

        //update filament_presets
        for i in 0..self.filament_presets.len() {
            let selected_filament = self.filaments.find_preset(&self.filament_presets[i], false);
            if selected_filament.is_none() {
                //it should be the project embedded presets
                let current_printer = self.printers.get_selected_preset();
                let prefered_filament_profiles = strings_ref(&current_printer.config, "default_filament_profile").expect("default_filament_profile");
                let prefered_filament_profile = prefered_filament_profiles.first().cloned().unwrap_or_default();
                if !prefered_filament_profile.is_empty() {
                    self.filament_presets[i] = prefered_filament_profile;
                } else {
                    self.filament_presets[i] = self.filaments.first_visible().name.clone();
                }
                // The embedded preset this slot had is gone along with its project; nothing here is a
                // user choice to preserve, including whatever flow it happened to be sitting on, which
                // may not even exist for the filament that just replaced it (see settle_filament_slot_flow()).
                settle_filament_slot_flow(self, i)?;
            }
        }
        Ok(())
    }

    /// The file of a printer model of a vendor, as PresetBundle.cpp:818-898 find it: the last
    /// vendor (in map order) with a model of that name wins, the first such model of a vendor.
    fn vendor_file_for_printer_model(&self, model_name: &str, file: impl Fn(&PrinterModel) -> &String) -> String {
        let mut file_name = String::new();
        let mut vendor_name = String::new();
        let mut out = String::new();

        for (vendor_first, vendor_second) in self.vendors.iter() {
            for vendor_model in vendor_second.models.iter() {
                if vendor_model.name == model_name {
                    file_name = file(vendor_model).clone();
                    vendor_name = vendor_first.clone();
                    break;
                }
            }
        }

        if !file_name.is_empty() {
            out = format!("{}/vendor/{}/{}", crate::log_sink::data_dir(), vendor_name, file_name);
            if !fs_exists(&out) {
                out = format!("{}/profiles/{}/{}", crate::utils::resources_dir().to_string_lossy(), vendor_name, file_name);
            }
        }

        out
    }

    /// PresetBundle.cpp:817-843 `std::string PresetBundle::get_texture_for_printer_model(std::string model_name)`:
    /// BBS: get bed texture for printer model.
    pub fn get_texture_for_printer_model(&self, model_name: &str) -> String {
        self.vendor_file_for_printer_model(model_name, |m| &m.bed_texture)
    }

    /// PresetBundle.cpp:845-871 `std::string PresetBundle::get_stl_model_for_printer_model(std::string model_name)`:
    /// BBS: get stl model for printer model.
    pub fn get_stl_model_for_printer_model(&self, model_name: &str) -> String {
        self.vendor_file_for_printer_model(model_name, |m| &m.bed_model)
    }

    /// PresetBundle.cpp:873-898 `std::string PresetBundle::get_hotend_model_for_printer_model(std::string model_name)`
    pub fn get_hotend_model_for_printer_model(&self, model_name: &str) -> String {
        self.vendor_file_for_printer_model(model_name, |m| &m.hotend_model)
    }

    /// PresetBundle.cpp:900-943 `PresetsConfigSubstitutions PresetBundle::load_user_presets(std::string user, ForwardCompatibilitySubstitutionRule substitution_rule)`
    pub fn load_user_presets(&mut self, user: &str, substitution_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<PresetsConfigSubstitutions> {
        let mut substitutions = PresetsConfigSubstitutions::new();
        let mut errors_cummulative = String::new();

        let user_folder = format!("{}/{}", crate::log_sink::data_dir(), PRESET_USER_DIR);
        if !fs_exists(&user_folder) {
            fs_create_directory(&user_folder)?;
        }

        let dir_user_presets = format!("{}/{}/{}", crate::log_sink::data_dir(), PRESET_USER_DIR, user);
        let folder = boost_path_append(&user_folder, user);
        if !fs_exists(&folder) {
            fs_create_directory(&folder)?;
        }

        // BBS do not load sla_print
        // BBS: change directoties by design
        {
            let print_selected_preset_name = self.prints.get_selected_preset().name.clone();
            match self.prints.load_presets(&dir_user_presets, PRESET_PRINT_NAME, &mut substitutions, substitution_rule) {
                Ok(()) => {
                    self.prints.select_preset_by_name(&print_selected_preset_name, false, false);
                }
                Err(err) => errors_cummulative += &err.what(),
            }
        }
        {
            let filament_selected_preset_name = self.filaments.get_selected_preset().name.clone();
            match self.filaments.load_presets(&dir_user_presets, PRESET_FILAMENT_NAME, &mut substitutions, substitution_rule) {
                Ok(()) => {
                    self.filaments.select_preset_by_name(&filament_selected_preset_name, false, false);
                }
                Err(err) => errors_cummulative += &err.what(),
            }
        }
        {
            let printer_selected_preset_name = self.printers.get_selected_preset().name.clone();
            match self.printers.load_presets(&dir_user_presets, PRESET_PRINTER_NAME, &mut substitutions, substitution_rule) {
                Ok(()) => {
                    self.printers.select_preset_by_name(&printer_selected_preset_name, false, false);
                }
                Err(err) => errors_cummulative += &err.what(),
            }
        }
        if !errors_cummulative.is_empty() {
            return Err(runtime_error(errors_cummulative));
        }
        self.update_multi_material_filament_presets(usize::MAX)?;
        self.update_compatible(PresetSelectCompatibleType::Never)?;

        self.set_calibrate_printer("");

        Ok(PresetsConfigSubstitutions::new())
    }

    /// PresetBundle.cpp:945-1022 `PresetsConfigSubstitutions PresetBundle::load_user_presets(AppConfig &config, std::map<std::string, std::map<std::string, std::string>> &my_presets, ForwardCompatibilitySubstitutionRule substitution_rule)`
    pub fn load_user_presets_2(
        &mut self,
        config: &mut AppConfig,
        my_presets: &mut BTreeMap<String, BTreeMap<String, String>>,
        substitution_rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<PresetsConfigSubstitutions> {
        // First load the vendor specific system presets.
        let mut substitutions = PresetsConfigSubstitutions::new();
        let mut errors_cummulative = String::new();
        let mut _process_added = false;
        let mut _filament_added = false;
        let mut _machine_added = false;

        // Sync removing
        self.remove_users_preset(config, Some(my_presets))?;

        for pass in 0..2 {
            for (name, value_map) in my_presets.iter() {
                // Load user root presets at first pass
                let inherits_iter = value_map.get(BBL_JSON_KEY_INHERITS);
                if (pass == 1) == (inherits_iter.map_or(true, |v| v.is_empty())) {
                    continue;
                }
                //get the type first
                let type_iter = match value_map.get(BBL_JSON_KEY_TYPE) {
                    Some(t) => t,
                    // can not find type for setting
                    None => continue,
                };
                let r = if type_iter == PRESET_IOT_PRINT_TYPE {
                    self.prints.load_user_preset(name, value_map.clone(), &mut substitutions, substitution_rule).map(|a| _process_added |= a)
                } else if type_iter == PRESET_IOT_FILAMENT_TYPE {
                    self.filaments.load_user_preset(name, value_map.clone(), &mut substitutions, substitution_rule).map(|a| _filament_added |= a)
                } else if type_iter == PRESET_IOT_PRINTER_TYPE {
                    self.printers.load_user_preset(name, value_map.clone(), &mut substitutions, substitution_rule).map(|a| _machine_added |= a)
                } else {
                    // invalid type for setting
                    continue;
                };
                if let Err(err) = r {
                    errors_cummulative += &err.what();
                }
            }
        }

        self.update_multi_material_filament_presets(usize::MAX)?;
        self.update_compatible(PresetSelectCompatibleType::Never)?;
        //this->load_selections(config, PresetPreferences());

        self.set_calibrate_printer("");

        if !errors_cummulative.is_empty() {
            return Err(runtime_error(errors_cummulative));
        }

        Ok(substitutions)
    }

    /// PresetBundle.cpp:1024-1109 `PresetsConfigSubstitutions PresetBundle::import_presets(std::vector<std::string> &files, std::function<int(std::string const &)> override_confirm, ForwardCompatibilitySubstitutionRule rule)`
    ///
    /// FIDELITY-NOTE: a bundle (.bbscfg, .bbsflmt, .zip) is read with the `zip` crate instead of
    /// miniz: every entry is extracted under its file name into the temporary folder and imported;
    /// an entry miniz cannot write (a directory: its name ends with a separator) is skipped as the
    /// C++ skips it.
    pub fn import_presets(
        &mut self,
        files: &mut Vec<String>,
        override_confirm: &mut dyn FnMut(&str) -> i32,
        rule: ForwardCompatibilitySubstitutionRule,
    ) -> PresetsConfigSubstitutions {
        let mut substitutions = PresetsConfigSubstitutions::new();
        let mut overwrite = 0;
        let mut result: Vec<String> = Vec::new();
        for file in files.iter_mut() {
            if crate::utils::is_json_file(file) {
                let mut f = file.clone();
                self.import_json_presets(&mut substitutions, &mut f, override_confirm, rule, &mut overwrite, &mut result);
            }
            // Determine if it is a preset bundle
            if iends_with(file, ".bbscfg") || iends_with(file, ".bbsflmt") || iends_with(file, ".zip") {
                // create user folder
                let user_folder = format!("{}/{}", crate::log_sink::data_dir(), PRESET_USER_DIR);
                if !fs_exists(&user_folder) {
                    let _ = std::fs::create_dir(&user_folder);
                }
                // create default folder
                let default_folder = boost_path_append(&user_folder, DEFAULT_USER_FOLDER_NAME);
                if !fs_exists(&default_folder) {
                    let _ = std::fs::create_dir(&default_folder);
                }
                //create temp folder
                let temp_folder = boost_make_preferred(boost_path_append(&default_folder, "temp"));
                if fs_exists(&temp_folder) {
                    let _ = std::fs::remove_dir_all(&temp_folder);
                }
                let _ = std::fs::create_dir(&temp_folder);

                *file = boost_make_preferred(file.clone());
                let zip_file = match std::fs::File::open(&*file) {
                    Ok(f) => f,
                    // Failed to initialize reader ZIP archive
                    Err(_) => return substitutions,
                };
                let mut archive = match zip::ZipArchive::new(zip_file) {
                    Ok(a) => a,
                    // Failed to initialize reader ZIP archive
                    Err(_) => return substitutions,
                };

                // Extract Files
                let num_files = archive.len();
                for i in 0..num_files {
                    let mut entry = match archive.by_index(i) {
                        Ok(e) => e,
                        Err(_) => continue,
                    };
                    let mut file_name = entry.name().to_string();
                    if let Some(index) = file_name.rfind(['/', '\\']) {
                        file_name = file_name[index + 1..].to_string();
                    }
                    if file_name == BUNDLE_STRUCTURE_JSON_NAME {
                        continue;
                    }
                    // create target file path
                    let mut target_file_path = boost_make_preferred(boost_path_append(&temp_folder, &file_name));

                    let mut data = Vec::new();
                    let extracted = std::io::Read::read_to_end(&mut entry, &mut data).is_ok()
                        && !file_name.is_empty()
                        && std::fs::write(crate::utils::encode_path(&target_file_path), &data).is_ok();
                    // target file is opened
                    if extracted {
                        let _is_success =
                            self.import_json_presets(&mut substitutions, &mut target_file_path, override_confirm, rule, &mut overwrite, &mut result);
                    }
                }
                if fs_exists(&temp_folder) {
                    let _ = std::fs::remove_dir_all(&temp_folder);
                }
            }
        }
        *files = result;
        substitutions
    }

    /// PresetBundle.cpp:1111-1222 `bool PresetBundle::import_json_presets(PresetsConfigSubstitutions &substitutions, std::string &file, std::function<int(std::string const &)> override_confirm, ForwardCompatibilitySubstitutionRule rule, int &overwrite, std::vector<std::string> &result)`
    pub fn import_json_presets(
        &mut self,
        substitutions: &mut PresetsConfigSubstitutions,
        file: &mut String,
        override_confirm: &mut dyn FnMut(&str) -> i32,
        rule: ForwardCompatibilitySubstitutionRule,
        overwrite: &mut i32,
        result: &mut Vec<String>,
    ) -> bool {
        match self.import_json_presets_inner(substitutions, file, override_confirm, rule, overwrite, result) {
            Ok(Some(r)) => r,
            Ok(None) => true,
            // Failed importing config file (logged only)
            Err(_err) => true,
        }
    }

    /// The `try` block of import_json_presets: `Some(r)` for a `return r` inside it.
    fn import_json_presets_inner(
        &mut self,
        substitutions: &mut PresetsConfigSubstitutions,
        file: &mut String,
        override_confirm: &mut dyn FnMut(&str) -> i32,
        rule: ForwardCompatibilitySubstitutionRule,
        overwrite: &mut i32,
        result: &mut Vec<String>,
    ) -> CfgResult<Option<bool>> {
        let mut config = DynamicPrintConfig::new();
        // BBS: change to json format
        // ConfigSubstitutions config_substitutions = config.load_from_ini(preset.file, substitution_rule);
        let mut key_values: BTreeMap<String, String> = BTreeMap::new();
        let mut reason = String::new();
        let config_substitutions = config.load_from_json(file, rule, &mut key_values, &mut reason);
        let name = key_values.entry(BBL_JSON_KEY_NAME.to_string()).or_default().clone();
        let version_str = key_values.entry(BBL_JSON_KEY_VERSION.to_string()).or_default().clone();
        let version = match Semver::parse(&version_str) {
            Some(v) => v,
            None => return Ok(Some(false)),
        };
        let _app_version = Semver::parse(SLIC3R_VERSION);
        /*if (version->maj() > app_version.maj()) { ... }*/

        let collection_type = if config.has("printer_settings_id") {
            Type::TypePrinter
        } else if config.has("print_settings_id") {
            Type::TypePrint
        } else if config.has("filament_settings_id") {
            Type::TypeFilament
        } else {
            // Preset type is unknown, not loading
            return Ok(Some(false));
        };
        let collection = self.collection_of_type_mut(collection_type).expect("collection");
        if *overwrite == 0 {
            *overwrite = 1;
        }
        if let Some(p) = collection.find_preset(&name, false) {
            if p.is_default || p.is_system {
                // Preset already present and is system preset, not loading
                return Ok(Some(false));
            }
            if *overwrite != 2 && *overwrite != 3 {
                *overwrite = override_confirm(&name); //3: yes to all  2: no to all
            }
        }
        if *overwrite == 0 || *overwrite == 2 {
            // Preset already present, not loading
            return Ok(Some(false));
        }

        let mut new_config: DynamicPrintConfig;
        let mut inherits_value = String::new();
        let inherits_config_value: Option<Option<String>> =
            config.option(BBL_JSON_KEY_INHERITS).map(|o| o.as_any().downcast_ref::<ConfigOptionString>().map(|s| s.value.clone()));
        let mut inherit_preset_config: Option<DynamicPrintConfig> = None;
        if let Some(inherits_config) = &inherits_config_value {
            // dynamic_cast<ConfigOptionString *>(inherits_config)->value: the option is a string.
            inherits_value = inherits_config.clone().expect("ConfigOptionString");
            inherit_preset_config = collection.find_preset_real(&inherits_value, false, true).map(|p| p.config.clone());
        }
        let has_inherit_preset = inherit_preset_config.is_some();
        if let Some(inherit_config) = &inherit_preset_config {
            new_config = inherit_config.clone();
            // Merge child diff onto parent the same way PresetCollection::load_presets() does,
            // so that "nil" sentinel slots in nullable variant arrays (e.g.
            // nozzle_temperature == ["270","nil"]) fall back to the parent's value instead of
            // leaking INT_MAX into the active config via DynamicPrintConfig::apply().
            let mut extruder_id_name = String::new();
            let mut extruder_variant_name = String::new();
            let mut key_set1: &'static BTreeSet<String> = &crate::print_config::config_def::empty_options;
            let mut key_set2: &'static BTreeSet<String> = &crate::print_config::config_def::empty_options;
            Preset::get_extruder_names_and_keysets(collection.type_(), &mut extruder_id_name, &mut extruder_variant_name, &mut key_set1, &mut key_set2);

            extend_default_config_length(&mut config, inherit_config, false, &DynamicPrintConfig::new())?;
            new_config.update_diff_values_to_child_config(&mut config, &extruder_id_name, &extruder_variant_name, key_set1, key_set2)?;
        } else {
            // We support custom root preset now
            if let Some(Some(v)) = &inherits_config_value {
                if !v.is_empty() {
                    // we should skip this preset here
                    return Ok(Some(false));
                }
            }
            // Find a default preset for the config. The PrintPresetCollection provides different default preset based on the "printer_technology" field.
            let default_preset_config = collection.default_preset_for(&config).config.clone();
            new_config = default_preset_config.clone();
            new_config.apply(&config, false)?;
            extend_default_config_length(&mut new_config, &DynamicPrintConfig::new(), true, &default_preset_config)?;
        }

        let path = collection.path_from_name(&name, !has_inherit_preset);
        collection.load_preset_move(&path, &name, new_config.clone(), false, Semver::new())?;
        let idx = collection.find_preset_internal(&name);
        // pointer maybe wrong after insert, redo find
        let inherit_setting_id: Option<String> = collection.find_preset_real(&inherits_value, false, true).map(|p| p.setting_id.clone());
        let inherit_config_now: Option<DynamicPrintConfig> = collection.find_preset_real(&inherits_value, false, true).map(|p| p.config.clone());
        let default_preset_config = collection.default_preset_for(&new_config).config.clone();
        let collection_type = collection.type_();
        let mut preset = std::mem::take(&mut collection.m_presets[idx]);
        if let Some(v) = key_values.get(BBL_JSON_KEY_FILAMENT_ID) {
            preset.filament_id = v.clone();
        }
        preset.is_external = true;
        preset.version = version;
        if let Some(sid) = &inherit_setting_id {
            preset.base_id = sid.clone();
        }
        let normalized = Preset::normalize(&mut preset.config);
        let mut save_result: CfgResult<bool> = Ok(true);
        if normalized.is_ok() {
            // Report configuration fields, which are misplaced into a wrong group.
            let _incorrect_keys = Preset::remove_invalid_keys(&mut preset.config, &default_preset_config);
            if !config_substitutions.is_empty() {
                substitutions.push(PresetConfigSubstitutions {
                    preset_name: name.clone(),
                    preset_type: collection_type,
                    preset_file: file.clone(),
                    preset_source: PresetConfigSubstitutionsSource::UserFile,
                    substitutions: config_substitutions,
                });
            }
            collection.set_custom_preset_alias(&mut preset);
            save_result = preset.save(inherit_config_now.as_ref());
        }
        collection.m_presets[idx] = preset;
        normalized?;
        save_result?;
        result.push(file.clone());
        Ok(None)
    }

    /// PresetBundle.cpp:1224-1247 `void PresetBundle::save_user_presets(AppConfig& config, std::vector<std::string>& need_to_delete_list)`:
    /// BBS save user preset to user_id preset folder.
    pub fn save_user_presets(&mut self, config: &AppConfig, need_to_delete_list: &mut Vec<String>) -> CfgResult<()> {
        let mut user_sub_folder = DEFAULT_USER_FOLDER_NAME.to_string();
        if !config.get_key("preset_folder").is_empty() {
            user_sub_folder = config.get_key("preset_folder");
        }
        //BBS: change directory by design
        let dir_user_presets = format!("{}/{}/{}", crate::log_sink::data_dir(), PRESET_USER_DIR, user_sub_folder);

        let user_folder = format!("{}/{}", crate::log_sink::data_dir(), PRESET_USER_DIR);
        if !fs_exists(&user_folder) {
            fs_create_directory(&user_folder)?;
        }

        if !fs_exists(&dir_user_presets) {
            fs_create_directory(&dir_user_presets)?;
        }

        self.prints.save_user_presets(&dir_user_presets, PRESET_PRINT_NAME, need_to_delete_list)?;
        self.filaments.save_user_presets(&dir_user_presets, PRESET_FILAMENT_NAME, need_to_delete_list)?;
        self.printers.save_user_presets(&dir_user_presets, PRESET_PRINTER_NAME, need_to_delete_list)?;
        Ok(())
    }

    /// PresetBundle.cpp:1249-1269 `void PresetBundle::update_user_presets_directory(const std::string preset_folder)`:
    /// BBS: save user preset to user_id preset folder.
    pub fn update_user_presets_directory(&mut self, preset_folder: &str) -> CfgResult<()> {
        //BBS: change directory by design
        let dir_user_presets = format!("{}/{}/{}", crate::log_sink::data_dir(), PRESET_USER_DIR, preset_folder);

        let user_folder = format!("{}/{}", crate::log_sink::data_dir(), PRESET_USER_DIR);
        if !fs_exists(&user_folder) {
            fs_create_directory(&user_folder)?;
        }

        if !fs_exists(&dir_user_presets) {
            fs_create_directory(&dir_user_presets)?;
        }

        self.prints.update_user_presets_directory(&dir_user_presets, PRESET_PRINT_NAME)?;
        self.filaments.update_user_presets_directory(&dir_user_presets, PRESET_FILAMENT_NAME)?;
        self.printers.update_user_presets_directory(&dir_user_presets, PRESET_PRINTER_NAME)?;
        Ok(())
    }

    /// PresetBundle.cpp:1271-1284 `void PresetBundle::remove_user_presets_directory(const std::string preset_folder)`
    pub fn remove_user_presets_directory(&self, preset_folder: &str) -> CfgResult<()> {
        let dir_user_presets = format!("{}/{}/{}", crate::log_sink::data_dir(), PRESET_USER_DIR, preset_folder);

        if preset_folder.is_empty() {
            // preset_folder is empty, no need to remove directory
            return Ok(());
        }
        if fs_exists(&dir_user_presets) {
            // fs::remove_all() throws when it fails.
            std::fs::remove_dir_all(&dir_user_presets).map_err(|e| boost_fs_error("remove_all", &e, &dir_user_presets))?;
        }
        Ok(())
    }

    /// PresetBundle.cpp:1286-1334 `void PresetBundle::update_system_preset_setting_ids(std::map<std::string, std::map<std::string, std::string>>& system_presets)`
    pub fn update_system_preset_setting_ids(&mut self, system_presets: &BTreeMap<String, BTreeMap<String, String>>) {
        for (name, value_map) in system_presets.iter() {
            //get the type first
            let type_iter = match value_map.get(BBL_JSON_KEY_TYPE) {
                Some(t) => t.clone(),
                // can not find type for setting
                None => continue,
            };
            let preset_collection: &mut PresetCollection = if type_iter == PRESET_IOT_PRINT_TYPE {
                &mut self.prints
            } else if type_iter == PRESET_IOT_FILAMENT_TYPE {
                &mut self.filaments
            } else if type_iter == PRESET_IOT_PRINTER_TYPE {
                &mut self.printers.base
            } else {
                // invalid type for setting
                continue;
            };
            let setting_id = match value_map.get(BBL_JSON_KEY_SETTING_ID) {
                Some(s) => s.clone(),
                // can not find setting_id for setting
                None => continue,
            };
            if let Some(preset) = preset_collection.find_preset_mut(name, false, true) {
                if !preset.setting_id.is_empty() && preset.setting_id != setting_id {
                    // local setting_id is different with remote id (logged only)
                } else if preset.setting_id.is_empty() {
                    preset.setting_id = setting_id;
                }
            } else {
                // can not find setting in system presets
                continue;
            }
        }
    }

    /// PresetBundle.cpp:1337-1368 `std::vector<std::vector<std::vector<float>>> PresetBundle::get_full_flush_matrix(bool with_multiplier) const`
    pub fn get_full_flush_matrix(&self, with_multiplier: bool) -> CfgResult<Vec<Vec<Vec<f32>>>> {
        let full_config = self.full_config(true, None, None)?;
        let extruder_nums = floats_ref(&full_config, "nozzle_diameter").expect("nozzle_diameter").len();
        let flush_volume_value: Vec<f64> = floats_ref(&full_config, "flush_volumes_matrix").expect("flush_volumes_matrix").clone();
        let filament_nums = strings_ref(&full_config, "filament_type").expect("filament_type").len();

        let mut matrix: Vec<Vec<Vec<f32>>> = Vec::new();
        for extruder_id in 0..extruder_nums {
            let flush_matrix: Vec<f32> = get_flush_volumes_matrix(&flush_volume_value, extruder_id, extruder_nums)
                .iter()
                .map(|v| *v as f32)
                .collect();
            let mut wipe_volumes: Vec<Vec<f32>> = Vec::new();
            for i in 0..filament_nums {
                wipe_volumes.push(flush_matrix[i * filament_nums..(i + 1) * filament_nums].to_vec());
            }

            matrix.push(wipe_volumes);
        }

        if with_multiplier {
            let prime_volume_mode = self.project_config.option("prime_volume_mode").expect("prime_volume_mode").get_int()?;
            let use_fast = prime_volume_mode == PrimeVolumeMode::PvmFast as i32;
            let mut flush_multiplies: Vec<f64> = if use_fast {
                floats_ref(&self.project_config, "flush_multiplier_fast").expect("flush_multiplier_fast").clone()
            } else {
                floats_ref(&self.project_config, "flush_multiplier").expect("flush_multiplier").clone()
            };
            flush_multiplies.resize(extruder_nums, 1.0);
            for extruder_id in 0..extruder_nums {
                for vec in matrix[extruder_id].iter_mut() {
                    for v in vec.iter_mut() {
                        *v = (*v as f64 * flush_multiplies[extruder_id]) as f32;
                    }
                }
            }
        }

        Ok(matrix)
    }

    /// PresetBundle.cpp:1370-1445 `int PresetBundle::validate_presets(const std::string &file_name, DynamicPrintConfig& config, std::set<std::string>& different_gcodes)`:
    /// BBS: validate printers from previous project.
    pub fn validate_presets(&self, _file_name: &str, config: &mut DynamicPrintConfig, different_gcodes: &mut BTreeSet<String>) -> CfgResult<i32> {
        let mut inherits_values: Vec<String> = strings_mut(config, "inherits_group", true)?.expect("inherits_group").clone();
        let mut filament_preset_name: Vec<String> = strings_mut(config, "filament_settings_id", true)?.expect("filament_settings_id").clone();
        let printer_preset: String =
            config.option_typed_mut::<ConfigOptionString>("printer_settings_id", true)?.expect("printer_settings_id").value.clone();
        let has_different_settings_to_system = config.option("different_settings_to_system").is_some();
        let mut different_values: Vec<String> = Vec::new();
        let mut ret = VALIDATE_PRESETS_SUCCESS;

        if has_different_settings_to_system {
            different_values = strings_mut(config, "different_settings_to_system", true)?.expect("different_settings_to_system").clone();
        }

        //PrinterTechnology printer_technology = Preset::printer_technology(config);
        let filament_count = floats_ref(config, "filament_diameter").expect("filament_diameter").len();
        inherits_values.resize(filament_count + 2, String::new());
        different_values.resize(filament_count + 2, String::new());
        filament_preset_name.resize(filament_count, String::new());

        let printer_inherits = inherits_values[filament_count + 1].clone();

        let validated = self.printers.validate_preset(&printer_preset, &printer_inherits);
        if !validated {
            // found the printer preset not inherit from system
            different_gcodes.insert(printer_preset.clone());
            ret = VALIDATE_PRESETS_PRINTER_NOT_FOUND;
        }
        for index in 0..filament_count {
            let filament_preset = &filament_preset_name[index];
            let filament_inherits = &inherits_values[index + 1];

            let validated = self.filaments.validate_preset(filament_preset, filament_inherits);
            if !validated {
                // found the filament preset not inherit from system
                different_gcodes.insert(filament_preset.clone());
                ret = VALIDATE_PRESETS_FILAMENTS_NOT_FOUND;
            }
        }

        //self defined presets, return directly
        if ret != 0 {
            return Ok(ret);
        }

        for index in 1..filament_count + 2 {
            let different_settingss = &different_values[index];

            let mut different_keys: Vec<String> = Vec::new();

            unescape_strings_cstyle(different_settingss, &mut different_keys);

            for key in &different_keys {
                if GCODES_KEY_SET.contains(&key.as_str()) {
                    different_gcodes.insert(key.clone());
                }
            }
        }

        if !different_gcodes.is_empty() {
            return Ok(VALIDATE_PRESETS_MODIFIED_GCODES);
        }

        Ok(VALIDATE_PRESETS_SUCCESS)
    }

    /// PresetBundle.cpp:1447-1540 `void PresetBundle::remove_users_preset(AppConfig &config, std::map<std::string, std::map<std::string, std::string>> *my_presets)`
    pub fn remove_users_preset(&mut self, config: &AppConfig, my_presets: Option<&BTreeMap<String, BTreeMap<String, String>>>) -> CfgResult<()> {
        let check_removed = |preset: &mut Preset| -> bool {
            let my_presets = match my_presets {
                None => return true,
                Some(m) => m,
            };
            if my_presets.contains_key(&preset.name) {
                return false;
            }
            if !preset.sync_info.is_empty() {
                return false; // syncing, not remove
            }
            if preset.setting_id.is_empty() {
                return false; // no id, not remove
            }
            // Saved preset is removed by another session
            if preset.is_dirty {
                preset.setting_id.clear();
                return false;
            }
            preset.remove_files();
            true
        };
        let preset_folder_user_id = config.get_key("preset_folder");
        let printer_selected_preset_name = self.printers.get_selected_preset().name.clone();
        let mut need_reset_printer_preset = false;
        let mut it = self.printers.begin();
        while it != self.printers.end() {
            let p = &mut self.printers.m_presets[it];
            if p.is_user() && p.user_id == preset_folder_user_id && check_removed(p) {
                if p.name == printer_selected_preset_name {
                    need_reset_printer_preset = true;
                }
                it = self.printers.erase(it);
            } else {
                it += 1;
            }
        }

        if need_reset_printer_preset {
            let default_printer_model = BBL_DEFAULT_PRINTER_MODEL;
            let mut default_printer_name = String::new();
            for p in self.printers.iter() {
                if p.config.has("printer_model") && opt_string(&p.config, "printer_model") == default_printer_model {
                    default_printer_name = p.name.clone();
                    break;
                }
            }
            self.printers.select_preset_by_name(&default_printer_name, true, false);
        } else {
            self.printers.select_preset_by_name(&printer_selected_preset_name, false, false);
        }

        let selected_print_name = self.prints.get_selected_preset().name.clone();
        let mut need_reset_print_preset = false;
        // remove preset if user_id is not current user
        let mut it = self.prints.begin();
        while it != self.prints.end() {
            let p = &mut self.prints.m_presets[it];
            if p.is_user() && p.user_id == preset_folder_user_id && check_removed(p) {
                if p.name == selected_print_name {
                    need_reset_print_preset = true;
                }
                it = self.prints.erase(it);
            } else {
                it += 1;
            }
        }
        if need_reset_print_preset && self.printers.get_selected_preset().config.has("default_print_profile") {
            let default_print_profile_name = opt_string(&self.printers.get_selected_preset().config, "default_print_profile").clone();
            self.prints.select_preset_by_name(&default_print_profile_name, true, false);
        } else {
            self.prints.select_preset_by_name(&selected_print_name, false, false);
        }

        let selected_filament_name = self.filaments.get_selected_preset().name.clone();
        let mut need_reset_filament_preset = false;
        let mut it = self.filaments.begin();
        while it != self.filaments.end() {
            let p = &mut self.filaments.m_presets[it];
            if p.is_user() && p.user_id == preset_folder_user_id && check_removed(p) {
                if p.name == selected_filament_name {
                    need_reset_filament_preset = true;
                }
                it = self.filaments.erase(it);
            } else {
                it += 1;
            }
        }
        if need_reset_filament_preset && self.printers.get_selected_preset().config.has("default_filament_profile") {
            let prefered_filament_profiles =
                strings_ref(&self.printers.get_selected_preset().config, "default_filament_profile").expect("default_filament_profile").clone();
            if !prefered_filament_profiles.is_empty() {
                self.filaments.select_preset_by_name(&prefered_filament_profiles[0], true, false);
            }
        } else {
            self.filaments.select_preset_by_name(&selected_filament_name, false, false);
        }

        self.update_compatible(PresetSelectCompatibleType::Always)?;

        /* set selected preset */
        for i in 0..self.filament_presets.len() {
            let preset = self.filaments.find_preset(&self.filament_presets[i], false);
            if preset.is_none() {
                self.filament_presets[i] = self.filaments.get_selected_preset_name();
            }
        }
        Ok(())
    }

    /// The part of the load_system_* functions that turns a system rule into the rule of the
    /// vendor files (PresetBundle.cpp:1608-1613).
    fn system_compatibility_rule(compatibility_rule: ForwardCompatibilitySubstitutionRule) -> ForwardCompatibilitySubstitutionRule {
        if compatibility_rule == ForwardCompatibilitySubstitutionRule::EnableSystemSilent {
            // Loading system presets, don't log substitutions.
            ForwardCompatibilitySubstitutionRule::EnableSilent
        } else if compatibility_rule == ForwardCompatibilitySubstitutionRule::EnableSilentDisableSystem {
            // Loading system presets, throw on unknown option value.
            ForwardCompatibilitySubstitutionRule::Disable
        } else {
            compatibility_rule
        }
    }

    /// The loop of PresetBundle.cpp:1621-1654 and 1716-1746: every vendor json of `dir` loaded
    /// with `flags`, the first into this bundle, the others merged into it.
    fn load_vendor_jsons_merged(
        &mut self,
        dir: &str,
        flags: LoadConfigBundleAttributes,
        compatibility_rule: ForwardCompatibilitySubstitutionRule,
        substitutions: &mut PresetsConfigSubstitutions,
        errors_cummulative: &mut String,
    ) -> CfgResult<bool> {
        let mut first = true;
        for vendor_file in directory_entries(dir)? {
            if crate::utils::is_json_file(&vendor_file) {
                let mut vendor_name = path_filename(&vendor_file);
                // Remove the .json suffix.
                vendor_name.truncate(vendor_name.len() - 5);
                let r: CfgResult<()> = (|| {
                    // Load the config bundle, flatten it.
                    if first {
                        // Reset this PresetBundle and load the first vendor config.
                        substitutions.extend(self.load_vendor_configs_from_json(dir, &vendor_name, flags, compatibility_rule)?.0);
                        first = false;
                    } else {
                        // Load the other vendor configs, merge them with this PresetBundle.
                        // Report duplicate profiles.
                        let mut other = PresetBundle::new();
                        substitutions.extend(other.load_vendor_configs_from_json(dir, &vendor_name, flags, compatibility_rule)?.0);
                        let duplicates = self.merge_presets(other);
                        if !duplicates.is_empty() {
                            *errors_cummulative += &format!("Found duplicated settings in vendor {}'s json file lists: ", vendor_name);
                            for (i, d) in duplicates.iter().enumerate() {
                                if i > 0 {
                                    *errors_cummulative += ", ";
                                }
                                *errors_cummulative += d;
                            }
                        }
                    }
                    Ok(())
                })();
                if let Err(err) = r {
                    *errors_cummulative += &err.what();
                    *errors_cummulative += "\n";
                }
            }
        }
        Ok(first)
    }

    /// PresetBundle.cpp:1603-1665 `std::pair<PresetsConfigSubstitutions, std::string> PresetBundle::load_system_presets_from_json(ForwardCompatibilitySubstitutionRule compatibility_rule)`:
    /// BBS: add json related logic, load system presets from json.
    fn load_system_presets_from_json(&mut self, compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<(PresetsConfigSubstitutions, String)> {
        let compatibility_rule = Self::system_compatibility_rule(compatibility_rule);

        // Here the vendor specific read only Config Bundles are stored.
        //BBS: change directory by design
        let dir = boost_make_preferred(boost_path_append(&crate::log_sink::data_dir(), PRESET_SYSTEM_DIR));
        let mut substitutions = PresetsConfigSubstitutions::new();
        let mut errors_cummulative = String::new();
        let first = self.load_vendor_jsons_merged(
            &dir,
            LoadConfigBundleAttributes::new(LoadConfigBundleAttribute::LoadSystem),
            compatibility_rule,
            &mut substitutions,
            &mut errors_cummulative,
        )?;

        if first {
            // No config bundle loaded, reset.
            self.reset(false);
        }

        self.update_system_maps();
        Ok((substitutions, errors_cummulative))
    }

    /// PresetBundle.cpp:1667-1699 `std::pair<PresetsConfigSubstitutions, std::string> PresetBundle::load_system_models_from_json(ForwardCompatibilitySubstitutionRule compatibility_rule)`
    pub fn load_system_models_from_json(&mut self, compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<(PresetsConfigSubstitutions, String)> {
        let compatibility_rule = Self::system_compatibility_rule(compatibility_rule);

        // Here the vendor specific read only Config Bundles are stored.
        let dir = boost_make_preferred(boost_path_append(&crate::utils::resources_dir().to_string_lossy(), "profiles"));
        let mut substitutions = PresetsConfigSubstitutions::new();
        let mut errors_cummulative = String::new();
        for vendor_file in directory_entries(&dir)? {
            if crate::utils::is_json_file(&vendor_file) {
                let mut vendor_name = path_filename(&vendor_file);
                // Remove the .json suffix.
                vendor_name.truncate(vendor_name.len() - 5);
                // Load the config bundle, flatten it.
                match self.load_vendor_configs_from_json(
                    &dir,
                    &vendor_name,
                    LoadConfigBundleAttributes::new(LoadConfigBundleAttribute::LoadVendorOnly),
                    compatibility_rule,
                ) {
                    Ok(r) => substitutions.extend(r.0),
                    Err(err) => {
                        errors_cummulative += &err.what();
                        errors_cummulative += "\n";
                    }
                }
            }
        }

        Ok((substitutions, errors_cummulative))
    }

    /// PresetBundle.cpp:1701-1750 `std::pair<PresetsConfigSubstitutions, std::string> PresetBundle::load_system_filaments_json(ForwardCompatibilitySubstitutionRule compatibility_rule)`
    pub fn load_system_filaments_json(&mut self, compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<(PresetsConfigSubstitutions, String)> {
        let compatibility_rule = Self::system_compatibility_rule(compatibility_rule);

        // Here the vendor specific read only Config Bundles are stored.
        let dir = boost_make_preferred(boost_path_append(&crate::utils::resources_dir().to_string_lossy(), "profiles"));
        let mut substitutions = PresetsConfigSubstitutions::new();
        let mut errors_cummulative = String::new();
        self.load_vendor_jsons_merged(
            &dir,
            LoadConfigBundleAttributes::new(LoadConfigBundleAttribute::LoadSystem) | LoadConfigBundleAttribute::LoadFilamentOnly,
            compatibility_rule,
            &mut substitutions,
            &mut errors_cummulative,
        )?;

        Ok((substitutions, errors_cummulative))
    }

    /// PresetBundle.cpp:1752-1776 `VendorProfile PresetBundle::get_custom_vendor_models() const`
    pub fn get_custom_vendor_models(&self) -> VendorProfile {
        let mut vendor = VendorProfile::new();
        vendor.name = PRESET_CUSTOM_VENDOR.to_string();
        vendor.id = PRESET_CUSTOM_VENDOR.to_string();
        for preset in self.printers.get_presets().iter() {
            if preset.is_system {
                continue;
            }
            if !self.printers.get_preset_base(preset).map_or(false, |b| std::ptr::eq(b, preset)) {
                continue;
            }
            if preset.is_default {
                continue;
            }
            let model = opt_string(&preset.config, "printer_model").clone();
            let variant = opt_string(&preset.config, "printer_variant").clone();
            match vendor.models.iter_mut().find(|m| m.name == model) {
                None => {
                    let mut m = PrinterModel::default();
                    m.id = model.clone();
                    m.name = model.clone();
                    m.variants = vec![PrinterVariant::with_name(&variant)];
                    vendor.models.push(m);
                }
                Some(m) => {
                    m.variants.push(PrinterVariant::with_name(&variant));
                }
            }
        }
        vendor
    }
}

/// PrintConfig.hpp:2176-2184 `template<typename T> static std::vector<T> get_flush_volumes_matrix(const std::vector<T> &fv_matrix, size_t extruder_id = -1, size_t nozzle_nums = 1)`
fn get_flush_volumes_matrix<T: Clone>(fv_matrix: &[T], extruder_id: usize, nozzle_nums: usize) -> Vec<T> {
    if extruder_id != usize::MAX && nozzle_nums != 1 {
        let from = ((fv_matrix.len() / nozzle_nums * extruder_id) as f64 + crate::libslic3r::EPSILON) as usize;
        let to = ((fv_matrix.len() / nozzle_nums * (extruder_id + 1)) as f64 + crate::libslic3r::EPSILON) as usize;
        return fv_matrix[from..to].to_vec();
    }
    fv_matrix.to_vec()
}

/// PresetBundle.cpp:1778-1789 `std::string GetNozzleVolumeTypeStr(const NozzleVolumeType &type)`
#[allow(non_snake_case)]
pub fn GetNozzleVolumeTypeStr(r#type: NozzleVolumeType) -> String {
    match r#type {
        NozzleVolumeType::NvtStandard => "Standard".to_string(),
        NozzleVolumeType::NvtHighFlow => "High Flow".to_string(),
        NozzleVolumeType::NvtTPUHighFlow => "TPU High Flow".to_string(),
        NozzleVolumeType::NvtTPU285 => "TPU285".to_string(),
        NozzleVolumeType::NvtE3DHighFlow => "E3D High Flow".to_string(),
        NozzleVolumeType::NvtHybrid => "Hybrid".to_string(),
        _ => String::new(),
    }
}

/// PresetBundle.cpp:1791-1797 `std::map<NozzleVolumeType, int> count_default_nozzle_volume_types(const ConfigBase &printer_config, int extruder_id, int slot_count)`:
/// count one extruder's physical nozzle slots by their preset default volume type (never Hybrid).
pub fn count_default_nozzle_volume_types(printer_config: &(impl ConfigOptionResolver + ?Sized), extruder_id: i32, slot_count: i32) -> BTreeMap<NozzleVolumeType, i32> {
    let mut counts: BTreeMap<NozzleVolumeType, i32> = BTreeMap::new();
    for slot in 0..slot_count {
        *counts.entry(get_default_nozzle_volume_type(printer_config, extruder_id, slot)).or_insert(0) += 1;
    }
    counts
}

/// PresetBundle.cpp:1799-1820 `ExtruderFullStat build_extruder_full_stat_from_nozzle_stat(const ExtruderNozzleStat &nozzle_stat, const std::vector<int> &extruder_max_nozzle_count, const std::string &diameter_str)`
pub fn build_extruder_full_stat_from_nozzle_stat(nozzle_stat: &ExtruderNozzleStat, extruder_max_nozzle_count: &[i32], diameter_str: &str) -> ExtruderFullStat {
    let mut full_stat = ExtruderFullStat::default();
    let raw_stats = nozzle_stat.get_raw_stat();
    full_stat.resize(raw_stats.len());
    for extruder_id in 0..raw_stats.len() {
        let mut nozzle_stats: Vec<(String, String)> = Vec::new();
        let max_count = if extruder_id < extruder_max_nozzle_count.len() { extruder_max_nozzle_count[extruder_id].max(0) } else { 0 };
        for (volume_type, count) in raw_stats[extruder_id].iter() {
            let volume_str = GetNozzleVolumeTypeStr(*volume_type);
            let mut i = 0;
            while i < *count && (nozzle_stats.len() as i32) < max_count {
                nozzle_stats.push((diameter_str.to_string(), volume_str.clone()));
                i += 1;
            }
        }
        // 保留空槽占位,不臆造口径错误的喷嘴。
        nozzle_stats.resize(max_count as usize, (String::new(), String::new()));
        full_stat.set_extruder_stats(extruder_id, &nozzle_stats);
    }
    full_stat
}

/// PresetBundle.cpp:1822-1843 `bool per_slot_nozzle_options_consistent(const DynamicPrintConfig &config)`
pub fn per_slot_nozzle_options_consistent(config: &DynamicPrintConfig) -> bool {
    // option<ConfigOptionInts>, option<ConfigOptionEnumsGeneric>, option<ConfigOptionFloats>, option<ConfigOptionInts>
    let empty_i: Vec<i32> = Vec::new();
    let empty_f: Vec<f64> = Vec::new();
    let counts = ints_ref(config, "extruder_nozzle_count").unwrap_or(&empty_i);
    let vtypes = enums_ref(config, "extruder_nozzle_volume_type").unwrap_or(&empty_i);
    let diams = floats_ref(config, "extruder_nozzle_diameter").unwrap_or(&empty_f);
    let slots = ints_ref(config, "extruder_nozzle_slot").unwrap_or(&empty_i);

    let mut total: i32 = 0;
    for &c in counts {
        if c < 0 {
            return false;
        }
        total = total.wrapping_add(c);
    }
    total > 0 && vtypes.len() as i32 == total && diams.len() as i32 == total && slots.len() as i32 == total
}

/// PresetBundle.cpp:1845-1871 `void sync_per_slot_nozzle_options_from_full_stat(DynamicPrintConfig &config, const ExtruderFullStat &full_stat)`
pub fn sync_per_slot_nozzle_options_from_full_stat(config: &mut DynamicPrintConfig, full_stat: &ExtruderFullStat) -> CfgResult<()> {
    let raw = full_stat.get_raw_stat();
    let mut nozzle_counts: Vec<i32> = Vec::with_capacity(raw.len());
    let mut nozzle_volume_types: Vec<i32> = Vec::new();
    let mut nozzle_diameters: Vec<f64> = Vec::new();
    let mut nozzle_slots: Vec<i32> = Vec::new();
    let mut global_slot = 0;
    for ext in raw {
        let mut installed = 0;
        for slot in ext {
            if !slot.1.is_empty() {
                nozzle_volume_types.push(nozzle_volume_type_from_string(&slot.1) as i32);
                nozzle_diameters.push(if slot.0.is_empty() { 0.0 } else { string_to_double_decimal_point(&slot.0).0 });
                nozzle_slots.push(global_slot);
                installed += 1;
            }
            global_slot += 1;
        }
        nozzle_counts.push(installed);
    }
    *ints_mut(config, "extruder_nozzle_count", true)?.expect("extruder_nozzle_count") = nozzle_counts;
    *enums_mut(config, "extruder_nozzle_volume_type", true)?.expect("extruder_nozzle_volume_type") = nozzle_volume_types;
    *floats_mut(config, "extruder_nozzle_diameter", true)?.expect("extruder_nozzle_diameter") = nozzle_diameters;
    *ints_mut(config, "extruder_nozzle_slot", true)?.expect("extruder_nozzle_slot") = nozzle_slots;
    Ok(())
}

/// PresetBundle.cpp:1873-1904 `static std::vector<double> smallest_installed_diameters(const NozzleInventory &nozzles, const std::vector<double> &fallback)`:
/// the smallest installed nozzle diameter of each extruder, so slicing uses the selected
/// diameters; an extruder with no installed slot keeps its fallback value.
fn smallest_installed_diameters(nozzles: &NozzleInventory, fallback: &[f64]) -> Vec<f64> {
    let mut result = fallback.to_vec();
    for e in 0..result.len() {
        let mut min_diam = 0.0;
        let mut found = false;
        for slot in nozzles.slots(e as i32).iter() {
            if slot.empty() {
                continue;
            }
            let d = string_to_double_decimal_point(&slot.diameter).0;
            if d <= 0.0 {
                continue;
            }
            if !found || d < min_diam {
                min_diam = d;
                found = true;
            }
        }
        if found {
            result[e] = min_diam;
        }
    }
    result
}

impl PresetBundle {
    /// PresetBundle.cpp:1906-1925 `std::vector<std::string> PresetBundle::merge_presets(PresetBundle &&other)`:
    /// merge one vendor's presets with the other vendor's presets, report duplicates.
    fn merge_presets(&mut self, other: PresetBundle) -> Vec<String> {
        let PresetBundle { vendors, prints, sla_prints, filaments, sla_materials, printers, obsolete_presets, .. } = other;
        for (k, v) in vendors {
            self.vendors.entry(k).or_insert(v);
        }
        let mut duplicate_prints = self.prints.merge_presets(prints, &self.vendors);
        let duplicate_sla_prints = self.sla_prints.merge_presets(sla_prints, &self.vendors);
        let duplicate_filaments = self.filaments.merge_presets(filaments, &self.vendors);
        let duplicate_sla_materials = self.sla_materials.merge_presets(sla_materials, &self.vendors);
        let duplicate_printers = self.printers.base.merge_presets(printers.base, &self.vendors);
        append(&mut self.obsolete_presets.prints, obsolete_presets.prints);
        append(&mut self.obsolete_presets.sla_prints, obsolete_presets.sla_prints);
        append(&mut self.obsolete_presets.filaments, obsolete_presets.filaments);
        append(&mut self.obsolete_presets.sla_materials, obsolete_presets.sla_materials);
        append(&mut self.obsolete_presets.printers, obsolete_presets.printers);
        append(&mut duplicate_prints, duplicate_sla_prints);
        append(&mut duplicate_prints, duplicate_filaments);
        append(&mut duplicate_prints, duplicate_sla_materials);
        append(&mut duplicate_prints, duplicate_printers);
        duplicate_prints
    }

    /// PresetBundle.cpp:1927-1939 `void PresetBundle::update_system_maps()`
    fn update_system_maps(&mut self) {
        self.prints.update_map_system_profile_renamed();
        self.sla_prints.update_map_system_profile_renamed();
        self.filaments.update_map_system_profile_renamed();
        self.sla_materials.update_map_system_profile_renamed();
        self.printers.update_map_system_profile_renamed();

        self.prints.update_map_alias_to_profile_name();
        self.sla_prints.update_map_alias_to_profile_name();
        self.filaments.update_map_alias_to_profile_name();
        self.sla_materials.update_map_alias_to_profile_name();
    }
}

/// libslic3r.h:130-137 `template<typename T> inline void append(std::vector<T>& dest, std::vector<T>&& src)`
fn append<T>(dest: &mut Vec<T>, src: Vec<T>) {
    if dest.is_empty() {
        *dest = src;
    } else {
        dest.extend(src);
    }
}

/// PresetBundle.cpp:1941-1947 `static inline std::string remove_ini_suffix(const std::string &name)`
fn remove_ini_suffix(name: &str) -> String {
    let mut out = name.to_string();
    if iends_with(&out, ".ini") {
        out.truncate(out.len() - 4);
    }
    out
}

/// PresetBundle.cpp:2083 `static bool upgraded` of quick_fix_for_filaments_due_to_upgrade.
static QUICK_FIX_UPGRADED: AtomicBool = AtomicBool::new(false);

impl PresetBundle {
    /// PresetBundle.cpp:1949-1957 `void PresetBundle::load_installed_printers(const AppConfig &config)`:
    /// set the "enabled" flag for printer vendors, printer models and printer variants based on
    /// the user configuration. If the "vendor" section is missing, enable all models and variants
    /// of the particular vendor.
    pub fn load_installed_printers(&mut self, config: &AppConfig) {
        self.update_system_maps();
        let begin = self.printers.begin();
        for preset in self.printers.m_presets[begin..].iter_mut() {
            preset.set_visible_from_appconfig(config);
        }
    }

    /// PresetBundle.cpp:1959-1971 `const std::string& PresetBundle::get_preset_name_by_alias(const Preset::Type& preset_type, const std::string& alias) const`
    pub fn get_preset_name_by_alias(&self, preset_type: Type, alias: &str) -> String {
        // there are not aliases for Printers profiles
        if preset_type == Type::TypePrinter || preset_type == Type::TypeInvalid {
            return alias.to_string();
        }

        let presets: &PresetCollection = match preset_type {
            Type::TypePrint => &self.prints,
            Type::TypeSlaPrint => &self.sla_prints,
            Type::TypeFilament => &self.filaments,
            _ => &self.sla_materials,
        };

        presets.get_preset_name_by_alias(alias)
    }

    /// PresetBundle.cpp:1973-1981 `const int PresetBundle::get_required_hrc_by_filament_id(const std::string& filament_id) const`:
    /// BBS: get filament required hrc by filament type.
    pub fn get_required_hrc_by_filament_id(&self, filament_id: &str) -> i32 {
        for preset in self.filaments.m_presets.iter() {
            if !preset.filament_id.is_empty() && preset.filament_id == filament_id && preset.config.has("required_nozzle_HRC") {
                return preset.config.opt_int_at("required_nozzle_HRC", 0);
            }
        }

        0
    }

    /// PresetBundle.cpp:1983-2013 `void PresetBundle::save_changes_for_preset(const std::string& new_name, Preset::Type type, const std::vector<std::string>& unselected_options, bool save_to_project)`:
    /// BBS: add project embedded preset logic.
    pub fn save_changes_for_preset(&mut self, new_name: &str, r#type: Type, unselected_options: &[String], save_to_project: bool) -> CfgResult<()> {
        // if we want to save just some from selected options
        if !unselected_options.is_empty() {
            let presets = self.collection_for_type(r#type);
            // revert unselected options to the old values
            let selected_config = presets.get_selected_preset().config.clone();
            presets.get_edited_preset_mut().config.apply_only(&selected_config, unselected_options, false)?;
        }

        if r#type == Type::TypePrinter {
            self.stamp_printer_nozzle_diameter_mode()?;
        }

        // Save the preset into Slic3r::data_dir / presets / section_name / preset_name.ini
        //BBS: add project embedded preset logic
        //presets.save_current_preset(new_name);
        self.collection_for_type(r#type).save_current_preset(new_name, false, save_to_project, None, None)?;
        // Mark the print & filament enabled if they are compatible with the currently selected preset.
        // If saving the preset changes compatibility with other presets, keep the now incompatible dependent presets selected, however with a "red flag" icon showing that they are no more compatible.
        self.update_compatible(PresetSelectCompatibleType::Never)?;

        if r#type == Type::TypeFilament {
            // synchronize the first filament presets.
            let name = self.filaments.get_selected_preset_name();
            self.set_filament_preset(0, &name);
        }
        Ok(())
    }

    /// `type == TYPE_PRINT ? prints : type == TYPE_SLA_PRINT ? sla_prints : type == TYPE_FILAMENT ? filaments : type == TYPE_SLA_MATERIAL ? sla_materials : printers`
    fn collection_for_type(&mut self, r#type: Type) -> &mut PresetCollection {
        match r#type {
            Type::TypePrint => &mut self.prints,
            Type::TypeSlaPrint => &mut self.sla_prints,
            Type::TypeFilament => &mut self.filaments,
            Type::TypeSlaMaterial => &mut self.sla_materials,
            _ => &mut self.printers.base,
        }
    }

    /// PresetBundle.cpp:2015-2075 `void PresetBundle::load_installed_filaments(AppConfig &config)`
    fn load_installed_filaments(&mut self, config: &mut AppConfig) -> CfgResult<()> {
        //if (! config.has_section(AppConfig::SECTION_FILAMENTS)
        //    || config.get_section(AppConfig::SECTION_FILAMENTS).empty()) {
        // Compatibility with the PrusaSlicer 2.1.1 and older, where the filament profiles were not installable yet.
        // Find all filament profiles, which are compatible with installed printers, and act as if these filament profiles
        // were installed.
        // std::unordered_set<const Preset*>: only the set of names matters below (each is set to "true").
        let mut compatible_filaments: BTreeSet<usize> = BTreeSet::new();
        for printer in self.printers.iter() {
            if printer.is_visible
                && printer.printer_technology() == PrinterTechnology::PtFFF
                && printer.vendor.as_ref().map_or(false, |v| !v.models.is_empty())
            {
                let mut add_default_materials = true;
                if config.has_section(AppConfig::SECTION_FILAMENTS) {
                    let installed_filament = config.get_section(AppConfig::SECTION_FILAMENTS);
                    for (filament_first, _) in installed_filament.iter() {
                        let filament = self.filaments.find_preset_real(filament_first, false, true);
                        if let Some(filament) = filament {
                            if is_compatible_with_printer(
                                &PresetWithVendorProfile::new(filament, filament.vendor.as_deref()),
                                &PresetWithVendorProfile::new(printer, printer.vendor.as_deref()),
                                None,
                            ) {
                                //already has compatible filament
                                add_default_materials = false;
                                break;
                            }
                        }
                    }
                }

                if !add_default_materials {
                    continue;
                }

                let printer_model = match preset_utils::system_printer_model(printer) {
                    Some(m) => m,
                    // can not find printer_model for printer
                    None => continue,
                };
                for default_filament in printer_model.default_materials.iter() {
                    if let Some(i) = find_preset_index(&self.filaments, default_filament, true) {
                        if self.filaments_preset_at(i).is_system {
                            compatible_filaments.insert(i);
                        }
                    }
                }
            }
        }
        // and mark these filaments as installed, therefore this code will not be executed at the next start of the application.
        for &i in compatible_filaments.iter() {
            let name = self.filaments_preset_at(i).name.clone();
            config.set(AppConfig::SECTION_FILAMENTS, &name, "true");
        }
        //}

        let begin = self.filaments.begin();
        for preset in self.filaments.m_presets[begin..].iter_mut() {
            preset.set_visible_from_appconfig(config);
        }

        self.quick_fix_for_filaments_due_to_upgrade(config)
    }

    /// A preset of `filaments` by the index `find_preset_index` gives.
    fn filaments_preset_at(&self, i: usize) -> &Preset {
        if i == EDITED_PRESET_IDX {
            self.filaments.get_edited_preset()
        } else {
            &self.filaments.m_presets[i]
        }
    }

    /// PresetBundle.cpp:2077-2119 `void PresetBundle::quick_fix_for_filaments_due_to_upgrade(AppConfig &config)`
    fn quick_fix_for_filaments_due_to_upgrade(&mut self, config: &mut AppConfig) -> CfgResult<()> {
        // for a visible/selected filament recorded in BambuStudio.conf fialments section
        // make all filaments of the same series(vendor+type) visible no matter the corresponding machine is selected or not
        // this is also the behaviour of the configure guide at fist startup of a clean install
        // filaments are always selected as a group by type, which is filament.alias in code
        if QUICK_FIX_UPGRADED.load(Ordering::SeqCst) {
            return Ok(());
        }
        QUICK_FIX_UPGRADED.store(true, Ordering::SeqCst);

        if !config.has_section(AppConfig::SECTION_FILAMENTS) {
            return Ok(());
        }

        let old_version = config.orig_version().clone();
        // Semver(const std::string&) throws when the text is not a version.
        let version_str = config.get_key("version");
        let cur_version = Semver::parse(&version_str).ok_or_else(|| runtime_error(format!("Could not parse version string: {}", version_str)))?;
        if old_version == cur_version {
            return Ok(());
        }

        let installed_filament = config.get_section(AppConfig::SECTION_FILAMENTS).clone();

        let mut visible_aliases: BTreeSet<String> = BTreeSet::new();
        for (name, _) in installed_filament.iter() {
            let filament = match self.filaments.find_preset_real(name, false, true) {
                Some(f) => f,
                None => continue,
            };
            if !filament.is_visible || filament.alias.is_empty() {
                continue;
            }
            visible_aliases.insert(filament.alias.clone());
        }

        let printer_indices: Vec<usize> =
            (self.printers.begin()..self.printers.end()).filter(|&i| self.printers.m_presets[i].printer_technology() == PrinterTechnology::PtFFF).collect();

        // For each invisible filament, check if it should become visible
        for fi in self.filaments.begin()..self.filaments.end() {
            {
                let filament = &self.filaments.m_presets[fi];
                if filament.is_visible || filament.alias.is_empty() || !filament.is_system {
                    continue;
                }
            }

            for &pi in printer_indices.iter() {
                let compatible = {
                    let printer_profile = self.printers.get_preset_with_vendor_profile(&self.printers.m_presets[pi]);
                    let filament = &self.filaments.m_presets[fi];
                    is_compatible_with_printer(&self.filaments.get_preset_with_vendor_profile(filament), &printer_profile, None)
                };
                if !compatible {
                    continue;
                }
                if !visible_aliases.contains(&self.filaments.m_presets[fi].alias) {
                    continue;
                }
                self.filaments.m_presets[fi].is_visible = true;
                let name = self.filaments.m_presets[fi].name.clone();
                config.set(AppConfig::SECTION_FILAMENTS, &name, "true");
            }
        }
        Ok(())
    }

    /// PresetBundle.cpp:2121-2142 `void PresetBundle::load_installed_sla_materials(AppConfig &config)`
    fn load_installed_sla_materials(&mut self, config: &mut AppConfig) {
        if !config.has_section(AppConfig::SECTION_MATERIALS) {
            let mut comp_sla_materials: BTreeSet<String> = BTreeSet::new();
            // Compatibility with the PrusaSlicer 2.1.1 and older, where the SLA material profiles were not installable yet.
            // Find all SLA material profiles, which are compatible with installed printers, and act as if these SLA material profiles
            // were installed.
            for printer in self.printers.iter() {
                if printer.is_visible && printer.printer_technology() == PrinterTechnology::PtSLA {
                    let printer_with_vendor_profile = self.printers.get_preset_with_vendor_profile(printer);
                    for material in self.sla_materials.iter() {
                        if material.is_system
                            && is_compatible_with_printer(&self.sla_materials.get_preset_with_vendor_profile(material), &printer_with_vendor_profile, None)
                        {
                            comp_sla_materials.insert(material.name.clone());
                        }
                    }
                }
            }
            // and mark these SLA materials as installed, therefore this code will not be executed at the next start of the application.
            for material in comp_sla_materials.iter() {
                config.set(AppConfig::SECTION_MATERIALS, material, "true");
            }
        }

        let begin = self.sla_materials.begin();
        for preset in self.sla_materials.m_presets[begin..].iter_mut() {
            preset.set_visible_from_appconfig(config);
        }
    }
}

/// The index of `collection.find_preset(name, false, real)`: an index into `m_presets`, or
/// `EDITED_PRESET_IDX` for the edited preset.
fn find_preset_index(collection: &PresetCollection, name: &str, real: bool) -> Option<usize> {
    let p = collection.find_preset_real(name, false, real)?;
    if std::ptr::eq(p, collection.get_edited_preset()) {
        return Some(EDITED_PRESET_IDX);
    }
    collection.m_presets.iter().position(|q| std::ptr::eq(q, p))
}

/// FilamentMixer.cpp:262-315 `parse_gradient_curve(s).empty()`: fewer than two valid points.
///
/// FIDELITY-NOTE: FilamentMixer's gradient-curve functions are not in the crate
/// (`filament_mixer.rs`); this reproduces the part of `parse_gradient_curve` that decides whether
/// the curve is empty: '|'-separated segments (`std::getline`), each of 2 or 4 comma-separated
/// fields whose first two parse with `std::stod`.
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
/// (see gradient_curve_is_empty).
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

/// `std::stoi(s)`: leading white space, a sign, decimal digits; `std::invalid_argument` without a
/// digit and `std::out_of_range` beyond int (both escape the C++ callers).
fn std_stoi(s: &str) -> CfgResult<i32> {
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
        return Err(runtime_error("stoi: no conversion"));
    }
    let v = if neg { -v } else { v };
    if v < i32::MIN as i64 || v > i32::MAX as i64 {
        return Err(runtime_error("stoi: out of range"));
    }
    Ok(v as i32)
}

/// `values.resize(n)` of `ConfigOptionStrings::resize(n)` (Config.hpp:614-627): new values copy
/// the last one; an empty vector cannot grow without a default (it throws).
fn option_resize(config: &mut DynamicConfig, key: &str, n: usize) -> CfgResult<()> {
    let opt = config.option_mut(key, false)?.expect("option");
    opt.as_vector_base_mut().expect("vector").resize(n, None)
}

/// `container.erase(begin() + id)` if it is longer than `id`, else `container.resize(id)`
/// (PresetBundle.cpp:2606-2612).
fn erase_or_resize<T: Clone + Default>(container: &mut Vec<T>, to_del_flament_id: usize) {
    if container.len() > to_del_flament_id {
        container.remove(to_del_flament_id);
    } else {
        container.resize(to_del_flament_id, T::default());
    }
}

/// The comma-joined "0"/"1" of a ConfigOptionBools (PresetBundle.cpp:2448-2455 and the like).
fn join_bools(values: &[u8]) -> String {
    let mut s = String::new();
    for (i, v) in values.iter().enumerate() {
        if i > 0 {
            s += ",";
        }
        s += if *v != 0 { "1" } else { "0" };
    }
    s
}

impl PresetBundle {
    /// PresetBundle.cpp:2144-2393 `void PresetBundle::load_selections(AppConfig &config, const PresetPreferences& preferred_selection)`:
    /// load selections (current print, current filaments, current printer) from config.ini. This
    /// is done on application start up or after updates are applied.
    pub fn load_selections(&mut self, config: &mut AppConfig, preferred_selection: &PresetPreferences) -> CfgResult<()> {
        // Update visibility of presets based on application vendor / model / variant configuration.
        self.load_installed_printers(config);

        // Update visibility of filament and sla material presets
        self.load_installed_filaments(config)?;
        self.load_installed_sla_materials(config);

        // Parse the initial print / filament / printer profile names.
        let mut initial_print_profile_name = remove_ini_suffix(&config.get("presets", PRESET_PRINT_NAME));
        let initial_sla_print_profile_name = remove_ini_suffix(&config.get("presets", PRESET_SLA_PRINT_NAME));
        let mut initial_filament_profile_name = remove_ini_suffix(&config.get("presets", PRESET_FILAMENT_NAME));
        let initial_sla_material_profile_name = remove_ini_suffix(&config.get("presets", PRESET_SLA_MATERIALS_NAME));
        let initial_printer_profile_name = remove_ini_suffix(&config.get("presets", PRESET_PRINTER_NAME));

        // Activate print / filament / printer profiles from either the config,
        // or from the preferred_model_id suggestion passed in by ConfigWizard.
        // If the printer profile enumerated by the config are not visible, select an alternate preset.
        // Do not select alternate profiles for the print / filament profiles as those presets
        // will be selected by the following call of this->update_compatible(PresetSelectCompatibleType::Always).

        let initial_printer = find_preset_index(&self.printers, &initial_printer_profile_name, false);
        // If executed due to a Config Wizard update, preferred_printer contains the first newly installed printer, otherwise nullptr.
        let preferred_printer: Option<usize> = self
            .printers
            .find_system_preset_by_model_and_variant(&preferred_selection.printer_model_id, &preferred_selection.printer_variant)
            .map(|p| self.printers.m_presets.iter().position(|q| std::ptr::eq(q, p)).expect("preset of the collection"));
        let select_name = match preferred_printer {
            Some(i) => self.printers.m_presets[i].name.clone(),
            None => initial_printer_profile_name.clone(),
        };
        self.printers.select_preset_by_name(&select_name, true, false);

        //BBS: set default print/filament profiles to BBL's default setting
        if let Some(pi) = preferred_printer {
            let pp = &self.printers.m_presets[pi];
            let prefered_print_profile = opt_string(&pp.config, "default_print_profile").clone();
            if initial_print_profile_name == "Default Setting" && !prefered_print_profile.is_empty() {
                initial_print_profile_name = prefered_print_profile;
            }

            let prefered_filament_profiles = strings_ref(&pp.config, "default_filament_profile").expect("default_filament_profile");
            if initial_filament_profile_name == "Default Filament" && !prefered_filament_profiles.is_empty() {
                initial_filament_profile_name = prefered_filament_profiles[0].clone();
            }
        }

        // Selects the profile, leaves it to -1 if the initial profile name is empty or if it was not found.
        self.prints.select_preset_by_name_strict(&initial_print_profile_name);
        self.filaments.select_preset_by_name_strict(&initial_filament_profile_name);
        self.sla_prints.select_preset_by_name_strict(&initial_sla_print_profile_name);
        self.sla_materials.select_preset_by_name_strict(&initial_sla_material_profile_name);

        // Load the names of the other filament profiles selected for a multi-material printer.
        // Load it even if the current printer technology is SLA.
        // The possibly excessive filament names will be later removed with this->update_multi_material_filament_presets()
        // once the FFF technology gets selected.
        self.filament_presets = vec![self.filaments.get_selected_preset_name()];
        for i in 1u32..1000 {
            let name = format!("filament_{:02}", i);
            if !config.has("presets", &name) {
                break;
            }
            self.filament_presets.push(remove_ini_suffix(&config.get("presets", &name)));
        }

        // Load data from AppConfig to ProjectConfig when Studio is initialized.
        let mut filament_colors: Vec<String> = Vec::new();
        if config.has("presets", "filament_colors") {
            filament_colors = boost_split(&config.get("presets", "filament_colors"), &[',']);
        }
        filament_colors.resize(self.filament_presets.len(), "#00AE42".to_string());
        *strings_mut(&mut self.project_config, "filament_colour", false)?.expect("filament_colour") = filament_colors.clone();

        let mut multi_filament_colors: Vec<String> = Vec::new();
        if config.has("presets", "filament_multi_colors") {
            multi_filament_colors = boost_split(&config.get("presets", "filament_multi_colors"), &[',']);
        }
        if multi_filament_colors.is_empty() {
            *strings_mut(&mut self.project_config, "filament_multi_colour", false)?.expect("filament_multi_colour") = filament_colors.clone();
        } else {
            *strings_mut(&mut self.project_config, "filament_multi_colour", false)?.expect("filament_multi_colour") = multi_filament_colors;
        }

        let mut filament_color_types: Vec<String> = Vec::new();
        if config.has("presets", "filament_color_types") {
            filament_color_types = boost_split(&config.get("presets", "filament_color_types"), &[',']);
        }
        filament_color_types.resize(self.filament_presets.len(), "1".to_string());
        *strings_mut(&mut self.project_config, "filament_colour_type", false)?.expect("filament_colour_type") = filament_color_types;

        let filament_maps: Vec<i32> = vec![1; filament_colors.len()];
        *ints_mut(&mut self.project_config, "filament_map", false)?.expect("filament_map") = filament_maps;

        let filament_nozzle_maps: Vec<i32> = vec![0; filament_colors.len()];
        *ints_mut(&mut self.project_config, "filament_nozzle_map", false)?.expect("filament_nozzle_map") = filament_nozzle_maps;

        let filament_volume_maps: Vec<i32> = vec![NozzleVolumeType::NvtStandard as i32; filament_colors.len()];
        *ints_mut(&mut self.project_config, "filament_volume_map", false)?.expect("filament_volume_map") = filament_volume_maps;

        let mut extruder_ams_count_str: Vec<String> = Vec::new();
        if config.has("presets", "extruder_ams_count") {
            extruder_ams_count_str = boost_split(&config.get("presets", "extruder_ams_count"), &[',']);
        }
        self.extruder_ams_counts = get_extruder_ams_count(&extruder_ams_count_str);
        self.extruder_ams_selections.clear();

        let parse_doubles = |text: String| -> CfgResult<Vec<f64>> { boost_split(&text, &['|']).iter().map(|s| lexical_cast_double(s)).collect() };
        if config.has("presets", "flush_volumes_matrix") {
            let v = parse_doubles(config.get("presets", "flush_volumes_matrix"))?;
            *floats_mut(&mut self.project_config, "flush_volumes_matrix", false)?.expect("flush_volumes_matrix") = v;
        }
        if config.has("presets", "flush_volumes_vector") {
            let v = parse_doubles(config.get("presets", "flush_volumes_vector"))?;
            *floats_mut(&mut self.project_config, "flush_volumes_vector", false)?.expect("flush_volumes_vector") = v;
        }
        if config.has("app", "flush_multiplier") {
            let v = parse_doubles(config.get("app", "flush_multiplier"))?;
            *floats_mut(&mut self.project_config, "flush_multiplier", false)?.expect("flush_multiplier") = v;
        }
        if config.has("app", "flush_multiplier_fast") {
            let v = parse_doubles(config.get("app", "flush_multiplier_fast"))?;
            *floats_mut(&mut self.project_config, "flush_multiplier_fast", false)?.expect("flush_multiplier_fast") = v;
        }

        // Mixed filament metadata
        let n_filaments = self.filament_presets.len();
        if config.has("presets", "filament_is_mixed") {
            let parts = boost_split(&config.get("presets", "filament_is_mixed"), &[',']);
            let vals = bools_mut(&mut self.project_config, "filament_is_mixed", false)?.expect("filament_is_mixed");
            vals.clear();
            for p in &parts {
                vals.push((p == "1") as u8);
            }
            vals.resize(n_filaments, 0);
        }
        if config.has("presets", "filament_mixed_components") {
            let mut parts = boost_split(&config.get("presets", "filament_mixed_components"), &['|']);
            parts.resize(n_filaments, String::new());
            *strings_mut(&mut self.project_config, "filament_mixed_components", false)?.expect("filament_mixed_components") = parts;
        }
        if config.has("presets", "filament_mixed_sublayer_ratios") {
            let mut parts = boost_split(&config.get("presets", "filament_mixed_sublayer_ratios"), &['|']);
            parts.resize(n_filaments, String::new());
            *strings_mut(&mut self.project_config, "filament_mixed_sublayer_ratios", false)?.expect("filament_mixed_sublayer_ratios") = parts;
        }
        if config.has("presets", "filament_mixed_gradient") {
            let parts = boost_split(&config.get("presets", "filament_mixed_gradient"), &[',']);
            let vals = bools_mut(&mut self.project_config, "filament_mixed_gradient", false)?.expect("filament_mixed_gradient");
            vals.clear();
            for p in &parts {
                vals.push((p == "1") as u8);
            }
            vals.resize(n_filaments, 0);
        }
        if config.has("presets", "filament_mixed_gradient_range") {
            let mut parts = boost_split(&config.get("presets", "filament_mixed_gradient_range"), &['|']);
            parts.resize(n_filaments, String::new());
            *strings_mut(&mut self.project_config, "filament_mixed_gradient_range", false)?.expect("filament_mixed_gradient_range") = parts;
        }
        {
            // Old projects may not contain filament_mixed_gradient_curve; resize unconditionally.
            let has_curve = config.has("presets", "filament_mixed_gradient_curve");
            let raw = config.get("presets", "filament_mixed_gradient_curve");
            let vals = strings_mut(&mut self.project_config, "filament_mixed_gradient_curve", false)?.expect("filament_mixed_gradient_curve");
            if has_curve {
                // New format: C-style escaped ";" (escape_strings_cstyle), safe for curve values
                // that contain "|" (the intra-slot control-point delimiter). Legacy format used
                // "|" as the inter-slot delimiter, which collided with the intra-slot "|" and
                // split multi-point curves across slots. Disambiguate by size: the new format
                // always round-trips to exactly n_filaments elements; a legacy "|" split of a
                // corrupted curve yields != n_filaments. Invariant: a non-empty curve implies a
                // mixed gradient slot, which requires n_filaments >= 3, so the new format's
                // escaped form always contains ";" and unescapes to size == n_filaments.
                let mut parts: Vec<String> = Vec::new();
                if unescape_strings_cstyle(&raw, &mut parts) && parts.len() == n_filaments {
                    *vals = parts;
                } else {
                    *vals = boost_split(&raw, &['|']);
                }
            }
            vals.resize(n_filaments, String::new());
            // Heal legacy corruption: clear any non-empty slot that ended up with < 2 points
            // (e.g. a curve split across slots by the old "|" delimiter). Falls back to linear.
            sanitize_mixed_gradient_curve_array(vals);
        }
        {
            // 兜底：旧工程没有 filament_mixed_gradient_per_part 时也保证数组长度与 n_filaments 一致
            let has = config.has("presets", "filament_mixed_gradient_per_part");
            let raw = config.get("presets", "filament_mixed_gradient_per_part");
            let vals = bools_mut(&mut self.project_config, "filament_mixed_gradient_per_part", false)?.expect("filament_mixed_gradient_per_part");
            if has {
                let parts = boost_split(&raw, &[',']);
                vals.clear();
                for p in &parts {
                    vals.push((p == "1") as u8);
                }
            }
            vals.resize(n_filaments, 0);
        }

        // Update visibility of presets based on their compatibility with the active printer.
        // Always try to select a compatible print and filament preset to the current printer preset,
        // as the application may have been closed with an active "external" preset, which does not
        // exist.
        self.update_compatible(PresetSelectCompatibleType::Always)?;
        self.update_multi_material_filament_presets(usize::MAX)?;

        if initial_printer.is_some() && (preferred_printer.is_none() || initial_printer == preferred_printer) {
            // Don't run the following code, as we want to activate default filament / SLA material profiles when installing and selecting a new printer.
            // Only run this code if just a filament / SLA material was installed by Config Wizard for an active Printer.
            let printer_technology = self.printers.get_selected_preset().printer_technology();
            if printer_technology == PrinterTechnology::PtFFF && !preferred_selection.filament.is_empty() {
                let preferred_preset_name = self.get_preset_name_by_alias(Type::TypeFilament, &preferred_selection.filament);
                let it = self.filaments.find_preset_internal(&preferred_preset_name);
                if it != self.filaments.end()
                    && self.filaments.m_presets[it].name == preferred_preset_name
                    && self.filaments.m_presets[it].is_visible
                    && self.filaments.m_presets[it].is_compatible
                {
                    self.filaments.select_preset_by_name_strict(&preferred_preset_name);
                    self.filament_presets[0] = self.filaments.get_selected_preset_name();
                }
            } else if printer_technology == PrinterTechnology::PtSLA && !preferred_selection.sla_material.is_empty() {
                let preferred_preset_name = self.get_preset_name_by_alias(Type::TypeSlaMaterial, &preferred_selection.sla_material);
                let it = self.sla_materials.find_preset_internal(&preferred_preset_name);
                if it != self.sla_materials.end() && self.sla_materials.m_presets[it].is_visible && self.sla_materials.m_presets[it].is_compatible {
                    self.sla_materials.select_preset_by_name_strict(&preferred_preset_name);
                }
            }
        }

        let mut first_visible_filament_name = String::new();
        for k in 0..self.filament_presets.len() {
            // Only replace a slot's preset when it genuinely can no longer be used: the preset was
            // deleted/renamed (end()) or is incompatible with the current printer. A preset that is
            // merely hidden (is_visible == false, e.g. the user just unchecked it in the "system
            // materials" filter) still exists and may still be perfectly usable — hiding only keeps
            // it out of pick lists for *new* selections, it must not evict a slot already using it.
            let it = self.filaments.find_preset_internal(&self.filament_presets[k]);
            if it == self.filaments.end() || !self.filaments.m_presets[it].is_compatible {
                if first_visible_filament_name.is_empty() {
                    first_visible_filament_name = self.filaments.first_compatible().name.clone();
                }
                self.filament_presets[k] = first_visible_filament_name.clone();
            }
        }

        // The history of each printer lives in the app config this bundle loads its selections from.
        // FIDELITY-NOTE: the AppConfig is shared with the inventory for this call only (see the module notes).
        let history = Arc::new(Mutex::new(std::mem::take(config)));
        self.nozzle_inventory.set_history(Some(history.clone()));
        let r = self.nozzle_select_printer();
        self.nozzle_inventory.set_history(None);
        *config = match Arc::try_unwrap(history) {
            Ok(m) => m.into_inner().unwrap_or_else(|e| e.into_inner()),
            Err(_) => unreachable!("the inventory keeps no other reference to the history"),
        };
        r?;

        // Parse the initial physical printer name.
        let initial_physical_printer_name = remove_ini_suffix(&config.get("presets", "physical_printer"));

        // Activate physical printer from the config
        if !initial_physical_printer_name.is_empty() {
            self.physical_printers.select_printer(&initial_physical_printer_name);
        }
        Ok(())
    }

    /// PresetBundle.cpp:2395-2491 `void PresetBundle::export_selections(AppConfig &config)`:
    /// export selections (current print, current filaments, current printer) into config.ini.
    pub fn export_selections(&self, config: &mut AppConfig) {
        config.clear_section("presets");
        // clear_section() above wipes the whole presets section, so any key not re-emitted here is lost on save.
        // Re-emit the per-extruder AMS counts from the source-of-truth member (mirrors AMSCountPopupWindow::SetAMSCount).
        config.set("presets", "extruder_ams_count", &boost_join(&save_extruder_ams_count_to_string(&self.extruder_ams_counts), ","));
        config.set("presets", PRESET_PRINT_NAME, &self.prints.get_selected_preset_name());
        config.set("presets", PRESET_FILAMENT_NAME, &self.filament_presets[0]);
        for i in 1..self.filament_presets.len() {
            let name = format!("filament_{:02}", i);
            config.set("presets", &name, &self.filament_presets[i]);
        }
        // Load project config data into app config
        let pc = &self.project_config;
        let filament_colors = boost_join(strings_ref(pc, "filament_colour").expect("filament_colour"), ",");
        config.set("presets", "filament_colors", &filament_colors);

        // Load filament multi color data into app config
        let filament_multi_colors = boost_join(strings_ref(pc, "filament_multi_colour").expect("filament_multi_colour"), ",");
        config.set("presets", "filament_multi_colors", &filament_multi_colors);

        // Load filament color type data into app config
        let filament_color_types = boost_join(strings_ref(pc, "filament_colour_type").expect("filament_colour_type"), ",");
        config.set("presets", "filament_color_types", &filament_color_types);

        let join_doubles = |key: &str| -> String {
            let v: Vec<String> = floats_ref(pc, key).unwrap_or_else(|| panic!("{key}")).iter().map(|d| std_to_string_f64(*d)).collect();
            boost_join(&v, "|")
        };
        let flush_volumes_matrix = join_doubles("flush_volumes_matrix");
        config.set("presets", "flush_volumes_matrix", &flush_volumes_matrix);
        let flush_volumes_vector = join_doubles("flush_volumes_vector");
        config.set("presets", "flush_volumes_vector", &flush_volumes_vector);

        config.set("presets", PRESET_PRINTER_NAME, &self.printers.get_selected_preset_name());

        let flush_multiplier_str = join_doubles("flush_multiplier");
        config.set_app("flush_multiplier", &flush_multiplier_str);
        let flush_multiplier_fast_str = join_doubles("flush_multiplier_fast");
        config.set_app("flush_multiplier_fast", &flush_multiplier_fast_str);

        // Mixed filament metadata
        if let Some(opt) = bools_ref(pc, "filament_is_mixed") {
            config.set("presets", "filament_is_mixed", &join_bools(opt));
        }
        if let Some(opt) = strings_ref(pc, "filament_mixed_components") {
            config.set("presets", "filament_mixed_components", &boost_join(opt, "|"));
        }
        if let Some(opt) = strings_ref(pc, "filament_mixed_sublayer_ratios") {
            config.set("presets", "filament_mixed_sublayer_ratios", &boost_join(opt, "|"));
        }
        if let Some(opt) = bools_ref(pc, "filament_mixed_gradient") {
            config.set("presets", "filament_mixed_gradient", &join_bools(opt));
        }
        if let Some(opt) = strings_ref(pc, "filament_mixed_gradient_range") {
            config.set("presets", "filament_mixed_gradient_range", &boost_join(opt, "|"));
        }
        if let Some(opt) = strings_ref(pc, "filament_mixed_gradient_curve") {
            // Use C-style escaped ";" encoding (same as ConfigOptionStrings::serialize) instead
            // of "|" join: each slot's curve value may itself contain "|" as the control-point
            // delimiter (serialize_gradient_curve), which would collide with "|" as the inter-
            // slot delimiter and split the curve across slots on the next load_selections.
            config.set("presets", "filament_mixed_gradient_curve", &escape_strings_cstyle(opt));
        }
        if let Some(opt) = bools_ref(pc, "filament_mixed_gradient_per_part") {
            config.set("presets", "filament_mixed_gradient_per_part", &join_bools(opt));
        }

        // BBS
        //config.set("presets", "sla_print",    sla_prints.get_selected_preset_name());
        //config.set("presets", "sla_material", sla_materials.get_selected_preset_name());
        //config.set("presets", "physical_printer", physical_printers.get_selected_full_printer_name());
    }

    /// PresetBundle.cpp:2493-2566 `void PresetBundle::set_num_filaments(unsigned int n, std::string new_color)` (BBS)
    pub fn set_num_filaments(&mut self, mut n: u32, new_color: &str) -> CfgResult<()> {
        if n > crate::texture_painting::MAX_FILAMENT_COUNT as u32 {
            n = crate::texture_painting::MAX_FILAMENT_COUNT as u32;
        }
        let old_filament_count = self.filament_presets.len() as u32;
        if n > old_filament_count && old_filament_count != 0 {
            let back = self.filament_presets.last().unwrap().clone();
            self.filament_presets.resize(n as usize, back);
        } else {
            self.filament_presets.resize(n as usize, String::new());
        }
        // A new slot copies its filament preset from the previous one (filament_presets.resize above), so
        // inherit that slot's nozzle diameter / flow too instead of snapping to the first installed
        // Standard nozzle. Read before the resizes below, which is where the padding value is consumed.
        // validate_filament_nozzle_diameter_selections() at the end repairs the inherited pair when the
        // new slot's filament cannot print that flow (e.g. a 2.85mm TPU needs a TPU285 nozzle).
        // Restricted to the machines whose filament panel exposes the per-slot nozzle selector: elsewhere
        // filament_volume_map is a grouping input/result rather than a user choice worth propagating.
        let inherit_nozzle_selection = old_filament_count > 0
            && multi_nozzle_diameter_mode_enabled(&self.project_config)
            && is_single_ext_multi_nozzle(&self.printers.get_edited_preset().config);
        let filament_volume_map = ints_ref(&self.project_config, "filament_volume_map").expect("filament_volume_map").clone();
        let selected_nozzle_diameters = strings_ref(&self.project_config, "filament_diameter_map").expect("filament_diameter_map").clone();
        let inherited_volume = if inherit_nozzle_selection && (old_filament_count as usize) <= filament_volume_map.len() {
            filament_volume_map[old_filament_count as usize - 1]
        } else {
            NozzleVolumeType::NvtStandard as i32
        };
        let inherited_diameter = if inherit_nozzle_selection && (old_filament_count as usize) <= selected_nozzle_diameters.len() {
            selected_nozzle_diameters[old_filament_count as usize - 1].clone()
        } else {
            String::new()
        };

        let n = n as usize;
        option_resize(&mut self.project_config, "filament_colour", n)?;
        option_resize(&mut self.project_config, "filament_multi_colour", n)?;
        option_resize(&mut self.project_config, "filament_colour_type", n)?;
        ints_mut(&mut self.project_config, "filament_map", false)?.expect("filament_map").resize(n, 1);
        self.ams_multi_color_filment.resize(n, Vec::new());
        ints_mut(&mut self.project_config, "filament_nozzle_map", false)?.expect("filament_nozzle_map").resize(n, 0);
        ints_mut(&mut self.project_config, "filament_volume_map", false)?.expect("filament_volume_map").resize(n, inherited_volume);
        strings_mut(&mut self.project_config, "filament_diameter_map", false)?.expect("filament_diameter_map").resize(n, inherited_diameter);

        if let Some(v) = bools_mut(&mut self.project_config, "filament_is_mixed", false)? {
            v.resize(n, 0);
        }
        if let Some(v) = strings_mut(&mut self.project_config, "filament_mixed_components", false)? {
            v.resize(n, String::new());
        }
        if let Some(v) = strings_mut(&mut self.project_config, "filament_mixed_sublayer_ratios", false)? {
            v.resize(n, String::new());
        }
        if let Some(v) = bools_mut(&mut self.project_config, "filament_mixed_gradient", false)? {
            v.resize(n, 0);
        }
        if let Some(v) = strings_mut(&mut self.project_config, "filament_mixed_gradient_range", false)? {
            v.resize(n, String::new());
        }
        if let Some(v) = strings_mut(&mut self.project_config, "filament_mixed_gradient_curve", false)? {
            v.resize(n, String::new());
        }
        if let Some(v) = bools_mut(&mut self.project_config, "filament_mixed_gradient_per_part", false)? {
            v.resize(n, 0);
        }

        //BBS set new filament color to new_color
        if (old_filament_count as usize) < n && !new_color.is_empty() {
            for i in old_filament_count as usize..n {
                strings_mut(&mut self.project_config, "filament_colour", false)?.expect("filament_colour")[i] = new_color.to_string();
                strings_mut(&mut self.project_config, "filament_multi_colour", false)?.expect("filament_multi_colour")[i] = new_color.to_string();
                strings_mut(&mut self.project_config, "filament_colour_type", false)?.expect("filament_colour_type")[i] = "1".to_string(); // default color type
            }
        }

        self.validate_filament_nozzle_diameter_selections(None, false)?;
        self.update_multi_material_filament_presets(usize::MAX)
    }

    /// PresetBundle.cpp:2568-2646 `void PresetBundle::update_num_filaments(unsigned int to_del_flament_id)`
    pub fn update_num_filaments(&mut self, to_del_flament_id: u32) -> CfgResult<()> {
        let to_del = to_del_flament_id as usize;
        self.filament_presets.remove(to_del);

        // update edited_preset
        {
            let edited_name = self.filaments.get_edited_preset().name.clone();
            let edited_preset_deleted = !self.filament_presets.iter().any(|n| *n == edited_name);
            if edited_preset_deleted {
                let front = self.filament_presets[0].clone();
                self.filaments.select_preset_by_name(&front, false, false);
            }
        }

        let pc = &mut self.project_config;
        let filament_color_len = strings_ref(pc, "filament_colour").expect("filament_colour").len();
        if filament_color_len > to_del {
            strings_mut(pc, "filament_colour", false)?.expect("filament_colour").remove(to_del);
            let filament_map = ints_mut(pc, "filament_map", false)?.expect("filament_map");
            if filament_map.len() > to_del {
                filament_map.remove(to_del);
            }
        } else {
            strings_mut(pc, "filament_colour", false)?.expect("filament_colour").resize(to_del, String::new());
            ints_mut(pc, "filament_map", false)?.expect("filament_map").resize(to_del, 1);
        }

        erase_or_resize(strings_mut(pc, "filament_multi_colour", false)?.expect("filament_multi_colour"), to_del);
        erase_or_resize(strings_mut(pc, "filament_colour_type", false)?.expect("filament_colour_type"), to_del);
        erase_or_resize(strings_mut(pc, "filament_diameter_map", false)?.expect("filament_diameter_map"), to_del);
        erase_or_resize(&mut self.ams_multi_color_filment, to_del);

        {
            let is_mixed = bools_ref(pc, "filament_is_mixed").cloned();
            let has_comp = strings_ref(pc, "filament_mixed_components").is_some();
            if let (Some(is_mixed), true) = (&is_mixed, has_comp) {
                let del_is_physical = to_del >= is_mixed.len() || is_mixed[to_del] == 0;
                if del_is_physical {
                    let comp = strings_mut(pc, "filament_mixed_components", false)?.expect("filament_mixed_components");
                    crate::filament_mixer::remap_mixed_components_on_delete(is_mixed, comp, to_del_flament_id + 1);
                }
            }
            if let Some(v) = bools_mut(pc, "filament_is_mixed", false)? {
                erase_or_resize(v, to_del);
            }
            if let Some(v) = strings_mut(pc, "filament_mixed_components", false)? {
                erase_or_resize(v, to_del);
            }
        }
        if let Some(v) = strings_mut(pc, "filament_mixed_sublayer_ratios", false)? {
            erase_or_resize(v, to_del);
        }
        if let Some(v) = bools_mut(pc, "filament_mixed_gradient", false)? {
            erase_or_resize(v, to_del);
        }
        if let Some(v) = strings_mut(pc, "filament_mixed_gradient_range", false)? {
            erase_or_resize(v, to_del);
        }
        if let Some(v) = strings_mut(pc, "filament_mixed_gradient_curve", false)? {
            erase_or_resize(v, to_del);
        }
        if let Some(v) = bools_mut(pc, "filament_mixed_gradient_per_part", false)? {
            erase_or_resize(v, to_del);
        }

        self.update_multi_material_filament_presets(to_del)
    }

    /// PresetBundle.cpp:2648-2652 `bool PresetBundle::is_mixed_filament(size_t idx) const`
    pub fn is_mixed_filament(&self, idx: usize) -> bool {
        bools_ref(&self.project_config, "filament_is_mixed").map_or(false, |v| idx < v.len() && v[idx] != 0)
    }

    /// PresetBundle.cpp:2654-2661 `std::vector<size_t> PresetBundle::physical_filament_config_indices() const`
    pub fn physical_filament_config_indices(&self) -> Vec<usize> {
        (0..self.filament_presets.len()).filter(|&i| !self.is_mixed_filament(i)).collect()
    }

    /// PresetBundle.cpp:2663-2666 `size_t PresetBundle::physical_filament_count() const`
    pub fn physical_filament_count(&self) -> usize {
        self.filament_presets.len().wrapping_sub(self.mixed_filament_count())
    }

    /// PresetBundle.cpp:2668-2675 `size_t PresetBundle::mixed_filament_count() const`
    pub fn mixed_filament_count(&self) -> usize {
        (0..self.filament_presets.len()).filter(|&i| self.is_mixed_filament(i)).count()
    }

    /// `std::find_if(filaments.begin(), filaments.end(), [](auto &f) { return f.is_compatible && filaments.get_preset_base(f) == &f && f.filament_id == filament_id; })`
    fn find_base_filament_by_id(&self, filament_id: &str, mut visit: impl FnMut(&Preset)) -> Option<usize> {
        for i in self.filaments.begin()..self.filaments.end() {
            let f = &self.filaments.m_presets[i];
            visit(f);
            if f.is_compatible && self.filaments.get_preset_base(f).map_or(false, |b| std::ptr::eq(b, f)) && f.filament_id == filament_id {
                return Some(i);
            }
        }
        None
    }

    /// `std::find_if(filaments.begin(), filaments.end(), pred)` over the presets after the defaults.
    fn find_filament_if(&self, pred: impl Fn(&Preset) -> bool) -> Option<usize> {
        (self.filaments.begin()..self.filaments.end()).find(|&i| pred(&self.filaments.m_presets[i]))
    }

    /// PresetBundle.cpp:2677-2738 `void PresetBundle::get_ams_cobox_infos(AMSComboInfo &combox_info, bool skip_ext)`
    pub fn get_ams_cobox_infos(&self, combox_info: &mut AMSComboInfo, skip_ext: bool) {
        combox_info.clear();
        let mut added_filaments: BTreeSet<(String, String)> = BTreeSet::new();
        for (_k, ams) in self.filament_ams_list.iter() {
            let mut filament_id = opt_string_at(ams, "filament_id", 0);
            let filament_color = opt_string_at(ams, "filament_colour", 0);
            let ams_name = opt_string_at(ams, "tray_name", 0);
            if ams_name == "Ext" && skip_ext {
                continue;
            }
            if skip_ext {
                let filament_pair = (ams_name.clone(), filament_id.clone());
                if added_filaments.contains(&filament_pair) {
                    continue;
                }
                added_filaments.insert(filament_pair);
            }
            let filament_changed = !ams.has("filament_changed") || ams.opt_bool("filament_changed");
            let filament_multi_color =
                ams.opt::<ConfigOptionStrings>("filament_multi_colour").expect("filament_multi_colour").values.clone();
            if filament_id.is_empty() {
                continue;
            }
            if !filament_changed && self.filament_presets.len() > combox_info.ams_filament_presets.len() {
                combox_info.ams_filament_presets.push(self.filament_presets[combox_info.ams_filament_presets.len()].clone());
                combox_info.ams_filament_colors.push(filament_color);
                combox_info.ams_multi_color_filment.push(filament_multi_color);
                combox_info.ams_names.push(ams_name);
                continue;
            }
            let mut iter = self.find_base_filament_by_id(&filament_id, |_| {});
            if iter.is_none() {
                // filament_id not found or system or compatible
                let mut filament_type = opt_string_at(ams, "filament_type", 0);
                if !filament_type.is_empty() {
                    filament_type = format!("Generic {}", filament_type);
                    iter = self.find_filament_if(|f| f.is_compatible && f.is_system && f.name.starts_with(&filament_type));
                }
                if iter.is_none() {
                    // Prefer old selection
                    if combox_info.ams_filament_presets.len() < self.filament_presets.len() {
                        combox_info.ams_filament_presets.push(self.filament_presets[combox_info.ams_filament_presets.len()].clone());
                        combox_info.ams_filament_colors.push(filament_color);
                        combox_info.ams_multi_color_filment.push(filament_multi_color);
                        combox_info.ams_names.push(ams_name);
                        continue;
                    }
                    iter = self.find_filament_if(|f| f.is_compatible && f.is_system);
                    if iter.is_none() {
                        continue;
                    }
                }
                filament_id = self.filaments.m_presets[iter.unwrap()].filament_id.clone();
            }
            let _ = filament_id;
            combox_info.ams_filament_presets.push(self.filaments.m_presets[iter.unwrap()].name.clone());
            combox_info.ams_filament_colors.push(filament_color);
            combox_info.ams_multi_color_filment.push(filament_multi_color);
            combox_info.ams_names.push(ams_name);
        }
    }
}

/// PresetBundle.cpp:2742-2758 `struct AmsInfo`: one AMS tray as seen by sync_ams_list, plus the
/// side effects that have to wait until we know whether the tray actually reaches the project
/// filament list.
#[derive(Debug, Clone, Default)]
struct AmsInfo {
    valid: bool,
    is_map: bool,
    filament_color: String,
    filament_color_type: String,
    filament_preset: String,
    mutli_filament_color: Vec<String>,
    // The tray config this entry came from, i.e. the same key stored in `unknowns`.
    source: Option<i32>,
    // Preset to enable once the tray is known to be synced. Null when the tray reuses a preset
    // that is already in the project filament list.
    install_preset: Option<usize>,
    mapping: AMSMapInfo,
}

/// PresetBundle.cpp:2760-2781 `void apply_synced_tray_side_effects(const std::vector<AmsInfo> &ams_infos, const std::set<const DynamicPrintConfig *> &unsynced, std::vector<std::pair<DynamicPrintConfig *, std::string>> &unknowns)`:
/// apply the side effects the first pass over the AMS trays deferred.
fn apply_synced_tray_side_effects(filaments: &mut PresetCollection, ams_infos: &[AmsInfo], unsynced: &BTreeSet<i32>, unknowns: &mut Vec<(i32, String)>) {
    for info in ams_infos {
        if let Some(p) = info.install_preset {
            if !info.source.map_or(false, |s| unsynced.contains(&s)) {
                filaments.m_presets[p].is_visible = true;
            }
        }
    }

    if unsynced.is_empty() {
        return;
    }

    unknowns.retain(|unknown| !unsynced.contains(&unknown.0));
}

/// PresetBundle.cpp:2949-2959 `struct MixedSlotSnapshot` of sync_ams_list.
#[derive(Debug, Clone, Default)]
struct MixedSlotSnapshot {
    preset: String,
    color: String,
    color_type: String,
    mixed_components: String,
    mixed_sublayer_ratios: String,
    mixed_gradient: bool,
    mixed_gradient_range: String,
    mixed_gradient_curve: String,
    mixed_gradient_per_part: bool,
}

impl PresetBundle {
    /// PresetBundle.cpp:2785-3256 `unsigned int PresetBundle::sync_ams_list(std::vector<std::pair<DynamicPrintConfig *, std::string>> &unknowns, bool use_map, std::map<int, AMSMapInfo> &maps, bool enable_append, MergeFilamentInfo &merge_info, bool skip_ext)`
    ///
    /// `unknowns` holds the keys of `filament_ams_list` of the trays (see the module notes).
    pub fn sync_ams_list(
        &mut self,
        unknowns: &mut Vec<(i32, String)>,
        use_map: bool,
        maps: &mut BTreeMap<i32, AMSMapInfo>,
        enable_append: bool,
        merge_info: &mut MergeFilamentInfo,
        skip_ext: bool,
    ) -> CfgResult<u32> {
        self.ams_sync_substitutions.clear();
        let mut ams_filament_presets: Vec<String> = Vec::new();
        let mut ams_filament_colors: Vec<String> = Vec::new();
        let mut ams_filament_color_types: Vec<String> = Vec::new();
        let mut ams_array_maps: Vec<AMSMapInfo> = Vec::new();
        let mut synced_ams_maps: Vec<AMSMapInfo> = Vec::new();
        self.ams_multi_color_filment.clear();
        let mut ams_infos: Vec<AmsInfo> = Vec::new();
        let mut added_filaments: BTreeSet<(String, String)> = BTreeSet::new();
        let keys: Vec<i32> = self.filament_ams_list.keys().copied().collect();
        for key in keys {
            let ams = &self.filament_ams_list[&key];
            let mut filament_id = opt_string_at(ams, "filament_id", 0);
            let tray_name = opt_string_at(ams, "tray_name", 0);
            if tray_name == "Ext" && skip_ext {
                continue;
            }
            if skip_ext {
                let filament_pair = (tray_name.clone(), filament_id.clone());
                if added_filaments.contains(&filament_pair) {
                    continue;
                }
                added_filaments.insert(filament_pair);
            }
            let filament_color = opt_string_at(ams, "filament_colour", 0);
            let filament_color_type = opt_string_at(ams, "filament_colour_type", 0);
            let filament_changed = !ams.has("filament_changed") || ams.opt_bool("filament_changed");
            let mut filament_multi_color =
                ams.opt::<ConfigOptionStrings>("filament_multi_colour").expect("filament_multi_colour").values.clone();
            let ams_id = opt_string_at(ams, "ams_id", 0);
            let slot_id = opt_string_at(ams, "slot_id", 0);
            ams_infos.push(AmsInfo { valid: !filament_id.is_empty(), is_map: false, filament_color: filament_color.clone(), ..Default::default() });
            ams_infos.last_mut().unwrap().source = Some(key);
            let mut temp = AMSMapInfo { ams_id: ams_id.clone(), slot_id: slot_id.clone(), ..Default::default() };
            if let Some(tray_diameters) = floats_ref(ams, "tray_nozzle_diameter") {
                if !tray_diameters.is_empty() && tray_diameters[0] > 0. {
                    temp.nozzle_diameter = format_diameter_to_str(tray_diameters[0]);
                }
            }
            // Diameter alone does not name a nozzle on a rack carrying the same diameter in two flows,
            // and the slot keeps whichever flow it already had -- so a tray on a high-flow nozzle synced
            // onto a slot sitting at standard flow silently stayed standard. Take the flow here too.
            if let Some(tray_volume_types) = ints_ref(ams, "tray_nozzle_volume_type") {
                if !tray_volume_types.is_empty() {
                    temp.nozzle_volume_type = Some(NozzleVolumeType::from_i32(tray_volume_types[0]));
                }
            }
            ams_infos.last_mut().unwrap().mapping = temp.clone();
            ams_array_maps.push(temp.clone());
            if filament_id.is_empty() {
                if use_map {
                    let mut j = maps.len() as i32 - 1;
                    while j >= 0 {
                        // maps[j]: operator[] inserts a default entry for a missing key.
                        let e = maps.entry(j).or_default();
                        if e.slot_id == slot_id && e.ams_id == ams_id {
                            maps.remove(&j);
                        }
                        j -= 1;
                    }
                    ams_filament_presets.push("Generic PLA".to_string()); //for unknow matieral
                    let default_unknown_color = "#CECECE";
                    ams_filament_colors.push(default_unknown_color.to_string());
                    ams_filament_color_types.push("1".to_string());
                    if filament_multi_color.is_empty() {
                        filament_multi_color.push(default_unknown_color.to_string());
                    }
                    self.ams_multi_color_filment.push(filament_multi_color);
                }
                continue;
            }
            if !filament_changed && self.filament_presets.len() > ams_filament_presets.len() {
                ams_filament_presets.push(self.filament_presets[ams_filament_presets.len()].clone());
                ams_filament_colors.push(filament_color);
                ams_filament_color_types.push(filament_color_type);
                self.ams_multi_color_filment.push(filament_multi_color);
                synced_ams_maps.push(temp);
                continue;
            }
            let mut has_type = false;
            let mut filament_type = opt_string_at(ams, "filament_type", 0);
            let ft = filament_type.clone();
            let mut iter = self.find_base_filament_by_id(&filament_id, |f| {
                has_type |= opt_string_at(&f.config, "filament_type", 0) == ft;
            });
            if iter.is_none() {
                // Why it failed, not just that it did: the three conditions above fail for very different
                // reasons (no such material locally / the material exists but not for the printer in use /
                // only non-base variants) and the message is the same for all of them. (logged only)
                // Every stand-in picked below is approximate by definition -- it is chosen by material type,
                // not by the tray's own id. Which slot a tray finally lands on is only known after the
                // append/overwrite merge, so the slot is flagged at the end of this function rather than here.
                if !filament_type.is_empty() {
                    filament_type = format!("Generic {}", filament_type);
                    iter = self.find_filament_if(|f| f.is_compatible && f.is_system && f.name.starts_with(&filament_type));
                }
                if iter.is_none() {
                    // Prefer old selection
                    if ams_filament_presets.len() < self.filament_presets.len() {
                        ams_filament_presets.push(self.filament_presets[ams_filament_presets.len()].clone());
                        ams_filament_colors.push(filament_color);
                        ams_filament_color_types.push(filament_color_type);
                        self.ams_multi_color_filment.push(filament_multi_color);
                        synced_ams_maps.push(temp);
                        unknowns.push((
                            key,
                            if has_type {
                                L("The filament may not be compatible with the current machine settings. Generic filament presets will be used.")
                            } else {
                                L("The filament model is unknown. Still using the previous filament preset.")
                            },
                        ));
                        continue;
                    }
                    iter = self.find_filament_if(|f| f.is_compatible && f.is_system);
                    if iter.is_none() {
                        continue;
                    }
                }
                let iter_name = &self.filaments.m_presets[iter.unwrap()].name;
                unknowns.push((
                    key,
                    if iter_name.starts_with(&filament_type) {
                        if has_type {
                            L("The filament may not be compatible with the current machine settings. Generic filament presets will be used.")
                        } else {
                            L("The filament model is unknown. Generic filament presets will be used.")
                        }
                    } else if has_type {
                        L("The filament may not be compatible with the current machine settings. A random filament preset will be used.")
                    } else {
                        L("The filament model is unknown. A random filament preset will be used.")
                    },
                ));
                filament_id = self.filaments.m_presets[iter.unwrap()].filament_id.clone();
            }
            let _ = filament_id;
            // A filament that the machine actually uses should be enabled (installed)
            // in Studio even if the user never ticked it in the filament preferences,
            // otherwise it stays hidden in the filament settings dropdown after sync.
            // Deferred to apply_synced_tray_side_effects: a tray that the mapping pass below
            // drops was never synced, so installing its preset here would enable a filament
            // the user asked not to add.
            ams_infos.last_mut().unwrap().install_preset = iter;
            ams_filament_presets.push(self.filaments.m_presets[iter.unwrap()].name.clone());
            ams_filament_colors.push(filament_color);
            ams_filament_color_types.push(filament_color_type);
            self.ams_multi_color_filment.push(filament_multi_color);
            synced_ams_maps.push(temp);
        }
        if ams_filament_presets.is_empty() {
            return Ok(0);
        }

        // Snapshot and temporarily strip mixed filament slots so AMS sync
        // operates on physical filaments only, preserving "physical-first" ordering.
        let mut mixed_snapshots: Vec<MixedSlotSnapshot> = Vec::new();
        let pc = &mut self.project_config;
        let has_is_mixed = bools_ref(pc, "filament_is_mixed").is_some();
        let has_comp = strings_ref(pc, "filament_mixed_components").is_some();
        let has_ratios = strings_ref(pc, "filament_mixed_sublayer_ratios").is_some();
        let has_gradient = bools_ref(pc, "filament_mixed_gradient").is_some();
        let has_grad_range = strings_ref(pc, "filament_mixed_gradient_range").is_some();
        let has_grad_curve = strings_ref(pc, "filament_mixed_gradient_curve").is_some();
        let has_per_part = bools_ref(pc, "filament_mixed_gradient_per_part").is_some();
        if has_is_mixed {
            let is_mixed = bools_ref(pc, "filament_is_mixed").unwrap().clone();
            let filament_color = strings_ref(pc, "filament_colour").expect("filament_colour").clone();
            let filament_color_type = strings_ref(pc, "filament_colour_type").expect("filament_colour_type").clone();
            let get_s = |key: &str, i: usize| strings_ref(pc, key).and_then(|v| v.get(i).cloned());
            let get_b = |key: &str, i: usize| bools_ref(pc, key).and_then(|v| v.get(i).copied());
            let mut i = 0;
            while i < is_mixed.len() && i < self.filament_presets.len() {
                if is_mixed[i] == 0 {
                    i += 1;
                    continue;
                }
                let mut snap = MixedSlotSnapshot {
                    preset: self.filament_presets[i].clone(),
                    color: filament_color.get(i).cloned().unwrap_or_default(),
                    color_type: filament_color_type.get(i).cloned().unwrap_or_default(),
                    ..Default::default()
                };
                if let Some(v) = get_s("filament_mixed_components", i) {
                    snap.mixed_components = v;
                }
                if let Some(v) = get_s("filament_mixed_sublayer_ratios", i) {
                    snap.mixed_sublayer_ratios = v;
                }
                if let Some(v) = get_b("filament_mixed_gradient", i) {
                    snap.mixed_gradient = v != 0;
                }
                if let Some(v) = get_s("filament_mixed_gradient_range", i) {
                    snap.mixed_gradient_range = v;
                }
                if let Some(v) = get_s("filament_mixed_gradient_curve", i) {
                    snap.mixed_gradient_curve = v;
                }
                if let Some(v) = get_b("filament_mixed_gradient_per_part", i) {
                    snap.mixed_gradient_per_part = v != 0;
                }
                mixed_snapshots.push(snap);
                i += 1;
            }
            if !mixed_snapshots.is_empty() {
                let phys_count = self.filament_presets.len() - mixed_snapshots.len();
                self.filament_presets.truncate(phys_count);
                strings_mut(pc, "filament_colour", false)?.unwrap().resize(phys_count, String::new());
                strings_mut(pc, "filament_colour_type", false)?.unwrap().resize(phys_count, String::new());
                ints_mut(pc, "filament_map", false)?.expect("filament_map").resize(phys_count, 1);
                bools_mut(pc, "filament_is_mixed", false)?.unwrap().resize(phys_count, 0);
                if has_comp {
                    strings_mut(pc, "filament_mixed_components", false)?.unwrap().resize(phys_count, String::new());
                }
                if has_ratios {
                    strings_mut(pc, "filament_mixed_sublayer_ratios", false)?.unwrap().resize(phys_count, String::new());
                }
                if has_gradient {
                    bools_mut(pc, "filament_mixed_gradient", false)?.unwrap().resize(phys_count, 0);
                }
                if has_grad_range {
                    strings_mut(pc, "filament_mixed_gradient_range", false)?.unwrap().resize(phys_count, String::new());
                }
                if has_grad_curve {
                    strings_mut(pc, "filament_mixed_gradient_curve", false)?.unwrap().resize(phys_count, String::new());
                }
                if has_per_part {
                    bools_mut(pc, "filament_mixed_gradient_per_part", false)?.unwrap().resize(phys_count, 0);
                }
            }
        }

        // The project's materials as they stand going into this sync, indexed by physical slot (the mixed
        // slots have just been stripped, so these indices line up with the ones `maps` ends up using).
        // Consulted at the end to tell a slot the user already configured apart from one this sync appends.
        let presets_before_sync: Vec<String> = self.filament_presets.clone();

        // AMS trays that end up excluded from the project filament list, so their deferred side
        // effects must not be applied. Only the mapping branch can exclude a tray; the overwrite
        // branch takes every synced tray, so the set stays empty there.
        let mut unsynced_trays: BTreeSet<i32> = BTreeSet::new();

        if use_map {
            let check_has_merge_info = |maps: &BTreeMap<i32, AMSMapInfo>, merge_info: &mut MergeFilamentInfo, _exist_colors_size: usize| {
                let mut done: BTreeSet<i32> = BTreeSet::new();
                let entries: Vec<(&i32, &AMSMapInfo)> = maps.iter().collect();
                for i in 0..entries.len() {
                    let mut same_ams: Vec<i32> = vec![*entries[i].0];
                    for j in i + 1..entries.len() {
                        if done.contains(entries[j].0) {
                            continue;
                        }
                        if entries[i].1.slot_id.is_empty() || entries[i].1.ams_id.is_empty() {
                            continue;
                        }
                        if entries[i].1.slot_id == entries[j].1.slot_id && entries[i].1.ams_id == entries[j].1.ams_id {
                            same_ams.push(*entries[j].0);
                            done.insert(*entries[j].0);
                        }
                    }
                    if same_ams.len() > 1 {
                        merge_info.merges.push(same_ams);
                    }
                }
            };
            let filament_color_len = strings_ref(&self.project_config, "filament_colour").expect("filament_colour").len();
            check_has_merge_info(maps, merge_info, filament_color_len);
            let get_map_index = |ams_infos: &mut Vec<AmsInfo>, infos: &[AMSMapInfo], temp: &AMSMapInfo| -> i32 {
                for i in 0..infos.len() {
                    let diameter_matches = temp.nozzle_diameter.is_empty()
                        || infos[i].nozzle_diameter.is_empty()
                        || (string_to_double_decimal_point(&infos[i].nozzle_diameter).0 - string_to_double_decimal_point(&temp.nozzle_diameter).0).abs()
                            < 1e-6;
                    if infos[i].slot_id == temp.slot_id && infos[i].ams_id == temp.ams_id && diameter_matches {
                        ams_infos[i].is_map = true;
                        return i as i32;
                    }
                }
                -1
            };
            let mut need_append_colors: Vec<AmsInfo> = Vec::new();
            let mut exist_colors: Vec<String> = strings_ref(&self.project_config, "filament_colour").expect("filament_colour").clone();
            let mut exist_color_types: Vec<String> = strings_ref(&self.project_config, "filament_colour_type").expect("filament_colour_type").clone();
            let mut exist_filament_presets: Vec<String> = self.filament_presets.clone();
            let mut exist_multi_color_filment: Vec<Vec<String>> = vec![Vec::new(); exist_colors.len()];
            for i in 0..exist_colors.len() {
                exist_multi_color_filment[i] = vec![exist_colors[i].clone()];
            }
            for i in 0..exist_colors.len() {
                let ii = i as i32;
                if maps.contains_key(&ii) {
                    //mapping exist
                    let m = maps[&ii].clone();
                    let valid_index = get_map_index(&mut ams_infos, &ams_array_maps, &m);
                    if valid_index >= 0 && (valid_index as usize) < ams_filament_presets.len() {
                        let vi = valid_index as usize;
                        let e = maps.get_mut(&ii).unwrap();
                        e.nozzle_diameter = ams_array_maps[vi].nozzle_diameter.clone();
                        e.nozzle_volume_type = ams_array_maps[vi].nozzle_volume_type;
                        exist_colors[i] = ams_filament_colors[vi].clone();
                        exist_color_types[i] = ams_filament_color_types[vi].clone();
                        exist_filament_presets[i] = ams_filament_presets[vi].clone();
                        exist_multi_color_filment[i] = self.ams_multi_color_filment[vi].clone();
                    } else {
                        // check error: array bound (mapping exist)
                    }
                }
            }
            for i in 0..ams_infos.len() {
                // check append
                if ams_infos[i].valid {
                    if i >= ams_filament_presets.len() {
                        // check error: array bound (check append)
                        continue;
                    }
                    ams_infos[i].filament_color_type = ams_filament_color_types[i].clone();
                    ams_infos[i].filament_preset = ams_filament_presets[i].clone();
                    ams_infos[i].mutli_filament_color = self.ams_multi_color_filment[i].clone();
                    if !ams_infos[i].is_map {
                        need_append_colors.push(ams_infos[i].clone());
                        // Unmapped: only reaches the list if it is appended below.
                        if let Some(s) = ams_infos[i].source {
                            unsynced_trays.insert(s);
                        }
                        ams_filament_colors[i] = String::new();
                        ams_filament_color_types[i] = String::new();
                        ams_filament_presets[i] = String::new();
                        self.ams_multi_color_filment[i] = Vec::new();
                    }
                } else {
                    ams_filament_colors[i] = String::new();
                    ams_filament_color_types[i] = String::new();
                    ams_filament_presets[i] = String::new();
                    self.ams_multi_color_filment[i] = Vec::new();
                }
            }
            //delete redundant color
            ams_filament_colors.retain(|v| !v.is_empty());
            ams_filament_color_types.retain(|v| !v.is_empty());
            ams_filament_presets.retain(|v| !v.is_empty());
            self.ams_multi_color_filment.retain(|v| !v.is_empty());
            if !need_append_colors.is_empty() && enable_append {
                let get_idx_in_array = |presets: &[String], colors: &[String], preset: &str, color: &str| -> i32 {
                    for i in 0..presets.len() {
                        if presets[i] == preset && colors[i] == color {
                            return i as i32;
                        }
                    }
                    -1
                };
                for item in need_append_colors.iter() {
                    // Mixed slots are stripped here and re-appended after the sync, so exist_filament_presets holds
                    // physical filaments only; the total still has to leave room for the mixed ones.
                    if exist_filament_presets.len() >= crate::texture_painting::MAX_PHYSICAL_FILAMENT_COUNT
                        || exist_filament_presets.len() + mixed_snapshots.len() >= crate::texture_painting::MAX_FILAMENT_COUNT
                    {
                        break;
                    }
                    let idx = get_idx_in_array(&exist_filament_presets, &exist_colors, &item.filament_preset, &item.filament_color);
                    // Represented in the final list either way: appended just below, or already
                    // present as an identical preset and colour.
                    if let Some(s) = item.source {
                        unsynced_trays.remove(&s);
                    }
                    if idx >= 0 {
                        maps.entry(idx).or_insert_with(|| item.mapping.clone());
                        continue;
                    }
                    maps.insert(exist_filament_presets.len() as i32, item.mapping.clone());
                    exist_filament_presets.push(item.filament_preset.clone());
                    exist_colors.push(item.filament_color.clone());
                    exist_color_types.push(item.filament_color_type.clone());
                    exist_multi_color_filment.push(item.mutli_filament_color.clone());
                }
            }
            *strings_mut(&mut self.project_config, "filament_colour", false)?.unwrap() = exist_colors;
            *strings_mut(&mut self.project_config, "filament_colour_type", false)?.unwrap() = exist_color_types;
            self.ams_multi_color_filment = exist_multi_color_filment;
            let n = exist_filament_presets.len();
            self.filament_presets = exist_filament_presets;
            ints_mut(&mut self.project_config, "filament_map", false)?.expect("filament_map").resize(n, 1);
        } else {
            // overwrite;
            let n = ams_filament_colors.len();
            *strings_mut(&mut self.project_config, "filament_colour", false)?.unwrap() = ams_filament_colors;
            *strings_mut(&mut self.project_config, "filament_colour_type", false)?.unwrap() = ams_filament_color_types.clone();
            self.filament_presets = ams_filament_presets;
            ints_mut(&mut self.project_config, "filament_map", false)?.expect("filament_map").resize(n, 1);
            maps.clear();
            for (i, m) in synced_ams_maps.iter().enumerate() {
                maps.insert(i as i32, m.clone());
            }

            let print_config = &mut self.prints.get_edited_preset_mut().config;
            let n_types = ams_filament_color_types.len() as i32;
            let support_filament_opt = print_config.option_typed_mut::<ConfigOptionInt>("support_filament", false)?.expect("support_filament");
            if support_filament_opt.value > n_types {
                support_filament_opt.value = 0;
            }

            let support_interface_filament_opt =
                print_config.option_typed_mut::<ConfigOptionInt>("support_interface_filament", false)?.expect("support_interface_filament");
            if support_interface_filament_opt.value > n_types {
                support_interface_filament_opt.value = 0;
            }
        }

        apply_synced_tray_side_effects(&mut self.filaments, &ams_infos, &unsynced_trays, unknowns);

        // Flag every already-configured slot whose tray had no matching preset, and give its material back.
        // The pass above could only seed such a slot with a "Generic <type>" stand-in, an arbitrary
        // compatible preset, or -- when not even those existed -- the slot's own preset left untouched.
        // All three mean the same thing to the user: the machine is feeding that slot from a tray the
        // filament list does not represent. Done only here because the slot a tray lands on is decided
        // by the append/overwrite merge, and `maps` is that finished slot -> tray binding.
        for unknown in unknowns.iter() {
            let tray = &self.filament_ams_list[&unknown.0];
            let ams_id = opt_string_at(tray, "ams_id", 0);
            let slot_id = opt_string_at(tray, "slot_id", 0);
            for (mapping_first, mapping_second) in maps.iter() {
                if *mapping_first < 0 || mapping_second.ams_id != ams_id || mapping_second.slot_id != slot_id {
                    continue;
                }
                let slot = *mapping_first as usize;
                // A slot this sync appended did not exist before it, so there was no earlier choice of the
                // user's for the stand-in to be standing in for -- leave it alone and stay quiet about it.
                if slot >= presets_before_sync.len() || slot >= self.filament_presets.len() {
                    continue;
                }
                // The stand-in stays. It is the same fallback the AMS dropdown offers for this tray, so
                // sync and that list now agree on what this tray maps to; the user is told about the swap
                // below instead of finding the slot silently reverted to whatever was there before.
                if self.ams_sync_substitutions.iter().any(|s| s.slot == slot) {
                    continue;
                }
                let mut record = AmsSyncSubstitution { slot, tray: opt_string_at(tray, "tray_name", 0), ..Default::default() };
                // What the tray actually holds. Its presets are all incompatible with this printer -- that
                // is why we are here -- so the name is read off any of them rather than from a usable one.
                let tray_filament_id = opt_string_at(tray, "filament_id", 0);
                for it in self.filaments.iter() {
                    if !tray_filament_id.is_empty() && it.filament_id == tray_filament_id {
                        record.wanted = if it.alias.is_empty() { it.name.clone() } else { it.alias.clone() };
                        break;
                    }
                }
                if record.wanted.is_empty() {
                    record.wanted = opt_string_at(tray, "filament_type", 0);
                }
                // Whatever the slot ends up on, including the case where no stand-in was found and it
                // simply kept what it had: from the user's side the tray's material was not applied and
                // this is what got used instead, which is the same sentence either way.
                record.applied = self.filament_presets[slot].clone();
                // The display name, to match `wanted` -- the stored name carries a printer/nozzle suffix
                // that says nothing the user needs here.
                if let Some(applied) = self.filaments.find_preset(&record.applied, false) {
                    record.applied = if applied.alias.is_empty() { applied.name.clone() } else { applied.alias.clone() };
                }
                self.ams_sync_substitutions.push(record);
            }
        }
        // Filled in tray order, but it is read out as a list of slot numbers for the user.
        let subs = std::mem::take(&mut self.ams_sync_substitutions);
        let mut idx: Vec<usize> = (0..subs.len()).collect();
        crate::clipper::libcxx_sort(&mut idx, |a, b| subs[*a].slot < subs[*b].slot);
        let mut taken: Vec<Option<AmsSyncSubstitution>> = subs.into_iter().map(Some).collect();
        self.ams_sync_substitutions = idx.into_iter().map(|i| taken[i].take().unwrap()).collect();

        // Re-append mixed filament slots that were stripped before AMS sync
        if !mixed_snapshots.is_empty() {
            let new_phys_count = self.filament_presets.len();
            let pc = &mut self.project_config;
            if let Some(v) = bools_mut(pc, "filament_is_mixed", false)? {
                v.resize(new_phys_count, 0);
            }
            if let Some(v) = strings_mut(pc, "filament_mixed_components", false)? {
                v.resize(new_phys_count, String::new());
            }
            if let Some(v) = strings_mut(pc, "filament_mixed_sublayer_ratios", false)? {
                v.resize(new_phys_count, String::new());
            }
            if let Some(v) = bools_mut(pc, "filament_mixed_gradient", false)? {
                v.resize(new_phys_count, 0);
            }
            if let Some(v) = strings_mut(pc, "filament_mixed_gradient_range", false)? {
                v.resize(new_phys_count, String::new());
            }
            if let Some(v) = strings_mut(pc, "filament_mixed_gradient_curve", false)? {
                v.resize(new_phys_count, String::new());
            }
            if let Some(v) = bools_mut(pc, "filament_mixed_gradient_per_part", false)? {
                v.resize(new_phys_count, 0);
            }

            for snap in mixed_snapshots.iter() {
                self.filament_presets.push(snap.preset.clone());
                strings_mut(pc, "filament_colour", false)?.unwrap().push(snap.color.clone());
                strings_mut(pc, "filament_colour_type", false)?.unwrap().push(snap.color_type.clone());
                self.ams_multi_color_filment.push(vec![snap.color.clone()]);
                ints_mut(pc, "filament_map", false)?.expect("filament_map").push(1);
                if let Some(v) = bools_mut(pc, "filament_is_mixed", false)? {
                    v.push(1);
                }
                if let Some(v) = strings_mut(pc, "filament_mixed_components", false)? {
                    v.push(snap.mixed_components.clone());
                }
                if let Some(v) = strings_mut(pc, "filament_mixed_sublayer_ratios", false)? {
                    v.push(snap.mixed_sublayer_ratios.clone());
                }
                if let Some(v) = bools_mut(pc, "filament_mixed_gradient", false)? {
                    v.push(snap.mixed_gradient as u8);
                }
                if let Some(v) = strings_mut(pc, "filament_mixed_gradient_range", false)? {
                    v.push(snap.mixed_gradient_range.clone());
                }
                if let Some(v) = strings_mut(pc, "filament_mixed_gradient_curve", false)? {
                    v.push(snap.mixed_gradient_curve.clone());
                }
                if let Some(v) = bools_mut(pc, "filament_mixed_gradient_per_part", false)? {
                    v.push(snap.mixed_gradient_per_part as u8);
                }
            }
        }

        // Update ams_multi_color_filment
        self.update_filament_multi_color()?;
        self.update_multi_material_filament_presets(usize::MAX)?;
        Ok(self.filament_presets.len() as u32)
    }

    /// PresetBundle.cpp:3258-3277 `void PresetBundle::update_filament_multi_color()`
    fn update_filament_multi_color(&mut self) -> CfgResult<()> {
        let mut exsit_multi_colors: Vec<String> = Vec::new();
        for fil_item in self.ams_multi_color_filment.iter() {
            if fil_item.is_empty() {
                break;
            }
            if fil_item.len() == 1 {
                exsit_multi_colors.push(fil_item[0].clone());
            } else {
                let mut colors = String::new();
                for color in fil_item {
                    colors += &format!("{} ", color);
                }
                colors.pop(); // remove last space
                exsit_multi_colors.push(colors);
            }
        }
        option_resize(&mut self.project_config, "filament_multi_colour", exsit_multi_colors.len())?;
        *strings_mut(&mut self.project_config, "filament_multi_colour", false)?.expect("filament_multi_colour") = exsit_multi_colors;
        Ok(())
    }

    /// PresetBundle.cpp:3279-3301 `std::vector<int> PresetBundle::get_used_tpu_filaments(const std::vector<int> &used_filaments)`
    pub fn get_used_tpu_filaments(&self, used_filaments: &[i32]) -> Vec<i32> {
        let mut tpu_filaments: Vec<i32> = Vec::new();
        for i in 0..self.filament_presets.len() {
            if !used_filaments.contains(&((i + 1) as i32)) {
                continue;
            }

            let filament_name = &self.filament_presets[i];
            for f_index in 0..self.filaments.size() {
                let preset = self.filaments.preset(f_index);
                if *filament_name == preset.name {
                    let mut display_filament_type = String::new();
                    let _filament_type = preset.config.get_filament_type_displayed(&mut display_filament_type, 0);
                    if display_filament_type == "TPU" {
                        tpu_filaments.push(i as i32);
                    }
                }
            }
        }
        tpu_filaments
    }

    /// PresetBundle.cpp:3303-3324 `void PresetBundle::set_calibrate_printer(std::string name)`
    pub fn set_calibrate_printer(&mut self, name: &str) {
        if name.is_empty() {
            self.calibrate_filaments.clear();
            return;
        }
        if !name.is_empty() {
            self.calibrate_printer = find_preset_index(&self.printers, name, false);
        }
        let printer_preset: &Preset = match self.calibrate_printer {
            Some(EDITED_PRESET_IDX) => self.printers.get_edited_preset(),
            Some(i) => &self.printers.m_presets[i],
            None => self.printers.get_edited_preset(),
        };
        let active_printer = self.printers.get_preset_with_vendor_profile(printer_preset);
        let mut config = DynamicPrintConfig::new();
        config.set_key_value("printer_preset", Box::new(ConfigOptionString::with_value(active_printer.preset.name.clone())));
        if let Some(opt) = active_printer.preset.config.option("nozzle_diameter") {
            let n = opt.as_vector_base().map_or(0, |v| v.size());
            config.set_key_value("num_extruders", Box::new(ConfigOptionInt::with_value(n as i32)));
        }
        let mut calibrate_filaments: BTreeSet<usize> = BTreeSet::new();
        for i in self.filaments.num_default_presets()..self.filaments.size() {
            let preset = &self.filaments.m_presets[i];
            let this_preset_with_vendor_profile = self.filaments.get_preset_with_vendor_profile(preset);
            let is_compatible = is_compatible_with_printer(&this_preset_with_vendor_profile, &active_printer, Some(&config));
            if is_compatible {
                calibrate_filaments.insert(i);
            }
        }
        self.calibrate_filaments = calibrate_filaments;
    }

    /// PresetBundle.cpp:3326-3341 `std::vector<std::vector<DynamicPrintConfig>> PresetBundle::get_extruder_filament_info() const`
    pub fn get_extruder_filament_info(&self) -> Vec<Vec<DynamicPrintConfig>> {
        let mut filament_infos: Vec<Vec<DynamicPrintConfig>> = Vec::new();
        let extruder_nums = self.get_printer_extruder_count();
        if extruder_nums > 1 {
            filament_infos.resize(extruder_nums as usize, Vec::new());
            for (first, second) in self.filament_ams_list.iter() {
                if first & 0x10000 != 0 {
                    // right
                    filament_infos[1].push(second.clone());
                } else {
                    // left
                    filament_infos[0].push(second.clone());
                }
            }
        }
        filament_infos
    }

    /// PresetBundle.cpp:3343-3377 `std::set<std::string> PresetBundle::get_printer_names_by_printer_type_and_nozzle(const std::string &printer_type, std::string nozzle_diameter_str, bool system_only)`
    pub fn get_printer_names_by_printer_type_and_nozzle(&self, printer_type: &str, nozzle_diameter_str: &str, system_only: bool) -> BTreeSet<String> {
        let mut printer_names: BTreeSet<String> = BTreeSet::new();

        /* unknown or empty printer type */
        if printer_type.is_empty() {
            return printer_names;
        }

        let mut nozzle_diameter_str = nozzle_diameter_str.to_string();
        if nozzle_diameter_str == "0.0" || nozzle_diameter_str.is_empty() {
            nozzle_diameter_str = "0.4".to_string();
        }

        for printer in self.printers.iter() {
            if system_only && !printer.is_system {
                continue;
            }

            let printer_model_str = match printer.config.option("printer_model").and_then(|o| o.as_any().downcast_ref::<ConfigOptionString>()) {
                Some(s) => s,
                None => continue,
            };

            // use printer_model as printer type
            if printer_model_str.value != printer_type {
                continue;
            }

            if printer.name.contains(&nozzle_diameter_str) {
                printer_names.insert(printer.name.clone());
            }
        }

        printer_names
    }

    /// PresetBundle.cpp:3379-3439 `bool PresetBundle::check_filament_temp_equation_by_printer_type_and_nozzle_for_mas_tray(const std::string &printer_type, std::string& nozzle_diameter_str, std::string &setting_id, std::string &tag_uid, std::string &nozzle_temp_min, std::string &nozzle_temp_max, std::string& preset_setting_id)`
    #[allow(clippy::too_many_arguments)]
    pub fn check_filament_temp_equation_by_printer_type_and_nozzle_for_mas_tray(
        &self,
        printer_type: &str,
        nozzle_diameter_str: &mut String,
        setting_id: &mut String,
        tag_uid: &mut String,
        nozzle_temp_min: &mut String,
        nozzle_temp_max: &mut String,
        preset_setting_id: &mut String,
    ) -> CfgResult<bool> {
        let mut is_equation = true;

        let filament_list = self.filaments.get_filament_presets();
        let printer_names = self.get_printer_names_by_printer_type_and_nozzle(printer_type, nozzle_diameter_str, true);

        let list = match filament_list.get(setting_id.as_str()) {
            Some(l) => l,
            None => return Ok(is_equation),
        };
        for &pi in list.iter() {
            let preset = &self.filaments.m_presets[pi];
            if tag_uid == "0" || (tag_uid.len() == 16 && &tag_uid[12..14] == "01") {
                continue;
            }
            if !preset.is_user() {
                continue;
            }
            let printer_strs = preset.config.option("compatible_printers").and_then(|o| o.as_any().downcast_ref::<ConfigOptionStrings>()).expect("compatible_printers");
            let mut compared = false;
            for printer_str in printer_strs.values.iter() {
                if printer_names.contains(printer_str) {
                    // Compare only once
                    if !compared {
                        compared = true;
                        let mut min_temp_equation = false;
                        let mut max_temp_equation = false;
                        let mut _min_nozzle_temp = std_stoi(nozzle_temp_min)?;
                        let mut _max_nozzle_temp = std_stoi(nozzle_temp_max)?;
                        if let Some(opt_min) = preset.config.option("nozzle_temperature_range_low") {
                            let v = opt_min.vector_values_any().and_then(|v| v.downcast_ref::<Vec<i32>>()).expect("ConfigOptionInts");
                            _min_nozzle_temp = if v.is_empty() { panic!("get_at(0) of an empty vector") } else { v[0] };
                            if _min_nozzle_temp.to_string() == *nozzle_temp_min {
                                min_temp_equation = true;
                            } else {
                                *nozzle_temp_min = _min_nozzle_temp.to_string();
                            }
                        }
                        if let Some(opt_max) = preset.config.option("nozzle_temperature_range_high") {
                            let v = opt_max.vector_values_any().and_then(|v| v.downcast_ref::<Vec<i32>>()).expect("ConfigOptionInts");
                            _max_nozzle_temp = if v.is_empty() { panic!("get_at(0) of an empty vector") } else { v[0] };
                            if _max_nozzle_temp.to_string() == *nozzle_temp_max {
                                max_temp_equation = true;
                            } else {
                                *nozzle_temp_max = _max_nozzle_temp.to_string();
                            }
                        }
                        if min_temp_equation && max_temp_equation {
                            // Determine if the temperature has changed: no changed
                        } else {
                            // Determine if the temperature has changed: has changed
                            *preset_setting_id = preset.setting_id.clone();
                            is_equation = false;
                        }
                    } else {
                        // assert(false): compiled out (-DNDEBUG).
                    }
                }
            }
        }
        Ok(is_equation)
    }

    /// PresetBundle.cpp:3441-3472 `Preset *PresetBundle::get_similar_printer_preset(std::string printer_model, std::string printer_variant)`:
    /// the index of the preset in `printers.m_presets`.
    pub fn get_similar_printer_preset(&self, printer_model: &str, printer_variant: &str) -> Option<usize> {
        let mut printer_model = printer_model.to_string();
        let mut printer_variant = printer_variant.to_string();
        if printer_model.is_empty() {
            printer_model = opt_string(&self.printers.get_selected_preset().config, "printer_model").clone();
        }
        let printer_variant_old = opt_string(&self.printers.get_selected_preset().config, "printer_variant").clone();
        let mut printer_presets: BTreeMap<String, usize> = BTreeMap::new();
        for (i, preset) in self.printers.m_presets.iter().enumerate() {
            if printer_variant.is_empty() && !preset.is_system {
                continue;
            }
            if *opt_string(&preset.config, "printer_model") == printer_model {
                printer_presets.entry(preset.name.clone()).or_insert(i);
            }
        }
        if printer_presets.is_empty() {
            return None;
        }
        let mut prefer_printer = self.printers.get_selected_preset().name.clone();
        if !printer_variant.is_empty() {
            // boost::replace_all: an empty search text replaces nothing.
            if !printer_variant_old.is_empty() {
                prefer_printer = prefer_printer.replace(&printer_variant_old, &printer_variant);
            }
        } else if let Some(n) = prefer_printer.find(&printer_variant_old) {
            prefer_printer = format!("{} {}{}", printer_model, printer_variant_old, &prefer_printer[n + printer_variant_old.len()..]);
        }
        if let Some(&i) = printer_presets.get(&prefer_printer) {
            if printer_variant.is_empty() || *opt_string(&self.printers.m_presets[i].config, "printer_variant") == printer_variant {
                return Some(i);
            }
        }
        if printer_variant.is_empty() {
            printer_variant = printer_variant_old;
        }
        for (_name, &i) in printer_presets.iter() {
            if *opt_string(&self.printers.m_presets[i].config, "printer_variant") == printer_variant {
                return Some(i);
            }
        }
        printer_presets.values().next().copied()
    }

    /// PresetBundle.cpp:3474-3489 `void PresetBundle::set_ams_count(int index, int ams4, int ams1)`
    pub fn set_ams_count(&mut self, index: i32, ams4: i32, ams1: i32) {
        let n = self.get_printer_extruder_count();
        self.extruder_ams_counts.resize(n.max(0) as usize, BTreeMap::new());

        let valid = 0 <= index && (index as usize) < self.extruder_ams_counts.len();
        if !valid {
            return;
        }

        let counts = &mut self.extruder_ams_counts[index as usize];
        counts.insert(4, ams4);
        counts.insert(1, ams1);
        if (index as usize) < self.extruder_ams_selections.len() && self.extruder_ams_selections[index as usize].initialized {
            self.extruder_ams_selections[index as usize].ams4.resize(ams4.max(0) as usize, String::new());
            self.extruder_ams_selections[index as usize].ams1.resize(ams1.max(0) as usize, String::new());
        }
    }

    /// PresetBundle.cpp:3491-3499 `std::pair<int, int> PresetBundle::get_ams_count(int index)`
    pub fn get_ams_count(&mut self, index: i32) -> (i32, i32) {
        let valid = 0 <= index && (index as usize) < self.extruder_ams_counts.len();
        if !valid {
            return (0, 0);
        }

        let counts = &mut self.extruder_ams_counts[index as usize];
        let n4 = *counts.entry(4).or_insert(0);
        let n1 = *counts.entry(1).or_insert(0);
        (n4, n1)
    }

    /// PresetBundle.cpp:3501-3507 `ExtruderAmsSelection *PresetBundle::get_ams_selection(int index)`
    pub fn get_ams_selection(&mut self, index: i32) -> Option<&mut ExtruderAmsSelection> {
        let n = self.get_printer_extruder_count();
        self.extruder_ams_selections.resize(n.max(0) as usize, ExtruderAmsSelection::default());
        let valid = 0 <= index && (index as usize) < self.extruder_ams_selections.len();
        if valid {
            Some(&mut self.extruder_ams_selections[index as usize])
        } else {
            None
        }
    }

    /// PresetBundle.cpp:3509-3526 `void PresetBundle::initialize_ams_selection(int index, const std::string &device_id, const std::vector<std::string> &ams4_ids, const std::vector<std::string> &ams1_ids)`
    pub fn initialize_ams_selection(&mut self, index: i32, device_id: &str, ams4_ids: &[String], ams1_ids: &[String]) {
        match self.get_ams_selection(index) {
            None => return,
            Some(selection) => {
                if selection.initialized && (device_id.is_empty() || selection.device_id == device_id) {
                    return;
                }
            }
        }

        let (n4, n1) = self.get_ams_count(index);
        let initialize = |units: &mut Vec<String>, count: i32, device_ids: &[String]| {
            units.clear();
            units.reserve(count.max(0) as usize);
            for i in 0..count {
                units.push(if (i as usize) < device_ids.len() { device_ids[i as usize].clone() } else { String::new() });
            }
        };
        let selection = &mut self.extruder_ams_selections[index as usize];
        initialize(&mut selection.ams4, n4, ams4_ids);
        initialize(&mut selection.ams1, n1, ams1_ids);
        selection.initialized = true;
        selection.device_id = device_id.to_string();
    }

    /// PresetBundle.cpp:3528-3560 `void PresetBundle::resize_ams_selection(int index, const std::string &device_id, const std::vector<std::string> &ams4_ids, const std::vector<std::string> &ams1_ids, int ams4, int ams1)`
    #[allow(clippy::too_many_arguments)]
    pub fn resize_ams_selection(&mut self, index: i32, device_id: &str, ams4_ids: &[String], ams1_ids: &[String], ams4: i32, ams1: i32) {
        self.initialize_ams_selection(index, device_id, ams4_ids, ams1_ids);
        let selection = match self.get_ams_selection(index) {
            None => return,
            Some(s) => s,
        };

        let resize = |selected: &mut Vec<String>, count: i32, device_ids: &[String]| {
            let wanted = count.max(0) as usize;
            if selected.len() > wanted {
                selected.truncate(wanted);
            }
            while selected.len() < wanted {
                let missing = device_ids.iter().position(|id| !selected.contains(id));
                let missing = match missing {
                    None => {
                        selected.push(String::new());
                        continue;
                    }
                    Some(m) => m,
                };

                let missing_rank = missing;
                let insert_at = selected
                    .iter()
                    .position(|id| {
                        if id.is_empty() {
                            return true;
                        }
                        match device_ids.iter().position(|d| d == id) {
                            Some(rank) => rank > missing_rank,
                            None => false,
                        }
                    })
                    .unwrap_or(selected.len());
                selected.insert(insert_at, device_ids[missing].clone());
            }
        };
        resize(&mut selection.ams4, ams4, ams4_ids);
        resize(&mut selection.ams1, ams1, ams1_ids);
        self.set_ams_count(index, ams4, ams1);
    }

    /// PresetBundle.cpp:3562-3573 `void PresetBundle::replace_ams_selection(int index, const std::string &device_id, const std::vector<std::string> &ams4_ids, const std::vector<std::string> &ams1_ids)`
    pub fn replace_ams_selection(&mut self, index: i32, device_id: &str, ams4_ids: &[String], ams1_ids: &[String]) {
        let selection = match self.get_ams_selection(index) {
            None => return,
            Some(s) => s,
        };

        selection.initialized = true;
        selection.device_id = device_id.to_string();
        selection.ams4 = ams4_ids.to_vec();
        selection.ams1 = ams1_ids.to_vec();
        self.set_ams_count(index, ams4_ids.len() as i32, ams1_ids.len() as i32);
    }

    /// PresetBundle.cpp:3575-3626 `std::vector<std::string> PresetBundle::get_extra_compatible_printers_by_full_stat()`
    pub fn get_extra_compatible_printers_by_full_stat(&self) -> Vec<String> {
        let mut result: Vec<String> = Vec::new();
        // Broadening compatibility to sibling nozzle-diameter variants only makes sense while the project is
        // allowed to print with several diameters at once.
        if !multi_nozzle_diameter_mode_enabled(&self.project_config) {
            return result;
        }
        // A project load selects its nozzles only after its printer is in place, so in between the nozzles
        // may still describe the previous printer; skip broadening rather than leak its diameters.
        let extruder_count = self.nozzle_inventory.extruder_count();
        if extruder_count == 0 || extruder_count != self.get_printer_extruder_count() {
            return result;
        }

        // Collect the distinct diameters the user selected via "set nozzle diameter".
        let mut selected_diameters: Vec<f64> = Vec::new();
        for e in 0..extruder_count {
            for slot in self.nozzle_inventory.slots(e).iter() {
                if slot.empty() {
                    continue;
                }
                let d = string_to_double_decimal_point(&slot.diameter).0;
                if d > 0.0 && !selected_diameters.contains(&d) {
                    selected_diameters.push(d);
                }
            }
        }
        // Multi-diameter mode deliberately pins the active printer/process preset to 0.2. Even when the
        // physical machine currently has only one configured diameter (for example only 0.4), that sibling
        // printer preset still has to broaden filament compatibility; otherwise every 0.4 filament appears
        // under "Unavailable" merely because the active slicing baseline is 0.2.
        if selected_diameters.is_empty() {
            return result;
        }
        selected_diameters.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let printer_model = opt_string(&self.printers.get_edited_preset().config, "printer_model").clone();
        if printer_model.is_empty() {
            return result;
        }

        let mut names: BTreeSet<String> = BTreeSet::new();
        for printer in self.printers.iter() {
            if *opt_string(&printer.config, "printer_model") != printer_model {
                continue;
            }
            let variant_diameter = string_to_double_decimal_point(opt_string(&printer.config, "printer_variant")).0;
            if variant_diameter <= 0.0 {
                continue;
            }
            for &d in selected_diameters.iter() {
                if (variant_diameter - d).abs() < 1e-3 {
                    names.insert(printer.name.clone());
                    break;
                }
            }
        }
        result.extend(names);
        result
    }

    /// PresetBundle.cpp:3628-3677 `std::vector<std::string> PresetBundle::compute_filament_supported_nozzle_diameters(size_t num_filaments) const`
    pub fn compute_filament_supported_nozzle_diameters(&self, num_filaments: usize) -> Vec<String> {
        let mut result: Vec<String> = vec![String::new(); num_filaments];

        // Map every printer preset name to its (normalized) variant diameter. Filament presets reference
        // printers by name in compatible_printers, so this lets us translate a compatible printer into the
        // diameter it represents. Only variants of the active model belong to this project's choice set.
        let mut printer_name_to_diameter: BTreeMap<String, String> = BTreeMap::new();
        let printer_model = opt_string(&self.printers.get_edited_preset().config, "printer_model").clone();
        for printer in self.printers.iter() {
            if *opt_string(&printer.config, "printer_model") != printer_model {
                continue;
            }
            let variant_diameter = string_to_double_decimal_point(opt_string(&printer.config, "printer_variant")).0;
            if variant_diameter > 0.0 {
                printer_name_to_diameter.insert(printer.name.clone(), format_diameter_to_str(variant_diameter));
            }
        }
        if printer_name_to_diameter.is_empty() {
            return result;
        }

        for i in 0..num_filaments {
            if i >= self.filament_presets.len() {
                continue;
            }
            let preset = match self.filaments.find_preset(&self.filament_presets[i], false) {
                Some(p) => p,
                None => continue,
            };

            let compatible = match strings_ref(&preset.config, "compatible_printers") {
                Some(c) => c,
                None => continue,
            };

            // Use the filament preset's own compatible_printers only. The multi-diameter broadening that
            // makes sibling-variant filaments selectable must NOT leak here: the restriction has to reflect
            // the diameters the filament natively supports, not the loosened selectable set.
            let mut diameters: BTreeSet<String> = BTreeSet::new();
            for printer_name in compatible.iter() {
                if let Some(found) = printer_name_to_diameter.get(printer_name) {
                    diameters.insert(found.clone());
                }
            }

            let mut joined = String::new();
            for d in diameters.iter() {
                if !joined.is_empty() {
                    joined.push(';');
                }
                joined += d;
            }
            result[i] = joined;
        }
        result
    }
}

/// PresetBundle.cpp:3679-3692 `static std::set<std::string> filament_preset_diameters_for_model(const Preset &preset, const std::map<std::string, std::string> &printer_name_to_diameter)`
fn filament_preset_diameters_for_model(preset: &Preset, printer_name_to_diameter: &BTreeMap<String, String>) -> BTreeSet<String> {
    let mut result: BTreeSet<String> = BTreeSet::new();
    let compatible = match strings_ref(&preset.config, "compatible_printers") {
        Some(c) => c,
        None => return result,
    };
    for printer_name in compatible.iter() {
        if let Some(it) = printer_name_to_diameter.get(printer_name) {
            result.insert(it.clone());
        }
    }
    result
}

/// PresetBundle.cpp:3694-3706 `static std::map<std::string, std::string> build_printer_name_to_diameter(const PresetBundle &bundle)`
fn build_printer_name_to_diameter(bundle: &PresetBundle) -> BTreeMap<String, String> {
    let printer_model = opt_string(&bundle.printers.get_edited_preset().config, "printer_model").clone();
    let mut result: BTreeMap<String, String> = BTreeMap::new();
    for it in bundle.printers.iter() {
        if *opt_string(&it.config, "printer_model") != printer_model {
            continue;
        }
        let diameter = string_to_double_decimal_point(opt_string(&it.config, "printer_variant")).0;
        if diameter > 0.0 {
            result.insert(it.name.clone(), format_diameter_to_str(diameter));
        }
    }
    result
}

/// PresetBundle.cpp:3708-3724 `static bool uses_per_slot_nozzle_diameters(const ConfigBase &config)`
fn uses_per_slot_nozzle_diameters(config: &DynamicConfig) -> bool {
    if let Some(diameters) = floats_ref(config, "nozzle_diameter") {
        if diameters.len() > 1 {
            return true;
        }
    }

    if let Some(max_counts) = ints_ref(config, "extruder_max_nozzle_count") {
        if max_counts.iter().any(|count| *count > 1) {
            return true;
        }
    }

    config.option_typed::<ConfigOptionBool>("handle_hotend_as_extruder").map_or(false, |o| o.value)
}

/// PresetBundle.cpp:3726-3761 `static std::set<std::string> collect_configured_nozzle_diameters(const PresetBundle &bundle)`
fn collect_configured_nozzle_diameters(bundle: &PresetBundle) -> BTreeSet<String> {
    let mut result: BTreeSet<String> = BTreeSet::new();
    let printer = bundle.printers.get_edited_preset();

    // With the project-level gate off the machine offers exactly one diameter - the one its printer preset
    // is on - no matter how many the rack physically carries. Every caller keys its single-diameter branch
    // on this set holding fewer than two entries, so the whole preset/selector side follows from here.
    if !multi_nozzle_diameter_mode_enabled(&bundle.project_config) || !uses_per_slot_nozzle_diameters(&printer.config) {
        let primary_diameter = string_to_double_decimal_point(opt_string(&printer.config, "printer_variant")).0;
        if primary_diameter > 0.0 {
            result.insert(format_diameter_to_str(primary_diameter));
        }
        return result;
    }

    // Keep every physical slot visible to the selector. Whether a machine can print multiple diameters
    // has nothing to do with how many slots one extruder has: a dual-extruder machine holding one
    // diameter per side (H2D) is just as multi-diameter as a single extruder holding several (H3C).
    // The representative per-extruder diameter is intentionally not used here either, because one
    // logical extruder may carry several physical diameters.
    for extruder_id in 0..bundle.nozzle_inventory.extruder_count() {
        for slot in bundle.nozzle_inventory.slots(extruder_id).iter() {
            let diameter = string_to_double_decimal_point(&slot.diameter).0;
            if diameter > 0.0 {
                result.insert(format_diameter_to_str(diameter));
            }
        }
    }

    // Without a per-slot snapshot the printer preset is all we know about the hardware.
    if result.is_empty() {
        let primary_diameter = string_to_double_decimal_point(opt_string(&printer.config, "printer_variant")).0;
        if primary_diameter > 0.0 {
            result.insert(format_diameter_to_str(primary_diameter));
        }
    }
    result
}

/// PresetBundle.cpp:3763-3833 `static std::set<std::pair<std::string, NozzleVolumeType>> collect_installed_nozzle_variants(const PresetBundle &bundle)`:
/// companion of collect_configured_nozzle_diameters() on the flow axis: which (diameter, flow)
/// nozzles the rack actually carries.
fn collect_installed_nozzle_variants(bundle: &PresetBundle) -> BTreeSet<(String, NozzleVolumeType)> {
    let mut result: BTreeSet<(String, NozzleVolumeType)> = BTreeSet::new();
    let printer = bundle.printers.get_edited_preset();

    // The machine's own diameter paired with the per-extruder project flow setting. Serves both the
    // gate below and the fallback for an empty per-slot snapshot.
    let fill_from_printer_preset = |result: &mut BTreeSet<(String, NozzleVolumeType)>| {
        let primary_diameter = string_to_double_decimal_point(opt_string(&printer.config, "printer_variant")).0;
        if primary_diameter <= 0.0 {
            return;
        }
        let diameter = format_diameter_to_str(primary_diameter);
        match enums_ref(&bundle.project_config, "nozzle_volume_type") {
            Some(volume_types) if !volume_types.is_empty() => {
                for &value in volume_types.iter() {
                    result.insert((diameter.clone(), NozzleVolumeType::from_i32(value)));
                }
            }
            _ => {
                result.insert((diameter, NozzleVolumeType::NvtStandard));
            }
        }
    };

    // Same gate as collect_configured_nozzle_diameters(), but only the diameter axis collapses with
    // the project-level gate off: every slot then prints at the printer preset's single diameter.
    // The flow axis does not collapse with it -- the slots can carry different flows at that one
    // diameter, and the grouping is free to put a filament on whichever slot has the flow it needs.
    // So the diameter comes from the printer preset while the flows are still read off the rack.
    if !multi_nozzle_diameter_mode_enabled(&bundle.project_config) || !uses_per_slot_nozzle_diameters(&printer.config) {
        let primary_diameter = string_to_double_decimal_point(opt_string(&printer.config, "printer_variant")).0;
        if primary_diameter > 0.0 {
            let diameter = format_diameter_to_str(primary_diameter);
            for extruder_id in 0..bundle.nozzle_inventory.extruder_count() {
                for slot in bundle.nozzle_inventory.slots(extruder_id).iter() {
                    if slot.empty() {
                        continue;
                    }
                    let slot_diameter = string_to_double_decimal_point(&slot.diameter).0;
                    if slot_diameter <= 0.0 || format_diameter_to_str(slot_diameter) != diameter {
                        continue;
                    }
                    result.insert((diameter.clone(), nvt_of_inventory(slot.volume_type)));
                }
            }
        }
        // Nothing mounted to read -- the printer preset and the project's flow setting are all there is.
        if result.is_empty() {
            fill_from_printer_preset(&mut result);
        }
        return result;
    }

    for extruder_id in 0..bundle.nozzle_inventory.extruder_count() {
        for slot in bundle.nozzle_inventory.slots(extruder_id).iter() {
            // An empty slot carries no nozzle; skipping those is what keeps the selector from
            // offering a variant the machine does not physically have.
            if slot.empty() {
                continue;
            }
            let diameter = string_to_double_decimal_point(&slot.diameter).0;
            if diameter <= 0.0 {
                continue;
            }
            result.insert((format_diameter_to_str(diameter), nvt_of_inventory(slot.volume_type)));
        }
    }

    // A rack with every slot empty gives no variant. collect_configured_nozzle_diameters() falls back
    // to the printer preset in that case, so this does the same: the two describe the same rack.
    if result.is_empty() {
        fill_from_printer_preset(&mut result);
    }
    result
}

/// PresetBundle.cpp:3835-3863 `static std::vector<NozzleVolumeType> get_filament_nozzle_flow_options(const PresetBundle &bundle, size_t filament_index, const std::string &diameter, const std::set<std::pair<std::string, NozzleVolumeType>> &installed)`
fn get_filament_nozzle_flow_options(
    bundle: &PresetBundle,
    filament_index: usize,
    diameter: &str,
    installed: &BTreeSet<(String, NozzleVolumeType)>,
) -> Vec<NozzleVolumeType> {
    let mut result: Vec<NozzleVolumeType> = Vec::new();
    if filament_index >= bundle.filament_presets.len() || diameter.is_empty() {
        return result;
    }

    // Same printability rule PartPlate uses to validate a slice: a 2.85mm TPU only declares the
    // TPU285 variant, so it must not inherit a Standard/High Flow selection from another slot.
    let mut filament_variants: Vec<String> = Vec::new();
    let mut variants_complete = false;
    if let Some(preset) = bundle.filaments.find_preset(&bundle.filament_presets[filament_index], false) {
        if let Some(opt) = strings_ref(&preset.config, "filament_extruder_variant") {
            filament_variants = opt.clone();
        }
        variants_complete = bundle.filaments.is_bbl_brand_filament(preset);
    }

    for variant in installed.iter() {
        if variant.0 != diameter || variant.1 == NozzleVolumeType::NvtHybrid {
            continue;
        }
        if !crate::print_config::config_def::is_nozzle_printable_for_filament(variant.1, &filament_variants, variants_complete) {
            continue;
        }
        result.push(variant.1);
    }
    result
}

/// PresetBundle.cpp:3865-3905 `static std::vector<std::string> get_filament_nozzle_diameter_options(const PresetBundle &bundle, size_t filament_index, const std::map<std::string, std::string> &printer_name_to_diameter, const std::set<std::string> &configured)`
fn get_filament_nozzle_diameter_options(
    bundle: &PresetBundle,
    filament_index: usize,
    printer_name_to_diameter: &BTreeMap<String, String>,
    configured: &BTreeSet<String>,
) -> Vec<String> {
    if filament_index >= bundle.filament_presets.len() || bundle.is_mixed_filament(filament_index) {
        return Vec::new();
    }

    let current = match bundle.filaments.find_preset(&bundle.filament_presets[filament_index], false) {
        Some(c) => c,
        None => return Vec::new(),
    };

    let mut supported = filament_preset_diameters_for_model(current, printer_name_to_diameter);
    if !current.filament_id.is_empty() {
        for it in bundle.filaments.iter() {
            // Broaden by every sibling preset sharing the same filament_id, not just system ones:
            // user-created filaments (e.g. "Polymaker ABS 777") are saved as one preset per printer-
            // nozzle variant with matching filament_id, all is_system == false. Requiring is_system
            // dropped every one of those, leaving only the current variant's own compatible_printers,
            // so the FilamentNozzlePopup only offered the currently active diameter for custom
            // filaments even when the user had explicitly configured all four nozzle variants.
            if it.filament_id == current.filament_id {
                let diameters = filament_preset_diameters_for_model(it, printer_name_to_diameter);
                supported.extend(diameters);
            }
        }
    }

    let result: Vec<String> = if supported.is_empty() {
        configured.iter().cloned().collect()
    } else {
        // std::set_intersection of two sorted ranges
        supported.intersection(configured).cloned().collect()
    };
    // std::sort with the comparator (libc++; unstable, so through indices)
    let mut idx: Vec<usize> = (0..result.len()).collect();
    crate::clipper::libcxx_sort(&mut idx, |a, b| string_to_double_decimal_point(&result[*a]).0 < string_to_double_decimal_point(&result[*b]).0);
    idx.into_iter().map(|i| result[i].clone()).collect()
}

/// PresetBundle.cpp:3907-3935 `static void settle_filament_slot_flow(PresetBundle &bundle, size_t idx)`:
/// settles a slot's filament_volume_map entry onto a flow the slot's current filament preset can
/// actually use at its current (or, failing that, first available) diameter (PresetBundle.cpp:53-56).
fn settle_filament_slot_flow(bundle: &mut PresetBundle, idx: usize) -> CfgResult<()> {
    match ints_mut(&mut bundle.project_config, "filament_volume_map", false)? {
        None => return Ok(()),
        Some(volume_map) => {
            if idx >= volume_map.len() {
                volume_map.resize(idx + 1, NozzleVolumeType::NvtStandard as i32);
            }
        }
    }

    let printer_name_to_diameter = build_printer_name_to_diameter(bundle);
    let configured = collect_configured_nozzle_diameters(bundle);
    let options = get_filament_nozzle_diameter_options(bundle, idx, &printer_name_to_diameter, &configured);
    if options.is_empty() {
        return Ok(());
    }

    let mut target_diameter = match strings_ref(&bundle.project_config, "filament_diameter_map") {
        Some(sd) if idx < sd.len() => sd[idx].clone(),
        _ => String::new(),
    };
    if target_diameter.is_empty() || !options.contains(&target_diameter) {
        target_diameter = options[0].clone();
    }

    let installed = collect_installed_nozzle_variants(bundle);
    let flows = get_filament_nozzle_flow_options(bundle, idx, &target_diameter, &installed);
    if flows.is_empty() {
        return Ok(());
    }
    let volume_map = ints_mut(&mut bundle.project_config, "filament_volume_map", false)?.expect("filament_volume_map");
    let current_flow = NozzleVolumeType::from_i32(volume_map[idx]);
    if !flows.contains(&current_flow) {
        volume_map[idx] = flows[0] as i32;
    }
    Ok(())
}

/// PresetBundle.cpp:3937-3987 `static void repair_filament_nozzle_flows(const PresetBundle &bundle, ConfigOptionInts *volume_map, const std::set<std::pair<std::string, NozzleVolumeType>> &installed, const std::map<std::string, std::string> &printer_name_to_diameter, const std::set<std::string> &configured)`:
/// repairs filament_volume_map so every slot's flow is one the rack carries at that slot's nozzle
/// diameter and that the slot's filament can print. `volume_map` is whether the C++ pointer (to
/// `filament_volume_map` of the project config) is set.
fn repair_filament_nozzle_flows(
    bundle: &mut PresetBundle,
    volume_map: bool,
    installed: &BTreeSet<(String, NozzleVolumeType)>,
    printer_name_to_diameter: &BTreeMap<String, String>,
    configured: &BTreeSet<String>,
) -> CfgResult<()> {
    if !volume_map {
        return Ok(());
    }
    let n = bundle.filament_presets.len();
    ints_mut(&mut bundle.project_config, "filament_volume_map", false)?.expect("filament_volume_map").resize(n, NozzleVolumeType::NvtStandard as i32);

    for i in 0..bundle.filament_presets.len() {
        if bundle.is_mixed_filament(i) {
            continue;
        }
        let mut diameter = match strings_ref(&bundle.project_config, "filament_diameter_map") {
            Some(selected) if i < selected.len() => selected[i].clone(),
            _ => String::new(),
        };

        // Ahead of the flagged-slot skip below, which preserves selections the user made. This one
        // was not: it is left over from the slot's previous diameter, and keeping it would pin the
        // slot to a flagged state with no way out -- the nozzle settings will not offer it either.
        {
            let vm = ints_mut(&mut bundle.project_config, "filament_volume_map", false)?.expect("filament_volume_map");
            if !diameter.is_empty() && !crate::print_config::config_def::nozzle_variant_exists(&diameter, NozzleVolumeType::from_i32(vm[i])) {
                vm[i] = NozzleVolumeType::NvtStandard as i32;
            }
        }

        if bundle.incompatible_nozzle_diameter_slots.contains(&i) {
            continue;
        }
        if diameter.is_empty() {
            // A machine carrying a single diameter leaves filament_diameter_map empty on purpose, yet
            // the flow is still selectable there (e.g. a 0.4 Standard next to a 0.4 High Flow). Key off
            // the one diameter this slot can use, the same fallback the sidebar's nozzle label renders.
            let options = get_filament_nozzle_diameter_options(bundle, i, printer_name_to_diameter, configured);
            if options.is_empty() {
                continue;
            }
            diameter = options[0].clone();
        }
        let flows = get_filament_nozzle_flow_options(bundle, i, &diameter, installed);
        if flows.is_empty() {
            continue; // nothing installed can print this slot: leave it for the pre-slice checks to report
        }
        let vm = ints_mut(&mut bundle.project_config, "filament_volume_map", false)?.expect("filament_volume_map");
        if !flows.contains(&NozzleVolumeType::from_i32(vm[i])) {
            vm[i] = flows[0] as i32;
        }
    }
    Ok(())
}

impl PresetBundle {
    /// PresetBundle.cpp:3989-3994 `std::vector<std::string> PresetBundle::get_filament_nozzle_diameter_options(size_t filament_index) const`
    pub fn get_filament_nozzle_diameter_options(&self, filament_index: usize) -> Vec<String> {
        let printer_name_to_diameter = build_printer_name_to_diameter(self);
        let configured = collect_configured_nozzle_diameters(self);
        get_filament_nozzle_diameter_options(self, filament_index, &printer_name_to_diameter, &configured)
    }

    /// PresetBundle.cpp:3996-3999 `std::set<std::pair<std::string, NozzleVolumeType>> PresetBundle::get_installed_nozzle_variants() const`
    pub fn get_installed_nozzle_variants(&self) -> BTreeSet<(String, NozzleVolumeType)> {
        collect_installed_nozzle_variants(self)
    }

    /// PresetBundle.cpp:4001-4008 `bool PresetBundle::is_filament_nozzle_diameter_valid(size_t filament_index, const std::string &diameter) const`
    pub fn is_filament_nozzle_diameter_valid(&self, filament_index: usize, diameter: &str) -> bool {
        if diameter.is_empty() {
            return true;
        }
        let normalized = format_diameter_to_str(string_to_double_decimal_point(diameter).0);
        let options = self.get_filament_nozzle_diameter_options(filament_index);
        options.contains(&normalized)
    }
}

/// PresetBundle.cpp:4010-4077 `static bool set_filament_nozzle_diameter(PresetBundle &bundle, size_t filament_index, const std::string &diameter, std::string *selected_preset_name, const std::map<std::string, std::string> &printer_name_to_diameter, const std::vector<std::string> &options)`
fn set_filament_nozzle_diameter(
    bundle: &mut PresetBundle,
    filament_index: usize,
    diameter: &str,
    selected_preset_name: Option<&mut String>,
    printer_name_to_diameter: &BTreeMap<String, String>,
    options: &[String],
) -> CfgResult<bool> {
    if filament_index >= bundle.filament_presets.len() {
        return Ok(false);
    }

    let n = bundle.filament_presets.len();
    strings_mut(&mut bundle.project_config, "filament_diameter_map", false)?.expect("filament_diameter_map").resize(n, String::new());
    if diameter.is_empty() {
        strings_mut(&mut bundle.project_config, "filament_diameter_map", false)?.expect("filament_diameter_map")[filament_index].clear();
        if let Some(s) = selected_preset_name {
            *s = bundle.filament_presets[filament_index].clone();
        }
        return Ok(true);
    }

    let normalized = format_diameter_to_str(string_to_double_decimal_point(diameter).0);
    if !options.contains(&normalized) {
        return Ok(false);
    }

    let (current_name, current_filament_id, current_is_system, current_diameters) =
        match bundle.filaments.find_preset(&bundle.filament_presets[filament_index], false) {
            None => return Ok(false),
            Some(c) => (c.name.clone(), c.filament_id.clone(), c.is_system, filament_preset_diameters_for_model(c, printer_name_to_diameter)),
        };

    let mut target_preset = current_name;
    if !current_diameters.is_empty() && !current_diameters.contains(&normalized) {
        target_preset.clear();
        if !current_filament_id.is_empty() {
            // Mirrors the sibling lookup in get_filament_nozzle_diameter_options(): user-created
            // filaments store one preset per nozzle variant sharing filament_id, all is_system ==
            // false, so requiring is_system here found no sibling and failed the switch on a diameter
            // that popup had already offered. Preferring a sibling of the same kind as the current
            // preset keeps a preset saved off a system one on the user's own variants rather than
            // jumping back to the system preset and dropping its edits.
            let mut fallback = String::new();
            for it in bundle.filaments.iter() {
                if it.filament_id != current_filament_id {
                    continue;
                }
                let diameters = filament_preset_diameters_for_model(it, printer_name_to_diameter);
                if !diameters.contains(&normalized) {
                    continue;
                }
                if it.is_system == current_is_system {
                    target_preset = it.name.clone();
                    break;
                }
                if fallback.is_empty() {
                    fallback = it.name.clone();
                }
            }
            if target_preset.is_empty() {
                target_preset = fallback;
            }
        }
        if target_preset.is_empty() {
            return Ok(false);
        }
        bundle.filament_presets[filament_index] = target_preset.clone();
        if filament_index == 0 {
            bundle.filaments.select_preset_by_name(&target_preset, false, true);
        }
    }

    strings_mut(&mut bundle.project_config, "filament_diameter_map", false)?.expect("filament_diameter_map")[filament_index] = normalized;
    if let Some(s) = selected_preset_name {
        *s = target_preset;
    }
    Ok(true)
}

impl PresetBundle {
    /// PresetBundle.cpp:4079-4097 `bool PresetBundle::set_filament_nozzle_diameter(size_t filament_index, const std::string &diameter, std::string *selected_preset_name, bool diameter_picked_by_user)`
    pub fn set_filament_nozzle_diameter(
        &mut self,
        filament_index: usize,
        diameter: &str,
        selected_preset_name: Option<&mut String>,
        diameter_picked_by_user: bool,
    ) -> CfgResult<bool> {
        let printer_name_to_diameter = build_printer_name_to_diameter(self);
        let mut configured = collect_configured_nozzle_diameters(self);
        // Admitting the requested diameter to the configured set only widens what the options below may
        // intersect with; the filament still has to have a preset for it, so an outright unsupported
        // diameter is refused exactly as before.
        if diameter_picked_by_user && !diameter.is_empty() {
            configured.insert(format_diameter_to_str(string_to_double_decimal_point(diameter).0));
        }
        let options = get_filament_nozzle_diameter_options(self, filament_index, &printer_name_to_diameter, &configured);
        let updated = set_filament_nozzle_diameter(self, filament_index, diameter, selected_preset_name, &printer_name_to_diameter, &options)?;
        if updated {
            crate::print_config::config_def::sync_mixed_filament_nozzle_diameters(&mut self.project_config);
        }
        Ok(updated)
    }

    /// PresetBundle.cpp:4099-4125 `std::string PresetBundle::tray_diameter_for_filament_slot(size_t filament_index) const`
    pub fn tray_diameter_for_filament_slot(&self, filament_index: usize) -> String {
        if filament_index >= self.filament_presets.len() {
            return String::new();
        }
        let preset = match self.filaments.find_preset(&self.filament_presets[filament_index], false) {
            Some(p) if !p.filament_id.is_empty() => p,
            _ => return String::new(),
        };
        let color = match strings_ref(&self.project_config, "filament_colour") {
            Some(colors) if filament_index < colors.len() => colors[filament_index].clone(),
            _ => String::new(),
        };

        let mut first_match = String::new();
        for (_k, tray) in self.filament_ams_list.iter() {
            if opt_string_at(tray, "filament_id", 0) != preset.filament_id {
                continue;
            }
            if !color.is_empty() && opt_string_at(tray, "filament_colour", 0) != color {
                continue;
            }
            let diameters = match floats_ref(tray, "tray_nozzle_diameter") {
                Some(d) if !d.is_empty() && d[0] > 0. => d,
                _ => continue,
            };
            let diameter = format_diameter_to_str(diameters[0]);
            if first_match.is_empty() {
                first_match = diameter;
            }
        }
        first_match
    }

    /// PresetBundle.cpp:4127-4135 `void PresetBundle::stamp_printer_nozzle_diameter_mode()`
    pub fn stamp_printer_nozzle_diameter_mode(&mut self) -> CfgResult<()> {
        let mut tag = String::new();
        if let Some(printer_base) = self.printers.get_preset_base(self.printers.get_edited_preset()) {
            if printer_base.is_system && is_single_ext_multi_nozzle(&printer_base.config) {
                tag = if multi_nozzle_diameter_mode_enabled(&self.project_config) {
                    crate::print_config::config_def::printer_nozzle_diameter_mode_multi().to_string()
                } else {
                    crate::print_config::config_def::printer_nozzle_diameter_mode_single().to_string()
                };
            }
        }
        self.printers
            .get_edited_preset_mut()
            .config
            .option_typed_mut::<ConfigOptionString>("printer_nozzle_diameter_mode", true)?
            .expect("printer_nozzle_diameter_mode")
            .value = tag;
        Ok(())
    }

    /// PresetBundle.cpp:4137-4151 `bool PresetBundle::is_multi_nozzle_user_printer_preset(const Preset &preset) const`
    pub fn is_multi_nozzle_user_printer_preset(&self, preset: &Preset) -> bool {
        if preset.is_system || preset.is_default {
            return false;
        }
        match self.printers.get_preset_base(preset) {
            Some(base)
                if base.is_system
                    && is_single_ext_multi_nozzle(&base.config)
                    && *opt_string(&base.config, "printer_variant") == crate::print_config::config_def::canonical_multi_nozzle_printer_variant() => {}
            _ => return false,
        }
        let tag = crate::print_config::config_def::printer_nozzle_diameter_mode_tag(&preset.config);
        if tag == crate::print_config::config_def::printer_nozzle_diameter_mode_multi() {
            return true;
        }
        if tag == crate::print_config::config_def::printer_nozzle_diameter_mode_single() {
            return false;
        }
        preset.name.contains("Multiple nozzle")
    }

    /// PresetBundle.cpp:4153-4202 `void PresetBundle::seed_filament_nozzles_for_multi_diameter(const std::string &fallback_diameter)`
    pub fn seed_filament_nozzles_for_multi_diameter(&mut self, fallback_diameter: &str) -> CfgResult<()> {
        let normalized = format_diameter_to_str(string_to_double_decimal_point(fallback_diameter).0);
        if normalized.is_empty() {
            return Ok(());
        }

        if strings_ref(&self.project_config, "filament_diameter_map").is_none() || ints_ref(&self.project_config, "filament_volume_map").is_none() {
            return Ok(());
        }

        let n = self.filament_presets.len();
        strings_mut(&mut self.project_config, "filament_diameter_map", false)?.unwrap().resize(n, String::new());
        ints_mut(&mut self.project_config, "filament_volume_map", false)?.unwrap().resize(n, NozzleVolumeType::NvtStandard as i32);

        let printer_name_to_diameter = build_printer_name_to_diameter(self);
        let configured = collect_configured_nozzle_diameters(self);
        let installed = collect_installed_nozzle_variants(self);

        for i in 0..self.filament_presets.len() {
            if self.is_mixed_filament(i) {
                continue;
            }

            let options = get_filament_nozzle_diameter_options(self, i, &printer_name_to_diameter, &configured);

            // Single-diameter mode never recorded which nozzle each tray feeds -- validate_filament_
            // nozzle_diameter_selections() returns from its configured.size() < 2 branch long before the
            // device-map code that would have read it. The machine did report it though, and
            // filament_ams_list still holds it, so take it here rather than flattening every slot onto
            // the one diameter that happened to be selected. Falling back to that diameter only when
            // the machine has nothing to say about this slot keeps the old behaviour for filaments the
            // user picked by hand.
            let mut target = self.tray_diameter_for_filament_slot(i);
            if target.is_empty() || !options.contains(&target) {
                target = normalized.clone();
            }

            if !set_filament_nozzle_diameter(self, i, &target, None, &printer_name_to_diameter, &options)? {
                // Keep the requested diameter so validation flags the slot instead of rewriting
                // it to the smallest remaining option.
                strings_mut(&mut self.project_config, "filament_diameter_map", false)?.unwrap()[i] = target;
                continue;
            }

            let flows = get_filament_nozzle_flow_options(self, i, &target, &installed);
            if flows.is_empty() {
                continue;
            }
            ints_mut(&mut self.project_config, "filament_volume_map", false)?.unwrap()[i] = flows[0] as i32;
        }
        Ok(())
    }
}

/// PresetBundle.cpp:4204-4241 `static const Preset *find_default_filament_for_nozzle_diameter(PresetBundle &bundle, const std::string &diameter, const std::map<std::string, std::string> &printer_name_to_diameter)`
fn find_default_filament_for_nozzle_diameter<'a>(
    bundle: &'a PresetBundle,
    diameter: &str,
    printer_name_to_diameter: &BTreeMap<String, String>,
) -> Option<&'a Preset> {
    let printer_model = opt_string(&bundle.printers.get_edited_preset().config, "printer_model").clone();
    for printer in bundle.printers.iter() {
        if *opt_string(&printer.config, "printer_model") != printer_model {
            continue;
        }
        let variant = string_to_double_decimal_point(opt_string(&printer.config, "printer_variant")).0;
        if variant <= 0. || format_diameter_to_str(variant) != diameter {
            continue;
        }
        let defaults = match strings_ref(&printer.config, "default_filament_profile") {
            Some(d) => d,
            None => continue,
        };
        for name in defaults.iter() {
            if let Some(candidate) = bundle.filaments.find_preset(name, false) {
                if opt_string_at(&candidate.config, "filament_type", 0) == "PLA"
                    && filament_preset_diameters_for_model(candidate, printer_name_to_diameter).contains(diameter)
                {
                    return Some(candidate);
                }
            }
        }
    }

    // Keep the established incompatible-filament fallback even if the target printer preset does not
    // declare a usable default profile.
    let mut first_pla: Option<&Preset> = None;
    for candidate in bundle.filaments.iter() {
        if !candidate.is_system
            || opt_string_at(&candidate.config, "filament_type", 0) != "PLA"
            || !filament_preset_diameters_for_model(candidate, printer_name_to_diameter).contains(diameter)
        {
            continue;
        }
        if candidate.name == BBL_DEFAULT_FILAMENT || candidate.name.starts_with(&format!("{} @", BBL_DEFAULT_FILAMENT)) {
            return Some(candidate);
        }
        if first_pla.is_none() {
            first_pla = Some(candidate);
        }
    }
    first_pla
}

impl PresetBundle {
    /// The tray diameter of a mapping (the `tray_diameter` lambda of PresetBundle.cpp:4465-4479).
    fn tray_diameter(&self, mapping: &AMSMapInfo) -> String {
        let mapped_diameter = string_to_double_decimal_point(&mapping.nozzle_diameter).0;
        if mapped_diameter > 0. {
            return format_diameter_to_str(mapped_diameter);
        }
        for (_k, tray) in self.filament_ams_list.iter() {
            if opt_string_at(tray, "ams_id", 0) != mapping.ams_id || opt_string_at(tray, "slot_id", 0) != mapping.slot_id {
                continue;
            }
            if let Some(diameters) = floats_ref(tray, "tray_nozzle_diameter") {
                if !diameters.is_empty() && diameters[0] > 0. {
                    return format_diameter_to_str(diameters[0]);
                }
            }
        }
        String::new()
    }

    /// The `tray_volume_type` lambda of PresetBundle.cpp:4481-4496: companion of tray_diameter() on
    /// the flow axis.
    fn tray_volume_type(&self, mapping: &AMSMapInfo) -> Option<NozzleVolumeType> {
        if mapping.nozzle_volume_type.is_some() {
            return mapping.nozzle_volume_type;
        }
        for (_k, tray) in self.filament_ams_list.iter() {
            if opt_string_at(tray, "ams_id", 0) != mapping.ams_id || opt_string_at(tray, "slot_id", 0) != mapping.slot_id {
                continue;
            }
            if let Some(volume_types) = ints_ref(tray, "tray_nozzle_volume_type") {
                if !volume_types.is_empty() {
                    return Some(NozzleVolumeType::from_i32(volume_types[0]));
                }
            }
        }
        None
    }

    fn selected_diameter_at(&self, i: usize) -> String {
        strings_ref(&self.project_config, "filament_diameter_map").expect("filament_diameter_map")[i].clone()
    }
    fn set_selected_diameter_at(&mut self, i: usize, v: String) -> CfgResult<()> {
        strings_mut(&mut self.project_config, "filament_diameter_map", false)?.expect("filament_diameter_map")[i] = v;
        Ok(())
    }
    fn volume_map_at(&self, i: usize) -> NozzleVolumeType {
        let vm = ints_ref(&self.project_config, "filament_volume_map").expect("filament_volume_map");
        if i < vm.len() {
            NozzleVolumeType::from_i32(vm[i])
        } else {
            NozzleVolumeType::NvtStandard
        }
    }

    /// PresetBundle.cpp:4243-4658 `void PresetBundle::validate_filament_nozzle_diameter_selections(const std::map<int, AMSMapInfo> *device_maps, bool device_maps_are_user_chosen)`
    pub fn validate_filament_nozzle_diameter_selections(
        &mut self,
        device_maps: Option<&BTreeMap<i32, AMSMapInfo>>,
        device_maps_are_user_chosen: bool,
    ) -> CfgResult<()> {
        let n = self.filament_presets.len();
        strings_mut(&mut self.project_config, "filament_diameter_map", false)?.expect("filament_diameter_map").resize(n, String::new());
        let supports_multi_diameter = is_single_ext_multi_nozzle(&self.printers.get_edited_preset().config);
        if !supports_multi_diameter {
            if let Some(mode) = self.project_config.option_typed_mut::<ConfigOptionBool>("multi_nozzle_diameter_mode", false)? {
                mode.value = false;
            }
            for v in strings_mut(&mut self.project_config, "filament_diameter_map", false)?.unwrap().iter_mut() {
                *v = String::new();
            }
        }
        // multi_nozzle_diameter_mode is a project-level toggle that outlives a printer switch (see the
        // C++ comment at PresetBundle.cpp:4255-4275); gated on is_single_ext_multi_nozzle().
        let multi_diameter_mode = multi_nozzle_diameter_mode_enabled(&self.project_config) && supports_multi_diameter;
        let retained_selections: Vec<String> =
            if multi_diameter_mode { Vec::new() } else { strings_ref(&self.project_config, "filament_diameter_map").unwrap().clone() };
        let printer_name_to_diameter = build_printer_name_to_diameter(self);
        let configured = collect_configured_nozzle_diameters(self);
        let installed = collect_installed_nozzle_variants(self);

        // Cleared unconditionally, not just inside the configured.size() < 2 branch below (PresetBundle.cpp:4284-4297).
        let previously_incompatible: Vec<usize> = self.incompatible_nozzle_diameter_slots.clone();
        self.incompatible_nozzle_diameter_slots.clear();
        let was_marked_incompatible = |slot: usize| previously_incompatible.contains(&slot);

        // Per-filament flow is only user-owned where the filament panel's nozzle selector is shown
        // (single-extruder multi-nozzle + multi-diameter mode). On every other machine
        // filament_volume_map is still a grouping result, so repairing it here would clobber the
        // slicer's own assignment. Null pointer means "leave the flow alone".
        let mut volume_map = false;
        if multi_diameter_mode && is_single_ext_multi_nozzle(&self.printers.get_edited_preset().config) {
            if let Some(vm) = ints_mut(&mut self.project_config, "filament_volume_map", false)? {
                volume_map = true;
                vm.resize(n, NozzleVolumeType::NvtStandard as i32);
            }
        }

        // Normalize any sibling preset selected by the previous multi-diameter state. Turning the mode off
        // only makes the selections inactive; retain them so turning it back on restores the project state.
        if configured.len() < 2 {
            if configured.len() == 1 {
                let target = configured.iter().next().unwrap().clone();
                for i in 0..self.filament_presets.len() {
                    if self.is_mixed_filament(i) {
                        continue;
                    }

                    let options = get_filament_nozzle_diameter_options(self, i, &printer_name_to_diameter, &configured);

                    if multi_diameter_mode {
                        // Same question single-diameter mode asks below (PresetBundle.cpp:4329-4334).
                        if options.is_empty() {
                            self.incompatible_nozzle_diameter_slots.push(i);
                            continue;
                        }

                        // Only ever assign a diameter here when the slot never had one (a brand-new
                        // slot). An already-selected slot is left exactly as it was.
                        let current_value = self.selected_diameter_at(i);
                        if current_value.is_empty() {
                            if options.contains(&target) {
                                set_filament_nozzle_diameter(self, i, &target, None, &printer_name_to_diameter, &options)?;
                            }
                            continue;
                        }
                        if format_diameter_to_str(string_to_double_decimal_point(&current_value).0) == target {
                            // The diameter matching is not enough -- the slot names a flow too
                            // (PresetBundle.cpp:4356-4364).
                            if volume_map {
                                let flows = get_filament_nozzle_flow_options(self, i, &target, &installed);
                                let current_flow = self.volume_map_at(i);
                                if flows.is_empty() || !flows.contains(&current_flow) {
                                    self.incompatible_nozzle_diameter_slots.push(i);
                                }
                            }
                            continue; // the nozzle for this slot's diameter is still installed
                        }

                        // The nozzle for this slot's previously-selected diameter was removed. Flag
                        // the slot instead of silently moving it onto the sole remaining diameter.
                        self.incompatible_nozzle_diameter_slots.push(i);
                        continue;
                    }

                    // One flow is not interchangeable with the others: a filament declaring nvtTPU285 can
                    // print on nothing else, and no other filament can use such a nozzle.
                    let mut filament_variants: Vec<String> = Vec::new();
                    if let Some(preset) = self.filaments.find_preset(&self.filament_presets[i], false) {
                        if let Some(opt) = strings_ref(&preset.config, "filament_extruder_variant") {
                            filament_variants = opt.clone();
                        }
                    }
                    let needs_dedicated_flow = !filament_variants.is_empty()
                        && filament_variants
                            .iter()
                            .all(|variant| crate::print_config::config_def::convert_to_nvt_type(variant) == NozzleVolumeType::NvtTPU285);
                    if needs_dedicated_flow && !installed.contains(&(target.clone(), NozzleVolumeType::NvtTPU285)) {
                        self.incompatible_nozzle_diameter_slots.push(i);
                        continue;
                    }

                    // The mirror question: every nozzle the rack carries at this diameter is one this
                    // filament cannot print on (e.g. a lone TPU285 nozzle feeding an ordinary PLA).
                    let any_flow_at_target = installed.iter().any(|variant| variant.0 == target);
                    if any_flow_at_target && get_filament_nozzle_flow_options(self, i, &target, &installed).is_empty() {
                        self.incompatible_nozzle_diameter_slots.push(i);
                        continue;
                    }

                    // Legacy single-diameter-machine behavior: the whole rack switched to a different
                    // target diameter, so follow it when this filament preset supports it, otherwise
                    // replace the preset with a default for the new diameter.
                    if options.contains(&target) && set_filament_nozzle_diameter(self, i, &target, None, &printer_name_to_diameter, &options)? {
                        continue;
                    }

                    if self.block_filament_fallback_substitution || was_marked_incompatible(i) {
                        // Preserve the user's original filament selection instead of silently
                        // substituting a different material; the UI marks this slot instead.
                        self.incompatible_nozzle_diameter_slots.push(i);
                        continue;
                    }

                    let fallback_name = find_default_filament_for_nozzle_diameter(self, &target, &printer_name_to_diameter).map(|p| p.name.clone());
                    if let Some(fallback_name) = fallback_name {
                        self.filament_presets[i] = fallback_name.clone();
                        if i == 0 {
                            self.filaments.select_preset_by_name(&fallback_name, false, true);
                        }
                        let fallback_options = get_filament_nozzle_diameter_options(self, i, &printer_name_to_diameter, &configured);
                        set_filament_nozzle_diameter(self, i, &target, None, &printer_name_to_diameter, &fallback_options)?;
                    }
                }
            }
            if !multi_diameter_mode {
                // Turning multi-diameter mode off only makes the per-slot selections inactive; restore
                // the retained values so turning it back on brings the project state back.
                *strings_mut(&mut self.project_config, "filament_diameter_map", false)?.unwrap() = retained_selections;
            }
            // Keep filament_diameter_map populated even when only one diameter is installed
            // (PresetBundle.cpp:4455-4460); the flow repair has to run on this path too.
            return repair_filament_nozzle_flows(self, volume_map, &installed, &printer_name_to_diameter, &configured);
        }

        for i in 0..self.filament_presets.len() {
            if self.is_mixed_filament(i) {
                strings_mut(&mut self.project_config, "filament_diameter_map", false)?.unwrap()[i].clear();
                continue;
            }
            let options = get_filament_nozzle_diameter_options(self, i, &printer_name_to_diameter, &configured);
            if options.is_empty() {
                // This filament preset has no compatible diameter among any currently installed nozzle
                // at all -- flag it the same way instead of silently blanking the slot's selection.
                if !self.selected_diameter_at(i).is_empty() {
                    self.incompatible_nozzle_diameter_slots.push(i);
                }
                continue;
            }

            // A failed nozzle-mapping keeps the requested diameter on a material that cannot serve it
            // (PresetBundle.cpp:4514-4523).
            if was_marked_incompatible(i) && !self.selected_diameter_at(i).is_empty() {
                let stored = format_diameter_to_str(string_to_double_decimal_point(&self.selected_diameter_at(i)).0);
                if !options.contains(&stored) {
                    self.incompatible_nozzle_diameter_slots.push(i);
                    continue;
                }
            }

            let mut device_diameter = String::new();
            let mut device_volume: Option<NozzleVolumeType> = None;
            if let Some(device_maps) = device_maps {
                if let Some(mapped) = device_maps.get(&(i as i32)) {
                    if !mapped.ams_id.is_empty() && !mapped.slot_id.is_empty() {
                        device_diameter = self.tray_diameter(mapped);
                        device_volume = self.tray_volume_type(mapped);
                    }
                }
            }

            if !device_diameter.is_empty() {
                let normalized_device = format_diameter_to_str(string_to_double_decimal_point(&device_diameter).0);
                if options.contains(&normalized_device)
                    && set_filament_nozzle_diameter(self, i, &normalized_device, None, &printer_name_to_diameter, &options)?
                {
                    // The diameter stuck, so the flow the machine reports for the same nozzle applies too
                    // (PresetBundle.cpp:4547-4550).
                    if volume_map {
                        if let Some(dv) = device_volume {
                            let flows = get_filament_nozzle_flow_options(self, i, &normalized_device, &installed);
                            if flows.contains(&dv) {
                                ints_mut(&mut self.project_config, "filament_volume_map", false)?.unwrap()[i] = dv as i32;
                            } else if device_maps_are_user_chosen {
                                // The user paired this tray with this slot in the matching dialog, so the
                                // pairing is carried out even though the filament has no preset for that
                                // nozzle, and the slot is flagged.
                                ints_mut(&mut self.project_config, "filament_volume_map", false)?.unwrap()[i] = dv as i32;
                                self.incompatible_nozzle_diameter_slots.push(i);
                            } else {
                                // Nobody asked for this pairing -- it came out of the positional merge -- so
                                // the slot keeps the flow it had and stays printable.
                            }
                        }
                    }
                    continue;
                }

                // Reaching here means the machine says this tray feeds a nozzle the slot's filament has
                // no preset for (PresetBundle.cpp:4580-4586). Keep it, record the diameter, flag the slot.
                self.set_selected_diameter_at(i, normalized_device)?;
                self.incompatible_nozzle_diameter_slots.push(i);
                continue;
            }

            // Only reached when the device named no diameter for this slot, so there is nothing authoritative
            // to apply and the slot's own selection stands (PresetBundle.cpp:4595-4600).
            let mut target = self.selected_diameter_at(i);
            let normalized = format_diameter_to_str(string_to_double_decimal_point(&target).0);
            if !options.contains(&normalized) {
                // A non-empty target that fell out of `options` means a nozzle of that diameter was
                // physically removed (Nozzle Settings dialog). Flag it instead of silently moving the
                // slot onto another diameter -- availability is refreshed; the selection is not rewritten.
                if !target.is_empty() && multi_diameter_mode {
                    self.incompatible_nozzle_diameter_slots.push(i);
                    continue;
                }
                target = options[0].clone();
            }
            // `options` only says a filament preset exists for that diameter. The rack still has to carry a
            // nozzle of that diameter whose flow this slot's filament can print (PresetBundle.cpp:4613-4615).
            if volume_map {
                let flows = get_filament_nozzle_flow_options(self, i, &target, &installed);
                let current_flow = self.volume_map_at(i);
                if flows.is_empty() {
                    // Nothing printable at this diameter at all, so moving the slot to a diameter that does
                    // work is still an improvement rather than a rewrite of a usable choice.
                    let mut moved_to_printable_diameter = false;
                    for candidate in options.iter() {
                        if !get_filament_nozzle_flow_options(self, i, candidate, &installed).is_empty() {
                            target = candidate.clone();
                            moved_to_printable_diameter = true;
                            break;
                        }
                    }
                    // No installed nozzle can print this filament at any of its diameters.
                    if !moved_to_printable_diameter {
                        self.incompatible_nozzle_diameter_slots.push(i);
                        continue;
                    }
                } else if !flows.contains(&current_flow) {
                    // The diameter is fine, only the flow the slot names is not on the rack. Flagged
                    // rather than quietly moved to the flow that is -- see the single-diameter branch.
                    self.incompatible_nozzle_diameter_slots.push(i);
                    continue;
                }
            }
            if !set_filament_nozzle_diameter(self, i, &target, None, &printer_name_to_diameter, &options)? {
                strings_mut(&mut self.project_config, "filament_diameter_map", false)?.unwrap()[i].clear();
            }
        }

        // Runs after the loop, not inside it: set_filament_nozzle_diameter() can switch a slot to a
        // sibling preset serving another diameter, which changes filament_extruder_variant, so the flow
        // has to be resolved against the diameter that actually stuck.
        crate::print_config::config_def::sync_mixed_filament_nozzle_diameters(&mut self.project_config);
        repair_filament_nozzle_flows(self, volume_map, &installed, &printer_name_to_diameter, &configured)
    }
}

/// PresetBundle.cpp:4758-4761 `const std::set<std::string> ignore_settings_list`
fn ignore_settings_list() -> BTreeSet<String> {
    ["inherits", "print_settings_id", "filament_settings_id", "printer_settings_id"].iter().map(|s| s.to_string()).collect()
}

/// The `add_if_some_non_empty` lambda of PresetBundle.cpp:5043-5052 (and :289-297, :5123-5132).
fn add_if_some_non_empty(out: &mut DynamicPrintConfig, values: Vec<String>, key: &str) {
    let nonempty = values.iter().any(|v| !v.is_empty());
    if nonempty {
        out.set_key_value(key, Box::new(ConfigOptionStrings::from_values(values)));
    }
}

/// `option<ConfigOptionEnumGeneric>("printer_technology", true)->value = pt` (see set_printer_technology_fff).
fn set_printer_technology(out: &mut DynamicPrintConfig, pt: PrinterTechnology) -> CfgResult<()> {
    let opt = out.option_mut("printer_technology", true)?.expect("printer_technology");
    if opt.r#type() == ConfigOptionType::CoEnum {
        opt.set_int(pt as i32)?;
    }
    Ok(())
}

impl PresetBundle {
    /// PresetBundle.cpp:4660-4686 `bool PresetBundle::is_the_only_edited_filament(unsigned int filament_index)`:
    /// BBS: check whether this is the only edited filament.
    pub fn is_the_only_edited_filament(&self, filament_index: u32) -> bool {
        let n = self.filament_presets.len() as u32;
        if filament_index >= n {
            return false;
        }

        let name = &self.filament_presets[filament_index as usize];
        let edited_preset = self.filaments.get_edited_preset();
        if edited_preset.name != *name {
            return false;
        }

        let mut index = 0u32;
        while index < n {
            if index == filament_index {
                index += 1;
                continue;
            }
            let filament_preset = &self.filament_presets[index as usize];
            if edited_preset.name == *filament_preset {
                return false;
            } else {
                index += 1;
            }
        }
        true
    }

    /// PresetBundle.cpp:4688-4695 `int PresetBundle::get_printer_extruder_count() const`
    pub fn get_printer_extruder_count(&self) -> i32 {
        let printer_preset = self.printers.get_edited_preset();

        floats_ref(&printer_preset.config, "nozzle_diameter").expect("nozzle_diameter").len() as i32
    }

    /// PresetBundle.cpp:4697-4704 `bool PresetBundle::support_different_extruders()`
    pub fn support_different_extruders(&self) -> bool {
        let printer_preset = self.printers.get_edited_preset();
        let mut extruder_count = 0;
        printer_preset.config.support_different_extruders(&mut extruder_count)
    }

    /// PresetBundle.cpp:4706-4720 `std::vector<NozzleVolumeType> PresetBundle::get_printer_nozzle_volume_list()`
    pub fn get_printer_nozzle_volume_list(&self) -> Vec<NozzleVolumeType> {
        let printer_preset = self.printers.get_edited_preset();
        let mut unique_types: BTreeSet<NozzleVolumeType> = BTreeSet::new();
        if let Some(variants) = strings_ref(&printer_preset.config, "printer_extruder_variant") {
            for variant in variants.iter() {
                unique_types.insert(crate::print_config::config_def::convert_to_nvt_type(variant));
            }
        }

        unique_types.into_iter().collect()
    }

    /// PresetBundle.cpp:4722-4736 `std::vector<int> PresetBundle::get_default_nozzle_volume_types_for_filaments(std::vector<int>& f_maps)`
    pub fn get_default_nozzle_volume_types_for_filaments(&self, f_maps: &[i32]) -> Vec<i32> {
        let filament_count = f_maps.len();
        let mut result: Vec<i32> = vec![NozzleVolumeType::NvtStandard as i32; filament_count];

        // dynamic_cast<const ConfigOptionEnumsGeneric*>
        let opt_nozzle_volume_type =
            self.project_config.option("nozzle_volume_type").and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionEnumsGeneric>());
        for index in 0..filament_count {
            if let Some(opt) = opt_nozzle_volume_type {
                // `values.size() > (f_maps[index] - 1)`: an int compared with a size_t.
                let k = (f_maps[index] - 1) as i64 as u64 as usize;
                if opt.values.len() > k {
                    result[index] = opt.values[k];
                }
            }
        }

        result
    }

    /// PresetBundle.cpp:4738-4743 `DynamicPrintConfig PresetBundle::full_config(bool apply_extruder, std::optional<std::vector<int>> filament_maps, std::optional<std::vector<int>> filament_volume_maps) const`
    pub fn full_config(&self, apply_extruder: bool, filament_maps: Option<Vec<i32>>, filament_volume_maps: Option<Vec<i32>>) -> CfgResult<DynamicPrintConfig> {
        if self.printers.get_edited_preset().printer_technology() == PrinterTechnology::PtFFF {
            self.full_fff_config(apply_extruder, filament_maps, filament_volume_maps)
        } else {
            self.full_sla_config()
        }
    }

    /// PresetBundle.cpp:4745-4756 `DynamicPrintConfig PresetBundle::full_config_secure(std::optional<std::vector<int>> filament_maps) const`
    pub fn full_config_secure(&self, filament_maps: Option<Vec<i32>>) -> CfgResult<DynamicPrintConfig> {
        let mut config = self.full_fff_config(false, filament_maps, None)?;
        //BBS example: config.erase("print_host");
        config.erase("print_host_webui");
        config.erase("printhost_apikey");
        config.erase("printhost_cafile");
        config.erase("printhost_user");
        config.erase("printhost_password");
        config.erase("printhost_port");
        Ok(config)
    }

    /// PresetBundle.cpp:4763-5088 `DynamicPrintConfig PresetBundle::full_fff_config(bool apply_extruder, std::optional<std::vector<int>> filament_maps_new, std::optional<std::vector<int>> filament_volume_maps_new) const`
    ///
    /// FIDELITY-NOTE: the C++ reads `compatible_printers_condition()` and the like through the const
    /// accessors, which create the option in the preset when it is missing; the presets of the
    /// collections have them (see the notes of `preset.rs`), so they are read without creating here.
    fn full_fff_config(&self, apply_extruder: bool, filament_maps_new: Option<Vec<i32>>, filament_volume_maps_new: Option<Vec<i32>>) -> CfgResult<DynamicPrintConfig> {
        let ignore_settings_list = ignore_settings_list();
        let mut out = DynamicPrintConfig::new();
        out.apply(static_print_config_defaults("FullPrintConfig"), false)?;
        out.apply(&self.prints.get_edited_preset().config, false)?;
        // Add the default filament preset to have the "filament_preset_id" defined.
        out.apply(&self.filaments.default_preset(0).config, false)?;
        out.apply(&self.printers.get_edited_preset().config, false)?;
        out.apply(&self.project_config, false)?;

        // BBS
        let num_filaments = self.filament_presets.len();

        let mut filament_maps: Vec<i32> = ints_ref(&out, "filament_map").expect("filament_map").clone();
        let mut filament_volume_maps: Vec<i32> = vec![NozzleVolumeType::NvtStandard as i32; num_filaments];

        let filament_volume_map_opt = ints_ref(&out, "filament_volume_map").cloned();
        if let Some(m) = filament_maps_new {
            filament_maps = m;
        }
        if let Some(m) = filament_volume_maps_new {
            filament_volume_maps = m;
            *ints_mut(&mut out, "filament_volume_map", true)?.expect("filament_volume_map") = filament_volume_maps.clone();
        } else if let Some(opt) = &filament_volume_map_opt {
            if opt.len() == num_filaments {
                filament_volume_maps = opt.clone();
            }
        }
        //in some middle state, they may be different
        if filament_maps.len() != num_filaments {
            filament_maps.resize(num_filaments, 1);
        }
        if filament_volume_maps.len() != num_filaments {
            filament_volume_maps.resize(num_filaments, NozzleVolumeType::NvtStandard as i32);
        }

        // Collect the "compatible_printers_condition" and "inherits" values over all presets (print, filaments, printers) into a single vector.
        let mut compatible_printers_condition: Vec<String> = Vec::new();
        let mut compatible_prints_condition: Vec<String> = Vec::new();
        let mut inherits: Vec<String> = Vec::new();
        let mut filament_ids: Vec<String> = Vec::new();
        let mut print_compatible_printers: Vec<String> = Vec::new();
        //BBS: add logic for settings check between different system presets
        let mut different_settings: Vec<String> = Vec::new();
        let mut different_print_settings = String::new();
        let mut different_printer_settings = String::new();
        compatible_printers_condition.push(self.prints.get_edited_preset().compatible_printers_condition().to_string());

        if let Some(compatible_printers) = strings_ref(&self.prints.get_edited_preset().config, "compatible_printers") {
            print_compatible_printers = compatible_printers.clone();
        }
        //BBS: add logic for settings check between different system presets
        let print_inherits = self.prints.get_edited_preset().inherits().to_string();
        inherits.push(print_inherits);
        if let Some(print_parent_preset) = self.prints.get_selected_preset_parent() {
            let dirty_options = PresetCollection::dirty_options_without_option_list(
                Some(self.prints.get_edited_preset()),
                Some(print_parent_preset),
                &ignore_settings_list,
                false,
            )?;
            if !dirty_options.is_empty() {
                different_print_settings = escape_strings_cstyle(&dirty_options);
            }
        }
        different_settings.push(different_print_settings);

        //BBS: update printer config related with variants
        let mut nozzle_volume_types: Vec<Vec<NozzleVolumeType>> = Vec::new();
        let mut extruder_count: i32 = 1;
        let mut extruder_volume_type_count: i32 = 1;
        let mut different_extruder = false;
        if apply_extruder {
            different_extruder = out.support_different_extruders(&mut extruder_count);
            extruder_volume_type_count = out.get_extruder_nozzle_volume_count(extruder_count, &mut nozzle_volume_types);

            if extruder_count > 1 || different_extruder {
                out.update_values_to_printer_extruders(None, extruder_count, extruder_volume_type_count, &nozzle_volume_types, &printer_options_with_variant_2, "printer_extruder_id", "printer_extruder_variant", 2, 0, NozzleVolumeType::NvtStandard)?;
                out.update_values_to_printer_extruders(None, extruder_count, extruder_volume_type_count, &nozzle_volume_types, &printer_options_with_variant_1, "printer_extruder_id", "printer_extruder_variant", 1, 0, NozzleVolumeType::NvtStandard)?;
                //update print config related with variants
                out.update_values_to_printer_extruders(None, extruder_count, extruder_volume_type_count, &nozzle_volume_types, &print_options_with_variant, "print_extruder_id", "print_extruder_variant", 1, 0, NozzleVolumeType::NvtStandard)?;
            }
        }

        if num_filaments <= 1 {
            //BBS: update filament config related with variants
            let mut filament_config = self.filaments.get_edited_preset().config.clone();
            if apply_extruder && (extruder_count > 1 || different_extruder) {
                filament_config.update_values_to_printer_extruders(Some(&out), extruder_count, extruder_volume_type_count, &nozzle_volume_types, &filament_options_with_variant, "", "filament_extruder_variant", 1, filament_maps[0] as u32, NozzleVolumeType::from_i32(filament_volume_maps[0]))?;
            }
            out.apply(&filament_config, false)?;
            compatible_printers_condition.push(self.filaments.get_edited_preset().compatible_printers_condition().to_string());
            compatible_prints_condition.push(self.filaments.get_edited_preset().compatible_prints_condition().to_string());
            //BBS: add logic for settings check between different system presets
            //std::string filament_inherits = this->filaments.get_edited_preset().inherits();
            let current_preset_name = &self.filament_presets[0];
            let preset = self.filaments.find_preset(current_preset_name, true).expect("find_preset(first_visible_if_not_found)");
            let filament_inherits = preset.inherits().to_string();
            inherits.push(filament_inherits);
            filament_ids.push(self.filaments.get_edited_preset().filament_id.clone());

            let mut different_filament_settings = String::new();
            if let Some(filament_parent_preset) = self.filaments.get_selected_preset_parent() {
                let dirty_options = PresetCollection::dirty_options_without_option_list(
                    Some(self.filaments.get_edited_preset()),
                    Some(filament_parent_preset),
                    &ignore_settings_list,
                    false,
                )?;
                if !dirty_options.is_empty() {
                    different_filament_settings = escape_strings_cstyle(&dirty_options);
                }
            }

            different_settings.push(different_filament_settings);

            let index_size = strings_ref(&out, "filament_extruder_variant").expect("filament_extruder_variant").len();
            let filament_self_indice = ints_mut(&mut out, "filament_self_index", true)?.expect("filament_self_index");
            filament_self_indice.resize(index_size, 1);
        } else {
            // Retrieve filament presets and build a single config object for them.
            // First collect the filament configurations based on the user selection of this->filament_presets.
            // Here this->filaments.find_preset() and this->filaments.first_visible() return the edited copy of the preset if active.
            let mut filament_presets: Vec<&Preset> = Vec::new();
            for filament_preset_name in self.filament_presets.iter() {
                let preset = self.filaments.find_preset(filament_preset_name, true).expect("find_preset(first_visible_if_not_found)");
                filament_presets.push(preset);
            }
            while filament_presets.len() < num_filaments {
                filament_presets.push(self.filaments.first_visible());
            }
            for index in 0..num_filaments {
                let preset = filament_presets[index];
                let cfg = &preset.config;
                // The compatible_prints/printers_condition() returns a reference to configuration key, which may not yet exist.
                compatible_printers_condition.push(string_ref(cfg, "compatible_printers_condition").cloned().unwrap_or_default());
                compatible_prints_condition.push(string_ref(cfg, "compatible_prints_condition").cloned().unwrap_or_default());

                //BBS: add logic for settings check between different system presets
                let filament_inherits = string_ref(cfg, "inherits").cloned().unwrap_or_default();
                inherits.push(filament_inherits.clone());
                filament_ids.push(preset.filament_id.clone());
                let mut different_filament_settings = String::new();

                let mut filament_parent_preset: Option<&Preset> = None;
                if preset.is_system || preset.is_default {
                    let is_selected = self.filaments.get_selected_preset_name() == preset.name;
                    if is_selected {
                        //use the real preset
                        filament_parent_preset = self.filaments.find_preset_real(&preset.name, false, true);
                    } else {
                        filament_parent_preset = Some(preset);
                    }
                } else if !filament_inherits.is_empty() {
                    filament_parent_preset = self.filaments.find_preset_real(&filament_inherits, false, true);
                }

                if let Some(parent) = filament_parent_preset {
                    let mut dirty_options = cfg.diff(&parent.config)?;
                    if !dirty_options.is_empty() {
                        dirty_options.retain(|k| !ignore_settings_list.contains(k));
                        different_filament_settings = escape_strings_cstyle(&dirty_options);
                    }
                }

                different_settings.push(different_filament_settings);
            }

            let mut filament_temp_configs: Vec<DynamicPrintConfig> = vec![DynamicPrintConfig::new(); num_filaments];
            for i in 0..num_filaments {
                filament_temp_configs[i] = filament_presets[i].config.clone();
                if apply_extruder && (extruder_count > 1 || different_extruder) {
                    filament_temp_configs[i].update_values_to_printer_extruders(Some(&out), extruder_count, extruder_volume_type_count, &nozzle_volume_types, &filament_options_with_variant, "", "filament_extruder_variant", 1, filament_maps[i] as u32, NozzleVolumeType::from_i32(filament_volume_maps[i]))?;
                }
            }

            // loop through options and apply them to the resulting config.
            let mut filament_variant_count: Vec<usize> = vec![1; num_filaments];
            for key in self.filaments.default_preset(0).config.keys() {
                if key == "compatible_prints" || key == "compatible_printers" {
                    continue;
                }
                // Get a destination option.
                let opt_dst = out.option_mut(&key, false)?.expect("opt_dst");
                if opt_dst.is_scalar() {
                    // Get an option, do not create if it does not exist.
                    if let Some(opt_src) = filament_temp_configs[0].option(&key) {
                        opt_dst.set(opt_src)?;
                    }
                } else {
                    // BBS
                    let opt_vec_dst = opt_dst.as_vector_base_mut().expect("ConfigOptionVectorBase");
                    if apply_extruder {
                        // Setting a vector value from all filament_configs.
                        let filament_opts: Vec<&dyn ConfigOption> =
                            (0..num_filaments).map(|i| filament_temp_configs[i].option(&key).expect("filament option")).collect();
                        opt_vec_dst.set_from_options(&filament_opts)?;
                    } else {
                        for i in 0..num_filaments {
                            let filament_option = filament_temp_configs[i].option(&key).expect("filament option");
                            if i == 0 {
                                opt_vec_dst.set(filament_option)?;
                            } else {
                                opt_vec_dst.append(filament_option)?;
                            }

                            if key == "filament_extruder_variant" {
                                filament_variant_count[i] = filament_option.as_vector_base().expect("vector").size();
                            }
                        }
                    }
                }
            }

            let index_size = strings_ref(&out, "filament_extruder_variant").expect("filament_extruder_variant").len();
            let filament_self_indice = ints_mut(&mut out, "filament_self_index", true)?.expect("filament_self_index");
            if apply_extruder {
                filament_self_indice.resize(index_size, 1);
                let mut i = 0;
                while i < num_filaments && i < filament_self_indice.len() {
                    filament_self_indice[i] = (i + 1) as i32;
                    i += 1;
                }
            } else {
                //append filament_self_index
                filament_self_indice.resize(index_size, 1);
                let mut k = 0usize;
                for i in 0..num_filaments {
                    for _j in 0..filament_variant_count[i] {
                        filament_self_indice[k] = (i + 1) as i32;
                        k += 1;
                    }
                }
            }
        }

        // Filament overrides edited as one value per filament (the variant vector was still
        // length 1) must be copied onto every variant slot before extruder lookup.
        crate::print_config::config_def::align_filament_variant_overrides(&mut out)?;

        //BBS: add logic for settings check between different system presets
        let printer_inherits = self.printers.get_edited_preset().inherits().to_string();
        // Don't store the "compatible_printers_condition" for the printer profile, there is none.
        inherits.push(printer_inherits);
        if let Some(printer_parent_preset) = self.printers.get_selected_preset_parent() {
            let dirty_options = PresetCollection::dirty_options_without_option_list(
                Some(self.printers.get_edited_preset()),
                Some(printer_parent_preset),
                &ignore_settings_list,
                false,
            )?;
            if !dirty_options.is_empty() {
                different_printer_settings = escape_strings_cstyle(&dirty_options);
            }
        }
        different_settings.push(different_printer_settings);

        // These value types clash between the print and filament profiles. They should be renamed.
        out.erase("compatible_prints");
        out.erase("compatible_prints_condition");
        out.erase("compatible_printers");
        out.erase("compatible_printers_condition");
        out.erase("inherits");
        //BBS: add logic for settings check between different system presets
        out.erase("different_settings_to_system");
        out.erase("filament_map_2");

        for key in ["support_filament", "support_interface_filament"] {
            let opt = out.option_typed_mut::<ConfigOptionInt>(key, false)?.expect("ConfigOptionInt");
            opt.value = opt.value.clamp(0, num_filaments as i32);
        }

        out.option_typed_mut::<ConfigOptionString>("print_settings_id", true)?.expect("print_settings_id").value = self.prints.get_selected_preset_name();
        *strings_mut(&mut out, "filament_settings_id", true)?.expect("filament_settings_id") = self.filament_presets.clone();
        out.option_typed_mut::<ConfigOptionString>("printer_settings_id", true)?.expect("printer_settings_id").value = self.printers.get_selected_preset_name();
        *strings_mut(&mut out, "filament_ids", true)?.expect("filament_ids") = filament_ids.clone();
        *ints_mut(&mut out, "filament_map", true)?.expect("filament_map") = filament_maps;
        // Serialize the collected "compatible_printers_condition" and "inherits" fields.
        // There will be 1 + num_exturders fields for "inherits" and 2 + num_extruders for "compatible_printers_condition" stored.
        // The vector will not be stored if all fields are empty strings.
        add_if_some_non_empty(&mut out, compatible_printers_condition, "compatible_machine_expression_group");
        add_if_some_non_empty(&mut out, compatible_prints_condition, "compatible_process_expression_group");
        add_if_some_non_empty(&mut out, inherits, "inherits_group");
        //BBS: add logic for settings check between different system presets
        add_if_some_non_empty(&mut out, different_settings, "different_settings_to_system");
        add_if_some_non_empty(&mut out, print_compatible_printers, "print_compatible_printers");
        *strings_mut(&mut out, "extruder_ams_count", true)?.expect("extruder_ams_count") = save_extruder_ams_count_to_string(&self.extruder_ams_counts);
        // Nozzle counts and per-slot nozzle params for the grouping backend.
        self.sync_nozzle_mirror();
        let mut nozzle_keys = nozzle_print_config_of(&out);
        self.nozzle_inventory.export_to(&mut nozzle_keys);
        write_nozzle_keys(&nozzle_keys, &mut out)?;

        // Multi-diameter: let slicing read the user-selected diameters (persisted with the project through
        // the nozzle inventory) rather than the printer preset's fixed nozzle_diameter
        // (PresetBundle.cpp:5063-5071).
        let multi_diameter_mode = multi_nozzle_diameter_mode_enabled(&self.project_config);
        if multi_diameter_mode && uses_per_slot_nozzle_diameters(&out) && self.nozzle_inventory.extruder_count() > 0 {
            if let Some(nozzle_diameter_opt) = floats_mut(&mut out, "nozzle_diameter", false)? {
                *nozzle_diameter_opt = smallest_installed_diameters(&self.nozzle_inventory, nozzle_diameter_opt);
            }
        }

        let mut selected_nozzle_diameters: Vec<String> = Vec::new();
        if multi_diameter_mode {
            selected_nozzle_diameters = strings_ref(&self.project_config, "filament_diameter_map").expect("filament_diameter_map").clone();
        }
        selected_nozzle_diameters.resize(filament_ids.len(), String::new());
        *strings_mut(&mut out, "filament_diameter_map", true)?.expect("filament_diameter_map") = selected_nozzle_diameters;

        set_printer_technology(&mut out, PrinterTechnology::PtFFF)?;
        Ok(out)
    }

    /// PresetBundle.cpp:5090-5139 `DynamicPrintConfig PresetBundle::full_sla_config() const`
    fn full_sla_config(&self) -> CfgResult<DynamicPrintConfig> {
        let mut out = DynamicPrintConfig::new();
        out.apply(static_print_config_defaults("SLAFullPrintConfig"), false)?;
        out.apply(&self.sla_prints.get_edited_preset().config, false)?;
        out.apply(&self.sla_materials.get_edited_preset().config, false)?;
        out.apply(&self.printers.get_edited_preset().config, false)?;
        // There are no project configuration values as of now, the project_config is reserved for FFF printers.
        //    out.apply(this->project_config);

        // Collect the "compatible_printers_condition" and "inherits" values over all presets (sla_prints, sla_materials, printers) into a single vector.
        let mut compatible_printers_condition: Vec<String> = Vec::new();
        let mut compatible_prints_condition: Vec<String> = Vec::new();
        let mut inherits: Vec<String> = Vec::new();
        compatible_printers_condition.push(self.sla_prints.get_edited_preset().compatible_printers_condition().to_string());
        inherits.push(self.sla_prints.get_edited_preset().inherits().to_string());
        compatible_printers_condition.push(self.sla_materials.get_edited_preset().compatible_printers_condition().to_string());
        compatible_prints_condition.push(self.sla_materials.get_edited_preset().compatible_prints_condition().to_string());
        inherits.push(self.sla_materials.get_edited_preset().inherits().to_string());
        inherits.push(self.printers.get_edited_preset().inherits().to_string());

        // These two value types clash between the print and filament profiles. They should be renamed.
        out.erase("compatible_printers");
        out.erase("compatible_printers_condition");
        out.erase("inherits");

        out.option_typed_mut::<ConfigOptionString>("sla_print_settings_id", true)?.expect("sla_print_settings_id").value = self.sla_prints.get_selected_preset_name();
        out.option_typed_mut::<ConfigOptionString>("sla_material_settings_id", true)?.expect("sla_material_settings_id").value =
            self.sla_materials.get_selected_preset_name();
        out.option_typed_mut::<ConfigOptionString>("printer_settings_id", true)?.expect("printer_settings_id").value = self.printers.get_selected_preset_name();

        // Serialize the collected "compatible_printers_condition" and "inherits" fields.
        // There will be 1 + num_exturders fields for "inherits" and 2 + num_extruders for "compatible_printers_condition" stored.
        // The vector will not be stored if all fields are empty strings.
        add_if_some_non_empty(&mut out, compatible_printers_condition, "compatible_machine_expression_group");
        add_if_some_non_empty(&mut out, compatible_prints_condition, "compatible_process_expression_group");
        add_if_some_non_empty(&mut out, inherits, "inherits_group");

        set_printer_technology(&mut out, PrinterTechnology::PtSLA)?;
        Ok(out)
    }
}

impl PresetBundle {
    /// PresetBundle.hpp:367-368 `void load_config_from_wizard(const std::string &name, DynamicPrintConfig config, Semver file_version)`
    pub fn load_config_from_wizard(&mut self, name: &str, config: DynamicPrintConfig, file_version: Semver) -> CfgResult<()> {
        self.load_config_file_config(name, false, config, file_version, true)
    }

    /// PresetBundle.hpp:370-371 `void load_config_model(const std::string &name, DynamicPrintConfig config, Semver file_version = Semver())`
    pub fn load_config_model(&mut self, name: &str, config: DynamicPrintConfig, file_version: Semver) -> CfgResult<()> {
        self.load_config_file_config(name, true, config, file_version, false)
    }

    /// PresetBundle.cpp:5141-5210 `ConfigSubstitutions PresetBundle::load_config_file(const std::string &path, ForwardCompatibilitySubstitutionRule compatibility_rule)`:
    /// load an external config file containing the print, filament and printer presets (a G-code
    /// containing the full set of parameters). If the file is loaded successfully, its print /
    /// filament / printer profiles will be activated.
    pub fn load_config_file(&mut self, path: &str, compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<ConfigSubstitutions> {
        if crate::utils::is_gcode_file(path) {
            let mut config = DynamicPrintConfig::new();
            config.apply(static_print_config_defaults("FullPrintConfig"), false)?;
            let config_substitutions = config.load_from_gcode_file(path, compatibility_rule)?;
            Preset::normalize(&mut config)?;
            self.load_config_file_config(path, true, config, Semver::new(), false)?;
            return Ok(config_substitutions);
        }

        Err(runtime_error(format!("Unknown configuration file: {}", path)))
        // 1) Try to load the config file into a boost property tree. (commented out at the pin)
    }
}

/// PresetBundle.cpp:5218 `static std::map<std::string, std::map<std::string, std::string>> filament_preset_convert`
static FILAMENT_PRESET_CONVERT: Mutex<BTreeMap<String, BTreeMap<String, String>>> = Mutex::new(BTreeMap::new());

/// `json::items()`: the members of an object, the elements of an array keyed by their index, and a
/// primitive as itself with an empty key.
fn json_items(j: &serde_json::Value) -> Vec<(String, serde_json::Value)> {
    match j {
        serde_json::Value::Object(m) => m.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
        serde_json::Value::Array(a) => a.iter().enumerate().map(|(i, v)| (i.to_string(), v.clone())).collect(),
        serde_json::Value::Null => Vec::new(),
        other => vec![(String::new(), other.clone())],
    }
}

/// PresetBundle.cpp:5213-5255 `bool find_filament_preset_map_name(std::string machine_name, std::string filament_old_name, std::string &filament_new_name)`:
/// some filament presets split from one to separate ones; the map of
/// resources/profiles/BBL/filament/filament_name_map.json records them. A value that is not a
/// string throws (`json::type_error` of the implicit conversion), which the C++ does not catch.
pub fn find_filament_preset_map_name(machine_name: &str, filament_old_name: &str, filament_new_name: &mut String) -> CfgResult<bool> {
    let mut filament_preset_convert = FILAMENT_PRESET_CONVERT.lock().unwrap_or_else(|e| e.into_inner());

    if filament_preset_convert.is_empty() {
        let path = boost_path_append(
            &boost_path_append(&boost_path_append(&boost_path_append(&crate::utils::resources_dir().to_string_lossy(), "profiles"), "BBL"), "filament"),
            "filament_name_map.json",
        );

        let data = match std::fs::read(&path) {
            Ok(d) => d,
            // Failed to open filament preset map
            Err(_) => return Ok(false),
        };

        let j = match crate::config::nlohmann_json::parse(&data) {
            Ok(j) => j,
            // JSON parse error
            Err(_) => return Ok(false),
        };

        for (printer_name, filament_json) in json_items(&j) {
            let mut inner_map: BTreeMap<String, String> = BTreeMap::new();
            for (old_name, new_name) in json_items(&filament_json) {
                match new_name {
                    serde_json::Value::String(s) => {
                        inner_map.insert(old_name, s);
                    }
                    other => {
                        return Err(runtime_error(format!(
                            "[json.exception.type_error.302] type must be string, but is {}",
                            match other {
                                serde_json::Value::Null => "null",
                                serde_json::Value::Bool(_) => "boolean",
                                serde_json::Value::Number(_) => "number",
                                serde_json::Value::Array(_) => "array",
                                _ => "object",
                            }
                        )))
                    }
                }
            }
            filament_preset_convert.insert(printer_name, inner_map);
        }
    }

    if let Some(filament_maps) = filament_preset_convert.get(machine_name) {
        if let Some(n) = filament_maps.get(filament_old_name) {
            *filament_new_name = n.clone();
            return Ok(true);
        }
    }
    Ok(false)
}

/// PresetBundle.cpp:5257-5262 `void convert_filament_preset_name(std::string &machine_name, std::string &filament_name)`:
/// convert the old filament preset to new one after split.
pub fn convert_filament_preset_name(machine_name: &str, filament_name: &mut String) -> CfgResult<()> {
    let mut filament_new_name = String::new();
    if find_filament_preset_map_name(machine_name, filament_name, &mut filament_new_name)? {
        *filament_name = filament_new_name;
    }
    Ok(())
}

/// The `clear_compatible_printers` lambda of PresetBundle.cpp:5269-5276.
fn clear_compatible_printers(config: &mut DynamicPrintConfig) -> CfgResult<()> {
    if let Some(opt_compatible) = config.optptr_mut("compatible_printers", false)? {
        if opt_compatible.r#type() == ConfigOptionType::CoStrings {
            if let Some(v) = opt_compatible.vector_values_any_mut().and_then(|v| v.downcast_mut::<Vec<String>>()) {
                v.clear();
            }
        }
    }
    Ok(())
}

/// The set of keys of `different_settings_to_system[idx]` plus ignore_settings_list
/// (PresetBundle.cpp:5401-5409 and the like).
fn different_keys_set(different_values: &[String], idx: usize) -> BTreeSet<String> {
    let mut keys_vector: Vec<String> = Vec::new();
    unescape_strings_cstyle(&different_values[idx], &mut keys_vector);
    let mut set: BTreeSet<String> = keys_vector.into_iter().collect();
    set.extend(ignore_settings_list());
    set
}

impl PresetBundle {
    /// The `load_preset` lambda of PresetBundle.cpp:5377-5394: split the
    /// "compatible_printers_condition" and "inherits" values one by one from a single vector to the
    /// print & printer profiles, then load the preset.
    #[allow(clippy::too_many_arguments)]
    fn load_config_file_config_preset(
        presets: &mut PresetCollection,
        config: &mut DynamicPrintConfig,
        inherits_values: &[String],
        compatible_printers_condition_values: &[String],
        compatible_prints_condition_values: &[String],
        is_external: bool,
        name: &str,
        name_or_path: &str,
        file_version: &Semver,
        selected: bool,
        idx: usize,
        key: &str,
        different_keys: &BTreeSet<String>,
        filament_id: &str,
    ) -> CfgResult<()> {
        *Preset::inherits_cfg(config) = inherits_values[idx].clone();
        *Preset::compatible_printers_condition_cfg(config) = compatible_printers_condition_values[idx].clone();
        if idx > 0 && idx - 1 < compatible_prints_condition_values.len() {
            *Preset::compatible_prints_condition_cfg(config) = compatible_prints_condition_values[idx - 1].clone();
        }
        if is_external {
            let original_name = config.opt_string_mut(key, true).clone();
            presets.load_external_preset(
                name_or_path,
                name,
                &original_name,
                config,
                different_keys,
                crate::preset::LoadAndSelect::Always,
                file_version.clone(),
                filament_id,
            )?;
        } else {
            let inherits_empty = Preset::inherits_cfg(config).is_empty();
            let path = presets.path_from_name(name, inherits_empty);
            presets.load_preset(&path, name, config, selected, file_version.clone())?.save(None)?;
        }
        Ok(())
    }

    /// PresetBundle.cpp:5263-5613 `void PresetBundle::load_config_file_config(const std::string &name_or_path, bool is_external, DynamicPrintConfig &&config, Semver file_version, bool selected)`:
    /// load a config file from a boost property_tree. `is_external == false` if called from
    /// ConfigWizard.
    fn load_config_file_config(&mut self, name_or_path: &str, is_external: bool, mut config: DynamicPrintConfig, file_version: Semver, selected: bool) -> CfgResult<()> {
        let printer_technology = Preset::printer_technology_cfg(&config);

        clear_compatible_printers(&mut config)?;

        // BBS: use filament_colour insteadof filament_settings_id, filament_settings_id sometimes is not generated
        let num_filaments = strings_ref(&config, "filament_colour").map_or(0, |v| v.len());
        if num_filaments == 0 {
            return Err(runtime_error(format!("Invalid configuration file: {}", name_or_path)));
        }

        // Make a copy of the "compatible_machine_expression_group" and "inherits_group" vectors, which
        // accumulate values over all presets (print, filaments, printers).
        // These values will be distributed into their particular presets when loading.
        let take_strings = |config: &mut DynamicPrintConfig, key: &str| -> CfgResult<Vec<String>> {
            Ok(std::mem::take(strings_mut(config, key, true)?.unwrap_or_else(|| panic!("option<ConfigOptionStrings>({key}, true)"))))
        };
        let mut compatible_printers_condition_values = take_strings(&mut config, "compatible_machine_expression_group")?;
        let mut compatible_prints_condition_values = take_strings(&mut config, "compatible_process_expression_group")?;
        let mut inherits_values = take_strings(&mut config, "inherits_group")?;
        let mut filament_ids = take_strings(&mut config, "filament_ids")?;
        let print_compatible_printers = take_strings(&mut config, "print_compatible_printers")?;
        //BBS: add different settings check logic
        let _has_different_settings_to_system = config.option("different_settings_to_system").is_some();
        let mut different_values = take_strings(&mut config, "different_settings_to_system")?;
        Preset::compatible_printers_condition_cfg(&mut config);
        Preset::compatible_prints_condition_cfg(&mut config);
        Preset::inherits_cfg(&mut config);
        compatible_printers_condition_values.resize(num_filaments + 2, String::new());
        compatible_prints_condition_values.resize(num_filaments, String::new());
        inherits_values.resize(num_filaments + 2, String::new());
        different_values.resize(num_filaments + 2, String::new());
        filament_ids.resize(num_filaments, String::new());
        // The "default_filament_profile" will be later extracted into the printer profile.
        match printer_technology {
            PrinterTechnology::PtFFF => {
                config.option_typed_mut::<ConfigOptionString>("default_print_profile", true)?;
                strings_mut(&mut config, "default_filament_profile", true)?;
            }
            PrinterTechnology::PtSLA => {
                config.option_typed_mut::<ConfigOptionString>("default_sla_print_profile", true)?;
                config.option_typed_mut::<ConfigOptionString>("default_sla_material_profile", true)?;
            }
            _ => {}
        }

        let mut process_multi_extruder = false;
        let mut filament_variant_index: Vec<usize> = Vec::new();
        let mut extruder_variant_count: usize = 0;
        if ints_ref(&config, "filament_self_index").is_none() {
            let filament_self_indice = ints_mut(&mut config, "filament_self_index", true)?.expect("filament_self_index");
            filament_self_indice.resize(num_filaments, 0);
            for index in 0..num_filaments {
                filament_self_indice[index] = (index + 1) as i32;
            }
        }
        let filament_self_indice: Vec<i32> = std::mem::take(ints_mut(&mut config, "filament_self_index", false)?.expect("filament_self_index"));
        if config.option("extruder_variant_list").is_some() {
            //3mf support multiple extruder logic
            let _extruder_count = floats_ref(&config, "nozzle_diameter").expect("nozzle_diameter").len();
            extruder_variant_count = strings_mut(&mut config, "filament_extruder_variant", true)?.expect("filament_extruder_variant").len();
            if extruder_variant_count != filament_self_indice.len() || extruder_variant_count < num_filaments {
                // invalid config file, can not find suitable filament_extruder_variant or filament_self_index
                return Err(runtime_error(format!("Invalid configuration file: {}", name_or_path)));
            }
            if num_filaments != extruder_variant_count {
                process_multi_extruder = true;
                filament_variant_index.resize(num_filaments, 0);

                let mut cur_filament_id: usize = 1;
                for index in 0..filament_self_indice.len() {
                    if filament_self_indice[index] as i64 == cur_filament_id as i64 {
                        filament_variant_index[cur_filament_id - 1] = index;
                        cur_filament_id += 1;
                        if cur_filament_id > num_filaments {
                            break;
                        }
                    }
                }
            }
        }
        //no need to parse extruder_ams_count
        let extruder_ams_count = take_strings(&mut config, "extruder_ams_count")?;
        config.erase("extruder_ams_count");
        config.erase("enable_filament_dynamic_map");
        if self.extruder_ams_counts.is_empty() {
            self.extruder_ams_counts = get_extruder_ams_count(&extruder_ams_count);
        }

        // 1) Create a name from the file name.
        // Keep the suffix (.ini, .gcode, .amf, .3mf etc) to differentiate it from the normal profiles.
        let name = if is_external { path_filename(name_or_path) } else { name_or_path.to_string() };

        // 2) If the loading succeeded, split and load the config into print / filament / printer settings.
        // First load the print and printer presets.

        match Preset::printer_technology_cfg(&config) {
            PrinterTechnology::PtFFF => {
                //BBS: add different settings logic
                let print_different_keys_set = different_keys_set(&different_values, 0);
                if !print_compatible_printers.is_empty() {
                    *strings_mut(&mut config, "compatible_printers", true)?.expect("compatible_printers") = print_compatible_printers.clone();
                }

                Self::load_config_file_config_preset(
                    &mut self.prints,
                    &mut config,
                    &inherits_values,
                    &compatible_printers_condition_values,
                    &compatible_prints_condition_values,
                    is_external,
                    &name,
                    name_or_path,
                    &file_version,
                    selected,
                    0,
                    "print_settings_id",
                    &print_different_keys_set,
                    "",
                )?;

                //clear compatible printers
                clear_compatible_printers(&mut config)?;

                let printer_different_keys_set = different_keys_set(&different_values, num_filaments + 1);
                Self::load_config_file_config_preset(
                    &mut self.printers.base,
                    &mut config,
                    &inherits_values,
                    &compatible_printers_condition_values,
                    &compatible_prints_condition_values,
                    is_external,
                    &name,
                    name_or_path,
                    &file_version,
                    selected,
                    num_filaments + 1,
                    "printer_settings_id",
                    &printer_different_keys_set,
                    "",
                )?;

                // 3) Now load the filaments. If there are multiple filament presets, split them and load them.
                strings_mut(&mut config, "filament_settings_id", true)?.expect("filament_settings_id").resize(num_filaments, String::new());

                let old_machine_profile_name = config.option_typed_mut::<ConfigOptionString>("printer_settings_id", true)?.expect("printer_settings_id").value.clone();
                // `machine_inherit` is the "inherits" option of `config`, read when it is used below.
                config.option_typed_mut::<ConfigOptionString>("inherits", true)?;

                if num_filaments <= 1 {
                    // Split the "compatible_printers_condition" and "inherits" from the cummulative vectors to separate filament presets.
                    *Preset::inherits_cfg(&mut config) = inherits_values[1].clone();
                    *Preset::compatible_printers_condition_cfg(&mut config) = compatible_printers_condition_values[1].clone();
                    *Preset::compatible_prints_condition_cfg(&mut config) = compatible_prints_condition_values[0].clone();

                    //BBS: add different settings logic
                    let filament_different_keys_set = different_keys_set(&different_values, 1);

                    let filament_id = filament_ids[0].clone();
                    let loaded_name: String;
                    if is_external {
                        let machine_inherit = Preset::inherits_cfg(&mut config).clone();
                        let machine_name = if machine_inherit.is_empty() { old_machine_profile_name.clone() } else { machine_inherit };
                        if Preset::inherits_cfg(&mut config).is_empty() {
                            let names = strings_mut(&mut config, "filament_settings_id", false)?.expect("filament_settings_id");
                            convert_filament_preset_name(&machine_name, &mut names[0])?;
                        } else {
                            convert_filament_preset_name(&machine_name, Preset::inherits_cfg(&mut config))?;
                        }
                        let original_name = strings_ref(&config, "filament_settings_id").expect("filament_settings_id")[0].clone();
                        let (loaded, _) = self.filaments.load_external_preset(
                            name_or_path,
                            &name,
                            &original_name,
                            &config,
                            &filament_different_keys_set,
                            crate::preset::LoadAndSelect::Always,
                            file_version.clone(),
                            &filament_id,
                        )?;
                        loaded_name = loaded.name.clone();
                    } else {
                        // called from Config Wizard.
                        let inherits_empty = Preset::inherits_cfg(&mut config).is_empty();
                        let path = self.filaments.path_from_name(&name, inherits_empty);
                        let loaded = self.filaments.load_preset(&path, &name, &config, true, file_version.clone())?;
                        loaded.save(None)?;
                        loaded_name = loaded.name.clone();
                    }
                    self.filament_presets.clear();
                    self.filament_presets.push(loaded_name);
                } else {
                    // Split the filament presets, load each of them separately.
                    // Expand per-filament overrides onto variant slots before they are split,
                    // otherwise a length-1 edit is read as the previous filament's High Flow slot.
                    crate::print_config::config_def::align_filament_variant_overrides(&mut config)?;

                    let default_config = self.filaments.default_preset(0).config.clone();
                    let mut configs: Vec<DynamicPrintConfig> = vec![default_config.clone(); num_filaments];
                    // loop through options and scatter them into configs.
                    for key in default_config.keys() {
                        let other_opt = match config.option_mut(&key, false)? {
                            None => continue,
                            Some(o) => o,
                        };
                        if other_opt.is_scalar() {
                            for c in configs.iter_mut() {
                                c.option_mut(&key, false)?.expect("option").set(other_opt)?;
                            }
                        } else if key != "compatible_printers" && key != "compatible_prints" {
                            for i in 0..configs.len() {
                                if process_multi_extruder && filament_options_with_variant.contains(&key) {
                                    let other_opt_vec = other_opt.as_vector_base_mut().expect("vector");
                                    if other_opt_vec.size() != extruder_variant_count {
                                        other_opt_vec.resize(extruder_variant_count, None)?;
                                    }
                                    let next_index = if i < configs.len() - 1 { filament_variant_index[i + 1] } else { extruder_variant_count };
                                    configs[i]
                                        .option_mut(&key, false)?
                                        .expect("option")
                                        .as_vector_base_mut()
                                        .expect("vector")
                                        .set_range(other_opt, filament_variant_index[i], next_index - filament_variant_index[i])?;
                                } else {
                                    configs[i].option_mut(&key, false)?.expect("option").as_vector_base_mut().expect("vector").set_at(other_opt, 0, i)?;
                                }
                            }
                        }
                    }
                    // Load the configs into this->filaments and make them active.
                    self.filament_presets = vec![String::new(); configs.len()];
                    // To avoid incorrect selection of the first filament preset (means a value of Preset->m_idx_selected)
                    // in a case when next added preset take a place of previosly selected preset,
                    // we should add presets from last to first
                    let mut any_modified = false;
                    let machine_inherit = Preset::inherits_cfg(&mut config).clone();
                    let mut old_filament_profile_names: Vec<String> = strings_ref(&config, "filament_settings_id").expect("filament_settings_id").clone();
                    for i in (0..configs.len()).rev() {
                        let cfg = &mut configs[i];
                        // Split the "compatible_printers_condition" and "inherits" from the cummulative vectors to separate filament presets.
                        *cfg.opt_string_mut("compatible_printers_condition", true) = compatible_printers_condition_values[i + 1].clone();
                        *cfg.opt_string_mut("compatible_prints_condition", true) = compatible_prints_condition_values[i].clone();
                        *cfg.opt_string_mut("inherits", true) = inherits_values[i + 1].clone();

                        //BBS: add different settings logic
                        let filament_different_keys_set = different_keys_set(&different_values, i + 1);

                        let filament_id = filament_ids[i].clone();

                        // Load all filament presets, but only select the first one in the preset dialog.
                        let machine_name = if machine_inherit.is_empty() { old_machine_profile_name.clone() } else { machine_inherit.clone() };
                        if cfg.opt_string_mut("inherits", true).is_empty() && i < old_filament_profile_names.len() {
                            convert_filament_preset_name(&machine_name, &mut old_filament_profile_names[i])?;
                        } else {
                            convert_filament_preset_name(&machine_name, cfg.opt_string_mut("inherits", true))?;
                        }
                        let original_name = if i < old_filament_profile_names.len() { old_filament_profile_names[i].clone() } else { String::new() };
                        let select = if i == 0 {
                            crate::preset::LoadAndSelect::Always
                        } else if any_modified {
                            crate::preset::LoadAndSelect::Never
                        } else {
                            crate::preset::LoadAndSelect::OnlyIfModified
                        };
                        let cfg = std::mem::take(cfg);
                        let (loaded, modified) = self.filaments.load_external_preset(
                            name_or_path,
                            &name,
                            &original_name,
                            &cfg,
                            &filament_different_keys_set,
                            select,
                            file_version.clone(),
                            &filament_id,
                        )?;
                        any_modified |= modified;
                        self.filament_presets[i] = loaded.name.clone();
                    }
                    // old_filament_profile_names points into `config` in the C++.
                    *strings_mut(&mut config, "filament_settings_id", false)?.expect("filament_settings_id") = old_filament_profile_names;
                }

                // Omitted 3mf keys keep the previous value. Reset them from print_config_def.
                // Wizard loads pass is_external false and keep their partial merge.
                if is_external {
                    for key in S_PROJECT_OPTIONS.iter() {
                        if config.has(key) {
                            continue;
                        }
                        let optdef = match crate::print_config::config_def::print_config_def().get(key) {
                            None => continue,
                            Some(d) => d,
                        };
                        self.project_config.erase(key);
                        self.project_config.set_key_value(key, optdef.create_default_option()?);
                    }
                }

                // 4) Load the project config values (the per extruder wipe matrix etc).
                self.project_config.apply_only(&config, &s_project_options(), false)?;
                // The copy above replaced the nozzle keys; write them again for the printer of the file.
                // A project load then replaces them with the nozzles of the file in open_project().
                self.nozzle_select_printer()?;
                let n = self.filament_presets.len();
                strings_mut(&mut self.project_config, "filament_diameter_map", false)?.expect("filament_diameter_map").resize(n, String::new());
            }
            PrinterTechnology::PtSLA => {
                /*std::set<std::string> different_keys_set;
                load_preset(this->sla_prints, 0, "sla_print_settings_id", different_keys_set);
                load_preset(this->sla_materials, 1, "sla_material_settings_id", different_keys_set);
                load_preset(this->printers, 2, "printer_settings_id", different_keys_set);*/
            }
            _ => {}
        }

        self.update_compatible(PresetSelectCompatibleType::Never)?;
        self.update_multi_material_filament_presets(usize::MAX)?;

        //BBS
        //const std::string &physical_printer = config.option<ConfigOptionString>("physical_printer_settings_id", true)->value;
        let physical_printer = String::new();
        if self.printers.get_edited_preset().is_external || physical_printer.is_empty() {
            self.physical_printers.unselect_printer();
        } else {
            // Activate the physical printer profile if possible.
            let edited_name = self.printers.get_edited_preset().name.clone();
            let pp_name = self.physical_printers.find_printer(&physical_printer, true).filter(|pp| pp.preset_names.contains(&edited_name)).map(|pp| pp.name.clone());
            match pp_name {
                Some(n) => self.physical_printers.select_printer_with_preset(&n, &edited_name),
                None => self.physical_printers.unselect_printer(),
            }
        }
        Ok(())
    }
}

/// `boost::iequals(a, b)` for ASCII keys.
fn iequals(a: &str, b: &str) -> bool {
    a.len() == b.len() && a.as_bytes().eq_ignore_ascii_case(b.as_bytes())
}

/// `std::string s = j;` of a json value: `json::type_error` (302) unless it is a string.
fn json_string(v: &serde_json::Value) -> CfgResult<String> {
    match v {
        serde_json::Value::String(s) => Ok(s.clone()),
        other => Err(runtime_error(format!(
            "[json.exception.type_error.302] type must be string, but is {}",
            match other {
                serde_json::Value::Null => "null",
                serde_json::Value::Bool(_) => "boolean",
                serde_json::Value::Number(_) => "number",
                serde_json::Value::Array(_) => "array",
                _ => "object",
            }
        ))),
    }
}

/// `boost::nowide::ifstream ifs(path); json j; ifs >> j;`: a file that cannot be opened reads as
/// empty, which nlohmann rejects; `Err(Ok(what))` is a parse_error, `Err(Err(what))` another
/// json exception.
fn read_json_file(path: &str) -> Result<serde_json::Value, Result<String, String>> {
    let data = std::fs::read(path).unwrap_or_default();
    match crate::config::nlohmann_json::parse(&data) {
        Ok(j) => Ok(j),
        Err(crate::config::nlohmann_json::Error::Parse(what)) => Err(Ok(what)),
        Err(crate::config::nlohmann_json::Error::Other(what)) => Err(Err(what)),
    }
}

/// `boost::algorithm::starts_with(it.value(), prefix)` of PresetBundle.cpp:7027: the json value is
/// the range, so its elements are compared with the characters of the prefix (a char converts to a
/// json number). A primitive is a range of itself, an array of its elements, null an empty range:
/// only an array whose first elements are the character codes of the prefix starts with it (the
/// string "SLA" does not).
fn json_range_starts_with(value: &serde_json::Value, prefix: &str) -> bool {
    let elements: Vec<&serde_json::Value> = match value {
        serde_json::Value::Null => Vec::new(),
        serde_json::Value::Array(a) => a.iter().collect(),
        serde_json::Value::Object(m) => m.values().collect(),
        other => vec![other],
    };
    let prefix = prefix.as_bytes();
    if elements.len() < prefix.len() {
        return false;
    }
    prefix.iter().zip(elements.iter()).all(|(c, e)| match e {
        serde_json::Value::Number(n) => n.as_f64().map_or(false, |v| v == *c as f64),
        _ => false,
    })
}

/// `Slic3r::sort_remove_duplicates(vec)` (libslic3r.h)
fn sort_remove_duplicates(v: &mut Vec<String>) {
    v.sort();
    v.dedup();
}

/// `boost::trim_right(s)`: white space of the classic locale removed from the end.
fn trim_right(s: &str) -> String {
    s.trim_end_matches(|c: char| c == ' ' || ('\t'..='\r').contains(&c)).to_string()
}

/// PresetBundle.cpp:6338-6345 `struct ParallelPresetLoadData`
struct ParallelPresetLoadData {
    subfile: (String, String),
    config_src: DynamicPrintConfig,
    reason: String,
    key_values: BTreeMap<String, String>,
    context: crate::config::ConfigSubstitutionContext,
}

/// The `get_name_and_subpath` lambda of PresetBundle.cpp:6199-6225.
fn get_name_and_subpath(value: &serde_json::Value, subfile_map: &mut Vec<(String, String)>) -> CfgResult<()> {
    if let serde_json::Value::Array(items) = value {
        for item in items {
            if let serde_json::Value::Object(members) = item {
                let mut name = String::new();
                let mut subpath = String::new();
                for (key, v) in members {
                    if v.is_string() {
                        if iequals(key, crate::preset::BBL_JSON_KEY_NAME) {
                            name = json_string(v)?;
                        } else if iequals(key, crate::preset::BBL_JSON_KEY_SUB_PATH) {
                            subpath = json_string(v)?;
                        }
                    } else {
                        // invalid value type (logged only)
                    }
                }
                if !name.is_empty() && !subpath.is_empty() {
                    subfile_map.push((name, subpath));
                }
            } else {
                // invalid type (logged only)
            }
        }
    } else {
        // invalid type (logged only)
    }
    Ok(())
}

impl PresetBundle {
    /// PresetBundle.cpp:6179-6990 `std::pair<PresetsConfigSubstitutions, size_t> PresetBundle::load_vendor_configs_from_json(const std::string &path, const std::string &vendor_name, LoadConfigBundleAttributes flags, ForwardCompatibilitySubstitutionRule compatibility_rule)`
    ///
    /// The pin builds with `PARALLEL_LOAD_PRESET 1` (PresetBundle.cpp:36): the json files of a list
    /// are read in parallel and then parsed in list order (parse_config). FIDELITY-NOTE: the
    /// printer models are appended to the vendor profile from parallel tasks, in an order that
    /// depends on the scheduling; here they are appended in list order (the order of the serial
    /// build of the same file). The json files are read one after the other.
    pub fn load_vendor_configs_from_json(
        &mut self,
        path: &str,
        vendor_name: &str,
        flags: LoadConfigBundleAttributes,
        compatibility_rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<(PresetsConfigSubstitutions, usize)> {
        // Enable substitutions for user config bundle, throw an exception when loading a system profile.
        let mut substitution_context = crate::config::ConfigSubstitutionContext::new(compatibility_rule);
        let mut substitutions = PresetsConfigSubstitutions::new();

        if flags.has(LoadConfigBundleAttribute::ResetUserProfile) || flags.has(LoadConfigBundleAttribute::LoadSystem) {
            // Reset this bundle, delete user profile files if SaveImported.
            self.reset(flags.has(LoadConfigBundleAttribute::SaveImported));
        }

        // 1) load the vroot json and construct the vendor profile
        let mut vendor_profile = VendorProfile::with_id(vendor_name.to_string());
        let root_file = format!("{}/{}.json", path, vendor_name);
        let mut machine_model_subfiles: Vec<(String, String)> = Vec::new();
        let mut process_subfiles: Vec<(String, String)> = Vec::new();
        let mut filament_subfiles: Vec<(String, String)> = Vec::new();
        let mut machine_subfiles: Vec<(String, String)> = Vec::new();
        match read_json_file(&root_file) {
            Err(Ok(what)) => {
                return Err(ConfigurationError::new(format!(
                    "Failed loading configuration file {}: {}\nSuggest closing studio, deleting all files in {}, and restarting studio",
                    root_file, what, path
                )));
            }
            Err(Err(what)) => return Err(runtime_error(what)),
            Ok(j) => {
                //parse the json elements
                for (key, value) in json_items(&j) {
                    if iequals(&key, BBL_JSON_KEY_VERSION) {
                        //get version
                        let version_str = json_string(&value)?;
                        match Semver::parse(&version_str) {
                            None => {
                                return Err(ConfigurationError::new(format!(
                                    "vendor {}'s config version: {} invalid\nSuggest closing studio, deleting all files in {}, and restarting studio",
                                    vendor_name, version_str, path
                                )));
                            }
                            Some(v) => vendor_profile.config_version = v,
                        }
                    } else if iequals(&key, crate::preset::BBL_JSON_KEY_URL) {
                        //get url
                        vendor_profile.config_update_url = json_string(&value)?;
                    } else if iequals(&key, crate::preset::BBL_JSON_KEY_DESCRIPTION) {
                        //get description
                    } else if iequals(&key, BBL_JSON_KEY_NAME) {
                        //get name
                        vendor_profile.name = json_string(&value)?;
                    } else if iequals(&key, crate::preset::BBL_JSON_KEY_MACHINE_MODEL_LIST) {
                        //get machine model list
                        get_name_and_subpath(&value, &mut machine_model_subfiles)?;
                    } else if iequals(&key, crate::preset::BBL_JSON_KEY_PROCESS_LIST) {
                        //get process list
                        get_name_and_subpath(&value, &mut process_subfiles)?;
                    } else if iequals(&key, crate::preset::BBL_JSON_KEY_FILAMENT_LIST) {
                        //get filament list
                        get_name_and_subpath(&value, &mut filament_subfiles)?;
                    } else if iequals(&key, crate::preset::BBL_JSON_KEY_MACHINE_LIST) {
                        //get machine list
                        get_name_and_subpath(&value, &mut machine_subfiles)?;
                    }
                }
            }
        }

        if flags.has(LoadConfigBundleAttribute::LoadFilamentOnly) {
            machine_model_subfiles.clear();
            machine_subfiles.clear();
            process_subfiles.clear();
        }

        //2) paste the machine model
        for machine_model in machine_model_subfiles.iter() {
            let subfile = format!("{}/{}/{}", path, vendor_name, machine_model.1);
            if !fs_exists(&subfile) {
                // file not exist
                continue;
            }
            let mut model = self.load_vendor_configs_from_json_2(&subfile)?;
            model.id = machine_model.0.clone();
            if !model.id.is_empty() && !model.variants.is_empty() {
                vendor_profile.models.push(model);
            }
        }

        //insert the vendor profile
        self.vendors.entry(vendor_name.to_string()).or_insert_with(|| Arc::new(vendor_profile));
        let current_vendor_profile: Arc<VendorProfile> = self.vendors[vendor_name].clone();

        if flags.has(LoadConfigBundleAttribute::LoadVendorOnly) {
            return Ok((PresetsConfigSubstitutions::new(), 0));
        }

        // 3) paste the process/filament/print configs
        let mut presets_loaded: usize = 0;
        let mut configs: BTreeMap<String, DynamicPrintConfig> = BTreeMap::new();
        let mut filament_id_maps: BTreeMap<String, String> = BTreeMap::new();
        let mut description_maps: BTreeMap<String, String> = BTreeMap::new();

        let lists: [(Type, Vec<(String, String)>, &str); 3] =
            [(Type::TypePrint, process_subfiles, "process"), (Type::TypeFilament, filament_subfiles, "filament"), (Type::TypePrinter, machine_subfiles, "printer")];
        for (collection_type, subfiles, _what) in lists {
            //3.1) paste the process, 3.2) paste the filaments, 3.3) paste the printers
            configs.clear();
            filament_id_maps.clear();

            let mut parallel_load_data: Vec<ParallelPresetLoadData> = Vec::new();
            for subfile in subfiles {
                parallel_load_data.push(ParallelPresetLoadData {
                    subfile,
                    config_src: DynamicPrintConfig::new(),
                    reason: String::new(),
                    key_values: BTreeMap::new(),
                    context: crate::config::ConfigSubstitutionContext::new(substitution_context.rule),
                });
            }
            for config_data in parallel_load_data.iter_mut() {
                let subfile = format!("{}/{}/{}", path, vendor_name, config_data.subfile.1);
                config_data.config_src.load_from_json_with_context(&subfile, &mut config_data.context, false, &mut config_data.key_values, &mut config_data.reason);
            }
            for load_data in parallel_load_data.iter_mut() {
                let reason = self.parse_config(
                    load_data,
                    &mut substitution_context,
                    &mut substitutions,
                    flags,
                    &mut configs,
                    &mut filament_id_maps,
                    collection_type,
                    &mut presets_loaded,
                    &mut description_maps,
                    path,
                    vendor_name,
                    &current_vendor_profile,
                )?;
                if !reason.is_empty() {
                    // parse error
                    let subfile_path = format!("{}/{}/{}", path, vendor_name, load_data.subfile.1);
                    return Err(ConfigurationError::new(format!(
                        "Failed loading configuration file {}\nSuggest closing studio, deleting all files in {}, and restarting studio",
                        subfile_path, path
                    )));
                }
            }
        }
        Ok((substitutions, presets_loaded))
    }

    /// The `parse_config` lambda of PresetBundle.cpp:6348-6598 (the build's PARALLEL_LOAD_PRESET
    /// branch). Returns the reason of a failure, empty on success; an exception the lambda does not
    /// catch is an `Err`.
    #[allow(clippy::too_many_arguments)]
    fn parse_config(
        &mut self,
        preset_load_data: &mut ParallelPresetLoadData,
        substitution_context: &mut crate::config::ConfigSubstitutionContext,
        substitutions: &mut PresetsConfigSubstitutions,
        flags: LoadConfigBundleAttributes,
        config_maps: &mut BTreeMap<String, DynamicPrintConfig>,
        filament_id_maps: &mut BTreeMap<String, String>,
        collection_type: Type,
        count: &mut usize,
        description_maps: &mut BTreeMap<String, String>,
        path: &str,
        vendor_name: &str,
        current_vendor_profile: &Arc<VendorProfile>,
    ) -> CfgResult<String> {
        let subfile_iter = preset_load_data.subfile.clone();
        // `path` names the file in the log lines of the C++ only.
        let _ = path;
        // Load the print, filament or printer preset.
        let mut alias_name = String::new();
        let inherits: String;
        let mut description = String::new();
        let instantiation: String;
        let mut setting_id = String::new();
        let mut filament_id = String::new();
        let mut renamed_from: Vec<String> = Vec::new();
        let mut includes: Vec<String> = Vec::new();
        let default_config: DynamicPrintConfig;
        let mut reason: String;

        let key_values = &mut preset_load_data.key_values;
        substitution_context.substitutions.clear();

        // parse the json elements
        reason = preset_load_data.reason.clone();
        substitution_context.substitutions = std::mem::take(&mut preset_load_data.context.substitutions);
        for item in preset_load_data.context.unrecogized_keys.iter() {
            if !substitution_context.unrecogized_keys.contains(item) {
                // emplace back
                substitution_context.unrecogized_keys.push(item.clone());
            }
        }
        if !reason.is_empty() {
            // load config file Failed
            return Ok(reason);
        }
        let preset_name = key_values.entry(BBL_JSON_KEY_NAME.to_string()).or_default().clone();
        instantiation = key_values.entry(crate::preset::BBL_JSON_KEY_INSTANTIATION.to_string()).or_default().clone();
        if let Some(s) = key_values.get(BBL_JSON_KEY_SETTING_ID) {
            setting_id = s.clone();
        }
        if let Some(f) = key_values.get(BBL_JSON_KEY_FILAMENT_ID) {
            filament_id = f.clone();
        }
        let presets_collection: &PresetCollection = match collection_type {
            Type::TypePrint => &self.prints,
            Type::TypeFilament => &self.filaments,
            _ => &self.printers.base,
        };
        //check whether it inherits other preset or not
        if let Some(it1) = key_values.get(BBL_JSON_KEY_INHERITS) {
            inherits = it1.clone();
            match config_maps.get(&inherits) {
                Some(c) => {
                    default_config = c.clone();
                    if filament_id.is_empty() && presets_collection.type_() == Type::TypeFilament {
                        if let Some(f) = filament_id_maps.get(&inherits) {
                            filament_id = f.clone();
                        }
                    }
                }
                None => {
                    // can not find inherits
                    reason = format!("Can not find inherits: {}", inherits);
                    return Ok(reason);
                }
            }
            if let Some(ds) = description_maps.get(&inherits) {
                description = ds.clone();
            }
        } else {
            inherits = String::new();
            default_config = if presets_collection.type_() == Type::TypePrinter {
                presets_collection.default_preset_for(&preset_load_data.config_src).config.clone()
            } else {
                presets_collection.default_preset(0).config.clone()
            };
        }
        let _ = &inherits;
        let mut config = default_config.clone();

        if let Some(include) = key_values.get(crate::preset::BBL_JSON_KEY_INCLUDES) {
            if !include.is_empty() {
                let include = include.clone();
                let b = include.as_bytes();
                if b[0] == b'"' && b[b.len() - 1] == b'"' {
                    // include.substr(1, include.length() - 2)
                    includes.push(if include.len() >= 2 { include[1..include.len() - 1].to_string() } else { String::new() });
                } else if b[0] == b'[' && b[b.len() - 1] == b']' {
                    if let Ok(serde_json::Value::Array(items)) = crate::config::nlohmann_json::parse(include.as_bytes()) {
                        for include_item in items {
                            if let serde_json::Value::String(s) = include_item {
                                includes.push(s);
                            }
                        }
                    }
                } else {
                    // invalid includes format (logged only)
                }
            }
        }
        for name in includes.iter() {
            if let Some(include_config) = config_maps.get(name) {
                let include_default_config = if presets_collection.type_() == Type::TypePrinter {
                    &presets_collection.default_preset_for(include_config).config
                } else {
                    &presets_collection.default_preset(0).config
                };
                let keys = include_config.diff(include_default_config)?;

                config.apply_only(include_config, &keys, false)?;
            } else {
                // can not find include (logged only)
            }
        }

        if let Some(ds) = key_values.get(crate::preset::BBL_JSON_KEY_DESCRIPTION) {
            description = ds.clone();
        }

        if !description.is_empty() {
            description_maps.entry(preset_name.clone()).or_insert_with(|| description.clone());
        }

        config.apply(&preset_load_data.config_src, false)?;
        if instantiation == "false" && vendor_name != "Template" {
            if preset_name.is_empty() || preset_name.contains("gcode") {
                //pure included gcode file
                config_maps.entry(subfile_iter.0.clone()).or_insert_with(|| std::mem::take(&mut preset_load_data.config_src));
                return Ok(reason);
            } else {
                config_maps.entry(preset_name.clone()).or_insert(config);
            }
            if presets_collection.type_() == Type::TypeFilament && !filament_id.is_empty() {
                filament_id_maps.entry(preset_name.clone()).or_insert(filament_id);
            }
            return Ok(reason);
        } else if instantiation.is_empty() && (preset_name.is_empty() || preset_name.contains("gcode")) {
            //pure included config
            config_maps.entry(subfile_iter.0.clone()).or_insert_with(|| std::mem::take(&mut preset_load_data.config_src));
            return Ok(reason);
        }
        if config.has("alias") {
            alias_name = config.option("alias").and_then(|o| o.as_any().downcast_ref::<ConfigOptionString>()).expect("ConfigOptionString").value.clone();
        }
        if config.has("renamed_from") {
            renamed_from = config.option("renamed_from").and_then(|o| o.as_vector_base()).expect("ConfigOptionVectorBase").vserialize()?;
        }
        Preset::normalize(&mut config)?;

        // Report configuration fields, which are misplaced into a wrong group.
        let _incorrect_keys = Preset::remove_invalid_keys(&mut config, &default_config);

        if presets_collection.type_() == Type::TypePrinter {
            // Filter out printer presets, which are not mentioned in the vendor profile.
            // These presets are considered not installed.
            let printer_model = opt_string(&config, "printer_model").clone();
            if printer_model.is_empty() {
                // The printer preset defines no printer model, it will be ignored.
                return Ok("can not find printer_model".to_string());
            }
            let printer_variant = opt_string(&config, "printer_variant").clone();
            if printer_variant.is_empty() {
                // The printer preset defines no printer variant, it will be ignored.
                return Ok("can not find printer_variant".to_string());
            }
            let it_model = match current_vendor_profile.models.iter().find(|m| m.id == printer_model) {
                None => {
                    // The printer preset defines invalid printer model, it will be ignored.
                    return Ok("can not find printer model in vendor profile".to_string());
                }
                Some(m) => m,
            };
            if it_model.variant(&printer_variant).is_none() {
                // The printer preset defines invalid printer variant, it will be ignored.
                return Ok("can not find printer_variant in vendor profile".to_string());
            }
        }
        if presets_collection.find_preset(&preset_name, false).is_some() {
            // The printer preset has already been loaded from another Confing Bundle.
            return Ok("duplicated defines".to_string());
        }

        let file_path = boost_make_preferred(boost_path_append(
            &boost_path_append(&boost_path_append(&crate::log_sink::data_dir(), PRESET_SYSTEM_DIR), vendor_name),
            &subfile_iter.1,
        ));
        // Load the preset into the list of presets, save it to disk.
        let presets_collection: &mut PresetCollection = match collection_type {
            Type::TypePrint => &mut self.prints,
            Type::TypeFilament => &mut self.filaments,
            _ => &mut self.printers.base,
        };
        let loaded = presets_collection.load_preset_move(&file_path, &preset_name, config, false, Semver::new())?;
        if flags.has(LoadConfigBundleAttribute::LoadSystem) {
            loaded.is_system = true;
            loaded.vendor = Some(current_vendor_profile.clone());
            loaded.version = current_vendor_profile.config_version.clone();
            loaded.description = description.clone();
            loaded.setting_id = setting_id.clone();
            loaded.filament_id = filament_id.clone();
            if collection_type == Type::TypeFilament {
                if filament_id.is_empty() && vendor_name != "Template" {
                    // can not find filament_id
                    return Ok(format!("Can not find filament_id for {}", preset_name));
                } else {
                    filament_id_maps.entry(preset_name.clone()).or_insert(filament_id.clone());
                }
            }
        }

        // Derive the profile logical name aka alias from the preset name if the alias was not stated explicitely.
        if alias_name.is_empty() {
            if let Some(end_pos) = preset_name.find('@') {
                alias_name = preset_name[..end_pos].to_string();
                if renamed_from.is_empty() {
                    // Add the preset name with the '@' character removed into the "renamed_from" list.
                    renamed_from.push(format!("{}{}", alias_name, &preset_name[end_pos + 1..]));
                }
                alias_name = trim_right(&alias_name);
            }
        }
        let hold_alias: Option<(String, Preset)>;
        if alias_name.is_empty() {
            loaded.alias = preset_name.clone();
            hold_alias = None;
        } else {
            loaded.alias = alias_name;
            hold_alias = Some((loaded.alias.clone(), loaded.clone()));
        }
        loaded.renamed_from = renamed_from;
        let loaded_config = loaded.config.clone();
        if let Some((alias, preset)) = hold_alias {
            self.filaments.set_printer_hold_alias(&alias, &preset, false);
        }
        if !substitution_context.empty() {
            substitutions.push(PresetConfigSubstitutions {
                preset_name: preset_name.clone(),
                preset_type: collection_type,
                preset_file: String::new(),
                preset_source: PresetConfigSubstitutionsSource::ConfigBundle,
                substitutions: std::mem::take(&mut substitution_context.substitutions),
            });
        }
        config_maps.entry(preset_name).or_insert(loaded_config);
        *count += 1;
        Ok(reason)
    }

    /// PresetBundle.cpp:6992-7102 `VendorProfile::PrinterModel PresetBundle::load_vendor_configs_from_json(const std::string &path)`
    pub fn load_vendor_configs_from_json_2(&self, path: &str) -> CfgResult<PrinterModel> {
        let mut model = PrinterModel::default();
        if !fs_exists(path) {
            return Err(ConfigurationError::new(format!("Failed loading configuration file, please check {} firstly", path)));
        }
        let j = match read_json_file(path) {
            Err(Ok(what)) => {
                return Err(ConfigurationError::new(format!(
                    "Failed loading configuration file  {}\nSuggest closing studio, deleting all files in {}, and restarting studio",
                    what, path
                )));
            }
            Err(Err(what)) => return Err(runtime_error(what)),
            Ok(j) => j,
        };
        use crate::preset::*;
        // parse the json elements
        for (key, value) in json_items(&j) {
            if iequals(&key, BBL_JSON_KEY_VERSION) {
                // get version
            } else if iequals(&key, BBL_JSON_KEY_URL) {
                // get url
            } else if iequals(&key, BBL_JSON_KEY_NAME) {
                // get name
                model.name = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_MODEL_ID) {
                // get model_id
                model.model_id = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_NOZZLE_DIAMETER) {
                // get nozzle diameter
                let nozzle_diameters = json_string(&value)?;
                let mut variants: Vec<String> = Vec::new();
                if unescape_strings_cstyle(&nozzle_diameters, &mut variants) {
                    for variant_name in variants.iter() {
                        if model.variant(variant_name).is_none() {
                            model.variants.push(PrinterVariant::with_name(variant_name));
                        }
                    }
                }
            } else if iequals(&key, BBL_JSON_KEY_PRINTER_TECH) {
                // get printer tech
                if json_range_starts_with(&value, "SL") {
                    model.technology = PrinterTechnology::PtSLA;
                } else {
                    model.technology = PrinterTechnology::PtFFF;
                }
            } else if iequals(&key, BBL_JSON_KEY_FAMILY) {
                // get family
                model.family = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_BED_MODEL) {
                // get bed model
                model.bed_model = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_DEFAULT_BED_TYPE) {
                // get bed type
                model.default_bed_type = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_RIGHT_ICON_OFFSET_BED) {
                model.right_icon_offset_bed = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_BOTTOM_TEXTURE_END_NAME) {
                model.bottom_texture_end_name = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_USE_DOUBLE_EXTRUDER_DEFAULT_TEXTURE) {
                model.use_double_extruder_default_texture = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_BOTTOM_TEXTURE_RECT) {
                model.bottom_texture_rect = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_BOTTOM_TEXTURE_RECT_LONGER) {
                model.bottom_texture_rect_longer = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_BOTTOM_TEXTURE_RECT_LONGER_IGNORE_LIST) {
                let longer_ignore_field = json_string(&value)?;
                if unescape_strings_cstyle(&longer_ignore_field, &mut model.bottom_texture_rect_longer_ignore_list) {
                    sort_remove_duplicates(&mut model.bottom_texture_rect_longer_ignore_list);
                    if !model.bottom_texture_rect_longer_ignore_list.is_empty() && model.bottom_texture_rect_longer_ignore_list[0].is_empty() {
                        model.bottom_texture_rect_longer_ignore_list.remove(0);
                    }
                } else {
                    // invalid bottom_texture_rect_longer_ignore_list (logged only)
                }
            } else if iequals(&key, BBL_JSON_KEY_MIDDLE_TEXTURE_RECT) {
                model.middle_texture_rect = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_USE_RECT_GRID) {
                model.use_rect_grid = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_IMAGE_BED_TYPE) {
                model.image_bed_type = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_BED_TEXTURE) {
                // get bed texture
                model.bed_texture = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_HOTEND_MODEL) {
                model.hotend_model = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_DEFAULT_MATERIALS) {
                // get machine list
                let default_materials_field = json_string(&value)?;
                if unescape_strings_cstyle(&default_materials_field, &mut model.default_materials) {
                    sort_remove_duplicates(&mut model.default_materials);
                    if !model.default_materials.is_empty() && model.default_materials[0].is_empty() {
                        // An empty material was inserted into the list of default materials. Remove it.
                        model.default_materials.remove(0);
                    }
                }
            } else if iequals(&key, BBL_JSON_KEY_NOT_SUPPORT_BED_TYPE) {
                // get machine list
                let not_support_bed_type_field = json_string(&value)?;
                if unescape_strings_cstyle(&not_support_bed_type_field, &mut model.not_support_bed_types) {
                    sort_remove_duplicates(&mut model.not_support_bed_types);
                    if !model.not_support_bed_types.is_empty() && model.not_support_bed_types[0].is_empty() {
                        // An empty material was inserted into the list of default materials. Remove it.
                        model.not_support_bed_types.remove(0);
                    }
                } else {
                    // invalid not_support_bed_types (logged only)
                }
            } else if iequals(&key, BBL_JSON_KEY_SUPPORT_SIDE_PANEL_FAN) {
                model.support_side_panel_fan = json_string(&value)?;
            } else if iequals(&key, BBL_JSON_KEY_RESOURCE_BIND_NAME) {
                model.resource_bind_name = json_string(&value)?;
            }
        }
        Ok(model)
    }

    /// PresetBundle.cpp:7104-7113 `void PresetBundle::on_extruders_count_changed(int extruders_count, bool reset_volume_type)`
    pub fn on_extruders_count_changed(&mut self, extruders_count: i32, reset_volume_type: bool) -> CfgResult<()> {
        self.printers.get_edited_preset_mut().set_num_extruders(extruders_count as u32)?;
        self.update_multi_material_filament_presets(usize::MAX)?;
        // A project load selects the file's nozzles itself, after its printer is in place.
        if reset_volume_type {
            self.nozzle_select_printer()?;
        }
        self.extruder_ams_counts.resize(extruders_count.max(0) as usize, BTreeMap::new());
        self.extruder_ams_selections.clear();
        Ok(())
    }
}

impl PresetBundle {
    /// PresetBundle.cpp:7115-7179 `void PresetBundle::update_multi_material_filament_presets(size_t to_delete_filament_id)`
    ///
    /// `to_delete_filament_id` of `usize::MAX` is the default `size_t(-1)`.
    pub fn update_multi_material_filament_presets(&mut self, mut to_delete_filament_id: usize) -> CfgResult<()> {
        if self.printers.get_edited_preset().printer_technology() != PrinterTechnology::PtFFF {
            return Ok(());
        }

        // BBS
        let num_filaments = self.filament_presets.len();
        if to_delete_filament_id == usize::MAX {
            to_delete_filament_id = num_filaments;
        }

        // Now verify if flush_volumes_matrix has proper size (it is used to deduce number of extruders in wipe tower generator):
        let old_matrix: Vec<f64> = floats_ref(&self.project_config, "flush_volumes_matrix").expect("flush_volumes_matrix").clone();
        let old_nozzle_nums = floats_ref(&self.project_config, "flush_multiplier").expect("flush_multiplier").len();
        let old_number_of_filaments = (((old_matrix.len() / old_nozzle_nums) as f64).sqrt() + crate::libslic3r::EPSILON) as usize;
        let nozzle_nums = self.get_printer_extruder_count() as usize;
        if old_nozzle_nums != nozzle_nums {
            floats_mut(&mut self.project_config, "flush_multiplier", false)?.expect("flush_multiplier").resize(nozzle_nums, 1.0f32 as f64);
            floats_mut(&mut self.project_config, "flush_multiplier_fast", false)?.expect("flush_multiplier_fast").resize(nozzle_nums, 1.2f32 as f64);
        }

        if num_filaments * num_filaments != old_matrix.len() / old_nozzle_nums {
            // First verify if purging volumes presets for each extruder matches number of extruders
            let filaments = floats_mut(&mut self.project_config, "flush_volumes_vector", false)?.expect("flush_volumes_vector");
            while filaments.len() < 2 * num_filaments {
                let a = if filaments.len() > 1 { filaments[0] } else { 140. }; // copy the values from the first extruder
                filaments.push(a);
                let b = if filaments.len() > 1 { filaments[1] } else { 140. };
                filaments.push(b);
            }
            while filaments.len() > 2 * num_filaments {
                filaments.pop();
                filaments.pop();
            }
            let filaments = filaments.clone();

            let old_matrix_size = old_number_of_filaments * old_number_of_filaments;
            let new_matrix_size = num_filaments * num_filaments;
            let mut new_matrix: Vec<f64> = vec![0.0; new_matrix_size * nozzle_nums];
            for i in 0..num_filaments {
                for j in 0..num_filaments {
                    if i < old_number_of_filaments && j < old_number_of_filaments {
                        let old_i = if i >= to_delete_filament_id { i + 1 } else { i };
                        let old_j = if j >= to_delete_filament_id { j + 1 } else { j };
                        for nozzle_id in 0..nozzle_nums {
                            new_matrix[i * num_filaments + j + new_matrix_size * nozzle_id] =
                                old_matrix[old_i * old_number_of_filaments + old_j + old_matrix_size * nozzle_id];
                        }
                    } else {
                        for nozzle_id in 0..nozzle_nums {
                            new_matrix[i * num_filaments + j + new_matrix_size * nozzle_id] = if i == j { 0. } else { filaments[2 * i] + filaments[2 * j + 1] };
                        }
                    }
                }
            }
            *floats_mut(&mut self.project_config, "flush_volumes_matrix", false)?.expect("flush_volumes_matrix") = new_matrix;
        }
        Ok(())
    }
}

/// PresetBundle.cpp:7183-7207 `class PreferedProfileMatch`: ranks presets by preference when
/// picking a replacement for the current selection.
#[derive(Clone)]
struct PreferedProfileMatch {
    m_prefered_alias: String,
    m_prefered_name: String,
}

impl PreferedProfileMatch {
    fn new(prefered_alias: &str, prefered_name: &str) -> Self {
        PreferedProfileMatch { m_prefered_alias: prefered_alias.to_string(), m_prefered_name: prefered_name.to_string() }
    }

    /// PresetBundle.cpp:7191-7202 `int operator()(const Preset &preset) const`
    fn call(&self, preset: &Preset) -> i32 {
        if preset.is_default || preset.is_external {
            // Don't match any properties of the "-- default --" profile or the external profiles when switching printer profile.
            0
        } else if !self.m_prefered_alias.is_empty() && self.m_prefered_alias == preset.alias {
            // Matching an alias, always take this preset with priority.
            i32::MAX
        } else {
            // Otherwise take the prefered profile, or the first compatible.
            (preset.name == self.m_prefered_name) as i32
        }
    }
}

/// PresetBundle.cpp:7209-7234 `class PreferedPrintProfileMatch : public PreferedProfileMatch`:
/// matching by the layer height in addition.
struct PreferedPrintProfileMatch {
    base: PreferedProfileMatch,
    m_prefered_layer_height: f64,
}

impl PreferedPrintProfileMatch {
    fn new(preset: Option<&Preset>, prefered_name: &str) -> Self {
        PreferedPrintProfileMatch {
            base: PreferedProfileMatch::new(preset.map_or("", |p| p.alias.as_str()), prefered_name),
            m_prefered_layer_height: preset.map_or(0.0, |p| *p.config.opt_float("layer_height")),
        }
    }

    /// PresetBundle.cpp:7216-7230
    fn call(&self, preset: &Preset) -> i32 {
        // Don't match any properties of the "-- default --" profile or the external profiles when switching printer profile.
        if preset.is_default || preset.is_external {
            return 0;
        }
        let mut match_quality = self.base.call(preset);
        if match_quality < i32::MAX {
            match_quality += 1;
            if preset.is_visible {
                match_quality += 1;
            }
            if self.m_prefered_layer_height > 0. && (*preset.config.opt_float("layer_height") - self.m_prefered_layer_height).abs() < 0.0005 {
                match_quality *= 10;
            }
        }
        match_quality
    }
}

/// PresetBundle.cpp:7236-7262 `class PreferedFilamentProfileMatch : public PreferedProfileMatch`:
/// matching by the filament type in addition.
struct PreferedFilamentProfileMatch {
    base: PreferedProfileMatch,
    m_prefered_filament_type: String,
}

impl PreferedFilamentProfileMatch {
    fn new(preset: Option<&Preset>, prefered_name: &str) -> Self {
        PreferedFilamentProfileMatch {
            base: PreferedProfileMatch::new(preset.map_or("", |p| p.alias.as_str()), prefered_name),
            m_prefered_filament_type: preset.map_or(String::new(), |p| opt_string_at(&p.config, "filament_type", 0)),
        }
    }

    /// PresetBundle.cpp:7244-7258
    fn call(&self, preset: &Preset) -> i32 {
        // Don't match any properties of the "-- default --" profile or the external profiles when switching printer profile.
        if preset.is_default || preset.is_external {
            return 0;
        }
        let mut match_quality = self.base.call(preset);
        if match_quality < i32::MAX {
            match_quality += 1;
            if preset.is_visible {
                match_quality += 1;
            }
            if !self.m_prefered_filament_type.is_empty() && self.m_prefered_filament_type == opt_string_at(&preset.config, "filament_type", 0) {
                match_quality *= 10;
            }
        }
        match_quality
    }
}

/// PresetBundle.cpp:7346-7373 `class PreferedFilamentsProfileMatch` (local to update_compatible).
struct PreferedFilamentsProfileMatch {
    m_prefered_alias: String,
    m_prefered_filament_type: String,
    m_prefered_names: Vec<String>,
}

impl PreferedFilamentsProfileMatch {
    fn new(preset: Option<&Preset>, prefered_names: &[String]) -> Self {
        PreferedFilamentsProfileMatch {
            m_prefered_alias: preset.map_or(String::new(), |p| p.alias.clone()),
            // BBS: default choose PLA
            m_prefered_filament_type: preset.map_or("PLA".to_string(), |p| opt_string_at(&p.config, "filament_type", 0)),
            m_prefered_names: prefered_names.to_vec(),
        }
    }

    /// PresetBundle.cpp:7355-7367
    fn call(&self, preset: &Preset) -> i32 {
        // Don't match any properties of the "-- default --" profile or the external profiles when switching printer profile.
        if preset.is_default || preset.is_external {
            return 0;
        }
        if !self.m_prefered_alias.is_empty() && self.m_prefered_alias == preset.alias {
            // Matching an alias, always take this preset with priority.
            return i32::MAX;
        }
        let mut match_quality = self.m_prefered_names.contains(&preset.name) as i32 + 1;
        if !self.m_prefered_filament_type.is_empty() && self.m_prefered_filament_type == opt_string_at(&preset.config, "filament_type", 0) {
            match_quality *= 10;
        }
        match_quality
    }
}

/// PresetBundle.cpp:7264-7294 `bool replace_default_filament_slots(PresetBundle &bundle, const std::vector<std::string> &prefered_names)`:
/// replaces each built-in default filament slot with the printer's preferred filament for that
/// slot. Returns true if any slot changed.
fn replace_default_filament_slots(bundle: &mut PresetBundle, prefered_names: &[String]) -> CfgResult<bool> {
    let mut changed = false;
    for idx in 0..bundle.filament_presets.len() {
        match bundle.filaments.find_preset(&bundle.filament_presets[idx], false) {
            Some(p) if p.is_default => {}
            _ => continue,
        }

        let prefered_name = if idx < prefered_names.len() { &prefered_names[idx] } else { &prefered_names[0] };
        let matcher = PreferedFilamentProfileMatch::new(None, prefered_name);
        let replacement = bundle.filaments.preset(bundle.filaments.first_compatible_idx_with(|p| matcher.call(p)));
        if replacement.is_default {
            continue;
        }

        bundle.filament_presets[idx] = replacement.name.clone();
        changed = true;
        settle_filament_slot_flow(bundle, idx)?;
    }
    Ok(changed)
}

/// PresetBundle.cpp:7296-7312 `void log_default_filament_slots(const PresetCollection &filaments, const std::vector<std::string> &previous_slots, const std::vector<std::string> &slots, const std::string &printer_name)`:
/// traces filament slots that hold the built-in default preset after a compatibility update, with
/// the reason.
fn log_default_filament_slots(filaments: &PresetCollection, previous_slots: &[String], slots: &[String], printer_name: &str) {
    for idx in 0..slots.len() {
        match filaments.find_preset(&slots[idx], false) {
            Some(p) if p.is_default => {}
            _ => continue,
        }

        let previous = if idx < previous_slots.len() { previous_slots[idx].clone() } else { String::new() };
        let reason = if previous == slots[idx] {
            "kept, the default preset is always compatible so it is never replaced".to_string()
        } else {
            format!("picked to replace \"{}\", no compatible filament preset exists", previous)
        };
        crate::boost_log_trivial!(
            Warning,
            "[default-preset] filament slot ",
            idx,
            " holds the built-in default preset \"",
            slots[idx],
            "\" for printer \"",
            printer_name,
            "\": ",
            reason
        );
    }
}

impl PresetBundle {
    /// PresetBundle.cpp:7316-7338 `bool PresetBundle::replace_default_presets()`: replace the
    /// built-in default process and filament presets with the printer's preferred ones.
    pub fn replace_default_presets(&mut self) -> CfgResult<bool> {
        let mut changed = false;

        if self.prints.get_edited_preset().is_default && self.printers.get_edited_preset().config.has("default_print_profile") {
            let matcher = PreferedPrintProfileMatch::new(None, opt_string(&self.printers.get_edited_preset().config, "default_print_profile"));
            let idx = self.prints.first_compatible_idx_with(|p| matcher.call(p));
            self.prints.select_preset(idx);
            // Refresh the filament compatibility against the new process before picking filaments.
            self.update_compatible(PresetSelectCompatibleType::Never)?;
            changed = !self.prints.get_edited_preset().is_default;
        }

        let prefered_filaments: Vec<String> = match strings_ref(&self.printers.get_edited_preset().config, "default_filament_profile") {
            Some(v) if !v.is_empty() => v.clone(),
            _ => return Ok(changed),
        };

        if replace_default_filament_slots(self, &prefered_filaments)? {
            if self.filaments.get_edited_preset().is_default {
                let front = self.filament_presets[0].clone();
                self.filaments.select_preset_by_name(&front, true, false);
            }
            changed = true;
        }
        Ok(changed)
    }

    /// PresetBundle.hpp:397 `void update_compatible(PresetSelectCompatibleType select_other_if_incompatible)`
    pub fn update_compatible(&mut self, select_other_if_incompatible: PresetSelectCompatibleType) -> CfgResult<()> {
        self.update_compatible_2(select_other_if_incompatible, select_other_if_incompatible)
    }

    /// PresetBundle.cpp:7340-7452 `void PresetBundle::update_compatible(PresetSelectCompatibleType select_other_print_if_incompatible, PresetSelectCompatibleType select_other_filament_if_incompatible)`
    pub fn update_compatible_2(
        &mut self,
        select_other_print_if_incompatible: PresetSelectCompatibleType,
        select_other_filament_if_incompatible: PresetSelectCompatibleType,
    ) -> CfgResult<()> {
        match self.printers.get_edited_preset().printer_technology() {
            PrinterTechnology::PtFFF => {
                let prefered_filament_profiles: Vec<String> =
                    strings_ref(&self.printers.get_edited_preset().config, "default_filament_profile").expect("default_filament_profile").clone();
                {
                    let print_matcher = PreferedPrintProfileMatch::new(
                        if self.prints.get_selected_idx() == usize::MAX { None } else { Some(self.prints.get_edited_preset()) },
                        opt_string(&self.printers.get_edited_preset().config, "default_print_profile"),
                    );
                    let printer_preset_with_vendor_profile = self.printers.get_preset_with_vendor_profile(self.printers.get_edited_preset());
                    self.prints.update_compatible_with(&printer_preset_with_vendor_profile, None, select_other_print_if_incompatible, |p| print_matcher.call(p));
                }
                // Remember whether the filament profiles were compatible before updating the filament compatibility.
                let mut filament_preset_was_compatible: Vec<bool> = vec![false; self.filament_presets.len()];
                let previous_filament_presets: Vec<String> = self.filament_presets.clone();
                for idx in 0..self.filament_presets.len() {
                    let preset = self.filaments.find_preset(&self.filament_presets[idx], false);
                    filament_preset_was_compatible[idx] = preset.map_or(false, |p| p.is_compatible);
                }
                // First select a first compatible profile for the preset editor.
                // BBS multi-diameter: also treat filament presets bound to sibling nozzle-diameter variants
                // (selected via "set nozzle diameter") as compatible, so the user can keep using them while
                // running mixed-diameter printing. Reset afterwards to avoid leaking state into later passes.
                let extra = self.get_extra_compatible_printers_by_full_stat();
                self.filaments.set_extra_compatible_printers(&extra);
                // When fallback substitution is blocked, also stop this call from silently reselecting the
                // filaments collection's own edited/selected preset to some other compatible default.
                let filaments_select_other_if_incompatible =
                    if self.block_filament_fallback_substitution { PresetSelectCompatibleType::Never } else { select_other_filament_if_incompatible };
                {
                    let filaments_matcher = PreferedFilamentsProfileMatch::new(
                        if self.filaments.get_selected_idx() == usize::MAX { None } else { Some(self.filaments.get_edited_preset()) },
                        &prefered_filament_profiles,
                    );
                    let printer_preset_with_vendor_profile = self.printers.get_preset_with_vendor_profile(self.printers.get_edited_preset());
                    let print_preset_with_vendor_profile = self.prints.get_edited_preset_with_vendor_profile();
                    self.filaments.update_compatible_with(
                        &printer_preset_with_vendor_profile,
                        Some(&print_preset_with_vendor_profile),
                        filaments_select_other_if_incompatible,
                        |p| filaments_matcher.call(p),
                    );
                }
                self.filaments.clear_extra_compatible_printers();
                if select_other_filament_if_incompatible != PresetSelectCompatibleType::Never && !self.block_filament_fallback_substitution {
                    // Verify validity of the current filament presets.
                    let prefered_filament_profile = prefered_filament_profiles.first().cloned().unwrap_or_default();
                    if self.filament_presets.len() == 1 {
                        // The compatible profile should have been already selected for the preset editor. Just use it.
                        if select_other_filament_if_incompatible == PresetSelectCompatibleType::Always || filament_preset_was_compatible[0] {
                            self.filament_presets[0] = self.filaments.get_edited_preset().name.clone();
                        }
                    } else {
                        for idx in 0..self.filament_presets.len() {
                            let preset = self.filaments.find_preset(&self.filament_presets[idx], false);
                            if preset.map_or(true, |p| {
                                !p.is_compatible
                                    && (select_other_filament_if_incompatible == PresetSelectCompatibleType::Always || filament_preset_was_compatible[idx])
                            }) {
                                // Pick a compatible profile. If there are prefered_filament_profiles, use them.
                                let matcher = PreferedFilamentProfileMatch::new(
                                    preset,
                                    if idx < prefered_filament_profiles.len() { &prefered_filament_profiles[idx] } else { &prefered_filament_profile },
                                );
                                let name = self.filaments.preset(self.filaments.first_compatible_idx_with(|p| matcher.call(p))).name.clone();
                                self.filament_presets[idx] = name;
                            }
                        }
                    }
                }
                log_default_filament_slots(&self.filaments, &previous_filament_presets, &self.filament_presets, &self.printers.get_edited_preset().name);
            }
            PrinterTechnology::PtSLA => {
                {
                    let print_matcher = PreferedPrintProfileMatch::new(
                        if self.sla_prints.get_selected_idx() == usize::MAX { None } else { Some(self.sla_prints.get_edited_preset()) },
                        opt_string(&self.printers.get_edited_preset().config, "default_sla_print_profile"),
                    );
                    let printer_preset_with_vendor_profile = self.printers.get_preset_with_vendor_profile(self.printers.get_edited_preset());
                    self.sla_prints.update_compatible_with(&printer_preset_with_vendor_profile, None, select_other_print_if_incompatible, |p| print_matcher.call(p));
                }
                let material_matcher = PreferedProfileMatch::new(
                    &(if self.sla_materials.get_selected_idx() == usize::MAX { String::new() } else { self.sla_materials.get_edited_preset().alias.clone() }),
                    opt_string(&self.printers.get_edited_preset().config, "default_sla_material_profile"),
                );
                let printer_preset_with_vendor_profile = self.printers.get_preset_with_vendor_profile(self.printers.get_edited_preset());
                let sla_print_preset_with_vendor_profile = self.sla_prints.get_edited_preset_with_vendor_profile();
                self.sla_materials.update_compatible_with(
                    &printer_preset_with_vendor_profile,
                    Some(&sla_print_preset_with_vendor_profile),
                    select_other_filament_if_incompatible,
                    |p| material_matcher.call(p),
                );
            }
            _ => {}
        }
        Ok(())
    }

    /// PresetBundle.cpp:7628-7657 `std::vector<std::string> PresetBundle::export_current_configs(const std::string &path, std::function<int(std::string const &)> override_confirm, bool include_modify, bool export_system_settings)`
    ///
    /// FIDELITY-NOTE: the C++ collects the presets in a `std::set<Preset const *>` and writes them in
    /// the order of their addresses. The edited presets are members of the bundle, in the member order
    /// prints, filaments, printers; the other presets live in heap blocks. Here the presets of the
    /// collections come first (in collection order, then index order), then the edited presets in member
    /// order: the order of a bundle that lives above its heap blocks (the stack on macOS arm64).
    pub fn export_current_configs(
        &self,
        path: &str,
        override_confirm: &mut dyn FnMut(&str) -> i32,
        include_modify: bool,
        export_system_settings: bool,
    ) -> CfgResult<Vec<String>> {
        // (heap first?, collection rank, index): see the note above.
        let mut presets: BTreeMap<(u8, u8, usize), &Preset> = BTreeMap::new();
        let key_of = |collection: &PresetCollection, rank: u8, p: &Preset| -> (u8, u8, usize) {
            if std::ptr::eq(p, collection.get_edited_preset()) {
                (1, rank, 0)
            } else {
                (0, rank, collection.m_presets.iter().position(|q| std::ptr::eq(q, p)).unwrap_or(usize::MAX))
            }
        };
        let print_preset = if include_modify { self.prints.get_edited_preset() } else { self.prints.get_selected_preset() };
        let printer_preset = if include_modify { self.printers.get_edited_preset() } else { self.printers.get_selected_preset() };
        presets.insert(key_of(&self.prints, 0, print_preset), print_preset);
        presets.insert(key_of(&self.printers, 4, printer_preset), printer_preset);
        for f in self.filament_presets.iter() {
            if let Some(filament_preset) = self.filaments.find_preset(f, include_modify) {
                presets.insert(key_of(&self.filaments, 2, filament_preset), filament_preset);
            }
        }

        let mut overwrite = 0;
        let mut result: Vec<String> = Vec::new();
        for (_k, preset) in presets.iter() {
            if (preset.is_system && !export_system_settings) || preset.is_default {
                continue;
            }
            let file = format!("{}/{}.json", path, preset.name);
            if overwrite == 0 {
                overwrite = 1;
            }
            if fs_exists(&file) && overwrite < 2 {
                overwrite = override_confirm(&preset.name);
            }
            if overwrite == 0 || overwrite == 2 {
                continue;
            }
            preset.config.save_to_json(&file, &preset.name, "", &preset.version.to_string())?;
            result.push(file);
        }
        Ok(result)
    }

    /// PresetBundle.cpp:7659-7684 `void PresetBundle::set_filament_preset(size_t idx, const std::string &name)`:
    /// set the filament preset name. As the name could come from the UI selection box, an optional
    /// "(modified)" suffix will be removed from the filament name.
    pub fn set_filament_preset(&mut self, idx: usize, name: &str) {
        if idx >= self.filament_presets.len() {
            // Warning: set_filament_preset out of range
            return;
        }
        self.filament_presets[idx] = Preset::remove_suffix_modified(name);

        let n = self.filament_presets.len();
        let not_found = self.filaments.find_preset(&self.filament_presets[idx], false).is_none();
        let selected = match strings_mut(&mut self.project_config, "filament_diameter_map", false) {
            Ok(Some(s)) => s,
            _ => return,
        };
        selected.resize(n, String::new());

        if not_found {
            selected[idx].clear();
        }
        // Picking a different material for this slot is not a diameter choice, so the slot's diameter
        // stays exactly as it was (PresetBundle.cpp:7676-7683).
    }

    /// PresetBundle.cpp:7686-7693 `void PresetBundle::set_default_suppressed(bool default_suppressed)`
    pub fn set_default_suppressed(&mut self, default_suppressed: bool) {
        self.prints.set_default_suppressed(default_suppressed);
        self.filaments.set_default_suppressed(default_suppressed);
        self.sla_prints.set_default_suppressed(default_suppressed);
        self.sla_materials.set_default_suppressed(default_suppressed);
        self.printers.set_default_suppressed(default_suppressed);
    }
}

impl PresetBundle {
    /// The printers and vendors of this bundle, as the `Preset` methods that take a `PresetBundle*`
    /// in C++ read them (`preset.rs` `PresetBundleView`): `preset.get_printer_type(Some(&bundle.view()))`.
    pub fn view(&self) -> crate::preset::PresetBundleView<'_> {
        crate::preset::PresetBundleView { printers: &self.printers, vendors: &self.vendors }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bbl_constants() {
        assert_eq!(BBL_BUNDLE, "BBL");
        assert_eq!(BBL_DEFAULT_PRINTER_MODEL, "Bambu Lab X1 Carbon");
        assert_eq!(BBL_DEFAULT_PRINTER_VARIANT, "0.4");
        assert_eq!(BBL_DEFAULT_FILAMENT, "Generic PLA");
        assert_eq!(PresetBundle::BBL_BUNDLE, BBL_BUNDLE);
    }

    #[test]
    fn test_validate_preset_codes() {
        assert_eq!(VALIDATE_PRESETS_SUCCESS, 0);
        assert_eq!(VALIDATE_PRESETS_PRINTER_NOT_FOUND, 1);
        assert_eq!(VALIDATE_PRESETS_FILAMENTS_NOT_FOUND, 2);
        assert_eq!(VALIDATE_PRESETS_MODIFIED_GCODES, 3);
    }

    #[test]
    fn test_project_options_first_last() {
        // PresetBundle.cpp:59-95
        assert_eq!(S_PROJECT_OPTIONS.first().copied(), Some("flush_volumes_vector"));
        assert_eq!(S_PROJECT_OPTIONS.last().copied(), Some("has_filament_switcher"));
        assert_eq!(S_PROJECT_OPTIONS.len(), 34);
    }

    #[test]
    fn test_gcodes_key_set() {
        assert_eq!(GCODES_KEY_SET.len(), 12);
        assert!(GCODES_KEY_SET.contains(&"machine_start_gcode"));
        assert!(GCODES_KEY_SET.contains(&"wrapping_detection_gcode"));
        // the order of the std::set
        assert!(GCODES_KEY_SET.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn test_ams_combo_info_empty() {
        let mut info = AMSComboInfo::default();
        assert!(info.empty());
        info.ams_names.push("AMS-1".to_string());
        assert!(!info.empty());
        info.clear();
        assert!(info.empty());
    }

    #[test]
    fn test_merge_filament_info_empty() {
        let info = MergeFilamentInfo::default();
        assert!(info.is_empty());
    }

    #[test]
    fn test_filament_base_info_compatibility() {
        let mut info = FilamentBaseInfo::default();
        assert_eq!(info.nozzle_temp_range_low, 220);
        assert_eq!(info.temperature_vitrification, i32::MAX);
        assert_eq!(info.filament_printable, 3);
        info.set_filament_extruder_compatibility(0b110_011);
        assert_eq!(info.get_extruder_compatibility(0), 0b011);
        assert_eq!(info.get_extruder_compatibility(1), 0b110);
        assert_eq!(info.get_extruder_compatibility(-1), 0);
        assert_eq!(info.get_extruder_compatibility(10), 0);
    }

    #[test]
    fn test_extruder_full_stat() {
        let mut stat = ExtruderFullStat::default();
        assert!(stat.empty());
        stat.set_extruder_stats(1, &[("0.4".to_string(), "Standard".to_string())]);
        assert_eq!(stat.get_extruder_count(), 2);
        assert_eq!(stat.get_extruder_nozzle_count(0), 0);
        assert_eq!(stat.get_extruder_nozzle_count(1), 1);
        assert_eq!(stat.get_extruder_nozzle_count(5), 0);
        stat.clear_extruder(1);
        assert_eq!(stat.get_extruder_nozzle_count(1), 0);
        stat.clear();
        assert!(stat.empty());
    }

    #[test]
    fn test_json_range_starts_with() {
        // boost::algorithm::starts_with over a json value: a string is one element.
        assert!(!json_range_starts_with(&serde_json::json!("SLA"), "SL"));
        assert!(!json_range_starts_with(&serde_json::json!(null), "SL"));
        assert!(json_range_starts_with(&serde_json::json!([83, 76, 65]), "SL"));
        assert!(!json_range_starts_with(&serde_json::json!([83]), "SL"));
    }

    #[test]
    fn test_gradient_curve_is_empty() {
        assert!(gradient_curve_is_empty(""));
        assert!(gradient_curve_is_empty("0,0.2"));
        assert!(!gradient_curve_is_empty("0,0.2|1,0.8"));
        assert!(!gradient_curve_is_empty("0,0.2,,|1,0.8,nan,"));
        assert!(gradient_curve_is_empty("0,0.2|x,0.8"));
        assert!(gradient_curve_is_empty("0,0.2,1|1,0.8,1"));
        let mut v = vec!["0,1".to_string(), String::new(), "0,0|1,1".to_string()];
        sanitize_mixed_gradient_curve_array(&mut v);
        assert_eq!(v, vec![String::new(), String::new(), "0,0|1,1".to_string()]);
    }

    #[test]
    fn test_std_stoi_and_lexical_cast() {
        assert_eq!(std_stoi(" 42x").unwrap(), 42);
        assert_eq!(std_stoi("-7").unwrap(), -7);
        assert!(std_stoi("abc").is_err());
        assert!(std_stoi("99999999999").is_err());
        assert_eq!(lexical_cast_double("1.5").unwrap(), 1.5);
        assert!(lexical_cast_double("").is_err());
        assert!(lexical_cast_double(" 1").is_err());
        assert!(lexical_cast_double("1x").is_err());
    }

    #[test]
    fn test_boost_split_and_remove_ini_suffix() {
        assert_eq!(boost_split("", &[',']), vec![String::new()]);
        assert_eq!(boost_split("a,,b", &[',']), vec!["a".to_string(), String::new(), "b".to_string()]);
        assert_eq!(remove_ini_suffix("x.INI"), "x");
        assert_eq!(remove_ini_suffix("x.json"), "x.json");
    }
}
