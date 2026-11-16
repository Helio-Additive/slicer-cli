//! Port of libslic3r/Preset.cpp and Preset.hpp (Bambu Studio at the pin; no override-layer copy).
//!
//! The classes are built on the ported configuration layer (`config.rs`, the `DynamicPrintConfig`
//! of `print_config::config_def`): `Preset` holds its `DynamicPrintConfig`, `PresetCollection` and
//! `PrinterPresetCollection` keep their presets sorted by name, `PhysicalPrinter` and
//! `PhysicalPrinterCollection` follow. Files are read and written with std::fs as the C++ does with
//! boost::filesystem (absolute paths, `/` joins, `replace_extension`); JSON goes through
//! `ConfigBase::load_from_json` / `save_to_json`, INI through a port of boost's `read_ini`.
//!
//! Mappings that hold for the whole file:
//! - `std::deque<Preset>` is a `Vec<Preset>`; an iterator is an index, `end()` the length.
//! - `size_t(-1)` is `usize::MAX`; where the C++ mixes it with signed or iterator arithmetic the
//!   code says how the value behaves.
//! - `const VendorProfile*` is an `Option<Arc<VendorProfile>>` (FIDELITY-NOTE: the C++ points
//!   into PresetBundle::vendors; `VendorMap` here holds `Arc`s and the identity of a vendor is
//!   the identity of its allocation, compared with `std::ptr::eq` as the C++ compares pointers).
//! - `PresetCollection::lock()` / `unlock()` guard the background sync thread of the GUI. Access
//!   to a collection is exclusive through `&mut self` here, so they do nothing (FIDELITY-NOTE).
//! - `BOOST_LOG_TRIVIAL` lines are not reproduced (they write nothing the program reads).
//! - A thrown `std::runtime_error` (ConfigurationError, RuntimeError, ptree errors) is an
//!   `Err(ConfigurationError)`; `catch` blocks match on it.
//! - FIDELITY-NOTE: the const accessors `inherits()`, `compatible_prints_condition()` and
//!   `compatible_printers_condition()` of the C++ `const_cast` the preset and create the option
//!   when it is missing. The `&self` forms here read it without creating it (an empty string when
//!   missing); the `_mut` forms create it, and are used where the C++ calls the accessor on a
//!   non-const preset. Every preset of a collection made with the key lists of this file has the
//!   three options already.

use crate::app_config::AppConfig;
use crate::config::{
    unescape_strings_cstyle, ConfigBase, ConfigBaseTyped, ConfigEnum, ConfigOptionEnum, ConfigOptionEnumGeneric,
    ConfigOptionFloats, ConfigOptionInt, ConfigOptionResolver, ConfigOptionResolverTyped, ConfigOptionString, ConfigOptionStrings,
    ConfigOptionType, ConfigSubstitution, ConfigSubstitutions, ConfigurationError, ConfigurationErrorKind, DynamicConfig,
    ForwardCompatibilitySubstitutionRule, PrinterTechnology, t_config_option_keys,
};
use crate::print_config::config_def::{
    empty_options, filament_dev_options, filament_options_with_variant, handle_legacy_sla, is_filament_extruder_override_key,
    print_config_def, print_options_with_variant, printer_options_with_variant_1, printer_options_with_variant_2,
    static_print_config_defaults, DynamicPrintConfig,
};
use crate::print_config::enums::BedType;
use crate::semver::{Semver, SLIC3R_VERSION};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::sync::{Arc, LazyLock, RwLock};

type CfgResult<T> = Result<T, ConfigurationError>;

/// `throw Slic3r::RuntimeError(msg)`.
fn runtime_error(msg: impl Into<String>) -> ConfigurationError {
    ConfigurationError::with_kind(ConfigurationErrorKind::RuntimeError, msg)
}

// ============================================================================
// Preset.hpp:18-80 constants
// ============================================================================

//BBS: change system directories
pub const PRESET_SYSTEM_DIR: &str = "system"; // Preset.hpp:19
pub const PRESET_USER_DIR: &str = "user"; // Preset.hpp:20
pub const PRESET_FILAMENT_NAME: &str = "filament"; // Preset.hpp:21
pub const PRESET_PRINT_NAME: &str = "process"; // Preset.hpp:22
pub const PRESET_PRINTER_NAME: &str = "machine"; // Preset.hpp:23
pub const PRESET_SLA_PRINT_NAME: &str = "sla_print"; // Preset.hpp:24
pub const PRESET_SLA_MATERIALS_NAME: &str = "sla_materials"; // Preset.hpp:25
pub const PRESET_PROFILES_DIR: &str = "profiles"; // Preset.hpp:26
pub const PRESET_PROFILES_TEMOLATE_DIR: &str = "profiles_template"; // Preset.hpp:27
pub const PRESET_TEMPLATE_DIR: &str = "Template"; // Preset.hpp:28
pub const PRESET_CUSTOM_VENDOR: &str = "Custom"; // Preset.hpp:29

//BBS: iot preset type strings
pub const PRESET_IOT_PRINTER_TYPE: &str = "printer"; // Preset.hpp:32
pub const PRESET_IOT_FILAMENT_TYPE: &str = "filament"; // Preset.hpp:33
pub const PRESET_IOT_PRINT_TYPE: &str = "print"; // Preset.hpp:34

//BBS: add json support (Preset.hpp:38-80)
pub const BBL_JSON_KEY_VERSION: &str = "version";
pub const BBL_JSON_KEY_IS_CUSTOM: &str = "is_custom_defined";
pub const BBL_JSON_KEY_URL: &str = "url";
pub const BBL_JSON_KEY_NAME: &str = "name";
pub const BBL_JSON_KEY_DESCRIPTION: &str = "description";
pub const BBL_JSON_KEY_FORCE_UPDATE: &str = "force_update";
pub const BBL_JSON_KEY_MACHINE_MODEL_LIST: &str = "machine_model_list";
pub const BBL_JSON_KEY_PROCESS_LIST: &str = "process_list";
pub const BBL_JSON_KEY_SUB_PATH: &str = "sub_path";
pub const BBL_JSON_KEY_FILAMENT_LIST: &str = "filament_list";
pub const BBL_JSON_KEY_MACHINE_LIST: &str = "machine_list";
pub const BBL_JSON_KEY_TYPE: &str = "type";
pub const BBL_JSON_KEY_FROM: &str = "from";
pub const BBL_JSON_KEY_SETTING_ID: &str = "setting_id";
pub const BBL_JSON_KEY_BASE_ID: &str = "base_id";
pub const BBL_JSON_KEY_USER_ID: &str = "user_id";
pub const BBL_JSON_KEY_FILAMENT_ID: &str = "filament_id";
pub const BBL_JSON_KEY_UPDATE_TIME: &str = "updated_time";
pub const BBL_JSON_KEY_INHERITS: &str = "inherits";
pub const BBL_JSON_KEY_INCLUDES: &str = "include";
pub const BBL_JSON_KEY_INSTANTIATION: &str = "instantiation";
pub const BBL_JSON_KEY_NOZZLE_DIAMETER: &str = "nozzle_diameter";
pub const BBL_JSON_KEY_PRINTER_TECH: &str = "machine_tech";
pub const BBL_JSON_KEY_FAMILY: &str = "family";
pub const BBL_JSON_KEY_BED_MODEL: &str = "bed_model";
pub const BBL_JSON_KEY_BED_TEXTURE: &str = "bed_texture";
pub const BBL_JSON_KEY_USE_RECT_GRID: &str = "use_rect_grid";
pub const BBL_JSON_KEY_IMAGE_BED_TYPE: &str = "image_bed_type";
pub const BBL_JSON_KEY_DEFAULT_BED_TYPE: &str = "default_bed_type";
pub const BBL_JSON_KEY_BOTTOM_TEXTURE_END_NAME: &str = "bottom_texture_end_name";
pub const BBL_JSON_KEY_USE_DOUBLE_EXTRUDER_DEFAULT_TEXTURE: &str = "use_double_extruder_default_texture";
pub const BBL_JSON_KEY_BOTTOM_TEXTURE_RECT: &str = "bottom_texture_rect";
pub const BBL_JSON_KEY_BOTTOM_TEXTURE_RECT_LONGER: &str = "bottom_texture_rect_longer";
pub const BBL_JSON_KEY_BOTTOM_TEXTURE_RECT_LONGER_IGNORE_LIST: &str = "bottom_texture_rect_longer_ignore_list";
pub const BBL_JSON_KEY_MIDDLE_TEXTURE_RECT: &str = "middle_texture_rect";
pub const BBL_JSON_KEY_RIGHT_ICON_OFFSET_BED: &str = "right_icon_offset_bed";
pub const BBL_JSON_KEY_HOTEND_MODEL: &str = "hotend_model";
pub const BBL_JSON_KEY_DEFAULT_MATERIALS: &str = "default_materials";
pub const BBL_JSON_KEY_NOT_SUPPORT_BED_TYPE: &str = "not_support_bed_type";
pub const BBL_JSON_KEY_MODEL_ID: &str = "model_id";
pub const BBL_JSON_KEY_SUPPORT_SIDE_PANEL_FAN: &str = "support_side_panel_fan";
pub const BBL_JSON_KEY_RESOURCE_BIND_NAME: &str = "resource_bind_name";

// ============================================================================
// ConfigFileType (Preset.hpp:90-96)
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConfigFileType {
    ConfigFileTypeUnknown,
    ConfigFileTypeAppConfig,
    ConfigFileTypeConfig,
    ConfigFileTypeConfigBundle,
}

// ============================================================================
// Paths, as boost::filesystem builds them on POSIX
// ============================================================================

/// A directory separator of the platform's paths (`/`, and the backslash on Windows too).
fn is_path_sep(c: char) -> bool {
    c == '/' || (cfg!(windows) && c == '\\')
}

/// `boost::filesystem::path(a) / b` (filesystem v3): an empty operand gives the other one; the
/// preferred separator is added unless there is one between them already.
pub fn boost_path_append(a: &str, b: &str) -> String {
    if b.is_empty() {
        return a.to_string();
    }
    if a.is_empty() {
        return b.to_string();
    }
    if a.ends_with(is_path_sep) || b.starts_with(is_path_sep) {
        format!("{}{}", a, b)
    } else {
        format!("{}{}{}", a, std::path::MAIN_SEPARATOR, b)
    }
}

/// `path.make_preferred()`: `/` becomes a backslash on Windows; nothing changes elsewhere.
pub fn boost_make_preferred(p: String) -> String {
    if cfg!(windows) {
        p.replace('/', "\\")
    } else {
        p
    }
}

/// `boost::filesystem::absolute(p)`: `current_path() / p` for a relative path.
pub fn boost_absolute(p: &str) -> String {
    if std::path::Path::new(p).is_absolute() {
        return p.to_string();
    }
    let cwd = std::env::current_dir().map(|d| d.to_string_lossy().into_owned()).unwrap_or_default();
    boost_path_append(&cwd, p)
}

/// `path.filename()` of filesystem v3 as text: what follows the last separator.
fn boost_filename(p: &str) -> &str {
    match p.rfind(is_path_sep) {
        Some(i) => &p[i + 1..],
        None => p,
    }
}

/// `path.replace_extension(new_ext)` (filesystem v3): the extension of the file name (from its
/// last dot, none for "." and "..") is removed and `new_ext` appended, with a dot in front when
/// it has none.
pub fn boost_replace_extension(p: &str, new_ext: &str) -> String {
    let name = boost_filename(p);
    let stem_len = if name == "." || name == ".." {
        name.len()
    } else {
        match name.rfind('.') {
            Some(i) => i,
            None => name.len(),
        }
    };
    let mut out = p[..p.len() - name.len() + stem_len].to_string();
    if !new_ext.is_empty() {
        if !new_ext.starts_with('.') {
            out.push('.');
        }
        out.push_str(new_ext);
    }
    out
}

/// `path.parent_path()` (filesystem v3) as text.
fn boost_parent_path(p: &str) -> &str {
    match p.rfind(is_path_sep) {
        Some(0) => "/",
        Some(i) => &p[..i],
        None => "",
    }
}

/// `boost::filesystem::exists(p)`.
fn fs_exists(p: &str) -> bool {
    !p.is_empty() && std::path::Path::new(p).exists()
}

/// `boost::nowide::remove(p)` / `fs::remove(p)`, the error ignored as the C++ ignores the result.
fn fs_remove(p: &str) {
    let _ = std::fs::remove_file(p);
}

/// `boost::iends_with(s, suffix)` for an ASCII suffix.
fn iends_with(s: &str, suffix: &str) -> bool {
    s.len() >= suffix.len() && s.as_bytes()[s.len() - suffix.len()..].eq_ignore_ascii_case(suffix.as_bytes())
}

// ============================================================================
// boost::property_tree as read_ini builds it
// ============================================================================

/// `boost::property_tree::ptree`: the data of a node and its children in order (keys may repeat
/// in a ptree; read_ini never makes two equal keys under one parent).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PTree {
    pub data: String,
    pub children: Vec<(String, PTree)>,
}

impl PTree {
    /// `ptree::empty()`: no children.
    pub fn empty(&self) -> bool {
        self.children.is_empty()
    }
    /// `ptree::find(key)`: the first child of that key.
    pub fn find(&self, key: &str) -> Option<&PTree> {
        self.children.iter().find(|(k, _)| k == key).map(|(_, v)| v)
    }
    /// `ptree::get_child_optional(path)` with `.` separating the keys of the path.
    pub fn get_child_optional(&self, path: &str) -> Option<&PTree> {
        let mut node = self;
        for key in path.split('.') {
            node = node.find(key)?;
        }
        Some(node)
    }
    /// `ptree::get<std::string>(path, default_value)`.
    pub fn get_string(&self, path: &str, default_value: &str) -> String {
        match self.get_child_optional(path) {
            Some(n) => n.data.clone(),
            None => default_value.to_string(),
        }
    }
}

/// `std::isspace` in the classic locale, which boost's `detail::trim` uses through the stream's locale.
fn is_c_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r')
}

fn trim_c(s: &[u8]) -> &[u8] {
    let mut b = 0;
    let mut e = s.len();
    while b < e && is_c_space(s[b]) {
        b += 1;
    }
    while e > b && is_c_space(s[e - 1]) {
        e -= 1;
    }
    &s[b..e]
}

/// `boost::property_tree::read_ini(stream, pt)` (boost/property_tree/ini_parser.hpp) over the bytes
/// of a stream; an error is the `ini_parser_error` it throws ("<unspecified file>(line): message").
pub fn read_ini(data: &[u8]) -> CfgResult<PTree> {
    let err = |msg: &str, line_no: usize| {
        ConfigurationError::with_kind(ConfigurationErrorKind::IniParserError, format!("<unspecified file>({}): {}", line_no, msg))
    };
    let text = |b: &[u8]| String::from_utf8_lossy(b).into_owned();
    let mut local = PTree::default();
    let mut section: Option<usize> = None;
    let mut line_no = 0usize;
    // For all lines (std::getline: the text after the last '\n' is a line when it is not empty).
    let mut lines: Vec<&[u8]> = data.split(|&c| c == b'\n').collect();
    if lines.last().map_or(false, |l| l.is_empty()) {
        lines.pop();
    }
    for raw in lines {
        line_no += 1;
        let line = trim_c(raw);
        // If line is non-empty
        if line.is_empty() {
            continue;
        }
        // Comment, section or key?
        if line[0] == b';' || line[0] == b'#' {
            // Ignore comments
        } else if line[0] == b'[' {
            // If the previous section was empty, drop it again.
            if let Some(s) = section {
                if local.children[s].1.empty() {
                    local.children.pop();
                }
            }
            let end = match line.iter().position(|&c| c == b']') {
                None => return Err(err("unmatched '['", line_no)),
                Some(end) => end,
            };
            let key = text(trim_c(&line[1..end]));
            if local.find(&key).is_some() {
                return Err(err("duplicate section name", line_no));
            }
            local.children.push((key, PTree::default()));
            section = Some(local.children.len() - 1);
        } else {
            let eqpos = match line.iter().position(|&c| c == b'=') {
                None => return Err(err("'=' character not found in line", line_no)),
                Some(eqpos) => eqpos,
            };
            if eqpos == 0 {
                return Err(err("key expected", line_no));
            }
            let key = text(trim_c(&line[..eqpos]));
            let data = text(trim_c(&line[eqpos + 1..]));
            let container = match section {
                Some(s) => &mut local.children[s].1,
                None => &mut local,
            };
            if container.find(&key).is_some() {
                return Err(err("duplicate key name", line_no));
            }
            container.children.push((key, PTree { data, children: Vec::new() }));
        }
    }
    // If the last section was empty, drop it again.
    if let Some(s) = section {
        if local.children[s].1.empty() {
            local.children.pop();
        }
    }
    Ok(local)
}

// ============================================================================
// Preset.cpp:56-128 filament list order
// ============================================================================

/// Preset.hpp:110-115 `struct FilamentListKey`: identity of a filament preset as far as its
/// position in selection lists is concerned.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FilamentListKey {
    /// display name, e.g. "Bambu PLA Basic"
    pub alias: String,
    /// filament_vendor; empty for user presets
    pub vendor: String,
    /// filament_type
    pub r#type: String,
}

/// Preset.cpp:57-61. A trailing '*' turns an entry into a prefix match, so a whole product line keeps one slot.
struct VendorFilamentOrder {
    vendor: &'static str,
    aliases: Vec<&'static str>,
}

/// Preset.cpp:63-88. Empty vendor (user presets) leads; unlisted vendors follow by name.
fn curated_filament_orders() -> &'static Vec<VendorFilamentOrder> {
    static ORDERS: LazyLock<Vec<VendorFilamentOrder>> = LazyLock::new(|| {
        vec![
            VendorFilamentOrder { vendor: "", aliases: vec![] },
            VendorFilamentOrder {
                vendor: "Bambu",
                aliases: vec![
                    "Bambu PLA Basic", "Bambu PLA Matte", "Bambu PLA Pure", "Bambu PLA Lite",
                    "Bambu PLA Tough+", "Bambu PLA Palette", "Bambu PETG Basic", "Bambu PETG Matte",
                    "Bambu PETG HF", "Bambu ABS", "Bambu ASA", "Bambu PLA Silk+",
                    "Bambu PLA Silk", "Bambu PLA-CF", "Bambu PLA Marble", "Bambu PLA Metal",
                    "Bambu PLA Sparkle", "Bambu PLA Galaxy", "Bambu PLA Glow", "Bambu PLA Wood",
                    "Bambu PLA Translucent", "Bambu PETG Translucent", "Bambu PC", "Bambu PC FR",
                    "Bambu PETG-CF", "Bambu ABS-GF", "Bambu ASA-CF", "Bambu PA6-CF",
                    "Bambu PA6-GF", "Bambu PAHT-CF", "Bambu PET-CF", "Bambu PPA-CF",
                    "Bambu PPS-CF", "Bambu PLA Aero", "Bambu ASA-Aero", "Bambu TPU for AMS",
                    "Bambu TPU 95A HF", "Bambu TPU 90A", "Bambu TPU 85A", "Bambu TPU 75A(2.85mm)",
                    "Bambu Support For PLA", "Bambu Support For PLA/PETG", "Bambu Support for ABS", "Bambu PVA",
                    "Bambu Support For PA/PET", "Bambu TPU 95A", "Bambu PA-CF", "Bambu PLA Tough",
                    "Bambu PLA Dynamic", "Bambu Support W", "Bambu Support G",
                ],
            },
            VendorFilamentOrder { vendor: "Generic", aliases: vec!["Generic PLA", "Generic PETG", "Generic ABS", "Generic TPU"] },
            VendorFilamentOrder { vendor: "Polymaker", aliases: vec!["PolyLite PLA", "PolyTerra PLA", "PolyLite PETG", "Fiberon *"] },
            VendorFilamentOrder { vendor: "Overture", aliases: vec!["Overture Matte PLA", "Overture PLA"] },
            VendorFilamentOrder { vendor: "eSUN", aliases: vec!["eSUN PLA+"] },
        ]
    });
    &ORDERS
}

/// Preset.cpp:90-96
fn alias_rank(aliases: &[&'static str], alias: &str) -> usize {
    let matches = |entry: &&str| {
        if entry.ends_with('*') {
            alias.starts_with(&entry[..entry.len() - 1])
        } else {
            alias == *entry
        }
    };
    aliases.iter().position(matches).unwrap_or(aliases.len())
}

/// Preset.cpp:98-103. Unlisted vendors keep the common materials on top.
fn type_rank(r#type: &str) -> usize {
    const FIRST_TYPES: [&str; 4] = ["PLA", "PETG", "ABS", "TPU"];
    FIRST_TYPES.iter().position(|t| *t == r#type).unwrap_or(FIRST_TYPES.len())
}

/// Preset.cpp:107-128 `bool filament_list_less(const FilamentListKey &l, const FilamentListKey &r)`:
/// strict weak "listed before" order for filament presets in selection lists.
pub fn filament_list_less(l: &FilamentListKey, r: &FilamentListKey) -> bool {
    let normalized_vendor = |vendor: &str| if vendor == "Bambu Lab" { "Bambu".to_string() } else { vendor.to_string() };
    let lv = normalized_vendor(&l.vendor);
    let rv = normalized_vendor(&r.vendor);

    let orders = curated_filament_orders();
    let find_order = |vendor: &str| orders.iter().position(|o| o.vendor == vendor).unwrap_or(orders.len());
    let lo = find_order(&lv);
    let ro = find_order(&rv);
    if lo != ro {
        return lo < ro;
    }
    if lv != rv {
        return lv < rv;
    }

    if lo != orders.len() && !orders[lo].aliases.is_empty() {
        let la = alias_rank(&orders[lo].aliases, &l.alias);
        let ra = alias_rank(&orders[lo].aliases, &r.alias);
        if la != ra {
            return la < ra;
        }
    } else {
        let lt = type_rank(&l.r#type);
        let rt = type_rank(&r.r#type);
        if lt != rt {
            return lt < rt;
        }
    }
    l.alias < r.alias
}

// ============================================================================
// Preset.cpp:130-209 JSON / INI helpers
// ============================================================================

/// The bytes `boost::nowide::ifstream ifs(path)` reads; nothing when the file cannot be opened.
fn read_file_bytes(path: &str) -> Vec<u8> {
    std::fs::read(path).unwrap_or_default()
}

/// `j.at(key)` converted to `std::string`: nlohmann's out_of_range / type_error when the key is
/// missing or the value is not a string.
fn json_at_string(j: &serde_json::Value, key: &str) -> CfgResult<String> {
    let obj = match j.as_object() {
        Some(o) => o,
        None => {
            return Err(runtime_error(format!(
                "[json.exception.type_error.304] cannot use at() with {}",
                crate::config::nlohmann_type_name(j)
            )))
        }
    };
    match obj.get(key) {
        None => Err(runtime_error(format!("[json.exception.out_of_range.403] key '{}' not found", key))),
        Some(serde_json::Value::String(s)) => Ok(s.clone()),
        Some(v) => Err(runtime_error(format!(
            "[json.exception.type_error.302] type must be string, but is {}",
            crate::config::nlohmann_type_name(v)
        ))),
    }
}

/// Preset.cpp:130-151 `Semver get_version_from_json(std::string file_path)`. A parse error gives
/// `Semver()`; the other exceptions of nlohmann (a missing key, a value that is not a string, a
/// number out of range) are not caught by the C++ and are the `Err` here.
pub fn get_version_from_json(file_path: &str) -> CfgResult<Semver> {
    let j = match crate::config::nlohmann_json_parse(&read_file_bytes(file_path)) {
        Ok(j) => j,
        Err((true, _what)) => return Ok(Semver::new()),
        Err((false, what)) => return Err(runtime_error(what)),
    };
    let version_str = json_at_string(&j, BBL_JSON_KEY_VERSION)?;

    match Semver::parse(&version_str) {
        None => Ok(Semver::new()),
        Some(config_version) => Ok(config_version),
    }
}

/// Preset.cpp:153-175 `int get_values_from_json(std::string file_path, std::vector<std::string>& keys, std::map<std::string, std::string>& key_values)`
pub fn get_values_from_json(file_path: &str, keys: &[String], key_values: &mut BTreeMap<String, String>) -> CfgResult<i32> {
    let j = match crate::config::nlohmann_json_parse(&read_file_bytes(file_path)) {
        Ok(j) => j,
        Err((true, _what)) => return Ok(0),
        Err((false, what)) => return Err(runtime_error(what)),
    };
    for key in keys {
        let contains = j.as_object().map_or(false, |o| o.contains_key(key));
        if contains {
            let value = json_at_string(&j, key)?;
            key_values.entry(key.clone()).or_insert(value);
        }
    }
    Ok(key_values.len() as i32)
}

/// Preset.cpp:177-209 `ConfigFileType guess_config_file_type(const ptree &tree)`
/// (SUPPORT_BACKGROUND_PROCESSING is not defined in the build).
pub fn guess_config_file_type(tree: &PTree) -> ConfigFileType {
    let mut app_config: usize = 0;
    let mut bundle: usize = 0;
    let mut config: usize = 0;
    for (first, second) in &tree.children {
        if second.empty() {
            if first == "last_export_path" {
                app_config += 1;
            } else if first == "nozzle_diameter" || first == "filament_diameter" {
                config += 1;
            }
        } else if first.starts_with("print:") || first.starts_with("filament:") || first.starts_with("printer:") || first == "settings" {
            bundle += 1;
        } else if first == "presets" {
            app_config += 1;
            bundle += 1;
        } else if first == "recent" {
            for (kvp_first, _) in &second.children {
                if kvp_first == "settings_folder" || kvp_first == "last_opened_folder" {
                    app_config += 1;
                }
            }
        }
    }
    if app_config > bundle && app_config > config {
        ConfigFileType::ConfigFileTypeAppConfig
    } else if bundle > config {
        ConfigFileType::ConfigFileTypeConfigBundle
    } else {
        ConfigFileType::ConfigFileTypeConfig
    }
}

/// Preset.cpp:220-228 `static const std::unordered_map<std::string, std::string> pre_family_model_map`
fn pre_family_model_map(id: &str) -> Option<&'static str> {
    match id {
        "MK3" => Some("MK3"),
        "MK3MMU2" => Some("MK3"),
        "MK2.5" => Some("MK2.5"),
        "MK2.5MMU2" => Some("MK2.5"),
        "MK2S" => Some("MK2"),
        "MK2SMM" => Some("MK2"),
        "SL1" => Some("SL1"),
        _ => None,
    }
}

/// Preset.cpp:231-287 `void extend_default_config_length(DynamicPrintConfig &config, const DynamicPrintConfig &inherit_config, const bool set_nil_to_default, const DynamicPrintConfig &defaults)`.
/// Compatibility of intermediate versions: a nil value is first changed to the default, then
/// extended. The C++ callers pass `{}` (an empty config) for the parameters they do not use.
pub fn extend_default_config_length(
    config: &mut DynamicPrintConfig,
    inherit_config: &DynamicPrintConfig,
    set_nil_to_default: bool,
    defaults: &DynamicPrintConfig,
) -> CfgResult<()> {
    const DEFAULT_PARAM_LENGTH: i32 = 1;

    // Helper to copy variant/ID from inherit if missing, and return variant length
    let ensure_variant_and_get_len = |config: &mut DynamicPrintConfig, variant_key: &str, id_key: &str| -> i32 {
        if !config.has(variant_key) && inherit_config.has(variant_key) {
            config.set_key_value(variant_key, inherit_config.option(variant_key).unwrap().clone());
        }

        if !id_key.is_empty() && !config.has(id_key) && inherit_config.has(id_key) {
            config.set_key_value(id_key, inherit_config.option(id_key).unwrap().clone());
        }

        if !config.has(variant_key) || (!id_key.is_empty() && !config.has(id_key)) {
            config.erase(variant_key);
            config.erase(id_key);
        }

        if let Some(opt) = config.option_typed::<ConfigOptionStrings>(variant_key) {
            return opt.values.len() as i32;
        }
        DEFAULT_PARAM_LENGTH
    };

    let filament_variant_length = ensure_variant_and_get_len(config, "filament_extruder_variant", "");
    let process_variant_length = ensure_variant_and_get_len(config, "print_extruder_variant", "print_extruder_id");
    let machine_variant_length = ensure_variant_and_get_len(config, "printer_extruder_variant", "printer_extruder_id");

    let replace_nil_and_resize = |config: &mut DynamicPrintConfig, key: &str, length: i32| -> CfgResult<()> {
        let raw = config.option(key).expect("extend_default_config_length(): config.option(key)").clone();
        let opt_vec = config
            .option_mut(key, false)?
            .and_then(|o| o.as_vector_base_mut())
            .expect("extend_default_config_length(): static_cast<ConfigOptionVectorBase*>");
        if set_nil_to_default && raw.is_nil() && defaults.has(key) && !is_filament_extruder_override_key(key) {
            opt_vec.clear();
            opt_vec.resize(length as usize, defaults.option(key))?;
        } else {
            // `opt_vec->resize(length, raw_ptr)`: the option is its own default.
            opt_vec.resize(length as usize, Some(&*raw))?;
        }
        Ok(())
    };

    for key in config.keys() {
        if print_options_with_variant.contains(&key) {
            replace_nil_and_resize(config, &key, process_variant_length)?;
        } else if filament_options_with_variant.contains(&key) {
            replace_nil_and_resize(config, &key, filament_variant_length)?;
        } else if printer_options_with_variant_1.contains(&key) {
            replace_nil_and_resize(config, &key, machine_variant_length)?;
        } else if printer_options_with_variant_2.contains(&key) {
            replace_nil_and_resize(config, &key, machine_variant_length * 2)?;
        }
    }
    Ok(())
}

// ============================================================================
// VendorProfile (Preset.hpp:125-202, Preset.cpp:211-440)
// ============================================================================

/// Preset.hpp:134-138
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PrinterVariant {
    pub name: String,
}

impl PrinterVariant {
    /// Preset.hpp:135
    pub fn new() -> Self {
        Self::default()
    }
    /// Preset.hpp:136
    pub fn with_name(name: &str) -> Self {
        Self { name: name.to_string() }
    }
}

/// Preset.hpp:140-181
#[derive(Clone, Debug)]
pub struct PrinterModel {
    pub id: String,
    pub name: String,
    //BBS: this is internal id for the printer. Currently only used for searching in database
    pub model_id: String,
    pub technology: PrinterTechnology,
    pub family: String,
    pub variants: Vec<PrinterVariant>,
    pub default_materials: Vec<String>,
    pub not_support_bed_types: Vec<String>,
    // Bed types that should keep default bottom texture position
    // instead of applying bottom_texture_rect_longer.
    pub bottom_texture_rect_longer_ignore_list: Vec<String>,
    // Vendor & Printer Model specific print bed model & texture.
    pub bed_model: String,
    pub bed_texture: String,
    pub use_rect_grid: String,
    pub image_bed_type: String,
    pub default_bed_type: String,
    pub bottom_texture_end_name: String,
    pub use_double_extruder_default_texture: String,
    pub bottom_texture_rect: String,
    pub bottom_texture_rect_longer: String,
    pub middle_texture_rect: String,
    pub right_icon_offset_bed: String,
    pub hotend_model: String,
    pub support_side_panel_fan: String,
    // Optional per-printer-model suffix appended to the base thumbnail image
    // name when looking up bed thumbnails (see Sidebar::get_cur_select_bed_image).
    // Empty means "no override" and the default image name is used as-is.
    pub resource_bind_name: String,
}

impl Default for PrinterModel {
    /// Preset.hpp:141 `PrinterModel() {}`. FIDELITY-NOTE: `technology` is left uninitialized by
    /// the C++ constructor; it is ptFFF here.
    fn default() -> Self {
        Self {
            id: String::new(),
            name: String::new(),
            model_id: String::new(),
            technology: PrinterTechnology::PtFFF,
            family: String::new(),
            variants: Vec::new(),
            default_materials: Vec::new(),
            not_support_bed_types: Vec::new(),
            bottom_texture_rect_longer_ignore_list: Vec::new(),
            bed_model: String::new(),
            bed_texture: String::new(),
            use_rect_grid: String::new(),
            image_bed_type: String::new(),
            default_bed_type: String::new(),
            bottom_texture_end_name: String::new(),
            use_double_extruder_default_texture: String::new(),
            bottom_texture_rect: String::new(),
            bottom_texture_rect_longer: String::new(),
            middle_texture_rect: String::new(),
            right_icon_offset_bed: String::new(),
            hotend_model: String::new(),
            // Preset.hpp:167 `std::string support_side_panel_fan{ "true" };`
            support_side_panel_fan: "true".to_string(),
            resource_bind_name: String::new(),
        }
    }
}

impl PrinterModel {
    /// Preset.hpp:172-177
    pub fn variant_mut(&mut self, name: &str) -> Option<&mut PrinterVariant> {
        for v in self.variants.iter_mut() {
            if v.name == name {
                return Some(v);
            }
        }
        None
    }

    /// Preset.hpp:179
    pub fn variant(&self, name: &str) -> Option<&PrinterVariant> {
        self.variants.iter().find(|v| v.name == name)
    }

    /// Preset.cpp:289-298 `std::map<std::string, std::string> VendorProfile::PrinterModel::get_bed_texture_maps() const`
    pub fn get_bed_texture_maps(&self) -> BTreeMap<String, String> {
        let mut maps: BTreeMap<String, String> = BTreeMap::new();
        if !self.use_double_extruder_default_texture.is_empty() {
            maps.insert("use_double_extruder_default_texture".to_string(), self.use_double_extruder_default_texture.clone());
        }
        if !self.bottom_texture_end_name.is_empty() {
            maps.insert("bottom_texture_end_name".to_string(), self.bottom_texture_end_name.clone());
        }
        if !self.bottom_texture_rect.is_empty() {
            maps.insert("bottom_texture_rect".to_string(), self.bottom_texture_rect.clone());
        }
        if !self.bottom_texture_rect_longer.is_empty() {
            maps.insert("bottom_texture_rect_longer".to_string(), self.bottom_texture_rect_longer.clone());
        }
        if !self.middle_texture_rect.is_empty() {
            maps.insert("middle_texture_rect".to_string(), self.middle_texture_rect.clone());
        }
        maps
    }
}

/// Preset.hpp:125-202
#[derive(Clone, Debug, Default)]
pub struct VendorProfile {
    pub name: String,
    pub id: String,
    pub config_version: Semver,
    pub config_update_url: String,
    pub changelog_url: String,
    pub models: Vec<PrinterModel>,
    pub default_filaments: BTreeSet<String>,
    pub default_sla_materials: BTreeSet<String>,
}

impl VendorProfile {
    /// Preset.hpp:187 `VendorProfile() {}`
    pub fn new() -> Self {
        Self::default()
    }

    /// Preset.hpp:188 `VendorProfile(std::string id) : id(std::move(id)) {}`
    pub fn with_id(id: String) -> Self {
        Self { id, ..Self::default() }
    }

    /// Preset.hpp:190
    pub fn valid(&self) -> bool {
        !self.name.is_empty() && !self.id.is_empty() && self.config_version.valid()
    }

    /// Preset.cpp:212-218 `VendorProfile VendorProfile::from_ini(const boost::filesystem::path &path, bool load_all)`:
    /// load a VendorProfile from an ini file. If `load_all` is false, only the header with basic
    /// info (name, version, URLs) is loaded.
    pub fn from_ini(path: &str, load_all: bool) -> CfgResult<VendorProfile> {
        let tree = read_ini(&read_file_bytes(path))?;
        VendorProfile::from_ini_tree(&tree, path, load_all)
    }

    /// Preset.cpp:300-425 `VendorProfile VendorProfile::from_ini(const ptree &tree, const boost::filesystem::path &path, bool load_all)`
    pub fn from_ini_tree(tree: &PTree, path: &str, load_all: bool) -> CfgResult<VendorProfile> {
        const PRINTER_MODEL_KEY: &str = "printer_model:";
        const FILAMENTS_SECTION: &str = "default_filaments";
        const MATERIALS_SECTION: &str = "default_sla_materials";

        // path.stem()
        let file_name = boost_filename(path);
        let id = if file_name == "." || file_name == ".." {
            file_name.to_string()
        } else {
            match file_name.rfind('.') {
                Some(i) => file_name[..i].to_string(),
                None => file_name.to_string(),
            }
        };

        if !fs_exists(path) {
            return Err(runtime_error(format!(
                "Cannot load Vendor Config Bundle `{}`: File not found: `\"{}\"`.",
                id, path
            )));
        }

        let mut res = VendorProfile::with_id(id.clone());

        // Helper to get compulsory fields
        let get_or_throw = |tree: &'_ PTree, key: &str| -> CfgResult<PTree> {
            match tree.find(key) {
                None => Err(runtime_error(format!(
                    "Vendor Config Bundle `{}` is not valid: Missing secion or key: `{}`.",
                    id, key
                ))),
                Some(t) => Ok(t.clone()),
            }
        };

        // Load the header
        let vendor_section = get_or_throw(tree, "vendor")?;
        res.name = get_or_throw(&vendor_section, "name")?.data;

        let config_version_str = get_or_throw(&vendor_section, "config_version")?.data;
        match Semver::parse(&config_version_str) {
            None => {
                return Err(runtime_error(format!(
                    "Vendor Config Bundle `{}` is not valid: Cannot parse config_version: `{}`.",
                    id, config_version_str
                )))
            }
            Some(config_version) => res.config_version = config_version,
        }

        // Load URLs
        if let Some(config_update_url) = vendor_section.find("config_update_url") {
            res.config_update_url = config_update_url.data.clone();
        }

        if let Some(changelog_url) = vendor_section.find("changelog_url") {
            res.changelog_url = changelog_url.data.clone();
        }

        if !load_all {
            return Ok(res);
        }

        // Load printer models
        for (section_first, section_second) in &tree.children {
            if section_first.starts_with(PRINTER_MODEL_KEY) {
                let mut model = PrinterModel::default();
                model.id = section_first[PRINTER_MODEL_KEY.len()..].to_string();
                model.name = section_second.get_string("name", &model.id);

                let technology_fallback = if model.id.starts_with("SL") { "SLA" } else { "FFF" };

                let technology_field = section_second.get_string("technology", technology_fallback);
                if !ConfigOptionEnum::<PrinterTechnology>::from_string(&technology_field, &mut model.technology) {
                    // BOOST_LOG_TRIVIAL(error) << "Vendor bundle: `%1%`: Invalid printer technology field: `%2%`"
                    model.technology = PrinterTechnology::PtFFF;
                }

                model.family = section_second.get_string("family", "");
                if model.family.is_empty() && res.name == "BBL" {
                    // If no family is specified, it can be inferred for known printers
                    if let Some(from_pre_map) = pre_family_model_map(&model.id) {
                        model.family = from_pre_map.to_string();
                    }
                }
                let variants_field = section_second.get_string("variants", "");
                let mut variants: Vec<String> = Vec::new();
                if unescape_strings_cstyle(&variants_field, &mut variants) {
                    for variant_name in &variants {
                        if model.variant(variant_name).is_none() {
                            model.variants.push(PrinterVariant::with_name(variant_name));
                        }
                    }
                } else {
                    // BOOST_LOG_TRIVIAL(error) << "Vendor bundle: `%1%`: Malformed variants field: `%2%`"
                }
                let mut default_materials_field = section_second.get_string("default_materials", "");
                if default_materials_field.is_empty() {
                    default_materials_field = section_second.get_string("default_filaments", "");
                }
                if unescape_strings_cstyle(&default_materials_field, &mut model.default_materials) {
                    // Slic3r::sort_remove_duplicates
                    model.default_materials.sort();
                    model.default_materials.dedup();
                    if !model.default_materials.is_empty() && model.default_materials[0].is_empty() {
                        // An empty material was inserted into the list of default materials. Remove it.
                        model.default_materials.remove(0);
                    }
                } else {
                    // BOOST_LOG_TRIVIAL(error) << "Vendor bundle: `%1%`: Malformed default_materials field: `%2%`"
                }
                model.bed_model = section_second.get_string("bed_model", "");
                model.bed_texture = section_second.get_string("bed_texture", "");
                if !model.id.is_empty() && !model.variants.is_empty() {
                    res.models.push(model);
                }
            }
        }

        // Load filaments and sla materials to be installed by default
        if let Some(filaments) = tree.find(FILAMENTS_SECTION) {
            for (first, second) in &filaments.children {
                if second.data == "1" {
                    res.default_filaments.insert(first.clone());
                }
            }
        }
        if let Some(materials) = tree.find(MATERIALS_SECTION) {
            for (first, second) in &materials.children {
                if second.data == "1" {
                    res.default_sla_materials.insert(first.clone());
                }
            }
        }

        Ok(res)
    }

    /// Preset.hpp:197
    pub fn num_variants(&self) -> usize {
        let mut n: usize = 0;
        for model in &self.models {
            n += model.variants.len();
        }
        n
    }

    /// Preset.cpp:427-440 `std::vector<std::string> VendorProfile::families() const`
    pub fn families(&self) -> Vec<String> {
        let mut res: Vec<String> = Vec::new();
        let mut _num_familiies: u32 = 0;

        for model in &self.models {
            if !res.iter().any(|f| *f == model.family) {
                res.push(model.family.clone());
                _num_familiies += 1;
            }
        }

        res
    }
}

// Preset.hpp:200 `bool operator< (const VendorProfile &rhs) const { return this->id <  rhs.id; }`
// Preset.hpp:201 `bool operator==(const VendorProfile &rhs) const { return this->id == rhs.id; }`
impl PartialEq for VendorProfile {
    fn eq(&self, rhs: &Self) -> bool {
        self.id == rhs.id
    }
}
impl Eq for VendorProfile {}
impl PartialOrd for VendorProfile {
    fn partial_cmp(&self, rhs: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(rhs))
    }
}
impl Ord for VendorProfile {
    fn cmp(&self, rhs: &Self) -> std::cmp::Ordering {
        self.id.cmp(&rhs.id)
    }
}

/// Preset.hpp:206-213 `struct PresetWithVendorProfile`: a profile with its vendor definition,
/// where the vendor definition may have been extracted from a parent system preset.
#[derive(Clone, Copy)]
pub struct PresetWithVendorProfile<'a> {
    pub preset: &'a Preset,
    pub vendor: Option<&'a VendorProfile>,
}

impl<'a> PresetWithVendorProfile<'a> {
    /// Preset.hpp:210
    pub fn new(preset: &'a Preset, vendor: Option<&'a VendorProfile>) -> Self {
        PresetWithVendorProfile { preset, vendor }
    }
}

/// `const VendorProfile*` equality: the same object.
fn vendor_ptr_eq(a: Option<&VendorProfile>, b: Option<&VendorProfile>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => std::ptr::eq(a, b),
        _ => false,
    }
}

/// Preset.hpp:215-219 `typedef std::map<std::string, VendorProfile> VendorMap;` (FIDELITY-NOTE:
/// the profiles are held in `Arc`s, which presets point to; see the module notes).
pub type VendorMap = BTreeMap<String, Arc<VendorProfile>>;

// ============================================================================
// Preset (Preset.hpp:221-413)
// ============================================================================

/// Preset.hpp:224-240 `enum Preset::Type`
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub enum Type {
    TypeInvalid,
    TypePrint,
    TypeSlaPrint,
    TypeFilament,
    TypeSlaMaterial,
    TypePrinter,
    TypeCount,
    // This type is here to support PresetConfigSubstitutions for physical printers, however it does not belong to the Preset class,
    // PhysicalPrinter class is used instead.
    TypePhysicalPrinter,
    // BBS: plate config
    TypePlate,
    // BBS: model config
    TypeModel,
}

// Suffix to be added to a modified preset name in the combo box.
// Preset.cpp:443  static std::string g_suffix_modified = " (modified)";
static G_SUFFIX_MODIFIED: RwLock<Option<String>> = RwLock::new(None);

fn g_suffix_modified_get() -> String {
    let guard = G_SUFFIX_MODIFIED.read().unwrap();
    match guard.as_ref() {
        Some(s) => s.clone(),
        None => " (modified)".to_string(),
    }
}

impl Type {
    /// Preset.cpp:523-539 `std::string Preset::get_type_string(Preset::Type type)`
    pub fn get_type_string(ty: Type) -> String {
        match ty {
            Type::TypeFilament => PRESET_FILAMENT_NAME.to_string(),
            Type::TypePrint => PRESET_PRINT_NAME.to_string(),
            Type::TypePrinter => PRESET_PRINTER_NAME.to_string(),
            Type::TypePhysicalPrinter => "physical_printer".to_string(),
            Type::TypeInvalid => "invalid".to_string(),
            _ => "invalid".to_string(),
        }
    }

    /// Preset.cpp:541-554 `std::string Preset::get_iot_type_string(Preset::Type type)`
    pub fn get_iot_type_string(ty: Type) -> String {
        match ty {
            Type::TypeFilament => PRESET_IOT_FILAMENT_TYPE.to_string(),
            Type::TypePrint => PRESET_IOT_PRINT_TYPE.to_string(),
            Type::TypePrinter => PRESET_IOT_PRINTER_TYPE.to_string(),
            _ => "invalid".to_string(),
        }
    }

    /// Preset.cpp:556-567 `Preset::Type Preset::get_type_from_string(std::string type_str)`:
    /// make the type string compatibility with local and iot type string
    pub fn get_type_from_string(type_str: &str) -> Type {
        if type_str == PRESET_PRINT_NAME || type_str == PRESET_IOT_PRINT_TYPE {
            Type::TypePrint
        } else if type_str == PRESET_FILAMENT_NAME || type_str == PRESET_IOT_FILAMENT_TYPE {
            Type::TypeFilament
        } else if type_str == PRESET_PRINTER_NAME || type_str == PRESET_IOT_PRINTER_TYPE {
            Type::TypePrinter
        } else {
            Type::TypeInvalid
        }
    }
}

/// Preset.hpp:221-413 `class Preset`
pub struct Preset {
    pub r#type: Type,
    // The preset represents a "default" set of properties,
    // pulled from the default values of the PrintConfig (see PrintConfigDef for their definitions).
    pub is_default: bool,
    // External preset points to a configuration, which has been loaded but not imported
    // into the Slic3r default configuration location.
    pub is_external: bool,
    // System preset is read-only.
    pub is_system: bool,
    // Preset is visible, if it is associated with a printer model / variant that is enabled in the AppConfig
    // or if it has no printer model / variant association.
    // Also the "default" preset is only visible, if it is the only preset in the list.
    pub is_visible: bool,
    // Has this preset been modified?
    pub is_dirty: bool,
    // Is this preset compatible with the currently active printer?
    pub is_compatible: bool,
    //BBS: add type for project-embedded
    pub is_project_embedded: bool,
    /// `ConfigSubstitutions *loading_substitutions{nullptr};` (FIDELITY-NOTE: owned here; the
    /// C++ pointer is shallow-copied with the preset and freed by load_project_embedded_presets).
    pub loading_substitutions: Option<ConfigSubstitutions>,
    // Name of the preset, usually derived form the file name.
    pub name: String,
    // File name of the preset. This could be a Print / Filament / Printer preset,
    // or a Configuration file bundling the Print + Filament + Printer presets (in that case is_external and possibly is_system will be true),
    // or it could be a G-code (again, is_external will be true).
    pub file: String,
    // If this is a system profile, then there should be a vendor data available to display at the UI.
    pub vendor: Option<Arc<VendorProfile>>,
    // Has this profile been loaded?
    pub loaded: bool,
    // Configuration data, loaded from a file, or set from the defaults.
    pub config: DynamicPrintConfig,
    // Alias of the preset
    pub alias: String,
    // List of profile names, from which this profile was renamed at some point of time.
    // This list is then used to match profiles by their names when loaded from .gcode, .3mf, .amf,
    // and to match the "inherits" field of user profiles with updated system profiles.
    pub renamed_from: Vec<String>,
    //BBS
    pub version: Semver,       // version of preset
    pub ini_str: String,       // ini string of preset
    pub setting_id: String,    // setting id in cloud database
    pub filament_id: String,   // setting id in cloud database
    pub user_id: String,       // preset user_id
    pub base_id: String,       // base id of preset
    pub sync_info: String,     // enum: "delete", "create", "update", ""
    pub description: String,   //
    pub updated_time: i64,     //last updated time
    pub key_values: BTreeMap<String, String>,
}

fn clone_substitutions(s: &ConfigSubstitutions) -> ConfigSubstitutions {
    s.iter()
        .map(|x| ConfigSubstitution { opt_def: x.opt_def, old_value: x.old_value.clone(), new_value: x.new_value.clone() })
        .collect()
}

impl Clone for Preset {
    fn clone(&self) -> Self {
        Preset {
            r#type: self.r#type,
            is_default: self.is_default,
            is_external: self.is_external,
            is_system: self.is_system,
            is_visible: self.is_visible,
            is_dirty: self.is_dirty,
            is_compatible: self.is_compatible,
            is_project_embedded: self.is_project_embedded,
            loading_substitutions: self.loading_substitutions.as_ref().map(clone_substitutions),
            name: self.name.clone(),
            file: self.file.clone(),
            vendor: self.vendor.clone(),
            loaded: self.loaded,
            config: self.config.clone(),
            alias: self.alias.clone(),
            renamed_from: self.renamed_from.clone(),
            version: self.version.clone(),
            ini_str: self.ini_str.clone(),
            setting_id: self.setting_id.clone(),
            filament_id: self.filament_id.clone(),
            user_id: self.user_id.clone(),
            base_id: self.base_id.clone(),
            sync_info: self.sync_info.clone(),
            description: self.description.clone(),
            updated_time: self.updated_time,
            key_values: self.key_values.clone(),
        }
    }
}

impl std::fmt::Debug for Preset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Preset")
            .field("type", &self.r#type)
            .field("name", &self.name)
            .field("file", &self.file)
            .field("is_default", &self.is_default)
            .field("is_system", &self.is_system)
            .field("is_external", &self.is_external)
            .field("is_visible", &self.is_visible)
            .field("is_dirty", &self.is_dirty)
            .field("is_compatible", &self.is_compatible)
            .field("options", &self.config.keys().len())
            .finish()
    }
}

impl Default for Preset {
    /// Preset.hpp:409 `Preset() = default;` (protected)
    fn default() -> Self {
        Preset::new(Type::TypeInvalid, "", false)
    }
}

impl Preset {
    pub const TYPE_INVALID: Type = Type::TypeInvalid;
    pub const TYPE_PRINT: Type = Type::TypePrint;
    pub const TYPE_SLA_PRINT: Type = Type::TypeSlaPrint;
    pub const TYPE_FILAMENT: Type = Type::TypeFilament;
    pub const TYPE_SLA_MATERIAL: Type = Type::TypeSlaMaterial;
    pub const TYPE_PRINTER: Type = Type::TypePrinter;
    pub const TYPE_COUNT: Type = Type::TypeCount;
    pub const TYPE_PHYSICAL_PRINTER: Type = Type::TypePhysicalPrinter;
    pub const TYPE_PLATE: Type = Type::TypePlate;
    pub const TYPE_MODEL: Type = Type::TypeModel;

    /// Preset.hpp:405-406 `Preset(Type type, const std::string &name, bool is_default = false)` (BBS: move constructor to public)
    pub fn new(ty: Type, name: &str, is_default: bool) -> Self {
        Self {
            r#type: ty,
            is_default,
            is_external: false,
            is_system: false,
            is_visible: true,
            is_dirty: false,
            is_compatible: true,
            is_project_embedded: false,
            loading_substitutions: None,
            name: name.to_string(),
            file: String::new(),
            vendor: None,
            loaded: false,
            config: DynamicPrintConfig::new(),
            alias: String::new(),
            renamed_from: Vec::new(),
            version: Semver::default(),
            ini_str: String::new(),
            setting_id: String::new(),
            filament_id: String::new(),
            user_id: String::new(),
            base_id: String::new(),
            sync_info: String::new(),
            description: String::new(),
            updated_time: 0,
            key_values: BTreeMap::new(),
        }
    }

    /// Preset.hpp:264
    pub fn is_user(&self) -> bool {
        !self.is_default && !self.is_system && !self.is_project_embedded
    }

    /// Preset.cpp:523 (see `Type::get_type_string`)
    pub fn get_type_string(ty: Type) -> String {
        Type::get_type_string(ty)
    }

    /// Preset.cpp:541 (see `Type::get_iot_type_string`)
    pub fn get_iot_type_string(ty: Type) -> String {
        Type::get_iot_type_string(ty)
    }

    /// Preset.cpp:557 (see `Type::get_type_from_string`)
    pub fn get_type_from_string(type_str: &str) -> Type {
        Type::get_type_from_string(type_str)
    }

    /// Preset.cpp:570-611 `void Preset::load_info(const std::string& file)`
    pub fn load_info(&mut self, file: &str) {
        let tree = match read_ini(&read_file_bytes(file)) {
            Ok(t) => t,
            Err(_) => {
                // catch (...) { BOOST_LOG_TRIVIAL(warning) << current_exception_message(); return; }
                return;
            }
        };
        if tree.empty() {
            return;
        }
        for (first, second) in &tree.children {
            if first == "sync_info" {
                self.sync_info = second.data.clone();
            } else if first == "user_id" {
                self.user_id = second.data.clone();
            } else if first == "setting_id" {
                self.setting_id = second.data.clone();
                if self.setting_id == "null" {
                    self.setting_id.clear();
                }
            } else if first == "base_id" {
                self.base_id = second.data.clone();
                if self.base_id == "null" {
                    self.base_id.clear();
                }
            } else if first == "updated_time" {
                let time = &second.data;
                self.updated_time = c_atoll(time);
            }
        }

        //TODO: workaround for current info file convert, will remove it later
        if self.updated_time == 0 {
            self.updated_time = crate::time::get_current_time_utc();
            //this->sync_info = "update";
            self.save_info("");
        }
    }

    /// Preset.cpp:613-646 `void Preset::save_info(std::string file)`
    pub fn save_info(&mut self, file: &str) {
        //BBS: add project embedded preset logic
        if self.is_project_embedded {
            return;
        }
        let file = if file.is_empty() { boost_replace_extension(&self.file, ".info") } else { file.to_string() };

        // Symmetric with load_info() above: a freshly-cloned preset arrives here with
        // updated_time == 0 (clone_presets doesn't stamp it). Stamping here fills that gap once, at save time.
        if self.updated_time == 0 {
            self.updated_time = crate::time::get_current_time_utc();
        }

        let mut sync_info_to_save = String::new();
        //BBS: hold is used for stop requesting to server this time
        if self.sync_info != "hold" {
            sync_info_to_save = self.sync_info.clone();
        }
        let mut c = String::new();
        c.push_str(&format!("sync_info = {}\n", sync_info_to_save));
        c.push_str(&format!("user_id = {}\n", self.user_id));
        c.push_str(&format!("setting_id = {}\n", self.setting_id));
        c.push_str(&format!("base_id = {}\n", self.base_id));
        c.push_str(&format!("updated_time = {}\n", self.updated_time));
        // c.open(file, std::ios::out | std::ios::trunc): nothing is written when it cannot be opened.
        let _ = std::fs::write(&file, c);
    }

    /// Preset.cpp:648-659 `void Preset::remove_files()`
    pub fn remove_files(&self) {
        //BBS: add project embedded preset logic
        if self.is_project_embedded {
            return;
        }
        // Erase the preset file.
        fs_remove(&self.file);
        let idx_path = boost_replace_extension(&self.file, ".info");
        if fs_exists(&idx_path) {
            fs_remove(&idx_path);
        }
    }

    /// Preset.cpp:661-737 `bool Preset::save(DynamicPrintConfig* parent_config)`: only the
    /// difference to the parent is saved when there is a parent, otherwise all keys.
    pub fn save(&mut self, parent_config: Option<&DynamicPrintConfig>) -> CfgResult<bool> {
        //BBS: add project embedded preset logic
        if self.is_project_embedded {
            return Ok(false);
        }
        let from_str = if self.is_user() {
            "User"
        } else if self.is_project_embedded {
            "Project"
        } else if self.is_system {
            "System"
        } else {
            "Default"
        };

        // boost::filesystem::create_directories(p, ec) fails (invalid_argument) for an empty path.
        let parent_path = boost_parent_path(&self.file).to_string();
        if !fs_exists(&parent_path) && (parent_path.is_empty() || std::fs::create_dir_all(&parent_path).is_err()) {
            // BOOST_LOG_TRIVIAL(error) << "create directory failed: " ...
            return Ok(false);
        }

        //BBS: only save difference if it has parent
        if let Some(parent_config) = parent_config {
            let mut temp_config = DynamicPrintConfig::new();
            let mut dirty_options = self.config.base.diff(&parent_config.base)?;
            let mut extruder_id_name = String::new();
            let mut extruder_variant_name = String::new();
            let mut key_set1: &'static BTreeSet<String> = &empty_options;
            let mut key_set2: &'static BTreeSet<String> = &empty_options;
            Preset::get_extruder_names_and_keysets(self.r#type, &mut extruder_id_name, &mut extruder_variant_name, &mut key_set1, &mut key_set2);

            if !extruder_id_name.is_empty() {
                dirty_options.push(extruder_id_name.clone());
            }
            if !extruder_variant_name.is_empty() {
                dirty_options.push(extruder_variant_name.clone());
            }

            for option in &dirty_options {
                set_option_for_save(&mut temp_config, &self.config, parent_config, option, key_set1, key_set2)?;
            }
            temp_config.save_to_json(&self.file, &self.name, from_str, &self.version.to_string())?;
        } else if !self.filament_id.is_empty() && self.inherits_mut().is_empty() {
            let mut temp_config = self.config.clone();
            temp_config.set_key_value(BBL_JSON_KEY_FILAMENT_ID, Box::new(ConfigOptionString::with_value(self.filament_id.clone())));
            temp_config.save_to_json(&self.file, &self.name, from_str, &self.version.to_string())?;
        } else {
            self.config.save_to_json(&self.file, &self.name, from_str, &self.version.to_string())?;
        }

        let idx_file = boost_replace_extension(&self.file, ".info");
        self.save_info(&idx_file);
        Ok(true)
    }

    /// Preset.cpp:739-754 `void Preset::reload(Preset const & parent)`
    pub fn reload(&mut self, parent: &Preset) {
        let mut config = DynamicPrintConfig::new();
        // BBS: change to json format
        let mut key_values: BTreeMap<String, String> = BTreeMap::new();
        let mut reason = String::new();
        let substitution_rule = ForwardCompatibilitySubstitutionRule::Disable;
        let _config_substitutions = config.load_from_json(&self.file, substitution_rule, &mut key_values, &mut reason);
        self.config = parent.config.clone();
        if let Err(_err) = self.config.apply(&config, false) {
            // BOOST_LOG_TRIVIAL(error) << "Failed loading the user-config file: %1%. Reason: %2%"
        }
    }

    /// Preset.cpp:756-761 `std::string Preset::label(bool no_alias) const`: a label of this
    /// preset, consisting of a name and a "(modified)" suffix, if this preset is dirty.
    pub fn label(&self, no_alias: bool) -> String {
        let prefix = if self.is_dirty { g_suffix_modified_get() } else { String::new() };
        let body = if no_alias || self.alias.is_empty() { self.name.clone() } else { self.alias.clone() };
        prefix + &body
    }

    /// Preset.cpp:763-782 `std::string Preset::display_name() const`: a short display name:
    /// alias (or name before '@') with vendor prefix stripped.
    pub fn display_name(&self) -> String {
        let mut result: String;
        if !self.alias.is_empty() {
            result = self.alias.clone();
        } else {
            let at_pos = self.name.find('@').unwrap_or(self.name.len());
            result = self.name[..at_pos].to_string();
            // boost::algorithm::trim
            result = String::from_utf8_lossy(trim_c(result.as_bytes())).into_owned();
        }

        let vendor = self.config.get_filament_vendor();
        if !vendor.is_empty() {
            let prefix = (if vendor == "Bambu Lab" { "Bambu".to_string() } else { vendor }) + " ";
            if result.as_bytes().starts_with(prefix.as_bytes()) {
                result = result[prefix.len()..].to_string();
            }
        }

        result
    }

    /// Preset.hpp:321 `void set_dirty(const DynamicPrintConfig &config)`: set the is_dirty flag
    /// if the provided config is different from the active one.
    pub fn set_dirty_config(&mut self, config: &DynamicPrintConfig) -> CfgResult<()> {
        self.is_dirty = !self.config.base.diff(&config.base)?.is_empty();
        Ok(())
    }
    /// Preset.hpp:322 `void set_dirty(bool dirty = true)`
    pub fn set_dirty(&mut self, dirty: bool) {
        self.is_dirty = dirty;
    }
    /// Preset.hpp:323
    pub fn reset_dirty(&mut self) {
        self.is_dirty = false;
    }

    /// `cfg.option<ConfigOptionString>(key, true)->value`
    fn string_option_mut<'a>(cfg: &'a mut DynamicPrintConfig, key: &str) -> &'a mut String {
        &mut cfg
            .option_typed_mut::<ConfigOptionString>(key, true)
            .expect("Preset: no definition")
            .expect("Preset: option<ConfigOptionString>(key, true) is null")
            .value
    }
    /// The value of a string option, without creating it (see the module notes).
    fn string_option<'a>(cfg: &'a DynamicPrintConfig, key: &str) -> &'a str {
        match cfg.option_typed::<ConfigOptionString>(key) {
            Some(o) => &o.value,
            None => "",
        }
    }

    /// Preset.hpp:326 `static std::string& inherits(DynamicPrintConfig &cfg)`: the name of the
    /// preset, from which this preset inherits.
    pub fn inherits_cfg(cfg: &mut DynamicPrintConfig) -> &mut String {
        Preset::string_option_mut(cfg, "inherits")
    }
    /// Preset.hpp:327 `std::string& inherits()`
    pub fn inherits_mut(&mut self) -> &mut String {
        Preset::inherits_cfg(&mut self.config)
    }
    /// Preset.hpp:328 `const std::string& inherits() const` (see the module notes).
    pub fn inherits(&self) -> &str {
        Preset::string_option(&self.config, "inherits")
    }

    /// Preset.hpp:331 `static std::string& compatible_prints_condition(DynamicPrintConfig &cfg)`
    pub fn compatible_prints_condition_cfg(cfg: &mut DynamicPrintConfig) -> &mut String {
        Preset::string_option_mut(cfg, "compatible_prints_condition")
    }
    /// Preset.hpp:332-335 (the assert of the type is compiled out)
    pub fn compatible_prints_condition_mut(&mut self) -> &mut String {
        Preset::compatible_prints_condition_cfg(&mut self.config)
    }
    /// Preset.hpp:336 (see the module notes)
    pub fn compatible_prints_condition(&self) -> &str {
        Preset::string_option(&self.config, "compatible_prints_condition")
    }

    /// Preset.hpp:339 `static std::string& compatible_printers_condition(DynamicPrintConfig &cfg)`
    pub fn compatible_printers_condition_cfg(cfg: &mut DynamicPrintConfig) -> &mut String {
        Preset::string_option_mut(cfg, "compatible_printers_condition")
    }
    /// Preset.hpp:340-343 (the assert of the type is compiled out)
    pub fn compatible_printers_condition_mut(&mut self) -> &mut String {
        Preset::compatible_printers_condition_cfg(&mut self.config)
    }
    /// Preset.hpp:344 (see the module notes)
    pub fn compatible_printers_condition(&self) -> &str {
        Preset::string_option(&self.config, "compatible_printers_condition")
    }

    /// Preset.hpp:347-353 `static PrinterTechnology printer_technology(const DynamicPrintConfig &cfg)`:
    /// ptFFF if the printer technology is not set.
    pub fn printer_technology_cfg(cfg: &DynamicPrintConfig) -> PrinterTechnology {
        // cfg.option<ConfigOptionEnum<PrinterTechnology>>("printer_technology"): the type code is
        // compared, then the value read through the class.
        match cfg.option("printer_technology") {
            Some(opt) if opt.r#type() == ConfigOptionType::CoEnum => {
                PrinterTechnology::from_int(opt.get_int().expect("ConfigOptionEnum<PrinterTechnology>::getInt"))
            }
            _ => PrinterTechnology::PtFFF,
        }
    }
    /// Preset.hpp:354
    pub fn printer_technology(&self) -> PrinterTechnology {
        Preset::printer_technology_cfg(&self.config)
    }
    /// Preset.hpp:356 `PrinterTechnology& printer_technology_ref()`: may add a new entry into the config.
    ///
    /// FIDELITY-NOTE: an enum option created in a DynamicConfig is a ConfigOptionEnumGeneric
    /// (ConfigOptionDef::create_default_option), which the C++ reaches through a static_cast to
    /// ConfigOptionEnum<PrinterTechnology>; the reference is to its int value here. An option
    /// stored as ConfigOptionEnum<PrinterTechnology> is first replaced by the generic class with
    /// the same value.
    pub fn printer_technology_ref(&mut self) -> &mut i32 {
        let existing = self
            .config
            .option("printer_technology")
            .and_then(|o| o.as_any().downcast_ref::<ConfigOptionEnum<PrinterTechnology>>())
            .map(|o| o.value.to_int());
        if let Some(v) = existing {
            self.config.set_key_value(
                "printer_technology",
                Box::new(ConfigOptionEnumGeneric::with_value(Some(ConfigOptionEnum::<PrinterTechnology>::get_enum_values()), v)),
            );
        }
        let opt = self.config.option_mut("printer_technology", true).expect("Preset: no definition").expect("printer_technology");
        assert!(opt.r#type() == ConfigOptionType::CoEnum, "option<ConfigOptionEnum<PrinterTechnology>>(\"printer_technology\", true) is null");
        &mut opt.as_any_mut().downcast_mut::<ConfigOptionEnumGeneric>().expect("ConfigOptionEnumGeneric").value
    }

    /// Preset.cpp:873-902 `void Preset::set_visible_from_appconfig(const AppConfig &app_config)`:
    /// set is_visible according to application config.
    pub fn set_visible_from_appconfig(&mut self, app_config: &AppConfig) {
        let vendor = match &self.vendor {
            None => return,
            Some(v) => v.clone(),
        };

        if self.r#type == Type::TypePrinter {
            let model = self.config.opt_string("printer_model").clone();
            let variant = self.config.opt_string("printer_variant").clone();
            if model.is_empty() || variant.is_empty() {
                return;
            }
            self.is_visible = app_config.get_variant(&vendor.id, &model, &variant);
        } else if self.r#type == Type::TypeFilament || self.r#type == Type::TypeSlaMaterial {
            let section_name = if self.r#type == Type::TypeFilament { AppConfig::SECTION_FILAMENTS } else { AppConfig::SECTION_MATERIALS };
            if app_config.has_section(section_name) {
                // Check whether this profile is marked as "installed" in PrusaSlicer.ini,
                // or whether a profile is marked as "installed", which this profile may have been renamed from.
                let installed = app_config.get_section(section_name);
                let has = |name: &str| match installed.get(name) {
                    Some(v) => !v.is_empty(),
                    None => false,
                };
                self.is_visible = has(&self.name);
                let mut i = 0;
                while !self.is_visible && i < self.renamed_from.len() {
                    self.is_visible = has(&self.renamed_from[i]);
                    i += 1;
                }
            }
        }
    }

    /// Preset.hpp:362 `void set_num_extruders(unsigned int n)`: resize the extruder specific
    /// fields, initialize them with the content of the 1st extruder.
    pub fn set_num_extruders(&mut self, n: u32) -> CfgResult<()> {
        self.config.set_num_extruders(n)
    }

    /// Preset.hpp:365 `bool operator<(const Preset &other) const`: sort lexicographically by a
    /// preset name. The preset name shall be unique across a single PresetCollection.
    pub fn operator_lt(&self, other: &Preset) -> bool {
        self.name < other.name
    }

    /// Preset.cpp:904-907 `std::string Preset::get_filament_type(std::string &display_filament_type)`
    pub fn get_filament_type(&self, display_filament_type: &mut String) -> String {
        self.config.get_filament_type_displayed(display_filament_type, 0)
    }

    /// Preset.cpp:909-924 `std::string Preset::get_printer_type(PresetBundle *preset_bundle)`:
    /// the model id of the edited printer preset.
    pub fn get_printer_type(&self, preset_bundle: Option<&PresetBundleView>) -> String {
        if let Some(preset_bundle) = preset_bundle {
            let config = &preset_bundle.printers.get_edited_preset().config;
            let mut _vendor_name = String::new();
            for (vendor_first, vendor_second) in preset_bundle.vendors.iter() {
                for vendor_model in &vendor_second.models {
                    if vendor_model.name == *config.opt_string("printer_model") {
                        _vendor_name = vendor_first.clone();
                        return vendor_model.model_id.clone();
                    }
                }
            }
        }
        String::new()
    }

    /// Preset.cpp:926-939 `std::string Preset::get_printer_name(PresetBundle *preset_bundle)`
    pub fn get_printer_name(&self, preset_bundle: Option<&PresetBundleView>) -> String {
        if let Some(preset_bundle) = preset_bundle {
            let config = &preset_bundle.printers.get_edited_preset().config;
            for (_vendor_first, vendor_second) in preset_bundle.vendors.iter() {
                for vendor_model in &vendor_second.models {
                    if vendor_model.name == *config.opt_string("printer_model") {
                        return vendor_model.name.clone();
                    }
                }
            }
        }
        String::new()
    }

    /// Preset.cpp:941-955 `std::string Preset::get_current_printer_type(PresetBundle *preset_bundle)`:
    /// the model id of this preset.
    pub fn get_current_printer_type(&self, preset_bundle: Option<&PresetBundleView>) -> String {
        if let Some(preset_bundle) = preset_bundle {
            let config = &self.config;
            let mut _vendor_name = String::new();
            for (vendor_first, vendor_second) in preset_bundle.vendors.iter() {
                for vendor_model in &vendor_second.models {
                    if vendor_model.name == *config.opt_string("printer_model") {
                        _vendor_name = vendor_first.clone();
                        return vendor_model.model_id.clone();
                    }
                }
            }
        }
        String::new()
    }

    /// Preset.cpp:957-980 `static void Preset::get_extruder_names_and_keysets(Type type, std::string& extruder_id_name, std::string& extruder_variant_name, std::set<std::string>** p_key_set1, std::set<std::string>** p_key_set2)`
    pub fn get_extruder_names_and_keysets(
        ty: Type,
        extruder_id_name: &mut String,
        extruder_variant_name: &mut String,
        p_key_set1: &mut &'static BTreeSet<String>,
        p_key_set2: &mut &'static BTreeSet<String>,
    ) {
        if ty == Type::TypePrint {
            *extruder_id_name = "print_extruder_id".to_string();
            *extruder_variant_name = "print_extruder_variant".to_string();
            *p_key_set1 = &print_options_with_variant;
            *p_key_set2 = &empty_options;
        } else if ty == Type::TypePrinter {
            *extruder_id_name = "printer_extruder_id".to_string();
            *extruder_variant_name = "printer_extruder_variant".to_string();
            *p_key_set1 = &printer_options_with_variant_1;
            *p_key_set2 = &printer_options_with_variant_2;
        } else if ty == Type::TypeFilament {
            *extruder_variant_name = "filament_extruder_variant".to_string();
            *p_key_set1 = &filament_options_with_variant;
            *p_key_set2 = &empty_options;
        } else {
            *p_key_set1 = &empty_options;
            *p_key_set2 = &empty_options;
        }
    }

    /// Preset.hpp:374 `std::string get_printer_id() const`
    pub fn get_printer_id(&self) -> String {
        match &self.vendor {
            Some(v) => v.id.clone(),
            None => String::new(),
        }
    }

    /// Preset.cpp:982-999 `bool Preset::has_lidar(PresetBundle *preset_bundle)`
    pub fn has_lidar(&self, preset_bundle: Option<&PresetBundleView>) -> bool {
        let mut has_lidar = false;
        if let Some(preset_bundle) = preset_bundle {
            let config = &preset_bundle.printers.get_edited_preset().config;
            let mut vendor_name = String::new();
            for (vendor_first, vendor_second) in preset_bundle.vendors.iter() {
                for vendor_model in &vendor_second.models {
                    if vendor_model.name == *config.opt_string("printer_model") {
                        vendor_name = vendor_first.clone();
                        break;
                    }
                }
            }
            if !vendor_name.is_empty() {
                has_lidar = vendor_name == "BBL";
            }
        }
        has_lidar
    }

    /// Preset.cpp:1001-1032 `bool Preset::is_bbl_vendor_preset(PresetBundle *preset_bundle)`.
    /// The method previously only supports to be called on preset_bundle->printers.get_edited_preset();
    /// it is extended to support call on all presets.
    pub fn is_bbl_vendor_preset(&self, preset_bundle: Option<&PresetBundleView>) -> bool {
        let mut is_bbl_vendor_preset = false;
        if let Some(preset_bundle) = preset_bundle {
            let mut config = &self.config;
            if self.r#type != Type::TypePrinter {
                if let Some(printers) = config.opt::<ConfigOptionStrings>("compatible_printers") {
                    if !printers.values.is_empty() {
                        if let Some(printer) = preset_bundle.printers.find_preset(&printers.values[0], false) {
                            config = &printer.config;
                        }
                    }
                }
            }
            if let Some(printer_model_opt) = config.opt::<ConfigOptionString>("printer_model") {
                let mut vendor_name = String::new();
                for (vendor_first, vendor_second) in preset_bundle.vendors.iter() {
                    for vendor_model in &vendor_second.models {
                        if vendor_model.name == printer_model_opt.value {
                            vendor_name = vendor_first.clone();
                            break;
                        }
                    }
                }
                if !vendor_name.is_empty() {
                    is_bbl_vendor_preset = vendor_name == "BBL";
                }
            }
        }
        is_bbl_vendor_preset
    }

    /// Preset.cpp:1034-1054 `BedType Preset::get_default_bed_type(PresetBundle* preset_bundle)`.
    /// FIDELITY-NOTE: "default_bed_type" has no definition in print_config_def; when the option
    /// was set by hand its value is `atoi`-ed and cast to BedType, which panics here for a number
    /// that is not a value of the enum.
    pub fn get_default_bed_type(&self, preset_bundle: Option<&PresetBundleView>) -> BedType {
        if self.config.has("default_bed_type") && !self.config.opt_string("default_bed_type").is_empty() {
            let str_bed_type = self.config.opt_string("default_bed_type").clone();
            let bed_type_value = crate::config::c_atoi_str(&str_bed_type);
            return BedType::from_i32(bed_type_value);
        }

        let model_id = self.get_printer_type(preset_bundle);
        if model_id == "BL-P001" || model_id == "BL-P002" || model_id == "C13" {
            return BedType::BtPC;
        } else if model_id == "C11" {
            return BedType::BtPEI;
        }
        BedType::BtPEI
    }

    /// Preset.cpp:1056-1063 `bool Preset::has_cali_lines(PresetBundle* preset_bundle)`
    pub fn has_cali_lines(&self, preset_bundle: Option<&PresetBundleView>) -> bool {
        let model_id = self.get_printer_type(preset_bundle);
        if model_id == "BL-P001" || model_id == "BL-P002" || model_id == "C13" {
            return true;
        }
        false
    }

    // ------------------------------------------------------------------
    // Static option-name vectors (Preset.cpp:1065-1345)
    // ------------------------------------------------------------------

    /// Preset.cpp:1326
    pub fn print_options() -> &'static Vec<String> {
        static V: LazyLock<Vec<String>> = LazyLock::new(|| strings(S_PRESET_PRINT_OPTIONS));
        &V
    }

    /// Preset.cpp:1327
    pub fn filament_options() -> &'static Vec<String> {
        static V: LazyLock<Vec<String>> = LazyLock::new(|| strings(S_PRESET_FILAMENT_OPTIONS));
        &V
    }

    /// Preset.cpp:1328
    pub fn machine_limits_options() -> &'static Vec<String> {
        static V: LazyLock<Vec<String>> = LazyLock::new(|| strings(S_PRESET_MACHINE_LIMITS_OPTIONS));
        &V
    }

    /// Preset.cpp:1329-1331: the nozzle options of a printer profile will be adjusted to match
    /// the size of the nozzle_diameter vector.
    pub fn nozzle_options() -> &'static Vec<String> {
        print_config_def().extruder_option_keys()
    }

    /// Preset.cpp:1332
    pub fn sla_print_options() -> &'static Vec<String> {
        static V: LazyLock<Vec<String>> = LazyLock::new(|| strings(S_PRESET_SLA_PRINT_OPTIONS));
        &V
    }

    /// Preset.cpp:1333
    pub fn sla_material_options() -> &'static Vec<String> {
        static V: LazyLock<Vec<String>> = LazyLock::new(|| strings(S_PRESET_SLA_MATERIAL_OPTIONS));
        &V
    }

    /// Preset.cpp:1334
    pub fn sla_printer_options() -> &'static Vec<String> {
        static V: LazyLock<Vec<String>> = LazyLock::new(|| strings(S_PRESET_SLA_PRINTER_OPTIONS));
        &V
    }

    /// Preset.cpp:1336-1345 `const std::vector<std::string>& Preset::printer_options()`:
    /// printer options contain the nozzle options.
    pub fn printer_options() -> &'static Vec<String> {
        static S_OPTS: LazyLock<Vec<String>> = LazyLock::new(|| {
            let mut opts = strings(S_PRESET_PRINTER_OPTIONS);
            opts.extend(strings(S_PRESET_MACHINE_LIMITS_OPTIONS));
            opts.extend(Preset::nozzle_options().iter().cloned());
            opts
        });
        &S_OPTS
    }

    /// Preset.cpp:444-447 `const std::string& Preset::suffix_modified()`
    pub fn suffix_modified() -> String {
        g_suffix_modified_get()
    }

    /// Preset.cpp:449-452 `void Preset::update_suffix_modified(const std::string& new_suffix_modified)`
    pub fn update_suffix_modified(new_suffix_modified: &str) {
        let mut guard = G_SUFFIX_MODIFIED.write().unwrap();
        *guard = Some(new_suffix_modified.to_string());
    }

    /// Preset.cpp:453-460 `std::string Preset::remove_suffix_modified(const std::string &name)`:
    /// remove an optional "(modified)" suffix from a name. This converts a UI name to a unique
    /// preset identifier. (The C++ tests the start of the name.)
    pub fn remove_suffix_modified(name: &str) -> String {
        let g_suffix_modified = g_suffix_modified_get();
        if name.as_bytes().starts_with(g_suffix_modified.as_bytes()) {
            String::from_utf8_lossy(&name.as_bytes()[g_suffix_modified.len()..]).into_owned()
        } else {
            name.to_string()
        }
    }

    /// Preset.cpp:462-505 `void Preset::normalize(DynamicPrintConfig &config)`: update new
    /// extruder fields at the printer profile.
    pub fn normalize(config: &mut DynamicPrintConfig) -> CfgResult<()> {
        // BBS: dynamic_cast<const ConfigOptionFloats*>(config.option("filament_diameter"))
        let filament_diameter_size =
            config.option("filament_diameter").and_then(|o| o.as_any().downcast_ref::<ConfigOptionFloats>()).map(|o| o.values.len());

        if let Some(n) = filament_diameter_size {
            // This config contains single or multiple filament presets.
            // Ensure that the filament preset vector options contain the correct number of values.
            let defaults = static_print_config_defaults("FullPrintConfig");
            for key in Preset::filament_options() {
                if key == "compatible_prints" || key == "compatible_printers" {
                    continue;
                }
                if filament_options_with_variant.contains(key) {
                    continue;
                }
                if filament_dev_options.contains(key) {
                    continue;
                }
                if let Some(opt) = config.option_mut(key, false)? {
                    if opt.is_vector() {
                        opt.as_vector_base_mut().expect("ConfigOptionVectorBase").resize(n, defaults.option(key))?;
                    }
                }
            }
            // The following keys are mandatory for the UI, but they are not part of FullPrintConfig, therefore they are handled separately.
            for key in ["filament_settings_id"] {
                if let Some(opt) = config.option_mut(key, false)? {
                    if opt.r#type() == ConfigOptionType::CoStrings {
                        opt.as_any_mut().downcast_mut::<ConfigOptionStrings>().expect("ConfigOptionStrings").values.resize(n, String::new());
                    }
                }
            }
        }

        handle_legacy_sla(config)?;

        // Repair invalid filament extrusion parameters carried by corrupted/legacy project files,
        // before they propagate NaN into slicing speeds or extrusion amounts.
        config.repair_invalid_filament_extrusion_parameters();
        Ok(())
    }

    /// Preset.cpp:507-521 `std::string Preset::remove_invalid_keys(DynamicPrintConfig &config, const DynamicPrintConfig &default_config)`:
    /// report configuration fields, which are misplaced into a wrong group, remove them from the config.
    pub fn remove_invalid_keys(config: &mut DynamicPrintConfig, default_config: &DynamicPrintConfig) -> String {
        let mut incorrect_keys = String::new();
        for key in config.keys() {
            if !default_config.has(&key) {
                if incorrect_keys.is_empty() {
                    incorrect_keys = key.clone();
                } else {
                    incorrect_keys += ", ";
                    incorrect_keys += &key;
                }
                config.erase(&key);
            }
        }
        incorrect_keys
    }
}

/// `atoll` of a text (Preset.cpp uses `std::atoll(time.c_str())`).
fn c_atoll(s: &str) -> i64 {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && is_c_space(b[i]) {
        i += 1;
    }
    let mut neg = false;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        neg = b[i] == b'-';
        i += 1;
    }
    let mut v: i64 = 0;
    while i < b.len() && b[i].is_ascii_digit() {
        v = v.wrapping_mul(10).wrapping_add((b[i] - b'0') as i64);
        i += 1;
    }
    if neg {
        v.wrapping_neg()
    } else {
        v
    }
}

fn strings(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| s.to_string()).collect()
}

/// The loop body of Preset::save (Preset.cpp:700-721) and of
/// PresetCollection::get_preset_differed_for_save (Preset.cpp:1640-1661): the option `option`
/// of `src` copied into `temp_config`, a vector of more than one value of a nullable option with
/// the values equal to the parent's set to nil.
fn set_option_for_save(
    temp_config: &mut DynamicPrintConfig,
    src: &DynamicPrintConfig,
    parent_config: &DynamicPrintConfig,
    option: &str,
    key_set1: &BTreeSet<String>,
    key_set2: &BTreeSet<String>,
) -> CfgResult<()> {
    let opt_src = src.option(option).expect("config.option(option) is null");
    let opt_dst = temp_config.option_mut(option, true)?.expect("temp_config.option(option, true) is null");
    if opt_dst.is_scalar() || !opt_dst.nullable() || is_filament_extruder_override_key(option) {
        opt_dst.set(opt_src)?;
    } else {
        let opt_vec_src = opt_src.as_vector_base().expect("static_cast<ConfigOptionVectorBase*>(opt_src)");
        let opt_vec_inherit = || parent_config.option(option).expect("parent_config->option(option) is null");
        if opt_vec_src.size() == 1 {
            opt_dst.set(opt_src)?;
        } else if key_set1.contains(option) {
            opt_dst.as_vector_base_mut().expect("ConfigOptionVectorBase").set_with_nil(opt_src, opt_vec_inherit(), 1)?;
        } else if key_set2.contains(option) {
            opt_dst.as_vector_base_mut().expect("ConfigOptionVectorBase").set_with_nil(opt_src, opt_vec_inherit(), 2)?;
        } else {
            opt_dst.set(opt_src)?;
        }
    }
    Ok(())
}

// ============================================================================
// Preset.cpp:1065-1324 option-name vectors
// ============================================================================

/// Preset.cpp:1065-1142 `static std::vector<std::string> s_Preset_print_options`
pub static S_PRESET_PRINT_OPTIONS: &[&str] = &[
    "layer_height", "initial_layer_print_height", "wall_loops", "slice_closing_radius", "spiral_mode", "spiral_mode_smooth", "spiral_mode_max_xy_smoothing", "slicing_mode",
    "top_shell_layers", "top_shell_thickness", "bottom_shell_layers", "bottom_shell_thickness", "ensure_vertical_shell_thickness", "reduce_crossing_wall", "detect_thin_wall",
    "detect_overhang_wall", "top_color_penetration_layers", "bottom_color_penetration_layers",
    "infill_instead_top_bottom_surfaces",
    "smooth_speed_discontinuity_area","smooth_coefficient", "seam_position", "seam_placement_away_from_overhangs", "wall_sequence", "is_infill_first", "sparse_infill_density", "fill_multiline",
    "sparse_infill_pattern", "conformal_infill", "conformal_stagger", "conformal_link_keep_layers", "conformal_link_flip_layers", "conformal_pole", "conformal_ray_count", "conformal_hub_radius", "sparse_infill_anchor", "sparse_infill_anchor_max", "top_surface_pattern", "monotonic_travel_into_wall",
    "locked_skin_infill_pattern", "locked_skeleton_infill_pattern",
    "bottom_surface_pattern", "internal_solid_infill_pattern", "sub_top_surface_pattern", "infill_direction", "bridge_angle", "infill_shift_step", "skeleton_infill_density", "infill_lock_depth", "skin_infill_depth", "skin_infill_density",
    "infill_rotate_step","top_surface_density", "bottom_surface_density",
    "symmetric_infill_y_axis","sparse_infill_lattice_angle_1","sparse_infill_lattice_angle_2",
    "minimum_sparse_infill_area", "reduce_infill_retraction_mode", "ironing_pattern", "ironing_type",
    "ironing_flow", "ironing_speed", "ironing_spacing","ironing_direction", "ironing_inset",
    "enable_support_ironing","support_ironing_pattern","support_ironing_speed",
    "support_ironing_flow","support_ironing_spacing","support_ironing_inset","support_ironing_direction",
    "max_travel_detour_distance", "avoid_crossing_wall_includes_support",
    "fuzzy_skin", "fuzzy_skin_thickness", "fuzzy_skin_point_distance",
    "fuzzy_skin_first_layer", "fuzzy_skin_noise_type", "fuzzy_skin_scale", "fuzzy_skin_octaves", "fuzzy_skin_persistence", "fuzzy_skin_mode",
    // #ifdef HAS_PRESSURE_EQUALIZER (not defined, PrintConfig.hpp:34): "max_volumetric_extrusion_rate_slope_positive", "max_volumetric_extrusion_rate_slope_negative",
    "inner_wall_speed", "outer_wall_speed", "sparse_infill_speed", "internal_solid_infill_speed",
    "top_surface_speed", "support_speed", "support_object_xy_distance", "support_object_first_layer_gap","support_interface_speed",
    "bridge_speed", "gap_infill_speed", "travel_speed", "travel_speed_z", "initial_layer_speed", "outer_wall_acceleration",
    "initial_layer_acceleration", "top_surface_acceleration", "default_acceleration", "travel_acceleration", "travel_short_distance_acceleration", "initial_layer_travel_acceleration", "inner_wall_acceleration", "sparse_infill_acceleration",
    "accel_to_decel_enable", "accel_to_decel_factor", "skirt_loops", "skirt_distance",
    "skirt_per_object", "skirt_height", "draft_shield",
    "brim_width", "brim_object_gap", "brim_type", "enable_support", "support_type", "support_threshold_angle", "enforce_support_layers",
    "raft_layers", "raft_first_layer_density", "raft_first_layer_expansion", "raft_contact_distance", "raft_expansion",
    "support_base_pattern", "support_base_pattern_spacing", "support_expansion", "support_style",
    // BBS
    "print_extruder_id", "print_extruder_variant", "independent_support_layer_height",
    "top_z_overrides_xy_distance",
    "support_angle", "support_interface_top_layers", "support_interface_bottom_layers",
    "support_interface_pattern", "support_interface_spacing", "support_interface_loop_pattern",
    "support_top_z_distance", "support_on_build_plate_only","support_critical_regions_only", "support_remove_small_overhang",
    "bridge_no_support", "thick_bridges", "max_bridge_length", "print_sequence",
    "filename_format", "wall_filament", "support_bottom_z_distance",
    "sparse_infill_filament", "solid_infill_filament", "support_filament", "support_interface_filament","support_interface_not_for_body",
    "ooze_prevention", "standby_temperature_delta", "interface_shells", "line_width", "initial_layer_line_width",
    "initial_layer_infill_line_width", "inner_wall_line_width",
    "outer_wall_line_width", "sparse_infill_line_width", "internal_solid_infill_line_width",
    "skin_infill_line_width","skeleton_infill_line_width",
    "top_surface_line_width", "support_line_width", "infill_wall_overlap", "bridge_flow", "counterbore_hole_bridging",
    "elefant_foot_compensation", "xy_contour_compensation", "xy_hole_compensation", "resolution", "enable_prime_tower", "prime_tower_enable_framework",
    "prime_tower_width", "prime_tower_brim_width", "prime_tower_skip_points","prime_tower_max_speed","prime_tower_feedrate_factor","prime_tower_tail_speed_factor","prime_tower_tail_slowdown_volume","enable_tower_interface_features",
    "prime_tower_rib_wall","prime_tower_extra_rib_length","prime_tower_rib_width","prime_tower_fillet_wall","prime_tower_infill_gap","prime_tower_lift_speed","prime_tower_lift_height",
    "prime_tower_acceleration","prime_tower_line_width","prime_tower_ramming_line_width","prime_tower_wipe_speed","prime_tower_wipe_distance",
    "prime_tower_flat_ironing","prime_tower_line_ironing","prime_tower_ironing_speed",
    "prime_tower_side_scrape","prime_tower_side_scrape_face","prime_tower_side_scrape_length","prime_tower_side_scrape_height","prime_tower_side_scrape_speed","prime_tower_side_scrape_wall_gap",
    "prime_tower_side_scrape_point_spacing","prime_tower_side_scrape_along_length",
    "enable_circle_compensation", "circle_compensation_manual_offset", "apply_scarf_seam_on_circles",
    "wipe_tower_no_sparse_layers", "compatible_printers", "compatible_printers_condition", "inherits",
    "flush_into_infill", "flush_into_objects", "flush_into_support","process_notes", "enable_mixed_color_sublayer",
    // BBS
     "tree_support_branch_angle", "tree_support_wall_count", "tree_support_branch_distance", "tree_support_branch_diameter",
    "tree_support_branch_diameter_angle", "tree_support_tip_diameter",
     "detect_narrow_internal_solid_infill",
     "gcode_add_line_number", "enable_arc_fitting", "precise_z_height", "infill_combination", /*"adaptive_layer_height",*/
     "support_bottom_interface_spacing", "enable_overhang_speed", "overhang_1_4_speed", "overhang_2_4_speed", "overhang_3_4_speed", "overhang_4_4_speed", "overhang_totally_speed",
    "enable_height_slowdown","slowdown_start_height", "slowdown_start_speed", "slowdown_start_acc", "slowdown_end_height", "slowdown_end_speed", "slowdown_end_acc",
     "initial_layer_infill_speed", "top_one_wall_type", "top_area_threshold", "only_one_wall_first_layer",
     "timelapse_type", "internal_bridge_support_thickness",
     "wall_generator", "wall_transition_length", "wall_transition_filter_deviation", "wall_transition_angle",
     "wall_distribution_count", "min_feature_size", "min_bead_width", "post_process",
    "seam_gap", "wipe_speed", "top_solid_infill_flow_ratio", "initial_layer_flow_ratio",
    "default_jerk", "outer_wall_jerk", "inner_wall_jerk", "infill_jerk", "top_surface_jerk", "initial_layer_jerk", "travel_jerk",
    "filter_out_gap_fill", "mmu_segmented_region_max_width", "mmu_segmented_region_interlocking_depth",
    "small_perimeter_speed", "small_perimeter_threshold", "z_direction_outwall_speed_continuous",
    "vertical_shell_speed","detect_floating_vertical_shell", "enable_wrapping_detection", "enable_order_independent_overlap_carving",
     // calib
    "print_flow_ratio",
    //Orca
    "exclude_object", "override_filament_scarf_seam_setting", "seam_slope_type", "seam_slope_conditional", "scarf_angle_threshold",
    "seam_slope_start_height", "seam_slope_entire_loop", "seam_slope_min_length",
    "seam_slope_steps", "seam_slope_inner_walls", "role_base_wipe_speed", "seam_slope_gap", "precise_outer_wall",
    "interlocking_beam", "interlocking_orientation", "interlocking_beam_layer_count", "interlocking_depth", "interlocking_boundary_avoidance", "interlocking_beam_width", "embedding_wall_into_infill",
    "alternate_extra_wall", "periodic_modifier", "periodic_modifier_skip_layers", "periodic_modifier_apply_layers", "modifier_ignore_infill" ];


/// Preset.cpp:1144-1199 `static std::vector<std::string> s_Preset_filament_options`
pub static S_PRESET_FILAMENT_OPTIONS: &[&str] = &[/*"filament_colour", */ "default_filament_colour", "required_nozzle_HRC", "filament_diameter", "volumetric_speed_coefficients", "filament_type",
                                                          "filament_soluble", "filament_is_support", "filament_printable", "filament_extruder_compatibility", "filament_scarf_seam_type", "filament_scarf_height",
                                                          "filament_scarf_gap", "filament_scarf_length",
    "filament_max_volumetric_speed", "impact_strength_z", "filament_ramming_volumetric_speed", "filament_ramming_volumetric_speed_nc", "filament_adaptive_volumetric_speed", "filament_prime_tower_speed_factor_curve",
    "filament_flow_ratio", "filament_density", "filament_adhesiveness_category", "filament_metal_stickiness", "filament_cost", "filament_minimal_purge_on_wipe_tower",
    "nozzle_temperature", "nozzle_temperature_initial_layer",
    // BBS
    "cool_plate_temp", "eng_plate_temp", "hot_plate_temp", "textured_plate_temp", "cool_plate_temp_initial_layer", "eng_plate_temp_initial_layer", "hot_plate_temp_initial_layer","textured_plate_temp_initial_layer",
    "supertack_plate_temp_initial_layer", "supertack_plate_temp",
    "circle_compensation_speed", "counter_coef_1", "counter_coef_2", "counter_coef_3", "hole_coef_1", "hole_coef_2", "hole_coef_3",
    "counter_limit_min", "counter_limit_max", "hole_limit_min", "hole_limit_max", "diameter_limit",
    // "bed_type",
    //BBS:temperature_vitrification
    "temperature_vitrification", "reduce_fan_stop_start_freq", "slow_down_for_layer_cooling", "no_slow_down_for_cooling_on_outwalls", "cooling_slowdown_logic", "cooling_perimeter_transition_distance", "fan_min_speed","filament_ramming_travel_time","filament_pre_cooling_temperature","filament_ramming_travel_time_nc","filament_pre_cooling_temperature_nc",
    "fan_max_speed", "enable_overhang_bridge_fan", "overhang_fan_speed", "ironing_fan_speed", "pre_start_fan_time", "overhang_fan_threshold", "overhang_threshold_participating_cooling","close_fan_the_first_x_layers", "first_x_layer_part_fan_speed", "close_additional_fan_first_x_layers", "close_optional_fan_first_x_layers", "first_x_layer_fan_speed", "first_x_layer_optional_fan_speed", "full_fan_speed_layer", "additional_fan_full_speed_layer", "optional_fan_full_speed_layer", "fan_cooling_layer_time", "slow_down_layer_time", "slow_down_min_speed",
    "filament_start_gcode", "filament_end_gcode",
    //exhaust fan control
    "activate_air_filtration","during_print_exhaust_fan_speed","complete_print_exhaust_fan_speed",
    // Retract overrides
    "filament_retraction_length", "filament_z_hop", "filament_z_hop_types", "filament_retraction_speed", "filament_deretraction_speed", "filament_retract_length_nc", "filament_retract_length_toolchange", "filament_retract_restart_extra_toolchange", "filament_retract_restart_extra", "filament_retraction_minimum_travel",
    "filament_retract_when_changing_layer", "filament_wipe", "filament_retract_before_wipe",
    // Profile compatibility
    "filament_vendor", "compatible_prints", "compatible_prints_condition", "compatible_printers", "compatible_printers_condition", "inherits",
    //BBS
    "filament_wipe_distance", "additional_cooling_fan_speed", "optional_cooling_fan_speed",
    "nozzle_temperature_range_low", "nozzle_temperature_range_high",
    "filament_extruder_variant",
    //OrcaSlicer
    "enable_pressure_advance", "pressure_advance", "chamber_temperatures", "fins_extrude_safe_temp", "filament_notes",
    "filament_long_retractions_when_cut","filament_retraction_distances_when_cut","filament_shrink", "filament_velocity_adaptation_factor",
    //BBS filament change length while the extruder color
    "filament_change_length","filament_change_length_nc","filament_idle_travel_cooling_enabled","filament_prime_volume","filament_prime_volume_nc","filament_flush_volumetric_speed","filament_flush_temp","filament_flush_temp_fast",
    "long_retractions_when_ec", "retraction_distances_when_ec",
    "filament_enable_overhang_speed",
    "filament_bridge_speed",
    "filament_overhang_1_4_speed",
    "filament_overhang_2_4_speed",
    "filament_overhang_3_4_speed",
    "filament_overhang_4_4_speed",
    "filament_overhang_totally_speed",
    "override_process_overhang_speed",
    "filament_pre_heating_enabled",
    "filament_preheat_temperature_delta",
    "filament_cooling_before_tower",
    "filament_tower_interface_pre_extrusion_dist",
    "filament_tower_interface_pre_extrusion_length",
    "filament_tower_ironing_area",
    "filament_tower_interface_purge_volume",
    "filament_tower_interface_print_temp",
    "min_temperature_when_ec",
    //ams chamber
    "filament_dev_ams_drying_ams_limitations", "filament_dev_ams_drying_temperature", "filament_dev_ams_drying_time", "filament_dev_ams_drying_heat_distortion_temperature",
    "filament_dev_chamber_drying_bed_temperature", "filament_dev_chamber_drying_time",
    "filament_dev_drying_softening_temperature", "filament_dev_drying_cooling_temperature"

];


/// Preset.cpp:1201-1208 `static std::vector<std::string> s_Preset_machine_limits_options`
pub static S_PRESET_MACHINE_LIMITS_OPTIONS: &[&str] = &[
    "machine_max_acceleration_extruding", "machine_max_acceleration_retracting", "machine_max_acceleration_travel",
    "machine_max_acceleration_x", "machine_max_acceleration_y", "machine_max_acceleration_z", "machine_max_acceleration_e",
    "machine_max_speed_x", "machine_max_speed_y", "machine_max_speed_z", "machine_max_speed_e",
    "machine_min_extruding_rate", "machine_min_travel_rate",
    "machine_max_jerk_x", "machine_max_jerk_y", "machine_max_jerk_z", "machine_max_jerk_e",
    "machine_max_force_Y", "machine_bed_mass_Y","machine_max_printed_mass",
];


/// Preset.cpp:1210-1236 `static std::vector<std::string> s_Preset_printer_options`
pub static S_PRESET_PRINTER_OPTIONS: &[&str] = &[
    "printer_technology",
    "printable_area", "extruder_printable_area", "bed_exclude_area", "bed_heat_soak_area", "bed_custom_texture", "bed_custom_model", "gcode_flavor",
    "single_extruder_multi_material", "machine_start_gcode", "machine_end_gcode","printing_by_object_gcode","before_layer_change_gcode", "layer_change_gcode", "time_lapse_gcode", "wrapping_detection_gcode", "change_filament_gcode",
    "printer_model", "printer_variant", "printer_extruder_id", "printer_extruder_variant", "extruder_variant_list", "default_nozzle_volume_type",
    "printable_height", "extruder_printable_height", "extruder_clearance_dist_to_rod",  "extruder_clearance_max_radius","extruder_clearance_height_to_lid", "extruder_clearance_height_to_rod",
    "nozzle_height", "master_extruder_id",
    "default_print_profile", "inherits",
    "silent_mode",
    // BBS
    "scan_first_layer", "wrapping_detection_layers", "wrapping_exclude_area", "machine_load_filament_time", "machine_unload_filament_time",
    "ams_filament_load_time_ams", "ams_filament_load_time_ams_lite", "ams_filament_load_time_n3f_s",
    "ams_filament_unload_time_ams", "ams_filament_unload_time_ams_lite", "ams_filament_unload_time_n3f_s", "default_ams_type", "machine_pause_gcode", "template_custom_gcode","machine_hotend_change_time",
    "auxiliary_fan", "sync_auxiliary_fan_to_second_fan", "aux_fan_independent_control", "aux_fan_standard_side", "fan_direction", "nozzle_volume","upward_compatible_machine", "z_hop_types","support_chamber_temp_control","support_air_filtration","support_cooling_filter","cooling_filter_enabled","printer_structure","farthest_point_timelapse","default_wipe_tower_pos","thumbnail_size",
    "best_object_pos", "head_wrap_detect_zone","printer_notes","print_in_clockwise",
    "enable_long_retraction_when_cut","long_retractions_when_cut","retraction_distances_when_cut",
    //OrcaSlicer
    "host_type", "print_host", "printhost_apikey",
    "print_host_webui",
    "printhost_cafile","printhost_port","printhost_authorization_type",
    "printhost_user", "printhost_password", "printhost_ssl_ignore_revoke",
    "use_relative_e_distances", "extruder_type","use_firmware_retraction",
    "grab_length","machine_switch_extruder_time","hotend_cooling_rate","hotend_cooling_rate_on_rack","hotend_heating_rate","enable_pre_heating", "support_object_skip_flush","physical_extruder_map",
    "bed_temperature_formula","machine_prepare_compensation_time", "nozzle_flush_dataset",
    "group_algo_with_time","extruder_max_nozzle_count", "ams_nozzle_idx", "handle_hotend_as_extruder", "min_temperature_when_ec","support_fast_purge_mode",
    "printer_nozzle_diameter_mode"
];


/// Preset.cpp:1238-1283 `static std::vector<std::string> s_Preset_sla_print_options`
pub static S_PRESET_SLA_PRINT_OPTIONS: &[&str] = &[
    "layer_height",
    "faded_layers",
    "supports_enable",
    "support_head_front_diameter",
    "support_head_penetration",
    "support_head_width",
    "support_pillar_diameter",
    "support_small_pillar_diameter_percent",
    "support_max_bridges_on_pillar",
    "support_pillar_connection_mode",
    "support_buildplate_only",
    "support_pillar_widening_factor",
    "support_base_diameter",
    "support_base_height",
    "support_base_safety_distance",
    "support_critical_angle",
    "support_max_bridge_length",
    "support_max_pillar_link_distance",
    "support_object_elevation",
    "support_points_density_relative",
    "support_points_minimal_distance",
    "slice_closing_radius",
    "pad_enable",
    "pad_wall_thickness",
    "pad_wall_height",
    "pad_brim_size",
    "pad_max_merge_distance",
    // "pad_edge_radius",
    "pad_wall_slope",
    "pad_object_gap",
    "pad_around_object",
    "pad_around_object_everywhere",
    "pad_object_connector_stride",
    "pad_object_connector_width",
    "pad_object_connector_penetration",
    "hollowing_enable",
    "hollowing_min_thickness",
    "hollowing_quality",
    "hollowing_closing_distance",
    "filename_format",
    "default_sla_print_profile",
    "compatible_printers",
    "compatible_printers_condition",
    "inherits"
];


/// Preset.cpp:1285-1304 `static std::vector<std::string> s_Preset_sla_material_options`
pub static S_PRESET_SLA_MATERIAL_OPTIONS: &[&str] = &[
    "material_colour",
    "material_type",
    "initial_layer_height",
    "bottle_cost",
    "bottle_volume",
    "bottle_weight",
    "material_density",
    "exposure_time",
    "initial_exposure_time",
    "material_correction",
    "material_correction_x",
    "material_correction_y",
    "material_correction_z",
    "material_vendor",
    "material_print_speed",
    "default_sla_material_profile",
    "compatible_prints", "compatible_prints_condition",
    "compatible_printers", "compatible_printers_condition", "inherits"
];


/// Preset.cpp:1306-1324 `static std::vector<std::string> s_Preset_sla_printer_options`
pub static S_PRESET_SLA_PRINTER_OPTIONS: &[&str] = &[
    "printer_technology",
    "printable_area","bed_custom_texture", "bed_custom_model", "printable_height",
    "display_width", "display_height", "display_pixels_x", "display_pixels_y",
    "display_mirror_x", "display_mirror_y",
    "display_orientation",
    "fast_tilt_time", "slow_tilt_time", "area_fill",
    "relative_correction",
    "relative_correction_x",
    "relative_correction_y",
    "relative_correction_z",
    "absolute_correction",
    "elefant_foot_compensation",
    "elefant_foot_min_width",
    "gamma_correction",
    "min_exposure_time", "max_exposure_time",
    "min_initial_exposure_time", "max_initial_exposure_time",
    "inherits"
];

/// Preset.cpp:3683-3697 `static std::vector<std::string> s_PhysicalPrinter_opts`
pub static S_PHYSICAL_PRINTER_OPTS: &[&str] = &[
    "preset_name", // temporary option to compatibility with older Slicer
    "preset_names",
    "printer_technology",
    "host_type",
    "print_host",
    "printhost_apikey",
    "printhost_cafile",
    "printhost_port",
    "printhost_authorization_type",
    // HTTP digest authentization (RFC 2617)
    "printhost_user",
    "printhost_password",
    "printhost_ssl_ignore_revoke"
];



// ============================================================================
// Compatibility (Preset.cpp:784-871)
// ============================================================================

/// `PlaceholderParser::evaluate_boolean_expression(templ, config, config_override)`.
pub type BooleanExpressionEvaluator = fn(&str, &DynamicConfig, Option<&DynamicConfig>) -> Result<bool, String>;

static BOOLEAN_EXPRESSION_EVALUATOR: RwLock<Option<BooleanExpressionEvaluator>> = RwLock::new(None);

/// The compatibility conditions are evaluated by the function installed here (tests install their
/// own); with none installed, by [`placeholder_parser_evaluate_boolean_expression`], the port of
/// `PlaceholderParser::evaluate_boolean_expression`. An evaluation that fails takes the path the C++
/// takes when the expression throws: compatible.
pub fn set_boolean_expression_evaluator(evaluator: Option<BooleanExpressionEvaluator>) {
    *BOOLEAN_EXPRESSION_EVALUATOR.write().unwrap() = evaluator;
}

/// PlaceholderParser.cpp:1554-1562 `PlaceholderParser::evaluate_boolean_expression(templ, config, config_override)`,
/// the `what()` of the exception it throws as the error (every exception it throws is a
/// `std::runtime_error`, which Preset.cpp:795 / 832 catch).
pub fn placeholder_parser_evaluate_boolean_expression(
    templ: &str,
    config: &DynamicConfig,
    config_override: Option<&DynamicConfig>,
) -> Result<bool, String> {
    crate::placeholder_parser::PlaceholderParser::evaluate_boolean_expression(templ, config, config_override)
        .map_err(|e| e.what().to_string())
}

fn evaluate_boolean_expression(templ: &str, config: &DynamicConfig, config_override: Option<&DynamicConfig>) -> Result<bool, String> {
    match *BOOLEAN_EXPRESSION_EVALUATOR.read().unwrap() {
        Some(f) => f(templ, config, config_override),
        // Gate PRESET_PLACEHOLDER_BOOLEAN_EXPRESSION (faithful, on): =0 restores the earlier arm, where
        // no evaluator was available and every condition failed (compatible).
        None if crate::faithful_gate("PRESET_PLACEHOLDER_BOOLEAN_EXPRESSION") => {
            placeholder_parser_evaluate_boolean_expression(templ, config, config_override)
        }
        None => Err("PlaceholderParser::evaluate_boolean_expression is not available".to_string()),
    }
}

/// `dynamic_cast<const ConfigOptionStrings*>(config.option(key))`
fn strings_option<'a>(config: &'a DynamicPrintConfig, key: &str) -> Option<&'a ConfigOptionStrings> {
    config.option(key).and_then(|o| o.as_any().downcast_ref::<ConfigOptionStrings>())
}

/// Preset.cpp:784-804 `bool is_compatible_with_print(const PresetWithVendorProfile &preset, const PresetWithVendorProfile &active_print, const PresetWithVendorProfile &active_printer)`
pub fn is_compatible_with_print(
    preset: &PresetWithVendorProfile,
    active_print: &PresetWithVendorProfile,
    active_printer: &PresetWithVendorProfile,
) -> bool {
    if preset.vendor.is_some() && !vendor_ptr_eq(preset.vendor, active_printer.vendor) {
        // The current profile has a vendor assigned and it is different from the active print's vendor.
        return false;
    }
    let condition = preset.preset.compatible_prints_condition();
    let compatible_prints = strings_option(&preset.preset.config, "compatible_prints");
    let has_compatible_prints = compatible_prints.map_or(false, |c| !c.values.is_empty());
    if !has_compatible_prints && !condition.is_empty() {
        match evaluate_boolean_expression(condition, &active_print.preset.config.base, None) {
            Ok(b) => return b,
            Err(_err) => {
                //FIXME in case of an error, return "compatible with everything".
                return true;
            }
        }
    }
    preset.preset.is_default
        || active_print.preset.name.is_empty()
        || !has_compatible_prints
        || compatible_prints.unwrap().values.iter().any(|n| *n == active_print.preset.name)
}

/// Preset.cpp:806-819 `bool is_compatible_with_parent_printer(const PresetWithVendorProfile& preset, const PresetWithVendorProfile& active_printer)`.
/// BBS: If one filament or process preset is compatible with one system printer preset, then it
/// is compatible with all user printer presets inherited from this system printer preset,
/// because printer_model and nozzle_diameter in BBL system machine preset can't be changed by user.
pub fn is_compatible_with_parent_printer(preset: &PresetWithVendorProfile, active_printer: &PresetWithVendorProfile) -> bool {
    let compatible_printers = strings_option(&preset.preset.config, "compatible_printers");
    let has_compatible_printers = compatible_printers.map_or(false, |c| !c.values.is_empty());
    //BBS: FIXME only check the parent now, but should check grand-parent as well.
    has_compatible_printers && compatible_printers.unwrap().values.iter().any(|n| n == active_printer.preset.inherits())
}

/// Preset.cpp:821-843 `bool is_compatible_with_printer(const PresetWithVendorProfile &preset, const PresetWithVendorProfile &active_printer, const DynamicPrintConfig *extra_config)`
pub fn is_compatible_with_printer(
    preset: &PresetWithVendorProfile,
    active_printer: &PresetWithVendorProfile,
    extra_config: Option<&DynamicPrintConfig>,
) -> bool {
    if preset.vendor.is_some() && !vendor_ptr_eq(preset.vendor, active_printer.vendor) {
        // The current profile has a vendor assigned and it is different from the active print's vendor.
        return false;
    }
    let condition = preset.preset.compatible_printers_condition();
    let compatible_printers = strings_option(&preset.preset.config, "compatible_printers");
    let has_compatible_printers = compatible_printers.map_or(false, |c| !c.values.is_empty());
    if !has_compatible_printers && !condition.is_empty() {
        match evaluate_boolean_expression(condition, &active_printer.preset.config.base, extra_config.map(|c| &c.base)) {
            Ok(b) => return b,
            Err(_err) => {
                //FIXME in case of an error, return "compatible with everything".
                return true;
            }
        }
    }
    preset.preset.is_default
        || active_printer.preset.name.is_empty()
        || !has_compatible_printers
        || compatible_printers.unwrap().values.iter().any(|n| *n == active_printer.preset.name)
        //BBS
        || (!active_printer.preset.is_system && is_compatible_with_parent_printer(preset, active_printer))
}

/// Preset.cpp:845-853 `bool is_compatible_with_printer(const PresetWithVendorProfile &preset, const PresetWithVendorProfile &active_printer)`
pub fn is_compatible_with_printer_2(preset: &PresetWithVendorProfile, active_printer: &PresetWithVendorProfile) -> bool {
    let mut config = DynamicPrintConfig::new();
    config.set_key_value("printer_preset", Box::new(ConfigOptionString::with_value(active_printer.preset.name.clone())));
    if let Some(opt) = active_printer.preset.config.option("nozzle_diameter") {
        // static_cast<const ConfigOptionFloatsNullable*>(opt)->values.size()
        let n = opt.as_vector_base().expect("static_cast<const ConfigOptionFloatsNullable*>").size();
        config.set_key_value("num_extruders", Box::new(ConfigOptionInt::with_value(n as i32)));
    }
    is_compatible_with_printer(preset, active_printer, Some(&config))
}

/// Preset.cpp:855-871 `bool is_compatible_with_printer_names(const PresetWithVendorProfile &preset, const std::vector<std::string> &printer_names)`.
/// BBS multi-diameter: accept a preset when its compatible_printers list references any of the
/// given printer preset names (e.g. sibling nozzle-diameter variants selected via "set nozzle diameter").
pub fn is_compatible_with_printer_names(preset: &PresetWithVendorProfile, printer_names: &[String]) -> bool {
    if printer_names.is_empty() {
        return false;
    }
    let compatible_printers = match strings_option(&preset.preset.config, "compatible_printers") {
        Some(c) if !c.values.is_empty() => c,
        _ => return false,
    };
    for name in printer_names {
        if compatible_printers.values.iter().any(|n| n == name) {
            return true;
        }
    }
    false
}

/// Preset.hpp:423-430 `enum class PresetSelectCompatibleType`
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PresetSelectCompatibleType {
    // Never select a compatible preset if the newly selected profile is not compatible.
    Never,
    // Only select a compatible preset if the active profile used to be compatible, but it is no more.
    OnlyIfWasCompatible,
    // Always select a compatible preset if the active profile is no more compatible.
    Always,
}

/// Preset.hpp:438-444 `enum class PresetConfigSubstitutions::Source`
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PresetConfigSubstitutionsSource {
    UserFile,
    ConfigBundle,
    //BBS: add cloud and project type
    UserCloud,
    ProjectFile,
}

/// Preset.hpp:432-450 `struct PresetConfigSubstitutions`: substitutions having been performed
/// during parsing a single configuration file.
pub struct PresetConfigSubstitutions {
    // User readable preset name.
    pub preset_name: String,
    // Type of the preset (Print / Filament / Printer ...)
    pub preset_type: Type,
    pub preset_source: PresetConfigSubstitutionsSource,
    // Source of the preset. It may be empty in case of a ConfigBundle being loaded.
    pub preset_file: String,
    // What config value has been substituted with what.
    pub substitutions: ConfigSubstitutions,
}

/// Preset.hpp:452-454: substitutions having been performed during parsing a set of
/// configuration files.
pub type PresetsConfigSubstitutions = Vec<PresetConfigSubstitutions>;

/// Preset.hpp:543-550 `enum class PresetCollection::LoadAndSelect`
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LoadAndSelect {
    // Never select
    Never,
    // Always select
    Always,
    // Select a profile only if it was modified.
    OnlyIfModified,
}

/// Preset.hpp:465 `typedef std::function<void(Preset* preset, std::string sync_info)> SyncFunc;`
pub type SyncFunc = Arc<dyn Fn(&mut Preset, String) + Send + Sync>;

/// Preset.cpp:3151-3180 `template<class T> void add_correct_opts_to_diff(const std::string &opt_key, t_config_option_keys& vec, const ConfigBase &other, const ConfigBase &this_c, bool strict)`
fn add_correct_opts_to_diff<T: crate::config::ConfigValue>(
    opt_key: &str,
    vec: &mut t_config_option_keys,
    other: &dyn ConfigBase,
    this_c: &dyn ConfigBase,
    strict: bool,
) {
    let opt_init_o = other.option(opt_key).expect("add_correct_opts_to_diff(): other.option(opt_key)");
    let opt_cur_o = this_c.option(opt_key).expect("add_correct_opts_to_diff(): this_c.option(opt_key)");
    let opt_init = opt_init_o.vector_values_any().and_then(|v| v.downcast_ref::<Vec<T>>()).expect("static_cast<const T*>(other.option(opt_key))");
    let opt_cur = opt_cur_o.vector_values_any().and_then(|v| v.downcast_ref::<Vec<T>>()).expect("static_cast<const T*>(this_c.option(opt_key))");
    let opt_init_max_id = opt_init.len() as i32 - 1;
    if opt_init_max_id < 0 {
        for i in 0..opt_cur.len() as i32 {
            vec.push(format!("{}#{}", opt_key, i));
        }
        return;
    }

    let cur_vec = opt_cur_o.as_vector_base().expect("ConfigOptionVectorBase");
    let init_vec = opt_init_o.as_vector_base().expect("ConfigOptionVectorBase");
    for i in 0..opt_cur.len() as i32 {
        let init_id = if i <= opt_init_max_id { i } else { 0 };
        if opt_cur[i as usize] != opt_init[init_id as usize] {
            if opt_cur_o.nullable() {
                if cur_vec.is_nil_at(i as usize) {
                    if strict && !init_vec.is_nil_at(init_id as usize) {
                        vec.push(format!("{}#{}", opt_key, i));
                    }
                } else if strict || !init_vec.is_nil_at(init_id as usize) {
                    vec.push(format!("{}#{}", opt_key, i));
                }
            } else {
                vec.push(format!("{}#{}", opt_key, i));
            }
        }
    }
}

/// Preset.cpp:3182-3240 `inline t_config_option_keys deep_diff(const ConfigBase &config_this, const ConfigBase &config_other, bool strict = true)`:
/// changed options, considering individual options for each extruder.
pub fn deep_diff(config_this: &dyn ConfigBase, config_other: &dyn ConfigBase, strict: bool) -> CfgResult<t_config_option_keys> {
    let mut diff: t_config_option_keys = Vec::new();
    let keys: t_config_option_keys = if strict {
        let keys_this = config_this.keys();
        let keys_other = config_other.keys();
        // std::set_union of the two sorted key lists
        let mut keys = Vec::with_capacity(keys_this.len() + keys_other.len());
        let (mut i, mut j) = (0, 0);
        while i < keys_this.len() {
            if j == keys_other.len() {
                keys.extend(keys_this[i..].iter().cloned());
                break;
            }
            if keys_other[j] < keys_this[i] {
                keys.push(keys_other[j].clone());
                j += 1;
            } else {
                if !(keys_this[i] < keys_other[j]) {
                    j += 1;
                }
                keys.push(keys_this[i].clone());
                i += 1;
            }
        }
        keys.extend(keys_other[j.min(keys_other.len())..].iter().cloned());
        keys
    } else {
        config_this.keys()
    };
    for opt_key in &keys {
        let this_opt = config_this.option(opt_key);
        let other_opt = config_other.option(opt_key);
        let both_differ = match (this_opt, other_opt) {
            (Some(t), Some(o)) => t.operator_ne(o)?,
            _ => false,
        };
        if both_differ {
            let other_opt = other_opt.unwrap();
            //BBS: add bed_exclude_area
            if opt_key == "printable_area"
                || opt_key == "bed_exclude_area"
                || opt_key == "compatible_prints"
                || opt_key == "compatible_printers"
                || opt_key == "thumbnail_size"
                || opt_key == "wrapping_exclude_area"
            {
                // Scalar variable, or a vector variable, which is independent from number of extruders,
                // thus the vector is presented to the user as a single input.
                diff.push(opt_key.clone());
            } else if opt_key == "default_filament_profile" {
                // Ignore this field, it is not presented to the user, therefore showing a "modified" flag for this parameter does not help.
                // Also the length of this field may differ, which may lead to a crash if the block below is used.
            } else {
                match other_opt.r#type() {
                    ConfigOptionType::CoInts => add_correct_opts_to_diff::<i32>(opt_key, &mut diff, config_other, config_this, strict),
                    ConfigOptionType::CoBools => add_correct_opts_to_diff::<u8>(opt_key, &mut diff, config_other, config_this, strict),
                    ConfigOptionType::CoFloats => add_correct_opts_to_diff::<f64>(opt_key, &mut diff, config_other, config_this, strict),
                    ConfigOptionType::CoStrings => add_correct_opts_to_diff::<String>(opt_key, &mut diff, config_other, config_this, strict),
                    ConfigOptionType::CoPercents => add_correct_opts_to_diff::<f64>(opt_key, &mut diff, config_other, config_this, strict),
                    ConfigOptionType::CoFloatsOrPercents => {
                        add_correct_opts_to_diff::<crate::config::FloatOrPercent>(opt_key, &mut diff, config_other, config_this, strict)
                    }
                    ConfigOptionType::CoPoints => add_correct_opts_to_diff::<crate::geometry::Vec2d>(opt_key, &mut diff, config_other, config_this, strict),
                    // BBS
                    ConfigOptionType::CoEnums => add_correct_opts_to_diff::<i32>(opt_key, &mut diff, config_other, config_this, strict),
                    _ => diff.push(opt_key.clone()),
                }
            }
        } else if strict {
            let opt = match (this_opt, other_opt) {
                (Some(t), None) => Some(t),
                (None, Some(o)) => Some(o),
                _ => None,
            };
            if let Some(opt) = opt {
                if (opt.r#type() as i32 & ConfigOptionType::CoVectorType as i32) != 0 {
                    let vec = opt.as_vector_base().expect("dynamic_cast<ConfigOptionVectorBase const *>");
                    for i in 0..vec.size() {
                        diff.push(format!("{}#{}", opt_key, i));
                    }
                } else {
                    diff.push(opt_key.clone());
                }
            }
        }
    }
    Ok(diff)
}

/// Preset.cpp:3242 `static constexpr const std::initializer_list<const char*> optional_keys`
const OPTIONAL_KEYS: [&str; 2] = ["compatible_prints", "compatible_printers"];
/// Preset.cpp:3243-3244 `static std::set<std::string> skipped_in_dirty`: BBS: skip these keys for dirty check
static SKIPPED_IN_DIRTY: LazyLock<BTreeSet<String>> = LazyLock::new(|| {
    ["printer_settings_id", "print_settings_id", "filament_settings_id", "printer_nozzle_diameter_mode"].iter().map(|s| s.to_string()).collect()
});

/// Preset.cpp:2262-2276 `static bool profile_print_params_same(const DynamicPrintConfig &cfg_old, const DynamicPrintConfig &cfg_new)`
fn profile_print_params_same(cfg_old: &DynamicPrintConfig, cfg_new: &DynamicPrintConfig) -> CfgResult<bool> {
    let mut diff = cfg_old.base.diff(&cfg_new.base)?;
    // Following keys are used by the UI, not by the slicing core, therefore they are not important
    // when comparing profiles for equality. Ignore them.
    for key in [
        "compatible_prints", "compatible_prints_condition",
        "compatible_printers", "compatible_printers_condition", "inherits",
        "print_settings_id", "filament_settings_id", "sla_print_settings_id", "sla_material_settings_id", "printer_settings_id",
        "printer_nozzle_diameter_mode",
        "printer_model", "printer_variant", "default_print_profile", "default_filament_profile", "default_sla_print_profile", "default_sla_material_profile",
    ] {
        diff.retain(|k| k != key);
    }
    // Preset with the same name as stored inside the config exists.
    Ok(diff.is_empty())
}

// ============================================================================
// PresetCollection (Preset.hpp:456-868, Preset.cpp:1347-3612)
// ============================================================================

/// Preset.hpp:456-868 `class PresetCollection`: a collection of presets of the same type (one
/// of the Print, Filament or Printer type), sorted by name after the default presets.
///
/// The members are public: PresetBundle, a friend of the C++ class, reads and writes them.
pub struct PresetCollection {
    //BBS
    pub sync_func: Option<SyncFunc>,
    // Type of this PresetCollection: TYPE_PRINT, TYPE_FILAMENT or TYPE_PRINTER.
    pub m_type: Type,
    // List of presets, starting with the "- default -" preset.
    pub m_presets: Vec<Preset>,
    // System profiles may have aliases. Map to the full profile name.
    pub m_map_alias_to_profile_name: BTreeMap<String, Vec<String>>,
    pub m_printer_hold_alias: HashMap<String, HashSet<String>>,
    // Map from old system profile name to a current system profile name.
    pub m_map_system_profile_renamed: BTreeMap<String, String>,
    // Initially this preset contains a copy of the selected preset. Later on, this copy may be modified by the user.
    pub m_edited_preset: Preset,
    // Contains a copy of the last saved selected preset.
    pub m_saved_preset: Preset,
    // Selected preset.
    pub m_idx_selected: usize,
    // BBS multi-diameter: extra printer preset names accepted as compatible in update_compatible_internal().
    pub m_extra_compatible_printers: Vec<String>,
    // BBS multi-diameter: name of the printer preset used in the last update_compatible_internal() pass.
    pub m_active_printer_name: String,
    // Persist whether the last compatibility pass was broadened.
    pub m_prefer_active_printer_alias: bool,
    // Is the "- default -" preset suppressed?
    pub m_default_suppressed: bool,
    pub m_num_default_presets: usize,
    // Path to the directory to store the config files into.
    pub m_dir_path: String,
    /// FIDELITY-NOTE: `default_preset_for()` is virtual in C++; PrinterPresetCollection
    /// overrides it. The collection made by `PrinterPresetCollection::new` sets this.
    m_printer_collection: bool,
}

/// Where a `Preset*` of the C++ points: an element of m_presets or m_edited_preset.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PresetSlot {
    Index(usize),
    Edited,
}

impl PresetCollection {
    /// Preset.cpp:1347-1357 `PresetCollection::PresetCollection(Preset::Type type, const std::vector<std::string> &keys, const Slic3r::StaticPrintConfig &defaults, const std::string &default_name)`:
    /// initialize the PresetCollection with the "- default -" preset. `defaults` is the
    /// StaticPrintConfig (`print_config::config_def::static_print_config_defaults`).
    pub fn new(ty: Type, keys: &[String], defaults: &dyn ConfigBase, default_name: &str) -> CfgResult<PresetCollection> {
        let mut c = PresetCollection {
            sync_func: None,
            m_type: ty,
            m_presets: Vec::new(),
            m_map_alias_to_profile_name: BTreeMap::new(),
            m_printer_hold_alias: HashMap::new(),
            m_map_system_profile_renamed: BTreeMap::new(),
            m_edited_preset: Preset::new(ty, "", false),
            m_saved_preset: Preset::new(ty, "", false),
            m_idx_selected: 0,
            m_extra_compatible_printers: Vec::new(),
            m_active_printer_name: String::new(),
            m_prefer_active_printer_alias: false,
            m_default_suppressed: true,
            m_num_default_presets: 0,
            m_dir_path: String::new(),
            m_printer_collection: false,
        };
        // Insert just the default preset.
        c.add_default_preset(keys, defaults, default_name)?;
        let front = c.m_presets[0].config.clone();
        c.m_edited_preset.config.apply(&front, false)?;
        c.update_saved_preset_from_current_preset();
        Ok(c)
    }

    /// Preset.cpp:1359-1377 `PresetCollection& PresetCollection::operator=(const PresetCollection &rhs)`
    /// (BBS: not the default; m_printer_hold_alias and sync_func are not copied).
    pub fn assign(&mut self, rhs: &PresetCollection) {
        self.m_type = rhs.m_type;
        self.m_presets = rhs.m_presets.clone();
        self.m_map_alias_to_profile_name = rhs.m_map_alias_to_profile_name.clone();
        self.m_map_system_profile_renamed = rhs.m_map_system_profile_renamed.clone();
        self.m_edited_preset = rhs.m_edited_preset.clone();
        self.m_saved_preset = rhs.m_saved_preset.clone();
        self.m_idx_selected = rhs.m_idx_selected;
        self.m_extra_compatible_printers = rhs.m_extra_compatible_printers.clone();
        self.m_active_printer_name = rhs.m_active_printer_name.clone();
        self.m_prefer_active_printer_alias = rhs.m_prefer_active_printer_alias;
        self.m_default_suppressed = rhs.m_default_suppressed;
        self.m_num_default_presets = rhs.m_num_default_presets;
        self.m_dir_path = rhs.m_dir_path.clone();
    }

    /// Preset.hpp:467 `Iterator lbegin()`: BBS get m_presets begin
    pub fn lbegin(&self) -> usize {
        0
    }
    /// Preset.hpp:471-473 `begin()` / `cbegin()`: the first preset after the defaults.
    pub fn begin(&self) -> usize {
        self.m_num_default_presets
    }
    /// Preset.hpp:473
    pub fn cbegin(&self) -> usize {
        self.m_num_default_presets
    }
    /// Preset.hpp:474-476 `end()` / `cend()`
    pub fn end(&self) -> usize {
        self.m_presets.len()
    }
    /// Preset.hpp:476
    pub fn cend(&self) -> usize {
        self.m_presets.len()
    }
    /// The presets of `begin()..end()`.
    pub fn iter(&self) -> std::slice::Iter<'_, Preset> {
        self.m_presets[self.m_num_default_presets..].iter()
    }
    /// Preset.hpp:479 `Iterator erase(Iterator it)` (BBS)
    pub fn erase(&mut self, it: usize) -> usize {
        self.m_presets.remove(it);
        it
    }
    /// Preset.hpp:481
    pub fn set_sync_func(&mut self, func: SyncFunc) {
        self.sync_func = Some(func);
    }
    /// Preset.hpp:483 (see the module notes)
    pub fn lock(&self) {}
    /// Preset.hpp:484 (see the module notes)
    pub fn unlock(&self) {}

    /// Preset.cpp:1379-1403 `void PresetCollection::reset(bool delete_files)`
    pub fn reset(&mut self, delete_files: bool) {
        //BBS: add lock logic for sync preset in background
        self.lock();
        if self.m_presets.len() > self.m_num_default_presets {
            if delete_files {
                // Erase the preset files.
                for preset in &self.m_presets {
                    if !preset.is_default && !preset.is_external && !preset.is_system {
                        //BBS remove idx and ini files
                        preset.remove_files();
                    }
                }
            }
            // Don't use m_presets.resize() here as it requires a default constructor for Preset.
            self.m_presets.truncate(self.m_num_default_presets);
            self.select_preset(0);
        }
        //BBS: add lock logic for sync preset in background
        self.unlock();
        self.m_map_alias_to_profile_name.clear();
        self.m_map_system_profile_renamed.clear();
        self.m_extra_compatible_printers.clear();
        self.m_active_printer_name.clear();
        self.m_prefer_active_printer_alias = false;
    }

    /// Preset.hpp:488 `Preset::Type type() const`
    pub fn type_(&self) -> Type {
        self.m_type
    }

    /// Preset.cpp:3553-3563 `std::string PresetCollection::name() const`: name, to be used on
    /// the screen and in error messages. Not localized.
    pub fn name(&self) -> String {
        match self.type_() {
            Type::TypePrint => PRESET_PRINT_NAME.to_string(),
            Type::TypeFilament => PRESET_FILAMENT_NAME.to_string(),
            Type::TypePrinter => PRESET_PRINTER_NAME.to_string(),
            _ => "invalid".to_string(),
        }
    }

    /// Preset.cpp:3565-3576 `std::string PresetCollection::section_name() const`: name, to be
    /// used as a section name in config bundle, and as a folder name for presets.
    pub fn section_name(&self) -> String {
        match self.type_() {
            Type::TypePrint => PRESET_PRINT_NAME.to_string(),
            Type::TypeFilament => PRESET_FILAMENT_NAME.to_string(),
            Type::TypePrinter => PRESET_PRINTER_NAME.to_string(),
            _ => "invalid".to_string(),
        }
    }

    /// Preset.hpp:493 `const std::deque<Preset>& operator()() const`
    pub fn operator_call(&self) -> &Vec<Preset> {
        &self.m_presets
    }

    /// Preset.cpp:1405-1412 `void PresetCollection::add_default_preset(const std::vector<std::string> &keys, const Slic3r::StaticPrintConfig &defaults, const std::string &preset_name)`:
    /// add default preset at the start of the collection, increment the m_default_preset counter.
    pub fn add_default_preset(&mut self, keys: &[String], defaults: &dyn ConfigBase, preset_name: &str) -> CfgResult<()> {
        // Insert just the default preset.
        self.m_presets.push(Preset::new(self.type_(), preset_name, true));
        let keys: Vec<String> = if keys.is_empty() { defaults.keys() } else { keys.to_vec() };
        self.m_presets.last_mut().unwrap().config.apply_only(defaults, &keys, false)?;
        self.m_presets.last_mut().unwrap().loaded = true;
        self.m_num_default_presets += 1;
        Ok(())
    }

    /// Sorts `m_presets[from..]` by name as `std::sort(m_presets.begin() + from, m_presets.end())`
    /// does with libc++ (Preset::operator<).
    fn sort_presets_from(&mut self, from: usize) {
        let mut idx: Vec<usize> = (from..self.m_presets.len()).collect();
        let presets = &self.m_presets;
        crate::clipper::libcxx_sort(&mut idx, |a, b| presets[*a].name < presets[*b].name);
        let mut taken: Vec<Option<Preset>> = self.m_presets.drain(from..).map(Some).collect();
        for i in idx {
            let p = taken[i - from].take().unwrap();
            self.m_presets.push(p);
        }
    }

    /// Preset.cpp:1414-1606 `void PresetCollection::load_presets(const std::string &dir_path, const std::string &subdir, PresetsConfigSubstitutions& substitutions, ForwardCompatibilitySubstitutionRule substitution_rule)`:
    /// load all presets found in dir_path. Throws an exception on error.
    pub fn load_presets(
        &mut self,
        dir_path: &str,
        subdir: &str,
        substitutions: &mut PresetsConfigSubstitutions,
        substitution_rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<()> {
        // Don't use boost::filesystem::canonical() on Windows, it is broken in regard to reparse points,
        // see https://github.com/prusa3d/PrusaSlicer/issues/732
        let dir = boost_make_preferred(boost_absolute(&boost_path_append(dir_path, subdir)));

        // Load custom roots first
        if fs_exists(&boost_path_append(&dir, "base")) {
            self.load_presets(&dir, "base", substitutions, substitution_rule)?;
        }

        //BBS do not parse folder if not exists
        self.m_dir_path = dir.clone();
        if !fs_exists(&dir) {
            std::fs::create_dir(&dir).map_err(|e| runtime_error(format!("boost::filesystem::create_directory: {}: \"{}\"", e, dir)))?;
            return Ok(());
        }

        let errors_cummulative = String::new();
        // Store the loaded presets into a new vector, otherwise the binary search for already existing presets would be broken.
        // (see the "Preset already present, not loading" message).
        let mut presets_loaded: Vec<Preset> = Vec::new();

        //BBS: get the extruder related info for this preset collection
        let mut extruder_id_name = String::new();
        let mut extruder_variant_name = String::new();
        let mut key_set1: &'static BTreeSet<String> = &empty_options;
        let mut key_set2: &'static BTreeSet<String> = &empty_options;
        Preset::get_extruder_names_and_keysets(self.m_type, &mut extruder_id_name, &mut extruder_variant_name, &mut key_set1, &mut key_set2);

        //BBS: change to json format
        let entries = std::fs::read_dir(&dir).map_err(|e| runtime_error(format!("boost::filesystem::directory_iterator::construct: {}: \"{}\"", e, dir)))?;
        for dir_entry in entries {
            let dir_entry = match dir_entry {
                Ok(e) => e,
                Err(e) => return Err(runtime_error(format!("boost::filesystem::directory_iterator::operator++: {}", e))),
            };
            let file_name = dir_entry.file_name().to_string_lossy().into_owned();
            if !crate::utils::is_json_file(&file_name) {
                continue;
            }
            // Remove the .json suffix.
            let name = file_name[..file_name.len() - 5].to_string();
            if self.find_preset(&name, false).is_some() {
                // This happens when there's is a preset (most likely legacy one) with the same name as a system preset
                // that's already been loaded from a bundle.
                continue;
            }
            let mut preset = Preset::new(self.m_type, &name, false);
            preset.file = boost_path_append(&dir, &file_name);
            // Load the preset file, apply preset values on top of defaults.
            let result = self.load_presets_one(
                &mut preset,
                &name,
                substitutions,
                substitution_rule,
                &extruder_id_name,
                &extruder_variant_name,
                key_set1,
                key_set2,
            );
            match result {
                Ok(true) => {}
                Ok(false) => continue,
                Err(_err) => {
                    // catch (const std::ifstream::failure &err) / catch (const std::runtime_error &err):
                    // "Failed loading the user-config file", the file and its .info removed.
                    let mut file_path = preset.file.clone();
                    if fs_exists(&file_path) {
                        fs_remove(&file_path);
                    }
                    file_path = boost_replace_extension(&file_path, ".info");
                    if fs_exists(&file_path) {
                        fs_remove(&file_path);
                    }
                }
            }

            presets_loaded.push(preset);
        }
        if !presets_loaded.is_empty() {
            self.m_presets.extend(presets_loaded);
        }
        self.sort_presets_from(self.m_num_default_presets);
        //this->select_preset(first_visible_idx());
        if !errors_cummulative.is_empty() {
            return Err(runtime_error(errors_cummulative));
        }
        Ok(())
    }

    /// The inner `try` block of load_presets (Preset.cpp:1466-1568): Ok(false) for its
    /// `continue` statements.
    #[allow(clippy::too_many_arguments)]
    fn load_presets_one(
        &mut self,
        preset: &mut Preset,
        name: &str,
        substitutions: &mut PresetsConfigSubstitutions,
        substitution_rule: ForwardCompatibilitySubstitutionRule,
        extruder_id_name: &str,
        extruder_variant_name: &str,
        key_set1: &BTreeSet<String>,
        key_set2: &BTreeSet<String>,
    ) -> CfgResult<bool> {
        let idx_path = boost_replace_extension(&preset.file, ".info");
        if fs_exists(&idx_path) {
            preset.load_info(&idx_path);
        }
        let mut config = DynamicPrintConfig::new();
        //BBS: change to json format
        let mut key_values: BTreeMap<String, String> = BTreeMap::new();
        let mut reason = String::new();
        let config_substitutions = config.load_from_json(&preset.file, substitution_rule, &mut key_values, &mut reason);
        if !config_substitutions.is_empty() {
            substitutions.push(PresetConfigSubstitutions {
                preset_name: preset.name.clone(),
                preset_type: self.m_type,
                preset_source: PresetConfigSubstitutionsSource::UserFile,
                preset_file: preset.file.clone(),
                substitutions: config_substitutions,
            });
        }
        if !reason.is_empty() {
            let mut file_path = preset.file.clone();
            if fs_exists(&file_path) {
                fs_remove(&file_path);
            }
            file_path = boost_replace_extension(&file_path, ".info");
            if fs_exists(&file_path) {
                fs_remove(&file_path);
            }
            return Ok(false);
        }

        let version_str = key_values.entry(BBL_JSON_KEY_VERSION.to_string()).or_default().clone();
        let version = match Semver::parse(&version_str) {
            None => return Ok(false),
            Some(v) => v,
        };
        let app_version = Semver::parse(SLIC3R_VERSION).expect("Semver::parse(SLIC3R_VERSION)");
        if version.maj() > app_version.maj() {
            return Ok(false);
        }
        preset.version = version;

        if let Some(v) = key_values.get(BBL_JSON_KEY_FILAMENT_ID) {
            preset.filament_id = v.clone();
        }
        if let Some(v) = key_values.get(BBL_JSON_KEY_DESCRIPTION) {
            preset.description = v.clone();
        }
        if let Some(v) = key_values.get(BBL_JSON_KEY_INSTANTIATION) {
            preset.is_visible = v != "false";
        }

        //BBS: use inherit config as the base
        let mut inherit_preset: Option<usize> = None;
        let inherits_value: Option<String> = config.option(BBL_JSON_KEY_INHERITS).map(|o| {
            o.as_any().downcast_ref::<ConfigOptionString>().expect("dynamic_cast<ConfigOptionString *>(inherits_config)").value.clone()
        });

        // check inherits_config
        if let Some(inherits_value) = &inherits_value {
            inherit_preset = self.find_preset_slot(inherits_value, false, true).map(|s| match s {
                PresetSlot::Index(i) => i,
                PresetSlot::Edited => unreachable!("find_preset(.., real = true)"),
            });
        }
        let default_preset = self.default_preset_for_idx(&config);
        if let Some(ip) = inherit_preset {
            preset.config = self.m_presets[ip].config.clone();
            preset.filament_id = self.m_presets[ip].filament_id.clone();
            extend_default_config_length(&mut config, &self.m_presets[ip].config, false, &DynamicPrintConfig::new())?;
            preset.config.update_diff_values_to_child_config(&mut config, extruder_id_name, extruder_variant_name, key_set1, key_set2)?;
        } else {
            if let Some(v) = &inherits_value {
                if !v.is_empty() {
                    // BOOST_LOG_TRIVIAL(error) << "can not find parent %1% for config %2%!"
                    return Ok(false);
                }
            }
            // We support custom root preset now
            // Find a default preset for the config. The PrintPresetCollection provides different default preset based on the "printer_technology" field.
            preset.config = self.m_presets[default_preset].config.clone();
            preset.config.apply(&config, false)?;
            extend_default_config_length(&mut preset.config, &DynamicPrintConfig::new(), true, &self.m_presets[default_preset].config)?;
        }

        Preset::normalize(&mut preset.config)?;
        // Report configuration fields, which are misplaced into a wrong group.
        let _incorrect_keys = Preset::remove_invalid_keys(&mut preset.config, &self.m_presets[default_preset].config);

        if preset.r#type == Type::TypeFilament && preset.is_user() && preset.inherits_mut().is_empty() {
            let mut added = false;
            {
                let compatible_printers = preset
                    .config
                    .option_mut("compatible_printers", true)?
                    .and_then(|o| o.as_any_mut().downcast_mut::<ConfigOptionStrings>());
                if let Some(compatible_printers) = compatible_printers {
                    if compatible_printers.values.is_empty() {
                        if let Some(at_pos) = name.find('@') {
                            if at_pos + 1 < name.len() {
                                compatible_printers.values.push(name[at_pos + 1..].to_string());
                                added = true;
                            }
                        }
                    }
                }
            }
            if added {
                preset.save(None)?;
            }
        }

        preset.loaded = true;
        //BBS: add some workaround for previous incorrect settings
        if !preset.setting_id.is_empty() && preset.setting_id == preset.base_id {
            preset.setting_id.clear();
        }
        // add alias for custom filament preset
        self.set_custom_preset_alias(preset);
        Ok(true)
    }

    /// Preset.cpp:1608-1667 `Preset* PresetCollection::get_preset_differed_for_save(Preset& preset)`:
    /// BBS: a preset holding only what differs from its parent, for saving (`None` for a system
    /// or default preset, or when the parent is not found).
    pub fn get_preset_differed_for_save(&self, preset: &Preset) -> CfgResult<Option<Preset>> {
        if preset.is_system || preset.is_default {
            return Ok(None);
        }

        let mut new_preset: Option<Preset> = None;
        //BBS: only save difference for user preset
        let inherits = preset.inherits();
        let mut parent_preset: Option<&Preset> = None;
        if !inherits.is_empty() {
            parent_preset = self.find_preset_real(inherits, false, true);
        }
        if let Some(parent_preset) = parent_preset {
            let mut np = preset.clone();

            let mut temp_config = DynamicPrintConfig::new();
            let mut dirty_options = preset.config.base.diff(&parent_preset.config.base)?;

            let mut extruder_id_name = String::new();
            let mut extruder_variant_name = String::new();
            let mut key_set1: &'static BTreeSet<String> = &empty_options;
            let mut key_set2: &'static BTreeSet<String> = &empty_options;
            Preset::get_extruder_names_and_keysets(self.m_type, &mut extruder_id_name, &mut extruder_variant_name, &mut key_set1, &mut key_set2);

            if !extruder_id_name.is_empty() {
                dirty_options.push(extruder_id_name.clone());
            }
            if !extruder_variant_name.is_empty() {
                dirty_options.push(extruder_variant_name.clone());
            }

            for option in &dirty_options {
                set_option_for_save(&mut temp_config, &preset.config, &parent_preset.config, option, key_set1, key_set2)?;
            }

            np.config = temp_config;
            new_preset = Some(np);
        }

        Ok(new_preset)
    }

    /// Preset.cpp:1669-1735 `int PresetCollection::get_differed_values_to_update(Preset& preset, std::map<std::string, std::string>& key_values)`:
    /// BBS: the different values to update.
    pub fn get_differed_values_to_update(&self, preset: &Preset, key_values: &mut BTreeMap<String, String>) -> CfgResult<i32> {
        if preset.is_system || preset.is_default || preset.is_project_embedded {
            return Ok(-1);
        }

        //BBS: only save difference for user preset
        let inherit_preset = preset.inherits();
        let mut parent_preset: Option<&Preset> = None;
        if !inherit_preset.is_empty() {
            parent_preset = self.find_preset_real(inherit_preset, false, true);
        }
        if let Some(parent_preset) = parent_preset {
            let dirty_options = preset.config.base.diff(&parent_preset.config.base)?;

            for option in &dirty_options {
                if let Some(opt_src) = preset.config.option(option) {
                    key_values.insert(option.clone(), opt_src.serialize()?);
                }
            }
        } else {
            for (first, second) in preset.config.cbegin() {
                key_values.insert(first.clone(), second.serialize()?);
            }
        }

        //add other values
        key_values.insert(BBL_JSON_KEY_VERSION.to_string(), preset.version.to_string());
        if !preset.base_id.is_empty() {
            key_values.insert(BBL_JSON_KEY_BASE_ID.to_string(), preset.base_id.clone());
        } else {
            key_values.remove(BBL_JSON_KEY_BASE_ID);
            if self.get_preset_base(preset).map_or(false, |b| std::ptr::eq(b, preset)) && !preset.filament_id.is_empty() {
                key_values.insert(BBL_JSON_KEY_FILAMENT_ID.to_string(), preset.filament_id.clone());
            }
        }
        key_values.insert(BBL_JSON_KEY_UPDATE_TIME.to_string(), preset.updated_time.to_string());
        key_values.insert(BBL_JSON_KEY_TYPE.to_string(), Preset::get_iot_type_string(preset.r#type));

        let mut extruder_id_name = String::new();
        let mut extruder_variant_name = String::new();
        let mut key_set1: &'static BTreeSet<String> = &empty_options;
        let mut key_set2: &'static BTreeSet<String> = &empty_options;
        Preset::get_extruder_names_and_keysets(self.m_type, &mut extruder_id_name, &mut extruder_variant_name, &mut key_set1, &mut key_set2);

        if !extruder_id_name.is_empty() && !key_values.contains_key(&extruder_id_name) {
            let v = preset.config.option(&extruder_id_name).expect("preset.config.option(extruder_id_name)").serialize()?;
            key_values.insert(extruder_id_name.clone(), v);
        }
        if !extruder_variant_name.is_empty() && !key_values.contains_key(&extruder_variant_name) {
            let v = preset.config.option(&extruder_variant_name).expect("preset.config.option(extruder_variant_name)").serialize()?;
            key_values.insert(extruder_variant_name.clone(), v);
        }

        let mut update_size: i32 = 0;
        for (first, second) in key_values.iter() {
            update_size += first.len() as i32;
            update_size += second.len() as i32;
        }
        if update_size > 350 * 1024 {
            return Ok(-2);
        }

        Ok(0)
    }

    /// Preset.cpp:1737-1819 `void PresetCollection::load_project_embedded_presets(std::vector<Preset*>& project_presets, const std::string& type, PresetsConfigSubstitutions& substitutions, ForwardCompatibilitySubstitutionRule rule)`
    pub fn load_project_embedded_presets(
        &mut self,
        project_presets: &mut [Preset],
        r#type: &str,
        substitutions: &mut PresetsConfigSubstitutions,
        _rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<()> {
        let mut errors_cummulative = String::new();
        // Store the loaded presets into a new vector, otherwise the binary search for already existing presets would be broken.
        let mut presets_loaded: Vec<Preset> = Vec::new();

        let mut extruder_id_name = String::new();
        let mut extruder_variant_name = String::new();
        let mut key_set1: &'static BTreeSet<String> = &empty_options;
        let mut key_set2: &'static BTreeSet<String> = &empty_options;
        Preset::get_extruder_names_and_keysets(self.m_type, &mut extruder_id_name, &mut extruder_variant_name, &mut key_set1, &mut key_set2);

        self.lock();
        for preset in project_presets.iter_mut() {
            if preset.r#type != Preset::get_type_from_string(r#type) {
                continue;
            }
            if !preset.is_project_embedded {
                continue;
            }
            let name = preset.name.clone();
            if self.find_preset(&name, false).is_some() {
                continue;
            }
            let result: CfgResult<bool> = (|| {
                let mut config = preset.config.clone();
                if let Some(subs) = preset.loading_substitutions.take() {
                    if !subs.is_empty() {
                        substitutions.push(PresetConfigSubstitutions {
                            preset_name: preset.name.clone(),
                            preset_type: self.m_type,
                            preset_source: PresetConfigSubstitutionsSource::ProjectFile,
                            preset_file: preset.name.clone(),
                            substitutions: subs,
                        });
                        // free(preset->loading_substitutions); preset->loading_substitutions = NULL;
                    } else {
                        // An empty list stays attached to the preset in the C++.
                        preset.loading_substitutions = Some(subs);
                    }
                }
                //BBS: use inherit config as the base
                let mut inherit_preset: Option<usize> = None;
                if let Some(o) = config.option(BBL_JSON_KEY_INHERITS) {
                    let inherits_value = o.as_any().downcast_ref::<ConfigOptionString>().expect("dynamic_cast<ConfigOptionString *>").value.clone();
                    inherit_preset = self.find_preset_slot(&inherits_value, false, true).map(|s| match s {
                        PresetSlot::Index(i) => i,
                        PresetSlot::Edited => unreachable!("find_preset(.., real = true)"),
                    });
                }
                let default_preset = self.default_preset_for_idx(&config);
                if let Some(ip) = inherit_preset {
                    preset.config = self.m_presets[ip].config.clone();
                    preset.filament_id = self.m_presets[ip].filament_id.clone();
                } else {
                    // Find a default preset for the config. The PrintPresetCollection provides different default preset based on the "printer_technology" field.
                    //BBS 202407: don't load project embedded preset when can not find inherit
                    return Ok(false);
                }
                preset.config.update_diff_values_to_child_config(&mut config, &extruder_id_name, &extruder_variant_name, key_set1, key_set2)?;
                Preset::normalize(&mut preset.config)?;
                // Report configuration fields, which are misplaced into a wrong group.
                let _incorrect_keys = Preset::remove_invalid_keys(&mut preset.config, &self.m_presets[default_preset].config);
                preset.loaded = true;
                presets_loaded.push(preset.clone());
                Ok(true)
            })();
            match result {
                Ok(_) => {}
                Err(err) => {
                    errors_cummulative += err.what();
                    errors_cummulative += "\n";
                }
            }
        }

        self.m_presets.extend(presets_loaded);
        self.sort_presets_from(self.m_num_default_presets);
        //don't select it here
        self.unlock();

        if !errors_cummulative.is_empty() {
            return Err(runtime_error(errors_cummulative));
        }
        Ok(())
    }

    /// Preset.cpp:1821-1839 `std::vector<Preset*> PresetCollection::get_project_embedded_presets()`
    /// (the C++ returns presets the caller frees).
    pub fn get_project_embedded_presets(&self) -> CfgResult<Vec<Preset>> {
        let mut project_presets: Vec<Preset> = Vec::new();

        self.lock();
        for preset in &self.m_presets {
            if !preset.is_project_embedded {
                continue;
            }

            if let Some(new_preset) = self.get_preset_differed_for_save(preset)? {
                project_presets.push(new_preset);
            }
        }
        self.unlock();
        Ok(project_presets)
    }

    /// Preset.cpp:1841-1873 `bool PresetCollection::reset_project_embedded_presets()`
    pub fn reset_project_embedded_presets(&mut self) -> bool {
        let mut it = 0usize;
        let mut re_select = false;
        let mut count: isize = -1;

        self.lock();
        while it != self.m_presets.len() {
            count += 1;
            if self.m_presets[it].is_project_embedded {
                // size_t compared with int: the int converted to size_t.
                if !re_select && self.m_idx_selected == count as usize {
                    re_select = true;
                }
                if self.m_idx_selected > count as usize {
                    self.m_idx_selected = self.m_idx_selected.wrapping_sub(1);
                    count -= 1;
                }
                self.m_presets.remove(it);
            } else {
                it += 1;
            }
        }

        if re_select {
            self.m_idx_selected = usize::MAX;
        }

        self.unlock();

        re_select
    }

    /// Preset.cpp:1875-1903 `void PresetCollection::set_sync_info_and_save(std::string name, std::string setting_id, std::string syncinfo, long long update_time)`
    pub fn set_sync_info_and_save(&mut self, name: &str, setting_id: &str, syncinfo: &str, update_time: i64) -> CfgResult<()> {
        self.lock();
        for it in 0..self.m_presets.len() {
            if self.m_presets[it].name == name {
                if syncinfo.is_empty() {
                    self.m_presets[it].sync_info.clear();
                } else {
                    self.m_presets[it].sync_info = syncinfo.to_string();
                }
                let is_base = {
                    let p = &self.m_presets[it];
                    self.get_preset_base(p).map_or(false, |b| std::ptr::eq(b, p))
                };
                if is_base {
                    let preset_name = self.m_presets[it].name.clone();
                    for preset2 in self.m_presets.iter_mut() {
                        if *preset2.inherits_mut() == preset_name {
                            preset2.base_id = setting_id.to_string();
                            preset2.save_info("");
                        }
                    }
                }
                self.m_presets[it].setting_id = setting_id.to_string();
                if update_time > 0 {
                    self.m_presets[it].updated_time = update_time;
                }
                if self.m_presets[it].sync_info == "update" {
                    self.m_presets[it].save(None)?;
                } else {
                    self.m_presets[it].save_info("");
                }
                break;
            }
        }
        self.unlock();
        Ok(())
    }

    /// Preset.cpp:1905-1912 `bool PresetCollection::need_sync(std::string name, std::string setting_id, long long update_time)`
    pub fn need_sync(&self, name: &str, setting_id: &str, update_time: i64) -> bool {
        self.lock();
        let preset = self.find_preset_real(name, false, true);
        let need = match preset {
            None => true,
            Some(p) => p.setting_id != setting_id || p.updated_time < update_time,
        };
        self.unlock();
        need
    }

    /// Preset.cpp:1914-1934 `int PresetCollection::get_user_presets(PresetBundle *preset_bundle, std::vector<Preset> &result_presets)`
    /// (the bundle is not read).
    pub fn get_user_presets(&self, _preset_bundle: Option<&PresetBundleView>, result_presets: &mut Vec<Preset>) -> i32 {
        let mut count = 0;
        result_presets.clear();

        self.lock();
        for preset in &self.m_presets {
            if !preset.is_user() {
                continue;
            }
            if preset.base_id.is_empty() && !preset.inherits().is_empty() {
                continue;
            }
            if !preset.setting_id.is_empty() && preset.sync_info.is_empty() {
                continue;
            }
            //if (!preset.is_bbl_vendor_preset(preset_bundle)) continue;
            if preset.sync_info == "hold" {
                continue;
            }

            result_presets.push(preset.clone());
            count += 1;
        }
        self.unlock();

        count
    }

    /// Preset.cpp:1936-1945 `void PresetCollection::update_user_presets_directory(const std::string& dir_path, const std::string& type)`
    pub fn update_user_presets_directory(&mut self, dir_path: &str, r#type: &str) -> CfgResult<()> {
        let dir = boost_make_preferred(boost_absolute(&boost_path_append(dir_path, r#type)));

        if !fs_exists(&dir) {
            std::fs::create_dir(&dir).map_err(|e| runtime_error(format!("boost::filesystem::create_directory: {}: \"{}\"", e, dir)))?;
        }

        self.m_dir_path = dir;
        Ok(())
    }

    /// Preset.cpp:1947-1998 `void PresetCollection::save_user_presets(const std::string& dir_path, const std::string& type, std::vector<std::string>& need_to_delete_list)`
    pub fn save_user_presets(&mut self, dir_path: &str, r#type: &str, _need_to_delete_list: &mut Vec<String>) -> CfgResult<()> {
        let dir = boost_make_preferred(boost_absolute(&boost_path_append(dir_path, r#type)));

        if !fs_exists(&dir) {
            std::fs::create_dir(&dir).map_err(|e| runtime_error(format!("boost::filesystem::create_directory: {}: \"{}\"", e, dir)))?;
        }

        self.m_dir_path = dir;

        let mut delete_name_list: Vec<String> = Vec::new();
        for it in 0..self.m_presets.len() {
            if !self.m_presets[it].is_user() {
                continue;
            }
            if self.m_presets[it].sync_info != "save" {
                continue;
            }
            self.m_presets[it].sync_info.clear();
            let file = self.path_for_preset(&self.m_presets[it]);
            self.m_presets[it].file = file;

            //BBS: only save difference for user preset
            let inherits = Preset::inherits_cfg(&mut self.m_presets[it].config).clone();
            if inherits.is_empty() {
                // We support custom root preset now
                self.m_presets[it].save(None)?;
                continue;
            }
            let parent = match self.find_preset_slot(&inherits, false, true) {
                Some(PresetSlot::Index(i)) => i,
                _ => {
                    // BOOST_LOG_TRIVIAL(error) << "can not find parent preset for %1% , inherits %2%"
                    continue;
                }
            };

            if self.m_presets[it].base_id.is_empty() {
                self.m_presets[it].base_id = self.m_presets[parent].setting_id.clone();
            }
            let parent_config = self.m_presets[parent].config.clone();
            self.m_presets[it].save(Some(&parent_config))?;
        }

        for delete_name in &delete_name_list {
            self.delete_preset(delete_name);
        }
        delete_name_list.clear();

        Ok(())
    }

    /// Preset.cpp:2000-2205 `bool PresetCollection::load_user_preset(std::string name, std::map<std::string, std::string> preset_values, PresetsConfigSubstitutions& substitutions, ForwardCompatibilitySubstitutionRule rule)`:
    /// BBS: load one user preset from key-values.
    pub fn load_user_preset(
        &mut self,
        name: &str,
        preset_values: BTreeMap<String, String>,
        substitutions: &mut PresetsConfigSubstitutions,
        rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<bool> {
        let mut errors_cummulative = String::new();

        //if the version is not matching, skip it
        if !preset_values.contains_key(BBL_JSON_KEY_VERSION) {
            return Ok(false);
        }
        let version_str = preset_values[BBL_JSON_KEY_VERSION].clone();
        let cloud_version = match Semver::parse(&version_str) {
            None => return Ok(false),
            Some(v) => v,
        };
        let app_version = Semver::parse(SLIC3R_VERSION).expect("Semver::parse(SLIC3R_VERSION)");
        if cloud_version.maj() > app_version.maj() {
            return Ok(false);
        }

        //setting_id
        if !preset_values.contains_key(BBL_JSON_KEY_SETTING_ID) {
            return Ok(false);
        }
        let cloud_setting_id = preset_values[BBL_JSON_KEY_SETTING_ID].clone();

        //update_time
        let mut cloud_update_time: i64 = 0;
        if let Some(v) = preset_values.get(BBL_JSON_KEY_UPDATE_TIME) {
            cloud_update_time = c_atoll(v);
        }

        //user_id
        if !preset_values.contains_key(BBL_JSON_KEY_USER_ID) {
            return Ok(false);
        }
        let cloud_user_id = preset_values[BBL_JSON_KEY_USER_ID].clone();

        self.lock();
        let iter = self.find_preset_internal(name);
        let mut need_update = false;
        if iter != self.m_presets.len() && self.m_presets[iter].name == name {
            //BBS: we should compare the time between cloud and local
            if cloud_update_time == 0 || cloud_update_time <= self.m_presets[iter].updated_time {
                if cloud_update_time < self.m_presets[iter].updated_time {
                    self.m_presets[iter].sync_info = "update".to_string();
                } else {
                    self.m_presets[iter].sync_info.clear();
                }
                // Fixup possible data lost
                self.m_presets[iter].setting_id = cloud_setting_id;
                let idx_file = boost_replace_extension(&self.m_presets[iter].file, ".info");
                self.m_presets[iter].save_info(&idx_file);
                self.unlock();
                return Ok(false);
            } else {
                //update the one from cloud which is newer
                need_update = true;
            }
        }

        // base_id
        if !preset_values.contains_key(BBL_JSON_KEY_BASE_ID) {
            self.unlock();
            return Ok(false);
        }
        let cloud_base_id = preset_values[BBL_JSON_KEY_BASE_ID].clone();

        //filament_id
        let mut cloud_filament_id = String::new();
        if self.m_type == Type::TypeFilament {
            if let Some(v) = preset_values.get(BBL_JSON_KEY_FILAMENT_ID) {
                cloud_filament_id = v.clone();
            }
        }

        let result: CfgResult<Option<bool>> = (|| {
            let mut new_config: DynamicPrintConfig;
            let mut cloud_config = DynamicPrintConfig::new();
            let config_substitutions = cloud_config.load_string_map(&preset_values, rule)?;
            if !config_substitutions.is_empty() {
                substitutions.push(PresetConfigSubstitutions {
                    preset_name: name.to_string(),
                    preset_type: self.m_type,
                    preset_source: PresetConfigSubstitutionsSource::UserCloud,
                    preset_file: name.to_string(),
                    substitutions: config_substitutions,
                });
            }

            //BBS: use inherit config as the base
            let mut inherit_preset: Option<usize> = None;
            let inherits_value: Option<String> = cloud_config.option(BBL_JSON_KEY_INHERITS).map(|o| {
                o.as_any().downcast_ref::<ConfigOptionString>().expect("dynamic_cast<ConfigOptionString *>").value.clone()
            });
            if let Some(inherits_value) = &inherits_value {
                inherit_preset = self.find_preset_slot(inherits_value, false, true).map(|s| match s {
                    PresetSlot::Index(i) => i,
                    PresetSlot::Edited => unreachable!("find_preset(.., real = true)"),
                });
            }
            let default_preset = self.default_preset_for_idx(&cloud_config);
            if let Some(ip) = inherit_preset {
                new_config = self.m_presets[ip].config.clone();
                if cloud_filament_id == "null" {
                    cloud_filament_id = self.m_presets[ip].filament_id.clone();
                }
            } else {
                // We support custom root preset now
                if let Some(v) = &inherits_value {
                    if !v.is_empty() {
                        //we should skip this preset here
                        return Ok(None);
                    }
                }
                // Find a default preset for the config. The PrintPresetCollection provides different default preset based on the "printer_technology" field.
                new_config = self.m_presets[default_preset].config.clone();
            }

            if let Some(ip) = inherit_preset {
                extend_default_config_length(&mut cloud_config, &self.m_presets[ip].config, false, &DynamicPrintConfig::new())?;
                let mut extruder_id_name = String::new();
                let mut extruder_variant_name = String::new();
                let mut key_set1: &'static BTreeSet<String> = &empty_options;
                let mut key_set2: &'static BTreeSet<String> = &empty_options;
                Preset::get_extruder_names_and_keysets(self.m_type, &mut extruder_id_name, &mut extruder_variant_name, &mut key_set1, &mut key_set2);

                new_config.update_diff_values_to_child_config(&mut cloud_config, &extruder_id_name, &extruder_variant_name, key_set1, key_set2)?;
            } else {
                extend_default_config_length(&mut cloud_config, &DynamicPrintConfig::new(), false, &DynamicPrintConfig::new())?;
                new_config.apply(&cloud_config, false)?;
                extend_default_config_length(&mut new_config, &DynamicPrintConfig::new(), true, &self.m_presets[default_preset].config)?;
            }
            Preset::normalize(&mut new_config)?;
            // Report configuration fields, which are misplaced into a wrong group.
            let _incorrect_keys = Preset::remove_invalid_keys(&mut new_config, &self.m_presets[default_preset].config);
            if need_update {
                if self.m_presets[iter].name == self.m_edited_preset.name && self.m_presets[iter].is_dirty {
                    // Keep modifies when update from remote
                    let keys = self.m_edited_preset.config.base.diff(&self.m_presets[iter].config.base)?;
                    new_config.apply_only(&self.m_edited_preset.config, &keys, false)?;
                }
                let p = &mut self.m_presets[iter];
                p.config = new_config;
                p.updated_time = cloud_update_time;
                p.sync_info = "save".to_string();
                p.version = cloud_version.clone();
                p.user_id = cloud_user_id.clone();
                p.setting_id = cloud_setting_id.clone();
                p.base_id = cloud_base_id.clone();
                p.filament_id = cloud_filament_id.clone();
            } else {
                //create a new one
                let mut preset = Preset::new(self.m_type, name, false);
                preset.is_system = false;
                preset.loaded = true;
                preset.config = new_config;
                preset.updated_time = cloud_update_time;
                preset.sync_info = "save".to_string();
                preset.version = cloud_version.clone();
                preset.user_id = cloud_user_id.clone();
                preset.setting_id = cloud_setting_id.clone();
                preset.base_id = cloud_base_id.clone();
                preset.filament_id = cloud_filament_id.clone();

                let cur_index = iter;
                self.m_presets.insert(iter, preset);
                if cur_index <= self.m_idx_selected {
                    // size_t: size_t(-1) wraps to 0.
                    self.m_idx_selected = self.m_idx_selected.wrapping_add(1);
                }
            }
            Ok(Some(true))
        })();
        match result {
            Ok(Some(_)) => {}
            Ok(None) => {
                self.unlock();
                return Ok(false);
            }
            Err(err) => {
                let msg = format!("{}: {}", name, err.what());
                errors_cummulative += &msg;
                errors_cummulative += "\n";
            }
        }

        self.unlock();

        if !errors_cummulative.is_empty() {
            return Err(runtime_error(errors_cummulative));
        }

        Ok(!need_update)
    }

    /// Preset.cpp:2207-2219 `void PresetCollection::update_after_user_presets_loaded()`: re-sort and re-select
    pub fn update_after_user_presets_loaded(&mut self) {
        self.lock();
        let selected_name = self.get_selected_preset_name();
        self.sort_presets_from(self.m_num_default_presets);
        self.select_preset_by_name(&selected_name, false, false);
        self.unlock();
    }

    /// Preset.cpp:2221-2250 `bool PresetCollection::validate_preset(const std::string &preset_name, std::string &inherit_name)`
    pub fn validate_preset(&self, preset_name: &str, inherit_name: &str) -> bool {
        let mut it = self.find_preset_internal(preset_name);
        let mut found = it != self.m_presets.len()
            && self.m_presets[it].name == preset_name
            && (self.m_presets[it].is_system || self.m_presets[it].is_default);
        if !found {
            it = self.find_preset_renamed(preset_name);
            found = it != self.m_presets.len() && (self.m_presets[it].is_system || self.m_presets[it].is_default);
        }
        if !found {
            if !inherit_name.is_empty() {
                it = self.find_preset_internal(inherit_name);
                found = it != self.m_presets.len()
                    && self.m_presets[it].name == inherit_name
                    && (self.m_presets[it].is_system || self.m_presets[it].is_default);
            } else {
                //inherit is null , should not happen , just consider it as valid
                found = false;
            }
        }

        found
    }

    /// Preset.cpp:2253-2260 `Preset& PresetCollection::load_preset(const std::string &path, const std::string &name, const DynamicPrintConfig &config, bool select, Semver file_version)`:
    /// load a preset from an already parsed config file, insert it into the sorted sequence of
    /// presets and select it, losing previous modifications.
    pub fn load_preset(&mut self, path: &str, name: &str, config: &DynamicPrintConfig, select: bool, file_version: Semver) -> CfgResult<&mut Preset> {
        let mut cfg = self.default_preset(0).config.clone();
        let keys = cfg.keys();
        cfg.apply_only(config, &keys, true)?;
        self.load_preset_move(path, name, cfg, select, file_version)
    }

    /// Preset.cpp:2526-2551 `Preset& PresetCollection::load_preset(const std::string &path, const std::string &name, DynamicPrintConfig &&config, bool select, Semver file_version)`
    pub fn load_preset_move(&mut self, path: &str, name: &str, config: DynamicPrintConfig, select: bool, file_version: Semver) -> CfgResult<&mut Preset> {
        let it = self.load_preset_move_idx(path, name, config, select, file_version);
        Ok(&mut self.m_presets[it])
    }

    fn load_preset_move_idx(&mut self, path: &str, name: &str, config: DynamicPrintConfig, select: bool, file_version: Semver) -> usize {
        self.lock();
        let it = self.find_preset_internal(name);
        if it == self.m_presets.len() || self.m_presets[it].name != name {
            // The preset was not found. Create a new preset.
            // `m_presets.begin() + m_idx_selected >= it`: the deque iterator adds a signed
            // difference, size_t(-1) is begin() - 1.
            if (self.m_idx_selected as isize) >= it as isize {
                self.m_idx_selected = self.m_idx_selected.wrapping_add(1);
            }
            self.m_presets.insert(it, Preset::new(self.m_type, name, false));
        }
        let preset = &mut self.m_presets[it];
        preset.file = path.to_string();
        preset.config = config;
        preset.loaded = true;
        preset.is_dirty = false;

        //BBS
        if file_version.valid() {
            preset.version = file_version;
        }
        if select {
            self.select_preset_by_name(name, true, false);
        }
        self.unlock();
        it
    }

    /// Preset.cpp:2553-2601 `bool PresetCollection::clone_presets(std::vector<Preset const *> const &presets, std::vector<std::string> &failures, std::function<void(Preset &, Preset::Type &)> modifier, bool force_rewritten)`
    pub fn clone_presets(
        &mut self,
        presets: &[&Preset],
        failures: &mut Vec<String>,
        mut modifier: impl FnMut(&mut Preset, &mut Type),
        force_rewritten: bool,
    ) -> CfgResult<bool> {
        let mut new_presets: Vec<Preset> = Vec::new();
        for curr_preset in presets {
            new_presets.push((*curr_preset).clone());
            let preset = new_presets.last_mut().unwrap();
            preset.vendor = None;
            preset.renamed_from.clear();
            preset.setting_id.clear();
            preset.inherits_mut().clear();
            preset.is_default = false;
            preset.is_system = false;
            preset.is_external = false;
            preset.is_visible = true;
            preset.is_project_embedded = false;
            let mut ty = self.m_type;
            modifier(preset, &mut ty);
            if self.find_preset(&preset.name, false).is_some() && !force_rewritten {
                failures.push(preset.name.clone());
            }
            preset.file = self.path_for_preset(preset);
            if self.m_type == Type::TypePrint {
                *Preset::string_option_mut(&mut preset.config, "print_settings_id") = preset.name.clone();
            } else if self.m_type == Type::TypeFilament {
                strings_option_mut(&mut preset.config, "filament_settings_id").values[0] = preset.name.clone();
            } else if self.m_type == Type::TypePrinter {
                *Preset::string_option_mut(&mut preset.config, "printer_settings_id") = preset.name.clone();
            }
        }
        if !failures.is_empty() && !force_rewritten {
            return Ok(false);
        }
        self.lock();
        let old_name = self.get_edited_preset().name.clone();
        for mut preset in new_presets {
            preset.alias.clear();
            self.set_custom_preset_alias(&mut preset);
            preset.base_id.clear();
            let it = self.find_preset_internal(&preset.name);
            if it == self.m_presets.len() || self.m_presets[it].name != preset.name {
                self.m_presets.insert(it, preset);
                self.m_presets[it].save(None)?;
            } else if force_rewritten {
                self.m_presets[it] = preset;
                self.m_presets[it].save(None)?;
            }
        }
        self.select_preset_by_name(&old_name, true, false);
        self.unlock();
        Ok(true)
    }

    /// Preset.cpp:2603-2621 `bool PresetCollection::clone_presets_for_printer(std::vector<Preset const *> const &templates, std::vector<std::string> &failures, std::string const &printer, std::function<std::string(std::string)> create_filament_id, bool force_rewritten)`
    pub fn clone_presets_for_printer(
        &mut self,
        templates: &[&Preset],
        failures: &mut Vec<String>,
        printer: &str,
        create_filament_id: impl Fn(String) -> String,
        force_rewritten: bool,
    ) -> CfgResult<bool> {
        self.clone_presets(
            templates,
            failures,
            |preset: &mut Preset, ty: &mut Type| {
                let cut = preset.name.find(" @").unwrap_or(preset.name.len());
                let prefix = preset.name[..cut].replace('/', "-");
                preset.name = format!("{} @{}", prefix, printer);
                let compatible_printers = preset
                    .config
                    .option_mut("compatible_printers", false)
                    .ok()
                    .flatten()
                    .and_then(|o| o.as_any_mut().downcast_mut::<ConfigOptionStrings>())
                    .expect("dynamic_cast<ConfigOptionStrings *>(preset.config.option(\"compatible_printers\"))");
                compatible_printers.values = vec![printer.to_string()];
                preset.is_visible = true;
                if *ty == Type::TypeFilament {
                    preset.filament_id = create_filament_id(prefix);
                }
            },
            force_rewritten,
        )
    }

    /// Preset.cpp:2623-2647 `bool PresetCollection::clone_presets_for_filament(Preset const *const &preset, std::vector<std::string> &failures, std::string const &filament_name, std::string const &filament_id, const DynamicConfig &dynamic_config, const std::string &compatible_printers, bool force_rewritten)`
    #[allow(clippy::too_many_arguments)]
    pub fn clone_presets_for_filament(
        &mut self,
        preset: &Preset,
        failures: &mut Vec<String>,
        filament_name: &str,
        filament_id: &str,
        dynamic_config: &dyn ConfigBase,
        compatible_printers: &str,
        force_rewritten: bool,
    ) -> CfgResult<bool> {
        let presets: Vec<&Preset> = vec![preset];
        let mut error: Option<ConfigurationError> = None;
        let r = self.clone_presets(
            &presets,
            failures,
            |preset: &mut Preset, ty: &mut Type| {
                preset.name = format!("{} @{}", filament_name, compatible_printers);
                if *ty == Type::TypeFilament {
                    let keys: Vec<String> = ["filament_vendor", "compatible_printers", "filament_type"].iter().map(|s| s.to_string()).collect();
                    if let Err(e) = preset.config.apply_only(dynamic_config, &keys, true) {
                        error.get_or_insert(e);
                    }

                    preset.filament_id = filament_id.to_string();
                    let compatible = preset
                        .config
                        .option_mut("compatible_printers", false)
                        .ok()
                        .flatten()
                        .and_then(|o| o.as_any_mut().downcast_mut::<ConfigOptionStrings>())
                        .expect("dynamic_cast<ConfigOptionStrings *>(preset.config.option(\"compatible_printers\"))");
                    if compatible.values.is_empty() {
                        compatible.values.push(compatible_printers.to_string());
                    }
                }
            },
            force_rewritten,
        );
        if let Some(e) = error {
            return Err(e);
        }
        r
    }

    /// Preset.cpp:2649-2660 `std::map<std::string, std::vector<Preset const *>> PresetCollection::get_filament_presets() const`
    /// (the presets as their indices in m_presets).
    pub fn get_filament_presets(&self) -> BTreeMap<String, Vec<usize>> {
        let mut filament_presets: BTreeMap<String, Vec<usize>> = BTreeMap::new();
        for (i, preset) in self.m_presets.iter().enumerate() {
            if preset.is_user() {
                if preset.inherits().is_empty() {
                    filament_presets.entry(preset.filament_id.clone()).or_default().push(i);
                }
                continue;
            }
            if self.get_preset_base(preset).map_or(false, |b| std::ptr::eq(b, preset)) {
                filament_presets.entry(preset.filament_id.clone()).or_default().push(i);
            }
        }
        filament_presets
    }
}

/// `cfg.option<ConfigOptionStrings>(key, true)`
fn strings_option_mut<'a>(cfg: &'a mut DynamicPrintConfig, key: &str) -> &'a mut ConfigOptionStrings {
    cfg.option_typed_mut::<ConfigOptionStrings>(key, true)
        .expect("Preset: no definition")
        .expect("option<ConfigOptionStrings>(key, true) is null")
}

impl PresetCollection {
    /// Preset.cpp:2278-2524 `std::pair<Preset*, bool> PresetCollection::load_external_preset(...)`:
    /// load a preset from an already parsed config file, insert it into the sorted sequence of
    /// presets and select it, losing previous modifications. Only a single profile could be
    /// edited at at the same time, which introduces complexity when loading filament profiles
    /// for multi-extruder printers. Returns the preset and whether an existing preset was
    /// selected AND modified from config.
    #[allow(clippy::too_many_arguments)]
    pub fn load_external_preset(
        &mut self,
        // Path to the profile source file (a G-code, an AMF or 3MF file, a config file)
        path: &str,
        // Name of the profile, derived from the source file name.
        name: &str,
        // Original name of the profile, extracted from the loaded config. Empty, if the name has not been stored.
        original_name: &str,
        // Config to initialize the preset from. It may contain configs of all presets merged in a single dictionary!
        combined_config: &DynamicPrintConfig,
        //different settings list
        different_settings_list: &BTreeSet<String>,
        // Select the preset after loading?
        select: LoadAndSelect,
        file_version: Semver,
        filament_id: &str,
    ) -> CfgResult<(&mut Preset, bool)> {
        let (idx, modified) = self.load_external_preset_idx(path, name, original_name, combined_config, different_settings_list, select, file_version, filament_id)?;
        Ok((&mut self.m_presets[idx], modified))
    }

    #[allow(clippy::too_many_arguments)]
    fn load_external_preset_idx(
        &mut self,
        path: &str,
        name: &str,
        original_name: &str,
        combined_config: &DynamicPrintConfig,
        different_settings_list: &BTreeSet<String>,
        select: LoadAndSelect,
        file_version: Semver,
        filament_id: &str,
    ) -> CfgResult<(usize, bool)> {
        // Load the preset over a default preset, so that the missing fields are filled in from the default preset.
        let mut cfg = self.m_presets[self.default_preset_for_idx(combined_config)].config.clone();
        // OrcaSlicer: ignore print connection info from project
        cfg.erase("print_host");
        cfg.erase("print_host_webui");
        cfg.erase("printhost_apikey");
        cfg.erase("printhost_cafile");
        cfg.erase("printhost_user");
        cfg.erase("printhost_password");
        cfg.erase("printhost_port");

        let keys = cfg.keys();
        cfg.apply_only(combined_config, &keys, true)?;
        // `std::string &inherits = Preset::inherits(cfg);` is read through the option each time.
        let inherits = Preset::inherits_cfg(&mut cfg).clone();

        //BBS: add different settings check logic, replace the old system preset's default value with new system preset's default values
        let mut it = self.find_preset_internal(original_name);
        let mut found = it != self.m_presets.len() && self.m_presets[it].name == original_name;
        if !found {
            // Try to match the original_name against the "renamed_from" profile names of loaded system profiles.
            it = self.find_preset_renamed(original_name);
            found = it != self.m_presets.len();
        }

        let mut extruder_id_name = String::new();
        let mut extruder_variant_name = String::new();
        let mut key_set1: &'static BTreeSet<String> = &empty_options;
        let mut key_set2: &'static BTreeSet<String> = &empty_options;
        Preset::get_extruder_names_and_keysets(self.m_type, &mut extruder_id_name, &mut extruder_variant_name, &mut key_set1, &mut key_set2);

        if !inherits.is_empty() && !different_settings_list.is_empty() {
            let iter = self.find_preset_internal(&inherits);
            if iter != self.m_presets.len() && self.m_presets[iter].name == inherits {
                cfg.update_non_diff_values_to_base_config(
                    &mut self.m_presets[iter].config,
                    &keys,
                    different_settings_list,
                    &extruder_id_name,
                    &extruder_variant_name,
                    key_set1,
                    key_set2,
                )?;
            }
        } else if found && self.m_presets[it].is_system && !different_settings_list.is_empty() {
            cfg.update_non_diff_values_to_base_config(
                &mut self.m_presets[it].config,
                &keys,
                different_settings_list,
                &extruder_id_name,
                &extruder_variant_name,
                key_set1,
                key_set2,
            )?;
        }
        let inherits = Preset::inherits_cfg(&mut cfg).clone();

        if select == LoadAndSelect::Never {
            // Some filament profile has been selected and modified already.
            // Check whether this profile is equal to the modified edited profile.
            let edited = self.get_edited_preset();
            if (edited.name == original_name || edited.name == inherits) && profile_print_params_same(&edited.config, &cfg)? {
                // Just point to that already selected and edited profile.
                let edited_name = edited.name.clone();
                return Ok((self.find_preset_internal(&edited_name), false));
            }
        }
        // Is there a preset already loaded with the name stored inside the config?
        if found && profile_print_params_same(&self.m_presets[it].config, &cfg)? {
            // The preset exists and it matches the values stored inside config.
            if select == LoadAndSelect::Always {
                self.select_preset(it);
            }
            //BBS: set the preset to visible
            if !self.m_presets[it].is_visible {
                self.m_presets[it].is_visible = true;
            }
            return Ok((it, false));
        }
        if !found && select != LoadAndSelect::Never && !inherits.is_empty() {
            // Try to use a system profile as a base to select the system profile
            // and override its settings with the loaded ones.
            it = self.find_preset_internal(&inherits);
            found = it != self.m_presets.len() && self.m_presets[it].name == inherits;
            if found && profile_print_params_same(&self.m_presets[it].config, &cfg)? {
                // The system preset exists and it matches the values stored inside config.
                if select == LoadAndSelect::Always {
                    self.select_preset(it);
                }
                //BBS: set the preset to visible
                if !self.m_presets[it].is_visible {
                    self.m_presets[it].is_visible = true;
                }
                return Ok((it, false));
            }
        }
        if found {
            //BBS: only select preset for always
            if select == LoadAndSelect::Always {
                // Select the existing preset and override it with new values, so that
                // the differences will be shown in the preset editor against the referenced profile.
                self.select_preset(it);
                // The source config may contain keys from many possible preset types. Just copy those that relate to this preset.
                self.get_edited_preset_mut().config.apply_only(&cfg, &keys, true)?;
                self.update_dirty()?;
                self.update_saved_preset_from_current_preset();
                //BBS: set the preset to visible
                if !self.m_presets[it].is_visible {
                    self.m_presets[it].is_visible = true;
                }
                let dirty = self.get_edited_preset().is_dirty;
                return Ok((it, dirty));
            }

            //BBS: for other filaments under AMS
            if self.m_presets[it].is_project_embedded {
                //update the properties back to the preset
                self.m_presets[it].config.apply_only(&cfg, &keys, true)?;
                self.m_presets[it].is_dirty = false;

                return Ok((it, false));
            }
            if inherits.is_empty() {
                // Update the "inherits" field.
                // There is a profile with the same name already loaded. Should we update the "inherits" field?
                let v = if self.m_presets[it].vendor.is_some() {
                    self.m_presets[it].name.clone()
                } else {
                    self.m_presets[it].inherits_mut().clone()
                };
                *Preset::inherits_cfg(&mut cfg) = v;
            }
        }
        let inherits = Preset::inherits_cfg(&mut cfg).clone();

        // The external preset does not match an internal preset, load the external preset.
        let mut new_name;
        //BBS: add project embedded preset logic
        //BBS: refine the name logic
        let mut idx: usize = 0;
        loop {
            let mut prefix = String::new();
            if original_name.is_empty() {
                if !inherits.is_empty() {
                    if idx == 0 {
                        prefix = inherits.clone();
                    } else {
                        prefix = format!("{}-{}", inherits, idx);
                    }
                } else if idx > 0 {
                    prefix = idx.to_string();
                }
            } else {
                let reduced_name = original_name.to_string();
                if idx == 0 {
                    prefix = reduced_name;
                } else {
                    prefix = format!("{}-{}", reduced_name, idx);
                }
            }
            new_name = format!("{}({})", prefix, name);
            it = self.find_preset_internal(&new_name);
            if it == self.m_presets.len() || self.m_presets[it].name != new_name {
                // Unique profile name. Insert a new profile.
                break;
            }
            if profile_print_params_same(&self.m_presets[it].config, &cfg)? {
                // The preset exists and it matches the values stored inside config.
                if select == LoadAndSelect::Always {
                    self.select_preset(it);
                }
                return Ok((it, false));
            }
            // Form another profile name.
            idx += 1;
        }
        // Insert a new profile.
        //BBS: add project embedded preset logic
        let from_project = iends_with(name, ".3mf");
        if self.m_type == Type::TypePrint {
            *Preset::string_option_mut(&mut cfg, "print_settings_id") = new_name.clone();
        } else if self.m_type == Type::TypeFilament {
            strings_option_mut(&mut cfg, "filament_settings_id").values[0] = new_name.clone();
        } else if self.m_type == Type::TypePrinter {
            *Preset::string_option_mut(&mut cfg, "printer_settings_id") = new_name.clone();
        }
        let pidx = self.load_preset_move_idx(path, &new_name, cfg, select == LoadAndSelect::Always, Semver::new());
        self.m_presets[pidx].is_external = true;
        self.m_presets[pidx].version = file_version;
        if !filament_id.is_empty() {
            self.m_presets[pidx].filament_id = filament_id.to_string();
        } else {
            // `inherits` refers to the option of the config moved into the preset.
            let inherits = self.m_presets[pidx].inherits_mut().clone();
            if !inherits.is_empty() {
                if let Some(PresetSlot::Index(parent)) = self.find_preset_slot(&inherits, false, true) {
                    self.m_presets[pidx].filament_id = self.m_presets[parent].filament_id.clone();
                }
            }
        }
        if from_project {
            self.m_presets[pidx].is_project_embedded = true;
        } else {
            //external config
            let file = self.path_for_preset(&self.m_presets[pidx]);
            self.m_presets[pidx].file = file;
            //BBS: save full config here for external
            //we can not reach here
            self.m_presets[pidx].save(None)?;
        }
        self.get_selected_preset_mut();
        if self.m_idx_selected == pidx {
            let pe = self.m_presets[pidx].is_project_embedded;
            self.get_edited_preset_mut().is_external = true;
            self.get_edited_preset_mut().is_project_embedded = pe;
        }

        Ok((pidx, false))
    }

    /// Preset.cpp:2662-2781 `void PresetCollection::save_current_preset(const std::string &new_name, bool detach, bool save_to_project, Preset *_curr_preset, std::map<std::string, std::string>* extra_map)`:
    /// save the preset under a new name. If the name is different from the old one, a new
    /// preset is stored into the list of presets. All presets are marked as not modified and the
    /// new preset is activated.
    pub fn save_current_preset(
        &mut self,
        new_name: &str,
        detach: bool,
        save_to_project: bool,
        curr_preset_in: Option<&Preset>,
        extra_map: Option<&BTreeMap<String, String>>,
    ) -> CfgResult<()> {
        let curr_preset = match curr_preset_in {
            Some(p) => p.clone(),
            None => self.m_edited_preset.clone(),
        };
        //BBS: add lock logic for sync preset in background
        let final_inherits: String;
        self.lock();
        // 1) Find the preset with a new_name or create a new one,
        // initialize it with the edited config.
        let it = self.find_preset_internal(new_name);
        if it != self.m_presets.len() && self.m_presets[it].name == new_name {
            // Preset with the same name found.
            //BBS: add project embedded preset logic
            if self.m_presets[it].is_default || self.m_presets[it].is_system {
                // Cannot overwrite the default preset.
                self.unlock();
                return Ok(());
            }
            let m_type = self.m_type;
            let preset = &mut self.m_presets[it];
            // Overwriting an existing preset.
            preset.config = curr_preset.config;
            // The newly saved preset will be activated -> make it visible.
            preset.is_visible = true;
            //TODO: remove the detach logic
            if detach {
                // Clear the link to the parent profile.
                preset.vendor = None;
                preset.inherits_mut().clear();
                preset.alias.clear();
                preset.renamed_from.clear();
            }
            //BBS: add lock logic for sync preset in background

            if m_type == Type::TypePrint {
                *Preset::string_option_mut(&mut preset.config, "print_settings_id") = new_name.to_string();
            } else if m_type == Type::TypeFilament {
                strings_option_mut(&mut preset.config, "filament_settings_id").values[0] = new_name.to_string();
            } else if m_type == Type::TypePrinter {
                *Preset::string_option_mut(&mut preset.config, "printer_settings_id") = new_name.to_string();
                if let Some(extra_map) = extra_map {
                    for (k, v) in extra_map {
                        *Preset::string_option_mut(&mut preset.config, k) = v.clone();
                    }
                }
            }
            final_inherits = preset.inherits_mut().clone();
            self.unlock();
            // TODO: apply change from custom root to devided presets.
            if self.m_presets[it].inherits_mut().is_empty() {
                let parent = self.m_presets[it].clone();
                for preset2 in self.m_presets.iter_mut() {
                    if *preset2.inherits_mut() == parent.name {
                        preset2.reload(&parent);
                    }
                }
            }
        } else {
            // Creating a new preset.
            self.m_presets.insert(it, curr_preset);
            let is_base = self.is_base_preset(&self.m_presets[it]);
            let m_type = self.m_type;
            {
                let preset = &mut self.m_presets[it];
                let old_name = preset.name.clone();
                preset.name = new_name.to_string();
                preset.vendor = None;
                preset.alias.clear();
                preset.renamed_from.clear();
                preset.setting_id.clear();
                if detach {
                    // Clear the link to the parent profile.
                    preset.inherits_mut().clear();
                } else if is_base {
                    *preset.inherits_mut() = old_name;
                }
                preset.is_default = false;
                preset.is_system = false;
                preset.is_external = false;
            }
            let file = self.path_for_preset(&self.m_presets[it]);
            let preset = &mut self.m_presets[it];
            preset.file = file;
            // The newly saved preset will be activated -> make it visible.
            preset.is_visible = true;
            // Just system presets have aliases
            preset.alias.clear();
            //BBS: add project embedded preset logic
            if save_to_project {
                preset.is_project_embedded = true;
            } else {
                preset.is_project_embedded = false;
            }
            if m_type == Type::TypePrint {
                *Preset::string_option_mut(&mut preset.config, "print_settings_id") = new_name.to_string();
            } else if m_type == Type::TypeFilament {
                strings_option_mut(&mut preset.config, "filament_settings_id").values[0] = new_name.to_string();
            } else if m_type == Type::TypePrinter {
                *Preset::string_option_mut(&mut preset.config, "printer_settings_id") = new_name.to_string();
                if let Some(extra_map) = extra_map {
                    for (k, v) in extra_map {
                        *Preset::string_option_mut(&mut preset.config, k) = v.clone();
                    }
                }
            }
            //BBS: add lock logic for sync preset in background
            final_inherits = preset.inherits_mut().clone();
            self.unlock();
        }
        // 2) Activate the saved preset.
        self.select_preset_by_name(new_name, true, false);
        // 2) Store the active preset to disk.
        //BBS: only save difference for user preset
        let mut parent_preset: Option<usize> = None;
        if !final_inherits.is_empty() {
            parent_preset = match self.find_preset_slot(&final_inherits, false, true) {
                Some(PresetSlot::Index(i)) => Some(i),
                _ => None,
            };
            if let Some(parent) = parent_preset {
                if self.get_selected_preset_mut().base_id.is_empty() {
                    let sid = self.m_presets[parent].setting_id.clone();
                    self.get_selected_preset_mut().base_id = sid;
                }
            }
        }
        if let Some(parent) = parent_preset {
            let parent_config = self.m_presets[parent].config.clone();
            self.get_selected_preset_mut().save(Some(&parent_config))?;
        } else {
            self.get_selected_preset_mut().save(None)?;
        }
        Ok(())
    }

    /// Preset.cpp:2783-2815 `bool PresetCollection::delete_current_preset()`: delete the current
    /// preset, activate the first visible preset. Returns true if the preset was deleted successfully.
    pub fn delete_current_preset(&mut self) -> bool {
        self.get_selected_preset_mut();
        let sel = self.m_idx_selected;
        if self.m_presets[sel].is_default {
            return false;
        }

        let is_base = {
            let s = &self.m_presets[sel];
            self.get_preset_base(s).map_or(false, |b| std::ptr::eq(b, s))
        };
        if is_base {
            let selected_name = self.m_presets[sel].name.clone();
            for preset2 in self.m_presets.iter_mut() {
                if *preset2.inherits_mut() == selected_name {
                    return false;
                }
            }
        }

        //BBS: add project embedded preset logic and refine is_external
        if !self.m_presets[sel].is_system {
            //BBS Erase the preset file.
            self.m_presets[sel].remove_files();
        }
        //BBS: add lock logic for sync preset in background
        self.lock();
        // Remove the preset from the list.
        self.m_presets.remove(self.m_idx_selected);
        self.unlock();

        // Find the next visible preset.
        let mut new_selected_idx = self.m_idx_selected;
        if new_selected_idx < self.m_presets.len() {
            while new_selected_idx < self.m_presets.len() && !self.m_presets[new_selected_idx].is_visible {
                new_selected_idx += 1;
            }
        }
        if new_selected_idx == self.m_presets.len() {
            new_selected_idx -= 1;
            while new_selected_idx > 0 && !self.m_presets[new_selected_idx].is_visible {
                new_selected_idx -= 1;
            }
        }
        self.select_preset(new_selected_idx);
        true
    }

    /// Preset.cpp:2817-2838 `bool PresetCollection::delete_preset(const std::string& name)`.
    /// The position the binary search gives is deleted without comparing the name, as in the C++.
    pub fn delete_preset(&mut self, name: &str) -> bool {
        let it = self.find_preset_internal(name);

        if self.m_presets[it].is_default {
            return false;
        }
        //BBS: add project embedded preset logic and refine is_external
        if !self.m_presets[it].is_system {
            self.m_presets[it].remove_files();
        }
        //BBS: add lock logic for sync preset in background
        self.lock();
        let p = self.m_presets[it].clone();
        self.set_printer_hold_alias(&p.alias, &p, true);
        // `it = m_presets.erase(it)`: the next preset, at the same index.
        self.m_presets.remove(it);
        // std::distance (ptrdiff_t) < m_idx_selected (size_t): compared as size_t.
        if it < self.m_idx_selected {
            self.m_idx_selected = self.m_idx_selected.wrapping_sub(1);
        }
        self.unlock();

        true
    }

    /// Preset.cpp:3051-3059 `void PresetCollection::set_default_suppressed(bool default_suppressed)`:
    /// enable / disable the "- default -" preset.
    pub fn set_default_suppressed(&mut self, default_suppressed: bool) {
        if self.m_default_suppressed != default_suppressed {
            self.m_default_suppressed = default_suppressed;
            let default_visible = !default_suppressed || self.m_idx_selected < self.m_num_default_presets;
            for i in 0..self.m_num_default_presets {
                self.m_presets[i].is_visible = default_visible;
            }
        }
    }
    /// Preset.hpp:582
    pub fn is_default_suppressed(&self) -> bool {
        self.m_default_suppressed
    }

    /// Preset.hpp:587-593 `Preset& get_selected_preset()`: the selected preset, without the user
    /// modifications applied. BBS fix crash when m_idx_selected == -1, give a default value.
    pub fn get_selected_preset_mut(&mut self) -> &mut Preset {
        if self.m_idx_selected >= self.m_presets.len() {
            let idx = self.first_visible_idx();
            self.select_preset(idx);
        }
        &mut self.m_presets[self.m_idx_selected]
    }
    /// Preset.hpp:594 `const Preset& get_selected_preset() const`
    pub fn get_selected_preset(&self) -> &Preset {
        &self.m_presets[self.m_idx_selected]
    }
    /// Preset.hpp:595
    pub fn get_selected_idx(&self) -> usize {
        self.m_idx_selected
    }
    /// Preset.hpp:597-601: the name of the selected preset, or an empty string if no preset is selected.
    pub fn get_selected_preset_name(&self) -> String {
        if self.m_idx_selected == usize::MAX || self.m_idx_selected >= self.m_presets.len() {
            return String::new();
        }
        self.get_selected_preset().name.clone()
    }

    /// Preset.cpp:2840-2869 `const Preset* PresetCollection::get_selected_preset_parent() const`:
    /// for the current edited preset, the parent preset if there is one. The parent preset may
    /// be a system preset or a user preset, which will be reflected by the UI.
    pub fn get_selected_preset_parent(&self) -> Option<&Preset> {
        if self.get_selected_idx() == usize::MAX {
            // This preset collection has no preset activated yet. Only the get_edited_preset() is valid.
            return None;
        }

        let selected_preset = self.get_selected_preset();
        if self.get_preset_base(selected_preset).map_or(false, |b| std::ptr::eq(b, selected_preset)) {
            return Some(selected_preset);
        }

        let edited_preset = self.get_edited_preset();
        let inherits = edited_preset.inherits();
        let mut preset: Option<&Preset>;
        if inherits.is_empty() {
            if selected_preset.is_external {
                return None;
            }
            preset = Some(self.default_preset(
                if self.m_type == Type::TypePrinter && edited_preset.printer_technology() == PrinterTechnology::PtSLA { 1 } else { 0 },
            ));
        } else {
            preset = self.find_preset(inherits, false);
        }
        if preset.is_none() {
            // Resolve the "renamed_from" field.
            let it = self.find_preset_renamed(inherits);
            if it != self.m_presets.len() {
                preset = Some(&self.m_presets[it]);
            }
        }
        //BBS: add project embedded preset logic and refine is_external
        preset
    }

    /// Preset.cpp:2871-2893 `const Preset* PresetCollection::get_preset_parent(const Preset& child) const`:
    /// the parent preset for a child preset, based on the "inherits" field of a child, where the
    /// "inherits" profile name is searched for in both m_presets and m_map_system_profile_renamed.
    pub fn get_preset_parent<'a>(&'a self, child: &Preset) -> Option<&'a Preset> {
        let inherits = child.inherits();
        if inherits.is_empty() {
            return None;
        }
        let mut preset = self.find_preset(inherits, false);
        if preset.is_none() {
            let it = self.find_preset_renamed(inherits);
            if it != self.m_presets.len() {
                preset = Some(&self.m_presets[it]);
            }
        }
        match preset {
            // not found
            None => None,
            // this should not happen, however people are creative, see GH #4996
            Some(p) if std::ptr::eq(p, child) => None,
            Some(p) => Some(p),
        }
    }

    /// Preset.cpp:2895-2909 `const Preset *PresetCollection::get_preset_base(const Preset &child) const`
    pub fn get_preset_base<'a>(&'a self, child: &'a Preset) -> Option<&'a Preset> {
        if child.is_system || child.is_default {
            return Some(child);
        }
        // Handle user preset
        if child.inherits().is_empty() {
            return Some(child); // this is user root
        }
        let inherits = self.find_preset(child.inherits(), false);

        // Guard against a self-referential "inherits" (corrupt data where inherits == own name):
        // find_preset() resolves back to child, so recursing would loop forever. Treat it as its own root.
        if let Some(i) = inherits {
            if std::ptr::eq(i, child) {
                return Some(child);
            }
        }

        match inherits {
            Some(i) => self.get_preset_base(i),
            None => None,
        }
    }

    /// Preset.cpp:2911-2918 `bool PresetCollection::is_bbl_brand_filament(const Preset &preset) const`:
    /// true when the inheritance chain roots in a Bambu Lab branded system filament preset.
    pub fn is_bbl_brand_filament(&self, preset: &Preset) -> bool {
        let base = match self.get_preset_base(preset) {
            Some(b) if b.is_system => b,
            _ => return false,
        };
        match base.config.option_typed::<ConfigOptionStrings>("filament_vendor") {
            Some(vendor) => !vendor.values.is_empty() && vendor.values[0] == "Bambu Lab",
            None => false,
        }
    }

    /// Preset.hpp:614-615
    pub fn get_edited_preset(&self) -> &Preset {
        &self.m_edited_preset
    }
    /// Preset.hpp:614
    pub fn get_edited_preset_mut(&mut self) -> &mut Preset {
        &mut self.m_edited_preset
    }

    /// Preset.cpp:2920-2933 `PresetWithVendorProfile PresetCollection::get_preset_with_vendor_profile(const Preset &preset) const`:
    /// the vendor of the first parent profile, for which the vendor is defined, or null if such
    /// profile does not exist.
    pub fn get_preset_with_vendor_profile<'a>(&'a self, preset: &'a Preset) -> PresetWithVendorProfile<'a> {
        let mut p: Option<&'a Preset> = Some(preset);
        let mut v: Option<&'a VendorProfile> = None;
        while let Some(pp) = p {
            if let Some(vendor) = &pp.vendor {
                v = Some(&**vendor);
                break;
            }
            p = self.get_preset_parent(pp);
        }
        PresetWithVendorProfile::new(preset, v)
    }
    /// Preset.hpp:622
    pub fn get_edited_preset_with_vendor_profile(&self) -> PresetWithVendorProfile<'_> {
        self.get_preset_with_vendor_profile(self.get_edited_preset())
    }

    /// Preset.cpp:2935-2965 `const std::string& PresetCollection::get_preset_name_by_alias(const std::string& alias) const`.
    /// BBS multi-diameter: when several nozzle-diameter variants share an alias and are all
    /// compatible, prefer the variant bound to the active printer preset (the main diameter).
    /// Fall back to the first compatible variant otherwise.
    pub fn get_preset_name_by_alias(&self, alias: &str) -> String {
        let mut fallback: Option<&String> = None;
        // Find the 1st profile name with the alias; continue over all profile names with the same alias.
        for (first, second) in self.m_map_alias_to_profile_name.range::<str, _>((std::ops::Bound::Included(alias), std::ops::Bound::Unbounded)) {
            if first != alias {
                break;
            }
            for preset_name in second {
                let it_preset = self.find_preset_internal(preset_name);
                if it_preset != self.m_presets.len()
                    && self.m_presets[it_preset].name == *preset_name
                    && self.m_presets[it_preset].is_visible
                    && (self.m_presets[it_preset].is_compatible || it_preset == self.m_idx_selected)
                {
                    if self.m_prefer_active_printer_alias && !self.m_active_printer_name.is_empty() {
                        if let Some(compatible_printers) = strings_option(&self.m_presets[it_preset].config, "compatible_printers") {
                            if compatible_printers.values.iter().any(|n| *n == self.m_active_printer_name) {
                                return self.m_presets[it_preset].name.clone();
                            }
                        }
                    }
                    if fallback.is_none() {
                        fallback = Some(&self.m_presets[it_preset].name);
                    }
                }
            }
        }

        if let Some(f) = fallback {
            return f.clone();
        }

        alias.to_string()
    }

    /// Preset.cpp:2967-2973 `const std::string* PresetCollection::get_preset_name_renamed(const std::string &old_name) const`
    pub fn get_preset_name_renamed(&self, old_name: &str) -> Option<&String> {
        self.m_map_system_profile_renamed.get(old_name)
    }

    /// Preset.cpp:2975-2995 `bool PresetCollection::is_alias_exist(const std::string &alias, Preset* preset)`
    pub fn is_alias_exist(&self, alias: &str, preset: Option<&Preset>) -> bool {
        if !self.m_map_alias_to_profile_name.contains_key(alias) {
            return false;
        }
        let preset = match preset {
            None => return true,
            Some(p) => p,
        };

        let compatible_printers = match strings_option(&preset.config, "compatible_printers") {
            None => return true,
            Some(c) => c,
        };

        for printer_name in &compatible_printers.values {
            if let Some(aliases) = self.m_printer_hold_alias.get(printer_name) {
                if aliases.contains(alias) {
                    return true;
                }
            }
        }
        false
    }

    /// Preset.hpp:630 `const std::deque<Preset>& get_presets() const`: used to update
    /// preset_choice from Tab.
    pub fn get_presets(&self) -> &Vec<Preset> {
        &self.m_presets
    }
    /// Preset.hpp:631
    pub fn get_idx_selected(&self) -> usize {
        self.m_idx_selected
    }
    /// Preset.cpp:2997-2999 `const std::string& PresetCollection::get_suffix_modified()`
    pub fn get_suffix_modified() -> String {
        g_suffix_modified_get()
    }

    /// Preset.hpp:635-636 `Preset& default_preset(size_t idx = 0)` (the assert is compiled out).
    pub fn default_preset(&self, idx: usize) -> &Preset {
        &self.m_presets[idx]
    }
    /// Preset.hpp:635
    pub fn default_preset_mut(&mut self, idx: usize) -> &mut Preset {
        &mut self.m_presets[idx]
    }

    /// The index of `default_preset_for(config)`: Preset.hpp:637 for a PresetCollection,
    /// Preset.cpp:3614-3618 for a PrinterPresetCollection (see `m_printer_collection`).
    fn default_preset_for_idx(&self, config: &DynamicPrintConfig) -> usize {
        if self.m_printer_collection {
            // config.opt<ConfigOptionEnumGeneric>("printer_technology"): a dynamic_cast.
            let opt_printer_technology = config.opt::<ConfigOptionEnumGeneric>("printer_technology");
            match opt_printer_technology {
                None => 0,
                Some(o) if o.value == PrinterTechnology::PtFFF.to_int() => 0,
                Some(_) => 1,
            }
        } else {
            0
        }
    }
    /// Preset.hpp:637 `virtual const Preset& default_preset_for(const DynamicPrintConfig &config) const`
    pub fn default_preset_for(&self, config: &DynamicPrintConfig) -> &Preset {
        self.default_preset(self.default_preset_for_idx(config))
    }

    /// Preset.hpp:639-642 `Preset& preset(size_t idx, bool real = false)`: a preset by an index.
    /// If the preset is active, the edited copy is returned unless `real`.
    pub fn preset_mut(&mut self, idx: usize, real: bool) -> &mut Preset {
        if real {
            return &mut self.m_presets[idx];
        }
        if idx == self.m_idx_selected {
            &mut self.m_edited_preset
        } else {
            &mut self.m_presets[idx]
        }
    }
    /// Preset.hpp:643 `const Preset& preset(size_t idx) const`
    pub fn preset(&self, idx: usize) -> &Preset {
        self.preset_real(idx, false)
    }
    fn preset_real(&self, idx: usize, real: bool) -> &Preset {
        match self.preset_slot(idx, real) {
            PresetSlot::Index(i) => &self.m_presets[i],
            PresetSlot::Edited => &self.m_edited_preset,
        }
    }
    fn preset_slot(&self, idx: usize, real: bool) -> PresetSlot {
        if real {
            return PresetSlot::Index(idx);
        }
        if idx == self.m_idx_selected {
            PresetSlot::Edited
        } else {
            PresetSlot::Index(idx)
        }
    }

    /// Preset.hpp:644-648 `void discard_current_changes()`
    pub fn discard_current_changes(&mut self) {
        self.m_presets[self.m_idx_selected].reset_dirty();
        self.m_edited_preset = self.m_presets[self.m_idx_selected].clone();
    }

    /// Preset.cpp:3001-3010 `Preset* PresetCollection::find_preset(const std::string &name, bool first_visible_if_not_found, bool real)`:
    /// where the preset of that name is: the edited copy when it is the selected one (unless
    /// `real`), the first visible one when not found if asked, or nothing.
    fn find_preset_slot(&self, name: &str, first_visible_if_not_found: bool, real: bool) -> Option<PresetSlot> {
        let key = Preset::new(self.m_type, name, false);
        let it = self.find_preset_internal(name);
        // Ensure that a temporary copy is returned if the preset found is currently selected.
        if it != self.m_presets.len() && self.m_presets[it].name == key.name {
            Some(self.preset_slot(it, real))
        } else if first_visible_if_not_found {
            Some(self.preset_slot(self.first_visible_idx(), false))
        } else {
            None
        }
    }
    /// Preset.cpp:3003 `Preset* find_preset(const std::string &name, bool first_visible_if_not_found, bool real)`
    pub fn find_preset_mut(&mut self, name: &str, first_visible_if_not_found: bool, real: bool) -> Option<&mut Preset> {
        match self.find_preset_slot(name, first_visible_if_not_found, real)? {
            PresetSlot::Index(i) => Some(&mut self.m_presets[i]),
            PresetSlot::Edited => Some(&mut self.m_edited_preset),
        }
    }
    /// The non-const `find_preset` read through a shared borrow.
    pub fn find_preset_real(&self, name: &str, first_visible_if_not_found: bool, real: bool) -> Option<&Preset> {
        match self.find_preset_slot(name, first_visible_if_not_found, real)? {
            PresetSlot::Index(i) => Some(&self.m_presets[i]),
            PresetSlot::Edited => Some(&self.m_edited_preset),
        }
    }
    /// Preset.hpp:654-655 `const Preset* find_preset(const std::string &name, bool first_visible_if_not_found = false) const`
    pub fn find_preset(&self, name: &str, first_visible_if_not_found: bool) -> Option<&Preset> {
        self.find_preset_real(name, first_visible_if_not_found, false)
    }

    /// Preset.cpp:3012-3038 `size_t PresetCollection::first_visible_idx() const`: the index of
    /// the first visible preset. BBS: set first visible filament to PLA.
    pub fn first_visible_idx(&self) -> usize {
        let mut first_visible: usize = usize::MAX;
        let mut idx = if self.m_default_suppressed { self.m_num_default_presets } else { 0 };
        while idx < self.m_presets.len() {
            if self.m_presets[idx].is_visible && self.m_presets[idx].get_printer_id() == "BBL" {
                if first_visible == usize::MAX {
                    first_visible = idx;
                }
                if self.m_type != Type::TypeFilament {
                    break;
                } else if self.m_presets[idx].name.contains("PLA") {
                    first_visible = idx;
                    break;
                }
            }
            idx += 1;
        }
        if first_visible == usize::MAX {
            if self.m_presets.len() > 1 && self.m_default_suppressed {
                first_visible = if self.m_presets.len() == self.m_num_default_presets { 0 } else { self.m_num_default_presets };
            } else {
                first_visible = 0;
            }
        }
        first_visible
    }

    /// Preset.hpp:660-685 `template<typename PreferedCondition> size_t first_compatible_idx(PreferedCondition prefered_condition) const`:
    /// the index of the first compatible preset. Certainly at least the '- default -' preset
    /// shall be compatible. If one of the prefered_alternates is compatible, select it.
    pub fn first_compatible_idx_with<F: Fn(&Preset) -> i32>(&self, prefered_condition: F) -> usize {
        let mut i = if self.m_default_suppressed { self.m_num_default_presets } else { 0 };
        let n = self.m_presets.len();
        let mut i_compatible = n;
        let mut match_quality: i32 = -1;
        while i < n {
            // Since we use the filament selection from Wizard, it's needed to control the preset visibility too
            if self.m_presets[i].is_compatible {
                let this_match_quality = prefered_condition(&self.m_presets[i]);
                if this_match_quality > match_quality {
                    if match_quality == i32::MAX {
                        // Better match will not be found.
                        return i;
                    }
                    // Store the first compatible profile with highest match quality into i_compatible.
                    i_compatible = i;
                    match_quality = this_match_quality;
                }
            }
            i += 1;
        }
        if i_compatible == n {
            // No compatible preset found, return the default preset.
            0
        } else {
            // Compatible preset found.
            i_compatible
        }
    }
    /// Preset.hpp:687 `size_t first_compatible_idx() const`
    pub fn first_compatible_idx(&self) -> usize {
        self.first_compatible_idx_with(|_| 0)
    }

    /// Preset.cpp:3040-3049 `std::vector<std::string> PresetCollection::diameters_of_selected_printer()`
    pub fn diameters_of_selected_printer(&self) -> Vec<String> {
        let mut diameters: BTreeSet<String> = BTreeSet::new();
        let printer_model = self.m_edited_preset.config.opt_string("printer_model").clone();
        for preset in &self.m_presets {
            if *preset.config.opt_string("printer_model") == printer_model {
                diameters.insert(preset.config.opt_string("printer_variant").clone());
            }
        }
        diameters.into_iter().collect()
    }

    /// Preset.hpp:693 `Preset& first_visible()`
    pub fn first_visible(&self) -> &Preset {
        self.preset(self.first_visible_idx())
    }
    /// Preset.hpp:695 `Preset& first_compatible()`
    pub fn first_compatible(&self) -> &Preset {
        self.preset(self.first_compatible_idx())
    }

    /// Preset.hpp:701: number of presets including the "- default -" preset.
    pub fn size(&self) -> usize {
        self.m_presets.len()
    }
    /// Preset.hpp:702
    pub fn has_defaults_only(&self) -> bool {
        self.m_presets.len() <= self.m_num_default_presets
    }

    /// Preset.hpp:705-713 `template<typename PreferedCondition> void update_compatible(...)`: for
    /// Print / Filament presets, disable those, which are not compatible with the printer.
    pub fn update_compatible_with<F: Fn(&Preset) -> i32>(
        &mut self,
        active_printer: &PresetWithVendorProfile,
        active_print: Option<&PresetWithVendorProfile>,
        select_other_if_incompatible: PresetSelectCompatibleType,
        prefered_condition: F,
    ) {
        if self.update_compatible_internal(active_printer, active_print, select_other_if_incompatible) == usize::MAX {
            // Find some other compatible preset, or the "-- default --" preset.
            let index = self.first_compatible_idx_with(prefered_condition);
            self.select_preset(index);
        }
    }
    /// Preset.hpp:714-715
    pub fn update_compatible(
        &mut self,
        active_printer: &PresetWithVendorProfile,
        active_print: Option<&PresetWithVendorProfile>,
        select_other_if_incompatible: PresetSelectCompatibleType,
    ) {
        self.update_compatible_with(active_printer, active_print, select_other_if_incompatible, |_| 0);
    }

    /// Preset.hpp:720: BBS multi-diameter: extra printer preset names (sibling nozzle-diameter
    /// variants selected via "set nozzle diameter") that are additionally accepted as compatible
    /// during the next update_compatible() pass.
    pub fn set_extra_compatible_printers(&mut self, printer_names: &[String]) {
        self.m_extra_compatible_printers = printer_names.to_vec();
    }
    /// Preset.hpp:721
    pub fn clear_extra_compatible_printers(&mut self) {
        self.m_extra_compatible_printers.clear();
    }

    /// Preset.hpp:723
    pub fn num_visible(&self) -> usize {
        self.m_presets.iter().filter(|p| p.is_visible).count()
    }

    /// Preset.hpp:726-727: compare the content of get_selected_preset() with get_edited_preset()
    /// configs, return true if they differ.
    pub fn current_is_dirty(&self) -> CfgResult<bool> {
        PresetCollection::is_dirty(Some(self.get_edited_preset()), Some(self.get_selected_preset()))
    }
    /// Preset.hpp:729-730: the list of keys where they differ.
    pub fn current_dirty_options(&self, deep_compare: bool) -> CfgResult<Vec<String>> {
        PresetCollection::dirty_options(Some(self.get_edited_preset()), Some(self.get_selected_preset()), deep_compare)
    }
    /// Preset.hpp:732-733
    pub fn current_different_from_parent_options(&self, deep_compare: bool) -> CfgResult<Vec<String>> {
        PresetCollection::dirty_options(Some(self.get_edited_preset()), self.get_selected_preset_parent(), deep_compare)
    }
    /// Preset.hpp:736-737: compare the content of get_saved_preset() with get_edited_preset()
    /// configs, return true if they differ.
    pub fn saved_is_dirty(&self) -> CfgResult<bool> {
        PresetCollection::is_dirty(Some(self.get_edited_preset()), Some(&self.m_saved_preset))
    }
    /// Preset.hpp:742: copy edited preset into saved preset.
    pub fn update_saved_preset_from_current_preset(&mut self) {
        self.m_saved_preset = self.m_edited_preset.clone();
    }

    /// Preset.cpp:3578-3595 `std::vector<std::string> PresetCollection::system_preset_names() const`:
    /// a sorted list of system preset names, including the former names of these presets. Used
    /// for validating the "inherits" flag when importing user's config bundles.
    pub fn system_preset_names(&self) -> Vec<String> {
        let mut num = 0usize;
        for preset in &self.m_presets {
            if preset.is_system {
                num += 1;
            }
        }
        let mut out: Vec<String> = Vec::with_capacity(num);
        for preset in &self.m_presets {
            if preset.is_system {
                out.push(preset.name.clone());
                out.extend(preset.renamed_from.iter().cloned());
            }
        }
        out.sort();
        out
    }

    /// Preset.cpp:3139-3149 `bool PresetCollection::update_dirty()`: update a dirty flag of the
    /// current preset. Return true if the dirty flag changed.
    pub fn update_dirty(&mut self) -> CfgResult<bool> {
        let was_dirty = self.get_selected_preset_mut().is_dirty;
        let is_dirty = self.current_is_dirty()?;
        self.get_selected_preset_mut().is_dirty = is_dirty;
        self.get_edited_preset_mut().is_dirty = is_dirty;

        Ok(was_dirty != is_dirty)
    }

    /// Preset.hpp:757
    pub fn is_base_preset(&self, preset: &Preset) -> bool {
        preset.is_system || (preset.is_user() && preset.inherits().is_empty())
    }

    /// Preset.cpp:3597-3607 `std::string PresetCollection::path_from_name(const std::string &new_name, bool detach) const`:
    /// a file path from a profile name, with the ".json" suffix added if it is missing.
    pub fn path_from_name(&self, new_name: &str, detach: bool) -> String {
        //BBS: change to json format
        let file_name = if iends_with(new_name, ".json") { new_name.to_string() } else { format!("{}.json", new_name) };
        if detach {
            boost_make_preferred(boost_path_append(&boost_path_append(&self.m_dir_path, "base"), &file_name))
        } else {
            boost_make_preferred(boost_path_append(&self.m_dir_path, &file_name))
        }
    }

    /// Preset.cpp:3609-3612 `std::string PresetCollection::path_for_preset(const Preset & preset) const`
    pub fn path_for_preset(&self, preset: &Preset) -> String {
        self.path_from_name(&preset.name, self.is_base_preset(preset))
    }

    /// Preset.cpp:3533-3551 `std::string PresetCollection::get_preset_alias(Preset &preset, bool force)`:
    /// the alias of a preset, setting it if it's empty.
    pub fn get_preset_alias(&mut self, preset: &mut Preset, force: bool) -> String {
        if !preset.alias.is_empty() {
            return preset.alias.clone();
        } else {
            self.set_custom_preset_alias(preset);
        }

        if !preset.alias.is_empty() || !force {
            return preset.alias.clone();
        }

        let mut alias_name = String::new();
        let preset_name = preset.name.clone();
        if let Some(end_pos) = preset_name.find('@') {
            alias_name = preset_name[..end_pos].to_string();
            // boost::trim_right
            alias_name = String::from_utf8_lossy(trim_c_right(alias_name.as_bytes())).into_owned();
        }
        alias_name
    }

    /// Preset.hpp:766
    pub fn num_default_presets(&self) -> usize {
        self.m_num_default_presets
    }

    /// Preset.cpp:3438-3447 `void PresetCollection::update_vendor_ptrs_after_copy(const VendorMap &new_vendors)`:
    /// after copying a collection, adjust Profile::vendor pointers.
    pub fn update_vendor_ptrs_after_copy(&mut self, new_vendors: &VendorMap) {
        for preset in self.m_presets.iter_mut() {
            sync_vendor_ptr(new_vendors, preset);
        }

        sync_vendor_ptr(new_vendors, &mut self.m_edited_preset);
        sync_vendor_ptr(new_vendors, &mut self.m_saved_preset);
    }

    /// Preset.cpp:3382-3404 `bool PresetCollection::select_preset_by_name_strict(const std::string &name)`:
    /// select a preset, if it exists. If it does not exist, select an invalid (-1) index. This
    /// is a temporary state, which shall be fixed immediately by the following step.
    pub fn select_preset_by_name_strict(&mut self, name: &str) -> bool {
        // 1) Try to find the preset by its name.
        let it = self.find_preset_internal(name);

        let mut idx = usize::MAX;
        if it != self.m_presets.len() && self.m_presets[it].name == name && self.m_presets[it].is_visible {
            // Preset found by its name.
            idx = it;
        }
        // 2) Select the new preset.
        if idx != usize::MAX {
            self.select_preset(idx);
            return true;
        }
        self.m_idx_selected = idx;
        false
    }

    /// Preset.cpp:3406-3427 `std::vector<std::string> PresetCollection::merge_presets(PresetCollection &&other, const VendorMap &new_vendors)`:
    /// merge one vendor's presets with the other vendor's presets, report duplicates.
    pub fn merge_presets(&mut self, other: PresetCollection, new_vendors: &VendorMap) -> Vec<String> {
        let mut duplicates: Vec<String> = Vec::new();
        for mut preset in other.m_presets {
            if preset.is_default || preset.is_external {
                continue;
            }
            // std::lower_bound(m_presets.begin() + m_num_default_presets, m_presets.end(), key)
            let n = self.m_num_default_presets;
            let it = n + self.m_presets[n..].partition_point(|p| p.name < preset.name);
            if it == self.m_presets.len() || self.m_presets[it].name != preset.name {
                if let Some(vendor) = &preset.vendor {
                    // Re-assign a pointer to the vendor structure in the new PresetBundle.
                    let v = new_vendors.get(&vendor.id).expect("new_vendors.find(preset.vendor->id)");
                    preset.vendor = Some(v.clone());
                }
                self.m_presets.insert(it, preset);
            } else {
                duplicates.push(preset.name);
            }
        }
        duplicates
    }

    /// Preset.cpp:3449-3457 `void PresetCollection::update_map_alias_to_profile_name()`: update
    /// m_map_alias_to_profile_name from loaded system profiles.
    pub fn update_map_alias_to_profile_name(&mut self) {
        self.m_map_alias_to_profile_name.clear();
        for preset in &self.m_presets {
            self.m_map_alias_to_profile_name.entry(preset.alias.clone()).or_default().push(preset.name.clone());
        }
        // now m_map_alias_to_profile_name is map, not need sort
    }

    /// Preset.cpp:3459-3468 `void PresetCollection::update_map_system_profile_renamed()`: update
    /// m_map_system_profile_renamed from loaded system profiles.
    pub fn update_map_system_profile_renamed(&mut self) {
        self.m_map_system_profile_renamed.clear();
        for preset in &self.m_presets {
            for renamed_from in &preset.renamed_from {
                // insert: an existing entry is kept (an error is logged).
                self.m_map_system_profile_renamed.entry(renamed_from.clone()).or_insert_with(|| preset.name.clone());
            }
        }
    }

    /// Preset.cpp:3470-3491 `void PresetCollection::set_custom_preset_alias(Preset &preset)`
    pub fn set_custom_preset_alias(&mut self, preset: &mut Preset) {
        if self.m_type == Type::TypeFilament
            && preset.is_user()
            && preset.config.has(BBL_JSON_KEY_INHERITS)
            && preset.config.option_typed::<ConfigOptionString>(BBL_JSON_KEY_INHERITS).expect("option<ConfigOptionString>(inherits)").value.is_empty()
        {
            let mut alias_name = String::new();
            let preset_name = preset.name.clone();
            if alias_name.is_empty() {
                if let Some(end_pos) = preset_name.find('@') {
                    alias_name = preset_name[..end_pos].to_string();
                    alias_name = String::from_utf8_lossy(trim_c_right(alias_name.as_bytes())).into_owned();
                }
            }
            if alias_name.is_empty() || self.is_alias_exist(&alias_name, Some(preset)) {
                preset.alias = String::new();
            } else {
                preset.alias = alias_name;
                self.m_map_alias_to_profile_name.entry(preset.alias.clone()).or_default().push(preset.name.clone());
                let alias = preset.alias.clone();
                self.set_printer_hold_alias(&alias, preset, false);
            }
        }
    }

    /// Preset.cpp:3493-3531 `void PresetCollection::set_printer_hold_alias(const std::string &alias, Preset &preset, bool remove)`
    pub fn set_printer_hold_alias(&mut self, alias: &str, preset: &Preset, remove: bool) {
        let compatible_printers = match strings_option(&preset.config, "compatible_printers") {
            None => return,
            Some(c) => c,
        };
        for printer_name in &compatible_printers.values {
            let mut _insert_success = false;
            let mut _remove_success = false;
            if !self.m_printer_hold_alias.contains_key(printer_name) {
                if !remove {
                    _insert_success = true;
                    self.m_printer_hold_alias.entry(printer_name.clone()).or_default().insert(alias.to_string());
                }
            } else {
                let printer_filament_alias = self.m_printer_hold_alias.get_mut(printer_name).unwrap();
                if !printer_filament_alias.contains(alias) {
                    if !remove {
                        _insert_success = true;
                        printer_filament_alias.insert(alias.to_string());
                    }
                } else if remove {
                    if preset.inherits().is_empty() {
                        _remove_success = true;
                        printer_filament_alias.remove(alias);
                    }
                    if let Some(presets) = self.m_map_alias_to_profile_name.get_mut(alias) {
                        presets.retain(|n| *n != preset.name);
                        if presets.is_empty() {
                            self.m_map_alias_to_profile_name.remove(alias);
                        }
                    }
                }
            }
        }
    }

    /// Preset.cpp:3081-3137 `size_t PresetCollection::update_compatible_internal(const PresetWithVendorProfile &active_printer, const PresetWithVendorProfile *active_print, PresetSelectCompatibleType unselect_if_incompatible)`
    fn update_compatible_internal(
        &mut self,
        active_printer: &PresetWithVendorProfile,
        active_print: Option<&PresetWithVendorProfile>,
        unselect_if_incompatible: PresetSelectCompatibleType,
    ) -> usize {
        let mut config = DynamicPrintConfig::new();
        config.set_key_value("printer_preset", Box::new(ConfigOptionString::with_value(active_printer.preset.name.clone())));
        if let Some(opt) = active_printer.preset.config.option("nozzle_diameter") {
            let n = opt.as_vector_base().expect("static_cast<const ConfigOptionFloatsNullable*>").size();
            config.set_key_value("num_extruders", Box::new(ConfigOptionInt::with_value(n as i32)));
        }
        // Keep the disambiguation policy after m_extra_compatible_printers is cleared by PresetBundle.
        self.m_active_printer_name = active_printer.preset.name.clone();
        self.m_prefer_active_printer_alias = !self.m_extra_compatible_printers.is_empty();
        let mut some_compatible = 0i32;

        for idx_preset in self.m_num_default_presets..self.m_presets.len() {
            let selected = idx_preset == self.m_idx_selected;
            let (was_compatible, is_compatible) = {
                let preset_edited = if selected { &self.m_edited_preset } else { &self.m_presets[idx_preset] };

                let this_preset_with_vendor_profile = self.get_preset_with_vendor_profile(preset_edited);
                let was_compatible = preset_edited.is_compatible;
                let mut is_compatible = is_compatible_with_printer(&this_preset_with_vendor_profile, active_printer, Some(&config));
                // BBS multi-diameter: broaden compatibility to sibling nozzle-diameter printer variants that the
                // user selected via "set nozzle diameter", so their filament presets stay selectable.
                if !is_compatible
                    && !self.m_extra_compatible_printers.is_empty()
                    && (this_preset_with_vendor_profile.vendor.is_none()
                        || vendor_ptr_eq(this_preset_with_vendor_profile.vendor, active_printer.vendor))
                {
                    is_compatible = is_compatible_with_printer_names(&this_preset_with_vendor_profile, &self.m_extra_compatible_printers);
                }
                if is_compatible {
                    some_compatible += 1;
                }
                if let Some(active_print) = active_print {
                    is_compatible &= is_compatible_with_print(&this_preset_with_vendor_profile, active_print, active_printer);
                }
                (was_compatible, is_compatible)
            };
            if selected {
                self.m_edited_preset.is_compatible = is_compatible;
            } else {
                self.m_presets[idx_preset].is_compatible = is_compatible;
            }
            if !is_compatible
                && selected
                && (unselect_if_incompatible == PresetSelectCompatibleType::Always
                    || (unselect_if_incompatible == PresetSelectCompatibleType::OnlyIfWasCompatible && was_compatible))
            {
                self.m_idx_selected = usize::MAX;
            }
            if selected {
                self.m_presets[idx_preset].is_compatible = is_compatible;
            }
        }
        // Update visibility of the default profiles here if the defaults are suppressed, the current profile is not compatible and we don't want to select another compatible profile.
        if self.m_idx_selected >= self.m_num_default_presets && self.m_default_suppressed {
            for i in 0..self.m_num_default_presets {
                self.m_presets[i].is_visible = some_compatible == 0;
            }
        }

        // if (m_idx_selected < m_num_default_presets) log_default_selection(...): a log record only.
        self.m_idx_selected
    }

    /// Preset.cpp:3246-3260 `bool PresetCollection::is_dirty(const Preset *edited, const Preset *reference)`
    pub fn is_dirty(edited: Option<&Preset>, reference: Option<&Preset>) -> CfgResult<bool> {
        if let (Some(edited), Some(reference)) = (edited, reference) {
            // Only compares options existing in both configs.
            if !reference.config.base.equals(&edited.config.base, Some(&SKIPPED_IN_DIRTY))? {
                return Ok(true);
            }
            // The "compatible_printers" option key is handled differently from the others:
            // It is not mandatory. If the key is missing, it means it is compatible with any printer.
            // If the key exists and it is empty, it means it is compatible with no printer.
            for opt_key in OPTIONAL_KEYS {
                if reference.config.has(opt_key) != edited.config.has(opt_key) {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    /// Preset.cpp:3262-3278 `std::vector<std::string> PresetCollection::dirty_options(const Preset *edited, const Preset *reference, const bool deep_compare)`
    pub fn dirty_options(edited: Option<&Preset>, reference: Option<&Preset>, deep_compare: bool) -> CfgResult<Vec<String>> {
        let mut changed: Vec<String> = Vec::new();
        if let (Some(edited), Some(reference)) = (edited, reference) {
            // Only compares options existing in both configs.
            changed = if deep_compare {
                deep_diff(&edited.config, &reference.config, true)?
            } else {
                reference.config.base.diff(&edited.config.base)?
            };
            // The "compatible_printers" option key is handled differently from the others:
            // It is not mandatory. If the key is missing, it means it is compatible with any printer.
            // If the key exists and it is empty, it means it is compatible with no printer.
            for opt_key in OPTIONAL_KEYS {
                if reference.config.has(opt_key) != edited.config.has(opt_key) {
                    changed.push(opt_key.to_string());
                }
            }
        }
        Ok(changed)
    }

    /// Preset.cpp:3280-3307 `std::vector<std::string> PresetCollection::dirty_options_without_option_list(const Preset *edited, const Preset *reference, const std::set<std::string>& option_ignore_list, const bool deep_compare)`
    pub fn dirty_options_without_option_list(
        edited: Option<&Preset>,
        reference: Option<&Preset>,
        option_ignore_list: &BTreeSet<String>,
        deep_compare: bool,
    ) -> CfgResult<Vec<String>> {
        let mut changed: Vec<String> = Vec::new();
        if let (Some(edited), Some(reference)) = (edited, reference) {
            // Only compares options existing in both configs.
            changed = if deep_compare {
                deep_diff(&edited.config, &reference.config, true)?
            } else {
                reference.config.base.diff(&edited.config.base)?
            };
            for opt_key in OPTIONAL_KEYS {
                if reference.config.has(opt_key) != edited.config.has(opt_key) {
                    changed.push(opt_key.to_string());
                }
            }
            changed.retain(|k| !option_ignore_list.contains(k));
        }
        Ok(changed)
    }

    /// Preset.cpp:3309-3336 `Preset& PresetCollection::select_preset(size_t idx)`: select a new
    /// preset. This resets all the edits done to the currently selected preset. If the preset
    /// with index idx does not exist, a first visible preset is selected.
    pub fn select_preset(&mut self, mut idx: usize) -> &mut Preset {
        for preset in self.m_presets.iter_mut() {
            preset.is_dirty = false;
        }
        if idx >= self.m_presets.len() {
            idx = self.first_visible_idx();
        }
        self.m_idx_selected = idx;
        self.m_edited_preset = self.m_presets[idx].clone();
        self.update_saved_preset_from_current_preset();
        let default_visible = !self.m_default_suppressed || self.m_idx_selected < self.m_num_default_presets;
        for i in 0..self.m_num_default_presets {
            self.m_presets[i].is_visible = default_visible;
        }

        //set this preset to true
        if !self.m_presets[idx].is_visible {
            self.m_presets[idx].is_visible = true;
        }
        // log_default_selection(*this, "selected, previous ..."): a log record only.
        &mut self.m_presets[idx]
    }

    /// Preset.cpp:3338-3380 `bool PresetCollection::select_preset_by_name(const std::string &name_w_suffix, bool force, bool select_invisible)`:
    /// select a profile by its name. Return true if the selection changed. Without force, the
    /// selection is only updated if the index changes. With force, the changes are reverted if
    /// the new index is the same as the old index.
    pub fn select_preset_by_name(&mut self, name_w_suffix: &str, force: bool, select_invisible: bool) -> bool {
        let name = Preset::remove_suffix_modified(name_w_suffix);
        // 1) Try to find the preset by its name.
        let it = self.find_preset_internal(&name);
        let mut idx: usize = 0;
        if it != self.m_presets.len() && self.m_presets[it].name == name && (self.m_presets[it].is_visible || select_invisible) {
            // Preset found by its name and it is visible.
            idx = it;
            if select_invisible && !self.m_presets[it].is_visible {
                self.m_presets[it].is_visible = true;
            }
        } else {
            // Find the first visible preset.
            let start = if self.m_default_suppressed { self.m_num_default_presets } else { 0 };
            for i in start..self.m_presets.len() {
                if self.m_presets[i].is_visible {
                    idx = i;
                    break;
                }
            }
            // If the first visible preset was not found, return the 0th element, which is the default preset.
        }

        // 2) Select the new preset.
        if self.m_idx_selected != idx || force {
            self.select_preset(idx);
            return true;
        }

        false
    }

    /// Preset.hpp:799-811 `std::deque<Preset>::iterator find_preset_internal(const std::string &name)`:
    /// a preset position in the sorted list of presets. The "-- default -- " preset is always the
    /// first, so it needs to be handled differently. If a preset does not exist, the position
    /// where to insert a preset with the same name.
    pub fn find_preset_internal(&self, name: &str) -> usize {
        let n = self.m_num_default_presets;
        let mut it = n + self.m_presets[n..].partition_point(|l| l.name.as_str() < name);
        if it == self.m_presets.len() || self.m_presets[it].name != name {
            // Preset has not been not found in the sorted list of non-default presets. Try the defaults.
            for i in 0..self.m_num_default_presets {
                if self.m_presets[i].name == name {
                    it = i;
                    break;
                }
            }
        }
        it
    }

    /// Preset.hpp:814-819 `std::deque<Preset>::iterator find_preset_renamed(const std::string &name)`
    pub fn find_preset_renamed(&self, name: &str) -> usize {
        match self.m_map_system_profile_renamed.get(name) {
            None => self.m_presets.len(),
            Some(renamed) => self.find_preset_internal(renamed),
        }
    }
}

/// `boost::trim_right`
fn trim_c_right(s: &[u8]) -> &[u8] {
    let mut e = s.len();
    while e > 0 && is_c_space(s[e - 1]) {
        e -= 1;
    }
    &s[..e]
}

/// Preset.cpp:3429-3436 `void inline static sync_vendor_ptr(const VendorMap &new_vendors, Preset &preset)`
fn sync_vendor_ptr(new_vendors: &VendorMap, preset: &mut Preset) {
    let id = match &preset.vendor {
        None => return,
        Some(v) => v.id.clone(),
    };
    let it = new_vendors.get(&id).expect("new_vendors.find(preset.vendor->id)");
    preset.vendor = Some(it.clone());
}

// ============================================================================
// PrinterPresetCollection (Preset.hpp:870-891, Preset.cpp:3614-3673)
// ============================================================================

/// Preset.hpp:870-891 `class PrinterPresetCollection : public PresetCollection`: printer
/// supports the FFF and SLA technologies, with different set of configuration values, therefore
/// this PresetCollection needs to handle two defaults. The base class is the field `base`.
pub struct PrinterPresetCollection {
    pub base: PresetCollection,
}

impl std::ops::Deref for PrinterPresetCollection {
    type Target = PresetCollection;
    fn deref(&self) -> &PresetCollection {
        &self.base
    }
}

impl std::ops::DerefMut for PrinterPresetCollection {
    fn deref_mut(&mut self) -> &mut PresetCollection {
        &mut self.base
    }
}

impl PrinterPresetCollection {
    /// Preset.hpp:875-876 (default_name "Default Printer")
    pub fn new(ty: Type, keys: &[String], defaults: &dyn ConfigBase, default_name: &str) -> CfgResult<PrinterPresetCollection> {
        let mut base = PresetCollection::new(ty, keys, defaults, default_name)?;
        base.m_printer_collection = true;
        Ok(PrinterPresetCollection { base })
    }

    /// Preset.cpp:3614-3618 `const Preset& PrinterPresetCollection::default_preset_for(const DynamicPrintConfig &config) const`
    pub fn default_preset_for(&self, config: &DynamicPrintConfig) -> &Preset {
        self.base.default_preset_for(config)
    }

    /// Preset.cpp:3620-3633 `const Preset* PrinterPresetCollection::find_system_preset_by_model_and_variant(const std::string &model_id, const std::string& variant) const`
    pub fn find_system_preset_by_model_and_variant(&self, model_id: &str, variant: &str) -> Option<&Preset> {
        if model_id.is_empty() {
            return None;
        }

        self.base.iter().find(|preset| {
            if !preset.is_system || preset.config.opt_string("printer_model") != model_id {
                return false;
            }
            if variant.is_empty() {
                return true;
            }
            preset.config.opt_string("printer_variant") == variant
        })
    }

    /// Preset.cpp:3635-3648 `const Preset *PrinterPresetCollection::find_custom_preset_by_model_and_variant(const std::string &model_id, const std::string &variant) const`
    pub fn find_custom_preset_by_model_and_variant(&self, model_id: &str, variant: &str) -> Option<&Preset> {
        if model_id.is_empty() {
            return None;
        }

        self.base.iter().find(|preset| {
            if preset.config.opt_string("printer_model") != model_id {
                return false;
            }
            if variant.is_empty() {
                return true;
            }
            preset.config.opt_string("printer_variant") == variant
        })
    }

    /// Preset.cpp:3650-3664 `std::vector<const Preset*> PrinterPresetCollection::find_all_presets_by_model(const std::string &model_id, bool system_only) const`
    pub fn find_all_presets_by_model(&self, model_id: &str, system_only: bool) -> Vec<&Preset> {
        let mut result: Vec<&Preset> = Vec::new();
        if model_id.is_empty() {
            return result;
        }

        for p in self.base.iter() {
            if system_only && !p.is_system {
                continue;
            }
            if p.config.opt_string("printer_model") == model_id {
                result.push(p);
            }
        }

        result
    }

    /// Preset.cpp:3666-3673 `bool PrinterPresetCollection::only_default_printers() const`
    pub fn only_default_printers(&self) -> bool {
        for printer in self.base.get_presets() {
            if !printer.name.starts_with("Default") && printer.is_visible {
                return false;
            }
        }
        true
    }
}

// ============================================================================
// PresetUtils (Preset.hpp:893-899, Preset.cpp:4191-4241)
// ============================================================================

pub mod preset_utils {
    use super::*;

    /// Preset.cpp:4192-4204 `const VendorProfile::PrinterModel* system_printer_model(const Preset &preset)`:
    /// the PrinterModel of a system profile, from which this preset is derived, or null if it is
    /// not derived from a system profile.
    pub fn system_printer_model(preset: &Preset) -> Option<&PrinterModel> {
        let mut out: Option<&PrinterModel> = None;
        if let Some(vendor) = &preset.vendor {
            if let Some(printer_model) = preset.config.option_typed::<ConfigOptionString>("printer_model") {
                if !printer_model.value.is_empty() {
                    if let Some(pm) = vendor.models.iter().find(|pm| pm.id == printer_model.value) {
                        out = Some(pm);
                    }
                }
            }
        }
        out
    }

    fn system_printer_file(preset: &Preset, file: impl Fn(&PrinterModel) -> &String) -> String {
        let mut out = String::new();
        if let Some(pm) = system_printer_model(preset) {
            if !file(pm).is_empty() {
                let vendor_id = &preset.vendor.as_ref().unwrap().id;
                out = format!("{}/vendor/{}/{}", crate::log_sink::data_dir(), vendor_id, file(pm));
                if !fs_exists(&out) {
                    out = format!("{}/profiles/{}/{}", crate::utils::resources_dir().to_string_lossy(), vendor_id, file(pm));
                }
            }
        }
        out
    }

    /// Preset.cpp:4206-4216 `std::string system_printer_bed_model(const Preset& preset)`
    pub fn system_printer_bed_model(preset: &Preset) -> String {
        system_printer_file(preset, |pm| &pm.bed_model)
    }

    /// Preset.cpp:4218-4228 `std::string system_printer_bed_texture(const Preset& preset)`
    pub fn system_printer_bed_texture(preset: &Preset) -> String {
        system_printer_file(preset, |pm| &pm.bed_texture)
    }

    /// Preset.cpp:4230-4240 `std::string system_printer_hotend_model(const Preset& preset)`
    pub fn system_printer_hotend_model(preset: &Preset) -> String {
        system_printer_file(preset, |pm| &pm.hotend_model)
    }
}

/// FIDELITY-NOTE: the Preset methods that take a `PresetBundle*` read its printers and vendors
/// only; this holds those two, so a caller without a whole `preset_bundle::PresetBundle` (the tests
/// of this unit) can pass them. A bundle gives its view with `PresetBundle::view()`.
pub struct PresetBundleView<'a> {
    pub printers: &'a PrinterPresetCollection,
    pub vendors: &'a VendorMap,
}

// ============================================================================
// PhysicalPrinter (Preset.hpp:904-970, Preset.cpp:3674-3832)
// ============================================================================

/// Preset.hpp:904-970 `class PhysicalPrinter`
#[derive(Clone)]
pub struct PhysicalPrinter {
    // Name of the Physical Printer, usually derived form the file name.
    pub name: String,
    // File name of the Physical Printer.
    pub file: String,
    // Configuration data, loaded from a file, or set from the defaults.
    pub config: DynamicPrintConfig,
    // set of presets used with this physical printer
    pub preset_names: BTreeSet<String>,
    // Has this profile been loaded?
    pub loaded: bool,
}

impl std::fmt::Debug for PhysicalPrinter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PhysicalPrinter")
            .field("name", &self.name)
            .field("file", &self.file)
            .field("preset_names", &self.preset_names)
            .field("loaded", &self.loaded)
            .finish()
    }
}

impl Default for PhysicalPrinter {
    /// A printer with no name and an empty configuration (not a C++ constructor; kept for the
    /// callers of the earlier port).
    fn default() -> Self {
        PhysicalPrinter { name: String::new(), file: String::new(), config: DynamicPrintConfig::new(), preset_names: BTreeSet::new(), loaded: false }
    }
}

/// Preset.cpp:3724-3732 `static void update_preset_name_option(const std::set<std::string>& preset_names, DynamicPrintConfig& config)`:
/// temporary workaround for compatibility with older Slicer.
fn update_preset_name_option(preset_names: &BTreeSet<String>, config: &mut DynamicPrintConfig) {
    let mut name = String::new();
    for el in preset_names {
        name += el;
        name += ";";
    }
    name.pop();
    config.set_key_value("preset_name", Box::new(ConfigOptionString::with_value(name)));
}

impl PhysicalPrinter {
    /// Preset.cpp:3797-3801 `PhysicalPrinter::PhysicalPrinter(const std::string& name, const DynamicPrintConfig& default_config)`
    pub fn new(name: &str, default_config: &DynamicPrintConfig) -> CfgResult<PhysicalPrinter> {
        let mut p = PhysicalPrinter { name: name.to_string(), file: String::new(), config: default_config.clone(), preset_names: BTreeSet::new(), loaded: false };
        let c = p.config.clone();
        p.update_from_config(&c)?;
        Ok(p)
    }

    /// Preset.cpp:3803-3807 `PhysicalPrinter::PhysicalPrinter(const std::string& name, const DynamicPrintConfig &default_config, const Preset& preset)`
    pub fn with_preset(name: &str, default_config: &DynamicPrintConfig, preset: &Preset) -> CfgResult<PhysicalPrinter> {
        let mut p = PhysicalPrinter { name: name.to_string(), file: String::new(), config: default_config.clone(), preset_names: BTreeSet::new(), loaded: false };
        p.update_from_preset(preset)?;
        Ok(p)
    }

    /// Preset.cpp:3809-3812 `void PhysicalPrinter::set_name(const std::string& name)`
    pub fn set_name(&mut self, name: &str) {
        self.name = name.to_string();
    }

    /// Preset.cpp:3678-3681 `std::string PhysicalPrinter::separator()`
    pub fn separator() -> String {
        " * ".to_string()
    }

    /// Preset.cpp:3699-3702 `const std::vector<std::string>& PhysicalPrinter::printer_options()`
    pub fn printer_options() -> &'static Vec<String> {
        static V: LazyLock<Vec<String>> = LazyLock::new(|| strings(S_PHYSICAL_PRINTER_OPTS));
        &V
    }

    /// Preset.cpp:3704-3712 `std::vector<std::string> PhysicalPrinter::presets_with_print_host_information(const PrinterPresetCollection& printer_presets)`
    pub fn presets_with_print_host_information(printer_presets: &PrinterPresetCollection) -> Vec<String> {
        let mut presets: Vec<String> = Vec::new();
        for preset in printer_presets.iter() {
            if PhysicalPrinter::has_print_host_information(&preset.config) {
                presets.push(preset.name.clone());
            }
        }

        presets
    }

    /// Preset.cpp:3714-3717 `bool PhysicalPrinter::has_print_host_information(const DynamicPrintConfig& config)`
    pub fn has_print_host_information(_config: &DynamicPrintConfig) -> bool {
        false
    }

    /// Preset.cpp:3719-3722 `const std::set<std::string>& PhysicalPrinter::get_preset_names() const`
    pub fn get_preset_names(&self) -> &BTreeSet<String> {
        &self.preset_names
    }

    /// Preset.cpp:3734-3745 `void PhysicalPrinter::update_preset_names_in_config()`
    pub fn update_preset_names_in_config(&mut self) {
        if !self.preset_names.is_empty() {
            let values = &mut self
                .config
                .option_typed_mut::<ConfigOptionStrings>("preset_names", false)
                .expect("PhysicalPrinter: no definition")
                .expect("config.option<ConfigOptionStrings>(\"preset_names\") is null")
                .values;
            values.clear();
            for preset in &self.preset_names {
                values.push(preset.clone());
            }

            // temporary workaround for compatibility with older Slicer
            update_preset_name_option(&self.preset_names, &mut self.config);
        }
    }

    /// Preset.hpp:935 `void save(DynamicPrintConfig* parent_config)` (BBS: change to json format)
    pub fn save(&self, _parent_config: Option<&DynamicPrintConfig>) -> CfgResult<()> {
        self.config.save_to_json(&self.file, "Physical_Printer", "User", SLIC3R_VERSION)
    }

    /// Preset.cpp:3747-3756 `void PhysicalPrinter::save(const std::string& file_name_from, const std::string& file_name_to)`:
    /// rename the file, then save the configuration.
    pub fn save_2(&mut self, file_name_from: &str, file_name_to: &str) -> CfgResult<()> {
        // rename the file
        let _ = std::fs::rename(file_name_from, file_name_to);
        self.file = file_name_to.to_string();
        // save configuration
        //BBS: change to save
        self.config.save_to_json(&self.file, "Physical_Printer", "User", SLIC3R_VERSION)
    }

    /// Preset.cpp:3758-3764 `void PhysicalPrinter::update_from_preset(const Preset& preset)`
    pub fn update_from_preset(&mut self, preset: &Preset) -> CfgResult<()> {
        self.config.apply_only(&preset.config, PhysicalPrinter::printer_options(), true)?;
        // add preset names to the options list
        self.preset_names.insert(preset.name.clone());
        self.update_preset_names_in_config();
        Ok(())
    }

    /// Preset.cpp:3766-3780 `void PhysicalPrinter::update_from_config(const DynamicPrintConfig& new_config)`
    pub fn update_from_config(&mut self, new_config: &DynamicPrintConfig) -> CfgResult<()> {
        self.config.apply_only(new_config, PhysicalPrinter::printer_options(), false)?;

        let values = self
            .config
            .option_typed::<ConfigOptionStrings>("preset_names")
            .expect("config.option<ConfigOptionStrings>(\"preset_names\") is null")
            .values
            .clone();

        if values.is_empty() {
            self.preset_names.clear();
        } else {
            for val in &values {
                self.preset_names.insert(val.clone());
            }
            // temporary workaround for compatibility with older Slicer
            update_preset_name_option(&self.preset_names, &mut self.config);
        }
        Ok(())
    }

    /// Preset.cpp:3787-3790 `bool PhysicalPrinter::add_preset(const std::string& preset_name)`:
    /// false, if a preset with this name already exists in the set.
    pub fn add_preset(&mut self, preset_name: &str) -> bool {
        self.preset_names.insert(preset_name.to_string())
    }

    /// Preset.cpp:3792-3795 `bool PhysicalPrinter::delete_preset(const std::string& preset_name)`
    pub fn delete_preset(&mut self, preset_name: &str) -> bool {
        self.preset_names.remove(preset_name)
    }

    /// Preset.cpp:3782-3785 `void PhysicalPrinter::reset_presets()`
    pub fn reset_presets(&mut self) {
        self.preset_names.clear();
    }

    /// Preset.hpp:948-953 `static PrinterTechnology printer_technology(const DynamicPrintConfig& cfg)`:
    /// ptFFF if the printer technology is not set.
    pub fn printer_technology_cfg(cfg: &DynamicPrintConfig) -> PrinterTechnology {
        Preset::printer_technology_cfg(cfg)
    }
    /// Preset.hpp:954
    pub fn printer_technology(&self) -> PrinterTechnology {
        PhysicalPrinter::printer_technology_cfg(&self.config)
    }

    /// Preset.hpp:957 `bool operator<(const PhysicalPrinter& other) const`
    pub fn operator_lt(&self, other: &PhysicalPrinter) -> bool {
        self.name < other.name
    }

    /// Preset.cpp:3814-3817 `std::string PhysicalPrinter::get_full_name(std::string preset_name) const`:
    /// the full printer name including the name of the preset.
    pub fn get_full_name(&self, preset_name: &str) -> String {
        self.name.clone() + &PhysicalPrinter::separator() + preset_name
    }

    /// Preset.cpp:3819-3825 `std::string PhysicalPrinter::get_short_name(std::string full_name)`:
    /// the printer name from the full name including the preset name.
    pub fn get_short_name(full_name: &str) -> String {
        let mut full_name = full_name.as_bytes().to_vec();
        // int pos = full_name.find(separator()): npos is -1 as an int.
        let sep = PhysicalPrinter::separator();
        let pos: i64 = match find_bytes(&full_name, sep.as_bytes()) {
            Some(p) => p as i64,
            None => -1,
        };
        // if (pos > 0) boost::erase_tail(full_name, full_name.length() - pos);
        if pos > 0 {
            full_name.truncate(pos as usize);
        }
        String::from_utf8_lossy(&full_name).into_owned()
    }

    /// Preset.cpp:3827-3832 `std::string PhysicalPrinter::get_preset_name(std::string name)`:
    /// the preset name from the full name including the printer name.
    pub fn get_preset_name(name: &str) -> String {
        let bytes = name.as_bytes();
        // int pos = name.find(separator()): npos is -1 as an int.
        let sep = PhysicalPrinter::separator();
        let pos: i64 = match find_bytes(bytes, sep.as_bytes()) {
            Some(p) => p as i64,
            None => -1,
        };
        // boost::erase_head(name, pos + 3): the first (pos + 3) characters removed, all of them
        // when there are fewer.
        let head = (pos + 3).max(0) as usize;
        let rest = if head >= bytes.len() { Vec::new() } else { bytes[head..].to_vec() };
        Preset::remove_suffix_modified(&String::from_utf8_lossy(&rest))
    }
}

fn find_bytes(hay: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() {
        return Some(0);
    }
    hay.windows(needle.len()).position(|w| w == needle)
}

// ============================================================================
// PhysicalPrinterCollection (Preset.hpp:973-1100, Preset.cpp:3835-4188)
// ============================================================================

/// Preset.hpp:978-1100 `class PhysicalPrinterCollection`: collections of physical printers.
pub struct PhysicalPrinterCollection {
    // List of printers
    pub m_printers: Vec<PhysicalPrinter>,
    // Default config for a physical printer containing all key/value pairs of PhysicalPrinter::printer_options().
    pub m_default_config: DynamicPrintConfig,
    // Selected printer.
    pub m_idx_selected: usize,
    // The name of the preset which is currently select for this printer
    pub m_selected_preset: String,
    // Path to the directory to store the config files into.
    pub m_dir_path: String,
}

impl PhysicalPrinterCollection {
    /// Preset.cpp:3839-3848 `PhysicalPrinterCollection::PhysicalPrinterCollection( const std::vector<std::string>& keys)`
    pub fn new(keys: &[String]) -> PhysicalPrinterCollection {
        let mut c = PhysicalPrinterCollection {
            m_printers: Vec::new(),
            m_default_config: DynamicPrintConfig::new(),
            m_idx_selected: usize::MAX,
            m_selected_preset: String::new(),
            m_dir_path: String::new(),
        };
        // Default config for a physical printer containing all key/value pairs of PhysicalPrinter::printer_options().
        for key in keys {
            let opt = print_config_def().get(key).expect("print_config_def.get(key)");
            let dv = opt.default_value.as_ref().expect("opt->default_value");
            c.m_default_config.set_key_value(key, dv.clone());
        }
        c
    }

    /// Preset.hpp:985-990 `begin()` / `end()`
    pub fn begin(&self) -> usize {
        0
    }
    /// Preset.hpp:988
    pub fn end(&self) -> usize {
        self.m_printers.len()
    }
    /// Preset.hpp:992
    pub fn empty(&self) -> bool {
        self.m_printers.is_empty()
    }
    /// Preset.hpp:994 `void reset(bool delete_files) {};`
    pub fn reset(&mut self, _delete_files: bool) {}
    /// Preset.hpp:996 `const std::deque<PhysicalPrinter>& operator()() const`
    pub fn operator_call(&self) -> &Vec<PhysicalPrinter> {
        &self.m_printers
    }

    fn sort_printers(&mut self) {
        let mut idx: Vec<usize> = (0..self.m_printers.len()).collect();
        let printers = &self.m_printers;
        crate::clipper::libcxx_sort(&mut idx, |a, b| printers[*a].operator_lt(&printers[*b]));
        let mut taken: Vec<Option<PhysicalPrinter>> = self.m_printers.drain(..).map(Some).collect();
        for i in idx {
            self.m_printers.push(taken[i].take().unwrap());
        }
    }

    /// Preset.cpp:3850-3910 `void PhysicalPrinterCollection::load_printers(const std::string& dir_path, const std::string& subdir, PresetsConfigSubstitutions& substitutions, ForwardCompatibilitySubstitutionRule substitution_rule)`:
    /// load all printers found in dir_path. Throws an exception on error.
    pub fn load_printers(
        &mut self,
        dir_path: &str,
        subdir: &str,
        substitutions: &mut PresetsConfigSubstitutions,
        substitution_rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<()> {
        let dir = boost_make_preferred(boost_absolute(&boost_path_append(dir_path, subdir)));
        self.m_dir_path = dir.clone();
        let mut errors_cummulative = String::new();
        // Store the loaded printers into a new vector, otherwise the binary search for already existing presets would be broken.
        let mut printers_loaded: Vec<PhysicalPrinter> = Vec::new();
        //BBS: change to json format
        let entries = std::fs::read_dir(&dir).map_err(|e| runtime_error(format!("boost::filesystem::directory_iterator::construct: {}: \"{}\"", e, dir)))?;
        for dir_entry in entries {
            let dir_entry = match dir_entry {
                Ok(e) => e,
                Err(e) => return Err(runtime_error(format!("boost::filesystem::directory_iterator::operator++: {}", e))),
            };
            let file_name = dir_entry.file_name().to_string_lossy().into_owned();
            if !crate::utils::is_json_file(&file_name) {
                continue;
            }
            // Remove the .json suffix.
            let name = file_name[..file_name.len() - 5].to_string();
            if self.find_printer(&name, false).is_some() {
                // This happens when there's is a preset (most likely legacy one) with the same name as a system preset
                // that's already been loaded from a bundle.
                continue;
            }
            let result: CfgResult<PhysicalPrinter> = (|| {
                let mut printer = PhysicalPrinter::new(&name, self.default_config())?;
                printer.file = boost_path_append(&dir, &file_name);
                // Load the preset file, apply preset values on top of defaults.
                let inner: CfgResult<()> = (|| {
                    let mut config = DynamicPrintConfig::new();
                    let mut key_values: BTreeMap<String, String> = BTreeMap::new();
                    let mut reason = String::new();
                    let config_substitutions = config.load_from_json(&printer.file, substitution_rule, &mut key_values, &mut reason);
                    if !config_substitutions.is_empty() {
                        substitutions.push(PresetConfigSubstitutions {
                            preset_name: name.clone(),
                            preset_type: Type::TypePhysicalPrinter,
                            preset_source: PresetConfigSubstitutionsSource::UserFile,
                            preset_file: printer.file.clone(),
                            substitutions: config_substitutions,
                        });
                    }
                    printer.update_from_config(&config)?;
                    printer.loaded = true;
                    Ok(())
                })();
                if let Err(err) = inner {
                    return Err(runtime_error(format!("Failed loading the preset file: {}\n\tReason: {}", printer.file, err.what())));
                }
                Ok(printer)
            })();
            match result {
                Ok(printer) => printers_loaded.push(printer),
                Err(err) => {
                    errors_cummulative += err.what();
                    errors_cummulative += "\n";
                }
            }
        }
        self.m_printers.extend(printers_loaded);
        self.sort_printers();
        if !errors_cummulative.is_empty() {
            return Err(runtime_error(errors_cummulative));
        }
        Ok(())
    }

    /// Preset.cpp:3930-3976 `void PhysicalPrinterCollection::load_printers_from_presets(PrinterPresetCollection& printer_presets)`:
    /// the body is under `#if 0` (BBS), it does nothing.
    pub fn load_printers_from_presets(&mut self, _printer_presets: &mut PrinterPresetCollection) {}

    /// Preset.cpp:3912-3928 `void PhysicalPrinterCollection::load_printer(const std::string& path, const std::string& name, DynamicPrintConfig&& config, bool select, bool save)`:
    /// load printer from the loaded configuration.
    pub fn load_printer(&mut self, path: &str, name: &str, config: DynamicPrintConfig, select: bool, save: bool) -> CfgResult<()> {
        let it = self.find_printer_internal(name, true);
        if it == self.m_printers.len() || self.m_printers[it].name != name {
            // The preset was not found. Create a new preset.
            let p = PhysicalPrinter::new(name, &config)?;
            self.m_printers.insert(it, p);
        }

        self.m_printers[it].file = path.to_string();
        self.m_printers[it].config = config;
        self.m_printers[it].loaded = true;
        if select {
            let n = self.m_printers[it].name.clone();
            self.select_printer(&n);
        }

        if save {
            self.m_printers[it].save(None)?;
        }
        Ok(())
    }

    /// Preset.cpp:4022-4060 `void PhysicalPrinterCollection::save_printer(PhysicalPrinter& edited_printer, const std::string& renamed_from)`:
    /// save the printer under a new name. If the name is different from the old one, a new
    /// printer is stored into the list of printers. The new printer is activated.
    pub fn save_printer(&mut self, edited_printer: &mut PhysicalPrinter, renamed_from: &str) -> CfgResult<()> {
        // controll and update preset_names in edited_printer config
        edited_printer.update_preset_names_in_config();

        let name = if renamed_from.is_empty() { edited_printer.name.clone() } else { renamed_from.to_string() };
        // 1) Find the printer with a new_name or create a new one,
        // initialize it with the edited config.
        let mut it = self.find_printer_internal(&name, true);
        if it != self.m_printers.len() && self.m_printers[it].name == name {
            // Printer with the same name found.
            // Overwriting an existing preset (the edited config is moved out).
            self.m_printers[it].config = std::mem::take(&mut edited_printer.config);
            self.m_printers[it].name = edited_printer.name.clone();
            self.m_printers[it].preset_names = edited_printer.preset_names.clone();
            // sort printers and get new it
            self.sort_printers();
            it = self.find_printer_internal(&edited_printer.name, true);
        } else {
            // Creating a new printer.
            self.m_printers.insert(it, edited_printer.clone());
        }

        self.save_printer_at(it)
    }

    /// The second half of save_printer (Preset.cpp:4047-4059), on the printer at `it`.
    fn save_printer_at(&mut self, it: usize) -> CfgResult<()> {
        // 2) Save printer
        if self.m_printers[it].file.is_empty() {
            let f = self.path_from_name(&self.m_printers[it].name);
            self.m_printers[it].file = f;
        }

        let path = self.path_from_name(&self.m_printers[it].name);
        if self.m_printers[it].file == path {
            self.m_printers[it].save(None)?;
        } else {
            // if printer was renamed, we should rename a file and than save the config
            let from = self.m_printers[it].file.clone();
            self.m_printers[it].save_2(&from, &path)?;
        }

        // update idx_selected
        self.m_idx_selected = it;
        Ok(())
    }

    /// `save_printer(printer)` called on an element of m_printers (delete_preset_from_printers,
    /// Preset.cpp:4098): the edited printer is the stored one, so `it->config =
    /// std::move(edited_printer.config)` is a self move, which clears the configuration
    /// (Config.hpp:2695-2702: `this->clear()` first).
    fn save_printer_self(&mut self, idx: usize) -> CfgResult<()> {
        self.m_printers[idx].update_preset_names_in_config();
        let name = self.m_printers[idx].name.clone();
        let mut it = self.find_printer_internal(&name, true);
        if it != self.m_printers.len() && self.m_printers[it].name == name {
            self.m_printers[it].config.clear();
            self.sort_printers();
            it = self.find_printer_internal(&name, true);
        } else {
            let p = self.m_printers[idx].clone();
            self.m_printers.insert(it, p);
        }
        self.save_printer_at(it)
    }

    /// Preset.cpp:4062-4073 `bool PhysicalPrinterCollection::delete_printer(const std::string& name)`.
    /// The position the binary search gives is deleted without comparing the name, as in the C++.
    pub fn delete_printer(&mut self, name: &str) -> bool {
        let it = self.find_printer_internal(name, true);
        if it == self.m_printers.len() {
            return false;
        }

        // Erase the preset file.
        fs_remove(&self.m_printers[it].file);
        self.m_printers.remove(it);
        true
    }

    /// Preset.cpp:4075-4089 `bool PhysicalPrinterCollection::delete_selected_printer()`
    pub fn delete_selected_printer(&mut self) -> bool {
        if !self.has_selection() {
            return false;
        }

        // Erase the preset file.
        fs_remove(&self.get_selected_printer().file);
        // Remove the preset from the list.
        self.m_printers.remove(self.m_idx_selected);
        // unselect all printers
        self.unselect_printer();

        true
    }

    /// Preset.cpp:4091-4107 `bool PhysicalPrinterCollection::delete_preset_from_printers( const std::string& preset_name)`:
    /// delete preset_name preset from all printers; a printer whose last preset it is goes too.
    pub fn delete_preset_from_printers(&mut self, preset_name: &str) -> CfgResult<bool> {
        let mut printers_for_delete: Vec<String> = Vec::new();
        let mut i = 0;
        while i < self.m_printers.len() {
            let only = self.m_printers[i].preset_names.len() == 1 && self.m_printers[i].preset_names.iter().next().unwrap() == preset_name;
            if only {
                printers_for_delete.push(self.m_printers[i].name.clone());
            } else if self.m_printers[i].delete_preset(preset_name) {
                self.save_printer_self(i)?;
            }
            i += 1;
        }

        if !printers_for_delete.is_empty() {
            for printer_name in &printers_for_delete {
                self.delete_printer(printer_name);
            }
        }

        self.unselect_printer();
        Ok(true)
    }

    /// Preset.cpp:4109-4122 `std::vector<std::string> PhysicalPrinterCollection::get_printers_with_preset(const std::string& preset_name)`:
    /// the printers which have more than one preset, "preset_name" one of them.
    pub fn get_printers_with_preset(&self, preset_name: &str) -> Vec<String> {
        let mut printers: Vec<String> = Vec::new();

        for printer in &self.m_printers {
            if printer.preset_names.len() == 1 {
                continue;
            }
            if printer.preset_names.contains(preset_name) {
                printers.push(printer.name.clone());
            }
        }

        printers
    }

    /// Preset.cpp:4124-4134 `std::vector<std::string> PhysicalPrinterCollection::get_printers_with_only_preset(const std::string& preset_name)`
    pub fn get_printers_with_only_preset(&self, preset_name: &str) -> Vec<String> {
        let mut printers: Vec<String> = Vec::new();

        for printer in &self.m_printers {
            if printer.preset_names.len() == 1 && printer.preset_names.iter().next().unwrap() == preset_name {
                printers.push(printer.name.clone());
            }
        }

        printers
    }

    /// Preset.hpp:1026-1027 `PhysicalPrinter& get_selected_printer()`
    pub fn get_selected_printer(&self) -> &PhysicalPrinter {
        &self.m_printers[self.m_idx_selected]
    }
    /// Preset.hpp:1026
    pub fn get_selected_printer_mut(&mut self) -> &mut PhysicalPrinter {
        &mut self.m_printers[self.m_idx_selected]
    }
    /// Preset.hpp:1029
    pub fn get_selected_idx(&self) -> usize {
        self.m_idx_selected
    }
    /// Preset.hpp:1031: the name of the selected printer, or an empty string if none is selected.
    pub fn get_selected_printer_name(&self) -> String {
        if self.m_idx_selected == usize::MAX {
            String::new()
        } else {
            self.get_selected_printer().name.clone()
        }
    }
    /// Preset.hpp:1033: the config of the selected printer, or nullptr if no printer is selected.
    pub fn get_selected_printer_config(&mut self) -> Option<&mut DynamicPrintConfig> {
        if self.m_idx_selected == usize::MAX {
            None
        } else {
            Some(&mut self.get_selected_printer_mut().config)
        }
    }
    /// Preset.hpp:1035
    pub fn get_selected_printer_technology(&self) -> PrinterTechnology {
        if self.m_idx_selected == usize::MAX {
            PrinterTechnology::PtAny
        } else {
            self.get_selected_printer().printer_technology()
        }
    }

    /// Preset.cpp:4136-4139 `std::string PhysicalPrinterCollection::get_selected_full_printer_name() const`
    pub fn get_selected_full_printer_name(&self) -> String {
        if self.m_idx_selected == usize::MAX {
            String::new()
        } else {
            self.get_selected_printer().get_full_name(&self.m_selected_preset)
        }
    }
    /// Preset.hpp:1042
    pub fn get_selected_printer_preset_name(&self) -> String {
        if self.m_idx_selected == usize::MAX {
            String::new()
        } else {
            self.m_selected_preset.clone()
        }
    }

    /// Preset.cpp:4141-4159 `void PhysicalPrinterCollection::select_printer(const std::string& full_name)`:
    /// select printer by the full printer name, which contains name of printer, separator and
    /// name of selected preset. If full_name doesn't contain name of selected preset, then the
    /// first preset in the list for this printer is selected.
    pub fn select_printer(&mut self, full_name: &str) {
        let printer_name = PhysicalPrinter::get_short_name(full_name);
        let it = self.find_printer_internal(&printer_name, true);
        if it == self.m_printers.len() {
            self.unselect_printer();
            return;
        }

        // update idx_selected
        self.m_idx_selected = it;

        // update name of the currently selected preset
        if printer_name == full_name {
            // use first preset in the list
            self.m_selected_preset = self.m_printers[it].preset_names.iter().next().expect("*it->preset_names.begin()").clone();
        } else {
            self.m_selected_preset = PhysicalPrinter::get_preset_name(full_name);
        }
    }

    /// Preset.cpp:4161-4166 `void PhysicalPrinterCollection::select_printer(const std::string& printer_name, const std::string& preset_name)`
    pub fn select_printer_with_preset(&mut self, printer_name: &str, preset_name: &str) {
        if preset_name.is_empty() {
            return self.select_printer(printer_name);
        }
        self.select_printer(&format!("{}{}{}", printer_name, PhysicalPrinter::separator(), preset_name))
    }

    /// Preset.cpp:4168-4171 `void PhysicalPrinterCollection::select_printer(const PhysicalPrinter& printer)`
    pub fn select_printer_2(&mut self, printer: &PhysicalPrinter) {
        let n = printer.name.clone();
        self.select_printer(&n)
    }

    /// Preset.cpp:4173-4176 `bool PhysicalPrinterCollection::has_selection() const`
    pub fn has_selection(&self) -> bool {
        self.m_idx_selected != usize::MAX
    }

    /// Preset.cpp:4178-4182 `void PhysicalPrinterCollection::unselect_printer()`
    pub fn unselect_printer(&mut self) {
        self.m_idx_selected = usize::MAX;
        self.m_selected_preset.clear();
    }

    /// Preset.cpp:4184-4188 `bool PhysicalPrinterCollection::is_selected(PhysicalPrinterCollection::ConstIterator it, const std::string& preset_name) const`
    pub fn is_selected(&self, it: usize, preset_name: &str) -> bool {
        self.m_idx_selected == it && self.m_selected_preset == preset_name
    }

    /// Preset.hpp:1054-1055 `PhysicalPrinter& printer(size_t idx)`
    pub fn printer(&self, idx: usize) -> &PhysicalPrinter {
        &self.m_printers[idx]
    }

    /// Preset.cpp:3978-3992 `PhysicalPrinter* PhysicalPrinterCollection::find_printer( const std::string& name, bool case_sensitive_search)`:
    /// a printer by its name, possibly case insensitive.
    pub fn find_printer(&self, name: &str, case_sensitive_search: bool) -> Option<&PhysicalPrinter> {
        let it = self.find_printer_internal(name, case_sensitive_search);

        let is_equal_name = |in_name: &str| {
            if case_sensitive_search {
                return in_name == name;
            }
            boost_to_lower(in_name) == boost_to_lower(name)
        };

        if it == self.m_printers.len() || !is_equal_name(&self.m_printers[it].name) {
            return None;
        }
        Some(self.printer(it))
    }

    /// Preset.cpp:4013-4020 `std::string PhysicalPrinterCollection::path_from_name(const std::string& new_name) const`:
    /// a file path from a profile name, with the ".json" suffix added if it is missing.
    pub fn path_from_name(&self, new_name: &str) -> String {
        //BBS: change to json format
        let file_name = if iends_with(new_name, ".json") { new_name.to_string() } else { format!("{}.json", new_name) };
        boost_make_preferred(boost_path_append(&self.m_dir_path, &file_name))
    }

    /// Preset.hpp:1069
    pub fn default_config(&self) -> &DynamicPrintConfig {
        &self.m_default_config
    }

    /// Preset.cpp:3994-4011 `std::deque<PhysicalPrinter>::iterator PhysicalPrinterCollection::find_printer_internal(const std::string& name, bool case_sensitive_search)`:
    /// a physical printer position in the sorted list of printers. The name of a printer should
    /// be unique and case insensitive.
    pub fn find_printer_internal(&self, name: &str, case_sensitive_search: bool) -> usize {
        if case_sensitive_search {
            return self.m_printers.partition_point(|l| l.name.as_str() < name);
        }

        let low_name = boost_to_lower(name);

        let mut i = 0usize;
        for printer in &self.m_printers {
            if boost_to_lower(&printer.name) == low_name {
                break;
            }
            i += 1;
        }
        if i == self.m_printers.len() {
            return self.m_printers.len();
        }

        i
    }
}

/// `boost::to_lower_copy` in the classic locale (ASCII letters).
fn boost_to_lower(s: &str) -> String {
    s.to_ascii_lowercase()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_type_string() {
        assert_eq!(Type::get_type_string(Type::TypeFilament), "filament");
        assert_eq!(Type::get_type_string(Type::TypePrint), "process");
        assert_eq!(Type::get_type_string(Type::TypePrinter), "machine");
        assert_eq!(
            Type::get_type_string(Type::TypePhysicalPrinter),
            "physical_printer"
        );
        assert_eq!(Type::get_type_string(Type::TypeInvalid), "invalid");
        assert_eq!(Type::get_type_string(Type::TypePlate), "invalid");
    }

    #[test]
    fn test_get_type_from_string() {
        assert_eq!(Type::get_type_from_string("process"), Type::TypePrint);
        assert_eq!(Type::get_type_from_string("print"), Type::TypePrint);
        assert_eq!(Type::get_type_from_string("filament"), Type::TypeFilament);
        assert_eq!(Type::get_type_from_string("machine"), Type::TypePrinter);
        assert_eq!(Type::get_type_from_string("printer"), Type::TypePrinter);
        assert_eq!(Type::get_type_from_string("bogus"), Type::TypeInvalid);
    }

    #[test]
    fn test_remove_suffix_modified() {
        assert_eq!(Preset::remove_suffix_modified(" (modified)foo"), "foo");
        assert_eq!(Preset::remove_suffix_modified("foo"), "foo");
    }

    #[test]
    fn test_label() {
        let mut p = Preset::new(Type::TypePrint, "MyPreset", false);
        assert_eq!(p.label(true), "MyPreset");
        p.alias = "Alias".to_string();
        assert_eq!(p.label(false), "Alias");
        assert_eq!(p.label(true), "MyPreset");
        p.is_dirty = true;
        assert_eq!(p.label(true), " (modified)MyPreset");
    }

    #[test]
    fn test_physical_printer_names() {
        let p = PhysicalPrinter {
            name: "Printer".to_string(),
            ..Default::default()
        };
        assert_eq!(p.get_full_name("Preset"), "Printer * Preset");
        assert_eq!(PhysicalPrinter::get_short_name("Printer * Preset"), "Printer");
        assert_eq!(PhysicalPrinter::get_preset_name("Printer * Preset"), "Preset");
    }
}
