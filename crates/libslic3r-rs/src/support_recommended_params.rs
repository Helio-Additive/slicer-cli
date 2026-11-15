//! Port of `libslic3r/SupportRecommendedParams.hpp` and `SupportRecommendedParams.cpp`.
//!
//! The recommended support parameters of a (support material, model material, printer model)
//! combination, read from `profiles/BBL/filament/support_recommended_params.json` under the
//! resources directory, and the built-in rules for material pairs.
//!
//! C++ reference: `references/BambuStudio/src/libslic3r/SupportRecommendedParams.{hpp,cpp}`.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use crate::libslic3r::EPSILON;
use crate::utils::resources_dir;

/// SupportRecommendedParams.hpp:15 —
/// `using ParamValue = std::variant<double, bool, std::string, std::vector<double>>;`
#[derive(Debug, Clone, PartialEq)]
pub enum ParamValue {
    Double(f64),
    Bool(bool),
    String(String),
    VecDouble(Vec<f64>),
}

/// The alternatives of [`ParamValue`], for `FilamentCombinationParams::get<T>`: `std::get_if<T>`.
pub trait ParamValueType: Sized {
    fn get_if(value: &ParamValue) -> Option<&Self>;
}

impl ParamValueType for f64 {
    fn get_if(value: &ParamValue) -> Option<&Self> {
        match value {
            ParamValue::Double(v) => Some(v),
            _ => None,
        }
    }
}

impl ParamValueType for bool {
    fn get_if(value: &ParamValue) -> Option<&Self> {
        match value {
            ParamValue::Bool(v) => Some(v),
            _ => None,
        }
    }
}

impl ParamValueType for String {
    fn get_if(value: &ParamValue) -> Option<&Self> {
        match value {
            ParamValue::String(v) => Some(v),
            _ => None,
        }
    }
}

impl ParamValueType for Vec<f64> {
    fn get_if(value: &ParamValue) -> Option<&Self> {
        match value {
            ParamValue::VecDouble(v) => Some(v),
            _ => None,
        }
    }
}

/// SupportRecommendedParams.hpp:12-32
///
/// Recommended parameters for support filament combination
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FilamentCombinationParams {
    pub params: BTreeMap<String, ParamValue>,
}

impl FilamentCombinationParams {
    /// SupportRecommendedParams.hpp:17-19
    pub fn has_recommended_params(&self) -> bool {
        !self.params.is_empty()
    }

    /// SupportRecommendedParams.hpp:22-31
    pub fn get<T: ParamValueType + Clone>(&self, key: &str) -> Option<T> {
        if let Some(it) = self.params.get(key) {
            if let Some(val) = T::get_if(it) {
                return Some(val.clone());
            }
        }
        None
    }
}

/// SupportRecommendedParams.hpp:35-44
///
/// Looked up by (support_material, model_material, printer_model).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SupportRecommendedParams {
    /// Model material (type or name).
    pub model_material: String,
    /// "name" or "type".
    pub model_material_type: String,
    /// Support materials (types or names).
    pub support_material_list: Vec<String>,
    /// "name" or "type".
    pub support_material_type: String,
    /// The printer models (`printer_model`, as named) the entry applies to; empty disables the entry.
    pub printer_models: Vec<String>,
    /// The higher, the more it counts.
    pub priority: i32,
    pub params: FilamentCombinationParams,
}

/// SupportRecommendedParams.hpp:46-52
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SupportMaterialProperties {
    pub name: String,
    pub r#type: String,
    pub soluble: bool,
    pub support: bool,
}

/// SupportRecommendedParams.hpp:54-62
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BuiltInSupportMaterialRule {
    #[default]
    None,
    Soluble,
    DedicatedSupport,
    PetgForPla,
    PlaForPetg,
    PlaForTpu,
}

/// SupportRecommendedParams.hpp:64-70
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SupportMaterialCombination {
    /// A pointer into the table in the C++; `None` is `nullptr`.
    pub recommended_params: Option<&'static SupportRecommendedParams>,
    pub built_in_rule: BuiltInSupportMaterialRule,
    pub zero_top: bool,
    pub zero_bottom: bool,
}

/// SupportRecommendedParams.cpp:16-22
pub fn support_recommended_material_name(mut preset_name: String) -> String {
    if let Some(pos) = preset_name.as_bytes().iter().position(|b| *b == b'@') {
        // '@' is one byte, so `pos` is a character boundary.
        preset_name.truncate(pos);
    }
    // boost::trim_right: `std::isspace` in the classic locale.
    let trimmed = preset_name.trim_end_matches(|c| matches!(c, ' ' | '\t' | '\n' | '\x0b' | '\x0c' | '\r')).len();
    preset_name.truncate(trimmed);
    preset_name
}

/// SupportRecommendedParams.cpp:26-28
///
/// Keyed by "support_material|model_material|printer_model". Multiple typed entries may share the
/// textual key; lookup selects the highest-priority entry with matching name/type semantics.
pub type SupportRecommendedParamsMap = BTreeMap<String, Vec<SupportRecommendedParams>>;

/// SupportRecommendedParams.cpp:30-33
fn support_recommended_params_key(support_material: &str, model_material: &str, printer_model: &str) -> String {
    let mut key = String::with_capacity(support_material.len() + model_material.len() + printer_model.len() + 2);
    key.push_str(support_material);
    key.push('|');
    key.push_str(model_material);
    key.push('|');
    key.push_str(printer_model);
    key
}

/// SupportRecommendedParams.cpp:35-38
fn support_recommended_params_path() -> PathBuf {
    resources_dir().join("profiles").join("BBL").join("filament").join("support_recommended_params.json")
}

/// `std::stod(const std::string &)`: `strtod` over the string; `None` where the C++ throws —
/// `std::invalid_argument` when nothing converts, `std::out_of_range` when `strtod` reports
/// `ERANGE` (the value overflows, or underflows to a subnormal or to zero).
fn stod(text: &str) -> Option<f64> {
    let bytes = text.as_bytes();
    // A std::string handed to strtod ends at its first NUL.
    let bytes = &bytes[..bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len())];
    let (value, end) = crate::format::objparser::strtod(bytes, 0);
    if end == 0 {
        return None;
    }
    let consumed = &bytes[..end];
    let spelled_special = consumed.iter().any(|b| matches!(b.to_ascii_lowercase(), b'i' | b'n'))
        && !consumed.iter().any(|b| matches!(b, b'x' | b'X'));
    if spelled_special {
        // "inf", "infinity", "nan": not a range error.
        return Some(value);
    }
    if value.is_infinite() {
        return None;
    }
    if value == 0.0 || value.is_subnormal() {
        // Zero is a range error only when the digits of the mantissa are not all zero.
        let hex = consumed.iter().any(|b| matches!(b, b'x' | b'X'));
        let mantissa_end = consumed
            .iter()
            .position(|b| if hex { matches!(b, b'p' | b'P') } else { matches!(b, b'e' | b'E') })
            .unwrap_or(consumed.len());
        let mantissa = &consumed[..mantissa_end];
        let mantissa = match mantissa.iter().position(|b| matches!(b, b'x' | b'X')) {
            Some(x) => &mantissa[x + 1..],
            None => mantissa,
        };
        let nonzero = mantissa.iter().any(|b| b.is_ascii_hexdigit() && *b != b'0');
        if nonzero {
            return None;
        }
    }
    Some(value)
}

/// `value.get<double>()` of a nlohmann number: the stored integer converted, or the stored double.
fn json_number_as_double(number: &serde_json::Number) -> f64 {
    if let Some(v) = number.as_u64() {
        v as f64
    } else if let Some(v) = number.as_i64() {
        v as f64
    } else {
        number.as_f64().unwrap_or(0.0)
    }
}

/// `combo.contains(key)`: false for anything that is not an object.
fn json_get<'a>(value: &'a serde_json::Value, key: &str) -> Option<&'a serde_json::Value> {
    value.as_object().and_then(|object| object.get(key))
}

/// nlohmann reads the token `-0` as the integer 0 (`is_number_integer()`, `get<double>()` is
/// `+0.0`); serde_json reads it as the double `-0.0`. This drops the sign of every such token, so
/// that both see the integer 0. `-0.0`, `-0e0` and the like are doubles for both and stay.
pub(crate) fn json_minus_zero_as_integer(text: &[u8]) -> std::borrow::Cow<'_, [u8]> {
    let mut out: Option<Vec<u8>> = None;
    let mut in_string = false;
    let mut i = 0;
    while i < text.len() {
        let b = text[i];
        let mut keep = true;
        if in_string {
            if b == b'\\' {
                // The escaped character is copied with the backslash.
                if let Some(out) = out.as_mut() {
                    out.push(b);
                }
                i += 1;
                if i >= text.len() {
                    break;
                }
            } else if b == b'"' {
                in_string = false;
            }
        } else if b == b'"' {
            in_string = true;
        } else if b == b'-'
            && text.get(i + 1) == Some(&b'0')
            && !matches!(text.get(i + 2), Some(b'0'..=b'9' | b'.' | b'e' | b'E'))
        {
            keep = false;
            if out.is_none() {
                out = Some(text[..i].to_vec());
            }
        }
        if keep {
            if let Some(out) = out.as_mut() {
                out.push(text[i]);
            }
        }
        i += 1;
    }
    match out {
        Some(out) => std::borrow::Cow::Owned(out),
        None => std::borrow::Cow::Borrowed(text),
    }
}

/// SupportRecommendedParams.cpp:40-162
///
/// File-local in the C++ (anonymous namespace); public here so that the loader can be compared
/// with the C++ on files other than the one under the resources directory.
///
/// The JSON is read by `serde_json` where the C++ uses nlohmann::json. The two agree on everything
/// this function looks at except the token `-0`, which `json_minus_zero_as_integer` takes care of.
pub fn load_support_recommended_params_map(json_path: &Path) -> SupportRecommendedParamsMap {
    let mut support_recommended_params_map = SupportRecommendedParamsMap::new();

    if !json_path.exists() {
        log::warn!("Support recommended params JSON file not found: {}", json_path.display());
        return support_recommended_params_map;
    }

    let bytes = match std::fs::read(json_path) {
        Ok(bytes) => bytes,
        Err(_) => {
            log::error!("Failed to open support recommended params JSON file");
            return support_recommended_params_map;
        }
    };

    // `ifs >> j`: one JSON value; what follows it in the file is not read. nlohmann skips a UTF-8
    // byte order mark.
    let text: &[u8] = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF]).unwrap_or(&bytes);
    let text = json_minus_zero_as_integer(text);
    let j: serde_json::Value = match serde_json::Deserializer::from_slice(&text).into_iter::<serde_json::Value>().next() {
        Some(Ok(j)) => j,
        Some(Err(e)) => {
            log::error!("Error loading support recommended params JSON: {e}");
            return support_recommended_params_map;
        }
        None => {
            log::error!("Error loading support recommended params JSON: unexpected end of input");
            return support_recommended_params_map;
        }
    };

    if let Some(combinations) = json_get(&j, "combinations").and_then(|c| c.as_array()) {
        for combo in combinations {
            let mut rec_params = SupportRecommendedParams::default();

            // Load model_material (single string)
            if let Some(v) = json_get(combo, "model_material").and_then(|v| v.as_str()) {
                rec_params.model_material = v.to_string();
            }

            // Load model_material_type (default to "type")
            if let Some(v) = json_get(combo, "model_material_type").and_then(|v| v.as_str()) {
                rec_params.model_material_type = v.to_string();
            } else {
                rec_params.model_material_type = "type".to_string();
            }

            // Load support_material_list (array)
            if let Some(list) = json_get(combo, "support_material_list").and_then(|v| v.as_array()) {
                for mat in list {
                    if let Some(mat) = mat.as_str() {
                        rec_params.support_material_list.push(mat.to_string());
                    }
                }
            }

            // Load support_material_type (default to "type")
            if let Some(v) = json_get(combo, "support_material_type").and_then(|v| v.as_str()) {
                rec_params.support_material_type = v.to_string();
            } else {
                rec_params.support_material_type = "type".to_string();
            }

            // Load printer_models (array)
            if let Some(list) = json_get(combo, "printer_models").and_then(|v| v.as_array()) {
                for pm in list {
                    if let Some(pm) = pm.as_str() {
                        rec_params.printer_models.push(pm.to_string());
                    }
                }
            }

            // Load priority (default to 0)
            if let Some(serde_json::Value::Number(n)) = json_get(combo, "priority") {
                // `is_number_integer()`: a signed or an unsigned integer, not a float.
                // `get<int>()`: the stored 64 bit integer narrowed.
                if let Some(v) = n.as_i64() {
                    rec_params.priority = v as i32;
                } else if let Some(v) = n.as_u64() {
                    rec_params.priority = v as i32;
                }
            }

            // An entry without printer_models would apply to every machine, including those it was
            // never validated on, so require the scope to be explicit.
            if rec_params.model_material.is_empty()
                || rec_params.support_material_list.is_empty()
                || rec_params.printer_models.is_empty()
            {
                continue;
            }

            // Load recommended params
            if let Some(params) = json_get(combo, "recommended_params").and_then(|v| v.as_object()) {
                for (key, value) in params {
                    if (key == "support_top_z_distance" || key == "support_bottom_z_distance") && !value.is_number() {
                        log::warn!("Ignoring non-numeric support distance parameter: {key}");
                        continue;
                    }

                    if let serde_json::Value::Number(n) = value {
                        rec_params.params.params.insert(key.clone(), ParamValue::Double(json_number_as_double(n)));
                    } else if let Some(b) = value.as_bool() {
                        rec_params.params.params.insert(key.clone(), ParamValue::Bool(b));
                    } else if let Some(s) = value.as_str() {
                        rec_params.params.params.insert(key.clone(), ParamValue::String(s.to_string()));
                    } else if let Some(array) = value.as_array() {
                        let mut vec: Vec<f64> = Vec::new();
                        for elem in array {
                            if let serde_json::Value::Number(n) = elem {
                                vec.push(json_number_as_double(n));
                            } else if let Some(s) = elem.as_str() {
                                match stod(s) {
                                    Some(v) => vec.push(v),
                                    None => log::warn!("Failed to parse array element as double for {key}"),
                                }
                            }
                        }
                        if !vec.is_empty() {
                            rec_params.params.params.insert(key.clone(), ParamValue::VecDouble(vec));
                        }
                    }
                }
            }
            if !rec_params.params.has_recommended_params() {
                continue;
            }

            // Create key for each (support_material, printer_model): "support_material|model_material|printer_model"
            for support_mat in &rec_params.support_material_list {
                for printer_model in &rec_params.printer_models {
                    support_recommended_params_map
                        .entry(support_recommended_params_key(support_mat, &rec_params.model_material, printer_model))
                        .or_default()
                        .push(rec_params.clone());
                }
            }
        }
    }

    let mut entry_count: usize = 0;
    for item in support_recommended_params_map.values() {
        entry_count += item.len();
    }
    log::info!("Loaded {entry_count} support recommended params from JSON");
    support_recommended_params_map
}

/// SupportRecommendedParams.cpp:164-168
fn support_recommended_params() -> &'static SupportRecommendedParamsMap {
    static TABLE: OnceLock<SupportRecommendedParamsMap> = OnceLock::new();
    TABLE.get_or_init(|| load_support_recommended_params_map(&support_recommended_params_path()))
}

/// SupportRecommendedParams.cpp:172-175
pub fn load_support_recommended_params() {
    let _ = support_recommended_params();
}

/// SupportRecommendedParams.cpp:177-219
fn find_support_recommended_params(
    support_material_name: &str,
    support_material_type: &str,
    model_material_name: &str,
    model_material_type: &str,
    printer_model: &str,
) -> Option<&'static SupportRecommendedParams> {
    let table = support_recommended_params();
    struct MaterialQuery<'a> {
        support_material: &'a str,
        support_key_type: &'static str,
        model_material: &'a str,
        model_key_type: &'static str,
    }
    let queries: [MaterialQuery; 4] = [
        MaterialQuery { support_material: support_material_name, support_key_type: "name", model_material: model_material_name, model_key_type: "name" },
        MaterialQuery { support_material: support_material_type, support_key_type: "type", model_material: model_material_name, model_key_type: "name" },
        MaterialQuery { support_material: support_material_name, support_key_type: "name", model_material: model_material_type, model_key_type: "type" },
        MaterialQuery { support_material: support_material_type, support_key_type: "type", model_material: model_material_type, model_key_type: "type" },
    ];
    let mut best: Option<&'static SupportRecommendedParams> = None;
    let mut best_query_index = queries.len();
    for query_index in 0..queries.len() {
        let query = &queries[query_index];
        if query.support_material.is_empty() || query.model_material.is_empty() {
            continue;
        }
        let Some(it) =
            table.get(&support_recommended_params_key(query.support_material, query.model_material, printer_model))
        else {
            continue;
        };
        for entry in it {
            if entry.support_material_type != query.support_key_type || entry.model_material_type != query.model_key_type {
                continue;
            }
            // Query order resolves equal priorities from specific to general; within one key, the later
            // JSON entry wins as it did when the table stored a single value.
            let better = match best {
                None => true,
                Some(best) => {
                    entry.priority > best.priority || (entry.priority == best.priority && query_index == best_query_index)
                }
            };
            if better {
                best = Some(entry);
                best_query_index = query_index;
            }
        }
    }
    best
}

/// SupportRecommendedParams.cpp:221-266
///
/// Resolves the JSON recommendation and built-in material-pair rules in one place. An explicit JSON
/// top/bottom distance overrides the corresponding built-in decision; a missing field inherits it.
pub fn evaluate_support_material_combination(
    support_material: &SupportMaterialProperties,
    model_material: &SupportMaterialProperties,
    printer_model: &str,
) -> SupportMaterialCombination {
    let mut result = SupportMaterialCombination::default();
    if !printer_model.is_empty() {
        result.recommended_params = find_support_recommended_params(
            &support_material.name,
            &support_material.r#type,
            &model_material.name,
            &model_material.r#type,
            printer_model,
        );
    }

    if support_material.r#type == "PLA" && (model_material.r#type == "TPU" || model_material.r#type == "TPU-AMS") {
        result.built_in_rule = BuiltInSupportMaterialRule::PlaForTpu;
    } else if support_material.soluble {
        result.built_in_rule = BuiltInSupportMaterialRule::Soluble;
    } else if support_material.support {
        result.built_in_rule = BuiltInSupportMaterialRule::DedicatedSupport;
    } else if support_material.r#type == "PETG" && model_material.r#type == "PLA" {
        result.built_in_rule = BuiltInSupportMaterialRule::PetgForPla;
    } else if support_material.r#type == "PLA" && model_material.r#type == "PETG" {
        result.built_in_rule = BuiltInSupportMaterialRule::PlaForPetg;
    }

    let built_in_zero_top = result.built_in_rule != BuiltInSupportMaterialRule::None;
    let built_in_zero_bottom = support_material.r#type == "PVA" && model_material.r#type == "PLA";
    let json_top: Option<f64> = match result.recommended_params {
        None => None,
        Some(recommended_params) => recommended_params.params.get::<f64>("support_top_z_distance"),
    };
    let json_bottom: Option<f64> = match result.recommended_params {
        None => None,
        Some(recommended_params) => recommended_params.params.get::<f64>("support_bottom_z_distance"),
    };

    result.zero_top = match json_top {
        Some(json_top) => json_top.abs() < EPSILON,
        None => built_in_zero_top,
    };
    result.zero_bottom = match json_bottom {
        Some(json_bottom) => json_bottom.abs() < EPSILON,
        None => built_in_zero_bottom,
    };
    result
}
