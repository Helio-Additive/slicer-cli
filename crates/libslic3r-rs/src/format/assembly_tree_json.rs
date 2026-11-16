//! Port of `Format/AssemblyTreeJson.cpp` and `Format/AssemblyTreeJson.hpp` of Bambu Studio's
//! libslic3r at the pin: the tree of the assembly view (`AssemblyTreeData`) and its JSON form.
//!
//! The types and inline functions of the .hpp come first, then the functions of the .cpp in its
//! order.
//!
//! JSON. The C++ uses `nlohmann::json` (not `ordered_json`), whose objects print their keys in byte
//! order. Here a document is a `serde_json::Value`, read by `nlohmann_parse` and printed by
//! `nlohmann_dump` of [`crate::desensitize`], through the `parse` and `dump` of
//! [`super::assembly_steps_json`]. The functions under "nlohmann::json members" are the members of
//! `basic_json` this unit calls (`value`, `get<T>`), with the conversions and the exceptions they
//! have in the nlohmann the engine bundles (3.10.4).
//!
//! FIDELITY-NOTE: a parse error. `nlohmann::json::parse` throws `parse_error` with a text that
//! names the position and the token; here the text `from_json_string` hands back in `error` is
//! [`super::assembly_steps_json::PARSE_ERROR_WHAT`]. The `type_error` texts are nlohmann's.

use std::sync::atomic::AtomicBool;

use serde_json::{Map, Value};

use super::assembly_steps_json::{dump, parse, type_name, JsonException};

// ---------------------------------------------------------------------------------------------
// nlohmann::json members
// ---------------------------------------------------------------------------------------------

/// `json.get<int>()`: detail/conversions/from_json.hpp:331-366, the overload for an arithmetic
/// type that is none of the number types of `basic_json`. A `static_cast<int>` of the stored
/// 64 bit integer, double (the reference build, arm64, saturates and takes a NaN to 0, as `as`
/// does) or boolean; `type_error.302` for anything else.
fn get_int(j: &Value) -> Result<i32, JsonException> {
    match j {
        Value::Number(n) => {
            if let Some(v) = n.as_u64() {
                Ok(v as i32)
            } else if let Some(v) = n.as_i64() {
                Ok(v as i32)
            } else {
                Ok(n.as_f64().unwrap_or(0.0) as i32)
            }
        }
        Value::Bool(b) => Ok(*b as i32),
        _ => Err(JsonException(format!("[json.exception.type_error.302] type must be number, but is {}", type_name(j)))),
    }
}

/// `json.get<bool>()`: from_json.hpp:78-85.
fn get_bool(j: &Value) -> Result<bool, JsonException> {
    match j {
        Value::Bool(b) => Ok(*b),
        _ => Err(JsonException(format!("[json.exception.type_error.302] type must be boolean, but is {}", type_name(j)))),
    }
}

/// `json.get<std::string>()`: from_json.hpp:88-95.
fn get_string(j: &Value) -> Result<String, JsonException> {
    match j {
        Value::String(s) => Ok(s.clone()),
        _ => Err(JsonException(format!("[json.exception.type_error.302] type must be string, but is {}", type_name(j)))),
    }
}

/// `json.get<std::vector<int>>()`: from_json.hpp:226-241 and 258-267, every element by
/// `get<int>()`.
fn get_vector_int(j: &Value) -> Result<Vec<i32>, JsonException> {
    match j {
        Value::Array(array) => {
            let mut ret: Vec<i32> = Vec::with_capacity(array.len());
            for i in array {
                ret.push(get_int(i)?);
            }
            Ok(ret)
        }
        _ => Err(JsonException(format!("[json.exception.type_error.302] type must be array, but is {}", type_name(j)))),
    }
}

/// `json.value(key, default_value)` (json.hpp:3989-4005): the value under `key` converted by
/// `get`, the default where the object has no such key, `type_error.306` where `json` is not an
/// object.
fn value<T>(j: &Value, key: &str, default_value: T, get: fn(&Value) -> Result<T, JsonException>) -> Result<T, JsonException> {
    // at only works for objects
    if let Value::Object(object) = j {
        // if key is found, return value and given default value otherwise
        if let Some(it) = object.get(key) {
            return get(it);
        }

        return Ok(default_value);
    }

    Err(JsonException(format!("[json.exception.type_error.306] cannot use value() with {}", type_name(j))))
}

// ---------------------------------------------------------------------------------------------
// AssemblyTreeJson.hpp
// ---------------------------------------------------------------------------------------------

/// AssemblyTreeJson.hpp:9-19
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AssemblyTreeNodeData {
    pub id: i32,
    pub parent_id: i32,
    pub object_idx: i32,
    pub volume_idx: i32,
    pub selectable: bool,
    pub uid: String,
    pub label: String,
    pub children: Vec<i32>,
}

impl Default for AssemblyTreeNodeData {
    fn default() -> Self {
        AssemblyTreeNodeData {
            id: -1,
            parent_id: -1,
            object_idx: -1,
            volume_idx: -1,
            selectable: true,
            uid: String::new(),
            label: String::new(),
            children: Vec::new(),
        }
    }
}

/// AssemblyTreeJson.hpp:21-39
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct AssemblyTreeData {
    pub nodes: Vec<AssemblyTreeNodeData>,
    pub roots: Vec<i32>,
    pub checked_node_indices: Vec<i32>,
}

/// AssemblyTreeJson.hpp:26 `static bool show_origin_step_tree`, AssemblyTreeJson.cpp:24
/// `bool AssemblyTreeData::show_origin_step_tree = false;`
pub static SHOW_ORIGIN_STEP_TREE: AtomicBool = AtomicBool::new(false);

impl AssemblyTreeData {
    /// AssemblyTreeJson.hpp:27
    pub fn empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// AssemblyTreeJson.hpp:28-33
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.roots.clear();
        self.checked_node_indices.clear();
    }
}

// ---------------------------------------------------------------------------------------------
// AssemblyTreeJson.cpp
// ---------------------------------------------------------------------------------------------

/// A `std::vector<int>` as a json: an array.
fn json_ints(values: &[i32]) -> Value {
    Value::Array(values.iter().map(|v| Value::from(*v)).collect())
}

/// AssemblyTreeJson.cpp:11-23
fn node_to_json(node: &AssemblyTreeNodeData) -> Value {
    let mut object = Map::new();
    object.insert("id".to_string(), Value::from(node.id));
    object.insert("parent_id".to_string(), Value::from(node.parent_id));
    object.insert("object_idx".to_string(), Value::from(node.object_idx));
    object.insert("volume_idx".to_string(), Value::from(node.volume_idx));
    object.insert("selectable".to_string(), Value::from(node.selectable));
    object.insert("uid".to_string(), Value::from(node.uid.clone()));
    object.insert("label".to_string(), Value::from(node.label.clone()));
    object.insert("children".to_string(), json_ints(&node.children));
    Value::Object(object)
}

/// AssemblyTreeJson.cpp:25-37
fn node_from_json(j: &Value) -> Result<AssemblyTreeNodeData, JsonException> {
    let mut node = AssemblyTreeNodeData::default();
    node.id = value(j, "id", -1, get_int)?;
    node.parent_id = value(j, "parent_id", -1, get_int)?;
    node.object_idx = value(j, "object_idx", -1, get_int)?;
    node.volume_idx = value(j, "volume_idx", -1, get_int)?;
    node.selectable = value(j, "selectable", true, get_bool)?;
    node.uid = value(j, "uid", String::new(), get_string)?;
    node.label = value(j, "label", String::new(), get_string)?;
    node.children = value(j, "children", Vec::new(), get_vector_int)?;
    Ok(node)
}

impl AssemblyTreeData {
    /// AssemblyTreeJson.cpp:39-49
    pub fn to_json_string(&self) -> String {
        let mut j = Map::new();
        j.insert("version".to_string(), Value::from(1)); //V1.0 2020610
        j.insert("roots".to_string(), json_ints(&self.roots));
        j.insert("checked_node_indices".to_string(), json_ints(&self.checked_node_indices));
        let mut nodes: Vec<Value> = Vec::new();
        for node in &self.nodes {
            nodes.push(node_to_json(node));
        }
        j.insert("nodes".to_string(), Value::Array(nodes));
        dump(&Value::Object(j), -1)
    }

    /// AssemblyTreeJson.cpp:51-77
    pub fn from_json_string(json_string: &str, tree: &mut AssemblyTreeData, error: Option<&mut String>) -> bool {
        let run = || -> Result<AssemblyTreeData, JsonException> {
            let j = parse(json_string)?;
            let mut parsed = AssemblyTreeData::default();
            parsed.roots = value(&j, "roots", Vec::new(), get_vector_int)?;
            // j.find(key): end() for what is not an object.
            let checked_node_indices_it = j.as_object().and_then(|object| object.get("checked_node_indices"));
            if let Some(checked_node_indices_it) = checked_node_indices_it {
                parsed.checked_node_indices = get_vector_int(checked_node_indices_it)?;
            }
            if let Some(Value::Array(nodes_it)) = j.as_object().and_then(|object| object.get("nodes")) {
                parsed.nodes.reserve(nodes_it.len());
                for node_json in nodes_it {
                    parsed.nodes.push(node_from_json(node_json)?);
                }
            }
            if checked_node_indices_it.is_none() {
                parsed.checked_node_indices.reserve(parsed.nodes.len());
                for node_idx in 0..parsed.nodes.len() {
                    parsed.checked_node_indices.push(if parsed.nodes[node_idx].id >= 0 { parsed.nodes[node_idx].id } else { node_idx as i32 });
                }
            }
            Ok(parsed)
        };
        match run() {
            Ok(parsed) => {
                *tree = parsed;
                true
            }
            Err(e) => {
                if let Some(error) = error {
                    *error = e.what().to_string();
                }
                false
            }
        }
    }
}
