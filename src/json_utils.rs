use serde_json::{Map, Value};
use std::{fs, path::Path};

use crate::locations::object_location;

pub fn location_value(value: &Value, context: &str) -> Result<String, String> {
    if let Some(location) = value.as_str() {
        return Ok(location.to_owned());
    }
    if let Some(location) = object_location(value)? {
        return Ok(location);
    }
    Err(format!("{context} must be a location string or object"))
}

pub fn string_field(obj: &Map<String, Value>, key: &str) -> Result<String, String> {
    obj.get(key)
        .map(|value| string_value(value, key))
        .unwrap_or_else(|| Err(format!("{key} is required")))
}

pub fn optional_string(obj: &Map<String, Value>, key: &str) -> Option<String> {
    obj.get(key).and_then(Value::as_str).map(str::to_owned)
}

pub fn string_value(value: &Value, context: &str) -> Result<String, String> {
    value
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| format!("{context} must be a string"))
}

pub fn expect_object<'a>(
    value: &'a Value,
    context: &str,
) -> Result<&'a Map<String, Value>, String> {
    value
        .as_object()
        .ok_or_else(|| format!("{context} must be a JSON object"))
}

pub fn overlay(base: &mut Map<String, Value>, next: Map<String, Value>) {
    for (key, value) in next {
        base.insert(key, value);
    }
}

pub fn write_json(path: &Path, value: &Value) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("create {}: {e}", parent.display()))?;
    }
    let file = fs::File::create(path).map_err(|e| format!("create {}: {e}", path.display()))?;
    serde_json::to_writer_pretty(file, value).map_err(|e| format!("write {}: {e}", path.display()))
}
