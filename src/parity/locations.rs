//! Where a job's files are. The parity harness reads and writes local files
//! only; a remote location is refused with a sentence.

use serde_json::Value;
use std::path::{Path, PathBuf};
use tempfile::TempDir;

use super::json_utils::{string_value, write_json};

pub struct PreparedOutput {
    pub requested: String,
    pub local_path: PathBuf,
}

pub fn object_location(value: &Value) -> Result<Option<String>, String> {
    let Some(obj) = value.as_object() else {
        return Ok(None);
    };
    if obj.contains_key("name") || obj.contains_key("inherits") || obj.contains_key("type") {
        return Ok(None);
    }
    for key in ["location", "path", "uri"] {
        if let Some(value) = obj.get(key) {
            return string_value(value, key).map(Some);
        }
    }
    Ok(None)
}

fn refuse_remote(location: &str, label: &str) -> Result<(), String> {
    if is_remote_location(location) {
        return Err(format!(
            "{label} is a remote location ({location}); slicer-parity reads and writes local \
             files only. Download it first and give the local path."
        ));
    }
    Ok(())
}

pub fn materialize_input(
    location: &str,
    label: &str,
    _temp_dirs: &mut Vec<TempDir>,
) -> Result<PathBuf, String> {
    refuse_remote(location, label)?;
    Ok(PathBuf::from(location))
}

pub fn prepare_output(location: &str) -> Result<PreparedOutput, String> {
    refuse_remote(location, "output")?;
    Ok(PreparedOutput {
        requested: location.to_owned(),
        local_path: PathBuf::from(location),
    })
}

pub fn write_json_to_location(location: &str, value: &Value) -> Result<(), String> {
    refuse_remote(location, "output.resolved_config")?;
    write_json(Path::new(location), value)
}

pub fn is_s3_location(location: &str) -> bool {
    location.starts_with("s3://")
}

fn is_remote_location(location: &str) -> bool {
    is_s3_location(location) || location.starts_with("http://") || location.starts_with("https://")
}
