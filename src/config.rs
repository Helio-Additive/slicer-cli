use base64::{engine::general_purpose, Engine as _};
use serde_json::Value;
use std::path::Path;

use crate::{
    job_config::{ConfigSource, JobInput, JobOutput, Location},
    json_utils::location_value,
    locations::materialize_input,
    profiles::{load_profile, resolve_config_value},
};

pub use crate::job_config::JobConfig;

impl JobConfig {
    pub fn load(path: &Path) -> Result<Self, String> {
        let value = load_profile(path)?;
        serde_json::from_value(value).map_err(|e| format!("parse job config: {e}"))
    }

    pub fn load_arg(config: &str) -> Result<Self, String> {
        let path = Path::new(config);
        if path.exists() {
            return Self::load(path);
        }

        let value = decode_base64_config(config).map_err(|decode_err| {
            format!("--config must be an existing config path or base64-encoded JSON: {decode_err}")
        })?;
        serde_json::from_value(value).map_err(|e| format!("parse base64 job config: {e}"))
    }

    pub fn input_config(
        &self,
        temp_dirs: &mut Vec<tempfile::TempDir>,
    ) -> Result<Option<Value>, String> {
        let Some(config) = self.input.config.as_ref().or(self.config.as_ref()) else {
            return Ok(None);
        };
        let config = config.value(temp_dirs)?;
        ensure_profile_triple(&config)?;

        Ok(Some(Value::Object(resolve_config_value(
            &config, temp_dirs,
        )?)))
    }
}

impl JobInput {
    pub fn location(&self) -> Result<String, String> {
        first_location(
            [
                self.model.as_ref(),
                self.file.as_ref(),
                self.location.as_ref(),
            ],
            "input",
        )
    }

    pub fn kind(&self, location: &str) -> Result<String, String> {
        if let Some(kind) = self.type_name.as_ref().or(self.kind.as_ref()) {
            let kind = kind.to_ascii_lowercase();
            return match kind.as_str() {
                "stl" | "3mf" => Ok(kind),
                "model" => infer_kind_from_location(location),
                _ => Err(format!("unsupported input type: {kind}")),
            };
        }
        infer_kind_from_location(location)
    }
}

impl JobOutput {
    pub fn location(&self) -> Result<String, String> {
        first_location(
            [
                self.gcode.as_ref(),
                self.file.as_ref(),
                self.location.as_ref(),
            ],
            "output",
        )
    }

    pub fn resolved_config_location(&self) -> Result<Option<String>, String> {
        self.resolved_config
            .as_ref()
            .map(Location::as_location_string)
            .transpose()
    }
}

impl Location {
    pub fn as_location_string(&self) -> Result<String, String> {
        match self {
            Self::String(location) => Ok(location.clone()),
            Self::Object(obj) => location_value(&Value::Object(obj.clone()), "location"),
        }
    }
}

impl ConfigSource {
    pub fn value(&self, temp_dirs: &mut Vec<tempfile::TempDir>) -> Result<Value, String> {
        match self {
            Self::Base64(config) => decode_base64_config(&config.base64),
            Self::File(config) => {
                load_config_location(&config.file.as_location_string()?, temp_dirs)
            }
            Self::Location(config) => {
                load_config_location(&config.location.as_location_string()?, temp_dirs)
            }
            Self::Path(config) => load_config_location(&config.path, temp_dirs),
            Self::Uri(config) => load_config_location(&config.uri, temp_dirs),
            Self::Inline(config) => serde_json::to_value(config)
                .map_err(|e| format!("serialize inline input.config: {e}")),
        }
    }
}

fn load_config_location(
    location: &str,
    temp_dirs: &mut Vec<tempfile::TempDir>,
) -> Result<Value, String> {
    let path = materialize_input(location, "input.config", temp_dirs)?;
    load_profile(&path)
}

fn decode_base64_config(encoded: &str) -> Result<Value, String> {
    let bytes = general_purpose::STANDARD
        .decode(encoded)
        .map_err(|e| format!("decode input.config base64: {e}"))?;
    let raw = String::from_utf8(bytes)
        .map_err(|e| format!("input.config base64 decoded non-UTF8 data: {e}"))?;
    serde_json::from_str(&raw).map_err(|e| format!("parse input.config base64 JSON: {e}"))
}

fn ensure_profile_triple(config: &Value) -> Result<(), String> {
    let obj = config
        .as_object()
        .ok_or_else(|| "STL input.config must be an object".to_owned())?;
    let profiles = obj
        .get("profiles")
        .and_then(Value::as_object)
        .unwrap_or(obj);

    if profiles.contains_key("machine")
        && profiles.contains_key("filament")
        && profiles.contains_key("process")
    {
        return Ok(());
    }

    Err("STL input.config must include machine, filament, and process configs".to_owned())
}

pub fn infer_kind_from_location(location: &str) -> Result<String, String> {
    let lower = location
        .split_once('?')
        .map_or(location, |(path, _)| path)
        .to_ascii_lowercase();
    if lower.ends_with(".stl") {
        Ok("stl".to_owned())
    } else if lower.ends_with(".3mf") {
        Ok("3mf".to_owned())
    } else {
        Err(format!(
            "could not infer input type from location, use input.type or --prepared-3mf: {location}"
        ))
    }
}

fn first_location<'a, I>(locations: I, context: &str) -> Result<String, String>
where
    I: IntoIterator<Item = Option<&'a Location>>,
{
    if let Some(location) = locations.into_iter().flatten().next() {
        return location.as_location_string();
    }
    Err(format!("{context} requires one of: model, file, location"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use base64::engine::general_purpose;
    use std::{fs, io::Write};
    use tempfile::TempDir;

    #[test]
    fn resolves_base64_config_source() {
        let encoded = general_purpose::STANDARD.encode(
            r#"{
                "machine": { "name": "machine", "nozzle_diameter": ["0.4"] },
                "filament": { "name": "filament", "filament_type": ["PLA"] },
                "process": { "name": "process", "layer_height": 0.2 }
            }"#,
        );
        let job: JobConfig = serde_json::from_value(serde_json::json!({
            "input": {
                "type": "stl",
                "model": "part.stl",
                "config": { "base64": encoded }
            },
            "output": {
                "gcode": "out.gcode"
            }
        }))
        .unwrap();

        let mut temp_dirs = Vec::new();
        let config = job.input_config(&mut temp_dirs).unwrap().unwrap();

        assert_eq!(config["layer_height"], serde_json::json!(0.2));
    }

    #[test]
    fn resolves_file_config_source() {
        let tmp = TempDir::new().unwrap();
        let config_path = tmp.path().join("config.json");
        let mut file = fs::File::create(&config_path).unwrap();
        file.write_all(
            br#"{
                "machine": { "name": "machine", "nozzle_diameter": ["0.4"] },
                "filament": { "name": "filament", "filament_type": ["PLA"] },
                "process": { "name": "process", "layer_height": 0.3 }
            }"#,
        )
        .unwrap();

        let job: JobConfig = serde_json::from_value(serde_json::json!({
            "input": {
                "type": "stl",
                "model": "part.stl",
                "config": { "file": config_path }
            },
            "output": {
                "gcode": "out.gcode"
            }
        }))
        .unwrap();

        let mut temp_dirs = Vec::new();
        let config = job.input_config(&mut temp_dirs).unwrap().unwrap();

        assert_eq!(config["layer_height"], serde_json::json!(0.3));
    }
}
