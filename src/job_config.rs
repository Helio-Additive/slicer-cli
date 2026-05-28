use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::{Map, Value};
use std::path::PathBuf;

#[derive(Debug, Deserialize, JsonSchema)]
pub struct JobConfig {
    pub input: JobInput,
    pub output: JobOutput,
    pub native_binary: Option<PathBuf>,
    pub config: Option<ConfigSource>,
    pub callback: Option<Value>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct JobInput {
    #[serde(rename = "type")]
    #[schemars(rename = "type")]
    pub type_name: Option<String>,
    pub kind: Option<String>,
    pub model: Option<Location>,
    pub file: Option<Location>,
    pub location: Option<Location>,
    pub config: Option<ConfigSource>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct JobOutput {
    pub gcode: Option<Location>,
    pub file: Option<Location>,
    pub location: Option<Location>,
    pub resolved_config: Option<Location>,
    pub callback: Option<Value>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum Location {
    String(String),
    Object(Map<String, Value>),
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum ConfigSource {
    Base64(Base64Config),
    File(FileConfig),
    Location(LocationConfig),
    Path(PathConfig),
    Uri(UriConfig),
    Inline(SlicerConfig),
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Base64Config {
    pub base64: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FileConfig {
    pub file: Location,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LocationConfig {
    pub location: Location,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PathConfig {
    pub path: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UriConfig {
    pub uri: String,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct SlicerConfig(pub Map<String, Value>);
