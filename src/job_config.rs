use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
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

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
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

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
pub struct SlicerConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub machine: Option<ProfileReference>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filament: Option<ProfileReference>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub process: Option<ProfileReference>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub profiles: Option<ProfileTriple>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub profile_root: Option<StringList>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub profile_roots: Option<StringList>,
    #[serde(flatten)]
    pub settings: Map<String, Value>,
}

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
pub struct ProfileTriple {
    pub machine: ProfileReference,
    pub filament: ProfileReference,
    pub process: ProfileReference,
}

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum ProfileReference {
    Name(String),
    File(FileProfileReference),
    Location(LocationProfileReference),
    Path(PathProfileReference),
    Uri(UriProfileReference),
    Inline(ProfileDefinition),
}

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(transparent)]
#[schemars(rename = "MachineProfile")]
#[allow(dead_code)]
pub struct MachineProfile(pub ProfileReference);

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(transparent)]
#[schemars(rename = "FilamentProfile")]
#[allow(dead_code)]
pub struct FilamentProfile(pub ProfileReference);

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(transparent)]
#[schemars(rename = "ProcessProfile")]
#[allow(dead_code)]
pub struct ProcessProfile(pub ProfileReference);

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FileProfileReference {
    pub file: Location,
}

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LocationProfileReference {
    pub location: Location,
}

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PathProfileReference {
    pub path: String,
}

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UriProfileReference {
    pub uri: String,
}

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
pub struct ProfileDefinition {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(rename = "type")]
    #[schemars(rename = "type")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub type_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inherits: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub include: Option<StringList>,
    #[serde(flatten)]
    pub settings: Map<String, Value>,
}

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum StringList {
    One(String),
    Many(Vec<String>),
}
