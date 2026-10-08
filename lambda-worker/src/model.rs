use serde::{Deserialize, Serialize};
use serde_json::Value;

pub const SCHEMA_VERSION: u8 = 1;

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SliceRequest {
    pub schema_version: u8,
    pub job_id: String,
    pub input: InputObject,
    pub output: OutputObject,
    pub engine: Engine,
    pub profiles: Profiles,
    #[serde(default)]
    pub options: SliceOptions,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct InputObject {
    pub bucket: String,
    pub key: String,
    #[serde(default)]
    pub version_id: Option<String>,
    pub sha256: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct OutputObject {
    pub bucket: String,
    pub key: String,
}

#[derive(Debug, Clone, Copy, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Engine {
    Bambu,
    Orca,
}

impl Engine {
    pub fn executable(self) -> &'static str {
        match self {
            Self::Bambu => "slicer_cli",
            Self::Orca => "slicer_cli-orcaslicer",
        }
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(tag = "mode", rename_all = "camelCase", deny_unknown_fields)]
pub enum Profiles {
    Embedded,
    Explicit {
        machine: String,
        filament: String,
        process: String,
        #[serde(default)]
        config: Option<InputObject>,
    },
}

#[derive(Debug, Clone, Default, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SliceOptions {
    #[serde(default)]
    pub plate: Option<u32>,
    #[serde(default)]
    pub overrides: SliceOverrides,
}

#[derive(Debug, Clone, Default, Deserialize, Serialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SliceOverrides {
    #[serde(default)]
    pub layer_height_mm: Option<f64>,
    #[serde(default)]
    pub infill_percent: Option<u8>,
    #[serde(default)]
    pub perimeters: Option<u16>,
    #[serde(default)]
    pub nozzle_diameter_mm: Option<f64>,
    #[serde(default)]
    pub nozzle_temperature_c: Option<u16>,
    #[serde(default)]
    pub bed_temperature_c: Option<u16>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SliceResult {
    pub schema_version: u8,
    pub job_id: String,
    pub status: SliceStatus,
    pub engine: Engine,
    pub duration_ms: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub engine_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub engine_git_sha: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deployment_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub output: Option<OutputArtifact>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub diagnostics: Vec<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<FailureDetail>,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SliceStatus {
    Succeeded,
    Failed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct OutputArtifact {
    pub bucket: String,
    pub key: String,
    pub size_bytes: u64,
    pub sha256: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub etag: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version_id: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, thiserror::Error)]
#[error("{code}: {message}")]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct FailureDetail {
    pub code: String,
    pub message: String,
    pub retryable: bool,
}

impl FailureDetail {
    pub fn permanent(code: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            code: code.into(),
            message: message.into(),
            retryable: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exported_schema_examples_deserialize_as_worker_requests() {
        let schema: Value = serde_json::from_str(include_str!(
            "../../schemas/json/exported/slice-request.schema.json"
        ))
        .unwrap();
        let examples = schema["examples"].as_array().unwrap();
        assert_eq!(examples.len(), 2);

        for example in examples {
            let request: SliceRequest = serde_json::from_value(example.clone()).unwrap();
            assert_eq!(request.schema_version, SCHEMA_VERSION);
            assert_eq!(request.input.bucket, "helio-slicer-input-production");
            assert_eq!(request.output.bucket, "helio-slicer-output-production");
        }
    }
}
