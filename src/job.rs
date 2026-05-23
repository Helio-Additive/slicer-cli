use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// External contract — what K8s callers / other services produce.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct JobSpec {
    pub job_id: String,

    /// Where to fetch the input mesh. Local path or http(s) URI.
    pub input: InputRef,

    /// Where to put the resulting G-code. Local path or s3/http target.
    pub output: OutputTarget,

    /// Optional post-slice webhooks (status, completion).
    #[serde(default)]
    pub callbacks: Callbacks,

    /// BambuStudio machine preset name (e.g. "Bambu Lab X1 Carbon 0.4 nozzle").
    pub machine: String,

    /// BambuStudio filament preset names, one per slot.
    pub filament: Vec<String>,

    /// BambuStudio process preset name (e.g. "0.20mm Standard @BBL X1C").
    pub process: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum InputRef {
    Path { path: String },
    Url { url: String },
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum OutputTarget {
    Path { path: String },
    S3 { bucket: String, key: String },
    HttpPut { url: String },
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
pub struct Callbacks {
    #[serde(default)]
    pub event_webhook: Option<String>,
    #[serde(default)]
    pub completion_webhook: Option<String>,
}

