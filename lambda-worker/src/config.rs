use std::{env, path::PathBuf, time::Duration};

use anyhow::{Context, Result, bail};
use url::Url;

#[derive(Debug, Clone)]
pub struct WorkerConfig {
    pub input_bucket: String,
    pub output_bucket: String,
    pub input_key_prefix: String,
    pub output_key_prefix: String,
    pub slicer_root: PathBuf,
    pub callback_url: Url,
    pub callback_hmac_secret: Vec<u8>,
    pub callback_attempts: usize,
    pub callback_timeout: Duration,
    pub slice_timeout: Duration,
    pub max_input_bytes: u64,
    pub max_config_bytes: u64,
    pub max_output_bytes: u64,
    pub max_3mf_entries: usize,
    pub max_3mf_expanded_bytes: u64,
    pub max_3mf_compression_ratio: u64,
    pub max_diagnostic_events: usize,
    pub max_diagnostic_bytes: usize,
    pub deployment_version: Option<String>,
}

impl WorkerConfig {
    pub fn from_env() -> Result<Self> {
        let callback_url = Url::parse(&required("CALLBACK_URL")?)
            .context("CALLBACK_URL must be an absolute URL")?;
        let allow_insecure = optional_bool("ALLOW_INSECURE_CALLBACK", false)?;
        if callback_url.scheme() != "https"
            && !(allow_insecure
                && matches!(callback_url.host_str(), Some("localhost" | "127.0.0.1")))
        {
            bail!("CALLBACK_URL must use HTTPS (HTTP is allowed only for local testing)");
        }

        let callback_hmac_secret = required("CALLBACK_HMAC_SECRET")?.into_bytes();
        if callback_hmac_secret.len() < 32 {
            bail!("CALLBACK_HMAC_SECRET must contain at least 32 bytes");
        }

        Ok(Self {
            input_bucket: required("INPUT_BUCKET")?,
            output_bucket: required("OUTPUT_BUCKET")?,
            input_key_prefix: optional("INPUT_KEY_PREFIX", "jobs/"),
            output_key_prefix: optional("OUTPUT_KEY_PREFIX", "jobs/"),
            slicer_root: PathBuf::from(optional("SLICER_ROOT", "/opt/slicer-cli")),
            callback_url,
            callback_hmac_secret,
            callback_attempts: optional_number("CALLBACK_ATTEMPTS", 3usize)?,
            callback_timeout: Duration::from_secs(optional_number(
                "CALLBACK_TIMEOUT_SECONDS",
                10u64,
            )?),
            slice_timeout: Duration::from_secs(optional_number("SLICE_TIMEOUT_SECONDS", 780u64)?),
            max_input_bytes: optional_number("MAX_INPUT_BYTES", 512 * 1024 * 1024u64)?,
            max_config_bytes: optional_number("MAX_CONFIG_BYTES", 4 * 1024 * 1024u64)?,
            max_output_bytes: optional_number("MAX_OUTPUT_BYTES", 4 * 1024 * 1024 * 1024u64)?,
            max_3mf_entries: optional_number("MAX_3MF_ENTRIES", 10_000usize)?,
            max_3mf_expanded_bytes: optional_number(
                "MAX_3MF_EXPANDED_BYTES",
                2 * 1024 * 1024 * 1024u64,
            )?,
            max_3mf_compression_ratio: optional_number("MAX_3MF_COMPRESSION_RATIO", 200u64)?,
            max_diagnostic_events: optional_number("MAX_DIAGNOSTIC_EVENTS", 200usize)?,
            max_diagnostic_bytes: optional_number("MAX_DIAGNOSTIC_BYTES", 256 * 1024usize)?,
            deployment_version: env::var("DEPLOYMENT_VERSION").ok(),
        })
    }
}

fn required(name: &str) -> Result<String> {
    env::var(name).with_context(|| format!("missing required environment variable {name}"))
}

fn optional(name: &str, default: &str) -> String {
    env::var(name).unwrap_or_else(|_| default.to_owned())
}

fn optional_number<T>(name: &str, default: T) -> Result<T>
where
    T: std::str::FromStr,
    T::Err: std::error::Error + Send + Sync + 'static,
{
    match env::var(name) {
        Ok(value) => value
            .parse()
            .with_context(|| format!("{name} must be a valid number")),
        Err(_) => Ok(default),
    }
}

fn optional_bool(name: &str, default: bool) -> Result<bool> {
    match env::var(name) {
        Ok(value) => match value.as_str() {
            "true" | "1" => Ok(true),
            "false" | "0" => Ok(false),
            _ => bail!("{name} must be true, false, 1, or 0"),
        },
        Err(_) => Ok(default),
    }
}
