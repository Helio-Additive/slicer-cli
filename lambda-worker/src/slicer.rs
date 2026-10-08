use std::{
    ffi::OsString,
    path::{Component, Path, PathBuf},
    process::Stdio,
    time::Instant,
};

use anyhow::{Context, Result};
use serde_json::{Value, json};
use tempfile::TempDir;
use tokio::{
    io::{AsyncRead, AsyncReadExt},
    process::Command,
    time::timeout,
};

use crate::{
    config::WorkerConfig,
    model::{FailureDetail, Profiles, SCHEMA_VERSION, SliceRequest, SliceResult, SliceStatus},
    storage::{Storage, TransferError},
};

const EVENT_PREFIX: &str = "[[SLICER_EVENT]] ";
const FAILURE_LOG_BYTES: usize = 16 * 1024;
const MAX_OUTPUT_LINE_BYTES: usize = 1024 * 1024;

#[derive(Debug, Clone, Default)]
pub struct EngineIdentity {
    pub version: Option<String>,
    pub git_sha: Option<String>,
}

pub async fn execute(
    config: &WorkerConfig,
    storage: &Storage,
    identity: &EngineIdentity,
    request: &SliceRequest,
) -> Result<SliceResult> {
    let started = Instant::now();
    if let Err(failure) = validate_request(config, request) {
        return Ok(failed_result(
            config,
            identity,
            request,
            started,
            failure,
            vec![],
        ));
    }

    let temp_dir = tempfile::Builder::new()
        .prefix(&format!("slicer-{}-", request.job_id))
        .tempdir_in("/tmp")
        .context("creating per-job temporary directory")?;
    let extension = input_extension(&request.input.key).expect("validated input extension");
    let input_path = temp_dir.path().join(format!("input.{extension}"));
    let output_path = temp_dir.path().join("output.gcode");

    if let Err(error) = storage
        .download_verified(&request.input, &input_path, config.max_input_bytes)
        .await
    {
        match error {
            TransferError::Rejected(message) => {
                return Ok(failed_result(
                    config,
                    identity,
                    request,
                    started,
                    FailureDetail::permanent("InputRejected", message),
                    vec![],
                ));
            }
            TransferError::Unavailable(error) => {
                return Err(error.context("retrieving slice input"));
            }
        }
    }

    if extension == "3mf" {
        let path = input_path.clone();
        let max_entries = config.max_3mf_entries;
        let max_expanded = config.max_3mf_expanded_bytes;
        let max_ratio = config.max_3mf_compression_ratio;
        let validation = tokio::task::spawn_blocking(move || {
            validate_3mf(&path, max_entries, max_expanded, max_ratio)
        })
        .await
        .context("joining 3MF validation task")?;
        if let Err(failure) = validation {
            return Ok(failed_result(
                config,
                identity,
                request,
                started,
                failure,
                vec![],
            ));
        }
    }

    let config_path = match &request.profiles {
        Profiles::Explicit {
            config: Some(config_object),
            ..
        } => {
            let path = temp_dir.path().join("resolved-config.json");
            if let Err(error) = storage
                .download_verified(config_object, &path, config.max_config_bytes)
                .await
            {
                match error {
                    TransferError::Rejected(message) => {
                        return Ok(failed_result(
                            config,
                            identity,
                            request,
                            started,
                            FailureDetail::permanent("ProfileConfigRejected", message),
                            vec![],
                        ));
                    }
                    TransferError::Unavailable(error) => {
                        return Err(error.context("retrieving resolved profile config"));
                    }
                }
            }
            Some(path)
        }
        _ => None,
    };

    let arguments = match build_arguments(
        config,
        request,
        &input_path,
        &output_path,
        config_path.as_deref(),
    ) {
        Ok(arguments) => arguments,
        Err(failure) => {
            return Ok(failed_result(
                config,
                identity,
                request,
                started,
                failure,
                vec![],
            ));
        }
    };

    let executable = config
        .slicer_root
        .join("bin")
        .join(request.engine.executable());
    let run = run_slicer(config, &temp_dir, &executable, &arguments).await?;
    let duration_ms = elapsed_millis(started);

    if run.timed_out {
        return Ok(failed_result_with_duration(
            config,
            identity,
            request,
            duration_ms,
            FailureDetail::permanent(
                "SliceTimedOut",
                format!(
                    "slicer exceeded the {} second worker deadline",
                    config.slice_timeout.as_secs()
                ),
            ),
            run.diagnostics,
        ));
    }

    if !run.status.success() {
        let message = first_nonempty(&run.stderr_tail, &run.stdout_tail)
            .unwrap_or("slicer exited without an error message");
        return Ok(failed_result_with_duration(
            config,
            identity,
            request,
            duration_ms,
            FailureDetail::permanent(
                "SlicerFailed",
                format!("slicer exited with {}: {message}", run.status),
            ),
            run.diagnostics,
        ));
    }

    if let Some(event) = run.diagnostics.iter().find(|event| fatal_diagnostic(event)) {
        let event_name = event
            .get("event")
            .and_then(Value::as_str)
            .unwrap_or("unknown");
        return Ok(failed_result_with_duration(
            config,
            identity,
            request,
            duration_ms,
            FailureDetail::permanent(
                "UnsafeSlicerDiagnostic",
                format!("slicer reported a configuration failure: {event_name}"),
            ),
            run.diagnostics,
        ));
    }

    let output = match storage
        .upload_output(
            &request.job_id,
            &request.output,
            &output_path,
            config.max_output_bytes,
        )
        .await
    {
        Ok(output) => output,
        Err(TransferError::Rejected(message)) => {
            return Ok(failed_result_with_duration(
                config,
                identity,
                request,
                duration_ms,
                FailureDetail::permanent("OutputRejected", message),
                run.diagnostics,
            ));
        }
        Err(TransferError::Unavailable(error)) => {
            return Err(error.context("publishing slice output"));
        }
    };

    Ok(SliceResult {
        schema_version: SCHEMA_VERSION,
        job_id: request.job_id.clone(),
        status: SliceStatus::Succeeded,
        engine: request.engine,
        duration_ms,
        engine_version: identity.version.clone(),
        engine_git_sha: identity.git_sha.clone(),
        deployment_version: config.deployment_version.clone(),
        output: Some(output),
        diagnostics: run.diagnostics,
        error: None,
    })
}

pub fn validate_request(
    config: &WorkerConfig,
    request: &SliceRequest,
) -> std::result::Result<(), FailureDetail> {
    if request.schema_version != SCHEMA_VERSION {
        return Err(FailureDetail::permanent(
            "UnsupportedSchemaVersion",
            format!(
                "schemaVersion {} is not supported; expected {SCHEMA_VERSION}",
                request.schema_version
            ),
        ));
    }
    if request.job_id.is_empty()
        || request.job_id.len() > 128
        || !request
            .job_id
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_'))
    {
        return Err(FailureDetail::permanent(
            "InvalidJobId",
            "jobId must contain 1-128 ASCII letters, digits, hyphens, or underscores",
        ));
    }
    if request.input.bucket != config.input_bucket || request.output.bucket != config.output_bucket
    {
        return Err(FailureDetail::permanent(
            "InvalidBucket",
            "input.bucket and output.bucket must match the worker's configured buckets",
        ));
    }
    validate_object_key(&request.input.key, &config.input_key_prefix, "input.key")?;
    validate_object_key(&request.output.key, &config.output_key_prefix, "output.key")?;
    let input_job_prefix = job_object_prefix(&config.input_key_prefix, &request.job_id);
    let output_job_prefix = job_object_prefix(&config.output_key_prefix, &request.job_id);
    if !request.input.key.starts_with(&input_job_prefix)
        || !request.output.key.starts_with(&output_job_prefix)
    {
        return Err(FailureDetail::permanent(
            "CrossJobObjectKey",
            "input.key and output.key must be inside this job's object prefix",
        ));
    }
    if !request.output.key.to_ascii_lowercase().ends_with(".gcode") {
        return Err(FailureDetail::permanent(
            "InvalidOutputKey",
            "output.key must end in .gcode",
        ));
    }
    let extension = input_extension(&request.input.key).ok_or_else(|| {
        FailureDetail::permanent(
            "UnsupportedInputFormat",
            "input.key must end in .stl or .3mf",
        )
    })?;
    validate_sha256(&request.input.sha256, "input.sha256")?;

    if extension == "stl" && matches!(&request.profiles, Profiles::Embedded) {
        return Err(FailureDetail::permanent(
            "ProfilesRequired",
            "STL inputs require explicit machine, filament, and process profiles",
        ));
    }
    if let Profiles::Explicit {
        config: Some(object),
        ..
    } = &request.profiles
    {
        if object.bucket != config.input_bucket {
            return Err(FailureDetail::permanent(
                "InvalidBucket",
                "profiles.config.bucket must match the worker's configured input bucket",
            ));
        }
        validate_object_key(&object.key, &config.input_key_prefix, "profiles.config.key")?;
        if !object.key.starts_with(&input_job_prefix) {
            return Err(FailureDetail::permanent(
                "CrossJobObjectKey",
                "profiles.config.key must be inside this job's object prefix",
            ));
        }
        validate_sha256(&object.sha256, "profiles.config.sha256")?;
        if !object.key.to_ascii_lowercase().ends_with(".json") {
            return Err(FailureDetail::permanent(
                "InvalidProfileConfig",
                "profiles.config.key must end in .json",
            ));
        }
    }

    if matches!(request.options.plate, Some(0)) {
        return Err(FailureDetail::permanent(
            "InvalidPlate",
            "options.plate is 1-based and must be greater than zero",
        ));
    }
    let overrides = &request.options.overrides;
    if overrides
        .layer_height_mm
        .is_some_and(|value| !value.is_finite() || !(0.01..=5.0).contains(&value))
    {
        return Err(FailureDetail::permanent(
            "InvalidLayerHeight",
            "layerHeightMm must be between 0.01 and 5.0",
        ));
    }
    if overrides.infill_percent.is_some_and(|value| value > 100) {
        return Err(FailureDetail::permanent(
            "InvalidInfill",
            "infillPercent must be between 0 and 100",
        ));
    }
    if overrides
        .perimeters
        .is_some_and(|value| !(1..=100).contains(&value))
    {
        return Err(FailureDetail::permanent(
            "InvalidPerimeters",
            "perimeters must be between 1 and 100",
        ));
    }
    if overrides
        .nozzle_diameter_mm
        .is_some_and(|value| !value.is_finite() || !(0.1..=2.0).contains(&value))
    {
        return Err(FailureDetail::permanent(
            "InvalidNozzleDiameter",
            "nozzleDiameterMm must be between 0.1 and 2.0",
        ));
    }
    if overrides
        .nozzle_temperature_c
        .is_some_and(|value| value > 500)
    {
        return Err(FailureDetail::permanent(
            "InvalidNozzleTemperature",
            "nozzleTemperatureC must not exceed 500",
        ));
    }
    if overrides.bed_temperature_c.is_some_and(|value| value > 200) {
        return Err(FailureDetail::permanent(
            "InvalidBedTemperature",
            "bedTemperatureC must not exceed 200",
        ));
    }
    Ok(())
}

fn validate_object_key(
    key: &str,
    prefix: &str,
    field: &str,
) -> std::result::Result<(), FailureDetail> {
    if key.is_empty()
        || key.len() > 1024
        || !key.starts_with(prefix)
        || key.contains('\0')
        || key
            .split('/')
            .any(|component| component.is_empty() || matches!(component, "." | ".."))
    {
        return Err(FailureDetail::permanent(
            "InvalidObjectKey",
            format!("{field} must be a normalized key within the configured {prefix:?} prefix"),
        ));
    }
    Ok(())
}

fn job_object_prefix(prefix: &str, job_id: &str) -> String {
    format!("{}/{job_id}/", prefix.trim_end_matches('/'))
}

fn validate_sha256(value: &str, field: &str) -> std::result::Result<(), FailureDetail> {
    if value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(FailureDetail::permanent(
            "InvalidSha256",
            format!("{field} must contain exactly 64 hexadecimal characters"),
        ));
    }
    Ok(())
}

fn input_extension(key: &str) -> Option<&'static str> {
    let lower = key.to_ascii_lowercase();
    if lower.ends_with(".stl") {
        Some("stl")
    } else if lower.ends_with(".3mf") {
        Some("3mf")
    } else {
        None
    }
}

fn build_arguments(
    config: &WorkerConfig,
    request: &SliceRequest,
    input_path: &Path,
    output_path: &Path,
    config_path: Option<&Path>,
) -> std::result::Result<Vec<OsString>, FailureDetail> {
    let mut arguments = vec![input_path.as_os_str().to_owned()];
    if let Profiles::Explicit {
        machine,
        filament,
        process,
        ..
    } = &request.profiles
    {
        if let Some(path) = config_path {
            arguments.push("--config".into());
            arguments.push(path.as_os_str().to_owned());
        }
        for (flag, relative) in [
            ("--machine", machine),
            ("--filament", filament),
            ("--process", process),
        ] {
            arguments.push(flag.into());
            arguments.push(resolve_packaged_file(&config.slicer_root, relative)?.into_os_string());
        }
    }
    if let Some(plate) = request.options.plate {
        arguments.push("--plate".into());
        arguments.push(plate.to_string().into());
    }
    let overrides = &request.options.overrides;
    push_value(&mut arguments, "--layer-height", overrides.layer_height_mm);
    push_value(&mut arguments, "--infill", overrides.infill_percent);
    push_value(&mut arguments, "--perimeters", overrides.perimeters);
    push_value(&mut arguments, "--nozzle", overrides.nozzle_diameter_mm);
    push_value(&mut arguments, "--temp", overrides.nozzle_temperature_c);
    push_value(&mut arguments, "--bed-temp", overrides.bed_temperature_c);
    arguments.push("--output".into());
    arguments.push(output_path.as_os_str().to_owned());
    Ok(arguments)
}

fn push_value<T: ToString>(arguments: &mut Vec<OsString>, flag: &str, value: Option<T>) {
    if let Some(value) = value {
        arguments.push(flag.into());
        arguments.push(value.to_string().into());
    }
}

fn resolve_packaged_file(
    root: &Path,
    relative: &str,
) -> std::result::Result<PathBuf, FailureDetail> {
    let path = Path::new(relative);
    if !relative.to_ascii_lowercase().ends_with(".json") {
        return Err(FailureDetail::permanent(
            "InvalidProfilePath",
            "packaged profile paths must end in .json",
        ));
    }
    if path.is_absolute()
        || path
            .components()
            .any(|component| !matches!(component, Component::Normal(_)))
    {
        return Err(FailureDetail::permanent(
            "InvalidProfilePath",
            "profile paths must be relative paths without traversal",
        ));
    }
    let canonical_root = root.canonicalize().map_err(|error| {
        FailureDetail::permanent(
            "SlicerPackageInvalid",
            format!("cannot resolve slicer root: {error}"),
        )
    })?;
    let resolved = root.join(path).canonicalize().map_err(|error| {
        FailureDetail::permanent(
            "ProfileNotFound",
            format!("cannot resolve packaged profile {relative:?}: {error}"),
        )
    })?;
    if !resolved.starts_with(&canonical_root) || !resolved.is_file() {
        return Err(FailureDetail::permanent(
            "InvalidProfilePath",
            format!("profile path {relative:?} is outside the slicer package"),
        ));
    }
    Ok(resolved)
}

fn validate_3mf(
    path: &Path,
    max_entries: usize,
    max_expanded_bytes: u64,
    max_ratio: u64,
) -> std::result::Result<(), FailureDetail> {
    let file = std::fs::File::open(path).map_err(|error| {
        FailureDetail::permanent("Invalid3mf", format!("cannot open 3MF: {error}"))
    })?;
    let mut archive = zip::ZipArchive::new(file).map_err(|error| {
        FailureDetail::permanent("Invalid3mf", format!("invalid 3MF ZIP container: {error}"))
    })?;
    if archive.len() > max_entries {
        return Err(FailureDetail::permanent(
            "ThreeMfEntryLimit",
            format!(
                "3MF contains {} entries; limit is {max_entries}",
                archive.len()
            ),
        ));
    }
    let mut expanded = 0u64;
    for index in 0..archive.len() {
        let entry = archive.by_index_raw(index).map_err(|error| {
            FailureDetail::permanent("Invalid3mf", format!("cannot inspect 3MF entry: {error}"))
        })?;
        if entry.enclosed_name().is_none() {
            return Err(FailureDetail::permanent(
                "UnsafeThreeMfPath",
                format!("3MF entry {:?} contains an unsafe path", entry.name()),
            ));
        }
        expanded = expanded.checked_add(entry.size()).ok_or_else(|| {
            FailureDetail::permanent("ThreeMfExpandedLimit", "3MF expanded size overflowed")
        })?;
        if expanded > max_expanded_bytes {
            return Err(FailureDetail::permanent(
                "ThreeMfExpandedLimit",
                format!("3MF expands to more than {max_expanded_bytes} bytes"),
            ));
        }
        let compressed = entry.compressed_size();
        if entry.size() > 0
            && (compressed == 0
                || u128::from(entry.size()) > u128::from(compressed) * u128::from(max_ratio))
        {
            return Err(FailureDetail::permanent(
                "ThreeMfCompressionRatio",
                format!(
                    "3MF entry {:?} exceeds the compression ratio limit",
                    entry.name()
                ),
            ));
        }
    }
    Ok(())
}

struct RunResult {
    status: std::process::ExitStatus,
    timed_out: bool,
    diagnostics: Vec<Value>,
    stdout_tail: String,
    stderr_tail: String,
}

async fn run_slicer(
    config: &WorkerConfig,
    temp_dir: &TempDir,
    executable: &Path,
    arguments: &[OsString],
) -> Result<RunResult> {
    let mut child = Command::new(executable)
        .args(arguments)
        .current_dir(temp_dir.path())
        .env("HOME", temp_dir.path())
        .env("TMPDIR", temp_dir.path())
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()
        .with_context(|| format!("starting {}", executable.display()))?;
    let stdout = child.stdout.take().context("capturing slicer stdout")?;
    let stderr = child.stderr.take().context("capturing slicer stderr")?;
    let stdout_task = tokio::spawn(capture_stream(
        stdout,
        true,
        config.max_diagnostic_events,
        config.max_diagnostic_bytes,
    ));
    let stderr_task = tokio::spawn(capture_stream(stderr, false, 0, 0));

    let (status, timed_out) = match timeout(config.slice_timeout, child.wait()).await {
        Ok(status) => (status.context("waiting for slicer")?, false),
        Err(_) => {
            child.kill().await.context("killing timed-out slicer")?;
            (
                child.wait().await.context("reaping timed-out slicer")?,
                true,
            )
        }
    };
    let stdout = stdout_task.await.context("joining stdout reader")??;
    let stderr = stderr_task.await.context("joining stderr reader")??;
    Ok(RunResult {
        status,
        timed_out,
        diagnostics: stdout.events,
        stdout_tail: stdout.tail,
        stderr_tail: stderr.tail,
    })
}

struct StreamCapture {
    events: Vec<Value>,
    tail: String,
}

async fn capture_stream<R: AsyncRead + Unpin>(
    mut reader: R,
    parse_events: bool,
    max_events: usize,
    max_event_bytes: usize,
) -> Result<StreamCapture> {
    let mut pending = Vec::new();
    let mut buffer = [0u8; 8192];
    let mut discarding_line = false;
    let mut tail = String::new();
    let mut events = Vec::new();
    let mut event_bytes = 0usize;
    let mut omitted = false;
    loop {
        let count = reader
            .read(&mut buffer)
            .await
            .context("reading slicer output")?;
        if count == 0 {
            break;
        }
        append_tail(
            &mut tail,
            &String::from_utf8_lossy(&buffer[..count]),
            FAILURE_LOG_BYTES,
        );
        let mut start = 0usize;
        for end in buffer[..count]
            .iter()
            .enumerate()
            .filter_map(|(index, byte)| (*byte == b'\n').then_some(index))
        {
            if !discarding_line {
                append_line_bytes(
                    &mut pending,
                    &buffer[start..end],
                    &mut discarding_line,
                    &mut omitted,
                    parse_events,
                );
                capture_event(
                    &pending,
                    parse_events,
                    max_events,
                    max_event_bytes,
                    &mut event_bytes,
                    &mut events,
                    &mut omitted,
                );
            }
            pending.clear();
            discarding_line = false;
            start = end + 1;
        }
        if start < count && !discarding_line {
            append_line_bytes(
                &mut pending,
                &buffer[start..count],
                &mut discarding_line,
                &mut omitted,
                parse_events,
            );
        }
    }
    if !pending.is_empty() && !discarding_line {
        capture_event(
            &pending,
            parse_events,
            max_events,
            max_event_bytes,
            &mut event_bytes,
            &mut events,
            &mut omitted,
        );
    }
    if omitted {
        events.push(json!({
            "event": "diagnostics_truncated",
            "message": "additional or oversized slicer diagnostic events were omitted"
        }));
    }
    Ok(StreamCapture { events, tail })
}

fn append_line_bytes(
    pending: &mut Vec<u8>,
    bytes: &[u8],
    discarding_line: &mut bool,
    omitted: &mut bool,
    parse_events: bool,
) {
    if pending.len().saturating_add(bytes.len()) <= MAX_OUTPUT_LINE_BYTES {
        pending.extend_from_slice(bytes);
    } else {
        pending.clear();
        *discarding_line = true;
        *omitted |= parse_events;
    }
}

fn capture_event(
    line: &[u8],
    parse_events: bool,
    max_events: usize,
    max_event_bytes: usize,
    event_bytes: &mut usize,
    events: &mut Vec<Value>,
    omitted: &mut bool,
) {
    if !parse_events {
        return;
    }
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    let Some(payload) = line.strip_prefix(EVENT_PREFIX.as_bytes()) else {
        return;
    };
    if events.len() >= max_events || event_bytes.saturating_add(payload.len()) > max_event_bytes {
        *omitted = true;
        return;
    }
    match serde_json::from_slice::<Value>(payload) {
        Ok(event) => {
            *event_bytes += payload.len();
            events.push(event);
        }
        Err(error) => events.push(json!({
            "event": "diagnostic_parse_error",
            "message": error.to_string()
        })),
    }
}

fn append_tail(target: &mut String, value: &str, max_bytes: usize) {
    target.push_str(value);
    if target.len() <= max_bytes {
        return;
    }
    let mut start = target.len() - max_bytes;
    while !target.is_char_boundary(start) {
        start += 1;
    }
    target.drain(..start);
}

fn fatal_diagnostic(event: &Value) -> bool {
    matches!(
        event.get("event").and_then(Value::as_str),
        Some(
            "config_load_failed"
                | "preset_resolution_failed"
                | "override_rejected"
                | "config_value_rejected"
                | "load_error"
                | "input_error"
                | "preset_error"
                | "slicing_error"
        )
    )
}

fn first_nonempty<'a>(first: &'a str, second: &'a str) -> Option<&'a str> {
    [first, second]
        .into_iter()
        .map(str::trim)
        .find(|value| !value.is_empty())
}

fn failed_result(
    config: &WorkerConfig,
    identity: &EngineIdentity,
    request: &SliceRequest,
    started: Instant,
    error: FailureDetail,
    diagnostics: Vec<Value>,
) -> SliceResult {
    failed_result_with_duration(
        config,
        identity,
        request,
        elapsed_millis(started),
        error,
        diagnostics,
    )
}

fn failed_result_with_duration(
    config: &WorkerConfig,
    identity: &EngineIdentity,
    request: &SliceRequest,
    duration_ms: u64,
    error: FailureDetail,
    diagnostics: Vec<Value>,
) -> SliceResult {
    SliceResult {
        schema_version: SCHEMA_VERSION,
        job_id: request.job_id.clone(),
        status: SliceStatus::Failed,
        engine: request.engine,
        duration_ms,
        engine_version: identity.version.clone(),
        engine_git_sha: identity.git_sha.clone(),
        deployment_version: config.deployment_version.clone(),
        output: None,
        diagnostics,
        error: Some(error),
    }
}

fn elapsed_millis(started: Instant) -> u64 {
    u64::try_from(started.elapsed().as_millis()).unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use url::Url;

    use super::*;
    use crate::model::{Engine, InputObject, OutputObject, SliceOptions, SliceOverrides};

    fn test_config(root: PathBuf) -> WorkerConfig {
        WorkerConfig {
            input_bucket: "input".into(),
            output_bucket: "output".into(),
            input_key_prefix: "jobs/".into(),
            output_key_prefix: "jobs/".into(),
            slicer_root: root,
            callback_url: Url::parse("https://api.example.test/callback").unwrap(),
            callback_hmac_secret: vec![b'x'; 32],
            callback_attempts: 3,
            callback_timeout: std::time::Duration::from_secs(1),
            slice_timeout: std::time::Duration::from_secs(10),
            max_input_bytes: 1024,
            max_config_bytes: 1024,
            max_output_bytes: 1024,
            max_3mf_entries: 10,
            max_3mf_expanded_bytes: 1024,
            max_3mf_compression_ratio: 20,
            max_diagnostic_events: 10,
            max_diagnostic_bytes: 1024,
            deployment_version: None,
        }
    }

    fn request(profiles: Profiles) -> SliceRequest {
        SliceRequest {
            schema_version: SCHEMA_VERSION,
            job_id: "job-1".into(),
            input: InputObject {
                bucket: "input".into(),
                key: "jobs/job-1/model.stl".into(),
                version_id: None,
                sha256: "a".repeat(64),
            },
            output: OutputObject {
                bucket: "output".into(),
                key: "jobs/job-1/output.gcode".into(),
            },
            engine: Engine::Bambu,
            profiles,
            options: SliceOptions::default(),
        }
    }

    #[test]
    fn stl_requires_explicit_profiles() {
        let config = test_config(PathBuf::from("/unused"));
        let failure = validate_request(&config, &request(Profiles::Embedded)).unwrap_err();
        assert_eq!(failure.code, "ProfilesRequired");
    }

    #[test]
    fn rejects_unconfigured_buckets() {
        let config = test_config(PathBuf::from("/unused"));
        let mut request = request(Profiles::Embedded);
        request.input.bucket = "another-input".into();
        let failure = validate_request(&config, &request).unwrap_err();
        assert_eq!(failure.code, "InvalidBucket");
    }

    #[test]
    fn rejects_output_outside_configured_prefix() {
        let config = test_config(PathBuf::from("/unused"));
        let mut request = request(Profiles::Embedded);
        request.input.key = "jobs/job-1/model.3mf".into();
        request.output.key = "other/output.gcode".into();
        let failure = validate_request(&config, &request).unwrap_err();
        assert_eq!(failure.code, "InvalidObjectKey");
    }

    #[test]
    fn rejects_cross_job_object_keys() {
        let config = test_config(PathBuf::from("/unused"));
        let mut request = request(Profiles::Embedded);
        request.input.key = "jobs/another-job/model.3mf".into();
        let failure = validate_request(&config, &request).unwrap_err();
        assert_eq!(failure.code, "CrossJobObjectKey");
    }

    #[test]
    fn rejects_non_normalized_object_keys() {
        let config = test_config(PathBuf::from("/unused"));
        let mut request = request(Profiles::Embedded);
        request.input.key = "jobs/job-1/../model.3mf".into();
        let failure = validate_request(&config, &request).unwrap_err();
        assert_eq!(failure.code, "InvalidObjectKey");
    }

    #[test]
    fn arguments_use_packaged_profiles_without_a_shell() {
        let root = tempfile::tempdir().unwrap();
        for path in ["machine.json", "filament.json", "process.json"] {
            fs::write(root.path().join(path), "{}").unwrap();
        }
        fs::create_dir_all(root.path().join("bin")).unwrap();
        let config = test_config(root.path().to_owned());
        let request = request(Profiles::Explicit {
            machine: "machine.json".into(),
            filament: "filament.json".into(),
            process: "process.json".into(),
            config: None,
        });
        let arguments = build_arguments(
            &config,
            &request,
            Path::new("/tmp/input.stl"),
            Path::new("/tmp/output.gcode"),
            None,
        )
        .unwrap();
        let arguments: Vec<_> = arguments
            .iter()
            .map(|value| value.to_string_lossy().into_owned())
            .collect();
        assert_eq!(arguments[0], "/tmp/input.stl");
        assert!(
            arguments
                .windows(2)
                .any(|pair| pair[0] == "--machine" && pair[1].ends_with("machine.json"))
        );
        assert_eq!(
            &arguments[arguments.len() - 2..],
            ["--output", "/tmp/output.gcode"]
        );
    }

    #[test]
    fn rejects_profile_path_traversal() {
        let root = tempfile::tempdir().unwrap();
        let failure = resolve_packaged_file(root.path(), "../secret.json").unwrap_err();
        assert_eq!(failure.code, "InvalidProfilePath");
    }

    #[tokio::test]
    async fn captures_structured_events_and_bounds_tail() {
        let input = format!(
            "plain\n{EVENT_PREFIX}{{\"event\":\"mesh_repaired\"}}\n{}",
            "x".repeat(FAILURE_LOG_BYTES + 100)
        );
        let captured = capture_stream(input.as_bytes(), true, 10, 1024)
            .await
            .unwrap();
        assert_eq!(captured.events[0]["event"], "mesh_repaired");
        assert!(captured.tail.len() <= FAILURE_LOG_BYTES);
    }

    #[test]
    fn validates_override_ranges() {
        let config = test_config(PathBuf::from("/unused"));
        let mut request = request(Profiles::Embedded);
        request.input.key = "jobs/job-1/model.3mf".into();
        request.options.overrides = SliceOverrides {
            infill_percent: Some(101),
            ..Default::default()
        };
        // u8 accepts 101, but the API contract does not.
        let failure = validate_request(&config, &request).unwrap_err();
        assert_eq!(failure.code, "InvalidInfill");
    }
}
