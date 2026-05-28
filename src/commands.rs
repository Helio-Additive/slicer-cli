use reqwest::Method;
use serde_json::{json, Value};
use tempfile::NamedTempFile;

use crate::{
    cli::{PresetsArgs, SliceArgs},
    config::JobConfig,
    json_utils::{optional_string, string_field, write_json},
    locations::{materialize_input, prepare_output, upload_output, write_json_to_location},
    native::{default_native_binary, run_native_slice},
    profiles::resolve_config_refs,
};

pub fn slice(args: SliceArgs) -> Result<u8, String> {
    let job = JobConfig::load_arg(&args.config)?;
    let mut temp_dirs = Vec::new();
    let input_location = job.input.location()?;
    let input_kind = job.input.kind(&input_location)?;
    let input_path = materialize_input(&input_location, "input", &mut temp_dirs)?;
    let output_location = job.output.location()?;
    let output = prepare_output(&output_location, "output.gcode", &mut temp_dirs)?;

    let native_binary = args
        .native_binary
        .or_else(|| job.native_binary.clone())
        .unwrap_or_else(default_native_binary);

    let mut temp_config = None;
    let resolved_config_location = job.output.resolved_config_location()?;

    if input_kind == "stl" {
        let resolved = job
            .input_config(&mut temp_dirs)?
            .ok_or_else(|| "STL input requires input.config".to_owned())?;
        let mut file =
            NamedTempFile::new().map_err(|e| format!("create temp slicer config: {e}"))?;
        serde_json::to_writer_pretty(&mut file, &resolved)
            .map_err(|e| format!("write temp slicer config: {e}"))?;
        file.as_file_mut()
            .sync_all()
            .map_err(|e| format!("flush temp slicer config: {e}"))?;

        if let Some(location) = &resolved_config_location {
            write_json_to_location(location, &resolved)?;
        }
        temp_config = Some(file);
    } else if input_kind != "3mf" {
        return Err(format!("unsupported input type: {input_kind}"));
    }

    let code = run_native_slice(
        &native_binary,
        &input_path,
        temp_config.as_ref().map(NamedTempFile::path),
        &output.local_path,
        args.verbose,
        args.dry_run,
    )?;

    if code == 0 && output.upload_uri.is_some() && !args.dry_run {
        upload_output(&output)?;
    }

    if let Some(callback) = job.output.callback.as_ref().or(job.callback.as_ref()) {
        let payload = json!({
            "status": if code == 0 { "succeeded" } else { "failed" },
            "exit_code": code,
            "input": {
                "type": input_kind,
                "location": input_location,
            },
            "output": {
                "location": output.requested,
            },
            "resolved_config": resolved_config_location,
        });
        send_callback(callback, &payload, args.dry_run)?;
    }

    Ok(code)
}

pub fn presets(args: PresetsArgs) -> Result<u8, String> {
    let config = resolve_config_refs(
        &args.machine,
        &args.filament,
        &args.process,
        &args.profile_root,
    )?;
    write_json(&args.output, &Value::Object(config))?;
    Ok(0)
}

fn send_callback(callback: &Value, payload: &Value, dry_run: bool) -> Result<(), String> {
    if callback.is_null() || callback == &Value::Bool(false) {
        return Ok(());
    }

    let (url, method, headers) = match callback {
        Value::String(url) => (url.clone(), "POST".to_owned(), Vec::new()),
        Value::Object(obj) => {
            let url = string_field(obj, "url")?;
            let method = optional_string(obj, "method").unwrap_or_else(|| "POST".to_owned());
            let headers = obj
                .get("headers")
                .and_then(Value::as_object)
                .map(|headers| {
                    headers
                        .iter()
                        .filter_map(|(key, value)| {
                            value.as_str().map(|value| (key.clone(), value.to_owned()))
                        })
                        .collect::<Vec<_>>()
                })
                .unwrap_or_default();
            (url, method, headers)
        }
        _ => return Err("callback must be null, false, URL string, or object".to_owned()),
    };

    let method = Method::from_bytes(method.as_bytes())
        .map_err(|e| format!("invalid callback method {method}: {e}"))?;

    if dry_run {
        println!("callback {} {}", method.as_str(), url);
        return Ok(());
    }

    let client = reqwest::blocking::Client::new();
    let mut request = client.request(method, &url).json(payload);
    for (key, value) in headers {
        request = request.header(&key, value);
    }
    request
        .send()
        .and_then(reqwest::blocking::Response::error_for_status)
        .map(|_| ())
        .map_err(|e| format!("callback request failed: {e}"))
}

#[cfg(test)]
mod tests {
    #[test]
    fn infers_stl_input() {
        assert_eq!(
            crate::config::infer_kind_from_location("part.STL").unwrap(),
            "stl"
        );
    }

    #[test]
    fn infers_3mf_input() {
        assert_eq!(
            crate::config::infer_kind_from_location("job.3mf").unwrap(),
            "3mf"
        );
    }
}
