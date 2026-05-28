use serde_json::Value;
use std::{
    fs,
    path::{Path, PathBuf},
};
use tempfile::{NamedTempFile, TempDir};

use crate::json_utils::{string_value, write_json};

pub struct PreparedOutput {
    pub requested: String,
    pub local_path: PathBuf,
    pub upload_uri: Option<String>,
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

pub fn materialize_input(
    location: &str,
    label: &str,
    temp_dirs: &mut Vec<TempDir>,
) -> Result<PathBuf, String> {
    if !is_remote_location(location) {
        return Ok(PathBuf::from(location));
    }

    let dir = TempDir::new().map_err(|e| format!("create temp dir for {label}: {e}"))?;
    let filename = location_filename(location).unwrap_or(label);
    let path = dir.path().join(filename);
    if is_s3_location(location) {
        download_s3(location, &path, label)?;
    } else {
        download_http(location, &path, label)?;
    }
    temp_dirs.push(dir);
    Ok(path)
}

pub fn prepare_output(
    location: &str,
    default_filename: &str,
    temp_dirs: &mut Vec<TempDir>,
) -> Result<PreparedOutput, String> {
    if !is_s3_location(location) {
        return Ok(PreparedOutput {
            requested: location.to_owned(),
            local_path: PathBuf::from(location),
            upload_uri: None,
        });
    }

    let dir = TempDir::new().map_err(|e| format!("create temp dir for output: {e}"))?;
    let filename = s3_filename(location).unwrap_or(default_filename);
    let local_path = dir.path().join(filename);
    temp_dirs.push(dir);
    Ok(PreparedOutput {
        requested: location.to_owned(),
        local_path,
        upload_uri: Some(location.to_owned()),
    })
}

pub fn upload_output(output: &PreparedOutput) -> Result<(), String> {
    if let Some(uri) = &output.upload_uri {
        upload_s3(&output.local_path, uri, "output")?;
    }
    Ok(())
}

pub fn write_json_to_location(location: &str, value: &Value) -> Result<(), String> {
    if !is_s3_location(location) {
        return write_json(Path::new(location), value);
    }

    let file = NamedTempFile::new().map_err(|e| format!("create temp file for {location}: {e}"))?;
    write_json(file.path(), value)?;
    upload_s3(file.path(), location, "resolved_config")
}

pub fn is_s3_location(location: &str) -> bool {
    location.starts_with("s3://")
}

fn is_http_location(location: &str) -> bool {
    location.starts_with("http://") || location.starts_with("https://")
}

fn is_remote_location(location: &str) -> bool {
    is_s3_location(location) || is_http_location(location)
}

fn download_http(uri: &str, destination: &Path, label: &str) -> Result<(), String> {
    let mut response = reqwest::blocking::get(uri)
        .and_then(reqwest::blocking::Response::error_for_status)
        .map_err(|e| format!("download {label} from {uri}: {e}"))?;
    let mut file = fs::File::create(destination)
        .map_err(|e| format!("create {}: {e}", destination.display()))?;
    response.copy_to(&mut file).map_err(|e| {
        format!(
            "write HTTP response for {label} to {}: {e}",
            destination.display()
        )
    })?;
    Ok(())
}

fn download_s3(uri: &str, destination: &Path, label: &str) -> Result<(), String> {
    let (bucket, key) = parse_s3_uri(uri)?;
    let destination = destination.to_path_buf();
    aws_runtime()?.block_on(async move {
        let config = aws_config::load_defaults(aws_config::BehaviorVersion::latest()).await;
        let client = aws_sdk_s3::Client::new(&config);
        let object = client
            .get_object()
            .bucket(bucket)
            .key(key)
            .send()
            .await
            .map_err(|e| format!("download {label} from {uri}: {e}"))?;
        let bytes = object
            .body
            .collect()
            .await
            .map_err(|e| format!("read s3 response for {label}: {e}"))?
            .into_bytes();
        fs::write(&destination, bytes).map_err(|e| format!("write {}: {e}", destination.display()))
    })
}

fn upload_s3(source: &Path, uri: &str, label: &str) -> Result<(), String> {
    let (bucket, key) = parse_s3_uri(uri)?;
    let source = source.to_path_buf();
    aws_runtime()?.block_on(async move {
        let config = aws_config::load_defaults(aws_config::BehaviorVersion::latest()).await;
        let client = aws_sdk_s3::Client::new(&config);
        let body = aws_sdk_s3::primitives::ByteStream::from_path(&source)
            .await
            .map_err(|e| format!("read {} for s3 upload: {e}", source.display()))?;
        client
            .put_object()
            .bucket(bucket)
            .key(key)
            .body(body)
            .send()
            .await
            .map_err(|e| format!("upload {label} to {uri}: {e}"))?;
        Ok(())
    })
}

fn aws_runtime() -> Result<tokio::runtime::Runtime, String> {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .map_err(|e| format!("create async runtime for s3: {e}"))
}

fn parse_s3_uri(uri: &str) -> Result<(String, String), String> {
    let path = uri
        .strip_prefix("s3://")
        .ok_or_else(|| format!("not an s3 URI: {uri}"))?;
    let (bucket, key) = path
        .split_once('/')
        .ok_or_else(|| format!("s3 URI requires bucket and key: {uri}"))?;
    if bucket.is_empty() || key.is_empty() {
        return Err(format!("s3 URI requires bucket and key: {uri}"));
    }
    Ok((bucket.to_owned(), key.to_owned()))
}

fn s3_filename(location: &str) -> Option<&str> {
    location
        .rsplit('/')
        .next()
        .filter(|name| !name.is_empty() && *name != "s3:")
}

fn location_filename(location: &str) -> Option<&str> {
    let without_query = location.split_once('?').map_or(location, |(path, _)| path);
    without_query
        .rsplit('/')
        .next()
        .filter(|name| !name.is_empty() && *name != "s3:")
}
