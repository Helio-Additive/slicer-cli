use reqwest::Method;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use tempfile::{NamedTempFile, TempDir};

use crate::{
    cli::{
        CompareArgs, CompatibleProcessesArgs, Engine, PresetsArgs, ProfilesArgs, ProfilesCommand,
        SliceArgs,
    },
    config::JobConfig,
    json_utils::{optional_string, string_field, write_json},
    locations::{
        materialize_input, prepare_output, upload_output, write_json_to_location, PreparedOutput,
    },
    bambu::{default_bambu_binary, run_bambu_slice},
    profiles::{compatible_processes_for_printer, list_profiles, resolve_config_refs},
};

/// A job whose input has been materialized and whose slicer config has been
/// resolved to a temp file. Shared by both the `slice` and `compare` commands.
struct ResolvedJob {
    job: JobConfig,
    input_location: String,
    input_kind: String,
    input_path: PathBuf,
    output: PreparedOutput,
    bambu_binary: PathBuf,
    /// Resolved BambuStudio-format slicer config (STL inputs only); held alive so
    /// its on-disk path stays valid for the duration of slicing.
    temp_config: Option<NamedTempFile>,
    resolved_config_location: Option<String>,
    /// Temp dirs (downloads, etc.) kept alive for the duration of the job.
    _temp_dirs: Vec<TempDir>,
}

impl ResolvedJob {
    /// Path to the resolved slicer config, if one was produced (STL inputs).
    fn config_path(&self) -> Option<&Path> {
        self.temp_config.as_ref().map(NamedTempFile::path)
    }
}

/// Load the job config, materialize the input model, prepare the output location,
/// resolve the bambu binary, and (for STL inputs) resolve the slicer config into
/// a temp file. This is the shared front half of `slice` and `compare`.
fn resolve_job(
    config: &str,
    bambu_binary: Option<PathBuf>,
    output_default_filename: &str,
) -> Result<ResolvedJob, String> {
    let job = JobConfig::load_arg(config)?;
    let mut temp_dirs = Vec::new();
    let input_location = job.input.location()?;
    let input_kind = job.input.kind(&input_location)?;
    let input_path = materialize_input(&input_location, "input", &mut temp_dirs)?;
    let output_location = job.output.location()?;
    let output = prepare_output(&output_location, output_default_filename, &mut temp_dirs)?;

    let bambu_binary = bambu_binary
        .or_else(|| job.bambu_binary.clone())
        .unwrap_or_else(default_bambu_binary);

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

    Ok(ResolvedJob {
        job,
        input_location,
        input_kind,
        input_path,
        output,
        bambu_binary,
        temp_config,
        resolved_config_location,
        _temp_dirs: temp_dirs,
    })
}

pub fn slice(args: SliceArgs) -> Result<u8, String> {
    let resolved = resolve_job(&args.config, args.bambu_binary.clone(), "output.gcode")?;

    let code = match args.engine {
        Engine::Bambu => run_bambu_slice(
            &resolved.bambu_binary,
            &resolved.input_path,
            resolved.config_path(),
            &resolved.output.local_path,
            args.verbose,
            args.dry_run,
        )?,
        Engine::Rust => match resolved.input_kind.as_str() {
            "stl" => {
                let config_path = resolved
                    .config_path()
                    .ok_or_else(|| "rust engine requires a resolved slicer config".to_owned())?;
                if args.dry_run {
                    println!(
                        "rust-engine slice {} --settings {} --output {}",
                        resolved.input_path.display(),
                        config_path.display(),
                        resolved.output.local_path.display()
                    );
                    0
                } else {
                    slicer::app_slice::slice_to_gcode(
                        &resolved.input_path,
                        config_path,
                        &resolved.output.local_path,
                    )
                    .map_err(|e| format!("rust engine slice failed: {e:#}"))?;
                    0
                }
            }
            "3mf" => {
                // A BambuStudio 3MF carries its own embedded
                // Metadata/project_settings.config, so no resolved profile triple
                // is required. If the job did supply a resolved config, it
                // overrides the embedded settings.
                let settings_override = resolved.config_path();
                if args.dry_run {
                    println!(
                        "rust-engine slice {} (3mf, embedded settings) --output {}",
                        resolved.input_path.display(),
                        resolved.output.local_path.display()
                    );
                    0
                } else {
                    slicer::app_slice::slice_3mf_to_gcode(
                        &resolved.input_path,
                        settings_override,
                        &resolved.output.local_path,
                    )
                    .map_err(|e| format!("rust engine slice failed: {e:#}"))?;
                    0
                }
            }
            other => {
                return Err(format!(
                    "the rust engine supports STL and 3MF input (got {other}); use --engine bambu"
                ));
            }
        },
    };

    if code == 0 && resolved.output.upload_uri.is_some() && !args.dry_run {
        upload_output(&resolved.output)?;
    }

    if let Some(callback) = resolved
        .job
        .output
        .callback
        .as_ref()
        .or(resolved.job.callback.as_ref())
    {
        let payload = json!({
            "status": if code == 0 { "succeeded" } else { "failed" },
            "exit_code": code,
            "input": {
                "type": resolved.input_kind,
                "location": resolved.input_location,
            },
            "output": {
                "location": resolved.output.requested,
            },
            "resolved_config": resolved.resolved_config_location,
        });
        send_callback(callback, &payload, args.dry_run)?;
    }

    Ok(code)
}

/// Slice the same job with both engines (bambu (C++) subprocess + in-process Rust)
/// and print a diff of the two G-code outputs.
pub fn compare(args: CompareArgs) -> Result<u8, String> {
    let resolved = resolve_job(&args.config, args.bambu_binary.clone(), "compare.gcode")?;

    if resolved.input_kind != "stl" {
        return Err(format!(
            "compare only supports STL input (got {}); the rust engine is STL-only",
            resolved.input_kind
        ));
    }
    let config_path = resolved
        .config_path()
        .ok_or_else(|| "compare requires a resolved slicer config".to_owned())?
        .to_path_buf();

    let tmp = TempDir::new().map_err(|e| format!("create temp dir for compare: {e}"))?;
    let bambu_out = tmp.path().join("bambu.gcode");
    let rust_out = tmp.path().join("rust.gcode");

    // C++ engine (subprocess).
    let bambu_code = run_bambu_slice(
        &resolved.bambu_binary,
        &resolved.input_path,
        Some(config_path.as_path()),
        &bambu_out,
        args.verbose,
        false,
    )?;
    if bambu_code != 0 {
        return Err(format!("bambu (C++) engine exited with code {bambu_code}"));
    }

    // Rust engine (in-process).
    slicer::app_slice::slice_to_gcode(&resolved.input_path, &config_path, &rust_out)
        .map_err(|e| format!("rust engine slice failed: {e:#}"))?;

    let bambu_gcode = std::fs::read(&bambu_out)
        .map_err(|e| format!("read bambu gcode {}: {e}", bambu_out.display()))?;
    let rust_gcode = std::fs::read(&rust_out)
        .map_err(|e| format!("read rust gcode {}: {e}", rust_out.display()))?;

    if let Ok(keep) = std::env::var("COMPARE_KEEP_DIR") {
        let _ = std::fs::create_dir_all(&keep);
        let _ = std::fs::write(std::path::Path::new(&keep).join("bambu.gcode"), &bambu_gcode);
        let _ = std::fs::write(std::path::Path::new(&keep).join("rust.gcode"), &rust_gcode);
    }

    let report = diff_gcode(&bambu_gcode, &rust_gcode);
    print!("{report}");

    if bambu_gcode == rust_gcode {
        Ok(0)
    } else {
        Ok(1)
    }
}

/// Build the comparison report string for two G-code byte buffers.
fn diff_gcode(bambu: &[u8], rust: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    use std::fmt::Write as _;

    let bambu_text = String::from_utf8_lossy(bambu);
    let rust_text = String::from_utf8_lossy(rust);
    let bambu_lines: Vec<&str> = bambu_text.lines().collect();
    let rust_lines: Vec<&str> = rust_text.lines().collect();

    let bambu_sha = hex(&Sha256::digest(bambu));
    let rust_sha = hex(&Sha256::digest(rust));
    let identical = bambu == rust;

    let mut out = String::new();
    let _ = writeln!(out, "=== G-code comparison: bambu (C++) vs rust ===");
    let _ = writeln!(
        out,
        "total lines     bambu={}  rust={}",
        bambu_lines.len(),
        rust_lines.len()
    );
    let _ = writeln!(out, "sha256 bambu   {bambu_sha}");
    let _ = writeln!(out, "sha256 rust     {rust_sha}");
    let _ = writeln!(
        out,
        "byte-identical  {}",
        if identical { "YES" } else { "NO" }
    );

    // First differing line + count of differing lines.
    let max_lines = bambu_lines.len().max(rust_lines.len());
    let mut first_diff: Option<usize> = None;
    let mut diff_count = 0usize;
    for i in 0..max_lines {
        let n = bambu_lines.get(i).copied();
        let r = rust_lines.get(i).copied();
        if n != r {
            if first_diff.is_none() {
                first_diff = Some(i);
            }
            diff_count += 1;
        }
    }
    let _ = writeln!(out, "differing lines {diff_count}");
    match first_diff {
        Some(i) => {
            let _ = writeln!(out, "first divergence at line {}", i + 1);
            let _ = writeln!(
                out,
                "  bambu: {}",
                bambu_lines.get(i).copied().unwrap_or("<EOF>")
            );
            let _ = writeln!(
                out,
                "  rust:   {}",
                rust_lines.get(i).copied().unwrap_or("<EOF>")
            );
        }
        None => {
            let _ = writeln!(out, "first divergence none (line sequences match)");
        }
    }

    // Parsed summary diff.
    let _ = writeln!(out, "--- parsed summary ---");
    summary_field(&mut out, "total filament length", &bambu_lines, &rust_lines);
    summary_field(&mut out, "total layer number", &bambu_lines, &rust_lines);

    let bambu_features = feature_counts(&bambu_lines);
    let rust_features = feature_counts(&rust_lines);
    let _ = writeln!(out, "FEATURE tag counts (bambu vs rust):");
    let mut all_tags: Vec<&String> =
        bambu_features.keys().chain(rust_features.keys()).collect();
    all_tags.sort();
    all_tags.dedup();
    if all_tags.is_empty() {
        let _ = writeln!(out, "  (no ; FEATURE: tags found)");
    }
    for tag in all_tags {
        let n = bambu_features.get(tag).copied().unwrap_or(0);
        let r = rust_features.get(tag).copied().unwrap_or(0);
        let flag = if n == r { "" } else { "  <-- differs" };
        let _ = writeln!(out, "  {tag:<24} bambu={n:<6} rust={r}{flag}");
    }

    out
}

/// Print the first matching `; <label>` value from each side.
fn summary_field(out: &mut String, label: &str, bambu: &[&str], rust: &[&str]) {
    use std::fmt::Write as _;
    let needle = format!("; {label}");
    let find = |lines: &[&str]| -> Option<String> {
        lines
            .iter()
            .find(|l| l.trim_start().starts_with(&needle))
            .map(|l| l.trim().to_string())
    };
    let n = find(bambu);
    let r = find(rust);
    let _ = writeln!(out, "{label}:");
    let _ = writeln!(out, "  bambu: {}", n.as_deref().unwrap_or("<absent>"));
    let _ = writeln!(out, "  rust:   {}", r.as_deref().unwrap_or("<absent>"));
}

/// Count `; FEATURE: <name>` tags by name.
fn feature_counts(lines: &[&str]) -> BTreeMap<String, usize> {
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    for l in lines {
        let t = l.trim_start();
        if let Some(rest) = t.strip_prefix("; FEATURE:") {
            *counts.entry(rest.trim().to_string()).or_insert(0) += 1;
        }
    }
    counts
}

/// Lowercase hex encoding of a byte slice.
fn hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s
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

pub fn profiles(args: ProfilesArgs) -> Result<u8, String> {
    match args.command {
        ProfilesCommand::List(args) => {
            let profiles = list_profiles(args.kind, &args.profile_root)?;
            serde_json::to_writer_pretty(std::io::stdout(), &profiles)
                .map_err(|e| format!("write profiles JSON: {e}"))?;
            println!();
            Ok(0)
        }
        ProfilesCommand::CompatibleProcesses(args) => compatible_processes(args),
    }
}

fn compatible_processes(args: CompatibleProcessesArgs) -> Result<u8, String> {
    let report = compatible_processes_for_printer(&args.printer, &args.profile_root)?;
    serde_json::to_writer_pretty(std::io::stdout(), &report)
        .map_err(|e| format!("write compatible processes JSON: {e}"))?;
    println!();
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
