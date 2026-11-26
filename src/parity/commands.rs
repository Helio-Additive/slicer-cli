//! The `slice` and `compare` commands of `slicer-parity`.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use tempfile::{NamedTempFile, TempDir};

use super::{
    bambu::{default_bambu_binary, run_bambu_slice},
    config::JobConfig,
    locations::{materialize_input, prepare_output, write_json_to_location, PreparedOutput},
};

/// Which slicing engine to drive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Engine {
    /// The C++ Bambu Studio engine: `slicer_cli`, run as a subprocess.
    Bambu,
    /// The Rust port (`crates/libslic3r-rs`), run in-process.
    Rust,
}

pub struct SliceArgs {
    pub config: String,
    pub bambu_binary: Option<PathBuf>,
    pub verbose: bool,
    pub dry_run: bool,
    pub engine: Engine,
}

pub struct CompareArgs {
    pub config: String,
    pub bambu_binary: Option<PathBuf>,
    pub verbose: bool,
}

/// A job whose slicer config has been resolved to a temp file. Shared by the
/// `slice` and `compare` commands.
struct ResolvedJob {
    input_kind: String,
    input_path: PathBuf,
    output: PreparedOutput,
    bambu_binary: PathBuf,
    /// Resolved flat settings bundle (STL inputs only); held alive so its
    /// on-disk path stays valid for the duration of slicing.
    temp_config: Option<NamedTempFile>,
    _temp_dirs: Vec<TempDir>,
}

impl ResolvedJob {
    /// Path to the resolved settings bundle, if one was produced (STL inputs).
    fn config_path(&self) -> Option<&Path> {
        self.temp_config.as_ref().map(NamedTempFile::path)
    }
}

/// Load the job config, find the input model, prepare the output location,
/// resolve the `slicer_cli` binary, and (for STL inputs) resolve the settings
/// into a temp file. Both engines are then given that same file.
fn resolve_job(config: &str, bambu_binary: Option<PathBuf>) -> Result<ResolvedJob, String> {
    let job = JobConfig::load_arg(config)?;
    let mut temp_dirs = Vec::new();
    let input_location = job.input.location()?;
    let input_kind = job.input.kind(&input_location)?;
    let input_path = materialize_input(&input_location, "input", &mut temp_dirs)?;
    let output = prepare_output(&job.output.location()?)?;

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
        input_kind,
        input_path,
        output,
        bambu_binary,
        temp_config,
        _temp_dirs: temp_dirs,
    })
}

/// Slice with the Rust engine. An STL takes the resolved settings bundle; a
/// Bambu Studio 3MF carries its own, which a resolved bundle overrides.
fn run_rust_slice(resolved: &ResolvedJob, output: &Path) -> Result<(), String> {
    match resolved.input_kind.as_str() {
        "stl" => {
            let config_path = resolved
                .config_path()
                .ok_or_else(|| "rust engine requires a resolved slicer config".to_owned())?;
            slicer::app_slice::slice_to_gcode(&resolved.input_path, config_path, output)
        }
        _ => slicer::app_slice::slice_3mf_to_gcode(
            &resolved.input_path,
            resolved.config_path(),
            output,
        ),
    }
    .map_err(|e| format!("rust engine slice failed: {e:#}"))
}

pub fn slice(args: SliceArgs) -> Result<u8, String> {
    let resolved = resolve_job(&args.config, args.bambu_binary.clone())?;

    match args.engine {
        Engine::Bambu => run_bambu_slice(
            &resolved.bambu_binary,
            &resolved.input_path,
            resolved.config_path(),
            &resolved.output.local_path,
            args.verbose,
            args.dry_run,
        ),
        Engine::Rust => {
            if args.dry_run {
                println!(
                    "rust-engine slice {} --output {}",
                    resolved.input_path.display(),
                    resolved.output.requested
                );
                return Ok(0);
            }
            if let Some(parent) = resolved.output.local_path.parent() {
                std::fs::create_dir_all(parent)
                    .map_err(|e| format!("create {}: {e}", parent.display()))?;
            }
            run_rust_slice(&resolved, &resolved.output.local_path)?;
            Ok(0)
        }
    }
}

/// Slice the same job with both engines (C++ subprocess, Rust in-process) and
/// print a diff of the two G-code outputs. Exit status 0 when they are
/// byte-identical, 1 when they differ.
pub fn compare(args: CompareArgs) -> Result<u8, String> {
    let resolved = resolve_job(&args.config, args.bambu_binary.clone())?;

    let tmp = TempDir::new().map_err(|e| format!("create temp dir for compare: {e}"))?;
    let bambu_out = tmp.path().join("bambu.gcode");
    let rust_out = tmp.path().join("rust.gcode");

    // C++ engine (subprocess). Wall-clock timed for slice-time parity (R387).
    // The subprocess time includes process spawn/teardown, which the in-process
    // Rust path does not pay; treat the ratio as an order-of-magnitude guide, not
    // a microbenchmark. Set COMPARE_TIMING_RUNS=N to take the min over N Rust runs.
    let bambu_start = std::time::Instant::now();
    let bambu_code = run_bambu_slice(
        &resolved.bambu_binary,
        &resolved.input_path,
        resolved.config_path(),
        &bambu_out,
        args.verbose,
        false,
    )?;
    let bambu_secs = bambu_start.elapsed().as_secs_f64();
    if bambu_code != 0 {
        return Err(format!("bambu (C++) engine exited with code {bambu_code}"));
    }

    // Rust engine (in-process). Optionally re-run to report the best-of-N time,
    // which suppresses first-run cold-cache noise when comparing against C++.
    let timing_runs: u32 = std::env::var("COMPARE_TIMING_RUNS")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|&n| n >= 1)
        .unwrap_or(1);
    let mut rust_secs = f64::INFINITY;
    for _ in 0..timing_runs {
        let rust_start = std::time::Instant::now();
        run_rust_slice(&resolved, &rust_out)?;
        rust_secs = rust_secs.min(rust_start.elapsed().as_secs_f64());
    }

    let bambu_gcode = std::fs::read(&bambu_out)
        .map_err(|e| format!("read bambu gcode {}: {e}", bambu_out.display()))?;
    let rust_gcode = std::fs::read(&rust_out)
        .map_err(|e| format!("read rust gcode {}: {e}", rust_out.display()))?;

    if let Ok(keep) = std::env::var("COMPARE_KEEP_DIR") {
        let keep = Path::new(&keep);
        std::fs::create_dir_all(keep).map_err(|e| format!("create {}: {e}", keep.display()))?;
        std::fs::write(keep.join("bambu.gcode"), &bambu_gcode)
            .and_then(|()| std::fs::write(keep.join("rust.gcode"), &rust_gcode))
            .map_err(|e| format!("write G-code into {}: {e}", keep.display()))?;
    }

    let report = diff_gcode(&bambu_gcode, &rust_gcode);
    print!("{report}");
    print!("{}", timing_report(bambu_secs, rust_secs, timing_runs));

    if bambu_gcode == rust_gcode {
        Ok(0)
    } else {
        Ok(1)
    }
}

/// Format the slice-time parity section: per-engine wall-clock and the
/// rust/bambu ratio. Complements the physical-equivalence diff — a toolpath can
/// be shape-correct yet far slower, which the user tracks separately (R387).
fn timing_report(bambu_secs: f64, rust_secs: f64, rust_runs: u32) -> String {
    use std::fmt::Write as _;
    let mut out = String::new();
    let _ = writeln!(out, "--- slice time (wall-clock) ---");
    let _ = writeln!(
        out,
        "bambu (C++)     {bambu_secs:.3}s  (subprocess, incl. spawn)"
    );
    let runs_note = if rust_runs > 1 {
        format!("  (best of {rust_runs})")
    } else {
        String::new()
    };
    let _ = writeln!(out, "rust            {rust_secs:.3}s{runs_note}");
    if bambu_secs > 0.0 && rust_secs.is_finite() {
        let ratio = rust_secs / bambu_secs;
        let verdict = if ratio <= 1.10 {
            "at parity (<=1.10x)"
        } else if ratio <= 2.0 {
            "slower"
        } else {
            "much slower"
        };
        let _ = writeln!(out, "ratio rust/bambu {ratio:.2}x  ({verdict})");
    }
    out
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
    let mut all_tags: Vec<&String> = bambu_features.keys().chain(rust_features.keys()).collect();
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
