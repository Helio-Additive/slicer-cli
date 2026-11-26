//! `slicer_cli-rust`: file in, G-code out, through the Rust engine.
//!
//! The flags it takes are the ones of `slicer_cli`'s default call that the
//! Rust engine can honour today (`cli_command_line.cpp`, `legacy_flags()`):
//! `--config`, `-o`/`--output`, `-v`/`--verbose`, `--input`, `--single-instance` and
//! `--no-normalize-legacy-gcode`. The separate `--gcode-timing` mode reads a pair of existing files. Every other flag
//! is refused by name, before anything loads, with a sentence that names the
//! program to use instead (the convention of `cli_flag_support.cpp`).
//!
//! Return codes are the official ones `slicer_cli` returns
//! (libslic3r `Utils.hpp`, `CLI_*`), as the process exit status; a job
//! `Print::validate()` refuses exits 1, as `slicer_cli` does (main.cpp:11635-11636).

mod printer;

pub use crate::cli_command_line::{parse, refusal, Request, SliceRequest};
use std::path::Path;

/// `CLI_SUCCESS` (Utils.hpp).
pub const CLI_SUCCESS: i32 = 0;
/// `CLI_INVALID_PARAMS` (Utils.hpp).
pub const CLI_INVALID_PARAMS: i32 = -2;
/// `CLI_FILE_NOTFOUND` (Utils.hpp).
pub const CLI_FILE_NOTFOUND: i32 = -3;
/// `CLI_SLICING_ERROR` (Utils.hpp).
pub const CLI_SLICING_ERROR: i32 = -100;

/// The exit status a `CLI_*` code becomes, as it does when `slicer_cli`
/// returns the code from `main`.
pub fn exit_status(code: i32) -> u8 {
    code as u8
}

pub(crate) const PROGRAM: &str = "slicer_cli-rust";

const USAGE: &str = "\
slicer_cli-rust: the Rust slicing engine, an in-progress port of the Bambu Studio engine.
Its G-code does not match slicer_cli's yet; use slicer_cli for prints.

Usage: slicer_cli-rust [--config FILE] [-o FILE] [-v] MODEL

  MODEL                 An STL, or a Bambu Studio 3MF with its settings inside
      --input FILE      Same as MODEL
      --config FILE     Settings bundle (JSON). Required for an STL; for a 3MF it
                        overrides the settings in the file
  -o, --output FILE     Output G-code file (default: output.gcode)
  -v, --verbose         Verbose engine log
";

const USAGE_TAIL: &str = "  -h, --help            This text

Every other flag of slicer_cli is refused by name. slicer_cli (Bambu Studio engine),
slicer_cli-orcaslicer (OrcaSlicer engine) and slicer_cli-snapmaker (Snapmaker Orca
engine) take the full command line.
";

fn extension(path: &Path) -> String {
    path.extension()
        .map(|e| e.to_string_lossy().to_ascii_lowercase())
        .unwrap_or_default()
}

/// The sentence and code an engine error becomes. A hard validation error is refused as slicer_cli
/// refuses it (main.cpp:11539-11636): its sentence with the tip, and the CLI_* code of its type
/// (app_slice.rs `ValidationRefusal`); anything else is a slicing error.
fn engine_failure<E>(e: E) -> (String, i32)
where
    E: AsRef<dyn std::error::Error + Send + Sync + 'static> + std::fmt::Display,
{
    let mut source: Option<&(dyn std::error::Error + 'static)> = Some(e.as_ref());
    while let Some(err) = source {
        if let Some(refusal) = err.downcast_ref::<slicer::app_slice::ValidationRefusal>() {
            // main.cpp:11536 emit_validation_event, on stdout, then the sentence on stderr and exit 1.
            use std::io::Write;
            let mut out = std::io::stdout();
            let _ = out.write_all(refusal.event_record().as_bytes());
            let _ = out.flush();
            return (refusal.stderr_text(), refusal.exit_code());
        }
        if let Some(refusal) = err.downcast_ref::<slicer::app_slice::ManualMapRefusal>() {
            return (format!("Error: {refusal}"), 1);
        }
        if let Some(refusal) = err.downcast_ref::<slicer::app_slice::NozzleVariantRefusal>() {
            let event = serde_json::json!({"event": "config_refused", "tag": "NozzleFlagNotAPrinterVariant",
                "message": refusal.message, "nozzle": refusal.given, "printer_variant": refusal.printer_variant,
                "printer_preset": refusal.printer_preset});
            println!("\n[[SLICER_EVENT]] {event}");
            return (format!("Error: {}", refusal.message), 1);
        }
        if let Some(refusal) = err.downcast_ref::<slicer::app_slice::ConfigRefusal>() {
            // main.cpp:9583-9590: the config_refused event on stdout, the sentence on stderr, exit 1.
            use std::io::Write;
            let mut out = std::io::stdout();
            let _ = out.write_all(refusal.event_record().as_bytes());
            let _ = out.flush();
            return (refusal.stderr_text(), refusal.exit_code());
        }
        source = err.source();
    }
    (format!("The Rust engine could not slice: {e:#}"), CLI_SLICING_ERROR)
}

/// main.cpp 597–608 and diagnostic_output.hpp 14–18.
fn legacy_alias_record(keys: &[&str], count: usize) -> String {
    use slicer::app_slice::{BOUND_EXTRUDER_TOKEN as to, LEGACY_FILAMENT_TOKEN as from};
    let message = format!(
        "Aliased unbound placeholder '{from}' -> '{to}' in {} custom-gcode key(s); {count} occurrence(s) rewritten",
        keys.len()
    );
    let event = serde_json::json!({
        "event": "config_normalized", "tag": "LegacyGcodeTokenAliased",
        "message": message, "from": from, "to": to, "keys": keys, "count": count,
    });
    format!("\n[[SLICER_EVENT]] {event}\n")
}

fn report_legacy_alias(keys: &[&str], count: usize) {
    use std::io::Write;
    let mut out = std::io::stdout().lock();
    let _ = out.write_all(legacy_alias_record(keys, count).as_bytes());
    let _ = out.flush();
}

fn slice(request: &SliceRequest) -> Result<(), (String, i32)> {
    if !request.input.is_file() {
        return Err((
            format!("The model file {} was not found.", request.input.display()),
            CLI_FILE_NOTFOUND,
        ));
    }
    if let Some(config) = &request.config {
        if !config.is_file() {
            return Err((
                format!("The settings file {} was not found.", config.display()),
                CLI_FILE_NOTFOUND,
            ));
        }
    }
    if let Some(parent) = request.output.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|e| {
                (
                    format!("Could not create {}: {e}", parent.display()),
                    CLI_INVALID_PARAMS,
                )
            })?;
        }
    }

    use slicer::app_slice::LegacyGcode;
    let legacy_gcode = if !request.normalize_legacy_gcode {
        LegacyGcode::Preserve
    } else if slicer::faithful_gate("CLI_LEGACY_GCODE") {
        LegacyGcode::NormalizeAndReport(&report_legacy_alias)
    } else {
        LegacyGcode::Normalize
    };
    // main.cpp 7237-7328. The legacy wall, infill and thermal overrides use std::stoi: a decimal
    // integer prefix is sufficient. Report rejections only when slicing
    // reaches the application step, after the config has been validated.
    let mut values = request.geometry.extra_config.clone();
    let mut rejected = Vec::new();
    let mut fill_density = None;
    let nozzle_rejected = std::cell::Cell::new(false);
    let reject_nozzle = || nozzle_rejected.set(true);
    // main.cpp 9554–9568 applies the file's bundled parent before the
    // physical check. A flat settings file does not require a catalogue.
    let file_inherits = slicer::faithful_gate("CLI_FILE_INHERITS");
    let load_parent = |path: &std::path::Path, doc: &serde_json::Value| {
        let name = doc.get("inherits")?.as_str()?;
        if name.is_empty() { return None; }
        if file_inherits && doc.get("from").is_some_and(|v| !v.is_string()) { return None; }
        let settings = crate::desktop_presets::inherited_settings(&printer::profiles_dir()?, path)?;
        if file_inherits && request.verbose {
            println!("Applied {name}, the preset {} inherits", path.display());
        }
        Some(settings)
    };
    let nozzle = request.geometry.overrides.get("nozzle_diameter").map(|raw| slicer::app_slice::NozzleOverride {
        given: raw.as_str(), inherited: None, check_variant: &printer::check_nozzle_variant, rejected: &reject_nozzle,
    });
    for (key, raw) in &request.geometry.overrides {
        if key == "nozzle_diameter" {
            // Checked on the resolved printer, after the official flags have landed.
            rejected.push((key, raw));
            continue;
        }
        match slicer::preset_bundle::std_stoi(raw) {
            Ok(value) if key == "fill_density" => {
                values.insert("sparse_infill_density".into(), format!("{value}%"));
                fill_density = Some(value);
            }
            Ok(value) if key == "perimeters" => { values.insert("wall_loops".into(), value.to_string()); }
            Ok(value) => { values.insert(key.clone(), value.to_string()); }
            Err(_) => rejected.push((key, raw)),
        }
    }
    let report = || {
        for &(key, value) in &rejected {
            if key == "nozzle_diameter" && !nozzle_rejected.get() { continue; }
            let message = format!("Command-line override for '{key}' was rejected and had no effect");
            let event = serde_json::json!({"event": "override_rejected", "message": message,
                "opt_key": key, "tag": "InvalidOverrideValue", "value": value});
            println!("[[SLICER_EVENT]] {event}");
            eprintln!("Warning: Invalid value for {key}: {value}");
        }
    };
    let overrides = slicer::app_slice::SliceOverrides { values: &values, fill_density, report_rejections: &report };
    let overrides = (!values.is_empty() || !rejected.is_empty()).then_some(&overrides);
    let job = slicer::app_slice::CliJobSettings {
        overrides,
        nozzle: nozzle.as_ref(),
        load_parent: (file_inherits || nozzle.is_some()).then_some(&load_parent),
    };
    let failed = engine_failure;
    match extension(&request.input).as_str() {
        "stl" => {
            let config = request.config.as_deref().ok_or_else(|| {
                (
                    format!(
                        "An STL carries no settings, and {PROGRAM} does not load presets: give \
                         --config FILE (a settings bundle, JSON), or slice it with slicer_cli."
                    ),
                    CLI_INVALID_PARAMS,
                )
            })?;
            slicer::app_slice::slice_to_gcode_with_job_settings(&request.input, config, &request.output, legacy_gcode, job)
                .map_err(failed)
        }
        "3mf" => slicer::app_slice::slice_3mf_to_gcode_with_job_settings(
            &request.input,
            request.config.as_deref(),
            &request.output,
            legacy_gcode,
            job,
        )
        .map_err(failed),
        other => Err((
            format!(
                "{PROGRAM} reads STL and 3MF files; {} is a .{other} file. Use slicer_cli for it.",
                request.input.display()
            ),
            CLI_INVALID_PARAMS,
        )),
    }
}

/// Run the program; the return value is the process exit status.
pub fn run(args: &[String]) -> u8 {
    // main.cpp 13486-13498. The timing mode returns before logging and slicing.
    if slicer::faithful_gate("CLI_GCODE_TIMING") && crate::cli_command_line::gcode_timing_requested(args) {
        return crate::cli_gcode_timing::run(args);
    }
    let request = match parse(args) {
        Ok(request) => request,
        Err((sentence, code)) => {
            eprintln!("{sentence}");
            return exit_status(code);
        }
    };
    let request = match request {
        Request::Help => {
            let additional = if crate::cli_command_line::native_tokens() {
                "      --single-instance Accepted for desktop compatibility\n      --                Treat following words as model paths\n"
            } else {
                ""
            };
            let legacy = if slicer::faithful_gate("CLI_LEGACY_GCODE") {
                "      --no-normalize-legacy-gcode  Keep legacy placeholders in custom G-code templates\n"
            } else {
                ""
            };
            let timing = if slicer::faithful_gate("CLI_GCODE_TIMING") {
                "      --gcode-timing ORIGINAL OPTIMIZED  Report both G-code print times as JSON\n"
            } else {
                ""
            };
            let geometry = if crate::cli_command_line::geometry_options_enabled() {
                "      --layer-height MM  Set the layer height\n      --wall-loops N, --perimeters N  Set the wall count\n      --sparse-infill-density PERCENT, --infill N  Set sparse infill density\n"
            } else { "" };
            let diameter = if crate::cli_command_line::nozzle_diameter_enabled() {
                "      --nozzle-diameter MM[,MM...]  Set the per-extruder nozzle diameters\n"
            } else { "" };
            let nozzle = if crate::cli_command_line::nozzle_override_enabled() {
                "      --nozzle MM        Check the printer variant and set the nozzle diameter\n"
            } else { "" };
            let thermal = if crate::cli_command_line::temperature_options_enabled() {
                "      --temp C           Set the nozzle temperature after the first layer\n      --bed-temp C       Set the selected plate temperature for both phases\n"
            } else { "" };
            print!("{USAGE}{additional}{geometry}{diameter}{nozzle}{thermal}{legacy}{timing}{USAGE_TAIL}");
            return exit_status(CLI_SUCCESS);
        }
        Request::Slice(request) => request,
    };
    if request.verbose {
        let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
            .try_init();
    }
    match slice(&request) {
        Ok(()) => exit_status(CLI_SUCCESS),
        Err((sentence, code)) => {
            eprintln!("{sentence}");
            exit_status(code)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| (*s).to_owned()).collect()
    }

    #[test]
    fn legacy_gcode_normalization_matches_native_records() {
        use slicer::app_slice::LegacyGcode;
        use std::cell::Cell;
        use std::fmt::Write;
        let mut actual = String::new();
        for line in include_str!("../tests/rust-engine/data/legacy_gcode/inputs.jsonl").lines() {
            let mut input: serde_json::Value = serde_json::from_str(line).unwrap();
            let mut settings = input["settings"].take();
            let event = Cell::new(None);
            let report = |keys: &[&str], count| event.set(Some(legacy_alias_record(keys, count)));
            let policy = if input["normalize"] == false {
                LegacyGcode::Preserve
            } else if input["report"] == true {
                LegacyGcode::NormalizeAndReport(&report)
            } else {
                LegacyGcode::Normalize
            };
            let count = policy.apply(&mut settings);
            if let Some(record) = event.take() {
                actual.push_str(&record);
            }
            writeln!(actual, "{}", serde_json::json!({"count": count, "settings": settings})).unwrap();
        }
        assert_eq!(actual, include_str!("../tests/rust-engine/data/legacy_gcode/native.txt"));
    }

    #[test]
    fn default_call_parses() {
        let request = parse(&args(&["--config", "c.json", "-o", "out.gcode", "-v", "m.stl"]));
        assert_eq!(
            request,
            Ok(Request::Slice(SliceRequest {
                input: "m.stl".into(),
                config: Some("c.json".into()),
                output: "out.gcode".into(),
                verbose: true,
                normalize_legacy_gcode: true,
                geometry: Default::default(),
            }))
        );
    }

    #[test]
    fn output_defaults_as_in_slicer_cli() {
        let Ok(Request::Slice(request)) = parse(&args(&["--input=m.3mf"])) else {
            panic!("expected a slice request");
        };
        assert_eq!(request.output, PathBuf::from("output.gcode"));
        assert_eq!(request.input, PathBuf::from("m.3mf"));
    }

    #[test]
    fn a_flag_it_does_not_take_is_refused_by_name() {
        for flag in ["--slice", "--arrange", "--load-settings", "--simplify", "--plate"] {
            let (sentence, code) = parse(&args(&[flag, "1", "m.3mf"])).unwrap_err();
            assert_eq!(code, CLI_INVALID_PARAMS);
            assert!(sentence.starts_with(flag), "{sentence}");
            assert!(sentence.contains("Use slicer_cli "), "{sentence}");
        }
    }

    #[test]
    fn refused_before_anything_loads() {
        // The model file does not exist; the flag is still what is reported.
        let (sentence, _) = parse(&args(&["no-such.stl", "--orient"])).unwrap_err();
        assert!(sentence.starts_with("--orient"), "{sentence}");
    }

    #[test]
    fn a_validation_error_is_refused_as_slicer_cli_refuses_it() {
        use slicer::print_base::{StringExceptionType, StringObjectException};
        let refusal = slicer::app_slice::ValidationRefusal {
            exception: StringObjectException {
                string: "Layer height cannot exceed nozzle diameter".to_string(),
                ..Default::default()
            },
        };
        // main.cpp:11541-11543
        assert_eq!(
            refusal.stderr_text(),
            "Validation error: Layer height cannot exceed nozzle diameter\n\nTip: You may need to specify proper \
             config files:\n  --machine, --filament, and --process options"
        );
        // slicer_cli exits 1 (main.cpp:11635-11636) and writes CLI_VALIDATE_ERROR (Utils.hpp:52) to result.json.
        assert_eq!(exit_status(refusal.exit_code()), 1);
        assert_eq!(refusal.return_code(), -51);
        // main.cpp:465-484, as slicer_cli printed it for layer_height 0.5 on a 0.4 nozzle.
        let mut lh = refusal.clone();
        lh.exception.opt_key = "layer_height".to_string();
        assert_eq!(
            lh.event_record(),
            "\n[[SLICER_EVENT]] {\"event\":\"validation_error\",\"message\":\"Layer height cannot exceed nozzle diameter\",\
             \"opt_key\":\"layer_height\",\"tag\":\"STRING_EXCEPT_NOT_DEFINED\"}\n"
        );
        let mut seq = refusal.clone();
        seq.exception.r#type = StringExceptionType::StringExceptObjectCollisionInSeqPrint;
        assert_eq!(seq.return_code(), -63);
    }

    #[test]
    fn a_config_refusal_is_refused_as_slicer_cli_refuses_it() {
        // main.cpp:9554-9590, as slicer_cli printed it for a spiral-vase job with two walls.
        let mut settings = std::collections::BTreeMap::new();
        settings.insert("wall_loops".to_string(), "Invalid value when spiral vase mode is enabled: 2".to_string());
        let refusal = slicer::app_slice::ConfigRefusal::new(settings);
        let sentence = "BambuStudio cannot print this file as it is: wall_loops: Invalid value when spiral vase mode is \
                        enabled: 2. Give --wall-loops a value in range to override it. --allow-substitution lets the \
                        engine use its own default for a value it does not know instead.";
        assert_eq!(refusal.sentence, sentence);
        assert_eq!(refusal.stderr_text(), format!("Param values in 3mf/config error: {sentence}"));
        assert_eq!(
            refusal.event_record(),
            format!(
                "\n[[SLICER_EVENT]] {{\"event\":\"config_refused\",\"message\":\"{sentence}\",\"settings\":{{\"wall_loops\":\
                 \"Invalid value when spiral vase mode is enabled: 2\"}},\"tag\":\"InvalidValues\",\"unknown_values\":[]}}\n"
            )
        );
        assert_eq!(exit_status(refusal.exit_code()), 1);
        // CLI_INVALID_VALUES_IN_3MF (Utils.hpp)
        assert_eq!(refusal.return_code(), -18);
    }

    #[test]
    fn codes_become_the_exit_status_slicer_cli_gives() {
        assert_eq!(exit_status(CLI_INVALID_PARAMS), 254);
        assert_eq!(exit_status(CLI_FILE_NOTFOUND), 253);
        assert_eq!(exit_status(CLI_SLICING_ERROR), 156);
    }
}
