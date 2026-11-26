//! `slicer_cli-rust`: file in, G-code out, through the Rust engine.
//!
//! The flags it takes are the ones of `slicer_cli`'s default call that the
//! Rust engine can honour today (`cli_command_line.cpp`, `legacy_flags()`):
//! `--config`, `-o`/`--output`, `-v`/`--verbose`, `--input` and `--single-instance`. Every other flag
//! is refused by name, before anything loads, with a sentence that names the
//! program to use instead (the convention of `cli_flag_support.cpp`).
//!
//! Return codes are the official ones `slicer_cli` returns
//! (libslic3r `Utils.hpp`, `CLI_*`), as the process exit status; a job
//! `Print::validate()` refuses exits 1, as `slicer_cli` does (main.cpp:11635-11636).

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
            slicer::app_slice::slice_to_gcode(&request.input, config, &request.output)
                .map_err(failed)
        }
        "3mf" => slicer::app_slice::slice_3mf_to_gcode(
            &request.input,
            request.config.as_deref(),
            &request.output,
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
            print!("{USAGE}{additional}{USAGE_TAIL}");
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
    fn default_call_parses() {
        let request = parse(&args(&["--config", "c.json", "-o", "out.gcode", "-v", "m.stl"]));
        assert_eq!(
            request,
            Ok(Request::Slice(SliceRequest {
                input: "m.stl".into(),
                config: Some("c.json".into()),
                output: "out.gcode".into(),
                verbose: true,
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
        let (sentence, _) = parse(&args(&["no-such.stl", "--gcode-timing"])).unwrap_err();
        assert!(sentence.starts_with("--gcode-timing"), "{sentence}");
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
