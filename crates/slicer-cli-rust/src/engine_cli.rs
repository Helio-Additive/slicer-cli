//! `slicer_cli-rust`: file in, G-code out, through the Rust engine.
//!
//! The flags it takes are the ones of `slicer_cli`'s default call that the
//! Rust engine can honour today (`cli_command_line.cpp`, `legacy_flags()`):
//! `--config`, `-o`/`--output`, `-v`/`--verbose`, `--input`. Every other flag
//! is refused by name, before anything loads, with a sentence that names the
//! program to use instead (the convention of `cli_flag_support.cpp`).
//!
//! Return codes are the official ones `slicer_cli` returns
//! (libslic3r `Utils.hpp`, `CLI_*`), as the process exit status.

use std::path::{Path, PathBuf};

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

const PROGRAM: &str = "slicer_cli-rust";

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
  -h, --help            This text

Every other flag of slicer_cli is refused by name. slicer_cli (Bambu Studio engine),
slicer_cli-orcaslicer (OrcaSlicer engine) and slicer_cli-snapmaker (Snapmaker Orca
engine) take the full command line.
";

/// What the command line asked for.
#[derive(Debug, PartialEq, Eq)]
pub enum Request {
    Help,
    Slice(SliceRequest),
}

#[derive(Debug, PartialEq, Eq)]
pub struct SliceRequest {
    pub input: PathBuf,
    pub config: Option<PathBuf>,
    pub output: PathBuf,
    pub verbose: bool,
}

/// The sentence for a flag this program does not take.
pub fn refusal(flag: &str) -> String {
    format!(
        "{flag} is not a flag of {PROGRAM}: the Rust engine takes only --config, -o/--output, \
         -v/--verbose and --input so far. Use slicer_cli (the Bambu Studio engine) for {flag}."
    )
}

/// Parse the command line. `Err` carries the sentence to print and the
/// `CLI_*` code to return.
pub fn parse(args: &[String]) -> Result<Request, (String, i32)> {
    let invalid = |sentence: String| (sentence, CLI_INVALID_PARAMS);
    let mut input: Option<PathBuf> = None;
    let mut config: Option<PathBuf> = None;
    let mut output: Option<PathBuf> = None;
    let mut verbose = false;

    let mut i = 0;
    while i < args.len() {
        let arg = args[i].as_str();
        i += 1;
        // `--flag=value` and `--flag value` are both accepted, as in slicer_cli.
        let (name, inline_value) = match arg.split_once('=') {
            Some((name, value)) if name.starts_with('-') => (name, Some(value.to_owned())),
            _ => (arg, None),
        };
        let mut value_of = |name: &str| -> Result<PathBuf, (String, i32)> {
            if let Some(value) = &inline_value {
                return Ok(PathBuf::from(value));
            }
            let value = args
                .get(i)
                .ok_or_else(|| invalid(format!("{name} needs a file name after it.")))?;
            i += 1;
            Ok(PathBuf::from(value))
        };
        match name {
            "-h" | "--help" => return Ok(Request::Help),
            "-v" | "--verbose" => verbose = true,
            "--config" => config = Some(value_of(name)?),
            "-o" | "--output" => output = Some(value_of(name)?),
            "--input" => {
                let value = value_of(name)?;
                if input.replace(value).is_some() {
                    return Err(invalid(format!(
                        "{PROGRAM} slices one model file per call; more than one was given."
                    )));
                }
            }
            _ if name.starts_with('-') && name.len() > 1 => return Err(invalid(refusal(name))),
            _ => {
                if input.replace(PathBuf::from(arg)).is_some() {
                    return Err(invalid(format!(
                        "{PROGRAM} slices one model file per call; more than one was given."
                    )));
                }
            }
        }
    }

    let input = input.ok_or_else(|| {
        invalid(format!(
            "No model file was given. Run {PROGRAM} --help for the call."
        ))
    })?;
    Ok(Request::Slice(SliceRequest {
        input,
        config,
        output: output.unwrap_or_else(|| PathBuf::from("output.gcode")),
        verbose,
    }))
}

fn extension(path: &Path) -> String {
    path.extension()
        .map(|e| e.to_string_lossy().to_ascii_lowercase())
        .unwrap_or_default()
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

    let failed = |e| (format!("The Rust engine could not slice: {e:#}"), CLI_SLICING_ERROR);
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
            print!("{USAGE}");
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
    fn codes_become_the_exit_status_slicer_cli_gives() {
        assert_eq!(exit_status(CLI_INVALID_PARAMS), 254);
        assert_eq!(exit_status(CLI_FILE_NOTFOUND), 253);
        assert_eq!(exit_status(CLI_SLICING_ERROR), 156);
    }
}
