//! The Bambu options used by the Rust front end, from cli_command_line.cpp.
use crate::engine_cli::{CLI_INVALID_PARAMS, PROGRAM};
use std::path::PathBuf;

pub(crate) fn native_tokens() -> bool {
    slicer::faithful_gate("CLI_OPTION_TOKENS")
}

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
    pub normalize_legacy_gcode: bool,
}

pub fn refusal(flag: &str) -> String {
    let flags = if native_tokens() {
        "--config, -o/--output, -v/--verbose, --input and --single-instance"
    } else {
        "--config, -o/--output, -v/--verbose and --input"
    };
    let extra = if slicer::faithful_gate("CLI_LEGACY_GCODE") { ", plus --no-normalize-legacy-gcode" } else { "" };
    let timing = if slicer::faithful_gate("CLI_GCODE_TIMING") { ", and --gcode-timing" } else { "" };
    format!(
        "{flag} is not a flag of {PROGRAM}: the Rust engine takes only {flags}{extra}{timing} so far. \
         Use slicer_cli (the Bambu Studio engine) for {flag}."
    )
}

/// Parse the supported single-model calls. cli_command_line.cpp 465–571, 620,
/// 751–752 and 781–782. A parse refusal before slicing exits 1 (main.cpp
/// 13580–13617). Unsupported Rust flags retain their named refusal.
pub fn parse(args: &[String]) -> Result<Request, (String, i32)> {
    let native = native_tokens();
    let mut inputs = Vec::new();
    let mut config = None;
    let mut output = None;
    let mut verbose = false;
    let mut normalize_legacy_gcode = true;
    let mut help = false;
    let mut parse_options = true;
    let multiple_models = || {
        (
            format!("{PROGRAM} slices one model file per call; more than one was given."),
            CLI_INVALID_PARAMS,
        )
    };

    let mut words = args.iter().map(String::as_str);
    while let Some(word) = words.next() {
        let (spelling, inline) = match word.split_once('=') {
            Some((flag, value)) => (flag, Some(value)),
            None => (word, None),
        };
        if !parse_options || !word.starts_with('-') || word == "-" || (!native && spelling == "-") {
            inputs.push(PathBuf::from(word));
        } else if native && word == "--" {
            parse_options = false;
        } else if native && cfg!(target_vendor = "apple") && word.starts_with("-psn_") {
            // Config.cpp's macOS process serial number (cli_command_line.cpp 473–476).
            continue;
        } else {
            if !native
                && !matches!(
                    spelling,
                    "-h" | "--help"
                        | "-v"
                        | "--verbose"
                        | "-o"
                        | "--output"
                        | "--config"
                        | "--input"
                        | "--no-normalize-legacy-gcode"
                )
            {
                return Err((refusal(spelling), CLI_INVALID_PARAMS));
            }
            let flag = spelling.strip_prefix("--").unwrap_or(&spelling[1..]);
            let mut value = || {
                inline
                    .or_else(|| words.next())
                    .map(PathBuf::from)
                    .ok_or_else(|| {
                        if native {
                            (format!("Error: --{flag} needs a value"), 1)
                        } else {
                            (
                                format!("{spelling} needs a file name after it."),
                                CLI_INVALID_PARAMS,
                            )
                        }
                    })
            };
            match flag {
                "help" | "h" => {
                    if !native {
                        return Ok(Request::Help);
                    }
                    // store_official validates the bool. given_flag("help") selects
                    // the action even for an explicit zero (cli_command_line.cpp 781).
                    if let Some(value) = inline.filter(|v| !matches!(*v, "" | "0" | "1")) {
                        return Err((format!("Error: --{flag}: '{value}' is not a value BambuStudio takes for this flag; it takes 1 or 0"), 1));
                    }
                    help = true;
                }
                "v" | "verbose" => verbose = true,
                "config" => {
                    let path = value()?;
                    config = if native && path.as_os_str().is_empty() {
                        None
                    } else {
                        Some(path)
                    };
                }
                "o" | "output" => output = Some(value()?),
                "input" => inputs.push(value()?),
                "single-instance" if native => {}
                // cli_command_line.cpp 572, cli_options.hpp 29.
                "no-normalize-legacy-gcode" if slicer::faithful_gate("CLI_LEGACY_GCODE") => {
                    normalize_legacy_gcode = false;
                }
                _ => return Err((refusal(spelling), CLI_INVALID_PARAMS)),
            }
        }
        if !native && inputs.len() > 1 {
            return Err(multiple_models());
        }
    }
    if help {
        return Ok(Request::Help);
    }
    if inputs.len() > 1 {
        return Err(multiple_models());
    }
    let input = inputs.pop().ok_or_else(|| {
        (
            format!("No model file was given. Run {PROGRAM} --help for the call."),
            CLI_INVALID_PARAMS,
        )
    })?;
    Ok(Request::Slice(SliceRequest {
        input,
        config,
        output: output.unwrap_or_else(|| PathBuf::from("output.gcode")),
        verbose,
        normalize_legacy_gcode,
    }))
}

/// cli_command_line.cpp 75–126,137–155,388–422. A value-taking option
/// consumes the next word even when that word spells the timing flag.
pub(crate) fn gcode_timing_requested(args: &[String]) -> bool {
    fn split_option(word: &str) -> Option<(&str, bool)> {
        let option = word.strip_prefix("--").or_else(|| word.strip_prefix('-'))?;
        Some(option.split_once('=').map_or((option, false), |(name, _)| (name, true)))
    }
    if !args.iter().any(|word| matches!(split_option(word), Some(("gcode-timing", _)))) {
        return false;
    }
    let mut words = args.iter();
    while let Some(word) = words.next() {
        if word == "--" {
            break;
        }
        let Some((name, inline)) = split_option(word) else { continue };
        if name == "gcode-timing" {
            return true;
        }
        if !inline && timing_option_takes_value(name) {
            words.next();
        }
    }
    false
}

fn timing_option_takes_value(name: &str) -> bool {
    use slicer::config::ConfigOptionType::{CoBool, CoBools};
    use std::{collections::HashMap, sync::OnceLock};
    static PROFILE_OPTIONS: OnceLock<HashMap<String, bool>> = OnceLock::new();
    let profile = PROFILE_OPTIONS.get_or_init(|| {
        let mut names = HashMap::new();
        for (key, def) in &slicer::print_config::config_def::print_config_def().options {
            for alias in def.cli_args(key) {
                names.insert(alias, !matches!(def.r#type, CoBool | CoBools));
            }
        }
        names
    });
    if let Some(&takes_value) = profile.get(name) {
        return takes_value;
    }
    // CLIActionsConfigDef, CLITransformConfigDef and CLIMiscConfigDef
    // (BambuStudio PrintConfig.cpp 10935–11434), followed by legacy_flags().
    matches!(name,
        "export-3mf" | "export-slicedata" | "load-slicedata" | "export-stls" | "slice" | "export-png"
        | "mtcpp" | "mstpp" | "export-settings" | "pipe" | "arrange" | "repetitions" | "orient"
        | "rotate" | "rotate-x" | "rotate-y" | "scale" | "load-settings" | "load-filaments"
        | "skip-objects" | "clone-objects" | "uptodate-settings" | "uptodate-filaments"
        | "downward-settings" | "load-assemble-list" | "datadir" | "outputdir" | "debug"
        | "load-custom-gcodes" | "load-filament-ids" | "makerlab-name" | "makerlab-version"
        | "metadata-name" | "metadata-value" | "camera-view"
        | "o" | "output" | "machine" | "filament" | "process" | "config" | "layer-height" | "infill"
        | "perimeters" | "nozzle" | "temp" | "bed-temp" | "plate" | "input" | "calib-mode"
        | "calib-start" | "calib-end" | "calib-step" | "calib-extruder-id" | "layout"
        | "printer-preset" | "process-preset" | "filament-preset" | "printer" | "engine-info"
        | "simplify" | "simplify-ratio" | "simplify-triangles" | "simplify-objects")
}

#[cfg(test)]
mod timing_tests {
    use super::gcode_timing_requested;

    #[test]
    fn timing_scan_matches_each_native_option() {
        let vectors = include_str!("../tests/rust-engine/data/cli_command_line/timing_options.txt");
        for row in vectors.lines() {
            let (name, selected) = row.split_once(' ').unwrap();
            let args = [format!("--{name}"), "--gcode-timing".into()];
            assert_eq!(gcode_timing_requested(&args), selected == "1", "{row}");
        }
        assert!(vectors.lines().count() > 500);
    }
}
