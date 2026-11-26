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
    pub geometry: GeometryOptions,
}

/// cli_command_line.cpp 496-563 keeps official settings and raw legacy
/// overrides separately. main.cpp 10676-10681 applies the latter last.
#[derive(Default, Debug, Eq, PartialEq)]
pub struct GeometryOptions {
    pub extra_config: std::collections::BTreeMap<String, String>,
    pub overrides: std::collections::BTreeMap<String, String>,
}

pub(crate) fn geometry_options_enabled() -> bool {
    native_tokens() && slicer::faithful_gate("CLI_GEOMETRY_OPTIONS")
}

pub(crate) fn nozzle_diameter_enabled() -> bool {
    native_tokens() && slicer::faithful_gate("CLI_NOZZLE_DIAMETER")
}

pub(crate) fn temperature_options_enabled() -> bool {
    native_tokens() && slicer::faithful_gate("CLI_TEMPERATURE_OPTIONS")
}

pub fn refusal(flag: &str) -> String {
    let flags = if native_tokens() {
        "--config, -o/--output, -v/--verbose, --input and --single-instance"
    } else {
        "--config, -o/--output, -v/--verbose and --input"
    };
    let extra = if slicer::faithful_gate("CLI_LEGACY_GCODE") { ", plus --no-normalize-legacy-gcode" } else { "" };
    let geometry = if geometry_options_enabled() { ", --layer-height, --wall-loops, --sparse-infill-density, --perimeters and --infill" } else { "" };
    let diameter = if nozzle_diameter_enabled() { ", --nozzle-diameter" } else { "" };
    let temperatures = if temperature_options_enabled() { ", --temp and --bed-temp" } else { "" };
    let timing = if slicer::faithful_gate("CLI_GCODE_TIMING") { ", and --gcode-timing" } else { "" };
    format!(
        "{flag} is not a flag of {PROGRAM}: the Rust engine takes only {flags}{geometry}{diameter}{temperatures}{extra}{timing} so far. \
         Use slicer_cli (the Bambu Studio engine) for {flag}."
    )
}

/// Parse the supported single-model calls. cli_command_line.cpp 465–571, 620,
/// 751–752 and 781–782. A parse refusal before slicing exits 1 (main.cpp
/// 13580–13617). Unsupported Rust flags retain their named refusal.
pub fn parse(args: &[String]) -> Result<Request, (String, i32)> {
    use slicer::config::{ConfigBase, ConfigOptionResolver};
    let mut official = slicer::print_config::config_def::DynamicPrintConfig::new();
    let mut geometry = GeometryOptions::default();
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
                "layer-height" | "wall-loops" | "sparse-infill-density" if geometry_options_enabled() => {
                    let (key, kind) = match flag {
                        "layer-height" => ("layer_height", "a number"),
                        "wall-loops" => ("wall_loops", "a whole number"),
                        _ => ("sparse_infill_density", "a percentage such as 15%"),
                    };
                    let word = value()?;
                    let text = word.to_str().expect("arguments are UTF-8 strings");
                    official.set_deserialize_strict(key, text, false).map_err(|_| (
                        format!("Error: --{flag}: '{text}' is not a value BambuStudio takes for this flag; it takes {kind}"), 1,
                    ))?;
                    // Keep the typed value through slicing. ConfigOption::serialize
                    // deliberately rounds floats to six significant digits for output.
                    let decoded = match key {
                        "wall_loops" => official.opt_int(key).to_string(),
                        _ => {
                            let value = official.option(key).and_then(|opt| opt.get_float().ok())
                                .ok_or_else(|| (format!("Error: missing numeric setting {key}"), 1))?;
                            if key == "layer_height" { value.to_string() } else { format!("{value}%") }
                        }
                    };
                    geometry.extra_config.insert(key.into(), decoded);
                }
                "nozzle-diameter" if nozzle_diameter_enabled() => {
                    // cli_command_line.cpp 227–252: clear defaults on the first
                    // occurrence, then append typed rows on each subsequent one.
                    use slicer::config::ConfigOptionFloatsNullable;
                    let key = "nozzle_diameter";
                    let word = value()?;
                    let text = word.to_str().expect("arguments are UTF-8 strings");
                    let append = official.has(key);
                    official.set_deserialize_strict(key, text, append).map_err(|_| (
                        format!("Error: --{flag}: '{text}' is not a value BambuStudio takes for this flag; it takes a numbers, comma separated"), 1,
                    ))?;
                    let rows = official.option(key).and_then(|opt| opt.as_any().downcast_ref::<ConfigOptionFloatsNullable>())
                        .expect("PrintConfig.cpp 5223–5230 defines a nullable float vector");
                    // The profile loader needs each double before the config
                    // block rounds it to six significant digits for display.
                    let mut values = Vec::with_capacity(rows.values.len());
                    for &diameter in &rows.values {
                        if diameter.is_infinite() {
                            return Err((format!("Error: --{flag}: Serializing invalid number"), 1));
                        }
                        values.push(if diameter.is_nan() { "nil".into() } else { diameter.to_string() });
                    }
                    geometry.extra_config.insert(key.into(), values.join(","));
                }
                "infill" | "perimeters" if geometry_options_enabled() => {
                    let key = if flag == "infill" { "fill_density" } else { "perimeters" };
                    geometry.overrides.insert(key.into(), value()?.to_string_lossy().into_owned());
                }
                "temp" | "bed-temp" if temperature_options_enabled() => {
                    let key = if flag == "temp" { "nozzle_temperature" } else { "bed_temperature" };
                    geometry.overrides.insert(key.into(), value()?.to_string_lossy().into_owned());
                }
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
    // cli_command_line.cpp 705-716 validates the final values after repeats.
    if !geometry.extra_config.is_empty() {
        let findings = official.validate(true).map_err(|e| (e.to_string(), 1))?;
        if !findings.is_empty() {
            let message = findings.iter().map(|(key, why)| format!("{key}: {why}")).collect::<Vec<_>>().join("; ");
            return Err((format!("Error: Params in command line error: {message}"), 1));
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
        geometry,
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
