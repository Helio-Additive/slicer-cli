//! The supported single-model calls through Bambu's actual parse_command_line.
use serde::Deserialize;
use slicer_cli_rust::engine_cli::{parse, Request};
use std::path::Path;

#[derive(Deserialize)]
struct Case {
    args: Vec<String>,
    help: Option<bool>,
    error: Option<String>,
    input: Option<String>,
    output: Option<String>,
    config: Option<String>,
    verbose: Option<bool>,
}

#[test]
fn matches_native_option_tokens() {
    for row in include_str!("rust-engine/data/cli_command_line/tokens.jsonl").lines() {
        let case: Case = serde_json::from_str(row).unwrap();
        let result = parse(&case.args);
        if let Some(error) = case.error {
            assert_eq!(result, Err((format!("Error: {error}"), 1)), "{row}");
        } else if case.help == Some(true) {
            assert_eq!(result, Ok(Request::Help), "{row}");
        } else {
            let Ok(Request::Slice(slice)) = result else {
                panic!("{row}: {result:?}");
            };
            assert_eq!(
                Some(slice.input.as_path()),
                case.input.as_deref().map(Path::new),
                "{row}"
            );
            assert_eq!(
                Some(slice.output.as_path()),
                case.output.as_deref().map(Path::new),
                "{row}"
            );
            let config = case
                .config
                .as_deref()
                .filter(|s| !s.is_empty())
                .map(Path::new);
            assert_eq!(slice.config.as_deref(), config, "{row}");
            assert_eq!(Some(slice.verbose), case.verbose, "{row}");
        }
    }
}

#[test]
fn process_serial_number_follows_the_host_parser() {
    // cli_command_line.cpp 473–476 skips this argument in Apple builds.
    let result = parse(&["-psn_0_12345".into(), "model.stl".into()]);
    if cfg!(target_vendor = "apple") {
        let Ok(Request::Slice(slice)) = result else {
            panic!("{result:?}")
        };
        assert_eq!(slice.input, Path::new("model.stl"));
    } else {
        assert!(result.unwrap_err().0.starts_with("-psn_0_12345"));
    }
}

#[test]
fn slice_dash_prefixed_model_and_replay_old_refusal() {
    use std::{fs, process::Command};
    let dir = tempfile::tempdir().unwrap();
    let name = "-calib.3mf";
    fs::copy(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/calib_base.3mf"),
        dir.path().join(name),
    )
    .unwrap();
    let run = |words: &[&str], gate: &str| {
        Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
            .current_dir(dir.path())
            .env("CLI_OPTION_TOKENS", gate)
            .args(words)
            .output()
            .unwrap()
    };
    let before = run(&["--input", name, "-o", "base.gcode"], "0");
    assert!(
        before.status.success(),
        "{}",
        String::from_utf8_lossy(&before.stderr)
    );
    let words = ["--single-instance", "-output=alias.gcode", "--", name];
    let after = run(&words, "1");
    assert!(
        after.status.success(),
        "{}",
        String::from_utf8_lossy(&after.stderr)
    );
    let a = fs::read(dir.path().join("base.gcode")).unwrap();
    let b = fs::read(dir.path().join("alias.gcode")).unwrap();
    assert!(!a.is_empty() && a == b);
    let disabled = run(&words, "0");
    assert_eq!(disabled.status.code(), Some(254));
    assert!(disabled
        .stderr
        .starts_with(b"--single-instance is not a flag"));
}

#[test]
fn geometry_options_match_native_parse_state() {
    use std::collections::BTreeMap;
    #[derive(Deserialize)]
    struct GeometryCase {
        args: Vec<String>,
        error: Option<String>,
        extra_config: Option<BTreeMap<String, String>>,
        overrides: Option<BTreeMap<String, String>>,
    }
    for row in include_str!("rust-engine/data/cli_command_line/geometry-options.jsonl").lines() {
        let case: GeometryCase = serde_json::from_str(row).unwrap();
        // Physical nozzle and temperature settings still have a named refusal.
        // Their native vectors also exercise shared option tokenization.
        if let Some(flag) = case.args.iter().filter_map(|arg| arg.split('=').next())
            .find(|flag| matches!(*flag, "--nozzle" | "--temp" | "--bed-temp")) {
            assert!(parse(&case.args).unwrap_err().0.starts_with(flag), "{row}");
            continue;
        }
        let result = parse(&case.args);
        if let Some(error) = case.error {
            assert_eq!(result, Err((format!("Error: {error}"), 1)), "{row}");
            continue;
        }
        let Ok(Request::Slice(slice)) = result else { panic!("{row}: {result:?}") };
        use slicer::config::ConfigBase;
        let mut typed = slicer::print_config::config_def::DynamicPrintConfig::new();
        let mut published = BTreeMap::new();
        for (key, value) in &slice.geometry.extra_config {
            typed.set_deserialize_strict(key, value, false).unwrap();
            published.insert(key.clone(), typed.opt_serialize(key).unwrap());
        }
        assert_eq!(published, case.extra_config.unwrap(), "{row}");
        assert_eq!(slice.geometry.overrides, case.overrides.unwrap(), "{row}");
    }
}

#[test]
fn geometry_settings_keep_full_precision_before_serialization() {
    let Ok(Request::Slice(slice)) = parse(&["--layer-height=0.123456789".into(),
        "--sparse-infill-density=27.123456789%".into(), "model.stl".into()]) else { panic!("native numeric options") };
    assert_eq!(slice.geometry.extra_config["layer_height"], "0.123456789");
    assert_eq!(slice.geometry.extra_config["sparse_infill_density"], "27.123456789%");
}
