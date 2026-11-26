//! Root CLI geometry settings from cli_command_line.cpp 496–563 and main.cpp 7237–7327, 10676–10681.
use std::{
    fs,
    path::Path,
    process::{Command, Output},
};

fn run(model: &Path, path: &Path, flags: &[&str]) -> Output {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
        .args(["--config"])
        .arg(root.join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json"))
        .args(flags)
        .arg("-o")
        .arg(path)
        .arg(model)
        .output()
        .unwrap()
}

#[test]
fn geometry_settings_reach_stl_and_bambu_project() {
    let dir = tempfile::tempdir().unwrap();
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    for file in [
        "tests/rust-engine/fixtures/Cube_25.6.stl",
        "tests/fixtures/calib_base.3mf",
    ] {
        let path = dir.path().join("out.gcode");
        let result = run(
            &root.join(file),
            &path,
            &[
                "--layer-height",
                "0.25",
                "--infill",
                "27",
                "--perimeters",
                "3",
            ],
        );
        assert!(
            result.status.success(),
            "{file}: {}",
            String::from_utf8_lossy(&result.stderr)
        );
        let gcode = fs::read_to_string(path).unwrap();
        assert_eq!(
            gcode
                .lines()
                .filter(|s| s.starts_with("; Z_HEIGHT:"))
                .count(),
            103,
            "{file}"
        );
        for row in [
            "; layer_height = 0.25",
            "; sparse_infill_density = 27%",
            "; fill_density = 27%",
            "; wall_loops = 3",
        ] {
            assert!(gcode.lines().any(|l| l == row), "{file}, missing {row}");
        }
    }
}

#[test]
fn official_value_is_validated_before_model_loading() {
    let dir = tempfile::tempdir().unwrap();
    let result = run(
        &dir.path().join("absent.stl"),
        &dir.path().join("out.gcode"),
        &["--layer-height", "not-a-number"],
    );
    assert_eq!(result.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&result.stderr).contains("--layer-height: 'not-a-number' is not a value BambuStudio takes for this flag; it takes a number"));
}

#[test]
fn bad_legacy_geometry_option_emits_native_event_and_continues() {
    let dir = tempfile::tempdir().unwrap();
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let model = root.join("tests/fixtures/calib_base.3mf");
    let path = dir.path().join("out.gcode");
    let result = run(&model, &path, &["--perimeters", "bad"]);
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let events = String::from_utf8_lossy(&result.stdout);
    assert!(events.lines().any(|line| line
        .strip_prefix("[[SLICER_EVENT]] ")
        .and_then(|j| serde_json::from_str::<serde_json::Value>(j).ok())
        .is_some_and(|doc| doc["event"] == "override_rejected"
            && doc["tag"] == "InvalidOverrideValue"
            && doc["opt_key"] == "perimeters"
            && doc["value"] == "bad")));
    assert!(fs::read_to_string(path)
        .unwrap()
        .lines()
        .any(|s| s == "; wall_loops = 2"));
}

#[test]
fn nozzle_diameter_vector_reaches_stl_and_bambu_project() {
    use serde_json::{json, Value};
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let defaults =
        fs::read(root.join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json")).unwrap();
    let dir = tempfile::tempdir().unwrap();
    let profile = dir.path().join("profile.json");
    let output = dir.path().join("out.gcode");
    for model in [
        "tests/rust-engine/fixtures/Cube_25.6.stl",
        "tests/fixtures/calib_base.3mf",
    ] {
        for (baseline, requested) in [(json!(["0.4"]), "0.6"), (json!(["0.4", "0.4"]), "0.4,0.6")] {
            let slice = |rows: Value, flags: &[&str]| {
                let mut settings: Value = serde_json::from_slice(&defaults).unwrap();
                settings["nozzle_diameter"] = rows;
                fs::write(&profile, serde_json::to_vec(&settings).unwrap()).unwrap();
                let run = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
                    .current_dir(dir.path())
                    .arg("--config")
                    .arg(&profile)
                    .args(flags)
                    .arg("-o")
                    .arg(&output)
                    .arg(root.join(model))
                    .output()
                    .unwrap();
                assert!(
                    run.status.success(),
                    "{model} {flags:?}: {}",
                    String::from_utf8_lossy(&run.stderr)
                );
                let gcode = fs::read_to_string(&output).unwrap();
                assert_eq!(
                    gcode
                        .lines()
                        .find_map(|l| l.strip_prefix("; nozzle_diameter = ")),
                    Some(requested)
                );
                let motion: Vec<_> = gcode
                    .lines()
                    .filter(|l| {
                        matches!(l.split_whitespace().next(), Some("G0" | "G1" | "G2" | "G3"))
                    })
                    .map(str::to_owned)
                    .collect();
                assert!(!motion.is_empty());
                motion
            };
            let control = slice(requested.split(',').collect::<Vec<_>>().into(), &[]);
            let flagged = slice(baseline, &["--nozzle-diameter", requested]);
            assert!(
                flagged == control,
                "{model} {requested}: motion differs from the equivalent profile"
            );
        }
    }
}

#[test]
fn nozzle_diameter_validation_precedes_model_loading_and_replays_the_old_refusal() {
    let run = |gate: &str| {
        Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
            .env("CLI_NOZZLE_DIAMETER", gate)
            .args(["--nozzle-diameter=0.4,bad", "missing.stl"])
            .output()
            .unwrap()
    };
    let enabled = run("1");
    assert_eq!(enabled.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&enabled.stderr)
        .contains("Params in command line error: nozzle_diameter: invalid value 0.4,0"));
    let disabled = run("0");
    assert_eq!(disabled.status.code(), Some(254));
    assert!(disabled
        .stderr
        .starts_with(b"--nozzle-diameter is not a flag"));
}
