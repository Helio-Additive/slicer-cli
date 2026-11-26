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
