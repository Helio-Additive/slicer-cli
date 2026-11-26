//! Legacy thermal flags from cli_command_line.cpp 562–563 and main.cpp 7267–7327.
use serde_json::{json, Value};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
};

fn root() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn slice(dir: &Path, input: &str, profile: Option<&Path>, flags: &[&str]) -> (Output, PathBuf) {
    let path = dir.join("part.gcode");
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"));
    cmd.current_dir(dir);
    if let Some(profile) = profile {
        cmd.arg("--config").arg(profile);
    }
    let result = cmd
        .args(flags)
        .arg("-o")
        .arg(&path)
        .arg(root().join(input))
        .output()
        .unwrap();
    (result, path)
}

fn setting<'a>(gcode: &'a str, key: &str) -> Option<&'a str> {
    let key = format!("; {key} = ");
    gcode.lines().find_map(|s| s.strip_prefix(&key))
}

#[test]
fn overrides_use_the_selected_plate_and_keep_initial_nozzle_temperature() {
    let dir = tempfile::tempdir().unwrap();
    let original =
        fs::read(root().join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json")).unwrap();
    for (plate, key) in [
        ("Cool Plate", "cool_plate_temp"),
        ("Engineering Plate", "eng_plate_temp"),
        ("High Temp Plate", "hot_plate_temp"),
        ("Textured PEI Plate", "textured_plate_temp"),
        ("Supertack Plate", "supertack_plate_temp"),
    ] {
        let mut cfg: Value = serde_json::from_slice(&original).unwrap();
        cfg["curr_bed_type"] = json!(plate);
        let path = dir.path().join("plate.json");
        fs::write(&path, serde_json::to_vec(&cfg).unwrap()).unwrap();
        let (result, out) = slice(
            dir.path(),
            "tests/rust-engine/fixtures/Cube_25.6.stl",
            Some(&path),
            &["--temp", "211", "--temp=222C", "--bed-temp", "53 C"],
        );
        assert!(
            result.status.success(),
            "{plate}: {}",
            String::from_utf8_lossy(&result.stderr)
        );
        let gcode = fs::read_to_string(out).unwrap();
        assert_eq!(setting(&gcode, "curr_bed_type"), Some(plate));
        for k in ["bed_temperature", key, &format!("{key}_initial_layer")] {
            assert_eq!(setting(&gcode, k), Some("53"), "{plate} {k}");
        }
        assert_eq!(setting(&gcode, "nozzle_temperature"), Some("222"));
        assert!(setting(&gcode, "nozzle_temperature_initial_layer")
            .unwrap()
            .split(',')
            .all(|v| v == "220"));
        assert!(
            gcode.lines().any(|s| s.starts_with("M190 S53")),
            "{plate}: bed heat"
        );
        assert!(
            gcode.lines().any(|s| s.starts_with("M104 M S222")),
            "{plate}: nozzle heat"
        );
    }
}

#[test]
fn temperature_override_reads_embedded_project_and_reports_rejected_values() {
    let dir = tempfile::tempdir().unwrap();
    let model = "tests/fixtures/calib_base.3mf";
    let (result, out) = slice(dir.path(), model, None, &["--temp", "222", "--bed-temp=53"]);
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let gcode = fs::read_to_string(out).unwrap();
    assert_eq!(setting(&gcode, "bed_temperature"), Some("53"));
    assert_eq!(
        setting(&gcode, "textured_plate_temp_initial_layer"),
        Some("53")
    );
    assert_eq!(setting(&gcode, "nozzle_temperature"), Some("222"));
    let (result, out) = slice(
        dir.path(),
        model,
        None,
        &["--temp=bad", "--bed-temp", "99999999999999"],
    );
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let events: Vec<_> = String::from_utf8_lossy(&result.stdout)
        .lines()
        .filter_map(|line| line.strip_prefix("[[SLICER_EVENT]] "))
        .filter_map(|row| serde_json::from_str::<Value>(row).ok())
        .filter(|v| v["tag"] == "InvalidOverrideValue")
        .map(|v| {
            (
                v["opt_key"].as_str().unwrap().to_owned(),
                v["value"].as_str().unwrap().to_owned(),
            )
        })
        .collect();
    assert_eq!(
        events,
        [
            ("bed_temperature".into(), "99999999999999".into()),
            ("nozzle_temperature".into(), "bad".into())
        ]
    );
    assert_eq!(
        setting(&fs::read_to_string(out).unwrap(), "nozzle_temperature"),
        Some("220")
    );
}
