// main.cpp 735–756, 808–850 and 9550–9581. A settings file overlays the printer in the project.
use serde_json::{json, Value};
use std::{fs, path::Path, process::Command};

#[test]
fn project_printer_fields_survive_a_partial_external_profile() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let dir = tempfile::tempdir().unwrap();
    let settings = dir.path().join("settings.json");
    let output = dir.path().join("out.gcode");
    let mut profile: Value = serde_json::from_slice(
        &fs::read(root.join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json")).unwrap(),
    )
    .unwrap();
    for key in [
        "nozzle_diameter",
        "printer_model",
        "printer_variant",
        "printer_settings_id",
    ] {
        profile.as_object_mut().unwrap().remove(key);
    }
    for (override_value, diameter) in [
        (None, "0.4"),
        (Some(json!([false, "0.6"])), "0.6"),
        (Some(json!({})), "0.4"),
    ] {
        if let Some(value) = override_value {
            profile["nozzle_diameter"] = value;
        }
        fs::write(&settings, serde_json::to_vec(&profile).unwrap()).unwrap();
        let run = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
            .current_dir(dir.path())
            .arg("--config")
            .arg(&settings)
            .arg("-o")
            .arg(&output)
            .arg(root.join("tests/fixtures/calib_base.3mf"))
            .output()
            .unwrap();
        assert!(
            run.status.success(),
            "{}",
            String::from_utf8_lossy(&run.stderr)
        );
        let body = fs::read_to_string(&output).unwrap();
        for (key, value) in [
            ("nozzle_diameter", diameter),
            ("printer_model", "Bambu Lab X1 Carbon"),
            ("printer_variant", "0.4"),
            ("printer_settings_id", "Bambu Lab X1 Carbon 0.4 nozzle"),
        ] {
            let expected = format!("; {key} = {value}");
            assert!(body.lines().any(|l| l == expected), "missing {expected}");
        }
    }
}
