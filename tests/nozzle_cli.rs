// cli_command_line.cpp 561 and main.cpp 7244–7257, 9737–9790.
use serde_json::{json, Value};
use std::{fs, path::Path, process::Command};

#[test]
fn nozzle_diameter_checks_the_printer_and_keeps_the_native_rows() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let dir = tempfile::tempdir().unwrap();
    let settings = dir.path().join("settings.json");
    let out = dir.path().join("model.gcode");
    let defaults =
        fs::read(root.join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json")).unwrap();
    for model in [
        "tests/rust-engine/fixtures/Cube_25.6.stl",
        "tests/fixtures/calib_base.3mf",
    ] {
        for (initial, flags, expected, tag) in [
            (json!(["0.4"]), vec!["--nozzle", "0.4"], Some("0.4"), None),
            (
                json!(["0.4", "0.4"]),
                vec!["--nozzle=.4"],
                Some("0.4,0.4"),
                None,
            ),
            (
                json!(["0.4", "0.6"]),
                vec!["--nozzle", "0.4,0.6"],
                Some("0.4,0.6"),
                None,
            ),
            (json!(["0.4"]), vec!["--nozzle", "0.6mm"], Some("0.6"), None),
            (
                json!(["0.4"]),
                vec!["--nozzle", "bad"],
                Some("0.4"),
                Some("InvalidOverrideValue"),
            ),
            (
                json!(["0.4", "0.6"]),
                vec!["--nozzle", "0.4"],
                None,
                Some("NozzleFlagNotAPrinterVariant"),
            ),
            (
                json!(["0.4"]),
                vec!["--nozzle-diameter=0.6", "--nozzle", "0.4"],
                None,
                Some("NozzleFlagNotAPrinterVariant"),
            ),
        ] {
            let mut profile: Value = serde_json::from_slice(&defaults).unwrap();
            profile["nozzle_diameter"] = initial;
            fs::write(&settings, serde_json::to_vec(&profile).unwrap()).unwrap();
            let run = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
                .current_dir(dir.path())
                .arg("--config")
                .arg(&settings)
                .args(&flags)
                .arg("-o")
                .arg(&out)
                .arg(root.join(model))
                .output()
                .unwrap();
            let text = format!(
                "{}\n{}",
                String::from_utf8_lossy(&run.stdout),
                String::from_utf8_lossy(&run.stderr)
            );
            assert_eq!(
                run.status.code(),
                Some(if expected.is_some() { 0 } else { 1 }),
                "{model} {flags:?}: {text}"
            );
            if let Some(expected) = expected {
                let gcode = fs::read_to_string(&out).unwrap();
                assert_eq!(
                    gcode
                        .lines()
                        .find_map(|l| l.strip_prefix("; nozzle_diameter = ")),
                    Some(expected)
                );
            }
            if let Some(tag) = tag {
                let events: Vec<Value> = text
                    .lines()
                    .filter_map(|s| s.strip_prefix("[[SLICER_EVENT]] "))
                    .filter_map(|s| serde_json::from_str(s).ok())
                    .collect();
                let event = events.iter().find(|e| e["tag"] == tag).unwrap();
                if tag == "NozzleFlagNotAPrinterVariant" {
                    assert_eq!(event["nozzle"], "0.4");
                    assert!(event["message"]
                        .as_str()
                        .unwrap()
                        .contains("printer preset"));
                } else {
                    assert_eq!(event["opt_key"], "nozzle_diameter");
                    assert_eq!(event["value"], "bad");
                }
            }
        }
    }
}

#[test]
fn nozzle_guard_reads_the_bundled_parent_and_names_the_shipped_variant() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let dir = tempfile::tempdir().unwrap();
    let package = dir.path().join("package");
    let profiles = package.join("resources/profiles");
    let machine = profiles.join("BBL/machine");
    fs::create_dir_all(&machine).unwrap();
    let model = "Bambu Lab X1 Carbon";
    let small = format!("{model} 0.4 nozzle");
    let large = format!("{model} 0.6 nozzle");
    let manifest = [&small, &large].map(|name| json!({"name": name, "sub_path": format!("machine/{name}.json")}));
    fs::write(profiles.join("BBL.json"), serde_json::to_vec(&json!({"machine_list": manifest})).unwrap()).unwrap();
    for (name, d) in [(&small, "0.4"), (&large, "0.6")] {
        let printer = json!({"type":"machine", "name":name, "from":"system", "printer_model":model,
            "printer_variant":d, "nozzle_diameter":[d]});
        fs::write(
            machine.join(format!("{name}.json")),
            serde_json::to_vec(&printer).unwrap(),
        )
        .unwrap();
    }
    let profile = dir.path().join("settings.json");
    let mut input: Value = serde_json::from_slice(
        &fs::read(root.join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json")).unwrap(),
    )
    .unwrap();
    for key in [
        "nozzle_diameter",
        "printer_model",
        "printer_variant",
        "printer_settings_id",
    ] {
        input.as_object_mut().unwrap().remove(key);
    }
    input["type"] = "machine".into();
    input["inherits"] = large.as_str().into();
    fs::write(&profile, serde_json::to_vec(&input).unwrap()).unwrap();
    let binary = Path::new(env!("CARGO_BIN_EXE_slicer_cli-rust"));
    for folder in [package.clone(), package.join("bin")] {
        fs::create_dir_all(&folder).unwrap();
        let exe = folder.join(binary.file_name().unwrap());
        fs::copy(binary, &exe).unwrap();
        let output = dir.path().join("out.gcode");
        let run = |diameter: &str| {
            Command::new(&exe)
                .current_dir(dir.path())
                .arg("--config")
                .arg(&profile)
                .args(["--nozzle", diameter])
                .arg("-o")
                .arg(&output)
                .arg(root.join("tests/rust-engine/fixtures/Cube_25.6.stl"))
                .output()
                .unwrap()
        };
        let refused = run("0.4");
        assert_eq!(refused.status.code(), Some(1));
        let event = String::from_utf8_lossy(&refused.stdout)
            .lines()
            .filter_map(|l| l.strip_prefix("[[SLICER_EVENT]] "))
            .filter_map(|l| serde_json::from_str::<Value>(l).ok())
            .find(|v| v["tag"] == "NozzleFlagNotAPrinterVariant")
            .unwrap();
        assert_eq!(event["printer_preset"], large);
        assert_eq!(event["printer_variant"], "0.6");
        assert_eq!(event["message"], format!("--nozzle 0.4 does not change the printer: '{large}' is built for a 0.6 mm nozzle. The desktop app changes the nozzle by choosing the printer variant, so give the printer preset of that nozzle instead: --printer-preset \"{small}\". This engine ships {small} and {large}."));
        let accepted = run("0.6");
        assert!(
            accepted.status.success(),
            "{}",
            String::from_utf8_lossy(&accepted.stderr)
        );
        assert!(fs::read_to_string(&output)
            .unwrap()
            .lines()
            .any(|l| l == "; nozzle_diameter = 0.6"));
    }
}
