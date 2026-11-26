// GCodeWriter.cpp 100–125 and GCode.cpp 6473–6503, 8498–8523.
use std::{fs, path::Path, process::Command};

#[test]
fn profile_axis_mode_reaches_startup_and_model_moves() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let dir = tempfile::tempdir().unwrap();
    let bytes =
        fs::read(root.join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json")).unwrap();
    let mut config: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    let profile = dir.path().join("settings.json");
    let output = dir.path().join("model.gcode");
    for model in [
        "tests/rust-engine/fixtures/Cube_25.6.stl",
        "tests/fixtures/calib_base.3mf",
    ] {
        for (setting, preamble, first_xy_e) in [
            (
                "0",
                "M82 ; use absolute distances for extrusion\nG92 E0\n",
                "E1.28964",
            ),
            (
                "1",
                "M83 ; use relative distances for extrusion\n",
                "E.88964",
            ),
        ] {
            config["use_relative_e_distances"] = setting.into();
            fs::write(&profile, serde_json::to_vec(&config).unwrap()).unwrap();
            let run = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
                .current_dir(dir.path())
                .arg("--config")
                .arg(&profile)
                .arg("-o")
                .arg(&output)
                .arg(root.join(model))
                .output()
                .unwrap();
            assert!(
                run.status.success(),
                "{model}: {}",
                String::from_utf8_lossy(&run.stderr)
            );
            let gcode = fs::read_to_string(&output).unwrap();
            let (_, body) = gcode
                .split_once(preamble)
                .expect("native extrusion preamble");
            let lines: Vec<_> = body.lines().collect();
            let i = lines.iter().position(|l| l.starts_with("G1 E-")).unwrap();
            assert_eq!(lines[i + 1] == "G92 E0", setting == "0", "{model}");
            if model.ends_with(".stl") {
                let deposition = lines[i + 1..]
                    .iter()
                    .find(|l| l.starts_with("G1 X") && l.contains(" E"))
                    .unwrap();
                assert_eq!(
                    deposition.split_whitespace().find(|s| s.starts_with('E')),
                    Some(first_xy_e)
                );
            }
        }
    }
}
