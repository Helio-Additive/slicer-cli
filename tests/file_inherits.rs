// main.cpp 5459–5517, 9554–9568 and desktop_presets.cpp 197–259, 393–424.
use serde_json::{json, Value};
use std::{fs, path::Path, process::Command};

fn write(file: &Path, data: Value) {
    fs::create_dir_all(file.parent().unwrap()).unwrap();
    fs::write(file, serde_json::to_vec(&data).unwrap()).unwrap();
}

#[test]
fn external_process_settings_inherit_before_official_overrides() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let temp = tempfile::tempdir().unwrap();
    let package = temp.path().join("package");
    let profiles = package.join("resources/profiles");
    let vendor = profiles.join("BBL");
    let binary = Path::new(env!("CARGO_BIN_EXE_slicer_cli-rust"));
    let exe = package.join(binary.file_name().unwrap());
    write(&profiles.join("BBL.json"), json!({"process_list": [
        {"name":"Base","sub_path":"process/base/parent.json"},
        {"name":"Fine","sub_path":"process/child.json"}]}));
    write(&vendor.join("process/base/parent.json"), json!({"type":"process","name":"Base",
        "from":"system","wall_loops":"5","layer_height":"0.2"}));
    write(&vendor.join("process/include.json"), json!({"type":"process","name":"include",
        "from":"system","wall_loops":"2","top_shell_layers":"7"}));
    write(&vendor.join("process/child.json"), json!({"type":"process","name":"Fine",
        "from":"system","inherits":"Base","include":["include","missing"],"layer_height":"0.16"}));
    fs::copy(binary, &exe).unwrap();
    let settings = temp.path().join("profile.json");
    let output = temp.path().join("model.gcode");
    let mut base: Value = serde_json::from_slice(
        &fs::read(root.join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json")).unwrap(),
    ).unwrap();
    for key in ["wall_loops", "layer_height", "top_shell_layers"] {
        base.as_object_mut().unwrap().remove(key);
    }
    base["type"] = "process".into();
    base["from"] = "user".into();
    base["inherits"] = "Fine".into();
    for model in ["tests/rust-engine/fixtures/Cube_25.6.stl", "tests/fixtures/calib_base.3mf"] {
        for (own, flag, expected_walls, expected_height) in [
            (None, None, "5", "0.16"),
            (Some("4"), None, "4", "0.16"),
            (Some("4"), Some("0.25"), "4", "0.25"),
        ] {
            let mut file = base.clone();
            if let Some(count) = own { file["wall_loops"] = count.into(); }
            write(&settings, file);
            let mut command = Command::new(&exe);
            command.current_dir(temp.path()).arg("--config").arg(&settings).arg("--verbose");
            if let Some(height) = flag { command.args(["--layer-height", height]); }
            let run = command.arg("-o").arg(&output).arg(root.join(model)).output().unwrap();
            assert!(run.status.success(), "{}", String::from_utf8_lossy(&run.stderr));
            let gcode = fs::read_to_string(&output).unwrap();
            for (key, value) in [("wall_loops", expected_walls), ("layer_height", expected_height),
                                 ("top_shell_layers", "7"), ("inherits", "Fine")] {
                let line = format!("; {key} = {value}");
                assert!(gcode.lines().any(|l| l == line), "{model}: {line}");
            }
            let message = format!("Applied Fine, the preset {} inherits", settings.display());
            assert!(String::from_utf8_lossy(&run.stdout).lines().any(|l| l == message));
        }
    }
}
