// BambuStudio GCode.cpp 3602-3629. Each filament and then the machine
// consume the same layer override, with the actual writer's filament active.
use serde_json::{json, Value};
use std::{fs, path::Path, process::Command};

#[test]
fn terminal_filament_and_machine_templates_match_the_native_export() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let dir = tempfile::tempdir().unwrap();
    let cfg = dir.path().join("settings.json");
    let out = dir.path().join("out.gcode");
    let base: Value = serde_json::from_slice(
        &fs::read(root.join("crates/libslic3r-rs/tests/data/cube_arachne_settings.json")).unwrap(),
    )
    .unwrap();
    let fields = "f{current_filament_id} e{current_extruder_id} n{current_nozzle_id} l{layer_num} g{current_extruder} d{nozzle_diameter_at_nozzle_id[current_nozzle_id]} type{nozzle_volume_types[current_nozzle_id]}";
    let row = format!("; MARK_END {fields}");
    let machine = format!(
        "; MARK_MACHINE {fields} v{{filament_extruder_id}} z{{layer_z}} max{{max_layer_z}}"
    );
    let vectors: Vec<Value> =
        serde_json::from_str(include_str!("rust-engine/data/end_templates.json")).unwrap();
    for vector in vectors {
        let model = match vector["format"].as_str().unwrap() {
            "stl" => "tests/rust-engine/fixtures/Cube_25.6.stl",
            "3mf" => "tests/fixtures/calib_base.3mf",
            v => panic!("unknown model format {v}"),
        };
        let case = vector["case"].as_str().unwrap();
        let mut profile = base.clone();
        profile["filament_start_gcode"] = json!([""]);
        profile["change_filament_gcode"] = json!("");
        profile["machine_start_gcode"] = json!("; MARK_START");
        profile["filament_end_gcode"] = json!([row]);
        profile["machine_end_gcode"] = json!(machine);
        profile["single_extruder_multi_material"] = json!("1");
        match case {
            "multiple" | "duplicate" | "empty" => {
                profile["filament_type"] = json!(["PLA", "PLA", "PLA"]);
                profile["filament_diameter"] = json!(["1.75", "1.75", "1.75"]);
                profile["filament_colour"] = json!(["#FFFFFF", "#000000", "#999999"]);
                profile["filament_map"] = json!(["1", "1", "1"]);
                profile["wall_filament"] = json!("2");
                profile["sparse_infill_filament"] = json!("3");
                profile["solid_infill_filament"] = json!("2");
                profile["enable_prime_tower"] = json!("0");
                profile["single_extruder_multi_material"] = json!("0");
                profile["filament_start_gcode"] = json!(["", "", ""]);
                profile["flush_volumes_matrix"] =
                    json!(["0", "280", "280", "280", "0", "280", "280", "280", "0"]);
                profile["filament_end_gcode"] = match case {
                    "multiple" => {
                        json!([format!("{row} A"), format!("{row} B"), format!("{row} C")])
                    }
                    "duplicate" => json!([row, row, row]),
                    _ => json!([row, "", format!("{row}\n")]),
                };
            }
            "two-first" | "two-second" => {
                profile["nozzle_diameter"] = json!(["0.4", "0.6"]);
                profile["physical_extruder_map"] = json!(["0", "1"]);
                profile["nozzle_volume_type"] = json!(["Standard", "High Flow"]);
                profile["extruder_type"] = json!(["Direct Drive", "Direct Drive"]);
                for key in ["print_extruder_id", "printer_extruder_id"] {
                    profile[key] = json!(["1", "2"]);
                }
                profile["filament_self_index"] = json!(["1", "1"]);
                for key in [
                    "extruder_variant_list",
                    "print_extruder_variant",
                    "printer_extruder_variant",
                    "filament_extruder_variant",
                ] {
                    profile[key] = json!(["Direct Drive Standard", "Direct Drive High Flow"]);
                }
                profile["filament_map"] = if case == "two-second" {
                    json!(["2"])
                } else {
                    json!(["1"])
                };
            }
            "single" => (),
            _ => panic!("unknown case {case}"),
        }
        fs::write(&cfg, serde_json::to_vec(&profile).unwrap()).unwrap();
        let run = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
            .current_dir(dir.path())
            .arg("--config")
            .arg(&cfg)
            .arg("-o")
            .arg(&out)
            .arg(root.join(model))
            .output()
            .unwrap();
        assert!(
            run.status.success(),
            "{model} {case}: {}",
            String::from_utf8_lossy(&run.stderr)
        );
        let gcode = fs::read_to_string(&out).unwrap();
        let (_, tail) = gcode.rsplit_once("; MACHINE_END_GCODE_START\n").unwrap();
        assert_eq!(
            tail.split_once("; EXECUTABLE_BLOCK_END").unwrap().0,
            vector["terminal"].as_str().unwrap(),
            "{model} {case}"
        );
    }
}
