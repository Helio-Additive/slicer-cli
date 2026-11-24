//! `GCode::append_full_config` (GCode.cpp:6410-6430) and the CONFIG_BLOCK of `GCode::_do_export`
//! (GCode.cpp:2686-2723, `GCode::config_block_pin`), checked against lines the pin's slicer_cli
//! writes for the Benchy job (H2D 0.4, Bambu PLA Basic @BBL H2D, 0.16mm Standard @BBL H2D): the
//! expected strings below are copied from that G-code's CONFIG_BLOCK.

use slicer::g_code::GCode;

#[test]
fn config_block_pin_serializes_like_the_pin() {
    let settings = serde_json::json!({
        "default_wipe_tower_pos": "0.644x0.977",
        "filament_max_volumetric_speed": ["25", "40", "25"],
        "z_hop_types": ["Auto Lift", "Auto Lift"],
        "volumetric_speed_coefficients": ["0 0 0 0 0 0", "0 0 0 0 0 0", "0 0 0 0 0 0"],
        "print_host": "a-host",
        "printhost_apikey": "a-key",
        "flush_multiplier": ["2"],
        "flush_volumes_matrix": ["0", "280", "280", "0"],
        "filament_colour": ["#FFFFFF", "#000000"],
        "post_process": [],
    });
    let block = GCode::config_block_pin(&settings);
    assert!(block.starts_with("; CONFIG_BLOCK_START\n"));
    assert!(block.ends_with("; CONFIG_BLOCK_END\n\n"));
    let lines: Vec<&str> = block.lines().collect();
    let has = |l: &str| lines.iter().any(|x| *x == l);
    // ConfigOptionPoint::serialize writes "x,y" (Config.hpp), whatever separator the file used.
    assert!(has("; default_wipe_tower_pos = 0.644,0.977"), "default_wipe_tower_pos");
    // Vector options keep every value (the pin writes the profile's three variant values).
    assert!(has("; filament_max_volumetric_speed = 25,40,25"), "filament_max_volumetric_speed");
    assert!(has("; z_hop_types = Auto Lift,Auto Lift"), "z_hop_types");
    assert!(
        has("; volumetric_speed_coefficients = \"0 0 0 0 0 0\";\"0 0 0 0 0 0\";\"0 0 0 0 0 0\""),
        "volumetric_speed_coefficients"
    );
    assert!(has("; post_process = "), "post_process");
    // GCode.cpp:2704-2711: the matrix is scaled by the flush multiplier of its head.
    assert!(has("; flush_volumes_matrix = 0,560,560,0"), "flush_volumes_matrix");
    // GCode.cpp:6413-6423: the host keys are never written.
    assert!(!lines.iter().any(|l| l.starts_with("; print_host ")), "print_host banned");
    assert!(!lines.iter().any(|l| l.starts_with("; printhost_apikey ")), "printhost_apikey banned");
    // DynamicConfig::keys() is the sorted order of the option map (Config.cpp:1846-1853).
    let body: Vec<&str> = lines
        .iter()
        .copied()
        .filter(|l| *l != "; CONFIG_BLOCK_START" && *l != "; CONFIG_BLOCK_END" && !l.is_empty())
        .collect();
    let keys: Vec<&str> =
        body.iter().map(|l| l.trim_start_matches("; ").split(" = ").next().unwrap()).collect();
    let mut sorted = keys.clone();
    sorted.sort();
    assert_eq!(keys, sorted);
    // Every option of the full print config is written (the pin's block has about 600 lines).
    assert!(keys.len() > 500, "{} keys", keys.len());
}

/// The front end's values (slicer_cli main.cpp:1355-1366 `ensure_temp_opt`, main.cpp:9944-9950 the
/// `master_extruder_id` clamp), as the pin writes them: Benchy's block (H2D, one nozzle_diameter
/// after the single-filament reduction) holds `bed_temperature = 60`,
/// `bed_temperature_initial_layer = 60` and `master_extruder_id = 1`; Majora's (X1C) the same 60s.
#[test]
fn config_block_pin_writes_the_front_end_values() {
    let settings = serde_json::json!({
        "nozzle_diameter": ["0.4"],
        "master_extruder_id": "2",
        "nozzle_temperature": ["220", "220"],
        "flush_multiplier": ["1"],
        "flush_volumes_matrix": ["0"],
        "filament_colour": ["#FFFFFF"],
    });
    let block = GCode::config_block_pin(&settings);
    let lines: Vec<&str> = block.lines().collect();
    let has = |l: &str| lines.iter().any(|x| *x == l);
    assert!(has("; bed_temperature = 60"), "bed_temperature");
    assert!(has("; bed_temperature_initial_layer = 60"), "bed_temperature_initial_layer");
    assert!(has("; master_extruder_id = 1"), "master_extruder_id");
    // ensure_temp_opt only fills a missing key.
    assert!(has("; nozzle_temperature = 220,220"), "nozzle_temperature");
    // DynamicConfig::keys() is sorted: the two keys the front end adds sit in that order.
    let pos = |k: &str| lines.iter().position(|l| l.starts_with(k)).unwrap();
    assert!(pos("; bed_temperature = ") < pos("; bed_temperature_formula = "));
    assert!(pos("; bed_temperature_formula = ") < pos("; bed_temperature_initial_layer = "));
}

/// The front end's padding of short vectors to `extruder_count` (slicer_cli main.cpp:9902-9935, with
/// `skip_pad` 9816-9862 and `is_filament_setting_key` 1654-1667), as the pin writes it: Benchy's block
/// (H2D, one nozzle_diameter) holds `extruder_ams_count = ""` and `extruder_nozzle_slot = 0`;
/// Majora's (X1C) `extruder_printable_height = 0`. A vector already as long or longer stays as it is
/// (Benchy's `extruder_printable_height = 320,325`), and filament settings and the skip_pad keys are
/// not padded.
#[test]
fn config_block_pin_pads_short_vectors_to_the_extruder_count() {
    let settings = serde_json::json!({
        "nozzle_diameter": ["0.4"],
        "flush_multiplier": ["1"],
        "flush_volumes_matrix": ["0"],
        "filament_colour": ["#FFFFFF"],
        "extruder_ams_count": [],
        "extruder_printable_area": [],
    });
    let block = GCode::config_block_pin(&settings);
    let lines: Vec<&str> = block.lines().collect();
    let has = |l: &str| lines.iter().any(|x| *x == l);
    assert!(has("; extruder_ams_count = \"\""), "extruder_ams_count");
    assert!(
        has("; extruder_nozzle_slot = 0"),
        "extruder_nozzle_slot: {:?}",
        lines.iter().filter(|l| l.starts_with("; extruder_n")).collect::<Vec<_>>()
    );
    assert!(has("; extruder_printable_height = 0"), "extruder_printable_height");
    // skip_pad: extruder_printable_area stays empty.
    assert!(has("; extruder_printable_area = "), "extruder_printable_area");

    let settings2 = serde_json::json!({
        "nozzle_diameter": ["0.4", "0.4"],
        "flush_multiplier": ["1", "1"],
        "flush_volumes_matrix": ["0", "0"],
        "filament_colour": ["#FFFFFF"],
        "extruder_printable_height": ["320", "325", "330"],
    });
    let block2 = GCode::config_block_pin(&settings2);
    let lines2: Vec<&str> = block2.lines().collect();
    let has2 = |l: &str| lines2.iter().any(|x| *x == l);
    // not truncated
    assert!(has2("; extruder_printable_height = 320,325,330"), "extruder_printable_height");
    // padded to two
    assert!(
        has2("; extruder_nozzle_slot = 0,0"),
        "extruder_nozzle_slot: {:?}",
        lines2.iter().filter(|l| l.starts_with("; extruder_n") || l.starts_with("; nozzle_d")).collect::<Vec<_>>()
    );
    // a filament setting keeps its length
    assert!(has2("; filament_colour = #FFFFFF"), "filament_colour");
}

/// Print::apply's reduction of the extruder-related values (PrintApply.cpp:1437-1453), as Majora's
/// block (X1C, one extruder, two variants in `extruder_variant_list`) shows it in the pin:
/// `bridge_speed = 50`, `print_extruder_variant = "Direct Drive Standard"` where the 3MF's settings
/// carry two values per variant.
#[test]
fn config_block_pin_reduces_the_variant_values_to_the_printer_extruders() {
    let settings = serde_json::json!({
        "nozzle_diameter": ["0.4"],
        "flush_multiplier": ["1"],
        "flush_volumes_matrix": ["0"],
        "filament_colour": ["#FFFFFF"],
        "extruder_type": ["Direct Drive"],
        "nozzle_volume_type": ["Standard"],
        "extruder_variant_list": ["Direct Drive Standard,Direct Drive High Flow"],
        "printer_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow"],
        "printer_extruder_id": ["1", "1"],
        "print_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow"],
        "print_extruder_id": ["1", "1"],
        "bridge_speed": ["50", "50"],
        "outer_wall_speed": ["150", "350"],
    });
    let block = GCode::config_block_pin(&settings);
    let lines: Vec<&str> = block.lines().collect();
    let has = |l: &str| lines.iter().any(|x| *x == l);
    let show = || lines.iter().filter(|l| l.contains("_speed =") || l.contains("extruder_")).cloned().collect::<Vec<_>>();
    assert!(has("; bridge_speed = 50"), "bridge_speed: {:?}", show());
    assert!(has("; outer_wall_speed = 150"), "outer_wall_speed: {:?}", show());
    assert!(has("; print_extruder_variant = \"Direct Drive Standard\""), "print_extruder_variant: {:?}", show());
}

/// The filament half (R943): main.cpp:1740-1808 `align_per_filament_config_vectors` grows the
/// per-filament vectors to the roster (`filament_is_mixed = 0,0,0`), PrintApply.cpp:1455-1483
/// reduces the filament options with variant to one value per filament
/// (`update_values_to_printer_extruders_for_multiple_filaments`, PrintConfig.cpp:9996-10191),
/// GCode.cpp:2719 writes `filament_map_2` as Print.cpp:3266-3285 computes it, and the tool
/// ordering's nozzle map replaces `filament_nozzle_map` (Print.cpp:3258-3261), as Majora's block
/// shows them in the pin.
#[test]
fn config_block_pin_reduces_the_filament_values_and_writes_the_maps() {
    let settings = serde_json::json!({
        "nozzle_diameter": ["0.4"],
        "flush_multiplier": ["1"],
        "flush_volumes_matrix": ["0", "0", "0", "0", "0", "0", "0", "0", "0"],
        "filament_colour": ["#FFFFFF", "#000000", "#FF0000"],
        "filament_map": ["1", "1", "1"],
        "filament_volume_map": ["0", "0", "0"],
        "extruder_type": ["Direct Drive"],
        "nozzle_volume_type": ["Standard"],
        "extruder_variant_list": ["Direct Drive Standard,Direct Drive High Flow"],
        "printer_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow"],
        "printer_extruder_id": ["1", "1"],
        "print_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow"],
        "print_extruder_id": ["1", "1"],
        "filament_extruder_variant": [
            "Direct Drive Standard", "Direct Drive High Flow",
            "Direct Drive Standard", "Direct Drive High Flow",
            "Direct Drive Standard", "Direct Drive High Flow"
        ],
        "filament_self_index": ["1", "1", "2", "2", "3", "3"],
        "filament_max_volumetric_speed": ["10", "20", "11", "21", "12", "22"],
    });
    let block = GCode::config_block_pin_with_nozzle_map(&settings, Some(&[0, 0, 0]));
    let lines: Vec<&str> = block.lines().collect();
    let has = |l: &str| lines.iter().any(|x| *x == l);
    let show = || lines.iter().filter(|l| l.starts_with("; filament_")).cloned().collect::<Vec<_>>();
    assert!(has("; filament_is_mixed = 0,0,0"), "filament_is_mixed: {:?}", show());
    assert!(has("; filament_max_volumetric_speed = 10,11,12"), "filament_max_volumetric_speed: {:?}", show());
    assert!(has("; filament_self_index = 1,2,3"), "filament_self_index: {:?}", show());
    assert!(has("; filament_map_2 = 0,0,0"), "filament_map_2: {:?}", show());
    assert!(has("; filament_nozzle_map = 0,0,0"), "filament_nozzle_map: {:?}", show());
}

/// slicer_cli main.cpp:1809-1946: a saved 2x2 `flush_volumes_matrix` of a three-filament job is
/// rebuilt to 3x3, the saved pairs kept, the new ones `flush_volumes_vector[2i] + [2j+1]` off the
/// diagonal and 0 on it (gate GCODE_CONFIG_FRONT_END_FLUSH_RESHAPE_PIN; with it `=0` the saved four
/// values are written as they are).
#[test]
fn config_block_pin_reshapes_a_short_flush_volumes_matrix() {
    let settings = serde_json::json!({
        "nozzle_diameter": ["0.4"],
        "flush_multiplier": ["1"],
        "flush_volumes_vector": ["10", "20", "30", "40", "50", "60"],
        "flush_volumes_matrix": ["0", "100", "200", "0"],
        "filament_colour": ["#FFFFFF", "#000000", "#FF0000"],
    });
    let block = GCode::config_block_pin(&settings);
    let lines: Vec<&str> = block.lines().collect();
    let got: Vec<&str> = lines.iter().filter(|l| l.starts_with("; flush_volumes_matrix")).cloned().collect();
    assert_eq!(got, vec!["; flush_volumes_matrix = 0,100,70,200,0,90,70,90,0"]);
}
