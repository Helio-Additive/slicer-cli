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
