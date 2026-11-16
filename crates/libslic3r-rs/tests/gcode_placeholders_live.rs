//! The export's placeholder parser (`slicer::g_code::GCodePlaceholders`, GCode.cpp:2765-3182,
//! 4110-4127) and the template funnel `process_gcode_template` (gate PLACEHOLDER_PARSER_LIVE): the
//! templates are evaluated by `PlaceholderParser::process` with the placeholders typed as GCode.cpp
//! types them, so they behave as the C++ does where the earlier substitution did not.

use slicer::config::ConfigOptionResolver;
use slicer::g_code::{full_print_config_from_settings, ExportPlaceholderInputs, GCodePlaceholders};
use slicer::print_config::PrintConfig;

fn settings() -> serde_json::Value {
    serde_json::json!({
        "name": "project_settings",
        "printer_settings_id": "Test printer",
        "nozzle_diameter": ["0.4"],
        "filament_type": ["PLA", "PETG"],
        "filament_vendor": ["Bambu Lab", "Bambu Lab"],
        "filament_map": ["1", "1"],
        "curr_bed_type": "Textured PEI Plate",
        "textured_plate_temp": ["55", "70"],
        "textured_plate_temp_initial_layer": ["60", "75"],
        "nozzle_temperature": ["220", "250"],
        "nozzle_temperature_initial_layer": ["225", "255"],
        "filament_max_volumetric_speed": ["21", "12"],
        "filament_flush_temp": ["0", "0"],
        "nozzle_temperature_range_high": ["240", "270"],
        "machine_start_gcode": "M190 S[bed_temperature_initial_layer_single]\nM104 S{nozzle_temperature_initial_layer[initial_no_support_extruder]}\n",
        "filament_start_gcode": ["; start {current_filament_id}", "; start {current_filament_id}"],
    })
}

fn live() -> bool {
    slicer::faithful_gate("PLACEHOLDER_PARSER_LIVE")
}

#[test]
fn full_config_reads_strings_and_arrays_by_type() {
    let cfg = full_print_config_from_settings(&settings());
    let ty = cfg.option("filament_type").unwrap().serialize().unwrap();
    assert_eq!(ty, "PLA;PETG");
    assert_eq!(cfg.option("nozzle_temperature").unwrap().serialize().unwrap(), "220,250");
    assert_eq!(cfg.option("curr_bed_type").unwrap().serialize().unwrap(), "Textured PEI Plate");
    // A key the file does not set keeps its default.
    assert!(cfg.option("layer_height").is_some());
}

#[test]
fn start_placeholders_follow_do_export() {
    let full = full_print_config_from_settings(&settings());
    let inputs = ExportPlaceholderInputs::from_settings(&full, &settings(), 12.3);
    let mut gp = GCodePlaceholders::new(full, &inputs);
    gp.init_export_placeholders(&inputs);
    // GCode.cpp:3068: the first layer's bed temperature of the initial filament on this plate.
    let out = gp.placeholder_parser_process("t", "[bed_temperature_initial_layer_single] {max_print_z} {flush_temperatures[1]}", 0, None);
    // GCode.cpp:3031 max_print_z = ceil; 8605-8606 flush_temperatures falls back to the range high.
    assert_eq!(out, "60 13 270");
    // GCode.cpp:3083: is_all_bbl_filament is a bool, rendered as the parser renders a bool.
    assert_eq!(gp.placeholder_parser_process("t", "{is_all_bbl_filament} [is_all_bbl_filament]", 0, None), "true 1");
    assert!(gp.check_placeholder_parser_failed().is_ok());
}

#[test]
fn a_failed_template_reports_like_the_cpp() {
    let full = full_print_config_from_settings(&settings());
    let inputs = ExportPlaceholderInputs::from_settings(&full, &settings(), 0.0);
    let mut gp = GCodePlaceholders::new(full, &inputs);
    gp.init_export_placeholders(&inputs);
    // An int is not a boolean expression (PlaceholderParser.cpp); the earlier substitution took it.
    let out = gp.placeholder_parser_process("machine_end_gcode", "{if total_layer_count}x{endif}", 0, None);
    assert!(out.starts_with("\n!!!!! Failed to process the custom G-code template machine_end_gcode\n"), "{out:?}");
    assert!(out.ends_with("!!!!! End of an error report for the custom G-code template machine_end_gcode\n\n"), "{out:?}");
    let err = gp.check_placeholder_parser_failed().unwrap_err();
    assert!(err.what().starts_with("Failed to generate gcode for invalid custom G-code.\n\nmachine_end_gcode "), "{}", err.what());
    assert!(err.what().ends_with("You can find it from 'Printer settings' -> 'Machine G-code'."), "{}", err.what());
}

#[test]
fn funnel_types_the_injected_keys() {
    if !live() {
        return;
    }
    let config = PrintConfig::default();
    let mut s = settings();
    s["layer_num"] = serde_json::json!(3);
    s["layer_z"] = serde_json::Value::String("0.36".into());
    s["first_layer_print_min"] = serde_json::json!(["20.0429", "44.5913"]);
    s["first_layer_print_size"] = serde_json::json!(["207.5036", "197.1683"]);
    // GCode.cpp:4763-4764 layer_num an int, layer_z a float; 2994-2996 the first-layer box as
    // floats, printed with six significant digits.
    let out = slicer::gcode::process_gcode_template(
        "[layer_num] {layer_z + 0.04} X{first_layer_print_min[0]} I{first_layer_print_size[0]} J{first_layer_print_size[1]}",
        &s,
        &config,
    );
    assert_eq!(out, "3 0.4 X20.0429 I207.504 J197.168");
    // An int in a condition is a parse error in the C++.
    let out = slicer::gcode::process_gcode_template("{if layer_num}x{endif}", &s, &config);
    assert!(out.contains("!!!!! Failed to process the custom G-code template custom_gcode"), "{out:?}");
    // A per-filament template is processed for its filament (GCode.cpp:3618).
    let out = slicer::gcode::process_gcode_template("; start {current_filament_id}", &s, &config);
    assert_eq!(out, "; start 0");
    // The machine start template, by its name (GCode.cpp:3184).
    let start = s["machine_start_gcode"].as_str().unwrap().to_string();
    assert_eq!(slicer::gcode::process_gcode_template(&start, &s, &config), "M190 S60\nM104 S225\n");
}
