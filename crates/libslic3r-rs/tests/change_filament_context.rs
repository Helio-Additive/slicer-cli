//! End-to-end: build a change-filament Context from a Majora-like config and run
//! the REAL template through it — proving the data side + evaluator combine into
//! a fully-resolved, sane tool-change G-code block.

use slicer::gcode::change_filament::build_context;
use slicer::gcode::gcode_template::process;
use slicer::print_config::PrintConfig;

const TEMPLATE: &str = include_str!("data/majora_change_filament.txt");

/// Majora-like uniform 8-filament PLA config.
fn majora_config() -> PrintConfig {
    let mut c = PrintConfig::default();
    c.filament_diameters = vec![1.75; 8];
    c.filament_colours = vec!["#FFFFFF".into(); 8];
    c.extruder_temperature = 200;
    c.first_layer_extruder_temperature = 220;
    c.nozzle_temperature_range_high = 240;
    c.filament_max_volumetric_speed = 12.0;
    c.filament_type = "PLA".into();
    c.retract_length_toolchange = 2.0;
    c.retraction_distances_when_cut = 18.0;
    c.first_layer_height = 0.3;
    c.initial_layer_acceleration = 500.0;
    c.default_acceleration = 10000.0;
    c
}

#[test]
fn real_template_resolves_from_built_context() {
    let cfg = majora_config();
    // A representative mid-print change 1→2 with a 700mm³ flush.
    let ctx = build_context(&cfg, 1, 2, 12.4, 12.4, 5, 700.0, false);
    let out = process(TEMPLATE, &ctx);

    assert!(
        !out.contains('{') && !out.contains('['),
        "unresolved placeholder:\n{}",
        out.lines()
            .filter(|l| l.contains('{') || l.contains('['))
            .collect::<Vec<_>>()
            .join("\n")
    );
    // The toolchange itself and the AMS header/trailer for next_extruder=2.
    assert!(out.contains("M620 S2A"), "AMS header");
    assert!(out.contains("\nT2\n"), "bare toolchange");
    assert!(out.contains("M621 S2A"), "AMS trailer");
    // Flush temp comes from the range-high fallback (240).
    assert!(out.contains("M109 S240"), "flush temp 240:\n{}", out);
}

#[test]
fn flush_length_split_matches_cpp_formula() {
    // purge 700 → clamped max(700,100)=700; flush_count=min(4,round(700/135))=
    // min(4,5)=4; purge_length=700/(pi/4*1.75^2)=700/2.4053=291.0; unit=291/4=72.76
    let cfg = majora_config();
    let ctx = build_context(&cfg, 0, 1, 5.0, 5.0, 3, 700.0, false);
    let out = process("F1 {flush_length_1} F2 {flush_length_2} F4 {flush_length_4}", &ctx);
    let area = std::f64::consts::PI / 4.0 * 1.75 * 1.75;
    let unit = (700.0 / area) / 4.0;
    let expect = format!("F1 {u} F2 {u} F4 {u}", u = ostream_default(unit));
    assert_eq!(out.trim(), expect, "flush split: {}", out);
    // The pin's PlaceholderParser prints exactly this for the flush_length_*
    // values GCode.cpp:1104-1119 computes (a generator doing both).
    assert_eq!(out.trim(), "F1 72.7565 F2 72.7565 F4 72.7565");
}

#[test]
fn small_purge_uses_single_segment() {
    // purge 80 → clamped to max(80,100)=100; round(100/135)=1 → flush_count=1.
    let cfg = majora_config();
    let ctx = build_context(&cfg, 0, 1, 5.0, 5.0, 3, 80.0, false);
    let out = process("{flush_length_1}|{flush_length_2}", &ctx);
    let area = std::f64::consts::PI / 4.0 * 1.75 * 1.75;
    let unit = 100.0 / area; // flush_count=1 → whole purge_length in segment 1
    assert_eq!(out.trim(), format!("{}|0", ostream_default(unit)));
    // As the pin's PlaceholderParser prints it (a generator).
    assert_eq!(out.trim(), "41.5752|0");
}

/// How the placeholder parser prints a double: `std::ostringstream ss; ss <<
/// data.d;` (PlaceholderParser.cpp:277-281), the stream default of six
/// significant digits with trailing zeros dropped (printf `%g`). Only the
/// fixed-point range these tests reach is handled.
fn ostream_default(f: f64) -> String {
    if f == 0.0 {
        return "0".into();
    }
    let exp = f.abs().log10().floor() as i32;
    assert!((-5..6).contains(&exp), "scientific range not handled");
    let s = format!("{:.*}", (5 - exp).max(0) as usize, f);
    let s = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.') } else { &s };
    s.to_string()
}
