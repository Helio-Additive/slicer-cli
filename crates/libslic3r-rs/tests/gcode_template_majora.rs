//! Validates the `gcode_template` evaluator against the REAL BambuStudio
//! Majora `change_filament_gcode` (185 lines, extracted from the fixture 3MF).
//! Asserts every `{expr}`/`[var]`/`{if…}` resolves — no placeholder survives.

use slicer::gcode::gcode_template::{process, Context, Value};

const TEMPLATE: &str = include_str!("data/majora_change_filament.gcode");

fn n_of(v: f64, n: usize) -> Vec<Value> {
    vec![Value::Float(v); n]
}

fn full_ctx(next: i64, prev: i64) -> Context {
    let n = 8usize;
    let mut c = Context::new();
    c.set_int("next_extruder", next)
        .set_int("previous_extruder", prev)
        .set_int("old_filament_temp", 200)
        .set_int("new_filament_temp", 200)
        .set_float("max_layer_z", 12.4)
        .set_float("layer_z", 12.4)
        .set_int("toolchange_count", 2)
        .set_float("old_retract_length_toolchange", 2.0)
        .set_float("new_retract_length_toolchange", 2.0)
        .set_float("initial_layer_print_height", 0.3)
        .set_int("initial_layer_acceleration", 500)
        .set_int("default_acceleration", 10000)
        // exercise flush blocks 1 (big/pulsatile) and 2, skip 3/4
        .set_float("flush_length_1", 40.0)
        .set_float("flush_length_2", 5.0)
        .set_float("flush_length_3", 0.0)
        .set_float("flush_length_4", 0.0)
        .set_float("x_after_toolchange", 100.0)
        .set_float("y_after_toolchange", 100.0)
        .set_float("z_after_toolchange", 12.4)
        .set_float("travel_point_1_x", 10.0)
        .set_float("travel_point_1_y", 20.0)
        .set_float("travel_point_2_x", 30.0)
        .set_float("travel_point_2_y", 40.0)
        .set_float("travel_point_3_x", 50.0)
        .set_float("travel_point_3_y", 60.0)
        .set_array("flush_temperatures", vec![Value::Int(200); n])
        .set_array("flush_volumetric_speeds", n_of(12.0, n))
        .set_array(
            "filament_type",
            vec![Value::Str("PLA".into()); n],
        )
        .set_array("retraction_distances_when_cut", n_of(18.0, n))
        .set_array("long_retractions_when_cut", vec![Value::Bool(false); n]);
    c
}

#[test]
fn real_template_fully_resolves() {
    let out = process(TEMPLATE, &full_ctx(2, 1));
    // No placeholder delimiter may survive a full context.
    assert!(
        !out.contains('{'),
        "leftover brace placeholder:\n{}",
        out.lines().filter(|l| l.contains('{')).collect::<Vec<_>>().join("\n")
    );
    assert!(
        !out.contains('['),
        "leftover bracket placeholder:\n{}",
        out.lines().filter(|l| l.contains('[')).collect::<Vec<_>>().join("\n")
    );
    // Concrete substitutions for next_extruder=2.
    assert!(out.contains("M620 S2A"), "M620 header: {}", &out[..200.min(out.len())]);
    assert!(out.contains("\nT2\n"), "bare toolchange T2 present");
    assert!(out.contains("M621 S2A"), "M621 trailer present");
    // The next<255 branch runs, so the x_after_toolchange else-branch does NOT.
    assert!(!out.contains("x_after_toolchange"));
}

#[test]
fn resolves_for_all_valid_tool_indices() {
    // Real toolchanges use valid filament indices (0..8), never the 255 "no
    // next tool" sentinel — check every ordered pair fully resolves.
    for next in 0..8i64 {
        for prev in 0..8i64 {
            if next == prev {
                continue;
            }
            let out = process(TEMPLATE, &full_ctx(next, prev));
            assert!(
                !out.contains('{') && !out.contains('['),
                "unresolved placeholder for next={next} prev={prev}:\n{}",
                out.lines()
                    .filter(|l| l.contains('{') || l.contains('['))
                    .collect::<Vec<_>>()
                    .join("\n")
            );
            assert!(out.contains(&format!("\nT{next}\n")), "T{next} present");
        }
    }
}
