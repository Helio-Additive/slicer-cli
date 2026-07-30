//! Tests for the expression-capable G-code template evaluator
//! (`gcode::gcode_template`), the working core of a PlaceholderParser port.
//! Cases mirror the real BambuStudio `change_filament_gcode` constructs.
//!
//! Integration target because the crate's in-lib `#[cfg(test)]` does not compile.

use slicer::gcode::gcode_template::{eval_expr, process, Context, Value};

fn ctx() -> Context {
    let mut c = Context::new();
    c.set_int("next_extruder", 3)
        .set_int("previous_extruder", 1)
        .set_int("old_filament_temp", 220)
        .set_float("max_layer_z", 12.4)
        .set_int("toolchange_count", 2)
        .set_int("flush_length_1", 40)
        .set_float("old_retract_length_toolchange", 2.0)
        .set_array(
            "flush_temperatures",
            vec![Value::Int(210), Value::Int(220), Value::Int(230), Value::Int(240)],
        )
        .set_array(
            "flush_volumetric_speeds",
            vec![
                Value::Float(12.0),
                Value::Float(12.0),
                Value::Float(12.0),
                Value::Float(12.0),
            ],
        )
        .set_array(
            "retraction_distances_when_cut",
            vec![Value::Float(18.0); 4],
        )
        .set_array(
            "long_retractions_when_cut",
            vec![Value::Bool(false); 4],
        )
        .set_array(
            "filament_type",
            vec![
                Value::Str("PLA".into()),
                Value::Str("PLA".into()),
                Value::Str("PETG".into()),
                Value::Str("PVA".into()),
            ],
        );
    c
}

#[test]
fn bracket_scalar_substitution() {
    let out = process("M620 S[next_extruder]A", &ctx());
    assert_eq!(out.trim(), "M620 S3A");
}

#[test]
fn brace_arithmetic() {
    let out = process("G1 Z{max_layer_z + 3.0} F1200", &ctx());
    assert_eq!(out.trim(), "G1 Z15.4 F1200");
}

#[test]
fn array_index_scalar() {
    // {flush_temperatures[next_extruder]} -> arr[3] = 240
    let out = process("M109 S{flush_temperatures[next_extruder]}", &ctx());
    assert_eq!(out.trim(), "M109 S240");
}

#[test]
fn array_index_with_arithmetic() {
    // flush_volumetric_speeds[previous_extruder]=12; 12/2.4053*60 = 299.34...
    let v = eval_expr("flush_volumetric_speeds[previous_extruder]/2.4053*60", &ctx()).unwrap();
    let f = match v {
        Value::Float(f) => f,
        other => panic!("expected float, got {:?}", other),
    };
    assert!((f - (12.0 / 2.4053 * 60.0)).abs() < 1e-6);
}

#[test]
fn conditional_and_comparison() {
    let tmpl = "{if old_filament_temp > 142 && next_extruder < 255}\nM104 S[old_filament_temp]\n{endif}";
    let out = process(tmpl, &ctx());
    assert_eq!(out.trim(), "M104 S220");
}

#[test]
fn conditional_false_skips() {
    let tmpl = "{if old_filament_temp > 500}\nSHOULD_NOT_APPEAR\n{endif}\nKEEP";
    let out = process(tmpl, &ctx());
    assert!(!out.contains("SHOULD_NOT_APPEAR"));
    assert!(out.contains("KEEP"));
}

#[test]
fn if_elsif_else_string_equality() {
    // filament_type[next_extruder] = "PVA" (index 3) -> the elsif branch
    let tmpl = "{if filament_type[next_extruder] == \"PETG\"}\nM109 S260\n{elsif filament_type[next_extruder] == \"PVA\"}\nM109 S210\n{else}\nM109 S{flush_temperatures[next_extruder]}\n{endif}";
    let out = process(tmpl, &ctx());
    assert_eq!(out.trim(), "M109 S210");
}

#[test]
fn nested_conditionals() {
    let tmpl = "{if flush_length_1 > 1}\nOUTER\n{if flush_length_1 > 23.7}\nINNER_BIG\n{else}\nINNER_SMALL\n{endif}\n{endif}";
    let out = process(tmpl, &ctx());
    assert!(out.contains("OUTER"));
    assert!(out.contains("INNER_BIG"));
    assert!(!out.contains("INNER_SMALL"));
}

#[test]
fn parenthesized_arithmetic() {
    // (flush_length_1 - 23.7) * 0.02 = (40-23.7)*0.02 = 0.326
    let out = process("G1 E{(flush_length_1 - 23.7) * 0.02} F50", &ctx());
    assert_eq!(out.trim(), "G1 E0.326 F50");
}

#[test]
fn negative_bracket_expr() {
    let out = process("G1 E-[old_retract_length_toolchange] F1800", &ctx());
    assert_eq!(out.trim(), "G1 E-2 F1800");
}

#[test]
fn unknown_variable_left_verbatim() {
    let out = process("M104 S[does_not_exist]", &ctx());
    assert_eq!(out.trim(), "M104 S[does_not_exist]");
}

#[test]
fn boolean_array_condition() {
    // long_retractions_when_cut[previous_extruder] = false -> else branch
    let tmpl = "{if long_retractions_when_cut[previous_extruder]}\nCUT\n{else}\nNOCUT\n{endif}";
    let out = process(tmpl, &ctx());
    assert_eq!(out.trim(), "NOCUT");
}
