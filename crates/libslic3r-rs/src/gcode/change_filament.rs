//! Build the variable [`Context`] a `change_filament_gcode` template needs at a
//! tool change — the data side of BambuStudio `WipeTowerIntegration::append_tcr`
//! (GCode.cpp:829-920). Paired with [`super::gcode_template`] (the evaluator),
//! this turns the config + a tool change into a concrete tool-change G-code block.
//!
//! Scope: fills the variables the Majora template actually references. Per-filament
//! arrays are filled UNIFORMLY from the (scalar) filament config — correct for
//! Majora (all filaments share one PLA profile) and any single-profile job; a
//! genuinely heterogeneous multi-filament job (H2D) would need per-filament arrays
//! (a later step). A few geometry variables are stubbed (see NOTEs) because they
//! only appear in branches that don't run for a normal tower tool change.

use crate::gcode::gcode_template::{Context, Value};
use crate::print_config::PrintConfig;
use std::f64::consts::PI;

// GCode.cpp:84-86
const G_MIN_PURGE_VOLUME: f64 = 100.0;
const G_PURGE_VOLUME_ONE_TIME: f64 = 135.0;
const G_MAX_FLUSH_COUNT: i32 = 4;

/// Build the change-filament template context for a `prev_tool → next_tool`
/// change at `layer_z` (with running `max_layer_z` and `toolchange_count`), given
/// the tower `purge_volume` (mm³) for this change.
#[allow(clippy::too_many_arguments)]
pub fn build_context(
    config: &PrintConfig,
    prev_tool: usize,
    next_tool: usize,
    layer_z: f64,
    max_layer_z: f64,
    toolchange_count: i64,
    purge_volume: f64,
    first_layer: bool,
) -> Context {
    let n = config.num_filaments().max(1);

    // GCode.cpp:895 — purge_volume is clamped: <eps → 0, else at least g_min.
    let purge_volume = if purge_volume < 1e-4 {
        0.0
    } else {
        purge_volume.max(G_MIN_PURGE_VOLUME)
    };
    let diameter = config.filament_diameters.first().copied().unwrap_or(1.75);
    let filament_area = (PI / 4.0) * diameter * diameter; // ≈2.405 for 1.75mm
    let purge_length = if filament_area > 0.0 {
        purge_volume / filament_area
    } else {
        0.0
    };

    // GCode.cpp:921-936 — split purge_length into `flush_count` equal segments.
    let mut flush_count = ((purge_volume / G_PURGE_VOLUME_ONE_TIME).round() as i32)
        .min(G_MAX_FLUSH_COUNT)
        .max(0);
    if flush_count == 0 && purge_volume > 0.0 {
        flush_count = 1;
    }
    let mut flush_unit = if flush_count > 0 {
        purge_length / flush_count as f64
    } else {
        0.0
    };
    // FLUSH_LENGTH_FLOAT (on): GCode.cpp:985-987, 1104-1108 compute in float:
    // `float purge_volume`, `float new_filament_area = float((M_PI/4.f) *
    // pow(d, 2))`, `float purge_length = purge_volume / new_filament_area`,
    // `std::round(purge_volume / g_purge_volume_one_time)` with float
    // constants and `float flush_unit = purge_length / flush_count`; the
    // template then reads the float as a double, and the sixth digit of
    // `{flush_length_2 * 0.18}` follows it. =0 computes in double.
    if crate::faithful_gate("FLUSH_LENGTH_FLOAT") {
        let pv = purge_volume as f32;
        let area = ((PI / 4.0) * diameter.powi(2)) as f32;
        let purge_length_f = if area > 0.0 { pv / area } else { 0.0 };
        let mut count = ((pv / G_PURGE_VOLUME_ONE_TIME as f32).round() as i32).min(G_MAX_FLUSH_COUNT);
        if count == 0 && pv > 0.0 {
            count += 1;
        }
        flush_count = count;
        flush_unit = if count > 0 { (purge_length_f / count as f32) as f64 } else { 0.0 };
    }

    let filament_temp = if first_layer {
        config.first_layer_extruder_temperature
    } else {
        config.extruder_temperature
    } as i64;
    // filament_flush_temp is 0 for Majora → fall back to the range-high temp.
    let flush_temp = config.nozzle_temperature_range_high as i64;
    let flush_vspeed = config.filament_max_volumetric_speed;
    let ftype = config.filament_type.clone();

    let mut c = Context::new();
    c.set_int("previous_extruder", prev_tool as i64)
        .set_int("next_extruder", next_tool as i64)
        .set_int("old_filament_temp", filament_temp)
        .set_int("new_filament_temp", filament_temp)
        .set_float("old_retract_length_toolchange", config.retract_length_toolchange)
        .set_float("new_retract_length_toolchange", config.retract_length_toolchange)
        .set_float("max_layer_z", max_layer_z)
        .set_float("layer_z", layer_z)
        .set_int("toolchange_count", toolchange_count)
        .set_float("initial_layer_print_height", config.first_layer_height)
        .set_int("initial_layer_acceleration", config.initial_layer_acceleration as i64)
        .set_int("default_acceleration", config.default_acceleration as i64)
        // Per-change flush segments.
        .set_float("flush_length_1", if flush_count >= 1 { flush_unit } else { 0.0 })
        .set_float("flush_length_2", if flush_count >= 2 { flush_unit } else { 0.0 })
        .set_float("flush_length_3", if flush_count >= 3 { flush_unit } else { 0.0 })
        .set_float("flush_length_4", if flush_count >= 4 { flush_unit } else { 0.0 });

    // Uniform per-filament arrays (single-profile job).
    c.set_array("flush_temperatures", vec![Value::Int(flush_temp); n])
        .set_array("flush_volumetric_speeds", vec![Value::Float(flush_vspeed); n])
        .set_array("filament_type", vec![Value::Str(ftype); n])
        .set_array(
            "retraction_distances_when_cut",
            vec![Value::Float(config.retraction_distances_when_cut); n],
        )
        // long_retractions_when_cut is 0 (off) for Majora.
        .set_array("long_retractions_when_cut", vec![Value::Bool(false); n]);

    // NOTE: geometry-only variables, stubbed. `x/y/z_after_toolchange` appear only
    // in the `{else}` (next_extruder==255) branch that a normal tower change never
    // takes; `travel_point_*` only when `toolchange_count == 2` (the very first
    // change) for a travel-avoidance path. Faithful values need the tower/object
    // approach geometry (append_tcr's tool_change_start_pos + travel path) — a
    // later refinement. Zero keeps the template resolvable meanwhile.
    for k in [
        "x_after_toolchange",
        "y_after_toolchange",
        "z_after_toolchange",
        "travel_point_1_x",
        "travel_point_1_y",
        "travel_point_2_x",
        "travel_point_2_y",
        "travel_point_3_x",
        "travel_point_3_y",
    ] {
        c.set_float(k, 0.0);
    }

    // GCode.cpp:1025 — `placeholder_parser().set("current_extruder", new_filament_id)`.
    // The stock `filament_start_gcode` selects a chamber-fan speed with
    // `bed_temperature[current_extruder]` / `bed_temperature_initial_layer[...]`,
    // so both the index and the two arrays must resolve or every branch of the
    // chain evaluates false and the block collapses to its comment (R495).
    // Our config carries one bed temperature rather than a per-filament vector,
    // so broadcast it across enough slots to index any filament.
    c.set_int("current_extruder", next_tool as i64);
    let slots = (prev_tool.max(next_tool) + 1).max(16);
    c.set_array(
        "bed_temperature",
        vec![Value::Int(config.bed_temperature as i64); slots],
    );
    c.set_array(
        "bed_temperature_initial_layer",
        vec![Value::Int(config.first_layer_bed_temperature as i64); slots],
    );

    c
}
