//! GCode/CoolingBuffer.cpp — the slow-down solver of a layer.
//!
//! `CoolingBuffer::calculate_layer_slowdown` (CoolingBuffer.cpp:259-350) and the
//! four functions it is built from: `new_feedrate_to_reach_time_stretch` (4-49),
//! `extruder_range_slow_down_proportional` (59-102),
//! `extruder_range_slow_down_consistent_surface` (108-186) and
//! `extruder_range_slow_down_non_proportional` (188-256).
//!
//! The per-extruder data it works on is `PerExtruderAdjustments` of
//! GCodeEditor.hpp, which the G-code editor in `cooling.rs` still keeps as
//! `PostProcAdjustments`; this file is compiled as a child of that module until
//! the editor moves to `g_code_editor.rs`.

use super::{CoolingLine, PostProcAdjustments, EPSILON};

/// CoolingBuffer.hpp:8-18
pub(super) struct CoolingBuffer {
    /// CoolingBuffer.hpp:17 — "Old logic: proportional."
    pub(super) m_cooling_logic_proportional: bool,
}

impl CoolingBuffer {
    /// CoolingBuffer.cpp:259-350 — slow down an extruder range down to
    /// slow_down_layer_time; returns the total time for the complete layer.
    pub(super) fn calculate_layer_slowdown(
        &self,
        per_extruder_adjustments: &mut [PostProcAdjustments],
    ) -> f32 {
        let cooling_logic_proportional = self.m_cooling_logic_proportional;
        static COOLPROBE_LAYER: std::sync::atomic::AtomicI32 = std::sync::atomic::AtomicI32::new(-1);
        let probe_layer = COOLPROBE_LAYER.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
        let cp = crate::probe_enabled("COOLPROBE_SOLVER");
        let cpl: Option<i32> = crate::env_f64("COOLPROBE_LINES").map(|v| v as i32);
        if cp {
            for adj in per_extruder_adjustments.iter() {
                let tt: f32 = adj.lines.iter().map(|l| l.time).sum();
                eprintln!(
                    "CPSOLVE li={} ex={} en={} n={} tt={:.6} target={:.4}",
                    probe_layer,
                    adj.extruder_id,
                    adj.cooling_slow_down_enabled as i32,
                    adj.lines.len(),
                    tt,
                    adj.slow_down_layer_time
                );
                if cpl == Some(probe_layer) {
                    for (i, l) in adj.lines.iter().enumerate() {
                        eprintln!(
                            "CPLINE li={} i={} type={:x} len={:.5} f={:.4} t={:.6} tmax={:.6}",
                            probe_layer, i, l.line_type, l.length, l.feedrate, l.time, l.time_max
                        );
                    }
                }
            }
        }
        // Sort extruders by slow_down_layer_time (increasing)
        // Reference: CoolingBuffer.cpp:259-350
        let mut by_slowdown_time: Vec<usize> = Vec::new();
        let mut elapsed_time_total0 = 0.0f32;

        for (idx, adj) in per_extruder_adjustments.iter_mut().enumerate() {
            adj.time_total = adj.elapsed_time_total();
            adj.time_maximum = adj.maximum_time_after_slowdown_bool(true);

            if adj.cooling_slow_down_enabled && !adj.lines.is_empty() {
                by_slowdown_time.push(idx);

                // For ConsistentSurface, prepare non-adjustable segments
                if adj.cooling_slowdown_logic == 1 {
                    for line in &mut adj.lines {
                        if (line.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0 {
                            line.adjustable_length = line.length;
                            line.adjustable_time = line.time;
                            line.adjustable_time_max = line.time_max;
                        }
                    }
                    let dist = adj.cooling_perimeter_transition_distance;
                    adj.create_non_adjustable_segments(dist);
                }

                if !cooling_logic_proportional {
                    adj.sort_lines_by_decreasing_feedrate();
                }
            } else {
                elapsed_time_total0 += adj.elapsed_time_total();
            }
        }

        // Sort by increasing slow_down_layer_time
        by_slowdown_time.sort_by(|&a, &b| {
            per_extruder_adjustments[a]
                .slow_down_layer_time
                .partial_cmp(&per_extruder_adjustments[b].slow_down_layer_time)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for cur_begin in 0..by_slowdown_time.len() {
            let begin_idx = by_slowdown_time[cur_begin];
            let mut total = elapsed_time_total0;
            for &idx in &by_slowdown_time[cur_begin..] {
                total += per_extruder_adjustments[idx].time_total;
            }

            let slow_down_layer_time = per_extruder_adjustments[begin_idx].slow_down_layer_time * 1.001;

            if total > slow_down_layer_time {
                // No adjustment needed for this tier
            } else {
                let mut max_time = elapsed_time_total0;
                for &idx in &by_slowdown_time[cur_begin..] {
                    max_time += per_extruder_adjustments[idx].time_maximum;
                }

                if max_time > slow_down_layer_time {
                    let time_stretch = slow_down_layer_time - total;
                    let first_logic = per_extruder_adjustments[begin_idx].cooling_slowdown_logic;

                    if first_logic == 1 {
                        // ConsistentSurface: two-phase slowdown
                        // slow non-visible features
                        let remaining = extruder_range_slow_down_consistent_surface(
                            per_extruder_adjustments,
                            &by_slowdown_time[cur_begin..],
                            time_stretch,
                            0, // None: slow only non-visible features
                        );
                        // slow external + first internal perimeters
                        if remaining > 0.0 {
                            extruder_range_slow_down_consistent_surface(
                                per_extruder_adjustments,
                                &by_slowdown_time[cur_begin..],
                                remaining,
                                3, // ExternalPerimeters(1) | FirstInternalPerimeters(2)
                            );
                        }
                    } else if cooling_logic_proportional {
                        // Proportional slowdown
                        extruder_range_slow_down_proportional(
                            per_extruder_adjustments,
                            &by_slowdown_time[cur_begin..],
                            elapsed_time_total0,
                            total,
                            slow_down_layer_time,
                        );
                    } else {
                        // Non-proportional slowdown
                        extruder_range_slow_down_non_proportional(
                            per_extruder_adjustments,
                            &by_slowdown_time[cur_begin..],
                            time_stretch,
                        );
                    }
                } else {
                    // Slow everything to maximum
                    for &idx in &by_slowdown_time[cur_begin..] {
                        per_extruder_adjustments[idx].slowdown_to_minimum_feedrate_bool(true);
                    }
                }
            }

            elapsed_time_total0 += per_extruder_adjustments[begin_idx].elapsed_time_total();
        }

        // Sort all lines back by position for output
        for adj in per_extruder_adjustments.iter_mut() {
            adj.lines.sort_by_key(|l| l.line_start);
        }

        if cp {
            eprintln!("CPSOLVE-RES li={} t={:.6}", probe_layer, elapsed_time_total0);
        }
        if cpl == Some(probe_layer) {
            // R803 CPOUT: post-solve feedrates (cpp twin: same line)
            for adj in per_extruder_adjustments.iter() {
                for l in adj.lines.iter() {
                    eprintln!("CPOUT li={} pos={} type={:x} len={:.5} f={:.4} t={:.6}", probe_layer, l.line_start, l.line_type, l.length, l.feedrate, l.time);
                }
            }
        }
        elapsed_time_total0
    }
}

/// Proportional slowdown algorithm for post-processor.
/// Reference: CoolingBuffer.cpp:59-102
fn extruder_range_slow_down_proportional(
    adjustments: &mut [PostProcAdjustments],
    indices: &[usize],
    elapsed_time_total0: f32,
    elapsed_time_before_slowdown: f32,
    slow_down_layer_time: f32,
) -> f32 {
    let mut total_after = elapsed_time_before_slowdown;

    // Check if non-external perimeter slowdown is sufficient
    let mut max_time_nep = elapsed_time_total0;
    for &idx in indices {
        max_time_nep += adjustments[idx].maximum_time_after_slowdown_bool(false);
    }

    if max_time_nep > slow_down_layer_time {
        // Slow only non-external perimeters
        let mut non_adj_time = elapsed_time_total0;
        for &idx in indices {
            non_adj_time += adjustments[idx].non_adjustable_time_bool(false);
        }
        for _iter in 0..5 {
            let factor = (slow_down_layer_time - non_adj_time) / (total_after - non_adj_time);
            total_after = elapsed_time_total0;
            for &idx in indices {
                total_after += adjustments[idx].slow_down_proportional(factor, false);
            }
            if total_after > 0.95 * slow_down_layer_time {
                break;
            }
        }
    } else {
        // Slow everything: first max out non-external, then slow external proportionally
        for &idx in indices {
            adjustments[idx].slowdown_to_minimum_feedrate_bool(false);
        }
        let mut non_adj_time = elapsed_time_total0;
        for &idx in indices {
            non_adj_time += adjustments[idx].non_adjustable_time_bool(true);
        }
        for _iter in 0..5 {
            let factor = (slow_down_layer_time - non_adj_time) / (total_after - non_adj_time);
            total_after = elapsed_time_total0;
            for &idx in indices {
                total_after += adjustments[idx].slow_down_proportional(factor, true);
            }
            if total_after > 0.95 * slow_down_layer_time {
                break;
            }
        }
    }

    total_after
}

/// Iterative exact-stretch feedrate solver for the post-processor path.
/// Reference: CoolingBuffer.cpp:4-49 `new_feedrate_to_reach_time_stretch`.
/// nomin/denom are doubles in C++; the per-line products are float promoted.
fn new_feedrate_to_reach_time_stretch(
    adjustments: &[PostProcAdjustments],
    indices: &[usize],
    mut min_feedrate: f32,
    time_stretch: f32,
    max_iter: usize,
) -> f32 {
    let mut new_feedrate = min_feedrate;
    for _iter in 0..max_iter {
        let mut nomin = 0.0f64;
        let mut denom = time_stretch as f64;
        for &idx in indices {
            let adj = &adjustments[idx];
            for i in 0..adj.n_lines_adjustable {
                let line = &adj.lines[i];
                if line.feedrate > min_feedrate {
                    nomin += line.time as f64 * line.feedrate as f64;
                    denom += line.time as f64;
                }
            }
        }
        if denom < 0.0 {
            return min_feedrate;
        }
        new_feedrate = (nomin / denom) as f32;
        if new_feedrate < min_feedrate + EPSILON {
            return new_feedrate;
        }
        let mut not_finished = false;
        'scan: for &idx in indices {
            let adj = &adjustments[idx];
            for i in 0..adj.n_lines_adjustable {
                let line = &adj.lines[i];
                if line.feedrate > min_feedrate && line.feedrate < new_feedrate {
                    not_finished = true;
                    break 'scan;
                }
            }
        }
        if !not_finished {
            return new_feedrate;
        }
        min_feedrate = new_feedrate;
    }
    new_feedrate
}

/// Non-proportional slowdown for post-processor (equalize feedrates).
/// Reference: CoolingBuffer.cpp:188-256
fn extruder_range_slow_down_non_proportional(
    adjustments: &mut [PostProcAdjustments],
    indices: &[usize],
    mut time_stretch: f32,
) {
    // Sort by slow_down_min_speed descending
    let mut by_min_speed: Vec<usize> = indices.to_vec();
    by_min_speed.sort_by(|&a, &b| {
        adjustments[b]
            .slow_down_min_speed
            .partial_cmp(&adjustments[a].slow_down_min_speed)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Find highest adjustable feedrate
    let mut feedrate = 0.0f32;
    for &idx in &by_min_speed {
        adjustments[idx].idx_line_begin = 0;
        adjustments[idx].idx_line_end = 0;
        if adjustments[idx].n_lines_adjustable > 0 {
            let f = adjustments[idx].lines[0].feedrate;
            if f > feedrate {
                feedrate = f;
            }
        }
    }

    let mut loop_guard = 0;
    loop {
        loop_guard += 1;
        if loop_guard > 1000 {
            break; // Safety: prevent infinite loop
        }
        // Find span of lines with feedrate near current
        for &idx in &by_min_speed {
            let adj = &mut adjustments[idx];
            adj.idx_line_end = adj.idx_line_begin;
            while adj.idx_line_end < adj.n_lines_adjustable
                && adj.lines[adj.idx_line_end].feedrate > feedrate - EPSILON
            {
                adj.idx_line_end += 1;
            }
        }

        // Find next highest feedrate
        let mut feedrate_next = 0.0f32;
        for &idx in &by_min_speed {
            let adj = &adjustments[idx];
            if adj.idx_line_end < adj.n_lines_adjustable {
                let f = adj.lines[adj.idx_line_end].feedrate;
                if f > feedrate_next {
                    feedrate_next = f;
                }
            }
        }

        // Process each speed tier
        let mut tier_idx = 0;
        while tier_idx < by_min_speed.len() {
            let current_adj_idx = by_min_speed[tier_idx];
            let min_speed = adjustments[current_adj_idx].slow_down_min_speed;

            if min_speed == 0.0 {
                let mut time_adjustable = 0.0f32;
                for i in tier_idx..by_min_speed.len() {
                    time_adjustable += adjustments[by_min_speed[i]].adjustable_time_bool(true);
                }
                let rate = (time_adjustable + time_stretch) / time_adjustable;
                for i in tier_idx..by_min_speed.len() {
                    adjustments[by_min_speed[i]].slow_down_proportional(rate, true);
                }
                return;
            } else {
                let feedrate_limit = feedrate_next.max(min_speed);
                let mut time_stretch_max = 0.0f32;
                for i in tier_idx..by_min_speed.len() {
                    time_stretch_max += adjustments[by_min_speed[i]]
                        .time_stretch_when_slowing_down_to_feedrate(feedrate_limit);
                }
                if time_stretch_max >= time_stretch {
                    if crate::faithful_gate("COOLING_EXACT_STRETCH") {
                        // CoolingBuffer.cpp:191 — the final tier solves the
                        // exact feedrate via new_feedrate_to_reach_time_stretch
                        // (iterative weighted mean, double accumulators), NOT a
                        // tolerance-bounded binary search. The graft below
                        // overshot the target layer time by up to 0.21 s (rust
                        // hit 4.004 s on 6/300 layers vs native 190/300, R757).
                        let f = new_feedrate_to_reach_time_stretch(
                            adjustments,
                            &by_min_speed[tier_idx..],
                            feedrate_limit,
                            time_stretch,
                            20,
                        );
                        for i in tier_idx..by_min_speed.len() {
                            adjustments[by_min_speed[i]].slow_down_to_feedrate(f);
                        }
                        return;
                    }
                    // Binary search for exact feedrate
                    let mut f_low = feedrate_limit;
                    let mut f_high = feedrate;
                    for _ in 0..20 {
                        let f_mid = (f_low + f_high) / 2.0;
                        let mut s = 0.0f32;
                        for i in tier_idx..by_min_speed.len() {
                            s += adjustments[by_min_speed[i]]
                                .time_stretch_when_slowing_down_to_feedrate(f_mid);
                        }
                        if s < time_stretch {
                            f_high = f_mid;
                        } else {
                            f_low = f_mid;
                        }
                        if (s - time_stretch).abs() < 0.01 {
                            break;
                        }
                    }
                    for i in tier_idx..by_min_speed.len() {
                        adjustments[by_min_speed[i]].slow_down_to_feedrate(f_low);
                    }
                    return;
                } else {
                    time_stretch -= time_stretch_max;
                    for i in tier_idx..by_min_speed.len() {
                        adjustments[by_min_speed[i]].slow_down_to_feedrate(feedrate_limit);
                    }
                }
            }

            // Skip to next speed tier
            let current_min = adjustments[by_min_speed[tier_idx]].slow_down_min_speed;
            tier_idx += 1;
            while tier_idx < by_min_speed.len() {
                if adjustments[by_min_speed[tier_idx]].slow_down_min_speed < current_min - EPSILON {
                    break;
                }
                tier_idx += 1;
            }
        }

        if feedrate_next == 0.0 {
            break;
        }

        for &idx in &by_min_speed {
            adjustments[idx].idx_line_begin = adjustments[idx].idx_line_end;
        }
        feedrate = feedrate_next;
    }
}

/// ConsistentSurface two-phase slowdown for post-processor.
/// Reference: CoolingBuffer.cpp:108-184
fn extruder_range_slow_down_consistent_surface(
    adjustments: &mut [PostProcAdjustments],
    indices: &[usize],
    time_stretch: f32,
    additional_features: u32,
) -> f32 {
    if time_stretch <= 0.0 {
        return 0.0;
    }

    // Sort by slow_down_min_speed descending
    let mut by_min_speed: Vec<usize> = indices.to_vec();
    by_min_speed.sort_by(|&a, &b| {
        adjustments[b]
            .slow_down_min_speed
            .partial_cmp(&adjustments[a].slow_down_min_speed)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Find highest adjustable feedrate
    let mut feedrate = 0.0f32;
    for &idx in &by_min_speed {
        for i in 0..adjustments[idx].n_lines_adjustable {
            let line = &adjustments[idx].lines[i];
            if line.adjustable_for_features(additional_features) && line.feedrate > feedrate {
                feedrate = line.feedrate;
            }
        }
    }

    if feedrate == 0.0 {
        return time_stretch;
    }

    let mut remaining = time_stretch;
    let mut processed = 0;

    while processed < by_min_speed.len() {
        let feedrate_limit = adjustments[by_min_speed[processed]].slow_down_min_speed;

        let mut time_stretch_max = 0.0f32;
        for i in processed..by_min_speed.len() {
            time_stretch_max += adjustments[by_min_speed[i]]
                .time_stretch_when_slowing_down_to_feedrate_features(
                    feedrate_limit,
                    additional_features,
                );
        }

        if time_stretch_max >= remaining {
            // Binary search
            let mut f_high = feedrate;
            let mut f_low = feedrate_limit;
            for _iter in 0..20 {
                let f_mid = (f_high + f_low) / 2.0;
                let mut stretch = 0.0f32;
                for i in processed..by_min_speed.len() {
                    stretch += adjustments[by_min_speed[i]]
                        .time_stretch_when_slowing_down_to_feedrate_features(
                            f_mid,
                            additional_features,
                        );
                }
                if stretch < remaining {
                    f_high = f_mid;
                } else {
                    f_low = f_mid;
                }
                if (stretch - remaining).abs() < 0.01 {
                    break;
                }
            }
            for i in processed..by_min_speed.len() {
                adjustments[by_min_speed[i]]
                    .slow_down_to_feedrate_features(f_low, additional_features);
            }
            return 0.0;
        } else {
            remaining -= time_stretch_max;
            for i in processed..by_min_speed.len() {
                adjustments[by_min_speed[i]]
                    .slow_down_to_feedrate_features(feedrate_limit, additional_features);
            }
        }

        // Skip to next tier
        let current_min = adjustments[by_min_speed[processed]].slow_down_min_speed;
        processed += 1;
        while processed < by_min_speed.len() {
            if adjustments[by_min_speed[processed]].slow_down_min_speed < current_min - EPSILON {
                break;
            }
            processed += 1;
        }
    }

    remaining
}
