//! Cooling buffer slowdown calculations.
//!
//! C++ Reference:
//! - GCode/CoolingBuffer.hpp
//! - GCode/CoolingBuffer.cpp
//!
//! Faithful 1:1 port of `Slic3r::CoolingBuffer` and its free functions. This
//! calculates how much to slow down extrusion on a per-layer basis to meet
//! minimum layer time requirements, mirroring BambuStudio's algorithm exactly.

use crate::gcode::gcode_editor::{
    AdjustableFeatureType, CoolingLineType, CoolingSlowdownLogicType, PerExtruderAdjustments,
};

// EPSILON used throughout CoolingBuffer.cpp (Slic3r::EPSILON from libslic3r.h).
const EPSILON: f32 = 1e-4;

// CoolingBuffer.cpp:4
fn new_feedrate_to_reach_time_stretch(
    // [it_begin, it_end) is a const range into a vector of PerExtruderAdjustments*;
    // this routine only reads, so we take a vector of shared references in the same order.
    range: &[&PerExtruderAdjustments],
    mut min_feedrate: f32,
    time_stretch: f32,
    max_iter: usize, // CoolingBuffer.cpp:8 (default 20, always passed explicitly here)
) -> f32 {
    // CoolingBuffer.cpp:10
    let mut new_feedrate = min_feedrate;
    // CoolingBuffer.cpp:11
    for _iter in 0..max_iter {
        // CoolingBuffer.cpp:12
        let mut nomin: f64 = 0.0;
        // CoolingBuffer.cpp:13
        let mut denom: f64 = time_stretch as f64;
        // CoolingBuffer.cpp:14
        for it in range.iter() {
            // CoolingBuffer.cpp:15
            debug_assert!(it.slow_down_min_speed < min_feedrate + EPSILON);
            // CoolingBuffer.cpp:16
            for i in 0..it.n_lines_adjustable {
                // CoolingBuffer.cpp:17
                let line = &it.lines[i];
                // CoolingBuffer.cpp:18
                if line.feedrate > min_feedrate {
                    // CoolingBuffer.cpp:19
                    nomin += line.time as f64 * line.feedrate as f64;
                    // CoolingBuffer.cpp:20
                    denom += line.time as f64;
                }
            }
        }
        // CoolingBuffer.cpp:24
        debug_assert!(denom > 0.0);
        // CoolingBuffer.cpp:25
        if denom < 0.0 {
            return min_feedrate;
        }
        // CoolingBuffer.cpp:26
        new_feedrate = (nomin / denom) as f32;
        // CoolingBuffer.cpp:27
        debug_assert!(new_feedrate > min_feedrate - EPSILON);
        // CoolingBuffer.cpp:28
        if new_feedrate < min_feedrate + EPSILON {
            // goto finished;
            return new_feedrate;
        }
        // CoolingBuffer.cpp:29-38
        // Some of the line segments taken into account in the calculation of nomin / denom are now slower than new_feedrate,
        // which makes the new_feedrate lower than it should be.
        // Re-run the calculation with a new min_feedrate limit, so that the segments with current feedrate lower than new_feedrate
        // are not taken into account.
        let mut not_finished_yet = false;
        'outer: for it in range.iter() {
            for i in 0..it.n_lines_adjustable {
                let line = &it.lines[i];
                if line.feedrate > min_feedrate && line.feedrate < new_feedrate {
                    not_finished_yet = true;
                    break 'outer;
                }
            }
        }
        if !not_finished_yet {
            // CoolingBuffer.cpp:39 goto finished;
            return new_feedrate;
        }
        // CoolingBuffer.cpp:40-41 not_finished_yet:
        min_feedrate = new_feedrate;
    }
    // CoolingBuffer.cpp:42-43
    // Failed to find the new feedrate for the time_stretch.

    // CoolingBuffer.cpp:45 finished:
    // Test whether the time_stretch was achieved.
    // (NDEBUG block at CoolingBuffer.cpp:47-53 is a debug-only assertion check.)

    // CoolingBuffer.cpp:55
    new_feedrate
}

// CoolingBuffer.cpp:57-58
// Slow down an extruder range proportionally down to slow_down_layer_time.
// Return the total time for the complete layer.
// CoolingBuffer.cpp:59
fn extruder_range_slow_down_proportional(
    range: &mut [&mut PerExtruderAdjustments],
    // Elapsed time for the extruders already processed. // CoolingBuffer.cpp:62
    elapsed_time_total0: f32,
    // Initial total elapsed time before slow down. // CoolingBuffer.cpp:64
    elapsed_time_before_slowdown: f32,
    // Target time for the complete layer (all extruders applied). // CoolingBuffer.cpp:66
    slow_down_layer_time: f32,
) -> f32 {
    // CoolingBuffer.cpp:68-69
    // Total layer time after the slow down has been applied.
    let mut total_after_slowdown = elapsed_time_before_slowdown;
    // CoolingBuffer.cpp:70-71
    // Now decide, whether the external perimeters shall be slowed down as well.
    let mut max_time_nep = elapsed_time_total0;
    // CoolingBuffer.cpp:72
    for it in range.iter() {
        max_time_nep += it.maximum_time_after_slowdown(false);
    }
    // CoolingBuffer.cpp:73
    if max_time_nep > slow_down_layer_time {
        // CoolingBuffer.cpp:74-75
        // It is sufficient to slow down the non-external perimeter moves to reach the target layer time.
        // Slow down the non-external perimeters proportionally.
        // CoolingBuffer.cpp:76
        let mut non_adjustable_time = elapsed_time_total0;
        // CoolingBuffer.cpp:77
        for it in range.iter() {
            non_adjustable_time += it.non_adjustable_time(false);
        }
        // CoolingBuffer.cpp:78-79
        // The following step is a linear programming task due to the minimum movement speeds of the print moves.
        // Run maximum 5 iterations until a good enough approximation is reached.
        // CoolingBuffer.cpp:80
        for _iter in 0..5 {
            // CoolingBuffer.cpp:81
            let factor =
                (slow_down_layer_time - non_adjustable_time) / (total_after_slowdown - non_adjustable_time);
            // CoolingBuffer.cpp:82
            debug_assert!(factor > 1.0);
            // CoolingBuffer.cpp:83
            total_after_slowdown = elapsed_time_total0;
            // CoolingBuffer.cpp:84
            for it in range.iter_mut() {
                total_after_slowdown += it.slow_down_proportional(factor, false);
            }
            // CoolingBuffer.cpp:85
            if total_after_slowdown > 0.95 * slow_down_layer_time {
                break;
            }
        }
    } else {
        // CoolingBuffer.cpp:87-88
        // Slow down everything. First slow down the non-external perimeters to maximum.
        // CoolingBuffer.cpp:89
        for it in range.iter_mut() {
            it.slowdown_to_minimum_feedrate(false);
        }
        // CoolingBuffer.cpp:90
        // Slow down the external perimeters proportionally.
        // CoolingBuffer.cpp:91
        let mut non_adjustable_time = elapsed_time_total0;
        // CoolingBuffer.cpp:92
        for it in range.iter() {
            non_adjustable_time += it.non_adjustable_time(true);
        }
        // CoolingBuffer.cpp:93
        for _iter in 0..5 {
            // CoolingBuffer.cpp:94
            let factor =
                (slow_down_layer_time - non_adjustable_time) / (total_after_slowdown - non_adjustable_time);
            // CoolingBuffer.cpp:95
            debug_assert!(factor > 1.0);
            // CoolingBuffer.cpp:96
            total_after_slowdown = elapsed_time_total0;
            // CoolingBuffer.cpp:97
            for it in range.iter_mut() {
                total_after_slowdown += it.slow_down_proportional(factor, true);
            }
            // CoolingBuffer.cpp:98
            if total_after_slowdown > 0.95 * slow_down_layer_time {
                break;
            }
        }
    }
    // CoolingBuffer.cpp:101
    total_after_slowdown
}

// CoolingBuffer.cpp:104-107
// Slow down an extruder range for ConsistentSurface logic.
// This function first tries to slow down only non-visible features (infill, internal perimeters),
// and only slows down external perimeters if more time is needed.
// Returns the remaining time stretch that couldn't be achieved.
// CoolingBuffer.cpp:108
fn extruder_range_slow_down_consistent_surface(
    range: &mut [&mut PerExtruderAdjustments],
    mut time_stretch: f32,
    additional_slowdown_features: AdjustableFeatureType,
) -> f32 {
    // CoolingBuffer.cpp:114
    if time_stretch <= 0.0 {
        return 0.0;
    }

    // CoolingBuffer.cpp:117-118
    // Slow down. Try to equalize the feedrates for the allowed feature types.
    // by_min_print_speed is a vector of pointers into [it_begin, it_end). We track
    // the underlying entries by index into `range` so we can mutate them.
    let mut by_min_print_speed: Vec<usize> = (0..range.len()).collect();

    // CoolingBuffer.cpp:120-121
    // Find the highest adjustable feedrate among the extruders for allowed features.
    let mut feedrate = 0.0f32;
    // CoolingBuffer.cpp:122
    for &idx in by_min_print_speed.iter() {
        let adj = &mut *range[idx];
        // CoolingBuffer.cpp:123
        adj.idx_line_begin = 0;
        // CoolingBuffer.cpp:124
        adj.idx_line_end = 0;
        // CoolingBuffer.cpp:125
        for i in 0..adj.n_lines_adjustable {
            // CoolingBuffer.cpp:126
            let line = &adj.lines[i];
            // CoolingBuffer.cpp:127-128
            if line.adjustable_with_features(additional_slowdown_features) && line.feedrate > feedrate {
                feedrate = line.feedrate;
            }
        }
    }

    // CoolingBuffer.cpp:132-133
    if feedrate == 0.0 {
        return time_stretch; // No adjustable features found
    }

    // CoolingBuffer.cpp:135-139
    // Sort by slow_down_min_speed, maximum speed first.
    by_min_print_speed.sort_by(|&a, &b| {
        range[b]
            .slow_down_min_speed
            .partial_cmp(&range[a].slow_down_min_speed)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // CoolingBuffer.cpp:141-142
    // Slow down, fast moves first.
    let mut adj_pos = 0usize;
    while adj_pos < by_min_print_speed.len() {
        // CoolingBuffer.cpp:143
        let feedrate_limit = range[by_min_print_speed[adj_pos]].slow_down_min_speed;
        // CoolingBuffer.cpp:144
        let mut time_stretch_max = 0.0f32;

        // CoolingBuffer.cpp:146-147
        for &idx in by_min_print_speed[adj_pos..].iter() {
            time_stretch_max += range[idx]
                .time_stretch_when_slowing_down_to_feedrate_features(feedrate_limit, additional_slowdown_features);
        }

        // CoolingBuffer.cpp:149
        if time_stretch_max >= time_stretch {
            // CoolingBuffer.cpp:150-151
            // We can achieve the required time stretch by slowing down to some feedrate above feedrate_limit
            // Binary search for the right feedrate
            // CoolingBuffer.cpp:152
            let mut feedrate_high = feedrate;
            // CoolingBuffer.cpp:153
            let mut feedrate_low = feedrate_limit;
            // CoolingBuffer.cpp:154
            for _iter in 0..20 {
                // CoolingBuffer.cpp:155
                let feedrate_mid = (feedrate_high + feedrate_low) / 2.0;
                // CoolingBuffer.cpp:156
                let mut stretch = 0.0f32;
                // CoolingBuffer.cpp:157-158
                for &idx in by_min_print_speed[adj_pos..].iter() {
                    stretch += range[idx].time_stretch_when_slowing_down_to_feedrate_features(
                        feedrate_mid,
                        additional_slowdown_features,
                    );
                }
                // CoolingBuffer.cpp:159-162
                if stretch < time_stretch {
                    feedrate_high = feedrate_mid;
                } else {
                    feedrate_low = feedrate_mid;
                }
                // CoolingBuffer.cpp:163-164
                if (stretch - time_stretch).abs() < 0.01 {
                    break;
                }
            }
            // CoolingBuffer.cpp:166-167
            for k in adj_pos..by_min_print_speed.len() {
                let idx = by_min_print_speed[k];
                range[idx].slow_down_to_feedrate_features(feedrate_low, additional_slowdown_features);
            }
            // CoolingBuffer.cpp:168
            return 0.0; // Time stretch achieved
        } else {
            // CoolingBuffer.cpp:169-170
            // Slow down to minimum for these features
            // CoolingBuffer.cpp:171
            time_stretch -= time_stretch_max;
            // CoolingBuffer.cpp:172-173
            for k in adj_pos..by_min_print_speed.len() {
                let idx = by_min_print_speed[k];
                range[idx].slow_down_to_feedrate_features(feedrate_limit, additional_slowdown_features);
            }
        }

        // CoolingBuffer.cpp:176-180
        // Skip extruders with nearly the same slow_down_min_speed
        let adj_speed = range[by_min_print_speed[adj_pos]].slow_down_min_speed;
        let mut next = adj_pos + 1;
        while next < by_min_print_speed.len()
            && range[by_min_print_speed[next]].slow_down_min_speed > adj_speed - EPSILON
        {
            next += 1;
        }
        adj_pos = next;
    }

    // CoolingBuffer.cpp:183
    time_stretch // Return remaining time stretch that couldn't be achieved
}

// CoolingBuffer.cpp:186-187
// Slow down an extruder range to slow_down_layer_time.
// Return the total time for the complete layer.
// CoolingBuffer.cpp:188
fn extruder_range_slow_down_non_proportional(
    range: &mut [&mut PerExtruderAdjustments],
    mut time_stretch: f32,
) {
    // CoolingBuffer.cpp:192-193
    // Slow down. Try to equalize the feedrates.
    let mut by_min_print_speed: Vec<usize> = (0..range.len()).collect();
    // CoolingBuffer.cpp:194-195
    // Find the next highest adjustable feedrate among the extruders.
    let mut feedrate = 0.0f32;
    // CoolingBuffer.cpp:196
    for &idx in by_min_print_speed.iter() {
        let adj = &mut *range[idx];
        // CoolingBuffer.cpp:197
        adj.idx_line_begin = 0;
        // CoolingBuffer.cpp:198
        adj.idx_line_end = 0;
        // CoolingBuffer.cpp:199
        debug_assert!(adj.idx_line_begin < adj.n_lines_adjustable);
        // CoolingBuffer.cpp:200
        if adj.lines[adj.idx_line_begin].feedrate > feedrate {
            feedrate = adj.lines[adj.idx_line_begin].feedrate;
        }
    }
    // CoolingBuffer.cpp:202
    debug_assert!(feedrate > 0.0);
    // CoolingBuffer.cpp:203-205
    // Sort by slow_down_min_speed, maximum speed first.
    by_min_print_speed.sort_by(|&a, &b| {
        range[b]
            .slow_down_min_speed
            .partial_cmp(&range[a].slow_down_min_speed)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    // CoolingBuffer.cpp:206-207
    // Slow down, fast moves first.
    loop {
        // CoolingBuffer.cpp:208-213
        // For each extruder, find the span of lines with a feedrate close to feedrate.
        for &idx in by_min_print_speed.iter() {
            let adj = &mut *range[idx];
            adj.idx_line_end = adj.idx_line_begin;
            while adj.idx_line_end < adj.n_lines_adjustable
                && adj.lines[adj.idx_line_end].feedrate > feedrate - EPSILON
            {
                adj.idx_line_end += 1;
            }
        }
        // CoolingBuffer.cpp:214-217
        // Find the next highest adjustable feedrate among the extruders.
        let mut feedrate_next = 0.0f32;
        for &idx in by_min_print_speed.iter() {
            let adj = &*range[idx];
            if adj.idx_line_end < adj.n_lines_adjustable
                && adj.lines[adj.idx_line_end].feedrate > feedrate_next
            {
                feedrate_next = adj.lines[adj.idx_line_end].feedrate;
            }
        }
        // CoolingBuffer.cpp:218-219
        // Slow down, limited by max(feedrate_next, slow_down_min_speed).
        let mut adj_pos = 0usize;
        while adj_pos < by_min_print_speed.len() {
            // CoolingBuffer.cpp:220-221
            // Slow down at most by time_stretch.
            if range[by_min_print_speed[adj_pos]].slow_down_min_speed == 0.0 {
                // CoolingBuffer.cpp:222-223
                // All the adjustable speeds are now lowered to the same speed,
                // and the minimum speed is set to zero.
                // CoolingBuffer.cpp:224
                let mut time_adjustable = 0.0f32;
                // CoolingBuffer.cpp:225
                for &idx in by_min_print_speed[adj_pos..].iter() {
                    time_adjustable += range[idx].adjustable_time(true);
                }
                // CoolingBuffer.cpp:226
                let rate = (time_adjustable + time_stretch) / time_adjustable;
                // CoolingBuffer.cpp:227
                for k in adj_pos..by_min_print_speed.len() {
                    let idx = by_min_print_speed[k];
                    range[idx].slow_down_proportional(rate, true);
                }
                // CoolingBuffer.cpp:228
                return;
            } else {
                // CoolingBuffer.cpp:230
                let mut feedrate_limit = feedrate_next.max(range[by_min_print_speed[adj_pos]].slow_down_min_speed);
                // CoolingBuffer.cpp:231
                let mut done = false;
                // CoolingBuffer.cpp:232
                let mut time_stretch_max = 0.0f32;
                // CoolingBuffer.cpp:233
                for &idx in by_min_print_speed[adj_pos..].iter() {
                    time_stretch_max += range[idx].time_stretch_when_slowing_down_to_feedrate(feedrate_limit);
                }
                // CoolingBuffer.cpp:234
                if time_stretch_max >= time_stretch {
                    // CoolingBuffer.cpp:235
                    // The const range [adj, by_min_print_speed.end()) follows the
                    // by_min_print_speed sort order, which differs from `range`'s
                    // physical order; reconstruct it explicitly as shared refs.
                    let const_range: Vec<&PerExtruderAdjustments> = by_min_print_speed[adj_pos..]
                        .iter()
                        .map(|&idx| &*range[idx])
                        .collect();
                    feedrate_limit = new_feedrate_to_reach_time_stretch(
                        &const_range,
                        feedrate_limit,
                        time_stretch,
                        20,
                    );
                    // CoolingBuffer.cpp:236
                    done = true;
                } else {
                    // CoolingBuffer.cpp:238
                    time_stretch -= time_stretch_max;
                }
                // CoolingBuffer.cpp:239
                for k in adj_pos..by_min_print_speed.len() {
                    let idx = by_min_print_speed[k];
                    range[idx].slow_down_to_feedrate(feedrate_limit);
                }
                // CoolingBuffer.cpp:240
                if done {
                    return;
                }
            }
            // CoolingBuffer.cpp:242-245
            // Skip the other extruders with nearly the same slow_down_min_speed, as they have been processed already.
            let adj_speed = range[by_min_print_speed[adj_pos]].slow_down_min_speed;
            let mut next = adj_pos + 1;
            while next < by_min_print_speed.len()
                && range[by_min_print_speed[next]].slow_down_min_speed > adj_speed - EPSILON
            {
                next += 1;
            }
            adj_pos = next;
        }
        // CoolingBuffer.cpp:248-250
        if feedrate_next == 0.0 {
            // There are no other extrusions available for slow down.
            break;
        }
        // CoolingBuffer.cpp:251-254
        for &idx in by_min_print_speed.iter() {
            range[idx].idx_line_begin = range[idx].idx_line_end;
        }
        feedrate = feedrate_next;
    }
}

/// Cooling buffer that manages per-layer slowdown calculations.
/// Corresponds to C++ CoolingBuffer. // CoolingBuffer.hpp:8
#[derive(Debug, Clone)]
pub struct CoolingBuffer {
    // Old logic: proportional. // CoolingBuffer.hpp:16-17
    cooling_logic_proportional: bool,
}

impl CoolingBuffer {
    // CoolingBuffer.hpp:11
    pub fn new() -> Self {
        Self {
            cooling_logic_proportional: false,
        }
    }

    // CoolingBuffer.cpp:258-259
    // Calculate slow down for all the extruders.
    pub fn calculate_layer_slowdown(
        &self,
        per_extruder_adjustments: &mut [PerExtruderAdjustments],
    ) -> f32 {
        // CoolingBuffer.cpp:261-264
        // Sort the extruders by an increasing slow_down_layer_time.
        // The layers with a lower slow_down_layer_time are slowed down
        // together with all the other layers with slow_down_layer_time above.
        // by_slowdown_time holds indices into per_extruder_adjustments (C++ used pointers).
        let mut by_slowdown_time: Vec<usize> = Vec::with_capacity(per_extruder_adjustments.len());
        // CoolingBuffer.cpp:266-268
        // Only insert entries, which are adjustable (have cooling enabled and non-zero stretchable time).
        // Collect total print time of non-adjustable extruders.
        let mut elapsed_time_total0 = 0.0f32;

        // CoolingBuffer.cpp:270-271
        // Check if any extruder uses ConsistentSurface logic
        let mut _any_consistent_surface = false;

        // CoolingBuffer.cpp:273
        for (adj_idx, adj) in per_extruder_adjustments.iter_mut().enumerate() {
            // CoolingBuffer.cpp:274-275
            // Current total time for this extruder.
            adj.time_total = adj.elapsed_time_total();
            // CoolingBuffer.cpp:276-277
            // Maximum time for this extruder, when all extrusion moves are slowed down to min_extrusion_speed.
            adj.time_maximum = adj.maximum_time_after_slowdown(true);
            // CoolingBuffer.cpp:278
            if adj.cooling_slow_down_enabled && !adj.lines.is_empty() {
                // CoolingBuffer.cpp:279
                by_slowdown_time.push(adj_idx);

                // CoolingBuffer.cpp:281-282
                // For ConsistentSurface logic, prepare the non-adjustable segments
                if adj.cooling_slowdown_logic == CoolingSlowdownLogicType::CslConsistentSurface {
                    // CoolingBuffer.cpp:283
                    _any_consistent_surface = true;
                    // CoolingBuffer.cpp:284-291
                    // Initialize adjustable fields for all lines
                    for line in &mut adj.lines {
                        if (line.line_type & CoolingLineType::ADJUSTABLE) != 0 {
                            line.adjustable_length = line.length;
                            line.adjustable_time = line.time;
                            line.adjustable_time_max = line.time_max;
                        }
                    }
                    // CoolingBuffer.cpp:292-293
                    // Create non-adjustable segments at the end of perimeter loops
                    adj.create_non_adjustable_segments(adj.cooling_perimeter_transition_distance);
                }

                // CoolingBuffer.cpp:296-298
                if !self.cooling_logic_proportional {
                    // sorts the lines, also sets adj.time_non_adjustable
                    adj.sort_lines_by_decreasing_feedrate();
                }
            } else {
                // CoolingBuffer.cpp:300
                elapsed_time_total0 += adj.elapsed_time_total();
            }
        }
        // CoolingBuffer.cpp:302-303
        by_slowdown_time.sort_by(|&a, &b| {
            per_extruder_adjustments[a]
                .slow_down_layer_time
                .partial_cmp(&per_extruder_adjustments[b].slow_down_layer_time)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // CoolingBuffer.cpp:305
        for cur_begin in 0..by_slowdown_time.len() {
            // CoolingBuffer.cpp:306
            // adj = *(*cur_begin)
            let adj_idx = by_slowdown_time[cur_begin];
            // CoolingBuffer.cpp:307-309
            // Calculate the current adjusted elapsed_time_total over the non-finalized extruders.
            let mut total = elapsed_time_total0;
            for &idx in by_slowdown_time[cur_begin..].iter() {
                total += per_extruder_adjustments[idx].time_total;
            }
            // CoolingBuffer.cpp:310
            let slow_down_layer_time = per_extruder_adjustments[adj_idx].slow_down_layer_time * 1.001;
            // CoolingBuffer.cpp:311-313
            if total > slow_down_layer_time {
                // The current total time is above the minimum threshold of the rest of the extruders, don't adjust anything.
            } else {
                // CoolingBuffer.cpp:314-316
                // Adjust this and all the following (higher m_config.slow_down_layer_time) extruders.
                // Sum maximum slow down time as if everything was slowed down including the external perimeters.
                let mut max_time = elapsed_time_total0;
                // CoolingBuffer.cpp:317
                for &idx in by_slowdown_time[cur_begin..].iter() {
                    max_time += per_extruder_adjustments[idx].time_maximum;
                }
                // CoolingBuffer.cpp:318
                if max_time > slow_down_layer_time {
                    // CoolingBuffer.cpp:319
                    let time_stretch = slow_down_layer_time - total;

                    // Build the mutable view [cur_begin, end) of the adjustments, as the
                    // free functions operate on `std::vector<PerExtruderAdjustments*>` ranges.
                    let adj_logic = per_extruder_adjustments[adj_idx].cooling_slowdown_logic;
                    let mut adj_ptrs: Vec<&mut PerExtruderAdjustments> =
                        collect_range_mut(per_extruder_adjustments, &by_slowdown_time[cur_begin..]);

                    // CoolingBuffer.cpp:321-322
                    // Check if this extruder uses ConsistentSurface logic
                    if adj_logic == CoolingSlowdownLogicType::CslConsistentSurface {
                        // CoolingBuffer.cpp:323-324
                        // ConsistentSurface: Two-phase slowdown
                        // Phase 1: Try slowing down only non-external perimeter features (infill, internal perimeters)
                        // CoolingBuffer.cpp:325-326
                        let remaining = extruder_range_slow_down_consistent_surface(
                            &mut adj_ptrs,
                            time_stretch,
                            AdjustableFeatureType::NONE,
                        );

                        // CoolingBuffer.cpp:328-329
                        // Phase 2: If still not enough time, allow external perimeter and first internal slowdown
                        if remaining > 0.0 {
                            // CoolingBuffer.cpp:330-332
                            extruder_range_slow_down_consistent_surface(
                                &mut adj_ptrs,
                                remaining,
                                AdjustableFeatureType::EXTERNAL_PERIMETERS
                                    | AdjustableFeatureType::FIRST_INTERNAL_PERIMETERS,
                            );
                        }
                    } else if self.cooling_logic_proportional {
                        // CoolingBuffer.cpp:334-336
                        // Uniform cooling with proportional slowdown
                        extruder_range_slow_down_proportional(
                            &mut adj_ptrs,
                            elapsed_time_total0,
                            total,
                            slow_down_layer_time,
                        );
                    } else {
                        // CoolingBuffer.cpp:337-339
                        // Uniform cooling with non-proportional slowdown
                        extruder_range_slow_down_non_proportional(&mut adj_ptrs, time_stretch);
                    }
                } else {
                    // CoolingBuffer.cpp:341-343
                    // Slow down to maximum possible.
                    for &idx in by_slowdown_time[cur_begin..].iter() {
                        per_extruder_adjustments[idx].slowdown_to_minimum_feedrate(true);
                    }
                }
            }
            // CoolingBuffer.cpp:346
            elapsed_time_total0 += per_extruder_adjustments[adj_idx].elapsed_time_total();
        }

        // CoolingBuffer.cpp:349
        elapsed_time_total0
    }
}

impl Default for CoolingBuffer {
    fn default() -> Self {
        Self::new()
    }
}

// Helper: gather a vector of unique mutable references to the slice elements
// addressed by `indices`. Mirrors building a `std::vector<PerExtruderAdjustments*>`
// over the [cur_begin, end) range of the sort order. `indices` must contain
// distinct entries (they always do here: they are a sorted permutation suffix).
fn collect_range_mut<'a>(
    slice: &'a mut [PerExtruderAdjustments],
    indices: &[usize],
) -> Vec<&'a mut PerExtruderAdjustments> {
    let base = slice.as_mut_ptr();
    let mut out: Vec<&mut PerExtruderAdjustments> = Vec::with_capacity(indices.len());
    for &i in indices {
        debug_assert!(i < slice.len());
        // SAFETY: `indices` are distinct and in-bounds, so the produced &mut
        // references are non-aliasing and live for the borrow of `slice`.
        unsafe {
            out.push(&mut *base.add(i));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::gcode_editor::{CoolingLine, CoolingLineType};

    fn make_adjustable_line(time: f32, time_max: f32, length: f32, feedrate: f32) -> CoolingLine {
        let mut line = CoolingLine::new(CoolingLineType::G1 | CoolingLineType::ADJUSTABLE, 0, 10);
        line.time = time;
        line.time_max = time_max;
        line.length = length;
        line.feedrate = feedrate;
        line
    }

    #[test]
    fn test_cooling_buffer_new() {
        let buf = CoolingBuffer::new();
        assert!(!buf.cooling_logic_proportional);
    }

    #[test]
    fn test_calculate_layer_slowdown_empty() {
        let buf = CoolingBuffer::new();
        let time = buf.calculate_layer_slowdown(&mut []);
        assert_eq!(time, 0.0);
    }

    #[test]
    fn test_calculate_layer_slowdown_no_cooling() {
        // Cooling disabled: time accumulates into elapsed_time_total0 untouched.
        let buf = CoolingBuffer::new();
        let mut adj = PerExtruderAdjustments::new();
        adj.cooling_slow_down_enabled = false;
        adj.lines.push(make_adjustable_line(2.0, 5.0, 20.0, 10.0));

        let time = buf.calculate_layer_slowdown(&mut [adj]);
        assert!((time - 2.0).abs() < 1e-5);
    }

    #[test]
    fn test_calculate_layer_slowdown_slows_to_target() {
        // Single extruder, fast line, large slow_down_layer_time forces slowdown.
        let buf = CoolingBuffer::new();
        let mut adj = PerExtruderAdjustments::new();
        adj.cooling_slow_down_enabled = true;
        adj.slow_down_layer_time = 10.0;
        adj.slow_down_min_speed = 1.0;
        // line: time 2s, max 10s (length 20mm, feedrate 10mm/s -> min 2mm/s)
        adj.lines
            .push(make_adjustable_line(2.0, 20.0, 20.0, 10.0));

        let time = buf.calculate_layer_slowdown(&mut [adj]);
        // It should slow down toward (but not necessarily exactly) the target.
        assert!(time >= 2.0);
    }
}
