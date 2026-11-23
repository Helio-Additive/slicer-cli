// Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
// Original source of Thomas Rahm's tree supports:
// https://github.com/ThomasRahm/CuraEngine
//
// Original CuraEngine copyright:
// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

//! Port of BambuStudio's `src/libslic3r/Support/TreeSupport3D.{hpp,cpp}` (namespace
//! `Slic3r::TreeSupport3D`) at the pin, over the classes of `crate::print::faithful`, the settings of
//! `tree_support_common.rs` (TreeSupportCommon.hpp) and the volumes of `tree_model_volumes.rs`.
//!
//! The file has two parts:
//!  * the port of the pin: the types of TreeSupport3D.hpp (`AreaIncreaseSettings`,
//!    `SupportElementStateBits`, `SupportElementState`, `SupportElement`) and the functions of
//!    TreeSupport3D.cpp under their C++ names, each citing the lines it ports;
//!  * `legacy`: the generator the crate had before (`TreeSupport3D`, `TreeSupport3DConfig`,
//!    `TreeSupport3DResult` and the functions they run, over the types of `tree_support_settings.rs`),
//!    which `support/mod.rs` still drives. It is kept as it was so that what the engine puts out for a
//!    tree support does not change while the port of the pin is completed and compared.
//!
//! `coord_t` is `Coord` (i64), `LayerIndex` is `i32`, `Polygons` is `Vec<Polygon>`. A `tbb::parallel_for`
//! runs its items one after the other, in the order the pin takes them on one thread.

/// The generator the crate had before the port of the pin (see the head of the file): an earlier
/// TreeSupport3D.cpp over the types of `tree_support_settings.rs`, with tips sampled from the outlines of
/// the overhangs and circles for branches. `support/mod.rs` drives it.
pub mod legacy {
use crate::geometry::{
    self, area_polygons, contains_polygons, get_extents_polygons, make_circle, perp,
    polygons_simplify, BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polygons,
};
use crate::support::support_common::{safe_offset_inc, safe_union};
use crate::support::tree_model_volumes::{AvoidanceType as VolAvoidanceType, TreeModelVolumes};
use crate::support::tree_support_common::{tree_supports_show_error, LayerIndex};
use crate::support::tree_support_settings::{
    AreaIncreaseSettings, AvoidanceTypeCompact as AvoidanceType, LineStatus, SupportElement,
    SupportElementState, TreeSupportSettings,
};
use crate::libslic3r::SCALED_EPSILON;
use crate::utils::round_up_divide;
use crate::{scale, unscale, Coord, CoordF, SCALING_FACTOR};

// TreeSupport3D.cpp:67
pub type LineInformation = Vec<(Point, LineStatus)>;
// TreeSupport3D.cpp:68
pub type LineInformations = Vec<LineInformation>;

/// `using SupportElements = std::deque<SupportElement>;` (TreeSupport3D.hpp:282)
pub type SupportElements = Vec<SupportElement>;

/// `std::vector<SupportElements>` — all support elements indexed by layer.
pub type LayerSupportElements = Vec<SupportElements>;

// ----------------------------------------------------------------------------
// `AreaIncreaseSettings::avoidance_type` is the compact enum (Fast/FastSafe/Slow)
// while the TreeModelVolumes queries take its own `AvoidanceType`
// (Slow=0, FastSafe=1, Fast=2). Convert between them for `getAvoidance` calls.
// ----------------------------------------------------------------------------
#[inline]
fn vol_avoidance(t: AvoidanceType) -> VolAvoidanceType {
    match t {
        AvoidanceType::Fast => VolAvoidanceType::Fast,
        AvoidanceType::FastSafe => VolAvoidanceType::FastSafe,
        AvoidanceType::Slow => VolAvoidanceType::Slow,
    }
}

// TreeSupport3D.cpp:127
// static constexpr const auto tiny_area_threshold = sqr(scaled<double>(0.001));
fn tiny_area_threshold() -> CoordF {
    // scaled<double>(0.001): FloatingOnly overload (Point.hpp:527) = v / SCALING_FACTOR
    // with no rounding, kept as double.
    let s = 0.001 * SCALING_FACTOR; // == 100.0
    s * s
}

// ----------------------------------------------------------------------------
// ClipperUtils.hpp helpers used by this translation unit.
// ----------------------------------------------------------------------------

// `Polygons diff(const Polygons &subject, const Polygons &clip)`
fn diff(subject: &[Polygon], clip: &[Polygon]) -> Polygons {
    let subj: ExPolygons = subject.iter().map(|p| ExPolygon::new(p.clone())).collect();
    let cl: ExPolygons = clip.iter().map(|p| ExPolygon::new(p.clone())).collect();
    expolys_to_polygons(&crate::clipper_utils::difference(&subj, &cl))
}

// `Polygons union_(const Polygons &subject, const Polygons &subject2 = {})`
// Plain ClipperLib union (NOT safe_union — no safety offset applied).
fn union_(subject: &[Polygon], subject2: &[Polygon]) -> Polygons {
    let mut all: Polygons = subject.to_vec();
    all.extend_from_slice(subject2);
    expolys_to_polygons(&crate::clipper_utils::union_polygons_ex(&all))
}

// `Polygons intersection(const Polygons &subject, const Polygons &clip)`
fn intersection(subject: &[Polygon], clip: &[Polygon]) -> Polygons {
    let subj: ExPolygons = subject.iter().map(|p| ExPolygon::new(p.clone())).collect();
    let cl: ExPolygons = clip.iter().map(|p| ExPolygon::new(p.clone())).collect();
    expolys_to_polygons(&crate::clipper_utils::intersection(&subj, &cl))
}

// `Polygons diff_clipped(const Polygons &src, const Polygons &clipping)`
// In BambuStudio diff_clipped(a, b) == diff(a, b, ApplySafetyOffset::No) but it
// trims the clip to the subject's bbox for performance — the geometric result
// is identical to diff(a, b).
fn diff_clipped(src: &[Polygon], clipping: &[Polygon]) -> Polygons {
    diff(src, clipping)
}

// `double area(const Polygons &polys)`
#[inline]
fn area(polys: &[Polygon]) -> CoordF {
    area_polygons(polys)
}

// `bool contains(const Polygons &polygons, const Point &p)` — border_result = true.
#[inline]
fn contains(polygons: &[Polygon], p: &Point) -> bool {
    contains_polygons(polygons, p, true)
}

// `BoundingBox get_extents(const Polygons &polygons)`
#[inline]
fn get_extents(polygons: &[Polygon]) -> BoundingBox {
    get_extents_polygons(polygons)
}

fn expolys_to_polygons(src: &[ExPolygon]) -> Polygons {
    let mut out: Polygons = Vec::new();
    for ex in src {
        out.push(ex.contour.clone());
        for h in &ex.holes {
            out.push(h.clone());
        }
    }
    out
}

// ============================================================================
// TreeSupport3D.cpp:344-364 — get_avoidance_status
// picked from convert_lines_to_internal()
// ============================================================================
#[allow(clippy::too_many_arguments)]
pub fn get_avoidance_status(
    p: &Point,
    radius: Coord,
    layer_idx: LayerIndex,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
) -> LineStatus {
    // TreeSupport3D.cpp:348
    let min_xy_dist = config.xy_distance > config.xy_min_distance;

    // TreeSupport3D.cpp:350
    let mut type_ = LineStatus::Invalid;

    // TreeSupport3D.cpp:352
    if !contains(
        &volumes.get_avoidance_full(radius, layer_idx, VolAvoidanceType::FastSafe, false, min_xy_dist),
        p,
    ) {
        type_ = LineStatus::ToBuildPlateSafe;
    } else if !contains(
        // TreeSupport3D.cpp:354
        &volumes.get_avoidance_full(radius, layer_idx, VolAvoidanceType::Fast, false, min_xy_dist),
        p,
    ) {
        type_ = LineStatus::ToBuildPlate;
    } else if config.support_rests_on_model
        && !contains(
            // TreeSupport3D.cpp:356
            &volumes.get_avoidance_full(radius, layer_idx, VolAvoidanceType::FastSafe, true, min_xy_dist),
            p,
        )
    {
        type_ = LineStatus::ToModelGraciousSafe;
    } else if config.support_rests_on_model
        && !contains(
            // TreeSupport3D.cpp:358
            &volumes.get_avoidance_full(radius, layer_idx, VolAvoidanceType::Fast, true, min_xy_dist),
            p,
        )
    {
        type_ = LineStatus::ToModelGracious;
    } else if config.support_rests_on_model
        && !contains(
            // TreeSupport3D.cpp:360
            &volumes.get_collision_min_xy(radius, layer_idx, min_xy_dist),
            p,
        )
    {
        type_ = LineStatus::ToModel;
    }

    // TreeSupport3D.cpp:363
    type_
}

// ============================================================================
// TreeSupport3D.cpp:374-408 — convert_lines_to_internal
// ============================================================================
// Called by generate_initial_areas()
pub fn convert_lines_to_internal(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    polylines: &[Vec<Point>],
    layer_idx: LayerIndex,
) -> LineInformations {
    // TreeSupport3D.cpp:378
    let min_xy_dist = config.xy_distance > config.xy_min_distance;
    let r0 = config.get_radius(0, 0.0);

    // TreeSupport3D.cpp:380
    let mut result: LineInformations = Vec::new();
    // Also checks if the position is valid, if it is NOT, it deletes that point
    // TreeSupport3D.cpp:382
    for line in polylines {
        // TreeSupport3D.cpp:383
        let mut res_line: LineInformation = Vec::new();
        // TreeSupport3D.cpp:384
        for &p in line {
            // TreeSupport3D.cpp:385
            if !contains(
                &volumes.get_avoidance_full(r0, layer_idx, VolAvoidanceType::FastSafe, false, min_xy_dist),
                &p,
            ) {
                res_line.push((p, LineStatus::ToBuildPlateSafe));
            } else if !contains(
                // TreeSupport3D.cpp:387
                &volumes.get_avoidance_full(r0, layer_idx, VolAvoidanceType::Fast, false, min_xy_dist),
                &p,
            ) {
                res_line.push((p, LineStatus::ToBuildPlate));
            } else if config.support_rests_on_model
                && !contains(
                    // TreeSupport3D.cpp:389
                    &volumes.get_avoidance_full(r0, layer_idx, VolAvoidanceType::FastSafe, true, min_xy_dist),
                    &p,
                )
            {
                res_line.push((p, LineStatus::ToModelGraciousSafe));
            } else if config.support_rests_on_model
                && !contains(
                    // TreeSupport3D.cpp:391
                    &volumes.get_avoidance_full(r0, layer_idx, VolAvoidanceType::Fast, true, min_xy_dist),
                    &p,
                )
            {
                res_line.push((p, LineStatus::ToModelGracious));
            } else if config.support_rests_on_model
                && !contains(
                    // TreeSupport3D.cpp:393
                    &volumes.get_collision_min_xy(r0, layer_idx, min_xy_dist),
                    &p,
                )
            {
                res_line.push((p, LineStatus::ToModel));
            } else if !res_line.is_empty() {
                // TreeSupport3D.cpp:395
                result.push(res_line.clone());
                res_line.clear();
            }
        }
        // TreeSupport3D.cpp:400
        if !res_line.is_empty() {
            result.push(res_line.clone());
            res_line.clear();
        }
    }

    // TreeSupport3D.cpp:406-407
    result
}

// ============================================================================
// TreeSupport3D.cpp:437-452 — evaluate_point_for_next_layer_function
// ============================================================================
pub fn evaluate_point_for_next_layer_function(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    current_layer: usize,
    p: &(Point, LineStatus),
) -> bool {
    // TreeSupport3D.cpp:442
    let min_xy_dist = config.xy_distance > config.xy_min_distance;
    let r0 = config.get_radius(0, 0.0);
    // TreeSupport3D.cpp:443
    if !contains(
        &volumes.get_avoidance_full(
            r0,
            (current_layer as LayerIndex) - 1,
            if p.1 == LineStatus::ToBuildPlateSafe {
                VolAvoidanceType::FastSafe
            } else {
                VolAvoidanceType::Fast
            },
            false,
            min_xy_dist,
        ),
        &p.0,
    ) {
        return true;
    }
    // TreeSupport3D.cpp:445
    if config.support_rests_on_model
        && p.1 != LineStatus::ToBuildPlate
        && p.1 != LineStatus::ToBuildPlateSafe
    {
        // TreeSupport3D.cpp:446-450
        let forbidden = if p.1 == LineStatus::ToModelGracious || p.1 == LineStatus::ToModelGraciousSafe {
            volumes.get_avoidance_full(
                r0,
                (current_layer as LayerIndex) - 1,
                if p.1 == LineStatus::ToModelGraciousSafe {
                    VolAvoidanceType::FastSafe
                } else {
                    VolAvoidanceType::Fast
                },
                true,
                min_xy_dist,
            )
        } else {
            volumes.get_collision_min_xy(r0, (current_layer as LayerIndex) - 1, min_xy_dist)
        };
        return !contains(&forbidden, &p.0);
    }
    // TreeSupport3D.cpp:451
    false
}

// ============================================================================
// TreeSupport3D.cpp:461-485 — split_lines
// ============================================================================
// Returns (keep, set_free).
pub fn split_lines<F>(lines: &LineInformations, evaluate_point: F) -> (LineInformations, LineInformations)
where
    F: Fn(&(Point, LineStatus)) -> bool,
{
    // assumes all Points on the current line are valid
    // TreeSupport3D.cpp:466-467
    let mut keep: LineInformations = Vec::new();
    let mut set_free: LineInformations = Vec::new();
    // TreeSupport3D.cpp:468
    for line in lines {
        // TreeSupport3D.cpp:469
        let mut current_keep = true;
        // TreeSupport3D.cpp:470
        let mut resulting_line: LineInformation = Vec::new();
        // TreeSupport3D.cpp:471
        for me in line {
            // TreeSupport3D.cpp:472
            if evaluate_point(me) != current_keep {
                // TreeSupport3D.cpp:473
                if !resulting_line.is_empty() {
                    if current_keep {
                        keep.push(std::mem::take(&mut resulting_line));
                    } else {
                        set_free.push(std::mem::take(&mut resulting_line));
                    }
                }
                // TreeSupport3D.cpp:476
                current_keep = !current_keep;
            }
            // TreeSupport3D.cpp:477
            resulting_line.push(*me);
        }
        // TreeSupport3D.cpp:479
        if !resulting_line.is_empty() {
            if current_keep {
                keep.push(resulting_line);
            } else {
                set_free.push(resulting_line);
            }
        }
    }
    // TreeSupport3D.cpp:484
    (keep, set_free)
}

// ============================================================================
// TreeSupport3D.cpp:490-534 — polyline_sample_next_point_at_distance
// Ported from CURA's PolygonUtils::getNextPointWithDistance()
// ============================================================================
fn polyline_sample_next_point_at_distance(
    polyline: &[Point],
    start_pt: &Point,
    start_idx: usize,
    dist: CoordF,
) -> Option<(Point, usize)> {
    // TreeSupport3D.cpp:492-494
    let dist2 = dist * dist;
    let dist2i = dist2 as i64;
    let eps = 0.01 * SCALING_FACTOR; // scaled<double>(0.01) == 1000.0 (no rounding)

    // TreeSupport3D.cpp:496
    for i in (start_idx + 1)..polyline.len() {
        // TreeSupport3D.cpp:497
        let p1 = polyline[i];
        // TreeSupport3D.cpp:498
        if (p1 - *start_pt).length_squared() >= dist2i as i128 {
            // The end point is outside the circle with center "start_pt" and radius "dist".
            // TreeSupport3D.cpp:500
            let p0 = polyline[i - 1];
            // TreeSupport3D.cpp:501 — Vec2d v = (p1 - p0).cast<double>();
            let v = ((p1 - p0).x as CoordF, (p1 - p0).y as CoordF);
            // TreeSupport3D.cpp:502
            let l2v = v.0 * v.0 + v.1 * v.1;
            // TreeSupport3D.cpp:503
            if l2v < eps * eps {
                // Very short segment.
                // TreeSupport3D.cpp:505
                let c = (p0 + p1) / 2;
                // TreeSupport3D.cpp:506
                let norm = (((*start_pt - c).x as CoordF).powi(2)
                    + ((*start_pt - c).y as CoordF).powi(2))
                .sqrt();
                if (norm - dist).abs() < eps {
                    return Some((c, i - 1)); // TreeSupport3D.cpp:507
                } else {
                    continue; // TreeSupport3D.cpp:509
                }
            }
            // TreeSupport3D.cpp:511 — Vec2d p0f = (start_pt - p0).cast<double>();
            let p0f = ((*start_pt - p0).x as CoordF, (*start_pt - p0).y as CoordF);
            // TreeSupport3D.cpp:513 — Foot point of start_pt into v.
            let p0f_dot_v = p0f.0 * v.0 + p0f.1 * v.1;
            let foot_pt = (v.0 * (p0f_dot_v / l2v), v.1 * (p0f_dot_v / l2v));
            // TreeSupport3D.cpp:515 — Vector from foot point of "start_pt" to "start_pt".
            let xf = (p0f.0 - foot_pt.0, p0f.1 - foot_pt.1);
            // TreeSupport3D.cpp:517 — Squared distance of "start_pt" from the ray (p0, p1).
            let l2_from_line = xf.0 * xf.0 + xf.1 * xf.1;
            // TreeSupport3D.cpp:519
            let mut l2_intersection = dist2 - l2_from_line;
            if l2_intersection > -(SCALED_EPSILON) {
                // The ray (p0, p1) touches or intersects a circle centered at "start_pt" with radius "dist".
                // TreeSupport3D.cpp:523
                l2_intersection = l2_intersection.max(0.0);
                // TreeSupport3D.cpp:524 — (v - foot_pt).squaredNorm() >= l2_intersection
                let vmf = (v.0 - foot_pt.0, v.1 - foot_pt.1);
                if vmf.0 * vmf.0 + vmf.1 * vmf.1 >= l2_intersection {
                    // Intersection of the circle with the segment (p0, p1) is on the right side (close to p1) from the foot point.
                    // TreeSupport3D.cpp:526
                    let k = (l2_intersection / l2v).sqrt();
                    let add = (foot_pt.0 + v.0 * k, foot_pt.1 + v.1 * k);
                    let p = p0 + Point::new(add.0 as Coord, add.1 as Coord);
                    return Some((p, i - 1)); // TreeSupport3D.cpp:528
                }
            }
        }
    }
    // TreeSupport3D.cpp:533
    None
}

// ============================================================================
// TreeSupport3D.cpp:544-636 — ensure_maximum_distance_polyline
// ============================================================================
pub fn ensure_maximum_distance_polyline(
    input: &[Vec<Point>],
    distance: CoordF,
    min_points: usize,
) -> Vec<Vec<Point>> {
    // TreeSupport3D.cpp:546
    let mut result: Vec<Vec<Point>> = Vec::new();
    // TreeSupport3D.cpp:547
    for part_in in input {
        let mut part: Vec<Point> = part_in.clone();
        // TreeSupport3D.cpp:548
        if part.is_empty() {
            continue;
        }

        // TreeSupport3D.cpp:551 — double len = length(part.points);
        let len = polyline_length(&part);
        // TreeSupport3D.cpp:552
        let mut line: Vec<Point> = Vec::new();
        // TreeSupport3D.cpp:553
        let mut current_distance = distance.max(0.1 * SCALING_FACTOR); // scaled<double>(0.1)
        // TreeSupport3D.cpp:554
        if len < 2.0 * distance && min_points <= 1 {
            // Insert the opposite point of the first one.
            // TreeSupport3D.cpp:558-560 — clip_end(len/2) then take last point.
            let pl = polyline_clip_end(&part, len / 2.0);
            if let Some(last) = pl.last() {
                line.push(*last);
            }
        } else {
            // TreeSupport3D.cpp:564
            let mut optimal_end_index = part.len() - 1;

            // TreeSupport3D.cpp:566
            if part.first() == part.last() {
                // TreeSupport3D.cpp:567
                let mut optimal_start_index = 0usize;
                // TreeSupport3D.cpp:571 — C++ declares this `coord_t` (int32) but
                // assigns a `double` squaredNorm to it (truncating + risking int32
                // overflow for large separations). We keep it as `f64` to avoid
                // the overflow; the comparison promotes to double either way.
                // FIDELITY-NOTE(F2): coord_t==int32 store elided in favour of f64.
                let mut max_dist2_between_vertecies: CoordF = 0.0;
                // TreeSupport3D.cpp:572
                for idx in 0..(part.len() - 1) {
                    // TreeSupport3D.cpp:573
                    for inner_idx in 0..(part.len() - 1) {
                        let d = part[idx] - part[inner_idx];
                        let d2 = (d.x as CoordF) * (d.x as CoordF) + (d.y as CoordF) * (d.y as CoordF);
                        // TreeSupport3D.cpp:574
                        if d2 > max_dist2_between_vertecies {
                            optimal_start_index = idx;
                            optimal_end_index = inner_idx;
                            max_dist2_between_vertecies = d2;
                        }
                    }
                }
                // TreeSupport3D.cpp:581 — std::rotate(part.begin(), part.begin()+osi, part.end()-1);
                let n = part.len();
                part[..n - 1].rotate_left(optimal_start_index);
                // TreeSupport3D.cpp:582 — part.back() = part.front();
                part[n - 1] = part[0];
                // TreeSupport3D.cpp:583
                optimal_end_index =
                    (part.len() + optimal_end_index - optimal_start_index - 1) % (part.len() - 1);
            }

            // TreeSupport3D.cpp:586
            while line.len() < min_points && current_distance >= 0.1 * SCALING_FACTOR {
                // TreeSupport3D.cpp:588
                line.clear();
                // TreeSupport3D.cpp:589
                let mut current_point = part[0];
                // TreeSupport3D.cpp:590
                line.push(part[0]);
                // TreeSupport3D.cpp:591
                let d_end = part[0] - part[optimal_end_index];
                let norm_end = ((d_end.x as CoordF).powi(2) + (d_end.y as CoordF).powi(2)).sqrt();
                if min_points > 1 || norm_end > current_distance {
                    line.push(part[optimal_end_index]); // TreeSupport3D.cpp:592
                }
                // TreeSupport3D.cpp:593
                let mut current_index = 0usize;
                // TreeSupport3D.cpp:595
                let mut next_distance = current_distance;
                // TreeSupport3D.cpp:598
                while let Some(next_point) =
                    polyline_sample_next_point_at_distance(&part, &current_point, current_index, next_distance)
                {
                    // TreeSupport3D.cpp:602
                    let mut min_distance_to_existing_point = CoordF::MAX;
                    for p in &line {
                        let d = *p - next_point.0;
                        let nd = ((d.x as CoordF).powi(2) + (d.y as CoordF).powi(2)).sqrt();
                        min_distance_to_existing_point = min_distance_to_existing_point.min(nd);
                    }
                    // TreeSupport3D.cpp:605
                    if min_distance_to_existing_point >= current_distance {
                        // viable point was found. Add to possible result.
                        // TreeSupport3D.cpp:607-610
                        line.push(next_point.0);
                        current_point = next_point.0;
                        current_index = next_point.1;
                        next_distance = current_distance;
                    } else {
                        // TreeSupport3D.cpp:612
                        if current_point == next_point.0 {
                            // In case a fixpoint is encountered, better aggressively overcompensate.
                            // TreeSupport3D.cpp:616
                            tree_supports_show_error(
                                "Encountered issue while placing tips. Some tips may be missing.",
                                true,
                            );
                            // TreeSupport3D.cpp:617
                            if next_distance > 2.0 * current_distance {
                                break; // TreeSupport3D.cpp:619
                            }
                            // TreeSupport3D.cpp:620
                            next_distance += current_distance;
                            continue;
                        }
                        // TreeSupport3D.cpp:624
                        next_distance = (current_distance - min_distance_to_existing_point)
                            .max(0.1 * SCALING_FACTOR); // scaled<double>(0.1)
                        // TreeSupport3D.cpp:625-626
                        current_point = next_point.0;
                        current_index = next_point.1;
                    }
                }
                // TreeSupport3D.cpp:629
                current_distance *= 0.9;
            }
        }
        // TreeSupport3D.cpp:632
        result.push(line);
    }
    // TreeSupport3D.cpp:634
    result
}

// Polyline.hpp `double length(const Points &)`
fn polyline_length(pts: &[Point]) -> CoordF {
    let mut total = 0.0;
    for i in 1..pts.len() {
        total += (pts[i] - pts[i - 1]).length();
    }
    total
}

// Polyline::clip_end(distance) — remove `distance` of length from the end.
fn polyline_clip_end(pts: &[Point], mut distance: CoordF) -> Vec<Point> {
    let mut out = pts.to_vec();
    while distance > 0.0 {
        if out.len() < 2 {
            break;
        }
        let last = *out.last().unwrap();
        let prev = out[out.len() - 2];
        let last_length = (last - prev).length();
        if last_length <= distance {
            out.pop();
            distance -= last_length;
        } else {
            let dir = last - prev;
            let new_last = prev
                + Point::new(
                    (dir.x as CoordF * (last_length - distance) / last_length) as Coord,
                    (dir.y as CoordF * (last_length - distance) / last_length) as Coord,
                );
            *out.last_mut().unwrap() = new_last;
            break;
        }
    }
    out
}

// ============================================================================
// TreeSupport3D.cpp:1329-1430 — move_inside
// ============================================================================
// Returns the index of the polygon the point was moved into, or u32::MAX (-1).
fn move_inside(polygons: &[Polygon], from: &mut Point, distance: i64, max_dist2: i64) -> u32 {
    // TreeSupport3D.cpp:1331
    let mut ret = *from;
    // TreeSupport3D.cpp:1332
    let mut best_dist2 = f64::MAX;
    // TreeSupport3D.cpp:1333
    let mut best_poly: u32 = u32::MAX;
    // TreeSupport3D.cpp:1334
    let mut is_already_on_correct_side_of_boundary = false;
    // TreeSupport3D.cpp:1335
    for poly_idx in 0..polygons.len() {
        // TreeSupport3D.cpp:1336
        let poly = &polygons[poly_idx];
        let pts = &poly.points;
        // TreeSupport3D.cpp:1337
        if pts.len() < 2 {
            continue;
        }
        // TreeSupport3D.cpp:1339
        let mut p0 = pts[pts.len() - 2];
        // TreeSupport3D.cpp:1340
        let mut p1 = pts[pts.len() - 1];
        // TreeSupport3D.cpp:1343
        let mut projected_p_beyond_prev_segment =
            (p1 - p0).dot(&(*from - p0)) >= (p1 - p0).length_squared();
        // TreeSupport3D.cpp:1344
        for &p2 in pts.iter() {
            // TreeSupport3D.cpp:1348-1352
            let a = p1;
            let b = p2;
            let p = *from;
            let ab = b - a;
            let ap = p - a;
            // TreeSupport3D.cpp:1353
            let ab_length2 = ab.length_squared();
            // TreeSupport3D.cpp:1354
            if ab_length2 <= 0 {
                p1 = p2;
                continue;
            }
            // TreeSupport3D.cpp:1358
            let dot_prod = ab.dot(&ap);
            // TreeSupport3D.cpp:1359
            if dot_prod <= 0 {
                // x is projected to before ab
                // TreeSupport3D.cpp:1360
                if projected_p_beyond_prev_segment {
                    // TreeSupport3D.cpp:1362
                    projected_p_beyond_prev_segment = false;
                    // TreeSupport3D.cpp:1363
                    let x = p1;
                    // TreeSupport3D.cpp:1365
                    let dist2 = (x - p).length_squared() as f64;
                    // TreeSupport3D.cpp:1366
                    if dist2 < best_dist2 {
                        best_dist2 = dist2;
                        best_poly = poly_idx as u32;
                        // TreeSupport3D.cpp:1369
                        if distance == 0 {
                            ret = x;
                        } else {
                            // TreeSupport3D.cpp:1372-1379
                            let abd = (ab.x as f64, ab.y as f64);
                            let p1p2 = ((p1 - p0).x as f64, (p1 - p0).y as f64);
                            let lab = (abd.0 * abd.0 + abd.1 * abd.1).sqrt();
                            let lp1p2 = (p1p2.0 * p1p2.0 + p1p2.1 * p1p2.1).sqrt();
                            let s10 = 10.0 * SCALING_FACTOR; // scaled<double>(10.0) == 1_000_000.0
                            let sum = (abd.0 * (s10 / lab) + p1p2.0 * (s10 / lp1p2),
                                       abd.1 * (s10 / lab) + p1p2.1 * (s10 / lp1p2));
                            let inward_dir = (-sum.1, sum.0); // perp(v)
                            let inward_norm = (inward_dir.0 * inward_dir.0 + inward_dir.1 * inward_dir.1).sqrt();
                            ret = x
                                + Point::new(
                                    (inward_dir.0 * (distance as f64 / inward_norm)) as Coord,
                                    (inward_dir.1 * (distance as f64 / inward_norm)) as Coord,
                                );
                            // TreeSupport3D.cpp:1380
                            let px = p - x;
                            is_already_on_correct_side_of_boundary =
                                (inward_dir.0 * px.x as f64 + inward_dir.1 * px.y as f64) * distance as f64 >= 0.0;
                        }
                    }
                } else {
                    // TreeSupport3D.cpp:1384-1387
                    projected_p_beyond_prev_segment = false;
                    p0 = p1;
                    p1 = p2;
                    continue;
                }
            } else if dot_prod >= ab_length2 {
                // TreeSupport3D.cpp:1389-1394 — x is projected to beyond ab
                projected_p_beyond_prev_segment = true;
                p0 = p1;
                p1 = p2;
                continue;
            } else {
                // TreeSupport3D.cpp:1396-1397 — x is properly on the segment
                projected_p_beyond_prev_segment = false;
                // TreeSupport3D.cpp:1398
                let x = a
                    + Point::new(
                        (ab.x as f64 * (dot_prod as f64 / ab_length2 as f64)) as Coord,
                        (ab.y as f64 * (dot_prod as f64 / ab_length2 as f64)) as Coord,
                    );
                // TreeSupport3D.cpp:1399
                let dist2 = (p - x).length_squared() as f64;
                // TreeSupport3D.cpp:1400
                if dist2 < best_dist2 {
                    best_dist2 = dist2;
                    best_poly = poly_idx as u32;
                    // TreeSupport3D.cpp:1403
                    if distance == 0 {
                        ret = x;
                    } else {
                        // TreeSupport3D.cpp:1406-1408
                        let abd = (ab.x as f64, ab.y as f64);
                        let abd_norm = (abd.0 * abd.0 + abd.1 * abd.1).sqrt();
                        let scaled_v = (abd.0 * (distance as f64 / abd_norm), abd.1 * (distance as f64 / abd_norm));
                        let inward_dir = (-scaled_v.1, scaled_v.0); // perp
                        ret = x + Point::new(inward_dir.0 as Coord, inward_dir.1 as Coord);
                        // TreeSupport3D.cpp:1409
                        let px = p - x;
                        is_already_on_correct_side_of_boundary =
                            inward_dir.0 * px.x as f64 + inward_dir.1 * px.y as f64 >= 0.0;
                    }
                }
            }
            // TreeSupport3D.cpp:1413-1414
            p0 = p1;
            p1 = p2;
        }
    }
    // TreeSupport3D.cpp:1418
    if is_already_on_correct_side_of_boundary {
        // TreeSupport3D.cpp:1419
        if best_dist2 < (distance as f64) * (distance as f64) {
            *from = ret;
        }
        // else: from stays unaltered.
        // TreeSupport3D.cpp:1424
        best_poly
    } else if best_dist2 < max_dist2 as f64 {
        // TreeSupport3D.cpp:1426-1427
        *from = ret;
        best_poly
    } else {
        // TreeSupport3D.cpp:1429
        u32::MAX
    }
}

// ============================================================================
// TreeSupport3D.cpp:1432-1437 — move_inside_if_outside
// ============================================================================
fn move_inside_if_outside(polygons: &[Polygon], mut from: Point) -> Point {
    // TreeSupport3D.cpp:1434
    if !contains(polygons, &from) {
        // TreeSupport3D.cpp:1435 — move_inside(polygons, from) with default distance=0, maxDist2=MAX
        move_inside(polygons, &mut from, 0, i64::MAX);
    }
    // TreeSupport3D.cpp:1436
    from
}

// ============================================================================
// TreeSupport3D.cpp:1459-1589 — increase_single_area
// ============================================================================
#[allow(clippy::too_many_arguments)]
fn increase_single_area(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    settings: &AreaIncreaseSettings,
    layer_idx: LayerIndex,
    parent: &SupportElement,
    relevant_offset: &Polygons,
    to_bp_data: &mut Polygons,
    to_model_data: &mut Polygons,
    increased: &mut Polygons,
    overspeed: Coord,
    mergelayer: bool,
) -> Option<SupportElementState> {
    // TreeSupport3D.cpp:1472
    let mut current_elem = parent.state.propagate_down();
    // TreeSupport3D.cpp:1473
    let mut check_layer_data: Polygons;
    // TreeSupport3D.cpp:1474
    if settings.increase_radius {
        current_elem.effective_radius_height += 1;
    }
    // TreeSupport3D.cpp:1476
    let mut radius = current_elem.get_collision_radius(config);

    // TreeSupport3D.cpp:1478
    if settings.allow_move {
        // TreeSupport3D.cpp:1479
        *increased = relevant_offset.clone();
        // TreeSupport3D.cpp:1480
        if overspeed > 0 {
            // TreeSupport3D.cpp:1481-1483
            let safe_movement_distance = (if current_elem.bits.use_min_xy_dist {
                config.xy_min_distance
            } else {
                config.xy_distance
            }) + (if config.z_distance_top_layers.min(config.z_distance_bottom_layers) > 0 {
                config.min_feature_size
            } else {
                0
            });
            // TreeSupport3D.cpp:1486
            *increased = safe_offset_inc(
                increased,
                overspeed,
                &volumes.get_wall_restriction(
                    parent.state.get_collision_radius(config),
                    layer_idx,
                    parent.state.bits.use_min_xy_dist,
                ),
                safe_movement_distance,
                safe_movement_distance + radius,
                1,
            );
        }
        // TreeSupport3D.cpp:1489
        if settings.no_error && settings.allow_move {
            // TreeSupport3D.cpp:1491
            // scaled<float>(0.025) == 2500.0f, exactly representable; promoted to double.
            *increased = polygons_simplify(increased, 0.025 * SCALING_FACTOR);
        }
    } else {
        // TreeSupport3D.cpp:1494 — keep parent area as no move == offset(0)
        *increased = parent.influence_area.clone();
    }

    // TreeSupport3D.cpp:1496
    if mergelayer || current_elem.bits.to_buildplate {
        // TreeSupport3D.cpp:1497
        *to_bp_data = safe_union(
            &diff_clipped(
                increased,
                &volumes.get_avoidance_full(
                    radius,
                    layer_idx - 1,
                    vol_avoidance(settings.avoidance_type),
                    false,
                    settings.use_min_distance,
                ),
            ),
            &Vec::new(),
        );
        // TreeSupport3D.cpp:1498
        if !current_elem.bits.to_buildplate && area(to_bp_data) > tiny_area_threshold() {
            // TreeSupport3D.cpp:1500
            current_elem.bits.to_buildplate = true;
        }
    }
    // TreeSupport3D.cpp:1505
    if config.support_rests_on_model {
        // TreeSupport3D.cpp:1506
        if mergelayer || current_elem.bits.to_model_gracious {
            *to_model_data = safe_union(
                &diff_clipped(
                    increased,
                    &volumes.get_avoidance_full(
                        radius,
                        layer_idx - 1,
                        vol_avoidance(settings.avoidance_type),
                        true,
                        settings.use_min_distance,
                    ),
                ),
                &Vec::new(),
            );
        }
        // TreeSupport3D.cpp:1509
        if !current_elem.bits.to_model_gracious {
            // TreeSupport3D.cpp:1510
            if mergelayer && area(to_model_data) >= tiny_area_threshold() {
                current_elem.bits.to_model_gracious = true;
            } else {
                // TreeSupport3D.cpp:1515
                *to_model_data = safe_union(
                    &diff_clipped(
                        increased,
                        &volumes.get_collision_min_xy(radius, layer_idx - 1, settings.use_min_distance),
                    ),
                    &Vec::new(),
                );
            }
        }
    }

    // TreeSupport3D.cpp:1519
    check_layer_data = if current_elem.bits.to_buildplate {
        to_bp_data.clone()
    } else {
        to_model_data.clone()
    };

    // TreeSupport3D.cpp:1521
    if settings.increase_radius && area(&check_layer_data) > tiny_area_threshold() {
        // TreeSupport3D.cpp:1522 — validWithRadius lambda
        let valid_with_radius = |next_radius: Coord, cur_elem: &SupportElementState, cur_radius: Coord| -> bool {
            // TreeSupport3D.cpp:1523
            if volumes.ceil_radius_min_xy(next_radius, settings.use_min_distance)
                <= volumes.ceil_radius_min_xy(cur_radius, settings.use_min_distance)
            {
                return true;
            }
            // TreeSupport3D.cpp:1526
            let mut to_bp_data_2: Polygons = Vec::new();
            if cur_elem.bits.to_buildplate {
                to_bp_data_2 = diff_clipped(
                    increased,
                    &volumes.get_avoidance_full(
                        next_radius,
                        layer_idx - 1,
                        vol_avoidance(settings.avoidance_type),
                        false,
                        settings.use_min_distance,
                    ),
                );
            }
            // TreeSupport3D.cpp:1530
            let mut to_model_data_2: Polygons = Vec::new();
            if config.support_rests_on_model && !cur_elem.bits.to_buildplate {
                to_model_data_2 = diff_clipped(
                    increased,
                    &if cur_elem.bits.to_model_gracious {
                        volumes.get_avoidance_full(
                            next_radius,
                            layer_idx - 1,
                            vol_avoidance(settings.avoidance_type),
                            true,
                            settings.use_min_distance,
                        )
                    } else {
                        volumes.get_collision_min_xy(next_radius, layer_idx - 1, settings.use_min_distance)
                    },
                );
            }
            // TreeSupport3D.cpp:1536
            let check_layer_data_2 = if cur_elem.bits.to_buildplate {
                to_bp_data_2
            } else {
                to_model_data_2
            };
            // TreeSupport3D.cpp:1537
            area(&check_layer_data_2) > tiny_area_threshold()
        };
        // TreeSupport3D.cpp:1539
        let ceil_radius_before = volumes.ceil_radius_min_xy(radius, settings.use_min_distance);

        // TreeSupport3D.cpp:1541
        if current_elem.get_collision_radius(config) < config.increase_radius_until_radius
            && current_elem.get_collision_radius(config) < current_elem.get_radius(config)
        {
            // TreeSupport3D.cpp:1542
            let target_radius = current_elem.get_radius(config).min(config.increase_radius_until_radius);
            // TreeSupport3D.cpp:1543
            let mut current_ceil_radius = volumes.get_radius_next_ceil(radius, settings.use_min_distance);

            // TreeSupport3D.cpp:1545
            while current_ceil_radius < target_radius
                && valid_with_radius(
                    volumes.get_radius_next_ceil(current_ceil_radius + 1, settings.use_min_distance),
                    &current_elem,
                    radius,
                )
            {
                current_ceil_radius =
                    volumes.get_radius_next_ceil(current_ceil_radius + 1, settings.use_min_distance);
            }
            // TreeSupport3D.cpp:1547
            let mut resulting_eff_dtt = current_elem.effective_radius_height as usize;
            // TreeSupport3D.cpp:1548
            while resulting_eff_dtt + 1 < current_elem.distance_to_top as usize
                && config.get_radius(resulting_eff_dtt + 1, current_elem.elephant_foot_increases)
                    <= current_ceil_radius
                && config.get_radius(resulting_eff_dtt + 1, current_elem.elephant_foot_increases)
                    <= current_elem.get_radius(config)
            {
                resulting_eff_dtt += 1;
            }
            // TreeSupport3D.cpp:1552
            current_elem.effective_radius_height = resulting_eff_dtt as u32;
        }
        // TreeSupport3D.cpp:1554
        radius = current_elem.get_collision_radius(config);

        // TreeSupport3D.cpp:1556
        // C++ declares `const coord_t foot_radius_increase = std::max(double, 0.0)`,
        // truncating the double max() result toward zero into int32. The truncated
        // integer is then used as the divisor below, so the truncation is observable.
        // FIDELITY-NOTE(F2): coord_t==int32 truncation reproduced via `as i32`.
        let foot_radius_increase =
            ((config.bp_radius_increase_per_layer - config.branch_radius_increase_per_layer).max(0.0)
                as i32) as f64;
        // TreeSupport3D.cpp:1559
        let planned_foot_increase = if foot_radius_increase != 0.0 {
            1.0_f64.min(
                (recommended_min_radius(config, layer_idx - 1) - current_elem.get_radius(config)) as f64
                    / foot_radius_increase,
            )
        } else {
            // foot_radius_increase == 0 → division by zero in C++ yields +/-inf; min(1.0, inf)=1.0
            1.0
        };
        // TreeSupport3D.cpp:1561
        let increase_bp_foot = planned_foot_increase > 0.0 && current_elem.bits.to_buildplate;

        // TreeSupport3D.cpp:1564
        if increase_bp_foot
            && current_elem.get_radius(config) >= config.branch_radius
            && current_elem.get_radius(config) >= config.increase_radius_until_radius
            && valid_with_radius(
                // TreeSupport3D.cpp:1565
                config.get_radius(
                    current_elem.effective_radius_height as usize,
                    current_elem.elephant_foot_increases + planned_foot_increase,
                ),
                &current_elem,
                radius,
            )
        {
            // TreeSupport3D.cpp:1566-1567
            current_elem.elephant_foot_increases += planned_foot_increase;
            radius = current_elem.get_collision_radius(config);
        }

        // TreeSupport3D.cpp:1570
        if ceil_radius_before != volumes.ceil_radius_min_xy(radius, settings.use_min_distance) {
            // TreeSupport3D.cpp:1571
            if current_elem.bits.to_buildplate {
                *to_bp_data = safe_union(
                    &diff_clipped(
                        increased,
                        &volumes.get_avoidance_full(
                            radius,
                            layer_idx - 1,
                            vol_avoidance(settings.avoidance_type),
                            false,
                            settings.use_min_distance,
                        ),
                    ),
                    &Vec::new(),
                );
            }
            // TreeSupport3D.cpp:1573
            if config.support_rests_on_model && (!current_elem.bits.to_buildplate || mergelayer) {
                *to_model_data = safe_union(
                    &diff_clipped(
                        increased,
                        &if current_elem.bits.to_model_gracious {
                            volumes.get_avoidance_full(
                                radius,
                                layer_idx - 1,
                                vol_avoidance(settings.avoidance_type),
                                true,
                                settings.use_min_distance,
                            )
                        } else {
                            volumes.get_collision_min_xy(radius, layer_idx - 1, settings.use_min_distance)
                        },
                    ),
                    &Vec::new(),
                );
            }
            // TreeSupport3D.cpp:1579
            check_layer_data = if current_elem.bits.to_buildplate {
                to_bp_data.clone()
            } else {
                to_model_data.clone()
            };
            // TreeSupport3D.cpp:1580
            if area(&check_layer_data) < tiny_area_threshold() {
                tree_supports_show_error(
                    "Area lost catching up radius. May not cause visible malformation.",
                    true,
                );
            }
        }
    }

    // TreeSupport3D.cpp:1588
    if area(&check_layer_data) > tiny_area_threshold() {
        Some(current_elem)
    } else {
        None
    }
}

// TreeSupportCommon.hpp:555 recommendedMinRadius(layer_idx) — bp foot radius profile.
//   double num_layers_widened = layer_start_bp_radius - layer_idx;
//   return num_layers_widened > 0 ? branch_radius + num_layers_widened * bp_radius_increase_per_layer : 0;
// The result is `coord_t` (int32): the double expression truncates toward zero on
// return. FIDELITY-NOTE(F2): coord_t==int32 truncation reproduced via `as i32`.
#[inline]
fn recommended_min_radius(config: &TreeSupportSettings, layer_idx: LayerIndex) -> Coord {
    let num_layers_widened = config.layer_start_bp_radius as f64 - layer_idx as f64;
    if num_layers_widened > 0.0 {
        ((config.branch_radius as f64
            + num_layers_widened * config.bp_radius_increase_per_layer) as i32) as Coord
    } else {
        0
    }
}

// ============================================================================
// TreeSupport3D.cpp:1591-1604 — SupportElementInfluenceAreas
// ============================================================================
#[derive(Debug, Clone, Default)]
struct SupportElementInfluenceAreas {
    // All influence areas: both to build plate and model.
    influence_areas: Polygons,
    // Influence areas just to build plate.
    to_bp_areas: Polygons,
    // Influence areas just to model.
    to_model_areas: Polygons,
}

impl SupportElementInfluenceAreas {
    // TreeSupport3D.cpp:1599
    fn clear(&mut self) {
        self.influence_areas.clear();
        self.to_bp_areas.clear();
        self.to_model_areas.clear();
    }
}

// ============================================================================
// TreeSupport3D.cpp:1606-1625 — SupportElementMerging
// ============================================================================
#[derive(Debug, Clone)]
struct SupportElementMerging {
    state: SupportElementState,
    // All elements in the layer above the current one that are supported by this element
    parents: Vec<i32>,
    areas: SupportElementInfluenceAreas,
    // Bounding box of all influence areas.
    bbox_data: BoundingBox,
}

impl SupportElementMerging {
    fn new(state: SupportElementState, parents: Vec<i32>) -> Self {
        Self {
            state,
            parents,
            areas: SupportElementInfluenceAreas::default(),
            bbox_data: BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(0, 0)),
        }
    }
    // TreeSupport3D.cpp:1617
    fn bbox(&self) -> BoundingBox {
        self.bbox_data
    }
    // TreeSupport3D.cpp:1618
    fn centroid(&self) -> Point {
        (self.bbox_data.min + self.bbox_data.max) / 2
    }
    // TreeSupport3D.cpp:1619 — set_bbox: inflate by SCALED_EPSILON.
    fn set_bbox(&mut self, abbox: &BoundingBox) {
        let eps = Point::new(SCALED_EPSILON as Coord, SCALED_EPSILON as Coord);
        self.bbox_data = BoundingBox::from_points_minmax(abbox.min - eps, abbox.max + eps);
    }
}

// SourceNode impl for the 2D AABBTreeIndirect builder (build_modify_input).
impl crate::aabb_tree_lines::aabb_tree_indirect_2d::SourceNode for SupportElementMerging {
    fn idx(&self) -> usize {
        // TreeSupport3D.cpp:1624 — not needed, thus zero is returned.
        0
    }
    fn centroid(&self) -> [f64; 2] {
        let c = SupportElementMerging::centroid(self);
        [c.x as f64, c.y as f64]
    }
    fn bbox(&self) -> crate::aabb_tree_lines::aabb_tree_indirect_2d::BoundingBox {
        crate::aabb_tree_lines::aabb_tree_indirect_2d::BoundingBox {
            min: [self.bbox_data.min.x as f64, self.bbox_data.min.y as f64],
            max: [self.bbox_data.max.x as f64, self.bbox_data.max.y as f64],
        }
    }
}

// ============================================================================
// TreeSupport3D.cpp:1645-1907 — increase_areas_one_layer
// Ported sequentially (C++ used tbb::parallel_for over merging_areas).
// ============================================================================
fn increase_areas_one_layer(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    merging_areas: &mut [SupportElementMerging],
    layer_idx: LayerIndex,
    layer_elements: &mut SupportElements,
    mergelayer: bool,
) {
    // TreeSupport3D.cpp:1662
    for merging_area_idx in 0..merging_areas.len() {
        // TreeSupport3D.cpp:1664
        debug_assert_eq!(merging_areas[merging_area_idx].parents.len(), 1);
        let parent_idx = merging_areas[merging_area_idx].parents[0] as usize;
        // TreeSupport3D.cpp:1666
        let mut elem = layer_elements[parent_idx].state.propagate_down();
        // TreeSupport3D.cpp:1667-1669 — wall_restriction
        let wall_restriction = volumes.get_wall_restriction(
            layer_elements[parent_idx].state.get_collision_radius(config),
            layer_idx,
            layer_elements[parent_idx].state.bits.use_min_xy_dist,
        );

        // TreeSupport3D.cpp:1677
        let mut to_bp_data: Polygons = Vec::new();
        let mut to_model_data: Polygons = Vec::new();
        // TreeSupport3D.cpp:1678
        let mut radius = elem.get_collision_radius(config);

        // TreeSupport3D.cpp:1685-1686
        let mut extra_speed: Coord = 5;
        let mut extra_slow_speed: Coord = 0;
        // TreeSupport3D.cpp:1687
        let ceiled_parent_radius = volumes.ceil_radius_min_xy(
            layer_elements[parent_idx].state.get_collision_radius(config),
            layer_elements[parent_idx].state.bits.use_min_xy_dist,
        );
        // TreeSupport3D.cpp:1688
        let projected_radius_increased = config.get_radius(
            layer_elements[parent_idx].state.effective_radius_height as usize + 1,
            layer_elements[parent_idx].state.elephant_foot_increases,
        );
        // TreeSupport3D.cpp:1689
        let projected_radius_delta =
            projected_radius_increased - layer_elements[parent_idx].state.get_collision_radius(config);

        // TreeSupport3D.cpp:1698-1700
        let safe_movement_distance = (if elem.bits.use_min_xy_dist {
            config.xy_min_distance
        } else {
            config.xy_distance
        }) + (if config.z_distance_top_layers.min(config.z_distance_bottom_layers) > 0 {
            config.min_feature_size
        } else {
            0
        });
        // TreeSupport3D.cpp:1701
        if ceiled_parent_radius
            == volumes.ceil_radius_min_xy(
                projected_radius_increased,
                layer_elements[parent_idx].state.bits.use_min_xy_dist,
            )
            || projected_radius_increased < config.increase_radius_until_radius
        {
            // TreeSupport3D.cpp:1704
            extra_speed += projected_radius_delta;
        } else {
            // TreeSupport3D.cpp:1708
            extra_slow_speed += projected_radius_delta.min(
                (config.maximum_move_distance + extra_speed)
                    - (config.maximum_move_distance_slow + extra_slow_speed),
            );
        }

        // TreeSupport3D.cpp:1710
        if (config.layer_start_bp_radius as LayerIndex) > layer_idx
            && recommended_min_radius(config, layer_idx - 1)
                < config.get_radius(elem.effective_radius_height as usize + 1, elem.elephant_foot_increases)
        {
            // TreeSupport3D.cpp:1713
            if ceiled_parent_radius
                == volumes.ceil_radius_min_xy(
                    config.get_radius(
                        layer_elements[parent_idx].state.effective_radius_height as usize + 1,
                        layer_elements[parent_idx].state.elephant_foot_increases + 1.0,
                    ),
                    layer_elements[parent_idx].state.bits.use_min_xy_dist,
                )
            {
                extra_speed += config.bp_radius_increase_per_layer as Coord;
            } else {
                // TreeSupport3D.cpp:1716
                extra_slow_speed += (config.bp_radius_increase_per_layer as Coord)
                    .min(config.maximum_move_distance - (config.maximum_move_distance_slow + extra_slow_speed));
            }
        }

        // TreeSupport3D.cpp:1720-1721
        let fast_speed = config.maximum_move_distance + extra_speed;
        let slow_speed = config.maximum_move_distance_slow + extra_speed + extra_slow_speed;

        // TreeSupport3D.cpp:1723
        let mut offset_slow: Polygons = Vec::new();
        let mut offset_fast: Polygons = Vec::new();

        // TreeSupport3D.cpp:1725-1726
        let mut add = false;
        let mut bypass_merge = false;
        // TreeSupport3D.cpp:1727 — aliases
        let increase_radius = true;
        let no_error = true;
        let use_min_radius = true;
        let move_ = true;

        // TreeSupport3D.cpp:1730-1738 — order with insertSetting helper
        let mut order: Vec<AreaIncreaseSettings> = Vec::new();
        let insert_setting = |order: &mut Vec<AreaIncreaseSettings>, settings: AreaIncreaseSettings, back: bool| {
            if !order.iter().any(|s| *s == settings) {
                if back {
                    order.push(settings);
                } else {
                    order.insert(0, settings);
                }
            }
        };

        // TreeSupport3D.cpp:1740-1741
        let parent_moved_slow = elem.last_area_increase.increase_speed < config.maximum_move_distance;
        let avoidance_speed_mismatch =
            parent_moved_slow && elem.last_area_increase.avoidance_type != AvoidanceType::Slow;
        // TreeSupport3D.cpp:1742
        if elem.last_area_increase.allow_move
            && elem.last_area_increase.no_error
            && elem.bits.can_use_safe_radius
            && !mergelayer
            && !avoidance_speed_mismatch
            && (elem.distance_to_top as usize >= config.tip_layers || parent_moved_slow)
        {
            // TreeSupport3D.cpp:1745
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: elem.last_area_increase.avoidance_type,
                    increase_speed: if elem.last_area_increase.increase_speed < config.maximum_move_distance {
                        slow_speed
                    } else {
                        fast_speed
                    },
                    increase_radius,
                    no_error: elem.last_area_increase.no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: elem.last_area_increase.allow_move,
                },
                true,
            );
            // TreeSupport3D.cpp:1747
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: elem.last_area_increase.avoidance_type,
                    increase_speed: if elem.last_area_increase.increase_speed < config.maximum_move_distance {
                        slow_speed
                    } else {
                        fast_speed
                    },
                    increase_radius: !increase_radius,
                    no_error: elem.last_area_increase.no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: elem.last_area_increase.allow_move,
                },
                true,
            );
        }
        // TreeSupport3D.cpp:1751
        if !elem.bits.can_use_safe_radius {
            // TreeSupport3D.cpp:1755
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::Slow,
                    increase_speed: slow_speed,
                    increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: !move_,
                },
                true,
            );
            // TreeSupport3D.cpp:1758
            if (elem.distance_to_top as usize) < round_up_divide(config.tip_layers as i64, 2) as usize {
                insert_setting(
                    &mut order,
                    AreaIncreaseSettings {
                        avoidance_type: AvoidanceType::Fast,
                        increase_speed: slow_speed,
                        increase_radius,
                        no_error,
                        use_min_distance: !use_min_radius,
                        allow_move: !move_,
                    },
                    true,
                );
            }
            // TreeSupport3D.cpp:1760
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::FastSafe,
                    increase_speed: fast_speed,
                    increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: !move_,
                },
                true,
            );
            // TreeSupport3D.cpp:1761
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::FastSafe,
                    increase_speed: fast_speed,
                    increase_radius: !increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: move_,
                },
                true,
            );
            // TreeSupport3D.cpp:1762
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::Fast,
                    increase_speed: fast_speed,
                    increase_radius: !increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: move_,
                },
                true,
            );
        } else {
            // TreeSupport3D.cpp:1764
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::Slow,
                    increase_speed: slow_speed,
                    increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: move_,
                },
                true,
            );
            // TreeSupport3D.cpp:1768
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::Slow,
                    increase_speed: slow_speed,
                    increase_radius: !increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: move_,
                },
                true,
            );
            // TreeSupport3D.cpp:1769
            if (elem.distance_to_top as usize) < config.tip_layers {
                insert_setting(
                    &mut order,
                    AreaIncreaseSettings {
                        avoidance_type: AvoidanceType::FastSafe,
                        increase_speed: slow_speed,
                        increase_radius,
                        no_error,
                        use_min_distance: !use_min_radius,
                        allow_move: move_,
                    },
                    true,
                );
            }
            // TreeSupport3D.cpp:1771
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::FastSafe,
                    increase_speed: fast_speed,
                    increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: move_,
                },
                true,
            );
            // TreeSupport3D.cpp:1772
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::FastSafe,
                    increase_speed: fast_speed,
                    increase_radius: !increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: move_,
                },
                true,
            );
        }

        // TreeSupport3D.cpp:1775
        if elem.bits.use_min_xy_dist {
            // TreeSupport3D.cpp:1776
            let mut new_order: Vec<AreaIncreaseSettings> = Vec::new();
            // TreeSupport3D.cpp:1779
            for settings in &order {
                new_order.push(*settings);
                // TreeSupport3D.cpp:1781
                new_order.push(AreaIncreaseSettings {
                    avoidance_type: settings.avoidance_type,
                    increase_speed: settings.increase_speed,
                    increase_radius: settings.increase_radius,
                    no_error: settings.no_error,
                    use_min_distance: use_min_radius,
                    allow_move: settings.allow_move,
                });
            }
            order = new_order;
        }
        // TreeSupport3D.cpp:1785
        if elem.bits.to_buildplate
            || (elem.bits.to_model_gracious
                && intersection(
                    &layer_elements[parent_idx].influence_area,
                    &volumes.get_placeable_areas(radius, layer_idx, &|| {}),
                )
                .is_empty())
        {
            // TreeSupport3D.cpp:1788 — error case
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::Fast,
                    increase_speed: fast_speed,
                    increase_radius: !increase_radius,
                    no_error: !no_error,
                    use_min_distance: elem.bits.use_min_xy_dist,
                    allow_move: move_,
                },
                true,
            );
        }
        // TreeSupport3D.cpp:1790
        if (elem.distance_to_top as u32) < elem.dont_move_until && elem.bits.can_use_safe_radius {
            // TreeSupport3D.cpp:1792
            insert_setting(
                &mut order,
                AreaIncreaseSettings {
                    avoidance_type: AvoidanceType::Slow,
                    increase_speed: 0,
                    increase_radius,
                    no_error,
                    use_min_distance: !use_min_radius,
                    allow_move: !move_,
                },
                false,
            );
        }

        // TreeSupport3D.cpp:1794
        let mut inc_wo_collision: Polygons = Vec::new();
        // TreeSupport3D.cpp:1797
        let offset_independant_faster = (radius / safe_movement_distance)
            - (if config.maximum_move_distance + extra_speed < radius + safe_movement_distance {
                1
            } else {
                0
            })
            > round_up_divide(
                (extra_speed + extra_slow_speed + config.maximum_move_distance_slow) as i64,
                safe_movement_distance as i64,
            );

        // TreeSupport3D.cpp:1799
        for settings in &order {
            // TreeSupport3D.cpp:1800
            if settings.allow_move {
                // TreeSupport3D.cpp:1801
                if offset_slow.is_empty()
                    && (settings.increase_speed == slow_speed || !offset_independant_faster)
                {
                    // TreeSupport3D.cpp:1804
                    offset_slow = safe_offset_inc(
                        &layer_elements[parent_idx].influence_area,
                        extra_speed + extra_slow_speed + config.maximum_move_distance_slow,
                        &wall_restriction,
                        safe_movement_distance,
                        if offset_independant_faster {
                            safe_movement_distance + radius
                        } else {
                            0
                        },
                        2,
                    );
                }
                // TreeSupport3D.cpp:1812
                if offset_fast.is_empty() && settings.increase_speed != slow_speed {
                    // TreeSupport3D.cpp:1813
                    if offset_independant_faster {
                        offset_fast = safe_offset_inc(
                            &layer_elements[parent_idx].influence_area,
                            extra_speed + config.maximum_move_distance,
                            &wall_restriction,
                            safe_movement_distance,
                            if offset_independant_faster {
                                safe_movement_distance + radius
                            } else {
                                0
                            },
                            1,
                        );
                    } else {
                        // TreeSupport3D.cpp:1817
                        let delta_slow_fast = config.maximum_move_distance
                            - (config.maximum_move_distance_slow + extra_slow_speed);
                        offset_fast = safe_offset_inc(
                            &offset_slow,
                            delta_slow_fast,
                            &wall_restriction,
                            safe_movement_distance,
                            safe_movement_distance + radius,
                            if offset_independant_faster { 2 } else { 1 },
                        );
                    }
                }
            }
            // TreeSupport3D.cpp:1827-1828
            let result: Option<SupportElementState>;
            inc_wo_collision.clear();
            // TreeSupport3D.cpp:1829
            if !settings.no_error {
                // ERROR CASE
                // TreeSupport3D.cpp:1832 — offset(to_polylines(parent.influence_area), 0.005, jtMiter, 1.2)
                // FIDELITY-NOTE(F1): geo-clipper offset_polyline uses jtSquare/etOpenButt,
                // not ClipperLib jtMiter(1.2). scaled<float>(0.005) == 500.0.
                let lines_offset = offset_polylines_polygons(
                    &layer_elements[parent_idx].influence_area,
                    0.005 * SCALING_FACTOR,
                );
                // TreeSupport3D.cpp:1833 — union_(parent.influence_area, lines_offset)
                // Plain union_, NOT safe_union (no safety offset in the C++).
                let base_error_area =
                    union_(&layer_elements[parent_idx].influence_area, &lines_offset);
                // TreeSupport3D.cpp:1834
                result = increase_single_area(
                    volumes,
                    config,
                    settings,
                    layer_idx,
                    &layer_elements[parent_idx],
                    &base_error_area,
                    &mut to_bp_data,
                    &mut to_model_data,
                    &mut inc_wo_collision,
                    ((config.maximum_move_distance + extra_speed) as f64 * 1.5) as Coord,
                    mergelayer,
                );
                // TreeSupport3D.cpp:1848
                tree_supports_show_error("Potentially lost branch!", true);
            } else {
                // TreeSupport3D.cpp:1850
                let relevant = if settings.increase_speed == slow_speed {
                    offset_slow.clone()
                } else {
                    offset_fast.clone()
                };
                result = increase_single_area(
                    volumes,
                    config,
                    settings,
                    layer_idx,
                    &layer_elements[parent_idx],
                    &relevant,
                    &mut to_bp_data,
                    &mut to_model_data,
                    &mut inc_wo_collision,
                    0,
                    mergelayer,
                );
            }

            // TreeSupport3D.cpp:1853
            if let Some(r) = result {
                // TreeSupport3D.cpp:1854-1856
                elem = r;
                radius = elem.get_collision_radius(config);
                elem.last_area_increase = *settings;
                add = true;
                // TreeSupport3D.cpp:1859
                bypass_merge = !settings.allow_move
                    || (settings.use_min_distance && (elem.distance_to_top as usize) < config.tip_layers);
                // TreeSupport3D.cpp:1860
                if settings.allow_move {
                    elem.dont_move_until = 0;
                } else {
                    elem.result_on_layer = layer_elements[parent_idx].state.result_on_layer;
                }
                // TreeSupport3D.cpp:1865
                elem.bits.can_use_safe_radius = settings.avoidance_type != AvoidanceType::Fast;
                // TreeSupport3D.cpp:1867
                if !settings.use_min_distance {
                    elem.bits.use_min_xy_dist = false;
                }
                // TreeSupport3D.cpp:1876
                break;
            }
        }

        // TreeSupport3D.cpp:1881
        if add {
            // TreeSupport3D.cpp:1884 — max_influence_area
            let max_influence_area = safe_union(
                &diff_clipped(
                    &inc_wo_collision,
                    &volumes.get_collision_min_xy(radius, layer_idx - 1, elem.bits.use_min_xy_dist),
                ),
                &safe_union(&to_bp_data, &to_model_data),
            );
            // TreeSupport3D.cpp:1887
            merging_areas[merging_area_idx].state = elem.clone();
            // TreeSupport3D.cpp:1889
            let ext = get_extents(&max_influence_area);
            merging_areas[merging_area_idx].set_bbox(&ext);
            // TreeSupport3D.cpp:1890
            merging_areas[merging_area_idx].areas.influence_areas = max_influence_area;
            // TreeSupport3D.cpp:1891
            if !bypass_merge {
                if elem.bits.to_buildplate {
                    merging_areas[merging_area_idx].areas.to_bp_areas = std::mem::take(&mut to_bp_data);
                }
                if config.support_rests_on_model {
                    merging_areas[merging_area_idx].areas.to_model_areas = std::mem::take(&mut to_model_data);
                }
            }
        } else {
            // TreeSupport3D.cpp:1901-1902
            layer_elements[parent_idx].state.result_on_layer_reset();
            layer_elements[parent_idx].state.bits.to_model_gracious = false;
        }
    }
}

// `offset(to_polylines(polygons), delta, jtMiter, 1.2)` helper.
fn offset_polylines_polygons(polygons: &[Polygon], delta: CoordF) -> Polygons {
    let mut out: Polygons = Vec::new();
    for poly in polygons {
        let pl = crate::geometry::Polyline::from_points(poly.points.clone());
        out.extend(crate::clipper_utils::offset_polyline(&pl, delta));
    }
    out
}

// ============================================================================
// TreeSupport3D.cpp:1909-1953 — merge_support_element_states
// ============================================================================
fn merge_support_element_states(
    first: &SupportElementState,
    second: &SupportElementState,
    next_position: &Point,
    layer_idx: LayerIndex,
    config: &TreeSupportSettings,
) -> SupportElementState {
    // TreeSupport3D.cpp:1913
    let mut out = SupportElementState::default();
    // TreeSupport3D.cpp:1914-1921
    out.next_position = *next_position;
    out.layer_idx = layer_idx as usize;
    out.bits.use_min_xy_dist = first.bits.use_min_xy_dist || second.bits.use_min_xy_dist;
    out.bits.supports_roof = first.bits.supports_roof || second.bits.supports_roof;
    out.dont_move_until = first.dont_move_until.max(second.dont_move_until);
    out.bits.can_use_safe_radius = first.bits.can_use_safe_radius || second.bits.can_use_safe_radius;
    out.missing_roof_layers = first.missing_roof_layers.min(second.missing_roof_layers);
    out.bits.skip_ovalisation = false;
    // TreeSupport3D.cpp:1922
    if first.target_height > second.target_height {
        out.target_height = first.target_height;
        out.target_position = first.target_position;
    } else {
        out.target_height = second.target_height;
        out.target_position = second.target_position;
    }
    // TreeSupport3D.cpp:1929-1930
    out.effective_radius_height = first.effective_radius_height.max(second.effective_radius_height);
    out.distance_to_top = first.distance_to_top.max(second.distance_to_top);

    // TreeSupport3D.cpp:1932-1933
    out.bits.to_buildplate = first.bits.to_buildplate && second.bits.to_buildplate;
    out.bits.to_model_gracious = first.bits.to_model_gracious && second.bits.to_model_gracious;

    // TreeSupport3D.cpp:1935
    out.elephant_foot_increases = 0.0;
    // TreeSupport3D.cpp:1936
    if config.bp_radius_increase_per_layer > 0.0 {
        // TreeSupport3D.cpp:1937
        let foot_increase_radius = (support_element_collision_radius_state(config, second)
            .max(support_element_collision_radius_state(config, first))
            - support_element_collision_radius_state(config, &out))
        .abs();
        // TreeSupport3D.cpp:1940
        out.elephant_foot_increases = foot_increase_radius as f64
            / (config.bp_radius_increase_per_layer - config.branch_radius_increase_per_layer);
    }

    // TreeSupport3D.cpp:1944 — last_area_increase = best of both
    out.last_area_increase = AreaIncreaseSettings {
        avoidance_type: min_avoidance(first.last_area_increase.avoidance_type, second.last_area_increase.avoidance_type),
        increase_speed: first.last_area_increase.increase_speed.min(second.last_area_increase.increase_speed),
        increase_radius: first.last_area_increase.increase_radius || second.last_area_increase.increase_radius,
        no_error: first.last_area_increase.no_error || second.last_area_increase.no_error,
        use_min_distance: first.last_area_increase.use_min_distance && second.last_area_increase.use_min_distance,
        allow_move: first.last_area_increase.allow_move || second.last_area_increase.allow_move,
    };

    // TreeSupport3D.cpp:1952
    out
}

// std::min on AreaIncreaseSettings::type, which in C++ is
// `TreeModelVolumes::AvoidanceType` (TreeModelVolumes.hpp:72) declared in the
// order Slow=0, FastSafe=1, Fast=2. std::min compares the underlying int8_t, so
// we must rank by that same C++ enum order, NOT by the compact-enum order.
fn avoidance_rank(t: AvoidanceType) -> u8 {
    match t {
        AvoidanceType::Slow => 0,
        AvoidanceType::FastSafe => 1,
        AvoidanceType::Fast => 2,
    }
}
fn min_avoidance(a: AvoidanceType, b: AvoidanceType) -> AvoidanceType {
    if avoidance_rank(a) <= avoidance_rank(b) {
        a
    } else {
        b
    }
}

#[inline]
fn support_element_collision_radius_state(config: &TreeSupportSettings, s: &SupportElementState) -> Coord {
    s.get_collision_radius(config)
}
#[inline]
fn support_element_radius_state(config: &TreeSupportSettings, s: &SupportElementState) -> Coord {
    s.get_radius(config)
}

// ============================================================================
// TreeSupport3D.cpp:1955-2098 — merge_influence_areas_two_elements
// ============================================================================
fn merge_influence_areas_two_elements(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    layer_idx: LayerIndex,
    dst: &mut SupportElementMerging,
    src: &mut SupportElementMerging,
) -> bool {
    // TreeSupport3D.cpp:1960-1964
    let merging_gracious_and_non_gracious = dst.state.bits.to_model_gracious != src.state.bits.to_model_gracious;
    let merging_min_and_regular_xy = dst.state.bits.use_min_xy_dist != src.state.bits.use_min_xy_dist;

    // TreeSupport3D.cpp:1966
    if merging_gracious_and_non_gracious || merging_min_and_regular_xy {
        return false;
    }

    // TreeSupport3D.cpp:1969
    let dst_radius_bigger =
        support_element_collision_radius_state(config, &dst.state) > support_element_collision_radius_state(config, &src.state);
    // borrow the elements immutably for reads via clones of state references
    let (smaller_state, bigger_state, smaller_areas, bigger_areas, smaller_bbox_orig, bigger_bbox) =
        if dst_radius_bigger {
            (
                src.state.clone(),
                dst.state.clone(),
                src.areas.clone(),
                dst.areas.clone(),
                src.bbox(),
                dst.bbox(),
            )
        } else {
            (
                dst.state.clone(),
                src.state.clone(),
                dst.areas.clone(),
                src.areas.clone(),
                dst.bbox(),
                src.bbox(),
            )
        };
    // TreeSupport3D.cpp:1972
    let real_radius_delta =
        (support_element_radius_state(config, &bigger_state) - support_element_radius_state(config, &smaller_state)).abs();
    {
        // TreeSupport3D.cpp:1979-1983
        let mut smaller_bbox = smaller_bbox_orig;
        smaller_bbox.min = smaller_bbox.min - Point::new(real_radius_delta, real_radius_delta);
        smaller_bbox.max = smaller_bbox.max + Point::new(real_radius_delta, real_radius_delta);
        if !bbox_intersects(&smaller_bbox, &bigger_bbox) {
            return false;
        }
    }

    // TreeSupport3D.cpp:1987
    let mut increased_to_model_radius: Coord = 0;
    // TreeSupport3D.cpp:1988
    let merging_to_bp = dst.state.bits.to_buildplate && src.state.bits.to_buildplate;
    // TreeSupport3D.cpp:1989
    if !merging_to_bp {
        // TreeSupport3D.cpp:1991
        if dst.state.bits.to_buildplate != src.state.bits.to_buildplate {
            // TreeSupport3D.cpp:1994-1995
            let rdst = support_element_radius_state(config, &dst.state);
            let rsrc = support_element_radius_state(config, &src.state);
            // TreeSupport3D.cpp:1996
            if dst.state.bits.to_buildplate {
                if rsrc < rdst {
                    increased_to_model_radius = src.state.increased_to_model_radius + rdst - rsrc;
                }
            } else if rsrc > rdst {
                increased_to_model_radius = dst.state.increased_to_model_radius + rsrc - rdst;
            }
            // TreeSupport3D.cpp:2003
            if increased_to_model_radius > config.max_to_model_radius_increase {
                return false;
            }
        }
        // TreeSupport3D.cpp:2009
        if !dst.state.bits.supports_roof
            && !src.state.bits.supports_roof
            && (src.state.distance_to_top.max(dst.state.distance_to_top) as usize) < config.min_dtt_to_model
        {
            return false;
        }
    }

    // TreeSupport3D.cpp:2016
    if !bigger_state.bits.can_use_safe_radius && smaller_state.bits.can_use_safe_radius {
        return false;
    }

    // TreeSupport3D.cpp:2023
    let use_min_radius = bigger_state.bits.use_min_xy_dist && smaller_state.bits.use_min_xy_dist;

    // TreeSupport3D.cpp:2029
    let smaller_collision_radius = support_element_collision_radius_state(config, &smaller_state);
    // TreeSupport3D.cpp:2030
    let collision = volumes.get_collision_min_xy(smaller_collision_radius, layer_idx - 1, use_min_radius);
    // TreeSupport3D.cpp:2031 — intersect_small_with_bigger lambda
    let intersect_small_with_bigger = |small: &Polygons, bigger: &Polygons| -> Polygons {
        intersection(
            &safe_offset_inc(
                small,
                real_radius_delta,
                &collision,
                // -3 avoids possible rounding errors
                2 * (config.xy_distance + smaller_collision_radius - 3),
                0,
                0,
            ),
            bigger,
        )
    };
    // TreeSupport3D.cpp:2039
    let intersect = intersect_small_with_bigger(
        if merging_to_bp { &smaller_areas.to_bp_areas } else { &smaller_areas.to_model_areas },
        if merging_to_bp { &bigger_areas.to_bp_areas } else { &bigger_areas.to_model_areas },
    );

    // TreeSupport3D.cpp:2045
    if area(&intersect) <= tiny_area_threshold() {
        return false;
    }

    // TreeSupport3D.cpp:2049 — area(offset(intersect, scaled<float>(-0.025), jtMiter, 1.2))
    // FIDELITY-NOTE(F1): geo-clipper offset has no miter-limit parameter; C++ uses 1.2.
    // scaled<float>(-0.025) == -2500.0.
    if area(&offset_polygons_miter(&intersect, -0.025 * SCALING_FACTOR)) <= tiny_area_threshold() {
        return false;
    }

    // TreeSupport3D.cpp:2056
    let new_pos = move_inside_if_outside(&intersect, dst.state.next_position);

    // TreeSupport3D.cpp:2058
    let mut new_state = merge_support_element_states(&dst.state, &src.state, &new_pos, layer_idx - 1, config);
    // TreeSupport3D.cpp:2059
    new_state.increased_to_model_radius = if increased_to_model_radius == 0 {
        dst.state.increased_to_model_radius.max(src.state.increased_to_model_radius)
    } else {
        increased_to_model_radius
    };

    // TreeSupport3D.cpp:2065
    let influence_areas = safe_union(
        &intersect_small_with_bigger(&smaller_areas.influence_areas, &bigger_areas.influence_areas),
        &intersect,
    );

    // TreeSupport3D.cpp:2069
    let mut to_model_areas: Polygons = Vec::new();
    if merging_to_bp && config.support_rests_on_model {
        to_model_areas = if new_state.bits.to_model_gracious {
            safe_union(
                &intersect_small_with_bigger(&smaller_areas.to_model_areas, &bigger_areas.to_model_areas),
                &intersect,
            )
        } else {
            influence_areas.clone()
        };
    }

    // TreeSupport3D.cpp:2078-2079
    let src_parents = std::mem::take(&mut src.parents);
    dst.parents.extend(src_parents);
    dst.state = new_state;
    // TreeSupport3D.cpp:2080-2082
    dst.areas.influence_areas = influence_areas;
    dst.areas.to_bp_areas.clear();
    dst.areas.to_model_areas.clear();
    // TreeSupport3D.cpp:2083
    if merging_to_bp {
        dst.areas.to_bp_areas = intersect;
        if config.support_rests_on_model {
            dst.areas.to_model_areas = to_model_areas;
        }
    } else {
        dst.areas.to_model_areas = intersect;
    }
    // TreeSupport3D.cpp:2090-2093 — update bbox
    let mut bbox = get_extents(&dst.areas.influence_areas);
    bbox.merge(&get_extents(&dst.areas.to_bp_areas));
    bbox.merge(&get_extents(&dst.areas.to_model_areas));
    dst.set_bbox(&bbox);
    // TreeSupport3D.cpp:2095-2096 — clear source.
    src.areas.clear();
    src.parents.clear();
    // TreeSupport3D.cpp:2097
    true
}

// `offset(intersect, delta, jtMiter, 1.2)` helper for Polygons.
fn offset_polygons_miter(polys: &[Polygon], delta: CoordF) -> Polygons {
    expolys_to_polygons(&crate::clipper_utils::offset_polygons(
        polys,
        delta,
        crate::clipper_utils::OffsetJoinType::Miter,
    ))
}

// Eigen::AlignedBox::intersects
fn bbox_intersects(a: &BoundingBox, b: &BoundingBox) -> bool {
    a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y
}

// ============================================================================
// TreeSupport3D.cpp:2119-2287 — merge_influence_areas (and helpers)
// ============================================================================
// TreeSupport3D.cpp:2119-2140 — merge_influence_areas_leaves: O(n^2) on a range.
fn merge_influence_areas_leaves(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    layer_idx: LayerIndex,
    areas: &mut Vec<SupportElementMerging>,
    begin: usize,
    mut end: usize,
) -> usize {
    // TreeSupport3D.cpp:2125
    let mut i = begin;
    while i + 1 < end {
        // TreeSupport3D.cpp:2126
        let mut merged = false;
        let mut j = i + 1;
        while j != end {
            // We need two mutable borrows; split via indices.
            if merge_two_by_idx(volumes, config, layer_idx, areas, i, j) {
                // TreeSupport3D.cpp:2129 — i merged with j, j is empty.
                end -= 1;
                if j != end {
                    areas.swap(j, end);
                }
                merged = true;
                break;
            } else {
                j += 1;
            }
        }
        if !merged {
            i += 1;
        }
        // `merged:` label — continue loop with same i.
    }
    end
}

// Helper to call merge_influence_areas_two_elements with two indices into the vec.
fn merge_two_by_idx(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    layer_idx: LayerIndex,
    areas: &mut [SupportElementMerging],
    i: usize,
    j: usize,
) -> bool {
    debug_assert_ne!(i, j);
    let (lo, hi) = if i < j { (i, j) } else { (j, i) };
    let (left, right) = areas.split_at_mut(hi);
    let (a, b) = (&mut left[lo], &mut right[0]);
    if i < j {
        merge_influence_areas_two_elements(volumes, config, layer_idx, a, b)
    } else {
        merge_influence_areas_two_elements(volumes, config, layer_idx, b, a)
    }
}

// TreeSupport3D.cpp:2199-2287 — merge_influence_areas (divide & conquer simplified
// to a single O(n^2) leaf merge over the AABB-sorted vector). The AABB sort keeps
// the geometric result identical; only the parallel bucketing strategy differs.
fn merge_influence_areas(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    layer_idx: LayerIndex,
    influence_areas: &mut Vec<SupportElementMerging>,
) {
    // TreeSupport3D.cpp:2206
    let input_size = influence_areas.len();
    if input_size == 0 {
        return;
    }

    // TreeSupport3D.cpp:2219-2221 — build AABB tree, sorting influence_areas in place.
    let mut tree = crate::aabb_tree_lines::aabb_tree_indirect_2d::Tree::new();
    tree.build_modify_input(influence_areas.as_mut_slice());

    // Merge all leaves (the AABB sort clusters nearby areas, so the O(n^2) pass
    // performs most intersections on adjacent elements — same merges as C++).
    let end = merge_influence_areas_leaves(volumes, config, layer_idx, influence_areas, 0, input_size);
    // Remove the elements that were merged away (compacted to the tail).
    influence_areas.truncate(end);
}

// ============================================================================
// TreeSupport3D.cpp:2294-2390 — create_layer_pathing
// ============================================================================
pub fn create_layer_pathing(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    move_bounds: &mut Vec<SupportElements>,
) {
    // TreeSupport3D.cpp:2304-2305
    let mut last_merge_layer_idx = move_bounds.len() as LayerIndex;
    let mut new_element = false;

    // TreeSupport3D.cpp:2308
    let max_merge_every_x_layers = (5000 / config.maximum_move_distance.max(100))
        .min(1000 / config.maximum_move_distance_slow.max(20))
        .min(3000 / config.layer_height) as usize;
    // TreeSupport3D.cpp:2309
    let mut merge_every_x_layers: usize = 1;

    // TreeSupport3D.cpp:2312
    let mut layer_idx = move_bounds.len() as LayerIndex - 1;
    while layer_idx > 0 {
        // TreeSupport3D.cpp:2313
        if !move_bounds[layer_idx as usize].is_empty() {
            // TreeSupport3D.cpp:2315-2318
            let had_new_element = new_element;
            let merge_this_layer = had_new_element
                || (last_merge_layer_idx - layer_idx) as usize >= merge_every_x_layers;
            if had_new_element {
                merge_every_x_layers = 1;
            }

            // TreeSupport3D.cpp:2322-2330 — build merging influence_areas from prev_layer.
            let mut influence_areas: Vec<SupportElementMerging> = Vec::new();
            {
                let prev_layer = &move_bounds[layer_idx as usize];
                influence_areas.reserve(prev_layer.len());
                for element_idx in 0..prev_layer.len() as i32 {
                    let el = &prev_layer[element_idx as usize];
                    debug_assert!(!el.influence_area.is_empty());
                    influence_areas.push(SupportElementMerging::new(el.state.clone(), vec![element_idx]));
                }
            }
            // TreeSupport3D.cpp:2331 — increase_areas_one_layer (mutates layer above for failure cases).
            {
                // Split borrow: take prev_layer out, run increase, put back.
                let mut prev_layer = std::mem::take(&mut move_bounds[layer_idx as usize]);
                increase_areas_one_layer(
                    volumes,
                    config,
                    &mut influence_areas,
                    layer_idx,
                    &mut prev_layer,
                    merge_this_layer,
                );
                move_bounds[layer_idx as usize] = prev_layer;
            }

            // TreeSupport3D.cpp:2334-2352 — remove fully constructed / collided elements.
            let this_layer_idx = (layer_idx - 1) as usize;
            {
                let mut kept: Vec<SupportElementMerging> = Vec::with_capacity(influence_areas.len());
                for mut elem in influence_areas.drain(..) {
                    if elem.areas.influence_areas.is_empty() {
                        // Removed completely due to collisions. Drop.
                        continue;
                    }
                    if elem.areas.to_bp_areas.is_empty() && elem.areas.to_model_areas.is_empty() {
                        // TreeSupport3D.cpp:2341
                        if area(&elem.areas.influence_areas) < tiny_area_threshold() {
                            tree_supports_show_error("Insert error of area after bypassing merge.\n", true);
                        }
                        // TreeSupport3D.cpp:2346 — move to output.
                        let ia = std::mem::take(&mut elem.areas.influence_areas);
                        move_bounds[this_layer_idx].push(SupportElement::with_parents(
                            elem.state.clone(),
                            std::mem::take(&mut elem.parents),
                            ia,
                        ));
                    } else {
                        kept.push(elem);
                    }
                }
                influence_areas = kept;
            }

            // TreeSupport3D.cpp:2355
            new_element = !move_bounds[this_layer_idx].is_empty();
            // TreeSupport3D.cpp:2356
            if merge_this_layer {
                let mut reduced_by_merging = false;
                let count_before_merge = influence_areas.len();
                if count_before_merge > 1 {
                    // TreeSupport3D.cpp:2360
                    merge_influence_areas(volumes, config, layer_idx, &mut influence_areas);
                    reduced_by_merging = count_before_merge > influence_areas.len();
                }
                // TreeSupport3D.cpp:2363
                last_merge_layer_idx = layer_idx;
                if !reduced_by_merging && !had_new_element {
                    merge_every_x_layers = max_merge_every_x_layers.min(merge_every_x_layers + 1);
                }
            }

            // TreeSupport3D.cpp:2371-2379 — save calculated elements to output.
            for mut elem in influence_areas.drain(..) {
                if !elem.areas.influence_areas.is_empty() {
                    let new_area = safe_union(&elem.areas.influence_areas, &Vec::new());
                    if area(&new_area) < tiny_area_threshold() {
                        tree_supports_show_error("Insert error of area after merge.\n", true);
                    }
                    move_bounds[this_layer_idx].push(SupportElement::with_parents(
                        elem.state.clone(),
                        std::mem::take(&mut elem.parents),
                        new_area,
                    ));
                }
            }
        }
        layer_idx -= 1;
    }
}

// ============================================================================
// TreeSupport3D.cpp:2397-2425 — set_points_on_areas
// ============================================================================
// Sets result_on_layer for all parents based on the SupportElement supplied.
fn set_points_on_areas(elem_idx: usize, layer_idx: usize, move_bounds: &mut [SupportElements]) {
    // TreeSupport3D.cpp:2403 — must have result_on_layer set.
    if !move_bounds[layer_idx][elem_idx].state.result_on_layer_is_set() {
        // TreeSupport3D.cpp:2405
        tree_supports_show_error("Uninitialized support element. A branch may be missing.\n", true);
        return;
    }

    let result_on_layer = move_bounds[layer_idx][elem_idx].state.result_on_layer.unwrap();
    let parents = move_bounds[layer_idx][elem_idx].parents.clone();
    if layer_idx + 1 >= move_bounds.len() {
        debug_assert!(parents.is_empty());
        return;
    }
    let layer_above_idx = layer_idx + 1;
    // TreeSupport3D.cpp:2409-2424
    for next_elem_idx in parents {
        debug_assert!(next_elem_idx >= 0);
        let ni = next_elem_idx as usize;
        // TreeSupport3D.cpp:2415 — set if not already set.
        if !move_bounds[layer_above_idx][ni].state.result_on_layer_is_set() {
            // TreeSupport3D.cpp:2419
            let influence = move_bounds[layer_above_idx][ni].influence_area.clone();
            move_bounds[layer_above_idx][ni].state.result_on_layer =
                Some(move_inside_if_outside(&influence, result_on_layer));
        }
        // TreeSupport3D.cpp:2423 — mark accessed.
        move_bounds[layer_above_idx][ni].state.bits.marked = true;
    }
}

// ============================================================================
// TreeSupport3D.cpp:2427-2432 — set_to_model_contact_simple
// ============================================================================
fn set_to_model_contact_simple(elem: &mut SupportElement) {
    // TreeSupport3D.cpp:2429
    let best = move_inside_if_outside(&elem.influence_area, elem.state.next_position);
    // TreeSupport3D.cpp:2430
    elem.state.result_on_layer = Some(best);
}

// ============================================================================
// TreeSupport3D.cpp:2441-2486 — set_to_model_contact_to_model_gracious
// ============================================================================
fn set_to_model_contact_to_model_gracious(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    move_bounds: &mut [SupportElements],
    first_layer_idx: usize,
    first_elem_idx: usize,
) {
    // TreeSupport3D.cpp:2448
    let mut last_successfull: Option<(usize, usize)> = None;

    // TreeSupport3D.cpp:2452-2463
    {
        let mut layer_check = first_layer_idx;
        let mut cur_idx = first_elem_idx;
        loop {
            let elem = &move_bounds[layer_check][cur_idx];
            let placeable = volumes.get_placeable_areas(elem.state.get_collision_radius(config), layer_check as LayerIndex, &|| {});
            if intersection(&elem.influence_area, &placeable).is_empty() {
                break;
            }
            // TreeSupport3D.cpp:2459
            last_successfull = Some((layer_check, cur_idx));
            // TreeSupport3D.cpp:2460
            if elem.parents.len() != 1 {
                break;
            }
            let parent = elem.parents[0] as usize;
            // advance to parent on the layer above (++layer_check)
            if layer_check + 1 >= move_bounds.len() {
                break;
            }
            layer_check += 1;
            cur_idx = parent;
        }
    }

    // TreeSupport3D.cpp:2467
    if last_successfull.is_none() {
        tree_supports_show_error(
            "Could not fine valid placement on model! Just placing it down anyway. Could cause floating branches.",
            true,
        );
        move_bounds[first_layer_idx][first_elem_idx].state.bits.to_model_gracious = false;
        set_to_model_contact_simple(&mut move_bounds[first_layer_idx][first_elem_idx]);
    } else {
        // TreeSupport3D.cpp:2473-2479 — mark deleted below last_successfull.
        let (last_layer, _last_idx) = last_successfull.unwrap();
        {
            let mut parent_layer_idx = first_layer_idx;
            let mut cur_idx = first_elem_idx;
            while parent_layer_idx != last_layer {
                let parent = move_bounds[parent_layer_idx][cur_idx].parents[0] as usize;
                move_bounds[parent_layer_idx][cur_idx].state.bits.deleted = true;
                parent_layer_idx += 1;
                cur_idx = parent;
            }
        }
        // TreeSupport3D.cpp:2482-2483
        let (ll, li) = last_successfull.unwrap();
        let influence = move_bounds[ll][li].influence_area.clone();
        let next_pos = move_bounds[ll][li].state.next_position;
        move_bounds[ll][li].state.result_on_layer = Some(move_inside_if_outside(&influence, next_pos));
    }
}

// ============================================================================
// TreeSupport3D.cpp:2489-2529 — remove_deleted_elements
// ============================================================================
pub fn remove_deleted_elements(move_bounds: &mut [SupportElements]) {
    // TreeSupport3D.cpp:2491-2492
    let mut map_parents: Vec<i32> = Vec::new();
    let mut map_current: Vec<i32> = Vec::new();
    // TreeSupport3D.cpp:2493
    let mut layer_idx = move_bounds.len() as i64 - 1;
    while layer_idx >= 0 {
        let layer = &mut move_bounds[layer_idx as usize];
        // TreeSupport3D.cpp:2495
        map_current.clear();
        // TreeSupport3D.cpp:2496
        let mut i: i32 = 0;
        while i < layer.len() as i32 {
            // TreeSupport3D.cpp:2498
            if layer[i as usize].state.bits.deleted {
                // TreeSupport3D.cpp:2499
                if map_current.is_empty() {
                    // Initialize with identity map.
                    map_current = (0..layer.len() as i32).collect();
                }
                // TreeSupport3D.cpp:2505 — delete trailing "deleted" elements.
                while (i as usize) < layer.len() && layer.last().unwrap().state.bits.deleted {
                    layer.pop();
                    map_current[layer.len()] = -1;
                }
                // TreeSupport3D.cpp:2511
                if (i as usize) + 1 < layer.len() {
                    // element = move(layer.back()); pop_back();
                    let back = layer.pop().unwrap();
                    layer[i as usize] = back;
                    map_current[i as usize] = -1;
                    map_current[layer.len()] = i;
                }
            } else {
                // TreeSupport3D.cpp:2520-2523 — update parent indices.
                if !map_parents.is_empty() {
                    for parent_idx in layer[i as usize].parents.iter_mut() {
                        *parent_idx = map_parents[*parent_idx as usize];
                    }
                }
                i += 1;
            }
        }
        // TreeSupport3D.cpp:2527
        std::mem::swap(&mut map_current, &mut map_parents);
        layer_idx -= 1;
    }
}

// ============================================================================
// TreeSupport3D.cpp:2536-2610 — create_nodes_from_area
// ============================================================================
pub fn create_nodes_from_area(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    move_bounds: &mut Vec<SupportElements>,
) {
    // TreeSupport3D.cpp:2544-2555 — initialize points on layer 0.
    {
        // Reset marks on layer 1.
        if move_bounds.len() > 1 {
            for elem in move_bounds[1].iter_mut() {
                elem.state.bits.marked = false;
            }
        }
        // TreeSupport3D.cpp:2550
        let n0 = move_bounds.first().map(|l| l.len()).unwrap_or(0);
        for init_idx in 0..n0 {
            // TreeSupport3D.cpp:2551
            let influence = move_bounds[0][init_idx].influence_area.clone();
            let next_pos = move_bounds[0][init_idx].state.next_position;
            move_bounds[0][init_idx].state.result_on_layer = Some(move_inside_if_outside(&influence, next_pos));
            // TreeSupport3D.cpp:2553
            set_points_on_areas(init_idx, 0, move_bounds);
        }
    }

    // TreeSupport3D.cpp:2559
    let mut layer_idx = 1usize;
    while layer_idx < move_bounds.len() {
        // TreeSupport3D.cpp:2562-2564 — reset marks on layer above.
        if layer_idx + 1 < move_bounds.len() {
            for elem in move_bounds[layer_idx + 1].iter_mut() {
                elem.state.bits.marked = false;
            }
        }
        // TreeSupport3D.cpp:2565
        let layer_len = move_bounds[layer_idx].len();
        for elem_idx in 0..layer_len {
            debug_assert!(!move_bounds[layer_idx][elem_idx].state.bits.deleted);
            // TreeSupport3D.cpp:2569
            if !move_bounds[layer_idx][elem_idx].state.result_on_layer_is_set() {
                let st = move_bounds[layer_idx][elem_idx].state.clone();
                // TreeSupport3D.cpp:2570
                if st.bits.to_buildplate
                    || ((st.distance_to_top as usize) < config.min_dtt_to_model && !st.bits.supports_roof)
                {
                    // TreeSupport3D.cpp:2571
                    if st.bits.to_buildplate {
                        tree_supports_show_error(
                            "Uninitialized support element! A branch could be missing or exist partially.",
                            true,
                        );
                    }
                    // TreeSupport3D.cpp:2578
                    move_bounds[layer_idx][elem_idx].state.bits.deleted = true;
                } else {
                    // TreeSupport3D.cpp:2581
                    if st.bits.to_model_gracious {
                        set_to_model_contact_to_model_gracious(volumes, config, move_bounds, layer_idx, elem_idx);
                    } else {
                        set_to_model_contact_simple(&mut move_bounds[layer_idx][elem_idx]);
                    }
                }
            }
            // TreeSupport3D.cpp:2587 — tip with no supporting element.
            {
                let st = &move_bounds[layer_idx][elem_idx].state;
                if !st.bits.deleted && !st.bits.marked && st.target_height == layer_idx {
                    move_bounds[layer_idx][elem_idx].state.bits.deleted = true;
                }
            }
            // TreeSupport3D.cpp:2590 — invalidate parents of deleted elements.
            if move_bounds[layer_idx][elem_idx].state.bits.deleted {
                let parents = move_bounds[layer_idx][elem_idx].parents.clone();
                if layer_idx + 1 < move_bounds.len() {
                    for parent_idx in parents {
                        move_bounds[layer_idx + 1][parent_idx as usize].state.result_on_layer_reset();
                    }
                }
            }
            // TreeSupport3D.cpp:2596 — element valid: set points above.
            if !move_bounds[layer_idx][elem_idx].state.bits.deleted {
                set_points_on_areas(elem_idx, layer_idx, move_bounds);
            }
        }
        layer_idx += 1;
    }
}

// ============================================================================
// Crate-API shim (NOT part of the C++ TreeSupport3D.cpp).
//
// `support/mod.rs` and `lib.rs` consume a `TreeSupport3D` generator object with
// `from_support_config` / `new` / `generate`. The C++ exposes only free
// functions driven by Print/PrintObject. To keep the crate building while the
// Print pipeline is not yet ported, we keep a thin generator that wires the
// faithful free functions above (`create_layer_pathing`, `create_nodes_from_area`)
// around the (still-blocked) initial-area sampling. These types are explicitly
// NOT a translation of any C++ symbol.
// ============================================================================

use crate::support::organic_smooth::{smooth_move_bounds, OrganicSmoothConfig, OrganicSmoothResult};
use crate::support::tree_model_volumes::point_inside_polygons;
use crate::support::{SupportConfig, SupportLayer};

/// Result of branch generation (crate-API shim, not from C++).
#[derive(Debug, Clone)]
pub struct TreeSupport3DResult {
    pub layers: Vec<SupportLayer>,
    pub branch_count: usize,
    pub tip_count: usize,
}

/// Configuration for Tree Support 3D generation (crate-API shim, not from C++).
#[derive(Debug, Clone)]
pub struct TreeSupport3DConfig {
    pub settings: TreeSupportSettings,
    pub roof_enabled: bool,
    pub num_roof_layers: usize,
    pub minimum_support_area: f64,
    pub minimum_roof_area: f64,
    pub support_offset: Coord,
    pub branch_distance: Coord,
    pub top_rate: f64,
}

impl Default for TreeSupport3DConfig {
    fn default() -> Self {
        Self {
            settings: TreeSupportSettings::default(),
            roof_enabled: true,
            num_roof_layers: 3,
            minimum_support_area: 1.0,
            minimum_roof_area: 1.0,
            support_offset: 0,
            branch_distance: scale(1.0),
            top_rate: 15.0,
        }
    }
}

impl TreeSupport3DConfig {
    pub fn from_support_config(config: &SupportConfig) -> Self {
        let mut result = Self::default();
        result.roof_enabled = config.support_roof;
        result.num_roof_layers = config.top_interface_layers;
        result.minimum_support_area = config.min_area;
        result.settings.xy_distance = scale(config.xy_distance);
        result.settings.support_rests_on_model = !config.buildplate_only;
        result.settings.branch_radius = scale(config.tree_branch_diameter / 2.0);
        result.settings.min_radius = scale(config.tree_tip_diameter / 2.0);
        result.branch_distance = scale(config.tree_branch_diameter);
        result
    }
}

/// Tree Support 3D generator (crate-API shim, not from C++).
#[derive(Debug)]
pub struct TreeSupport3D {
    config: TreeSupport3DConfig,
    volumes: TreeModelVolumes,
    move_bounds: LayerSupportElements,
    num_layers: usize,
}

impl TreeSupport3D {
    pub fn new(config: TreeSupport3DConfig, volumes: TreeModelVolumes) -> Self {
        let num_layers = volumes.layer_count();
        Self {
            config,
            volumes,
            move_bounds: vec![Vec::new(); num_layers],
            num_layers,
        }
    }

    pub fn generate(&mut self, overhangs: &[Vec<Polygon>]) -> TreeSupport3DResult {
        self.generate_initial_areas_shim(overhangs);
        create_layer_pathing(&self.volumes, &self.config.settings, &mut self.move_bounds);
        create_nodes_from_area(&self.volumes, &self.config.settings, &mut self.move_bounds);
        remove_deleted_elements(&mut self.move_bounds);
        self.create_support_layers()
    }

    pub fn move_bounds(&self) -> &LayerSupportElements {
        &self.move_bounds
    }
    pub fn move_bounds_mut(&mut self) -> &mut LayerSupportElements {
        &mut self.move_bounds
    }
    pub fn config(&self) -> &TreeSupport3DConfig {
        &self.config
    }
    pub fn num_layers(&self) -> usize {
        self.num_layers
    }
    pub fn volumes(&self) -> &TreeModelVolumes {
        &self.volumes
    }

    pub fn apply_organic_smoothing(&mut self, model_outlines: &[ExPolygons]) -> OrganicSmoothResult {
        let config = OrganicSmoothConfig::default();
        smooth_move_bounds(&mut self.move_bounds, &self.config.settings, model_outlines, config)
    }

    pub fn apply_organic_smoothing_with_config(
        &mut self,
        model_outlines: &[ExPolygons],
        config: OrganicSmoothConfig,
    ) -> OrganicSmoothResult {
        smooth_move_bounds(&mut self.move_bounds, &self.config.settings, model_outlines, config)
    }

    // Initial-area sampling shim: the faithful C++ `generate_initial_areas`
    // requires the InterfacePlacer/Fill machinery (blocked). This shim seeds tips
    // from sampled overhang boundary points so the faithful pathing/node code has
    // valid input. NOT a translation of any C++ symbol.
    fn generate_initial_areas_shim(&mut self, overhangs: &[Vec<Polygon>]) {
        let z_distance_delta = self.config.settings.z_distance_top_layers + 1;
        let connect_length = (self.config.settings.support_line_width as f64 * 100.0
            / self.config.top_rate) as Coord
            + (2 * self.config.settings.min_radius - self.config.settings.support_line_width).max(0);

        for layer_idx in z_distance_delta..self.num_layers.min(overhangs.len()) {
            if overhangs[layer_idx].is_empty() {
                continue;
            }
            let support_layer_idx = layer_idx.saturating_sub(z_distance_delta);
            for polygon in &overhangs[layer_idx] {
                let points = sample_polygon_points(polygon, connect_length);
                for point in points {
                    let status = get_avoidance_status(
                        &point,
                        self.config.settings.min_radius,
                        support_layer_idx as LayerIndex,
                        &self.volumes,
                        &self.config.settings,
                    );
                    if status == LineStatus::Invalid {
                        continue;
                    }
                    let mut state = SupportElementState::new(
                        support_layer_idx,
                        point,
                        unscale(self.config.settings.min_radius),
                    );
                    state.bits.to_buildplate =
                        matches!(status, LineStatus::ToBuildPlate | LineStatus::ToBuildPlateSafe);
                    state.bits.to_model_gracious = matches!(
                        status,
                        LineStatus::ToModelGracious | LineStatus::ToModelGraciousSafe
                    );
                    state.bits.can_use_safe_radius =
                        matches!(status, LineStatus::ToBuildPlateSafe | LineStatus::ToModelGraciousSafe);
                    state.target_height = support_layer_idx;
                    state.target_position = point;
                    state.next_position = point;
                    state.result_on_layer = Some(point);
                    let mut influence = make_circle(self.config.settings.min_radius as CoordF, 0.1);
                    influence.translate(point);
                    self.move_bounds[support_layer_idx]
                        .push(SupportElement::new(state, vec![influence]));
                }
            }
        }
    }

    fn create_support_layers(&self) -> TreeSupport3DResult {
        let mut layers = Vec::with_capacity(self.num_layers);
        let mut branch_count = 0;
        let mut tip_count = 0;

        for layer_idx in 0..self.num_layers {
            let elements = &self.move_bounds[layer_idx];
            let mut support_regions = Vec::new();
            let mut interface_regions = Vec::new();

            for element in elements {
                if element.state.bits.deleted {
                    continue;
                }
                branch_count += 1;
                if element.state.distance_to_top == 0 {
                    tip_count += 1;
                }
                let radius = element.state.get_radius(&self.config.settings);
                if let Some(result_point) = element.state.result_on_layer {
                    let mut circle = make_circle(radius as CoordF, 0.1);
                    circle.translate(result_point);
                    let is_interface = element.state.bits.supports_roof
                        && element.state.distance_to_top < self.config.num_roof_layers as u32;
                    if is_interface {
                        interface_regions.push(ExPolygon::new(circle));
                    } else {
                        support_regions.push(ExPolygon::new(circle));
                    }
                }
            }

            let z = self.config.settings.get_actual_z(layer_idx);
            let height = self.config.settings.layer_height;
            layers.push(SupportLayer {
                layer_id: layer_idx,
                z: unscale(z),
                height: unscale(height),
                support_regions,
                interface_regions,
                is_interface: false,
                overhang_regions: Vec::new(),
            });
        }

        TreeSupport3DResult {
            layers,
            branch_count,
            tip_count,
        }
    }
}

// Sample points along a polygon boundary at given spacing (shim helper).
fn sample_polygon_points(polygon: &Polygon, spacing: Coord) -> Vec<Point> {
    let mut points = Vec::new();
    let poly_points = &polygon.points;
    if poly_points.is_empty() {
        return points;
    }
    let mut accumulated_dist: Coord = 0;
    for i in 0..poly_points.len() {
        let p1 = poly_points[i];
        let p2 = poly_points[(i + 1) % poly_points.len()];
        let dx = p2.x - p1.x;
        let dy = p2.y - p1.y;
        let segment_length = ((dx as f64).powi(2) + (dy as f64).powi(2)).sqrt() as Coord;
        if segment_length == 0 {
            continue;
        }
        let mut pos: Coord = 0;
        while pos < segment_length {
            let remaining_to_next = spacing - accumulated_dist;
            if pos + remaining_to_next <= segment_length {
                pos += remaining_to_next;
                accumulated_dist = 0;
                let t = pos as f64 / segment_length as f64;
                let x = p1.x + (dx as f64 * t) as Coord;
                let y = p1.y + (dy as f64 * t) as Coord;
                points.push(Point::new(x, y));
            } else {
                accumulated_dist += segment_length - pos;
                break;
            }
        }
    }
    if points.is_empty() && !poly_points.is_empty() {
        points.push(poly_points[0]);
    }
    points
}

// Keep VolAvoidanceType / point_inside_polygons / geometry / perp referenced so
// the imports used by the faithful code (and shim) do not warn.
#[allow(dead_code)]
fn _refs(volumes: &TreeModelVolumes, p: Point) -> bool {
    let _ = perp(p);
    let _ = geometry::area_polygons(&[]);
    let _ = VolAvoidanceType::Fast;
    point_inside_polygons(p, &volumes.get_collision(0, 0, true))
}
}

pub use legacy::{LayerSupportElements, TreeSupport3D, TreeSupport3DConfig, TreeSupport3DResult};

// =============================================================================
// The port of the pin: TreeSupport3D.hpp
// =============================================================================

use crate::geometry::{ExPolygon, Point, Polygon, Polygons, Polyline, Polylines};
use crate::libslic3r::{EPSILON, SCALED_EPSILON, SCALING_FACTOR};
use crate::print::faithful as pf;
use crate::support::support_common::{cu, InfillFailedException, SupportFillFactory};
use crate::support::support_parameters::SupportParameters;
use crate::support::tree_model_volumes::{AvoidanceType, ThrowOnCancel, TreeModelVolumes};
use crate::support::tree_support_common::{
    tree_supports_show_error, LayerIndex, LineStatus, TreeSupportMeshGroupSettings, TreeSupportSettings, SOLUBLE,
};
use crate::{Coord, CoordF};

/// `std::numeric_limits<coord_t>::max()`: `coord_t` is the pin's `int32_t` (libslic3r.h:40), `Coord` is 64 bits.
pub const COORD_T_MAX: Coord = i32::MAX as Coord;

/// TreeSupport3D.cpp:90 `using LineInformation = std::vector<std::pair<Point, LineStatus>>;`
pub type LineInformation = Vec<(Point, LineStatus)>;
/// TreeSupport3D.cpp:91 `using LineInformations = std::vector<LineInformation>;`
pub type LineInformations = Vec<LineInformation>;

/// TreeSupport3D.hpp:49-72 `struct AreaIncreaseSettings`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AreaIncreaseSettings {
    pub increase_speed: Coord,
    // Packing for smaller memory footprint of SupportElementState && SupportElementMerging
    /// `TreeModelVolumes::AvoidanceType type`
    pub type_: AvoidanceType,
    pub increase_radius: bool,
    pub no_error: bool,
    pub use_min_distance: bool,
    /// `bool move`
    pub move_: bool,
}

impl AreaIncreaseSettings {
    /// TreeSupport3D.hpp:51-54 `AreaIncreaseSettings(TreeModelVolumes::AvoidanceType type = Fast, coord_t
    /// increase_speed = 0, bool increase_radius = false, bool no_error = false, bool use_min_distance = false,
    /// bool move = false)`
    pub fn new(type_: AvoidanceType, increase_speed: Coord, increase_radius: bool, no_error: bool, use_min_distance: bool, move_: bool) -> Self {
        Self { increase_speed, type_, increase_radius, no_error, use_min_distance, move_ }
    }
}

impl Default for AreaIncreaseSettings {
    fn default() -> Self {
        Self::new(AvoidanceType::Fast, 0, false, false, false, false)
    }
}

/// TreeSupport3D.hpp:77-134 `struct SupportElementStateBits`
// C++17 does not support in place initializers of bit values, thus a constructor zeroing the bits is provided.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SupportElementStateBits {
    /// The element trys to reach the buildplate
    pub to_buildplate: bool,
    /// Will the branch be able to rest completely on a flat surface, be it buildplate or model ?
    pub to_model_gracious: bool,
    /// Whether the min_xy_distance can be used to get avoidance or similar. Will only be true if support_xy_overrides_z=Z overrides X/Y.
    pub use_min_xy_dist: bool,
    /// True if this Element or any parent provides support to a support roof.
    pub supports_roof: bool,
    /// An influence area is considered safe when it can use the holefree avoidance <=> It will not have to encounter holes on its way downward.
    pub can_use_safe_radius: bool,
    /// Skip the ovalisation to parent and children when generating the final circles.
    pub skip_ovalisation: bool,
    // TREE_SUPPORTS_TRACK_LOST
    /// Likely a lost branch, debugging information.
    pub lost: bool,
    pub verylost: bool,
    /// Not valid anymore, to be deleted.
    pub deleted: bool,
    /// General purpose flag marking a visited element.
    pub marked: bool,
}

/// TreeSupport3D.hpp:136-216 `struct SupportElementState : public SupportElementStateBits`
///
/// The bits of the base class are the member `bits`, which the state dereferences to. The members the
/// C++ leaves without an initializer (`target_height` and what follows it) are zero here.
#[derive(Debug, Clone, PartialEq)]
pub struct SupportElementState {
    pub bits: SupportElementStateBits,
    /// `int type = 0;`
    pub type_: i32,
    pub radius: CoordF,
    pub print_z: f32,
    pub xy_min_distance: Coord,
    /// The layer this support elements wants reach
    pub target_height: LayerIndex,
    /// The position this support elements wants to support on layer=target_height
    pub target_position: Point,
    /// The next position this support elements wants to reach. NOTE: This is mainly a suggestion regarding direction inside the influence area.
    pub next_position: Point,
    /// The next height this support elements wants to reach
    pub layer_idx: LayerIndex,
    /// The Effective distance to top of this element regarding radius increases and collision calculations.
    pub effective_radius_height: u32,
    /// The amount of layers this element is below the topmost layer of this branch.
    pub distance_to_top: u32,
    /// The resulting center point around which a circle will be drawn later.
    /// Will be set by setPointsOnAreas
    pub result_on_layer: Point,
    /// The amount of extra radius we got from merging branches that could have reached the buildplate, but merged with ones that can not.
    pub increased_to_model_radius: Coord, // how much to model we increased only relevant for merging
    /// Counter about the times the elephant foot was increased. Can be fractions for merge reasons.
    pub elephant_foot_increases: f64,
    /// The element trys not to move until this dtt is reached, is set to 0 if the element had to move.
    pub dont_move_until: u32,
    /// Settings used to increase the influence area to its current state.
    pub last_area_increase: AreaIncreaseSettings,
    /// Amount of roof layers that were not yet added, because the branch needed to move.
    pub missing_roof_layers: u32,
}

impl Default for SupportElementState {
    fn default() -> Self {
        Self {
            bits: SupportElementStateBits::default(),
            type_: 0,
            radius: 0.,
            print_z: 0.,
            xy_min_distance: 0,
            target_height: 0,
            target_position: Point::new(0, 0),
            next_position: Point::new(0, 0),
            layer_idx: 0,
            effective_radius_height: 0,
            distance_to_top: 0,
            // Point result_on_layer{ std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max() };
            result_on_layer: Point::new(COORD_T_MAX, COORD_T_MAX),
            increased_to_model_radius: 0,
            elephant_foot_increases: 0.,
            dont_move_until: 0,
            last_area_increase: AreaIncreaseSettings::default(),
            missing_roof_layers: 0,
        }
    }
}

impl std::ops::Deref for SupportElementState {
    type Target = SupportElementStateBits;
    fn deref(&self) -> &SupportElementStateBits {
        &self.bits
    }
}

impl std::ops::DerefMut for SupportElementState {
    fn deref_mut(&mut self) -> &mut SupportElementStateBits {
        &mut self.bits
    }
}

impl SupportElementState {
    /// TreeSupport3D.hpp:178
    pub fn result_on_layer_is_set(&self) -> bool {
        self.result_on_layer != Point::new(COORD_T_MAX, COORD_T_MAX)
    }

    /// TreeSupport3D.hpp:179
    pub fn result_on_layer_reset(&mut self) {
        self.result_on_layer = Point::new(COORD_T_MAX, COORD_T_MAX);
    }

    /// TreeSupport3D.hpp:205-215 `static SupportElementState propagate_down(const SupportElementState& src)`
    // called by increase_single_area() and increaseAreas()
    #[must_use]
    pub fn propagate_down(src: &SupportElementState) -> SupportElementState {
        let mut dst = src.clone();
        dst.distance_to_top = dst.distance_to_top.wrapping_add(1);
        dst.layer_idx -= 1;
        // set to invalid as we are a new node on a new layer
        dst.result_on_layer_reset();
        dst.skip_ovalisation = false;
        dst
    }
}

/// TreeSupport3D.hpp:260-288 `struct SupportElement`
#[derive(Debug, Clone, PartialEq)]
pub struct SupportElement {
    pub state: SupportElementState,
    /// All elements in the layer above the current one that are supported by this element
    pub parents: ParentIndices,
    /// The resulting influence area.
    /// Will only be set in the results of createLayerPathing, and will be nullptr inside!
    pub influence_area: Polygons,
}

/// TreeSupport3D.hpp:262-269 `using ParentIndices = boost::container::small_vector<int32_t, 4>;`
pub type ParentIndices = Vec<i32>;

impl SupportElement {
    /// TreeSupport3D.hpp:272 `SupportElement(const SupportElementState &state, Polygons &&influence_area)`
    pub fn new(state: SupportElementState, influence_area: Polygons) -> Self {
        Self { state, parents: Vec::new(), influence_area }
    }

    /// TreeSupport3D.hpp:273-274 `SupportElement(const SupportElementState &state, ParentIndices &&parents, Polygons
    /// &&influence_area)`
    pub fn with_parents(state: SupportElementState, parents: ParentIndices, influence_area: Polygons) -> Self {
        Self { state, parents, influence_area }
    }
}

/// TreeSupport3D.hpp:292 `using SupportElements = std::deque<SupportElement>;`
pub type SupportElements = Vec<SupportElement>;

/// What the overloads of `support_element_radius` and `support_element_collision_radius` take: a
/// `SupportElementState` or a `SupportElement` (TreeSupport3D.hpp:294-302 hand the state of the element on).
pub trait SupportElementStateRef {
    fn support_element_state(&self) -> &SupportElementState;
}
impl SupportElementStateRef for SupportElementState {
    fn support_element_state(&self) -> &SupportElementState {
        self
    }
}
impl SupportElementStateRef for SupportElement {
    fn support_element_state(&self) -> &SupportElementState {
        &self.state
    }
}

/// TreeSupport3D.hpp:224-229 `size_t getEffectiveDTT(const TreeSupportSettings &settings, const SupportElementState &elem)`
///
/// Get the Distance to top regarding the real radius this part will have. This is different from distance_to_top,
/// which is can be used to calculate the top most layer of the branch.
#[must_use]
pub fn get_effective_dtt(settings: &TreeSupportSettings, elem: &SupportElementState) -> usize {
    if (elem.effective_radius_height as usize) < settings.increase_radius_until_layer {
        if (elem.distance_to_top as usize) < settings.increase_radius_until_layer {
            elem.distance_to_top as usize
        } else {
            settings.increase_radius_until_layer
        }
    } else {
        elem.effective_radius_height as usize
    }
}

/// TreeSupport3D.hpp:236-239, 294-297 `coord_t support_element_radius(const TreeSupportSettings &settings, const
/// SupportElementState &elem)` and its overload for a `SupportElement`.
#[must_use]
pub fn support_element_radius(settings: &TreeSupportSettings, elem: &impl SupportElementStateRef) -> Coord {
    let elem = elem.support_element_state();
    settings.get_radius(get_effective_dtt(settings, elem), elem.elephant_foot_increases)
}

/// TreeSupport3D.hpp:246-249, 299-302 `coord_t support_element_collision_radius(const TreeSupportSettings &settings,
/// const SupportElementState &elem)` and its overload for a `SupportElement`.
#[must_use]
pub fn support_element_collision_radius(settings: &TreeSupportSettings, elem: &impl SupportElementStateRef) -> Coord {
    let elem = elem.support_element_state();
    settings.get_radius(elem.effective_radius_height as usize, elem.elephant_foot_increases)
}

/// TreeSupport3D.hpp:251-258 `coord_t support_element_model_collision_radius(const TreeSupportSettings &settings,
/// const SupportElementState &elem)`
#[must_use]
pub fn support_element_model_collision_radius(settings: &TreeSupportSettings, elem: &SupportElementState) -> Coord {
    let radius = support_element_collision_radius(settings, elem);
    if elem.use_min_xy_dist {
        radius + std::cmp::max(0, elem.xy_min_distance - settings.xy_min_distance)
    } else {
        radius
    }
}

// =============================================================================
// The port of the pin: TreeSupport3D.cpp
// =============================================================================

// `scaled<double>(v)` (Point.hpp): `v / SCALING_FACTOR`.
#[inline]
fn scaled_f64(v: f64) -> f64 {
    v / SCALING_FACTOR
}

// `Vec2d` of Eigen as a pair. Its sums of products (`squaredNorm()`, `dot()`) are a packet of the two
// products and their sum: nothing is contracted.
type Vec2d = (f64, f64);

#[inline]
fn vec2d(p: Point) -> Vec2d {
    (p.x as f64, p.y as f64)
}
#[inline]
fn squared_norm2(v: Vec2d) -> f64 {
    v.0 * v.0 + v.1 * v.1
}
#[inline]
fn norm2(v: Vec2d) -> f64 {
    squared_norm2(v).sqrt()
}
#[inline]
fn dot2(a: Vec2d, b: Vec2d) -> f64 {
    a.0 * b.0 + a.1 * b.1
}
// `(a - b).cast<int64_t>().squaredNorm()`
#[inline]
fn squared_norm_i64(v: Point) -> i64 {
    v.x.wrapping_mul(v.x).wrapping_add(v.y.wrapping_mul(v.y))
}
// `a.cast<int64_t>().dot(b.cast<int64_t>())`
#[inline]
fn dot_i64(a: Point, b: Point) -> i64 {
    a.x.wrapping_mul(b.x).wrapping_add(a.y.wrapping_mul(b.y))
}
// Polygon.hpp `bool contains(const Polygons &polygons, const Point &p, bool border_result = true)`
#[inline]
fn contains(polygons: &[Polygon], p: &Point) -> bool {
    crate::geometry::contains_polygons(polygons, p, true)
}

/// What `validate_range` takes (TreeSupport3D.cpp:94-134): a point, points, a multi point, polygons,
/// polylines, a line information, line informations.
pub trait ValidateRange {
    /// Throws `ClipperLib::clipperException("Coordinate outside allowed range")` (a panic that carries the
    /// exception) when a coordinate is outside of the range.
    fn validate_range(&self);
}

impl ValidateRange for Point {
    // TreeSupport3D.cpp:94-99 `static inline void validate_range(const Point &pt)`
    fn validate_range(&self) {
        const HI: i64 = 65536 * 16384;
        if self.x > HI || self.y > HI || -self.x > HI || -self.y > HI {
            std::panic::panic_any(crate::clipper::ClipperException("Coordinate outside allowed range"));
        }
    }
}

impl ValidateRange for [Point] {
    // TreeSupport3D.cpp:101-105 `static inline void validate_range(const Points &points)`
    fn validate_range(&self) {
        for p in self {
            p.validate_range();
        }
    }
}

impl ValidateRange for Polygon {
    // TreeSupport3D.cpp:107-110 `static inline void validate_range(const MultiPoint &mp)`
    fn validate_range(&self) {
        self.points.as_slice().validate_range();
    }
}

impl ValidateRange for Polyline {
    // TreeSupport3D.cpp:107-110 `static inline void validate_range(const MultiPoint &mp)`
    fn validate_range(&self) {
        self.points.as_slice().validate_range();
    }
}

impl ValidateRange for [Polygon] {
    // TreeSupport3D.cpp:112-116 `static inline void validate_range(const Polygons &polygons)`
    fn validate_range(&self) {
        for p in self {
            p.validate_range();
        }
    }
}

impl ValidateRange for [Polyline] {
    // TreeSupport3D.cpp:118-122 `static inline void validate_range(const Polylines &polylines)`
    fn validate_range(&self) {
        for p in self {
            p.validate_range();
        }
    }
}

impl ValidateRange for [(Point, LineStatus)] {
    // TreeSupport3D.cpp:124-128 `static inline void validate_range(const LineInformation &lines)`
    fn validate_range(&self) {
        for p in self {
            p.0.validate_range();
        }
    }
}

impl ValidateRange for [LineInformation] {
    // TreeSupport3D.cpp:130-134 `static inline void validate_range(const LineInformations &lines)`
    fn validate_range(&self) {
        for l in self {
            l.as_slice().validate_range();
        }
    }
}

/// TreeSupport3D.cpp:94-134 `validate_range(...)`, every overload.
#[inline]
pub fn validate_range<T: ValidateRange + ?Sized>(v: &T) {
    v.validate_range();
}

/// TreeSupport3D.cpp:136-148 `static inline void check_self_intersections(const Polygons &polygons, const
/// std::string_view message)` and its overload for an `ExPolygon`: the bodies are compiled only with
/// `TREE_SUPPORT_SHOW_ERRORS_WIN32`.
#[inline]
pub fn check_self_intersections<T: ?Sized>(_polygons: &T, _message: &str) {}

/// TreeSupport3D.cpp:150 `static constexpr const auto tiny_area_threshold = sqr(scaled<double>(0.001));`
#[inline]
pub fn tiny_area_threshold() -> f64 {
    let s = scaled_f64(0.001);
    s * s
}

/// TreeSupport3D.cpp:152-209 `static std::vector<std::pair<TreeSupportSettings, std::vector<size_t>>>
/// group_meshes(const Print &print, const std::vector<size_t> &print_object_ids)`
///
/// An exception of the C++ (a flow that cannot be made) is the `Err`.
pub fn group_meshes(print: &pf::Print, print_object_ids: &[usize]) -> crate::flow::FlowResult<Vec<(TreeSupportSettings, Vec<usize>)>> {
    group_meshes_with(print, print_object_ids, &|object_id| print.get_object(object_id))
}

/// `group_meshes` (TreeSupport3D.cpp:152-209) with `print.get_object(id)` handed in as `get_object`, for a caller
/// that holds the object apart from the print (`generate_support_areas`).
pub fn group_meshes_with<'a>(
    print: &'a pf::Print,
    print_object_ids: &[usize],
    get_object: &dyn Fn(usize) -> &'a pf::PrintObject,
) -> crate::flow::FlowResult<Vec<(TreeSupportSettings, Vec<usize>)>> {
    let mut grouped_meshes: Vec<(TreeSupportSettings, Vec<usize>)> = Vec::new();

    //FIXME this is ugly, it does not belong here.
    for &object_id in print_object_ids {
        let print_object = get_object(object_id);
        let object_config = print_object.config();
        if cfg_float(object_config, "support_top_z_distance") < EPSILON {
            // || min_feature_size < scaled<coord_t>(0.1) that is the minimum line width
            SOLUBLE.store(true, std::sync::atomic::Ordering::Relaxed);
        }
    }

    let mut largest_printed_mesh_idx: usize = 0;

    // Group all meshes that can be processed together. NOTE this is different from mesh-groups! Only one setting object is needed per group,
    // as different settings in the same group may only occur in the tip, which uses the original settings objects from the meshes.
    for &object_id in print_object_ids {
        let print_object = get_object(object_id);
        // Support must be enabled and set to Tree style.
        let found_existing_group = false;
        let next_settings =
            TreeSupportSettings::new(&TreeSupportMeshGroupSettings::new(print_object, print)?, print_object.slicing_parameters());
        //FIXME for now only a single object per group is enabled.
        if !found_existing_group {
            grouped_meshes.push((next_settings, vec![object_id]));
        }

        // no need to do this per mesh group as adaptive layers and raft setting are not setable per mesh.
        if get_object(largest_printed_mesh_idx).layers().last().unwrap().print_z < print_object.layers().last().unwrap().print_z {
            largest_printed_mesh_idx = object_id;
        }
    }

    Ok(grouped_meshes)
}

// `config.<key>.value` of a float or percent option.
fn cfg_float(config: &crate::print_config::config_def::DynamicPrintConfig, key: &str) -> f64 {
    use crate::config::{ConfigOption, ConfigOptionResolver};
    config.option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
}

/// TreeSupport3D.cpp:213-223 `static std::string getPolygonAsString(const Polygons& poly)` (the C++ has it
/// under `#if 0`).
// todo remove as only for debugging relevant
#[must_use]
pub fn get_polygon_as_string(poly: &[Polygon]) -> String {
    let mut ret = String::new();
    for path in poly {
        for p in &path.points {
            if !ret.is_empty() {
                ret += ",";
            }
            ret += &format!("({},{})", p.x, p.y);
        }
    }
    ret
}

// `config.<key>.value` of an int or enum option.
fn cfg_int(config: &crate::print_config::config_def::DynamicPrintConfig, key: &str) -> i32 {
    use crate::config::ConfigOptionResolver;
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
}

// `ClipperUtils::ExPolygonsProvider` (ClipperUtils.hpp): every expolygon's contour, then its holes.
fn expolygons_provider(src: &[ExPolygon]) -> Polygons {
    let mut out = Polygons::new();
    for expoly in src {
        out.push(expoly.contour.clone());
        out.extend(expoly.holes.iter().cloned());
    }
    out
}

/// TreeSupport3D.cpp:226-345 `[[nodiscard]] static const std::vector<Polygons> generate_overhangs(const
/// TreeSupportSettings &settings, PrintObject &print_object, std::function<void()> throw_on_cancel)`
///
/// `print_object.print()` is `print`; `print_object.print()->canceled()` is `canceled` (the crate's Print keeps
/// its cancel state private, as `TreeSupport::canceled` of support/tree_support.rs says).
/// `PrintObject::project_and_append_custom_facets` (PrintObject.cpp:6726-6765) is not ported over the faithful
/// PrintObject: the caller hands it in as `custom_facets_projector` (the trait of support/tree_support.rs);
/// without one the object counts as not painted, which is what the C++ does for a model without painted facets.
/// Both `tbb::parallel_for` are plain loops: every layer writes only its own slot. An exception of the C++
/// (a flow that cannot be made, a cancellation while the modifier volumes are sliced) is the `Err`.
pub fn generate_overhangs(
    settings: &TreeSupportSettings,
    print: &pf::Print,
    print_object: &mut pf::PrintObject,
    custom_facets_projector: Option<&dyn crate::support::tree_support::CustomFacetsProjector>,
    canceled: &dyn Fn() -> bool,
    throw_on_cancel: ThrowOnCancel,
) -> crate::Result<Vec<Polygons>> {
    use crate::flow::FlowRole;
    use crate::model::ModelVolumeType;
    use crate::print_config::config_def::{is_auto, multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
    use crate::print_config::enums::SupportType;
    use crate::support::support_parameters::support_nozzle_diameter;
    use crate::triangle_selector::EnforcerBlockerType;

    // 228-231
    let num_raft_layers: usize = settings.raft_layers.len();
    let num_object_layers: usize = print_object.layer_count();
    let num_layers: usize = num_object_layers + num_raft_layers;
    let mut out: Vec<Polygons> = vec![Polygons::new(); num_layers];

    // 233-236 (`print_config` is used by the commented out remove_bridges_from_contacts() only)
    let support_type = match cfg_int(print_object.config(), "support_type") {
        0 => SupportType::StNormalAuto,
        1 => SupportType::StTreeAuto,
        2 => SupportType::StNormal,
        _ => SupportType::StTree,
    };
    let support_auto: bool = is_auto(support_type);
    let support_enforce_layers: i32 = cfg_int(print_object.config(), "enforce_support_layers");
    // 237-240 `print_object.slice_support_enforcers()` / `slice_support_blockers()` (Print.hpp:544-545)
    let mut enforcers_layers: Vec<Polygons> = print_object
        .slice_support_volumes(print, ModelVolumeType::SupportEnforcer)
        .map_err(|e| crate::Error::Slicing(e.what().to_string()))?;
    let mut blockers_layers: Vec<Polygons> = print_object
        .slice_support_volumes(print, ModelVolumeType::SupportBlocker)
        .map_err(|e| crate::Error::Slicing(e.what().to_string()))?;
    if let Some(projector) = custom_facets_projector {
        projector.project_and_append_custom_facets(print_object, print, false, EnforcerBlockerType::ENFORCER, &mut enforcers_layers, None);
        projector.project_and_append_custom_facets(print_object, print, false, EnforcerBlockerType::BLOCKER, &mut blockers_layers, None);
    }
    // 241-244
    let support_threshold: i32 = cfg_int(print_object.config(), "support_threshold_angle");
    let support_threshold_auto: bool = support_threshold == 0;
    // +1 makes the threshold inclusive
    let tan_threshold: f64 = if support_threshold_auto { 0. } else { (std::f64::consts::PI * f64::from(support_threshold + 1) / 180.).tan() };
    //FIXME this is a fudge constant!
    // Same as the classic path: dilate by the effective tip, not the raw setting. settings.min_radius is
    // half the already clamped tree_support_tip_diameter and is in scaled units.
    // 248
    let enforcer_overhang_offset: f64 = (settings.min_radius * 2) as f64;
    // 249 `g_config_tree_support_collision_resolution` (libslic3r.h:92) is the float 0.2
    let radius_sample_resolution: CoordF = f64::from(0.2f32);

    // calc the extrudable expolygons of each layer
    // 252-257
    let extrusion_width: CoordF = {
        let config = print_object.config();
        let full_print_config = print.full_print_config();
        crate::flow::Flow::extrusion_width_for_nozzle(
            &crate::config::ConfigOptionFloat::with_value(cfg_float(config, "line_width")),
            primary_nozzle_diameter(full_print_config) as f32,
            support_nozzle_diameter(print, cfg_int(config, "support_filament") as u32),
            multi_nozzle_diameter_mode_enabled(full_print_config),
        )
    };
    let extrusion_width_scaled: CoordF = extrusion_width / SCALING_FACTOR;
    // 258-267
    for layer_nr in 0..print_object.layer_count() {
        if canceled() {
            break;
        }
        let layer = &mut print_object.m_layers[layer_nr];
        // Filter out areas whose diameter that is smaller than extrusion_width, but we don't want to lose any details.
        layer.lslices_extrudable = cu::intersection_ex(
            &expolygons_provider(&layer.lslices),
            &expolygons_provider(&cu::offset2_ex(
                &layer.lslices,
                (-extrusion_width_scaled / 2.) as f32,
                extrusion_width_scaled as f32,
                cu::DEFAULT_JOIN_TYPE,
                cu::DEFAULT_MITER_LIMIT,
            )),
            cu::ApplySafetyOffset::No,
        );
    }

    // 269 `size_t(support_enforce_layers)` wraps a negative value, as the cast does.
    let num_overhang_layers: usize = if support_auto {
        num_object_layers
    } else {
        std::cmp::min(num_object_layers, std::cmp::max(support_enforce_layers as usize, enforcers_layers.len()))
    };
    // 270-342
    for layer_id in 1..num_overhang_layers as LayerIndex {
        let current_layer = print_object.get_layer(layer_id);
        let lower_layer = print_object.get_layer(layer_id - 1);
        // Full overhangs with zero lower_layer_offset and no blockers applied.
        let mut raw_overhangs = Polygons::new();
        let mut raw_overhangs_calculated = false;
        // Final overhangs.
        let mut overhangs = Polygons::new();
        // For how many layers full overhangs shall be supported.
        let enforced_layer: bool = layer_id < support_enforce_layers;
        if support_auto || enforced_layer {
            let lower_layer_offset: f32;
            if enforced_layer {
                lower_layer_offset = 0.;
            } else if support_threshold_auto {
                // 287-291 the sum is a float; `LayerRegion::flow(role)` is `flow(role, m_layer->height)`
                let mut external_perimeter_width: f32 = 0.;
                for layerm in lower_layer.regions() {
                    external_perimeter_width += layerm.flow(FlowRole::ExternalPerimeter, lower_layer.height)?.scaled_width() as f32;
                }
                external_perimeter_width /= lower_layer.region_count() as f32;
                lower_layer_offset = (0.5 * f64::from(external_perimeter_width)) as f32;
            } else {
                // 293 `scaled<float>(v)` is `float(v / SCALING_FACTOR)` (Point.hpp)
                lower_layer_offset = ((lower_layer.height / tan_threshold) / SCALING_FACTOR) as f32;
            }
            let lower_layer_offseted: Polygons =
                cu::offset_expolygons(&lower_layer.lslices_extrudable, lower_layer_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
            overhangs = cu::diff(&expolygons_provider(&current_layer.lslices_extrudable), &lower_layer_offseted, cu::ApplySafetyOffset::No);
            if lower_layer_offset == 0. {
                raw_overhangs = overhangs.clone();
                raw_overhangs_calculated = true;
            }
            if !(enforced_layer || blockers_layers.is_empty() || blockers_layers[layer_id as usize].is_empty()) {
                overhangs = cu::diff_pe(
                    &overhangs,
                    &cu::offset_ex(
                        &cu::union_(&blockers_layers[layer_id as usize]),
                        (radius_sample_resolution / SCALING_FACTOR) as f32,
                        cu::DEFAULT_JOIN_TYPE,
                        cu::DEFAULT_MITER_LIMIT,
                    ),
                    cu::ApplySafetyOffset::Yes,
                );
            }
            //if (config.bridge_no_support) {
            //    for (const LayerRegion *layerm : current_layer.regions())
            //        remove_bridges_from_contacts(print_config, lower_layer, *layerm,
            //            float(layerm->flow(frExternalPerimeter).scaled_width()), overhangs);
            //}
        }
        //check_self_intersections(overhangs, "generate_overhangs1");
        if !enforcers_layers.is_empty() && !enforcers_layers[layer_id as usize].is_empty() {
            // Has some support enforcers at this layer, apply them to the overhangs, don't apply the support threshold angle.
            let mut enforced_overhangs: Polygons = cu::intersection(
                &if raw_overhangs_calculated {
                    raw_overhangs
                } else {
                    cu::diff(
                        &expolygons_provider(&current_layer.lslices),
                        &expolygons_provider(&lower_layer.lslices),
                        cu::ApplySafetyOffset::No,
                    )
                },
                &enforcers_layers[layer_id as usize], /*, ApplySafetyOffset::Yes */
                cu::ApplySafetyOffset::No,
            );
            if !enforced_overhangs.is_empty() {
                //FIXME this is a hack to make enforcers work on steep overhangs.
                //FIXME enforcer_overhang_offset is a fudge constant!
                enforced_overhangs = cu::diff_pe(
                    &cu::offset_expolygons(
                        &cu::union_ex(&enforced_overhangs),
                        enforcer_overhang_offset as f32,
                        cu::DEFAULT_JOIN_TYPE,
                        cu::DEFAULT_MITER_LIMIT,
                    ),
                    &lower_layer.lslices,
                    cu::ApplySafetyOffset::No,
                );
                //check_self_intersections(enforced_overhangs, "generate_overhangs - enforced overhangs2");
                overhangs = if overhangs.is_empty() { enforced_overhangs } else { cu::union_2(&overhangs, &enforced_overhangs) };
                //check_self_intersections(overhangs, "generate_overhangs - enforcers");
            }
        }
        out[layer_id as usize + num_raft_layers] = overhangs;
        throw_on_cancel();
    }

    Ok(out)
}

/// TreeSupport3D.cpp:349-370 `static LayerIndex precalculate(const Print &print, const std::vector<Polygons>
/// &overhangs, const TreeSupportSettings &config, const std::vector<size_t> &object_ids, TreeModelVolumes
/// &volumes, std::function<void()> throw_on_cancel)`
///
/// Precalculates all avoidances, that could be required.
#[must_use]
pub fn precalculate(
    print: &pf::Print,
    overhangs: &[Polygons],
    config: &TreeSupportSettings,
    object_ids: &[usize],
    volumes: &mut TreeModelVolumes,
    throw_on_cancel: ThrowOnCancel,
) -> LayerIndex {
    precalculate_with(overhangs, config, object_ids, volumes, throw_on_cancel, &|object_id| print.get_object(object_id))
}

/// `precalculate` (TreeSupport3D.cpp:349-370) with `print.get_object(id)` handed in as `get_object`, for a caller
/// that holds the object apart from the print (`generate_support_areas`).
#[must_use]
pub fn precalculate_with<'a>(
    overhangs: &[Polygons],
    config: &TreeSupportSettings,
    object_ids: &[usize],
    volumes: &mut TreeModelVolumes,
    throw_on_cancel: ThrowOnCancel,
    get_object: &dyn Fn(usize) -> &'a pf::PrintObject,
) -> LayerIndex {
    // calculate top most layer that is relevant for support
    let mut max_layer: LayerIndex = 0;
    for &object_id in object_ids {
        let print_object = get_object(object_id);
        let num_raft_layers = config.raft_layers.len() as i32;
        let num_layers = print_object.layer_count() as i32 + num_raft_layers;
        let mut max_support_layer_id: i32 = 0;
        for layer_id in std::cmp::max(num_raft_layers, 1)..num_layers {
            if !overhangs[layer_id as usize].is_empty() {
                max_support_layer_id = layer_id;
            }
        }
        max_layer = std::cmp::max(max_support_layer_id - config.z_distance_top_layers as i32, 0);
    }
    if max_layer > 0 {
        // The actual precalculation happens in TreeModelVolumes.
        volumes.precalculate(get_object(object_ids[0]), max_layer, throw_on_cancel);
    }
    max_layer
}

/// TreeSupport3D.cpp:372-395 `LineStatus get_avoidance_status(const Point& p, coord_t radius, LayerIndex layer_idx,
/// const TreeModelVolumes& volumes, const TreeSupportSettings& config, coord_t xy_min_distance = -1)`
// picked from convert_lines_to_internal()
#[must_use]
pub fn get_avoidance_status(
    p: &Point,
    mut radius: Coord,
    layer_idx: LayerIndex,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    mut xy_min_distance: Coord,
) -> LineStatus {
    if xy_min_distance < 0 {
        xy_min_distance = config.xy_min_distance;
    }
    let min_xy_dist = config.xy_distance > xy_min_distance;
    radius += if min_xy_dist { std::cmp::max(0, xy_min_distance - config.xy_min_distance) } else { 0 };

    let mut type_ = LineStatus::INVALID;

    if !contains(&volumes.get_avoidance(radius, layer_idx, AvoidanceType::FastSafe, false, min_xy_dist), p) {
        type_ = LineStatus::TO_BP_SAFE;
    } else if !contains(&volumes.get_avoidance(radius, layer_idx, AvoidanceType::Fast, false, min_xy_dist), p) {
        type_ = LineStatus::TO_BP;
    } else if config.support_rests_on_model
        && !contains(&volumes.get_avoidance(radius, layer_idx, AvoidanceType::FastSafe, true, min_xy_dist), p)
    {
        type_ = LineStatus::TO_MODEL_GRACIOUS_SAFE;
    } else if config.support_rests_on_model
        && !contains(&volumes.get_avoidance(radius, layer_idx, AvoidanceType::Fast, true, min_xy_dist), p)
    {
        type_ = LineStatus::TO_MODEL_GRACIOUS;
    } else if config.support_rests_on_model && !contains(&volumes.get_collision(radius, layer_idx, min_xy_dist), p) {
        type_ = LineStatus::TO_MODEL;
    }

    type_
}

/// TreeSupport3D.cpp:397-443 `static LineInformations convert_lines_to_internal(const TreeModelVolumes &volumes,
/// const TreeSupportSettings &config, const Polylines &polylines, LayerIndex layer_idx, coord_t xy_min_distance = -1)`
///
/// Converts a Polygons object representing a line into the internal format.
// Called by generate_initial_areas()
#[must_use]
pub fn convert_lines_to_internal(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    polylines: &[Polyline],
    layer_idx: LayerIndex,
    mut xy_min_distance: Coord,
) -> LineInformations {
    if xy_min_distance < 0 {
        xy_min_distance = config.xy_min_distance;
    }
    let min_xy_dist = config.xy_distance > xy_min_distance;
    let collision_radius: Coord =
        config.get_radius(0, 0.) + if min_xy_dist { std::cmp::max(0, xy_min_distance - config.xy_min_distance) } else { 0 };

    let mut result: LineInformations = Vec::new();
    // Also checks if the position is valid, if it is NOT, it deletes that point
    for line in polylines {
        let mut res_line: LineInformation = Vec::new();
        for &p in &line.points {
            if !contains(&volumes.get_avoidance(collision_radius, layer_idx, AvoidanceType::FastSafe, false, min_xy_dist), &p) {
                res_line.push((p, LineStatus::TO_BP_SAFE));
            } else if !contains(&volumes.get_avoidance(collision_radius, layer_idx, AvoidanceType::Fast, false, min_xy_dist), &p) {
                res_line.push((p, LineStatus::TO_BP));
            } else if config.support_rests_on_model
                && !contains(&volumes.get_avoidance(collision_radius, layer_idx, AvoidanceType::FastSafe, true, min_xy_dist), &p)
            {
                res_line.push((p, LineStatus::TO_MODEL_GRACIOUS_SAFE));
            } else if config.support_rests_on_model
                && !contains(&volumes.get_avoidance(collision_radius, layer_idx, AvoidanceType::Fast, true, min_xy_dist), &p)
            {
                res_line.push((p, LineStatus::TO_MODEL_GRACIOUS));
            } else if config.support_rests_on_model && !contains(&volumes.get_collision(collision_radius, layer_idx, min_xy_dist), &p) {
                res_line.push((p, LineStatus::TO_MODEL));
            } else if !res_line.is_empty() {
                result.push(res_line.clone());
                res_line.clear();
            }
        }
        if !res_line.is_empty() {
            result.push(res_line.clone());
            res_line.clear();
        }
    }

    validate_range(result.as_slice());
    result
}

/// TreeSupport3D.cpp:446-463 `static Polylines convert_internal_to_lines(LineInformations lines)` (the C++ has it
/// under `#if 0`).
///
/// Converts lines in internal format into a Polygons object representing these lines.
#[must_use]
pub fn convert_internal_to_lines(lines: &[LineInformation]) -> Polylines {
    let mut result: Polylines = Vec::new();
    for line in lines {
        let mut path = Polyline::new();
        for point_data in line {
            path.points.push(point_data.0);
        }
        result.push(path);
    }
    validate_range(result.as_slice());
    result
}

/// TreeSupport3D.cpp:466-491 `static bool evaluate_point_for_next_layer_function(const TreeModelVolumes &volumes,
/// const TreeSupportSettings &config, size_t current_layer, const std::pair<Point, LineStatus> &p, coord_t
/// xy_min_distance = -1)`
///
/// Evaluates if a point has to be added now. Required for a split_lines call in generate_initial_areas().
#[must_use]
pub fn evaluate_point_for_next_layer_function(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    current_layer: usize,
    p: &(Point, LineStatus),
    mut xy_min_distance: Coord,
) -> bool {
    if xy_min_distance < 0 {
        xy_min_distance = config.xy_min_distance;
    }
    let min_xy_dist = config.xy_distance > xy_min_distance;
    let collision_radius: Coord =
        config.get_radius(0, 0.) + if min_xy_dist { std::cmp::max(0, xy_min_distance - config.xy_min_distance) } else { 0 };
    // `current_layer - 1` is a `size_t` handed to a `LayerIndex`.
    let layer_below = (current_layer.wrapping_sub(1)) as LayerIndex;
    if !contains(
        &volumes.get_avoidance(
            collision_radius,
            layer_below,
            if p.1 == LineStatus::TO_BP_SAFE { AvoidanceType::FastSafe } else { AvoidanceType::Fast },
            false,
            min_xy_dist,
        ),
        &p.0,
    ) {
        return true;
    }
    if config.support_rests_on_model && (p.1 != LineStatus::TO_BP && p.1 != LineStatus::TO_BP_SAFE) {
        return !contains(
            &if p.1 == LineStatus::TO_MODEL_GRACIOUS || p.1 == LineStatus::TO_MODEL_GRACIOUS_SAFE {
                volumes.get_avoidance(
                    collision_radius,
                    layer_below,
                    if p.1 == LineStatus::TO_MODEL_GRACIOUS_SAFE { AvoidanceType::FastSafe } else { AvoidanceType::Fast },
                    true,
                    min_xy_dist,
                )
            } else {
                volumes.get_collision(collision_radius, layer_below, min_xy_dist)
            },
            &p.0,
        );
    }
    false
}

/// TreeSupport3D.cpp:493-524 `template<typename EvaluatePointFn> static std::pair<LineInformations,
/// LineInformations> split_lines(const LineInformations &lines, EvaluatePointFn evaluatePoint)`
///
/// Evaluates which points of some lines are not valid one layer below and which are. Assumes all points are valid
/// on the current layer. Validity is evaluated using supplied lambda. Returns a pair with which points are still
/// valid in the first slot and which are not in the second slot.
#[must_use]
pub fn split_lines<F>(lines: &[LineInformation], mut evaluate_point: F) -> (LineInformations, LineInformations)
where
    F: FnMut(&(Point, LineStatus)) -> bool,
{
    // assumes all Points on the current line are valid

    let mut keep: LineInformations = Vec::new();
    let mut set_free: LineInformations = Vec::new();
    for line in lines {
        let mut current_keep = true;
        let mut resulting_line: LineInformation = Vec::new();
        for me in line {
            if evaluate_point(me) != current_keep {
                if !resulting_line.is_empty() {
                    (if current_keep { &mut keep } else { &mut set_free }).push(std::mem::take(&mut resulting_line));
                }
                current_keep = !current_keep;
            }
            resulting_line.push(*me);
        }
        if !resulting_line.is_empty() {
            (if current_keep { &mut keep } else { &mut set_free }).push(resulting_line);
        }
    }
    validate_range(keep.as_slice());
    validate_range(set_free.as_slice());
    (keep, set_free)
}

/// TreeSupport3D.cpp:526-573 `static std::optional<std::pair<Point, size_t>>
/// polyline_sample_next_point_at_distance(const Points &polyline, const Point &start_pt, size_t start_idx, double dist)`
// Ported from CURA's PolygonUtils::getNextPointWithDistance()
// Sample a next point at distance "dist" from start_pt on polyline segment (start_idx, start_idx + 1).
// Returns sample point and start index of its segment on polyline if such sample exists.
pub fn polyline_sample_next_point_at_distance(polyline: &[Point], start_pt: &Point, start_idx: usize, dist: f64) -> Option<(Point, usize)> {
    let dist2 = dist * dist;
    let dist2i = dist2 as i64;
    let eps = scaled_f64(0.01);

    for i in (start_idx + 1)..polyline.len() {
        let p1 = polyline[i];
        if squared_norm_i64(p1 - *start_pt) >= dist2i {
            // The end point is outside the circle with center "start_pt" and radius "dist".
            let p0 = polyline[i - 1];
            let v: Vec2d = vec2d(p1 - p0);
            let l2v = squared_norm2(v);
            if l2v < eps * eps {
                // Very short segment.
                let c = Point::new((p0.x + p1.x) / 2, (p0.y + p1.y) / 2);
                if (norm2(vec2d(*start_pt - c)) - dist).abs() < eps {
                    return Some((c, i - 1));
                } else {
                    continue;
                }
            }
            let p0f: Vec2d = vec2d(*start_pt - p0);
            // Foot point of start_pt into v.
            let s = dot2(p0f, v) / l2v;
            let foot_pt: Vec2d = (v.0 * s, v.1 * s);
            // Vector from foot point of "start_pt" to "start_pt".
            let xf: Vec2d = (p0f.0 - foot_pt.0, p0f.1 - foot_pt.1);
            // Squared distance of "start_pt" from the ray (p0, p1).
            let l2_from_line = squared_norm2(xf);
            // Squared distance of an intersection point of a circle with center at the foot point.
            let mut l2_intersection = dist2 - l2_from_line;
            if l2_intersection > -SCALED_EPSILON {
                // The ray (p0, p1) touches or intersects a circle centered at "start_pt" with radius "dist".
                // Distance of the circle intersection point from the foot point.
                l2_intersection = if l2_intersection < 0. { 0. } else { l2_intersection };
                if squared_norm2((v.0 - foot_pt.0, v.1 - foot_pt.1)) >= l2_intersection {
                    // Intersection of the circle with the segment (p0, p1) is on the right side (close to p1) from the foot point.
                    let k = (l2_intersection / l2v).sqrt();
                    let p = p0 + Point::new((foot_pt.0 + v.0 * k) as Coord, (foot_pt.1 + v.1 * k) as Coord);
                    validate_range(&p);
                    return Some((p, i - 1));
                }
            }
        }
    }
    None
}

// MultiPoint.hpp:134-142 `inline double length(const Points &pts)`
fn length(pts: &[Point]) -> f64 {
    let mut total = 0.;
    for i in 1..pts.len() {
        total += norm2(vec2d(pts[i] - pts[i - 1]));
    }
    total
}

/// TreeSupport3D.cpp:575-675 `static Polylines ensure_maximum_distance_polyline(const Polylines &input, double
/// distance, size_t min_points)`
///
/// Eensures that every line segment is about distance in length. The resulting lines may differ from the original
/// but all points are on the original
#[must_use]
pub fn ensure_maximum_distance_polyline(input: &[Polyline], distance: f64, min_points: usize) -> Polylines {
    let mut result: Polylines = Vec::new();
    for part in input {
        let mut part: Polyline = part.clone();
        if part.points.is_empty() {
            continue;
        }

        let len = length(&part.points);
        let mut line = Polyline::new();
        // std::max(distance, scaled<double>(0.1))
        let mut current_distance = if distance < scaled_f64(0.1) { scaled_f64(0.1) } else { distance };
        if len < 2. * distance && min_points <= 1 {
            // Insert the opposite point of the first one.
            //FIXME pretty expensive
            let mut pl = part.clone();
            pl.clip_end(len / 2.);
            line.points.push(*pl.points.last().unwrap());
        } else {
            let mut optimal_end_index = part.points.len() - 1;

            if part.points[0] == part.points[part.points.len() - 1] {
                let mut optimal_start_index: usize = 0;
                // If the polyline was a polygon, there is a high chance it was an overhang. Overhangs that are <60� tend to be very thin areas, so lets get the beginning and end of them and ensure that they are supported.
                // The first point of the line will always be supported, so rotate the order of points in this polyline that one of the two corresponding points that are furthest from each other is in the beginning.
                // The other will be manually added (optimal_end_index)
                // `coord_t max_dist2_between_vertecies`: the squared norms are doubles and `coord_t` is the pin's
                // int32_t (libslic3r.h:40), so the stored one is converted to 32 bits. A squared distance of
                // 2^31 or more (vertices further apart than 0.46 mm) does not fit; the engine's arm64 build
                // converts with `fcvtzs`, which saturates at INT32_MAX, as `as i32` does. From then on every
                // pair further apart than that replaces the indices: the last such pair wins, not the furthest.
                let mut max_dist2_between_vertecies: i32 = 0;
                for idx in 0..(part.points.len() - 1) {
                    for inner_idx in 0..(part.points.len() - 1) {
                        if squared_norm2(vec2d(part.points[idx] - part.points[inner_idx])) > max_dist2_between_vertecies as f64 {
                            optimal_start_index = idx;
                            optimal_end_index = inner_idx;
                            max_dist2_between_vertecies = squared_norm2(vec2d(part.points[idx] - part.points[inner_idx])) as i32;
                        }
                    }
                }
                // std::rotate(part.begin(), part.begin() + optimal_start_index, part.end() - 1);
                let n = part.points.len();
                part.points[..n - 1].rotate_left(optimal_start_index);
                part.points[n - 1] = part.points[0]; // restore that property that this polyline ends where it started.
                optimal_end_index = (part.points.len() + optimal_end_index - optimal_start_index - 1) % (part.points.len() - 1);
            }

            while line.points.len() < min_points && current_distance >= scaled_f64(0.1) {
                line.points.clear();
                let mut current_point = part.points[0];
                line.points.push(part.points[0]);
                if min_points > 1 || norm2(vec2d(part.points[0] - part.points[optimal_end_index])) > current_distance {
                    line.points.push(part.points[optimal_end_index]);
                }
                let mut current_index: usize = 0;
                let mut next_distance = current_distance;
                // Get points so that at least min_points are added and they each are current_distance away from each other. If that is impossible, decrease current_distance a bit.
                // The input are lines, that means that the line from the last to the first vertex does not have to exist, so exclude all points that are on this line!
                while let Some(next_point) = polyline_sample_next_point_at_distance(&part.points, &current_point, current_index, next_distance) {
                    // Not every point that is distance away, is valid, as it may be much closer to another point. This is especially the case when the overhang is very thin.
                    // So this ensures that the points are actually a certain distance from each other.
                    // This assurance is only made on a per polygon basis, as different but close polygon may not be able to use support below the other polygon.
                    let mut min_distance_to_existing_point = f64::MAX;
                    for p in &line.points {
                        let d = norm2(vec2d(*p - next_point.0));
                        // std::min(a, b): `(b < a) ? b : a`
                        if d < min_distance_to_existing_point {
                            min_distance_to_existing_point = d;
                        }
                    }
                    if min_distance_to_existing_point >= current_distance {
                        // viable point was found. Add to possible result.
                        line.points.push(next_point.0);
                        current_point = next_point.0;
                        current_index = next_point.1;
                        next_distance = current_distance;
                    } else {
                        if current_point == next_point.0 {
                            // In case a fixpoint is encountered, better aggressively overcompensate so the code does not become stuck here...
                            tree_supports_show_error("Encountered issue while placing tips. Some tips may be missing.", true);
                            if next_distance > 2. * current_distance {
                                // This case should never happen, but better safe than sorry.
                                break;
                            }
                            next_distance += current_distance;
                            continue;
                        }
                        // if the point was too close, the next possible viable point is at least distance-min_distance_to_existing_point away from the one that was just checked.
                        let d = current_distance - min_distance_to_existing_point;
                        next_distance = if d < scaled_f64(0.1) { scaled_f64(0.1) } else { d };
                        current_point = next_point.0;
                        current_index = next_point.1;
                    }
                }
                current_distance *= 0.9;
            }
        }
        result.push(line);
    }
    validate_range(result.as_slice());
    result
}

/// TreeSupport3D.cpp:677-767 `static Polylines generate_support_infill_lines(const Polygons &polygon, const
/// SupportParameters &support_params, bool roof, LayerIndex layer_idx, coord_t support_infill_distance)`
///
/// Returns Polylines representing the (infill) lines that will result in slicing the given area. `fillers` is
/// `Fill::new_from_type`.
#[must_use]
pub fn generate_support_infill_lines(
    polygon: &[Polygon],
    support_params: &SupportParameters,
    roof: bool,
    layer_idx: LayerIndex,
    support_infill_distance: Coord,
    fillers: &dyn SupportFillFactory,
) -> Polylines {
    let flow = if roof { &support_params.support_material_interface_flow } else { &support_params.support_material_flow };
    let mut filler = fillers.new_from_type(if roof { support_params.interface_fill_pattern } else { support_params.base_fill_pattern });
    let mut fill_params = crate::fill::FillParams::default();

    filler.fill_mut().layer_id = layer_idx as usize;
    filler.fill_mut().spacing = flow.spacing();
    filler.fill_mut().angle = if roof {
        //fixme support_layer.interface_id() instead of layer_idx
        // `(support_params.interface_angle + (layer_idx & 1) ? float(- M_PI / 4.) : float(+ M_PI / 4.))`: the sum is
        // the condition.
        if support_params.interface_angle + (layer_idx & 1) as f32 != 0. {
            (-std::f64::consts::PI / 4.) as f32
        } else {
            (std::f64::consts::PI / 4.) as f32
        }
    } else {
        support_params.base_angle
    };

    // `float(roof ? support_params.interface_density : scaled<float>(filler->spacing) / (scaled<float>(filler->spacing)
    // + float(support_infill_distance)))`
    fill_params.density = (if roof {
        support_params.interface_density
    } else {
        let scaled_spacing = (filler.fill().spacing / SCALING_FACTOR) as f32;
        (scaled_spacing / (scaled_spacing + support_infill_distance as f32)) as f64
    }) as f32 as f64;
    fill_params.dont_adjust = true;

    let mut out: Polylines = Vec::new();
    for expoly in cu::union_ex(polygon) {
        // The surface type does not matter.
        check_self_intersections(&expoly, "generate_support_infill_lines");
        let surface = crate::surface::Surface::new(crate::surface::SurfaceType::Internal, expoly);
        match filler.fill_surface(&surface, &fill_params) {
            Ok(pl) => out.extend(pl),
            Err(InfillFailedException) => {}
        }
    }
    validate_range(out.as_slice());
    out
}

/// TreeSupport3D.cpp:1429-1530 `static unsigned int move_inside(const Polygons &polygons, Point &from, int distance
/// = 0, int64_t maxDist2 = std::numeric_limits<int64_t>::max())`
pub fn move_inside(polygons: &[Polygon], from: &mut Point, distance: i32, max_dist2: i64) -> u32 {
    let mut ret = *from;
    let mut best_dist2 = f64::MAX;
    let mut best_poly: u32 = u32::MAX;
    let mut is_already_on_correct_side_of_boundary = false; // whether [from] is already on the right side of the boundary
    for poly_idx in 0..polygons.len() as u32 {
        let poly = &polygons[poly_idx as usize];
        if poly.points.len() < 2 {
            continue;
        }
        let mut p0 = poly.points[poly.points.len() - 2];
        let mut p1 = *poly.points.last().unwrap();
        // because we compare with vSize2 here (no division by zero), we also need to compare by vSize2 inside the loop
        // to avoid integer rounding edge cases
        let mut projected_p_beyond_prev_segment = dot_i64(p1 - p0, *from - p0) >= squared_norm_i64(p1 - p0);
        for &p2 in &poly.points {
            // X = A + Normal(B-A) * (((B-A) dot (P-A)) / VSize(B-A));
            //   = A +       (B-A) *  ((B-A) dot (P-A)) / VSize2(B-A);
            // X = P projected on AB
            let a = p1;
            let b = p2;
            let p = *from;
            let ab = b - a;
            let ap = p - a;
            let ab_length2: i64 = squared_norm_i64(ab);
            if ab_length2 <= 0 {
                //A = B, i.e. the input polygon had two adjacent points on top of each other.
                p1 = p2; //Skip only one of the points.
                continue;
            }
            let dot_prod: i64 = dot_i64(ab, ap);
            if dot_prod <= 0 {
                // x is projected to before ab
                if projected_p_beyond_prev_segment {
                    //  case which looks like:   > .
                    projected_p_beyond_prev_segment = false;
                    let x = p1;

                    let dist2 = squared_norm_i64(x - p);
                    if (dist2 as f64) < best_dist2 {
                        best_dist2 = dist2 as f64;
                        best_poly = poly_idx;
                        if distance == 0 {
                            ret = x;
                        } else {
                            let abd: Vec2d = vec2d(ab);
                            let p1p2: Vec2d = vec2d(p1 - p0);
                            let lab = norm2(abd);
                            let lp1p2 = norm2(p1p2);
                            // inward direction irrespective of sign of [distance]
                            let sa = scaled_f64(10.0) / lab;
                            let sb = scaled_f64(10.0) / lp1p2;
                            let sum: Vec2d = (abd.0 * sa + p1p2.0 * sb, abd.1 * sa + p1p2.1 * sb);
                            // perp(v): (-v.y, v.x)
                            let inward_dir: Vec2d = (-sum.1, sum.0);
                            // MM2INT(10.0) to retain precision for the eventual normalization
                            let k = distance as f64 / norm2(inward_dir);
                            ret = x + Point::new((inward_dir.0 * k) as Coord, (inward_dir.1 * k) as Coord);
                            is_already_on_correct_side_of_boundary = dot2(inward_dir, vec2d(p - x)) * distance as f64 >= 0.;
                        }
                    }
                } else {
                    projected_p_beyond_prev_segment = false;
                    p0 = p1;
                    p1 = p2;
                    continue;
                }
            } else if dot_prod >= ab_length2 {
                // x is projected to beyond ab
                projected_p_beyond_prev_segment = true;
                p0 = p1;
                p1 = p2;
                continue;
            } else {
                // x is projected to a point properly on the line segment (not onto a vertex). The case which looks like | .
                projected_p_beyond_prev_segment = false;
                let t = dot_prod as f64 / ab_length2 as f64;
                let x = a + Point::new((ab.x as f64 * t) as Coord, (ab.y as f64 * t) as Coord);
                let dist2 = squared_norm_i64(p - x);
                if (dist2 as f64) < best_dist2 {
                    best_dist2 = dist2 as f64;
                    best_poly = poly_idx;
                    if distance == 0 {
                        ret = x;
                    } else {
                        let abd: Vec2d = vec2d(ab);
                        let k = distance as f64 / norm2(abd);
                        // inward or outward depending on the sign of [distance]
                        let inward_dir: Vec2d = (-(abd.1 * k), abd.0 * k);
                        ret = x + Point::new(inward_dir.0 as Coord, inward_dir.1 as Coord);
                        is_already_on_correct_side_of_boundary = dot2(inward_dir, vec2d(p - x)) >= 0.;
                    }
                }
            }
            p0 = p1;
            p1 = p2;
        }
    }
    // when the best point is already inside and we're moving inside, or when the best point is already outside and we're moving outside
    if is_already_on_correct_side_of_boundary {
        // `distance * distance` is a product of two ints.
        if best_dist2 < distance.wrapping_mul(distance) as f64 {
            *from = ret;
        } else {
            // from = from; // original point stays unaltered. It is already inside by enough distance
        }
        return best_poly;
    } else if best_dist2 < max_dist2 as f64 {
        *from = ret;
        return best_poly;
    }
    u32::MAX
}

/// TreeSupport3D.cpp:1532-1537 `static Point move_inside_if_outside(const Polygons &polygons, Point from, int
/// distance = 0, int64_t maxDist2 = std::numeric_limits<int64_t>::max())`: the last two arguments are not handed
/// on to `move_inside`.
pub fn move_inside_if_outside(polygons: &[Polygon], mut from: Point, _distance: i32, _max_dist2: i64) -> Point {
    if !contains(polygons, &from) {
        move_inside(polygons, &mut from, 0, i64::MAX);
    }
    from
}

// -----------------------------------------------------------------------------
// TreeSupport3D.cpp:769-1427: the tips of the trees
// -----------------------------------------------------------------------------

use crate::support::support_common::safe_offset_inc;
use crate::support::support_layer::{SupportGeneratorLayerStorage, SupportGeneratorLayersPtr};
use crate::support::tree_support_common::{InterfacePlacer, SUPPORT_GENERATOR_LAYER_PTR_NULL, SUPPORT_TREE_CIRCLE_RESOLUTION};

/// The outputs an `InterfacePlacer` of the C++ holds references to (TreeSupportCommon.hpp:811-815): the
/// storage of the generator layers and the top contact, top interface and top base interface layers.
pub struct InterfacePlacerOutputs<'a> {
    pub layer_storage: &'a mut SupportGeneratorLayerStorage,
    pub top_contacts: &'a mut SupportGeneratorLayersPtr,
    pub top_interfaces: &'a mut SupportGeneratorLayersPtr,
    pub top_base_interfaces: &'a mut SupportGeneratorLayersPtr,
}

/// TreeSupport3D.cpp:769-931 `class RichInterfacePlacer : public InterfacePlacer`
pub struct RichInterfacePlacer<'a, 'b> {
    /// The base class.
    pub interface_placer: InterfacePlacer<'a>,
    /// The outputs of the base class.
    pub outputs: InterfacePlacerOutputs<'b>,
    pub volumes: &'a TreeModelVolumes,
    // Radius of the tree tip is large enough to be covered by an interface.
    pub force_tip_to_roof: bool,
    // Outputs
    move_bounds: &'b mut Vec<SupportElements>,
    // Temps
    m_base_circle: Polygon,
    // Mutexes, guards
    m_already_inserted: Vec<std::collections::HashSet<Point>>,
    /// `Fill::new_from_type`
    fillers: &'a dyn SupportFillFactory,
}

impl<'a, 'b> RichInterfacePlacer<'a, 'b> {
    /// TreeSupport3D.cpp:925 `static constexpr const auto m_base_radius = scaled<int>(0.01);`
    pub fn m_base_radius() -> i32 {
        (0.01 / SCALING_FACTOR) as i32
    }

    /// TreeSupport3D.cpp:771-782 `RichInterfacePlacer(const InterfacePlacer &interface_placer, const TreeModelVolumes
    /// &volumes, bool force_tip_to_roof, size_t num_support_layers, std::vector<SupportElements> &move_bounds)`
    pub fn new(
        interface_placer: InterfacePlacer<'a>,
        outputs: InterfacePlacerOutputs<'b>,
        volumes: &'a TreeModelVolumes,
        force_tip_to_roof: bool,
        num_support_layers: usize,
        move_bounds: &'b mut Vec<SupportElements>,
        fillers: &'a dyn SupportFillFactory,
    ) -> Self {
        Self {
            interface_placer,
            outputs,
            volumes,
            force_tip_to_roof,
            move_bounds,
            // `make_circle(m_base_radius, SUPPORT_TREE_CIRCLE_RESOLUTION)`: Polygon.cpp:729 `make_circle(double radius,
            // double error)`, the number of vertices of a circle taken as the error.
            m_base_circle: crate::geometry::make_circle(Self::m_base_radius() as f64, SUPPORT_TREE_CIRCLE_RESOLUTION as f64),
            m_already_inserted: vec![std::collections::HashSet::new(); num_support_layers],
            fillers,
        }
    }

    fn config(&self) -> &'a TreeSupportSettings {
        self.interface_placer.config
    }

    /// TreeSupportCommon.hpp:772-780 `add_roofs` of the base class.
    pub fn add_roofs(&mut self, new_roofs: Vec<Polygons>, insert_layer_idx: usize) {
        self.interface_placer.add_roofs(
            new_roofs,
            insert_layer_idx,
            self.outputs.layer_storage,
            self.outputs.top_contacts,
            self.outputs.top_interfaces,
            self.outputs.top_base_interfaces,
        );
    }

    /// TreeSupportCommon.hpp:782-786 `add_roof` of the base class.
    pub fn add_roof(&mut self, new_roof: Polygons, insert_layer_idx: usize, dtt_tip: usize) {
        self.interface_placer.add_roof(
            new_roof,
            insert_layer_idx,
            dtt_tip,
            self.outputs.layer_storage,
            self.outputs.top_contacts,
            self.outputs.top_interfaces,
            self.outputs.top_base_interfaces,
        );
    }

    /// TreeSupportCommon.hpp:789-793 `add_roof_build_plate` of the base class.
    pub fn add_roof_build_plate(&mut self, overhang_areas: Polygons, dtt_roof: usize) {
        self.interface_placer.add_roof_build_plate(
            overhang_areas,
            dtt_roof,
            self.outputs.layer_storage,
            self.outputs.top_contacts,
            self.outputs.top_interfaces,
            self.outputs.top_base_interfaces,
        );
    }

    /// TreeSupport3D.cpp:788-867 `void add_points_along_lines(LineInformations lines, LayerIndex insert_layer_idx,
    /// size_t roof_tip_layers, size_t supports_roof_layers, size_t dont_move_until, coord_t xy_min_distance)`
    // called by sample_overhang_area()
    pub fn add_points_along_lines(
        &mut self,
        // Insert points (tree tips or top contact interfaces) along these lines.
        mut lines: LineInformations,
        // Start at this layer.
        insert_layer_idx: LayerIndex,
        // Insert this number of interface layers.
        roof_tip_layers: usize,
        // True if an interface is already generated above these lines.
        supports_roof_layers: usize,
        // The element tries to not move until this dtt is reached.
        dont_move_until: usize,
        xy_min_distance: Coord,
    ) {
        validate_range(lines.as_slice());
        let config = self.config();
        let volumes = self.volumes;
        // Add tip area as roof (happens when minimum roof area > minimum tip area) if possible
        // `insert_layer_idx - dtt_roof_tip` is unsigned arithmetic (an int and a size_t).
        let insert_layer_idx_u = insert_layer_idx as usize;
        let mut dtt_roof_tip: usize = 0;
        while dtt_roof_tip < roof_tip_layers && insert_layer_idx_u.wrapping_sub(dtt_roof_tip) >= 1 {
            let this_layer_idx: usize = insert_layer_idx_u.wrapping_sub(dtt_roof_tip);
            //FIXME Vojtech: The circle is just shifted, it has a known size, the infill should fit all the time!
            let evaluate_roof_will_generate = |_p: &(Point, LineStatus)| true;

            {
                // keep all lines that are still valid on the next layer
                let split = split_lines(&lines, |p| evaluate_point_for_next_layer_function(volumes, config, this_layer_idx, p, xy_min_distance));
                let mut points: LineInformations = split.1;
                // Not all roofs are guaranteed to actually generate lines, so filter these out and add them as points.
                let split = split_lines(&split.0, evaluate_roof_will_generate);
                lines = split.0;
                points.extend(split.1);
                // add all points that would not be valid
                for line in &points {
                    for point_data in line {
                        self.add_point_as_influence_area(
                            *point_data,
                            this_layer_idx as LayerIndex,
                            // don't move until
                            roof_tip_layers - dtt_roof_tip,
                            // supports roof
                            dtt_roof_tip + supports_roof_layers > 0,
                            // disable ovalization
                            false,
                            xy_min_distance,
                        );
                    }
                }
            }

            // add all tips as roof to the roof storage
            let mut new_roofs: Polygons = Vec::new();
            for line in &lines {
                //FIXME sweep the tip radius along the line?
                for p in line {
                    let mut roof_circle = self.m_base_circle.clone();
                    // `roof_circle.scale(config.min_radius / m_base_radius)`: a quotient of integers; MultiPoint.cpp:6-10
                    // scales with Point.hpp:199 `operator*=(const double &)`, which truncates.
                    let factor = (config.min_radius / Self::m_base_radius() as Coord) as f64;
                    for pt in roof_circle.points.iter_mut() {
                        *pt = Point::new((pt.x as f64 * factor) as Coord, (pt.y as f64 * factor) as Coord);
                    }
                    roof_circle.translate(p.0);
                    new_roofs.push(roof_circle);
                }
            }
            self.add_roof(new_roofs, this_layer_idx, dtt_roof_tip + supports_roof_layers);
            dtt_roof_tip += 1;
        }

        for line in &lines {
            // If a line consists of enough tips, the assumption is that it is not a single tip, but part of a simulated support pattern.
            // Ovalisation should be disabled for these to improve the quality of the lines when tip_diameter=line_width
            let disable_ovalistation = config.min_radius < 3 * config.support_line_width && roof_tip_layers == 0 && dtt_roof_tip == 0 && line.len() > 5;
            for point_data in line {
                self.add_point_as_influence_area(
                    *point_data,
                    insert_layer_idx_u.wrapping_sub(dtt_roof_tip) as LayerIndex,
                    // don't move until
                    if dont_move_until > dtt_roof_tip { dont_move_until - dtt_roof_tip } else { 0 },
                    // supports roof
                    dtt_roof_tip + supports_roof_layers > 0,
                    disable_ovalistation,
                    xy_min_distance,
                );
            }
        }
    }

    /// TreeSupport3D.cpp:870-919 `void add_point_as_influence_area(std::pair<Point, LineStatus> p, LayerIndex
    /// insert_layer, size_t dont_move_until, bool roof, bool skip_ovalisation, coord_t xy_min_distance)`
    // called by this->add_points_along_lines()
    fn add_point_as_influence_area(
        &mut self,
        p: (Point, LineStatus),
        insert_layer: LayerIndex,
        dont_move_until: usize,
        roof: bool,
        skip_ovalisation: bool,
        xy_min_distance: Coord,
    ) {
        let config = self.config();
        let to_bp = p.1 == LineStatus::TO_BP || p.1 == LineStatus::TO_BP_SAFE;
        let gracious = to_bp || p.1 == LineStatus::TO_MODEL_GRACIOUS || p.1 == LineStatus::TO_MODEL_GRACIOUS_SAFE;
        let safe_radius = p.1 == LineStatus::TO_BP_SAFE || p.1 == LineStatus::TO_MODEL_GRACIOUS_SAFE;
        if !config.support_rests_on_model && !to_bp {
            tree_supports_show_error("Unable to add tip. Some overhang may not be supported correctly.", true);
            return;
        }
        let mut circle: Polygons = vec![self.m_base_circle.clone()];
        circle[0].translate(p.0);
        {
            // `p.first / ((config.min_radius + 1) / 10)`: integer divisions.
            let d = (config.min_radius + 1) / 10;
            let hash_pos = Point::new(p.0.x / d, p.0.y / d);
            if !self.m_already_inserted[insert_layer as usize].contains(&hash_pos) {
                // normalize the point a bit to also catch points which are so close that inserting it would achieve nothing
                self.m_already_inserted[insert_layer as usize].insert(hash_pos);
                const DTT: u32 = 0;
                let mut state = SupportElementState::default();
                state.target_height = insert_layer;
                state.target_position = p.0;
                state.next_position = p.0;
                state.layer_idx = insert_layer;
                state.effective_radius_height = DTT;
                state.to_buildplate = to_bp;
                state.distance_to_top = DTT;
                state.result_on_layer = p.0;
                state.increased_to_model_radius = 0;
                state.to_model_gracious = gracious;
                state.elephant_foot_increases = 0.;
                state.use_min_xy_dist = config.xy_distance > xy_min_distance;
                state.xy_min_distance = xy_min_distance;
                state.supports_roof = roof;
                state.dont_move_until = dont_move_until as u32;
                state.can_use_safe_radius = safe_radius;
                state.missing_roof_layers = if self.force_tip_to_roof { dont_move_until as u32 } else { 0 };
                state.skip_ovalisation = skip_ovalisation;
                self.move_bounds[insert_layer as usize].push(SupportElement::new(state, circle));
            }
        }
    }
}

/// TreeSupport3D.cpp:934-953 `int generate_raft_contact(const PrintObject &print_object, const TreeSupportSettings
/// &config, InterfacePlacer &interface_placer)`
pub fn generate_raft_contact(
    print_object: &pf::PrintObject,
    config: &TreeSupportSettings,
    interface_placer: &mut InterfacePlacer,
    outputs: &mut InterfacePlacerOutputs,
) -> i32 {
    let mut raft_contact_layer_idx: i32 = -1;
    if print_object.has_raft() && print_object.layer_count() > 0 {
        // Produce raft contact layer outside of the tree support loop, so that no trees will be generated for the raft contact layer.
        // Raft layers supporting raft contact interface will be produced by the classic raft generator.
        // Find the raft contact layer.
        raft_contact_layer_idx = config.raft_layers.len() as i32 - 1;
        while raft_contact_layer_idx > 0
            && config.raft_layers[raft_contact_layer_idx as usize] > print_object.slicing_parameters().raft_contact_top_z + EPSILON
        {
            raft_contact_layer_idx -= 1;
        }
        // Create the raft contact layer.
        let lslices = &print_object.get_layer(0).lslices;
        let expansion: f64 = cfg_float(print_object.config(), "raft_expansion");
        interface_placer.add_roof_unguarded(
            if expansion > 0. {
                // `offset(lslices, scaled<float>(expansion))`
                cu::offset_expolygons(lslices, (expansion / SCALING_FACTOR) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
            } else {
                crate::geometry::to_polygons(lslices)
            },
            raft_contact_layer_idx as usize,
            0,
            outputs.layer_storage,
            outputs.top_contacts,
            outputs.top_interfaces,
            outputs.top_base_interfaces,
        );
    }
    raft_contact_layer_idx
}

/// TreeSupport3D.cpp:955-1006 `void finalize_raft_contact(const PrintObject &print_object, const int
/// raft_contact_layer_idx, SupportGeneratorLayersPtr &top_contacts, std::vector<SupportElements> &move_bounds)`
pub fn finalize_raft_contact(
    print_object: &pf::PrintObject,
    raft_contact_layer_idx: i32,
    layer_storage: &SupportGeneratorLayerStorage,
    top_contacts: &mut SupportGeneratorLayersPtr,
    move_bounds: &mut Vec<SupportElements>,
) {
    if raft_contact_layer_idx >= 0 {
        let first_tree_layer: usize = print_object.slicing_parameters().raft_layers() - 1;
        // Remove tree tips that start below the raft contact,
        // remove interface layers below the raft contact.
        for i in 0..first_tree_layer {
            top_contacts[i] = SUPPORT_GENERATOR_LAYER_PTR_NULL;
            move_bounds[i].clear();
        }
        if raft_contact_layer_idx >= 0 && cfg_float(print_object.config(), "raft_expansion") > 0. {
            // If any tips at first_tree_layer now are completely inside the expanded raft layer, remove them as well before they are propagated to the ground.
            let raft_polygons = &layer_storage[top_contacts[raft_contact_layer_idx as usize]].polygons;
            // EdgeGrid::Grid grid(get_extents(raft_polygons).inflated(SCALED_EPSILON));
            // grid.create(raft_polygons, Polylines{}, coord_t(scale_(10.)));
            let mut bbox = crate::geometry::get_extents_polygons(raft_polygons);
            let eps = SCALED_EPSILON as Coord;
            bbox.min = Point::new(bbox.min.x - eps, bbox.min.y - eps);
            bbox.max = Point::new(bbox.max.x + eps, bbox.max.y + eps);
            let mut grid = crate::edge_grid::EdgeGrid::new();
            grid.set_bbox(bbox);
            grid.create_from_polygons(raft_polygons, (10. / SCALING_FACTOR) as Coord);
            let first_layer_move_bounds = &mut move_bounds[first_tree_layer];
            let threshold: f64 = (cfg_float(print_object.config(), "raft_expansion") / SCALING_FACTOR) * 2.;
            first_layer_move_bounds.retain(|el| {
                // `grid.signed_distance_edges(el.state.result_on_layer, threshold, dist)`: the search radius is a `coord_t`.
                if let Some((dist, _)) = grid.signed_distance_edges(&el.state.result_on_layer, threshold as Coord) {
                    // Support point is inside the expanded raft, remove it.
                    return !(dist < -0.);
                }
                true
            });
        }
    }
}

/// TreeSupport3D.cpp:1008-1158 `void sample_overhang_area(Polygons&& overhang_area, const bool large_horizontal_roof,
/// const size_t layer_idx, const size_t num_support_roof_layers, const coord_t connect_length, const coord_t
/// xy_min_distance, const TreeSupportMeshGroupSettings& mesh_group_settings, RichInterfacePlacer& interface_placer)`
// Called by generate_initial_areas(), used in parallel by multiple layers.
// Produce
// 1) Maximum num_support_roof_layers roof (top interface & contact) layers.
// 2) Tree tips supporting either the roof layers or the object itself.
// num_support_roof_layers should always be respected:
// If num_support_roof_layers contact layers could not be produced, then the tree tip
// is augmented with SupportElementState::missing_roof_layers
// and the top "missing_roof_layers" of such particular tree tips are supposed to be coverted to
// roofs aka interface layers by the tool path generator.
#[allow(clippy::too_many_arguments)]
pub fn sample_overhang_area(
    // Area to support
    mut overhang_area: Polygons,
    // If true, then the overhang_area is likely large and wide, thus it is worth to try
    // to cover it with continuous interfaces supported by zig-zag patterned tree tips.
    large_horizontal_roof: bool,
    // Index of the top suport layer generated by this function.
    layer_idx: usize,
    // Maximum number of roof (contact, interface) layers between the overhang and tree tips to be generated.
    num_support_roof_layers: usize,
    //
    connect_length: Coord,
    xy_min_distance: Coord,
    // Configuration classes
    mesh_group_settings: &TreeSupportMeshGroupSettings,
    // Configuration & Output
    interface_placer: &mut RichInterfacePlacer,
) {
    // Assumption is that roof will support roof further up to avoid a lot of unnecessary branches. Each layer down it is checked whether the roof area
    // is still large enough to be a roof and aborted as soon as it is not. This part was already reworked a few times, and there could be an argument
    // made to change it again if there are actual issues encountered regarding supporting roofs.
    // Main problem is that some patterns change each layer, so just calculating points and checking if they are still valid an layer below is not useful,
    // as the pattern may be different one layer below. Same with calculating which points are now no longer being generated as result from
    // a decreasing roof, as there is no guarantee that a line will be above these points. Implementing a separate roof support behavior
    // for each pattern harms maintainability as it very well could be >100 LOC
    let support_parameters = interface_placer.interface_placer.support_parameters;
    let fillers = interface_placer.fillers;
    let config = interface_placer.config();
    let volumes = interface_placer.volumes;
    let generate_roof_lines = |area: &[Polygon], layer_idx: LayerIndex| -> Polylines {
        generate_support_infill_lines(area, support_parameters, true, layer_idx, mesh_group_settings.support_roof_line_distance, fillers)
    };

    let mut overhang_lines: LineInformations = Vec::new();
    // Track how many top contact / interface layers were already generated.
    let mut dtt_roof: usize = 0;
    let mut layer_generation_dtt: usize = 0;

    if large_horizontal_roof {
        // Sometimes roofs could be empty as the pattern does not generate lines if the area is narrow enough (i am looking at you, concentric infill).
        // To catch these cases the added roofs are saved to be evaluated later.
        let mut added_roofs: Vec<Polygons> = vec![Vec::new(); num_support_roof_layers];
        let mut last_overhang: Polygons = overhang_area.clone();
        dtt_roof = 0;
        while dtt_roof < num_support_roof_layers && layer_idx.wrapping_sub(dtt_roof) >= 1 {
            // here the roof is handled. If roof can not be added the branches will try to not move instead
            let forbidden_next: Polygons;
            {
                let min_xy_dist = config.xy_distance > xy_min_distance;
                let collision_radius: Coord =
                    config.get_radius(0, 0.) + if min_xy_dist { std::cmp::max(0, xy_min_distance - config.xy_min_distance) } else { 0 };
                let layer_below = layer_idx.wrapping_sub(dtt_roof + 1) as LayerIndex;
                let forbidden_next_raw = if config.support_rests_on_model {
                    volumes.get_collision(collision_radius, layer_below, min_xy_dist)
                } else {
                    volumes.get_avoidance(collision_radius, layer_below, AvoidanceType::Fast, false, min_xy_dist)
                };
                // prevent rounding errors down the line
                //FIXME maybe use SafetyOffset::Yes at the following diff() instead?
                forbidden_next = cu::offset_expolygons(&cu::union_ex(&forbidden_next_raw), (0.005 / SCALING_FACTOR) as f32, crate::clipper::JoinType::jtMiter, 1.2);
            }
            let overhang_area_next: Polygons = cu::diff(&overhang_area, &forbidden_next, cu::ApplySafetyOffset::No);
            if crate::geometry::area_polygons(&overhang_area_next) < mesh_group_settings.minimum_roof_area {
                // Next layer down the roof area would be to small so we have to insert our roof support here.
                if dtt_roof > 0 {
                    let dtt_before = dtt_roof - 1;
                    // Produce support head points supporting an interface layer: First produce the interface lines, then sample them.
                    let layer = layer_idx - dtt_before;
                    overhang_lines = split_lines(
                        &convert_lines_to_internal(
                            volumes,
                            config,
                            &ensure_maximum_distance_polyline(&generate_roof_lines(&last_overhang, layer as LayerIndex), connect_length as f64, 1),
                            layer as LayerIndex,
                            xy_min_distance,
                        ),
                        |p| evaluate_point_for_next_layer_function(volumes, config, layer, p, xy_min_distance),
                    )
                    .0;
                }
                break;
            }
            added_roofs[dtt_roof] = overhang_area.clone();
            last_overhang = overhang_area;
            overhang_area = overhang_area_next;
            dtt_roof += 1;
        }

        layer_generation_dtt = std::cmp::max(dtt_roof, 1) - 1; // 1 inside max and -1 outside to avoid underflow. layer_generation_dtt=dtt_roof-1 if dtt_roof!=0;
        // if the roof should be valid, check that the area does generate lines. This is NOT guaranteed.
        if overhang_lines.is_empty() && dtt_roof != 0 && generate_roof_lines(&overhang_area, (layer_idx - layer_generation_dtt) as LayerIndex).is_empty() {
            for idx in 0..dtt_roof {
                // check for every roof area that it has resulting lines. Remember idx 1 means the 2. layer of roof => higher idx == lower layer
                if generate_roof_lines(&added_roofs[idx], (layer_idx - idx) as LayerIndex).is_empty() {
                    dtt_roof = idx;
                    layer_generation_dtt = std::cmp::max(dtt_roof, 1) - 1;
                    break;
                }
            }
        }
        added_roofs.truncate(dtt_roof);
        interface_placer.add_roofs(added_roofs, layer_idx);
    }

    if overhang_lines.is_empty() {
        // support_line_width to form a line here as otherwise most will be unsupported. Technically this violates branch distance, but not only is this the only reasonable choice,
        // but it ensures consistant behaviour as some infill patterns generate each line segment as its own polyline part causing a similar line forming behaviour.
        // This is not doen when a roof is above as the roof will support the model and the trees only need to support the roof
        let supports_roof = dtt_roof > 0;
        let continuous_tips = !supports_roof && large_horizontal_roof;
        let mut polylines: Polylines = ensure_maximum_distance_polyline(
            &generate_support_infill_lines(
                &overhang_area,
                support_parameters,
                supports_roof,
                (layer_idx - layer_generation_dtt) as LayerIndex,
                if supports_roof { mesh_group_settings.support_roof_line_distance } else { mesh_group_settings.support_tree_branch_distance },
                fillers,
            ),
            (if continuous_tips { config.min_radius / 2 } else { connect_length }) as f64,
            1,
        );
        let mut point_count: usize = 0;
        for poly in &polylines {
            point_count += poly.points.len();
        }
        // std::max(coord_t(1), std::min(coord_t(3), coord_t(total_length(overhang_area) / connect_length)))
        let min_support_points: usize =
            std::cmp::max(1, std::cmp::min(3, (total_length(&overhang_area) / connect_length as f64) as Coord)) as usize;
        if point_count <= min_support_points {
            // add the outer wall (of the overhang) to ensure it is correct supported instead. Try placing the support points in a way that they fully support the outer wall, instead of just the with half of the the support line width.
            // I assume that even small overhangs are over one line width wide, so lets try to place the support points in a way that the full support area generated from them
            // will support the overhang (if this is not done it may only be half). This WILL NOT be the case when supporting an angle of about < 60 degrees so there is a fallback,
            // as some support is better than none.
            let reduced_overhang_area: Polygons = cu::offset_expolygons(
                &cu::union_ex(&overhang_area),
                (-(config.support_line_width as f64) / 2.2) as f32,
                crate::clipper::JoinType::jtMiter,
                1.2,
            );
            let tiny = scaled_f64(0.001);
            polylines = ensure_maximum_distance_polyline(
                &to_polylines(
                    if !reduced_overhang_area.is_empty()
                        && crate::geometry::area_polygons(&cu::offset_expolygons(
                            &cu::diff_ex(&overhang_area, &reduced_overhang_area, cu::ApplySafetyOffset::No),
                            std::cmp::max(config.support_line_width, connect_length) as f32,
                            crate::clipper::JoinType::jtMiter,
                            1.2,
                        )) < tiny * tiny
                    {
                        &reduced_overhang_area
                    } else {
                        &overhang_area
                    },
                ),
                connect_length as f64,
                min_support_points,
            );
        }
        overhang_lines = convert_lines_to_internal(volumes, config, &polylines, (layer_idx.wrapping_sub(dtt_roof)) as LayerIndex, xy_min_distance);
    }

    if dtt_roof >= layer_idx && large_horizontal_roof {
        // Reached buildplate when generating contact, interface and base interface layers.
        interface_placer.add_roof_build_plate(overhang_area, dtt_roof);
    } else {
        // normal trees have to be generated
        let roof_enabled = num_support_roof_layers > 0;
        let force_tip_to_roof = interface_placer.force_tip_to_roof;
        interface_placer.add_points_along_lines(
            // Sample along these lines
            overhang_lines,
            // First layer index to insert the tree tips or interfaces.
            (layer_idx - dtt_roof) as LayerIndex,
            // Remaining roof tip layers.
            if force_tip_to_roof { num_support_roof_layers.wrapping_sub(dtt_roof) } else { 0 },
            // Supports roof already? How many roof layers were already produced above these tips?
            dtt_roof,
            // Don't move until the following distance to top is reached.
            if roof_enabled { num_support_roof_layers.wrapping_sub(dtt_roof) } else { 0 },
            xy_min_distance,
        );
    }
}

// Polygon.hpp:122-128 `inline double total_length(const Polygons &polylines)`: the sum of `Polygon::length()`
// (Polygon.cpp: the lengths of the edges, the closing one included).
fn total_length(polygons: &[Polygon]) -> f64 {
    let mut total = 0.;
    for polygon in polygons {
        // double Polygon::length() const
        let mut l = 0.;
        if polygon.points.len() > 1 {
            l = norm2(vec2d(*polygon.points.last().unwrap() - polygon.points[0]));
            for i in 1..polygon.points.len() {
                l += norm2(vec2d(polygon.points[i] - polygon.points[i - 1]));
            }
        }
        total += l;
    }
    total
}

// Polygon.hpp:230-240 `inline Polylines to_polylines(const Polygons &polygons)`: every polygon closed by its first point.
fn to_polylines(polygons: &[Polygon]) -> Polylines {
    let mut out: Polylines = Vec::with_capacity(polygons.len());
    for polygon in polygons {
        let mut pl = Polyline::new();
        pl.points = polygon.points.clone();
        if let Some(front) = polygon.points.first() {
            pl.points.push(*front);
        }
        out.push(pl);
    }
    out
}

/// TreeSupport3D.cpp:1161-1427 `void generate_initial_areas(const PrintObject &print_object, const TreeModelVolumes
/// &volumes, const TreeSupportSettings &config, const std::vector<Polygons> &overhangs, std::vector<SupportElements>
/// &move_bounds, InterfacePlacer &interface_placer, std::function<void()> throw_on_cancel)`
///
/// Creates the initial influence areas (that can later be propagated down) by placing them below the overhang.
/// Generates Points where the Model should be supported and creates the areas where these points have to be placed.
///
/// `print_object.print()` is `print`; `outputs` are the outputs of the interface placer; `fillers` is
/// `Fill::new_from_type`. An exception of the C++ (a flow that cannot be made) is the `Err`.
#[allow(clippy::too_many_arguments)]
pub fn generate_initial_areas(
    print_object: &pf::PrintObject,
    print: &pf::Print,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    overhangs: &[Polygons],
    move_bounds: &mut Vec<SupportElements>,
    interface_placer: &mut InterfacePlacer,
    mut outputs: InterfacePlacerOutputs,
    fillers: &dyn SupportFillFactory,
    throw_on_cancel: ThrowOnCancel,
) -> crate::flow::FlowResult<()> {
    use std::f64::consts::PI as M_PI;
    let mesh_group_settings = TreeSupportMeshGroupSettings::new(print_object, print)?;
    let split_overhangs_by_region = print.uses_multiple_nozzle_diameters()
        && config.xy_min_distance_per_region.len() == print_object.num_printing_regions()
        // std::adjacent_find(..., [](coord_t lhs, coord_t rhs) { return lhs != rhs; }) != end()
        && config.xy_min_distance_per_region.windows(2).any(|w| w[0] != w[1]);

    let z_distance_delta: usize = config.z_distance_top_layers + 1;

    // `(config.support_line_width * 100. / mesh_group_settings.support_tree_top_rate) + std::max(2. * config.min_radius -
    // 1.0 * config.support_line_width, 0.0)`: the difference of the two products with the left one fused.
    let connect_length: Coord = {
        let d = 2.0_f64.mul_add(config.min_radius as f64, -(1.0 * config.support_line_width as f64));
        ((config.support_line_width as f64 * 100. / mesh_group_settings.support_tree_top_rate) + if d < 0.0 { 0.0 } else { d }) as Coord
    };
    // As r*r=x*x+y*y (circle equation): If a circle with center at (0,0) the top most point is at (0,r) as in y=r.
    // This calculates how far one has to move on the x-axis so that y=r-support_line_width/2.
    // In other words how far does one need to move on the x-axis to be support_line_width/2 away from the circle line.
    // As a circle is round this length is identical for every axis as long as the 90 degrees angle between both remains.
    let circle_length_to_half_linewidth_change: Coord = if config.min_radius < config.support_line_width {
        config.min_radius / 2
    } else {
        // scale_(sqrt(sqr(unscale<double>(config.min_radius)) - sqr(unscale<double>(config.min_radius - config.support_line_width / 2))))
        let a = config.min_radius as f64 * SCALING_FACTOR;
        let b = (config.min_radius - config.support_line_width / 2) as f64 * SCALING_FACTOR;
        ((a * a - b * b).sqrt() / SCALING_FACTOR) as Coord
    };
    // Extra support offset to compensate for larger tip radiis. Also outset a bit more when z overwrites xy, because supporting something with a part of a support line is better than not supporting it at all.
    //FIXME Vojtech: This is not sufficient for support enforcers to work.
    //FIXME There is no account for the support overhang angle.
    //FIXME There is no account for the width of the collision regions.
    let num_support_roof_layers: usize = mesh_group_settings.support_roof_layers as usize;
    let roof_enabled = num_support_roof_layers > 0;
    let force_tip_to_roof = roof_enabled
        && (interface_placer.support_parameters.soluble_interface
            || (config.min_radius as f64 * config.min_radius as f64) * M_PI > mesh_group_settings.minimum_roof_area);
    // cap for how much layer below the overhang a new support point may be added, as other than with regular support every new inserted point
    // may cause extra material and time cost.  Could also be an user setting or differently calculated. Idea is that if an overhang
    // does not turn valid in double the amount of layers a slope of support angle would take to travel xy_distance, nothing reasonable will come from it.
    // The 2*z_distance_delta is only a catch for when the support angle is very high.
    // Used only if not min_xy_dist.
    // (`max_overhang_insert_lag` is read only by a block the C++ compiles out.)

    let num_support_layers: usize;
    let raft_contact_layer_idx: i32;
    // Layers with their overhang regions.
    struct RawOverhang {
        layer_idx: usize,
        polygons: Polygons,
        xy_min_distance: Coord,
    }
    let mut raw_overhangs: Vec<RawOverhang> = Vec::new();

    {
        let num_raft_layers: usize = config.raft_layers.len();
        let first_support_layer: usize = std::cmp::max(num_raft_layers as i32 - z_distance_delta as i32, 1) as usize;
        num_support_layers = std::cmp::max(0, print_object.layer_count() as i32 + num_raft_layers as i32 - z_distance_delta as i32) as usize;
        raft_contact_layer_idx = generate_raft_contact(print_object, config, interface_placer, &mut outputs);
        // Enumerate layers for which the support tips may be generated from overhangs above.
        for layer_idx in first_support_layer..num_support_layers {
            let overhang_idx = layer_idx + z_distance_delta;
            if overhangs[overhang_idx].is_empty() {
                continue;
            }

            let mut remaining: Polygons = overhangs[overhang_idx].clone();
            let object_layer_idx = overhang_idx - num_raft_layers;
            if split_overhangs_by_region && object_layer_idx < print_object.layer_count() {
                let object_layer = print_object.get_layer(object_layer_idx as i32);
                let mut region_areas: Vec<(Coord, Polygons)> = Vec::with_capacity(object_layer.regions().len());
                for layerm in object_layer.regions() {
                    let region_id = layerm.region().print_region_id() as usize;
                    if region_id >= config.xy_min_distance_per_region.len() {
                        continue;
                    }
                    let mut region_area: Polygons = Vec::new();
                    if layerm.adaptive_merge_patches.is_empty() {
                        // to_polygons(layerm->slices.surfaces)
                        for surface in &layerm.slices.surfaces {
                            region_area.push(surface.expolygon.contour.clone());
                            region_area.extend(surface.expolygon.holes.iter().cloned());
                        }
                    } else {
                        for patch in &layerm.adaptive_merge_patches {
                            region_area.extend(crate::geometry::to_polygons(patch.area()));
                        }
                    }
                    region_areas.push((config.xy_min_distance_per_region[region_id], region_area));
                }
                // std::sort(region_areas.begin(), region_areas.end(), [](const auto &lhs, const auto &rhs) { return lhs.first <
                // rhs.first; }), as libc++ orders equal keys.
                let mut order: Vec<usize> = (0..region_areas.len()).collect();
                crate::clipper::libcxx_sort(&mut order, |&lhs, &rhs| region_areas[lhs].0 < region_areas[rhs].0);
                for &i in &order {
                    let (xy_min_distance, region_area) = &region_areas[i];
                    let part: Polygons = cu::intersection(&remaining, region_area, cu::ApplySafetyOffset::No);
                    if !part.is_empty() {
                        raw_overhangs.push(RawOverhang { layer_idx, polygons: part, xy_min_distance: *xy_min_distance });
                        remaining = cu::diff(&remaining, region_area, cu::ApplySafetyOffset::No);
                    }
                }
            }
            if !remaining.is_empty() {
                raw_overhangs.push(RawOverhang { layer_idx, polygons: remaining, xy_min_distance: config.xy_min_distance });
            }
        }
    }

    {
        let mut rich_interface_placer = RichInterfacePlacer::new(
            InterfacePlacer::new(interface_placer.slicing_parameters, interface_placer.support_parameters, interface_placer.config),
            InterfacePlacerOutputs {
                layer_storage: &mut *outputs.layer_storage,
                top_contacts: &mut *outputs.top_contacts,
                top_interfaces: &mut *outputs.top_interfaces,
                top_base_interfaces: &mut *outputs.top_base_interfaces,
            },
            volumes,
            force_tip_to_roof,
            num_support_layers,
            &mut *move_bounds,
            fillers,
        );

        // tbb::parallel_for over the raw overhangs: on one thread the pin takes them in their order.
        for raw_overhang in &raw_overhangs {
            let layer_idx: usize = raw_overhang.layer_idx;
            let overhang_raw: &Polygons = &raw_overhang.polygons;
            let xy_min_distance: Coord = raw_overhang.xy_min_distance;
            let min_xy_dist = config.xy_distance > xy_min_distance;
            let collision_radius: Coord =
                config.get_radius(0, 0.) + if min_xy_dist { std::cmp::max(0, xy_min_distance - config.xy_min_distance) } else { 0 };
            let extra_outset: Coord = std::cmp::max(0, config.min_radius - config.support_line_width / 2)
                + if min_xy_dist { config.support_line_width / 2 } else { 0 };

            // take the least restrictive avoidance possible
            let relevant_forbidden: Polygons;
            {
                let relevant_forbidden_raw = if config.support_rests_on_model {
                    volumes.get_collision(collision_radius, layer_idx as LayerIndex, min_xy_dist)
                } else {
                    volumes.get_avoidance(collision_radius, layer_idx as LayerIndex, AvoidanceType::Fast, false, min_xy_dist)
                };
                // prevent rounding errors down the line, points placed directly on the line of the forbidden area may not be added otherwise.
                relevant_forbidden =
                    cu::offset_expolygons(&cu::union_ex(&relevant_forbidden_raw), (0.005 / SCALING_FACTOR) as f32, crate::clipper::JoinType::jtMiter, 1.2);
            }

            // every overhang has saved if a roof should be generated for it. This can NOT be done in the for loop as an area may NOT have a roof
            // even if it is larger than the minimum_roof_area when it is only larger because of the support horizontal expansion and
            // it would not have a roof if the overhang is offset by support roof horizontal expansion instead. (At least this is the current behavior of the regular support)
            let mut overhang_regular: Polygons;
            {
                // When support_offset = 0 safe_offset_inc will only be the difference between overhang_raw and relevant_forbidden, that has to be calculated anyway.
                overhang_regular = safe_offset_inc(
                    overhang_raw,
                    mesh_group_settings.support_offset,
                    &relevant_forbidden,
                    (config.min_radius as f64).mul_add(1.75, xy_min_distance as f64) as Coord,
                    0,
                    1,
                );

                // offset ensures that areas that could be supported by a part of a support line, are not considered unsupported overhang
                let mut remaining_overhang: Polygons = cu::intersection(
                    &cu::diff(
                        &if mesh_group_settings.support_offset == 0 {
                            overhang_raw.clone()
                        } else {
                            cu::offset_expolygons(
                                &cu::union_ex(overhang_raw),
                                mesh_group_settings.support_offset as f32,
                                crate::clipper::JoinType::jtMiter,
                                1.2,
                            )
                        },
                        &cu::offset_expolygons(
                            &cu::union_ex(&overhang_regular),
                            (config.support_line_width as f64 * 0.5) as f32,
                            crate::clipper::JoinType::jtMiter,
                            1.2,
                        ),
                        cu::ApplySafetyOffset::No,
                    ),
                    &relevant_forbidden,
                    cu::ApplySafetyOffset::No,
                );

                // Offset the area to compensate for large tip radiis. Offset happens in multiple steps to ensure the tip is as close to the original overhang as possible.
                //+config.support_line_width / 80  to avoid calculating very small (useless) offsets because of rounding errors.
                //FIXME likely a better approach would be to find correspondences between the full overhang and the trimmed overhang
                // and if there is no correspondence, project the missing points to the clipping curve.
                let mut extra_total_offset_acc: Coord = 0;
                while !remaining_overhang.is_empty() && extra_total_offset_acc + config.support_line_width / 8 < extra_outset {
                    let offset_current_step: Coord = std::cmp::min(
                        if extra_total_offset_acc + 2 * config.support_line_width > config.min_radius {
                            config.support_line_width / 8
                        } else {
                            circle_length_to_half_linewidth_change
                        },
                        extra_outset - extra_total_offset_acc,
                    );
                    extra_total_offset_acc += offset_current_step;
                    let raw_collision = volumes.get_collision(0, layer_idx as LayerIndex, true);
                    let offset_step: Coord = xy_min_distance + config.support_line_width;
                    // Reducing the remaining overhang by the areas already supported.
                    //FIXME 1.5 * extra_total_offset_acc seems to be too much, it may remove some remaining overhang without being supported at all.
                    remaining_overhang = cu::diff(
                        &remaining_overhang,
                        &safe_offset_inc(&overhang_regular, (1.5 * extra_total_offset_acc as f64) as Coord, &raw_collision, offset_step, 0, 1),
                        cu::ApplySafetyOffset::No,
                    );
                    // Extending the overhangs by the inflated remaining overhangs.
                    overhang_regular = cu::union_2(
                        &overhang_regular,
                        &cu::diff(
                            &safe_offset_inc(&remaining_overhang, extra_total_offset_acc, &raw_collision, offset_step, 0, 1),
                            &relevant_forbidden,
                            cu::ApplySafetyOffset::No,
                        ),
                    );
                }
            }

            throw_on_cancel();

            if roof_enabled {
                const SUPPORT_ROOF_OFFSET: Coord = 0;
                let mut overhang_roofs: Polygons = safe_offset_inc(
                    overhang_raw,
                    SUPPORT_ROOF_OFFSET,
                    &relevant_forbidden,
                    config.min_radius * 2 + xy_min_distance,
                    0,
                    1,
                );
                if mesh_group_settings.minimum_support_area > 0 {
                    crate::geometry::polygon::remove_small(&mut overhang_roofs, mesh_group_settings.minimum_roof_area);
                }
                overhang_regular = cu::diff(&overhang_regular, &overhang_roofs, cu::ApplySafetyOffset::Yes);
                for roof_part in cu::union_ex(&overhang_roofs) {
                    sample_overhang_area(
                        crate::geometry::to_polygons(std::slice::from_ref(&roof_part)),
                        true,
                        layer_idx,
                        num_support_roof_layers,
                        connect_length,
                        xy_min_distance,
                        &mesh_group_settings,
                        &mut rich_interface_placer,
                    );
                    throw_on_cancel();
                }
            }
            // Either the roof is not enabled, then these are all the overhangs to be supported,
            // or roof is enabled and these are the thin overhangs at object slopes (not horizontal overhangs).
            if mesh_group_settings.minimum_support_area > 0 {
                crate::geometry::polygon::remove_small(&mut overhang_regular, mesh_group_settings.minimum_support_area as f64);
            }

            for support_part in cu::union_ex(&overhang_regular) {
                sample_overhang_area(
                    crate::geometry::to_polygons(std::slice::from_ref(&support_part)),
                    false,
                    layer_idx,
                    num_support_roof_layers,
                    connect_length,
                    xy_min_distance,
                    &mesh_group_settings,
                    &mut rich_interface_placer,
                );
                throw_on_cancel();
            }
        }
    }

    finalize_raft_contact(print_object, raft_contact_layer_idx, outputs.layer_storage, outputs.top_contacts, move_bounds);
    Ok(())
}

// =============================================================================
// The port of the pin: TreeSupport3D.cpp, the influence areas (increase_single_area() to
// create_layer_pathing())
// =============================================================================

use crate::support::support_common::safe_union;

// ClipperUtils.cpp:679-680 `Polygons diff_clipped(const Polygons &subject, const Polygons &clip, ApplySafetyOffset
// do_safety_offset)`: the difference with the clip polygons clipped by the bounding box of the subject.
fn diff_clipped(subject: &[Polygon], clip: &[Polygon]) -> Polygons {
    // get_extents(subject).inflated(SCALED_EPSILON)
    let mut bbox = crate::geometry::get_extents_polygons(subject);
    if !bbox.is_defined() {
        // The difference of nothing.
        return Vec::new();
    }
    let eps = SCALED_EPSILON as Coord;
    bbox.min = Point::new(bbox.min.x - eps, bbox.min.y - eps);
    bbox.max = Point::new(bbox.max.x + eps, bbox.max.y + eps);
    cu::diff(subject, &crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_polygons(clip, &bbox), cu::ApplySafetyOffset::No)
}

// A division of `coord_t` as the engine's arm64 build does it: `sdiv` gives zero for a zero divisor, where the C++
// is undefined (a distance of zero to the object in the plane and no gap above or below).
#[inline]
fn div_coord(a: Coord, b: Coord) -> Coord {
    if b == 0 {
        0
    } else {
        a / b
    }
}

// Polygon.hpp `inline double area(const Polygons &polys)`
#[inline]
fn area(polygons: &[Polygon]) -> f64 {
    crate::geometry::area_polygons(polygons)
}

/// TreeSupport3D.cpp:1559-1695 `static std::optional<SupportElementState> increase_single_area(const TreeModelVolumes
/// &volumes, const TreeSupportSettings &config, const AreaIncreaseSettings &settings, const LayerIndex layer_idx,
/// const SupportElement &parent, const Polygons &relevant_offset, Polygons &to_bp_data, Polygons &to_model_data,
/// Polygons &increased, const coord_t overspeed, const bool mergelayer)`
///
/// Checks if an influence area contains a valid subsection and returns the corresponding metadata and the new
/// Influence area.
#[allow(clippy::too_many_arguments)]
#[must_use]
pub fn increase_single_area(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    settings: &AreaIncreaseSettings,
    layer_idx: LayerIndex,
    parent: &SupportElement,
    relevant_offset: &Polygons,
    to_bp_data: &mut Polygons,
    to_model_data: &mut Polygons,
    increased: &mut Polygons,
    overspeed: Coord,
    mergelayer: bool,
) -> Option<SupportElementState> {
    let mut current_elem: SupportElementState = SupportElementState::propagate_down(&parent.state);
    if settings.increase_radius {
        current_elem.effective_radius_height += 1;
    }
    let mut radius: Coord = support_element_collision_radius(config, &current_elem);
    let xy_clearance_delta: Coord =
        if settings.use_min_distance { std::cmp::max(0, current_elem.xy_min_distance - config.xy_min_distance) } else { 0 };
    let model_radius = |branch_radius: Coord| -> Coord { branch_radius + xy_clearance_delta };

    if settings.move_ {
        *increased = relevant_offset.clone();
        if overspeed > 0 {
            let safe_movement_distance: Coord = (if current_elem.use_min_xy_dist { current_elem.xy_min_distance } else { config.xy_distance })
                + if std::cmp::min(config.z_distance_top_layers, config.z_distance_bottom_layers) > 0 { config.min_feature_size } else { 0 };
            // The difference to ensure that the result not only conforms to wall_restriction, but collision/avoidance is done later.
            // The higher last_safe_step_movement_distance comes exactly from the fact that the collision will be subtracted later.
            *increased = safe_offset_inc(
                increased,
                overspeed,
                &volumes.get_wall_restriction(
                    model_radius(support_element_collision_radius(config, &parent.state)),
                    layer_idx,
                    parent.state.use_min_xy_dist,
                ),
                safe_movement_distance,
                safe_movement_distance + radius,
                1,
            );
        }
        // `if (settings.no_error && settings.move) polygons_simplify(increased, scaled<float>(0.025));`
        // (TreeSupport3D.cpp:1594-1596): polygons_simplify() takes its polygons by const reference and returns the
        // simplified ones (Polygon.hpp:163), which the pin drops here: `increased` stays as it is.
    } else {
        // if no movement is done the areas keep parent area as no move == offset(0)
        *increased = parent.influence_area.clone();
    }

    if mergelayer || current_elem.to_buildplate {
        *to_bp_data = safe_union(
            &diff_clipped(increased, &volumes.get_avoidance(model_radius(radius), layer_idx - 1, settings.type_, false, settings.use_min_distance)),
            &Vec::new(),
        );
        if !current_elem.to_buildplate && area(to_bp_data) > tiny_area_threshold() {
            // mostly happening in the tip, but with merges one should check every time, just to be sure.
            current_elem.to_buildplate = true; // sometimes nodes that can reach the buildplate are marked as cant reach, tainting subtrees. This corrects it.
        }
    }
    if config.support_rests_on_model {
        if mergelayer || current_elem.to_model_gracious {
            *to_model_data = safe_union(
                &diff_clipped(increased, &volumes.get_avoidance(model_radius(radius), layer_idx - 1, settings.type_, true, settings.use_min_distance)),
                &Vec::new(),
            );
        }

        if !current_elem.to_model_gracious {
            if mergelayer && area(to_model_data) >= tiny_area_threshold() {
                current_elem.to_model_gracious = true;
            } else {
                *to_model_data = safe_union(
                    &diff_clipped(increased, &volumes.get_collision(model_radius(radius), layer_idx - 1, settings.use_min_distance)),
                    &Vec::new(),
                );
            }
        }
    }

    // `check_layer_data = current_elem.to_buildplate ? to_bp_data : to_model_data;`: a copy of which only the area
    // is read.
    let mut check_layer_data_area: f64 = area(if current_elem.to_buildplate { &*to_bp_data } else { &*to_model_data });

    if settings.increase_radius && check_layer_data_area > tiny_area_threshold() {
        let valid_with_radius = |next_radius: Coord, current_elem: &SupportElementState, increased: &Polygons| -> bool {
            if volumes.ceil_radius_min_xy(model_radius(next_radius), settings.use_min_distance)
                <= volumes.ceil_radius_min_xy(model_radius(radius), settings.use_min_distance)
            {
                return true;
            }

            let mut to_bp_data_2: Polygons = Vec::new();
            if current_elem.to_buildplate {
                // regular union as output will not be used later => this area should always be a subset of the safe_union one (i think)
                to_bp_data_2 = diff_clipped(
                    increased,
                    &volumes.get_avoidance(model_radius(next_radius), layer_idx - 1, settings.type_, false, settings.use_min_distance),
                );
            }
            let mut to_model_data_2: Polygons = Vec::new();
            if config.support_rests_on_model && !current_elem.to_buildplate {
                to_model_data_2 = diff_clipped(
                    increased,
                    &if current_elem.to_model_gracious {
                        volumes.get_avoidance(model_radius(next_radius), layer_idx - 1, settings.type_, true, settings.use_min_distance)
                    } else {
                        volumes.get_collision(model_radius(next_radius), layer_idx - 1, settings.use_min_distance)
                    },
                );
            }
            let check_layer_data_2 = if current_elem.to_buildplate { &to_bp_data_2 } else { &to_model_data_2 };
            area(check_layer_data_2) > tiny_area_threshold()
        };
        let ceil_radius_before: Coord = volumes.ceil_radius_min_xy(model_radius(radius), settings.use_min_distance);

        if support_element_collision_radius(config, &current_elem) < config.increase_radius_until_radius
            && support_element_collision_radius(config, &current_elem) < support_element_radius(config, &current_elem)
        {
            let target_radius: Coord = std::cmp::min(support_element_radius(config, &current_elem), config.increase_radius_until_radius);
            let mut current_ceil_radius: Coord = volumes.get_radius_next_ceil(model_radius(radius), settings.use_min_distance) - xy_clearance_delta;

            while current_ceil_radius < target_radius {
                let next_radius: Coord =
                    volumes.get_radius_next_ceil(model_radius(current_ceil_radius + 1), settings.use_min_distance) - xy_clearance_delta;
                if !valid_with_radius(next_radius, &current_elem, increased) {
                    break;
                }
                current_ceil_radius = next_radius;
            }
            let mut resulting_eff_dtt: usize = current_elem.effective_radius_height as usize;
            while resulting_eff_dtt + 1 < current_elem.distance_to_top as usize
                && config.get_radius(resulting_eff_dtt + 1, current_elem.elephant_foot_increases) <= current_ceil_radius
                && config.get_radius(resulting_eff_dtt + 1, current_elem.elephant_foot_increases) <= support_element_radius(config, &current_elem)
            {
                resulting_eff_dtt += 1;
            }
            current_elem.effective_radius_height = resulting_eff_dtt as u32;
        }
        radius = support_element_collision_radius(config, &current_elem);

        // Plate foot is applied as 2D slices after extrude (append_organic_plate_foot_slices).
        // Do not grow elephant_foot_increases here: inflating the node radius would also
        // enlarge self-intersection repair hemispheres into a circular blob.

        if ceil_radius_before != volumes.ceil_radius_min_xy(model_radius(radius), settings.use_min_distance) {
            if current_elem.to_buildplate {
                *to_bp_data = safe_union(
                    &diff_clipped(increased, &volumes.get_avoidance(model_radius(radius), layer_idx - 1, settings.type_, false, settings.use_min_distance)),
                    &Vec::new(),
                );
            }
            if config.support_rests_on_model && (!current_elem.to_buildplate || mergelayer) {
                *to_model_data = safe_union(
                    &diff_clipped(
                        increased,
                        &if current_elem.to_model_gracious {
                            volumes.get_avoidance(model_radius(radius), layer_idx - 1, settings.type_, true, settings.use_min_distance)
                        } else {
                            volumes.get_collision(model_radius(radius), layer_idx - 1, settings.use_min_distance)
                        },
                    ),
                    &Vec::new(),
                );
            }
            check_layer_data_area = area(if current_elem.to_buildplate { &*to_bp_data } else { &*to_model_data });
            if check_layer_data_area < tiny_area_threshold() {
                tree_supports_show_error("Area lost catching up radius. May not cause visible malformation.", true);
            }
        }
    }

    if check_layer_data_area > tiny_area_threshold() {
        Some(current_elem)
    } else {
        None
    }
}

/// TreeSupport3D.cpp:1697-1710 `struct SupportElementInfluenceAreas`
#[derive(Debug, Clone, Default)]
pub struct SupportElementInfluenceAreas {
    /// All influence areas: both to build plate and model.
    pub influence_areas: Polygons,
    /// Influence areas just to build plate.
    pub to_bp_areas: Polygons,
    /// Influence areas just to model.
    pub to_model_areas: Polygons,
}

impl SupportElementInfluenceAreas {
    /// TreeSupport3D.cpp:1705-1709
    pub fn clear(&mut self) {
        self.influence_areas.clear();
        self.to_bp_areas.clear();
        self.to_model_areas.clear();
    }
}

/// TreeSupport3D.cpp:1712-1731 `struct SupportElementMerging`
#[derive(Debug, Clone)]
pub struct SupportElementMerging {
    pub state: SupportElementState,
    /// All elements in the layer above the current one that are supported by this element
    pub parents: ParentIndices,

    pub areas: SupportElementInfluenceAreas,
    /// Bounding box of all influence areas: `Eigen::AlignedBox<coord_t, 2> bbox_data`, its corners. An
    /// `AlignedBox` starts empty (the highest `coord_t` for its minimum, the lowest for its maximum).
    pub bbox_data: (Point, Point),
}

impl SupportElementMerging {
    /// `{ el.state, parents }` (TreeSupport3D.cpp:2476): the aggregate of a state and the parents.
    pub fn new(state: SupportElementState, parents: ParentIndices) -> Self {
        Self {
            state,
            parents,
            areas: SupportElementInfluenceAreas::default(),
            bbox_data: (Point::new(COORD_T_MAX, COORD_T_MAX), Point::new(i32::MIN as Coord, i32::MIN as Coord)),
        }
    }

    /// TreeSupport3D.cpp:1723 `const Eigen::AlignedBox<coord_t, 2>& bbox() const`
    pub fn bbox(&self) -> &(Point, Point) {
        &self.bbox_data
    }

    /// TreeSupport3D.cpp:1724 `const Point centroid() const`: a division of integers.
    pub fn centroid(&self) -> Point {
        Point::new((self.bbox_data.0.x + self.bbox_data.1.x) / 2, (self.bbox_data.0.y + self.bbox_data.1.y) / 2)
    }

    /// TreeSupport3D.cpp:1725-1726 `void set_bbox(const BoundingBox& abbox)`
    pub fn set_bbox(&mut self, abbox: &crate::geometry::BoundingBox) {
        let eps = Point::new(SCALED_EPSILON as Coord, SCALED_EPSILON as Coord);
        // A bounding box of nothing holds zeros (BoundingBox.hpp: `min`, `max` of `BoundingBoxBase()`).
        let (min, max) = if abbox.is_defined() { (abbox.min, abbox.max) } else { (Point::new(0, 0), Point::new(0, 0)) };
        self.bbox_data = (min - eps, max + eps);
    }

    /// What an assignment from `std::move(*this)` leaves and hands on: the vectors go with the value, the
    /// state and the bounding box are copied.
    fn take_moved(&mut self) -> SupportElementMerging {
        SupportElementMerging {
            state: self.state.clone(),
            parents: std::mem::take(&mut self.parents),
            areas: std::mem::take(&mut self.areas),
            bbox_data: self.bbox_data,
        }
    }
}

// Called by the AABBTree builder (`AABBTreeIndirect::Tree<2, coord_t>::build_modify_input()`).
impl crate::aabb_tree_lines::aabb_tree_indirect_2d::SourceNode for SupportElementMerging {
    /// TreeSupport3D.cpp:1728-1730 `static size_t idx() { return 0; }`: not needed, thus zero is returned.
    fn idx(&self) -> usize {
        0
    }
    fn centroid(&self) -> [f64; 2] {
        let c = SupportElementMerging::centroid(self);
        [c.x as f64, c.y as f64]
    }
    fn bbox(&self) -> crate::aabb_tree_lines::aabb_tree_indirect_2d::BoundingBox {
        crate::aabb_tree_lines::aabb_tree_indirect_2d::BoundingBox {
            min: [self.bbox_data.0.x as f64, self.bbox_data.0.y as f64],
            max: [self.bbox_data.1.x as f64, self.bbox_data.1.y as f64],
        }
    }
}

/// TreeSupport3D.cpp:1751-2004 `static void increase_areas_one_layer(const TreeModelVolumes &volumes, const
/// TreeSupportSettings &config, std::vector<SupportElementMerging> &merging_areas, const LayerIndex layer_idx,
/// SupportElements &layer_elements, const bool mergelayer, std::function<void()> throw_on_cancel)`
///
/// Increases influence areas as far as required.
///
/// The pin runs the elements through `tbb::parallel_for`; an element reads and writes its own merging area and
/// its own parent only, so they are taken in their order here.
#[allow(clippy::too_many_arguments)]
pub fn increase_areas_one_layer(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    // New areas at the layer below layer_idx
    merging_areas: &mut [SupportElementMerging],
    // Layer above merging_areas.
    layer_idx: LayerIndex,
    // Layer elements above merging_areas.
    layer_elements: &mut SupportElements,
    // If false, the merging_areas will not be merged for performance reasons.
    mergelayer: bool,
    throw_on_cancel: ThrowOnCancel,
) {
    for merging_area in merging_areas.iter_mut() {
        debug_assert!(merging_area.parents.len() == 1);
        let parent_idx = merging_area.parents[0] as usize;
        let parent: &SupportElement = &layer_elements[parent_idx];
        let mut elem: SupportElementState = SupportElementState::propagate_down(&parent.state);
        let wall_restriction =
            // Abstract representation of the model outline. If an influence area would move through it, it could teleport through a wall.
            volumes.get_wall_restriction(support_element_model_collision_radius(config, &parent.state), layer_idx, parent.state.use_min_xy_dist);

        let mut to_bp_data: Polygons = Vec::new();
        let mut to_model_data: Polygons = Vec::new();
        let mut radius: Coord = support_element_collision_radius(config, &elem);

        // When the radius increases, the outer "support wall" of the branch will have been moved farther away from the center (as this is the definition of radius).
        // As it is not specified that the support_tree_angle has to be one of the center of the branch, it is here seen as the smaller angle of the outer wall of the branch, to the outer wall of the same branch one layer above.
        // As the branch may have become larger the distance between these 2 walls is smaller than the distance of the center points.
        // These extra distance is added to the movement distance possible for this layer.

        let mut extra_speed: Coord = 5; // The extra speed is added to both movement distances. Also move 5 microns faster than allowed to avoid rounding errors, this may cause issues at VERY VERY small layer heights.
        let mut extra_slow_speed: Coord = 0; // Only added to the slow movement distance.
        let ceiled_parent_radius: Coord =
            volumes.ceil_radius_min_xy(support_element_model_collision_radius(config, &parent.state), parent.state.use_min_xy_dist);
        let projected_radius_increased: Coord =
            config.get_radius(parent.state.effective_radius_height as usize + 1, parent.state.elephant_foot_increases);
        let projected_radius_delta: Coord = projected_radius_increased - support_element_collision_radius(config, &parent.state);

        // When z distance is more than one layer up and down the Collision used to calculate the wall restriction will always include the wall (and not just the xy_min_distance) of the layer above and below like this (d = blocked area because of z distance):
        /*
         *  layer z+1:dddddiiiiiioooo
         *  layer z+0:xxxxxdddddddddd
         *  layer z-1:dddddxxxxxxxxxx
         *  For more detailed visualisation see calculateWallRestrictions
         */
        let safe_movement_distance: Coord = (if elem.use_min_xy_dist { elem.xy_min_distance } else { config.xy_distance })
            + if std::cmp::min(config.z_distance_top_layers, config.z_distance_bottom_layers) > 0 { config.min_feature_size } else { 0 };
        if ceiled_parent_radius
            == volumes.ceil_radius_min_xy(
                projected_radius_increased
                    + if parent.state.use_min_xy_dist { std::cmp::max(0, parent.state.xy_min_distance - config.xy_min_distance) } else { 0 },
                parent.state.use_min_xy_dist,
            )
            || projected_radius_increased < config.increase_radius_until_radius
        {
            // If it is guaranteed possible to increase the radius, the maximum movement speed can be increased, as it is assumed that the maximum movement speed is the one of the slower moving wall
            extra_speed += projected_radius_delta;
        } else {
            // if a guaranteed radius increase is not possible, only increase the slow speed
            // Ensure that the slow movement distance can not become larger than the fast one.
            extra_slow_speed += std::cmp::min(
                projected_radius_delta,
                (config.maximum_move_distance + extra_speed) - (config.maximum_move_distance_slow + extra_slow_speed),
            );
        }

        let fast_speed: Coord = config.maximum_move_distance + extra_speed;
        let slow_speed: Coord = config.maximum_move_distance_slow + extra_speed + extra_slow_speed;

        let mut offset_slow: Polygons = Vec::new();
        let mut offset_fast: Polygons = Vec::new();

        let mut add = false;
        let mut bypass_merge = false;
        // aliases for better readability
        const INCREASE_RADIUS: bool = true;
        const NO_ERROR: bool = true;
        const USE_MIN_RADIUS: bool = true;
        const MOVE: bool = true;

        // Determine in which order configurations are checked if they result in a valid influence area. Check will stop if a valid area is found
        let mut order: Vec<AreaIncreaseSettings> = Vec::new();
        let insert_setting = |order: &mut Vec<AreaIncreaseSettings>, settings: AreaIncreaseSettings, back: bool| {
            if !order.contains(&settings) {
                if back {
                    order.push(settings);
                } else {
                    order.insert(0, settings);
                }
            }
        };

        let parent_moved_slow = elem.last_area_increase.increase_speed < config.maximum_move_distance;
        let avoidance_speed_mismatch = parent_moved_slow && elem.last_area_increase.type_ != AvoidanceType::Slow;
        if elem.last_area_increase.move_
            && elem.last_area_increase.no_error
            && elem.can_use_safe_radius
            && !mergelayer
            && !avoidance_speed_mismatch
            && (elem.distance_to_top as usize >= config.tip_layers || parent_moved_slow)
        {
            // assume that the avoidance type that was best for the parent is best for me. Makes this function about 7% faster.
            let speed = if elem.last_area_increase.increase_speed < config.maximum_move_distance { slow_speed } else { fast_speed };
            insert_setting(
                &mut order,
                AreaIncreaseSettings::new(
                    elem.last_area_increase.type_,
                    speed,
                    INCREASE_RADIUS,
                    elem.last_area_increase.no_error,
                    !USE_MIN_RADIUS,
                    elem.last_area_increase.move_,
                ),
                true,
            );
            insert_setting(
                &mut order,
                AreaIncreaseSettings::new(
                    elem.last_area_increase.type_,
                    speed,
                    !INCREASE_RADIUS,
                    elem.last_area_increase.no_error,
                    !USE_MIN_RADIUS,
                    elem.last_area_increase.move_,
                ),
                true,
            );
        }
        // branch may still go though a hole, so a check has to be done whether the hole was already passed, and the regular avoidance can be used.
        if !elem.can_use_safe_radius {
            // if the radius until which it is always increased can not be guaranteed, move fast. This is to avoid holes smaller than the real branch radius.
            // This does not guarantee the avoidance of such holes, but ensures they are avoided if possible.
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::Slow, slow_speed, INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, !MOVE), true); // did we go through the hole
            // in many cases the definition of hole is overly restrictive, so to avoid unnecessary fast movement in the tip, it is ignored there for a bit.
            // This CAN cause a branch to go though a hole it otherwise may have avoided.
            // round_up_divide(config.tip_layers, size_t(2))
            if (elem.distance_to_top as usize) < (config.tip_layers + 2 - 1) / 2 {
                insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::Fast, slow_speed, INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, !MOVE), true);
            }
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::FastSafe, fast_speed, INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, !MOVE), true); // did we manage to avoid the hole
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::FastSafe, fast_speed, !INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, MOVE), true);
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::Fast, fast_speed, !INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, MOVE), true);
        } else {
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::Slow, slow_speed, INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, MOVE), true);
            // while moving fast to be able to increase the radius (b) may seems preferable (over a) this can cause the a sudden skip in movement,
            // which looks similar to a layer shift and can reduce stability.
            // as such idx have chosen to only use the user setting for radius increases as a friendly recommendation.
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::Slow, slow_speed, !INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, MOVE), true); // a
            if (elem.distance_to_top as usize) < config.tip_layers {
                insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::FastSafe, slow_speed, INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, MOVE), true);
            }
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::FastSafe, fast_speed, INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, MOVE), true); // b
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::FastSafe, fast_speed, !INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, MOVE), true);
        }

        if elem.use_min_xy_dist {
            let mut new_order: Vec<AreaIncreaseSettings> = Vec::new();
            // if the branch currently has to use min_xy_dist check if the configuration would also be valid
            // with the regular xy_distance before checking with use_min_radius (Only happens when Support Distance priority is z overrides xy )
            for settings in &order {
                new_order.push(*settings);
                new_order.push(AreaIncreaseSettings::new(
                    settings.type_,
                    settings.increase_speed,
                    settings.increase_radius,
                    settings.no_error,
                    USE_MIN_RADIUS,
                    settings.move_,
                ));
            }
            order = new_order;
        }
        if elem.to_buildplate
            || (elem.to_model_gracious
                && cu::intersection(&parent.influence_area, &volumes.get_placeable_areas(radius, layer_idx, throw_on_cancel), cu::ApplySafetyOffset::No)
                    .is_empty())
        {
            // error case
            // it is normal that we wont be able to find a new area at some point in time if we wont be able to reach layer 0 aka have to connect with the model
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::Fast, fast_speed, !INCREASE_RADIUS, !NO_ERROR, elem.use_min_xy_dist, MOVE), true);
        }
        if elem.distance_to_top < elem.dont_move_until && elem.can_use_safe_radius {
            // only do not move when holes would be avoided in every case.
            // Only do not move when already in a no hole avoidance with the regular xy distance.
            insert_setting(&mut order, AreaIncreaseSettings::new(AvoidanceType::Slow, 0, INCREASE_RADIUS, NO_ERROR, !USE_MIN_RADIUS, !MOVE), false);
        }

        let mut inc_wo_collision: Polygons = Vec::new();
        // Check whether it is faster to calculate the area increased with the fast speed independently from the slow area, or time could be saved by reusing the slow area to calculate the fast one.
        // Calculated by comparing the steps saved when calcualting idependently with the saved steps when not.
        // round_up_divide(dividend, divisor) (Utils.hpp:302): `(dividend + divisor - 1) / divisor`
        let offset_independant_faster: bool = div_coord(radius, safe_movement_distance)
            - (config.maximum_move_distance + extra_speed < radius + safe_movement_distance) as Coord
            > div_coord(extra_speed + extra_slow_speed + config.maximum_move_distance_slow + safe_movement_distance - 1, safe_movement_distance);
        for settings in &order {
            if settings.move_ {
                if offset_slow.is_empty() && (settings.increase_speed == slow_speed || !offset_independant_faster) {
                    // offsetting in 2 steps makes our offsetted area rounder preventing (rounding) errors created by to pointy areas. At this point one can see that the Polygons class
                    // was never made for precision in the single digit micron range.
                    offset_slow = safe_offset_inc(
                        &parent.influence_area,
                        extra_speed + extra_slow_speed + config.maximum_move_distance_slow,
                        &wall_restriction,
                        safe_movement_distance,
                        if offset_independant_faster { safe_movement_distance + radius } else { 0 },
                        2,
                    );
                }
                if offset_fast.is_empty() && settings.increase_speed != slow_speed {
                    if offset_independant_faster {
                        offset_fast = safe_offset_inc(
                            &parent.influence_area,
                            extra_speed + config.maximum_move_distance,
                            &wall_restriction,
                            safe_movement_distance,
                            if offset_independant_faster { safe_movement_distance + radius } else { 0 },
                            1,
                        );
                    } else {
                        let delta_slow_fast: Coord = config.maximum_move_distance - (config.maximum_move_distance_slow + extra_slow_speed);
                        offset_fast = safe_offset_inc(
                            &offset_slow,
                            delta_slow_fast,
                            &wall_restriction,
                            safe_movement_distance,
                            safe_movement_distance + radius,
                            if offset_independant_faster { 2 } else { 1 },
                        );
                    }
                }
            }
            let result: Option<SupportElementState>;
            inc_wo_collision.clear();
            if !settings.no_error {
                // ERROR CASE
                // if the area becomes for whatever reason something that clipper sees as a line, offset would stop working, so ensure that even if it would be a line wrongly, it still actually has an area that can be increased
                let lines_offset: Polygons = cu::offset_polylines(
                    &to_polylines(&parent.influence_area),
                    (0.005 / SCALING_FACTOR) as f32,
                    crate::clipper::JoinType::jtMiter,
                    1.2,
                    crate::clipper::EndType::etOpenButt,
                );
                let base_error_area: Polygons = cu::union_2(&parent.influence_area, &lines_offset);
                result = increase_single_area(
                    volumes,
                    config,
                    settings,
                    layer_idx,
                    parent,
                    &base_error_area,
                    &mut to_bp_data,
                    &mut to_model_data,
                    &mut inc_wo_collision,
                    // `(config.maximum_move_distance + extra_speed) * 1.5` handed to a `coord_t`
                    ((config.maximum_move_distance + extra_speed) as f64 * 1.5) as i32 as Coord,
                    mergelayer,
                );
                tree_supports_show_error("Potentially lost branch!", true);
            } else {
                result = increase_single_area(
                    volumes,
                    config,
                    settings,
                    layer_idx,
                    parent,
                    if settings.increase_speed == slow_speed { &offset_slow } else { &offset_fast },
                    &mut to_bp_data,
                    &mut to_model_data,
                    &mut inc_wo_collision,
                    0,
                    mergelayer,
                );
            }

            if let Some(result) = result {
                elem = result;
                radius = support_element_collision_radius(config, &elem);
                elem.last_area_increase = *settings;
                add = true;
                // do not merge if the branch should not move or the priority has to be to get farther away from the model.
                bypass_merge = !settings.move_ || (settings.use_min_distance && (elem.distance_to_top as usize) < config.tip_layers);
                if settings.move_ {
                    elem.dont_move_until = 0;
                } else {
                    elem.result_on_layer = parent.state.result_on_layer;
                }

                elem.can_use_safe_radius = settings.type_ != AvoidanceType::Fast;

                if !settings.use_min_distance {
                    elem.use_min_xy_dist = false;
                }
                break;
            }
        }

        if add {
            // Union seems useless, but some rounding errors somewhere can cause to_bp_data to be slightly bigger than it should be.
            debug_assert!(!inc_wo_collision.is_empty() || !to_bp_data.is_empty() || !to_model_data.is_empty());
            let max_influence_area: Polygons = safe_union(
                &diff_clipped(
                    &inc_wo_collision,
                    &volumes.get_collision(
                        radius + if elem.use_min_xy_dist { std::cmp::max(0, elem.xy_min_distance - config.xy_min_distance) } else { 0 },
                        layer_idx - 1,
                        elem.use_min_xy_dist,
                    ),
                ),
                &safe_union(&to_bp_data, &to_model_data),
            );
            let to_buildplate = elem.to_buildplate;
            merging_area.state = elem;
            debug_assert!(!max_influence_area.is_empty());
            merging_area.set_bbox(&crate::geometry::get_extents_polygons(&max_influence_area));
            merging_area.areas.influence_areas = max_influence_area;
            if !bypass_merge {
                if to_buildplate {
                    merging_area.areas.to_bp_areas = to_bp_data;
                }
                if config.support_rests_on_model {
                    merging_area.areas.to_model_areas = to_model_data;
                }
            }
        } else {
            // If the bottom most point of a branch is set, later functions will assume that the position is valid, and ignore it.
            // But as branches connecting with the model that are to small have to be culled, the bottom most point has to be not set.
            // A point can be set on the top most tip layer (maybe more if it should not move for a few layers).
            let parent = &mut layer_elements[parent_idx];
            parent.state.result_on_layer_reset();
            parent.state.to_model_gracious = false;
        }
        throw_on_cancel();
    }
}

/// TreeSupport3D.cpp:2015-2062 `static SupportElementState merge_support_element_states(const SupportElementState
/// &first, const SupportElementState &second, const Point &next_position, const coord_t layer_idx, const
/// TreeSupportSettings &config)`
#[must_use]
pub fn merge_support_element_states(
    first: &SupportElementState,
    second: &SupportElementState,
    next_position: &Point,
    layer_idx: LayerIndex,
    config: &TreeSupportSettings,
) -> SupportElementState {
    let mut out = SupportElementState::default();
    out.next_position = *next_position;
    out.layer_idx = layer_idx;
    out.use_min_xy_dist = first.use_min_xy_dist || second.use_min_xy_dist;
    out.xy_min_distance = if out.use_min_xy_dist { std::cmp::max(first.xy_min_distance, second.xy_min_distance) } else { config.xy_min_distance };
    out.supports_roof = first.supports_roof || second.supports_roof;
    out.dont_move_until = std::cmp::max(first.dont_move_until, second.dont_move_until);
    out.can_use_safe_radius = first.can_use_safe_radius || second.can_use_safe_radius;
    out.missing_roof_layers = std::cmp::min(first.missing_roof_layers, second.missing_roof_layers);
    out.skip_ovalisation = false;
    if first.target_height > second.target_height {
        out.target_height = first.target_height;
        out.target_position = first.target_position;
    } else {
        out.target_height = second.target_height;
        out.target_position = second.target_position;
    }
    out.effective_radius_height = std::cmp::max(first.effective_radius_height, second.effective_radius_height);
    out.distance_to_top = std::cmp::max(first.distance_to_top, second.distance_to_top);

    out.to_buildplate = first.to_buildplate && second.to_buildplate;
    out.to_model_gracious = first.to_model_gracious && second.to_model_gracious; // valid as we do not merge non-gracious with gracious

    out.elephant_foot_increases = 0.;
    if config.bp_radius_increase_per_layer > 0. {
        let foot_increase_radius: Coord = (std::cmp::max(support_element_collision_radius(config, second), support_element_collision_radius(config, first))
            - support_element_collision_radius(config, &out))
        .abs();
        // elephant_foot_increases has to be recalculated, as when a smaller tree with a larger elephant_foot_increases merge with a larger branch
        // the elephant_foot_increases may have to be lower as otherwise the radius suddenly increases. This results often in a non integer value.
        out.elephant_foot_increases = foot_increase_radius as f64 / (config.bp_radius_increase_per_layer - config.branch_radius_increase_per_layer);
    }

    // set last settings to the best out of both parents. If this is wrong, it will only cause a small performance penalty instead of weird behavior.
    out.last_area_increase = AreaIncreaseSettings::new(
        // std::min of the enumerators: Slow < FastSafe < Fast
        if (second.last_area_increase.type_ as i32) < (first.last_area_increase.type_ as i32) {
            second.last_area_increase.type_
        } else {
            first.last_area_increase.type_
        },
        std::cmp::min(first.last_area_increase.increase_speed, second.last_area_increase.increase_speed),
        first.last_area_increase.increase_radius || second.last_area_increase.increase_radius,
        first.last_area_increase.no_error || second.last_area_increase.no_error,
        first.last_area_increase.use_min_distance && second.last_area_increase.use_min_distance,
        first.last_area_increase.move_ || second.last_area_increase.move_,
    );

    out
}

// `Eigen::AlignedBox::intersects()`
#[inline]
fn aligned_box_intersects(a: &(Point, Point), b: &(Point, Point)) -> bool {
    a.0.x <= b.1.x && a.0.y <= b.1.y && b.0.x <= a.1.x && b.0.y <= a.1.y
}

/// TreeSupport3D.cpp:2064-2247 `static bool merge_influence_areas_two_elements(const TreeModelVolumes &volumes, const
/// TreeSupportSettings &config, const LayerIndex layer_idx, SupportElementMerging &dst, SupportElementMerging &src)`
pub fn merge_influence_areas_two_elements(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    layer_idx: LayerIndex,
    dst: &mut SupportElementMerging,
    src: &mut SupportElementMerging,
) -> bool {
    // Don't merge gracious with a non gracious area as bad placement could negatively impact reliability of the whole subtree.
    let merging_gracious_and_non_gracious = dst.state.to_model_gracious != src.state.to_model_gracious;
    // Could cause some issues with the increase of one area, as it is assumed that if the smaller is increased
    // by the delta to the larger it is engulfed by it already. But because a different collision
    // may be removed from the in draw_area() generated circles, this assumption could be wrong.
    let merging_min_and_regular_xy = dst.state.use_min_xy_dist != src.state.use_min_xy_dist;
    let merging_different_min_xy = dst.state.use_min_xy_dist && src.state.use_min_xy_dist && dst.state.xy_min_distance != src.state.xy_min_distance;

    if merging_gracious_and_non_gracious || merging_min_and_regular_xy || merging_different_min_xy {
        return false;
    }

    let dst_radius_bigger = support_element_collision_radius(config, &dst.state) > support_element_collision_radius(config, &src.state);
    let intersect: Polygons;
    let new_state: SupportElementState;
    let influence_areas: Polygons;
    let mut to_model_areas: Polygons = Vec::new();
    let merging_to_bp: bool;
    {
        let (smaller_rad, bigger_rad): (&SupportElementMerging, &SupportElementMerging) = if dst_radius_bigger { (&*src, &*dst) } else { (&*dst, &*src) };
        let real_radius_delta: Coord = (support_element_radius(config, &bigger_rad.state) - support_element_radius(config, &smaller_rad.state)).abs();
        // Cap the lateral merge reach. The merge test below inflates the thin branch by the radius
        // delta, so a large radius difference lets a thick branch swallow a far-away thin one in a
        // single layer - the source of the aggressive sideways merge at large diameters.
        // (TreeSupport3D.cpp:2087-2101 has the whole of the reasoning.)
        const OVALISATION_SLOW_RATIO: f64 = 0.75; // mirrors draw_area()
        const MAX_MERGE_DELAY_LAYERS: Coord = 30;
        // support_tree_angle is only clamped below 90 deg, not to the 60 deg the setting exposes, so a
        // config that bypasses that bound (hand edited project, command line) can drive
        // maximum_move_distance close to the coord_t limit. Cap it before doubling to avoid overflow.
        // scaled<coord_t>(10.)
        let max_sane_move_distance: Coord = (10. / SCALING_FACTOR) as Coord;
        let move_budget: Coord = std::cmp::min(config.maximum_move_distance, max_sane_move_distance);
        let mut merge_reach: Coord =
            std::cmp::min(2 * move_budget, (OVALISATION_SLOW_RATIO * support_element_radius(config, &bigger_rad.state) as f64) as Coord);
        merge_reach = std::cmp::max(merge_reach, config.support_line_width / 2);
        merge_reach = std::cmp::max(merge_reach, real_radius_delta - 2 * MAX_MERGE_DELAY_LAYERS * move_budget);
        // Never exceed the geometric "engulfed by the radius delta" limit this whole test is built on.
        merge_reach = std::cmp::min(merge_reach, real_radius_delta);
        {
            // Testing intersection of bounding boxes.
            // Expand the smaller radius branch bounding box to match the lambda intersect_small_with_bigger() below.
            // Because the lambda intersect_small_with_bigger() applies a rounded offset, a snug offset of the bounding box
            // is sufficient. On the other side, if a mitered offset was used by the lambda,
            // the bounding box expansion would have to account for the mitered extension of the sharp corners.
            let mut smaller_bbox: (Point, Point) = *smaller_rad.bbox();
            smaller_bbox.0 = smaller_bbox.0 - Point::new(merge_reach, merge_reach);
            smaller_bbox.1 = smaller_bbox.1 + Point::new(merge_reach, merge_reach);
            if !aligned_box_intersects(&smaller_bbox, bigger_rad.bbox()) {
                return false;
            }
        }

        // Accumulator of a radius increase of a "to model" branch by merging in a "to build plate" branch.
        let mut increased_to_model_radius: Coord = 0;
        merging_to_bp = dst.state.to_buildplate && src.state.to_buildplate;
        if !merging_to_bp {
            // Get the real radius increase as the user does not care for the collision model.
            if dst.state.to_buildplate != src.state.to_buildplate {
                // Merging a "to build plate" branch with a "to model" branch.
                // Don't allow merging a thick "to build plate" branch into a thinner "to model" branch.
                let rdst: Coord = support_element_radius(config, &dst.state);
                let rsrc: Coord = support_element_radius(config, &src.state);
                if dst.state.to_buildplate {
                    if rsrc < rdst {
                        increased_to_model_radius = src.state.increased_to_model_radius + rdst - rsrc;
                    }
                } else if rsrc > rdst {
                    increased_to_model_radius = dst.state.increased_to_model_radius + rsrc - rdst;
                }
                if increased_to_model_radius > config.max_to_model_radius_increase {
                    return false;
                }
            }
            // if a merge could place a stable branch on unstable ground, would be increasing the radius further
            // than allowed to when merging to model and to_bp trees or would merge to model before it is known
            // they will even been drawn the merge is skipped
            if !dst.state.supports_roof
                && !src.state.supports_roof
                && (std::cmp::max(src.state.distance_to_top, dst.state.distance_to_top) as usize) < config.min_dtt_to_model
            {
                return false;
            }
        }

        // Area of the bigger radius is used to ensure correct placement regarding the relevant avoidance,
        // so if that would change an invalid area may be created.
        if !bigger_rad.state.can_use_safe_radius && smaller_rad.state.can_use_safe_radius {
            return false;
        }

        // the bigger radius is used to verify that the area is still valid after the increase with the delta.
        // If there were a point where the big influence area could be valid with can_use_safe_radius
        // the element would already be can_use_safe_radius.
        // the smaller radius, which gets increased by delta may reach into the area where use_min_xy_dist is no longer required.
        let use_min_radius = bigger_rad.state.use_min_xy_dist && smaller_rad.state.use_min_xy_dist;

        // The idea is that the influence area with the smaller collision radius is increased by the radius difference.
        // If this area has any intersections with the influence area of the larger collision radius, a branch (of the larger collision radius) placed in this intersection, has already engulfed the branch of the smaller collision radius.
        // Because of this a merge may happen even if the influence areas (that represent possible center points of branches) do not intersect yet.
        // Remember that collision radius <= real radius as otherwise this assumption would be false.
        let smaller_collision_radius: Coord = support_element_collision_radius(config, &smaller_rad.state);
        let collision = volumes.get_collision(
            smaller_collision_radius + if use_min_radius { std::cmp::max(0, smaller_rad.state.xy_min_distance - config.xy_min_distance) } else { 0 },
            layer_idx - 1,
            use_min_radius,
        );
        let intersect_small_with_bigger = |small: &Polygons, bigger: &Polygons| -> Polygons {
            cu::intersection(
                &safe_offset_inc(
                    small,
                    merge_reach,
                    &collision,
                    // -3 avoids possible rounding errors
                    2 * (config.xy_distance + smaller_collision_radius - 3),
                    0,
                    0,
                ),
                bigger,
                cu::ApplySafetyOffset::No,
            )
        };
        intersect = intersect_small_with_bigger(
            if merging_to_bp { &smaller_rad.areas.to_bp_areas } else { &smaller_rad.areas.to_model_areas },
            if merging_to_bp { &bigger_rad.areas.to_bp_areas } else { &bigger_rad.areas.to_model_areas },
        );

        // dont use empty as a line is not empty, but for this use-case it very well may be (and would be one layer down as union does not keep lines)
        // check if the overlap is large enough (Small ares tend to attract rounding errors in clipper).
        if area(&intersect) <= tiny_area_threshold() {
            return false;
        }

        // While 0.025 was guessed as enough, i did not have reason to change it.
        if area(&cu::offset(&intersect, (-0.025 / SCALING_FACTOR) as f32, crate::clipper::JoinType::jtMiter, 1.2)) <= tiny_area_threshold() {
            return false;
        }

        // Do the actual merge now that the branches are confirmed to be able to intersect.
        // calculate which point is closest to the point of the last merge (or tip center if no merge above it has happened)
        // used at the end to estimate where to best place the branch on the bottom most layer
        // could be replaced with a random point inside the new area
        let new_pos: Point = move_inside_if_outside(&intersect, dst.state.next_position, 0, i64::MAX);

        let mut state = merge_support_element_states(&dst.state, &src.state, &new_pos, layer_idx - 1, config);
        state.increased_to_model_radius = if increased_to_model_radius == 0 {
            // increased_to_model_radius was not set yet. Propagate maximum.
            std::cmp::max(dst.state.increased_to_model_radius, src.state.increased_to_model_radius)
        } else {
            increased_to_model_radius
        };
        new_state = state;

        // Rather unioning with "intersect" due to some rounding errors.
        influence_areas =
            safe_union(&intersect_small_with_bigger(&smaller_rad.areas.influence_areas, &bigger_rad.areas.influence_areas), &intersect);

        if merging_to_bp && config.support_rests_on_model {
            to_model_areas = if new_state.to_model_gracious {
                // Rather unioning with "intersect" due to some rounding errors.
                safe_union(&intersect_small_with_bigger(&smaller_rad.areas.to_model_areas, &bigger_rad.areas.to_model_areas), &intersect)
            } else {
                influence_areas.clone()
            };
        }
    }

    dst.parents.extend_from_slice(&src.parents);
    dst.state = new_state;
    dst.areas.influence_areas = influence_areas;
    dst.areas.to_bp_areas.clear();
    dst.areas.to_model_areas.clear();
    if merging_to_bp {
        dst.areas.to_bp_areas = intersect;
        if config.support_rests_on_model {
            dst.areas.to_model_areas = to_model_areas;
        }
    } else {
        dst.areas.to_model_areas = intersect;
    }
    // Update the bounding box.
    let mut bbox = crate::geometry::get_extents_polygons(&dst.areas.influence_areas);
    bbox.merge(&crate::geometry::get_extents_polygons(&dst.areas.to_bp_areas));
    bbox.merge(&crate::geometry::get_extents_polygons(&dst.areas.to_model_areas));
    dst.set_bbox(&bbox);
    // Clear the source data.
    src.areas.clear();
    src.parents.clear();
    true
}

// Two elements of the one vector, as the pin's two pointers into it.
fn two_elements(v: &mut [SupportElementMerging], a: usize, b: usize) -> (&mut SupportElementMerging, &mut SupportElementMerging) {
    debug_assert!(a != b);
    if a < b {
        let (lo, hi) = v.split_at_mut(b);
        (&mut lo[a], &mut hi[0])
    } else {
        let (lo, hi) = v.split_at_mut(a);
        (&mut hi[0], &mut lo[b])
    }
}

/// TreeSupport3D.cpp:2266-2287 `static SupportElementMerging* merge_influence_areas_leaves(const TreeModelVolumes
/// &volumes, const TreeSupportSettings &config, const LayerIndex layer_idx, SupportElementMerging * const dst_begin,
/// SupportElementMerging *dst_end)`
///
/// The pointers are indices into `influence_areas`.
pub fn merge_influence_areas_leaves(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    layer_idx: LayerIndex,
    influence_areas: &mut [SupportElementMerging],
    dst_begin: usize,
    mut dst_end: usize,
) -> usize {
    // Merging at the lowest level of the AABB tree. Checking one against each other, O(n^2).
    debug_assert!(dst_begin < dst_end);
    let mut i = dst_begin;
    while i + 1 < dst_end {
        let mut merged = false;
        let mut j = i + 1;
        while j != dst_end {
            let (ei, ej) = two_elements(influence_areas, i, j);
            if merge_influence_areas_two_elements(volumes, config, layer_idx, ei, ej) {
                // i was merged with j, j is empty.
                dst_end -= 1;
                if j != dst_end {
                    influence_areas[j] = influence_areas[dst_end].take_moved();
                }
                merged = true;
                break;
            } else {
                j += 1;
            }
        }
        if !merged {
            // not merged
            i += 1;
        }
    }
    dst_end
}

/// TreeSupport3D.cpp:2289-2330 `static SupportElementMerging* merge_influence_areas_two_sets(const TreeModelVolumes
/// &volumes, const TreeSupportSettings &config, const LayerIndex layer_idx, SupportElementMerging * const dst_begin,
/// SupportElementMerging *dst_end, SupportElementMerging *src_begin, SupportElementMerging * const src_end)`
///
/// The pointers are indices into `influence_areas`.
#[allow(clippy::too_many_arguments)]
pub fn merge_influence_areas_two_sets(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    layer_idx: LayerIndex,
    influence_areas: &mut [SupportElementMerging],
    dst_begin: usize,
    mut dst_end: usize,
    mut src_begin: usize,
    src_end: usize,
) -> usize {
    // Merging src into dst.
    // Areas of src should not overlap with areas of another elements of src.
    // Areas of dst should not overlap with areas of another elements of dst.
    // The memory from dst_begin to src_end is reserved for the merging operation,
    // src follows dst.
    debug_assert!(src_begin < src_end);
    debug_assert!(dst_begin < dst_end);
    debug_assert!(dst_end <= src_begin);
    let mut src = src_begin;
    while src != src_end {
        let mut dst = dst_begin;
        // `SupportElementMerging *merged = nullptr;`
        let mut merged: usize = usize::MAX;
        while dst != dst_end {
            let (ed, es) = two_elements(influence_areas, dst, src);
            if merge_influence_areas_two_elements(volumes, config, layer_idx, ed, es) {
                merged = dst;
                dst += 1;
                if src != src_begin {
                    // Compactify src.
                    influence_areas[src] = influence_areas[src_begin].take_moved();
                }
                src_begin += 1;
                break;
            }
            dst += 1;
        }
        while dst != dst_end {
            let (em, ed) = two_elements(influence_areas, merged, dst);
            if merge_influence_areas_two_elements(volumes, config, layer_idx, em, ed) {
                // Compactify dst.
                dst_end -= 1;
                if dst != dst_end {
                    influence_areas[dst] = influence_areas[dst_end].take_moved();
                }
            } else {
                dst += 1;
            }
        }
        src += 1;
    }
    // Compactify src elements that were not merged with dst to the end of dst.
    debug_assert!(dst_end <= src_begin);
    if dst_end == src_begin {
        dst_end = src_end;
    } else {
        while src_begin != src_end {
            influence_areas[dst_end] = influence_areas[src_begin].take_moved();
            dst_end += 1;
            src_begin += 1;
        }
    }

    dst_end
}

/// What `tbb::this_task_arena::max_concurrency()` gives merge_influence_areas() (TreeSupport3D.cpp:2376): the number
/// of threads of the machine, by which the pin chooses the size of its first buckets, and with it the order in
/// which the areas of a layer are merged. Zero (the initial value) stands for the number of threads of this
/// machine; a test sets the number of the machine its vectors were made on.
pub static MERGE_INFLUENCE_AREAS_NUM_THREADS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

fn merge_influence_areas_num_threads() -> usize {
    match MERGE_INFLUENCE_AREAS_NUM_THREADS.load(std::sync::atomic::Ordering::Relaxed) {
        0 => std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1),
        n => n,
    }
}

/// TreeSupport3D.cpp:2346-2433 `static void merge_influence_areas(const TreeModelVolumes &volumes, const
/// TreeSupportSettings &config, const LayerIndex layer_idx, std::vector<SupportElementMerging> &influence_areas,
/// std::function<void()> throw_on_cancel)`
///
/// Merges Influence Areas at one layer if possible. The vector keeps its size: the elements that were merged into
/// others stay behind it with no areas.
pub fn merge_influence_areas(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    layer_idx: LayerIndex,
    influence_areas: &mut Vec<SupportElementMerging>,
    throw_on_cancel: ThrowOnCancel,
) {
    let input_size: usize = influence_areas.len();
    if input_size == 0 {
        return;
    }

    // Merging by divide & conquer.
    // The majority of time is consumed by Clipper polygon operations, intersection is accelerated by bounding boxes.
    // Sorting input into an AABB tree helps to perform most of the intersections at first iterations,
    // thus reducing computation when merging larger subtrees.
    // The actual merge logic is found in merge_influence_areas_two_sets.

    // Build an AABB tree over the influence areas.
    //FIXME A full tree does not need to be built, the lowest level branches will be always bucketed.
    // However the additional time consumed is negligible.
    let mut tree = crate::aabb_tree_lines::aabb_tree_indirect_2d::Tree::new();
    // Sort influence_areas in place.
    tree.build_modify_input(influence_areas.as_mut_slice());

    throw_on_cancel();

    // Prepare the initial buckets as ranges of influence areas. The initial buckets contain power of 2 influence areas to follow
    // the branching of the AABB tree.
    // Vectors of ranges of influence areas, following the branching of the AABB tree:
    let mut buckets: Vec<(usize, usize)> = Vec::new();
    // Initial number of buckets for 1st round of merging.
    let num_buckets_initial: usize;
    {
        // How many buckets per first merge iteration?
        let num_threads: usize = merge_influence_areas_num_threads();
        // 4 buckets per thread if possible,
        let num_buckets_min: usize = (input_size + 2) / 4;
        // 2 buckets per thread otherwise.
        let num_buckets_max: usize = input_size / 2;
        num_buckets_initial = if num_buckets_min >= num_threads { num_buckets_min } else { num_buckets_max };
        let bucket_size: usize = if num_buckets_min >= num_threads { 4 } else { 2 };
        // Fill in the buckets.
        let mut it: usize = 0;
        // Reserve one more bucket to keep a single influence area which will not be merged in the first iteration.
        buckets.reserve(num_buckets_initial + 1);
        for _ in 0..num_buckets_initial {
            buckets.push((it, it + bucket_size));
            it += bucket_size;
        }
        let it_end: usize = influence_areas.len();
        // `buckets.back()` of no bucket (one influence area) is not defined; the pin gets there with one area
        // only from a caller that does not call it then (TreeSupport3D.cpp:2507).
        if !buckets.is_empty() && buckets[buckets.len() - 1].1 >= it_end {
            // Last bucket is less than size 4, but bigger than size 1.
            let last = buckets.len() - 1;
            buckets[last].1 = std::cmp::min(buckets[last].1, it_end);
        } else {
            // Last bucket is size 1, it will not be merged in the first iteration.
            debug_assert!(it + 1 == it_end);
            buckets.push((it, it_end));
        }
    }

    // 1st merge iteration, merge one with each other.
    for idx in 0..num_buckets_initial {
        // Merge bucket_count adjacent to each other, merging uneven bucket numbers into even buckets
        buckets[idx].1 = merge_influence_areas_leaves(volumes, config, layer_idx, influence_areas.as_mut_slice(), buckets[idx].0, buckets[idx].1);
        throw_on_cancel();
    }

    // Further merge iterations, merging one AABB subtree with another one, hopefully minimizing intersections between the elements
    // of each of the subtree.
    while buckets.len() > 1 {
        for idx in 0..buckets.len() / 2 {
            let bucket_pair_idx: usize = idx * 2;
            // Merge bucket_count adjacent to each other, merging uneven bucket numbers into even buckets
            buckets[bucket_pair_idx].1 = merge_influence_areas_two_sets(
                volumes,
                config,
                layer_idx,
                influence_areas.as_mut_slice(),
                buckets[bucket_pair_idx].0,
                buckets[bucket_pair_idx].1,
                buckets[bucket_pair_idx + 1].0,
                buckets[bucket_pair_idx + 1].1,
            );
            throw_on_cancel();
        }
        // Remove odd buckets, which were merged into even buckets.
        let new_size: usize = (buckets.len() + 1) / 2;
        for i in 1..new_size {
            buckets[i] = buckets[i * 2];
        }
        buckets.truncate(new_size);
    }
}

/// TreeSupport3D.cpp:2441-2536 `void create_layer_pathing(const TreeModelVolumes &volumes, const TreeSupportSettings
/// &config, std::vector<SupportElements> &move_bounds, std::function<void()> throw_on_cancel)`
///
/// Propagates influence downwards, and merges overlapping ones.
pub fn create_layer_pathing(volumes: &TreeModelVolumes, config: &TreeSupportSettings, move_bounds: &mut [SupportElements], throw_on_cancel: ThrowOnCancel) {
    let mut last_merge_layer_idx: LayerIndex = move_bounds.len() as LayerIndex;
    let mut new_element = false;

    // Ensures at least one merge operation per 3mm height, 50 layers, 1 mm movement of slow speed or 5mm movement of fast speed (whatever is lowest). Values were guessed.
    let max_merge_every_x_layers: usize = std::cmp::min(
        std::cmp::min(5000 / std::cmp::max(config.maximum_move_distance, 100), 1000 / std::cmp::max(config.maximum_move_distance_slow, 20)),
        3000 / config.layer_height,
    ) as usize;
    let mut merge_every_x_layers: usize = 1;
    // Calculate the influence areas for each layer below (Top down)
    // This is done by first increasing the influence area by the allowed movement distance, and merging them with other influence areas if possible
    let mut layer_idx: i32 = move_bounds.len() as i32 - 1;
    while layer_idx > 0 {
        if !move_bounds[layer_idx as usize].is_empty() {
            let (lower, upper) = move_bounds.split_at_mut(layer_idx as usize);
            let prev_layer: &mut SupportElements = &mut upper[0];
            // merging is expensive and only parallelized to a max speedup of 2. As such it may be useful in some cases to only merge every few layers to improve performance.
            let had_new_element = new_element;
            let merge_this_layer = had_new_element || (last_merge_layer_idx - layer_idx) as usize >= merge_every_x_layers;
            if had_new_element {
                merge_every_x_layers = 1;
            }

            // ### Increase the influence areas by the allowed movement distance
            let mut influence_areas: Vec<SupportElementMerging> = Vec::with_capacity(prev_layer.len());
            for element_idx in 0..prev_layer.len() as i32 {
                let el: &SupportElement = &prev_layer[element_idx as usize];
                debug_assert!(!el.influence_area.is_empty());
                let parents: ParentIndices = vec![element_idx];
                influence_areas.push(SupportElementMerging::new(el.state.clone(), parents));
            }
            increase_areas_one_layer(volumes, config, &mut influence_areas, layer_idx, prev_layer, merge_this_layer, throw_on_cancel);

            // Place already fully constructed elements to the output, remove them from influence_areas.
            let this_layer: &mut SupportElements = &mut lower[layer_idx as usize - 1];
            // std::remove_if(): the predicate is asked once for every element, in their order.
            let mut kept: Vec<SupportElementMerging> = Vec::with_capacity(influence_areas.len());
            for mut elem in influence_areas {
                if elem.areas.influence_areas.is_empty() {
                    // This area was removed completely due to collisions.
                    continue;
                }
                if elem.areas.to_bp_areas.is_empty() && elem.areas.to_model_areas.is_empty() {
                    if area(&elem.areas.influence_areas) < tiny_area_threshold() {
                        tree_supports_show_error("Insert error of area after bypassing merge.\n", true);
                    }
                    // Move the area to output.
                    this_layer.push(SupportElement::with_parents(
                        elem.state.clone(),
                        std::mem::take(&mut elem.parents),
                        std::mem::take(&mut elem.areas.influence_areas),
                    ));
                    continue;
                }
                // Keep the area.
                kept.push(elem);
            }
            let mut influence_areas = kept;

            new_element = !this_layer.is_empty();
            if merge_this_layer {
                let mut reduced_by_merging = false;
                let count_before_merge: usize = influence_areas.len();
                if count_before_merge > 1 {
                    // ### Calculate which influence areas overlap, and merge them into a new influence area (simplified: an intersection of influence areas that have such an intersection)
                    merge_influence_areas(volumes, config, layer_idx, &mut influence_areas, throw_on_cancel);
                    reduced_by_merging = count_before_merge > influence_areas.len();
                }
                last_merge_layer_idx = layer_idx;
                if !reduced_by_merging && !had_new_element {
                    merge_every_x_layers = std::cmp::min(max_merge_every_x_layers, merge_every_x_layers + 1);
                }
            }

            // Save calculated elements to output, and allocate Polygons on heap, as they will not be changed again.
            for elem in &mut influence_areas {
                if !elem.areas.influence_areas.is_empty() {
                    let new_area: Polygons = safe_union(&elem.areas.influence_areas, &Vec::new());
                    if area(&new_area) < tiny_area_threshold() {
                        tree_supports_show_error("Insert error of area after merge.\n", true);
                    }
                    this_layer.push(SupportElement::with_parents(elem.state.clone(), std::mem::take(&mut elem.parents), new_area));
                }
            }

            throw_on_cancel();
        }
        layer_idx -= 1;
    }
}

// =============================================================================
// The port of the pin: TreeSupport3D.cpp, the nodes (set_points_on_areas() to create_nodes_from_area())
// =============================================================================

/// TreeSupport3D.cpp:2544-2572 `static void set_points_on_areas(const SupportElement &elem, SupportElements
/// *layer_above)`
///
/// Sets the result_on_layer for all parents based on the SupportElement supplied.
pub fn set_points_on_areas(elem: &SupportElement, layer_above: Option<&mut SupportElements>) {
    debug_assert!(!elem.state.deleted);
    debug_assert!(layer_above.is_some() || elem.parents.is_empty());

    // Based on the branch center point of the current layer, the point on the next (further up) layer is calculated.
    if !elem.state.result_on_layer_is_set() {
        tree_supports_show_error("Uninitialized support element. A branch may be missing.\n", true);
        return;
    }

    if let Some(layer_above) = layer_above {
        for &next_elem_idx in &elem.parents {
            debug_assert!(next_elem_idx >= 0);
            let next_elem: &mut SupportElement = &mut layer_above[next_elem_idx as usize];
            debug_assert!(!next_elem.state.deleted);
            // if the value was set somewhere else it it kept. This happens when a branch tries not to move after being unable to create a roof.
            if !next_elem.state.result_on_layer_is_set() {
                // Move inside has edgecases (see tests) so DONT use Polygons.inside to confirm correct move, Error with distance 0 is <= 1
                // it is not required to check if how far this move moved a point as is can be larger than maximum_movement_distance.
                // While this seems like a problem it may for example occur after merges.
                next_elem.state.result_on_layer = move_inside_if_outside(&next_elem.influence_area, elem.state.result_on_layer, 0, i64::MAX);
                // do not call recursive because then amount of layers would be restricted by the stack size
            }
            // Mark the parent element as accessed from a valid child element.
            next_elem.state.marked = true;
        }
    }
}

/// TreeSupport3D.cpp:2574-2579 `static void set_to_model_contact_simple(SupportElement &elem)`
pub fn set_to_model_contact_simple(elem: &mut SupportElement) {
    let best: Point = move_inside_if_outside(&elem.influence_area, elem.state.next_position, 0, i64::MAX);
    elem.state.result_on_layer = best;
}

/// TreeSupport3D.cpp:2588-2633 `static void set_to_model_contact_to_model_gracious(const TreeModelVolumes &volumes,
/// const TreeSupportSettings &config, std::vector<SupportElements> &move_bounds, SupportElement &first_elem,
/// std::function<void()> throw_on_cancel)`
///
/// Get the best point to connect to the model and set the result_on_layer of the relevant SupportElement
/// accordingly. `first_elem` is the element `first_elem.1` of the layer `first_elem.0` of `move_bounds`; the
/// pointers of the pin are such pairs.
pub fn set_to_model_contact_to_model_gracious(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    move_bounds: &mut [SupportElements],
    first_elem: (usize, usize),
    throw_on_cancel: ThrowOnCancel,
) {
    // `SupportElement *last_successfull_layer = nullptr;`
    let mut last_successfull_layer: Option<(usize, usize)> = None;

    // check for every layer upwards, up to the point where this influence area was created (either by initial insert or merge) if the branch could be placed on it, and highest up layer index.
    {
        let mut elem: (usize, usize) = first_elem;
        let mut layer_check: LayerIndex = move_bounds[elem.0][elem.1].state.layer_idx;
        loop {
            let e: &SupportElement = &move_bounds[elem.0][elem.1];
            if cu::intersection(
                &e.influence_area,
                &volumes.get_placeable_areas(support_element_collision_radius(config, &e.state), layer_check, throw_on_cancel),
                cu::ApplySafetyOffset::No,
            )
            .is_empty()
            {
                break;
            }
            debug_assert!(e.state.layer_idx == layer_check);
            debug_assert!(!e.state.deleted);
            debug_assert!(e.state.to_model_gracious);
            last_successfull_layer = Some(elem);
            if e.parents.len() != 1 {
                // Reached merge point.
                break;
            }
            // elem = &move_bounds[++ layer_check][elem->parents.front()]
            layer_check += 1;
            elem = (layer_check as usize, e.parents[0] as usize);
        }
    }

    // Could not find valid placement, even though it should exist => error handling
    match last_successfull_layer {
        None => {
            tree_supports_show_error("Could not fine valid placement on model! Just placing it down anyway. Could cause floating branches.", true);
            let first = &mut move_bounds[first_elem.0][first_elem.1];
            first.state.to_model_gracious = false;
            set_to_model_contact_simple(first);
        }
        Some(last_successfull_layer) => {
            // Found a gracious area above first_elem. Remove all below last_successfull_layer.
            {
                let mut parent_layer_idx: LayerIndex = move_bounds[first_elem.0][first_elem.1].state.layer_idx;
                let mut elem: (usize, usize) = first_elem;
                while elem != last_successfull_layer {
                    let e: &mut SupportElement = &mut move_bounds[elem.0][elem.1];
                    debug_assert!(!e.state.deleted);
                    e.state.deleted = true;
                    // elem = &move_bounds[++ parent_layer_idx][elem->parents.front()]
                    parent_layer_idx += 1;
                    elem = (parent_layer_idx as usize, e.parents[0] as usize);
                }
            }
            // Guess a point inside the influence area, in which the branch will be placed in.
            let last: &mut SupportElement = &mut move_bounds[last_successfull_layer.0][last_successfull_layer.1];
            let best: Point = move_inside_if_outside(&last.influence_area, last.state.next_position, 0, i64::MAX);
            last.state.result_on_layer = best;
        }
    }
}

/// TreeSupport3D.cpp:2636-2675 `static void remove_deleted_elements(std::vector<SupportElements> &move_bounds)`
// Remove elements marked as "deleted", update indices to parents.
pub fn remove_deleted_elements(move_bounds: &mut [SupportElements]) {
    let mut map_parents: Vec<i32> = Vec::new();
    let mut map_current: Vec<i32> = Vec::new();
    let mut layer_idx: LayerIndex = move_bounds.len() as LayerIndex - 1;
    while layer_idx >= 0 {
        let layer: &mut SupportElements = &mut move_bounds[layer_idx as usize];
        map_current.clear();
        let mut i: i32 = 0;
        while i < layer.len() as i32 {
            if layer[i as usize].state.deleted {
                if map_current.is_empty() {
                    // Initialize with identity map.
                    map_current = (0..layer.len() as i32).collect();
                }
                // Delete all "deleted" elements from the end of the layer vector.
                while i < layer.len() as i32 && layer[layer.len() - 1].state.deleted {
                    layer.pop();
                    // Mark as deleted in the map.
                    map_current[layer.len()] = -1;
                }
                debug_assert!(i as usize == layer.len() || (i as usize) + 1 < layer.len());
                if i + 1 < layer.len() as i32 {
                    // element = std::move(layer.back()); layer.pop_back();
                    let back = layer.pop().unwrap();
                    layer[i as usize] = back;
                    // Mark the current element as deleted.
                    map_current[i as usize] = -1;
                    // Mark the moved element as moved to index i.
                    map_current[layer.len()] = i;
                }
            } else {
                // Current element is not deleted. Update its parent indices.
                if !map_parents.is_empty() {
                    for parent_idx in layer[i as usize].parents.iter_mut() {
                        *parent_idx = map_parents[*parent_idx as usize];
                    }
                }
                i += 1;
            }
        }
        std::mem::swap(&mut map_current, &mut map_parents);
        layer_idx -= 1;
    }
}

/// TreeSupport3D.cpp:2683-2798 `void create_nodes_from_area(const TreeModelVolumes &volumes, const
/// TreeSupportSettings &config, std::vector<SupportElements> &move_bounds, std::function<void()> throw_on_cancel)`
///
/// Set the result_on_layer point for all influence areas
pub fn create_nodes_from_area(volumes: &TreeModelVolumes, config: &TreeSupportSettings, move_bounds: &mut [SupportElements], throw_on_cancel: ThrowOnCancel) {
    // Initialize points on layer 0, with a "random" point in the influence area.
    // Point is chosen based on an inaccurate estimate where the branches will split into two, but every point inside the influence area would produce a valid result.
    {
        let (front, rest) = move_bounds.split_at_mut(1);
        let mut layer_above: Option<&mut SupportElements> = rest.first_mut();
        if let Some(layer_above) = layer_above.as_deref_mut() {
            for elem in layer_above.iter_mut() {
                elem.state.marked = false;
            }
        }
        for init in front[0].iter_mut() {
            init.state.result_on_layer = move_inside_if_outside(&init.influence_area, init.state.next_position, 0, i64::MAX);
            // Also set the parent nodes, as these will be required for the first iteration of the loop below and mark the parent nodes.
            set_points_on_areas(init, layer_above.as_deref_mut());
        }
    }

    throw_on_cancel();

    for layer_idx in 1..move_bounds.len() as LayerIndex {
        let has_layer_above = layer_idx + 1 < move_bounds.len() as LayerIndex;
        if has_layer_above {
            for elem in move_bounds[layer_idx as usize + 1].iter_mut() {
                elem.state.marked = false;
            }
        }
        for elem_idx in 0..move_bounds[layer_idx as usize].len() {
            {
                let elem: &mut SupportElement = &mut move_bounds[layer_idx as usize][elem_idx];
                debug_assert!(!elem.state.deleted);
                debug_assert!(elem.state.layer_idx == layer_idx);
                // check if the resulting center point is not yet set
                if !elem.state.result_on_layer_is_set() {
                    if elem.state.to_buildplate || ((elem.state.distance_to_top as usize) < config.min_dtt_to_model && !elem.state.supports_roof) {
                        if elem.state.to_buildplate {
                            tree_supports_show_error("Uninitialized support element! A branch could be missing or exist partially.", true);
                        }
                        // we dont need to remove yet the parents as they will have a lower dtt and also no result_on_layer set
                        elem.state.deleted = true;
                    } else {
                        // set the point where the branch will be placed on the model
                        if elem.state.to_model_gracious {
                            set_to_model_contact_to_model_gracious(volumes, config, move_bounds, (layer_idx as usize, elem_idx), throw_on_cancel);
                        } else {
                            set_to_model_contact_simple(elem);
                        }
                    }
                }
            }
            let (lower, upper) = move_bounds.split_at_mut(layer_idx as usize + 1);
            let elem: &mut SupportElement = &mut lower[layer_idx as usize][elem_idx];
            let mut layer_above: Option<&mut SupportElements> = if has_layer_above { upper.first_mut() } else { None };
            if !elem.state.deleted && !elem.state.marked && elem.state.target_height == layer_idx {
                // Just a tip surface with no supporting element.
                elem.state.deleted = true;
            }
            if elem.state.deleted {
                for &parent_idx in &elem.parents {
                    // When the roof was not able to generate downwards enough, the top elements may have not moved, and have result_on_layer already set.
                    // As this branch needs to be removed => all parents result_on_layer have to be invalidated.
                    layer_above.as_deref_mut().expect("layer_above")[parent_idx as usize].state.result_on_layer_reset();
                }
            }
            if !elem.state.deleted {
                // Element is valid now setting points in the layer above and mark the parent nodes.
                set_points_on_areas(elem, layer_above);
            }
        }
        throw_on_cancel();
    }

    remove_deleted_elements(move_bounds);
}

// =============================================================================
// The port of the pin: TreeSupport3D.cpp, the areas of the branches (DrawArea to draw_areas())
// =============================================================================

use crate::support::support_common::cu_expolygon;
use crate::support::support_layer::SupporLayerType;
use crate::support::tree_support_common::{layer_allocate, InterfacePreference};

/// TreeSupport3D.cpp:2802-2810 `struct DrawArea`
///
/// For producing circular / elliptical areas from SupportElements (one DrawArea per one SupportElement)
/// and for smoothing those areas along the tree branches. The pointers of the pin are the layer of the element
/// in `move_bounds` and its index in that layer.
#[derive(Debug, Clone)]
pub struct DrawArea {
    /// Element to be processed.
    pub element: (usize, usize),
    /// Element below, if there is such an element. nullptr if element is a root of a tree.
    pub child_element: Option<(usize, usize)>,
    /// Polygons to be extruded for this element.
    pub polygons: Polygons,
}

// `Point(double x, double y)` (Point.hpp:179): `lrint` of the coordinates.
#[inline]
fn point_lrint(x: f64, y: f64) -> Point {
    Point::new(x.round_ties_even() as Coord, y.round_ties_even() as Coord)
}

/// TreeSupport3D.cpp:2819-2957 `static void generate_branch_areas(const TreeModelVolumes &volumes, const
/// TreeSupportSettings &config, const std::vector<SupportElements> &move_bounds, std::vector<DrawArea> &linear_data,
/// std::function<void()> throw_on_cancel)`
///
/// Draws circles around result_on_layer points of the influence areas. The pin runs the areas through
/// `tbb::parallel_for`; an area is written by its own pass only.
pub fn generate_branch_areas(
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    move_bounds: &[SupportElements],
    linear_data: &mut [DrawArea],
    throw_on_cancel: ThrowOnCancel,
) {
    // Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
    let branch_circle: Polygon = crate::geometry::make_circle(config.branch_radius as f64, SUPPORT_TREE_CIRCLE_RESOLUTION as f64);

    for draw_area in linear_data.iter_mut() {
        let element: &SupportElement = &move_bounds[draw_area.element.0][draw_area.element.1];
        let layer_idx: LayerIndex = element.state.layer_idx;
        let radius: Coord = support_element_radius(config, element);
        let mut parent_uses_min = false;
        let mut local_xy_min_distance: Coord = if element.state.use_min_xy_dist { element.state.xy_min_distance } else { config.xy_min_distance };

        // Calculate multiple ovalized circles, to connect with every parent and child. Also generate regular circle for the current layer. Merge all these into one area.
        let mut movement_directions: Vec<(Point, Coord)> = vec![(Point::new(0, 0), radius)];
        if !element.state.skip_ovalisation {
            if let Some(child_element) = draw_area.child_element {
                let child: &SupportElement = &move_bounds[child_element.0][child_element.1];
                let movement: Point = child.state.result_on_layer - element.state.result_on_layer;
                movement_directions.push((movement, radius));
            }
            // const SupportElements *layer_above = layer_idx + 1 < LayerIndex(move_bounds.size()) ? &move_bounds[layer_idx + 1] : nullptr;
            for &parent_idx in &element.parents {
                let parent: &SupportElement = &move_bounds[(layer_idx + 1) as usize][parent_idx as usize];
                let movement: Point = parent.state.result_on_layer - element.state.result_on_layer;
                //FIXME why max(..., config.support_line_width)?
                movement_directions.push((movement, std::cmp::max(support_element_radius(config, parent), config.support_line_width)));
                parent_uses_min |= parent.state.use_min_xy_dist;
                if parent.state.use_min_xy_dist {
                    local_xy_min_distance = std::cmp::max(local_xy_min_distance, parent.state.xy_min_distance);
                }
            }
        }

        let uses_min_xy = parent_uses_min || element.state.use_min_xy_dist;
        let collision =
            volumes.get_collision(if uses_min_xy { std::cmp::max(0, local_xy_min_distance - config.xy_min_distance) } else { 0 }, layer_idx, uses_min_xy);
        let branch_radius: Coord = config.branch_radius;
        let support_line_width: Coord = config.support_line_width;
        let generate_area = |aoffset: Coord, max_speed: &mut f64| -> Polygons {
            let mut poly: Polygons = Vec::new();
            *max_speed = 0.;
            for movement in &movement_directions {
                // std::max(max_speed, movement.first.cast<double>().norm())
                let speed = norm2(vec2d(movement.0));
                if *max_speed < speed {
                    *max_speed = speed;
                }

                // Visualization: https://jsfiddle.net/0zvcq39L/2/
                // Ovalizes the circle to an ellipse, that contains both old center and new target position.
                let used_scale: f64 = (movement.1 + aoffset) as f64 / (1.0 * branch_radius as f64);
                // `movement.first / 2`: a division of integers.
                let center_position: Point = element.state.result_on_layer + Point::new(movement.0.x / 2, movement.0.y / 2);
                let move_x: f64 = movement.0.x as f64 / (used_scale * branch_radius as f64);
                let move_y: f64 = movement.0.y as f64 / (used_scale * branch_radius as f64);
                // One expression each: the left product of a sum of two products is contracted into the sum, as is
                // the product next to a constant.
                let vsize_inv: f64 = 0.5 / (0.01 + move_x.mul_add(move_x, move_y * move_y).sqrt());

                let matrix: [f64; 4] = [
                    used_scale * (move_x * move_x).mul_add(vsize_inv, 1.),
                    used_scale * (move_x * move_y).mul_add(vsize_inv, 0.),
                    used_scale * (move_x * move_y).mul_add(vsize_inv, 0.),
                    used_scale * (move_y * move_y).mul_add(vsize_inv, 1.),
                ];
                let mut circle = Polygon::new();
                for vertex in &branch_circle.points {
                    circle.points.push(
                        center_position
                            + point_lrint(
                                matrix[0].mul_add(vertex.x as f64, matrix[1] * vertex.y as f64),
                                matrix[2].mul_add(vertex.x as f64, matrix[3] * vertex.y as f64),
                            ),
                    );
                }
                poly.push(circle);
            }

            // There seem to be some rounding errors, causing a branch to be a tiny bit further away from the model that it has to be.
            // This can cause the tip to be slightly further away front the overhang (x/y wise) than optimal. This fixes it, and for every other part, 0.05mm will not be noticed.
            diff_clipped(
                &cu::offset(&cu::union_(&poly), std::cmp::min(50, support_line_width / 4) as f32, crate::clipper::JoinType::jtMiter, 1.2),
                &collision,
            )
        };

        // Ensure branch area will not overlap with model/collision. This can happen because of e.g. ovalization or increase_until_radius.
        let mut max_speed: f64 = 0.;
        let mut polygons: Polygons = generate_area(0, &mut max_speed);
        let fast_relative_movement = max_speed > radius as f64 * 0.75;

        if fast_relative_movement
            || support_element_radius(config, element) - support_element_collision_radius(config, &element.state) > config.support_line_width
        {
            // Simulate the path the nozzle will take on the outermost wall.
            // If multiple parts exist, the outer line will not go all around the support part potentially causing support material to be printed mid air.
            let mut nozzle_path: Vec<ExPolygon> =
                cu::offset_ex(&polygons, (-config.support_line_width / 2) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
            if nozzle_path.len() > 1 {
                // Just try to make the area a tiny bit larger.
                polygons = generate_area(config.support_line_width / 2, &mut max_speed);
                nozzle_path = cu::offset_ex(&polygons, (-config.support_line_width / 2) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                // If larger area did not fix the problem, all parts off the nozzle path that do not contain the center point are removed, hoping for the best.
                if nozzle_path.len() > 1 {
                    let mut polygons_with_correct_center: Vec<ExPolygon> = Vec::new();
                    for part in nozzle_path {
                        let mut drop = false;
                        if !cu_expolygon::contains(&part, &element.state.result_on_layer, true) {
                            // try a fuzzy inside as sometimes the point should be on the border, but is not because of rounding errors...
                            let mut pt: Point = element.state.result_on_layer;
                            move_inside(&crate::geometry::to_polygons(std::slice::from_ref(&part)), &mut pt, 0, i64::MAX);
                            drop = norm2(vec2d(element.state.result_on_layer - pt)) >= scaled_f64(0.025);
                        }
                        if !drop {
                            polygons_with_correct_center.push(part);
                        }
                    }
                    // Increase the area again, to ensure the nozzle path when calculated later is very similar to the one assumed above.
                    polygons = diff_clipped(
                        &cu::offset_expolygons(
                            &polygons_with_correct_center,
                            (config.support_line_width / 2) as f32,
                            crate::clipper::JoinType::jtMiter,
                            1.2,
                        ),
                        //FIXME Vojtech: Clipping may split the region into multiple pieces again, reversing the fixing effort.
                        &collision,
                    );
                }
            }
        }

        draw_area.polygons = polygons;

        throw_on_cancel();
    }
}

/// TreeSupport3D.cpp:2959-3080 `static void smooth_branch_areas(const TreeSupportSettings &config,
/// std::vector<SupportElements> &move_bounds, std::vector<DrawArea> &linear_data, const std::vector<size_t>
/// &linear_data_layers, std::function<void()> throw_on_cancel)`
///
/// Applies some smoothing to the outer wall, intended to smooth out sudden jumps as they can happen when a branch
/// moves though a hole. The areas of a layer go through `tbb::parallel_for` in the pin; here they are taken in
/// their order, as one thread takes them.
pub fn smooth_branch_areas(
    config: &TreeSupportSettings,
    move_bounds: &mut [SupportElements],
    linear_data: &mut [DrawArea],
    linear_data_layers: &[usize],
    throw_on_cancel: ThrowOnCancel,
) {
    let max_radius_change_per_layer: Coord = 1 + config.support_line_width / 2; // this is the upper limit a radius may change per layer. +1 to avoid rounding errors

    // smooth upwards
    let mut layer_idx: LayerIndex = 0;
    while layer_idx < move_bounds.len() as LayerIndex - 1 {
        let processing_base: usize = linear_data_layers[layer_idx as usize];
        let processing_base_above: usize = linear_data_layers[layer_idx as usize + 1];
        let layer_above: &SupportElements = &move_bounds[layer_idx as usize + 1];
        for processing_idx in 0..(processing_base_above - processing_base) {
            let element_at = linear_data[processing_base + processing_idx].element;
            let element: &SupportElement = &move_bounds[element_at.0][element_at.1];
            debug_assert!(element.state.layer_idx == layer_idx);
            let mut max_outer_wall_distance: f64 = 0.;
            let mut do_something = false;
            for &parent_idx in &element.parents {
                let parent: &SupportElement = &layer_above[parent_idx as usize];
                debug_assert!(parent.state.layer_idx == layer_idx + 1);
                if support_element_radius(config, parent) != support_element_collision_radius(config, parent) {
                    do_something = true;
                    let distance = norm2(vec2d(element.state.result_on_layer - parent.state.result_on_layer))
                        - (support_element_radius(config, element) - support_element_radius(config, parent)) as f64;
                    // std::max(max_outer_wall_distance, distance)
                    if max_outer_wall_distance < distance {
                        max_outer_wall_distance = distance;
                    }
                }
            }
            max_outer_wall_distance += max_radius_change_per_layer as f64; // As this change is a bit larger than what usually appears, lost radius can be slowly reclaimed over the layers.
            if do_something {
                let max_allowed_area: Polygons = cu::offset(
                    &linear_data[processing_base + processing_idx].polygons,
                    max_outer_wall_distance as f32,
                    crate::clipper::JoinType::jtMiter,
                    1.2,
                );
                for &parent_idx in &element.parents {
                    let parent: &SupportElement = &layer_above[parent_idx as usize];
                    if support_element_radius(config, parent) != support_element_collision_radius(config, parent) {
                        // No other element on this layer than the current one may be connected to &parent,
                        // thus it is safe to update parent's DrawArea directly.
                        let dst: &mut Polygons = &mut linear_data[processing_base_above + parent_idx as usize].polygons;
                        if !dst.is_empty() {
                            *dst = cu::intersection(dst, &max_allowed_area, cu::ApplySafetyOffset::No);
                        }
                    }
                }
            }
            throw_on_cancel();
        }
        layer_idx += 1;
    }

    // smooth downwards
    if let Some(back) = move_bounds.last_mut() {
        for element in back.iter_mut() {
            element.state.marked = false;
        }
    }
    let mut layer_idx: i32 = move_bounds.len() as i32 - 2;
    while layer_idx >= 0 {
        let processing_base: usize = linear_data_layers[layer_idx as usize];
        let processing_base_above: usize = linear_data_layers[layer_idx as usize + 1];
        for processing_idx in 0..(processing_base_above - processing_base) {
            let element_at = linear_data[processing_base + processing_idx].element;
            let mut do_something = false;
            let mut max_allowed_area: Polygons = Vec::new();
            {
                let element: &SupportElement = &move_bounds[element_at.0][element_at.1];
                let layer_above: &SupportElements = &move_bounds[layer_idx as usize + 1];
                for &parent_idx in &element.parents {
                    let parent: &SupportElement = &layer_above[parent_idx as usize];
                    let max_outer_line_increase: Coord = max_radius_change_per_layer;
                    let mut result: Polygons = cu::offset(
                        &linear_data[processing_base_above + parent_idx as usize].polygons,
                        max_outer_line_increase as f32,
                        crate::clipper::JoinType::jtMiter,
                        1.2,
                    );
                    let direction: Point = element.state.result_on_layer - parent.state.result_on_layer;
                    // move the polygons object
                    for outer in result.iter_mut() {
                        for p in outer.points.iter_mut() {
                            *p = *p + direction;
                        }
                    }
                    max_allowed_area.extend(result);
                    do_something = do_something
                        || parent.state.marked
                        || support_element_collision_radius(config, parent) != support_element_radius(config, parent);
                }
            }
            if do_something {
                // Trim the current drawing areas with max_allowed_area.
                let draw_area: &mut DrawArea = &mut linear_data[processing_base + processing_idx];
                let result: Polygons = cu::intersection(&max_allowed_area, &draw_area.polygons, cu::ApplySafetyOffset::No);
                if area(&result) < area(&draw_area.polygons) {
                    // Mark parent as modified to propagate down.
                    move_bounds[element_at.0][element_at.1].state.marked = true;
                    draw_area.polygons = result;
                }
            }
            throw_on_cancel();
        }
        layer_idx -= 1;
    }
}

/// TreeSupport3D.cpp:3090-3116 `static void drop_non_gracious_areas(const TreeModelVolumes &volumes, const
/// std::vector<DrawArea> &linear_data, std::vector<Polygons> &support_layer_storage, std::function<void()>
/// throw_on_cancel)`
///
/// Drop down areas that do rest non-gracefully on the model to ensure the branch actually rests on something.
/// `move_bounds` holds the elements the areas name.
pub fn drop_non_gracious_areas(
    volumes: &TreeModelVolumes,
    move_bounds: &[SupportElements],
    linear_data: &[DrawArea],
    support_layer_storage: &mut [Polygons],
    throw_on_cancel: ThrowOnCancel,
) {
    let mut dropped_down_areas: Vec<Vec<(LayerIndex, Polygons)>> = vec![Vec::new(); linear_data.len()];
    for idx in 0..linear_data.len() {
        // If a element has no child, it connects to whatever is below as no support further down for it will exist.
        let draw_element: &DrawArea = &linear_data[idx];
        let element: &SupportElement = &move_bounds[draw_element.element.0][draw_element.element.1];
        if !element.state.to_model_gracious && draw_element.child_element.is_none() {
            let mut rest_support: Polygons = Vec::new();
            let layer_idx_first: LayerIndex = element.state.layer_idx - 1;
            let mut layer_idx: LayerIndex = layer_idx_first;
            // As the pin has it: the condition asks for the area of `rest_support` ahead of the first pass, when it
            // is the area of nothing, so no pass is made and nothing is dropped down.
            while area(&rest_support) > tiny_area_threshold() && layer_idx >= 0 {
                rest_support = diff_clipped(
                    if layer_idx == layer_idx_first { &draw_element.polygons } else { &rest_support },
                    &volumes.get_collision(0, layer_idx, false),
                );
                dropped_down_areas[idx].push((layer_idx, rest_support.clone()));
                layer_idx -= 1;
            }
        }
        throw_on_cancel();
    }

    for areas in dropped_down_areas.iter_mut() {
        for pair in areas.drain(..) {
            support_layer_storage[pair.0 as usize].extend(pair.1);
        }
    }
}

/// TreeSupport3D.cpp:3125-3273 `static void finalize_interface_and_support_areas(const PrintObject &print_object, const
/// TreeModelVolumes &volumes, const TreeSupportSettings &config, const std::vector<Polygons> &overhangs,
/// std::vector<Polygons> &support_layer_storage, std::vector<Polygons> &support_roof_storage,
/// SupportGeneratorLayersPtr &bottom_contacts, SupportGeneratorLayersPtr &top_contacts, SupportGeneratorLayersPtr
/// &intermediate_layers, SupportGeneratorLayerStorage &layer_storage, std::function<void()> throw_on_cancel)`
///
/// Generates Support Floor, ensures Support Roof can not cut of branches, and saves the branches as support to
/// storage. The layers go through `tbb::parallel_for` in the pin; one thread takes them in their order, which
/// is the order the layers are allocated in.
#[allow(clippy::too_many_arguments)]
pub fn finalize_interface_and_support_areas(
    print_object: &pf::PrintObject,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    overhangs: &[Polygons],
    support_layer_storage: &mut [Polygons],
    support_roof_storage: &mut [Polygons],
    bottom_contacts: &mut SupportGeneratorLayersPtr,
    top_contacts: &mut SupportGeneratorLayersPtr,
    intermediate_layers: &mut SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
    throw_on_cancel: ThrowOnCancel,
) {
    let interface_pref: InterfacePreference = config.interface_preference; // InterfacePreference::SupportLinesOverwriteInterface;

    // Iterate over the generated circles in parallel and clean them up. Also add support floor.
    for layer_idx in 0..support_layer_storage.len() {
        // Subtract support lines of the branches from the roof
        let mut support_roof: usize = if layer_idx < top_contacts.len() { top_contacts[layer_idx] } else { SUPPORT_GENERATOR_LAYER_PTR_NULL };
        let mut support_roof_polygons: Polygons = Vec::new();
        if !support_roof_storage[layer_idx].is_empty() {
            if support_roof != SUPPORT_GENERATOR_LAYER_PTR_NULL && !layer_storage[support_roof].polygons.is_empty() {
                support_roof_polygons = cu::union_2(&support_roof_storage[layer_idx], &layer_storage[support_roof].polygons);
                layer_storage[support_roof].polygons.clear();
            } else {
                support_roof_polygons = std::mem::take(&mut support_roof_storage[layer_idx]);
            }
        } else if support_roof != SUPPORT_GENERATOR_LAYER_PTR_NULL {
            support_roof_polygons = std::mem::take(&mut layer_storage[support_roof].polygons);
            layer_storage[support_roof].polygons.clear();
        }
        let mut base_layer_polygons: Polygons = std::mem::take(&mut support_layer_storage[layer_idx]);

        if !base_layer_polygons.is_empty() {
            // Most of the time in this function is this union call. Can take 300+ ms when a lot of areas are to be unioned.
            base_layer_polygons = crate::mutable_polygon::smooth_outward_polygons(cu::union_(&base_layer_polygons), config.support_line_width); //FIXME was .smooth(50);
            // simplify a bit, to ensure the output does not contain outrageous amounts of vertices. Should not be necessary, just a precaution.
            // std::min(scaled<double>(0.03), double(config.resolution))
            let resolution = config.resolution as f64;
            let tolerance = if resolution < scaled_f64(0.03) { resolution } else { scaled_f64(0.03) };
            base_layer_polygons = crate::support::tree_model_volumes::polygons_simplify_pin(&base_layer_polygons, tolerance);
        }

        if !support_roof_polygons.is_empty() && !base_layer_polygons.is_empty() {
            match interface_pref {
                InterfacePreference::InterfaceAreaOverwritesSupport => {
                    base_layer_polygons = cu::diff(&base_layer_polygons, &support_roof_polygons, cu::ApplySafetyOffset::No);
                }
                InterfacePreference::SupportAreaOverwritesInterface => {
                    support_roof_polygons = cu::diff(&support_roof_polygons, &base_layer_polygons, cu::ApplySafetyOffset::No);
                }
                // `assert(false); [[fallthrough]];` to the case that does nothing.
                InterfacePreference::InterfaceLinesOverwriteSupport | InterfacePreference::SupportLinesOverwriteInterface => {
                    debug_assert!(false);
                }
                InterfacePreference::Nothing => {}
            }
        }

        // Subtract support floors from the support area and add them to the support floor instead.
        if config.support_bottom_layers > 0 && !base_layer_polygons.is_empty() {
            let layer_outset: Polygons = diff_clipped(
                &if config.support_bottom_offset > 0 {
                    cu::offset(&base_layer_polygons, config.support_bottom_offset as f32, crate::clipper::JoinType::jtMiter, 1.2)
                } else {
                    base_layer_polygons.clone()
                },
                &volumes.get_collision(0, layer_idx as LayerIndex, false),
            );
            let mut floor_layer: Polygons = Vec::new();
            let mut layers_below: usize = 0;
            while layers_below <= config.support_bottom_layers {
                // one sample at 0 layers below, another at config.support_bottom_layers. In-between samples at config.performance_interface_skip_layers distance from each other.
                let sample_layer: usize =
                    std::cmp::max(0, (layer_idx as i32 - layers_below as i32) - config.z_distance_bottom_layers as i32) as usize;
                //FIXME subtract the wipe tower
                floor_layer.extend(cu::intersection(&layer_outset, &overhangs[sample_layer], cu::ApplySafetyOffset::No));
                if layers_below < config.support_bottom_layers {
                    layers_below = std::cmp::min(layers_below + 1, config.support_bottom_layers);
                } else {
                    break;
                }
            }
            if !floor_layer.is_empty() {
                if bottom_contacts[layer_idx] == SUPPORT_GENERATOR_LAYER_PTR_NULL {
                    bottom_contacts[layer_idx] =
                        layer_allocate(layer_storage, SupporLayerType::SltBottomContact, print_object.slicing_parameters(), config, layer_idx);
                }
                let support_bottom: usize = bottom_contacts[layer_idx];
                layer_storage[support_bottom].polygons = cu::union_2(&floor_layer, &layer_storage[support_bottom].polygons);
                // Subtract the support floor from the normal support.
                base_layer_polygons = diff_clipped(
                    &base_layer_polygons,
                    &cu::offset(&layer_storage[support_bottom].polygons, (0.01 / SCALING_FACTOR) as f32, crate::clipper::JoinType::jtMiter, 1.2),
                );
            }
        }

        if !support_roof_polygons.is_empty() {
            if support_roof == SUPPORT_GENERATOR_LAYER_PTR_NULL {
                support_roof = layer_allocate(layer_storage, SupporLayerType::SltTopContact, print_object.slicing_parameters(), config, layer_idx);
                top_contacts[layer_idx] = support_roof;
            }
            layer_storage[support_roof].polygons = cu::union_(&support_roof_polygons);
        }
        if !base_layer_polygons.is_empty() {
            let base_layer: usize = layer_allocate(layer_storage, SupporLayerType::SltBase, print_object.slicing_parameters(), config, layer_idx);
            intermediate_layers[layer_idx] = base_layer;
            layer_storage[base_layer].polygons = cu::union_(&base_layer_polygons);
        }

        throw_on_cancel();
    }
}

/// The areas of draw_areas() and their first indices by layer (TreeSupport3D.cpp:3296-3326, the block of
/// draw_areas() that puts all SupportElements into a layer independent storage): `linear_data` and
/// `linear_data_layers`. An element's child is the element of the layer below that names it as a parent whose
/// point is set; the pin looks it up by the address of the element, only one link points to a node above from
/// below.
pub fn draw_areas_linear_data(move_bounds: &[SupportElements]) -> (Vec<DrawArea>, Vec<usize>) {
    let mut linear_data: Vec<DrawArea> = Vec::new();
    let mut linear_data_layers: Vec<usize> = Vec::new();
    {
        // By the index of the element of this layer: the element below.
        let mut map_downwards_old: Vec<Option<usize>> = Vec::new();
        for layer_idx in 0..move_bounds.len() {
            let layer_above: Option<&SupportElements> = if layer_idx + 1 < move_bounds.len() { Some(&move_bounds[layer_idx + 1]) } else { None };
            let mut map_downwards_new: Vec<Option<usize>> = vec![None; layer_above.map_or(0, |l| l.len())];
            linear_data_layers.push(linear_data.len());
            for (elem_idx, elem) in move_bounds[layer_idx].iter().enumerate() {
                let mut child: Option<(usize, usize)> = None;
                if layer_idx > 0 {
                    if let Some(Some(child_idx)) = map_downwards_old.get(elem_idx) {
                        child = Some((layer_idx - 1, *child_idx));
                    }
                }
                for &parent_idx in &elem.parents {
                    let parent: &SupportElement = &layer_above.expect("layer_above")[parent_idx as usize];
                    if parent.state.result_on_layer_is_set() && map_downwards_new[parent_idx as usize].is_none() {
                        map_downwards_new[parent_idx as usize] = Some(elem_idx);
                    }
                }
                linear_data.push(DrawArea { element: (layer_idx, elem_idx), child_element: child, polygons: Vec::new() });
            }
            map_downwards_old = map_downwards_new;
        }
        linear_data_layers.push(linear_data.len());
    }
    (linear_data, linear_data_layers)
}

/// TreeSupport3D.cpp:3282-3436 `static void draw_areas(PrintObject &print_object, const TreeModelVolumes &volumes, const
/// TreeSupportSettings &config, const std::vector<Polygons> &overhangs, std::vector<SupportElements> &move_bounds,
/// SupportGeneratorLayersPtr &bottom_contacts, SupportGeneratorLayersPtr &top_contacts, SupportGeneratorLayersPtr
/// &intermediate_layers, SupportGeneratorLayerStorage &layer_storage, std::function<void()> throw_on_cancel)`
///
/// Draws circles around result_on_layer points of the influence areas and applies some post processing.
#[allow(clippy::too_many_arguments)]
pub fn draw_areas(
    print_object: &pf::PrintObject,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    overhangs: &[Polygons],
    move_bounds: &mut [SupportElements],
    bottom_contacts: &mut SupportGeneratorLayersPtr,
    top_contacts: &mut SupportGeneratorLayersPtr,
    intermediate_layers: &mut SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
    throw_on_cancel: ThrowOnCancel,
) {
    let mut support_layer_storage: Vec<Polygons> = vec![Vec::new(); move_bounds.len()];
    let mut support_roof_storage: Vec<Polygons> = vec![Vec::new(); move_bounds.len()];
    // All SupportElements are put into a layer independent storage to improve parallelization.
    let (mut linear_data, linear_data_layers) = draw_areas_linear_data(move_bounds);

    throw_on_cancel();

    // Generate the circles that will be the branches.
    generate_branch_areas(volumes, config, move_bounds, &mut linear_data, throw_on_cancel);

    // In some edgecases a branch may go though a hole, where the regular radius does not fit. This can result in an apparent jump in branch radius. As such this cases need to be caught and smoothed out.
    smooth_branch_areas(config, move_bounds, &mut linear_data, &linear_data_layers, throw_on_cancel);

    // drop down all trees that connect non gracefully with the model
    drop_non_gracious_areas(volumes, move_bounds, &linear_data, &mut support_layer_storage, throw_on_cancel);

    // Single threaded combining all support areas to the right layers.
    {
        let mut begin: usize = 0;
        for layer_idx in 0..move_bounds.len() as LayerIndex {
            let mut end: usize = begin;
            while end != linear_data.len() && move_bounds[linear_data[end].element.0][linear_data[end].element.1].state.layer_idx == layer_idx {
                end += 1;
            }
            for it in begin..end {
                let state = &move_bounds[linear_data[it].element.0][linear_data[it].element.1].state;
                let polygons = std::mem::take(&mut linear_data[it].polygons);
                if state.missing_roof_layers > state.distance_to_top {
                    support_roof_storage[layer_idx as usize].extend(polygons);
                } else {
                    support_layer_storage[layer_idx as usize].extend(polygons);
                }
            }
            begin = end;
        }
    }

    finalize_interface_and_support_areas(
        print_object,
        volumes,
        config,
        overhangs,
        &mut support_layer_storage,
        &mut support_roof_storage,
        bottom_contacts,
        top_contacts,
        intermediate_layers,
        layer_storage,
        throw_on_cancel,
    );
}

// =============================================================================
// The port of the pin: TreeSupport3D.cpp, the meshes of the branches (circles_intersect() to discretize_polygon())
// =============================================================================

use crate::geometry::geometry::{eigen, Vec3d};
use crate::normal_utils::{indexed_triangle_set, StlTriangleVertexIndices, Vec3f};

/// TreeSupport3D.cpp:3434-3451 `static bool circles_intersect(const Vec3d &p1, const Vec3d &n1, const double r1, const
/// Vec3d &p2, const Vec3d &n2, const double r2)`
///
/// Test whether two circles, each on its own plane in 3D intersect.
/// Circles are considered intersecting, if the lowest point on one circle is below the other circle's plane.
/// Assumption: The two planes are oriented the same way.
pub fn circles_intersect(p1: &Vec3d, n1: &Vec3d, r1: f64, p2: &Vec3d, n2: &Vec3d, r2: f64) -> bool {
    debug_assert!(eigen::dot3(n1, n2) >= 0.);

    let z: Vec3d = eigen::cross3(n1, n2);
    let dir1: Vec3d = eigen::cross3(&z, n1);
    // p1 + dir1 * (r1 / dir1.norm())
    let s1 = r1 / eigen::norm3(&dir1);
    let lowest_point1 = Vec3d::new(p1.x + dir1.x * s1, p1.y + dir1.y * s1, p1.z + dir1.z * s1);
    if eigen::dot3(n2, &lowest_point1) <= 0. {
        return true;
    }
    let dir2: Vec3d = eigen::cross3(&z, n2);
    let s2 = r2 / eigen::norm3(&dir2);
    let lowest_point2 = Vec3d::new(p2.x + dir2.x * s2, p2.y + dir2.y * s2, p2.z + dir2.z * s2);
    eigen::dot3(n1, &lowest_point2) <= 0.
}

/// TreeSupport3D.cpp:3453-3468 `template<bool flip_normals> void triangulate_fan(indexed_triangle_set &its, int ifan,
/// int ibegin, int iend)`
pub fn triangulate_fan(its: &mut indexed_triangle_set, flip_normals: bool, ifan: i32, ibegin: i32, iend: i32) {
    // at least 3 vertices, increasing order.
    debug_assert!(ibegin + 3 <= iend);
    debug_assert!(ibegin >= 0 && iend as usize <= its.vertices.len());
    debug_assert!(ifan >= 0 && (ifan as usize) < its.vertices.len());
    let num_faces = iend - ibegin;
    its.indices.reserve((num_faces * 3).max(0) as usize);
    let mut v = ibegin;
    let mut u = iend - 1;
    while v < iend {
        if flip_normals {
            its.indices.push(StlTriangleVertexIndices::new(ifan, u, v));
        } else {
            its.indices.push(StlTriangleVertexIndices::new(ifan, v, u));
        }
        u = v;
        v += 1;
    }
}

// `(a - b).squaredNorm()` of two `Vec3f`: the difference, then the redux of its squares (a packet of the first
// two, then the third added), as `eigen::squared_norm3` has it for doubles.
#[inline]
fn squared_norm_diff3f(a: &Vec3f, b: &Vec3f) -> f32 {
    let (dx, dy, dz) = (a.x - b.x, a.y - b.y, a.z - b.z);
    (dx * dx + dy * dy) + dz * dz
}

/// TreeSupport3D.cpp:3470-3533 `static void triangulate_strip(indexed_triangle_set &its, int ibegin1, int iend1, int
/// ibegin2, int iend2)`
pub fn triangulate_strip(its: &mut indexed_triangle_set, ibegin1: i32, iend1: i32, ibegin2: i32, iend2: i32) {
    // at least 3 vertices, increasing order.
    debug_assert!(ibegin1 + 3 <= iend1);
    debug_assert!(ibegin1 >= 0 && iend1 as usize <= its.vertices.len());
    debug_assert!(ibegin2 + 3 <= iend2);
    debug_assert!(ibegin2 >= 0 && iend2 as usize <= its.vertices.len());
    let mut n1 = iend1 - ibegin1;
    let mut n2 = iend2 - ibegin2;
    its.indices.reserve(((n1 + n2) * 3).max(0) as usize);

    // For the first vertex of 1st strip, find the closest vertex on the 2nd strip.
    let mut istart2 = ibegin2;
    {
        let p1: Vec3f = its.vertices[ibegin1 as usize];
        let mut d2min = f32::MAX;
        for i in ibegin2..iend2 {
            let p2: &Vec3f = &its.vertices[i as usize];
            let d2: f32 = squared_norm_diff3f(p2, &p1);
            if d2 < d2min {
                d2min = d2;
                istart2 = i;
            }
        }
    }

    // Now triangulate the strip zig-zag fashion taking always the shortest connection if possible.
    let mut u = ibegin1;
    let mut v = istart2;
    while n1 > 0 || n2 > 0 {
        let take_first: bool;
        let mut u2: i32 = 0;
        let mut v2: i32 = 0;
        let update_u2 = |u2: &mut i32| {
            *u2 = u;
            *u2 += 1;
            if *u2 == iend1 {
                *u2 = ibegin1;
            }
        };
        let update_v2 = |v2: &mut i32| {
            *v2 = v;
            *v2 += 1;
            if *v2 == iend2 {
                *v2 = ibegin2;
            }
        };
        if n1 == 0 {
            take_first = false;
            update_v2(&mut v2);
        } else if n2 == 0 {
            take_first = true;
            update_u2(&mut u2);
        } else {
            update_u2(&mut u2);
            update_v2(&mut v2);
            let l1: f32 = squared_norm_diff3f(&its.vertices[u2 as usize], &its.vertices[v as usize]);
            let l2: f32 = squared_norm_diff3f(&its.vertices[v2 as usize], &its.vertices[u as usize]);
            take_first = l1 < l2;
        }
        if take_first {
            its.indices.push(StlTriangleVertexIndices::new(u, u2, v));
            n1 -= 1;
            u = u2;
        } else {
            its.indices.push(StlTriangleVertexIndices::new(u, v2, v));
            n2 -= 1;
            v = v2;
        }
    }
}

/// `sin(x)` and `cos(x)` of one `float`, in the order (sin, cos), where the C++ takes both of one argument
/// (discretize_circle(), TreeSupport3D.cpp:3555): clang for macOS turns the two calls into one call of
/// `__sincosf_stret` (see `crate::clipper::sin_cos` for the doubles), which this calls itself on Apple targets.
///
/// FIDELITY-NOTE: on the other targets this is `x.sin()` and `x.cos()`; whether the Linux and Windows builds of
/// the C++ get the same bits there has not been tested.
#[inline]
pub fn sin_cos_f32(x: f32) -> (f32, f32) {
    #[cfg(target_vendor = "apple")]
    {
        #[repr(C)]
        struct Float2 {
            sinval: f32,
            cosval: f32,
        }
        extern "C" {
            fn __sincosf_stret(x: f32) -> Float2;
        }
        // SAFETY: `__sincosf_stret` of libSystem takes a float and returns the two floats by value.
        let r = unsafe { __sincosf_stret(x) };
        (r.sinval, r.cosval)
    }
    #[cfg(not(target_vendor = "apple"))]
    {
        (x.sin(), x.cos())
    }
}

// `a.cross(b)` of two `Vec3f` (OrthoMethods.h `cross_impl<..., 3>`: each difference one expression, its left
// product contracted), as `eigen::cross3` has it for doubles.
#[inline]
fn cross3f(a: &Vec3f, b: &Vec3f) -> Vec3f {
    Vec3f::new(a.y.mul_add(b.z, -(a.z * b.y)), a.z.mul_add(b.x, -(a.x * b.z)), a.x.mul_add(b.y, -(a.y * b.x)))
}

// `v.normalized()` of a `Vec3f` (Dot.h: `n / sqrt(z)` when the squared norm `z > 0`, else a copy), as
// `eigen::normalized3` has it for doubles.
#[inline]
fn normalized3f(v: &Vec3f) -> Vec3f {
    let z: f32 = (v.x * v.x + v.y * v.y) + v.z * v.z;
    if z > 0. {
        let s = z.sqrt();
        Vec3f::new(v.x / s, v.y / s, v.z / s)
    } else {
        *v
    }
}

/// TreeSupport3D.cpp:3536-3559 `static std::pair<int, int> discretize_circle(const Vec3f &center, const Vec3f &normal,
/// const float radius, const float eps, std::vector<Vec3f> &pts)`
///
/// Discretize 3D circle, append to output vector, return ranges of indices of the points added.
pub fn discretize_circle(center: &Vec3f, normal: &Vec3f, radius: f32, eps: f32, pts: &mut Vec<Vec3f>) -> (i32, i32) {
    // Calculate discretization step and number of steps.
    // `float angle_step = 2. * acos(1. - eps / radius);`: the quotient of the floats, the rest in doubles.
    let mut angle_step: f32 = (2. * (1. - (eps / radius) as f64).acos()) as f32;
    let nsteps: i32 = (2. * std::f64::consts::PI / angle_step as f64).ceil() as i32;
    angle_step = (2. * std::f64::consts::PI / nsteps as f64) as f32;

    // Prepare coordinate system for the circle plane.
    let mut x: Vec3f = normalized3f(&cross3f(normal, &Vec3f::new(0., -1., 0.)));
    let mut y: Vec3f = normalized3f(&cross3f(normal, &x));

    // Discretize the circle.
    let begin = pts.len() as i32;
    pts.reserve(nsteps.max(0) as usize);
    let mut angle: f32 = 0.;
    x = Vec3f::new(x.x * radius, x.y * radius, x.z * radius);
    y = Vec3f::new(y.x * radius, y.y * radius, y.z * radius);
    for _ in 0..nsteps {
        let (s, c) = sin_cos_f32(angle);
        // center + x * cos(angle) + y * sin(angle)
        pts.push(Vec3f::new((center.x + x.x * c) + y.x * s, (center.y + x.y * c) + y.y * s, (center.z + x.z * c) + y.z * s));
        angle += angle_step;
    }
    (begin, pts.len() as i32)
}

/// TreeSupport3D.cpp:3562-3575 `static std::pair<int, int> discretize_polygon(const Vec3f& center, const Polygons& polys,
/// std::vector<Vec3f>& pts)`
///
/// Discretize polygon, append to output vector, return ranges of indices of the points added.
pub fn discretize_polygon(center: &Vec3f, polys: &[Polygon], pts: &mut Vec<Vec3f>) -> (i32, i32) {
    let poly: &Polygon = &polys[0];
    let nsteps: usize = poly.points.len();
    // Discretize the circle.
    let begin = pts.len() as i32;
    pts.reserve(nsteps);
    for i in 0..nsteps {
        // `Vec3f pt(poly.points[i].x(), poly.points[i].y(), center.z());`
        pts.push(Vec3f::new(poly.points[i].x as f32, poly.points[i].y as f32, center.z));
    }
    (begin, pts.len() as i32)
}

// =============================================================================
// The port of the pin: TreeSupport3D.cpp, the tube of a branch (TubeEndCap to extrude_branch())
// =============================================================================

/// TreeSupport3D.cpp:3576-3579 `enum class TubeEndCap { HemisphereMesh, FlatDisk };`
///
/// End treatment of a tube piece. HemisphereMesh is the original closed cap.
/// FlatDisk is used at 61563 split joints: close the frustum in the section plane
/// without a polar hemisphere; the missing hemisphere is injected later as 2D slices.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TubeEndCap {
    HemisphereMesh,
    FlatDisk,
}

/// TreeSupport3D.cpp:3581-3594 `static bool split_cap_use_mesh_hemispheres()`
///
/// TREE_SUPPORT_SPLIT_CAP_MODE:
///   analytic (default) — FlatDisk + hemisphere_slice_polygon() injection
///   mesh               — 61563-style 3D hemispheres at split joints (no 2D inject)
pub fn split_cap_use_mesh_hemispheres() -> bool {
    static USE_MESH: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *USE_MESH.get_or_init(|| match std::env::var_os("TREE_SUPPORT_SPLIT_CAP_MODE") {
        Some(v) => v == "mesh" || v == "3d" || v == "Mesh" || v == "MESH",
        None => false,
    })
}

// `to_3d(unscaled<double>(p), z)`: Point.hpp:565-577, `p.cast<double>() * SCALING_FACTOR`.
#[inline]
fn unscaled_to_3d(p: &Point, z: f64) -> Vec3d {
    Vec3d::new(p.x as f64 * crate::libslic3r::SCALING_FACTOR, p.y as f64 * crate::libslic3r::SCALING_FACTOR, z)
}

// `unscaled<float>(v)` of a coordinate: Point.hpp:556-563, `float(v) * float(SCALING_FACTOR)`.
#[inline]
fn unscaled_f32(v: Coord) -> f32 {
    v as f32 * crate::libslic3r::SCALING_FACTOR as f32
}

// `unscaled<double>(v)` of a coordinate: Point.hpp:556-563, `double(v) * SCALING_FACTOR`.
#[inline]
fn unscaled_f64(v: Coord) -> f64 {
    v as f64 * crate::libslic3r::SCALING_FACTOR
}

// `v.cast<float>()` of a `Vec3d`
#[inline]
fn cast3f(v: &Vec3d) -> Vec3f {
    Vec3f::new(v.x as f32, v.y as f32, v.z as f32)
}

#[inline]
fn sub3d(a: &Vec3d, b: &Vec3d) -> Vec3d {
    Vec3d::new(a.x - b.x, a.y - b.y, a.z - b.z)
}

#[inline]
fn add3d(a: &Vec3d, b: &Vec3d) -> Vec3d {
    Vec3d::new(a.x + b.x, a.y + b.y, a.z + b.z)
}

/// TreeSupport3D.cpp:3596-3701 `static std::pair<float, float> extrude_branch_tube(const std::vector<const
/// SupportElement*> &path, const TreeSupportSettings &config, const SlicingParameters &slicing_params, const
/// std::vector<SupportElements> &move_bounds, indexed_triangle_set &result, TubeEndCap bottom_cap =
/// TubeEndCap::HemisphereMesh, TubeEndCap top_cap = TubeEndCap::HemisphereMesh)`
///
/// Returns Z span of the generated mesh. Generates one closed tube (bottom cap,
/// bisector-normal section circles connected by zig-zag strips, top cap) for the
/// given continuous path. This is the original extrude_branch body, now a reusable piece
/// generator invoked by extrude_branch(), which may split a path into self-intersection-free
/// pieces before calling this.
///
/// `cos(angle)` and `sin(angle)` of the hemispheres take a `float` and are one call (see `sin_cos_f32`).
pub fn extrude_branch_tube(
    path: &[&SupportElement],
    config: &TreeSupportSettings,
    slicing_params: &crate::slicing::SlicingParams,
    _move_bounds: &[SupportElements],
    result: &mut indexed_triangle_set,
    bottom_cap: TubeEndCap,
    top_cap: TubeEndCap,
) -> (f32, f32) {
    use crate::support::tree_support_common::layer_z;
    use std::f64::consts::PI as M_PI;

    let mut p1: Vec3d;
    let mut p2: Vec3d;
    let mut p3: Vec3d;
    let mut v1: Vec3d;
    let mut v2: Vec3d;
    let mut nprev: Vec3d;
    let mut ncurrent: Vec3d;
    const EPS: f32 = 0.015;
    // Not initialized at the pin; a bottom hemisphere of one step (a radius under 0.052 mm) would leave it so.
    let mut prev_strip: (i32, i32) = (0, 0);
    let vertex_begin: usize = result.vertices.len();

    for ipath in 1..path.len() {
        let prev: &SupportElement = path[ipath - 1];
        let current: &SupportElement = path[ipath];
        p1 = unscaled_to_3d(&prev.state.result_on_layer, layer_z(slicing_params, config, prev.state.layer_idx as usize));
        p2 = unscaled_to_3d(&current.state.result_on_layer, layer_z(slicing_params, config, current.state.layer_idx as usize));
        v1 = eigen::normalized3(&sub3d(&p2, &p1));
        if ipath == 1 {
            nprev = v1;
            let radius: f32 = unscaled_f32(support_element_radius(config, prev));
            if bottom_cap == TubeEndCap::FlatDisk {
                // Section-plane disk at p1; no polar vertices, so mesh Z stays inside the frustum.
                let ifan = result.vertices.len() as i32;
                result.vertices.push(cast3f(&p1));
                prev_strip = discretize_circle(&cast3f(&p1), &cast3f(&nprev), radius, EPS, &mut result.vertices);
                triangulate_fan(result, false, ifan, prev_strip.0, prev_strip.1);
            } else {
                // Extrude the bottom half sphere.
                // `float angle_step = 2. * acos(1. - eps / radius);`: the quotient of the floats, the rest in doubles.
                let mut angle_step: f32 = (2. * (1. - (EPS / radius) as f64).acos()) as f32;
                let nsteps: i32 = (M_PI / (2. * angle_step as f64)).ceil() as i32;
                angle_step = (M_PI / (2. * nsteps as f64)) as f32;
                let ifan = result.vertices.len() as i32;
                // (p1 - nprev * radius).cast<float>()
                let r = radius as f64;
                result.vertices.push(cast3f(&Vec3d::new(p1.x - nprev.x * r, p1.y - nprev.y * r, p1.z - nprev.z * r)));
                let mut angle: f32 = angle_step;
                let mut i = 1;
                while i < nsteps {
                    let (s, c) = sin_cos_f32(angle);
                    // (p1 - nprev * radius * cos(angle)).cast<float>()
                    let c = c as f64;
                    let center = Vec3d::new(p1.x - nprev.x * r * c, p1.y - nprev.y * r * c, p1.z - nprev.z * r * c);
                    let strip = discretize_circle(&cast3f(&center), &cast3f(&nprev), radius * s, EPS, &mut result.vertices);
                    if i == 1 {
                        triangulate_fan(result, false, ifan, strip.0, strip.1);
                    } else {
                        triangulate_strip(result, prev_strip.0, prev_strip.1, strip.0, strip.1);
                    }
                    prev_strip = strip;
                    i += 1;
                    angle += angle_step;
                }
            }
        }
        if ipath + 1 == path.len() {
            // End of the tube.
            ncurrent = v1;
            let radius: f32 = unscaled_f32(support_element_radius(config, current));
            if top_cap == TubeEndCap::FlatDisk {
                let strip = discretize_circle(&cast3f(&p2), &cast3f(&ncurrent), radius, EPS, &mut result.vertices);
                triangulate_strip(result, prev_strip.0, prev_strip.1, strip.0, strip.1);
                let ifan = result.vertices.len() as i32;
                result.vertices.push(cast3f(&p2));
                triangulate_fan(result, true, ifan, strip.0, strip.1);
            } else {
                // Extrude the top half sphere.
                let mut angle_step: f32 = (2. * (1. - (EPS / radius) as f64).acos()) as f32;
                let nsteps: i32 = (M_PI / (2. * angle_step as f64)).ceil() as i32;
                angle_step = (M_PI / (2. * nsteps as f64)) as f32;
                let mut angle: f32 = (M_PI / 2.) as f32;
                let r = radius as f64;
                for _ in 0..nsteps {
                    let (s, c) = sin_cos_f32(angle);
                    // (p2 + ncurrent * radius * cos(angle)).cast<float>()
                    let c = c as f64;
                    let center = Vec3d::new(p2.x + ncurrent.x * r * c, p2.y + ncurrent.y * r * c, p2.z + ncurrent.z * r * c);
                    let strip = discretize_circle(&cast3f(&center), &cast3f(&ncurrent), radius * s, EPS, &mut result.vertices);
                    triangulate_strip(result, prev_strip.0, prev_strip.1, strip.0, strip.1);
                    prev_strip = strip;
                    angle -= angle_step;
                }
                let ifan = result.vertices.len() as i32;
                // (p2 + ncurrent * radius).cast<float>()
                result.vertices.push(cast3f(&Vec3d::new(p2.x + ncurrent.x * r, p2.y + ncurrent.y * r, p2.z + ncurrent.z * r)));
                triangulate_fan(result, true, ifan, prev_strip.0, prev_strip.1);
            }
        } else {
            let next: &SupportElement = path[ipath + 1];
            p3 = unscaled_to_3d(&next.state.result_on_layer, layer_z(slicing_params, config, next.state.layer_idx as usize));
            v2 = eigen::normalized3(&sub3d(&p3, &p2));
            ncurrent = eigen::normalized3(&add3d(&v1, &v2));
            let radius: f32 = unscaled_f32(support_element_radius(config, current));
            let strip = discretize_circle(&cast3f(&p2), &cast3f(&ncurrent), radius, EPS, &mut result.vertices);
            triangulate_strip(result, prev_strip.0, prev_strip.1, strip.0, strip.1);
            prev_strip = strip;
        }
    }

    // A tilted hemisphere reaches farther in Z than its axis pole. Compute the
    // span from every generated vertex so callers slice the complete mesh.
    let mut zmin: f32 = f32::MAX;
    let mut zmax: f32 = -f32::MAX;
    for i in vertex_begin..result.vertices.len() {
        let z = result.vertices[i].z;
        // std::min(zmin, z), std::max(zmax, z)
        if z < zmin {
            zmin = z;
        }
        if zmax < z {
            zmax = z;
        }
    }
    (zmin, zmax)
}

/// TreeSupport3D.cpp:3703-3731 `static bool section_circles_intersect(const Vec3d &p1, const Vec3d &n1, const double
/// r1, const Vec3d &p2, const Vec3d &n2, const double r2)`
///
/// Geometrically correct section-circle intersection test (the file-scope circles_intersect
/// above is incomplete dead code). Two oriented section discs (p1,n1,r1) and (p2,n2,r2)
/// intersect only when each disc crosses the other's plane AND their chords on the shared
/// plane-intersection line overlap. Near-parallel normals mean cleanly stacked circles
/// (a straight tube segment) -> not intersecting.
pub fn section_circles_intersect(p1: &Vec3d, n1: &Vec3d, r1: f64, p2: &Vec3d, n2: &Vec3d, r2: f64) -> bool {
    // std::max(0.0, x)
    #[inline]
    fn max0(x: f64) -> f64 {
        if 0.0 < x {
            x
        } else {
            0.0
        }
    }
    // std::clamp(n1.dot(n2), -1.0, 1.0)
    let d = eigen::dot3(n1, n2);
    let cos_tilt: f64 = if d < -1.0 {
        -1.0
    } else if 1.0 < d {
        1.0
    } else {
        d
    };
    // `1.0 - cos_tilt * cos_tilt`: one expression, the product contracted into the difference.
    let sin_tilt: f64 = max0((-cos_tilt).mul_add(cos_tilt, 1.0)).sqrt(); // sin of tilt angle
    if sin_tilt < 1e-4 {
        return false;
    }
    let delta: Vec3d = sub3d(p1, p2);
    // Signed distance of each center to the other section's plane; a disc reaches the other
    // plane only if this does not exceed the disc's projected extent (r * sin_tilt).
    let dist1: f64 = eigen::dot3(n2, &delta);
    let dist2: f64 = eigen::dot3(n1, &delta);
    if dist1.abs() > r1 * sin_tilt || dist2.abs() > r2 * sin_tilt {
        return false;
    }
    // Both discs cross the plane-intersection line, but they truly intersect only if their
    // chords on that line overlap. e = n1 x n2 runs along the line with |e| = sin_tilt; h1/h2
    // are the chord half-lengths, axial is the gap between the two chord midpoints.
    let e: Vec3d = eigen::cross3(n1, n2);
    let axial: f64 = eigen::dot3(&e, &delta).abs() / sin_tilt;
    // `r1 * r1 - (dist1 / sin_tilt) * (dist1 / sin_tilt)`: one expression, the left product contracted.
    let q1 = dist1 / sin_tilt;
    let q2 = dist2 / sin_tilt;
    let h1: f64 = max0(r1.mul_add(r1, -(q1 * q1))).sqrt();
    let h2: f64 = max0(r2.mul_add(r2, -(q2 * q2))).sqrt();
    axial <= h1 + h2
}

/// TreeSupport3D.cpp:3740-3746 `struct ExtrudeSplitCap`
///
/// Split-joint hemisphere descriptors for analytic 2D injection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtrudeSplitCap {
    pub center: Vec3d,
    pub normal: Vec3d,
    pub radius: f64,
    pub is_bottom: bool,
}

impl Default for ExtrudeSplitCap {
    // center { Vec3d::Zero() }, normal { Vec3d::UnitZ() }, radius { 0. }, is_bottom { false }
    fn default() -> Self {
        ExtrudeSplitCap { center: Vec3d::new(0., 0., 0.), normal: Vec3d::new(0., 0., 1.), radius: 0., is_bottom: false }
    }
}

/// TreeSupport3D.cpp:3733-3739, 3748-3847 `static std::pair<float, float> extrude_branch(const std::vector<const
/// SupportElement*> &path, const TreeSupportSettings &config, const SlicingParameters &slicing_params, const
/// std::vector<SupportElements> &move_bounds, indexed_triangle_set &result, std::vector<ExtrudeSplitCap>
/// *split_caps = nullptr)`
///
/// Safe-joints wrapper around extrude_branch_tube. When consecutive section circles
/// intersect in 3D, the zig-zag triangulation self-folds and slicing drops that area
/// (a gap / missing slice in the sliced layers). In that case split the path at the
/// offending edges and emit each unsafe edge as a standalone closed capsule. Adjacent
/// pieces meet at a FlatDisk in the shared node's section plane; the missing hemispheres
/// are injected later as analytic 2D slices and merged by the downstream non-zero fill
/// clipping (diff_clipped / intersection), same as the original overlapping end-caps.
pub fn extrude_branch(
    path: &[&SupportElement],
    config: &TreeSupportSettings,
    slicing_params: &crate::slicing::SlicingParams,
    move_bounds: &[SupportElements],
    result: &mut indexed_triangle_set,
    mut split_caps: Option<&mut Vec<ExtrudeSplitCap>>,
) -> (f32, f32) {
    use crate::support::tree_support_common::layer_z;

    let n: usize = path.len();
    if n < 3 {
        return extrude_branch_tube(path, config, slicing_params, move_bounds, result, TubeEndCap::HemisphereMesh, TubeEndCap::HemisphereMesh);
    }

    // Per-node section circle: interior nodes use the bisector normal (matches
    // extrude_branch_tube's mid-tube sections), endpoints use the segment direction.
    let mut node_pos: Vec<Vec3d> = vec![Vec3d::new(0., 0., 0.); n];
    let mut node_radius: Vec<f64> = vec![0.; n];
    let mut node_normal: Vec<Vec3d> = vec![Vec3d::new(0., 0., 0.); n];
    for i in 0..n {
        node_pos[i] = unscaled_to_3d(&path[i].state.result_on_layer, layer_z(slicing_params, config, path[i].state.layer_idx as usize));
        node_radius[i] = unscaled_f64(support_element_radius(config, path[i]));
    }
    node_normal[0] = eigen::normalized3(&sub3d(&node_pos[1], &node_pos[0]));
    node_normal[n - 1] = eigen::normalized3(&sub3d(&node_pos[n - 1], &node_pos[n - 2]));
    for i in 1..n - 1 {
        let v1: Vec3d = eigen::normalized3(&sub3d(&node_pos[i], &node_pos[i - 1]));
        let v2: Vec3d = eigen::normalized3(&sub3d(&node_pos[i + 1], &node_pos[i]));
        node_normal[i] = eigen::normalized3(&add3d(&v1, &v2));
    }

    // Mark unsafe edges.
    let mut edge_unsafe: Vec<bool> = vec![false; n - 1];
    let mut any_unsafe = false;
    for k in 0..n - 1 {
        if section_circles_intersect(&node_pos[k], &node_normal[k], node_radius[k], &node_pos[k + 1], &node_normal[k + 1], node_radius[k + 1]) {
            edge_unsafe[k] = true;
            any_unsafe = true;
        }
    }

    if !any_unsafe {
        return extrude_branch_tube(path, config, slicing_params, move_bounds, result, TubeEndCap::HemisphereMesh, TubeEndCap::HemisphereMesh);
    }

    // Split into ordered pieces: maximal safe runs + isolated unsafe single-edge capsules.
    // Adjacent pieces share the boundary node and close with FlatDisks there; analytic
    // hemispheres are injected as 2D slices and merged by downstream clipping.
    let mut zmin: f32 = f32::MAX;
    let mut zmax: f32 = -f32::MAX;
    let mut piece_path: Vec<&SupportElement> = Vec::new();
    let mut piece_mesh = indexed_triangle_set::default();
    // Original path ends keep a 3D hemisphere. Split joints: FlatDisk + later 2D
    // inject (default), or HemisphereMesh when TREE_SUPPORT_SPLIT_CAP_MODE=mesh.
    let mesh_caps: bool = split_cap_use_mesh_hemispheres();
    let mut emit_piece = |begin: usize, end: usize| {
        piece_path.clear();
        piece_path.extend_from_slice(&path[begin..end + 1]);
        piece_mesh.vertices.clear();
        piece_mesh.indices.clear();
        let bottom_cap = if begin > 0 {
            if mesh_caps {
                TubeEndCap::HemisphereMesh
            } else {
                TubeEndCap::FlatDisk
            }
        } else {
            TubeEndCap::HemisphereMesh
        };
        let top_cap = if end < n - 1 {
            if mesh_caps {
                TubeEndCap::HemisphereMesh
            } else {
                TubeEndCap::FlatDisk
            }
        } else {
            TubeEndCap::HemisphereMesh
        };
        let span: (f32, f32) = extrude_branch_tube(&piece_path, config, slicing_params, move_bounds, &mut piece_mesh, bottom_cap, top_cap);
        // std::min(zmin, span.first), std::max(zmax, span.second)
        if span.0 < zmin {
            zmin = span.0;
        }
        if zmax < span.1 {
            zmax = span.1;
        }
        crate::triangle_mesh::its_merge(result, &piece_mesh);
        // Descriptors for analytic hemispheres at piece ends that are not the original path ends.
        if let Some(split_caps) = split_caps.as_deref_mut() {
            if end > begin {
                if begin > 0 {
                    split_caps.push(ExtrudeSplitCap {
                        center: node_pos[begin],
                        normal: eigen::normalized3(&sub3d(&node_pos[begin + 1], &node_pos[begin])),
                        radius: node_radius[begin],
                        is_bottom: true,
                    });
                }
                if end < n - 1 {
                    split_caps.push(ExtrudeSplitCap {
                        center: node_pos[end],
                        normal: eigen::normalized3(&sub3d(&node_pos[end], &node_pos[end - 1])),
                        radius: node_radius[end],
                        is_bottom: false,
                    });
                }
            }
        }
    };
    let mut run_start: usize = 0;
    for k in 0..n - 1 {
        if edge_unsafe[k] {
            if run_start < k {
                emit_piece(run_start, k); // safe run [run_start, k]
            }
            emit_piece(k, k + 1); // unsafe edge as its own capsule
            run_start = k + 1;
        }
    }
    if run_start < n - 1 {
        emit_piece(run_start, n - 1); // trailing safe run [run_start, n-1]
    }

    (zmin, zmax)
}

// =============================================================================
// The port of the pin: TreeSupport3D.cpp, the organic smoothing and the mesh of all branches
// (organic_smooth_branches_avoid_collisions() to draw_branches())
// =============================================================================

use crate::aabb_tree_lines::aabb_tree_indirect_2d as tree2d;

/// Line.hpp:228-239 `class Linef`: a line of two `Vec2d`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Linef {
    pub a: (f64, f64),
    pub b: (f64, f64),
}

/// AABBTreeLines.hpp:166-200 `build_aabb_tree_over_indexed_lines(const std::vector<LineType> &lines)` for
/// `LineType = Linef` (aabb_tree_lines.rs holds the one for `Line`): the tree is an
/// `AABBTreeIndirect::Tree<2, double>`, the centroid of a line is `(line.a + line.b) * 0.5` in doubles.
fn build_aabb_tree_over_indexed_linesf(lines: &[Linef]) -> tree2d::Tree {
    struct InputType {
        m_idx: usize,
        m_bbox: tree2d::BoundingBox,
        m_centroid: [f64; 2],
    }
    impl tree2d::SourceNode for InputType {
        fn idx(&self) -> usize {
            self.m_idx
        }
        fn centroid(&self) -> [f64; 2] {
            self.m_centroid
        }
        fn bbox(&self) -> tree2d::BoundingBox {
            self.m_bbox
        }
    }

    let mut input: Vec<InputType> = Vec::with_capacity(lines.len());
    for (i, line) in lines.iter().enumerate() {
        // n.m_centroid = (line.a + line.b) * 0.5;
        let m_centroid = [(line.a.0 + line.b.0) * 0.5, (line.a.1 + line.b.1) * 0.5];
        // n.m_bbox = BoundingBox(line.a, line.a); n.m_bbox.extend(line.b);
        let mut m_bbox = tree2d::BoundingBox::from_point([line.a.0, line.a.1]);
        m_bbox.extend_point([line.b.0, line.b.1]);
        input.push(InputType { m_idx: i, m_bbox, m_centroid });
    }

    let mut out = tree2d::Tree::new();
    out.build(input);
    out
}

/// Line.hpp:42-68 `line_alg::distance_to_squared(const L &line, const Vec<Dim<L>, Scalar<L>> &point, Vec<Dim<L>,
/// Scalar<L>> *nearest_point)` for `L = Linef`: everything in doubles, nothing truncated. The sums of products
/// are those of `Vec2d` (a packet of the two products and their sum, see `squared_norm2`).
fn distance_to_squared_linef(line: &Linef, point: (f64, f64), nearest_point: &mut (f64, f64)) -> f64 {
    let v = (line.b.0 - line.a.0, line.b.1 - line.a.1);
    let va = (point.0 - line.a.0, point.1 - line.a.1);
    let l2 = squared_norm2(v); // avoid a sqrt
    if l2 == 0.0 {
        // a == b case
        *nearest_point = line.a;
        return squared_norm2(va);
    }
    // Consider the line extending the segment, parameterized as a + t (b - a).
    // We find projection of this point onto the line.
    // It falls where t = [(this-a) . (b-a)] / |b-a|^2
    let t = dot2(va, v) / l2;
    if t <= 0.0 {
        // beyond the 'a' end of the segment
        *nearest_point = line.a;
        return squared_norm2(va);
    } else if t >= 1.0 {
        // beyond the 'b' end of the segment
        *nearest_point = line.b;
        return squared_norm2((point.0 - line.b.0, point.1 - line.b.1));
    }

    *nearest_point = (line.a.0 + t * v.0, line.a.1 + t * v.1);
    squared_norm2((t * v.0 - va.0, t * v.1 - va.1))
}

/// `Eigen::AlignedBox<double, 2>::squaredExteriorDistance(p)`, which AABBTreeIndirect.hpp:551-567
/// `squared_exterior_distance` hands on to for a box of a floating point scalar: `dist2 += aux*aux` is one
/// expression, the product is contracted into the sum.
fn squared_exterior_distance_f64(bbox: &tree2d::BoundingBox, p: [f64; 2]) -> f64 {
    let mut dist2: f64 = 0.;
    for k in 0..2 {
        if bbox.min[k] > p[k] {
            let aux = bbox.min[k] - p[k];
            dist2 = aux.mul_add(aux, dist2);
        } else if p[k] > bbox.max[k] {
            let aux = p[k] - bbox.max[k];
            dist2 = aux.mul_add(aux, dist2);
        }
    }
    dist2
}

/// AABBTreeLines.hpp:16-35 `detail::IndexedLinesDistancer<Linef, Tree<2, double>, Vec2d>`
struct IndexedLinesfDistancer<'a> {
    lines: &'a [Linef],
    tree: &'a tree2d::Tree,
    origin: (f64, f64),
}

impl IndexedLinesfDistancer<'_> {
    /// AABBTreeLines.hpp:28-34 `closest_point_to_origin(size_t primitive_index, ScalarType &squared_distance)`
    fn closest_point_to_origin(&self, primitive_index: usize, squared_distance: &mut f64) -> (f64, f64) {
        let mut nearest_point: (f64, f64) = (0., 0.);
        let line = &self.lines[primitive_index];
        *squared_distance = distance_to_squared_linef(line, self.origin, &mut nearest_point);
        nearest_point
    }
}

/// AABBTreeIndirect.hpp:569-632 `detail::squared_distance_to_indexed_primitives_recursive` for the distancer of
/// `Linef` lines.
fn squared_distance_to_indexed_primitives_recursive_linef(
    distancer: &IndexedLinesfDistancer,
    node_idx: usize,
    low_sqr_d: f64,
    mut up_sqr_d: f64,
    i: &mut usize,
    c: &mut (f64, f64),
) -> f64 {
    if low_sqr_d > up_sqr_d {
        return low_sqr_d;
    }

    let node = distancer.tree.node(node_idx);
    debug_assert!(node.is_valid());
    if node.is_leaf() {
        let mut sqr_dist: f64 = 0.;
        let c_candidate = distancer.closest_point_to_origin(node.idx, &mut sqr_dist);
        // set_min(sqr_dist, node.idx, c_candidate)
        if sqr_dist < up_sqr_d {
            *i = node.idx;
            *c = c_candidate;
            up_sqr_d = sqr_dist;
        }
    } else {
        let left_node_idx = node_idx * 2 + 1;
        let right_node_idx = left_node_idx + 1;
        let node_left = distancer.tree.node(left_node_idx);
        let node_right = distancer.tree.node(right_node_idx);
        debug_assert!(node_left.is_valid());
        debug_assert!(node_right.is_valid());

        let mut looked_left = false;
        let mut looked_right = false;
        let origin = [distancer.origin.0, distancer.origin.1];
        // look_left() / look_right(): the side, the best so far.
        let look = |child_idx: usize, up_sqr_d: &mut f64, i: &mut usize, c: &mut (f64, f64)| {
            let mut i_child: usize = 0;
            let mut c_child = *c;
            let sqr_d_child = squared_distance_to_indexed_primitives_recursive_linef(distancer, child_idx, low_sqr_d, *up_sqr_d, &mut i_child, &mut c_child);
            // set_min(sqr_d_child, i_child, c_child)
            if sqr_d_child < *up_sqr_d {
                *i = i_child;
                *c = c_child;
                *up_sqr_d = sqr_d_child;
            }
        };

        // must look left or right if in box
        if node_left.bbox.contains(origin) {
            look(left_node_idx, &mut up_sqr_d, i, c);
            looked_left = true;
        }
        if node_right.bbox.contains(origin) {
            look(right_node_idx, &mut up_sqr_d, i, c);
            looked_right = true;
        }
        // if haven't looked left and could be less than current min, then look
        let left_up_sqr_d = squared_exterior_distance_f64(&node_left.bbox, origin);
        let right_up_sqr_d = squared_exterior_distance_f64(&node_right.bbox, origin);
        if left_up_sqr_d < right_up_sqr_d {
            if !looked_left && left_up_sqr_d < up_sqr_d {
                look(left_node_idx, &mut up_sqr_d, i, c);
            }
            if !looked_right && right_up_sqr_d < up_sqr_d {
                look(right_node_idx, &mut up_sqr_d, i, c);
            }
        } else {
            if !looked_right && right_up_sqr_d < up_sqr_d {
                look(right_node_idx, &mut up_sqr_d, i, c);
            }
            if !looked_left && left_up_sqr_d < up_sqr_d {
                look(left_node_idx, &mut up_sqr_d, i, c);
            }
        }
    }
    up_sqr_d
}

/// AABBTreeLines.hpp:205-219 `squared_distance_to_indexed_lines(lines, tree, point, hit_idx_out, hit_point_out,
/// max_sqr_dist)` for `Linef` lines and a `Vec2d` point.
///
/// Finding a closest line, its closest point and squared distance to the closest point.
/// Returns squared distance to the closest point or -1 if the input is empty; `max_sqr_dist` when no point is
/// closer than that (the outputs are then left as they were).
fn squared_distance_to_indexed_linesf(
    lines: &[Linef],
    tree: &tree2d::Tree,
    point: (f64, f64),
    hit_idx_out: &mut usize,
    hit_point_out: &mut (f64, f64),
    max_sqr_dist: f64,
) -> f64 {
    if tree.empty() {
        return -1.;
    }
    let distancer = IndexedLinesfDistancer { lines, tree, origin: point };
    squared_distance_to_indexed_primitives_recursive_linef(&distancer, 0, 0., max_sqr_dist, hit_idx_out, hit_point_out)
}

/// TreeSupport3D.cpp:4177-4213 (draw_branches()) and :5458-5498 (organic_draw_branches()): all SupportElements
/// are put into a layer independent storage to improve parallelization. `elements_with_link_down` holds of
/// every element its layer and index in `move_bounds` (the pointer of the pin) and the index of its child in
/// the layer below, -1 for none; `linear_data_layers` the first index of every layer and the count.
///
/// The pin finds the child through a list sorted by the address of the parent: only one link points to a node
/// above from below (it asserts so), here the first element of the layer below that names the parent.
pub fn organic_elements_with_link_down(move_bounds: &[SupportElements]) -> (Vec<((usize, usize), i32)>, Vec<usize>) {
    let mut elements_with_link_down: Vec<((usize, usize), i32)> = Vec::new();
    let mut linear_data_layers: Vec<usize> = Vec::new();
    {
        // By the index of the element of this layer: the element below.
        let mut map_downwards_old: Vec<i32> = Vec::new();
        linear_data_layers.push(0);
        for layer_idx in 0..move_bounds.len() {
            let layer_above: Option<&SupportElements> = if layer_idx + 1 < move_bounds.len() { Some(&move_bounds[layer_idx + 1]) } else { None };
            let mut map_downwards_new: Vec<i32> = vec![-1; layer_above.map_or(0, |l| l.len())];
            let layer = &move_bounds[layer_idx];
            for (elem_idx, elem) in layer.iter().enumerate() {
                let mut child: i32 = -1;
                if layer_idx > 0 {
                    if let Some(&c) = map_downwards_old.get(elem_idx) {
                        child = c;
                    }
                }
                for &parent_idx in &elem.parents {
                    let parent: &SupportElement = &layer_above.expect("layer_above")[parent_idx as usize];
                    if parent.state.result_on_layer_is_set() && map_downwards_new[parent_idx as usize] == -1 {
                        map_downwards_new[parent_idx as usize] = elem_idx as i32;
                    }
                }

                elements_with_link_down.push(((layer_idx, elem_idx), child));
            }
            map_downwards_old = map_downwards_new;
            linear_data_layers.push(elements_with_link_down.len());
        }
    }
    (elements_with_link_down, linear_data_layers)
}

/// TreeSupport3D.cpp:3850-4061 `void organic_smooth_branches_avoid_collisions(const PrintObject &print_object, const
/// TreeModelVolumes &volumes, const TreeSupportSettings &config, const std::vector<std::pair<SupportElement*, int>>
/// &elements_with_link_down, const std::vector<size_t> &linear_data_layers, std::function<void()> throw_on_cancel)`
/// (`TREE_SUPPORT_ORGANIC_NUDGE_NEW` is defined at :51: this is the compiled one).
///
/// New version using per layer AABB trees of lines for nudging spheres away from an object.
///
/// The elements are named by layer and index, so `move_bounds` comes along. The pin moves the spheres of an
/// iteration through `tbb::parallel_for`: a sphere reads the previous positions of its neighbours and writes
/// its own position only, the order does not matter. The positions are `Vec3f`; where the pin computes in
/// doubles the floats are widened first.
pub fn organic_smooth_branches_avoid_collisions(
    print_object: &pf::PrintObject,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    move_bounds: &mut [SupportElements],
    elements_with_link_down: &[((usize, usize), i32)],
    linear_data_layers: &[usize],
    throw_on_cancel: ThrowOnCancel,
) {
    use crate::support::tree_support_common::{layer_idx_ceil, layer_idx_floor, layer_z};

    struct LayerCollisionCache {
        min_element_radius: Coord,
        #[allow(dead_code)]
        collision_radius: Coord,
        lines: Vec<Linef>,
        aabbtree_lines: tree2d::Tree,
    }
    impl LayerCollisionCache {
        fn new() -> Self {
            // coord_t min_element_radius{ std::numeric_limits<coord_t>::max() }; coord_t collision_radius{ 0 };
            Self { min_element_radius: COORD_T_MAX, collision_radius: 0, lines: Vec::new(), aabbtree_lines: tree2d::Tree::new() }
        }
        fn min_element_radius_known(&self) -> bool {
            self.min_element_radius != COORD_T_MAX
        }
        fn empty(&self) -> bool {
            self.lines.is_empty()
        }
    }
    let mut layer_collision_cache: Vec<LayerCollisionCache> = Vec::with_capacity(1024);
    let slicing_params = print_object.slicing_parameters();
    for element in elements_with_link_down {
        let elem: &SupportElement = &move_bounds[element.0 .0][element.0 .1];
        let layer_idx: LayerIndex = elem.state.layer_idx;
        let num_layers = (layer_idx + 1) as usize;
        if num_layers > layer_collision_cache.len() {
            if num_layers > layer_collision_cache.capacity() {
                let cap = crate::utils::next_highest_power_of_2(num_layers);
                layer_collision_cache.reserve(cap - layer_collision_cache.len());
            }
            layer_collision_cache.resize_with(num_layers, LayerCollisionCache::new);
        }
        let l = &mut layer_collision_cache[layer_idx as usize];
        l.min_element_radius = std::cmp::min(l.min_element_radius, support_element_radius(config, elem));
    }

    throw_on_cancel();

    for layer_idx in 0..layer_collision_cache.len() as LayerIndex {
        let l = &mut layer_collision_cache[layer_idx as usize];
        if !l.min_element_radius_known() {
            l.min_element_radius = 0;
        } else {
            //FIXME
            l.min_element_radius = 0;
            // `assert(res.has_value());`: the collision of radius zero is precalculated for every layer with elements.
            let res: (Coord, crate::support::tree_model_volumes::PolygonsRef) =
                volumes.get_collision_lower_bound_area(layer_idx, l.min_element_radius).expect("collision area of the layer");
            l.collision_radius = res.0;
            // Lines alines = to_lines(res->second.get());
            l.lines.reserve(res.1.iter().map(|p| p.points.len()).sum());
            // unscaled<double>(line.a), unscaled<double>(line.b)
            let un = |p: &Point| (p.x as f64 * SCALING_FACTOR, p.y as f64 * SCALING_FACTOR);
            for poly in res.1.iter() {
                let n = poly.points.len();
                for k in 0..n.saturating_sub(1) {
                    l.lines.push(Linef { a: un(&poly.points[k]), b: un(&poly.points[k + 1]) });
                }
                l.lines.push(Linef { a: un(&poly.points[n - 1]), b: un(&poly.points[0]) });
            }
            l.aabbtree_lines = build_aabb_tree_over_indexed_linesf(&l.lines);
            throw_on_cancel();
        }
    }

    struct CollisionSphere {
        element: (usize, usize),
        element_below_id: i32,
        locked: bool,
        radius: f32,
        // Current position, when nudged away from the collision.
        position: Vec3f,
        // Previous position, for Laplacian smoothing.
        prev_position: Vec3f,
        //
        last_collision: Vec3f,
        last_collision_depth: f64,
        // Minimum Z for which the sphere collision will be evaluated.
        // Limited by the minimum sloping angle and by the bottom of the tree.
        min_z: f32,
        // Maximum Z for which the sphere collision will be evaluated.
        // Limited by the minimum sloping angle and by the tip of the current branch.
        max_z: f32,
        layer_begin: u32,
        layer_end: u32,
    }

    let mut collision_spheres: Vec<CollisionSphere> = Vec::with_capacity(elements_with_link_down.len());
    for element_with_link in elements_with_link_down {
        let element: &SupportElement = &move_bounds[element_with_link.0 .0][element_with_link.0 .1];
        let link_down: i32 = element_with_link.1;
        // to_3d(unscaled<float>(element.state.result_on_layer), float(layer_z(slicing_params, config, element.state.layer_idx)))
        let position = Vec3f::new(
            unscaled_f32(element.state.result_on_layer.x),
            unscaled_f32(element.state.result_on_layer.y),
            layer_z(slicing_params, config, element.state.layer_idx as usize) as f32,
        );
        collision_spheres.push(CollisionSphere {
            element: element_with_link.0,
            element_below_id: link_down,
            // locked
            locked: element.parents.is_empty() || (link_down == -1 && element.state.layer_idx > 0),
            radius: unscaled_f32(support_element_radius(config, element)),
            // 3D position
            position,
            // The pin leaves the next two as they are (not read before they are written).
            prev_position: Vec3f::new(0., 0., 0.),
            last_collision: Vec3f::new(0., 0., 0.),
            last_collision_depth: 0.,
            min_z: -f32::MAX,
            max_z: f32::MAX,
            layer_begin: 0,
            layer_end: 0,
        });
        // Update min_z coordinate to min_z of the tree below.
        let last = collision_spheres.len() - 1;
        if link_down != -1 {
            let offset_below: usize = linear_data_layers[(element.state.layer_idx - 1) as usize];
            collision_spheres[last].min_z = collision_spheres[offset_below + link_down as usize].min_z;
        } else {
            collision_spheres[last].min_z = collision_spheres[last].position.z;
        }
    }
    // Update max_z by propagating max_z from the tips of the branches.
    for collision_sphere_id in (0..collision_spheres.len()).rev() {
        let element: &SupportElement = &move_bounds[collision_spheres[collision_sphere_id].element.0][collision_spheres[collision_sphere_id].element.1];
        if element.parents.is_empty() {
            // Tip
            collision_spheres[collision_sphere_id].max_z = collision_spheres[collision_sphere_id].position.z;
        } else {
            // Below tip
            let offset_above: usize = linear_data_layers[(element.state.layer_idx + 1) as usize];
            for &iparent in &element.parents {
                let parent_z: f32 = collision_spheres[offset_above + iparent as usize].max_z;
                // collision_sphere.max_z = std::min(collision_sphere.max_z, parent_z);
                if parent_z < collision_spheres[collision_sphere_id].max_z {
                    collision_spheres[collision_sphere_id].max_z = parent_z;
                }
            }
        }
    }
    // Update min_z / max_z to limit the search Z span of a given sphere for collision detection.
    for collision_sphere in collision_spheres.iter_mut() {
        let layer_idx: LayerIndex = move_bounds[collision_sphere.element.0][collision_sphere.element.1].state.layer_idx;
        //FIXME limit the collision span by the tree slope.
        // std::max(min_z, position.z() - radius), std::min(max_z, position.z() + radius): of floats.
        let lo: f32 = collision_sphere.position.z - collision_sphere.radius;
        if collision_sphere.min_z < lo {
            collision_sphere.min_z = lo;
        }
        let hi: f32 = collision_sphere.position.z + collision_sphere.radius;
        if hi < collision_sphere.max_z {
            collision_sphere.max_z = hi;
        }
        collision_sphere.layer_begin = std::cmp::min(layer_idx, layer_idx_ceil(slicing_params, config, collision_sphere.min_z as f64)) as u32;
        debug_assert!((collision_sphere.layer_begin as usize) < layer_collision_cache.len());
        collision_sphere.layer_end = std::cmp::min(
            layer_collision_cache.len() as LayerIndex,
            std::cmp::max(layer_idx, layer_idx_floor(slicing_params, config, collision_sphere.max_z as f64)) + 1,
        ) as u32;
    }

    throw_on_cancel();

    const COLLISION_EXTRA_GAP: f64 = 0.1;
    const MAX_NUDGE_COLLISION_AVOIDANCE: f64 = 0.5;
    const MAX_NUDGE_SMOOTHING: f64 = 0.2;
    const NUM_ITER: usize = 100; // 1000;
    for _iter in 0..NUM_ITER {
        // Back up prev position before Laplacian smoothing.
        for collision_sphere in collision_spheres.iter_mut() {
            collision_sphere.prev_position = collision_sphere.position;
        }
        let mut num_moved: usize = 0;
        for collision_sphere_id in 0..collision_spheres.len() {
            if collision_spheres[collision_sphere_id].locked {
                continue;
            }
            let element: &SupportElement = &move_bounds[collision_spheres[collision_sphere_id].element.0][collision_spheres[collision_sphere_id].element.1];
            let layer_idx: LayerIndex = element.state.layer_idx;
            // Laplacian smoothing reads the previous positions of the neighbours: collected before the sphere is borrowed.
            let offset_above: usize = linear_data_layers[(layer_idx + 1) as usize];
            let parents_prev: Vec<(f64, f64)> = element
                .parents
                .iter()
                .map(|&iparent| {
                    let p = &collision_spheres[offset_above + iparent as usize].prev_position;
                    (p.x as f64, p.y as f64)
                })
                .collect();
            let below_prev: Option<(f64, f64)> = if collision_spheres[collision_sphere_id].element_below_id != -1 {
                let offset_below: usize = linear_data_layers[(layer_idx - 1) as usize];
                let p = &collision_spheres[offset_below + collision_spheres[collision_sphere_id].element_below_id as usize].prev_position;
                Some((p.x as f64, p.y as f64))
            } else {
                None
            };
            let collision_sphere = &mut collision_spheres[collision_sphere_id];

            // Calculate collision of multiple 2D layers against a collision sphere.
            collision_sphere.last_collision_depth = -f64::MAX;
            let mut layer_id: u32 = collision_sphere.layer_begin;
            while layer_id != collision_sphere.layer_end {
                if layer_id as usize >= layer_collision_cache.len() {
                    layer_id = layer_id.wrapping_add(1);
                    continue;
                }
                // `(layer_id - collision_sphere.element.state.layer_idx) * slicing_params.layer_height`: the
                // difference of an `uint32_t` and an `int` is unsigned, it wraps for the layers below the
                // element (dz is then about 4e9 layers and the layer is passed over).
                let dz: f64 = layer_id.wrapping_sub(layer_idx as u32) as f64 * slicing_params.layer_height;
                // sqr(collision_sphere.radius) - sqr(dz): the square of the float, then doubles.
                let r2: f64 = (collision_sphere.radius * collision_sphere.radius) as f64 - dz * dz;
                if r2 > 0. {
                    let layer_collision_cache_item = &layer_collision_cache[layer_id as usize];
                    if !layer_collision_cache_item.empty() {
                        let mut hit_idx_out: usize = 0;
                        // Not initialized at the pin: when no line is closer than the radius the distance comes
                        // back as r2, the depth is zero and the point is stored but never used.
                        let mut hit_point_out: (f64, f64) = (0., 0.);
                        let dist: f64 = squared_distance_to_indexed_linesf(
                            &layer_collision_cache_item.lines,
                            &layer_collision_cache_item.aabbtree_lines,
                            (collision_sphere.position.x as f64, collision_sphere.position.y as f64),
                            &mut hit_idx_out,
                            &mut hit_point_out,
                            r2,
                        )
                        .sqrt();
                        if dist >= 0. {
                            let collision_depth: f64 = r2.sqrt() - dist;
                            if collision_depth > collision_sphere.last_collision_depth {
                                collision_sphere.last_collision_depth = collision_depth;
                                collision_sphere.last_collision =
                                    Vec3f::new(hit_point_out.0 as f32, hit_point_out.1 as f32, layer_z(slicing_params, config, layer_id as usize) as f32);
                            }
                        }
                    }
                }
                layer_id = layer_id.wrapping_add(1);
            }
            if collision_sphere.last_collision_depth > 0. {
                // Collision detected to be removed.
                // Nudge the circle center away from the collision.
                if collision_sphere.last_collision_depth > EPSILON {
                    // a little bit of hysteresis to detect end of
                    num_moved += 1;
                }
                // Shift by maximum 2mm.
                // std::min(std::max(0., last_collision_depth + collision_extra_gap), max_nudge_collision_avoidance)
                let d = collision_sphere.last_collision_depth + COLLISION_EXTRA_GAP;
                let d = if 0. < d { d } else { 0. };
                let nudge_dist: f64 = if MAX_NUDGE_COLLISION_AVOIDANCE < d { MAX_NUDGE_COLLISION_AVOIDANCE } else { d };
                // (to_2d(position) - to_2d(last_collision)).cast<double>().normalized() * nudge_dist
                let diff: (f64, f64) = (
                    (collision_sphere.position.x - collision_sphere.last_collision.x) as f64,
                    (collision_sphere.position.y - collision_sphere.last_collision.y) as f64,
                );
                let n = normalized2(diff);
                let nudge_vector: (f64, f64) = (n.0 * nudge_dist, n.1 * nudge_dist);
                // collision_sphere.position.head<2>() += (nudge_vector * nudge_dist).cast<float>();
                collision_sphere.position.x += (nudge_vector.0 * nudge_dist) as f32;
                collision_sphere.position.y += (nudge_vector.1 * nudge_dist) as f32;
            }
            // Laplacian smoothing
            let mut avg: (f64, f64) = (0., 0.);
            let mut weight: f64 = 0.;
            for prev in &parents_prev {
                let w: f64 = collision_sphere.radius as f64;
                avg.0 += w * prev.0;
                avg.1 += w * prev.1;
                weight += w;
            }
            if let Some(prev) = below_prev {
                let w: f64 = weight;
                avg.0 += w * prev.0;
                avg.1 += w * prev.1;
                weight += w;
            }
            avg.0 /= weight;
            avg.1 /= weight;
            const SMOOTHING_FACTOR: f64 = 0.5;
            let old_pos: (f64, f64) = (collision_sphere.position.x as f64, collision_sphere.position.y as f64);
            let new_pos: (f64, f64) =
                ((1. - SMOOTHING_FACTOR) * old_pos.0 + SMOOTHING_FACTOR * avg.0, (1. - SMOOTHING_FACTOR) * old_pos.1 + SMOOTHING_FACTOR * avg.1);
            let shift: (f64, f64) = (new_pos.0 - old_pos.0, new_pos.1 - old_pos.1);
            let nudge_dist_max: f64 = norm2(shift);
            // Shift by maximum 1mm, less than the collision avoidance factor.
            // std::min(std::max(0., nudge_dist_max), max_nudge_smoothing)
            let d = if 0. < nudge_dist_max { nudge_dist_max } else { 0. };
            let nudge_dist: f64 = if MAX_NUDGE_SMOOTHING < d { MAX_NUDGE_SMOOTHING } else { d };
            let n = normalized2(shift);
            collision_sphere.position.x += (n.0 * nudge_dist) as f32;
            collision_sphere.position.y += (n.1 * nudge_dist) as f32;

            throw_on_cancel();
        }
        if num_moved == 0 {
            break;
        }
    }

    for i in 0..collision_spheres.len() {
        // scaled<coord_t>(to_2d(collision_spheres[i].position)): `(v / SCALING_FACTOR).cast<coord_t>()` of a
        // vector of floats, the factor taken as a float; the cast to `int32_t` saturates.
        let p = &collision_spheres[i].position;
        let e = elements_with_link_down[i].0;
        move_bounds[e.0][e.1].state.result_on_layer =
            Point::new((p.x / SCALING_FACTOR as f32) as i32 as Coord, (p.y / SCALING_FACTOR as f32) as i32 as Coord);
    }
}

// `Vec2d::normalized()` (Dot.h: `n / sqrt(z)` when `z > 0`, else a copy).
#[inline]
fn normalized2(v: (f64, f64)) -> (f64, f64) {
    let z = squared_norm2(v);
    if z > 0. {
        let s = z.sqrt();
        (v.0 / s, v.1 / s)
    } else {
        v
    }
}

/// TreeSupport3D.cpp:4167-4285 `indexed_triangle_set draw_branches(PrintObject &print_object, const TreeModelVolumes
/// &volumes, const TreeSupportSettings &config, std::vector<SupportElements> &move_bounds, std::function<void()>
/// throw_on_cancel)`
///
/// Organic specific: Smooth branches and produce one cummulative mesh to be sliced.
pub fn draw_branches(
    print_object: &pf::PrintObject,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    move_bounds: &mut [SupportElements],
    throw_on_cancel: ThrowOnCancel,
) -> indexed_triangle_set {
    // All SupportElements are put into a layer independent storage to improve parallelization.
    let (elements_with_link_down, linear_data_layers) = organic_elements_with_link_down(move_bounds);

    throw_on_cancel();

    organic_smooth_branches_avoid_collisions(print_object, volumes, config, move_bounds, &elements_with_link_down, &linear_data_layers, throw_on_cancel);

    // Unmark all nodes.
    for elements in move_bounds.iter_mut() {
        for element in elements.iter_mut() {
            element.state.marked = false;
        }
    }

    // Traverse all nodes, generate tubes.
    // Traversal stack with nodes and thier current parent
    let slicing_params = print_object.slicing_parameters();
    let mut path: Vec<(usize, usize)> = Vec::new();
    let mut cummulative_mesh = indexed_triangle_set::default();
    let mut partial_mesh = indexed_triangle_set::default();
    let mut layer_idx: usize = 0;
    while layer_idx + 1 < move_bounds.len() {
        for start_idx in 0..move_bounds[layer_idx].len() {
            let start_parents: usize = move_bounds[layer_idx][start_idx].parents.len();
            if !move_bounds[layer_idx][start_idx].state.marked && start_parents != 0 {
                // Collect elements up to a bifurcation above.
                move_bounds[layer_idx][start_idx].state.marked = true;
                for parent_idx in 0..start_parents {
                    path.clear();
                    path.push((layer_idx, start_idx));
                    // Traverse each branch until it branches again.
                    let first_parent: (usize, usize) = (layer_idx + 1, move_bounds[layer_idx][start_idx].parents[parent_idx] as usize);
                    path.push(first_parent);
                    let first_parent_parents: usize = move_bounds[first_parent.0][first_parent.1].parents.len();
                    if first_parent_parents < 2 {
                        move_bounds[first_parent.0][first_parent.1].state.marked = true;
                    }
                    if first_parent_parents == 1 {
                        let mut parent: (usize, usize) = first_parent;
                        loop {
                            let p: &SupportElement = &move_bounds[parent.0][parent.1];
                            let next_parent: (usize, usize) = ((p.state.layer_idx + 1) as usize, p.parents[0] as usize);
                            path.push(next_parent);
                            let next_parent_parents: usize = move_bounds[next_parent.0][next_parent.1].parents.len();
                            if next_parent_parents > 1 {
                                break;
                            }
                            move_bounds[next_parent.0][next_parent.1].state.marked = true;
                            if next_parent_parents == 0 {
                                break;
                            }
                            parent = next_parent;
                        }
                    }
                    // Triangulate the tube.
                    partial_mesh.vertices.clear();
                    partial_mesh.indices.clear();
                    {
                        let path_elements: Vec<&SupportElement> = path.iter().map(|&(l, i)| &move_bounds[l][i]).collect();
                        extrude_branch(&path_elements, config, slicing_params, move_bounds, &mut partial_mesh, None);
                    }
                    crate::triangle_mesh::its_merge(&mut cummulative_mesh, &partial_mesh);
                }
                throw_on_cancel();
            }
        }
        layer_idx += 1;
    }
    cummulative_mesh
}

// =============================================================================
// The port of the pin: TreeSupport3D.cpp, the slices of the branches (slice_branches(), the helpers of
// organic_draw_branches() and organic_draw_branches() itself)
// =============================================================================

// ClipperUtils.cpp `Polygons intersection_clipped(const Polygons &subject, const Polygons &clip, ApplySafetyOffset
// do_safety_offset)`: the intersection with the clip polygons clipped by the bounding box of the subject.
fn intersection_clipped(subject: &[Polygon], clip: &[Polygon]) -> Polygons {
    // get_extents(subject).inflated(SCALED_EPSILON)
    let mut bbox = crate::geometry::get_extents_polygons(subject);
    if !bbox.is_defined() {
        // The intersection of nothing.
        return Vec::new();
    }
    let eps = SCALED_EPSILON as Coord;
    bbox.min = Point::new(bbox.min.x - eps, bbox.min.y - eps);
    bbox.max = Point::new(bbox.max.x + eps, bbox.max.y + eps);
    cu::intersection(subject, &crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_polygons(clip, &bbox), cu::ApplySafetyOffset::No)
}

// Polygon.cpp:729-746 `make_circle(double radius, double error)`: the cosine and the sine of an angle are one
// call in the pin's build (`crate::clipper::sin_cos`), the coordinates are truncated.
fn make_circle_pin(radius: f64, error: f64) -> Polygon {
    let angle: f64 = 2. * (1. - error / radius).acos();
    let num_segments: usize = (2. * std::f64::consts::PI / angle).ceil() as usize;
    let mut out = Polygon::new();
    out.points.reserve(num_segments);
    let angle_inc: f64 = 2.0 * std::f64::consts::PI / num_segments as f64;
    for i in 0..num_segments {
        let angle: f64 = angle_inc * i as f64;
        let (s, c) = crate::clipper::sin_cos(angle);
        out.points.push(Point::new((c * radius) as i32 as Coord, (s * radius) as i32 as Coord));
    }
    out
}

/// TreeSupport3D.cpp:4287-4348 `void slice_branches(PrintObject &print_object, const TreeModelVolumes &volumes, const
/// TreeSupportSettings &config, const std::vector<Polygons> &overhangs, std::vector<SupportElements> &move_bounds,
/// const indexed_triangle_set &cummulative_mesh, SupportGeneratorLayersPtr &bottom_contacts,
/// SupportGeneratorLayersPtr &top_contacts, SupportGeneratorLayersPtr &intermediate_layers,
/// SupportGeneratorLayerStorage &layer_storage, std::function<void()> throw_on_cancel)`
///
/// Organic specific: Slice the cummulative mesh produced by draw_branches(). (The pin does not call it:
/// generate_support_areas() draws the branches with organic_draw_branches().)
#[allow(clippy::too_many_arguments)]
pub fn slice_branches(
    print_object: &pf::PrintObject,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    overhangs: &[Polygons],
    move_bounds: &mut [SupportElements],
    cummulative_mesh: &indexed_triangle_set,
    bottom_contacts: &mut SupportGeneratorLayersPtr,
    top_contacts: &mut SupportGeneratorLayersPtr,
    intermediate_layers: &mut SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
    throw_on_cancel: ThrowOnCancel,
) {
    use crate::triangle_mesh_slicer::{slice_mesh_ex_its_identity, MeshSlicingParamsEx, SlicingMode};

    let slicing_params = print_object.slicing_parameters().clone();
    let mut slice_z: Vec<f32> = Vec::new();
    for layer_idx in 0..move_bounds.len() {
        // `object_print_z_min + first_object_layer_height + layer_idx * layer_height`: the product is fused into
        // the last addition.
        let print_z: f64 = (layer_idx as f64).mul_add(slicing_params.layer_height, slicing_params.object_print_z_min + slicing_params.first_object_layer_height);
        let layer_height: f64 = if layer_idx == 0 { slicing_params.first_object_layer_height } else { slicing_params.layer_height };
        slice_z.push((-layer_height).mul_add(0.5, print_z) as f32);
    }
    // Remove the trailing slices.
    while !slice_z.is_empty() {
        if move_bounds[slice_z.len() - 1].is_empty() {
            slice_z.pop();
        } else {
            break;
        }
    }

    let mut params = MeshSlicingParamsEx::default();
    params.closing_radius = cfg_float(print_object.config(), "slice_closing_radius") as f32;
    params.base.mode = SlicingMode::Positive;
    let mut slices: Vec<Vec<ExPolygon>> = slice_mesh_ex_its_identity(cummulative_mesh, &slice_z, &params, &|| throw_on_cancel());
    for layer_idx in 0..slice_z.len() {
        if !slices[layer_idx].is_empty() {
            if intermediate_layers[layer_idx] == SUPPORT_GENERATOR_LAYER_PTR_NULL {
                intermediate_layers[layer_idx] = layer_allocate(layer_storage, SupporLayerType::SltBase, &slicing_params, config, layer_idx);
            }
            let l = intermediate_layers[layer_idx];
            let expolygons: Vec<ExPolygon> = std::mem::take(&mut slices[layer_idx]);
            layer_storage[l].polygons.extend(crate::geometry::to_polygons(&expolygons));
        }
    }

    // Trim the slices.
    for layer_idx in 0..intermediate_layers.len() {
        let layer = intermediate_layers[layer_idx];
        if layer != SUPPORT_GENERATOR_LAYER_PTR_NULL {
            let poly: Polygons = diff_clipped(&layer_storage[layer].polygons, &volumes.get_collision(0, layer_idx as LayerIndex, true));
            layer_storage[layer].polygons = cu::intersection(&poly, std::slice::from_ref(&volumes.m_bed_area), cu::ApplySafetyOffset::No);
        }
    }

    let mut support_layer_storage: Vec<Polygons> = vec![Vec::new(); move_bounds.len()];
    let mut support_roof_storage: Vec<Polygons> = vec![Vec::new(); move_bounds.len()];
    finalize_interface_and_support_areas(
        print_object,
        volumes,
        config,
        overhangs,
        &mut support_layer_storage,
        &mut support_roof_storage,
        bottom_contacts,
        top_contacts,
        intermediate_layers,
        layer_storage,
        throw_on_cancel,
    );
}

/// TreeSupport3D.cpp:5149-5234 `static void append_organic_plate_foot_slices(const std::vector<const SupportElement*>
/// &path, const TreeSupportSettings &config, const LayerIndex layer_begin, std::vector<Polygons> &slices)`
///
/// Widen the single plate trunk in 2D after the tube is sliced.
/// Extrude still uses the original branch radius, so self-intersection repair hemispheres
/// do not grow.
///
/// Soft foot (not a hard push to bp_radius):
///   r_plate = min(bp_radius, r_trunk + brim_extra)
/// bp_radius stays the hard cap (legacy 7.5mm diameter / 3.75mm radius). Thin trunks get
/// trunk+brim; thick trunks already near the cap get little or no extra flare.
/// Profile uses ease-out so the flare is flatter near the trunk and opens toward the plate.
/// Mean radial flare angle is limited to ~12deg (gentler than the legacy ~40deg
/// bp_radius_increase). The quadratic ease-out is steepest at the plate and becomes
/// tangent to the trunk at the blend point.
/// Short trunks shrink the flare instead of exceeding the mean angle limit.
///
/// Circles merge with the tube via downstream non-zero union in diff_clipped()/intersection(),
/// so a circle smaller than the tube never carves into it. Same getCollision(0,..,min_xy_dist)
/// + bed clip as the tube: intentional crescent trim / neighbour merge near the model.
pub fn append_organic_plate_foot_slices(path: &[&SupportElement], config: &TreeSupportSettings, layer_begin: LayerIndex, slices: &mut [Polygons]) {
    use std::f64::consts::PI as M_PI;

    if path.len() < 2 || slices.is_empty() {
        return;
    }
    let root: &SupportElement = path[0];
    if !root.state.to_buildplate || !root.state.result_on_layer_is_set() {
        return;
    }

    let r_root: Coord = support_element_radius(config, root);
    let r_cap: Coord = config.bp_radius;
    if r_root <= 0 || r_cap <= 0 || r_root >= r_cap {
        return;
    }

    // Fixed radial extension beyond the blend-in trunk radius; hard-capped by bp_radius.
    const K_BRIM_MM: f64 = 1.2;
    const K_MEAN_FLARE_ANGLE_RAD: f64 = 12. * M_PI / 180.;
    // scaled<coord_t>(k_brim_mm)
    let brim_extra: Coord = (K_BRIM_MM / SCALING_FACTOR) as i32 as Coord;
    // config.layer_height is already scaled, matching bp_radius_increase_per_layer units.
    let mean_flare_per_layer: f64 = K_MEAN_FLARE_ANGLE_RAD.tan() * config.layer_height as f64;
    if mean_flare_per_layer <= 0. || brim_extra <= 0 {
        return;
    }

    let max_flare: Coord = std::cmp::min(brim_extra, (r_cap - r_root) as i32 as Coord);
    if max_flare <= 0 {
        return;
    }
    // Upper bound on cone height from the gentler slope (not the legacy ~40deg).
    let layers_needed: usize = (max_flare as f64 / mean_flare_per_layer).ceil() as usize;
    if layers_needed == 0 {
        return;
    }
    let top: usize = std::cmp::min(layers_needed, path.len() - 1);
    if top == 0 {
        return;
    }

    let el_top: &SupportElement = path[top];
    let r_trunk: Coord = support_element_radius(config, el_top);
    let z0: LayerIndex = root.state.layer_idx;
    let z1: LayerIndex = el_top.state.layer_idx;
    if z1 <= z0 {
        return;
    }

    // Relative to trunk at the blend point; never force a full 3.75mm when trunk+brim is enough.
    let mut r_plate: Coord = std::cmp::min(r_cap, (r_trunk + brim_extra) as i32 as Coord);
    // Keep the mean radial flare angle within the limit: short trunks reduce the flare.
    // coord_t(std::lround(double(r_trunk) + double(z1 - z0) * mean_flare_per_layer)): the product is fused.
    r_plate = std::cmp::min(r_plate, ((z1 - z0) as f64).mul_add(mean_flare_per_layer, r_trunk as f64).round() as i64 as i32 as Coord);
    if r_plate <= r_root || r_plate <= r_trunk {
        return;
    }

    for k in 0..=top {
        let el: &SupportElement = path[k];
        if !el.state.result_on_layer_is_set() {
            continue;
        }
        let si: i32 = el.state.layer_idx - layer_begin;
        if si < 0 || si >= slices.len() as i32 {
            continue;
        }
        // t=0 at plate, t=1 at trunk blend. Ease-out: s = 1-(1-t)^2 → flatter near the trunk.
        let t: f64 = (el.state.layer_idx - z0) as f64 / (z1 - z0) as f64;
        // `1. - (1. - t) * (1. - t)`: the product is fused into the difference.
        let s: f64 = (-(1. - t)).mul_add(1. - t, 1.);
        // coord_t(std::lround(double(r_plate) + s * double(r_trunk - r_plate))): the product is fused.
        let target: Coord = s.mul_add((r_trunk - r_plate) as f64, r_plate as f64).round() as i64 as i32 as Coord;
        if target <= 0 {
            continue;
        }
        // make_circle(target, std::max(double(target) / 100., 1.))
        let error: f64 = target as f64 / 100.;
        let mut circle: Polygon = make_circle_pin(target as f64, if error < 1. { 1. } else { error });
        circle.translate(el.state.result_on_layer);
        slices[si as usize].push(circle);
    }
}

/// TreeSupport3D.cpp:5236-5245 `static int hemisphere_circle_nsteps(double radius)`
///
/// Same sagitta as discretize_circle (eps=0.015). nsteps is taken from the full
/// hemisphere radius so every layer of one cap uses an identical vertex count.
pub fn hemisphere_circle_nsteps(radius: f64) -> i32 {
    use std::f64::consts::PI as M_PI;
    const EPS: f64 = 0.015;
    // std::max(radius, eps)
    let r: f64 = if radius < EPS { EPS } else { radius };
    // std::clamp(1. - eps / r, -1., 1.)
    let c: f64 = 1. - EPS / r;
    let c: f64 = if c < -1. {
        -1.
    } else if 1. < c {
        1.
    } else {
        c
    };
    let angle_step: f64 = 2. * c.acos();
    // std::max(3, int(std::ceil(2. * M_PI / std::max(angle_step, 1e-6))))
    let n: i32 = (2. * M_PI / if angle_step < 1e-6 { 1e-6 } else { angle_step }).ceil() as i32;
    std::cmp::max(3, n)
}

/// TreeSupport3D.cpp:5247-5260 `static void hemisphere_disk_xy_basis(const Vec3d &normal, Vec2d &x2, Vec2d &y2)`
///
/// XY basis for make_oriented_disk_xy / prepared caps. Matches discretize_circle's
/// local x-axis (n × (0,-1,0)) projected to XY, so phase is constant across layers.
pub fn hemisphere_disk_xy_basis(normal: &Vec3d, x2: &mut (f64, f64), y2: &mut (f64, f64)) {
    let mut x3: Vec3d = eigen::cross3(normal, &Vec3d::new(0., -1., 0.));
    if eigen::squared_norm3(&x3) < 1e-12 {
        x3 = eigen::cross3(normal, &Vec3d::new(1., 0., 0.));
    }
    *x2 = (x3.x, x3.y);
    if squared_norm2(*x2) < 1e-12 {
        *x2 = (1., 0.);
    } else {
        // x2.normalize()
        *x2 = normalized2(*x2);
    }
    *y2 = (-x2.1, x2.0);
}

/// TreeSupport3D.cpp:5262-5276 `static Polygon make_oriented_disk_xy(const Vec3d &center, const Vec2d &x2, const Vec2d
/// &y2, double rho, int nsteps)`
///
/// Horizontal n-gon of radius rho using a precomputed XY basis.
pub fn make_oriented_disk_xy(center: &Vec3d, x2: &(f64, f64), y2: &(f64, f64), rho: f64, nsteps: i32) -> Polygon {
    use std::f64::consts::PI as M_PI;
    let mut poly = Polygon::new();
    poly.points.reserve(nsteps.max(0) as usize);
    let da: f64 = 2. * M_PI / nsteps as f64;
    for i in 0..nsteps {
        let a: f64 = da * i as f64;
        // The cosine and the sine of `a`: one call.
        let (sin_a, cos_a) = crate::clipper::sin_cos(a);
        // `center.x() + rho * (x2.x() * std::cos(a) + y2.x() * std::sin(a))`: the left product of the sum is fused
        // into it, the product with rho into the outer sum.
        let px: f64 = rho.mul_add(x2.0.mul_add(cos_a, y2.0 * sin_a), center.x);
        let py: f64 = rho.mul_add(x2.1.mul_add(cos_a, y2.1 * sin_a), center.y);
        // scaled<coord_t>(px), scaled<coord_t>(py)
        poly.points.push(Point::new((px / SCALING_FACTOR) as i32 as Coord, (py / SCALING_FACTOR) as i32 as Coord));
    }
    poly
}

/// TreeSupport3D.cpp:5278-5321 `static Polygon clip_convex_polygon_halfplane(const Polygon &in, double ax, double ay,
/// double c)`
///
/// Convex clip: keep ax*x + ay*y <= c (unscaled XY). Inserts edge-line hits so the
/// cut does not go through Clipper's large half-plane quad.
pub fn clip_convex_polygon_halfplane(in_: &Polygon, ax: f64, ay: f64, c: f64) -> Polygon {
    if in_.points.len() < 3 {
        return Polygon::new();
    }
    // `ax * unscaled<double>(p.x()) + ay * unscaled<double>(p.y()) - c`: `unscaled` is a call; the first product
    // is fused into the sum of the two, the difference is plain.
    let side = |p: &Point| -> f64 { ax.mul_add(unscaled_f64(p.x), ay * unscaled_f64(p.y)) - c };
    let hit = |p: &Point, q: &Point, sp: f64, sq: f64| -> Point {
        // Inside is sp <= EPSILON, so sp may be slightly positive here and sp - sq
        // arbitrarily small. Clamp to keep the hit on the segment; t == 0 then yields p,
        // which is exactly the intended EPSILON overlap past the clip line.
        // std::clamp(sp / (sp - sq), 0., 1.)
        let t: f64 = sp / (sp - sq);
        let t: f64 = if t < 0. {
            0.
        } else if 1. < t {
            1.
        } else {
            t
        };
        // `unscaled<double>(p.x()) + t * (unscaled<double>(q.x()) - unscaled<double>(p.x()))`: fused.
        let x: f64 = t.mul_add(unscaled_f64(q.x) - unscaled_f64(p.x), unscaled_f64(p.x));
        let y: f64 = t.mul_add(unscaled_f64(q.y) - unscaled_f64(p.y), unscaled_f64(p.y));
        // Point(scaled<coord_t>(x), scaled<coord_t>(y))
        Point::new((x / SCALING_FACTOR) as i32 as Coord, (y / SCALING_FACTOR) as i32 as Coord)
    };

    let mut out = Polygon::new();
    out.points.reserve(in_.points.len() + 2);
    let n: usize = in_.points.len();
    for i in 0..n {
        let p: &Point = &in_.points[i];
        let q: &Point = &in_.points[(i + 1) % n];
        let sp: f64 = side(p);
        let sq: f64 = side(q);
        let pin: bool = sp <= EPSILON;
        let qin: bool = sq <= EPSILON;
        if pin && qin {
            out.points.push(*q);
        } else if pin && !qin {
            out.points.push(hit(p, q, sp, sq));
        } else if !pin && qin {
            out.points.push(hit(p, q, sp, sq));
            out.points.push(*q);
        }
    }
    if out.points.len() < 3 {
        return Polygon::new();
    }
    if out.area() < 0. {
        out.make_counter_clockwise();
    }
    out
}

/// TreeSupport3D.cpp:5323-5360 `static bool hemisphere_slice_polygon(const Vec3d &center, const Vec3d &normal, double
/// radius, bool is_bottom, double slice_z, int nsteps, const Vec2d &x2, const Vec2d &y2, Polygon &out)`
///
/// Horizontal cross-section of a hemisphere: disk of radius sqrt(r^2-dz^2) clipped to the
/// half-plane of the section. Used to inject 61563 split-joint caps into production slices.
#[allow(clippy::too_many_arguments)]
pub fn hemisphere_slice_polygon(
    center: &Vec3d,
    normal: &Vec3d,
    radius: f64,
    is_bottom: bool,
    slice_z: f64,
    nsteps: i32,
    x2: &(f64, f64),
    y2: &(f64, f64),
    out: &mut Polygon,
) -> bool {
    let dz: f64 = slice_z - center.z;
    if dz.abs() >= radius - EPSILON {
        return false;
    }

    // std::sqrt(std::max(0., radius * radius - dz * dz)): the left product is fused into the difference.
    let d: f64 = radius.mul_add(radius, -(dz * dz));
    let rho: f64 = (if 0. < d { d } else { 0. }).sqrt();
    if rho < 1e-4 {
        return false;
    }

    let mut disk: Polygon = make_oriented_disk_xy(center, x2, y2, rho, nsteps);

    let nxy: f64 = normal.x.hypot(normal.y);
    if nxy > 1e-6 {
        // Half-plane in XY from plane equation n·(q-p) ≥ 0 with q.z = slice_z.
        // n.x*(x-cx) + n.y*(y-cy) + n.z*dz  ≥  0
        // `normal.x() * center.x() + normal.y() * center.y() - normal.z() * dz`: the first product is fused into the
        // sum, the last one into the difference.
        let rhs: f64 = (-normal.z).mul_add(dz, normal.x.mul_add(center.x, normal.y * center.y));
        // bottom: n.xy · r <= rhs ; top: n.xy · r >= rhs  <=>  (-n.xy)·r <= -rhs
        disk = if is_bottom {
            clip_convex_polygon_halfplane(&disk, normal.x, normal.y, rhs)
        } else {
            clip_convex_polygon_halfplane(&disk, -normal.x, -normal.y, -rhs)
        };
        if disk.points.len() < 3 {
            return false;
        }
    } else {
        // Vertical axis: keep only the hemisphere side of the equator.
        if is_bottom && dz > EPSILON {
            return false;
        }
        if !is_bottom && dz < -EPSILON {
            return false;
        }
    }

    *out = disk;
    true
}

/// TreeSupport3D.cpp:5362-5371 `struct PreparedSplitCap`
///
/// Production-slice cache for one ExtrudeSplitCap: valid mid-Z layer range, circle
/// discretization and XY basis so injection walks O(H) layers per cap instead of O(S).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PreparedSplitCap {
    /// The cap itself (a pointer in the pin).
    pub cap: ExtrudeSplitCap,
    pub layer_begin: LayerIndex,
    pub layer_end: LayerIndex,
    pub nsteps: i32,
    pub x2: (f64, f64),
    pub y2: (f64, f64),
}

/// TreeSupport3D.cpp:5373-5397 `static PreparedSplitCap prepare_split_cap(const ExtrudeSplitCap &sc, const
/// SlicingParameters &slicing_params, const TreeSupportSettings &config, LayerIndex num_layers)`
pub fn prepare_split_cap(sc: &ExtrudeSplitCap, slicing_params: &crate::slicing::SlicingParams, config: &TreeSupportSettings, num_layers: LayerIndex) -> PreparedSplitCap {
    use crate::support::tree_support_common::{layer_idx_mid_ceil, layer_idx_mid_floor};
    let mut out = PreparedSplitCap { cap: *sc, layer_begin: 0, layer_end: 0, nsteps: 0, x2: (1., 0.), y2: (0., 1.) };
    out.nsteps = hemisphere_circle_nsteps(sc.radius);
    hemisphere_disk_xy_basis(&sc.normal, &mut out.x2, &mut out.y2);
    // Write the hemisphere as m·(q-c) <= 0. It crosses the horizontal plane through its
    // own center whenever the axis is tilted, still reaching r * sqrt(1 - m.z^2) on the
    // far side; bounding the scan at that plane instead would drop the very band this
    // injection restores. Bounding it by the full sphere only wastes layers.
    let m: Vec3d = if sc.is_bottom { sc.normal } else { Vec3d::new(-sc.normal.x, -sc.normal.y, -sc.normal.z) };
    // std::clamp(m.z(), -1., 1.)
    let mz: f64 = if m.z < -1. {
        -1.
    } else if 1. < m.z {
        1.
    } else {
        m.z
    };
    // sc.radius * std::sqrt(std::max(0., 1. - mz * mz)): the product is fused into the difference.
    let d: f64 = (-mz).mul_add(mz, 1.);
    let reach: f64 = sc.radius * (if 0. < d { d } else { 0. }).sqrt();
    let z_lo: f64 = sc.center.z - if mz < 0. { reach } else { sc.radius };
    let z_hi: f64 = sc.center.z + if mz > 0. { reach } else { sc.radius };
    out.layer_begin = layer_idx_mid_ceil(slicing_params, config, z_lo, num_layers);
    out.layer_end = layer_idx_mid_floor(slicing_params, config, z_hi, num_layers) + 1;
    out.layer_begin = std::cmp::max(out.layer_begin, 0);
    out.layer_end = std::cmp::min(out.layer_end, num_layers);
    out
}

/// TreeSupport3D.cpp:5399-5438 `static std::vector<Polygons> merged_band_keep_out(const PrintObject &print_object,
/// const TreeSupportSettings &config, size_t num_support_layers)`
///
/// BBS multi-diameter: per support layer, the merged bands starting within the top gap above it, kept at the
/// XY gap the collision would apply to them. The collision only knows the lslices, so a branch growing next to
/// such a band may end up under it and leave base slivers the roof does not cover. Only the finished areas are
/// clipped: feeding the bands into the collision would also grow the avoidance and drop branches that have to
/// pass close to them.
pub fn merged_band_keep_out(print_object: &pf::PrintObject, config: &TreeSupportSettings, num_support_layers: usize) -> Vec<Polygons> {
    use crate::support::support_common::merged_band_footprint;

    let num_raft_layers: usize = config.raft_layers.len();
    let mut band_keep_out: Vec<Polygons> = vec![Vec::new(); num_support_layers];
    let mut band_footprint: Vec<Polygons> = vec![Vec::new(); print_object.layer_count()];
    let mut has_band = false;
    for obj_layer_idx in 0..band_footprint.len() {
        let band: Polygons = merged_band_footprint(print_object.get_layer(obj_layer_idx as i32), obj_layer_idx);
        if !band.is_empty() {
            band_footprint[obj_layer_idx] = cu::union_(&band);
            has_band = true;
        }
    }
    if has_band && num_support_layers > num_raft_layers {
        // Same XY gap as TreeModelVolumes::calculateCollision(): the full gap on the layer itself, fading out
        // over the layers of the top gap above it, down to the bare footprint.
        let z_distance_top_layers: usize = config.z_distance_top_layers;
        let mut xy_gap: Vec<Coord> = vec![config.xy_min_distance; z_distance_top_layers + 1];
        for dz in 1..=z_distance_top_layers {
            // std::max<coord_t>(0, coord_t(double(config.xy_min_distance) * (1. - (double(dz) - (z_distance_top_layers
            // == 1 ? 0.5 : 0.)) / double(z_distance_top_layers))))
            let v: f64 = config.xy_min_distance as f64 * (1. - (dz as f64 - if z_distance_top_layers == 1 { 0.5 } else { 0. }) / z_distance_top_layers as f64);
            xy_gap[dz] = std::cmp::max(0, v as i32 as Coord);
        }
        for layer_idx in num_raft_layers..num_support_layers {
            let obj_layer_idx: usize = layer_idx - num_raft_layers;
            let mut band: Polygons = Vec::new();
            let mut dz: usize = 0;
            while dz <= z_distance_top_layers && obj_layer_idx + dz < band_footprint.len() {
                let footprint: &Polygons = &band_footprint[obj_layer_idx + dz];
                if !footprint.is_empty() {
                    if xy_gap[dz] > 0 {
                        band.extend(cu::offset(footprint, xy_gap[dz] as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
                    } else {
                        band.extend(footprint.iter().cloned());
                    }
                }
                dz += 1;
            }
            if !band.is_empty() {
                band_keep_out[layer_idx] = cu::union_(&band);
            }
        }
    }
    band_keep_out
}

/// TreeSupport3D.cpp:5539-5543 `struct Branch` of organic_draw_branches(): the elements of a branch, by layer and
/// index in `move_bounds`.
#[derive(Debug, Clone, Default)]
pub struct OrganicBranch {
    pub path: Vec<(usize, usize)>,
    pub has_root: bool,
    pub has_tip: bool,
}

/// TreeSupport3D.cpp:5545-5549 `struct Slice` of organic_draw_branches()
#[derive(Debug, Clone, Default)]
pub struct OrganicSlice {
    pub polygons: Polygons,
    pub bottom_contacts: Polygons,
    pub num_branches: usize,
}

/// TreeSupport3D.cpp:5551-5556 `struct Tree` of organic_draw_branches()
#[derive(Debug, Clone)]
pub struct OrganicTree {
    pub branches: Vec<OrganicBranch>,
    pub slices: Vec<OrganicSlice>,
    pub first_layer_id: LayerIndex,
}

impl Default for OrganicTree {
    fn default() -> Self {
        // LayerIndex first_layer_id{ -1 };
        Self { branches: Vec::new(), slices: Vec::new(), first_layer_id: -1 }
    }
}

/// TreeSupport3D.cpp:5560-5611 `TreeVisitor::visit_recursive(std::vector<SupportElements>& move_bounds,
/// SupportElement& start_element, Tree& out)` of organic_draw_branches()
fn organic_visit_recursive(move_bounds: &mut [SupportElements], start_element: (usize, usize), out: &mut OrganicTree) {
    // Collect elements up to a bifurcation above.
    move_bounds[start_element.0][start_element.1].state.marked = true;
    // For each branch bifurcating from this point:
    let layer_above: usize = (move_bounds[start_element.0][start_element.1].state.layer_idx + 1) as usize;
    let root: bool = out.branches.is_empty();
    let start_parents: usize = move_bounds[start_element.0][start_element.1].parents.len();
    for parent_idx in 0..start_parents {
        let mut branch = OrganicBranch::default();
        branch.path.push(start_element);
        // Traverse each branch until it branches again.
        let first_parent: (usize, usize) = (layer_above, move_bounds[start_element.0][start_element.1].parents[parent_idx] as usize);
        branch.path.push(first_parent);
        let first_parent_parents: usize = move_bounds[first_parent.0][first_parent.1].parents.len();
        if first_parent_parents < 2 {
            move_bounds[first_parent.0][first_parent.1].state.marked = true;
        }
        let mut next_branch: Option<(usize, usize)> = None;
        if first_parent_parents == 1 {
            let mut parent: (usize, usize) = first_parent;
            loop {
                let p: &SupportElement = &move_bounds[parent.0][parent.1];
                let next_parent: (usize, usize) = ((p.state.layer_idx + 1) as usize, p.parents[0] as usize);
                branch.path.push(next_parent);
                let next_parent_parents: usize = move_bounds[next_parent.0][next_parent.1].parents.len();
                if next_parent_parents > 1 {
                    // Branching point was reached.
                    next_branch = Some(next_parent);
                    break;
                }
                move_bounds[next_parent.0][next_parent.1].state.marked = true;
                if next_parent_parents == 0 {
                    // Tip is reached.
                    break;
                }
                parent = next_parent;
            }
        } else if first_parent_parents > 1 {
            // Branching point was reached.
            next_branch = Some(first_parent);
        }
        branch.has_root = root;
        branch.has_tip = next_branch.is_none();
        out.branches.push(branch);
        if let Some(next_branch) = next_branch {
            organic_visit_recursive(move_bounds, next_branch, out);
        }
    }
}

/// TreeSupport3D.cpp:5440-6049 `void organic_draw_branches(PrintObject& print_object, TreeModelVolumes& volumes, const
/// TreeSupportSettings& config, std::vector<SupportElements>& move_bounds, SupportGeneratorLayersPtr& bottom_contacts,
/// SupportGeneratorLayersPtr& top_contacts, InterfacePlacer& interface_placer, SupportGeneratorLayersPtr&
/// intermediate_layers, SupportGeneratorLayerStorage& layer_storage, std::vector<ExPolygons>& cooldown_areas,
/// std::function<void()> throw_on_cancel)`
///
/// The branches of the organic supports: the points of the elements smoothed and nudged off the object, every
/// tree extruded branch by branch into meshes that are sliced at the middle of the layers, the slices of a tree
/// put together, what the object cut off dropped, then the layers.
///
/// The pin takes the trees and the layers through `tbb::parallel_for`; a tree and a layer are written by their
/// own pass only, here they go in their order (which is also the order the layers are allocated in).
#[allow(clippy::too_many_arguments)]
pub fn organic_draw_branches(
    print_object: &pf::PrintObject,
    volumes: &mut TreeModelVolumes,
    config: &TreeSupportSettings,
    move_bounds: &mut [SupportElements],
    // I/O:
    bottom_contacts: &mut SupportGeneratorLayersPtr,
    top_contacts: &mut SupportGeneratorLayersPtr,
    _interface_placer: &mut InterfacePlacer,
    // Output:
    intermediate_layers: &mut SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
    cooldown_areas: &mut [Vec<ExPolygon>],
    throw_on_cancel: ThrowOnCancel,
) {
    use crate::support::tree_support_common::{layer_idx_mid_ceil, layer_idx_mid_floor, layer_mid_z};
    use crate::triangle_mesh_slicer::{slice_mesh_its_identity, MeshSlicingParams, SlicingMode};
    use std::f64::consts::PI as M_PI;

    // All SupportElements are put into a layer independent storage to improve parallelization.
    let (elements_with_link_down, linear_data_layers) = organic_elements_with_link_down(move_bounds);

    throw_on_cancel();

    organic_smooth_branches_avoid_collisions(print_object, volumes, config, move_bounds, &elements_with_link_down, &linear_data_layers, throw_on_cancel);

    //save the cooling area
    let mut layer_idx: usize = 0;
    while layer_idx + 1 < move_bounds.len() {
        for elem_idx in 0..move_bounds[layer_idx].len() {
            let elem: &SupportElement = &move_bounds[layer_idx][elem_idx];
            if !elem.parents.is_empty() {
                let layer_above: &SupportElements = &move_bounds[layer_idx + 1];
                for &parent_elem in &elem.parents {
                    let parent: &SupportElement = &layer_above[parent_elem as usize];
                    // std::min(10.0, (parent.state.result_on_layer - elem.state.result_on_layer).cast<double>().norm() /
                    // config.support_line_width * 10): the difference of two `Point`s wraps in 32 bits.
                    let d: (f64, f64) = (
                        (parent.state.result_on_layer.x - elem.state.result_on_layer.x) as i32 as f64,
                        (parent.state.result_on_layer.y - elem.state.result_on_layer.y) as i32 as f64,
                    );
                    let v: f64 = norm2(d) / config.support_line_width as f64 * 10.;
                    let overhang_degree: f64 = if v < 10.0 { v } else { 10.0 };

                    if overhang_degree >= 2. {
                        let radius: Coord = support_element_radius(config, parent);
                        // make_circle(radius, radius / 100): a division of integers.
                        let mut circle: Polygon = make_circle_pin(radius as f64, (radius / 100) as f64);
                        circle.translate(parent.state.result_on_layer);
                        cooldown_areas[layer_idx + 1].push(ExPolygon::new(circle));
                    }
                }
            }
        }
        layer_idx += 1;
    }

    // Reduce memory footprint. After this point only finalize_interface_and_support_areas() will use volumes and from that only collisions with zero radius will be used.
    volumes.clear_all_but_object_collision();
    let volumes: &TreeModelVolumes = volumes;

    // Unmark all nodes.
    for elements in move_bounds.iter_mut() {
        for element in elements.iter_mut() {
            element.state.marked = false;
        }
    }

    // Traverse all nodes, generate tubes.
    // Traversal stack with nodes and their current parent
    let mut trees: Vec<OrganicTree> = Vec::new();

    let mut layer_idx: usize = 0;
    while layer_idx + 1 < move_bounds.len() {
        for start_idx in 0..move_bounds[layer_idx].len() {
            let start_element: &SupportElement = &move_bounds[layer_idx][start_idx];
            if !start_element.state.marked && !start_element.parents.is_empty() {
                trees.push(OrganicTree::default());
                let last = trees.len() - 1;
                organic_visit_recursive(move_bounds, (layer_idx, start_idx), &mut trees[last]);
            }
        }
        layer_idx += 1;
    }

    let slicing_params = print_object.slicing_parameters().clone();
    let mut mesh_slicing_params = MeshSlicingParams::default();
    mesh_slicing_params.mode = SlicingMode::Positive;

    {
        let move_bounds: &[SupportElements] = move_bounds;
        let mut partial_mesh = indexed_triangle_set::default();
        let mut slice_z: Vec<f32> = Vec::new();
        let mut bottom_contacts: Vec<Polygons> = Vec::new();
        let mesh_split_caps: bool = split_cap_use_mesh_hemispheres();
        for tree in trees.iter_mut() {
            for branch_idx in 0..tree.branches.len() {
                let branch: &OrganicBranch = &tree.branches[branch_idx];
                let path: Vec<&SupportElement> = branch.path.iter().map(|&(l, i)| &move_bounds[l][i]).collect();
                let path_front: &SupportElement = path[0];
                let path_back: &SupportElement = path[path.len() - 1];
                // Triangulate the tube.
                partial_mesh.vertices.clear();
                partial_mesh.indices.clear();
                let mut split_caps: Vec<ExtrudeSplitCap> = Vec::new();
                let zspan: (f32, f32) =
                    extrude_branch(&path, config, &slicing_params, move_bounds, &mut partial_mesh, if mesh_split_caps { None } else { Some(&mut split_caps) });
                // Slice planes are layer mid-Z (not print_z). Map mesh/cap spans with
                // layer_idx_mid_* so the last/first mid-plane inside the span is not skipped.
                let num_layers: LayerIndex = move_bounds.len() as LayerIndex;
                let mesh_begin: LayerIndex = if branch.has_root {
                    path_front.state.layer_idx
                } else {
                    std::cmp::min(path_front.state.layer_idx, layer_idx_mid_ceil(&slicing_params, config, zspan.0 as f64, num_layers))
                };
                let mesh_end: LayerIndex = (if branch.has_tip {
                    path_back.state.layer_idx
                } else {
                    std::cmp::max(path_back.state.layer_idx, layer_idx_mid_floor(&slicing_params, config, zspan.1 as f64, num_layers))
                }) + 1;
                // Analytic mode: mesh Z excludes split-joint hemispheres; expand the output
                // layer range so 2D caps can be injected. Mesh mode already includes them in zspan.
                let mut layer_begin: LayerIndex = mesh_begin;
                let mut layer_end: LayerIndex = mesh_end;
                let mut prepared_caps: Vec<PreparedSplitCap> = Vec::new();
                if !split_caps.is_empty() {
                    prepared_caps.reserve(split_caps.len());
                    for sc in &split_caps {
                        let pc: PreparedSplitCap = prepare_split_cap(sc, &slicing_params, config, num_layers);
                        if pc.layer_begin >= pc.layer_end {
                            continue;
                        }
                        layer_begin = std::cmp::min(layer_begin, pc.layer_begin);
                        layer_end = std::cmp::max(layer_end, pc.layer_end);
                        prepared_caps.push(pc);
                    }
                }
                if branch.has_root {
                    layer_begin = std::cmp::max(layer_begin, path_front.state.layer_idx);
                }
                if branch.has_tip {
                    layer_end = std::cmp::min(layer_end, path_back.state.layer_idx + 1);
                }
                layer_begin = std::cmp::max(layer_begin, 0);
                layer_end = std::cmp::min(layer_end, num_layers);
                if layer_begin >= layer_end {
                    continue;
                }
                slice_z.clear();
                for layer_idx in layer_begin..layer_end {
                    slice_z.push(layer_mid_z(&slicing_params, config, layer_idx) as f32);
                }
                let mut slices: Vec<Polygons>;
                // Fast path: no analytic inject needed — slice the full mid-Z range once.
                if prepared_caps.is_empty() {
                    slices = slice_mesh_its_identity(&partial_mesh, &slice_z, &mesh_slicing_params, &|| throw_on_cancel());
                } else {
                    let clip_mesh_begin: LayerIndex = std::cmp::max(mesh_begin, layer_begin);
                    let clip_mesh_end: LayerIndex = std::cmp::min(mesh_end, layer_end);
                    let mut mesh_slice_z: Vec<f32> = Vec::new();
                    if clip_mesh_begin < clip_mesh_end {
                        mesh_slice_z.extend_from_slice(&slice_z[(clip_mesh_begin - layer_begin) as usize..(clip_mesh_end - layer_begin) as usize]);
                    }
                    let mesh_slices: Vec<Polygons> = if mesh_slice_z.is_empty() {
                        Vec::new()
                    } else {
                        slice_mesh_its_identity(&partial_mesh, &mesh_slice_z, &mesh_slicing_params, &|| throw_on_cancel())
                    };
                    slices = vec![Vec::new(); slice_z.len()];
                    for (i, s) in mesh_slices.into_iter().enumerate() {
                        slices[(clip_mesh_begin - layer_begin) as usize + i] = s;
                    }
                    // Cap-driven inject: each prepared cap only walks its mid-Z [begin,end).
                    for pc in &prepared_caps {
                        let inj_begin: LayerIndex = std::cmp::max(pc.layer_begin, layer_begin);
                        let inj_end: LayerIndex = std::cmp::min(pc.layer_end, layer_end);
                        for layer_idx in inj_begin..inj_end {
                            let mut poly = Polygon::new();
                            if hemisphere_slice_polygon(
                                &pc.cap.center,
                                &pc.cap.normal,
                                pc.cap.radius,
                                pc.cap.is_bottom,
                                slice_z[(layer_idx - layer_begin) as usize] as f64,
                                pc.nsteps,
                                &pc.x2,
                                &pc.y2,
                                &mut poly,
                            ) {
                                slices[(layer_idx - layer_begin) as usize].push(poly);
                            }
                        }
                    }
                }

                // Plate cone on the single trunk only. Must not change extrude radii.
                if branch.has_root {
                    append_organic_plate_foot_slices(&path, config, layer_begin, &mut slices);
                }

                bottom_contacts.clear();
                //FIXME parallelize?
                for i in 0..slices.len() {
                    slices[i] = diff_clipped(&slices[i], &volumes.get_collision(0, layer_begin + i as LayerIndex, true)); // FIXME parent_uses_min || draw_area.element->state.use_min_xy_dist);
                    slices[i] = cu::intersection(&slices[i], std::slice::from_ref(&volumes.m_bed_area), cu::ApplySafetyOffset::No);
                }
                // Per-branch model-severed fragment removal is deferred to a tree-level
                // connectivity pass (keep_main) that runs after sibling branches are merged,
                // so a fragment held up by a sibling branch is not dropped prematurely.
                let mut num_empty: usize = 0;
                if slices[0].is_empty() {
                    // Some of the initial layers are empty.
                    num_empty = slices.iter().position(|s| !s.is_empty()).unwrap_or(slices.len());
                } else if branch.has_root {
                    if path_front.state.to_model_gracious {
                        if config.settings.support_floor_layers > 0 {
                            //FIXME one may just take the whole tree slice as bottom interface.
                            bottom_contacts.push(intersection_clipped(&slices[0], &volumes.get_placeable_areas(0, layer_begin, &|| {})));
                        }
                    } else if layer_begin > 0 {
                        // Drop down areas that do rest non - gracefully on the model to ensure the branch actually rests on something.
                        struct BottomExtraSlice {
                            polygons: Polygons,
                            #[allow(dead_code)]
                            area: f64,
                        }
                        let mut bottom_extra_slices: Vec<BottomExtraSlice> = Vec::new();
                        let mut rest_support: Polygons = Vec::new();
                        let bottom_radius: Coord = support_element_radius(config, path_front);
                        // nofloat (Solution 1): drop the non-gracious "fake root" until it
                        // wraps onto the model surface (rest area pinched below
                        // support_area_stop) or reaches the bed, instead of a fixed
                        // layers_propagate_max budget that left the bottom floating. Sibling
                        // overlaps are welded later by the per-tree union + keep_main pass.
                        let layer_bottommost: LayerIndex = 0;
                        let branch_radius: f64 = config.branch_radius as f64;
                        let support_area_min_radius: f64 = M_PI * (branch_radius * branch_radius);
                        // std::max(0.2 * M_PI * sqr(double(bottom_radius)), 0.5 * support_area_min_radius)
                        let bottom_radius_f: f64 = bottom_radius as f64;
                        let a: f64 = 0.2 * M_PI * (bottom_radius_f * bottom_radius_f);
                        let b: f64 = 0.5 * support_area_min_radius;
                        let support_area_stop: f64 = if a < b { b } else { a };
                        // Only propagate until the rest area is smaller than this threshold.
                        let mut layer_idx: LayerIndex = layer_begin - 1;
                        while layer_idx >= layer_bottommost {
                            // Drop is now bounded by the model/bed instead of layers_propagate_max, so poll cancel every 16 layers.
                            if ((layer_begin - 1 - layer_idx) & 15) == 15 {
                                throw_on_cancel();
                            }
                            rest_support = diff_clipped(if rest_support.is_empty() { &slices[0] } else { &rest_support }, &volumes.get_collision(0, layer_idx, false));
                            let rest_support_area: f64 = area(&rest_support);
                            if rest_support_area < support_area_stop {
                                // Don't propagate a fraction of the tree contact surface.
                                break;
                            }
                            bottom_extra_slices.push(BottomExtraSlice { polygons: rest_support.clone(), area: rest_support_area });
                            layer_idx -= 1;
                        }
                        // Now remove those bottom slices that are not supported at all.
                        if config.settings.support_floor_layers > 0 {
                            let mut i: i32 = bottom_extra_slices.len() as i32 - 2;
                            while i >= 0 {
                                bottom_contacts
                                    .push(intersection_clipped(&bottom_extra_slices[i as usize].polygons, &volumes.get_placeable_areas(0, layer_begin - i - 1, &|| {})));
                                i -= 1;
                            }
                        }
                        layer_begin -= bottom_extra_slices.len() as LayerIndex;
                        // slices.insert(slices.begin(), bottom_extra_slices.size(), {}); then the extra slices from the
                        // last one to the first.
                        let mut new_slices: Vec<Polygons> = Vec::with_capacity(bottom_extra_slices.len() + slices.len());
                        for src in bottom_extra_slices.into_iter().rev() {
                            new_slices.push(src.polygons);
                        }
                        new_slices.append(&mut slices);
                        slices = new_slices;
                    }
                }

                layer_begin += num_empty as LayerIndex;
                while !slices.is_empty() && slices[slices.len() - 1].is_empty() {
                    slices.pop();
                    layer_end -= 1;
                }
                if layer_begin < layer_end {
                    let new_begin: LayerIndex = if tree.first_layer_id == -1 { layer_begin } else { std::cmp::min(tree.first_layer_id, layer_begin) };
                    let new_end: LayerIndex =
                        if tree.first_layer_id == -1 { layer_end } else { std::cmp::max(tree.first_layer_id + tree.slices.len() as LayerIndex, layer_end) };
                    let new_size: usize = (new_end - new_begin) as usize;
                    if tree.first_layer_id == -1 {
                    } else {
                        // Empty slices in front (the pin makes a new vector when the capacity does not do).
                        let dif: LayerIndex = tree.first_layer_id - new_begin;
                        if dif > 0 {
                            let mut new_slices: Vec<OrganicSlice> = Vec::with_capacity(new_size);
                            new_slices.resize_with(dif as usize, OrganicSlice::default);
                            new_slices.append(&mut tree.slices);
                            tree.slices = new_slices;
                        }
                    }
                    tree.slices.resize_with(new_size, OrganicSlice::default);
                    layer_begin -= num_empty as LayerIndex;
                    for i in layer_begin..layer_end {
                        let j: usize = (i - layer_begin) as usize;
                        if !slices[j].is_empty() {
                            let src: Polygons = std::mem::take(&mut slices[j]);
                            let dst: &mut OrganicSlice = &mut tree.slices[(i - new_begin) as usize];
                            dst.num_branches += 1;
                            if dst.num_branches > 1 {
                                dst.polygons.extend(src);
                                if j < bottom_contacts.len() {
                                    dst.bottom_contacts.extend(std::mem::take(&mut bottom_contacts[j]));
                                }
                            } else {
                                dst.polygons = src;
                                if j < bottom_contacts.len() {
                                    dst.bottom_contacts = std::mem::take(&mut bottom_contacts[j]);
                                }
                            }
                        }
                    }
                    tree.first_layer_id = new_begin;
                }
            }
        }
    }

    for tree in trees.iter_mut() {
        for slice in tree.slices.iter_mut() {
            if slice.num_branches > 1 {
                slice.polygons = cu::union_(&slice.polygons);
                slice.bottom_contacts = cu::union_(&slice.bottom_contacts);
                slice.num_branches = 1;
            }
        }
        // keep_main (tree level): now that sibling branches of this tree are merged per
        // layer, drop slice components not connected through vertical overlap to any branch
        // axis of this tree. Running at tree scope (instead of per branch) keeps fragments
        // held up by a sibling branch, which a per-branch pass cannot see.
        if tree.first_layer_id >= 0 {
            let base: LayerIndex = tree.first_layer_id;
            let n: usize = tree.slices.len();
            // 25 um: treat an axis point on/near a part boundary as inside.
            let axis_touch_tol: f64 = scaled_f64(0.025);
            let mut parts: Vec<Vec<ExPolygon>> = Vec::with_capacity(n);
            let mut keep: Vec<Vec<bool>> = Vec::with_capacity(n);
            for i in 0..n {
                parts.push(cu::union_ex(&tree.slices[i].polygons));
                keep.push(vec![false; parts[i].len()]);
            }
            // Seed with every branch axis node of this tree.
            for branch in &tree.branches {
                for &(l, e) in &branch.path {
                    let el: &SupportElement = &move_bounds[l][e];
                    let layer: LayerIndex = el.state.layer_idx;
                    if layer < base || (layer - base) as usize >= n {
                        continue;
                    }
                    let i: usize = (layer - base) as usize;
                    if parts[i].is_empty() {
                        continue;
                    }
                    let axis: Point = el.state.result_on_layer;
                    for j in 0..parts[i].len() {
                        if keep[i][j] {
                            continue;
                        }
                        let mut ok: bool = cu_expolygon::contains(&parts[i][j], &axis, true);
                        if !ok {
                            let mut pt: Point = axis;
                            move_inside(&parts[i][j].to_polygons(), &mut pt, 0, i64::MAX);
                            // (axis - pt).cast<double>().norm(): the difference wraps in 32 bits.
                            ok = norm2(((axis.x - pt.x) as i32 as f64, (axis.y - pt.y) as i32 as f64)) < axis_touch_tol;
                        }
                        if ok {
                            keep[i][j] = true;
                        }
                    }
                }
            }
            // Cache each layer's kept contour union; invalidated when its keep[] changes.
            let mut kept_cache: Vec<Polygons> = vec![Vec::new(); n];
            let mut cache_dirty: Vec<bool> = vec![true; n];
            fn kept_polys(i: usize, parts: &[Vec<ExPolygon>], keep: &[Vec<bool>], kept_cache: &mut [Polygons], cache_dirty: &mut [bool]) {
                if cache_dirty[i] {
                    let mut out: Polygons = Vec::new();
                    for j in 0..parts[i].len() {
                        if keep[i][j] {
                            out.extend(parts[i][j].to_polygons());
                        }
                    }
                    kept_cache[i] = out;
                    cache_dirty[i] = false;
                }
            }
            // One directional sweep: keep any part overlapping a kept part in the adjacent
            // lower (up) / upper (down) layer. Returns whether keep[] changed.
            let sweep = |up: bool, keep: &mut Vec<Vec<bool>>, kept_cache: &mut Vec<Polygons>, cache_dirty: &mut Vec<bool>| -> bool {
                let mut changed = false;
                for s in 1..n {
                    let i: usize = if up { s } else { n - 1 - s };
                    let p: usize = if up { i - 1 } else { i + 1 };
                    kept_polys(p, &parts, keep, kept_cache, cache_dirty);
                    if kept_cache[p].is_empty() {
                        continue;
                    }
                    for j in 0..parts[i].len() {
                        if !keep[i][j] && !cu::intersection(&parts[i][j].to_polygons(), &kept_cache[p], cu::ApplySafetyOffset::No).is_empty() {
                            keep[i][j] = true;
                            cache_dirty[i] = true;
                            changed = true;
                        }
                    }
                }
                changed
            };
            // Iterate to a fixpoint: a single up+down pass is not a connected-component
            // computation and would drop fragments linked to an axis through zig-zag
            // (multi-direction) layer chains. Keep flags only grow, so this terminates.
            let mut changed = true;
            while changed {
                let up_changed: bool = sweep(true, &mut keep, &mut kept_cache, &mut cache_dirty);
                let down_changed: bool = sweep(false, &mut keep, &mut kept_cache, &mut cache_dirty);
                changed = up_changed || down_changed;
            }
            for i in 0..n {
                kept_polys(i, &parts, &keep, &mut kept_cache, &mut cache_dirty);
                let kept: Polygons = kept_cache[i].clone();
                if !tree.slices[i].bottom_contacts.is_empty() {
                    tree.slices[i].bottom_contacts = cu::intersection(&tree.slices[i].bottom_contacts, &kept, cu::ApplySafetyOffset::No);
                }
                tree.slices[i].polygons = kept;
            }
        }
        throw_on_cancel();
    }

    let mut num_layers: usize = 0;
    for tree in &trees {
        if tree.first_layer_id >= 0 {
            num_layers = std::cmp::max(num_layers, tree.first_layer_id as usize + tree.slices.len());
        }
    }

    let mut slices: Vec<OrganicSlice> = vec![OrganicSlice::default(); num_layers];
    for tree in trees.iter_mut() {
        if tree.first_layer_id >= 0 {
            for i in tree.first_layer_id..tree.first_layer_id + tree.slices.len() as LayerIndex {
                let src: &mut OrganicSlice = &mut tree.slices[(i - tree.first_layer_id) as usize];
                if !src.polygons.is_empty() {
                    let dst: &mut OrganicSlice = &mut slices[i as usize];
                    dst.num_branches += 1;
                    if dst.num_branches > 1 {
                        dst.polygons.extend(std::mem::take(&mut src.polygons));
                        dst.bottom_contacts.extend(std::mem::take(&mut src.bottom_contacts));
                    } else {
                        dst.polygons = std::mem::take(&mut src.polygons);
                        dst.bottom_contacts = std::mem::take(&mut src.bottom_contacts);
                    }
                }
            }
        }
    }

    let num_final_layers: usize = std::cmp::min(move_bounds.len(), slices.len());
    let band_keep_out: Vec<Polygons> = merged_band_keep_out(print_object, config, num_final_layers);

    for layer_idx in 0..num_final_layers {
        let slice: &mut OrganicSlice = &mut slices[layer_idx];
        let mut base_layer_polygons: Polygons = if slice.num_branches > 1 { cu::union_(&slice.polygons) } else { std::mem::take(&mut slice.polygons) };
        let mut bottom_contact_polygons: Polygons =
            if slice.num_branches > 1 { cu::union_(&slice.bottom_contacts) } else { std::mem::take(&mut slice.bottom_contacts) };

        if !base_layer_polygons.is_empty() {
            // Most of the time in this function is this union call. Can take 300+ ms when a lot of areas are to be unioned.
            base_layer_polygons = crate::mutable_polygon::smooth_outward_polygons(cu::union_(&base_layer_polygons), config.support_line_width); //FIXME was .smooth(50);
            // simplify a bit, to ensure the output does not contain outrageous amounts of vertices. Should not be necessary, just a precaution.
            // std::min(scaled<double>(0.03), double(config.resolution))
            let resolution = config.resolution as f64;
            let tolerance = if resolution < scaled_f64(0.03) { resolution } else { scaled_f64(0.03) };
            base_layer_polygons = crate::support::tree_model_volumes::polygons_simplify_pin(&base_layer_polygons, tolerance);
        }
        let band: &Polygons = &band_keep_out[layer_idx];
        if !band.is_empty() {
            if !base_layer_polygons.is_empty() {
                base_layer_polygons = cu::diff(&base_layer_polygons, band, cu::ApplySafetyOffset::No);
            }
            if !bottom_contact_polygons.is_empty() {
                bottom_contact_polygons = cu::diff(&bottom_contact_polygons, band, cu::ApplySafetyOffset::No);
            }
        }

        // Subtract top contact layer polygons from support base.
        let top_contact_layer: usize = if top_contacts.is_empty() { SUPPORT_GENERATOR_LAYER_PTR_NULL } else { top_contacts[layer_idx] };
        if top_contact_layer != SUPPORT_GENERATOR_LAYER_PTR_NULL && !layer_storage[top_contact_layer].polygons.is_empty() && !base_layer_polygons.is_empty() {
            base_layer_polygons = cu::diff(&base_layer_polygons, &layer_storage[top_contact_layer].polygons, cu::ApplySafetyOffset::No);
            if !bottom_contact_polygons.is_empty() {
                //FIXME it may be better to clip bottom contacts with top contacts first after they are propagated to produce interface layers.
                bottom_contact_polygons = cu::diff(&bottom_contact_polygons, &layer_storage[top_contact_layer].polygons, cu::ApplySafetyOffset::No);
            }
        }
        if !bottom_contact_polygons.is_empty() {
            base_layer_polygons = cu::diff(&base_layer_polygons, &bottom_contact_polygons, cu::ApplySafetyOffset::No);
            let bottom_contact_layer: usize = layer_allocate(layer_storage, SupporLayerType::SltBottomContact, &slicing_params, config, layer_idx);
            bottom_contacts[layer_idx] = bottom_contact_layer;
            layer_storage[bottom_contact_layer].polygons = bottom_contact_polygons;
        }
        if !base_layer_polygons.is_empty() {
            let base_layer: usize = layer_allocate(layer_storage, SupporLayerType::SltBase, &slicing_params, config, layer_idx);
            intermediate_layers[layer_idx] = base_layer;
            layer_storage[base_layer].polygons = cu::union_(&base_layer_polygons);
        }

        throw_on_cancel();
    }
}

/// TreeSupport3D.cpp:4388-4792 `static void organic_lightning_infill(PrintObject &print_object, const
/// TreeModelVolumes &volumes, const TreeSupportSettings &config, SupportGeneratorLayersPtr &bottom_contacts,
/// SupportGeneratorLayersPtr &top_contacts, SupportGeneratorLayersPtr &intermediate_layers,
/// std::vector<Polylines> &lightning_infill_lines, std::function<void()> throw_on_cancel)`.
///
/// Lightning-infill style pass for internal voids in the organic tree support (see the C++ comment
/// :4357-4387). The `tbb::parallel_for`s over the layers are independent per layer and are plain loops here.
/// `print` is the print of `print_object` (the lightning generator's support constructor reads the print's
/// config through `m_object->print()`, Generator.cpp:102-104); the layers are looked up in `layer_storage`.
/// The `LIGHTNING_INFILL_DEBUG` blocks (:4411-4413, :4494-4498, :4569-4583, :4593-4597, :4620-4639,
/// :4642-4644, :4670-4674, :4700-4710, :4718-4734, :4736-4791) are not compiled at the pin.
#[allow(clippy::too_many_arguments)]
pub fn organic_lightning_infill(
    print_object: &pf::PrintObject,
    print: &pf::Print,
    volumes: &TreeModelVolumes,
    config: &TreeSupportSettings,
    bottom_contacts: &SupportGeneratorLayersPtr,
    top_contacts: &SupportGeneratorLayersPtr,
    intermediate_layers: &SupportGeneratorLayersPtr,
    layer_storage: &SupportGeneratorLayerStorage,
    lightning_infill_lines: &mut Vec<Vec<crate::geometry::Polyline>>,
    throw_on_cancel: ThrowOnCancel,
) {
    use crate::fill::lightning::generator::Generator;
    use crate::geometry::Polyline;

    // :4398-4400
    let num_layers: usize = intermediate_layers.len();
    if num_layers == 0 {
        return;
    }

    // :4402
    let line_width: Coord = config.support_line_width;
    // Scaled^2 area per 1 mm^2, used to express the feed-area window in real units.
    // :4404 `sqr(scaled<double>(1.))`
    let scaled_1: f64 = crate::scaled_f(1.);
    let area_scaled_per_mm2: f64 = scaled_1 * scaled_1;
    // Feed-area acceptance window for floating-sheath pieces and the void polygons that bound them.
    // Smaller than this is extrusion noise; larger than this is a genuine gap between separate branches
    // rather than a merge void.
    // :4408-4409
    let min_feed_area: f64 = 0.02 * area_scaled_per_mm2;
    let max_feed_area: f64 = 50.0 * area_scaled_per_mm2;

    // Per-layer grounding contours (the full solid support cross-section, holes included) and the
    // internal-void overhang to fill.
    // :4417-4418
    let mut contours: Vec<Polygons> = vec![Polygons::new(); num_layers];
    let mut lightning_overhangs: Vec<Polygons> = vec![Polygons::new(); num_layers];

    // :4420-4428
    let tree_base_at = |layer_idx: usize| -> Polygons {
        if layer_idx < num_layers && intermediate_layers[layer_idx] != SUPPORT_GENERATOR_LAYER_PTR_NULL {
            layer_storage[intermediate_layers[layer_idx]].polygons.clone()
        } else {
            Polygons::new()
        }
    };
    let roof_at = |layer_idx: usize| -> Polygons {
        if layer_idx < top_contacts.len() && top_contacts[layer_idx] != SUPPORT_GENERATOR_LAYER_PTR_NULL {
            layer_storage[top_contacts[layer_idx]].polygons.clone()
        } else {
            Polygons::new()
        }
    };
    let bottom_at = |layer_idx: usize| -> Polygons {
        if layer_idx < bottom_contacts.len() && bottom_contacts[layer_idx] != SUPPORT_GENERATOR_LAYER_PTR_NULL {
            layer_storage[bottom_contacts[layer_idx]].polygons.clone()
        } else {
            Polygons::new()
        }
    };

    // Reconstruct the real solid support cross-section at a layer. intermediate_layers has already had
    // the top/bottom contacts subtracted (see organic_draw_branches), which would otherwise introduce
    // spurious holes; unioning the base with its contacts restores the actual printed cross-section.
    // :4433-4438
    let support_at = |layer_idx: usize| -> Vec<ExPolygon> {
        let mut s: Polygons = tree_base_at(layer_idx);
        s.extend(bottom_at(layer_idx));
        s.extend(roof_at(layer_idx));
        cu::union_ex(&s)
    };
    // Interior holes of a cross-section within the acceptance window, returned as positively-oriented
    // region polygons. Used to bound lightning seeds to void interiors when thickening floating sheath.
    // :4441-4453
    let holes_of = |expolys: &[ExPolygon]| -> Polygons {
        let mut out = Polygons::new();
        for ep in expolys {
            for h in &ep.holes {
                let a: f64 = h.area().abs();
                if a < min_feed_area || a > max_feed_area {
                    continue;
                }
                let mut p: Polygon = h.clone();
                p.make_counter_clockwise();
                out.push(p);
            }
        }
        out
    };
    // Per-layer solid cross-section (ExPolygons) reused by both the contour and the detection below.
    // contours[L] carries the outer boundary and the hole rims, so a hole-interior overhang sample
    // grounds onto the surrounding support ring (Layer::getBestGroundingLocation scans every ring).
    // :4457-4465
    let mut support_ex: Vec<Vec<ExPolygon>> = vec![Vec::new(); num_layers];
    for layer_idx in 0..num_layers {
        support_ex[layer_idx] = support_at(layer_idx);
        contours[layer_idx] = crate::geometry::to_polygons(&support_ex[layer_idx]);
        throw_on_cancel();
    }

    // ---- Internal-void ("floating") detection -------------------------------------------------------
    // (the C++ comment :4467-4478)
    // :4479
    let half_line_width: f32 = (0.5 * line_width as f64) as f32;
    // Fallback when a support layer has no height recorded (e.g. empty placeholder). Prefer the
    // support layer's own height below so independent_support_layer_height stays accurate.
    // :4482 `float(scaled<double>(print_object.config().layer_height.value))`
    let fallback_support_tolerance: f32 = crate::scaled_f(cfg_float(print_object.config(), "layer_height")) as f32;
    // :4483-4487
    let sheath_of = |expolys: &[ExPolygon]| -> Polygons {
        if expolys.is_empty() {
            return Polygons::new();
        }
        cu::diff(
            &cu::offset_expolygons(expolys, half_line_width, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
            &cu::offset_expolygons(expolys, -half_line_width, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
            cu::ApplySafetyOffset::No,
        )
    };
    // :4488-4493
    let layer_support_tolerance = |layer_idx: usize| -> f32 {
        if layer_idx < intermediate_layers.len()
            && intermediate_layers[layer_idx] != SUPPORT_GENERATOR_LAYER_PTR_NULL
            && layer_storage[intermediate_layers[layer_idx]].height > crate::libslic3r::EPSILON
        {
            return crate::scaled_f(layer_storage[intermediate_layers[layer_idx]].height) as f32;
        }
        fallback_support_tolerance
    };
    // :4500-4506
    let mut sheaths: Vec<Polygons> = vec![Polygons::new(); num_layers];
    for layer_idx in 0..num_layers {
        sheaths[layer_idx] = sheath_of(&support_ex[layer_idx]);
        throw_on_cancel();
    }
    // :4507-4577 `detect_floating`, called for the layers 1.. (:4585-4590)
    for layer_idx in 1..num_layers.max(1) {
        throw_on_cancel();
        let sheath_here: &Polygons = &sheaths[layer_idx];
        if sheath_here.is_empty() {
            continue;
        }
        let sheath_below: &Polygons = &sheaths[layer_idx - 1];
        let support_tolerance: f32 = layer_support_tolerance(layer_idx);
        let mut floating: Polygons = if sheath_below.is_empty() {
            sheath_here.clone()
        } else {
            cu::diff(sheath_here, &cu::offset(sheath_below, support_tolerance, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT), cu::ApplySafetyOffset::No)
        };
        if floating.is_empty() {
            continue;
        }

        // Drop outer-wall growth: keep only floating sheath inside the branch. An empty inset means every
        // branch on this layer is thinner than two line widths and has no interior to seed into, so the
        // layer is rejected rather than let through with the outer sheath unfiltered.
        // :4522-4531
        let mut branch_interior = Polygons::new();
        for ep in &support_ex[layer_idx] {
            let inset: Vec<ExPolygon> = cu::offset_ex_expolygons(
                &[ExPolygon { contour: ep.contour.clone(), holes: Vec::new() }],
                -(line_width as f32),
                cu::DEFAULT_JOIN_TYPE,
                cu::DEFAULT_MITER_LIMIT,
            );
            branch_interior.extend(crate::geometry::to_polygons(&inset));
        }
        if branch_interior.is_empty() {
            continue;
        }
        floating = cu::intersection(&floating, &cu::union_(&branch_interior), cu::ApplySafetyOffset::No);
        if floating.is_empty() {
            continue;
        }

        // Apply the feed-area window to floating sheath pieces before thickening. Thickening can merge
        // several valid rims into one polygon larger than max_feed_area; re-applying max afterwards
        // would drop that whole feed. After thickening only discard crumbs below min_feed_area.
        // :4536-4548
        {
            let mut filtered_floating = Polygons::with_capacity(floating.len());
            for p in floating.drain(..) {
                let a: f64 = p.area().abs();
                if a < min_feed_area || a > max_feed_area {
                    continue;
                }
                filtered_floating.push(p);
            }
            floating = filtered_floating;
            if floating.is_empty() {
                continue;
            }
        }

        // Thicken the unsupported rim slightly into the adjacent void so DistanceField gets enough
        // samples; clip to this layer's holes (and holes below) so we never seed outside the void.
        // :4552-4557
        let holes_here: Polygons = holes_of(&support_ex[layer_idx]);
        let holes_below: Polygons = holes_of(&support_ex[layer_idx - 1]);
        let void_region: Polygons = cu::union_2(&holes_here, &holes_below);
        let mut feed: Polygons = floating.clone();
        if !void_region.is_empty() {
            feed = cu::union_2(
                &floating,
                &cu::intersection(&cu::offset(&floating, line_width as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT), &void_region, cu::ApplySafetyOffset::No),
            );
        }

        // :4559-4568
        let mut filtered = Polygons::with_capacity(feed.len());
        for p in feed {
            if p.area().abs() < min_feed_area {
                continue;
            }
            filtered.push(p);
        }
        if filtered.is_empty() {
            continue;
        }
        lightning_overhangs[layer_idx] = filtered;
    }

    // Critical safety clamp (the C++ comment :4599-4605): clip every fed overhang to its own layer's contour
    // bounding box as a defensive guarantee that the field always terminates.
    // :4606-4618
    let mut fed_layers: i32 = 0;
    for layer_idx in 0..num_layers {
        if lightning_overhangs[layer_idx].is_empty() {
            continue;
        }
        let cb = crate::geometry::get_extents_polygons(&contours[layer_idx]);
        if contours[layer_idx].is_empty() || !cb.is_defined() {
            lightning_overhangs[layer_idx].clear();
            continue;
        }
        lightning_overhangs[layer_idx] = cu::intersection(&lightning_overhangs[layer_idx], &[cb.polygon()], cu::ApplySafetyOffset::No);
        if !lightning_overhangs[layer_idx].is_empty() {
            fed_layers += 1;
        }
    }

    // :4641-4646
    if fed_layers == 0 {
        return;
    }

    // Density mirrors the legacy hybrid lightning reuse (TreeSupport::generate_toolpaths): derived
    // from the support base pattern spacing. The generator clamps it to at least 0.15.
    // :4650-4654
    let obj_cfg = print_object.config();
    let line_width_mm: f64 = unscaled_f64(line_width);
    let support_spacing_mm: f64 = cfg_float(obj_cfg, "support_base_pattern_spacing") + line_width_mm;
    // `std::min(1.0, x)`: x where x < 1.0
    let mut density: f64 = if support_spacing_mm > crate::libslic3r::EPSILON {
        let x = line_width_mm / support_spacing_mm * 2.0;
        if x < 1.0 {
            x
        } else {
            1.0
        }
    } else {
        0.15
    };
    // `std::max(0.15, density)`: density where 0.15 < density
    density = if 0.15 < density { density } else { 0.15 };

    // :4656
    let generator = Generator::new_for_support(print_object, print, &mut contours, &mut lightning_overhangs, throw_on_cancel, density as f32);

    // Store the generated lightning network per layer as polylines, the same way the hybrid tree does
    // (TreeSupport::generate_toolpaths). They are NOT merged into intermediate_layers (the C++ comment
    // :4658-4665).
    // :4666-4669
    if lightning_infill_lines.len() < num_layers {
        lightning_infill_lines.resize(num_layers, Vec::new());
    }
    // Anything shorter than this is a stub at a branch tip that costs a travel move and supports nothing.
    let min_line_length: f64 = crate::scaled_f(1.);
    // The area the lightning lines may run in: the cross-section with its voids filled, inset by half a line
    // width, minus the model. Only for the layers that have lightning trees.
    // :4676-4689
    let mut footprints: Vec<Polygons> = vec![Polygons::new(); num_layers];
    for layer_idx in 0..num_layers {
        if generator.get_trees_for_layer(layer_idx).tree_roots.is_empty() {
            continue;
        }
        let mut footprint = Polygons::new();
        for ep in &support_ex[layer_idx] {
            footprint.push(ep.contour.clone());
        }
        footprint = cu::offset(&cu::union_(&footprint), -half_line_width, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
        if !footprint.is_empty() {
            footprint = diff_clipped(&footprint, &volumes.get_collision(0, layer_idx as LayerIndex, true));
        }
        footprints[layer_idx] = footprint;
        throw_on_cancel();
    }
    // convertToPolylines() uses rand(): layer by layer in order so the result is the same every time
    // :4691-4735
    for layer_idx in 0..num_layers {
        throw_on_cancel();
        let footprint: &Polygons = &footprints[layer_idx];
        if footprint.is_empty() {
            continue;
        }

        let lightning_layer = generator.get_trees_for_layer(layer_idx);
        let mut lines: Vec<Polyline> = lightning_layer.convert_to_lines(footprint, 0);
        if lines.is_empty() {
            continue;
        }
        // :4711-4713
        lines.retain(|pl| !(pl.length() < min_line_length));
        if lines.is_empty() {
            continue;
        }

        // :4717
        lightning_infill_lines[layer_idx] = crate::shortest_path::chain_polylines(lines, None);
    }
}

/// A marker for the native test, read by nothing in the crate: set while generate_support_areas() is inside
/// generate_support_toolpaths() (TreeSupport3D.cpp:5110-5112). The pin has one `Fill::new_from_type()` for the
/// fillers TreeSupport3D.cpp asks for itself and for those SupportCommon.cpp asks for; the test's generator
/// compiles only TreeSupport3D.cpp with its recording filler, so the test's factory has to tell the two apart.
pub static GENERATE_SUPPORT_AREAS_IN_TOOLPATHS: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// TreeSupport3D.cpp:4794-5147 `static void generate_support_areas(Print &print, TreeSupport* tree_support, const
/// BuildVolume &build_volume, const std::vector<size_t> &print_object_ids, std::function<void()> throw_on_cancel)`.
///
/// The object of `print_object_ids` is the one `tree_support` holds (`TreeSupport::m_object`; in this crate the
/// TreeSupport borrows its object apart from the print, so `print.get_object(id)` of the C++ is that object here:
/// `group_meshes_with` / `precalculate_with`). The fillers of the support toolpaths come in as `fill_factory`.
/// The timing statistics (`std::chrono`, the log lines) are not kept except for `support_stage_times()`;
/// `print.set_status()` (:4950, :5103) is the caller's. An exception of the C++ is the `Err`.
pub fn generate_support_areas(
    print: &pf::Print,
    tree_support: &mut crate::support::tree_support::TreeSupport<'_>,
    build_volume: &crate::build_volume::BuildVolume,
    print_object_ids: &[usize],
    fill_factory: &dyn SupportFillFactory,
    throw_on_cancel: ThrowOnCancel,
) -> crate::Result<()> {
    use crate::libslic3r::EPSILON;
    use crate::print_config::enums::SupportMaterialPattern as Smp;
    use crate::support::support_common::cu_polygon::overlaps;
    use crate::support::support_common::{
        generate_interface_layers, generate_raft_base, generate_support_layers, generate_support_toolpaths, merge_support_layers_multi_diameter,
    };
    use crate::support::support_material::make_support_merge_plan_of_object;
    use crate::support::support_parameters::SupportParameters;
    use crate::support::tree_support::OverhangType;
    use crate::support::tree_support_common::layer_z;
    use crate::support_merge_plan::Candidate;
    use ordered_float::OrderedFloat;
    use std::collections::BTreeMap;
    let flow_err = |e: crate::flow::FlowError| crate::Error::Slicing(e.to_string());

    // Settings with the indexes of meshes that use these settings.
    // :4797-4799
    let mut grouped_meshes: Vec<(TreeSupportSettings, Vec<usize>)> = {
        let print_object: &pf::PrintObject = &*tree_support.m_object;
        group_meshes_with(print, print_object_ids, &|_object_id| print_object).map_err(flow_err)?
    };
    if grouped_meshes.is_empty() {
        return Ok(());
    }

    // Process every mesh group. These groups can not be processed parallel, as the processing in each group is parallelized, and nested parallelization is disables and slow.
    // :4804
    for processing in grouped_meshes.iter_mut() {
        // Generator for model collision, avoidance and internal guide volumes.
        // :4818-4824
        let mut volumes = {
            let config = &processing.0;
            TreeModelVolumes::from_print_object(
                &*tree_support.m_object,
                print,
                build_volume,
                config.maximum_move_distance,
                config.maximum_move_distance_slow,
                processing.1[0],
                &[],
            )
            .map_err(crate::Error::Slicing)?
        };
        // :4830-4833: use smart overhang detection
        let support_detect_begin_time = std::time::Instant::now();
        tree_support.detect_overhangs(false)?;
        let vertical_enforcer_points = tree_support.m_vertical_enforcer_points.clone();
        let detect_ms: i64;
        let mut interface_generate_ms: i64 = 0;

        // What the rest of the group needs of the object while it is only read: :4834-5067.
        let print_object: &pf::PrintObject = &*tree_support.m_object;
        let sp = print_object.slicing_parameters().clone();

        //FIXME generating overhangs just for the first mesh of the group.
        // :4827 `assert(processing.second.size() == 1)`

        // :4831-4845
        let num_raft_layers: usize = processing.0.raft_layers.len();
        let num_layers: usize = print_object.layer_count() + num_raft_layers;
        let mut overhangs: Vec<Polygons> = vec![Polygons::new(); num_layers];
        for i in 0..print_object.layer_count() {
            for expoly_type in &print_object.get_layer(i as i32).loverhangs_with_type {
                let mut polys: Polygons = crate::geometry::to_polygons(std::slice::from_ref(&expoly_type.0));
                if expoly_type.1 & (OverhangType::SHARP_TAIL.0 as i32) != 0 {
                    // `offset(polys, scale_(0.2))`: the double as the float delta
                    polys = cu::offset(&polys, crate::scaled_f(0.2) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                }
                overhangs[i + num_raft_layers].extend(polys);
            }
        }
        // add vertical enforcer points
        // :4847-4861
        let zs: Vec<f32> = crate::layer::zs_from_layers(print_object.layers());
        let base_circle: Polygon = make_circle_pin(crate::scaled_f(0.5), SUPPORT_TREE_CIRCLE_RESOLUTION as f64);
        for pt_and_normal in &vertical_enforcer_points {
            let pt = pt_and_normal.0;
            // normal seems useless
            // `std::lower_bound(zs.begin(), zs.end(), pt.z())`
            let layer_nr: usize = zs.partition_point(|&z| z < pt[2]);
            if layer_nr != zs.len() && layer_nr > 0 && layer_nr < print_object.layer_count() {
                let mut circle: Polygon = base_circle.clone();
                // `circle.translate(to_2d(pt).cast<coord_t>())`: the floats cut to the pin's 32 bit coord_t
                let (dx, dy) = (pt[0] as i32 as Coord, pt[1] as i32 as Coord);
                for p in &mut circle.points {
                    p.x += dx;
                    p.y += dy;
                }
                overhangs[layer_nr + num_raft_layers].push(circle);
            }
        }
        // :4862-4863
        detect_ms = support_detect_begin_time.elapsed().as_millis() as i64;

        // ### Precalculate avoidances, collision etc.
        // :4868
        let mut num_support_layers: usize =
            precalculate_with(&overhangs, &processing.0, &processing.1, &mut volumes, throw_on_cancel, &|_object_id| print_object) as usize;

        // :4870-4875
        let has_support: bool = num_support_layers > 0;
        let has_raft: bool = !processing.0.raft_layers.is_empty();
        num_support_layers = num_support_layers.max(processing.0.raft_layers.len());

        if num_support_layers == 0 {
            continue;
        }

        // :4877-4882
        let mut support_params = SupportParameters::new(print_object, print).map_err(flow_err)?;
        support_params.with_sheath = true;
        // organic support default pattern is none.
        if processing.0.support_pattern == Smp::SmpDefault as i32 {
            processing.0.support_pattern = Smp::SmpNone as i32;
        }
        let config: &TreeSupportSettings = &processing.0;

        // :4884-4897
        let mut layer_storage = SupportGeneratorLayerStorage::new();
        let mut top_contacts: SupportGeneratorLayersPtr = Vec::new();
        let mut bottom_contacts: SupportGeneratorLayersPtr = Vec::new();
        let mut interface_layers: SupportGeneratorLayersPtr = Vec::new();
        let mut base_interface_layers: SupportGeneratorLayersPtr = Vec::new();
        let mut intermediate_layers: SupportGeneratorLayersPtr = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
        if support_params.has_top_contacts || has_raft {
            top_contacts = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
        }
        if support_params.has_bottom_contacts {
            bottom_contacts = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
        }
        if support_params.has_interfaces() || has_raft {
            interface_layers = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
        }
        if support_params.has_base_interfaces() || has_raft {
            base_interface_layers = vec![SUPPORT_GENERATOR_LAYER_PTR_NULL; num_support_layers];
        }

        // :4899-4908 `remove_undefined_layers`
        fn doit(layers: &mut SupportGeneratorLayersPtr) {
            layers.retain(|&ptr| ptr != SUPPORT_GENERATOR_LAYER_PTR_NULL);
        }

        // :4910-4913
        let mut interface_placer = InterfacePlacer::new(&sp, &support_params, config);

        // :4915-4924
        let mut cooldown_areas: Vec<Vec<ExPolygon>> = vec![Vec::new(); num_support_layers];
        // Filled at the dense layer_idx (same as intermediate_layers before compaction), then
        // compacted to the non-null intermediate slots so generate_support_toolpaths can look
        // them up by idx_layer_intermediate (print_z), not by support_layer_id.
        let mut lightning_infill_lines: Vec<Vec<crate::geometry::Polyline>> = vec![Vec::new(); num_support_layers];
        // The auxiliary areas stay indexed by the original organic grid even after undefined generator
        // layers are removed. Preserve the grid's real Z coordinates for the final support-layer remap.
        let mut per_layer_area_print_z: Vec<f64> = vec![0.; num_support_layers];
        for layer_idx in 0..num_support_layers {
            per_layer_area_print_z[layer_idx] = layer_z(&sp, config, layer_idx);
        }
        // :4925
        if has_support {
            // value is the area where support may be placed. As this is calculated in CreateLayerPathing it is saved and reused in draw_areas
            let mut move_bounds: Vec<SupportElements> = vec![Vec::new(); num_support_layers];

            // :4931-4933
            for _mesh_idx in &processing.1 {
                generate_initial_areas(
                    print_object,
                    print,
                    &volumes,
                    config,
                    &overhangs,
                    &mut move_bounds,
                    &mut interface_placer,
                    InterfacePlacerOutputs {
                        layer_storage: &mut layer_storage,
                        top_contacts: &mut top_contacts,
                        top_interfaces: &mut interface_layers,
                        top_base_interfaces: &mut base_interface_layers,
                    },
                    fill_factory,
                    throw_on_cancel,
                )
                .map_err(flow_err)?;
            }

            // ### Propagate the influence areas downwards. This is an inherently serial operation.
            // :4951
            create_layer_pathing(&volumes, config, &mut move_bounds, throw_on_cancel);

            // ### Set a point in each influence area
            // :4955
            create_nodes_from_area(&volumes, config, &mut move_bounds, throw_on_cancel);

            // ### draw these points as circles
            // this new function give correct result when raft is also enabled
            // :4967-4970
            organic_draw_branches(
                print_object,
                &mut volumes,
                config,
                &mut move_bounds,
                &mut bottom_contacts,
                &mut top_contacts,
                &mut interface_placer,
                &mut intermediate_layers,
                &mut layer_storage,
                &mut cooldown_areas,
                throw_on_cancel,
            );
            // ### Lightning infill for newly opened internal voids in organic hollow trunks (always on).
            // :4973-4974
            organic_lightning_infill(
                print_object,
                print,
                &volumes,
                config,
                &bottom_contacts,
                &top_contacts,
                &intermediate_layers,
                &layer_storage,
                &mut lightning_infill_lines,
                throw_on_cancel,
            );

            // lightning_infill_lines was filled at the same dense layer_idx as intermediate_layers
            // (including empty slots). Drop the matching entries so the two arrays stay aligned
            // after the nullptrs are removed; generate_support_toolpaths looks them up by the
            // compacted intermediate index (print_z), not by support_layer_id.
            // :4982-4992
            {
                let mut kept_lightning: Vec<Vec<crate::geometry::Polyline>> = Vec::with_capacity(intermediate_layers.len());
                for i in 0..intermediate_layers.len() {
                    if intermediate_layers[i] == SUPPORT_GENERATOR_LAYER_PTR_NULL {
                        continue;
                    }
                    kept_lightning.push(if i < lightning_infill_lines.len() { std::mem::take(&mut lightning_infill_lines[i]) } else { Vec::new() });
                }
                lightning_infill_lines = kept_lightning;
            }
            // :4993 `remove_undefined_layers()`
            doit(&mut bottom_contacts);
            doit(&mut top_contacts);
            doit(&mut interface_layers);
            doit(&mut base_interface_layers);
            doit(&mut intermediate_layers);

            // :4995-4999
            let support_interface_begin_time = std::time::Instant::now();
            let (new_interface_layers, new_base_interface_layers) = generate_interface_layers(
                print_object.config(),
                &support_params,
                &bottom_contacts,
                &top_contacts,
                &mut interface_layers,
                &mut base_interface_layers,
                &intermediate_layers,
                &mut layer_storage,
            );
            interface_layers = new_interface_layers;
            base_interface_layers = new_base_interface_layers;
            interface_generate_ms = support_interface_begin_time.elapsed().as_millis() as i64;

            // A very low overhang produces roofs that have no tree body underneath. Printing the whole
            // stack as interface leaves it without an anchor to the bed, so turn the bed-touching interface
            // layer into a base once the roofs above it are tall enough to spare it. A one- or two-layer
            // stack stays interface-only, where converting the bed layer would consume the whole support.
            // :5005-5026
            let bed_print_z: f64 = sp.object_print_z_min;
            if intermediate_layers.is_empty() && !interface_layers.is_empty() && layer_storage[interface_layers[0]].bottom_z < bed_print_z + EPSILON {
                let base_layer: usize = interface_layers[0];
                // Estimate the stack height from every roof whose footprint overlaps this bed interface
                // layer. Roofs sharing a layer index are merged into one layer object, and a genuine
                // single-layer overhang lives in top_contacts (dtt_roof == 0), so it is never demoted here;
                // only a bed interface layer that already carries taller roofs above it gets converted.
                let mut stack_zs: Vec<f64> = Vec::new();
                for layers in [&interface_layers, &base_interface_layers, &top_contacts] {
                    for &layer in layers.iter() {
                        if layer != SUPPORT_GENERATOR_LAYER_PTR_NULL
                            && !layer_storage[layer].polygons.is_empty()
                            && overlaps(&layer_storage[layer].polygons, &layer_storage[base_layer].polygons)
                        {
                            stack_zs.push(layer_storage[layer].print_z);
                        }
                    }
                }
                // `std::sort` of doubles; `std::unique` with `std::abs(l - r) < EPSILON` (the kept one against the next)
                stack_zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
                stack_zs.dedup_by(|r, l| (*l - *r).abs() < EPSILON);
                if stack_zs.len() >= 3 {
                    layer_storage[base_layer].layer_type = SupporLayerType::SltBase;
                    intermediate_layers.push(base_layer);
                    interface_layers.remove(0);
                }
            }

            // :5043
            move_bounds.clear();
        } else if generate_raft_contact(
            print_object,
            config,
            &mut interface_placer,
            &mut InterfacePlacerOutputs {
                layer_storage: &mut layer_storage,
                top_contacts: &mut top_contacts,
                top_interfaces: &mut interface_layers,
                top_base_interfaces: &mut base_interface_layers,
            },
        ) >= 0
        {
            // :5046 `remove_undefined_layers()`
            doit(&mut bottom_contacts);
            doit(&mut top_contacts);
            doit(&mut interface_layers);
            doit(&mut base_interface_layers);
            doit(&mut intermediate_layers);
        } else {
            // No raft.
            continue;
        }

        // Produce the support G-code.
        // :5053
        let raft_layers: SupportGeneratorLayersPtr =
            generate_raft_base(print_object, &support_params, &sp, &top_contacts, &interface_layers, &base_interface_layers, &intermediate_layers, &mut layer_storage);

        // BBS multi-diameter: merge the support areas into slabs a coarse nozzle can extrude. The organic tree
        // is grown on a layer grid of a fixed height, so this post-pass is the only place its Z ladder can
        // still be coarsened. print_z -> filament the merged slab at that height was planned for.
        // :5058-5067
        let mut planned_filaments: BTreeMap<OrderedFloat<f64>, Candidate> = BTreeMap::new();
        let mut planned_interface_filaments: BTreeMap<OrderedFloat<f64>, Candidate> = BTreeMap::new();
        let mut planned_base_heights: BTreeMap<OrderedFloat<f64>, f64> = BTreeMap::new();
        let mut planned_interface_heights: BTreeMap<OrderedFloat<f64>, f64> = BTreeMap::new();
        let plan = make_support_merge_plan_of_object(print_object, print);
        merge_support_layers_multi_diameter(
            print_object,
            &plan,
            &support_params,
            &sp,
            &raft_layers,
            &mut bottom_contacts,
            &mut top_contacts,
            &mut intermediate_layers,
            &mut interface_layers,
            &mut base_interface_layers,
            &mut planned_filaments,
            &mut planned_interface_filaments,
            &mut planned_base_heights,
            &mut planned_interface_heights,
            Some(&per_layer_area_print_z),
            Some(&mut cooldown_areas),
            None,
            // The tips were placed with the tree's own gap, which follows the nozzles in multi-diameter mode.
            // `double(config.z_distance_top_layers) * unscaled<double>(config.layer_height)`
            config.z_distance_top_layers as f64 * unscaled_f64(config.layer_height),
            &mut layer_storage,
        );
        // The object is written from here on: :5069-5109.
        let _ = print_object;
        let object: &mut pf::PrintObject = &mut *tree_support.m_object;
        object.m_support_stage_times.detect += detect_ms;
        object.m_support_stage_times.interface_generate += interface_generate_ms;

        // :5069
        let layers_sorted: SupportGeneratorLayersPtr = generate_support_layers(
            object,
            &raft_layers,
            &bottom_contacts,
            &top_contacts,
            &intermediate_layers,
            &interface_layers,
            &base_interface_layers,
            &mut layer_storage,
        );

        // Hand the planned filament over to the support layers the object owns now. Matching by print_z is safe
        // because generate_support_layers() gives every support layer a unique print_z.
        // :5073-5095
        for support_layer in object.m_support_layers.iter_mut() {
            let print_z = support_layer.layer.print_z;
            // `map.lower_bound(print_z - EPSILON)`, then `it->first <= print_z + EPSILON`
            if let Some((z, candidate)) = planned_filaments.range(OrderedFloat(print_z - EPSILON)..).next() {
                if z.0 <= print_z + EPSILON {
                    support_layer.planned_filament_id = candidate.filament_id;
                    support_layer.planned_nozzle_diameter = candidate.nozzle_diameter;
                }
            }
            if let Some((z, candidate)) = planned_interface_filaments.range(OrderedFloat(print_z - EPSILON)..).next() {
                if z.0 <= print_z + EPSILON {
                    support_layer.planned_interface_filament_id = candidate.filament_id;
                    support_layer.planned_interface_nozzle_diameter = candidate.nozzle_diameter;
                }
            }
            if let Some((z, height)) = planned_base_heights.range(OrderedFloat(print_z - EPSILON)..).next() {
                if z.0 <= print_z + EPSILON {
                    support_layer.base_slab_height = *height;
                }
            }
            if let Some((z, height)) = planned_interface_heights.range(OrderedFloat(print_z - EPSILON)..).next() {
                if z.0 <= print_z + EPSILON {
                    support_layer.interface_slab_height = *height;
                }
            }
        }

        // BBS: This is a hack to avoid the support being generated outside the bed area. See #4769.
        // :5098-5100
        for &layer in &layers_sorted {
            if layer != SUPPORT_GENERATOR_LAYER_PTR_NULL {
                layer_storage[layer].polygons = cu::intersection(&layer_storage[layer].polygons, std::slice::from_ref(&volumes.m_bed_area), cu::ApplySafetyOffset::No);
            }
        }

        // Don't fill in the tree supports, make them hollow with just a single sheath line.
        // :5104-5109
        let support_toolpath_begin_time = std::time::Instant::now();
        let object_config = object.config().clone();
        GENERATE_SUPPORT_AREAS_IN_TOOLPATHS.store(true, std::sync::atomic::Ordering::Relaxed);
        let toolpaths_result = generate_support_toolpaths(
            &mut object.m_support_layers,
            &object_config,
            &support_params,
            &sp,
            &raft_layers,
            &bottom_contacts,
            &top_contacts,
            &intermediate_layers,
            &interface_layers,
            &base_interface_layers,
            &cooldown_areas,
            Some(&mut lightning_infill_lines),
            Some(&|| throw_on_cancel()),
            &mut layer_storage,
            fill_factory,
        );
        GENERATE_SUPPORT_AREAS_IN_TOOLPATHS.store(false, std::sync::atomic::Ordering::Relaxed);
        toolpaths_result.map_err(flow_err)?;
        object.m_support_stage_times.toolpath_generate += support_toolpath_begin_time.elapsed().as_millis() as i64;
    }

    //   storage.support.generated = true;
    Ok(())
}

/// TreeSupport3D.cpp:6054-6069 `void generate_tree_support_3D(PrintObject &print_object, TreeSupport* tree_support,
/// std::function<void()> throw_on_cancel)`.
///
/// The object is `tree_support.m_object` (see `generate_support_areas`); `idx` is its place among the objects of
/// the print, which the caller knows (the TreeSupport of this crate borrows the object apart from the print).
pub fn generate_tree_support_3d(
    idx: usize,
    tree_support: &mut crate::support::tree_support::TreeSupport<'_>,
    fill_factory: &dyn SupportFillFactory,
    throw_on_cancel: ThrowOnCancel,
) -> crate::Result<()> {
    use crate::config::ConfigOptionResolver;
    // :6063-6066
    let bedpts: Vec<Point> = tree_support.m_machine_border.contour.points.clone();
    // `unscale(p)`: `Vec2d(unscale<double>(p.x()), unscale<double>(p.y()))`
    let bedptsf: Vec<crate::geometry::PointF> = bedpts.iter().map(|p| crate::geometry::PointF::new(unscaled_f64(p.x), unscaled_f64(p.y))).collect();
    let printable_height: f64 = tree_support.m_print_config.option("printable_height").and_then(|o| o.get_float().ok()).unwrap_or(0.0);
    let build_volume = crate::build_volume::BuildVolume::new_from_config(bedptsf, printable_height, Vec::new(), Vec::new());

    // :6068
    let print: &pf::Print = tree_support.m_print;
    generate_support_areas(print, tree_support, &build_volume, &[idx], fill_factory, throw_on_cancel)
}

/// The organic style of `TreeSupport::generate()` (`crate::support::tree_support::TreeSupport3DGenerator`):
/// `generate_tree_support_3D(*m_object, this, throw_on_cancel)` (TreeSupport.cpp, the `tree_organic` branch of
/// `TreeSupport::generate`). Holds what the C++ function reaches through the object and its print: the place of
/// the object among the print's objects, the fillers of the support toolpaths and the cancel callback.
pub struct OrganicTreeSupport3D<'f> {
    /// The index of the object in `print.objects()` (TreeSupport3D.cpp:6056-6061).
    pub idx: usize,
    pub fill_factory: &'f dyn SupportFillFactory,
    pub throw_on_cancel: ThrowOnCancel<'f>,
}

impl crate::support::tree_support::TreeSupport3DGenerator for OrganicTreeSupport3D<'_> {
    fn generate_tree_support_3d(&self, tree_support: &mut crate::support::tree_support::TreeSupport<'_>) -> crate::Result<()> {
        generate_tree_support_3d(self.idx, tree_support, self.fill_factory, self.throw_on_cancel)
    }
}
