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
            result_on_layer: Point::new(Coord::MAX, Coord::MAX),
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
        self.result_on_layer != Point::new(Coord::MAX, Coord::MAX)
    }

    /// TreeSupport3D.hpp:179
    pub fn result_on_layer_reset(&mut self) {
        self.result_on_layer = Point::new(Coord::MAX, Coord::MAX);
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
    let mut grouped_meshes: Vec<(TreeSupportSettings, Vec<usize>)> = Vec::new();

    //FIXME this is ugly, it does not belong here.
    for &object_id in print_object_ids {
        let print_object = print.get_object(object_id);
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
        let print_object = print.get_object(object_id);
        // Support must be enabled and set to Tree style.
        let found_existing_group = false;
        let next_settings =
            TreeSupportSettings::new(&TreeSupportMeshGroupSettings::new(print_object, print)?, print_object.slicing_parameters());
        //FIXME for now only a single object per group is enabled.
        if !found_existing_group {
            grouped_meshes.push((next_settings, vec![object_id]));
        }

        // no need to do this per mesh group as adaptive layers and raft setting are not setable per mesh.
        if print.get_object(largest_printed_mesh_idx).layers().last().unwrap().print_z < print_object.layers().last().unwrap().print_z {
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
    // calculate top most layer that is relevant for support
    let mut max_layer: LayerIndex = 0;
    for &object_id in object_ids {
        let print_object = print.get_object(object_id);
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
        volumes.precalculate(print.get_object(object_ids[0]), max_layer, throw_on_cancel);
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
                // `coord_t max_dist2_between_vertecies`: the squared norms are doubles, the stored one is truncated.
                let mut max_dist2_between_vertecies: Coord = 0;
                for idx in 0..(part.points.len() - 1) {
                    for inner_idx in 0..(part.points.len() - 1) {
                        if squared_norm2(vec2d(part.points[idx] - part.points[inner_idx])) > max_dist2_between_vertecies as f64 {
                            optimal_start_index = idx;
                            optimal_end_index = inner_idx;
                            max_dist2_between_vertecies = squared_norm2(vec2d(part.points[idx] - part.points[inner_idx])) as Coord;
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
