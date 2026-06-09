//! Common support generation utilities.
//!
//! C++ Reference:
//! - Support/SupportCommon.hpp
//! - Support/SupportCommon.cpp
//!
//! Faithful 1:1 line-by-line port of the self-contained, leaf utilities from
//! `SupportCommon.{hpp,cpp}`. These are the geometry-only helpers that do not
//! depend on the still-unported support pipeline types (`SupportGeneratorLayer`,
//! `SupportGeneratorLayerStorage`, `PrintObject`, `Fill`,
//! `ExtrusionEntityCollection`, `Flow`, `ClipperLib_Z`, TBB).
//!
//! Blocked (see notes / divergences):
//! - `generate_interface_layers`           — needs `SupportGeneratorLayer*`, `SupportGeneratorLayerStorage`, `SupportParameters` (full C++), `PrintObjectConfig`, TBB.
//! - `generate_raft_base`                  — needs `PrintObject`, `SlicingParameters`, `SupportGeneratorLayer*`, `SupportGeneratorLayerStorage`.
//! - `generate_support_layers`             — needs `PrintObject`, `SupportGeneratorLayer*`, `SupportLayer`.
//! - `generate_support_toolpaths`          — needs `Fill`, `ExtrusionEntityCollection`, `Flow`, `SupportLayer`, `LoopInterfaceProcessor`, TBB.
//! - `tree_supports_generate_paths`        — needs `ExtrusionEntityCollection`, `Flow`, `ClipperLib_Z` annotated paths.
//! - `fill_expolygons_with_sheath_generate_paths` — needs `Fill`, `ExtrusionEntityCollection`, `Flow`.
//! - `LoopInterfaceProcessor`              — needs `Flow`, `ClosestPointInRadiusLookup`, `ExtrusionEntityCollection`.
//! - `modulate_extrusion_by_overlapping_layers` — needs `ExtrusionPath`/`ExtrusionMultiPath`, `Flow`.

use crate::clipper_utils::{
    self, clip_clipper_polygons_with_subject_bbox_expolygon, offset_expolygons, offset_polygons,
    offset_polyline, OffsetJoinType,
};
use crate::geometry::{
    expolygons_simplify, get_extents as get_extents_expolygons, get_extents_polygons,
    polygons_simplify, ExPolygon, ExPolygons, Polygon, Polygons, Polylines,
};
// The ExPolygon-flavour `to_polylines(const ExPolygons&)`.
use crate::geometry::to_polylines as expolygons_to_polylines;
use crate::libslic3r::SCALED_EPSILON;
use crate::{scaled, Coord, CoordF};

// Slic3r::to_polylines(const Polygons&) -> Polylines (Polygon.hpp:235-239).
// The Polygon-flavour overload is not re-exported from `geometry`, so reconstruct it here.
fn polygons_to_polylines(polygons: &[Polygon]) -> Polylines {
    let mut out: Polylines = Vec::new();
    for polygon in polygons.iter() {
        out.push(polygon.to_polyline());
    }
    out
}

// SupportCommon.cpp:43
// how much we extend support around the actual contact area
//FIXME this should be dependent on the nozzle diameter!
#[allow(dead_code)]
pub const SUPPORT_MATERIAL_MARGIN: f64 = 1.5;

// SupportCommon.cpp:45-47
//#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 3.
//#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 1.5
// #define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.
#[allow(dead_code)]
pub const SUPPORT_SURFACES_OFFSET_JOIN_TYPE: OffsetJoinType = OffsetJoinType::Square;

// ============================================================================
// SupportCommon.hpp:83-150 — index search helpers (template functions).
// ============================================================================

// SupportCommon.hpp:78-108
// FN_HIGHER_EQUAL: the provided object pointer has a Z value >= of an internal threshold.
// Find the first item with Z value >= of an internal threshold of fn_higher_equal.
// If no vec item with Z value >= of an internal threshold of fn_higher_equal is found, return vec.size()
// If the initial idx is size_t(-1), then use binary search.
// Otherwise search linearly upwards.
//
// In C++ `idx` is an `IndexType` where `IndexType(-1)` (e.g. `size_t(-1)`) signals "use binary
// search". We model the index as `isize`, using `-1` as the binary-search sentinel.
pub fn idx_higher_or_equal<T, F>(slice: &[T], mut idx: isize, fn_higher_equal: F) -> isize
where
    F: Fn(&T) -> bool,
{
    // SupportCommon.hpp:86
    let size = slice.len() as isize;
    if size == 0 {
        // SupportCommon.hpp:88
        idx = 0;
    } else if idx == -1 {
        // SupportCommon.hpp:90-101
        // First of the batch of layers per thread pool invocation. Use binary search.
        let mut idx_low: isize = 0;
        let mut idx_high: isize = std::cmp::max(0, size - 1);
        while idx_low + 1 < idx_high {
            let idx_mid = (idx_low + idx_high) / 2;
            if fn_higher_equal(&slice[idx_mid as usize]) {
                idx_high = idx_mid;
            } else {
                idx_low = idx_mid;
            }
        }
        idx = if fn_higher_equal(&slice[idx_low as usize]) {
            idx_low
        } else if fn_higher_equal(&slice[idx_high as usize]) {
            idx_high
        } else {
            size
        };
    } else {
        // SupportCommon.hpp:103-105
        // For the other layers of this batch of layers, search incrementally, which is cheaper than the binary search.
        while idx < size && !fn_higher_equal(&slice[idx as usize]) {
            idx += 1;
        }
    }
    // SupportCommon.hpp:107
    idx
}

// SupportCommon.hpp:115-145
// FN_LOWER_EQUAL: the provided object pointer has a Z value <= of an internal threshold.
// Find the first item with Z value <= of an internal threshold of fn_lower_equal.
// If no vec item with Z value <= of an internal threshold of fn_lower_equal is found, return -1.
// If the initial idx is < -1, then use binary search.
// Otherwise search linearly downwards.
pub fn idx_lower_or_equal<T, F>(slice: &[T], mut idx: isize, fn_lower_equal: F) -> isize
where
    F: Fn(&T) -> bool,
{
    // SupportCommon.hpp:123
    let size = slice.len() as isize;
    if size == 0 {
        // SupportCommon.hpp:125
        idx = -1;
    } else if idx < -1 {
        // SupportCommon.hpp:127-138
        // First of the batch of layers per thread pool invocation. Use binary search.
        let mut idx_low: isize = 0;
        let mut idx_high: isize = std::cmp::max(0, size - 1);
        while idx_low + 1 < idx_high {
            let idx_mid = (idx_low + idx_high) / 2;
            if fn_lower_equal(&slice[idx_mid as usize]) {
                idx_low = idx_mid;
            } else {
                idx_high = idx_mid;
            }
        }
        idx = if fn_lower_equal(&slice[idx_high as usize]) {
            idx_high
        } else if fn_lower_equal(&slice[idx_low as usize]) {
            idx_low
        } else {
            -1
        };
    } else {
        // SupportCommon.hpp:140-142
        // For the other layers of this batch of layers, search incrementally, which is cheaper than the binary search.
        while idx >= 0 && !fn_lower_equal(&slice[idx as usize]) {
            idx -= 1;
        }
    }
    // SupportCommon.hpp:144
    idx
}

// ============================================================================
// Internal Polygons-flavoured Clipper helpers.
//
// The C++ file calls the free functions `union_`, `diff`, `offset`, `expand`,
// `polygons_simplify` (all operating on raw `Polygons`). The Rust crate currently
// exposes only ExPolygon-flavoured boolean ops, so these thin wrappers reconstruct
// the `Polygons -> Polygons` behaviour by going through ExPolygons and flattening
// the result to contours + holes, matching how the rest of the crate bridges the
// gap (see `support/tree_support_3d.rs`).
// ============================================================================

#[inline]
fn polygons_to_expolygons(polygons: &[crate::geometry::Polygon]) -> ExPolygons {
    polygons.iter().map(|p| ExPolygon::new(p.clone())).collect()
}

#[inline]
fn expolygons_to_polygons(expolygons: &[ExPolygon]) -> Polygons {
    let mut out: Polygons = Vec::new();
    for ex in expolygons {
        out.push(ex.contour.clone());
        for h in &ex.holes {
            out.push(h.clone());
        }
    }
    out
}

// Slic3r::union_(const Polygons&) -> Polygons
fn union_polygons(polygons: &[crate::geometry::Polygon]) -> Polygons {
    expolygons_to_polygons(&clipper_utils::union_polygons_ex(polygons))
}

// Slic3r::diff(const Polygons &subject, const Polygons &clip) -> Polygons
fn diff_polygons(subject: &[crate::geometry::Polygon], clip: &[crate::geometry::Polygon]) -> Polygons {
    let s = polygons_to_expolygons(subject);
    let c = polygons_to_expolygons(clip);
    expolygons_to_polygons(&clipper_utils::difference(&s, &c))
}

// Slic3r::offset(const Polygons&, delta, join_type, miter/arc) -> Polygons
fn offset_polygons_pp(
    polygons: &[crate::geometry::Polygon],
    delta: CoordF,
    join_type: OffsetJoinType,
) -> Polygons {
    expolygons_to_polygons(&offset_polygons(polygons, delta, join_type))
}

// ============================================================================
// SupportCommon.cpp:2089-2200 — safe_union / safe_offset_inc (Polygons).
// ============================================================================

/*
 * \brief Unions two Polygons. Ensures that if the input is non empty that the output also will be non empty.
 * \param first[in] The first Polygon.
 * \param second[in] The second Polygon.
 * \return The union of both Polygons
 */
// SupportCommon.cpp:2089
#[must_use]
pub fn safe_union(first: &Polygons, second: &Polygons) -> Polygons {
    // unionPolygons can slowly remove Polygons under certain circumstances, because of rounding issues (Polygons that have a thin area).
    // This does not cause a problem when actually using it on large areas, but as influence areas (representing centerpoints) can be very thin, this does occur so this ugly
    // workaround is needed Here is an example of a Polygons object that will loose vertices when unioning, and will be gone after a few times unionPolygons was called:
    /*
    Polygons example;
    Polygon exampleInner;
    exampleInner.add(Point(120410,83599));//A
    exampleInner.add(Point(120384,83643));//B
    exampleInner.add(Point(120399,83618));//C
    exampleInner.add(Point(120414,83591));//D
    exampleInner.add(Point(120423,83570));//E
    exampleInner.add(Point(120419,83580));//F
    example.add(exampleInner);
    for(int i=0;i<10;i++){
         log("Iteration %d Example area: %f\n",i,area(example));
         example=example.unionPolygons();
    }
    */

    // SupportCommon.cpp:2110
    let mut result: Polygons = Vec::new();
    // SupportCommon.cpp:2111
    if !first.is_empty() || !second.is_empty() {
        // SupportCommon.cpp:2112 — union_(first, second)
        let mut combined: Polygons = first.clone();
        combined.extend(second.iter().cloned());
        result = union_polygons(&combined);
        // SupportCommon.cpp:2113
        if result.is_empty() {
            // SupportCommon.cpp:2114 (debug log omitted)
            // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
            // SupportCommon.cpp:2116 — union_(offset(to_polylines(first), scaled<float>(0.002), jtMiter, 1.2), offset(to_polylines(second), ...))
            let mut merged: Polygons = Vec::new();
            for pl in polygons_to_polylines(first) {
                merged.extend(offset_polyline(&pl, scaled(0.002) as CoordF));
            }
            for pl in polygons_to_polylines(second) {
                merged.extend(offset_polyline(&pl, scaled(0.002) as CoordF));
            }
            result = union_polygons(&merged);
        }
    }

    // SupportCommon.cpp:2120
    result
}

// SupportCommon.cpp:2122
#[must_use]
pub fn safe_union_ex(first: &ExPolygons, second: &ExPolygons) -> ExPolygons {
    // SupportCommon.cpp:2124
    let mut result: ExPolygons = Vec::new();
    // SupportCommon.cpp:2125
    if !first.is_empty() || !second.is_empty() {
        // SupportCommon.cpp:2126 — union_ex(first, second)
        result = clipper_utils::union(first, second);
        // SupportCommon.cpp:2127
        if result.is_empty() {
            // SupportCommon.cpp:2128 (debug log omitted)
            // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
            // SupportCommon.cpp:2130 — union_(offset(to_polylines(first), scaled<float>(0.002), jtMiter, 1.2), offset(to_polylines(second), ...))
            let mut merged: Polygons = Vec::new();
            for pl in expolygons_to_polylines(first) {
                merged.extend(offset_polyline(&pl, scaled(0.002) as CoordF));
            }
            for pl in expolygons_to_polylines(second) {
                merged.extend(offset_polyline(&pl, scaled(0.002) as CoordF));
            }
            let result_polys = union_polygons(&merged);
            // SupportCommon.cpp:2131 — for (auto &poly : result_polys) result.emplace_back(ExPolygon(poly));
            for poly in result_polys {
                result.push(ExPolygon::new(poly));
            }
        }
    }

    // SupportCommon.cpp:2135
    result
}

/*
 * \brief Offsets (increases the area of) a polygons object in multiple steps to ensure that it does not lag through over a given obstacle.
 * \param me[in] Polygons object that has to be offset.
 * \param distance[in] The distance by which me should be offset. Expects values >=0.
 * \param collision[in] The area representing obstacles.
 * \param last_step_offset_without_check[in] The most it is allowed to offset in one step.
 * \param min_amount_offset[in] How many steps have to be done at least. As this uses round offset this increases the amount of vertices, which may be required if Polygons get
 * very small. Required as arcTolerance is not exposed in offset, which should result with a similar result. \return The resulting Polygons object.
 */
// SupportCommon.cpp:2147
#[must_use]
pub fn safe_offset_inc(
    me: &Polygons,
    distance: Coord,
    collision: &Polygons,
    safe_step_size: Coord,
    last_step_offset_without_check: Coord,
    min_amount_offset: usize,
) -> Polygons {
    // SupportCommon.cpp:2150
    let mut do_final_difference = last_step_offset_without_check == 0;
    // SupportCommon.cpp:2151 — ensure sane input
    let mut ret: Polygons = safe_union(me, &Vec::new());

    // SupportCommon.cpp:2153-2159
    // Trim the collision polygons with the region of interest for diff() efficiency.
    // Lazily-evaluated, cached trimmed collision (mirrors the C++ lambda + buffer).
    let mut collision_trimmed_buffer: Polygons = Vec::new();
    let mut collision_trimmed_done = false;
    macro_rules! collision_trimmed {
        () => {{
            if !collision_trimmed_done {
                if collision_trimmed_buffer.is_empty() && !collision.is_empty() {
                    let bbox = get_extents_polygons(&ret)
                        .expanded(std::cmp::max(0, distance) + scaled(SCALED_EPSILON));
                    // ClipperUtils::clip_clipper_polygons_with_subject_bbox(collision, bbox)
                    let mut trimmed: Polygons = Vec::new();
                    for ex in polygons_to_expolygons(collision) {
                        trimmed.extend(clip_clipper_polygons_with_subject_bbox_expolygon(
                            &ex, &bbox, false,
                        ));
                    }
                    collision_trimmed_buffer = trimmed;
                }
                collision_trimmed_done = true;
            }
            &collision_trimmed_buffer
        }};
    }

    // SupportCommon.cpp:2161
    if distance == 0 {
        return if do_final_difference {
            diff_polygons(&ret, collision_trimmed!())
        } else {
            union_polygons(&ret)
        };
    }
    // SupportCommon.cpp:2162-2165
    if safe_step_size < 0 || last_step_offset_without_check < 0 {
        // BOOST_LOG_TRIVIAL(error) << "Offset increase got invalid parameter!";
        return if do_final_difference {
            diff_polygons(&ret, collision_trimmed!())
        } else {
            union_polygons(&ret)
        };
    }

    // SupportCommon.cpp:2167
    let mut step_size = safe_step_size;
    // SupportCommon.cpp:2168
    let mut steps: i32 = if distance > last_step_offset_without_check {
        ((distance - last_step_offset_without_check) / step_size) as i32
    } else {
        0
    };
    // SupportCommon.cpp:2169-2175
    if distance - (steps as Coord) * step_size > last_step_offset_without_check {
        if ((steps as Coord) + 1) * step_size <= distance {
            // This will be the case when last_step_offset_without_check >= safe_step_size
            steps += 1;
        } else {
            do_final_difference = true;
        }
    }
    // SupportCommon.cpp:2176-2186
    // steps + (bool) < int(min_amount_offset) && min_amount_offset > 1
    let extra_step: Coord =
        Coord::from(distance < last_step_offset_without_check || (distance % step_size) != 0);
    if (steps as Coord) + extra_step < min_amount_offset as Coord && min_amount_offset > 1 {
        // yes one can add a bool as the standard specifies that a result from compare operators has to be 0 or 1
        // reduce the stepsize to ensure it is offset the required amount of times
        step_size = distance / min_amount_offset as Coord;
        if step_size >= safe_step_size {
            // effectivly reduce last_step_offset_without_check
            step_size = safe_step_size;
            steps = min_amount_offset as i32;
        } else {
            steps = (distance / step_size) as i32;
        }
    }
    // SupportCommon.cpp:2187-2192 — offset in steps
    for i in 0..steps {
        ret = diff_polygons(
            &offset_polygons_pp(&ret, step_size as CoordF, OffsetJoinType::Round),
            collision_trimmed!(),
        );
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if i % 10 == 7 {
            ret = polygons_simplify(&ret, scaled(0.015) as CoordF);
        }
    }
    // SupportCommon.cpp:2193-2195 — offset the remainder
    let last_offset = distance - (steps as Coord) * step_size;
    if last_offset > scaled(SCALED_EPSILON) {
        ret = offset_polygons_pp(
            &ret,
            (distance - (steps as Coord) * step_size) as CoordF,
            OffsetJoinType::Round,
        );
    }
    // SupportCommon.cpp:2196
    ret = polygons_simplify(&ret, scaled(0.015) as CoordF);

    // SupportCommon.cpp:2198
    if do_final_difference {
        ret = diff_polygons(&ret, collision_trimmed!());
    }
    // SupportCommon.cpp:2199
    union_polygons(&ret)
}

// SupportCommon.hpp:169-223 — templated ExPolygons overload.
/*
 * \brief Offsets (increases the area of) a polygons object in multiple steps to ensure that it does not lag through over a given obstacle.
 * (CollisionPolyType may be ExPolygons or Polygons. Here the collision is taken as Polygons to match the trimming path.)
 */
// SupportCommon.hpp:170
#[must_use]
pub fn safe_offset_inc_ex(
    me: &ExPolygons,
    distance: Coord,
    collision: &Polygons,
    safe_step_size: Coord,
    last_step_offset_without_check: Coord,
    min_amount_offset: usize,
) -> ExPolygons {
    // SupportCommon.hpp:173
    let mut do_final_difference = last_step_offset_without_check == 0;
    // SupportCommon.hpp:174 — ensure sane input
    let mut ret: ExPolygons = safe_union_ex(me, &Vec::new());

    // SupportCommon.hpp:176-182
    // Trim the collision polygons with the region of interest for diff() efficiency.
    let mut collision_trimmed_buffer: Polygons = Vec::new();
    let mut collision_trimmed_done = false;
    macro_rules! collision_trimmed {
        () => {{
            if !collision_trimmed_done {
                if collision_trimmed_buffer.is_empty() && !collision.is_empty() {
                    let bbox = get_extents_expolygons(&ret)
                        .expanded(std::cmp::max(0, distance) + scaled(SCALED_EPSILON));
                    let mut trimmed: Polygons = Vec::new();
                    for ex in polygons_to_expolygons(collision) {
                        trimmed.extend(clip_clipper_polygons_with_subject_bbox_expolygon(
                            &ex, &bbox, false,
                        ));
                    }
                    collision_trimmed_buffer = trimmed;
                }
                collision_trimmed_done = true;
            }
            &collision_trimmed_buffer
        }};
    }

    // SupportCommon.hpp:184
    if distance == 0 {
        return if do_final_difference {
            clipper_utils::difference(&ret, &polygons_to_expolygons(collision_trimmed!()))
        } else {
            clipper_utils::union_ex(&ret)
        };
    }
    // SupportCommon.hpp:185-188
    if safe_step_size < 0 || last_step_offset_without_check < 0 {
        // BOOST_LOG_TRIVIAL(error) << "Offset increase got invalid parameter!";
        return if do_final_difference {
            clipper_utils::difference(&ret, &polygons_to_expolygons(collision_trimmed!()))
        } else {
            clipper_utils::union_ex(&ret)
        };
    }

    // SupportCommon.hpp:190
    let mut step_size = safe_step_size;
    // SupportCommon.hpp:191
    let mut steps: i32 = if distance > last_step_offset_without_check {
        ((distance - last_step_offset_without_check) / step_size) as i32
    } else {
        0
    };
    // SupportCommon.hpp:192-198
    if distance - (steps as Coord) * step_size > last_step_offset_without_check {
        if ((steps as Coord) + 1) * step_size <= distance {
            steps += 1;
        } else {
            do_final_difference = true;
        }
    }
    // SupportCommon.hpp:199-209
    let extra_step: Coord =
        Coord::from(distance < last_step_offset_without_check || (distance % step_size) != 0);
    if (steps as Coord) + extra_step < min_amount_offset as Coord && min_amount_offset > 1 {
        step_size = distance / min_amount_offset as Coord;
        if step_size >= safe_step_size {
            step_size = safe_step_size;
            steps = min_amount_offset as i32;
        } else {
            steps = (distance / step_size) as i32;
        }
    }
    // SupportCommon.hpp:210-215 — offset in steps
    for i in 0..steps {
        ret = clipper_utils::difference(
            &offset_expolygons(&ret, step_size as CoordF, OffsetJoinType::Round),
            &polygons_to_expolygons(collision_trimmed!()),
        );
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if i % 10 == 7 {
            // C++: expolygons_simplify(ret, scaled<double>(0.015)). The Rust
            // `expolygons_simplify` scales its tolerance internally (mm in), so pass 0.015 mm.
            ret = expolygons_simplify(&ret, 0.015);
        }
    }
    // SupportCommon.hpp:216-218 — offset the remainder
    let last_offset = distance - (steps as Coord) * step_size;
    if last_offset > scaled(SCALED_EPSILON) {
        ret = offset_expolygons(
            &ret,
            (distance - (steps as Coord) * step_size) as CoordF,
            OffsetJoinType::Round,
        );
    }
    // SupportCommon.hpp:219
    // C++: expolygons_simplify(ret, scaled<double>(0.015)); pass mm (scaled internally).
    ret = expolygons_simplify(&ret, 0.015);

    // SupportCommon.hpp:221
    if do_final_difference {
        ret = clipper_utils::difference(&ret, &polygons_to_expolygons(collision_trimmed!()));
    }
    // SupportCommon.hpp:222
    clipper_utils::union_ex(&ret)
}
