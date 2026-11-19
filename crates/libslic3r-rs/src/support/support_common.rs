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
    self, clip_clipper_polygons_with_subject_bbox_expolygon, offset_expolygons_round,
    offset_polygons, offset_polygons_round, offset_polyline_miter, OffsetJoinType,
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
#[allow(dead_code)]
fn offset_polygons_pp(
    polygons: &[crate::geometry::Polygon],
    delta: CoordF,
    join_type: OffsetJoinType,
) -> Polygons {
    expolygons_to_polygons(&offset_polygons(polygons, delta, join_type))
}

// Slic3r::offset(const Polygons&, delta, ClipperLib::jtRound, scaled<float>(0.01)) -> Polygons
// Round join with an explicit arc tolerance of 0.01 mm (= scaled<float>(0.01)), matching
// SupportCommon.cpp:2189 / :2195.
fn offset_polygons_round_pp(polygons: &[crate::geometry::Polygon], delta: CoordF) -> Polygons {
    // arc_tolerance_mm = 0.01 mm, i.e. C++ scaled<float>(0.01).
    expolygons_to_polygons(&offset_polygons_round(polygons, delta, 0.01))
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

    if support_common_pin() {
        return safe_union_pin(first, second);
    }
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
                merged.extend(offset_polyline_miter(&pl, scaled(0.002) as CoordF, 1.2));
            }
            for pl in polygons_to_polylines(second) {
                merged.extend(offset_polyline_miter(&pl, scaled(0.002) as CoordF, 1.2));
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
    if support_common_pin() {
        return safe_union_ex_pin(first, second);
    }
    // SupportCommon.cpp:2124
    let mut result: ExPolygons = Vec::new();
    // SupportCommon.cpp:2125
    if !first.is_empty() || !second.is_empty() {
        // SupportCommon.cpp:2126 — union_ex(first, second). The two-arg ClipperUtils
        // union_ex (ClipperUtils.cpp:825-831) concatenates poly1 ++ poly2 and runs a
        // single-set union_ex over the whole input (NonZero), so do the same here:
        // this also merges overlaps when only one side is non-empty.
        let mut combined: ExPolygons = first.clone();
        combined.extend(second.iter().cloned());
        result = clipper_utils::union_ex(&combined);
        // SupportCommon.cpp:2127
        if result.is_empty() {
            // SupportCommon.cpp:2128 (debug log omitted)
            // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
            // SupportCommon.cpp:2130 — union_(offset(to_polylines(first), scaled<float>(0.002), jtMiter, 1.2), offset(to_polylines(second), ...))
            let mut merged: Polygons = Vec::new();
            for pl in expolygons_to_polylines(first) {
                merged.extend(offset_polyline_miter(&pl, scaled(0.002) as CoordF, 1.2));
            }
            for pl in expolygons_to_polylines(second) {
                merged.extend(offset_polyline_miter(&pl, scaled(0.002) as CoordF, 1.2));
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
    if support_common_pin() {
        return safe_offset_inc_pin(me, distance, collision, safe_step_size, last_step_offset_without_check, min_amount_offset);
    }
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
                    // C++: get_extents(ret).inflated(std::max(0, distance) + SCALED_EPSILON)
                    // SCALED_EPSILON is already a scaled coord_t (= scale_(EPSILON) = 10),
                    // so it must NOT be re-scaled. (libslic3r.h:84 #define SCALED_EPSILON scale_(EPSILON))
                    let bbox = get_extents_polygons(&ret)
                        .expanded(std::cmp::max(0, distance) + SCALED_EPSILON as Coord);
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
            &offset_polygons_round_pp(&ret, step_size as CoordF),
            collision_trimmed!(),
        );
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if i % 10 == 7 {
            ret = polygons_simplify(&ret, scaled(0.015) as CoordF);
        }
    }
    // SupportCommon.cpp:2193-2195 — offset the remainder
    let last_offset = distance - (steps as Coord) * step_size;
    // C++: if (last_offset > SCALED_EPSILON). SCALED_EPSILON is already a scaled
    // coord_t (= scale_(EPSILON) = 10); compare directly, do not re-scale.
    if last_offset > SCALED_EPSILON as Coord {
        ret = offset_polygons_round_pp(
            &ret,
            (distance - (steps as Coord) * step_size) as CoordF,
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
    if support_common_pin() {
        return safe_offset_inc_ex_pin(me, distance, collision, safe_step_size, last_step_offset_without_check, min_amount_offset);
    }
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
                    // C++: get_extents(ret).inflated(std::max(0, distance) + SCALED_EPSILON)
                    // SCALED_EPSILON is already a scaled coord_t (= scale_(EPSILON) = 10),
                    // so it must NOT be re-scaled. (libslic3r.h:84 #define SCALED_EPSILON scale_(EPSILON))
                    let bbox = get_extents_expolygons(&ret)
                        .expanded(std::cmp::max(0, distance) + SCALED_EPSILON as Coord);
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
            // C++: offset_ex(ret, step_size, ClipperLib::jtRound, scaled<float>(0.01)).
            &offset_expolygons_round(&ret, step_size as CoordF, 0.01),
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
    // C++: if (last_offset > SCALED_EPSILON). SCALED_EPSILON is already a scaled
    // coord_t (= scale_(EPSILON) = 10); compare directly, do not re-scale.
    if last_offset > SCALED_EPSILON as Coord {
        // C++: offset_ex(ret, distance - steps * step_size, ClipperLib::jtRound, scaled<float>(0.01)).
        ret = offset_expolygons_round(
            &ret,
            (distance - (steps as Coord) * step_size) as CoordF,
            0.01,
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

// ============================================================================
// safe_union / safe_offset_inc on ClipperLib, as the pin runs them.
// ============================================================================

/// Gate of the ClipperLib arms of `safe_union`, `safe_union_ex`, `safe_offset_inc` and `safe_offset_inc_ex`
/// (on by default; `SUPPORT_COMMON_PIN=0` restores the earlier arms, which run the booleans and offsets on
/// another clipper). Checked against the pin by tests/support_common_native_parity.rs.
pub fn support_common_pin() -> bool {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *G.get_or_init(|| crate::faithful_gate("SUPPORT_COMMON_PIN"))
}

// Polygon.hpp:233-240 `Polylines to_polylines(const Polygons &polygons)`: every polygon closed.
fn to_polylines_closed(polygons: &[Polygon]) -> Polylines {
    polygons.iter().map(crate::geometry::polygon::to_polyline).collect()
}

// `offset(to_polylines(...), scaled<float>(0.002), jtMiter, 1.2)` of safe_union().
fn safe_union_offset_polylines(polylines: &Polylines) -> Polygons {
    // raw_offset() of no paths is no paths
    if polylines.is_empty() {
        Vec::new()
    } else {
        cu::offset_polylines(
            polylines,
            (0.002 / crate::libslic3r::SCALING_FACTOR) as f32,
            crate::clipper::JoinType::jtMiter,
            1.2,
            crate::clipper::EndType::etOpenButt,
        )
    }
}

// SupportCommon.cpp:3004-3036 `Polygons safe_union(const Polygons first, const Polygons second)`
fn safe_union_pin(first: &Polygons, second: &Polygons) -> Polygons {
    let mut result: Polygons = Vec::new();
    if !first.is_empty() || !second.is_empty() {
        result = cu::union_2(first, second);
        if result.is_empty() {
            // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
            result = cu::union_2(
                &safe_union_offset_polylines(&to_polylines_closed(first)),
                &safe_union_offset_polylines(&to_polylines_closed(second)),
            );
        }
    }
    result
}

// SupportCommon.cpp:3037-3051 `ExPolygons safe_union(const ExPolygons first, const ExPolygons second)`
fn safe_union_ex_pin(first: &ExPolygons, second: &ExPolygons) -> ExPolygons {
    let mut result: ExPolygons = Vec::new();
    if !first.is_empty() || !second.is_empty() {
        result = cu_expolygon::union_ex_2(first, second);
        if result.is_empty() {
            // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
            let to_polylines = |src: &ExPolygons| -> Polylines {
                let mut out = Vec::new();
                for expoly in src {
                    out.extend(cu_expolygon::to_polylines_closed(expoly));
                }
                out
            };
            let result_polys = cu::union_2(&safe_union_offset_polylines(&to_polylines(first)), &safe_union_offset_polylines(&to_polylines(second)));
            for poly in result_polys {
                result.push(ExPolygon::new(poly));
            }
        }
    }
    result
}

// ExPolygon.hpp:451-458 `ExPolygons expolygons_simplify(const ExPolygons &expolys, double tolerance)`:
// `exp.simplify(tolerance, &out)` = `append(out, union_ex(exp.simplify_p(tolerance)))`, tolerance scaled.
fn expolygons_simplify_pin(expolys: &[ExPolygon], tolerance: f64) -> ExPolygons {
    let mut out: ExPolygons = Vec::with_capacity(expolys.len());
    for exp in expolys {
        out.extend(cu::union_ex(&clipper_utils::simplify_polygons_clib(&exp.simplify_p_dp_rings_faithful(tolerance), 1)));
    }
    out
}

/// The step plan of `safe_offset_inc` (SupportCommon.cpp:3082-3101, SupportCommon.hpp:190-209): the step size,
/// the number of steps and `do_final_difference`. coord_t is 32 bits (libslic3r.h:40), so is the arithmetic.
fn safe_offset_inc_steps(distance: i32, safe_step_size: i32, last_step_offset_without_check: i32, min_amount_offset: usize, do_final_difference: &mut bool) -> (i32, i32) {
    let mut step_size: i32 = safe_step_size;
    let mut steps: i32 = if distance > last_step_offset_without_check { (distance - last_step_offset_without_check) / step_size } else { 0 };
    if distance - steps * step_size > last_step_offset_without_check {
        if (steps + 1) * step_size <= distance {
            // This will be the case when last_step_offset_without_check >= safe_step_size
            steps += 1;
        } else {
            *do_final_difference = true;
        }
    }
    if steps + ((distance < last_step_offset_without_check || (distance % step_size) != 0) as i32) < min_amount_offset as i32 && min_amount_offset > 1 {
        // yes one can add a bool as the standard specifies that a result from compare operators has to be 0 or 1
        // reduce the stepsize to ensure it is offset the required amount of times
        // `distance / min_amount_offset`: the division is unsigned (size_t)
        step_size = ((distance as i64 as u64) / min_amount_offset as u64) as i32;
        if step_size >= safe_step_size {
            // effectivly reduce last_step_offset_without_check
            step_size = safe_step_size;
            steps = min_amount_offset as i32;
        } else {
            steps = distance / step_size;
        }
    }
    (step_size, steps)
}

// `get_extents(ret).inflated(std::max(0, distance) + SCALED_EPSILON)`: the sum is a double, truncated to the
// coord_t inflated() takes.
fn safe_offset_inc_bbox(mut bbox: crate::geometry::BoundingBox, distance: Coord) -> crate::geometry::BoundingBox {
    let delta = (0.max(distance) as f64 + 1e-4 / crate::libslic3r::SCALING_FACTOR) as Coord;
    bbox.min = crate::geometry::Point::new(bbox.min.x - delta, bbox.min.y - delta);
    bbox.max = crate::geometry::Point::new(bbox.max.x + delta, bbox.max.y + delta);
    bbox
}

// SupportCommon.cpp:3062-3115 `Polygons safe_offset_inc(const Polygons &me, coord_t distance, const Polygons
// &collision, coord_t safe_step_size, coord_t last_step_offset_without_check, size_t min_amount_offset)`
fn safe_offset_inc_pin(
    me: &Polygons,
    distance: Coord,
    collision: &Polygons,
    safe_step_size: Coord,
    last_step_offset_without_check: Coord,
    min_amount_offset: usize,
) -> Polygons {
    use crate::clipper::JoinType;
    use cu::ApplySafetyOffset::No;
    let mut do_final_difference = last_step_offset_without_check == 0;
    let mut ret: Polygons = safe_union_pin(me, &Vec::new()); // ensure sane input

    // Trim the collision polygons with the region of interest for diff() efficiency.
    let mut collision_trimmed_buffer: Polygons = Vec::new();
    macro_rules! collision_trimmed {
        () => {{
            if collision_trimmed_buffer.is_empty() && !collision.is_empty() {
                collision_trimmed_buffer =
                    clipper_utils::clip_clipper_polygons_with_subject_bbox_polygons(collision, &safe_offset_inc_bbox(get_extents_polygons(&ret), distance));
            }
            &collision_trimmed_buffer
        }};
    }
    // scaled<float>(0.01), the arc tolerance of jtRound
    let arc_tolerance = (0.01 / crate::libslic3r::SCALING_FACTOR) as f32 as f64;
    // scaled<double>(0.015)
    let simplify_tolerance = 0.015 / crate::libslic3r::SCALING_FACTOR;

    if distance == 0 {
        return if do_final_difference { cu::diff(&ret, collision_trimmed!(), No) } else { cu::union_(&ret) };
    }
    if safe_step_size < 0 || last_step_offset_without_check < 0 {
        // BOOST_LOG_TRIVIAL(error) << "Offset increase got invalid parameter!";
        return if do_final_difference { cu::diff(&ret, collision_trimmed!(), No) } else { cu::union_(&ret) };
    }

    let (step_size, steps) =
        safe_offset_inc_steps(distance as i32, safe_step_size as i32, last_step_offset_without_check as i32, min_amount_offset, &mut do_final_difference);
    let distance = distance as i32;
    // offset in steps
    for i in 0..steps {
        ret = cu::diff(&cu::offset(&ret, step_size as f32, JoinType::jtRound, arc_tolerance), collision_trimmed!(), No);
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if i % 10 == 7 {
            ret = polygons_simplify(&ret, simplify_tolerance);
        }
    }
    // offset the remainder
    let last_offset: f32 = (distance - steps * step_size) as f32;
    if last_offset as f64 > 1e-4 / crate::libslic3r::SCALING_FACTOR {
        ret = cu::offset(&ret, (distance - steps * step_size) as f32, JoinType::jtRound, arc_tolerance);
    }
    ret = polygons_simplify(&ret, simplify_tolerance);

    if do_final_difference {
        ret = cu::diff(&ret, collision_trimmed!(), No);
    }
    cu::union_(&ret)
}

// SupportCommon.hpp:169-223 `template<typename CollisionPolyType> ExPolygons safe_offset_inc(const ExPolygons
// &me, coord_t distance, const CollisionPolyType &collision, coord_t safe_step_size, coord_t
// last_step_offset_without_check, size_t min_amount_offset)` with CollisionPolyType = Polygons.
fn safe_offset_inc_ex_pin(
    me: &ExPolygons,
    distance: Coord,
    collision: &Polygons,
    safe_step_size: Coord,
    last_step_offset_without_check: Coord,
    min_amount_offset: usize,
) -> ExPolygons {
    use crate::clipper::JoinType;
    use cu::ApplySafetyOffset::No;
    let mut do_final_difference = last_step_offset_without_check == 0;
    let mut ret: ExPolygons = safe_union_ex_pin(me, &Vec::new()); // ensure sane input

    // Trim the collision polygons with the region of interest for diff() efficiency.
    let mut collision_trimmed_buffer: Polygons = Vec::new();
    macro_rules! collision_trimmed {
        () => {{
            if collision_trimmed_buffer.is_empty() && !collision.is_empty() {
                collision_trimmed_buffer =
                    clipper_utils::clip_clipper_polygons_with_subject_bbox_polygons(collision, &safe_offset_inc_bbox(get_extents_expolygons(&ret), distance));
            }
            &collision_trimmed_buffer
        }};
    }
    // scaled<float>(0.01), the arc tolerance of jtRound
    let arc_tolerance = (0.01 / crate::libslic3r::SCALING_FACTOR) as f32 as f64;
    // scaled<double>(0.015)
    let simplify_tolerance = 0.015 / crate::libslic3r::SCALING_FACTOR;

    if distance == 0 {
        return if do_final_difference { cu::diff_ex_ep(&ret, collision_trimmed!(), No) } else { cu::union_ex_expolygons(&ret) };
    }
    if safe_step_size < 0 || last_step_offset_without_check < 0 {
        // BOOST_LOG_TRIVIAL(error) << "Offset increase got invalid parameter!";
        return if do_final_difference { cu::diff_ex_ep(&ret, collision_trimmed!(), No) } else { cu::union_ex_expolygons(&ret) };
    }

    let (step_size, steps) =
        safe_offset_inc_steps(distance as i32, safe_step_size as i32, last_step_offset_without_check as i32, min_amount_offset, &mut do_final_difference);
    let distance = distance as i32;
    // offset in steps
    for i in 0..steps {
        ret = cu::diff_ex_ep(&cu::offset_ex_expolygons(&ret, step_size as f32, JoinType::jtRound, arc_tolerance), collision_trimmed!(), No);
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if i % 10 == 7 {
            ret = expolygons_simplify_pin(&ret, simplify_tolerance);
        }
    }
    // offset the remainder
    let last_offset: f32 = (distance - steps * step_size) as f32;
    if last_offset as f64 > 1e-4 / crate::libslic3r::SCALING_FACTOR {
        ret = cu::offset_ex_expolygons(&ret, (distance - steps * step_size) as f32, JoinType::jtRound, arc_tolerance);
    }
    ret = expolygons_simplify_pin(&ret, simplify_tolerance);

    if do_final_difference {
        ret = cu::diff_ex_ep(&ret, collision_trimmed!(), No);
    }
    cu::union_ex_expolygons(&ret)
}

// ============================================================================
// The ClipperUtils.cpp operations of SupportCommon.cpp and SupportMaterial.cpp.
// ============================================================================

/// The ClipperUtils.cpp / ClipperUtils.hpp operations SupportCommon.cpp and SupportMaterial.cpp
/// call, under their C++ names, on the ClipperLib port (`crate::clipper`) and the ClipperUtils
/// building blocks of `crate::clipper_utils` (`clipper_do`, `offset_paths`, `expand_paths`, ...).
/// The `*_provider` functions are the `ClipperUtils::*Provider` path sources.
pub mod cu {
    use crate::clipper::{ClipType, EndType, IntPoint, JoinType, Path, Paths, PolyFillType, PolyTree};
    use crate::clipper_utils as u;
    pub use crate::clipper_utils::ApplySafetyOffset;
    use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polygons, Polyline, Polylines};

    /// ClipperUtils.hpp:29 `ClipperSafetyOffset = 10.f`
    pub const CLIPPER_SAFETY_OFFSET: f32 = 10.;
    /// ClipperUtils.hpp:31 `DefaultJoinType = jtMiter`
    pub const DEFAULT_JOIN_TYPE: JoinType = JoinType::jtMiter;
    /// ClipperUtils.hpp:32 `DefaultEndType = etOpenButt`
    pub const DEFAULT_END_TYPE: EndType = EndType::etOpenButt;
    /// ClipperUtils.hpp:37 `DefaultMiterLimit = 3.`
    pub const DEFAULT_MITER_LIMIT: f64 = 3.;
    /// ClipperUtils.hpp:39 `DefaultLineJoinType = jtSquare`
    pub const DEFAULT_LINE_JOIN_TYPE: JoinType = JoinType::jtSquare;
    /// ClipperUtils.hpp:41 `DefaultLineMiterLimit = 0.`
    pub const DEFAULT_LINE_MITER_LIMIT: f64 = 0.;

    fn path_of(points: &[Point]) -> Path {
        points.iter().map(|p| IntPoint::new(p.x as _, p.y as _)).collect()
    }
    /// `ClipperUtils::SinglePathProvider`
    pub fn single_path_provider(points: &[Point]) -> Paths {
        vec![path_of(points)]
    }
    /// `ClipperUtils::PolygonsProvider`
    pub fn polygons_provider(polygons: &[Polygon]) -> Paths {
        polygons.iter().map(|p| path_of(&p.points)).collect()
    }
    /// `ClipperUtils::ExPolygonsProvider`: per expolygon its contour, then its holes.
    pub fn expolygons_provider(expolygons: &[ExPolygon]) -> Paths {
        let mut out = Paths::new();
        for expoly in expolygons {
            out.push(path_of(&expoly.contour.points));
            for hole in &expoly.holes {
                out.push(path_of(&hole.points));
            }
        }
        out
    }
    /// `ClipperUtils::PolylinesProvider`
    pub fn polylines_provider(polylines: &[Polyline]) -> Paths {
        polylines.iter().map(|p| path_of(&p.points)).collect()
    }
    fn points_of(path: &Path) -> Vec<Point> {
        path.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()
    }
    /// ClipperUtils.cpp `Polygons to_polygons(ClipperLib::Paths &&paths)`
    pub fn to_polygons(paths: &Paths) -> Polygons {
        paths.iter().map(|p| Polygon::from_points(points_of(p))).collect()
    }
    /// ClipperUtils.cpp `PolyTreeToPolylines` output as Slic3r polylines.
    fn to_polylines(paths: &Paths) -> Polylines {
        paths.iter().map(|p| Polyline::from_points(points_of(p))).collect()
    }
    /// An expolygon as the (contour, holes) pair the `crate::clipper_utils` offset helpers take.
    fn expolygon_paths(expolygons: &[ExPolygon]) -> Vec<(Path, Paths)> {
        expolygons
            .iter()
            .map(|e| (path_of(&e.contour.points), e.holes.iter().map(|h| path_of(&h.points)).collect()))
            .collect()
    }

    /// ClipperUtils.cpp:410 `offset(const Polygon &, float delta, JoinType, double miterLimit)`
    pub fn offset_polygon(polygon: &Polygon, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        to_polygons(&u::raw_offset(&single_path_provider(&polygon.points), delta as f64, join_type, miter_limit, EndType::etClosedPolygon))
    }
    /// ClipperUtils.cpp:413 `offset(const Polygons &, float delta, JoinType, double miterLimit)`
    pub fn offset(polygons: &[Polygon], delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        to_polygons(&u::offset_paths::<Paths>(&polygons_provider(polygons), delta as f64, join_type, miter_limit))
    }
    /// ClipperUtils.cpp:415 `offset_ex(const Polygons &, float delta, JoinType, double miterLimit)`
    pub fn offset_ex(polygons: &[Polygon], delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        u::poly_tree_to_ex_polygons(&u::offset_paths::<PolyTree>(&polygons_provider(polygons), delta as f64, join_type, miter_limit))
    }
    /// ClipperUtils.cpp:418 `offset(const Polyline &, float delta, JoinType, double miterLimit, EndType)`
    pub fn offset_polyline(polyline: &Polyline, delta: f32, join_type: JoinType, miter_limit: f64, end_type: EndType) -> Polygons {
        debug_assert!(delta > 0.);
        to_polygons(&u::clipper_union::<Paths>(
            &u::raw_offset_polyline(&single_path_provider(&polyline.points), delta as f64, join_type, miter_limit, end_type),
            PolyFillType::pftNonZero,
        ))
    }
    /// ClipperUtils.cpp:420 `offset(const Polylines &, float delta, JoinType, double miterLimit, EndType)`
    pub fn offset_polylines(polylines: &[Polyline], delta: f32, join_type: JoinType, miter_limit: f64, end_type: EndType) -> Polygons {
        debug_assert!(delta > 0.);
        to_polygons(&u::clipper_union::<Paths>(
            &u::raw_offset_polyline(&polylines_provider(polylines), delta as f64, join_type, miter_limit, end_type),
            PolyFillType::pftNonZero,
        ))
    }
    /// ClipperUtils.cpp:559 `offset(const ExPolygon &, float delta, JoinType, double miterLimit)`
    pub fn offset_expolygon(expolygon: &ExPolygon, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        let (contour, holes) = expolygon_paths(std::slice::from_ref(expolygon)).pop().unwrap();
        to_polygons(&u::expolygon_offset(&contour, &holes, delta as f64, join_type, miter_limit))
    }
    /// ClipperUtils.cpp:561 `offset(const ExPolygons &, float delta, JoinType, double miterLimit)`
    pub fn offset_expolygons(expolygons: &[ExPolygon], delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        to_polygons(&u::expolygons_offset(&expolygon_paths(expolygons), delta as f64, join_type, miter_limit))
    }
    /// ClipperUtils.cpp:570 `offset_ex(const ExPolygons &, float delta, JoinType, double miterLimit)`
    pub fn offset_ex_expolygons(expolygons: &[ExPolygon], delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        u::poly_tree_to_ex_polygons(&u::expolygons_offset_pt(&expolygon_paths(expolygons), delta as f64, join_type, miter_limit))
    }
    /// ClipperUtils.hpp:383 `expand(const Polygons &, float delta, JoinType, double miterLimit)`
    pub fn expand(polygons: &[Polygon], delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        debug_assert!(delta > 0.);
        offset(polygons, delta, join_type, miter_limit)
    }
    /// ClipperUtils.hpp:388 `shrink(const Polygons &, float delta, JoinType, double miterLimit)`
    pub fn shrink(polygons: &[Polygon], delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        debug_assert!(delta > 0.);
        offset(polygons, -delta, join_type, miter_limit)
    }
    /// ClipperUtils.hpp:390 `shrink_ex(const Polygons &, float delta, JoinType, double miterLimit)`
    pub fn shrink_ex(polygons: &[Polygon], delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        debug_assert!(delta > 0.);
        offset_ex(polygons, -delta, join_type, miter_limit)
    }
    /// ClipperUtils.cpp:592-597 `closing(const Polygons &, float delta1, float delta2, JoinType, double miterLimit)`
    pub fn closing(polygons: &[Polygon], delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        debug_assert!(delta1 > 0.);
        debug_assert!(delta2 > 0.);
        to_polygons(&u::shrink_paths::<Paths>(
            &u::expand_paths::<Paths>(&polygons_provider(polygons), delta1 as f64, join_type, miter_limit),
            delta2 as f64,
            join_type,
            miter_limit,
        ))
    }
    /// ClipperUtils.cpp:598-603 `closing_ex(const Polygons &, float delta1, float delta2, JoinType, double miterLimit)`
    pub fn closing_ex(polygons: &[Polygon], delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        debug_assert!(delta1 > 0.);
        debug_assert!(delta2 > 0.);
        u::poly_tree_to_ex_polygons(&u::shrink_paths::<PolyTree>(
            &u::expand_paths::<Paths>(&polygons_provider(polygons), delta1 as f64, join_type, miter_limit),
            delta2 as f64,
            join_type,
            miter_limit,
        ))
    }
    /// ClipperUtils.cpp:612-617 `opening(const Polygons &, float delta1, float delta2, JoinType, double miterLimit)`
    pub fn opening(polygons: &[Polygon], delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        debug_assert!(delta1 > 0.);
        debug_assert!(delta2 > 0.);
        to_polygons(&u::expand_paths::<Paths>(
            &u::shrink_paths::<Paths>(&polygons_provider(polygons), delta1 as f64, join_type, miter_limit),
            delta2 as f64,
            join_type,
            miter_limit,
        ))
    }
    /// ClipperUtils.cpp:581-584 `offset2_ex(const ExPolygons &, float delta1, float delta2, JoinType, double miterLimit)`
    pub fn offset2_ex(expolygons: &[ExPolygon], delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        u::poly_tree_to_ex_polygons(&u::offset_paths::<PolyTree>(
            &u::expolygons_offset(&expolygon_paths(expolygons), delta1 as f64, join_type, miter_limit),
            delta2 as f64,
            join_type,
            miter_limit,
        ))
    }
    /// ClipperUtils.hpp:370 `union_safety_offset(const Polygons &)`
    pub fn union_safety_offset(polygons: &[Polygon]) -> Polygons {
        offset(polygons, CLIPPER_SAFETY_OFFSET, DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT)
    }
    /// ClipperUtils.hpp:372 `union_safety_offset_ex(const Polygons &)`
    pub fn union_safety_offset_ex(polygons: &[Polygon]) -> ExPolygons {
        offset_ex(polygons, CLIPPER_SAFETY_OFFSET, DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT)
    }

    /// ClipperUtils.cpp:669-673 `_clipper(clipType, subject, clip, do_safety_offset)`
    pub fn clipper_paths(clip_type: ClipType, subject: &Paths, clip: &Paths, do_safety_offset: ApplySafetyOffset) -> Polygons {
        to_polygons(&u::clipper_do_safety_offset::<Paths>(clip_type, subject, clip, PolyFillType::pftNonZero, do_safety_offset))
    }
    /// ClipperUtils.cpp:677 `diff(const Polygons &, const Polygons &, ApplySafetyOffset)`
    pub fn diff(subject: &[Polygon], clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
        clipper_paths(ClipType::ctDifference, &polygons_provider(subject), &polygons_provider(clip), do_safety_offset)
    }
    /// ClipperUtils.cpp:687 `diff(const Polygons &, const ExPolygons &, ApplySafetyOffset)`
    pub fn diff_pe(subject: &[Polygon], clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
        clipper_paths(ClipType::ctDifference, &polygons_provider(subject), &expolygons_provider(clip), do_safety_offset)
    }
    /// ClipperUtils.cpp:701 `intersection(const Polygons &, const Polygons &, ApplySafetyOffset)`
    pub fn intersection(subject: &[Polygon], clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
        clipper_paths(ClipType::ctIntersection, &polygons_provider(subject), &polygons_provider(clip), do_safety_offset)
    }
    /// ClipperUtils.cpp:721 `union_(const Polygons &)`
    pub fn union_(subject: &[Polygon]) -> Polygons {
        clipper_paths(ClipType::ctUnion, &polygons_provider(subject), &Paths::new(), ApplySafetyOffset::No)
    }
    /// ClipperUtils.cpp:723 `union_(const ExPolygons &)`
    pub fn union_expolygons(subject: &[ExPolygon]) -> Polygons {
        clipper_paths(ClipType::ctUnion, &expolygons_provider(subject), &Paths::new(), ApplySafetyOffset::No)
    }
    /// ClipperUtils.cpp:727-734 `union_(const Polygons &, const Polygons &)` (BBS: the two
    /// concatenated, then `union_`).
    pub fn union_2(subject: &[Polygon], subject2: &[Polygon]) -> Polygons {
        let mut polys: Polygons = subject.to_vec();
        for poly in subject2 {
            polys.push(poly.clone());
        }
        union_(&polys)
    }
    /// ClipperUtils.cpp:813 `union_ex(const Polygons &, PolyFillType fill_type = pftNonZero)`
    pub fn union_ex(subject: &[Polygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctUnion, &polygons_provider(subject), &Paths::new(), ApplySafetyOffset::No, PolyFillType::pftNonZero)
    }
    /// ClipperUtils.cpp:815 `union_ex(const ExPolygons &)`
    pub fn union_ex_expolygons(subject: &[ExPolygon]) -> ExPolygons {
        u::poly_tree_to_ex_polygons(&u::clipper_do_polytree(ClipType::ctUnion, &expolygons_provider(subject), &Paths::new(), PolyFillType::pftNonZero))
    }
    /// ClipperUtils.cpp:741 `diff_ex(const Polygons &, const Polygons &, ApplySafetyOffset)`
    pub fn diff_ex(subject: &[Polygon], clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
        u::_clipper_ex(ClipType::ctDifference, &polygons_provider(subject), &polygons_provider(clip), do_safety_offset, PolyFillType::pftNonZero)
    }
    /// ClipperUtils.cpp:753 `diff_ex(const ExPolygons &, const Polygons &, ApplySafetyOffset)`
    pub fn diff_ex_ep(subject: &[ExPolygon], clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
        u::_clipper_ex(ClipType::ctDifference, &expolygons_provider(subject), &polygons_provider(clip), do_safety_offset, PolyFillType::pftNonZero)
    }
    /// ClipperUtils.cpp:755 `diff_ex(const ExPolygons &, const ExPolygons &, ApplySafetyOffset)`
    pub fn diff_ex_ee(subject: &[ExPolygon], clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
        u::_clipper_ex(ClipType::ctDifference, &expolygons_provider(subject), &expolygons_provider(clip), do_safety_offset, PolyFillType::pftNonZero)
    }
    /// ClipperUtils.cpp:788 `intersection_ex(const Polygons &, const Polygons &, ApplySafetyOffset)`
    pub fn intersection_ex(subject: &[Polygon], clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
        u::_clipper_ex(ClipType::ctIntersection, &polygons_provider(subject), &polygons_provider(clip), do_safety_offset, PolyFillType::pftNonZero)
    }
    /// ClipperUtils.cpp:794 `intersection_ex(const Polygons &, const ExPolygons &, ApplySafetyOffset)`
    pub fn intersection_ex_pe(subject: &[Polygon], clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
        u::_clipper_ex(ClipType::ctIntersection, &polygons_provider(subject), &expolygons_provider(clip), do_safety_offset, PolyFillType::pftNonZero)
    }
    /// ClipperUtils.cpp:911 `diff_pl(const Polylines &, const Polygons &)`
    pub fn diff_pl(subject: &[Polyline], clip: &[Polygon]) -> Polylines {
        to_polylines(&u::_clipper_pl_open(ClipType::ctDifference, &polylines_provider(subject), &polygons_provider(clip)))
    }
    /// ClipperUtils.cpp:935 `intersection_pl(const Polylines &, const ExPolygons &)`
    pub fn intersection_pl_ex(subject: &[Polyline], clip: &[ExPolygon]) -> Polylines {
        to_polylines(&u::_clipper_pl_open(ClipType::ctIntersection, &polylines_provider(subject), &expolygons_provider(clip)))
    }
    /// ClipperUtils.cpp `intersection_pl(const Polylines &, const Polygons &)`
    pub fn intersection_pl(subject: &[Polyline], clip: &[Polygon]) -> Polylines {
        to_polylines(&u::_clipper_pl_open(ClipType::ctIntersection, &polylines_provider(subject), &polygons_provider(clip)))
    }
}

/// ExPolygon.cpp:109-119, 158-195 and ClipperUtils.cpp `offset_ex(const ExPolygon &, ...)`: the
/// expolygon predicates and the one-expolygon offset SupportMaterial.cpp's overhang clusters use.
pub mod cu_expolygon {
    use super::cu;
    use crate::clipper::{ClipType, JoinType};
    use crate::clipper_utils as u;
    use crate::geometry::{contains_polygon, ExPolygon, ExPolygons, Point, Polyline, Polylines};

    /// ExPolygon.hpp:214-229 `to_polylines(const ExPolygon &)`: every ring closed.
    pub fn to_polylines_closed(src: &ExPolygon) -> Polylines {
        let mut polylines: Polylines = Vec::with_capacity(src.holes.len() + 1);
        let mut pl = Polyline::from_points(src.contour.points.clone());
        pl.points.push(pl.points[0]);
        polylines.push(pl);
        for hole in &src.holes {
            let mut pl = Polyline::from_points(hole.points.clone());
            pl.points.push(hole.points[0]);
            polylines.push(pl);
        }
        polylines
    }
    /// ExPolygon.cpp:109-119 `ExPolygon::contains(const Point &point, bool border_result = true)`
    pub fn contains(expoly: &ExPolygon, point: &Point, border_result: bool) -> bool {
        if !contains_polygon(&expoly.contour, point, border_result) {
            // Outside the outer contour, not on the contour boundary.
            return false;
        }
        for hole in &expoly.holes {
            if contains_polygon(hole, point, !border_result) {
                // Inside a hole, not on the hole boundary.
                return false;
            }
        }
        true
    }
    /// ExPolygon.cpp:158-184 `ExPolygon::overlaps(const ExPolygon &other)`
    pub fn overlaps(this: &ExPolygon, other: &ExPolygon) -> bool {
        if this.contour.points.is_empty() || other.contour.points.is_empty() {
            return false;
        }
        // intersection_pl(to_polylines(other), *this)
        let pl_out = u::_clipper_pl_open(
            ClipType::ctIntersection,
            &cu::polylines_provider(&to_polylines_closed(other)),
            &cu::expolygons_provider(std::slice::from_ref(this)),
        );
        // See unit test SCENARIO("Clipper diff with polyline", "[Clipper]")
        // for in which case the intersection_pl produces any intersection.
        !pl_out.is_empty() ||
            // If *this is completely inside other, then pl_out is empty, but the expolygons overlap. Test for that situation.
            contains(other, &this.contour.points[0], true)
    }
    /// ExPolygon.cpp:186-195 `overlaps(const ExPolygons &expolys1, const ExPolygons &expolys2)`
    pub fn overlaps_expolygons(expolys1: &[ExPolygon], expolys2: &[ExPolygon]) -> bool {
        for expoly1 in expolys1 {
            for expoly2 in expolys2 {
                if overlaps(expoly1, expoly2) {
                    return true;
                }
            }
        }
        false
    }
    /// ClipperUtils.cpp:567-569 `offset_ex(const ExPolygon &, float delta, JoinType, double miterLimit)`:
    /// `ClipperPaths_to_Slic3rExPolygons(expolygon_offset(...))`, an even-odd union.
    pub fn offset_ex(expolygon: &ExPolygon, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        let contour = cu::single_path_provider(&expolygon.contour.points).pop().unwrap();
        let holes = cu::polygons_provider(&expolygon.holes);
        u::clipper_paths_to_slic3r_ex_polygons(&u::expolygon_offset(&contour, &holes, delta as f64, join_type, miter_limit), false)
    }
    /// ClipperUtils.cpp:825-831 `union_ex(const ExPolygons &poly1, const ExPolygons &poly2, bool)` (BBS: the
    /// two concatenated, then `union_ex`).
    pub fn union_ex_2(poly1: &[ExPolygon], poly2: &[ExPolygon]) -> ExPolygons {
        let mut expolys: ExPolygons = poly1.to_vec();
        for expoly in poly2 {
            expolys.push(expoly.clone());
        }
        cu::union_ex_expolygons(&expolys)
    }
}

// ============================================================================
// SupportCommon.cpp at the pin, over the Print classes of `crate::print::faithful`.
//
// A `SupportGeneratorLayer*` is the index of the layer in the `SupportGeneratorLayerStorage`
// (`SupportGeneratorLayersPtr` is a vector of them); a null pointer is `NULL_LAYER`. The storage hands out
// the indices in the order of the allocations, which is the order of the addresses of the C++ deque
// within one of its blocks.
// ============================================================================

use crate::clipper::JoinType;
use crate::geometry::Point;
use crate::print::faithful as pf;
use crate::print_config::config_def::DynamicPrintConfig;
use crate::print_config::enums;
use crate::slicing::SlicingParams as SlicingParameters;
use crate::support::support_layer::{SupporLayerType, SupportGeneratorLayer, SupportGeneratorLayerStorage, SupportGeneratorLayersPtr};
use crate::support::support_parameters::SupportParameters;

/// A null `SupportGeneratorLayer*` of a `SupportGeneratorLayersPtr`.
pub const NULL_LAYER: usize = usize::MAX;

/// libslic3r.h `scale_(val)`: `val / SCALING_FACTOR`, a double.
#[inline]
fn scale_(val: f64) -> f64 {
    val / crate::libslic3r::SCALING_FACTOR
}

/// libslic3r.h `EPSILON`
const EPSILON: f64 = crate::libslic3r::EPSILON;

/// libslic3r.h `is_approx(x, y, precision = EPSILON)`
#[inline]
fn is_approx(x: f64, y: f64, precision: f64) -> bool {
    (x - y).abs() < precision
}

// `config.<key>.value` of a float or percent option.
fn cfg_float(config: &DynamicPrintConfig, key: &str) -> f64 {
    use crate::config::ConfigOptionResolver;
    config.option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
}

// `config.<key>.value` of an int or enum option.
fn cfg_int(config: &DynamicPrintConfig, key: &str) -> i32 {
    use crate::config::ConfigOptionResolver;
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
}

// `config.<key>.value` of a bool option.
fn cfg_bool(config: &DynamicPrintConfig, key: &str) -> bool {
    use crate::config::ConfigOptionResolver;
    config.option(key).and_then(|o| o.get_bool().ok()).unwrap_or(false)
}

/// SupportLayer.hpp:75-88 `void SupportGeneratorLayer::merge(SupportGeneratorLayer &&rhs)` on ClipperLib
/// (`union_(const Polygons &, const Polygons &)`, ClipperUtils.cpp:727-734).
///
/// FIDELITY-NOTE: `SupportGeneratorLayer::merge` of support_layer.rs unions on another clipper; this is
/// the function SupportMaterial.cpp's `merge_contact_layers` needs.
pub fn support_generator_layer_merge(layer_storage: &mut SupportGeneratorLayerStorage, this: usize, rhs: usize) {
    let mut rhs_layer = std::mem::take(&mut layer_storage[rhs]);
    let this_layer = &mut layer_storage[this];
    // The union_() does not support move semantic yet, but maybe one day it will.
    this_layer.polygons = cu::union_2(&this_layer.polygons, &rhs_layer.polygons);
    let merge = |dst: &mut Option<Polygons>, src: &mut Option<Polygons>| {
        if dst.is_none() || dst.as_ref().unwrap().is_empty() {
            *dst = src.take();
        } else if src.as_ref().map(|s| !s.is_empty()).unwrap_or(false) {
            let merged = cu::union_2(dst.as_ref().unwrap(), src.as_ref().unwrap());
            *dst = Some(merged);
        }
    };
    merge(&mut this_layer.contact_polygons, &mut rhs_layer.contact_polygons);
    merge(&mut this_layer.overhang_polygons, &mut rhs_layer.overhang_polygons);
    merge(&mut this_layer.enforcer_polygons, &mut rhs_layer.enforcer_polygons);
    // rhs.reset(): the layer left behind is a default one (std::mem::take above).
}

// SupportCommon.cpp:51-56
// Drop null and empty layers so the compacted vector only carries real support geometry.
pub fn compact_layers(vec: &mut SupportGeneratorLayersPtr, layer_storage: &SupportGeneratorLayerStorage) {
    vec.retain(|&ptr| !(ptr == NULL_LAYER || layer_storage[ptr].polygons.is_empty()));
}

// SupportCommon.cpp:88-128, the lambda `insert_layer` of generate_interface_layers().
// Insert a new layer into base_interface_layers, if intersection with base exists.
#[allow(clippy::too_many_arguments)]
fn generate_interface_layers_insert_layer(
    layer_storage: &mut SupportGeneratorLayerStorage,
    smooth_supports: bool,
    closing_distance: f64,
    smoothing_distance: f64,
    minimum_island_radius: f64,
    intermediate_layer: usize,
    bottom: &mut Polygons,
    top: Polygons,
    top_interface_layer: Option<usize>,
    subtract: Option<usize>,
    r#type: SupporLayerType,
) -> Option<usize> {
    let has_top_interface = top_interface_layer.map(|l| !layer_storage[l].polygons.is_empty()).unwrap_or(false);
    debug_assert!(!bottom.is_empty() || !top.is_empty() || has_top_interface);
    // Merge top into bottom, unite them with a safety offset.
    bottom.extend(top);
    // Merge top / bottom interfaces. For snug supports, merge using closing distance and regularize (close concave corners).
    *bottom = cu::intersection(
        &if smooth_supports {
            crate::mutable_polygon::smooth_outward_polygons(
                // closing(Polygons, float delta1, float delta2, SUPPORT_SURFACES_OFFSET_PARAMETERS)
                cu::closing(bottom, (closing_distance + minimum_island_radius) as f32, closing_distance as f32, JoinType::jtSquare, 0.),
                // smooth_outward(Polygons, coord_t clip_dist_scaled): the double truncated to the 32 bit coord_t
                smoothing_distance as i32 as Coord,
            )
        } else {
            cu::union_safety_offset(bottom)
        },
        &layer_storage[intermediate_layer].polygons,
        cu::ApplySafetyOffset::No,
    );
    if has_top_interface {
        // Don't trim the precomputed Organic supports top interface with base layer
        // as the precomputed top interface likely expands over multiple tree tips.
        let top_interface_layer = top_interface_layer.unwrap();
        *bottom = cu::union_2(&layer_storage[top_interface_layer].polygons, bottom);
        layer_storage[top_interface_layer].polygons.clear();
    }
    if !bottom.is_empty() {
        //FIXME Remove non-printable tiny islands, let them be printed using the base support.
        //bottom = opening(std::move(bottom), minimum_island_radius);
        if !bottom.is_empty() {
            let layer_new = match top_interface_layer {
                Some(l) => l,
                None => {
                    let idx = layer_storage.len();
                    layer_storage.allocate(r#type);
                    idx
                }
            };
            let (print_z, bottom_z, height, bridging) = {
                let il = &layer_storage[intermediate_layer];
                (il.print_z, il.bottom_z, il.height, il.bridging)
            };
            {
                let l = &mut layer_storage[layer_new];
                l.polygons = std::mem::take(bottom);
                l.print_z = print_z;
                l.bottom_z = bottom_z;
                l.height = height;
                l.bridging = bridging;
            }
            // Subtract the interface from the base regions.
            let trimmed = cu::diff(&layer_storage[intermediate_layer].polygons, &layer_storage[layer_new].polygons, cu::ApplySafetyOffset::No);
            layer_storage[intermediate_layer].polygons = trimmed;
            if let Some(subtract) = subtract {
                // Trim the base interface layer with the interface layer.
                let trimmed = cu::diff(&layer_storage[layer_new].polygons, &layer_storage[subtract].polygons, cu::ApplySafetyOffset::No);
                layer_storage[layer_new].polygons = trimmed;
            }
            //FIXME filter layer_new.polygons islands by a minimum area?
            //                  $interface_area = [ grep abs($_->area) >= $area_threshold, @$interface_area ];
            return Some(layer_new);
        }
    }
    None
}

// SupportCommon.cpp:58-298
// Convert some of the intermediate layers into top/bottom interface layers as well as base interface layers.
//
// The two `tbb::parallel_for` run in the order of the layers. The second one leaves its sub-range at the
// first layer without a base interface (`return` in the body of the range, SupportCommon.cpp:253); see the
// note at that loop.
#[allow(clippy::too_many_arguments)]
pub fn generate_interface_layers(
    config: &pf::PrintObjectConfig,
    support_params: &SupportParameters,
    bottom_contacts: &SupportGeneratorLayersPtr,
    top_contacts: &SupportGeneratorLayersPtr,
    // Input / output, will be merged with output. Only provided for Organic supports.
    top_interface_layers: &mut SupportGeneratorLayersPtr,
    top_base_interface_layers: &mut SupportGeneratorLayersPtr,
    // Input, will be trimmed with the newly created interface layers.
    intermediate_layers: &SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
) -> (SupportGeneratorLayersPtr, SupportGeneratorLayersPtr) {
    let mut base_and_interface_layers: (SupportGeneratorLayersPtr, SupportGeneratorLayersPtr) = (Vec::new(), Vec::new());

    if !intermediate_layers.is_empty() && support_params.has_interfaces() {
        // For all intermediate layers, collect top contact surfaces, which are not further than support_material_interface_layers.
        let snug_supports = support_params.support_style == enums::SupportMaterialStyle::SmsSnug as i32;
        let smooth_supports = support_params.support_style != enums::SupportMaterialStyle::SmsGrid as i32;
        let mut interface_layers: SupportGeneratorLayersPtr = vec![NULL_LAYER; intermediate_layers.len()];
        let mut base_interface_layers: SupportGeneratorLayersPtr = Vec::new();
        if support_params.has_base_interfaces() {
            base_interface_layers = vec![NULL_LAYER; intermediate_layers.len()];
        }
        let smoothing_distance = support_params.support_material_interface_flow.scaled_spacing() as f64 * 1.5;
        let minimum_island_radius = support_params.support_material_interface_flow.scaled_spacing() as f64 / support_params.interface_density;
        let closing_distance = smoothing_distance; // scaled<float>(config.support_material_closing_radius.value);
        let istree = crate::print_config::config_def::is_tree(enums::SupportType::from_i32(cfg_int(config, "support_type")));
        {
            // Gather the top / bottom contact layers intersecting with num_interface_layers resp. num_interface_layers_only intermediate layers above / below
            // this intermediate layer.
            // Index of the first top contact layer intersecting the current intermediate layer.
            let mut idx_top_contact_first: isize = -1;
            // Index of the first bottom contact layer intersecting the current intermediate layer.
            let mut idx_bottom_contact_first: isize = -1;
            // Index of the first top interface layer intersecting the current intermediate layer.
            let mut idx_top_interface_first: isize = -1;
            // Index of the first top contact interface layer intersecting the current intermediate layer.
            let mut idx_top_base_interface_first: isize = -1;
            let num_intermediate = intermediate_layers.len() as i32;
            let num_top_interface_layers = support_params.num_top_interface_layers as i32;
            let num_top_base_interface_layers = support_params.num_top_base_interface_layers as i32;
            let overhang_or_polygons = |layer: &SupportGeneratorLayer| -> Polygons {
                if snug_supports {
                    layer.overhang_polygons.as_ref().expect("overhang_polygons of a top contact layer").clone()
                } else {
                    layer.polygons.clone()
                }
            };
            for idx_intermediate_layer in 0..num_intermediate {
                let intermediate_layer = intermediate_layers[idx_intermediate_layer as usize];
                let (intermediate_print_z, intermediate_bottom_z) = (layer_storage[intermediate_layer].print_z, layer_storage[intermediate_layer].bottom_z);
                let mut polygons_top_contact_projected_interface: Polygons = Vec::new();
                let mut polygons_top_contact_projected_base: Polygons = Vec::new();
                let mut polygons_bottom_contact_projected_interface: Polygons = Vec::new();
                let mut polygons_bottom_contact_projected_base: Polygons = Vec::new();
                if support_params.num_top_interface_layers > 0 {
                    if istree {
                        // Top Z coordinate of a slab, over which we are collecting the top / bottom contact surfaces
                        let top_z = layer_storage
                            [intermediate_layers[std::cmp::min(num_intermediate - 1, idx_intermediate_layer + num_top_interface_layers - 1) as usize]]
                            .print_z;
                        let mut top_inteface_z = f64::MAX;
                        if support_params.num_top_base_interface_layers > 0 {
                            // Some top base interface layers will be generated.
                            top_inteface_z = if support_params.num_top_interface_layers_only() == 0 {
                                // Only base interface layers to generate.
                                -f64::MAX
                            } else {
                                layer_storage[intermediate_layers[std::cmp::min(
                                    num_intermediate - 1,
                                    idx_intermediate_layer + support_params.num_top_interface_layers_only() as i32 - 1,
                                ) as usize]]
                                    .print_z
                            };
                        }
                        // Move idx_top_contact_first up until above the current print_z.
                        idx_top_contact_first =
                            idx_higher_or_equal(top_contacts, idx_top_contact_first, |&layer| layer_storage[layer].print_z >= intermediate_print_z); //  - EPSILON
                        // Collect the top contact areas above this intermediate layer, below top_z.
                        for idx_top_contact in idx_top_contact_first..top_contacts.len() as isize {
                            let top_contact_layer = &layer_storage[top_contacts[idx_top_contact as usize]];
                            // FIXME maybe this adds one interface layer in excess?
                            if top_contact_layer.bottom_z - EPSILON > top_z {
                                break;
                            }
                            // For snug supports, project the overhang polygons covering the whole overhang, so that they will merge without a gap with support
                            // polygons of the other layers. For grid supports, merging of support regions will be performed by the projection into grid.
                            let src = overhang_or_polygons(top_contact_layer);
                            if top_contact_layer.bottom_z - EPSILON > top_inteface_z {
                                polygons_top_contact_projected_base.extend(src);
                            } else {
                                polygons_top_contact_projected_interface.extend(src);
                            }
                        }
                    } else {
                        // Move idx_top_contact_first up until above the current print_z.
                        idx_top_contact_first =
                            idx_higher_or_equal(top_contacts, idx_top_contact_first, |&layer| layer_storage[layer].print_z >= intermediate_print_z); //  - EPSILON
                        // Collect the top contact areas above this intermediate layer, below top_z.
                        for idx_top_contact in idx_top_contact_first..top_contacts.len() as isize {
                            let top_contact_layer = &layer_storage[top_contacts[idx_top_contact as usize]];
                            let is_top_contact =
                                is_approx(top_contact_layer.bottom_z, layer_storage[intermediate_layers[(num_intermediate - 1) as usize]].print_z, EPSILON);
                            if is_top_contact {
                                if idx_intermediate_layer > num_intermediate - num_top_interface_layers {
                                    polygons_top_contact_projected_interface.extend(overhang_or_polygons(top_contact_layer));
                                } else if idx_intermediate_layer > num_intermediate - num_top_interface_layers - num_top_base_interface_layers {
                                    polygons_top_contact_projected_base.extend(overhang_or_polygons(top_contact_layer));
                                }
                            } else {
                                if top_contact_layer.print_z - EPSILON
                                    < layer_storage[intermediate_layers
                                        [std::cmp::min(idx_intermediate_layer + num_top_interface_layers - 1, num_intermediate - 1) as usize]]
                                        .print_z
                                {
                                    polygons_top_contact_projected_interface.extend(overhang_or_polygons(top_contact_layer));
                                } else if top_contact_layer.print_z - EPSILON
                                    < layer_storage[intermediate_layers[std::cmp::min(
                                        idx_intermediate_layer + num_top_interface_layers - 1 + num_top_base_interface_layers,
                                        num_intermediate - 1,
                                    ) as usize]]
                                        .print_z
                                {
                                    polygons_top_contact_projected_base.extend(overhang_or_polygons(top_contact_layer));
                                }
                            }
                        }
                    }
                }
                if support_params.num_bottom_interface_layers > 0 {
                    // Bottom Z coordinate of a slab, over which we are collecting the top / bottom contact surfaces
                    let bottom_z = layer_storage
                        [intermediate_layers[std::cmp::max(0, idx_intermediate_layer - support_params.num_bottom_interface_layers as i32 + 1) as usize]]
                        .bottom_z;
                    let mut bottom_interface_z = -f64::MAX;
                    if support_params.num_bottom_base_interface_layers > 0 {
                        // Some bottom base interface layers will be generated.
                        bottom_interface_z = if support_params.num_bottom_interface_layers_only() == 0 {
                            // Only base interface layers to generate.
                            f64::MAX
                        } else {
                            layer_storage[intermediate_layers
                                [std::cmp::max(0, idx_intermediate_layer - support_params.num_bottom_interface_layers_only() as i32) as usize]]
                                .bottom_z
                        };
                    }
                    // Move idx_bottom_contact_first up until touching bottom_z.
                    idx_bottom_contact_first =
                        idx_higher_or_equal(bottom_contacts, idx_bottom_contact_first, |&layer| layer_storage[layer].print_z >= bottom_z - EPSILON);
                    // Collect the top contact areas above this intermediate layer, below top_z.
                    for idx_bottom_contact in idx_bottom_contact_first..bottom_contacts.len() as isize {
                        let bottom_contact_layer = &layer_storage[bottom_contacts[idx_bottom_contact as usize]];
                        if bottom_contact_layer.print_z - EPSILON > intermediate_bottom_z {
                            break;
                        }
                        if bottom_contact_layer.print_z - EPSILON > bottom_interface_z {
                            polygons_bottom_contact_projected_interface.extend(bottom_contact_layer.polygons.iter().cloned());
                        } else {
                            polygons_bottom_contact_projected_base.extend(bottom_contact_layer.polygons.iter().cloned());
                        }
                    }
                }
                let resolve_same_layer = |layers: &SupportGeneratorLayersPtr, idx: &mut isize, print_z: f64, layer_storage: &SupportGeneratorLayerStorage| -> Option<usize> {
                    if !layers.is_empty() {
                        *idx = idx_higher_or_equal(layers, *idx, |&layer| layer_storage[layer].print_z > print_z - EPSILON);
                        if *idx < layers.len() as isize && layer_storage[layers[*idx as usize]].print_z < print_z + EPSILON {
                            return Some(layers[*idx as usize]);
                        }
                    }
                    None
                };
                let top_interface_layer = resolve_same_layer(top_interface_layers, &mut idx_top_interface_first, intermediate_print_z, layer_storage);
                let top_base_interface_layer =
                    resolve_same_layer(top_base_interface_layers, &mut idx_top_base_interface_first, intermediate_print_z, layer_storage);
                let mut interface_layer: Option<usize> = None;
                if !polygons_bottom_contact_projected_interface.is_empty()
                    || !polygons_top_contact_projected_interface.is_empty()
                    || top_interface_layer.map(|l| !layer_storage[l].polygons.is_empty()).unwrap_or(false)
                {
                    let r#type =
                        if polygons_top_contact_projected_interface.is_empty() { SupporLayerType::SltBottomInterface } else { SupporLayerType::SltTopInterface };
                    interface_layer = generate_interface_layers_insert_layer(
                        layer_storage,
                        smooth_supports,
                        closing_distance,
                        smoothing_distance,
                        minimum_island_radius,
                        intermediate_layer,
                        &mut polygons_bottom_contact_projected_interface,
                        std::mem::take(&mut polygons_top_contact_projected_interface),
                        top_interface_layer,
                        None,
                        r#type,
                    );
                    interface_layers[idx_intermediate_layer as usize] = interface_layer.unwrap_or(NULL_LAYER);
                }
                if !polygons_bottom_contact_projected_base.is_empty()
                    || !polygons_top_contact_projected_base.is_empty()
                    || top_base_interface_layer.map(|l| !layer_storage[l].polygons.is_empty()).unwrap_or(false)
                {
                    base_interface_layers[idx_intermediate_layer as usize] = generate_interface_layers_insert_layer(
                        layer_storage,
                        smooth_supports,
                        closing_distance,
                        smoothing_distance,
                        minimum_island_radius,
                        intermediate_layer,
                        &mut polygons_bottom_contact_projected_base,
                        std::mem::take(&mut polygons_top_contact_projected_base),
                        top_base_interface_layer,
                        interface_layer,
                        SupporLayerType::SltBase,
                    )
                    .unwrap_or(NULL_LAYER);
                }
            }
        }

        if support_params.num_top_base_interface_layers > 1 && !istree {
            // FIDELITY-NOTE: the body of this `tbb::parallel_for` leaves its sub-range with `return` at the
            // first layer without a base interface (SupportCommon.cpp:253), so the result depends on how the
            // auto partitioner cuts [1, size): oneTBB (partitioner.h, `auto_partition_type`, initial divisor
            // 2 * max_concurrency, `__TBB_INIT_DEPTH` 5) bisects down to sub-ranges of one layer for every
            // range of up to about 64 layers on one thread and about 64 * threads otherwise, where the
            // `return` skips that one layer only. That is what is ported here; a longer range on few threads
            // leaves sub-ranges of two or more layers, whose rest the pin skips.
            for layer_id in 1..base_interface_layers.len() {
                let base_interface_layer = base_interface_layers[layer_id];
                if base_interface_layer == NULL_LAYER {
                    // `return` of the body of a sub-range of one layer
                    continue;
                }

                for &contact in top_contacts.iter() {
                    if is_approx(
                        layer_storage[contact].bottom_z,
                        // size_t arithmetic
                        layer_storage[intermediate_layers
                            [std::cmp::min((layer_id - 1).wrapping_add(support_params.num_top_interface_layers), intermediate_layers.len() - 1)]]
                            .print_z,
                        0.01,
                    ) && crate::geometry::polygon::overlaps(&layer_storage[base_interface_layer].polygons, &layer_storage[contact].polygons)
                    {
                        layer_storage[base_interface_layer].up = true;
                        break;
                    }
                }
            }
        }

        // Compress contact_out, remove the nullptr items.
        // The parallel_for above may not have merged all the interface and base_interface layers
        // generated by the Organic supports code, do it here.
        let merge_remove_empty = |in1: &mut SupportGeneratorLayersPtr, in2: &mut SupportGeneratorLayersPtr, layer_storage: &SupportGeneratorLayerStorage| -> SupportGeneratorLayersPtr {
            compact_layers(in1, layer_storage);
            compact_layers(in2, layer_storage);
            if in2.is_empty() {
                std::mem::take(in1)
            } else if in1.is_empty() {
                std::mem::take(in2)
            } else {
                // std::merge(in1, in2, [](auto* l, auto* r) { return l->print_z < r->print_z; })
                let mut out: SupportGeneratorLayersPtr = Vec::with_capacity(in1.len() + in2.len());
                let (mut i, mut j) = (0, 0);
                while i < in1.len() && j < in2.len() {
                    if layer_storage[in2[j]].print_z < layer_storage[in1[i]].print_z {
                        out.push(in2[j]);
                        j += 1;
                    } else {
                        out.push(in1[i]);
                        i += 1;
                    }
                }
                out.extend_from_slice(&in1[i..]);
                out.extend_from_slice(&in2[j..]);
                out
            }
        };
        base_and_interface_layers.0 = merge_remove_empty(&mut interface_layers, top_interface_layers, layer_storage);
        base_and_interface_layers.1 = merge_remove_empty(&mut base_interface_layers, top_base_interface_layers, layer_storage);
    } else if support_params.has_interfaces() {
        // Organic supports can place roofs that have no tree body underneath at all, which happens for
        // overhangs sitting only a couple of layers above the bed. The loop above is driven by
        // intermediate_layers, so an empty base would drop the already computed roofs and leave a single
        // contact layer floating above empty layers. Pass them through instead.
        compact_layers(top_interface_layers, layer_storage);
        compact_layers(top_base_interface_layers, layer_storage);
        base_and_interface_layers.0 = std::mem::take(top_interface_layers);
        base_and_interface_layers.1 = std::mem::take(top_base_interface_layers);
    }

    base_and_interface_layers
}

// SupportCommon.cpp:300-476
// Generate raft layers, also expand the 1st support layer
// in case there is no raft layer to improve support adhesion.
#[allow(clippy::too_many_arguments)]
pub fn generate_raft_base(
    object: &pf::PrintObject,
    support_params: &SupportParameters,
    slicing_params: &SlicingParameters,
    top_contacts: &SupportGeneratorLayersPtr,
    interface_layers: &SupportGeneratorLayersPtr,
    base_interface_layers: &SupportGeneratorLayersPtr,
    base_layers: &SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
) -> SupportGeneratorLayersPtr {
    use cu::ApplySafetyOffset::No;
    // If there is brim to be generated, calculate the trimming regions.
    let mut brim: Polygons = Vec::new();
    if object.has_brim() {
        // The object does not have a raft.
        // Calculate the area covered by the brim.
        let brim_type = cfg_int(object.config(), "brim_type");
        let brim_outer = brim_type == enums::BrimType::BtOuterOnly as i32 || brim_type == enums::BrimType::BtOuterAndInner as i32;
        let brim_inner = brim_type == enums::BrimType::BtInnerOnly as i32 || brim_type == enums::BrimType::BtOuterAndInner as i32;
        // BBS: the pattern of raft and brim are the same, thus the brim can be serpated by support raft.
        // scaled<float>(object.config().brim_object_gap.value)
        let brim_object_gap = scale_(cfg_float(object.config(), "brim_object_gap")) as f32;
        //const auto     brim_object_gap = scaled<float>(object.config().brim_object_gap.value + object.config().brim_width.value);
        // float(scale_(0.1))
        let arc_tolerance = scale_(0.1) as f32 as f64;
        for ex in &object.layers()[0].lslices {
            if brim_outer && brim_inner {
                brim.extend(cu::offset_expolygon(ex, brim_object_gap, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
            } else {
                if brim_outer {
                    brim.extend(cu::offset_polygon(&ex.contour, brim_object_gap, JoinType::jtRound, arc_tolerance));
                } else {
                    brim.push(ex.contour.clone());
                }
                if brim_inner {
                    let mut holes: Polygons = ex.holes.clone();
                    crate::geometry::polygon::polygons_reverse(&mut holes);
                    // shrink(holes, brim_object_gap, ClipperLib::jtRound, float(scale_(0.1))): offset(holes, -delta, ...)
                    holes = cu::offset(&holes, -brim_object_gap, JoinType::jtRound, arc_tolerance);
                    crate::geometry::polygon::polygons_reverse(&mut holes);
                    brim.extend(holes);
                } else {
                    brim.extend(ex.holes.iter().cloned());
                }
            }
        }
        brim = cu::union_(&brim);
    }

    // How much to inflate the support columns to be stable. This also applies to the 1st layer, if no raft layers are to be printed.
    let inflate_factor_fine: f32 = scale_(if slicing_params.raft_layers() > 1 { 0.5 } else { EPSILON }) as f32;
    let mut inflate_factor_1st_layer: f32 = scale_(cfg_float(object.config(), "raft_first_layer_expansion")) as f32;
    let front = |layers: &SupportGeneratorLayersPtr| -> Option<usize> { layers.first().copied() };
    let mut contacts = front(top_contacts);
    let mut interfaces = front(interface_layers);
    let mut base_interfaces = front(base_interface_layers);
    let mut columns_base = front(base_layers);
    if let Some(l) = contacts {
        // std::max(a, b): (a < b) ? b : a
        let m = if slicing_params.first_print_layer_height < slicing_params.raft_contact_top_z {
            slicing_params.raft_contact_top_z
        } else {
            slicing_params.first_print_layer_height
        };
        if layer_storage[l].print_z > m + EPSILON {
            // This is not the raft contact layer.
            contacts = None;
        }
    }
    if let Some(l) = interfaces {
        if layer_storage[l].bottom_print_z() > slicing_params.raft_interface_top_z + EPSILON {
            // This is not the raft column base layer.
            interfaces = None;
        }
    }
    if let Some(l) = base_interfaces {
        if layer_storage[l].bottom_print_z() > slicing_params.raft_interface_top_z + EPSILON {
            // This is not the raft column base layer.
            base_interfaces = None;
        }
    }
    if let Some(l) = columns_base {
        if layer_storage[l].bottom_print_z() > slicing_params.raft_interface_top_z + EPSILON {
            // This is not the raft interface layer.
            columns_base = None;
        }
    }

    let mut interface_polygons: Polygons = Vec::new();
    for l in [contacts, interfaces, base_interfaces].into_iter().flatten() {
        if !layer_storage[l].polygons.is_empty() {
            // expand(polygons, inflate_factor_fine, SUPPORT_SURFACES_OFFSET_PARAMETERS)
            interface_polygons.extend(cu::expand(&layer_storage[l].polygons, inflate_factor_fine, JoinType::jtSquare, 0.));
        }
    }

    // Output vector.
    let mut raft_layers: SupportGeneratorLayersPtr = Vec::new();

    if slicing_params.raft_layers() > 1 {
        let mut base: Polygons = Vec::new();
        let mut columns: Polygons = Vec::new();
        let mut first_layer: Polygons = Vec::new();
        if let Some(columns_base) = columns_base {
            if layer_storage[columns_base].bottom_print_z() > slicing_params.raft_interface_top_z - EPSILON {
                // Classic supports with colums above the raft interface.
                base = layer_storage[columns_base].polygons.clone();
                columns = base.clone();
                if !interface_polygons.is_empty() {
                    // Trim the 1st layer columns with the inflated interface polygons.
                    columns = cu::diff(&columns, &interface_polygons, No);
                }
            } else {
                // Organic supports with raft on print bed.
                debug_assert!(is_approx(layer_storage[columns_base].print_z, slicing_params.first_print_layer_height, EPSILON));
                first_layer = layer_storage[columns_base].polygons.clone();
            }
        }
        if !interface_polygons.is_empty() {
            // Merge the untrimmed columns base with the expanded raft interface, to be used for the support base and interface.
            base = cu::union_2(&base, &interface_polygons);
        }
        // Do not add the raft contact layer, only add the raft layers below the contact layer.
        // Insert the 1st layer.
        {
            let new_layer = layer_storage.len();
            layer_storage
                .allocate(if slicing_params.base_raft_layers > 0 { SupporLayerType::SltRaftBase } else { SupporLayerType::SltRaftInterface });
            raft_layers.push(new_layer);
            layer_storage[new_layer].print_z = slicing_params.first_print_layer_height;
            layer_storage[new_layer].height = slicing_params.first_print_layer_height;
            layer_storage[new_layer].bottom_z = 0.;
            first_layer = cu::union_2(&first_layer, &base);
            if inflate_factor_1st_layer < 0. {
                //means auto
                inflate_factor_1st_layer = scale_(2.) as f32;
                layer_storage[new_layer].polygons =
                    cu::expand(&first_layer, inflate_factor_1st_layer - inflate_factor_fine, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
            } else {
                layer_storage[new_layer].polygons = if inflate_factor_1st_layer > inflate_factor_fine {
                    cu::expand(&first_layer, inflate_factor_1st_layer - inflate_factor_fine, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
                } else {
                    first_layer
                };
            }
        }
        // Insert the base layers.
        for _ in 1..slicing_params.base_raft_layers {
            let print_z = layer_storage[*raft_layers.last().unwrap()].print_z;
            let new_layer = layer_storage.len();
            layer_storage.allocate_unguarded(SupporLayerType::SltRaftBase);
            raft_layers.push(new_layer);
            layer_storage[new_layer].print_z = print_z + slicing_params.base_raft_layer_height;
            layer_storage[new_layer].height = slicing_params.base_raft_layer_height;
            layer_storage[new_layer].bottom_z = print_z;
            layer_storage[new_layer].polygons = base.clone();
        }
        // Insert the interface layers.
        for _ in 1..slicing_params.interface_raft_layers {
            let print_z = layer_storage[*raft_layers.last().unwrap()].print_z;
            let new_layer = layer_storage.len();
            layer_storage.allocate_unguarded(SupporLayerType::SltRaftInterface);
            raft_layers.push(new_layer);
            layer_storage[new_layer].print_z = print_z + slicing_params.interface_raft_layer_height;
            layer_storage[new_layer].height = slicing_params.interface_raft_layer_height;
            layer_storage[new_layer].bottom_z = print_z;
            layer_storage[new_layer].polygons = interface_polygons.clone();
            //FIXME misusing contact_polygons for support columns.
            layer_storage[new_layer].contact_polygons = Some(columns.clone());
        }
    } else {
        if let Some(columns_base) = columns_base {
            // Expand the bases of the support columns in the 1st layer.
            let mut raft: Polygons = std::mem::take(&mut layer_storage[columns_base].polygons);
            // BBS: if first layer of support is intersected with object island, it must have the same function as brim unless in nobrim mode.
            // brim_object_gap is changed to 0 by default, it's no longer appropriate to use it to determine the gap of first layer support.
            //if (object.has_brim())
            //    trimming = offset(object.layers().front()->lslices, (float)scale_(object.config().brim_object_gap.value), SUPPORT_SURFACES_OFFSET_PARAMETERS);
            //else
            let trimming: Polygons =
                cu::offset_expolygons(&object.layers()[0].lslices, scale_(support_params.gap_xy_first_layer) as f32, JoinType::jtSquare, 0.);
            if inflate_factor_1st_layer < 0. {
                //means auto
                inflate_factor_1st_layer = scale_(2.) as f32;
            }
            if inflate_factor_1st_layer as f64 > inflate_factor_fine as f64 + scale_(EPSILON) {
                inflate_factor_1st_layer = inflate_factor_1st_layer - inflate_factor_fine;
                // Inflate in multiple steps to avoid leaking of the support 1st layer through object walls.
                // std::max(5, int(ceil(inflate_factor_1st_layer / support_params.first_layer_flow.scaled_width()))): a float quotient
                let nsteps = std::cmp::max(5, (inflate_factor_1st_layer / support_params.first_layer_flow.scaled_width() as f32).ceil() as i32);
                let step: f32 = inflate_factor_1st_layer / nsteps as f32;
                for _ in 0..nsteps {
                    raft = cu::diff(&cu::expand(&raft, step, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT), &trimming, No);
                }
            } else {
                raft = cu::diff(&raft, &trimming, No);
            }
            layer_storage[columns_base].polygons = raft;
            if !interface_polygons.is_empty() {
                let trimmed = cu::diff(&layer_storage[columns_base].polygons, &interface_polygons, No);
                layer_storage[columns_base].polygons = trimmed;
            }
        }
        if !brim.is_empty() {
            for l in [columns_base, contacts, interfaces, base_interfaces].into_iter().flatten() {
                let trimmed = cu::diff(&layer_storage[l].polygons, &brim, No);
                layer_storage[l].polygons = trimmed;
            }
        }
    }

    raft_layers
}

// SupportCommon.cpp:1490-1493
// Support layer that is covered by some form of dense interface.
pub const SUPPORT_TYPES_INTERFACE: [SupporLayerType; 5] = [
    SupporLayerType::SltRaftInterface,
    SupporLayerType::SltBottomContact,
    SupporLayerType::SltBottomInterface,
    SupporLayerType::SltTopContact,
    SupporLayerType::SltTopInterface,
];

/// SupportLayer.hpp:56-73 `bool SupportGeneratorLayer::operator<(const SupportGeneratorLayer &layer2) const`
// Order the layers by lexicographically by an increasing print_z and a decreasing layer height.
#[inline]
fn support_generator_layer_less(l1: &SupportGeneratorLayer, l2: &SupportGeneratorLayer) -> bool {
    if l1.print_z < l2.print_z {
        true
    } else if l1.print_z == l2.print_z {
        if l1.height > l2.height {
            true
        } else if l1.height == l2.height {
            // Bridging layers first.
            l1.bridging && !l2.bridging
        } else {
            false
        }
    } else {
        false
    }
}

// SupportCommon.cpp:2189-2268
//
// The layers are sorted by their address first (to drop the duplicates), then by `operator<` with
// `std::sort`: layers that compare equal come out in an order that depends on their addresses. Here the
// address is the index in the storage.
#[allow(clippy::too_many_arguments)]
pub fn generate_support_layers(
    object: &mut pf::PrintObject,
    raft_layers: &SupportGeneratorLayersPtr,
    bottom_contacts: &SupportGeneratorLayersPtr,
    top_contacts: &SupportGeneratorLayersPtr,
    intermediate_layers: &SupportGeneratorLayersPtr,
    interface_layers: &SupportGeneratorLayersPtr,
    base_interface_layers: &SupportGeneratorLayersPtr,
    layer_storage: &mut SupportGeneratorLayerStorage,
) -> SupportGeneratorLayersPtr {
    // Install support layers into the object.
    // A support layer installed on a PrintObject has a unique print_z.
    let mut layers_sorted: SupportGeneratorLayersPtr = Vec::with_capacity(
        raft_layers.len() + bottom_contacts.len() + top_contacts.len() + intermediate_layers.len() + interface_layers.len() + base_interface_layers.len(),
    );
    layers_sorted.extend_from_slice(raft_layers);
    layers_sorted.extend_from_slice(bottom_contacts);
    layers_sorted.extend_from_slice(top_contacts);
    layers_sorted.extend_from_slice(intermediate_layers);
    layers_sorted.extend_from_slice(interface_layers);
    layers_sorted.extend_from_slice(base_interface_layers);
    // remove dupliated layers
    layers_sorted.sort_unstable();
    layers_sorted.dedup();

    // Sort the layers lexicographically by a raising print_z and a decreasing height.
    crate::clipper::libcxx_sort(&mut layers_sorted, |&l1, &l2| support_generator_layer_less(&layer_storage[l1], &layer_storage[l2]));
    let mut layer_id: i32 = 0;
    let mut layer_id_interface: i32 = 0;
    debug_assert!(object.support_layers().is_empty());
    let mut i = 0;
    while i < layers_sorted.len() {
        // Find the last layer with roughly the same print_z, find the minimum layer height of all.
        // Due to the floating point inaccuracies, the print_z may not be the same even if in theory they should.
        let mut j = i + 1;
        let zmax = layer_storage[layers_sorted[i]].print_z + EPSILON;
        while j < layers_sorted.len() && layer_storage[layers_sorted[j]].print_z <= zmax {
            j += 1;
        }
        // Assign an average print_z to the set of layers with nearly equal print_z.
        let zavg = 0.5 * (layer_storage[layers_sorted[i]].print_z + layer_storage[layers_sorted[j - 1]].print_z);
        let mut height_min = layer_storage[layers_sorted[i]].height;
        let mut empty = true;
        // For snug supports, layers where the direction of the support interface shall change are accounted for.
        let mut num_interfaces: usize = 0;
        let mut num_top_contacts: usize = 0;
        let mut top_contact_bottom_z: f64 = 0.;
        for u in i..j {
            let layer = &mut layer_storage[layers_sorted[u]];
            if !layer.polygons.is_empty() {
                empty = false;
                num_interfaces += SUPPORT_TYPES_INTERFACE.contains(&layer.layer_type) as usize;
                if layer.layer_type == SupporLayerType::SltTopContact {
                    num_top_contacts += 1;
                    debug_assert!(num_top_contacts <= 1);
                    // All top contact layers sharing this print_z shall also share bottom_z.
                    //assert(num_top_contacts == 1 || (top_contact_bottom_z - layer.bottom_z) < EPSILON);
                    top_contact_bottom_z = layer.bottom_z;
                }
            }
            layer.print_z = zavg;
            // std::min(height_min, layer.height)
            if layer.height < height_min {
                height_min = layer.height;
            }
        }
        if !empty {
            // Here the upper_layer and lower_layer pointers are left to null at the support layers,
            // as they are never used. These pointers are candidates for removal.
            let this_layer_contacts_only = num_top_contacts > 0 && num_top_contacts == num_interfaces;
            let mut this_layer_id_interface = layer_id_interface as usize;
            if this_layer_contacts_only {
                // Find a supporting layer for its interface ID.
                for other_layer in object.support_layers().iter().rev() {
                    if (other_layer.layer.print_z - top_contact_bottom_z).abs() < EPSILON {
                        // other_layer supports this top contact layer. Assign a different support interface direction to this layer
                        // from the layer that supports it.
                        this_layer_id_interface = other_layer.interface_id() + 1;
                    }
                }
            }
            object.add_support_layer(layer_id, this_layer_id_interface as i32, height_min, zavg);
            layer_id += 1;
            if num_interfaces != 0 && !this_layer_contacts_only {
                layer_id_interface += 1;
            }
        }
        i = j;
    }
    layers_sorted
}

// SupportCommon.cpp:1518-1526
// BBS multi-diameter: the XY the merged bands occupy on object layer layer_idx. A band is extruded with its
// retained footprint from the band bottom up, so on its lower layers it occupies XY their lslices do not
// hold. Only patches spanning the layer count: marker copies reach down to the band's planned floor, below a
// locally clipped patch's own bottom_idx. Not unioned.
// C++: `Polygons merged_band_footprint(const Layer &layer, size_t layer_idx)`
pub fn merged_band_footprint(layer: &crate::layer::Layer, layer_idx: usize) -> Polygons {
    let mut band: Polygons = Vec::new();
    for layerm in layer.regions() {
        for patch in &layerm.adaptive_merge_patches {
            if patch.bottom_idx <= layer_idx && layer_idx <= patch.top_idx {
                band.extend(crate::geometry::to_polygons(patch.area()));
            }
        }
    }
    band
}

// SupportCommon.cpp:1146-1192
// 只保留靠近 keep_bbox 的线段（两侧各多留一段）。keep_bbox 需比实际关心的区域外扩 1.5 倍偏移距离，
// 这样截断后再偏移，结果在关心区域内与原来一致。
pub fn keep_polylines_near_bbox(polylines: &mut Polylines, keep_bbox: &crate::geometry::BoundingBox) {
    let segment_near = |a: &Point, b: &Point| -> bool {
        std::cmp::max(a.x, b.x) >= keep_bbox.min.x
            && std::cmp::min(a.x, b.x) <= keep_bbox.max.x
            && std::cmp::max(a.y, b.y) >= keep_bbox.min.y
            && std::cmp::min(a.y, b.y) <= keep_bbox.max.y
    };
    let mut out: Polylines = Vec::with_capacity(polylines.len());
    let mut keep: Vec<u8> = Vec::new();
    for polyline in polylines.drain(..) {
        if polyline.points.len() < 2 {
            out.push(polyline);
            continue;
        }
        let points = &polyline.points;
        let num_segments = points.len() - 1;
        keep.clear();
        keep.resize(num_segments, 0);
        let mut all_near = true;
        for i in 0..num_segments {
            if segment_near(&points[i], &points[i + 1]) {
                keep[i] = 1;
                if i > 0 {
                    keep[i - 1] = 1;
                }
                if i + 1 < num_segments {
                    keep[i + 1] |= 2;
                }
            } else {
                all_near = false;
            }
        }
        if all_near {
            out.push(polyline);
            continue;
        }
        let mut i = 0;
        while i < num_segments {
            if keep[i] == 0 {
                i += 1;
                continue;
            }
            let mut j = i;
            while j < num_segments && keep[j] != 0 {
                j += 1;
            }
            out.push(crate::geometry::Polyline::from_points(points[i..j + 1].to_vec()));
            i = j;
        }
    }
    *polylines = out;
}

// SupportCommon.cpp:528-544
pub fn draw_perimeters(expoly: &ExPolygon, clip_length: f64) -> Polylines {
    // Draw the perimeters.
    let mut polylines: Polylines = Vec::with_capacity(expoly.holes.len() + 1);
    for i in 0..=expoly.holes.len() {
        let mut pl = crate::geometry::Polyline::from_points(if i == 0 { expoly.contour.points.clone() } else { expoly.holes[i - 1].points.clone() });
        let front = pl.points[0];
        pl.points.push(front);
        if i > 0 {
            // It is a hole, reverse it.
            pl.points.reverse();
        }
        // so that all contours are CCW oriented.
        pl.clip_end(clip_length);
        polylines.push(pl);
    }
    polylines
}

// SupportCommon.cpp:1123-1135 (SLIC3R_DEBUG)
pub fn dbg_index_to_color(idx: i32) -> String {
    if idx < 0 {
        return "yellow".to_string();
    }
    let idx = idx % 3;
    match idx {
        0 => "red".to_string(),
        1 => "green".to_string(),
        _ => "blue".to_string(),
    }
}

/// Polygon.cpp:245-256 and 699-706 on ClipperLib (`intersection_pl(const Polylines &, const Polygon &)`,
/// ClipperUtils.cpp:923): the overlap tests SupportCommon.cpp and SupportMaterial.cpp call.
pub mod cu_polygon {
    use super::cu;
    use crate::clipper::ClipType;
    use crate::clipper_utils as u;
    use crate::geometry::{contains_polygon, Polygon, Polygons, Polyline, Polylines};

    /// Polygon.hpp:233-240 `Polylines to_polylines(const Polygons &polygons)`: every polygon closed.
    pub fn to_polylines(polygons: &[Polygon]) -> Polylines {
        polygons
            .iter()
            .map(|p| {
                let mut pl = Polyline::from_points(p.points.clone());
                if let Some(&front) = p.points.first() {
                    pl.points.push(front);
                }
                pl
            })
            .collect()
    }
    /// Polygon.cpp:245-256 `bool Polygon::overlaps(const Polygons &other) const`
    pub fn polygon_overlaps(this: &Polygon, other: &Polygons) -> bool {
        if this.points.is_empty() || other.is_empty() {
            return false;
        }
        let pl_out = u::_clipper_pl_open(
            ClipType::ctIntersection,
            &cu::polylines_provider(&to_polylines(other)),
            &cu::single_path_provider(&this.points),
        );
        // See unit test SCENARIO("Clipper diff with polyline", "[Clipper]")
        // for in which case the intersection_pl produces any intersection.
        !pl_out.is_empty() ||
            // If *this is completely inside other, then pl_out is empty, but the expolygons overlap. Test for that situation.
            other.iter().any(|poly| contains_polygon(poly, &this.points[0], true))
    }
    /// Polygon.cpp:699-706 `bool overlaps(const Polygons &polys1, const Polygons &polys2)`
    pub fn overlaps(polys1: &Polygons, polys2: &Polygons) -> bool {
        for poly1 in polys1 {
            if polygon_overlaps(poly1, polys2) {
                return true;
            }
        }
        false
    }
}

// ============================================================================
// The fillers and the extrusions of SupportCommon.cpp.
// ============================================================================

use crate::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionMultiPath, ExtrusionPath, ExtrusionRole};
use crate::fill::FillParams;
use crate::flow::{Flow, FlowResult};
use crate::print_config::InfillPattern;
use crate::surface::{Surface, SurfaceType};

/// ExtrusionEntity.hpp `typedef std::vector<ExtrusionEntity*> ExtrusionEntitiesPtr`
pub type ExtrusionEntitiesPtr = Vec<ExtrusionEntityType>;

/// FillBase.hpp:33-37 `class InfillFailedException`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InfillFailedException;

/// `Fill *` of SupportCommon.cpp: the data members of `class Fill` (FillBase.hpp:114-141) and its virtual
/// `Polylines fill_surface(const Surface *surface, const FillParams &params)` (FillBase.hpp:155).
///
/// FIDELITY-NOTE: the fillers of `crate::fill` are not behind one base class with a `fill_surface()` each
/// and there is no `Fill::new_from_type()`; the support code takes its fillers through this trait and
/// `SupportFillFactory`, and everything around the call is the C++.
pub trait SupportFill {
    /// The members of the `Fill` base: `angle`, `spacing`, `link_max_length`, `layer_id`, `z`, `overlap`,
    /// `bounding_box` (`set_bounding_box(bbox)` is `fill_mut().bounding_box = bbox`, FillBase.hpp:147).
    fn fill(&self) -> &crate::fill::fill_base::Fill;
    fn fill_mut(&mut self) -> &mut crate::fill::fill_base::Fill;
    /// FillBase.hpp:155 `virtual Polylines fill_surface(const Surface *surface, const FillParams &params)`
    fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Result<Polylines, InfillFailedException>;
}

/// FillBase.cpp:33-67 `Fill* Fill::new_from_type(const InfillPattern type)`
pub trait SupportFillFactory {
    fn new_from_type(&self, pattern: InfillPattern) -> Box<dyn SupportFill>;
}

/// ExtrusionEntity.hpp:658-668 `inline void extrusion_entities_append_paths(ExtrusionEntitiesPtr &dst, Polylines
/// &&polylines, ExtrusionRole role, double mm3_per_mm, float width, float height, bool can_reverse = true)`:
/// an `ExtrusionPathOriented` (ExtrusionEntity.hpp:385-395, `can_reverse()` false) when `can_reverse` is not set.
pub fn extrusion_entities_append_paths(dst: &mut ExtrusionEntitiesPtr, polylines: Polylines, role: ExtrusionRole, mm3_per_mm: f64, width: f32, height: f32, can_reverse: bool) {
    dst.reserve(polylines.len());
    for polyline in polylines {
        if polyline.is_valid() {
            let mut extrusion_path = ExtrusionPath::with_params(role, mm3_per_mm, width as f64, height as f64, false);
            if !can_reverse {
                extrusion_path.can_reverse = false;
            }
            extrusion_path.polyline = polyline;
            dst.push(ExtrusionEntityType::Path(extrusion_path));
        }
    }
}

// SupportCommon.cpp:478-499
#[allow(clippy::too_many_arguments)]
pub fn fill_expolygon_generate_paths(
    dst: &mut ExtrusionEntitiesPtr,
    expolygon: ExPolygon,
    filler: &mut dyn SupportFill,
    fill_params: &FillParams,
    _density: f32,
    role: ExtrusionRole,
    flow: &Flow,
) -> FlowResult<()> {
    let surface = Surface::new(SurfaceType::Internal, expolygon);
    debug_assert!(!fill_params.use_arachne);
    // catch (InfillFailedException &) {}
    let polylines: Polylines = filler.fill_surface(&surface, fill_params).unwrap_or_default();
    extrusion_entities_append_paths(dst, polylines, role, flow.mm3_per_mm()?, flow.width() as f32, flow.height() as f32, true);
    Ok(())
}

// SupportCommon.cpp:501-512
#[allow(clippy::too_many_arguments)]
pub fn fill_expolygons_generate_paths_params(
    dst: &mut ExtrusionEntitiesPtr,
    expolygons: ExPolygons,
    filler: &mut dyn SupportFill,
    fill_params: &FillParams,
    density: f32,
    role: ExtrusionRole,
    flow: &Flow,
) -> FlowResult<()> {
    for expoly in expolygons {
        fill_expolygon_generate_paths(dst, expoly, filler, fill_params, density, role, flow)?;
    }
    Ok(())
}

/// `FillParams fill_params; fill_params.density = density; fill_params.dont_adjust = true;`
///
/// FIDELITY-NOTE: `FillParams::new()` of the crate sets `density` 1, `extrusion_role` erInternalInfill and a
/// flow, where the C++ members start at 0, `ExtrusionRole(0)` and a default flow (FillBase.hpp:51-110).
fn support_fill_params(density: f32) -> FillParams {
    let mut fill_params = FillParams::new();
    fill_params.density = density as f64;
    fill_params.dont_adjust = true;
    fill_params.extrusion_role = ExtrusionRole::None;
    fill_params.flow = Flow::default_constructed();
    fill_params
}

// SupportCommon.cpp:514-526
pub fn fill_expolygons_generate_paths(
    dst: &mut ExtrusionEntitiesPtr,
    expolygons: ExPolygons,
    filler: &mut dyn SupportFill,
    density: f32,
    role: ExtrusionRole,
    flow: &Flow,
) -> FlowResult<()> {
    let fill_params = support_fill_params(density);
    fill_expolygons_generate_paths_params(dst, expolygons, filler, &fill_params, density, role, flow)
}

// SupportCommon.cpp:555-638, the lambda `shrink_expolygon_with_contour_idx` of tree_supports_generate_paths().
// Offset expolygon inside, returns number of expolygons collected (0 or 1).
// Vertices of output paths are marked with Z = source contour index of the expoly.
// Vertices at the intersection of source contours are marked with Z = -1.
fn shrink_expolygon_with_contour_idx(expoly: &ExPolygon, delta: f32, join_type: JoinType, miter_limit: f64, out: &mut crate::clipper_lib_z::Paths) -> i32 {
    use crate::clipper::{ClipperOffset, EndType, Paths};
    use crate::clipper_lib_z as cz;
    debug_assert!(delta > 0.);
    let append_paths_with_z = |src: &Paths, contour_idx: i32, dst: &mut cz::Paths| {
        for contour in src {
            let mut tmp: cz::Path = Vec::with_capacity(contour.len());
            for p in contour {
                tmp.push(cz::IntPoint::new_xyz(p.x, p.y, contour_idx));
            }
            dst.push(tmp);
        }
    };
    let offset_contour = |points: &[Point], delta_execute: f64| -> Paths {
        let mut co = ClipperOffset::default();
        if join_type == JoinType::jtRound {
            co.arc_tolerance = miter_limit;
        } else {
            co.miter_limit = miter_limit;
        }
        co.shortest_edge_length = delta as f64 * 0.005;
        co.add_path(&cu::single_path_provider(points)[0], join_type, EndType::etClosedPolygon);
        let mut raw = Paths::new();
        co.execute(&mut raw, delta_execute);
        raw
    };

    // 1) Offset the outer contour.
    let mut contours = cz::Paths::new();
    {
        let contours_raw = offset_contour(&expoly.contour.points, -(delta as f64));
        if contours_raw.is_empty() {
            // No need to try to offset the holes.
            return 0;
        }
        append_paths_with_z(&contours_raw, 0, &mut contours);
    }

    if expoly.holes.is_empty() {
        // No need to subtract holes from the offsetted expolygon, we are done.
        out.extend(contours);
    } else {
        // 2) Offset the holes one by one, collect the offsetted holes.
        let mut holes = cz::Paths::new();
        for (idx_hole, hole) in expoly.holes.iter().enumerate() {
            // Execute reorients the contours so that the outer most contour has a positive area. Thus the output
            // contours will be CCW oriented even though the input paths are CW oriented.
            // Offset is applied after contour reorientation, thus the signum of the offset value is reversed.
            let out2 = offset_contour(&hole.points, delta as f64);
            append_paths_with_z(&out2, 1 + idx_hole as i32, &mut holes);
        }

        // 3) Subtract holes from the contours.
        if holes.is_empty() {
            // No hole remaining after an offset. Just copy the outer contour.
            out.extend(contours);
        } else {
            // Negative offset. There is a chance, that the offsetted hole intersects the outer contour.
            // Subtract the offsetted holes from the offsetted contours.
            let mut clipper = cz::Clipper::new(0);
            clipper.z_fill_function(cz::ZFillCallback::new(|_e1bot, _e1top, _e2bot, _e2top, pt| {
                // Just mark the intersection.
                pt.z = -1;
            }));
            clipper.add_paths(&contours, cz::PolyType::ptSubject, true);
            clipper.add_paths(&holes, cz::PolyType::ptClip, true);
            let mut output = cz::Paths::new();
            clipper.execute(cz::ClipType::ctDifference, &mut output, cz::PolyFillType::pftNonZero, cz::PolyFillType::pftNonZero);
            if !output.is_empty() {
                out.extend(output);
            } else {
                // The offsetted holes have eaten up the offsetted outer contour.
                return 0;
            }
        }
    }

    1
}

// SupportCommon.cpp:546-765
pub fn tree_supports_generate_paths(dst: &mut ExtrusionEntitiesPtr, polygons: &Polygons, flow: &Flow, support_params: &SupportParameters) -> FlowResult<()> {
    use crate::clipper_lib_z as cz;
    let spacing: f64 = flow.scaled_spacing() as f64;
    // Clip the sheath path to avoid the extruder to get exactly on the first point of the loop.
    let clip_length: f64 = spacing * 0.15;
    let anchor_length: f64 = spacing * 6.;
    let scaled_width = flow.scaled_width();
    let mut anchor_candidates = cz::Paths::new();
    // closing_ex(polygons, float(SCALED_EPSILON), float(SCALED_EPSILON + 0.5 * flow.scaled_width()))
    let closed = cu::closing_ex(
        polygons,
        scale_(EPSILON) as f32,
        (scale_(EPSILON) + 0.5 * scaled_width as f64) as f32,
        cu::DEFAULT_JOIN_TYPE,
        cu::DEFAULT_MITER_LIMIT,
    );
    for expoly in closed {
        let mut eec: Option<ExtrusionEntityCollection> = None;
        let mut regions_to_draw_inner_wall: ExPolygons = vec![expoly.clone()];
        let area: f64 = expoly.area();
        if support_params.tree_branch_diameter_double_wall_area_scaled > 0. && area > support_params.tree_branch_diameter_double_wall_area_scaled {
            let mut collection = ExtrusionEntityCollection::new();
            // Don't reorder internal / external loops of the same island.
            collection.no_sort = true;
            // Make the tree branch stable by adding another perimeter.
            // offset2_ex({expoly}, -1.5 * flow.scaled_width(), 0.5 * flow.scaled_width())
            let level2: ExPolygons = cu::offset2_ex(
                std::slice::from_ref(&expoly),
                (-1.5 * scaled_width as f64) as f32,
                (0.5 * scaled_width as f64) as f32,
                cu::DEFAULT_JOIN_TYPE,
                cu::DEFAULT_MITER_LIMIT,
            );
            if !level2.is_empty() {
                regions_to_draw_inner_wall = level2.clone();
                extrusion_entities_append_paths(
                    &mut collection.entities,
                    draw_perimeters(&expoly, clip_length),
                    ExtrusionRole::SupportMaterial,
                    flow.mm3_per_mm()?,
                    flow.width() as f32,
                    flow.height() as f32,
                    // Disable reversal of the path, always start with the anchor, always print CCW.
                    false,
                );
                // `expoly = level2.front();`: not read again, the loop below declares its own `expoly`.
            }
            eec = Some(collection);
        }
        let mut entities: ExtrusionEntitiesPtr = Vec::new();
        for expoly in &regions_to_draw_inner_wall {
            // Try to produce one more perimeter to place the seam anchor.
            // First genrate a 2nd perimeter loop as a source for anchor candidates.
            // The anchor candidate points are annotated with an index of the source contour or with -1 if on intersection.
            anchor_candidates.clear();
            shrink_expolygon_with_contour_idx(expoly, scaled_width as f32, cu::DEFAULT_JOIN_TYPE, 1.2, &mut anchor_candidates);
            // Orient all contours CW.
            for path in anchor_candidates.iter_mut() {
                if cz::area(path) > 0. {
                    path.reverse();
                }
            }

            // Draw the perimeters.
            let mut polylines: Polylines = Vec::with_capacity(expoly.holes.len() + 1);
            for idx_loop in 0..expoly.num_contours() as i32 {
                // Open the loop with a seam.
                let r#loop: &Polygon = if idx_loop == 0 { &expoly.contour } else { &expoly.holes[idx_loop as usize - 1] };
                let mut pl = crate::geometry::Polyline::from_points(r#loop.points.clone());
                // Orient all contours CW, because the anchor will be added to the end of polyline while we want to start a loop with the anchor.
                if idx_loop == 0 {
                    // It is an outer contour.
                    pl.points.reverse();
                }
                let front = pl.points[0];
                pl.points.push(front);
                pl.clip_end(clip_length);
                if pl.points.len() < 2 {
                    continue;
                }
                // Find the foot of the seam point on anchor_candidates. Only pick an anchor point that was created by offsetting the source contour.
                let mut closest_contour: Option<usize> = None;
                let mut closest_point: (f64, f64) = (0., 0.);
                let mut closest_point_idx: i32 = -1;
                let mut d2min: f64 = f64::MAX;
                let back = *pl.points.last().unwrap();
                let seam_pt: (f64, f64) = (back.x as f64, back.y as f64);
                for (idx_path, path) in anchor_candidates.iter().enumerate() {
                    for i in 0..path.len() as i32 {
                        // next_idx_modulo(i, path)
                        let j = if i + 1 == path.len() as i32 { 0 } else { i + 1 };
                        let (qi, qj) = (&path[i as usize], &path[j as usize]);
                        if qi.z == idx_loop || qj.z == idx_loop {
                            let pi = (qi.x as f64, qi.y as f64);
                            let pj = (qj.x as f64, qj.y as f64);
                            let v = (pj.0 - pi.0, pj.1 - pi.1);
                            let w = (seam_pt.0 - pi.0, seam_pt.1 - pi.1);
                            let l2 = v.0 * v.0 + v.1 * v.1;
                            // std::clamp((l2 == 0) ? 0 : v.dot(w) / l2, 0., 1.)
                            let t0 = if l2 == 0. { 0. } else { (v.0 * w.0 + v.1 * w.1) / l2 };
                            let t = if t0 < 0. {
                                0.
                            } else if 1. < t0 {
                                1.
                            } else {
                                t0
                            };
                            if (qi.z == idx_loop || t > EPSILON) && (qj.z == idx_loop || t < 1. - EPSILON) {
                                // Closest point.
                                let fp = (pi.0 + v.0 * t, pi.1 + v.1 * t);
                                let d = (fp.0 - seam_pt.0, fp.1 - seam_pt.1);
                                let d2 = d.0 * d.0 + d.1 * d.1;
                                if d2 < d2min {
                                    d2min = d2;
                                    closest_contour = Some(idx_path);
                                    closest_point = fp;
                                    closest_point_idx = i;
                                }
                            }
                        }
                    }
                }
                // sqr(flow.scaled_width() * 3.)
                let w3 = scaled_width as f64 * 3.;
                if d2min < w3 * w3
                    && regions_to_draw_inner_wall.iter().map(|e| e.area()).fold(0., |s, a| s + a) + scale_(EPSILON) > area
                    // shrink_ex({expoly}, -2. * flow.scaled_width()): offset_ex(expolygons, -delta)
                    && !cu::offset_ex_expolygons(std::slice::from_ref(expoly), -((-2. * scaled_width as f64) as f32), cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
                        .is_empty()
                {
                    // Try to cut an anchor from the closest_contour.
                    // Both closest_contour and pl are CW oriented.
                    // Walk the CW contour backwards, so once pl is reversed the anchor lies ahead of the seam in
                    // the CCW print direction. A double wall above starts its inner loop at the same seam and runs
                    // CCW too, so it is laid on top of the anchor instead of moving away from it.
                    // closest_point.cast<coord_t>()
                    pl.points.push(Point::new(closest_point.0 as i32 as Coord, closest_point.1 as i32 as Coord));
                    let path = &anchor_candidates[closest_contour.unwrap()];
                    let num_points = path.len() as i32;
                    let seam_dist = {
                        let d = (seam_pt.0 - closest_point.0, seam_pt.1 - closest_point.1);
                        (d.0 * d.0 + d.1 * d.1).sqrt()
                    };
                    // std::min(anchor_length - (seam_pt - closest_point).norm(), pl.length() / 12.)
                    let (a, b) = (anchor_length - seam_dist, pl.length() / 12.);
                    let mut remaining_length: f64 = if b < a { b } else { a };
                    // Walk at most once around the contour, stopping where it leaves the source loop.
                    let mut p = closest_point;
                    let mut j = closest_point_idx;
                    let mut step = 0;
                    while remaining_length > 0. && step < num_points {
                        let pj = (path[j as usize].x as f64, path[j as usize].y as f64);
                        let dv = (pj.0 - p.0, pj.1 - p.1);
                        let l = (dv.0 * dv.0 + dv.1 * dv.1).sqrt();
                        if remaining_length <= l {
                            let s = remaining_length / l;
                            pl.points.push(Point::new((p.0 + dv.0 * s) as i32 as Coord, (p.1 + dv.1 * s) as i32 as Coord));
                            break;
                        }
                        if l > EPSILON {
                            pl.points.push(Point::new(path[j as usize].x as Coord, path[j as usize].y as Coord));
                        }
                        remaining_length -= l;
                        if path[j as usize].z != idx_loop {
                            break;
                        }
                        p = pj;
                        step += 1;
                        // prev_idx_modulo(j, num_points)
                        j = if j == 0 { num_points - 1 } else { j - 1 };
                    }
                }
                // Start with the anchor.
                pl.points.reverse();
                polylines.push(pl);
            }

            let out: &mut ExtrusionEntitiesPtr = match eec.as_mut() {
                Some(eec) => &mut eec.entities,
                None => &mut entities,
            };
            extrusion_entities_append_paths(
                out,
                polylines,
                ExtrusionRole::SupportMaterial,
                flow.mm3_per_mm()?,
                flow.width() as f32,
                flow.height() as f32,
                // Disable reversal of the path, always start with the anchor, always print CCW.
                false,
            );
        }
        dst.append(&mut entities);
        if let Some(mut eec) = eec {
            // Loops were appended external first. Print the internal loop first, except in multi nozzle
            // diameter mode, which keeps the external loop first.
            if !support_params.multi_nozzle_diameter_mode {
                eec.entities.reverse();
            }
            dst.push(ExtrusionEntityType::Collection(Box::new(eec)));
        }
    }
    Ok(())
}

// SupportCommon.cpp:767-814
#[allow(clippy::too_many_arguments)]
pub fn fill_expolygons_with_sheath_generate_paths(
    dst: &mut ExtrusionEntitiesPtr,
    polygons: &Polygons,
    filler: &mut dyn SupportFill,
    density: f32,
    role: ExtrusionRole,
    flow: &Flow,
    support_params: &SupportParameters,
    with_sheath: bool,
    no_sort: bool,
) -> FlowResult<()> {
    if polygons.is_empty() {
        return Ok(());
    }

    if with_sheath {
        if density == 0. {
            return tree_supports_generate_paths(dst, polygons, flow, support_params);
        }
    } else {
        // closing_ex(polygons, float(SCALED_EPSILON))
        let closed = cu::closing_ex(polygons, scale_(EPSILON) as f32, scale_(EPSILON) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
        return fill_expolygons_generate_paths(dst, closed, filler, density, role, flow);
    }

    let fill_params = support_fill_params(density);

    let spacing: f64 = flow.scaled_spacing() as f64;
    // Clip the sheath path to avoid the extruder to get exactly on the first point of the loop.
    let clip_length: f64 = spacing * 0.15;

    let closed = cu::closing_ex(
        polygons,
        scale_(EPSILON) as f32,
        (scale_(EPSILON) + 0.5 * flow.scaled_width() as f64) as f32,
        cu::DEFAULT_JOIN_TYPE,
        cu::DEFAULT_MITER_LIMIT,
    );
    for expoly in closed {
        // Don't reorder the skirt and its infills.
        let mut eec = ExtrusionEntityCollection::new();
        if no_sort {
            eec.no_sort = true;
        }
        {
            let out: &mut ExtrusionEntitiesPtr = if no_sort { &mut eec.entities } else { dst };
            extrusion_entities_append_paths(
                out,
                draw_perimeters(&expoly, clip_length),
                ExtrusionRole::SupportMaterial,
                flow.mm3_per_mm()?,
                flow.width() as f32,
                flow.height() as f32,
                true,
            );
            // Fill in the rest.
            fill_expolygons_generate_paths_params(
                out,
                cu_expolygon::offset_ex(&expoly, (-0.4 * spacing) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                filler,
                &fill_params,
                density,
                role,
                flow,
            )?;
        }
        if no_sort && !eec.entities.is_empty() {
            dst.push(ExtrusionEntityType::Collection(Box::new(eec)));
        }
    }
    Ok(())
}

// SupportCommon.cpp:816-889
// Support layers, partially processed.
#[derive(Debug, Clone, Default)]
pub struct SupportGeneratorLayerExtruded {
    // The source layer. It carries the height and extrusion type (bridging / non bridging, extrusion height).
    pub layer: Option<usize>,
    // Collect extrusions. They will be exported sorted by the bottom height.
    pub extrusions: ExtrusionEntitiesPtr,
    // In case the extrusions are non-empty, m_polygons_to_extrude may contain the rest areas yet to be filled by additional support.
    // This is useful mainly for the loop interfaces, which are generated before the zig-zag infills.
    m_polygons_to_extrude: Option<Polygons>,
}

impl SupportGeneratorLayerExtruded {
    // SupportCommon.cpp:819-825 `SupportGeneratorLayerExtruded& operator=(SupportGeneratorLayerExtruded &&rhs)`
    pub fn assign(&mut self, rhs: &mut SupportGeneratorLayerExtruded) {
        self.layer = rhs.layer;
        self.extrusions = std::mem::take(&mut rhs.extrusions);
        self.m_polygons_to_extrude = rhs.m_polygons_to_extrude.take();
        rhs.layer = None;
    }

    // SupportCommon.cpp:827-829
    pub fn empty(&self, layer_storage: &SupportGeneratorLayerStorage) -> bool {
        match self.layer {
            None => true,
            Some(layer) => layer_storage[layer].polygons.is_empty(),
        }
    }

    // SupportCommon.cpp:831-836
    pub fn set_polygons_to_extrude(&mut self, polygons: Polygons) {
        self.m_polygons_to_extrude = Some(polygons);
    }

    // SupportCommon.cpp:837-838
    pub fn polygons_to_extrude<'a>(&'a self, layer_storage: &'a SupportGeneratorLayerStorage) -> &'a Polygons {
        match &self.m_polygons_to_extrude {
            None => &layer_storage[self.layer.expect("polygons_to_extrude() of a layer")].polygons,
            Some(polygons) => polygons,
        }
    }

    // SupportCommon.cpp:840-844
    pub fn could_merge(&self, other: &SupportGeneratorLayerExtruded, layer_storage: &SupportGeneratorLayerStorage) -> bool {
        !self.empty(layer_storage) && !other.empty(layer_storage) && {
            let (this_layer, other_layer) = (&layer_storage[self.layer.unwrap()], &layer_storage[other.layer.unwrap()]);
            (this_layer.height - other_layer.height).abs() < EPSILON && this_layer.bridging == other_layer.bridging
        }
    }

    // SupportCommon.cpp:846-873
    // Merge regions, perform boolean union over the merged polygons.
    pub fn merge(&mut self, other: &mut SupportGeneratorLayerExtruded, layer_storage: &mut SupportGeneratorLayerStorage) {
        debug_assert!(self.could_merge(other, layer_storage));
        let (this_layer, other_layer) = (self.layer.unwrap(), other.layer.unwrap());
        // 1) Merge the rest polygons to extrude, if there are any.
        if let Some(other_polygons) = other.m_polygons_to_extrude.take() {
            if self.m_polygons_to_extrude.is_none() {
                // This layer has no extrusions generated yet, if it has no m_polygons_to_extrude (its area to extrude was not reduced yet).
                debug_assert!(self.extrusions.is_empty());
                self.m_polygons_to_extrude = Some(layer_storage[this_layer].polygons.clone());
            }
            let polygons = self.m_polygons_to_extrude.as_mut().unwrap();
            polygons.extend(other_polygons);
            *polygons = cu::union_safety_offset(polygons);
        } else if let Some(polygons) = self.m_polygons_to_extrude.as_mut() {
            // The other layer has no extrusions generated yet, if it has no m_polygons_to_extrude (its area to extrude was not reduced yet).
            debug_assert!(other.extrusions.is_empty());
            polygons.extend(layer_storage[other_layer].polygons.iter().cloned());
            *polygons = cu::union_safety_offset(polygons);
        }
        // 2) Merge the extrusions.
        self.extrusions.append(&mut other.extrusions);
        // 3) Merge the infill polygons.
        let other_polygons = std::mem::take(&mut layer_storage[other_layer].polygons);
        layer_storage[this_layer].polygons.extend(other_polygons);
        let merged = cu::union_safety_offset(&layer_storage[this_layer].polygons);
        layer_storage[this_layer].polygons = merged;
    }

    // SupportCommon.cpp:875-878
    pub fn polygons_append(&self, dst: &mut Polygons, layer_storage: &SupportGeneratorLayerStorage) {
        if let Some(layer) = self.layer {
            if !layer_storage[layer].polygons.is_empty() {
                dst.extend(layer_storage[layer].polygons.iter().cloned());
            }
        }
    }
}

// SupportCommon.cpp:893-916
#[derive(Debug, Clone)]
pub struct LoopInterfaceProcessor {
    pub n_contact_loops: i32,
    pub circle_radius: f64,
    pub circle_distance: f64,
    pub circle: Polygon,
}

impl LoopInterfaceProcessor {
    // SupportCommon.cpp:895-906
    pub fn new(circle_r: f64) -> Self {
        let mut this = LoopInterfaceProcessor { n_contact_loops: 0, circle_radius: circle_r, circle_distance: circle_r * 3., circle: Polygon::new() };
        // Shape of the top contact area.
        this.circle.points.reserve(6);
        for i in 0..6 {
            let angle: f64 = i as f64 * std::f64::consts::PI / 3.;
            let (sin, cos) = crate::clipper::sin_cos(angle);
            // Point(double, double): lrint
            this.circle.points.push(Point::new(
                (this.circle_radius * cos).round_ties_even() as i32 as Coord,
                (this.circle_radius * sin).round_ties_even() as i32 as Coord,
            ));
        }
        this
    }

    // SupportCommon.cpp:918-1121
    // Generate loop contacts at the top_contact_layer,
    // trim the top_contact_layer->polygons with the areas covered by the loops.
    pub fn generate(&self, top_contact_layer: &mut SupportGeneratorLayerExtruded, interface_flow_src: &Flow, layer_storage: &mut SupportGeneratorLayerStorage) -> FlowResult<()> {
        use cu::ApplySafetyOffset::No;
        if self.n_contact_loops == 0 || top_contact_layer.empty(layer_storage) {
            return Ok(());
        }
        let layer = top_contact_layer.layer.unwrap();

        let flow = interface_flow_src.with_height(layer_storage[layer].height)?;
        let scaled_width_f: f32 = flow.scaled_width() as f32;
        let scaled_spacing = flow.scaled_spacing();

        let mut overhang_polygons: Polygons = Vec::new();
        if let Some(polygons) = layer_storage[layer].overhang_polygons.as_mut() {
            overhang_polygons = std::mem::take(polygons);
        }

        // Generate the outermost loop.
        // Find centerline of the external loop (or any other kind of extrusions should the loop be skipped)
        let mut top_contact_expolygons: ExPolygons =
            cu::offset_ex_expolygons(&cu::union_ex(&layer_storage[layer].polygons), -0.5f32 * scaled_width_f, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);

        // Grid size and bit shifts for quick and exact to/from grid coordinates manipulation.
        // (circle_grid_resolution and circle_grid_powerof2 are computed and not used, SupportCommon.cpp:934-943.)

        let mut loops0: Polygons;
        {
            // find centerline of the external loop of the contours
            // Only consider the loops facing the overhang.
            let mut external_loops: Polygons = Vec::new();
            // Holes in the external loops.
            let mut circles: Polygons = Vec::new();
            let overhang_with_margin: Polygons =
                cu::offset_expolygons(&cu::union_ex(&overhang_polygons), 0.5f32 * scaled_width_f, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
            for contact_expoly in top_contact_expolygons.iter_mut() {
                // Store the circle centers placed for an expolygon into a regular grid, hashed by the circle centers.
                let mut circle_centers_lookup =
                    crate::geometry::ClosestPointInRadiusLookup::new((self.circle_distance - scale_(EPSILON)) as i32 as Coord, |pt: &Point| Some(*pt));
                let mut circle_centers: Vec<Point> = Vec::new();
                let mut center_last = Point::new(0, 0);
                // For each contour of the expolygon, start with the outer contour, continue with the holes.
                for i_contour in 0..=contact_expoly.holes.len() {
                    let contour: &mut Polygon = if i_contour == 0 { &mut contact_expoly.contour } else { &mut contact_expoly.holes[i_contour - 1] };
                    // `seg_current_pt`: the index of the point it points at.
                    let mut seg_current_pt: Option<usize> = None;
                    let mut seg_current_t: f64 = 0.;
                    if !cu::intersection_pl(&[contour.split_at_first_point()], &overhang_with_margin).is_empty() {
                        // The contour is below the overhang at least to some extent.
                        //FIXME ideally one would place the circles below the overhang only.
                        // Walk around the contour and place circles so their centers are not closer than circle_distance from each other.
                        if circle_centers.is_empty() {
                            // Place the first circle.
                            seg_current_pt = Some(0);
                            seg_current_t = 0.;
                            center_last = contour.points[0];
                            circle_centers_lookup.insert(center_last);
                            circle_centers.push(center_last);
                        }
                        let mut it: usize = 1;
                        while it < contour.points.len() {
                            // Is it possible to place a circle on this segment? Is it not too close to any of the circles already placed on this contour?
                            let p1 = contour.points[it - 1];
                            let p2 = contour.points[it];
                            // Intersection of a ray (p1, p2) with a circle placed at center_last, with radius of circle_distance.
                            let v_seg = (p2.x as f64 - p1.x as f64, p2.y as f64 - p1.y as f64);
                            // coordf_t(p1(0) - center_last(0)): the difference of two coord_t
                            let v_cntr = (
                                (p1.x as i32).wrapping_sub(center_last.x as i32) as f64,
                                (p1.y as i32).wrapping_sub(center_last.y as i32) as f64,
                            );
                            let a: f64 = v_seg.0 * v_seg.0 + v_seg.1 * v_seg.1;
                            let b: f64 = 2. * (v_seg.0 * v_cntr.0 + v_seg.1 * v_cntr.1);
                            // v_cntr.squaredNorm() - circle_distance * circle_distance
                            let c: f64 = (-self.circle_distance).mul_add(self.circle_distance, v_cntr.0 * v_cntr.0 + v_cntr.1 * v_cntr.1);
                            // b * b - 4. * a * c
                            let disc: f64 = b.mul_add(b, -(4. * a * c));
                            if disc > 0. {
                                // The circle intersects a ray. Avoid the parts of the segment inside the circle.
                                let t1: f64 = (-b - disc.sqrt()) / (2. * a);
                                let t2: f64 = (-b + disc.sqrt()) / (2. * a);
                                let t0: f64 = if seg_current_pt == Some(it - 1) { seg_current_t } else { 0. };
                                // Take the lowest t in <t0, 1.>, excluding <t1, t2>.
                                let t: f64;
                                if t0 <= t1 {
                                    t = t0;
                                } else if t2 <= 1. {
                                    t = t2;
                                } else {
                                    // Try the following segment.
                                    seg_current_pt = None;
                                    it += 1;
                                    continue;
                                }
                                seg_current_pt = Some(it - 1);
                                seg_current_t = t;
                                center_last = Point::new(
                                    (p1.x as i32).wrapping_add((v_seg.0 * t) as i32) as Coord,
                                    (p1.y as i32).wrapping_add((v_seg.1 * t) as i32) as Coord,
                                );
                                // It has been verified that the new point is far enough from center_last.
                                // Ensure, that it is far enough from all the centers.
                                if circle_centers_lookup.find(&center_last).is_some() {
                                    // -- it; continue;
                                    continue;
                                }
                            } else {
                                // All of the segment is outside the circle. Take the first point.
                                seg_current_pt = Some(it - 1);
                                seg_current_t = 0.;
                                center_last = p1;
                            }
                            // Place the first circle.
                            circle_centers_lookup.insert(center_last);
                            circle_centers.push(center_last);
                            it += 1;
                        }
                        external_loops.push(std::mem::take(contour));
                        for center in &circle_centers {
                            let mut c = self.circle.clone();
                            for p in c.points.iter_mut() {
                                *p = Point::new((p.x as i32).wrapping_add(center.x as i32) as Coord, (p.y as i32).wrapping_add(center.y as i32) as Coord);
                            }
                            circles.push(c);
                        }
                    }
                    let _ = seg_current_t;
                }
            }
            // Apply a pattern to the external loops.
            loops0 = cu::diff(&external_loops, &circles, No);
        }

        let mut loop_lines: Polylines;
        {
            // make more loops
            let mut loop_polygons: Polygons = loops0.clone();
            for i in 1..self.n_contact_loops {
                // opening(loops0, i * flow.scaled_spacing() + 0.5f * flow.scaled_spacing(), 0.5f * flow.scaled_spacing())
                loop_polygons.extend(cu::opening(
                    &loops0,
                    0.5f32.mul_add(scaled_spacing as f32, ((i as i32).wrapping_mul(scaled_spacing as i32)) as f32),
                    0.5f32 * scaled_spacing as f32,
                    cu::DEFAULT_JOIN_TYPE,
                    cu::DEFAULT_MITER_LIMIT,
                ));
            }
            // Clip such loops to the side oriented towards the object.
            // Collect split points, so they will be recognized after the clipping.
            // At the split points the clipped pieces will be stitched back together.
            loop_lines = Vec::with_capacity(loop_polygons.len());
            let mut map_split_points: std::collections::HashMap<(Coord, Coord), i32> = std::collections::HashMap::new();
            for it in &loop_polygons {
                debug_assert!(!map_split_points.contains_key(&(it.points[0].x, it.points[0].y)));
                map_split_points.insert((it.points[0].x, it.points[0].y), -1);
                loop_lines.push(it.split_at_first_point());
            }
            // expand(overhang_polygons, scale_(SUPPORT_MATERIAL_MARGIN))
            loop_lines = cu::intersection_pl(
                &loop_lines,
                &cu::expand(&overhang_polygons, scale_(SUPPORT_MATERIAL_MARGIN) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
            );
            // Because a closed loop has been split to a line, loop_lines may contain continuous segments split to 2 pieces.
            // Try to connect them.
            for i_line in 0..loop_lines.len() {
                if loop_lines[i_line].points.is_empty() {
                    // first_point() of an empty polyline: not reached with what intersection_pl() makes
                    continue;
                }
                let first = loop_lines[i_line].points[0];
                if let Some(second) = map_split_points.get_mut(&(first.x, first.y)) {
                    // This is a stitching point.
                    // If this assert triggers, multiple source polygons likely intersected at this point.
                    debug_assert!(*second != -2);
                    if *second < 0 {
                        // First occurence.
                        *second = i_line as i32;
                    } else {
                        // Second occurence. Join the lines.
                        let idx_1st = *second as usize;
                        *second = -2;
                        let polyline = std::mem::take(&mut loop_lines[i_line]);
                        let polyline_1st = &mut loop_lines[idx_1st];
                        debug_assert!(polyline_1st.points[0] == first || *polyline_1st.points.last().unwrap() == first);
                        if polyline_1st.points[0] == first {
                            polyline_1st.reverse();
                        }
                        polyline_append_moved(polyline_1st, polyline);
                    }
                    continue;
                }
                let last = *loop_lines[i_line].points.last().unwrap();
                if let Some(second) = map_split_points.get_mut(&(last.x, last.y)) {
                    // This is a stitching point.
                    // If this assert triggers, multiple source polygons likely intersected at this point.
                    debug_assert!(*second != -2);
                    if *second < 0 {
                        // First occurence.
                        *second = i_line as i32;
                    } else {
                        // Second occurence. Join the lines.
                        let idx_1st = *second as usize;
                        *second = -2;
                        let mut polyline = std::mem::take(&mut loop_lines[i_line]);
                        let polyline_1st = &mut loop_lines[idx_1st];
                        debug_assert!(polyline_1st.points[0] == last || *polyline_1st.points.last().unwrap() == last);
                        if polyline_1st.points[0] == last {
                            polyline_1st.reverse();
                        }
                        polyline.reverse();
                        polyline_append_moved(polyline_1st, polyline);
                    }
                }
            }
            // Remove empty lines.
            // remove_degenerate(Polylines &): the polylines of less than two points go (Polyline.cpp)
            loop_lines.retain(|pl| pl.points.len() >= 2);
        }
        let _ = &mut loops0;

        // add the contact infill area to the interface area
        // note that growing loops by $circle_radius ensures no tiny
        // extrusions are left inside the circles; however it creates
        // a very large gap between loops and contact_infill_polygons, so maybe another
        // solution should be found to achieve both goals
        // Store the trimmed polygons into a separate polygon set, so the original infill area remains intact for
        // "modulate by layer thickness".
        top_contact_layer.set_polygons_to_extrude(cu::diff(
            &layer_storage[layer].polygons,
            // offset(loop_lines, float(circle_radius * 1.1))
            &offset_polylines_or_none(&loop_lines, (self.circle_radius * 1.1) as f32),
            No,
        ));

        // Transform loops into ExtrusionPath objects.
        extrusion_entities_append_paths(
            &mut top_contact_layer.extrusions,
            loop_lines,
            ExtrusionRole::SupportMaterialInterface,
            flow.mm3_per_mm()?,
            flow.width() as f32,
            flow.height() as f32,
            true,
        );
        Ok(())
    }
}

// `polyline_1st.append(std::move(polyline))` (Polyline.cpp:381-403) with `polyline` taken out of
// `loop_lines[i_line]`: the C++ leaves an invalid source where it was and a moved one empty, and
// remove_degenerate() drops either.
fn polyline_append_moved(polyline_1st: &mut crate::geometry::Polyline, polyline: crate::geometry::Polyline) {
    if polyline.points.len() < 2 {
        return;
    }
    if polyline_1st.points.is_empty() {
        *polyline_1st = polyline;
    } else {
        polyline_1st.append(&polyline);
    }
}

// `offset(const Polylines &, float delta)` with the defaults of ClipperUtils.hpp (jtSquare, 0., etOpenButt);
// no polylines, no polygons.
fn offset_polylines_or_none(polylines: &Polylines, delta: f32) -> Polygons {
    if polylines.is_empty() {
        Vec::new()
    } else {
        cu::offset_polylines(polylines, delta, cu::DEFAULT_LINE_JOIN_TYPE, cu::DEFAULT_LINE_MITER_LIMIT, cu::DEFAULT_END_TYPE)
    }
}

/// `BoundingBox` (BoundingBox.hpp) as SupportCommon.cpp reads it: a box made of points is `defined` only
/// when it has a positive area (`construct()`, BoundingBox.hpp:88-102), an undefined box does not take
/// part in `merge()`, and `offset()` rounds its delta with `Point(double, double)`.
///
/// FIDELITY-NOTE: `crate::geometry::BoundingBox` is defined from its first point on, which is another
/// answer for the boxes of axis-parallel paths that SupportCommon.cpp:1297-1308 tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BBox {
    pub min: Point,
    pub max: Point,
    pub defined: bool,
}

impl BBox {
    /// BoundingBox.hpp:20 `BoundingBoxBase()`
    pub fn new() -> Self {
        BBox { min: Point::new(0, 0), max: Point::new(0, 0), defined: false }
    }
    /// BoundingBox.hpp:32-34, 88-102 `BoundingBoxBase(const std::vector<PointClass> &points)`
    pub fn from_points(points: &[Point]) -> Self {
        let mut out = BBox::new();
        if let Some(&first) = points.first() {
            out.min = first;
            out.max = first;
            for p in &points[1..] {
                out.min = Point::new(out.min.x.min(p.x), out.min.y.min(p.y));
                out.max = Point::new(out.max.x.max(p.x), out.max.y.max(p.y));
            }
            out.defined = out.min.x < out.max.x && out.min.y < out.max.y;
        }
        out
    }
    /// BoundingBox.cpp:98-111 `merge(const BoundingBoxBase<PointClass> &bb)`
    pub fn merge(&mut self, bb: &BBox) {
        if bb.defined {
            if self.defined {
                self.min = Point::new(self.min.x.min(bb.min.x), self.min.y.min(bb.min.y));
                self.max = Point::new(self.max.x.max(bb.max.x), self.max.y.max(bb.max.y));
            } else {
                self.min = bb.min;
                self.max = bb.max;
                self.defined = true;
            }
        }
    }
    /// BoundingBox.cpp:89-93 `merge(const std::vector<PointClass> &points)`
    pub fn merge_points(&mut self, points: &[Point]) {
        self.merge(&BBox::from_points(points));
    }
    /// BoundingBox.cpp:212-218 `offset(coordf_t delta)`: `PointClass v(delta, delta)` rounds (Point.hpp:181).
    pub fn offset(&mut self, delta: f64) {
        let v = delta.round_ties_even() as i32;
        self.min = Point::new((self.min.x as i32).wrapping_sub(v) as Coord, (self.min.y as i32).wrapping_sub(v) as Coord);
        self.max = Point::new((self.max.x as i32).wrapping_add(v) as Coord, (self.max.y as i32).wrapping_add(v) as Coord);
    }
    /// BoundingBox.hpp:48 `inflated(coordf_t delta)`
    pub fn inflated(&self, delta: f64) -> BBox {
        let mut out = *self;
        out.offset(delta);
        out
    }
    /// BoundingBox.hpp:57-60 `overlap(const BoundingBoxBase<PointClass> &other)`
    pub fn overlap(&self, other: &BBox) -> bool {
        !(self.max.x < other.min.x || self.min.x > other.max.x || self.max.y < other.min.y || self.min.y > other.max.y)
    }
    /// The box with the same corners for the functions of the crate that take one.
    pub fn to_bounding_box(&self) -> crate::geometry::BoundingBox {
        crate::geometry::BoundingBox::from_points_minmax(self.min, self.max)
    }
}

impl Default for BBox {
    fn default() -> Self {
        BBox::new()
    }
}

/// Polygon.cpp:416-425 `BoundingBox get_extents(const Polygons &polygons)`
pub fn get_extents_polygons_bbox(polygons: &[Polygon]) -> BBox {
    let mut bb = BBox::new();
    if !polygons.is_empty() {
        bb = BBox::from_points(&polygons[0].points);
        for polygon in &polygons[1..] {
            bb.merge(&BBox::from_points(&polygon.points));
        }
    }
    bb
}

/// Polyline.cpp:522-531 `BoundingBox get_extents(const Polylines &polylines)`
pub fn get_extents_polylines_bbox(polylines: &[crate::geometry::Polyline]) -> BBox {
    let mut bb = BBox::new();
    if !polylines.is_empty() {
        bb = BBox::from_points(&polylines[0].points);
        for polyline in &polylines[1..] {
            bb.merge_points(&polyline.points);
        }
    }
    bb
}

// SupportCommon.cpp:1212-1221, `struct ExtrusionPathFragment` of modulate_extrusion_paths_by_overlapping_layers().
#[derive(Debug, Clone)]
struct ExtrusionPathFragment {
    polylines: Polylines,
    mm3_per_mm: f64,
    width: f32,
    height: f32,
}

// SupportCommon.cpp:1332-1339, `struct ExtrusionPathFragmentEnd`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ExtrusionPathFragmentEnd {
    layer_idx: usize,
    polyline_idx: usize,
    is_start: bool,
}

// SupportCommon.cpp:1194-1444
pub fn modulate_extrusion_paths_by_overlapping_layers(
    // Extrusions generated for this_layer.
    extrusions_in_out: &mut ExtrusionEntitiesPtr,
    this_layer: usize,
    // Multiple layers overlapping with this_layer, sorted bottom up.
    overlapping_layers: &SupportGeneratorLayersPtr,
    layer_storage: &SupportGeneratorLayerStorage,
) -> FlowResult<()> {
    let n_overlapping_layers = overlapping_layers.len();
    if n_overlapping_layers == 0 || extrusions_in_out.is_empty() {
        // The extrusions do not overlap with any other extrusion.
        return Ok(());
    }

    // Get the initial extrusion parameters.
    let (extrusion_role, extrusion_width, template): (ExtrusionRole, f32, ExtrusionPathFragment) = match &extrusions_in_out[0] {
        ExtrusionEntityType::Path(path) => (
            path.role,
            path.width as f32,
            ExtrusionPathFragment { polylines: Vec::new(), mm3_per_mm: path.mm3_per_mm, width: path.width as f32, height: path.height as f32 },
        ),
        _ => panic!("modulate_extrusion_paths_by_overlapping_layers: the extrusions are ExtrusionPath objects"),
    };

    // Split the extrusions by the overlapping layers, reduce their extrusion rate.
    // The last path_fragment is from this_layer.
    let path_fragments: std::cell::RefCell<Vec<ExtrusionPathFragment>> = std::cell::RefCell::new(vec![template; n_overlapping_layers + 1]);

    // End points of the original paths.
    let mut path_ends: Vec<(Point, Point)> = Vec::new();
    // Collect the paths of this_layer.
    {
        let mut fragments = path_fragments.borrow_mut();
        let polylines = &mut fragments.last_mut().unwrap().polylines;
        for ee in extrusions_in_out.drain(..) {
            match ee {
                ExtrusionEntityType::Path(path) => {
                    path_ends.push((path.polyline.points[0], *path.polyline.points.last().unwrap()));
                    polylines.push(path.polyline);
                }
                _ => panic!("modulate_extrusion_paths_by_overlapping_layers: the extrusions are ExtrusionPath objects"),
            }
        }
    }
    // Destroy the original extrusion paths, their polylines were moved to path_fragments already.
    // This will be the destination for the new paths.

    // Fragment the path segments by overlapping layers. The overlapping layers are sorted by an increasing print_z.
    // Trim by the highest overlapping layer first.
    // 重叠层外扩后碰不到路径时跳过裁剪，结果不变。
    {
        let mut fragments = path_fragments.borrow_mut();
        let mut paths_bbox = BBox::new();
        for polyline in &fragments.last().unwrap().polylines {
            if !polyline.points.is_empty() {
                paths_bbox.merge(&BBox::from_points(&polyline.points));
            }
        }
        for i_overlapping_layer in (0..n_overlapping_layers).rev() {
            let overlapping_layer = &layer_storage[overlapping_layers[i_overlapping_layer]];
            let mut trimming_bbox = get_extents_polygons_bbox(&overlapping_layer.polygons);
            // miter 尖角最多伸出 DefaultMiterLimit 倍偏移距离
            if trimming_bbox.defined {
                // coord_t(DefaultMiterLimit * scale_(0.5 * extrusion_width)) + SCALED_EPSILON
                trimming_bbox.offset((cu::DEFAULT_MITER_LIMIT * scale_(0.5 * extrusion_width as f64)) as i32 as f64 + scale_(EPSILON));
            }
            if paths_bbox.defined && trimming_bbox.defined && paths_bbox.overlap(&trimming_bbox) {
                let polygons_trimming: Polygons = cu::offset_expolygons(
                    &cu::union_ex(&overlapping_layer.polygons),
                    scale_(0.5 * extrusion_width as f64) as f32,
                    cu::DEFAULT_JOIN_TYPE,
                    cu::DEFAULT_MITER_LIMIT,
                );
                let last = n_overlapping_layers;
                let trimmed = cu::intersection_pl(&fragments[last].polylines, &polygons_trimming);
                fragments[i_overlapping_layer].polylines = trimmed;
                let rest = cu::diff_pl(&fragments[last].polylines, &polygons_trimming);
                fragments[last].polylines = rest;
            }
            // Adjust the extrusion parameters for a reduced layer height and a non-bridging flow (nozzle_dmr = -1, does not matter).
            debug_assert!(layer_storage[this_layer].print_z > overlapping_layer.print_z);
            let frag = &mut fragments[i_overlapping_layer];
            frag.height = (layer_storage[this_layer].print_z - overlapping_layer.print_z) as f32;
            frag.mm3_per_mm = Flow::new(frag.width as f64, frag.height as f64, -1.)?.mm3_per_mm()?;
        }
    }

    // Now chain the split segments using hashing and a nearly exact match, maintaining the order of segments.
    // Create a single ExtrusionPath or ExtrusionEntityCollection per source ExtrusionPath.
    // Map of fragment start/end points to a pair of <i_overlapping_layer, i_polyline_in_layer>
    // Because a non-exact matching is used for the end points, a multi-map is used.
    // As the clipper library may reverse the order of some clipped paths, store both ends into the map.
    let search_radius: Coord = 7;
    // Return an end point of a fragment, or nullptr if the fragment has been consumed already.
    let accessor = |fragment_end: &ExtrusionPathFragmentEnd| -> Option<Point> {
        let fragments = path_fragments.borrow();
        let polyline = &fragments[fragment_end.layer_idx].polylines[fragment_end.polyline_idx];
        if polyline.points.is_empty() {
            None
        } else if fragment_end.is_start {
            Some(polyline.points[0])
        } else {
            Some(*polyline.points.last().unwrap())
        }
    };
    let mut map_fragment_starts = crate::geometry::ClosestPointInRadiusLookup::new(search_radius, accessor);
    for i_overlapping_layer in 0..=n_overlapping_layers {
        let n_polylines = path_fragments.borrow()[i_overlapping_layer].polylines.len();
        for i_polyline in 0..n_polylines {
            // Map a starting point of a polyline to a pair of <layer, polyline>
            if path_fragments.borrow()[i_overlapping_layer].polylines[i_polyline].points.len() >= 2 {
                map_fragment_starts.insert(ExtrusionPathFragmentEnd { layer_idx: i_overlapping_layer, polyline_idx: i_polyline, is_start: true });
                map_fragment_starts.insert(ExtrusionPathFragmentEnd { layer_idx: i_overlapping_layer, polyline_idx: i_polyline, is_start: false });
            }
        }
    }

    // For each source path:
    for &(pt_start, pt_end) in &path_ends {
        let mut pt_current = pt_start;
        // Find a chain of fragments with the original / reduced print height.
        let mut multipath = ExtrusionMultiPath::new();
        loop {
            // Find a closest end point to pt_current.
            let end_and_dist2 = map_fragment_starts.find(&pt_current).map(|(end, dist2)| (*end, dist2));
            // There may be a bug in Clipper flipping the order of two last points in a fragment?
            let Some((fragment_end_min, _)) = end_and_dist2 else {
                // New fragment connecting to pt_current was not found.
                // Verify that the last point found is close to the original end point of the unfragmented path.
                // End of the path.
                break;
            };
            // Fragment to consume.
            let (frag_mm3_per_mm, frag_width, frag_height, mut frag_polyline) = {
                let mut fragments = path_fragments.borrow_mut();
                let frag = &mut fragments[fragment_end_min.layer_idx];
                (frag.mm3_per_mm, frag.width, frag.height, std::mem::take(&mut frag.polylines[fragment_end_min.polyline_idx]))
            };
            // Path to append the fragment to.
            let mut have_path = !multipath.paths.is_empty();
            if have_path {
                // Verify whether the path is compatible with the current fragment.
                let path = multipath.paths.last().unwrap();
                if path.height as f32 != frag_height || path.mm3_per_mm != frag_mm3_per_mm {
                    have_path = false;
                }
                // Merging with the previous path. This can only happen if the current layer was reduced by a base layer, which was split into a base and interface layer.
            }
            if !have_path {
                // Allocate a new path.
                multipath.paths.push(ExtrusionPath::with_params(extrusion_role, frag_mm3_per_mm, frag_width as f64, frag_height as f64, false));
            }
            let path = multipath.paths.last_mut().unwrap();
            // The Clipper library may flip the order of the clipped polylines arbitrarily.
            // Reverse the source polyline, if connecting to the end.
            if !fragment_end_min.is_start {
                frag_polyline.reverse();
            }
            // Enforce exact overlap of the end points of successive fragments.
            frag_polyline.points[0] = pt_current;
            // Don't repeat the first point.
            if !path.polyline.points.is_empty() {
                path.polyline.points.pop();
            }
            // Consume the fragment's polyline, remove it from the input fragments, so it will be ignored the next time.
            if path.polyline.points.is_empty() {
                path.polyline = frag_polyline;
            } else {
                path.polyline.append(&frag_polyline);
            }
            pt_current = *path.polyline.points.last().unwrap();
            if pt_current == pt_end {
                // End of the path.
                break;
            }
        }
        if !multipath.paths.is_empty() {
            if multipath.paths.len() == 1 {
                // This path was not fragmented.
                extrusions_in_out.push(ExtrusionEntityType::Path(multipath.paths.pop().unwrap()));
            } else {
                // This path was fragmented. Copy the collection as a whole object, so the order inside the collection will not be changed
                // during the chaining of extrusions_in_out.
                extrusions_in_out.push(ExtrusionEntityType::MultiPath(multipath));
            }
        }
    }
    drop(map_fragment_starts);
    // If there are any non-consumed fragments, add them separately.
    //FIXME this shall not happen, if the Clipper works as expected and all paths split to fragments could be re-connected.
    for fragment in path_fragments.into_inner() {
        extrusion_entities_append_paths(extrusions_in_out, fragment.polylines, extrusion_role, fragment.mm3_per_mm, fragment.width, fragment.height, true);
    }
    Ok(())
}

// SupportCommon.cpp:1446-1488
pub fn modulate_extrusion_by_overlapping_layers(
    extrusions_in_out: &mut ExtrusionEntitiesPtr,
    this_layer: usize,
    overlapping_layers: &SupportGeneratorLayersPtr,
    layer_storage: &SupportGeneratorLayerStorage,
) -> FlowResult<()> {
    if overlapping_layers.is_empty() || extrusions_in_out.is_empty() {
        return Ok(());
    }

    // Tree support double walls are grouped in no-sort collections. Preserve that hierarchy while
    // applying the existing path modulation to each run of paths inside the collection.
    let mut modulated: ExtrusionEntitiesPtr = Vec::new();
    let mut paths: ExtrusionEntitiesPtr = Vec::new();
    fn flush_paths(
        paths: &mut ExtrusionEntitiesPtr,
        modulated: &mut ExtrusionEntitiesPtr,
        this_layer: usize,
        overlapping_layers: &SupportGeneratorLayersPtr,
        layer_storage: &SupportGeneratorLayerStorage,
    ) -> FlowResult<()> {
        if paths.is_empty() {
            return Ok(());
        }
        modulate_extrusion_paths_by_overlapping_layers(paths, this_layer, overlapping_layers, layer_storage)?;
        modulated.append(paths);
        Ok(())
    }

    for extrusion in std::mem::take(extrusions_in_out) {
        match extrusion {
            ExtrusionEntityType::Path(_) => paths.push(extrusion),
            ExtrusionEntityType::Collection(mut collection) => {
                flush_paths(&mut paths, &mut modulated, this_layer, overlapping_layers, layer_storage)?;
                modulate_extrusion_by_overlapping_layers(&mut collection.entities, this_layer, overlapping_layers, layer_storage)?;
                if !collection.entities.is_empty() {
                    modulated.push(ExtrusionEntityType::Collection(collection));
                }
            }
            other => {
                // Support path generators are expected to produce paths or collections of paths here.
                // Keep an unexpected entity intact rather than dereferencing an invalid downcast.
                flush_paths(&mut paths, &mut modulated, this_layer, overlapping_layers, layer_storage)?;
                modulated.push(other);
            }
        }
    }
    flush_paths(&mut paths, &mut modulated, this_layer, overlapping_layers, layer_storage)?;

    *extrusions_in_out = modulated;
    Ok(())
}

// Geometry.hpp `template<typename T> T deg2rad(T angle) { return T(PI) * angle / T(180.0); }` with T = float.
fn deg2rad_f32(angle: f32) -> f32 {
    std::f64::consts::PI as f32 * angle / 180f32
}

// ExtrusionEntity.cpp:55-58 `void ExtrusionPath::polygons_covered_by_width(Polygons &out, const float
// scaled_epsilon) const` on ClipperLib (`offset(const Polyline &, float)`, ClipperUtils.cpp:418).
fn extrusion_path_polygons_covered_by_width(path: &ExtrusionPath, out: &mut Polygons, scaled_epsilon: f32) {
    // float(scale_(this->width / 2)) + scaled_epsilon
    let delta: f32 = scale_((path.width as f32 / 2.) as f64) as f32 + scaled_epsilon;
    out.extend(cu::offset_polyline(&path.polyline, delta, cu::DEFAULT_LINE_JOIN_TYPE, cu::DEFAULT_LINE_MITER_LIMIT, cu::DEFAULT_END_TYPE));
}

// SupportCommon.cpp:2724-2734, the lambda `polygonsCoveredByWidth` of generate_support_toolpaths().
fn polygons_covered_by_width(entity: &ExtrusionEntityType, polygons: &mut Polygons, throw_on_cancel: Option<&dyn Fn()>) {
    if let Some(throw_on_cancel) = throw_on_cancel {
        throw_on_cancel();
    }
    match entity {
        ExtrusionEntityType::Collection(collection) => {
            for child in &collection.entities {
                polygons_covered_by_width(child, polygons, throw_on_cancel);
            }
        }
        ExtrusionEntityType::Path(path) => extrusion_path_polygons_covered_by_width(path, polygons, 0.),
        // ExtrusionEntity.cpp:97-101, 432-436
        ExtrusionEntityType::MultiPath(multipath) => {
            for path in &multipath.paths {
                extrusion_path_polygons_covered_by_width(path, polygons, 0.);
            }
        }
        ExtrusionEntityType::Loop(r#loop) => {
            for path in &r#loop.paths {
                extrusion_path_polygons_covered_by_width(path, polygons, 0.);
            }
        }
    }
}

// SupportCommon.cpp:2735-2751, the lambda `setOverhangDegreeImpl` of generate_support_toolpaths().
fn set_overhang_degree_impl(entities: &mut ExtrusionEntitiesPtr, cooldown_areas: &ExPolygons, throw_on_cancel: Option<&dyn Fn()>) {
    for entity in entities.iter_mut() {
        let mut polygons: Polygons = Vec::new();
        polygons_covered_by_width(entity, &mut polygons, throw_on_cancel);
        // to_expolygons(std::move(polygons)): a contour each (ExPolygon.hpp:361-368)
        let expolygons: ExPolygons = polygons.into_iter().map(ExPolygon::new).collect();
        if cu_expolygon::overlaps_expolygons(&expolygons, cooldown_areas) {
            match entity {
                ExtrusionEntityType::Collection(collection) => set_overhang_degree_impl(&mut collection.entities, cooldown_areas, throw_on_cancel),
                ExtrusionEntityType::Path(path) => path.set_overhang_degree(10),
                ExtrusionEntityType::MultiPath(multipath) => {
                    for path in multipath.paths.iter_mut() {
                        path.set_overhang_degree(10);
                    }
                }
                ExtrusionEntityType::Loop(r#loop) => {
                    for path in r#loop.paths.iter_mut() {
                        path.set_overhang_degree(10);
                    }
                }
            }
        }
    }
}

// SupportCommon.cpp:2384-2406, `struct LayerCacheItem` and `struct LayerCache` of generate_support_toolpaths().
// The five `SupportGeneratorLayerExtruded` of a cache are `layers[LC_*]`; an item names its one by that index.
const LC_BOTTOM_CONTACT: usize = 0;
const LC_TOP_CONTACT: usize = 1;
const LC_BASE: usize = 2;
const LC_INTERFACE: usize = 3;
const LC_BASE_INTERFACE: usize = 4;

#[derive(Debug, Clone, Default)]
struct LayerCacheItem {
    layer_extruded: usize,
    overlapping: Vec<usize>,
}

#[derive(Debug, Clone, Default)]
struct LayerCache {
    layers: [SupportGeneratorLayerExtruded; 5],
    nonempty: Vec<LayerCacheItem>,
    // support interface ironing related
    polys_to_iron: Polygons,
}

impl LayerCache {
    // SupportCommon.cpp:2399-2405
    fn add_nonempty_and_sort(&mut self, layer_storage: &SupportGeneratorLayerStorage) {
        for item in [LC_BOTTOM_CONTACT, LC_TOP_CONTACT, LC_INTERFACE, LC_BASE_INTERFACE, LC_BASE] {
            if !self.layers[item].empty(layer_storage) {
                self.nonempty.push(LayerCacheItem { layer_extruded: item, overlapping: Vec::new() });
            }
        }
        // Sort the layers with the same print_z coordinate by their heights, thickest first.
        let height = |lc: &LayerCacheItem| layer_storage[self.layers[lc.layer_extruded].layer.unwrap()].height;
        let mut nonempty = std::mem::take(&mut self.nonempty);
        nonempty.sort_by(|lc1, lc2| {
            if height(lc1) > height(lc2) {
                std::cmp::Ordering::Less
            } else if height(lc2) > height(lc1) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });
        self.nonempty = nonempty;
    }
}

// Two of the five layers of a cache, mutably.
fn layer_cache_pair(layers: &mut [SupportGeneratorLayerExtruded; 5], a: usize, b: usize) -> (&mut SupportGeneratorLayerExtruded, &mut SupportGeneratorLayerExtruded) {
    debug_assert!(a != b);
    if a < b {
        let (lo, hi) = layers.split_at_mut(b);
        (&mut lo[a], &mut hi[0])
    } else {
        let (lo, hi) = layers.split_at_mut(a);
        (&mut hi[0], &mut lo[b])
    }
}

// SupportCommon.cpp:2420-2441: the fillers of a range of generate_support_toolpaths().
struct SupportFillers {
    filler_interface: Box<dyn SupportFill>,
    // Filler for the 1st layer interface, if different from filler_interface.
    filler_first_layer_ptr: Option<Box<dyn SupportFill>>,
    // Filler for the 1st layer interface, if different from filler_interface.
    filler_raft_contact_ptr: Option<Box<dyn SupportFill>>,
    // Filler for the base interface (to be used for soluble interface / non soluble base, to produce non soluble interface layer below soluble interface layer).
    filler_base_interface: Option<Box<dyn SupportFill>>,
    filler_support: Box<dyn SupportFill>,
}

impl SupportFillers {
    // Pointer to the 1st layer interface filler.
    fn filler_first_layer(&mut self) -> &mut dyn SupportFill {
        match self.filler_first_layer_ptr.as_mut() {
            Some(filler) => filler.as_mut(),
            None => self.filler_interface.as_mut(),
        }
    }
    // Pointer to the 1st layer interface filler.
    fn filler_raft_contact(&mut self) -> &mut dyn SupportFill {
        match self.filler_raft_contact_ptr.as_mut() {
            Some(filler) => filler.as_mut(),
            None => self.filler_interface.as_mut(),
        }
    }
}

// SupportCommon.cpp:2533 `enum class InterfaceLayerType`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InterfaceLayerType {
    TopContact,
    BottomContact,
    RaftContact,
    Interface,
    InterfaceAsBase,
}

// SupportCommon.cpp:2534-2595, the lambda `extrude_interface` of generate_support_toolpaths().
#[allow(clippy::too_many_arguments)]
fn extrude_interface(
    layer_ex: &mut SupportGeneratorLayerExtruded,
    interface_layer_type: InterfaceLayerType,
    layer_storage: &SupportGeneratorLayerStorage,
    fillers: &mut SupportFillers,
    config: &pf::PrintObjectConfig,
    support_params: &SupportParameters,
    support_layer: &pf::SupportLayer,
    support_layer_id: usize,
    support_interface_angle: f32,
    angles: &[f32],
    link_max_length_factor: f64,
) -> FlowResult<()> {
    if !layer_ex.empty(layer_storage) && !layer_ex.polygons_to_extrude(layer_storage).is_empty() {
        let layer = &layer_storage[layer_ex.layer.unwrap()];
        let interface_as_base = interface_layer_type == InterfaceLayerType::InterfaceAsBase;
        let raft_contact = interface_layer_type == InterfaceLayerType::RaftContact;
        //FIXME Bottom interfaces are extruded with the briding flow. Some bridging layers have its height slightly reduced, therefore
        // the bridging flow does not quite apply. Reduce the flow to area of an ellipse? (A = pi * a * b)
        let mut interface_flow_src: Flow = if raft_contact {
            support_params.raft_interface_flow
        } else if interface_as_base {
            support_params.support_material_flow
        } else {
            support_params.support_material_interface_flow
        };
        if !raft_contact && support_layer.planned_interface_nozzle_diameter > EPSILON {
            interface_flow_src = if interface_as_base {
                support_params.support_material_flow_for_nozzle(support_layer.planned_interface_nozzle_diameter, layer.height as f32)?
            } else {
                support_params.support_material_interface_flow_for_nozzle(support_layer.planned_interface_nozzle_diameter, layer.height as f32)?
            };
        }
        let bridge_nozzle: f64 = if support_layer.planned_interface_nozzle_diameter > EPSILON {
            support_layer.planned_interface_nozzle_diameter
        } else {
            support_params.support_material_bottom_interface_flow.nozzle_diameter()
        };
        let interface_flow = if layer.bridging {
            // Flow::bridging_flow(float dmr, float nozzle_diameter)
            Flow::bridging_flow(layer.height as f32 as f64, bridge_nozzle as f32 as f64)
        } else {
            interface_flow_src.with_height(layer.height as f32 as f64)?
        };
        let angle: f32 = if interface_as_base {
            // If zero interface layers are configured, use the same angle as for the base layers.
            angles[support_layer_id % angles.len()]
        } else if raft_contact {
            // Use interface angle for the interface layers.
            support_params.raft_interface_angle(support_layer.interface_id())
        } else {
            support_interface_angle
        };
        let density: f64 = if raft_contact {
            support_params.raft_interface_density
        } else if interface_as_base {
            support_params.support_density
        } else {
            // std::min(1., interface_flow_src.spacing() / (config.support_interface_spacing.value + interface_flow_src.spacing()))
            let v = interface_flow_src.spacing() / (cfg_float(config, "support_interface_spacing") + interface_flow_src.spacing());
            if v < 1. {
                v
            } else {
                1.
            }
        };
        let filler: &mut dyn SupportFill = if raft_contact { fillers.filler_raft_contact() } else { fillers.filler_interface.as_mut() };
        filler.fill_mut().angle = angle;
        // filler->spacing = interface_flow_src.spacing(): the float as a coordf_t
        filler.fill_mut().spacing = interface_flow_src.spacing();
        filler.fill_mut().link_max_length = scale_(filler.fill().spacing * link_max_length_factor / density) as i32 as Coord;
        // replace the old extrusion, avoiding empty infill generation which will cause the floating extrusion
        let expolygons = cu::union_safety_offset_ex(layer_ex.polygons_to_extrude(layer_storage));
        // fill_params.density = density: the double to the float member
        let fill_params = support_fill_params(density as f32);
        for expoly in &expolygons {
            let old_size = layer_ex.extrusions.len();
            // Keep the polygon for the outline fallback when a coarse nozzle cannot fit
            // the configured interface pattern into a narrow contact region.
            fill_expolygon_generate_paths(
                &mut layer_ex.extrusions,
                expoly.clone(),
                filler,
                &fill_params,
                density as f32,
                ExtrusionRole::SupportMaterialInterface,
                &interface_flow,
            )?;
            // no any infill extrusions generated
            if old_size != layer_ex.extrusions.len() {
                continue;
            }
            // using the outline path
            let spacing: f64 = interface_flow.scaled_spacing() as f64;
            let clip_length: f64 = spacing * 0.5;
            // offset_ex(expoly, -clip_length): the double to the float delta
            let fallback_regions: ExPolygons = cu_expolygon::offset_ex(expoly, (-clip_length) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
            for shrink_expoly in &fallback_regions {
                extrusion_entities_append_paths(
                    &mut layer_ex.extrusions,
                    draw_perimeters(shrink_expoly, clip_length),
                    ExtrusionRole::SupportMaterialInterface,
                    interface_flow.mm3_per_mm()?,
                    interface_flow.width() as f32,
                    interface_flow.height() as f32,
                    // Disable reversal of the path, always start with the anchor, always print CCW.
                    false,
                );
            }
        }
    }
    Ok(())
}

// SupportCommon.cpp:2270-2894
//
// The three `tbb::parallel_for` run in the order of the support layers.
//
// FIDELITY-NOTE: the second one makes its fillers per sub-range of the layers, and two of them only in the
// sub-range that begins at layer 0 (`filler_first_layer_ptr`) and at layer `n_raft_layers`
// (`filler_raft_contact_ptr`, SupportCommon.cpp:2422-2427); the other sub-ranges print the first layer and
// the raft contact with `filler_interface`. Which layers share a sub-range is up to the partitioner of tbb.
// As for the `up` flag of generate_interface_layers() the sub-ranges are of one layer here, which is what
// oneTBB makes of a range of up to about 64 layers per thread.
#[allow(clippy::too_many_arguments)]
pub fn generate_support_toolpaths(
    support_layers: &mut [pf::SupportLayer],
    config: &pf::PrintObjectConfig,
    support_params: &SupportParameters,
    slicing_params: &SlicingParameters,
    raft_layers: &SupportGeneratorLayersPtr,
    bottom_contacts: &SupportGeneratorLayersPtr,
    top_contacts: &SupportGeneratorLayersPtr,
    intermediate_layers: &SupportGeneratorLayersPtr,
    interface_layers: &SupportGeneratorLayersPtr,
    base_interface_layers: &SupportGeneratorLayersPtr,
    cooldown_areas: &[ExPolygons],
    mut lightning_infill_lines: Option<&mut Vec<Polylines>>,
    throw_on_cancel: Option<&dyn Fn()>,
    layer_storage: &mut SupportGeneratorLayerStorage,
    fill_factory: &dyn SupportFillFactory,
) -> FlowResult<()> {
    use cu::ApplySafetyOffset::No;
    let mut angles: Vec<f32> = vec![support_params.base_angle];
    if cfg_int(config, "support_base_pattern") == enums::SupportMaterialPattern::SmpRectilinearGrid as i32 {
        angles.push(support_params.interface_angle);
    }

    // BoundingBox bbox_object(Point(-scale_(1.), -scale_(1.0)), Point(scale_(1.), scale_(1.)));
    let bbox_object = crate::geometry::BoundingBox::from_points_minmax(
        Point::new((-scale_(1.)).round_ties_even() as Coord, (-scale_(1.)).round_ties_even() as Coord),
        Point::new(scale_(1.).round_ties_even() as Coord, scale_(1.).round_ties_even() as Coord),
    );

    //    const coordf_t link_max_length_factor = 3.;
    let link_max_length_factor: f64 = 0.;

    // Insert the raft base layers.
    let n_raft_layers: usize = std::cmp::min(support_layers.len(), std::cmp::max(0, slicing_params.raft_layers() as i32 - 1) as usize);

    for support_layer_id in 0..n_raft_layers {
        if let Some(throw_on_cancel) = throw_on_cancel {
            throw_on_cancel();
        }
        debug_assert!(support_layer_id < raft_layers.len());
        let support_layer = &mut support_layers[support_layer_id];
        debug_assert!(support_layer.support_fills.entities.is_empty());
        let raft_layer = &layer_storage[raft_layers[support_layer_id]];

        let mut filler_interface = fill_factory.new_from_type(support_params.raft_interface_fill_pattern);
        let mut filler_support = fill_factory.new_from_type(support_params.base_fill_pattern);
        filler_interface.fill_mut().bounding_box = bbox_object;
        filler_support.fill_mut().bounding_box = bbox_object;

        // Print the tree supports cutting through the raft with the exception of the 1st layer, where a full support layer will be printed below
        // both the raft and the trees.
        // Trim the raft layers with the tree polygons.
        let tree_polygons: Polygons = if support_layer_id > 0
            && support_layer_id < intermediate_layers.len()
            && is_approx(layer_storage[intermediate_layers[support_layer_id]].print_z, support_layer.layer.print_z, EPSILON)
        {
            layer_storage[intermediate_layers[support_layer_id]].polygons.clone()
        } else {
            Vec::new()
        };

        // Print the support base below the support columns, or the support base for the support columns plus the contacts.
        if support_layer_id > 0 {
            let empty: Polygons = Vec::new();
            let to_infill_polygons: &Polygons = if support_layer_id < slicing_params.base_raft_layers as usize {
                &raft_layer.polygons
            } else {
                //FIXME misusing contact_polygons for support columns.
                match raft_layer.contact_polygons.as_ref() {
                    None => &empty,
                    Some(polygons) => polygons,
                }
            };
            // Trees may cut through the raft layers down to a print bed.
            let flow = Flow::new(
                support_params.support_material_flow.width() as f32 as f64,
                raft_layer.height as f32 as f64,
                support_params.support_material_flow.nozzle_diameter(),
            )?;
            debug_assert!(!raft_layer.bridging);
            if !to_infill_polygons.is_empty() {
                let filler = filler_support.as_mut();
                filler.fill_mut().angle = support_params.raft_angle_base;
                filler.fill_mut().spacing = support_params.support_material_flow.spacing();
                filler.fill_mut().link_max_length = scale_(filler.fill().spacing * link_max_length_factor / support_params.support_density) as i32 as Coord;
                let trimmed: Polygons;
                fill_expolygons_with_sheath_generate_paths(
                    // Destination
                    &mut support_layer.support_fills.entities,
                    // Regions to fill
                    if tree_polygons.is_empty() {
                        to_infill_polygons
                    } else {
                        trimmed = cu::diff(to_infill_polygons, &tree_polygons, No);
                        &trimmed
                    },
                    // Filler and its parameters
                    filler,
                    support_params.support_density as f32,
                    // Extrusion parameters
                    ExtrusionRole::SupportMaterial,
                    &flow,
                    support_params,
                    support_params.with_sheath,
                    false,
                )?;
            }
            if !tree_polygons.is_empty() {
                tree_supports_generate_paths(&mut support_layer.support_fills.entities, &tree_polygons, &flow, support_params)?;
            }
        }

        let filler = filler_interface.as_mut();
        let mut flow = support_params.first_layer_flow;
        let density: f32;
        if support_layer_id == 0 {
            // Base flange.
            filler.fill_mut().angle = support_params.raft_angle_1st_layer;
            filler.fill_mut().spacing = support_params.first_layer_flow.spacing();
            density = (cfg_float(config, "raft_first_layer_density") * 0.01) as f32;
        } else if support_layer_id >= slicing_params.base_raft_layers as usize {
            filler.fill_mut().angle = support_params.raft_interface_angle(support_layer.interface_id());
            // We don't use $base_flow->spacing because we need a constant spacing
            // value that guarantees that all layers are correctly aligned.
            filler.fill_mut().spacing = support_params.support_material_flow.spacing();
            debug_assert!(!raft_layer.bridging);
            flow = Flow::new(
                support_params.raft_interface_flow.width() as f32 as f64,
                raft_layer.height as f32 as f64,
                support_params.raft_interface_flow.nozzle_diameter(),
            )?;
            density = support_params.raft_interface_density as f32;
        } else {
            continue;
        }
        filler.fill_mut().link_max_length = scale_(filler.fill().spacing * link_max_length_factor / density as f64) as i32 as Coord;
        let trimmed: Polygons;
        fill_expolygons_with_sheath_generate_paths(
            // Destination
            &mut support_layer.support_fills.entities,
            // Regions to fill
            if tree_polygons.is_empty() {
                &raft_layer.polygons
            } else {
                trimmed = cu::diff(&raft_layer.polygons, &tree_polygons, No);
                &trimmed
            },
            // Filler and its parameters
            filler,
            density,
            // Extrusion parameters
            if support_layer_id < slicing_params.base_raft_layers as usize { ExtrusionRole::SupportMaterial } else { ExtrusionRole::SupportMaterialInterface },
            &flow,
            // sheath at first layer
            support_params,
            support_layer_id == 0,
            support_layer_id == 0,
        )?;
    }

    let mut layer_caches: Vec<LayerCache> = vec![LayerCache::default(); support_layers.len()];

    {
        let support_interface_top_layers = cfg_int(config, "support_interface_top_layers");
        let support_interface_bottom_layers = cfg_int(config, "support_interface_bottom_layers");
        let fill_type_first_layer = InfillPattern::Rectilinear;
        for support_layer_id in n_raft_layers..support_layers.len() {
            // The range of the layer: `range.begin()` is `support_layer_id`.
            // Indices of the 1st layer in their respective container at the support layer height.
            let mut idx_layer_bottom_contact: isize = -1;
            let mut idx_layer_top_contact: isize = -1;
            let mut idx_layer_intermediate: isize = -1;
            let mut idx_layer_interface: isize = -1;
            let mut idx_layer_base_interface: isize = -1;
            let mut fillers = SupportFillers {
                filler_interface: fill_factory.new_from_type(support_params.contact_fill_pattern),
                // range.begin() == 0 && support_params.contact_fill_pattern != fill_type_first_layer
                filler_first_layer_ptr: if support_layer_id == 0 && support_params.contact_fill_pattern != fill_type_first_layer {
                    Some(fill_factory.new_from_type(fill_type_first_layer))
                } else {
                    None
                },
                // range.begin() == n_raft_layers && config.support_interface_top_layers.value == 0
                filler_raft_contact_ptr: if support_layer_id == n_raft_layers && support_interface_top_layers == 0 {
                    Some(fill_factory.new_from_type(support_params.raft_interface_fill_pattern))
                } else {
                    None
                },
                filler_base_interface: if base_interface_layers.is_empty() {
                    None
                } else {
                    Some(fill_factory.new_from_type(if support_params.interface_density > 0.95 || support_params.with_sheath {
                        InfillPattern::Rectilinear
                    } else {
                        InfillPattern::SupportBase
                    }))
                },
                filler_support: fill_factory.new_from_type(support_params.base_fill_pattern),
            };
            fillers.filler_interface.fill_mut().bounding_box = bbox_object;
            if let Some(filler) = fillers.filler_first_layer_ptr.as_mut() {
                filler.fill_mut().bounding_box = bbox_object;
            }
            if let Some(filler) = fillers.filler_raft_contact_ptr.as_mut() {
                filler.fill_mut().bounding_box = bbox_object;
            }
            if let Some(filler) = fillers.filler_base_interface.as_mut() {
                filler.fill_mut().bounding_box = bbox_object;
            }
            fillers.filler_support.fill_mut().bounding_box = bbox_object;
            if let Some(throw_on_cancel) = throw_on_cancel {
                throw_on_cancel();
            }
            let support_layer = &mut support_layers[support_layer_id];
            let layer_cache = &mut layer_caches[support_layer_id];
            let support_layer_print_z = support_layer.layer.print_z;
            let support_interface_angle: f32 = support_params.support_interface_angle(
                support_layer.interface_id(),
                enums::SupportMaterialInterfacePattern::from_i32(cfg_int(config, "support_interface_pattern")),
            );
            // Find polygons with the same print_z.
            // Increment the layer indices to find a layer at support_layer.print_z.
            {
                let storage: &SupportGeneratorLayerStorage = layer_storage;
                let fun = |l: &usize| storage[*l].print_z >= support_layer_print_z - EPSILON;
                idx_layer_bottom_contact = idx_higher_or_equal(bottom_contacts, idx_layer_bottom_contact, fun);
                idx_layer_top_contact = idx_higher_or_equal(top_contacts, idx_layer_top_contact, fun);
                idx_layer_intermediate = idx_higher_or_equal(intermediate_layers, idx_layer_intermediate, fun);
                idx_layer_interface = idx_higher_or_equal(interface_layers, idx_layer_interface, fun);
                idx_layer_base_interface = idx_higher_or_equal(base_interface_layers, idx_layer_base_interface, fun);
            }
            // Copy polygons from the layers.
            {
                let at = |layers: &SupportGeneratorLayersPtr, idx: isize| -> Option<usize> {
                    if (idx as usize) < layers.len() && layer_storage[layers[idx as usize]].print_z < support_layer_print_z + EPSILON {
                        Some(layers[idx as usize])
                    } else {
                        None
                    }
                };
                if let Some(l) = at(bottom_contacts, idx_layer_bottom_contact) {
                    layer_cache.layers[LC_BOTTOM_CONTACT].layer = Some(l);
                }
                if let Some(l) = at(top_contacts, idx_layer_top_contact) {
                    layer_cache.layers[LC_TOP_CONTACT].layer = Some(l);
                }
                if let Some(l) = at(interface_layers, idx_layer_interface) {
                    layer_cache.layers[LC_INTERFACE].layer = Some(l);
                }
                if let Some(l) = at(base_interface_layers, idx_layer_base_interface) {
                    layer_cache.layers[LC_BASE_INTERFACE].layer = Some(l);
                }
                if let Some(l) = at(intermediate_layers, idx_layer_intermediate) {
                    layer_cache.layers[LC_BASE].layer = Some(l);
                }
            }

            // This layer is a raft contact layer. Any contact polygons at this layer are raft contacts.
            let raft_layer: bool = slicing_params.interface_raft_layers != 0
                && layer_cache.layers[LC_TOP_CONTACT].layer.is_some()
                && is_approx(layer_storage[layer_cache.layers[LC_TOP_CONTACT].layer.unwrap()].print_z, slicing_params.raft_contact_top_z, EPSILON);
            if support_interface_top_layers == 0 {
                // If no top interface layers were requested, we treat the contact layer exactly as a generic base layer.
                // Don't merge the raft contact layer though.
                if support_params.can_merge_support_regions && !raft_layer {
                    let (base_layer, top_contact_layer) = layer_cache_pair(&mut layer_cache.layers, LC_BASE, LC_TOP_CONTACT);
                    if base_layer.could_merge(top_contact_layer, layer_storage) {
                        base_layer.merge(top_contact_layer, layer_storage);
                    } else if base_layer.empty(layer_storage) {
                        base_layer.assign(top_contact_layer);
                    }
                }
            } else {
                // save the polygons to extrude in topo contact layer into polys to iron
                if support_params.enable_support_ironing && !layer_cache.layers[LC_TOP_CONTACT].empty(layer_storage) {
                    layer_cache.polys_to_iron = layer_cache.layers[LC_TOP_CONTACT].polygons_to_extrude(layer_storage).clone();
                }
                if !layer_cache.layers[LC_TOP_CONTACT].empty(layer_storage) {
                    let top_contact_height = layer_storage[layer_cache.layers[LC_TOP_CONTACT].layer.unwrap()].height;
                    let contact_loop_flow: Flow = if support_layer.planned_interface_nozzle_diameter > EPSILON {
                        support_params.support_material_interface_flow_for_nozzle(support_layer.planned_interface_nozzle_diameter, top_contact_height as f32)?
                    } else {
                        support_params.support_material_interface_flow
                    };
                    let mut loop_interface_processor = LoopInterfaceProcessor::new(1.5 * contact_loop_flow.scaled_width() as f64);
                    loop_interface_processor.n_contact_loops = if cfg_bool(config, "support_interface_loop_pattern") { 1 } else { 0 };
                    loop_interface_processor.generate(&mut layer_cache.layers[LC_TOP_CONTACT], &contact_loop_flow, layer_storage)?;
                }
                // If no loops are allowed, we treat the contact layer exactly as a generic interface layer.
                // Merge interface_layer into top_contact_layer, as the top_contact_layer is not synchronized and therefore it will be used
                // to trim other layers.
                let (top_contact_layer, interface_layer) = layer_cache_pair(&mut layer_cache.layers, LC_TOP_CONTACT, LC_INTERFACE);
                if top_contact_layer.could_merge(interface_layer, layer_storage) && !raft_layer {
                    top_contact_layer.merge(interface_layer, layer_storage);
                }
            }
            if (support_interface_top_layers == 0 || support_interface_bottom_layers == 0) && support_params.can_merge_support_regions {
                let (base_layer, bottom_contact_layer) = layer_cache_pair(&mut layer_cache.layers, LC_BASE, LC_BOTTOM_CONTACT);
                if base_layer.could_merge(bottom_contact_layer, layer_storage) {
                    base_layer.merge(bottom_contact_layer, layer_storage);
                } else if base_layer.empty(layer_storage)
                    && !bottom_contact_layer.empty(layer_storage)
                    && !layer_storage[bottom_contact_layer.layer.unwrap()].bridging
                {
                    base_layer.assign(bottom_contact_layer);
                }
            } else if {
                let (bottom_contact_layer, top_contact_layer) = layer_cache_pair(&mut layer_cache.layers, LC_BOTTOM_CONTACT, LC_TOP_CONTACT);
                bottom_contact_layer.could_merge(top_contact_layer, layer_storage)
            } && !raft_layer
            {
                let (top_contact_layer, bottom_contact_layer) = layer_cache_pair(&mut layer_cache.layers, LC_TOP_CONTACT, LC_BOTTOM_CONTACT);
                top_contact_layer.merge(bottom_contact_layer, layer_storage);
            } else {
                let (bottom_contact_layer, interface_layer) = layer_cache_pair(&mut layer_cache.layers, LC_BOTTOM_CONTACT, LC_INTERFACE);
                if bottom_contact_layer.could_merge(interface_layer, layer_storage) {
                    bottom_contact_layer.merge(interface_layer, layer_storage);
                }
            }

            // Top and bottom contacts, interface layers.
            let top_interfaces: bool = support_interface_top_layers != 0;
            let bottom_interfaces: bool = top_interfaces && support_interface_bottom_layers != 0;
            for (item, interface_layer_type) in [
                (
                    LC_TOP_CONTACT,
                    if raft_layer {
                        InterfaceLayerType::RaftContact
                    } else if top_interfaces {
                        InterfaceLayerType::TopContact
                    } else {
                        InterfaceLayerType::InterfaceAsBase
                    },
                ),
                (LC_BOTTOM_CONTACT, if bottom_interfaces { InterfaceLayerType::BottomContact } else { InterfaceLayerType::InterfaceAsBase }),
                (LC_INTERFACE, if top_interfaces { InterfaceLayerType::Interface } else { InterfaceLayerType::InterfaceAsBase }),
            ] {
                extrude_interface(
                    &mut layer_cache.layers[item],
                    interface_layer_type,
                    layer_storage,
                    &mut fillers,
                    config,
                    support_params,
                    support_layer,
                    support_layer_id,
                    support_interface_angle,
                    &angles,
                    link_max_length_factor,
                )?;
            }

            // Base interface layers under soluble interfaces
            if !layer_cache.layers[LC_BASE_INTERFACE].empty(layer_storage) && !layer_cache.layers[LC_BASE_INTERFACE].polygons_to_extrude(layer_storage).is_empty() {
                let base_interface_layer = &mut layer_cache.layers[LC_BASE_INTERFACE];
                let filler = fillers.filler_base_interface.as_mut().expect("filler_base_interface").as_mut();
                //FIXME Bottom interfaces are extruded with the briding flow. Some bridging layers have its height slightly reduced, therefore
                // the bridging flow does not quite apply. Reduce the flow to area of an ellipse? (A = pi * a * b)
                let layer = &layer_storage[base_interface_layer.layer.unwrap()];
                debug_assert!(!layer.bridging);
                let interface_flow: Flow = if support_layer.planned_nozzle_diameter > EPSILON {
                    support_params.support_material_flow_for_nozzle(support_layer.planned_nozzle_diameter, layer.height as f32)?
                } else {
                    support_params.support_material_flow.with_height(layer.height as f32 as f64)?
                };
                filler.fill_mut().angle = support_interface_angle;
                filler.fill_mut().spacing = interface_flow.spacing();
                // std::min(1., interface_flow.spacing() / (config.support_interface_spacing.value + interface_flow.spacing()))
                let density: f64 = {
                    let v = interface_flow.spacing() / (cfg_float(config, "support_interface_spacing") + interface_flow.spacing());
                    if v < 1. {
                        v
                    } else {
                        1.
                    }
                };
                filler.fill_mut().link_max_length = scale_(filler.fill().spacing * link_max_length_factor / density) as i32 as Coord;
                let regions = cu::union_safety_offset_ex(base_interface_layer.polygons_to_extrude(layer_storage));
                fill_expolygons_generate_paths(
                    // Destination
                    &mut base_interface_layer.extrusions,
                    // Regions to fill
                    regions,
                    // Filler and its parameters
                    filler,
                    density as f32,
                    // Extrusion parameters
                    ExtrusionRole::SupportTransition,
                    &interface_flow,
                )?;
            }

            // Base support or flange.
            if !layer_cache.layers[LC_BASE].empty(layer_storage) && !layer_cache.layers[LC_BASE].polygons_to_extrude(layer_storage).is_empty() {
                let base_layer_idx = layer_cache.layers[LC_BASE].layer.unwrap();
                let (base_height, base_bottom_z) = (layer_storage[base_layer_idx].height, layer_storage[base_layer_idx].bottom_z);
                // We don't use $base_flow->spacing because we need a constant spacing
                // value that guarantees that all layers are correctly aligned.
                debug_assert!(!layer_storage[base_layer_idx].bridging);
                // BBS multi-diameter: a merged slab is extruded by the filament the support merge plan reserved
                // for it, so its line width has to follow that nozzle instead of the support_filament default.
                let base_flow_src: Flow = if support_layer.planned_nozzle_diameter > EPSILON {
                    support_params.support_material_flow_for_nozzle(support_layer.planned_nozzle_diameter, support_params.support_material_flow.height() as f32)?
                } else {
                    support_params.support_material_flow
                };
                let mut flow = base_flow_src.with_height(base_height as f32 as f64)?;
                let mut use_first_layer_filler = false;
                {
                    let filler = fillers.filler_support.as_mut();
                    filler.fill_mut().angle = angles[support_layer_id % angles.len()];
                    filler.fill_mut().spacing = base_flow_src.spacing();
                    filler.fill_mut().link_max_length = scale_(filler.fill().spacing * link_max_length_factor / support_params.support_density) as i32 as Coord;
                }
                let mut density: f32 = support_params.support_density as f32;
                let mut sheath: bool = support_params.with_sheath;
                let mut no_sort: bool = false;
                let mut done: bool = false;
                if base_bottom_z < EPSILON {
                    // A merged base flange keeps the first-layer pattern and width, while its flow height
                    // spans the complete slab down to the bed.
                    use_first_layer_filler = true;
                    let filler = fillers.filler_first_layer();
                    // Geometry::deg2rad(float(config.support_angle.value + 90.))
                    filler.fill_mut().angle = deg2rad_f32((cfg_float(config, "support_angle") + 90.) as f32);
                    density = (cfg_float(config, "raft_first_layer_density") * 0.01) as f32;
                    flow = if support_layer.planned_nozzle_diameter > EPSILON {
                        support_params.support_material_first_layer_flow_for_nozzle(support_layer.planned_nozzle_diameter, base_height as f32)?
                    } else {
                        support_params.first_layer_flow.with_height(base_height as f32 as f64)?
                    };
                    // use the proper spacing for first layer as we don't need to align
                    // its pattern to the other layers
                    //FIXME When paralellizing, each thread shall have its own copy of the fillers.
                    filler.fill_mut().spacing = flow.spacing();
                    filler.fill_mut().link_max_length = scale_(filler.fill().spacing * link_max_length_factor / density as f64) as i32 as Coord;
                    sheath = true;
                    no_sort = true;
                } else if support_params.support_style == enums::SupportMaterialStyle::SmsTreeOrganic as i32 {
                    // if the tree supports are too tall, use double wall to make it stronger
                    let mut support_params2 = support_params.clone();
                    if support_layer_print_z > 100.0 {
                        support_params2.tree_branch_diameter_double_wall_area_scaled = 0.1;
                    }
                    {
                        let polygons = layer_cache.layers[LC_BASE].polygons_to_extrude(layer_storage).clone();
                        tree_supports_generate_paths(&mut layer_cache.layers[LC_BASE].extrusions, &polygons, &flow, &support_params2)?;
                    }
                    // Ground internal floating voids. The organic tree base above is printed sheath-only
                    // (hollow), so material printed over an internal hole gets no support. The polylines
                    // were already clipped to this layer's filled footprint minus the model by
                    // organic_lightning_infill. They are indexed like the compacted intermediate_layers
                    // (print_z), not like support_layers (which also holds raft and contact-only rows).
                    // Emit only for the intermediate already bound as this support layer's base.
                    if let Some(lightning_infill_lines) = lightning_infill_lines.as_mut() {
                        let idx = idx_layer_intermediate as usize;
                        if idx < lightning_infill_lines.len()
                            && idx < intermediate_layers.len()
                            && base_layer_idx == intermediate_layers[idx]
                            && !lightning_infill_lines[idx].is_empty()
                        {
                            // The lines were planned before these toolpaths existed. Keep them 0.85 w off the
                            // sheath centrelines (enough overlap left to bond) and off the contact / interface areas.
                            let lightning: &mut Polylines = &mut lightning_infill_lines[idx];
                            let bbox = get_extents_polylines_bbox(lightning).inflated(flow.scaled_width() as f64);
                            let mut sheath_lines: Polylines = Vec::new();
                            for entity in &layer_cache.layers[LC_BASE].extrusions {
                                extrusion_entity_collect_polylines(entity, &mut sheath_lines);
                            }
                            sheath_lines.retain(|pl| bbox.overlap(&BBox::from_points(&pl.points)));
                            // 外墙远长于闪电线，只偏移闪电线附近的一段外墙，结果不变。
                            let sheath_offset: f32 = 0.85f32 * flow.scaled_width() as f32;
                            // coord_t(1.5f * sheath_offset) + SCALED_EPSILON
                            keep_polylines_near_bbox(
                                &mut sheath_lines,
                                &get_extents_polylines_bbox(lightning).inflated((1.5f32 * sheath_offset) as i32 as f64 + scale_(EPSILON)).to_bounding_box(),
                            );
                            let mut blocked: Polygons = offset_polylines_or_none(&sheath_lines, sheath_offset);
                            for other in [LC_BOTTOM_CONTACT, LC_TOP_CONTACT, LC_INTERFACE, LC_BASE_INTERFACE] {
                                if !layer_cache.layers[other].empty(layer_storage) {
                                    blocked.extend(cu::offset(
                                        &clipper_utils::clip_clipper_polygons_with_subject_bbox_polygons(
                                            layer_cache.layers[other].polygons_to_extrude(layer_storage),
                                            &bbox.to_bounding_box(),
                                        ),
                                        0.5f32 * flow.scaled_width() as f32,
                                        cu::DEFAULT_JOIN_TYPE,
                                        cu::DEFAULT_MITER_LIMIT,
                                    ));
                                }
                            }
                            *lightning = cu::diff_pl(lightning, &blocked);
                            // pl.length() < scaled<double>(1.)
                            lightning.retain(|pl| !(pl.length() < scale_(1.)));
                            // float(flow.mm3_per_mm()), float(flow.width()), float(flow.height())
                            extrusion_entities_append_paths(
                                &mut layer_cache.layers[LC_BASE].extrusions,
                                std::mem::take(lightning),
                                ExtrusionRole::SupportMaterial,
                                flow.mm3_per_mm()? as f32 as f64,
                                flow.width() as f32,
                                flow.height() as f32,
                                true,
                            );
                        }
                    }
                    done = true;
                }
                if !done {
                    let polygons = layer_cache.layers[LC_BASE].polygons_to_extrude(layer_storage).clone();
                    let filler: &mut dyn SupportFill = if use_first_layer_filler { fillers.filler_first_layer() } else { fillers.filler_support.as_mut() };
                    fill_expolygons_with_sheath_generate_paths(
                        // Destination
                        &mut layer_cache.layers[LC_BASE].extrusions,
                        // Regions to fill
                        &polygons,
                        // Filler and its parameters
                        filler,
                        density,
                        // Extrusion parameters
                        ExtrusionRole::SupportMaterial,
                        &flow,
                        support_params,
                        sheath,
                        no_sort,
                    )?;
                }
            }

            if support_layer_id < cooldown_areas.len() && !cooldown_areas[support_layer_id].is_empty() {
                for item in [LC_BASE, LC_TOP_CONTACT, LC_BOTTOM_CONTACT, LC_INTERFACE, LC_BASE_INTERFACE] {
                    set_overhang_degree_impl(&mut layer_cache.layers[item].extrusions, &cooldown_areas[support_layer_id], throw_on_cancel);
                }
                if let Some(throw_on_cancel) = throw_on_cancel {
                    throw_on_cancel();
                }
            }

            // Merge base_interface_layers to base_layers to avoid unneccessary retractions
            {
                let (base_layer, base_interface_layer) = layer_cache_pair(&mut layer_cache.layers, LC_BASE, LC_BASE_INTERFACE);
                if !base_layer.empty(layer_storage)
                    && !base_interface_layer.empty(layer_storage)
                    && !base_layer.polygons_to_extrude(layer_storage).is_empty()
                    && !base_interface_layer.polygons_to_extrude(layer_storage).is_empty()
                    && base_layer.could_merge(base_interface_layer, layer_storage)
                {
                    base_layer.merge(base_interface_layer, layer_storage);
                }
            }

            layer_cache.add_nonempty_and_sort(layer_storage);

            // Collect the support areas with this print_z into islands, as there is no need
            // for retraction over these islands.
            let mut polys: Polygons = Vec::new();
            // Collect the extrusions, sorted by the bottom extrusion height.
            for i_item in 0..layer_cache.nonempty.len() {
                let layer_extruded = layer_cache.nonempty[i_item].layer_extruded;
                // Collect islands to polys.
                layer_cache.layers[layer_extruded].polygons_append(&mut polys, layer_storage);
                // The print_z of the top contact surfaces and bottom_z of the bottom contact surfaces are "free"
                // in a sense that they are not synchronized with other support layers. As the top and bottom contact surfaces
                // are inflated to achieve a better anchoring, it may happen, that these surfaces will at least partially
                // overlap in Z with another support layers, leading to over-extrusion.
                // Mitigate the over-extrusion by modulating the extrusion rate over these regions.
                // The print head will follow the same print_z, but the layer thickness will be reduced
                // where it overlaps with another support layer.
                //FIXME When printing a briging path, what is an equivalent height of the squished extrudate of the same width?
                // Collect overlapping top/bottom surfaces.
                let layer = &layer_storage[layer_cache.layers[layer_extruded].layer.unwrap()];
                let bottom_z: f64 = layer.bottom_print_z() + EPSILON;
                let overlapping = &mut layer_cache.nonempty[i_item].overlapping;
                let mut add_overlapping = |layers: &SupportGeneratorLayersPtr, idx_top: isize| {
                    let mut i = idx_top as i32 - 1;
                    while i >= 0 && layer_storage[layers[i as usize]].print_z > bottom_z {
                        overlapping.push(layers[i as usize]);
                        i -= 1;
                    }
                };
                add_overlapping(top_contacts, idx_layer_top_contact);
                if layer.layer_type == SupporLayerType::SltBottomContact {
                    // Bottom contact layer may overlap with a base layer, which may be changed to interface layer.
                    add_overlapping(intermediate_layers, idx_layer_intermediate);
                    add_overlapping(interface_layers, idx_layer_interface);
                    add_overlapping(base_interface_layers, idx_layer_base_interface);
                }
                // Order the layers by lexicographically by an increasing print_z and a decreasing layer height.
                // std::stable_sort(..., [](auto *l1, auto *l2) { return *l1 < *l2; })
                overlapping.sort_by(|&l1, &l2| {
                    if support_generator_layer_less(&layer_storage[l1], &layer_storage[l2]) {
                        std::cmp::Ordering::Less
                    } else if support_generator_layer_less(&layer_storage[l2], &layer_storage[l1]) {
                        std::cmp::Ordering::Greater
                    } else {
                        std::cmp::Ordering::Equal
                    }
                });
            }
            debug_assert!(support_layer.support_islands.is_empty());
            if !polys.is_empty() {
                support_layer.support_islands = cu::union_ex(&polys);
            }
        } // for each support_layer_id
    }

    // Now modulate the support layer height in parallel.
    for support_layer_id in n_raft_layers..support_layers.len() {
        if let Some(throw_on_cancel) = throw_on_cancel {
            throw_on_cancel();
        }
        let layer_cache = &mut layer_caches[support_layer_id];
        // For all extrusion types at this print_z, ordered by decreasing layer height:
        for i_item in 0..layer_cache.nonempty.len() {
            let layer_extruded = layer_cache.nonempty[i_item].layer_extruded;
            // Trim the extrusion height from the bottom by the overlapping layers.
            let this_layer = layer_cache.layers[layer_extruded].layer.unwrap();
            modulate_extrusion_by_overlapping_layers(&mut layer_cache.layers[layer_extruded].extrusions, this_layer, &layer_cache.nonempty[i_item].overlapping, layer_storage)?;
            // support_layer.support_fills.append(std::move(layer_cache_item.layer_extruded->extrusions))
            support_layers[support_layer_id].support_fills.entities.append(&mut layer_cache.layers[layer_extruded].extrusions);
        }

        // support interface ironing related generation from Orca: Generate iron toolpath for contact layer
        if !layer_cache.polys_to_iron.is_empty() && support_params.enable_support_ironing {
            let mut ironing_fill = fill_factory.new_from_type(support_params.ironing_pattern);
            {
                let support_layer = &support_layers[support_layer_id];
                let fill = ironing_fill.fill_mut();
                fill.bounding_box = bbox_object;
                fill.layer_id = support_layer.layer.id();
                fill.z = support_layer.layer.print_z;
                fill.overlap = 0.;
                // ironing_fill->angle = support_params.ironing_angle: the double to the float member
                fill.angle = support_params.ironing_angle as f32;
                fill.spacing = support_params.ironing_line_spacing;
                fill.link_max_length = scale_(3. * fill.spacing) as i32 as Coord;
            }

            let mut polys_to_iron: ExPolygons = cu::union_safety_offset_ex(&layer_cache.polys_to_iron);
            layer_cache.polys_to_iron.clear();

            // Find the layer above that directly overlaps current layer, clip the overlapped part
            if support_layer_id < support_layers.len() - 1 {
                let upper_layer = &support_layers[support_layer_id + 1];
                if !upper_layer.support_islands.is_empty() && upper_layer.layer.bottom_z() <= support_layers[support_layer_id].layer.print_z + EPSILON {
                    polys_to_iron = cu::diff_ex_ee(&polys_to_iron, &upper_layer.support_islands, No);
                }
            }
            if support_params.ironing_inset > EPSILON {
                // union_ex(offset(polys_to_iron, -scale_(support_params.ironing_inset)))
                polys_to_iron = cu::union_ex(&cu::offset_expolygons(&polys_to_iron, (-scale_(support_params.ironing_inset)) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
            }

            let support_layer = &mut support_layers[support_layer_id];
            let interface_height: f64 = if support_layer.interface_slab_height > EPSILON { support_layer.interface_slab_height } else { support_layer.layer.height };
            let interface_flow: Flow = if support_layer.planned_interface_nozzle_diameter > EPSILON {
                support_params.support_material_interface_flow_for_nozzle(support_layer.planned_interface_nozzle_diameter, interface_height as f32)?
            } else {
                support_params.support_material_interface_flow.with_height(interface_height as f32 as f64)?
            };
            // Flow support_ironing_flow(interface_flow.width(), interface_flow.height() * support_params.ironing_flow_percent * 0.01,
            //                           interface_flow.nozzle_diameter()): the height a double product, then a float
            let support_ironing_flow = Flow::new(
                interface_flow.width(),
                (interface_flow.height() as f32 as f64 * support_params.ironing_flow_percent * 0.01) as f32 as f64,
                interface_flow.nozzle_diameter(),
            )?;
            fill_expolygons_generate_paths(
                // Destination
                &mut support_layer.support_fills.entities,
                // Regions to fill
                polys_to_iron,
                // Filler and its parameters
                ironing_fill.as_mut(),
                1.,
                // Extrusion parameters
                ExtrusionRole::SupportIroning,
                &support_ironing_flow,
            )?;
        }
    }
    Ok(())
}

// ExtrusionEntity.hpp `virtual void collect_polylines(Polylines &dst) const`: ExtrusionPath (its polyline when
// not empty), ExtrusionMultiPath and ExtrusionLoop (their polyline as one), ExtrusionEntityCollection (its
// entities).
fn extrusion_entity_collect_polylines(entity: &ExtrusionEntityType, dst: &mut Polylines) {
    match entity {
        ExtrusionEntityType::Path(path) => {
            if !path.polyline.points.is_empty() {
                dst.push(path.polyline.clone());
            }
        }
        ExtrusionEntityType::MultiPath(multipath) => {
            let pl = multipath.as_polyline();
            if !pl.points.is_empty() {
                dst.push(pl);
            }
        }
        ExtrusionEntityType::Loop(r#loop) => {
            let pl = r#loop.as_polyline();
            if !pl.points.is_empty() {
                dst.push(pl);
            }
        }
        ExtrusionEntityType::Collection(collection) => {
            for child in &collection.entities {
                extrusion_entity_collect_polylines(child, dst);
            }
        }
    }
}

// ============================================================================
// SupportCommon.cpp:1495-2187 — BBS multi-diameter: merging the support layers into slabs.
// ============================================================================

use crate::support_merge_plan::{Candidate as SupportMergePlanCandidate, SupportMergePlan};
use ordered_float::OrderedFloat;
use std::collections::BTreeMap;

// SupportCommon.cpp:1495-1498
// BBS multi-diameter: the mergeable layer arrays, in the order merge_support_layers_multi_diameter() indexes them.
const SMA_BOTTOM_CONTACT: usize = 0;
const SMA_TOP_CONTACT: usize = 1;
const SMA_INTERMEDIATE: usize = 2;
const SMA_INTERFACE: usize = 3;
const SMA_BASE_INTERFACE: usize = 4;
const SMA_COUNT: usize = 5;
// Everything but the sparse base faces the object and follows the interface rule.
const SUPPORT_MERGE_ARRAY_IS_INTERFACE: [bool; SMA_COUNT] = [true, true, false, true, true];
// SupportCommon.cpp:1697 `top_interface_arrays`
const TOP_INTERFACE_ARRAYS: [usize; 2] = [SMA_TOP_CONTACT, SMA_INTERFACE];
// SupportCommon.cpp:2027 `rescued_arrays`
const RESCUED_ARRAYS: [usize; 3] = [SMA_TOP_CONTACT, SMA_INTERFACE, SMA_BASE_INTERFACE];

// SupportCommon.cpp:1502-1514
// One print_z carrying support areas, together with the layer of every array that shares that print_z.
// Ordering the levels by print_z reproduces the support layer ids generate_support_layers() will hand out,
// because it installs exactly one support layer per non-empty print_z.
#[derive(Debug, Clone, Copy)]
struct SupportMergeLevel {
    print_z: f64,
    // Below the raft contact / the first object layer, where the adhesion geometry must not be touched.
    frozen: bool,
    // A raft-only level has no entry in arrays[], but still installs a support layer.
    has_raft: bool,
    // Index into the corresponding array, or -1.
    layer_idx: [i32; SMA_COUNT],
}

// The state the lambdas of merge_support_layers_multi_diameter() share.
struct SupportMergeState<'a> {
    arrays: [&'a mut SupportGeneratorLayersPtr; SMA_COUNT],
    levels: Vec<SupportMergeLevel>,
    layer_storage: &'a mut SupportGeneratorLayerStorage,
    plan: &'a SupportMergePlan,
    // const ConstLayerPtrsAdaptor object_layers = object.layers();
    object_layers: &'a [crate::layer::Layer],
    gap_xy_scaled: i32,
    top_gap: f64,
    // The object slices grown by the XY gap, per object layer. Every slab asks for the same layers again.
    object_keep_out_cache: Vec<Option<Polygons>>,
    band_keep_out_cache: Vec<Option<Polygons>>,
    interface_borrowed_areas: Vec<Polygons>,
}

impl SupportMergeState<'_> {
    // SupportCommon.cpp:1631-1634, the lambda `layer_of`.
    fn layer_of(&self, level: usize, array_id: usize) -> Option<usize> {
        let idx = self.levels[level].layer_idx[array_id];
        if idx < 0 {
            None
        } else {
            let layer = self.arrays[array_id][idx as usize];
            if layer == NULL_LAYER {
                None
            } else {
                Some(layer)
            }
        }
    }

    // `layer != nullptr && ! layer->polygons.empty()`
    fn nonempty_layer_of(&self, level: usize, array_id: usize) -> Option<usize> {
        self.layer_of(level, array_id).filter(|&layer| !self.layer_storage[layer].polygons.is_empty())
    }

    // SupportCommon.cpp:1639-1644, the lambda `object_keep_out`.
    fn object_keep_out(&mut self, object_layer_idx: usize) -> Polygons {
        if self.object_keep_out_cache[object_layer_idx].is_none() {
            self.object_keep_out_cache[object_layer_idx] =
                Some(cu::offset_expolygons(&self.object_layers[object_layer_idx].lslices, self.gap_xy_scaled as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
        }
        self.object_keep_out_cache[object_layer_idx].clone().unwrap()
    }

    // SupportCommon.cpp:1646-1653, the lambda `band_keep_out`.
    fn band_keep_out(&mut self, object_layer_idx: usize) -> Polygons {
        if self.band_keep_out_cache[object_layer_idx].is_none() {
            let band: Polygons = merged_band_footprint(&self.object_layers[object_layer_idx], object_layer_idx);
            self.band_keep_out_cache[object_layer_idx] = Some(if band.is_empty() {
                Vec::new()
            } else {
                cu::offset(&cu::union_(&band), self.gap_xy_scaled as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
            });
        }
        self.band_keep_out_cache[object_layer_idx].clone().unwrap()
    }

    // SupportCommon.cpp:1654-1657, the lambda `first_object_layer_above`.
    fn first_object_layer_above(&self, z: f64) -> usize {
        self.object_layers.partition_point(|layer| layer.print_z < z + EPSILON)
    }

    // SupportCommon.cpp:1662-1671, the lambda `object_clearance`.
    // A support area is only kept clear of the object at its own level. A slab is a single extrusion over
    // its whole Z span, so it covers the XY the object occupies further down as well - a shape widening
    // towards the bed is the canonical case - and has to be cut back to the same clearance there.
    fn object_clearance(&mut self, bottom_z: f64, top_z: f64) -> Polygons {
        let mut trimming: Polygons = Vec::new();
        for idx in self.first_object_layer_above(bottom_z)..self.object_layers.len() {
            let object_layer = &self.object_layers[idx];
            if object_layer.print_z - object_layer.height > top_z - EPSILON {
                break;
            }
            let keep_out = self.object_keep_out(idx);
            trimming.extend(keep_out);
        }
        if trimming.is_empty() {
            Vec::new()
        } else {
            cu::union_(&trimming)
        }
    }

    // SupportCommon.cpp:1675-1690, the lambda `top_gap_zone`.
    // The XY of the object starting within the top gap above top_z. An area lifted to top_z from below was
    // placed against an object surface higher up; where that surface starts in here, lifting it eats into
    // the gap, down to none when the lift equals the gap. object_clearance() only covers what starts lower.
    fn top_gap_zone(&mut self, top_z: f64) -> Polygons {
        let mut zone: Polygons = Vec::new();
        if self.top_gap <= EPSILON {
            return zone;
        }
        for idx in self.first_object_layer_above(top_z)..self.object_layers.len() {
            let object_layer = &self.object_layers[idx];
            let layer_bottom_z: f64 = object_layer.print_z - object_layer.height;
            if layer_bottom_z > top_z + self.top_gap - EPSILON {
                break;
            }
            if layer_bottom_z < top_z - EPSILON {
                continue;
            }
            let keep_out = self.object_keep_out(idx);
            zone.extend(keep_out);
            let keep_out = self.band_keep_out(idx);
            zone.extend(keep_out);
        }
        if zone.is_empty() {
            Vec::new()
        } else {
            cu::union_(&zone)
        }
    }

    // SupportCommon.cpp:1698-1703, the lambda `first_top_interface`.
    fn first_top_interface(&self, level: usize) -> Option<usize> {
        for array_id in TOP_INTERFACE_ARRAYS {
            if let Some(layer) = self.nonempty_layer_of(level, array_id) {
                return Some(layer);
            }
        }
        None
    }

    // SupportCommon.cpp:1704-1714, the lambda `interface_height_range`.
    fn interface_height_range(&self, level: usize) -> (f64, f64) {
        let mut range: (f64, f64) = (f64::MAX, 0.);
        for array_id in TOP_INTERFACE_ARRAYS {
            if let Some(layer) = self.nonempty_layer_of(level, array_id) {
                let height = self.layer_storage[layer].height;
                // std::min(range.first, layer->height), std::max(range.second, layer->height)
                if height < range.0 {
                    range.0 = height;
                }
                if range.1 < height {
                    range.1 = height;
                }
            }
        }
        range
    }

    // SupportCommon.cpp:1715-1722, the lambda `candidate_fits_bottom_contact`.
    fn candidate_fits_bottom_contact(&self, level: usize, candidate: &SupportMergePlanCandidate) -> bool {
        match self.layer_of(level, SMA_BOTTOM_CONTACT) {
            None => true,
            Some(bottom_contact) => {
                let bottom_contact = &self.layer_storage[bottom_contact];
                bottom_contact.polygons.is_empty() || candidate.window.contains(bottom_contact.height)
            }
        }
    }

    // SupportCommon.cpp:1723-1738, the lambda `level_bottom_z`.
    fn level_bottom_z(&self, level: usize) -> f64 {
        let mut bottom_z: f64 = self.levels[level].print_z;
        let mut found = false;
        for array_id in TOP_INTERFACE_ARRAYS {
            if let Some(layer) = self.nonempty_layer_of(level, array_id) {
                let layer = &self.layer_storage[layer];
                // std::min(bottom_z, layer->print_z - layer->height)
                let v = layer.print_z - layer.height;
                if v < bottom_z {
                    bottom_z = v;
                }
                found = true;
            }
        }
        if let Some(base) = self.nonempty_layer_of(level, SMA_INTERMEDIATE) {
            let base = &self.layer_storage[base];
            let v = base.print_z - base.height;
            if v < bottom_z {
                bottom_z = v;
            }
            found = true;
        }
        if found {
            bottom_z
        } else {
            self.levels[level].print_z
        }
    }

    // SupportCommon.cpp:1741-1747, the lambda `interface_candidate`.
    // Keep interface candidate validation in one place. Interface and sparse base may use different layer
    // ladders; their overlap is resolved spatially while the sparse base slabs are built below.
    fn interface_candidate(&self, level: usize, top_z: f64, height: f64) -> SupportMergePlanCandidate {
        let mut candidate = self.plan.finest_interface_for_slab(top_z, height);
        if candidate.valid() && !self.candidate_fits_bottom_contact(level, &candidate) {
            candidate = SupportMergePlanCandidate::default();
        }
        candidate
    }

    // `(*arrays[array_id])[levels[i].layer_idx[array_id]] = nullptr; levels[i].layer_idx[array_id] = -1;`
    fn drop_layer(&mut self, level: usize, array_id: usize) {
        let idx = self.levels[level].layer_idx[array_id] as usize;
        self.arrays[array_id][idx] = NULL_LAYER;
        self.levels[level].layer_idx[array_id] = -1;
    }

    // SupportCommon.cpp:1751-1835, the lambda `merge_interface_slab`.
    // Returns false when the slab was left alone, in which case the areas keep their own levels.
    fn merge_interface_slab(
        &mut self,
        owner_idx: i32,
        bottom_idx: i32,
        candidate: &SupportMergePlanCandidate,
        planned_interface_filaments: &mut BTreeMap<OrderedFloat<f64>, SupportMergePlanCandidate>,
        planned_interface_heights: &mut BTreeMap<OrderedFloat<f64>, f64>,
    ) -> bool {
        use cu::ApplySafetyOffset::No;
        let Some(interface_top) = self.first_top_interface(owner_idx as usize) else {
            return false;
        };

        let top_z: f64 = self.levels[owner_idx as usize].print_z;
        let bottom_z: f64 = self.level_bottom_z(bottom_idx as usize);
        let slab_height: f64 = top_z - bottom_z;
        // What a lower level holds under an object surface starting within the gap above top_z stays at its
        // own level: lifted, it would print into that gap.
        let gap_zone: Polygons = self.top_gap_zone(top_z);
        // Refuse a slab the object leaves no room for at all rather than merge the areas away: printing
        // them at their own level is unprintably thin, but dropping them removes the support entirely.
        let clearance: Polygons = self.object_clearance(bottom_z, top_z);
        if !clearance.is_empty() {
            let mut slab: Polygons = Vec::new();
            for i in (bottom_idx..=owner_idx).rev() {
                for array_id in TOP_INTERFACE_ARRAYS {
                    if let Some(source) = self.layer_of(i as usize, array_id) {
                        let source = &self.layer_storage[source];
                        if i < owner_idx && !gap_zone.is_empty() {
                            slab.extend(cu::diff(&source.polygons, &gap_zone, No));
                        } else {
                            slab.extend(source.polygons.iter().cloned());
                        }
                    }
                }
            }
            if cu::diff(&cu::union_(&slab), &clearance, No).is_empty() {
                return false;
            }
        }

        let mut kept_sources: Vec<(i32, usize)> = Vec::new();
        for i in (bottom_idx..=owner_idx).rev() {
            for array_id in TOP_INTERFACE_ARRAYS {
                let Some(source) = self.layer_of(i as usize, array_id) else {
                    continue;
                };
                if source == interface_top {
                    continue;
                }
                if i < owner_idx && !gap_zone.is_empty() {
                    let kept: Polygons = cu::intersection(&self.layer_storage[source].polygons, &gap_zone, No);
                    if !kept.is_empty() {
                        let merged = cu::union_2(&self.layer_storage[interface_top].polygons, &cu::diff(&self.layer_storage[source].polygons, &gap_zone, No));
                        self.layer_storage[interface_top].polygons = merged;
                        self.layer_storage[source].polygons = kept;
                        kept_sources.push((i, array_id));
                        continue;
                    }
                }
                support_generator_layer_merge(self.layer_storage, interface_top, source);
                self.drop_layer(i as usize, array_id);
            }
        }

        self.layer_storage[interface_top].print_z = top_z;
        self.layer_storage[interface_top].height = slab_height;
        self.layer_storage[interface_top].bottom_z = bottom_z;
        if !clearance.is_empty() {
            let trimmed = cu::diff(&self.layer_storage[interface_top].polygons, &clearance, No);
            self.layer_storage[interface_top].polygons = trimmed;
        }

        let borrowed_area: Polygons = cu::union_(&self.layer_storage[interface_top].polygons);
        // The slab now spans the levels its kept areas stay on, so they must not overlap it.
        for &(i, array_id) in &kept_sources {
            let source = self.layer_of(i as usize, array_id).expect("a kept source");
            let trimmed = cu::diff(&self.layer_storage[source].polygons, &borrowed_area, No);
            self.layer_storage[source].polygons = trimmed;
            if self.layer_storage[source].polygons.is_empty() {
                self.drop_layer(i as usize, array_id);
            }
        }
        for i in bottom_idx..=owner_idx {
            self.interface_borrowed_areas[i as usize].extend(borrowed_area.iter().cloned());
            if let Some(base) = self.nonempty_layer_of(i as usize, SMA_INTERMEDIATE) {
                let trimmed = cu::diff(&self.layer_storage[base].polygons, &borrowed_area, No);
                self.layer_storage[base].polygons = trimmed;
            }

            let level_print_z = self.levels[i as usize].print_z;
            map_erase_near(planned_interface_filaments, level_print_z);
            map_erase_near(planned_interface_heights, level_print_z);
        }

        planned_interface_filaments.insert(OrderedFloat(top_z), *candidate);
        planned_interface_heights.insert(OrderedFloat(top_z), slab_height);
        true
    }

    // SupportCommon.cpp:2127-2134, the lambda `level_has_areas`.
    fn level_has_areas(&self, level: usize) -> bool {
        if self.levels[level].has_raft {
            return true;
        }
        for array_id in 0..SMA_COUNT {
            if self.nonempty_layer_of(level, array_id).is_some() {
                return true;
            }
        }
        false
    }
}

// `auto it = map.lower_bound(print_z - EPSILON); if (it != map.end() && it->first <= print_z + EPSILON) map.erase(it);`
fn map_erase_near<T>(map: &mut BTreeMap<OrderedFloat<f64>, T>, print_z: f64) {
    let key = map.range(OrderedFloat(print_z - EPSILON)..).next().map(|(k, _)| *k);
    if let Some(key) = key {
        if key.0 <= print_z + EPSILON {
            map.remove(&key);
        }
    }
}

// SupportCommon.cpp:1528-2187
//
// `plan` is `make_support_merge_plan(object)` (SupportCommon.cpp:1547).
//
// FIDELITY-NOTE: the crate's `make_support_merge_plan` takes what it reads of the object as a record
// (support_merge_plan.rs), not the PrintObject, so the plan is handed in instead of being made here.
#[allow(clippy::too_many_arguments)]
pub fn merge_support_layers_multi_diameter(
    object: &pf::PrintObject,
    plan: &SupportMergePlan,
    support_params: &SupportParameters,
    slicing_params: &SlicingParameters,
    raft_layers: &SupportGeneratorLayersPtr,
    bottom_contacts: &mut SupportGeneratorLayersPtr,
    top_contacts: &mut SupportGeneratorLayersPtr,
    intermediate_layers: &mut SupportGeneratorLayersPtr,
    interface_layers: &mut SupportGeneratorLayersPtr,
    base_interface_layers: &mut SupportGeneratorLayersPtr,
    planned_filaments: &mut BTreeMap<OrderedFloat<f64>, SupportMergePlanCandidate>,
    planned_interface_filaments: &mut BTreeMap<OrderedFloat<f64>, SupportMergePlanCandidate>,
    planned_base_heights: &mut BTreeMap<OrderedFloat<f64>, f64>,
    planned_interface_heights: &mut BTreeMap<OrderedFloat<f64>, f64>,
    per_layer_area_print_z: Option<&Vec<f64>>,
    cooldown_areas: Option<&mut Vec<ExPolygons>>,
    lightning_infill_areas: Option<&mut Vec<ExPolygons>>,
    mut top_gap: f64,
    layer_storage: &mut SupportGeneratorLayerStorage,
) {
    use cu::ApplySafetyOffset::No;
    if top_gap < 0. {
        top_gap = slicing_params.gap_support_object;
    }

    // Keep raft and bridging-contact geometry fixed, but let a support first layer join the first band.
    // Its merged extrusion still reaches the bed (or the raft) and retains the first-layer footprint.
    let z_frozen_max: f64 = slicing_params.raft_contact_top_z + EPSILON;

    let object_layers = object.layers();
    let mut st = SupportMergeState {
        arrays: [bottom_contacts, top_contacts, intermediate_layers, interface_layers, base_interface_layers],
        levels: Vec::new(),
        layer_storage,
        plan,
        object_layers,
        // scaled<coord_t>(support_params.gap_xy)
        gap_xy_scaled: scale_(support_params.gap_xy) as i32,
        top_gap,
        object_keep_out_cache: vec![None; object_layers.len()],
        band_keep_out_cache: vec![None; object_layers.len()],
        interface_borrowed_areas: Vec::new(),
    };

    // Collect the print_z levels that carry support areas. The raft is only listed so the level indices match
    // the support layer ids; its layers are never merged.
    // print_z -> levels 下标，避免逐个扫描；命中多个时取最早创建的，与原线性查找一致。
    // (std::multimap<coordf_t, size_t>: the entries of one key in the order they were put in.)
    let mut level_index: BTreeMap<OrderedFloat<f64>, Vec<usize>> = BTreeMap::new();
    fn level_at(levels: &mut Vec<SupportMergeLevel>, level_index: &mut BTreeMap<OrderedFloat<f64>, Vec<usize>>, print_z: f64) -> usize {
        let mut found: usize = usize::MAX;
        // for (auto it = level_index.upper_bound(print_z - EPSILON); it != level_index.end() && it->first < print_z + EPSILON; ++ it)
        for (key, indices) in level_index.range((std::ops::Bound::Excluded(OrderedFloat(print_z - EPSILON)), std::ops::Bound::Unbounded)) {
            if !(key.0 < print_z + EPSILON) {
                break;
            }
            if (key.0 - print_z).abs() < EPSILON {
                for &second in indices {
                    found = std::cmp::min(found, second);
                }
            }
        }
        if found == usize::MAX {
            found = levels.len();
            levels.push(SupportMergeLevel { print_z, frozen: false, has_raft: false, layer_idx: [-1; SMA_COUNT] });
            level_index.entry(OrderedFloat(print_z)).or_default().push(found);
        }
        found
    }

    for &layer in raft_layers.iter() {
        if layer != NULL_LAYER && !st.layer_storage[layer].polygons.is_empty() {
            let level = level_at(&mut st.levels, &mut level_index, st.layer_storage[layer].print_z);
            st.levels[level].frozen = true;
            st.levels[level].has_raft = true;
        }
    }

    for array_id in 0..SMA_COUNT {
        for i in 0..st.arrays[array_id].len() {
            let layer = st.arrays[array_id][i];
            if layer == NULL_LAYER || st.layer_storage[layer].polygons.is_empty() {
                continue;
            }
            let (print_z, bridging) = (st.layer_storage[layer].print_z, st.layer_storage[layer].bridging);
            let level = level_at(&mut st.levels, &mut level_index, print_z);
            // A bridging layer hangs at a fixed gap above the object, so print_z - height is where its
            // extrusion starts rather than where the layer starts. Growing it would close that gap.
            st.levels[level].frozen |= print_z < z_frozen_max || bridging;
            if st.levels[level].layer_idx[array_id] >= 0 {
                if !plan.enabled {
                    continue;
                }
                let kept = st.arrays[array_id][st.levels[level].layer_idx[array_id] as usize];
                // generate_interface_layers() reuses the organic top interface layer object as the layer it
                // writes into (SupportGeneratorLayer &layer_new = top_interface_layer ? *top_interface_layer
                // : ...), so interface_layers and top_interface_layers end up holding the same pointer and
                // std::merge() lists it twice. Merging a layer into itself ends in rhs.reset() and wipes its
                // polygons, which is the whole interface stack. Drop the duplicate slot instead.
                if kept == layer {
                    st.arrays[array_id][i] = NULL_LAYER;
                    continue;
                }
                // Downstream the arrays are looked up by print_z, so two areas of the same kind at the same
                // height cannot be told apart. Fold them together right away.
                support_generator_layer_merge(st.layer_storage, kept, layer);
                st.arrays[array_id][i] = NULL_LAYER;
                continue;
            }
            st.levels[level].layer_idx[array_id] = i as i32;
        }
    }

    crate::clipper::libcxx_sort(&mut st.levels, |lhs, rhs| lhs.print_z < rhs.print_z);
    if !plan.enabled {
        for level in st.levels.iter_mut() {
            level.frozen = true;
        }
    }

    let num_levels = st.levels.len();
    st.interface_borrowed_areas = vec![Vec::new(); num_levels];

    // Resolve the top contact stack from the model downwards. A thin level stays untouched when that does not
    // strand an unprintable contact directly below it. Otherwise adjacent contacts and, if necessary, sparse
    // base are folded into a continuous slab printable by the largest available interface nozzle.
    // Base-interface is a support-transition region printed with the support-body extruder, not with the
    // contact extruder. Keep it out of contact slabs so nozzle planning and extrusion roles stay consistent.
    let mut top: i32 = num_levels as i32 - 1;
    while top >= 0 {
        'body: {
            let t = top as usize;
            if st.first_top_interface(t).is_none() || st.levels[t].frozen {
                break 'body;
            }

            let top_z: f64 = st.levels[t].print_z;
            let height_range: (f64, f64) = st.interface_height_range(t);
            let mut candidate = st.interface_candidate(t, top_z, height_range.0);
            if candidate.valid() && !candidate.window.contains(height_range.1) {
                candidate = SupportMergePlanCandidate::default();
            }
            if candidate.valid() {
                planned_interface_filaments.insert(OrderedFloat(top_z), candidate);
                break 'body;
            }

            let mut bottom: i32 = top;
            let mut bottom_z: f64 = st.level_bottom_z(t);
            let mut slab_height: f64 = top_z - bottom_z;
            candidate = st.interface_candidate(t, top_z, slab_height);
            while !candidate.valid() && bottom > 0 {
                let below = (bottom - 1) as usize;
                if st.levels[below].frozen || st.levels[below].print_z < bottom_z - EPSILON {
                    break;
                }
                let has_interface = st.first_top_interface(below).is_some();
                if !has_interface && st.nonempty_layer_of(below, SMA_INTERMEDIATE).is_none() {
                    break;
                }
                // Stop once the slab outgrows the tallest band any interface nozzle ends here with, instead of
                // walking the whole stack down for a slab that can no longer match.
                let grown_height: f64 = top_z - st.level_bottom_z(below);
                if grown_height > plan.max_interface_height(top_z, grown_height) + EPSILON {
                    break;
                }
                bottom -= 1;
                bottom_z = st.level_bottom_z(bottom as usize);
                slab_height = top_z - bottom_z;
                candidate = st.interface_candidate(t, top_z, slab_height);
            }
            if candidate.valid() && st.merge_interface_slab(top, bottom, &candidate, planned_interface_filaments, planned_interface_heights) {
                // Resume at the highest level still holding an area kept out of the top gap, so it gets planned
                // at its own height or merged with what lies below it.
                let mut resume: i32 = bottom;
                let mut i = top - 1;
                while i >= bottom {
                    if st.first_top_interface(i as usize).is_some() {
                        resume = i + 1;
                        break;
                    }
                    i -= 1;
                }
                top = resume;
                break 'body;
            }

            // A printable upper contact may still absorb this thin level. This closes a gap such as a printable
            // 0.2 mm layer above an unprintable 0.1 mm layer by turning both into one continuous 0.3 mm slab.
            let mut upper: i32 = top + 1;
            while upper < num_levels as i32 && st.first_top_interface(upper as usize).is_none() {
                upper += 1;
            }
            if upper < num_levels as i32 && !st.levels[upper as usize].frozen {
                let upper_interface = &st.layer_storage[st.first_top_interface(upper as usize).unwrap()];
                let upper_bottom_z: f64 = upper_interface.print_z - upper_interface.height;
                if upper_bottom_z <= top_z + EPSILON {
                    let combined_height: f64 = st.levels[upper as usize].print_z - st.level_bottom_z(t);
                    candidate = st.interface_candidate(upper as usize, st.levels[upper as usize].print_z, combined_height);
                    if candidate.valid() {
                        st.merge_interface_slab(upper, top, &candidate, planned_interface_filaments, planned_interface_heights);
                    }
                }
            }
        }
        top -= 1;
    }

    // Only the sparse base is merged. An interface area is extruded straight against the object, so it keeps
    // its own print_z and its own height and is cut out of the slab growing past it. Base and interface can
    // differ in height at the same print_z because generate_support_toolpaths() builds their flows from the
    // height of their own SupportGeneratorLayer, not from the height of the installed SupportLayer.
    //
    // slab_top_of_level[i] is the level that swallowed level i's base, so the per support layer areas of a
    // level the merge emptied can follow it.
    let mut slab_top_of_level: Vec<i32> = (0..num_levels as i32).collect();

    let mut top: i32 = num_levels as i32 - 1;
    while top >= 0 {
        'body: {
            let t = top as usize;
            let Some(base_top) = st.nonempty_layer_of(t, SMA_INTERMEDIATE) else {
                break 'body;
            };
            if st.levels[t].frozen {
                break 'body;
            }

            let slab_top_z: f64 = st.levels[t].print_z;
            let mut bottom: i32 = top;
            let mut slab_height: f64 = st.layer_storage[base_top].height;
            let mut pinned = plan.coarsest_for_slab(slab_top_z, slab_height);
            let mut probe: i32 = top;
            let mut probe_height: f64 = slab_height;

            // Probe down to the next fixed-grid boundary. Intermediate heights may be unprintable by every
            // candidate (for example 0.2 and 0.3 on a 0.8 / 0.1 ladder) but still lead to its 0.4 band.
            while probe > 0 && !st.levels[(probe - 1) as usize].frozen {
                let Some(base_below) = st.nonempty_layer_of((probe - 1) as usize, SMA_INTERMEDIATE) else {
                    break;
                };
                let base_below = &st.layer_storage[base_below];
                // levels only holds the heights that carry support areas, so the level below may sit under a Z
                // gap. The slab is a single extrusion and would hang over that void, so it stops here instead.
                if base_below.print_z < slab_top_z - probe_height - EPSILON {
                    break;
                }
                let height: f64 = slab_top_z - (base_below.print_z - base_below.height);
                if height > plan.max_height_for_slab(slab_top_z, height) + EPSILON {
                    break;
                }
                let candidate = plan.coarsest_for_slab(slab_top_z, height);
                probe -= 1;
                probe_height = height;
                if candidate.valid() {
                    bottom = probe;
                    slab_height = height;
                    pinned = candidate;
                }
            }

            if bottom < top {
                let includes_bed: bool = st.level_bottom_z(bottom as usize) <= EPSILON;
                let representative = st.layer_of(if includes_bed { bottom as usize } else { t }, SMA_INTERMEDIATE).expect("representative");
                // A merged extrusion cannot follow every fine-layer contour through Z. Keep the band face used
                // by the model ladder instead: top for an ordinary band, bottom for the first bed-adhesion band.
                // Unioning all contours creates a swept wedge when a tree branch moves laterally between layers.
                let mut representative_polygons: Polygons = st.layer_storage[representative].polygons.clone();

                // Interface extrusions may be thinner than this sparse-base slab while sharing its print_z.
                // Removing their footprint from the complete slab would open a column down to slab_bottom_z.
                // Keep the original fine base layers in that footprint instead, and merge only the remainder.
                let mut interface_columns: Polygons = Vec::new();
                for i in bottom..=top {
                    for array_id in 0..SMA_COUNT {
                        if SUPPORT_MERGE_ARRAY_IS_INTERFACE[array_id] {
                            if let Some(layer) = st.nonempty_layer_of(i as usize, array_id) {
                                interface_columns.extend(st.layer_storage[layer].polygons.iter().cloned());
                            }
                        }
                    }
                }
                interface_columns = cu::union_(&interface_columns);
                if !interface_columns.is_empty() {
                    representative_polygons = cu::diff(&representative_polygons, &interface_columns, No);
                }

                let mut i = top - 1;
                while i >= bottom {
                    let idx = st.levels[i as usize].layer_idx[SMA_INTERMEDIATE] as usize;
                    let base = st.arrays[SMA_INTERMEDIATE][idx];
                    let base_print_z = st.layer_storage[base].print_z;
                    let mut landing_columns: Polygons = Vec::new();
                    for j in bottom..=top {
                        for array_id in 0..SMA_COUNT {
                            if SUPPORT_MERGE_ARRAY_IS_INTERFACE[array_id] {
                                if let Some(layer) = st.nonempty_layer_of(j as usize, array_id) {
                                    let layer = &st.layer_storage[layer];
                                    if layer.print_z - layer.height >= base_print_z - EPSILON {
                                        landing_columns.extend(layer.polygons.iter().cloned());
                                    }
                                }
                            }
                        }
                    }
                    let preserved: Polygons =
                        if landing_columns.is_empty() { Vec::new() } else { cu::intersection(&st.layer_storage[base].polygons, &cu::union_(&landing_columns), No) };
                    if preserved.is_empty() {
                        st.arrays[SMA_INTERMEDIATE][idx] = NULL_LAYER;
                        st.levels[i as usize].layer_idx[SMA_INTERMEDIATE] = -1;
                        slab_top_of_level[i as usize] = top;
                    } else {
                        st.layer_storage[base].polygons = preserved;
                    }
                    i -= 1;
                }
                st.layer_storage[base_top].polygons = representative_polygons;
                // Levels holding a bridging layer are frozen, so a slab never grows over one of them.
                debug_assert!(!st.layer_storage[base_top].bridging);
                st.layer_storage[base_top].print_z = slab_top_z;
                st.layer_storage[base_top].height = slab_height;
                st.layer_storage[base_top].bottom_z = slab_top_z - slab_height;

                // The merged part of the base fills the whole slab. The transition columns were already removed
                // above, while the object still has to be kept clear over the complete slab height.
                let trimming: Polygons = st.object_clearance(st.layer_storage[base_top].bottom_z, slab_top_z);
                if !trimming.is_empty() && !st.layer_storage[base_top].polygons.is_empty() {
                    let trimmed = cu::diff(&st.layer_storage[base_top].polygons, &cu::union_(&trimming), No);
                    st.layer_storage[base_top].polygons = trimmed;
                }
                if !st.layer_storage[base_top].polygons.is_empty() {
                    planned_base_heights.insert(OrderedFloat(slab_top_z), slab_height);
                }
            }

            // Pin the slab to its filament only once it got thicker than the object layers: a slab of base height
            // can be extruded by any nozzle, and leaving it alone lets the exporter minimize tool changes.
            if pinned.valid() && slab_height > plan.base_height_at(slab_top_z) + EPSILON {
                planned_filaments.insert(OrderedFloat(slab_top_z), pinned);
            }

            // The for loop's decrement then moves below the slab.
            top = bottom;
        }
        top -= 1;
    }

    // A merged base slab is one extrusion, so cutting an interface column out of it drops the support under
    // that column over the slab's whole Z span. Grow the interface down to where support actually resumes
    // rather than leave it hanging above the void the cut created.
    for i in (0..num_levels).rev() {
        if st.levels[i].frozen {
            continue;
        }
        for rescued_id in RESCUED_ARRAYS {
            let Some(interface_top) = st.nonempty_layer_of(i, rescued_id) else {
                continue;
            };

            // A support transition area is extruded with the support body filament, not with the contact
            // filament, so its reach and its nozzle come from the base plan.
            let with_base_filament: bool = rescued_id == SMA_BASE_INTERFACE;
            let (interface_print_z, interface_height) = (st.layer_storage[interface_top].print_z, st.layer_storage[interface_top].height);
            let bottom_z: f64 = interface_print_z - interface_height;
            // No filament could extrude a taller slab, so nothing below that reach can be a landing.
            let max_reach: f64 = if with_base_filament {
                plan.max_height_for_slab(interface_print_z, interface_height)
            } else {
                plan.max_interface_height(interface_print_z, interface_height)
            };
            if max_reach <= interface_height + EPSILON {
                continue;
            }

            // Land on the highest support surface the column reaches, so the grown slab never runs into
            // material another column of the same interface still rests on.
            let mut landing_z: f64 = 0.;
            let mut landed = false;
            let mut blocked = false;
            let mut j: i32 = i as i32 - 1;
            while j >= 0 && !blocked {
                if interface_print_z - st.levels[j as usize].print_z > max_reach + EPSILON {
                    break;
                }
                for array_id in 0..SMA_COUNT {
                    let Some(below) = st.nonempty_layer_of(j as usize, array_id) else {
                        continue;
                    };
                    if cu::intersection(&st.layer_storage[below].polygons, &st.layer_storage[interface_top].polygons, No).is_empty() {
                        continue;
                    }
                    let below_print_z = st.layer_storage[below].print_z;
                    if below_print_z > bottom_z + EPSILON {
                        // Support already reaches into the range the slab would grow through.
                        blocked = true;
                        break;
                    }
                    // std::max(landing_z, below->print_z)
                    if landing_z < below_print_z {
                        landing_z = below_print_z;
                    }
                    landed = true;
                }
                j -= 1;
            }
            if blocked || !landed || landing_z > bottom_z - EPSILON {
                continue;
            }

            // The base slab merge keeps its distance to the object, and so must an interface growing down.
            let mut hits_object = false;
            let mut idx = st.first_object_layer_above(landing_z);
            while idx < st.object_layers.len() && !hits_object {
                let object_layer = &st.object_layers[idx];
                if object_layer.print_z - object_layer.height > bottom_z - EPSILON {
                    break;
                }
                let keep_out = st.object_keep_out(idx);
                hits_object = !cu::intersection(&keep_out, &st.layer_storage[interface_top].polygons, No).is_empty();
                idx += 1;
            }
            if hits_object {
                continue;
            }

            let height: f64 = interface_print_z - landing_z;
            let mut candidate = SupportMergePlanCandidate::default();
            if with_base_filament {
                // When the base slab merge already pinned this height, that is the filament which will
                // extrude the transition area. Otherwise pick one, which also pins the sparse base of this
                // level and therefore has to be able to print it at its own height.
                let pin = planned_filaments.range(OrderedFloat(interface_print_z - EPSILON)..).next().map(|(k, v)| (*k, *v));
                match pin {
                    Some((key, pinned)) if key.0 <= interface_print_z + EPSILON => {
                        if pinned.window.contains(height) {
                            candidate = pinned;
                        }
                    }
                    _ => {
                        candidate = plan.coarsest_for_slab(interface_print_z, height);
                        if candidate.valid() {
                            if let Some(base) = st.nonempty_layer_of(i, SMA_INTERMEDIATE) {
                                if !candidate.window.contains(st.layer_storage[base].height) {
                                    candidate = SupportMergePlanCandidate::default();
                                }
                            }
                        }
                    }
                }
            } else {
                candidate = plan.finest_interface_for_slab(interface_print_z, height);
                if candidate.valid() && !st.candidate_fits_bottom_contact(i, &candidate) {
                    candidate = SupportMergePlanCandidate::default();
                }
            }
            if !candidate.valid() {
                // BOOST_LOG_TRIVIAL(warning) << "Support interface at z=" ... << " stays " ... "mm above the support below it: no filament can extrude a " ... "mm slab there";
                continue;
            }
            st.layer_storage[interface_top].height = height;
            st.layer_storage[interface_top].bottom_z = landing_z;
            if with_base_filament {
                planned_filaments.insert(OrderedFloat(interface_print_z), candidate);
                continue;
            }
            planned_interface_filaments.insert(OrderedFloat(interface_print_z), candidate);
            let planned_height = planned_interface_heights.entry(OrderedFloat(interface_print_z)).or_insert(0.);
            // std::max(planned_height, height)
            if *planned_height < height {
                *planned_height = height;
            }
        }
    }

    // Rebuild the areas generate_support_toolpaths() indexes by support layer id: a level the merge left
    // without any area no longer gets a support layer, so everything below it shifts down by one.
    let mut support_layer_id_of_level: Vec<i32> = vec![-1; num_levels];
    let mut num_support_layers: i32 = 0;
    for i in 0..num_levels {
        if st.level_has_areas(i) {
            support_layer_id_of_level[i] = num_support_layers;
            num_support_layers += 1;
        }
    }
    // A level the merge emptied hands its areas over to the slab that swallowed its base.
    for i in 0..num_levels {
        if support_layer_id_of_level[i] < 0 {
            support_layer_id_of_level[i] = support_layer_id_of_level[slab_top_of_level[i] as usize];
        }
    }

    // SupportCommon.cpp:2145-2182, the lambda `remap_per_layer_areas`.
    let mut remap_per_layer_areas = |areas: Option<&mut Vec<ExPolygons>>| {
        let Some(areas) = areas else {
            return;
        };
        if areas.is_empty() {
            return;
        }
        let mut merged: Vec<ExPolygons> = vec![Vec::new(); num_support_layers as usize];
        for i in 0..areas.len() {
            if areas[i].is_empty() {
                continue;
            }
            let mut source_level: usize = i;
            if let Some(per_layer_area_print_z) = per_layer_area_print_z {
                if i >= per_layer_area_print_z.len() {
                    continue;
                }
                let print_z: f64 = per_layer_area_print_z[i];
                let level_it = st.levels.partition_point(|level| level.print_z < print_z - EPSILON);
                if level_it == st.levels.len() || st.levels[level_it].print_z > print_z + EPSILON {
                    continue;
                }
                source_level = level_it;
            }
            if source_level >= slab_top_of_level.len() {
                continue;
            }
            let remaining: ExPolygons = if st.interface_borrowed_areas[source_level].is_empty() {
                std::mem::take(&mut areas[i])
            } else {
                cu::diff_ex_ep(&areas[i], &cu::union_(&st.interface_borrowed_areas[source_level]), No)
            };
            if remaining.is_empty() {
                continue;
            }
            let target_level: usize = slab_top_of_level[source_level] as usize;
            if target_level >= support_layer_id_of_level.len() {
                continue;
            }
            let id = support_layer_id_of_level[target_level];
            if id >= 0 {
                merged[id as usize].extend(remaining);
            }
        }
        for expolygons in merged.iter_mut() {
            if expolygons.len() > 1 {
                *expolygons = cu::union_ex_expolygons(expolygons);
            }
        }
        *areas = merged;
    };
    remap_per_layer_areas(cooldown_areas);
    remap_per_layer_areas(lightning_infill_areas);

    // SupportCommon.cpp:1555-1560, the lambda `compact_arrays`.
    // Folding areas together leaves holes in the arrays, and generate_support_layers() cannot cope with those.
    for array_id in 0..SMA_COUNT {
        st.arrays[array_id].retain(|&layer| layer != NULL_LAYER);
    }
}
