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
// The two `tbb::parallel_for` run as one range each, in the order of the layers. The second one leaves its
// range at the first layer without a base interface (`return` in the body of the range, SupportCommon.cpp:253):
// with one range that is the rest of the layers.
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
            for layer_id in 1..base_interface_layers.len() {
                let base_interface_layer = base_interface_layers[layer_id];
                if base_interface_layer == NULL_LAYER {
                    // `return` of the body of the range
                    break;
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
//
// FIDELITY-NOTE: the crate's LayerRegion has no `adaptive_merge_patches` member (Layer.hpp); the patches of
// the regions of the layer are handed in: per patch `bottom_idx`, `top_idx` and `area()`.
pub fn merged_band_footprint(patches: &[(usize, usize, ExPolygons)], layer_idx: usize) -> Polygons {
    let mut band: Polygons = Vec::new();
    for (bottom_idx, top_idx, area) in patches {
        if *bottom_idx <= layer_idx && layer_idx <= *top_idx {
            band.extend(crate::geometry::to_polygons(area));
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
