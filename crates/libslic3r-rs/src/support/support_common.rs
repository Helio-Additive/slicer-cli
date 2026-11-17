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
