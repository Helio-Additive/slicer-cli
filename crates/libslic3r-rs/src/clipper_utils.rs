//! Clipper polygon boolean operations module.
//!
//! This module provides polygon boolean operations (union, intersection, difference, XOR)
//! and offset operations: ports of ClipperUtils.cpp on the Rust port of ClipperLib
//! (`crate::clipper`), at coord_t like the C++.
//!
//! These operations are essential for:
//! - Computing perimeter offsets
//! - Infill clipping
//! - Support generation
//! - Layer boolean operations

use crate::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polyline};
use crate::surface::Surface;
use crate::{Coord, CoordF};

/// Safety offset behavior for boolean operations
/// ClipperUtils.hpp:445
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApplySafetyOffset {
    /// Apply safety offset (grow by epsilon before operation)
    Yes,
    /// No safety offset
    No,
}

// R910/R911 — the helpers of this section ran on geo-clipper (`clipper-sys`,
// Clipper 6 in C++, on a 1 um grid) until R910; they follow the ClipperUtils.cpp
// function each stands for on the Rust ClipperLib, at coord_t:
//
// | helper | ClipperUtils (pin) |
// |-|-|
// | `union` | `union_ex(const ExPolygons&, const ExPolygons&)`, ClipperUtils.cpp:824-831 |
// | `xor` | `xor_ex(const ExPolygons&, const ExPolygons&)`, ClipperUtils.cpp:836-838 |
// | `offset_polyline`, `offset_polyline_miter` | `offset(const Polyline&, ...)`, ClipperUtils.cpp:418-419 |
// | `offset_polygon` | `offset_ex(const Polygon&, ...)`, ClipperUtils.hpp:358-364 |
// | `offset_polygons`, `offset_polygons_round` | `offset_ex(const Polygons&, ...)`, ClipperUtils.cpp:415-416 |
// | `offset_expolygon` | `offset_ex(const ExPolygon&, ...)`, ClipperUtils.cpp:567-569 |
// | `offset_expolygons` and its miter and round variants | `offset_ex(const ExPolygons&, ...)`, ClipperUtils.cpp:570-571 |
// | `intersection_base` | `intersection_ex(const ExPolygons&, const ExPolygons&)`, ClipperUtils.cpp:802-803 |
//
// `union_ex`, `union_polygons_ex`, `intersection` and `difference`, and the
// reconstruction of the `_clib` booleans and offsets, are their ClipperLib arms.
// The deltas keep each helper's unit: millimetres for the closed offsets (the
// scaled `float` the C++ receives is `mm_to_scaled_f32`), scaled units for the
// polyline offsets (rounded to `float` as the C++ parameter does).

/// Offset type for polygon inflation/deflation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OffsetType {
    /// Offset outward (grow the polygon)
    Inflate,
    /// Offset inward (shrink the polygon)
    Deflate,
}

/// Join type for offset corners.
/// Defaults to Miter to match BambuStudio's DefaultJoinType.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OffsetJoinType {
    /// Square corners
    Square,
    /// Round corners
    Round,
    /// Mitered corners (default, matches BambuStudio)
    #[default]
    Miter,
}

// ============================================================================
// Boolean Operations
// ============================================================================

/// Compute the union of two sets of polygons.
///
/// ClipperUtils.cpp:824-831 `union_ex(poly1, poly2)` unions the two sets appended,
/// `union_ex(const ExPolygons&)` (ClipperUtils.cpp:815-816):
/// PolyTreeToExPolygons(clipper_do_polytree(ctUnion, ExPolygonsProvider(subject),
/// EmptyPathsProvider(), pftNonZero)), which `union_ex_clib(_, 1)` is. No early
/// return: the C++ runs the union for an empty operand too.
pub fn union(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    let mut loops: Vec<Polygon> = Vec::new();
    for e in subject.iter().chain(clip.iter()) {
        loops.push(e.contour.clone());
        loops.extend(e.holes.iter().cloned());
    }
    union_ex_clib(&loops, 1)
}

/// Compute the union of a single set of potentially overlapping polygons.
///
/// ClipperUtils.cpp:815 `ExPolygons union_ex(const ExPolygons &subject)` —
/// `PolyTreeToExPolygons(clipper_do_polytree(ctUnion, ExPolygonsProvider(subject),
/// EmptyPathsProvider(), pftNonZero))`. This is a SINGLE Clipper union of the
/// whole subject set against an empty clip (NonZero fill rule), not an iterative
/// pairwise fold.
///
/// FIDELITY-NOTE: a single expolygon is returned as it is; the C++ unions it too.
pub fn union_ex(polygons: &[ExPolygon]) -> ExPolygons {
    if polygons.is_empty() {
        return vec![];
    }
    if polygons.len() == 1 {
        return polygons.to_vec();
    }

    // R728 — see `union_polygons_ex`. Nothing in the `_clib` family calls this
    // wrapper (verified by brace-counting scan), so it can dispatch directly.
    let mut loops: Vec<Polygon> = Vec::new();
    for e in polygons {
        loops.push(e.contour.clone());
        loops.extend(e.holes.iter().cloned());
    }
    union_ex_clib(&loops, 1)
}

/// Compute the union of raw Polygons into ExPolygons.
/// This is used after simplification to reconstruct proper ExPolygons with holes.
/// Equivalent to BambuStudio's union_ex(simplify_p(...))
///
/// R728 — native `union_ex(Polygons)` (ClipperUtils.cpp:813-814) is one NonZero
/// ClipperLib union at coord_t, which `union_ex_clib(_, 1)` is. Shipped on at
/// R738; priced on all four fixtures at +7,394 in-order matched lines against
/// the geo-clipper body it replaced, removed at R911.
pub fn union_polygons_ex(polygons: &[Polygon]) -> ExPolygons {
    union_ex_clib(polygons, 1)
}

/// Compute the intersection of two sets of polygons.
///
/// C++ `intersection_ex(ExPolygons, ExPolygons)` = `_clipper_ex(ctIntersection, …, pftNonZero)`
/// (ClipperUtils.cpp:802-803) — closed-path ClipperLib intersection at coord_t (1e5)
/// precision: `intersection_clib` (`clipper_do<Paths>(ctIntersection)` and the PolyTree
/// re-union).
pub fn intersection(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return vec![];
    }
    intersection_clib(subject, clip)
}

/// `CLIPPER_DIFF_EMPTY_CLIP` (on unless `CLIPPER_DIFF_EMPTY_CLIP=0`): a difference with an empty clip runs
/// Clipper, as `diff` / `diff_ex` of the pin always do (ClipperUtils.cpp:640-673: `_clipper` and
/// `clipper_do_polytree` do not test for an empty clip, so the subject comes back unioned, with Clipper's
/// orientation, vertex order and first vertices). With the gate off the earlier arm returns the subject as it is.
#[inline]
fn diff_empty_clip_returns_subject() -> bool {
    !crate::faithful_gate("CLIPPER_DIFF_EMPTY_CLIP")
}

/// Compute the difference of two sets of polygons (subject - clip).
///
/// C++ `diff_ex(ExPolygons, ExPolygons)` (ClipperUtils.cpp:755-756), which
/// `difference_clib` is (R728).
///
/// With an empty clip the C++ unions the subject; so does this unless `CLIPPER_DIFF_EMPTY_CLIP=0`
/// (the earlier arm: the subject returned as it is).
pub fn difference(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() {
        return vec![];
    }
    if clip.is_empty() && diff_empty_clip_returns_subject() {
        return subject.to_vec();
    }
    difference_clib(subject, clip)
}

/// Compute the XOR of two sets of polygons.
///
/// ClipperUtils.cpp:836-838 `xor_ex(const ExPolygons&, const ExPolygons&,
/// ApplySafetyOffset::No)` = `_clipper_ex(ctXor, ...)`, empty operands included.
pub fn xor(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    _clipper_ex(
        crate::clipper::ClipType::ctXor,
        &clib_expolygons_paths(subject),
        &clib_expolygons_paths(clip),
        ApplySafetyOffset::No,
        crate::clipper::PolyFillType::pftNonZero,
    )
}

// ============================================================================
// Offset Operations
// ============================================================================

/// Offset an open polyline by a given (scaled) distance, returning closed polygons.
///
/// ClipperUtils.cpp:418-419 `Slic3r::Polygons offset(const Slic3r::Polyline &polyline,
/// const float delta, JoinType joinType, double miterLimit, EndType end_type)` with the
/// defaults of ClipperUtils.hpp:346 (DefaultLineJoinType = jtSquare,
/// DefaultLineMiterLimit = 0, DefaultEndType = etOpenButt). The C++ has no early
/// return for a short polyline.
///
/// `delta` is expressed in scaled (`coord_t`) units exactly as the C++ caller passes it
/// (e.g. `float(scale_(width/2)) + scaled_epsilon`), rounded to f32 as the `float`
/// parameter does.
pub fn offset_polyline(polyline: &Polyline, delta_scaled: CoordF) -> Vec<Polygon> {
    clib_polygons_from_paths(&offset_polyline_path(
        &clib_path_of_points(polyline.points()),
        (delta_scaled as f32) as f64,
        crate::clipper::JoinType::jtSquare,
        0.0,
        crate::clipper::EndType::etOpenButt,
    ))
}

/// Offset an open polyline with an explicit ClipperLib `jtMiter` join and miter limit,
/// returning closed polygons.
///
/// ClipperUtils.cpp:418-419 `offset(const Polyline&, delta, joinType, miterLimit,
/// end_type)` with jtMiter, the caller's miter limit and DefaultEndType = etOpenButt,
/// for the caller `SupportCommon.cpp:2116`, which passes `offset(to_polylines(...),
/// scaled<float>(0.002), jtMiter, 1.2)`. The default-argument flavour is
/// [`offset_polyline`].
///
/// `delta_scaled` is in scaled (`coord_t`) units, rounded to f32 as the `float`
/// parameter does; `miter_limit` is the dimensionless ClipperLib miter limit.
pub fn offset_polyline_miter(
    polyline: &Polyline,
    delta_scaled: CoordF,
    miter_limit: CoordF,
) -> Vec<Polygon> {
    clib_polygons_from_paths(&offset_polyline_path(
        &clib_path_of_points(polyline.points()),
        (delta_scaled as f32) as f64,
        crate::clipper::JoinType::jtMiter,
        miter_limit,
        crate::clipper::EndType::etOpenButt,
    ))
}

/// Offset a polygon by a given distance.
///
/// Positive delta inflates (grows) the polygon, negative delta deflates (shrinks) it.
///
/// ClipperUtils.hpp:358-364 `offset_ex(const Polygon&, delta, joinType,
/// DefaultMiterLimit)` = `offset_ex(Polygons{ polygon }, ...)` (ClipperUtils.cpp:415-416),
/// which `offset_ex_polygons_clib` is.
///
/// # Arguments
/// * `polygon` - The polygon to offset
/// * `delta` - The offset distance in mm (positive = grow, negative = shrink)
/// * `join_type` - The type of join to use at corners
///
/// # Returns
/// A vector of ExPolygons representing the offset result.
pub fn offset_polygon(polygon: &Polygon, delta: CoordF, join_type: OffsetJoinType) -> ExPolygons {
    offset_ex_polygons_clib(std::slice::from_ref(polygon), mm_to_scaled_f32(delta), join_type)
}

/// Offset an ExPolygon by a given distance (mm).
///
/// Positive delta inflates (grows) the polygon, negative delta deflates (shrinks) it.
///
/// ClipperUtils.cpp:567-569 `offset_ex(const ExPolygon&, delta, joinType,
/// DefaultMiterLimit)` — `expolygon_offset`, then ClipperPaths_to_Slic3rExPolygons with
/// its default `do_union = false`: one pftEvenOdd union into a PolyTree.
pub fn offset_expolygon(
    expolygon: &ExPolygon,
    delta: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset_ex_expolygon_clib(expolygon, mm_to_scaled_f32(delta), join_type, CLIB_MITER_LIMIT)
}

/// Offset multiple ExPolygons by a given distance (mm).
///
/// Positive delta inflates (grows) the polygons, negative delta deflates (shrinks) them.
///
/// ClipperUtils.cpp:570-571 `offset_ex(const ExPolygons&, delta, joinType,
/// DefaultMiterLimit)`.
pub fn offset_expolygons(
    expolygons: &[ExPolygon],
    delta: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset_ex_expolygons_clib_ml(expolygons, mm_to_scaled_f32(delta), join_type, CLIB_MITER_LIMIT)
}

/// Offset multiple ExPolygons with `ClipperLib::jtMiter` and an explicit
/// miter limit.
///
/// ClipperUtils.cpp:570-571 `offset_ex(const ExPolygons&, float delta,
/// ClipperLib::jtMiter, double miterLimit)` — needed by `SLA/Pad.cpp:257-260`, which
/// passes a miter limit of `1` instead of the crate-wide default `3.0` baked into
/// `OffsetJoinType::Miter`.
pub fn offset_expolygons_miter_limit(
    expolygons: &[ExPolygon],
    delta: CoordF,
    miter_limit: f64,
) -> ExPolygons {
    offset_ex_expolygons_clib_ml(expolygons, mm_to_scaled_f32(delta), OffsetJoinType::Miter, miter_limit)
}

/// Offset multiple Polygons by a given distance (mm).
///
/// ClipperUtils.cpp:415-416 `offset_ex(const Polygons&, delta, joinType,
/// DefaultMiterLimit)`, which `offset_ex_polygons_clib` is.
pub fn offset_polygons(
    polygons: &[Polygon],
    delta: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset_ex_polygons_clib(polygons, mm_to_scaled_f32(delta), join_type)
}

/// Offset Polygons by `delta` (mm) using round joins with an explicit arc
/// tolerance (mm).
///
/// ClipperUtils.cpp:415-416 `offset_ex(const Polygons&, delta, jtRound, miterLimit)`,
/// where `miterLimit` is the arc tolerance in scaled units (raw_offset,
/// ClipperUtils.cpp:279-282); it is a `double` there. For callers that pass a
/// non-default arc tolerance (e.g. SLA/ConcaveHull.cpp:136-137).
pub fn offset_polygons_round(
    polygons: &[Polygon],
    delta: CoordF,
    arc_tolerance_mm: CoordF,
) -> ExPolygons {
    offset_ex_polygons_clib_ml(
        polygons,
        mm_to_scaled_f32(delta),
        OffsetJoinType::Round,
        arc_tolerance_mm * crate::SCALING_FACTOR,
    )
}

/// ExPolygons variant of [`offset_polygons_round`].
///
/// ClipperUtils.cpp:570-571 `offset_ex(const ExPolygons&, delta, jtRound, miterLimit)`,
/// `miterLimit` the arc tolerance in scaled units (offset_expolygon_inner,
/// ClipperUtils.cpp:443-446).
pub fn offset_expolygons_round(
    expolygons: &[ExPolygon],
    delta: CoordF,
    arc_tolerance_mm: CoordF,
) -> ExPolygons {
    offset_ex_expolygons_clib_ml(
        expolygons,
        mm_to_scaled_f32(delta),
        OffsetJoinType::Round,
        arc_tolerance_mm * crate::SCALING_FACTOR,
    )
}

/// Shrink (inset) ExPolygons by a given distance.
///
/// This is a convenience function that calls offset with a negative delta.
pub fn shrink(expolygons: &[ExPolygon], distance: CoordF, join_type: OffsetJoinType) -> ExPolygons {
    offset_expolygons(expolygons, -distance.abs(), join_type)
}

/// Grow (outset) ExPolygons by a given distance.
///
/// This is a convenience function that calls offset with a positive delta.
pub fn grow(expolygons: &[ExPolygon], distance: CoordF, join_type: OffsetJoinType) -> ExPolygons {
    offset_expolygons(expolygons, distance.abs(), join_type)
}

// ============================================================================
// R117 — scale_-faithful delta helpers (R109/R113/R115 class)
// ============================================================================
// Native BBS `scale_(v)` is the macro `((v) / SCALING_FACTOR)` (libslic3r.h:81)
// with the f64 literal SCALING_FACTOR = 0.00001 (slightly ABOVE 1e-5), then
// `coord_t()` truncates. The spacings/widths it scales are stored as `float`
// (Flow::m_width/m_spacing), so the faithful chain is `trunc(f32(mm) / 0.00001)`:
//   scale_(0.45)=44999, scale_(0.5)=49999, scale_(0.42)=41999 — NOT the crate's
// `scale()` (multiply by exact 100000.0 + round). The only_one_wall_top block
// (min_width_top_surface, bridge_offset) and the perimeter offsets need this for
// byte-parity. Pure helpers — callers opt in under F1_UNION.

/// Native `coord_t(scale_(mm))` = `trunc(f32(mm) / 0.00001)`.
pub fn scale_faithful(mm: CoordF) -> Coord {
    ((mm as f32) as f64 / 0.00001_f64).trunc() as Coord
}

/// The mm delta to hand `offset_expolygons_clib` (which multiplies by
/// `SCALING_FACTOR` = 1e5 internally) so the resulting ClipperOffset delta equals
/// native's `float(scaled_delta)` — the native scaled delta AFTER the f32 cast that
/// `offset(paths, const float delta)` applies (ClipperUtils.hpp:341). Used for the
/// block offsets (bridge_offset = 1.5·max(scale_(eps),scale_(pw)); min_width_top).
pub fn offset_delta_mm_from_scaled_f32(scaled_delta: CoordF) -> CoordF {
    ((scaled_delta as f32) as f64) / crate::SCALING_FACTOR
}

// ============================================================================
// Utility Functions
// ============================================================================

/// Simplify polygons by removing small details.
pub fn simplify(expolygons: &[ExPolygon], tolerance: CoordF) -> ExPolygons {
    expolygons
        .iter()
        .map(|expoly| {
            let mut result = expoly.clone();
            result.simplify(crate::scale(tolerance));
            result
        })
        .filter(|expoly| !expoly.is_empty() && expoly.area().abs() > tolerance * tolerance)
        .collect()
}

/// Remove very small polygons from a set.
pub fn remove_small(expolygons: &[ExPolygon], min_area: CoordF) -> ExPolygons {
    expolygons
        .iter()
        .filter(|expoly| expoly.area().abs() > min_area)
        .cloned()
        .collect()
}

/// Check if two sets of polygons overlap.
pub fn polygons_overlap(a: &[ExPolygon], b: &[ExPolygon]) -> bool {
    !intersection_base(a, b).is_empty()
}

/// Helper intersection that doesn't conflict with trait method
///
/// ClipperUtils.cpp:802-803 `intersection_ex(const ExPolygons&, const ExPolygons&)`,
/// which `intersection_clib` is.
fn intersection_base(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    intersection_clib(subject, clip)
}

/// Compute the total area of a set of polygons.
pub fn total_area(expolygons: &[ExPolygon]) -> CoordF {
    expolygons.iter().map(|p| p.area()).sum()
}

/// Morphological opening: shrink then grow by the same amount.
///
/// This removes small protrusions and smooths contours while preserving
/// the overall shape. Used for gap detection and thin wall handling.
///
/// # Arguments
/// * `expolygons` - The polygons to process
/// * `distance` - The opening distance in mm
/// * `join_type` - The join type for offset corners
pub fn opening(
    expolygons: &[ExPolygon],
    distance: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() || distance <= 0.0 {
        return expolygons.to_vec();
    }
    let shrunk = shrink(expolygons, distance, join_type);
    grow(&shrunk, distance, join_type)
}

/// Opening operation with ExPolygon output
/// ClipperUtils.hpp:428
/// C++: ExPolygons opening_ex(const ExPolygons &polygons, const float delta, ...)
pub fn opening_ex(expolygons: &[ExPolygon], distance: CoordF) -> ExPolygons {
    if expolygons.is_empty() || distance <= 0.0 {
        return expolygons.to_vec();
    }
    // Native opening_ex = offset2_ex(-d, +d, jtMiter) via ClipperLib @1e5
    // (ClipperUtils.hpp:428); geo grids to 1um (R100 class). Gated full-res —
    // TOPFILL_FAITHFUL flips the whole classification pipeline at once.
    if crate::faithful_gate("TOPFILL_FAITHFUL") {
        return offset2_ex_clib(expolygons, -distance, distance, OffsetJoinType::Miter);
    }
    let shrunk = shrink(expolygons, distance, OffsetJoinType::Miter);
    grow(&shrunk, distance, OffsetJoinType::Miter)
}

/// Difference operation with ExPolygon output (surfaces vs surfaces)
/// ClipperUtils.hpp:449
/// C++: ExPolygons diff_ex(const Surfaces &subject, const Surfaces &clip, ApplySafetyOffset)
pub fn diff_ex(
    subject: &[Surface],
    clip: &[Surface],
    _safety_offset: ApplySafetyOffset,
) -> ExPolygons {
    if subject.is_empty() {
        return vec![];
    }
    if clip.is_empty() && diff_empty_clip_returns_subject() {
        return subject.iter().map(|s| s.expolygon.clone()).collect();
    }

    let subject_expolygons: Vec<ExPolygon> = subject.iter().map(|s| s.expolygon.clone()).collect();
    let clip_expolygons: Vec<ExPolygon> = clip.iter().map(|s| s.expolygon.clone()).collect();

    // Gated faithful route: ClipperLib @1e5 honoring ApplySafetyOffset (the geo
    // path IGNORES _safety_offset — native ::Yes raw-offsets the clip by +10u,
    // R116 shim).
    if crate::faithful_gate("TOPFILL_FAITHFUL") {
        return match _safety_offset {
            ApplySafetyOffset::Yes => {
                difference_clib_safety(&subject_expolygons, &clip_expolygons)
            }
            ApplySafetyOffset::No => difference_clib(&subject_expolygons, &clip_expolygons),
        };
    }
    difference(&subject_expolygons, &clip_expolygons)
}

/// Difference operation with ExPolygon output (surfaces vs expolygons)
/// ClipperUtils.hpp:456
/// C++: ExPolygons diff_ex(const Surfaces &subject, const ExPolygons &clip, ApplySafetyOffset)
pub fn diff_ex_surfaces_expolygons(
    subject: &[Surface],
    clip: &[ExPolygon],
    _safety_offset: ApplySafetyOffset,
) -> ExPolygons {
    if subject.is_empty() {
        return vec![];
    }
    if clip.is_empty() && diff_empty_clip_returns_subject() {
        return subject.iter().map(|s| s.expolygon.clone()).collect();
    }

    let subject_expolygons: Vec<ExPolygon> = subject.iter().map(|s| s.expolygon.clone()).collect();
    if crate::faithful_gate("TOPFILL_FAITHFUL") {
        return match _safety_offset {
            ApplySafetyOffset::Yes => difference_clib_safety(&subject_expolygons, clip),
            ApplySafetyOffset::No => difference_clib(&subject_expolygons, clip),
        };
    }
    difference(&subject_expolygons, clip)
}

/// Difference operation with ExPolygon output (polygons vs surfaces)
/// ClipperUtils.hpp:449
/// C++: ExPolygons diff_ex(const Polygons &subject, const Surfaces &clip, ApplySafetyOffset)
pub fn diff_ex_polygons_surfaces(
    subject: &[Polygon],
    clip: &[Surface],
    _safety_offset: ApplySafetyOffset,
) -> ExPolygons {
    if subject.is_empty() {
        return vec![];
    }
    if clip.is_empty() && diff_empty_clip_returns_subject() {
        // Convert polygons to expolygons (no holes)
        return subject.iter().map(|p| ExPolygon::new(p.clone())).collect();
    }

    let subject_expolygons: Vec<ExPolygon> =
        subject.iter().map(|p| ExPolygon::new(p.clone())).collect();
    let clip_expolygons: Vec<ExPolygon> = clip.iter().map(|s| s.expolygon.clone()).collect();

    if crate::faithful_gate("TOPFILL_FAITHFUL") {
        return match _safety_offset {
            ApplySafetyOffset::Yes => {
                difference_clib_safety(&subject_expolygons, &clip_expolygons)
            }
            ApplySafetyOffset::No => difference_clib(&subject_expolygons, &clip_expolygons),
        };
    }
    difference(&subject_expolygons, &clip_expolygons)
}

/// Intersection operation with ExPolygon output (for surfaces and fill_expolygons)
/// ClipperUtils.cpp
/// C++: ExPolygons intersection_ex(const SurfacesPtr &subject, const ExPolygons &clip)
pub fn intersection_ex(subject: &[Surface], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return vec![];
    }

    let subject_expolygons: Vec<ExPolygon> = subject.iter().map(|s| s.expolygon.clone()).collect();
    if crate::faithful_gate("TOPFILL_FAITHFUL") {
        return intersection_clib(&subject_expolygons, clip);
    }
    intersection_base(&subject_expolygons, clip)
}

/// Intersection operation (ExPolygons with Surfaces)
/// ClipperUtils.cpp
pub fn intersection_surfaces(subject: &[ExPolygon], clip: &[Surface]) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return vec![];
    }

    let clip_expolygons: Vec<ExPolygon> = clip.iter().map(|s| s.expolygon.clone()).collect();
    if crate::faithful_gate("TOPFILL_FAITHFUL") {
        return intersection_clib(subject, &clip_expolygons);
    }
    intersection_base(subject, &clip_expolygons)
}

/// Intersection operation (Surfaces with ExPolygons)
/// ClipperUtils.cpp
pub fn intersection_surfaces_expolygons(subject: &[Surface], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return vec![];
    }

    let subject_expolygons: Vec<ExPolygon> = subject.iter().map(|s| s.expolygon.clone()).collect();
    if crate::faithful_gate("TOPFILL_FAITHFUL") {
        return intersection_clib(&subject_expolygons, clip);
    }
    intersection_base(&subject_expolygons, clip)
}

/// Convert Surfaces to Polygons for boolean operations
/// ClipperUtils.cpp
pub fn to_polygons(surfaces: &[Surface]) -> Vec<Polygon> {
    let mut polygons = Vec::new();
    for surface in surfaces {
        polygons.push(surface.expolygon.contour.clone());
        for hole in &surface.expolygon.holes {
            polygons.push(hole.clone());
        }
    }
    polygons
}

/// Morphological closing: grow then shrink by the same amount.
///
/// This fills small gaps and holes while preserving the overall shape.
///
/// # Arguments
/// * `expolygons` - The polygons to process
/// * `distance` - The closing distance in mm
/// * `join_type` - The join type for offset corners
pub fn closing(
    expolygons: &[ExPolygon],
    distance: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() || distance <= 0.0 {
        return expolygons.to_vec();
    }
    let grown = grow(expolygons, distance, join_type);
    shrink(&grown, distance, join_type)
}

/// Offset2: shrink by amount1, then grow by amount2.
///
/// This is the general morphological operation used in BambuStudio for
/// perimeter generation and gap detection. When amount1 != amount2, it
/// creates controlled erosion/dilation.
///
/// # Arguments
/// * `expolygons` - The polygons to process
/// * `shrink_amount` - Amount to shrink (positive)
/// * `grow_amount` - Amount to grow back (positive)
/// * `join_type` - The join type for offset corners
pub fn offset2(
    expolygons: &[ExPolygon],
    shrink_amount: CoordF,
    grow_amount: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() {
        return vec![];
    }
    let shrunk = shrink(expolygons, shrink_amount.abs(), join_type);
    if shrunk.is_empty() {
        return vec![];
    }
    grow(&shrunk, grow_amount.abs(), join_type)
}

// ============================================================================
// ClipperLib (clipper-z-sys) vertex-exact offset — perimeter inner-wall path
//
// PARITY: geo-clipper's miter offset inserts ~10-18% extra vertices on
// sub-|delta| edges (proven in /tmp/offsetvtx_findings.md). The vendored
// BambuStudio ClipperLib (clipper-z-sys, namespaced ClipperZSys::ClipperLib,
// CLIPPERLIB_INT32) is vertex-EXACT. These wrappers route ONLY the perimeter
// inner-wall successive offsets (offset2/shrink/grow) through it to byte-match
// native inner-wall vertex density. (Since R910 every helper of this file runs
// on ClipperLib; geo-clipper left the crate at R911.)
//
// Faithfulness: cz_offset_expolygon replicates ClipperUtils.cpp
// `offset_expolygon_inner` (per-ExPolygon ClipperOffset of contour + holes, then
// difference/append). The cross-ExPolygon union + path->ExPolygon reconstruction
// is `polytree_ex_clib` (ClipperLib @1e5, full-resolution). Until R911 the
// reconstruction of F1_UNION=0 was geo-clipper's 1 um grid (R100/R106: the
// ungated geo reconstruction in `difference_clib` silently re-gridded `last` and
// defeated two rounds of routing).
//
// i32 RANGE: ClipperLib here is CLIPPERLIB_INT32. libslic3r coords are i64@1e5.
// Benchy ~6e6 << i32 max 2.1e9 → safe. The wrapper debug-asserts the range
// (clipper_z::narrow_i32 inside marshalling) and documents the limit.
// ============================================================================

/// ClipperLib join-type discriminant for the shim: 0=Miter, 1=Round, 2=Square.
#[inline]
fn clib_join_code(jt: OffsetJoinType) -> i32 {
    match jt {
        OffsetJoinType::Miter => 0,
        OffsetJoinType::Round => 1,
        OffsetJoinType::Square => 2,
    }
}

/// `miterLimit` argument to the ClipperOffset, in the SAME convention as C++:
/// for jtMiter it is the unitless MiterLimit (DefaultMiterLimit=3.0); for jtRound
/// it is the ArcTolerance in SCALED coord units (DefaultMiterLimit=3.0 scaled).
const CLIB_MITER_LIMIT: f64 = 3.0;

#[inline]
fn assert_i32_scaled(v: Coord) {
    debug_assert!(
        v >= i32::MIN as i64 && v <= i32::MAX as i64,
        "clib offset: scaled coordinate {v} out of i32 range (ClipperLib is \
         CLIPPERLIB_INT32; max ~2.1e9 = ~21000mm at 1e5/mm). Geometry too large \
         for the perimeter-offset ClipperLib path."
    );
}

// ============================================================================
// ClipperUtils.cpp over the Rust port of ClipperLib (`crate::clipper`).
//
// The functions below are the ports of the ClipperUtils.cpp functions they are
// named after, on `crate::clipper`, and the `*_clib` wrappers of this file call
// them. Until R891 the wrappers went through the shim of `clipper-z-sys`
// (`cz_*`), each routine of which is those few statements of ClipperUtils.cpp
// around the C++ library; the shim stays as the reference of the tests
// (`tests/clipper_utils_native_parity.rs`).
//
// They work on `ClipperLib::Paths` (i32 coordinates), like the C++; the wrappers
// keep their conversion from and to the engine's polygons.
//
// The offsets of the C++ are `float` parameters. The shim takes them as `double`
// and its callers round them to f32 before the call where the C++ would; the
// functions here take the same `f64` the shim takes.
// ============================================================================

/// ClipperUtils.hpp:29 `static constexpr const float ClipperSafetyOffset = 10.f;`
const CLIPPER_SAFETY_OFFSET: f64 = 10.0;
/// ClipperUtils.hpp:44 `static constexpr const double ClipperOffsetShortestEdgeFactor = 0.005;`
const CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR: f64 = 0.005;

/// The shim's `read_closed_paths` (clipper_z_shim.cpp): the flat (x, y) pairs and
/// the per-path point counts the wrappers build, as `ClipperLib::Paths`.
#[doc(hidden)]
pub fn clib_read_closed_paths(xy: &[i32], lens: &[i32]) -> crate::clipper::Paths {
    let mut paths = crate::clipper::Paths::with_capacity(lens.len());
    let mut cursor = 0usize;
    for &len in lens {
        let len = len as usize;
        let mut path = crate::clipper::Path::with_capacity(len);
        for i in 0..len {
            path.push(crate::clipper::IntPoint::new(xy[cursor + 2 * i], xy[cursor + 2 * i + 1]));
        }
        cursor += 2 * len;
        paths.push(path);
    }
    paths
}

/// [`clib_read_closed_paths`] for a wrapper that hands the shim a null pointer
/// when a buffer is empty: `read_closed_paths` then returns no path at all.
#[doc(hidden)]
pub fn clib_read_closed_paths_or_null(xy: &[i32], lens: &[i32]) -> crate::clipper::Paths {
    if xy.is_empty() || lens.is_empty() {
        return crate::clipper::Paths::new();
    }
    clib_read_closed_paths(xy, lens)
}

/// What the wrappers do with the `CzZPaths` of a flat shim routine
/// (`marshal_paths`): every path becomes a `Polygon`, in order. A result without
/// any point has a null `coords` there and decodes to nothing.
#[doc(hidden)]
pub fn clib_polygons_from_paths(paths: &crate::clipper::Paths) -> Vec<Polygon> {
    if paths.iter().all(|p| p.is_empty()) {
        return Vec::new();
    }
    paths
        .iter()
        .map(|p| Polygon::from_points(p.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()))
        .collect()
}

/// The shim's code of a fill type: 0=pftEvenOdd, 1=pftNonZero, 2=pftPositive,
/// 3=pftNegative, anything else pftNonZero.
#[doc(hidden)]
pub fn clib_fill_type(fill_type: i32) -> crate::clipper::PolyFillType {
    match fill_type {
        0 => crate::clipper::PolyFillType::pftEvenOdd,
        1 => crate::clipper::PolyFillType::pftNonZero,
        2 => crate::clipper::PolyFillType::pftPositive,
        3 => crate::clipper::PolyFillType::pftNegative,
        _ => crate::clipper::PolyFillType::pftNonZero,
    }
}

/// The shim's code of a join type: 0=jtMiter, 1=jtRound, 2=jtSquare, anything
/// else jtMiter.
#[doc(hidden)]
pub fn clib_join_type(join_type: i32) -> crate::clipper::JoinType {
    match join_type {
        1 => crate::clipper::JoinType::jtRound,
        2 => crate::clipper::JoinType::jtSquare,
        _ => crate::clipper::JoinType::jtMiter,
    }
}

/// The `TResult` of the ClipperUtils.cpp templates: `ClipperLib::Paths` or
/// `ClipperLib::PolyTree`, the two solutions `Clipper::Execute` writes.
#[doc(hidden)]
pub trait ClipperResult: Default {
    /// `clipper.Execute(clipType, retval, subjFillType, clipFillType)`
    fn execute(
        clipper: &mut crate::clipper::Clipper,
        clip_type: crate::clipper::ClipType,
        retval: &mut Self,
        subj_fill_type: crate::clipper::PolyFillType,
        clip_fill_type: crate::clipper::PolyFillType,
    ) -> bool;
    /// ClipperUtils.cpp:375-379
    /// C++: template<class Container> static void remove_outermost_polygon(Container & solution);
    fn remove_outermost_polygon(&mut self);
}

impl ClipperResult for crate::clipper::Paths {
    fn execute(
        clipper: &mut crate::clipper::Clipper,
        clip_type: crate::clipper::ClipType,
        retval: &mut Self,
        subj_fill_type: crate::clipper::PolyFillType,
        clip_fill_type: crate::clipper::PolyFillType,
    ) -> bool {
        clipper.execute(clip_type, retval, subj_fill_type, clip_fill_type)
    }

    /// ClipperUtils.cpp:376-377
    /// C++: template<> void remove_outermost_polygon<ClipperLib::Paths>(ClipperLib::Paths &solution)
    fn remove_outermost_polygon(&mut self) {
        if !self.is_empty() {
            self.remove(0);
        }
    }
}

impl ClipperResult for crate::clipper::PolyTree {
    fn execute(
        clipper: &mut crate::clipper::Clipper,
        clip_type: crate::clipper::ClipType,
        retval: &mut Self,
        subj_fill_type: crate::clipper::PolyFillType,
        clip_fill_type: crate::clipper::PolyFillType,
    ) -> bool {
        clipper.execute_poly_tree(clip_type, retval, subj_fill_type, clip_fill_type)
    }

    /// ClipperUtils.cpp:378-379
    /// C++: template<> void remove_outermost_polygon<ClipperLib::PolyTree>(ClipperLib::PolyTree &solution)
    fn remove_outermost_polygon(&mut self) {
        crate::clipper::PolyTree::remove_outermost_polygon(self);
    }
}

/// ClipperUtils.cpp:175-210
/// C++: static ExPolygons PolyTreeToExPolygons(ClipperLib::PolyTree &&polytree)
///
/// Replaces the shim's `polytree_to_grouped` + `marshal_grouped` and the decoding
/// of that layout in the wrappers (a contour, its holes, then the contours nested
/// in the holes): the same ExPolygons in the same order.
#[doc(hidden)]
pub fn poly_tree_to_ex_polygons(polytree: &crate::clipper::PolyTree) -> ExPolygons {
    fn to_polygon(contour: &crate::clipper::Path) -> Polygon {
        Polygon::from_points(contour.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect())
    }
    // static void PolyTreeToExPolygonsRecursive(ClipperLib::PolyNode &&polynode, ExPolygons *expolygons)
    fn poly_tree_to_ex_polygons_recursive(
        polytree: &crate::clipper::PolyTree,
        polynode: usize,
        expolygons: &mut ExPolygons,
    ) {
        let node = polytree.node(polynode);
        let cnt = expolygons.len();
        expolygons.push(ExPolygon::new(to_polygon(&node.contour)));
        expolygons[cnt].holes.reserve(node.child_count() as usize);
        for i in 0..node.child_count() as usize {
            let hole = polytree.node(node.childs[i]);
            expolygons[cnt].holes.push(to_polygon(&hole.contour));
        }
        for i in 0..node.child_count() as usize {
            let hole = polytree.node(node.childs[i]);
            // Add outer polygons contained by (nested within) holes.
            for j in 0..hole.child_count() as usize {
                poly_tree_to_ex_polygons_recursive(polytree, hole.childs[j], expolygons);
            }
        }
    }

    let mut retval: ExPolygons = Vec::new();
    for i in 0..polytree.child_count() as usize {
        poly_tree_to_ex_polygons_recursive(polytree, polytree.root.childs[i], &mut retval);
    }
    retval
}

/// ClipperUtils.cpp:213-229
/// C++: Polylines PolyTreeToPolylines(ClipperLib::PolyTree &&polytree)
///
/// The contours as `ClipperLib::Paths`: the tree itself first (its contour is
/// empty), then every node before its children.
#[doc(hidden)]
pub fn poly_tree_to_polylines(polytree: &crate::clipper::PolyTree) -> crate::clipper::Paths {
    // static void AddPolyNodeToPaths(ClipperLib::PolyNode &polynode, Polylines &out)
    fn add_poly_node_to_paths(polytree: &crate::clipper::PolyTree, polynode: usize, out: &mut crate::clipper::Paths) {
        let node = polytree.node(polynode);
        if !node.contour.is_empty() {
            out.push(node.contour.clone());
        }
        for &child in &node.childs {
            add_poly_node_to_paths(polytree, child, out);
        }
    }

    let mut out = crate::clipper::Paths::new();
    out.reserve(polytree.total().max(0) as usize);
    add_poly_node_to_paths(polytree, crate::clipper::POLY_TREE_ROOT, &mut out);
    out
}

/// ClipperUtils.cpp:270-300
/// C++: template<typename PathsProvider> static ClipperLib::Paths raw_offset(PathsProvider &&paths, float offset, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType endType = ClipperLib::etClosedPolygon)
///
/// Offset CCW contours outside, CW contours (holes) inside.
/// Don't calculate union of the output paths.
#[doc(hidden)]
pub fn raw_offset(
    paths: &crate::clipper::Paths,
    offset: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
    end_type: crate::clipper::EndType,
) -> crate::clipper::Paths {
    let mut co = crate::clipper::ClipperOffset::default();
    let mut out = crate::clipper::Paths::new();
    out.reserve(paths.len());
    let mut out_this = crate::clipper::Paths::new();
    if join_type == crate::clipper::JoinType::jtRound {
        co.arc_tolerance = miter_limit;
    } else {
        co.miter_limit = miter_limit;
    }
    co.shortest_edge_length = (offset * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR).abs();
    for path in paths {
        co.clear();
        // Execute reorients the contours so that the outer most contour has a positive area. Thus the output
        // contours will be CCW oriented even though the input paths are CW oriented.
        // Offset is applied after contour reorientation, thus the signum of the offset value is reversed.
        co.add_path(path, join_type, end_type);
        let ccw = if end_type == crate::clipper::EndType::etClosedPolygon {
            crate::clipper::orientation(path)
        } else {
            true
        };
        co.execute(&mut out_this, if ccw { offset } else { -offset });
        if !ccw {
            // Reverse the resulting contours.
            for path in out_this.iter_mut() {
                path.reverse();
            }
        }
        out.append(&mut out_this);
    }
    out
}

/// ClipperUtils.cpp:302-307
/// C++: template<typename PathsProvider> static ClipperLib::Paths safety_offset(PathsProvider &&paths)
///
/// Offset outside by 10um, one by one. The shim's `cz_safety_offset`.
#[doc(hidden)]
pub fn safety_offset(paths: &crate::clipper::Paths) -> crate::clipper::Paths {
    raw_offset(
        paths,
        CLIPPER_SAFETY_OFFSET,
        crate::clipper::JoinType::jtMiter, // DefaultJoinType
        3.0,                               // DefaultMiterLimit
        crate::clipper::EndType::etClosedPolygon,
    )
}

/// ClipperUtils.cpp:309-322
/// C++: template<class TResult, class TSubj, class TClip> TResult clipper_do(const ClipperLib::ClipType clipType, TSubj && subject, TClip && clip, const ClipperLib::PolyFillType fillType)
///
/// With `TResult = ClipperLib::Paths` this is what the shim runs in
/// `cz_difference_closed` (ctDifference) and `cz_intersection_closed`
/// (ctIntersection), both with pftNonZero.
#[doc(hidden)]
pub fn clipper_do<TResult: ClipperResult>(
    clip_type: crate::clipper::ClipType,
    subject: &crate::clipper::Paths,
    clip: &crate::clipper::Paths,
    fill_type: crate::clipper::PolyFillType,
) -> TResult {
    let mut clipper = crate::clipper::Clipper::new(0);
    clipper.add_paths(subject, crate::clipper::PolyType::ptSubject, true);
    clipper.add_paths(clip, crate::clipper::PolyType::ptClip, true);
    let mut retval = TResult::default();
    TResult::execute(&mut clipper, clip_type, &mut retval, fill_type, fill_type);
    retval
}

/// ClipperUtils.cpp:324-337
/// C++: template<class TResult, class TSubj, class TClip> TResult clipper_do(const ClipperLib::ClipType clipType, TSubj && subject, TClip && clip, const ClipperLib::PolyFillType fillType, const ApplySafetyOffset do_safety_offset)
///
/// With `ApplySafetyOffset::Yes` and `TResult = ClipperLib::Paths` this is what
/// the shim runs in `cz_difference_closed_safety` (ctDifference) and
/// `cz_intersection_closed_safety` (ctIntersection).
#[doc(hidden)]
pub fn clipper_do_safety_offset<TResult: ClipperResult>(
    clip_type: crate::clipper::ClipType,
    subject: &crate::clipper::Paths,
    clip: &crate::clipper::Paths,
    fill_type: crate::clipper::PolyFillType,
    do_safety_offset: ApplySafetyOffset,
) -> TResult {
    // Safety offset only allowed on intersection and difference.
    debug_assert!(do_safety_offset == ApplySafetyOffset::No || clip_type != crate::clipper::ClipType::ctUnion);
    if do_safety_offset == ApplySafetyOffset::Yes {
        clipper_do::<TResult>(clip_type, subject, &safety_offset(clip), fill_type)
    } else {
        clipper_do::<TResult>(clip_type, subject, clip, fill_type)
    }
}

/// ClipperUtils.cpp:339-350
/// C++: template<class TResult, class TSubj> TResult clipper_union(TSubj && subject, const ClipperLib::PolyFillType fillType = ClipperLib::pftNonZero)
///
/// With `TResult = ClipperLib::Paths` and pftNonZero this is the shim's
/// `cz_union_flat`; with `TResult = ClipperLib::PolyTree`, followed by
/// [`poly_tree_to_ex_polygons`], its `cz_union_polytree`.
#[doc(hidden)]
pub fn clipper_union<TResult: ClipperResult>(
    subject: &crate::clipper::Paths,
    // fillType pftNonZero and pftPositive "should" produce the same result for "normalized with implicit union" set of polygons
    fill_type: crate::clipper::PolyFillType,
) -> TResult {
    let mut clipper = crate::clipper::Clipper::new(0);
    clipper.add_paths(subject, crate::clipper::PolyType::ptSubject, true);
    let mut retval = TResult::default();
    TResult::execute(&mut clipper, crate::clipper::ClipType::ctUnion, &mut retval, fill_type, fill_type);
    retval
}

/// ClipperUtils.cpp:352-357
/// C++: ExPolygons ClipperPaths_to_Slic3rExPolygons(const ClipperLib::Paths &input, bool do_union)
///
/// Perform union of input polygons using the positive rule, convert to ExPolygons.
/// `do_union` defaults to false (ClipperUtils.hpp:337): pftEvenOdd.
#[doc(hidden)]
pub fn clipper_paths_to_slic3r_ex_polygons(input: &crate::clipper::Paths, do_union: bool) -> ExPolygons {
    //FIXME is there any benefit of not doing the boolean / using pftEvenOdd?
    poly_tree_to_ex_polygons(&clipper_union::<crate::clipper::PolyTree>(
        input,
        if do_union { crate::clipper::PolyFillType::pftNonZero } else { crate::clipper::PolyFillType::pftEvenOdd },
    ))
}

/// ClipperUtils.cpp:359-364
/// C++: template<typename PathsProvider> static ClipperLib::Paths raw_offset_polyline(PathsProvider &&paths, float offset, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType end_type = ClipperLib::etOpenButt)
#[doc(hidden)]
pub fn raw_offset_polyline(
    paths: &crate::clipper::Paths,
    offset: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
    end_type: crate::clipper::EndType,
) -> crate::clipper::Paths {
    debug_assert!(offset > 0.0);
    raw_offset(paths, offset, join_type, miter_limit, end_type)
}

/// ClipperUtils.cpp:366-372
/// C++: template<class TResult, typename PathsProvider> static TResult expand_paths(PathsProvider &&paths, float offset, ClipperLib::JoinType joinType, double miterLimit)
#[doc(hidden)]
pub fn expand_paths<TResult: ClipperResult>(
    paths: &crate::clipper::Paths,
    offset: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> TResult {
    // BBS
    //assert(offset > 0);
    clipper_union::<TResult>(
        &raw_offset(paths, offset, join_type, miter_limit, crate::clipper::EndType::etClosedPolygon),
        crate::clipper::PolyFillType::pftNonZero,
    )
}

/// ClipperUtils.cpp:381-397
/// C++: template<class TResult, typename PathsProvider> static TResult shrink_paths(PathsProvider &&paths, float offset, ClipperLib::JoinType joinType, double miterLimit)
#[doc(hidden)]
pub fn shrink_paths<TResult: ClipperResult>(
    paths: &crate::clipper::Paths,
    offset: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> TResult {
    // BBS
    //assert(offset > 0);
    let mut out = TResult::default();
    let raw = raw_offset(paths, -offset, join_type, miter_limit, crate::clipper::EndType::etClosedPolygon);
    if !raw.is_empty() {
        let mut clipper = crate::clipper::Clipper::new(0);
        clipper.add_paths(&raw, crate::clipper::PolyType::ptSubject, true);
        let r = clipper.get_bounds();
        clipper.add_path(
            &vec![
                crate::clipper::IntPoint::new(r.left.wrapping_sub(10), r.bottom.wrapping_add(10)),
                crate::clipper::IntPoint::new(r.right.wrapping_add(10), r.bottom.wrapping_add(10)),
                crate::clipper::IntPoint::new(r.right.wrapping_add(10), r.top.wrapping_sub(10)),
                crate::clipper::IntPoint::new(r.left.wrapping_sub(10), r.top.wrapping_sub(10)),
            ],
            crate::clipper::PolyType::ptSubject,
            true,
        );
        clipper.set_reverse_solution(true);
        TResult::execute(
            &mut clipper,
            crate::clipper::ClipType::ctUnion,
            &mut out,
            crate::clipper::PolyFillType::pftNegative,
            crate::clipper::PolyFillType::pftNegative,
        );
        out.remove_outermost_polygon();
    }
    out
}

/// ClipperUtils.cpp:399-408
/// C++: template<class TResult, typename PathsProvider> static TResult offset_paths(PathsProvider &&paths, float offset, ClipperLib::JoinType joinType, double miterLimit)
///
/// With `TResult = ClipperLib::Paths` this is the shim's `cz_offset_paths`
/// (`offset(const Polygons&, float delta)`, ClipperUtils.cpp:413); with
/// `TResult = ClipperLib::PolyTree`, followed by [`poly_tree_to_ex_polygons`], its
/// `cz_offset_ex_paths` (`offset_ex(const Polygons&, float delta)`,
/// ClipperUtils.cpp:415).
#[doc(hidden)]
pub fn offset_paths<TResult: ClipperResult>(
    paths: &crate::clipper::Paths,
    offset: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> TResult {
    // BBS
    //assert(offset != 0);

    if offset > 0.0 {
        expand_paths::<TResult>(paths, offset, join_type, miter_limit)
    } else {
        shrink_paths::<TResult>(paths, -offset, join_type, miter_limit)
    }
}

/// ClipperUtils.cpp:418-419
/// C++: Slic3r::Polygons offset(const Slic3r::Polyline &polyline, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType end_type)
///
/// The shim's `cz_offset_polyline`, which fixes `joinType` to jtSquare
/// (DefaultLineJoinType) and `end_type` to etOpenButt (DefaultEndType), returns
/// nothing for a polyline of fewer than two points, and skips the union of an
/// empty offset.
#[doc(hidden)]
pub fn offset_polyline_paths(polyline: &crate::clipper::Path, delta: f64, miter_limit: f64) -> crate::clipper::Paths {
    if polyline.len() < 2 {
        return crate::clipper::Paths::new();
    }
    let raw = raw_offset_polyline(
        &vec![polyline.clone()],
        delta,
        crate::clipper::JoinType::jtSquare,
        miter_limit,
        crate::clipper::EndType::etOpenButt,
    );
    if raw.is_empty() {
        return crate::clipper::Paths::new();
    }
    clipper_union::<crate::clipper::Paths>(&raw, crate::clipper::PolyFillType::pftNonZero)
}

/// ClipperUtils.cpp:418-419
/// C++: Slic3r::Polygons offset(const Slic3r::Polyline &polyline, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType end_type)
///
/// As the C++ has it, unlike [`offset_polyline_paths`] (the shim's routine): any
/// join type and end type, and no early return for a polyline of fewer than two
/// points (ClipperOffset draws a square or a circle around a single point).
/// `delta` is the f64 of the `float` parameter; the C++ asserts it positive.
#[doc(hidden)]
pub fn offset_polyline_path(
    polyline: &crate::clipper::Path,
    delta: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
    end_type: crate::clipper::EndType,
) -> crate::clipper::Paths {
    debug_assert!(delta > 0.0);
    clipper_union::<crate::clipper::Paths>(
        &raw_offset_polyline(&vec![polyline.clone()], delta, join_type, miter_limit, end_type),
        crate::clipper::PolyFillType::pftNonZero,
    )
}

/// ClipperUtils.cpp:436-501
/// C++: static int offset_expolygon_inner(const Slic3r::ExPolygon &expoly, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::Paths &out)
///
/// returns number of expolygons collected (0 or 1).
#[doc(hidden)]
pub fn offset_expolygon_inner(
    contour: &crate::clipper::Path,
    expoly_holes: &crate::clipper::Paths,
    delta: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
    out: &mut crate::clipper::Paths,
) -> i32 {
    // 1) Offset the outer contour.
    let mut contours = crate::clipper::Paths::new();
    {
        let mut co = crate::clipper::ClipperOffset::default();
        if join_type == crate::clipper::JoinType::jtRound {
            co.arc_tolerance = miter_limit;
        } else {
            co.miter_limit = miter_limit;
        }
        co.shortest_edge_length = (delta * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR).abs();
        co.add_path(contour, join_type, crate::clipper::EndType::etClosedPolygon);
        co.execute(&mut contours, delta);
    }
    if contours.is_empty() {
        // No need to try to offset the holes.
        return 0;
    }

    if expoly_holes.is_empty() {
        // No need to subtract holes from the offsetted expolygon, we are done.
        out.append(&mut contours);
    } else {
        // 2) Offset the holes one by one, collect the offsetted holes.
        let mut holes = crate::clipper::Paths::new();
        {
            for hole in expoly_holes {
                let mut co = crate::clipper::ClipperOffset::default();
                if join_type == crate::clipper::JoinType::jtRound {
                    co.arc_tolerance = miter_limit;
                } else {
                    co.miter_limit = miter_limit;
                }
                co.shortest_edge_length = (delta * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR).abs();
                co.add_path(hole, join_type, crate::clipper::EndType::etClosedPolygon);
                let mut out2 = crate::clipper::Paths::new();
                // Execute reorients the contours so that the outer most contour has a positive area. Thus the output
                // contours will be CCW oriented even though the input paths are CW oriented.
                // Offset is applied after contour reorientation, thus the signum of the offset value is reversed.
                co.execute(&mut out2, -delta);
                holes.append(&mut out2);
            }
        }

        // 3) Subtract holes from the contours.
        if holes.is_empty() {
            // No hole remaining after an offset. Just copy the outer contour.
            out.append(&mut contours);
        } else if delta < 0.0 {
            // Negative offset. There is a chance, that the offsetted hole intersects the outer contour.
            // Subtract the offsetted holes from the offsetted contours.
            let mut output = clipper_do::<crate::clipper::Paths>(
                crate::clipper::ClipType::ctDifference,
                &contours,
                &holes,
                crate::clipper::PolyFillType::pftNonZero,
            );
            if !output.is_empty() {
                out.append(&mut output);
            } else {
                // The offsetted holes have eaten up the offsetted outer contour.
                return 0;
            }
        } else {
            // Positive offset. As long as the Clipper offset does what one expects it to do, the offsetted hole will have a smaller
            // area than the original hole or even disappear, therefore there will be no new intersections.
            // Just collect the reversed holes.
            out.reserve(contours.len() + holes.len());
            out.append(&mut contours);
            // Reverse the holes in place.
            for hole in holes.iter_mut() {
                hole.reverse();
            }
            out.append(&mut holes);
        }
    }

    1
}

/// ClipperUtils.cpp:508-513
/// C++: ClipperLib::Paths expolygon_offset(const Slic3r::ExPolygon &expolygon, const float delta, ClipperLib::JoinType joinType, double miterLimit)
///
/// The shim's `cz_offset_expolygon`.
#[doc(hidden)]
pub fn expolygon_offset(
    contour: &crate::clipper::Path,
    holes: &crate::clipper::Paths,
    delta: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> crate::clipper::Paths {
    let mut out = crate::clipper::Paths::new();
    offset_expolygon_inner(contour, holes, delta, join_type, miter_limit, &mut out);
    out
}

/// ClipperUtils.cpp:515-531
/// C++: template<typename ExPolygonVector> static std::pair<ClipperLib::Paths, size_t> expolygons_offset_raw(const ExPolygonVector &expolygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)
///
/// An expolygon is its contour and its holes.
#[doc(hidden)]
pub fn expolygons_offset_raw(
    expolygons: &[(crate::clipper::Path, crate::clipper::Paths)],
    delta: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> (crate::clipper::Paths, usize) {
    // Offsetted ExPolygons before they are united.
    let mut output = crate::clipper::Paths::new();
    output.reserve(expolygons.len());
    // How many non-empty offsetted expolygons were actually collected into output?
    // If only one, then there is no need to do a final union.
    let mut expolygons_collected = 0usize;
    for (contour, holes) in expolygons {
        expolygons_collected += offset_expolygon_inner(contour, holes, delta, join_type, miter_limit, &mut output) as usize;
    }
    (output, expolygons_collected)
}

/// ClipperUtils.cpp:533-545
/// C++: template<typename ExPolygonVector> static ClipperLib::Paths expolygons_offset(const ExPolygonVector &expolygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)
///
/// See comment on expolygon_offsets_raw. In addition, for positive offset the contours are united.
#[doc(hidden)]
pub fn expolygons_offset(
    expolygons: &[(crate::clipper::Path, crate::clipper::Paths)],
    delta: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> crate::clipper::Paths {
    let (output, expolygons_collected) = expolygons_offset_raw(expolygons, delta, join_type, miter_limit);
    // Unite the offsetted expolygons.
    if expolygons_collected > 1 && delta > 0.0 {
        // There is a chance that the outwards offsetted expolygons may intersect. Perform a union.
        clipper_union::<crate::clipper::Paths>(&output, crate::clipper::PolyFillType::pftNonZero)
    } else {
        // Negative offset. The shrunk expolygons shall not mutually intersect. Just copy the output.
        output
    }
}

/// ClipperUtils.cpp:550-557
/// C++: template<typename ExPolygonVector> static ClipperLib::PolyTree expolygons_offset_pt(const ExPolygonVector &expolygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)
///
/// See comment on expolygons_offset_raw. In addition, the polygons are always united to conver to polytree.
#[doc(hidden)]
pub fn expolygons_offset_pt(
    expolygons: &[(crate::clipper::Path, crate::clipper::Paths)],
    delta: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> crate::clipper::PolyTree {
    let (output, _expolygons_collected) = expolygons_offset_raw(expolygons, delta, join_type, miter_limit);
    // Unite the offsetted expolygons for both the
    clipper_union::<crate::clipper::PolyTree>(&output, crate::clipper::PolyFillType::pftNonZero)
}

/// ClipperUtils.cpp:567-569
/// C++: Slic3r::ExPolygons offset_ex(const Slic3r::ExPolygon &expolygon, const float delta, ClipperLib::JoinType joinType, double miterLimit)
#[doc(hidden)]
pub fn offset_ex_expolygon_paths(
    contour: &crate::clipper::Path,
    holes: &crate::clipper::Paths,
    delta: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> ExPolygons {
    //FIXME one may spare one Clipper Union call.
    clipper_paths_to_slic3r_ex_polygons(&expolygon_offset(contour, holes, delta, join_type, miter_limit), false)
}

/// ClipperUtils.cpp:570-571
/// C++: Slic3r::ExPolygons offset_ex(const Slic3r::ExPolygons &expolygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)
#[doc(hidden)]
pub fn offset_ex_expolygons_paths(
    expolygons: &[(crate::clipper::Path, crate::clipper::Paths)],
    delta: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> ExPolygons {
    poly_tree_to_ex_polygons(&expolygons_offset_pt(expolygons, delta, join_type, miter_limit))
}

/// ClipperUtils.cpp:581-584
/// C++: ExPolygons offset2_ex(const ExPolygons &expolygons, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)
///
/// The shim's `cz_offset2_ex`. `xy`, `lens` and `is_hole` are the layout the
/// wrapper builds for it: a path with `is_hole == 0` starts an expolygon, the
/// others are the holes of the expolygon before them.
#[doc(hidden)]
pub fn offset2_ex_paths(
    xy: &[i32],
    lens: &[i32],
    is_hole: &[i32],
    delta1: f64,
    delta2: f64,
    join_type: crate::clipper::JoinType,
    miter_limit: f64,
) -> ExPolygons {
    let mut expolys: Vec<(crate::clipper::Path, crate::clipper::Paths)> = Vec::new();
    for (p, path) in clib_read_closed_paths(xy, lens).into_iter().enumerate() {
        if is_hole[p] == 0 {
            expolys.push((path, crate::clipper::Paths::new()));
        } else if let Some(last) = expolys.last_mut() {
            last.1.push(path);
        }
    }
    poly_tree_to_ex_polygons(&offset_paths::<crate::clipper::PolyTree>(
        &expolygons_offset(&expolys, delta1, join_type, miter_limit),
        delta2,
        join_type,
        miter_limit,
    ))
}

/// ClipperUtils.cpp:636-654
/// C++: template<typename PathProvider1, typename PathProvider2> inline ClipperLib::PolyTree clipper_do_polytree(const ClipperLib::ClipType clipType, PathProvider1 &&subject, PathProvider2 &&clip, const ClipperLib::PolyFillType fillType)
///
/// 1) Peform the Clipper operation with the output to Paths. This method handles overlaps in a reasonable time.
/// 2) Run Clipper Union once again to extract the PolyTree from the result of 1).
///
/// With ctUnion and no clip, followed by [`poly_tree_to_ex_polygons`], this is the
/// shim's `cz_union_ex` (`union_ex(const Polygons&, PolyFillType)`,
/// ClipperUtils.cpp:813-814).
#[doc(hidden)]
pub fn clipper_do_polytree(
    clip_type: crate::clipper::ClipType,
    subject: &crate::clipper::Paths,
    clip: &crate::clipper::Paths,
    fill_type: crate::clipper::PolyFillType,
) -> crate::clipper::PolyTree {
    // Perform the operation with the output to input_subject.
    // This pass does not generate a PolyTree, which is a very expensive operation with the current Clipper library
    // if there are overapping edges.
    let output = clipper_do::<crate::clipper::Paths>(clip_type, subject, clip, fill_type);
    if !output.is_empty() {
        // Perform an additional Union operation to generate the PolyTree ordering.
        return clipper_union::<crate::clipper::PolyTree>(&output, fill_type);
    }
    crate::clipper::PolyTree::new()
}

/// ClipperUtils.cpp:655-667
/// C++: template<typename PathProvider1, typename PathProvider2> inline ClipperLib::PolyTree clipper_do_polytree(const ClipperLib::ClipType clipType, PathProvider1 &&subject, PathProvider2 &&clip, const ClipperLib::PolyFillType fillType, const ApplySafetyOffset do_safety_offset)
#[doc(hidden)]
pub fn clipper_do_polytree_safety_offset(
    clip_type: crate::clipper::ClipType,
    subject: &crate::clipper::Paths,
    clip: &crate::clipper::Paths,
    fill_type: crate::clipper::PolyFillType,
    do_safety_offset: ApplySafetyOffset,
) -> crate::clipper::PolyTree {
    debug_assert!(do_safety_offset == ApplySafetyOffset::No || clip_type != crate::clipper::ClipType::ctUnion);
    if do_safety_offset == ApplySafetyOffset::Yes {
        clipper_do_polytree(clip_type, subject, &safety_offset(clip), fill_type)
    } else {
        clipper_do_polytree(clip_type, subject, clip, fill_type)
    }
}

/// ClipperUtils.cpp:737-739
/// C++: template <typename TSubject, typename TClip> static ExPolygons _clipper_ex(ClipperLib::ClipType clipType, TSubject &&subject, TClip &&clip, ApplySafetyOffset do_safety_offset, ClipperLib::PolyFillType fill_type = ClipperLib::pftNonZero)
///
/// What `diff_ex`, `intersection_ex` and `xor_ex` of ExPolygons run
/// (ClipperUtils.cpp:755-756, 802-803, 836-838).
#[doc(hidden)]
pub fn _clipper_ex(
    clip_type: crate::clipper::ClipType,
    subject: &crate::clipper::Paths,
    clip: &crate::clipper::Paths,
    do_safety_offset: ApplySafetyOffset,
    fill_type: crate::clipper::PolyFillType,
) -> ExPolygons {
    poly_tree_to_ex_polygons(&clipper_do_polytree_safety_offset(clip_type, subject, clip, fill_type, do_safety_offset))
}

/// The shim's `cz_union_ex_safety`, which stands for
/// `union_safety_offset_ex(const Polygons&)` (ClipperUtils.hpp:372): the subject
/// goes through [`safety_offset`] and then through the two unions of
/// [`clipper_do_polytree`].
///
/// FIDELITY-NOTE: the C++ `union_safety_offset_ex` is `offset_ex(polygons,
/// ClipperSafetyOffset)`, that is [`offset_paths`] into a PolyTree: one union
/// after the raw offset, not two. This follows the shim, so the output of the
/// engine does not change with the gate.
#[doc(hidden)]
pub fn union_safety_offset_ex_paths(subject: &crate::clipper::Paths) -> ExPolygons {
    poly_tree_to_ex_polygons(&clipper_do_polytree(
        crate::clipper::ClipType::ctUnion,
        &safety_offset(subject),
        &crate::clipper::Paths::new(),
        crate::clipper::PolyFillType::pftNonZero,
    ))
}

/// ClipperUtils.cpp:838-847
/// C++: template<typename PathsProvider1, typename PathsProvider2> Polylines _clipper_pl_open(ClipperLib::ClipType clipType, PathsProvider1 &&subject, PathsProvider2 &&clip)
///
/// The shim's `cz_intersection_pl` (ctIntersection) and `cz_diff_pl`
/// (ctDifference).
#[doc(hidden)]
pub fn _clipper_pl_open(
    clip_type: crate::clipper::ClipType,
    subject: &crate::clipper::Paths,
    clip: &crate::clipper::Paths,
) -> crate::clipper::Paths {
    let mut clipper = crate::clipper::Clipper::new(0);
    clipper.add_paths(subject, crate::clipper::PolyType::ptSubject, false);
    clipper.add_paths(clip, crate::clipper::PolyType::ptClip, true);
    let mut retval = crate::clipper::PolyTree::new();
    clipper.execute_poly_tree(
        clip_type,
        &mut retval,
        crate::clipper::PolyFillType::pftNonZero,
        crate::clipper::PolyFillType::pftNonZero,
    );
    poly_tree_to_polylines(&retval)
}

/// ClipperUtils.cpp:1077-1093
/// C++: ClipperLib::Paths fix_after_outer_offset(const ClipperLib::Path &input, ClipperLib::PolyFillType filltype, bool reverse_result)
///
/// Outer offset shall not split the input contour into multiples. It is expected, that the solution will be non empty and it will contain just a single polygon.
#[doc(hidden)]
pub fn fix_after_outer_offset(
    input: &crate::clipper::Path,
    filltype: crate::clipper::PolyFillType,
    reverse_result: bool,
) -> crate::clipper::Paths {
    let mut solution = crate::clipper::Paths::new();
    if !input.is_empty() {
        let mut clipper = crate::clipper::Clipper::new(0);
        clipper.add_path(input, crate::clipper::PolyType::ptSubject, true);
        clipper.set_reverse_solution(reverse_result);
        clipper.execute(crate::clipper::ClipType::ctUnion, &mut solution, filltype, filltype);
    }
    solution
}

/// ClipperUtils.cpp:1095-1118
/// C++: ClipperLib::Paths fix_after_inner_offset(const ClipperLib::Path &input, ClipperLib::PolyFillType filltype, bool reverse_result)
///
/// Inner offset may split the source contour into multiple contours, but one resulting contour shall not lie inside the other.
#[doc(hidden)]
pub fn fix_after_inner_offset(
    input: &crate::clipper::Path,
    filltype: crate::clipper::PolyFillType,
    reverse_result: bool,
) -> crate::clipper::Paths {
    let mut solution = crate::clipper::Paths::new();
    if !input.is_empty() {
        let mut clipper = crate::clipper::Clipper::new(0);
        clipper.add_path(input, crate::clipper::PolyType::ptSubject, true);
        let mut r = clipper.get_bounds();
        r.left = r.left.wrapping_sub(10);
        r.top = r.top.wrapping_sub(10);
        r.right = r.right.wrapping_add(10);
        r.bottom = r.bottom.wrapping_add(10);
        if filltype == crate::clipper::PolyFillType::pftPositive {
            clipper.add_path(
                &vec![
                    crate::clipper::IntPoint::new(r.left, r.bottom),
                    crate::clipper::IntPoint::new(r.left, r.top),
                    crate::clipper::IntPoint::new(r.right, r.top),
                    crate::clipper::IntPoint::new(r.right, r.bottom),
                ],
                crate::clipper::PolyType::ptSubject,
                true,
            );
        } else {
            clipper.add_path(
                &vec![
                    crate::clipper::IntPoint::new(r.left, r.bottom),
                    crate::clipper::IntPoint::new(r.right, r.bottom),
                    crate::clipper::IntPoint::new(r.right, r.top),
                    crate::clipper::IntPoint::new(r.left, r.top),
                ],
                crate::clipper::PolyType::ptSubject,
                true,
            );
        }
        clipper.set_reverse_solution(reverse_result);
        clipper.execute(crate::clipper::ClipType::ctUnion, &mut solution, filltype, filltype);
        if !solution.is_empty() {
            solution.remove(0);
        }
    }
    solution
}

/// ClipperUtils.cpp:1120-1258
/// C++: ClipperLib::Path mittered_offset_path_scaled(const Points &contour, const std::vector<float> &deltas, double miter_limit)
///
/// As the shim's `vo::mittered_offset_path_scaled` has it: the points and the
/// normals as pairs of doubles instead of `Vec2d`, the number of deltas checked
/// instead of asserted equal to the number of points. A product that the C++
/// adds to or subtracts from another term in the same expression is fused into
/// that sum (`mul_add`), as clang compiles the shim (`-ffp-contract=on`, its
/// default) for a target with a fused multiply-add.
///
/// [`crate::clipper2_utils`] has the same function for the Clipper2 paths of the
/// other variable offsets; that one does not fuse.
#[doc(hidden)]
pub fn mittered_offset_path_scaled(
    contour: &crate::clipper::Path,
    deltas: &[f32],
    mut miter_limit: f64,
) -> crate::clipper::Path {
    let mut out = crate::clipper::Path::new();
    if deltas.len() > 2 {
        out.reserve(contour.len() * 2);

        // Clamp miter limit to 2.
        miter_limit = if miter_limit > 2.0 { 2.0 / (miter_limit * miter_limit) } else { 0.5 };

        // Add a new point to the output, round to cInt.
        let add_offset_point = |out: &mut crate::clipper::Path, mut x: f64, mut y: f64| {
            x += 0.5 - ((x < 0.0) as i32 as f64);
            y += 0.5 - ((y < 0.0) as i32 as f64);
            out.push(crate::clipper::IntPoint::new(x as crate::clipper::cInt, y as crate::clipper::cInt));
        };
        let pt_of = |p: &crate::clipper::IntPoint| -> (f64, f64) { (p.x as f64, p.y as f64) };

        // Minimum edge length, squared.
        // `*std::max_element(deltas.begin(), deltas.end())`: the first of the largest.
        let mut largest = deltas[0];
        for &d in &deltas[1..] {
            if largest < d {
                largest = d;
            }
        }
        let lmin = largest as f64 * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR;
        let l2min = lmin * lmin;
        // Minimum angle to consider two edges to be parallel.
        // Implementation equal to Clipper.
        let sin_min_parallel = 1.0_f64;

        // Find the last point further from pt by l2min.
        let (mut ptx, mut pty) = pt_of(&contour[0]);
        let mut iprev = contour.len() - 1;
        let (mut ppx, mut ppy) = (0.0_f64, 0.0_f64);
        while iprev > 0 {
            (ppx, ppy) = pt_of(&contour[iprev]);
            let (dx, dy) = (ppx - ptx, ppy - pty);
            if dx.mul_add(dx, dy * dy) > l2min {
                break;
            }
            iprev -= 1;
        }

        if iprev != 0 {
            let ilast = iprev;
            // Normal to the (pt - ptprev) segment.
            let (vx, vy) = (ptx - ppx, pty - ppy);
            let (mut npx, mut npy) = (vy, -vx);
            {
                let n = npx.mul_add(npx, npy * npy).sqrt();
                npx /= n;
                npy /= n;
            }
            let mut i = 0usize;
            loop {
                // Find the next point further from pt by l2min.
                let mut j = i + 1;
                let (mut nxx, mut nxy) = (0.0_f64, 0.0_f64);
                while j <= ilast {
                    (nxx, nxy) = pt_of(&contour[j]);
                    let (dx, dy) = (nxx - ptx, nxy - pty);
                    if dx.mul_add(dx, dy * dy) > l2min {
                        break;
                    }
                    j += 1;
                }
                if j > ilast {
                    // If the last edge is too short, merge it with the previous edge.
                    i = ilast;
                    (nxx, nxy) = pt_of(&contour[0]);
                }

                // Normal to the (ptnext - pt) segment.
                let (wx, wy) = (nxx - ptx, nxy - pty);
                let (mut nnx, mut nny) = (wy, -wx);
                {
                    let n = nnx.mul_add(nnx, nny * nny).sqrt();
                    nnx /= n;
                    nny /= n;
                }

                let delta = deltas[i] as f64;
                // cross2(nprev, nnext): `ax * by - ay * bx`
                let mut sin_a = npx.mul_add(nny, -(npy * nnx));
                if sin_a < -1.0 {
                    sin_a = -1.0;
                } else if sin_a > 1.0 {
                    sin_a = 1.0;
                }
                let convex = sin_a * delta;
                if convex <= -sin_min_parallel {
                    // Concave corner.
                    add_offset_point(&mut out, npx.mul_add(delta, ptx), npy.mul_add(delta, pty));
                    add_offset_point(&mut out, ptx, pty);
                    add_offset_point(&mut out, nnx.mul_add(delta, ptx), nny.mul_add(delta, pty));
                } else {
                    let dot = npx.mul_add(nnx, npy * nny);
                    if convex < sin_min_parallel && dot > 0.0 {
                        // Nearly parallel.
                        if dot > 0.0 {
                            add_offset_point(&mut out, npx.mul_add(delta, ptx), npy.mul_add(delta, pty));
                        } else {
                            add_offset_point(&mut out, ptx, pty);
                        }
                    } else {
                        // Convex corner.
                        let r = 1.0 + dot;
                        if r >= miter_limit {
                            add_offset_point(
                                &mut out,
                                (npx + nnx).mul_add(delta / r, ptx),
                                (npy + nny).mul_add(delta / r, pty),
                            );
                        } else {
                            let dx = (sin_a.atan2(dot) / 4.0).tan();
                            // newpt1 = pt + (nprev - perp(nprev) * dx) * delta, perp(nprev) = (npy, -npx)
                            add_offset_point(
                                &mut out,
                                (-npy).mul_add(dx, npx).mul_add(delta, ptx),
                                npx.mul_add(dx, npy).mul_add(delta, pty),
                            );
                            // newpt2 = pt + (nnext + perp(nnext) * dx) * delta, perp(nnext) = (nny, -nnx)
                            add_offset_point(
                                &mut out,
                                nny.mul_add(dx, nnx).mul_add(delta, ptx),
                                (-nnx).mul_add(dx, nny).mul_add(delta, pty),
                            );
                        }
                    }
                }

                if i == ilast {
                    break;
                }

                (ppx, ppy) = (ptx, pty);
                (npx, npy) = (nnx, nny);
                (ptx, pty) = (nxx, nxy);
                i = j;
            }
            let _ = (ppx, ppy);
        }
    }
    out
}

/// ClipperUtils.cpp:1390-1440
/// C++: ExPolygons variable_offset_inner_ex(const ExPolygon &expoly, const std::vector<std::vector<float>> &deltas, double miter_limit)
///
/// The shim's `cz_variable_offset_inner_ex`. `rings` is the contour followed by
/// the holes, `deltas` the offsets of all their points one ring after the other.
#[doc(hidden)]
pub fn variable_offset_inner_ex_paths(rings: &crate::clipper::Paths, deltas: &[f32], miter_limit: f64) -> ExPolygons {
    let mut d = 0usize;
    // 1) Offset the outer contour.
    let contours = fix_after_inner_offset(
        &mittered_offset_path_scaled(&rings[0], &deltas[d..d + rings[0].len()], miter_limit),
        crate::clipper::PolyFillType::pftNegative,
        true,
    );
    d += rings[0].len();

    // 2) Offset the holes one by one, collect the results.
    let mut holes = crate::clipper::Paths::new();
    for hole in &rings[1..] {
        holes.append(&mut fix_after_outer_offset(
            &mittered_offset_path_scaled(hole, &deltas[d..d + hole.len()], miter_limit),
            crate::clipper::PolyFillType::pftNegative,
            false,
        ));
        d += hole.len();
    }

    // 3) Subtract holes from the contours.
    if holes.is_empty() {
        contours
            .iter()
            .map(|path| ExPolygon::new(Polygon::from_points(path.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect())))
            .collect()
    } else {
        let mut clipper = crate::clipper::Clipper::new(0);
        clipper.add_paths(&contours, crate::clipper::PolyType::ptSubject, true);
        clipper.add_paths(&holes, crate::clipper::PolyType::ptClip, true);
        let mut polytree = crate::clipper::PolyTree::new();
        clipper.execute_poly_tree(
            crate::clipper::ClipType::ctDifference,
            &mut polytree,
            crate::clipper::PolyFillType::pftNonZero,
            crate::clipper::PolyFillType::pftNonZero,
        );
        poly_tree_to_ex_polygons(&polytree)
    }
}

/// Faithful `offset_expolygon_inner` (ClipperUtils.cpp:437) for a single
/// ExPolygon via the vertex-exact ClipperLib shim. `delta` is in mm (positive =
/// grow, negative = shrink); coords/delta are scaled to i64@1e5 then narrowed to
/// i32. Returns the offset paths as scaled Polygons (contours + holes, not yet
/// reconstructed into ExPolygons). Empty contour offset yields an empty Vec.
fn clib_offset_expolygon_paths_prescaled(
    expoly: &ExPolygon,
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
) -> Vec<Polygon> {
    let contour_xy: Vec<i32> = expoly
        .contour
        .points()
        .iter()
        .flat_map(|p| {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            [p.x as i32, p.y as i32]
        })
        .collect();
    let contour_n = expoly.contour.points().len() as i32;
    if contour_n == 0 {
        return Vec::new();
    }
    let mut holes_xy: Vec<i32> = Vec::new();
    let mut hole_lens: Vec<i32> = Vec::with_capacity(expoly.holes.len());
    for hole in &expoly.holes {
        hole_lens.push(hole.points().len() as i32);
        for p in hole.points() {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            holes_xy.push(p.x as i32);
            holes_xy.push(p.y as i32);
        }
    }
    clib_offset_expolygon_paths_scaled(expoly, delta_scaled, join_type, contour_xy, contour_n, holes_xy, hole_lens)
}

fn clib_offset_expolygon_paths(
    expoly: &ExPolygon,
    delta_mm: CoordF,
    join_type: OffsetJoinType,
) -> Vec<Polygon> {
    let contour_xy: Vec<i32> = expoly
        .contour
        .points()
        .iter()
        .flat_map(|p| {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            [p.x as i32, p.y as i32]
        })
        .collect();
    let contour_n = expoly.contour.points().len() as i32;
    if contour_n == 0 {
        return Vec::new();
    }

    let mut holes_xy: Vec<i32> = Vec::new();
    let mut hole_lens: Vec<i32> = Vec::with_capacity(expoly.holes.len());
    for hole in &expoly.holes {
        hole_lens.push(hole.points().len() as i32);
        for p in hole.points() {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            holes_xy.push(p.x as i32);
            holes_xy.push(p.y as i32);
        }
    }

    let delta_scaled = delta_mm * crate::SCALING_FACTOR;
    clib_offset_expolygon_paths_scaled(expoly, delta_scaled, join_type, contour_xy, contour_n, holes_xy, hole_lens)
}

/// Scaled-delta core of [`clib_offset_expolygon_paths`] — callers that must
/// reproduce native's exact delta posture (e.g. `float(scale_(f32_series))`,
/// PerimeterGenerator.cpp:1871) pass the final f64 delta directly.
#[allow(clippy::too_many_arguments)]
fn clib_offset_expolygon_paths_scaled(
    expoly: &ExPolygon,
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
    contour_xy: Vec<i32>,
    contour_n: i32,
    holes_xy: Vec<i32>,
    hole_lens: Vec<i32>,
) -> Vec<Polygon> {
    let _ = expoly;

    // The shim reads the contour, and the holes when there are any.
    let contour = clib_read_closed_paths(&contour_xy, &[contour_n]).swap_remove(0);
    let holes = if expoly.holes.is_empty() {
        crate::clipper::Paths::new()
    } else {
        clib_read_closed_paths(&holes_xy, &hole_lens)
    };
    return clib_polygons_from_paths(&expolygon_offset(
        &contour,
        &holes,
        delta_scaled,
        clib_join_type(clib_join_code(join_type)),
        CLIB_MITER_LIMIT,
    ));
}

/// ClipperLib-backed `expolygons_offset` (ClipperUtils.cpp:539): vertex-exact
/// per-ExPolygon offset, then union the resulting paths back into ExPolygons.
///
/// Mirrors `offset_ex(const ExPolygons&)` / the `offset_polygons` single offset
/// for the perimeter inner-wall path. `delta` is in mm. The cross-ExPolygon
/// union runs the vertex-exact `polytree_ex_clib` (ClipperLib @1e5, full-res).
/// Scaled-delta variant of [`offset_expolygons_clib`]: `delta_scaled` is the
/// FINAL f64 delta handed to ClipperOffset (native posture already applied,
/// e.g. `double(float(scale_(f32)))`).
pub fn offset_expolygons_clib_scaled(
    expolygons: &[ExPolygon],
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset_expolygons_clib_impl(expolygons, delta_scaled, true, join_type)
}

pub fn offset_expolygons_clib(
    expolygons: &[ExPolygon],
    delta: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset_expolygons_clib_impl(expolygons, delta, false, join_type)
}

/// RAW variant matching native `ClipperUtils::offset(ExPolygons, float delta)`
/// EXACTLY: per-expolygon ClipperOffset paths concatenated as flat `Polygons` —
/// NO union / PolyTree reconstruction (Class-5: the union re-merges overlapping
/// grown islands native keeps separate). `delta_scaled` = final f64 delta.
pub fn offset_expolygons_clib_raw_scaled(
    expolygons: &[ExPolygon],
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
) -> Vec<Polygon> {
    offset_expolygons_clib_raw_scaled_counted(expolygons, delta_scaled, join_type).0
}

/// As [`offset_expolygons_clib_raw_scaled`], also returning how many expolygons
/// contributed non-empty output (native `expolygons_collected` —
/// ClipperUtils.cpp:523-534; the >1 && delta>0 union trigger).
pub fn offset_expolygons_clib_raw_scaled_counted(
    expolygons: &[ExPolygon],
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
) -> (Vec<Polygon>, usize) {
    let mut out: Vec<Polygon> = Vec::new();
    let mut collected = 0usize;
    for expoly in expolygons {
        let paths = clib_offset_expolygon_paths_prescaled(expoly, delta_scaled, join_type);
        if !paths.is_empty() {
            collected += 1;
        }
        out.extend(paths);
    }
    (out, collected)
}

fn offset_expolygons_clib_impl(
    expolygons: &[ExPolygon],
    delta: CoordF,
    delta_is_scaled: bool,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() {
        return vec![];
    }

    let mut all_paths: Vec<Polygon> = Vec::new();
    for expoly in expolygons {
        if delta_is_scaled {
            all_paths.extend(clib_offset_expolygon_paths_prescaled(expoly, delta, join_type));
        } else {
            all_paths.extend(clib_offset_expolygon_paths(expoly, delta, join_type));
        }
    }
    if all_paths.is_empty() {
        return vec![];
    }

    // expolygons_offset_pt (ClipperUtils.cpp:550-557): one NonZero union into a
    // PolyTree, then PolyTree->ExPolygons. R100: the geo-clipper reconstruction it
    // replaced (F1_UNION=0, removed at R911) snapped every offset vertex to a
    // 1-micron grid — the dominant wall-loop divergence from native's 10nm ClipperLib.
    polytree_ex_clib(&all_paths)
}

/// ClipperLib-backed `shrink` (negative offset) for the perimeter inner-wall path.
pub fn shrink_clib(
    expolygons: &[ExPolygon],
    distance: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset_expolygons_clib(expolygons, -distance.abs(), join_type)
}

/// ClipperLib-backed `grow` (positive offset) for the perimeter inner-wall path.
pub fn grow_clib(
    expolygons: &[ExPolygon],
    distance: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset_expolygons_clib(expolygons, distance.abs(), join_type)
}

/// ClipperLib-backed `offset2_ex` (ClipperUtils.cpp:581): shrink by `shrink_amount`
/// then grow by `grow_amount`, both through the vertex-exact ClipperLib offset.
/// This is the successive inner-wall perimeter-loop operation whose compounding
/// geo-clipper densification this swap removes.
pub fn offset2_clib(
    expolygons: &[ExPolygon],
    shrink_amount: CoordF,
    grow_amount: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if expolygons.is_empty() {
        return vec![];
    }
    let shrunk = shrink_clib(expolygons, shrink_amount.abs(), join_type);
    if shrunk.is_empty() {
        return vec![];
    }
    grow_clib(&shrunk, grow_amount.abs(), join_type)
}

/// R775 — faithful `offset_ex(const Polygons &, float delta)` over the
/// vendored ClipperLib (ClipperUtils.cpp:415): per-path orientation-aware
/// raw_offset + PolyTree reconstruction (NonZero union grow / pftNegative
/// shrink trick). `delta_scaled` is the final SCALED delta; pre-round it to
/// f32 like the C++ float parameter. This replaces the geo-clipper
/// `offset_polygons` (1µm re-gridding) on paths that must stay vertex-exact.
pub fn offset_ex_polygons_clib(
    polygons: &[Polygon],
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset_ex_polygons_clib_ml(polygons, delta_scaled, join_type, CLIB_MITER_LIMIT)
}

/// [`offset_ex_polygons_clib`] with the `miterLimit` argument of the C++
/// `offset_ex(const Polygons&, ...)` (ClipperUtils.cpp:415-416) spelled out: the
/// unitless miter limit for jtMiter, the arc tolerance in scaled units for jtRound.
pub fn offset_ex_polygons_clib_ml(
    polygons: &[Polygon],
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
    miter_limit: f64,
) -> ExPolygons {
    if polygons.is_empty() {
        return vec![];
    }
    let (xy, lens, num) = clib_flatten_polygons(polygons);
    if num == 0 {
        return vec![];
    }
    let jt: i32 = match join_type {
        OffsetJoinType::Miter => 0,
        OffsetJoinType::Round => 1,
        OffsetJoinType::Square => 2,
    };
    return poly_tree_to_ex_polygons(&offset_paths::<crate::clipper::PolyTree>(
        &clib_read_closed_paths(&xy, &lens),
        delta_scaled,
        clib_join_type(jt),
        miter_limit,
    ));
}

/// The points of a ring or a polyline as a `ClipperLib::Path` (cInt = i32).
fn clib_path_of_points(points: &[Point]) -> crate::clipper::Path {
    points
        .iter()
        .map(|p| {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            crate::clipper::IntPoint::new(p.x as i32, p.y as i32)
        })
        .collect()
}

/// `ClipperUtils::ExPolygonsProvider` (ClipperUtils.hpp): the contour, then the
/// holes, of each expolygon, empty rings included.
fn clib_expolygons_paths(expolygons: &[ExPolygon]) -> crate::clipper::Paths {
    let mut out = crate::clipper::Paths::new();
    for e in expolygons {
        out.push(clib_path_of_points(e.contour.points()));
        for hole in &e.holes {
            out.push(clib_path_of_points(hole.points()));
        }
    }
    out
}

/// An expolygon as the contour and holes `offset_expolygon_inner` reads.
fn clib_expolygon_rings(expolygon: &ExPolygon) -> (crate::clipper::Path, crate::clipper::Paths) {
    (
        clib_path_of_points(expolygon.contour.points()),
        expolygon.holes.iter().map(|h| clib_path_of_points(h.points())).collect(),
    )
}

/// `offset_ex(const ExPolygon&, float delta, joinType, miterLimit)`
/// (ClipperUtils.cpp:567-569). `delta_scaled` is the f64 of the C++ `float`;
/// `miter_limit` as for [`offset_ex_polygons_clib_ml`].
pub fn offset_ex_expolygon_clib(
    expolygon: &ExPolygon,
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
    miter_limit: f64,
) -> ExPolygons {
    let (contour, holes) = clib_expolygon_rings(expolygon);
    offset_ex_expolygon_paths(&contour, &holes, delta_scaled, clib_join_type(clib_join_code(join_type)), miter_limit)
}

/// `offset_ex(const ExPolygons&, float delta, joinType, miterLimit)`
/// (ClipperUtils.cpp:570-571): each expolygon offset by `offset_expolygon_inner`,
/// then one pftNonZero union into a PolyTree. `delta_scaled` is the f64 of the C++
/// `float`; `miter_limit` as for [`offset_ex_polygons_clib_ml`].
pub fn offset_ex_expolygons_clib_ml(
    expolygons: &[ExPolygon],
    delta_scaled: CoordF,
    join_type: OffsetJoinType,
    miter_limit: f64,
) -> ExPolygons {
    let rings: Vec<(crate::clipper::Path, crate::clipper::Paths)> = expolygons.iter().map(clib_expolygon_rings).collect();
    offset_ex_expolygons_paths(&rings, delta_scaled, clib_join_type(clib_join_code(join_type)), miter_limit)
}

/// Flatten a set of ExPolygons into the flat (x,y i32 pairs + per-path lengths)
/// layout the `cz_difference_closed` shim expects. Each ExPolygon contributes its
/// contour and each hole as a separate closed path, in natural orientation —
/// matching ClipperUtils::ExPolygonsProvider.
fn clib_flatten_expolygons(expolygons: &[ExPolygon]) -> (Vec<i32>, Vec<i32>, i32) {
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    for ex in expolygons {
        let mut push_ring = |ring: &Polygon| {
            let pts = ring.points();
            lens.push(pts.len() as i32);
            for p in pts {
                assert_i32_scaled(p.x);
                assert_i32_scaled(p.y);
                xy.push(p.x as i32);
                xy.push(p.y as i32);
            }
        };
        push_ring(&ex.contour);
        for hole in &ex.holes {
            push_ring(hole);
        }
    }
    let num = lens.len() as i32;
    (xy, lens, num)
}

/// Flatten raw `Polygons` (orientation preserved as stored) for the clipper-z
/// shim — the raw-path counterpart of [`clib_flatten_expolygons`].
fn clib_flatten_polygons(polygons: &[Polygon]) -> (Vec<i32>, Vec<i32>, i32) {
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    for ring in polygons {
        let pts = ring.points();
        if pts.is_empty() {
            continue;
        }
        lens.push(pts.len() as i32);
        for p in pts {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            xy.push(p.x as i32);
            xy.push(p.y as i32);
        }
    }
    let num = lens.len() as i32;
    (xy, lens, num)
}

/// ClipperLib-backed `diff(const Polygons &subject, const ExPolygons &clip)`
/// returning raw `Polygons` (ClipperUtils `diff` = `clipper_do<Paths>
/// (ctDifference, ..., pftNonZero)` with the output paths taken as-is — NO
/// PolyTree re-union, unlike `diff_ex`). Used by
/// `mmu_segmentation_top_and_bottom_layers`'s occlusion trim (MMS.cpp:1447-1452),
/// which keeps working on raw polygon soup until the per-layer `union_ex`.
pub fn difference_polygons_clib(subject: &[Polygon], clip: &[ExPolygon]) -> Vec<Polygon> {
    if subject.is_empty() {
        return vec![];
    }
    if clip.is_empty() && diff_empty_clip_returns_subject() {
        return subject.to_vec();
    }
    let (subject_xy, subject_lens, subject_num) = clib_flatten_polygons(subject);
    let (clip_xy, clip_lens, _) = clib_flatten_expolygons(clip);
    if subject_num == 0 {
        return vec![];
    }
    return clib_polygons_from_paths(&clipper_do::<crate::clipper::Paths>(
        crate::clipper::ClipType::ctDifference,
        &clib_read_closed_paths(&subject_xy, &subject_lens),
        &clib_read_closed_paths_or_null(&clip_xy, &clip_lens),
        crate::clipper::PolyFillType::pftNonZero,
    ));
}

/// Raw closed-path boolean over the ClipperLib shim: `_clipper(clipType,
/// PolygonsProvider(subject), PolygonsProvider(clip), ApplySafetyOffset::No)`
/// (ClipperUtils.cpp:669-673) — one `clipper_do<Paths>(…, pftNonZero)`, the
/// output paths taken as they come. `intersection` selects ctIntersection,
/// otherwise ctDifference. The operation always runs, an empty clip included.
fn clib_boolean_polygons_raw(subject: &[Polygon], clip: &[Polygon], intersection: bool) -> Vec<Polygon> {
    let (subject_xy, subject_lens, subject_num) = clib_flatten_polygons(subject);
    let (clip_xy, clip_lens, _) = clib_flatten_polygons(clip);
    if subject_num == 0 {
        return vec![];
    }
    return clib_polygons_from_paths(&clipper_do::<crate::clipper::Paths>(
        if intersection {
            crate::clipper::ClipType::ctIntersection
        } else {
            crate::clipper::ClipType::ctDifference
        },
        &clib_read_closed_paths(&subject_xy, &subject_lens),
        &clib_read_closed_paths_or_null(&clip_xy, &clip_lens),
        crate::clipper::PolyFillType::pftNonZero,
    ));
}

/// `diff(const Polygons &subject, const Polygons &clip)` (ClipperUtils.cpp:677),
/// raw `Polygons` out.
pub fn diff_polygons_raw_clib(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
    clib_boolean_polygons_raw(subject, clip, false)
}

/// `intersection(const Polygons &subject, const Polygons &clip)`
/// (ClipperUtils.cpp `_clipper(ctIntersection, …)`), raw `Polygons` out.
pub fn intersection_polygons_raw_clib(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
    if clip.is_empty() {
        return vec![];
    }
    clib_boolean_polygons_raw(subject, clip, true)
}

/// ClipperLib-backed `diff_ex(subject, clip, ApplySafetyOffset::No)`
/// (ClipperUtils.cpp:742-768): closed-path boolean DIFFERENCE (subject - clip)
/// over the vertex-exact vendored ClipperLib (clipper-z-sys @ 1e5), instead of
/// geo-clipper @ 1µm which over-segments the result contours.
///
/// Faithfulness: the shim runs `clipper_do<Paths>(ctDifference, subject, clip,
/// pftNonZero)` and returns the raw difference paths; we then re-union them via
/// `union_polygons_ex` (NonZero union + PolyTree nesting), which is exactly
/// `diff_ex` = `PolyTreeToExPolygons(clipper_do_polytree(...))` (the polytree
/// pass is itself a re-union of the difference Paths — ClipperUtils.cpp:640-653).
/// No safety offset is applied (the gap-fill call sites use ApplySafetyOffset::No).
pub fn difference_clib(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() {
        return vec![];
    }
    if clip.is_empty() && diff_empty_clip_returns_subject() {
        return subject.to_vec();
    }

    let (subject_xy, subject_lens, subject_num) = clib_flatten_expolygons(subject);
    let (clip_xy, clip_lens, _) = clib_flatten_expolygons(clip);
    if subject_num == 0 {
        return vec![];
    }

    let all_paths: Vec<Polygon> = clib_polygons_from_paths(&clipper_do::<crate::clipper::Paths>(
        crate::clipper::ClipType::ctDifference,
        &clib_read_closed_paths(&subject_xy, &subject_lens),
        &clib_read_closed_paths_or_null(&clip_xy, &clip_lens),
        crate::clipper::PolyFillType::pftNonZero,
    ));

    if all_paths.is_empty() {
        return vec![];
    }

    // R106: the difference-path RECONSTRUCTION, the faithful match to C++ diff_ex's
    // clipper_do_polytree re-union (ClipperUtils.cpp:641-654): one NonZero union into a
    // PolyTree, then PolyTree->ExPolygons, full-resolution. The geo-clipper
    // reconstruction it replaced (F1_UNION=0, removed at R911) snapped every
    // difference-output vertex to the 1µm grid, which silently defeated the R104/R105
    // only_one_wall_top un-gridding (`last` stayed 56.4% on the 1µm grid; 1.9% after).
    polytree_ex_clib(&all_paths)
}

/// ClipperLib-backed `intersection_ex(subject, clip)` (ClipperUtils.cpp:802,
/// ApplySafetyOffset::No): the faithful closed-path `ctIntersection` + PolyTree
/// re-union, structurally identical to [`difference_clib`] but with a single
/// `cz_intersection_closed` op. This REPLACES the R118 `A - (A - B)` double-
/// difference intersection substitute, which (per the R119 stage-split oracle)
/// added a near-collinear vertex on the L2/L3/L4/L6/L8/L16 band (the second
/// ctDifference re-processed the intermediate region, leaving a ~0.001µm-off
/// point native's single-op intersection drops). Byte-faithful reconstruction is
/// the same `polytree_ex_clib`.
pub fn intersection_clib(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return vec![];
    }

    let (subject_xy, subject_lens, subject_num) = clib_flatten_expolygons(subject);
    let (clip_xy, clip_lens, clip_num) = clib_flatten_expolygons(clip);
    if subject_num == 0 || clip_num == 0 {
        return vec![];
    }

    let all_paths: Vec<Polygon> = clib_polygons_from_paths(&clipper_do::<crate::clipper::Paths>(
        crate::clipper::ClipType::ctIntersection,
        &clib_read_closed_paths(&subject_xy, &subject_lens),
        &clib_read_closed_paths(&clip_xy, &clip_lens),
        crate::clipper::PolyFillType::pftNonZero,
    ));

    if all_paths.is_empty() {
        return vec![];
    }

    polytree_ex_clib(&all_paths)
}

/// ClipperLib-backed `diff_ex(subject, clip, ApplySafetyOffset::Yes)`
/// (ClipperUtils.cpp:334): identical to [`difference_clib`] but the clip paths are
/// safety-offset (raw +10-unit ClipperOffset, jtMiter/ML3, orientation-aware — no
/// union reconstruction) inside the shim before the ctDifference, matching native's
/// `clipper_do(ctDifference, subject, safety_offset(clip), pftNonZero)`. The
/// only_one_wall_top block's differences (top_polygons, inner_polygons, bridge
/// lower-diff) all use ApplySafetyOffset::Yes; the No-safety `difference_clib` was a
/// systemic gap (R113-R115). Reconstruction is the same `polytree_ex_clib`.
/// `intersection(subject, clip, ApplySafetyOffset::Yes)` via the vendored
/// ClipperLib (cz_intersection_closed_safety): ctIntersection against
/// safety_offset(clip) (+10u jtMiter/3 per-path), output re-unioned to
/// ExPolygons like the difference variant.
pub fn intersection_clib_safety(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return vec![];
    }
    let (subject_xy, subject_lens, subject_num) = clib_flatten_expolygons(subject);
    let (clip_xy, clip_lens, clip_num) = clib_flatten_expolygons(clip);
    if subject_num == 0 || clip_num == 0 {
        return vec![];
    }
    let all_paths: Vec<Polygon> = clib_polygons_from_paths(&clipper_do_safety_offset::<crate::clipper::Paths>(
        crate::clipper::ClipType::ctIntersection,
        &clib_read_closed_paths(&subject_xy, &subject_lens),
        &clib_read_closed_paths(&clip_xy, &clip_lens),
        crate::clipper::PolyFillType::pftNonZero,
        ApplySafetyOffset::Yes,
    ));
    if all_paths.is_empty() {
        return vec![];
    }
    polytree_ex_clib(&all_paths)
}

pub fn difference_clib_safety(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    if subject.is_empty() {
        return vec![];
    }
    if clip.is_empty() && diff_empty_clip_returns_subject() {
        return subject.to_vec();
    }

    let (subject_xy, subject_lens, subject_num) = clib_flatten_expolygons(subject);
    let (clip_xy, clip_lens, _) = clib_flatten_expolygons(clip);
    if subject_num == 0 {
        return vec![];
    }

    let all_paths: Vec<Polygon> = clib_polygons_from_paths(&clipper_do_safety_offset::<crate::clipper::Paths>(
        crate::clipper::ClipType::ctDifference,
        &clib_read_closed_paths(&subject_xy, &subject_lens),
        &clib_read_closed_paths_or_null(&clip_xy, &clip_lens),
        crate::clipper::PolyFillType::pftNonZero,
        ApplySafetyOffset::Yes,
    ));

    if all_paths.is_empty() {
        return vec![];
    }
    polytree_ex_clib(&all_paths)
}

/// ClipperLib-backed `union_ex(const Polygons&, PolyFillType)`
/// (ClipperUtils.cpp:813-814): the slice-stage F1 union behind make_expolygons
/// (TriangleMeshSlicer.cpp:1819-1823). Runs the vertex-exact vendored ClipperLib
/// (clipper-z-sys @ native 1e5 scale) union over the input loops and rebuilds
/// ExPolygons via the EXACT PolyTreeToExPolygons nesting, instead of geo-clipper
/// @ scale-1000 which re-quantizes the slice coordinates to a coarse grid (R83:
/// the residual that blocks slice byte-match). The shim encodes the ExPolygon
/// grouping in each output point's z (contour z=0 starts a new ExPolygon, hole
/// z=1 attaches to the current contour); paths arrive in PolyTreeToExPolygons
/// order (contour immediately followed by its holes).
///
/// `fill_type`: 0=EvenOdd, 1=NonZero, 2=Positive, 3=Negative (matches C++
/// make_expolygons' fill_type derived from the slicing mode).
/// R195: faithful `clipper_union<ClipperLib::Paths>` (ClipperUtils.cpp:339-350)
/// — ONE flat Execute(ctUnion, pftNonZero) over closed paths, returning flat
/// Polygons in Clipper's native output order. Used by expolygons_offset's
/// positive-delta multi-island union (lower_polygons_series grow).
pub fn union_flat_clib(loops: &[Polygon]) -> Vec<Polygon> {
    if loops.is_empty() {
        return vec![];
    }
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    for ring in loops {
        let pts = ring.points();
        if pts.is_empty() {
            continue;
        }
        lens.push(pts.len() as i32);
        for p in pts {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            xy.push(p.x as i32);
            xy.push(p.y as i32);
        }
    }
    let num = lens.len() as i32;
    if num == 0 {
        return vec![];
    }
    return clib_polygons_from_paths(&clipper_union::<crate::clipper::Paths>(
        &clib_read_closed_paths(&xy, &lens),
        crate::clipper::PolyFillType::pftNonZero,
    ));
}

/// R320: faithful variable_offset_inner_ex via the vendored ClipperLib shim
/// (verbatim mitered offset + Clipper1 NEGATIVE-fill cleanup). `deltas` are
/// SCALED f32, ring-major (contour first, then holes), one per vertex.
pub fn variable_offset_inner_ex_clib(
    expoly: &ExPolygon,
    deltas: &[Vec<f32>],
    miter_limit: f64,
) -> ExPolygons {
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    let mut flat_deltas: Vec<f32> = Vec::new();
    let mut push_ring = |pts: &[Point], ds: &[f32]| {
        debug_assert_eq!(pts.len(), ds.len());
        lens.push(pts.len() as i32);
        for p in pts {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            xy.push(p.x as i32);
            xy.push(p.y as i32);
        }
        flat_deltas.extend_from_slice(ds);
    };
    push_ring(&expoly.contour.points, &deltas[0]);
    for (h, hole) in expoly.holes.iter().enumerate() {
        push_ring(hole.points(), &deltas[1 + h]);
    }
    return variable_offset_inner_ex_paths(&clib_read_closed_paths(&xy, &lens), &flat_deltas, miter_limit);
}

/// R729 — call/size/time census for `union_ex_clib`, to separate per-CALL
/// overhead from per-VERTEX work. `UNIONPROF=1` prints a histogram at exit.
static UNIONPROF_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static UNIONPROF_VERTS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static UNIONPROF_NANOS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static UNIONPROF_FFI_NANOS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
/// buckets by total input vertex count: <4, <10, <32, <128, <512, >=512
static UNIONPROF_BUCKETS: [std::sync::atomic::AtomicUsize; 6] = [
    std::sync::atomic::AtomicUsize::new(0), std::sync::atomic::AtomicUsize::new(0),
    std::sync::atomic::AtomicUsize::new(0), std::sync::atomic::AtomicUsize::new(0),
    std::sync::atomic::AtomicUsize::new(0), std::sync::atomic::AtomicUsize::new(0),
];

pub fn unionprof_report() {
    use std::sync::atomic::Ordering::Relaxed;
    if !crate::probe_enabled("UNIONPROF") {
        return;
    }
    let c = UNIONPROF_CALLS.load(Relaxed);
    let v = UNIONPROF_VERTS.load(Relaxed);
    let n = UNIONPROF_NANOS.load(Relaxed);
    let f = UNIONPROF_FFI_NANOS.load(Relaxed);
    let b: Vec<usize> = UNIONPROF_BUCKETS.iter().map(|x| x.load(Relaxed)).collect();
    eprintln!(
        "[UNIONPROF] calls={c} verts={v} verts_per_call={:.1} total_ms={:.1} ffi_ms={:.1} ffi_share={:.0}%",
        if c > 0 { v as f64 / c as f64 } else { 0.0 },
        n as f64 / 1e6, f as f64 / 1e6,
        if n > 0 { 100.0 * f as f64 / n as f64 } else { 0.0 }
    );
    eprintln!(
        "[UNIONPROF] size buckets  <4:{} <10:{} <32:{} <128:{} <512:{} >=512:{}",
        b[0], b[1], b[2], b[3], b[4], b[5]
    );
}

pub fn union_ex_clib(loops: &[Polygon], fill_type: i32) -> ExPolygons {
    union_ex_clib_impl(loops, fill_type, false)
}

/// The reconstruction step of `diff_ex`, `intersection_ex` and
/// `offset_ex(ExPolygons)`: `PolyTreeToExPolygons(clipper_union<PolyTree>(paths))`
/// — ONE NonZero union into a PolyTree over the raw output paths of the
/// operation before it (clipper_do_polytree, ClipperUtils.cpp:641-654;
/// expolygons_offset_pt, ClipperUtils.cpp:552-557). [`union_ex_clib`] is
/// `union_ex`, which unions twice; behind a boolean operation or an offset that
/// is one union more than C++ runs, and now and then it drops a vertex.
/// CLIB_POLYTREE_ONE_PASS=0 goes back to the two unions.
pub fn polytree_ex_clib(paths: &[Polygon]) -> ExPolygons {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let one_pass = *G.get_or_init(|| crate::faithful_gate("CLIB_POLYTREE_ONE_PASS"));
    union_ex_clib_impl(paths, 1, one_pass)
}

fn union_ex_clib_impl(loops: &[Polygon], fill_type: i32, one_pass: bool) -> ExPolygons {
    if loops.is_empty() {
        return vec![];
    }
    let prof = crate::probe_enabled("UNIONPROF");
    let t_start = if prof { Some(std::time::Instant::now()) } else { None };
    // Flatten loops into (x,y) i32 pairs + per-path lengths.
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    for ring in loops {
        let pts = ring.points();
        if pts.is_empty() {
            continue;
        }
        lens.push(pts.len() as i32);
        for p in pts {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            xy.push(p.x as i32);
            xy.push(p.y as i32);
        }
    }
    let num = lens.len() as i32;
    if num == 0 {
        return vec![];
    }

    let t_ffi = if prof { Some(std::time::Instant::now()) } else { None };
    let result: ExPolygons = {
        let subject = clib_read_closed_paths(&xy, &lens);
        let polytree = if one_pass {
            clipper_union::<crate::clipper::PolyTree>(&subject, crate::clipper::PolyFillType::pftNonZero)
        } else {
            clipper_do_polytree(
                crate::clipper::ClipType::ctUnion,
                &subject,
                &crate::clipper::Paths::new(),
                clib_fill_type(fill_type),
            )
        };
        if let Some(t) = t_ffi {
            use std::sync::atomic::Ordering::Relaxed;
            UNIONPROF_FFI_NANOS.fetch_add(t.elapsed().as_nanos() as usize, Relaxed);
        }
        poly_tree_to_ex_polygons(&polytree)
    };

    if let Some(t) = t_start {
        use std::sync::atomic::Ordering::Relaxed;
        let nv = xy.len() / 2;
        UNIONPROF_CALLS.fetch_add(1, Relaxed);
        UNIONPROF_VERTS.fetch_add(nv, Relaxed);
        UNIONPROF_NANOS.fetch_add(t.elapsed().as_nanos() as usize, Relaxed);
        let bi = if nv < 4 { 0 } else if nv < 10 { 1 } else if nv < 32 { 2 }
                 else if nv < 128 { 3 } else if nv < 512 { 4 } else { 5 };
        UNIONPROF_BUCKETS[bi].fetch_add(1, Relaxed);
    }

    result
}

/// ClipperLib-backed `union_safety_offset_ex(Polygons)`: the subject paths get the
/// native raw +10u safety offset (no union) before the two-pass NonZero union
/// (cz_union_ex_safety). Same grouped output decode as [`union_ex_clib`].
pub fn union_safety_offset_ex_clib(loops: &[Polygon]) -> ExPolygons {
    if loops.is_empty() {
        return vec![];
    }
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    for ring in loops {
        let pts = ring.points();
        if pts.is_empty() {
            continue;
        }
        lens.push(pts.len() as i32);
        for p in pts {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            xy.push(p.x as i32);
            xy.push(p.y as i32);
        }
    }
    let num = lens.len() as i32;
    if num == 0 {
        return vec![];
    }
    return union_safety_offset_ex_paths(&clib_read_closed_paths(&xy, &lens));
}

/// ClipperLib-backed `offset2_ex(ExPolygons, delta1, delta2)` (ClipperUtils.cpp:581)
/// — the post-union morphological CLOSE applied by make_expolygons
/// (TriangleMeshSlicer.cpp:1820, delta1=+scale(closing_radius),
/// delta2=-scale(closing_radius)). `delta1_mm`/`delta2_mm` are in mm; scaled to 1e5
/// inside. Unlike the geo-clipper `offset_expolygons` of the time (scale-1000, two
/// independent passes), this keeps the whole grow→shrink (incl. the intermediate union) on the
/// vertex-exact ClipperLib @1e5 via `cz_offset2_ex`, matching C++ byte-for-byte.
/// Input `expolygons` is the union output (contour CCW + holes CW); output is the
/// closed ExPolygons.
pub fn offset2_ex_clib(
    expolygons: &[ExPolygon],
    delta1_mm: CoordF,
    delta2_mm: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    offset2_ex_clib_miter(expolygons, delta1_mm, delta2_mm, join_type, CLIB_MITER_LIMIT)
}

/// offset2_ex with an explicit miter limit — discover_horizontal_shells'
/// opening uses jtMiter with limit 5 (PrintObject.cpp:3502/3514), not the
/// DefaultMiterLimit 3.
pub fn offset2_ex_clib_miter(
    expolygons: &[ExPolygon],
    delta1_mm: CoordF,
    delta2_mm: CoordF,
    join_type: OffsetJoinType,
    miter_limit: f64,
) -> ExPolygons {
    if expolygons.is_empty() {
        return vec![];
    }
    // Flatten into the grouped (xy pairs, per-path lens, per-path is_hole) layout.
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    let mut is_hole: Vec<i32> = Vec::new();
    for ex in expolygons {
        let cpts = ex.contour.points();
        if cpts.is_empty() {
            continue;
        }
        lens.push(cpts.len() as i32);
        is_hole.push(0);
        for p in cpts {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            xy.push(p.x as i32);
            xy.push(p.y as i32);
        }
        for hole in &ex.holes {
            lens.push(hole.points().len() as i32);
            is_hole.push(1);
            for p in hole.points() {
                assert_i32_scaled(p.x);
                assert_i32_scaled(p.y);
                xy.push(p.x as i32);
                xy.push(p.y as i32);
            }
        }
    }
    let num = lens.len() as i32;
    if num == 0 {
        return vec![];
    }
    // R98: native `offset2_ex(const ExPolygons&, const float delta1, const float
    // delta2, ...)` (ClipperUtils.cpp:581, ClipperUtils.hpp:397) takes the deltas
    // through *float* parameters. make_expolygons passes `scale_(closing_radius)`
    // (a f64, = closing_radius / 0.00001), which is TRUNCATED to f32 at the call
    // boundary and only then promoted back to f64 inside `co.Execute(double)`.
    // For closing_radius = 0.049 that is f32(4899.99987) == 4900.0 exactly. cz
    // previously passed the full-precision f64 4899.99987, shifting the negative
    // ClipperOffset points by ~1 scaled unit on ~43 layers. Reproduce the f32
    // truncation (scale_ divides by SCALING_FACTOR = 0.00001; libslic3r.h:81).
    let delta1 = ((delta1_mm / 0.00001_f64) as f32) as f64;
    let delta2 = ((delta2_mm / 0.00001_f64) as f32) as f64;

    return offset2_ex_paths(
        &xy,
        &lens,
        &is_hole,
        delta1,
        delta2,
        clib_join_type(clib_join_code(join_type)),
        miter_limit,
    );
}

/// ClipperLib-backed `simplify_polygons(subject, preserve_collinear=false)`
/// (ClipperUtils.cpp:1026-1040) = `ClipperLib::SimplifyPolygons(subject, pftNonZero)`
/// = ctUnion with StrictlySimple(true). The post-DP step of ExPolygon::simplify_p.
/// Returns flat Polygons (caller re-unions via union_ex). StrictlySimple(true) is
/// the KEY difference from a plain ctUnion — different vertex retention (R91).
pub fn simplify_polygons_clib(loops: &[Polygon], fill_type: i32) -> Vec<Polygon> {
    if loops.is_empty() {
        return vec![];
    }
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    for ring in loops {
        let pts = ring.points();
        if pts.is_empty() {
            continue;
        }
        lens.push(pts.len() as i32);
        for p in pts {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            xy.push(p.x as i32);
            xy.push(p.y as i32);
        }
    }
    let num = lens.len() as i32;
    if num == 0 {
        return vec![];
    }
    // ClipperLib::SimplifyPolygons (clipper.hpp:558-566), which is all the shim's
    // cz_simplify_polygons runs.
    return clib_polygons_from_paths(&crate::clipper::simplify_polygons(
        &clib_read_closed_paths(&xy, &lens),
        clib_fill_type(fill_type),
    ));
}

/// R803 — faithful `offset(const Polygons&, float delta)` (ClipperUtils.cpp:413)
/// via the ClipperLib shim: per-path raw_offset, then `clipper_union<Paths>`
/// (delta > 0) or the pftNegative shrink trick (delta <= 0). Returns the flat
/// Paths in Clipper's own order, ring START vertex included — the property the
/// geo route and the raw-paths route both lose, and which
/// `WallToolPaths::generate` depends on (its `simplify` walks from index 0).
/// `delta` is SCALED; callers pass it f32-rounded like the C++ float parameter.
pub fn offset_polygons_clib(loops: &[Polygon], delta: f64, join_type: OffsetJoinType) -> Vec<Polygon> {
    offset_polygons_clib_ml(loops, delta, join_type, 3.0)
}

/// [`offset_polygons_clib`] with the `miterLimit` argument of the C++ `offset`
/// spelled out: the unitless miter limit for jtMiter, the arc tolerance in
/// scaled units for jtRound (ClipperUtils.cpp `raw_offset`).
pub fn offset_polygons_clib_ml(
    loops: &[Polygon],
    delta: f64,
    join_type: OffsetJoinType,
    miter_limit: f64,
) -> Vec<Polygon> {
    if loops.is_empty() {
        return vec![];
    }
    let mut xy: Vec<i32> = Vec::new();
    let mut lens: Vec<i32> = Vec::new();
    for ring in loops {
        let pts = ring.points();
        if pts.is_empty() {
            continue;
        }
        lens.push(pts.len() as i32);
        for p in pts {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            xy.push(p.x as i32);
            xy.push(p.y as i32);
        }
    }
    let num = lens.len() as i32;
    if num == 0 {
        return vec![];
    }
    let jt: i32 = match join_type {
        OffsetJoinType::Round => 1,
        OffsetJoinType::Square => 2,
        _ => 0,
    };
    return clib_polygons_from_paths(&offset_paths::<crate::clipper::Paths>(
        &clib_read_closed_paths(&xy, &lens),
        delta,
        clib_join_type(jt),
        miter_limit,
    ));
}

/// Detect gaps between two polygon sets.
///
/// Gaps are the narrow regions that exist in the outer area but not in the
/// inner area. This is used for gap fill detection between perimeter levels.
///
/// # Arguments
/// * `outer` - The outer boundary (e.g., previous perimeter level)
/// * `inner` - The inner boundary (e.g., current perimeter level after offset)
/// * `min_width` - Minimum gap width to detect (mm)
/// * `max_width` - Maximum gap width to detect (mm)
/// * `join_type` - The join type for offset corners
///
/// # Returns
/// ExPolygons representing the detected gap regions.
pub fn detect_gaps(
    outer: &[ExPolygon],
    inner: &[ExPolygon],
    min_width: CoordF,
    max_width: CoordF,
    join_type: OffsetJoinType,
) -> ExPolygons {
    if outer.is_empty() {
        return vec![];
    }

    // Gap detection algorithm from BambuStudio:
    // 1. Shrink outer by half of expected gap width to get where gaps might be
    // 2. Grow inner back by half of max width plus safety offset
    // 3. Take the difference - these are the potential gap regions
    // 4. Apply opening to remove regions smaller than min_width
    // 5. Apply offset2 to remove regions larger than max_width

    let half_min = min_width / 2.0;
    let half_max = max_width / 2.0;
    const SAFETY_OFFSET: CoordF = 0.00001; // 10nm safety offset

    // Regions that might be gaps: shrink outer slightly
    let potential_gaps = shrink(outer, half_min, join_type);

    // Regions covered by inner perimeters (with some margin)
    let inner_expanded = grow(inner, half_max + SAFETY_OFFSET, join_type);

    // Gaps are where potential gaps exist but inner doesn't cover
    let raw_gaps = difference(&potential_gaps, &inner_expanded);

    if raw_gaps.is_empty() {
        return vec![];
    }

    // Clean up: apply morphological opening to remove too-narrow regions
    let opened = opening(&raw_gaps, half_min, join_type);

    // Apply offset2 to remove regions that are too wide
    // (they should be filled by normal infill, not gap fill)
    let gaps = offset2(&opened, half_max, half_max + SAFETY_OFFSET, join_type);

    gaps
}

/// Extract centerlines from narrow polygon regions using offset approximation.
///
/// This is a simplified alternative to medial axis computation. It works by
/// progressively shrinking the polygon and collecting the resulting contours.
///
/// # Arguments
/// * `expolygons` - The narrow regions to extract centerlines from
/// * `width` - The expected width of the regions (mm)
/// * `join_type` - The join type for offset corners
///
/// # Returns
/// Polylines representing approximate centerlines.
pub fn extract_centerlines(
    expolygons: &[ExPolygon],
    width: CoordF,
    join_type: OffsetJoinType,
) -> Vec<Polyline> {
    if expolygons.is_empty() || width <= 0.0 {
        return vec![];
    }

    let mut centerlines = Vec::new();

    // Shrink by half the width to get approximate centerlines
    let half_width = width / 2.0;
    let shrunk = shrink(expolygons, half_width, join_type);

    // Convert the resulting polygons to polylines
    for expoly in &shrunk {
        // Convert contour to polyline
        if !expoly.contour.is_empty() {
            let mut points = expoly.contour.points().to_vec();
            // Close the polyline by adding the first point at the end
            if let Some(first) = points.first().cloned() {
                points.push(first);
            }
            if points.len() >= 2 {
                centerlines.push(Polyline::from_points(points));
            }
        }

        // Convert holes to polylines
        for hole in &expoly.holes {
            if !hole.is_empty() {
                let mut points = hole.points().to_vec();
                if let Some(first) = points.first().cloned() {
                    points.push(first);
                }
                if points.len() >= 2 {
                    centerlines.push(Polyline::from_points(points));
                }
            }
        }
    }

    // If shrinking eliminated everything, try a different approach:
    // Take the original polygon contours as approximate centerlines
    if centerlines.is_empty() {
        for expoly in expolygons {
            if !expoly.contour.is_empty() {
                let mut points = expoly.contour.points().to_vec();
                if let Some(first) = points.first().cloned() {
                    points.push(first);
                }
                if points.len() >= 2 {
                    centerlines.push(Polyline::from_points(points));
                }
            }
        }
    }

    centerlines
}

/// Intersect polylines with a set of ExPolygons, returning clipped polylines.
///
/// This clips the input polylines to only the portions that fall inside
/// the given ExPolygons. Each input polyline may produce zero, one, or
/// multiple output polylines depending on how it intersects the clipping regions.
///
/// # Arguments
/// * `polylines` - The polylines to clip
/// * `clip` - The ExPolygons to clip against (portions inside these are kept)
///
/// # Returns
/// A vector of polylines representing the clipped portions.
pub fn intersect_polylines_with_expolygons(
    polylines: &[Polyline],
    clip: &[ExPolygon],
) -> Vec<Polyline> {
    if polylines.is_empty() || clip.is_empty() {
        return vec![];
    }

    let mut result = Vec::new();

    for polyline in polylines {
        let clipped = clip_polyline_to_expolygons(polyline, clip);
        result.extend(clipped);
    }

    result
}

/// Clip a single polyline to a set of ExPolygons.
fn clip_polyline_to_expolygons(polyline: &Polyline, clip: &[ExPolygon]) -> Vec<Polyline> {
    let points = polyline.points();
    if points.len() < 2 {
        return vec![];
    }

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();

    // Process each segment of the polyline
    for i in 0..points.len() - 1 {
        let p1 = points[i];
        let p2 = points[i + 1];

        // Find all intersections of this segment with clip boundaries
        let clipped_segments = clip_segment_to_expolygons(p1, p2, clip);

        for segment in clipped_segments {
            if segment.len() >= 2 {
                // Try to connect to current segment
                if !current_segment.is_empty() {
                    let last = *current_segment.last().unwrap();
                    let first = segment[0];
                    // Check if segments are connected (within tolerance)
                    if (last.x - first.x).abs() <= 1 && (last.y - first.y).abs() <= 1 {
                        // Connected, extend current segment
                        current_segment.extend(segment.into_iter().skip(1));
                    } else {
                        // Not connected, save current and start new
                        if current_segment.len() >= 2 {
                            result.push(Polyline::from_points(current_segment));
                        }
                        current_segment = segment;
                    }
                } else {
                    current_segment = segment;
                }
            }
        }
    }

    // Don't forget the last segment
    if current_segment.len() >= 2 {
        result.push(Polyline::from_points(current_segment));
    }

    result
}

/// Clip a line segment to a set of ExPolygons.
/// Returns the portions of the segment that are inside any of the ExPolygons.
fn clip_segment_to_expolygons(p1: Point, p2: Point, clip: &[ExPolygon]) -> Vec<Vec<Point>> {
    // Simple approach: sample points along the segment and check if they're inside
    // For better performance, we'd use proper line-polygon intersection

    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let len_sq = dx as f64 * dx as f64 + dy as f64 * dy as f64;

    if len_sq < 1.0 {
        // Segment too short
        if point_in_expolygons(p1, clip) {
            return vec![vec![p1, p2]];
        }
        return vec![];
    }

    let len = len_sq.sqrt();
    let step = 100_000i64; // 1.0mm sampling step (SCALING_FACTOR=100_000/mm). FIDELITY-NOTE(F1): geo backend has no exact open-path clip; per-segment sampling approximates ClipperLib.
    let num_samples = ((len / step as f64).ceil() as usize).max(2);

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();
    let mut last_inside = false;

    for i in 0..=num_samples {
        let t = i as f64 / num_samples as f64;
        let px = p1.x as f64 + dx as f64 * t;
        let py = p1.y as f64 + dy as f64 * t;
        let pt = Point::new(px.round() as i64, py.round() as i64);

        let inside = point_in_expolygons(pt, clip);

        if inside {
            if !last_inside && !current_segment.is_empty() {
                // Was outside, now inside - save previous segment if any
                if current_segment.len() >= 2 {
                    result.push(current_segment);
                }
                current_segment = Vec::new();
            }
            current_segment.push(pt);
        } else {
            if last_inside && !current_segment.is_empty() {
                // Was inside, now outside
                if current_segment.len() >= 2 {
                    result.push(current_segment);
                }
                current_segment = Vec::new();
            }
        }
        last_inside = inside;
    }

    // Save final segment
    if current_segment.len() >= 2 {
        result.push(current_segment);
    }

    result
}

/// Check if a point is inside any of the given ExPolygons.
fn point_in_expolygons(pt: Point, expolygons: &[ExPolygon]) -> bool {
    for expoly in expolygons {
        if expoly.contains_point(&pt) {
            return true;
        }
    }
    false
}

/// Subtract polygons from polylines (polyline difference).
///
/// This returns the portions of the input polylines that fall OUTSIDE
/// the given ExPolygons. This is the complement of `intersect_polylines_with_expolygons`.
///
/// In libslic3r, this is `diff_pl()`.
///
/// # Arguments
/// * `polylines` - The polylines to clip
/// * `clip` - The ExPolygons to subtract (portions outside these are kept)
///
/// # Returns
/// A vector of polylines representing the portions outside the clip regions.
// Clip source polygon to be used as a clipping polygon with a bounding box around the source (to be clipped) polygon.
// Useful as an optimization for expensive ClipperLib operations, for example when clipping source polygons one by one
// with a set of polygons covering the whole layer below.
// ClipperUtils.cpp:66-118 `clip_clipper_polygon_with_subject_bbox_templ`
fn clip_clipper_polygon_with_subject_bbox_points(
    src: &[Point],
    bbox: &BoundingBox,
    out: &mut Vec<Point>,
    get_entire_polygons: bool,
) {
    // ClipperUtils.cpp:68
    out.clear();
    // ClipperUtils.cpp:69-70
    let cnt = src.len();
    if cnt < 3 {
        return;
    }

    // ClipperUtils.cpp:72-77
    // enum class Side { Left = 1, Right = 2, Top = 4, Bottom = 8 };
    const SIDE_LEFT: i32 = 1;
    const SIDE_RIGHT: i32 = 2;
    const SIDE_TOP: i32 = 4;
    const SIDE_BOTTOM: i32 = 8;

    // ClipperUtils.cpp:78-83
    let sides = |p: &Point| -> i32 {
        (p.x < bbox.min.x) as i32 * SIDE_LEFT
            + (p.x > bbox.max.x) as i32 * SIDE_RIGHT
            + (p.y < bbox.min.y) as i32 * SIDE_BOTTOM
            + (p.y > bbox.max.y) as i32 * SIDE_TOP
    };

    // ClipperUtils.cpp:85-87
    let mut sides_prev = sides(&src[cnt - 1]);
    let mut sides_this = sides(&src[0]);
    let last = cnt - 1;
    // ClipperUtils.cpp:88-102
    for i in 0..last {
        let sides_next = sides(&src[i + 1]);
        if
        // This point is inside. Take it.
        sides_this == 0 ||
            // Either this point is outside and previous or next is inside, or
            // the edge possibly cuts corner of the bounding box.
            (sides_prev & sides_this & sides_next) == 0
        {
            // ClipperUtils.cpp:95-96
            out.push(src[i]);
            sides_prev = sides_this;
        } else {
            // ClipperUtils.cpp:98-100
            // All the three points (this, prev, next) are outside at the same side.
            // Ignore this point.
        }
        // ClipperUtils.cpp:101
        sides_this = sides_next;
    }

    // ClipperUtils.cpp:104-116
    // Never produce just a single point output polygon.
    if !out.is_empty() {
        if get_entire_polygons {
            // ClipperUtils.cpp:107
            *out = src.to_vec();
        } else {
            // ClipperUtils.cpp:109-115
            let sides_next = sides(&out[0]);
            if
            // The last point is inside. Take it.
            sides_this == 0 ||
                // Either this point is outside and previous or next is inside, or
                // the edge possibly cuts corner of the bounding box.
                (sides_prev & sides_this & sides_next) == 0
            {
                out.push(src[cnt - 1]);
            }
        }
    }
}

// ClipperUtils.cpp:137-142 `Polygon clip_clipper_polygon_with_subject_bbox(const Polygon &src, const BoundingBox &bbox, const bool get_entire_polygons)`
pub(crate) fn clip_clipper_polygon_with_subject_bbox(
    src: &Polygon,
    bbox: &BoundingBox,
    get_entire_polygons: bool,
) -> Polygon {
    // ClipperUtils.cpp:139-141
    let mut out = Polygon::new();
    clip_clipper_polygon_with_subject_bbox_points(
        src.points(),
        bbox,
        out.points_mut(),
        get_entire_polygons,
    );
    out
}

// ClipperUtils.cpp:152-160 `Polygons clip_clipper_polygons_with_subject_bbox(const ExPolygon &src, const BoundingBox &bbox, const bool get_entire_polygons)`
pub fn clip_clipper_polygons_with_subject_bbox_expolygon(
    src: &ExPolygon,
    bbox: &BoundingBox,
    get_entire_polygons: bool,
) -> Vec<Polygon> {
    // ClipperUtils.cpp:154-155
    let mut out: Vec<Polygon> = Vec::with_capacity(src.num_contours());
    // ClipperUtils.cpp:156
    out.push(clip_clipper_polygon_with_subject_bbox(
        &src.contour,
        bbox,
        get_entire_polygons,
    ));
    // ClipperUtils.cpp:157
    for p in &src.holes {
        out.push(clip_clipper_polygon_with_subject_bbox(
            p,
            bbox,
            get_entire_polygons,
        ));
    }
    // ClipperUtils.cpp:158
    out.retain(|polygon| !polygon.is_empty());
    out
}

// ClipperUtils.cpp:144-151 `Polygons clip_clipper_polygons_with_subject_bbox(const Polygons &src, const BoundingBox &bbox)`
pub fn clip_clipper_polygons_with_subject_bbox_polygons(
    src: &[Polygon],
    bbox: &BoundingBox,
) -> Vec<Polygon> {
    // ClipperUtils.cpp:146-147
    let mut out: Vec<Polygon> = Vec::with_capacity(src.len());
    // ClipperUtils.cpp:148
    for p in src {
        out.push(clip_clipper_polygon_with_subject_bbox(p, bbox, false));
    }
    // ClipperUtils.cpp:149
    out.retain(|polygon| !polygon.is_empty());
    // ClipperUtils.cpp:150
    out
}

/// ClipperUtils.cpp:911 `diff_pl(const Polylines&, const Polygons&)` —
/// `_clipper_pl_open(ctDifference, subject, clip)`: open-path subject clipped
/// against closed clip, returning the portions OUTSIDE the clip.
//
// FIDELITY-NOTE(F1): written when the geo-clipper backend had no exact open-path
// boolean. This reproduces `_clipper_pl_open(ctDifference, ...)` by per-segment sampling +
// reconstruction instead of ClipperLib open-path clipping. The faithful
// Clipper2 equivalent is `crate::clipper2_utils::diff_pl_2` (Clipper2Utils.cpp).
pub fn diff_pl(polylines: &[Polyline], clip: &[ExPolygon]) -> Vec<Polyline> {
    if polylines.is_empty() {
        return vec![];
    }

    if clip.is_empty() {
        // Nothing to subtract, return original polylines
        return polylines.to_vec();
    }

    let mut result = Vec::new();

    for polyline in polylines {
        let clipped = diff_polyline_from_expolygons(polyline, clip);
        result.extend(clipped);
    }

    result
}

/// Compute the intersection of polylines with ExPolygons.
///
/// ClipperUtils.cpp:935 `intersection_pl(const Polylines&, const ExPolygons&)` —
/// `_clipper_pl_open(ctIntersection, subject, clip)`: open-path subject clipped
/// against closed clip, returning the portions INSIDE the clip.
//
// FIDELITY-NOTE(F1): written when the geo-clipper backend had no exact open-path
// boolean. This reproduces `_clipper_pl_open(ctIntersection, ...)` by per-segment sampling +
// reconstruction instead of ClipperLib open-path clipping. The faithful
// Clipper2 equivalent is `crate::clipper2_utils::intersection_pl_2` (Clipper2Utils.cpp).
pub fn intersection_pl(polylines: &[Polyline], clip: &[ExPolygon]) -> Vec<Polyline> {
    if polylines.is_empty() || clip.is_empty() {
        return vec![];
    }

    // R792 (gate CLIB_INTERSECTION_PL) — the vendored-ClipperLib open-path clip
    // (_clipper_pl_open ctIntersection, ClipperUtils.cpp:841): ONE Execute over
    // all subjects+clips with the PolyTree preorder output, replacing this
    // hand-rolled per-segment clipper whose crossing points and path order both
    // deviate from native.
    if crate::faithful_gate("CLIB_INTERSECTION_PL") {
        return intersection_pl_clib(polylines, clip);
    }

    let mut result = Vec::new();

    for polyline in polylines {
        let clipped = intersection_polyline_with_expolygons(polyline, clip);
        result.extend(clipped);
    }

    result
}

/// R792 — `intersection_pl(Polylines, ExPolygons)` via the vendored ClipperLib
/// (cz_intersection_pl): open subjects, closed clips, pftNonZero, PolyTree
/// preorder flatten (ClipperUtils.cpp:841-849 + 213-229).
pub fn intersection_pl_clib(polylines: &[Polyline], clip: &[ExPolygon]) -> Vec<Polyline> {
    let mut subject_xy: Vec<i32> = Vec::new();
    let mut subject_lens: Vec<i32> = Vec::new();
    for pl in polylines {
        if pl.points.len() < 2 {
            continue;
        }
        subject_lens.push(pl.points.len() as i32);
        for p in &pl.points {
            subject_xy.push(p.x as i32);
            subject_xy.push(p.y as i32);
        }
    }
    let (clip_xy, clip_lens, clip_num) = clib_flatten_expolygons(clip);
    if subject_lens.is_empty() || clip_num == 0 {
        return vec![];
    }

    return _clipper_pl_open(
        crate::clipper::ClipType::ctIntersection,
        &clib_read_closed_paths(&subject_xy, &subject_lens),
        &clib_read_closed_paths(&clip_xy, &clip_lens),
    )
    .iter()
    .filter(|path| path.len() >= 2)
    .map(|path| Polyline::from_points(path.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()))
    .collect();
}

/// `diff_pl(const Polylines &subject, const Polygons &clip)` (ClipperUtils.cpp
/// 911-912) via the vendored ClipperLib (cz_diff_pl): open subjects, closed
/// clips as raw polygons, pftNonZero, PolyTree preorder flatten.
pub fn diff_pl_clib(polylines: &[Polyline], clip: &[Polygon]) -> Vec<Polyline> {
    let mut subject_xy: Vec<i32> = Vec::new();
    let mut subject_lens: Vec<i32> = Vec::new();
    for pl in polylines {
        if pl.points.is_empty() {
            continue;
        }
        subject_lens.push(pl.points.len() as i32);
        for p in &pl.points {
            assert_i32_scaled(p.x);
            assert_i32_scaled(p.y);
            subject_xy.push(p.x as i32);
            subject_xy.push(p.y as i32);
        }
    }
    if subject_lens.is_empty() {
        return vec![];
    }
    let (clip_xy, clip_lens, _) = clib_flatten_polygons(clip);

    return _clipper_pl_open(
        crate::clipper::ClipType::ctDifference,
        &clib_read_closed_paths(&subject_xy, &subject_lens),
        &clib_read_closed_paths_or_null(&clip_xy, &clip_lens),
    )
    .iter()
    .map(|path| Polyline::from_points(path.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()))
    .collect();
}

/// Compute intersection of a single polyline with ExPolygons.
/// Returns the portions of the polyline that are INSIDE any clip region.
fn intersection_polyline_with_expolygons(polyline: &Polyline, clip: &[ExPolygon]) -> Vec<Polyline> {
    let points = polyline.points();
    if points.len() < 2 {
        return vec![];
    }

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();

    // Process each segment of the polyline
    for i in 0..points.len() - 1 {
        let p1 = points[i];
        let p2 = points[i + 1];

        // Find all portions of this segment that are inside clip regions
        let inside_segments = intersection_segment_with_expolygons(p1, p2, clip);

        for segment in inside_segments {
            if segment.len() >= 2 {
                // Try to connect to current segment
                if !current_segment.is_empty() {
                    let last = *current_segment.last().unwrap();
                    let first = segment[0];
                    // Check if segments are connected (within tolerance)
                    if (last.x - first.x).abs() <= 1 && (last.y - first.y).abs() <= 1 {
                        // Connected, extend current segment
                        current_segment.extend(segment.into_iter().skip(1));
                    } else {
                        // Not connected, save current and start new
                        if current_segment.len() >= 2 {
                            result.push(Polyline::from_points(current_segment));
                        }
                        current_segment = segment;
                    }
                } else {
                    current_segment = segment;
                }
            }
        }
    }

    // Save final segment
    if current_segment.len() >= 2 {
        result.push(Polyline::from_points(current_segment));
    }

    result
}

/// Find portions of a line segment that are INSIDE clip regions.
fn intersection_segment_with_expolygons(
    p1: Point,
    p2: Point,
    clip: &[ExPolygon],
) -> Vec<Vec<Point>> {
    // R461: EXACT segment-vs-expolygon clip.
    //
    // This used to SAMPLE the segment at 1.0mm steps and keep whichever samples were
    // inside ("FIDELITY-NOTE(F1): sampling approximation of ClipperLib open-path
    // clip"). That put every clipped endpoint up to a full step from the true
    // boundary crossing — measured at 150um mean on Majora's gyroid, with only 0.5%
    // of endpoints within 1um of the polygon they were clipped against. Two
    // consequences, both real: the clipped path LOST up to a step of length at every
    // crossing, and, because the endpoints did not lie on the boundary,
    // `Fill::connect_infill`'s `EdgeGrid::closest_point(pt, SCALED_EPSILON)` lookup
    // (FillBase.cpp:1361-1382) could not map ANY of them onto a contour, so infill
    // lines were never chained (100% BOUNDARY_IDX_UNCONNECTED).
    //
    // ClipperLib clips an open path by computing the real crossings, so do that:
    // gather the parameters at which the segment crosses every edge, then keep the
    // sub-intervals whose midpoint is inside.
    let dx = (p2.x - p1.x) as f64;
    let dy = (p2.y - p1.y) as f64;
    if dx == 0.0 && dy == 0.0 {
        return Vec::new();
    }

    let mut ts: Vec<f64> = vec![0.0, 1.0];
    for ex in clip {
        let mut cross_ring = |ring: &Polygon| {
            let pts = &ring.points;
            let n = pts.len();
            for i in 0..n {
                let a = pts[i];
                let b = pts[(i + 1) % n];
                let ex_ = (b.x - a.x) as f64;
                let ey_ = (b.y - a.y) as f64;
                // p1 + t*(d) == a + u*(e)
                let denom = dx * ey_ - dy * ex_;
                if denom.abs() < 1e-12 {
                    continue; // parallel
                }
                let rx = (a.x - p1.x) as f64;
                let ry = (a.y - p1.y) as f64;
                // p1 + t*d == a + u*e  =>  t*d - u*e = r  (r = a - p1)
                //   cross with e:  t*(d x e) =  r x e  =>  t = (r x e) / denom
                //   cross with d: -u*(e x d) =  r x d  =>  u = (r x d) / denom
                // (since e x d == -(d x e) == -denom). R471: this second line had
                // `/ -denom`, flipping u's sign, so the `u in [0,1]` test admitted
                // intersections with each edge's INFINITE extension on the far side and
                // rejected genuine ones — emitting "crossing" points that lie on the
                // segment but nowhere near the edge (measured at the source: 96.8% more
                // than 1um off, mean 133.9um).
                let t = (rx * ey_ - ry * ex_) / denom;
                let u = (rx * dy - ry * dx) / denom;
                if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
                    ts.push(t);
                }
            }
        };
        cross_ring(&ex.contour);
        for h in &ex.holes {
            cross_ring(h);
        }
    }

    ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    ts.dedup_by(|a, b| (*a - *b).abs() < 1e-12);

    let at = |t: f64| Point {
        x: p1.x + (dx * t).round() as i64,
        y: p1.y + (dy * t).round() as i64,
    };

    // R471 (GYROID_ENDPOINT_DEBUG=1): verify AT THE SOURCE that a point emitted for a
    // crossing parameter really lies on a clip edge. If these all read ~0 here but the
    // caller measures ~26um, the displacement happens after this function.
    let dbg_here = crate::probe_enabled("GYROID_ENDPOINT_DEBUG");
    let dist_here = |p: &Point| -> f64 {
        let mut best = f64::MAX;
        for ex in clip {
            let mut ring = |r: &Polygon| {
                let pts = &r.points;
                for i in 0..pts.len() {
                    let a = pts[i];
                    let b = pts[(i + 1) % pts.len()];
                    let (ax, ay) = (a.x as f64, a.y as f64);
                    let (bx, by) = (b.x as f64, b.y as f64);
                    let (px, py) = (p.x as f64, p.y as f64);
                    let (ddx, ddy) = (bx - ax, by - ay);
                    let l2 = ddx * ddx + ddy * ddy;
                    let tt = if l2 <= 0.0 { 0.0 } else { (((px-ax)*ddx + (py-ay)*ddy)/l2).clamp(0.0,1.0) };
                    let d = ((px - (ax+tt*ddx)).powi(2) + (py - (ay+tt*ddy)).powi(2)).sqrt();
                    if d < best { best = d; }
                }
            };
            ring(&ex.contour);
            for h in &ex.holes { ring(h); }
        }
        best
    };

    let mut result: Vec<Vec<Point>> = Vec::new();
    let mut run: Vec<Point> = Vec::new();
    for w in ts.windows(2) {
        let (t0, t1) = (w[0], w[1]);
        let mid = at(0.5 * (t0 + t1));
        if point_in_expolygons(mid, clip) {
            let (a, b) = (at(t0), at(t1));
            if dbg_here {
                use std::sync::atomic::Ordering::Relaxed;
                for (tv, pv) in [(t0, a), (t1, b)] {
                    if tv > 1e-12 && tv < 1.0 - 1e-12 {
                        // a genuine crossing parameter
                        crate::fill::SRC_CROSS_N.fetch_add(1, Relaxed);
                        let d_um = dist_here(&pv) / crate::SCALING_FACTOR * 1000.0;
                        crate::fill::SRC_CROSS_SUM.fetch_add((d_um * 1000.0) as usize, Relaxed);
                        if d_um > 1.0 { crate::fill::SRC_CROSS_BAD.fetch_add(1, Relaxed); }
                    }
                }
            }
            if run.is_empty() {
                run.push(a);
            }
            if *run.last().unwrap() != b {
                run.push(b);
            }
        } else if run.len() >= 2 {
            result.push(std::mem::take(&mut run));
        } else {
            run.clear();
        }
    }
    if run.len() >= 2 {
        result.push(run);
    }

    result
}

/// Subtract ExPolygons from a single polyline.
/// Returns the portions of the polyline that are OUTSIDE all clip regions.
fn diff_polyline_from_expolygons(polyline: &Polyline, clip: &[ExPolygon]) -> Vec<Polyline> {
    let points = polyline.points();
    if points.len() < 2 {
        return vec![];
    }

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();

    // Process each segment of the polyline
    for i in 0..points.len() - 1 {
        let p1 = points[i];
        let p2 = points[i + 1];

        // Find all portions of this segment that are outside clip regions
        let outside_segments = diff_segment_from_expolygons(p1, p2, clip);

        for segment in outside_segments {
            if segment.len() >= 2 {
                // Try to connect to current segment
                if !current_segment.is_empty() {
                    let last = *current_segment.last().unwrap();
                    let first = segment[0];
                    // Check if segments are connected (within tolerance)
                    if (last.x - first.x).abs() <= 1 && (last.y - first.y).abs() <= 1 {
                        // Connected, extend current segment
                        current_segment.extend(segment.into_iter().skip(1));
                    } else {
                        // Not connected, save current and start new
                        if current_segment.len() >= 2 {
                            result.push(Polyline::from_points(current_segment));
                        }
                        current_segment = segment;
                    }
                } else {
                    current_segment = segment;
                }
            }
        }
    }

    // Don't forget the last segment
    if current_segment.len() >= 2 {
        result.push(Polyline::from_points(current_segment));
    }

    result
}

/// Get the portions of a line segment that are OUTSIDE all ExPolygons.
fn diff_segment_from_expolygons(p1: Point, p2: Point, clip: &[ExPolygon]) -> Vec<Vec<Point>> {
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let len_sq = dx as f64 * dx as f64 + dy as f64 * dy as f64;

    if len_sq < 1.0 {
        // Segment too short - check if it's outside
        if !point_in_expolygons(p1, clip) {
            return vec![vec![p1, p2]];
        }
        return vec![];
    }

    let len = len_sq.sqrt();
    let step = 100_000i64; // 1.0mm sampling step (SCALING_FACTOR=100_000/mm). FIDELITY-NOTE(F1): geo backend has no exact open-path clip; per-segment sampling approximates ClipperLib.
    let num_samples = ((len / step as f64).ceil() as usize).max(2);

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();
    let mut last_outside = false;

    for i in 0..=num_samples {
        let t = i as f64 / num_samples as f64;
        let px = p1.x as f64 + dx as f64 * t;
        let py = p1.y as f64 + dy as f64 * t;
        let pt = Point::new(px.round() as i64, py.round() as i64);

        // Check if point is OUTSIDE all clip regions
        let outside = !point_in_expolygons(pt, clip);

        if outside {
            if !last_outside && !current_segment.is_empty() {
                // Was inside, now outside - save previous segment if any
                if current_segment.len() >= 2 {
                    result.push(current_segment);
                }
                current_segment = Vec::new();
            }
            current_segment.push(pt);
        } else {
            if last_outside && !current_segment.is_empty() {
                // Was outside, now inside
                if current_segment.len() >= 2 {
                    result.push(current_segment);
                }
                current_segment = Vec::new();
            }
        }
        last_outside = outside;
    }

    // Save final segment
    if current_segment.len() >= 2 {
        result.push(current_segment);
    }

    result
}

/// Convert polygons to polylines (open paths).
/// Each polygon becomes a polyline with the same points (not closed).
pub fn polygons_to_polylines(polygons: &[Polygon]) -> Vec<Polyline> {
    polygons
        .iter()
        .map(|p| Polyline::from_points(p.points().to_vec()))
        .collect()
}

/// Convert ExPolygons to polylines (contours and holes as separate polylines).
pub fn expolygons_to_polylines(expolygons: &[ExPolygon]) -> Vec<Polyline> {
    let mut result = Vec::new();
    for expoly in expolygons {
        result.push(Polyline::from_points(expoly.contour.points().to_vec()));
        for hole in &expoly.holes {
            result.push(Polyline::from_points(hole.points().to_vec()));
        }
    }
    result
}

/// Union with a safety offset to prevent gaps (port of ClipperUtils::union_safety_offset_ex).
///
/// ClipperUtils.hpp:372 `union_safety_offset_ex(const Polygons&) { return offset_ex(polygons,
/// ClipperSafetyOffset); }` — a SINGLE outward `offset_ex` by +ClipperSafetyOffset (+10u),
/// i.e. `clipper_union(raw_offset(paths, +10u, jtMiter, ML3))` (ClipperUtils.cpp:539-548 with
/// delta>0). It does NOT shrink back.
///
/// PERF/PARITY (CLIPPER_INT, default on): route through the vertex-exact vendored ClipperLib
/// (`cz_union_ex_safety`) which does exactly `safety_offset(+10u)` + NonZero union at native
/// 1e5 scale — one integer op. The legacy path below does grow(+10u)+union+shrink(-10u),
/// which is both ~3x the ClipperOffset work (on geo-clipper it was the Majora
/// `bridge_over_infill` hot spot: ~36% of slice time) AND less faithful (the extra
/// shrink-back deviates from native's single dilated union). Set `CLIPPER_INT=0` to take
/// the legacy path.
pub fn union_safety_offset_ex(polygons: &[Polygon]) -> ExPolygons {
    if polygons.is_empty() {
        return ExPolygons::new();
    }
    // R806 — native `union_safety_offset_ex(const Polygons&)` is exactly
    // `offset_ex(polygons, ClipperSafetyOffset)` (ClipperUtils.hpp:372): one
    // per-path raw_offset(+10) and ONE PolyTree union. The older clib twin ran a
    // safety offset plus two union passes, which re-rounds the merged boundary
    // (Majora post-MMS lslices: 0/656 layers exact even where every region was
    // byte-exact). Gate LSLICES_OFFSET_EX (faithful ON).
    if crate::faithful_gate("CLIPPER_INT") && crate::faithful_gate("LSLICES_OFFSET_EX") {
        return offset_ex_polygons_clib(polygons, (10.0f32) as f64, OffsetJoinType::Miter);
    }

    if crate::faithful_gate("CLIPPER_INT") {
        return union_safety_offset_ex_clib(polygons);
    }

    // Apply small offset to ensure overlap
    const SAFETY_OFFSET: CoordF = 0.0001; // ClipperSafetyOffset = 10 scaled units = 0.0001mm

    // Convert to ExPolygons for offset operation
    let expolygons: ExPolygons = polygons.iter().map(|p| ExPolygon::new(p.clone())).collect();

    // Offset outward slightly
    let expanded = grow(&expolygons, SAFETY_OFFSET, OffsetJoinType::Miter);

    // Union
    let unioned = union_ex(&expanded);

    // Offset back inward
    shrink(&unioned, SAFETY_OFFSET, OffsetJoinType::Miter)
}

/// `union_safety_offset_ex` over ExPolygons.
/// ClipperUtils C++: `ExPolygons union_safety_offset_ex(const ExPolygons &expolygons)`
/// — identical to the Polygons overload after flattening contours + holes.
pub fn union_safety_offset_ex_expolygons(expolygons: &[ExPolygon]) -> ExPolygons {
    union_safety_offset_ex(&crate::geometry::to_polygons(expolygons))
}

/// POLYGONS_OPS_CLIB (on): the `Polygons` overloads below — `diff`, `diff_ex`,
/// `intersection_ex`, `expand`, `shrink`, `opening`, `closing` — run on the
/// vendored ClipperLib at coord_t, over the raw paths as C++ hands them over
/// (ClipperUtils.cpp:413-415, 592-613, 669-680, 741-742, 802). =0 unions the
/// operands into expolygons first and runs the ExPolygons helpers, as before (on the
/// 1 um geo backend until R910).
pub fn polygons_ops_clib() -> bool {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *G.get_or_init(|| crate::faithful_gate("POLYGONS_OPS_CLIB"))
}

/// A millimetre delta as the scaled `float` a C++ offset call receives.
pub fn mm_to_scaled_f32(delta_mm: CoordF) -> CoordF {
    ((delta_mm * crate::SCALING_FACTOR) as f32) as f64
}

/// Raw polygons as hole-less expolygons, for the helpers that take their
/// paths from `&[ExPolygon]`: each ring is flattened as it is.
pub fn polygons_as_paths(polygons: &[Polygon]) -> ExPolygons {
    polygons.iter().map(|p| ExPolygon::new(p.clone())).collect()
}

/// `shrink(const Polygons &, float delta)` = `offset(polygons, -delta)`
/// (ClipperUtils.hpp), miter join.
pub fn shrink_polygons_clib(polygons: &[Polygon], delta_mm: CoordF) -> Vec<Polygon> {
    offset_polygons_clib(polygons, -mm_to_scaled_f32(delta_mm.abs()), OffsetJoinType::Miter)
}

/// `closing(const Polygons &, delta1, delta2)` (ClipperUtils.cpp:592-597):
/// shrink_paths<Paths>(expand_paths<Paths>(polygons, delta1), delta2).
pub fn closing_polygons_clib(polygons: &[Polygon], delta1_mm: CoordF, delta2_mm: CoordF) -> Vec<Polygon> {
    let grown = offset_polygons_clib(polygons, mm_to_scaled_f32(delta1_mm.abs()), OffsetJoinType::Miter);
    offset_polygons_clib(&grown, -mm_to_scaled_f32(delta2_mm.abs()), OffsetJoinType::Miter)
}

/// `opening(const Polygons &, delta1, delta2)` (ClipperUtils.cpp:606-613):
/// expand_paths<Paths>(shrink_paths<Paths>(polygons, delta1), delta2).
pub fn opening_polygons_clib(polygons: &[Polygon], delta1_mm: CoordF, delta2_mm: CoordF) -> Vec<Polygon> {
    let shrunk = offset_polygons_clib(polygons, -mm_to_scaled_f32(delta1_mm.abs()), OffsetJoinType::Miter);
    offset_polygons_clib(&shrunk, mm_to_scaled_f32(delta2_mm.abs()), OffsetJoinType::Miter)
}

/// Difference of raw polygon sets with Polygons output.
/// ClipperUtils C++: `Polygons diff(const Polygons &subject, const Polygons &clip, ApplySafetyOffset)`
/// (default ApplySafetyOffset::No). Hole semantics follow the NonZero fill
/// rule: reversed (hole) rings in either operand cancel enclosing area, which
/// `union_polygons_ex` reproduces before the ExPolygons difference.
pub fn diff_polygons(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
    if subject.is_empty() {
        return Vec::new();
    }
    if polygons_ops_clib() {
        return diff_polygons_raw_clib(subject, clip);
    }
    let subject_ex = union_polygons_ex(subject);
    if clip.is_empty() {
        return crate::geometry::to_polygons(&subject_ex);
    }
    let clip_ex = union_polygons_ex(clip);
    crate::geometry::to_polygons(&difference(&subject_ex, &clip_ex))
}

/// Difference of raw polygon sets with ExPolygons output.
/// ClipperUtils C++: `ExPolygons diff_ex(const Polygons &subject, const Polygons &clip, ApplySafetyOffset)`.
/// The safety-offset flag is accepted for signature parity; like the other
/// `diff_ex` variants in this module, the POLYGONS_OPS_CLIB=0 arm ignores it.
pub fn diff_ex_polygons_polygons(
    subject: &[Polygon],
    clip: &[Polygon],
    _safety_offset: ApplySafetyOffset,
) -> ExPolygons {
    if subject.is_empty() {
        return Vec::new();
    }
    if polygons_ops_clib() {
        return if matches!(_safety_offset, ApplySafetyOffset::Yes) {
            difference_clib_safety(&polygons_as_paths(subject), &polygons_as_paths(clip))
        } else {
            polytree_ex_clib(&diff_polygons_raw_clib(subject, clip))
        };
    }
    let subject_ex = union_polygons_ex(subject);
    if clip.is_empty() {
        return subject_ex;
    }
    let clip_ex = union_polygons_ex(clip);
    difference(&subject_ex, &clip_ex)
}

/// Intersection of raw polygon sets with ExPolygons output.
/// ClipperUtils C++: `ExPolygons intersection_ex(const Polygons &subject, const Polygons &clip, ApplySafetyOffset)`.
pub fn intersection_ex_polygons_polygons(
    subject: &[Polygon],
    clip: &[Polygon],
    _safety_offset: ApplySafetyOffset,
) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return Vec::new();
    }
    if polygons_ops_clib() {
        return if matches!(_safety_offset, ApplySafetyOffset::Yes) {
            intersection_clib_safety(&polygons_as_paths(subject), &polygons_as_paths(clip))
        } else {
            polytree_ex_clib(&intersection_polygons_raw_clib(subject, clip))
        };
    }
    let subject_ex = union_polygons_ex(subject);
    let clip_ex = union_polygons_ex(clip);
    intersection(&subject_ex, &clip_ex)
}

/// Intersection of ExPolygons with raw polygons, ExPolygons output.
/// ClipperUtils C++: `ExPolygons intersection_ex(const ExPolygons &subject, const Polygons &clip)`.
pub fn intersection_ex_expolygons_polygons(
    subject: &[ExPolygon],
    clip: &[Polygon],
) -> ExPolygons {
    if subject.is_empty() || clip.is_empty() {
        return Vec::new();
    }
    if polygons_ops_clib() {
        return polytree_ex_clib(&intersection_polygons_raw_clib(
            &crate::geometry::to_polygons(subject),
            clip,
        ));
    }
    let clip_ex = union_polygons_ex(clip);
    intersection(subject, &clip_ex)
}

/// Outward offset of raw polygons with Polygons output.
/// ClipperUtils C++: `Polygons expand(const Polygons &polygons, const float delta, ...)`.
/// `delta_mm` is in millimeters (this module's offset backend convention; the
/// C++ callers pass scaled floats and the call sites unscale).
pub fn expand_polygons(polygons: &[Polygon], delta_mm: CoordF) -> Vec<Polygon> {
    if polygons.is_empty() {
        return Vec::new();
    }
    if polygons_ops_clib() {
        return offset_polygons_clib(polygons, mm_to_scaled_f32(delta_mm.abs()), OffsetJoinType::Miter);
    }
    crate::geometry::to_polygons(&offset_polygons(
        polygons,
        delta_mm.abs(),
        OffsetJoinType::Miter,
    ))
}

/// Two-radius morphological opening on raw polygons with Polygons output.
/// ClipperUtils C++: `Polygons opening(const Polygons &polygons, const float delta1, const float delta2, ...)`
/// = shrink by `delta1`, then grow by `delta2`. Deltas in millimeters (see
/// [`expand_polygons`]).
pub fn opening_polygons_2(
    polygons: &[Polygon],
    delta1_mm: CoordF,
    delta2_mm: CoordF,
) -> Vec<Polygon> {
    if polygons.is_empty() {
        return Vec::new();
    }
    if polygons_ops_clib() {
        return opening_polygons_clib(polygons, delta1_mm, delta2_mm);
    }
    let shrunk = offset_polygons(polygons, -delta1_mm.abs(), OffsetJoinType::Miter);
    crate::geometry::to_polygons(&offset_expolygons(
        &shrunk,
        delta2_mm.abs(),
        OffsetJoinType::Miter,
    ))
}

/// `clip_clipper_polygons_with_subject_bbox` over ExPolygons.
/// ClipperUtils.cpp:161-172 `Polygons clip_clipper_polygons_with_subject_bbox(const ExPolygons &src, const BoundingBox &bbox, const bool get_entire_polygons)`.
pub fn clip_clipper_polygons_with_subject_bbox_expolygons(
    src: &[ExPolygon],
    bbox: &BoundingBox,
    get_entire_polygons: bool,
) -> Vec<Polygon> {
    // ClipperUtils.cpp:163-164
    let mut out: Vec<Polygon> = Vec::new();
    // ClipperUtils.cpp:165-168
    for p in src {
        let temp = clip_clipper_polygons_with_subject_bbox_expolygon(p, bbox, get_entire_polygons);
        out.extend(temp);
    }
    // ClipperUtils.cpp:170
    out.retain(|polygon| !polygon.is_empty());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use crate::{scale, Coord};

    // R117: scale_-faithful helper matches the known native scale_ constants.
    #[test]
    fn r117_scale_faithful_matches_native() {
        assert_eq!(scale_faithful(0.45), 44999); // ext_perimeter_spacing
        assert_eq!(scale_faithful(0.5), 49999); // L0 external width
        assert_eq!(scale_faithful(0.42), 41999); // L1-239 external width
        assert_eq!(scale_faithful(0.44), 43999);
        assert_eq!(scale_faithful(0.56), 56000);
    }

    // R117: bridge_offset = 1.5·max(scale_(eps),scale_(pw)) = 67498.5 (benchy
    // eps=0.45 dominates), and offset_delta_mm_from_scaled_f32 round-trips through
    // offset_expolygons_clib's ×SCALING_FACTOR to that exact value.
    #[test]
    fn r117_bridge_offset_delta_faithful() {
        let bo = 1.5 * scale_faithful(0.45) as f64;
        assert_eq!(bo, 67498.5);
        let dmm = offset_delta_mm_from_scaled_f32(bo);
        assert_eq!(dmm * crate::SCALING_FACTOR, 67498.5);
    }

    fn make_square(x: Coord, y: Coord, size: Coord) -> ExPolygon {
        let poly = Polygon::rectangle(Point::new(x, y), Point::new(x + size, y + size));
        poly.into()
    }

    fn make_square_mm(x: f64, y: f64, size: f64) -> ExPolygon {
        make_square(crate::scale(x), crate::scale(y), crate::scale(size))
    }

    #[test]
    fn test_difference_clib_ring() {
        // 20mm square minus a centered 10mm square => a ring: one ExPolygon with
        // one hole, area ≈ 400 - 100 = 300 mm².
        let outer = make_square_mm(0.0, 0.0, 20.0);
        let inner = make_square_mm(5.0, 5.0, 10.0);
        let result = difference_clib(&[outer], &[inner]);
        assert_eq!(result.len(), 1, "ring should be a single ExPolygon, got {result:?}");
        assert_eq!(result[0].holes.len(), 1, "ring should have exactly one hole");
        let area_mm2: CoordF =
            result.iter().map(|e| e.area()).sum::<CoordF>()
                / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        assert!(
            (area_mm2 - 300.0).abs() < 1.0,
            "ring area should be ~300 mm², got {area_mm2}"
        );
    }

    #[test]
    fn test_difference_clib_empty_clip_passthrough() {
        // Empty clip => subject returned unchanged (matches diff with no clip).
        let outer = make_square_mm(0.0, 0.0, 10.0);
        let result = difference_clib(&[outer.clone()], &[]);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].holes.len(), 0);
    }

    #[test]
    fn test_offset_polygon_grow() {
        let square = Polygon::rectangle(
            Point::new(crate::scale(10.0), crate::scale(10.0)),
            Point::new(crate::scale(20.0), crate::scale(20.0)),
        );

        // Original area: 10mm * 10mm = 100mm² (area() returns scaled² units as CoordF)
        let original_area = square.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        assert!((original_area - 100.0).abs() < 1.0);

        // Grow by 1mm
        let grown = offset_polygon(&square, 1.0, OffsetJoinType::Square);
        assert!(!grown.is_empty());

        // New area should be larger (approximately 12mm * 12mm = 144mm² for square join)
        let grown_area: CoordF = grown.iter().map(|p| p.area()).sum();
        let grown_area_mm2 = grown_area / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        assert!(grown_area_mm2 > original_area);
    }

    #[test]
    fn test_offset_polygon_shrink() {
        let square = Polygon::rectangle(
            Point::new(crate::scale(10.0), crate::scale(10.0)),
            Point::new(crate::scale(30.0), crate::scale(30.0)),
        );

        // Shrink by 2mm
        let shrunk = offset_polygon(&square, -2.0, OffsetJoinType::Square);
        assert!(!shrunk.is_empty());

        // New area should be smaller (approximately 16mm * 16mm = 256mm²)
        let original_area = square.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        let shrunk_area: CoordF = shrunk.iter().map(|p| p.area()).sum();
        let shrunk_area_mm2 = shrunk_area / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        assert!(shrunk_area_mm2 < original_area);
    }

    #[test]
    fn test_offset_polygon_shrink_to_nothing() {
        // Small 2mm x 2mm square
        let square = Polygon::rectangle(
            Point::new(crate::scale(10.0), crate::scale(10.0)),
            Point::new(crate::scale(12.0), crate::scale(12.0)),
        );

        // Shrink by 2mm should eliminate it
        let shrunk = offset_polygon(&square, -2.0, OffsetJoinType::Square);
        assert!(shrunk.is_empty() || shrunk.iter().all(|p| p.area().abs() < 1e-6));
    }

    #[test]
    fn test_union() {
        // Two overlapping squares
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(5.0, 0.0, 10.0);

        let result = union(&[square1.clone()], &[square2.clone()]);
        assert!(!result.is_empty());

        // Union area should be less than sum of individual areas (due to overlap)
        let area1 = square1.area();
        let area2 = square2.area();
        let union_area: CoordF = result.iter().map(|p| p.area()).sum();
        assert!(union_area < area1 + area2);
        assert!(union_area > area1.max(area2));
    }

    #[test]
    fn test_intersection() {
        // Two overlapping squares
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(5.0, 0.0, 10.0);

        let result = intersection(&[square1], &[square2]);
        assert!(!result.is_empty());

        // Intersection should be a 5mm x 10mm rectangle
        let int_area: CoordF = result.iter().map(|p| p.area()).sum();
        assert!(int_area > 0.0);
    }

    #[test]
    fn test_intersection_no_overlap() {
        // Two non-overlapping squares
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(20.0, 0.0, 10.0);

        let result = intersection(&[square1], &[square2]);
        assert!(result.is_empty() || result.iter().all(|p| p.area().abs() < 1e-6));
    }

    #[test]
    fn test_difference() {
        // Large square minus smaller square inside
        let large = make_square_mm(0.0, 0.0, 20.0);
        let small = make_square_mm(5.0, 5.0, 10.0);

        let result = difference(&[large.clone()], &[small.clone()]);
        assert!(!result.is_empty());

        // Difference area should be large area - small area
        let diff_area: CoordF = result.iter().map(|p| p.area()).sum();
        let expected_area = large.area() - small.area();
        assert!((diff_area - expected_area).abs() / expected_area < 0.01); // 1% tolerance
    }

    #[test]
    fn test_shrink_grow_convenience() {
        let square = make_square_mm(10.0, 10.0, 20.0);
        let original_area = square.area();

        let shrunk = shrink(&[square.clone()], 2.0, OffsetJoinType::Round);
        let shrunk_area: CoordF = shrunk.iter().map(|p| p.area()).sum();
        assert!(shrunk_area < original_area);

        let grown = grow(&[square], 2.0, OffsetJoinType::Round);
        let grown_area: CoordF = grown.iter().map(|p| p.area()).sum();
        assert!(grown_area > original_area);
    }

    #[test]
    fn test_expolygon_with_hole() {
        // Create a square with a hole
        let outer = Polygon::rectangle(
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(20.0)),
        );
        let inner = Polygon::rectangle(
            Point::new(crate::scale(5.0), crate::scale(5.0)),
            Point::new(crate::scale(15.0), crate::scale(15.0)),
        );
        let expoly = ExPolygon::with_holes(outer, vec![inner]);

        // Shrink should maintain the hole
        let shrunk = offset_expolygon(&expoly, -1.0, OffsetJoinType::Square);
        assert!(!shrunk.is_empty());
    }

    #[test]
    fn test_total_area() {
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(20.0, 0.0, 10.0);

        let total = total_area(&[square1.clone(), square2.clone()]);
        let expected = square1.area() + square2.area();
        assert!((total - expected).abs() < 1e-6);
    }

    #[test]
    fn test_polygons_overlap() {
        let square1 = make_square_mm(0.0, 0.0, 10.0);
        let square2 = make_square_mm(5.0, 0.0, 10.0);
        let square3 = make_square_mm(20.0, 0.0, 10.0);

        assert!(polygons_overlap(&[square1.clone()], &[square2]));
        assert!(!polygons_overlap(&[square1], &[square3]));
    }

    #[test]
    fn test_remove_small() {
        let large = make_square_mm(0.0, 0.0, 10.0);
        let small = make_square_mm(20.0, 0.0, 0.1);

        let polys = vec![large.clone(), small];
        // area() returns scaled² units, so 1mm² = SCALING_FACTOR²
        let min_area_scaled = 1.0 * crate::SCALING_FACTOR * crate::SCALING_FACTOR;
        let filtered = remove_small(&polys, min_area_scaled);

        assert_eq!(filtered.len(), 1);
        assert!((filtered[0].area() - large.area()).abs() < 1e-6);
    }

    #[test]
    fn test_opening() {
        // Opening removes small protrusions
        let square = make_square_mm(0.0, 0.0, 10.0);
        let original_area = square.area();

        // Opening with small distance should approximately preserve area
        let opened = opening(&[square], 0.1, OffsetJoinType::Round);
        assert!(!opened.is_empty());

        let opened_area: CoordF = opened.iter().map(|p| p.area()).sum();
        // Area should be slightly smaller due to corner rounding
        assert!(opened_area > 0.0);
        assert!(opened_area <= original_area * 1.01); // Allow small tolerance
    }

    #[test]
    fn test_closing() {
        // Closing fills small gaps
        let square = make_square_mm(0.0, 0.0, 10.0);
        let original_area = square.area();

        // Closing with small distance should approximately preserve area
        let closed = closing(&[square], 0.1, OffsetJoinType::Round);
        assert!(!closed.is_empty());

        let closed_area: CoordF = closed.iter().map(|p| p.area()).sum();
        // Area should be slightly larger due to corner filling
        assert!(closed_area > 0.0);
        assert!(closed_area >= original_area * 0.99); // Allow small tolerance
    }

    #[test]
    fn test_offset2() {
        let square = make_square_mm(0.0, 0.0, 10.0);
        let original_area = square.area();

        // offset2 with equal shrink/grow should approximately preserve shape
        let result = offset2(&[square], 0.5, 0.5, OffsetJoinType::Round);
        assert!(!result.is_empty());

        let result_area: CoordF = result.iter().map(|p| p.area()).sum();
        // Should be roughly similar (some corner effects expected)
        assert!(result_area > original_area * 0.8);
    }

    // NOTE: vertex-exactness vs geo-clipper for offset_expolygons_clib / offset2_clib
    // is verified in the standalone integration test
    // `tests/clib_offset_vertex_exact.rs` (the in-lib `#[cfg(test)]` target does
    // not currently compile due to unrelated pre-existing test modules).

    #[test]
    fn test_offset2_removes_thin_features() {
        // Create a shape with a thin protrusion
        // offset2 should remove it
        let thin_protrusion = Polygon::from_points(vec![
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(10.0)),
            Point::new(crate::scale(5.5), crate::scale(10.0)), // Thin protrusion starts
            Point::new(crate::scale(5.5), crate::scale(11.0)), // 0.5mm wide, 1mm tall
            Point::new(crate::scale(4.5), crate::scale(11.0)),
            Point::new(crate::scale(4.5), crate::scale(10.0)), // Thin protrusion ends
            Point::new(crate::scale(0.0), crate::scale(10.0)),
        ]);
        let expoly: ExPolygon = thin_protrusion.into();

        // offset2 with 1mm shrink/grow should remove the 0.5mm wide protrusion
        let result = offset2(&[expoly], 1.0, 1.0, OffsetJoinType::Round);

        // Result should exist but be simpler (protrusion removed)
        if !result.is_empty() {
            let result_area: CoordF = result.iter().map(|p| p.area()).sum();
            // Should be close to 100mm² (the main square)
            let expected_area = 100.0 * crate::SCALING_FACTOR * crate::SCALING_FACTOR;
            assert!(result_area < expected_area * 1.1);
        }
    }

    #[test]
    fn test_detect_gaps_no_gaps() {
        // Two concentric squares - no gaps should be detected
        let outer = make_square_mm(0.0, 0.0, 20.0);
        let inner = make_square_mm(2.0, 2.0, 16.0);

        let gaps = detect_gaps(&[outer], &[inner], 0.2, 2.0, OffsetJoinType::Round);

        // Depending on geometry, gaps may or may not be found
        // The key is that the function doesn't crash
        println!("Detected {} gap regions", gaps.len());
    }

    #[test]
    fn test_detect_gaps_empty_inputs() {
        let square = make_square_mm(0.0, 0.0, 10.0);

        // Empty outer should return empty
        let gaps1 = detect_gaps(&[], &[square.clone()], 0.2, 2.0, OffsetJoinType::Round);
        assert!(gaps1.is_empty());

        // Empty inner should work (everything is a gap)
        let gaps2 = detect_gaps(&[square], &[], 0.2, 2.0, OffsetJoinType::Round);
        // Result depends on parameters, but shouldn't crash
        println!("Gaps with empty inner: {}", gaps2.len());
    }

    #[test]
    fn test_extract_centerlines_simple() {
        // A narrow rectangle should produce a centerline along its length
        let thin_rect = Polygon::from_points(vec![
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(1.0)), // 1mm wide
            Point::new(crate::scale(0.0), crate::scale(1.0)),
        ]);
        let expoly: ExPolygon = thin_rect.into();

        let centerlines = extract_centerlines(&[expoly], 1.0, OffsetJoinType::Round);

        // Should produce at least one centerline
        println!("Extracted {} centerlines", centerlines.len());
        // The function should not crash and should return something
    }

    #[test]
    fn test_extract_centerlines_empty() {
        let centerlines = extract_centerlines(&[], 1.0, OffsetJoinType::Round);
        assert!(centerlines.is_empty());
    }

    #[test]
    fn test_opening_empty_input() {
        let result = opening(&[], 1.0, OffsetJoinType::Round);
        assert!(result.is_empty());
    }

    #[test]
    fn test_closing_empty_input() {
        let result = closing(&[], 1.0, OffsetJoinType::Round);
        assert!(result.is_empty());
    }

    #[test]
    fn test_offset2_empty_input() {
        let result = offset2(&[], 1.0, 1.0, OffsetJoinType::Round);
        assert!(result.is_empty());
    }

    #[test]
    fn test_diff_pl_empty_polylines() {
        let polylines: Vec<Polyline> = Vec::new();
        let clip = vec![make_square_mm(0.0, 0.0, 10.0)];
        let result = diff_pl(&polylines, &clip);
        assert!(result.is_empty());
    }

    #[test]
    fn test_diff_pl_empty_clip() {
        let pts = vec![
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(0.0)),
        ];
        let polylines = vec![Polyline::from_points(pts)];
        let clip: Vec<ExPolygon> = Vec::new();

        let result = diff_pl(&polylines, &clip);
        // With no clip regions, original polylines should be returned
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_diff_pl_line_outside_clip() {
        // Line completely outside the clip region
        let pts = vec![
            Point::new(scale(50.0), scale(50.0)),
            Point::new(scale(60.0), scale(50.0)),
        ];
        let polylines = vec![Polyline::from_points(pts)];
        let clip = vec![make_square_mm(0.0, 0.0, 10.0)];

        let result = diff_pl(&polylines, &clip);
        // Line is outside, should be kept
        assert!(!result.is_empty());
    }

    #[test]
    fn test_diff_pl_line_inside_clip() {
        // Line completely inside the clip region
        let pts = vec![
            Point::new(scale(2.0), scale(2.0)),
            Point::new(scale(8.0), scale(2.0)),
        ];
        let polylines = vec![Polyline::from_points(pts)];
        let clip = vec![make_square_mm(0.0, 0.0, 10.0)];

        let result = diff_pl(&polylines, &clip);
        // Line is inside, should be removed (empty result)
        assert!(result.is_empty());
    }

    #[test]
    fn test_polygons_to_polylines() {
        let square = Polygon::rectangle(
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(10.0)),
        );
        let polygons = vec![square];

        let result = polygons_to_polylines(&polygons);
        assert_eq!(result.len(), 1);
        assert!(result[0].len() >= 4); // At least 4 points for a rectangle
    }

    #[test]
    fn test_polygons_to_polylines_empty() {
        let polygons: Vec<Polygon> = Vec::new();
        let result = polygons_to_polylines(&polygons);
        assert!(result.is_empty());
    }

    #[test]
    fn test_expolygons_to_polylines_with_holes() {
        // Create an expolygon with a hole
        let outer = Polygon::rectangle(
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(20.0), scale(20.0)),
        );
        let hole = Polygon::rectangle(
            Point::new(scale(5.0), scale(5.0)),
            Point::new(scale(15.0), scale(15.0)),
        );
        let expoly = ExPolygon::with_holes(outer, vec![hole]);
        let expolygons = vec![expoly];

        let result = expolygons_to_polylines(&expolygons);
        // Should have 2 polylines: one for outer contour, one for hole
        assert_eq!(result.len(), 2);
    }
}

// ============================================================================
// ClipperUtils.hpp / ClipperUtils.cpp under their C++ names
// ============================================================================

/// ClipperUtils.hpp and ClipperUtils.cpp as the pin declares them: the paths providers of `namespace
/// ClipperUtils`, the templates of ClipperUtils.cpp, and every overload of every function under its C++ name
/// (an overload set is a trait, one implementation per overload, so the name is the C++ one), over the port
/// of ClipperLib (`crate::clipper`). The functions above this module are the earlier, differently named entry
/// points of the crate; they stay as they are.
#[allow(non_snake_case, non_camel_case_types, clippy::too_many_arguments, clippy::type_complexity)]
pub mod cpp {
    use super::ApplySafetyOffset;
    use crate::clipper::{ClipType, EndType, IntPoint, JoinType, Path, Paths, PolyFillType, PolyTree};
    use crate::geometry::{ExPolygon, ExPolygons, Line, Lines, Point, Points, Polygon, Polygons, Polyline, Polylines};
    use crate::surface::Surface;

    /// The bounding box the functions of this module take: its `min` and `max` are read, as the C++ reads them.
    pub type BoundingBox = crate::geometry::BoundingBox;

    /// ClipperUtils.hpp:29 `static constexpr const float ClipperSafetyOffset = 10.f;`
    pub const CLIPPER_SAFETY_OFFSET: f32 = 10.;
    /// ClipperUtils.hpp:31 `DefaultJoinType`
    pub const DEFAULT_JOIN_TYPE: JoinType = JoinType::jtMiter;
    /// ClipperUtils.hpp:32 `DefaultEndType`
    pub const DEFAULT_END_TYPE: EndType = EndType::etOpenButt;
    /// ClipperUtils.hpp:33-37 `DefaultMiterLimit`
    //FIXME evaluate the default miter limit. 3 seems to be extreme, Cura uses 1.2.
    // Mitter Limit 3 is useful for perimeter generator, where sharp corners are extruded without needing a gap fill.
    // However such a high limit causes issues with large positive or negative offsets, where a sharp corner
    // is extended excessively.
    pub const DEFAULT_MITER_LIMIT: f64 = 3.;
    /// ClipperUtils.hpp:39 `DefaultLineJoinType`
    pub const DEFAULT_LINE_JOIN_TYPE: JoinType = JoinType::jtSquare;
    /// ClipperUtils.hpp:40-41 `DefaultLineMiterLimit`: Miter limit is ignored for jtSquare.
    pub const DEFAULT_LINE_MITER_LIMIT: f64 = 0.;
    /// ClipperUtils.hpp:43-44 `ClipperOffsetShortestEdgeFactor`: Decimation factor applied on input contour when
    /// doing offset, multiplied by the offset distance.
    pub const CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR: f64 = 0.005;
    /// libslic3r.h:58 `SCALED_EPSILON`: `scale_(1e-4)`
    const SCALED_EPSILON: f64 = 1e-4 / 0.00001;

    // `ClipperLib::Path` of a `Points`: the two are one type at the pin (clipper.hpp `typedef Slic3r::Points Path`).
    fn path_of_points(points: &[Point]) -> Path {
        points.iter().map(|p| IntPoint::new(p.x as i32, p.y as i32)).collect()
    }
    fn points_of_path(path: &Path) -> Points {
        path.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()
    }

    // Polygon.hpp:267-274 `inline Polygons to_polygons(const std::vector<Points> &paths)`: a polygon of every path.
    fn to_polygons(paths: Paths) -> Polygons {
        let mut out: Polygons = Vec::with_capacity(paths.len());
        for path in &paths {
            out.push(Polygon::from_points(points_of_path(path)));
        }
        out
    }

    // ClipperUtils.cpp:175-211 `static ExPolygons PolyTreeToExPolygons(ClipperLib::PolyTree &&polytree)`
    fn poly_tree_to_ex_polygons(polytree: PolyTree) -> ExPolygons {
        super::poly_tree_to_ex_polygons(&polytree)
    }

    // ClipperUtils.cpp:213-229 `Polylines PolyTreeToPolylines(ClipperLib::PolyTree &&polytree)`
    fn poly_tree_to_polylines(polytree: PolyTree) -> Polylines {
        super::poly_tree_to_polylines(&polytree).iter().map(|path| Polyline::from_points(points_of_path(path))).collect()
    }

    // ============================================================================
    // namespace ClipperUtils: the paths providers (ClipperUtils.hpp:51-314)
    // ============================================================================

    /// What the templates of ClipperUtils.cpp take as a `PathsProvider`: the paths ClipperLib reads from it, in the
    /// order of its iterator. A `ClipperLib::Paths` is one too.
    pub trait Provider {
        fn to_clipper_paths(&self) -> Paths;
    }
    impl Provider for Paths {
        fn to_clipper_paths(&self) -> Paths {
            self.clone()
        }
    }

    /// ClipperUtils.hpp:52-59 `class PathsProviderIteratorBase` and the `iterator` of every provider: a position in
    /// the paths of the provider. `cbegin()` is the first position, `cend()` the one behind the last.
    #[derive(Debug, Clone)]
    pub struct ProviderIterator<'a> {
        m_paths: Vec<&'a [Point]>,
        m_idx: usize,
    }
    impl<'a> ProviderIterator<'a> {
        /// `const Points& operator*() const`
        pub fn get(&self) -> &'a [Point] {
            self.m_paths[self.m_idx]
        }
        /// `iterator& operator++()`
        pub fn inc(&mut self) -> &mut Self {
            self.m_idx += 1;
            self
        }
    }
    impl PartialEq for ProviderIterator<'_> {
        /// `bool operator==(const iterator &rhs) const`
        fn eq(&self, rhs: &Self) -> bool {
            self.m_idx == rhs.m_idx
        }
    }
    impl<'a> Iterator for ProviderIterator<'a> {
        type Item = &'a [Point];
        /// `const Points& operator++(int)`, until `cend()`
        fn next(&mut self) -> Option<&'a [Point]> {
            if self.m_idx < self.m_paths.len() {
                let out = self.m_paths[self.m_idx];
                self.m_idx += 1;
                Some(out)
            } else {
                None
            }
        }
    }
    fn provider_paths<'a>(begin: ProviderIterator<'a>) -> Paths {
        begin.map(path_of_points).collect()
    }

    /// ClipperUtils.hpp:61-81 `class EmptyPathsProvider`
    #[derive(Debug, Clone, Copy, Default)]
    pub struct EmptyPathsProvider;
    impl EmptyPathsProvider {
        pub fn new() -> Self {
            EmptyPathsProvider
        }
        // all iterators point to end.
        pub fn cend(&self) -> ProviderIterator<'static> {
            ProviderIterator { m_paths: Vec::new(), m_idx: 0 }
        }
        pub fn end(&self) -> ProviderIterator<'static> {
            self.cend()
        }
        pub fn cbegin(&self) -> ProviderIterator<'static> {
            self.cend()
        }
        pub fn begin(&self) -> ProviderIterator<'static> {
            self.cend()
        }
        pub fn size(&self) -> usize {
            0
        }
    }
    impl Provider for EmptyPathsProvider {
        fn to_clipper_paths(&self) -> Paths {
            provider_paths(self.begin())
        }
    }

    /// ClipperUtils.hpp:83-108 `class SinglePathProvider`
    #[derive(Debug, Clone, Copy)]
    pub struct SinglePathProvider<'a> {
        m_points: &'a [Point],
    }
    impl<'a> SinglePathProvider<'a> {
        pub fn new(points: &'a [Point]) -> Self {
            SinglePathProvider { m_points: points }
        }
        pub fn cbegin(&self) -> ProviderIterator<'a> {
            ProviderIterator { m_paths: vec![self.m_points], m_idx: 0 }
        }
        pub fn begin(&self) -> ProviderIterator<'a> {
            self.cbegin()
        }
        pub fn cend(&self) -> ProviderIterator<'a> {
            ProviderIterator { m_paths: vec![self.m_points], m_idx: 1 }
        }
        pub fn end(&self) -> ProviderIterator<'a> {
            self.cend()
        }
        pub fn size(&self) -> usize {
            1
        }
    }
    impl Provider for SinglePathProvider<'_> {
        fn to_clipper_paths(&self) -> Paths {
            provider_paths(self.begin())
        }
    }

    /// ClipperUtils.hpp:110-135 `template<typename PathType> class PathsProvider`, of `Points`
    #[derive(Debug, Clone, Copy)]
    pub struct PathsProvider<'a> {
        m_paths: &'a [Points],
    }
    impl<'a> PathsProvider<'a> {
        pub fn new(paths: &'a [Points]) -> Self {
            PathsProvider { m_paths: paths }
        }
        pub fn cbegin(&self) -> ProviderIterator<'a> {
            ProviderIterator { m_paths: self.m_paths.iter().map(|p| p.as_slice()).collect(), m_idx: 0 }
        }
        pub fn begin(&self) -> ProviderIterator<'a> {
            self.cbegin()
        }
        pub fn cend(&self) -> ProviderIterator<'a> {
            let mut it = self.cbegin();
            it.m_idx = it.m_paths.len();
            it
        }
        pub fn end(&self) -> ProviderIterator<'a> {
            self.cend()
        }
        pub fn size(&self) -> usize {
            self.m_paths.len()
        }
    }
    impl Provider for PathsProvider<'_> {
        fn to_clipper_paths(&self) -> Paths {
            provider_paths(self.begin())
        }
    }

    /// What `MultiPointsProvider` is made over: a `MultiPoint`, its `points`.
    pub trait MultiPointType {
        fn points(&self) -> &[Point];
    }
    impl MultiPointType for Polygon {
        fn points(&self) -> &[Point] {
            &self.points
        }
    }
    impl MultiPointType for Polyline {
        fn points(&self) -> &[Point] {
            &self.points
        }
    }

    /// ClipperUtils.hpp:137-162 `template<typename MultiPointType> class MultiPointsProvider`
    #[derive(Debug)]
    pub struct MultiPointsProvider<'a, M: MultiPointType> {
        m_multipoints: &'a [M],
    }
    impl<'a, M: MultiPointType> MultiPointsProvider<'a, M> {
        pub fn new(multipoints: &'a [M]) -> Self {
            MultiPointsProvider { m_multipoints: multipoints }
        }
        pub fn cbegin(&self) -> ProviderIterator<'a> {
            ProviderIterator { m_paths: self.m_multipoints.iter().map(|m| m.points()).collect(), m_idx: 0 }
        }
        pub fn begin(&self) -> ProviderIterator<'a> {
            self.cbegin()
        }
        pub fn cend(&self) -> ProviderIterator<'a> {
            let mut it = self.cbegin();
            it.m_idx = it.m_paths.len();
            it
        }
        pub fn end(&self) -> ProviderIterator<'a> {
            self.cend()
        }
        pub fn size(&self) -> usize {
            self.m_multipoints.len()
        }
    }
    impl<M: MultiPointType> Provider for MultiPointsProvider<'_, M> {
        fn to_clipper_paths(&self) -> Paths {
            provider_paths(self.begin())
        }
    }

    /// ClipperUtils.hpp:164 `using PolygonsProvider = MultiPointsProvider<Polygon>;`
    pub type PolygonsProvider<'a> = MultiPointsProvider<'a, Polygon>;
    /// ClipperUtils.hpp:165 `using PolylinesProvider = MultiPointsProvider<Polyline>;`
    pub type PolylinesProvider<'a> = MultiPointsProvider<'a, Polyline>;

    // The contour of an expolygon, then its holes.
    fn expolygon_paths<'a>(expoly: &'a ExPolygon, out: &mut Vec<&'a [Point]>) {
        out.push(&expoly.contour.points);
        for hole in &expoly.holes {
            out.push(&hole.points);
        }
    }

    /// ClipperUtils.hpp:167-191 `struct ExPolygonProvider`
    #[derive(Debug, Clone, Copy)]
    pub struct ExPolygonProvider<'a> {
        m_expoly: &'a ExPolygon,
    }
    impl<'a> ExPolygonProvider<'a> {
        pub fn new(expoly: &'a ExPolygon) -> Self {
            ExPolygonProvider { m_expoly: expoly }
        }
        pub fn cbegin(&self) -> ProviderIterator<'a> {
            let mut paths = Vec::with_capacity(self.size());
            expolygon_paths(self.m_expoly, &mut paths);
            ProviderIterator { m_paths: paths, m_idx: 0 }
        }
        pub fn begin(&self) -> ProviderIterator<'a> {
            self.cbegin()
        }
        pub fn cend(&self) -> ProviderIterator<'a> {
            let mut it = self.cbegin();
            it.m_idx = self.m_expoly.holes.len() + 1;
            it
        }
        pub fn end(&self) -> ProviderIterator<'a> {
            self.cend()
        }
        pub fn size(&self) -> usize {
            self.m_expoly.holes.len() + 1
        }
    }
    impl Provider for ExPolygonProvider<'_> {
        fn to_clipper_paths(&self) -> Paths {
            provider_paths(self.begin())
        }
    }

    /// ClipperUtils.hpp:193-232 `struct ExPolygonsProvider`
    #[derive(Debug, Clone, Copy)]
    pub struct ExPolygonsProvider<'a> {
        m_expolygons: &'a [ExPolygon],
        m_size: usize,
    }
    impl<'a> ExPolygonsProvider<'a> {
        pub fn new(expolygons: &'a [ExPolygon]) -> Self {
            let mut m_size = 0;
            for expoly in expolygons {
                m_size += expoly.holes.len() + 1;
            }
            ExPolygonsProvider { m_expolygons: expolygons, m_size }
        }
        pub fn cbegin(&self) -> ProviderIterator<'a> {
            let mut paths = Vec::with_capacity(self.m_size);
            for expoly in self.m_expolygons {
                expolygon_paths(expoly, &mut paths);
            }
            ProviderIterator { m_paths: paths, m_idx: 0 }
        }
        pub fn begin(&self) -> ProviderIterator<'a> {
            self.cbegin()
        }
        pub fn cend(&self) -> ProviderIterator<'a> {
            let mut it = self.cbegin();
            it.m_idx = it.m_paths.len();
            it
        }
        pub fn end(&self) -> ProviderIterator<'a> {
            self.cend()
        }
        pub fn size(&self) -> usize {
            self.m_size
        }
    }
    impl Provider for ExPolygonsProvider<'_> {
        fn to_clipper_paths(&self) -> Paths {
            provider_paths(self.begin())
        }
    }

    /// ClipperUtils.hpp:234-273 `struct SurfacesProvider`
    #[derive(Debug, Clone, Copy)]
    pub struct SurfacesProvider<'a> {
        m_surfaces: &'a [Surface],
        m_size: usize,
    }
    impl<'a> SurfacesProvider<'a> {
        pub fn new(surfaces: &'a [Surface]) -> Self {
            let mut m_size = 0;
            for surface in surfaces {
                m_size += surface.expolygon.holes.len() + 1;
            }
            SurfacesProvider { m_surfaces: surfaces, m_size }
        }
        pub fn cbegin(&self) -> ProviderIterator<'a> {
            let mut paths = Vec::with_capacity(self.m_size);
            for surface in self.m_surfaces {
                expolygon_paths(&surface.expolygon, &mut paths);
            }
            ProviderIterator { m_paths: paths, m_idx: 0 }
        }
        pub fn begin(&self) -> ProviderIterator<'a> {
            self.cbegin()
        }
        pub fn cend(&self) -> ProviderIterator<'a> {
            let mut it = self.cbegin();
            it.m_idx = it.m_paths.len();
            it
        }
        pub fn end(&self) -> ProviderIterator<'a> {
            self.cend()
        }
        pub fn size(&self) -> usize {
            self.m_size
        }
    }
    impl Provider for SurfacesProvider<'_> {
        fn to_clipper_paths(&self) -> Paths {
            provider_paths(self.begin())
        }
    }

    /// ClipperUtils.hpp:275-314 `struct SurfacesPtrProvider`
    #[derive(Debug, Clone, Copy)]
    pub struct SurfacesPtrProvider<'a> {
        m_surfaces: &'a [&'a Surface],
        m_size: usize,
    }
    impl<'a> SurfacesPtrProvider<'a> {
        pub fn new(surfaces: &'a [&'a Surface]) -> Self {
            let mut m_size = 0;
            for surface in surfaces {
                m_size += surface.expolygon.holes.len() + 1;
            }
            SurfacesPtrProvider { m_surfaces: surfaces, m_size }
        }
        pub fn cbegin(&self) -> ProviderIterator<'a> {
            let mut paths = Vec::with_capacity(self.m_size);
            for surface in self.m_surfaces {
                expolygon_paths(&surface.expolygon, &mut paths);
            }
            ProviderIterator { m_paths: paths, m_idx: 0 }
        }
        pub fn begin(&self) -> ProviderIterator<'a> {
            self.cbegin()
        }
        pub fn cend(&self) -> ProviderIterator<'a> {
            let mut it = self.cbegin();
            it.m_idx = it.m_paths.len();
            it
        }
        pub fn end(&self) -> ProviderIterator<'a> {
            self.cend()
        }
        pub fn size(&self) -> usize {
            self.m_size
        }
    }
    impl Provider for SurfacesPtrProvider<'_> {
        fn to_clipper_paths(&self) -> Paths {
            provider_paths(self.begin())
        }
    }

    // ============================================================================
    // namespace ClipperUtils: clip_clipper_polygon_with_subject_bbox (ClipperUtils.cpp:63-173)
    // ============================================================================

    /// ClipperUtils.hpp:317-319: For ClipperLib with Z coordinates. `using ZPoint = Vec3i32;`
    pub type ZPoint = [i32; 3];
    pub type ZPoints = Vec<ZPoint>;

    /// The `PointType` of `clip_clipper_polygon_with_subject_bbox_templ`: a Point or a ZPoint, its x and y.
    pub trait PointType: Clone {
        fn x(&self) -> i64;
        fn y(&self) -> i64;
    }
    impl PointType for Point {
        fn x(&self) -> i64 {
            self.x
        }
        fn y(&self) -> i64 {
            self.y
        }
    }
    impl PointType for ZPoint {
        fn x(&self) -> i64 {
            self[0] as i64
        }
        fn y(&self) -> i64 {
            self[1] as i64
        }
    }

    // Clip source polygon to be used as a clipping polygon with a bouding box around the source (to be clipped) polygon.
    // Useful as an optimization for expensive ClipperLib operations, for example when clipping source polygons one by one
    // with a set of polygons covering the whole layer below.
    // ClipperUtils.cpp:66-118
    // template<typename PointType> inline void clip_clipper_polygon_with_subject_bbox_templ(const std::vector<PointType> &src,
    //     const BoundingBox &bbox, std::vector<PointType> &out, const bool get_entire_polygons=false)
    pub fn clip_clipper_polygon_with_subject_bbox_templ<P: PointType>(src: &[P], bbox: &BoundingBox, out: &mut Vec<P>, get_entire_polygons: bool) {
        out.clear();
        let cnt = src.len();
        if cnt < 3 {
            return;
        }

        // enum class Side
        const LEFT: i32 = 1;
        const RIGHT: i32 = 2;
        const TOP: i32 = 4;
        const BOTTOM: i32 = 8;
        let sides = |p: &P| -> i32 {
            (p.x() < bbox.min.x) as i32 * LEFT + (p.x() > bbox.max.x) as i32 * RIGHT + (p.y() < bbox.min.y) as i32 * BOTTOM + (p.y() > bbox.max.y) as i32 * TOP
        };

        let mut sides_prev = sides(&src[cnt - 1]);
        let mut sides_this = sides(&src[0]);
        let last = cnt - 1;
        for i in 0..last {
            let sides_next = sides(&src[i + 1]);
            if
            // This point is inside. Take it.
            sides_this == 0 ||
                // Either this point is outside and previous or next is inside, or
                // the edge possibly cuts corner of the bounding box.
                (sides_prev & sides_this & sides_next) == 0
            {
                out.push(src[i].clone());
                sides_prev = sides_this;
            } else {
                // All the three points (this, prev, next) are outside at the same side.
                // Ignore this point.
            }
            sides_this = sides_next;
        }

        // Never produce just a single point output polygon.
        if !out.is_empty() {
            if get_entire_polygons {
                *out = src.to_vec();
            } else {
                let sides_next = sides(&out[0]);
                if
                // The last point is inside. Take it.
                sides_this == 0 ||
                    // Either this point is outside and previous or next is inside, or
                    // the edge possibly cuts corner of the bounding box.
                    (sides_prev & sides_this & sides_next) == 0
                {
                    out.push(src[cnt - 1].clone());
                }
            }
        }
    }

    /// The overloads of `clip_clipper_polygon_with_subject_bbox` that write into `out`.
    pub trait ClipClipperPolygonInto<Out: ?Sized> {
        fn clip_clipper_polygon_with_subject_bbox(&self, bbox: &BoundingBox, out: &mut Out, get_entire_polygons: bool);
    }
    // ClipperUtils.cpp:120
    // `void clip_clipper_polygon_with_subject_bbox(const Points &src, const BoundingBox &bbox, Points &out, const bool get_entire_polygons)`
    impl ClipClipperPolygonInto<Points> for [Point] {
        fn clip_clipper_polygon_with_subject_bbox(&self, bbox: &BoundingBox, out: &mut Points, get_entire_polygons: bool) {
            clip_clipper_polygon_with_subject_bbox_templ(self, bbox, out, get_entire_polygons);
        }
    }
    // ClipperUtils.cpp:121
    // `void clip_clipper_polygon_with_subject_bbox(const ZPoints &src, const BoundingBox &bbox, ZPoints &out)`: `get_entire_polygons`
    // is the default of the template, false, whatever is handed in here.
    impl ClipClipperPolygonInto<ZPoints> for [ZPoint] {
        fn clip_clipper_polygon_with_subject_bbox(&self, bbox: &BoundingBox, out: &mut ZPoints, _get_entire_polygons: bool) {
            clip_clipper_polygon_with_subject_bbox_templ(self, bbox, out, false);
        }
    }
    // ClipperUtils.cpp:133-135
    // `void clip_clipper_polygon_with_subject_bbox(const Polygon &src, const BoundingBox &bbox, Polygon &out)`: `get_entire_polygons`
    // is the default of the overload of Points, false, whatever is handed in here.
    impl ClipClipperPolygonInto<Polygon> for Polygon {
        fn clip_clipper_polygon_with_subject_bbox(&self, bbox: &BoundingBox, out: &mut Polygon, _get_entire_polygons: bool) {
            ClipClipperPolygonInto::clip_clipper_polygon_with_subject_bbox(self.points.as_slice(), bbox, &mut out.points, false);
        }
    }

    /// The second `clip_clipper_polygon_with_subject_bbox_templ` and the overloads of
    /// `clip_clipper_polygon_with_subject_bbox` that return the clipped points.
    pub trait ClipClipperPolygon {
        type Output;
        fn clip_clipper_polygon_with_subject_bbox_templ(&self, bbox: &BoundingBox) -> Self::Output;
        fn clip_clipper_polygon_with_subject_bbox(&self, bbox: &BoundingBox) -> Self::Output;
    }
    impl ClipClipperPolygon for [Point] {
        type Output = Points;
        // ClipperUtils.cpp:123-128
        // `template<typename PointType> std::vector<PointType> clip_clipper_polygon_with_subject_bbox_templ(const std::vector<PointType> &src, const BoundingBox &bbox)`
        fn clip_clipper_polygon_with_subject_bbox_templ(&self, bbox: &BoundingBox) -> Points {
            let mut out: Points = Vec::new();
            ClipClipperPolygonInto::clip_clipper_polygon_with_subject_bbox(self, bbox, &mut out, false);
            out
        }
        // ClipperUtils.cpp:130 `Points clip_clipper_polygon_with_subject_bbox(const Points &src, const BoundingBox &bbox)`
        fn clip_clipper_polygon_with_subject_bbox(&self, bbox: &BoundingBox) -> Points {
            ClipClipperPolygon::clip_clipper_polygon_with_subject_bbox_templ(self, bbox)
        }
    }
    impl ClipClipperPolygon for [ZPoint] {
        type Output = ZPoints;
        // ClipperUtils.cpp:123-128, of ZPoints
        fn clip_clipper_polygon_with_subject_bbox_templ(&self, bbox: &BoundingBox) -> ZPoints {
            let mut out: ZPoints = Vec::new();
            ClipClipperPolygonInto::clip_clipper_polygon_with_subject_bbox(self, bbox, &mut out, false);
            out
        }
        // ClipperUtils.cpp:131 `ZPoints clip_clipper_polygon_with_subject_bbox(const ZPoints &src, const BoundingBox &bbox)`
        fn clip_clipper_polygon_with_subject_bbox(&self, bbox: &BoundingBox) -> ZPoints {
            ClipClipperPolygon::clip_clipper_polygon_with_subject_bbox_templ(self, bbox)
        }
    }

    // ClipperUtils.cpp:137-142
    // `Polygon clip_clipper_polygon_with_subject_bbox(const Polygon &src, const BoundingBox &bbox, const bool get_entire_polygons)`
    pub fn clip_clipper_polygon_with_subject_bbox(src: &Polygon, bbox: &BoundingBox, get_entire_polygons: bool) -> Polygon {
        let mut out = Polygon::from_points(Vec::new());
        ClipClipperPolygonInto::clip_clipper_polygon_with_subject_bbox(src.points.as_slice(), bbox, &mut out.points, get_entire_polygons);
        out
    }

    /// `clip_clipper_polygons_with_subject_bbox` of ClipperUtils.hpp:330-332: one implementation for every overload.
    pub trait ClipClipperPolygons {
        fn clip_clipper_polygons_with_subject_bbox(&self, bbox: &BoundingBox, get_entire_polygons: bool) -> Polygons;
    }
    // ClipperUtils.cpp:144-151
    // `Polygons clip_clipper_polygons_with_subject_bbox(const Polygons &src, const BoundingBox &bbox)`: without `get_entire_polygons`,
    // which is the default of the overload of a Polygon, false, whatever is handed in here.
    impl ClipClipperPolygons for [Polygon] {
        fn clip_clipper_polygons_with_subject_bbox(&self, bbox: &BoundingBox, _get_entire_polygons: bool) -> Polygons {
            let mut out: Polygons = Vec::with_capacity(self.len());
            for p in self {
                out.push(clip_clipper_polygon_with_subject_bbox(p, bbox, false));
            }
            out.retain(|polygon| !polygon.points.is_empty());
            out
        }
    }
    // ClipperUtils.cpp:152-160
    // `Polygons clip_clipper_polygons_with_subject_bbox(const ExPolygon &src, const BoundingBox &bbox, const bool get_entire_polygons)`
    impl ClipClipperPolygons for ExPolygon {
        fn clip_clipper_polygons_with_subject_bbox(&self, bbox: &BoundingBox, get_entire_polygons: bool) -> Polygons {
            let mut out: Polygons = Vec::with_capacity(self.holes.len() + 1);
            out.push(clip_clipper_polygon_with_subject_bbox(&self.contour, bbox, get_entire_polygons));
            for p in &self.holes {
                out.push(clip_clipper_polygon_with_subject_bbox(p, bbox, get_entire_polygons));
            }
            out.retain(|polygon| !polygon.points.is_empty());
            out
        }
    }
    // ClipperUtils.cpp:161-172
    // `Polygons clip_clipper_polygons_with_subject_bbox(const ExPolygons &src, const BoundingBox &bbox, const bool get_entire_polygons)`
    impl ClipClipperPolygons for [ExPolygon] {
        fn clip_clipper_polygons_with_subject_bbox(&self, bbox: &BoundingBox, get_entire_polygons: bool) -> Polygons {
            let mut out: Polygons = Vec::new();
            for p in self {
                let temp = ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(p, bbox, get_entire_polygons);
                out.extend(temp);
            }

            out.retain(|polygon| !polygon.points.is_empty());
            out
        }
    }

    // ============================================================================
    // ClipperUtils.cpp:231-268 has_duplicate_points(const ClipperLib::PolyTree &)
    // ============================================================================

    // Point.hpp:323-327 `inline bool has_duplicate_points(const std::vector<Point> &pts)` over Point.cpp:225-232: the
    // points are copied, sorted and checked for duplicates globally.
    fn has_duplicate_points_path(pts: &Path) -> bool {
        let mut cpy: Vec<(i32, i32)> = pts.iter().map(|p| (p.x, p.y)).collect();
        cpy.sort();
        for i in 1..cpy.len() {
            if cpy[i - 1] == cpy[i] {
                return true;
            }
        }
        false
    }

    /// The `#if 0` arm of ClipperUtils.cpp:231-247, which the pin does not compile: the global test.
    pub mod global_test {
        use super::{Path, PolyTree};
        // ClipperUtils.cpp:233-247 `bool has_duplicate_points(const ClipperLib::PolyTree &polytree)`
        pub fn has_duplicate_points(polytree: &PolyTree) -> bool {
            // static void collect_points_recursive(const ClipperLib::PolyNode &polynode, ClipperLib::Path &out)
            fn collect_points_recursive(polytree: &PolyTree, polynode: usize, out: &mut Path) {
                // For each hole of the current expolygon:
                let node = polytree.node(polynode);
                out.extend(node.contour.iter().cloned());
                for i in 0..node.child_count() as usize {
                    collect_points_recursive(polytree, node.childs[i], out);
                }
            }
            let mut pts = Path::new();
            for i in 0..polytree.child_count() as usize {
                collect_points_recursive(polytree, polytree.root.childs[i], &mut pts);
            }
            // Point.cpp:225-232 `bool has_duplicate_points(std::vector<Point> &&pts)`: sorted, a point equal to its neighbour
            let mut pts: Vec<(i32, i32)> = pts.iter().map(|p| (p.x, p.y)).collect();
            pts.sort();
            for i in 1..pts.len() {
                if pts[i - 1] == pts[i] {
                    return true;
                }
            }
            false
        }
    }

    // Local test inside each of the contours.
    // ClipperUtils.cpp:250-267 `bool has_duplicate_points(const ClipperLib::PolyTree &polytree)`
    pub fn has_duplicate_points(polytree: &PolyTree) -> bool {
        // static bool has_duplicate_points_recursive(const ClipperLib::PolyNode &polynode)
        fn has_duplicate_points_recursive(polytree: &PolyTree, polynode: usize) -> bool {
            let node = polytree.node(polynode);
            if has_duplicate_points_path(&node.contour) {
                return true;
            }
            for i in 0..node.child_count() as usize {
                if has_duplicate_points_recursive(polytree, node.childs[i]) {
                    return true;
                }
            }
            false
        }
        for i in 0..polytree.child_count() as usize {
            if has_duplicate_points_recursive(polytree, polytree.root.childs[i]) {
                return true;
            }
        }
        false
    }

    // For debugging the Clipper library, for providing bug reports to the Clipper author.
    // ClipperUtils.cpp:26-57 (under CLIPPER_UTILS_DEBUG, which the pin does not define)
    // `bool export_clipper_input_polygons_bin(const char *path, const ClipperLib::Paths &input_subject, const ClipperLib::Paths &input_clip)`
    pub fn export_clipper_input_polygons_bin(path: &std::path::Path, input_subject: &Paths, input_clip: &Paths) -> bool {
        use std::io::Write;
        let Ok(mut pfile) = std::fs::File::create(path) else {
            return false;
        };
        let mut bytes: Vec<u8> = Vec::new();
        // `fwrite(path.data(), sizeof(ClipperLib::IntPoint), sz, pfile)`: the two ints of every point
        let mut write_paths = |paths: &Paths| {
            let sz = paths.len() as u32;
            bytes.extend_from_slice(&sz.to_ne_bytes());
            for path in paths {
                let sz = path.len() as u32;
                bytes.extend_from_slice(&sz.to_ne_bytes());
                for p in path {
                    bytes.extend_from_slice(&p.x.to_ne_bytes());
                    bytes.extend_from_slice(&p.y.to_ne_bytes());
                }
            }
        };
        write_paths(input_subject);
        write_paths(input_clip);
        pfile.write_all(&bytes).is_ok()
    }

    // ============================================================================
    // The templates of ClipperUtils.cpp over the providers (ClipperUtils.cpp:270-408, 508-557, 633-673, 737-739,
    // 840-907). The offsets are the `float` of the C++; `super::` has the bodies, over `ClipperLib::Paths`.
    // ============================================================================

    // Offset CCW contours outside, CW contours (holes) inside.
    // Don't calculate union of the output paths.
    // ClipperUtils.cpp:272-300
    pub fn raw_offset<P: Provider + ?Sized>(paths: &P, offset: f32, join_type: JoinType, miter_limit: f64, end_type: EndType) -> Paths {
        super::raw_offset(&paths.to_clipper_paths(), offset as f64, join_type, miter_limit, end_type)
    }

    // Offset outside by 10um, one by one.
    // ClipperUtils.cpp:303-307
    pub fn safety_offset<P: Provider + ?Sized>(paths: &P) -> Paths {
        raw_offset(paths, CLIPPER_SAFETY_OFFSET, DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT, EndType::etClosedPolygon)
    }

    // ClipperUtils.cpp:324-337, the overload with `do_safety_offset` (the one without it is `super::clipper_do`)
    pub fn clipper_do<TResult: super::ClipperResult, TSubj: Provider + ?Sized, TClip: Provider + ?Sized>(
        clip_type: ClipType,
        subject: &TSubj,
        clip: &TClip,
        fill_type: PolyFillType,
        do_safety_offset: ApplySafetyOffset,
    ) -> TResult {
        // Safety offset only allowed on intersection and difference.
        debug_assert!(do_safety_offset == ApplySafetyOffset::No || clip_type != ClipType::ctUnion);
        if do_safety_offset == ApplySafetyOffset::Yes {
            super::clipper_do::<TResult>(clip_type, &subject.to_clipper_paths(), &safety_offset(clip), fill_type)
        } else {
            super::clipper_do::<TResult>(clip_type, &subject.to_clipper_paths(), &clip.to_clipper_paths(), fill_type)
        }
    }

    // ClipperUtils.cpp:339-350
    pub fn clipper_union<TResult: super::ClipperResult, TSubj: Provider + ?Sized>(
        subject: &TSubj,
        // fillType pftNonZero and pftPositive "should" produce the same result for "normalized with implicit union" set of polygons
        fill_type: PolyFillType,
    ) -> TResult {
        super::clipper_union::<TResult>(&subject.to_clipper_paths(), fill_type)
    }

    // ClipperUtils.cpp:359-364
    pub fn raw_offset_polyline<P: Provider + ?Sized>(paths: &P, offset: f32, join_type: JoinType, miter_limit: f64, end_type: EndType) -> Paths {
        debug_assert!(offset > 0.);
        raw_offset(paths, offset, join_type, miter_limit, end_type)
    }

    // ClipperUtils.cpp:366-372
    pub fn expand_paths<TResult: super::ClipperResult, P: Provider + ?Sized>(paths: &P, offset: f32, join_type: JoinType, miter_limit: f64) -> TResult {
        // BBS
        //assert(offset > 0);
        super::expand_paths::<TResult>(&paths.to_clipper_paths(), offset as f64, join_type, miter_limit)
    }

    // ClipperUtils.cpp:381-397
    pub fn shrink_paths<TResult: super::ClipperResult, P: Provider + ?Sized>(paths: &P, offset: f32, join_type: JoinType, miter_limit: f64) -> TResult {
        // BBS
        //assert(offset > 0);
        super::shrink_paths::<TResult>(&paths.to_clipper_paths(), offset as f64, join_type, miter_limit)
    }

    // ClipperUtils.cpp:399-408
    pub fn offset_paths<TResult: super::ClipperResult, P: Provider + ?Sized>(paths: &P, offset: f32, join_type: JoinType, miter_limit: f64) -> TResult {
        // BBS
        //assert(offset != 0);

        if offset > 0. {
            expand_paths::<TResult, P>(paths, offset, join_type, miter_limit)
        } else {
            shrink_paths::<TResult, P>(paths, -offset, join_type, miter_limit)
        }
    }

    /// `offset_expolygon_inner` of ClipperUtils.cpp:436-511: one implementation for every overload.
    pub trait OffsetExpolygonInner {
        // returns number of expolygons collected (0 or 1).
        fn offset_expolygon_inner(&self, delta: f32, join_type: JoinType, miter_limit: f64, out: &mut Paths) -> i32;
    }
    // ClipperUtils.cpp:437-506
    // `static int offset_expolygon_inner(const Slic3r::ExPolygon &expoly, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::Paths &out)`
    impl OffsetExpolygonInner for ExPolygon {
        fn offset_expolygon_inner(&self, delta: f32, join_type: JoinType, miter_limit: f64, out: &mut Paths) -> i32 {
            let holes: Paths = self.holes.iter().map(|hole| path_of_points(&hole.points)).collect();
            super::offset_expolygon_inner(&path_of_points(&self.contour.points), &holes, delta as f64, join_type, miter_limit, out)
        }
    }
    // ClipperUtils.cpp:508-509
    // `static int offset_expolygon_inner(const Slic3r::Surface &surface, ...)`
    impl OffsetExpolygonInner for Surface {
        fn offset_expolygon_inner(&self, delta: f32, join_type: JoinType, miter_limit: f64, out: &mut Paths) -> i32 {
            OffsetExpolygonInner::offset_expolygon_inner(&self.expolygon, delta, join_type, miter_limit, out)
        }
    }
    // ClipperUtils.cpp:510-511
    // `static int offset_expolygon_inner(const Slic3r::Surface *surface, ...)`
    impl OffsetExpolygonInner for &Surface {
        fn offset_expolygon_inner(&self, delta: f32, join_type: JoinType, miter_limit: f64, out: &mut Paths) -> i32 {
            OffsetExpolygonInner::offset_expolygon_inner(&self.expolygon, delta, join_type, miter_limit, out)
        }
    }

    // ClipperUtils.cpp:513-518
    pub fn expolygon_offset(expolygon: &ExPolygon, delta: f32, join_type: JoinType, miter_limit: f64) -> Paths {
        let mut out = Paths::new();
        OffsetExpolygonInner::offset_expolygon_inner(expolygon, delta, join_type, miter_limit, &mut out);
        out
    }

    // This is a safe variant of the polygons offset, tailored for multiple ExPolygons.
    // It is required, that the input expolygons do not overlap and that the holes of each ExPolygon don't intersect with their respective outer contours.
    // Each ExPolygon is offsetted separately. For outer offset, the the offsetted ExPolygons shall be united outside of this function.
    // ClipperUtils.cpp:523-535
    pub fn expolygons_offset_raw<E: OffsetExpolygonInner>(expolygons: &[E], delta: f32, join_type: JoinType, miter_limit: f64) -> (Paths, usize) {
        // Offsetted ExPolygons before they are united.
        let mut output = Paths::new();
        output.reserve(expolygons.len());
        // How many non-empty offsetted expolygons were actually collected into output?
        // If only one, then there is no need to do a final union.
        let mut expolygons_collected: usize = 0;
        for expoly in expolygons {
            expolygons_collected += expoly.offset_expolygon_inner(delta, join_type, miter_limit, &mut output) as usize;
        }
        (output, expolygons_collected)
    }

    // See comment on expolygon_offsets_raw. In addition, for positive offset the contours are united.
    // ClipperUtils.cpp:538-548
    pub fn expolygons_offset<E: OffsetExpolygonInner>(expolygons: &[E], delta: f32, join_type: JoinType, miter_limit: f64) -> Paths {
        let (output, expolygons_collected) = expolygons_offset_raw(expolygons, delta, join_type, miter_limit);
        // Unite the offsetted expolygons.
        if expolygons_collected > 1 && delta > 0. {
            // There is a chance that the outwards offsetted expolygons may intersect. Perform a union.
            clipper_union::<Paths, _>(&output, PolyFillType::pftNonZero)
        } else {
            // Negative offset. The shrunk expolygons shall not mutually intersect. Just copy the output.
            output
        }
    }

    // See comment on expolygons_offset_raw. In addition, the polygons are always united to conver to polytree.
    // ClipperUtils.cpp:551-557
    pub fn expolygons_offset_pt<E: OffsetExpolygonInner>(expolygons: &[E], delta: f32, join_type: JoinType, miter_limit: f64) -> PolyTree {
        let (output, _expolygons_collected) = expolygons_offset_raw(expolygons, delta, join_type, miter_limit);
        // Unite the offsetted expolygons for both the
        clipper_union::<PolyTree, _>(&output, PolyFillType::pftNonZero)
    }

    // Fix of #117: A large fractal pyramid takes ages to slice
    // The Clipper library has difficulties processing overlapping polygons.
    // Namely, the function ClipperLib::JoinCommonEdges() has potentially a terrible time complexity if the output
    // of the operation is of the PolyTree type.
    // This function implemenets a following workaround:
    // 1) Peform the Clipper operation with the output to Paths. This method handles overlaps in a reasonable time.
    // 2) Run Clipper Union once again to extract the PolyTree from the result of 1).
    // ClipperUtils.cpp:655-667, the overload with `do_safety_offset` (the one without it is `super::clipper_do_polytree`)
    pub fn clipper_do_polytree<PathProvider1: Provider + ?Sized, PathProvider2: Provider + ?Sized>(
        clip_type: ClipType,
        subject: &PathProvider1,
        clip: &PathProvider2,
        fill_type: PolyFillType,
        do_safety_offset: ApplySafetyOffset,
    ) -> PolyTree {
        debug_assert!(do_safety_offset == ApplySafetyOffset::No || clip_type != ClipType::ctUnion);
        if do_safety_offset == ApplySafetyOffset::Yes {
            super::clipper_do_polytree(clip_type, &subject.to_clipper_paths(), &safety_offset(clip), fill_type)
        } else {
            super::clipper_do_polytree(clip_type, &subject.to_clipper_paths(), &clip.to_clipper_paths(), fill_type)
        }
    }

    // ClipperUtils.cpp:669-673
    pub fn _clipper<TSubj: Provider + ?Sized, TClip: Provider + ?Sized>(clip_type: ClipType, subject: &TSubj, clip: &TClip, do_safety_offset: ApplySafetyOffset) -> Polygons {
        to_polygons(clipper_do::<Paths, TSubj, TClip>(clip_type, subject, clip, PolyFillType::pftNonZero, do_safety_offset))
    }

    // ClipperUtils.cpp:737-739
    pub fn _clipper_ex<TSubject: Provider + ?Sized, TClip: Provider + ?Sized>(
        clip_type: ClipType,
        subject: &TSubject,
        clip: &TClip,
        do_safety_offset: ApplySafetyOffset,
        fill_type: PolyFillType,
    ) -> ExPolygons {
        poly_tree_to_ex_polygons(clipper_do_polytree(clip_type, subject, clip, fill_type, do_safety_offset))
    }

    // ClipperUtils.cpp:840-849
    pub fn _clipper_pl_open<PathsProvider1: Provider + ?Sized, PathsProvider2: Provider + ?Sized>(clip_type: ClipType, subject: &PathsProvider1, clip: &PathsProvider2) -> Polylines {
        let mut clipper = crate::clipper::Clipper::new(0);
        clipper.add_paths(&subject.to_clipper_paths(), crate::clipper::PolyType::ptSubject, false);
        clipper.add_paths(&clip.to_clipper_paths(), crate::clipper::PolyType::ptClip, true);
        let mut retval = PolyTree::new();
        clipper.execute_poly_tree(clip_type, &mut retval, PolyFillType::pftNonZero, PolyFillType::pftNonZero);
        poly_tree_to_polylines(retval)
    }

    // If the split_at_first_point() call above happens to split the polygon inside the clipping area
    // we would get two consecutive polylines instead of a single one, so we go through them in order
    // to recombine continuous polylines.
    // ClipperUtils.cpp:854-887
    pub fn _clipper_pl_recombine(polylines: &mut Polylines) {
        let mut i = 0;
        while i < polylines.len() {
            let mut j = i + 1;
            while j < polylines.len() {
                let (i_front, i_back) = (polylines[i].points[0], polylines[i].points[polylines[i].points.len() - 1]);
                let (j_front, j_back) = (polylines[j].points[0], polylines[j].points[polylines[j].points.len() - 1]);
                if i_back == j_front {
                    /* If last point of i coincides with first point of j,
                    append points of j to i and delete j */
                    let tail: Points = polylines[j].points[1..].to_vec();
                    polylines[i].points.extend(tail);
                    polylines.remove(j);
                    // `--j` and the `++j` of the loop
                    continue;
                } else if i_front == j_back {
                    /* If first point of i coincides with last point of j,
                    prepend points of j to i and delete j */
                    let n = polylines[j].points.len();
                    let head: Points = polylines[j].points[..n - 1].to_vec();
                    polylines[i].points.splice(0..0, head);
                    polylines.remove(j);
                    continue;
                } else if i_front == j_front {
                    /* Since Clipper does not preserve orientation of polylines,
                    also check the case when first point of i coincides with first point of j. */
                    polylines[j].points.reverse();
                    let n = polylines[j].points.len();
                    let head: Points = polylines[j].points[..n - 1].to_vec();
                    polylines[i].points.splice(0..0, head);
                    polylines.remove(j);
                    continue;
                } else if i_back == j_back {
                    /* Since Clipper does not preserve orientation of polylines,
                    also check the case when last point of i coincides with last point of j. */
                    polylines[j].points.reverse();
                    let tail: Points = polylines[j].points[1..].to_vec();
                    polylines[i].points.extend(tail);
                    polylines.remove(j);
                    continue;
                }
                j += 1;
            }
            i += 1;
        }
    }

    // ClipperUtils.cpp:889-907
    pub fn _clipper_pl_closed<PathProvider1: Provider + ?Sized, PathProvider2: Provider + ?Sized>(clip_type: ClipType, subject: &PathProvider1, clip: &PathProvider2) -> Polylines {
        // Transform input polygons into open paths.
        let subject = subject.to_clipper_paths();
        let mut paths = Paths::new();
        paths.reserve(subject.len());
        for poly in &subject {
            // Emplace polygon, duplicate the 1st point.
            let mut path: Path = poly.clone();
            // (`poly.front()` of a polygon without points is not defined in the C++; nothing is added here)
            if let Some(front) = poly.first() {
                path.push(*front);
            }
            paths.push(path);
        }
        // perform clipping
        let mut retval = _clipper_pl_open(clip_type, &paths, clip);
        _clipper_pl_recombine(&mut retval);
        retval
    }

    // ============================================================================
    // The functions of ClipperUtils.hpp, one implementation for every overload
    // ============================================================================

    /// `diff` of ClipperUtils.hpp: one implementation for every overload. Safety offset is applied to the clipping polygons only.
    pub trait Diff<Clip: ?Sized> {
        fn diff(&self, clip: &Clip, do_safety_offset: ApplySafetyOffset) -> Polygons;
    }
    // ClipperUtils.cpp:675-676 `diff(const Polygon &subject, const Polygon &clip, ApplySafetyOffset do_safety_offset)`
    impl Diff<Polygon> for Polygon {
        fn diff(&self, clip: &Polygon, do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctDifference, &SinglePathProvider::new(&self.points), &SinglePathProvider::new(&clip.points), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:677-678 `diff(const Polygons &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Diff<[Polygon]> for [Polygon] {
        fn diff(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctDifference, &PolygonsProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:687-688 `diff(const Polygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Diff<[ExPolygon]> for [Polygon] {
        fn diff(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctDifference, &PolygonsProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:689-690 `diff(const ExPolygons &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Diff<[Polygon]> for [ExPolygon] {
        fn diff(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctDifference, &ExPolygonsProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:691-692 `diff(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Diff<[ExPolygon]> for [ExPolygon] {
        fn diff(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctDifference, &ExPolygonsProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:693-694 `diff(const Surfaces &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Diff<[Polygon]> for [Surface] {
        fn diff(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctDifference, &SurfacesProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset)
        }
    }

    /// `intersection` of ClipperUtils.hpp: one implementation for every overload. Safety offset is applied to the clipping polygons only.
    pub trait Intersection<Clip: ?Sized> {
        fn intersection(&self, clip: &Clip, do_safety_offset: ApplySafetyOffset) -> Polygons;
    }
    // ClipperUtils.cpp:695-696 `intersection(const Polygon &subject, const Polygon &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<Polygon> for Polygon {
        fn intersection(&self, clip: &Polygon, do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctIntersection, &SinglePathProvider::new(&self.points), &SinglePathProvider::new(&clip.points), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:699-700 `intersection(const Polygons &subject, const ExPolygon &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<ExPolygon> for [Polygon] {
        fn intersection(&self, clip: &ExPolygon, do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctIntersection, &PolygonsProvider::new(self), &ExPolygonProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:701-702 `intersection(const Polygons &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<[Polygon]> for [Polygon] {
        fn intersection(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctIntersection, &PolygonsProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:703-704 `intersection(const ExPolygon &subject, const ExPolygon &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<ExPolygon> for ExPolygon {
        fn intersection(&self, clip: &ExPolygon, do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctIntersection, &ExPolygonProvider::new(self), &ExPolygonProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:705-706 `intersection(const ExPolygons &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<[Polygon]> for [ExPolygon] {
        fn intersection(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctIntersection, &ExPolygonsProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:707-708 `intersection(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<[ExPolygon]> for [ExPolygon] {
        fn intersection(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctIntersection, &ExPolygonsProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:709-710 `intersection(const Surfaces &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<[Polygon]> for [Surface] {
        fn intersection(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctIntersection, &SurfacesProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:711-712 `intersection(const Surfaces &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<[ExPolygon]> for [Surface] {
        fn intersection(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            _clipper(ClipType::ctIntersection, &SurfacesProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset)
        }
    }
    // ClipperUtils.cpp:714-719 `intersection(const Polygons &subject, const Polygon &clip, ApplySafetyOffset do_safety_offset)`
    impl Intersection<Polygon> for [Polygon] {
        fn intersection(&self, clip: &Polygon, do_safety_offset: ApplySafetyOffset) -> Polygons {
            let mut clip_temp: Polygons = Vec::new();
            clip_temp.push(clip.clone());
            Intersection::intersection(self, clip_temp.as_slice(), do_safety_offset)
        }
    }

    /// `diff_ex` of ClipperUtils.hpp: one implementation for every overload. Safety offset is applied to the clipping polygons only.
    pub trait DiffEx<Clip: ?Sized> {
        fn diff_ex(&self, clip: &Clip, do_safety_offset: ApplySafetyOffset) -> ExPolygons;
    }
    // ClipperUtils.cpp:741-742 `diff_ex(const Polygons &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[Polygon]> for [Polygon] {
        fn diff_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &PolygonsProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:743-744 `diff_ex(const Polygons &subject, const Surfaces &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[Surface]> for [Polygon] {
        fn diff_ex(&self, clip: &[Surface], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &PolygonsProvider::new(self), &SurfacesProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:745-746 `diff_ex(const Polygon &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[ExPolygon]> for Polygon {
        fn diff_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &SinglePathProvider::new(&self.points), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:747-748 `diff_ex(const Polygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[ExPolygon]> for [Polygon] {
        fn diff_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &PolygonsProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:749-750 `diff_ex(const ExPolygon &subject, const Polygon &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<Polygon> for ExPolygon {
        fn diff_ex(&self, clip: &Polygon, do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &ExPolygonProvider::new(self), &SinglePathProvider::new(&clip.points), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:751-752 `diff_ex(const ExPolygon &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[Polygon]> for ExPolygon {
        fn diff_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &ExPolygonProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:753-754 `diff_ex(const ExPolygons &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[Polygon]> for [ExPolygon] {
        fn diff_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &ExPolygonsProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:755-756 `diff_ex(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[ExPolygon]> for [ExPolygon] {
        fn diff_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &ExPolygonsProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:757-758 `diff_ex(const Surfaces &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[Polygon]> for [Surface] {
        fn diff_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &SurfacesProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:759-760 `diff_ex(const Surfaces &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[ExPolygon]> for [Surface] {
        fn diff_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &SurfacesProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:761-762 `diff_ex(const ExPolygons &subject, const Surfaces &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[Surface]> for [ExPolygon] {
        fn diff_ex(&self, clip: &[Surface], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &ExPolygonsProvider::new(self), &SurfacesProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:763-764 `diff_ex(const Surfaces &subject, const Surfaces &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[Surface]> for [Surface] {
        fn diff_ex(&self, clip: &[Surface], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &SurfacesProvider::new(self), &SurfacesProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:765-766 `diff_ex(const SurfacesPtr &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl<'s> DiffEx<[Polygon]> for [&'s Surface] {
        fn diff_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &SurfacesPtrProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:767-768 `diff_ex(const SurfacesPtr &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl<'s> DiffEx<[ExPolygon]> for [&'s Surface] {
        fn diff_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctDifference, &SurfacesPtrProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:770-776 `diff_ex(const Polygon &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[Polygon]> for Polygon {
        fn diff_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            let mut subject_temp: Polygons = Vec::new();
            subject_temp.push(self.clone());

            DiffEx::diff_ex(subject_temp.as_slice(), clip, do_safety_offset)
        }
    }
    // ClipperUtils.cpp:778-786 `diff_ex(const Polygon &subject, const Polygon &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<Polygon> for Polygon {
        fn diff_ex(&self, clip: &Polygon, do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            let mut subject_temp: Polygons = Vec::new();
            let mut clip_temp: Polygons = Vec::new();

            subject_temp.push(self.clone());
            clip_temp.push(clip.clone());
            DiffEx::diff_ex(subject_temp.as_slice(), clip_temp.as_slice(), do_safety_offset)
        }
    }
    // ClipperUtils.hpp:470-478 `diff_ex(const ExPolygon &subject, const ExPolygon &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<ExPolygon> for ExPolygon {
        fn diff_ex(&self, clip: &ExPolygon, do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            let _ = do_safety_offset;
            let mut subject_temp: ExPolygons = Vec::new();
            let mut clip_temp: ExPolygons = Vec::new();

            subject_temp.push(self.clone());
            clip_temp.push(clip.clone());
            // `return diff_ex(subject_temp, clip_temp);`: the default, ApplySafetyOffset::No
            DiffEx::diff_ex(subject_temp.as_slice(), clip_temp.as_slice(), ApplySafetyOffset::No)
        }
    }
    // ClipperUtils.hpp:480-486 `diff_ex(const ExPolygon &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<[ExPolygon]> for ExPolygon {
        fn diff_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            let mut subject_temp: ExPolygons = Vec::new();
            subject_temp.push(self.clone());

            DiffEx::diff_ex(subject_temp.as_slice(), clip, do_safety_offset)
        }
    }
    // ClipperUtils.hpp:488-494 `diff_ex(const ExPolygons &subject, const ExPolygon &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffEx<ExPolygon> for [ExPolygon] {
        fn diff_ex(&self, clip: &ExPolygon, do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            let mut clip_temp: ExPolygons = Vec::new();
            clip_temp.push(clip.clone());

            DiffEx::diff_ex(self, clip_temp.as_slice(), do_safety_offset)
        }
    }

    /// `intersection_ex` of ClipperUtils.hpp: one implementation for every overload.
    pub trait IntersectionEx<Clip: ?Sized> {
        fn intersection_ex(&self, clip: &Clip, do_safety_offset: ApplySafetyOffset) -> ExPolygons;
    }
    // ClipperUtils.cpp:788-789 `intersection_ex(const Polygons &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[Polygon]> for [Polygon] {
        fn intersection_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &PolygonsProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:790-791 `intersection_ex(const ExPolygon &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[Polygon]> for ExPolygon {
        fn intersection_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &ExPolygonProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:792-793 `intersection_ex(const ExPolygon &subject, const ExPolygon &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<ExPolygon> for ExPolygon {
        fn intersection_ex(&self, clip: &ExPolygon, do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &ExPolygonProvider::new(self), &ExPolygonProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:794-795 `intersection_ex(const Polygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[ExPolygon]> for [Polygon] {
        fn intersection_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &PolygonsProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:796-797 `intersection_ex(const ExPolygons &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[Polygon]> for [ExPolygon] {
        fn intersection_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &ExPolygonsProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:798-799 `intersection_ex(const ExPolygons &subject, const ExPolygon &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<ExPolygon> for [ExPolygon] {
        fn intersection_ex(&self, clip: &ExPolygon, do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &ExPolygonsProvider::new(self), &ExPolygonProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:800-801 `intersection_ex(const ExPolygon &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[ExPolygon]> for ExPolygon {
        fn intersection_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &ExPolygonProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:802-803 `intersection_ex(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[ExPolygon]> for [ExPolygon] {
        fn intersection_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &ExPolygonsProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:804-805 `intersection_ex(const Surfaces &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[Polygon]> for [Surface] {
        fn intersection_ex(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &SurfacesProvider::new(self), &PolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:806-807 `intersection_ex(const Surfaces &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[ExPolygon]> for [Surface] {
        fn intersection_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &SurfacesProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:808-809 `intersection_ex(const Surfaces &subject, const Surfaces &clip, ApplySafetyOffset do_safety_offset)`
    impl IntersectionEx<[Surface]> for [Surface] {
        fn intersection_ex(&self, clip: &[Surface], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &SurfacesProvider::new(self), &SurfacesProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:810-811 `intersection_ex(const SurfacesPtr &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl<'s> IntersectionEx<[ExPolygon]> for [&'s Surface] {
        fn intersection_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctIntersection, &SurfacesPtrProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }

    /// `xor_ex` of ClipperUtils.hpp: one implementation for every overload.
    pub trait XorEx<Clip: ?Sized> {
        fn xor_ex(&self, clip: &Clip, do_safety_offset: ApplySafetyOffset) -> ExPolygons;
    }
    // ClipperUtils.cpp:833-835 `xor_ex(const ExPolygons &subject, const ExPolygon &clip, ApplySafetyOffset do_safety_offset)`
    impl XorEx<ExPolygon> for [ExPolygon] {
        fn xor_ex(&self, clip: &ExPolygon, do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctXor, &ExPolygonsProvider::new(self), &ExPolygonProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }
    // ClipperUtils.cpp:836-838 `xor_ex(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset)`
    impl XorEx<[ExPolygon]> for [ExPolygon] {
        fn xor_ex(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            _clipper_ex(ClipType::ctXor, &ExPolygonsProvider::new(self), &ExPolygonsProvider::new(clip), do_safety_offset, PolyFillType::pftNonZero)
        }
    }

    /// `diff_pl` of ClipperUtils.hpp: one implementation for every overload.
    pub trait DiffPl<Clip: ?Sized> {
        fn diff_pl(&self, clip: &Clip) -> Polylines;
    }
    // ClipperUtils.cpp:909-910 `diff_pl(const Polyline &subject, const Polygons &clip)`
    impl DiffPl<[Polygon]> for Polyline {
        fn diff_pl(&self, clip: &[Polygon]) -> Polylines {
            _clipper_pl_open(ClipType::ctDifference, &SinglePathProvider::new(&self.points), &PolygonsProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:911-912 `diff_pl(const Polylines &subject, const Polygons &clip)`
    impl DiffPl<[Polygon]> for [Polyline] {
        fn diff_pl(&self, clip: &[Polygon]) -> Polylines {
            _clipper_pl_open(ClipType::ctDifference, &PolylinesProvider::new(self), &PolygonsProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:913-914 `diff_pl(const Polyline &subject, const ExPolygon &clip)`
    impl DiffPl<ExPolygon> for Polyline {
        fn diff_pl(&self, clip: &ExPolygon) -> Polylines {
            _clipper_pl_open(ClipType::ctDifference, &SinglePathProvider::new(&self.points), &ExPolygonProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:915-916 `diff_pl(const Polyline &subject, const ExPolygons &clip)`
    impl DiffPl<[ExPolygon]> for Polyline {
        fn diff_pl(&self, clip: &[ExPolygon]) -> Polylines {
            _clipper_pl_open(ClipType::ctDifference, &SinglePathProvider::new(&self.points), &ExPolygonsProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:917-918 `diff_pl(const Polylines &subject, const ExPolygon &clip)`
    impl DiffPl<ExPolygon> for [Polyline] {
        fn diff_pl(&self, clip: &ExPolygon) -> Polylines {
            _clipper_pl_open(ClipType::ctDifference, &PolylinesProvider::new(self), &ExPolygonProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:919-920 `diff_pl(const Polylines &subject, const ExPolygons &clip)`
    impl DiffPl<[ExPolygon]> for [Polyline] {
        fn diff_pl(&self, clip: &[ExPolygon]) -> Polylines {
            _clipper_pl_open(ClipType::ctDifference, &PolylinesProvider::new(self), &ExPolygonsProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:921-922 `diff_pl(const Polygons &subject, const Polygons &clip)`
    impl DiffPl<[Polygon]> for [Polygon] {
        fn diff_pl(&self, clip: &[Polygon]) -> Polylines {
            _clipper_pl_closed(ClipType::ctDifference, &PolygonsProvider::new(self), &PolygonsProvider::new(clip))
        }
    }

    /// `intersection_pl` of ClipperUtils.hpp: one implementation for every overload.
    pub trait IntersectionPl<Clip: ?Sized> {
        fn intersection_pl(&self, clip: &Clip) -> Polylines;
    }
    // ClipperUtils.cpp:923-924 `intersection_pl(const Polylines &subject, const Polygon &clip)`
    impl IntersectionPl<Polygon> for [Polyline] {
        fn intersection_pl(&self, clip: &Polygon) -> Polylines {
            _clipper_pl_open(ClipType::ctIntersection, &PolylinesProvider::new(self), &SinglePathProvider::new(&clip.points))
        }
    }
    // ClipperUtils.cpp:925-926 `intersection_pl(const Polyline &subject, const ExPolygon &clip)`
    impl IntersectionPl<ExPolygon> for Polyline {
        fn intersection_pl(&self, clip: &ExPolygon) -> Polylines {
            _clipper_pl_open(ClipType::ctIntersection, &SinglePathProvider::new(&self.points), &ExPolygonProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:927-928 `intersection_pl(const Polylines &subject, const ExPolygon &clip)`
    impl IntersectionPl<ExPolygon> for [Polyline] {
        fn intersection_pl(&self, clip: &ExPolygon) -> Polylines {
            _clipper_pl_open(ClipType::ctIntersection, &PolylinesProvider::new(self), &ExPolygonProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:929-930 `intersection_pl(const Polyline &subject, const Polygons &clip)`
    impl IntersectionPl<[Polygon]> for Polyline {
        fn intersection_pl(&self, clip: &[Polygon]) -> Polylines {
            _clipper_pl_open(ClipType::ctIntersection, &SinglePathProvider::new(&self.points), &PolygonsProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:931-932 `intersection_pl(const Polyline &subject, const ExPolygons &clip)`
    impl IntersectionPl<[ExPolygon]> for Polyline {
        fn intersection_pl(&self, clip: &[ExPolygon]) -> Polylines {
            _clipper_pl_open(ClipType::ctIntersection, &SinglePathProvider::new(&self.points), &ExPolygonsProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:933-934 `intersection_pl(const Polylines &subject, const Polygons &clip)`
    impl IntersectionPl<[Polygon]> for [Polyline] {
        fn intersection_pl(&self, clip: &[Polygon]) -> Polylines {
            _clipper_pl_open(ClipType::ctIntersection, &PolylinesProvider::new(self), &PolygonsProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:935-936 `intersection_pl(const Polylines &subject, const ExPolygons &clip)`
    impl IntersectionPl<[ExPolygon]> for [Polyline] {
        fn intersection_pl(&self, clip: &[ExPolygon]) -> Polylines {
            _clipper_pl_open(ClipType::ctIntersection, &PolylinesProvider::new(self), &ExPolygonsProvider::new(clip))
        }
    }
    // ClipperUtils.cpp:937-938 `intersection_pl(const Polygons &subject, const Polygons &clip)`
    impl IntersectionPl<[Polygon]> for [Polygon] {
        fn intersection_pl(&self, clip: &[Polygon]) -> Polylines {
            _clipper_pl_closed(ClipType::ctIntersection, &PolygonsProvider::new(self), &PolygonsProvider::new(clip))
        }
    }

    /// `offset` of ClipperUtils.hpp: one implementation for every overload. Wherever applicable, please use the expand() / shrink() variants instead, they convey their purpose better.
    pub trait Offset {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons;
    }
    // ClipperUtils.cpp:410-411 `offset(const Polygon &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Offset for Polygon {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            to_polygons(raw_offset(&SinglePathProvider::new(&self.points), delta, join_type, miter_limit, EndType::etClosedPolygon))
        }
    }
    // ClipperUtils.cpp:413-414 `offset(const Polygons &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Offset for [Polygon] {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            to_polygons(offset_paths::<Paths, _>(&PolygonsProvider::new(self), delta, join_type, miter_limit))
        }
    }
    // ClipperUtils.cpp:559-560 `offset(const ExPolygon &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Offset for ExPolygon {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            to_polygons(expolygon_offset(self, delta, join_type, miter_limit))
        }
    }
    // ClipperUtils.cpp:561-562 `offset(const ExPolygons &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Offset for [ExPolygon] {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            to_polygons(expolygons_offset(self, delta, join_type, miter_limit))
        }
    }
    // ClipperUtils.cpp:563-564 `offset(const Surfaces &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Offset for [Surface] {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            to_polygons(expolygons_offset(self, delta, join_type, miter_limit))
        }
    }
    // ClipperUtils.cpp:565-566 `offset(const SurfacesPtr &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl<'s> Offset for [&'s Surface] {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            to_polygons(expolygons_offset(self, delta, join_type, miter_limit))
        }
    }

    /// `offset` of ClipperUtils.hpp: one implementation for every overload. The offsets of polylines (the defaults: DEFAULT_LINE_JOIN_TYPE, DEFAULT_LINE_MITER_LIMIT, DEFAULT_END_TYPE).
    pub trait OffsetPolyline {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64, end_type: EndType) -> Polygons;
    }
    // ClipperUtils.cpp:418-419 `offset(const Polyline &x, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType end_type)`
    impl OffsetPolyline for Polyline {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64, end_type: EndType) -> Polygons {
            debug_assert!(delta > 0.);
            to_polygons(clipper_union::<Paths, _>(
                &raw_offset_polyline(&SinglePathProvider::new(&self.points), delta, join_type, miter_limit, end_type),
                PolyFillType::pftNonZero,
            ))
        }
    }
    // ClipperUtils.cpp:420-421 `offset(const Polylines &x, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType end_type)`
    impl OffsetPolyline for [Polyline] {
        fn offset(&self, delta: f32, join_type: JoinType, miter_limit: f64, end_type: EndType) -> Polygons {
            debug_assert!(delta > 0.);
            to_polygons(clipper_union::<Paths, _>(
                &raw_offset_polyline(&PolylinesProvider::new(self), delta, join_type, miter_limit, end_type),
                PolyFillType::pftNonZero,
            ))
        }
    }

    /// `offset_ex` of ClipperUtils.hpp: one implementation for every overload.
    pub trait OffsetEx {
        fn offset_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons;
    }
    // ClipperUtils.cpp:415-416 `offset_ex(const Polygons &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl OffsetEx for [Polygon] {
        fn offset_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            poly_tree_to_ex_polygons(offset_paths::<PolyTree, _>(&PolygonsProvider::new(self), delta, join_type, miter_limit))
        }
    }
    // ClipperUtils.cpp:567-569 `offset_ex(const ExPolygon &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl OffsetEx for ExPolygon {
        fn offset_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            //FIXME one may spare one Clipper Union call.
            super::clipper_paths_to_slic3r_ex_polygons(&expolygon_offset(self, delta, join_type, miter_limit), false)
        }
    }
    // ClipperUtils.cpp:570-571 `offset_ex(const ExPolygons &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl OffsetEx for [ExPolygon] {
        fn offset_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            poly_tree_to_ex_polygons(expolygons_offset_pt(self, delta, join_type, miter_limit))
        }
    }
    // ClipperUtils.cpp:572-573 `offset_ex(const Surfaces &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl OffsetEx for [Surface] {
        fn offset_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            poly_tree_to_ex_polygons(expolygons_offset_pt(self, delta, join_type, miter_limit))
        }
    }
    // ClipperUtils.cpp:574-575 `offset_ex(const SurfacesPtr &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl<'s> OffsetEx for [&'s Surface] {
        fn offset_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            poly_tree_to_ex_polygons(expolygons_offset_pt(self, delta, join_type, miter_limit))
        }
    }
    // ClipperUtils.hpp:358-364 `offset_ex(const Polygon &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl OffsetEx for Polygon {
        fn offset_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            let mut temp: Polygons = Vec::new();
            temp.push(self.clone());

            OffsetEx::offset_ex(temp.as_slice(), delta, join_type, miter_limit)
        }
    }

    /// `contour_to_polygons` of ClipperUtils.hpp: one implementation for every overload. convert stroke to path by offsetting of contour
    pub trait ContourToPolygons {
        fn contour_to_polygons(&self, line_width: f32, join_type: JoinType, miter_limit: f64) -> Polygons;
    }
    // ClipperUtils.cpp:423-428 `contour_to_polygons(const Polygon &x, const float line_width, ClipperLib::JoinType join_type, double miter_limit)`
    impl ContourToPolygons for Polygon {
        fn contour_to_polygons(&self, line_width: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            debug_assert!(line_width > 1.);
            to_polygons(clipper_union::<Paths, _>(
                &raw_offset(&SinglePathProvider::new(&self.points), line_width / 2., join_type, miter_limit, EndType::etClosedLine),
                PolyFillType::pftNonZero,
            ))
        }
    }
    // ClipperUtils.cpp:429-434 `contour_to_polygons(const Polygons &x, const float line_width, ClipperLib::JoinType join_type, double miter_limit)`
    impl ContourToPolygons for [Polygon] {
        fn contour_to_polygons(&self, line_width: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            debug_assert!(line_width > 1.);
            to_polygons(clipper_union::<Paths, _>(
                &raw_offset(&PolygonsProvider::new(self), line_width / 2., join_type, miter_limit, EndType::etClosedLine),
                PolyFillType::pftNonZero,
            ))
        }
    }

    /// `union_safety_offset` of ClipperUtils.hpp:370-371.
    pub trait UnionSafetyOffset {
        fn union_safety_offset(&self) -> Polygons;
    }
    // ClipperUtils.hpp:370 `union_safety_offset(const Polygons &polygons)`
    impl UnionSafetyOffset for [Polygon] {
        fn union_safety_offset(&self) -> Polygons {
            Offset::offset(self, CLIPPER_SAFETY_OFFSET, DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT)
        }
    }
    // ClipperUtils.hpp:371 `union_safety_offset(const ExPolygons &expolygons)`
    impl UnionSafetyOffset for [ExPolygon] {
        fn union_safety_offset(&self) -> Polygons {
            Offset::offset(self, CLIPPER_SAFETY_OFFSET, DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT)
        }
    }

    /// `union_safety_offset_ex` of ClipperUtils.hpp:372-373.
    pub trait UnionSafetyOffsetEx {
        fn union_safety_offset_ex(&self) -> ExPolygons;
    }
    // ClipperUtils.hpp:372 `union_safety_offset_ex(const Polygons &polygons)`
    impl UnionSafetyOffsetEx for [Polygon] {
        fn union_safety_offset_ex(&self) -> ExPolygons {
            OffsetEx::offset_ex(self, CLIPPER_SAFETY_OFFSET, DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT)
        }
    }
    // ClipperUtils.hpp:373 `union_safety_offset_ex(const ExPolygons &expolygons)`
    impl UnionSafetyOffsetEx for [ExPolygon] {
        fn union_safety_offset_ex(&self) -> ExPolygons {
            OffsetEx::offset_ex(self, CLIPPER_SAFETY_OFFSET, DEFAULT_JOIN_TYPE, DEFAULT_MITER_LIMIT)
        }
    }

    /// `expand` of ClipperUtils.hpp: one implementation for every overload. Aliases for the various offset(...) functions, conveying the purpose of the offset.
    pub trait Expand {
        fn expand(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons;
    }
    // ClipperUtils.hpp:381-382 `expand(const Polygon &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Expand for Polygon {
        fn expand(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            debug_assert!(delta > 0.);
            Offset::offset(self, delta, join_type, miter_limit)
        }
    }
    // ClipperUtils.hpp:383-384 `expand(const Polygons &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Expand for [Polygon] {
        fn expand(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            debug_assert!(delta > 0.);
            Offset::offset(self, delta, join_type, miter_limit)
        }
    }

    /// `shrink_ex` of ClipperUtils.hpp: one implementation for every overload. Input polygons for shrinking shall be "normalized": There must be no overlap / intersections between the input polygons.
    pub trait ShrinkEx {
        fn shrink_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons;
    }
    // ClipperUtils.hpp:390-391 `shrink_ex(const Polygons &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl ShrinkEx for [Polygon] {
        fn shrink_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            debug_assert!(delta > 0.);
            OffsetEx::offset_ex(self, -delta, join_type, miter_limit)
        }
    }
    // ClipperUtils.hpp:392-393 `shrink_ex(const ExPolygons &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl ShrinkEx for [ExPolygon] {
        fn shrink_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            debug_assert!(delta > 0.);
            OffsetEx::offset_ex(self, -delta, join_type, miter_limit)
        }
    }

    /// `offset2_ex` of ClipperUtils.hpp: one implementation for every overload.
    pub trait Offset2Ex {
        fn offset2_ex(&self, delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons;
    }
    // ClipperUtils.cpp:581-584 `offset2_ex(const ExPolygons &x, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    impl Offset2Ex for [ExPolygon] {
        fn offset2_ex(&self, delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            poly_tree_to_ex_polygons(offset_paths::<PolyTree, _>(&expolygons_offset(self, delta1, join_type, miter_limit), delta2, join_type, miter_limit))
        }
    }
    // ClipperUtils.cpp:585-589 `offset2_ex(const Surfaces &x, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    impl Offset2Ex for [Surface] {
        fn offset2_ex(&self, delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            //FIXME it may be more efficient to offset to_expolygons(surfaces) instead of to_polygons(surfaces).
            poly_tree_to_ex_polygons(offset_paths::<PolyTree, _>(&expolygons_offset(self, delta1, join_type, miter_limit), delta2, join_type, miter_limit))
        }
    }

    /// `closing` of ClipperUtils.hpp: one implementation for every overload; `Delta` is `(delta1, delta2)` or `delta`. Offset outside, then inside produces morphological closing. All deltas should be positive.
    pub trait Closing<Delta> {
        fn closing(&self, delta: Delta, join_type: JoinType, miter_limit: f64) -> Polygons;
    }
    // ClipperUtils.cpp:592-597 `closing(const Polygons &polygons, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    impl Closing<(f32, f32)> for [Polygon] {
        fn closing(&self, (delta1, delta2): (f32, f32), join_type: JoinType, miter_limit: f64) -> Polygons {
            debug_assert!(delta1 > 0.);
            debug_assert!(delta2 > 0.);
            to_polygons(shrink_paths::<Paths, _>(
                &expand_paths::<Paths, _>(&PolygonsProvider::new(self), delta1, join_type, miter_limit),
                delta2,
                join_type,
                miter_limit,
            ))
        }
    }
    // ClipperUtils.hpp:407-408 `closing(const Polygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Closing<f32> for [Polygon] {
        fn closing(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            Closing::closing(self, (delta, delta), join_type, miter_limit)
        }
    }

    /// `closing_ex` of ClipperUtils.hpp: one implementation for every overload; `Delta` is `(delta1, delta2)` or `delta`.
    pub trait ClosingEx<Delta> {
        fn closing_ex(&self, delta: Delta, join_type: JoinType, miter_limit: f64) -> ExPolygons;
    }
    // ClipperUtils.cpp:598-603 `closing_ex(const Polygons &polygons, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    impl ClosingEx<(f32, f32)> for [Polygon] {
        fn closing_ex(&self, (delta1, delta2): (f32, f32), join_type: JoinType, miter_limit: f64) -> ExPolygons {
            debug_assert!(delta1 > 0.);
            debug_assert!(delta2 > 0.);
            poly_tree_to_ex_polygons(shrink_paths::<PolyTree, _>(
                &expand_paths::<Paths, _>(&PolygonsProvider::new(self), delta1, join_type, miter_limit),
                delta2,
                join_type,
                miter_limit,
            ))
        }
    }
    // ClipperUtils.cpp:604-610 `closing_ex(const Surfaces &polygons, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    impl ClosingEx<(f32, f32)> for [Surface] {
        fn closing_ex(&self, (delta1, delta2): (f32, f32), join_type: JoinType, miter_limit: f64) -> ExPolygons {
            debug_assert!(delta1 > 0.);
            debug_assert!(delta2 > 0.);
            //FIXME it may be more efficient to offset to_expolygons(surfaces) instead of to_polygons(surfaces).
            poly_tree_to_ex_polygons(shrink_paths::<PolyTree, _>(
                &expand_paths::<Paths, _>(&SurfacesProvider::new(self), delta1, join_type, miter_limit),
                delta2,
                join_type,
                miter_limit,
            ))
        }
    }
    // ClipperUtils.hpp:410-411 `closing_ex(const Polygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl ClosingEx<f32> for [Polygon] {
        fn closing_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            ClosingEx::closing_ex(self, (delta, delta), join_type, miter_limit)
        }
    }
    // ClipperUtils.hpp:412-413 `closing_ex(const ExPolygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl ClosingEx<f32> for [ExPolygon] {
        fn closing_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            debug_assert!(delta > 0.);
            Offset2Ex::offset2_ex(self, delta, -delta, join_type, miter_limit)
        }
    }
    // ClipperUtils.hpp:414-415 `closing_ex(const Surfaces &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl ClosingEx<f32> for [Surface] {
        fn closing_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            debug_assert!(delta > 0.);
            Offset2Ex::offset2_ex(self, delta, -delta, join_type, miter_limit)
        }
    }

    /// `opening` of ClipperUtils.hpp: one implementation for every overload; `Delta` is `(delta1, delta2)` or `delta`. Offset inside, then outside produces morphological opening. All deltas should be positive. Input polygons for opening shall be "normalized": There must be no overlap / intersections between the input polygons.
    pub trait Opening<Delta> {
        fn opening(&self, delta: Delta, join_type: JoinType, miter_limit: f64) -> Polygons;
    }
    // ClipperUtils.cpp:613-618 `opening(const Polygons &polygons, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    impl Opening<(f32, f32)> for [Polygon] {
        fn opening(&self, (delta1, delta2): (f32, f32), join_type: JoinType, miter_limit: f64) -> Polygons {
            debug_assert!(delta1 > 0.);
            debug_assert!(delta2 > 0.);
            to_polygons(expand_paths::<Paths, _>(
                &shrink_paths::<Paths, _>(&PolygonsProvider::new(self), delta1, join_type, miter_limit),
                delta2,
                join_type,
                miter_limit,
            ))
        }
    }
    // ClipperUtils.cpp:619-624 `opening(const ExPolygons &polygons, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    impl Opening<(f32, f32)> for [ExPolygon] {
        fn opening(&self, (delta1, delta2): (f32, f32), join_type: JoinType, miter_limit: f64) -> Polygons {
            debug_assert!(delta1 > 0.);
            debug_assert!(delta2 > 0.);
            to_polygons(expand_paths::<Paths, _>(
                &shrink_paths::<Paths, _>(&ExPolygonsProvider::new(self), delta1, join_type, miter_limit),
                delta2,
                join_type,
                miter_limit,
            ))
        }
    }
    // ClipperUtils.cpp:625-631 `opening(const Surfaces &polygons, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    impl Opening<(f32, f32)> for [Surface] {
        fn opening(&self, (delta1, delta2): (f32, f32), join_type: JoinType, miter_limit: f64) -> Polygons {
            debug_assert!(delta1 > 0.);
            debug_assert!(delta2 > 0.);
            //FIXME it may be more efficient to offset to_expolygons(surfaces) instead of to_polygons(surfaces).
            to_polygons(expand_paths::<Paths, _>(
                &shrink_paths::<Paths, _>(&SurfacesProvider::new(self), delta1, join_type, miter_limit),
                delta2,
                join_type,
                miter_limit,
            ))
        }
    }
    // ClipperUtils.hpp:422-423 `opening(const Polygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Opening<f32> for [Polygon] {
        fn opening(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            Opening::opening(self, (delta, delta), join_type, miter_limit)
        }
    }
    // ClipperUtils.hpp:424-425 `opening(const ExPolygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Opening<f32> for [ExPolygon] {
        fn opening(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            Opening::opening(self, (delta, delta), join_type, miter_limit)
        }
    }
    // ClipperUtils.hpp:426-427 `opening(const Surfaces &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl Opening<f32> for [Surface] {
        fn opening(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
            Opening::opening(self, (delta, delta), join_type, miter_limit)
        }
    }

    /// `opening_ex` of ClipperUtils.hpp: one implementation for every overload.
    pub trait OpeningEx {
        fn opening_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons;
    }
    // ClipperUtils.hpp:428-429 `opening_ex(const ExPolygons &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl OpeningEx for [ExPolygon] {
        fn opening_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            debug_assert!(delta > 0.);
            Offset2Ex::offset2_ex(self, -delta, delta, join_type, miter_limit)
        }
    }
    // ClipperUtils.hpp:430-431 `opening_ex(const Surfaces &x, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    impl OpeningEx for [Surface] {
        fn opening_ex(&self, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
            debug_assert!(delta > 0.);
            Offset2Ex::offset2_ex(self, -delta, delta, join_type, miter_limit)
        }
    }

    // ClipperUtils.hpp:385-386
    // `inline Slic3r::ExPolygons expand_ex(const Slic3r::Polygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    pub fn expand_ex(polygons: &[Polygon], delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        debug_assert!(delta > 0.);
        OffsetEx::offset_ex(polygons, delta, join_type, miter_limit)
    }

    // Input polygons for shrinking shall be "normalized": There must be no overlap / intersections between the input polygons.
    // ClipperUtils.hpp:388-389
    // `inline Slic3r::Polygons shrink(const Slic3r::Polygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)`
    pub fn shrink(polygons: &[Polygon], delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        debug_assert!(delta > 0.);
        Offset::offset(polygons, -delta, join_type, miter_limit)
    }

    // Wherever applicable, please use the opening() / closing() variants instead, they convey their purpose better.
    // Input polygons for negative offset shall be "normalized": There must be no overlap / intersections between the input polygons.
    // ClipperUtils.cpp:577-580
    // `Polygons offset2(const ExPolygons &expolygons, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit)`
    pub fn offset2(expolygons: &[ExPolygon], delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        to_polygons(offset_paths::<Paths, _>(&expolygons_offset(expolygons, delta1, join_type, miter_limit), delta2, join_type, miter_limit))
    }

    // ----------------------------------------------------------------------------
    // The bounding boxes of the `_clipped` functions, as BoundingBox.hpp and Polygon.cpp / ExPolygon.cpp make them.
    // ----------------------------------------------------------------------------

    // BoundingBox.hpp:14-102 `BoundingBoxBase<Point>`: (0, 0)-(0, 0) and not defined when default constructed; of a
    // range of points, defined only with a positive area.
    #[derive(Debug, Clone, Copy)]
    struct BoundingBoxBase {
        min: Point,
        max: Point,
        defined: bool,
    }
    impl BoundingBoxBase {
        fn new() -> Self {
            BoundingBoxBase { min: Point::new(0, 0), max: Point::new(0, 0), defined: false }
        }
        // BoundingBox.hpp:88-102 `construct<false>(out, from, to)`
        fn of_points(points: &[Point]) -> Self {
            let mut out = BoundingBoxBase::new();
            if let Some(first) = points.first() {
                out.min = *first;
                out.max = out.min;
                for vec in &points[1..] {
                    out.min = Point::new(out.min.x.min(vec.x), out.min.y.min(vec.y));
                    out.max = Point::new(out.max.x.max(vec.x), out.max.y.max(vec.y));
                }
                out.defined = out.min.x < out.max.x && out.min.y < out.max.y;
            }
            out
        }
        // BoundingBox.cpp `template <class PointClass> void BoundingBoxBase<PointClass>::merge(const BoundingBoxBase<PointClass> &bb)`
        fn merge(&mut self, bb: &BoundingBoxBase) {
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
        // BoundingBox.hpp:48 `inflated(coordf_t delta)` over BoundingBox.cpp `offset(coordf_t delta)`: `PointClass v(delta,
        // delta)`, a Point of two doubles (lrint), whether the box is defined or not.
        fn inflated(&self, delta: f64) -> BoundingBox {
            let v = delta.round_ties_even() as i64;
            let mut out = BoundingBox::from_points_minmax(Point::new(self.min.x - v, self.min.y - v), Point::new(self.max.x + v, self.max.y + v));
            // the corners are all the clipping reads
            out.min = Point::new(self.min.x - v, self.min.y - v);
            out.max = Point::new(self.max.x + v, self.max.y + v);
            out
        }
    }

    // Polygon.cpp:411-425 `BoundingBox get_extents(const Polygons &polygons)`
    fn get_extents_polygons(polygons: &[Polygon]) -> BoundingBoxBase {
        let mut bb = BoundingBoxBase::new();
        if !polygons.is_empty() {
            bb = BoundingBoxBase::of_points(&polygons[0].points);
            for i in 1..polygons.len() {
                bb.merge(&BoundingBoxBase::of_points(&polygons[i].points));
            }
        }
        bb
    }

    // ExPolygon.cpp:490-504 `BoundingBox get_extents(const ExPolygons &expolygons)`
    fn get_extents_expolygons(expolygons: &[ExPolygon]) -> BoundingBoxBase {
        let mut bbox = BoundingBoxBase::new();
        if !expolygons.is_empty() {
            for i in 0..expolygons.len() {
                if !expolygons[i].contour.points.is_empty() {
                    bbox.merge(&BoundingBoxBase::of_points(&expolygons[i].contour.points));
                }
            }
        }
        bbox
    }

    /// `diff_clipped` of ClipperUtils.hpp:441-443: one implementation for every overload.
    // Optimized version clipping the "clipping" polygon using clip_clipper_polygon_with_subject_bbox().
    // To be used with complex clipping polygons, where majority of the clipping polygons are outside of the source polygon.
    pub trait DiffClipped<Clip: ?Sized> {
        type Output;
        fn diff_clipped(&self, clip: &Clip, do_safety_offset: ApplySafetyOffset) -> Self::Output;
    }
    // ClipperUtils.cpp:679-680
    // `Slic3r::Polygons diff_clipped(const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffClipped<[Polygon]> for [Polygon] {
        type Output = Polygons;
        fn diff_clipped(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
            Diff::diff(
                self,
                ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(clip, &get_extents_polygons(self).inflated(SCALED_EPSILON), false).as_slice(),
                do_safety_offset,
            )
        }
    }
    // ClipperUtils.cpp:681-682
    // `Slic3r::ExPolygons diff_clipped(const Slic3r::ExPolygons &subject, const Slic3r::Polygons &clip, ApplySafetyOffset do_safety_offset)`
    impl DiffClipped<[Polygon]> for [ExPolygon] {
        type Output = ExPolygons;
        fn diff_clipped(&self, clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            DiffEx::diff_ex(
                self,
                ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(clip, &get_extents_expolygons(self).inflated(SCALED_EPSILON), false).as_slice(),
                do_safety_offset,
            )
        }
    }
    // ClipperUtils.cpp:683-686
    // `Slic3r::ExPolygons diff_clipped(const Slic3r::ExPolygons & subject, const Slic3r::ExPolygons & clip, ApplySafetyOffset do_safety_offset)`
    impl DiffClipped<[ExPolygon]> for [ExPolygon] {
        type Output = ExPolygons;
        fn diff_clipped(&self, clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
            DiffEx::diff_ex(
                self,
                ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(clip, &get_extents_expolygons(self).inflated(SCALED_EPSILON), false).as_slice(),
                do_safety_offset,
            )
        }
    }

    // Optimized version clipping the "clipping" polygon using clip_clipper_polygon_with_subject_bbox().
    // To be used with complex clipping polygons, where majority of the clipping polygons are outside of the source polygon.
    // ClipperUtils.cpp:697-698
    // `Slic3r::Polygons intersection_clipped(const Slic3r::Polygons &subject, const Slic3r::Polygons &clip, ApplySafetyOffset do_safety_offset)`
    pub fn intersection_clipped(subject: &[Polygon], clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> Polygons {
        Intersection::intersection(
            subject,
            ClipClipperPolygons::clip_clipper_polygons_with_subject_bbox(clip, &get_extents_polygons(subject).inflated(SCALED_EPSILON), false).as_slice(),
            do_safety_offset,
        )
    }

    /// `union_` of ClipperUtils.hpp:548-552: one implementation for every overload; `Arg` is `()` for the overloads
    /// of one argument, else the second argument.
    pub trait Union_<Arg> {
        fn union_(&self, arg: Arg) -> Polygons;
    }
    // ClipperUtils.cpp:721-722 `Slic3r::Polygons union_(const Slic3r::Polygons &subject)`
    impl Union_<()> for [Polygon] {
        fn union_(&self, _: ()) -> Polygons {
            _clipper(ClipType::ctUnion, &PolygonsProvider::new(self), &EmptyPathsProvider::new(), ApplySafetyOffset::No)
        }
    }
    // ClipperUtils.cpp:723-724 `Slic3r::Polygons union_(const Slic3r::ExPolygons &subject)`
    impl Union_<()> for [ExPolygon] {
        fn union_(&self, _: ()) -> Polygons {
            _clipper(ClipType::ctUnion, &ExPolygonsProvider::new(self), &EmptyPathsProvider::new(), ApplySafetyOffset::No)
        }
    }
    // ClipperUtils.cpp:725-726 `Slic3r::Polygons union_(const Slic3r::Polygons &subject, const ClipperLib::PolyFillType fillType)`
    impl Union_<PolyFillType> for [Polygon] {
        fn union_(&self, fill_type: PolyFillType) -> Polygons {
            to_polygons(clipper_do::<Paths, _, _>(ClipType::ctUnion, &PolygonsProvider::new(self), &EmptyPathsProvider::new(), fill_type, ApplySafetyOffset::No))
        }
    }
    // ClipperUtils.cpp:727-734 `Slic3r::Polygons union_(const Slic3r::Polygons &subject, const Slic3r::Polygons &subject2)`
    impl Union_<&[Polygon]> for [Polygon] {
        fn union_(&self, subject2: &[Polygon]) -> Polygons {
            // BBS
            let mut polys: Polygons = self.to_vec();
            for poly in subject2 {
                polys.push(poly.clone());
            }
            Union_::union_(polys.as_slice(), ())
        }
    }
    // ClipperUtils.cpp:735-736 `Slic3r::Polygons union_(const Slic3r::Polygons &subject, const Slic3r::ExPolygon &subject2)`
    impl Union_<&ExPolygon> for [Polygon] {
        fn union_(&self, subject2: &ExPolygon) -> Polygons {
            _clipper(ClipType::ctUnion, &PolygonsProvider::new(self), &ExPolygonProvider::new(subject2), ApplySafetyOffset::No)
        }
    }

    /// `union_ex` of ClipperUtils.hpp:554-559: one implementation for every overload; `Arg` is `()` for the
    /// overloads of one argument, else the arguments after the first.
    // May be used to "heal" unusual models (3DLabPrints etc.) by providing fill_type (pftEvenOdd, pftNonZero, pftPositive, pftNegative).
    pub trait UnionEx<Arg> {
        fn union_ex(&self, arg: Arg) -> ExPolygons;
    }
    // ClipperUtils.cpp:813-814 `Slic3r::ExPolygons union_ex(const Slic3r::Polygons &subject, ClipperLib::PolyFillType fill_type)`
    impl UnionEx<PolyFillType> for [Polygon] {
        fn union_ex(&self, fill_type: PolyFillType) -> ExPolygons {
            _clipper_ex(ClipType::ctUnion, &PolygonsProvider::new(self), &EmptyPathsProvider::new(), ApplySafetyOffset::No, fill_type)
        }
    }
    // ClipperUtils.cpp:815-816 `Slic3r::ExPolygons union_ex(const Slic3r::ExPolygons &subject)`
    impl UnionEx<()> for [ExPolygon] {
        fn union_ex(&self, _: ()) -> ExPolygons {
            poly_tree_to_ex_polygons(super::clipper_do_polytree(
                ClipType::ctUnion,
                &ExPolygonsProvider::new(self).to_clipper_paths(),
                &EmptyPathsProvider::new().to_clipper_paths(),
                PolyFillType::pftNonZero,
            ))
        }
    }
    // ClipperUtils.cpp:817-821 `Slic3r::ExPolygons union_ex(const Slic3r::ExPolygons &subject, const Slic3r::Polygons &subject2)`
    impl UnionEx<&[Polygon]> for [ExPolygon] {
        fn union_ex(&self, subject2: &[Polygon]) -> ExPolygons {
            poly_tree_to_ex_polygons(super::clipper_do_polytree(
                ClipType::ctUnion,
                &ExPolygonsProvider::new(self).to_clipper_paths(),
                &PolygonsProvider::new(subject2).to_clipper_paths(),
                PolyFillType::pftNonZero,
            ))
        }
    }
    // ClipperUtils.cpp:822-823 `Slic3r::ExPolygons union_ex(const Slic3r::Surfaces &subject)`
    impl UnionEx<()> for [Surface] {
        fn union_ex(&self, _: ()) -> ExPolygons {
            poly_tree_to_ex_polygons(super::clipper_do_polytree(
                ClipType::ctUnion,
                &SurfacesProvider::new(self).to_clipper_paths(),
                &EmptyPathsProvider::new().to_clipper_paths(),
                PolyFillType::pftNonZero,
            ))
        }
    }
    // BBS
    // ClipperUtils.cpp:825-831 `Slic3r::ExPolygons union_ex(const Slic3r::ExPolygons& poly1, const Slic3r::ExPolygons& poly2, bool safety_offset_)`
    impl UnionEx<(&[ExPolygon], bool)> for [ExPolygon] {
        fn union_ex(&self, (poly2, _safety_offset_): (&[ExPolygon], bool)) -> ExPolygons {
            let mut expolys: ExPolygons = self.to_vec();
            for expoly in poly2 {
                expolys.push(expoly.clone());
            }
            UnionEx::union_ex(expolys.as_slice(), ())
        }
    }

    // ClipperUtils.cpp:940-958
    // `Lines _clipper_ln(ClipperLib::ClipType clipType, const Lines &subject, const Polygons &clip)`
    pub fn _clipper_ln(clip_type: ClipType, subject: &[Line], clip: &[Polygon]) -> Lines {
        // convert Lines to Polylines
        let mut polylines: Polylines = Vec::with_capacity(subject.len());
        for line in subject {
            polylines.push(Polyline::from_points(vec![line.a, line.b]));
        }

        // perform operation
        let polylines = _clipper_pl_open(clip_type, &PolylinesProvider::new(&polylines), &PolygonsProvider::new(clip));

        // convert Polylines to Lines
        let mut retval: Lines = Vec::new();
        for polyline in &polylines {
            if polyline.points.len() >= 2 {
                //FIXME It may happen, that Clipper produced a polyline with more than 2 collinear points by clipping a single line with polygons. It is a very rare issue, but it happens, see GH #6933.
                retval.push(Line { a: polyline.points[0], b: polyline.points[polyline.points.len() - 1] });
            }
        }
        retval
    }

    // ClipperUtils.hpp:496-499 `inline Slic3r::Lines diff_ln(const Slic3r::Lines &subject, const Slic3r::Polygons &clip)`
    pub fn diff_ln(subject: &[Line], clip: &[Polygon]) -> Lines {
        _clipper_ln(ClipType::ctDifference, subject, clip)
    }

    /// `intersection_ln` of ClipperUtils.hpp:536-546: one implementation for every overload.
    pub trait IntersectionLn {
        fn intersection_ln(&self, clip: &[Polygon]) -> Lines;
    }
    // ClipperUtils.hpp:536-539 `inline Slic3r::Lines intersection_ln(const Slic3r::Lines &subject, const Slic3r::Polygons &clip)`
    impl IntersectionLn for [Line] {
        fn intersection_ln(&self, clip: &[Polygon]) -> Lines {
            _clipper_ln(ClipType::ctIntersection, self, clip)
        }
    }
    // ClipperUtils.hpp:541-546 `inline Slic3r::Lines intersection_ln(const Slic3r::Line &subject, const Slic3r::Polygons &clip)`
    impl IntersectionLn for Line {
        fn intersection_ln(&self, clip: &[Polygon]) -> Lines {
            let mut lines: Lines = Vec::new();
            lines.push(self.clone());
            _clipper_ln(ClipType::ctIntersection, &lines, clip)
        }
    }

    /// `union_pt` of ClipperUtils.hpp:561-564: one implementation for every overload.
    // Convert polygons / expolygons into ClipperLib::PolyTree using ClipperLib::pftEvenOdd, thus union will NOT be performed.
    // If the contours are not intersecting, their orientation shall not be modified by union_pt().
    pub trait UnionPt {
        fn union_pt(&self) -> PolyTree;
    }
    // ClipperUtils.cpp:962-965 `ClipperLib::PolyTree union_pt(const Polygons &subject)`
    impl UnionPt for [Polygon] {
        fn union_pt(&self) -> PolyTree {
            super::clipper_do::<PolyTree>(ClipType::ctUnion, &PolygonsProvider::new(self).to_clipper_paths(), &EmptyPathsProvider::new().to_clipper_paths(), PolyFillType::pftEvenOdd)
        }
    }
    // ClipperUtils.cpp:966-969 `ClipperLib::PolyTree union_pt(const ExPolygons &subject)`
    impl UnionPt for [ExPolygon] {
        fn union_pt(&self) -> PolyTree {
            super::clipper_do::<PolyTree>(ClipType::ctUnion, &ExPolygonsProvider::new(self).to_clipper_paths(), &EmptyPathsProvider::new().to_clipper_paths(), PolyFillType::pftEvenOdd)
        }
    }

    /// `ClipperLib::PolyNodes` (clipper.hpp:134 `typedef std::vector<PolyNode*> PolyNodes;`): the nodes of a
    /// `PolyTree` by their index in it (`POLY_TREE_ROOT` is the tree itself).
    pub type PolyNodes = Vec<usize>;

    // Simple spatial ordering of Polynodes
    // ClipperUtils.cpp:972-987 `ClipperLib::PolyNodes order_nodes(const ClipperLib::PolyNodes &nodes)`
    pub fn order_nodes(tree: &PolyTree, nodes: &[usize]) -> PolyNodes {
        // collect ordering points
        let mut ordering_points: Points = Vec::with_capacity(nodes.len());

        for &node in nodes {
            let front = &tree.node(node).contour[0];
            ordering_points.push(Point::new(front.x as i64, front.y as i64));
        }

        // perform the ordering
        // ShortestPath.cpp:1975-1978 `chain_clipper_polynodes(points, items)` is `chain_path_items(points, items)`: the
        // items in the order `chain_points` gives their points.
        let ordered_nodes: PolyNodes = crate::shortest_path::chain_points(&ordering_points, None).into_iter().map(|idx| nodes[idx]).collect();

        ordered_nodes
    }

    // Implementing generalized loop (foreach) over a list of nodes which can be
    // ordered or unordered (performance gain) based on template parameter
    // ClipperUtils.hpp:575-578 `enum class e_ordering`
    #[allow(non_camel_case_types)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum e_ordering {
        ON,
        OFF,
    }

    // Wrapper function for the foreach_node which can deduce arguments automatically
    // ClipperUtils.hpp:580-607 `template<e_ordering o, class Fn> void foreach_node(const ClipperLib::PolyNodes &nodes, Fn &&fn)`, with
    // the two specializations of `_foreach_node`.
    pub fn foreach_node(o: e_ordering, tree: &PolyTree, nodes: &[usize], func: &mut dyn FnMut(usize)) {
        match o {
            // Specialization with NO ordering
            e_ordering::OFF => {
                for &n in nodes {
                    func(n);
                }
            }
            // Specialization with ordering
            e_ordering::ON => {
                // `auto ordered_nodes = order_nodes(nodes); for (auto &n : nodes) fn(n);`: the pin orders the nodes and
                // then walks them in their own order.
                let _ordered_nodes = order_nodes(tree, nodes);
                for &n in nodes {
                    func(n);
                }
            }
        }
    }

    /// The two `traverse_pt` of a node, ClipperUtils.hpp:609-650: what they collect the polygons of the tree into.
    pub trait TraversePt: Sized {
        fn traverse_pt(ordering: e_ordering, polytree: &PolyTree, tree: Option<usize>, out: &mut Self);
    }
    // Collecting polygons of the tree into a list of Polygons, holes have clockwise
    // orientation.
    // ClipperUtils.hpp:611-621 `template<e_ordering ordering = e_ordering::OFF> void traverse_pt(const ClipperLib::PolyNode *tree, Polygons *out)`
    impl TraversePt for Polygons {
        fn traverse_pt(ordering: e_ordering, polytree: &PolyTree, tree: Option<usize>, out: &mut Polygons) {
            let Some(tree) = tree else {
                return; // terminates recursion
            };

            // Push the contour of the current level
            out.push(Polygon::from_points(points_of_path(&polytree.node(tree).contour)));

            // Do the recursion for all the children.
            traverse_pt(ordering, polytree, &polytree.node(tree).childs, out);
        }
    }
    // Collecting polygons of the tree into a list of ExPolygons.
    // ClipperUtils.hpp:623-650 `template<e_ordering ordering = e_ordering::OFF> void traverse_pt(const ClipperLib::PolyNode *tree, ExPolygons *out)`
    impl TraversePt for ExPolygons {
        fn traverse_pt(ordering: e_ordering, polytree: &PolyTree, tree: Option<usize>, out: &mut ExPolygons) {
            let Some(tree) = tree else {
                return;
            };
            if polytree.is_hole(tree) {
                // Levels of holes are skipped and handled together with the
                // contour levels.
                traverse_pt(ordering, polytree, &polytree.node(tree).childs, out);
                return;
            }

            let mut level = ExPolygon::new(Polygon::from_points(points_of_path(&polytree.node(tree).contour)));

            foreach_node(ordering, polytree, &polytree.node(tree).childs, &mut |node: usize| {
                // Holes are collected here.
                level.holes.push(Polygon::from_points(points_of_path(&polytree.node(node).contour)));

                // By doing a recursion, a new level expoly is created with the contour
                // and holes of the lower level. Doing this for all the childs.
                traverse_pt(ordering, polytree, &polytree.node(node).childs, out);
            });

            out.push(level);
        }
    }

    // ClipperUtils.hpp:652-658
    // `template<e_ordering o = e_ordering::OFF, class ExOrJustPolygons> void traverse_pt(const ClipperLib::PolyNodes &nodes, ExOrJustPolygons *retval)`
    pub fn traverse_pt<ExOrJustPolygons: TraversePt>(o: e_ordering, polytree: &PolyTree, nodes: &[usize], retval: &mut ExOrJustPolygons) {
        foreach_node(o, polytree, nodes, &mut |node: usize| {
            TraversePt::traverse_pt(o, polytree, Some(node), retval);
        });
    }

    // ClipperUtils.cpp:989-997 `static void traverse_pt_noholes(const ClipperLib::PolyNodes &nodes, Polygons *out)`
    pub fn traverse_pt_noholes(polytree: &PolyTree, nodes: &[usize], out: &mut Polygons) {
        foreach_node(e_ordering::ON, polytree, nodes, &mut |node: usize| {
            traverse_pt_noholes(polytree, &polytree.node(node).childs, out);
            out.push(Polygon::from_points(points_of_path(&polytree.node(node).contour)));
            if polytree.is_hole(node) {
                out.last_mut().unwrap().points.reverse(); // ccw
            }
        });
    }

    // ClipperUtils.cpp:999-1017 `static void traverse_pt_outside_in(ClipperLib::PolyNodes &&nodes, Polygons *retval)`
    pub fn traverse_pt_outside_in(polytree: &PolyTree, nodes: &[usize], retval: &mut Polygons) {
        // collect ordering points
        let mut ordering_points: Points = Vec::with_capacity(nodes.len());
        for &node in nodes {
            let front = &polytree.node(node).contour[0];
            ordering_points.push(Point::new(front.x as i64, front.y as i64));
        }

        // Perform the ordering, push results recursively.
        //FIXME pass the last point to chain_clipper_polynodes?
        for idx in crate::shortest_path::chain_points(&ordering_points, None) {
            let node = nodes[idx];
            retval.push(Polygon::from_points(points_of_path(&polytree.node(node).contour)));
            if polytree.is_hole(node) {
                // Orient a hole, which is clockwise oriented, to CCW.
                retval.last_mut().unwrap().points.reverse();
            }
            // traverse the next depth
            traverse_pt_outside_in(polytree, &polytree.node(node).childs, retval);
        }
    }

    // ClipperUtils.cpp:1019-1024 `Polygons union_pt_chained_outside_in(const Polygons &subject)`
    pub fn union_pt_chained_outside_in(subject: &[Polygon]) -> Polygons {
        let mut retval: Polygons = Vec::new();
        let tree = UnionPt::union_pt(subject);
        traverse_pt_outside_in(&tree, &tree.root.childs, &mut retval);
        retval
    }

    /* OTHER */
    // ClipperUtils.cpp:1026-1041 `Polygons simplify_polygons(const Polygons &subject, bool preserve_collinear)`
    pub fn simplify_polygons(subject: &[Polygon], preserve_collinear: bool) -> Polygons {
        let output: Paths;
        if preserve_collinear {
            let mut c = crate::clipper::Clipper::new(0);
            c.set_preserve_collinear(true);
            c.set_strictly_simple(true);
            c.add_paths(&PolygonsProvider::new(subject).to_clipper_paths(), crate::clipper::PolyType::ptSubject, true);
            let mut out = Paths::new();
            c.execute(ClipType::ctUnion, &mut out, PolyFillType::pftNonZero, PolyFillType::pftNonZero);
            output = out;
        } else {
            output = crate::clipper::simplify_polygons(&PolygonsProvider::new(subject).to_clipper_paths(), PolyFillType::pftNonZero);
        }

        // convert into Slic3r polygons
        to_polygons(output)
    }

    // ClipperUtils.cpp:1043-1057 `ExPolygons simplify_polygons_ex(const Polygons &subject, bool preserve_collinear)`
    pub fn simplify_polygons_ex(subject: &[Polygon], preserve_collinear: bool) -> ExPolygons {
        if !preserve_collinear {
            return UnionEx::union_ex(simplify_polygons(subject, false).as_slice(), PolyFillType::pftNonZero);
        }

        let mut polytree = PolyTree::new();
        let mut c = crate::clipper::Clipper::new(0);
        c.set_preserve_collinear(true);
        c.set_strictly_simple(true);
        c.add_paths(&PolygonsProvider::new(subject).to_clipper_paths(), crate::clipper::PolyType::ptSubject, true);
        c.execute_poly_tree(ClipType::ctUnion, &mut polytree, PolyFillType::pftNonZero, PolyFillType::pftNonZero);

        // convert into ExPolygons
        poly_tree_to_ex_polygons(polytree)
    }

    // ClipperUtils.cpp:1059-1074 `Polygons top_level_islands(const Slic3r::Polygons &polygons)`
    pub fn top_level_islands(polygons: &[Polygon]) -> Polygons {
        // init Clipper
        let mut clipper = crate::clipper::Clipper::new(0);
        clipper.clear();
        // perform union
        clipper.add_paths(&PolygonsProvider::new(polygons).to_clipper_paths(), crate::clipper::PolyType::ptSubject, true);
        let mut polytree = PolyTree::new();
        clipper.execute_poly_tree(ClipType::ctUnion, &mut polytree, PolyFillType::pftEvenOdd, PolyFillType::pftEvenOdd);
        // Convert only the top level islands to the output.
        let mut out: Polygons = Vec::with_capacity(polytree.child_count() as usize);
        for i in 0..polytree.child_count() as usize {
            out.push(Polygon::from_points(points_of_path(&polytree.node(polytree.root.childs[i]).contour)));
        }
        out
    }

    // ClipperUtils.cpp:1120-1258
    // `ClipperLib::Path mittered_offset_path_scaled(const Points &contour, const std::vector<float> &deltas, double miter_limit)`
    pub fn mittered_offset_path_scaled(contour: &[Point], deltas: &[f32], miter_limit: f64) -> Path {
        debug_assert!(contour.len() == deltas.len());
        super::mittered_offset_path_scaled(&path_of_points(contour), deltas, miter_limit)
    }

    // ClipperUtils.cpp:1260-1300
    // `Polygons variable_offset_inner(const ExPolygon &expoly, const std::vector<std::vector<float>> &deltas, double miter_limit)`
    pub fn variable_offset_inner(expoly: &ExPolygon, deltas: &[Vec<f32>], miter_limit: f64) -> Polygons {
        // 1) Offset the outer contour.
        let contours: Paths =
            super::fix_after_inner_offset(&mittered_offset_path_scaled(&expoly.contour.points, &deltas[0], miter_limit), PolyFillType::pftNegative, true);

        // 2) Offset the holes one by one, collect the results.
        let mut holes = Paths::new();
        holes.reserve(expoly.holes.len());
        for (i, hole) in expoly.holes.iter().enumerate() {
            holes.extend(super::fix_after_outer_offset(&mittered_offset_path_scaled(&hole.points, &deltas[1 + i], miter_limit), PolyFillType::pftNegative, false));
        }

        // 3) Subtract holes from the contours.
        let mut output = Paths::new();
        if holes.is_empty() {
            output = contours;
        } else {
            let mut clipper = crate::clipper::Clipper::new(0);
            clipper.clear();
            clipper.add_paths(&contours, crate::clipper::PolyType::ptSubject, true);
            clipper.add_paths(&holes, crate::clipper::PolyType::ptClip, true);
            clipper.execute(ClipType::ctDifference, &mut output, PolyFillType::pftNonZero, PolyFillType::pftNonZero);
        }

        to_polygons(output)
    }

    // ClipperUtils.cpp:1302-1342
    // `Polygons variable_offset_outer(const ExPolygon &expoly, const std::vector<std::vector<float>> &deltas, double miter_limit)`
    pub fn variable_offset_outer(expoly: &ExPolygon, deltas: &[Vec<f32>], miter_limit: f64) -> Polygons {
        // 1) Offset the outer contour.
        let contours: Paths =
            super::fix_after_outer_offset(&mittered_offset_path_scaled(&expoly.contour.points, &deltas[0], miter_limit), PolyFillType::pftPositive, false);

        // 2) Offset the holes one by one, collect the results.
        let mut holes = Paths::new();
        holes.reserve(expoly.holes.len());
        for (i, hole) in expoly.holes.iter().enumerate() {
            holes.extend(super::fix_after_inner_offset(&mittered_offset_path_scaled(&hole.points, &deltas[1 + i], miter_limit), PolyFillType::pftPositive, true));
        }

        // 3) Subtract holes from the contours.
        let mut output = Paths::new();
        if holes.is_empty() {
            output = contours;
        } else {
            let mut clipper = crate::clipper::Clipper::new(0);
            clipper.clear();
            clipper.add_paths(&contours, crate::clipper::PolyType::ptSubject, true);
            clipper.add_paths(&holes, crate::clipper::PolyType::ptClip, true);
            clipper.execute(ClipType::ctDifference, &mut output, PolyFillType::pftNonZero, PolyFillType::pftNonZero);
        }

        to_polygons(output)
    }

    // ClipperUtils.cpp:1344-1387
    // `ExPolygons variable_offset_outer_ex(const ExPolygon &expoly, const std::vector<std::vector<float>> &deltas, double miter_limit)`
    pub fn variable_offset_outer_ex(expoly: &ExPolygon, deltas: &[Vec<f32>], miter_limit: f64) -> ExPolygons {
        // 1) Offset the outer contour.
        let contours: Paths =
            super::fix_after_outer_offset(&mittered_offset_path_scaled(&expoly.contour.points, &deltas[0], miter_limit), PolyFillType::pftPositive, false);

        // 2) Offset the holes one by one, collect the results.
        let mut holes = Paths::new();
        holes.reserve(expoly.holes.len());
        for (i, hole) in expoly.holes.iter().enumerate() {
            holes.extend(super::fix_after_inner_offset(&mittered_offset_path_scaled(&hole.points, &deltas[1 + i], miter_limit), PolyFillType::pftPositive, true));
        }

        // 3) Subtract holes from the contours.
        let mut output: ExPolygons = Vec::new();
        if holes.is_empty() {
            output.reserve(contours.len());
            for path in &contours {
                output.push(ExPolygon::new(Polygon::from_points(points_of_path(path))));
            }
        } else {
            let mut clipper = crate::clipper::Clipper::new(0);
            clipper.add_paths(&contours, crate::clipper::PolyType::ptSubject, true);
            clipper.add_paths(&holes, crate::clipper::PolyType::ptClip, true);
            let mut polytree = PolyTree::new();
            clipper.execute_poly_tree(ClipType::ctDifference, &mut polytree, PolyFillType::pftNonZero, PolyFillType::pftNonZero);
            output = poly_tree_to_ex_polygons(polytree);
        }

        output
    }

    // ClipperUtils.cpp:1390-1433
    // `ExPolygons variable_offset_inner_ex(const ExPolygon &expoly, const std::vector<std::vector<float>> &deltas, double miter_limit)`
    pub fn variable_offset_inner_ex(expoly: &ExPolygon, deltas: &[Vec<f32>], miter_limit: f64) -> ExPolygons {
        // 1) Offset the outer contour.
        let contours: Paths =
            super::fix_after_inner_offset(&mittered_offset_path_scaled(&expoly.contour.points, &deltas[0], miter_limit), PolyFillType::pftNegative, true);

        // 2) Offset the holes one by one, collect the results.
        let mut holes = Paths::new();
        holes.reserve(expoly.holes.len());
        for (i, hole) in expoly.holes.iter().enumerate() {
            holes.extend(super::fix_after_outer_offset(&mittered_offset_path_scaled(&hole.points, &deltas[1 + i], miter_limit), PolyFillType::pftNegative, false));
        }

        // 3) Subtract holes from the contours.
        let mut output: ExPolygons = Vec::new();
        if holes.is_empty() {
            output.reserve(contours.len());
            for path in &contours {
                output.push(ExPolygon::new(Polygon::from_points(points_of_path(path))));
            }
        } else {
            let mut clipper = crate::clipper::Clipper::new(0);
            clipper.add_paths(&contours, crate::clipper::PolyType::ptSubject, true);
            clipper.add_paths(&holes, crate::clipper::PolyType::ptClip, true);
            let mut polytree = PolyTree::new();
            clipper.execute_poly_tree(ClipType::ctDifference, &mut polytree, PolyFillType::pftNonZero, PolyFillType::pftNonZero);
            output = poly_tree_to_ex_polygons(polytree);
        }

        output
    }
}
