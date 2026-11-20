//! Tree support generation (2D algorithm).
//!
//! 1:1 faithful port of:
//! - Support/TreeSupport.hpp
//! - Support/TreeSupport.cpp
//!
//! This module implements the original 2D tree support algorithm. Tree supports
//! grow organic branch structures from overhang points down to the build plate,
//! using per-layer 2D collision avoidance.
//!
//! PORTING STATUS (partial): the standalone geometry helpers, the node and layer
//! height structures, `TreeSupportData` (the collision / avoidance caches and the
//! node arena), the profiler, and the `TreeSupport` members that work on the
//! object's layers and the node graph are ported in C++ order. What is left is
//! listed at the bottom of the file with what blocks it.
//!
//! The node graph. C++ links `SupportNode`s by raw pointers into
//! `TreeSupportData::contact_nodes` (`std::vector<std::unique_ptr<SupportNode>>`),
//! which owns every node. Here that vector owns the nodes by value and a
//! `SupportNode*` is the index of the node in it (`NodeId`); `nullptr` is `None`.
//! Nodes are never removed from the arena before `clear_nodes()`, so an index
//! stays valid exactly as long as the C++ pointer does.
//!
//! NOTE: no live code path calls into this file yet; the slicing pipeline still
//! runs `support/mod.rs` (`PrintObject::generate_support_material`).

use crate::clipper_utils::OffsetJoinType;
use crate::geometry::{
    get_extents_expoly, turn90_ccw, BoundingBox, ExPolygon, ExPolygons, Line, Lines, Point, Polygon,
    Polygons,
};
use crate::{Coord, CoordF};
use crate::config::{ConfigOption, ConfigOptionFloat, ConfigOptionResolver};
use crate::layer::Layer;
use crate::print::faithful::{Print, PrintObject};
use crate::print_config::enums::{SupportMaterialPattern, SupportMaterialStyle, SupportType};
use crate::print_config::config_def::DynamicPrintConfig;
use crate::slicing::SlicingParams;
use crate::support::support_parameters::SupportParameters;
use crate::support_merge_plan::SupportMergePlan;
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::Arc;

/// `SupportNode*`: the index of the node in `TreeSupportData::contact_nodes`.
pub type NodeId = usize;

// libslic3r.h:23 `static constexpr double SCALING_FACTOR = 0.000001;` as BambuStudio
// defines it (0.00001), not the crate's inverse `crate::SCALING_FACTOR`.
const SCALING_FACTOR: f64 = crate::libslic3r::SCALING_FACTOR;
// libslic3r.h:24 `static constexpr double EPSILON = 1e-4;`
const EPSILON: f64 = crate::libslic3r::EPSILON;
// ClipperLib::pftNonZero
const PFT_NON_ZERO: i32 = 1;

// ============================================================================
// ClipperUtils.hpp entry points used by this translation unit, over the
// vendored ClipperLib (the `*_clib` functions of clipper_utils). Deltas are the
// scaled `float` the C++ signature takes.
// ============================================================================

// ClipperUtils.hpp `ExPolygons union_ex(const ExPolygons &subject)`
fn union_ex(subject: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::union_ex_clib(&to_polygons(subject), PFT_NON_ZERO)
}

// ClipperUtils.hpp `ExPolygons union_ex(const Polygons &subject)`
fn union_ex_polygons(subject: &[Polygon]) -> ExPolygons {
    crate::clipper_utils::union_ex_clib(subject, PFT_NON_ZERO)
}

// ClipperUtils.hpp `ExPolygons diff_ex(const ExPolygons &subject, const ExPolygons &clip)`
fn diff_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::difference_clib(subject, clip)
}

// ClipperUtils.hpp `ExPolygons diff_ex(const ExPolygons &subject, const Polygons &clip)`
fn diff_ex_polygons(subject: &[ExPolygon], clip: &[Polygon]) -> ExPolygons {
    crate::clipper_utils::difference_clib(subject, &crate::clipper_utils::polygons_as_paths(clip))
}

// ClipperUtils.hpp `ExPolygons intersection_ex(const ExPolygons &subject, const ExPolygons &clip)`
fn intersection_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::intersection_clib(subject, clip)
}

// ClipperUtils.hpp `ExPolygons intersection_ex(const ExPolygons &subject, const Polygons &clip)`
fn intersection_ex_polygons(subject: &[ExPolygon], clip: &[Polygon]) -> ExPolygons {
    crate::clipper_utils::intersection_clib(subject, &crate::clipper_utils::polygons_as_paths(clip))
}

// ClipperUtils.hpp `ExPolygons offset_ex(const ExPolygons &expolygons, const float delta,
// ClipperLib::JoinType joinType = DefaultJoinType, double miterLimit = DefaultMiterLimit)`
fn offset_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
    offset_ex_jt(expolygons, delta, OffsetJoinType::Miter, 3.0)
}

fn offset_ex_jt(expolygons: &[ExPolygon], delta: f32, join_type: OffsetJoinType, miter_limit: f64) -> ExPolygons {
    crate::clipper_utils::offset_ex_expolygons_clib_ml(expolygons, delta as f64, join_type, miter_limit)
}

// ClipperUtils.hpp `Polygons offset(const ExPolygons &expolygons, const float delta, ...)`:
// each expolygon offset on its own, the paths concatenated.
fn offset_expolygons_jt(expolygons: &[ExPolygon], delta: f32, join_type: OffsetJoinType) -> Polygons {
    crate::clipper_utils::offset_expolygons_clib_raw_scaled(expolygons, delta as f64, join_type)
}

// ClipperUtils.hpp `Polygons offset(const Polygon &polygon, const float delta, ...)`
fn offset_polygon(polygon: &Polygon, delta: f32) -> Polygons {
    crate::clipper_utils::offset_polygons_clib(std::slice::from_ref(polygon), delta as f64, OffsetJoinType::Miter)
}

// ClipperUtils.hpp `ExPolygons offset2_ex(const ExPolygons &expolygons, const float delta1,
// const float delta2, ...)`. offset2_ex_clib takes the deltas in mm and truncates
// `mm / SCALING_FACTOR` to float, which gives back the scaled float handed in here.
fn offset2_ex(expolygons: &[ExPolygon], delta1: f32, delta2: f32) -> ExPolygons {
    crate::clipper_utils::offset2_ex_clib(
        expolygons,
        delta1 as f64 * SCALING_FACTOR,
        delta2 as f64 * SCALING_FACTOR,
        OffsetJoinType::Miter,
    )
}

// ClipperUtils.hpp `ExPolygons opening_ex(const ExPolygons &src, const float delta)`
// = offset2_ex(src, -delta, delta).
fn opening_ex(src: &[ExPolygon], delta: f32) -> ExPolygons {
    offset2_ex(src, -delta, delta)
}

// Polygon.hpp `Polygons to_polygons(const ExPolygons &src)`
fn to_polygons(src: &[ExPolygon]) -> Polygons {
    let mut out: Polygons = Vec::new();
    for expoly in src {
        out.push(expoly.contour.clone());
        for hole in &expoly.holes {
            out.push(hole.clone());
        }
    }
    out
}

// ExPolygon.hpp `void append(ExPolygons &dst, const ExPolygons &src)`
fn append(dst: &mut ExPolygons, src: ExPolygons) {
    dst.extend(src);
}

// ExPolygon.cpp:231-251 `Polygons ExPolygon::simplify_p(double tolerance) const`,
// tolerance scaled: Douglas-Peucker on the closed contour and holes, then
// `simplify_polygons(pp)` (ClipperLib::SimplifyPolygons, pftNonZero).
fn expolygon_simplify_p(expoly: &ExPolygon, tolerance: f64) -> Polygons {
    crate::clipper_utils::simplify_polygons_clib(&expoly.simplify_p_dp_rings_faithful(tolerance), PFT_NON_ZERO)
}

// ExPolygon.hpp:451-458 `ExPolygons expolygons_simplify(const ExPolygons &expolys, double tolerance)`:
// `exp.simplify(tolerance, &out)` = `append(out, union_ex(exp.simplify_p(tolerance)))`.
fn expolygons_simplify(expolys: &[ExPolygon], tolerance: f64) -> ExPolygons {
    let mut out: ExPolygons = Vec::with_capacity(expolys.len());
    for exp in expolys {
        append(&mut out, union_ex_polygons(&expolygon_simplify_p(exp, tolerance)));
    }
    out
}

// BoundingBox.hpp `double radius() const { return 0.5 * (max - min).cast<double>().norm(); }`
// Eigen's norm() sums the squares in its reduction functor, a separate expression
// from the products, so nothing is contracted.
fn bbox_radius(bbox: &BoundingBox) -> f64 {
    let dx = (bbox.max.x - bbox.min.x) as f64;
    let dy = (bbox.max.y - bbox.min.y) as f64;
    0.5 * (dx * dx + dy * dy).sqrt()
}

// Eigen `(v).cast<double>().norm()` of a Point.
fn point_norm(v: Point) -> f64 {
    let x = v.x as f64;
    let y = v.y as f64;
    (x * x + y * y).sqrt()
}

// ExPolygon.hpp: const Polygon& contour_or_hole(int i) const
// Returns the contour for i==0, otherwise holes[i-1]. (Mirrors BambuStudio's helper.)
#[inline]
fn contour_or_hole(expoly: &ExPolygon, i: usize) -> &Polygon {
    if i == 0 {
        &expoly.contour
    } else {
        &expoly.holes[i - 1]
    }
}

// TreeSupport.hpp:17-19 (SQ macro)
// #define SQ(x) ((x)*(x))
#[inline]
fn sq(x: f64) -> f64 {
    x * x
}

// TreeSupport.cpp:37-39
// #ifndef M_PI ... 3.1415926535897932384626433832795
const M_PI: f64 = std::f64::consts::PI;

// TreeSupport.cpp:40-42
// #define SIGN(x) (x>=0?1:-1)
#[inline]
fn sign(x: f64) -> i32 {
    if x >= 0.0 {
        1
    } else {
        -1
    }
}

// TreeSupport.cpp:43
// #define TAU (2.0 * M_PI)
const TAU: f64 = 2.0 * M_PI;

// TreeSupport.cpp:44
// #define NO_INDEX (std::numeric_limits<unsigned int>::max())
const NO_INDEX: u32 = u32::MAX;

// TreeSupport.cpp:45
// #define USE_SUPPORT_3D 0
const USE_SUPPORT_3D: bool = false;

// Branch radius limits (TreeSupport.hpp:447-450)
const MAX_BRANCH_RADIUS: CoordF = 10.0;
const MIN_BRANCH_RADIUS: CoordF = 0.4;
#[allow(dead_code)]
const MAX_BRANCH_RADIUS_FIRST_LAYER: CoordF = 12.0;
#[allow(dead_code)]
const MIN_BRANCH_RADIUS_FIRST_LAYER: CoordF = 2.0;

// TreeSupport.cpp:55
// #define unscale_(val) ((val) * SCALING_FACTOR)
#[inline]
fn unscale_(val: Coord) -> f64 {
    val as f64 * SCALING_FACTOR
}

// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`: a double; the
// call site converts it where the C++ does.
#[inline]
fn scale_(val: f64) -> f64 {
    val / SCALING_FACTOR
}

// Point.hpp:200 `Point operator*(const double &rhs) { return Point(this->x() * rhs, this->y() * rhs); }`,
// the member chosen for a non-const Point: Point(double, double) rounds with lrint.
#[inline]
fn point_mul(pt: Point, r: f64) -> Point {
    Point::new((pt.x as f64 * r).round_ties_even() as Coord, (pt.y as f64 * r).round_ties_even() as Coord)
}

// Point.hpp:179 `Point(double x, double y) : Vec2crd(coord_t(lrint(x)), coord_t(lrint(y)))`
#[inline]
fn point_from_f64(x: f64, y: f64) -> Point {
    Point::new(x.round_ties_even() as Coord, y.round_ties_even() as Coord)
}

// Point.cpp:157-180 `Point Point::projection_onto(const Line &line) const`. A local copy:
// `Point::project_onto_segment` of the crate takes the differences in 32 bits.
fn point_projection_onto_line(pt: &Point, a: Point, b: Point) -> Point {
    if a == b {
        return a;
    }
    let lx = (b.x - a.x) as f64;
    let ly = (b.y - a.y) as f64;
    // `( (double)(b(0) - x)*lx + (double)(b(1) - y)*ly ) / ( sqr<double>(lx) + sqr<double>(ly) )`:
    // the left product of the numerator is contracted with the sum, the squares are calls.
    let theta = ((b.x - pt.x) as f64).mul_add(lx, (b.y - pt.y) as f64 * ly) / (lx * lx + ly * ly);
    if 0.0 <= theta && theta <= 1.0 {
        // `(theta * a.cast<coordf_t>() + (1.0-theta) * b.cast<coordf_t>()).cast<coord_t>()`: Eigen's
        // functors, nothing contracted; the cast truncates.
        return Point::new(
            (theta * a.x as f64 + (1.0 - theta) * b.x as f64) as Coord,
            (theta * a.y as f64 + (1.0 - theta) * b.y as f64) as Coord,
        );
    }
    // Else pick closest endpoint.
    let sqn = |p: Point| {
        let (x, y) = ((p.x - pt.x) as f64, (p.y - pt.y) as f64);
        x * x + y * y
    };
    if sqn(a) < sqn(b) {
        a
    } else {
        b
    }
}

// Point.cpp:141-155 `Point Point::projection_onto(const MultiPoint &poly) const` on a Polygon:
// `poly.lines()` is virtual, Polygon::lines() (Polygon.hpp:199-209 `to_lines`) closes the contour and
// is empty under three points. A local copy: `Point::projection_onto_multipoint` of the crate leaves
// the closing edge out.
fn point_projection_onto_polygon(pt: &Point, poly: &Polygon) -> Point {
    let mut running_projection = poly.points[0];
    let mut running_min = point_norm(Point::new(running_projection.x - pt.x, running_projection.y - pt.y));
    let n = poly.points.len();
    if n > 2 {
        for i in 0..n {
            let (a, b) = (poly.points[i], poly.points[if i + 1 == n { 0 } else { i + 1 }]);
            let point_temp = point_projection_onto_line(pt, a, b);
            if point_norm(Point::new(point_temp.x - pt.x, point_temp.y - pt.y)) < running_min {
                running_projection = point_temp;
                running_min = point_norm(Point::new(running_projection.x - pt.x, running_projection.y - pt.y));
            }
        }
    }
    running_projection
}

// ExPolygon.cpp:206-223 `Point projection_onto(const ExPolygons& polygons, const Point& from)`
pub fn projection_onto(polygons: &[ExPolygon], from: &Point) -> Point {
    let mut projected_pt = Point::new(0, 0);
    let mut min_dist = f64::MAX;
    for poly in polygons {
        for i in 0..poly.num_contours() {
            let p = point_projection_onto_polygon(from, contour_or_hole(poly, i));
            let (dx, dy) = ((from.x - p.x) as f64, (from.y - p.y) as f64);
            let dist = dx * dx + dy * dy;
            if dist < min_dist {
                projected_pt = p;
                min_dist = dist;
            }
        }
    }
    projected_pt
}

// ============================================================================
// The ClipperUtils.cpp / ClipperUtils.hpp overloads the functions below call, each on the building blocks
// of `crate::clipper_utils` (`_clipper_ex`, `expolygons_offset_pt`, ... over the ClipperLib port
// `crate::clipper`), named after the C++ argument types. Which overload a call resolves to matters:
// `offset_ex({expolygon}, delta)` is the overload of one ExPolygon, not the one of ExPolygons.
// ============================================================================
mod cx {
    use crate::clipper::{ClipType, EndType, IntPoint, JoinType, Path, Paths, PolyFillType, PolyTree};
    use crate::clipper_utils as u;
    use crate::clipper_utils::ApplySafetyOffset;
    use crate::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polygons, Polyline, Polylines};

    const NON_ZERO: PolyFillType = PolyFillType::pftNonZero;
    /// ClipperUtils.hpp:31 `DefaultJoinType = jtMiter`
    pub const DEFAULT_JOIN_TYPE: JoinType = JoinType::jtMiter;
    /// ClipperUtils.hpp:37 `DefaultMiterLimit = 3.`
    pub const DEFAULT_MITER_LIMIT: f64 = 3.;

    fn path_of(points: &[Point]) -> Path {
        points.iter().map(|p| IntPoint::new(p.x as _, p.y as _)).collect()
    }
    fn points_of(path: &Path) -> Vec<Point> {
        path.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()
    }
    /// `ClipperUtils::PolygonsProvider`
    pub fn polygons_provider(polygons: &[Polygon]) -> Paths {
        polygons.iter().map(|p| path_of(&p.points)).collect()
    }
    /// `ClipperUtils::ExPolygonsProvider` / `ExPolygonProvider`: per expolygon its contour, then its holes.
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
    /// ClipperUtils.cpp `Polygons to_polygons(ClipperLib::Paths &&paths)`
    pub fn to_polygons(paths: &Paths) -> Polygons {
        paths.iter().map(|p| Polygon::from_points(points_of(p))).collect()
    }
    fn to_polylines_of_paths(paths: &Paths) -> Polylines {
        paths.iter().map(|p| Polyline::from_points(points_of(p))).collect()
    }
    fn expolygon_paths(expolygons: &[ExPolygon]) -> Vec<(Path, Paths)> {
        expolygons.iter().map(|e| (path_of(&e.contour.points), e.holes.iter().map(|h| path_of(&h.points)).collect())).collect()
    }

    /// ClipperUtils.cpp:755 `diff_ex(const ExPolygons &subject, const ExPolygons &clip)`
    pub fn diff_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctDifference, &expolygons_provider(subject), &expolygons_provider(clip), ApplySafetyOffset::No, NON_ZERO)
    }
    /// ClipperUtils.cpp:753 `diff_ex(const ExPolygons &subject, const Polygons &clip)`
    pub fn diff_ex_polygons(subject: &[ExPolygon], clip: &[Polygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctDifference, &expolygons_provider(subject), &polygons_provider(clip), ApplySafetyOffset::No, NON_ZERO)
    }
    /// ClipperUtils.cpp:802 `intersection_ex(const ExPolygons &subject, const ExPolygons &clip)` and
    /// ClipperUtils.cpp:798 `intersection_ex(const ExPolygons &subject, const ExPolygon &clip)` on a slice of one.
    pub fn intersection_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctIntersection, &expolygons_provider(subject), &expolygons_provider(clip), ApplySafetyOffset::No, NON_ZERO)
    }
    /// ClipperUtils.cpp:796 `intersection_ex(const ExPolygons &subject, const Polygons &clip)`
    pub fn intersection_ex_polygons(subject: &[ExPolygon], clip: &[Polygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctIntersection, &expolygons_provider(subject), &polygons_provider(clip), ApplySafetyOffset::No, NON_ZERO)
    }
    /// ClipperUtils.cpp:815 `union_ex(const ExPolygons &subject)`
    pub fn union_ex(subject: &[ExPolygon]) -> ExPolygons {
        u::poly_tree_to_ex_polygons(&u::clipper_do_polytree(ClipType::ctUnion, &expolygons_provider(subject), &Paths::new(), NON_ZERO))
    }
    /// ClipperUtils.cpp:813 `union_ex(const Polygons &subject, PolyFillType fill_type = pftNonZero)`
    pub fn union_ex_polygons(subject: &[Polygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctUnion, &polygons_provider(subject), &Paths::new(), ApplySafetyOffset::No, NON_ZERO)
    }
    /// ClipperUtils.cpp:825-831 `union_ex(const ExPolygons &poly1, const ExPolygons &poly2, bool safety_offset_ = false)`
    /// (BBS: the two concatenated, then `union_ex`).
    pub fn union_ex_2(poly1: &[ExPolygon], poly2: &[ExPolygon]) -> ExPolygons {
        let mut expolys: ExPolygons = poly1.to_vec();
        for expoly in poly2 {
            expolys.push(expoly.clone());
        }
        union_ex(&expolys)
    }
    /// ClipperUtils.cpp:721 `union_(const Polygons &subject)`
    pub fn union_(subject: &[Polygon]) -> Polygons {
        to_polygons(&u::clipper_do_safety_offset::<Paths>(ClipType::ctUnion, &polygons_provider(subject), &Paths::new(), NON_ZERO, ApplySafetyOffset::No))
    }
    /// ClipperUtils.cpp:727-734 `union_(const Polygons &subject, const Polygons &subject2)` (BBS: the two
    /// concatenated, then `union_`).
    pub fn union_2(subject: &[Polygon], subject2: &[Polygon]) -> Polygons {
        let mut polys: Polygons = subject.to_vec();
        for poly in subject2 {
            polys.push(poly.clone());
        }
        union_(&polys)
    }
    /// ClipperUtils.cpp:570 `offset_ex(const ExPolygons &expolygons, const float delta, JoinType, double miterLimit)`
    pub fn offset_ex(expolygons: &[ExPolygon], delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        u::poly_tree_to_ex_polygons(&u::expolygons_offset_pt(&expolygon_paths(expolygons), delta as f64, join_type, miter_limit))
    }
    /// ClipperUtils.cpp:567-569 `offset_ex(const ExPolygon &expolygon, const float delta, JoinType, double miterLimit)`:
    /// `ClipperPaths_to_Slic3rExPolygons(expolygon_offset(...))`, an even-odd union.
    pub fn offset_ex_expolygon(expolygon: &ExPolygon, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        let (contour, holes) = expolygon_paths(std::slice::from_ref(expolygon)).pop().unwrap();
        u::clipper_paths_to_slic3r_ex_polygons(&u::expolygon_offset(&contour, &holes, delta as f64, join_type, miter_limit), false)
    }
    /// ClipperUtils.cpp:410 `offset(const Polygon &polygon, const float delta, JoinType, double miterLimit)`
    pub fn offset_polygon(polygon: &Polygon, delta: f32, join_type: JoinType, miter_limit: f64) -> Polygons {
        to_polygons(&u::raw_offset(&vec![path_of(&polygon.points)], delta as f64, join_type, miter_limit, EndType::etClosedPolygon))
    }
    /// ClipperUtils.cpp:420 `offset(const Polylines &polylines, const float delta, JoinType, double miterLimit, EndType)`
    pub fn offset_polylines(polylines: &[Polyline], delta: f32, join_type: JoinType, miter_limit: f64, end_type: EndType) -> Polygons {
        to_polygons(&u::clipper_union::<Paths>(
            &u::raw_offset_polyline(&polylines_provider(polylines), delta as f64, join_type, miter_limit, end_type),
            NON_ZERO,
        ))
    }
    /// ClipperUtils.cpp:581-584 `offset2_ex(const ExPolygons &expolygons, const float delta1, const float delta2, JoinType,
    /// double miterLimit)`
    pub fn offset2_ex(expolygons: &[ExPolygon], delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        u::poly_tree_to_ex_polygons(&u::offset_paths::<PolyTree>(
            &u::expolygons_offset(&expolygon_paths(expolygons), delta1 as f64, join_type, miter_limit),
            delta2 as f64,
            join_type,
            miter_limit,
        ))
    }
    /// ClipperUtils.cpp:161-172 `clip_clipper_polygons_with_subject_bbox(const ExPolygons &src, const BoundingBox &bbox)`
    pub fn clip_clipper_polygons_with_subject_bbox(src: &[ExPolygon], bbox: &BoundingBox) -> Polygons {
        u::clip_clipper_polygons_with_subject_bbox_expolygons(src, bbox, false)
    }
    /// ClipperUtils.cpp:683-686 `diff_clipped(const ExPolygons &subject, const ExPolygons &clip)`:
    /// `diff_ex(subject, clip_clipper_polygons_with_subject_bbox(clip, get_extents(subject).inflated(SCALED_EPSILON)))`
    pub fn diff_clipped(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
        let mut bbox = super::get_extents_expolygons(subject);
        // inflated(coordf_t): `Point v(delta, delta)` rounds
        let eps = (crate::libslic3r::EPSILON / crate::libslic3r::SCALING_FACTOR).round_ties_even() as i64;
        bbox.min = Point::new(bbox.min.x - eps, bbox.min.y - eps);
        bbox.max = Point::new(bbox.max.x + eps, bbox.max.y + eps);
        diff_ex_polygons(subject, &clip_clipper_polygons_with_subject_bbox(clip, &bbox))
    }
    /// ExPolygon.hpp:214-229 `to_polylines(const ExPolygon &src)`: every ring closed.
    pub fn to_polylines(src: &ExPolygon) -> Polylines {
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
    /// ClipperUtils.cpp:927 `intersection_pl(const Polylines &subject, const ExPolygon &clip)`
    pub fn intersection_pl(subject: &[Polyline], clip: &ExPolygon) -> Polylines {
        to_polylines_of_paths(&u::_clipper_pl_open(ClipType::ctIntersection, &polylines_provider(subject), &expolygons_provider(std::slice::from_ref(clip))))
    }
    /// ExPolygon.cpp:109-119 `ExPolygon::contains(const Point &point, bool border_result = true)`
    pub fn contains(expoly: &ExPolygon, point: &Point, border_result: bool) -> bool {
        if !crate::geometry::contains_polygon(&expoly.contour, point, border_result) {
            // Outside the outer contour, not on the contour boundary.
            return false;
        }
        for hole in &expoly.holes {
            if crate::geometry::contains_polygon(hole, point, !border_result) {
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
        let pl_out = intersection_pl(&to_polylines(other), this);
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
}

// ExPolygon.cpp:495-504 `BoundingBox get_extents(const ExPolygons &expolygons)`: the box of the contours; a
// default constructed BoundingBox, (0, 0)-(0, 0) and not defined (BoundingBox.hpp:20), when there is none.
fn get_extents_expolygons(expolygons: &[ExPolygon]) -> BoundingBox {
    let mut bbox = BoundingBox::new();
    for expolygon in expolygons {
        if !expolygon.contour.points.is_empty() {
            bbox.merge(&BoundingBox::from_points(&expolygon.contour.points));
        }
    }
    if !bbox.is_defined() {
        bbox.min = Point::new(0, 0);
        bbox.max = Point::new(0, 0);
    }
    bbox
}

// ============================================================================
// Header data structures (TreeSupport.hpp)
// ============================================================================

/// TreeSupport.hpp:27-37: struct LayerHeightData
#[derive(Debug, Clone, Default)]
pub struct LayerHeightData {
    // TreeSupport.hpp:29: coordf_t print_z = 0;
    pub print_z: CoordF,
    // TreeSupport.hpp:30: coordf_t height = 0;
    pub height: CoordF,
    // TreeSupport.hpp:31: size_t obj_layer_nr = 0;
    pub obj_layer_nr: usize,
}

impl LayerHeightData {
    // TreeSupport.hpp:32: LayerHeightData() = default;
    pub fn new() -> Self {
        Self::default()
    }

    // TreeSupport.hpp:33
    // LayerHeightData(coordf_t z, coordf_t h, size_t obj_layer) : print_z(z), height(h), obj_layer_nr(obj_layer) {}
    pub fn from_values(z: CoordF, h: CoordF, obj_layer: usize) -> Self {
        Self {
            print_z: z,
            height: h,
            obj_layer_nr: obj_layer,
        }
    }

    // TreeSupport.hpp:34-36
    // coordf_t bottom_z() { return print_z - height; }
    pub fn bottom_z(&self) -> CoordF {
        self.print_z - self.height
    }
}

/// TreeSupport.hpp:39-43: enum TreeNodeType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TreeNodeType {
    // TreeSupport.hpp:40
    ECircle,
    // TreeSupport.hpp:41
    ESquare,
    // TreeSupport.hpp:42
    EPolygon,
}

impl Default for TreeNodeType {
    fn default() -> Self {
        // SupportNode default: type = eCircle (TreeSupport.hpp:125)
        Self::ECircle
    }
}

/// TreeSupport.cpp / TreeSupport.hpp:422: enum OverhangType : uint8_t
///
/// These are flag bits combined bitwise (Cantilever == 1<<1 etc.).
/// Note: Cantilever and SharpTail==1 share the bit pattern from the header;
/// reproduced verbatim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OverhangType(pub u8);

impl OverhangType {
    // TreeSupport.hpp:422
    pub const NORMAL: OverhangType = OverhangType(0);
    pub const SHARP_TAIL: OverhangType = OverhangType(1);
    pub const CANTILEVER: OverhangType = OverhangType(1 << 1);
    pub const SMALL: OverhangType = OverhangType(1 << 2);
    pub const BIG_FLAT: OverhangType = OverhangType(1 << 3);
    pub const THIN_PLATE: OverhangType = OverhangType(1 << 4);
    pub const SHARP_TAIL_LOWESST: OverhangType = OverhangType(1 << 5);
}

impl Default for OverhangType {
    fn default() -> Self {
        Self::NORMAL
    }
}

/// TreeSupport.hpp:47-218: struct SupportNode
///
/// Represents the metadata of a node in the tree. The pointers of the C++ struct
/// (`parent`, `parents`, `child`, `merged_neighbours`) are `NodeId`s into the arena
/// `TreeSupportData::contact_nodes`, see the module documentation.
#[derive(Debug, Clone)]
pub struct SupportNode {
    /// TreeSupport.hpp:110-114
    /// The number of layers to go to the top of this branch.
    /// Negative value means it's a virtual node between support and overhang, which doesn't need to be extruded.
    pub distance_to_top: i32,
    // TreeSupport.hpp:115 coordf_t dist_mm_to_top = 0;  // dist to bottom contact in mm
    pub dist_mm_to_top: CoordF,

    // TreeSupport.hpp:123 Point position;
    pub position: Point,
    // TreeSupport.hpp:124 Point movement; // movement towards neighbor center or outline
    pub movement: Point,
    // TreeSupport.hpp:125 Point orig_pos;
    pub orig_pos: Point,
    // TreeSupport.hpp:126 mutable double radius = 0.0;
    pub radius: f64,
    // TreeSupport.hpp:127 mutable double max_move_dist = 0.0;
    pub max_move_dist: f64,
    // TreeSupport.hpp:128 TreeNodeType type = eCircle;
    pub type_: TreeNodeType,
    // TreeSupport.hpp:129 bool is_corner = false;
    pub is_corner: bool,
    // TreeSupport.hpp:130 bool is_processed = false;
    pub is_processed: bool,
    // TreeSupport.hpp:131 bool need_extra_wall = false;
    pub need_extra_wall: bool,
    // TreeSupport.hpp:132 bool is_sharp_tail = false;
    pub is_sharp_tail: bool,
    // TreeSupport.hpp:133 bool valid = true;
    pub valid: bool,
    // TreeSupport.hpp:134 float carried_mass = 0.0;  // Virtual carried mass (supported overhang area)
    // used to prioritize light-to-heavy branch merging
    pub carried_mass: f32,

    // TreeSupport.hpp:144-148 BBS multi-diameter: set by fold_nodes_multi_diameter(). slab_height > 0 marks
    // a node that carries a whole slab: its disk is one extrusion spanning [print_z - slab_height, print_z].
    pub slab_height: CoordF,
    // TreeSupport.hpp:149-150 Lowest object layer the slab covers, for the span collision lookup.
    pub slab_bottom_obj_layer: usize,
    // TreeSupport.hpp:151-153 Position of this node on the folded grid, counted from the bed.
    pub slab_index: i32,
    // TreeSupport.hpp:154-156 BBS multi-diameter: wall line width of this base node when it sits next to a
    // nozzle change, set by smooth_line_width_multi_diameter(). 0 keeps the layer's default support line width.
    pub line_width: CoordF,
    // TreeSupport.hpp:157 bool fading = false;
    pub fading: bool,
    // TreeSupport.hpp:158 double overhang_degree = 0.0;  // overhang degree for cooling just like perimeter
    pub overhang_degree: f64,
    // TreeSupport.hpp:159 ExPolygon overhang; // when type==ePolygon, set this value to get original overhang area
    pub overhang: ExPolygon,
    // TreeSupport.hpp:160 coordf_t origin_area;
    pub origin_area: CoordF,
    // TreeSupport.hpp:161 coordf_t target_radius = -1.;
    pub target_radius: CoordF,

    // TreeSupport.hpp:169 Point skin_direction;
    pub skin_direction: Point,

    // TreeSupport.hpp:179 int support_roof_layers_below;
    pub support_roof_layers_below: i32,
    // TreeSupport.hpp:180 int obj_layer_nr;
    pub obj_layer_nr: i32,

    // TreeSupport.hpp:189 bool to_buildplate;
    pub to_buildplate: bool,

    // TreeSupport.hpp:198 SupportNode* parent;
    pub parent: Option<NodeId>,
    // TreeSupport.hpp:199 std::vector<SupportNode*> parents;
    pub parents: Vec<NodeId>,
    // TreeSupport.hpp:200 SupportNode* child = nullptr;
    pub child: Option<NodeId>,

    // TreeSupport.hpp:209 std::list<SupportNode*> merged_neighbours;
    pub merged_neighbours: Vec<NodeId>,

    // TreeSupport.hpp:211 coordf_t print_z;
    pub print_z: CoordF,
    // TreeSupport.hpp:212 coordf_t height;
    pub height: CoordF,
}

// TreeSupport.hpp:118 `static double diameter_angle_scale_factor;` (defined TreeSupport.cpp:5101,
// set by drop_nodes() TreeSupport.cpp:5162). A process-wide static as in C++, kept as the bits of the double.
static SUPPORT_NODE_DIAMETER_ANGLE_SCALE_FACTOR: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

impl SupportNode {
    // TreeSupport.hpp:52 static constexpr SupportNode* NO_PARENT = nullptr;
    pub const NO_PARENT: Option<NodeId> = None;

    /// `SupportNode::diameter_angle_scale_factor`
    pub fn diameter_angle_scale_factor() -> f64 {
        f64::from_bits(SUPPORT_NODE_DIAMETER_ANGLE_SCALE_FACTOR.load(std::sync::atomic::Ordering::Relaxed))
    }

    /// `SupportNode::diameter_angle_scale_factor = value;`
    pub fn set_diameter_angle_scale_factor(value: f64) {
        SUPPORT_NODE_DIAMETER_ANGLE_SCALE_FACTOR.store(value.to_bits(), std::sync::atomic::Ordering::Relaxed);
    }

    // TreeSupport.hpp:54-64: SupportNode() default constructor
    pub fn new() -> Self {
        Self {
            distance_to_top: 0,
            dist_mm_to_top: 0.0,
            position: Point::new(0, 0),
            movement: Point::new(0, 0),
            orig_pos: Point::new(0, 0),
            radius: 0.0,
            max_move_dist: 0.0,
            type_: TreeNodeType::ECircle,
            is_corner: false,
            is_processed: false,
            need_extra_wall: false,
            is_sharp_tail: false,
            valid: true,
            carried_mass: 0.0,
            slab_height: 0.0,
            slab_bottom_obj_layer: 0,
            slab_index: -1,
            line_width: 0.0,
            fading: false,
            overhang_degree: 0.0,
            overhang: ExPolygon::default(),
            origin_area: 0.0,
            target_radius: -1.0,
            skin_direction: Point::new(0, 0),
            support_roof_layers_below: 0,
            obj_layer_nr: 0,
            to_buildplate: true,
            parent: None,
            parents: Vec::new(),
            child: None,
            merged_neighbours: Vec::new(),
            print_z: 0.0,
            height: 0.0,
        }
    }

    /// TreeSupport.hpp:66-100: the full SupportNode constructor.
    ///
    /// `nodes` is the arena the new node will be pushed onto at index `self_idx`
    /// (`this`); the constructor writes `parent->child` and the merged neighbours'
    /// `child` there, as the C++ does through the pointers.
    #[allow(clippy::too_many_arguments)]
    pub fn with_parent(
        nodes: &mut [SupportNode],
        self_idx: NodeId,
        position: Point,
        distance_to_top: i32,
        obj_layer_nr: i32,
        support_roof_layers_below: i32,
        to_buildplate: bool,
        parent: Option<NodeId>,
        print_z_: CoordF,
        height_: CoordF,
        dist_mm_to_top_: CoordF,
        radius_: CoordF,
    ) -> Self {
        let mut node = SupportNode {
            distance_to_top,
            position,
            orig_pos: position,
            obj_layer_nr,
            support_roof_layers_below,
            to_buildplate,
            parent,
            print_z: print_z_,
            height: height_,
            dist_mm_to_top: dist_mm_to_top_,
            radius: radius_,
            ..SupportNode::new()
        };

        // TreeSupport.hpp:81-99
        if let Some(parent_idx) = parent {
            // parents.push_back(parent);
            node.parents.push(parent_idx);
            // type = parent->type;
            node.type_ = nodes[parent_idx].type_;
            // overhang = parent->overhang;
            node.overhang = nodes[parent_idx].overhang.clone();
            // orig_pos = parent->orig_pos;
            node.orig_pos = nodes[parent_idx].orig_pos;
            // carried_mass = parent->carried_mass;
            node.carried_mass = nodes[parent_idx].carried_mass;
            // if (dist_mm_to_top == 0) dist_mm_to_top = parent->dist_mm_to_top + parent->height;
            if node.dist_mm_to_top == 0.0 {
                node.dist_mm_to_top = nodes[parent_idx].dist_mm_to_top + nodes[parent_idx].height;
            }
            // if (radius == 0 && parent->radius>0)
            //     radius = parent->radius + (dist_mm_to_top - parent->dist_mm_to_top) * diameter_angle_scale_factor;
            if node.radius == 0.0 && nodes[parent_idx].radius > 0.0 {
                node.radius = (node.dist_mm_to_top - nodes[parent_idx].dist_mm_to_top)
                    .mul_add(Self::diameter_angle_scale_factor(), nodes[parent_idx].radius);
            }
            // parent->child = this;
            nodes[parent_idx].child = Some(self_idx);
            // for (auto& neighbor : parent->merged_neighbours) {
            //     neighbor->child = this; parents.push_back(neighbor); carried_mass += neighbor->carried_mass; }
            let neighbours = nodes[parent_idx].merged_neighbours.clone();
            for neighbor in neighbours {
                nodes[neighbor].child = Some(self_idx);
                node.parents.push(neighbor);
                node.carried_mass += nodes[neighbor].carried_mass;
            }
            // is_sharp_tail = parent->is_sharp_tail;
            node.is_sharp_tail = nodes[parent_idx].is_sharp_tail;
            // skin_direction = parent->skin_direction;
            node.skin_direction = nodes[parent_idx].skin_direction;
        }

        node
    }

    /// TreeSupport.hpp:136-142 `float total_carried_mass() const`
    pub fn total_carried_mass(&self, nodes: &[SupportNode]) -> f32 {
        let mut mass = self.carried_mass;
        for &neighbor in &self.merged_neighbours {
            mass += nodes[neighbor].carried_mass;
        }
        mass
    }

    // TreeSupport.hpp:214-217 bool operator==(const SupportNode& other) const { return position == other.position; }
    pub fn eq_position(&self, other: &SupportNode) -> bool {
        self.position == other.position
    }
}

impl Default for SupportNode {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for SupportNode {
    // TreeSupport.hpp:214-217
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}

/// TreeSupport.hpp:355-360: struct LineHash
///
/// size_t operator()(const Line& line) const {
///     return (hash(line.a(0)) ^ hash(line.b(1))) * 102 +
///            (hash(line.a(1)) ^ hash(line.b(0))) * 10222;
/// }
#[derive(Debug, Clone, Default)]
pub struct LineHash;

impl LineHash {
    pub fn new() -> Self {
        Self
    }

    pub fn hash(&self, line: &Line) -> u64 {
        // std::hash<coord_t>() on int32_t coords (libslic3r.h:40) is the identity.
        // FIDELITY-NOTE(F2): coord_t is int32_t — reproduce the int32 truncation locally.
        let a0 = (line.a.x as i32) as u64;
        let a1 = (line.a.y as i32) as u64;
        let b0 = (line.b.x as i32) as u64;
        let b1 = (line.b.y as i32) as u64;
        (a0 ^ b1).wrapping_mul(102).wrapping_add((a1 ^ b0).wrapping_mul(10222))
    }
}

/// TreeSupport.hpp:267-272: struct RadiusLayerPair (key for the collision/avoidance caches)
#[derive(Debug, Clone, Copy, Default)]
pub struct RadiusLayerPair {
    // TreeSupport.hpp:268: coordf_t radius;
    pub radius: CoordF,
    // TreeSupport.hpp:269: size_t layer_nr;
    pub layer_nr: usize,
    // TreeSupport.hpp:270: int recursions;
    pub recursions: i32,
}

impl RadiusLayerPair {
    // TreeSupport.hpp:273-277: RadiusLayerPairEquality
    // bool operator()(_Left, _Right) const { return _Left.radius == _Right.radius && _Left.layer_nr == _Right.layer_nr; }
    pub fn equals(&self, other: &RadiusLayerPair) -> bool {
        self.radius == other.radius && self.layer_nr == other.layer_nr
    }

    // TreeSupport.hpp:278-282: RadiusLayerPairHash
    // size_t operator()(const RadiusLayerPair& elem) const {
    //     return std::hash<coord_t>()(elem.radius) ^ std::hash<coord_t>()(elem.layer_nr * 7919);
    // }
    pub fn hash(&self) -> u64 {
        // C++ casts radius (coordf_t/double) through std::hash<coord_t>(): radius is first
        // converted to coord_t (int32_t per libslic3r.h:40) then hashed (identity for ints).
        // FIDELITY-NOTE(F2): coord_t is int32_t — reproduce the int32 truncation locally.
        ((self.radius as i32) as u64) ^ ((self.layer_nr.wrapping_mul(7919)) as u64)
    }
}

impl RadiusLayerPair {
    /// The part of the key the cache compares (RadiusLayerPairEquality): the radius and the layer.
    fn cache_key(&self) -> (u64, usize) {
        (self.radius.to_bits(), self.layer_nr)
    }
}

/// TreeSupport.hpp:225-379 `class TreeSupportData`: lazily generates tree guidance volumes.
///
/// The caches are `mutable tbb::concurrent_unordered_map`s filled from const methods in C++;
/// here they are `RefCell`s, and a lookup hands back a copy of the cached areas.
#[derive(Debug)]
pub struct TreeSupportData {
    // TreeSupport.hpp:284 std::vector<LayerHeightData> layer_heights;
    pub layer_heights: Vec<LayerHeightData>,
    // TreeSupport.hpp:286 std::vector<std::unique_ptr<SupportNode>> contact_nodes;
    pub contact_nodes: Vec<SupportNode>,
    // TreeSupport.hpp:336 bool is_slim = false;
    pub is_slim: bool,
    // TreeSupport.hpp:340 coordf_t m_xy_distance;
    pub m_xy_distance: CoordF,
    // TreeSupport.hpp:342 double branch_scale_factor = 1.0; // tan(45 degrees)
    pub branch_scale_factor: f64,
    // TreeSupport.hpp:351 coordf_t m_radius_sample_resolution;
    pub m_radius_sample_resolution: CoordF,
    // TreeSupport.hpp:356 std::vector<ExPolygons> m_layer_outlines;
    pub m_layer_outlines: Vec<ExPolygons>,
    // TreeSupport.hpp:359 std::vector<ExPolygons> m_layer_outlines_below; // union contours of all layers below
    pub m_layer_outlines_below: Vec<ExPolygons>,
    // TreeSupport.hpp:361 std::vector<double> m_max_move_distances;
    pub m_max_move_distances: Vec<f64>,
    // TreeSupport.hpp:375-376 the collision and avoidance caches, keyed by (radius, layer_nr).
    pub m_collision_cache: RefCell<HashMap<(u64, usize), Arc<ExPolygons>>>,
    pub m_avoidance_cache: RefCell<HashMap<(u64, usize), Arc<ExPolygons>>>,
}

impl Default for TreeSupportData {
    // TreeSupport.hpp:228 TreeSupportData() = default;
    fn default() -> Self {
        TreeSupportData {
            layer_heights: Vec::new(),
            contact_nodes: Vec::new(),
            is_slim: false,
            m_xy_distance: 0.0,
            branch_scale_factor: 1.0,
            m_radius_sample_resolution: 0.0,
            m_layer_outlines: Vec::new(),
            m_layer_outlines_below: Vec::new(),
            m_max_move_distances: Vec::new(),
            m_collision_cache: RefCell::new(HashMap::new()),
            m_avoidance_cache: RefCell::new(HashMap::new()),
        }
    }
}

// TreeSupport.cpp:70-83 enum TreeSupportStage
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u32)]
pub enum TreeSupportStage {
    StageDetectOverhangs = 0,
    StageGenerateContactNodes,
    StageDropDownNodes,
    StageDrawCircles,
    StageGenerateToolpaths,
    StageMinimumSpanningTree,
    StageGetAvoidance,
    StageProjectionOntoEx,
    StageGetCollision,
    StageIntersectionLn,
    StageTotal,
    NumStages,
}

const NUM_STAGES: usize = TreeSupportStage::NumStages as usize;

/// TreeSupport.cpp:87-150 `class TreeSupportProfiler`. The boost local clock is
/// `std::time::Instant`; the durations are whole milliseconds as `total_milliseconds()`.
#[derive(Debug, Clone)]
pub struct TreeSupportProfiler {
    pub stage_durations: [u32; NUM_STAGES],
    pub stage_index: u32,
    pub tic_time: Option<std::time::Instant>,
    pub toc_time: Option<std::time::Instant>,
    m_stage_start_times: [Option<std::time::Instant>; NUM_STAGES],
}

impl TreeSupportProfiler {
    // TreeSupport.cpp:96-101
    pub const fn new() -> Self {
        TreeSupportProfiler {
            stage_durations: [0; NUM_STAGES],
            stage_index: 0,
            tic_time: None,
            toc_time: None,
            m_stage_start_times: [None; NUM_STAGES],
        }
    }

    // TreeSupport.cpp:103-109
    pub fn stage_start(&mut self, stage: TreeSupportStage) {
        if stage > TreeSupportStage::NumStages {
            return;
        }
        // TreeSupport.cpp:108 (NUM_STAGES itself passes the check and indexes one past the
        // array in C++; it is never passed)
        if let Some(slot) = self.m_stage_start_times.get_mut(stage as usize) {
            *slot = Some(std::time::Instant::now());
        }
    }

    // TreeSupport.cpp:111-118
    pub fn stage_finish(&mut self, stage: TreeSupportStage) {
        if stage > TreeSupportStage::NumStages {
            return;
        }
        let time = std::time::Instant::now();
        let idx = stage as usize;
        if idx < NUM_STAGES {
            let start = self.m_stage_start_times[idx].unwrap_or(time);
            self.stage_durations[idx] = time.duration_since(start).as_millis() as u32;
        }
    }

    // TreeSupport.cpp:120
    pub fn tic(&mut self) {
        self.tic_time = Some(std::time::Instant::now());
    }

    // TreeSupport.cpp:121-124
    pub fn toc(&mut self) -> u32 {
        let now = std::time::Instant::now();
        self.toc_time = Some(now);
        now.duration_since(self.tic_time.unwrap_or(now)).as_millis() as u32
    }

    // TreeSupport.cpp:125-130
    pub fn stage_add(&mut self, stage: TreeSupportStage) {
        if stage > TreeSupportStage::NumStages {
            return;
        }
        let elapsed = self.toc();
        if let Some(d) = self.stage_durations.get_mut(stage as usize) {
            *d = d.wrapping_add(elapsed);
        }
    }

    // TreeSupport.cpp:132-147
    pub fn report(&self) -> String {
        format!(
            "total overhange cost: {}; STAGE_DETECT_OVERHANGS: {}; STAGE_GENERATE_CONTACT_NODES: {}; STAGE_DROP_DOWN_NODES: {}; STAGE_DRAW_CIRCLES: {}; STAGE_GENERATE_TOOLPATHS: {}; STAGE_MinimumSpanningTree: {}; STAGE_GET_AVOIDANCE: {}; STAGE_projection_onto_ex: {}; STAGE_get_collision: {}; STAGE_intersection_ln: {}",
            self.stage_durations[TreeSupportStage::StageTotal as usize],
            self.stage_durations[TreeSupportStage::StageDetectOverhangs as usize],
            self.stage_durations[TreeSupportStage::StageGenerateContactNodes as usize],
            self.stage_durations[TreeSupportStage::StageDropDownNodes as usize],
            self.stage_durations[TreeSupportStage::StageDrawCircles as usize],
            self.stage_durations[TreeSupportStage::StageGenerateToolpaths as usize],
            self.stage_durations[TreeSupportStage::StageMinimumSpanningTree as usize],
            self.stage_durations[TreeSupportStage::StageGetAvoidance as usize],
            self.stage_durations[TreeSupportStage::StageProjectionOntoEx as usize],
            self.stage_durations[TreeSupportStage::StageGetCollision as usize],
            self.stage_durations[TreeSupportStage::StageIntersectionLn as usize],
        )
    }
}

impl Default for TreeSupportProfiler {
    fn default() -> Self {
        Self::new()
    }
}

// TreeSupport.cpp:152 `TreeSupportProfiler profiler;`
static PROFILER: std::sync::Mutex<TreeSupportProfiler> = std::sync::Mutex::new(TreeSupportProfiler::new());

fn profiler() -> std::sync::MutexGuard<'static, TreeSupportProfiler> {
    PROFILER.lock().unwrap_or_else(|e| e.into_inner())
}

// ============================================================================
// TreeSupport.cpp free functions / static helpers
// ============================================================================

// TreeSupport.cpp:59-62
// inline double dot_with_unscale(const Point a, const Point b)
// { return unscale_(a(0)) * unscale_(b(0)) + unscale_(a(1)) * unscale_(b(1)); }
#[inline]
pub fn dot_with_unscale(a: Point, b: Point) -> f64 {
    // The left product is contracted with the sum (clang -ffp-contract=on).
    unscale_(a.x).mul_add(unscale_(b.x), unscale_(a.y) * unscale_(b.y))
}

// TreeSupport.cpp:64-67
// inline double vsize2_with_unscale(const Point pt) { return dot_with_unscale(pt, pt); }
#[inline]
pub fn vsize2_with_unscale(pt: Point) -> f64 {
    dot_with_unscale(pt, pt)
}

// TreeSupport.cpp:69-74
// inline Point normal(Point pt, double scale)
// { double length = scale_(sqrt(vsize2_with_unscale(pt))); return pt * (scale / length); }
#[inline]
pub fn normal(pt: Point, scale_factor: f64) -> Point {
    // double length = scale_(sqrt(vsize2_with_unscale(pt)));
    let length = scale_(vsize2_with_unscale(pt).sqrt());
    // return pt * (scale / length);  `pt` is a non-const Point: the member operator* (lrint).
    point_mul(pt, scale_factor / length)
}

// ============================================================================
// MinimumSpanningTree.hpp:18-74, MinimumSpanningTree.cpp:22-138 `class MinimumSpanningTree`. A local copy:
// `crate::minimum_spanning_tree` takes the squared distance of two vertices as `(x / 1e5)^2 + (y / 1e5)^2`
// without the contraction, where MinimumSpanningTree.cpp:12-20 has the one expression
// `unscale_(a(0)) * unscale_(b(0)) + unscale_(a(1)) * unscale_(b(1))` (the text of TreeSupport.cpp:56-64,
// `vsize2_with_unscale` below). The nodes of an overhang sit on a regular grid, whose equal distances then
// compare differently and connect to other neighbours.
// ============================================================================

/// MinimumSpanningTree.hpp:20-36 `struct Edge`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MinimumSpanningTreeEdge {
    start: Point,
    end: Point,
}

/// MinimumSpanningTree.hpp:18-74 `class MinimumSpanningTree`
///
/// `adjacency_graph` is a `std::unordered_map<Point, std::vector<Edge>, PointHash>`; `m_keys` keeps its
/// keys in the order they were inserted, which `leaves()` and `vertices()` walk (the C++ walks the hash
/// table, an order nothing but the debug drawings of TreeSupport.cpp depends on).
#[derive(Debug, Clone, Default)]
pub struct MinimumSpanningTree {
    adjacency_graph: HashMap<Point, Vec<MinimumSpanningTreeEdge>>,
    m_keys: Vec<Point>,
}

impl MinimumSpanningTree {
    /// MinimumSpanningTree.hpp:38 `MinimumSpanningTree() = default;`
    pub fn new() -> Self {
        Self::default()
    }

    /// MinimumSpanningTree.cpp:22-25 `MinimumSpanningTree::MinimumSpanningTree(std::vector<Point> vertices)`
    pub fn from_vertices(vertices: Vec<Point>) -> Self {
        let mut this = Self::default();
        this.prim(vertices);
        this
    }

    /// `result[point]` of prim(): the edges of the point, added when missing.
    fn edges_of(&mut self, point: Point) -> &mut Vec<MinimumSpanningTreeEdge> {
        if !self.adjacency_graph.contains_key(&point) {
            self.m_keys.push(point);
        }
        self.adjacency_graph.entry(point).or_default()
    }

    /// MinimumSpanningTree.cpp:27-103 `auto MinimumSpanningTree::prim(std::vector<Point> vertices) const -> AdjacencyGraph_t`
    ///
    /// The maps of the C++ are keyed by the address of a vertex in `vertices_list`: here by its index.
    /// `std::min_element` walks the hash table with a comparator that orders by distance, x and y, so
    /// its result does not depend on the order of the walk unless two vertices are the same point.
    fn prim(&mut self, vertices: Vec<Point>) {
        if vertices.is_empty() {
            return; //No vertices, so we can't create edges either.
        }
        // If there's only one vertex, we can't go creating any edges so just add the point to the adjacency list with no
        // edges
        if vertices.len() == 1 {
            self.edges_of(vertices[0]);
            return;
        }
        let vertices_list: Vec<Point> = vertices;

        //The shortest distance to the current tree.
        let mut smallest_distance: Vec<Option<CoordF>> = vec![None; vertices_list.len()];
        //Which point the shortest distance goes towards.
        let mut smallest_distance_to: Vec<usize> = vec![0; vertices_list.len()];
        for vertex_index in 1..vertices_list.len() {
            let vert = vertices_list[vertex_index];
            smallest_distance[vertex_index] = Some(vsize2_with_unscale(Point::new(vert.x - vertices_list[0].x, vert.y - vertices_list[0].y)));
            smallest_distance_to[vertex_index] = 0;
        }

        //All of the vertices need to be in the tree at the end.
        while self.adjacency_graph.len() < vertices_list.len() {
            //Choose the closest vertex to connect to that is not yet in the tree.
            let mut closest: Option<usize> = None;
            for (index, distance) in smallest_distance.iter().enumerate() {
                let Some(distance) = *distance else { continue };
                let less = match closest {
                    None => true,
                    Some(best) => {
                        let best_distance = smallest_distance[best].unwrap();
                        let (a, b) = (vertices_list[index], vertices_list[best]);
                        if distance != best_distance {
                            distance < best_distance
                        } else if a.x != b.x {
                            a.x < b.x
                        } else {
                            a.y < b.y
                        }
                    }
                };
                if less {
                    closest = Some(index);
                }
            }
            // `std::min_element` of an empty map is end(), which the C++ dereferences: the vertices were not
            // distinct points.
            let Some(closest) = closest else { break };

            //Add this point to the graph and remove it from the candidates.
            let closest_point = vertices_list[closest];
            let other_end = vertices_list[smallest_distance_to[closest]];
            self.edges_of(closest_point).push(MinimumSpanningTreeEdge { start: closest_point, end: other_end });
            self.edges_of(other_end).push(MinimumSpanningTreeEdge { start: other_end, end: closest_point });
            smallest_distance[closest] = None; //Remove it so we don't check for these points again.

            //Update the distances of all points that are not in the graph.
            for index in 0..vertices_list.len() {
                let Some(old_distance) = smallest_distance[index] else { continue };
                let p = vertices_list[index];
                let new_distance: CoordF = vsize2_with_unscale(Point::new(closest_point.x - p.x, closest_point.y - p.y));
                if new_distance < old_distance {
                    //New point is closer.
                    smallest_distance[index] = Some(new_distance);
                    smallest_distance_to[index] = closest;
                }
            }
        }
    }

    /// MinimumSpanningTree.cpp:105-116 `std::vector<Point> MinimumSpanningTree::adjacent_nodes(Point node) const`
    pub fn adjacent_nodes(&self, node: Point) -> Vec<Point> {
        let mut result: Vec<Point> = Vec::new();
        if let Some(edges) = self.adjacency_graph.get(&node) {
            for e in edges {
                result.push(if e.start == node { e.end } else { e.start });
            }
        }
        result
    }

    /// MinimumSpanningTree.cpp:118-129 `std::vector<Point> MinimumSpanningTree::leaves() const`
    pub fn leaves(&self) -> Vec<Point> {
        let mut result: Vec<Point> = Vec::new();
        for node in &self.m_keys {
            if self.adjacency_graph[node].len() <= 1 {
                result.push(*node);
            }
        }
        result
    }

    /// MinimumSpanningTree.cpp:131-138 `std::vector<Point> MinimumSpanningTree::vertices() const`
    pub fn vertices(&self) -> Vec<Point> {
        self.m_keys.clone()
    }
}

// TreeSupport.cpp:157-183
// Lines spanning_tree_to_lines(const std::vector<MinimumSpanningTree>& spanning_trees)
pub fn spanning_tree_to_lines(spanning_trees: &[MinimumSpanningTree]) -> Lines {
    // Lines polylines;
    let mut polylines: Lines = Vec::new();
    // for (const MinimumSpanningTree& mst : spanning_trees) {
    for mst in spanning_trees {
        // std::vector<Point> points = mst.vertices();
        let points = mst.vertices();
        // std::unordered_set<Point, PointHash> to_ignore;
        let mut to_ignore: HashSet<Point> = HashSet::new();
        // for (Point pt1 : points) {
        for pt1 in &points {
            // if (to_ignore.find(pt1) != to_ignore.end()) continue;
            if to_ignore.contains(pt1) {
                continue;
            }

            // const std::vector<Point>& neighbours = mst.adjacent_nodes(pt1);
            let neighbours = mst.adjacent_nodes(*pt1);
            // if (neighbours.empty()) continue;
            if neighbours.is_empty() {
                continue;
            }

            // for (Point pt2 : neighbours) {
            for pt2 in &neighbours {
                // if (to_ignore.find(pt2) != to_ignore.end()) continue;
                if to_ignore.contains(pt2) {
                    continue;
                }

                // Line line(pt1, pt2);
                // polylines.push_back(line);
                polylines.push(Line::new(*pt1, *pt2));
            }

            // to_ignore.insert(pt1);
            to_ignore.insert(*pt1);
        }
    }
    // return polylines;
    polylines
}

// TreeSupport.cpp:274-399
// Move point from inside polygon if distance>0, outside if distance<0.
// Special case: distance=0 means find the nearest point of from on the polygon contour.
// The max move distance should not excceed max_move_distance.
// @return success(true) or not(false)
pub fn move_inside_expoly(
    polygon: &ExPolygon,
    from: &mut Point,
    distance: f64,
    max_move_distance: f64,
) -> bool {
    //TODO: This is copied from the moveInside of Polygons.
    // Point ret = from;
    let mut ret = *from;
    // double bestDist2 = std::numeric_limits<double>::max();
    let mut best_dist2 = f64::MAX;
    // bool is_already_on_correct_side_of_boundary = false;
    let mut is_already_on_correct_side_of_boundary = false;
    // const Polygon &contour = polygon.contour;
    let contour = &polygon.contour;

    // if (contour.points.size() < 2) return false;
    if contour.points.len() < 2 {
        return false;
    }
    // Point p0 = contour.points[polygon.contour.size() - 2];
    let mut p0 = contour.points[polygon.contour.points.len() - 2];
    // Point p1 = contour.points.back();
    let mut p1 = *contour.points.last().unwrap();
    // bool projected_p_beyond_prev_segment = dot_with_unscale(p1 - p0, from - p0) >= vsize2_with_unscale(p1 - p0);
    let mut projected_p_beyond_prev_segment =
        dot_with_unscale(p1 - p0, *from - p0) >= vsize2_with_unscale(p1 - p0);
    // for(const Point& p2 : polygon.contour.points)
    for &p2 in &polygon.contour.points {
        // X = P projected on AB
        // const Point& a = p1; const Point& b = p2; const Point& p = from;
        let a = p1;
        let b = p2;
        let p = *from;
        // Point ab = b - a; Point ap = p - a;
        let ab = b - a;
        let ap = p - a;
        // double ab_length2 = vsize2_with_unscale(ab);
        let ab_length2 = vsize2_with_unscale(ab);
        // if(ab_length2 <= 0) { p1 = p2; continue; }
        if ab_length2 <= 0.0 {
            p1 = p2;
            continue;
        }
        // double dot_prod = dot_with_unscale(ab, ap);
        let dot_prod = dot_with_unscale(ab, ap);
        if dot_prod <= 0.0 {
            // x is projected to before ab
            if projected_p_beyond_prev_segment {
                //  case which looks like:   > .
                projected_p_beyond_prev_segment = false;
                // Point& x = p1;
                let x = p1;

                // double dist2 = vsize2_with_unscale(x - p);
                let dist2 = vsize2_with_unscale(x - p);
                if dist2 < best_dist2 {
                    best_dist2 = dist2;
                    if distance == 0.0 {
                        ret = x;
                    } else {
                        // Point inward_dir = turn90_ccw(normal(ab, 10.0) + normal(p1 - p0, 10.0));
                        let inward_dir = turn90_ccw(normal(ab, 10.0) + normal(p1 - p0, 10.0));
                        // ret = x + normal(inward_dir, scale_(distance));
                        ret = x + normal(inward_dir, scale_(distance) as f64);
                        // is_already_on_correct_side_of_boundary = dot_with_unscale(inward_dir, p - x) * distance >= 0;
                        is_already_on_correct_side_of_boundary =
                            dot_with_unscale(inward_dir, p - x) * distance >= 0.0;
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
            // x is projected to a point properly on the line segment. The case which looks like | .
            projected_p_beyond_prev_segment = false;
            // Point x = a + ab * (dot_prod / ab_length2);
            let x = a + point_mul(ab, dot_prod / ab_length2);

            // double dist2 = vsize2_with_unscale(p - x);
            let dist2 = vsize2_with_unscale(p - x);
            if dist2 < best_dist2 {
                best_dist2 = dist2;
                if distance == 0.0 {
                    ret = x;
                } else {
                    // Point inward_dir = turn90_ccw(normal(ab, scale_(distance)));
                    let inward_dir = turn90_ccw(normal(ab, scale_(distance) as f64));
                    // ret = x + inward_dir;
                    ret = x + inward_dir;
                    // is_already_on_correct_side_of_boundary = dot_with_unscale(inward_dir, p - x) >= 0;
                    is_already_on_correct_side_of_boundary =
                        dot_with_unscale(inward_dir, p - x) >= 0.0;
                }
            }
        }

        p0 = p1;
        p1 = p2;
    }

    if is_already_on_correct_side_of_boundary {
        // BBS. Remove this condition.
        if best_dist2 < distance * distance {
            *from = ret;
        }
        true
    } else if best_dist2 < max_move_distance * max_move_distance {
        *from = ret;
        true
    } else {
        false
    }
}

// TreeSupport.cpp:401-527
// Implementation assumes moving inside, but moving outside should just as well be possible.
pub fn move_inside_expolys(
    polygons: &ExPolygons,
    from: &mut Point,
    distance: f64,
    max_move_distance: f64,
) -> bool {
    // Point from0 = from;
    let from0 = *from;
    // Point ret = from;
    let mut ret = *from;
    // std::vector<Point> valid_pts;
    let mut valid_pts: Vec<Point> = Vec::new();
    // double bestDist2 = std::numeric_limits<double>::max();
    let mut best_dist2 = f64::MAX;
    // unsigned int bestPoly = NO_INDEX;
    let mut best_poly: u32 = NO_INDEX;
    // bool is_already_on_correct_side_of_boundary = false;
    let mut is_already_on_correct_side_of_boundary = false;
    // Point inward_dir;
    let mut inward_dir;
    // for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    for poly_idx in 0..polygons.len() {
        // const ExPolygon poly = polygons[poly_idx];
        let poly = &polygons[poly_idx];
        // if (poly.contour.size() < 2) continue;
        if poly.contour.points.len() < 2 {
            continue;
        }
        // Point p0 = poly.contour[poly.contour.size()-2];
        let mut p0 = poly.contour.points[poly.contour.points.len() - 2];
        // Point p1 = poly.contour.points.back();
        let mut p1 = *poly.contour.points.last().unwrap();
        // bool projected_p_beyond_prev_segment = dot_with_unscale(p1 - p0, from - p0) >= vsize2_with_unscale(p1 - p0);
        let mut projected_p_beyond_prev_segment =
            dot_with_unscale(p1 - p0, *from - p0) >= vsize2_with_unscale(p1 - p0);
        // for(const Point p2 : poly.contour.points)
        for &p2 in &poly.contour.points {
            // Point a = p1; Point b = p2; Point p = from;
            let a = p1;
            let b = p2;
            let p = *from;
            // Point ab = b - a; Point ap = p - a;
            let ab = b - a;
            let ap = p - a;
            // double ab_length2 = vsize2_with_unscale(ab);
            let ab_length2 = vsize2_with_unscale(ab);
            // if(ab_length2 <= 0) { p1 = p2; continue; }
            if ab_length2 <= 0.0 {
                p1 = p2;
                continue;
            }
            // double dot_prod = dot_with_unscale(ab, ap);
            let dot_prod = dot_with_unscale(ab, ap);
            if dot_prod <= 0.0 {
                // x is projected to before ab
                if projected_p_beyond_prev_segment {
                    //  case which looks like:   > .
                    projected_p_beyond_prev_segment = false;
                    // Point& x = p1;
                    let x = p1;

                    // double dist2 = vsize2_with_unscale(x - p);
                    let dist2 = vsize2_with_unscale(x - p);
                    if dist2 < best_dist2 {
                        best_dist2 = dist2;
                        best_poly = poly_idx as u32;
                        if distance == 0.0 {
                            ret = x;
                        } else {
                            // inward_dir = turn90_ccw(normal(ab, 10.0) + normal(p1 - p0, 10.0));
                            inward_dir = turn90_ccw(normal(ab, 10.0) + normal(p1 - p0, 10.0));
                            // ret = x + normal(inward_dir, scale_(distance));
                            ret = x + normal(inward_dir, scale_(distance) as f64);
                            // is_already_on_correct_side_of_boundary = dot_with_unscale(inward_dir, p - x) * distance >= 0;
                            is_already_on_correct_side_of_boundary =
                                dot_with_unscale(inward_dir, p - x) * distance >= 0.0;
                            // if (is_already_on_correct_side_of_boundary && dist2 < distance * distance)
                            //     valid_pts.push_back(ret-from0);
                            if is_already_on_correct_side_of_boundary && dist2 < distance * distance {
                                valid_pts.push(ret - from0);
                            }
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
                // x is projected to a point properly on the line segment. The case which looks like | .
                projected_p_beyond_prev_segment = false;
                // Point x = a + ab * (dot_prod / ab_length2);
                let x = a + point_mul(ab, dot_prod / ab_length2);

                // double dist2 = vsize2_with_unscale(p - x);
                let dist2 = vsize2_with_unscale(p - x);
                if dist2 < best_dist2 {
                    best_dist2 = dist2;
                    best_poly = poly_idx as u32;
                    if distance == 0.0 {
                        ret = x;
                    } else {
                        // inward_dir = turn90_ccw(normal(ab, scale_(distance)));
                        inward_dir = turn90_ccw(normal(ab, scale_(distance) as f64));
                        // ret = x + inward_dir;
                        ret = x + inward_dir;
                        // is_already_on_correct_side_of_boundary = dot_with_unscale(inward_dir, p - x) >= 0;
                        is_already_on_correct_side_of_boundary =
                            dot_with_unscale(inward_dir, p - x) >= 0.0;
                        // if (is_already_on_correct_side_of_boundary && dist2<distance*distance)
                        //     valid_pts.push_back(ret-from0);
                        if is_already_on_correct_side_of_boundary && dist2 < distance * distance {
                            valid_pts.push(ret - from0);
                        }
                    }
                }
            }
            p0 = p1;
            p1 = p2;
        }
    }

    // (commented-out valid_pts combine block left out, matching C++ #if 0)
    let _ = (&valid_pts, best_poly);

    if is_already_on_correct_side_of_boundary {
        if best_dist2 < distance * distance {
            *from = ret;
        }
        true
    } else if best_dist2 < max_move_distance * max_move_distance {
        *from = ret;
        true
    } else {
        false
    }
}

// MultiPoint.hpp:53-68: `closest_point_index` / `closest_point` return the closest
// *vertex* of the contour (by Euclidean norm), NOT the closest point projected onto an
// edge. The crate's `Polygon::closest_point` does edge projection, so we reproduce the
// vertex semantics locally for fidelity with C++ `MultiPoint::closest_point`.
fn polygon_closest_vertex(poly: &Polygon, point: &Point) -> Option<Point> {
    if poly.points.is_empty() {
        return None;
    }
    let mut idx = 0usize;
    // double dist_min = (point - points.front()).cast<double>().norm();
    let mut dist_min = point.distance(&poly.points[0]);
    for i in 1..poly.points.len() {
        // double d = (points[i] - point).cast<double>().norm();
        let d = point.distance(&poly.points[i]);
        if d < dist_min {
            dist_min = d;
            idx = i;
        }
    }
    Some(poly.points[idx])
}

// TreeSupport.cpp:529-546
// static Point find_closest_ex(Point from, const ExPolygons& polygons)
pub fn find_closest_ex(from: Point, polygons: &ExPolygons) -> Point {
    // Point closest_pt;
    let mut closest_pt = Point::new(0, 0);
    // double min_dist2 = std::numeric_limits<double>::max();
    let mut min_dist2 = f64::MAX;

    // for (const ExPolygon &poly : polygons) {
    for poly in polygons {
        // for (int i = 0; i < poly.num_contours(); i++) {
        for i in 0..poly.num_contours() {
            // const Point* candidate = poly.contour_or_hole(i).closest_point(from);
            let candidate = match polygon_closest_vertex(contour_or_hole(poly, i), &from) {
                Some(c) => c,
                None => continue,
            };
            // double dist2 = vsize2_with_unscale(*candidate - from);
            let dist2 = vsize2_with_unscale(candidate - from);
            if dist2 < min_dist2 {
                closest_pt = candidate;
                min_dist2 = dist2;
            }
        }
    }

    closest_pt
}

// TreeSupport.cpp:548-551
// static bool move_outside_expolys(const ExPolygons& polygons, Point& from, double distance, double max_move_distance)
// { return move_inside_expolys(polygons, from, -distance, -max_move_distance); }
pub fn move_outside_expolys(
    polygons: &ExPolygons,
    from: &mut Point,
    distance: f64,
    max_move_distance: f64,
) -> bool {
    move_inside_expolys(polygons, from, -distance, -max_move_distance)
}

// TreeSupport.cpp:553-559
// static bool is_inside_ex(const ExPolygon &polygon, const Point &pt)
pub fn is_inside_ex_poly(polygon: &ExPolygon, pt: &Point) -> bool {
    // if (!get_extents(polygon).contains(pt)) return false;
    if !get_extents_expoly(polygon).contains_point(pt) {
        return false;
    }
    // return polygon.contains(pt);  ExPolygon.cpp:109-119, border_result = true: a point on the
    // boundary of a hole is inside.
    polygon.contains(pt, true)
}

// TreeSupport.cpp:561-569
// static bool is_inside_ex(const ExPolygons &polygons, const Point &pt)
pub fn is_inside_ex(polygons: &ExPolygons, pt: &Point) -> bool {
    // for (const ExPolygon &poly : polygons) { if (is_inside_ex(poly, pt)) return true; }
    for poly in polygons {
        if is_inside_ex_poly(poly, pt) {
            return true;
        }
    }
    false
}

// TreeSupport.cpp:571-600
// use project_onto which is more accurate but more expensive
// static bool move_out_expolys(const ExPolygons& polygons, Point& from, double distance, double max_move_distance)
pub fn move_out_expolys(
    polygons: &ExPolygons,
    from: &mut Point,
    distance: f64,
    max_move_distance: f64,
) -> bool {
    // Point from0 = from;
    let _from0 = *from;
    // ExPolygons polys_dilated = union_ex(offset_ex(polygons, scale_(distance)));
    let polys_dilated = union_ex(&offset_ex(polygons, scale_(distance) as f32));
    // Point pt = projection_onto(polys_dilated, from);
    let mut pt = projection_onto(&polys_dilated, from);
    // Point outward_dir = pt - from;
    let outward_dir = pt - *from;
    // Point pt_max = from + normal(outward_dir, scale_(max_move_distance));
    let pt_max = *from + normal(outward_dir, scale_(max_move_distance) as f64);
    // double dist2 = vsize2_with_unscale(outward_dir);
    let dist2 = vsize2_with_unscale(outward_dir);
    // if (dist2 > SQ(max_move_distance)) pt = pt_max;
    if dist2 > sq(max_move_distance) {
        pt = pt_max;
    }
    // case 5: already outside and far enough, no need to move
    // if (!is_inside_ex(polys_dilated, from)) return true;
    if !is_inside_ex(&polys_dilated, from) {
        true
    }
    // else if (!is_inside_ex(polygons, from)) { from = pt; return true; }
    else if !is_inside_ex(polygons, from) {
        // case 4: already outside but not far enough
        *from = pt;
        true
    } else {
        // bool pt_max_in_poly = is_inside_ex(polygons, pt_max);
        let pt_max_in_poly = is_inside_ex(polygons, &pt_max);
        if !pt_max_in_poly {
            *from = pt_max;
            true
        } else {
            false
        }
    }
}

// TreeSupport.cpp:602-605
// static Point bounding_box_middle(const BoundingBox &bbox) { return (bbox.max + bbox.min) / 2; }
pub fn bounding_box_middle(bbox: &BoundingBox) -> Point {
    (bbox.max + bbox.min) / 2
}

// TreeSupport.cpp:650-658
// bool is_stable(float height, const ExPolygon &overhang, float strength_z)
pub fn is_stable(height: f32, overhang: &ExPolygon, _strength_z: f32) -> bool {
    // float stability = 1. * height;  // computed in double, stored as float
    let stability: f32 = (1.0_f64 * height as f64) as f32;
    // double Ixx = 0, Iyy = 0;
    let mut ixx = 0.0_f64;
    let mut iyy = 0.0_f64;
    // auto props = compSecondMoment({overhang}, Ixx, Iyy);  // Brim.cpp
    let _props =
        crate::brim::comp_second_moment_expolygons(&vec![overhang.clone()], &mut ixx, &mut iyy);
    // double moment = std::min(Ixx * pow(SCALING_FACTOR, 4), Iyy * pow(SCALING_FACTOR, 4));
    let moment = (ixx * SCALING_FACTOR.powf(4.0)).min(iyy * SCALING_FACTOR.powf(4.0));
    // return moment / stability > 3.;  (stability promoted float->double for the division)
    moment / stability as f64 > 3.0
}

// TreeSupport.cpp:2081-2099
// coordf_t TreeSupport::calc_branch_radius(base_radius, layers_to_top, tip_layers, diameter_angle_scale_factor)
//
// Ported as a free function. `is_slim` is the TreeSupport member flag (TreeSupport.hpp:457).
pub fn calc_branch_radius_by_layers(
    base_radius: CoordF,
    layers_to_top: usize,
    tip_layers: usize,
    diameter_angle_scale_factor: f64,
    is_slim: bool,
) -> CoordF {
    // double radius;
    let radius;
    if !is_slim {
        if (layers_to_top + 1) > tip_layers {
            // radius = base_radius + base_radius * (layers_to_top + 1) * diameter_angle_scale_factor;
            radius = base_radius
                + base_radius * (layers_to_top + 1) as f64 * diameter_angle_scale_factor;
        } else {
            // radius = base_radius * (layers_to_top + 1) / tip_layers;
            radius = base_radius * (layers_to_top + 1) as f64 / tip_layers as f64;
        }
    } else if (layers_to_top + 1) > tip_layers * 2 {
        // radius = base_radius + base_radius * (layers_to_top + 1) * diameter_angle_scale_factor;
        radius =
            base_radius + base_radius * (layers_to_top + 1) as f64 * diameter_angle_scale_factor;
    } else {
        // radius = base_radius * (layers_to_top + 1) / (tip_layers * 2);
        radius = base_radius * (layers_to_top + 1) as f64 / (tip_layers * 2) as f64;
    }
    // radius = std::clamp(radius, MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS);
    radius.clamp(MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS)
}

// TreeSupport.cpp:2101-2122
// coordf_t TreeSupport::calc_branch_radius(base_radius, mm_to_top, diameter_angle_scale_factor, use_min_distance)
//
// Ported as a free function. `support_interface_top_layers` is the object config
// value (TreeSupport.cpp:2115). The `m_model_volumes->ceilRadius` block is guarded
// by `#if USE_SUPPORT_3D` which is 0, so it is not executed.
pub fn calc_branch_radius_by_mm(
    base_radius: CoordF,
    mm_to_top: CoordF,
    diameter_angle_scale_factor: f64,
    _use_min_distance: bool,
    support_interface_top_layers: i32,
) -> CoordF {
    // double radius;
    let mut radius;
    // coordf_t tip_height = base_radius; // this is a 45 degree tip
    let tip_height = base_radius;
    if mm_to_top > tip_height {
        // radius = base_radius + (mm_to_top-tip_height) * diameter_angle_scale_factor;
        radius = base_radius + (mm_to_top - tip_height) * diameter_angle_scale_factor;
    } else {
        // radius = mm_to_top; // this is a 45 degree tip
        radius = mm_to_top;
    }
    // radius = std::clamp(radius, MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS);
    radius = radius.clamp(MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS);
    // if (m_object_config->support_interface_top_layers.value > 0) radius = std::max(radius, base_radius);
    if support_interface_top_layers > 0 {
        radius = radius.max(base_radius);
    }
    // #if USE_SUPPORT_3D ... ceilRadius ... #endif  (USE_SUPPORT_3D == 0, skipped)
    radius
}

// TreeSupport.cpp:2189-2206
// ExPolygons avoid_object_remove_extra_small_parts(const ExPolygon &expoly, const ExPolygons& avoid_region)
pub fn avoid_object_remove_extra_small_parts(
    expoly: &ExPolygon,
    avoid_region: &ExPolygons,
) -> ExPolygons {
    // ExPolygons expolys_out;
    let mut expolys_out: ExPolygons = Vec::new();
    // if(expoly.empty()) return expolys_out;
    if expoly.contour.points.is_empty() {
        return expolys_out;
    }
    // auto clipped_avoid_region = ClipperUtils::clip_clipper_polygons_with_subject_bbox(avoid_region, get_extents(expoly));
    // ClipperUtils.cpp:161-172: the ExPolygons overload flattens each ExPolygon's
    // contour+holes into a single Polygons list (mixed CW/CCW windings), which the
    // subsequent diff_ex interprets via Clipper's fill rule. We must reproduce that
    // flat Polygons clip, not wrap each polygon as an independent positive contour.
    let subject_bbox = get_extents_expoly(expoly);
    let clipped_avoid_region: Vec<Polygon> =
        crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(
            avoid_region,
            &subject_bbox,
            false,
        );
    // auto expolys_avoid = diff_ex(expoly, clipped_avoid_region);
    let expolys_avoid = diff_ex_polygons(std::slice::from_ref(expoly), &clipped_avoid_region);
    // int idx_max_area = -1;
    let mut idx_max_area: i32 = -1;
    // float max_area = 0;
    let mut max_area: f32 = 0.0;
    // for (int i = 0; i < expolys_avoid.size(); ++i)
    for (i, ea) in expolys_avoid.iter().enumerate() {
        // auto a = expolys_avoid[i].area();  // ExPolygon::area() returns double
        let a: f64 = ea.area();
        // if (a > max_area)  -- max_area (float) promoted to double for the comparison
        if a > max_area as f64 {
            // max_area = a;  -- narrowed double->float on assignment
            max_area = a as f32;
            idx_max_area = i as i32;
        }
    }
    // if (idx_max_area >= 0) expolys_out.emplace_back(std::move(expolys_avoid[idx_max_area]));
    if idx_max_area >= 0 {
        expolys_out.push(expolys_avoid[idx_max_area as usize].clone());
    }

    expolys_out
}

// ============================================================================
// Configuration access. `PrintObjectConfig` / `PrintConfig` are the dynamic
// configurations of `crate::print::faithful`; `config.<key>.value` is read by key.
// ============================================================================

// `config.<key>.value` of a float option.
fn cfg_float(config: &DynamicPrintConfig, key: &str) -> f64 {
    config.option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
}

// `config.<key>.value` of an int or enum option.
fn cfg_int(config: &DynamicPrintConfig, key: &str) -> i32 {
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
}

// `config.<key>.value` of a bool option.
fn cfg_bool(config: &DynamicPrintConfig, key: &str) -> bool {
    config.option(key).and_then(|o| o.get_bool().ok()).unwrap_or(false)
}

// `config.<key>` of a ConfigOptionFloat, for the callees that take the option.
fn cfg_float_option(config: &DynamicPrintConfig, key: &str) -> ConfigOptionFloat {
    ConfigOptionFloat::with_value(cfg_float(config, key))
}

// Point.hpp `Point operator/(const Point&, coord_t)` of Eigen: integer division of both coordinates.
#[inline]
fn point_div(pt: Point, d: Coord) -> Point {
    Point::new(pt.x / d, pt.y / d)
}

// Eigen `(a - b).cast<double>().norm()`.
#[inline]
fn point_dist(a: Point, b: Point) -> f64 {
    point_norm(Point::new(a.x - b.x, a.y - b.y))
}

// ============================================================================
// TreeSupportData (TreeSupport.cpp:6707-6877)
// ============================================================================

impl TreeSupportData {
    /// TreeSupport.cpp:6707-6763
    /// `TreeSupportData::TreeSupportData(const PrintObject &object, coordf_t xy_distance, coordf_t radius_sample_resolution)`
    ///
    /// FIDELITY-NOTE: `get_bed_shape_with_excluded_area(object.print()->config())` (PrintConfig.cpp:11529)
    /// has no port in the crate; the caller hands over its result as `bed_shape_with_excluded_area`.
    /// `LayerRegion::adaptive_merge_patches` is not a member of the crate's LayerRegion, so the merged band
    /// footprints of lines 6741-6749 (multi-diameter adaptive layers) are not added to the outlines.
    pub fn new(
        object: &PrintObject,
        print: &Print,
        bed_shape_with_excluded_area: &Polygon,
        xy_distance: CoordF,
        radius_sample_resolution: CoordF,
    ) -> Self {
        let mut this = TreeSupportData { m_xy_distance: xy_distance, m_radius_sample_resolution: radius_sample_resolution, ..TreeSupportData::default() };
        // 6710 branch_scale_factor = tan(object.config().tree_support_branch_angle.value * M_PI / 180.);
        this.branch_scale_factor = (cfg_float(object.config(), "tree_support_branch_angle") * M_PI / 180.0).tan();
        this.clear_nodes();
        let n_layers = object.layers().len();
        this.m_max_move_distances.resize(n_layers, 0.0);
        this.m_layer_outlines.resize(n_layers, Vec::new());
        this.m_layer_outlines_below.resize(object.layer_count(), Vec::new());
        let mut machine_border: ExPolygons = Vec::new();
        let mut m_machine_border = ExPolygon::default();

        // 6718-6722 cal m_machine_border twice, this may happen before TreeSupport builds
        m_machine_border.contour = bed_shape_with_excluded_area.clone();
        let plate_offset = print.get_plate_origin();
        // align with the centered object in current plate (may not be the 1st plate, so need to add the plate offset)
        let shift = object.instances()[0].shift;
        let offset_pt = point_from_f64(scale_(plate_offset[0]), scale_(plate_offset[1]));
        m_machine_border.translate(Point::new(offset_pt.x - shift.x, offset_pt.y - shift.y));

        // 6724-6729
        if !m_machine_border.contour.points.is_empty() {
            let mut hole = m_machine_border.contour.clone();
            hole.reverse();
            let machine_outline =
                ExPolygon::with_holes(offset_polygon(&m_machine_border.contour, scale_(1000.0) as f32)[0].clone(), vec![hole]);
            machine_border = machine_outline.split_expoly_with_holes(scale_(1.0) as Coord, &vec![m_machine_border.clone()]);
        }

        // 6731-6762
        for layer_nr in 0..n_layers {
            let layer = object.get_layer(layer_nr as i32);
            // m_max_move_distances[layer_nr] = layer->height * branch_scale_factor;
            this.m_max_move_distances[layer_nr] = layer.height * this.branch_scale_factor;
            let mut outline: ExPolygons = Vec::with_capacity(layer.lslices.len());
            for poly in &layer.lslices {
                append(&mut outline, crate::clipper2_utils::union_ex_2(&expolygon_simplify_p(poly, scale_(this.m_radius_sample_resolution))));
            }
            append(&mut outline, machine_border.clone());

            if layer_nr == 0 {
                this.m_layer_outlines_below[layer_nr] = outline.clone();
            } else {
                let mut below = this.m_layer_outlines_below[layer_nr - 1].clone();
                below.extend(outline.iter().cloned());
                if layer_nr % 10 == 0 {
                    below = crate::clipper2_utils::union_ex_2_expolygons(&below);
                }
                this.m_layer_outlines_below[layer_nr] = below;
            }
            this.m_layer_outlines[layer_nr] = outline;
        }
        this
    }

    /// TreeSupport.cpp:6765-6774 `const ExPolygons& TreeSupportData::get_collision(coordf_t radius, size_t layer_nr) const`
    pub fn get_collision(&self, radius: CoordF, layer_nr: usize) -> Arc<ExPolygons> {
        profiler().tic();
        let radius = self.ceil_radius(radius);
        let key = (radius.to_bits(), layer_nr);
        let found = self.m_collision_cache.borrow().get(&key).cloned();
        let collision = match found {
            Some(c) => c,
            None => self.calculate_collision(radius, layer_nr),
        };
        profiler().stage_add(TreeSupportStage::StageGetCollision);
        collision
    }

    /// TreeSupport.cpp:6776-6786 `const ExPolygons& TreeSupportData::get_avoidance(coordf_t radius, size_t layer_nr, int recursions) const`
    pub fn get_avoidance(&self, radius: CoordF, layer_nr: usize, _recursions: i32) -> Arc<ExPolygons> {
        profiler().tic();
        let radius = self.ceil_radius(radius);
        let key = (radius.to_bits(), layer_nr);
        let found = self.m_avoidance_cache.borrow().get(&key).cloned();
        let avoidance = match found {
            Some(a) => a,
            None => self.calculate_avoidance(radius, layer_nr),
        };
        profiler().stage_add(TreeSupportStage::StageGetAvoidance);
        avoidance
    }

    /// TreeSupport.cpp:6788-6796 `Polygons TreeSupportData::get_contours(size_t layer_nr) const`
    pub fn get_contours(&self, layer_nr: usize) -> Polygons {
        let mut contours: Polygons = Vec::new();
        for expoly in &self.m_layer_outlines[layer_nr] {
            contours.push(expoly.contour.clone());
        }
        contours
    }

    /// TreeSupport.cpp:6798-6806 `Polygons TreeSupportData::get_contours_with_holes(size_t layer_nr) const`
    pub fn get_contours_with_holes(&self, layer_nr: usize) -> Polygons {
        let mut contours: Polygons = Vec::new();
        for expoly in &self.m_layer_outlines[layer_nr] {
            for i in 0..expoly.num_contours() {
                contours.push(contour_or_hole(expoly, i).clone());
            }
        }
        contours
    }

    /// TreeSupport.cpp:6808-6819 `SupportNode* TreeSupportData::create_node(...)`
    #[allow(clippy::too_many_arguments)]
    pub fn create_node(
        &mut self,
        position: Point,
        distance_to_top: i32,
        obj_layer_nr: i32,
        support_roof_layers_below: i32,
        to_buildplate: bool,
        parent: Option<NodeId>,
        print_z_: CoordF,
        height_: CoordF,
        dist_mm_to_top_: CoordF,
        radius_: CoordF,
    ) -> NodeId {
        // this function may be called from multiple threads, need to lock (the arena is owned here)
        let raw_ptr = self.contact_nodes.len();
        let node = SupportNode::with_parent(
            &mut self.contact_nodes,
            raw_ptr,
            position,
            distance_to_top,
            obj_layer_nr,
            support_roof_layers_below,
            to_buildplate,
            parent,
            print_z_,
            height_,
            dist_mm_to_top_,
            radius_,
        );
        self.contact_nodes.push(node);
        // if (parent) raw_ptr->movement = position - parent->position;
        if let Some(parent) = parent {
            let pp = self.contact_nodes[parent].position;
            self.contact_nodes[raw_ptr].movement = Point::new(position.x - pp.x, position.y - pp.y);
        }
        raw_ptr
    }

    /// TreeSupport.cpp:6821-6825 `void TreeSupportData::clear_nodes()`
    pub fn clear_nodes(&mut self) {
        self.contact_nodes.clear();
    }

    /// TreeSupport.cpp:6827-6837 `coordf_t TreeSupportData::ceil_radius(coordf_t radius) const`
    pub fn ceil_radius(&self, radius: CoordF) -> CoordF {
        // size_t factor = (size_t)(radius / m_radius_sample_resolution);
        let factor = (radius / self.m_radius_sample_resolution) as usize;
        // coordf_t remains = radius - m_radius_sample_resolution * factor;
        let remains = (-self.m_radius_sample_resolution).mul_add(factor as f64, radius);
        if remains > EPSILON {
            radius + self.m_radius_sample_resolution - remains
        } else {
            radius
        }
    }

    /// TreeSupport.cpp:6839-6848 `const ExPolygons& TreeSupportData::calculate_collision(const RadiusLayerPair& key) const`
    fn calculate_collision(&self, radius: CoordF, layer_nr: usize) -> Arc<ExPolygons> {
        debug_assert!(layer_nr < self.m_layer_outlines.len());
        let mut collision_areas = offset_ex(&self.m_layer_outlines[layer_nr], scale_(radius + self.m_xy_distance) as f32);
        collision_areas = expolygons_simplify(&collision_areas, scale_(self.m_radius_sample_resolution));
        // insert() keeps the element already there
        let mut cache = self.m_collision_cache.borrow_mut();
        cache.entry((radius.to_bits(), layer_nr)).or_insert_with(|| Arc::new(collision_areas)).clone()
    }

    /// TreeSupport.cpp:6850-6877 `const ExPolygons& TreeSupportData::calculate_avoidance(const RadiusLayerPair& key) const`
    fn calculate_avoidance(&self, radius: CoordF, layer_nr: usize) -> Arc<ExPolygons> {
        let mut avoidance_areas: ExPolygons = Vec::new();
        if layer_nr > 0 {
            // Avoidance for a given layer depends on all layers beneath it so could have very deep recursion depths if
            // called at high layer heights. Limit the recursion depth to N by forcing the calculation of the layer N below.
            const MAX_RECURSION_DEPTH: usize = 100;
            // Check if we would exceed the recursion limit by trying to process this layer
            if layer_nr >= MAX_RECURSION_DEPTH
                && !self.m_avoidance_cache.borrow().contains_key(&(radius.to_bits(), layer_nr - MAX_RECURSION_DEPTH))
            {
                // Force the calculation of the layer `max_recursion_depth` below our current one, ignoring the result.
                self.get_avoidance(radius, layer_nr - MAX_RECURSION_DEPTH, 0);
            }
            avoidance_areas =
                offset_ex(&self.get_avoidance(radius, layer_nr - 1, 0), scale_(-self.m_max_move_distances[layer_nr - 1]) as f32);
        }
        let collision = self.get_collision(radius, layer_nr);
        avoidance_areas.extend(collision.iter().cloned());
        avoidance_areas = union_ex(&avoidance_areas);
        let mut cache = self.m_avoidance_cache.borrow_mut();
        cache.entry((radius.to_bits(), layer_nr)).or_insert_with(|| Arc::new(avoidance_areas)).clone()
    }
}

// ============================================================================
// TreeSupport (TreeSupport.hpp:381-643, TreeSupport.cpp:605-6705)
// ============================================================================

// libslic3r.h:92 `static constexpr float g_config_tree_support_collision_resolution = 0.2;`
const G_CONFIG_TREE_SUPPORT_COLLISION_RESOLUTION: f32 = 0.2;

// Flow.cpp:293-301 `float support_nozzle_diameter(const PrintObject *object, unsigned int filament_id)`
fn support_nozzle_diameter(print: &Print, filament_id: u32) -> f32 {
    if filament_id == 0 {
        let single_nozzle = print.single_diameter_layering_nozzle();
        if single_nozzle > EPSILON {
            return single_nozzle as f32;
        }
    }
    crate::print::faithful::nozzle_diameter_for_filament(print.config(), filament_id)
}

/// A key of a `std::map<coordf_t, ...>`: ordered as the doubles are.
#[derive(Debug, Clone, Copy, PartialEq)]
struct MapKeyF64(f64);
impl Eq for MapKeyF64 {}
impl PartialOrd for MapKeyF64 {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for MapKeyF64 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // std::less<double>: neither below the other is the same key (0.0 and -0.0).
        if self.0 < other.0 {
            std::cmp::Ordering::Less
        } else if other.0 < self.0 {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    }
}

/// TreeSupport.hpp:516-520 `struct FoldCarrier`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FoldCarrier {
    pub layer: usize,
    pub bottom_z: CoordF,
}

/// TreeSupport.hpp:381-643 `class TreeSupport`.
///
/// `m_object` / `m_object_config` / `m_print_config`: the object and print of `crate::print::faithful`.
/// `m_ts_data` is the `std::shared_ptr<TreeSupportData>` the object allocates
/// (`PrintObject::alloc_tree_support_preview_cache()`, PrintObject.cpp:1520-1528); the crate's PrintObject has
/// no preview cache member, so the tree support owns it. `contact_nodes` holds `NodeId`s into
/// `m_ts_data.contact_nodes`.
///
/// FIDELITY-NOTE: `SupportParameters m_support_params(object)` is built by the caller: the crate's
/// `SupportParameters::from_print_object` takes the slicing pipeline's PrintObject, not this one. Its
/// `support_base_width`, `preset_nozzle_diameter` and `multi_nozzle_diameter_mode` members
/// (SupportParameters.hpp:338-341) are not in the crate's struct and are kept here.
pub struct TreeSupport<'a> {
    // TreeSupport.hpp:424-431
    pub avg_node_per_layer: i32,
    pub nodes_angle: f32,
    pub has_sharp_tails: bool,
    pub has_cantilever: bool,
    pub max_cantilever_dist: f64,
    /// `SupportType support_type` as its integer value (`enums::SupportType`).
    pub support_type: i32,
    // TreeSupport.hpp:437-438
    pub m_print_config: &'a DynamicPrintConfig,
    pub m_machine_border: ExPolygon,
    /// TreeSupport.hpp:441 `std::map<const ExPolygon*, OverhangType> overhang_types`: not keyed by address here,
    /// the types travel in `Layer::loverhangs_with_type`.
    // TreeSupport.hpp:442
    pub m_vertical_enforcer_points: Vec<([f32; 3], [f32; 3])>,

    // TreeSupport.hpp:446-457
    pub contact_nodes: Vec<Vec<NodeId>>,
    pub m_ts_data: TreeSupportData,
    pub m_object: &'a mut PrintObject,
    pub m_print: &'a Print,
    pub m_slicing_params: SlicingParams,
    pub m_support_params: SupportParameters,
    pub m_support_merge_plan: SupportMergePlan,
    pub m_raft_layers: usize,
    pub m_highest_overhang_layer: usize,
    pub m_spanning_trees: Vec<Vec<MinimumSpanningTree>>,

    // TreeSupport.hpp:460 float DO_NOT_MOVER_UNDER_MM = 2.0;
    pub do_not_mover_under_mm: f32,
    // TreeSupport.hpp:461-465
    pub base_radius: CoordF,
    pub tip_radius: CoordF,
    // TreeSupport.hpp:472 double diameter_angle_scale_factor = tan(5.0*M_PI/180.0);
    pub diameter_angle_scale_factor: f64,
    // TreeSupport.hpp:474 const double minimum_roof_area{SQ(scaled<double>(1.))};
    pub minimum_roof_area: f64,
    // TreeSupport.hpp:486-491
    pub top_z_distance: f32,
    pub m_top_z_outlines: Vec<Vec<(f32, ExPolygons)>>,
    pub m_top_z_max: f32,
    // TreeSupport.hpp:503-505
    pub is_strong: bool,
    pub is_slim: bool,
    pub with_infill: bool,
    // TreeSupport.hpp:523-528
    pub m_fold_carriers: Vec<Vec<FoldCarrier>>,
    pub m_fold_slab_of_layer: Vec<i32>,
    pub m_base_only_fold_layers: Vec<usize>,

    /// TreeSupport.hpp:432 `std::unique_ptr<FillLightning::Generator> generator;`
    pub generator: Option<crate::fill::lightning::generator::Generator>,
    /// TreeSupport.hpp:433 `std::unordered_map<double, size_t> printZ_to_lightninglayer;`, keyed by the bits of the
    /// double (0.0 and -0.0 are one key).
    pub print_z_to_lightninglayer: HashMap<u64, usize>,
    /// What makes the `FillLightning::Generator` of draw_circles(), see `LightningSupportGenerator`; and the
    /// arguments draw_circles() hands its constructor.
    pub lightning_support_generator: Option<Box<dyn LightningSupportGenerator + 'a>>,
    pub lightning_contours: Vec<Polygons>,
    pub lightning_overhangs: Vec<Polygons>,
    pub lightning_support_density: f32,

    /// `m_object->print()->canceled()`: the crate's Print keeps its cancel state private.
    pub canceled: Box<dyn Fn() -> bool + 'a>,
    /// Layer.hpp:172-174 `mutable ExPolygons sharp_tails; mutable ExPolygons cantilevers; mutable std::vector<float>
    /// sharp_tails_height;` of every object layer, by its index, as detect_overhangs() fills them: the crate's
    /// Layer declares them as `Polygons` and one `f64`.
    pub layer_sharp_tails: Vec<ExPolygons>,
    pub layer_sharp_tails_height: Vec<Vec<f32>>,
    pub layer_cantilevers: Vec<ExPolygons>,
    /// `PrintObject::project_and_append_custom_facets()` for detect_overhangs(), see `CustomFacetsProjector`.
    pub custom_facets_projector: Option<Box<dyn CustomFacetsProjector + 'a>>,
    /// SupportParameters.hpp:338 `ConfigOptionFloat support_base_width`
    pub support_base_width: ConfigOptionFloat,
    /// SupportParameters.hpp:340 `float preset_nozzle_diameter`
    pub preset_nozzle_diameter: f32,
    /// SupportParameters.hpp:341 `bool multi_nozzle_diameter_mode`
    pub multi_nozzle_diameter_mode: bool,
}

// TreeSupport.hpp:466-470, 480-484
const TS_MAX_BRANCH_RADIUS: CoordF = 10.0;
const TS_MIN_BRANCH_RADIUS: CoordF = 0.4;
/// TreeSupport.hpp:478 `const double MIN_CARRIED_RATIO = 0.3;`
pub const MIN_CARRIED_RATIO: f64 = 0.3;
/// TreeSupport.hpp:484 `const double MIN_CARRIED_RATIO_INTERFACE = 0.02;`
pub const MIN_CARRIED_RATIO_INTERFACE: f64 = 0.02;

impl<'a> TreeSupport<'a> {
    /// `m_object_config`
    fn object_config(&self) -> &DynamicPrintConfig {
        self.m_object.config()
    }

    /// TreeSupport.cpp:605-648 `TreeSupport::TreeSupport(PrintObject& object, const SlicingParameters &slicing_params)`
    ///
    /// FIDELITY-NOTE: see the struct for `support_params`; `bed_shape_with_excluded_area` is the result of
    /// `get_bed_shape_with_excluded_area(*m_print_config)` (PrintConfig.cpp:11529), which the crate has no port of.
    pub fn new(
        object: &'a mut PrintObject,
        print: &'a Print,
        slicing_params: &SlicingParams,
        support_params: SupportParameters,
        bed_shape_with_excluded_area: &Polygon,
    ) -> Self {
        let m_print_config = print.config();
        let full_print_config = print.full_print_config();
        // 609 m_raft_layers = slicing_params.base_raft_layers + slicing_params.interface_raft_layers;
        let m_raft_layers = slicing_params.base_raft_layers + slicing_params.interface_raft_layers;
        let mut m_support_params = support_params;
        let object_config = object.config();
        // 610 support_type = m_object_config->support_type;
        let support_type = cfg_int(object_config, "support_type");

        // 612-617
        let mut support_pattern = cfg_int(object_config, "support_base_pattern");
        if m_support_params.support_style == SupportMaterialStyle::SmsTreeHybrid as i32
            && support_pattern == SupportMaterialPattern::SmpDefault as i32
        {
            support_pattern = SupportMaterialPattern::SmpRectilinear as i32; //smpLightning;
        }
        if support_pattern == SupportMaterialPattern::SmpLightning as i32 {
            m_support_params.base_fill_pattern = crate::print_config::InfillPattern::Lightning;
        }
        // 618 diameter_angle_scale_factor = std::clamp<double>(tree_support_branch_diameter_angle * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
        let diameter_angle_scale_factor =
            (cfg_float(object_config, "tree_support_branch_diameter_angle") * M_PI / 180.0).clamp(0.0, 0.5 * M_PI - EPSILON);
        let support_type_enum = support_type_from_i32(support_type);
        // 619-620
        let is_slim = crate::print_config::config_def::is_tree_slim(support_type_enum, support_style_from_i32(m_support_params.support_style));
        let is_strong = crate::print_config::config_def::is_tree(support_type_enum)
            && m_support_params.support_style == SupportMaterialStyle::SmsTreeStrong as i32;
        // 621-625
        let preset_nozzle_diameter = crate::print_config::config_def::primary_nozzle_diameter(full_print_config) as f32;
        let multi_nozzle_diameter_mode = crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(full_print_config);
        let support_extrusion_width: CoordF = crate::flow::Flow::extrusion_width_for_nozzle(
            &cfg_float_option(object_config, "line_width"),
            preset_nozzle_diameter,
            support_nozzle_diameter(print, cfg_int(object_config, "support_filament") as u32),
            multi_nozzle_diameter_mode,
        );
        // 626-630 A branch may not start thinner than a single extrusion, or draw_circles() is handed a circle it
        // cannot extrude, and it may not start thicker than the branch it grows into.
        let branch_diameter = cfg_float(object_config, "tree_support_branch_diameter");
        let tip_diameter_max = support_extrusion_width.max(branch_diameter);
        // std::clamp(v, lo, hi): (v < lo) ? lo : (hi < v) ? hi : v
        let tip_diameter = cfg_float(object_config, "tree_support_tip_diameter");
        let tip_radius = (if tip_diameter < support_extrusion_width {
            support_extrusion_width
        } else if tip_diameter_max < tip_diameter {
            tip_diameter_max
        } else {
            tip_diameter
        }) / 2.0;
        let base_radius = tip_radius.max(branch_diameter / 2.0);
        // 632 by default tree support needs no infill, unless it's tree hybrid which contains normal nodes.
        let with_infill = m_support_params.support_base_pattern != SupportMaterialPattern::SmpNone as i32;
        // 633-636
        let mut m_machine_border = ExPolygon::default();
        m_machine_border.contour = bed_shape_with_excluded_area.clone();
        let plate_offset = print.get_plate_origin();
        // align with the centered object in current plate (may not be the 1st plate, so need to add the plate offset)
        let shift = object.instances()[0].shift;
        let offset_pt = point_from_f64(scale_(plate_offset[0]), scale_(plate_offset[1]));
        m_machine_border.translate(Point::new(offset_pt.x - shift.x, offset_pt.y - shift.y));
        // 637 m_ts_data = m_object->alloc_tree_support_preview_cache();  (PrintObject.cpp:1520-1528)
        let xy_distance = cfg_float(object_config, "support_object_xy_distance");
        let m_ts_data = TreeSupportData::new(
            object,
            print,
            bed_shape_with_excluded_area,
            xy_distance,
            G_CONFIG_TREE_SUPPORT_COLLISION_RESOLUTION as f64,
        );
        // 638-640
        let mut top_z_distance = cfg_float(object_config, "support_top_z_distance") as f32;
        if top_z_distance as f64 > EPSILON {
            top_z_distance = top_z_distance.max(slicing_params.min_layer_height as f32);
        }
        // SupportParameters.hpp:123
        let support_base_width = if cfg_float(object_config, "support_line_width") > 0.0 {
            cfg_float_option(object_config, "support_line_width")
        } else {
            cfg_float_option(object_config, "line_width")
        };
        let mut this = TreeSupport {
            avg_node_per_layer: 0,
            nodes_angle: 0.0,
            has_sharp_tails: false,
            has_cantilever: false,
            max_cantilever_dist: 0.0,
            support_type,
            m_print_config,
            m_machine_border,
            m_vertical_enforcer_points: Vec::new(),
            contact_nodes: Vec::new(),
            m_ts_data,
            m_object: object,
            m_print: print,
            m_slicing_params: slicing_params.clone(),
            m_support_params,
            m_support_merge_plan: SupportMergePlan::default(),
            m_raft_layers,
            m_highest_overhang_layer: 0,
            m_spanning_trees: Vec::new(),
            do_not_mover_under_mm: 2.0,
            base_radius,
            tip_radius,
            diameter_angle_scale_factor,
            minimum_roof_area: sq(scale_(1.0)),
            top_z_distance,
            m_top_z_outlines: Vec::new(),
            m_top_z_max: 0.0,
            is_strong,
            is_slim,
            with_infill,
            m_fold_carriers: Vec::new(),
            m_fold_slab_of_layer: Vec::new(),
            m_base_only_fold_layers: Vec::new(),
            generator: None,
            print_z_to_lightninglayer: HashMap::new(),
            lightning_support_generator: None,
            lightning_contours: Vec::new(),
            lightning_overhangs: Vec::new(),
            lightning_support_density: 0.0,
            canceled: Box::new(|| false),
            layer_sharp_tails: Vec::new(),
            layer_sharp_tails_height: Vec::new(),
            layer_cantilevers: Vec::new(),
            custom_facets_projector: None,
            support_base_width,
            preset_nozzle_diameter,
            multi_nozzle_diameter_mode,
        };
        // 644
        this.init_top_z_by_nozzle();
        this
    }

    /// TreeSupport.hpp:410-422 `SupportNode* create_node(...)`
    #[allow(clippy::too_many_arguments)]
    pub fn create_node(
        &mut self,
        position: Point,
        distance_to_top: i32,
        obj_layer_nr: i32,
        support_roof_layers_below: i32,
        to_buildplate: bool,
        parent: Option<NodeId>,
        print_z_: CoordF,
        height_: CoordF,
        dist_mm_to_top_: CoordF,
        radius_: CoordF,
    ) -> NodeId {
        self.m_ts_data.create_node(
            position,
            distance_to_top,
            obj_layer_nr,
            support_roof_layers_below,
            to_buildplate,
            parent,
            print_z_,
            height_,
            dist_mm_to_top_,
            radius_,
        )
    }

    /// TreeSupport.cpp:650-712 `void TreeSupport::init_top_z_by_nozzle()`
    ///
    /// FIDELITY-NOTE: the tbb::parallel_for over the layers runs in order (every iteration writes its own
    /// element). `layerm->adaptive_merge_patches` (675-676) is not a member of the crate's LayerRegion.
    pub fn init_top_z_by_nozzle(&mut self) {
        self.m_top_z_outlines.clear();
        self.m_top_z_max = self.top_z_distance;
        // The contact nodes are pre-split on the object layer grid, see generate_contact_points(), so independent
        // support layers keep the global gap. A zero gap is the soluble interface contract and is never replaced.
        if (self.top_z_distance as f64) < EPSILON
            || self.m_support_params.independent_layer_height
            || !self.m_print.uses_multiple_nozzle_diameters()
        {
            return;
        }

        let print_config = self.m_print.config();
        let min_gap = self.m_slicing_params.min_layer_height as f32;
        // const coord_t resolution = scale_(m_ts_data->m_radius_sample_resolution);
        let resolution = scale_(self.m_ts_data.m_radius_sample_resolution) as Coord;
        // An overhang's underside is printed by the walls and the bridge infill, take the larger gap of the two.
        let region_gap = |wall_filament: u32, solid_infill_filament: u32| -> f32 {
            let nozzle: f64 = (crate::print::faithful::nozzle_diameter_for_filament(print_config, wall_filament)
                .max(crate::print::faithful::nozzle_diameter_for_filament(print_config, solid_infill_filament)))
                as f64;
            crate::adaptive_layer_merge::support_top_z_for_nozzle(nozzle).max(min_gap)
        };

        let layer_count = self.m_object.layer_count();
        self.m_top_z_outlines.resize(layer_count, Vec::new());
        for layer_nr in 0..layer_count {
            let mut buckets: Vec<(f32, ExPolygons)> = Vec::new();
            // Same footprint as TreeSupportData::m_layer_outlines: the region slices. Union before simplifying, so
            // neighbouring surfaces of one region cannot open cracks between them.
            for layerm in self.m_object.get_layer(layer_nr as i32).regions() {
                let region_area: ExPolygons = crate::surface::to_expolygons(&layerm.get_slices().surfaces);
                if region_area.is_empty() {
                    continue;
                }
                let gap = region_gap(layerm.region().config().wall_filament as u32, layerm.region().config().solid_infill_filament as u32);
                // bucket_of(gap)
                let pos = match buckets.iter().position(|bucket| ((bucket.0 - gap).abs() as f64) < EPSILON) {
                    Some(pos) => pos,
                    None => {
                        buckets.push((gap, Vec::new()));
                        buckets.len() - 1
                    }
                };
                for poly in &union_ex(&region_area) {
                    append(&mut buckets[pos].1, crate::clipper2_utils::union_ex_2(&expolygon_simplify_p(poly, resolution as f64)));
                }
            }
            for bucket in buckets.iter_mut() {
                bucket.1 = union_ex(&bucket.1);
            }
            buckets.retain(|bucket| !bucket.1.is_empty());
            // std::sort by the gap: the gaps of the buckets differ by EPSILON or more, so no two compare
            // equal and any sort gives the one order.
            buckets.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            self.m_top_z_outlines[layer_nr] = buckets;
        }

        let mut max_gap: f32 = 0.0;
        for buckets in &self.m_top_z_outlines {
            if !buckets.is_empty() {
                max_gap = max_gap.max(buckets[buckets.len() - 1].0);
            }
        }
        if max_gap <= 0.0 {
            self.m_top_z_outlines.clear();
        } else {
            self.m_top_z_max = max_gap;
        }
    }

    /// TreeSupport.cpp:714-721 `float TreeSupport::top_z_at(size_t obj_layer_nr) const`
    pub fn top_z_at(&self, obj_layer_nr: usize) -> f32 {
        if self.m_top_z_outlines.is_empty() {
            return self.top_z_distance;
        }
        if obj_layer_nr >= self.m_top_z_outlines.len() || self.m_top_z_outlines[obj_layer_nr].is_empty() {
            return self.m_top_z_max;
        }
        let buckets = &self.m_top_z_outlines[obj_layer_nr];
        buckets[buckets.len() - 1].0
    }

    /// TreeSupport.cpp:723-741 `float TreeSupport::top_z_for_overhang(size_t obj_layer_nr, const ExPolygon &overhang) const`
    pub fn top_z_for_overhang(&self, obj_layer_nr: usize, overhang: &ExPolygon) -> f32 {
        if self.m_top_z_outlines.is_empty() || obj_layer_nr >= self.m_top_z_outlines.len() || self.m_top_z_outlines[obj_layer_nr].len() < 2 {
            return self.top_z_at(obj_layer_nr);
        }
        // The outlines are simplified, so a neighbouring region's may poke a sliver into the overhang. Only an
        // overlap wider than that error counts; a mere touch is the fallback when nothing wider is found.
        let sliver = scale_(self.m_ts_data.m_radius_sample_resolution) as f32;
        let mut gap: f32 = 0.0;
        let mut touched: f32 = 0.0;
        for bucket in &self.m_top_z_outlines[obj_layer_nr] {
            let common = intersection_ex(&bucket.1, std::slice::from_ref(overhang));
            if common.is_empty() {
                continue;
            }
            touched = touched.max(bucket.0);
            if !opening_ex(&common, sliver).is_empty() {
                gap = gap.max(bucket.0);
            }
        }
        if gap > 0.0 {
            gap
        } else if touched > 0.0 {
            touched
        } else {
            self.top_z_at(obj_layer_nr)
        }
    }

    /// TreeSupport.cpp:743-776
    /// `void TreeSupport::union_outlines_within_top_z(ExPolygons &collision, size_t obj_layer_nr, bool clamp_to_first_layer_gap) const`
    pub fn union_outlines_within_top_z(&self, collision: &mut ExPolygons, obj_layer_nr: usize, clamp_to_first_layer_gap: bool) {
        if self.top_z_distance as f64 <= EPSILON {
            return;
        }
        // The object first layer is already offset by support_object_first_layer_gap. Outlines above still
        // have to be cleared, but expanding them by the full top-z gap cancels a smaller first-layer gap on a
        // vertical wall (the upper outline matches this one). Clamp only the XY expansion; the Z window stays
        // the top-z gap so a real overhang within that height is still kept out.
        let first_layer_gap: f32 =
            if clamp_to_first_layer_gap { cfg_float(self.object_config(), "support_object_first_layer_gap") as f32 } else { 0.0 };
        // returns scale_(gap): a double, narrowed to the float delta of offset_ex
        let xy_expand = |mut gap: f32| -> f32 {
            if clamp_to_first_layer_gap {
                gap = gap.min(first_layer_gap);
            }
            scale_(gap as f64) as f32
        };
        let mut above: ExPolygons = Vec::new();
        let mut accum_height: f32 = 0.0;
        for layer_id in obj_layer_nr + 1..self.m_ts_data.m_layer_outlines.len() {
            // accum_height += m_object->get_layer(layer_id)->height;  (float += double)
            accum_height = (accum_height as f64 + self.m_object.get_layer(layer_id as i32).height) as f32;
            if accum_height > self.m_top_z_max {
                break;
            }
            if self.m_top_z_outlines.is_empty() {
                append(&mut above, offset_ex(&self.m_ts_data.m_layer_outlines[layer_id], xy_expand(self.top_z_distance)));
            } else {
                // A fine nozzle's own overhang right above its thin gap must not be pulled down onto its interface
                // because a coarse nozzle elsewhere asks for a wider gap.
                for bucket in &self.m_top_z_outlines[layer_id] {
                    if accum_height <= bucket.0 {
                        append(&mut above, offset_ex(&bucket.1, xy_expand(bucket.0)));
                    }
                }
            }
        }
        if !above.is_empty() {
            append(collision, above);
            *collision = union_ex(collision);
        }
    }
}

/// TreeSupport.cpp:778-782 `void add_overhang(Layer *layer, const ExPolygon &overhang, int type)`
pub fn add_overhang(layer: &mut Layer, overhang: &ExPolygon, type_: i32) {
    layer.loverhangs_with_type.push((overhang.clone(), type_));
    layer.loverhangs.push(overhang.clone());
}

// PrintConfig.hpp `SupportType` / `SupportMaterialStyle` of their integer values.
fn support_type_from_i32(v: i32) -> SupportType {
    match v {
        0 => SupportType::StNormalAuto,
        1 => SupportType::StTreeAuto,
        2 => SupportType::StNormal,
        _ => SupportType::StTree,
    }
}

fn support_style_from_i32(v: i32) -> SupportMaterialStyle {
    match v {
        0 => SupportMaterialStyle::SmsDefault,
        1 => SupportMaterialStyle::SmsGrid,
        2 => SupportMaterialStyle::SmsSnug,
        3 => SupportMaterialStyle::SmsTreeSlim,
        4 => SupportMaterialStyle::SmsTreeStrong,
        5 => SupportMaterialStyle::SmsTreeHybrid,
        _ => SupportMaterialStyle::SmsTreeOrganic,
    }
}

// Polygon.cpp:171-187 `Point Polygon::centroid() const`. The result is `Point(Vec2d(...))`, the explicit
// constructor of Point.hpp:181, which rounds with lrint. `cross2` (Point.hpp) is one expression,
// `v1.x() * v2.y() - v2.x() * v1.y()`.
pub fn polygon_centroid(poly: &Polygon) -> Point {
    let points = &poly.points;
    let mut area_sum = 0.0_f64;
    let mut c = (0.0_f64, 0.0_f64);
    if points.len() >= 3 {
        let back = points[points.len() - 1];
        let mut p1 = (back.x as f64, back.y as f64);
        for p in points {
            let p2 = (p.x as f64, p.y as f64);
            let a = p1.0.mul_add(p2.1, -(p2.0 * p1.1));
            area_sum += a;
            c.0 += (p1.0 + p2.0) * a;
            c.1 += (p1.1 + p2.1) * a;
            p1 = p2;
        }
    }
    point_from_f64(c.0 / (3.0 * area_sum), c.1 / (3.0 * area_sum))
}

// Point.hpp:226-231 `void Point::rotate(double cos_a, double sin_a)`
#[inline]
fn point_rotate(pt: Point, cos_a: f64, sin_a: f64) -> Point {
    let cur_x = pt.x as f64;
    let cur_y = pt.y as f64;
    Point::new(cos_a.mul_add(cur_x, -(sin_a * cur_y)).round() as Coord, cos_a.mul_add(cur_y, sin_a * cur_x).round() as Coord)
}

/// TreeSupport.cpp:1758-1784 `static ExPolygon tree_support_node_disk(const SupportNode &node, const Polygon &unit_circle,
/// coordf_t branch_radius, coordf_t branch_radius_scaled, bool square_support, coordf_t radius = 0.)`
pub fn tree_support_node_disk(
    node: &SupportNode,
    unit_circle: &Polygon,
    branch_radius: CoordF,
    branch_radius_scaled: CoordF,
    square_support: bool,
    radius: CoordF,
) -> ExPolygon {
    let mut circle = unit_circle.clone();
    let scale = (if radius > 0.0 { radius } else { node.radius }) / branch_radius;
    let move_x = node.movement.x as f64 / (scale * branch_radius_scaled);
    let move_y = node.movement.y as f64 / (scale * branch_radius_scaled);
    if !square_support && move_x.abs() > 0.001 && move_y.abs() > 0.001 {
        let vsize_inv = 0.5 / (0.01 + move_x.mul_add(move_x, move_y * move_y).sqrt());
        let matrix: [f64; 4] = [
            scale * (move_x * move_x).mul_add(vsize_inv, 1.0),
            scale * move_x * move_y * vsize_inv,
            scale * move_x * move_y * vsize_inv,
            scale * (move_y * move_y).mul_add(vsize_inv, 1.0),
        ];
        for vertex in circle.points.iter_mut() {
            let x = matrix[0].mul_add(vertex.x as f64, matrix[1] * vertex.y as f64);
            let y = matrix[2].mul_add(vertex.x as f64, matrix[3] * vertex.y as f64);
            let p = point_from_f64(x, y);
            *vertex = Point::new(node.position.x + p.x, node.position.y + p.y);
        }
    } else {
        for vertex in circle.points.iter_mut() {
            // vertex = vertex * scale + node.position;  the member operator* of a non-const Point (lrint)
            let p = point_mul(*vertex, scale);
            *vertex = Point::new(p.x + node.position.x, p.y + node.position.y);
        }
    }
    ExPolygon::new(circle)
}

/// TreeSupport.cpp:4613-4625 `static double carried_area(const ExPolygons &carrier,
/// const std::vector<BoundingBox> &carrier_bboxes, const ExPolygon &region)`
///
/// Area of a region that the carrier below holds up. The bounding boxes come from get_extents_vector(carrier)
/// and spare the clipper boolean the candidates that are nowhere near the region.
pub fn carried_area(carrier: &ExPolygons, carrier_bboxes: &[BoundingBox], region: &ExPolygon) -> f64 {
    if carrier.is_empty() {
        return 0.0;
    }
    let region_bbox = get_extents_expoly(region);
    let mut near_carrier: Polygons = Vec::new();
    for i in 0..carrier.len() {
        if bbox_overlap(&carrier_bboxes[i], &region_bbox) {
            near_carrier.extend(crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygon(&carrier[i], &region_bbox, false));
        }
    }
    if near_carrier.is_empty() {
        return 0.0;
    }
    let mut area = 0.0;
    for part in &intersection_ex_polygons(std::slice::from_ref(region), &near_carrier) {
        area += part.area();
    }
    area
}

/// TreeSupport.cpp:4628-4633 `static double carried_area_ratio(...)`: fraction of a region that the carrier below holds up.
pub fn carried_area_ratio(carrier: &ExPolygons, carrier_bboxes: &[BoundingBox], region: &ExPolygon) -> f64 {
    let region_area = region.area();
    if region_area <= 0.0 {
        return 0.0;
    }
    carried_area(carrier, carrier_bboxes, region) / region_area
}

// BoundingBox.hpp:57-60 `bool overlap(const BoundingBoxBase<PointClass> &other) const`
#[inline]
fn bbox_overlap(a: &BoundingBox, b: &BoundingBox) -> bool {
    !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y || a.min.y > b.max.y)
}

/// TreeSupport.cpp:5103-5122 `static void smooth_filter(const Polygon &poly, int filter_size,
/// std::unordered_map<Point, Point, PointHash> &movements, double max_move)`
pub fn smooth_filter(poly: &Polygon, filter_size: i32, movements: &mut HashMap<Point, Point>, max_move: f64) {
    let pts = &poly.points;
    let n = pts.len() as i32;
    let center = polygon_centroid(poly);
    let mut newpoly = poly.clone();
    for i in 0..n {
        let mut filter_pt = Point::new(0, 0);
        let mut j = (i + n - filter_size) % n;
        while j != (i + filter_size + 1) % n {
            filter_pt = Point::new(filter_pt.x + pts[j as usize].x, filter_pt.y + pts[j as usize].y);
            j = (j + 1) % n;
        }
        let avg = point_div(filter_pt, (2 * filter_size + 1) as Coord);
        newpoly.points[i as usize] = Point::new(avg.x - center.x, avg.y - center.y);
    }
    if newpoly.area() < sq(scale_(10.0)) {
        // MultiPoint.cpp:6-10 `void MultiPoint::scale(double factor)`: `pt *= factor`, Point.hpp:199 (truncates)
        let factor = sq(scale_(10.0)) / newpoly.area();
        for pt in newpoly.points.iter_mut() {
            *pt = Point::new((pt.x as f64 * factor) as Coord, (pt.y as f64 * factor) as Coord);
        }
    }
    for i in 0..n as usize {
        let pt = pts[i];
        let mut move_ = Point::new(newpoly.points[i].x - (pt.x - center.x), newpoly.points[i].y - (pt.y - center.y));
        if vsize2_with_unscale(move_) > sq(max_move) {
            move_ = normal(move_, scale_(max_move));
        }
        movements.insert(pt, move_);
    }
}

/// TreeSupport.cpp:5124-5145 `static void densify_polygon(Polygon &poly, double max_edge_length)`
pub fn densify_polygon(poly: &mut Polygon, max_edge_length: f64) {
    let mut dense: Vec<Point> = Vec::new();
    let pts = &poly.points;
    let n = pts.len();
    for i in 0..n {
        let i2 = (i + 1) % n;
        dense.push(pts[i]);
        let mut prev = pts[i];
        // auto edge = pts[i2] - pts[i];
        let edge = Point::new(pts[i2].x - pts[i].x, pts[i2].y - pts[i].y);
        let n_frag = (vsize2_with_unscale(edge).sqrt() / max_edge_length).ceil() as i32;
        for j in 1..n_frag {
            // curr = pts[i] + edge * (double(j) / double(n_frag));  `edge` is a Point (Point.hpp `operator-`), not
            // const: the member operator* (lrint)
            let step = point_mul(edge, j as f64 / n_frag as f64);
            let curr = Point::new(pts[i].x + step.x, pts[i].y + step.y);
            if curr != prev {
                prev = curr;
                dense.push(curr);
            }
        }
    }
    poly.points = dense;
}

impl<'a> TreeSupport<'a> {
    /// TreeSupport.cpp:1554-1596 `void TreeSupport::create_tree_support_layers()`
    pub fn create_tree_support_layers(&mut self) {
        let mut layer_id: i32 = 0;
        if self.m_raft_layers > 0 {
            //create raft layers
            let mut raft_print_z: CoordF = 0.0;
            let mut raft_slice_z: CoordF;
            if self.m_slicing_params.base_raft_layers > 0 {
                // Do not add the raft contact layer, 1st layer should use first_print_layer_height
                let height = self.m_slicing_params.first_print_layer_height;
                raft_print_z += height;
                raft_slice_z = raft_print_z - height / 2.0;
                self.m_object.add_tree_support_layer(layer_id, height, raft_print_z, raft_slice_z);
                layer_id += 1;
            }
            // Insert the base layers.
            for _i in 1..self.m_slicing_params.base_raft_layers {
                let height = self.m_slicing_params.base_raft_layer_height;
                raft_print_z += height;
                raft_slice_z = raft_print_z - height / 2.0;
                self.m_object.add_tree_support_layer(layer_id, height, raft_print_z, raft_slice_z);
                layer_id += 1;
            }
            // Insert the interface layers.
            for _i in 0..self.m_slicing_params.interface_raft_layers {
                let height = self.m_slicing_params.interface_raft_layer_height;
                raft_print_z += height;
                raft_slice_z = raft_print_z - height / 2.0;
                self.m_object.add_tree_support_layer(layer_id, height, raft_print_z, raft_slice_z);
                layer_id += 1;
            }
            // Layers between the raft contacts and bottom of the object.
            let dist_to_go = self.m_slicing_params.object_print_z_min - raft_print_z;
            if dist_to_go > EPSILON {
                let nsteps = (dist_to_go / self.m_slicing_params.max_suport_layer_height).ceil() as i32;
                let height = dist_to_go / nsteps as f64;
                for _i in 0..nsteps {
                    raft_print_z += height;
                    raft_slice_z = raft_print_z - height / 2.0;
                    self.m_object.add_tree_support_layer(layer_id, height, raft_print_z, raft_slice_z);
                    layer_id += 1;
                }
            }
            self.m_raft_layers = layer_id as usize;
        }
    }

    /// TreeSupport.cpp:1745-1756 `coordf_t TreeSupport::support_flare_step_width(coordf_t print_z, coordf_t layer_height) const`
    pub fn support_flare_step_width(&self, print_z: CoordF, layer_height: CoordF) -> CoordF {
        let mut band_height = 0.0_f64;
        let candidate = self.m_support_merge_plan.coarsest_band_containing(print_z, &mut band_height);
        if !candidate.valid() || band_height <= layer_height + EPSILON {
            return self.m_support_params.support_extrusion_width;
        }
        // const Flow slab_flow = m_support_params.support_material_flow_for_nozzle(candidate.nozzle_diameter, float(band_height));
        // SupportParameters.hpp:342-346 -> Flow::new_from_config_width (Flow.cpp:136-157); only the width is read:
        // `float w`, the automatic width when the option is 0, else extrusion_width_for_nozzle().
        let nozzle_diameter = candidate.nozzle_diameter as f32;
        let w: f32 = if self.support_base_width.value == 0.0 {
            crate::flow::Flow::auto_extrusion_width(crate::flow::FlowRole::SupportMaterial, nozzle_diameter as f64) as f32
        } else {
            crate::flow::Flow::extrusion_width_for_nozzle(
                &self.support_base_width,
                self.preset_nozzle_diameter,
                nozzle_diameter,
                self.multi_nozzle_diameter_mode,
            ) as f32
        };
        // return slab_flow.width() * layer_height / band_height;
        w as f64 * layer_height / band_height
    }

    /// TreeSupport.cpp:2978-2988 `std::vector<size_t> TreeSupport::fold_carriers_above(size_t layer_nr) const`
    pub fn fold_carriers_above(&self, layer_nr: usize) -> Vec<usize> {
        let mut carriers: Vec<usize> = Vec::new();
        if layer_nr >= self.m_fold_slab_of_layer.len() || self.m_fold_slab_of_layer[layer_nr] < 0 {
            return carriers;
        }
        let layer_top = self.m_ts_data.layer_heights[layer_nr].print_z;
        for carrier in &self.m_fold_carriers[self.m_fold_slab_of_layer[layer_nr] as usize] {
            if carrier.layer > layer_nr && layer_top > carrier.bottom_z + EPSILON {
                carriers.push(carrier.layer);
            }
        }
        carriers
    }

    /// TreeSupport.cpp:3702-3720
    /// `coordf_t TreeSupport::calc_branch_radius(coordf_t base_radius, size_t layers_to_top, size_t tip_layers, double diameter_angle_scale_factor)`
    pub fn calc_branch_radius_layers(&self, base_radius: CoordF, layers_to_top: usize, tip_layers: usize, diameter_angle_scale_factor: f64) -> CoordF {
        let radius: f64;
        if !self.is_slim {
            if (layers_to_top + 1) > tip_layers {
                radius = (base_radius * (layers_to_top + 1) as f64).mul_add(diameter_angle_scale_factor, base_radius);
            } else {
                radius = base_radius * (layers_to_top + 1) as f64 / tip_layers as f64;
            }
        } else if (layers_to_top + 1) > tip_layers * 2 {
            radius = (base_radius * (layers_to_top + 1) as f64).mul_add(diameter_angle_scale_factor, base_radius);
        } else {
            radius = base_radius * (layers_to_top + 1) as f64 / (tip_layers * 2) as f64;
        }
        // radius = std::clamp(radius, tip_radius, MAX_BRANCH_RADIUS);
        std_clamp(radius, self.tip_radius, TS_MAX_BRANCH_RADIUS)
    }

    /// TreeSupport.cpp:3722-3743
    /// `coordf_t TreeSupport::calc_branch_radius(coordf_t base_radius, coordf_t mm_to_top, double diameter_angle_scale_factor, bool use_min_distance)`
    pub fn calc_branch_radius(&self, base_radius: CoordF, mm_to_top: CoordF, diameter_angle_scale_factor: f64, _use_min_distance: bool) -> CoordF {
        let mut radius: f64;
        let tip_height = base_radius; // this is a 45 degree tip
        if mm_to_top > tip_height {
            radius = (mm_to_top - tip_height).mul_add(diameter_angle_scale_factor, base_radius);
        } else {
            radius = mm_to_top; // this is a 45 degree tip
        }
        radius = std_clamp(radius, self.tip_radius, TS_MAX_BRANCH_RADIUS);
        if cfg_int(self.object_config(), "support_interface_top_layers") > 0 {
            radius = radius.max(base_radius);
        }
        // #if USE_SUPPORT_3D ... #endif
        radius
    }

    /// TreeSupport.cpp:3745-3748 `coordf_t TreeSupport::calc_radius(coordf_t mm_to_top)`
    pub fn calc_radius(&self, mm_to_top: CoordF) -> CoordF {
        self.calc_branch_radius(self.base_radius, mm_to_top, self.diameter_angle_scale_factor, true)
    }

    /// TreeSupport.cpp:3750-3755 `coordf_t TreeSupport::get_radius(const SupportNode* node)`
    pub fn get_radius(&mut self, node: NodeId) -> CoordF {
        if self.m_ts_data.contact_nodes[node].radius == 0.0 {
            let radius = self.calc_radius(self.m_ts_data.contact_nodes[node].dist_mm_to_top);
            self.m_ts_data.contact_nodes[node].radius = radius;
        }
        self.m_ts_data.contact_nodes[node].radius
    }

    /// TreeSupport.cpp:3757-3772 `ExPolygons TreeSupport::get_avoidance(coordf_t radius, size_t obj_layer_nr)` (USE_SUPPORT_3D 0)
    pub fn get_avoidance(&self, radius: CoordF, obj_layer_nr: usize) -> Arc<ExPolygons> {
        self.m_ts_data.get_avoidance(radius, obj_layer_nr, 0)
    }

    /// TreeSupport.cpp:3774-3789 `ExPolygons TreeSupport::get_collision(coordf_t radius, size_t layer_nr)` (USE_SUPPORT_3D 0)
    pub fn get_collision(&self, radius: CoordF, layer_nr: usize) -> Arc<ExPolygons> {
        self.m_ts_data.get_collision(radius, layer_nr)
    }

    /// TreeSupport.cpp:3791-3808 `Polygons TreeSupport::get_collision_polys(coordf_t radius, size_t layer_nr)` (USE_SUPPORT_3D 0)
    pub fn get_collision_polys(&self, radius: CoordF, layer_nr: usize) -> Polygons {
        let expolys = self.m_ts_data.get_collision(radius, layer_nr);
        let mut polys: Polygons = Vec::new();
        for expoly in expolys.iter() {
            for i in 0..expoly.num_contours() {
                polys.push(contour_or_hole(expoly, i).clone());
            }
        }
        polys
    }

    /// TreeSupport.cpp:6036-6136 `void TreeSupport::smooth_nodes()` (USE_TREE_SUPPORT_CARRIED_MASS 1)
    pub fn smooth_nodes(&mut self) {
        for layer_nr in 0..self.contact_nodes.len() {
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            for &node in &self.contact_nodes[layer_nr] {
                self.m_ts_data.contact_nodes[node].is_processed = false;
            }
        }
        // 6045-6053
        let support_nozzle: CoordF = support_nozzle_diameter(self.m_print, cfg_int(self.object_config(), "support_filament") as u32) as f64;
        let support_lw: CoordF = if cfg_float(self.object_config(), "support_line_width") > 0.0 {
            crate::flow::Flow::extrusion_width_for_nozzle(
                &cfg_float_option(self.object_config(), "support_line_width"),
                crate::print_config::config_def::primary_nozzle_diameter(self.m_print.full_print_config()) as f32,
                support_nozzle as f32,
                crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(self.m_print.full_print_config()),
            )
        } else {
            self.m_support_params.support_extrusion_width
        };
        let max_move: f32 = scale_(support_lw / 2.0) as f32;
        // if the branch is very tall, the tip also needs extra wall
        let thresh_tall_branch: f32 = 100.0;
        let thresh_dist_to_top: f32 = 30.0;
        let nodes = &mut self.m_ts_data.contact_nodes;
        for layer_nr in 0..self.contact_nodes.len() {
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            for &node in &self.contact_nodes[layer_nr] {
                if !nodes[node].is_processed {
                    let mut pts: Vec<Point> = Vec::new();
                    let mut radii: Vec<f64> = Vec::new();
                    let mut branch: Vec<NodeId> = Vec::new();
                    let mut p_node: Option<NodeId> = Some(node);
                    let mut total_height: f32 = 0.0;
                    // add a fixed head if it's not a polygon node, see STUDIO-4403
                    // Polygon node can't be added because the move distance might be huge, making the nodes in between jump and dangling
                    if let Some(child) = nodes[node].child {
                        if nodes[child].type_ != TreeNodeType::EPolygon {
                            pts.push(nodes[child].position);
                            radii.push(nodes[child].radius);
                            branch.push(child);
                            total_height = (total_height as f64 + nodes[child].height) as f32;
                        }
                    }
                    loop {
                        let p = p_node.unwrap();
                        pts.push(nodes[p].position);
                        radii.push(nodes[p].radius);
                        branch.push(p);
                        total_height = (total_height as f64 + nodes[p].height) as f32;
                        p_node = nodes[p].parent;
                        if !(p_node.is_some() && !nodes[p_node.unwrap()].is_processed) {
                            break;
                        }
                    }
                    if pts.len() < 3 {
                        continue;
                    }

                    let mut pts1 = pts.clone();
                    let mut radii1 = radii.clone();
                    // TODO here we assume layer height gap is constant. If not true, need to consider height jump
                    const ITERATIONS: usize = 100;
                    for k in 0..ITERATIONS {
                        for i in 1..pts.len() - 1 {
                            let pt = point_div(
                                Point::new(pts[i - 1].x + pts[i].x + pts[i + 1].x, pts[i - 1].y + pts[i].y + pts[i + 1].y),
                                3,
                            );
                            pts1[i] = pt;
                            radii1[i] = (radii[i - 1] + radii[i] + radii[i + 1]) / 3.0;
                            if k == ITERATIONS - 1 {
                                let b = &mut nodes[branch[i]];
                                b.position = pt;
                                b.radius = radii1[i];
                                b.movement = point_div(Point::new(pts[i + 1].x - pts[i - 1].x, pts[i + 1].y - pts[i - 1].y), 2);
                                b.is_processed = true;
                                b.overhang_degree = (10.0_f64).min(point_norm(b.movement) * SCALING_FACTOR / support_lw * 10.0);
                                if (b.movement.x as f32 > max_move || b.movement.y as f32 > max_move)
                                    || (total_height > thresh_tall_branch && b.dist_mm_to_top < thresh_dist_to_top as f64)
                                {
                                    b.need_extra_wall = true;
                                }
                            }
                        }
                        if k < ITERATIONS - 1 {
                            std::mem::swap(&mut pts, &mut pts1);
                            std::mem::swap(&mut radii, &mut radii1);
                        }
                    }
                }
            }
        }
    }

    /// TreeSupport.cpp:6257-6387 `std::vector<LayerHeightData> TreeSupport::plan_layer_heights()`
    pub fn plan_layer_heights(&mut self) -> Vec<LayerHeightData> {
        let mut layer_heights: Vec<LayerHeightData> = Vec::new();
        let mut z_heights: BTreeMap<MapKeyF64, CoordF> = BTreeMap::new(); // print_z:height
        if !self.m_support_params.independent_layer_height {
            layer_heights.resize(self.m_object.layer_count(), LayerHeightData::default());
            for layer_nr in 0..self.m_object.layer_count() {
                let layer = self.m_object.get_layer(layer_nr as i32);
                z_heights.insert(MapKeyF64(layer.print_z), layer.height);
                layer_heights[layer_nr] = LayerHeightData::from_values(layer.print_z, layer.height, layer_nr);
            }
        } else {
            let max_layer_height = self.m_slicing_params.max_suport_layer_height;
            let min_layer_height = self.m_slicing_params.min_layer_height;
            let mut bounds: BTreeMap<MapKeyF64, CoordF> = BTreeMap::new(); // print_z: height
            // Keep first layer still
            let layer0_print_z = self.m_object.get_layer(0).print_z;
            let layer0_height = self.m_object.get_layer(0).height;
            bounds.insert(MapKeyF64(layer0_print_z), layer0_height);
            let mut obj_layer_zs: Vec<f32> = Vec::with_capacity(self.m_object.layer_count());
            for l in self.m_object.layers() {
                obj_layer_zs.push(l.print_z as f32);
            }
            z_heights.insert(MapKeyF64(layer0_print_z), layer0_height);
            // Collect top contact layers
            for layer_nr in 1..self.contact_nodes.len() {
                if !self.contact_nodes[layer_nr].is_empty() {
                    let front = &self.m_ts_data.contact_nodes[self.contact_nodes[layer_nr][0]];
                    let print_z = front.print_z;
                    let height = front.height;
                    // insertion will fail if there is already a key of print_z, so no need to check
                    bounds.entry(MapKeyF64(print_z)).or_insert(height);
                    bounds.entry(MapKeyF64(print_z - height)).or_insert(0.0); // the bottom_z of the layer
                }
            }
            let bounds_vec: Vec<(CoordF, CoordF)> = bounds.iter().map(|(k, v)| (k.0, *v)).collect();
            let mut it1 = 0usize;
            for it2 in 1..bounds_vec.len() {
                let z2 = bounds_vec[it2].0;
                let z1 = bounds_vec[it1].0;
                let dist = z2 - z1;
                if dist < min_layer_height - EPSILON {
                    continue;
                }
                // Insert intermediate layers.
                let n_layers_extra = (dist / max_layer_height).ceil() as usize;
                let step = dist / n_layers_extra as CoordF;
                let mut print_z = z1 + step;
                for _i in 0..n_layers_extra {
                    z_heights.insert(MapKeyF64(print_z), step);
                    print_z += step;
                }
                it1 = it2;
            }

            // map z_heights to layer_heights
            let mut obj_layer_nr: usize = 0;
            layer_heights.resize(z_heights.len(), LayerHeightData::default());
            for (i, (k, v)) in z_heights.iter().enumerate() {
                let print_z = k.0;
                let height = *v;
                while obj_layer_nr < obj_layer_zs.len() && (obj_layer_zs[obj_layer_nr] as f64) < print_z - height / 2.0 {
                    obj_layer_nr += 1;
                }
                layer_heights[i] = LayerHeightData::from_values(print_z, height, obj_layer_nr);
            }
        }

        // add support layers according to layer_heights
        let mut support_layer_nr: i32 = self.m_raft_layers as i32;
        for i in 0..layer_heights.len() {
            let (id, idx) = {
                let ts_layer = self.m_object.add_tree_support_layer(
                    support_layer_nr,
                    layer_heights[i].height,
                    layer_heights[i].print_z,
                    layer_heights[i].print_z - layer_heights[i].height / 2.0,
                );
                (ts_layer.layer.id(), 0usize)
            };
            let _ = idx;
            if id > self.m_raft_layers {
                // SupportLayer *lower_layer = m_object->get_support_layer(ts_layer->id() - 1);
                // The links are the positions in `m_support_layers`, which is what the ids of these layers are.
                let n = self.m_object.m_support_layers.len();
                if id - 1 < n {
                    self.m_object.m_support_layers[id - 1].layer.upper_layer_id = Some(n - 1);
                    self.m_object.m_support_layers[n - 1].layer.lower_layer_id = Some(id - 1);
                }
            }
            support_layer_nr += 1;
        }

        // re-distribute contact_nodes to support layers
        let mut contact_nodes2: Vec<Vec<NodeId>> = vec![Vec::new(); support_layer_nr as usize];
        for layer_nr in 0..self.contact_nodes.len() {
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            let node1_print_z = self.m_ts_data.contact_nodes[self.contact_nodes[layer_nr][0]].print_z;
            // std::min_element: the first of the smallest
            let mut it = 0usize;
            for l in 1..layer_heights.len() {
                if (layer_heights[l].print_z - node1_print_z).abs() < (layer_heights[it].print_z - node1_print_z).abs() {
                    it = l;
                }
            }
            let layer_nr2 = it;
            let src = self.contact_nodes[layer_nr].clone();
            contact_nodes2[layer_nr2].extend(src);
        }
        self.contact_nodes = contact_nodes2;

        // adjust contact nodes' distance_to_top and support_roof_layers_below according to layer_heights
        // In case of very large top z distance, one gap layer is not enough, we need to split it into multiple layers
        for layer_nr in 0..self.contact_nodes.len() {
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            let node1_height = self.m_ts_data.contact_nodes[self.contact_nodes[layer_nr][0]].height;
            let new_height = layer_heights[layer_nr].height;
            if (node1_height - new_height).abs() < EPSILON {
                continue;
            }
            if (self.top_z_distance as f64) < EPSILON && node1_height < EPSILON {
                continue; // top_z_distance==0, this is soluable interface
            }
            let mut accum_height: CoordF = 0.0;
            let mut num_layers: i32 = 0;
            let mut i = layer_nr as i32;
            while i >= 0 {
                if layer_heights[i as usize].height > EPSILON {
                    accum_height += layer_heights[i as usize].height;
                    num_layers += 1;
                    if accum_height > node1_height - EPSILON {
                        break;
                    }
                }
                i -= 1;
            }
            for &node in &self.contact_nodes[layer_nr] {
                let n = &mut self.m_ts_data.contact_nodes[node];
                n.height = new_height;
                n.distance_to_top = -num_layers;
                n.support_roof_layers_below += num_layers - 1;
            }
        }
        layer_heights
    }

    /// TreeSupport.cpp:6693-6705 `void TreeSupport::insert_dropped_node(std::vector<SupportNode*>& nodes_layer, SupportNode* p_node)`
    ///
    /// `std::find` over the pointers: the conflicting node is the node itself when it is in the layer already.
    pub fn insert_dropped_node(&mut self, nodes_layer: &mut Vec<NodeId>, p_node: NodeId) {
        let conflicting_node_it = nodes_layer.iter().position(|&n| n == p_node);
        let Some(pos) = conflicting_node_it else {
            //No conflict.
            nodes_layer.push(p_node);
            return;
        };
        let conflicting_node = nodes_layer[pos];
        let (dtt, roof) = {
            let p = &self.m_ts_data.contact_nodes[p_node];
            (p.distance_to_top, p.support_roof_layers_below)
        };
        let c = &mut self.m_ts_data.contact_nodes[conflicting_node];
        c.distance_to_top = c.distance_to_top.max(dtt);
        c.support_roof_layers_below = c.support_roof_layers_below.max(roof);
    }
}

// `std::clamp(v, lo, hi)`: `(v < lo) ? lo : (hi < v) ? hi : v`
#[inline]
fn std_clamp(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}

// ============================================================================
// libnest2d `placers::EdgeCache<ExPolygon>` (libnest2d/placers/nfpplacer.hpp:155-345), what
// generate_contact_points() uses of it: the cache of the edges, `circumference()` and `coords()`.
// ============================================================================

// nfpplacer.hpp:160-165 `struct ContourCache`
#[derive(Debug, Clone, Default)]
struct EdgeCacheContour {
    emap: Vec<(Point, Point)>,
    distances: Vec<f64>,
    full_distance: f64,
}

/// libnest2d `EdgeCache<ExPolygon>`
#[derive(Debug, Clone, Default)]
pub struct EdgeCache {
    contour_: EdgeCacheContour,
    holes_: Vec<EdgeCacheContour>,
}

impl EdgeCache {
    // nfpplacer.hpp:171-174 `static double length(const Edge &e)`: sqrt of `_Segment::sqlength<double>()`
    // (geometry_traits.hpp:334-339, `a * a + b * b`).
    fn length(e: &(Point, Point)) -> f64 {
        let a = e.1.x as f64 - e.0.x as f64;
        let b = e.1.y as f64 - e.0.y as f64;
        a.mul_add(a, b * b).sqrt()
    }

    // nfpplacer.hpp:177-194, 197-213: the edges of one path, closed when its ends differ.
    fn create_contour_cache(points: &[Point]) -> EdgeCacheContour {
        let mut cache = EdgeCacheContour::default();
        if points.is_empty() {
            return cache;
        }
        cache.distances.reserve(points.len());
        for i in 0..points.len() - 1 {
            cache.emap.push((points[i], points[i + 1]));
            cache.full_distance += Self::length(&cache.emap[cache.emap.len() - 1]);
            cache.distances.push(cache.full_distance);
        }
        if points[0] != points[points.len() - 1] {
            cache.emap.push((points[points.len() - 1], points[0]));
            cache.full_distance += Self::length(&cache.emap[cache.emap.len() - 1]);
            cache.distances.push(cache.full_distance);
        }
        cache
    }

    /// nfpplacer.hpp:303-306 `EdgeCache(const RawShape& sh)` → `createCache(sh)` (176-217)
    pub fn new(sh: &ExPolygon) -> Self {
        let mut cache = EdgeCache { contour_: Self::create_contour_cache(&sh.contour.points), holes_: Vec::new() };
        for h in &sh.holes {
            cache.holes_.push(Self::create_contour_cache(&h.points));
        }
        cache
    }

    // nfpplacer.hpp:258-289 `Vertex coords(const ContourCache& cache, double distance) const`
    fn coords_cache(cache: &EdgeCacheContour, mut distance: f64) -> Point {
        if cache.distances.is_empty() || cache.emap.is_empty() {
            return Point::new(0, 0);
        }
        // Zero-perimeter contour (all edges collapsed to one point)
        if cache.full_distance <= 0.0 {
            return cache.emap[0].0;
        }
        if distance > 1.0 {
            distance %= 1.0;
        }
        // distance is from 0.0 to 1.0, we scale it up to the full length of the circumference
        let d = distance * cache.full_distance;
        let distances = &cache.distances;
        // Magic: we find the right edge in log time (std::lower_bound)
        let idx = distances.partition_point(|&x| x < d);
        let edge = cache.emap[idx]; // extrac the edge
        // Get the remaining distance on the target edge
        let ed = d - (if idx > 0 { distances[idx - 1] } else { 0.0 });
        // geometry_traits.hpp:455-468 `_Segment::angleToXaxis()`
        let dx = edge.1.x - edge.0.x;
        let dy = edge.1.y - edge.0.y;
        let mut angle = (dy as f64).atan2(dx as f64);
        if angle.is_sign_negative() {
            angle += 2.0 * std::f64::consts::PI;
        }
        // Get the point on the edge which lies in ed distance from the start
        Point::new(edge.0.x + (ed * angle.cos()).round() as Coord, edge.0.y + (ed * angle.sin()).round() as Coord)
    }

    /// nfpplacer.hpp:319-321
    pub fn coords(&self, distance: f64) -> Point {
        Self::coords_cache(&self.contour_, distance)
    }
    /// nfpplacer.hpp:323-326
    pub fn coords_hole(&self, hidx: usize, distance: f64) -> Point {
        Self::coords_cache(&self.holes_[hidx], distance)
    }
    /// nfpplacer.hpp:328-330
    pub fn circumference(&self) -> f64 {
        self.contour_.full_distance
    }
    /// nfpplacer.hpp:332-334
    pub fn circumference_hole(&self, hidx: usize) -> f64 {
        self.holes_[hidx].full_distance
    }
    /// `holeCount()`
    pub fn hole_count(&self) -> usize {
        self.holes_.len()
    }
}

// ClipperUtils.cpp:683-686 `ExPolygons diff_clipped(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset::No)`
fn diff_clipped(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
    let mut bbox = crate::geometry::get_extents(subject);
    // .inflated(SCALED_EPSILON)
    let eps = crate::libslic3r::SCALED_EPSILON as Coord;
    bbox.min = Point::new(bbox.min.x - eps, bbox.min.y - eps);
    bbox.max = Point::new(bbox.max.x + eps, bbox.max.y + eps);
    let clipped = crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(&clip.to_vec(), &bbox, false);
    diff_ex_polygons(subject, &clipped)
}

// ClipperUtils.hpp `Lines intersection_ln(const Line &subject, const Polygons &clip)` = `_clipper_ln(ctIntersection, {subject}, clip)`
// (ClipperUtils.cpp): the line as a polyline clipped by the polygons, each piece as a line of its ends.
fn intersection_ln(subject: &Line, clip: &[Polygon]) -> Lines {
    let polylines = vec![crate::geometry::Polyline::from_points(vec![subject.a, subject.b])];
    let polylines = crate::clipper_utils::intersection_pl_clib(&polylines, &crate::clipper_utils::polygons_as_paths(clip));
    let mut retval: Lines = Vec::new();
    for polyline in &polylines {
        if polyline.points.len() >= 2 {
            retval.push(Line::new(polyline.points[0], polyline.points[polyline.points.len() - 1]));
        }
    }
    retval
}

// ClipperUtils.hpp:392-393 `ExPolygons shrink_ex(const ExPolygons &polygons, const float delta)` = offset_ex(polygons, -delta)
fn shrink_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
    offset_ex(expolygons, -delta)
}

// SupportCommon.cpp:3037-3051 `ExPolygons safe_union(const ExPolygons first, const ExPolygons second = {})`.
// A local copy: `safe_union_ex` of support_common.rs unions on another clipper.
fn safe_union(first: &[ExPolygon], second: &[ExPolygon]) -> ExPolygons {
    use crate::clipper::{EndType, JoinType};
    use crate::support::support_common::{cu, cu_expolygon};
    let mut result: ExPolygons = Vec::new();
    if !first.is_empty() || !second.is_empty() {
        result = cu_expolygon::union_ex_2(first, second);
        if result.is_empty() {
            // just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may aleady have problems with the area.
            let to_polylines = |src: &[ExPolygon]| -> crate::geometry::Polylines {
                let mut out = Vec::new();
                for expoly in src {
                    out.extend(cu_expolygon::to_polylines_closed(expoly));
                }
                out
            };
            // scaled<float>(0.002)
            let delta = (0.002 / SCALING_FACTOR) as f32;
            let offset = |src: &[ExPolygon]| -> Polygons {
                let polylines = to_polylines(src);
                // raw_offset() of no paths is no paths
                if polylines.is_empty() {
                    Vec::new()
                } else {
                    cu::offset_polylines(&polylines, delta, JoinType::jtMiter, 1.2, EndType::etOpenButt)
                }
            };
            let result_polys = cu::union_2(&offset(first), &offset(second));
            for poly in result_polys {
                result.push(ExPolygon::new(poly));
            }
        }
    }
    result
}

// SupportCommon.hpp:218-256 `template<typename CollisionPolyType> ExPolygons safe_offset_inc(const ExPolygons &me,
// coord_t distance, const CollisionPolyType &collision, coord_t safe_step_size, coord_t last_step_offset_without_check,
// size_t min_amount_offset)` with CollisionPolyType = ExPolygons. A local copy: `safe_offset_inc_ex` of
// support_common.rs takes the collision as Polygons and runs on another clipper.
fn safe_offset_inc(
    me: &ExPolygons,
    distance: Coord,
    collision: &[ExPolygon],
    safe_step_size: Coord,
    last_step_offset_without_check: Coord,
    min_amount_offset: usize,
) -> ExPolygons {
    use crate::clipper::JoinType;
    use crate::support::support_common::cu;
    let mut do_final_difference = last_step_offset_without_check == 0;
    let mut ret: ExPolygons = safe_union(me, &[]); // ensure sane input

    // Trim the collision polygons with the region of interest for diff() efficiency.
    let mut collision_trimmed_buffer: Polygons = Vec::new();
    macro_rules! collision_trimmed {
        () => {{
            if collision_trimmed_buffer.is_empty() && !collision.is_empty() {
                // get_extents(ret).inflated(std::max(0, distance) + SCALED_EPSILON): the sum is a double,
                // truncated to the coord_t inflated() takes
                let delta = (0.max(distance) as f64 + scale_(EPSILON)) as Coord;
                let mut bbox = crate::geometry::get_extents(&ret);
                bbox.min = Point::new(bbox.min.x - delta, bbox.min.y - delta);
                bbox.max = Point::new(bbox.max.x + delta, bbox.max.y + delta);
                collision_trimmed_buffer =
                    crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(&collision.to_vec(), &bbox, false);
            }
            &collision_trimmed_buffer
        }};
    }
    // scaled<float>(0.01), the arc tolerance of jtRound
    let arc_tolerance = (0.01 / SCALING_FACTOR) as f32 as f64;
    // scaled<double>(0.015)
    let simplify_tolerance = 0.015 / SCALING_FACTOR;

    if distance == 0 {
        return if do_final_difference {
            cu::diff_ex_ep(&ret, collision_trimmed!(), cu::ApplySafetyOffset::No)
        } else {
            cu::union_ex_expolygons(&ret)
        };
    }
    if safe_step_size < 0 || last_step_offset_without_check < 0 {
        // BOOST_LOG_TRIVIAL(error) << "Offset increase got invalid parameter!";
        return if do_final_difference {
            cu::diff_ex_ep(&ret, collision_trimmed!(), cu::ApplySafetyOffset::No)
        } else {
            cu::union_ex_expolygons(&ret)
        };
    }

    // coord_t is 32 bits (libslic3r.h:40), so is the arithmetic of the steps
    let distance = distance as i32;
    let last_step_offset_without_check = last_step_offset_without_check as i32;
    let safe_step_size = safe_step_size as i32;
    let mut step_size: i32 = safe_step_size;
    let mut steps: i32 = if distance > last_step_offset_without_check { (distance - last_step_offset_without_check) / step_size } else { 0 };
    if distance - steps * step_size > last_step_offset_without_check {
        if (steps + 1) * step_size <= distance {
            // This will be the case when last_step_offset_without_check >= safe_step_size
            steps += 1;
        } else {
            do_final_difference = true;
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
    // offset in steps
    for i in 0..steps {
        ret = cu::diff_ex_ep(
            &cu::offset_ex_expolygons(&ret, step_size as f32, JoinType::jtRound, arc_tolerance),
            collision_trimmed!(),
            cu::ApplySafetyOffset::No,
        );
        // ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
        if i % 10 == 7 {
            ret = expolygons_simplify(&ret, simplify_tolerance);
        }
    }
    // offset the remainder
    let last_offset: f32 = (distance - steps * step_size) as f32;
    if last_offset as f64 > scale_(EPSILON) {
        ret = cu::offset_ex_expolygons(&ret, (distance - steps * step_size) as f32, JoinType::jtRound, arc_tolerance);
    }
    ret = expolygons_simplify(&ret, simplify_tolerance);

    if do_final_difference {
        ret = cu::diff_ex_ep(&ret, collision_trimmed!(), cu::ApplySafetyOffset::No);
    }
    cu::union_ex_expolygons(&ret)
}

// ClipperUtils.cpp:567-569 `ExPolygons offset_ex(const ExPolygon &expolygon, const float delta, ...)`, the overload
// `offset_ex({expolygon}, delta)` and `offset_ex(expolygons[0], delta)` resolve to:
// `ClipperPaths_to_Slic3rExPolygons(expolygon_offset(...))`, an even-odd union, not the PolyTree of
// `offset_ex(const ExPolygons &, ...)`.
fn offset_ex_expolygon(expolygon: &ExPolygon, delta: f32) -> ExPolygons {
    crate::support::support_common::cu_expolygon::offset_ex(expolygon, delta, crate::clipper::JoinType::jtMiter, 3.0)
}

/// The locals the lambdas of `TreeSupport::generate_contact_points()` capture, per layer (TreeSupport.cpp:6470-6514).
struct ContactPointsLayer {
    layer_nr: usize,
    curr_nodes: Vec<NodeId>,
    already_inserted: HashSet<Point>,
    bottom_z: CoordF,
    added: bool,
    is_sharp_tail: bool,
    overhang_top_z: CoordF,
    overhang_gap_layers: i32,
    overhang_gap_height: CoordF,
}

impl<'a> TreeSupport<'a> {
    // TreeSupport.cpp:6479-6494 the lambda `set_overhang_top_z`
    fn contact_points_set_overhang_top_z(&self, ctx: &mut ContactPointsLayer, top_z: CoordF, gap_layers: i32, presplit_gap: bool) {
        ctx.overhang_top_z = top_z;
        ctx.overhang_gap_layers = gap_layers;
        ctx.overhang_gap_height = top_z;
        if !presplit_gap {
            return;
        }
        let mut accum_height: CoordF = 0.0;
        ctx.overhang_gap_layers = 0;
        let mut i = ctx.layer_nr as i32 - 1;
        while i >= 0 {
            accum_height += self.m_object.get_layer(i).height;
            ctx.overhang_gap_layers += 1;
            if accum_height > top_z - EPSILON {
                break;
            }
            i -= 1;
        }
        ctx.overhang_gap_height = self.m_object.get_layer(ctx.layer_nr as i32 - 1).height;
    }

    // TreeSupport.cpp:6495-6516 the lambda `insert_point`
    #[allow(clippy::too_many_arguments)]
    fn contact_points_insert_point(
        &mut self,
        ctx: &mut ContactPointsLayer,
        pt: Point,
        overhang: &ExPolygon,
        radius: f64,
        force_add: bool,
        add_interface: bool,
        radius_scaled: Coord,
        support_roof_layers: usize,
    ) -> Option<NodeId> {
        let hash_pos = point_div(pt, (radius_scaled + 1) / 1);
        let mut contact_node: Option<NodeId> = None;
        if force_add || !ctx.already_inserted.contains(&hash_pos) {
            ctx.already_inserted.insert(hash_pos);
            let to_buildplate = true;
            let roof_layers: usize = if add_interface { support_roof_layers } else { 0 };
            // add a new node as a virtual node which acts as the invisible gap between support and object
            // distance_to_top<0: it's virtual (-1, or minus the gap layers once pre-split)
            // print_z=object_layer->bottom_z: it directly contacts the bottom
            // height=z_distance_top: it's height is exactly the gap distance (the layer height once pre-split)
            // dist_mm_to_top=0: it directly contacts the bottom
            if ctx.bottom_z - ctx.overhang_top_z < self.m_object.get_layer(0).print_z {
                return contact_node; // dont add it if overlapping with the initial layer
            }
            let node = self.m_ts_data.create_node(
                pt,
                -ctx.overhang_gap_layers,
                ctx.layer_nr as i32 - 1,
                (roof_layers as i64 + ctx.overhang_gap_layers as i64) as i32,
                to_buildplate,
                SupportNode::NO_PARENT,
                ctx.bottom_z,
                ctx.overhang_gap_height,
                0.0,
                radius,
            );
            let n = &mut self.m_ts_data.contact_nodes[node];
            n.overhang = overhang.clone();
            n.is_sharp_tail = ctx.is_sharp_tail;
            n.carried_mass = (M_PI * sq(radius.max(self.tip_radius))) as f32;
            ctx.curr_nodes.push(node);
            ctx.added = true;
            contact_node = Some(node);
        }
        contact_node
    }

    /// TreeSupport.cpp:6389-6691 `void TreeSupport::generate_contact_points()`
    ///
    /// FIDELITY-NOTE: the tbb::parallel_for over the layers runs in order; every layer fills its own
    /// `contact_nodes[layer_nr - 1]`, and `all_nodes` (a concurrent vector whose order the threads decide) is
    /// filled in layer order. `layer->lower_layer` is the object layer below.
    pub fn generate_contact_points(&mut self) {
        let config = self.m_object.config();
        let point_spread: CoordF = scale_(cfg_float(config, "tree_support_branch_distance"));
        let max_bridge_length: CoordF = scale_(cfg_float(config, "max_bridge_length"));
        let radius_scaled: Coord = scale_(self.base_radius) as Coord;
        let _on_buildplate_only = cfg_bool(config, "support_on_build_plate_only");
        let roof_enabled = cfg_int(config, "support_interface_top_layers") > 0;
        let force_tip_to_roof = roof_enabled && self.m_support_params.soluble_interface;
        let extrusion_width: CoordF = crate::flow::Flow::extrusion_width_for_nozzle(
            &cfg_float_option(config, "line_width"),
            crate::print_config::config_def::primary_nozzle_diameter(self.m_print.full_print_config()) as f32,
            support_nozzle_diameter(self.m_print, cfg_int(config, "support_filament") as u32),
            crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(self.m_print.full_print_config()),
        );
        //First generate grid points to cover the entire area of the print.
        let bounding_box = self.m_object.bounding_box();
        let bounding_box_size = Point::new(bounding_box.max.x - bounding_box.min.x, bounding_box.max.y - bounding_box.min.y);
        let rotate_angle: f64 = 22.0 / 180.0 * M_PI;
        let center = point_div(Point::new(bounding_box.max.x + bounding_box.min.x, bounding_box.max.y + bounding_box.min.y), 2);
        let sin_angle = rotate_angle.sin();
        let cos_angle = rotate_angle.cos();
        let rotated_dims = point_div(
            point_from_f64(
                (bounding_box_size.x as f64).mul_add(cos_angle, bounding_box_size.y as f64 * sin_angle),
                (bounding_box_size.x as f64).mul_add(sin_angle, bounding_box_size.y as f64 * cos_angle),
            ),
            2,
        );

        let mut grid_points: Vec<Point> = Vec::new();
        let sample_step: CoordF = point_spread.max(max_bridge_length / 2.0);
        // `auto x = -rotated_dims(0)` is a coord_t: `x += sample_step` truncates every step.
        let mut x: Coord = -rotated_dims.x;
        while x < rotated_dims.x {
            let mut y: Coord = -rotated_dims.y;
            while y < rotated_dims.y {
                let mut pt = Point::new(x, y);
                pt = point_rotate(pt, cos_angle, sin_angle);
                pt = Point::new(pt.x + center.x, pt.y + center.y);
                if bounding_box.contains_point(&pt) {
                    grid_points.push(pt);
                }
                y = (y as f64 + sample_step) as Coord;
            }
            x = (x as f64 + sample_step) as Coord;
        }

        let layer_height: CoordF = cfg_float(config, "layer_height");
        let z_distance_top: CoordF = self.top_z_distance as f64;
        // const int z_distance_top_layers = round_up_divide(scale_(m_top_z_max), scale_(layer_height)) + 1;
        // round_up_divide<double>: (dividend + divisor - 1) / divisor
        let z_distance_top_layers: i32 = {
            let dividend = scale_(self.m_top_z_max as f64);
            let divisor = scale_(layer_height);
            ((dividend + divisor - 1.0) / divisor + 1.0) as i32
        }; //Support must always be 1 layer below overhang.
        let gap_layers: i32 = if z_distance_top == 0.0 { 0 } else { 1 };
        // BBS multi-diameter: the gap follows the overhang's nozzle, so one layer's contacts may need different
        // gaps. plan_layer_heights() splits a gap into virtual layers from the first contact of each layer only,
        // so split each contact here instead, exactly as it would.
        let presplit_gap = !self.m_top_z_outlines.is_empty();

        let mut support_roof_layers: usize = cfg_int(config, "support_interface_top_layers") as usize;
        if support_roof_layers > 0 {
            support_roof_layers += 1; // BBS: add a normal support layer below interface (if we have interface)
        }
        let support_threshold_angle = cfg_int(config, "support_threshold_angle");
        let _thresh_angle: CoordF =
            (89.0_f32).min(if (support_threshold_angle as f64) < EPSILON { 30.0_f32 } else { support_threshold_angle as f32 }) as f64;
        let support_expansion = cfg_float(config, "support_expansion");
        let support_object_xy_distance = cfg_float(config, "support_object_xy_distance");
        let mut xy_expansion: CoordF = scale_(support_expansion);
        if self.m_support_params.soluble_interface && xy_expansion < EPSILON {
            xy_expansion = scale_(2.0);
        }

        // fix bug of generating support for very thin objects
        let n_layers = self.m_object.layers().len();
        if n_layers <= (z_distance_top_layers + 1) as usize {
            return;
        }

        self.contact_nodes.clear();
        self.contact_nodes.resize(n_layers, Vec::new());

        // add vertical enforcer points
        // Layer.hpp:411-418 zs_from_layers(): `(float)l->slice_z`
        let zs: Vec<f32> = self.m_object.layers().iter().map(|l| l.slice_z as f32).collect();
        let mut vertical_enforcer_points_by_layers: Vec<Vec<(Point, Point)>> = vec![Vec::new(); self.m_object.layer_count()];
        for pt_and_normal in &self.m_vertical_enforcer_points {
            let pt = pt_and_normal.0;
            let normal = pt_and_normal.1; // normal seems useless
            let iter = zs.partition_point(|&z| z < pt[2]);
            if iter != zs.len() {
                let layer_nr = iter;
                if layer_nr > 0 && layer_nr < self.contact_nodes.len() {
                    // { to_2d(pt).cast<coord_t>(), scaled(to_2d(normal)) }: Point.hpp:550-554 `(v / SCALING_FACTOR).cast<coord_t>()`
                    // on a float vector, the divisor as a float
                    vertical_enforcer_points_by_layers[layer_nr].push((
                        Point::new(pt[0] as Coord, pt[1] as Coord),
                        Point::new((normal[0] / SCALING_FACTOR as f32) as Coord, (normal[1] / SCALING_FACTOR as f32) as Coord),
                    ));
                }
            }
        }

        let mut nonempty_layers: i32 = 0;
        let mut all_nodes: Vec<[f32; 3]> = Vec::new();
        let soluble_interface = self.m_support_params.soluble_interface;
        let support_style = self.m_support_params.support_style;
        let xy_distance = self.m_ts_data.m_xy_distance;
        for layer_nr in 1..n_layers {
            if (self.canceled)() {
                break;
            }
            let (loverhangs_with_type, layer_bottom_z, lower_lslices_extrudable) = {
                let layer = self.m_object.get_layer(layer_nr as i32);
                let lower = self.m_object.get_layer(layer_nr as i32 - 1);
                (layer.loverhangs_with_type.clone(), layer.bottom_z(), lower.lslices_extrudable.clone())
            };
            let mut ctx = ContactPointsLayer {
                layer_nr,
                curr_nodes: std::mem::take(&mut self.contact_nodes[layer_nr - 1]),
                already_inserted: HashSet::new(),
                bottom_z: layer_bottom_z,
                added: false, // Did we add a point this way?
                is_sharp_tail: false,
                overhang_top_z: z_distance_top,
                overhang_gap_layers: gap_layers,
                overhang_gap_height: z_distance_top,
            };

            // auto extrudable_collision = offset_ex(layer->lower_layer->lslices_extrudable, m_ts_data->m_xy_distance);
            let extrudable_collision = offset_ex(&lower_lslices_extrudable, xy_distance as f32);
            for overhang_with_type in &loverhangs_with_type {
                let overhang_part = &overhang_with_type.0;
                let overhang_type = overhang_with_type.1;
                ctx.is_sharp_tail = (overhang_type & OverhangType::SHARP_TAIL.0 as i32) != 0;
                let top_z = self.top_z_for_overhang(layer_nr, overhang_part) as CoordF;
                self.contact_points_set_overhang_top_z(&mut ctx, top_z, gap_layers, presplit_gap);
                let mut add_interface = force_tip_to_roof || overhang_part.area() > self.minimum_roof_area;
                let relevant_forbidden = self.get_collision(0.0, layer_nr - 1);
                let mut overhangs: ExPolygons = vec![overhang_part.clone()];
                let mut overhangs_regular: ExPolygons;
                if !ctx.is_sharp_tail && (support_expansion > EPSILON) {
                    overhangs = safe_offset_inc(
                        &vec![overhang_part.clone()],
                        scale_(support_expansion) as Coord,
                        &relevant_forbidden,
                        scale_(TS_MIN_BRANCH_RADIUS * 1.75) as Coord,
                        0,
                        1,
                    );
                }
                if support_style == SupportMaterialStyle::SmsTreeHybrid as i32
                    && (overhang_type & (OverhangType::BIG_FLAT.0 as i32 | OverhangType::THIN_PLATE.0 as i32)) != 0
                {
                    let outlines_below_offset =
                        offset_ex(&self.m_ts_data.m_layer_outlines_below[layer_nr - 1], scale_(support_object_xy_distance) as f32);
                    overhangs_regular = intersection_ex(&overhangs, &outlines_below_offset);
                    let mut overhangs_normal = offset2_ex(
                        &diff_ex(std::slice::from_ref(overhang_part), &overhangs_regular),
                        scale_(extrusion_width) as f32,
                        -scale_(extrusion_width) as f32,
                    );
                    if add_interface && xy_expansion > EPSILON && !ctx.is_sharp_tail {
                        overhangs_normal = safe_offset_inc(
                            &overhangs_normal,
                            xy_expansion as Coord,
                            &outlines_below_offset,
                            scale_(TS_MIN_BRANCH_RADIUS * 1.75) as Coord,
                            0,
                            1,
                        );
                    }
                    // if the outside area is still big, we can need normal nodes
                    let gap_width: Coord = (scale_(extrusion_width / 2.0) + scale_(xy_distance)) as Coord;
                    let mut overhangs_normal_split: ExPolygons = Vec::new();
                    let collision_layer = self.get_collision(0.0, layer_nr);
                    for overhang in &overhangs_normal {
                        let sub_overhangs = overhang.split_expoly_with_holes(gap_width, &collision_layer);
                        if !sub_overhangs.is_empty() {
                            for sub in sub_overhangs {
                                overhangs_normal_split.push(sub);
                            }
                        } else {
                            overhangs_normal_split.push(overhang.clone());
                        }
                    }
                    for overhang in &overhangs_normal_split {
                        if !is_stable(layer_bottom_z as f32, overhang, 0.0) || overhang.area() < sq(scale_(2.0)) {
                            let unstable_overhangs = intersection_ex(std::slice::from_ref(overhang), &overhangs);
                            overhangs_regular.extend(unstable_overhangs);
                            continue;
                        }
                        let overhang_bounds = get_extents_expoly(overhang);
                        let radius = bbox_radius(&overhang_bounds) * SCALING_FACTOR;
                        let candidate =
                            point_div(Point::new(overhang_bounds.min.x + overhang_bounds.max.x, overhang_bounds.min.y + overhang_bounds.max.y), 2);
                        let Some(contact_node) =
                            self.contact_points_insert_point(&mut ctx, candidate, overhang, radius, true, true, radius_scaled, support_roof_layers)
                        else {
                            continue;
                        };
                        let n = &mut self.m_ts_data.contact_nodes[contact_node];
                        n.type_ = TreeNodeType::EPolygon;
                        n.carried_mass = n.carried_mass.max((overhang.area() * SCALING_FACTOR * SCALING_FACTOR) as f32);
                        ctx.curr_nodes.push(contact_node);
                    }
                } else {
                    overhangs_regular = overhangs.clone();
                }
                // if (!is_sharp_tail && layer->lower_layer)
                if !ctx.is_sharp_tail {
                    overhangs_regular = diff_ex(&overhangs_regular, &extrudable_collision);
                }
                for overhang in &overhangs_regular {
                    let nodes_before = ctx.curr_nodes.len();
                    if ctx.is_sharp_tail && !soluble_interface && overhang.area() < sq(scale_(2.0)) {
                        add_interface = false;
                    }
                    let overhang_bounds = get_extents_expoly(overhang);
                    let radius: f64 = std_clamp(bbox_radius(&overhang_bounds) * SCALING_FACTOR, self.tip_radius, self.base_radius);
                    // add supports at corners for both auto and manual overhangs, github #2008
                    let points = &overhang.contour.points;
                    let n_size = points.len() as i32;
                    for i in 0..n_size {
                        let pt = points[i as usize];
                        let v1 = vec2d_normalized(pt, points[((i - 1 + n_size) % n_size) as usize]);
                        let v2 = vec2d_normalized(pt, points[((i + 1) % n_size) as usize]);
                        if v1.0 * v2.0 + v1.1 * v2.1 > -0.7 {
                            // angle smaller than 135 degrees
                            let contact_node =
                                self.contact_points_insert_point(&mut ctx, pt, overhang, radius, false, add_interface, radius_scaled, support_roof_layers);
                            if let Some(contact_node) = contact_node {
                                self.m_ts_data.contact_nodes[contact_node].is_corner = true;
                            }
                        }
                    }
                    // add supports along contours
                    let edge_cache = EdgeCache::new(overhang);
                    for i in 0..edge_cache.hole_count() + 1 {
                        let step = point_spread / (if i == 0 { edge_cache.circumference() } else { edge_cache.circumference_hole(i - 1) });
                        let mut distance = 0.0_f64;
                        while distance < 1.0 {
                            let pt = if i == 0 { edge_cache.coords(distance) } else { edge_cache.coords_hole(i - 1, distance) };
                            self.contact_points_insert_point(&mut ctx, pt, overhang, radius, false, add_interface, radius_scaled, support_roof_layers);
                            distance += step;
                        }
                    }
                    // don't add inner supports for sharp tails
                    if ctx.is_sharp_tail {
                        let mut cent = polygon_centroid(&overhang.contour);
                        // For ring/concave sharp tails the centroid may fall outside the overhang
                        // (e.g. inside its hole). In that case project it onto the nearest edge of
                        // the overhang so the sharp tail still gets a contact point.
                        if !is_inside_ex(&vec![overhang.clone()], &cent) {
                            cent = overhang.point_projection(&cent);
                        }
                        self.contact_points_insert_point(&mut ctx, cent, overhang, radius, false, add_interface, radius_scaled, support_roof_layers);
                    } else {
                        // add inner supports
                        // overhang_bounds.inflated(-radius_scaled);  (the inflated copy is dropped)
                        let overhang_inner = offset_ex(std::slice::from_ref(overhang), -(radius_scaled as f32));
                        for &candidate in &grid_points {
                            if overhang_bounds.contains_point(&candidate) {
                                // BBS: move_inside_expoly shouldn't be used if candidate is already inside, as it moves point to boundary and the inside is not well supported!
                                let is_inside = is_inside_ex(&overhang_inner, &candidate);
                                if is_inside {
                                    self.contact_points_insert_point(
                                        &mut ctx,
                                        candidate,
                                        overhang,
                                        radius,
                                        false,
                                        add_interface,
                                        radius_scaled,
                                        support_roof_layers,
                                    );
                                }
                            }
                        }
                    }
                    let nodes_added = ctx.curr_nodes.len() - nodes_before;
                    if nodes_added > 0 {
                        let overhang_area_mm2 = (overhang.area() * SCALING_FACTOR * SCALING_FACTOR) as f32;
                        let node_mass = (overhang_area_mm2 / nodes_added as f32).max((M_PI * sq(self.tip_radius)) as f32);
                        for idx in nodes_before..ctx.curr_nodes.len() {
                            self.m_ts_data.contact_nodes[ctx.curr_nodes[idx]].carried_mass = node_mass;
                        }
                    }
                }
            }
            if !vertical_enforcer_points_by_layers[layer_nr].is_empty() {
                let top_z = self.top_z_at(layer_nr) as CoordF;
                self.contact_points_set_overhang_top_z(&mut ctx, top_z, gap_layers, presplit_gap);
            }
            for pt_and_normal in &vertical_enforcer_points_by_layers[layer_nr] {
                ctx.is_sharp_tail = true; // fake it as sharp tail point so the contact distance will be 0
                let vertical_enforcer_point = pt_and_normal.0;
                // insert_point(vertical_enforcer_point, ExPolygon(), false): the radius is `false`, 0
                let node = self.contact_points_insert_point(
                    &mut ctx,
                    vertical_enforcer_point,
                    &ExPolygon::default(),
                    0.0,
                    false,
                    true,
                    radius_scaled,
                    support_roof_layers,
                );
                if let Some(node) = node {
                    self.m_ts_data.contact_nodes[node].skin_direction = pt_and_normal.1;
                }
            }
            if !ctx.curr_nodes.is_empty() {
                nonempty_layers += 1;
            }
            for &node in &ctx.curr_nodes {
                let n = &self.m_ts_data.contact_nodes[node];
                all_nodes.push([n.position.x as f32, n.position.y as f32, scale_(n.print_z) as f32]);
            }
            self.contact_nodes[layer_nr - 1] = ctx.curr_nodes;
        }

        let n_nodes = all_nodes.len() as i32;
        self.avg_node_per_layer = 0;
        self.nodes_angle = 0.0;
        if n_nodes > 0 {
            self.avg_node_per_layer = n_nodes / nonempty_layers;
            // get orientation of nodes by line fitting
            // line: y=kx+b, where
            //       k=tan(nodes_angle)=(n\sum{xy}-\sum{x}\sum{y})/(n\sum{x^2}-\sum{x}^2)
            let (mut mx, mut my, mut mxy, mut mx2) = (0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32);
            for pt in &all_nodes {
                let x: f32 = (pt[0] as f64 * SCALING_FACTOR) as f32;
                let y: f32 = (pt[1] as f64 * SCALING_FACTOR) as f32;
                mx += x;
                my += y;
                mxy = x.mul_add(y, mxy);
                mx2 = x.mul_add(x, mx2);
            }
            let n = n_nodes as f32;
            self.nodes_angle = n.mul_add(mxy, -(mx * my)).atan2(n.mul_add(mx2, -(mx * mx)));
        }
    }
}

// `(a - b).cast<double>().normalized()` of Eigen: the vector over its norm when the squared norm is positive.
fn vec2d_normalized(a: Point, b: Point) -> (f64, f64) {
    let x = (a.x - b.x) as f64;
    let y = (a.y - b.y) as f64;
    let z = x * x + y * y;
    if z > 0.0 {
        let n = z.sqrt();
        (x / n, y / n)
    } else {
        (x, y)
    }
}

// TreeSupport.cpp:5237-5255 the lambda `is_line_cut_by_contour` of drop_nodes(); the cache is the
// `tbb::concurrent_unordered_map<Line, bool, LineHash>` of the layer.
fn is_line_cut_by_contour(cache: &mut HashMap<(Point, Point), bool>, layer_contours: &Polygons, a: Point, b: Point) -> bool {
    if let Some(&cut) = cache.get(&(a, b)) {
        if cut {
            return true;
        }
    } else {
        profiler().tic();
        let ln = Line::new(b, a);
        let pls_intersect = intersection_ln(&ln, layer_contours);
        // insert() keeps an element already there
        cache.entry((a, b)).or_insert(!pls_intersect.is_empty());
        cache.entry((b, a)).or_insert(!pls_intersect.is_empty());
        profiler().stage_add(TreeSupportStage::StageIntersectionLn);
        if !pls_intersect.is_empty() {
            return true;
        }
    }
    false
}

impl<'a> TreeSupport<'a> {
    // TreeSupport.cpp:5165-5176 the lambda `get_max_move_dist` of drop_nodes()
    fn get_max_move_dist(&mut self, node: NodeId, tan_angle: f64, power: i32) -> f64 {
        if self.m_ts_data.contact_nodes[node].max_move_dist == 0.0 {
            let radius = self.get_radius(node);
            self.m_ts_data.contact_nodes[node].radius = radius;
            let (height, print_z) = (self.m_ts_data.contact_nodes[node].height, self.m_ts_data.contact_nodes[node].print_z);
            self.m_ts_data.contact_nodes[node].max_move_dist = (tan_angle * height).min(self.support_flare_step_width(print_z, height));
        }
        let mut move_dist = self.m_ts_data.contact_nodes[node].max_move_dist;
        if power == 2 {
            move_dist = sq(move_dist);
        }
        move_dist
    }

    /// TreeSupport.cpp:5147-6034 `void TreeSupport::drop_nodes()` (USE_TREE_SUPPORT_CARRIED_MASS 1)
    ///
    /// FIDELITY-NOTE: the second pass is a `tbb::parallel_for_each` over the nodes of a part, which appends
    /// the nodes of the layer below under a mutex in the order the threads reach it; here the nodes are taken
    /// in order, the schedule of one thread. The avoidance of all radii is precalculated in order as well.
    pub fn drop_nodes(&mut self) {
        macro_rules! n {
            ($id:expr) => {
                self.m_ts_data.contact_nodes[$id]
            };
        }
        let config = self.m_object.config();
        // Use Minimum Spanning Tree to connect the points on each layer and move them while dropping them down.
        let support_extrusion_width: CoordF = self.m_support_params.support_extrusion_width;
        let layer_height: CoordF = cfg_float(config, "layer_height");
        let angle: f64 = cfg_float(config, "tree_support_branch_angle") * M_PI / 180.0;
        let wall_count: i32 = 1.max(cfg_int(config, "tree_support_wall_count"));
        let tan_angle = angle.tan(); // when nodes are thick, they can move further. this is the max angle
        let max_move_distance: CoordF = if angle < M_PI / 2.0 { (tan_angle * layer_height) * wall_count as f64 } else { CoordF::MAX };
        let max_move_distance2: f64 = max_move_distance * max_move_distance;
        let _tip_layers: usize = (self.base_radius / layer_height) as usize; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
        let radius_sample_resolution: CoordF = self.m_ts_data.m_radius_sample_resolution;
        let support_on_buildplate_only = cfg_bool(config, "support_on_build_plate_only");
        let _top_interface_layers: usize = cfg_int(config, "support_interface_top_layers") as usize;
        SupportNode::set_diameter_angle_scale_factor(self.diameter_angle_scale_factor);
        // enabled only when the wall count is set to auto or dual-wall is explicitly enabled by the user.
        let tree_support_wall_count = cfg_int(config, "tree_support_wall_count");
        let bottom_expand_enabled = tree_support_wall_count > 1 || tree_support_wall_count < 0;
        let do_not_mover_under_mm: f64 = self.do_not_mover_under_mm as f64;
        let num_top_interface_layers: usize = self.m_support_params.num_top_interface_layers;
        let is_strong = self.is_strong;

        if self.m_ts_data.layer_heights.is_empty() {
            return;
        }

        // precalculate avoidance of all possible radii.
        // This will cause computing more (radius, layer_nr) pairs, but it's worth to do so since we are doning this in parallel.
        {
            // get all the possible radiis
            let n = self.contact_nodes.len();
            let mut all_layer_radius: Vec<std::collections::BTreeSet<MapKeyF64>> = vec![Default::default(); n];
            let mut all_layer_node_dist: Vec<std::collections::BTreeSet<MapKeyF64>> = vec![Default::default(); n];
            let mut layer_nr = n - 1;
            while layer_nr > 0 {
                for &p_node in &self.contact_nodes[layer_nr] {
                    all_layer_node_dist[layer_nr].insert(MapKeyF64(n!(p_node).dist_mm_to_top));
                }
                let layer_nr_next = layer_nr - 1;
                if layer_nr_next <= n - 1 && layer_nr_next > 0 {
                    let dists: Vec<f64> = all_layer_node_dist[layer_nr].iter().map(|d| d.0).collect();
                    for node_dist in dists {
                        all_layer_node_dist[layer_nr_next].insert(MapKeyF64(node_dist + self.m_ts_data.layer_heights[layer_nr].height));
                    }
                }
                let dists: Vec<f64> = all_layer_node_dist[layer_nr].iter().map(|d| d.0).collect();
                for node_dist in dists {
                    all_layer_radius[layer_nr].insert(MapKeyF64(self.calc_radius(node_dist)));
                }
                layer_nr -= 1;
            }
            // parallel pre-compute avoidance
            for layer_nr in 0..n - 1 {
                for node_radius in &all_layer_radius[layer_nr] {
                    let obj_layer_nr = self.m_ts_data.layer_heights[layer_nr].obj_layer_nr;
                    self.m_ts_data.get_avoidance(node_radius.0, obj_layer_nr, 0);
                    self.get_collision(0.0, obj_layer_nr);
                    self.get_collision(node_radius.0, obj_layer_nr);
                }
            }
        }

        self.m_spanning_trees.resize(self.contact_nodes.len(), Vec::new());

        let mut layer_nr = self.contact_nodes.len() - 1;
        // Skip layer 0, since we can't drop down the vertices there.
        while layer_nr > 0 {
            let this_layer_nr = layer_nr;
            layer_nr -= 1;
            let layer_nr = this_layer_nr;
            if (self.canceled)() {
                break;
            }
            if self.contact_nodes[layer_nr].is_empty() {
                continue;
            }

            let layer_nr_next = layer_nr - 1;
            let _print_z: CoordF = self.m_ts_data.layer_heights[layer_nr].print_z;
            let print_z_next: CoordF = self.m_ts_data.layer_heights[layer_nr_next].print_z;
            let height_next: CoordF = self.m_ts_data.layer_heights[layer_nr_next].height;
            let obj_layer_nr: usize = self.m_ts_data.layer_heights[layer_nr].obj_layer_nr;
            let obj_layer_nr_next: usize = self.m_ts_data.layer_heights[layer_nr_next].obj_layer_nr;
            // All nodes that are leaves on this layer that would result in unsupported ('mid-air') branches.
            let mut unsupported_branch_leaves: std::collections::VecDeque<(usize, NodeId)> = std::collections::VecDeque::new();

            let layer_contours: Polygons = self.m_ts_data.get_contours_with_holes(obj_layer_nr);
            let mut mst_line_x_layer_contour_cache: HashMap<(Point, Point), bool> = HashMap::new();

            //Group together all nodes for each part.
            let parts: ExPolygons = self.m_ts_data.m_layer_outlines_below[obj_layer_nr].clone();
            //All nodes that aren't inside a part get grouped together in the 0th part.
            let mut nodes_per_part: Vec<HashMap<Point, NodeId>> = vec![HashMap::new(); 1 + parts.len()];
            let layer_contact_nodes = self.contact_nodes[layer_nr].clone();
            for &p_node in &layer_contact_nodes {
                if !n!(p_node).to_buildplate {
                    // Can't rest on model and unable to reach the build plate. Then we must drop the node and leave parts unsupported.
                    if support_on_buildplate_only {
                        unsupported_branch_leaves.push_front((layer_nr, p_node));
                        continue;
                    }

                    //If permitted to fall on the model, downward growth ceases.
                    let node_radius = n!(p_node).radius;
                    let node_position = n!(p_node).position;
                    let collision0 = self.get_collision(0.0, obj_layer_nr);
                    let overlap_with_circle = shrink_ex(&collision0, scale_(node_radius) as f32);
                    if !overlap_with_circle.is_empty() && is_inside_ex(&overlap_with_circle, &node_position) {
                        continue;
                    }
                    let mut circle = crate::geometry::make_circle(scale_(node_radius), 0.00789 * scale_(node_radius));
                    circle.translate(node_position);
                    let area = avoid_object_remove_extra_small_parts(&ExPolygon::new(circle), &collision0);
                    if !area.is_empty() {
                        n!(p_node).overhang = area[0].clone();
                        let parent = n!(p_node).parent.expect("a node that cannot reach the build plate has a parent");
                        if area[0].area() < sq(scale_(1.0))
                            || (!n!(parent).to_buildplate
                                && !crate::ex_polygon::overlaps_expolys(
                                    std::slice::from_ref(&n!(p_node).overhang),
                                    std::slice::from_ref(&n!(parent).overhang),
                                ))
                        {
                            n!(p_node).valid = false;
                            n!(p_node).is_processed = true;
                            continue;
                        }
                    } else {
                        continue;
                    }
                }
                if n!(p_node).to_buildplate || parts.is_empty() {
                    //It's outside, so make it go towards the build plate.
                    if n!(p_node).type_ == TreeNodeType::ECircle && nodes_per_part[0].contains_key(&n!(p_node).position) {
                        let p = n!(p_node).position;
                        n!(p_node).position = Point::new(p.x + 1, p.y + 1);
                    }
                    nodes_per_part[0].insert(n!(p_node).position, p_node);
                    continue;
                }

                // Find which part this node is located in and group the nodes in the same part together.
                // Since nodes have a radius and the avoidance areas are offset by that radius, the set of parts
                // may be different per node. Here we consider a node to be inside the part that is closest.
                let node_position = n!(p_node).position;
                let mut closest_part_distance2: CoordF = CoordF::MAX;
                let mut closest_part: usize = usize::MAX;
                for part_index in 0..parts.len() {
                    //If it's inside, the distance is 0 and this part is considered the best.
                    if is_inside_ex_poly(&parts[part_index], &node_position) {
                        closest_part = part_index;
                        closest_part_distance2 = 0.0;
                        break;
                    }
                    let closest_point = polygon_closest_vertex(&parts[part_index].contour, &node_position).unwrap_or(Point::new(0, 0));
                    let distance2: CoordF =
                        vsize2_with_unscale(Point::new(node_position.x - closest_point.x, node_position.y - closest_point.y));
                    if distance2 < closest_part_distance2 {
                        closest_part_distance2 = distance2;
                        closest_part = part_index;
                    }
                }
                let _ = closest_part_distance2;

                //Put it in the best one. Index + 1 because the 0th index is the outside part.
                let part_slot = closest_part.wrapping_add(1);
                if n!(p_node).type_ == TreeNodeType::ECircle && nodes_per_part[part_slot].contains_key(&n!(p_node).position) {
                    let p = n!(p_node).position;
                    n!(p_node).position = Point::new(p.x + 1, p.y + 1);
                }
                nodes_per_part[part_slot].insert(n!(p_node).position, p_node);
            }

            //Create a MST for every part.
            profiler().tic();
            let mut spanning_trees: Vec<MinimumSpanningTree> = Vec::new();
            for group in &nodes_per_part {
                let mut points_to_buildplate: Vec<Point> = group.keys().copied().collect();
                // the keys are distinct, so the order is the one order by (x, y)
                points_to_buildplate.sort_by(|a, b| if a.x != b.x { a.x.cmp(&b.x) } else { a.y.cmp(&b.y) });
                spanning_trees.push(MinimumSpanningTree::from_vertices(points_to_buildplate));
            }
            profiler().stage_add(TreeSupportStage::StageMinimumSpanningTree);

            for group_index in 0..nodes_per_part.len() {
                let nodes_this_part = nodes_per_part[group_index].clone();
                let mst = &spanning_trees[group_index];
                let mut nodes_vec: Vec<(Point, NodeId)> = nodes_this_part.iter().map(|(k, v)| (*k, *v)).collect();
                nodes_vec.sort_by(|a, b| if a.0.x != b.0.x { a.0.x.cmp(&b.0.x) } else { a.0.y.cmp(&b.0.y) });
                //In the first pass, merge all nodes that are close together.
                let mut lower_avoid_p1: Arc<ExPolygons> = Arc::new(Vec::new());
                if group_index == 0 {
                    let lookahead_layers: i32 = 1.max((8.0 / height_next.max(0.1)).round() as i32);
                    let lower_obj_layer: usize =
                        if obj_layer_nr_next > lookahead_layers as usize { obj_layer_nr_next - lookahead_layers as usize } else { 0 };
                    lower_avoid_p1 = self.get_avoidance(TS_MIN_BRANCH_RADIUS, lower_obj_layer);
                }
                for entry in &nodes_vec {
                    let p_node = entry.1;
                    if !n!(p_node).valid {
                        continue;
                    }
                    if n!(p_node).fading {
                        continue;
                    }
                    let node_position = n!(p_node).position;
                    let neighbours: Vec<Point> = mst.adjacent_nodes(node_position);
                    'first_pass_next: {
                        if n!(p_node).type_ == TreeNodeType::EPolygon {
                            // Remove all circle neighbours that are completely inside the polygon and merge them into this node.
                            for neighbour in &neighbours {
                                let neighbour_node = nodes_this_part[neighbour];
                                let mut can_merge = false;
                                if !n!(neighbour_node).valid {
                                    continue;
                                }
                                if n!(neighbour_node).fading {
                                    continue;
                                }
                                if n!(neighbour_node).type_ == TreeNodeType::EPolygon {
                                    let node_dtt = n!(p_node).distance_to_top;
                                    let nb_dtt = n!(neighbour_node).distance_to_top;
                                    // `int > size_t`: compared as unsigned
                                    if (node_dtt < 0 && nb_dtt < 0)
                                        || ((node_dtt as i64 as usize) > num_top_interface_layers + 1
                                            && (nb_dtt as i64 as usize) > num_top_interface_layers + 1)
                                    {
                                        let node_overhang = n!(p_node).overhang.clone();
                                        let nb_overhang = n!(neighbour_node).overhang.clone();
                                        let overhang_shrinked =
                                            shrink_ex(std::slice::from_ref(&node_overhang), scale_(support_extrusion_width) as f32);
                                        if !overhang_shrinked.is_empty()
                                            && crate::ex_polygon::overlaps_expolys(&overhang_shrinked, std::slice::from_ref(&nb_overhang))
                                        {
                                            let mut tmp = union_ex(&[node_overhang, nb_overhang]);
                                            if !tmp.is_empty() && tmp.len() == 1 {
                                                let next_pt = polygon_centroid(&tmp[0].contour);
                                                let roof = n!(p_node).support_roof_layers_below.max(n!(neighbour_node).support_roof_layers_below) - 1;
                                                let next_node = self.m_ts_data.create_node(
                                                    next_pt,
                                                    node_dtt.max(nb_dtt) + 1,
                                                    obj_layer_nr_next as i32,
                                                    roof,
                                                    true,
                                                    Some(p_node),
                                                    print_z_next,
                                                    height_next,
                                                    0.0,
                                                    0.0,
                                                );
                                                n!(next_node).max_move_dist = 0.0;
                                                n!(next_node).overhang = tmp.remove(0);
                                                n!(next_node).target_radius = n!(p_node).target_radius.max(n!(neighbour_node).target_radius);
                                                n!(next_node).origin_area = n!(next_node).overhang.area();
                                                let nb_mass = n!(neighbour_node).total_carried_mass(&self.m_ts_data.contact_nodes);
                                                n!(next_node).carried_mass += nb_mass;
                                                self.contact_nodes[layer_nr_next].push(next_node);
                                                n!(p_node).valid = false;
                                                n!(neighbour_node).valid = false;
                                                break 'first_pass_next;
                                            }
                                        }
                                    }
                                } else {
                                    let neighbour_radius: Coord = scale_(n!(neighbour_node).radius) as Coord;
                                    let pt_north = Point::new(neighbour.x, neighbour.y + neighbour_radius);
                                    let pt_south = Point::new(neighbour.x, neighbour.y - neighbour_radius);
                                    let pt_west = Point::new(neighbour.x - neighbour_radius, neighbour.y);
                                    let pt_east = Point::new(neighbour.x + neighbour_radius, neighbour.y);
                                    let node_overhang = n!(p_node).overhang.clone();
                                    can_merge = is_inside_ex_poly(&node_overhang, neighbour)
                                        && is_inside_ex_poly(&node_overhang, &pt_north)
                                        && is_inside_ex_poly(&node_overhang, &pt_south)
                                        && is_inside_ex_poly(&node_overhang, &pt_west)
                                        && is_inside_ex_poly(&node_overhang, &pt_east);
                                    if !can_merge && is_inside_ex_poly(&node_overhang, neighbour) {
                                        n!(neighbour_node).fading = true;
                                        if bbox_radius(&node_overhang.contour.bounding_box()) < neighbour_radius as f64 {
                                            n!(p_node).target_radius = n!(neighbour_node).radius;
                                        }
                                    }
                                }
                                if n!(p_node).valid && can_merge {
                                    let nb_merged = n!(neighbour_node).merged_neighbours.clone();
                                    n!(p_node).merged_neighbours.insert(0, neighbour_node);
                                    n!(p_node).merged_neighbours.extend(nb_merged);
                                    n!(neighbour_node).valid = false;
                                }
                            }
                        } else if neighbours.len() == 1
                            && vsize2_with_unscale(Point::new(neighbours[0].x - node_position.x, neighbours[0].y - node_position.y))
                                < self.get_max_move_dist(p_node, tan_angle, 2)
                            && mst.adjacent_nodes(neighbours[0]).len() == 1
                            && n!(nodes_this_part[&neighbours[0]]).type_ != TreeNodeType::EPolygon
                        {
                            // We have just two nodes left, and they're very close, and the only neighbor is not ePolygon
                            //Insert a completely new node and let both original nodes fade.
                            let neighbour = nodes_this_part[&neighbours[0]];
                            //Average position of the two nodes.
                            let mid_position = point_div(Point::new(node_position.x + neighbours[0].x, node_position.y + neighbours[0].y), 2);
                            let mut next_position = mid_position;
                            let next_radius: CoordF = self.calc_radius(n!(p_node).dist_mm_to_top + height_next);
                            let avoid_layer = self.get_avoidance(next_radius, obj_layer_nr_next);
                            {
                                let m_self: f64 = (n!(p_node).total_carried_mass(&self.m_ts_data.contact_nodes) as f64).max(EPSILON);
                                let m_nb: f64 = (n!(neighbour).total_carried_mass(&self.m_ts_data.contact_nodes) as f64).max(EPSILON);
                                let mut nb_weight = m_nb / (m_self + m_nb);
                                if group_index == 0 && !lower_avoid_p1.is_empty() {
                                    let me_in_low = is_inside_ex(&lower_avoid_p1, &node_position);
                                    let nb_in_low = is_inside_ex(&lower_avoid_p1, &neighbours[0]);
                                    if !me_in_low && nb_in_low {
                                        nb_weight = nb_weight.min(0.2);
                                    } else if me_in_low && !nb_in_low {
                                        nb_weight = nb_weight.max(0.8);
                                    }
                                }
                                let weighted_pos = Point::new(
                                    node_position.x + (nb_weight * (neighbours[0].x - node_position.x) as f64).round() as Coord,
                                    node_position.y + (nb_weight * (neighbours[0].y - node_position.y) as f64).round() as Coord,
                                );
                                if !is_inside_ex(&self.get_collision(0.0, obj_layer_nr_next), &weighted_pos)
                                    && (group_index > 0 || !is_inside_ex(&avoid_layer, &weighted_pos))
                                {
                                    next_position = weighted_pos;
                                }
                            }
                            if group_index == 0 {
                                //Avoid collisions.
                                //100 micron extra for rounding errors.
                                let max_move_between_samples: CoordF = max_move_distance + radius_sample_resolution + EPSILON;
                                move_out_expolys(&avoid_layer, &mut next_position, radius_sample_resolution + EPSILON, max_move_between_samples);
                            }

                            let node_parent: NodeId = if n!(p_node).parent.is_some() && n!(neighbour).parent.is_some() {
                                if n!(p_node).radius >= n!(neighbour).radius {
                                    p_node
                                } else {
                                    neighbour
                                }
                            } else if n!(p_node).parent.is_some() {
                                p_node
                            } else {
                                neighbour
                            };
                            // Make sure the next pass doesn't drop down either of these (since that already happened).
                            let other = if node_parent == p_node { neighbour } else { p_node };
                            n!(node_parent).merged_neighbours.insert(0, other);
                            let to_buildplate = !is_inside_ex(&self.get_collision(0.0, obj_layer_nr_next), &next_position);
                            let (parent_dtt, parent_roof) = (n!(node_parent).distance_to_top, n!(node_parent).support_roof_layers_below);
                            let next_node = self.m_ts_data.create_node(
                                next_position,
                                parent_dtt + 1,
                                obj_layer_nr_next as i32,
                                parent_roof - 1,
                                to_buildplate,
                                Some(node_parent),
                                print_z_next,
                                height_next,
                                0.0,
                                0.0,
                            );
                            let dist_xy_to_node = point_dist(next_position, n!(p_node).position);
                            let dist_xy_to_neighbor = point_dist(next_position, n!(neighbour).position);
                            n!(next_node).orig_pos = if dist_xy_to_neighbor < dist_xy_to_node { n!(p_node).orig_pos } else { n!(neighbour).orig_pos };
                            self.get_max_move_dist(next_node, tan_angle, 1);
                            self.contact_nodes[layer_nr_next].push(next_node);
                            n!(neighbour).valid = false;
                            n!(p_node).valid = false;
                        } else if neighbours.len() > 1 {
                            //Don't merge leaf nodes because we would then incur movement greater than the maximum move distance.
                            //Remove all neighbours that are too close and merge them into this node.
                            for neighbour in &neighbours {
                                if vsize2_with_unscale(Point::new(neighbour.x - node_position.x, neighbour.y - node_position.y))
                                    < self.get_max_move_dist(p_node, tan_angle, 2)
                                {
                                    let neighbour_node = nodes_this_part[neighbour];
                                    if !n!(neighbour_node).valid {
                                        continue;
                                    }
                                    if n!(neighbour_node).type_ == TreeNodeType::EPolygon {
                                        continue;
                                    }
                                    // bug fix: STUDIO-6326, do not merge if current node has smaller radius than neighbor.
                                    if n!(p_node).radius < n!(neighbour_node).radius {
                                        continue;
                                    }
                                    if n!(p_node).valid {
                                        let nb_merged = n!(neighbour_node).merged_neighbours.clone();
                                        n!(p_node).merged_neighbours.insert(0, neighbour_node);
                                        n!(p_node).merged_neighbours.extend(nb_merged);
                                        n!(neighbour_node).valid = false;
                                    }
                                }
                            }
                        }
                    }
                }

                //In the second pass, move all middle nodes.
                for entry in &nodes_vec {
                    let p_node = entry.1;
                    if !n!(p_node).valid {
                        continue;
                    }
                    if n!(p_node).fading {
                        let next_radius: CoordF = n!(p_node).radius - max_move_distance;
                        if next_radius < EPSILON {
                            continue;
                        }
                        let (pos, dtt, roof, tb) =
                            (n!(p_node).position, n!(p_node).distance_to_top, n!(p_node).support_roof_layers_below, n!(p_node).to_buildplate);
                        let next_node = self.m_ts_data.create_node(
                            pos,
                            dtt + 1,
                            obj_layer_nr_next as i32,
                            roof - 1,
                            tb,
                            Some(p_node),
                            print_z_next,
                            height_next,
                            0.0,
                            0.0,
                        );
                        n!(next_node).max_move_dist = 0.0;
                        n!(next_node).radius = next_radius;
                        n!(next_node).fading = true;
                        self.contact_nodes[layer_nr_next].push(next_node);
                        continue;
                    }
                    if n!(p_node).type_ == TreeNodeType::EPolygon {
                        // polygon node do not merge or move
                        if n!(p_node).overhang.contour.points.is_empty() {
                            n!(p_node).valid = false;
                            continue;
                        }
                        let to_buildplate = true;
                        let node_overhang = n!(p_node).overhang.clone();
                        let mut overhangs_next: ExPolygons = vec![node_overhang.clone()];
                        let poly_radius = bbox_radius(&node_overhang.contour.bounding_box());
                        let mut offseted = false;
                        let node_target_radius = n!(p_node).target_radius;
                        // polygon support expansion cases:
                        // 1. bottom expansion is enabled
                        // 2. large tree supports merge into the region
                        if (bottom_expand_enabled && n!(p_node).print_z < do_not_mover_under_mm)
                            || (poly_radius < scale_(node_target_radius) || node_overhang.area() < 3.0 * sq(scale_(node_target_radius)))
                        {
                            // `offset_ex({node.overhang}, ...)`: the overload of one ExPolygon
                            overhangs_next = offset_ex_expolygon(&node_overhang, scale_(max_move_distance / 2.0) as f32);
                            offseted = true;
                        }
                        // keep only the part that won't be removed by the next layer
                        let collision_next = self.get_collision(0.0, obj_layer_nr_next);
                        overhangs_next = diff_clipped(&overhangs_next, &collision_next);
                        let node_dtt = n!(p_node).distance_to_top;
                        if node_dtt == 0 {
                            overhangs_next = offset2_ex(&overhangs_next, scale_(max_move_distance) as f32, -scale_(max_move_distance) as f32);
                            n!(p_node).origin_area = node_overhang.area();
                            densify_polygon(&mut n!(p_node).overhang.contour, 2.0);
                        }
                        let node_roof = n!(p_node).support_roof_layers_below;
                        if num_top_interface_layers > 0
                            && obj_layer_nr_next > 0
                            && node_roof == 1
                            && (node_dtt as i64 as usize) >= num_top_interface_layers
                        {
                            overhangs_next = safe_offset_inc(
                                &overhangs_next,
                                scale_(max_move_distance) as Coord,
                                &collision_next,
                                scale_(TS_MIN_BRANCH_RADIUS * 1.75) as Coord,
                                0,
                                1,
                            );
                        }
                        let node_origin_area = n!(p_node).origin_area;
                        for mut overhang in overhangs_next {
                            if overhang.contour.points.is_empty() {
                                continue;
                            }
                            if overhang.area() > node_origin_area / 2.0 && overhang.area() > sq(scale_(10.0)) {
                                let contour = overhang.contour.clone();
                                let mut movements: HashMap<Point, Point> = HashMap::new();
                                smooth_filter(&overhang.contour, 2, &mut movements, max_move_distance / 2.0);
                                let contour_expolys = vec![ExPolygon::new(contour)];
                                for pt in overhang.contour.points.iter_mut() {
                                    let mv = movements[pt];
                                    let tmp = Point::new(pt.x + mv.x, pt.y + mv.y);
                                    if !is_inside_ex(&contour_expolys, &tmp) && !is_inside_ex(&self.m_ts_data.m_layer_outlines_below[obj_layer_nr], &tmp) {
                                        *pt = tmp;
                                    }
                                }
                            }
                            // if the part would fall straight to th buildplate, shrink it a little
                            if !offseted && node_roof < 0 && overhang.area() > node_origin_area / 2.0 && overhang.area() > sq(scale_(10.0)) {
                                let shrink_overhangs = union_ex(&shrink_ex(
                                    &safe_union(std::slice::from_ref(&overhang), &[]),
                                    scale_(max_move_distance / 2.0) as f32,
                                ));
                                if shrink_overhangs.len() == 1
                                    && shrink_overhangs[0].area() > sq(scale_(10.0))
                                    && !crate::ex_polygon::overlaps_expolys(
                                        std::slice::from_ref(&overhang),
                                        &self.m_ts_data.m_layer_outlines_below[obj_layer_nr_next],
                                    )
                                {
                                    if diff_ex(
                                        std::slice::from_ref(&overhang),
                                        &offset_ex_expolygon(&shrink_overhangs[0], scale_(max_move_distance) as f32),
                                    )
                                    .is_empty()
                                    {
                                        overhang = shrink_overhangs[0].clone();
                                    }
                                }
                            }
                            // the two arms of the C++ differ in the order of the assignments only
                            let next_pt = polygon_centroid(&overhang.contour);
                            let next_node = self.m_ts_data.create_node(
                                next_pt,
                                node_dtt + 1,
                                obj_layer_nr_next as i32,
                                node_roof - 1,
                                to_buildplate,
                                Some(p_node),
                                print_z_next,
                                height_next,
                                0.0,
                                0.0,
                            );
                            n!(next_node).max_move_dist = 0.0;
                            n!(next_node).overhang = overhang;
                            n!(next_node).target_radius = node_target_radius;
                            n!(next_node).origin_area = node_origin_area;
                            self.contact_nodes[layer_nr_next].push(next_node);
                        }
                        continue;
                    }

                    let node_position = n!(p_node).position;
                    //If the branch falls completely inside a collision area (the entire branch would be removed by the X/Y offset), delete it.
                    if group_index > 0 && is_inside_ex(&self.get_collision(0.0, obj_layer_nr), &node_position) {
                        let branch_radius_node: CoordF = self.get_radius(p_node);
                        let to_outside = projection_onto(&self.get_collision(0.0, obj_layer_nr), &node_position);
                        let dist2_to_outside = vsize2_with_unscale(Point::new(node_position.x - to_outside.x, node_position.y - to_outside.y));
                        if dist2_to_outside >= branch_radius_node * branch_radius_node {
                            //Too far inside.
                            if support_on_buildplate_only {
                                unsupported_branch_leaves.push_front((layer_nr, p_node));
                            } else {
                                n!(p_node).valid = false;
                            }
                            continue;
                        }
                        // if the link between parent and current is cut by contours, mark current as bottom contact node
                        if let Some(parent) = n!(p_node).parent {
                            if !intersection_ln(&Line::new(n!(p_node).position, n!(parent).position), &layer_contours).is_empty() {
                                n!(p_node).valid = false;
                                continue;
                            }
                        }
                    }
                    let mut next_layer_vertex = node_position;
                    let mut move_to_neighbor_center = Point::new(0, 0);
                    let mut move_to_neighbor_center_base = Point::new(0, 0);

                    let neighbours: Vec<Point> = mst.adjacent_nodes(node_position);
                    // 1. do not merge neighbors under 5mm
                    // 2. Only merge node with single neighbor in distance between [max_move_distance, 10mm/layer_height]
                    let dist2_to_first_neighbor: f32 = if neighbours.is_empty() {
                        0.0
                    } else {
                        vsize2_with_unscale(Point::new(neighbours[0].x - node_position.x, neighbours[0].y - node_position.y)) as f32
                    };
                    let node_print_z = n!(p_node).print_z;
                    let node_dist_mm_to_top = n!(p_node).dist_mm_to_top;
                    // Only nodes that aren't about to collapse.
                    if node_print_z > do_not_mover_under_mm
                        && (neighbours.len() > 1
                            || (neighbours.len() == 1 && dist2_to_first_neighbor as f64 >= self.get_max_move_dist(p_node, tan_angle, 2)))
                    {
                        // Move towards the average position of all neighbours, weighted by carried_mass
                        // to encourage light branches to move towards heavy trunks and suppress heavy trunks
                        // from deflecting towards light branches.
                        let self_mass: f64 = (n!(p_node).total_carried_mass(&self.m_ts_data.contact_nodes) as f64).max(EPSILON);
                        let mut max_nb_mass: f64 = 0.0;
                        let prev_move = match n!(p_node).parent {
                            Some(parent) => Point::new(node_position.x - n!(parent).position.x, node_position.y - n!(parent).position.y),
                            None => Point::new(0, 0),
                        };
                        let prev_move_d = (prev_move.x as f64, prev_move.y as f64);
                        let has_prev_move = prev_move_d.0 * prev_move_d.0 + prev_move_d.1 * prev_move_d.1 > scale_(0.01) * scale_(0.01);
                        // Direction towards the nearest edge of the model outlines below (for group_index == 0).
                        // Pulling a branch deeper over a lower model part forces it to either reverse outward later
                        // (elbow bend) or land on the model top.
                        let mut outlines_below_out_d = (0.0_f64, 0.0_f64);
                        let mut in_outlines_below = false;
                        if group_index == 0 && !parts.is_empty() && !lower_avoid_p1.is_empty() && is_inside_ex(&parts, &node_position) {
                            let proj_out = projection_onto(&parts, &node_position);
                            outlines_below_out_d = ((proj_out.x - node_position.x) as f64, (proj_out.y - node_position.y) as f64);
                            in_outlines_below = outlines_below_out_d.0 * outlines_below_out_d.0 + outlines_below_out_d.1 * outlines_below_out_d.1
                                > scale_(0.2) * scale_(0.2);
                        }
                        let mut base_dir_d = (0.0_f64, 0.0_f64);
                        let mut sum_dir_d = (0.0_f64, 0.0_f64);
                        for neighbour in &neighbours {
                            // do not move to the neighbor to be deleted
                            let neighbour_node = nodes_this_part[neighbour];
                            if !n!(neighbour_node).valid {
                                continue;
                            }
                            let direction: Point;
                            if n!(neighbour_node).type_ == TreeNodeType::EPolygon && n!(neighbour_node).overhang.is_valid() {
                                let contact_point = projection_onto(std::slice::from_ref(&n!(neighbour_node).overhang), &node_position);
                                direction = Point::new(contact_point.x - node_position.x, contact_point.y - node_position.y);
                            } else {
                                direction = Point::new(neighbour.x - node_position.x, neighbour.y - node_position.y);
                            }
                            // do not move to neighbor that's too far away
                            let dist2_to_neighbor: f32 = vsize2_with_unscale(direction) as f32;

                            let branch_bottom_radius: CoordF = self.calc_radius(node_dist_mm_to_top + node_print_z);
                            let neighbour_bottom_radius: CoordF = self.calc_radius(n!(neighbour_node).dist_mm_to_top + n!(neighbour_node).print_z);
                            let max_converge_distance: f64 =
                                tan_angle.mul_add(node_print_z - do_not_mover_under_mm, branch_bottom_radius.max(neighbour_bottom_radius));
                            if dist2_to_neighbor as f64 > max_converge_distance * max_converge_distance {
                                continue;
                            }

                            if is_line_cut_by_contour(&mut mst_line_x_layer_contour_cache, &layer_contours, node_position, *neighbour) {
                                continue;
                            }

                            let mut mass_weight: f64;
                            let nb_mass: f64 = (n!(neighbour_node).total_carried_mass(&self.m_ts_data.contact_nodes) as f64).max(EPSILON);
                            max_nb_mass = max_nb_mass.max(nb_mass);
                            {
                                let dir_d = (direction.x as f64, direction.y as f64);
                                let reverses_prev = has_prev_move && dir_d.0 * prev_move_d.0 + dir_d.1 * prev_move_d.1 < 0.0;
                                if nb_mass >= self_mass {
                                    mass_weight = (2.0 * nb_mass / (self_mass + nb_mass)).sqrt();
                                } else if reverses_prev {
                                    mass_weight = (2.0 * nb_mass / (self_mass + nb_mass)).sqrt();
                                } else {
                                    mass_weight = 1.0;
                                }
                                if group_index == 0 && !lower_avoid_p1.is_empty() {
                                    let me_in_low = is_inside_ex(&lower_avoid_p1, &node_position);
                                    let nb_in_low = is_inside_ex(&lower_avoid_p1, neighbour);
                                    if !me_in_low && nb_in_low {
                                        mass_weight *= 0.25;
                                    } else if me_in_low && !nb_in_low {
                                        mass_weight = mass_weight.max(1.4);
                                    } else if me_in_low && nb_in_low && in_outlines_below {
                                        if dir_d.0 * outlines_below_out_d.0 + dir_d.1 * outlines_below_out_d.1 > 0.0 {
                                            mass_weight = mass_weight.max(1.3);
                                        } else {
                                            mass_weight *= 0.4;
                                        }
                                    }
                                }
                            }

                            if !is_strong {
                                let w = mass_weight / dist2_to_neighbor as f64;
                                sum_dir_d = (sum_dir_d.0 + direction.x as f64 * w, sum_dir_d.1 + direction.y as f64 * w);
                                let wb = 1.0 / dist2_to_neighbor as f64;
                                base_dir_d = (base_dir_d.0 + direction.x as f64 * wb, base_dir_d.1 + direction.y as f64 * wb);
                            } else {
                                sum_dir_d = (sum_dir_d.0 + direction.x as f64 * mass_weight, sum_dir_d.1 + direction.y as f64 * mass_weight);
                                base_dir_d = (base_dir_d.0 + direction.x as f64, base_dir_d.1 + direction.y as f64);
                            }
                        }

                        let sum_direction = Point::new(sum_dir_d.0.round() as Coord, sum_dir_d.1.round() as Coord);
                        let base_direction = Point::new(base_dir_d.0.round() as Coord, base_dir_d.1.round() as Coord);
                        if !is_strong {
                            move_to_neighbor_center = sum_direction;
                            move_to_neighbor_center_base = base_direction;
                        } else {
                            if vsize2_with_unscale(sum_direction) <= self.get_max_move_dist(p_node, tan_angle, 2) {
                                move_to_neighbor_center = sum_direction;
                            } else {
                                move_to_neighbor_center = normal(sum_direction, scale_(self.get_max_move_dist(p_node, tan_angle, 1)));
                            }
                            if vsize2_with_unscale(base_direction) <= self.get_max_move_dist(p_node, tan_angle, 2) {
                                move_to_neighbor_center_base = base_direction;
                            } else {
                                move_to_neighbor_center_base = normal(base_direction, scale_(self.get_max_move_dist(p_node, tan_angle, 1)));
                            }
                        }
                        // When all reachable neighbours carry less mass than this node and pull it opposite to its
                        // trajectory (or deeper over lower model outlines), cap the attraction step by mass ratio.
                        if max_nb_mass > 0.0 && max_nb_mass < self_mass {
                            let move_d = (move_to_neighbor_center.x as f64, move_to_neighbor_center.y as f64);
                            let rev_move = (has_prev_move && move_d.0 * prev_move_d.0 + move_d.1 * prev_move_d.1 < 0.0)
                                || (in_outlines_below && move_d.0 * outlines_below_out_d.0 + move_d.1 * outlines_below_out_d.1 < 0.0);
                            if rev_move {
                                let attract_scale: f64 = if support_on_buildplate_only || is_strong { 1.0 } else { 2.0 };
                                let mass_factor: f64 = std_clamp(2.0 * max_nb_mass / (self_mass + max_nb_mass), 0.15, 1.0);
                                let max_attract: f64 = (self.get_max_move_dist(p_node, tan_angle, 1) * mass_factor) / attract_scale;
                                if vsize2_with_unscale(move_to_neighbor_center) > sq(max_attract) {
                                    move_to_neighbor_center = normal(move_to_neighbor_center, scale_(max_attract));
                                }
                            }
                        }
                    }

                    let next_radius: CoordF = self.calc_radius(node_dist_mm_to_top + height_next);
                    let avoidance_next = self.get_avoidance(next_radius, obj_layer_nr_next);

                    let mut to_outside = projection_onto(&avoidance_next, &node_position);
                    let mut direction_to_outer = Point::new(to_outside.x - node_position.x, to_outside.y - node_position.y);
                    let skin_direction = n!(p_node).skin_direction;
                    if skin_direction != Point::new(0, 0) && node_dist_mm_to_top < 3.0 {
                        move_to_neighbor_center = normal(skin_direction, scale_(max_move_distance));
                        direction_to_outer = move_to_neighbor_center;
                    }
                    let mut dist2_to_outer: f64 = vsize2_with_unscale(direction_to_outer);
                    // don't move if
                    // 1) line of node and to_outside is cut by contour (means supports may intersect with object)
                    // 2) it's impossible to move to build plate
                    if is_line_cut_by_contour(&mut mst_line_x_layer_contour_cache, &layer_contours, node_position, to_outside)
                        || dist2_to_outer > max_move_distance2 * (obj_layer_nr * obj_layer_nr) as f64
                        || !is_inside_ex(&avoidance_next, &node_position)
                    {
                        // try move to outside of lower layer instead
                        let mut candidate_vertex = node_position;
                        // 100 micron extra for rounding errors.
                        let max_move_between_samples: CoordF = max_move_distance + radius_sample_resolution + EPSILON;
                        // use get_collision instead of get_avoidance here (See STUDIO-4252)
                        let is_outside = move_out_expolys(
                            &self.get_collision(next_radius, obj_layer_nr_next),
                            &mut candidate_vertex,
                            max_move_between_samples,
                            max_move_between_samples,
                        );
                        if is_outside {
                            direction_to_outer = Point::new(candidate_vertex.x - node_position.x, candidate_vertex.y - node_position.y);
                            dist2_to_outer = vsize2_with_unscale(direction_to_outer);
                        } else {
                            direction_to_outer = Point::new(0, 0);
                            dist2_to_outer = 0.0;
                        }
                    }
                    // move to the averaged direction of neighbor center and contour edge if they are roughly same direction
                    if dist2_to_outer > EPSILON && move_to_neighbor_center_base != Point::new(0, 0) {
                        // When escaping an obstacle or avoidance zone, keep the unweighted attraction magnitude
                        // so carried_mass scaling never alters the escape-vs-attraction vector balance.
                        move_to_neighbor_center = move_to_neighbor_center_base;
                    }
                    let mut movement: Point;
                    if support_on_buildplate_only {
                        // `Point * 2`: Eigen's integer product
                        movement = Point::new(move_to_neighbor_center.x + direction_to_outer.x * 2, move_to_neighbor_center.y + direction_to_outer.y * 2);
                    } else if !is_strong {
                        // `direction_to_outer * (1 / dist2_to_outer)`: the member operator* of a non-const Point (lrint)
                        let outer = if dist2_to_outer > EPSILON { point_mul(direction_to_outer, 1.0 / dist2_to_outer) } else { Point::new(0, 0) };
                        movement = Point::new(move_to_neighbor_center.x * 2 + outer.x, move_to_neighbor_center.y * 2 + outer.y);
                    } else {
                        // `movement` is a default constructed Point here, (0, 0): its dot product is 0
                        movement = Point::new(0, 0);
                        let dot = movement.x * move_to_neighbor_center.x + movement.y * move_to_neighbor_center.y;
                        if dot as f64 >= 0.2 || move_to_neighbor_center == Point::new(0, 0) {
                            movement = Point::new(direction_to_outer.x + move_to_neighbor_center.x, direction_to_outer.y + move_to_neighbor_center.y);
                        } else {
                            movement = move_to_neighbor_center; // otherwise move to neighbor center first
                        }
                    }

                    if vsize2_with_unscale(movement) > self.get_max_move_dist(p_node, tan_angle, 2) {
                        movement = normal(movement, scale_(self.get_max_move_dist(p_node, tan_angle, 1)));
                    }

                    next_layer_vertex = Point::new(next_layer_vertex.x + movement.x, next_layer_vertex.y + movement.y);

                    // `if (group_index == 0 && 0)`: never taken

                    let next_collision = self.get_collision(0.0, obj_layer_nr_next);
                    let to_buildplate = !is_inside_ex(&self.m_ts_data.m_layer_outlines[obj_layer_nr_next], &next_layer_vertex);
                    let (node_dtt, node_roof) = (n!(p_node).distance_to_top, n!(p_node).support_roof_layers_below);
                    let next_node = self.m_ts_data.create_node(
                        next_layer_vertex,
                        node_dtt + 1,
                        obj_layer_nr_next as i32,
                        node_roof - 1,
                        to_buildplate,
                        Some(p_node),
                        print_z_next,
                        height_next,
                        0.0,
                        0.0,
                    );
                    // don't increase radius if next node will collide partially with the object (STUDIO-7883)
                    to_outside = projection_onto(&next_collision, &n!(next_node).position);
                    direction_to_outer = Point::new(to_outside.x - node_position.x, to_outside.y - node_position.y);
                    let dist_to_outer: f64 = point_norm(direction_to_outer) * SCALING_FACTOR;
                    let node_radius = n!(p_node).radius;
                    let (next_print_z, next_height, next_radius_now) = (n!(next_node).print_z, n!(next_node).height, n!(next_node).radius);
                    n!(next_node).radius =
                        if bottom_expand_enabled && next_print_z < do_not_mover_under_mm && node_dist_mm_to_top > do_not_mover_under_mm {
                            node_radius + self.support_flare_step_width(next_print_z, next_height) / 2.0
                        } else {
                            node_radius.max(next_radius_now.min(dist_to_outer))
                        };
                    self.get_max_move_dist(next_node, tan_angle, 1);
                    self.contact_nodes[layer_nr_next].push(next_node);
                }

                // Sort contact_nodes for deterministic processing in the next layer
                {
                    let nodes = &self.m_ts_data.contact_nodes;
                    crate::clipper::libcxx_sort(&mut self.contact_nodes[layer_nr_next], |a: &NodeId, b: &NodeId| {
                        let (pa, pb) = (nodes[*a].position, nodes[*b].position);
                        if pa.x != pb.x {
                            return pa.x < pb.x;
                        }
                        pa.y < pb.y
                    });
                }
            }

            // BBS multi-diameter: a slab is one extrusion printed at the cross-section of its top face, so a
            // branch that drifts while crossing the slab ends up somewhere the slab below does not carry it.
            // Cap the drift a chain may accumulate between two slab tops by what the slab's own height allows.
            if self.m_support_merge_plan.enabled && layer_nr_next < self.m_ts_data.layer_heights.len() && !self.contact_nodes[layer_nr_next].is_empty() {
                let obj_layer_next = self.m_ts_data.layer_heights[layer_nr_next].obj_layer_nr;
                let slab = self.m_support_merge_plan.slab_for_layer(obj_layer_next).map(|s| (s.top_idx, s.height));
                // Nothing to cap on the slab's own top layer: that face is where the budget is handed out.
                if let Some((slab_top_idx, slab_height)) = slab {
                    if obj_layer_next != slab_top_idx && slab_height > EPSILON {
                        let budget_scaled: f64 = scale_(tan_angle * slab_height);
                        for &node in &self.contact_nodes[layer_nr_next] {
                            if n!(node).type_ == TreeNodeType::EPolygon || n!(node).parent.is_none() {
                                continue;
                            }
                            let mut anchor = node;
                            while n!(anchor).parent.is_some() && (n!(anchor).obj_layer_nr as usize) < slab_top_idx {
                                anchor = n!(anchor).parent.unwrap();
                            }
                            if anchor == node {
                                continue;
                            }
                            let delta = ((n!(node).position.x - n!(anchor).position.x) as f64, (n!(node).position.y - n!(anchor).position.y) as f64);
                            let len = (delta.0 * delta.0 + delta.1 * delta.1).sqrt();
                            if len > budget_scaled && len > EPSILON {
                                let capped = (delta.0 * (budget_scaled / len), delta.1 * (budget_scaled / len));
                                let ap = n!(anchor).position;
                                n!(node).position = Point::new(ap.x + capped.0 as Coord, ap.y + capped.1 as Coord);
                            }
                        }
                    }
                }
            }

            if layer_nr_next == 0 && support_on_buildplate_only && !self.contact_nodes[layer_nr_next].is_empty() {
                for &node in &self.contact_nodes[layer_nr_next] {
                    if !n!(node).to_buildplate {
                        unsupported_branch_leaves.push_front((layer_nr_next, node));
                    }
                }
            }

            // Prune all branches that couldn't find support on either the model or the buildplate (resulting in 'mid-air' branches).
            while !unsupported_branch_leaves.is_empty() {
                let entry = *unsupported_branch_leaves.back().unwrap();
                let mut i_node: Option<NodeId> = Some(entry.1);
                while let Some(i) = i_node {
                    let i_layer: usize = n!(i).obj_layer_nr as usize;
                    // update the parent-child chain
                    let i_child = n!(i).child;
                    let i_parent = n!(i).parent;
                    if let Some(parent) = i_parent {
                        n!(parent).child = i_child;
                        let parents = n!(i).parents.clone();
                        for parent in parents {
                            if n!(parent).child == Some(i) {
                                n!(parent).child = i_child;
                            }
                        }
                    }
                    if let Some(child) = i_child {
                        n!(child).parent = i_parent;
                        if let Some(pos) = n!(child).parents.iter().position(|&p| p == i) {
                            n!(child).parents.remove(pos);
                        }
                        if !n!(i).parents.is_empty() {
                            let parents = n!(i).parents.clone();
                            n!(child).parents.extend(parents);
                        }
                    }
                    n!(i).is_processed = true; // mark to be deleted later
                    let merged = n!(i).merged_neighbours.clone();
                    for neighbour in merged {
                        if !n!(neighbour).is_processed {
                            unsupported_branch_leaves.push_front((i_layer, neighbour));
                        }
                    }
                    i_node = n!(i).parent;
                }
                unsupported_branch_leaves.pop_back();
            }

            {
                let nodes = &self.m_ts_data.contact_nodes;
                for layer_contact_nodes in self.contact_nodes.iter_mut() {
                    if !layer_contact_nodes.is_empty() {
                        layer_contact_nodes.retain(|&node| !nodes[node].is_processed);
                    }
                }
            }

            if !self.contact_nodes.is_empty() {
                // make sure r[i - 1] - r[i] <= line_width
                let a = max_move_distance + TS_MIN_BRANCH_RADIUS;
                let radius_stable_threshold: f64 =
                    (max_move_distance + max_move_distance) / a.mul_add(a, -(TS_MIN_BRANCH_RADIUS * TS_MIN_BRANCH_RADIUS));
                let layer0 = self.contact_nodes[0].clone();
                for node0 in layer0 {
                    let mut trunk_top = node0;
                    while n!(trunk_top).parent.is_some() && n!(trunk_top).parents.len() <= 1 {
                        trunk_top = n!(trunk_top).parent.unwrap();
                    }
                    let mut node: Option<NodeId> = Some(node0);
                    while let Some(nd) = node {
                        let ref_pos = n!(trunk_top).position;
                        let stable_radius: f64 =
                            (point_dist(n!(nd).position, ref_pos).mul_add(SCALING_FACTOR, n!(nd).dist_mm_to_top) / radius_stable_threshold).sqrt();
                        if n!(nd).radius >= stable_radius {
                            break;
                        }
                        n!(nd).radius = stable_radius;
                        if n!(nd).parent.is_none() || n!(nd).parents.len() > 1 {
                            break;
                        }
                        node = n!(nd).parent;
                    }
                }
            }
        }
    }
}

// ============================================================================
// TreeSupport::draw_circles (TreeSupport.cpp:3905-4607) and resolve_material_covered_by_folded_interfaces
// (TreeSupport.cpp:4635-4901)
// ============================================================================

/// TreeSupport.cpp:3957-3961 `struct WidenedArea` of draw_circles(). BBS multi-diameter: disk of a node whose
/// wall is widened at a nozzle change.
#[derive(Debug, Clone)]
struct WidenedArea {
    area: ExPolygon,
    bbox: BoundingBox,
    width: CoordF,
}

/// TreeSupport.cpp:3964-3977 `struct LayerDrawData` of draw_circles(). BBS multi-diameter: the per layer state
/// handed from drawing the nodes to finishing the areas.
#[derive(Debug, Clone, Default)]
struct LayerDrawData {
    obj_layer_nr: usize,
    max_layers_above_base: CoordF,
    max_layers_above_roof: CoordF,
    max_layers_above_roof1: CoordF,
    interface_id: i32,
    collision_base: ExPolygons,
    area_poly: ExPolygons,
    extra_wall_area: ExPolygons,
    cooldown_area: ExPolygons,
    // Sorted by width, widest first.
    widened_areas: Vec<WidenedArea>,
}

/// The collision areas the lambdas `get_collision` and `get_node_collision` of draw_circles() cache per layer
/// (TreeSupport.cpp:4015-4016, 4037-4038).
#[derive(Default)]
struct DrawCirclesCollision {
    collision_sharp_tails: ExPolygons,
    collision_base: ExPolygons,
    collision_sharp_tails_span: ExPolygons,
    collision_base_span: ExPolygons,
}

/// Which of the cached areas of `DrawCirclesCollision` a lambda returned a reference to.
#[derive(Clone, Copy, PartialEq, Eq)]
enum DrawCirclesCollisionSlot {
    SharpTails,
    Base,
    SharpTailsSpan,
    BaseSpan,
}

impl DrawCirclesCollision {
    fn get(&self, slot: DrawCirclesCollisionSlot) -> &ExPolygons {
        match slot {
            DrawCirclesCollisionSlot::SharpTails => &self.collision_sharp_tails,
            DrawCirclesCollisionSlot::Base => &self.collision_base,
            DrawCirclesCollisionSlot::SharpTailsSpan => &self.collision_sharp_tails_span,
            DrawCirclesCollisionSlot::BaseSpan => &self.collision_base_span,
        }
    }
}

/// `FillLightning::Generator(PrintObject *m_object, std::vector<Polygons> &contours, std::vector<Polygons> &overhangs,
/// const std::function<void()> &throw_on_cancel_callback, float density)` (Fill/Lightning/Generator.cpp:100-143), the
/// constructor draw_circles() makes `TreeSupport::generator` with. The crate's `fill::lightning::generator::Generator`
/// has no constructor yet (its `generateTreesforSupport` waits on Fill/Lightning/Layer.cpp), so the caller hands it in.
pub trait LightningSupportGenerator {
    fn generate(&self, object: &PrintObject, print: &Print, contours: &mut Vec<Polygons>, overhangs: &mut Vec<Polygons>, density: f32)
        -> crate::fill::lightning::generator::Generator;
}

// `std::unordered_map<double, size_t>`: the key of a double; 0.0 and -0.0 are one key.
#[inline]
fn f64_key(v: f64) -> u64 {
    if v == 0.0 {
        0
    } else {
        v.to_bits()
    }
}

// `config.<key>.values` of a floats option.
fn cfg_floats(config: &DynamicPrintConfig, key: &str) -> Vec<f64> {
    match config.option(key) {
        Some(opt) => {
            if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionFloats>() {
                o.values.clone()
            } else if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionFloatsNullable>() {
                o.values.clone()
            } else {
                Vec::new()
            }
        }
        None => Vec::new(),
    }
}

impl<'a> TreeSupport<'a> {
    // TreeSupport.cpp:4017-4029 the lambda `get_collision` of draw_circles(): fills the area of the layer when it is
    // empty and tells which one it is.
    fn draw_circles_get_collision(&self, c: &mut DrawCirclesCollision, obj_layer_nr: usize, sharp_tail: bool) -> DrawCirclesCollisionSlot {
        let empty = if sharp_tail { c.collision_sharp_tails.is_empty() } else { c.collision_base.is_empty() };
        if empty {
            let delta: f64 = if sharp_tail {
                scale_(self.top_z_at(obj_layer_nr) as f64)
            } else {
                scale_(if obj_layer_nr == 0 { cfg_float(self.object_config(), "support_object_first_layer_gap") } else { self.m_ts_data.m_xy_distance })
            };
            let mut collision =
                cx::offset_ex(&self.m_ts_data.m_layer_outlines[obj_layer_nr], delta as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
            // the top layers may be too close to interface with adaptive layer heights and very small overhang angle.
            // On the object first layer, clamp that expansion so a smaller support_object_first_layer_gap
            // is not cancelled. Sharp tails keep the top-z clearance.
            self.union_outlines_within_top_z(&mut collision, obj_layer_nr, !sharp_tail && obj_layer_nr == 0);
            if sharp_tail {
                c.collision_sharp_tails = collision;
            } else {
                c.collision_base = collision;
            }
        }
        if sharp_tail {
            DrawCirclesCollisionSlot::SharpTails
        } else {
            DrawCirclesCollisionSlot::Base
        }
    }

    // TreeSupport.cpp:4039-4053 the lambda `get_node_collision` of draw_circles().
    //
    // BBS multi-diameter: fold_nodes_multi_diameter() may have made this layer the carrier of a slab. A slab is one
    // extrusion over its whole Z span, so it has to clear the object everywhere in that span and not only at the
    // face its cross-section was taken from. Only the folded nodes get that treatment.
    fn draw_circles_get_node_collision(
        &self,
        c: &mut DrawCirclesCollision,
        obj_layer_nr: usize,
        node: &SupportNode,
        sharp_tail: bool,
    ) -> DrawCirclesCollisionSlot {
        if node.slab_height <= EPSILON || node.slab_bottom_obj_layer >= obj_layer_nr {
            return self.draw_circles_get_collision(c, obj_layer_nr, sharp_tail);
        }
        let empty = if sharp_tail { c.collision_sharp_tails_span.is_empty() } else { c.collision_base_span.is_empty() };
        if empty {
            let slot = self.draw_circles_get_collision(c, obj_layer_nr, sharp_tail);
            let mut collision: ExPolygons = c.get(slot).clone();
            for layer_id in node.slab_bottom_obj_layer..obj_layer_nr {
                let delta: f64 = if sharp_tail {
                    scale_(self.top_z_at(layer_id) as f64)
                } else {
                    scale_(if layer_id == 0 { cfg_float(self.object_config(), "support_object_first_layer_gap") } else { self.m_ts_data.m_xy_distance })
                };
                collision = cx::union_ex_2(
                    &collision,
                    &cx::offset_ex(&self.m_ts_data.m_layer_outlines[layer_id], delta as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                );
            }
            if sharp_tail {
                c.collision_sharp_tails_span = collision;
            } else {
                c.collision_base_span = collision;
            }
        }
        if sharp_tail {
            DrawCirclesCollisionSlot::SharpTailsSpan
        } else {
            DrawCirclesCollisionSlot::BaseSpan
        }
    }

    /// TreeSupport.cpp:3905-4607 `void TreeSupport::draw_circles()`
    ///
    /// FIDELITY-NOTE: the two `tbb::parallel_for` over the layers write per layer state only; here the layers are
    /// taken in order. `print->objects()` is `m_print.objects()` and the object under work, which the caller holds
    /// apart from the Print (the brim and skirt tests are an any_of and a maximum, so counting it twice is the same).
    /// `holePropagationInfos` is keyed by the address of a hole (`const Polygon*`), which a later `push_back` into
    /// the same `holes` vector invalidates when the vector grows; here the key is the hole's position and the
    /// generation of the vector's buffer (libc++ doubles the capacity), so a pointer into a freed buffer never
    /// matches, where in C++ it would if the allocator handed that address out again to another vector of holes.
    /// The lightning generator is made by `lightning_support_generator` when the caller set one, see
    /// `LightningSupportGenerator`; its inputs are kept in `lightning_contours` / `lightning_overhangs` /
    /// `lightning_support_density`.
    pub fn draw_circles(&mut self) {
        use crate::print::faithful::{SupportLayerAreaGroup, SupportLayerAreaType};
        let has_brim = self.m_print.has_brim() || self.m_object.has_brim();
        let bottom_gap_layers: i32 = (self.m_slicing_params.gap_object_support / self.m_slicing_params.layer_height).round() as i32;
        let branch_radius: CoordF = cfg_float(self.object_config(), "tree_support_branch_diameter") / 2.0;
        let branch_radius_scaled: CoordF = scale_(branch_radius);
        let on_buildplate_only = cfg_bool(self.object_config(), "support_on_build_plate_only");
        //Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
        let mut branch_circle = Polygon::new();

        let z_overrides = cfg_bool(self.m_print.config(), "top_z_overrides_xy_distance");

        // Use square support if there are too many nodes per layer because circle support needs much longer time to compute
        // Hower circle support can be printed faster, so we prefer circle for fewer nodes case.
        let square_support = self.avg_node_per_layer > 200;
        // The number of vertices in each circle.
        let circle_resolution: i32 = if square_support { 4 } else { 25 };

        for i in 0..circle_resolution {
            let angle: f64 = if square_support {
                // `(double) i / CIRCLE_RESOLUTION * TAU + PI / 4.0 + nodes_angle`: the product is contracted with the first sum
                (i as f64 / circle_resolution as f64).mul_add(TAU, M_PI / 4.0) + self.nodes_angle as f64
            } else {
                i as f64 / circle_resolution as f64 * TAU
            };
            // cos(angle) and sin(angle) of one argument: one `__sincos_stret` call
            let (sin_a, cos_a) = crate::clipper::sin_cos(angle);
            branch_circle.points.push(point_from_f64(cos_a * branch_radius_scaled, sin_a * branch_radius_scaled));
        }

        // Performance optimization. Only generate lslices for brim and skirt.
        let mut brim_skirt_layers: usize = if has_brim { 1 } else { 0 };
        {
            let print_config = self.m_print.config();
            let has_infinite_skirt = self.m_print.has_infinite_skirt();
            let skirt_height = cfg_int(print_config, "skirt_height") as i64 as usize;
            for object in self.m_print.objects().iter().chain(std::iter::once(&*self.m_object)) {
                let skirt_layers: usize = if has_infinite_skirt { object.layer_count() } else { skirt_height.min(object.layer_count()) };
                brim_skirt_layers = brim_skirt_layers.max(skirt_layers);
            }
        }

        // generate areas
        let top_interface_layers: usize = cfg_int(self.object_config(), "support_interface_top_layers") as i64 as usize;
        let bottom_interface_layers: usize = 0; //config.support_interface_bottom_layers.value;
        let with_lightning_infill = self.m_support_params.base_fill_pattern == crate::print_config::InfillPattern::Lightning;
        let support_extrusion_width: CoordF = self.m_support_params.support_extrusion_width;
        let line_width_scaled: CoordF = scale_(support_extrusion_width);
        let tree_brim_width: f32 = cfg_float(self.object_config(), "raft_first_layer_expansion") as f32;
        let tree_support_wall_count = cfg_int(self.object_config(), "tree_support_wall_count");
        let bottom_expand_enabled = tree_support_wall_count > 1 || tree_support_wall_count < 0;
        let do_not_mover_under_mm: f64 = self.do_not_mover_under_mm as f64;
        let num_top_interface_layers: usize = self.m_support_params.num_top_interface_layers;
        let support_speed0: f64 = cfg_floats(self.object_config(), "support_speed").first().copied().unwrap_or(0.0);

        if self.m_object.support_layer_count() <= self.m_raft_layers {
            return;
        }

        let n_layers = self.m_ts_data.layer_heights.len();
        let mut layer_draw_data: Vec<LayerDrawData> = vec![LayerDrawData::default(); n_layers];

        for layer_nr in 0..n_layers {
            if (self.canceled)() {
                break;
            }

            let ts_idx = layer_nr + self.m_raft_layers;

            // skip if current layer has no points. This fixes potential crash in get_collision (see jira BBL001-355)
            if self.contact_nodes[layer_nr].is_empty() {
                let ts_layer = &mut self.m_object.m_support_layers[ts_idx];
                ts_layer.layer.print_z = 0.0;
                ts_layer.layer.height = 0.0;
                continue;
            }

            let obj_layer_nr: usize = self.m_ts_data.layer_heights[layer_nr].obj_layer_nr;
            {
                let (print_z, height) = (self.m_ts_data.layer_heights[layer_nr].print_z, self.m_ts_data.layer_heights[layer_nr].height);
                let ts_layer = &mut self.m_object.m_support_layers[ts_idx];
                ts_layer.layer.print_z = print_z;
                ts_layer.layer.height = height;
                if ts_layer.layer.height < EPSILON {
                    continue;
                }
            }

            // `ExPolygons& base_areas = ts_layer->base_areas;` and the like: taken out of the layer, put back below
            let mut base_areas: ExPolygons = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].base_areas);
            let mut roof_areas: ExPolygons = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].roof_areas);
            let mut roof_1st_layer: ExPolygons = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].roof_1st_layer);
            let mut roof_gap_areas: ExPolygons = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].roof_gap_areas);
            let mut max_layers_above_base: CoordF = 0.0;
            let mut max_layers_above_roof: CoordF = 0.0;
            let mut max_layers_above_roof1: CoordF = 0.0;
            let mut interface_id: i32 = 0;
            let mut _has_circle_node = false;
            let mut collision = DrawCirclesCollision::default();
            // A folded interface is one extrusion over the slab span as well. Its areas come from the overhang
            // and skip the node collision, so the span has to be cleared when the roof segments are joined.
            let mut roof_span_node: Option<NodeId> = None;

            //Draw the support areas and add the roofs appropriately to the support roof instead of normal areas.
            let mut area_poly: ExPolygons = Vec::new(); // the polygon node area which will be printed as normal support
            let mut extra_wall_area: ExPolygons = Vec::new(); //where nodes would have extra walls
            let mut cooldown_area: ExPolygons = Vec::new();
            let mut widened_areas: Vec<WidenedArea> = Vec::new();
            let curr_layer_nodes: Vec<NodeId> = self.contact_nodes[layer_nr].clone();
            for &p_node in &curr_layer_nodes {
                if (self.canceled)() {
                    break;
                }

                let node: &SupportNode = &self.m_ts_data.contact_nodes[p_node];
                let mut area: ExPolygons = Vec::new();
                let mut brim_width: f64 = tree_brim_width as f64;
                // Generate directly from overhang polygon if one of the following is true:
                // 1) node is a normal part of hybrid support
                // 2) node is virtual
                if node.type_ == TreeNodeType::EPolygon || (node.distance_to_top < 0 && !node.is_sharp_tail) {
                    // `offset_ex({node.overhang}, ...)`: the overload of one ExPolygon
                    area = cx::offset_ex_expolygon(
                        &node.overhang,
                        scale_(self.m_ts_data.m_xy_distance) as f32,
                        cx::DEFAULT_JOIN_TYPE,
                        cx::DEFAULT_MITER_LIMIT,
                    );
                    let slot = self.draw_circles_get_node_collision(&mut collision, obj_layer_nr, node, node.is_sharp_tail && node.distance_to_top <= 0);
                    area = cx::diff_clipped(&area, collision.get(slot));
                    if node.type_ == TreeNodeType::EPolygon {
                        append(&mut area_poly, area.clone());
                        if node.need_extra_wall {
                            append(&mut extra_wall_area, area.clone());
                        }
                    }
                } else {
                    // merge overhang to get a smoother interface surface
                    // Do not merge when buildplate_only is on, because some underneath nodes may have been deleted.
                    if top_interface_layers > 0
                        && (node.support_roof_layers_below > 1 || (node.support_roof_layers_below >= 0 && !node.is_sharp_tail))
                        && !on_buildplate_only
                    {
                        if node.overhang.contour.points.len() > 100 || node.overhang.holes.len() > 1 {
                            area.push(node.overhang.clone());
                        } else {
                            area = cx::offset_ex_expolygon(
                                &node.overhang,
                                scale_(self.m_ts_data.m_xy_distance) as f32,
                                cx::DEFAULT_JOIN_TYPE,
                                cx::DEFAULT_MITER_LIMIT,
                            );
                        }
                    } else {
                        let slot =
                            self.draw_circles_get_node_collision(&mut collision, obj_layer_nr, node, node.is_sharp_tail && node.distance_to_top <= 0);
                        area = avoid_object_remove_extra_small_parts(
                            &tree_support_node_disk(node, &branch_circle, branch_radius, branch_radius_scaled, square_support, 0.0),
                            collision.get(slot),
                        );
                    }

                    if !area.is_empty() {
                        _has_circle_node = true;
                    }
                    if node.need_extra_wall
                        || (bottom_expand_enabled && node.print_z < do_not_mover_under_mm && node.dist_mm_to_top > do_not_mover_under_mm)
                    {
                        append(&mut extra_wall_area, area.clone());
                    }
                    if node.overhang_degree >= 2.0 {
                        append(&mut cooldown_area, area.clone());
                    }
                }

                // BBS multi-diameter: a folded slab is emitted on its top layer, so the bed band no longer lives on
                // support layer 0. Ask the node whether its material reaches the bed instead of trusting the
                // layer index, or the first layer expansion is skipped for every folded print.
                if self.m_raft_layers == 0 && (if self.m_fold_carriers.is_empty() { layer_nr == 0 } else { node.print_z - node.height <= EPSILON }) {
                    if tree_brim_width >= 0.0 {
                        brim_width = tree_brim_width as f64;
                    } else {
                        for expoly in &area {
                            brim_width = brim_width.max(expoly.map_moment_to_expansion(support_speed0, node.dist_mm_to_top));
                        }
                    }
                    let slot = self.draw_circles_get_node_collision(&mut collision, obj_layer_nr, node, false);
                    area = safe_offset_inc(
                        &area,
                        scale_(brim_width) as i32 as Coord,
                        collision.get(slot),
                        scale_(TS_MIN_BRANCH_RADIUS * 0.5) as i32 as Coord,
                        0,
                        1,
                    );
                }
                if obj_layer_nr > 0 && node.distance_to_top < 0 {
                    append(&mut roof_gap_areas, area);
                } else if num_top_interface_layers > 0
                    && obj_layer_nr > 0
                    && (node.support_roof_layers_below == 0 || node.support_roof_layers_below == 1)
                    // `int >= size_t`: compared as unsigned
                    && (node.distance_to_top as i64 as usize) >= num_top_interface_layers
                {
                    append(&mut roof_1st_layer, area);
                    max_layers_above_roof1 = max_layers_above_roof1.max(node.dist_mm_to_top);
                    if node.slab_height > EPSILON {
                        roof_span_node = Some(p_node);
                    }
                } else if num_top_interface_layers > 0 && obj_layer_nr > 0 && node.support_roof_layers_below > 0 {
                    append(&mut roof_areas, area);
                    if node.slab_height > EPSILON {
                        roof_span_node = Some(p_node);
                    }
                    max_layers_above_roof = if self.top_z_distance == 0.0 { node.height } else { max_layers_above_roof.max(node.dist_mm_to_top) };
                    // BBS multi-diameter: consecutive folded slabs sit several object layers apart, so the
                    // object layer number modulo the interface layer count repeats and the interface pattern
                    // stops alternating. The folded ladder position still steps by one per printed layer.
                    let num = if node.slab_index >= 0 { node.slab_index } else { node.obj_layer_nr };
                    let den = top_interface_layers as i32;
                    interface_id = if den == 0 { num } else { num.wrapping_rem(den) };
                } else {
                    if node.line_width > EPSILON {
                        for expoly in &area {
                            widened_areas.push(WidenedArea {
                                area: expoly.clone(),
                                bbox: BoundingBox::from_points(&expoly.contour.points),
                                width: node.line_width,
                            });
                        }
                    }
                    append(&mut base_areas, area);
                    max_layers_above_base = max_layers_above_base.max(node.dist_mm_to_top);
                }
            }

            // join roof segments
            // the lambda `get_roof_collision` (TreeSupport.cpp:4153-4155)
            let roof_collision = |this: &Self, collision: &mut DrawCirclesCollision, sharp_tail: bool| -> DrawCirclesCollisionSlot {
                match roof_span_node {
                    Some(n) => this.draw_circles_get_node_collision(collision, obj_layer_nr, &this.m_ts_data.contact_nodes[n], sharp_tail),
                    None => this.draw_circles_get_collision(collision, obj_layer_nr, sharp_tail),
                }
            };
            let lw = line_width_scaled as f32;
            {
                let closed = cx::offset2_ex(&roof_areas, lw, -lw, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
                let slot = roof_collision(self, &mut collision, z_overrides);
                roof_areas = cx::diff_clipped(&closed, collision.get(slot));
            }
            roof_areas = cx::intersection_ex(&roof_areas, std::slice::from_ref(&self.m_machine_border));
            roof_areas = cx::union_ex(&roof_areas);
            {
                let closed = cx::offset2_ex(&roof_1st_layer, lw, -lw, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
                let clip: ExPolygons = if z_overrides {
                    let slot = roof_collision(self, &mut collision, z_overrides);
                    cx::offset_ex(collision.get(slot), (line_width_scaled / 2.0) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT)
                } else {
                    let slot = roof_collision(self, &mut collision, false);
                    collision.get(slot).clone()
                };
                roof_1st_layer = cx::diff_clipped(&closed, &clip);
            }

            // roof_1st_layer and roof_areas may intersect, so need to subtract roof_areas from roof_1st_layer
            roof_1st_layer = cx::diff_ex_polygons(
                &roof_1st_layer,
                &cx::clip_clipper_polygons_with_subject_bbox(&roof_areas, &get_extents_expolygons(&roof_1st_layer)),
            );
            roof_1st_layer = cx::intersection_ex(&roof_1st_layer, std::slice::from_ref(&self.m_machine_border));

            {
                let ts_layer = &mut self.m_object.m_support_layers[ts_idx];
                ts_layer.base_areas = base_areas;
                ts_layer.roof_areas = roof_areas;
                ts_layer.roof_1st_layer = roof_1st_layer;
                ts_layer.roof_gap_areas = roof_gap_areas;
            }

            let data = &mut layer_draw_data[layer_nr];
            data.obj_layer_nr = obj_layer_nr;
            data.max_layers_above_base = max_layers_above_base;
            data.max_layers_above_roof = max_layers_above_roof;
            data.max_layers_above_roof1 = max_layers_above_roof1;
            data.interface_id = interface_id;
            data.collision_base = std::mem::take(&mut collision.collision_base);
            data.area_poly = area_poly;
            data.extra_wall_area = extra_wall_area;
            data.cooldown_area = cooldown_area;
            // std::sort(..., a.width > b.width): the order of libc++ for equal widths
            {
                let mut order: Vec<usize> = (0..widened_areas.len()).collect();
                crate::clipper::libcxx_sort(&mut order, |a: &usize, b: &usize| widened_areas[*a].width > widened_areas[*b].width);
                data.widened_areas = order.into_iter().map(|i| widened_areas[i].clone()).collect();
            }
        }

        // BBS multi-diameter: the gap areas of the layers a slab swallowed, keyed by every carrier whose
        // extrusion spans their Z. Folding moves the roof up to the carrier but leaves the virtual gap nodes
        // on their own fine layers, so a carrier has to clear the gaps of its whole span and not only the ones
        // that happen to land on its own layer.
        let mut swallowed_gap_by_carrier: HashMap<i32, ExPolygons> = HashMap::new();
        for swallowed_nr in 0..self.m_fold_slab_of_layer.len() {
            if self.m_fold_slab_of_layer[swallowed_nr] < 0 {
                continue;
            }
            let swallowed = match self.m_object.m_support_layers.get(swallowed_nr + self.m_raft_layers) {
                Some(l) => l,
                None => continue,
            };
            if swallowed.roof_gap_areas.is_empty() {
                continue;
            }
            for carrier in self.fold_carriers_above(swallowed_nr) {
                swallowed_gap_by_carrier.entry(carrier as i32).or_default().extend(swallowed.roof_gap_areas.iter().cloned());
            }
        }

        // The capacity of the `holes` vector of every area group of a layer, for holePropagationInfos below: the
        // number of holes each expolygon was made with (PolyTreeToExPolygons resizes `holes` once, a copy takes
        // exactly its size), before the small ones are erased.
        let mut holes_capacity: Vec<Vec<usize>> = vec![Vec::new(); self.m_object.m_support_layers.len()];

        for layer_nr in 0..n_layers {
            if (self.canceled)() {
                break;
            }

            let ts_idx = layer_nr + self.m_raft_layers;
            if self.contact_nodes[layer_nr].is_empty() || self.m_object.m_support_layers[ts_idx].layer.height < EPSILON {
                continue;
            }

            let data = &layer_draw_data[layer_nr];
            let obj_layer_nr: usize = data.obj_layer_nr;
            let max_layers_above_base: CoordF = data.max_layers_above_base;
            let max_layers_above_roof: CoordF = data.max_layers_above_roof;
            let max_layers_above_roof1: CoordF = data.max_layers_above_roof1;
            let interface_id: i32 = data.interface_id;
            let collision_base: &ExPolygons = &data.collision_base;
            let area_poly: &ExPolygons = &data.area_poly;
            let extra_wall_area: &ExPolygons = &data.extra_wall_area;
            let cooldown_area: &ExPolygons = &data.cooldown_area;
            let mut base_areas: ExPolygons = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].base_areas);
            let mut roof_areas: ExPolygons = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].roof_areas);
            let mut roof_1st_layer: ExPolygons = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].roof_1st_layer);
            let mut floor_areas: ExPolygons = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].floor_areas);
            let roof_gap_areas: ExPolygons = self.m_object.m_support_layers[ts_idx].roof_gap_areas.clone();

            // The gap under an overhang may carry no material at all, so it carves the base below. A folded
            // carrier answers for its whole slab span, hence the gaps the swallowed layers left behind are pulled
            // in as well. The interface must not be carved.
            let mut gap_keep_out: ExPolygons = roof_gap_areas;
            if !self.m_fold_carriers.is_empty() {
                if let Some(swallowed) = swallowed_gap_by_carrier.get(&(layer_nr as i32)) {
                    append(&mut gap_keep_out, swallowed.clone());
                }
            }

            let mut roofs: ExPolygons = Vec::new();
            append(&mut roofs, roof_1st_layer.clone());
            append(&mut roofs, roof_areas.clone());
            append(&mut roofs, gap_keep_out);

            // slightly slow down the slicing speed
            base_areas = if collision_base.is_empty() {
                cx::diff_ex(&base_areas, &self.m_ts_data.get_collision(0.0, obj_layer_nr))
            } else {
                cx::diff_ex(&base_areas, collision_base)
            };

            base_areas = cx::diff_ex_polygons(&base_areas, &cx::clip_clipper_polygons_with_subject_bbox(&roofs, &get_extents_expolygons(&base_areas)));
            base_areas = cx::intersection_ex(&base_areas, std::slice::from_ref(&self.m_machine_border));

            if square_support {
                // simplify support contours
                // `area.simplify(tolerance, &base_areas_simplified)` of every area (ExPolygon.cpp:253-261)
                base_areas = expolygons_simplify(&base_areas, scale_(support_extrusion_width / 2.0));
            }
            //Subtract support floors. We can only compute floor_areas here instead of with roof_areas,
            // or we'll get much wider floor than necessary.
            // `size_t + int`: unsigned
            let gap_layers_u: usize = bottom_gap_layers as i64 as usize;
            if bottom_interface_layers.wrapping_add(gap_layers_u) > 0 {
                if layer_nr >= bottom_interface_layers.wrapping_add(gap_layers_u) {
                    // find the lowest interface layer
                    // TODO the gap may not be exact when "independent support layer height" is enabled
                    let layer_nr_next: usize = layer_nr - bottom_interface_layers;
                    let obj_layer_nr_next: usize = self.m_ts_data.layer_heights[layer_nr_next].obj_layer_nr;
                    let mut i: usize = 0;
                    while i <= gap_layers_u && i <= obj_layer_nr_next {
                        let below_layer = self.m_object.get_layer((obj_layer_nr_next - i) as i32);
                        let bottom_interface: ExPolygons = cx::intersection_ex(&base_areas, &below_layer.lslices);
                        floor_areas.extend(bottom_interface);
                        i += 1;
                    }
                }
                if !floor_areas.is_empty() {
                    base_areas = cx::diff_ex(&base_areas, &cx::offset_ex(&floor_areas, 10.0, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT));
                }
            }
            if bottom_gap_layers > 0 && self.m_ts_data.layer_heights[layer_nr].obj_layer_nr > gap_layers_u {
                let below_layer = self.m_object.get_layer((self.m_ts_data.layer_heights[layer_nr].obj_layer_nr - gap_layers_u) as i32);
                let bottom_gap_area: ExPolygons = cx::intersection_ex(&floor_areas, &below_layer.lslices);
                if !bottom_gap_area.is_empty() {
                    floor_areas = cx::diff_ex(&floor_areas, &bottom_gap_area);
                }
            }
            // prune_floating_supports() pairs each area_group back with its region by walking the vector below
            // in step with the group order, so the fills that follow have to keep appending in vector order
            // and nothing may reorder or drop entries on either side afterwards.
            let mut area_groups: Vec<SupportLayerAreaGroup> = std::mem::take(&mut self.m_object.m_support_layers[ts_idx].area_groups);
            for (idx, expoly) in base_areas.iter().enumerate() {
                //if (area(expoly) < SQ(scale_(1))) continue;
                let mut group = SupportLayerAreaGroup::new(idx, SupportLayerAreaType::BaseType, max_layers_above_base);
                group.need_infill = cx::overlaps_expolygons(std::slice::from_ref(expoly), area_poly);
                let need_extra_wall = cx::overlaps_expolygons(std::slice::from_ref(expoly), extra_wall_area);
                group.need_extra_wall = need_extra_wall && !group.need_infill;
                group.need_cooling = cx::overlaps_expolygons(std::slice::from_ref(expoly), cooldown_area);
                // The whole island takes the widest wall among the widened disks that overlap it.
                let expoly_bbox = BoundingBox::from_points(&expoly.contour.points);
                for widened in &data.widened_areas {
                    if bbox_overlap(&widened.bbox, &expoly_bbox) && cx::overlaps(&widened.area, expoly) {
                        group.line_width = widened.width;
                        break;
                    }
                }
                area_groups.push(group);
            }
            let mut new_roofs: ExPolygons = Vec::new();
            for expoly in &roof_areas {
                if expoly.area() < sq(scale_(1.0)) {
                    if max_layers_above_roof > EPSILON {
                        roof_1st_layer.push(expoly.clone());
                    }
                    continue;
                }
                new_roofs.push(expoly.clone());
            }
            roof_areas = new_roofs;
            for idx in 0..roof_areas.len() {
                let mut group = SupportLayerAreaGroup::new(idx, SupportLayerAreaType::RoofType, max_layers_above_roof);
                group.interface_id = interface_id;
                area_groups.push(group);
            }
            for idx in 0..floor_areas.len() {
                //if (area(expoly) < SQ(scale_(1))) continue;
                area_groups.push(SupportLayerAreaGroup::new(idx, SupportLayerAreaType::FloorType, 10000.0));
            }
            for idx in 0..roof_1st_layer.len() {
                //if (area(expoly) < SQ(scale_(1))) continue;
                area_groups.push(SupportLayerAreaGroup::new(idx, SupportLayerAreaType::Roof1stLayer, max_layers_above_roof1));
            }

            let ts_layer = &mut self.m_object.m_support_layers[ts_idx];
            ts_layer.base_areas = base_areas;
            ts_layer.roof_areas = roof_areas;
            ts_layer.roof_1st_layer = roof_1st_layer;
            ts_layer.floor_areas = floor_areas;

            // A folded bed band reaches the bed from its carrier, which may sit well above the first layer, and the
            // brim keeps off the support by these islands.
            let keep_support_islands = layer_nr < brim_skirt_layers
                || (has_brim
                    && self.m_raft_layers == 0
                    && !self.m_fold_carriers.is_empty()
                    && ts_layer.layer.print_z - ts_layer.effective_print_height() <= EPSILON);
            let mut capacities: Vec<usize> = Vec::with_capacity(area_groups.len());
            for area_group in &area_groups {
                let expoly = ts_layer.area_group_area_mut(area_group);
                capacities.push(expoly.holes.len());
                expoly.holes.retain(|hole| {
                    let bbox_size = BoundingBox::from_points(&hole.points).size();
                    !((bbox_size.x as f64) < scale_(2.0) && (bbox_size.y as f64) < scale_(2.0))
                });

                if keep_support_islands {
                    let expoly = expoly.clone();
                    ts_layer.support_islands.push(expoly);
                }
            }
            holes_capacity[ts_idx] = capacities;
            ts_layer.area_groups = area_groups;

            ts_layer.support_islands = cx::union_ex(&ts_layer.support_islands);
            //Must update bounding box which is used in avoid crossing perimeter
            ts_layer.layer.lslices_bboxes.clear();
            for expoly in &ts_layer.support_islands {
                ts_layer.layer.lslices_bboxes.push(BoundingBox::from_points(&expoly.contour.points));
            }
            ts_layer.layer.backup_untyped_slices();
        }

        // prune_floating_supports() is not called. Its carrier test deletes bed-landing
        // supports that still reach the plate, so those supports come out floating.
        for ts_idx in self.resolve_material_covered_by_folded_interfaces() {
            // a trimmed layer has its regions rebuilt: every `holes` vector is as large as it is full
            let ts_layer = &self.m_object.m_support_layers[ts_idx];
            holes_capacity[ts_idx] = ts_layer.area_groups.iter().map(|g| ts_layer.area_group_area(g).holes.len()).collect();
        }

        if with_lightning_infill {
            let mut contours: Vec<Polygons> = Vec::new();
            let mut overhangs: Vec<Polygons> = Vec::new();
            // BBS multi-diameter: a swallowed layer is not a gap in the support - its material moved up to
            // the slab carrier. FillLightning::Generator derives its prune and straightening lengths from a
            // single object_config.layer_height and treats consecutive entries as adjacent layers, so
            // skipping the swallowed ones would hand it a stack whose steps are a whole band apart and it
            // would join trees across that band with one straight segment. Feeding the footprint of every
            // carrier spanning those layers keeps the stack one nominal layer per step.
            // Each layer is read up to three times below, and a folded footprint is a union, so build them once.
            // Unfolded, the layer's own base_areas are read in place.
            let mut folded_base: Vec<ExPolygons> = Vec::new();
            if !self.m_fold_carriers.is_empty() {
                folded_base.resize(self.contact_nodes.len(), Vec::new());
                for layer_nr in 0..folded_base.len() {
                    folded_base[layer_nr] = self.draw_circles_effective_base(layer_nr as i32);
                }
            }
            let no_base: ExPolygons = Vec::new();
            let raft_layers = self.m_raft_layers;
            // the lambda `base_of` (TreeSupport.cpp:4400-4405)
            let base_of = |this: &Self, layer_nr: i32| -> ExPolygons {
                if !folded_base.is_empty() {
                    return folded_base[layer_nr as usize].clone();
                }
                match this.m_object.m_support_layers.get(layer_nr as usize + raft_layers) {
                    Some(ts) if !ts.area_groups.is_empty() => ts.base_areas.clone(),
                    _ => no_base.clone(),
                }
            };
            // A roof-only layer still has to be visited: its empty base turns the whole base below it into
            // overhang, which is what grows the lightning tree up under the roof.
            let has_nothing = |this: &Self, layer_nr: i32| -> bool {
                if !base_of(this, layer_nr).is_empty() {
                    return false;
                }
                match this.m_object.m_support_layers.get(layer_nr as usize + raft_layers) {
                    Some(ts) => ts.area_groups.is_empty(),
                    None => true,
                }
            };
            for layer_nr in 1..self.contact_nodes.len() as i32 {
                if (self.canceled)() {
                    break;
                }

                // skip if current layer has no points. This fixes potential crash in get_collision (see jira BBL001-355)
                if has_nothing(self, layer_nr) {
                    continue;
                }
                let base_areas = base_of(self, layer_nr);

                let mut layer_nr_lower: i32 = layer_nr - 1;
                while layer_nr_lower >= 0 {
                    if !has_nothing(self, layer_nr_lower) {
                        break;
                    }
                    layer_nr_lower -= 1;
                }
                if layer_nr_lower <= 0 {
                    continue;
                }

                let base_areas_lower = base_of(self, layer_nr_lower);

                let mut overhang: ExPolygons =
                    cx::diff_ex(&crate::clipper2_utils::offset_ex_2(&base_areas_lower, -2.5 * line_width_scaled), &base_areas);
                let mut it = 0;
                while it < overhang.len() {
                    overhang[it].remove_colinear_points();
                    if overhang[it].contour.points.is_empty() {
                        overhang.remove(it);
                    } else {
                        it += 1;
                    }
                }

                overhangs.push(to_polygons(&overhang));
                contours.push(to_polygons(&base_areas_lower));
                // draw_circles() zeroes print_z on a layer that got no node, which is every swallowed layer,
                // so key the entry by the Z the plan gave it. Those entries are never looked up - a
                // swallowed layer extrudes nothing - but a shared key of 0 would let them overwrite each
                // other and, worse, collide with a layer that really does print at 0.
                let lower_print_z: CoordF = if (layer_nr_lower as usize) < self.m_ts_data.layer_heights.len() {
                    self.m_ts_data.layer_heights[layer_nr_lower as usize].print_z
                } else {
                    self.m_object.m_support_layers[layer_nr_lower as usize + raft_layers].layer.print_z
                };
                self.print_z_to_lightninglayer.insert(f64_key(lower_print_z), overhangs.len() - 1);
            }

            let m_support_material_flow =
                crate::support::support_parameters::support_material_flow_of(self.m_object, self.m_print, self.m_slicing_params.layer_height as f32)
                    .expect("support_material_flow");
            let support_spacing: CoordF = cfg_float(self.object_config(), "support_base_pattern_spacing") + m_support_material_flow.spacing();
            // for lightning infill the density is defined differently, so need to double it
            let support_density: CoordF = (1.0_f64).min(m_support_material_flow.spacing() / support_spacing * 2.0);
            self.lightning_support_density = support_density as f32;
            if let Some(factory) = self.lightning_support_generator.as_ref() {
                self.generator = Some(factory.generate(self.m_object, self.m_print, &mut contours, &mut overhangs, support_density as f32));
            }
            self.lightning_contours = contours;
            self.lightning_overhangs = overhangs;
        } else if !self.with_infill {
            // move the holes to contour so they can be well supported

            // check if poly's contour intersects with expoly's contour
            // the lambda `intersects_contour` (TreeSupport.cpp:4468-4490)
            let intersects_contour =
                |poly: &Polygon, expoly: &ExPolygon, pt_on_poly: &mut Point, pt_on_expoly: &mut Point, pt_far_on_poly: &mut Point, dist_thresh: f32| -> bool {
                    let pl_out = cx::intersection_pl(&cx::to_polylines(expoly), &ExPolygon::new(poly.clone()));
                    if pl_out.is_empty() {
                        return false;
                    }
                    let mut min_dist: f32 = f32::MAX;
                    let mut max_dist: f32 = 0.0;
                    for &from in &poly.points {
                        for i in 0..expoly.num_contours() {
                            let candidate = match polygon_closest_vertex(contour_or_hole(expoly, i), &from) {
                                Some(c) => c,
                                None => continue,
                            };
                            let dist2: f64 = vsize2_with_unscale(Point::new(candidate.x - from.x, candidate.y - from.y));
                            if dist2 < min_dist as f64 {
                                min_dist = dist2 as f32;
                                *pt_on_poly = from;
                                *pt_on_expoly = candidate;
                            }
                            if dist2 > max_dist as f64 {
                                max_dist = dist2 as f32;
                                *pt_far_on_poly = from;
                            }
                            if dist2 < dist_thresh as f64 {
                                return true;
                            }
                        }
                    }
                    false
                };

            // polygon pointer: depth, direction, farPoint
            // `std::map<const Polygon*, std::tuple<int, Point, Point>>`: the pointer is (support layer, area group,
            // generation of the buffer of its `holes`, index of the hole)
            let mut hole_propagation_infos: HashMap<(usize, usize, u32, usize), (i32, Point, Point)> = HashMap::new();
            let mut holes_generation: Vec<Vec<u32>> = holes_capacity.iter().map(|c| vec![0u32; c.len()]).collect();
            let mut layer_nr: i32 = self.contact_nodes.len() as i32 - 1;
            while layer_nr > 0 {
                let this_layer_nr = layer_nr;
                layer_nr -= 1;
                let layer_nr = this_layer_nr;
                if (self.canceled)() {
                    break;
                }

                let ts_idx = layer_nr as usize + self.m_raft_layers;

                // skip if current layer has no points. This fixes potential crash in get_collision (see jira BBL001-355)
                if self.contact_nodes[layer_nr as usize].is_empty() {
                    continue;
                }
                if self.m_object.m_support_layers[ts_idx].layer.height < EPSILON {
                    continue;
                }
                if self.m_object.m_support_layers[ts_idx].area_groups.is_empty() {
                    continue;
                }

                let mut layer_nr_lower: i32 = layer_nr - 1;
                while layer_nr_lower >= 0 {
                    if !self.m_object.m_support_layers[layer_nr_lower as usize + self.m_raft_layers].area_groups.is_empty() {
                        break;
                    }
                    layer_nr_lower -= 1;
                }
                if layer_nr_lower < 0 {
                    continue;
                }
                let lower_idx = layer_nr_lower as usize + self.m_raft_layers;
                let n_groups_lower = self.m_object.m_support_layers[lower_idx].area_groups.len();

                for group_idx in 0..self.m_object.m_support_layers[ts_idx].area_groups.len() {
                    let area_group = self.m_object.m_support_layers[ts_idx].area_groups[group_idx].clone();
                    if area_group.type_ != SupportLayerAreaType::BaseType {
                        continue;
                    }
                    // the holes of `area`, which this loop does not change
                    let holes: Polygons = self.m_object.m_support_layers[ts_idx].area_group_area(&area_group).holes.clone();
                    for (hole_idx, hole) in holes.iter().enumerate() {
                        // `&hole`
                        let hole_key = (ts_idx, group_idx, holes_generation[ts_idx][group_idx], hole_idx);
                        for group_lower_idx in 0..n_groups_lower {
                            let area_group_lower = self.m_object.m_support_layers[lower_idx].area_groups[group_lower_idx].clone();
                            let mut pt_on_poly = Point::new(0, 0);
                            let mut pt_on_expoly = Point::new(0, 0);
                            let mut pt_far_on_poly = Point::new(0, 0);
                            // if a hole doesn't intersect with lower layer's contours, add a hole to lower layer and move it slightly to the contour
                            let contour_contains_front;
                            let intersects;
                            {
                                let base_area_lower = self.m_object.m_support_layers[lower_idx].area_group_area(&area_group_lower);
                                contour_contains_front = crate::geometry::contains_polygon(&base_area_lower.contour, &hole.points[0], true);
                                intersects = contour_contains_front
                                    && intersects_contour(
                                        hole,
                                        base_area_lower,
                                        &mut pt_on_poly,
                                        &mut pt_on_expoly,
                                        &mut pt_far_on_poly,
                                        // `0.01 + unscale_(line_width_scaled) * (need_extra_wall ? 2. : 0.5)`: contracted, narrowed to float
                                        (line_width_scaled * SCALING_FACTOR).mul_add(if area_group.need_extra_wall { 2.0 } else { 0.5 }, 0.01) as f32,
                                    );
                            }
                            if contour_contains_front && !intersects {
                                let mut hole_lower: Polygon = hole.clone();
                                let direction = normal(Point::new(pt_on_expoly.x - pt_on_poly.x, pt_on_expoly.y - pt_on_poly.y), line_width_scaled / 2.0);
                                hole_lower.translate(direction);
                                // note to expand a hole, we need to do negative offset
                                let mut hole_expanded =
                                    cx::offset_polygon(&hole_lower, (-line_width_scaled / 4.0) as f32, crate::clipper::JoinType::jtSquare, cx::DEFAULT_MITER_LIMIT);
                                if !hole_expanded.is_empty() {
                                    let lower_layer = &mut self.m_object.m_support_layers[lower_idx];
                                    let base_area_lower = lower_layer.area_group_area_mut(&area_group_lower);
                                    // push_back: the buffer of the vector is replaced when it is full
                                    if base_area_lower.holes.len() == holes_capacity[lower_idx][group_lower_idx] {
                                        holes_capacity[lower_idx][group_lower_idx] =
                                            (2 * holes_capacity[lower_idx][group_lower_idx]).max(base_area_lower.holes.len() + 1);
                                        holes_generation[lower_idx][group_lower_idx] += 1;
                                    }
                                    base_area_lower.holes.push(hole_expanded.swap_remove(0));
                                    hole_propagation_infos
                                        .entry((lower_idx, group_lower_idx, holes_generation[lower_idx][group_lower_idx], base_area_lower.holes.len() - 1))
                                        .or_insert((25, direction, pt_far_on_poly));
                                }
                                break;
                            } else if hole_propagation_infos.get(&hole_key).map(|info| {
                                info.0 > 0
                                    && crate::geometry::contains_polygon(
                                        &self.m_object.m_support_layers[lower_idx].area_group_area(&area_group_lower).contour,
                                        &info.2,
                                        true,
                                    )
                            }) == Some(true)
                            {
                                // after the hole connects to contour, shrink it gradually until it vanishes while moving it outwards. The moving direction is defined in the previous step
                                let info = hole_propagation_infos[&hole_key];
                                let mut hole_lower: Polygon = hole.clone();
                                let direction = info.1;
                                hole_lower.translate(direction);
                                // note to shrink a hole, we need to do positive offset
                                let mut hole_expanded =
                                    cx::offset_polygon(&hole_lower, (line_width_scaled / 2.0) as f32, crate::clipper::JoinType::jtSquare, cx::DEFAULT_MITER_LIMIT);
                                // `direction * 2`: Eigen's integer product
                                let far_point = Point::new(info.2.x + direction.x * 2, info.2.y + direction.y * 2);
                                if !hole_expanded.is_empty() {
                                    let lower_layer = &mut self.m_object.m_support_layers[lower_idx];
                                    let base_area_lower = lower_layer.area_group_area_mut(&area_group_lower);
                                    if base_area_lower.holes.len() == holes_capacity[lower_idx][group_lower_idx] {
                                        holes_capacity[lower_idx][group_lower_idx] =
                                            (2 * holes_capacity[lower_idx][group_lower_idx]).max(base_area_lower.holes.len() + 1);
                                        holes_generation[lower_idx][group_lower_idx] += 1;
                                    }
                                    base_area_lower.holes.push(hole_expanded.swap_remove(0));
                                    hole_propagation_infos
                                        .entry((lower_idx, group_lower_idx, holes_generation[lower_idx][group_lower_idx], base_area_lower.holes.len() - 1))
                                        .or_insert((info.0 - 1, direction, far_point));
                                }
                                break;
                            }
                        }
                        {
                            // if roof1 interface is inside a hole, need to expand the interface
                            let n_roof1 = self.m_object.m_support_layers[ts_idx].roof_1st_layer.len();
                            for roof1_idx in 0..n_roof1 {
                                let roof1_contour: Polygon = self.m_object.m_support_layers[ts_idx].roof_1st_layer[roof1_idx].contour.clone();
                                let is_inside_hole = roof1_contour.points.iter().all(|pt| crate::geometry::contains_polygon(hole, pt, true));
                                if is_inside_hole {
                                    let mut hole_reoriented: Polygon = hole.clone();
                                    if roof1_contour.is_counter_clockwise() {
                                        hole_reoriented.make_counter_clockwise();
                                    } else if roof1_contour.is_clockwise() {
                                        hole_reoriented.make_clockwise();
                                    }
                                    let tmp = cx::union_(&[roof1_contour.clone(), hole_reoriented]);
                                    if !tmp.is_empty() {
                                        self.m_object.m_support_layers[ts_idx].roof_1st_layer[roof1_idx].contour = tmp[0].clone();
                                    }

                                    // make sure 1) roof1 and object 2) roof1 and roof, won't intersect
                                    // Note: We can't replace roof1 directly, as we have recorded its address.
                                    //       So instead we need to replace its members one by one.
                                    let roof1: ExPolygon = self.m_object.m_support_layers[ts_idx].roof_1st_layer[roof1_idx].clone();
                                    let collision = self.get_collision(0.0, self.m_ts_data.layer_heights[layer_nr as usize].obj_layer_nr);
                                    let mut tmp1 = cx::diff_ex(std::slice::from_ref(&roof1), &collision);
                                    tmp1 = cx::diff_ex(&tmp1, &self.m_object.m_support_layers[ts_idx].roof_areas);
                                    if !tmp1.is_empty() {
                                        let first = tmp1.swap_remove(0);
                                        let roof1 = &mut self.m_object.m_support_layers[ts_idx].roof_1st_layer[roof1_idx];
                                        roof1.contour = first.contour;
                                        roof1.holes = first.holes;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Only remove empty non-raft support layers. Raft layers (first m_raft_layers entries)
        // must be preserved to avoid support_layer_count < m_raft_layers mismatch.
        {
            let raft_layers = self.m_raft_layers;
            let ts_layers = &mut self.m_object.m_support_layers;
            let mut idx = 0usize;
            ts_layers.retain(|ts_layer| {
                let keep = idx < raft_layers || !(ts_layer.layer.height < EPSILON);
                idx += 1;
                keep
            });
            let n = ts_layers.len();
            for layer_nr in 0..n {
                ts_layers[layer_nr].layer.upper_layer_id = if layer_nr != n - 1 { Some(layer_nr + 1) } else { None };
                ts_layers[layer_nr].layer.lower_layer_id = if layer_nr > 0 { Some(layer_nr - 1) } else { None };
            }
        }
    }

    // TreeSupport.cpp:4369-4388 the lambda `effective_base` of draw_circles()
    fn draw_circles_effective_base(&self, layer_nr: i32) -> ExPolygons {
        if layer_nr < 0 {
            return Vec::new();
        }
        let ts = self.m_object.m_support_layers.get(layer_nr as usize + self.m_raft_layers);
        let spanning: Vec<usize> = self.fold_carriers_above(layer_nr as usize);
        if spanning.is_empty() {
            return match ts {
                Some(ts) if !ts.area_groups.is_empty() => ts.base_areas.clone(),
                _ => Vec::new(),
            };
        }
        // A layer inside a folded slab contributes its own regions only while it still prints a column
        // of its own - a carrier, or a column whose carrier refused. Reading the footprint of a layer
        // without one would make the stack jump from a sliver to the full slab and invent an overhang.
        let own_column = self.contact_nodes[layer_nr as usize].iter().any(|&node| self.m_ts_data.contact_nodes[node].distance_to_top >= 0);
        let mut footprint: ExPolygons = Vec::new();
        if own_column {
            if let Some(ts) = ts {
                if !ts.area_groups.is_empty() {
                    footprint = ts.base_areas.clone();
                }
            }
        }
        for carrier_nr in spanning {
            if let Some(carrier) = self.m_object.m_support_layers.get(carrier_nr + self.m_raft_layers) {
                append(&mut footprint, carrier.base_areas.clone());
            }
        }
        cx::union_ex(&footprint)
    }
}

// ExPolygon.cpp `BoundingBox get_extents(const ExPolygon &expolygon)`: the box of the contour, a default
// constructed box when it has no points.
fn get_extents_expolygon(expolygon: &ExPolygon) -> BoundingBox {
    get_extents_expolygons(std::slice::from_ref(expolygon))
}

// TreeSupport.cpp:4643-4655 the lambda `sync_support_islands` of resolve_material_covered_by_folded_interfaces()
fn sync_support_islands(layer: &mut crate::print::faithful::SupportLayer) {
    if layer.support_islands.is_empty() {
        return;
    }
    let mut islands: ExPolygons = Vec::with_capacity(layer.area_groups.len());
    for area_group in &layer.area_groups {
        islands.push(layer.area_group_area(area_group).clone());
    }
    layer.support_islands = cx::union_ex(&islands);
    layer.layer.lslices_bboxes.clear();
    for expoly in &layer.support_islands {
        layer.layer.lslices_bboxes.push(get_extents_expolygon(expoly));
    }
}

// TreeSupport.cpp:4669-4752 the lambda `trim_layer` of resolve_material_covered_by_folded_interfaces()
fn trim_layer(layer: &mut crate::print::faithful::SupportLayer, base_cut: &[ExPolygon], interface_cut: &[ExPolygon]) -> bool {
    use crate::print::faithful::{SupportLayerAreaGroup, SupportLayerAreaType};
    if base_cut.is_empty() && interface_cut.is_empty() {
        return false;
    }

    const NUM_AREA_TYPES: usize = (SupportLayerAreaType::Roof1stLayer + 1) as usize;
    // `ExPolygons *regions_by_type[num_area_types] { &layer.base_areas, &layer.roof_areas, &layer.floor_areas, &layer.roof_1st_layer }`
    let regions_len = |layer: &crate::print::faithful::SupportLayer, type_: usize| -> usize {
        match type_ {
            0 => layer.base_areas.len(),
            1 => layer.roof_areas.len(),
            2 => layer.floor_areas.len(),
            _ => layer.roof_1st_layer.len(),
        }
    };
    let mut kept_regions: [ExPolygons; NUM_AREA_TYPES] = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    let base_cut_bboxes: Vec<BoundingBox> = base_cut.iter().map(get_extents_expolygon).collect();
    let interface_cut_bboxes: Vec<BoundingBox> = interface_cut.iter().map(get_extents_expolygon).collect();
    let mut kept_groups: Vec<SupportLayerAreaGroup> = Vec::with_capacity(layer.area_groups.len());
    let mut changed = false;
    let mut source_index: [usize; NUM_AREA_TYPES] = [0; NUM_AREA_TYPES];
    for area_group in &layer.area_groups {
        let type_ = area_group.type_;
        if type_ < SupportLayerAreaType::BaseType || type_ > SupportLayerAreaType::Roof1stLayer {
            continue;
        }
        let index = source_index[type_ as usize];
        source_index[type_ as usize] += 1;
        // `area_group.area == regions_by_type[type]->data() + index`
        let mapping_valid = index < regions_len(layer, type_ as usize) && area_group.area == index;
        debug_assert!(mapping_valid);
        if !mapping_valid {
            // BOOST_LOG_TRIVIAL(error) << "Cannot trim folded support overlap: stale area-group mapping";
            return false;
        }

        let (cut, cut_bboxes): (Option<&[ExPolygon]>, Option<&Vec<BoundingBox>>) = if type_ == SupportLayerAreaType::BaseType {
            (Some(base_cut), Some(&base_cut_bboxes))
        } else if type_ == SupportLayerAreaType::RoofType || type_ == SupportLayerAreaType::Roof1stLayer {
            (Some(interface_cut), Some(&interface_cut_bboxes))
        } else {
            (None, None)
        };
        let area: &ExPolygon = layer.area_group_area(area_group);
        let area_bbox = get_extents_expolygon(area);
        let mut local_cut: ExPolygons = Vec::new();
        if let (Some(cut), Some(cut_bboxes)) = (cut, cut_bboxes) {
            for cut_idx in 0..cut.len() {
                if bbox_overlap(&cut_bboxes[cut_idx], &area_bbox) {
                    local_cut.push(cut[cut_idx].clone());
                }
            }
        }
        let pieces: ExPolygons = if !local_cut.is_empty() { cx::diff_ex(std::slice::from_ref(area), &local_cut) } else { vec![area.clone()] };
        changed |= !local_cut.is_empty();

        for piece in pieces {
            kept_regions[type_ as usize].push(piece);
            // `kept.area = nullptr;` until the regions are in place
            kept_groups.push(area_group.clone());
        }
    }

    for type_ in 0..NUM_AREA_TYPES {
        let mapping_complete = source_index[type_] == regions_len(layer, type_);
        debug_assert!(mapping_complete);
        if !mapping_complete {
            // BOOST_LOG_TRIVIAL(error) << "Cannot trim folded support overlap: incomplete area-group mapping";
            return false;
        }
    }
    if !changed {
        return false;
    }

    let [base, roof, floor, roof1] = kept_regions;
    layer.base_areas = base;
    layer.roof_areas = roof;
    layer.floor_areas = floor;
    layer.roof_1st_layer = roof1;
    let mut next_index: [usize; NUM_AREA_TYPES] = [0; NUM_AREA_TYPES];
    for area_group in kept_groups.iter_mut() {
        area_group.area = next_index[area_group.type_ as usize];
        next_index[area_group.type_ as usize] += 1;
    }
    layer.area_groups = kept_groups;

    if layer.roof_areas.is_empty() && layer.roof_1st_layer.is_empty() {
        layer.planned_interface_filament_id = 0;
        layer.planned_interface_nozzle_diameter = 0.;
        layer.interface_slab_height = 0.;
    }
    if layer.base_areas.is_empty() {
        layer.planned_filament_id = 0;
        layer.planned_nozzle_diameter = 0.;
        layer.base_slab_height = 0.;
    }
    sync_support_islands(layer);
    true
}

impl<'a> TreeSupport<'a> {
    /// TreeSupport.cpp:4635-4901 `void TreeSupport::resolve_material_covered_by_folded_interfaces()`
    ///
    /// Remove fine-grid interface and ordinary base material already occupied by a folded interface slab.
    /// Runs after all typed areas are finalized and before later support passes read them back.
    ///
    /// Returns the positions in `m_object->support_layers()` of the layers it trimmed (the C++ counts them for
    /// its log line): draw_circles() follows the buffers of their `holes`.
    pub fn resolve_material_covered_by_folded_interfaces(&mut self) -> Vec<usize> {
        let mut trimmed_layers: Vec<usize> = Vec::new();
        if self.m_fold_carriers.is_empty() {
            return trimmed_layers;
        }
        let raft_layers = self.m_raft_layers;
        let n_layers = self.m_object.m_support_layers.len();
        if n_layers <= raft_layers {
            return trimmed_layers;
        }

        use crate::print::faithful::SupportLayer;
        let is_folded_interface = |layer: &SupportLayer| -> bool {
            layer.interface_slab_height > layer.layer.height + EPSILON && (!layer.roof_areas.is_empty() || !layer.roof_1st_layer.is_empty())
        };
        let interface_height =
            |layer: &SupportLayer| -> CoordF { if layer.interface_slab_height > EPSILON { layer.interface_slab_height } else { layer.layer.height } };
        let base_height = |layer: &SupportLayer| -> CoordF { if layer.base_slab_height > EPSILON { layer.base_slab_height } else { layer.layer.height } };

        let mut base_cut: Vec<ExPolygons> = vec![Vec::new(); n_layers];
        let mut interface_cut: Vec<ExPolygons> = vec![Vec::new(); n_layers];
        let mut source_idx = n_layers;
        while source_idx > raft_layers {
            source_idx -= 1;
            if trim_layer(&mut self.m_object.m_support_layers[source_idx], &base_cut[source_idx], &interface_cut[source_idx]) {
                trimmed_layers.push(source_idx);
            }

            {
                let source = &self.m_object.m_support_layers[source_idx];
                if !source.base_areas.is_empty() && (!source.roof_areas.is_empty() || !source.roof_1st_layer.is_empty()) {
                    let mut base_priority_cut: ExPolygons = source.roof_areas.clone();
                    append(&mut base_priority_cut, source.roof_1st_layer.clone());
                    if !base_priority_cut.is_empty()
                        && bbox_overlap(&get_extents_expolygons(&source.base_areas), &get_extents_expolygons(&base_priority_cut))
                        && !cx::intersection_ex(&source.base_areas, &base_priority_cut).is_empty()
                        && trim_layer(&mut self.m_object.m_support_layers[source_idx], &base_priority_cut, &[])
                    {
                        trimmed_layers.push(source_idx);
                    }
                }
            }

            // Nested folded bases share a complete Z interval, so the outer carrier already extrudes their XY
            // intersection. Trim that duplicate from the shorter layer using the final rendered geometry. True
            // partial-Z overlaps and interface-over-base conflicts were rejected before folding.
            if self.m_object.m_support_layers[source_idx].base_slab_height > self.m_object.m_support_layers[source_idx].layer.height + EPSILON
                && !self.m_object.m_support_layers[source_idx].base_areas.is_empty()
            {
                let source_top: CoordF = self.m_object.m_support_layers[source_idx].layer.print_z;
                let source_bottom: CoordF = source_top - self.m_object.m_support_layers[source_idx].base_slab_height;
                // The base-only fold layers of the second round keep clear of the interfaces that stay on the fine
                // grid inside their slab: the fine base carries those, the coarse layer leaves their projection empty.
                if self.m_base_only_fold_layers.binary_search(&(source_idx - raft_layers)).is_ok() {
                    let base_bbox = get_extents_expolygons(&self.m_object.m_support_layers[source_idx].base_areas);
                    let mut fine_interface: ExPolygons = Vec::new();
                    let mut victim_idx = source_idx;
                    while victim_idx > raft_layers {
                        victim_idx -= 1;
                        let victim = &self.m_object.m_support_layers[victim_idx];
                        if victim.layer.height < EPSILON {
                            continue;
                        }
                        if victim.layer.print_z <= source_bottom + EPSILON {
                            break;
                        }
                        for areas in [&victim.roof_areas, &victim.roof_1st_layer] {
                            for area in areas {
                                if bbox_overlap(&base_bbox, &get_extents_expolygon(area)) {
                                    fine_interface.push(area.clone());
                                }
                            }
                        }
                    }
                    if !fine_interface.is_empty() && trim_layer(&mut self.m_object.m_support_layers[source_idx], &cx::union_ex(&fine_interface), &[]) {
                        trimmed_layers.push(source_idx);
                    }
                }
                let source_base: ExPolygons = self.m_object.m_support_layers[source_idx].base_areas.clone();
                let source_bbox = get_extents_expolygons(&source_base);

                let mut victim_idx = source_idx;
                while victim_idx > raft_layers {
                    victim_idx -= 1;
                    if (self.canceled)() {
                        return trimmed_layers;
                    }
                    let victim = &self.m_object.m_support_layers[victim_idx];
                    if victim.layer.height < EPSILON || victim.base_areas.is_empty() {
                        continue;
                    }
                    let victim_top: CoordF = victim.layer.print_z;
                    let victim_bottom: CoordF = victim_top - base_height(victim);
                    if victim_top <= source_bottom + EPSILON {
                        break;
                    }
                    if !bbox_overlap(&source_bbox, &get_extents_expolygons(&victim.base_areas)) {
                        continue;
                    }

                    let overlap: ExPolygons = cx::intersection_ex(&victim.base_areas, &source_base);
                    if overlap.is_empty() {
                        continue;
                    }
                    if source_bottom <= victim_bottom + EPSILON && source_top >= victim_top - EPSILON {
                        append(&mut base_cut[victim_idx], overlap);
                    }
                    // else: BOOST_LOG_TRIVIAL(warning) << "Folded support base overlaps only part of a base Z span ...; keeping both regions"
                }
            }
            if !is_folded_interface(&self.m_object.m_support_layers[source_idx]) {
                continue;
            }

            let source_top: CoordF = self.m_object.m_support_layers[source_idx].layer.print_z;
            let source_bottom: CoordF = source_top - self.m_object.m_support_layers[source_idx].interface_slab_height;
            let mut footprint: ExPolygons = self.m_object.m_support_layers[source_idx].roof_areas.clone();
            append(&mut footprint, self.m_object.m_support_layers[source_idx].roof_1st_layer.clone());
            if footprint.is_empty() {
                continue;
            }
            let footprint_bbox = get_extents_expolygons(&footprint);

            // the lambda `collect_cut` (TreeSupport.cpp:4840-4869)
            let collect_cut = |victim_print_z: CoordF, victim_height: CoordF, victim_areas: &ExPolygons, cut: &mut ExPolygons| {
                if victim_height < EPSILON || victim_areas.is_empty() {
                    return;
                }
                let victim_top: CoordF = victim_print_z;
                let victim_bottom: CoordF = victim_top - victim_height;
                if source_top.min(victim_top) <= source_bottom.max(victim_bottom) + EPSILON {
                    return;
                }
                if !bbox_overlap(&footprint_bbox, &get_extents_expolygons(victim_areas)) {
                    return;
                }

                let xy_overlap: ExPolygons = cx::intersection_ex(victim_areas, &footprint);
                if xy_overlap.is_empty() {
                    return;
                }
                let fully_covered = source_bottom <= victim_bottom + EPSILON && source_top >= victim_top - EPSILON;
                if fully_covered {
                    append(cut, xy_overlap);
                }
                // else: keep both regions in this unexpected fallback path. The carrier precheck handles known
                // partial overlaps; aborting here is worse than accepting localized over-extrusion.
            };

            {
                let source = &self.m_object.m_support_layers[source_idx];
                collect_cut(source.layer.print_z, base_height(source), &source.base_areas, &mut base_cut[source_idx]);
            }

            let mut victim_idx = source_idx;
            while victim_idx > raft_layers {
                victim_idx -= 1;
                if (self.canceled)() {
                    return trimmed_layers;
                }
                let victim = &self.m_object.m_support_layers[victim_idx];
                if victim.layer.height < EPSILON {
                    continue;
                }

                let victim_top: CoordF = victim.layer.print_z;
                if victim_top <= source_bottom + EPSILON {
                    break;
                }
                collect_cut(victim_top, interface_height(victim), &victim.roof_areas, &mut interface_cut[victim_idx]);
                collect_cut(victim_top, interface_height(victim), &victim.roof_1st_layer, &mut interface_cut[victim_idx]);
                collect_cut(victim_top, base_height(victim), &victim.base_areas, &mut base_cut[victim_idx]);
            }

            if !base_cut[source_idx].is_empty() {
                if trim_layer(&mut self.m_object.m_support_layers[source_idx], &base_cut[source_idx], &[]) {
                    trimmed_layers.push(source_idx);
                }
                base_cut[source_idx].clear();
            }
        }
        trimmed_layers
    }
}

// ============================================================================
// The toolpaths (TreeSupport.cpp:1598-1743, 2990-3496)
// ============================================================================

use crate::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionPath, ExtrusionRole};
use crate::fill::FillParams;
use crate::flow::{Flow, FlowResult};
use crate::print_config::InfillPattern;
use crate::support::support_common::{ExtrusionEntitiesPtr, SupportFill, SupportFillFactory};

/// `FillParams fill_params;` (FillBase.hpp:51-110): `density` 0, `ExtrusionRole(0)`, a default flow.
///
/// FIDELITY-NOTE: `FillParams::new()` of the crate starts at `density` 1, erInternalInfill and a flow.
fn default_fill_params() -> FillParams {
    let mut fill_params = FillParams::new();
    fill_params.density = 0.;
    fill_params.dont_adjust = false;
    fill_params.dont_sort = false;
    fill_params.extrusion_role = ExtrusionRole::None;
    fill_params.flow = Flow::default_constructed();
    fill_params
}

// MultiPoint.hpp `BoundingBox MultiPoint::bounding_box() const { return BoundingBox(this->points); }`
fn multipoint_bounding_box(points: &[Point]) -> BoundingBox {
    let mut bbox = BoundingBox::from_points(points);
    if !bbox.is_defined() {
        bbox.min = Point::new(0, 0);
        bbox.max = Point::new(0, 0);
    }
    bbox
}

// ExtrusionEntity.hpp:647-656 `inline void extrusion_entities_append_paths(ExtrusionEntitiesPtr &dst, Polylines &&polylines,
// ExtrusionRole role, double mm3_per_mm, float width, float height, bool can_reverse = true)`
fn extrusion_entities_append_paths(dst: &mut ExtrusionEntitiesPtr, polylines: crate::geometry::Polylines, role: ExtrusionRole, mm3_per_mm: f64, width: f32, height: f32) {
    crate::support::support_common::extrusion_entities_append_paths(dst, polylines, role, mm3_per_mm, width, height, true);
}

// ExtrusionEntity.hpp:732-744 `inline void extrusion_entities_append_loops(ExtrusionEntitiesPtr &dst, Polygons &&loops,
// ExtrusionRole role, double mm3_per_mm, float width, float height)`
fn extrusion_entities_append_loops(dst: &mut ExtrusionEntitiesPtr, loops: Polygons, role: ExtrusionRole, mm3_per_mm: f64, width: f32, height: f32) {
    dst.reserve(loops.len());
    for poly in loops {
        if poly.is_valid() {
            let mut path = ExtrusionPath::with_params(role, mm3_per_mm, width as f64, height as f64, false);
            path.polyline.points = poly.points;
            let front = path.polyline.points[0];
            path.polyline.points.push(front);
            dst.push(ExtrusionEntityType::Loop(ExtrusionLoop::from_path(path, ExtrusionLoopRole::default())));
        }
    }
}

// `ExtrusionEntity::first_point()` of an entity of a vector of `ExtrusionEntity*`.
fn extrusion_entity_first_point(entity: &ExtrusionEntityType) -> Point {
    match entity {
        ExtrusionEntityType::Path(path) => path.first_point(),
        ExtrusionEntityType::MultiPath(multipath) => multipath.first_point(),
        ExtrusionEntityType::Loop(loop_) => loop_.first_point(),
        // ExtrusionEntityCollection.hpp `Point first_point() const { return this->entities.front()->first_point(); }`
        ExtrusionEntityType::Collection(collection) => extrusion_entity_first_point(&collection.entities[0]),
    }
}

// `static_cast<ExtrusionEntityCollection *>(entity)->empty()` (TreeSupport.cpp:1730, ShortestPath.cpp:1034) on an
// entity that is not a collection. The cast reads the three words after the vtable pointer as the vector
// `entities`: of an ExtrusionLoop and an ExtrusionMultiPath that is `paths`, so the entity counts as empty when
// it has no paths; of an ExtrusionPath it is the vtable pointer of the polyline and the begin of its points,
// which never are equal.
fn extrusion_entity_empty_as_collection(entity: &ExtrusionEntityType) -> bool {
    match entity {
        ExtrusionEntityType::Path(_) => false,
        ExtrusionEntityType::MultiPath(multipath) => multipath.paths.is_empty(),
        ExtrusionEntityType::Loop(loop_) => loop_.paths.is_empty(),
        ExtrusionEntityType::Collection(collection) => collection.entities.is_empty(),
    }
}

// TreeSupport.cpp:1598-1623
// `Surface surface(stInternal, std::move(expolygon))` binds to `Surface(SurfaceType, const ExPolygon &&)`
// (Surface.hpp:70), which copies: the expolygon of the caller stays as it is.
pub fn fill_expolygon_generate_paths(
    dst: &mut ExtrusionEntitiesPtr,
    expolygon: &mut ExPolygon,
    filler: &mut dyn SupportFill,
    fill_params: &FillParams,
    role: ExtrusionRole,
    flow: &Flow,
) -> FlowResult<BoundingBox> {
    let surface = crate::surface::Surface::new(crate::surface::SurfaceType::Internal, expolygon.clone());
    // try { polylines = filler->fill_surface(&surface, fill_params); } catch (InfillFailedException &) {}
    let polylines: crate::geometry::Polylines = filler.fill_surface(&surface, fill_params).unwrap_or_default();

    // `BoundingBox fill_bbox;`: (0, 0)-(0, 0), not defined (BoundingBox.hpp:20).
    let mut fill_bbox = multipoint_bounding_box(&[]);
    if !polylines.is_empty() {
        fill_bbox = multipoint_bounding_box(&polylines[0].points);
        for polyline in &polylines {
            fill_bbox.merge(&multipoint_bounding_box(&polyline.points));
        }
    }

    extrusion_entities_append_paths(dst, polylines, role, flow.mm3_per_mm()?, flow.width() as f32, flow.height() as f32);

    Ok(fill_bbox)
}

// TreeSupport.cpp:1625-1639
pub fn fill_expolygons_generate_paths(
    dst: &mut ExtrusionEntitiesPtr,
    expolygons: &mut ExPolygons,
    filler: &mut dyn SupportFill,
    fill_params: &FillParams,
    role: ExtrusionRole,
    flow: &Flow,
) -> FlowResult<Vec<BoundingBox>> {
    let mut fill_boxes: Vec<BoundingBox> = Vec::new();
    for expoly in expolygons.iter_mut() {
        let box_ = fill_expolygon_generate_paths(dst, expoly, filler, fill_params, role, flow)?;
        fill_boxes.push(box_);
    }
    Ok(fill_boxes)
}

// TreeSupport.cpp:1641-1674
// `std::list<ExPolygon> expoly_list` with `std::map<ExPolygon *, int> depth_per_expoly`: the depth travels with
// its expolygon. The new expolygons are inserted at the head one by one and the old head is erased after them.
pub fn _make_loops(loops_entities: &mut ExtrusionEntitiesPtr, support_area: &mut ExPolygons, role: ExtrusionRole, wall_count: usize, flow: &Flow) -> FlowResult<()> {
    let mut loops: Polygons = Vec::new();
    let mut expoly_list: std::collections::VecDeque<(ExPolygon, i32)> = std::collections::VecDeque::new();

    for expoly in support_area.iter_mut() {
        expoly_list.push_back((std::mem::take(expoly), 0));
    }
    if expoly_list.is_empty() {
        return Ok(());
    }

    while let Some((front, depth)) = expoly_list.pop_front() {
        loops.extend(to_polygons(std::slice::from_ref(&front)));

        // `depth_iter->second + 1 < wall_count`: the int against a size_t
        if ((depth + 1) as usize) < wall_count {
            //ExPolygons expolys_new = offset_ex(expoly_list.front(), -float(flow.scaled_spacing()), jtSquare);
            // shrink and then dilate to prevent overlapping and overflow
            let scaled_spacing = flow.scaled_spacing() as f32;
            let expolys_new: ExPolygons = cx::offset2_ex(
                std::slice::from_ref(&front),
                (-1.4 * scaled_spacing as f64) as f32,
                (0.4 * scaled_spacing as f64) as f32,
                cx::DEFAULT_JOIN_TYPE,
                cx::DEFAULT_MITER_LIMIT,
            );

            for expoly in expolys_new {
                expoly_list.push_front((expoly, depth + 1));
            }
        }
    }

    extrusion_entities_append_loops(loops_entities, loops, role, (flow.mm3_per_mm()? as f32) as f64, flow.width() as f32, flow.height() as f32);
    Ok(())
}

// TreeSupport.cpp:1676-1681
pub fn make_perimeter_and_inner_brim(dst: &mut ExtrusionEntitiesPtr, support_area: &ExPolygon, wall_count: usize, flow: &Flow, role: ExtrusionRole) -> FlowResult<()> {
    let mut support_area_new: ExPolygons =
        cx::offset_ex_expolygon(support_area, -0.5f32 * flow.scaled_spacing() as f32, crate::clipper::JoinType::jtSquare, cx::DEFAULT_MITER_LIMIT);
    _make_loops(dst, &mut support_area_new, role, wall_count, flow)
}

// TreeSupport.cpp:1683-1743
#[allow(clippy::too_many_arguments)]
pub fn make_perimeter_and_infill(
    dst: &mut ExtrusionEntitiesPtr,
    support_area: &ExPolygon,
    wall_count: usize,
    flow: &Flow,
    role: ExtrusionRole,
    filler_support: &mut dyn SupportFill,
    support_density: f64,
    infill_first: bool,
) -> FlowResult<()> {
    use crate::clipper::JoinType;
    let mut support_area_new: ExPolygons = cx::offset_ex_expolygon(support_area, -0.5f32 * flow.scaled_spacing() as f32, JoinType::jtSquare, cx::DEFAULT_MITER_LIMIT);

    // draw infill
    let mut fill_params = default_fill_params();
    // `float density`
    fill_params.density = (support_density as f32) as f64;
    fill_params.dont_adjust = true;
    let mut to_infill: ExPolygons =
        cx::offset_ex_expolygon(support_area, -(wall_count as f32) * flow.scaled_spacing() as f32, JoinType::jtSquare, cx::DEFAULT_MITER_LIMIT);
    let fill_boxes: Vec<BoundingBox> = fill_expolygons_generate_paths(dst, &mut to_infill, filler_support, &fill_params, role, flow)?;

    // allow wall_count to be zero, which means only draw infill
    if wall_count == 0 {
        for fill_bbox in &fill_boxes {
            let mut fill_bbox = fill_bbox.clone();
            // extend bounding box on x-axis
            let angle: f32 = filler_support.fill().angle;
            if angle.cos() >= angle.sin() {
                fill_bbox.min.x = (fill_bbox.min.x as f64 - scale_(10.)) as Coord;
                fill_bbox.max.x = (fill_bbox.max.x as f64 + scale_(10.)) as Coord;
            } else {
                fill_bbox.min.y = (fill_bbox.min.y as f64 - scale_(10.)) as Coord;
                fill_bbox.max.y = (fill_bbox.max.y as f64 + scale_(10.)) as Coord;
            }
            // `to_expolygons({ fill_bbox.polygon() })`
            let boxes: ExPolygons = vec![ExPolygon::new(fill_bbox.polygon())];
            support_area_new = cx::diff_ex(
                &support_area_new,
                &cx::offset_ex(&boxes, (0.5 * flow.scaled_width() as f64) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
            );
        }
        // filter out small areas
        let scaled_width = flow.scaled_width();
        support_area_new.retain(|it| !cx::offset_ex_expolygon(it, -scaled_width as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT).is_empty());
    }

    {
        // draw loops
        let mut loops_entities: ExtrusionEntitiesPtr = Vec::new();
        _make_loops(&mut loops_entities, &mut support_area_new, role, wall_count, flow)?;

        if infill_first {
            dst.extend(loops_entities);
        } else {
            // loops first
            loops_entities.append(dst);
            *dst = loops_entities;
        }
    }
    dst.retain(|entity| !extrusion_entity_empty_as_collection(entity));
    if infill_first {
        // sort regions to reduce travel
        let mut ordering_points: Vec<Point> = Vec::new();
        for area in dst.iter() {
            ordering_points.push(extrusion_entity_first_point(area));
        }
        let order: Vec<usize> = crate::shortest_path::chain_points(&ordering_points, None);
        let mut old: Vec<Option<ExtrusionEntityType>> = std::mem::take(dst).into_iter().map(Some).collect();
        let mut new_dst: ExtrusionEntitiesPtr = Vec::with_capacity(ordering_points.len());
        for i in order {
            // `new_dst.emplace_back(dst[i])`: the order is a permutation
            new_dst.push(old[i].take().expect("chain_points: a permutation"));
        }
        *dst = new_dst;
    }
    Ok(())
}

// TreeSupport.cpp:3411-3423 the lambda `setOverhangDegreeImpl` of generate_toolpaths()
fn set_overhang_degree_impl(entities: &mut ExtrusionEntitiesPtr) {
    for entity in entities.iter_mut() {
        match entity {
            ExtrusionEntityType::Collection(collection) => set_overhang_degree_impl(&mut collection.entities),
            ExtrusionEntityType::Path(path) => path.set_overhang_degree(10),
            ExtrusionEntityType::MultiPath(multipath) => {
                for path in multipath.paths.iter_mut() {
                    path.set_overhang_degree(10);
                }
            }
            ExtrusionEntityType::Loop(loop_) => {
                for path in loop_.paths.iter_mut() {
                    path.set_overhang_degree(10);
                }
            }
        }
    }
}

// TreeSupport.cpp:3000-3010 the lambda `bridge_angle_on` of generate_toolpaths()
fn bridge_angle_on(obj_layer: Option<&Layer>, polys: &[ExPolygon], angle: &mut f64) -> bool {
    let Some(obj_layer) = obj_layer else {
        return false;
    };
    for layerm in obj_layer.regions() {
        for surface in &layerm.fill_surfaces.surfaces {
            // `overlaps(const ExPolygons &expolys, const ExPolygon &expoly)` (ExPolygon.cpp:197-204)
            if surface.surface_type == crate::surface::SurfaceType::BottomBridge && polys.iter().any(|el| cx::overlaps(el, &surface.expolygon)) {
                // Surface.hpp `double bridge_angle = -1.`
                *angle = surface.bridge_angle.unwrap_or(-1.);
                return true;
            }
        }
    }
    false
}

// PrintObject.cpp:6802-6812 `const Layer* PrintObject::get_layer_at_bottomz(coordf_t bottom_z, coordf_t epsilon) const`
fn get_layer_at_bottomz(layers: &[Layer], bottom_z: CoordF, epsilon: CoordF) -> Option<&Layer> {
    let limit_upper = bottom_z + epsilon;
    let limit_lower = bottom_z - epsilon;

    for layer in layers {
        if layer.bottom_z() > limit_lower {
            return if layer.bottom_z() < limit_upper { Some(layer) } else { None };
        }
    }

    None
}

impl<'a> TreeSupport<'a> {
    // TreeSupport.cpp:3011-3021 the lambda `bridge_angle_above` of generate_toolpaths()
    fn bridge_angle_above(&self, print_z: CoordF, polys: &[ExPolygon], angle: &mut f64, layer_height: CoordF) -> bool {
        let layers = self.m_object.layers();
        if self.m_top_z_outlines.is_empty() {
            return bridge_angle_on(
                get_layer_at_bottomz(layers, print_z + cfg_float(self.object_config(), "support_top_z_distance"), layer_height / 10.),
                polys,
                angle,
            );
        }
        let z_max: CoordF = print_z + self.m_top_z_max as f64 + self.m_slicing_params.max_layer_height + EPSILON;
        // std::upper_bound(..., print_z - EPSILON, [](coordf_t z, const Layer *layer) { return z < layer->bottom_z(); })
        let z = print_z - EPSILON;
        let first = layers.partition_point(|layer| !(z < layer.bottom_z()));
        let mut it = first;
        while it != layers.len() && layers[it].bottom_z() <= z_max {
            if bridge_angle_on(Some(&layers[it]), polys, angle) {
                return true;
            }
            it += 1;
        }
        false
    }

    // TreeSupport.cpp:3034-3070 the lambda `generate_support_ironing_entity` of generate_toolpaths()
    #[allow(clippy::too_many_arguments)]
    fn generate_support_ironing_entity(
        &self,
        fill_factory: &dyn SupportFillFactory,
        unioned_expolygons: &ExPolygons,
        bbox_object: &BoundingBox,
        ts_layer_id: usize,
        ts_layer_print_z: CoordF,
        temp_entity_collection: &mut ExtrusionEntitiesPtr,
        interface_flow: &Flow,
    ) -> FlowResult<()> {
        // support interface ironing related generation, tree support logic
        if self.m_support_params.enable_support_ironing && !unioned_expolygons.is_empty() {
            let mut ironing_fill = fill_factory.new_from_type(self.m_support_params.ironing_pattern);
            {
                let f = ironing_fill.fill_mut();
                f.bounding_box = bbox_object.clone();
                f.layer_id = ts_layer_id;
                f.z = ts_layer_print_z;
                f.overlap = 0.;
                f.angle = self.m_support_params.ironing_angle as f32;
                f.spacing = self.m_support_params.ironing_line_spacing;
                f.link_max_length = scale_(3. * f.spacing) as Coord;
            }

            let mut polys_to_iron: ExPolygons = unioned_expolygons.clone();
            if self.m_support_params.ironing_inset > EPSILON {
                polys_to_iron = cx::union_ex_polygons(&crate::support::support_common::cu::offset_expolygons(
                    &polys_to_iron,
                    (-scale_(self.m_support_params.ironing_inset)) as f32,
                    cx::DEFAULT_JOIN_TYPE,
                    cx::DEFAULT_MITER_LIMIT,
                ));
            }

            // `Flow(float w, float h, float nozzle_diameter)`: the height is a product in doubles
            let support_ironing_flow = Flow::new(
                interface_flow.width(),
                ((interface_flow.height() * self.m_support_params.ironing_flow_percent * 0.01) as f32) as f64,
                interface_flow.nozzle_diameter(),
            )?;
            let mut ironing_fill_params = default_fill_params();
            ironing_fill_params.density = 1.0;
            ironing_fill_params.dont_adjust = true;
            fill_expolygons_generate_paths(
                // Destination
                temp_entity_collection,
                // Regions to fill
                &mut polys_to_iron,
                // Filler and its parameters
                ironing_fill.as_mut(),
                &ironing_fill_params,
                // Extrusion parameters
                ExtrusionRole::SupportIroning,
                &support_ironing_flow,
            )?;
        }
        Ok(())
    }

    /// TreeSupport.cpp:2990-3496 `void TreeSupport::generate_toolpaths()`
    ///
    /// `Fill::new_from_type()` is `fill_factory` (`SupportFillFactory` of support_common.rs: the fillers of the
    /// crate are not behind one base class). An exception of the C++ (a flow that cannot be made) is the `Err`.
    ///
    /// FIDELITY-NOTE: the tbb::parallel_for over the support layers runs in order: every iteration writes the
    /// extrusions of its own layer. The lightning infill of the bases needs `generator`, which draw_circles()
    /// only has when the caller handed in a `LightningSupportGenerator`; without one a layer is left as the C++
    /// leaves a layer the generator has no entry for.
    pub fn generate_toolpaths(&mut self, fill_factory: &dyn SupportFillFactory) -> FlowResult<()> {
        use crate::print::faithful::SupportLayerAreaType;
        use crate::print_config::enums::{SupportMaterialInterfacePattern, SupportMaterialPattern};
        const M_PI_2: f64 = std::f64::consts::FRAC_PI_2;
        let deg2rad = |angle: f64| -> f64 { M_PI * angle / 180.0 };

        let support_extrusion_width: CoordF = self.m_support_params.support_extrusion_width;
        let nozzle_diameter: CoordF = support_nozzle_diameter(self.m_print, cfg_int(self.object_config(), "support_filament") as u32) as CoordF;
        let layer_height: CoordF = cfg_float(self.object_config(), "layer_height");
        let wall_count: i32 = cfg_int(self.object_config(), "tree_support_wall_count");
        let support_interface_spacing_cfg = cfg_float(self.object_config(), "support_interface_spacing");
        let support_base_pattern_spacing = cfg_float(self.object_config(), "support_base_pattern_spacing");
        let raft_first_layer_density = cfg_float(self.object_config(), "raft_first_layer_density");
        let raft_first_layer_expansion = cfg_float(self.object_config(), "raft_first_layer_expansion");
        let support_angle = cfg_float(self.object_config(), "support_angle");
        let support_interface_pattern = cfg_int(self.object_config(), "support_interface_pattern");
        let object_support_base_pattern = cfg_int(self.object_config(), "support_base_pattern");

        // coconut: use same intensity settings as SupportMaterial.cpp
        let m_support_material_interface_flow =
            crate::support::support_parameters::support_material_interface_flow_of(self.m_object, self.m_print, self.m_slicing_params.layer_height as f32)?;
        let interface_spacing: CoordF = support_interface_spacing_cfg + m_support_material_interface_flow.spacing();
        let interface_density: CoordF = {
            let v = m_support_material_interface_flow.spacing() / interface_spacing;
            if v < 1. {
                v
            } else {
                1.
            }
        };

        if self.m_object.support_layers().is_empty() {
            return Ok(());
        }

        // calculate fill areas for raft layers
        let mut raft_areas: ExPolygons = Vec::new();
        if self.m_object.layer_count() > 0 {
            let layer = &self.m_object.layers()[0];
            for expoly in &layer.lslices {
                raft_areas.push(expoly.clone());
            }
        }

        if self.m_object.support_layer_count() > self.m_raft_layers {
            let ts_layer = &self.m_object.m_support_layers[self.m_raft_layers];
            for expoly in &ts_layer.floor_areas {
                raft_areas.push(expoly.clone());
            }
            for expoly in &ts_layer.roof_areas {
                raft_areas.push(expoly.clone());
            }
            for expoly in &ts_layer.base_areas {
                raft_areas.push(expoly.clone());
            }
        }

        raft_areas = cx::offset_ex(&raft_areas, scale_(3.) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);

        let mut layer_nr: usize = 0;
        while layer_nr < self.m_slicing_params.base_raft_layers {
            let ts_height = self.m_object.m_support_layers[layer_nr].layer.height;
            let expand_offset: CoordF = if layer_nr == 0 { raft_first_layer_expansion } else { 0. };
            let mut raft_areas1 = cx::offset_ex(&raft_areas, scale_(expand_offset) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);

            let support_flow = Flow::new(support_extrusion_width, ts_height, nozzle_diameter)?;
            let mut filler_raft = fill_factory.new_from_type(InfillPattern::Rectilinear);
            filler_raft.fill_mut().angle = if layer_nr == 0 { (M_PI / 2.) as f32 } else { 0. };
            filler_raft.fill_mut().spacing = support_flow.spacing();

            let mut fill_params = default_fill_params();
            let raft_density: CoordF = {
                let v = support_flow.spacing() / (support_base_pattern_spacing + support_flow.spacing());
                if v < 1. {
                    v
                } else {
                    1.
                }
            };
            fill_params.density = ((if layer_nr == 0 { raft_first_layer_density * 0.01 } else { raft_density }) as f32) as f64;
            fill_params.dont_adjust = true;

            let mut entities = std::mem::take(&mut self.m_object.m_support_layers[layer_nr].support_fills.entities);
            // wall of first layer raft
            if layer_nr == 0 {
                let flow = Flow::new(support_extrusion_width, ts_height, nozzle_diameter)?;
                extrusion_entities_append_loops(
                    &mut entities,
                    to_polygons(&raft_areas1),
                    ExtrusionRole::SupportMaterial,
                    (flow.mm3_per_mm()? as f32) as f64,
                    flow.width() as f32,
                    flow.height() as f32,
                );
                raft_areas1 = cx::offset_ex(&raft_areas1, (-(flow.scaled_spacing() as f64) / 2.) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
            }
            let res = fill_expolygons_generate_paths(&mut entities, &mut raft_areas1, filler_raft.as_mut(), &fill_params, ExtrusionRole::SupportMaterial, &support_flow);
            self.m_object.m_support_layers[layer_nr].support_fills.entities = entities;
            res?;
            layer_nr += 1;
        }

        // subtract the non-raft support bases, otherwise we'll get support base on top of raft interfaces which is not stable
        let mut first_non_raft_base: ExPolygons = Vec::new();
        if let Some(first_non_raft_layer) = self.m_object.m_support_layers.get(self.m_raft_layers) {
            for area_group in &first_non_raft_layer.area_groups {
                if area_group.type_ == SupportLayerAreaType::BaseType {
                    first_non_raft_base.push(first_non_raft_layer.area_group_area(area_group).clone());
                }
            }
        }
        // `offset_ex(first_non_raft_base, support_extrusion_width)`: the width in mm is the delta, as the C++ has it.
        first_non_raft_base = cx::offset_ex(&first_non_raft_base, support_extrusion_width as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
        let mut raft_base_areas: ExPolygons = cx::intersection_ex(&raft_areas, &first_non_raft_base);
        let mut raft_interface_areas: ExPolygons = cx::diff_ex(&raft_areas, &raft_base_areas);

        // raft interfaces
        layer_nr = self.m_slicing_params.base_raft_layers;
        while layer_nr < self.m_slicing_params.base_raft_layers + self.m_slicing_params.interface_raft_layers {
            let (ts_height, ts_id, ts_print_z) = {
                let ts_layer = &self.m_object.m_support_layers[layer_nr];
                (ts_layer.layer.height, ts_layer.layer.id(), ts_layer.layer.print_z)
            };

            let support_flow = Flow::new(support_extrusion_width, ts_height, nozzle_diameter)?;
            let mut filler_interface = fill_factory.new_from_type(InfillPattern::Rectilinear);
            filler_interface.fill_mut().angle = (M_PI / 2.) as f32; // interface should be perpendicular to base
            filler_interface.fill_mut().spacing = support_flow.spacing();

            let mut fill_params = default_fill_params();
            fill_params.density = (interface_density as f32) as f64;
            fill_params.dont_adjust = true;

            let mut entities = std::mem::take(&mut self.m_object.m_support_layers[layer_nr].support_fills.entities);
            let res = (|| -> FlowResult<()> {
                fill_expolygons_generate_paths(
                    &mut entities,
                    &mut raft_interface_areas,
                    filler_interface.as_mut(),
                    &fill_params,
                    ExtrusionRole::SupportMaterialInterface,
                    &support_flow,
                )?;

                if self.m_support_params.enable_support_ironing
                    && !raft_interface_areas.is_empty()
                    && layer_nr == self.m_slicing_params.base_raft_layers + self.m_slicing_params.interface_raft_layers - 1
                {
                    let bbox_object = BoundingBox::from_points_minmax(point_from_f64(-scale_(1.), -scale_(1.0)), point_from_f64(scale_(1.), scale_(1.)));
                    self.generate_support_ironing_entity(fill_factory, &raft_interface_areas, &bbox_object, ts_id, ts_print_z, &mut entities, &support_flow)?;
                }

                fill_params.density = ((raft_first_layer_density * 0.01) as f32) as f64;
                fill_expolygons_generate_paths(&mut entities, &mut raft_base_areas, filler_interface.as_mut(), &fill_params, ExtrusionRole::SupportMaterial, &support_flow)?;
                Ok(())
            })();
            self.m_object.m_support_layers[layer_nr].support_fills.entities = entities;
            res?;
            layer_nr += 1;
        }

        // layers between raft and object
        while layer_nr < self.m_raft_layers {
            let ts_height = self.m_object.m_support_layers[layer_nr].layer.height;
            let support_flow = Flow::new(support_extrusion_width, ts_height, nozzle_diameter)?;
            let mut filler_raft = fill_factory.new_from_type(InfillPattern::Rectilinear);
            filler_raft.fill_mut().angle = (M_PI / 2.) as f32;
            filler_raft.fill_mut().spacing = support_flow.spacing();
            let mut entities = std::mem::take(&mut self.m_object.m_support_layers[layer_nr].support_fills.entities);
            let mut res: FlowResult<()> = Ok(());
            for poly in &first_non_raft_base {
                // `std::min(1, wall_count)` handed to a size_t
                res = make_perimeter_and_infill(
                    &mut entities,
                    poly,
                    1.min(wall_count) as usize,
                    &support_flow,
                    ExtrusionRole::SupportMaterial,
                    filler_raft.as_mut(),
                    interface_density,
                    false,
                );
                if res.is_err() {
                    break;
                }
            }
            self.m_object.m_support_layers[layer_nr].support_fills.entities = entities;
            res?;
            layer_nr += 1;
        }

        if self.m_object.support_layer_count() <= self.m_raft_layers {
            return Ok(());
        }

        // generate tree support tool paths
        for layer_id in self.m_raft_layers..self.m_object.support_layer_count() {
            if (self.canceled)() {
                break;
            }

            let mut entities = std::mem::take(&mut self.m_object.m_support_layers[layer_id].support_fills.entities);
            let res = self.generate_toolpaths_layer(
                fill_factory,
                layer_id,
                &mut entities,
                support_extrusion_width,
                nozzle_diameter,
                layer_height,
                wall_count,
                support_interface_spacing_cfg,
                support_base_pattern_spacing,
                raft_first_layer_density,
                support_angle,
                support_interface_pattern == SupportMaterialInterfacePattern::SmipGrid as i32,
                object_support_base_pattern == SupportMaterialPattern::SmpDefault as i32,
                deg2rad,
                M_PI_2,
            );
            let ts_layer = &mut self.m_object.m_support_layers[layer_id];
            ts_layer.support_fills.entities = entities;
            // sort extrusions to reduce travel, also make sure walls go before infills
            if res? && !ts_layer.support_fills.no_sort {
                crate::shortest_path::chain_and_reorder_extrusion_entities(&mut ts_layer.support_fills.entities, None);
            }
        }
        Ok(())
    }

    // TreeSupport.cpp:3184-3488: the body of the tbb::parallel_for of generate_toolpaths() for one support layer, up
    // to the sort of its extrusions. `Ok(false)` is one of the `continue`s of the lightning block, which leave
    // the layer without the sort.
    #[allow(clippy::too_many_arguments)]
    fn generate_toolpaths_layer(
        &mut self,
        fill_factory: &dyn SupportFillFactory,
        layer_id: usize,
        entities: &mut ExtrusionEntitiesPtr,
        support_extrusion_width: CoordF,
        nozzle_diameter: CoordF,
        layer_height: CoordF,
        wall_count: i32,
        support_interface_spacing_cfg: CoordF,
        support_base_pattern_spacing: CoordF,
        raft_first_layer_density: CoordF,
        support_angle: CoordF,
        interface_pattern_grid: bool,
        object_base_pattern_default: bool,
        deg2rad: impl Fn(f64) -> f64,
        m_pi_2: f64,
    ) -> FlowResult<bool> {
        use crate::print::faithful::SupportLayerAreaType;
        let min1 = |v: f64| -> f64 {
            if v < 1. {
                v
            } else {
                1.
            }
        };
        let bbox_object = BoundingBox::from_points_minmax(point_from_f64(-scale_(1.), -scale_(1.0)), point_from_f64(scale_(1.), scale_(1.)));
        let mut filler_interface = fill_factory.new_from_type(self.m_support_params.contact_fill_pattern);
        let mut filler_roof1st_layer = fill_factory.new_from_type(InfillPattern::Rectilinear);

        let support_layer_count = self.m_object.support_layer_count();
        let (ts_height, ts_print_z, ts_id, base_slab_height, interface_slab_height, planned_nozzle_diameter, planned_interface_nozzle_diameter, area_groups) = {
            let ts_layer = &self.m_object.m_support_layers[layer_id];
            (
                ts_layer.layer.height,
                ts_layer.layer.print_z,
                ts_layer.layer.id(),
                ts_layer.base_slab_height,
                ts_layer.interface_slab_height,
                ts_layer.planned_nozzle_diameter as f64,
                ts_layer.planned_interface_nozzle_diameter as f64,
                ts_layer.area_groups.clone(),
            )
        };
        let base_height: CoordF = if base_slab_height > EPSILON { base_slab_height } else { ts_height };
        // An interface folded onto the bed band reaches the bed as well, even when no base was folded
        // onto this layer, and draw_circles() already gave it the first layer expansion. Its brim then
        // has to be extruded at the interface slab's height and with the interface slab's nozzle.
        let interface_only_bed = interface_slab_height > base_height + EPSILON;
        let bed_height: CoordF = if interface_only_bed { interface_slab_height } else { base_height };
        let bed_nozzle: f64 = if interface_only_bed { planned_interface_nozzle_diameter } else { planned_nozzle_diameter };
        let base_starts_on_bed = self.m_raft_layers == 0 && ts_print_z - bed_height <= EPSILON;
        let support_flow: Flow = if planned_nozzle_diameter > EPSILON {
            self.m_support_params.support_material_flow_for_nozzle(planned_nozzle_diameter, base_height as f32)?
        } else {
            Flow::new(support_extrusion_width, base_height, nozzle_diameter)?
        };
        let first_layer_support_flow: Flow = if bed_nozzle > EPSILON {
            self.m_support_params.support_material_first_layer_flow_for_nozzle(bed_nozzle, bed_height as f32)?
        } else {
            self.m_support_params.first_layer_flow.clone()
        };
        let interface_height: CoordF = if interface_slab_height > EPSILON { interface_slab_height } else { ts_height };
        let interface_flow: Flow = if planned_interface_nozzle_diameter > EPSILON {
            self.m_support_params.support_material_interface_flow_for_nozzle(planned_interface_nozzle_diameter, interface_height as f32)?
        } else {
            crate::support::support_parameters::support_material_interface_flow_of(self.m_object, self.m_print, interface_height as f32)?
        };
        // The interface is folded on the node grid now, so the roof and the transition layer below it
        // belong to the same slab and share its height. The floor is the one interface region that is
        // never folded - it is the bottom contact, generated per layer in postprocess_base_areas() -
        // so it keeps a flow of its own even when the roof above got pinned to a nozzle.
        let layer_interface_density: CoordF = min1(interface_flow.spacing() / (support_interface_spacing_cfg + interface_flow.spacing()));
        let floor_flow: Flow = if planned_interface_nozzle_diameter > EPSILON {
            self.m_support_params.support_material_interface_flow_for_nozzle(planned_interface_nozzle_diameter, ts_height as f32)?
        } else {
            crate::support::support_parameters::support_material_interface_flow_of(self.m_object, self.m_print, ts_height as f32)?
        };
        let floor_density: CoordF = min1(floor_flow.spacing() / (support_interface_spacing_cfg + floor_flow.spacing()));
        let support_spacing: CoordF = support_base_pattern_spacing + support_flow.spacing();
        let support_density: CoordF = min1(support_flow.spacing() / support_spacing);
        self.m_object.m_support_layers[layer_id].support_fills.no_sort = false;

        for area_group in &area_groups {
            filler_interface.fill_mut().bounding_box = bbox_object.clone();
            filler_roof1st_layer.fill_mut().bounding_box = bbox_object.clone();
            filler_interface.fill_mut().angle = deg2rad(support_angle + 90.) as f32;
            filler_roof1st_layer.fill_mut().angle = deg2rad(support_angle + 90.) as f32;

            let poly: ExPolygon = self.m_object.m_support_layers[layer_id].area_group_area(area_group).clone();
            let mut polys: ExPolygons = Vec::new();
            let mut fill_params = default_fill_params();
            if area_group.type_ != SupportLayerAreaType::BaseType {
                // interface
                // BBS multi-diameter: the bed band is emitted on its top layer once folded, so the
                // brim belongs to whichever layer's material actually reaches the bed.
                if base_starts_on_bed {
                    // This brim belongs to the tree support, so its flow must follow support_filament.
                    // Print::brim_flow() is derived from a model wall filament and may use a different
                    // nozzle diameter in multi-diameter mode.
                    let flow = first_layer_support_flow.clone();
                    make_perimeter_and_inner_brim(
                        entities,
                        &poly,
                        wall_count as usize,
                        &flow,
                        if area_group.type_ == SupportLayerAreaType::RoofType { ExtrusionRole::SupportMaterialInterface } else { ExtrusionRole::SupportMaterial },
                    )?;
                    polys = cx::offset_ex_expolygon(&poly, -flow.scaled_spacing() as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
                } else if area_group.type_ == SupportLayerAreaType::Roof1stLayer {
                    polys = cx::offset_ex_expolygon(&poly, (0.5 * support_flow.scaled_width() as f64) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
                } else {
                    polys.push(poly.clone());
                }
                fill_params.density = (layer_interface_density as f32) as f64;
                fill_params.dont_adjust = true;
            }
            if area_group.type_ == SupportLayerAreaType::Roof1stLayer {
                // use higher flow for roof_1st_layer ?
                let roof_1st_layer_flow = interface_flow.clone(); /*.with_flow_ratio(1.5)*/

                // Note: spacing means the separation between two lines as if they are tightly extruded
                filler_roof1st_layer.fill_mut().spacing = roof_1st_layer_flow.spacing();

                // BBS multi-diameter: the interface sits N *printed* layers above, and a folded slab
                // swallows the ones in between - draw_circles() zeroes their height. Adding the raw
                // layer count would land inside a slab, so walk the layers that actually print.
                let mut interface_layer_above: Option<usize> = None;
                if self.m_fold_carriers.is_empty() {
                    let idx = layer_id + self.m_support_params.num_top_interface_layers;
                    if idx < support_layer_count {
                        interface_layer_above = Some(idx);
                    }
                } else {
                    let mut left = self.m_support_params.num_top_interface_layers as i32;
                    let mut probe = layer_id as i32 + 1;
                    while left > 0 && probe < support_layer_count as i32 {
                        let candidate = &self.m_object.m_support_layers[probe as usize];
                        if candidate.layer.height < EPSILON {
                            probe += 1;
                            continue;
                        }
                        left -= 1;
                        if left == 0 {
                            interface_layer_above = Some(probe as usize);
                        }
                        probe += 1;
                    }
                }
                let mut bridge_angle = 0.;
                if let Some(above) = interface_layer_above {
                    let above_print_z = self.m_object.m_support_layers[above].layer.print_z;
                    if self.bridge_angle_above(above_print_z, &polys, &mut bridge_angle, layer_height) {
                        // `bridge_angle + (num_top_interface_layers - 1) * M_PI_2`: the size_t wraps at 0
                        filler_roof1st_layer.fill_mut().angle =
                            ((self.m_support_params.num_top_interface_layers.wrapping_sub(1)) as f64).mul_add(m_pi_2, bridge_angle) as f32;
                    }
                }

                // generate a perimeter first to support interface better
                let mut temp_support_fills = ExtrusionEntityCollection::new();
                make_perimeter_and_infill(
                    &mut temp_support_fills.entities,
                    &poly,
                    1,
                    &roof_1st_layer_flow,
                    ExtrusionRole::SupportTransition,
                    filler_roof1st_layer.as_mut(),
                    layer_interface_density,
                    false,
                )?;
                temp_support_fills.no_sort = true; // make sure loops are first
                if !temp_support_fills.entities.is_empty() {
                    entities.push(ExtrusionEntityType::Collection(Box::new(temp_support_fills)));
                }
            } else if area_group.type_ == SupportLayerAreaType::FloorType {
                // floor_areas
                fill_params.density = (floor_density as f32) as f64;
                filler_interface.fill_mut().spacing = floor_flow.spacing();
                fill_expolygons_generate_paths(entities, &mut polys, filler_interface.as_mut(), &fill_params, ExtrusionRole::SupportMaterialInterface, &floor_flow)?;
            } else if area_group.type_ == SupportLayerAreaType::RoofType {
                // roof_areas
                fill_params.density = (layer_interface_density as f32) as f64;
                filler_interface.fill_mut().spacing = interface_flow.spacing();
                if interface_pattern_grid {
                    filler_interface.fill_mut().angle = deg2rad(support_angle) as f32;
                    fill_params.dont_sort = true;
                }
                if self.m_support_params.contact_fill_pattern == InfillPattern::Rectilinear {
                    let mut bridge_found = false;
                    for i in 0..self.m_support_params.num_top_interface_layers {
                        if layer_id + i >= support_layer_count {
                            break;
                        }
                        let cur_print_z = self.m_object.m_support_layers[layer_id + i].layer.print_z;
                        let mut bridge_angle = 0.;
                        if self.bridge_angle_above(cur_print_z, &polys, &mut bridge_angle, layer_height) {
                            filler_interface.fill_mut().angle = ((i + 1) as f64).mul_add(m_pi_2, bridge_angle) as f32;
                            bridge_found = true;
                            break;
                        }
                    }
                    if !bridge_found {
                        filler_interface.fill_mut().layer_id = area_group.interface_id as usize;
                    } else {
                        filler_interface.fill_mut().layer_id = 0;
                    }
                }

                let mut temp_support_fills = ExtrusionEntityCollection::new();
                for single_poly in &cx::union_ex(&polys) {
                    make_perimeter_and_infill(
                        &mut temp_support_fills.entities,
                        single_poly,
                        1,
                        &interface_flow,
                        ExtrusionRole::SupportMaterialInterface,
                        filler_interface.as_mut(),
                        layer_interface_density,
                        false,
                    )?;
                }
                temp_support_fills.no_sort = true; // make sure loops are first
                if !temp_support_fills.entities.is_empty() {
                    entities.push(ExtrusionEntityType::Collection(Box::new(temp_support_fills)));
                }

                if self.m_support_params.enable_support_ironing {
                    let mut temp_support_ironings = ExtrusionEntityCollection::new();
                    let mut unioned_expolygons: ExPolygons = cx::union_ex(&polys);
                    let mut upper_expolygons: ExPolygons = Vec::new();
                    if let Some(ts_upper_layer) = self.m_object.m_support_layers.get(layer_id + 1) {
                        for area_t in &ts_upper_layer.area_groups {
                            upper_expolygons.push(ts_upper_layer.area_group_area(area_t).clone());
                        }
                        unioned_expolygons = cx::diff_ex(&unioned_expolygons, &upper_expolygons);
                        unioned_expolygons = cx::union_ex(&unioned_expolygons);
                    }
                    if !unioned_expolygons.is_empty() {
                        self.generate_support_ironing_entity(
                            fill_factory,
                            &unioned_expolygons,
                            &bbox_object,
                            ts_id,
                            ts_print_z,
                            &mut temp_support_ironings.entities,
                            &interface_flow,
                        )?;
                    }
                    temp_support_ironings.no_sort = true; // make sure loops are first
                    if !temp_support_ironings.entities.is_empty() {
                        entities.push(ExtrusionEntityType::Collection(Box::new(temp_support_ironings)));
                    }
                }
            } else {
                // base_areas
                let flow: Flow = if base_starts_on_bed { first_layer_support_flow.clone() } else { support_flow.clone() };
                let mut need_infill = self.with_infill;
                if object_base_pattern_default {
                    need_infill &= area_group.need_infill;
                }

                let mut filler_support =
                    fill_factory.new_from_type(if base_starts_on_bed { InfillPattern::Concentric } else { self.m_support_params.base_fill_pattern });
                filler_support.fill_mut().bounding_box = bbox_object.clone();
                filler_support.fill_mut().spacing = flow.spacing();
                filler_support.fill_mut().angle = deg2rad(support_angle) as f32;

                let loops: Polygons = to_polygons(std::slice::from_ref(&poly));
                if base_starts_on_bed {
                    let density: f32 = (raft_first_layer_density * 0.01) as f32;
                    crate::support::support_common::fill_expolygons_with_sheath_generate_paths(
                        entities,
                        &loops,
                        filler_support.as_mut(),
                        density,
                        ExtrusionRole::SupportMaterial,
                        &flow,
                        &self.m_support_params,
                        true,
                        false,
                    )?;
                } else if need_infill && self.m_support_params.base_fill_pattern != InfillPattern::Lightning {
                    // allow infill-only mode if support is thick enough (so min_wall_count is 0);
                    // otherwise must draw 1 wall
                    // Don't need extra walls if we have infill. Extra walls may overlap with the infills.
                    let min_wall_count: usize = if wall_count == 0 { 0 } else { 1 };
                    make_perimeter_and_infill(
                        entities,
                        &poly,
                        (min_wall_count as i32).max(wall_count) as usize,
                        &flow,
                        ExtrusionRole::SupportMaterial,
                        filler_support.as_mut(),
                        support_density,
                        true,
                    )?;
                } else {
                    // BBS multi-diameter: an area widened at a nozzle change also gets the second wall
                    // when the wall count is automatic. Its paths are inset by half their own width, so
                    // the bead's outer edge stays on the contour and it grows inwards onto the coarse bead.
                    let widened = area_group.line_width > flow.width() + EPSILON;
                    let mut support_params = self.m_support_params.clone();
                    if (area_group.need_extra_wall || widened) && wall_count == -1 {
                        support_params.tree_branch_diameter_double_wall_area_scaled = 0.1;
                    }
                    let wall_flow: Flow = if widened { flow.with_width((area_group.line_width as f32) as f64)? } else { flow.clone() };
                    crate::support::support_common::tree_supports_generate_paths(entities, &loops, &wall_flow, &support_params)?;
                }

                if area_group.need_cooling {
                    // `setOverhangDegreeImpl(&ts_layer->support_fills)`: every extrusion the layer has so far
                    set_overhang_degree_impl(entities);
                }
            }
        }
        if self.m_support_params.base_fill_pattern == InfillPattern::Lightning {
            // A base that reaches the bed was already filled above, concentrically and at the raft
            // first layer density, and that path has no lightning guard of its own. Laying a
            // lightning scaffold over it would extrude the area twice. Unfolded, this never showed:
            // the bed band is support layer 0, which never becomes a lower_layer and so never gets
            // a generator entry. Folded, the band is emitted on its top layer, which does.
            if base_starts_on_bed {
                return Ok(false);
            }
            let print_z = ts_print_z;
            let Some(&lightning_layer_idx) = self.print_z_to_lightninglayer.get(&f64_key(print_z)) else {
                return Ok(false);
            };
            // See the FIDELITY-NOTE of generate_toolpaths().
            let Some(generator) = self.generator.as_ref() else {
                return Ok(false);
            };
            let lightning_layer = generator.get_trees_for_layer(lightning_layer_idx);

            let flow: Flow = if base_starts_on_bed { first_layer_support_flow.clone() } else { support_flow.clone() };
            let areas: ExPolygons =
                cx::offset_ex(&self.m_object.m_support_layers[layer_id].base_areas, -flow.scaled_spacing() as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);

            for area in &areas {
                let mut polylines: crate::geometry::Polylines = lightning_layer.convert_to_lines(&to_polygons(std::slice::from_ref(area)), 0);
                polylines.retain(|itr| !(itr.length() < scale_(1.0)));
                //this wont create connection patterns along contours
                let opt_polylines: crate::geometry::Polylines = crate::shortest_path::chain_polylines(polylines, None);
                extrusion_entities_append_paths(
                    entities,
                    opt_polylines,
                    ExtrusionRole::SupportMaterial,
                    (flow.mm3_per_mm()? as f32) as f64,
                    flow.width() as f32,
                    flow.height() as f32,
                );
            }
        }
        Ok(true)
    }
}

// ============================================================================
// TreeSupport::detect_overhangs (TreeSupport.cpp:795-1550)
// ============================================================================

/// `void PrintObject::project_and_append_custom_facets(bool seam, EnforcerBlockerType type, std::vector<Polygons> &out,
/// std::vector<std::pair<Vec3f, Vec3f>> *vertical_points) const` (PrintObject.cpp:6726-6765), which detect_overhangs()
/// calls for the painted support blockers and enforcers. The crate has no port of it over the faithful PrintObject
/// (its `slice_mesh_slabs` has no `vertical_points` output), so the caller hands it in; without one the object
/// counts as not painted, which is what the C++ does for a model without painted facets.
pub trait CustomFacetsProjector {
    fn project_and_append_custom_facets(
        &self,
        object: &PrintObject,
        print: &Print,
        seam: bool,
        type_: crate::triangle_selector::EnforcerBlockerType,
        out: &mut Vec<Polygons>,
        vertical_points: Option<&mut Vec<([f32; 3], [f32; 3])>>,
    );
}

/// `BoundingBox` as BoundingBox.hpp has it: (0, 0)-(0, 0) and not defined when default constructed, defined by a
/// range of points only when it has a positive area (BoundingBox.hpp:20, 88-102).
#[derive(Debug, Clone, Copy)]
struct BBoxCpp {
    min: Point,
    max: Point,
    defined: bool,
}

impl BBoxCpp {
    fn new() -> Self {
        BBoxCpp { min: Point::new(0, 0), max: Point::new(0, 0), defined: false }
    }
    // BoundingBox.hpp:88-102 `construct<false>(out, from, to)`
    fn from_points(points: &[Point]) -> Self {
        let mut out = BBoxCpp::new();
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
    // BoundingBox.cpp `void BoundingBoxBase::merge(const BoundingBoxBase &bb)`
    fn merge(&mut self, bb: &BBoxCpp) {
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
    // BoundingBox.cpp `PointClass BoundingBoxBase::size() const { return PointClass(this->max - this->min); }`
    fn size(&self) -> Point {
        Point::new(self.max.x - self.min.x, self.max.y - self.min.y)
    }
    // BoundingBox.hpp:44 `double area() const { return double(this->max(0) - this->min(0)) * (this->max(1) - this->min(1)); }`
    fn area(&self) -> f64 {
        (self.max.x - self.min.x) as f64 * (self.max.y - self.min.y) as f64
    }
    // BoundingBox.hpp:58-61
    fn overlap(&self, other: &BBoxCpp) -> bool {
        !(self.max.x < other.min.x || self.min.x > other.max.x || self.max.y < other.min.y || self.min.y > other.max.y)
    }
    // BoundingBox.hpp:51-54
    fn contains(&self, point: &Point) -> bool {
        point.x >= self.min.x && point.x <= self.max.x && point.y >= self.min.y && point.y <= self.max.y
    }
    // ExPolygon.cpp:490-493 `BoundingBox get_extents(const ExPolygon &expolygon)`
    fn of_expolygon(expolygon: &ExPolygon) -> Self {
        BBoxCpp::from_points(&expolygon.contour.points)
    }
    // ExPolygon.cpp:495-504 `BoundingBox get_extents(const ExPolygons &expolygons)`
    fn of_expolygons(expolygons: &[ExPolygon]) -> Self {
        let mut bbox = BBoxCpp::new();
        for expolygon in expolygons {
            if !expolygon.contour.points.is_empty() {
                bbox.merge(&BBoxCpp::of_expolygon(expolygon));
            }
        }
        bbox
    }
}

// ExPolygon.hpp:486-492 `inline double area(const ExPolygons &polys)`
fn area_of_expolygons(polys: &[ExPolygon]) -> f64 {
    let mut s = 0.;
    for p in polys {
        s += p.area();
    }
    s
}

// ExPolygon.hpp:352-359 `inline ExPolygons to_expolygons(const Polygons &polys)`: every polygon a contour.
fn to_expolygons_of_polygons(polys: &[Polygon]) -> ExPolygons {
    polys.iter().map(|p| ExPolygon::new(p.clone())).collect()
}

// MultiPoint.hpp:53-74 `double MultiPoint::distance_to(const Point &point) const`: the distance to the closest of
// the points (`closest_point_index`), not to the contour.
fn multipoint_distance_to(points: &[Point], point: &Point) -> f64 {
    let mut idx = 0usize;
    let mut dist_min = point_dist(*point, points[0]);
    for (i, p) in points.iter().enumerate().skip(1) {
        let d = point_dist(*p, *point);
        if d < dist_min {
            dist_min = d;
            idx = i;
        }
    }
    point_dist(points[idx], *point)
}

/// TreeSupport.cpp:841-981 `struct OverhangCluster` of detect_overhangs(): for small overhang removal.
struct OverhangCluster {
    layer_overhangs: BTreeMap<i32, ExPolygons>,
    merged_poly: ExPolygons,
    merged_bbox: BBoxCpp,
    min_layer: i32,
    max_layer: i32,
    offset: CoordF,
    type_: OverhangType,
    small_reason: Option<&'static str>,
}

impl OverhangCluster {
    // 861-863 `OverhangCluster(const ExPolygon* expoly, int layer_nr)`
    fn new(expoly: &ExPolygon, layer_nr: i32) -> Self {
        let mut cluster = OverhangCluster {
            layer_overhangs: BTreeMap::new(),
            merged_poly: Vec::new(),
            merged_bbox: BBoxCpp::new(),
            min_layer: 10000000, // 1e7
            max_layer: 0,
            offset: 0.,
            type_: OverhangType::NORMAL,
            small_reason: None,
        };
        cluster.push_back(expoly, layer_nr);
        cluster
    }
    #[allow(dead_code)]
    fn is_cantilever(&self) -> bool {
        self.type_.0 & OverhangType::CANTILEVER.0 != 0
    }
    #[allow(dead_code)]
    fn is_sharp_tail(&self) -> bool {
        self.type_.0 & OverhangType::SHARP_TAIL.0 != 0
    }
    fn is_type(&self, type_mask: OverhangType) -> bool {
        self.type_.0 & type_mask.0 != 0
    }
    // 854-860
    fn set_type(&mut self, type_mask: OverhangType, is: bool) {
        if is {
            self.type_ = OverhangType(self.type_.0 | type_mask.0);
        } else {
            self.type_ = OverhangType(self.type_.0 & !type_mask.0);
        }
    }
    // 864-876
    fn push_back(&mut self, expoly: &ExPolygon, layer_nr: i32) {
        self.layer_overhangs.entry(layer_nr).or_default().push(expoly.clone());
        // `offset_ex(*expoly, offset)`: the overload of one ExPolygon
        let dilate1 = cx::offset_ex_expolygon(expoly, self.offset as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
        if !dilate1.is_empty() {
            self.merged_poly = cx::union_ex_2(&self.merged_poly, &dilate1);
        }
        self.min_layer = self.min_layer.min(layer_nr);
        self.max_layer = self.max_layer.max(layer_nr);
        self.merged_bbox.merge(&BBoxCpp::of_expolygons(&dilate1));
    }
    // 877-879
    fn height(&self) -> i32 {
        self.max_layer - self.min_layer + 1
    }
    // 880-892
    #[allow(dead_code)]
    fn intersects(&mut self, region: &ExPolygon, layer_nr: i32, offset: CoordF) -> bool {
        if layer_nr < 1 {
            return false;
        }
        let Some(overhangs) = self.layer_overhangs.get(&(layer_nr - 1)).cloned() else {
            return false;
        };

        self.offset = offset;
        let dilate1 = cx::offset_ex_expolygon(region, offset as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
        let bbox = BBoxCpp::of_expolygons(&dilate1);
        if !self.merged_bbox.overlap(&bbox) {
            return false;
        }
        cx::overlaps_expolygons(&overhangs, &dilate1)
    }
    // 893-926 it's basically the combination of push_back and intersects, but saves an offset_ex
    fn push_back_if_intersects(&mut self, region: &ExPolygon, layer_nr: i32, offset: CoordF, max_layer_gap: usize) -> bool {
        let mut is_intersect = false;
        self.offset = offset;
        let mut dilate1: ExPolygons = Vec::new();
        let mut bbox = BBoxCpp::new();
        #[allow(clippy::never_loop)]
        loop {
            if layer_nr < 1 {
                break;
            }
            // `auto it = layer_overhangs.lower_bound(layer_nr); if (it == layer_overhangs.begin()) break; -- it;`
            let Some((key, overhangs)) = self.layer_overhangs.range(..layer_nr).next_back() else {
                break;
            };
            if (layer_nr - *key) as usize > max_layer_gap {
                break;
            }

            dilate1 = cx::offset_ex_expolygon(region, offset as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
            if dilate1.is_empty() {
                break;
            }
            bbox = BBoxCpp::of_expolygons(&dilate1);
            if !self.merged_bbox.overlap(&bbox) {
                break;
            }
            is_intersect = cx::overlaps_expolygons(overhangs, &dilate1);
            break;
        }
        if is_intersect {
            self.layer_overhangs.entry(layer_nr).or_default().push(region.clone());
            self.merged_poly = cx::union_ex_2(&self.merged_poly, &dilate1);
            self.min_layer = self.min_layer.min(layer_nr);
            self.max_layer = self.max_layer.max(layer_nr);
            self.merged_bbox.merge(&bbox);
        }
        is_intersect
    }
    // 927-971
    fn check_small_overhang(&mut self, extrusion_width_scaled: CoordF, length_thresh_small_overhang: f64, radius_thresh_small_overhang: f64) {
        self.small_reason = None;
        if self.type_.0 & OverhangType::SHARP_TAIL.0 != 0 || self.type_.0 & OverhangType::CANTILEVER.0 != 0 {
            self.set_type(OverhangType::SMALL, false);
            return;
        }

        let mut all_layers_are_small = true;
        let mut avg_width: f32 = 0.;
        let mut avg_area: f32 = 0.;
        let mut num_layers: i32 = 0;
        for overhangs in self.layer_overhangs.values() {
            let erodel: ExPolygons = cx::offset_ex(overhangs, (-extrusion_width_scaled / 2.) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
            let mut narrow_width: Coord = 0;
            for poly in &erodel {
                let bbox_sz = BBoxCpp::of_expolygon(poly).size();
                narrow_width = narrow_width.max(bbox_sz.x.min(bbox_sz.y));
            }
            if narrow_width as f64 > extrusion_width_scaled {
                all_layers_are_small = false;
            }
            // `avg_width += narrow_width;`: the coord_t converted to float
            avg_width += narrow_width as f32;
            // `avg_area += area(overhangs);`: summed in doubles
            avg_area = (avg_area as f64 + area_of_expolygons(overhangs)) as f32;
            num_layers += 1;
        }
        avg_width /= num_layers as f32;
        avg_area /= num_layers as f32;
        let is_small_overhang = all_layers_are_small
            && (avg_width as f64) < extrusion_width_scaled / 2.
            && (avg_area as f64) < sq(length_thresh_small_overhang)
            && self.height() < 3;
        if is_small_overhang {
            self.small_reason = Some("per-layer");
            self.set_type(OverhangType::SMALL, true);
            return;
        }

        let erodel: ExPolygons = cx::offset_ex(&self.merged_poly, (-radius_thresh_small_overhang) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
        let bbox_sz = BBoxCpp::of_expolygons(&erodel).size();
        if erodel.is_empty()
            || ((bbox_sz.x as f64) < length_thresh_small_overhang
                && (bbox_sz.y as f64) < length_thresh_small_overhang
                && (self.height() as f64) < length_thresh_small_overhang / 2.)
        {
            self.small_reason = Some(if erodel.is_empty() { "radius" } else { "bbox-height" });
            self.set_type(OverhangType::SMALL, true);
        }
    }
    // 973-980
    fn check_polygon_node(&mut self, thresh_big_overhang: f32, m_layer_outlines_below: &ExPolygons) {
        let outlines_below_eroded: ExPolygons = cx::offset_ex(m_layer_outlines_below, (-self.offset) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
        // `SQ(thresh_big_overhang)`: a product of floats
        if area_of_expolygons(&self.merged_poly) > (thresh_big_overhang * thresh_big_overhang) as f64 {
            self.set_type(OverhangType::BIG_FLAT, true);
        } else {
            // `merged_bbox.size().any_comp(thresh_big_overhang, ">")`: Point::any_comp(const coord_t val, ...)
            let val = thresh_big_overhang as Coord;
            let sz = self.merged_bbox.size();
            if (sz.x > val || sz.y > val) && self.min_layer < 20 && !cx::overlaps_expolygons(&self.merged_poly, &outlines_below_eroded) {
                self.set_type(OverhangType::THIN_PLATE, true);
            }
        }
    }
}

// TreeSupport.cpp:994-1010 the lambda `find_and_insert_cluster` of detect_overhangs(): the index of the cluster.
fn find_and_insert_cluster(region_clusters: &mut Vec<OverhangCluster>, region: &ExPolygon, layer_nr: i32, offset: CoordF, max_cluster_layer_gap: usize) -> usize {
    for i in 0..region_clusters.len() {
        if region_clusters[i].push_back_if_intersects(region, layer_nr, offset, max_cluster_layer_gap) {
            return i;
        }
    }
    region_clusters.push(OverhangCluster::new(region, layer_nr));
    region_clusters.len() - 1
}

// ExtrusionEntity.hpp:192 `Polylines ExtrusionEntity::as_polylines() const` of a collection
// (ExtrusionEntityCollection.hpp:132-135): a path its polyline (ExtrusionEntity.hpp:346), a multi path and a loop
// their paths joined into one polyline (`as_polyline()`, ExtrusionEntity.hpp:479, 552); empty ones are left out.
fn extrusion_entities_collect_polylines(entities: &[ExtrusionEntityType], dst: &mut crate::geometry::Polylines) {
    use crate::geometry::Polyline;
    // ExtrusionEntity.cpp `Polyline ExtrusionMultiPath::as_polyline() const` / `ExtrusionLoop::as_polyline() const`:
    // the points of the first path, then of every other path its points but the first.
    fn joined(paths: &[ExtrusionPath]) -> Polyline {
        let mut out = Polyline::from_points(Vec::new());
        if !paths.is_empty() {
            out.points = paths[0].polyline.points.clone();
            for path in &paths[1..] {
                if path.polyline.points.len() > 1 {
                    out.points.extend_from_slice(&path.polyline.points[1..]);
                }
            }
        }
        out
    }
    for entity in entities {
        match entity {
            ExtrusionEntityType::Path(path) => {
                if !path.polyline.points.is_empty() {
                    dst.push(path.polyline.clone());
                }
            }
            ExtrusionEntityType::MultiPath(multipath) => {
                let pl = joined(&multipath.paths);
                if !pl.points.is_empty() {
                    dst.push(pl);
                }
            }
            ExtrusionEntityType::Loop(loop_) => {
                let pl = joined(&loop_.paths);
                if !pl.points.is_empty() {
                    dst.push(pl);
                }
            }
            ExtrusionEntityType::Collection(collection) => extrusion_entities_collect_polylines(&collection.entities, dst),
        }
    }
}

/// PrintObject.cpp:6360-6506 `template<typename PolysType> void PrintObject::remove_bridges_from_contacts(const Layer
/// *lower_layer, const Layer *current_layer, float extrusion_width, PolysType *overhang_regions, float
/// max_bridge_length, bool break_bridge, std::vector<std::pair<ExPolygon, int>> *overhang_regions_with_type)` with
/// `PolysType = ExPolygons`, as detect_overhangs() calls it. The crate has no port of it over the faithful
/// PrintObject, so it is here; `thick_bridges` is `current_layer->object()->config().thick_bridges`.
///
/// FIDELITY-NOTE: the loop over the regions of the layer is ported over the crate's `LayerRegion` and is not
/// compared with the pin (the vectors' layers have no regions). `layerm->bridging_flow()` of the crate needs the
/// configuration the slicing pipeline stamps on a region; a region without it is an `Err`.
#[allow(clippy::too_many_arguments)]
pub fn print_object_remove_bridges_from_contacts(
    lower_layer: &Layer,
    current_layer: &Layer,
    extrusion_width: f32,
    overhang_regions: &mut ExPolygons,
    max_bridge_length: f32,
    break_bridge: bool,
    overhang_regions_with_type: Option<&mut Vec<(ExPolygon, i32)>>,
    thick_bridges: bool,
) -> crate::Result<()> {
    use crate::clipper::JoinType;
    use crate::clipper_utils::ApplySafetyOffset;
    use crate::support::support_common::cu as scu;
    // Extrusion width accounts for the roundings of the extrudates.
    // It is the maximum widh of the extrudate.
    let fw: f32 = extrusion_width;
    // `Lines overhang_perimeters = to_lines(*overhang_regions);` is not read.
    let lower_layer_polygons: Polygons = to_polygons(&lower_layer.lslices);

    let mut all_bridges: Polygons = Vec::new();
    for layerm in current_layer.regions() {
        let mut bridges: Polygons = Vec::new();
        // Surface supporting this layer, expanded by 0.5 * nozzle_diameter, as we consider this kind of overhang to be sufficiently supported.
        let lower_grown_slices: Polygons = scu::offset(
            &lower_layer_polygons,
            //FIXME to mimic the decision in the perimeter generator, we should use half the external perimeter width.
            0.5f32 * fw,
            JoinType::jtSquare,
            0.,
        );
        let mut perimeters: crate::geometry::Polylines = Vec::new();
        extrusion_entities_collect_polylines(&layerm.perimeters.entities, &mut perimeters);
        let mut overhang_perimeters: crate::geometry::Polylines = scu::diff_pl(&perimeters, &lower_grown_slices);
        // only consider straight overhangs
        // only consider overhangs having endpoints inside layer's slices
        // convert bridging polylines into polygons by inflating them with their thickness
        // since we're dealing with bridges, we can't assume width is larger than spacing,
        // so we take the largest value and also apply safety offset to be ensure no gaps
        // are left in between
        let bridge_flow = layerm.bridging_flow(crate::flow::FlowRole::Perimeter, thick_bridges, current_layer.height)?;
        let w: f32 = bridge_flow.scaled_width().max(bridge_flow.scaled_spacing()) as f32;
        for polyline in overhang_perimeters.iter_mut() {
            if polyline.is_straight() {
                // This is a bridge
                polyline.extend_start(fw as f64);
                polyline.extend_end(fw as f64);
                // Is the straight perimeter segment supported at both sides?
                let pts = [polyline.first_point(), polyline.last_point()];
                let mut supported = [false, false];
                let mut i = 0;
                while i < lower_layer.lslices.len() && !(supported[0] && supported[1]) {
                    for j in 0..2 {
                        // `lower_layer->lslices_bboxes[i].contains(pts[j])`
                        let b = &lower_layer.lslices_bboxes[i];
                        let in_bbox = pts[j].x >= b.min.x && pts[j].x <= b.max.x && pts[j].y >= b.min.y && pts[j].y <= b.max.y;
                        if !supported[j] && in_bbox && cx::contains(&lower_layer.lslices[i], &pts[j], true)
                        {
                            supported[j] = true;
                        }
                    }
                    i += 1;
                }
                if supported[0] && supported[1] {
                    let mut lines: crate::geometry::Polylines = Vec::new();
                    // `polyline.length() > max_bridge_length + 10`: the float sum against the double
                    if polyline.length() > (max_bridge_length + 10.) as f64 {
                        if break_bridge {
                            // equally divide the polyline
                            let len: f32 = (polyline.length() / (polyline.length() / max_bridge_length as f64).ceil()) as f32;
                            lines = polyline.equally_spaced_lines(len as f64);
                            for line in lines.iter_mut() {
                                if line.is_valid() {
                                    line.clip_start(fw as f64);
                                }
                                if line.is_valid() {
                                    line.clip_end(fw as f64);
                                }
                            }
                        }
                    } else {
                        lines.push(polyline.clone());
                    }
                    // Offset a polyline into a thick line.
                    bridges.extend(scu::offset_polylines(
                        &lines,
                        0.5f32 * w + 10.,
                        scu::DEFAULT_LINE_JOIN_TYPE,
                        scu::DEFAULT_LINE_MITER_LIMIT,
                        scu::DEFAULT_END_TYPE,
                    ));
                }
            }
        }
        bridges = scu::union_(&bridges);

        // remove the entire bridges and only support the unsupported edges
        //FIXME the brided regions are already collected as layerm->bridged. Use it?
        for surface in &layerm.fill_surfaces.surfaces {
            // Surface.hpp `double bridge_angle = -1.`
            let bridge_angle: f64 = surface.bridge_angle.unwrap_or(-1.);
            if surface.surface_type == crate::surface::SurfaceType::BottomBridge && bridge_angle != -1. {
                let bbox = BBoxCpp::of_expolygon(&surface.expolygon);
                let bbox_size = bbox.size();
                if (bbox_size.x as f32) < max_bridge_length && (bbox_size.y as f32) < max_bridge_length {
                    bridges.extend(to_polygons(std::slice::from_ref(&surface.expolygon)));
                } else if break_bridge {
                    let mut holes: Polygons = Vec::new();
                    let x0: Coord = bbox.min.x;
                    let x1: Coord = bbox.max.x;
                    let y0: Coord = bbox.min.y;
                    let y1: Coord = bbox.max.y;
                    let grid_lw: i32 = (w / 2.) as i32; // grid line width

                    // `Vec2f bridge_direction{ cos(surface.bridge_angle),sin(surface.bridge_angle) }`
                    let bridge_direction: [f32; 2] = [bridge_angle.cos() as f32, bridge_angle.sin() as f32];
                    if bridge_direction[0].abs() > bridge_direction[1].abs() {
                        // cut bridge along x-axis if bridge direction is aligned to x-axis more than to y-axis
                        // Note: surface.bridge_angle may be pi, so we can't compare it to 0 & pi/2.
                        // `int step = bbox_size(0) / ceil(bbox_size(0) / max_bridge_length);`
                        let step: i32 = (bbox_size.x as f32 / (bbox_size.x as f32 / max_bridge_length).ceil()) as i32;
                        // `for (int x = x0 + step; x < x1; x += step)`
                        let mut x: i32 = (x0 + step as i64) as i32;
                        while (x as i64) < x1 {
                            holes.push(Polygon::from_points(vec![
                                Point::new((x - grid_lw) as i64, y0),
                                Point::new((x + grid_lw) as i64, y0),
                                Point::new((x + grid_lw) as i64, y1),
                                Point::new((x - grid_lw) as i64, y1),
                            ]));
                            x += step;
                        }
                    } else {
                        let step: i32 = (bbox_size.y as f32 / (bbox_size.y as f32 / max_bridge_length).ceil()) as i32;
                        let mut y: i32 = (y0 + step as i64) as i32;
                        while (y as i64) < y1 {
                            holes.push(Polygon::from_points(vec![
                                Point::new(x0, (y - grid_lw) as i64),
                                Point::new(x0, (y + grid_lw) as i64),
                                Point::new(x1, (y + grid_lw) as i64),
                                Point::new(x1, (y - grid_lw) as i64),
                            ]));
                            y += step;
                        }
                    }
                    let expoly = scu::diff_ex_ep(std::slice::from_ref(&surface.expolygon), &holes, ApplySafetyOffset::No);
                    bridges.extend(to_polygons(&expoly));
                }
            }
        }
        //FIXME add the gap filled areas. Extrude the gaps with a bridge flow?
        // Remove the unsupported ends of the bridges from the bridged areas.
        //FIXME add supports at regular intervals to support long bridges!
        bridges = scu::diff(
            &bridges,
            // Offset unsupported edges into polygons.
            // SupportMaterial.cpp:59 `#define SUPPORT_MATERIAL_MARGIN 1.2`
            &scu::offset_polylines(&layerm.unsupported_bridge_edges, scale_(1.2) as f32, JoinType::jtSquare, 0., scu::DEFAULT_END_TYPE),
            ApplySafetyOffset::No,
        );
        all_bridges.extend(bridges);
    }
    *overhang_regions = scu::diff_ex_ep(overhang_regions, &all_bridges, ApplySafetyOffset::Yes);

    // Keep an optional typed overhang list in sync by diffing against the same bridge set.
    // Skip items whose bbox doesn't overlap the bridges bbox to avoid unnecessary Clipper calls.
    if let Some(overhang_regions_with_type) = overhang_regions_with_type {
        if !overhang_regions_with_type.is_empty() && !all_bridges.is_empty() {
            // Polygon.cpp `BoundingBox get_extents(const Polygons &polygons)`
            let mut bridges_bbox = BBoxCpp::new();
            if !all_bridges.is_empty() {
                bridges_bbox = BBoxCpp::from_points(&all_bridges[0].points);
                for poly in &all_bridges[1..] {
                    bridges_bbox.merge(&BBoxCpp::from_points(&poly.points));
                }
            }
            let mut kept: Vec<(ExPolygon, i32)> = Vec::with_capacity(overhang_regions_with_type.len());
            let mut any_trimmed = false;
            for overhang_part in overhang_regions_with_type.iter_mut() {
                let part_bbox = BBoxCpp::of_expolygon(&overhang_part.0);
                if !bridges_bbox.overlap(&part_bbox) {
                    kept.push((std::mem::take(&mut overhang_part.0), overhang_part.1));
                    continue;
                }
                let remaining: ExPolygons = scu::diff_ex_ep(std::slice::from_ref(&overhang_part.0), &all_bridges, ApplySafetyOffset::Yes);
                any_trimmed = true;
                for expoly in remaining {
                    kept.push((expoly, overhang_part.1));
                }
            }
            // `kept.emplace_back(std::move(overhang_part.first), ...)` above leaves the expolygon of a part that
            // was not trimmed empty in the list, which stays as it is when no part was trimmed.
            if any_trimmed {
                *overhang_regions_with_type = kept;
            }
        }
    }
    Ok(())
}

impl<'a> TreeSupport<'a> {
    /// TreeSupport.cpp:795-1550 `void TreeSupport::detect_overhangs(bool check_support_necessity)`
    ///
    /// The sharp tails, their heights and the cantilevers of the layers (`Layer::sharp_tails`,
    /// `Layer::sharp_tails_height`, `Layer::cantilevers`, Layer.hpp:172-174) are kept in `layer_sharp_tails`,
    /// `layer_sharp_tails_height` and `layer_cantilevers`, by the index of the layer: the crate's Layer declares
    /// them as `Polygons` and one `f64`.
    ///
    /// FIDELITY-NOTE: the tbb::parallel_for loops run in layer order, which is what the pin does on one thread;
    /// the loop of 1117-1283 turns the sharp tail detection and the small overhang removal off for the layers
    /// after one with more than 100 overhangs (or after 30 seconds), so its result depends on that order.
    /// `enforcers[layer_nr]` of a layer past the trimmed end of the enforcers (1159, 1308: `trim_tail_empty` cut
    /// the empty tail off, the C++ reads the destroyed empty vectors) is empty.
    pub fn detect_overhangs(&mut self, check_support_necessity: bool) -> crate::Result<()> {
        use crate::clipper::JoinType;
        use crate::print_config::config_def::{is_auto, is_tree};
        use crate::support::support_common::cu as scu;
        let tree_support_enable =
            cfg_bool(self.object_config(), "enable_support") && is_tree(support_type_from_i32(cfg_int(self.object_config(), "support_type")));
        if !tree_support_enable && !check_support_necessity {
            // BOOST_LOG_TRIVIAL(info) << "Tree support is disabled.";
            return Ok(());
        }

        let layer_count = self.m_object.layer_count();
        // Clear and create Tree Support Layers
        // m_object->clear_support_layers();  (PrintObject.cpp:1506-1518)
        if self.m_object.m_shared_object.is_none() {
            self.m_object.m_support_layers.clear();
            self.layer_sharp_tails.clear();
            self.layer_sharp_tails_height.clear();
            self.layer_cantilevers.clear();
        }
        self.layer_sharp_tails.resize(layer_count, Vec::new());
        self.layer_sharp_tails_height.resize(layer_count, Vec::new());
        self.layer_cantilevers.resize(layer_count, Vec::new());
        // m_object->clear_tree_support_preview_cache(): the cache of the object is `m_ts_data`, which the tree
        // support keeps (the C++ holds a shared_ptr to it).

        let config = self.object_config();
        let stype: SupportType = support_type_from_i32(self.support_type);
        // `const coordf_t radius_sample_resolution = g_config_tree_support_collision_resolution;`: the float
        let radius_sample_resolution: CoordF = G_CONFIG_TREE_SUPPORT_COLLISION_RESOLUTION as f64;
        let full_print_config = self.m_print.full_print_config();
        let extrusion_width: CoordF = crate::flow::Flow::extrusion_width_for_nozzle(
            &cfg_float_option(config, "line_width"),
            crate::print_config::config_def::primary_nozzle_diameter(full_print_config) as f32,
            support_nozzle_diameter(self.m_print, cfg_int(config, "support_filament") as u32),
            crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(full_print_config),
        );
        let extrusion_width_scaled: CoordF = scale_(extrusion_width);
        let max_bridge_length: CoordF = scale_(cfg_float(config, "max_bridge_length"));
        let _bridge_no_support = max_bridge_length > 0.;
        let support_critical_regions_only = cfg_bool(config, "support_critical_regions_only");
        let mut config_remove_small_overhangs = cfg_bool(config, "support_remove_small_overhang");
        // libslic3r.h:91 `static constexpr bool g_config_support_sharp_tails = true;`
        const G_CONFIG_SUPPORT_SHARP_TAILS: bool = true;
        let mut config_detect_sharp_tails = G_CONFIG_SUPPORT_SHARP_TAILS;
        let enforce_support_layers: i32 = cfg_int(config, "enforce_support_layers");
        let _area_thresh_well_supported: f64 = sq(scale_(6.));
        let length_thresh_well_supported: f64 = scale_(6.);
        let length_thresh_small_overhang: f64 = scale_(2.);
        let radius_thresh_small_overhang: f64 = 2.5 * extrusion_width_scaled;
        // `static const double sharp_tail_max_support_height = 16.f;`
        const SHARP_TAIL_MAX_SUPPORT_HEIGHT: f64 = 16.;
        // a region is considered well supported if the number of layers below it exceeds this threshold
        let _thresh_layers_below: i32 = (10. / cfg_float(config, "layer_height")) as i32;
        // +1 makes the threshold inclusive
        let support_threshold_angle: i32 = cfg_int(config, "support_threshold_angle");
        let mut thresh_angle: f64 = if support_threshold_angle as f64 > EPSILON { (support_threshold_angle + 1) as f64 } else { 30. };
        thresh_angle = thresh_angle.min(89.); // should be smaller than 90
        // Geometry.hpp:295 `deg2rad(angle)`: `T(PI) * angle / T(180.0)`
        let threshold_rad: f64 = M_PI * thresh_angle / 180.0;
        // Dilate painted enforcers by the effective tip, not the raw setting: tip_radius is already clamped
        // to [support extrusion width, branch diameter], so a large tree_support_tip_diameter cannot blow this
        // fudge offset up to 10mm. Enforcers are unioned back in after blockers are subtracted below, so an
        // unclamped offset here would let a thick tip swallow nearby blockers.
        // `scaled<double>(tip_radius * 2)`
        let enforcer_overhang_offset: f64 = (self.tip_radius * 2.) / SCALING_FACTOR;
        let thick_bridges = cfg_bool(config, "thick_bridges");
        let thresh_big_overhang: f64 = self.m_support_params.thresh_big_overhang;

        let mut overhang_clusters: Vec<OverhangCluster> = Vec::new();

        // A coarse region only contributes overhangs at its band tops. Treat the swallowed fine-grid layers as
        // adjacent here, matching how consecutive coarse layers are clustered in a single-diameter print.
        let mut max_cluster_layer_gap: usize = 1;
        for layer in self.m_object.layers() {
            for region in layer.regions() {
                for patch in &region.adaptive_merge_patches {
                    if patch.group_top {
                        max_cluster_layer_gap = max_cluster_layer_gap.max(patch.top_idx - patch.bottom_idx + 1);
                    }
                }
            }
        }

        if !is_tree(stype) {
            return Ok(());
        }

        self.max_cantilever_dist = 0.;
        self.m_highest_overhang_layer = 0;

        // calc the extrudable expolygons of each layer
        for layer_nr in 0..layer_count {
            if (self.canceled)() {
                break;
            }
            let layer = &mut self.m_object.m_layers[layer_nr];
            // Filter out areas whose diameter that is smaller than extrusion_width, but we don't want to lose any details.
            layer.lslices_extrudable = cx::intersection_ex(
                &layer.lslices,
                &cx::offset2_ex(&layer.lslices, (-extrusion_width_scaled / 2.) as f32, extrusion_width_scaled as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
            );
            layer.loverhangs.clear();
            layer.loverhangs_with_type.clear();
            self.layer_sharp_tails[layer_nr].clear();
            self.layer_sharp_tails_height[layer_nr].clear();
            self.layer_cantilevers[layer_nr].clear();
        }

        // main part of overhang detection can be parallel
        let mut overhangs_all_layers: Vec<ExPolygons> = vec![Vec::new(); layer_count];

        // `m_object->slice_support_enforcers()` / `slice_support_blockers()` (Print.hpp:544-545)
        let mut enforcers: Vec<Polygons> = self
            .m_object
            .slice_support_volumes(self.m_print, crate::model::ModelVolumeType::SupportEnforcer)
            .map_err(|e| crate::Error::Slicing(e.what().to_string()))?;
        let mut blockers: Vec<Polygons> = self
            .m_object
            .slice_support_volumes(self.m_print, crate::model::ModelVolumeType::SupportBlocker)
            .map_err(|e| crate::Error::Slicing(e.what().to_string()))?;
        self.m_vertical_enforcer_points.clear();
        if let Some(projector) = self.custom_facets_projector.take() {
            use crate::triangle_selector::EnforcerBlockerType;
            projector.project_and_append_custom_facets(self.m_object, self.m_print, false, EnforcerBlockerType::BLOCKER, &mut blockers, None);
            projector.project_and_append_custom_facets(
                self.m_object,
                self.m_print,
                false,
                EnforcerBlockerType::ENFORCER,
                &mut enforcers,
                Some(&mut self.m_vertical_enforcer_points),
            );
            self.custom_facets_projector = Some(projector);
        }
        // 1042-1049 the lambda `trim_tail_empty`
        let trim_tail_empty = |vec: &mut Vec<Polygons>| match vec.iter().rposition(|e| !e.is_empty()) {
            Some(pos) => vec.truncate(pos + 1),
            None => vec.clear(),
        };
        trim_tail_empty(&mut enforcers);
        trim_tail_empty(&mut blockers);
        // `enforcers[layer_nr]` past the end, see the note above.
        let no_polygons: Polygons = Vec::new();

        // Adaptive multi-nozzle layers are sliced on the finest Z grid, but a coarse nozzle extrudes one
        // merged slab at the top of each band. Detecting overhangs from every fine-grid slice turns that slab's
        // overhang into several narrow strips which organic-support avoidance may remove completely. Recreate
        // the same overhang the coarse nozzle would see on its own: compare the retained slab footprint with
        // the model below the band and place the result at the slab's physical bottom.
        //
        // Every layer reads only its own patches and the already finished lslices_extrudable, so the layers are
        // processed in parallel. A slab's overhang belongs to the layer it starts on, not to the one that found
        // it; those are collected per layer and handed over afterwards, in layer order, before the union.
        let mut adaptive_region_slices: Vec<ExPolygons> = vec![Vec::new(); layer_count];
        let mut adaptive_overhangs: Vec<ExPolygons> = vec![Vec::new(); layer_count];
        let mut found_overhangs: Vec<Vec<(usize, ExPolygons)>> = vec![Vec::new(); layer_count];
        for layer_nr in 0..layer_count {
            if (self.canceled)() {
                break;
            }
            let layer = self.m_object.get_layer(layer_nr as i32);
            for layerm in layer.regions() {
                if layerm.adaptive_merge_patches.is_empty() {
                    continue;
                }

                for patch in &layerm.adaptive_merge_patches {
                    // The region is physically represented by this retained coarse-slab footprint throughout
                    // the band. LayerRegion::slices may already be empty on merged-out fine-grid layers, which
                    // would let the normal detector process those layers again and fragment one coarse overhang.
                    if !patch.area().is_empty() {
                        adaptive_region_slices[layer_nr].extend(patch.area().iter().cloned());
                    }
                    if !patch.group_top || patch.bottom_idx == 0 || patch.area().is_empty() {
                        continue;
                    }

                    let below_band = self.m_object.get_layer((patch.bottom_idx - 1) as i32);
                    let support_offset: Coord = scale_(patch.merged_height / threshold_rad.tan()) as Coord;
                    // Keep the same minimum-width filter as the normal detector. Comparing the raw patch against
                    // a filtered lower layer turns a narrowing tip into a fully floating overhang once the lower
                    // contour becomes thinner than the support extrusion width.
                    let patch_extrudable: ExPolygons = cx::intersection_ex(
                        patch.area(),
                        &cx::offset2_ex(patch.area(), (-extrusion_width_scaled / 2.) as f32, extrusion_width_scaled as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                    );
                    let overhangs: ExPolygons =
                        cx::diff_ex(&patch_extrudable, &cx::offset_ex(&below_band.lslices_extrudable, support_offset as f32, JoinType::jtSquare, 0.));
                    if !overhangs.is_empty() {
                        found_overhangs[layer_nr].push((patch.bottom_idx, overhangs));
                    }
                }
            }
            if !adaptive_region_slices[layer_nr].is_empty() {
                adaptive_region_slices[layer_nr] = cx::union_ex(&adaptive_region_slices[layer_nr]);
            }
        }
        for found in found_overhangs.iter_mut() {
            for overhang in found.drain(..) {
                adaptive_overhangs[overhang.0].extend(overhang.1);
            }
        }
        for layer_nr in 0..layer_count {
            if !adaptive_overhangs[layer_nr].is_empty() {
                adaptive_overhangs[layer_nr] = cx::union_ex(&adaptive_overhangs[layer_nr]);
            }
        }

        // The 30 s budget checked below covers the per-layer detection only, not the enforcer projection and the
        // coarse slab overhangs prepared above.
        let t0 = std::time::Instant::now();
        for layer_nr in 0..layer_count {
            if (self.canceled)() {
                break;
            }
            // FIXME the param enforce_support_layers is not set yet
            // `layer_nr >= enforce_support_layers` / `layer_nr < enforce_support_layers`: the int converted to size_t
            if !(is_auto(stype)
                || (enforce_support_layers > 0 && layer_nr >= enforce_support_layers as usize)
                || (layer_nr < enforcers.len() && !enforcers[layer_nr].is_empty()))
            {
                continue;
            }

            let layer = self.m_object.get_layer(layer_nr as i32);

            let Some(lower_layer_nr) = layer.lower_layer_id else {
                let mut curr_polys: ExPolygons = layer.lslices_extrudable.clone();
                if layer_nr < blockers.len() && !blockers[layer_nr].is_empty() {
                    curr_polys = cx::diff_ex(
                        &curr_polys,
                        &scu::offset_ex(&scu::union_(&blockers[layer_nr]), scale_(radius_sample_resolution) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                    );
                }
                let height = layer.height;
                for slice in &curr_polys {
                    let bbox_size = BBoxCpp::of_expolygon(slice).size();
                    if !(bbox_size.x as f64 > length_thresh_well_supported || bbox_size.y as f64 > length_thresh_well_supported) && G_CONFIG_SUPPORT_SHARP_TAILS {
                        self.layer_sharp_tails[layer_nr].push(slice.clone());
                        self.layer_sharp_tails_height[layer_nr].push(height as f32);
                    }
                }
                continue;
            };

            let lower_layer = self.m_object.get_layer(lower_layer_nr as i32);
            // `layer_nr < enforce_support_layers ? -0.15 * extrusion_width : (float)lower_layer->height / tan(threshold_rad)`
            let lower_layer_offset: CoordF = if (layer_nr as u64) < (enforce_support_layers as i64 as u64) {
                -0.15 * extrusion_width
            } else {
                (lower_layer.height as f32) as f64 / threshold_rad.tan()
            };
            //lower_layer_offset = std::min(lower_layer_offset, extrusion_width);
            let support_offset_scaled: CoordF = scale_(lower_layer_offset);
            let curr_polys: ExPolygons = if adaptive_region_slices[layer_nr].is_empty() {
                layer.lslices_extrudable.clone()
            } else {
                cx::diff_ex(&layer.lslices_extrudable, &adaptive_region_slices[layer_nr])
            };
            let lower_polys: &ExPolygons = &lower_layer.lslices_extrudable;
            let mut lower_layer_offseted: ExPolygons = cx::offset_ex(lower_polys, support_offset_scaled as f32, JoinType::jtSquare, 0.);

            // add enforcer
            let mut enforced_overhangs: ExPolygons = Vec::new();
            let mut blocker: ExPolygons = Vec::new();
            if !enforcers.is_empty() {
                enforced_overhangs = cx::intersection_ex_polygons(&cx::diff_ex(&layer.lslices_extrudable, &lower_layer.lslices_extrudable), enforcers.get(layer_nr).unwrap_or(&no_polygons));
            }
            if is_auto(stype) {
                // normal overhang
                overhangs_all_layers[layer_nr] = cx::diff_ex(&curr_polys, &lower_layer_offseted);
                if !adaptive_overhangs[layer_nr].is_empty() {
                    let mut combined: Polygons = to_polygons(&overhangs_all_layers[layer_nr]);
                    combined.extend(to_polygons(&adaptive_overhangs[layer_nr]));
                    overhangs_all_layers[layer_nr] = cx::union_ex_polygons(&combined);
                }
                // if is auto, add blocker first
                if layer_nr < blockers.len() && !blockers[layer_nr].is_empty() {
                    // Arthur: union_ is a must because after mirroring, the blocker polygons are in left-hand coordinates, ie clockwise,
                    // which are not valid polygons, and will be removed by offset_ex. union_ can make these polygons right.
                    blocker = scu::offset_ex(&scu::union_(&blockers[layer_nr]), scale_(radius_sample_resolution) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
                    if !blocker.is_empty() {
                        overhangs_all_layers[layer_nr] = cx::diff_ex(&overhangs_all_layers[layer_nr], &blocker);
                    }
                }
                if !enforced_overhangs.is_empty() {
                    enforced_overhangs = cx::diff_ex(
                        &cx::offset_ex(&enforced_overhangs, enforcer_overhang_offset as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                        &lower_layer.lslices_extrudable,
                    );
                    overhangs_all_layers[layer_nr] = cx::union_ex_2(&overhangs_all_layers[layer_nr], &enforced_overhangs);
                }
            } else if layer_nr < enforcers.len() {
                if !enforced_overhangs.is_empty() {
                    // FIXME this is a hack to make enforcers work on steep overhangs. See STUDIO-7538.
                    enforced_overhangs = cx::diff_ex(
                        &cx::offset_ex(&enforced_overhangs, enforcer_overhang_offset as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                        &lower_layer.lslices_extrudable,
                    );
                    // `overhangs_all_layers[layer_nr] = std::move(enforced_overhangs);`
                    overhangs_all_layers[layer_nr] = std::mem::take(&mut enforced_overhangs);
                }
            }

            let duration: f64 = t0.elapsed().as_secs_f64();
            if duration > 30. || overhangs_all_layers[layer_nr].len() > 100 {
                // BOOST_LOG_TRIVIAL(info) << "detect_overhangs takes more than 30 secs, skip cantilever and sharp tails detection: layer_nr=" << layer_nr << " duration=" << duration;
                config_detect_sharp_tails = false;
                config_remove_small_overhangs = false;
                continue;
            }
            if is_auto(stype) && config_detect_sharp_tails {
                let mut curr: ExPolygons = curr_polys.clone();
                if !blocker.is_empty() {
                    curr = cx::diff_ex(&curr, &blocker);
                }
                if !enforced_overhangs.is_empty() {
                    curr = cx::union_ex_2(&curr, &enforced_overhangs);
                }
                // BBS detect sharp tail
                for expoly in &curr {
                    let mut is_sharp_tail = false;
                    // 1. nothing below
                    // this is a sharp tail region if it's floating and non-ignorable
                    // The (area && bbox) gate excludes "large looped structures" but also catches
                    // thin-but-wide rings (small area, large bbox), which would otherwise be dropped
                    // by check_small_overhang and end up unsupported. Rescue them here: if eroding by
                    // radius_thresh_small_overhang (the same radius small-overhang detection uses)
                    // makes the polygon vanish, treat it as a sharp tail regardless of bbox.
                    if !cx::overlaps_expolygons(
                        &cx::offset_ex_expolygon(expoly, (0.1 * extrusion_width_scaled) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                        lower_polys,
                    ) && expoly.area() < sq(thresh_big_overhang)
                        && (BBoxCpp::of_expolygon(expoly).area() < sq(thresh_big_overhang)
                            || cx::offset_ex_expolygon(expoly, (-radius_thresh_small_overhang) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT).is_empty())
                    {
                        is_sharp_tail =
                            !cx::offset_ex_expolygon(expoly, (-0.1 * extrusion_width_scaled) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT).is_empty();
                    }
                    if is_sharp_tail {
                        if let Some(lower_lower_nr) = lower_layer.lower_layer_id {
                            if cx::overlaps_expolygons(
                                &cx::offset_ex_expolygon(expoly, (0.1 * extrusion_width_scaled) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                                &self.m_object.get_layer(lower_lower_nr as i32).lslices_extrudable,
                            ) {
                                is_sharp_tail = false;
                            }
                        }
                    }
                    if is_sharp_tail {
                        self.layer_sharp_tails[layer_nr].push(expoly.clone());
                        self.layer_sharp_tails_height[layer_nr].push(0.);

                        self.has_sharp_tails = true;
                    }
                }
            }

            // check cantilever
            let lower_layer_support_thresh: ExPolygons = cx::offset_ex(lower_polys, extrusion_width_scaled as f32, JoinType::jtSquare, 0.);
            // lower_layer_offset may be very small, so we need to do max and then add 0.1
            lower_layer_offseted = cx::offset_ex(
                &lower_layer_offseted,
                scale_((extrusion_width - lower_layer_offset).max(0.) + 0.1) as f32,
                cx::DEFAULT_JOIN_TYPE,
                cx::DEFAULT_MITER_LIMIT,
            );
            for poly in &overhangs_all_layers[layer_nr] {
                // check if there is some contour that is totally floating
                let mut is_cantilever = false;
                let mut dist_max: f64 = 0.;
                for i in 0..poly.num_contours() {
                    let contour = contour_or_hole(poly, i);
                    let mut is_floating = true;
                    let mut tmp_dist_max: f64 = 0.;
                    for pt in &contour.points {
                        if is_inside_ex(&lower_layer_support_thresh, pt) {
                            is_floating = false;
                            break;
                        } else {
                            tmp_dist_max = tmp_dist_max.max(point_dist(projection_onto(&lower_layer_support_thresh, pt), *pt));
                        }
                    }
                    if is_floating {
                        is_cantilever = true;
                        dist_max = tmp_dist_max;
                        break;
                    }
                }
                if !is_cantilever {
                    dist_max = 0.;
                    let cluster_boundary_ex = cx::intersection_ex(std::slice::from_ref(poly), &lower_layer_offseted);
                    let cluster_boundary: Polygons = to_polygons(&cluster_boundary_ex);
                    if cluster_boundary.is_empty() {
                        continue;
                    }

                    for pt in &poly.contour.points {
                        let mut dist_pt = f64::MAX;
                        for ply in &cluster_boundary {
                            let d = multipoint_distance_to(&ply.points, pt);
                            dist_pt = dist_pt.min(d);
                        }
                        dist_max = dist_max.max(dist_pt);
                    }
                    is_cantilever = dist_max > scale_(3.);
                }
                // is cantilever if the farmost point is larger than 3mm away from base or some contour is totally floating
                if is_cantilever {
                    self.max_cantilever_dist = self.max_cantilever_dist.max(dist_max);
                    self.layer_cantilevers[layer_nr].push(poly.clone());
                    self.has_cantilever = true;
                }
            }
        } // end tbb::parallel_for

        // BOOST_LOG_TRIVIAL(info) << "max_cantilever_dist=" << max_cantilever_dist;
        if check_support_necessity {
            return Ok(());
        }

        // check if the sharp tails should be extended higher
        if is_auto(stype) && config_detect_sharp_tails {
            for layer_nr in 0..layer_count {
                if (self.canceled)() {
                    break;
                }

                let layer = self.m_object.get_layer(layer_nr as i32);
                let Some(lower_layer_nr) = layer.lower_layer_id else {
                    continue;
                };
                let lower_layer = self.m_object.get_layer(lower_layer_nr as i32);

                // BBS detect sharp tail
                let lower_layer_sharptails: ExPolygons = self.layer_sharp_tails[lower_layer_nr].clone();
                let lower_layer_sharptails_height: Vec<f32> = self.layer_sharp_tails_height[lower_layer_nr].clone();

                let mut curr_polys: ExPolygons = layer.lslices_extrudable.clone();
                if !blockers.is_empty() && layer_nr < blockers.len() && !blockers[layer_nr].is_empty() {
                    curr_polys = cx::diff_ex(
                        &curr_polys,
                        &scu::offset_ex(&scu::union_(&blockers[layer_nr]), scale_(radius_sample_resolution) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                    );
                }
                if !enforcers.is_empty() && layer_nr < enforcers.len() && !enforcers[layer_nr].is_empty() {
                    curr_polys = cx::union_ex_2(
                        &curr_polys,
                        &cx::intersection_ex_polygons(&cx::diff_ex(&layer.lslices_extrudable, &lower_layer.lslices_extrudable), &enforcers[layer_nr]),
                    );
                }
                let layer_height = layer.height;
                for expoly in &curr_polys {
                    let mut is_sharp_tail = false;
                    let mut accum_height: f32 = layer_height as f32;
                    #[allow(clippy::never_loop)]
                    loop {
                        // 2. something below
                        // check whether this is above a sharp tail region.

                        // 2.1 If no sharp tail below, this is considered as common region.
                        let supported_by_lower: ExPolygons = cx::intersection_ex(std::slice::from_ref(expoly), &lower_layer_sharptails);
                        if supported_by_lower.is_empty() {
                            is_sharp_tail = false;
                            break;
                        }

                        // 2.2 If sharp tail below, check whether it support this region enough.
                        let bbox = BBoxCpp::of_expolygons(&supported_by_lower);
                        if bbox.size().x as f64 > length_thresh_well_supported && bbox.size().y as f64 > length_thresh_well_supported {
                            is_sharp_tail = false;
                            break;
                        }

                        // 2.3 check whether sharp tail exceed the max height
                        for i in 0..lower_layer_sharptails.len() {
                            if cx::overlaps(&lower_layer_sharptails[i], expoly) {
                                accum_height += lower_layer_sharptails_height[i];
                                break;
                            }
                        }
                        if accum_height as f64 > SHARP_TAIL_MAX_SUPPORT_HEIGHT {
                            is_sharp_tail = false;
                            break;
                        }

                        // 2.4 if the area grows fast than threshold, it get connected to other part or
                        // it has a sharp slop and will be auto supported.
                        let new_overhang_expolys: ExPolygons = cx::diff_ex(std::slice::from_ref(expoly), &lower_layer_sharptails);
                        let (sa, sb) = (BBoxCpp::of_expolygons(&new_overhang_expolys).size(), BBoxCpp::of_expolygons(&lower_layer_sharptails).size());
                        let d = Point::new(sa.x - sb.x, sa.y - sb.y);
                        // `.both_comp(Point(scale_(5), scale_(5)), ">")`
                        let five = point_from_f64(scale_(5.), scale_(5.));
                        if (d.x > five.x && d.y > five.y)
                            || !cx::offset_ex(&new_overhang_expolys, (-5.0 * extrusion_width_scaled) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT).is_empty()
                        {
                            is_sharp_tail = false;
                            break;
                        }

                        // 2.5 mark the expoly as sharptail
                        is_sharp_tail = true;
                        break;
                    }

                    if is_sharp_tail {
                        self.layer_sharp_tails[layer_nr].push(expoly.clone());
                        self.layer_sharp_tails_height[layer_nr].push(accum_height);
                    }
                }
            }
        }

        // group overhang clusters
        for layer_nr in 0..layer_count {
            if (self.canceled)() {
                break;
            }
            for overhang in &overhangs_all_layers[layer_nr] {
                let cluster = find_and_insert_cluster(&mut overhang_clusters, overhang, layer_nr as i32, extrusion_width_scaled, max_cluster_layer_gap);
                if cx::overlaps_expolygons(std::slice::from_ref(overhang), &self.layer_cantilevers[layer_nr]) {
                    overhang_clusters[cluster].set_type(OverhangType::CANTILEVER, true);
                }
            }
        }

        for cluster in overhang_clusters.iter_mut() {
            let mut enforce_add = false;
            // remove small overhangs
            if is_auto(stype) && config_remove_small_overhangs {
                // 3. check whether the small overhang is sharp tail
                cluster.set_type(OverhangType::SHARP_TAIL, false);
                // `cluster.layer_overhangs[layer_id]`: std::map::operator[] makes the entry of a layer that has none
                for layer_id in cluster.min_layer as usize..=cluster.max_layer as usize {
                    if cx::overlaps_expolygons(&self.layer_sharp_tails[layer_id], cluster.layer_overhangs.entry(layer_id as i32).or_default()) {
                        cluster.set_type(OverhangType::SHARP_TAIL, true);
                        break;
                    }
                }
                if !enforcers.is_empty() && (cluster.min_layer as usize) < enforcers.len() {
                    for layer_id in cluster.min_layer as usize..=cluster.max_layer as usize {
                        if layer_id >= enforcers.len() {
                            break;
                        }
                        if enforcers[layer_id].is_empty() {
                            continue;
                        }
                        if cx::overlaps_expolygons(&to_expolygons_of_polygons(&enforcers[layer_id]), cluster.layer_overhangs.entry(layer_id as i32).or_default()) {
                            enforce_add = true;
                            break;
                        }
                    }
                }

                cluster.check_small_overhang(extrusion_width_scaled, length_thresh_small_overhang, radius_thresh_small_overhang);
            }

            if !cluster.is_type(OverhangType::SMALL) || enforce_add {
                cluster.check_polygon_node(thresh_big_overhang as f32, &self.m_ts_data.m_layer_outlines_below[(cluster.min_layer - 1) as usize]);
                let cluster_type = cluster.type_.0 as i32;
                let is_small = cluster.is_type(OverhangType::SMALL);
                for (layer_nr, overhangs) in cluster.layer_overhangs.iter() {
                    let layer_nr = *layer_nr as usize;
                    let mut overhangs: ExPolygons = overhangs.clone();
                    if is_small {
                        if layer_nr >= enforcers.len()
                            || enforcers[layer_nr].is_empty()
                            || !cx::overlaps_expolygons(&to_expolygons_of_polygons(&enforcers[layer_nr]), &overhangs)
                        {
                            continue;
                        }
                        overhangs = cx::intersection_ex_polygons(&overhangs, &enforcers[layer_nr]);
                    }
                    for overhang in &overhangs {
                        add_overhang(&mut self.m_object.m_layers[layer_nr], overhang, cluster_type);
                    }
                }
            }
        }

        let mut _layers_with_overhangs = 0;
        let _layers_with_enforcers = 0;
        for layer_nr in 0..layer_count {
            if (self.canceled)() {
                break;
            }

            let lower_layer_nr: Option<usize> = self.m_object.m_layers[layer_nr].lower_layer_id;

            if support_critical_regions_only && is_auto(stype) {
                if lower_layer_nr.is_none() || enforcers.is_empty() || layer_nr >= enforcers.len() || enforcers[layer_nr].is_empty() {
                    let layer = &mut self.m_object.m_layers[layer_nr];
                    layer.loverhangs.clear();
                    layer.loverhangs_with_type.clear();
                    for cantilever in &self.layer_cantilevers[layer_nr] {
                        add_overhang(layer, cantilever, OverhangType::CANTILEVER.0 as i32);
                    }
                } else {
                    let enforced_overhangs: ExPolygons =
                        cx::offset_ex(&to_expolygons_of_polygons(&enforcers[layer_nr]), enforcer_overhang_offset as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
                    let mut loverhangs_new: ExPolygons = Vec::new();
                    let mut loverhangs_with_type_new: Vec<(ExPolygon, i32)> = Vec::new();
                    let mut bigflat: ExPolygons = Vec::new();
                    let layer = &mut self.m_object.m_layers[layer_nr];
                    for overhang_part in &layer.loverhangs_with_type {
                        let overhang = &overhang_part.0;
                        let type_ = overhang_part.1;
                        let mut overhangs: ExPolygons = Vec::new();
                        if type_ & OverhangType::CANTILEVER.0 as i32 != 0 || type_ & OverhangType::SHARP_TAIL.0 as i32 != 0 {
                            overhangs = vec![overhang.clone()];
                        } else if enforced_overhangs.iter().any(|e| cx::overlaps(e, overhang)) {
                            // `overlaps(const ExPolygons &expolys, const ExPolygon &expoly)` (ExPolygon.cpp:197-204)
                            // `intersection_ex(const ExPolygons &subject, const ExPolygon &clip)`
                            overhangs = cx::intersection_ex(&enforced_overhangs, std::slice::from_ref(overhang));
                        }
                        if !overhangs.is_empty() {
                            if type_ & OverhangType::BIG_FLAT.0 as i32 != 0 {
                                append(&mut bigflat, overhangs);
                            } else {
                                for expoly in &overhangs {
                                    loverhangs_new.push(expoly.clone());
                                    loverhangs_with_type_new.push((expoly.clone(), type_));
                                }
                            }
                        }
                    }
                    for expoly in cx::union_ex(&bigflat) {
                        loverhangs_new.push(expoly.clone());
                        loverhangs_with_type_new.push((expoly, OverhangType::BIG_FLAT.0 as i32));
                    }
                    layer.loverhangs = loverhangs_new;
                    layer.loverhangs_with_type = loverhangs_with_type_new;
                }
            }

            // add support for every 1mm height for sharp tails
            if let Some(lower_layer_nr) = lower_layer_nr {
                // libslic3r.h:64 `SCALED_RESOLUTION (RESOLUTION / SCALING_FACTOR)`, RESOLUTION = 0.0125
                let lower_layer_expanded: ExPolygons = cx::offset_ex(
                    &self.m_object.m_layers[lower_layer_nr].lslices_extrudable,
                    (0.0125 / SCALING_FACTOR) as f32,
                    cx::DEFAULT_JOIN_TYPE,
                    cx::DEFAULT_MITER_LIMIT,
                );
                for i in 0..self.layer_sharp_tails_height[layer_nr].len() {
                    let areas: ExPolygons = cx::diff_clipped(std::slice::from_ref(&self.layer_sharp_tails[layer_nr][i]), &lower_layer_expanded);
                    let accum_height: f32 = self.layer_sharp_tails_height[layer_nr][i];
                    // `int(accum_height * 10) % 5 == 0`
                    if !areas.is_empty() && ((accum_height * 10.) as i32) % 5 == 0 {
                        self.has_sharp_tails = true;
                        let type_ = if (accum_height as f64) < EPSILON {
                            (OverhangType::SHARP_TAIL.0 | OverhangType::SHARP_TAIL_LOWESST.0) as i32
                        } else {
                            OverhangType::SHARP_TAIL.0 as i32
                        };
                        for area in &areas {
                            add_overhang(&mut self.m_object.m_layers[layer_nr], area, type_);
                        }
                    }
                }
            }

            if let Some(lower_layer_nr) = lower_layer_nr {
                if max_bridge_length > 0. && !self.m_object.m_layers[layer_nr].loverhangs.is_empty() {
                    // do not break bridge as the interface will be poor, see #4318
                    let break_bridge = false;
                    let mut loverhangs = std::mem::take(&mut self.m_object.m_layers[layer_nr].loverhangs);
                    let mut loverhangs_with_type = std::mem::take(&mut self.m_object.m_layers[layer_nr].loverhangs_with_type);
                    let res = print_object_remove_bridges_from_contacts(
                        &self.m_object.m_layers[lower_layer_nr],
                        &self.m_object.m_layers[layer_nr],
                        extrusion_width_scaled as f32,
                        &mut loverhangs,
                        max_bridge_length as f32,
                        break_bridge,
                        Some(&mut loverhangs_with_type),
                        thick_bridges,
                    );
                    self.m_object.m_layers[layer_nr].loverhangs = loverhangs;
                    self.m_object.m_layers[layer_nr].loverhangs_with_type = loverhangs_with_type;
                    res?;
                }
            }

            let layer = &self.m_object.m_layers[layer_nr];
            let _n_detected = layer.loverhangs.len();

            if !layer.loverhangs_with_type.is_empty() {
                _layers_with_overhangs += 1;
                self.m_highest_overhang_layer = self.m_highest_overhang_layer.max(layer_nr);
            }
            if !self.layer_cantilevers[layer_nr].is_empty() {
                self.has_cantilever = true;
            }
        }

        // BOOST_LOG_TRIVIAL(info) << "Tree support overhang detection done. " << layers_with_overhangs << " layers with overhangs. nEnforced=" << layers_with_enforcers;
        Ok(())
    }
}

// ============================================================================
// TreeSupport::get_trim_support_regions (TreeSupport.cpp:3829-3903) and
// TreeSupport::prune_floating_supports (TreeSupport.cpp:4903-5099): members the pin defines and does not call.
// ============================================================================

impl<'a> TreeSupport<'a> {
    /// TreeSupport.cpp:3829-3903 `Polygons TreeSupport::get_trim_support_regions(const PrintObject &object, SupportLayer
    /// *support_layer_ptr, const coordf_t gap_extra_above, const coordf_t gap_extra_below, const coordf_t gap_xy)`
    ///
    /// `object_layer.sharp_tails` (3874) is `layer_sharp_tails` of the layer's index, see detect_overhangs():
    /// `object` is the object of the tree support.
    pub fn get_trim_support_regions(
        &self,
        object: &PrintObject,
        support_layer: &crate::print::faithful::SupportLayer,
        gap_extra_above: CoordF,
        gap_extra_below: CoordF,
        gap_xy: CoordF,
    ) -> Polygons {
        use crate::clipper::JoinType;
        use crate::support::support_common::cu as scu;
        // `static const double sharp_tail_xy_gap = 0.2f;` / `no_overlap_xy_gap = 0.2f`: the doubles of the float
        const SHARP_TAIL_XY_GAP: f64 = 0.2f32 as f64;
        const NO_OVERLAP_XY_GAP: f64 = 0.2f32 as f64;
        let gap_xy_scaled: f64 = scale_(gap_xy);
        let m_print_config = self.m_print_config;

        // 3844-3856 the lambda `is_layers_overlap`
        let is_layers_overlap = |support_layer: &crate::print::faithful::SupportLayer, object_layer: &Layer, bridging_height: CoordF| -> bool {
            if (support_layer.layer.print_z - object_layer.print_z).abs() < EPSILON {
                return true;
            }

            let object_lh: CoordF = if bridging_height > EPSILON { bridging_height } else { object_layer.height };
            if support_layer.layer.print_z < object_layer.print_z && support_layer.layer.print_z > object_layer.print_z - object_lh {
                return true;
            }

            if support_layer.layer.print_z > object_layer.print_z && support_layer.layer.bottom_z() < object_layer.print_z - EPSILON {
                return true;
            }

            false
        };

        // Find the overlapping object layers including the extra above / below gap.
        let z_threshold: CoordF = support_layer.layer.bottom_z() - gap_extra_below + EPSILON;
        let layers = object.layers();
        let idx_object_layer_overlapping: usize = Layer::idx_higher_or_equal(layers, usize::MAX, |layer: &Layer| layer.print_z >= z_threshold);
        // Collect all the object layers intersecting with this layer.
        let mut polygons_trimming: Polygons = Vec::new();
        let mut i = idx_object_layer_overlapping;
        let no_sharp_tails: ExPolygons = Vec::new();
        while i < layers.len() {
            let object_layer = &layers[i];
            if object_layer.bottom_z() > support_layer.layer.print_z + gap_extra_above - EPSILON {
                break;
            }

            let is_overlap = is_layers_overlap(support_layer, object_layer, 0.);
            let sharp_tails = self.layer_sharp_tails.get(i).unwrap_or(&no_sharp_tails);
            for expoly in &object_layer.lslices {
                // BBS
                let is_sharptail = !cx::intersection_ex(std::slice::from_ref(expoly), sharp_tails).is_empty();
                let trimming_offset: CoordF = if is_sharptail {
                    scale_(SHARP_TAIL_XY_GAP)
                } else if is_overlap {
                    gap_xy_scaled
                } else {
                    scale_(NO_OVERLAP_XY_GAP)
                };
                // `offset({ expoly }, trimming_offset, SUPPORT_SURFACES_OFFSET_PARAMETERS)`: the overload of one ExPolygon
                polygons_trimming.extend(scu::offset_expolygon(expoly, trimming_offset as f32, JoinType::jtSquare, 0.));
            }
            i += 1;
        }
        if !self.m_slicing_params.soluble_interface && cfg_bool(self.object_config(), "thick_bridges") {
            let nozzle_diameters: Vec<f64> = cfg_floats(m_print_config, "nozzle_diameter");
            // Collect all bottom surfaces, which will be extruded with a bridging flow.
            while i < layers.len() {
                let object_layer = &layers[i];
                let mut some_region_overlaps = false;
                for region in object_layer.regions() {
                    let bridging_height: CoordF = region.region().bridging_height_avg(&nozzle_diameters);
                    if object_layer.print_z - bridging_height > support_layer.layer.print_z + gap_extra_above - EPSILON {
                        break;
                    }
                    some_region_overlaps = true;

                    let is_overlap = is_layers_overlap(support_layer, object_layer, bridging_height);
                    let trimming_offset: CoordF = if is_overlap { gap_xy_scaled } else { scale_(NO_OVERLAP_XY_GAP) };
                    // `offset(region->fill_surfaces.filter_by_type(stBottomBridge), ...)`: the overload of SurfacesPtr
                    // (ClipperUtils.cpp:565), `to_polygons(expolygons_offset(surfaces, ...))`
                    let bridges: ExPolygons = region
                        .fill_surfaces
                        .surfaces
                        .iter()
                        .filter(|surface| surface.surface_type == crate::surface::SurfaceType::BottomBridge)
                        .map(|surface| surface.expolygon.clone())
                        .collect();
                    polygons_trimming.extend(scu::offset_expolygons(&bridges, trimming_offset as f32, JoinType::jtSquare, 0.));
                }
                if !some_region_overlaps {
                    break;
                }
                i += 1;
            }
        }

        polygons_trimming
    }

    /// TreeSupport.cpp:4903-5099 `void TreeSupport::prune_floating_supports()`
    ///
    /// Returns the number of regions it removed (the C++ writes it to the log).
    pub fn prune_floating_supports(&mut self) -> usize {
        use crate::print::faithful::{SupportLayerAreaGroup, SupportLayerAreaType};
        let mut num_pruned: usize = 0;
        if self.m_object.layers().is_empty() || self.m_object.m_support_layers.len() <= self.m_raft_layers {
            return num_pruned;
        }

        // Regions are stored in one vector per area type, and area_groups holds raw pointers into those vectors, so
        // both are pruned together and the pointers re-bound afterwards. The per type arrays below index by AreaType.
        const NUM_AREA_TYPES: usize = (SupportLayerAreaType::Roof1stLayer + 1) as usize;

        // A support layer is carried either by the support printed right below it, or by the model it came to rest
        // on. That model top sits gap_object_support below the support bottom, since the gap is left on purpose.
        // `std::max(0., m_slicing_params.gap_object_support)`
        let gap_object_support: CoordF = if 0. < self.m_slicing_params.gap_object_support { self.m_slicing_params.gap_object_support } else { 0. };
        // Support standing on the plate is carried by definition, and so is support resting on the raft, whose top
        // is not among the object slices searched below. This cannot be keyed off the lowest entry left in ts_layers:
        // draw_circles() drops the layers that got no node, so that entry may well be the first one up in the air.
        let grounded_z: CoordF = if self.m_raft_layers > 0 { self.m_object.m_support_layers[self.m_raft_layers - 1].layer.print_z } else { 0. };

        let branch_angle: f64 = cfg_float(self.object_config(), "tree_support_branch_angle") * M_PI / 180.;
        // Branches may then move freely, so no region can be told apart as unsupported.
        if branch_angle >= M_PI / 2. {
            return num_pruned;
        }
        let tan_branch_angle: f64 = branch_angle.tan();
        let wall_count: i32 = 1.max(cfg_int(self.object_config(), "tree_support_wall_count"));
        // Footprint of the thinnest branch the tree will ever print, and hence the least support that can still be
        // standing under a region rather than touching it by accident.
        // `M_PI * SQ(scaled<double>(MIN_BRANCH_RADIUS))`
        let min_anchor_area: f64 = M_PI * sq(MIN_BRANCH_RADIUS / SCALING_FACTOR);

        let obj_print_z: Vec<CoordF> = self.m_object.layers().iter().map(|object_layer| object_layer.print_z).collect();

        let mut support_below: ExPolygons = Vec::new(); // everything printed on the previous support layer
        for layer_nr in self.m_raft_layers..self.m_object.m_support_layers.len() {
            if (self.canceled)() {
                return num_pruned;
            }

            let ts_layer = &self.m_object.m_support_layers[layer_nr];
            // A layer that got no node keeps a zero height and is never printed, so it neither carries what is
            // above it nor interrupts the layer that does.
            if ts_layer.layer.height < EPSILON {
                continue;
            }
            // Only what sits near this layer's own regions can carry them, so the candidates are clipped to their
            // bounding box below. It stays undefined when the layer holds no region at all, or none with an extent,
            // and either way there is nothing here to prune and nothing to carry the layer above.
            let mut layer_bbox = BBoxCpp::new();
            for area_group in &ts_layer.area_groups {
                layer_bbox.merge(&BBoxCpp::of_expolygon(ts_layer.area_group_area(area_group)));
            }
            if !layer_bbox.defined {
                support_below.clear();
                continue;
            }

            // BBS multi-diameter: a folded layer carries a slab, so its material reaches down to the slab bottom
            // rather than to its own layer bottom, and the branch is free to drift over that whole height before
            // the layer below picks it up. Every bound here is taken from the carried height for that reason.
            let carry_height: CoordF = std_max3(ts_layer.layer.height, ts_layer.base_slab_height, ts_layer.interface_slab_height);
            let print_z: CoordF = ts_layer.layer.print_z;
            let bottom_z: CoordF = print_z - carry_height;
            let grounded: bool = bottom_z <= grounded_z + EPSILON;

            // The support printed below and the model are both carriers, but they are not interchangeable, so the
            // support one is kept apart: an interface may rest on nothing but the tip of the branch that spawned it,
            // and that concession must not extend to a model wall the interface merely brushes past.
            let mut carrier: ExPolygons = Vec::new();
            let mut support_carrier: ExPolygons = Vec::new();
            if !grounded {
                // A region may stick out past whatever carries it, but only by so much. Grow the carrier by that
                // much so legitimate growth is not counted as floating. drop_nodes() caps a step two different
                // ways and this has to bound both of them, since too little slack prunes healthy branches:
                //   - get_max_move_dist() caps how far a node may shift, by the node's own layer height;
                //   - its layer wide max_move_distance is how fast a fading branch loses radius and how far a
                //     polygon node may shrink or a merged node may escape, and that one scales with the wall count.
                // Both are driven by a layer height, and this support layer's own is taken for it rather than
                // config.layer_height, since independent_support_layer_height lets it run thicker than the
                // nominal one and the bound has to hold for the thicker case.
                // The last term is the line width gained under bottom expansion: passing the carried height makes
                // support_flare_step_width() return the slab's own line width rather than one layer's share of it.
                let flare_step_width: CoordF = self.support_flare_step_width(print_z, carry_height);
                // `std::min(a, b)`: `(b < a) ? b : a`
                let a = tan_branch_angle * carry_height;
                let first = if flare_step_width < a { flare_step_width } else { a };
                let carry_slack: CoordF = std_max3(first, tan_branch_angle * carry_height * wall_count as f64, flare_step_width / 2.);
                // Assembling the carrier means a union and an offset over everything below, which is a clipper
                // boolean across the whole layer. Clipping first keeps that off the parts of the layer that are too
                // far away to matter: on a plate full of small parts the rest of it is pure overhead.
                // `layer_bbox.offset(scale_(carry_slack) + SCALED_EPSILON)`: `PointClass v(delta, delta)` rounds
                let delta = (scale_(carry_slack) + scale_(EPSILON)).round_ties_even() as Coord;
                layer_bbox.min = Point::new(layer_bbox.min.x - delta, layer_bbox.min.y - delta);
                layer_bbox.max = Point::new(layer_bbox.max.x + delta, layer_bbox.max.y + delta);
                let clip_bbox = BoundingBox::from_points_minmax(layer_bbox.min, layer_bbox.max);

                // `std::upper_bound(obj_print_z.begin(), obj_print_z.end(), bottom_z + EPSILON) - obj_print_z.begin()`
                let z = bottom_z + EPSILON;
                let top_obj_layer_nr: i32 = obj_print_z.partition_point(|v| !(z < *v)) as i32 - 1;
                // Walk down by print_z instead of by a layer count. With adaptive layer height the object layers are
                // thinner than m_slicing_params.layer_height, so a count derived from that nominal height spans less
                // than the real gap and the model actually carrying this region would be missed, pruning it away.
                // One local layer of slack on top of the gap, the support bottom need not line up with an object layer.
                let lowest_z: CoordF =
                    if top_obj_layer_nr >= 0 { bottom_z - gap_object_support - self.m_object.get_layer(top_obj_layer_nr).height } else { 0. };
                let support_candidates: Polygons = cx::clip_clipper_polygons_with_subject_bbox(&support_below, &clip_bbox);
                let mut candidates: Polygons = support_candidates.clone();
                let mut obj_layer_nr = top_obj_layer_nr;
                while obj_layer_nr >= 0 && obj_print_z[obj_layer_nr as usize] >= lowest_z - EPSILON {
                    candidates.extend(cx::clip_clipper_polygons_with_subject_bbox(&self.m_object.get_layer(obj_layer_nr).lslices, &clip_bbox));
                    obj_layer_nr -= 1;
                }
                carrier = cx::offset_ex(&cx::union_ex_polygons(&candidates), scale_(carry_slack) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
                support_carrier =
                    cx::offset_ex(&cx::union_ex_polygons(&support_candidates), scale_(carry_slack) as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
            }
            let carrier_bboxes: Vec<BoundingBox> = carrier.iter().map(get_extents_expolygon).collect();
            let support_carrier_bboxes: Vec<BoundingBox> = support_carrier.iter().map(get_extents_expolygon).collect();

            let ts_layer = &mut self.m_object.m_support_layers[layer_nr];
            // `ExPolygons *regions_by_type[num_area_types] = {&ts_layer->base_areas, &ts_layer->roof_areas, &ts_layer->floor_areas, &ts_layer->roof_1st_layer};`
            let mut regions_by_type: [ExPolygons; NUM_AREA_TYPES] = [
                std::mem::take(&mut ts_layer.base_areas),
                std::mem::take(&mut ts_layer.roof_areas),
                std::mem::take(&mut ts_layer.floor_areas),
                std::mem::take(&mut ts_layer.roof_1st_layer),
            ];
            let mut kept_regions: [ExPolygons; NUM_AREA_TYPES] = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
            let mut kept_groups: Vec<SupportLayerAreaGroup> = Vec::with_capacity(ts_layer.area_groups.len());
            let mut next_index: [usize; NUM_AREA_TYPES] = [0; NUM_AREA_TYPES];
            for area_group in &ts_layer.area_groups {
                let type_ = area_group.type_;
                if type_ < SupportLayerAreaType::BaseType || type_ > SupportLayerAreaType::Roof1stLayer {
                    continue;
                }
                let index = next_index[type_ as usize];
                next_index[type_ as usize] += 1;
                if index >= regions_by_type[type_ as usize].len() {
                    continue;
                }
                // draw_circles() fills area_groups in vector order within each type and nothing reorders either
                // side before this point. Were that to change, the pruning below would drop the wrong regions and
                // the pointers would still re-bind, leaving no trace.
                debug_assert!(area_group.area == index);
                let region: &mut ExPolygon = &mut regions_by_type[type_ as usize][index];
                // Regions are dropped whole, never trimmed: what is kept here feeds the next layer's carrier, so
                // trimming would eat a little more off every layer and erode a healthy branch into a thread over a
                // hundred layers. A branch landing on the edge of its carrier still prints, so only drop the ones
                // mostly out in the air.
                let is_interface = type_ == SupportLayerAreaType::RoofType || type_ == SupportLayerAreaType::Roof1stLayer;
                let region_area: f64 = region.area();
                let mut carried = grounded;
                if !grounded && region_area > 0. {
                    if is_interface {
                        // An interface spans the whole overhang that spawned it, so it cannot be held to a fraction
                        // of its own area: it routinely rests on nothing but a branch tip a fraction that wide. That
                        // concession is what MIN_CARRIED_RATIO_INTERFACE buys, and it only makes sense measured
                        // against the support below. Measured against the model too, an interface passing a wall on
                        // its way up clears the bar on a few percent of incidental overlap, and once kept it becomes
                        // the carrier for the layer above, so a single such region grows a floating stack. The model
                        // still carries an interface that comes to rest on it, but then it has to do so as squarely
                        // as a base standing on the same ledge would.
                        carried = carried_area(&support_carrier, &support_carrier_bboxes, region) >= MIN_CARRIED_RATIO_INTERFACE * region_area
                            || carried_area(&carrier, &carrier_bboxes, region) >= MIN_CARRIED_RATIO * region_area;
                    } else {
                        // A region is the union of everything the layer grew, so a branch standing on its own can end
                        // up merged with an overhang area many times its size that a node spawned this layer. Judged
                        // by fraction alone the union comes out mostly floating and the healthy branch is deleted
                        // along with it, which orphans every layer above it for the rest of the object. A carried
                        // patch as wide as the thinnest branch that can be printed is an anchor on its own, however
                        // much unsupported area happens to share the region with it. Only the support below counts
                        // as such an anchor: a region merely brushing a model wall meets it over an area that small
                        // as well, and keeping it would grow the floating stack the interface rule guards against.
                        carried = carried_area(&carrier, &carrier_bboxes, region) >= MIN_CARRIED_RATIO * region_area
                            || carried_area(&support_carrier, &support_carrier_bboxes, region) >= min_anchor_area;
                    }
                }
                if !carried {
                    num_pruned += 1;
                    continue;
                }
                kept_regions[type_ as usize].push(std::mem::take(region));
                kept_groups.push(area_group.clone());
            }

            // `for (int type = 0; type < num_area_types; type++) *regions_by_type[type] = std::move(kept_regions[type]);`
            let [base, roof, floor, roof1] = kept_regions;
            ts_layer.base_areas = base;
            ts_layer.roof_areas = roof;
            ts_layer.floor_areas = floor;
            ts_layer.roof_1st_layer = roof1;
            let mut rebound_index: [usize; NUM_AREA_TYPES] = [0; NUM_AREA_TYPES];
            for area_group in kept_groups.iter_mut() {
                area_group.area = rebound_index[area_group.type_ as usize];
                rebound_index[area_group.type_ as usize] += 1;
            }
            ts_layer.area_groups = kept_groups;

            // support_islands feeds retraction avoidance, keep it in sync with what is left.
            sync_support_islands(ts_layer);

            support_below.clear();
            support_below.reserve(ts_layer.area_groups.len());
            for area_group in &ts_layer.area_groups {
                support_below.push(ts_layer.area_group_area(area_group).clone());
            }
        }

        // BOOST_LOG_TRIVIAL(info) << "Tree support floating region pruning done. " << num_pruned << " regions removed.";
        num_pruned
    }
}

// `std::max({a, b, c})`: the first of the greatest (std::max_element).
fn std_max3(a: f64, b: f64, c: f64) -> f64 {
    let mut largest = a;
    if largest < b {
        largest = b;
    }
    if largest < c {
        largest = c;
    }
    largest
}

// ============================================================================
// TreeSupport::fold_nodes_multi_diameter (TreeSupport.cpp:1806-2888) and
// TreeSupport::smooth_line_width_multi_diameter (TreeSupport.cpp:2894-2976)
// ============================================================================

/// TreeSupport.cpp:1856-1878 `struct Carrier` of fold_nodes_multi_diameter(): one column's slab. Its extrusion
/// spans from bottom_z up to the carrier, the column's top.
///
/// `Carrier *` is the key of the carrier: the index of its slab and its support layer (`carriers[slab][layer]`).
#[derive(Debug, Clone)]
struct FoldCarrierColumn {
    slab_idx: usize,
    top_layer: usize,
    base_source_layer: usize,
    bottom_obj_layer: usize,
    obj_layer: usize,
    bottom_z: CoordF,
    top_z: CoordF,
    slab_height: CoordF,
    original_base_source_layer: usize,
    original_bottom_obj_layer: usize,
    original_bottom_z: CoordF,
    original_slab_height: CoordF,
    footprint: ExPolygons,
    base_footprint: ExPolygons,
    interface_footprint: ExPolygons,
    footprint_bbox: BBoxCpp,
    base_footprint_bbox: BBoxCpp,
    partial_heads: Vec<usize>,
    lower_carrier: Option<FoldCarrierKey>,
    folds: bool,
}

type FoldCarrierKey = (usize, usize);

impl Default for FoldCarrierColumn {
    fn default() -> Self {
        FoldCarrierColumn {
            slab_idx: 0,
            top_layer: 0,
            base_source_layer: 0,
            bottom_obj_layer: 0,
            obj_layer: 0,
            bottom_z: 0.,
            top_z: 0.,
            slab_height: 0.,
            original_base_source_layer: 0,
            original_bottom_obj_layer: 0,
            original_bottom_z: 0.,
            original_slab_height: 0.,
            footprint: Vec::new(),
            base_footprint: Vec::new(),
            interface_footprint: Vec::new(),
            footprint_bbox: BBoxCpp::new(),
            base_footprint_bbox: BBoxCpp::new(),
            partial_heads: Vec::new(),
            lower_carrier: None,
            folds: false,
        }
    }
}

// TreeSupport.cpp:1841-1844 the lambda `foldable` of fold_nodes_multi_diameter(): only nodes that are going to
// move belong to a column: with the interface left on the fine grid, an interface node is not what a slab emits,
// so it must neither pick a carrier nor be swallowed.
fn fold_foldable(node: &SupportNode, fold_interface: bool) -> bool {
    node.distance_to_top >= 0 && (fold_interface || node.support_roof_layers_below < 0)
}

// TreeSupport.cpp:1846-1853 the lambda `for_each_parent`: parents holds parent plus every neighbour merged into
// the node; older nodes may only carry parent.
fn fold_parents(node: &SupportNode) -> &[NodeId] {
    if !node.parents.is_empty() {
        &node.parents
    } else {
        node.parent.as_slice()
    }
}

// TreeSupport.cpp:2085-2095 the lambda `move_target`
fn fold_move_target(
    column_top: &[HashMap<NodeId, usize>],
    carriers: &[BTreeMap<usize, FoldCarrierColumn>],
    nodes: &[SupportNode],
    slab_idx: usize,
    node: NodeId,
) -> Option<FoldCarrierKey> {
    let top = column_top[slab_idx].get(&node)?;
    let mut key: FoldCarrierKey = (slab_idx, *top);
    let print_z = nodes[node].print_z;
    loop {
        let carrier = &carriers[key.0][&key.1];
        match carrier.lower_carrier {
            Some(lower) if print_z <= carrier.bottom_z + EPSILON => key = lower,
            _ => break,
        }
    }
    let carrier = &carriers[key.0][&key.1];
    if carrier.folds && print_z > carrier.bottom_z + EPSILON {
        Some(key)
    } else {
        None
    }
}

// TreeSupport.cpp:2100-2124 the lambda `refuse_shared_carriers`: a carrier extrudes its whole layer at the slab
// height, so it may not share the layer with a node that stays there: that node was already printed on the layers
// below and would be extruded a second time. Refusing a carrier leaves its own column behind on lower layers,
// which may be carriers in turn.
fn fold_refuse_shared_carriers(
    column_top: &[HashMap<NodeId, usize>],
    carriers: &mut [BTreeMap<usize, FoldCarrierColumn>],
    contact_nodes: &[Vec<NodeId>],
    nodes: &[SupportNode],
    fold_interface: bool,
) {
    let mut refused = true;
    while refused {
        refused = false;
        for slab_idx in 0..carriers.len() {
            let tops = &column_top[slab_idx];
            let carrier_layers: Vec<usize> = carriers[slab_idx].keys().copied().collect();
            for carrier_layer in carrier_layers {
                if !carriers[slab_idx][&carrier_layer].folds {
                    continue;
                }
                let shared = contact_nodes[carrier_layer].iter().any(|&node| {
                    if !fold_foldable(&nodes[node], fold_interface) {
                        return false;
                    }
                    match tops.get(&node) {
                        Some(top) => *top != carrier_layer && fold_move_target(column_top, carriers, nodes, slab_idx, node).is_none(),
                        None => false,
                    }
                });
                if shared {
                    carriers[slab_idx].get_mut(&carrier_layer).unwrap().folds = false;
                    refused = true;
                }
            }
        }
    }
}

// TreeSupport.cpp:2255-2272 the lambda `refuse_carriers`
fn fold_refuse_carriers(
    refused: &std::collections::BTreeSet<FoldCarrierKey>,
    misaligned_carriers: &mut usize,
    column_top: &[HashMap<NodeId, usize>],
    carriers: &mut [BTreeMap<usize, FoldCarrierColumn>],
    contact_nodes: &[Vec<NodeId>],
    nodes: &[SupportNode],
    fold_interface: bool,
) {
    *misaligned_carriers += refused.len();
    for key in refused {
        carriers[key.0].get_mut(&key.1).unwrap().folds = false;
    }
    // Survivors changed, so every split is rebuilt from its original span in the next pass.
    for slab_carriers in carriers.iter_mut() {
        for carrier in slab_carriers.values_mut() {
            if carrier.lower_carrier.is_none() {
                continue;
            }
            carrier.base_source_layer = carrier.original_base_source_layer;
            carrier.bottom_obj_layer = carrier.original_bottom_obj_layer;
            carrier.bottom_z = carrier.original_bottom_z;
            carrier.slab_height = carrier.original_slab_height;
            carrier.lower_carrier = None;
        }
    }
    fold_refuse_shared_carriers(column_top, carriers, contact_nodes, nodes, fold_interface);
}

// TreeSupport.cpp:2130-2177 the lambda `evaluate_folded_radius`. The columns of a slab are joined along the
// parent of a node when both move to the same carrier (a union-find that keeps the greatest radius at its root);
// the radius of a node is the greatest of its group, whatever the order the std::unordered_map hands the nodes in.
fn fold_evaluate_folded_radius(column_top: &[HashMap<NodeId, usize>], nodes: &[SupportNode], moves: &HashMap<NodeId, usize>) -> HashMap<NodeId, CoordF> {
    struct FoldedColumn {
        parent: usize,
        max_radius: CoordF,
        carrier_layer: usize,
    }
    fn find_column(columns: &mut [FoldedColumn], mut column: usize) -> usize {
        let mut root = column;
        while columns[root].parent != root {
            root = columns[root].parent;
        }
        while columns[column].parent != column {
            let next = columns[column].parent;
            columns[column].parent = root;
            column = next;
        }
        root
    }
    let mut folded_radius: HashMap<NodeId, CoordF> = HashMap::new();
    for tops in column_top {
        let mut members: Vec<NodeId> = tops.keys().copied().filter(|node| moves.contains_key(node)).collect();
        members.sort_unstable();
        let mut index: HashMap<NodeId, usize> = HashMap::with_capacity(members.len());
        let mut columns: Vec<FoldedColumn> = Vec::with_capacity(members.len());
        for &node in &members {
            index.insert(node, columns.len());
            columns.push(FoldedColumn { parent: columns.len(), max_radius: nodes[node].radius, carrier_layer: moves[&node] });
        }
        for (i, &node) in members.iter().enumerate() {
            let Some(parent) = nodes[node].parent else {
                continue;
            };
            let Some(&parent_column) = index.get(&parent) else {
                continue;
            };
            if columns[parent_column].carrier_layer != columns[i].carrier_layer {
                continue;
            }
            let node_root = find_column(&mut columns, i);
            let parent_root = find_column(&mut columns, parent_column);
            if node_root != parent_root {
                columns[parent_root].max_radius = columns[parent_root].max_radius.max(columns[node_root].max_radius);
                columns[node_root].parent = parent_root;
            }
        }
        for (i, &node) in members.iter().enumerate() {
            let root = find_column(&mut columns, i);
            // `folded_radius.emplace(node, ...)`: the first entry of a node stays
            folded_radius.entry(node).or_insert(columns[root].max_radius);
        }
    }
    folded_radius
}

// TreeSupport.cpp:2183-2236 the lambda `evaluate_roof_layers`: { distance_to_top, support_roof_layers_below } in
// printed layers of every node, were the nodes of `moves` printed on the layers it names.
fn fold_evaluate_roof_layers(
    contact_nodes: &[Vec<NodeId>],
    nodes: &[SupportNode],
    num_layers: usize,
    fold_interface: bool,
    moves: &HashMap<NodeId, usize>,
) -> HashMap<NodeId, (i32, i32)> {
    let mut counters: HashMap<NodeId, (i32, i32)> = HashMap::new();
    if !fold_interface {
        return counters;
    }
    // The roof counter is carried down the chain like steps. Deriving it from the node's own fine counters
    // (roof + distance_to_top - steps) breaks below a merge: the merged node keeps the larger distance_to_top
    // of one branch and the larger roof of another, so their sum no longer matches the chain above and a deep
    // base node could come out as roof / roof_1st_layer again.
    struct ChainState {
        steps: i32,
        roof: i32,
        printed: usize,
    }
    let mut chain: HashMap<NodeId, ChainState> = HashMap::new();
    let mut merged: Vec<NodeId> = Vec::new();
    // A parent that another parent of the merge holds whole (a circle in a polygon, a small circle in a large
    // one) does not count; the outer parent decides.
    let contained_in = |inner: NodeId, outer: NodeId| -> bool {
        let (inner_n, outer_n) = (&nodes[inner], &nodes[outer]);
        if inner == outer || inner_n.type_ == TreeNodeType::EPolygon {
            return false;
        }
        if outer_n.type_ == TreeNodeType::EPolygon {
            return !outer_n.overhang.contour.points.is_empty() && cx::contains(&outer_n.overhang, &inner_n.position, true);
        }
        let (r_in, r_out) = (scale_(inner_n.radius), scale_(outer_n.radius));
        r_out > r_in + scale_(EPSILON) && point_dist(outer_n.position, inner_n.position) + r_in <= r_out
    };
    for layer_nr in (0..num_layers).rev() {
        for &node in &contact_nodes[layer_nr] {
            if nodes[node].distance_to_top < 0 {
                continue;
            }
            let printed: usize = moves.get(&node).copied().unwrap_or(layer_nr);
            let mut steps: i32 = -1;
            let mut roof: i32 = 0;
            let mut has_parent = false;
            merged.clear();
            for &parent in fold_parents(&nodes[node]) {
                if chain.contains_key(&parent) {
                    merged.push(parent);
                }
            }
            for &parent in &merged {
                if merged.len() > 1 && merged.iter().any(|&other| contained_in(parent, other)) {
                    continue;
                }
                let state = &chain[&parent];
                let step: i32 = if state.printed != printed { 1 } else { 0 };
                // Where branches merge: the greater number of layers, the smaller number of interface layers; the
                // merged node counts as the body of the deeper branch.
                steps = steps.max(state.steps + step);
                roof = if has_parent { roof.min(state.roof - step) } else { state.roof - step };
                has_parent = true;
            }
            if !has_parent {
                steps = nodes[node].distance_to_top;
                roof = nodes[node].support_roof_layers_below;
            }
            chain.insert(node, ChainState { steps, roof, printed });
            counters.insert(node, (steps, roof));
        }
    }
    counters
}

// TreeSupport.cpp:2246-2250 the lambda `footprints_overlap`
fn fold_footprints_overlap(lhs: &ExPolygons, lhs_bbox: &BBoxCpp, rhs: &ExPolygons, rhs_bbox: &BBoxCpp) -> bool {
    !lhs.is_empty() && !rhs.is_empty() && lhs_bbox.overlap(rhs_bbox) && !cx::intersection_ex(lhs, rhs).is_empty()
}

// TreeSupport.cpp:2251-2253 the lambda `contains_span`
fn fold_contains_span(outer: &FoldCarrierColumn, inner: &FoldCarrierColumn) -> bool {
    outer.bottom_z <= inner.bottom_z + EPSILON && outer.top_z >= inner.top_z - EPSILON
}

/// TreeSupport.cpp:1898-1928 the lambdas `probe_of` and `cached_probe_of` of fold_nodes_multi_diameter(): the raw
/// geometry draw_circles() will render of a node. Collision clipping is deliberately left out: it may only shrink
/// this probe, so refusing an ambiguous fold remains the safe outcome.
struct FoldProbe {
    branch_circle: Polygon,
    branch_radius: CoordF,
    branch_radius_scaled: CoordF,
    square_support: bool,
    on_buildplate_only: bool,
    xy_distance_scaled: Coord,
    base_probe_cache: HashMap<NodeId, ExPolygons>,
    interface_probe_cache: HashMap<NodeId, ExPolygons>,
}

impl FoldProbe {
    fn probe_of(&self, node: &SupportNode, use_interface_shape: bool, probe: &mut ExPolygons, radius: CoordF) {
        if node.distance_to_top < 0 {
            return;
        }
        if node.type_ == TreeNodeType::EPolygon {
            if node.overhang.contour.points.is_empty() {
                return;
            }
            // `offset_ex({ node->overhang }, float(xy_distance_scaled))`: the overload of one ExPolygon
            *probe = cx::offset_ex_expolygon(&node.overhang, self.xy_distance_scaled as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT);
            return;
        }
        if use_interface_shape && !self.on_buildplate_only {
            if node.overhang.contour.points.is_empty() {
                return;
            }
            *probe = if node.overhang.contour.points.len() > 100 || node.overhang.holes.len() > 1 {
                vec![node.overhang.clone()]
            } else {
                cx::offset_ex_expolygon(&node.overhang, self.xy_distance_scaled as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT)
            };
            return;
        }

        probe.push(tree_support_node_disk(node, &self.branch_circle, self.branch_radius, self.branch_radius_scaled, self.square_support, radius));
    }

    fn cached_probe_of(&mut self, nodes: &[SupportNode], node: NodeId, use_interface_shape: bool) -> &ExPolygons {
        let cached = if use_interface_shape { self.interface_probe_cache.contains_key(&node) } else { self.base_probe_cache.contains_key(&node) };
        if !cached {
            let mut probe: ExPolygons = Vec::new();
            self.probe_of(&nodes[node], use_interface_shape, &mut probe, nodes[node].radius);
            if use_interface_shape {
                self.interface_probe_cache.insert(node, probe);
            } else {
                self.base_probe_cache.insert(node, probe);
            }
        }
        if use_interface_shape {
            &self.interface_probe_cache[&node]
        } else {
            &self.base_probe_cache[&node]
        }
    }
}

impl<'a> TreeSupport<'a> {
    /// TreeSupport.cpp:1806-2888 `void TreeSupport::fold_nodes_multi_diameter()`
    ///
    /// The maps and sets the C++ keys by the address of a node or of a carrier are keyed by the node (its index)
    /// and by the slab and layer of the carrier here. None of the loops over them depends on their order: every
    /// one of them either writes what it reads of its own key or folds with a maximum.
    pub fn fold_nodes_multi_diameter(&mut self) {
        use std::collections::BTreeSet;
        self.m_fold_carriers.clear();
        self.m_fold_slab_of_layer.clear();
        self.m_base_only_fold_layers.clear();

        if !self.m_support_merge_plan.enabled || self.m_support_merge_plan.slabs.is_empty() {
            return;
        }
        let plan: SupportMergePlan = self.m_support_merge_plan.clone();
        let num_slabs = plan.slabs.len();

        let layer_heights: Vec<LayerHeightData> = self.m_ts_data.layer_heights.clone();
        let num_layers: usize = self.contact_nodes.len().min(layer_heights.len());
        if num_layers == 0 {
            return;
        }

        // Support layers grouped by the slab they fall into, ascending. A layer with no height occupies no Z, so
        // it can neither carry an extrusion nor be swallowed into one.
        let mut layers_per_slab: Vec<Vec<usize>> = vec![Vec::new(); num_slabs];
        for layer_nr in 0..num_layers {
            if layer_heights[layer_nr].height < EPSILON {
                continue;
            }
            // `plan.slab_for_layer(layer_heights[layer_nr].obj_layer_nr)` (SupportMergePlan.cpp:179-185), its index
            let obj_layer_nr = layer_heights[layer_nr].obj_layer_nr;
            if obj_layer_nr >= plan.slab_index_per_layer.len() {
                continue;
            }
            let slab_idx = plan.slab_index_per_layer[obj_layer_nr];
            if slab_idx < 0 {
                continue;
            }
            layers_per_slab[slab_idx as usize].push(layer_nr);
        }

        let explicit_interface_filament: i32 = cfg_int(self.object_config(), "support_interface_filament");
        let xy_distance_scaled: Coord = scale_(self.m_ts_data.m_xy_distance) as Coord;
        // One decision for the whole object: an interface that folds on some slabs and not on others would leave
        // the roof counters in two different units along one chain.
        let fold_interface: bool = plan.interface_fold_enabled;
        let num_top_interface_layers: usize = self.m_support_params.num_top_interface_layers;
        let m_raft_layers = self.m_raft_layers;

        // Per slab: the column top of every foldable node, and the candidate carriers keyed by support layer.
        let mut column_top: Vec<HashMap<NodeId, usize>> = vec![HashMap::new(); num_slabs];
        let mut carriers: Vec<BTreeMap<usize, FoldCarrierColumn>> = vec![BTreeMap::new(); num_slabs];

        let square_support = self.avg_node_per_layer > 200;
        let circle_resolution: i32 = if square_support { 4 } else { 25 };
        let branch_radius: CoordF = cfg_float(self.object_config(), "tree_support_branch_diameter") / 2.;
        let branch_radius_scaled: CoordF = scale_(branch_radius);
        let mut branch_circle = Polygon::new();
        for i in 0..circle_resolution {
            // `double(i) / circle_resolution * TAU + (square_support ? PI / 4. + nodes_angle : 0.)`: the product is
            // contracted with the sum
            let angle: f64 = (i as f64 / circle_resolution as f64).mul_add(TAU, if square_support { M_PI / 4. + self.nodes_angle as f64 } else { 0. });
            // cos(angle) and sin(angle) of one argument: one `__sincos_stret` call
            let (sin_a, cos_a) = crate::clipper::sin_cos(angle);
            branch_circle.points.push(point_from_f64(cos_a * branch_radius_scaled, sin_a * branch_radius_scaled));
        }
        let mut prober = FoldProbe {
            branch_circle,
            branch_radius,
            branch_radius_scaled,
            square_support,
            on_buildplate_only: cfg_bool(self.object_config(), "support_on_build_plate_only"),
            xy_distance_scaled,
            base_probe_cache: HashMap::new(),
            interface_probe_cache: HashMap::new(),
        };

        // Pass 1: columns and carriers, per slab.
        for slab_idx in 0..num_slabs {
            let layers = &layers_per_slab[slab_idx];
            if layers.len() < 2 {
                continue;
            }
            let nodes = &self.m_ts_data.contact_nodes;

            // Walk the slab top down so a node's parents inside the slab already know their column top. A parent
            // outside the slab - or one that does not move - is absent from the map and ends the column there.
            // When differently aligned columns merge, start a new carrier at the merge layer. This leaves their
            // partial heads on the fine grid without carrying that refusal into the common trunk below.
            for &layer in layers.iter().rev() {
                for &node in &self.contact_nodes[layer] {
                    if !fold_foldable(&nodes[node], fold_interface) {
                        continue;
                    }
                    let tops = &mut column_top[slab_idx];
                    let mut top: usize = layer;
                    let mut parent_top: usize = 0;
                    let mut has_parent_top = false;
                    let mut different_parent_tops = false;
                    for parent in fold_parents(&nodes[node]) {
                        let Some(&parent_it) = tops.get(parent) else {
                            continue;
                        };
                        if !has_parent_top {
                            parent_top = parent_it;
                            has_parent_top = true;
                        } else if parent_it != parent_top {
                            different_parent_tops = true;
                        }
                    }
                    if has_parent_top && !different_parent_tops {
                        top = top.max(parent_top);
                    } else if different_parent_tops {
                        let carrier = carriers[slab_idx].entry(top).or_default();
                        for parent in fold_parents(&nodes[node]) {
                            if let Some(&parent_it) = tops.get(parent) {
                                if parent_it != top {
                                    carrier.partial_heads.push(parent_it);
                                }
                            }
                        }
                    }
                    tops.insert(node, top);
                    carriers[slab_idx].entry(top).or_default();
                }
            }
            let tops = &column_top[slab_idx];
            if tops.is_empty() {
                continue;
            }

            let front: usize = layers[0];
            let slab_bottom_z: CoordF = layer_heights[front].print_z - layer_heights[front].height;
            let slab_bottom_obj_layer: usize = layer_heights[front].obj_layer_nr;
            // Without a raft the first band starts on the bed, so its bed face is the representative cross
            // section - the same choice the model band planner makes, and the one that keeps the adhesion
            // geometry. With a raft the band rests on the raft and behaves like any other.
            let bed_anchor: bool = plan.slabs[slab_idx].bottom_anchor && m_raft_layers == 0;

            // Carriers ascend, and so does the span each one has to clear: grow it incrementally.
            let mut span_collision: ExPolygons = Vec::new();
            let mut span_top_obj_layer: usize = slab_bottom_obj_layer;
            let carrier_layers: Vec<usize> = carriers[slab_idx].keys().copied().collect();
            for carrier_layer in carrier_layers {
                let carrier = carriers[slab_idx].get_mut(&carrier_layer).unwrap();
                carrier.slab_idx = slab_idx;
                carrier.top_layer = carrier_layer;
                carrier.obj_layer = layer_heights[carrier_layer].obj_layer_nr;
                carrier.bottom_obj_layer = slab_bottom_obj_layer;
                carrier.bottom_z = slab_bottom_z;
                carrier.top_z = layer_heights[carrier_layer].print_z;
                carrier.slab_height = carrier.top_z - slab_bottom_z;
                // A column whose top is the slab's bottom layer has nothing to fold.
                if carrier.slab_height <= layer_heights[carrier_layer].height + EPSILON {
                    continue;
                }
                // A carrier below the band top shortens the slab, so the nozzle has to be able to extrude the
                // shortened height as well. Too thin and the column simply stays on the fine grid, which is what
                // R4 already does for a chain head hanging off the top of the ladder.
                if !crate::adaptive_layer_merge::adaptive_layer_height_range(plan.slabs[slab_idx].nozzle_diameter).contains(carrier.slab_height) {
                    continue;
                }

                carrier.base_source_layer = carrier_layer;
                if bed_anchor {
                    for node in &self.contact_nodes[front] {
                        if tops.get(node) == Some(&carrier_layer) {
                            carrier.base_source_layer = front;
                            break;
                        }
                    }
                }
                carrier.original_base_source_layer = carrier.base_source_layer;
                carrier.original_bottom_obj_layer = carrier.bottom_obj_layer;
                carrier.original_bottom_z = carrier.bottom_z;
                carrier.original_slab_height = carrier.slab_height;

                // A slab is one extrusion over its whole Z span, so it has to clear the object everywhere in that
                // span and not only at the face its cross-section was taken from. Refuse the fold outright when
                // nothing would survive: printing those layers thin is wasteful for this nozzle, but dropping them
                // removes the support altogether, and keeping the thin ones is the lesser evil.
                let mut appended = false;
                while span_top_obj_layer < carrier.obj_layer {
                    if span_top_obj_layer < self.m_ts_data.m_layer_outlines.len() {
                        append(
                            &mut span_collision,
                            cx::offset_ex(&self.m_ts_data.m_layer_outlines[span_top_obj_layer], xy_distance_scaled as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT),
                        );
                        appended = true;
                    }
                    span_top_obj_layer += 1;
                }
                if appended {
                    span_collision = cx::union_ex(&span_collision);
                }
                if !span_collision.is_empty() {
                    let mut any_foldable = false;
                    let mut survives = false;
                    // Only this column's nodes on the faces it would draw matter. The roof counters are still in
                    // fine units here, so the interface is not told apart: a slab whose contact disks all vanish
                    // is no better than one whose base does.
                    for probe_layer in [carrier.base_source_layer, carrier_layer] {
                        for &node in &self.contact_nodes[probe_layer] {
                            if tops.get(&node) != Some(&carrier_layer) {
                                continue;
                            }
                            let n = &nodes[node];
                            let use_interface_shape =
                                num_top_interface_layers > 0 && (n.support_roof_layers_below > 1 || (n.support_roof_layers_below >= 0 && !n.is_sharp_tail));
                            let probe = prober.cached_probe_of(nodes, node, use_interface_shape);
                            if probe.is_empty() {
                                continue;
                            }
                            any_foldable = true;
                            if !cx::diff_ex(probe, &span_collision).is_empty() {
                                survives = true;
                                break;
                            }
                        }
                        if survives || probe_layer == carrier_layer {
                            break;
                        }
                    }
                    if any_foldable && !survives {
                        continue;
                    }
                }

                carrier.folds = true;
            }
        }

        // A common carrier replaces only the material below its synchronization boundary. Its differently
        // aligned heads stay on their original layers; refusing them must not disable the common trunk.
        for slab_idx in 0..num_slabs {
            let mut partial_heads: Vec<usize> = Vec::new();
            for entry in carriers[slab_idx].values() {
                if entry.folds {
                    partial_heads.extend_from_slice(&entry.partial_heads);
                }
            }
            let mut refused_heads: HashSet<usize> = HashSet::new();
            while let Some(partial_head) = partial_heads.pop() {
                if !refused_heads.insert(partial_head) {
                    continue;
                }
                // `carriers[slab_idx].at(partial_head)`
                let carrier = carriers[slab_idx].get_mut(&partial_head).expect("fold_nodes_multi_diameter: the carrier of a partial head");
                carrier.folds = false;
                partial_heads.extend_from_slice(&carrier.partial_heads);
            }
        }

        fold_refuse_shared_carriers(&column_top, &mut carriers, &self.contact_nodes, &self.m_ts_data.contact_nodes, fold_interface);

        // counters is { distance_to_top, support_roof_layers_below } in printed layers.
        // 2238-2245 the lambda `is_interface_role`. `distance_to_top >= m_support_params.num_top_interface_layers`
        // compares the int as a size_t: a negative distance is above every count.
        let is_interface_role = |counters: (i32, i32), printed_layer: usize| -> bool {
            let (distance_to_top, roof_layers) = counters;
            num_top_interface_layers > 0
                && layer_heights[printed_layer].obj_layer_nr > 0
                && (roof_layers > 0 || (roof_layers == 0 && (distance_to_top as i64 as u64) >= num_top_interface_layers as u64))
        };
        // 2291-2294 the lambda `use_interface_shape`
        let use_interface_shape =
            |node: &SupportNode, roof_layers: i32| -> bool { num_top_interface_layers > 0 && (roof_layers > 1 || (roof_layers >= 0 && !node.is_sharp_tail)) };

        let mut folded_radius: HashMap<NodeId, CoordF>;
        let mut misaligned_carriers: usize = 0;
        let moves_to: HashMap<NodeId, usize>;
        let final_counters: HashMap<NodeId, (i32, i32)>;
        // Every folded span is built on the support-layer grid. Interface material may contain a finer span:
        // the final-area pass removes what it covers. A genuine partial overlap cannot be repaired in XY, so
        // reject the less useful carrier first; the fixed-point check also rejects the survivor if the fine
        // layers left behind still overlap it unsafely.
        // Recompute the tentative printed counters and footprints after every refusal. The roof/base split is
        // defined in printed-layer units, so using the original counters here could omit a node that Pass 2
        // later turns into interface material.
        loop {
            let nodes = &self.m_ts_data.contact_nodes;
            let mut tentative_moves: HashMap<NodeId, usize> = HashMap::new();
            for slab_idx in 0..num_slabs {
                for &node in column_top[slab_idx].keys() {
                    if let Some(carrier) = fold_move_target(&column_top, &carriers, nodes, slab_idx, node) {
                        // `tentative_moves.emplace(node, carrier->top_layer)`
                        tentative_moves.entry(node).or_insert(carriers[carrier.0][&carrier.1].top_layer);
                    }
                }
            }

            let future_counters: HashMap<NodeId, (i32, i32)> = fold_evaluate_roof_layers(&self.contact_nodes, nodes, num_layers, fold_interface, &tentative_moves);
            // 2286-2290 the lambda `future_of`
            let future_of = |node: NodeId| -> (i32, i32) {
                match future_counters.get(&node) {
                    Some(counters) => *counters,
                    None => (nodes[node].distance_to_top, nodes[node].support_roof_layers_below),
                }
            };

            // Every loop follows a split or refusal, so the radius groups always match the current destinations.
            folded_radius = fold_evaluate_folded_radius(&column_top, nodes, &tentative_moves);
            for slab_carriers in carriers.iter_mut() {
                for carrier in slab_carriers.values_mut() {
                    carrier.footprint.clear();
                    carrier.base_footprint.clear();
                    carrier.interface_footprint.clear();
                    carrier.footprint_bbox = BBoxCpp::new();
                    carrier.base_footprint_bbox = BBoxCpp::new();
                }
            }
            for slab_idx in 0..num_slabs {
                for &layer_nr in &layers_per_slab[slab_idx] {
                    for &node in &self.contact_nodes[layer_nr] {
                        let Some(&move_it) = tentative_moves.get(&node) else {
                            continue;
                        };
                        // `carriers[slab_idx].at(move_it->second)`
                        let carrier = carriers[slab_idx].get_mut(&move_it).expect("fold_nodes_multi_diameter: the carrier of a move");
                        let counters = future_of(node);
                        let roof_layers = counters.1;
                        let interface_node = is_interface_role(counters, carrier.top_layer);
                        if !carrier.folds || (!interface_node && layer_nr != carrier.base_source_layer) {
                            continue;
                        }
                        let mut final_base_probe: ExPolygons = Vec::new();
                        let probe: &ExPolygons = if interface_node {
                            prober.cached_probe_of(nodes, node, use_interface_shape(&nodes[node], roof_layers))
                        } else {
                            let radius = folded_radius.get(&node).copied().unwrap_or(nodes[node].radius);
                            prober.probe_of(&nodes[node], false, &mut final_base_probe, radius);
                            &final_base_probe
                        };
                        if probe.is_empty() {
                            continue;
                        }
                        if interface_node {
                            carrier.interface_footprint.extend(probe.iter().cloned());
                        } else {
                            carrier.base_footprint.extend(probe.iter().cloned());
                        }
                    }
                }
            }
            for slab_carriers in carriers.iter_mut() {
                for carrier in slab_carriers.values_mut() {
                    if !carrier.folds {
                        continue;
                    }
                    if !carrier.base_footprint.is_empty() {
                        carrier.base_footprint = cx::union_ex(&carrier.base_footprint);
                        carrier.base_footprint_bbox = BBoxCpp::of_expolygons(&carrier.base_footprint);
                        carrier.footprint.extend(carrier.base_footprint.iter().cloned());
                    }
                    if !carrier.interface_footprint.is_empty() {
                        carrier.interface_footprint = cx::union_ex(&carrier.interface_footprint);
                        carrier.footprint.extend(carrier.interface_footprint.iter().cloned());
                    }
                    if !carrier.footprint.is_empty() {
                        carrier.footprint_bbox = BBoxCpp::of_expolygons(&carrier.footprint);
                    }
                }
            }

            let mut folded_spans: Vec<FoldCarrierKey> = Vec::new();
            for (slab_idx, slab_carriers) in carriers.iter().enumerate() {
                for (layer, entry) in slab_carriers {
                    if entry.folds {
                        folded_spans.push((slab_idx, *layer));
                    }
                }
            }
            // std::sort by the bottom, the slab and the top layer: no two spans compare equal.
            folded_spans.sort_by(|lhs, rhs| {
                let (l, r) = (&carriers[lhs.0][&lhs.1], &carriers[rhs.0][&rhs.1]);
                if l.bottom_z != r.bottom_z {
                    return l.bottom_z.partial_cmp(&r.bottom_z).unwrap_or(std::cmp::Ordering::Equal);
                }
                if l.slab_idx != r.slab_idx {
                    return l.slab_idx.cmp(&r.slab_idx);
                }
                l.top_layer.cmp(&r.top_layer)
            });

            // Select every split from this immutable footprint/span snapshot. Applying a split while scanning
            // would make later pairs depend on sort order and could leave an old partial-overlap refusal behind.
            #[derive(Default)]
            struct SplitDecision {
                highest_inner: Option<FoldCarrierKey>,
                required: bool,
            }
            let mut split_candidates: BTreeMap<FoldCarrierKey, SplitDecision> = BTreeMap::new();
            for lhs in 0..folded_spans.len() {
                let lhs_key = folded_spans[lhs];
                let lhs_carrier = &carriers[lhs_key.0][&lhs_key.1];
                for rhs in lhs + 1..folded_spans.len() {
                    let rhs_key = folded_spans[rhs];
                    let rhs_carrier = &carriers[rhs_key.0][&rhs_key.1];
                    if rhs_carrier.bottom_z >= lhs_carrier.top_z - EPSILON {
                        break;
                    }
                    let lhs_contains_rhs = fold_contains_span(lhs_carrier, rhs_carrier);
                    let rhs_contains_lhs = fold_contains_span(rhs_carrier, lhs_carrier);
                    if lhs_contains_rhs == rhs_contains_lhs {
                        continue;
                    }

                    let (outer_key, inner_key) = if lhs_contains_rhs { (lhs_key, rhs_key) } else { (rhs_key, lhs_key) };
                    let (outer, inner) = if lhs_contains_rhs { (lhs_carrier, rhs_carrier) } else { (rhs_carrier, lhs_carrier) };
                    if outer.slab_idx != inner.slab_idx || inner.top_z <= outer.bottom_z + EPSILON {
                        continue;
                    }

                    // Interface-over-base creates the need to split. Once a split is needed, use the highest
                    // overlapping inner boundary so the shortened outer cannot partially overlap another inner.
                    // A base-only overlap does not trigger a split by itself; final geometry trimming handles it.
                    let decision = split_candidates.entry(outer_key).or_default();
                    let higher = match decision.highest_inner {
                        None => true,
                        Some(best_key) => {
                            let best = &carriers[best_key.0][&best_key.1];
                            inner.top_z > best.top_z || (inner.top_z == best.top_z && inner.top_layer > best.top_layer)
                        }
                    };
                    // Interface-over-base implies a full-footprint overlap. Once a split is required, a lower inner
                    // cannot change the decision, so skip both polygon tests.
                    if (!higher && decision.required) || !fold_footprints_overlap(&inner.footprint, &inner.footprint_bbox, &outer.footprint, &outer.footprint_bbox) {
                        continue;
                    }
                    decision.required = decision.required
                        || fold_footprints_overlap(&inner.interface_footprint, &inner.footprint_bbox, &outer.base_footprint, &outer.base_footprint_bbox);
                    if higher {
                        decision.highest_inner = Some(inner_key);
                    }
                }
            }

            let mut split_misaligned: BTreeSet<FoldCarrierKey> = BTreeSet::new();
            let mut carriers_shortened = false;
            for (outer_key, decision) in &split_candidates {
                if !decision.required {
                    continue;
                }
                let inner_key = decision.highest_inner.expect("fold_nodes_multi_diameter: a required split has an inner carrier");
                let inner_top_z: CoordF = carriers[inner_key.0][&inner_key.1].top_z;
                let outer = carriers[outer_key.0].get_mut(&outer_key.1).unwrap();

                outer.bottom_z = inner_top_z;
                outer.base_source_layer = outer.top_layer;
                outer.lower_carrier = Some(inner_key);
                outer.slab_height = outer.top_z - outer.bottom_z;
                // std::upper_bound(..., outer.bottom_z + EPSILON, [](coordf_t z, const LayerHeightData &layer) { return z < layer.print_z; })
                let z = outer.bottom_z + EPSILON;
                let first_layer_above = layer_heights[..num_layers].partition_point(|layer| !(z < layer.print_z));
                outer.bottom_obj_layer = if first_layer_above == num_layers { outer.obj_layer } else { layer_heights[first_layer_above].obj_layer_nr };
                carriers_shortened = true;
                if outer.slab_height <= layer_heights[outer.top_layer].height + EPSILON
                    || !crate::adaptive_layer_merge::adaptive_layer_height_range(plan.slabs[outer.slab_idx].nozzle_diameter).contains(outer.slab_height)
                {
                    split_misaligned.insert(*outer_key);
                }
            }

            if carriers_shortened {
                if !split_misaligned.is_empty() {
                    fold_refuse_carriers(&split_misaligned, &mut misaligned_carriers, &column_top, &mut carriers, &self.contact_nodes, nodes, fold_interface);
                }
                continue;
            }

            let mut stationary_footprints: Vec<ExPolygons> = vec![Vec::new(); num_layers];
            let mut stationary_interface_footprints: Vec<ExPolygons> = vec![Vec::new(); num_layers];
            let mut stationary_bboxes: Vec<BBoxCpp> = vec![BBoxCpp::new(); num_layers];
            let mut stationary_ready: Vec<bool> = vec![false; num_layers];

            // No carrier changed after the snapshot above, so overlap refusals now describe the current spans.
            // Same-slab containment was either split at the highest valid boundary or rejected as unprintable.
            let mut newly_misaligned: BTreeSet<FoldCarrierKey> = BTreeSet::new();
            for lhs in 0..folded_spans.len() {
                let lhs_key = folded_spans[lhs];
                let lhs_carrier = &carriers[lhs_key.0][&lhs_key.1];
                if !lhs_carrier.folds || newly_misaligned.contains(&lhs_key) {
                    continue;
                }
                for rhs in lhs + 1..folded_spans.len() {
                    let rhs_key = folded_spans[rhs];
                    let rhs_carrier = &carriers[rhs_key.0][&rhs_key.1];
                    if rhs_carrier.bottom_z >= lhs_carrier.top_z - EPSILON {
                        break;
                    }
                    if !rhs_carrier.folds || newly_misaligned.contains(&rhs_key) {
                        continue;
                    }
                    let lhs_contains_rhs = fold_contains_span(lhs_carrier, rhs_carrier);
                    let rhs_contains_lhs = fold_contains_span(rhs_carrier, lhs_carrier);
                    if !lhs_contains_rhs && !rhs_contains_lhs {
                        if !fold_footprints_overlap(&lhs_carrier.footprint, &lhs_carrier.footprint_bbox, &rhs_carrier.footprint, &rhs_carrier.footprint_bbox) {
                            continue;
                        }
                        let lhs_height: CoordF = lhs_carrier.slab_height;
                        let rhs_height: CoordF = rhs_carrier.slab_height;
                        let rejected = if lhs_height + EPSILON < rhs_height {
                            lhs_key
                        } else if rhs_height + EPSILON < lhs_height {
                            rhs_key
                        } else if lhs_carrier.top_z < rhs_carrier.top_z {
                            lhs_key
                        } else {
                            rhs_key
                        };
                        newly_misaligned.insert(rejected);
                        if rejected == lhs_key {
                            break;
                        }
                        continue;
                    }
                    if lhs_contains_rhs && rhs_contains_lhs {
                        continue;
                    }

                    let (outer_key, outer, inner) = if lhs_contains_rhs { (lhs_key, lhs_carrier, rhs_carrier) } else { (rhs_key, rhs_carrier, lhs_carrier) };
                    if outer.slab_idx != inner.slab_idx
                        && inner.top_z > outer.bottom_z + EPSILON
                        && fold_footprints_overlap(&inner.interface_footprint, &inner.footprint_bbox, &outer.base_footprint, &outer.base_footprint_bbox)
                    {
                        newly_misaligned.insert(outer_key);
                        if outer_key == lhs_key {
                            break;
                        }
                    }
                }
            }

            // Only material left on its original fine layer can conflict with a folded span here. Nodes assigned
            // to another surviving carrier are covered by the carrier-to-carrier check above.
            for &span in &folded_spans {
                let carrier = &carriers[span.0][&span.1];
                if !carrier.folds || carrier.footprint.is_empty() {
                    continue;
                }
                // std::lower_bound(..., carrier.bottom_z + EPSILON, [](const LayerHeightData &layer, coordf_t z) { return layer.print_z < z; })
                let z = carrier.bottom_z + EPSILON;
                let first_layer = layer_heights[..num_layers].partition_point(|layer| layer.print_z < z);
                for layer_nr in first_layer..num_layers {
                    if layer_heights[layer_nr].height < EPSILON {
                        continue;
                    }
                    let layer_top: CoordF = layer_heights[layer_nr].print_z;
                    let layer_bottom: CoordF = layer_top - layer_heights[layer_nr].height;
                    if layer_top <= carrier.bottom_z + EPSILON {
                        continue;
                    }
                    if layer_bottom >= carrier.top_z - EPSILON {
                        break;
                    }
                    let carrier_contains_layer = carrier.bottom_z <= layer_bottom + EPSILON && carrier.top_z >= layer_top - EPSILON;
                    let layer_contains_carrier = layer_bottom <= carrier.bottom_z + EPSILON && layer_top >= carrier.top_z - EPSILON;
                    let partial_z = !carrier_contains_layer && !layer_contains_carrier;

                    // 2447-2463 the lambda `ensure_stationary`
                    if !stationary_ready[layer_nr] {
                        for &node in &self.contact_nodes[layer_nr] {
                            if !tentative_moves.contains_key(&node) {
                                let counters = future_of(node);
                                let probe = prober.cached_probe_of(nodes, node, use_interface_shape(&nodes[node], counters.1));
                                stationary_footprints[layer_nr].extend(probe.iter().cloned());
                                if is_interface_role(counters, layer_nr) {
                                    stationary_interface_footprints[layer_nr].extend(probe.iter().cloned());
                                }
                            }
                        }
                        if !stationary_footprints[layer_nr].is_empty() {
                            stationary_bboxes[layer_nr] = BBoxCpp::of_expolygons(&stationary_footprints[layer_nr]);
                        }
                        stationary_ready[layer_nr] = true;
                    }
                    let stationary = &stationary_footprints[layer_nr];
                    if stationary.is_empty() {
                        continue;
                    }
                    if partial_z {
                        if !fold_footprints_overlap(stationary, &stationary_bboxes[layer_nr], &carrier.footprint, &carrier.footprint_bbox) {
                            continue;
                        }
                        newly_misaligned.insert(span);
                        break;
                    }

                    let mut uncovered_interface_overlap: ExPolygons = Vec::new();
                    if !stationary_interface_footprints[layer_nr].is_empty()
                        && !carrier.base_footprint.is_empty()
                        && stationary_bboxes[layer_nr].overlap(&carrier.base_footprint_bbox)
                    {
                        let overlap: ExPolygons = cx::intersection_ex(&stationary_interface_footprints[layer_nr], &carrier.base_footprint);
                        if !overlap.is_empty() {
                            uncovered_interface_overlap = cx::diff_ex(&overlap, &carrier.interface_footprint);
                        }
                    }
                    if !carrier_contains_layer || !uncovered_interface_overlap.is_empty() {
                        if fold_footprints_overlap(stationary, &stationary_bboxes[layer_nr], &carrier.base_footprint, &carrier.base_footprint_bbox) {
                            newly_misaligned.insert(span);
                            break;
                        }
                        continue;
                    }
                }
            }
            if newly_misaligned.is_empty() {
                moves_to = tentative_moves;
                final_counters = future_counters;
                break;
            }
            fold_refuse_carriers(&newly_misaligned, &mut misaligned_carriers, &column_top, &mut carriers, &self.contact_nodes, nodes, fold_interface);
        }
        // if (misaligned_carriers > 0) BOOST_LOG_TRIVIAL(warning) << "refused " << misaligned_carriers << " folded support carriers with unsafe Z overlap";
        let _ = misaligned_carriers;

        // The second round: of the slabs the first round refused, only the body of the support that runs through
        // the whole slab is folded onto the top of the slab; the interface and the transition layer stay on the
        // fine layers. The roles are those of final_counters of the first round; a fold of a whole slab only ever
        // covers the fine layers of the slab completely, the overlap is trimmed.
        struct BaseFold {
            slab_idx: usize,
            tops: Vec<(NodeId, CoordF)>,
        }
        let mut base_folds: Vec<BaseFold> = Vec::new();
        // The nodes folded into a slab of the second round (the tops of the slabs among them).
        let mut base_folded: HashSet<NodeId> = HashSet::new();
        {
            let nodes = &self.m_ts_data.contact_nodes;
            let counters_of = |node: NodeId| -> (i32, i32) {
                match final_counters.get(&node) {
                    Some(counters) => *counters,
                    None => (nodes[node].distance_to_top, nodes[node].support_roof_layers_below),
                }
            };
            let unmoved = |node: NodeId| -> bool { nodes[node].distance_to_top >= 0 && !moves_to.contains_key(&node) };
            let is_left_base = |node: NodeId, layer_nr: usize| -> bool {
                fold_foldable(&nodes[node], fold_interface) && !moves_to.contains_key(&node) && !is_interface_role(counters_of(node), layer_nr)
            };
            // The fine interface the top of a slab may keep; the transition layer is extruded with the material of
            // the body and may stay on the fine layers only when the interface is of the material of this slab too.
            let stays_roof = |node: NodeId, keep_transition: bool| -> bool {
                let (dist, roof) = counters_of(node);
                let if_layers: i32 = num_top_interface_layers as i32;
                let transition = (roof == 0 || roof == 1) && dist >= if_layers;
                if_layers > 0 && (if transition { keep_transition } else { roof > 0 })
            };
            let mut members: HashSet<NodeId> = HashSet::new();
            let mut fine: HashSet<NodeId> = HashSet::new();
            let mut reaches: HashSet<NodeId> = HashSet::new();
            for slab_idx in 0..num_slabs {
                let layers = &layers_per_slab[slab_idx];
                let slab = &plan.slabs[slab_idx];
                if layers.len() < 2 || (slab.bottom_anchor && m_raft_layers == 0) {
                    continue;
                }
                let (top, bottom) = (layers[layers.len() - 1], layers[0]);
                if let Some(it) = carriers[slab_idx].get(&top) {
                    if it.folds {
                        continue;
                    }
                }
                let slab_height: CoordF = layer_heights[top].print_z - layer_heights[bottom].print_z + layer_heights[bottom].height;
                let keep_transition = explicit_interface_filament == slab.filament_id;
                if !crate::adaptive_layer_merge::adaptive_layer_height_range(slab.nozzle_diameter).contains(slab_height)
                    || self.contact_nodes[top].iter().any(|&node| unmoved(node) && !is_left_base(node, top) && !stays_roof(node, keep_transition))
                {
                    continue;
                }
                // Top down: what is left of the body are the members; one whose parents are not all members that
                // are not fine (under an interface, a new branch inside the slab) is fine and stays on the fine
                // layers to carry the interface.
                members.clear();
                fine.clear();
                for &layer in layers.iter().rev() {
                    for &node in &self.contact_nodes[layer] {
                        if is_left_base(node, layer) {
                            let mut keep = layer != top && nodes[node].parent.is_none() && nodes[node].parents.is_empty();
                            if layer != top {
                                for p in fold_parents(&nodes[node]) {
                                    keep = keep || !members.contains(p) || fine.contains(p);
                                }
                            }
                            if keep {
                                fine.insert(node);
                            }
                            members.insert(node);
                        }
                    }
                }
                if members.is_empty() {
                    continue;
                }
                // Bottom up: only the columns that reach the bottom of the slab through nodes of the body can be
                // folded, or the folded layer would hang in the air or be trimmed away by the interface (R1).
                reaches.clear();
                for &layer_nr in layers {
                    for &node in &self.contact_nodes[layer_nr] {
                        if members.contains(&node) && (layer_nr == bottom || nodes[node].child.map(|c| reaches.contains(&c)).unwrap_or(false)) {
                            reaches.insert(node);
                        }
                    }
                }
                // The nodes of the top that are not of the body were checked above; the body of the top has to
                // reach the bottom of the slab through the body, or the whole slab is given up (R3).
                let mut fold = BaseFold { slab_idx, tops: Vec::new() };
                let mut ok = true;
                for &node in &self.contact_nodes[top] {
                    if !members.contains(&node) {
                        continue;
                    }
                    ok = reaches.contains(&node);
                    if !ok {
                        break;
                    }
                    let mut radius: CoordF = nodes[node].radius;
                    let mut n = nodes[node].child;
                    while let Some(c) = n {
                        if !members.contains(&c) {
                            break;
                        }
                        radius = radius.max(nodes[c].radius);
                        n = nodes[c].child;
                    }
                    fold.tops.push((node, radius));
                }
                if !ok || fold.tops.is_empty() {
                    continue;
                }
                // As in the first round: no fold when the object is in the way of the whole slab and no
                // cross-section would be left.
                let mut span_collision: ExPolygons = Vec::new();
                let mut l = layer_heights[bottom].obj_layer_nr;
                while l < layer_heights[top].obj_layer_nr && l < self.m_ts_data.m_layer_outlines.len() {
                    append(&mut span_collision, cx::offset_ex(&self.m_ts_data.m_layer_outlines[l], xy_distance_scaled as f32, cx::DEFAULT_JOIN_TYPE, cx::DEFAULT_MITER_LIMIT));
                    l += 1;
                }
                if !span_collision.is_empty() {
                    span_collision = cx::union_ex(&span_collision);
                    // `std::none_of(...)`
                    let any = fold.tops.iter().any(|top_node| {
                        let mut probe: ExPolygons = Vec::new();
                        prober.probe_of(&nodes[top_node.0], false, &mut probe, top_node.1);
                        probe.is_empty() || !cx::diff_ex(&probe, &span_collision).is_empty()
                    });
                    if !any {
                        continue;
                    }
                }
                for &node in &reaches {
                    if members.contains(&node) && !fine.contains(&node) {
                        base_folded.insert(node);
                    }
                }
                base_folds.push(fold);
            }
        }

        // A rejected carrier keeps its original fine-grid geometry. Pin that material to the finest candidate
        // that can print the fine layer, otherwise ToolOrdering may select the coarse support nozzle even though
        // no coarse slab was emitted there.
        let mut fine_base_layers: Vec<bool> = vec![false; num_layers];
        let mut fine_interface_layers: Vec<bool> = vec![false; num_layers];
        for slab_idx in 0..num_slabs {
            for &layer_nr in &layers_per_slab[slab_idx] {
                for &node in &self.contact_nodes[layer_nr] {
                    if !column_top[slab_idx].contains_key(&node) || moves_to.contains_key(&node) || base_folded.contains(&node) {
                        continue;
                    }
                    let n = &self.m_ts_data.contact_nodes[node];
                    let counters: (i32, i32) = match final_counters.get(&node) {
                        Some(counters) => *counters,
                        None => (n.distance_to_top, n.support_roof_layers_below),
                    };
                    let interface_node = is_interface_role(counters, layer_nr);
                    if interface_node {
                        fine_interface_layers[layer_nr] = true;
                    } else {
                        fine_base_layers[layer_nr] = true;
                    }
                }
            }
        }
        // 2696-2704 the lambda `finest_candidate_for`
        let finest_candidate_for = |candidates: &[crate::support_merge_plan::Candidate], height: CoordF| -> crate::support_merge_plan::Candidate {
            let mut best = crate::support_merge_plan::Candidate::default();
            for candidate in candidates {
                if candidate.window.contains(height) && (!best.valid() || candidate.nozzle_diameter < best.nozzle_diameter) {
                    best = *candidate;
                }
            }
            best
        };
        for layer_nr in 0..num_layers {
            if !fine_base_layers[layer_nr] && !fine_interface_layers[layer_nr] {
                continue;
            }
            let Some(ts_layer) = self.m_object.m_support_layers.get_mut(layer_nr + m_raft_layers) else {
                continue;
            };
            let layer = &layer_heights[layer_nr];
            if fine_base_layers[layer_nr] {
                let candidate = finest_candidate_for(&plan.candidates_for_slab(layer.print_z, layer.height), layer.height);
                if candidate.valid() {
                    ts_layer.planned_filament_id = candidate.filament_id;
                    ts_layer.planned_nozzle_diameter = candidate.nozzle_diameter;
                }
            }
            if fine_interface_layers[layer_nr] {
                let candidate = plan.finest_interface_for_slab(layer.print_z, layer.height);
                if candidate.valid() {
                    ts_layer.planned_interface_filament_id = candidate.filament_id;
                    ts_layer.planned_interface_nozzle_diameter = candidate.nozzle_diameter;
                }
            }
        }
        if moves_to.is_empty() && base_folds.is_empty() {
            // BOOST_LOG_TRIVIAL(info) << "nothing folded of " << plan.slabs.size() << " slabs";
            return;
        }

        // No floor guard here. The area level merge refused an interface slab whose nozzle could not also
        // extrude a single layer, but only on a layer that actually carried floor areas. Those do not exist yet
        // at this point, and approximating the test from Z alone turns it into "a coarse nozzle may never fold
        // an interface over a fine grid" - which is precisely the configuration this whole path exists for, so
        // it silently disabled the feature. A carrier that ends up with floor areas extrudes them at the layer
        // height through floor_flow, which is the same compromise the unfolded path already makes.

        // Pass 2: apply the counters evaluated before folding while every node is still on its planned layer.
        for (node, counters) in &final_counters {
            let n = &mut self.m_ts_data.contact_nodes[*node];
            n.distance_to_top = counters.0;
            n.support_roof_layers_below = counters.1;
        }

        // Pass 3: fold.
        let mut folded_carriers: usize = 0;
        self.m_fold_slab_of_layer = vec![-1; num_layers];
        for slab_idx in 0..num_slabs {
            let slab = &plan.slabs[slab_idx];
            let layers = &layers_per_slab[slab_idx];

            // Per carrier: the base cross-section it takes, and every interface node of its columns.
            let mut adopted: BTreeMap<usize, (Vec<NodeId>, Vec<NodeId>)> = BTreeMap::new();
            for &layer_nr in layers {
                let mut stay: Vec<NodeId> = Vec::with_capacity(self.contact_nodes[layer_nr].len());
                for &node in &self.contact_nodes[layer_nr] {
                    // Virtual gap nodes, an interface left on the fine grid and the columns whose
                    // carrier refused keep their own layer. Polygon nodes do fold: they render a different shape
                    // but occupy the slab the same way a disk does.
                    let Some(&carrier_layer) = moves_to.get(&node) else {
                        stay.push(node);
                        continue;
                    };
                    // `carriers[slab_idx].at(carrier_layer)`
                    let carrier = carriers[slab_idx].get(&carrier_layer).expect("fold_nodes_multi_diameter: the carrier of a move");
                    let n = &self.m_ts_data.contact_nodes[node];
                    if is_interface_role((n.distance_to_top, n.support_roof_layers_below), carrier_layer) {
                        // Every roof in the column contributes: one extrusion covers all of them.
                        adopted.entry(carrier_layer).or_default().1.push(node);
                    } else if layer_nr == carrier.base_source_layer {
                        // A thick extrusion has one XY cross-section over its whole height. Unioning every layer
                        // would turn lateral branch movement into a swept wedge and distort both the tree shape
                        // and its clearance, so only the representative face is kept.
                        adopted.entry(carrier_layer).or_default().0.push(node);
                    }
                    // Anything else is swallowed by its column's slab. The node stays alive in TreeSupportData,
                    // it simply no longer renders a disk of its own.
                }
                self.contact_nodes[layer_nr] = stay;
            }
            if adopted.is_empty() {
                continue;
            }

            self.m_fold_carriers.push(Vec::new());
            let slab_carriers_idx = self.m_fold_carriers.len() - 1;
            for (carrier_layer, nodes) in &adopted {
                let carrier_layer = *carrier_layer;
                let carrier = carriers[slab_idx].get(&carrier_layer).expect("fold_nodes_multi_diameter: the carrier of a move");
                for (group_idx, group) in [&nodes.0, &nodes.1].into_iter().enumerate() {
                    for &node in group {
                        let n = &mut self.m_ts_data.contact_nodes[node];
                        if group_idx == 0 {
                            // `folded_radius.at(node)`
                            n.radius = folded_radius[&node];
                        }
                        n.slab_height = carrier.slab_height;
                        n.slab_bottom_obj_layer = carrier.bottom_obj_layer;
                        n.obj_layer_nr = carrier.obj_layer as i32;
                        n.print_z = carrier.top_z;
                        n.height = carrier.slab_height;
                        self.contact_nodes[carrier_layer].push(node);
                    }
                }
                self.m_fold_carriers[slab_carriers_idx].push(FoldCarrier { layer: carrier_layer, bottom_z: carrier.bottom_z });
                folded_carriers += 1;

                let Some(ts_layer) = self.m_object.m_support_layers.get_mut(carrier_layer + m_raft_layers) else {
                    continue;
                };
                if !nodes.0.is_empty() {
                    ts_layer.base_slab_height = carrier.slab_height;
                    // Pin the slab to its filament only once it got thicker than the object layers: a slab of base
                    // height can be extruded by any nozzle, and leaving it alone lets the exporter minimize tool
                    // changes.
                    if carrier.slab_height > plan.base_height_at(carrier.top_z) + EPSILON {
                        ts_layer.planned_filament_id = slab.filament_id;
                        ts_layer.planned_nozzle_diameter = slab.nozzle_diameter;
                    }
                }
                if !nodes.1.is_empty() {
                    ts_layer.interface_slab_height = carrier.slab_height;
                    // An explicitly pinned interface filament keeps the tool: the plan only allowed the fold
                    // because that filament can extrude this slab in the first place.
                    ts_layer.planned_interface_filament_id = if explicit_interface_filament > 0 { explicit_interface_filament } else { slab.filament_id };
                    ts_layer.planned_interface_nozzle_diameter = slab.nozzle_diameter;
                }
            }
            for &layer_nr in layers {
                self.m_fold_slab_of_layer[layer_nr] = (self.m_fold_carriers.len() - 1) as i32;
            }
        }
        // Later stages gate their folded-only behaviour on these being non-empty.
        if folded_carriers == 0 {
            self.m_fold_carriers.clear();
            self.m_fold_slab_of_layer.clear();
        }

        // The second round applied: the folded nodes leave the fine layers, the nodes of the top of the slab take
        // the height of the whole slab.
        if !base_folds.is_empty() && self.m_fold_slab_of_layer.is_empty() {
            self.m_fold_slab_of_layer = vec![-1; num_layers];
        }
        for fold in &base_folds {
            let slab = &plan.slabs[fold.slab_idx];
            let layers = &layers_per_slab[fold.slab_idx];
            let top: usize = layers[layers.len() - 1];
            let bottom_z: CoordF = layer_heights[layers[0]].print_z - layer_heights[layers[0]].height;
            let height: CoordF = layer_heights[top].print_z - bottom_z;
            for &layer_nr in layers {
                if layer_nr != top {
                    self.contact_nodes[layer_nr].retain(|node| !base_folded.contains(node));
                }
            }
            for (node, radius) in &fold.tops {
                let n = &mut self.m_ts_data.contact_nodes[*node];
                n.radius = *radius;
                n.slab_height = height;
                n.height = height;
                n.slab_bottom_obj_layer = layer_heights[layers[0]].obj_layer_nr;
            }
            if let Some(ts_layer) = self.m_object.m_support_layers.get_mut(top + m_raft_layers) {
                ts_layer.base_slab_height = height;
                ts_layer.planned_filament_id = slab.filament_id;
                ts_layer.planned_nozzle_diameter = slab.nozzle_diameter;
            }
            if self.m_fold_slab_of_layer[top] < 0 {
                self.m_fold_carriers.push(Vec::new());
                for &layer_nr in layers {
                    self.m_fold_slab_of_layer[layer_nr] = (self.m_fold_carriers.len() - 1) as i32;
                }
            }
            let slab_ref = self.m_fold_slab_of_layer[top] as usize;
            self.m_fold_carriers[slab_ref].push(FoldCarrier { layer: top, bottom_z });
            self.m_base_only_fold_layers.push(top); // the slabs ascend in Z, so the result is in order
        }
        folded_carriers += base_folds.len();

        // Pass 4: printed layer index for the interface pattern. Consecutive printed layers have to alternate,
        // and a folded stack prints several object layers apart, so count the layers that still print.
        if fold_interface && folded_carriers > 0 {
            let mut printed: i32 = 0;
            for layer_nr in 0..num_layers {
                if layer_heights[layer_nr].height < EPSILON || self.contact_nodes[layer_nr].is_empty() {
                    continue;
                }
                for &node in &self.contact_nodes[layer_nr] {
                    let n = &mut self.m_ts_data.contact_nodes[node];
                    if n.distance_to_top >= 0 {
                        n.slab_index = printed;
                    }
                }
                printed += 1;
            }
        }

        // BOOST_LOG_TRIVIAL(info) << "folded " << folded_carriers << " column carriers over " << m_fold_carriers.size() << " of "
        //     << plan.slabs.size() << " slabs, interface fold " << (fold_interface ? "on" : "off") << ", base only " << base_folds.size();
    }

    /// TreeSupport.cpp:2894-2976 `void TreeSupport::smooth_line_width_multi_diameter()`
    ///
    /// BBS multi-diameter: a fine branch section sitting next to a coarse slab only overlaps the coarse wall over
    /// its own narrow bead, so the joint is weak. Widen the wall of the fine side close to every nozzle change,
    /// ramping back to the default width within a short distance. Only base disks that draw_circles() actually
    /// renders take part, a node away from any change keeps line_width == 0, and nothing runs in single-diameter
    /// mode. An exception of the C++ (a flow that cannot be made) is the `Err`.
    pub fn smooth_line_width_multi_diameter(&mut self) -> FlowResult<()> {
        // Lightning roots are trimmed against the default wall, so a wider wall there would overlap them.
        if !self.m_support_merge_plan.enabled || self.m_support_params.base_fill_pattern == InfillPattern::Lightning {
            return Ok(());
        }

        let num_layers: usize = self.contact_nodes.len().min(self.m_ts_data.layer_heights.len());
        let has_roof: bool = self.m_support_params.num_top_interface_layers > 0;
        // Same classification as draw_circles(): what ends up as a plain base disk.
        let is_base_disk = |node: &SupportNode| -> bool {
            !node.fading && node.type_ != TreeNodeType::EPolygon && node.distance_to_top >= 0 && (!has_roof || node.support_roof_layers_below < 0)
        };

        // width: default base width of the node's layer; target: the widened width, raised pairwise below.
        struct Disk {
            node: NodeId,
            width: CoordF,
            top: CoordF,
            bottom: CoordF,
            target: CoordF,
        }
        // `std::unordered_map<SupportNode *, Disk> disks`: the disks in the order of the layers, and where the
        // disk of a node is. The loops below do not depend on the order.
        let mut disks: Vec<Disk> = Vec::new();
        let mut disk_of: HashMap<NodeId, usize> = HashMap::new();
        let mut min_width: CoordF = f64::MAX;
        let mut max_width: CoordF = 0.;
        let mut max_span: CoordF = 0.;
        for layer_nr in 0..num_layers {
            let layer_height: CoordF = self.m_ts_data.layer_heights[layer_nr].height;
            if layer_height < EPSILON || self.contact_nodes[layer_nr].is_empty() {
                continue;
            }
            // Default base width of this layer, resolved the way generate_toolpaths() does.
            let width: CoordF = match self.m_object.get_support_layer((layer_nr + self.m_raft_layers) as i32) {
                Some(ts_layer) if ts_layer.planned_nozzle_diameter > EPSILON => {
                    self.m_support_params.support_material_flow_for_nozzle(ts_layer.planned_nozzle_diameter, layer_height as f32)?.width()
                }
                _ => self.m_support_params.support_extrusion_width,
            };
            for &node in &self.contact_nodes[layer_nr] {
                let n = &self.m_ts_data.contact_nodes[node];
                if !is_base_disk(n) || n.height < EPSILON {
                    continue;
                }
                // `disks.emplace(node, ...)`: the first disk of a node stays
                if !disk_of.contains_key(&node) {
                    disk_of.insert(node, disks.len());
                    disks.push(Disk { node, width, top: n.print_z, bottom: n.print_z - n.height, target: width });
                }
                min_width = min_width.min(width);
                max_width = max_width.max(width);
                max_span = max_span.max(n.height);
            }
        }
        // No nozzle change anywhere: every node keeps its default width.
        if disks.is_empty() || max_width - min_width < EPSILON {
            return Ok(());
        }

        // The finer disk of a pair ramps from its capped width (the coarse width, at most twice its own) at contact
        // back to its own over max(coarse slab, two own layers) plus one own layer, so no partner farther than this
        // matters.
        let reach: CoordF = 3. * max_span;
        // Every node has a single child towards the bed, so walking down from each disk meets every pair of
        // rendered disks on one chain exactly once. Nodes swallowed by a fold are walked through: they only
        // bridge the Z gap between the disks that remain.
        for upper_idx in 0..disks.len() {
            let mut c = self.m_ts_data.contact_nodes[disks[upper_idx].node].child;
            while let Some(child) = c {
                if !(disks[upper_idx].bottom - self.m_ts_data.contact_nodes[child].print_z <= reach) {
                    break;
                }
                c = self.m_ts_data.contact_nodes[child].child;
                let Some(&lower_idx) = disk_of.get(&child) else {
                    continue;
                };
                let (upper, lower) = (&disks[upper_idx], &disks[lower_idx]);
                if (upper.width - lower.width).abs() < EPSILON {
                    continue;
                }
                let (fine_idx, coarse_idx) = if upper.width < lower.width { (upper_idx, lower_idx) } else { (lower_idx, upper_idx) };
                let (fine, coarse) = (&disks[fine_idx], &disks[coarse_idx]);
                // Ramp towards the capped peak rather than the coarse width, so the cap does not flatten the layers
                // next to the change. One own layer is added so the last widened layer does not snap back.
                let fine_h: CoordF = fine.top - fine.bottom;
                let peak: CoordF = coarse.width.min(2. * fine.width);
                let ramp: CoordF = (coarse.top - coarse.bottom).max(2. * fine_h) + fine_h;
                let gap: CoordF = 0f64.max(upper.bottom - lower.top);
                if gap < ramp {
                    // `fine.width + (peak - fine.width) * (1. - gap / ramp)`: the product is contracted with the sum
                    let target = (peak - fine.width).mul_add(1. - gap / ramp, fine.width);
                    disks[fine_idx].target = disks[fine_idx].target.max(target);
                }
            }
        }

        let mut _widened: usize = 0;
        for disk in &disks {
            // The target is already capped at twice the own width by the peak. It is also kept within the disk radius,
            // or the wall inset in tree_supports_generate_paths() would eat a thin branch away.
            let n = &mut self.m_ts_data.contact_nodes[disk.node];
            let width: CoordF = disk.target.min(disk.width.max(n.radius));
            if width > disk.width + EPSILON {
                n.line_width = width;
                _widened += 1;
            }
        }
        // BOOST_LOG_TRIVIAL(info) << "support line width smoothing widened " << widened << " of " << disks.size() << " base nodes, default widths " << min_width << " to " << max_width;
        Ok(())
    }
}

// ============================================================================
// TreeSupport::generate (TreeSupport.cpp:3534-3700)
// ============================================================================

/// `generate_tree_support_3D(PrintObject &print_object, TreeSupport *tree_support, std::function<void()>
/// throw_on_cancel)` (Support/TreeSupport3D.cpp), which TreeSupport::generate() hands the organic style to.
pub trait TreeSupport3DGenerator {
    fn generate_tree_support_3d(&self, tree_support: &mut TreeSupport<'_>) -> crate::Result<()>;
}

impl<'a> TreeSupport<'a> {
    /// `make_support_merge_plan(*m_object)` (SupportMergePlan.cpp:254-403): what the function reads of the object
    /// and of its print, handed to the crate's `make_support_merge_plan` (see `SupportMergePlanObject`).
    fn make_support_merge_plan_of_object(&self) -> SupportMergePlan {
        use crate::support_merge_plan::{make_support_merge_plan, SupportMergePlanLayer, SupportMergePlanObject, SupportMergePlanRegion};
        let print = self.m_print;
        let print_config = print.config();
        // SupportMergePlan.cpp:341-363 `layers[layer_idx]->regions()`, `layers[layer_idx]->empty()`
        let layers: Vec<SupportMergePlanLayer> = self
            .m_object
            .layers()
            .iter()
            .map(|layer| SupportMergePlanLayer {
                print_z: layer.print_z,
                height: layer.height,
                empty: layer.empty(),
                regions: layer
                    .regions()
                    .iter()
                    .map(|layerm| {
                        let slices_empty = layerm.slices.is_empty();
                        if slices_empty {
                            // `if (layerm->slices.empty()) continue;`: nothing else of the region is read
                            return SupportMergePlanRegion { slices_empty, ..Default::default() };
                        }
                        let region_config = layerm.region().config();
                        SupportMergePlanRegion {
                            slices_empty,
                            perimeters_empty: layerm.perimeters.is_empty(),
                            wall_filament: region_config.wall_filament as i32,
                            sparse_infill_filament: region_config.sparse_infill_filament as i32,
                            solid_infill_filament: region_config.solid_infill_filament as i32,
                            fill_filament_slots: layerm
                                .fills
                                .entities
                                .iter()
                                .filter_map(|ee| match ee {
                                    ExtrusionEntityType::Collection(fill) if !fill.is_empty() => {
                                        Some(crate::gcode::tool_ordering::LayerTools::filament_slot_by_role(fill, region_config) as i32)
                                    }
                                    _ => None,
                                })
                                .collect(),
                        }
                    })
                    .collect(),
            })
            .collect();
        // PrintConfig.cpp:454-469 `nozzle_diameter_for_filament(print_config, filament_id)`
        let nozzle_diameter_for_filament = |filament_id: u32| -> f32 { crate::print::faithful::nozzle_diameter_for_filament(print_config, filament_id) };
        // `print_config.filament_soluble.get_at(idx)`
        let soluble: Vec<bool> = match print_config.option("filament_soluble") {
            Some(opt) => match opt.as_any().downcast_ref::<crate::config::ConfigOptionBools>() {
                Some(o) => o.values.iter().map(|v| *v != 0).collect(),
                None => Vec::new(),
            },
            None => Vec::new(),
        };
        let filament_soluble = |idx: usize| -> bool {
            if idx < soluble.len() {
                soluble[idx]
            } else {
                soluble.first().copied().unwrap_or(false)
            }
        };
        // SupportMergePlan.cpp:320-327: the filaments of the model parts of the object
        let filament_count = print_config.option("filament_diameter").and_then(|o| o.as_vector_base()).map(|v| v.size()).unwrap_or(0);
        let global_features = crate::model::GlobalFeatureFilaments::from_config(print.default_region_config(), filament_count);
        let model_object = self.m_object.model_object(print);
        let model_part_extruders: Vec<Vec<i32>> =
            model_object.volumes.iter().filter(|volume| volume.is_model_part()).map(|volume| volume.get_extruders(model_object, &global_features)).collect();
        let object = SupportMergePlanObject {
            uses_multiple_nozzle_diameters: print.uses_multiple_nozzle_diameters(),
            layers,
            grid_nozzle: crate::print_config::config_def::layer_grid_nozzle_diameter(print.full_print_config()),
            base_height: self.m_object.slicing_parameters().layer_height,
            nozzle_diameter_for_filament: &nozzle_diameter_for_filament,
            filament_soluble: &filament_soluble,
            model_part_extruders,
            support_filament: cfg_int(self.object_config(), "support_filament"),
            support_interface_filament: cfg_int(self.object_config(), "support_interface_filament"),
        };
        make_support_merge_plan(&object)
    }

    /// TreeSupport.cpp:3534-3700 `void TreeSupport::generate()`
    ///
    /// `fill_factory` makes the fillers of generate_toolpaths(); `tree_support_3d` is
    /// `generate_tree_support_3D()` for the organic style (see `TreeSupport3DGenerator`). The exceptions of the
    /// C++ (detect_overhangs(), a flow that cannot be made) are the `Err`. `USE_SUPPORT_3D` is 0 at the pin: the
    /// block of TreeSupport.cpp:3561-3610 is not compiled.
    pub fn generate(&mut self, fill_factory: &dyn SupportFillFactory, tree_support_3d: &dyn TreeSupport3DGenerator) -> crate::Result<()> {
        use crate::print_config::config_def::is_tree;
        if !is_tree(support_type_from_i32(cfg_int(self.object_config(), "support_type"))) {
            return Ok(());
        }

        if self.m_support_params.support_style == SupportMaterialStyle::SmsTreeOrganic as i32 {
            return tree_support_3d.generate_tree_support_3d(self);
        }

        profiler().stage_start(TreeSupportStage::StageTotal);

        // Generate overhang areas
        profiler().stage_start(TreeSupportStage::StageDetectOverhangs);
        // `Slic3r::Utils::get_current_milliseconds_time_monotonic()`: a steady clock
        let support_detect_begin_time = std::time::Instant::now();
        self.m_print.base.base.set_status(55, "Generating support", 0, -1);
        self.detect_overhangs(false)?;
        self.m_object.m_support_stage_times.detect += support_detect_begin_time.elapsed().as_millis() as i64;
        profiler().stage_finish(TreeSupportStage::StageDetectOverhangs);

        self.create_tree_support_layers();
        //m_ts_data = m_object->alloc_tree_support_preview_cache();
        self.m_ts_data.is_slim = self.is_slim;

        self.generate_contact_points();
        profiler().stage_finish(TreeSupportStage::StageGenerateContactNodes);

        self.m_ts_data.layer_heights = self.plan_layer_heights();

        // The tree geometry is generated on the fine support-node grid, while a coarse support nozzle prints
        // several of those nodes as one fixed-grid slab. Cache the plan before dropping nodes so per-node growth
        // can be normalized to the slab that will eventually be emitted.
        self.m_support_merge_plan = self.make_support_merge_plan_of_object();

        //Drop nodes to lower layers.
        profiler().stage_start(TreeSupportStage::StageDropDownNodes);
        self.m_print.base.base.set_status(60, "Generating support", 0, -1);
        self.drop_nodes();
        profiler().stage_finish(TreeSupportStage::StageDropDownNodes);
        // log_zigzag_stats("zigzag"): a line of the debug log, nothing else (TreeSupport.cpp:3628-3662)

        self.smooth_nodes();

        // log_zigzag_stats("zigzag_post")

        // BBS multi-diameter: fold the node chains onto the slab grid a coarse nozzle can extrude. This has to
        // follow smoothing, which needs the fine grid samples, and precede everything that turns nodes into
        // geometry, so the floating support prune and the disks all see the folded tree.
        self.fold_nodes_multi_diameter();
        self.smooth_line_width_multi_diameter().map_err(|e| crate::Error::Slicing(e.to_string()))?;

        //Generate support areas.
        profiler().stage_start(TreeSupportStage::StageDrawCircles);
        self.m_print.base.base.set_status(65, "Generating support", 0, -1);
        let has_support_interface: bool =
            self.m_support_params.has_interfaces() || self.m_support_params.has_base_interfaces() || self.m_slicing_params.interface_raft_layers > 0;
        let support_area_begin_time = std::time::Instant::now();
        self.draw_circles();
        if has_support_interface {
            // The classic tree path creates base and interface areas in one pass, so this is the
            // narrowest wall-clock stage that contains its interface-layer generation.
            self.m_object.m_support_stage_times.interface_generate += support_area_begin_time.elapsed().as_millis() as i64;
        }
        profiler().stage_finish(TreeSupportStage::StageDrawCircles);

        profiler().stage_start(TreeSupportStage::StageGenerateToolpaths);
        let support_toolpath_begin_time = std::time::Instant::now();
        self.m_print.base.base.set_status(70, "Generating support", 0, -1);
        self.generate_toolpaths(fill_factory).map_err(|e| crate::Error::Slicing(e.to_string()))?;
        self.m_object.m_support_stage_times.toolpath_generate += support_toolpath_begin_time.elapsed().as_millis() as i64;
        profiler().stage_finish(TreeSupportStage::StageGenerateToolpaths);

        profiler().stage_finish(TreeSupportStage::StageTotal);
        // BOOST_LOG_TRIVIAL(info) << "tree support time " << profiler.report();
        Ok(())
    }
}

// @@TAIL@@
// Keep the unused-helper warnings quiet for the not-yet-wired constants/helpers.
#[allow(dead_code)]
fn _keep_unused() {
    let _ = (sign(0.0), TAU, USE_SUPPORT_3D, NO_INDEX, sq(0.0));
    let _ = (
        MAX_BRANCH_RADIUS_FIRST_LAYER,
        MIN_BRANCH_RADIUS_FIRST_LAYER,
        M_PI,
    );
    let _ = SupportNode::NO_PARENT;
}

