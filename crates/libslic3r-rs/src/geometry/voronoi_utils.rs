//! Voronoi diagram utility functions.
//!
//! C++ Reference:
//! - Geometry/VoronoiUtils.hpp
//! - Geometry/VoronoiUtils.cpp
//!
//! Provides utility functions for working with Voronoi diagrams in the context
//! of the Arachne variable-width algorithm.
//!
//! Scope note: the C++ `VoronoiUtils` also defines the boost.polygon-typed
//! templated members `get_source_segment` (VoronoiUtils.cpp:40), `get_source_point`
//! (VoronoiUtils.cpp:56), `get_source_point_index` (VoronoiUtils.cpp:83),
//! `compute_segment_cell_range` (VoronoiUtils.cpp:205) and the `is_in_range`
//! overloads (VoronoiUtils.hpp:99-115). Those operate on `VD::cell_type`/
//! `VD::edge_type` (boost.polygon) handles, which in this crate are provided by the
//! `boostvoronoi` (`bv`) types. Faithful ports of `get_source_segment`,
//! `get_source_point` and `compute_segment_cell_range` against `bv::Cell`/
//! `bv::Diagram` already live in `voronoi_utils_cgal.rs` (the only consumer,
//! `get_parabolic_segment`). This module ports the coordinate-level helpers
//! (`to_point`, `is_finite`, `make_rotated_vertex`) and `discretize_parabola`,
//! which is the only member that does not need a live Voronoi cell handle.

use crate::geometry::{perp, Point};
use crate::Coord;

/// Represents the range of edges around a trapezoid-shaped Voronoi cell
/// that belongs to a line segment source.
///
/// Geometry/VoronoiUtils.hpp: SegmentCellRange
#[derive(Debug, Clone)]
pub struct SegmentCellRange {
    /// The start point of the source segment of this cell.
    pub segment_start_point: Point,
    /// The end point of the source segment of this cell.
    pub segment_end_point: Point,
    /// Index of the edge where the loop around the cell starts (None if invalid).
    pub edge_begin: Option<usize>,
    /// Index of the edge where the loop around the cell ends (None if invalid).
    pub edge_end: Option<usize>,
}

impl SegmentCellRange {
    /// Create a new SegmentCellRange for the given segment endpoints.
    pub fn new(segment_start_point: Point, segment_end_point: Point) -> Self {
        Self {
            segment_start_point,
            segment_end_point,
            edge_begin: None,
            edge_end: None,
        }
    }

    /// Check if the cell range is valid (both edges set and different).
    pub fn is_valid(&self) -> bool {
        match (self.edge_begin, self.edge_end) {
            (Some(begin), Some(end)) => begin != end,
            _ => false,
        }
    }
}

/// Utility functions for working with Voronoi diagrams.
///
/// Geometry/VoronoiUtils.hpp: VoronoiUtils
pub struct VoronoiUtils;

impl VoronoiUtils {
    /// Convert a Voronoi vertex to an integer Point by rounding coordinates.
    ///
    /// VoronoiUtils.cpp:255 `VoronoiUtils::to_point(const VD::vertex_type &vertex)`.
    pub fn to_point(x: f64, y: f64) -> Point {
        // VoronoiUtils.cpp:259 assert(std::isfinite(x) && std::isfinite(y));
        debug_assert!(x.is_finite() && y.is_finite());
        // VoronoiUtils.cpp:262 return {std::llround(x), std::llround(y)};
        // f64::round() rounds half away from zero, matching std::llround.
        Point::new(x.round() as i64, y.round() as i64)
    }

    /// Check if a Voronoi vertex has finite coordinates.
    ///
    /// VoronoiUtils.cpp:265 `VoronoiUtils::is_finite`.
    pub fn is_finite(x: f64, y: f64) -> bool {
        // VoronoiUtils.cpp:267 return std::isfinite(vertex.x()) && std::isfinite(vertex.y());
        x.is_finite() && y.is_finite()
    }

    /// Create a rotated copy of a vertex position.
    ///
    /// VoronoiUtils.cpp:270 `VoronoiUtils::make_rotated_vertex`.
    /// Returns the rotated `(x, y)`; the C++ also copies `incident_edge`/`color`,
    /// which are boost.polygon vertex fields without a coordinate-level analogue.
    pub fn make_rotated_vertex(x: f64, y: f64, angle: f64) -> (f64, f64) {
        // VoronoiUtils.cpp:272-273
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        // VoronoiUtils.cpp:275 const double rotated_x = (cos_a * vertex.x() - sin_a * vertex.y());
        let rotated_x = cos_a * x - sin_a * y;
        // VoronoiUtils.cpp:276 const double rotated_y = (cos_a * vertex.y() + sin_a * vertex.x());
        let rotated_y = cos_a * y + sin_a * x;
        (rotated_x, rotated_y)
    }
}

/// Convert Voronoi vertex coordinates to a Point by rounding.
///
/// Geometry/VoronoiUtils.hpp: to_point (free function form)
pub fn to_point(x: f64, y: f64) -> Point {
    VoronoiUtils::to_point(x, y)
}

/// Create a rotated vertex from the given coordinates and angle.
///
/// Geometry/VoronoiUtils.hpp: make_rotated_vertex (free function form)
pub fn make_rotated_vertex(x: f64, y: f64, angle: f64) -> (f64, f64) {
    VoronoiUtils::make_rotated_vertex(x, y, angle)
}

/// Check if coordinates are finite.
///
/// Geometry/VoronoiUtils.hpp: is_finite (free function form)
pub fn is_finite(x: f64, y: f64) -> bool {
    VoronoiUtils::is_finite(x, y)
}

/// Discretize the parabolic Voronoi edge equidistant from a point-site `source_point`
/// and a segment-site `seg_a`→`seg_b`, between `start` and `end`, into a polyline.
/// VoronoiUtils.cpp:107 `VoronoiUtils::discretize_parabola`.
pub fn discretize_parabola(
    source_point: Point,
    seg_a: Point,
    seg_b: Point,
    start: Point,
    end: Point,
    approximate_step_size: Coord,
    transitioning_angle: f32,
) -> Vec<Point> {
    // VoronoiUtils.cpp:109 Points discretized;
    let mut discretized: Vec<Point> = Vec::new();
    // VoronoiUtils.cpp:110-111
    // x is distance of point projected on the segment ab
    // xx is point projected on the segment ab
    // VoronoiUtils.cpp:112-113 const Point a = source_segment.from(); const Point b = source_segment.to();
    let a = seg_a;
    let b = seg_b;
    // VoronoiUtils.cpp:114-116
    let ab = b - a;
    let as_ = start - a;
    let ae = end - a;

    // C++ casts the integer Point vectors to int64_t and uses Eigen .dot()/.norm().
    // Point::dot already returns i128 (no overflow), and .norm() is the truncated
    // integer sqrt of the squared length (Eigen casts the double sqrt back to the
    // scalar type, truncating toward zero).
    let isqrt = |p: Point| -> i64 {
        ((p.x as i128 * p.x as i128 + p.y as i128 * p.y as i128) as f64).sqrt() as i64
    };
    // VoronoiUtils.cpp:117 const coord_t ab_size = ab.cast<int64_t>().norm();
    // FIDELITY-NOTE(F2): coord_t is int32 in C++; the crate's Coord is i64, so this
    // keeps the i64 width rather than truncating to int32 (crate-wide decision).
    let ab_size = isqrt(ab);
    // VoronoiUtils.cpp:118 const coord_t sx = as.cast<int64_t>().dot(ab.cast<int64_t>()) / ab_size;
    let sx = (as_.dot(&ab) / ab_size as i128) as i64;
    // VoronoiUtils.cpp:119 const coord_t ex = ae.cast<int64_t>().dot(ab.cast<int64_t>()) / ab_size;
    let ex = (ae.dot(&ab) / ab_size as i128) as i64;
    // VoronoiUtils.cpp:120 const coord_t sxex = ex - sx;
    let sxex = ex - sx;

    // VoronoiUtils.cpp:122 const Point ap = source_point - a;
    let ap = source_point - a;
    // VoronoiUtils.cpp:123 const coord_t px = ap.cast<int64_t>().dot(ab.cast<int64_t>()) / ab_size;
    let px = (ap.dot(&ab) / ab_size as i128) as i64;

    // VoronoiUtils.cpp:125-126
    // Point pxx; Line(a, b).distance_to_infinite_squared(source_point, &pxx);
    // The closest point on the infinite line a-b is a + t*(b-a) with t = (ap.v)/|v|^2,
    // cast back to integer by truncation toward zero (Eigen .cast<coord_t>()), NOT
    // rounding. Computed inline here to match C++ exactly (project_point_infinite
    // rounds instead, so it is not equivalent).
    let pxx = {
        let v = ab; // (b - a)
        let l2 = (v.x as i128 * v.x as i128 + v.y as i128 * v.y as i128) as f64;
        if l2 == 0.0 {
            a // a == b case: closest point is a (Line.hpp:95)
        } else {
            let t = ap.dot(&v) as f64 / l2;
            Point::new(
                (a.x as f64 + t * v.x as f64) as Coord,
                (a.y as f64 + t * v.y as f64) as Coord,
            )
        }
    };
    // VoronoiUtils.cpp:127-128 const Point ppxx = pxx - source_point; const coord_t d = ppxx.cast<int64_t>().norm();
    let ppxx = pxx - source_point;
    let d = isqrt(ppxx);

    // VoronoiUtils.cpp:130-132 const Vec2d rot = perp(ppxx).cast<double>().normalized();
    // perp(p) = (-p.y, p.x) (Point.hpp:99).
    let perp_ppxx = perp(ppxx);
    let perp_x = perp_ppxx.x as f64;
    let perp_y = perp_ppxx.y as f64;
    let rot_len = perp_x.hypot(perp_y);
    // VoronoiUtils.cpp:131-132 const double rot_cos_theta = rot.x(); const double rot_sin_theta = rot.y();
    let rot_cos_theta = perp_x / rot_len;
    let rot_sin_theta = perp_y / rot_len;

    // VoronoiUtils.cpp:134-138
    if d == 0 {
        discretized.push(start);
        discretized.push(end);
        return discretized;
    }

    // VoronoiUtils.cpp:140 const double marking_bound = atan(transitioning_angle * 0.5);
    let marking_bound = (transitioning_angle as f64 * 0.5).atan();
    // VoronoiUtils.cpp:141 int64_t msx = -marking_bound * int64_t(d); // projected marking_start
    let mut msx = (-marking_bound * d as f64) as i64;
    // VoronoiUtils.cpp:142 int64_t mex = marking_bound * int64_t(d);  // projected marking_end
    let mut mex = (marking_bound * d as f64) as i64;

    // VoronoiUtils.cpp:144 const coord_t marking_start_end_h = msx * msx / (2 * d) + d / 2;
    let marking_start_end_h = msx * msx / (2 * d) + d / 2;
    // VoronoiUtils.cpp:145 Point marking_start = Point(coord_t(msx), marking_start_end_h).rotated(rot_cos_theta, rot_sin_theta) + pxx;
    let mut marking_start = Point::new(msx as Coord, marking_start_end_h as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    // VoronoiUtils.cpp:146 Point marking_end = Point(coord_t(mex), marking_start_end_h).rotated(rot_cos_theta, rot_sin_theta) + pxx;
    let mut marking_end = Point::new(mex as Coord, marking_start_end_h as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    // VoronoiUtils.cpp:147 const int dir = (sx > ex) ? -1 : 1;
    let dir: i64 = if sx > ex { -1 } else { 1 };
    // VoronoiUtils.cpp:148-151
    if dir < 0 {
        std::mem::swap(&mut marking_start, &mut marking_end);
        std::mem::swap(&mut msx, &mut mex);
    }

    // VoronoiUtils.cpp:153
    let mut add_marking_start = msx * dir > (sx - px) * dir && msx * dir < (ex - px) * dir;
    // VoronoiUtils.cpp:154
    let mut add_marking_end = mex * dir > (sx - px) * dir && mex * dir < (ex - px) * dir;

    // VoronoiUtils.cpp:156 const Point apex = Point(0, d / 2).rotated(rot_cos_theta, rot_sin_theta) + pxx;
    let apex = Point::new(0, (d / 2) as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    // VoronoiUtils.cpp:157 bool add_apex = int64_t(sx - px) * int64_t(dir) < 0 && int64_t(ex - px) * int64_t(dir) > 0;
    let mut add_apex = (sx - px) * dir < 0 && (ex - px) * dir > 0;

    // VoronoiUtils.cpp:159-161 assert + warning when discretization cannot place an apex/endpoint.
    debug_assert!(!add_marking_start || !add_marking_end || add_apex);
    if add_marking_start && add_marking_end && !add_apex {
        log::warn!("Failing to discretize parabola! Must add an apex or one of the endpoints.");
    }

    // VoronoiUtils.cpp:163 const coord_t step_count = lround(std::abs(ex - sx) / approximate_step_size);
    // lround rounds half away from zero, matching f64::round().
    let step_count = ((ex - sx).abs() as f64 / approximate_step_size as f64).round() as i64;
    // VoronoiUtils.cpp:164 discretized.emplace_back(start);
    discretized.push(start);
    // VoronoiUtils.cpp:165 for (coord_t step = 1; step < step_count; ++step)
    let mut step: i64 = 1;
    while step < step_count {
        // VoronoiUtils.cpp:166 const int64_t x = int64_t(sx) + int64_t(sxex) * int64_t(step) / int64_t(step_count) - int64_t(px);
        let x = sx + sxex * step / step_count - px;
        // VoronoiUtils.cpp:167 const int64_t y = int64_t(x) * int64_t(x) / int64_t(2 * d) + int64_t(d / 2);
        let y = x * x / (2 * d) + d / 2;

        // VoronoiUtils.cpp:169-172
        if add_marking_start && msx * dir < x * dir {
            discretized.push(marking_start);
            add_marking_start = false;
        }
        // VoronoiUtils.cpp:174-177
        if add_apex && x * dir > 0 {
            discretized.push(apex);
            add_apex = false; // only add the apex just before the
        }
        // VoronoiUtils.cpp:179-182
        if add_marking_end && mex * dir < x * dir {
            discretized.push(marking_end);
            add_marking_end = false;
        }
        // VoronoiUtils.cpp:185-186 const Point result = Point(x, y).rotated(...) + pxx; discretized.emplace_back(result);
        let result = Point::new(x as Coord, y as Coord)
            .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
            + pxx;
        discretized.push(result);
        step += 1;
    }

    // VoronoiUtils.cpp:189-190
    if add_apex {
        discretized.push(apex);
    }
    // VoronoiUtils.cpp:192-193
    if add_marking_end {
        discretized.push(marking_end);
    }
    // VoronoiUtils.cpp:195-196 discretized.emplace_back(end); return discretized;
    discretized.push(end);
    discretized
}
