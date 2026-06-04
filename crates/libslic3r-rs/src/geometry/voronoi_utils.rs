//! Voronoi diagram utility functions.
//!
//! C++ Reference:
//! - Geometry/VoronoiUtils.hpp
//! - Geometry/VoronoiUtils.cpp
//!
//! Provides utility functions for working with Voronoi diagrams in the context
//! of the Arachne variable-width algorithm.

use crate::geometry::{Line, Point};
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
    /// Geometry/VoronoiUtils.hpp: to_point
    pub fn to_point(x: f64, y: f64) -> Point {
        Point::new(x.round() as i64, y.round() as i64)
    }

    /// Check if a Voronoi vertex has finite coordinates.
    ///
    /// Geometry/VoronoiUtils.hpp: is_finite
    pub fn is_finite(x: f64, y: f64) -> bool {
        x.is_finite() && y.is_finite()
    }

    /// Create a rotated copy of a vertex position.
    ///
    /// Geometry/VoronoiUtils.hpp: make_rotated_vertex
    pub fn make_rotated_vertex(x: f64, y: f64, angle: f64) -> (f64, f64) {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        (x * cos_a - y * sin_a, x * sin_a + y * cos_a)
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
    let mut discretized: Vec<Point> = Vec::new();
    // x is the distance of a point projected onto the segment ab; pxx is the projection.
    let a = seg_a;
    let b = seg_b;
    let ab = b - a;
    let as_ = start - a;
    let ae = end - a;
    let dot = |p: Point, q: Point| -> i128 {
        p.x as i128 * q.x as i128 + p.y as i128 * q.y as i128
    };
    let isqrt = |p: Point| -> i64 {
        ((p.x as i128 * p.x as i128 + p.y as i128 * p.y as i128) as f64).sqrt() as i64
    };
    let ab_size = isqrt(ab); // ab.cast<int64_t>().norm()
    if ab_size == 0 {
        discretized.push(start);
        discretized.push(end);
        return discretized;
    }
    let sx = (dot(as_, ab) / ab_size as i128) as i64;
    let ex = (dot(ae, ab) / ab_size as i128) as i64;
    let sxex = ex - sx;

    let ap = source_point - a;
    let px = (dot(ap, ab) / ab_size as i128) as i64;

    // pxx = foot of perpendicular of source_point on the infinite line a-b.
    let pxx = Line::new(a, b).project_point_infinite(&source_point);
    let ppxx = pxx - source_point;
    let d = isqrt(ppxx);

    if d == 0 {
        discretized.push(start);
        discretized.push(end);
        return discretized;
    }

    // rot = perp(ppxx).normalized(); perp(p) = (-p.y, p.x) (Point.hpp).
    let perp_x = -ppxx.y as f64;
    let perp_y = ppxx.x as f64;
    let rot_len = perp_x.hypot(perp_y);
    let rot_cos_theta = perp_x / rot_len;
    let rot_sin_theta = perp_y / rot_len;

    let marking_bound = (transitioning_angle as f64 * 0.5).atan();
    let mut msx = (-marking_bound * d as f64) as i64; // projected marking_start
    let mut mex = (marking_bound * d as f64) as i64; // projected marking_end

    let marking_start_end_h = msx * msx / (2 * d) + d / 2;
    let mut marking_start = Point::new(msx as Coord, marking_start_end_h as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    let mut marking_end = Point::new(mex as Coord, marking_start_end_h as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    let dir: i64 = if sx > ex { -1 } else { 1 };
    if dir < 0 {
        std::mem::swap(&mut marking_start, &mut marking_end);
        std::mem::swap(&mut msx, &mut mex);
    }

    let mut add_marking_start = msx * dir > (sx - px) * dir && msx * dir < (ex - px) * dir;
    let mut add_marking_end = mex * dir > (sx - px) * dir && mex * dir < (ex - px) * dir;

    let apex = Point::new(0, (d / 2) as Coord)
        .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
        + pxx;
    let mut add_apex = (sx - px) * dir < 0 && (ex - px) * dir > 0;

    let step_count = ((ex - sx).abs() as f64 / approximate_step_size as f64).round() as i64;
    discretized.push(start);
    let mut step: i64 = 1;
    while step < step_count {
        let x = sx + sxex * step / step_count - px;
        let y = x * x / (2 * d) + d / 2;

        if add_marking_start && msx * dir < x * dir {
            discretized.push(marking_start);
            add_marking_start = false;
        }
        if add_apex && x * dir > 0 {
            discretized.push(apex);
            add_apex = false; // only add the apex just before crossing it
        }
        if add_marking_end && mex * dir < x * dir {
            discretized.push(marking_end);
            add_marking_end = false;
        }
        let result = Point::new(x as Coord, y as Coord)
            .rotate_by_cos_sin(rot_cos_theta, rot_sin_theta)
            + pxx;
        discretized.push(result);
        step += 1;
    }

    if add_apex {
        discretized.push(apex);
    }
    if add_marking_end {
        discretized.push(marking_end);
    }
    discretized.push(end);
    discretized
}
