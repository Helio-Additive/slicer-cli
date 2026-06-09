//! Geometry primitives for the slicer.
//!
//! This module provides the fundamental geometric types used throughout the slicing pipeline:
//! - [`Point`] and [`Point3`] - 2D and 3D points with integer coordinates (scaled)
//! - [`PointF`] and [`Point3F`] - 2D and 3D points with floating-point coordinates (unscaled)
//! - [`Line`] - Line segment between two points
//! - [`Polygon`] - Closed polygon (boundary)
//! - [`Polyline`] - Open polyline (path)
//! - [`ExPolygon`] - Polygon with holes (exterior + interior contours)
//! - [`BoundingBox`] and [`BoundingBox3`] - Axis-aligned bounding boxes
//!
//! ## Coordinate System
//!
//! The slicer uses scaled integer coordinates internally to avoid floating-point precision issues.
//! Coordinates are scaled by `SCALING_FACTOR` (1,000,000), so 1 unit = 1 nanometer.
//!
//! - Use `scale()` / `scaled()` to convert from mm to internal units
//! - Use `unscale()` / `unscaled()` to convert from internal units to mm

pub mod aabb_mesh;
pub mod aabb_tree;
pub mod bicubic;
mod bounding_box;
mod build_volume;
mod circle;
mod convex_hull;
pub mod curves;
pub mod elephant_foot;
mod expolygon;
// Geometry.cpp / Geometry.hpp — Transformation family + header math helpers.
pub mod geometry;
mod line;
mod medial_axis;
mod point;
mod polygon;
mod polyline;
pub mod simplify;
mod thick_polyline;
mod transform;
mod voronoi;
pub mod voronoi_annotation;
pub mod voronoi_diagram;
pub mod voronoi_offset;
pub mod voronoi_utils;
pub mod voronoi_utils_cgal;
pub mod voronoi_visual_utils;

pub use aabb_mesh::{AABBMesh, HitResult};
pub use bounding_box::{BoundingBox, BoundingBox3, BoundingBox3F, BoundingBoxF};
pub use build_volume::BuildVolume;
pub use circle::{
    circle_center, circle_center_taubin_newton, circle_center_taubin_newton_points, circle_ransac,
    circle_taubin_newton, ray_circle_intersections, ray_circle_intersections_r2_lv2_c,
    smallest_enclosing_circle2_welzl, smallest_enclosing_circle_welzl,
    smallest_enclosing_circle_welzl_eps, Circle, CircleSq,
};
pub use convex_hull::{
    convex_hull_3d, convex_hull_expolygons, convex_hull_points, convex_hull_polygons,
    convex_hull_polylines, decompose_convex_polygon_top_bottom, inside_convex_polygon,
};
pub use elephant_foot::{
    calculate_compensation, compensate_expolygon, compensate_expolygons, compensate_polygon,
    elephant_foot_spacing, scaled_elephant_foot_spacing, ElephantFootCompensator,
    ElephantFootConfig,
};
pub use expolygon::{
    area_expolygons, count_points, count_points_expoly, expolygons_append, expolygons_contain,
    expolygons_rotate, expolygons_simplify, get_extents, get_extents_expoly, get_extents_vector,
    keep_largest_contour_only, number_polygons, overlaps_expoly, overlaps_expolygons,
    polygons_append, polygons_append_expoly, remove_same_neighbor, to_expolygons_simple, to_lines,
    to_lines_expoly, to_points, to_polygons, to_polygons_expoly, to_polylines, to_polylines_expoly,
    translate_expolygons, ExPolygon, ExPolygons,
};
// Geometry.cpp / Geometry.hpp — Transformation family + transform helpers.
// NOTE: `geometry::geometry::{Vec3d, Vec2d, Orientation}` are intentionally NOT
// re-exported here to avoid clobbering the existing `geometry::Vec3d` (aabb
// `Vec3`) and `geometry::Orientation` (Point.hpp). Use the fully-qualified path
// `crate::geometry::geometry::*` for those.
pub use geometry::{
    angle_to_0_2pi, arrange, assemble_transform, assemble_transform_into, contains, deg2rad,
    extract_euler_angles, extract_euler_angles_from_matrix, extract_rotation, generate_transform,
    is_rotation_ninety_degrees, is_rotation_ninety_degrees_angle, mat_around_a_point_rotate,
    rad2deg, rotation_diff_z, rotation_from_two_vectors, rotation_transform, rotation_xyz_diff,
    scale_transform, scale_transform_into, scale_transform_uniform, scale_transform_uniform_into,
    to_range_pi_pi, transform3d_from_string, translation_transform, Axis as TransformAxis,
    Transformation, TransformationSVD,
};
pub use line::{Line, LineF, Lines};
pub use medial_axis::{
    compute_medial_axis, compute_medial_axis_multi, compute_medial_axis_thick,
    distance_to_boundary, MedialAxisConfig,
};
pub use point::{
    collect_duplicates, has_duplicate_points, shorter_then, Point, Point3, Point3F, PointF, Points,
    Points3,
};
pub use polygon::{Polygon, Polygons};
// Polygon.cpp free functions. Names that would collide with the ExPolygon-variant
// re-exports (get_extents, count_points, to_lines, to_points, to_polylines,
// to_polygons, polygons_append, remove_same_neighbor, get_extents_vector, area,
// contains) are intentionally NOT glob-re-exported; reach them via
// `crate::geometry::polygon::*`.
pub use polygon::{
    area_polygons, contains_polygon, contains_polygons, get_extents_polygons,
    get_extents_rotated, get_extents_rotated_polygons, has_duplicate_points as has_duplicate_points_polygons,
    make_circle, make_circle_num_segments, overlaps as overlaps_polygons, polygon_is_convex,
    polygon_is_convex_poly, polygons_match, polygons_reverse, polygons_rotate, polygons_simplify,
    remove_collinear, remove_collinear_polygons, remove_degenerate,
    remove_same_neighbor_polygons, remove_small as remove_small_polygons, remove_sticks,
    remove_sticks_polygons, total_length,
};
pub use polyline::{foot_pt, Polyline, Polylines};
pub use simplify::{
    douglas_peucker, douglas_peucker_polygon, douglas_peucker_polyline, remove_collinear_points,
    remove_duplicate_points, simplify_comprehensive, simplify_polygon,
    simplify_polygon_comprehensive, simplify_polygons, simplify_polyline,
    simplify_polyline_comprehensive, simplify_polylines, simplify_resolution, SimplifyConfig,
    COLLINEARITY_THRESHOLD, MESHFIX_MAXIMUM_DEVIATION, MESHFIX_MAXIMUM_EXTRUSION_AREA_DEVIATION,
    MESHFIX_MAXIMUM_RESOLUTION, MINIMUM_SEGMENT_LENGTH,
};
pub use thick_polyline::{ThickLine, ThickLines, ThickPolyline, ThickPolylines};
pub use transform::{Transform2D, Transform3D};
pub use voronoi::VoronoiDiagram;

// Re-export AABB tree types
pub use aabb_tree::{
    closest_point_on_triangle, ray_box_intersect, ray_triangle_intersect, AABBClosestPointResult,
    AABBNode, AABBTree, IndexedTriangleSet, RayHit, Vec3, AABB3,
};

// Re-export core coordinate types from crate root
pub use crate::{Coord, CoordF};

/// Type alias for 2D floating-point vector (compatible with BambuStudio Vec2d)
/// Point.hpp
pub type Vec2d = PointF;

/// Type alias for 3D floating-point vector (compatible with BambuStudio Vec3d)
/// Point.hpp
pub type Vec3d = Vec3;

#[inline]
/// Calculate the cross product of two 2D vectors (returns a scalar)
/// Point.hpp:147-150
pub fn cross2(v1: Point, v2: Point) -> i128 {
    // Point.hpp:148
    v1.x as i128 * v2.y as i128 - v1.y as i128 * v2.x as i128
}

#[inline]
/// Calculate the cross product of two 2D vectors (floating-point version)
/// Point.hpp:147-150
pub fn cross2f(v1: PointF, v2: PointF) -> CoordF {
    // Point.hpp:148
    v1.x * v2.y - v1.y * v2.x
}

/// Returns true if the two line segments (ip1,ip2) and (jp1,jp2) intersect,
/// including the collinear-overlap case.
/// Geometry.hpp:117-167
pub fn segments_intersect(ip1: Point, ip2: Point, jp1: Point, jp2: Point) -> bool {
    // Geometry.hpp:121-122
    //assert(ip1 != ip2);
    //assert(jp1 != jp2);

    // Geometry.hpp:124-137
    let segments_could_intersect = |ip1: Point, ip2: Point, jp1: Point, jp2: Point| -> (i32, i32) {
        // Geometry.hpp:128-130
        let iv = ip2 - ip1;
        let vij1 = jp1 - ip1;
        let vij2 = jp2 - ip1;
        // Geometry.hpp:131-132
        let tij1 = cross2(iv, vij1);
        let tij2 = cross2(iv, vij2);
        // Geometry.hpp:133-136 — signum
        (
            if tij1 > 0 {
                1
            } else if tij1 < 0 {
                -1
            } else {
                0
            },
            if tij2 > 0 {
                1
            } else if tij2 < 0 {
                -1
            } else {
                0
            },
        )
    };

    // Geometry.hpp:139-142
    let sign1 = segments_could_intersect(ip1, ip2, jp1, jp2);
    let sign2 = segments_could_intersect(jp1, jp2, ip1, ip2);
    let test1 = sign1.0 * sign1.1;
    let test2 = sign2.0 * sign2.1;
    // Geometry.hpp:143
    if test1 <= 0 && test2 <= 0 {
        // The segments possibly intersect. They may also be collinear, but not intersect.
        // Geometry.hpp:145-147
        if test1 != 0 || test2 != 0 {
            // Certainly not collinear, then the segments intersect.
            return true;
        }
        // If the first segment is collinear with the other, the other is collinear with the first segment.
        // Geometry.hpp:149
        debug_assert!((sign1.0 == 0 && sign1.1 == 0) == (sign2.0 == 0 && sign2.1 == 0));
        // Geometry.hpp:150
        if sign1.0 == 0 && sign1.1 == 0 {
            // The segments are certainly collinear. Now verify whether they overlap.
            // Geometry.hpp:152
            let vi = ip2 - ip1;
            // Project both on the longer coordinate of vi.
            // Geometry.hpp:154
            let axis = if vi.x.abs() > vi.y.abs() { 0 } else { 1 };
            // Geometry.hpp:155-158
            let mut i = if axis == 0 { ip1.x } else { ip1.y };
            let mut j = if axis == 0 { ip2.x } else { ip2.y };
            let mut k = if axis == 0 { jp1.x } else { jp1.y };
            let mut l = if axis == 0 { jp2.x } else { jp2.y };
            // Geometry.hpp:159-162
            if i > j {
                std::mem::swap(&mut i, &mut j);
            }
            if k > l {
                std::mem::swap(&mut k, &mut l);
            }
            // Geometry.hpp:163
            return (k >= i && k <= j) || (i >= k && i <= l);
        }
    }
    // Geometry.hpp:166
    false
}

/// True if two directions (radians) are parallel within `max_diff` (+EPSILON).
/// Geometry.cpp:29 `directions_parallel`.
pub fn directions_parallel(angle1: CoordF, angle2: CoordF, max_diff: CoordF) -> bool {
    let diff = (angle1 - angle2).abs();
    let max_diff = max_diff + crate::libslic3r::EPSILON;
    diff < max_diff || (diff - std::f64::consts::PI).abs() < max_diff
}

/// True if two directions (radians) are perpendicular within `max_diff` (+EPSILON).
/// Geometry.cpp:36 `directions_perpendicular`.
pub fn directions_perpendicular(angle1: CoordF, angle2: CoordF, max_diff: CoordF) -> bool {
    let diff = (angle1 - angle2).abs();
    let max_diff = max_diff + crate::libslic3r::EPSILON;
    (diff - 0.5 * std::f64::consts::PI).abs() < max_diff
        || (diff - 1.5 * std::f64::consts::PI).abs() < max_diff
}

/// Convert a line direction (radians) to a "compass" degree heading.
/// Geometry.cpp:53 `rad2deg_dir`.
pub fn rad2deg_dir(angle: CoordF) -> CoordF {
    let pi = std::f64::consts::PI;
    let mut a = if angle < pi { -angle + pi / 2.0 } else { angle + pi / 2.0 };
    if a < 0.0 {
        a += pi;
    }
    180.0 * a / pi // rad2deg
}

/// Linear remap of `value` from [oldmin,oldmax] to [newmin,newmax]. Geometry.cpp:73 `linint`.
pub fn linint(value: CoordF, oldmin: CoordF, oldmax: CoordF, newmin: CoordF, newmax: CoordF) -> CoordF {
    (value - oldmin) * (newmax - newmin) / (oldmax - oldmin) + newmin
}

/// Liang–Barsky clip of segment (x0,x1) against the axis-aligned box [bb_min,bb_max].
/// Returns the clipped endpoints, or None if fully outside.
/// Geometry.hpp `liang_barsky_line_clipping` / `_interval`.
pub fn liang_barsky_line_clipping(
    x0: (CoordF, CoordF),
    x1: (CoordF, CoordF),
    bb_min: (CoordF, CoordF),
    bb_max: (CoordF, CoordF),
) -> Option<((CoordF, CoordF), (CoordF, CoordF))> {
    let vx = x1.0 - x0.0;
    let vy = x1.1 - x0.1;
    let mut t0 = 0.0_f64;
    let mut t1 = 1.0_f64;
    // clip_side(p, q): traverse left/right/bottom/top edges.
    let mut clip = |p: CoordF, q: CoordF| -> bool {
        if p == 0.0 {
            // Line parallel to this edge: outside only if q < 0.
            return q >= 0.0;
        }
        let r = q / p;
        if p < 0.0 {
            if r > t1 {
                return false;
            }
            if r > t0 {
                t0 = r;
            }
        } else {
            if r < t0 {
                return false;
            }
            if r < t1 {
                t1 = r;
            }
        }
        true
    };
    if clip(-vx, -bb_min.0 + x0.0)
        && clip(vx, bb_max.0 - x0.0)
        && clip(-vy, -bb_min.1 + x0.1)
        && clip(vy, bb_max.1 - x0.1)
    {
        Some((
            (x0.0 + t0 * vx, x0.1 + t0 * vy),
            (x0.0 + t1 * vx, x0.1 + t1 * vy),
        ))
    } else {
        None
    }
}

#[inline]
/// Calculate the dot product of two 2D vectors
/// Point.hpp:152-155
pub fn dot2(v1: Point, v2: Point) -> i128 {
    // Point.hpp:153
    v1.x as i128 * v2.x as i128 + v1.y as i128 * v2.y as i128
}

#[inline]
/// Calculate the dot product of two 2D vectors (floating-point version)
/// Point.hpp:152-155
pub fn dot2f(v1: PointF, v2: PointF) -> CoordF {
    // Point.hpp:153
    v1.x * v2.x + v1.y * v2.y
}

#[inline]
/// Calculate the perpendicular vector (rotate 90 degrees counter-clockwise)
/// Point.hpp:157-160
pub fn perp(v: Point) -> Point {
    // Point.hpp:158
    Point::new(-v.y, v.x)
}

#[inline]
/// Calculate the perpendicular vector (floating-point version)
/// Point.hpp:157-160
pub fn perpf(v: PointF) -> PointF {
    // Point.hpp:158
    PointF::new(-v.y, v.x)
}

/// Calculate the angle between two vectors (in radians)
/// Point.hpp:162-167
pub fn angle_between(v1: PointF, v2: PointF) -> CoordF {
    // Compute dot product of the two vectors
    // Point.hpp:163
    let dot = dot2f(v1, v2);
    // Compute cross product of the two vectors
    // Point.hpp:164
    let cross = cross2f(v1, v2);
    // Point.hpp:165
    cross.atan2(dot)
}

#[inline]
/// Linear interpolation between two points
/// Point.hpp:169-174
pub fn lerp(a: Point, b: Point, t: CoordF) -> Point {
    // Point.hpp:170-172
    Point::new(
        (a.x as CoordF + (b.x - a.x) as CoordF * t).round() as Coord,
        (a.y as CoordF + (b.y - a.y) as CoordF * t).round() as Coord,
    )
}

#[inline]
/// Linear interpolation between two points (floating-point version)
/// Point.hpp:169-174
pub fn lerpf(a: PointF, b: PointF, t: CoordF) -> PointF {
    // Point.hpp:170-171
    PointF::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

#[inline]
/// Check if a value is approximately equal to another within epsilon
/// Point.hpp:176-179
pub fn approx_eq(a: CoordF, b: CoordF, epsilon: CoordF) -> bool {
    // Point.hpp:177
    (a - b).abs() < epsilon
}

#[inline]
/// Check if two points are approximately equal
/// Point.hpp:181-185
pub fn points_approx_eq(a: PointF, b: PointF, epsilon: CoordF) -> bool {
    // Point.hpp:182
    approx_eq(a.x, b.x, epsilon) && approx_eq(a.y, b.y, epsilon)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Orientation of three points
/// Point.hpp:187-193
pub enum Orientation {
    /// Counter-clockwise (left turn)
    /// Point.hpp:188
    CounterClockwise,
    /// Clockwise (right turn)
    /// Point.hpp:189
    Clockwise,
    /// Collinear (no turn)
    /// Point.hpp:190
    Collinear,
}

/// Determine the orientation of three points
/// Point.hpp:195-203
pub fn orientation(p1: Point, p2: Point, p3: Point) -> Orientation {
    // Compute cross product to determine turn direction
    // Point.hpp:196
    let cross = cross2(p2 - p1, p3 - p2);
    // Classify orientation based on cross product sign
    // Point.hpp:197-201
    if cross > 0 {
        Orientation::CounterClockwise
    } else
    // Check if clockwise turn
    // Point.hpp:199
    if cross < 0 {
        Orientation::Clockwise
    } else {
        Orientation::Collinear
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cross2() {
        let v1 = Point::new(1, 0);
        let v2 = Point::new(0, 1);
        assert_eq!(cross2(v1, v2), 1); // Counter-clockwise

        let v3 = Point::new(0, -1);
        assert_eq!(cross2(v1, v3), -1); // Clockwise
    }

    #[test]
    fn test_perp() {
        let v = Point::new(1, 0);
        let p = perp(v);
        assert_eq!(p.x, 0);
        assert_eq!(p.y, 1);
    }

    #[test]
    fn test_orientation() {
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1, 0);
        let p3 = Point::new(1, 1);
        assert_eq!(orientation(p1, p2, p3), Orientation::CounterClockwise);

        let p4 = Point::new(1, -1);
        assert_eq!(orientation(p1, p2, p4), Orientation::Clockwise);

        let p5 = Point::new(2, 0);
        assert_eq!(orientation(p1, p2, p5), Orientation::Collinear);
    }

    #[test]
    fn test_lerp() {
        let a = Point::new(0, 0);
        let b = Point::new(100, 100);
        let mid = lerp(a, b, 0.5);
        assert_eq!(mid.x, 50);
        assert_eq!(mid.y, 50);
    }
}
