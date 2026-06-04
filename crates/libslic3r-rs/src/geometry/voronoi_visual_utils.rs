//! Voronoi diagram visualization utilities.
//!
//! C++ Reference:
//! - Geometry/VoronoiVisualUtils.hpp
//!
//! Provides utilities for discretizing and visualizing Voronoi diagram edges,
//! particularly parabolic edges formed between point and segment sources.
//! Based on boost::polygon's voronoi_graphic_utils.

/// A 2D point with floating-point coordinates for visualization.
///
/// Geometry/VoronoiVisualUtils.hpp: Point template
#[derive(Debug, Clone, Copy, Default)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A line segment defined by two points.
///
/// Geometry/VoronoiVisualUtils.hpp: Segment
#[derive(Debug, Clone, Copy, Default)]
pub struct Segment {
    pub low: Point,
    pub high: Point,
}

impl Segment {
    pub fn new(low: Point, high: Point) -> Self {
        Self { low, high }
    }
}

/// Voronoi visual utilities for discretizing parabolic edges.
///
/// Geometry/VoronoiVisualUtils.hpp: voronoi_visual_utils
pub struct VoronoiVisualUtils;

impl VoronoiVisualUtils {
    /// Discretize a parabolic Voronoi edge.
    ///
    /// Parabolic Voronoi edges are formed by one point and one segment from
    /// the initial input set. This method subdivides the edge into line segments
    /// such that the maximum distance from the true parabola is less than max_dist.
    ///
    /// `discretization` should contain both edge endpoints initially.
    ///
    /// Geometry/VoronoiVisualUtils.hpp: voronoi_visual_utils::discretize
    pub fn discretize(
        point: &Point,
        segment: &Segment,
        max_dist: f64,
        discretization: &mut Vec<Point>,
    ) {
        if discretization.len() < 2 {
            return;
        }

        let segm_vec_x = segment.high.x - segment.low.x;
        let segm_vec_y = segment.high.y - segment.low.y;
        let sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

        if sqr_segment_length < 1e-12 {
            return; // degenerate segment
        }

        // Compute projections of the endpoints onto the segment direction
        let projection_start =
            sqr_segment_length * Self::get_point_projection(&discretization[0], segment);
        let projection_end =
            sqr_segment_length * Self::get_point_projection(&discretization[1], segment);

        if (projection_start - projection_end).abs() < 1e-12 {
            return;
        }

        // Compute parabola parameters in the transformed space
        let point_vec_x = point.x - segment.low.x;
        let point_vec_y = point.y - segment.low.y;
        let rot_x = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
        let rot_y = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;

        if rot_y.abs() < 1e-12 {
            return; // degenerate parabola
        }

        // Save the last point
        let last_point = discretization[1];
        discretization.pop();

        // Use stack to avoid recursion
        let mut point_stack = vec![projection_end];
        let mut cur_x = projection_start;
        let mut cur_y = Self::parabola_y(cur_x, rot_x, rot_y);

        let max_dist_transformed = max_dist * max_dist * sqr_segment_length;

        while let Some(&new_x) = point_stack.last() {
            let new_y = Self::parabola_y(new_x, rot_x, rot_y);
            let mid_x = (cur_x + new_x) * 0.5;
            let mid_y = Self::parabola_y(mid_x, rot_x, rot_y);

            // Compute distance from midpoint to the line segment (cur, new)
            let dist = (new_y - cur_y) * (mid_x - cur_x) - (new_x - cur_x) * (mid_y - cur_y);
            let dist_sq = dist * dist
                / ((new_y - cur_y) * (new_y - cur_y) + (new_x - cur_x) * (new_x - cur_x) + 1e-30);

            if dist_sq <= max_dist_transformed {
                // Close enough: output the point and advance
                point_stack.pop();
                let inter_x = (new_x - rot_x) / sqr_segment_length;
                let inter_y = new_y / sqr_segment_length;
                discretization.push(Point::new(
                    inter_x * segm_vec_x - inter_y * segm_vec_y + segment.low.x,
                    inter_x * segm_vec_y + inter_y * segm_vec_x + segment.low.y,
                ));
                cur_x = new_x;
                cur_y = new_y;
            } else {
                // Need to subdivide further
                point_stack.push(mid_x);
            }
        }

        // Replace the last generated point with the exact endpoint
        if let Some(last) = discretization.last_mut() {
            *last = last_point;
        }
    }

    /// Get the projection of a point onto a segment, normalized to [0, 1].
    fn get_point_projection(pt: &Point, segment: &Segment) -> f64 {
        let segment_vec_x = segment.high.x - segment.low.x;
        let segment_vec_y = segment.high.y - segment.low.y;
        let point_vec_x = pt.x - segment.low.x;
        let point_vec_y = pt.y - segment.low.y;
        let sqr_segment_length = segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
        if sqr_segment_length < 1e-30 {
            return 0.0;
        }
        let dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
        dot / sqr_segment_length
    }

    /// Compute the y coordinate of the parabola at the given x.
    /// Parabola: f(x) = ((x - rot_x)^2 + rot_y^2) / (2 * rot_y)
    fn parabola_y_internal(x: f64, rot_x: f64, rot_y: f64) -> f64 {
        ((x - rot_x) * (x - rot_x) + rot_y * rot_y) / (2.0 * rot_y)
    }
}

/// Compute the y coordinate of a parabola in transformed space.
///
/// The parabola has the representation:
///   f(x) = ((x - rot_x)^2 + rot_y^2) / (2.0 * rot_y)
///
/// Geometry/VoronoiVisualUtils.hpp: parabola_y
pub fn parabola_y(x: f64, rot_x: f64, rot_y: f64) -> f64 {
    ((x - rot_x) * (x - rot_x) + rot_y * rot_y) / (2.0 * rot_y)
}

// Private helper for VoronoiVisualUtils to call the free function
impl VoronoiVisualUtils {
    fn parabola_y(x: f64, rot_x: f64, rot_y: f64) -> f64 {
        parabola_y(x, rot_x, rot_y)
    }
}

/// Retrieve a point from a Voronoi cell source.
///
/// Returns the appropriate source point for a cell, depending on whether
/// the source is a point or a segment endpoint.
///
/// Geometry/VoronoiVisualUtils.hpp: retrieve_point
pub fn retrieve_point(segment: &Segment, is_start: bool) -> Point {
    if is_start {
        segment.low
    } else {
        segment.high
    }
}

/// Get the bounding box extents of a set of segments.
///
/// Returns (min_x, min_y, max_x, max_y).
///
/// Geometry/VoronoiVisualUtils.hpp: get_extents
pub fn get_extents(segments: &[Segment]) -> (f64, f64, f64, f64) {
    if segments.is_empty() {
        return (0.0, 0.0, 0.0, 0.0);
    }
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;

    for seg in segments {
        for pt in &[seg.low, seg.high] {
            min_x = min_x.min(pt.x);
            min_y = min_y.min(pt.y);
            max_x = max_x.max(pt.x);
            max_y = max_y.max(pt.y);
        }
    }
    (min_x, min_y, max_x, max_y)
}

/// Generate SVG output for Voronoi diagram visualization.
///
/// This is a stub that returns an empty string since SVG generation
/// is primarily a debugging/visualization tool and not needed for slicing.
///
/// Geometry/VoronoiVisualUtils.hpp: svg
pub fn svg() -> String {
    String::new()
}

/// Color the exterior cells/edges of a Voronoi diagram.
///
/// This is a no-op stub. In the full implementation, it would traverse
/// the Voronoi diagram and mark exterior elements for visualization.
///
/// Geometry/VoronoiVisualUtils.hpp: color_exterior
pub fn color_exterior() {
    // No-op: exterior coloring is a visualization concern
}
