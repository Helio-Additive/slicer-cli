//! MultiPoint - base class for polylines and polygons
//!
//! C++ Reference:
//! - MultiPoint.hpp (52 lines)
//! - MultiPoint.cpp (294 lines)
//!
//! This module provides the Douglas-Peucker simplification algorithm
//! and other multi-point operations used throughout libslic3r.

use crate::geometry::{Line, Point};

/// Douglas-Peucker line simplification algorithm
/// Reduces the number of points in a polyline while maintaining shape fidelity
/// MultiPoint.cpp:236-294
pub fn douglas_peucker(pts: &[Point], tolerance: f64) -> Vec<Point> {
    // Initialize result vector
    // MultiPoint.cpp:238
    let mut result_pts = Vec::new();

    // Compute tolerance squared for distance comparison
    // MultiPoint.cpp:239
    let tolerance_sq = tolerance * tolerance;

    // Handle empty input
    // MultiPoint.cpp:240
    if pts.is_empty() {
        return result_pts;
    }

    // Initialize anchor (start point) and floater (end point)
    // MultiPoint.cpp:241-244
    let anchor_idx = 0;
    let mut floater_idx = pts.len() - 1;

    // Reserve capacity for result
    // MultiPoint.cpp:245
    result_pts.reserve(pts.len());

    // Always include first point
    // MultiPoint.cpp:246
    result_pts.push(pts[anchor_idx]);

    // Handle single-point case
    // MultiPoint.cpp:247
    if anchor_idx == floater_idx {
        return result_pts;
    }

    // Multi-point simplification using stack-based recursion
    // MultiPoint.cpp:248-281
    // assert!(pts.len() > 1);

    // Stack for tracking segment endpoints during recursion
    // MultiPoint.cpp:249-251
    let mut dp_stack: Vec<usize> = Vec::new();
    dp_stack.reserve(pts.len());
    dp_stack.push(floater_idx);

    // Current anchor index (moves as we process segments)
    // MultiPoint.cpp:252
    let mut anchor_idx = anchor_idx;

    // Main simplification loop
    // MultiPoint.cpp:252
    loop {
        // Find furthest point from line segment (anchor, floater)
        // MultiPoint.cpp:253-260
        let mut max_dist_sq = 0.0;
        let mut furthest_idx = anchor_idx;

        // MultiPoint.cpp:256
        for i in (anchor_idx + 1)..floater_idx {
            // Compute perpendicular distance squared from point to line
            // MultiPoint.cpp:257
            let dist_sq = Line::distance_to_squared(pts[i], pts[anchor_idx], pts[floater_idx]);

            // MultiPoint.cpp:258-261
            if dist_sq > max_dist_sq {
                max_dist_sq = dist_sq;
                furthest_idx = i;
            }
        }

        // Check if furthest point is within tolerance
        // MultiPoint.cpp:263
        if max_dist_sq <= tolerance_sq {
            // All points between anchor and floater are within tolerance
            // Add floater to result and move to next segment
            // MultiPoint.cpp:264
            result_pts.push(pts[floater_idx]);

            // Move anchor to floater position
            // MultiPoint.cpp:265-266
            anchor_idx = floater_idx;

            // Pop from stack to get next segment
            // MultiPoint.cpp:267-268
            // assert!(dp_stack.back() == floater_idx);
            dp_stack.pop();

            // Exit if no more segments to process
            // MultiPoint.cpp:269-270
            if dp_stack.is_empty() {
                break;
            }

            // Get next floater from stack
            // MultiPoint.cpp:271
            floater_idx = *dp_stack.last().unwrap();
        } else {
            // Furthest point exceeds tolerance
            // Subdivide segment at furthest point and recurse
            // MultiPoint.cpp:273-275
            floater_idx = furthest_idx;
            dp_stack.push(floater_idx);
        }
    }

    // Verify result integrity (first and last points preserved)
    // MultiPoint.cpp:279-280
    debug_assert_eq!(result_pts.first(), pts.first());
    debug_assert_eq!(result_pts.last(), pts.last());

    // Return simplified point sequence
    // MultiPoint.cpp:292
    result_pts
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_douglas_peucker_empty() {
        let pts = vec![];
        let result = douglas_peucker(&pts, 0.1);
        assert_eq!(result.len(), 0);
    }

    #[test]
    fn test_douglas_peucker_single_point() {
        let pts = vec![Point::new(0, 0)];
        let result = douglas_peucker(&pts, 0.1);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0], pts[0]);
    }

    #[test]
    fn test_douglas_peucker_two_points() {
        let pts = vec![Point::new(0, 0), Point::new(100, 100)];
        let result = douglas_peucker(&pts, 0.1);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], pts[0]);
        assert_eq!(result[1], pts[1]);
    }

    #[test]
    fn test_douglas_peucker_straight_line() {
        /// Points on a straight line should be reduced to endpoints
        let pts = vec![
            Point::new(0, 0),
            Point::new(10, 10),
            Point::new(20, 20),
            Point::new(30, 30),
            Point::new(40, 40),
        ];
        let result = douglas_peucker(&pts, 1.0);
        /// Should keep only start and end
        assert_eq!(result.len(), 2);
        assert_eq!(result[0], pts[0]);
        assert_eq!(result[1], pts[4]);
    }

    #[test]
    fn test_douglas_peucker_zigzag() {
        /// Zigzag pattern should keep significant points
        let pts = vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(10, 10),
            Point::new(20, 10),
            Point::new(20, 0),
        ];
        /// With tight tolerance, should keep most points
        let result = douglas_peucker(&pts, 0.1);
        assert!(result.len() >= 3); // At least start, middle turn, end
        assert_eq!(result[0], pts[0]);
        assert_eq!(result[result.len() - 1], pts[pts.len() - 1]);
    }

    #[test]
    fn test_douglas_peucker_preserves_endpoints() {
        /// First and last points must always be preserved
        let pts = vec![
            Point::new(0, 0),
            Point::new(5, 1),
            Point::new(10, 2),
            Point::new(15, 1),
            Point::new(20, 0),
        ];
        let result = douglas_peucker(&pts, 10.0);
        assert!(result.len() >= 2);
        assert_eq!(result[0], pts[0]);
        assert_eq!(result[result.len() - 1], pts[pts.len() - 1]);
    }
}
