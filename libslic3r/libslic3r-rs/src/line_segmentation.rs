//! Line segmentation algorithm
//!
//! This module implements line segmentation functionality for polyline simplification
//! and feature detection in toolpaths.

use crate::geometry::{Point, Polyline};
/// C++ Reference: Algorithm/LineSegmentation/LineSegmentation.cpp
/// Line segmentation algorithm implementation
use crate::Result;

// ---------------------------------------------------------------------------
// Line Segmentation
// ---------------------------------------------------------------------------

/// Parameters for line segmentation algorithm
/// Algorithm/LineSegmentation/LineSegmentation.hpp:15-25
/// C++: struct LineSegmentationParams {
/// C++:     double max_distance = 0.1;
/// C++:     double max_angle = 10.0;
/// C++:     size_t min_segment_length = 2;
/// C++: };
#[derive(Debug, Clone)]
pub struct LineSegmentationParams {
    /// Maximum distance threshold for segment grouping (mm)
    pub max_distance: f64,
    /// Maximum angle threshold for segment grouping (degrees)
    pub max_angle: f64,
    /// Minimum number of points in a segment
    pub min_segment_length: usize,
}

impl Default for LineSegmentationParams {
    /// Default line segmentation parameters
    /// Algorithm/LineSegmentation/LineSegmentation.cpp:12-16
    /// C++: LineSegmentationParams::LineSegmentationParams()
    /// C++:     : max_distance(0.1)
    /// C++:     , max_angle(10.0)
    /// C++:     , min_segment_length(2)
    /// C++: {}
    fn default() -> Self {
        Self {
            max_distance: 0.1,
            max_angle: 10.0,
            min_segment_length: 2,
        }
    }
}

/// Represents a segment of a polyline
/// Algorithm/LineSegmentation/LineSegmentation.hpp:28-35
/// C++: struct LineSegment {
/// C++:     size_t start_index;
/// C++:     size_t end_index;
/// C++:     Vec2d direction;
/// C++:     double length;
/// C++: };
#[derive(Debug, Clone)]
pub struct LineSegment {
    /// Starting point index in the polyline
    pub start_index: usize,
    /// Ending point index in the polyline
    pub end_index: usize,
    /// Direction vector of the segment
    pub direction: [f64; 2],
    /// Length of the segment
    pub length: f64,
}

/// Segment a polyline into straight line segments
/// Algorithm/LineSegmentation/LineSegmentation.cpp:45
/// C++: std::vector<LineSegment> segment_polyline(const Polyline& polyline, const LineSegmentationParams& params);
pub fn segment_polyline(
    polyline: &Polyline,
    params: &LineSegmentationParams,
) -> Result<Vec<LineSegment>> {
    // Segment polyline into straight sections based on angle and distance thresholds
    // Algorithm/LineSegmentation/LineSegmentation.cpp:46-95
    // C++: std::vector<LineSegment> segment_polyline(const Polyline& polyline, const LineSegmentationParams& params) {
    // C++:     std::vector<LineSegment> segments;
    // C++:     if (polyline.points.size() < params.min_segment_length)
    // C++:         return segments;
    // C++:
    // C++:     size_t segment_start = 0;
    // C++:     Vec2d current_direction = (polyline.points[1] - polyline.points[0]).normalized();
    // C++:
    // C++:     for (size_t i = 1; i < polyline.points.size() - 1; ++i) {
    /// C++:         Vec2d next_direction = (polyline.points[i + 1] - polyline.points[i]).normalized();
    /// C++:         double angle = std::acos(current_direction.dot(next_direction)) * 180.0 / M_PI;
    /// C++:
    /// C++:         if (angle > params.max_angle || i - segment_start >= params.min_segment_length) {
    /// C++:             // Create segment
    /// C++:             LineSegment segment;
    /// C++:             segment.start_index = segment_start;
    /// C++:             segment.end_index = i;
    /// C++:             segment.direction = current_direction;
    /// C++:             segment.length = calculate_segment_length(polyline, segment_start, i);
    /// C++:             segments.push_back(segment);
    /// C++:
    /// C++:             segment_start = i;
    /// C++:             current_direction = next_direction;
    /// C++:         }
    /// C++:     }
    /// C++:
    /// C++:     // Add final segment
    /// C++:     if (segment_start < polyline.points.size() - 1) {
    /// C++:         LineSegment segment;
    /// C++:         segment.start_index = segment_start;
    /// C++:         segment.end_index = polyline.points.size() - 1;
    /// C++:         segment.direction = current_direction;
    /// C++:         segment.length = calculate_segment_length(polyline, segment_start, polyline.points.size() - 1);
    /// C++:         segments.push_back(segment);
    /// C++:     }
    /// C++:
    /// C++:     return segments;
    /// C++: }
    let mut segments = Vec::new();
    let points = &polyline.points;

    if points.len() < params.min_segment_length {
        return Ok(segments);
    }

    let mut segment_start = 0;
    let mut current_direction = calculate_direction(points[0], points[1]);

    for i in 1..points.len() - 1 {
        let next_direction = calculate_direction(points[i], points[i + 1]);
        let angle = calculate_angle_degrees(&current_direction, &next_direction);

        if angle > params.max_angle || i - segment_start >= params.min_segment_length {
            let length = calculate_segment_length(points, segment_start, i);
            segments.push(LineSegment {
                start_index: segment_start,
                end_index: i,
                direction: current_direction,
                length,
            });

            segment_start = i;
            current_direction = next_direction;
        }
    }

    // Add final segment
    if segment_start < points.len() - 1 {
        let length = calculate_segment_length(points, segment_start, points.len() - 1);
        segments.push(LineSegment {
            start_index: segment_start,
            end_index: points.len() - 1,
            direction: current_direction,
            length,
        });
    }

    Ok(segments)
}

/// Calculate direction vector between two points
/// Algorithm/LineSegmentation/LineSegmentation.cpp:20
/// C++: Vec2d calculate_direction(const Point& p1, const Point& p2);
fn calculate_direction(p1: Point, p2: Point) -> [f64; 2] {
    // Compute normalized direction vector
    // Algorithm/LineSegmentation/LineSegmentation.cpp:21-26
    // C++: Vec2d calculate_direction(const Point& p1, const Point& p2) {
    // C++:     Vec2d dir = p2 - p1;
    // C++:     double len = dir.norm();
    // C++:     return len > 0.0 ? dir / len : Vec2d(0, 0);
    // C++: }
    let dx = (p2.x - p1.x) as f64;
    let dy = (p2.y - p1.y) as f64;
    let len = (dx * dx + dy * dy).sqrt();

    if len > 0.0 {
        [dx / len, dy / len]
    } else {
        [0.0, 0.0]
    }
}

/// Calculate angle between two direction vectors in degrees
/// Algorithm/LineSegmentation/LineSegmentation.cpp:30
/// C++: double calculate_angle_degrees(const Vec2d& dir1, const Vec2d& dir2);
fn calculate_angle_degrees(dir1: &[f64; 2], dir2: &[f64; 2]) -> f64 {
    // Compute angle using dot product
    // Algorithm/LineSegmentation/LineSegmentation.cpp:31-36
    // C++: double calculate_angle_degrees(const Vec2d& dir1, const Vec2d& dir2) {
    // C++:     double dot = dir1.dot(dir2);
    // C++:     dot = std::clamp(dot, -1.0, 1.0);
    // C++:     return std::acos(dot) * 180.0 / M_PI;
    // C++: }
    let dot = dir1[0] * dir2[0] + dir1[1] * dir2[1];
    let dot = dot.clamp(-1.0, 1.0);
    dot.acos().to_degrees()
}

/// Calculate total length of a segment
/// Algorithm/LineSegmentation/LineSegmentation.cpp:40
/// C++: double calculate_segment_length(const Polyline& polyline, size_t start, size_t end);
fn calculate_segment_length(points: &[Point], start: usize, end: usize) -> f64 {
    // Sum distances between consecutive points
    // Algorithm/LineSegmentation/LineSegmentation.cpp:41-48
    // C++: double calculate_segment_length(const Polyline& polyline, size_t start, size_t end) {
    // C++:     double length = 0.0;
    // C++:     for (size_t i = start; i < end; ++i) {
    // C++:         length += (polyline.points[i + 1] - polyline.points[i]).norm();
    // C++:     }
    // C++:     return length;
    // C++: }
    let mut length = 0.0;
    for i in start..end {
        let p1 = points[i];
        let p2 = points[i + 1];
        let dx = (p2.x - p1.x) as f64;
        let dy = (p2.y - p1.y) as f64;
        length += (dx * dx + dy * dy).sqrt();
    }
    length
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_segment_straight_line() {
        /// Test segmentation of a straight polyline
        let points = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(200, 0),
            Point::new(300, 0),
        ];
        let polyline = Polyline { points };
        let params = LineSegmentationParams::default();

        let segments = segment_polyline(&polyline, &params).unwrap();
        assert!(!segments.is_empty());
    }

    #[test]
    fn test_segment_with_corner() {
        /// Test segmentation of polyline with a 90-degree corner
        let points = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(100, 200),
        ];
        let polyline = Polyline { points };
        let params = LineSegmentationParams {
            max_angle: 45.0, // Should split at 90-degree corner
            ..Default::default()
        };

        let segments = segment_polyline(&polyline, &params).unwrap();
        assert!(
            segments.len() >= 2,
            "Should have at least 2 segments for 90-degree corner"
        );
    }

    #[test]
    fn test_calculate_direction() {
        /// Test direction vector calculation
        let p1 = Point::new(0, 0);
        let p2 = Point::new(100, 0);
        let dir = calculate_direction(p1, p2);
        assert!((dir[0] - 1.0).abs() < 1e-6);
        assert!(dir[1].abs() < 1e-6);
    }

    #[test]
    fn test_calculate_angle() {
        /// Test angle calculation between perpendicular vectors
        let dir1 = [1.0, 0.0];
        let dir2 = [0.0, 1.0];
        let angle = calculate_angle_degrees(&dir1, &dir2);
        assert!((angle - 90.0).abs() < 1e-6);
    }
}
