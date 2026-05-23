//! Voronoi diagram utility functions.
//!
//! C++ Reference:
//! - Geometry/VoronoiUtils.hpp
//! - Geometry/VoronoiUtils.cpp
//!
//! Provides utility functions for working with Voronoi diagrams in the context
//! of the Arachne variable-width algorithm.

use crate::geometry::Point;

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
