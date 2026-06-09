//! Line infill pattern.
//!
//! C++ Reference:
//! - Fill/FillLine.hpp
//! - Fill/FillLine.cpp
//!
//! Line infill generates parallel lines with optional oscillation (wiggle).
//! It is a variant of rectilinear infill where alternating lines are offset
//! horizontally to create a zigzag-like pattern within each line.

use crate::geometry::{ExPolygon, Point, Polyline};
use crate::{Coord, CoordF};

/// Line infill pattern generator.
/// FillLine.hpp: class FillLine
#[derive(Debug, Clone)]
pub struct FillLine {
    /// Minimum spacing between lines.
    pub min_spacing: Coord,
    /// Actual line spacing (adjusted for density).
    pub line_spacing: Coord,
    /// Distance threshold for connecting adjacent lines into a continuous path.
    pub diagonal_distance: Coord,
    /// Horizontal oscillation offset for alternating lines.
    pub line_oscillation: Coord,
}

impl FillLine {
    /// Create a new FillLine with given spacing.
    pub fn new(spacing: CoordF, density: CoordF) -> Self {
        let min_spacing = (spacing * 1e6) as Coord;
        let line_spacing = if density > 0.0 {
            (min_spacing as f64 / density) as Coord
        } else {
            min_spacing
        };
        Self {
            min_spacing,
            line_spacing,
            diagonal_distance: line_spacing,
            line_oscillation: 0,
        }
    }

    /// Compute a single line segment with oscillation.
    /// FillLine.hpp: _line()
    fn line_segment(&self, i: i32, x: Coord, y_min: Coord, y_max: Coord) -> (Point, Point) {
        let osc = if i & 1 != 0 { self.line_oscillation } else { 0 };
        (Point::new(x - osc, y_min), Point::new(x + osc, y_max))
    }

    /// Check if two adjacent line endpoints can be connected.
    /// FillLine.hpp: _can_connect()
    fn can_connect(&self, dist_x: Coord, dist_y: Coord) -> bool {
        let tolerance = 10_000; // 10 * SCALED_EPSILON
        dist_x >= (self.line_spacing - self.line_oscillation) - tolerance
            && dist_x <= (self.line_spacing + self.line_oscillation) + tolerance
            && dist_y <= self.diagonal_distance
    }
}

impl Default for FillLine {
    fn default() -> Self {
        Self {
            min_spacing: 0,
            line_spacing: 0,
            diagonal_distance: 0,
            line_oscillation: 0,
        }
    }
}

/// Generate line infill for a single expolygon.
///
/// Produces parallel vertical lines spaced by `line_spacing`, clipped
/// to the expolygon boundary. Adjacent line segments are connected when
/// possible to reduce travel moves.
pub fn generate_line_infill(
    fill_area: &[ExPolygon],
    spacing: CoordF,
    density: CoordF,
) -> Vec<Polyline> {
    use crate::geometry::BoundingBox;

    let filler = FillLine::new(spacing, density);
    if filler.line_spacing <= 0 {
        return Vec::new();
    }

    let mut result = Vec::new();

    for expoly in fill_area {
        let mut bb = BoundingBox::default();
        for pt in &expoly.contour.points {
            bb.merge_point(*pt);
        }

        // Generate vertical lines across the bounding box
        let mut x = bb.min.x;
        let mut idx = 0i32;
        while x <= bb.max.x {
            let (p1, p2) = filler.line_segment(idx, x, bb.min.y, bb.max.y);
            result.push(Polyline::from_points(vec![p1, p2]));
            x += filler.line_spacing;
            idx += 1;
        }
    }

    // TODO: Clip polylines to expolygon boundaries and connect adjacent segments
    result
}
