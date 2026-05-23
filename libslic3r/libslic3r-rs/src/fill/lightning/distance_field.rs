//! Lightning infill distance field.
//!
//! C++ Reference:
//! - Fill/Lightning/DistanceField.hpp
//! - Fill/Lightning/DistanceField.cpp
//!
//! The distance field tracks which points in the fill region are not yet
//! supported by tree branches. It provides nearest-unsupported-point queries
//! for tree growth, and is updated as branches are added.

use crate::geometry::{ExPolygon, Point};
use crate::Coord;

/// A grid cell that may contain unsupported points.
///
/// DistanceField.hpp: UnsupportedCell
#[derive(Debug, Clone)]
pub struct UnsupportedCell {
    /// Points in this cell that are not yet supported.
    pub points: Vec<Point>,
    /// Whether this cell has been fully supported (all points reached).
    pub is_supported: bool,
}

impl UnsupportedCell {
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            is_supported: false,
        }
    }
}

impl Default for UnsupportedCell {
    fn default() -> Self {
        Self::new()
    }
}

/// Grid-based spatial index of unsupported points.
///
/// DistanceField.hpp: UnsupportedPointsGrid
#[derive(Debug, Clone)]
pub struct UnsupportedPointsGrid {
    /// Grid cells containing unsupported points.
    pub cells: Vec<UnsupportedCell>,
    /// Grid resolution (cell size in scaled units).
    pub cell_size: Coord,
    /// Grid dimensions.
    pub grid_width: usize,
    pub grid_height: usize,
    /// Origin of the grid (min corner of bounding box).
    pub origin: Point,
}

impl UnsupportedPointsGrid {
    pub fn new(cell_size: Coord) -> Self {
        Self {
            cells: Vec::new(),
            cell_size,
            grid_width: 0,
            grid_height: 0,
            origin: Point::new(0, 0),
        }
    }
}

impl Default for UnsupportedPointsGrid {
    fn default() -> Self {
        Self::new(1000)
    }
}

/// Distance field for lightning infill tree growth.
///
/// Tracks unsupported regions and provides queries for the nearest
/// unsupported point to guide tree branch growth.
///
/// DistanceField.hpp: class DistanceField
#[derive(Debug, Clone)]
pub struct DistanceField {
    /// Grid of unsupported points.
    pub grid: UnsupportedPointsGrid,
    /// Supporting distance: how far a tree branch can reach to support a point.
    pub supporting_radius: Coord,
    /// Total number of unsupported points remaining.
    pub unsupported_count: usize,
}

impl DistanceField {
    /// Create a new distance field from the fill region outlines.
    ///
    /// DistanceField.cpp: DistanceField()
    /// Populates the grid with sample points from the outlines.
    pub fn new(_outlines: &[ExPolygon], _supporting_radius: Coord, _cell_size: Coord) -> Self {
        // Full implementation would sample points along the outline edges
        // and interior, then populate the grid.
        Self {
            grid: UnsupportedPointsGrid::default(),
            supporting_radius: _supporting_radius,
            unsupported_count: 0,
        }
    }

    /// Update the distance field after a tree branch has been added.
    ///
    /// DistanceField.cpp: update()
    /// Marks points near the branch as supported.
    pub fn update(&mut self, _branch_from: Point, _branch_to: Point) {
        // Full implementation would find all grid cells near the branch
        // segment and mark their points as supported.
    }

    /// Mark a specific point index as erased (supported).
    ///
    /// DistanceField.cpp: mark_erased()
    pub fn mark_erased(&mut self, _point_idx: usize) {
        if self.unsupported_count > 0 {
            self.unsupported_count -= 1;
        }
    }

    /// Find the grid cell index for a given point.
    ///
    /// DistanceField.cpp: find_cell_idx()
    pub fn find_cell_idx(&self, point: Point) -> Option<usize> {
        if self.grid.cell_size <= 0 || self.grid.grid_width == 0 {
            return None;
        }
        let col = ((point.x - self.grid.origin.x) / self.grid.cell_size) as usize;
        let row = ((point.y - self.grid.origin.y) / self.grid.cell_size) as usize;
        if col < self.grid.grid_width && row < self.grid.grid_height {
            Some(row * self.grid.grid_width + col)
        } else {
            None
        }
    }

    /// Check if there are any unsupported points remaining.
    pub fn has_unsupported_points(&self) -> bool {
        self.unsupported_count > 0
    }
}

impl Default for DistanceField {
    fn default() -> Self {
        Self {
            grid: UnsupportedPointsGrid::default(),
            supporting_radius: 0,
            unsupported_count: 0,
        }
    }
}

/// Export distance field to SVG for debugging.
///
/// DistanceField.cpp: export_distance_field_to_svg()
pub fn export_distance_field_to_svg(_field: &DistanceField, _path: &str) {
    // Debug visualization - no-op in production.
}
