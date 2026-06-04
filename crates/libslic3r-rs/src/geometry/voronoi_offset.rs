//! Voronoi-based polygon offsetting.
//!
//! C++ Reference:
//! - Geometry/VoronoiOffset.hpp
//! - Geometry/VoronoiOffset.cpp
//!
//! Provides types and functions for polygon offsetting using Voronoi diagrams.
//! The Voronoi diagram cells, vertices, and edges are annotated with inside/outside
//! information to support the offset algorithm.

use crate::{Error, Result};

/// Category assigned to Voronoi diagram vertices.
///
/// Geometry/VoronoiOffset.hpp: VertexCategory
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexCategory {
    /// Voronoi vertex is on the input contour.
    OnContour,
    /// Vertex is inside the CCW input contour (holes respected).
    Inside,
    /// Vertex is outside the CCW input contour (holes respected).
    Outside,
    /// Not classified yet.
    Unknown,
}

impl Default for VertexCategory {
    fn default() -> Self {
        Self::Unknown
    }
}

/// Category assigned to Voronoi diagram half-edges.
/// Classified based on the target vertex (vertex1).
///
/// Geometry/VoronoiOffset.hpp: EdgeCategory
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeCategory {
    /// This half-edge points onto the contour.
    PointsToContour,
    /// This half-edge points inside the polygon.
    PointsInside,
    /// This half-edge points outside the polygon.
    PointsOutside,
    /// Not classified yet.
    Unknown,
}

impl Default for EdgeCategory {
    fn default() -> Self {
        Self::Unknown
    }
}

/// Category assigned to Voronoi diagram cells.
///
/// Geometry/VoronoiOffset.hpp: CellCategory
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellCategory {
    /// Cell is split by an input segment: one half inside, one outside.
    Boundary,
    /// Cell is completely inside the polygon.
    Inside,
    /// Cell is completely outside the polygon.
    Outside,
    /// Not classified yet.
    Unknown,
}

impl Default for CellCategory {
    fn default() -> Self {
        Self::Unknown
    }
}

/// Get the vertex category from a color value.
///
/// Geometry/VoronoiOffset.hpp: vertex_category
pub fn vertex_category(color: u8) -> VertexCategory {
    match color {
        0 => VertexCategory::OnContour,
        1 => VertexCategory::Inside,
        2 => VertexCategory::Outside,
        _ => VertexCategory::Unknown,
    }
}

/// Get the edge category from a color value.
///
/// Geometry/VoronoiOffset.hpp: edge_category (implicit)
pub fn edge_category(color: u8) -> EdgeCategory {
    match color {
        0 => EdgeCategory::PointsToContour,
        1 => EdgeCategory::PointsInside,
        2 => EdgeCategory::PointsOutside,
        _ => EdgeCategory::Unknown,
    }
}

/// Get the cell category from a color value.
///
/// Geometry/VoronoiOffset.hpp: cell_category
pub fn cell_category(color: u8) -> CellCategory {
    match color {
        0 => CellCategory::Boundary,
        1 => CellCategory::Inside,
        2 => CellCategory::Outside,
        _ => CellCategory::Unknown,
    }
}

/// Check if an offset intersection point represents an actual intersection
/// (i.e., is not NaN).
///
/// Geometry/VoronoiOffset.hpp: edge_offset_has_intersection
pub fn edge_offset_has_intersection(x: f64, _y: f64) -> bool {
    !x.is_nan()
}

/// Set the vertex category color value from a VertexCategory.
///
/// Geometry/VoronoiOffset.hpp: set_vertex_category
pub fn set_vertex_category(category: VertexCategory) -> u8 {
    match category {
        VertexCategory::OnContour => 0,
        VertexCategory::Inside => 1,
        VertexCategory::Outside => 2,
        VertexCategory::Unknown => 3,
    }
}

/// Set the edge category color value from an EdgeCategory.
///
/// Geometry/VoronoiOffset.hpp: set_edge_category
pub fn set_edge_category(category: EdgeCategory) -> u8 {
    match category {
        EdgeCategory::PointsToContour => 0,
        EdgeCategory::PointsInside => 1,
        EdgeCategory::PointsOutside => 2,
        EdgeCategory::Unknown => 3,
    }
}
