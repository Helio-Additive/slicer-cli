//! Lightning infill layer.
//!
//! C++ Reference:
//! - Fill/Lightning/Layer.hpp
//! - Fill/Lightning/Layer.cpp
//!
//! Each Layer holds a forest of tree nodes for one print layer. Trees
//! are grown from unsupported (overhang) points toward grounded regions,
//! and the edges of each tree become infill lines.

use super::tree_node::Node;
use crate::geometry::{ExPolygon, Point, Polygon, Polyline};
use crate::Coord;

/// A grounding location where a tree can connect to the build plate
/// or a previously supported region.
///
/// Layer.hpp: GroundingLocation
#[derive(Debug, Clone)]
pub struct GroundingLocation {
    /// The grounding point.
    pub point: Point,
    /// Index of the boundary polygon this location is on, if any.
    pub boundary_idx: Option<usize>,
}

impl GroundingLocation {
    pub fn new(point: Point) -> Self {
        Self {
            point,
            boundary_idx: None,
        }
    }
}

impl Default for GroundingLocation {
    fn default() -> Self {
        Self {
            point: Point::new(0, 0),
            boundary_idx: None,
        }
    }
}

/// A lightning infill layer containing a forest of tree nodes.
///
/// Layer.hpp: class Layer
#[derive(Debug, Clone, Default)]
pub struct Layer {
    /// Root nodes of the tree forest for this layer.
    pub tree_roots: Vec<Node>,
}

impl Layer {
    /// Create a new empty layer.
    pub fn new() -> Self {
        Self {
            tree_roots: Vec::new(),
        }
    }

    /// Convert the tree forest to polylines for infill extrusion.
    ///
    /// Layer.cpp: convert_to_lines()
    pub fn convert_to_lines(&self) -> Vec<Polyline> {
        let mut result = Vec::new();
        for root in &self.tree_roots {
            result.extend(root.to_polylines());
        }
        result
    }

    /// Reconnect tree roots that may have become disconnected after
    /// layer operations (e.g., after clipping to a new outline).
    ///
    /// Layer.cpp: reconnect_roots()
    pub fn reconnect_roots(&mut self, _outlines: &[ExPolygon], _max_distance: Coord) {
        // Full implementation would find disconnected roots and try to
        // re-ground them on the current layer's outline.
        // No-op for now; trees remain as-is.
    }

    /// Fill the spatial locator for fast nearest-neighbor queries.
    ///
    /// Layer.cpp: fill_locator()
    pub fn fill_locator(&self) -> Vec<Point> {
        // Returns all node locations for spatial indexing
        let mut points = Vec::new();
        fn collect_points(node: &Node, out: &mut Vec<Point>) {
            out.push(node.location);
            for child in &node.children {
                collect_points(child, out);
            }
        }
        for root in &self.tree_roots {
            collect_points(root, &mut points);
        }
        points
    }
}

/// Compute the weighted distance between two points for tree growth.
///
/// Layer.cpp: get_weighted_distance()
/// Returns the distance weighted by proximity to polygon boundaries.
pub fn get_weighted_distance(a: Point, b: Point, _boundary: &[Polygon]) -> f64 {
    // Simplified: just return Euclidean distance
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    (dx * dx + dy * dy).sqrt()
}
