//! Lightning tree node.
//!
//! C++ Reference:
//! - Fill/Lightning/TreeNode.hpp
//! - Fill/Lightning/TreeNode.cpp
//!
//! Each tree node represents a point in the lightning infill tree. Nodes form
//! a tree structure where the root is grounded (on the build plate or on a
//! previously printed layer) and leaves are at overhang points. Edges between
//! nodes become the infill lines.

use crate::geometry::{Point, Polyline};
use crate::Coord;

/// A node in the lightning infill tree.
///
/// TreeNode.hpp: class Node
#[derive(Debug, Clone)]
pub struct Node {
    /// Location of this node.
    pub location: Point,
    /// Child nodes (branches growing upward/outward).
    pub children: Vec<Node>,
    /// Whether this node is the root (grounded).
    pub is_root: bool,
}

impl Node {
    /// Create a new tree node at the given location.
    pub fn new(location: Point) -> Self {
        Self {
            location,
            children: Vec::new(),
            is_root: false,
        }
    }

    /// Create a root node at the given location.
    pub fn new_root(location: Point) -> Self {
        Self {
            location,
            children: Vec::new(),
            is_root: true,
        }
    }

    /// Set the location of this node.
    /// TreeNode.hpp: setLocation()
    pub fn set_location(&mut self, location: Point) {
        self.location = location;
    }

    /// Get the location of this node.
    pub fn get_location(&self) -> Point {
        self.location
    }

    /// Add a child node.
    pub fn add_child(&mut self, child: Node) {
        self.children.push(child);
    }

    /// Propagate this node's tree to the next layer.
    ///
    /// TreeNode.cpp: propagate_to_next_layer()
    /// Creates a copy of the tree structure for the layer below, potentially
    /// adjusting positions to avoid collisions.
    pub fn propagate_to_next_layer(&self) -> Option<Node> {
        // Simplified: just clone the tree structure
        Some(self.clone())
    }

    /// Convert this node and its subtree to polylines.
    ///
    /// Each edge (parent->child) becomes a polyline segment.
    pub fn to_polylines(&self) -> Vec<Polyline> {
        let mut result = Vec::new();
        for child in &self.children {
            // Edge from self to child
            result.push(Polyline::from_points(vec![self.location, child.location]));
            // Recurse into child's subtree
            result.extend(child.to_polylines());
        }
        result
    }

    /// Straighten the tree by removing unnecessary intermediate nodes.
    ///
    /// TreeNode.cpp: straighten()
    /// Removes nodes that lie approximately on the line between their
    /// parent and child (when they have exactly one child).
    pub fn straighten(&mut self, magnitude_limit: Coord) {
        for child in &mut self.children {
            child.straighten(magnitude_limit);
        }

        if self.children.len() == 1 {
            let child = &self.children[0];
            let dx = (child.location.x - self.location.x).abs();
            let dy = (child.location.y - self.location.y).abs();
            if dx + dy < magnitude_limit {
                // Remove intermediate node by adopting grandchildren
                let grandchildren = self.children[0].children.clone();
                let child_loc = self.children[0].location;
                self.children = grandchildren;
                // Adjust location to midpoint if no grandchildren
                if self.children.is_empty() {
                    self.location = Point::new(
                        (self.location.x + child_loc.x) / 2,
                        (self.location.y + child_loc.y) / 2,
                    );
                }
            }
        }
    }
}

impl Default for Node {
    fn default() -> Self {
        Self::new(Point::new(0, 0))
    }
}

/// Junction point for rectilinear lightning tree connections.
/// TreeNode.hpp: RectilinearJunction
#[derive(Debug, Clone, Default)]
pub struct RectilinearJunction {
    pub point: Point,
}

impl RectilinearJunction {
    pub fn new(point: Point) -> Self {
        Self { point }
    }
}

/// Compute the intersection of a line segment with a set of polygons.
///
/// TreeNode.cpp: line_segment_polygons_intersection()
/// Returns the closest intersection point along the segment, if any.
pub fn line_segment_polygons_intersection(
    _from: Point,
    _to: Point,
    _polygons: &[crate::geometry::Polygon],
) -> Option<Point> {
    // Full implementation would test the segment against each polygon edge
    // and return the nearest intersection. For now returns None (no intersection).
    None
}
