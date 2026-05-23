//! Minimum Spanning Tree implementation using Prim's algorithm
//!
//! C++ Reference: MinimumSpanningTree.hpp, MinimumSpanningTree.cpp
//!
//! This module implements Prim's algorithm to compute Minimum Spanning Trees (MST)
//! over a clique of vertices represented as 2D points.

use crate::geometry::Point;
use crate::unscale;
use std::collections::HashMap;

/// Represents an edge in the spanning tree
/// MinimumSpanningTree.hpp:19-35
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Edge {
    /// The point at which this edge starts
    /// MinimumSpanningTree.hpp:30
    start: Point,

    /// The point at which this edge ends
    /// MinimumSpanningTree.hpp:34
    end: Point,
}

impl Edge {
    /// Create a new edge
    fn new(start: Point, end: Point) -> Self {
        Self { start, end }
    }
}

/// Adjacency graph representation using HashMap
/// MinimumSpanningTree.hpp:70
type AdjacencyGraph = HashMap<Point, Vec<Edge>>;

/// Minimum Spanning Tree implementation using Prim's algorithm
/// MinimumSpanningTree.hpp:16-74
#[derive(Debug, Clone)]
pub struct MinimumSpanningTree {
    /// Adjacency graph representation of the MST
    /// MinimumSpanningTree.hpp:70
    adjacency_graph: AdjacencyGraph,
}

impl MinimumSpanningTree {
    /// Create an empty minimum spanning tree
    /// MinimumSpanningTree.hpp:37
    pub fn new() -> Self {
        Self {
            adjacency_graph: HashMap::new(),
        }
    }

    /// Constructs a minimum spanning tree that spans all given vertices
    /// MinimumSpanningTree.hpp:40-41
    /// MinimumSpanningTree.cpp:21-24
    pub fn from_vertices(vertices: Vec<Point>) -> Self {
        let adjacency_graph = Self::prim(vertices);
        Self { adjacency_graph }
    }

    /// Gets the nodes that are adjacent to the specified node
    ///
    /// # Returns
    /// A list of nodes that are adjacent to the given node
    ///
    /// MinimumSpanningTree.cpp:107-118
    pub fn adjacent_nodes(&self, node: Point) -> Vec<Point> {
        let mut result = Vec::new();

        if let Some(edges) = self.adjacency_graph.get(&node) {
            for edge in edges {
                result.push(if edge.start == node {
                    edge.end
                } else {
                    edge.start
                });
            }
        }

        result
    }

    /// Gets the leaves of the tree
    ///
    /// Leaves are nodes that have only one adjacent edge, or just the one node
    /// if the tree contains one node.
    ///
    /// # Returns
    /// A list of nodes that are all leaves of the tree
    ///
    /// MinimumSpanningTree.cpp:120-130
    pub fn leaves(&self) -> Vec<Point> {
        let mut result = Vec::new();

        for (node, edges) in &self.adjacency_graph {
            if edges.len() <= 1 {
                result.push(*node);
            }
        }

        result
    }

    /// Gets all vertices of the tree
    ///
    /// # Returns
    /// A list of all vertices in the tree
    ///
    /// MinimumSpanningTree.cpp:132-138
    pub fn vertices(&self) -> Vec<Point> {
        self.adjacency_graph.keys().copied().collect()
    }

    /// Helper function: dot product with unscaling
    /// MinimumSpanningTree.cpp:11-14
    fn dot_with_unscale(a: Point, b: Point) -> f64 {
        let ax = unscale(a.x);
        let ay = unscale(a.y);
        let bx = unscale(b.x);
        let by = unscale(b.y);
        ax * bx + ay * by
    }

    /// Helper function: squared magnitude with unscaling
    /// MinimumSpanningTree.cpp:16-19
    fn vsize2_with_unscale(pt: Point) -> f64 {
        Self::dot_with_unscale(pt, pt)
    }

    /// Computes the edges of a minimum spanning tree using Prim's algorithm
    ///
    /// # Arguments
    /// * `vertices` - The vertices to span
    ///
    /// # Returns
    /// An adjacency graph with for each point one or more edges
    ///
    /// MinimumSpanningTree.cpp:26-105
    fn prim(vertices: Vec<Point>) -> AdjacencyGraph {
        let mut result = AdjacencyGraph::new();

        // Handle empty vertex set
        // MinimumSpanningTree.cpp:28-31
        if vertices.is_empty() {
            return result;
        }

        // Handle single vertex - add with no edges
        // MinimumSpanningTree.cpp:32-38
        if vertices.len() == 1 {
            result.insert(vertices[0], Vec::new());
            return result;
        }

        result.reserve(vertices.len());

        // Initialize distance tracking maps
        // MinimumSpanningTree.cpp:43-52
        let mut smallest_distance: HashMap<Point, f64> = HashMap::new();
        let mut smallest_distance_to: HashMap<Point, Point> = HashMap::new();
        smallest_distance.reserve(vertices.len());
        smallest_distance_to.reserve(vertices.len());

        for i in 1..vertices.len() {
            let vert = vertices[i];
            let dist = Self::vsize2_with_unscale(vert - vertices[0]);
            smallest_distance.insert(vert, dist);
            smallest_distance_to.insert(vert, vertices[0]);
        }

        // Main Prim's algorithm loop
        // MinimumSpanningTree.cpp:54-103
        while result.len() < vertices.len() {
            // Find the closest vertex not yet in the tree
            // MinimumSpanningTree.cpp:55-64
            let mut min_dist = f64::MAX;
            let mut closest_point = None;

            for (point, &dist) in &smallest_distance {
                if dist < min_dist {
                    min_dist = dist;
                    closest_point = Some(*point);
                }
            }

            let closest_point = match closest_point {
                Some(p) => p,
                None => break,
            };

            // Add this point to the graph
            // MinimumSpanningTree.cpp:66-80
            let other_end = *smallest_distance_to.get(&closest_point).unwrap();

            result
                .entry(closest_point)
                .or_insert_with(Vec::new)
                .push(Edge::new(closest_point, other_end));

            result
                .entry(other_end)
                .or_insert_with(Vec::new)
                .push(Edge::new(other_end, closest_point));

            smallest_distance.remove(&closest_point);
            smallest_distance_to.remove(&closest_point);

            // Update distances of remaining points
            // MinimumSpanningTree.cpp:82-92
            let points_to_update: Vec<Point> = smallest_distance.keys().copied().collect();
            for point in points_to_update {
                let new_distance = Self::vsize2_with_unscale(closest_point - point);
                let old_distance = smallest_distance[&point];

                if new_distance < old_distance {
                    smallest_distance.insert(point, new_distance);
                    smallest_distance_to.insert(point, closest_point);
                }
            }
        }

        result
    }
}

impl Default for MinimumSpanningTree {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    #[test]
    fn test_empty_tree() {
        let tree = MinimumSpanningTree::from_vertices(vec![]);
        assert_eq!(tree.vertices().len(), 0);
        assert_eq!(tree.leaves().len(), 0);
    }

    #[test]
    fn test_single_vertex() {
        let p = Point::new(100, 200);
        let tree = MinimumSpanningTree::from_vertices(vec![p]);

        assert_eq!(tree.vertices().len(), 1);
        assert_eq!(tree.vertices()[0], p);
        assert_eq!(tree.leaves().len(), 1);
        assert_eq!(tree.leaves()[0], p);
        assert_eq!(tree.adjacent_nodes(p).len(), 0);
    }

    #[test]
    fn test_two_vertices() {
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1000000, 0); // 1mm apart

        let tree = MinimumSpanningTree::from_vertices(vec![p1, p2]);

        assert_eq!(tree.vertices().len(), 2);
        assert_eq!(tree.leaves().len(), 2); // Both are leaves in a 2-node tree

        let adj1 = tree.adjacent_nodes(p1);
        let adj2 = tree.adjacent_nodes(p2);

        assert_eq!(adj1.len(), 1);
        assert_eq!(adj2.len(), 1);
        assert_eq!(adj1[0], p2);
        assert_eq!(adj2[0], p1);
    }

    #[test]
    fn test_three_vertices_line() {
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1000000, 0); // 1mm
        let p3 = Point::new(2000000, 0); // 2mm

        let tree = MinimumSpanningTree::from_vertices(vec![p1, p2, p3]);

        assert_eq!(tree.vertices().len(), 3);

        // In a line, the middle point should have 2 connections
        let adj_middle = tree.adjacent_nodes(p2);
        assert_eq!(adj_middle.len(), 2);

        // The endpoints should have 1 connection each
        let leaves = tree.leaves();
        assert_eq!(leaves.len(), 2);
        assert!(leaves.contains(&p1));
        assert!(leaves.contains(&p3));
    }

    #[test]
    fn test_triangle() {
        // Equilateral triangle
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1000000, 0);
        let p3 = Point::new(500000, 866025); // approximately sqrt(3)/2

        let tree = MinimumSpanningTree::from_vertices(vec![p1, p2, p3]);

        assert_eq!(tree.vertices().len(), 3);

        // MST of triangle should have 2 edges total (3 nodes, 2 edges)
        let total_edges: usize = tree.adjacency_graph.values().map(|v| v.len()).sum();
        assert_eq!(total_edges, 4); // 4 because each edge is counted twice (bidirectional)

        // At least one vertex should have 2 connections
        let has_two_connections = tree.adjacency_graph.values().any(|edges| edges.len() == 2);
        assert!(has_two_connections);
    }

    #[test]
    fn test_square() {
        // Four corners of a square
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1000000, 0);
        let p3 = Point::new(1000000, 1000000);
        let p4 = Point::new(0, 1000000);

        let tree = MinimumSpanningTree::from_vertices(vec![p1, p2, p3, p4]);

        assert_eq!(tree.vertices().len(), 4);

        // MST should have 3 edges (4 nodes - 1)
        let total_edges: usize = tree.adjacency_graph.values().map(|v| v.len()).sum();
        assert_eq!(total_edges, 6); // 6 because each edge is counted twice

        // Should have exactly 2 leaves
        let leaves = tree.leaves();
        assert_eq!(leaves.len(), 2);
    }

    #[test]
    fn test_adjacent_nodes_nonexistent() {
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1000000, 0);
        let p_other = Point::new(5000000, 5000000);

        let tree = MinimumSpanningTree::from_vertices(vec![p1, p2]);

        // Query a point not in the tree
        let adj = tree.adjacent_nodes(p_other);
        assert_eq!(adj.len(), 0);
    }

    #[test]
    fn test_default() {
        let tree = MinimumSpanningTree::default();
        assert_eq!(tree.vertices().len(), 0);
    }

    #[test]
    fn test_new() {
        let tree = MinimumSpanningTree::new();
        assert_eq!(tree.vertices().len(), 0);
        assert_eq!(tree.leaves().len(), 0);
    }
}
