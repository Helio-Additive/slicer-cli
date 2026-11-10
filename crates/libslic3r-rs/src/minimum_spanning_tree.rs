//! Minimum Spanning Tree implementation using Prim's algorithm
//!
//! C++ Reference: MinimumSpanningTree.hpp, MinimumSpanningTree.cpp
//!
//! Implements Prim's algorithm to compute Minimum Spanning Trees (MST).
//! The minimum spanning tree is always computed from a clique of vertices.
//! MinimumSpanningTree.hpp:13-17

// MinimumSpanningTree.cpp:1-5
use crate::geometry::Point;
use crate::unscale;
use std::collections::HashMap;

// MinimumSpanningTree.cpp:10
// #define unscale_(val) ((val) * SCALING_FACTOR)
//
// In C++ SCALING_FACTOR == 0.00001, so unscale_(val) == val * 0.00001 == val / 100000.
// In this crate `unscale(v) == v / SCALING_FACTOR` with SCALING_FACTOR == 100_000.0,
// which yields the same value. We reuse `crate::unscale` for `unscale_`.

/// MinimumSpanningTree.cpp:12-15
#[inline]
fn dot_with_unscale(a: Point, b: Point) -> f64 {
    unscale(a.x()) * unscale(b.x()) + unscale(a.y()) * unscale(b.y())
}

/// MinimumSpanningTree.cpp:17-20
#[inline]
fn vsize2_with_unscale(pt: Point) -> f64 {
    dot_with_unscale(pt, pt)
}

/// Represents an edge of the tree.
///
/// While edges are meant to be undirected, these do have a start and end
/// point.
/// MinimumSpanningTree.hpp:20-36
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Edge {
    /// The point at which this edge starts.
    /// MinimumSpanningTree.hpp:30
    start: Point,

    /// The point at which this edge ends.
    /// MinimumSpanningTree.hpp:35
    end: Point,
}

/// MinimumSpanningTree.hpp:63
type AdjacencyGraphT = HashMap<Point, Vec<Edge>>;

/// Implements Prim's algorithm to compute Minimum Spanning Trees (MST).
///
/// The minimum spanning tree is always computed from a clique of vertices.
/// MinimumSpanningTree.hpp:18-74
#[derive(Debug, Clone, Default)]
pub struct MinimumSpanningTree {
    /// MinimumSpanningTree.hpp:63-64
    adjacency_graph: AdjacencyGraphT,
}

impl MinimumSpanningTree {
    /// MinimumSpanningTree() = default;
    /// MinimumSpanningTree.hpp:38
    pub fn new() -> Self {
        Self {
            adjacency_graph: AdjacencyGraphT::new(),
        }
    }

    /// Constructs a minimum spanning tree that spans all given vertices.
    /// MinimumSpanningTree.hpp:39-42
    /// MinimumSpanningTree.cpp:22-25
    pub fn from_vertices(vertices: Vec<Point>) -> Self {
        // : adjacency_graph(prim(vertices))
        // Just copy over the fields.
        Self {
            adjacency_graph: Self::prim(vertices),
        }
    }

    /// Computes the edges of a minimum spanning tree using Prim's algorithm.
    ///
    /// \param vertices The vertices to span.
    /// \return An adjacency graph with for each point one or more edges.
    /// MinimumSpanningTree.cpp:27-103
    fn prim(vertices: Vec<Point>) -> AdjacencyGraphT {
        // MinimumSpanningTree.cpp:29
        let mut result = AdjacencyGraphT::new();
        // MinimumSpanningTree.cpp:30-33
        if vertices.is_empty() {
            return result; // No vertices, so we can't create edges either.
        }
        // If there's only one vertex, we can't go creating any edges so just add the point to the adjacency list with no
        // edges
        // MinimumSpanningTree.cpp:36-42
        if vertices.len() == 1 {
            // unordered_map::operator[]() will construct an empty vector in place for us when we try and access an element
            // that doesnt exist
            result.entry(vertices[0]).or_default();
            return result;
        }
        // MinimumSpanningTree.cpp:43
        result.reserve(vertices.len());
        // MinimumSpanningTree.cpp:44
        let vertices_list: Vec<Point> = vertices.clone();

        // MinimumSpanningTree.cpp:46-49
        // smallest_distance: The shortest distance to the current tree.
        // smallest_distance_to: Which point the shortest distance goes towards.
        //
        // NOTE: C++ keys these maps on `const Point*` (the address of the entries in
        // vertices_list). Two coordinate-equal vertices would therefore be distinct
        // candidates. We key on the `Point` value, which is the natural Rust
        // translation and is equivalent for a clique of distinct vertices.
        let mut smallest_distance: HashMap<Point, f64> = HashMap::new();
        let mut smallest_distance_to: HashMap<Point, Point> = HashMap::new();
        smallest_distance.reserve(vertices_list.len());
        smallest_distance_to.reserve(vertices_list.len());
        // MinimumSpanningTree.cpp:50-55
        for vertex_index in 1..vertices_list.len() {
            let vert = vertices_list[vertex_index];
            smallest_distance.insert(vert, vsize2_with_unscale(vert - vertices_list[0]));
            smallest_distance_to.insert(vert, vertices_list[0]);
        }

        // MinimumSpanningTree.cpp:57
        // All of the vertices need to be in the tree at the end.
        while result.len() < vertices_list.len() {
            // Choose the closest vertex to connect to that is not yet in the tree.
            // This search is O(V) right now, which can be made down to O(log(V)). This reduces the overall time complexity from O(V*V) to O(V*log(E)).
            // However that requires an implementation of a heap that supports the decreaseKey operation, which is not in the std library.
            // TODO: Implement this?
            // MinimumSpanningTree.cpp:59-71
            //
            // std::min_element with comparator:
            //   if (a.second != b.second) return a.second < b.second;
            //   if (a.first->x() != b.first->x()) return a.first->x() < b.first->x();
            //   return a.first->y() < b.first->y();
            // i.e. order by (distance, x, y) and pick the minimum.
            let mut closest: Option<(Point, f64)> = None;
            for (&point, &dist) in &smallest_distance {
                let take = match closest {
                    None => true,
                    Some((best_point, best_dist)) => {
                        // less_than(candidate, current_best) under the C++ comparator;
                        // if candidate is strictly smaller, it becomes the new minimum.
                        if dist != best_dist {
                            dist < best_dist
                        } else if point.x() != best_point.x() {
                            point.x() < best_point.x()
                        } else {
                            point.y() < best_point.y()
                        }
                    }
                };
                if take {
                    closest = Some((point, dist));
                }
            }

            // Add this point to the graph and remove it from the candidates.
            // MinimumSpanningTree.cpp:73-87
            let closest_point = closest.expect("smallest_distance is non-empty in the loop").0;
            let other_end = *smallest_distance_to
                .get(&closest_point)
                .expect("smallest_distance_to has closest_point");
            // result[*closest_point].push_back({*closest_point, other_end});
            result.entry(closest_point).or_default().push(Edge {
                start: closest_point,
                end: other_end,
            });
            // result[other_end].push_back({other_end, *closest_point});
            result.entry(other_end).or_default().push(Edge {
                start: other_end,
                end: closest_point,
            });
            // Remove it so we don't check for these points again.
            smallest_distance.remove(&closest_point);
            smallest_distance_to.remove(&closest_point);

            // Update the distances of all points that are not in the graph.
            // MinimumSpanningTree.cpp:89-99
            let candidates: Vec<Point> = smallest_distance.keys().copied().collect();
            for point in candidates {
                let new_distance = vsize2_with_unscale(closest_point - point);
                let old_distance = smallest_distance[&point];
                if new_distance < old_distance {
                    // New point is closer.
                    smallest_distance.insert(point, new_distance);
                    smallest_distance_to.insert(point, closest_point);
                }
            }
        }

        // MinimumSpanningTree.cpp:101
        result
    }

    /// Gets the nodes that are adjacent to the specified node.
    /// \return A list of nodes that are adjacent.
    /// MinimumSpanningTree.cpp:105-116
    pub fn adjacent_nodes(&self, node: Point) -> Vec<Point> {
        let mut result: Vec<Point> = Vec::new();
        if let Some(edges) = self.adjacency_graph.get(&node) {
            for e in edges {
                result.push(if e.start == node { e.end } else { e.start });
            }
        }
        result
    }

    /// Gets the leaves of the tree.
    /// \return A list of nodes that are all leaves of the tree.
    /// MinimumSpanningTree.cpp:118-129
    pub fn leaves(&self) -> Vec<Point> {
        let mut result: Vec<Point> = Vec::new();
        for (node, edges) in &self.adjacency_graph {
            if edges.len() <= 1 {
                result.push(*node);
            }
        }
        result
    }

    /// Gets all vertices of the tree.
    /// \return A list of vertices of the tree.
    /// MinimumSpanningTree.cpp:131-138
    pub fn vertices(&self) -> Vec<Point> {
        let mut result: Vec<Point> = Vec::new();
        for node in self.adjacency_graph.keys() {
            result.push(*node);
        }
        result
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
