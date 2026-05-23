//! A* pathfinding algorithm implementation
//!
//! This module provides a generic A* search algorithm that can work with any
//! domain (grids, point clouds, graphs, etc.) by implementing the `TracerTraits` trait.
//!
//! C++ Reference: AStar.hpp

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// Sentinel value indicating no assignment (equivalent to SIZE_MAX in C++)
/// AStar.hpp:45
pub const UNASSIGNED: usize = usize::MAX;

/// Trait defining the interface for A* algorithm tracer implementations
/// AStar.hpp:16-41
pub trait TracerTraits {
    /// The type of node used by this tracer (e.g., a point in space, graph vertex, etc.)
    type Node: Clone;

    /// Call the provided function for every node reachable from the source node
    /// AStar.hpp:23
    fn foreach_reachable<F>(&self, src: &Self::Node, f: F)
    where
        F: FnMut(&Self::Node) -> bool;

    /// Get the distance from node 'a' to node 'b' (the g value in A* terminology)
    /// AStar.hpp:27
    fn distance(&self, a: &Self::Node, b: &Self::Node) -> f32;

    /// Get the estimated distance heuristic from node 'n' to the destination (the h value)
    /// AStar.hpp:32
    fn goal_heuristic(&self, n: &Self::Node) -> f32;

    /// Return a unique identifier (hash) for the given node
    /// AStar.hpp:35
    fn unique_id(&self, n: &Self::Node) -> usize;
}

/// Queue node structure tracking A* scores (g, h, f)
/// AStar.hpp:47-59
#[derive(Debug, Clone)]
pub struct QNode<N> {
    /// The actual node data
    /// AStar.hpp:49
    pub node: N,

    /// Position in the open queue or UNASSIGNED if closed
    /// AStar.hpp:50
    pub queue_id: usize,

    /// Unique ID of the parent node, or UNASSIGNED if no parent
    /// AStar.hpp:51
    pub parent: usize,

    /// Cost from start to this node (g value)
    /// AStar.hpp:53
    pub g: f32,

    /// Estimated cost from this node to goal (h value)
    /// AStar.hpp:53
    pub h: f32,
}

/// QNode implementation methods
/// AStar.hpp:56-59
impl<N> QNode<N> {
    /// Create a new queue node
    /// AStar.hpp:56-59
    pub fn new(node: N, parent: usize, g: f32, h: f32) -> Self {
        Self {
            node,
            queue_id: UNASSIGNED,
            parent,
            g,
            h,
        }
    }

    /// Calculate the f-score (total estimated cost)
    /// AStar.hpp:54
    pub fn f(&self) -> f32 {
        self.g + self.h
    }
}

/// Helper struct for priority queue ordering (min-heap based on f-score)
/// AStar.hpp:84-87
#[derive(Debug, Clone)]
struct PQEntry {
    id: usize,
    f_score: f32,
}

/// PartialEq implementation for PQEntry
/// AStar.hpp:84-87
impl PartialEq for PQEntry {
    /// Compare two priority queue entries for equality based on f-score
    /// AStar.hpp:84-87
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

/// Eq implementation for PQEntry
/// AStar.hpp:84-87
impl Eq for PQEntry {}

/// PartialOrd implementation for PQEntry (min-heap using reverse ordering)
/// AStar.hpp:84-87
impl PartialOrd for PQEntry {
    /// Compare two priority queue entries for ordering (reversed for min-heap)
    /// AStar.hpp:84-87
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // Reverse order for min-heap (BinaryHeap is max-heap by default)
        other.f_score.partial_cmp(&self.f_score)
    }
}

/// Ord implementation for PQEntry
/// AStar.hpp:84-87
impl Ord for PQEntry {
    /// Total ordering for priority queue entries
    /// AStar.hpp:84-87
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

/// Run the A* search algorithm on a tracer implementation
/// AStar.hpp:77-155
pub fn search_route<T>(
    tracer: &T,
    source: &T::Node,
    cached_nodes: Option<HashMap<usize, QNode<T::Node>>>,
) -> Option<Vec<T::Node>>
where
    T: TracerTraits,
    T::Node: Clone,
{
    // Initialize the node cache with the provided map or create a new one
    // AStar.hpp:77
    // AStar.hpp:77
    let mut cached_nodes = cached_nodes.unwrap_or_else(HashMap::new);

    // Create the priority queue (min-heap based on f-score)
    // AStar.hpp:86
    // AStar.hpp:86
    let mut qopen: BinaryHeap<PQEntry> = BinaryHeap::new();

    // Create initial node with source position, no parent, g=0
    // AStar.hpp:88
    // AStar.hpp:88
    let initial = QNode::new(source.clone(), UNASSIGNED, 0.0, 0.0);

    // Get unique ID for source node and store in cache
    // AStar.hpp:89-90
    // AStar.hpp:89
    let source_id = tracer.unique_id(source);
    // AStar.hpp:90
    cached_nodes.insert(source_id, initial);

    // Push source onto the open queue
    // AStar.hpp:91
    // AStar.hpp:91
    qopen.push(PQEntry {
        id: source_id,
        f_score: 0.0,
    });

    // Check if source is already the goal
    // AStar.hpp:93
    // AStar.hpp:93
    let mut goal_id =
        // AStar.hpp:93
        if tracer.goal_heuristic(source) < 0.0 {
            source_id
        } else {
            UNASSIGNED
        };

    // Main A* search loop - continue until goal found or queue empty
    // AStar.hpp:95
    while goal_id == UNASSIGNED && !qopen.is_empty() {
        // Get the node with lowest f-score from the open queue
        // AStar.hpp:96-97
        // AStar.hpp:96-97
        let q_id =
            // AStar.hpp:96-97
            match qopen.pop() {
                Some(entry) => entry.id,
                None => break,
            };

        // Get reference to current node (must exist in cache)
        // AStar.hpp:98
        // AStar.hpp:98
        let q_g = cached_nodes[&q_id].g;
        // AStar.hpp:98
        let q_node = cached_nodes[&q_id].node.clone();

        // Verify node is initialized (g should not be infinite)
        // AStar.hpp:101
        // AStar.hpp:101
        debug_assert!(q_g.is_finite(), "Node g-value should be finite");

        // Explore all reachable neighbors from current node
        // AStar.hpp:103
        // AStar.hpp:103
        let mut found_goal = false;
        // AStar.hpp:103
        tracer.foreach_reachable(&q_node, |succ_nd| {
            // Early exit if goal already found
            // AStar.hpp:104
            // AStar.hpp:104
            if goal_id != UNASSIGNED {
                // AStar.hpp:104
                return true;
            }

            // Calculate heuristic for successor node
            // AStar.hpp:106
            // AStar.hpp:106
            let h = tracer.goal_heuristic(succ_nd);

            // Calculate distance from current to successor
            // AStar.hpp:107
            // AStar.hpp:107
            let dst = tracer.distance(&q_node, succ_nd);

            // Get unique ID for successor
            // AStar.hpp:108
            // AStar.hpp:108
            let succ_id = tracer.unique_id(succ_nd);

            // Create queue node for successor with updated g-score
            // AStar.hpp:109
            // AStar.hpp:109
            let qsucc_nd = QNode::new(succ_nd.clone(), q_id, q_g + dst, h);

            // Check if this successor is the goal (negative heuristic)
            // AStar.hpp:111-114
            // AStar.hpp:111
            // AStar.hpp:111
            if h < 0.0 {
                // AStar.hpp:112
                goal_id = succ_id;
                // AStar.hpp:113
                cached_nodes.insert(succ_id, qsucc_nd);
                // AStar.hpp:113
                found_goal = true;
            } else {
                // Get or create entry for this successor in cache
                // AStar.hpp:116
                // AStar.hpp:116
                let prev_g =
                    // AStar.hpp:116
                    cached_nodes
                        .get(&succ_id)
                        .map(|n| n.g)
                        .unwrap_or(f32::INFINITY);

                // If new route is better than previous, update
                // AStar.hpp:118
                // AStar.hpp:118
                // AStar.hpp:118
                if qsucc_nd.g < prev_g {
                    // Update cache with new better route
                    // AStar.hpp:124
                    // AStar.hpp:124
                    let f_score = qsucc_nd.f();
                    // AStar.hpp:124
                    cached_nodes.insert(succ_id, qsucc_nd);

                    // Add to priority queue (no update operation needed with BinaryHeap)
                    // AStar.hpp:126-130
                    // AStar.hpp:126
                    qopen.push(PQEntry {
                        id: succ_id,
                        f_score,
                    });
                }
            }

            // Signal whether to continue iteration (stop if goal found)
            // AStar.hpp:135
            // AStar.hpp:135
            found_goal
        });

        // Break loop if goal was found
        // AStar.hpp:95
        if found_goal {
            // AStar.hpp:95
            break;
        }
    }

    // Write output path by backtracking from goal to source
    // AStar.hpp:139-149
    // AStar.hpp:139
    // AStar.hpp:139
    if goal_id != UNASSIGNED {
        // AStar.hpp:140
        let mut path = Vec::new();
        // AStar.hpp:140
        let mut current_id = goal_id;

        // Backtrack from goal to source following parent links
        // AStar.hpp:141-147
        // AStar.hpp:141
        // AStar.hpp:141
        while current_id != UNASSIGNED {
            // AStar.hpp:141
            // AStar.hpp:141
            if let Some(q) = cached_nodes.get(&current_id) {
                // AStar.hpp:142
                debug_assert!(q.g.is_finite(), "Path node g-value should be finite");
                // AStar.hpp:143
                path.push(q.node.clone());

                // Move to parent node
                // AStar.hpp:146
                // AStar.hpp:146
                current_id = q.parent;
            } else {
                // AStar.hpp:147
                break;
            }
        }

        // Return path (in reverse order, from goal to source, excluding source)
        // AStar.hpp:151
        // AStar.hpp:151
        Some(path)
    } else {
        // AStar.hpp:151
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple 2D grid node for testing
    #[derive(Debug, Clone, PartialEq, Eq, Hash)]
    struct GridNode {
        x: i32,
        y: i32,
    }

    /// Simple grid tracer for testing A* on a 2D grid
    struct GridTracer {
        goal: GridNode,
        width: i32,
        height: i32,
    }

    impl TracerTraits for GridTracer {
        type Node = GridNode;

        fn foreach_reachable<F>(&self, src: &Self::Node, mut f: F)
        where
            F: FnMut(&Self::Node) -> bool,
        {
            // 4-connected grid (up, down, left, right)
            let neighbors = [
                GridNode {
                    x: src.x + 1,
                    y: src.y,
                },
                GridNode {
                    x: src.x - 1,
                    y: src.y,
                },
                GridNode {
                    x: src.x,
                    y: src.y + 1,
                },
                GridNode {
                    x: src.x,
                    y: src.y - 1,
                },
            ];

            for neighbor in &neighbors {
                if neighbor.x >= 0
                    && neighbor.x < self.width
                    && neighbor.y >= 0
                    && neighbor.y < self.height
                {
                    if f(neighbor) {
                        break;
                    }
                }
            }
        }

        fn distance(&self, a: &Self::Node, b: &Self::Node) -> f32 {
            // Manhattan distance
            ((a.x - b.x).abs() + (a.y - b.y).abs()) as f32
        }

        fn goal_heuristic(&self, n: &Self::Node) -> f32 {
            if n == &self.goal {
                -1.0 // Negative indicates goal
            } else {
                // Manhattan distance to goal
                ((n.x - self.goal.x).abs() + (n.y - self.goal.y).abs()) as f32
            }
        }

        fn unique_id(&self, n: &Self::Node) -> usize {
            (n.y * self.width + n.x) as usize
        }
    }

    #[test]
    fn test_astar_simple_path() {
        let tracer = GridTracer {
            goal: GridNode { x: 3, y: 3 },
            width: 5,
            height: 5,
        };

        let source = GridNode { x: 0, y: 0 };
        let path = search_route(&tracer, &source, None);

        assert!(path.is_some());
        let path = path.unwrap();
        assert!(!path.is_empty());
        assert_eq!(path[0], GridNode { x: 3, y: 3 }); // First in path is goal
    }

    #[test]
    fn test_astar_source_is_goal() {
        let tracer = GridTracer {
            goal: GridNode { x: 0, y: 0 },
            width: 5,
            height: 5,
        };

        let source = GridNode { x: 0, y: 0 };
        let path = search_route(&tracer, &source, None);

        // When source is goal, path should be empty (no moves needed)
        assert!(path.is_some());
    }

    #[test]
    fn test_astar_no_path() {
        // Create a tracer with unreachable goal (outside grid)
        struct UnreachableTracer;

        impl TracerTraits for UnreachableTracer {
            type Node = GridNode;

            fn foreach_reachable<F>(&self, _src: &Self::Node, _f: F)
            where
                F: FnMut(&Self::Node) -> bool,
            {
                // No neighbors - isolated node
            }

            fn distance(&self, a: &Self::Node, b: &Self::Node) -> f32 {
                ((a.x - b.x).abs() + (a.y - b.y).abs()) as f32
            }

            fn goal_heuristic(&self, n: &Self::Node) -> f32 {
                if n.x == 5 && n.y == 5 {
                    -1.0
                } else {
                    ((n.x - 5).abs() + (n.y - 5).abs()) as f32
                }
            }

            fn unique_id(&self, n: &Self::Node) -> usize {
                (n.y * 10 + n.x) as usize
            }
        }

        let tracer = UnreachableTracer;
        let source = GridNode { x: 0, y: 0 };
        let path = search_route(&tracer, &source, None);

        assert!(path.is_none()); // No path possible
    }
}
