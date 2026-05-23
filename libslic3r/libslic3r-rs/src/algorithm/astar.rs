//! A* pathfinding algorithm.
//!
//! Implements the A* search algorithm for pathfinding on grids and graphs,
//! used for travel move optimization and toolpath planning.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::marker::PhantomData;

/// A* pathfinder for grid-based and graph-based pathfinding.
/// AStar.hpp:46
pub struct AStar<T: Clone + Eq + std::hash::Hash>(PhantomData<T>);

/// Implementation of A* pathfinding algorithm
/// AStar.hpp:46-152
impl<T: Clone + Eq + std::hash::Hash> AStar<T> {
    /// Find the shortest path between start and goal using A*.
    /// AStar.hpp:75
    pub fn find_path<F, G, H>(
        start: T,
        goal: T,
        neighbors: F,
        cost: G,
        heuristic: H,
    ) -> Option<Vec<T>>
    where
        F: Fn(&T) -> Vec<T>,
        G: Fn(&T, &T) -> f64,
        H: Fn(&T) -> f64,
    {
        // Initialize open set as priority queue
        // AStar.hpp:87
        let mut open_set = BinaryHeap::new();

        // Track path predecessors
        // AStar.hpp:79
        let mut came_from: HashMap<T, T> = HashMap::new();

        // Track cost from start to each node (g score)
        // AStar.hpp:52
        let mut g_score: HashMap<T, f64> = HashMap::new();

        // Track estimated total cost (f = g + h)
        // AStar.hpp:53
        let mut f_score: HashMap<T, f64> = HashMap::new();

        // Set initial g score for start node to 0
        // AStar.hpp:91
        g_score.insert(start.clone(), 0.0);

        // Calculate initial f score using heuristic
        // AStar.hpp:90
        f_score.insert(start.clone(), heuristic(&start));

        // Push start node into open set
        // AStar.hpp:92
        open_set.push(Node {
            position: start,
            f_score: heuristic(&goal),
        });

        // Main A* search loop
        // AStar.hpp:96
        while let Some(current) = open_set.pop() {
            // AStar.hpp:96
            // Check if we reached the goal
            // AStar.hpp:96-98
            if current.position == goal {
                // Reconstruct path from goal to start
                // AStar.hpp:141-150
                let mut path = vec![current.position];
                // AStar.hpp:145-147
                while let Some(prev) = came_from.get(path.last().unwrap()) {
                    // AStar.hpp:146
                    path.push(prev.clone());
                }
                // AStar.hpp:149
                path.reverse();
                // AStar.hpp:150
                return Some(path);
            }

            // Explore neighbors
            // AStar.hpp:104-137
            // AStar.hpp:104
            for neighbor in neighbors(&current.position) {
                // Calculate tentative g score
                // AStar.hpp:108-110
                let tentative_g = g_score[&current.position] + cost(&current.position, &neighbor);

                // Check if this path is better
                // AStar.hpp:119-133
                if tentative_g < *g_score.get(&neighbor).unwrap_or(&f64::INFINITY) {
                    // Update path and scores
                    // AStar.hpp:126
                    came_from.insert(neighbor.clone(), current.position.clone());
                    // AStar.hpp:127
                    g_score.insert(neighbor.clone(), tentative_g);
                    // AStar.hpp:128
                    let f = tentative_g + heuristic(&neighbor);
                    // AStar.hpp:129
                    f_score.insert(neighbor.clone(), f);
                    // AStar.hpp:130-133
                    open_set.push(Node {
                        position: neighbor,
                        f_score: f,
                    });
                }
            }
        }

        // No path found
        // AStar.hpp:152
        None
    }
}

/// Node structure for priority queue
/// AStar.hpp:46-58
#[derive(Clone)]
/// Node for A* priority queue with position and f-score
/// AStar.hpp:46-58
struct Node<T> {
    position: T,
    f_score: f64,
}

/// PartialEq implementation for Node comparison
/// AStar.hpp:46
impl<T: Eq> PartialEq for Node<T> {
    /// Compare nodes by position only
    /// AStar.hpp:46
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}

/// Eq implementation for Node
/// AStar.hpp:46
impl<T: Eq> Eq for Node<T> {}

/// PartialOrd implementation for priority queue ordering
/// AStar.hpp:83-85
impl<T: Eq> PartialOrd for Node<T> {
    /// Compare nodes by f score for priority queue
    /// AStar.hpp:83-85
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Ord implementation for min-heap ordering
/// AStar.hpp:83-85
impl<T: Eq> Ord for Node<T> {
    /// Compare nodes by f score (reversed for min-heap)
    /// AStar.hpp:83-85
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.partial_cmp(&self.f_score).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_astar_simple() {
        let start = 0i32;
        let goal = 5i32;

        let neighbors = |n: &i32| vec![n - 1, n + 1];
        let cost = |_a: &i32, _b: &i32| 1.0;
        let heuristic = |n: &i32| (*n - goal).abs() as f64;

        let path = AStar::find_path(start, goal, neighbors, cost, heuristic);
        assert!(path.is_some());
    }
}
