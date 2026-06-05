//! A* pathfinding algorithm implementation
//!
//! C++ Reference:
//! - BambuStudio/src/libslic3r/AStar.hpp
//!
//! This is a faithful 1:1 line-by-line port of the header-only `Slic3r::astar`
//! namespace. The C++ implementation is fully templated on a `Tracer` type that
//! describes the search domain (grid, point cloud, graph, ...) via the
//! `TracerTraits_` traits struct. Rust models this with the [`TracerTraits`]
//! trait.
//!
//! The open set is the ported [`MutablePriorityQueue`], constructed via
//! `make_mutable_priority_queue::<size_t, true>` in C++. Its comparison
//! predicate reads the `f()` score out of the node cache, and its index setter
//! writes the live `queue_id` back into the cache. To share the node cache
//! between the queue's closures and `search_route`, the cache is wrapped in an
//! `Rc<RefCell<..>>`, preserving the exact heap behaviour (same comparisons,
//! same `push` vs `update` decisions) as the C++ raw-reference closures.

use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

use crate::mutable_priority_queue::{make_mutable_priority_queue, INVALID_QUEUE_ID};

// AStar.hpp:11-12
// Borrowed from C++20
// template<class T> using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;
// (Not needed in Rust; the traits-based dispatch below handles cv/ref removal.)

/// Input interface for the Astar algorithm. Specialize this struct for a
/// particular type and implement all the 4 methods and specify the Node type
/// to register the new type for the astar implementation.
/// AStar.hpp:14-39 (TracerTraits_)
pub trait TracerTraits {
    // AStar.hpp:19-20
    /// The type of a node used by this tracer. Usually a point in space.
    type Node: Clone;

    // AStar.hpp:22-24
    /// Call fn for every new node reachable from node 'src'. fn should have the
    /// candidate node as its only argument.
    fn foreach_reachable<F>(&self, src: &Self::Node, f: F)
    where
        F: FnMut(&Self::Node) -> bool;

    // AStar.hpp:26-28
    /// Get the distance from node 'a' to node 'b'. This is sometimes referred
    /// to as the g value of a node in AStar context.
    fn distance(&self, a: &Self::Node, b: &Self::Node) -> f32;

    // AStar.hpp:30-35
    /// Get the estimated distance heuristic from node 'n' to the destination.
    /// This is referred to as the h value in AStar context.
    /// If node 'n' is the goal, this function should return a negative value.
    /// Note that this heuristic should be admissible (never bigger than the real
    /// cost) in order for Astar to work.
    fn goal_heuristic(&self, n: &Self::Node) -> f32;

    // AStar.hpp:37-38
    /// Return a unique identifier (hash) for node 'n'.
    fn unique_id(&self, n: &Self::Node) -> usize;
}

// AStar.hpp:42
// Helper definition to get the node type of a tracer
// template<class T> using TracerNodeT = typename TracerTraits_<remove_cvref_t<T>>::Node;
// In Rust this is `<T as TracerTraits>::Node`.

/// AStar.hpp:44
/// constexpr auto Unassigned = std::numeric_limits<size_t>::max();
pub const UNASSIGNED: usize = usize::MAX;

/// Queue node. Keeps track of scores g, and h
/// AStar.hpp:46-58 (template<class Tracer> struct QNode)
#[derive(Debug, Clone)]
pub struct QNode<N> {
    // AStar.hpp:48
    /// The actual node itself
    pub node: N,

    // AStar.hpp:49
    /// Position in the open queue or Unassigned if closed
    pub queue_id: usize,

    // AStar.hpp:50
    /// unique id of the parent or Unassigned
    pub parent: usize,

    // AStar.hpp:52
    pub g: f32,
    pub h: f32,
}

impl<N> QNode<N> {
    /// AStar.hpp:55-57
    /// QNode(TracerNodeT<Tracer> n = {}, size_t p = Unassigned,
    ///       float gval = std::numeric_limits<float>::infinity(), float hval = 0.f)
    ///     : node{std::move(n)}, parent{p}, queue_id{InvalidQueueID}, g{gval}, h{hval}
    pub fn new(n: N, p: usize, gval: f32, hval: f32) -> Self {
        Self {
            node: n,
            parent: p,
            queue_id: INVALID_QUEUE_ID,
            g: gval,
            h: hval,
        }
    }

    // AStar.hpp:53
    /// float f() const { return g + h; }
    pub fn f(&self) -> f32 {
        self.g + self.h
    }
}

/// Run the AStar algorithm on a tracer implementation.
/// The 'tracer' argument encapsulates the domain (grid, point cloud, etc...)
/// The 'source' argument is the starting node.
/// The 'out' argument is the output iterator into which the output nodes are
/// written. For performance reasons, the order is reverse, from the destination
/// to the source -- (destination included, source is not).
/// The 'cached_nodes' argument is an optional associative container to hold a
/// QNode entry for each visited node. Any compatible container can be used
/// (like std::map or maps with different allocators, even a sufficiently large
/// std::vector).
///
/// Note that no destination node is given in the signature. The tracer's
/// goal_heuristic() method should return a negative value if a node is a
/// destination node.
///
/// AStar.hpp:60-153 (template ... bool search_route(...))
///
/// The C++ writes the result through an output iterator in reverse order
/// (destination first, source excluded). This Rust port collects that same
/// reverse-order sequence into `out` (push order == C++ `*out = ...; ++out;`)
/// and returns the C++ boolean as `out.is_empty() == false` is *not* used;
/// the boolean is returned directly.
pub fn search_route<T>(
    tracer: &T,
    source: &T::Node,
    out: &mut Vec<T::Node>,
    cached_nodes: &mut HashMap<usize, QNode<T::Node>>,
) -> bool
where
    T: TracerTraits,
    T::Node: Clone,
{
    // AStar.hpp:77-79
    // using Node = TracerNodeT<Tracer>;
    // using QNode = QNode<Tracer>;
    // using TracerTraits = TracerTraits_<remove_cvref_t<Tracer>>;
    //
    // The node cache is shared by reference between the queue's closures
    // (LessPred / index setter) and this routine, matching the C++ semantics
    // where both capture `cached_nodes` by reference. The caller passes the map
    // by mutable reference (C++ passes it by `&`); the final state is written
    // back before returning so the caller can reconstruct paths from it
    // (JumpPointSearch.cpp:228-241 fallback).
    let cached_nodes_returned: &mut HashMap<usize, QNode<T::Node>> = cached_nodes;
    let cached_nodes: Rc<RefCell<HashMap<usize, QNode<T::Node>>>> =
        Rc::new(RefCell::new(std::mem::take(cached_nodes_returned)));

    // AStar.hpp:81-85
    // struct LessPred { NodeMap &m;
    //     bool operator()(size_t node_a, size_t node_b) { return m[node_a].f() < m[node_b].f(); } };
    let less_pred = {
        let m = Rc::clone(&cached_nodes);
        move |node_a: &usize, node_b: &usize| -> bool {
            let map = m.borrow();
            map[node_a].f() < map[node_b].f()
        }
    };

    // AStar.hpp:87
    // auto qopen = make_mutable_priority_queue<size_t, true>(
    //     [&cached_nodes](size_t el, size_t qidx) { cached_nodes[el].queue_id = qidx; },
    //     LessPred{cached_nodes});
    let index_setter = {
        let m = Rc::clone(&cached_nodes);
        move |el: &usize, qidx: usize| {
            m.borrow_mut().get_mut(el).unwrap().queue_id = qidx;
        }
    };
    let mut qopen = make_mutable_priority_queue::<usize, _, _>(true, index_setter, less_pred);

    // AStar.hpp:89-92
    // QNode initial{source, /*parent = */ Unassigned, /*g = */ 0.f};
    // size_t source_id = TracerTraits::unique_id(tracer, source);
    // cached_nodes[source_id] = initial;
    // qopen.push(source_id);
    let initial = QNode::new(source.clone(), UNASSIGNED, 0.0, 0.0);
    let source_id = tracer.unique_id(source);
    cached_nodes.borrow_mut().insert(source_id, initial);
    qopen.push(source_id);

    // AStar.hpp:94
    // size_t goal_id = TracerTraits::goal_heuristic(tracer, source) < 0.f ? source_id : Unassigned;
    let mut goal_id = if tracer.goal_heuristic(source) < 0.0 {
        source_id
    } else {
        UNASSIGNED
    };

    // AStar.hpp:96
    // while (goal_id == Unassigned && !qopen.empty()) {
    while goal_id == UNASSIGNED && !qopen.is_empty() {
        // AStar.hpp:97-99
        // size_t q_id = qopen.top();
        // qopen.pop();
        // QNode &q = cached_nodes[q_id];
        let q_id = *qopen.top().unwrap();
        qopen.pop();

        // The current node's scalar fields are snapshotted here. `q` is a
        // reference into `cached_nodes` in C++; the only fields read inside the
        // closure below are `q.node`, `q.g` and the captured `q_id`.
        let (q_node, q_g) = {
            let map = cached_nodes.borrow();
            let q = &map[&q_id];
            // AStar.hpp:102
            // This should absolutely be initialized in the cache already
            // assert(!std::isinf(q.g));
            debug_assert!(!q.g.is_infinite());
            (q.node.clone(), q.g)
        };

        // AStar.hpp:104
        // TracerTraits::foreach_reachable(tracer, q.node, [&](const Node &succ_nd) {
        tracer.foreach_reachable(&q_node, |succ_nd| {
            // AStar.hpp:105
            // if (goal_id != Unassigned) return true;
            if goal_id != UNASSIGNED {
                return true;
            }

            // AStar.hpp:107-110
            // float  h       = TracerTraits::goal_heuristic(tracer, succ_nd);
            // float  dst     = TracerTraits::distance(tracer, q.node, succ_nd);
            // size_t succ_id = TracerTraits::unique_id(tracer, succ_nd);
            // QNode  qsucc_nd{succ_nd, q_id, q.g + dst, h};
            let h = tracer.goal_heuristic(succ_nd);
            let dst = tracer.distance(&q_node, succ_nd);
            let succ_id = tracer.unique_id(succ_nd);
            let qsucc_nd = QNode::new(succ_nd.clone(), q_id, q_g + dst, h);

            // AStar.hpp:112
            // if (h < 0.f) {
            if h < 0.0 {
                // AStar.hpp:113-114
                // goal_id               = succ_id;
                // cached_nodes[succ_id] = qsucc_nd;
                goal_id = succ_id;
                cached_nodes.borrow_mut().insert(succ_id, qsucc_nd);
            } else {
                // AStar.hpp:116-117
                // If succ_id is not in cache, it gets created with g = infinity
                // QNode &prev_nd = cached_nodes[succ_id];
                //
                // Snapshot the previous g (defaulting to +inf as the default
                // QNode constructor does) before any mutation.
                let prev_g = {
                    let map = cached_nodes.borrow();
                    match map.get(&succ_id) {
                        Some(prev_nd) => prev_nd.g,
                        None => f32::INFINITY,
                    }
                };

                // AStar.hpp:119
                // if (qsucc_nd.g < prev_nd.g) {
                if qsucc_nd.g < prev_g {
                    // new route is better, apply it:

                    // AStar.hpp:122-123
                    // Save the old queue id, it would be lost after the next line
                    // size_t queue_id = prev_nd.queue_id;
                    let queue_id = {
                        let map = cached_nodes.borrow();
                        match map.get(&succ_id) {
                            Some(prev_nd) => prev_nd.queue_id,
                            None => INVALID_QUEUE_ID,
                        }
                    };

                    // AStar.hpp:125-126
                    // The cache needs to be updated either way
                    // prev_nd = qsucc_nd;
                    cached_nodes.borrow_mut().insert(succ_id, qsucc_nd);

                    // AStar.hpp:128-132
                    // if (queue_id == InvalidQueueID)
                    //     // was in closed or unqueued, rescheduling
                    //     qopen.push(succ_id);
                    // else // was in open, updating
                    //     qopen.update(queue_id);
                    if queue_id == INVALID_QUEUE_ID {
                        // was in closed or unqueued, rescheduling
                        qopen.push(succ_id);
                    } else {
                        // was in open, updating
                        qopen.update(queue_id);
                    }
                }
            }

            // AStar.hpp:136
            // return goal_id != Unassigned;
            goal_id != UNASSIGNED
        });
    }

    // AStar.hpp:140-150
    // Write the output, do not reverse. Clients can do so if they need to.
    // if (goal_id != Unassigned) {
    //     const QNode *q = &cached_nodes[goal_id];
    //     while (q->parent != Unassigned) {
    //         assert(!std::isinf(q->g)); // Uninitialized nodes are NOT allowed
    //         *out = q->node;
    //         ++out;
    //         q = &cached_nodes[q->parent];
    //     }
    // }
    if goal_id != UNASSIGNED {
        let map = cached_nodes.borrow();
        let mut q = &map[&goal_id];
        while q.parent != UNASSIGNED {
            debug_assert!(!q.g.is_infinite()); // Uninitialized nodes are NOT allowed

            out.push(q.node.clone());
            q = &map[&q.parent];
        }
    }

    // Write the final cache state back to the caller's map (C++ passes the map
    // by `&`, so it remains live and populated for the caller after the call).
    *cached_nodes_returned = cached_nodes.borrow().clone();

    // AStar.hpp:152
    // return goal_id != Unassigned;
    goal_id != UNASSIGNED
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
                    && f(neighbor)
                {
                    break;
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
        let mut path = Vec::new();
        let mut cache = HashMap::new();
        let found = search_route(&tracer, &source, &mut path, &mut cache);

        assert!(found);
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
        let mut path = Vec::new();
        let mut cache = HashMap::new();
        let found = search_route(&tracer, &source, &mut path, &mut cache);

        // When source is goal, search succeeds but path is empty (parent is
        // Unassigned), matching C++ where the while loop never runs.
        assert!(found);
        assert!(path.is_empty());
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
        let mut path = Vec::new();
        let mut cache = HashMap::new();
        let found = search_route(&tracer, &source, &mut path, &mut cache);

        assert!(!found); // No path possible
        assert!(path.is_empty());
    }
}
