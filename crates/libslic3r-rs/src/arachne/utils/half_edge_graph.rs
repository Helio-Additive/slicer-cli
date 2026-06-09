//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! Faithful 1:1 port of Arachne/utils/HalfEdgeGraph.hpp
//!
//! C++ Reference: Arachne/utils/HalfEdgeGraph.hpp (header-only template class)

use std::collections::LinkedList;

use super::half_edge::HalfEdge;
use super::half_edge_node::HalfEdgeNode;

// HalfEdgeGraph.hpp:18 template<class node_data_t, class edge_data_t, class derived_node_t, class derived_edge_t> // types of data contained in nodes and edges
// HalfEdgeGraph.hpp:19 class HalfEdgeGraph
//
// The C++ class is templated on four parameters: `node_data_t`, `edge_data_t`,
// `derived_node_t` and `derived_edge_t`. In C++ the `derived_*` parameters allow
// CRTP-style derivation (e.g. `SkeletalTrapezoidationGraph`) where the stored
// `edge_t`/`node_t` are the *derived* types. In this port we mirror the sibling
// `HalfEdge<EdgeData, NodeData>` / `HalfEdgeNode<EdgeData, NodeData>` types, which
// already carry the node/edge data; the `derived_*` distinction collapses because
// Rust has no implicit upcasting and the concrete graph uses these directly.
//
// HalfEdgeGraph.hpp:22 using edge_t = derived_edge_t;
// HalfEdgeGraph.hpp:23 using node_t = derived_node_t;
//
// Container choice: the C++ members are `std::list`, NOT `std::vector`. This is
// load-bearing for the Arachne half-edge structure: `HalfEdge`/`HalfEdgeNode`
// hold raw pointers (`twin`/`next`/`prev`/`from`/`to`/`incident_edge`) into other
// elements of these lists. `std::list` guarantees that references and pointers to
// elements remain valid across insertion/removal of *other* elements. We therefore
// use `std::collections::LinkedList`, which provides the same pointer-stability
// guarantee, rather than `Vec` (whose reallocation on growth would invalidate the
// raw pointers held by the half-edge graph).
#[derive(Debug)]
pub struct HalfEdgeGraph<EdgeData, NodeData> {
    // HalfEdgeGraph.hpp:24 std::list<edge_t> edges;
    pub edges: LinkedList<HalfEdge<EdgeData, NodeData>>,

    // HalfEdgeGraph.hpp:25 std::list<node_t> nodes;
    pub nodes: LinkedList<HalfEdgeNode<EdgeData, NodeData>>,
}

impl<EdgeData, NodeData> HalfEdgeGraph<EdgeData, NodeData> {
    // HalfEdgeGraph.hpp:19 class HalfEdgeGraph (implicit default constructor;
    // both std::list members are default-constructed empty)
    pub fn new() -> Self {
        Self {
            edges: LinkedList::new(),
            nodes: LinkedList::new(),
        }
    }
}

impl<EdgeData, NodeData> Default for HalfEdgeGraph<EdgeData, NodeData> {
    fn default() -> Self {
        Self::new()
    }
}

// HalfEdgeGraph is safe to send between threads if the data types are Send.
// (The half-edge graph stores raw pointers internally; this matches the
// Send/Sync impls on the sibling HalfEdge / HalfEdgeNode types.)
unsafe impl<EdgeData: Send, NodeData: Send> Send for HalfEdgeGraph<EdgeData, NodeData> {}

// HalfEdgeGraph is safe to share between threads if the data types are Sync.
unsafe impl<EdgeData: Sync, NodeData: Sync> Sync for HalfEdgeGraph<EdgeData, NodeData> {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    #[derive(Debug, Clone, PartialEq)]
    struct TestEdgeData {
        weight: i32,
    }

    #[derive(Debug, Clone, PartialEq)]
    struct TestNodeData {
        id: usize,
    }

    #[test]
    fn test_half_edge_graph_creation() {
        // Test basic HalfEdgeGraph creation
        // C++ Reference: Arachne/utils/HalfEdgeGraph.hpp:19 (implicit constructor)
        let graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::new();

        assert_eq!(graph.edges.len(), 0);
        assert_eq!(graph.nodes.len(), 0);
    }

    #[test]
    fn test_push_node_and_edge() {
        // The C++ class has no member functions; callers manipulate the public
        // `edges` / `nodes` std::list members directly. Mirror that here.
        let mut graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::new();

        graph
            .nodes
            .push_back(HalfEdgeNode::new(TestNodeData { id: 1 }, Point::new(0, 0)));
        graph.nodes.push_back(HalfEdgeNode::new(
            TestNodeData { id: 2 },
            Point::new(100, 100),
        ));
        graph
            .edges
            .push_back(HalfEdge::new(TestEdgeData { weight: 10 }));

        assert_eq!(graph.nodes.len(), 2);
        assert_eq!(graph.edges.len(), 1);
        assert_eq!(graph.nodes.front().unwrap().data.id, 1);
        assert_eq!(graph.nodes.back().unwrap().data.id, 2);
        assert_eq!(graph.edges.front().unwrap().data.weight, 10);
    }

    #[test]
    fn test_default() {
        // Test Default trait implementation
        let graph: HalfEdgeGraph<TestEdgeData, TestNodeData> = Default::default();

        assert_eq!(graph.edges.len(), 0);
        assert_eq!(graph.nodes.len(), 0);
    }
}
