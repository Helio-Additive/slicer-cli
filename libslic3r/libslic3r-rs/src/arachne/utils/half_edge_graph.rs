//! Half-edge graph data structure for Arachne wall generation
//!
//! C++ Reference:
//! - Arachne/utils/HalfEdgeGraph.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use super::half_edge::HalfEdge;
use super::half_edge_node::HalfEdgeNode;

/// Half-edge graph structure for representing planar graphs
///
/// This is the Rust equivalent of C++ template class HalfEdgeGraph<node_data_t, edge_data_t, derived_node_t, derived_edge_t>
///
/// C++ Reference: Arachne/utils/HalfEdgeGraph.hpp (template class HalfEdgeGraph)
///
/// The half-edge data structure is used to represent planar graphs efficiently.
/// It stores all edges and nodes in lists, with edges stored as pairs of half-edges.
///
/// In C++, this uses std::list for O(1) insertion/removal anywhere.
/// In Rust, we use Vec which is more cache-friendly but has O(n) removal.
/// For the Arachne algorithm's usage pattern (mostly sequential access), Vec is acceptable.
#[derive(Debug)]
pub struct HalfEdgeGraph<EdgeData, NodeData> {
    /// List of all edges in the graph
    /// C++: std::list<edge_t> edges;
    pub edges: Vec<HalfEdge<EdgeData, NodeData>>,

    /// List of all nodes in the graph
    /// C++: std::list<node_t> nodes;
    pub nodes: Vec<HalfEdgeNode<EdgeData, NodeData>>,
}

impl<EdgeData, NodeData> HalfEdgeGraph<EdgeData, NodeData> {
    /// Create a new empty HalfEdgeGraph
    /// C++ Reference: Implicit default constructor
    pub fn new() -> Self {
        Self {
            edges: Vec::new(),
            nodes: Vec::new(),
        }
    }

    /// Create a new HalfEdgeGraph with preallocated capacity
    ///
    /// This is useful when you know approximately how many edges and nodes you'll need,
    /// to avoid repeated allocations.
    pub fn with_capacity(edge_capacity: usize, node_capacity: usize) -> Self {
        Self {
            edges: Vec::with_capacity(edge_capacity),
            nodes: Vec::with_capacity(node_capacity),
        }
    }

    /// Add a new edge to the graph
    ///
    /// Returns a mutable reference to the newly added edge.
    ///
    /// Note: In C++, this would return an iterator to std::list.
    /// In Rust, we return a mutable reference with lifetime tied to the graph.
    pub fn add_edge(
        &mut self,
        edge: HalfEdge<EdgeData, NodeData>,
    ) -> &mut HalfEdge<EdgeData, NodeData> {
        self.edges.push(edge);
        self.edges.last_mut().unwrap()
    }

    /// Add a new node to the graph
    ///
    /// Returns a mutable reference to the newly added node.
    pub fn add_node(
        &mut self,
        node: HalfEdgeNode<EdgeData, NodeData>,
    ) -> &mut HalfEdgeNode<EdgeData, NodeData> {
        self.nodes.push(node);
        self.nodes.last_mut().unwrap()
    }

    /// Get the number of edges in the graph
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Get the number of nodes in the graph
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Clear all edges and nodes from the graph
    pub fn clear(&mut self) {
        self.edges.clear();
        self.nodes.clear();
    }

    /// Get an iterator over all edges
    pub fn edges_iter(&self) -> impl Iterator<Item = &HalfEdge<EdgeData, NodeData>> {
        self.edges.iter()
    }

    /// Get a mutable iterator over all edges
    pub fn edges_iter_mut(&mut self) -> impl Iterator<Item = &mut HalfEdge<EdgeData, NodeData>> {
        self.edges.iter_mut()
    }

    /// Get an iterator over all nodes
    pub fn nodes_iter(&self) -> impl Iterator<Item = &HalfEdgeNode<EdgeData, NodeData>> {
        self.nodes.iter()
    }

    /// Get a mutable iterator over all nodes
    pub fn nodes_iter_mut(
        &mut self,
    ) -> impl Iterator<Item = &mut HalfEdgeNode<EdgeData, NodeData>> {
        self.nodes.iter_mut()
    }
}

impl<EdgeData, NodeData> Default for HalfEdgeGraph<EdgeData, NodeData> {
    fn default() -> Self {
        Self::new()
    }
}

// HalfEdgeGraph is safe to send between threads if the data types are Send
unsafe impl<EdgeData: Send, NodeData: Send> Send for HalfEdgeGraph<EdgeData, NodeData> {}

// HalfEdgeGraph is safe to share between threads if the data types are Sync
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
        /// Test basic HalfEdgeGraph creation
        /// C++ Reference: Arachne/utils/HalfEdgeGraph.hpp (implicit constructor)
        let graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::new();

        assert_eq!(graph.edge_count(), 0);
        assert_eq!(graph.node_count(), 0);
    }

    #[test]
    fn test_half_edge_graph_with_capacity() {
        /// Test HalfEdgeGraph creation with preallocated capacity
        let graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::with_capacity(10, 5);

        assert_eq!(graph.edge_count(), 0);
        assert_eq!(graph.node_count(), 0);
        assert!(graph.edges.capacity() >= 10);
        assert!(graph.nodes.capacity() >= 5);
    }

    #[test]
    fn test_add_node() {
        /// Test adding nodes to the graph
        let mut graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::new();

        let node1 = HalfEdgeNode::new(TestNodeData { id: 1 }, Point::new(0, 0));
        let node2 = HalfEdgeNode::new(TestNodeData { id: 2 }, Point::new(100, 100));

        graph.add_node(node1);
        graph.add_node(node2);

        assert_eq!(graph.node_count(), 2);
        assert_eq!(graph.nodes[0].data.id, 1);
        assert_eq!(graph.nodes[1].data.id, 2);
    }

    #[test]
    fn test_add_edge() {
        /// Test adding edges to the graph
        let mut graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::new();

        let edge1 = HalfEdge::new(TestEdgeData { weight: 10 });
        let edge2 = HalfEdge::new(TestEdgeData { weight: 20 });

        graph.add_edge(edge1);
        graph.add_edge(edge2);

        assert_eq!(graph.edge_count(), 2);
        assert_eq!(graph.edges[0].data.weight, 10);
        assert_eq!(graph.edges[1].data.weight, 20);
    }

    #[test]
    fn test_clear() {
        /// Test clearing the graph
        let mut graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::new();

        graph.add_node(HalfEdgeNode::new(TestNodeData { id: 1 }, Point::new(0, 0)));
        graph.add_edge(HalfEdge::new(TestEdgeData { weight: 10 }));

        assert_eq!(graph.node_count(), 1);
        assert_eq!(graph.edge_count(), 1);

        graph.clear();

        assert_eq!(graph.node_count(), 0);
        assert_eq!(graph.edge_count(), 0);
    }

    #[test]
    fn test_iterators() {
        /// Test iteration over nodes and edges
        let mut graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::new();

        graph.add_node(HalfEdgeNode::new(TestNodeData { id: 1 }, Point::new(0, 0)));
        graph.add_node(HalfEdgeNode::new(
            TestNodeData { id: 2 },
            Point::new(10, 10),
        ));
        graph.add_edge(HalfEdge::new(TestEdgeData { weight: 5 }));

        let node_ids: Vec<usize> = graph.nodes_iter().map(|n| n.data.id).collect();
        assert_eq!(node_ids, vec![1, 2]);

        let edge_weights: Vec<i32> = graph.edges_iter().map(|e| e.data.weight).collect();
        assert_eq!(edge_weights, vec![5]);
    }

    #[test]
    fn test_mutable_access() {
        /// Test mutable access to graph elements
        let mut graph = HalfEdgeGraph::<TestEdgeData, TestNodeData>::new();

        graph.add_node(HalfEdgeNode::new(TestNodeData { id: 1 }, Point::new(0, 0)));
        graph.add_edge(HalfEdge::new(TestEdgeData { weight: 10 }));

        // Mutate node data
        graph.nodes[0].data.id = 99;
        assert_eq!(graph.nodes[0].data.id, 99);

        // Mutate edge data
        graph.edges[0].data.weight = 200;
        assert_eq!(graph.edges[0].data.weight, 200);
    }

    #[test]
    fn test_default() {
        /// Test Default trait implementation
        let graph: HalfEdgeGraph<TestEdgeData, TestNodeData> = Default::default();

        assert_eq!(graph.edge_count(), 0);
        assert_eq!(graph.node_count(), 0);
    }
}
