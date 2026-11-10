//! Half-edge node data structure for Arachne wall generation
//!
//! C++ Reference:
//! - Arachne/utils/HalfEdgeNode.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use crate::geometry::Point;
use std::ptr::NonNull;

// Forward declaration for HalfEdge
use super::half_edge::HalfEdge;

/// Node structure for a half-edge graph
///
/// This is the Rust equivalent of C++ template class HalfEdgeNode<node_data_t, edge_data_t, derived_node_t, derived_edge_t>
///
/// C++ Reference: Arachne/utils/HalfEdgeNode.hpp (template class HalfEdgeNode)
///
/// Each node stores data and a position, plus a pointer to one of its incident edges.
#[derive(Debug)]
pub struct HalfEdgeNode<EdgeData, NodeData> {
    /// Data associated with this node
    /// C++: node_data_t data;
    pub data: NodeData,

    /// Position of this node
    /// C++: Point p;
    pub p: Point,

    /// One of the incident half-edges (any edge that starts from this node)
    /// C++: edge_t* incident_edge = nullptr;
    pub incident_edge: Option<NonNull<HalfEdge<EdgeData, NodeData>>>,
}

impl<EdgeData, NodeData> HalfEdgeNode<EdgeData, NodeData> {
    /// Create a new HalfEdgeNode with the given data and position
    /// C++ Reference: Arachne/utils/HalfEdgeNode.hpp:26-29
    /// C++: HalfEdgeNode(node_data_t data, Point p)
    /// C++: : data(data)
    /// C++: , p(p)
    /// C++: {}
    pub fn new(data: NodeData, p: Point) -> Self {
        Self {
            data,
            p,
            incident_edge: None,
        }
    }

    /// Check if this node equals another node (pointer equality)
    /// C++ Reference: Arachne/utils/HalfEdgeNode.hpp:31-34
    /// C++: bool operator==(const node_t& other)
    /// C++: {
    /// C++:     return this == &other;
    /// C++: }
    pub fn ptr_eq(&self, other: &HalfEdgeNode<EdgeData, NodeData>) -> bool {
        std::ptr::eq(self, other)
    }

    /// Get the incident edge (immutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid
    pub unsafe fn incident_edge_ref(&self) -> Option<&HalfEdge<EdgeData, NodeData>> {
        self.incident_edge.map(|ptr| ptr.as_ref())
    }

    /// Get the incident edge (mutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid and not aliased mutably elsewhere
    pub unsafe fn incident_edge_mut(&mut self) -> Option<&mut HalfEdge<EdgeData, NodeData>> {
        self.incident_edge.map(|mut ptr| ptr.as_mut())
    }
}

// HalfEdgeNode is safe to send between threads if the data types are Send
unsafe impl<EdgeData: Send, NodeData: Send> Send for HalfEdgeNode<EdgeData, NodeData> {}

// HalfEdgeNode is safe to share between threads if the data types are Sync
unsafe impl<EdgeData: Sync, NodeData: Sync> Sync for HalfEdgeNode<EdgeData, NodeData> {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    #[derive(Debug, Clone, PartialEq)]
    struct TestEdgeData {
        value: i32,
    }

    #[derive(Debug, Clone, PartialEq)]
    struct TestNodeData {
        id: usize,
    }

    #[test]
    fn test_half_edge_node_creation() {
        /// Test basic HalfEdgeNode creation
        /// C++ Reference: Arachne/utils/HalfEdgeNode.hpp:26-29
        let node_data = TestNodeData { id: 1 };
        let point = Point::new(100, 200);
        let node = HalfEdgeNode::<TestEdgeData, TestNodeData>::new(node_data.clone(), point);

        assert_eq!(node.data.id, 1);
        assert_eq!(node.p.x(), 100);
        assert_eq!(node.p.y(), 200);
        assert!(node.incident_edge.is_none());
    }

    #[test]
    fn test_half_edge_node_ptr_eq() {
        /// Test pointer equality
        /// C++ Reference: Arachne/utils/HalfEdgeNode.hpp:31-34
        let node1 = HalfEdgeNode::<TestEdgeData, TestNodeData>::new(
            TestNodeData { id: 1 },
            Point::new(0, 0),
        );
        let node2 = HalfEdgeNode::<TestEdgeData, TestNodeData>::new(
            TestNodeData { id: 1 },
            Point::new(0, 0),
        );

        assert!(node1.ptr_eq(&node1));
        assert!(!node1.ptr_eq(&node2));
    }

    #[test]
    fn test_half_edge_node_data_mutation() {
        /// Test that node data can be mutated
        let mut node = HalfEdgeNode::<TestEdgeData, TestNodeData>::new(
            TestNodeData { id: 5 },
            Point::new(10, 20),
        );

        node.data.id = 10;
        assert_eq!(node.data.id, 10);

        node.p = Point::new(30, 40);
        assert_eq!(node.p.x(), 30);
        assert_eq!(node.p.y(), 40);
    }

    #[test]
    fn test_half_edge_node_position() {
        /// Test position storage and access
        let point = Point::new(123, 456);
        let node = HalfEdgeNode::<TestEdgeData, TestNodeData>::new(TestNodeData { id: 42 }, point);

        assert_eq!(node.p, point);
    }
}
