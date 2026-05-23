//! Half-edge data structure for Arachne wall generation
//!
//! C++ Reference:
//! - Arachne/utils/HalfEdge.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use std::ptr::NonNull;

/// Half-edge structure for representing a directed edge in a graph
///
/// This is the Rust equivalent of C++ template class HalfEdge<node_data_t, edge_data_t, derived_node_t, derived_edge_t>
///
/// C++ Reference: Arachne/utils/HalfEdge.hpp (template class HalfEdge)
///
/// The half-edge data structure represents edges as pairs of directed half-edges.
/// Each half-edge points to its twin (the opposite direction), the next/previous edges
/// in the face, and the nodes it connects.
#[derive(Debug)]
pub struct HalfEdge<EdgeData, NodeData> {
    /// Data associated with this edge
    /// C++: edge_data_t data;
    pub data: EdgeData,

    /// Twin half-edge (opposite direction)
    /// C++: edge_t* twin = nullptr;
    pub twin: Option<NonNull<HalfEdge<EdgeData, NodeData>>>,

    /// Next half-edge in the face
    /// C++: edge_t* next = nullptr;
    pub next: Option<NonNull<HalfEdge<EdgeData, NodeData>>>,

    /// Previous half-edge in the face
    /// C++: edge_t* prev = nullptr;
    pub prev: Option<NonNull<HalfEdge<EdgeData, NodeData>>>,

    /// Source node (from)
    /// C++: node_t* from = nullptr;
    pub from: Option<NonNull<HalfEdgeNode<EdgeData, NodeData>>>,

    /// Target node (to)
    /// C++: node_t* to = nullptr;
    pub to: Option<NonNull<HalfEdgeNode<EdgeData, NodeData>>>,
}

// Forward declaration for HalfEdgeNode
use super::half_edge_node::HalfEdgeNode;

impl<EdgeData, NodeData> HalfEdge<EdgeData, NodeData> {
    /// Create a new HalfEdge with the given data
    /// C++ Reference: Arachne/utils/HalfEdge.hpp:27-29
    /// C++: HalfEdge(edge_data_t data)
    /// C++: : data(data)
    /// C++: {}
    pub fn new(data: EdgeData) -> Self {
        Self {
            data,
            twin: None,
            next: None,
            prev: None,
            from: None,
            to: None,
        }
    }

    /// Check if this edge equals another edge (pointer equality)
    /// C++ Reference: Arachne/utils/HalfEdge.hpp:30-33
    /// C++: bool operator==(const edge_t& other)
    /// C++: {
    /// C++:     return this == &other;
    /// C++: }
    pub fn ptr_eq(&self, other: &HalfEdge<EdgeData, NodeData>) -> bool {
        std::ptr::eq(self, other)
    }

    /// Get the twin edge (mutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid and not aliased mutably elsewhere
    pub unsafe fn twin_mut(&mut self) -> Option<&mut HalfEdge<EdgeData, NodeData>> {
        self.twin.map(|mut ptr| ptr.as_mut())
    }

    /// Get the next edge (mutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid and not aliased mutably elsewhere
    pub unsafe fn next_mut(&mut self) -> Option<&mut HalfEdge<EdgeData, NodeData>> {
        self.next.map(|mut ptr| ptr.as_mut())
    }

    /// Get the previous edge (mutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid and not aliased mutably elsewhere
    pub unsafe fn prev_mut(&mut self) -> Option<&mut HalfEdge<EdgeData, NodeData>> {
        self.prev.map(|mut ptr| ptr.as_mut())
    }

    /// Get the from node (immutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid
    pub unsafe fn from_ref(&self) -> Option<&HalfEdgeNode<EdgeData, NodeData>> {
        self.from.map(|ptr| ptr.as_ref())
    }

    /// Get the to node (immutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid
    pub unsafe fn to_ref(&self) -> Option<&HalfEdgeNode<EdgeData, NodeData>> {
        self.to.map(|ptr| ptr.as_ref())
    }

    /// Get the from node (mutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid and not aliased mutably elsewhere
    pub unsafe fn from_mut(&mut self) -> Option<&mut HalfEdgeNode<EdgeData, NodeData>> {
        self.from.map(|mut ptr| ptr.as_mut())
    }

    /// Get the to node (mutable)
    ///
    /// # Safety
    /// The caller must ensure the pointer is valid and not aliased mutably elsewhere
    pub unsafe fn to_mut(&mut self) -> Option<&mut HalfEdgeNode<EdgeData, NodeData>> {
        self.to.map(|mut ptr| ptr.as_mut())
    }
}

// HalfEdge is safe to send between threads if the data types are Send
unsafe impl<EdgeData: Send, NodeData: Send> Send for HalfEdge<EdgeData, NodeData> {}

// HalfEdge is safe to share between threads if the data types are Sync
unsafe impl<EdgeData: Sync, NodeData: Sync> Sync for HalfEdge<EdgeData, NodeData> {}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, PartialEq)]
    struct TestEdgeData {
        value: i32,
    }

    #[derive(Debug, Clone, PartialEq)]
    struct TestNodeData {
        id: usize,
    }

    #[test]
    fn test_half_edge_creation() {
        /// Test basic HalfEdge creation
        /// C++ Reference: Arachne/utils/HalfEdge.hpp:27-29
        let edge_data = TestEdgeData { value: 42 };
        let edge = HalfEdge::<TestEdgeData, TestNodeData>::new(edge_data.clone());

        assert_eq!(edge.data.value, 42);
        assert!(edge.twin.is_none());
        assert!(edge.next.is_none());
        assert!(edge.prev.is_none());
        assert!(edge.from.is_none());
        assert!(edge.to.is_none());
    }

    #[test]
    fn test_half_edge_ptr_eq() {
        /// Test pointer equality
        /// C++ Reference: Arachne/utils/HalfEdge.hpp:30-33
        let edge1 = HalfEdge::<TestEdgeData, TestNodeData>::new(TestEdgeData { value: 1 });
        let edge2 = HalfEdge::<TestEdgeData, TestNodeData>::new(TestEdgeData { value: 1 });

        assert!(edge1.ptr_eq(&edge1));
        assert!(!edge1.ptr_eq(&edge2));
    }

    #[test]
    fn test_half_edge_data_mutation() {
        /// Test that edge data can be mutated
        let mut edge = HalfEdge::<TestEdgeData, TestNodeData>::new(TestEdgeData { value: 10 });

        edge.data.value = 20;
        assert_eq!(edge.data.value, 20);
    }
}
