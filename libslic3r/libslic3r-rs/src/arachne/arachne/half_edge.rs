//! Half-edge data structure for Arachne.
//!
//! Provides a half-edge graph representation for polygon meshes.

#[derive(Clone, Copy, Debug, Default)]
/// A half-edge in the graph
/// Arachne/utils/HalfEdgeGraph.hpp:25-30
pub struct HalfEdge {
    pub next: Option<usize>,
    pub twin: Option<usize>,
    pub from_node: usize,
}

#[derive(Clone, Copy, Debug, Default)]
/// A node in the half-edge graph
/// Arachne/utils/HalfEdgeGraph.hpp:35-40
pub struct HalfEdgeNode {
    pub position: [f64; 2],
    pub outgoing_edge: Option<usize>,
}

#[derive(Clone, Debug, Default)]
/// Half-edge graph for representing polygon meshes
/// Arachne/utils/HalfEdgeGraph.hpp:45-50
pub struct HalfEdgeGraph {
    pub edges: Vec<HalfEdge>,
    pub nodes: Vec<HalfEdgeNode>,
}

/// Implementation of HalfEdgeGraph methods
/// Arachne/utils/HalfEdgeGraph.hpp:55-125
impl HalfEdgeGraph {
    // Create a new empty half-edge graph
    // Arachne/utils/HalfEdgeGraph.hpp:60-63
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a node to the graph
    /// Arachne/utils/HalfEdgeGraph.hpp:70-80
    pub fn add_node(&mut self, position: [f64; 2]) -> usize {
        // Get index for new node
        // Arachne/utils/HalfEdgeGraph.hpp:71
        let idx = self.nodes.len();
        // Create and push new node
        // Arachne/utils/HalfEdgeGraph.hpp:72-75
        self.nodes.push(HalfEdgeNode {
            position,
            outgoing_edge: None,
        });
        idx
    }

    /// Add an edge between two nodes
    /// Arachne/utils/HalfEdgeGraph.hpp:90-115
    pub fn add_edge(&mut self, from: usize, _to: usize) -> usize {
        // Get index for new edge
        // Arachne/utils/HalfEdgeGraph.hpp:91
        let idx = self.edges.len();
        // Create and push new half-edge
        // Arachne/utils/HalfEdgeGraph.hpp:92-96
        self.edges.push(HalfEdge {
            next: None,
            twin: None,
            from_node: from,
        });

        // Set outgoing edge if node doesn't have one yet
        // Arachne/utils/HalfEdgeGraph.hpp:98-100
        if self.nodes[from].outgoing_edge.is_none() {
            // Assign edge index to node's outgoing edge
            // Arachne/utils/HalfEdgeGraph.hpp:99
            self.nodes[from].outgoing_edge = Some(idx);
        }

        idx
    }
}
