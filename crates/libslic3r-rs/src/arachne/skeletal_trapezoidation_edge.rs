//! Skeletal trapezoidation edge for Arachne wall generation
//!
//! C++ Reference:
//! - Arachne/SkeletalTrapezoidationEdge.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use crate::arachne::utils::extrusion_junction::LineJunctions;
use crate::geometry::Coord;
use parking_lot::RwLock;
use std::sync::{Arc, Weak};

/// Enum indicating whether an edge is central (significant)
/// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp (enum class Central)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Central {
    /// Unknown state
    /// C++: Central::UNKNOWN = -1
    Unknown = -1,
    /// Not central
    /// C++: Central::NO
    No = 0,
    /// Is central
    /// C++: Central::YES
    Yes = 1,
}

/// Representing the location along an edge where the anchor position of a transition should be placed
/// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp (struct TransitionMiddle)
#[derive(Debug, Clone)]
pub struct TransitionMiddle {
    /// Position along edge as measured from edge.from.p
    /// C++: coord_t pos;
    pub pos: Coord,

    /// Lower bead count at this transition
    /// C++: int lower_bead_count;
    pub lower_bead_count: i32,

    /// The feature radius at which this transition is placed
    /// C++: coord_t feature_radius;
    pub feature_radius: Coord,
}

impl TransitionMiddle {
    /// Create a new TransitionMiddle
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:30-33
    /// C++: TransitionMiddle(coord_t pos, int lower_bead_count, coord_t feature_radius)
    /// C++:     : pos(pos), lower_bead_count(lower_bead_count)
    /// C++:     , feature_radius(feature_radius)
    /// C++: {}
    pub fn new(pos: Coord, lower_bead_count: i32, feature_radius: Coord) -> Self {
        Self {
            pos,
            lower_bead_count,
            feature_radius,
        }
    }
}

/// Represents the location along an edge where the lower or upper end of a transition should be placed
/// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp (struct TransitionEnd)
#[derive(Debug, Clone)]
pub struct TransitionEnd {
    /// Position along edge as measured from edge.from.p, where the edge is always the half edge oriented from lower to higher R
    /// C++: coord_t pos;
    pub pos: Coord,

    /// Lower bead count at this transition
    /// C++: int lower_bead_count;
    pub lower_bead_count: i32,

    /// Whether this is the end of the transition with lower bead count
    /// C++: bool is_lower_end;
    pub is_lower_end: bool,
}

impl TransitionEnd {
    /// Create a new TransitionEnd
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:45-48
    /// C++: TransitionEnd(coord_t pos, int lower_bead_count, bool is_lower_end)
    /// C++:     : pos(pos), lower_bead_count(lower_bead_count), is_lower_end(is_lower_end)
    /// C++: {}
    pub fn new(pos: Coord, lower_bead_count: i32, is_lower_end: bool) -> Self {
        Self {
            pos,
            lower_bead_count,
            is_lower_end,
        }
    }
}

/// Edge type classification
/// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp (enum class EdgeType)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeType {
    /// Normal edge from voronoi diagram
    /// C++: NORMAL = 0
    Normal = 0,

    /// Extra VD edge introduced to voronoi diagram in order to make the gMAT
    /// C++: EXTRA_VD = 1
    ExtraVd = 1,

    /// Transition end edge introduced to voronoi diagram in order to make the gMAT
    /// C++: TRANSITION_END = 2
    TransitionEnd = 2,
}

/// Edge data for skeletal trapezoidation graph
/// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp (class SkeletalTrapezoidationEdge)
#[derive(Debug, Clone)]
pub struct SkeletalTrapezoidationEdge {
    /// Type of this edge
    /// C++: EdgeType type;
    pub edge_type: EdgeType,

    /// Whether the edge is significant; whether the source segments have a sharp angle
    /// C++: Central is_central;
    is_central: Central,

    /// Whether to apply hole compensation to this edge
    /// C++: bool apply_hole_compensation{ false };
    apply_hole_compensation: bool,

    /// Weak pointer to list of transition middles
    /// C++: std::weak_ptr<std::list<TransitionMiddle>> transitions;
    transitions: Weak<RwLock<Vec<TransitionMiddle>>>,

    /// Weak pointer to list of transition ends
    /// C++: std::weak_ptr<std::list<TransitionEnd>> transition_ends;
    transition_ends: Weak<RwLock<Vec<TransitionEnd>>>,

    /// Weak pointer to extrusion junctions
    /// C++: std::weak_ptr<LineJunctions> extrusion_junctions;
    extrusion_junctions: Weak<RwLock<LineJunctions>>,
}

impl SkeletalTrapezoidationEdge {
    /// Create a new SkeletalTrapezoidationEdge with default type (Normal)
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:58
    /// C++: SkeletalTrapezoidationEdge() : SkeletalTrapezoidationEdge(EdgeType::NORMAL) {}
    pub fn new() -> Self {
        Self::with_type(EdgeType::Normal)
    }

    /// Create a new SkeletalTrapezoidationEdge with specified type
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:59
    /// C++: SkeletalTrapezoidationEdge(const EdgeType &type) : type(type), is_central(Central::UNKNOWN) {}
    pub fn with_type(edge_type: EdgeType) -> Self {
        Self {
            edge_type,
            is_central: Central::Unknown,
            apply_hole_compensation: false,
            transitions: Weak::new(),
            transition_ends: Weak::new(),
            extrusion_junctions: Weak::new(),
        }
    }

    /// Check if this edge is central
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:61-65
    /// C++: bool isCentral() const
    /// C++: {
    /// C++:     assert(is_central != Central::UNKNOWN);
    /// C++:     return is_central == Central::YES;
    /// C++: }
    pub fn is_central(&self) -> bool {
        debug_assert!(
            self.is_central != Central::Unknown,
            "is_central must be set before querying"
        );
        self.is_central == Central::Yes
    }

    /// Set whether this edge is central
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:66-69
    /// C++: void setIsCentral(bool b)
    /// C++: {
    /// C++:     is_central = b ? Central::YES : Central::NO;
    /// C++: }
    pub fn set_is_central(&mut self, is_central: bool) {
        self.is_central = if is_central {
            Central::Yes
        } else {
            Central::No
        };
    }

    /// Check if central flag has been set
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:70-73
    /// C++: bool centralIsSet() const
    /// C++: {
    /// C++:     return is_central != Central::UNKNOWN;
    /// C++: }
    pub fn central_is_set(&self) -> bool {
        self.is_central != Central::Unknown
    }

    /// Check if this edge has transitions
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:75-78
    /// C++: bool hasTransitions(bool ignore_empty = false) const
    /// C++: {
    /// C++:     return transitions.use_count() > 0 && (ignore_empty || ! transitions.lock()->empty());
    /// C++: }
    pub fn has_transitions(&self, ignore_empty: bool) -> bool {
        if let Some(trans) = self.transitions.upgrade() {
            ignore_empty || !trans.read().is_empty()
        } else {
            false
        }
    }

    /// Set the transitions storage
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:79-82
    /// C++: void setTransitions(std::shared_ptr<std::list<TransitionMiddle>> storage)
    /// C++: {
    /// C++:     transitions = storage;
    /// C++: }
    pub fn set_transitions(&mut self, storage: Arc<RwLock<Vec<TransitionMiddle>>>) {
        self.transitions = Arc::downgrade(&storage);
    }

    /// Get the transitions
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:83-86
    /// C++: std::shared_ptr<std::list<TransitionMiddle>> getTransitions()
    /// C++: {
    /// C++:     return transitions.lock();
    /// C++: }
    pub fn get_transitions(&self) -> Option<Arc<RwLock<Vec<TransitionMiddle>>>> {
        self.transitions.upgrade()
    }

    /// Check if this edge has transition ends
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:88-91
    /// C++: bool hasTransitionEnds(bool ignore_empty = false) const
    /// C++: {
    /// C++:     return transition_ends.use_count() > 0 && (ignore_empty || ! transition_ends.lock()->empty());
    /// C++: }
    pub fn has_transition_ends(&self, ignore_empty: bool) -> bool {
        if let Some(ends) = self.transition_ends.upgrade() {
            ignore_empty || !ends.read().is_empty()
        } else {
            false
        }
    }

    /// Set the transition ends storage
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:92-95
    /// C++: void setTransitionEnds(std::shared_ptr<std::list<TransitionEnd>> storage)
    /// C++: {
    /// C++:     transition_ends = storage;
    /// C++: }
    pub fn set_transition_ends(&mut self, storage: Arc<RwLock<Vec<TransitionEnd>>>) {
        self.transition_ends = Arc::downgrade(&storage);
    }

    /// Get the transition ends
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:96-99
    /// C++: std::shared_ptr<std::list<TransitionEnd>> getTransitionEnds()
    /// C++: {
    /// C++:     return transition_ends.lock();
    /// C++: }
    pub fn get_transition_ends(&self) -> Option<Arc<RwLock<Vec<TransitionEnd>>>> {
        self.transition_ends.upgrade()
    }

    /// Check if this edge has extrusion junctions
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:101-104
    /// C++: bool hasExtrusionJunctions(bool ignore_empty = false) const
    /// C++: {
    /// C++:     return extrusion_junctions.use_count() > 0 && (ignore_empty || ! extrusion_junctions.lock()->empty());
    /// C++: }
    pub fn has_extrusion_junctions(&self, ignore_empty: bool) -> bool {
        if let Some(junctions) = self.extrusion_junctions.upgrade() {
            ignore_empty || !junctions.read().is_empty()
        } else {
            false
        }
    }

    /// Set the extrusion junctions storage
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:105-108
    /// C++: void setExtrusionJunctions(std::shared_ptr<LineJunctions> storage)
    /// C++: {
    /// C++:     extrusion_junctions = storage;
    /// C++: }
    pub fn set_extrusion_junctions(&mut self, storage: Arc<RwLock<LineJunctions>>) {
        self.extrusion_junctions = Arc::downgrade(&storage);
    }

    /// Get the extrusion junctions
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:109-112
    /// C++: std::shared_ptr<LineJunctions> getExtrusionJunctions()
    /// C++: {
    /// C++:     return extrusion_junctions.lock();
    /// C++: }
    pub fn get_extrusion_junctions(&self) -> Option<Arc<RwLock<LineJunctions>>> {
        self.extrusion_junctions.upgrade()
    }

    /// Set the hole compensation flag
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:114-117
    /// C++: void setHoleCompensationFlag(bool enabled)
    /// C++: {
    /// C++:     apply_hole_compensation = enabled;
    /// C++: }
    pub fn set_hole_compensation_flag(&mut self, enabled: bool) {
        self.apply_hole_compensation = enabled;
    }

    /// Get the hole compensation flag
    /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:119-122
    /// C++: bool getHoleCompensationFlag() const
    /// C++: {
    /// C++:     return apply_hole_compensation;
    /// C++: }
    pub fn get_hole_compensation_flag(&self) -> bool {
        self.apply_hole_compensation
    }
}

impl Default for SkeletalTrapezoidationEdge {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_edge_type() {
        /// Test edge type initialization
        /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:58-59
        let edge1 = SkeletalTrapezoidationEdge::new();
        assert_eq!(edge1.edge_type, EdgeType::Normal);

        let edge2 = SkeletalTrapezoidationEdge::with_type(EdgeType::ExtraVd);
        assert_eq!(edge2.edge_type, EdgeType::ExtraVd);
    }

    #[test]
    fn test_central_flag() {
        /// Test central flag operations
        /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:61-73
        let mut edge = SkeletalTrapezoidationEdge::new();

        assert!(!edge.central_is_set());

        edge.set_is_central(true);
        assert!(edge.central_is_set());
        assert!(edge.is_central());

        edge.set_is_central(false);
        assert!(!edge.is_central());
    }

    #[test]
    fn test_hole_compensation() {
        /// Test hole compensation flag
        /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:114-122
        let mut edge = SkeletalTrapezoidationEdge::new();

        assert!(!edge.get_hole_compensation_flag());

        edge.set_hole_compensation_flag(true);
        assert!(edge.get_hole_compensation_flag());

        edge.set_hole_compensation_flag(false);
        assert!(!edge.get_hole_compensation_flag());
    }

    #[test]
    fn test_transitions() {
        /// Test transition storage and retrieval
        /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:75-86
        let mut edge = SkeletalTrapezoidationEdge::new();

        assert!(!edge.has_transitions(false));
        assert!(edge.get_transitions().is_none());

        let transitions = Arc::new(RwLock::new(vec![
            TransitionMiddle::new(100, 2, 50),
            TransitionMiddle::new(200, 3, 60),
        ]));

        edge.set_transitions(transitions.clone());
        assert!(edge.has_transitions(false));
        assert!(edge.has_transitions(true));

        let retrieved = edge.get_transitions().unwrap();
        assert_eq!(retrieved.read().len(), 2);
    }

    #[test]
    fn test_transition_ends() {
        /// Test transition end storage and retrieval
        /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:88-99
        let mut edge = SkeletalTrapezoidationEdge::new();

        assert!(!edge.has_transition_ends(false));

        let ends = Arc::new(RwLock::new(vec![TransitionEnd::new(150, 2, true)]));

        edge.set_transition_ends(ends);
        assert!(edge.has_transition_ends(false));
    }

    #[test]
    fn test_extrusion_junctions() {
        /// Test extrusion junction storage
        /// C++ Reference: Arachne/SkeletalTrapezoidationEdge.hpp:101-112
        let mut edge = SkeletalTrapezoidationEdge::new();

        assert!(!edge.has_extrusion_junctions(false));
        assert!(edge.get_extrusion_junctions().is_none());
    }
}
