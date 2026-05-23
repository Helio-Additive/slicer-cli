//! Skeletal trapezoidation joint for Arachne wall generation
//!
//! C++ Reference:
//! - Arachne/SkeletalTrapezoidationJoint.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use crate::arachne::beading_strategy::beading_strategy::Beading;
use crate::geometry::Coord;
use parking_lot::RwLock;
use std::sync::{Arc, Weak};

/// Beading propagation information for a joint
/// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp (struct BeadingPropagation)
#[derive(Debug, Clone)]
pub struct BeadingPropagation {
    /// The beading at this joint
    /// C++: Beading beading;
    pub beading: Beading,

    /// Distance to the bottom source
    /// C++: coord_t dist_to_bottom_source;
    pub dist_to_bottom_source: Coord,

    /// Distance from the top source
    /// C++: coord_t dist_from_top_source;
    pub dist_from_top_source: Coord,

    /// Whether this is upward propagated only
    /// C++: bool is_upward_propagated_only;
    pub is_upward_propagated_only: bool,
}

impl BeadingPropagation {
    /// Create a new BeadingPropagation with a given beading
    /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:19-26
    /// C++: BeadingPropagation(const Beading& beading)
    /// C++:     : beading(beading)
    /// C++:     , dist_to_bottom_source(0)
    /// C++:     , dist_from_top_source(0)
    /// C++:     , is_upward_propagated_only(false)
    /// C++: {}
    pub fn new(beading: Beading) -> Self {
        Self {
            beading,
            dist_to_bottom_source: 0,
            dist_from_top_source: 0,
            is_upward_propagated_only: false,
        }
    }
}

/// Joint data for skeletal trapezoidation graph nodes
/// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp (class SkeletalTrapezoidationJoint)
#[derive(Debug, Clone)]
pub struct SkeletalTrapezoidationJoint {
    /// Distance to the nearest boundary
    /// C++: coord_t distance_to_boundary;
    pub distance_to_boundary: Coord,

    /// Number of beads at this joint
    /// C++: coord_t bead_count;
    pub bead_count: Coord,

    /// The distance near the skeleton to leave free because this joint is in the middle of a transition,
    /// as a fraction of the inner bead width of the bead at the higher transition
    /// C++: float transition_ratio;
    pub transition_ratio: f32,

    /// Weak pointer to beading propagation data
    /// C++: std::weak_ptr<BeadingPropagation> beading;
    beading: Weak<RwLock<BeadingPropagation>>,
}

impl SkeletalTrapezoidationJoint {
    /// Create a new SkeletalTrapezoidationJoint with default values
    /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:31-36
    /// C++: SkeletalTrapezoidationJoint()
    /// C++: : distance_to_boundary(-1)
    /// C++: , bead_count(-1)
    /// C++: , transition_ratio(0)
    /// C++: {}
    pub fn new() -> Self {
        Self {
            distance_to_boundary: -1,
            bead_count: -1,
            transition_ratio: 0.0,
            beading: Weak::new(),
        }
    }

    /// Check if this joint has beading information
    /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:38-41
    /// C++: bool hasBeading() const
    /// C++: {
    /// C++:     return beading.use_count() > 0;
    /// C++: }
    pub fn has_beading(&self) -> bool {
        self.beading.strong_count() > 0
    }

    /// Set the beading information
    /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:42-45
    /// C++: void setBeading(std::shared_ptr<BeadingPropagation> storage)
    /// C++: {
    /// C++:     beading = storage;
    /// C++: }
    pub fn set_beading(&mut self, storage: Arc<RwLock<BeadingPropagation>>) {
        self.beading = Arc::downgrade(&storage);
    }

    /// Get the beading information
    /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:46-49
    /// C++: std::shared_ptr<BeadingPropagation> getBeading()
    /// C++: {
    /// C++:     return beading.lock();
    /// C++: }
    pub fn get_beading(&self) -> Option<Arc<RwLock<BeadingPropagation>>> {
        self.beading.upgrade()
    }
}

impl Default for SkeletalTrapezoidationJoint {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arachne::beading_strategy::beading_strategy::Beading;

    #[test]
    fn test_joint_default_values() {
        /// Test default initialization
        /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:31-36
        let joint = SkeletalTrapezoidationJoint::new();

        assert_eq!(joint.distance_to_boundary, -1);
        assert_eq!(joint.bead_count, -1);
        assert_eq!(joint.transition_ratio, 0.0);
        assert!(!joint.has_beading());
    }

    #[test]
    fn test_joint_beading() {
        /// Test beading storage and retrieval
        /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:38-49
        let mut joint = SkeletalTrapezoidationJoint::new();

        assert!(!joint.has_beading());
        assert!(joint.get_beading().is_none());

        let beading = Beading {
            total_thickness: 1000,
            bead_widths: vec![300, 300, 400],
            toolpath_locations: vec![150, 450, 850],
            left_over: 0,
            right_over: 0,
        };

        let propagation = Arc::new(RwLock::new(BeadingPropagation::new(beading)));
        joint.set_beading(propagation.clone());

        assert!(joint.has_beading());

        let retrieved = joint.get_beading().unwrap();
        let locked = retrieved.read();
        assert_eq!(locked.beading.bead_widths.len(), 3);
        assert_eq!(locked.dist_to_bottom_source, 0);
        assert_eq!(locked.dist_from_top_source, 0);
        assert!(!locked.is_upward_propagated_only);
    }

    #[test]
    fn test_beading_propagation() {
        /// Test BeadingPropagation initialization
        /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:19-26
        let beading = Beading {
            total_thickness: 500,
            bead_widths: vec![250, 250],
            toolpath_locations: vec![125, 375],
            left_over: 0,
            right_over: 0,
        };

        let propagation = BeadingPropagation::new(beading.clone());

        assert_eq!(propagation.beading.total_thickness, 500);
        assert_eq!(propagation.dist_to_bottom_source, 0);
        assert_eq!(propagation.dist_from_top_source, 0);
        assert!(!propagation.is_upward_propagated_only);
    }

    #[test]
    fn test_joint_fields() {
        /// Test joint field assignment
        /// C++ Reference: Arachne/SkeletalTrapezoidationJoint.hpp:28-30
        let mut joint = SkeletalTrapezoidationJoint::new();

        joint.distance_to_boundary = 1000;
        joint.bead_count = 3;
        joint.transition_ratio = 0.5;

        assert_eq!(joint.distance_to_boundary, 1000);
        assert_eq!(joint.bead_count, 3);
        assert_eq!(joint.transition_ratio, 0.5);
    }
}
