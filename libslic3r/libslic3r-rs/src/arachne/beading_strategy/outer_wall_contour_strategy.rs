//! OuterWallContourStrategy - wrapper strategy for outer wall contour generation
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/OuterWallContourStrategy.hpp
//! - Arachne/BeadingStrategy/OuterWallContourStrategy.cpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation

use crate::arachne::beading_strategy::beading_strategy::{
    Beading, BeadingStrategy, BeadingStrategyPtr, FIRST_WALL_CONTOUR_MARKED_WIDTH,
};
use crate::geometry::Coord;
use std::sync::Arc;

/// Wrapper strategy that adds outer wall contour beads around a parent strategy
/// OuterWallContourStrategy.hpp:8-26
#[derive(Clone)]
pub struct OuterWallContourStrategy {
    /// The parent strategy being wrapped
    /// OuterWallContourStrategy.hpp:24
    parent: BeadingStrategyPtr,

    /// Base strategy fields (name, default_transition_length)
    name: String,
    default_transition_length: Coord,
}

impl OuterWallContourStrategy {
    /// Create a new OuterWallContourStrategy wrapping a parent strategy
    /// OuterWallContourStrategy.cpp:8-12
    pub fn new(parent: BeadingStrategyPtr) -> Self {
        let default_transition_length = parent.default_transition_length();
        let name = parent.name().to_string();

        Self {
            parent,
            name,
            default_transition_length,
        }
    }
}

impl BeadingStrategy for OuterWallContourStrategy {
    /// Compute beading for given thickness and bead count
    /// Adds two contour beads around the parent's beading
    /// OuterWallContourStrategy.cpp:62-83
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading {
        if bead_count <= 1 {
            return self.parent.compute(thickness, bead_count);
        }

        assert!(bead_count >= 3);
        let mut ret = self.parent.compute(thickness, bead_count - 2);

        // Single toolpath case - return as-is
        // OuterWallContourStrategy.cpp:70-72
        if ret.toolpath_locations.len() == 1 {
            return ret;
        }

        // Add contour beads on both sides
        // OuterWallContourStrategy.cpp:73-81
        if !ret.toolpath_locations.is_empty() {
            assert!(!ret.bead_widths.is_empty());

            // Add inner contour on the front side
            let location = ret.toolpath_locations[0] + ret.bead_widths[0] / 2;
            ret.toolpath_locations.insert(1, location);
            ret.bead_widths.insert(1, FIRST_WALL_CONTOUR_MARKED_WIDTH);

            // Add inner contour on the back side
            let last_idx = ret.toolpath_locations.len() - 1;
            let location_reverse = ret.toolpath_locations[last_idx] - ret.bead_widths[last_idx] / 2;
            ret.toolpath_locations.insert(last_idx, location_reverse);
            ret.bead_widths
                .insert(last_idx, FIRST_WALL_CONTOUR_MARKED_WIDTH);
        }

        ret
    }

    /// Get optimal thickness for given bead count
    /// Adjusts parent's result by adding 2 to account for contour beads
    /// OuterWallContourStrategy.cpp:57-61
    fn get_optimal_thickness(&self, bead_count: Coord) -> Coord {
        if bead_count <= 1 {
            return self.parent.get_optimal_thickness(bead_count);
        }
        self.parent.get_optimal_thickness(bead_count - 2) + 2
    }

    /// Get transition thickness for given lower bead count
    /// OuterWallContourStrategy.cpp:35-43
    fn get_transition_thickness(&self, lower_bead_count: Coord) -> Coord {
        if lower_bead_count <= 1 {
            self.parent.get_transition_thickness(lower_bead_count)
        } else if lower_bead_count == 2 || lower_bead_count == 3 {
            self.parent.get_transition_thickness(1)
        } else {
            self.parent.get_transition_thickness(lower_bead_count - 2)
        }
    }

    /// Get optimal bead count for given thickness
    /// Adjusts parent's result by adding 2 for contour beads
    /// OuterWallContourStrategy.cpp:46-52
    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord {
        let parent_bead_count = self.parent.get_optimal_bead_count(thickness);
        if parent_bead_count <= 1 {
            return parent_bead_count;
        }
        parent_bead_count + 2
    }

    /// Get transitioning length for given lower bead count
    /// OuterWallContourStrategy.cpp:19-22
    fn get_transitioning_length(&self, lower_bead_count: Coord) -> Coord {
        self.parent.get_transitioning_length(lower_bead_count)
    }

    /// Get transition anchor position
    /// OuterWallContourStrategy.cpp:24-27
    fn get_transition_anchor_pos(&self, lower_bead_count: Coord) -> f32 {
        self.parent.get_transition_anchor_pos(lower_bead_count)
    }

    /// Get nonlinear thicknesses for given lower bead count
    /// OuterWallContourStrategy.cpp:29-32
    fn get_nonlinear_thicknesses(&self, lower_bead_count: Coord) -> Vec<Coord> {
        self.parent.get_nonlinear_thicknesses(lower_bead_count)
    }

    /// Get the strategy name
    fn name(&self) -> &str {
        &self.name
    }

    /// Get the default transition length
    fn default_transition_length(&self) -> Coord {
        self.default_transition_length
    }

    /// Get the optimal width (delegate to parent)
    fn optimal_width(&self) -> Coord {
        self.parent.optimal_width()
    }

    /// Get the transitioning angle (delegate to parent)
    fn transitioning_angle(&self) -> f64 {
        self.parent.transitioning_angle()
    }

    /// Get the wall split middle threshold (delegate to parent)
    fn wall_split_middle_threshold(&self) -> f64 {
        self.parent.wall_split_middle_threshold()
    }

    /// Get the wall add middle threshold (delegate to parent)
    fn wall_add_middle_threshold(&self) -> f64 {
        self.parent.wall_add_middle_threshold()
    }
}

/// Helper function to create Arc-wrapped OuterWallContourStrategy
pub fn outer_wall_contour_strategy(parent: BeadingStrategyPtr) -> BeadingStrategyPtr {
    Arc::new(OuterWallContourStrategy::new(parent))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arachne::beading_strategy::beading_strategy::BeadingStrategy;
    use crate::arachne::beading_strategy::distributed_beading_strategy::DistributedBeadingStrategy;

    fn create_test_parent() -> BeadingStrategyPtr {
        Arc::new(DistributedBeadingStrategy::new(
            400,  // preferred_bead_width
            1.0,  // transitioning_angle
            0.4,  // wall_split_middle_threshold
            0.5,  // wall_add_middle_threshold
            1000, // default_transition_length
        ))
    }

    #[test]
    fn test_outer_wall_contour_strategy_creation() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent);
        assert!(strategy.name().contains("Distributed"));
    }

    #[test]
    fn test_bead_count_adjustment() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        // Single bead - should pass through to parent
        let count1 = strategy.get_optimal_bead_count(400);
        let parent_count1 = parent.get_optimal_bead_count(400);
        assert_eq!(count1, parent_count1);

        // Multiple beads - should add 2
        let count2 = strategy.get_optimal_bead_count(1200);
        let parent_count2 = parent.get_optimal_bead_count(1200);
        assert_eq!(count2, parent_count2 + 2);
    }

    #[test]
    fn test_optimal_thickness_adjustment() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        // Single bead - pass through
        let thickness1 = strategy.get_optimal_thickness(1);
        let parent_thickness1 = parent.get_optimal_thickness(1);
        assert_eq!(thickness1, parent_thickness1);

        // Multiple beads - adjust by parent(count-2) + 2
        let thickness3 = strategy.get_optimal_thickness(3);
        let parent_thickness1_plus2 = parent.get_optimal_thickness(1) + 2;
        assert_eq!(thickness3, parent_thickness1_plus2);
    }

    #[test]
    fn test_compute_single_bead() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        let beading = strategy.compute(400, 1);
        let parent_beading = parent.compute(400, 1);

        // Should be identical to parent for single bead
        assert_eq!(
            beading.toolpath_locations.len(),
            parent_beading.toolpath_locations.len()
        );
        assert_eq!(beading.bead_widths.len(), parent_beading.bead_widths.len());
    }

    #[test]
    fn test_compute_adds_contour_beads() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        // Request 3 beads (parent will compute 1 bead)
        let beading = strategy.compute(1200, 3);
        let parent_beading = parent.compute(1200, 1);

        // Parent with 1 bead returns 1 toolpath - strategy should keep it as-is
        if parent_beading.toolpath_locations.len() == 1 {
            assert_eq!(beading.toolpath_locations.len(), 1);
        }

        // Request more beads to test contour insertion
        let beading5 = strategy.compute(2000, 5);
        let parent_beading3 = parent.compute(2000, 3);

        // Should add 2 contour beads (one on each side)
        if parent_beading3.toolpath_locations.len() > 1 {
            assert_eq!(
                beading5.toolpath_locations.len(),
                parent_beading3.toolpath_locations.len() + 2
            );
            assert_eq!(
                beading5.bead_widths.len(),
                parent_beading3.bead_widths.len() + 2
            );

            // Check that contour beads have marked width
            assert!(beading5
                .bead_widths
                .contains(&FIRST_WALL_CONTOUR_MARKED_WIDTH));
        }
    }

    #[test]
    fn test_transition_thickness() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        // Lower bead count <= 1: pass through
        let t1 = strategy.get_transition_thickness(1);
        assert_eq!(t1, parent.get_transition_thickness(1));

        // Lower bead count 2 or 3: use parent's transition for 1
        let t2 = strategy.get_transition_thickness(2);
        assert_eq!(t2, parent.get_transition_thickness(1));

        let t3 = strategy.get_transition_thickness(3);
        assert_eq!(t3, parent.get_transition_thickness(1));

        // Higher counts: subtract 2
        let t5 = strategy.get_transition_thickness(5);
        assert_eq!(t5, parent.get_transition_thickness(3));
    }

    #[test]
    fn test_transitioning_length_passthrough() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        let length = strategy.get_transitioning_length(2);
        assert_eq!(length, parent.get_transitioning_length(2));
    }

    #[test]
    fn test_transition_anchor_pos_passthrough() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        let pos = strategy.get_transition_anchor_pos(2);
        assert_eq!(pos, parent.get_transition_anchor_pos(2));
    }

    #[test]
    fn test_nonlinear_thicknesses_passthrough() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        let thicknesses = strategy.get_nonlinear_thicknesses(2);
        assert_eq!(thicknesses, parent.get_nonlinear_thicknesses(2));
    }
}
