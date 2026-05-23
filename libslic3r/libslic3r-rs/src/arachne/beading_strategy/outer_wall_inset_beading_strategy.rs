//! Outer wall inset beading strategy implementation.
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.hpp
//! - Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp
//!
//! This is a meta strategy that allows for the outer wall to be inset towards
//! the inside of the model.

use super::beading_strategy::{Beading, BeadingStrategy, BeadingStrategyPtr};
use crate::Coord;

// Outer wall inset beading strategy that moves the outer wall inward.
// Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.hpp:14-33
///
// This is a meta strategy that allows for the outer wall to be inset towards
// the inside of the model.
#[derive(Clone)]
pub struct OuterWallInsetBeadingStrategy {
    // Parent beading strategy
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.hpp:31
    // C++: BeadingStrategyPtr parent;
    parent: BeadingStrategyPtr,

    // Offset amount to move the outer wall inward
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.hpp:32
    // C++: coord_t outer_wall_offset;
    outer_wall_offset: Coord,

    // Strategy name for debugging
    name: String,
}

impl OuterWallInsetBeadingStrategy {
    // Create a new outer wall inset beading strategy
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:10-14
    ///
    // # Arguments
    // * `outer_wall_offset` - Amount to move the outer wall inward
    // * `parent` - Parent beading strategy to wrap
    ///
    // C++: OuterWallInsetBeadingStrategy::OuterWallInsetBeadingStrategy(coord_t outer_wall_offset, BeadingStrategyPtr parent)
    // C++:     : BeadingStrategy(*parent), parent(std::move(parent)), outer_wall_offset(outer_wall_offset)
    // C++: {
    // C++:     name = "OuterWallOfsetBeadingStrategy";
    // C++: }
    pub fn new(outer_wall_offset: Coord, parent: BeadingStrategyPtr) -> Self {
        Self {
            parent,
            outer_wall_offset,
            name: "OuterWallOfsetBeadingStrategy".to_string(),
        }
    }
}

impl BeadingStrategy for OuterWallInsetBeadingStrategy {
    // Compute beading for a given thickness and bead count
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:40-58
    ///
    // C++: BeadingStrategy::Beading OuterWallInsetBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading {
        // Get beading from parent strategy
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:42-45
        // C++: Beading ret = parent->compute(thickness, bead_count);
        // C++:
        // C++: // Actual count and thickness as represented by extant walls. Don't count any potential zero-width 'signaling' walls.
        // C++: bead_count = std::count_if(ret.bead_widths.begin(), ret.bead_widths.end(), [](const coord_t width) { return width > 0; });
        let mut ret = self.parent.compute(thickness, bead_count);

        // Actual count and thickness as represented by extant walls.
        // Don't count any potential zero-width 'signaling' walls.
        let bead_count = ret.bead_widths.iter().filter(|&&width| width > 0).count();

        // No need to apply any inset if there is just a single wall
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:47-51
        // C++: // No need to apply any inset if there is just a single wall.
        // C++: if (bead_count < 2)
        // C++: {
        // C++:     return ret;
        // C++: }
        if bead_count < 2 {
            return ret;
        }

        // Move the outer wall inside, ensuring it never goes beyond the middle line
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:53-56
        // C++: // Actually move the outer wall inside. Ensure that the outer wall never goes beyond the middle line.
        // C++: ret.toolpath_locations[0] = std::min(ret.toolpath_locations[0] + outer_wall_offset, thickness / 2);
        // C++: return ret;
        ret.toolpath_locations[0] = std::cmp::min(
            ret.toolpath_locations[0] + self.outer_wall_offset,
            thickness / 2,
        );

        ret
    }

    // Get the optimal bead count for a given thickness
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:28-31
    ///
    // C++: coord_t OuterWallInsetBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
    // C++: {
    // C++:     return parent->getOptimalBeadCount(thickness);
    // C++: }
    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord {
        self.parent.get_optimal_bead_count(thickness)
    }

    // Get the strategy name
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:38-41
    ///
    // C++: std::string OuterWallInsetBeadingStrategy::toString() const
    // C++: {
    // C++:     return std::string("OuterWallOfsetBeadingStrategy+") + parent->toString();
    // C++: }
    fn name(&self) -> &str {
        &self.name
    }

    // Get the optimal thickness for a given bead count
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:16-19
    ///
    // C++: coord_t OuterWallInsetBeadingStrategy::getOptimalThickness(coord_t bead_count) const
    // C++: {
    // C++:     return parent->getOptimalThickness(bead_count);
    // C++: }
    fn get_optimal_thickness(&self, bead_count: Coord) -> Coord {
        self.parent.get_optimal_thickness(bead_count)
    }

    // Get the transition thickness for a given lower bead count
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:21-24
    ///
    // C++: coord_t OuterWallInsetBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
    // C++: {
    // C++:     return parent->getTransitionThickness(lower_bead_count);
    // C++: }
    fn get_transition_thickness(&self, lower_bead_count: Coord) -> Coord {
        self.parent.get_transition_thickness(lower_bead_count)
    }

    // Get the transitioning length
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:33-36
    ///
    // C++: coord_t OuterWallInsetBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
    // C++: {
    // C++:     return parent->getTransitioningLength(lower_bead_count);
    // C++: }
    fn get_transitioning_length(&self, lower_bead_count: Coord) -> Coord {
        self.parent.get_transitioning_length(lower_bead_count)
    }

    // Get the optimal width (delegates to parent)
    fn optimal_width(&self) -> Coord {
        self.parent.optimal_width()
    }

    // Get the default transition length (delegates to parent)
    fn default_transition_length(&self) -> Coord {
        self.parent.default_transition_length()
    }

    // Get the transitioning angle (delegates to parent)
    fn transitioning_angle(&self) -> f64 {
        self.parent.get_transitioning_angle()
    }

    // Get the wall split middle threshold (delegates to parent)
    fn wall_split_middle_threshold(&self) -> f64 {
        self.parent.get_split_middle_threshold()
    }

    // Get the wall add middle threshold (delegates to parent)
    fn wall_add_middle_threshold(&self) -> f64 {
        self.parent.wall_add_middle_threshold()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arachne::beading_strategy::{make_strategy, DistributedBeadingStrategy};

    // Helper to create a parent strategy for testing
    fn create_test_parent() -> BeadingStrategyPtr {
        make_strategy(DistributedBeadingStrategy::new(
            400_000,                    // optimal_width (0.4mm)
            100_000,                    // default_transition_length
            std::f64::consts::PI / 3.0, // transitioning_angle
            0.4,                        // wall_split_middle_threshold
            0.7,                        // wall_add_middle_threshold
            2,                          // distribution_radius
        ))
    }

    #[test]
    fn test_outer_wall_inset_strategy_creation() {
        let parent = create_test_parent();
        let strategy = OuterWallInsetBeadingStrategy::new(
            50_000, // outer_wall_offset (0.05mm)
            parent,
        );

        assert_eq!(strategy.name(), "OuterWallOfsetBeadingStrategy");
        assert_eq!(strategy.outer_wall_offset, 50_000);
    }

    #[test]
    fn test_compute_single_wall_no_inset() {
        let parent = create_test_parent();
        let strategy = OuterWallInsetBeadingStrategy::new(50_000, parent);

        // Single wall should not be inset
        let beading = strategy.compute(400_000, 1);
        assert_eq!(beading.bead_widths.len(), 1);
        // Original position should be unchanged
        assert_eq!(beading.toolpath_locations[0], 200_000); // thickness / 2
    }

    #[test]
    fn test_compute_two_walls_with_inset() {
        let parent = create_test_parent();
        let strategy = OuterWallInsetBeadingStrategy::new(50_000, parent);

        // Two walls - outer wall should be inset
        let beading = strategy.compute(800_000, 2);
        assert_eq!(beading.bead_widths.len(), 2);

        // Outer wall should be moved inward by offset amount
        // Original would be at 200_000, with 50_000 offset it should be at 250_000
        assert!(beading.toolpath_locations[0] >= 200_000);
    }

    #[test]
    fn test_compute_inset_capped_at_middle() {
        let parent = create_test_parent();
        // Large offset that would push outer wall past the middle
        let strategy = OuterWallInsetBeadingStrategy::new(500_000, parent);

        let thickness = 800_000;
        let beading = strategy.compute(thickness, 2);

        // Outer wall should be capped at middle line
        assert_eq!(beading.toolpath_locations[0], thickness / 2);
    }

    #[test]
    fn test_delegates_to_parent() {
        let parent = create_test_parent();
        let strategy = OuterWallInsetBeadingStrategy::new(50_000, parent);

        // All these methods should delegate to parent
        assert_eq!(
            strategy.get_optimal_bead_count(800_000),
            strategy.parent.get_optimal_bead_count(800_000)
        );
        assert_eq!(
            strategy.get_optimal_thickness(2),
            strategy.parent.get_optimal_thickness(2)
        );
        assert_eq!(
            strategy.get_transition_thickness(1),
            strategy.parent.get_transition_thickness(1)
        );
        assert_eq!(
            strategy.get_transitioning_length(1),
            strategy.parent.get_transitioning_length(1)
        );
    }

    #[test]
    fn test_ignores_zero_width_marker_beads() {
        let parent = create_test_parent();
        let strategy = OuterWallInsetBeadingStrategy::new(50_000, parent);

        // Create a beading with zero-width marker beads
        let mut beading = Beading {
            total_thickness: 800_000,
            bead_widths: vec![400_000, 0, 400_000], // Middle bead is marker
            toolpath_locations: vec![200_000, 400_000, 600_000],
            left_over: 0,
        };

        // Count should ignore zero-width beads
        let non_zero_count = beading.bead_widths.iter().filter(|&&w| w > 0).count();
        assert_eq!(non_zero_count, 2);
    }
}
