//! Redistribute beading strategy implementation.
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp
//! - Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp
//!
//! A meta-beading-strategy that takes outer and inner wall widths into account.
//! The outer wall will try to keep a constant width by only applying the beading
//! strategy on the inner walls.

use super::beading_strategy::{Beading, BeadingStrategy, BeadingStrategyPtr};
use crate::Coord;

// A meta-beading-strategy that takes outer and inner wall widths into account.
// Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp:19-53
///
// The outer wall will try to keep a constant width by only applying the beading strategy
// on the inner walls. This ensures that this outer wall doesn't react to changes happening
// to inner walls. It will limit print artifacts on the surface of the print. Although this
// strategy technically deviates from the original philosophy of the paper, it will generally
// result in better prints because of a smoother motion and less variation in extrusion width
// in the outer walls.
///
// If the thickness of the model is less than two times the optimal outer wall width and once
// the minimum inner wall width, it will keep the minimum inner wall at a minimum constant and
// vary the outer wall widths symmetrically. Until the thickness of the model is that of at
// least twice the optimal outer wall width, it will then use two symmetrical outer walls only.
// Until it transitions into a single outer wall. These last scenarios are always symmetrical
// in nature, disregarding the user specified strategy.
#[derive(Clone)]
pub struct RedistributeBeadingStrategy {
    // Parent strategy for inner walls
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp:50
    // C++: BeadingStrategyPtr parent;
    parent: BeadingStrategyPtr,

    // Outer wall width, guaranteed to be the actual (save rounding errors) at a bead count
    // if the parent strategies' optimum bead width is a weighted average of the outer and
    // inner walls at that bead count.
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp:51
    // C++: coord_t optimal_width_outer;
    optimal_width_outer: Coord,

    // Minimum factor that the variable line might deviate from the optimal width
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp:52
    // C++: double minimum_variable_line_ratio;
    minimum_variable_line_ratio: f64,

    // Strategy name for debugging
    name: String,
}

impl RedistributeBeadingStrategy {
    // Create a new redistribute beading strategy
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:11-18
    ///
    // # Arguments
    // * `optimal_width_outer` - Outer wall width
    // * `minimum_variable_line_ratio` - Minimum factor for variable line deviation
    // * `parent` - Parent beading strategy for inner walls
    ///
    // C++: RedistributeBeadingStrategy::RedistributeBeadingStrategy(
    // C++:     const coord_t optimal_width_outer,
    // C++:     const double minimum_variable_line_ratio,
    // C++:     BeadingStrategyPtr parent)
    // C++:     : BeadingStrategy(*parent)
    // C++:     , parent(std::move(parent))
    // C++:     , optimal_width_outer(optimal_width_outer)
    // C++:     , minimum_variable_line_ratio(minimum_variable_line_ratio)
    // C++: {
    // C++:     name = "RedistributeBeadingStrategy";
    // C++: }
    pub fn new(
        optimal_width_outer: Coord,
        minimum_variable_line_ratio: f64,
        parent: BeadingStrategyPtr,
    ) -> Self {
        Self {
            parent,
            optimal_width_outer,
            minimum_variable_line_ratio,
            name: "RedistributeBeadingStrategy".to_string(),
        }
    }

    // Get the parent strategy name for toString
    fn parent_name(&self) -> String {
        self.parent.name().to_string()
    }
}

impl BeadingStrategy for RedistributeBeadingStrategy {
    // Compute beading for a given thickness and bead count
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:64-107
    ///
    // C++: BeadingStrategy::Beading RedistributeBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading {
        // Initialize beading result
        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:66-72
        // C++: Beading ret;
        // C++:
        // C++: // Take care of all situations in which no lines are actually produced:
        // C++: if (bead_count == 0 || thickness < minimum_variable_line_ratio * optimal_width_outer) {
        // C++:     ret.left_over       = thickness;
        // C++:     ret.total_thickness = thickness;
        // C++:     return ret;
        // C++: }
        let mut ret = Beading::default();

        // Take care of all situations in which no lines are actually produced:
        if bead_count == 0
            || thickness
                < (self.minimum_variable_line_ratio * self.optimal_width_outer as f64) as Coord
        {
            ret.left_over = thickness;
            ret.total_thickness = thickness;
            return ret;
        }

        // Compute the beadings of the inner walls, if any
        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:74-80
        // C++: // Compute the beadings of the inner walls, if any:
        // C++: const coord_t inner_bead_count = bead_count - 2;
        // C++: const coord_t inner_thickness  = thickness - 2 * optimal_width_outer;
        // C++: if (inner_bead_count > 0 && inner_thickness > 0) {
        // C++:     ret = parent->compute(inner_thickness, inner_bead_count);
        // C++:     for (auto &toolpath_location : ret.toolpath_locations) toolpath_location += optimal_width_outer;
        // C++: }
        let inner_bead_count = bead_count - 2;
        let inner_thickness = thickness - 2 * self.optimal_width_outer;
        if inner_bead_count > 0 && inner_thickness > 0 {
            ret = self.parent.compute(inner_thickness, inner_bead_count);
            for toolpath_location in &mut ret.toolpath_locations {
                *toolpath_location += self.optimal_width_outer;
            }
        }

        // Insert the outer wall(s) around the previously computed inner wall(s)
        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:82-89
        // C++: // Insert the outer wall(s) around the previously computed inner wall(s), which may be empty:
        // C++: const coord_t actual_outer_thickness = bead_count > 2 ? std::min(thickness / 2, optimal_width_outer) : thickness / bead_count;
        // C++: ret.bead_widths.insert(ret.bead_widths.begin(), actual_outer_thickness);
        // C++: ret.toolpath_locations.insert(ret.toolpath_locations.begin(), actual_outer_thickness / 2);
        // C++: if (bead_count > 1) {
        // C++:     ret.bead_widths.push_back(actual_outer_thickness);
        // C++:     ret.toolpath_locations.push_back(thickness - actual_outer_thickness / 2);
        // C++: }
        let actual_outer_thickness = if bead_count > 2 {
            std::cmp::min(thickness / 2, self.optimal_width_outer)
        } else {
            thickness / bead_count
        };

        ret.bead_widths.insert(0, actual_outer_thickness);
        ret.toolpath_locations.insert(0, actual_outer_thickness / 2);

        if bead_count > 1 {
            ret.bead_widths.push(actual_outer_thickness);
            ret.toolpath_locations
                .push(thickness - actual_outer_thickness / 2);
        }

        // Ensure correct total and left over thickness
        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:91-94
        // C++: // Ensure correct total and left over thickness.
        // C++: ret.total_thickness = thickness;
        // C++: ret.left_over       = thickness - std::accumulate(ret.bead_widths.cbegin(), ret.bead_widths.cend(), static_cast<coord_t>(0));
        // C++: return ret;
        ret.total_thickness = thickness;
        ret.left_over = thickness - ret.bead_widths.iter().sum::<Coord>();
        ret
    }

    // Get the optimal bead count for a given thickness
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:42-47
    ///
    // C++: coord_t RedistributeBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
    // C++: {
    // C++:     if (thickness < minimum_variable_line_ratio * optimal_width_outer)
    // C++:         return 0;
    // C++:     if (thickness <= 2 * optimal_width_outer)
    // C++:         return thickness > (1.0 + parent->getSplitMiddleThreshold()) * optimal_width_outer ? 2 : 1;
    // C++:     return parent->getOptimalBeadCount(thickness - 2 * optimal_width_outer) + 2;
    // C++: }
    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord {
        if thickness < (self.minimum_variable_line_ratio * self.optimal_width_outer as f64) as Coord
        {
            return 0;
        }
        if thickness <= 2 * self.optimal_width_outer {
            return if thickness
                > ((1.0 + self.parent.get_split_middle_threshold())
                    * self.optimal_width_outer as f64) as Coord
            {
                2
            } else {
                1
            };
        }
        self.parent
            .get_optimal_bead_count(thickness - 2 * self.optimal_width_outer)
            + 2
    }

    // Get the strategy name
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:59-62
    ///
    // C++: std::string RedistributeBeadingStrategy::toString() const
    // C++: {
    // C++:     return std::string("RedistributeBeadingStrategy+") + parent->toString();
    // C++: }
    fn name(&self) -> &str {
        &self.name
    }

    // Get the optimal thickness for a given bead count
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:20-25
    ///
    // C++: coord_t RedistributeBeadingStrategy::getOptimalThickness(coord_t bead_count) const
    // C++: {
    // C++:     const coord_t inner_bead_count = std::max(static_cast<coord_t>(0), bead_count - 2);
    // C++:     const coord_t outer_bead_count = bead_count - inner_bead_count;
    // C++:     return parent->getOptimalThickness(inner_bead_count) + optimal_width_outer * outer_bead_count;
    // C++: }
    fn get_optimal_thickness(&self, bead_count: Coord) -> Coord {
        let inner_bead_count = std::cmp::max(0, bead_count - 2);
        let outer_bead_count = bead_count - inner_bead_count;
        self.parent.get_optimal_thickness(inner_bead_count)
            + self.optimal_width_outer * outer_bead_count
    }

    // Get the transition thickness for a given lower bead count
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:27-33
    ///
    // C++: coord_t RedistributeBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
    // C++: {
    // C++:     switch (lower_bead_count) {
    // C++:     case 0: return minimum_variable_line_ratio * optimal_width_outer;
    // C++:     case 1: return (1.0 + parent->getSplitMiddleThreshold()) * optimal_width_outer;
    // C++:     default: return parent->getTransitionThickness(lower_bead_count - 2) + 2 * optimal_width_outer;
    // C++:     }
    // C++: }
    fn get_transition_thickness(&self, lower_bead_count: Coord) -> Coord {
        match lower_bead_count {
            0 => (self.minimum_variable_line_ratio * self.optimal_width_outer as f64) as Coord,
            1 => {
                ((1.0 + self.parent.get_split_middle_threshold()) * self.optimal_width_outer as f64)
                    as Coord
            }
            _ => {
                self.parent.get_transition_thickness(lower_bead_count - 2)
                    + 2 * self.optimal_width_outer
            }
        }
    }

    // Get the transitioning length
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:49-52
    ///
    // C++: coord_t RedistributeBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
    // C++: {
    // C++:     return parent->getTransitioningLength(lower_bead_count);
    // C++: }
    fn get_transitioning_length(&self, lower_bead_count: Coord) -> Coord {
        self.parent.get_transitioning_length(lower_bead_count)
    }

    // Get the transition anchor position
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:54-57
    ///
    // C++: float RedistributeBeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
    // C++: {
    // C++:     return parent->getTransitionAnchorPos(lower_bead_count);
    // C++: }
    fn get_transition_anchor_pos(&self, lower_bead_count: Coord) -> f32 {
        self.parent.get_transition_anchor_pos(lower_bead_count)
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
        // Parent strategy provides this
        0.0 // TODO: This should come from parent if available
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
    fn test_redistribute_strategy_creation() {
        let parent = create_test_parent();
        let strategy = RedistributeBeadingStrategy::new(
            400_000, // optimal_width_outer (0.4mm)
            0.34,    // minimum_variable_line_ratio
            parent,
        );

        assert_eq!(strategy.name(), "RedistributeBeadingStrategy");
        assert_eq!(strategy.optimal_width_outer, 400_000);
        assert!((strategy.minimum_variable_line_ratio - 0.34).abs() < 1e-10);
    }

    #[test]
    fn test_optimal_bead_count_zero() {
        let parent = create_test_parent();
        let strategy = RedistributeBeadingStrategy::new(400_000, 0.34, parent);

        // Thickness below minimum should give 0 beads
        let bead_count = strategy.get_optimal_bead_count(100_000);
        assert_eq!(bead_count, 0);
    }

    #[test]
    fn test_optimal_bead_count_one() {
        let parent = create_test_parent();
        let strategy = RedistributeBeadingStrategy::new(400_000, 0.34, parent);

        // Thickness for one bead
        let bead_count = strategy.get_optimal_bead_count(500_000);
        assert_eq!(bead_count, 1);
    }

    #[test]
    fn test_optimal_bead_count_two() {
        let parent = create_test_parent();
        let strategy = RedistributeBeadingStrategy::new(400_000, 0.34, parent);

        // Thickness for two beads (above split threshold)
        let bead_count = strategy.get_optimal_bead_count(700_000);
        assert_eq!(bead_count, 2);
    }

    #[test]
    fn test_compute_no_beads() {
        let parent = create_test_parent();
        let strategy = RedistributeBeadingStrategy::new(400_000, 0.34, parent);

        let beading = strategy.compute(100_000, 0);
        assert_eq!(beading.total_thickness, 100_000);
        assert_eq!(beading.left_over, 100_000);
        assert_eq!(beading.bead_widths.len(), 0);
        assert_eq!(beading.toolpath_locations.len(), 0);
    }

    #[test]
    fn test_compute_single_bead() {
        let parent = create_test_parent();
        let strategy = RedistributeBeadingStrategy::new(400_000, 0.34, parent);

        let beading = strategy.compute(500_000, 1);
        assert_eq!(beading.total_thickness, 500_000);
        assert_eq!(beading.bead_widths.len(), 1);
        assert_eq!(beading.bead_widths[0], 500_000);
        assert_eq!(beading.toolpath_locations.len(), 1);
        assert_eq!(beading.toolpath_locations[0], 250_000);
    }

    #[test]
    fn test_compute_two_beads() {
        let parent = create_test_parent();
        let strategy = RedistributeBeadingStrategy::new(400_000, 0.34, parent);

        let beading = strategy.compute(800_000, 2);
        assert_eq!(beading.total_thickness, 800_000);
        assert_eq!(beading.bead_widths.len(), 2);
        // Two outer walls, symmetric
        assert_eq!(beading.bead_widths[0], 400_000);
        assert_eq!(beading.bead_widths[1], 400_000);
        assert_eq!(beading.toolpath_locations.len(), 2);
    }

    #[test]
    fn test_transition_thickness() {
        let parent = create_test_parent();
        let strategy = RedistributeBeadingStrategy::new(400_000, 0.34, parent);

        // Case 0
        let t0 = strategy.get_transition_thickness(0);
        assert_eq!(t0, (0.34 * 400_000.0) as Coord);

        // Case 1
        let t1 = strategy.get_transition_thickness(1);
        assert!(t1 > 400_000);
    }
}
