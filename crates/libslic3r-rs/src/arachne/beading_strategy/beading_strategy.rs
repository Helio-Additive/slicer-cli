//! Base beading strategy trait and types.
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/BeadingStrategy.hpp
//! - Arachne/BeadingStrategy/BeadingStrategy.cpp
//!
//! This module defines the core beading strategy interface used by Arachne
//! for determining how to distribute wall lines across a given thickness.

use crate::Coord;
use std::sync::Arc;

// Special marker for wall contours with 0 width (used to denote boundaries)
// Arachne/BeadingStrategy/BeadingStrategy.hpp:16
// C++: constexpr int WallContourMarkedWidth = 0;
pub const WALL_CONTOUR_MARKED_WIDTH: Coord = 0;

// Special marker for first wall contour
// Arachne/BeadingStrategy/BeadingStrategy.hpp:17
// C++: constexpr int FirstWallContourMarkedWidth = 1;
pub const FIRST_WALL_CONTOUR_MARKED_WIDTH: Coord = 1;

// The beading for a given horizontal model thickness.
// Arachne/BeadingStrategy/BeadingStrategy.hpp:23-29
#[derive(Debug, Clone, PartialEq)]
pub struct Beading {
    // Total thickness being covered
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:25
    // C++: coord_t total_thickness;
    pub total_thickness: Coord,

    // The line width of each bead from the outer inset inward
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:26
    // C++: std::vector<coord_t> bead_widths;
    pub bead_widths: Vec<Coord>,

    // The distance of the toolpath location of each bead from the outline
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:27
    // C++: std::vector<coord_t> toolpath_locations;
    pub toolpath_locations: Vec<Coord>,

    // The distance not covered by any bead; gap area
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:28
    // C++: coord_t left_over;
    pub left_over: Coord,
}

impl Default for Beading {
    fn default() -> Self {
        Self {
            total_thickness: 0,
            bead_widths: Vec::new(),
            toolpath_locations: Vec::new(),
            left_over: 0,
        }
    }
}

// Base trait for beading strategies.
// Arachne/BeadingStrategy/BeadingStrategy.hpp:31-115
///
// Strategy for covering a given (constant) horizontal model thickness with a number of beads.
// The beads may have different widths.
pub trait BeadingStrategy: Send + Sync {
    // Retrieve the bead widths with which to cover a given thickness.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:51
    ///
    // Requirement: Given a constant bead_count, the output of each bead width
    // must change gradually along with the thickness.
    ///
    // Note: The bead_count might be different from the optimal_bead_count.
    ///
    // C++: virtual Beading compute(coord_t thickness, coord_t bead_count) const = 0;
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading;

    // The number of beads should we ideally use for a given model thickness.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:61
    // C++: virtual coord_t getOptimalBeadCount(coord_t thickness) const = 0;
    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord;

    // Get the strategy name for debugging.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:98
    // C++: virtual std::string toString() const;
    fn name(&self) -> &str;

    // The ideal thickness for a given bead_count.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:56
    // C++: virtual coord_t getOptimalThickness(coord_t bead_count) const;
    fn get_optimal_thickness(&self, bead_count: Coord) -> Coord {
        // Default implementation
        // Arachne/BeadingStrategy/BeadingStrategy.cpp:30-32
        // C++: coord_t BeadingStrategy::getOptimalThickness(coord_t bead_count) const {
        // C++:     return bead_count * optimal_width;
        // C++: }
        bead_count * self.optimal_width()
    }

    // The model thickness at which optimal_bead_count transitions from
    // lower_bead_count to lower_bead_count + 1.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:61
    // C++: virtual coord_t getTransitionThickness(coord_t lower_bead_count) const;
    fn get_transition_thickness(&self, lower_bead_count: Coord) -> Coord {
        // Default implementation
        // Arachne/BeadingStrategy/BeadingStrategy.cpp:34-41
        // C++: coord_t BeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const {
        // C++:     if (lower_bead_count == 0) {
        // C++:         return 0;
        // C++:     }
        // C++:     return (lower_bead_count + 1) * optimal_width;
        // C++: }
        if lower_bead_count == 0 {
            0
        } else {
            (lower_bead_count + 1) * self.optimal_width()
        }
    }

    // The length of the transitioning region along the marked/significant regions of the skeleton.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:68
    ///
    // Transitions are used to smooth out the jumps in integer bead count;
    // the jumps turn into ramps with some incline defined by their length.
    ///
    // C++: virtual coord_t getTransitioningLength(coord_t lower_bead_count) const;
    fn get_transitioning_length(&self, _lower_bead_count: Coord) -> Coord {
        // Default implementation
        // Arachne/BeadingStrategy/BeadingStrategy.cpp:43-45
        // C++: coord_t BeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const {
        // C++:     return default_transition_length;
        // C++: }
        self.default_transition_length()
    }

    // The fraction of the transition length to put between the lower end of the transition
    // and the point where the unsmoothed bead count jumps.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:75
    ///
    // Transitions are used to smooth out the jumps in integer bead count;
    // the jumps turn into ramps which could be positioned relative to the jump location.
    ///
    // C++: virtual float getTransitionAnchorPos(coord_t lower_bead_count) const;
    fn get_transition_anchor_pos(&self, _lower_bead_count: Coord) -> f32 {
        // Default implementation
        // Arachne/BeadingStrategy/BeadingStrategy.cpp:47-49
        // C++: float BeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const {
        // C++:     return 0.5;
        // C++: }
        0.5
    }

    // Get the locations in a bead count region where compute() exhibits a bend in the widths.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:82
    ///
    // Ordered from lower thickness to higher.
    // This is used to insert extra support bones into the skeleton, so that the resulting
    // beads in long trapezoids don't linearly change between the two ends.
    ///
    // C++: virtual std::vector<coord_t> getNonlinearThicknesses(coord_t lower_bead_count) const;
    fn get_nonlinear_thicknesses(&self, _lower_bead_count: Coord) -> Vec<Coord> {
        // Default implementation
        // Arachne/BeadingStrategy/BeadingStrategy.cpp:51-53
        // C++: std::vector<coord_t> BeadingStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const {
        // C++:     return std::vector<coord_t>();
        // C++: }
        Vec::new()
    }

    // Get the threshold when a middle wall should be split into two.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:100
    // C++: double getSplitMiddleThreshold() const;
    fn get_split_middle_threshold(&self) -> f64 {
        // Default implementation
        // Arachne/BeadingStrategy/BeadingStrategy.cpp:60-62
        // C++: double BeadingStrategy::getSplitMiddleThreshold() const {
        // C++:     return wall_split_middle_threshold;
        // C++: }
        self.wall_split_middle_threshold()
    }

    // Get the transitioning angle.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:101
    // C++: double getTransitioningAngle() const;
    fn get_transitioning_angle(&self) -> f64 {
        // Default implementation
        // Arachne/BeadingStrategy/BeadingStrategy.cpp:64-66
        // C++: double BeadingStrategy::getTransitioningAngle() const {
        // C++:     return transitioning_angle;
        // C++: }
        self.transitioning_angle()
    }

    // Get the optimal bead width for this strategy.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:103
    fn optimal_width(&self) -> Coord;

    // Get the default transition length.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:105
    fn default_transition_length(&self) -> Coord;

    // Get the transitioning angle value.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:112
    fn transitioning_angle(&self) -> f64;

    // Get the wall split middle threshold.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:104
    fn wall_split_middle_threshold(&self) -> f64;

    // Get the wall add middle threshold.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:106
    fn wall_add_middle_threshold(&self) -> f64;
}

// Type alias for boxed beading strategy (equivalent to C++ std::unique_ptr<BeadingStrategy>)
// Arachne/BeadingStrategy/BeadingStrategy.hpp:117
// C++: using BeadingStrategyPtr = std::unique_ptr<BeadingStrategy>;
pub type BeadingStrategyPtr = Arc<dyn BeadingStrategy>;

// Helper function to create a BeadingStrategyPtr from a concrete strategy
pub fn make_strategy<T: BeadingStrategy + 'static>(strategy: T) -> BeadingStrategyPtr {
    Arc::new(strategy)
}

// Pi divided by a value - used for angle calculations
// Arachne/BeadingStrategy/BeadingStrategy.hpp:14
// C++: template<typename T> constexpr T pi_div(const T div) { return static_cast<T>(M_PI) / div; }
pub fn pi_div(div: f64) -> f64 {
    std::f64::consts::PI / div
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_beading_default() {
        let beading = Beading::default();
        assert_eq!(beading.total_thickness, 0);
        assert_eq!(beading.bead_widths.len(), 0);
        assert_eq!(beading.toolpath_locations.len(), 0);
        assert_eq!(beading.left_over, 0);
    }

    #[test]
    fn test_pi_div() {
        let result = std::f64::consts::PI / 3.0;
        let computed = pi_div(3.0);
        assert!((computed - result).abs() < 1e-10);
    }
}
