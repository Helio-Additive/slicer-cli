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
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:54
    ///
    // Requirement: Given a constant bead_count, the output of each bead width
    // must change gradually along with the thickness.
    ///
    // Note: The bead_count might be different from the optimal_bead_count.
    ///
    // C++: virtual Beading compute(coord_t thickness, coord_t bead_count) const = 0;
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading;

    // The number of beads should we ideally use for a given model thickness.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:69
    // C++: virtual coord_t getOptimalBeadCount(coord_t thickness) const = 0;
    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord;

    // Get the strategy name for debugging.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:93
    // C++: virtual std::string toString() const;
    // BeadingStrategy.cpp:51
    // C++: std::string BeadingStrategy::toString() const
    // C++: {
    // BeadingStrategy.cpp:53
    // C++:     return name;
    fn name(&self) -> &str;

    // The ideal thickness for a given bead_count.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:59
    // C++: virtual coord_t getOptimalThickness(coord_t bead_count) const;
    fn get_optimal_thickness(&self, bead_count: Coord) -> Coord {
        // BeadingStrategy.cpp:66
        // C++: coord_t BeadingStrategy::getOptimalThickness(coord_t bead_count) const
        // C++: {
        // BeadingStrategy.cpp:68
        // C++:     return optimal_width * bead_count;
        // FIDELITY-NOTE(F2): C++ multiplies two `coord_t` (int32) values, which wraps
        // on overflow at 32 bits; crate Coord is i64 so this is wider. Realistic
        // optimal_width * bead_count stays within int32, so behavior matches; narrowing
        // Coord is the crate-wide F2 rework and is out of per-file scope.
        self.optimal_width() * bead_count
    }

    // The model thickness at which optimal_bead_count transitions from
    // lower_bead_count to lower_bead_count + 1.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:64
    // C++: virtual coord_t getTransitionThickness(coord_t lower_bead_count) const;
    fn get_transition_thickness(&self, lower_bead_count: Coord) -> Coord {
        // BeadingStrategy.cpp:71
        // C++: coord_t BeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
        // C++: {
        // BeadingStrategy.cpp:73
        // C++:     const coord_t lower_ideal_width  = getOptimalThickness(lower_bead_count);
        let lower_ideal_width = self.get_optimal_thickness(lower_bead_count);
        // BeadingStrategy.cpp:74
        // C++:     const coord_t higher_ideal_width = getOptimalThickness(lower_bead_count + 1);
        let higher_ideal_width = self.get_optimal_thickness(lower_bead_count + 1);
        // BeadingStrategy.cpp:75
        // C++:     const double  threshold          = lower_bead_count % 2 == 1 ? wall_split_middle_threshold : wall_add_middle_threshold;
        let threshold = if lower_bead_count % 2 == 1 {
            self.wall_split_middle_threshold()
        } else {
            self.wall_add_middle_threshold()
        };
        // BeadingStrategy.cpp:76
        // C++:     return lower_ideal_width + threshold * (higher_ideal_width - lower_ideal_width);
        // The C++ expression promotes `lower_ideal_width` to double, computes the
        // whole sum in double, then implicitly converts the result back to coord_t
        // (truncation toward zero). Mirror that exact order here.
        (lower_ideal_width as f64 + threshold * (higher_ideal_width - lower_ideal_width) as f64)
            as Coord
    }

    // The length of the transitioning region along the marked/significant regions of the skeleton.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:76
    ///
    // Transitions are used to smooth out the jumps in integer bead count;
    // the jumps turn into ramps with some incline defined by their length.
    ///
    // C++: virtual coord_t getTransitioningLength(coord_t lower_bead_count) const;
    fn get_transitioning_length(&self, lower_bead_count: Coord) -> Coord {
        // BeadingStrategy.cpp:31
        // C++: coord_t BeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
        // C++: {
        // BeadingStrategy.cpp:33
        // C++:     if (lower_bead_count == 0)
        if lower_bead_count == 0 {
            // BeadingStrategy.cpp:34
            // C++:         return scaled<coord_t>(0.01);
            // C++ `scaled<coord_t>(v)` is defined as `coord_t(v / SCALING_FACTOR)`
            // (Point.hpp:537-541) with SCALING_FACTOR = 0.00001 — a *truncating*
            // cast, NOT std::round. For 0.01 this is `0.01 / 0.00001`, which in f64
            // evaluates to 999.99999999999988... and truncates to 999.
            // crate::scaled() instead does `(v * 100000.0).round()` -> 1000, which
            // diverges by 1 unit. Reproduce the exact C++ expression here to match.
            // FIDELITY-NOTE(F2): result is a Coord(i64); C++ coord_t is int32. The
            // value (999) is well within int32 range, so width does not affect this.
            // libslic3r.h:58 -> C++ SCALING_FACTOR = 0.00001 (crate::SCALING_FACTOR
            // is its reciprocal 100_000.0, so we use the C++ literal directly).
            return (0.01 / 0.00001) as Coord;
        }
        // BeadingStrategy.cpp:35
        // C++:     return default_transition_length;
        self.default_transition_length()
    }

    // The fraction of the transition length to put between the lower end of the transition
    // and the point where the unsmoothed bead count jumps.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:83
    ///
    // Transitions are used to smooth out the jumps in integer bead count;
    // the jumps turn into ramps which could be positioned relative to the jump location.
    ///
    // C++: virtual float getTransitionAnchorPos(coord_t lower_bead_count) const;
    fn get_transition_anchor_pos(&self, lower_bead_count: Coord) -> f32 {
        // BeadingStrategy.cpp:38
        // C++: float BeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
        // C++: {
        // BeadingStrategy.cpp:40
        // C++:     coord_t lower_optimum = getOptimalThickness(lower_bead_count);
        let lower_optimum = self.get_optimal_thickness(lower_bead_count);
        // BeadingStrategy.cpp:41
        // C++:     coord_t transition_point = getTransitionThickness(lower_bead_count);
        let transition_point = self.get_transition_thickness(lower_bead_count);
        // BeadingStrategy.cpp:42
        // C++:     coord_t upper_optimum = getOptimalThickness(lower_bead_count + 1);
        let upper_optimum = self.get_optimal_thickness(lower_bead_count + 1);
        // BeadingStrategy.cpp:43
        // C++:     return 1.0 - float(transition_point - lower_optimum) / float(upper_optimum - lower_optimum);
        // C++ evaluation order: each integer difference is cast to `float`, the
        // division `float / float` is done in float, then `1.0` (a *double* literal)
        // promotes the float result to double for the subtraction, and the double
        // result is narrowed back to `float` on return. Mirror that exactly: do the
        // ratio in f32, the `1.0 - ...` in f64, then narrow to f32.
        (1.0_f64
            - ((transition_point - lower_optimum) as f32 / (upper_optimum - lower_optimum) as f32)
                as f64) as f32
    }

    // Get the locations in a bead count region where compute() exhibits a bend in the widths.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:91
    ///
    // Ordered from lower thickness to higher.
    // This is used to insert extra support bones into the skeleton, so that the resulting
    // beads in long trapezoids don't linearly change between the two ends.
    ///
    // C++: virtual std::vector<coord_t> getNonlinearThicknesses(coord_t lower_bead_count) const;
    fn get_nonlinear_thicknesses(&self, _lower_bead_count: Coord) -> Vec<Coord> {
        // BeadingStrategy.cpp:46
        // C++: std::vector<coord_t> BeadingStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const
        // C++: {
        // BeadingStrategy.cpp:48
        // C++:     return {};
        Vec::new()
    }

    // Get the threshold when a middle wall should be split into two.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:95
    // C++: double getSplitMiddleThreshold() const;
    fn get_split_middle_threshold(&self) -> f64 {
        // BeadingStrategy.cpp:56
        // C++: double BeadingStrategy::getSplitMiddleThreshold() const
        // C++: {
        // BeadingStrategy.cpp:58
        // C++:     return wall_split_middle_threshold;
        self.wall_split_middle_threshold()
    }

    // Get the transitioning angle.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:96
    // C++: double getTransitioningAngle() const;
    fn get_transitioning_angle(&self) -> f64 {
        // BeadingStrategy.cpp:61
        // C++: double BeadingStrategy::getTransitioningAngle() const
        // C++: {
        // BeadingStrategy.cpp:63
        // C++:     return transitioning_angle;
        self.transitioning_angle()
    }

    // Get the optimal bead width for this strategy.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:101
    // C++: coord_t optimal_width;
    fn optimal_width(&self) -> Coord;

    // Get the default transition length.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:107
    // C++: coord_t default_transition_length;
    fn default_transition_length(&self) -> Coord;

    // Get the transitioning angle value.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:113
    // C++: double transitioning_angle;
    fn transitioning_angle(&self) -> f64;

    // Get the wall split middle threshold.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:103
    // C++: double wall_split_middle_threshold;
    fn wall_split_middle_threshold(&self) -> f64;

    // Get the wall add middle threshold.
    // Arachne/BeadingStrategy/BeadingStrategy.hpp:105
    // C++: double wall_add_middle_threshold;
    fn wall_add_middle_threshold(&self) -> f64;
}

// Type alias for boxed beading strategy (equivalent to C++ std::unique_ptr<BeadingStrategy>)
// Arachne/BeadingStrategy/BeadingStrategy.hpp:116
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
