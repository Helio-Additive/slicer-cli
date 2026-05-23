//! Widening beading strategy implementation.
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/WideningBeadingStrategy.hpp
//! - Arachne/BeadingStrategy/WideningBeadingStrategy.cpp
//!
//! This is a meta-strategy that can be applied on any other beading strategy.
//! If the part is thinner than a single line, this strategy adjusts the part
//! so that it becomes the minimum thickness of one line. This way, tiny pieces
//! that are smaller than a single line will still be printed.

use super::beading_strategy::{Beading, BeadingStrategy, BeadingStrategyPtr};
use crate::Coord;

/// Widening beading strategy that ensures thin features meet minimum width requirements.
/// Arachne/BeadingStrategy/WideningBeadingStrategy.hpp:19-45
///
/// This is a meta-strategy that can be applied on any other beading strategy. If the part
/// is thinner than a single line, this strategy adjusts the part so that it becomes the
/// minimum thickness of one line. This way, tiny pieces that are smaller than a single
/// line will still be printed.
#[derive(Clone)]
pub struct WideningBeadingStrategy {
    /// Parent beading strategy
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.hpp:41
    /// C++: BeadingStrategyPtr parent;
    parent: BeadingStrategyPtr,

    /// Minimum input width below which no bead is generated
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.hpp:42
    /// C++: const coord_t min_input_width;
    min_input_width: Coord,

    /// Minimum output width for generated beads (widened from input)
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.hpp:43
    /// C++: const coord_t min_output_width;
    min_output_width: Coord,

    /// Strategy name for debugging
    name: String,
}

impl WideningBeadingStrategy {
    /// Create a new widening beading strategy
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:9-15
    ///
    /// # Arguments
    /// * `parent` - Parent beading strategy to wrap
    /// * `min_input_width` - Minimum input width to generate a bead
    /// * `min_output_width` - Minimum output width for the generated bead
    ///
    /// C++: WideningBeadingStrategy::WideningBeadingStrategy(BeadingStrategyPtr parent, const coord_t min_input_width, const coord_t min_output_width)
    /// C++:     : BeadingStrategy(*parent)
    /// C++:     , parent(std::move(parent))
    /// C++:     , min_input_width(min_input_width)
    /// C++:     , min_output_width(min_output_width)
    /// C++: {
    /// C++: }
    pub fn new(
        parent: BeadingStrategyPtr,
        min_input_width: Coord,
        min_output_width: Coord,
    ) -> Self {
        Self {
            parent,
            min_input_width,
            min_output_width,
            name: "WideningBeadingStrategy".to_string(),
        }
    }
}

impl BeadingStrategy for WideningBeadingStrategy {
    /// Compute beading for a given thickness and bead count
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:22-37
    ///
    /// C++: WideningBeadingStrategy::Beading WideningBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading {
        // Check if thickness is below optimal width
        // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:24-35
        // C++: if (thickness < optimal_width) {
        // C++:     Beading ret;
        // C++:     ret.total_thickness = thickness;
        // C++:     if (thickness >= min_input_width)
        // C++:     {
        // C++:         ret.bead_widths.emplace_back(std::max(thickness, min_output_width));
        // C++:         ret.toolpath_locations.emplace_back(thickness / 2);
        // C++:     } else {
        // C++:         ret.left_over = thickness;
        // C++:     }
        // C++:     return ret;
        if thickness < self.parent.optimal_width() {
            let mut ret = Beading::default();
            ret.total_thickness = thickness;

            if thickness >= self.min_input_width {
                ret.bead_widths
                    .push(std::cmp::max(thickness, self.min_output_width));
                ret.toolpath_locations.push(thickness / 2);
            } else {
                ret.left_over = thickness;
            }

            ret
        } else {
            // Delegate to parent strategy if thickness is sufficient
            // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:36-37
            // C++: } else {
            // C++:     return parent->compute(thickness, bead_count);
            // C++: }
            self.parent.compute(thickness, bead_count)
        }
    }

    /// Get the optimal bead count for a given thickness
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:54-61
    ///
    /// C++: coord_t WideningBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
    /// C++: {
    /// C++:     if (thickness < min_input_width)
    /// C++:         return 0;
    /// C++:     coord_t ret = parent->getOptimalBeadCount(thickness);
    /// C++:     if (thickness >= min_input_width && ret < 1)
    /// C++:         return 1;
    /// C++:     return ret;
    /// C++: }
    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord {
        if thickness < self.min_input_width {
            return 0;
        }

        let ret = self.parent.get_optimal_bead_count(thickness);
        if thickness >= self.min_input_width && ret < 1 {
            return 1;
        }

        ret
    }

    /// Get the strategy name
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:17-20
    ///
    /// C++: std::string WideningBeadingStrategy::toString() const
    /// C++: {
    /// C++:     return std::string("Widening+") + parent->toString();
    /// C++: }
    fn name(&self) -> &str {
        &self.name
    }

    /// Get the optimal thickness for a given bead count
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:39-42
    ///
    /// C++: coord_t WideningBeadingStrategy::getOptimalThickness(coord_t bead_count) const
    /// C++: {
    /// C++:     return parent->getOptimalThickness(bead_count);
    /// C++: }
    fn get_optimal_thickness(&self, bead_count: Coord) -> Coord {
        self.parent.get_optimal_thickness(bead_count)
    }

    /// Get the transition thickness for a given lower bead count
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:44-50
    ///
    /// C++: coord_t WideningBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
    /// C++: {
    /// C++:     if (lower_bead_count == 0)
    /// C++:         return min_input_width;
    /// C++:     else
    /// C++:         return parent->getTransitionThickness(lower_bead_count);
    /// C++: }
    fn get_transition_thickness(&self, lower_bead_count: Coord) -> Coord {
        if lower_bead_count == 0 {
            self.min_input_width
        } else {
            self.parent.get_transition_thickness(lower_bead_count)
        }
    }

    /// Get the transitioning length
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:63-66
    ///
    /// C++: coord_t WideningBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
    /// C++: {
    /// C++:     return parent->getTransitioningLength(lower_bead_count);
    /// C++: }
    fn get_transitioning_length(&self, lower_bead_count: Coord) -> Coord {
        self.parent.get_transitioning_length(lower_bead_count)
    }

    /// Get the transition anchor position
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:68-71
    ///
    /// C++: float WideningBeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
    /// C++: {
    /// C++:     return parent->getTransitionAnchorPos(lower_bead_count);
    /// C++: }
    fn get_transition_anchor_pos(&self, lower_bead_count: Coord) -> f32 {
        self.parent.get_transition_anchor_pos(lower_bead_count)
    }

    /// Get nonlinear thicknesses
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:73-79
    ///
    /// C++: std::vector<coord_t> WideningBeadingStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const
    /// C++: {
    /// C++:     std::vector<coord_t> ret;
    /// C++:     ret.emplace_back(min_output_width);
    /// C++:     std::vector<coord_t> pret = parent->getNonlinearThicknesses(lower_bead_count);
    /// C++:     ret.insert(ret.end(), pret.begin(), pret.end());
    /// C++:     return ret;
    /// C++: }
    fn get_nonlinear_thicknesses(&self, lower_bead_count: Coord) -> Vec<Coord> {
        let mut ret = Vec::new();
        ret.push(self.min_output_width);
        let mut pret = self.parent.get_nonlinear_thicknesses(lower_bead_count);
        ret.append(&mut pret);
        ret
    }

    /// Get the optimal width (delegates to parent)
    fn optimal_width(&self) -> Coord {
        self.parent.optimal_width()
    }

    /// Get the default transition length (delegates to parent)
    fn default_transition_length(&self) -> Coord {
        self.parent.default_transition_length()
    }

    /// Get the transitioning angle (delegates to parent)
    fn transitioning_angle(&self) -> f64 {
        self.parent.get_transitioning_angle()
    }

    /// Get the wall split middle threshold (delegates to parent)
    fn wall_split_middle_threshold(&self) -> f64 {
        self.parent.get_split_middle_threshold()
    }

    /// Get the wall add middle threshold (delegates to parent)
    fn wall_add_middle_threshold(&self) -> f64 {
        self.parent.wall_add_middle_threshold()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arachne::beading_strategy::{make_strategy, DistributedBeadingStrategy};

    /// Helper to create a parent strategy for testing
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
    fn test_widening_strategy_creation() {
        let parent = create_test_parent();
        let strategy = WideningBeadingStrategy::new(
            parent, 100_000, // min_input_width (0.1mm)
            150_000, // min_output_width (0.15mm)
        );

        assert_eq!(strategy.name(), "WideningBeadingStrategy");
        assert_eq!(strategy.min_input_width, 100_000);
        assert_eq!(strategy.min_output_width, 150_000);
    }

    #[test]
    fn test_compute_below_min_input() {
        let parent = create_test_parent();
        let strategy = WideningBeadingStrategy::new(parent, 100_000, 150_000);

        // Thickness below min_input_width
        let beading = strategy.compute(50_000, 1);
        assert_eq!(beading.total_thickness, 50_000);
        assert_eq!(beading.left_over, 50_000);
        assert_eq!(beading.bead_widths.len(), 0);
    }

    #[test]
    fn test_compute_widened_bead() {
        let parent = create_test_parent();
        let strategy = WideningBeadingStrategy::new(parent, 100_000, 150_000);

        // Thickness above min_input but below optimal - should widen to min_output
        let beading = strategy.compute(120_000, 1);
        assert_eq!(beading.total_thickness, 120_000);
        assert_eq!(beading.bead_widths.len(), 1);
        assert_eq!(beading.bead_widths[0], 150_000); // Widened to min_output_width
        assert_eq!(beading.toolpath_locations[0], 60_000); // thickness / 2
    }

    #[test]
    fn test_compute_delegates_to_parent() {
        let parent = create_test_parent();
        let strategy = WideningBeadingStrategy::new(parent, 100_000, 150_000);

        // Thickness above optimal - should delegate to parent
        let beading = strategy.compute(800_000, 2);
        assert_eq!(beading.total_thickness, 800_000);
        // Should have beads from parent strategy
        assert!(beading.bead_widths.len() > 0);
    }

    #[test]
    fn test_optimal_bead_count_below_min() {
        let parent = create_test_parent();
        let strategy = WideningBeadingStrategy::new(parent, 100_000, 150_000);

        let count = strategy.get_optimal_bead_count(50_000);
        assert_eq!(count, 0);
    }

    #[test]
    fn test_optimal_bead_count_forces_one() {
        let parent = create_test_parent();
        let strategy = WideningBeadingStrategy::new(parent, 100_000, 150_000);

        // Even if parent would return 0, we force 1 if above min_input
        let count = strategy.get_optimal_bead_count(120_000);
        assert_eq!(count, 1);
    }

    #[test]
    fn test_transition_thickness_zero_bead() {
        let parent = create_test_parent();
        let strategy = WideningBeadingStrategy::new(parent, 100_000, 150_000);

        let thickness = strategy.get_transition_thickness(0);
        assert_eq!(thickness, 100_000); // Returns min_input_width
    }

    #[test]
    fn test_nonlinear_thicknesses() {
        let parent = create_test_parent();
        let strategy = WideningBeadingStrategy::new(parent, 100_000, 150_000);

        let thicknesses = strategy.get_nonlinear_thicknesses(2);
        // Should have min_output_width as first element
        assert_eq!(thicknesses[0], 150_000);
    }
}
