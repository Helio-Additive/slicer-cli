//! Beading strategy factory for creating configured strategy chains.
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/BeadingStrategyFactory.hpp
//! - Arachne/BeadingStrategy/BeadingStrategyFactory.cpp
//!
//! This factory creates a chain of beading strategies wrapped in meta-strategies
//! to handle various printing scenarios (thin walls, outer wall offsets, etc).

use super::beading_strategy::BeadingStrategy;
use super::beading_strategy::BeadingStrategyPtr;
use super::distributed_beading_strategy::DistributedBeadingStrategy;
use super::limited_beading_strategy::LimitedBeadingStrategy;
use super::outer_wall_inset_beading_strategy::OuterWallInsetBeadingStrategy;
use super::redistribute_beading_strategy::RedistributeBeadingStrategy;
use super::widening_beading_strategy::WideningBeadingStrategy;
use crate::{scale, Coord};
use std::sync::Arc;

// Legacy type alias - use BeadingStrategyPtr instead
pub type BeadingStrategyRc = Arc<dyn BeadingStrategy>;

// Factory for creating beading strategy chains
// Arachne/BeadingStrategy/BeadingStrategyFactory.hpp:13-31
pub struct BeadingStrategyFactory;

// Implementation of BeadingStrategyFactory
// Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:18-64
impl BeadingStrategyFactory {
    // Create a complete beading strategy chain with all meta-strategies applied
    // Arachne/BeadingStrategy/BeadingStrategyFactory.hpp:15-28
    // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:18-64
    pub fn make_strategy(
        preferred_bead_width_outer: Coord,
        preferred_bead_width_inner: Coord,
        preferred_transition_length: Coord,
        transitioning_angle: f64,
        print_thin_walls: bool,
        min_bead_width: Coord,
        min_feature_size: Coord,
        wall_split_middle_threshold: f64,
        wall_add_middle_threshold: f64,
        max_bead_count: Coord,
        outer_wall_offset: Coord,
        inward_distributed_center_wall_count: i32,
        minimum_variable_line_ratio: f64,
    ) -> BeadingStrategyPtr {
        // Start with DistributedBeadingStrategy as the base
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:34
        // C++: BeadingStrategyPtr ret = std::make_unique<DistributedBeadingStrategy>(preferred_bead_width_inner, preferred_transition_length, transitioning_angle, wall_split_middle_threshold, wall_add_middle_threshold, inward_distributed_center_wall_count);
        let mut ret: BeadingStrategyPtr = Arc::new(DistributedBeadingStrategy::new(
            preferred_bead_width_inner,
            preferred_transition_length,
            transitioning_angle,
            wall_split_middle_threshold,
            wall_add_middle_threshold,
            inward_distributed_center_wall_count,
        ));

        // Log application of Redistribute meta-strategy
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:35
        // C++: BOOST_LOG_TRIVIAL(debug) << "Applying the Redistribute meta-strategy with outer-wall width = " << preferred_bead_width_outer << ", inner-wall width = " << preferred_bead_width_inner << ".";
        log::debug!(
            "Applying the Redistribute meta-strategy with outer-wall width = {}, inner-wall width = {}.",
            preferred_bead_width_outer,
            preferred_bead_width_inner
        );

        // Wrap in RedistributeBeadingStrategy
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:36
        // C++: ret = std::make_unique<RedistributeBeadingStrategy>(preferred_bead_width_outer, minimum_variable_line_ratio, std::move(ret));
        ret = Arc::new(RedistributeBeadingStrategy::new(
            preferred_bead_width_outer,
            minimum_variable_line_ratio,
            ret,
        ));

        // Apply WideningBeadingStrategy if thin wall printing is enabled
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:38-41
        // C++: if (print_thin_walls) {
        if print_thin_walls {
            // Log application of Widening meta-strategy
            // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:39
            // C++: BOOST_LOG_TRIVIAL(debug) << "Applying the Widening Beading meta-strategy with minimum input width " << min_feature_size << " and minimum output width " << min_bead_width << ".";
            log::debug!(
                "Applying the Widening Beading meta-strategy with minimum input width {} and minimum output width {}.",
                min_feature_size,
                min_bead_width
            );

            // Wrap in WideningBeadingStrategy
            // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:40
            // C++: ret = std::make_unique<WideningBeadingStrategy>(std::move(ret), min_feature_size, min_bead_width);
            ret = Arc::new(WideningBeadingStrategy::new(
                ret,
                min_feature_size,
                min_bead_width,
            ));
        }

        // Apply OuterWallInsetBeadingStrategy if outer wall offset is non-zero
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:42-45
        // C++: if (outer_wall_offset != 0) {
        if outer_wall_offset != 0 {
            // Log application of OuterWallOffset meta-strategy
            // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:43
            // C++: BOOST_LOG_TRIVIAL(debug) << "Applying the OuterWallOffset meta-strategy with offset = " << outer_wall_offset << ".";
            log::debug!(
                "Applying the OuterWallOffset meta-strategy with offset = {}.",
                outer_wall_offset
            );

            // Wrap in OuterWallInsetBeadingStrategy
            // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:44
            // C++: ret = std::make_unique<OuterWallInsetBeadingStrategy>(outer_wall_offset, std::move(ret));
            ret = Arc::new(OuterWallInsetBeadingStrategy::new(outer_wall_offset, ret));
        }

        // Note: OuterWallContourStrategy is disabled in C++ code (lines 48-51)
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:48-51
        // C++: #if 0
        // C++:     //Apply the OuterWallContourStrategy last, since that adds a 1-width marker wall to mark the boundary of first beading.
        // C++:     BOOST_LOG_TRIVIAL(debug) << "Applying the First Beading Contour Strategy.";
        // C++:     ret = std::make_unique<OuterWallContourStrategy>(std::move(ret));
        // C++: #endif
        // Commented out as in C++ - causes junctions with different idx to link together
        // TODO: Fix and re-enable later

        // Apply LimitedBeadingStrategy as the final wrapper
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:52-53
        // C++: //Apply the LimitedBeadingStrategy last, since that adds a 0-width marker wall which other beading strategies shouldn't touch.
        // C++: BOOST_LOG_TRIVIAL(debug) << "Applying the Limited Beading meta-strategy with maximum bead count = " << max_bead_count << ".";
        log::debug!(
            "Applying the Limited Beading meta-strategy with maximum bead count = {}.",
            max_bead_count
        );

        // Wrap in LimitedBeadingStrategy
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:54
        // C++: ret = std::make_unique<LimitedBeadingStrategy>(max_bead_count, std::move(ret));
        ret = Arc::new(LimitedBeadingStrategy::new(max_bead_count, ret));

        // Return the complete strategy chain
        // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:58
        // C++: return ret;
        ret
    }

    // Helper to create strategy with default parameters
    // Uses defaults from BeadingStrategyFactory.hpp:15-28
    pub fn make_default_strategy() -> BeadingStrategyPtr {
        // Default parameters from C++ header
        // Arachne/BeadingStrategy/BeadingStrategyFactory.hpp:15-28
        Self::make_strategy(
            scale(0.0005),              // preferred_bead_width_outer
            scale(0.0005),              // preferred_bead_width_inner
            scale(0.0004),              // preferred_transition_length
            std::f64::consts::PI / 4.0, // transitioning_angle (45 degrees)
            false,                      // print_thin_walls
            0,                          // min_bead_width
            0,                          // min_feature_size
            0.5,                        // wall_split_middle_threshold
            0.5,                        // wall_add_middle_threshold
            0,                          // max_bead_count
            0,                          // outer_wall_offset
            2,                          // inward_distributed_center_wall_count
            0.5,                        // minimum_variable_line_ratio
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_make_strategy_basic() {
        // Test basic strategy creation
        let strategy = BeadingStrategyFactory::make_strategy(
            scale(0.4),
            scale(0.4),
            scale(0.3),
            std::f64::consts::PI / 4.0,
            false,
            0,
            0,
            0.5,
            0.5,
            0,
            0,
            2,
            0.5,
        );

        // Strategy should be created successfully
        assert!(!Rc::ptr_eq(&strategy, &strategy));
    }

    #[test]
    fn test_make_default_strategy() {
        // Test default strategy creation
        let strategy = BeadingStrategyFactory::make_default_strategy();

        // Strategy should be created successfully
        assert!(!Rc::ptr_eq(&strategy, &strategy));
    }

    #[test]
    fn test_make_strategy_with_thin_walls() {
        // Test strategy creation with thin wall printing enabled
        let strategy = BeadingStrategyFactory::make_strategy(
            scale(0.4),
            scale(0.4),
            scale(0.3),
            std::f64::consts::PI / 4.0,
            true, // print_thin_walls = true
            scale(0.1),
            scale(0.2),
            0.5,
            0.5,
            0,
            0,
            2,
            0.5,
        );

        // Strategy should be created with WideningBeadingStrategy wrapper
        assert!(!Rc::ptr_eq(&strategy, &strategy));
    }

    #[test]
    fn test_make_strategy_with_outer_wall_offset() {
        // Test strategy creation with outer wall offset
        let strategy = BeadingStrategyFactory::make_strategy(
            scale(0.4),
            scale(0.4),
            scale(0.3),
            std::f64::consts::PI / 4.0,
            false,
            0,
            0,
            0.5,
            0.5,
            0,
            scale(0.1), // outer_wall_offset > 0
            2,
            0.5,
        );

        // Strategy should be created with OuterWallInsetBeadingStrategy wrapper
        assert!(!Rc::ptr_eq(&strategy, &strategy));
    }
}
