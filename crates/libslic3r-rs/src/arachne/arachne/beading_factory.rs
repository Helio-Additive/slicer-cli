//! Beading strategy factory.
//!
//! Creates appropriate beading strategies based on configuration.

use super::beading::{BeadingCalculator, BeadingStrategy};

/// Type of beading strategy
/// Arachne/BeadingStrategy/BeadingStrategyFactory.hpp:13-20
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Enumeration for selecting beading strategy type
/// Arachne/BeadingStrategy/BeadingStrategyFactory.hpp:15-18
pub enum BeadingStrategyType {
    Distributed,
    Limited,
}

/// Factory for creating beading strategies.
/// Factory for creating BeadingStrategyPtr instances with configured strategies
/// Arachne/BeadingStrategy/BeadingStrategyFactory.hpp:13-31
pub struct BeadingStrategyFactory;

/// Implementation of BeadingStrategyFactory methods
/// Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:18-62
impl BeadingStrategyFactory {
    // Create a beading calculator.
    // Creates a BeadingCalculator with the specified strategy and parameters
    // Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:34-62
    pub fn create(
        strategy_type: BeadingStrategyType,
        bead_width: f64,
        min_bead_width: f64,
        wall_count: usize,
    ) -> BeadingCalculator {
        /// Select beading strategy based on type
        /// Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:34-58
        let strategy = {
            /// Match on strategy type to create appropriate strategy
            /// Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:36-55
            match strategy_type {
                BeadingStrategyType::Distributed => BeadingStrategy::Distributed,
                BeadingStrategyType::Limited => BeadingStrategy::Distributed, // Use same for now
            }
        };

        /// Create and return BeadingCalculator with selected strategy
        /// Arachne/BeadingStrategy/BeadingStrategyFactory.cpp:60-61
        BeadingCalculator::new(strategy, bead_width, bead_width, min_bead_width, wall_count)
    }
}
