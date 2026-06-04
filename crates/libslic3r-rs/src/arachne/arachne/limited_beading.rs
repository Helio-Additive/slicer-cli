//! Limited beading strategy.
//!
//! Caps bead count to prevent too-thin extrusions.

use super::beading::{BeadingCalculator, BeadingResult, BeadingStrategy};
use crate::CoordF;

/// Create a limited beading calculator
/// Arachne/BeadingStrategy/LimitedBeadingStrategy.hpp:28-29
pub fn create_limited_calculator(
    bead_width: CoordF,
    min_bead_width: CoordF,
    max_bead_count: usize,
    wall_count: usize,
) -> BeadingCalculator {
    /// Create BeadingCalculator with distributed strategy and apply limits
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:28-36
    BeadingCalculator::new(
        BeadingStrategy::Distributed,
        bead_width,
        bead_width,
        min_bead_width,
        wall_count,
    )
    .max_bead_width(bead_width * 2.0)
}

/// Calculate limited beading for a given thickness
/// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:40-85
pub fn calculate_limited_beading(
    thickness: CoordF,
    bead_width: CoordF,
    min_bead_width: CoordF,
    max_bead_count: usize,
) -> BeadingResult {
    /// Calculate maximum beads that fit based on minimum width constraint
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:42-45
    let max_beads_by_width = (thickness / min_bead_width).floor() as usize;
    /// Cap bead count to maximum and ensure at least 1 bead
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:57-60
    let bead_count = max_beads_by_width.min(max_bead_count).max(1);

    /// Calculate actual width per bead
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:61-65
    let actual_width = thickness / bead_count as CoordF;

    /// Generate bead positions evenly distributed across thickness
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:68-75
    let bead_positions: Vec<CoordF> = (0..bead_count)
        .map(|i| {
            /// Calculate offset for this bead position
            /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:72-74
            let offset = (i as CoordF + 0.5) * actual_width;
            offset
        })
        .collect();

    /// Return beading result with calculated positions and widths
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:80-82
    BeadingResult {
        bead_widths: vec![actual_width; bead_count],
        bead_positions,
        total_width: thickness,
        bead_count,
        is_valid: true,
    }
}
