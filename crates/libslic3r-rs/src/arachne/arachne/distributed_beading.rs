//! Distributed beading strategy.
//!
//! Distributes bead widths evenly across available space.

use super::beading::{BeadingCalculator, BeadingResult, BeadingStrategy};
use crate::CoordF;

/// Create a distributed beading calculator
/// Arachne/BeadingStrategy/DistributedBeadingStrategy.hpp:26-31
pub fn create_distributed_calculator(
    bead_width: CoordF,
    min_bead_width: CoordF,
    wall_count: usize,
) -> BeadingCalculator {
    /// Create BeadingCalculator with distributed strategy
    /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:9-22
    BeadingCalculator::new(
        BeadingStrategy::Distributed,
        bead_width,
        bead_width,
        min_bead_width,
        wall_count,
    )
}

/// Calculate distributed beading for a given thickness
/// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:24-88
pub fn calculate_distributed_beading(
    thickness: CoordF,
    bead_width: CoordF,
    min_bead_width: CoordF,
) -> BeadingResult {
    /// Check if thickness is too small for any beads
    /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:26-27
    if thickness < min_bead_width {
        /// Return empty result for insufficient thickness
        /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:72-74
        return BeadingResult::empty();
    }

    /// Calculate optimal bead count based on thickness
    /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:90-95
    let bead_count = (thickness / bead_width).round() as usize;
    /// Ensure at least one bead
    /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:94
    let bead_count = bead_count.max(1);

    /// Calculate actual width per bead by distributing thickness evenly
    /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:28-60
    let actual_width = thickness / bead_count as CoordF;

    /// Generate bead positions evenly distributed across thickness
    /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:48-56
    let bead_positions: Vec<CoordF> = (0..bead_count)
        .map(|i| {
            /// Calculate offset for this bead position
            /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:51-53
            let offset = (i as CoordF + 0.5) * actual_width;
            offset
        })
        .collect();

    /// Return beading result with calculated positions and widths
    /// Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:85-87
    BeadingResult {
        bead_widths: vec![actual_width; bead_count],
        bead_positions,
        total_width: thickness,
        bead_count,
        is_valid: true,
    }
}
