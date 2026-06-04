//! Beading strategies for Arachne variable-width perimeters.
//!
//! This module provides different strategies for distributing bead (extrusion)
//! widths when filling regions with variable-width walls.
//!
//! # Beading Strategies
//!
//! - **Center**: Place a single centered wall in thin regions
//! - **Distributed**: Evenly distribute width among all walls
//! - **InwardDistributed**: Distribute extra width to inner walls only
//! - **OuterOnly**: Keep outer wall at nominal width, adjust inner walls
//!
//! # BambuStudio Reference
//!
//! This corresponds to `src/libslic3r/Arachne/BeadingStrategy/` directory,
//! particularly:
//! - `BeadingStrategy.hpp`
//! - `DistributedBeadingStrategy.hpp`
//! - `LimitedBeadingStrategy.hpp`
//! - `WideningBeadingStrategy.hpp`

use crate::CoordF;

/// Strategy for distributing bead widths in variable-width perimeters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BeadingStrategy {
    /// Center strategy: places a single centered wall in very thin regions.
    /// Good for thin features that can only fit one wall.
    Center,

    /// Distributed strategy: evenly distributes extra width among all walls.
    /// This is the default and works well for most cases.
    #[default]
    Distributed,

    /// Inward distributed: keeps outer wall at nominal width, distributes
    /// extra width among inner walls only.
    /// Better dimensional accuracy on outer surfaces.
    InwardDistributed,

    /// Outer only: adjusts only the outer wall width when needed.
    /// Useful when inner walls must maintain exact width.
    OuterOnly,
}

impl BeadingStrategy {
    // Returns a human-readable name for the strategy.
    pub fn name(&self) -> &'static str {
        match self {
            BeadingStrategy::Center => "Center",
            BeadingStrategy::Distributed => "Distributed",
            BeadingStrategy::InwardDistributed => "Inward Distributed",
            BeadingStrategy::OuterOnly => "Outer Only",
        }
    }

    /// Returns a description of the strategy.
    pub fn description(&self) -> &'static str {
        match self {
            BeadingStrategy::Center => "Places a single centered wall in thin regions",
            BeadingStrategy::Distributed => "Evenly distributes width among all walls",
            BeadingStrategy::InwardDistributed => {
                "Distributes extra width to inner walls, keeping outer wall nominal"
            }
            BeadingStrategy::OuterOnly => "Adjusts only outer wall width, inner walls stay nominal",
        }
    }

    /// Calculate beading for a given thickness.
    /// This creates a calculator with the given parameters and computes the result.
    pub fn calculate(
        &self,
        thickness: CoordF,
        outer_width: CoordF,
        inner_width: CoordF,
    ) -> BeadingResult {
        let min_bead_width = outer_width.min(inner_width) * 0.25;
        let max_bead_width = (outer_width + inner_width) / 2.0 * 2.0;

        let calculator = BeadingCalculator::new(*self, outer_width, inner_width, min_bead_width, 3)
            .max_bead_width(max_bead_width);

        calculator.calculate(thickness)
    }
}

/// Result of a beading calculation for a given thickness.
#[derive(Debug, Clone)]
pub struct BeadingResult {
    /// The widths of each bead, from outer to inner.
    pub bead_widths: Vec<CoordF>,

    /// The positions (centerlines) of each bead relative to the outer edge.
    pub bead_positions: Vec<CoordF>,

    /// Total width covered by all beads.
    pub total_width: CoordF,

    /// Number of beads.
    pub bead_count: usize,

    /// Whether this is a valid beading (fits the region).
    pub is_valid: bool,
}

impl BeadingResult {
    // Create an empty/invalid result.
    pub fn empty() -> Self {
        Self {
            bead_widths: Vec::new(),
            bead_positions: Vec::new(),
            total_width: 0.0,
            bead_count: 0,
            is_valid: false,
        }
    }

    /// Create a result with a single bead.
    pub fn single(width: CoordF) -> Self {
        Self {
            bead_widths: vec![width],
            bead_positions: vec![width / 2.0],
            total_width: width,
            bead_count: 1,
            is_valid: true,
        }
    }

    /// Check if any beads exist.
    pub fn has_beads(&self) -> bool {
        self.bead_count > 0
    }

    /// Get the outer bead width (first bead).
    pub fn outer_width(&self) -> Option<CoordF> {
        self.bead_widths.first().copied()
    }

    /// Get the inner bead width (last bead).
    pub fn inner_width(&self) -> Option<CoordF> {
        self.bead_widths.last().copied()
    }

    /// Get the average bead width.
    pub fn average_width(&self) -> CoordF {
        if self.bead_widths.is_empty() {
            0.0
        } else {
            self.bead_widths.iter().sum::<CoordF>() / self.bead_widths.len() as CoordF
        }
    }
}

/// Calculator for beading based on a strategy.
#[derive(Debug, Clone)]
pub struct BeadingCalculator {
    /// The strategy to use.
    strategy: BeadingStrategy,

    /// Nominal (preferred) outer wall width (mm).
    nominal_outer_width: CoordF,

    /// Nominal (preferred) inner wall width (mm).
    nominal_inner_width: CoordF,

    /// Minimum allowed bead width (mm).
    min_bead_width: CoordF,

    /// Maximum allowed bead width (mm).
    max_bead_width: CoordF,

    /// Preferred number of walls.
    preferred_wall_count: usize,
}

impl BeadingCalculator {
    // Create a new beading calculator.
    pub fn new(
        strategy: BeadingStrategy,
        nominal_outer_width: CoordF,
        nominal_inner_width: CoordF,
        min_bead_width: CoordF,
        preferred_wall_count: usize,
    ) -> Self {
        Self {
            strategy,
            nominal_outer_width,
            nominal_inner_width,
            min_bead_width,
            // Max width is typically 2x nominal to handle wide gaps
            max_bead_width: nominal_inner_width * 2.0,
            preferred_wall_count,
        }
    }

    /// Create a calculator with default settings.
    pub fn with_defaults(nominal_width: CoordF) -> Self {
        Self::new(
            BeadingStrategy::Distributed,
            nominal_width,
            nominal_width,
            nominal_width * 0.25,
            3,
        )
    }

    /// Set the maximum bead width.
    pub fn max_bead_width(mut self, width: CoordF) -> Self {
        self.max_bead_width = width;
        self
    }

    /// Get the strategy.
    pub fn strategy(&self) -> BeadingStrategy {
        self.strategy
    }

    /// Calculate optimal number of beads for a given thickness.
    pub fn optimal_bead_count(&self, thickness: CoordF) -> usize {
        if thickness < self.min_bead_width {
            return 0;
        }

        // Simple heuristic: divide by nominal width, round to nearest
        let nominal_width = (self.nominal_outer_width + self.nominal_inner_width) / 2.0;
        let float_count = thickness / nominal_width;

        // Round to nearest integer, but at least 1 if thickness >= min
        let count = float_count.round() as usize;
        count.max(1)
    }

    /// Calculate beading for a given thickness.
    pub fn calculate(&self, thickness: CoordF) -> BeadingResult {
        // Too thin to print anything
        if thickness < self.min_bead_width {
            return BeadingResult::empty();
        }

        // Calculate optimal bead count
        let bead_count = self.optimal_bead_count(thickness);

        if bead_count == 0 {
            return BeadingResult::empty();
        }

        // Apply strategy-specific width distribution
        match self.strategy {
            BeadingStrategy::Center => self.calculate_center(thickness, bead_count),
            BeadingStrategy::Distributed => self.calculate_distributed(thickness, bead_count),
            BeadingStrategy::InwardDistributed => {
                self.calculate_inward_distributed(thickness, bead_count)
            }
            BeadingStrategy::OuterOnly => self.calculate_outer_only(thickness, bead_count),
        }
    }

    /// Calculate beading for a specific bead count.
    pub fn calculate_for_count(&self, thickness: CoordF, bead_count: usize) -> BeadingResult {
        if bead_count == 0 || thickness < self.min_bead_width {
            return BeadingResult::empty();
        }

        match self.strategy {
            BeadingStrategy::Center => self.calculate_center(thickness, bead_count),
            BeadingStrategy::Distributed => self.calculate_distributed(thickness, bead_count),
            BeadingStrategy::InwardDistributed => {
                self.calculate_inward_distributed(thickness, bead_count)
            }
            BeadingStrategy::OuterOnly => self.calculate_outer_only(thickness, bead_count),
        }
    }

    /// Center strategy: single centered bead or evenly spaced beads.
    fn calculate_center(&self, thickness: CoordF, bead_count: usize) -> BeadingResult {
        if bead_count == 1 {
            // Single centered bead uses the full thickness
            let width = thickness.min(self.max_bead_width);
            return BeadingResult::single(width);
        }

        // Multiple beads: distribute evenly
        self.calculate_distributed(thickness, bead_count)
    }

    /// Distributed strategy: evenly distribute width among all beads.
    fn calculate_distributed(&self, thickness: CoordF, bead_count: usize) -> BeadingResult {
        let width_per_bead = thickness / bead_count as CoordF;

        // Check if the resulting width is within bounds
        if width_per_bead < self.min_bead_width || width_per_bead > self.max_bead_width {
            // Try adjusting bead count
            if width_per_bead < self.min_bead_width && bead_count > 1 {
                // Reduce bead count
                return self.calculate_distributed(thickness, bead_count - 1);
            }
            // Still out of bounds, mark as potentially problematic but proceed
        }

        let bead_widths = vec![width_per_bead; bead_count];
        let mut bead_positions = Vec::with_capacity(bead_count);

        // Calculate positions (center of each bead)
        let mut position = width_per_bead / 2.0;
        for _ in 0..bead_count {
            bead_positions.push(position);
            position += width_per_bead;
        }

        BeadingResult {
            bead_widths,
            bead_positions,
            total_width: thickness,
            bead_count,
            is_valid: true,
        }
    }

    /// Inward distributed: keep outer at nominal, distribute rest to inner.
    fn calculate_inward_distributed(&self, thickness: CoordF, bead_count: usize) -> BeadingResult {
        if bead_count == 1 {
            // Single bead: use nominal outer width or thickness, whichever is smaller
            let width = thickness.min(self.nominal_outer_width);
            return BeadingResult::single(width);
        }

        let mut bead_widths = Vec::with_capacity(bead_count);
        let mut bead_positions = Vec::with_capacity(bead_count);

        // Outer bead uses nominal width (clamped to available space)
        let outer_width = self.nominal_outer_width.min(thickness / 2.0);
        bead_widths.push(outer_width);

        // Remaining thickness for inner beads
        let remaining = thickness - outer_width;
        let inner_count = bead_count - 1;

        if inner_count > 0 {
            let inner_width = remaining / inner_count as CoordF;
            for _ in 0..inner_count {
                bead_widths.push(inner_width);
            }
        }

        // Calculate positions
        let mut position = outer_width / 2.0;
        bead_positions.push(position);
        position += outer_width / 2.0;

        for i in 1..bead_count {
            let w = bead_widths[i];
            position += w / 2.0;
            bead_positions.push(position);
            position += w / 2.0;
        }

        BeadingResult {
            bead_widths,
            bead_positions,
            total_width: thickness,
            bead_count,
            is_valid: true,
        }
    }

    /// Outer only: adjust outer bead width, inner beads stay nominal.
    fn calculate_outer_only(&self, thickness: CoordF, bead_count: usize) -> BeadingResult {
        if bead_count == 1 {
            let width = thickness.min(self.max_bead_width);
            return BeadingResult::single(width);
        }

        let mut bead_widths = Vec::with_capacity(bead_count);
        let mut bead_positions = Vec::with_capacity(bead_count);

        // Inner beads use nominal width
        let inner_count = bead_count - 1;
        let total_inner_width = self.nominal_inner_width * inner_count as CoordF;

        // Outer bead gets whatever is left
        let outer_width = (thickness - total_inner_width).max(self.min_bead_width);

        bead_widths.push(outer_width);
        for _ in 0..inner_count {
            bead_widths.push(self.nominal_inner_width);
        }

        // Calculate positions
        let mut position = outer_width / 2.0;
        bead_positions.push(position);
        position += outer_width / 2.0;

        for i in 1..bead_count {
            let w = bead_widths[i];
            position += w / 2.0;
            bead_positions.push(position);
            position += w / 2.0;
        }

        BeadingResult {
            bead_widths,
            bead_positions,
            total_width: thickness,
            bead_count,
            is_valid: true,
        }
    }
}

impl Default for BeadingCalculator {
    fn default() -> Self {
        Self::with_defaults(0.45)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: CoordF, b: CoordF) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn test_beading_strategy_names() {
        assert_eq!(BeadingStrategy::Center.name(), "Center");
        assert_eq!(BeadingStrategy::Distributed.name(), "Distributed");
        assert_eq!(
            BeadingStrategy::InwardDistributed.name(),
            "Inward Distributed"
        );
        assert_eq!(BeadingStrategy::OuterOnly.name(), "Outer Only");
    }

    #[test]
    fn test_beading_strategy_default() {
        assert_eq!(BeadingStrategy::default(), BeadingStrategy::Distributed);
    }

    #[test]
    fn test_beading_result_empty() {
        let result = BeadingResult::empty();
        assert!(!result.is_valid);
        assert_eq!(result.bead_count, 0);
        assert!(!result.has_beads());
        assert!(result.outer_width().is_none());
    }

    #[test]
    fn test_beading_result_single() {
        let result = BeadingResult::single(0.45);
        assert!(result.is_valid);
        assert_eq!(result.bead_count, 1);
        assert!(result.has_beads());
        assert!(approx_eq(result.outer_width().unwrap(), 0.45));
        assert!(approx_eq(result.inner_width().unwrap(), 0.45));
        assert!(approx_eq(result.average_width(), 0.45));
    }

    #[test]
    fn test_calculator_optimal_bead_count() {
        let calc = BeadingCalculator::with_defaults(0.45);

        // Very thin - no beads
        assert_eq!(calc.optimal_bead_count(0.05), 0);

        // Around one bead width
        assert_eq!(calc.optimal_bead_count(0.45), 1);

        // Around two bead widths
        assert_eq!(calc.optimal_bead_count(0.9), 2);

        // Around three bead widths
        assert_eq!(calc.optimal_bead_count(1.35), 3);
    }

    #[test]
    fn test_calculator_distributed_single_bead() {
        let calc = BeadingCalculator::new(BeadingStrategy::Distributed, 0.45, 0.45, 0.1, 3);

        let result = calc.calculate(0.45);
        assert!(result.is_valid);
        assert_eq!(result.bead_count, 1);
        assert!(approx_eq(result.bead_widths[0], 0.45));
    }

    #[test]
    fn test_calculator_distributed_multiple_beads() {
        let calc = BeadingCalculator::new(BeadingStrategy::Distributed, 0.45, 0.45, 0.1, 3);

        let result = calc.calculate(0.9);
        assert!(result.is_valid);
        assert_eq!(result.bead_count, 2);

        // Both beads should be equal width
        assert!(approx_eq(result.bead_widths[0], 0.45));
        assert!(approx_eq(result.bead_widths[1], 0.45));

        // Positions should be centered within each bead
        assert!(approx_eq(result.bead_positions[0], 0.225));
        assert!(approx_eq(result.bead_positions[1], 0.675));
    }

    #[test]
    fn test_calculator_inward_distributed() {
        let calc = BeadingCalculator::new(BeadingStrategy::InwardDistributed, 0.4, 0.45, 0.1, 3);

        // Test with 1.2mm thickness (should get ~3 beads)
        let result = calc.calculate_for_count(1.2, 3);
        assert!(result.is_valid);
        assert_eq!(result.bead_count, 3);

        // Outer bead should be nominal outer width
        assert!(approx_eq(result.bead_widths[0], 0.4));

        // Inner beads split the remaining 0.8mm
        assert!(approx_eq(result.bead_widths[1], 0.4));
        assert!(approx_eq(result.bead_widths[2], 0.4));
    }

    #[test]
    fn test_calculator_outer_only() {
        let calc = BeadingCalculator::new(BeadingStrategy::OuterOnly, 0.45, 0.45, 0.1, 3);

        // Test with slightly more than 2 nominal widths
        let result = calc.calculate_for_count(1.0, 2);
        assert!(result.is_valid);
        assert_eq!(result.bead_count, 2);

        // Inner bead should be nominal
        assert!(approx_eq(result.bead_widths[1], 0.45));

        // Outer bead gets the remainder (1.0 - 0.45 = 0.55)
        assert!(approx_eq(result.bead_widths[0], 0.55));
    }

    #[test]
    fn test_calculator_too_thin() {
        let calc = BeadingCalculator::new(BeadingStrategy::Distributed, 0.45, 0.45, 0.1, 3);

        let result = calc.calculate(0.05);
        assert!(!result.is_valid);
        assert_eq!(result.bead_count, 0);
    }

    #[test]
    fn test_calculator_center_single() {
        let calc = BeadingCalculator::new(BeadingStrategy::Center, 0.45, 0.45, 0.1, 3);

        // For a thin region, center should produce single bead
        let result = calc.calculate_for_count(0.3, 1);
        assert!(result.is_valid);
        assert_eq!(result.bead_count, 1);
        assert!(approx_eq(result.bead_widths[0], 0.3));
    }

    #[test]
    fn test_calculator_default() {
        let calc = BeadingCalculator::default();
        assert_eq!(calc.strategy(), BeadingStrategy::Distributed);
        assert!(approx_eq(calc.nominal_outer_width, 0.45));
    }

    #[test]
    fn test_beading_result_average_width() {
        let result = BeadingResult {
            bead_widths: vec![0.4, 0.5, 0.6],
            bead_positions: vec![0.2, 0.65, 1.15],
            total_width: 1.5,
            bead_count: 3,
            is_valid: true,
        };

        assert!(approx_eq(result.average_width(), 0.5));
    }
}
