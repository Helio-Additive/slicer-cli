use super::beading::{BeadingResult, BeadingStrategy};
use crate::CoordF;

/// Strategy for outer wall contour beading
/// Arachne/BeadingStrategy/OuterWallContourStrategy.hpp:8-10
pub struct OuterWallContourStrategy {
    /// Bead width for outer wall
    /// Arachne/BeadingStrategy/OuterWallContourStrategy.hpp:24
    bead_width: CoordF,
}

/// Implementation of OuterWallContourStrategy methods
/// Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:8-40
impl OuterWallContourStrategy {
    // Create new outer wall contour strategy
    // Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:11
    pub fn new(bead_width: CoordF) -> Self {
        // Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:11
        Self { bead_width }
    }

    /// Compute beading for given thickness
    /// Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:14
    pub fn compute(&self, _thickness: CoordF) -> BeadingResult {
        // Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:14-20
        BeadingResult {
            bead_widths: vec![self.bead_width],
            bead_positions: vec![self.bead_width / 2.0],
            total_width: self.bead_width,
            bead_count: 1,
            is_valid: true,
        }
    }

    /// Get bead count for given thickness
    /// Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:17
    pub fn get_bead_count(&self, _thickness: CoordF) -> usize {
        // Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:17
        1
    }

    /// Get optimal thickness for this strategy
    /// Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:15
    pub fn get_optimal_thickness(&self) -> CoordF {
        // Arachne/BeadingStrategy/OuterWallContourStrategy.cpp:15
        self.bead_width
    }
}

/// Strategy for outer wall with inset beading
/// Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.hpp:8-10
pub struct OuterWallInsetBeadingStrategy {
    /// Bead width
    /// Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.hpp:24
    bead_width: CoordF,
    /// Inset distance
    /// Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.hpp:25
    inset: CoordF,
}

/// Implementation of OuterWallInsetBeadingStrategy methods
/// Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:8-50
impl OuterWallInsetBeadingStrategy {
    // Create new outer wall inset beading strategy
    // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:11
    pub fn new(bead_width: CoordF, inset: CoordF) -> Self {
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:11
        Self { bead_width, inset }
    }

    /// Compute beading for given thickness
    /// Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:14
    pub fn compute(&self, thickness: CoordF) -> BeadingResult {
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:14
        let available_thickness = thickness - 2.0 * self.inset;
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:15
        if available_thickness <= 0.0 {
            // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:16
            return BeadingResult::empty();
        }
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:18-24
        BeadingResult {
            bead_widths: vec![self.bead_width],
            bead_positions: vec![self.bead_width / 2.0],
            total_width: self.bead_width,
            bead_count: 1,
            is_valid: true,
        }
    }

    /// Get bead count for given thickness
    /// Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:27
    pub fn get_bead_count(&self, thickness: CoordF) -> usize {
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:27
        if thickness > 2.0 * self.inset {
            // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:28
            1
        } else {
            // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:30
            0
        }
    }

    /// Get optimal thickness for this strategy
    /// Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:35
    pub fn get_optimal_thickness(&self) -> CoordF {
        // Arachne/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp:35
        self.bead_width + 2.0 * self.inset
    }
}

/// Strategy that redistributes bead widths within constraints
/// Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp:8-10
pub struct RedistributeBeadingStrategy {
    /// Underlying beading strategy
    /// Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp:24
    strategy: BeadingStrategy,
    /// Minimum allowed bead width
    /// Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp:25
    min_bead_width: CoordF,
    /// Maximum allowed bead width
    /// Arachne/BeadingStrategy/RedistributeBeadingStrategy.hpp:26
    max_bead_width: CoordF,
}

/// Implementation of RedistributeBeadingStrategy methods
/// Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:8-50
impl RedistributeBeadingStrategy {
    // Create new redistribute beading strategy
    // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:11
    pub fn new(strategy: BeadingStrategy, min_bead_width: CoordF, max_bead_width: CoordF) -> Self {
        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:11-15
        Self {
            strategy,
            min_bead_width,
            max_bead_width,
        }
    }

    /// Compute beading with width redistribution
    /// Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:18
    pub fn compute(&self, thickness: CoordF) -> BeadingResult {
        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:18
        let mut result = self.strategy.calculate(thickness, 0.45, 0.45);

        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:21-23
        for bead in &mut result.bead_widths {
            // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:22
            *bead = bead.clamp(self.min_bead_width, self.max_bead_width);
        }

        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:25
        result
    }

    /// Get bead count for given thickness
    /// Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:28
    pub fn get_bead_count(&self, thickness: CoordF) -> usize {
        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:28
        self.strategy.calculate(thickness, 0.45, 0.45).bead_count
    }

    /// Get optimal thickness for this strategy
    /// Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:33
    pub fn get_optimal_thickness(&self) -> CoordF {
        // Arachne/BeadingStrategy/RedistributeBeadingStrategy.cpp:33
        0.45
    }
}

/// Strategy that widens thin regions to meet minimum thickness
/// Arachne/BeadingStrategy/WideningBeadingStrategy.hpp:8-10
pub struct WideningBeadingStrategy {
    /// Underlying beading strategy
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.hpp:24
    strategy: BeadingStrategy,
    /// Minimum input thickness to use
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.hpp:25
    min_input_thickness: CoordF,
}

/// Implementation of WideningBeadingStrategy methods
/// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:8-40
impl WideningBeadingStrategy {
    // Create new widening beading strategy
    // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:11
    pub fn new(strategy: BeadingStrategy, min_input_thickness: CoordF) -> Self {
        // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:11-13
        Self {
            strategy,
            min_input_thickness,
        }
    }

    /// Compute beading with thickness widening
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:16
    pub fn compute(&self, thickness: CoordF) -> BeadingResult {
        // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:16
        let effective_thickness = thickness.max(self.min_input_thickness);
        // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:17
        self.strategy.calculate(effective_thickness, 0.45, 0.45)
    }

    /// Get bead count for given thickness
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:20
    pub fn get_bead_count(&self, thickness: CoordF) -> usize {
        // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:20
        let effective_thickness = thickness.max(self.min_input_thickness);
        // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:21
        self.strategy
            .calculate(effective_thickness, 0.45, 0.45)
            .bead_count
    }

    /// Get optimal thickness for this strategy
    /// Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:26
    pub fn get_optimal_thickness(&self) -> CoordF {
        // Arachne/BeadingStrategy/WideningBeadingStrategy.cpp:26
        0.45
    }
}
