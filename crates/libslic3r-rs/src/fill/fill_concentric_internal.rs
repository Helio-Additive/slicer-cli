//! Concentric internal infill pattern.
//!
//! C++ Reference:
//! - Fill/FillConcentricInternal.hpp
//! - Fill/FillConcentricInternal.cpp
//!
//! This is a variant of concentric infill used internally for support and
//! certain dense regions. It generates concentric offset loops with optional
//! surface extrusion handling.

use crate::geometry::{ExPolygon, Polyline};
use crate::CoordF;

/// Internal concentric fill configuration.
/// FillConcentricInternal.hpp
#[derive(Debug, Clone)]
pub struct FillConcentricInternal {
    /// Spacing between concentric loops.
    pub spacing: CoordF,
}

impl FillConcentricInternal {
    /// Create a new FillConcentricInternal with given spacing.
    pub fn new(spacing: CoordF) -> Self {
        Self { spacing }
    }
}

impl Default for FillConcentricInternal {
    fn default() -> Self {
        Self { spacing: 0.0 }
    }
}

/// Generate concentric internal fill.
///
/// Delegates to the same concentric offset algorithm but with internal-specific defaults.
/// FillConcentricInternal.cpp: fill_surface_extrusion
pub fn fill_surface_extrusion(_fill_area: &[ExPolygon], _spacing: CoordF) -> Vec<Polyline> {
    // Delegates to concentric fill logic.
    // For internal fills the algorithm is the same as concentric but may have
    // different spacing/gap parameters.
    super::fill_concentric::generate_concentric_infill(_fill_area, _spacing)
}
