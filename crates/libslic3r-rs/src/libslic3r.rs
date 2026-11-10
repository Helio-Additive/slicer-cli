//! Main library header - common definitions and exports
//!
//! This module serves as the Rust equivalent of libslic3r.h, providing
//! core type definitions, constants, and re-exports used throughout the library.

/// C++ Reference: libslic3r.h
/// Main header file for the libslic3r library
use crate::Result;

// ---------------------------------------------------------------------------
// Core Constants
// ---------------------------------------------------------------------------

/// Scaling factor for coordinate precision (1 mm = 100,000 scaled units)
/// libslic3r.h:58
/// C++: static constexpr double SCALING_FACTOR = 0.00001;
/// NOTE: 0.000001 is the old PrusaSlicer value — BambuStudio uses 0.00001,
/// matching the crate-wide convention (lib.rs SCALING_FACTOR = 100_000.0).
pub const SCALING_FACTOR: f64 = 0.00001;

/// Epsilon for floating point comparisons
/// libslic3r.h:45
/// C++: constexpr double EPSILON = 1e-4;
pub const EPSILON: f64 = 1e-4;

/// Tolerance for geometric operations
/// libslic3r.h:48
/// C++: constexpr double SCALED_EPSILON = 10.0;
pub const SCALED_EPSILON: f64 = 10.0;

/// Overlap tolerance for perimeter/infill operations
/// libslic3r.h:72
/// C++: constexpr double INSET_OVERLAP_TOLERANCE = 0.4;
pub const INSET_OVERLAP_TOLERANCE: f64 = 0.4;

// ---------------------------------------------------------------------------
// Coordinate Scaling
// ---------------------------------------------------------------------------

/// Convert millimeters to scaled internal coordinates
/// libslic3r.h:55
/// C++: inline coord_t scale_(coordf_t v) { return coord_t(floor(v / SCALING_FACTOR + 0.5)); }
#[inline]
pub fn scale(mm: f64) -> i64 {
    (mm / SCALING_FACTOR + 0.5).floor() as i64
}

/// Convert scaled internal coordinates to millimeters
/// libslic3r.h:58
/// C++: inline coordf_t unscale(coord_t v) { return coordf_t(v) * SCALING_FACTOR; }
#[inline]
pub fn unscale(coord: i64) -> f64 {
    coord as f64 * SCALING_FACTOR
}

// ---------------------------------------------------------------------------
// Library Initialization
// ---------------------------------------------------------------------------

/// Initialize the library (version checks, etc.)
/// libslic3r.h:89
/// C++: void libslic3r_init();
pub fn init() -> Result<()> {
    /// Perform any necessary initialization
    /// libslic3r.h:90-95
    /// C++: void libslic3r_init() {
    /// C++:     // Initialize logging, check versions, etc.
    /// C++: }
    Ok(())
}

/// Get library version string
/// libslic3r.h:98
/// C++: const char* libslic3r_version();
pub fn version() -> &'static str {
    /// Return version information
    /// libslic3r.h:99
    /// C++: return SLIC3R_VERSION;
    env!("CARGO_PKG_VERSION")
}

// ---------------------------------------------------------------------------
// Re-exports
// ---------------------------------------------------------------------------

pub use crate::geometry::{BoundingBox, BoundingBox3};
pub use crate::geometry::{ExPolygon, Line, Polygon, Polyline};
/// Re-export core types for convenience
/// These are the fundamental types used throughout libslic3r
pub use crate::geometry::{Point, Point3};

// Type aliases for C++ compatibility
// Point.hpp: Vec2d is the floating-point 2D point type
// C++: using Pointfs = std::vector<Vec2d>;
pub use crate::geometry::PointF as Pointf;

// Point.hpp: Vec3d is the floating-point 3D point type
// C++: using Pointf3s = std::vector<Vec3d>;
pub use crate::geometry::Point3F as Pointf3;

// BoundingBox.hpp: Floating-point bounding box type aliases
pub use crate::geometry::BoundingBox3F as BoundingBoxf3;
pub use crate::geometry::BoundingBoxF as BoundingBoxf;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scaling() {
        /// Test coordinate scaling round-trip
        /// Verify scale/unscale preserve values within epsilon
        let mm = 123.456;
        let scaled = scale(mm);
        let unscaled = unscale(scaled);
        assert!((mm - unscaled).abs() < EPSILON);
    }

    #[test]
    fn test_version() {
        /// Verify version string is available
        let v = version();
        assert!(!v.is_empty());
    }
}
