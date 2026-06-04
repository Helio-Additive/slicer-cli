//! Extrusion junction types for Arachne wall generation
//!
//! C++ Reference:
//! - Arachne/utils/ExtrusionJunction.hpp
//! - Arachne/utils/ExtrusionJunction.cpp
//!
//! **STATUS:** 🟡 PARTIAL - Basic types defined, full implementation needed

use crate::geometry::{Coord, Point};

/// A single extrusion junction with position and width
/// C++ Reference: Arachne/utils/ExtrusionJunction.hpp (class ExtrusionJunction)
#[derive(Debug, Clone, Copy)]
pub struct ExtrusionJunction {
    /// Position of the junction
    /// C++: Point p;
    pub p: Point,

    /// Width at this junction
    /// C++: coord_t w;
    pub w: Coord,

    /// Perimeter index (which wall this belongs to)
    /// C++: size_t perimeter_index;
    pub perimeter_index: usize,

    /// Whether this junction is marked for hole compensation
    /// C++: bool hole_compensation_flag; (implied by context in ExtrusionLine.cpp:241-246)
    pub hole_compensation_flag: bool,
}

impl ExtrusionJunction {
    /// Create a new ExtrusionJunction
    /// C++ Reference: Arachne/utils/ExtrusionJunction.hpp
    pub fn new(p: Point, w: Coord, perimeter_index: usize) -> Self {
        Self {
            p,
            w,
            perimeter_index,
            hole_compensation_flag: false,
        }
    }

    /// Create a new ExtrusionJunction with hole compensation flag
    /// C++ Reference: Arachne/utils/ExtrusionJunction.hpp (extended)
    pub fn with_hole_compensation(
        p: Point,
        w: Coord,
        perimeter_index: usize,
        hole_compensation: bool,
    ) -> Self {
        Self {
            p,
            w,
            perimeter_index,
            hole_compensation_flag: hole_compensation,
        }
    }
}

/// Type alias for a collection of extrusion junctions (a line)
/// C++ Reference: Arachne/utils/ExtrusionJunction.hpp
/// C++: using LineJunctions = std::vector<ExtrusionJunction>;
pub type LineJunctions = Vec<ExtrusionJunction>;
