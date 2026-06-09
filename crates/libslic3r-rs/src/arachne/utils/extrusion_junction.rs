//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
// 1:1 faithful port of:
//   Arachne/utils/ExtrusionJunction.hpp
//   Arachne/utils/ExtrusionJunction.cpp
//
// coord_t -> i64 (Coord), Point mirrors C++ Slic3r::Point.

use crate::geometry::{Coord, Point};
use std::ops::Sub;

/// This struct represents one vertex in an extruded path.
///
/// It contains information on how wide the extruded path must be at this point,
/// and which perimeter it represents.
// ExtrusionJunction.hpp:19
#[derive(Debug, Clone, Copy)]
pub struct ExtrusionJunction {
    /// whether the junction is generated from a hole that needs compensation
    // ExtrusionJunction.hpp:24
    pub hole_compensation_flag: bool,
    /// The position of the centreline of the path when it reaches this junction.
    /// This is the position that should end up in the g-code eventually.
    // ExtrusionJunction.hpp:29
    pub p: Point,

    /// The width of the extruded path at this junction.
    // ExtrusionJunction.hpp:34
    pub w: Coord,

    /// Which perimeter this junction is part of.
    ///
    /// Perimeters are counted from the outside inwards. The outer wall has index
    /// 0.
    // ExtrusionJunction.hpp:42 (C++ stores this as size_t -> usize)
    pub perimeter_index: usize,
}

impl ExtrusionJunction {
    // ExtrusionJunction.hpp:44 / ExtrusionJunction.cpp:17
    // ExtrusionJunction(const Point p, const coord_t w, const coord_t perimeter_index, const bool hole_compensation = false);
    // C++ takes perimeter_index as coord_t and assigns it to the size_t field; here we
    // take it as usize directly to match the stored field type and all Rust callers.
    // This 3-argument form mirrors the C++ default argument `hole_compensation = false`.
    pub fn new(p: Point, w: Coord, perimeter_index: usize) -> Self {
        // ExtrusionJunction.cpp:17 — : p(p), w(w), perimeter_index(perimeter_index), hole_compensation_flag(compensation_flag)
        Self {
            p,
            w,
            perimeter_index,
            hole_compensation_flag: false,
        }
    }

    // ExtrusionJunction.cpp:17 — full constructor with the `compensation_flag` argument supplied explicitly.
    pub fn with_hole_compensation(
        p: Point,
        w: Coord,
        perimeter_index: usize,
        compensation_flag: bool,
    ) -> Self {
        // ExtrusionJunction.cpp:17 — : p(p), w(w), perimeter_index(perimeter_index), hole_compensation_flag(compensation_flag)
        Self {
            p,
            w,
            perimeter_index,
            hole_compensation_flag: compensation_flag,
        }
    }
}

// ExtrusionJunction.cpp:9-15 — bool ExtrusionJunction::operator ==(const ExtrusionJunction& other) const
impl PartialEq for ExtrusionJunction {
    fn eq(&self, other: &ExtrusionJunction) -> bool {
        // ExtrusionJunction.cpp:11-14
        self.p == other.p
            && self.w == other.w
            && self.perimeter_index == other.perimeter_index
            && self.hole_compensation_flag == other.hole_compensation_flag
    }
}

// ExtrusionJunction.hpp:49-52 — inline Point operator-(const ExtrusionJunction& a, const ExtrusionJunction& b)
impl Sub for ExtrusionJunction {
    type Output = Point;

    fn sub(self, other: ExtrusionJunction) -> Point {
        // ExtrusionJunction.hpp:51 — return a.p - b.p;
        self.p - other.p
    }
}

// ExtrusionJunction.hpp:49-52 — reference form, so callers holding &ExtrusionJunction can subtract without copying.
impl Sub for &ExtrusionJunction {
    type Output = Point;

    fn sub(self, other: &ExtrusionJunction) -> Point {
        // ExtrusionJunction.hpp:51 — return a.p - b.p;
        self.p - other.p
    }
}

// ExtrusionJunction.hpp:54-58
// Identity function, used to be able to make templated algorithms that do their operations on 'point-like' input.
// inline const Point& make_point(const ExtrusionJunction& ej)
pub fn make_point(ej: &ExtrusionJunction) -> &Point {
    // ExtrusionJunction.hpp:57 — return ej.p;
    &ej.p
}

// ExtrusionJunction.hpp:60 — using LineJunctions = std::vector<ExtrusionJunction>;
//<! The junctions along a line without further information. See \ref ExtrusionLine for a more extensive class.
pub type LineJunctions = Vec<ExtrusionJunction>;
// ExtrusionJunction.hpp:61 — using ExtrusionJunctions = std::vector<ExtrusionJunction>;
pub type ExtrusionJunctions = Vec<ExtrusionJunction>;
