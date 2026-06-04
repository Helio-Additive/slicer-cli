//! Extrusion junction for variable-width perimeters.
//!
//! This module provides the `ExtrusionJunction` type which represents a single
//! point along a variable-width extrusion path, including position and width
//! information.
//!
//! # BambuStudio Reference
//!
//! This corresponds to `src/libslic3r/Arachne/utils/ExtrusionJunction.hpp`

use crate::geometry::Point;
use crate::{unscale, Coord, CoordF};

#[derive(Debug, Clone, Copy, PartialEq)]
/// A junction (vertex) in a variable-width extrusion path
/// Arachne/utils/ExtrusionJunction.hpp:19-20
pub struct ExtrusionJunction {
    /// The position of the centerline at this junction (in scaled coordinates)
    /// Arachne/utils/ExtrusionJunction.hpp:26
    pub position: Point,

    /// The extrusion width at this junction (in scaled coordinates)
    /// Arachne/utils/ExtrusionJunction.hpp:31
    pub width: Coord,

    /// Which perimeter/wall index this junction belongs to (0 = outermost wall)
    /// Arachne/utils/ExtrusionJunction.hpp:37
    pub perimeter_index: usize,
}

/// Implementation of ExtrusionJunction methods
/// Arachne/utils/ExtrusionJunction.cpp:10-24
impl ExtrusionJunction {
    // Create a new extrusion junction
    // Arachne/utils/ExtrusionJunction.cpp:10-15
    pub fn new(position: Point, width: Coord, perimeter_index: usize) -> Self {
        Self {
            position,
            width,
            perimeter_index,
        }
    }

    /// Create a junction with a specific width in millimeters
    /// Arachne/utils/ExtrusionJunction.cpp:10-15
    pub fn with_width_mm(position: Point, width_mm: CoordF, perimeter_index: usize) -> Self {
        Self {
            position,
            width: crate::scale(width_mm),
            perimeter_index,
        }
    }

    #[inline]
    /// Get the X coordinate (scaled)
    /// Arachne/utils/ExtrusionJunction.hpp:26
    pub fn x(&self) -> Coord {
        // Arachne/utils/ExtrusionJunction.hpp:26
        self.position.x
    }

    #[inline]
    /// Get the Y coordinate (scaled)
    /// Arachne/utils/ExtrusionJunction.hpp:26
    pub fn y(&self) -> Coord {
        // Arachne/utils/ExtrusionJunction.hpp:26
        self.position.y
    }

    #[inline]
    /// Get the extrusion width in millimeters
    /// Arachne/utils/ExtrusionJunction.hpp:31
    pub fn width_mm(&self) -> CoordF {
        // Arachne/utils/ExtrusionJunction.hpp:31
        unscale(self.width)
    }

    #[inline]
    /// Get the position as a Point
    /// Arachne/utils/ExtrusionJunction.hpp:26
    pub fn point(&self) -> Point {
        // Arachne/utils/ExtrusionJunction.hpp:26
        self.position
    }

    /// Calculate the distance to another junction
    /// Arachne/utils/ExtrusionJunction.hpp:50-53
    pub fn distance_to(&self, other: &ExtrusionJunction) -> CoordF {
        self.position.distance(&other.position)
    }

    /// Calculate the squared distance to another junction
    /// Arachne/utils/ExtrusionJunction.hpp:50-53
    pub fn distance_squared_to(&self, other: &ExtrusionJunction) -> i128 {
        self.position.distance_squared(&other.position)
    }

    /// Check if this junction has the same position as another (within tolerance)
    /// Arachne/utils/ExtrusionJunction.hpp:50-53
    pub fn coincides_with(&self, other: &ExtrusionJunction, tolerance: Coord) -> bool {
        self.distance_squared_to(other) <= (tolerance as i128) * (tolerance as i128)
    }

    /// Linear interpolation between two junctions
    /// Arachne/utils/ExtrusionJunction.hpp:50-53
    pub fn lerp(&self, other: &ExtrusionJunction, t: f64) -> ExtrusionJunction {
        // Calculate interpolated X coordinate
        // Arachne/utils/ExtrusionJunction.hpp:50-53
        let x = self.position.x as f64 + t * (other.position.x - self.position.x) as f64;
        // Calculate interpolated Y coordinate
        // Arachne/utils/ExtrusionJunction.hpp:50-53
        let y = self.position.y as f64 + t * (other.position.y - self.position.y) as f64;
        // Calculate interpolated width
        // Arachne/utils/ExtrusionJunction.hpp:50-53
        let w = self.width as f64 + t * (other.width - self.width) as f64;

        ExtrusionJunction {
            position: Point::new(x.round() as Coord, y.round() as Coord),
            width: w.round() as Coord,
            perimeter_index: self.perimeter_index,
        }
    }

    #[inline]
    /// Check if this junction is an external (outer) perimeter
    /// Arachne/utils/ExtrusionJunction.hpp:37
    pub fn is_external(&self) -> bool {
        // Arachne/utils/ExtrusionJunction.hpp:37
        self.perimeter_index == 0
    }
}

/// Conversion from tuple to ExtrusionJunction
/// Arachne/utils/ExtrusionJunction.cpp:10-15
impl From<(Point, Coord, usize)> for ExtrusionJunction {
    // Convert tuple (position, width, perimeter_index) to ExtrusionJunction
    // Arachne/utils/ExtrusionJunction.cpp:10-15
    fn from((position, width, perimeter_index): (Point, Coord, usize)) -> Self {
        // Arachne/utils/ExtrusionJunction.cpp:10-15
        Self::new(position, width, perimeter_index)
    }
}

/// A collection of extrusion junctions forming a path segment
/// Arachne/utils/ExtrusionJunction.hpp:61
pub type ExtrusionJunctions = Vec<ExtrusionJunction>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scale;

    #[test]
    fn test_junction_new() {
        let p = Point::new(scale(10.0), scale(20.0));
        let j = ExtrusionJunction::new(p, scale(0.45), 0);

        assert_eq!(j.x(), scale(10.0));
        assert_eq!(j.y(), scale(20.0));
        assert!((j.width_mm() - 0.45).abs() < 0.001);
        assert_eq!(j.perimeter_index, 0);
        assert!(j.is_external());
    }

    #[test]
    fn test_junction_with_width_mm() {
        let p = Point::new(scale(5.0), scale(5.0));
        let j = ExtrusionJunction::with_width_mm(p, 0.4, 1);

        assert!((j.width_mm() - 0.4).abs() < 0.001);
        assert!(!j.is_external());
    }

    #[test]
    fn test_junction_distance() {
        let j1 = ExtrusionJunction::new(Point::new(0, 0), scale(0.4), 0);
        let j2 = ExtrusionJunction::new(Point::new(scale(3.0), scale(4.0)), scale(0.4), 0);

        // distance_to returns scaled distance, so we need to unscale for mm
        let dist = j1.distance_to(&j2);
        let dist_mm = unscale(dist as Coord);
        assert!((dist_mm - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_junction_lerp() {
        let j1 = ExtrusionJunction::new(Point::new(0, 0), scale(0.3), 0);
        let j2 = ExtrusionJunction::new(Point::new(scale(10.0), scale(10.0)), scale(0.5), 0);

        let mid = j1.lerp(&j2, 0.5);

        assert!((unscale(mid.x()) - 5.0).abs() < 0.001);
        assert!((unscale(mid.y()) - 5.0).abs() < 0.001);
        assert!((mid.width_mm() - 0.4).abs() < 0.001);
    }

    #[test]
    fn test_junction_coincides() {
        let j1 = ExtrusionJunction::new(Point::new(scale(1.0), scale(1.0)), scale(0.4), 0);
        let j2 = ExtrusionJunction::new(Point::new(scale(1.005), scale(1.005)), scale(0.4), 0);
        let j3 = ExtrusionJunction::new(Point::new(scale(2.0), scale(2.0)), scale(0.4), 0);

        // j1 and j2 are very close (within 0.01mm)
        assert!(j1.coincides_with(&j2, scale(0.01)));
        // j1 and j3 are far apart
        assert!(!j1.coincides_with(&j3, scale(0.01)));
    }

    #[test]
    fn test_junction_from_tuple() {
        let j: ExtrusionJunction = (Point::new(100, 200), 450000, 2).into();
        assert_eq!(j.x(), 100);
        assert_eq!(j.y(), 200);
        assert_eq!(j.width, 450000);
        assert_eq!(j.perimeter_index, 2);
    }
}
