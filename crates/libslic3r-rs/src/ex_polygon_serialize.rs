//! Serialization support for ExPolygon types using Serde
//!
//! C++ Reference:
//! - ExPolygonSerialize.hpp
//!
//! This module provides serialization/deserialization support for ExPolygon and Polygon
//! types. The C++ version uses Cereal library; Rust uses Serde which is the standard
//! serialization framework in the Rust ecosystem.
//!
//! ## C++ to Rust Mapping
//!
//! C++ uses Cereal's `serialize()` template functions:
//! ```cpp
//! template<class Archive>
//! void serialize(Archive &archive, Slic3r::Polygon &polygon) {
//!     archive(polygon.points);
//! }
//! ```
//!
//! Rust uses Serde's `#[derive(Serialize, Deserialize)]` attributes on the
//! Polygon and ExPolygon types directly (in geometry/mod.rs).

use crate::geometry::{ExPolygon, Point, Polygon};
use serde::{Deserialize, Serialize};

/// Re-export Polygon with serialization support
/// ExPolygonSerialize.hpp:18-20
/// C++: template<class Archive> void serialize(Archive &archive, Slic3r::Polygon &polygon)
///
/// In Rust, the Polygon type should already derive Serialize/Deserialize.
/// This module ensures the serialization trait bounds are available.
pub trait PolygonSerialize: Serialize + for<'de> Deserialize<'de> {}

impl PolygonSerialize for Polygon {}

/// Re-export ExPolygon with serialization support
/// ExPolygonSerialize.hpp:22-25
/// C++: template<class Archive> void serialize(Archive &archive, Slic3r::ExPolygon &expoly)
///
/// In Rust, the ExPolygon type should already derive Serialize/Deserialize.
/// This module ensures the serialization trait bounds are available.
pub trait ExPolygonSerialize: Serialize + for<'de> Deserialize<'de> {}

impl ExPolygonSerialize for ExPolygon {}

/// Serialize a Polygon to JSON string
/// Convenience function for common use case
pub fn serialize_polygon_to_json(polygon: &Polygon) -> Result<String, serde_json::Error> {
    serde_json::to_string(polygon)
}

/// Deserialize a Polygon from JSON string
/// Convenience function for common use case
pub fn deserialize_polygon_from_json(json: &str) -> Result<Polygon, serde_json::Error> {
    serde_json::from_str(json)
}

/// Serialize an ExPolygon to JSON string
/// Convenience function for common use case
pub fn serialize_expolygon_to_json(expolygon: &ExPolygon) -> Result<String, serde_json::Error> {
    serde_json::to_string(expolygon)
}

/// Deserialize an ExPolygon from JSON string
/// Convenience function for common use case
pub fn deserialize_expolygon_from_json(json: &str) -> Result<ExPolygon, serde_json::Error> {
    serde_json::from_str(json)
}

/// Serialize a vector of ExPolygons to JSON string
pub fn serialize_expolygons_to_json(expolygons: &[ExPolygon]) -> Result<String, serde_json::Error> {
    serde_json::to_string(expolygons)
}

/// Deserialize a vector of ExPolygons from JSON string
pub fn deserialize_expolygons_from_json(json: &str) -> Result<Vec<ExPolygon>, serde_json::Error> {
    serde_json::from_str(json)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_polygon_json_roundtrip() {
        let points = vec![
            Point::new(0, 0),
            Point::new(1000, 0),
            Point::new(1000, 1000),
            Point::new(0, 1000),
        ];
        let polygon = Polygon::new(points);

        // Serialize to JSON
        let json = serialize_polygon_to_json(&polygon).unwrap();
        assert!(!json.is_empty());

        // Deserialize back
        let deserialized = deserialize_polygon_from_json(&json).unwrap();
        assert_eq!(polygon, deserialized);
    }

    #[test]
    fn test_expolygon_json_roundtrip() {
        let contour_points = vec![
            Point::new(0, 0),
            Point::new(2000, 0),
            Point::new(2000, 2000),
            Point::new(0, 2000),
        ];
        let contour = Polygon::new(contour_points);

        let hole_points = vec![
            Point::new(500, 500),
            Point::new(1500, 500),
            Point::new(1500, 1500),
            Point::new(500, 1500),
        ];
        let hole = Polygon::new(hole_points);

        let expolygon = ExPolygon::new(contour, vec![hole]);

        // Serialize to JSON
        let json = serialize_expolygon_to_json(&expolygon).unwrap();
        assert!(!json.is_empty());

        // Deserialize back
        let deserialized = deserialize_expolygon_from_json(&json).unwrap();
        assert_eq!(expolygon, deserialized);
    }

    #[test]
    fn test_expolygons_vector_roundtrip() {
        let poly1 = ExPolygon::new(
            Polygon::new(vec![
                Point::new(0, 0),
                Point::new(1000, 0),
                Point::new(1000, 1000),
                Point::new(0, 1000),
            ]),
            vec![],
        );

        let poly2 = ExPolygon::new(
            Polygon::new(vec![
                Point::new(2000, 2000),
                Point::new(3000, 2000),
                Point::new(3000, 3000),
                Point::new(2000, 3000),
            ]),
            vec![],
        );

        let expolygons = vec![poly1, poly2];

        // Serialize to JSON
        let json = serialize_expolygons_to_json(&expolygons).unwrap();
        assert!(!json.is_empty());

        // Deserialize back
        let deserialized = deserialize_expolygons_from_json(&json).unwrap();
        assert_eq!(expolygons.len(), deserialized.len());
        assert_eq!(expolygons[0], deserialized[0]);
        assert_eq!(expolygons[1], deserialized[1]);
    }

    #[test]
    fn test_expolygon_with_multiple_holes() {
        let contour = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(5000, 0),
            Point::new(5000, 5000),
            Point::new(0, 5000),
        ]);

        let hole1 = Polygon::new(vec![
            Point::new(500, 500),
            Point::new(1500, 500),
            Point::new(1500, 1500),
            Point::new(500, 1500),
        ]);

        let hole2 = Polygon::new(vec![
            Point::new(3000, 3000),
            Point::new(4000, 3000),
            Point::new(4000, 4000),
            Point::new(3000, 4000),
        ]);

        let expolygon = ExPolygon::new(contour, vec![hole1, hole2]);

        // Serialize and deserialize
        let json = serialize_expolygon_to_json(&expolygon).unwrap();
        let deserialized = deserialize_expolygon_from_json(&json).unwrap();

        assert_eq!(expolygon.contour, deserialized.contour);
        assert_eq!(expolygon.holes.len(), deserialized.holes.len());
        assert_eq!(expolygon.holes[0], deserialized.holes[0]);
        assert_eq!(expolygon.holes[1], deserialized.holes[1]);
    }

    #[test]
    fn test_empty_polygon() {
        let polygon = Polygon::new(vec![]);

        let json = serialize_polygon_to_json(&polygon).unwrap();
        let deserialized = deserialize_polygon_from_json(&json).unwrap();

        assert_eq!(polygon, deserialized);
        assert!(deserialized.points.is_empty());
    }

    #[test]
    fn test_expolygon_no_holes() {
        let contour = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(1000, 0),
            Point::new(1000, 1000),
            Point::new(0, 1000),
        ]);

        let expolygon = ExPolygon::new(contour, vec![]);

        let json = serialize_expolygon_to_json(&expolygon).unwrap();
        let deserialized = deserialize_expolygon_from_json(&json).unwrap();

        assert_eq!(expolygon, deserialized);
        assert!(deserialized.holes.is_empty());
    }
}
