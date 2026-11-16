//! External serialization of ExPolygons.
//!
//! C++ Reference:
//! - ExPolygonSerialize.hpp
//!
//! ## C++ -> Rust mapping
//!
//! The C++ header provides *external* Cereal `serialize()` template functions for
//! `Slic3r::Polygon` and `Slic3r::ExPolygon`. Cereal dispatches a single `serialize`
//! function for both saving and loading, archiving the listed members in order:
//!
//! ```cpp
//! template<class Archive>
//! void serialize(Archive &archive, Slic3r::Polygon &polygon) {
//!     archive(polygon.points);
//! }
//!
//! template<class Archive>
//! void serialize(Archive &archive, Slic3r::ExPolygon &expoly) {
//!     archive(expoly.contour, expoly.holes);
//! }
//! ```
//!
//! In Rust the equivalent of an external Cereal `serialize` is a function over a
//! `serde::Serializer`. The derived `Serialize` of the crate's `Polygon` and `ExPolygon` is not
//! that: it wraps the members in a struct with named fields (`{"points": [...]}`), where Cereal
//! archives the members themselves. The two functions below archive exactly what the C++ does:
//! a polygon is its `points`, an expolygon is its contour (as a polygon, i.e. its points) followed
//! by its holes (a vector of polygons). Cereal archives the two members one after the other with
//! no grouping, which is a serde 2-tuple: binary formats write a tuple's items back to back with
//! no length, as Cereal's binary archive does.

use crate::geometry::{ExPolygon, Polygon};
use serde::ser::{SerializeSeq, SerializeTuple};
use serde::{Serialize, Serializer};

// ExPolygonSerialize.hpp:15  namespace cereal {

/// ExPolygonSerialize.hpp:17-20
/// C++:
/// ```cpp
/// template<class Archive>
/// void serialize(Archive &archive, Slic3r::Polygon &polygon) {
///     archive(polygon.points);
/// }
/// ```
pub fn serialize_polygon<A>(archive: A, polygon: &Polygon) -> Result<A::Ok, A::Error>
where
    A: Serializer,
{
    // ExPolygonSerialize.hpp:19  archive(polygon.points);
    polygon.points.serialize(archive)
}

/// A polygon archived by `serialize_polygon`, for the places where Cereal archives a `Polygon`
/// member through the function above (the contour and each hole of an expolygon).
struct CerealPolygon<'a>(&'a Polygon);

impl Serialize for CerealPolygon<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serialize_polygon(serializer, self.0)
    }
}

/// `std::vector<Polygon>` as `cereal/types/vector.hpp` archives it: the size, then each polygon
/// through `serialize_polygon`.
struct CerealPolygons<'a>(&'a [Polygon]);

impl Serialize for CerealPolygons<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut seq = serializer.serialize_seq(Some(self.0.len()))?;
        for polygon in self.0 {
            seq.serialize_element(&CerealPolygon(polygon))?;
        }
        seq.end()
    }
}

/// ExPolygonSerialize.hpp:22-25
/// C++:
/// ```cpp
/// template<class Archive>
/// void serialize(Archive &archive, Slic3r::ExPolygon &expoly) {
///     archive(expoly.contour, expoly.holes);
/// }
/// ```
pub fn serialize_ex_polygon<A>(archive: A, expoly: &ExPolygon) -> Result<A::Ok, A::Error>
where
    A: Serializer,
{
    // ExPolygonSerialize.hpp:24  archive(expoly.contour, expoly.holes);
    let mut members = archive.serialize_tuple(2)?;
    members.serialize_element(&CerealPolygon(&expoly.contour))?;
    members.serialize_element(&CerealPolygons(&expoly.holes))?;
    members.end()
}

// ExPolygonSerialize.hpp:27  } // namespace cereal

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};

    fn square(o: crate::Coord, s: crate::Coord) -> Polygon {
        Polygon::from_points(vec![
            Point::new(o, o),
            Point::new(o + s, o),
            Point::new(o + s, o + s),
            Point::new(o, o + s),
        ])
    }

    #[test]
    fn serialize_polygon_matches_derive() {
        // serialize_polygon must produce exactly the same output as serializing
        // `polygon.points`, since that is all the C++ function archives.
        let polygon = square(0, 1000);

        let mut buf_fn = Vec::new();
        {
            let mut ser = serde_json::Serializer::new(&mut buf_fn);
            serialize_polygon(&mut ser, &polygon).unwrap();
        }

        let buf_points = serde_json::to_vec(&polygon.points).unwrap();
        assert_eq!(buf_fn, buf_points);

        // The derived struct impl is a different thing (it names the field); it round-trips.
        let derived = serde_json::to_string(&polygon).unwrap();
        let back: Polygon = serde_json::from_str(&derived).unwrap();
        assert_eq!(polygon, back);
    }

    #[test]
    fn serialize_ex_polygon_orders_contour_then_holes() {
        let contour = square(0, 2000);
        let hole = square(500, 1000);
        let expoly = ExPolygon::with_holes(contour.clone(), vec![hole.clone()]);

        let mut buf_fn = Vec::new();
        {
            let mut ser = serde_json::Serializer::new(&mut buf_fn);
            serialize_ex_polygon(&mut ser, &expoly).unwrap();
        }

        // Cereal `archive(expoly.contour, expoly.holes)`: the contour's points, then the holes,
        // each a list of points, with no member names.
        let expected = serde_json::to_vec(&(&contour.points, vec![&hole.points])).unwrap();
        assert_eq!(buf_fn, expected);
    }

    #[test]
    fn ex_polygon_derive_roundtrip() {
        // The derived Serialize/Deserialize is the actual Cereal-equivalent path used by
        // the rest of the crate; verify it round-trips for a polygon with no holes and
        // one with multiple holes.
        let no_holes = ExPolygon::new(square(0, 1000));
        let json = serde_json::to_string(&no_holes).unwrap();
        let back: ExPolygon = serde_json::from_str(&json).unwrap();
        assert_eq!(no_holes, back);

        let multi = ExPolygon::with_holes(square(0, 5000), vec![square(500, 1000), square(3000, 1000)]);
        let json = serde_json::to_string(&multi).unwrap();
        let back: ExPolygon = serde_json::from_str(&json).unwrap();
        assert_eq!(multi, back);
    }

    #[test]
    fn empty_polygon_roundtrip() {
        let polygon = Polygon::new();
        let json = serde_json::to_string(&polygon).unwrap();
        let back: Polygon = serde_json::from_str(&json).unwrap();
        assert_eq!(polygon, back);
        assert!(back.points.is_empty());
    }
}
