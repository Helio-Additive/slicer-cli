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
//! In Rust the equivalent of an external Cereal `serialize` is the `serde::Serialize`
//! / `serde::Deserialize` implementation. `Polygon`, `ExPolygon` and `Point` all derive
//! these traits in `crate::geometry`, with their fields declared in the same order the
//! C++ archives them (`Polygon::points`; `ExPolygon::contour` then `ExPolygon::holes`),
//! so the derived serialization mirrors the C++ member order byte-for-byte for a given
//! data format.
//!
//! The two free functions below mirror the two C++ template functions 1:1 — each takes a
//! generic archive (a `serde::Serializer`) and archives exactly the same members in the
//! same order as the C++ code.

use crate::geometry::{ExPolygon, Polygon};
use serde::ser::{SerializeTuple, Serializer};
use serde::Serialize;

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
    // archive(polygon.points);
    polygon.points.serialize(archive)
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
    // archive(expoly.contour, expoly.holes);
    // Cereal archives the listed members as a flat sequence; mirror that ordering by
    // serializing contour then holes as a 2-element tuple.
    let mut tup = archive.serialize_tuple(2)?;
    tup.serialize_element(&expoly.contour)?;
    tup.serialize_element(&expoly.holes)?;
    tup.end()
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

        // And the whole-struct derive serializes the single `points` field, so the
        // derived form round-trips through the dedicated function's data.
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

        // contour then holes, as a flat sequence (mirroring archive(contour, holes)).
        let expected = serde_json::to_vec(&(&contour, &vec![hole])).unwrap();
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
