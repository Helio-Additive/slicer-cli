//! Boost.Geometry concept adaptation for Slic3r types.
//!
//! C++ Reference:
//! - SLA/BoostAdapter.hpp
//!
//! The C++ header specializes the `boost::geometry::traits` templates so that
//! `Slic3r::Point`, `Slic3r::Vec2d` and `Slic3r::Vec3d` model the
//! Boost.Geometry *Point* concept, and `Slic3r::BoundingBox` models the *Box*
//! concept. This is what allows those types to be used directly with
//! `boost::geometry` algorithms and `boost::geometry::index::rtree`
//! (see SLA/SpatIndex.cpp).
//!
//! Rust has no Boost.Geometry; the identical compile-time adaptation is
//! expressed as local traits with associated types/consts, implemented for
//! the equivalent crate types in the same order as the C++ specializations:
//!
//! | C++ trait specialization              | Rust item                                  |
//! |---------------------------------------|--------------------------------------------|
//! | `tag<T>::type = point_tag`            | `BoostGeometryPoint::Tag = PointTag`       |
//! | `tag<T>::type = box_tag`              | `BoostGeometryBox::Tag = BoxTag`           |
//! | `coordinate_type<T>::type`            | `…::CoordinateType`                        |
//! | `coordinate_system<T>::type`          | `…::CoordinateSystem = CsCartesian`        |
//! | `dimension<T>`                        | `…::DIMENSION`                             |
//! | `access<T, d>::get/set`               | `BoostGeometryPoint::{get, set}`           |
//! | `point_type<Box>::type`               | `BoostGeometryBox::PointType`              |
//! | `indexed_access<Box, Index, d>`       | `BoostGeometryBox::{get, set}`             |
//! | `range_value<std::vector<T>>::type`   | `RangeValue::Type` for `Vec<T>`            |

use crate::bounding_box::BoundingBox;
use crate::geometry::{Point, Vec2d, Vec3d};
use crate::{Coord, CoordF};

/// Boost.Geometry `point_tag` concept tag.
///
/// BoostAdapter.hpp:18 (`using type = point_tag;`)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PointTag;

/// Boost.Geometry `box_tag` concept tag.
///
/// BoostAdapter.hpp:98 (`using type = box_tag;`)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BoxTag;

/// Boost.Geometry `cs::cartesian` coordinate system marker.
///
/// BoostAdapter.hpp:26 (`using type = cs::cartesian;`)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CsCartesian;

/* ************************************************************************** */
/* Point concept adaptation ************************************************* */
/* ************************************************************************** */
// BoostAdapter.hpp:13-15

/// Mirror of the Boost.Geometry *Point* concept traits
/// (`tag`, `coordinate_type`, `coordinate_system`, `dimension`, `access`).
pub trait BoostGeometryPoint {
    /// `tag<T>::type` — always `point_tag` for point types.
    type Tag;
    /// `coordinate_type<T>::type`.
    type CoordinateType: Copy;
    /// `coordinate_system<T>::type` — always `cs::cartesian` here.
    type CoordinateSystem;
    /// `dimension<T>` (a `boost::mpl::int_<N>`).
    const DIMENSION: usize;

    /// `access<T, d>::get(a)`.
    fn get(&self, d: usize) -> Self::CoordinateType;

    /// `access<T, d>::set(a, value)`.
    fn set(&mut self, d: usize, value: Self::CoordinateType);
}

// template<> struct tag<Slic3r::Point> { using type = point_tag; };
// BoostAdapter.hpp:17-19
// template<> struct coordinate_type<Slic3r::Point> { using type = coord_t; };
// BoostAdapter.hpp:21-23
// template<> struct coordinate_system<Slic3r::Point> { using type = cs::cartesian; };
// BoostAdapter.hpp:25-27
// template<> struct dimension<Slic3r::Point>: boost::mpl::int_<2> {};
// BoostAdapter.hpp:29
// template<std::size_t d> struct access<Slic3r::Point, d>
// BoostAdapter.hpp:31-39
impl BoostGeometryPoint for Point {
    type Tag = PointTag; // BoostAdapter.hpp:18
    // FIDELITY-NOTE(F2): C++ `coord_t` is int32_t (libslic3r.h:40); crate-wide
    // `Coord` is i64. This is a pure compile-time concept-mapping header with no
    // arithmetic/truncation, so the wider type changes no logic here.
    type CoordinateType = Coord; // BoostAdapter.hpp:22 (coord_t)
    type CoordinateSystem = CsCartesian; // BoostAdapter.hpp:26
    const DIMENSION: usize = 2; // BoostAdapter.hpp:29

    // static inline coord_t get(Slic3r::Point const& a) { return a(d); }
    // BoostAdapter.hpp:32-34
    #[inline]
    fn get(&self, d: usize) -> Coord {
        match d {
            0 => self.x,
            1 => self.y,
            _ => panic!("Point access dimension out of range: {d}"),
        }
    }

    // static inline void set(Slic3r::Point& a, coord_t const& value) { a(d) = value; }
    // BoostAdapter.hpp:36-38
    #[inline]
    fn set(&mut self, d: usize, value: Coord) {
        match d {
            0 => self.x = value,
            1 => self.y = value,
            _ => panic!("Point access dimension out of range: {d}"),
        }
    }
}

// For Vec2d ///////////////////////////////////////////////////////////////////
// BoostAdapter.hpp:41

// template<> struct tag<Slic3r::Vec2d> { using type = point_tag; };
// BoostAdapter.hpp:43-45
// template<> struct coordinate_type<Slic3r::Vec2d> { using type = double; };
// BoostAdapter.hpp:47-49
// template<> struct coordinate_system<Slic3r::Vec2d> { using type = cs::cartesian; };
// BoostAdapter.hpp:51-53
// template<> struct dimension<Slic3r::Vec2d>: boost::mpl::int_<2> {};
// BoostAdapter.hpp:55
// template<std::size_t d> struct access<Slic3r::Vec2d, d>
// BoostAdapter.hpp:57-65
impl BoostGeometryPoint for Vec2d {
    type Tag = PointTag; // BoostAdapter.hpp:44
    type CoordinateType = CoordF; // BoostAdapter.hpp:48 (double)
    type CoordinateSystem = CsCartesian; // BoostAdapter.hpp:52
    const DIMENSION: usize = 2; // BoostAdapter.hpp:55

    // static inline double get(Slic3r::Vec2d const& a) { return a(d); }
    // BoostAdapter.hpp:58-60
    #[inline]
    fn get(&self, d: usize) -> CoordF {
        match d {
            0 => self.x,
            1 => self.y,
            _ => panic!("Vec2d access dimension out of range: {d}"),
        }
    }

    // static inline void set(Slic3r::Vec2d& a, double const& value) { a(d) = value; }
    // BoostAdapter.hpp:62-64
    #[inline]
    fn set(&mut self, d: usize, value: CoordF) {
        match d {
            0 => self.x = value,
            1 => self.y = value,
            _ => panic!("Vec2d access dimension out of range: {d}"),
        }
    }
}

// For Vec3d ///////////////////////////////////////////////////////////////////
// BoostAdapter.hpp:67

// template<> struct tag<Slic3r::Vec3d> { using type = point_tag; };
// BoostAdapter.hpp:69-71
// template<> struct coordinate_type<Slic3r::Vec3d> { using type = double; };
// BoostAdapter.hpp:73-75
// template<> struct coordinate_system<Slic3r::Vec3d> { using type = cs::cartesian; };
// BoostAdapter.hpp:77-79
// template<> struct dimension<Slic3r::Vec3d>: boost::mpl::int_<3> {};
// BoostAdapter.hpp:81
// template<std::size_t d> struct access<Slic3r::Vec3d, d>
// BoostAdapter.hpp:83-91
impl BoostGeometryPoint for Vec3d {
    type Tag = PointTag; // BoostAdapter.hpp:70
    type CoordinateType = CoordF; // BoostAdapter.hpp:74 (double)
    type CoordinateSystem = CsCartesian; // BoostAdapter.hpp:78
    const DIMENSION: usize = 3; // BoostAdapter.hpp:81

    // static inline double get(Slic3r::Vec3d const& a) { return a(d); }
    // BoostAdapter.hpp:84-86
    #[inline]
    fn get(&self, d: usize) -> CoordF {
        match d {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => panic!("Vec3d access dimension out of range: {d}"),
        }
    }

    // static inline void set(Slic3r::Vec3d& a, double const& value) { a(d) = value; }
    // BoostAdapter.hpp:88-90
    #[inline]
    fn set(&mut self, d: usize, value: CoordF) {
        match d {
            0 => self.x = value,
            1 => self.y = value,
            2 => self.z = value,
            _ => panic!("Vec3d access dimension out of range: {d}"),
        }
    }
}

/* ************************************************************************** */
/* Box concept adaptation *************************************************** */
/* ************************************************************************** */
// BoostAdapter.hpp:93-95

/// Mirror of the Boost.Geometry *Box* concept traits
/// (`tag`, `point_type`, `indexed_access`).
///
/// The corner index follows the Boost.Geometry convention:
/// `0` = `min_corner`, `1` = `max_corner`.
pub trait BoostGeometryBox {
    /// `tag<Box>::type` — always `box_tag` for box types.
    type Tag;
    /// `point_type<Box>::type`.
    type PointType;
    /// Coordinate type of `PointType`.
    type CoordinateType: Copy;

    /// `indexed_access<Box, Index, d>::get(box)`.
    fn get(&self, index: usize, d: usize) -> Self::CoordinateType;

    /// `indexed_access<Box, Index, d>::set(box, coord)`.
    fn set(&mut self, index: usize, d: usize, coord: Self::CoordinateType);
}

// template<> struct tag<Slic3r::BoundingBox> { using type = box_tag; };
// BoostAdapter.hpp:97-99
// template<> struct point_type<Slic3r::BoundingBox> { using type = Slic3r::Point; };
// BoostAdapter.hpp:101-103
// template<std::size_t d> struct indexed_access<Slic3r::BoundingBox, 0, d>
// BoostAdapter.hpp:105-113
// template<std::size_t d> struct indexed_access<Slic3r::BoundingBox, 1, d>
// BoostAdapter.hpp:115-123
impl BoostGeometryBox for BoundingBox {
    type Tag = BoxTag; // BoostAdapter.hpp:98
    type PointType = Point; // BoostAdapter.hpp:102
    // FIDELITY-NOTE(F2): C++ `coord_t` is int32_t (libslic3r.h:40); crate-wide
    // `Coord` is i64. Pure concept-mapping header, no arithmetic — no logic change.
    type CoordinateType = Coord; // coord_t

    #[inline]
    fn get(&self, index: usize, d: usize) -> Coord {
        match index {
            // static inline coord_t get(Slic3r::BoundingBox const& box) { return box.min(d); }
            // BoostAdapter.hpp:107-109
            0 => BoostGeometryPoint::get(&self.min, d),
            // static inline coord_t get(Slic3r::BoundingBox const& box) { return box.max(d); }
            // BoostAdapter.hpp:117-119
            1 => BoostGeometryPoint::get(&self.max, d),
            _ => panic!("BoundingBox indexed_access corner index out of range: {index}"),
        }
    }

    #[inline]
    fn set(&mut self, index: usize, d: usize, coord: Coord) {
        match index {
            // static inline void set(Slic3r::BoundingBox &box, coord_t const& coord) { box.min(d) = coord; }
            // BoostAdapter.hpp:110-112
            0 => BoostGeometryPoint::set(&mut self.min, d, coord),
            // static inline void set(Slic3r::BoundingBox &box, coord_t const& coord) { box.max(d) = coord; }
            // BoostAdapter.hpp:120-122
            1 => BoostGeometryPoint::set(&mut self.max, d, coord),
            _ => panic!("BoundingBox indexed_access corner index out of range: {index}"),
        }
    }
}

/// Mirror of `boost::range_value<Range>::type`.
pub trait RangeValue {
    /// `range_value<Range>::type`.
    type Type;
}

// template<> struct range_value<std::vector<Slic3r::Vec2d>> { using type = Slic3r::Vec2d; };
// BoostAdapter.hpp:128-130
impl RangeValue for Vec<Vec2d> {
    type Type = Vec2d; // BoostAdapter.hpp:129
}

#[cfg(test)]
mod tests {
    use super::*;

    // BoostAdapter.hpp:31-39 — access<Slic3r::Point, d>
    #[test]
    fn test_point_access() {
        let mut p = Point::new(3, -7);
        assert_eq!(BoostGeometryPoint::get(&p, 0), 3);
        assert_eq!(BoostGeometryPoint::get(&p, 1), -7);
        BoostGeometryPoint::set(&mut p, 0, 11);
        BoostGeometryPoint::set(&mut p, 1, 13);
        assert_eq!(p.x, 11);
        assert_eq!(p.y, 13);
        assert_eq!(<Point as BoostGeometryPoint>::DIMENSION, 2);
    }

    // BoostAdapter.hpp:57-65 — access<Slic3r::Vec2d, d>
    #[test]
    fn test_vec2d_access() {
        let mut v = Vec2d::new(1.5, -2.5);
        assert_eq!(BoostGeometryPoint::get(&v, 0), 1.5);
        assert_eq!(BoostGeometryPoint::get(&v, 1), -2.5);
        BoostGeometryPoint::set(&mut v, 0, 4.25);
        BoostGeometryPoint::set(&mut v, 1, -8.5);
        assert_eq!(v.x, 4.25);
        assert_eq!(v.y, -8.5);
        assert_eq!(<Vec2d as BoostGeometryPoint>::DIMENSION, 2);
    }

    // BoostAdapter.hpp:83-91 — access<Slic3r::Vec3d, d>
    #[test]
    fn test_vec3d_access() {
        let mut v = Vec3d::new(1.0, 2.0, 3.0);
        assert_eq!(BoostGeometryPoint::get(&v, 0), 1.0);
        assert_eq!(BoostGeometryPoint::get(&v, 1), 2.0);
        assert_eq!(BoostGeometryPoint::get(&v, 2), 3.0);
        BoostGeometryPoint::set(&mut v, 2, 9.0);
        assert_eq!(v.z, 9.0);
        assert_eq!(<Vec3d as BoostGeometryPoint>::DIMENSION, 3);
    }

    // BoostAdapter.hpp:105-123 — indexed_access<Slic3r::BoundingBox, {0,1}, d>
    #[test]
    fn test_bounding_box_indexed_access() {
        let mut bb =
            BoundingBox::new_from_points(Point::new(1, 2), Point::new(10, 20));
        assert_eq!(BoostGeometryBox::get(&bb, 0, 0), 1);
        assert_eq!(BoostGeometryBox::get(&bb, 0, 1), 2);
        assert_eq!(BoostGeometryBox::get(&bb, 1, 0), 10);
        assert_eq!(BoostGeometryBox::get(&bb, 1, 1), 20);
        BoostGeometryBox::set(&mut bb, 0, 0, -5);
        BoostGeometryBox::set(&mut bb, 1, 1, 50);
        assert_eq!(bb.min.x, -5);
        assert_eq!(bb.max.y, 50);
    }
}
