//! Bounding box types for 2D and 3D geometry.
//!
//! This module provides axis-aligned bounding box (AABB) types,
//! mirroring BambuStudio's BoundingBox classes.

use super::{Point, Point3, Point3F, PointF};
use crate::{unscale, Coord, CoordF};
use serde::{Deserialize, Serialize};
use std::fmt;

/// A 2D axis-aligned bounding box with scaled integer coordinates.
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Point,
    pub max: Point,
    defined: bool,
}

impl BoundingBox {
    // Create a new empty (undefined) bounding box.
    #[inline]
    pub fn new() -> Self {
        Self {
            min: Point::new(Coord::MAX, Coord::MAX),
            max: Point::new(Coord::MIN, Coord::MIN),
            defined: false,
        }
    }

    /// Create a bounding box from min and max points.
    #[inline]
    pub fn from_points_minmax(min: Point, max: Point) -> Self {
        Self {
            min,
            max,
            defined: true,
        }
    }

    /// Create a bounding box from a slice of points.
    pub fn from_points(points: &[Point]) -> Self {
        let mut bb = Self::new();
        for p in points {
            bb.merge_point(*p);
        }
        bb
    }

    /// Construct the bounding box from a range of points, mirroring C++
    /// `BoundingBox::construct<IncludeBoundary>(out, from, to)`.
    /// BoundingBox.hpp:88-102
    ///
    /// if `include_boundary`, then a bounding box is defined even for a single point.
    /// otherwise a bounding box is only defined if it has a positive area.
    /// The output bounding box is expected to be set to "undefined" initially.
    pub fn construct(out: &mut Self, points: &[Point], include_boundary: bool) {
        // BoundingBox.hpp:91
        if !points.is_empty() {
            // BoundingBox.hpp:92-94
            out.min = points[0];
            out.max = out.min;
            // BoundingBox.hpp:95-99
            for vec in &points[1..] {
                out.min.x = out.min.x.min(vec.x);
                out.min.y = out.min.y.min(vec.y);
                out.max.x = out.max.x.max(vec.x);
                out.max.y = out.max.y.max(vec.y);
            }
            // BoundingBox.hpp:100
            out.defined =
                include_boundary || (out.min.x < out.max.x && out.min.y < out.max.y);
        }
    }

    /// Create a bounding box from floating-point coordinates (in mm).
    #[inline]
    pub fn from_coords_scale(min_x: CoordF, min_y: CoordF, max_x: CoordF, max_y: CoordF) -> Self {
        Self {
            min: Point::new_scale(min_x, min_y),
            max: Point::new_scale(max_x, max_y),
            defined: true,
        }
    }

    /// Check if the bounding box is defined (has been merged with at least one point).
    #[inline]
    pub fn is_defined(&self) -> bool {
        self.defined
    }

    /// Check if the bounding box is empty (not defined).
    #[inline]
    pub fn is_empty(&self) -> bool {
        !self.defined
    }

    /// Reset the bounding box to undefined state.
    pub fn reset(&mut self) {
        self.min = Point::new(Coord::MAX, Coord::MAX);
        self.max = Point::new(Coord::MIN, Coord::MIN);
        self.defined = false;
    }

    /// Merge a point into the bounding box.
    pub fn merge_point(&mut self, p: Point) {
        if self.defined {
            self.min.x = self.min.x.min(p.x);
            self.min.y = self.min.y.min(p.y);
            self.max.x = self.max.x.max(p.x);
            self.max.y = self.max.y.max(p.y);
        } else {
            self.min = p;
            self.max = p;
            self.defined = true;
        }
    }

    /// Merge another bounding box into this one.
    pub fn merge(&mut self, other: &BoundingBox) {
        if other.defined {
            self.merge_point(other.min);
            self.merge_point(other.max);
        }
    }

    /// Get the width of the bounding box.
    #[inline]
    pub fn width(&self) -> Coord {
        if self.defined {
            self.max.x - self.min.x
        } else {
            0
        }
    }

    /// Get the height of the bounding box.
    #[inline]
    pub fn height(&self) -> Coord {
        if self.defined {
            self.max.y - self.min.y
        } else {
            0
        }
    }

    /// Get the size as a point (width, height).
    #[inline]
    pub fn size(&self) -> Point {
        Point::new(self.width(), self.height())
    }

    /// Get the center point of the bounding box.
    #[inline]
    pub fn center(&self) -> Point {
        Point::new((self.min.x + self.max.x) / 2, (self.min.y + self.max.y) / 2)
    }

    /// Build the axis-aligned rectangle polygon (CCW) for this bounding box.
    ///
    /// BoundingBox.cpp:15-28 — `void BoundingBox::polygon(Polygon* polygon) const`
    /// and BoundingBox.cpp:29-34 — `Polygon BoundingBox::polygon() const`.
    pub fn polygon(&self) -> super::Polygon {
        // BoundingBox.cpp:18-19 — points.clear(); points.resize(4);
        super::Polygon::from_points(vec![
            // BoundingBox.cpp:20-21 — points[0] = (min.x, min.y)
            Point::new(self.min.x, self.min.y),
            // BoundingBox.cpp:22-23 — points[1] = (max.x, min.y)
            Point::new(self.max.x, self.min.y),
            // BoundingBox.cpp:24-25 — points[2] = (max.x, max.y)
            Point::new(self.max.x, self.max.y),
            // BoundingBox.cpp:26-27 — points[3] = (min.x, max.y)
            Point::new(self.min.x, self.max.y),
        ])
    }

    /// Get the area of the bounding box.
    #[inline]
    pub fn area(&self) -> i128 {
        self.width() as i128 * self.height() as i128
    }

    /// Get the perimeter of the bounding box.
    #[inline]
    pub fn perimeter(&self) -> Coord {
        2 * (self.width() + self.height())
    }

    /// Check if a point is inside the bounding box.
    #[inline]
    pub fn contains_point(&self, p: &Point) -> bool {
        self.defined
            && p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
    }

    /// Check if a point is strictly inside the bounding box (not on boundary).
    #[inline]
    pub fn contains_point_strict(&self, p: &Point) -> bool {
        self.defined && p.x > self.min.x && p.x < self.max.x && p.y > self.min.y && p.y < self.max.y
    }

    /// Check if this bounding box contains another bounding box.
    #[inline]
    pub fn contains(&self, other: &BoundingBox) -> bool {
        self.defined
            && other.defined
            && self.contains_point(&other.min)
            && self.contains_point(&other.max)
    }

    /// Check if this bounding box intersects another bounding box.
    #[inline]
    pub fn intersects(&self, other: &BoundingBox) -> bool {
        self.defined
            && other.defined
            && self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    /// Get the intersection of two bounding boxes.
    pub fn intersection(&self, other: &BoundingBox) -> Option<BoundingBox> {
        if !self.intersects(other) {
            return None;
        }

        Some(BoundingBox::from_points_minmax(
            Point::new(self.min.x.max(other.min.x), self.min.y.max(other.min.y)),
            Point::new(self.max.x.min(other.max.x), self.max.y.min(other.max.y)),
        ))
    }

    /// Expand the bounding box by a margin on all sides.
    pub fn expand(&mut self, margin: Coord) {
        if self.defined {
            self.min.x -= margin;
            self.min.y -= margin;
            self.max.x += margin;
            self.max.y += margin;
        }
    }

    /// Return an expanded copy of the bounding box.
    pub fn expanded(&self, margin: Coord) -> Self {
        let mut result = *self;
        result.expand(margin);
        result
    }

    /// Shrink the bounding box by a margin on all sides.
    pub fn shrink(&mut self, margin: Coord) {
        self.expand(-margin);
    }

    /// Return a shrunk copy of the bounding box.
    pub fn shrunk(&self, margin: Coord) -> Self {
        self.expanded(-margin)
    }

    /// Translate the bounding box by a vector.
    pub fn translate(&mut self, v: Point) {
        if self.defined {
            self.min = self.min + v;
            self.max = self.max + v;
        }
    }

    /// Return a translated copy of the bounding box.
    pub fn translated(&self, v: Point) -> Self {
        let mut result = *self;
        result.translate(v);
        result
    }

    /// Scale the bounding box about the origin.
    pub fn scale(&mut self, factor: CoordF) {
        if self.defined {
            self.min = self.min * factor;
            self.max = self.max * factor;
        }
    }

    /// Return a scaled copy of the bounding box.
    pub fn scaled(&self, factor: CoordF) -> Self {
        let mut result = *self;
        result.scale(factor);
        result
    }

    /// Get the corners of the bounding box.
    pub fn corners(&self) -> [Point; 4] {
        [
            self.min,
            Point::new(self.max.x, self.min.y),
            self.max,
            Point::new(self.min.x, self.max.y),
        ]
    }

    /// Convert to a floating-point bounding box.
    #[inline]
    pub fn to_f64(&self) -> BoundingBoxF {
        BoundingBoxF {
            min: self.min.to_f64(),
            max: self.max.to_f64(),
            defined: self.defined,
        }
    }

    /// Clamp a point to be within the bounding box.
    /// Alias for `new()` — creates an empty (undefined) bounding box.
    /// Matches libslic3r naming convention.
    #[inline]
    pub fn empty() -> Self {
        Self::new()
    }

    /// Alias for `width()` — returns the X extent. Matches BoundingBox3 naming.
    #[inline]
    pub fn size_x(&self) -> Coord {
        self.width()
    }

    /// Alias for `height()` — returns the Y extent. Matches BoundingBox3 naming.
    #[inline]
    pub fn size_y(&self) -> Coord {
        self.height()
    }

    /// Alias for `expand()` — grow the bounding box by a margin on all sides.
    /// Matches libslic3r naming convention.
    #[inline]
    pub fn grow(&mut self, margin: Coord) {
        self.expand(margin);
    }

    pub fn clamp_point(&self, p: &Point) -> Point {
        Point::new(
            p.x.clamp(self.min.x, self.max.x),
            p.y.clamp(self.min.y, self.max.y),
        )
    }
}

impl fmt::Debug for BoundingBox {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.defined {
            write!(f, "BoundingBox({:?} - {:?})", self.min, self.max)
        } else {
            write!(f, "BoundingBox(undefined)")
        }
    }
}

impl fmt::Display for BoundingBox {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.defined {
            write!(
                f,
                "[({:.6}, {:.6}) - ({:.6}, {:.6})]",
                unscale(self.min.x),
                unscale(self.min.y),
                unscale(self.max.x),
                unscale(self.max.y)
            )
        } else {
            write!(f, "[undefined]")
        }
    }
}

/// A 2D axis-aligned bounding box with floating-point coordinates (in mm).
#[derive(Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct BoundingBoxF {
    pub min: PointF,
    pub max: PointF,
    defined: bool,
}

impl BoundingBoxF {
    // Create a new empty bounding box.
    #[inline]
    pub fn new() -> Self {
        Self {
            min: PointF::new(CoordF::MAX, CoordF::MAX),
            max: PointF::new(CoordF::MIN, CoordF::MIN),
            defined: false,
        }
    }

    /// Create a bounding box from min and max points.
    #[inline]
    pub fn from_points_minmax(min: PointF, max: PointF) -> Self {
        Self {
            min,
            max,
            defined: true,
        }
    }

    /// Create a bounding box from coordinates.
    #[inline]
    pub fn from_coords(min_x: CoordF, min_y: CoordF, max_x: CoordF, max_y: CoordF) -> Self {
        Self {
            min: PointF::new(min_x, min_y),
            max: PointF::new(max_x, max_y),
            defined: true,
        }
    }

    /// Check if the bounding box is defined.
    #[inline]
    pub fn is_defined(&self) -> bool {
        self.defined
    }

    /// Check if the bounding box is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        !self.defined
    }

    /// Merge a point into the bounding box.
    pub fn merge_point(&mut self, p: PointF) {
        if self.defined {
            self.min.x = self.min.x.min(p.x);
            self.min.y = self.min.y.min(p.y);
            self.max.x = self.max.x.max(p.x);
            self.max.y = self.max.y.max(p.y);
        } else {
            self.min = p;
            self.max = p;
            self.defined = true;
        }
    }

    /// Merge another bounding box into this one.
    pub fn merge(&mut self, other: &BoundingBoxF) {
        if other.defined {
            self.merge_point(other.min);
            self.merge_point(other.max);
        }
    }

    /// Get the width.
    #[inline]
    pub fn width(&self) -> CoordF {
        if self.defined {
            self.max.x - self.min.x
        } else {
            0.0
        }
    }

    /// Get the height.
    #[inline]
    pub fn height(&self) -> CoordF {
        if self.defined {
            self.max.y - self.min.y
        } else {
            0.0
        }
    }

    /// Get the size.
    #[inline]
    pub fn size(&self) -> PointF {
        PointF::new(self.width(), self.height())
    }

    /// Get the center.
    #[inline]
    pub fn center(&self) -> PointF {
        PointF::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
        )
    }

    /// Get the area.
    #[inline]
    pub fn area(&self) -> CoordF {
        self.width() * self.height()
    }

    /// Check if a point is inside.
    #[inline]
    pub fn contains_point(&self, p: &PointF) -> bool {
        self.defined
            && p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
    }

    /// Check if this bounding box intersects another.
    #[inline]
    pub fn intersects(&self, other: &BoundingBoxF) -> bool {
        self.defined
            && other.defined
            && self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    /// Expand by a margin.
    pub fn expand(&mut self, margin: CoordF) {
        if self.defined {
            self.min.x -= margin;
            self.min.y -= margin;
            self.max.x += margin;
            self.max.y += margin;
        }
    }

    /// Convert to scaled integer bounding box.
    #[inline]
    pub fn to_scaled(&self) -> BoundingBox {
        BoundingBox {
            min: self.min.to_scaled(),
            max: self.max.to_scaled(),
            defined: self.defined,
        }
    }
}

impl fmt::Debug for BoundingBoxF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.defined {
            write!(f, "BoundingBoxF({:?} - {:?})", self.min, self.max)
        } else {
            write!(f, "BoundingBoxF(undefined)")
        }
    }
}

impl fmt::Display for BoundingBoxF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.defined {
            write!(
                f,
                "[({:.6}, {:.6}) - ({:.6}, {:.6})]",
                self.min.x, self.min.y, self.max.x, self.max.y
            )
        } else {
            write!(f, "[undefined]")
        }
    }
}

impl From<BoundingBox> for BoundingBoxF {
    fn from(bb: BoundingBox) -> Self {
        bb.to_f64()
    }
}

/// A 3D axis-aligned bounding box with scaled integer coordinates.
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
    defined: bool,
}

impl BoundingBox3 {
    // Create a new empty bounding box.
    #[inline]
    pub fn new() -> Self {
        Self {
            min: Point3::new(Coord::MAX, Coord::MAX, Coord::MAX),
            max: Point3::new(Coord::MIN, Coord::MIN, Coord::MIN),
            defined: false,
        }
    }

    /// Create a bounding box from min and max points.
    #[inline]
    pub fn from_points_minmax(min: Point3, max: Point3) -> Self {
        Self {
            min,
            max,
            defined: true,
        }
    }

    /// Create a bounding box from a slice of points.
    pub fn from_points(points: &[Point3]) -> Self {
        let mut bb = Self::new();
        for p in points {
            bb.merge_point(*p);
        }
        bb
    }

    /// Check if the bounding box is defined.
    #[inline]
    pub fn is_defined(&self) -> bool {
        self.defined
    }

    /// Check if the bounding box is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        !self.defined
    }

    /// Reset the bounding box.
    pub fn reset(&mut self) {
        self.min = Point3::new(Coord::MAX, Coord::MAX, Coord::MAX);
        self.max = Point3::new(Coord::MIN, Coord::MIN, Coord::MIN);
        self.defined = false;
    }

    /// Merge a point into the bounding box.
    pub fn merge_point(&mut self, p: Point3) {
        if self.defined {
            self.min.x = self.min.x.min(p.x);
            self.min.y = self.min.y.min(p.y);
            self.min.z = self.min.z.min(p.z);
            self.max.x = self.max.x.max(p.x);
            self.max.y = self.max.y.max(p.y);
            self.max.z = self.max.z.max(p.z);
        } else {
            self.min = p;
            self.max = p;
            self.defined = true;
        }
    }

    /// Merge another bounding box.
    pub fn merge(&mut self, other: &BoundingBox3) {
        if other.defined {
            self.merge_point(other.min);
            self.merge_point(other.max);
        }
    }

    /// Get the size in x direction.
    #[inline]
    pub fn size_x(&self) -> Coord {
        if self.defined {
            self.max.x - self.min.x
        } else {
            0
        }
    }

    /// Get the size in y direction.
    #[inline]
    pub fn size_y(&self) -> Coord {
        if self.defined {
            self.max.y - self.min.y
        } else {
            0
        }
    }

    /// Get the size in z direction.
    #[inline]
    pub fn size_z(&self) -> Coord {
        if self.defined {
            self.max.z - self.min.z
        } else {
            0
        }
    }

    /// Get the size as a 3D point.
    #[inline]
    pub fn size(&self) -> Point3 {
        Point3::new(self.size_x(), self.size_y(), self.size_z())
    }

    /// Get the center point.
    #[inline]
    pub fn center(&self) -> Point3 {
        Point3::new(
            (self.min.x + self.max.x) / 2,
            (self.min.y + self.max.y) / 2,
            (self.min.z + self.max.z) / 2,
        )
    }

    /// Get the volume.
    #[inline]
    pub fn volume(&self) -> i128 {
        self.size_x() as i128 * self.size_y() as i128 * self.size_z() as i128
    }

    /// Check if a point is inside.
    #[inline]
    pub fn contains_point(&self, p: &Point3) -> bool {
        self.defined
            && p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    /// Check if this bounding box intersects another.
    #[inline]
    pub fn intersects(&self, other: &BoundingBox3) -> bool {
        self.defined
            && other.defined
            && self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Expand by a margin.
    pub fn expand(&mut self, margin: Coord) {
        if self.defined {
            self.min.x -= margin;
            self.min.y -= margin;
            self.min.z -= margin;
            self.max.x += margin;
            self.max.y += margin;
            self.max.z += margin;
        }
    }

    /// Translate by a vector.
    pub fn translate(&mut self, v: Point3) {
        if self.defined {
            self.min = self.min + v;
            self.max = self.max + v;
        }
    }

    /// Convert to floating-point bounding box.
    #[inline]
    pub fn to_f64(&self) -> BoundingBox3F {
        BoundingBox3F {
            min: self.min.to_f64(),
            max: self.max.to_f64(),
            defined: self.defined,
        }
    }

    /// Project to 2D (drop z coordinate).
    #[inline]
    pub fn to_2d(&self) -> BoundingBox {
        BoundingBox {
            min: self.min.to_2d(),
            max: self.max.to_2d(),
            defined: self.defined,
        }
    }
}

impl fmt::Debug for BoundingBox3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.defined {
            write!(f, "BoundingBox3({:?} - {:?})", self.min, self.max)
        } else {
            write!(f, "BoundingBox3(undefined)")
        }
    }
}

impl fmt::Display for BoundingBox3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.defined {
            write!(
                f,
                "[({:.6}, {:.6}, {:.6}) - ({:.6}, {:.6}, {:.6})]",
                unscale(self.min.x),
                unscale(self.min.y),
                unscale(self.min.z),
                unscale(self.max.x),
                unscale(self.max.y),
                unscale(self.max.z)
            )
        } else {
            write!(f, "[undefined]")
        }
    }
}

/// A 3D axis-aligned bounding box with floating-point coordinates (in mm).
#[derive(Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox3F {
    pub min: Point3F,
    pub max: Point3F,
    defined: bool,
}

impl BoundingBox3F {
    // Create a new empty bounding box.
    #[inline]
    pub fn new() -> Self {
        Self {
            min: Point3F::new(CoordF::MAX, CoordF::MAX, CoordF::MAX),
            max: Point3F::new(CoordF::MIN, CoordF::MIN, CoordF::MIN),
            defined: false,
        }
    }

    /// Alias for `new()` — creates an empty (undefined) bounding box.
    #[inline]
    pub fn empty() -> Self {
        Self::new()
    }

    /// Alias for `new()` — creates an empty (undefined) bounding box.
    /// BoundingBox.hpp: default constructor creates empty box
    /// C++: BoundingBox() { reset(); }
    #[inline]
    pub fn new_empty() -> Self {
        Self::new()
    }

    /// Create a bounding box from min and max points.
    #[inline]
    pub fn from_points_minmax(min: Point3F, max: Point3F) -> Self {
        Self {
            min,
            max,
            defined: true,
        }
    }

    /// Check if the bounding box is defined.
    #[inline]
    pub fn is_defined(&self) -> bool {
        self.defined
    }

    /// Check if the bounding box is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        !self.defined
    }

    /// Merge a point into the bounding box.
    pub fn merge_point(&mut self, p: Point3F) {
        if self.defined {
            self.min.x = self.min.x.min(p.x);
            self.min.y = self.min.y.min(p.y);
            self.min.z = self.min.z.min(p.z);
            self.max.x = self.max.x.max(p.x);
            self.max.y = self.max.y.max(p.y);
            self.max.z = self.max.z.max(p.z);
        } else {
            self.min = p;
            self.max = p;
            self.defined = true;
        }
    }

    /// Extend the bounding box to include a point (alias for merge_point)
    /// BoundingBox.hpp: extend method
    /// C++: void extend(const Vec<N, T>& point)
    #[inline]
    pub fn extend_point(&mut self, p: &Point3F) {
        self.merge_point(*p)
    }

    /// Extend the bounding box to include another bounding box
    /// BoundingBox.hpp: extend method
    /// C++: void extend(const BoundingBox<N, T>& other)
    #[inline]
    pub fn extend(&mut self, other: &BoundingBox3F) {
        self.merge(other)
    }

    /// Merge another bounding box.
    pub fn merge(&mut self, other: &BoundingBox3F) {
        if other.defined {
            self.merge_point(other.min);
            self.merge_point(other.max);
        }
    }

    /// Get the size in x direction.
    #[inline]
    pub fn size_x(&self) -> CoordF {
        if self.defined {
            self.max.x - self.min.x
        } else {
            0.0
        }
    }

    /// Get the size in y direction.
    #[inline]
    pub fn size_y(&self) -> CoordF {
        if self.defined {
            self.max.y - self.min.y
        } else {
            0.0
        }
    }

    /// Get the size in z direction.
    #[inline]
    pub fn size_z(&self) -> CoordF {
        if self.defined {
            self.max.z - self.min.z
        } else {
            0.0
        }
    }

    /// Get the size.
    #[inline]
    pub fn size(&self) -> Point3F {
        Point3F::new(self.size_x(), self.size_y(), self.size_z())
    }

    /// Get the center.
    #[inline]
    pub fn center(&self) -> Point3F {
        Point3F::new(
            (self.min.x + self.max.x) / 2.0,
            (self.min.y + self.max.y) / 2.0,
            (self.min.z + self.max.z) / 2.0,
        )
    }

    /// Get the volume.
    #[inline]
    pub fn volume(&self) -> CoordF {
        self.size_x() * self.size_y() * self.size_z()
    }

    /// Check if a point is inside.
    #[inline]
    pub fn contains_point(&self, p: &Point3F) -> bool {
        self.defined
            && p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    /// Check if this bounding box intersects another.
    #[inline]
    pub fn intersects(&self, other: &BoundingBox3F) -> bool {
        self.defined
            && other.defined
            && self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Expand by a margin.
    pub fn expand(&mut self, margin: CoordF) {
        if self.defined {
            self.min.x -= margin;
            self.min.y -= margin;
            self.min.z -= margin;
            self.max.x += margin;
            self.max.y += margin;
            self.max.z += margin;
        }
    }

    /// Convert to scaled integer bounding box.
    #[inline]
    pub fn to_scaled(&self) -> BoundingBox3 {
        BoundingBox3 {
            min: self.min.to_scaled(),
            max: self.max.to_scaled(),
            defined: self.defined,
        }
    }

    /// Project to 2D.
    #[inline]
    pub fn to_2d(&self) -> BoundingBoxF {
        BoundingBoxF {
            min: self.min.to_2d(),
            max: self.max.to_2d(),
            defined: self.defined,
        }
    }
}

impl fmt::Debug for BoundingBox3F {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.defined {
            write!(f, "BoundingBox3F({:?} - {:?})", self.min, self.max)
        } else {
            write!(f, "BoundingBox3F(undefined)")
        }
    }
}

impl fmt::Display for BoundingBox3F {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.defined {
            write!(
                f,
                "[({:.6}, {:.6}, {:.6}) - ({:.6}, {:.6}, {:.6})]",
                self.min.x, self.min.y, self.min.z, self.max.x, self.max.y, self.max.z
            )
        } else {
            write!(f, "[undefined]")
        }
    }
}

impl From<BoundingBox3> for BoundingBox3F {
    fn from(bb: BoundingBox3) -> Self {
        bb.to_f64()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bounding_box_new() {
        let bb = BoundingBox::new();
        assert!(!bb.is_defined());
        assert!(bb.is_empty());
    }

    #[test]
    fn test_bounding_box_from_points() {
        let points = vec![Point::new(10, 20), Point::new(50, 30), Point::new(30, 100)];
        let bb = BoundingBox::from_points(&points);
        assert!(bb.is_defined());
        assert_eq!(bb.min.x, 10);
        assert_eq!(bb.min.y, 20);
        assert_eq!(bb.max.x, 50);
        assert_eq!(bb.max.y, 100);
    }

    #[test]
    fn test_bounding_box_size() {
        let bb = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 50));
        assert_eq!(bb.width(), 100);
        assert_eq!(bb.height(), 50);
        assert_eq!(bb.size(), Point::new(100, 50));
    }

    #[test]
    fn test_bounding_box_center() {
        let bb = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 100));
        let center = bb.center();
        assert_eq!(center.x, 50);
        assert_eq!(center.y, 50);
    }

    #[test]
    fn test_bounding_box_area() {
        let bb = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 50));
        assert_eq!(bb.area(), 5000);
    }

    #[test]
    fn test_bounding_box_contains_point() {
        let bb = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 100));
        assert!(bb.contains_point(&Point::new(50, 50)));
        assert!(bb.contains_point(&Point::new(0, 0)));
        assert!(bb.contains_point(&Point::new(100, 100)));
        assert!(!bb.contains_point(&Point::new(-1, 50)));
        assert!(!bb.contains_point(&Point::new(101, 50)));
    }

    #[test]
    fn test_bounding_box_intersects() {
        let bb1 = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 100));
        let bb2 = BoundingBox::from_points_minmax(Point::new(50, 50), Point::new(150, 150));
        let bb3 = BoundingBox::from_points_minmax(Point::new(200, 200), Point::new(300, 300));

        assert!(bb1.intersects(&bb2));
        assert!(bb2.intersects(&bb1));
        assert!(!bb1.intersects(&bb3));
    }

    #[test]
    fn test_bounding_box_intersection() {
        let bb1 = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 100));
        let bb2 = BoundingBox::from_points_minmax(Point::new(50, 50), Point::new(150, 150));

        let inter = bb1.intersection(&bb2).unwrap();
        assert_eq!(inter.min, Point::new(50, 50));
        assert_eq!(inter.max, Point::new(100, 100));
    }

    #[test]
    fn test_bounding_box_expand() {
        let mut bb = BoundingBox::from_points_minmax(Point::new(10, 10), Point::new(90, 90));
        bb.expand(10);
        assert_eq!(bb.min, Point::new(0, 0));
        assert_eq!(bb.max, Point::new(100, 100));
    }

    #[test]
    fn test_bounding_box_translate() {
        let mut bb = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 100));
        bb.translate(Point::new(10, 20));
        assert_eq!(bb.min, Point::new(10, 20));
        assert_eq!(bb.max, Point::new(110, 120));
    }

    #[test]
    fn test_bounding_box_merge() {
        let mut bb1 = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(50, 50));
        let bb2 = BoundingBox::from_points_minmax(Point::new(25, 25), Point::new(100, 100));
        bb1.merge(&bb2);
        assert_eq!(bb1.min, Point::new(0, 0));
        assert_eq!(bb1.max, Point::new(100, 100));
    }

    #[test]
    fn test_bounding_box_corners() {
        let bb = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 100));
        let corners = bb.corners();
        assert_eq!(corners[0], Point::new(0, 0));
        assert_eq!(corners[1], Point::new(100, 0));
        assert_eq!(corners[2], Point::new(100, 100));
        assert_eq!(corners[3], Point::new(0, 100));
    }

    #[test]
    fn test_bounding_box3_basics() {
        let bb = BoundingBox3::from_points_minmax(Point3::new(0, 0, 0), Point3::new(100, 100, 100));
        assert!(bb.is_defined());
        assert_eq!(bb.size_x(), 100);
        assert_eq!(bb.size_y(), 100);
        assert_eq!(bb.size_z(), 100);
        assert_eq!(bb.volume(), 1_000_000);
    }

    #[test]
    fn test_bounding_box3_center() {
        let bb = BoundingBox3::from_points_minmax(Point3::new(0, 0, 0), Point3::new(100, 100, 100));
        let center = bb.center();
        assert_eq!(center.x, 50);
        assert_eq!(center.y, 50);
        assert_eq!(center.z, 50);
    }

    #[test]
    fn test_bounding_box3_contains_point() {
        let bb = BoundingBox3::from_points_minmax(Point3::new(0, 0, 0), Point3::new(100, 100, 100));
        assert!(bb.contains_point(&Point3::new(50, 50, 50)));
        assert!(!bb.contains_point(&Point3::new(150, 50, 50)));
    }

    #[test]
    fn test_bounding_box3_to_2d() {
        let bb3 =
            BoundingBox3::from_points_minmax(Point3::new(10, 20, 30), Point3::new(100, 200, 300));
        let bb2 = bb3.to_2d();
        assert_eq!(bb2.min, Point::new(10, 20));
        assert_eq!(bb2.max, Point::new(100, 200));
    }

    #[test]
    fn test_bounding_box_clamp() {
        let bb = BoundingBox::from_points_minmax(Point::new(0, 0), Point::new(100, 100));

        let p1 = bb.clamp_point(&Point::new(50, 50));
        assert_eq!(p1, Point::new(50, 50));

        let p2 = bb.clamp_point(&Point::new(-20, 150));
        assert_eq!(p2, Point::new(0, 100));
    }
}
