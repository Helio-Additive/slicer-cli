//! Line segment type.
//!
//! This module provides the Line type representing a line segment between two points,
//! mirroring BambuStudio's Line class.

use super::{BoundingBox, Point, PointF};
use crate::{unscale, Coord, CoordF};
use serde::{Deserialize, Serialize};
use std::fmt;

/// A line segment defined by two endpoints.
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Line {
    pub a: Point,
    pub b: Point,
}

impl Line {
    // Create a new line segment from two points.
    #[inline]
    pub const fn new(a: Point, b: Point) -> Self {
        Self { a, b }
    }

    /// Create a line from coordinates.
    #[inline]
    pub const fn from_coords(ax: Coord, ay: Coord, bx: Coord, by: Coord) -> Self {
        Self {
            a: Point::new(ax, ay),
            b: Point::new(bx, by),
        }
    }

    /// Create a line from floating-point coordinates (in mm).
    #[inline]
    pub fn from_coords_scale(ax: CoordF, ay: CoordF, bx: CoordF, by: CoordF) -> Self {
        Self {
            a: Point::new_scale(ax, ay),
            b: Point::new_scale(bx, by),
        }
    }

    /// Direction vector (b - a). Mirrors C++ `Line::vector()` (Line.hpp:179);
    /// the C++ scalar `Line::direction()` (the angle) is `direction_angle()` below.
    /// Kept under this name because it is used as a vector crate-wide.
    #[inline]
    pub fn direction(&self) -> Point {
        self.b - self.a
    }

    /// Get the direction vector as floating-point.
    #[inline]
    pub fn direction_f(&self) -> PointF {
        self.b.to_f64() - self.a.to_f64()
    }

    /// atan2 of the direction vector. Line.hpp:176 `double atan2_() const`.
    #[inline]
    pub fn atan2_(&self) -> CoordF {
        // atan2(b(1) - a(1), b(0) - a(0))
        ((self.b.y - self.a.y) as CoordF).atan2((self.b.x - self.a.x) as CoordF)
    }

    /// Line direction angle, folded as in C++ `Line::direction()` (Line.cpp:60).
    /// Returns 0 for a (near-)horizontal line pointing in -x, otherwise the
    /// atan2 of the direction shifted into (0, PI) by adding PI when negative.
    #[inline]
    pub fn direction_angle(&self) -> CoordF {
        // Line.cpp:62
        let atan2 = self.atan2_();
        // Line.cpp:63-65
        let pi = std::f64::consts::PI;
        if (atan2 - pi).abs() < crate::libslic3r::EPSILON {
            0.0
        } else if atan2 < 0.0 {
            atan2 + pi
        } else {
            atan2
        }
    }

    /// Get the length as f64 (same as length, but explicitly named).
    #[inline]
    pub fn length_f64(&self) -> CoordF {
        self.length()
    }

    /// Get the normal vector as floating-point.
    #[inline]
    pub fn normal_f64(&self) -> PointF {
        let dir = self.direction();
        PointF::new(-(dir.y as CoordF), dir.x as CoordF)
    }

    /// Get the midpoint of the line segment.
    #[inline]
    pub fn midpoint(&self) -> Point {
        Point::new((self.a.x + self.b.x) / 2, (self.a.y + self.b.y) / 2)
    }

    /// Get the squared length of the line segment.
    #[inline]
    pub fn length_squared(&self) -> i128 {
        self.a.distance_squared(&self.b)
    }

    /// Get the length of the line segment.
    #[inline]
    pub fn length(&self) -> CoordF {
        self.a.distance(&self.b)
    }

    /// Check if this line segment is a point (zero length).
    #[inline]
    pub fn is_point(&self) -> bool {
        self.a == self.b
    }

    /// Reverse the direction of the line segment.
    #[inline]
    pub fn reverse(&self) -> Self {
        Self {
            a: self.b,
            b: self.a,
        }
    }

    /// Reverse the line segment in place.
    #[inline]
    pub fn reverse_mut(&mut self) {
        std::mem::swap(&mut self.a, &mut self.b);
    }

    /// Get the normal vector (perpendicular to the line, rotated 90° CCW).
    #[inline]
    pub fn normal(&self) -> Point {
        let dir = self.direction();
        dir.rotate_90_ccw()
    }

    /// Get the unit normal vector.
    #[inline]
    pub fn unit_normal(&self) -> PointF {
        let dir = self.direction_f();
        dir.perp().normalize()
    }

    /// Calculate the distance from a point to this line segment.
    pub fn distance_to_point(&self, p: &Point) -> CoordF {
        let proj = p.project_onto_segment(self.a, self.b);
        p.distance(&proj)
    }

    /// Calculate the squared distance from a point to this line segment.
    pub fn distance_to_point_squared(&self, p: &Point) -> i128 {
        let proj = p.project_onto_segment(self.a, self.b);
        p.distance_squared(&proj)
    }

    /// Calculate the distance from a point to the infinite line through this segment.
    pub fn distance_to_point_infinite(&self, p: &Point) -> CoordF {
        let dir = self.direction();
        let len_sq = dir.length_squared();
        if len_sq == 0 {
            return p.distance(&self.a);
        }

        // Distance = |cross(b-a, p-a)| / |b-a|
        let ap = *p - self.a;
        let cross = (dir.x as i128 * ap.y as i128 - dir.y as i128 * ap.x as i128).abs();
        cross as CoordF / (len_sq as CoordF).sqrt()
    }

    /// Project a point onto this line segment, clamping to the segment bounds.
    #[inline]
    pub fn project_point(&self, p: &Point) -> Point {
        p.project_onto_segment(self.a, self.b)
    }

    /// Project a point onto the infinite line through this segment.
    pub fn project_point_infinite(&self, p: &Point) -> Point {
        let ab = self.direction();
        let ap = *p - self.a;

        let ab_len_sq = ab.length_squared();
        if ab_len_sq == 0 {
            return self.a;
        }

        let t = ap.dot(&ab) as CoordF / ab_len_sq as CoordF;

        Point::new(
            (self.a.x as CoordF + t * ab.x as CoordF).round() as Coord,
            (self.a.y as CoordF + t * ab.y as CoordF).round() as Coord,
        )
    }

    /// Check if a point lies on this line segment (within tolerance).
    pub fn contains_point(&self, p: &Point, tolerance: Coord) -> bool {
        // Check if point is on the infinite line
        let dist = self.distance_to_point(p);
        if dist > tolerance as CoordF {
            return false;
        }

        // Check if point is within segment bounds
        let proj = self.project_point(p);
        p.coincides_with(&proj, tolerance)
    }

    /// Check if two line segments intersect.
    pub fn intersects(&self, other: &Line) -> bool {
        self.intersection(other).is_some()
    }

    /// Calculate the intersection point of two line segments.
    /// Returns None if the segments don't intersect.
    /// Line.cpp:105 `Line::intersection` -> Line.hpp:123 `line_alg::intersection`.
    pub fn intersection(&self, other: &Line) -> Option<Point> {
        // v1 = l1.b - l1.a, v2 = l2.b - l2.a
        let d1 = self.direction();
        let d2 = other.direction();

        // denom = cross2(v1, v2); C++ rejects when |denom| < EPSILON. For integer
        // coord_t inputs cross2 is exact, so |denom| < 1e-4 is exactly denom == 0.
        let cross = d1.cross(&d2);
        if cross == 0 {
            return None;
        }

        // C++ v12 = l1.a - l2.a; nume_a = cross2(v2, v12); nume_b = cross2(v1, v12);
        // diff = l2.a - l1.a = -v12, so:
        //   diff x d2 = cross2(-v12, v2) = cross2(v2, v12) = nume_a  => t == t1
        //   diff x d1 = cross2(-v12, v1) = cross2(v1, v12) = nume_b  => u == t2
        let diff = other.a - self.a;
        let t = diff.cross(&d2) as CoordF / cross as CoordF;
        let u = diff.cross(&d1) as CoordF / cross as CoordF;

        // t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1
        if t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0 {
            Some(Point::new(
                (self.a.x as CoordF + t * d1.x as CoordF).round() as Coord,
                (self.a.y as CoordF + t * d1.y as CoordF).round() as Coord,
            ))
        } else {
            None
        }
    }

    /// Static method: Calculate the squared distance from a point to a line segment.
    /// This matches libslic3r's `Line::distance_to_squared()` static method.
    pub fn distance_to_squared(p: Point, a: Point, b: Point) -> f64 {
        let line = Line::new(a, b);
        let proj = p.project_onto_segment(a, b);
        p.distance_squared(&proj) as f64
    }

    /// Static method: Calculate the squared distance from a point to an infinite line.
    /// This matches libslic3r's `Line::distance_to_infinite_squared()` static method.
    pub fn distance_to_infinite_squared(p: Point, a: Point, b: Point) -> f64 {
        let dir = b - a;
        let len_sq = dir.length_squared();
        if len_sq == 0 {
            return p.distance_squared(&a) as f64;
        }

        // Distance² = cross(b-a, p-a)² / |b-a|²
        let ap = p - a;
        let cross = dir.x as i128 * ap.y as i128 - dir.y as i128 * ap.x as i128;
        (cross * cross) as f64 / len_sq as f64
    }

    /// Static method: Calculate the distance from a point to an infinite line.
    pub fn distance_to_infinite(p: Point, a: Point, b: Point) -> f64 {
        Self::distance_to_infinite_squared(p, a, b).sqrt()
    }

    /// Calculate the intersection point of two infinite lines.
    pub fn intersection_infinite(&self, other: &Line) -> Option<Point> {
        // Line.cpp:22 — intersection of the two infinite lines via cross2 on double
        // vectors, with the C++ EPSILON near-parallel check and a coord-overflow guard.
        let a1x = self.a.x as CoordF;
        let a1y = self.a.y as CoordF;
        let v12x = (other.a.x - self.a.x) as CoordF;
        let v12y = (other.a.y - self.a.y) as CoordF;
        let v1x = (self.b.x - self.a.x) as CoordF;
        let v1y = (self.b.y - self.a.y) as CoordF;
        let v2x = (other.b.x - other.a.x) as CoordF;
        let v2y = (other.b.y - other.a.y) as CoordF;
        let denom = v1x * v2y - v1y * v2x; // cross2(v1, v2)
        if denom.abs() < crate::libslic3r::EPSILON {
            return None;
        }
        let t1 = (v12x * v2y - v12y * v2x) / denom; // cross2(v12, v2) / denom
        let rx = a1x + t1 * v1x;
        let ry = a1y + t1 * v1y;
        // Line.cpp:33-38 — if a coordinate exceeds coord_t's range the point cannot
        // be stored without integer overflow (input lines are parallel / near-parallel).
        // FIDELITY-NOTE(F2): C++ coord_t == int32_t, so the guard is against int32
        // limits, not i64; reproduce the int32 bounds locally to match C++.
        let coord_max = i32::MAX as CoordF;
        let coord_min = i32::MIN as CoordF; // std::numeric_limits<coord_t>::lowest()
        if rx > coord_max || rx < coord_min || ry > coord_max || ry < coord_min {
            return None;
        }
        Some(Point::new(rx as Coord, ry as Coord))
    }

    /// Calculate the angle of this line segment (in radians, from positive x-axis).
    #[inline]
    pub fn angle(&self) -> CoordF {
        let dir = self.direction();
        (dir.y as CoordF).atan2(dir.x as CoordF)
    }

    /// Check if this line segment is parallel to another.
    pub fn is_parallel_to(&self, other: &Line) -> bool {
        let cross = self.direction().cross(&other.direction());
        cross == 0
    }

    /// Check if this line segment is perpendicular to another.
    pub fn is_perpendicular_to(&self, other: &Line) -> bool {
        let dot = self.direction().dot(&other.direction());
        dot == 0
    }

    /// Get the CCW (counter-clockwise) value of a point relative to this line.
    /// Positive if the point is to the left of the line (a -> b direction).
    #[inline]
    pub fn ccw(&self, p: &Point) -> i128 {
        let v1 = self.b - self.a;
        let v2 = *p - self.a;
        v1.cross(&v2)
    }

    /// Translate the line by a vector.
    #[inline]
    pub fn translate(&self, v: Point) -> Self {
        Self {
            a: self.a + v,
            b: self.b + v,
        }
    }

    /// Scale the line about the origin.
    #[inline]
    pub fn scale(&self, factor: CoordF) -> Self {
        Self {
            a: self.a * factor,
            b: self.b * factor,
        }
    }

    /// Rotate the line about the origin.
    #[inline]
    pub fn rotate(&self, angle: CoordF) -> Self {
        Self {
            a: self.a.rotate(angle),
            b: self.b.rotate(angle),
        }
    }

    /// Rotate the line about a center point.
    #[inline]
    pub fn rotate_around(&self, angle: CoordF, center: Point) -> Self {
        Self {
            a: self.a.rotate_around(angle, center),
            b: self.b.rotate_around(angle, center),
        }
    }

    /// Convert to floating-point line.
    #[inline]
    pub fn to_f64(&self) -> LineF {
        LineF {
            a: self.a.to_f64(),
            b: self.b.to_f64(),
        }
    }

    /// Direction vector (b - a). Line.hpp:179 `Vector vector() const`.
    #[inline]
    pub fn vector(&self) -> Point {
        self.b - self.a
    }

    /// Perpendicular distance from `point` to the infinite line through a,b.
    /// Line.cpp:43 `Line::perp_distance_to`.
    pub fn perp_distance_to(&self, point: &Point) -> CoordF {
        let vx = (self.b.x - self.a.x) as CoordF;
        let vy = (self.b.y - self.a.y) as CoordF;
        let vax = (point.x - self.a.x) as CoordF;
        let vay = (point.y - self.a.y) as CoordF;
        if self.a == self.b {
            return (vax * vax + vay * vay).sqrt();
        }
        // std::abs(cross2(v, va)) / v.norm()
        (vx * vay - vy * vax).abs() / (vx * vx + vy * vy).sqrt()
    }

    /// Orientation angle in [0, 2*PI). Line.cpp:53 `Line::orientation`.
    pub fn orientation(&self) -> CoordF {
        // atan2_() = atan2(b.y - a.y, b.x - a.x)
        let mut angle =
            ((self.b.y - self.a.y) as CoordF).atan2((self.b.x - self.a.x) as CoordF);
        if angle < 0.0 {
            angle = 2.0 * std::f64::consts::PI + angle;
        }
        angle
    }

    /// True if this line is parallel to `line` (within EPSILON). Line.cpp:73.
    pub fn parallel_to(&self, line: &Line) -> bool {
        let v1x = (self.b.x - self.a.x) as CoordF;
        let v1y = (self.b.y - self.a.y) as CoordF;
        let v2x = (line.b.x - line.a.x) as CoordF;
        let v2y = (line.b.y - line.a.y) as CoordF;
        let cross = v1x * v2y - v1y * v2x;
        let eps = crate::libslic3r::EPSILON;
        // sqr(cross2(v1,v2)) < sqr(EPSILON) * v1.squaredNorm() * v2.squaredNorm()
        cross * cross < eps * eps * (v1x * v1x + v1y * v1y) * (v2x * v2x + v2y * v2y)
    }

    /// True if this line is perpendicular to `line` (within EPSILON). Line.cpp:98.
    pub fn perpendicular_to(&self, line: &Line) -> bool {
        let v1x = (self.b.x - self.a.x) as CoordF;
        let v1y = (self.b.y - self.a.y) as CoordF;
        let v2x = (line.b.x - line.a.x) as CoordF;
        let v2y = (line.b.y - line.a.y) as CoordF;
        let dot = v1x * v2x + v1y * v2y;
        let eps = crate::libslic3r::EPSILON;
        // sqr(v1.dot(v2)) < sqr(EPSILON) * v1.squaredNorm() * v2.squaredNorm()
        dot * dot < eps * eps * (v1x * v1x + v1y * v1y) * (v2x * v2x + v2y * v2y)
    }

    /// Overlap length with a parallel, (near-)collinear line; None if not overlapping.
    /// Line.cpp:79 `Line::overlap`.
    pub fn overlap(&self, line: &Line) -> Option<CoordF> {
        if !self.parallel_to(line) {
            return None;
        }
        let line_ = Line::new(self.a, line.a);
        if line_.length() > crate::scaled(crate::libslic3r::EPSILON) as CoordF
            && !self.parallel_to(&line_)
        {
            return None;
        }
        let a_min = self.a.x.min(self.b.x);
        let a_max = self.a.x.max(self.b.x);
        let b_min = line.a.x.min(line.b.x);
        let b_max = line.a.x.max(line.b.x);
        if a_min > b_max || a_max < b_min {
            return None;
        }
        let mut overlap_length = (a_max.min(b_max) - a_min.max(b_min)).max(0) as CoordF;
        overlap_length /= (a_max as CoordF - a_min as CoordF) / self.length();
        Some(overlap_length)
    }

    /// Extend both endpoints outward by `offset` (scaled units), along the line direction.
    /// Line.cpp:121 `Line::extend`. C++ casts the offset vector to coord_t (truncation).
    pub fn extend(&mut self, offset: CoordF) {
        let vx = (self.b.x - self.a.x) as CoordF;
        let vy = (self.b.y - self.a.y) as CoordF;
        let norm = (vx * vx + vy * vy).sqrt();
        let ox = (offset * vx / norm) as Coord;
        let oy = (offset * vy / norm) as Coord;
        self.a.x -= ox;
        self.a.y -= oy;
        self.b.x += ox;
        self.b.y += oy;
    }

    /// True if this line's direction is parallel to `angle` (radians). Line.cpp:68
    /// `Line::parallel_to(double angle)`.
    pub fn parallel_to_angle(&self, angle: CoordF) -> bool {
        super::directions_parallel(self.direction_angle(), angle, 0.0)
    }

    /// True if this line's direction is perpendicular to `angle` (radians). Line.cpp:93
    /// `Line::perpendicular_to(double angle)`.
    pub fn perpendicular_to_angle(&self, angle: CoordF) -> bool {
        super::directions_perpendicular(self.direction_angle(), angle, 0.0)
    }

    /// Clip this segment to `bbox` (Liang–Barsky); returns true if any part remains.
    /// Line.cpp:110 `Line::clip_with_bbox`.
    pub fn clip_with_bbox(&mut self, bbox: &BoundingBox) -> bool {
        match super::liang_barsky_line_clipping(
            (self.a.x as CoordF, self.a.y as CoordF),
            (self.b.x as CoordF, self.b.y as CoordF),
            (bbox.min.x as CoordF, bbox.min.y as CoordF),
            (bbox.max.x as CoordF, bbox.max.y as CoordF),
        ) {
            Some((x0, x1)) => {
                self.a = Point::new(x0.0 as Coord, x0.1 as Coord);
                self.b = Point::new(x1.0 as Coord, x1.1 as Coord);
                true
            }
            None => false,
        }
    }
}

impl fmt::Debug for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Line({:?} -> {:?})", self.a, self.b)
    }
}

impl fmt::Display for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[({:.6}, {:.6}) -> ({:.6}, {:.6})]",
            unscale(self.a.x),
            unscale(self.a.y),
            unscale(self.b.x),
            unscale(self.b.y)
        )
    }
}

impl From<(Point, Point)> for Line {
    #[inline]
    fn from((a, b): (Point, Point)) -> Self {
        Self { a, b }
    }
}

impl From<Line> for (Point, Point) {
    #[inline]
    fn from(line: Line) -> Self {
        (line.a, line.b)
    }
}

/// A line segment with floating-point coordinates (in mm).
#[derive(Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct LineF {
    pub a: PointF,
    pub b: PointF,
}

impl LineF {
    // Create a new floating-point line segment.
    #[inline]
    pub const fn new(a: PointF, b: PointF) -> Self {
        Self { a, b }
    }

    /// Get the direction vector.
    #[inline]
    pub fn direction(&self) -> PointF {
        self.b - self.a
    }

    /// Get the midpoint.
    #[inline]
    pub fn midpoint(&self) -> PointF {
        PointF::new((self.a.x + self.b.x) / 2.0, (self.a.y + self.b.y) / 2.0)
    }

    /// Get the squared length.
    #[inline]
    pub fn length_squared(&self) -> CoordF {
        self.a.distance_squared(&self.b)
    }

    /// Get the length.
    #[inline]
    pub fn length(&self) -> CoordF {
        self.a.distance(&self.b)
    }

    /// Reverse the line.
    #[inline]
    pub fn reverse(&self) -> Self {
        Self {
            a: self.b,
            b: self.a,
        }
    }

    /// Get the unit normal vector.
    #[inline]
    pub fn unit_normal(&self) -> PointF {
        self.direction().perp().normalize()
    }

    /// Calculate the angle (in radians).
    #[inline]
    pub fn angle(&self) -> CoordF {
        let dir = self.direction();
        dir.y.atan2(dir.x)
    }

    /// Convert to scaled integer line.
    #[inline]
    pub fn to_scaled(&self) -> Line {
        Line {
            a: self.a.to_scaled(),
            b: self.b.to_scaled(),
        }
    }
}

impl fmt::Debug for LineF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "LineF({:?} -> {:?})", self.a, self.b)
    }
}

impl fmt::Display for LineF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[({:.6}, {:.6}) -> ({:.6}, {:.6})]",
            self.a.x, self.a.y, self.b.x, self.b.y
        )
    }
}

impl From<Line> for LineF {
    #[inline]
    fn from(line: Line) -> Self {
        line.to_f64()
    }
}

/// Type alias for a collection of lines.
pub type Lines = Vec<Line>;

/// Bounding box enclosing all endpoints of `lines`.
/// Line.cpp:135 `BoundingBox get_extents(const Lines &lines)`.
pub fn get_extents(lines: &[Line]) -> BoundingBox {
    let mut bbox = BoundingBox::new();
    for line in lines {
        bbox.merge_point(line.a);
        bbox.merge_point(line.b);
    }
    bbox
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_new() {
        let a = Point::new(0, 0);
        let b = Point::new(100, 100);
        let line = Line::new(a, b);
        assert_eq!(line.a, a);
        assert_eq!(line.b, b);
    }

    #[test]
    fn test_line_length() {
        let line = Line::from_coords(0, 0, 3_000_000, 4_000_000);
        let len = line.length();
        assert!((len - 5_000_000.0).abs() < 1.0);
    }

    #[test]
    fn test_line_midpoint() {
        let line = Line::from_coords(0, 0, 100, 100);
        let mid = line.midpoint();
        assert_eq!(mid.x, 50);
        assert_eq!(mid.y, 50);
    }

    #[test]
    fn test_line_direction() {
        let line = Line::from_coords(10, 20, 30, 50);
        let dir = line.direction();
        assert_eq!(dir.x, 20);
        assert_eq!(dir.y, 30);
    }

    #[test]
    fn test_line_reverse() {
        let line = Line::from_coords(0, 0, 100, 100);
        let reversed = line.reverse();
        assert_eq!(reversed.a, line.b);
        assert_eq!(reversed.b, line.a);
    }

    #[test]
    fn test_line_intersection() {
        let line1 = Line::from_coords(0, 0, 100, 100);
        let line2 = Line::from_coords(0, 100, 100, 0);
        let intersection = line1.intersection(&line2);
        assert!(intersection.is_some());
        let p = intersection.unwrap();
        assert_eq!(p.x, 50);
        assert_eq!(p.y, 50);
    }

    #[test]
    fn test_line_no_intersection() {
        let line1 = Line::from_coords(0, 0, 50, 50);
        let line2 = Line::from_coords(60, 60, 100, 100);
        assert!(line1.intersection(&line2).is_none());
    }

    #[test]
    fn test_line_parallel() {
        let line1 = Line::from_coords(0, 0, 100, 100);
        let line2 = Line::from_coords(0, 10, 100, 110);
        assert!(line1.is_parallel_to(&line2));
        assert!(line1.intersection(&line2).is_none());
    }

    #[test]
    fn test_line_distance_to_point() {
        let line = Line::from_coords(0, 0, 100, 0);
        let p = Point::new(50, 50);
        let dist = line.distance_to_point(&p);
        assert!((dist - 50.0).abs() < 1.0);
    }

    #[test]
    fn test_line_angle() {
        let line = Line::from_coords(0, 0, 100, 0);
        assert!((line.angle() - 0.0).abs() < 1e-10);

        let line2 = Line::from_coords(0, 0, 0, 100);
        assert!((line2.angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
    }

    #[test]
    fn test_line_ccw() {
        let line = Line::from_coords(0, 0, 100, 0);
        let p_left = Point::new(50, 50);
        let p_right = Point::new(50, -50);

        assert!(line.ccw(&p_left) > 0); // Point is to the left
        assert!(line.ccw(&p_right) < 0); // Point is to the right
    }

    #[test]
    fn test_line_project_point() {
        let line = Line::from_coords(0, 0, 100, 0);
        let p = Point::new(50, 50);
        let proj = line.project_point(&p);
        assert_eq!(proj.x, 50);
        assert_eq!(proj.y, 0);
    }

    #[test]
    fn test_line_translate() {
        let line = Line::from_coords(0, 0, 100, 100);
        let translated = line.translate(Point::new(10, 20));
        assert_eq!(translated.a.x, 10);
        assert_eq!(translated.a.y, 20);
        assert_eq!(translated.b.x, 110);
        assert_eq!(translated.b.y, 120);
    }
}
