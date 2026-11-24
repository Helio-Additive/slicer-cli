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

/// An int64 `coord_t` subtraction as the pin's compiled code does it: two's complement,
/// wrapping on overflow (Line.hpp:127-129, 137 compute `l1.b - l1.a`, `l2.b - l2.a`,
/// `l1.a - l2.a` on Vec2crd before the cast to double; clang arm64 emits a plain `sub`).
/// Only an overflowing subtraction looks at the gate, so the common case costs nothing.
/// LINE_INTERSECTION_WRAP_PIN=0 restores the earlier arm (a plain `-`: a panic in a debug
/// build, the same wrap in a release build).
#[inline]
fn pin_sub_i64(a: Coord, b: Coord) -> Coord {
    match a.checked_sub(b) {
        Some(v) => v,
        None if crate::faithful_gate("LINE_INTERSECTION_WRAP_PIN") => a.wrapping_sub(b),
        None => a - b,
    }
}

/// LINE_INTERSECTION_FMA_PIN (faithful): `Line::intersection`'s cross products fused as the
/// pin's `cross2`. Read once: the gates come from a snapshot of the environment (lib.rs).
#[inline]
fn line_intersection_fma_pin() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| crate::faithful_gate("LINE_INTERSECTION_FMA_PIN"))
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
        // Line.hpp:180, `Vector((b(1) - a(1)), -(b(0) - a(0)))`: the vector
        // turned clockwise.
        Point::new(self.b.y - self.a.y, -(self.b.x - self.a.x))
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
        // `line_alg::intersection` (Line.hpp:123-148) in double; the point is
        // cast to coord_t, which truncates.
        // Line.hpp:127-129. `l1.b - l1.a` is a Vec2crd (int64) subtraction, cast to double
        // afterwards; it wraps at the pin where it overflows (a saturated Voronoi vertex in
        // MultiMaterialSegmentation.cpp's build_graph), see `pin_sub_i64`.
        let v1x = pin_sub_i64(self.b.x, self.a.x) as CoordF;
        let v1y = pin_sub_i64(self.b.y, self.a.y) as CoordF;
        let v2x = pin_sub_i64(other.b.x, other.a.x) as CoordF;
        let v2y = pin_sub_i64(other.b.y, other.a.y) as CoordF;
        // `cross2(v1, v2)` (Point.hpp:86-89): one expression, so clang fuses the left product,
        // `fma(x1, y2, -(y1 * x2))` (as `Cross2 for PointF`, geometry/point.rs). Equal to the
        // unfused form whenever the products are exact (|coordinates| < 2^26), so only far
        // points (a saturated Voronoi vertex in build_graph) see it. Gate LINE_INTERSECTION_FMA_PIN.
        let fused = line_intersection_fma_pin();
        let cross = |ax: CoordF, ay: CoordF, bx: CoordF, by: CoordF| -> CoordF {
            if fused {
                ax.mul_add(by, -(ay * bx))
            } else {
                ax * by - ay * bx
            }
        };
        let denom = cross(v1x, v1y, v2x, v2y);
        // Line.hpp:130-136
        if denom.abs() < crate::libslic3r::EPSILON {
            return None;
        }
        // Line.hpp:137-141 (`l1.a - l2.a`, int64 as above)
        let v12x = pin_sub_i64(self.a.x, other.a.x) as CoordF;
        let v12y = pin_sub_i64(self.a.y, other.a.y) as CoordF;
        let nume_a = cross(v2x, v2y, v12x, v12y);
        let nume_b = cross(v1x, v1y, v12x, v12y);
        let t1 = nume_a / denom;
        let t2 = nume_b / denom;
        // Line.hpp:142-147
        if t1 >= 0.0 && t1 <= 1.0 && t2 >= 0.0 && t2 <= 1.0 {
            return Some(Point::new(
                (self.a.x as CoordF + t1 * v1x) as Coord,
                (self.a.y as CoordF + t1 * v1y) as Coord,
            ));
        }
        None
    }

    /// `Line::distance_to(const Point &point) const` under its C++ name: `distance_to(point, a, b)`, which is
    /// `sqrt(distance_to_squared(point, a, b))`.
    ///
    /// Line.hpp:169, 188
    pub fn distance_to(&self, point: &Point) -> f64 {
        Self::distance_to_squared(*point, self.a, self.b).sqrt()
    }

    /// Static method: Calculate the squared distance from a point to a line segment.
    /// This matches libslic3r's `Line::distance_to_squared()` static method.
    pub fn distance_to_squared(p: Point, a: Point, b: Point) -> f64 {
        // `line_alg::distance_to_squared` (Line.hpp:43-68), in double throughout;
        // the nearest point it also returns is not part of the value. No step is
        // fused in the C++ build (tests/douglas_peucker_native_parity.rs).
        // Line.hpp:45-47
        let vx = (b.x - a.x) as f64;
        let vy = (b.y - a.y) as f64;
        let vax = (p.x - a.x) as f64;
        let vay = (p.y - a.y) as f64;
        let l2 = vx * vx + vy * vy;
        // Line.hpp:48-52 — a == b case
        if l2 == 0.0 {
            return vax * vax + vay * vay;
        }
        // Line.hpp:56
        let t = (vax * vx + vay * vy) / l2;
        if t <= 0.0 {
            // Line.hpp:57-60 — beyond the 'a' end of the segment
            return vax * vax + vay * vay;
        } else if t >= 1.0 {
            // Line.hpp:61-64 — beyond the 'b' end of the segment
            let dx = (p.x - b.x) as f64;
            let dy = (p.y - b.y) as f64;
            return dx * dx + dy * dy;
        }
        // Line.hpp:66-67
        let ex = t * vx - vax;
        let ey = t * vy - vay;
        ex * ex + ey * ey
    }

    /// Squared distance from a point to the infinite line through `a` and `b`.
    /// Line.hpp:87-104 `line_alg::distance_to_infinite_squared`.
    pub fn distance_to_infinite_squared(p: Point, a: Point, b: Point) -> f64 {
        // Line.hpp:90-92
        let vx = (b.x - a.x) as f64;
        let vy = (b.y - a.y) as f64;
        let vax = (p.x - a.x) as f64;
        let vay = (p.y - a.y) as f64;
        let l2 = vx * vx + vy * vy;
        // Line.hpp:93-97
        if l2 == 0.0 {
            return vax * vax + vay * vay;
        }
        // Line.hpp:101-103
        let t = (vax * vx + vay * vy) / l2;
        let ex = t * vx - vax;
        let ey = t * vy - vay;
        ex * ex + ey * ey
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
        // Line.cpp:123, `offset * vector().normalized()`: the vector is divided
        // by its norm first.
        let (ox, oy) = ((offset * (vx / norm)) as Coord, (offset * (vy / norm)) as Coord);
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

/// Eigen's `squaredNorm()` of a 3-vector of doubles: the first two lanes are a packet, reduced by a
/// horizontal add, then the third square is added: `(x*x + y*y) + z*z`, nothing fused.
#[inline]
fn eigen_squared_norm3(x: f64, y: f64, z: f64) -> f64 {
    (x * x + y * y) + z * z
}

/// Line.hpp:212-226 (`class Line3`): a segment of two `Vec3crd` points.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Line3 {
    pub a: crate::geometry::Point3,
    pub b: crate::geometry::Point3,
}

impl Line3 {
    /// Line.hpp:216
    pub fn new(a: crate::geometry::Point3, b: crate::geometry::Point3) -> Self {
        Self { a, b }
    }
    /// Line.hpp:218 (`length()`): `(a - b).cast<double>().norm()`, the difference in `coord_t`.
    pub fn length(&self) -> f64 {
        let dx = (self.a.x as i32).wrapping_sub(self.b.x as i32) as f64;
        let dy = (self.a.y as i32).wrapping_sub(self.b.y as i32) as f64;
        let dz = (self.a.z as i32).wrapping_sub(self.b.z as i32) as f64;
        eigen_squared_norm3(dx, dy, dz).sqrt()
    }
    /// Line.hpp:219 (`vector()`): `b - a` in `coord_t`.
    pub fn vector(&self) -> crate::geometry::Point3 {
        crate::geometry::Point3::new(
            (self.b.x as i32).wrapping_sub(self.a.x as i32) as Coord,
            (self.b.y as i32).wrapping_sub(self.a.y as i32) as Coord,
            (self.b.z as i32).wrapping_sub(self.a.z as i32) as Coord,
        )
    }
}

/// Line.hpp:242-275 (`class Linef3`): a segment of two `Vec3d` points.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Linef3 {
    pub a: crate::geometry::Point3F,
    pub b: crate::geometry::Point3F,
}

impl Linef3 {
    /// Line.hpp:246
    pub fn new(a: crate::geometry::Point3F, b: crate::geometry::Point3F) -> Self {
        Self { a, b }
    }
    /// Line.cpp:128-133 (`Linef3::intersect_plane(double z)`)
    pub fn intersect_plane(&self, z: f64) -> crate::geometry::Point3F {
        // Line.cpp:130: v = (b - a).cast<double>()
        let (vx, vy, vz) = (self.b.x - self.a.x, self.b.y - self.a.y, self.b.z - self.a.z);
        // Line.cpp:131
        let t = (z - self.a.z) / vz;
        // Line.cpp:132: a(0) + v(0) * t, one expression each: fused
        crate::geometry::Point3F::new(vx.mul_add(t, self.a.x), vy.mul_add(t, self.a.y), z)
    }
    /// Line.hpp:249 (`scale(double factor)`): `a *= factor; b *= factor;`
    pub fn scale(&mut self, factor: f64) {
        self.a = crate::geometry::Point3F::new(self.a.x * factor, self.a.y * factor, self.a.z * factor);
        self.b = crate::geometry::Point3F::new(self.b.x * factor, self.b.y * factor, self.b.z * factor);
    }
    /// Line.hpp:250 (`vector()`): `b - a`
    pub fn vector(&self) -> crate::geometry::Point3F {
        crate::geometry::Point3F::new(self.b.x - self.a.x, self.b.y - self.a.y, self.b.z - self.a.z)
    }
    /// Line.hpp:251 (`unit_vector()`): zero for a zero length, else `vector().normalized()` (each
    /// coordinate divided by `sqrt(squaredNorm())`).
    pub fn unit_vector(&self) -> crate::geometry::Point3F {
        if self.length() == 0.0 {
            return crate::geometry::Point3F::new(0.0, 0.0, 0.0);
        }
        let v = self.vector();
        let n2 = eigen_squared_norm3(v.x, v.y, v.z);
        if n2 > 0.0 {
            let n = n2.sqrt();
            crate::geometry::Point3F::new(v.x / n, v.y / n, v.z / n)
        } else {
            v
        }
    }
    /// Line.hpp:252 (`length()`): `vector().norm()`
    pub fn length(&self) -> f64 {
        let v = self.vector();
        eigen_squared_norm3(v.x, v.y, v.z).sqrt()
    }
    /// Line.hpp:253 (`reverse()`): `std::swap(a, b)`
    pub fn reverse(&mut self) {
        std::mem::swap(&mut self.a, &mut self.b);
    }
    /// Line.hpp:260-266 (`static get_point_projection_to_line`): the projection of `pt` onto the line through
    /// `line_start` along `line_dir`; `proj_length` is a float.
    pub fn get_point_projection_to_line(
        pt: &crate::geometry::Point3F,
        line_start: &crate::geometry::Point3F,
        line_dir: &crate::geometry::Point3F,
        intersection_pt: &mut crate::geometry::Point3F,
        proj_length: &mut f32,
    ) {
        // Line.hpp:262: line_dir.normalized()
        let n2 = eigen_squared_norm3(line_dir.x, line_dir.y, line_dir.z);
        let d = if n2 > 0.0 {
            let n = n2.sqrt();
            crate::geometry::Point3F::new(line_dir.x / n, line_dir.y / n, line_dir.z / n)
        } else {
            *line_dir
        };
        // Line.hpp:263-264: BA = pt - line_start (lazy); proj_length = BA.dot(line_dir_), stored as float
        let (bx, by, bz) = (pt.x - line_start.x, pt.y - line_start.y, pt.z - line_start.z);
        *proj_length = ((bx * d.x + by * d.y) + bz * d.z) as f32;
        // Line.hpp:265: line_start + proj_length * line_dir_ (the float promoted to double), unfused
        let pl = *proj_length as f64;
        *intersection_pt = crate::geometry::Point3F::new(line_start.x + pl * d.x, line_start.y + pl * d.y, line_start.z + pl * d.z);
    }
    /// Line.hpp:267-273 (`get_distance_of_point_to_line(pt)`): `(intersection_pt - pt).norm()` as a float.
    pub fn get_distance_of_point_to_line(&self, pt: &crate::geometry::Point3F) -> f32 {
        let mut intersection_pt = crate::geometry::Point3F::new(0.0, 0.0, 0.0);
        let mut proj_length = 0f32;
        Self::get_point_projection_to_line(pt, &self.a, &self.vector(), &mut intersection_pt, &mut proj_length);
        let (dx, dy, dz) = (intersection_pt.x - pt.x, intersection_pt.y - pt.y, intersection_pt.z - pt.z);
        eigen_squared_norm3(dx, dy, dz).sqrt() as f32
    }
}

/// Line.cpp:10-20 (`transform(const Linef3&, const Transform3d&)`): `t * world_line.colwise().homogeneous()`,
/// which Eigen evaluates as `affine.leftCols<3>() * world_line` (a lazy 3x3 by 3x2 product: rows 0 and 1
/// a packet, `fma(a2, z, fma(a1, y, a0 * x))`; row 2 alone, `e0 + (e1 + e2)`) followed by `+=` the
/// translation column (Homogeneous.h, `homogeneous_left_product_impl`).
pub fn transform(line: &Linef3, t: &crate::geometry::Transform3D) -> Linef3 {
    use crate::geometry::geometry::eigen::{coeff_dot3, packet_dot3};
    let m = |r: usize, c: usize| t.get(r, c);
    let apply = |p: &crate::geometry::Point3F| {
        crate::geometry::Point3F::new(
            packet_dot3(m(0, 0), m(0, 1), m(0, 2), p.x, p.y, p.z) + m(0, 3),
            packet_dot3(m(1, 0), m(1, 1), m(1, 2), p.x, p.y, p.z) + m(1, 3),
            coeff_dot3(m(2, 0), m(2, 1), m(2, 2), p.x, p.y, p.z) + m(2, 3),
        )
    };
    // Line.cpp:19
    Linef3::new(apply(&line.a), apply(&line.b))
}

/// Line.hpp:277-290 (`class Line3float`): a segment of two `Vec3f` points.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Line3float {
    pub a: [f32; 3],
    pub b: [f32; 3],
}

impl Line3float {
    /// Line.hpp:281
    pub fn new(a: [f32; 3], b: [f32; 3]) -> Self {
        Self { a, b }
    }
    /// Line.hpp:283 (`vector()`): `b - a`
    pub fn vector(&self) -> [f32; 3] {
        [self.b[0] - self.a[0], self.b[1] - self.a[1], self.b[2] - self.a[2]]
    }
    /// Line.hpp:285 (`length()`): `vector().norm()` in float, returned as double.
    pub fn length(&self) -> f64 {
        let v = self.vector();
        ((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]).sqrt() as f64
    }
    /// Line.hpp:284 (`unit_vector()`): zero for a zero length, else `vector().normalized()` in float.
    pub fn unit_vector(&self) -> [f32; 3] {
        if self.length() == 0.0 {
            return [0.0; 3];
        }
        let v = self.vector();
        let n2 = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
        if n2 > 0.0 {
            let n = n2.sqrt();
            [v[0] / n, v[1] / n, v[2] / n]
        } else {
            v
        }
    }
}

/// `namespace line_alg` of Line.hpp:23-135 for the 2D integer line (`Line`, `Dim` 2, `Scalar` coord_t),
/// under the C++ names (R934). The arithmetic is the one of the verified static
/// `Line::distance_to_squared` above: in double, nothing fused (tests/line_native_parity.rs).
pub mod line_alg {
    use super::{Line, Point};

    /// Line.hpp:25-33 (`Traits<L>`): the end points of a line type.
    pub trait Traits {
        fn get_a(&self) -> Point;
        fn get_b(&self) -> Point;
    }

    impl Traits for Line {
        /// Line.hpp:31
        #[inline]
        fn get_a(&self) -> Point {
            self.a
        }
        /// Line.hpp:32
        #[inline]
        fn get_b(&self) -> Point {
            self.b
        }
    }

    /// Line.hpp:38 (`get_a(L &&l)`)
    #[inline]
    pub fn get_a<L: Traits>(l: &L) -> Point {
        Traits::get_a(l)
    }

    /// Line.hpp:39 (`get_b(L &&l)`)
    #[inline]
    pub fn get_b<L: Traits>(l: &L) -> Point {
        Traits::get_b(l)
    }

    /// Line.hpp:41-68 (`distance_to_squared(line, point, nearest_point)`): the squared distance to the
    /// closest point of the segment, which goes to `nearest_point`.
    pub fn distance_to_squared<L: Traits>(line: &L, point: &Point, nearest_point: &mut Point) -> f64 {
        let (a, b) = (get_a(line), get_b(line));
        // Line.hpp:45-47
        let vx = (b.x - a.x) as f64;
        let vy = (b.y - a.y) as f64;
        let vax = (point.x - a.x) as f64;
        let vay = (point.y - a.y) as f64;
        let l2 = vx * vx + vy * vy; // avoid a sqrt
        if l2 == 0.0 {
            // Line.hpp:48-52: a == b case
            *nearest_point = a;
            return vax * vax + vay * vay;
        }
        // Line.hpp:56: t = [(this-a) . (b-a)] / |b-a|^2
        let t = (vax * vx + vay * vy) / l2;
        if t <= 0.0 {
            // Line.hpp:57-60: beyond the 'a' end of the segment
            *nearest_point = a;
            return vax * vax + vay * vay;
        } else if t >= 1.0 {
            // Line.hpp:61-64: beyond the 'b' end of the segment
            *nearest_point = b;
            let dx = (point.x - b.x) as f64;
            let dy = (point.y - b.y) as f64;
            return dx * dx + dy * dy;
        }
        // Line.hpp:66: (a.cast<double>() + t * v).cast<coord_t>(), truncating
        *nearest_point = Point::new((a.x as f64 + t * vx) as i32 as i64, (a.y as f64 + t * vy) as i32 as i64);
        // Line.hpp:67
        let ex = t * vx - vax;
        let ey = t * vy - vay;
        ex * ex + ey * ey
    }

    /// Line.hpp:79-83 (`distance_to(line, point)`): `sqrt(distance_to_squared(line, point))`.
    #[inline]
    pub fn distance_to<L: Traits>(line: &L, point: &Point) -> f64 {
        // Line.hpp:71-77: distance_to_squared(line, point) with a throw-away nearest point
        let mut nearest_point = Point::new(0, 0);
        distance_to_squared(line, point, &mut nearest_point).sqrt()
    }

    /// Line.hpp:85-104 (`distance_to_infinite_squared(line, point, closest_point)`): the squared distance
    /// to the closest point of the infinite line, which goes to `closest_point` (it may lie beyond a or b).
    pub fn distance_to_infinite_squared<L: Traits>(line: &L, point: &Point, closest_point: &mut Point) -> f64 {
        let (a, b) = (get_a(line), get_b(line));
        // Line.hpp:90-92
        let vx = (b.x - a.x) as f64;
        let vy = (b.y - a.y) as f64;
        let vax = (point.x - a.x) as f64;
        let vay = (point.y - a.y) as f64;
        let l2 = vx * vx + vy * vy; // avoid a sqrt
        if l2 == 0.0 {
            // Line.hpp:93-97: a == b case
            *closest_point = a;
            return vax * vax + vay * vay;
        }
        // Line.hpp:101
        let t = (vax * vx + vay * vy) / l2;
        // Line.hpp:102: (a.cast<double>() + t * v).cast<coord_t>(), truncating
        *closest_point = Point::new((a.x as f64 + t * vx) as i32 as i64, (a.y as f64 + t * vy) as i32 as i64);
        // Line.hpp:103
        let ex = t * vx - vax;
        let ey = t * vy - vay;
        ex * ex + ey * ey
    }

    /// Line.hpp:115-121 (`distance_to_infinite(line, point)`): `sqrt(distance_to_infinite_squared(line, point))`.
    #[inline]
    pub fn distance_to_infinite<L: Traits>(line: &L, point: &Point) -> f64 {
        // Line.hpp:106-113: distance_to_infinite_squared(line, point) with a throw-away closest point
        let mut closest_point = Point::new(0, 0);
        distance_to_infinite_squared(line, point, &mut closest_point).sqrt()
    }
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
