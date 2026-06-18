//! Point types for 2D and 3D geometry.
//!
//! This module provides point types that mirror BambuStudio's Point class,
//! using scaled integer coordinates for precision.

use crate::{scale, unscale, Coord, CoordF};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// A 2D point with scaled integer coordinates.
///
/// Point.hpp:171-248
///
/// Points use integer coordinates scaled by `SCALING_FACTOR` to avoid
/// floating-point precision issues. 1 unit = 1 nanometer.
///
/// # Example
/// ```
/// use slicer::geometry::Point;
/// use slicer::scale;
///
/// // Create a point at (1mm, 2mm)
/// let p = Point::new(scale(1.0), scale(2.0));
///
/// // Or use new_scale for convenience
/// let p2 = Point::new_scale(1.0, 2.0);
/// ```
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Point {
    /// X coordinate in scaled units
    ///
    /// Point.hpp:174
    pub x: Coord,

    /// Y coordinate in scaled units
    ///
    /// Point.hpp:174
    pub y: Coord,
}

impl Point {
    // Create a new point with the given coordinates.
    //
    // Point.hpp:176
    #[inline]
    pub const fn new(x: Coord, y: Coord) -> Self {
        Self { x, y }
    }

    /// Create a new point from floating-point coordinates (in mm), scaling them.
    ///
    /// Point.hpp:185-186 (`new_scale` -> `coord_t(scale_(x))`).
    /// FIDELITY-NOTE(F2): C++ `scale_` is `scaled<coord_t>` (Point.hpp:537) which
    /// *truncates* `v / SCALING_FACTOR` toward zero, whereas the crate-wide `scale()`
    /// primitive rounds. This is the shared crate scaling posture (cross-cutting); it
    /// is left delegating to `scale()` rather than diverging this one call site.
    #[inline]
    pub fn new_scale(x: CoordF, y: CoordF) -> Self {
        Self {
            x: scale(x),
            y: scale(y),
        }
    }

    /// Create a point at the origin (0, 0).
    ///
    /// Point.hpp:176 (default constructor)
    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0, y: 0 }
    }

    /// Get the x coordinate.
    ///
    /// Point.hpp:174 (accessor)
    #[inline]
    pub const fn x(&self) -> Coord {
        self.x
    }

    /// Get the y coordinate.
    ///
    /// Point.hpp:174 (accessor)
    #[inline]
    pub const fn y(&self) -> Coord {
        self.y
    }

    /// Convert to floating-point coordinates (in mm).
    ///
    /// Point.hpp:132 (unscale function)
    #[inline]
    pub fn to_f64(&self) -> PointF {
        PointF {
            x: unscale(self.x),
            y: unscale(self.y),
        }
    }

    /// Calculate the squared distance to another point.
    /// Returns i128 to avoid overflow with large coordinates.
    ///
    /// Point.hpp:268-269 (is_approx uses similar logic)
    #[inline]
    pub fn distance_squared(&self, other: &Point) -> i128 {
        let dx = (other.x - self.x) as i128;
        let dy = (other.y - self.y) as i128;
        dx * dx + dy * dy
    }

    /// Calculate the distance to another point.
    ///
    /// Point.hpp:268-269 (derived from squared distance)
    #[inline]
    pub fn distance(&self, other: &Point) -> CoordF {
        (self.distance_squared(other) as CoordF).sqrt()
    }

    /// Alias for distance() - used in some contexts for consistency with C++
    #[inline]
    pub fn distance_to(&self, other: &Point) -> CoordF {
        self.distance(other)
    }

    /// Alias for distance() returning f64 explicitly
    #[inline]
    pub fn distance_to_f64(&self, other: Point) -> f64 {
        self.distance(&other)
    }

    /// Calculate the squared length (magnitude) of this point as a vector.
    ///
    /// Point.hpp:355 (shorter_then uses squaredNorm)
    #[inline]
    pub fn length_squared(&self) -> i128 {
        (self.x as i128) * (self.x as i128) + (self.y as i128) * (self.y as i128)
    }

    /// Calculate the length (magnitude) of this point as a vector.
    ///
    /// Point.hpp:144 (derived from length_squared)
    #[inline]
    pub fn length(&self) -> CoordF {
        (self.length_squared() as CoordF).sqrt()
    }

    /// Rotate this point by the given angle (in radians) around the origin.
    ///
    /// Point.hpp:225
    #[inline]
    pub fn rotate(&self, angle: CoordF) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        self.rotate_by_cos_sin(cos_a, sin_a)
    }

    /// Rotate this point by precomputed cos and sin values.
    ///
    /// Point.hpp:226-231
    #[inline]
    pub fn rotate_by_cos_sin(&self, cos_a: CoordF, sin_a: CoordF) -> Self {
        let x = self.x as CoordF;
        let y = self.y as CoordF;
        Self {
            x: (cos_a * x - sin_a * y).round() as Coord,
            y: (cos_a * y + sin_a * x).round() as Coord,
        }
    }

    /// Rotate this point around a center point.
    ///
    /// Point.cpp:48-58
    #[inline]
    pub fn rotate_around(&self, angle: CoordF, center: Point) -> Self {
        // Point.cpp:50-57
        let cur_x = self.x as CoordF; // (double)(*this)(0)
        let cur_y = self.y as CoordF; // (double)(*this)(1)
        let s = angle.sin(); // ::sin(angle)
        let c = angle.cos(); // ::cos(angle)
        let dx = cur_x - center.x as CoordF; // cur_x - (double)center(0)
        let dy = cur_y - center.y as CoordF; // cur_y - (double)center(1)
        Self {
            // (coord_t)round( (double)center(0) + c * dx - s * dy )
            x: (center.x as CoordF + c * dx - s * dy).round() as Coord,
            // (coord_t)round( (double)center(1) + c * dy + s * dx )
            y: (center.y as CoordF + c * dy + s * dx).round() as Coord,
        }
    }

    /// Point.hpp:201-207 (`both_comp`)
    /// Returns true if both coordinates satisfy the comparison `op` ("&gt;" or "&lt;").
    #[inline]
    pub fn both_comp(&self, rhs: &Point, op: &str) -> bool {
        // Point.hpp:202-205
        if op == ">" {
            self.x() > rhs.x() && self.y() > rhs.y()
        } else if op == "<" {
            self.x() < rhs.x() && self.y() < rhs.y()
        } else {
            // Point.hpp:206
            false
        }
    }

    /// Point.hpp:208-215 (`any_comp(const Point&, const std::string&)`)
    /// Returns true if either coordinate satisfies the comparison `op`.
    #[inline]
    pub fn any_comp(&self, rhs: &Point, op: &str) -> bool {
        // Point.hpp:210-213
        if op == ">" {
            self.x() > rhs.x() || self.y() > rhs.y()
        } else if op == "<" {
            self.x() < rhs.x() || self.y() < rhs.y()
        } else {
            // Point.hpp:214
            false
        }
    }

    /// Point.hpp:216-223 (`any_comp(const coord_t, const std::string&)`)
    /// Returns true if either coordinate satisfies the comparison `op` against `val`.
    #[inline]
    pub fn any_comp_val(&self, val: Coord, op: &str) -> bool {
        // Point.hpp:218-221
        if op == ">" {
            self.x() > val || self.y() > val
        } else if op == "<" {
            self.x() < val || self.y() < val
        } else {
            // Point.hpp:222
            false
        }
    }

    /// Rotate 90 degrees counter-clockwise.
    ///
    /// Point.hpp:237
    #[inline]
    pub const fn rotate_90_ccw(&self) -> Self {
        Self {
            x: -self.y,
            y: self.x,
        }
    }

    /// Rotate 90 degrees clockwise.
    ///
    /// Point.hpp:237 (negated)
    #[inline]
    pub const fn rotate_90_cw(&self) -> Self {
        Self {
            x: self.y,
            y: -self.x,
        }
    }

    /// Calculate the cross product with another point (2D pseudo-cross product).
    /// Returns a positive value if other is counter-clockwise from self.
    ///
    /// Point.hpp:86-89
    #[inline]
    pub fn cross(&self, other: &Point) -> i128 {
        (self.x as i128) * (other.y as i128) - (self.y as i128) * (other.x as i128)
    }

    /// Calculate the dot product with another point.
    ///
    /// Point.hpp:86-89 (similar pattern)
    #[inline]
    pub fn dot(&self, other: &Point) -> i128 {
        (self.x as i128) * (other.x as i128) + (self.y as i128) * (other.y as i128)
    }

    /* Three points are a counter-clockwise turn if ccw > 0, clockwise if
     * ccw < 0, and collinear if ccw = 0 because ccw is a determinant that
     * gives the signed area of the triangle formed by p1, p2 and this point.
     * In other words it is the 2D cross product of p1-p2 and p1-this, i.e.
     * z-component of their 3D cross product.
     * We return double because it must be big enough to hold 2*max(|coordinate|)^2
     */
    /// Point.cpp:118-123
    #[inline]
    pub fn ccw(&self, p1: &Point, p2: &Point) -> CoordF {
        // static_assert(sizeof(coord_t) == 4, "Point::ccw() requires a 32 bit coord_t");
        // Point.cpp:121: return cross2((p2 - p1).cast<int64_t>(), (*this - p1).cast<int64_t>());
        // FIDELITY-NOTE(F2): the differences (p2 - p1) and (*this - p1) are evaluated
        // at coord_t == int32_t precision in C++ before being cast<int64_t>(); the
        // `as i32` reproduces that int32 truncation. cross2<int64_t> then computes the
        // 2x2 determinant in int64 arithmetic (wrapping mirrors C++ int64 overflow).
        let a = *p2 - *p1;
        let b = *self - *p1;
        let ax = a.x as i32 as i64;
        let ay = a.y as i32 as i64;
        let bx = b.x as i32 as i64;
        let by = b.y as i32 as i64;
        (ax.wrapping_mul(by).wrapping_sub(ay.wrapping_mul(bx))) as CoordF
    }

    /// Point.cpp:125-128
    #[inline]
    pub fn ccw_line(&self, line: &crate::geometry::Line) -> CoordF {
        // Point.cpp:127: return this->ccw(line.a, line.b);
        self.ccw(&line.a, &line.b)
    }

    /// returns the CCW angle between this-p1 and this-p2
    /// i.e. this assumes a CCW rotation from p1 to p2 around this
    /// Point.cpp:132-139
    #[inline]
    pub fn ccw_angle(&self, p1: &Point, p2: &Point) -> CoordF {
        // FIXME this calculates an atan2 twice! Project one vector into the other!
        // Point.cpp:135-136
        let angle = (p1.x() as CoordF - self.x() as CoordF).atan2(p1.y() as CoordF - self.y() as CoordF)
            - (p2.x() as CoordF - self.x() as CoordF).atan2(p2.y() as CoordF - self.y() as CoordF);
        // we only want to return only positive angles
        // Point.cpp:138
        if angle <= 0.0 {
            angle + 2.0 * std::f64::consts::PI
        } else {
            angle
        }
    }

    /// Find the nearest point in a slice of points, returning its index.
    /// Returns -1 if `points` is empty.
    ///
    /// Point.cpp:60-101 (Points / PointConstPtrs / PointPtrs overloads collapse
    /// to a single slice-based implementation, matching the PointConstPtrs core
    /// at Point.cpp:69-92).
    pub fn nearest_point_index(&self, points: &[Point]) -> i32 {
        // Point.cpp:71-72
        let mut idx: i32 = -1;
        // double because long is limited to 2147483647 on some platforms and it's not enough
        let mut distance: CoordF = -1.0;

        // Point.cpp:74
        for (i, it) in points.iter().enumerate() {
            /* If the X distance of the candidate is > than the total distance of the
               best previous candidate, we know we don't want it */
            // Point.cpp:77: double d = sqr<double>((*this)(0) - (*it)->x());
            // FIDELITY-NOTE(F2): the coordinate difference is computed at coord_t ==
            // int32_t precision before sqr<double> casts it to double.
            let dx = (self.x() as i32).wrapping_sub(it.x() as i32) as CoordF;
            let mut d = dx * dx;
            // Point.cpp:78
            if distance != -1.0 && d > distance {
                continue;
            }

            /* If the Y distance of the candidate is > than the total distance of the
               best previous candidate, we know we don't want it */
            // Point.cpp:82: d += sqr<double>((*this)(1) - (*it)->y());
            let dy = (self.y() as i32).wrapping_sub(it.y() as i32) as CoordF;
            d += dy * dy;
            // Point.cpp:83
            if distance != -1.0 && d > distance {
                continue;
            }

            // Point.cpp:85-86
            idx = i as i32;
            distance = d;

            // Point.cpp:88
            if distance < crate::libslic3r::EPSILON {
                break;
            }
        }

        // Point.cpp:91
        idx
    }

    /// Point.cpp:103-109
    pub fn nearest_point(&self, points: &[Point], point: &mut Point) -> bool {
        // Point.cpp:105
        let idx = self.nearest_point_index(points);
        // Point.cpp:106
        if idx == -1 {
            return false;
        }
        // Point.cpp:107
        *point = points[idx as usize];
        // Point.cpp:108
        true
    }

    /// Project this point onto a line segment defined by two points.
    ///
    /// Point.cpp:157-180 (`Point::projection_onto(const Line &line)`)
    pub fn project_onto_segment(&self, a: Point, b: Point) -> Point {
        // Point.cpp:159: if (line.a == line.b) return line.a;
        if a == b {
            return a;
        }

        /*
            (Ported from VisiLibity by Karl J. Obermeyer)
            The projection of point_temp onto the line determined by
            line_segment_temp can be represented as an affine combination
            expressed in the form projection of
            Point = theta*line_segment_temp.first + (1.0-theta)*line_segment_temp.second.
            If theta is outside the interval [0,1], then one of the Line_Segment's endpoints
            must be closest to calling Point.
        */
        // Point.cpp:170-171
        // FIDELITY-NOTE(F2): line endpoint differences are computed at coord_t ==
        // int32_t precision in C++ before being cast to double (coordf_t).
        let lx = (b.x as i32).wrapping_sub(a.x as i32) as CoordF;
        let ly = (b.y as i32).wrapping_sub(a.y as i32) as CoordF;
        // Point.cpp:172-173
        let bx_minus = (b.x as i32).wrapping_sub(self.x as i32) as CoordF;
        let by_minus = (b.y as i32).wrapping_sub(self.y as i32) as CoordF;
        let theta = (bx_minus * lx + by_minus * ly) / (lx * lx + ly * ly);

        // Point.cpp:175-176
        // (theta * line.a.cast<coordf_t>() + (1.0-theta) * line.b.cast<coordf_t>()).cast<coord_t>()
        // FIDELITY-NOTE(F2): the affine combination is truncated to coord_t == int32_t.
        if (0.0..=1.0).contains(&theta) {
            return Point::new(
                (theta * a.x as CoordF + (1.0 - theta) * b.x as CoordF) as i32 as Coord,
                (theta * a.y as CoordF + (1.0 - theta) * b.y as CoordF) as i32 as Coord,
            );
        }

        // Else pick closest endpoint.
        // Point.cpp:179
        if (a - *self).length_squared() < (b - *self).length_squared() {
            a
        } else {
            b
        }
    }

    /// Point.cpp:141-155 (`Point::projection_onto(const MultiPoint &poly)`)
    pub fn projection_onto_multipoint(&self, poly: &[Point]) -> Point {
        // Point.cpp:143: Point running_projection = poly.first_point();
        let mut running_projection = poly[0];
        // Point.cpp:144: (running_projection - *this).cast<double>().norm()
        // NOTE: C++ takes the raw integer-coordinate Euclidean norm (cast<double>,
        // no unscale). Point::length() computes sqrt(x^2 + y^2) over the raw coords,
        // matching that exactly (and avoiding the SCALING_FACTOR division of to_f64()).
        let mut running_min = (running_projection - *self).length();

        // Point.cpp:146: Lines lines = poly.lines();
        // MultiPoint::lines() yields consecutive segments (a, b) along the polyline.
        // Point.cpp:147
        for w in poly.windows(2) {
            let line = crate::geometry::Line::new(w[0], w[1]);
            // Point.cpp:148
            let point_temp = self.project_onto_segment(line.a, line.b);
            // Point.cpp:149
            if (point_temp - *self).length() < running_min {
                // Point.cpp:150-151
                running_projection = point_temp;
                running_min = (running_projection - *self).length();
            }
        }
        // Point.cpp:154
        running_projection
    }

    /// Point.cpp:183-223 (`Point::is_in_lines(const Points &pts)`)
    pub fn is_in_lines(&self, pts: &[Point]) -> bool {
        // Point.cpp:185
        let check_point = *self;
        // Point.cpp:186
        for pt_idx in 1..pts.len() {
            // Point.cpp:187-188
            let pt = pts[pt_idx];
            let prev_pt = pts[pt_idx - 1];

            // if on the endpoints
            // Point.cpp:191
            if (check_point.x() == pt.x() && check_point.y() == pt.y())
                || (check_point.x() == prev_pt.x() && check_point.y() == prev_pt.y())
            {
                return true;
            }

            // Point.cpp:194-195
            let in_x_range = (check_point.x() > pt.x()) != (check_point.x() > prev_pt.x());
            let in_y_range = (check_point.y() > pt.y()) != (check_point.y() > prev_pt.y());

            // on vert line
            // Point.cpp:198
            if pt.x() == prev_pt.x() {
                // Point.cpp:199
                if in_y_range && pt.x() == check_point.x() {
                    return true;
                }
                continue;
            }

            // on hori line
            // Point.cpp:205
            if pt.y() == prev_pt.y() {
                // Point.cpp:206
                if in_x_range && pt.y() == check_point.y() {
                    return true;
                }
                continue;
            }

            // not right range
            // Point.cpp:212
            if !in_x_range || !in_y_range {
                continue;
            }

            // check if in line
            // Point.cpp:216-217: Line line(prev_pt, pt); double distance = line.distance_to(*this);
            // Line::distance_to(point) => distance to the (clamped) segment.
            let line = crate::geometry::Line::new(prev_pt, pt);
            let distance = line.distance_to_point(&check_point);
            // Point.cpp:218
            if distance.abs() < crate::libslic3r::SCALED_EPSILON {
                return true;
            }
        }

        // Point.cpp:222
        false
    }

    /// Check if this point coincides with another within a tolerance.
    ///
    /// Point.hpp:266-270
    #[inline]
    pub fn coincides_with(&self, other: &Point, tolerance: Coord) -> bool {
        (self.x - other.x).abs() <= tolerance && (self.y - other.y).abs() <= tolerance
    }

    /// Check if this point coincides with another (exact match).
    ///
    /// Point.hpp:266-270 (exact version)
    #[inline]
    pub fn coincides_with_exact(&self, other: &Point) -> bool {
        self.x == other.x && self.y == other.y
    }
}

/// Comparison for Point - lexicographic order (x, then y).
///
/// Point.hpp:171 (comparison operators)
/// C++: bool operator<(const Point &rhs) const { return x < rhs.x || (x == rhs.x && y < rhs.y); }
impl PartialOrd for Point {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Point {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match self.x.cmp(&other.x) {
            std::cmp::Ordering::Equal => self.y.cmp(&other.y),
            ordering => ordering,
        }
    }
}

/// Debug trait for Point.
///
/// Point.hpp:260-264
impl fmt::Debug for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Point({}, {})", self.x, self.y)
    }
}

/// Display trait for Point (outputs in mm).
///
/// Point.hpp:260-264
impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({:.6}, {:.6})", unscale(self.x), unscale(self.y))
    }
}

/// Add two points together.
///
/// Point.hpp:197
impl Add for Point {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

/// Add-assign operator for points.
///
/// Point.hpp:197
impl AddAssign for Point {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}

/// Subtract two points.
///
/// Point.hpp:198
impl Sub for Point {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

/// Subtract-assign operator for points.
///
/// Point.hpp:198
impl SubAssign for Point {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

/// Negate a point.
///
/// Point.hpp:198 (derived)
impl Neg for Point {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

/// Multiply point by scalar.
///
/// Point.hpp:199-200
impl Mul<Coord> for Point {
    type Output = Self;

    #[inline]
    fn mul(self, scalar: Coord) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

/// Multiply point by floating-point scalar.
///
/// Point.hpp:200 (member `Point operator*(const double &rhs)`), which builds the
/// result through `Point(double, double)` (Point.hpp:179) using `coord_t(lrint(x))`.
/// `lrint` rounds using the current FP rounding mode, which defaults to
/// round-to-nearest-even; `round_ties_even` mirrors that. (The free-function
/// `operator*` at Point.hpp:255-258 instead truncates via `coord_t(...)`, but the
/// member operator takes precedence for `Point * double` expressions.)
impl Mul<CoordF> for Point {
    type Output = Self;

    #[inline]
    fn mul(self, scalar: CoordF) -> Self {
        Self {
            x: (self.x as CoordF * scalar).round_ties_even() as Coord,
            y: (self.y as CoordF * scalar).round_ties_even() as Coord,
        }
    }
}

/// Divide point by scalar.
///
/// Point.hpp:199-200 (derived)
impl Div<Coord> for Point {
    type Output = Self;

    #[inline]
    fn div(self, scalar: Coord) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}

/// Convert from tuple to Point.
///
/// Point.hpp:176-187
impl From<(Coord, Coord)> for Point {
    #[inline]
    fn from((x, y): (Coord, Coord)) -> Self {
        Self { x, y }
    }
}

/// Convert from Point to tuple.
///
/// Point.hpp:176-187
impl From<Point> for (Coord, Coord) {
    #[inline]
    fn from(p: Point) -> Self {
        (p.x, p.y)
    }
}

/// Convert from PointF to Point (scale to internal units).
///
/// Point.hpp:132-133
impl From<PointF> for Point {
    #[inline]
    fn from(p: PointF) -> Self {
        Point::new_scale(p.x, p.y)
    }
}

/// A 2D point with floating-point coordinates (in mm, unscaled).
///
/// Point.hpp:171-248 (floating-point variant)
#[derive(Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct PointF {
    pub x: CoordF,
    pub y: CoordF,
}

impl PointF {
    // Create a new floating-point point.
    //
    // Point.hpp:176
    #[inline]
    pub const fn new(x: CoordF, y: CoordF) -> Self {
        Self { x, y }
    }

    /// Create a point at the origin.
    ///
    /// Point.hpp:176 (default constructor)
    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    /// Get the x coordinate.
    ///
    /// Point.hpp:174 (accessor)
    #[inline]
    pub const fn x(&self) -> CoordF {
        self.x
    }

    /// Get the y coordinate.
    ///
    /// Point.hpp:174 (accessor)
    #[inline]
    pub const fn y(&self) -> CoordF {
        self.y
    }

    /// Convert to scaled integer coordinates.
    ///
    /// Point.hpp:132-133
    #[inline]
    pub fn to_scaled(&self) -> Point {
        Point::from(*self)
    }

    /// Calculate the squared distance to another point.
    ///
    /// Point.hpp:268-269 (similar pattern for floats)
    #[inline]
    pub fn distance_squared(&self, other: &PointF) -> CoordF {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dx * dx + dy * dy
    }

    /// Calculate the distance to another point.
    ///
    /// Point.hpp:268-269
    #[inline]
    pub fn distance(&self, other: &PointF) -> CoordF {
        self.distance_squared(other).sqrt()
    }

    /// Calculate the squared length of this point as a vector.
    ///
    /// Point.hpp:355 (similar pattern for floats)
    #[inline]
    pub fn length_squared(&self) -> CoordF {
        self.x * self.x + self.y * self.y
    }

    /// Calculate the length of this point as a vector.
    ///
    /// Point.hpp:144
    #[inline]
    pub fn length(&self) -> CoordF {
        self.length_squared().sqrt()
    }

    /// Normalize this point to unit length.
    ///
    /// Point.hpp:144 (derived)
    #[inline]
    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len > 0.0 {
            Self {
                x: self.x / len,
                y: self.y / len,
            }
        } else {
            *self
        }
    }

    /// Rotate by an angle (in radians).
    ///
    /// Point.hpp:225-231 (floating-point version)
    #[inline]
    pub fn rotate(&self, angle: CoordF) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self {
            x: cos_a * self.x - sin_a * self.y,
            y: cos_a * self.y + sin_a * self.x,
        }
    }

    /// Perpendicular vector (90 degrees counter-clockwise).
    ///
    /// Point.hpp:99
    #[inline]
    pub fn perp(&self) -> Self {
        Self {
            x: -self.y,
            y: self.x,
        }
    }

    /// Dot product with another point.
    ///
    /// Point.hpp:86-89 (similar pattern for floats)
    #[inline]
    pub fn dot(&self, other: &PointF) -> CoordF {
        self.x * other.x + self.y * other.y
    }

    /// Cross product (2D pseudo-cross product).
    ///
    /// Point.hpp:86-89
    #[inline]
    pub fn cross(&self, other: &PointF) -> CoordF {
        self.x * other.y - self.y * other.x
    }

    /// Check if approximately equal to another point.
    ///
    /// Point.hpp:266-270 (floating-point version)
    #[inline]
    pub fn approx_eq(&self, other: &PointF, epsilon: CoordF) -> bool {
        (self.x - other.x).abs() < epsilon && (self.y - other.y).abs() < epsilon
    }
}

/// Debug trait for PointF.
///
/// Point.hpp:260-264
impl fmt::Debug for PointF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PointF({:.6}, {:.6})", self.x, self.y)
    }
}

/// Display trait for PointF.
///
/// Point.hpp:260-264
impl fmt::Display for PointF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({:.6}, {:.6})", self.x, self.y)
    }
}

/// Add two PointF values.
///
/// Point.hpp:197
impl Add for PointF {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

/// Subtract two PointF values.
///
/// Point.hpp:198
impl Sub for PointF {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

/// Negate a PointF.
///
/// Point.hpp:198 (derived)
impl Neg for PointF {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

/// Multiply PointF by scalar.
///
/// Point.hpp:199-200
impl Mul<CoordF> for PointF {
    type Output = Self;

    #[inline]
    fn mul(self, scalar: CoordF) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

/// Divide PointF by scalar.
///
/// Point.hpp:199-200 (derived)
impl Div<CoordF> for PointF {
    type Output = Self;

    #[inline]
    fn div(self, scalar: CoordF) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}

/// Convert from tuple to PointF.
///
/// Point.hpp:176-187
impl From<(CoordF, CoordF)> for PointF {
    #[inline]
    fn from((x, y): (CoordF, CoordF)) -> Self {
        Self { x, y }
    }
}

/// Convert from Point to PointF (unscale to mm).
///
/// Point.hpp:132-133
impl From<Point> for PointF {
    #[inline]
    fn from(p: Point) -> Self {
        p.to_f64()
    }
}

/// A 3D point with scaled integer coordinates.
///
/// Point.hpp:171-248 (3D variant)
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Point3 {
    pub x: Coord,
    pub y: Coord,
    pub z: Coord,
}

impl Point3 {
    // Create a new 3D point.
    //
    // Point.hpp:176
    #[inline]
    pub const fn new(x: Coord, y: Coord, z: Coord) -> Self {
        Self { x, y, z }
    }

    /// Create a new 3D point from floating-point coordinates (in mm).
    ///
    /// Point.hpp:185-186
    #[inline]
    pub fn new_scale(x: CoordF, y: CoordF, z: CoordF) -> Self {
        Self {
            x: scale(x),
            y: scale(y),
            z: scale(z),
        }
    }

    /// Create a point at the origin.
    ///
    /// Point.hpp:176 (default constructor)
    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0, y: 0, z: 0 }
    }

    /// Convert to floating-point coordinates.
    ///
    /// Point.hpp:132-136
    #[inline]
    pub fn to_f64(&self) -> Point3F {
        Point3F {
            x: unscale(self.x),
            y: unscale(self.y),
            z: unscale(self.z),
        }
    }

    /// Project to 2D (drop z coordinate).
    ///
    /// Point.hpp:171-248 (derived)
    #[inline]
    pub const fn to_2d(&self) -> Point {
        Point {
            x: self.x,
            y: self.y,
        }
    }

    /// Calculate squared distance to another point.
    ///
    /// Point.hpp:268-269 (3D variant)
    #[inline]
    pub fn distance_squared(&self, other: &Point3) -> i128 {
        let dx = (other.x - self.x) as i128;
        let dy = (other.y - self.y) as i128;
        let dz = (other.z - self.z) as i128;
        dx * dx + dy * dy + dz * dz
    }

    /// Calculate distance to another point.
    ///
    /// Point.hpp:268-269
    #[inline]
    pub fn distance(&self, other: &Point3) -> CoordF {
        (self.distance_squared(other) as CoordF).sqrt()
    }

    /// Calculate squared length.
    #[inline]
    pub fn length_squared(&self) -> i128 {
        (self.x as i128) * (self.x as i128)
            + (self.y as i128) * (self.y as i128)
            + (self.z as i128) * (self.z as i128)
    }

    /// Calculate length.
    #[inline]
    pub fn length(&self) -> CoordF {
        (self.length_squared() as CoordF).sqrt()
    }

    /// Dot product.
    #[inline]
    pub fn dot(&self, other: &Point3) -> i128 {
        (self.x as i128) * (other.x as i128)
            + (self.y as i128) * (other.y as i128)
            + (self.z as i128) * (other.z as i128)
    }

    /// Cross product.
    #[inline]
    pub fn cross(&self, other: &Point3) -> Point3 {
        Point3 {
            x: ((self.y as i128 * other.z as i128 - self.z as i128 * other.y as i128)
                .clamp(Coord::MIN as i128, Coord::MAX as i128)) as Coord,
            y: ((self.z as i128 * other.x as i128 - self.x as i128 * other.z as i128)
                .clamp(Coord::MIN as i128, Coord::MAX as i128)) as Coord,
            z: ((self.x as i128 * other.y as i128 - self.y as i128 * other.x as i128)
                .clamp(Coord::MIN as i128, Coord::MAX as i128)) as Coord,
        }
    }
}

impl fmt::Debug for Point3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Point3({}, {}, {})", self.x, self.y, self.z)
    }
}

impl fmt::Display for Point3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "({:.6}, {:.6}, {:.6})",
            unscale(self.x),
            unscale(self.y),
            unscale(self.z)
        )
    }
}

impl Add for Point3 {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Point3 {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Neg for Point3 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl From<(Coord, Coord, Coord)> for Point3 {
    #[inline]
    fn from((x, y, z): (Coord, Coord, Coord)) -> Self {
        Self { x, y, z }
    }
}

/// A 3D point with floating-point coordinates (in mm).
#[derive(Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Point3F {
    pub x: CoordF,
    pub y: CoordF,
    pub z: CoordF,
}

impl Point3F {
    // Create a new 3D floating-point point.
    #[inline]
    pub const fn new(x: CoordF, y: CoordF, z: CoordF) -> Self {
        Self { x, y, z }
    }

    /// Get x coordinate
    /// Point.hpp: coordinate accessor
    /// C++: T x() const { return (*this)(0); }
    #[inline]
    pub const fn x(&self) -> CoordF {
        self.x
    }

    /// Get y coordinate
    /// Point.hpp: coordinate accessor
    /// C++: T y() const { return (*this)(1); }
    #[inline]
    pub const fn y(&self) -> CoordF {
        self.y
    }

    /// Get z coordinate
    /// Point.hpp: coordinate accessor
    /// C++: T z() const { return (*this)(2); }
    #[inline]
    pub const fn z(&self) -> CoordF {
        self.z
    }

    /// Create a point at the origin.
    #[inline]
    pub const fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Convert to scaled integer coordinates.
    #[inline]
    pub fn to_scaled(&self) -> Point3 {
        Point3::new_scale(self.x, self.y, self.z)
    }

    /// Project to 2D.
    #[inline]
    pub const fn to_2d(&self) -> PointF {
        PointF {
            x: self.x,
            y: self.y,
        }
    }

    /// Calculate squared distance.
    #[inline]
    pub fn distance_squared(&self, other: &Point3F) -> CoordF {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let dz = other.z - self.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Calculate distance.
    #[inline]
    pub fn distance(&self, other: &Point3F) -> CoordF {
        self.distance_squared(other).sqrt()
    }

    /// Calculate squared length.
    #[inline]
    pub fn length_squared(&self) -> CoordF {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Calculate length.
    #[inline]
    pub fn length(&self) -> CoordF {
        self.length_squared().sqrt()
    }

    /// Normalize to unit length.
    #[inline]
    pub fn normalize(&self) -> Self {
        let len = self.length();
        if len > 0.0 {
            Self {
                x: self.x / len,
                y: self.y / len,
                z: self.z / len,
            }
        } else {
            *self
        }
    }

    /// Dot product.
    #[inline]
    pub fn dot(&self, other: &Point3F) -> CoordF {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product.
    #[inline]
    pub fn cross(&self, other: &Point3F) -> Point3F {
        Point3F {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Check if approximately equal.
    #[inline]
    pub fn approx_eq(&self, other: &Point3F, epsilon: CoordF) -> bool {
        (self.x - other.x).abs() < epsilon
            && (self.y - other.y).abs() < epsilon
            && (self.z - other.z).abs() < epsilon
    }
}

impl fmt::Debug for Point3F {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Point3F({:.6}, {:.6}, {:.6})", self.x, self.y, self.z)
    }
}

impl fmt::Display for Point3F {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({:.6}, {:.6}, {:.6})", self.x, self.y, self.z)
    }
}

impl Add for Point3F {
    type Output = Self;

    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Point3F {
    type Output = Self;

    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Neg for Point3F {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl Mul<CoordF> for Point3F {
    type Output = Self;

    #[inline]
    fn mul(self, scalar: CoordF) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Div<CoordF> for Point3F {
    type Output = Self;

    #[inline]
    fn div(self, scalar: CoordF) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl From<(CoordF, CoordF, CoordF)> for Point3F {
    #[inline]
    fn from((x, y, z): (CoordF, CoordF, CoordF)) -> Self {
        Self { x, y, z }
    }
}

impl From<Point3> for Point3F {
    #[inline]
    fn from(p: Point3) -> Self {
        p.to_f64()
    }
}

/// Type alias for a collection of 2D points.
pub type Points = Vec<Point>;

/// Type alias for a collection of 3D points.
pub type Points3 = Vec<Point3>;

/// Type alias for a collection of 2D floating-point points.
pub type PointsF = Vec<PointF>;

/// Type alias for a collection of 3D floating-point points.
pub type Points3F = Vec<Point3F>;

// ---------------------------------------------------------------------------
// Free functions (Point.cpp:10-301)
// ---------------------------------------------------------------------------

/// Point.cpp:29-46 (`Pointf3s transform(const Pointf3s&, const Transform3d&)`)
///
/// Applies an affine transform to a vector of 3D points (homogeneous).
/// Mirrors `dst = t * src.colwise().homogeneous();`.
///
/// NOTE: The C++ float overload `transform(const std::vector<Vec3f>&, const Transform3f&)`
/// (Point.cpp:10-27) is identical in structure but operates on f32. The crate's
/// `Transform3D` is f64-based, so both overloads collapse to this single f64 path.
pub fn transform(points: &[Point3F], t: &crate::geometry::Transform3D) -> Vec<Point3F> {
    // Point.cpp:31-32
    let vertices_count = points.len();
    if vertices_count == 0 {
        // Point.cpp:33
        return Vec::new();
    }

    // Point.cpp:37-44: src/dst Eigen matrices, dst = t * src.colwise().homogeneous();
    let mut ret_points: Vec<Point3F> = Vec::with_capacity(vertices_count);
    for p in points {
        ret_points.push(t.apply(*p));
    }
    // Point.cpp:45
    ret_points
}

/// if `include_boundary`, then a bounding box is defined even for a single point.
/// otherwise a bounding box is only defined if it has a positive area.
/// Point.cpp:251-259 (`BoundingBox get_extents<IncludeBoundary>(const Points &pts)`)
pub fn get_extents(pts: &[Point], include_boundary: bool) -> crate::geometry::BoundingBox {
    // Point.cpp:254-256
    let mut out = crate::geometry::BoundingBox::new();
    crate::geometry::BoundingBox::construct(&mut out, pts, include_boundary);
    out
}

/// if `include_boundary`, then a bounding box is defined even for a single point.
/// otherwise a bounding box is only defined if it has a positive area.
/// Point.cpp:263-270 (`BoundingBox get_extents<IncludeBoundary>(const VecOfPoints &pts)`)
pub fn get_extents_vec_of_points(
    pts: &[Vec<Point>],
    include_boundary: bool,
) -> crate::geometry::BoundingBox {
    // Point.cpp:266
    let mut bbox = crate::geometry::BoundingBox::new();
    // Point.cpp:267-268
    for p in pts {
        bbox.merge(&get_extents(p, include_boundary));
    }
    // Point.cpp:269
    bbox
}

/// Point.cpp:274-280 (`BoundingBoxf get_extents(const std::vector<Vec2d> &pts)`)
pub fn get_extents_vec2d(pts: &[PointF]) -> crate::geometry::BoundingBoxF {
    // Point.cpp:276
    let mut bbox = crate::geometry::BoundingBoxF::new();
    // Point.cpp:277-278
    for p in pts {
        bbox.merge_point(*p);
    }
    // Point.cpp:279
    bbox
}

/// Test for duplicate points in a vector of points.
/// The points are copied, sorted and checked for duplicates globally.
/// Point.cpp:225-232 (`bool has_duplicate_points(std::vector<Point> &&pts)`)
pub fn has_duplicate_points(mut pts: Vec<Point>) -> bool {
    // Point.cpp:227
    pts.sort();
    // Point.cpp:228-230
    for i in 1..pts.len() {
        if pts[i - 1] == pts[i] {
            return true;
        }
    }
    // Point.cpp:231
    false
}

/// Collect adjecent(duplicit points)
/// Point.cpp:234-249 (`Points collect_duplicates(Points pts /* Copy */)`)
pub fn collect_duplicates(mut pts: Points) -> Points {
    // Point.cpp:236
    pts.sort();
    // Point.cpp:237
    let mut duplicits: Points = Vec::new();
    // Point.cpp:238: const Point *prev = &pts.front();
    let mut prev = pts[0];
    // Point.cpp:239
    for i in 1..pts.len() {
        // Point.cpp:240: const Point *act = &pts[i];
        let act = pts[i];
        // Point.cpp:241
        if prev == act {
            // duplicit point
            // Point.cpp:243: only unique duplicits
            if !duplicits.is_empty() && *duplicits.last().unwrap() == act {
                continue;
            }
            // Point.cpp:244
            duplicits.push(act);
        }
        // Point.cpp:246
        prev = act;
    }
    // Point.cpp:248
    duplicits
}

/// Test for duplicate points in a vector of points.
/// Only successive points are checked for equality.
/// Point.hpp:331-337 (`has_duplicate_successive_points`)
pub fn has_duplicate_successive_points(pts: &[Point]) -> bool {
    // Point.hpp:333-335
    for i in 1..pts.len() {
        if pts[i - 1] == pts[i] {
            return true;
        }
    }
    // Point.hpp:336
    false
}

/// Test for duplicate points in a vector of points.
/// Only successive points are checked for equality. Additionally, first and last points are compared for equality.
/// Point.hpp:341-344 (`has_duplicate_successive_points_closed`)
pub fn has_duplicate_successive_points_closed(pts: &[Point]) -> bool {
    // Point.hpp:343
    has_duplicate_successive_points(pts) || (pts.len() >= 2 && pts[0] == *pts.last().unwrap())
}

/// Point.hpp:266-270 (`is_approx(const Point&, const Point&, coord_t)`)
#[inline]
pub fn is_approx(p1: &Point, p2: &Point, epsilon: Coord) -> bool {
    // Point.hpp:268: Point d = (p2 - p1).cwiseAbs();
    let d = *p2 - *p1;
    // Point.hpp:269
    d.x.abs() < epsilon && d.y.abs() < epsilon
}

/// Point.hpp:272 (`turn90_ccw`)
#[inline]
pub fn turn90_ccw(pt: Point) -> Point {
    Point::new(-pt.y, pt.x)
}

/// Point.hpp:298-302 (`lerp(const Point &a, const Point &b, double t)`)
#[inline]
pub fn lerp(a: &Point, b: &Point, t: CoordF) -> Point {
    // assert((t >= -EPSILON) && (t <= 1. + EPSILON));
    debug_assert!(t >= -crate::libslic3r::EPSILON && t <= 1.0 + crate::libslic3r::EPSILON);
    // Point.hpp:301: ((1. - t) * a.cast<double>() + t * b.cast<double>()).cast<coord_t>()
    // FIDELITY-NOTE(F2): the interpolated double is truncated to coord_t == int32_t.
    Point::new(
        ((1.0 - t) * a.x as CoordF + t * b.x as CoordF) as i32 as Coord,
        ((1.0 - t) * a.y as CoordF + t * b.y as CoordF) as i32 as Coord,
    )
}

/// Point.hpp:349-356 (`shorter_then(const Point& p0, const coord_t len)`)
#[inline]
pub fn shorter_then(p0: &Point, len: Coord) -> bool {
    // Point.hpp:351-352
    if p0.x > len || p0.x < -len {
        return false;
    }
    // Point.hpp:353-354
    if p0.y > len || p0.y < -len {
        return false;
    }
    // Point.hpp:355: p0.cast<int64_t>().squaredNorm() <= Slic3r::sqr(int64_t(len))
    p0.length_squared() <= (len as i128) * (len as i128)
}

/// Align a coordinate to a grid. The coordinate may be negative,
/// the aligned value will never be bigger than the original one.
/// Point.hpp:581-590 (`align_to_grid(coord_t, coord_t)`)
#[inline]
pub fn align_to_grid(coord: Coord, spacing: Coord) -> Coord {
    // Current C++ standard defines the result of integer division to be rounded to zero,
    // for both positive and negative numbers. Here we want to round down for negative
    // numbers as well.
    // Point.hpp:585-587
    let aligned = if coord < 0 {
        ((coord - spacing + 1) / spacing) * spacing
    } else {
        (coord / spacing) * spacing
    };
    debug_assert!(aligned <= coord);
    aligned
}

/// Point.hpp:591-592 (`align_to_grid(Point, Point)`)
#[inline]
pub fn align_to_grid_point(coord: Point, spacing: Point) -> Point {
    Point::new(
        align_to_grid(coord.x, spacing.x),
        align_to_grid(coord.y, spacing.y),
    )
}

/// Point.hpp:593-594 (`align_to_grid(coord_t, coord_t, coord_t)`)
#[inline]
pub fn align_to_grid_base(coord: Coord, spacing: Coord, base: Coord) -> Coord {
    base + align_to_grid(coord - base, spacing)
}

/// Point.hpp:595-596 (`align_to_grid(Point, Point, Point)`)
#[inline]
pub fn align_to_grid_point_base(coord: Point, spacing: Point, base: Point) -> Point {
    Point::new(
        align_to_grid_base(coord.x, spacing.x, base.x),
        align_to_grid_base(coord.y, spacing.y, base.y),
    )
}

/// MinMaxLimits
/// Point.hpp:598-602 (`template<typename T> struct MinMax`)
#[derive(Clone, Copy, Debug)]
pub struct MinMax<T> {
    pub min: T,
    pub max: T,
}

/// Clamp `val` to the `[limit.min, limit.max]` range, returning whether it was modified.
/// Point.hpp:608-619 (`template<typename T> static bool apply(T &val, const MinMax<T> &limit)`)
pub fn apply<T: PartialOrd + Copy>(val: &mut T, limit: &MinMax<T>) -> bool {
    // Point.hpp:610-613
    if *val > limit.max {
        *val = limit.max;
        return true;
    }
    // Point.hpp:614-617
    if *val < limit.min {
        *val = limit.min;
        return true;
    }
    // Point.hpp:618
    false
}

/// Clamp an optional `val` to the `[limit.min, limit.max]` range.
/// Point.hpp:603-607 (`static bool apply(std::optional<T> &val, const MinMax<T> &limit)`)
pub fn apply_opt<T: PartialOrd + Copy>(val: &mut Option<T>, limit: &MinMax<T>) -> bool {
    // Point.hpp:605-606
    match val {
        None => false,
        Some(v) => apply(v, limit),
    }
}

/// To be used by hash maps as a spatial hash of a point.
/// Point.hpp:368-372 (`struct PointHash`)
///
/// C++: `return coord_t((89 * 31 + int64_t(pt.x())) * 31 + pt.y());`
/// The arithmetic is performed in `int64_t` and truncated to `coord_t`, which in
/// BambuStudio is `int32_t` (libslic3r.h:40). The crate-wide `Coord` is `i64`, so
/// the final `as i32` reproduces the C++ int32 truncation bit-for-bit before
/// widening back to `Coord`; `wrapping_*` mirrors the C++ integer overflow.
#[inline]
pub fn point_hash(pt: &Point) -> Coord {
    // FIDELITY-NOTE(F2): pt.x()/pt.y() are coord_t == int32_t in C++; truncate to int32
    // before the int64 hash arithmetic, then truncate the result back to coord_t (int32).
    let px = pt.x as i32 as i64;
    let py = pt.y as i32 as i64;
    ((89i64
        .wrapping_mul(31)
        .wrapping_add(px)
        .wrapping_mul(31)
        .wrapping_add(py)) as i32) as Coord
}

/// A generic class to search for a closest Point in a given radius.
/// It uses a multimap to implement an efficient 2D spatial hashing.
/// The `point_accessor` has to return `Option<&Point>`.
/// If `None` is returned, the value is ignored by the query.
/// Point.hpp:378-509 (`template ClosestPointInRadiusLookup`)
///
/// `ValueType` is the stored payload; `point_accessor` maps a value to its
/// representative `Point` (or `None` to skip it), mirroring the C++
/// `PointAccessor` functor.
pub struct ClosestPointInRadiusLookup<ValueType, PointAccessor>
where
    PointAccessor: Fn(&ValueType) -> Option<Point>,
{
    // Point.hpp:503-508
    m_point_accessor: PointAccessor,
    m_map: std::collections::HashMap<(Coord, Coord), Vec<ValueType>>,
    m_search_radius: Coord,
    m_grid_resolution: Coord,
    m_grid_log2: Coord,
}

impl<ValueType, PointAccessor> ClosestPointInRadiusLookup<ValueType, PointAccessor>
where
    PointAccessor: Fn(&ValueType) -> Option<Point>,
{
    /// Point.hpp:381-411 (constructor)
    pub fn new(search_radius: Coord, point_accessor: PointAccessor) -> Self {
        // Point.hpp:382
        let m_search_radius = search_radius;
        let mut m_grid_log2: Coord = 0;
        // Resolution of a grid, twice the search radius + some epsilon.
        // Point.hpp:385
        let gridres: Coord = 2 * m_search_radius + 4;
        // Point.hpp:386
        let mut m_grid_resolution = gridres;
        // Point.hpp:387-388
        debug_assert!(m_grid_resolution > 0);
        debug_assert!(m_grid_resolution < (1 << 30));
        // Compute m_grid_log2 = log2(m_grid_resolution)
        // Point.hpp:390-393
        if m_grid_resolution > 32767 {
            m_grid_resolution >>= 16;
            m_grid_log2 += 16;
        }
        // Point.hpp:394-397
        if m_grid_resolution > 127 {
            m_grid_resolution >>= 8;
            m_grid_log2 += 8;
        }
        // Point.hpp:398-401
        if m_grid_resolution > 7 {
            m_grid_resolution >>= 4;
            m_grid_log2 += 4;
        }
        // Point.hpp:402-405
        if m_grid_resolution > 1 {
            m_grid_resolution >>= 2;
            m_grid_log2 += 2;
        }
        // Point.hpp:406-407
        if m_grid_resolution > 0 {
            m_grid_log2 += 1;
        }
        // Point.hpp:408
        m_grid_resolution = 1 << m_grid_log2;
        // Point.hpp:409-410
        debug_assert!(m_grid_resolution >= gridres);
        debug_assert!(gridres >= m_grid_resolution / 2);
        Self {
            m_point_accessor: point_accessor,
            m_map: std::collections::HashMap::new(),
            m_search_radius,
            m_grid_resolution,
            m_grid_log2,
        }
    }

    /// Point.hpp:413-417 / Point.hpp:419-423 (`insert`)
    pub fn insert(&mut self, value: ValueType) {
        // Point.hpp:414-416
        if let Some(pt) = (self.m_point_accessor)(&value) {
            self.m_map
                .entry((pt.x >> self.m_grid_log2, pt.y >> self.m_grid_log2))
                .or_default()
                .push(value);
        }
    }

    /// Erase a data point equal to value. (`ValueType` has to implement `PartialEq`).
    /// Returns true if the data point equal to value was found and removed.
    /// Point.hpp:427-441 (`erase`)
    pub fn erase(&mut self, value: &ValueType) -> bool
    where
        ValueType: PartialEq,
    {
        // Point.hpp:428-429
        if let Some(pt) = (self.m_point_accessor)(value) {
            // Range of fragment starts around grid_corner, close to pt.
            // Point.hpp:431
            let key = (pt.x >> self.m_grid_log2, pt.y >> self.m_grid_log2);
            if let Some(bucket) = self.m_map.get_mut(&key) {
                // Remove the first item.
                // Point.hpp:433-438
                if let Some(idx) = bucket.iter().position(|it| it == value) {
                    bucket.remove(idx);
                    return true;
                }
            }
        }
        // Point.hpp:440
        false
    }

    /// Return a pair of `(&ValueType, distance_squared)`.
    /// Point.hpp:444-473 (`find`)
    pub fn find(&self, pt: &Point) -> Option<(&ValueType, CoordF)> {
        // Iterate over 4 closest grid cells around pt,
        // find the closest start point inside these cells to pt.
        // Point.hpp:447-448
        let mut value_min: Option<&ValueType> = None;
        let mut dist_min: CoordF = CoordF::MAX;
        // Round pt to a closest grid_cell corner.
        // Point.hpp:450
        let grid_corner = Point::new(
            (pt.x + (self.m_grid_resolution >> 1)) >> self.m_grid_log2,
            (pt.y + (self.m_grid_resolution >> 1)) >> self.m_grid_log2,
        );
        // For four neighbors of grid_corner:
        // Point.hpp:452-453
        for neighbor_y in -1..1 {
            for neighbor_x in -1..1 {
                // Range of fragment starts around grid_corner, close to pt.
                // Point.hpp:455
                let key = (grid_corner.x + neighbor_x, grid_corner.y + neighbor_y);
                if let Some(bucket) = self.m_map.get(&key) {
                    // Find the map entry closest to pt.
                    // Point.hpp:457-467
                    for value in bucket {
                        if let Some(pt2) = (self.m_point_accessor)(value) {
                            // const double d2 = (pt - *pt2).cast<double>().squaredNorm();
                            let d = *pt - pt2;
                            let d2 = (d.x as CoordF) * (d.x as CoordF)
                                + (d.y as CoordF) * (d.y as CoordF);
                            if d2 < dist_min {
                                dist_min = d2;
                                value_min = Some(value);
                            }
                        }
                    }
                }
            }
        }
        // Point.hpp:470-472
        match value_min {
            Some(v)
                if dist_min < self.m_search_radius as CoordF * self.m_search_radius as CoordF =>
            {
                Some((v, dist_min))
            }
            _ => None,
        }
    }

    /// Returns all pairs of values and squared distances.
    /// Point.hpp:476-500 (`find_all`)
    pub fn find_all(&self, pt: &Point) -> Vec<(&ValueType, CoordF)> {
        // Iterate over 4 closest grid cells around pt,
        // Round pt to a closest grid_cell corner.
        // Point.hpp:479
        let grid_corner = Point::new(
            (pt.x + (self.m_grid_resolution >> 1)) >> self.m_grid_log2,
            (pt.y + (self.m_grid_resolution >> 1)) >> self.m_grid_log2,
        );
        // For four neighbors of grid_corner:
        // Point.hpp:481-482
        let mut out: Vec<(&ValueType, CoordF)> = Vec::new();
        let r2 = self.m_search_radius as CoordF * self.m_search_radius as CoordF;
        // Point.hpp:483-484
        for neighbor_y in -1..1 {
            for neighbor_x in -1..1 {
                // Range of fragment starts around grid_corner, close to pt.
                // Point.hpp:486
                let key = (grid_corner.x + neighbor_x, grid_corner.y + neighbor_y);
                if let Some(bucket) = self.m_map.get(&key) {
                    // Find the map entry closest to pt.
                    // Point.hpp:488-495
                    for value in bucket {
                        if let Some(pt2) = (self.m_point_accessor)(value) {
                            // const double d2 = (pt - *pt2).cast<double>().squaredNorm();
                            let d = *pt - pt2;
                            let d2 = (d.x as CoordF) * (d.x as CoordF)
                                + (d.y as CoordF) * (d.y as CoordF);
                            if d2 <= r2 {
                                out.push((value, d2));
                            }
                        }
                    }
                }
            }
        }
        // Point.hpp:499
        out
    }
}

/// Point.cpp:287-301 / Point.hpp:358-365 — exact orientation predicates.
pub mod int128 {
    use super::Point;
    use crate::int128::Int128;

    /// Exact orientation predicate,
    /// returns +1: CCW, 0: collinear, -1: CW.
    /// Point.cpp:289-294 (`int orient(const Vec2crd&, const Vec2crd&, const Vec2crd&)`)
    pub fn orient(p1: &Point, p2: &Point, p3: &Point) -> i32 {
        // Point.cpp:291-292
        let v1 = *p2 - *p1; // Slic3r::Vector v1(p2 - p1);
        let v2 = *p3 - *p1; // Slic3r::Vector v2(p3 - p1);
        // FIDELITY-NOTE(F2): v1/v2 are coord_t == int32_t vectors in C++; truncate to
        // int32 before feeding the exact int128 determinant predicate.
        // Point.cpp:293
        Int128::sign_determinant_2x2_filtered(
            v1.x as i32 as i64,
            v1.y as i32 as i64,
            v2.x as i32 as i64,
            v2.y as i32 as i64,
        )
    }

    /// Exact orientation predicate,
    /// returns +1: CCW, 0: collinear, -1: CW.
    /// Point.cpp:296-299 (`int cross(const Vec2crd&, const Vec2crd&)`)
    pub fn cross(v1: &Point, v2: &Point) -> i32 {
        // FIDELITY-NOTE(F2): v1/v2 are coord_t == int32_t vectors in C++; truncate to
        // int32 before feeding the exact int128 determinant predicate.
        // Point.cpp:298
        Int128::sign_determinant_2x2_filtered(
            v1.x as i32 as i64,
            v1.y as i32 as i64,
            v2.x as i32 as i64,
            v2.y as i32 as i64,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SCALING_FACTOR;

    #[test]
    fn test_point_new() {
        let p = Point::new(100, 200);
        assert_eq!(p.x, 100);
        assert_eq!(p.y, 200);
    }

    #[test]
    fn test_point_new_scale() {
        let p = Point::new_scale(1.0, 2.0);
        assert_eq!(p.x, SCALING_FACTOR as Coord);
        assert_eq!(p.y, 2 * SCALING_FACTOR as Coord);
    }

    #[test]
    fn test_point_to_f64() {
        let p = Point::new(SCALING_FACTOR as Coord, 2 * SCALING_FACTOR as Coord);
        let pf = p.to_f64();
        assert!((pf.x - 1.0).abs() < 1e-10);
        assert!((pf.y - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_point_distance() {
        let p1 = Point::new(0, 0);
        let p2 = Point::new(3_000_000, 4_000_000); // 3mm, 4mm
        let dist = p1.distance(&p2);
        // Should be 5mm = 5_000_000 units
        assert!((dist - 5_000_000.0).abs() < 1.0);
    }

    #[test]
    fn test_point_rotate() {
        let p = Point::new(1_000_000, 0); // 1mm on x-axis
        let rotated = p.rotate(std::f64::consts::FRAC_PI_2); // Rotate 90 degrees
        assert!(rotated.x.abs() < 100); // Should be ~0
        assert!((rotated.y - 1_000_000).abs() < 100); // Should be ~1mm
    }

    #[test]
    fn test_point_rotate_90_ccw() {
        let p = Point::new(1, 0);
        let rotated = p.rotate_90_ccw();
        assert_eq!(rotated.x, 0);
        assert_eq!(rotated.y, 1);
    }

    #[test]
    fn test_point_arithmetic() {
        let p1 = Point::new(10, 20);
        let p2 = Point::new(3, 4);

        let sum = p1 + p2;
        assert_eq!(sum.x, 13);
        assert_eq!(sum.y, 24);

        let diff = p1 - p2;
        assert_eq!(diff.x, 7);
        assert_eq!(diff.y, 16);

        let neg = -p1;
        assert_eq!(neg.x, -10);
        assert_eq!(neg.y, -20);
    }

    #[test]
    fn test_point_cross() {
        let v1 = Point::new(1, 0);
        let v2 = Point::new(0, 1);
        assert_eq!(v1.cross(&v2), 1);
        assert_eq!(v2.cross(&v1), -1);
    }

    #[test]
    fn test_point_dot() {
        let v1 = Point::new(3, 4);
        let v2 = Point::new(2, 5);
        assert_eq!(v1.dot(&v2), 3 * 2 + 4 * 5);
    }

    #[test]
    fn test_point3_basics() {
        let p = Point3::new(1, 2, 3);
        assert_eq!(p.x, 1);
        assert_eq!(p.y, 2);
        assert_eq!(p.z, 3);

        let p2d = p.to_2d();
        assert_eq!(p2d.x, 1);
        assert_eq!(p2d.y, 2);
    }

    #[test]
    fn test_point3_cross() {
        let v1 = Point3::new(1, 0, 0);
        let v2 = Point3::new(0, 1, 0);
        let cross = v1.cross(&v2);
        assert_eq!(cross.x, 0);
        assert_eq!(cross.y, 0);
        assert_eq!(cross.z, 1);
    }

    #[test]
    fn test_pointf_normalize() {
        let p = PointF::new(3.0, 4.0);
        let n = p.normalize();
        assert!((n.length() - 1.0).abs() < 1e-10);
        assert!((n.x - 0.6).abs() < 1e-10);
        assert!((n.y - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_nearest_point_index() {
        let target = Point::new(0, 0);
        let points = vec![Point::new(100, 100), Point::new(10, 10), Point::new(50, 50)];
        assert_eq!(target.nearest_point_index(&points), 1);
        // empty -> -1 (Point.cpp:71,91)
        assert_eq!(target.nearest_point_index(&[]), -1);
    }

    #[test]
    fn test_project_onto_segment() {
        let p = Point::new(5, 5);
        let a = Point::new(0, 0);
        let b = Point::new(10, 0);
        let proj = p.project_onto_segment(a, b);
        assert_eq!(proj.x, 5);
        assert_eq!(proj.y, 0);
    }

    #[test]
    fn test_get_extents() {
        let pts = vec![Point::new(1, 2), Point::new(5, -3), Point::new(0, 4)];
        // include_boundary = true -> defined.
        let bb = get_extents(&pts, true);
        assert!(bb.is_defined());
        assert_eq!(bb.min, Point::new(0, -3));
        assert_eq!(bb.max, Point::new(5, 4));
        // Single point with include_boundary=false has no positive area -> undefined.
        let bb1 = get_extents(&[Point::new(7, 7)], false);
        assert!(!bb1.is_defined());
        // Single point with include_boundary=true -> defined.
        let bb2 = get_extents(&[Point::new(7, 7)], true);
        assert!(bb2.is_defined());
        // Empty -> undefined.
        assert!(!get_extents(&[], false).is_defined());
    }

    #[test]
    fn test_get_extents_vec_of_points() {
        let a = vec![Point::new(0, 0), Point::new(2, 2)];
        let b = vec![Point::new(-1, 5), Point::new(3, -4)];
        let bb = get_extents_vec_of_points(&[a, b], false);
        assert_eq!(bb.min, Point::new(-1, -4));
        assert_eq!(bb.max, Point::new(3, 5));
    }

    #[test]
    fn test_point_hash_matches_cpp_formula() {
        let pt = Point::new(123, 456);
        let expected = ((89i64 * 31 + 123) * 31 + 456) as Coord;
        assert_eq!(point_hash(&pt), expected);
    }

    #[test]
    fn test_both_any_comp() {
        let a = Point::new(5, 5);
        let b = Point::new(3, 7);
        assert!(!a.both_comp(&b, ">"));
        assert!(a.any_comp(&b, ">"));
        assert!(b.both_comp(&Point::new(2, 2), ">"));
        assert!(a.any_comp_val(4, ">"));
        assert!(!a.any_comp_val(10, ">"));
        assert!(!a.both_comp(&b, "=="));
    }

    #[test]
    fn test_apply_clamp() {
        let limit = MinMax { min: 0i64, max: 10i64 };
        let mut v = 15i64;
        assert!(apply(&mut v, &limit));
        assert_eq!(v, 10);
        let mut v2 = -5i64;
        assert!(apply(&mut v2, &limit));
        assert_eq!(v2, 0);
        let mut v3 = 5i64;
        assert!(!apply(&mut v3, &limit));
        assert_eq!(v3, 5);
        let mut none: Option<i64> = None;
        assert!(!apply_opt(&mut none, &limit));
    }

    #[test]
    fn test_closest_point_in_radius_lookup() {
        // Stored value is the point itself; accessor returns it directly.
        let mut lookup =
            ClosestPointInRadiusLookup::new(100, |p: &Point| Some(*p));
        lookup.insert(Point::new(0, 0));
        lookup.insert(Point::new(10, 10));
        lookup.insert(Point::new(500, 500));
        // Query near origin: closest is (0,0), within radius.
        let found = lookup.find(&Point::new(2, 2));
        assert!(found.is_some());
        let (v, d2) = found.unwrap();
        assert_eq!(*v, Point::new(0, 0));
        assert!((d2 - 8.0).abs() < 1e-9);
        // Query far from everything but inside radius of (500,500) is out of grid neighbors range.
        // find_all near origin returns the two nearby points.
        let all = lookup.find_all(&Point::new(0, 0));
        assert!(all.iter().any(|(p, _)| **p == Point::new(0, 0)));
        // erase removes the value.
        assert!(lookup.erase(&Point::new(0, 0)));
        assert!(!lookup.erase(&Point::new(0, 0)));
    }
}
