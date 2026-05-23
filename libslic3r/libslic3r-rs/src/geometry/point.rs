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
    /// Point.hpp:185-186
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
        let translated = *self - center;
        translated.rotate(angle) + center
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

    /// Calculate the CCW (counter-clockwise) value for three points.
    /// Positive if p1->self->p2 is counter-clockwise.
    ///
    /// Point.cpp:118-123
    #[inline]
    pub fn ccw(&self, p1: &Point, p2: &Point) -> i128 {
        let v1 = *p1 - *self;
        let v2 = *p2 - *self;
        v1.cross(&v2)
    }

    /// Find the nearest point in a slice of points, returning its index.
    ///
    /// Point.cpp:60-92
    pub fn nearest_point_index(&self, points: &[Point]) -> Option<usize> {
        if points.is_empty() {
            return None;
        }

        let mut min_dist = i128::MAX;
        let mut min_idx = 0;

        for (i, p) in points.iter().enumerate() {
            let dist = self.distance_squared(p);
            if dist < min_dist {
                min_dist = dist;
                min_idx = i;
            }
        }

        Some(min_idx)
    }

    /// Project this point onto a line segment defined by two points.
    ///
    /// Point.cpp:157-180
    pub fn project_onto_segment(&self, a: Point, b: Point) -> Point {
        let ab = b - a;
        let ap = *self - a;

        let ab_len_sq = ab.length_squared();
        if ab_len_sq == 0 {
            return a;
        }

        let t = (ap.dot(&ab) as CoordF / ab_len_sq as CoordF).clamp(0.0, 1.0);

        Point::new(
            (a.x as CoordF + t * ab.x as CoordF).round() as Coord,
            (a.y as CoordF + t * ab.y as CoordF).round() as Coord,
        )
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
/// Point.hpp:199-200
impl Mul<CoordF> for Point {
    type Output = Self;

    #[inline]
    fn mul(self, scalar: CoordF) -> Self {
        Self {
            x: (self.x as CoordF * scalar).round() as Coord,
            y: (self.y as CoordF * scalar).round() as Coord,
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
        assert_eq!(target.nearest_point_index(&points), Some(1));
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
}
