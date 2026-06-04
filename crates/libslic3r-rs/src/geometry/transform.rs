//! Transform types for 2D and 3D affine transformations.
//!
//! This module provides transformation types for geometric operations,
//! mirroring BambuStudio's Transform classes.

use super::{Point, Point3, Point3F, PointF};
use crate::CoordF;
use serde::{Deserialize, Serialize};
use std::fmt;

/// A 2D affine transformation matrix.
///
/// Represented as a 3x3 matrix in homogeneous coordinates:
/// ```text
/// | a  b  tx |
/// | c  d  ty |
/// | 0  0  1  |
/// ```
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform2D {
    /// Matrix element [0,0] - x scale / rotation
    pub a: CoordF,
    /// Matrix element [0,1] - x shear / rotation
    pub b: CoordF,
    /// Matrix element [1,0] - y shear / rotation
    pub c: CoordF,
    /// Matrix element [1,1] - y scale / rotation
    pub d: CoordF,
    /// Translation x
    pub tx: CoordF,
    /// Translation y
    pub ty: CoordF,
}

impl Transform2D {
    // Create an identity transform.
    #[inline]
    pub fn identity() -> Self {
        Self {
            a: 1.0,
            b: 0.0,
            c: 0.0,
            d: 1.0,
            tx: 0.0,
            ty: 0.0,
        }
    }

    /// Create a translation transform.
    #[inline]
    pub fn translation(tx: CoordF, ty: CoordF) -> Self {
        Self {
            a: 1.0,
            b: 0.0,
            c: 0.0,
            d: 1.0,
            tx,
            ty,
        }
    }

    /// Create a scaling transform.
    #[inline]
    pub fn scaling(sx: CoordF, sy: CoordF) -> Self {
        Self {
            a: sx,
            b: 0.0,
            c: 0.0,
            d: sy,
            tx: 0.0,
            ty: 0.0,
        }
    }

    /// Create a uniform scaling transform.
    #[inline]
    pub fn uniform_scaling(s: CoordF) -> Self {
        Self::scaling(s, s)
    }

    /// Create a rotation transform (angle in radians).
    #[inline]
    pub fn rotation(angle: CoordF) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self {
            a: cos_a,
            b: -sin_a,
            c: sin_a,
            d: cos_a,
            tx: 0.0,
            ty: 0.0,
        }
    }

    /// Create a rotation transform around a point.
    pub fn rotation_around(angle: CoordF, cx: CoordF, cy: CoordF) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self {
            a: cos_a,
            b: -sin_a,
            c: sin_a,
            d: cos_a,
            tx: cx - cos_a * cx + sin_a * cy,
            ty: cy - sin_a * cx - cos_a * cy,
        }
    }

    /// Create a shearing transform.
    #[inline]
    pub fn shearing(shx: CoordF, shy: CoordF) -> Self {
        Self {
            a: 1.0,
            b: shx,
            c: shy,
            d: 1.0,
            tx: 0.0,
            ty: 0.0,
        }
    }

    /// Multiply this transform by another (compose transformations).
    /// Returns a transform that first applies self, then other.
    /// Mathematically: result = other * self (so result.apply(p) == other.apply(self.apply(p)))
    pub fn then(&self, other: &Transform2D) -> Self {
        Self {
            a: other.a * self.a + other.b * self.c,
            b: other.a * self.b + other.b * self.d,
            c: other.c * self.a + other.d * self.c,
            d: other.c * self.b + other.d * self.d,
            tx: other.a * self.tx + other.b * self.ty + other.tx,
            ty: other.c * self.tx + other.d * self.ty + other.ty,
        }
    }

    /// Apply this transform to a point.
    #[inline]
    pub fn apply(&self, p: PointF) -> PointF {
        PointF::new(
            self.a * p.x + self.b * p.y + self.tx,
            self.c * p.x + self.d * p.y + self.ty,
        )
    }

    /// Apply this transform to a scaled integer point.
    #[inline]
    pub fn apply_scaled(&self, p: Point) -> Point {
        let pf = p.to_f64();
        self.apply(pf).to_scaled()
    }

    /// Calculate the determinant of the linear part.
    #[inline]
    pub fn determinant(&self) -> CoordF {
        self.a * self.d - self.b * self.c
    }

    /// Check if this transform is invertible.
    #[inline]
    pub fn is_invertible(&self) -> bool {
        self.determinant().abs() > 1e-10
    }

    /// Calculate the inverse of this transform.
    pub fn inverse(&self) -> Option<Self> {
        let det = self.determinant();
        if det.abs() < 1e-10 {
            return None;
        }

        let inv_det = 1.0 / det;
        Some(Self {
            a: self.d * inv_det,
            b: -self.b * inv_det,
            c: -self.c * inv_det,
            d: self.a * inv_det,
            tx: (self.b * self.ty - self.d * self.tx) * inv_det,
            ty: (self.c * self.tx - self.a * self.ty) * inv_det,
        })
    }

    /// Check if this is approximately the identity transform.
    pub fn is_identity(&self, epsilon: CoordF) -> bool {
        (self.a - 1.0).abs() < epsilon
            && self.b.abs() < epsilon
            && self.c.abs() < epsilon
            && (self.d - 1.0).abs() < epsilon
            && self.tx.abs() < epsilon
            && self.ty.abs() < epsilon
    }

    /// Check if this transform has a reflection (negative determinant).
    #[inline]
    pub fn has_reflection(&self) -> bool {
        self.determinant() < 0.0
    }
}

impl Default for Transform2D {
    fn default() -> Self {
        Self::identity()
    }
}

impl fmt::Debug for Transform2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Transform2D([{:.6}, {:.6}, {:.6}; {:.6}, {:.6}, {:.6}])",
            self.a, self.b, self.tx, self.c, self.d, self.ty
        )
    }
}

impl fmt::Display for Transform2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[[{:.4}, {:.4}, {:.4}], [{:.4}, {:.4}, {:.4}]]",
            self.a, self.b, self.tx, self.c, self.d, self.ty
        )
    }
}

/// A 3D affine transformation matrix.
///
/// Represented as a 4x4 matrix in homogeneous coordinates.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform3D {
    /// The 4x4 matrix stored in column-major order.
    /// [m00, m10, m20, m30, m01, m11, m21, m31, m02, m12, m22, m32, m03, m13, m23, m33]
    pub matrix: [CoordF; 16],
}

impl Transform3D {
    // Create an identity transform.
    pub fn identity() -> Self {
        Self {
            matrix: [
                1.0, 0.0, 0.0, 0.0, // Column 0
                0.0, 1.0, 0.0, 0.0, // Column 1
                0.0, 0.0, 1.0, 0.0, // Column 2
                0.0, 0.0, 0.0, 1.0, // Column 3
            ],
        }
    }

    /// Create a translation transform.
    pub fn translation(tx: CoordF, ty: CoordF, tz: CoordF) -> Self {
        Self {
            matrix: [
                1.0, 0.0, 0.0, 0.0, // Column 0
                0.0, 1.0, 0.0, 0.0, // Column 1
                0.0, 0.0, 1.0, 0.0, // Column 2
                tx, ty, tz, 1.0, // Column 3
            ],
        }
    }

    /// Create a scaling transform.
    pub fn scaling(sx: CoordF, sy: CoordF, sz: CoordF) -> Self {
        Self {
            matrix: [
                sx, 0.0, 0.0, 0.0, // Column 0
                0.0, sy, 0.0, 0.0, // Column 1
                0.0, 0.0, sz, 0.0, // Column 2
                0.0, 0.0, 0.0, 1.0, // Column 3
            ],
        }
    }

    /// Create a uniform scaling transform.
    #[inline]
    pub fn uniform_scaling(s: CoordF) -> Self {
        Self::scaling(s, s, s)
    }

    /// Create a rotation transform around the X axis.
    pub fn rotation_x(angle: CoordF) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self {
            matrix: [
                1.0, 0.0, 0.0, 0.0, // Column 0
                0.0, cos_a, sin_a, 0.0, // Column 1
                0.0, -sin_a, cos_a, 0.0, // Column 2
                0.0, 0.0, 0.0, 1.0, // Column 3
            ],
        }
    }

    /// Create a rotation transform around the Y axis.
    pub fn rotation_y(angle: CoordF) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self {
            matrix: [
                cos_a, 0.0, -sin_a, 0.0, // Column 0
                0.0, 1.0, 0.0, 0.0, // Column 1
                sin_a, 0.0, cos_a, 0.0, // Column 2
                0.0, 0.0, 0.0, 1.0, // Column 3
            ],
        }
    }

    /// Create a rotation transform around the Z axis.
    pub fn rotation_z(angle: CoordF) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self {
            matrix: [
                cos_a, sin_a, 0.0, 0.0, // Column 0
                -sin_a, cos_a, 0.0, 0.0, // Column 1
                0.0, 0.0, 1.0, 0.0, // Column 2
                0.0, 0.0, 0.0, 1.0, // Column 3
            ],
        }
    }

    /// Translate this transform by (tx, ty, tz). Returns a new transform with translation applied.
    pub fn translate(self, tx: CoordF, ty: CoordF, tz: CoordF) -> Self {
        self.then(&Transform3D::translation(tx, ty, tz))
    }

    /// Rotate this transform around the Z axis (angle in radians). Returns a new transform.
    pub fn rotate_z(self, angle: CoordF) -> Self {
        self.then(&Transform3D::rotation_z(angle))
    }

    /// Scale this transform by (sx, sy, sz). Returns a new transform.
    pub fn scale(self, sx: CoordF, sy: CoordF, sz: CoordF) -> Self {
        self.then(&Transform3D::scaling(sx, sy, sz))
    }

    /// Get a matrix element by row and column.
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> CoordF {
        self.matrix[col * 4 + row]
    }

    /// Set a matrix element by row and column.
    #[inline]
    pub fn set(&mut self, row: usize, col: usize, value: CoordF) {
        self.matrix[col * 4 + row] = value;
    }

    /// Multiply this transform by another (compose transformations).
    /// Returns a transform that first applies self, then other.
    /// Mathematically: result = other * self (so result.apply(p) == other.apply(self.apply(p)))
    pub fn then(&self, other: &Transform3D) -> Self {
        let mut result = Self::identity();
        for i in 0..4 {
            for j in 0..4 {
                let mut sum = 0.0;
                for k in 0..4 {
                    sum += other.get(i, k) * self.get(k, j);
                }
                result.set(i, j, sum);
            }
        }
        result
    }

    /// Apply this transform to a point.
    pub fn apply(&self, p: Point3F) -> Point3F {
        let x = self.get(0, 0) * p.x + self.get(0, 1) * p.y + self.get(0, 2) * p.z + self.get(0, 3);
        let y = self.get(1, 0) * p.x + self.get(1, 1) * p.y + self.get(1, 2) * p.z + self.get(1, 3);
        let z = self.get(2, 0) * p.x + self.get(2, 1) * p.y + self.get(2, 2) * p.z + self.get(2, 3);
        let w = self.get(3, 0) * p.x + self.get(3, 1) * p.y + self.get(3, 2) * p.z + self.get(3, 3);

        if w.abs() > 1e-10 {
            Point3F::new(x / w, y / w, z / w)
        } else {
            Point3F::new(x, y, z)
        }
    }

    /// Apply this transform to a scaled integer point.
    pub fn apply_scaled(&self, p: Point3) -> Point3 {
        let pf = p.to_f64();
        self.apply(pf).to_scaled()
    }

    /// Get the translation component.
    pub fn translation_component(&self) -> Point3F {
        Point3F::new(self.get(0, 3), self.get(1, 3), self.get(2, 3))
    }

    /// Check if this is approximately the identity transform.
    pub fn is_identity(&self, epsilon: CoordF) -> bool {
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                if (self.get(i, j) - expected).abs() > epsilon {
                    return false;
                }
            }
        }
        true
    }

    /// Calculate the determinant of the 3x3 rotation/scale part.
    pub fn determinant_3x3(&self) -> CoordF {
        self.get(0, 0) * (self.get(1, 1) * self.get(2, 2) - self.get(1, 2) * self.get(2, 1))
            - self.get(0, 1) * (self.get(1, 0) * self.get(2, 2) - self.get(1, 2) * self.get(2, 0))
            + self.get(0, 2) * (self.get(1, 0) * self.get(2, 1) - self.get(1, 1) * self.get(2, 0))
    }

    /// Check if this transform has a reflection (negative determinant).
    #[inline]
    pub fn has_reflection(&self) -> bool {
        self.determinant_3x3() < 0.0
    }
}

impl Default for Transform3D {
    fn default() -> Self {
        Self::identity()
    }
}

impl fmt::Debug for Transform3D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Transform3D(")?;
        for i in 0..4 {
            if i > 0 {
                write!(f, "; ")?;
            }
            write!(
                f,
                "[{:.4}, {:.4}, {:.4}, {:.4}]",
                self.get(i, 0),
                self.get(i, 1),
                self.get(i, 2),
                self.get(i, 3)
            )?;
        }
        write!(f, ")")
    }
}

impl fmt::Display for Transform3D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for i in 0..4 {
            writeln!(
                f,
                "[{:8.4}, {:8.4}, {:8.4}, {:8.4}]",
                self.get(i, 0),
                self.get(i, 1),
                self.get(i, 2),
                self.get(i, 3)
            )?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: CoordF = 1e-10;

    fn approx_eq(a: CoordF, b: CoordF) -> bool {
        (a - b).abs() < EPSILON
    }

    #[test]
    fn test_transform2d_identity() {
        let t = Transform2D::identity();
        assert!(t.is_identity(EPSILON));
    }

    #[test]
    fn test_transform2d_translation() {
        let t = Transform2D::translation(10.0, 20.0);
        let p = PointF::new(5.0, 5.0);
        let result = t.apply(p);
        assert!(approx_eq(result.x, 15.0));
        assert!(approx_eq(result.y, 25.0));
    }

    #[test]
    fn test_transform2d_scaling() {
        let t = Transform2D::scaling(2.0, 3.0);
        let p = PointF::new(5.0, 10.0);
        let result = t.apply(p);
        assert!(approx_eq(result.x, 10.0));
        assert!(approx_eq(result.y, 30.0));
    }

    #[test]
    fn test_transform2d_rotation() {
        let t = Transform2D::rotation(std::f64::consts::FRAC_PI_2); // 90 degrees
        let p = PointF::new(1.0, 0.0);
        let result = t.apply(p);
        assert!(approx_eq(result.x, 0.0));
        assert!(approx_eq(result.y, 1.0));
    }

    #[test]
    fn test_transform2d_compose() {
        let t1 = Transform2D::translation(10.0, 0.0);
        let t2 = Transform2D::scaling(2.0, 2.0);
        let composed = t1.then(&t2);

        let p = PointF::new(5.0, 5.0);
        let result = composed.apply(p);
        // First translate: (15, 5), then scale: (30, 10)
        assert!(approx_eq(result.x, 30.0));
        assert!(approx_eq(result.y, 10.0));
    }

    #[test]
    fn test_transform2d_inverse() {
        let t = Transform2D::translation(10.0, 20.0);
        let inv = t.inverse().unwrap();

        let p = PointF::new(15.0, 25.0);
        let result = inv.apply(p);
        assert!(approx_eq(result.x, 5.0));
        assert!(approx_eq(result.y, 5.0));
    }

    #[test]
    fn test_transform2d_determinant() {
        let t = Transform2D::scaling(2.0, 3.0);
        assert!(approx_eq(t.determinant(), 6.0));
    }

    #[test]
    fn test_transform3d_identity() {
        let t = Transform3D::identity();
        assert!(t.is_identity(EPSILON));
    }

    #[test]
    fn test_transform3d_translation() {
        let t = Transform3D::translation(10.0, 20.0, 30.0);
        let p = Point3F::new(5.0, 5.0, 5.0);
        let result = t.apply(p);
        assert!(approx_eq(result.x, 15.0));
        assert!(approx_eq(result.y, 25.0));
        assert!(approx_eq(result.z, 35.0));
    }

    #[test]
    fn test_transform3d_scaling() {
        let t = Transform3D::scaling(2.0, 3.0, 4.0);
        let p = Point3F::new(5.0, 10.0, 15.0);
        let result = t.apply(p);
        assert!(approx_eq(result.x, 10.0));
        assert!(approx_eq(result.y, 30.0));
        assert!(approx_eq(result.z, 60.0));
    }

    #[test]
    fn test_transform3d_rotation_z() {
        let t = Transform3D::rotation_z(std::f64::consts::FRAC_PI_2); // 90 degrees
        let p = Point3F::new(1.0, 0.0, 0.0);
        let result = t.apply(p);
        assert!(approx_eq(result.x, 0.0));
        assert!(approx_eq(result.y, 1.0));
        assert!(approx_eq(result.z, 0.0));
    }

    #[test]
    fn test_transform3d_compose() {
        let t1 = Transform3D::translation(10.0, 0.0, 0.0);
        let t2 = Transform3D::scaling(2.0, 2.0, 2.0);
        let composed = t1.then(&t2);

        let p = Point3F::new(5.0, 5.0, 5.0);
        let result = composed.apply(p);
        // First translate: (15, 5, 5), then scale: (30, 10, 10)
        assert!(approx_eq(result.x, 30.0));
        assert!(approx_eq(result.y, 10.0));
        assert!(approx_eq(result.z, 10.0));
    }

    #[test]
    fn test_transform3d_determinant() {
        let t = Transform3D::scaling(2.0, 3.0, 4.0);
        assert!(approx_eq(t.determinant_3x3(), 24.0));
    }

    #[test]
    fn test_transform3d_reflection() {
        let t = Transform3D::scaling(-1.0, 1.0, 1.0);
        assert!(t.has_reflection());

        let t2 = Transform3D::scaling(1.0, 1.0, 1.0);
        assert!(!t2.has_reflection());
    }
}
