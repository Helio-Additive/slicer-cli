//! Principal component analysis for 2D polygon areas.
//!
//! C++ Reference:
//! - PrincipalComponents2D.hpp (24 lines)
//! - PrincipalComponents2D.cpp (139 lines)
//!
//! This module computes the principal components (eigenvectors) of the area
//! covered by a set of polygons using moment of area calculations.

use crate::geometry::{PointF, Polygons};
use crate::{unscale, CoordF};

/// Epsilon for floating point comparisons
/// PrincipalComponents2D.cpp:11
const EPSILON: f32 = 1e-10;

/// Compute moments of area for a triangle.
///
/// Returns: (area, first_moment_of_area_xy, second_moment_of_area_xy, second_moment_of_area_covariance)
///
/// None of the values is divided/normalized by area.
/// The function computes integral over the area of the triangle:
/// - f(x,y) = x for first moments of area (y is analogous)
/// - f(x,y) = x^2 for second moment of area
/// - f(x,y) = x*y for second moment of area covariance
///
/// PrincipalComponents2D.hpp:14-17
/// PrincipalComponents2D.cpp:13-68
pub fn compute_moments_of_area_of_triangle(
    a: PointF,
    b: PointF,
    c: PointF,
) -> (f32, PointF, PointF, f32) {
    // Based on coordinate transformation guide:
    // PrincipalComponents2D.cpp:14-30
    // Denote the vertices of S by a, b, c. Then the map
    //  g:(u,v)↦a+u(b−a)+v(c−a)
    // which in coordinates appears as
    //  g:(u,v)↦{x(u,v)=a1+u(b1−a1)+v(c1−a1)
    //          y(u,v)=a2+u(b2−a2)+v(c2−a2)
    // maps triangle bijectively. The Jacobian determinant is:
    //  Jg(u,v)=(b1−a1)(c2−a2)−(c1−a1)(b2−a2)

    /// Compute absolute value of Jacobian determinant
    /// PrincipalComponents2D.cpp:32
    let jacobian_determinant_abs = ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs();

    // Second moment of area for x and y coordinates
    // PrincipalComponents2D.cpp:34-42
    // coordinate transform: gx(u,v) = a.x + u * (b.x - a.x) + v * (c.x - a.x)
    // coordinate transform: gy(u,v) = a.y + u * (b.y - a.y) + v * (c.y - a.y)
    // second moment of area for x: f(x, y) = x^2;
    //              f(gx(u,v), gy(u,v)) = gx(u,v)^2 = ... (long expanded form)
    // result is Int_T func = jacobian_determinant_abs * Int_0^1 Int_0^1-u func(gx(u,v), gy(u,v)) dv du
    // integral_0^1 integral_0^(1 - u) (a + u (b - a) + v (c - a))^2 dv du = 1/12 (a^2 + a (b + c) + b^2 + b c + c^2)

    /// Second moment of area vector (component-wise for x and y)
    /// PrincipalComponents2D.cpp:43-56
    let second_moment_of_area_xy = PointF::new(
        jacobian_determinant_abs
            * (a.x * a.x + b.x * b.x + b.x * c.x + c.x * c.x + a.x * (b.x + c.x))
            / 12.0,
        jacobian_determinant_abs
            * (a.y * a.y + b.y * b.y + b.y * c.y + c.y * c.y + a.y * (b.y + c.y))
            / 12.0,
    );

    // Second moment of area covariance: f(x, y) = x*y
    // PrincipalComponents2D.cpp:44-56
    // f(gx(u,v), gy(u,v)) = gx(u,v)*gy(u,v) = ... (long expanded form)
    // (a_1 + u * (b_1 - a_1) + v * (c_1 - a_1)) * (a_2 + u * (b_2 - a_2) + v * (c_2 - a_2))
    // intermediate result: integral_0^(1 - u) (a_1 + u (b_1 - a_1) + v (c_1 - a_1)) (a_2 + u (b_2 - a_2) + v (c_2 - a_2)) dv =
    //  1/6 (u - 1) (-c_1 (u - 1) (a_2 (u - 1) - 3 b_2 u) - c_2 (u - 1) (a_1 (u - 1) - 3 b_1 u + 2 c_1 (u - 1)) + 3 b_1 u (a_2 (u - 1) - 2 b_2 u) + a_1 (u - 1) (3 b_2 u - 2 a_2 (u - 1)))
    // result = integral_0^1 ... du =
    //   1/24 (a_2 (b_1 + c_1) + a_1 (2 a_2 + b_2 + c_2) + b_2 c_1 + b_1 c_2 + 2 b_1 b_2 + 2 c_1 c_2)
    // result is Int_T func = jacobian_determinant_abs * Int_0^1 Int_0^1-u func(gx(u,v), gy(u,v)) dv du

    /// Second moment of area covariance (x*y term)
    /// PrincipalComponents2D.cpp:57-63
    let second_moment_of_area_covariance = (jacobian_determinant_abs
        * (1.0 / 24.0)
        * (a.y * (b.x + c.x)
            + a.x * (2.0 * a.y + b.y + c.y)
            + b.y * c.x
            + b.x * c.y
            + 2.0 * b.x * b.y
            + 2.0 * c.x * c.y)) as f32;

    /// Triangle area
    /// PrincipalComponents2D.cpp:58
    let area = (jacobian_determinant_abs * 0.5) as f32;

    /// First moment of area (centroid * area)
    /// PrincipalComponents2D.cpp:60
    let first_moment_of_area_xy = PointF::new(
        (jacobian_determinant_abs * (a.x + b.x + c.x) / 6.0) as CoordF,
        (jacobian_determinant_abs * (a.y + b.y + c.y) / 6.0) as CoordF,
    );

    /// Return tuple of moments
    /// PrincipalComponents2D.cpp:62
    (
        area,
        first_moment_of_area_xy,
        second_moment_of_area_xy,
        second_moment_of_area_covariance,
    )
}

/// Compute principal components (eigenvectors) of the area covered by polygons.
///
/// Returns two eigenvectors sorted by their corresponding eigenvalue, largest first.
///
/// PrincipalComponents2D.hpp:20-21
/// PrincipalComponents2D.cpp:65-139
pub fn compute_principal_components(polys: &Polygons) -> (PointF, PointF) {
    /// Initialize centroid accumulator
    /// PrincipalComponents2D.cpp:67
    let mut centroid_accumulator = PointF::new(0.0, 0.0);

    /// Initialize second moment of area accumulator
    /// PrincipalComponents2D.cpp:68
    let mut second_moment_of_area_accumulator = PointF::new(0.0, 0.0);

    /// Initialize second moment of area covariance accumulator
    /// PrincipalComponents2D.cpp:69
    let mut second_moment_of_area_covariance_accumulator = 0.0;

    /// Initialize total area accumulator
    /// PrincipalComponents2D.cpp:70
    let mut total_area = 0.0;

    /// Process each polygon
    /// PrincipalComponents2D.cpp:72
    for poly in polys {
        /// Get first point as origin for fan triangulation
        /// PrincipalComponents2D.cpp:73
        let p0_point = poly.points()[0];
        // PrincipalComponents2D.cpp:73
        let p0 = PointF::new(unscale(p0_point.x), unscale(p0_point.y));

        /// Triangulate polygon as a fan from p0
        /// PrincipalComponents2D.cpp:74
        for i in 2..poly.points().len() {
            /// Get second vertex of triangle
            /// PrincipalComponents2D.cpp:75
            let p1_point = poly.points()[i - 1];
            // PrincipalComponents2D.cpp:75
            let p1 = PointF::new(unscale(p1_point.x), unscale(p1_point.y));

            /// Get third vertex of triangle
            /// PrincipalComponents2D.cpp:76
            let p2_point = poly.points()[i];
            // PrincipalComponents2D.cpp:76
            let p2 = PointF::new(unscale(p2_point.x), unscale(p2_point.y));

            /// Determine triangle orientation (CCW = positive, CW = negative)
            /// PrincipalComponents2D.cpp:78
            let cross = (p1.x - p0.x) * (p2.y - p1.y) - (p1.y - p0.y) * (p2.x - p1.x);

            /// Compute sign based on cross product (ternary operator in C++)
            /// PrincipalComponents2D.cpp:78
            let sign =
                // PrincipalComponents2D.cpp:78
                if cross > 0.0 {
                    1.0
                } else {
                    -1.0
                };

            /// Compute moments for this triangle using structured binding
            /// PrincipalComponents2D.cpp:80-81
            let (
                triangle_area,
                first_moment_of_area,
                second_moment_area,
                second_moment_of_area_covariance,
            ) = compute_moments_of_area_of_triangle(p0, p1, p2);

            // Accumulate total area with sign
            // PrincipalComponents2D.cpp:82
            total_area += (sign * triangle_area) as CoordF;

            // Accumulate centroid with sign
            // PrincipalComponents2D.cpp:83
            centroid_accumulator.x += (sign * first_moment_of_area.x as f32) as CoordF;
            // PrincipalComponents2D.cpp:83
            centroid_accumulator.y += (sign * first_moment_of_area.y as f32) as CoordF;

            // Accumulate second moment of area with sign
            // PrincipalComponents2D.cpp:84
            second_moment_of_area_accumulator.x += (sign * second_moment_area.x as f32) as CoordF;
            // PrincipalComponents2D.cpp:84
            second_moment_of_area_accumulator.y += (sign * second_moment_area.y as f32) as CoordF;

            // Accumulate second moment of area covariance with sign
            // PrincipalComponents2D.cpp:85
            second_moment_of_area_covariance_accumulator +=
                (sign * second_moment_of_area_covariance) as CoordF;
        }
    }

    /// Handle zero or negative area
    /// PrincipalComponents2D.cpp:89-91
    if total_area <= 0.0 {
        // PrincipalComponents2D.cpp:90
        return (PointF::new(0.0, 0.0), PointF::new(0.0, 0.0));
    }

    /// Compute centroid by dividing accumulated first moment by area
    /// PrincipalComponents2D.cpp:93
    let centroid = PointF::new(
        centroid_accumulator.x / total_area as CoordF,
        centroid_accumulator.y / total_area as CoordF,
    );

    /// Compute variance by parallel axis theorem
    /// PrincipalComponents2D.cpp:94
    let variance = PointF::new(
        second_moment_of_area_accumulator.x / total_area as CoordF - centroid.x * centroid.x,
        second_moment_of_area_accumulator.y / total_area as CoordF - centroid.y * centroid.y,
    );

    /// Compute covariance by parallel axis theorem
    /// PrincipalComponents2D.cpp:95
    let covariance = (second_moment_of_area_covariance_accumulator / total_area as CoordF
        - centroid.x * centroid.y) as f32;

    /// Handle case where covariance is nearly zero (axes aligned)
    /// PrincipalComponents2D.cpp:101-107
    if covariance.abs() < EPSILON {
        /// Create eigenvectors aligned with axes
        /// PrincipalComponents2D.cpp:102
        let result_a = PointF::new(variance.x, 0.0);
        // PrincipalComponents2D.cpp:102
        let result_b = PointF::new(0.0, variance.y);

        /// Return eigenvectors sorted by variance magnitude
        /// PrincipalComponents2D.cpp:103-106
        if variance.y > variance.x {
            // PrincipalComponents2D.cpp:104
            return (result_b, result_a);
        } else {
            // PrincipalComponents2D.cpp:106
            return (result_a, result_b);
        }
    }

    // Compute eigenvalues of covariance matrix
    // Covariance matrix C is:  | VarX  Cov  |
    //                          | Cov   VarY |
    // Eigenvalues are solutions to det(C - λI) = 0
    // PrincipalComponents2D.cpp:109-117

    /// Compute larger eigenvalue of covariance matrix
    /// PrincipalComponents2D.cpp:110-111
    let eigenvalue_a = (0.5
        * (variance.x as f32
            + variance.y as f32
            + (((variance.x as f32 - variance.y as f32)
                * (variance.x as f32 - variance.y as f32))
                + 4.0 * (covariance * covariance))
                .sqrt())) as CoordF;

    /// Compute smaller eigenvalue of covariance matrix
    /// PrincipalComponents2D.cpp:113-114
    let eigenvalue_b = (0.5
        * (variance.x as f32 + variance.y as f32
            - (((variance.x as f32 - variance.y as f32)
                * (variance.x as f32 - variance.y as f32))
                + 4.0 * (covariance * covariance))
                .sqrt())) as CoordF;

    // Compute eigenvectors
    // For eigenvalue λ, eigenvector v satisfies: Cv = λv
    // From first row: VarX * v_x + Cov * v_y = λ * v_x
    // Therefore: v_x = (λ - VarY) / Cov, v_y = 1

    /// Compute eigenvector for larger eigenvalue
    /// PrincipalComponents2D.cpp:118
    let eigenvector_a = PointF::new(
        ((eigenvalue_a as f32 - variance.y as f32) / covariance) as CoordF,
        1.0,
    );

    /// Compute eigenvector for smaller eigenvalue
    /// PrincipalComponents2D.cpp:119
    let eigenvector_b = PointF::new(
        ((eigenvalue_b as f32 - variance.y as f32) / covariance) as CoordF,
        1.0,
    );

    /// Return eigenvectors sorted by eigenvalue (largest first)
    /// PrincipalComponents2D.cpp:128-132
    if eigenvalue_a > eigenvalue_b {
        // PrincipalComponents2D.cpp:129
        (eigenvector_a, eigenvector_b)
    } else {
        // PrincipalComponents2D.cpp:131
        (eigenvector_b, eigenvector_a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangle_moments_simple() {
        // Simple right triangle at origin
        let a = PointF::new(0.0, 0.0);
        let b = PointF::new(1.0, 0.0);
        let c = PointF::new(0.0, 1.0);

        let (area, _first, _second, _cov) = compute_moments_of_area_of_triangle(a, b, c);

        // Area should be 0.5
        assert!((area - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_principal_components_empty() {
        let polys = vec![];
        let (v1, v2) = compute_principal_components(&polys);

        assert_eq!(v1.x, 0.0);
        assert_eq!(v1.y, 0.0);
        assert_eq!(v2.x, 0.0);
        assert_eq!(v2.y, 0.0);
    }

    #[test]
    fn test_principal_components_square() {
        // Create a square polygon (axis-aligned)
        use crate::geometry::{Point, Polygon};
        let square = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(1000000, 0), // 1mm scaled
            Point::new(1000000, 1000000),
            Point::new(0, 1000000),
        ]);

        let polys = vec![square];
        let (v1, v2) = compute_principal_components(&polys);

        // For a square, principal components should be along axes
        // One should be mostly X, one mostly Y
        let v1_is_x = v1.x.abs() > v1.y.abs();
        let v2_is_y = v2.y.abs() > v2.x.abs();

        assert!(
            v1_is_x || v2_is_y,
            "Expected principal axes roughly aligned with X/Y"
        );
    }
}
