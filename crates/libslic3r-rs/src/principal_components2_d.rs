//! Principal component analysis for 2D polygon areas.
//!
//! C++ Reference:
//! - PrincipalComponents2D.hpp (24 lines)
//! - PrincipalComponents2D.cpp (139 lines)
//!
//! 1:1 faithful port of `src/libslic3r/PrincipalComponents2D.{hpp,cpp}`.
//!
//! NOTE ON TYPES: the C++ performs all of its arithmetic in `float` (`Vec2f`),
//! with the single exception of `covariance` which is promoted to `double` at
//! `PrincipalComponents2D.cpp:97`. To preserve byte-exact parity we mirror
//! `Vec2f` with a plain `(f32, f32)` tuple and keep every intermediate in `f32`
//! exactly as the C++ does. The public functions widen the resulting `f32`
//! `Vec2f` losslessly into the crate's `PointF` (`f64`) only at the return
//! boundary, so the original `f32` bit pattern is recoverable.

use crate::geometry::{PointF, Polygons};
use crate::unscale;

// `EPSILON` is the libslic3r-wide constant, declared `static constexpr double
// EPSILON = 1e-4;` in libslic3r.h:52 and used as a `double` at
// PrincipalComponents2D.cpp:104.
// PrincipalComponents2D.cpp:104 (libslic3r.h:52)
const EPSILON: f64 = 1e-4;

/// 2D float vector mirror of Eigen's `Vec2f`.
type Vec2f = (f32, f32);

// returns triangle area, first_moment_of_area_xy, second_moment_of_area_xy, second_moment_of_area_covariance
// none of the values is divided/normalized by area.
// The function computes intgeral over the area of the triangle, with function f(x,y) = x for first moments of area (y is analogous)
// f(x,y) = x^2 for second moment of area
// and f(x,y) = x*y for second moment of area covariance
// PrincipalComponents2D.cpp:8-13
pub fn compute_moments_of_area_of_triangle(a: Vec2f, b: Vec2f, c: Vec2f) -> (f32, Vec2f, Vec2f, f32) {
    // based on the following guide:
    // Denote the vertices of S by a, b, c. Then the map
    //  g:(u,v)↦a+u(b−a)+v(c−a) ,
    //  which in coordinates appears as
    //  g:(u,v)↦{x(u,v)y(u,v)=a1+u(b1−a1)+v(c1−a1)=a2+u(b2−a2)+v(c2−a2) ,(1)
    //  obviously maps S′ bijectively onto S. Therefore the transformation formula for multiple integrals steps into action, and we obtain
    //  ∫Sf(x,y)d(x,y)=∫S′f(x(u,v),y(u,v))∣∣Jg(u,v)∣∣ d(u,v) .
    //  In the case at hand the Jacobian determinant is a constant: From (1) we obtain
    //  Jg(u,v)=det[xuyuxvyv]=(b1−a1)(c2−a2)−(c1−a1)(b2−a2) .
    //  Therefore we can write
    //  ∫Sf(x,y)d(x,y)=∣∣Jg∣∣∫10∫1−u0f~(u,v) dv du ,
    //  where f~ denotes the pullback of f to S′:
    //  f~(u,v):=f(x(u,v),y(u,v)) .
    //  Don't forget taking the absolute value of Jg!

    // PrincipalComponents2D.cpp:30
    let jacobian_determinant_abs = ((b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1)).abs();

    // coordinate transform: gx(u,v) = a.x + u * (b.x - a.x) + v * (c.x - a.x)
    // coordinate transform: gy(u,v) = a.y + u * (b.y - a.y) + v * (c.y - a.y)
    // second moment of area for x: f(x, y) = x^2;
    //              f(gx(u,v), gy(u,v)) = gx(u,v)^2 = ... (long expanded form)

    // result is Int_T func = jacobian_determinant_abs * Int_0^1 Int_0^1-u func(gx(u,v), gy(u,v)) dv du
    // integral_0^1 integral_0^(1 - u) (a + u (b - a) + v (c - a))^2 dv du = 1/12 (a^2 + a (b + c) + b^2 + b c + c^2)

    // PrincipalComponents2D.cpp:40-43
    //   jacobian_determinant_abs * (a*a + b*b + b*c + c*c + a*(b+c)) / 12  (component-wise Vec2f)
    let second_moment_of_area_xy: Vec2f = (
        jacobian_determinant_abs * (a.0 * a.0 + b.0 * b.0 + b.0 * c.0 + c.0 * c.0 + a.0 * (b.0 + c.0)) / 12.0_f32,
        jacobian_determinant_abs * (a.1 * a.1 + b.1 * b.1 + b.1 * c.1 + c.1 * c.1 + a.1 * (b.1 + c.1)) / 12.0_f32,
    );
    // second moment of area covariance : f(x, y) = x*y;
    //              f(gx(u,v), gy(u,v)) = gx(u,v)*gy(u,v) = ... (long expanded form)
    //(a_1 + u * (b_1 - a_1) + v * (c_1 - a_1)) * (a_2 + u * (b_2 - a_2) + v * (c_2 - a_2))
    // ==    (a_1 + u (b_1 - a_1) + v (c_1 - a_1)) (a_2 + u (b_2 - a_2) + v (c_2 - a_2))

    // intermediate result: integral_0^(1 - u) (a_1 + u (b_1 - a_1) + v (c_1 - a_1)) (a_2 + u (b_2 - a_2) + v (c_2 - a_2)) dv =
    //  1/6 (u - 1) (-c_1 (u - 1) (a_2 (u - 1) - 3 b_2 u) - c_2 (u - 1) (a_1 (u - 1) - 3 b_1 u + 2 c_1 (u - 1)) + 3 b_1 u (a_2 (u - 1) - 2
    //  b_2 u) + a_1 (u - 1) (3 b_2 u - 2 a_2 (u - 1))) result = integral_0^1 1/6 (u - 1) (-c_1 (u - 1) (a_2 (u - 1) - 3 b_2 u) - c_2 (u -
    //  1) (a_1 (u - 1) - 3 b_1 u + 2 c_1 (u - 1)) + 3 b_1 u (a_2 (u - 1) - 2 b_2 u) + a_1 (u - 1) (3 b_2 u - 2 a_2 (u - 1))) du =
    //   1/24 (a_2 (b_1 + c_1) + a_1 (2 a_2 + b_2 + c_2) + b_2 c_1 + b_1 c_2 + 2 b_1 b_2 + 2 c_1 c_2)
    //  result is Int_T func = jacobian_determinant_abs * Int_0^1 Int_0^1-u func(gx(u,v), gy(u,v)) dv du
    // PrincipalComponents2D.cpp:55-57
    let second_moment_of_area_covariance = jacobian_determinant_abs
        * (1.0_f32 / 24.0_f32)
        * (a.1 * (b.0 + c.0)
            + a.0 * (2.0_f32 * a.1 + b.1 + c.1)
            + b.1 * c.0
            + b.0 * c.1
            + 2.0_f32 * b.0 * b.1
            + 2.0_f32 * c.0 * c.1);

    // PrincipalComponents2D.cpp:59
    let area = jacobian_determinant_abs * 0.5_f32;

    // PrincipalComponents2D.cpp:61
    let first_moment_of_area_xy: Vec2f = (
        jacobian_determinant_abs * (a.0 + b.0 + c.0) / 6.0_f32,
        jacobian_determinant_abs * (a.1 + b.1 + c.1) / 6.0_f32,
    );

    // PrincipalComponents2D.cpp:63
    (area, first_moment_of_area_xy, second_moment_of_area_xy, second_moment_of_area_covariance)
}

// returns two eigenvectors of the area covered by given polygons. The vectors are sorted by their corresponding eigenvalue, largest first
// PrincipalComponents2D.cpp:66-67
pub fn compute_principal_components(polys: &Polygons) -> (PointF, PointF) {
    // PrincipalComponents2D.cpp:69
    let mut centroid_accumulator: Vec2f = (0.0_f32, 0.0_f32);
    // PrincipalComponents2D.cpp:70
    let mut second_moment_of_area_accumulator: Vec2f = (0.0_f32, 0.0_f32);
    // PrincipalComponents2D.cpp:71
    let mut second_moment_of_area_covariance_accumulator: f32 = 0.0_f32;
    // PrincipalComponents2D.cpp:72
    let mut area: f32 = 0.0_f32;

    // PrincipalComponents2D.cpp:74
    for poly in polys {
        // PrincipalComponents2D.cpp:75
        // Vec2f p0 = unscaled(poly.first_point()).cast<float>();
        let fp = poly.first_point();
        let p0: Vec2f = (unscale(fp.x) as f32, unscale(fp.y) as f32);
        // PrincipalComponents2D.cpp:76
        for i in 2..poly.points().len() {
            // PrincipalComponents2D.cpp:77
            // Vec2f p1 = unscaled(poly.points[i - 1]).cast<float>();
            let pp1 = poly.points()[i - 1];
            let p1: Vec2f = (unscale(pp1.x) as f32, unscale(pp1.y) as f32);
            // PrincipalComponents2D.cpp:78
            // Vec2f p2 = unscaled(poly.points[i]).cast<float>();
            let pp2 = poly.points()[i];
            let p2: Vec2f = (unscale(pp2.x) as f32, unscale(pp2.y) as f32);

            // PrincipalComponents2D.cpp:80
            // float sign = cross2(p1 - p0, p2 - p1) > 0 ? 1.0f : -1.0f;
            let cross2 = (p1.0 - p0.0) * (p2.1 - p1.1) - (p1.1 - p0.1) * (p2.0 - p1.0);
            let sign: f32 = if cross2 > 0.0_f32 { 1.0_f32 } else { -1.0_f32 };

            // PrincipalComponents2D.cpp:82-83
            let (triangle_area, first_moment_of_area, second_moment_area, second_moment_of_area_covariance) =
                compute_moments_of_area_of_triangle(p0, p1, p2);
            // PrincipalComponents2D.cpp:84
            area += sign * triangle_area;
            // PrincipalComponents2D.cpp:85
            centroid_accumulator.0 += sign * first_moment_of_area.0;
            centroid_accumulator.1 += sign * first_moment_of_area.1;
            // PrincipalComponents2D.cpp:86
            second_moment_of_area_accumulator.0 += sign * second_moment_area.0;
            second_moment_of_area_accumulator.1 += sign * second_moment_area.1;
            // PrincipalComponents2D.cpp:87
            second_moment_of_area_covariance_accumulator += sign * second_moment_of_area_covariance;
        }
    }

    // PrincipalComponents2D.cpp:91-93
    if area <= 0.0_f32 {
        // PrincipalComponents2D.cpp:92
        return (PointF::new(0.0, 0.0), PointF::new(0.0, 0.0));
    }

    // PrincipalComponents2D.cpp:95
    // Vec2f centroid = centroid_accumulator / area;
    let centroid: Vec2f = (centroid_accumulator.0 / area, centroid_accumulator.1 / area);
    // PrincipalComponents2D.cpp:96
    // Vec2f variance = second_moment_of_area_accumulator / area - centroid.cwiseProduct(centroid);
    let variance: Vec2f = (
        second_moment_of_area_accumulator.0 / area - centroid.0 * centroid.0,
        second_moment_of_area_accumulator.1 / area - centroid.1 * centroid.1,
    );
    // PrincipalComponents2D.cpp:97
    // double covariance = second_moment_of_area_covariance_accumulator / area - centroid.x() * centroid.y();
    // C++ note: both `accumulator/area` and `centroid.x()*centroid.y()` are `float`,
    // so the subtraction is performed in `float` and only the resulting `float` is
    // widened to `double` on assignment. Mirror that: do the subtraction in `f32`
    // first, then widen — NOT a widen-then-subtract in `f64`.
    let covariance: f64 =
        (second_moment_of_area_covariance_accumulator / area - centroid.0 * centroid.1) as f64;
    // PrincipalComponents2D.cpp:98-103 (#if 0 debug prints omitted)

    // PrincipalComponents2D.cpp:104
    if covariance.abs() < EPSILON {
        // PrincipalComponents2D.cpp:105
        // std::tuple<Vec2f, Vec2f> result{Vec2f{variance.x(), 0.0}, Vec2f{0.0, variance.y()}};
        let result: (Vec2f, Vec2f) = ((variance.0, 0.0_f32), (0.0_f32, variance.1));
        // PrincipalComponents2D.cpp:106
        if variance.1 > variance.0 {
            // PrincipalComponents2D.cpp:107
            return (vec2f_to_pointf(result.1), vec2f_to_pointf(result.0));
        } else {
            // PrincipalComponents2D.cpp:109
            return (vec2f_to_pointf(result.0), vec2f_to_pointf(result.1));
        }
    }

    // now we find the first principal component of the covered area by computing max eigenvalue and the correspoding eigenvector of
    // covariance matrix
    //  covaraince matrix C is :  | VarX  Cov  |
    //                            | Cov   VarY |
    // Eigenvalues are solutions to det(C - lI) = 0, where l is the eigenvalue and I unit matrix
    // Eigenvector for eigenvalue l is any vector v such that Cv = lv

    // PrincipalComponents2D.cpp:119-120
    // C++ promotion: `covariance` is a `double`, so `4.0f * covariance * covariance`
    // is computed in `double`; `(varx-vary)*(varx-vary)` is a `float` then promoted
    // to `double` for the `+`; `sqrt(double)` is `double`; the whole `0.5f * (...)`
    // sum is evaluated in `double` and only truncated to `float` on assignment. The
    // inner `variance.x()-variance.y()` difference and its square stay in `f32`.
    let var_diff_sq: f32 = (variance.0 - variance.1) * (variance.0 - variance.1);
    let sqrt_arg: f64 = var_diff_sq as f64 + 4.0_f32 as f64 * covariance * covariance;
    let eigenvalue_a: f32 = (0.5_f32 as f64
        * (variance.0 as f64 + variance.1 as f64 + sqrt_arg.sqrt())) as f32;
    // PrincipalComponents2D.cpp:121-122
    let eigenvalue_b: f32 = (0.5_f32 as f64
        * (variance.0 as f64 + variance.1 as f64 - sqrt_arg.sqrt())) as f32;
    // PrincipalComponents2D.cpp:123
    // Vec2f eigenvector_a{(eigenvalue_a - variance.y()) / covariance, 1.0f};
    let eigenvector_a: Vec2f = (((eigenvalue_a - variance.1) as f64 / covariance) as f32, 1.0_f32);
    // PrincipalComponents2D.cpp:124
    // Vec2f eigenvector_b{(eigenvalue_b - variance.y()) / covariance, 1.0f};
    let eigenvector_b: Vec2f = (((eigenvalue_b - variance.1) as f64 / covariance) as f32, 1.0_f32);

    // PrincipalComponents2D.cpp:126-131 (#if 0 debug prints omitted)

    // PrincipalComponents2D.cpp:133-137
    if eigenvalue_a > eigenvalue_b {
        // PrincipalComponents2D.cpp:134
        (vec2f_to_pointf(eigenvector_a), vec2f_to_pointf(eigenvector_b))
    } else {
        // PrincipalComponents2D.cpp:136
        (vec2f_to_pointf(eigenvector_b), vec2f_to_pointf(eigenvector_a))
    }
}

/// Losslessly widen a `Vec2f` (the C++ `float` return) into the crate's `PointF`
/// (`f64`). The original `f32` value is exactly representable in `f64`.
#[inline]
fn vec2f_to_pointf(v: Vec2f) -> PointF {
    PointF::new(v.0 as f64, v.1 as f64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangle_moments_simple() {
        // Simple right triangle at origin
        let a: Vec2f = (0.0, 0.0);
        let b: Vec2f = (1.0, 0.0);
        let c: Vec2f = (0.0, 1.0);

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

        assert!(v1_is_x || v2_is_y, "Expected principal axes roughly aligned with X/Y");
    }
}
