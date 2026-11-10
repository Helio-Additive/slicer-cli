//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! 2D linear algebra utilities for Arachne
//!
//! C++ Reference: Arachne/utils/linearAlg2D.hpp
//! namespace Slic3r::Arachne::LinearAlg2D

// linearAlg2D.hpp:7  #include "../../Point.hpp"
use crate::geometry::Point;
use std::f64::consts::PI;

/// Test whether a point is inside a corner.
/// Whether point `query_point` is left of the corner abc.
/// Whether the `query_point` is in the circle half left of ab and left of bc, rather than to the right.
///
/// Test whether the `query_point` is inside of a polygon w.r.t a single corner.
///
/// linearAlg2D.hpp:19  inline static bool isInsideCorner(const Point &a, const Point &b, const Point &c, const Vec2i64 &query_point)
pub fn is_inside_corner(a: Point, b: Point, c: Point, query_point: Point) -> bool {
    //     Visualisation for the algorithm below:
    //
    //                 query
    //                   |
    //                   |
    //                   |
    //    perp-----------b
    //                  / \       (note that the lines
    //                 /   \      AB and AC are normalized
    //                /     \     to 10000 units length)
    //               a       c
    //

    // linearAlg2D.hpp:34  auto normal = [](const Point &p0, coord_t len) -> Point {
    // linearAlg2D.hpp:35      int64_t _len = p0.cast<int64_t>().norm();
    // linearAlg2D.hpp:36      if (_len < 1)
    // linearAlg2D.hpp:37          return {len, 0};
    // linearAlg2D.hpp:38      return (p0.cast<int64_t>() * int64_t(len) / _len).cast<coord_t>();
    // linearAlg2D.hpp:39  };
    let normal = |p: Point, len: i64| -> Point {
        let px = p.x as i64;
        let py = p.y as i64;
        let _len = ((px * px + py * py) as f64).sqrt() as i64;
        if _len < 1 {
            return Point::new(len, 0);
        }
        // FIDELITY-NOTE(F2): C++ `(p0.cast<int64_t>()*len/_len).cast<coord_t>()`
        // truncates each component to coord_t (int32) on the final cast; crate
        // Coord=i64 keeps the full int64 result. Equal for in-range coordinates.
        Point::new(px * len / _len, py * len / _len)
    };

    // linearAlg2D.hpp:41  auto rotate_90_degree_ccw = [](const Vec2d &p) -> Vec2d {
    // linearAlg2D.hpp:42      return {-p.y(), p.x()};
    // linearAlg2D.hpp:43  };
    let rotate_90_degree_ccw = |px: f64, py: f64| -> (f64, f64) { (-py, px) };

    // linearAlg2D.hpp:45  constexpr coord_t normal_length = 10000; //Create a normal vector of reasonable length in order to reduce rounding error.
    const NORMAL_LENGTH: i64 = 10000;
    // linearAlg2D.hpp:46  const Point ba = normal(a - b, normal_length);
    let ba = normal(a - b, NORMAL_LENGTH);
    // linearAlg2D.hpp:47  const Point bc = normal(c - b, normal_length);
    let bc = normal(c - b, NORMAL_LENGTH);
    // linearAlg2D.hpp:48  const Vec2d bq = query_point.cast<double>() - b.cast<double>();
    let bq_x = query_point.x as f64 - b.x as f64;
    let bq_y = query_point.y as f64 - b.y as f64;
    // linearAlg2D.hpp:49  const Vec2d perpendicular = rotate_90_degree_ccw(bq); //The query projects to this perpendicular to coordinate 0.
    let (perp_x, perp_y) = rotate_90_degree_ccw(bq_x, bq_y);

    // linearAlg2D.hpp:51  const double project_a_perpendicular = ba.cast<double>().dot(perpendicular); //Project vertex A on the perpendicular line.
    let project_a_perpendicular = ba.x as f64 * perp_x + ba.y as f64 * perp_y;
    // linearAlg2D.hpp:52  const double project_c_perpendicular = bc.cast<double>().dot(perpendicular); //Project vertex C on the perpendicular line.
    let project_c_perpendicular = bc.x as f64 * perp_x + bc.y as f64 * perp_y;
    // linearAlg2D.hpp:53  if ((project_a_perpendicular > 0.) != (project_c_perpendicular > 0.)) //Query is between A and C on the projection.
    if (project_a_perpendicular > 0.0) != (project_c_perpendicular > 0.0) {
        // linearAlg2D.hpp:55  return project_a_perpendicular > 0.; //Due to the winding order of corner ABC, this means that the query is inside.
        project_a_perpendicular > 0.0
    } else {
        // linearAlg2D.hpp:57  else //Beyond either A or C, but it could still be inside of the polygon.
        // linearAlg2D.hpp:59  const double project_a_parallel = ba.cast<double>().dot(bq); //Project not on the perpendicular, but on the original.
        let project_a_parallel = ba.x as f64 * bq_x + ba.y as f64 * bq_y;
        // linearAlg2D.hpp:60  const double project_c_parallel = bc.cast<double>().dot(bq);
        let project_c_parallel = bc.x as f64 * bq_x + bc.y as f64 * bq_y;

        //Either:
        // * A is to the right of B (project_a_perpendicular > 0) and C is below A (project_c_parallel < project_a_parallel), or
        // * A is to the left of B (project_a_perpendicular < 0) and C is above A (project_c_parallel > project_a_parallel).
        // linearAlg2D.hpp:65  return (project_c_parallel < project_a_parallel) == (project_a_perpendicular > 0.);
        (project_c_parallel < project_a_parallel) == (project_a_perpendicular > 0.0)
    }
}

/// Returns the determinant of the 2D matrix defined by the the vectors ab and ap as rows.
///
/// The returned value is zero for `p` lying (approximately) on the line going through `a` and `b`.
/// The value is positive for values lying to the left and negative for values lying to the right
/// when looking from `a` to `b`.
///
/// * `p` the point to check
/// * `a` the from point of the line
/// * `b` the to point of the line
///
/// Returns a positive value when `p` lies to the left of the line from `a` to `b`.
///
/// linearAlg2D.hpp:80  static inline int64_t pointIsLeftOfLine(const Point &p, const Point &a, const Point &b)
pub fn point_is_left_of_line(p: Point, a: Point, b: Point) -> i64 {
    // linearAlg2D.hpp:82  return int64_t(b.x() - a.x()) * int64_t(p.y() - a.y()) - int64_t(b.y() - a.y()) * int64_t(p.x() - a.x());
    (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x())
}

/// Compute the angle between two consecutive line segments.
///
/// The angle is computed from the left side of b when looking from a.
///
/// ```text
///   c
///    \                     .
///     \ b
/// angle|
///      |
///      a
/// ```
///
/// * `a` start of first line segment
/// * `b` end of first segment and start of second line segment
/// * `c` end of second line segment
///
/// Returns the angle in radians between 0 and 2 * pi of the corner in `b`.
///
/// linearAlg2D.hpp:102  static inline float getAngleLeft(const Point &a, const Point &b, const Point &c)
pub fn get_angle_left(a: Point, b: Point, c: Point) -> f32 {
    // linearAlg2D.hpp:104  const Vec2i64 ba   = (a - b).cast<int64_t>();
    let ba_x = (a.x - b.x) as i64;
    let ba_y = (a.y - b.y) as i64;
    // linearAlg2D.hpp:105  const Vec2i64 bc   = (c - b).cast<int64_t>();
    let bc_x = (c.x - b.x) as i64;
    let bc_y = (c.y - b.y) as i64;
    // linearAlg2D.hpp:106  const int64_t dott = ba.dot(bc);      // dot product
    let dott = ba_x * bc_x + ba_y * bc_y;
    // linearAlg2D.hpp:107  const int64_t det  = cross2(ba, bc); // determinant
    let det = ba_x * bc_y - ba_y * bc_x;
    // linearAlg2D.hpp:108  if (det == 0) {
    if det == 0 {
        // linearAlg2D.hpp:109  if ((ba.x() != 0 && (ba.x() > 0) == (bc.x() > 0)) || (ba.x() == 0 && (ba.y() > 0) == (bc.y() > 0)))
        if (ba_x != 0 && (ba_x > 0) == (bc_x > 0)) || (ba_x == 0 && (ba_y > 0) == (bc_y > 0)) {
            // linearAlg2D.hpp:110  return 0; // pointy bit
            return 0.0; // pointy bit
        } else {
            // linearAlg2D.hpp:112  return float(M_PI); // straight bit
            return PI as f32; // straight bit
        }
    }
    // linearAlg2D.hpp:114  const float angle = -atan2(double(det), double(dott)); // from -pi to pi
    let angle = -(det as f64).atan2(dott as f64) as f32;
    // linearAlg2D.hpp:115  if (angle >= 0)
    if angle >= 0.0 {
        // linearAlg2D.hpp:116  return angle;
        angle
    } else {
        // linearAlg2D.hpp:118  return M_PI * 2 + angle;
        // C++ `float(M_PI*2 + angle)`: the sum is evaluated in double (angle, a
        // float, promotes) and narrowed to float on return. Compute in f64 then
        // narrow — NOT `(2pi as f32) + (angle as f32)` which would differ in the
        // last bit.
        (PI * 2.0 + angle as f64) as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use std::f64::consts::PI;

    #[test]
    fn test_point_is_left_of_line() {
        // Test point left of line
        // linearAlg2D.hpp:80
        let a = Point::new(0, 0);
        let b = Point::new(100, 0);
        let p_left = Point::new(50, 50);
        let p_right = Point::new(50, -50);
        let p_on = Point::new(50, 0);

        assert!(point_is_left_of_line(p_left, a, b) > 0);
        assert!(point_is_left_of_line(p_right, a, b) < 0);
        assert_eq!(point_is_left_of_line(p_on, a, b), 0);
    }

    #[test]
    fn test_get_angle_left_right_angle() {
        // Test right angle (90 degrees)
        let a = Point::new(0, 0);
        let b = Point::new(100, 0);
        let c = Point::new(100, 100);

        let angle = get_angle_left(a, b, c);
        assert!((angle - (PI as f32 / 2.0)).abs() < 0.001);
    }

    #[test]
    fn test_get_angle_left_straight() {
        // Test straight line (180 degrees)
        let a = Point::new(0, 0);
        let b = Point::new(100, 0);
        let c = Point::new(200, 0);

        let angle = get_angle_left(a, b, c);
        assert!((angle - PI as f32).abs() < 0.001);
    }

    #[test]
    fn test_get_angle_left_pointy() {
        // Test pointy bit (0 degrees)
        let a = Point::new(200, 0);
        let b = Point::new(100, 0);
        let c = Point::new(0, 0);

        let angle = get_angle_left(a, b, c);
        assert!(angle.abs() < 0.001);
    }

    #[test]
    fn test_get_angle_left_acute() {
        // Test acute angle (45 degrees)
        let a = Point::new(0, 0);
        let b = Point::new(100, 0);
        let c = Point::new(100, 100);

        let angle = get_angle_left(a, b, c);
        assert!(angle > 0.0 && angle < PI as f32);
    }

    #[test]
    fn test_is_inside_corner_simple() {
        // Test basic corner containment
        let a = Point::new(0, 0);
        let b = Point::new(100, 100);
        let c = Point::new(200, 0);

        // Point inside the corner
        let inside = Point::new(100, 50);
        assert!(is_inside_corner(a, b, c, inside));

        // Point outside the corner
        let outside = Point::new(100, 150);
        assert!(!is_inside_corner(a, b, c, outside));
    }

    #[test]
    fn test_is_inside_corner_right_angle() {
        // Test right-angle corner
        let a = Point::new(0, 100);
        let b = Point::new(0, 0);
        let c = Point::new(100, 0);

        // Point inside the right angle
        let inside = Point::new(50, 50);
        assert!(is_inside_corner(a, b, c, inside));

        // Point outside on the left
        let outside_left = Point::new(-50, 50);
        assert!(!is_inside_corner(a, b, c, outside_left));

        // Point outside on the right
        let outside_right = Point::new(50, -50);
        assert!(!is_inside_corner(a, b, c, outside_right));
    }

    #[test]
    fn test_is_inside_corner_obtuse() {
        // Test obtuse angle corner (> 90 degrees)
        let a = Point::new(0, 50);
        let b = Point::new(100, 100);
        let c = Point::new(200, 50);

        // Point inside
        let inside = Point::new(100, 80);
        assert!(is_inside_corner(a, b, c, inside));

        // Point way outside
        let outside = Point::new(100, 0);
        assert!(!is_inside_corner(a, b, c, outside));
    }

    #[test]
    fn test_is_inside_corner_acute() {
        // Test acute angle corner (< 90 degrees)
        let a = Point::new(50, 0);
        let b = Point::new(100, 100);
        let c = Point::new(150, 0);

        // Point inside the narrow corner
        let inside = Point::new(100, 50);
        assert!(is_inside_corner(a, b, c, inside));
    }

    #[test]
    fn test_point_is_left_of_line_vertical() {
        // Test with vertical line
        let a = Point::new(100, 0);
        let b = Point::new(100, 200);
        let p_left = Point::new(50, 100);
        let p_right = Point::new(150, 100);

        assert!(point_is_left_of_line(p_left, a, b) > 0);
        assert!(point_is_left_of_line(p_right, a, b) < 0);
    }

    #[test]
    fn test_point_is_left_of_line_diagonal() {
        // Test with diagonal line
        let a = Point::new(0, 0);
        let b = Point::new(100, 100);
        let p_left = Point::new(0, 100);
        let p_right = Point::new(100, 0);

        assert!(point_is_left_of_line(p_left, a, b) > 0);
        assert!(point_is_left_of_line(p_right, a, b) < 0);
    }

    #[test]
    fn test_get_angle_left_full_circle() {
        // Test various angles around a circle
        let b = Point::new(100, 100);
        let a = Point::new(100, 0);

        // 45 degrees
        let c1 = Point::new(170, 70);
        let angle1 = get_angle_left(a, b, c1);
        assert!(angle1 > 0.0 && angle1 < PI as f32 / 2.0);

        // 90 degrees
        let c2 = Point::new(200, 100);
        let angle2 = get_angle_left(a, b, c2);
        assert!((angle2 - PI as f32 / 2.0).abs() < 0.1);

        // 270 degrees
        let c3 = Point::new(0, 100);
        let angle3 = get_angle_left(a, b, c3);
        assert!((angle3 - 3.0 * PI as f32 / 2.0).abs() < 0.1);
    }
}
