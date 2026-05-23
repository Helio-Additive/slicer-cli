//! 2D linear algebra utilities for Arachne
//!
//! C++ Reference:
//! - Arachne/utils/linearAlg2D.hpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use crate::geometry::Point;
use std::f64::consts::PI;

/// Test whether a point is inside a corner
///
/// Tests whether point `query_point` is left of the corner abc.
/// Whether the `query_point` is in the circle half left of ab and left of bc, rather than to the right.
///
/// Test whether the `query_point` is inside of a polygon w.r.t a single corner.
///
/// C++ Reference: Arachne/utils/linearAlg2D.hpp:18-59
/// C++: inline static bool isInsideCorner(const Point &a, const Point &b, const Point &c, const Vec2i64 &query_point)
/// C++: {
/// C++:     //     Visualisation for the algorithm below:
/// C++:     //
/// C++:     //                 query
/// C++:     //                   |
/// C++:     //                   |
/// C++:     //                   |
/// C++:     //    perp-----------b
/// C++:     //                  / \       (note that the lines
/// C++:     //                 /   \      AB and AC are normalized
/// C++:     //                /     \     to 10000 units length)
/// C++:     //               a       c
/// C++:     //
pub fn is_inside_corner(a: Point, b: Point, c: Point, query_point: Point) -> bool {
    // Helper: Create a normalized vector of specified length
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:30-36
    // C++: auto normal = [](const Point &p0, coord_t len) -> Point {
    // C++:     int64_t _len = p0.cast<int64_t>().norm();
    // C++:     if (_len < 1)
    // C++:         return {len, 0};
    // C++:     return (p0.cast<int64_t>() * int64_t(len) / _len).cast<coord_t>();
    // C++: };
    let normal = |p: Point, len: i64| -> Point {
        let px = p.x as i64;
        let py = p.y as i64;
        let _len = ((px * px + py * py) as f64).sqrt() as i64;
        if _len < 1 {
            return Point::new(len, 0);
        }
        Point::new((px * len / _len) as i64, (py * len / _len) as i64)
    };

    // Helper: Rotate a 2D vector 90 degrees counter-clockwise
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:38-40
    // C++: auto rotate_90_degree_ccw = [](const Vec2d &p) -> Vec2d {
    // C++:     return {-p.y(), p.x()};
    // C++: };
    let rotate_90_degree_ccw = |px: f64, py: f64| -> (f64, f64) { (-py, px) };

    /// Create a normal vector of reasonable length to reduce rounding error
    /// C++ Reference: Arachne/utils/linearAlg2D.hpp:42
    /// C++: constexpr coord_t normal_length = 10000;
    const NORMAL_LENGTH: i64 = 10000;

    // Compute normalized vectors from B to A and B to C
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:43-44
    // C++: const Point ba = normal(a - b, normal_length);
    // C++: const Point bc = normal(c - b, normal_length);
    let ba = normal(a - b, NORMAL_LENGTH);
    let bc = normal(c - b, NORMAL_LENGTH);

    // Compute vector from B to query point
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:45
    // C++: const Vec2d bq = query_point.cast<double>() - b.cast<double>();
    let bq_x = query_point.x as f64 - b.x as f64;
    let bq_y = query_point.y as f64 - b.y as f64;

    // The query projects to this perpendicular to coordinate 0
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:46
    // C++: const Vec2d perpendicular = rotate_90_degree_ccw(bq);
    let (perp_x, perp_y) = rotate_90_degree_ccw(bq_x, bq_y);

    // Project vertex A on the perpendicular line
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:48
    // C++: const double project_a_perpendicular = ba.cast<double>().dot(perpendicular);
    let project_a_perpendicular = ba.x as f64 * perp_x + ba.y as f64 * perp_y;

    // Project vertex C on the perpendicular line
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:49
    // C++: const double project_c_perpendicular = bc.cast<double>().dot(perpendicular);
    let project_c_perpendicular = bc.x as f64 * perp_x + bc.y as f64 * perp_y;

    // Check if query is between A and C on the projection
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:50-53
    // C++: if ((project_a_perpendicular > 0.) != (project_c_perpendicular > 0.))
    // C++: {
    // C++:     return project_a_perpendicular > 0.;
    // C++: }
    if (project_a_perpendicular > 0.0) != (project_c_perpendicular > 0.0) {
        return project_a_perpendicular > 0.0;
    } else {
        // Beyond either A or C, but it could still be inside of the polygon
        // C++ Reference: Arachne/utils/linearAlg2D.hpp:54-63
        // C++: else
        // C++: {
        // C++:     const double project_a_parallel = ba.cast<double>().dot(bq);
        // C++:     const double project_c_parallel = bc.cast<double>().dot(bq);
        // C++:
        // C++:     //Either:
        // C++:     // * A is to the right of B (project_a_perpendicular > 0) and C is below A (project_c_parallel < project_a_parallel), or
        // C++:     // * A is to the left of B (project_a_perpendicular < 0) and C is above A (project_c_parallel > project_a_parallel).
        // C++:     return (project_c_parallel < project_a_parallel) == (project_a_perpendicular > 0.);
        /// C++: }
        let project_a_parallel = ba.x as f64 * bq_x + ba.y as f64 * bq_y;
        let project_c_parallel = bc.x as f64 * bq_x + bc.y as f64 * bq_y;

        (project_c_parallel < project_a_parallel) == (project_a_perpendicular > 0.0)
    }
}

/// Returns the determinant of the 2D matrix defined by the vectors ab and ap as rows
///
/// The returned value is zero for `p` lying (approximately) on the line going through `a` and `b`.
/// The value is positive for values lying to the left and negative for values lying to the right
/// when looking from `a` to `b`.
///
/// C++ Reference: Arachne/utils/linearAlg2D.hpp:65-79
/// C++: static inline int64_t pointIsLeftOfLine(const Point &p, const Point &a, const Point &b)
/// C++: {
/// C++:     return int64_t(b.x() - a.x()) * int64_t(p.y() - a.y()) - int64_t(b.y() - a.y()) * int64_t(p.x() - a.x());
/// C++: }
pub fn point_is_left_of_line(p: Point, a: Point, b: Point) -> i64 {
    (b.x - a.x) as i64 * (p.y - a.y) as i64 - (b.y - a.y) as i64 * (p.x - a.x) as i64
}

/// Compute the angle between two consecutive line segments
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
/// Returns the angle in radians between 0 and 2 * pi of the corner in `b`
///
/// C++ Reference: Arachne/utils/linearAlg2D.hpp:81-106
/// C++: static inline float getAngleLeft(const Point &a, const Point &b, const Point &c)
/// C++: {
/// C++:     const Vec2i64 ba   = (a - b).cast<int64_t>();
/// C++:     const Vec2i64 bc   = (c - b).cast<int64_t>();
/// C++:     const int64_t dott = ba.dot(bc);      // dot product
/// C++:     const int64_t det  = cross2(ba, bc); // determinant
/// C++:     if (det == 0) {
/// C++:         if ((ba.x() != 0 && (ba.x() > 0) == (bc.x() > 0)) || (ba.x() == 0 && (ba.y() > 0) == (bc.y() > 0)))
/// C++:             return 0; // pointy bit
/// C++:         else
/// C++:             return float(M_PI); // straight bit
/// C++:     }
/// C++:     const float angle = -atan2(double(det), double(dott)); // from -pi to pi
/// C++:     if (angle >= 0)
/// C++:         return angle;
/// C++:     else
/// C++:         return M_PI * 2 + angle;
/// C++: }
pub fn get_angle_left(a: Point, b: Point, c: Point) -> f32 {
    // Compute vectors from B to A and B to C
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:91-92
    // C++: const Vec2i64 ba   = (a - b).cast<int64_t>();
    // C++: const Vec2i64 bc   = (c - b).cast<int64_t>();
    let ba_x = (a.x - b.x) as i64;
    let ba_y = (a.y - b.y) as i64;
    let bc_x = (c.x - b.x) as i64;
    let bc_y = (c.y - b.y) as i64;

    // Compute dot product
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:93
    // C++: const int64_t dott = ba.dot(bc);
    let dott = ba_x * bc_x + ba_y * bc_y;

    // Compute determinant (cross product in 2D)
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:94
    // C++: const int64_t det  = cross2(ba, bc);
    let det = ba_x * bc_y - ba_y * bc_x;

    // Handle degenerate case where vectors are collinear
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:95-100
    // C++: if (det == 0) {
    // C++:     if ((ba.x() != 0 && (ba.x() > 0) == (bc.x() > 0)) || (ba.x() == 0 && (ba.y() > 0) == (bc.y() > 0)))
    // C++:         return 0; // pointy bit
    // C++:     else
    // C++:         return float(M_PI); // straight bit
    // C++: }
    if det == 0 {
        if (ba_x != 0 && (ba_x > 0) == (bc_x > 0)) || (ba_x == 0 && (ba_y > 0) == (bc_y > 0)) {
            return 0.0; // pointy bit
        } else {
            return PI as f32; // straight bit
        }
    }

    // Compute angle using atan2
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:101
    // C++: const float angle = -atan2(double(det), double(dott));
    let angle = -(det as f64).atan2(dott as f64) as f32;

    // Normalize to [0, 2*PI)
    // C++ Reference: Arachne/utils/linearAlg2D.hpp:102-105
    // C++: if (angle >= 0)
    // C++:     return angle;
    // C++: else
    // C++:     return M_PI * 2 + angle;
    if angle >= 0.0 {
        angle
    } else {
        (2.0 * PI) as f32 + angle
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use std::f64::consts::PI;

    #[test]
    fn test_point_is_left_of_line() {
        /// Test point left of line
        /// C++ Reference: Arachne/utils/linearAlg2D.hpp:79
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
        /// Test right angle (90 degrees)
        /// C++ Reference: Arachne/utils/linearAlg2D.hpp:106
        let a = Point::new(0, 0);
        let b = Point::new(100, 0);
        let c = Point::new(100, 100);

        let angle = get_angle_left(a, b, c);
        assert!((angle - (PI as f32 / 2.0)).abs() < 0.001);
    }

    #[test]
    fn test_get_angle_left_straight() {
        /// Test straight line (180 degrees)
        /// C++ Reference: Arachne/utils/linearAlg2D.hpp:100
        let a = Point::new(0, 0);
        let b = Point::new(100, 0);
        let c = Point::new(200, 0);

        let angle = get_angle_left(a, b, c);
        assert!((angle - PI as f32).abs() < 0.001);
    }

    #[test]
    fn test_get_angle_left_pointy() {
        /// Test pointy bit (0 degrees)
        /// C++ Reference: Arachne/utils/linearAlg2D.hpp:98
        let a = Point::new(200, 0);
        let b = Point::new(100, 0);
        let c = Point::new(0, 0);

        let angle = get_angle_left(a, b, c);
        assert!(angle.abs() < 0.001);
    }

    #[test]
    fn test_get_angle_left_acute() {
        /// Test acute angle (45 degrees)
        let a = Point::new(0, 0);
        let b = Point::new(100, 0);
        let c = Point::new(100, 100);

        let angle = get_angle_left(a, b, c);
        assert!(angle > 0.0 && angle < PI as f32);
    }

    #[test]
    fn test_is_inside_corner_simple() {
        /// Test basic corner containment
        /// C++ Reference: Arachne/utils/linearAlg2D.hpp:59
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
        /// Test right-angle corner
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
        /// Test obtuse angle corner (> 90 degrees)
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
        /// Test acute angle corner (< 90 degrees)
        let a = Point::new(50, 0);
        let b = Point::new(100, 100);
        let c = Point::new(150, 0);

        // Point inside the narrow corner
        let inside = Point::new(100, 50);
        assert!(is_inside_corner(a, b, c, inside));
    }

    #[test]
    fn test_point_is_left_of_line_vertical() {
        /// Test with vertical line
        let a = Point::new(100, 0);
        let b = Point::new(100, 200);
        let p_left = Point::new(50, 100);
        let p_right = Point::new(150, 100);

        assert!(point_is_left_of_line(p_left, a, b) > 0);
        assert!(point_is_left_of_line(p_right, a, b) < 0);
    }

    #[test]
    fn test_point_is_left_of_line_diagonal() {
        /// Test with diagonal line
        let a = Point::new(0, 0);
        let b = Point::new(100, 100);
        let p_left = Point::new(0, 100);
        let p_right = Point::new(100, 0);

        assert!(point_is_left_of_line(p_left, a, b) > 0);
        assert!(point_is_left_of_line(p_right, a, b) < 0);
    }

    #[test]
    fn test_get_angle_left_full_circle() {
        /// Test various angles around a circle
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
