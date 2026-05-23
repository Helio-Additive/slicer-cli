//! Linear algebra utility functions for 2D geometry in the Arachne module.
//!
//! C++ Reference:
//! - Arachne/utils/linearAlg2D.hpp
//!
//! Ported from CuraEngine's LinearAlg2D namespace.

use crate::geometry::Point;

/// Test whether a point is inside a corner.
/// Whether `query_point` is left of the corner abc.
/// Whether the `query_point` is in the circle half left of ab and left of bc.
///
/// Arachne/utils/linearAlg2D.hpp: isInsideCorner
pub fn is_inside_corner(a: &Point, b: &Point, c: &Point, query_point: &Point) -> bool {
    let normal_length: i64 = 10000;

    let normal = |p0_x: i64, p0_y: i64, len: i64| -> (i64, i64) {
        let norm = ((p0_x as f64).powi(2) + (p0_y as f64).powi(2)).sqrt() as i64;
        if norm < 1 {
            return (len, 0);
        }
        (p0_x * len / norm, p0_y * len / norm)
    };

    let ba = normal(
        a.x as i64 - b.x as i64,
        a.y as i64 - b.y as i64,
        normal_length,
    );
    let bc = normal(
        c.x as i64 - b.x as i64,
        c.y as i64 - b.y as i64,
        normal_length,
    );
    let bq_x = query_point.x as f64 - b.x as f64;
    let bq_y = query_point.y as f64 - b.y as f64;

    // Perpendicular = rotate 90 degrees CCW
    let perp_x = -bq_y;
    let perp_y = bq_x;

    let project_a_perpendicular = ba.0 as f64 * perp_x + ba.1 as f64 * perp_y;
    let project_c_perpendicular = bc.0 as f64 * perp_x + bc.1 as f64 * perp_y;

    if (project_a_perpendicular > 0.0) != (project_c_perpendicular > 0.0) {
        // Query is between A and C on the projection
        return project_a_perpendicular > 0.0;
    }

    let project_a_parallel = ba.0 as f64 * bq_x + ba.1 as f64 * bq_y;
    let project_c_parallel = bc.0 as f64 * bq_x + bc.1 as f64 * bq_y;

    (project_c_parallel < project_a_parallel) == (project_a_perpendicular > 0.0)
}

/// Returns the determinant of the 2D matrix defined by the vectors ab and ap as rows.
///
/// The returned value is zero for `p` lying (approximately) on the line going through `a` and `b`.
/// The value is positive for values lying to the left and negative for values lying to the right
/// when looking from `a` to `b`.
///
/// Arachne/utils/linearAlg2D.hpp: pointIsLeftOfLine
pub fn point_is_left_of_line(p: &Point, a: &Point, b: &Point) -> i64 {
    (b.x as i64 - a.x as i64) * (p.y as i64 - a.y as i64)
        - (b.y as i64 - a.y as i64) * (p.x as i64 - a.x as i64)
}

/// Compute the angle between two consecutive line segments.
///
/// The angle is computed from the left side of b when looking from a.
/// Returns the angle in radians between 0 and 2 * pi of the corner in `b`.
///
/// Arachne/utils/linearAlg2D.hpp: getAngleLeft
pub fn get_angle_left(a: &Point, b: &Point, c: &Point) -> f32 {
    let ba_x = a.x as i64 - b.x as i64;
    let ba_y = a.y as i64 - b.y as i64;
    let bc_x = c.x as i64 - b.x as i64;
    let bc_y = c.y as i64 - b.y as i64;

    let dott = ba_x * bc_x + ba_y * bc_y;
    let det = ba_x * bc_y - ba_y * bc_x; // cross2

    if det == 0 {
        if (ba_x != 0 && (ba_x > 0) == (bc_x > 0)) || (ba_x == 0 && (ba_y > 0) == (bc_y > 0)) {
            return 0.0; // pointy bit
        } else {
            return std::f32::consts::PI; // straight bit
        }
    }

    let angle = -(det as f64).atan2(dott as f64) as f32; // from -pi to pi
    if angle >= 0.0 {
        angle
    } else {
        std::f32::consts::PI * 2.0 + angle
    }
}
