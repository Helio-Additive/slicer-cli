//! Circle and arc geometry for arc fitting in G-code generation
//!
//! C++ Reference:
//! - Circle.hpp
//! - Circle.cpp
//!
//! This module provides geometric primitives for detecting and representing
//! circular arcs in toolpaths. Used by ArcFitter to convert linear segments
//! into G2/G3 arc moves where appropriate.

use crate::geometry::{Line, Point};
use crate::{Error, Result};
use nalgebra::Vector2;

/// Tolerance for floating-point comparisons
/// Circle.hpp:8
/// C++: constexpr double ZERO_TOLERANCE = 0.000005;
pub const ZERO_TOLERANCE: f64 = 0.000005;

/// Maximum arc radius (scaled coordinates)
/// Circle.hpp:66
/// C++: #define DEFAULT_SCALED_MAX_RADIUS scale_(2000)
pub const DEFAULT_SCALED_MAX_RADIUS: f64 = 2000.0 * 1_000_000.0; // 2000mm in scaled units

/// Arc resolution (scaled coordinates)
/// Circle.hpp:67
/// C++: #define DEFAULT_SCALED_RESOLUTION scale_(0.05)
pub const DEFAULT_SCALED_RESOLUTION: f64 = 0.05 * 1_000_000.0; // 0.05mm in scaled units

/// Arc length tolerance percentage
/// Circle.hpp:68
/// C++: #define DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE 0.05
pub const DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE: f64 = 0.05; // 5%

/// Direction of arc rotation
/// Circle.hpp:59-64
/// C++: enum class ArcDirection : unsigned char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArcDirection {
    // Unknown direction
    // Circle.hpp:60
    // C++: Arc_Dir_unknow,
    Unknown,

    // Counter-clockwise (G3)
    // Circle.hpp:61
    // C++: Arc_Dir_CCW,
    CounterClockwise,

    // Clockwise (G2)
    // Circle.hpp:62
    // C++: Arc_Dir_CW,
    Clockwise,
}

/// Circle defined by center point and radius
/// Circle.hpp:10-57
/// C++: class Circle
#[derive(Debug, Clone)]
pub struct Circle {
    // Center point of the circle
    // Circle.hpp:19
    // C++: Point center;
    pub center: Point,

    // Radius of the circle (scaled coordinates)
    // Circle.hpp:20
    // C++: double radius;
    pub radius: f64,
}

impl Circle {
    // Create a new circle with zero radius at origin
    // Circle.hpp:12-15
    // C++: Circle() { center = Point(0,0); radius = 0; }
    pub fn new() -> Self {
        Circle {
            center: Point::new(0, 0),
            radius: 0.0,
        }
    }

    // Create a new circle with given center and radius
    // Circle.hpp:16-19
    // C++: Circle(Point &p, double r) { center = p; radius = r; }
    pub fn with_center_radius(center: Point, radius: f64) -> Self {
        Circle { center, radius }
    }

    // Get the closest point on the circle to the input point
    // Circle.hpp:22-25
    // C++: Point get_closest_point(const Point& input)
    pub fn get_closest_point(&self, input: Point) -> Point {
        // Compute normalized vector from center to input
        // Circle.hpp:23
        // C++: Vec2d v = (input - center).cast<double>().normalized();
        let dx = (input.x - self.center.x) as f64;
        let dy = (input.y - self.center.y) as f64;
        let len = (dx * dx + dy * dy).sqrt();

        if len < ZERO_TOLERANCE {
            return self.center;
        }

        let v_x = dx / len;
        let v_y = dy / len;

        // Scale by radius and add to center
        // Circle.hpp:24
        // C++: return (center + (v * radius).cast<coord_t>());
        Point::new(
            self.center.x + (v_x * self.radius).round() as i64,
            self.center.y + (v_y * self.radius).round() as i64,
        )
    }

    // Attempt to create a circle from three points
    // Circle.hpp:27
    // C++: static bool try_create_circle(const Point &p1, const Point &p2, const Point &p3, const double max_radius, Circle& new_circle);
    pub fn try_create_circle_from_points(
        p1: Point,
        p2: Point,
        p3: Point,
        max_radius: f64,
    ) -> Option<Circle> {
        // Convert points to f64 for precise calculation
        let x1 = p1.x as f64;
        let y1 = p1.y as f64;
        let x2 = p2.x as f64;
        let y2 = p2.y as f64;
        let x3 = p3.x as f64;
        let y3 = p3.y as f64;

        // Calculate circle center using perpendicular bisectors
        let ma = (y2 - y1) / (x2 - x1);
        let mb = (y3 - y2) / (x3 - x2);

        if Self::is_equal(ma, mb, ZERO_TOLERANCE) {
            // Points are collinear
            return None;
        }

        let cx = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) / (2.0 * (mb - ma));
        let cy = -(1.0 / ma) * (cx - (x1 + x2) / 2.0) + (y1 + y2) / 2.0;

        let radius = ((x1 - cx).powi(2) + (y1 - cy).powi(2)).sqrt();

        if radius > max_radius || radius < ZERO_TOLERANCE {
            return None;
        }

        Some(Circle {
            center: Point::new(cx.round() as i64, cy.round() as i64),
            radius,
        })
    }

    // Get polar angle (radians) from center to point
    // Circle.hpp:28
    // C++: double get_polar_radians(const Point& p1) const;
    pub fn get_polar_radians(&self, p: Point) -> f64 {
        let dx = (p.x - self.center.x) as f64;
        let dy = (p.y - self.center.y) as f64;
        dy.atan2(dx)
    }

    // Check if points deviate from circle by more than tolerance
    // Circle.hpp:29
    // C++: bool is_over_deviation(const Points& points, const double tolerance);
    pub fn is_over_deviation(&self, points: &[Point], tolerance: f64) -> bool {
        let tolerance_squared = tolerance * tolerance;

        for point in points {
            let dx = (point.x - self.center.x) as f64;
            let dy = (point.y - self.center.y) as f64;
            let dist = (dx * dx + dy * dy).sqrt();
            let deviation = (dist - self.radius).abs();

            if deviation * deviation > tolerance_squared {
                return true;
            }
        }

        false
    }

    // Compare two floats with tolerance
    // Circle.hpp:41-44
    // C++: static bool is_equal(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn is_equal(x: f64, y: f64, tolerance: f64) -> bool {
        (x - y).abs() < tolerance
    }

    // Check if x > y with tolerance
    // Circle.hpp:45-47
    // C++: static bool greater_than(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn greater_than(x: f64, y: f64, tolerance: f64) -> bool {
        x > y && !Self::is_equal(x, y, tolerance)
    }

    // Check if x >= y with tolerance
    // Circle.hpp:48-50
    // C++: static bool greater_than_or_equal(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn greater_than_or_equal(x: f64, y: f64, tolerance: f64) -> bool {
        x > y || Self::is_equal(x, y, tolerance)
    }

    // Check if x < y with tolerance
    // Circle.hpp:51-53
    // C++: static bool less_than(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn less_than(x: f64, y: f64, tolerance: f64) -> bool {
        x < y && !Self::is_equal(x, y, tolerance)
    }

    // Check if x <= y with tolerance
    // Circle.hpp:54-56
    // C++: static bool less_than_or_equal(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn less_than_or_equal(x: f64, y: f64, tolerance: f64) -> bool {
        x < y || Self::is_equal(x, y, tolerance)
    }
}

impl Default for Circle {
    fn default() -> Self {
        Self::new()
    }
}

/// Arc segment representing a circular arc between two points
/// Circle.hpp:70-135
/// C++: class ArcSegment: public Circle
#[derive(Debug, Clone)]
pub struct ArcSegment {
    // Circle parameters (center, radius)
    // Circle.hpp:70
    // C++: class ArcSegment: public Circle
    pub circle: Circle,

    // Whether this is a valid arc
    // Circle.hpp:87
    // C++: bool is_arc = false;
    pub is_arc: bool,

    // Arc length (scaled coordinates)
    // Circle.hpp:88
    // C++: double length = 0;
    pub length: f64,

    // Angle in radians
    // Circle.hpp:89
    // C++: double angle_radians = 0;
    pub angle_radians: f64,

    // Polar angle of start point
    // Circle.hpp:90
    // C++: double polar_start_theta = 0;
    pub polar_start_theta: f64,

    // Polar angle of end point
    // Circle.hpp:91
    // C++: double polar_end_theta = 0;
    pub polar_end_theta: f64,

    // Start point of arc
    // Circle.hpp:92
    // C++: Point start_point { Point(0,0) };
    pub start_point: Point,

    // End point of arc
    // Circle.hpp:93
    // C++: Point end_point{ Point(0,0) };
    pub end_point: Point,

    // Arc direction (CW or CCW)
    // Circle.hpp:94
    // C++: ArcDirection direction = ArcDirection::Arc_Dir_unknow;
    pub direction: ArcDirection,
}

impl ArcSegment {
    // Create a new invalid arc segment
    // Circle.hpp:72
    // C++: ArcSegment(): Circle() {}
    pub fn new() -> Self {
        ArcSegment {
            circle: Circle::new(),
            is_arc: false,
            length: 0.0,
            angle_radians: 0.0,
            polar_start_theta: 0.0,
            polar_end_theta: 0.0,
            start_point: Point::new(0, 0),
            end_point: Point::new(0, 0),
            direction: ArcDirection::Unknown,
        }
    }

    // Create a new arc segment from circle parameters
    // Circle.hpp:73-85
    // C++: ArcSegment(Point center, double radius, Point start, Point end, ArcDirection dir)
    pub fn with_parameters(
        center: Point,
        radius: f64,
        start_point: Point,
        end_point: Point,
        direction: ArcDirection,
    ) -> Self {
        // Check for invalid configurations
        // Circle.hpp:77-81
        // C++: if (radius == 0.0 || start_point == center || end_point == center || start_point == end_point) { is_arc = false; return; }
        if Circle::is_equal(radius, 0.0, ZERO_TOLERANCE)
            || start_point == center
            || end_point == center
            || start_point == end_point
        {
            return Self::new();
        }

        let mut arc = ArcSegment {
            circle: Circle::with_center_radius(center, radius),
            is_arc: true,
            length: 0.0,
            angle_radians: 0.0,
            polar_start_theta: 0.0,
            polar_end_theta: 0.0,
            start_point,
            end_point,
            direction,
        };

        // Update angle and length
        // Circle.hpp:82
        // C++: update_angle_and_length();
        arc.update_angle_and_length();

        arc
    }

    // Update angle and length based on current parameters
    // Circle.hpp:99
    // C++: void update_angle_and_length();
    fn update_angle_and_length(&mut self) {
        // Calculate polar angles for start and end points
        self.polar_start_theta = self.circle.get_polar_radians(self.start_point);
        self.polar_end_theta = self.circle.get_polar_radians(self.end_point);

        // Calculate angle swept by the arc
        let mut angle = self.polar_end_theta - self.polar_start_theta;

        // Normalize angle based on direction
        match self.direction {
            ArcDirection::CounterClockwise => {
                if angle < 0.0 {
                    angle += 2.0 * std::f64::consts::PI;
                }
            }
            ArcDirection::Clockwise => {
                if angle > 0.0 {
                    angle -= 2.0 * std::f64::consts::PI;
                }
                angle = angle.abs();
            }
            ArcDirection::Unknown => {}
        }

        self.angle_radians = angle.abs();

        // Calculate arc length: length = radius * angle
        self.length = self.circle.radius * self.angle_radians;
    }

    // Check if this is a valid arc
    // Circle.hpp:96
    // C++: bool is_valid() const { return is_arc; }
    pub fn is_valid(&self) -> bool {
        self.is_arc
    }

    // Reverse the arc direction
    // Circle.hpp:99
    // C++: bool reverse();
    pub fn reverse(&mut self) -> bool {
        if !self.is_arc {
            return false;
        }

        // Swap start and end points
        std::mem::swap(&mut self.start_point, &mut self.end_point);
        std::mem::swap(&mut self.polar_start_theta, &mut self.polar_end_theta);

        // Reverse direction
        self.direction = match self.direction {
            ArcDirection::Clockwise => ArcDirection::CounterClockwise,
            ArcDirection::CounterClockwise => ArcDirection::Clockwise,
            ArcDirection::Unknown => ArcDirection::Unknown,
        };

        true
    }

    // Attempt to create an arc from a sequence of points
    // Circle.hpp:105-110
    // C++: static bool try_create_arc(const Points &points, ArcSegment& target_arc, ...)
    pub fn try_create_arc(
        points: &[Point],
        approximate_length: f64,
        max_radius: f64,
        tolerance: f64,
        path_tolerance_percent: f64,
    ) -> Option<ArcSegment> {
        // Need at least 3 points to define an arc
        if points.len() < 3 {
            return None;
        }

        // Use first, middle, and last points to define circle
        let start_point = points[0];
        let mid_point = points[points.len() / 2];
        let end_point = points[points.len() - 1];

        // Try to create a circle from these three points
        let circle =
            Circle::try_create_circle_from_points(start_point, mid_point, end_point, max_radius)?;

        // Check if all points lie close to the circle
        if circle.is_over_deviation(points, tolerance) {
            return None;
        }

        // Determine arc direction by checking cross product
        let v1_x = (mid_point.x - start_point.x) as f64;
        let v1_y = (mid_point.y - start_point.y) as f64;
        let v2_x = (end_point.x - mid_point.x) as f64;
        let v2_y = (end_point.y - mid_point.y) as f64;

        let cross = v1_x * v2_y - v1_y * v2_x;
        let direction = if cross > 0.0 {
            ArcDirection::CounterClockwise
        } else {
            ArcDirection::Clockwise
        };

        // Create the arc segment
        let mut arc = ArcSegment::with_parameters(
            circle.center,
            circle.radius,
            start_point,
            end_point,
            direction,
        );

        // Check arc length is close to approximate length
        if approximate_length > ZERO_TOLERANCE {
            let length_diff = (arc.length - approximate_length).abs();
            let length_percent = length_diff / approximate_length;

            if length_percent > path_tolerance_percent {
                return None;
            }
        }

        Some(arc)
    }
}

impl Default for ArcSegment {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circle_new() {
        let circle = Circle::new();
        assert_eq!(circle.center, Point::new(0, 0));
        assert_eq!(circle.radius, 0.0);
    }

    #[test]
    fn test_circle_with_center_radius() {
        let center = Point::new(100, 200);
        let circle = Circle::with_center_radius(center, 50.0);
        assert_eq!(circle.center, center);
        assert_eq!(circle.radius, 50.0);
    }

    #[test]
    fn test_circle_is_equal() {
        assert!(Circle::is_equal(1.0, 1.000001, 0.0001));
        assert!(!Circle::is_equal(1.0, 1.1, 0.0001));
    }

    #[test]
    fn test_circle_comparisons() {
        assert!(Circle::greater_than(2.0, 1.0, ZERO_TOLERANCE));
        assert!(!Circle::greater_than(1.0, 1.0, ZERO_TOLERANCE));

        assert!(Circle::greater_than_or_equal(2.0, 1.0, ZERO_TOLERANCE));
        assert!(Circle::greater_than_or_equal(1.0, 1.0, ZERO_TOLERANCE));

        assert!(Circle::less_than(1.0, 2.0, ZERO_TOLERANCE));
        assert!(!Circle::less_than(1.0, 1.0, ZERO_TOLERANCE));

        assert!(Circle::less_than_or_equal(1.0, 2.0, ZERO_TOLERANCE));
        assert!(Circle::less_than_or_equal(1.0, 1.0, ZERO_TOLERANCE));
    }

    #[test]
    fn test_arc_segment_new() {
        let arc = ArcSegment::new();
        assert!(!arc.is_valid());
        assert_eq!(arc.length, 0.0);
        assert_eq!(arc.direction, ArcDirection::Unknown);
    }

    #[test]
    fn test_arc_segment_with_parameters() {
        let center = Point::new(0, 0);
        let radius = 100.0;
        let start = Point::new(100, 0);
        let end = Point::new(0, 100);
        let direction = ArcDirection::CounterClockwise;

        let arc = ArcSegment::with_parameters(center, radius, start, end, direction);

        assert!(arc.is_valid());
        assert_eq!(arc.start_point, start);
        assert_eq!(arc.end_point, end);
        assert_eq!(arc.direction, direction);
        assert!(arc.length > 0.0);
    }

    #[test]
    fn test_arc_segment_invalid_configurations() {
        // Zero radius
        let arc = ArcSegment::with_parameters(
            Point::new(0, 0),
            0.0,
            Point::new(100, 0),
            Point::new(0, 100),
            ArcDirection::CounterClockwise,
        );
        assert!(!arc.is_valid());

        // Start point equals center
        let arc = ArcSegment::with_parameters(
            Point::new(0, 0),
            100.0,
            Point::new(0, 0),
            Point::new(0, 100),
            ArcDirection::CounterClockwise,
        );
        assert!(!arc.is_valid());

        // Start point equals end point
        let arc = ArcSegment::with_parameters(
            Point::new(0, 0),
            100.0,
            Point::new(100, 0),
            Point::new(100, 0),
            ArcDirection::CounterClockwise,
        );
        assert!(!arc.is_valid());
    }

    #[test]
    fn test_arc_segment_reverse() {
        let center = Point::new(0, 0);
        let radius = 100.0;
        let start = Point::new(100, 0);
        let end = Point::new(0, 100);

        let mut arc =
            ArcSegment::with_parameters(center, radius, start, end, ArcDirection::CounterClockwise);

        let original_length = arc.length;

        assert!(arc.reverse());
        assert_eq!(arc.start_point, end);
        assert_eq!(arc.end_point, start);
        assert_eq!(arc.direction, ArcDirection::Clockwise);

        // Length should remain the same
        assert!((arc.length - original_length).abs() < 1.0);
    }

    #[test]
    fn test_arc_direction_values() {
        assert_eq!(ArcDirection::Unknown, ArcDirection::Unknown);
        assert_ne!(ArcDirection::Clockwise, ArcDirection::CounterClockwise);
    }
}
