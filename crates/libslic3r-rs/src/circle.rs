//! Circle and arc geometry for arc fitting in G-code generation
//!
//! C++ Reference:
//! - Circle.hpp
//! - Circle.cpp
//!
//! This module provides geometric primitives for detecting and representing
//! circular arcs in toolpaths. Used by ArcFitter to convert linear segments
//! into G2/G3 arc moves where appropriate.
//!
//! Faithful 1:1 port of BambuStudio's Circle.cpp / Circle.hpp. Same fn names
//! (snake_case), same order, same control flow, same constants and edge cases.

use crate::geometry::{Line, Point};
use crate::libslic3r::{EPSILON, SCALED_EPSILON};
use nalgebra::Vector2;

use std::f64::consts::PI;

/// Tolerance for floating-point comparisons
/// Circle.hpp:9
/// C++: constexpr double ZERO_TOLERANCE = 0.000005;
pub const ZERO_TOLERANCE: f64 = 0.000005;

/// BBS: threshold used to judge collineation
/// Circle.cpp:13
/// C++: static const double Parallel_area_threshold = 0.0001;
const PARALLEL_AREA_THRESHOLD: f64 = 0.0001;

/// Maximum arc radius (scaled coordinates)
/// Circle.hpp:64
/// C++: #define DEFAULT_SCALED_MAX_RADIUS scale_(2000)        // 2000mm
/// scale_(2000) = 2000 / SCALING_FACTOR = 2000 * 100000 = 2e8
pub const DEFAULT_SCALED_MAX_RADIUS: f64 = 200_000_000.0; // scale_(2000)

/// Arc resolution (scaled coordinates)
/// Circle.hpp:65
/// C++: #define DEFAULT_SCALED_RESOLUTION scale_(0.05)        // 0.05mm
/// scale_(0.05) = 0.05 * 100000 = 5000
pub const DEFAULT_SCALED_RESOLUTION: f64 = 5_000.0; // scale_(0.05)

/// Arc length tolerance percentage
/// Circle.hpp:66
/// C++: #define DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE  0.05    // 5 percent
pub const DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE: f64 = 0.05; // 5%

/// Direction of arc rotation
/// Circle.hpp:57-62
/// C++: enum class ArcDirection : unsigned char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArcDirection {
    // Unknown direction
    // Circle.hpp:58
    // C++: Arc_Dir_unknow,
    Unknown,

    // Counter-clockwise (G3)
    // Circle.hpp:59
    // C++: Arc_Dir_CCW,
    CounterClockwise,

    // Clockwise (G2)
    // Circle.hpp:60
    // C++: Arc_Dir_CW,
    Clockwise,
}

/// Circle defined by center point and radius
/// Circle.hpp:11-55
/// C++: class Circle
#[derive(Debug, Clone)]
pub struct Circle {
    // Center point of the circle
    // Circle.hpp:21
    // C++: Point center;
    pub center: Point,

    // Radius of the circle (scaled coordinates)
    // Circle.hpp:22
    // C++: double radius;
    pub radius: f64,
}

impl Circle {
    // Create a new circle with zero radius at origin
    // Circle.hpp:13-16
    // C++: Circle() { center = Point(0,0); radius = 0; }
    pub fn new() -> Self {
        Circle {
            center: Point::new(0, 0),
            radius: 0.0,
        }
    }

    // Create a new circle with given center and radius
    // Circle.hpp:17-20
    // C++: Circle(Point &p, double r) { center = p; radius = r; }
    pub fn with_center_radius(center: Point, radius: f64) -> Self {
        Circle { center, radius }
    }

    // Get the closest point on the circle to the input point
    // Circle.hpp:24-27
    // C++: Point get_closest_point(const Point& input) {
    // C++:     Vec2d v = (input - center).cast<double>().normalized();
    // C++:     return (center + (v * radius).cast<coord_t>());
    // C++: }
    pub fn get_closest_point(&self, input: Point) -> Point {
        let dx = (input.x() - self.center.x()) as f64;
        let dy = (input.y() - self.center.y()) as f64;
        // Eigen normalized(): divide by L2 norm.
        let len = (dx * dx + dy * dy).sqrt();
        let v_x = dx / len;
        let v_y = dy / len;
        // (v * radius).cast<coord_t>() truncates toward zero (C++ static_cast<int>).
        Point::new(
            self.center.x() + (v_x * self.radius) as i64,
            self.center.y() + (v_y * self.radius) as i64,
        )
    }

    // Attempt to create a circle from three points.
    // Circle.cpp:15-55
    // C++: bool Circle::try_create_circle(const Point& p1, const Point& p2, const Point& p3, const double max_radius, Circle& new_circle)
    pub fn try_create_circle_from_points(
        p1: Point,
        p2: Point,
        p3: Point,
        max_radius: f64,
    ) -> Option<Circle> {
        // Circle.cpp:17-22
        let x1 = p1.x() as f64;
        let y1 = p1.y() as f64;
        let x2 = p2.x() as f64;
        let y2 = p2.y() as f64;
        let x3 = p3.x() as f64;
        let y3 = p3.y() as f64;

        // BBS: use area of triangle to judge whether three points are almostly on one line
        // Because the point is scale_ once, so area should scale_ twice.
        // Circle.cpp:24-27
        // C++: if (fabs((y1 - y2) * (x1 - x3) - (y1 - y3) * (x1 - x2)) <= scale_(scale_(Parallel_area_threshold)))
        if ((y1 - y2) * (x1 - x3) - (y1 - y3) * (x1 - x2)).abs()
            <= scale_f(scale_f(PARALLEL_AREA_THRESHOLD))
        {
            return None;
        }

        // Circle.cpp:29
        let a = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;
        // BBS: take out to figure out how we handle very small values
        // Circle.cpp:30-32
        if a.abs() < SCALED_EPSILON {
            return None;
        }

        // Circle.cpp:34-36
        let b = (x1 * x1 + y1 * y1) * (y3 - y2)
            + (x2 * x2 + y2 * y2) * (y1 - y3)
            + (x3 * x3 + y3 * y3) * (y2 - y1);

        // Circle.cpp:38-40
        let c = (x1 * x1 + y1 * y1) * (x2 - x3)
            + (x2 * x2 + y2 * y2) * (x3 - x1)
            + (x3 * x3 + y3 * y3) * (x1 - x2);

        // Circle.cpp:42-43
        let center_x = -b / (2.0 * a);
        let center_y = -c / (2.0 * a);

        // Circle.cpp:45-49
        let delta_x = center_x - x1;
        let delta_y = center_y - y1;
        let radius = (delta_x * delta_x + delta_y * delta_y).sqrt();
        if radius > max_radius {
            return None;
        }

        // Circle.cpp:51-52
        // C++: new_circle.center = Point(center_x, center_y);
        // Point(double, double) truncates toward zero (C++ implicit conversion).
        Some(Circle {
            center: Point::new(center_x as i64, center_y as i64),
            radius,
        })
    }

    // Attempt to create the best-fitting circle from a list of points.
    // Circle.cpp:57-95
    // C++: bool Circle::try_create_circle(const Points& points, const double max_radius, const double tolerance, Circle& new_circle)
    pub fn try_create_circle_from_point_list(
        points: &[Point],
        max_radius: f64,
        tolerance: f64,
    ) -> Option<Circle> {
        // Circle.cpp:59-60
        let count = points.len();
        let middle_index = count / 2;
        // BBS: the middle point will almost always produce the best arcs with high possibility.
        // Circle.cpp:62-71
        if count == 3 {
            if let Some(new_circle) = Circle::try_create_circle_from_points(
                points[0],
                points[middle_index],
                points[count - 1],
                max_radius,
            ) {
                if !new_circle.is_over_deviation(points, tolerance) {
                    return Some(new_circle);
                }
            }
            // C++ returns the && of the two conditions: if either failed we fall through.
        } else {
            // Circle.cpp:66-67
            let middle_point = if count % 2 == 0 {
                (points[middle_index] + points[middle_index - 1]) / 2
            } else {
                (points[middle_index - 1] + points[middle_index + 1]) / 2
            };
            // Circle.cpp:68-70
            if let Some(new_circle) = Circle::try_create_circle_from_points(
                points[0],
                middle_point,
                points[count - 1],
                max_radius,
            ) {
                if !new_circle.is_over_deviation(points, tolerance) {
                    return Some(new_circle);
                }
            }
        }

        // BBS: Find the circle with the least deviation, if one exists.
        // Circle.cpp:73-94
        let mut new_circle: Option<Circle> = None;
        let mut least_deviation = 0.0_f64;
        let mut found_circle = false;
        for index in 1..count - 1 {
            // Circle.cpp:80-82
            if index == middle_index {
                // BBS: We already checked this one, and it failed. don't need to do again
                continue;
            }

            // Circle.cpp:84
            if let Some(test_circle) = Circle::try_create_circle_from_points(
                points[0],
                points[index],
                points[count - 1],
                max_radius,
            ) {
                if let Some(current_deviation) =
                    test_circle.get_deviation_sum_squared(points, tolerance)
                {
                    // Circle.cpp:86-91
                    if !found_circle || current_deviation < least_deviation {
                        found_circle = true;
                        least_deviation = current_deviation;
                        new_circle = Some(test_circle);
                    }
                }
            }
        }
        if found_circle {
            new_circle
        } else {
            None
        }
    }

    // Get polar angle (radians) from center to point
    // Circle.cpp:97-103
    // C++: double Circle::get_polar_radians(const Point& p1) const
    pub fn get_polar_radians(&self, p: Point) -> f64 {
        // Circle.cpp:99
        let mut polar_radians =
            ((p.y() - self.center.y()) as f64).atan2((p.x() - self.center.x()) as f64);
        // Circle.cpp:100-101
        if polar_radians < 0.0 {
            polar_radians = (2.0 * PI) + polar_radians;
        }
        polar_radians
    }

    // Check if points deviate from circle by more than tolerance
    // Circle.cpp:105-131
    // C++: bool Circle::is_over_deviation(const Points& points, const double tolerance)
    pub fn is_over_deviation(&self, points: &[Point], tolerance: f64) -> bool {
        // BBS: skip the first and last points since they has fit perfectly.
        // Circle.cpp:111
        for index in 0..points.len() - 1 {
            // Circle.cpp:113-120
            if index != 0 {
                // BBS: check fitting tolerance
                let temp = points[index] - self.center;
                let distance_from_center =
                    ((temp.x() as f64) * (temp.x() as f64) + (temp.y() as f64) * (temp.y() as f64))
                        .sqrt();
                if (distance_from_center - self.radius).abs() > tolerance {
                    return true;
                }
            }

            // BBS: Check the point perpendicular from the segment to the circle's center
            // Circle.cpp:122-128
            if let Some(closest_point) =
                Circle::get_closest_perpendicular_point(points[index], points[index + 1], self.center)
            {
                let temp = closest_point - self.center;
                let distance_from_center =
                    ((temp.x() as f64) * (temp.x() as f64) + (temp.y() as f64) * (temp.y() as f64))
                        .sqrt();
                if (distance_from_center - self.radius).abs() > tolerance {
                    return true;
                }
            }
        }
        false
    }

    // BBS: [(Cx - Ax)(Bx - Ax) + (Cy - Ay)(By - Ay)] / [(Bx - Ax) ^ 2 + (By - Ay) ^ 2]
    // Circle.cpp:133-153
    // C++: bool Circle::get_closest_perpendicular_point(const Point& p1, const Point& p2, const Point& c, Point& out)
    pub fn get_closest_perpendicular_point(p1: Point, p2: Point, c: Point) -> Option<Point> {
        // Circle.cpp:135-140
        let x1 = p1.x() as f64;
        let y1 = p1.y() as f64;
        let x2 = p2.x() as f64;
        let y2 = p2.y() as f64;
        let x_dif = x2 - x1;
        let y_dif = y2 - y1;
        // Circle.cpp:142-144
        let num = (c.x() as f64 - x1) * x_dif + (c.y() as f64 - y1) * y_dif;
        let denom = (x_dif * x_dif) + (y_dif * y_dif);
        let t = num / denom;

        // BBS: Considering this a failure if t == 0 or t==1 within tolerance. In that case we hit the endpoint, which is OK.
        // Circle.cpp:147-148
        if Circle::less_than_or_equal(t, 0.0, ZERO_TOLERANCE)
            || Circle::greater_than_or_equal(t, 1.0, ZERO_TOLERANCE)
        {
            return None;
        }

        // Circle.cpp:150-151
        // out[0]/out[1] are coord_t; double assigned to coord_t truncates toward zero.
        Some(Point::new(
            (x1 + t * (x2 - x1)) as i64,
            (y1 + t * (y2 - y1)) as i64,
        ))
    }

    // Compute the sum of squared deviations of points from the circle.
    // Returns None (false) if any deviation exceeds tolerance.
    // Circle.cpp:155-187
    // C++: bool Circle::get_deviation_sum_squared(const Points& points, const double tolerance, double& total_deviation)
    pub fn get_deviation_sum_squared(&self, points: &[Point], tolerance: f64) -> Option<f64> {
        // Circle.cpp:157
        let mut total_deviation = 0.0_f64;
        // BBS: skip the first and last points since they are on the circle
        // Circle.cpp:161-172
        for index in 1..points.len() - 1 {
            // BBS: make sure the length from the center of our circle to the test point is
            // at or below our max distance.
            let temp = points[index] - self.center;
            let distance_from_center =
                ((temp.x() as f64) * (temp.x() as f64) + (temp.y() as f64) * (temp.y() as f64))
                    .sqrt();
            let deviation = (distance_from_center - self.radius).abs();
            total_deviation += deviation * deviation;
            if deviation > tolerance {
                return None;
            }
        }
        // BBS: check the point perpendicular from the segment to the circle's center
        // Circle.cpp:175-185
        for index in 0..points.len() - 1 {
            if let Some(closest_point) =
                Circle::get_closest_perpendicular_point(points[index], points[index + 1], self.center)
            {
                let temp = closest_point - self.center;
                let distance_from_center =
                    ((temp.x() as f64) * (temp.x() as f64) + (temp.y() as f64) * (temp.y() as f64))
                        .sqrt();
                let deviation = (distance_from_center - self.radius).abs();
                total_deviation += deviation * deviation;
                if deviation > tolerance {
                    return None;
                }
            }
        }
        Some(total_deviation)
    }

    // BBS: only support calculate on X-Y plane, Z is useless
    // Circle.cpp:189-201
    // C++: Vec3f Circle::calc_tangential_vector(const Vec3f& pos, const Vec3f& center_pos, const bool is_ccw)
    pub fn calc_tangential_vector(
        pos: nalgebra::Vector3<f32>,
        center_pos: nalgebra::Vector3<f32>,
        is_ccw: bool,
    ) -> nalgebra::Vector3<f32> {
        // Circle.cpp:192-194
        let mut dir = center_pos - pos;
        dir[2] = 0.0;
        dir.normalize_mut();
        // Circle.cpp:195-199
        if is_ccw {
            nalgebra::Vector3::new(dir[1], -dir[0], 0.0)
        } else {
            nalgebra::Vector3::new(-dir[1], dir[0], 0.0)
        }
    }

    // Compare two floats with tolerance
    // Circle.hpp:38-41
    // C++: static bool is_equal(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn is_equal(x: f64, y: f64, tolerance: f64) -> bool {
        // Circle.hpp:39-40
        let abs_difference = (x - y).abs();
        abs_difference < tolerance
    }

    // Check if x > y with tolerance
    // Circle.hpp:42-44
    // C++: static bool greater_than(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn greater_than(x: f64, y: f64, tolerance: f64) -> bool {
        x > y && !Self::is_equal(x, y, tolerance)
    }

    // Check if x >= y with tolerance
    // Circle.hpp:45-47
    // C++: static bool greater_than_or_equal(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn greater_than_or_equal(x: f64, y: f64, tolerance: f64) -> bool {
        x > y || Self::is_equal(x, y, tolerance)
    }

    // Check if x < y with tolerance
    // Circle.hpp:48-50
    // C++: static bool less_than(double x, double y, double tolerance = ZERO_TOLERANCE)
    pub fn less_than(x: f64, y: f64, tolerance: f64) -> bool {
        x < y && !Self::is_equal(x, y, tolerance)
    }

    // Check if x <= y with tolerance
    // Circle.hpp:51-53
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

/// C++ macro `scale_(val) = val / SCALING_FACTOR`, with SCALING_FACTOR = 0.00001.
/// libslic3r.h:81
#[inline]
fn scale_f(v: f64) -> f64 {
    v / crate::libslic3r::SCALING_FACTOR
}

/// Arc segment representing a circular arc between two points
/// Circle.hpp:68-131
/// C++: class ArcSegment: public Circle
#[derive(Debug, Clone)]
pub struct ArcSegment {
    // Circle parameters (center, radius)
    // Circle.hpp:68
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

    // Angle in radians (signed: negative for CW)
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
    // Circle.hpp:70
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
    // Circle.hpp:71-85
    // C++: ArcSegment(Point center, double radius, Point start, Point end, ArcDirection dir)
    pub fn with_parameters(
        center: Point,
        radius: f64,
        start_point: Point,
        end_point: Point,
        direction: ArcDirection,
    ) -> Self {
        let mut arc = ArcSegment {
            circle: Circle::with_center_radius(center, radius),
            is_arc: false,
            length: 0.0,
            angle_radians: 0.0,
            polar_start_theta: 0.0,
            polar_end_theta: 0.0,
            start_point,
            end_point,
            direction,
        };

        // BBS: invalid configurations -> is_arc = false; return;
        // Circle.hpp:76-82
        // C++: if (radius == 0.0 || start_point == center || end_point == center || start_point == end_point)
        if radius == 0.0
            || start_point == center
            || end_point == center
            || start_point == end_point
        {
            arc.is_arc = false;
            return arc;
        }

        // Circle.hpp:83-84
        arc.update_angle_and_length();
        arc.is_arc = true;
        arc
    }

    // Update angle and length based on current parameters
    // Circle.cpp:256-267
    // C++: void ArcSegment::update_angle_and_length()
    fn update_angle_and_length(&mut self) {
        // Circle.cpp:258-260
        self.polar_start_theta = self.circle.get_polar_radians(self.start_point);
        self.polar_end_theta = self.circle.get_polar_radians(self.end_point);
        self.angle_radians = self.polar_end_theta - self.polar_start_theta;
        // Circle.cpp:261-264
        if self.angle_radians < 0.0 && self.direction == ArcDirection::CounterClockwise {
            self.angle_radians += 2.0 * PI;
        } else if self.angle_radians > 0.0 && self.direction == ArcDirection::Clockwise {
            self.angle_radians -= 2.0 * PI;
        }
        // Circle.cpp:265-266
        self.length = self.angle_radians.abs() * self.circle.radius;
        self.is_arc = true;
    }

    // Check if this is a valid arc
    // Circle.hpp:96
    // C++: bool is_valid() const { return is_arc; }
    pub fn is_valid(&self) -> bool {
        self.is_arc
    }

    // Reverse the arc direction
    // Circle.cpp:203-212
    // C++: bool ArcSegment::reverse()
    pub fn reverse(&mut self) -> bool {
        // Circle.cpp:205-206
        if !self.is_valid() {
            return false;
        }
        // Circle.cpp:207
        std::mem::swap(&mut self.start_point, &mut self.end_point);
        // Circle.cpp:208
        self.direction = if self.direction == ArcDirection::CounterClockwise {
            ArcDirection::Clockwise
        } else {
            ArcDirection::CounterClockwise
        };
        // Circle.cpp:209
        self.angle_radians *= -1.0;
        // Circle.cpp:210
        std::mem::swap(&mut self.polar_start_theta, &mut self.polar_end_theta);
        true
    }

    // Clip the start of the arc at the given point.
    // Circle.cpp:214-221
    // C++: bool ArcSegment::clip_start(const Point &point)
    pub fn clip_start(&mut self, point: Point) -> bool {
        // Circle.cpp:216-217
        if !self.is_valid() || point == self.circle.center || !self.is_point_inside(point) {
            return false;
        }
        // Circle.cpp:218
        self.start_point = self.circle.get_closest_point(point);
        // Circle.cpp:219
        self.update_angle_and_length();
        true
    }

    // Clip the end of the arc at the given point.
    // Circle.cpp:223-230
    // C++: bool ArcSegment::clip_end(const Point &point)
    pub fn clip_end(&mut self, point: Point) -> bool {
        // Circle.cpp:225-226
        if !self.is_valid() || point == self.circle.center || !self.is_point_inside(point) {
            return false;
        }
        // Circle.cpp:227
        self.end_point = self.circle.get_closest_point(point);
        // Circle.cpp:228
        self.update_angle_and_length();
        true
    }

    // Split the arc at the given point into two arcs.
    // Circle.cpp:232-240
    // C++: bool ArcSegment::split_at(const Point &point, ArcSegment& p1, ArcSegment& p2)
    pub fn split_at(&self, point: Point) -> Option<(ArcSegment, ArcSegment)> {
        // Circle.cpp:234-235
        if !self.is_valid() || point == self.circle.center || !self.is_point_inside(point) {
            return None;
        }
        // Circle.cpp:236
        let segment_point = self.circle.get_closest_point(point);
        // Circle.cpp:237-238
        let p1 = ArcSegment::with_parameters(
            self.circle.center,
            self.circle.radius,
            self.start_point,
            segment_point,
            self.direction,
        );
        let p2 = ArcSegment::with_parameters(
            self.circle.center,
            self.circle.radius,
            segment_point,
            self.end_point,
            self.direction,
        );
        Some((p1, p2))
    }

    // Check if a point lies within the arc's angular span.
    // Circle.cpp:242-254
    // C++: bool ArcSegment::is_point_inside(const Point& point) const
    pub fn is_point_inside(&self, point: Point) -> bool {
        // Circle.cpp:244
        let polar_theta = self.circle.get_polar_radians(point);
        // Circle.cpp:245
        let mut radian_delta = polar_theta - self.polar_start_theta;
        // Circle.cpp:246-247
        if radian_delta > 0.0 && self.direction == ArcDirection::Clockwise {
            radian_delta -= 2.0 * PI;
        // Circle.cpp:248-249
        } else if radian_delta < 0.0 && self.direction == ArcDirection::CounterClockwise {
            radian_delta += 2.0 * PI;
        }

        // Circle.cpp:251-253
        if self.direction == ArcDirection::CounterClockwise {
            radian_delta > 0.0 && radian_delta < self.angle_radians
        } else {
            radian_delta < 0.0 && radian_delta > self.angle_radians
        }
    }

    // Attempt to create an arc from a sequence of points.
    // Circle.cpp:269-292
    // C++: bool ArcSegment::try_create_arc(const Points& points, ArcSegment& target_arc, ...)
    pub fn try_create_arc(
        points: &[Point],
        approximate_length: f64,
        max_radius: f64,
        tolerance: f64,
        path_tolerance_percent: f64,
    ) -> Option<ArcSegment> {
        // Circle.cpp:277-279
        // C++: Circle test_circle = (Circle)target_arc;
        //      if (!Circle::try_create_circle(points, max_radius, tolerance, test_circle)) return false;
        let test_circle =
            Circle::try_create_circle_from_point_list(points, max_radius, tolerance)?;

        // Circle.cpp:281
        // C++: int mid_point_index = ((points.size() - 2) / 2) + 1;
        let mid_point_index = ((points.len() - 2) / 2) + 1;
        // Circle.cpp:282-284
        let test_arc = ArcSegment::try_create_arc_from_circle(
            &test_circle,
            points[0],
            points[mid_point_index],
            points[points.len() - 1],
            approximate_length,
            path_tolerance_percent,
        )?;

        // Circle.cpp:286-290
        if ArcSegment::are_points_within_slice(&test_arc, points) {
            Some(test_arc)
        } else {
            None
        }
    }

    // Attempt to create an arc on a given circle through start/mid/end points.
    // Circle.cpp:294-375
    // C++: bool ArcSegment::try_create_arc(const Circle& c, const Point& start_point,
    //          const Point& mid_point, const Point& end_point, ArcSegment& target_arc,
    //          double approximate_length, double path_tolerance_percent)
    fn try_create_arc_from_circle(
        c: &Circle,
        start_point: Point,
        mid_point: Point,
        end_point: Point,
        approximate_length: f64,
        path_tolerance_percent: f64,
    ) -> Option<ArcSegment> {
        // Circle.cpp:303-305
        let polar_start_theta = c.get_polar_radians(start_point);
        let polar_mid_theta = c.get_polar_radians(mid_point);
        let polar_end_theta = c.get_polar_radians(end_point);

        // Circle.cpp:307-308
        let mut angle_radians = 0.0_f64;
        let mut direction = ArcDirection::Unknown;
        // BBS: calculate the direction of the arc
        // Circle.cpp:310-328
        if polar_end_theta > polar_start_theta {
            if polar_start_theta < polar_mid_theta && polar_mid_theta < polar_end_theta {
                direction = ArcDirection::CounterClockwise;
                angle_radians = polar_end_theta - polar_start_theta;
            } else if (0.0 <= polar_mid_theta && polar_mid_theta < polar_start_theta)
                || (polar_end_theta < polar_mid_theta && polar_mid_theta < (2.0 * PI))
            {
                direction = ArcDirection::Clockwise;
                angle_radians = polar_start_theta + ((2.0 * PI) - polar_end_theta);
            }
        } else if polar_start_theta > polar_end_theta {
            if (polar_start_theta < polar_mid_theta && polar_mid_theta < (2.0 * PI))
                || (0.0 < polar_mid_theta && polar_mid_theta < polar_end_theta)
            {
                direction = ArcDirection::CounterClockwise;
                angle_radians = polar_end_theta + ((2.0 * PI) - polar_start_theta);
            } else if polar_end_theta < polar_mid_theta && polar_mid_theta < polar_start_theta {
                direction = ArcDirection::Clockwise;
                angle_radians = polar_start_theta - polar_end_theta;
            }
        }

        // BBS: this doesn't always work.. in rare situations, the angle may be backward
        // Circle.cpp:331-332
        if direction == ArcDirection::Unknown || angle_radians.abs() < EPSILON {
            return None;
        }

        // BBS: Check the length against the original length.
        // Circle.cpp:338-339
        let mut arc_length = c.radius * angle_radians;
        let mut difference = (arc_length - approximate_length) / approximate_length;
        // Circle.cpp:340-358
        if difference.abs() >= path_tolerance_percent {
            // BBS: So it's possible that vector calculation above got wrong direction.
            // BBS: Find the rest of the angle across the circle
            let test_radians = (angle_radians - 2.0 * PI).abs();
            // Calculate the length of that arc
            let test_arc_length = c.radius * test_radians;
            difference = (test_arc_length - approximate_length) / approximate_length;
            if difference.abs() >= path_tolerance_percent {
                return None;
            }
            // BBS: Set the new length and flip the direction (but not the angle)!
            arc_length = test_arc_length;
            direction = if direction == ArcDirection::CounterClockwise {
                ArcDirection::Clockwise
            } else {
                ArcDirection::CounterClockwise
            };
        }

        // Circle.cpp:360-361
        if direction == ArcDirection::Clockwise {
            angle_radians *= -1.0;
        }

        // Circle.cpp:363-372
        let target_arc = ArcSegment {
            circle: Circle::with_center_radius(c.center, c.radius),
            is_arc: true,
            direction,
            start_point,
            end_point,
            length: arc_length,
            angle_radians,
            polar_start_theta,
            polar_end_theta,
        };

        Some(target_arc)
    }

    // BBS: Check all the points and see if they fit inside of the angles
    // Circle.cpp:377-456
    // C++: bool ArcSegment::are_points_within_slice(const ArcSegment& test_arc, const Points& points)
    fn are_points_within_slice(test_arc: &ArcSegment, points: &[Point]) -> bool {
        // Circle.cpp:380-383
        let mut previous_polar = test_arc.polar_start_theta;
        let will_cross_zero;
        let mut crossed_zero = false;
        let point_count = points.len() as i64;

        // Circle.cpp:385-388
        let start_norm = Vector2::new(
            (test_arc.start_point.x() as f64 - test_arc.center().x() as f64) / test_arc.radius(),
            (test_arc.start_point.y() as f64 - test_arc.center().y() as f64) / test_arc.radius(),
        );
        let end_norm = Vector2::new(
            (test_arc.end_point.x() as f64 - test_arc.center().x() as f64) / test_arc.radius(),
            (test_arc.end_point.y() as f64 - test_arc.center().y() as f64) / test_arc.radius(),
        );

        // Circle.cpp:390-393
        if test_arc.direction == ArcDirection::CounterClockwise {
            will_cross_zero = test_arc.polar_start_theta > test_arc.polar_end_theta;
        } else {
            will_cross_zero = test_arc.polar_start_theta < test_arc.polar_end_theta;
        }

        // BBS: check if point 1 to point 2 cross zero
        // Circle.cpp:396-451
        let mut polar_test;
        let mut index = point_count - 2;
        while index < point_count {
            // Circle.cpp:399-402
            if index < point_count - 1 {
                polar_test = test_arc.get_polar_radians(points[index as usize]);
            } else {
                polar_test = test_arc.polar_end_theta;
            }

            // BBS: First ensure the test point is within the arc
            // Circle.cpp:405-443
            if test_arc.direction == ArcDirection::CounterClockwise {
                // BBS: Only check to see if we are within the arc if this isn't the endpoint
                if index < point_count - 1 {
                    if will_cross_zero {
                        if !(polar_test > test_arc.polar_start_theta
                            || polar_test < test_arc.polar_end_theta)
                        {
                            return false;
                        }
                    } else if !(test_arc.polar_start_theta < polar_test
                        && polar_test < test_arc.polar_end_theta)
                    {
                        return false;
                    }
                }
                // BBS: check the angles are increasing
                if previous_polar > polar_test {
                    if !will_cross_zero {
                        return false;
                    }
                    // BBS: Allow the angle to cross zero once
                    if crossed_zero {
                        return false;
                    }
                    crossed_zero = true;
                }
            } else {
                if index < point_count - 1 {
                    if will_cross_zero {
                        if !(polar_test < test_arc.polar_start_theta
                            || polar_test > test_arc.polar_end_theta)
                        {
                            return false;
                        }
                    } else if !(test_arc.polar_start_theta > polar_test
                        && polar_test > test_arc.polar_end_theta)
                    {
                        return false;
                    }
                }
                // BBS: Now make sure the angles are decreasing
                if previous_polar < polar_test {
                    if !will_cross_zero {
                        return false;
                    }
                    // BBS: Allow the angle to cross zero once
                    if crossed_zero {
                        return false;
                    }
                    crossed_zero = true;
                }
            }

            // BBS: check if the segment intersects either of the vector from the center of the
            // circle to the endpoints of the arc
            // Circle.cpp:446-449
            let segment = Line::new(points[(index - 1) as usize], points[index as usize]);
            if (index != 1
                && ArcSegment::ray_intersects_segment(test_arc.center(), start_norm, &segment))
                || (index != point_count - 1
                    && ArcSegment::ray_intersects_segment(test_arc.center(), end_norm, &segment))
            {
                return false;
            }
            previous_polar = polar_test;
            index += 1;
        }
        // BBS: Ensure that all arcs that cross zero
        // Circle.cpp:453-454
        if will_cross_zero != crossed_zero {
            return false;
        }
        true
    }

    // BBS: this function is used to detect whether a ray cross the segment
    // Circle.cpp:459-476
    // C++: bool ArcSegment::ray_intersects_segment(const Point &rayOrigin, const Vec2d &rayDirection, const Line& segment)
    pub fn ray_intersects_segment(
        ray_origin: Point,
        ray_direction: Vector2<f64>,
        segment: &Line,
    ) -> bool {
        // Circle.cpp:461-463
        let v1 = Vector2::new(
            (ray_origin.x() - segment.a.x()) as f64,
            (ray_origin.y() - segment.a.y()) as f64,
        );
        let v2 = Vector2::new(
            (segment.b.x() - segment.a.x()) as f64,
            (segment.b.y() - segment.a.y()) as f64,
        );
        let v3 = Vector2::new(-ray_direction[1], ray_direction[0]);

        // Circle.cpp:465-467
        let dot = v2[0] * v3[0] + v2[1] * v3[1];
        if dot.abs() < SCALED_EPSILON {
            return false;
        }

        // Circle.cpp:469-470
        let t1 = (v2[0] * v1[1] - v2[1] * v1[0]) / dot;
        let t2 = (v1[0] * v3[0] + v1[1] * v3[1]) / dot;

        // Circle.cpp:472-475
        t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0)
    }

    // BBS: new function to calculate arc radian in X-Y plane
    // Circle.cpp:479-501
    // C++: float ArcSegment::calc_arc_radian(Vec3f start_pos, Vec3f end_pos, Vec3f center_pos, bool is_ccw)
    pub fn calc_arc_radian(
        start_pos: nalgebra::Vector3<f32>,
        end_pos: nalgebra::Vector3<f32>,
        center_pos: nalgebra::Vector3<f32>,
        is_ccw: bool,
    ) -> f32 {
        // Circle.cpp:481-486
        let mut delta1 = center_pos - start_pos;
        let mut delta2 = center_pos - end_pos;
        // only consider arc in x-y plane, so clean z distance
        delta1[2] = 0.0;
        delta2[2] = 0.0;

        let radian: f32;
        // Circle.cpp:488-499
        if (delta1 - delta2).norm() < 1e-6 {
            // start_pos is same with end_pos, we think it's a full circle
            radian = 2.0 * std::f32::consts::PI;
        } else {
            let dot = delta1.dot(&delta2);
            let cross = delta1[0] as f64 * delta2[1] as f64 - delta1[1] as f64 * delta2[0] as f64;
            let mut r = (cross).atan2(dot as f64) as f32;
            if is_ccw {
                r = if r < 0.0 {
                    2.0 * std::f32::consts::PI + r
                } else {
                    r
                };
            } else {
                r = if r < 0.0 {
                    r.abs()
                } else {
                    2.0 * std::f32::consts::PI - r
                };
            }
            radian = r;
        }
        radian
    }

    // Circle.cpp:503-508
    // C++: float ArcSegment::calc_arc_radius(Vec3f start_pos, Vec3f center_pos)
    pub fn calc_arc_radius(
        start_pos: nalgebra::Vector3<f32>,
        center_pos: nalgebra::Vector3<f32>,
    ) -> f32 {
        // Circle.cpp:505-507
        let mut delta1 = center_pos - start_pos;
        delta1[2] = 0.0;
        delta1.norm()
    }

    // BBS: new function to calculate arc length in X-Y plane
    // Circle.cpp:510-514
    // C++: float ArcSegment::calc_arc_length(Vec3f start_pos, Vec3f end_pos, Vec3f center_pos, bool is_ccw)
    pub fn calc_arc_length(
        start_pos: nalgebra::Vector3<f32>,
        end_pos: nalgebra::Vector3<f32>,
        center_pos: nalgebra::Vector3<f32>,
        is_ccw: bool,
    ) -> f32 {
        // Circle.cpp:513
        ArcSegment::calc_arc_radius(start_pos, center_pos)
            * ArcSegment::calc_arc_radian(start_pos, end_pos, center_pos, is_ccw)
    }

    // Convenience accessors mirroring C++ `ArcSegment` inheriting `Circle::center`/`radius`.
    #[inline]
    fn center(&self) -> Point {
        self.circle.center
    }
    #[inline]
    fn radius(&self) -> f64 {
        self.circle.radius
    }
    // get_polar_radians is inherited from Circle in C++.
    #[inline]
    fn get_polar_radians(&self, p: Point) -> f64 {
        self.circle.get_polar_radians(p)
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

        // Length magnitude should remain the same (C++ flips angle sign but not magnitude).
        assert!((arc.length - original_length).abs() < 1.0);
    }

    #[test]
    fn test_arc_direction_values() {
        assert_eq!(ArcDirection::Unknown, ArcDirection::Unknown);
        assert_ne!(ArcDirection::Clockwise, ArcDirection::CounterClockwise);
    }
}
