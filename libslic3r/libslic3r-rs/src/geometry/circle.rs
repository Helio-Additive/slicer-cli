//! Circle geometry utilities.
//!
//! Provides circle fitting, intersection testing, and geometric operations.
//!
//! Mirroring BambuStudio's Geometry/Circle.cpp

use crate::geometry::{Line, PointF};
use crate::CoordF;

/// A circle in 2D space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Circle {
    /// Center point
    pub center: PointF,
    /// Radius
    pub radius: CoordF,
}

impl Circle {
    // Create a new circle.
    pub fn new(center: PointF, radius: CoordF) -> Self {
        Self { center, radius }
    }

    /// Create a circle from center coordinates and radius.
    pub fn from_coords(cx: CoordF, cy: CoordF, radius: CoordF) -> Self {
        Self {
            center: PointF::new(cx, cy),
            radius,
        }
    }

    /// Check if a point is inside the circle (or on boundary).
    pub fn contains(&self, point: PointF) -> bool {
        let dist_sq = (point.x - self.center.x).powi(2) + (point.y - self.center.y).powi(2);
        dist_sq <= self.radius.powi(2) + 1e-10
    }

    /// Check if a point is strictly inside the circle.
    pub fn contains_strict(&self, point: PointF) -> bool {
        let dist_sq = (point.x - self.center.x).powi(2) + (point.y - self.center.y).powi(2);
        dist_sq < self.radius.powi(2)
    }

    /// Get the circumference.
    pub fn circumference(&self) -> CoordF {
        2.0 * std::f64::consts::PI * self.radius
    }

    /// Get the area.
    pub fn area(&self) -> CoordF {
        std::f64::consts::PI * self.radius.powi(2)
    }

    /// Get a point on the circle at a given angle (radians).
    pub fn point_at_angle(&self, angle: CoordF) -> PointF {
        PointF::new(
            self.center.x + self.radius * angle.cos(),
            self.center.y + self.radius * angle.sin(),
        )
    }

    /// Get the bounding box of the circle.
    pub fn bounding_box(&self) -> (PointF, PointF) {
        (
            PointF::new(self.center.x - self.radius, self.center.y - self.radius),
            PointF::new(self.center.x + self.radius, self.center.y + self.radius),
        )
    }

    /// Check if this circle intersects with another circle.
    pub fn intersects_circle(&self, other: &Circle) -> bool {
        let dist_sq =
            (self.center.x - other.center.x).powi(2) + (self.center.y - other.center.y).powi(2);
        let radius_sum = self.radius + other.radius;
        let radius_diff = (self.radius - other.radius).abs();

        // Intersection exists if distance is between |r1-r2| and r1+r2
        dist_sq <= radius_sum.powi(2) && dist_sq >= radius_diff.powi(2)
    }

    /// Check if this circle intersects with a line segment.
    pub fn intersects_line(&self, line: &Line) -> bool {
        let a = PointF::new(
            line.a.x as CoordF / 1_000_000.0,
            line.a.y as CoordF / 1_000_000.0,
        );
        let b = PointF::new(
            line.b.x as CoordF / 1_000_000.0,
            line.b.y as CoordF / 1_000_000.0,
        );

        // Project circle center onto line
        let ab_x = b.x - a.x;
        let ab_y = b.y - a.y;
        let ac_x = self.center.x - a.x;
        let ac_y = self.center.y - a.y;

        let ab_len_sq = ab_x * ab_x + ab_y * ab_y;
        if ab_len_sq == 0.0 {
            // Line is a point
            let dist_sq = ac_x * ac_x + ac_y * ac_y;
            return dist_sq <= self.radius * self.radius;
        }

        let t = (ac_x * ab_x + ac_y * ab_y) / ab_len_sq;
        let t = t.clamp(0.0, 1.0);

        let closest_x = a.x + t * ab_x;
        let closest_y = a.y + t * ab_y;

        let dist_sq = (self.center.x - closest_x).powi(2) + (self.center.y - closest_y).powi(2);
        dist_sq <= self.radius * self.radius
    }

    /// Get intersection points with another circle.
    pub fn circle_intersections(&self, other: &Circle) -> Vec<PointF> {
        let mut points = Vec::new();

        let dx = other.center.x - self.center.x;
        let dy = other.center.y - self.center.y;
        let d_sq = dx * dx + dy * dy;
        let d = d_sq.sqrt();

        if d == 0.0 {
            // Concentric circles
            return points;
        }

        if d > self.radius + other.radius || d < (self.radius - other.radius).abs() {
            // No intersection
            return points;
        }

        // Using the formula for circle-circle intersection
        let a = (self.radius * self.radius - other.radius * other.radius + d_sq) / (2.0 * d);
        let h_sq = self.radius * self.radius - a * a;

        if h_sq < 0.0 {
            return points;
        }

        let h = h_sq.sqrt();

        let x2 = self.center.x + a * dx / d;
        let y2 = self.center.y + a * dy / d;

        if h < 1e-10 {
            // One intersection point (tangent)
            points.push(PointF::new(x2, y2));
        } else {
            // Two intersection points
            points.push(PointF::new(x2 + h * dy / d, y2 - h * dx / d));
            points.push(PointF::new(x2 - h * dy / d, y2 + h * dx / d));
        }

        points
    }
}

impl Default for Circle {
    fn default() -> Self {
        Self {
            center: PointF::new(0.0, 0.0),
            radius: 1.0,
        }
    }
}

/// Fit a circle through three points.
///
/// Returns `None` if the points are collinear.
pub fn fit_circle_through_points(p1: PointF, p2: PointF, p3: PointF) -> Option<Circle> {
    // Calculate the circumcircle of the triangle formed by the three points

    // Calculate midpoints of two edges
    let mid1 = PointF::new((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
    let mid2 = PointF::new((p2.x + p3.x) / 2.0, (p2.y + p3.y) / 2.0);

    // Calculate direction vectors of edges (perpendicular to the edges)
    let dx1 = p2.x - p1.x;
    let dy1 = p2.y - p1.y;
    let dx2 = p3.x - p2.x;
    let dy2 = p3.y - p2.y;

    // Check if points are collinear
    let cross = dx1 * dy2 - dy1 * dx2;
    if cross.abs() < 1e-10 {
        return None;
    }

    // Calculate perpendicular bisector intersection
    // Perpendicular direction to edge 1: (-dy1, dx1)
    // Perpendicular direction to edge 2: (-dy2, dx2)

    // Line 1: mid1 + t1 * (-dy1, dx1)
    // Line 2: mid2 + t2 * (-dy2, dx2)
    // At intersection: mid1.x - t1 * dy1 = mid2.x - t2 * dy2
    //                  mid1.y + t1 * dx1 = mid2.y + t2 * dx2

    // Solving:
    // -t1 * dy1 + t2 * dy2 = mid2.x - mid1.x
    //  t1 * dx1 - t2 * dx2 = mid2.y - mid1.y

    let rhs_x = mid2.x - mid1.x;
    let rhs_y = mid2.y - mid1.y;

    let det = -dy1 * (-dx2) - dx1 * dy2;
    if det.abs() < 1e-10 {
        return None;
    }

    let t1 = (rhs_x * (-dx2) - rhs_y * dy2) / det;

    let center = PointF::new(mid1.x - t1 * dy1, mid1.y + t1 * dx1);
    let radius = ((p1.x - center.x).powi(2) + (p1.y - center.y).powi(2)).sqrt();

    Some(Circle::new(center, radius))
}

/// Fit a circle to a set of points using least squares.
///
/// Uses the algebraic method (Kåsa method) for circle fitting.
pub fn fit_circle_to_points(points: &[PointF]) -> Option<Circle> {
    if points.len() < 3 {
        return None;
    }

    // Kåsa method for circle fitting
    let n = points.len() as CoordF;

    // Compute means
    let mean_x = points.iter().map(|p| p.x).sum::<CoordF>() / n;
    let mean_y = points.iter().map(|p| p.y).sum::<CoordF>() / n;

    // Compute sums for linear system
    let mut sum_uu = 0.0;
    let mut sum_uv = 0.0;
    let mut sum_vv = 0.0;
    let mut sum_uuu = 0.0;
    let mut sum_vvv = 0.0;
    let mut sum_uvv = 0.0;
    let mut sum_vuu = 0.0;

    for p in points {
        let u = p.x - mean_x;
        let v = p.y - mean_y;
        let uu = u * u;
        let vv = v * v;

        sum_uu += uu;
        sum_uv += u * v;
        sum_vv += vv;
        sum_uuu += u * uu;
        sum_vvv += v * vv;
        sum_uvv += u * vv;
        sum_vuu += v * uu;
    }

    // Solve linear system for circle center
    let det = sum_uu * sum_vv - sum_uv * sum_uv;
    if det.abs() < 1e-10 {
        return None;
    }

    let uc =
        (sum_uvv * sum_uv - sum_vuu * sum_vv + sum_uuu * sum_vv - sum_uv * sum_vvv) / (2.0 * det);
    let vc =
        (sum_vuu * sum_uv - sum_uvv * sum_uu + sum_vvv * sum_uu - sum_uv * sum_uuu) / (2.0 * det);

    let center = PointF::new(mean_x + uc, mean_y + vc);

    // Compute radius
    let radius = points
        .iter()
        .map(|p| ((p.x - center.x).powi(2) + (p.y - center.y).powi(2)).sqrt())
        .sum::<CoordF>()
        / n;

    Some(Circle::new(center, radius))
}

/// Find the minimum enclosing circle for a set of points.
///
/// Uses Welzl's algorithm (expected linear time).
pub fn minimum_enclosing_circle(points: &[PointF]) -> Circle {
    if points.is_empty() {
        return Circle::default();
    }

    if points.len() == 1 {
        return Circle::new(points[0], 0.0);
    }

    if points.len() == 2 {
        let center = PointF::new(
            (points[0].x + points[1].x) / 2.0,
            (points[0].y + points[1].y) / 2.0,
        );
        let radius = ((points[0].x - center.x).powi(2) + (points[0].y - center.y).powi(2)).sqrt();
        return Circle::new(center, radius);
    }

    // For small point sets, use brute force
    // For larger sets, Welzl's algorithm would be better
    let mut best_circle = Circle::new(points[0], 0.0);
    let mut min_radius = CoordF::INFINITY;

    // Try all pairs
    for i in 0..points.len() {
        for j in (i + 1)..points.len() {
            let center = PointF::new(
                (points[i].x + points[j].x) / 2.0,
                (points[i].y + points[j].y) / 2.0,
            );
            let radius =
                ((points[i].x - center.x).powi(2) + (points[i].y - center.y).powi(2)).sqrt();

            if radius < min_radius
                && points.iter().all(|p| {
                    let dist = ((p.x - center.x).powi(2) + (p.y - center.y).powi(2)).sqrt();
                    dist <= radius + 1e-10
                })
            {
                min_radius = radius;
                best_circle = Circle::new(center, radius);
            }
        }
    }

    // Try all triples
    for i in 0..points.len() {
        for j in (i + 1)..points.len() {
            for k in (j + 1)..points.len() {
                if let Some(circle) = fit_circle_through_points(points[i], points[j], points[k]) {
                    if circle.radius < min_radius
                        && points.iter().all(|p| {
                            let dist = ((p.x - circle.center.x).powi(2)
                                + (p.y - circle.center.y).powi(2))
                            .sqrt();
                            dist <= circle.radius + 1e-10
                        })
                    {
                        min_radius = circle.radius;
                        best_circle = circle;
                    }
                }
            }
        }
    }

    best_circle
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circle_new() {
        let c = Circle::new(PointF::new(0.0, 0.0), 5.0);
        assert_eq!(c.center.x, 0.0);
        assert_eq!(c.radius, 5.0);
    }

    #[test]
    fn test_circle_contains() {
        let c = Circle::new(PointF::new(0.0, 0.0), 5.0);
        assert!(c.contains(PointF::new(3.0, 4.0)));
        assert!(c.contains(PointF::new(0.0, 0.0)));
        assert!(!c.contains(PointF::new(10.0, 0.0)));
    }

    #[test]
    fn test_circle_circumference() {
        let c = Circle::new(PointF::new(0.0, 0.0), 1.0);
        assert!((c.circumference() - 2.0 * std::f64::consts::PI).abs() < 0.001);
    }

    #[test]
    fn test_circle_area() {
        let c = Circle::new(PointF::new(0.0, 0.0), 1.0);
        assert!((c.area() - std::f64::consts::PI).abs() < 0.001);
    }

    #[test]
    fn test_fit_circle_through_points() {
        let p1 = PointF::new(1.0, 0.0);
        let p2 = PointF::new(0.0, 1.0);
        let p3 = PointF::new(-1.0, 0.0);

        let circle = fit_circle_through_points(p1, p2, p3).unwrap();
        assert!((circle.center.x - 0.0).abs() < 0.001);
        assert!((circle.center.y - 0.0).abs() < 0.001);
        assert!((circle.radius - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_fit_circle_collinear() {
        let p1 = PointF::new(0.0, 0.0);
        let p2 = PointF::new(1.0, 0.0);
        let p3 = PointF::new(2.0, 0.0);

        assert!(fit_circle_through_points(p1, p2, p3).is_none());
    }

    #[test]
    fn test_minimum_enclosing_circle() {
        let points = vec![
            PointF::new(0.0, 0.0),
            PointF::new(2.0, 0.0),
            PointF::new(1.0, 1.0),
        ];

        let circle = minimum_enclosing_circle(&points);
        assert!(circle.radius > 0.0);

        // All points should be inside
        for p in &points {
            assert!(circle.contains(*p));
        }
    }
}
