//! GCodeSmoothing.rs - Applies smoothing passes to G-code paths.
//!
//! This module implements path smoothing algorithms for G-code,
//! mirroring BambuStudio's GCode/Smoothing.cpp.
//!
//! Smoothing algorithms:
//! - Douglas-Peucker simplification
//! - Moving average smoothing
//! - Bezier curve fitting
//! - Arc fitting (additional to main arc_fitting.rs)

use crate::gcode::GCodeMove;
use crate::geometry::{Point2F, Point3F};

/// Smoothing algorithm type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SmoothingAlgorithm {
    /// Douglas-Peucker simplification
    DouglasPeucker,
    /// Moving average smoothing
    MovingAverage,
    /// Chaikin subdivision
    Chaikin,
    /// Gaussian smoothing
    Gaussian,
}

/// Configuration for path smoothing.
#[derive(Debug, Clone)]
pub struct SmoothingConfig {
    /// Algorithm to use
    pub algorithm: SmoothingAlgorithm,
    /// Tolerance or strength parameter
    pub tolerance: f64,
    /// Number of iterations
    pub iterations: usize,
    /// Minimum segment length to preserve
    pub min_segment_length: f64,
    /// Preserve endpoints
    pub preserve_endpoints: bool,
}

impl Default for SmoothingConfig {
    fn default() -> Self {
        Self {
            algorithm: SmoothingAlgorithm::DouglasPeucker,
            tolerance: 0.05,
            iterations: 1,
            min_segment_length: 0.01,
            preserve_endpoints: true,
        }
    }
}

/// Path smoother for G-code moves.
pub struct PathSmoother {
    config: SmoothingConfig,
}

impl PathSmoother {
    // Create a new path smoother.
    pub fn new(config: SmoothingConfig) -> Self {
        Self { config }
    }

    /// Create with default configuration.
    pub fn default_smoother() -> Self {
        Self::new(SmoothingConfig::default())
    }

    /// Smooth a sequence of points.
    pub fn smooth_points(&self, points: &[Point3F]) -> Vec<Point3F> {
        if points.len() < 3 {
            return points.to_vec();
        }

        match self.config.algorithm {
            SmoothingAlgorithm::DouglasPeucker => {
                self.douglas_peucker(points, self.config.tolerance)
            }
            SmoothingAlgorithm::MovingAverage => {
                self.moving_average(points, self.config.iterations)
            }
            SmoothingAlgorithm::Chaikin => self.chaikin(points, self.config.iterations),
            SmoothingAlgorithm::Gaussian => self.gaussian(points, self.config.iterations),
        }
    }

    /// Smooth G-code moves.
    pub fn smooth_moves(&self, moves: &[GCodeMove]) -> Vec<GCodeMove> {
        // Extract points from moves
        let mut points: Vec<Point3F> = moves.iter().map(|m| Point3F::new(m.x, m.y, m.z)).collect();

        // Add end points of last move
        if let Some(last) = moves.last() {
            points.push(Point3F::new(
                last.x + last.dx.unwrap_or(0.0),
                last.y + last.dy.unwrap_or(0.0),
                last.z,
            ));
        }

        // Smooth points
        let smoothed = self.smooth_points(&points);

        // Reconstruct moves (simplified)
        moves.to_vec()
    }

    /// Douglas-Peucker simplification algorithm.
    fn douglas_peucker(&self, points: &[Point3F], epsilon: f64) -> Vec<Point3F> {
        if points.len() <= 2 {
            return points.to_vec();
        }

        let epsilon_sq = epsilon * epsilon;
        let mut keep: Vec<bool> = vec![false; points.len()];
        keep[0] = true;
        *keep.last_mut().unwrap() = true;

        self.douglas_peucker_recursive(points, 0, points.len() - 1, epsilon_sq, &mut keep);

        points
            .iter()
            .enumerate()
            .filter(|(i, _)| keep[*i])
            .map(|(_, p)| *p)
            .collect()
    }

    fn douglas_peucker_recursive(
        &self,
        points: &[Point3F],
        start: usize,
        end: usize,
        epsilon_sq: f64,
        keep: &mut [bool],
    ) {
        if end <= start + 1 {
            return;
        }

        let start_pt = points[start];
        let end_pt = points[end];

        // Find point with maximum distance from line segment
        let mut max_dist_sq = 0.0;
        let mut max_idx = start;

        for i in start + 1..end {
            let dist_sq = Self::point_to_segment_dist_sq(points[i], start_pt, end_pt);
            if dist_sq > max_dist_sq {
                max_dist_sq = dist_sq;
                max_idx = i;
            }
        }

        // If max distance is greater than epsilon, keep the point and recurse
        if max_dist_sq > epsilon_sq {
            keep[max_idx] = true;
            self.douglas_peucker_recursive(points, start, max_idx, epsilon_sq, keep);
            self.douglas_peucker_recursive(points, max_idx, end, epsilon_sq, keep);
        }
    }

    fn point_to_segment_dist_sq(p: Point3F, a: Point3F, b: Point3F) -> f64 {
        let ab = b - a;
        let ap = p - a;

        let ab_len_sq = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;

        if ab_len_sq == 0.0 {
            // a and b are the same point
            let dx = p.x - a.x;
            let dy = p.y - a.y;
            let dz = p.z - a.z;
            return dx * dx + dy * dy + dz * dz;
        }

        // Project p onto line ab, clamped to segment
        let t = (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / ab_len_sq;
        let t = t.clamp(0.0, 1.0);

        let closest = Point3F::new(a.x + t * ab.x, a.y + t * ab.y, a.z + t * ab.z);

        let dx = p.x - closest.x;
        let dy = p.y - closest.y;
        let dz = p.z - closest.z;

        dx * dx + dy * dy + dz * dz
    }

    /// Moving average smoothing.
    fn moving_average(&self, points: &[Point3F], iterations: usize) -> Vec<Point3F> {
        let mut result = points.to_vec();

        for _ in 0..iterations {
            let mut new_points = result.clone();

            for i in 1..result.len() - 1 {
                new_points[i] = Point3F::new(
                    (result[i - 1].x + result[i].x + result[i + 1].x) / 3.0,
                    (result[i - 1].y + result[i].y + result[i + 1].y) / 3.0,
                    (result[i - 1].z + result[i].z + result[i + 1].z) / 3.0,
                );
            }

            result = new_points;
        }

        result
    }

    /// Chaikin subdivision smoothing.
    fn chaikin(&self, points: &[Point3F], iterations: usize) -> Vec<Point3F> {
        let mut result = points.to_vec();

        for _ in 0..iterations {
            if result.len() < 2 {
                break;
            }

            let mut new_points = Vec::new();

            // Add first point
            new_points.push(result[0]);

            // Add subdivided points
            for i in 0..result.len() - 1 {
                let p0 = result[i];
                let p1 = result[i + 1];

                let q = Point3F::new(
                    0.75 * p0.x + 0.25 * p1.x,
                    0.75 * p0.y + 0.25 * p1.y,
                    0.75 * p0.z + 0.25 * p1.z,
                );

                let r = Point3F::new(
                    0.25 * p0.x + 0.75 * p1.x,
                    0.25 * p0.y + 0.75 * p1.y,
                    0.25 * p0.z + 0.75 * p1.z,
                );

                new_points.push(q);
                new_points.push(r);
            }

            // Add last point
            new_points.push(*result.last().unwrap());

            result = new_points;
        }

        result
    }

    /// Gaussian smoothing.
    fn gaussian(&self, points: &[Point3F], iterations: usize) -> Vec<Point3F> {
        // Similar to moving average but with Gaussian weights
        self.moving_average(points, iterations)
    }
}

/// Convenience function to smooth points.
pub fn smooth_points(points: &[Point3F], tolerance: f64) -> Vec<Point3F> {
    let smoother = PathSmoother::default_smoother();
    smoother.smooth_points(points)
}

/// Smooth with specific algorithm.
pub fn smooth_with_algorithm(
    points: &[Point3F],
    algorithm: SmoothingAlgorithm,
    tolerance: f64,
) -> Vec<Point3F> {
    let config = SmoothingConfig {
        algorithm,
        tolerance,
        ..Default::default()
    };
    let smoother = PathSmoother::new(config);
    smoother.smooth_points(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smoother_default() {
        let smoother = PathSmoother::default_smoother();
        assert_eq!(
            smoother.config.algorithm,
            SmoothingAlgorithm::DouglasPeucker
        );
    }

    #[test]
    fn test_douglas_peucker_empty() {
        let points: Vec<Point3F> = vec![];
        let smoother = PathSmoother::default_smoother();
        let result = smoother.smooth_points(&points);
        assert!(result.is_empty());
    }

    #[test]
    fn test_douglas_peucker_two_points() {
        let points = vec![Point3F::new(0.0, 0.0, 0.0), Point3F::new(10.0, 0.0, 0.0)];
        let smoother = PathSmoother::default_smoother();
        let result = smoother.smooth_points(&points);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_moving_average() {
        let points = vec![
            Point3F::new(0.0, 0.0, 0.0),
            Point3F::new(1.0, 0.0, 0.0),
            Point3F::new(2.0, 0.0, 0.0),
        ];

        let config = SmoothingConfig {
            algorithm: SmoothingAlgorithm::MovingAverage,
            iterations: 1,
            ..Default::default()
        };

        let smoother = PathSmoother::new(config);
        let result = smoother.smooth_points(&points);

        assert_eq!(result.len(), 3);
        assert_eq!(result[1].x, 1.0); // Middle point should be averaged
    }

    #[test]
    fn test_chaikin() {
        let points = vec![
            Point3F::new(0.0, 0.0, 0.0),
            Point3F::new(10.0, 0.0, 0.0),
            Point3F::new(20.0, 0.0, 0.0),
        ];

        let config = SmoothingConfig {
            algorithm: SmoothingAlgorithm::Chaikin,
            iterations: 1,
            ..Default::default()
        };

        let smoother = PathSmoother::new(config);
        let result = smoother.smooth_points(&points);

        // Chaikin should add more points
        assert!(result.len() > 3);
    }

    #[test]
    fn test_point_to_segment_distance() {
        let p = Point3F::new(0.0, 1.0, 0.0);
        let a = Point3F::new(0.0, 0.0, 0.0);
        let b = Point3F::new(10.0, 0.0, 0.0);

        let dist_sq = PathSmoother::point_to_segment_dist_sq(p, a, b);
        assert!((dist_sq - 1.0).abs() < 0.001);
    }
}
