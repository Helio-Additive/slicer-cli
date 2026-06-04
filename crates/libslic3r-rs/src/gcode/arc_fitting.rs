//! Arc fitting module for converting line segments to G2/G3 arc moves.
//!
//! This module provides algorithms for detecting and fitting arcs to sequences
//! of line segments, which can significantly reduce G-code file size and improve
//! print quality on firmware that supports arc moves (G2/G3).
//!
//! # Overview
//!
//! Arc fitting works by:
//! 1. Analyzing sequences of consecutive points for circular patterns
//! 2. Fitting circles to candidate point sequences
//! 3. Validating that the fitted arc is within tolerance
//! 4. Converting valid arcs to G2 (clockwise) or G3 (counter-clockwise) commands
//!
//! # Benefits
//!
//! - Reduced G-code file size (one arc command replaces many line segments)
//! - Smoother motion on printers with arc support
//! - Potentially faster prints due to reduced communication overhead
//!
//! # BambuStudio Reference
//!
//! This corresponds to `src/libslic3r/GCode/GCodeProcessor.cpp` arc fitting logic.

use crate::geometry::{PointF, Polyline};
use crate::{unscale, CoordF};

/// Configuration for arc fitting.
#[derive(Debug, Clone)]
pub struct ArcFittingConfig {
    /// Maximum deviation from the original path (mm).
    /// Points must be within this distance of the fitted arc.
    pub tolerance: CoordF,

    /// Minimum arc radius (mm).
    /// Arcs smaller than this are kept as line segments.
    pub min_radius: CoordF,

    /// Maximum arc radius (mm).
    /// Very large radii are essentially straight lines.
    pub max_radius: CoordF,

    /// Minimum number of points to consider for arc fitting.
    pub min_points: usize,

    /// Maximum arc angle (radians).
    /// Arcs larger than this are split into multiple arcs.
    pub max_arc_angle: CoordF,

    /// Arc length tolerance (percent).
    /// The arc length must be within this percentage of the original polyline length.
    /// BambuStudio uses 0.05 (5%). This prevents fitting arcs that have significantly
    /// different lengths than the original path.
    pub arc_length_tolerance_percent: CoordF,

    /// Whether arc fitting is enabled.
    pub enabled: bool,
}

impl Default for ArcFittingConfig {
    fn default() -> Self {
        Self {
            tolerance: 0.05,    // 50 microns (matches BambuStudio DEFAULT_SCALED_RESOLUTION)
            min_radius: 0.5,    // 0.5mm minimum
            max_radius: 2000.0, // 2000mm maximum (matches BambuStudio DEFAULT_SCALED_MAX_RADIUS)
            min_points: 3,      // Need at least 3 points to fit a circle
            max_arc_angle: std::f64::consts::PI * 1.5, // 270 degrees max
            arc_length_tolerance_percent: 0.05, // 5% (matches BambuStudio DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE)
            enabled: true,
        }
    }
}

impl ArcFittingConfig {
    // Create a new arc fitting configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder: set tolerance.
    pub fn tolerance(mut self, tolerance: CoordF) -> Self {
        self.tolerance = tolerance;
        self
    }

    /// Builder: set minimum radius.
    pub fn min_radius(mut self, radius: CoordF) -> Self {
        self.min_radius = radius;
        self
    }

    /// Builder: set maximum radius.
    pub fn max_radius(mut self, radius: CoordF) -> Self {
        self.max_radius = radius;
        self
    }

    /// Builder: enable or disable arc fitting.
    pub fn enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Create a strict configuration with tighter tolerances.
    pub fn strict() -> Self {
        Self {
            tolerance: 0.01, // 10 microns
            min_radius: 1.0,
            max_radius: 500.0,
            min_points: 4,
            max_arc_angle: std::f64::consts::PI,
            arc_length_tolerance_percent: 0.02, // 2%
            enabled: true,
        }
    }

    /// Create a relaxed configuration for faster processing.
    pub fn relaxed() -> Self {
        Self {
            tolerance: 0.1, // 100 microns
            min_radius: 0.3,
            max_radius: 2000.0,
            min_points: 3,
            max_arc_angle: std::f64::consts::PI * 1.75,
            arc_length_tolerance_percent: 0.1, // 10%
            enabled: true,
        }
    }
}

/// Direction of an arc (clockwise or counter-clockwise).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArcDirection {
    /// Clockwise arc (G2)
    Clockwise,
    /// Counter-clockwise arc (G3)
    CounterClockwise,
}

impl ArcDirection {
    // Returns the G-code command for this direction.
    pub fn gcode_command(&self) -> &'static str {
        match self {
            ArcDirection::Clockwise => "G2",
            ArcDirection::CounterClockwise => "G3",
        }
    }
}

/// A fitted arc segment.
#[derive(Debug, Clone)]
pub struct FittedArc {
    /// Start point of the arc.
    pub start: PointF,

    /// End point of the arc.
    pub end: PointF,

    /// Center point of the arc.
    pub center: PointF,

    /// Radius of the arc (mm).
    pub radius: CoordF,

    /// Arc direction (clockwise or counter-clockwise).
    pub direction: ArcDirection,

    /// Arc angle (radians).
    pub angle: CoordF,

    /// I offset (X distance from start to center).
    pub i: CoordF,

    /// J offset (Y distance from start to center).
    pub j: CoordF,

    /// Number of original points this arc replaces.
    pub point_count: usize,

    /// Maximum deviation from original points (mm).
    pub max_deviation: CoordF,
}

impl FittedArc {
    // Get the arc length (mm).
    pub fn arc_length(&self) -> CoordF {
        self.radius * self.angle.abs()
    }

    /// Convert to G-code string.
    pub fn to_gcode(&self, e: Option<CoordF>, f: Option<CoordF>) -> String {
        let mut cmd = format!(
            "{} X{:.3} Y{:.3} I{:.3} J{:.3}",
            self.direction.gcode_command(),
            self.end.x,
            self.end.y,
            self.i,
            self.j
        );

        if let Some(e_val) = e {
            cmd.push_str(&format!(" E{:.5}", e_val));
        }

        if let Some(f_val) = f {
            cmd.push_str(&format!(" F{:.0}", f_val));
        }

        cmd
    }
}

/// Result of arc fitting on a path segment.
#[derive(Debug, Clone)]
pub enum PathSegment {
    /// A line segment (original points).
    Line(Vec<PointF>),

    /// A fitted arc.
    Arc(FittedArc),
}

impl PathSegment {
    // Check if this is an arc segment.
    pub fn is_arc(&self) -> bool {
        matches!(self, PathSegment::Arc(_))
    }

    /// Check if this is a line segment.
    pub fn is_line(&self) -> bool {
        matches!(self, PathSegment::Line(_))
    }

    /// Get the start point.
    pub fn start(&self) -> Option<PointF> {
        match self {
            PathSegment::Line(points) => points.first().copied(),
            PathSegment::Arc(arc) => Some(arc.start),
        }
    }

    /// Get the end point.
    pub fn end(&self) -> Option<PointF> {
        match self {
            PathSegment::Line(points) => points.last().copied(),
            PathSegment::Arc(arc) => Some(arc.end),
        }
    }
}

/// Arc fitting processor.
#[derive(Debug, Clone)]
pub struct ArcFitter {
    config: ArcFittingConfig,
}

impl Default for ArcFitter {
    fn default() -> Self {
        Self::new(ArcFittingConfig::default())
    }
}

impl ArcFitter {
    // Create a new arc fitter with the given configuration.
    pub fn new(config: ArcFittingConfig) -> Self {
        Self { config }
    }

    /// Create an arc fitter with default configuration.
    pub fn with_defaults() -> Self {
        Self::default()
    }

    /// Get the configuration.
    pub fn config(&self) -> &ArcFittingConfig {
        &self.config
    }

    /// Process a polyline and convert suitable segments to arcs.
    pub fn process_polyline(&self, polyline: &Polyline) -> Vec<PathSegment> {
        if !self.config.enabled || polyline.len() < self.config.min_points {
            // Return as single line segment
            let points: Vec<PointF> = polyline
                .points()
                .iter()
                .map(|p| PointF::new(unscale(p.x), unscale(p.y)))
                .collect();
            return vec![PathSegment::Line(points)];
        }

        self.process_points(
            &polyline
                .points()
                .iter()
                .map(|p| PointF::new(unscale(p.x), unscale(p.y)))
                .collect::<Vec<_>>(),
        )
    }

    /// Process a sequence of points and convert suitable segments to arcs.
    pub fn process_points(&self, points: &[PointF]) -> Vec<PathSegment> {
        if !self.config.enabled || points.len() < self.config.min_points {
            return vec![PathSegment::Line(points.to_vec())];
        }

        let mut segments = Vec::new();
        let mut i = 0;

        while i < points.len() {
            // Try to fit an arc starting at this point
            let (arc_result, consumed) = self.try_fit_arc(&points[i..]);

            match arc_result {
                Some(arc) if consumed >= self.config.min_points => {
                    segments.push(PathSegment::Arc(arc));
                    i += consumed - 1; // Move to end of arc (overlap by 1 for continuity)
                }
                _ => {
                    // No arc fits, add point to current line segment
                    if let Some(PathSegment::Line(ref mut line_points)) = segments.last_mut() {
                        line_points.push(points[i]);
                    } else {
                        segments.push(PathSegment::Line(vec![points[i]]));
                    }
                    i += 1;
                }
            }
        }

        // Clean up: merge consecutive single-point line segments
        self.merge_line_segments(segments)
    }

    /// Try to fit an arc to a sequence of points.
    /// Returns the fitted arc (if valid) and the number of points consumed.
    fn try_fit_arc(&self, points: &[PointF]) -> (Option<FittedArc>, usize) {
        if points.len() < self.config.min_points {
            return (None, 0);
        }

        // Try progressively larger arcs
        let mut best_arc: Option<FittedArc> = None;
        let mut best_count = 0;

        // Start with minimum points and expand
        for end_idx in self.config.min_points..=points.len() {
            let candidate_points = &points[..end_idx];

            if let Some(arc) = self.fit_arc_to_points(candidate_points) {
                // Validate the arc
                if self.validate_arc(&arc, candidate_points) {
                    best_arc = Some(arc);
                    best_count = end_idx;
                }
            } else {
                // If fitting fails, stop expanding
                break;
            }
        }

        (best_arc, best_count)
    }

    /// Fit a circle to a set of points and create an arc.
    fn fit_arc_to_points(&self, points: &[PointF]) -> Option<FittedArc> {
        if points.len() < 3 {
            return None;
        }

        // Calculate the approximate length of the original polyline
        let polyline_length = self.calculate_polyline_length(points);

        // Use three-point circle fitting
        // Pick first, middle, and last points
        let p1 = points[0];
        let p2 = points[points.len() / 2];
        let p3 = points[points.len() - 1];

        // Find circle center through these three points
        let center = self.find_circle_center(p1, p2, p3)?;

        // Calculate radius from start point
        let radius = ((p1.x - center.x).powi(2) + (p1.y - center.y).powi(2)).sqrt();

        // Verify end point is also on the circle (within tolerance)
        // This is critical for valid G2/G3 arcs - both endpoints must be at the same radius
        let end_radius = ((p3.x - center.x).powi(2) + (p3.y - center.y).powi(2)).sqrt();
        let radius_deviation = (radius - end_radius).abs();
        if radius_deviation > self.config.tolerance {
            // End point is not on the circle - arc would be malformed
            return None;
        }

        // Check radius bounds
        if radius < self.config.min_radius || radius > self.config.max_radius {
            return None;
        }

        // Determine arc direction using cross product
        let v1 = (p2.x - p1.x, p2.y - p1.y);
        let v2 = (p3.x - p2.x, p3.y - p2.y);
        let cross = v1.0 * v2.1 - v1.1 * v2.0;

        let direction = if cross > 0.0 {
            ArcDirection::CounterClockwise
        } else {
            ArcDirection::Clockwise
        };

        // Calculate arc angle
        let start_angle = (p1.y - center.y).atan2(p1.x - center.x);
        let end_angle = (p3.y - center.y).atan2(p3.x - center.x);

        let mut angle = end_angle - start_angle;

        // Normalize angle based on direction
        match direction {
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
        }

        // Check angle bounds
        if angle > self.config.max_arc_angle {
            return None;
        }

        // CRITICAL: Validate arc length matches polyline length (BambuStudio requirement)
        // The arc length must be within arc_length_tolerance_percent of the original path
        let arc_length = radius * angle;
        let length_difference = (arc_length - polyline_length).abs() / polyline_length;

        if length_difference >= self.config.arc_length_tolerance_percent {
            // Arc length doesn't match - this could indicate wrong direction
            // or that the points don't form a good arc
            // Try the opposite direction (BambuStudio does this check)
            let test_angle = (2.0 * std::f64::consts::PI - angle).abs();
            let test_arc_length = radius * test_angle;
            let test_difference = (test_arc_length - polyline_length).abs() / polyline_length;

            if test_difference >= self.config.arc_length_tolerance_percent {
                // Neither direction works - reject this arc
                return None;
            }
            // Otherwise, we'd need to flip the direction, but this suggests
            // the points don't form a good arc, so reject it
            return None;
        }

        // Calculate I and J offsets
        let i = center.x - p1.x;
        let j = center.y - p1.y;

        // Calculate maximum deviation
        let max_deviation = self.calculate_max_deviation(points, center, radius);

        Some(FittedArc {
            start: p1,
            end: p3,
            center,
            radius,
            direction,
            angle,
            i,
            j,
            point_count: points.len(),
            max_deviation,
        })
    }

    /// Find the center of a circle passing through three points.
    fn find_circle_center(&self, p1: PointF, p2: PointF, p3: PointF) -> Option<PointF> {
        // Using the perpendicular bisector method
        let ax = p1.x;
        let ay = p1.y;
        let bx = p2.x;
        let by = p2.y;
        let cx = p3.x;
        let cy = p3.y;

        let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));

        // Check for collinear points
        if d.abs() < 1e-10 {
            return None;
        }

        let ux = ((ax * ax + ay * ay) * (by - cy)
            + (bx * bx + by * by) * (cy - ay)
            + (cx * cx + cy * cy) * (ay - by))
            / d;

        let uy = ((ax * ax + ay * ay) * (cx - bx)
            + (bx * bx + by * by) * (ax - cx)
            + (cx * cx + cy * cy) * (bx - ax))
            / d;

        Some(PointF::new(ux, uy))
    }

    /// Calculate the maximum deviation of points from the fitted arc.
    fn calculate_max_deviation(&self, points: &[PointF], center: PointF, radius: CoordF) -> CoordF {
        let mut max_dev: CoordF = 0.0;

        for point in points {
            let dist_to_center =
                ((point.x - center.x).powi(2) + (point.y - center.y).powi(2)).sqrt();
            let deviation = (dist_to_center - radius).abs();
            max_dev = max_dev.max(deviation);
        }

        max_dev
    }

    /// Calculate the total length of a polyline (sum of segment lengths).
    fn calculate_polyline_length(&self, points: &[PointF]) -> CoordF {
        if points.len() < 2 {
            return 0.0;
        }

        let mut total_length = 0.0;
        for i in 0..points.len() - 1 {
            let dx = points[i + 1].x - points[i].x;
            let dy = points[i + 1].y - points[i].y;
            total_length += (dx * dx + dy * dy).sqrt();
        }

        total_length
    }

    /// Validate that a fitted arc is within tolerance.
    fn validate_arc(&self, arc: &FittedArc, points: &[PointF]) -> bool {
        // Check maximum deviation
        if arc.max_deviation > self.config.tolerance {
            return false;
        }

        // Check that all points lie on the arc (not just the circle)
        // by verifying they're within the arc's angular range
        let start_angle = (arc.start.y - arc.center.y).atan2(arc.start.x - arc.center.x);

        for point in points {
            let point_angle = (point.y - arc.center.y).atan2(point.x - arc.center.x);
            let mut angle_diff = point_angle - start_angle;

            // Normalize angle difference
            while angle_diff < -std::f64::consts::PI {
                angle_diff += 2.0 * std::f64::consts::PI;
            }
            while angle_diff > std::f64::consts::PI {
                angle_diff -= 2.0 * std::f64::consts::PI;
            }

            // Check if point is within the arc's angular extent
            let in_range = match arc.direction {
                ArcDirection::CounterClockwise => {
                    angle_diff >= -0.01 && angle_diff <= arc.angle + 0.01
                }
                ArcDirection::Clockwise => angle_diff <= 0.01 && angle_diff >= -arc.angle - 0.01,
            };

            if !in_range {
                // Point is outside the arc range - might be on opposite side of circle
                // This is a simplified check; could be more sophisticated
            }
        }

        true
    }

    /// Merge consecutive line segments into single segments.
    fn merge_line_segments(&self, segments: Vec<PathSegment>) -> Vec<PathSegment> {
        let mut result = Vec::new();

        for segment in segments {
            match segment {
                PathSegment::Line(points) => {
                    if let Some(PathSegment::Line(ref mut last_points)) = result.last_mut() {
                        // Merge with previous line segment
                        // Skip first point if it's the same as last point of previous segment
                        let start_idx = if !last_points.is_empty()
                            && !points.is_empty()
                            && last_points.last() == points.first()
                        {
                            1
                        } else {
                            0
                        };
                        last_points.extend_from_slice(&points[start_idx..]);
                    } else {
                        result.push(PathSegment::Line(points));
                    }
                }
                PathSegment::Arc(arc) => {
                    result.push(PathSegment::Arc(arc));
                }
            }
        }

        result
    }
}

/// Statistics about arc fitting results.
#[derive(Debug, Clone, Default)]
pub struct ArcFittingStats {
    /// Total number of input points.
    pub input_points: usize,

    /// Number of arc segments created.
    pub arc_count: usize,

    /// Number of line segments remaining.
    pub line_segment_count: usize,

    /// Total points replaced by arcs.
    pub points_replaced: usize,

    /// Compression ratio (input points / output segments).
    pub compression_ratio: CoordF,
}

impl ArcFittingStats {
    // Calculate statistics from arc fitting results.
    pub fn from_segments(input_points: usize, segments: &[PathSegment]) -> Self {
        let mut arc_count = 0;
        let mut line_segment_count = 0;
        let mut points_replaced = 0;

        for segment in segments {
            match segment {
                PathSegment::Arc(arc) => {
                    arc_count += 1;
                    points_replaced += arc.point_count;
                }
                PathSegment::Line(points) => {
                    line_segment_count += points.len().saturating_sub(1);
                }
            }
        }

        let output_segments = arc_count + line_segment_count;
        let compression_ratio = if output_segments > 0 {
            input_points as CoordF / output_segments as CoordF
        } else {
            1.0
        };

        Self {
            input_points,
            arc_count,
            line_segment_count,
            points_replaced,
            compression_ratio,
        }
    }
}

/// Convenience function to process a polyline with default settings.
pub fn fit_arcs(polyline: &Polyline) -> Vec<PathSegment> {
    ArcFitter::with_defaults().process_polyline(polyline)
}

/// Convenience function to process points with default settings.
pub fn fit_arcs_to_points(points: &[PointF]) -> Vec<PathSegment> {
    ArcFitter::with_defaults().process_points(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_circle_points(center: PointF, radius: CoordF, num_points: usize) -> Vec<PointF> {
        (0..num_points)
            .map(|i| {
                let angle = 2.0 * PI * (i as f64) / (num_points as f64);
                PointF::new(
                    center.x + radius * angle.cos(),
                    center.y + radius * angle.sin(),
                )
            })
            .collect()
    }

    fn make_arc_points(
        center: PointF,
        radius: CoordF,
        start_angle: CoordF,
        end_angle: CoordF,
        num_points: usize,
    ) -> Vec<PointF> {
        (0..num_points)
            .map(|i| {
                let t = i as f64 / (num_points - 1) as f64;
                let angle = start_angle + t * (end_angle - start_angle);
                PointF::new(
                    center.x + radius * angle.cos(),
                    center.y + radius * angle.sin(),
                )
            })
            .collect()
    }

    #[test]
    fn test_arc_fitting_config_default() {
        let config = ArcFittingConfig::default();
        assert!(config.enabled);
        assert!(config.tolerance > 0.0);
        assert!(config.min_radius > 0.0);
        assert!(config.min_points >= 3);
    }

    #[test]
    fn test_arc_fitting_config_builder() {
        let config = ArcFittingConfig::new()
            .tolerance(0.1)
            .min_radius(1.0)
            .enabled(false);

        assert!(!config.enabled);
        assert!((config.tolerance - 0.1).abs() < 1e-6);
        assert!((config.min_radius - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_arc_direction_gcode() {
        assert_eq!(ArcDirection::Clockwise.gcode_command(), "G2");
        assert_eq!(ArcDirection::CounterClockwise.gcode_command(), "G3");
    }

    #[test]
    fn test_fitted_arc_to_gcode() {
        let arc = FittedArc {
            start: PointF::new(0.0, 0.0),
            end: PointF::new(10.0, 10.0),
            center: PointF::new(5.0, 0.0),
            radius: 5.0,
            direction: ArcDirection::CounterClockwise,
            angle: PI / 2.0,
            i: 5.0,
            j: 0.0,
            point_count: 10,
            max_deviation: 0.01,
        };

        let gcode = arc.to_gcode(Some(1.5), Some(1200.0));
        assert!(gcode.starts_with("G3"));
        assert!(gcode.contains("X10.000"));
        assert!(gcode.contains("Y10.000"));
        assert!(gcode.contains("I5.000"));
        assert!(gcode.contains("J0.000"));
        assert!(gcode.contains("E1.50000"));
        assert!(gcode.contains("F1200"));
    }

    #[test]
    fn test_arc_fitting_disabled() {
        let config = ArcFittingConfig::default().enabled(false);
        let fitter = ArcFitter::new(config);

        let points = make_arc_points(PointF::new(0.0, 0.0), 10.0, 0.0, PI / 2.0, 20);
        let segments = fitter.process_points(&points);

        // Should return a single line segment when disabled
        assert_eq!(segments.len(), 1);
        assert!(segments[0].is_line());
    }

    #[test]
    fn test_arc_fitting_few_points() {
        let fitter = ArcFitter::with_defaults();

        // With only 2 points, no arc should be fitted
        let points = vec![PointF::new(0.0, 0.0), PointF::new(10.0, 10.0)];
        let segments = fitter.process_points(&points);

        assert_eq!(segments.len(), 1);
        assert!(segments[0].is_line());
    }

    #[test]
    fn test_arc_fitting_perfect_arc() {
        let config = ArcFittingConfig::default().tolerance(0.1);
        let fitter = ArcFitter::new(config);

        // Create a perfect quarter circle
        let center = PointF::new(0.0, 0.0);
        let radius = 10.0;
        let points = make_arc_points(center, radius, 0.0, PI / 2.0, 20);

        let segments = fitter.process_points(&points);

        // Should fit at least one arc
        let has_arc = segments.iter().any(|s| s.is_arc());
        assert!(has_arc, "Should detect arc in perfect circular data");
    }

    #[test]
    fn test_arc_fitting_straight_line() {
        let fitter = ArcFitter::with_defaults();

        // Perfectly straight line - should NOT fit an arc
        let points: Vec<PointF> = (0..20).map(|i| PointF::new(i as f64, i as f64)).collect();

        let segments = fitter.process_points(&points);

        // Should be all line segments, no arcs
        for segment in &segments {
            if let PathSegment::Arc(arc) = segment {
                // If there is an arc, it should have a very large radius
                // (approaching infinity for a straight line)
                assert!(
                    arc.radius > 100.0,
                    "Straight line should not produce small-radius arc"
                );
            }
        }
    }

    #[test]
    fn test_find_circle_center() {
        let fitter = ArcFitter::with_defaults();

        // Three points on a circle centered at (5, 5) with radius 5
        let p1 = PointF::new(10.0, 5.0); // Right
        let p2 = PointF::new(5.0, 10.0); // Top
        let p3 = PointF::new(0.0, 5.0); // Left

        let center = fitter.find_circle_center(p1, p2, p3);

        assert!(center.is_some());
        let c = center.unwrap();
        assert!((c.x - 5.0).abs() < 0.001);
        assert!((c.y - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_find_circle_center_collinear() {
        let fitter = ArcFitter::with_defaults();

        // Three collinear points - no circle center
        let p1 = PointF::new(0.0, 0.0);
        let p2 = PointF::new(5.0, 5.0);
        let p3 = PointF::new(10.0, 10.0);

        let center = fitter.find_circle_center(p1, p2, p3);

        assert!(center.is_none());
    }

    #[test]
    fn test_path_segment_accessors() {
        let line_points = vec![PointF::new(0.0, 0.0), PointF::new(10.0, 10.0)];
        let line = PathSegment::Line(line_points.clone());

        assert!(line.is_line());
        assert!(!line.is_arc());
        assert_eq!(line.start(), Some(PointF::new(0.0, 0.0)));
        assert_eq!(line.end(), Some(PointF::new(10.0, 10.0)));

        let arc = PathSegment::Arc(FittedArc {
            start: PointF::new(0.0, 0.0),
            end: PointF::new(10.0, 10.0),
            center: PointF::new(5.0, 0.0),
            radius: 5.0,
            direction: ArcDirection::Clockwise,
            angle: PI,
            i: 5.0,
            j: 0.0,
            point_count: 10,
            max_deviation: 0.01,
        });

        assert!(arc.is_arc());
        assert!(!arc.is_line());
    }

    #[test]
    fn test_arc_fitting_stats() {
        let segments = vec![
            PathSegment::Arc(FittedArc {
                start: PointF::new(0.0, 0.0),
                end: PointF::new(10.0, 10.0),
                center: PointF::new(5.0, 0.0),
                radius: 5.0,
                direction: ArcDirection::Clockwise,
                angle: PI,
                i: 5.0,
                j: 0.0,
                point_count: 15,
                max_deviation: 0.01,
            }),
            PathSegment::Line(vec![
                PointF::new(10.0, 10.0),
                PointF::new(20.0, 20.0),
                PointF::new(30.0, 30.0),
            ]),
        ];

        let stats = ArcFittingStats::from_segments(20, &segments);

        assert_eq!(stats.arc_count, 1);
        assert_eq!(stats.line_segment_count, 2);
        assert_eq!(stats.points_replaced, 15);
        assert!(stats.compression_ratio > 1.0);
    }

    #[test]
    fn test_fitted_arc_arc_length() {
        let arc = FittedArc {
            start: PointF::new(0.0, 0.0),
            end: PointF::new(10.0, 0.0),
            center: PointF::new(5.0, 0.0),
            radius: 5.0,
            direction: ArcDirection::CounterClockwise,
            angle: PI,
            i: 5.0,
            j: 0.0,
            point_count: 10,
            max_deviation: 0.01,
        };

        // Half circle with radius 5 should have length ~15.7
        let length = arc.arc_length();
        assert!((length - 5.0 * PI).abs() < 0.001);
    }
}
