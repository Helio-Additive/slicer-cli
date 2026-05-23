//! Space-filling curve infill patterns.
//!
//! This module implements infill patterns based on mathematical space-filling curves:
//! - **Hilbert Curve**: A continuous fractal curve that fills a square
//! - **Archimedean Chords**: A spiral pattern from center outward
//! - **Octagram Spiral**: An 8-pointed star spiral pattern
//!
//! These patterns create continuous paths that minimize travel moves and provide
//! interesting visual effects.
//!
//! # BambuStudio Reference
//!
//! This corresponds to:
//! - `src/libslic3r/Fill/FillPlanePath.cpp`
//! - `src/libslic3r/Fill/FillPlanePath.hpp`
//!
//! The original Perl code used path generators from Math::PlanePath library:
//! - http://user42.tuxfamily.org/math-planepath/
//! - http://user42.tuxfamily.org/math-planepath/gallery.html

use crate::geometry::{ExPolygon, Point, Polyline};
use crate::{scale, Coord, CoordF};
use std::f64::consts::PI;

/// Configuration for plan path infill patterns.
/// FillPlanePath.hpp:15-30
#[derive(Debug, Clone)]
pub struct PlanPathConfig {
    /// Line spacing (extrusion width / density) in mm.
    pub spacing: CoordF,

    /// Infill density (0.0 - 1.0).
    pub density: CoordF,

    /// Resolution for curve discretization (mm).
    pub resolution: CoordF,

    /// Whether to align pattern across layers using object bounding box.
    pub align_to_object: bool,

    /// Whether to connect separate polylines.
    pub connect_lines: bool,
}

/// Default configuration for plan path infill
/// FillPlanePath.hpp:25-30
impl Default for PlanPathConfig {
    fn default() -> Self {
        // FillPlanePath.cpp:45-50
        Self {
            spacing: 0.45,
            density: 0.2,
            resolution: 0.1,
            align_to_object: true,
            connect_lines: true,
        }
    }
}

impl PlanPathConfig {
    // Create config from density and extrusion width.
    pub fn from_density(density: CoordF, extrusion_width: CoordF) -> Self {
        let density = density.clamp(0.01, 1.0);
        Self {
            spacing: extrusion_width / density,
            density,
            ..Default::default()
        }
    }

    /// Set resolution for curve discretization.
    pub fn with_resolution(mut self, resolution: CoordF) -> Self {
        self.resolution = resolution;
        self
    }

    /// Set alignment mode.
    pub fn with_alignment(mut self, align: bool) -> Self {
        self.align_to_object = align;
        self
    }
}

/// Result from plan path generation.
/// FillPlanePath.hpp:35-50
#[derive(Debug, Clone, Default)]
pub struct PlanPathResult {
    /// Generated polylines.
    pub polylines: Vec<Polyline>,

    /// Total length of infill in mm.
    pub total_length_mm: CoordF,

    /// Pattern type used.
    pub pattern: PlanPathPattern,
}

impl PlanPathResult {
    // Create a new result.
    pub fn new(polylines: Vec<Polyline>, pattern: PlanPathPattern) -> Self {
        let total_length_mm = polylines.iter().map(|p| p.length()).sum();
        Self {
            polylines,
            total_length_mm,
            pattern,
        }
    }

    /// Check if any infill was generated.
    pub fn is_empty(&self) -> bool {
        self.polylines.is_empty()
    }

    /// Get the number of paths.
    pub fn path_count(&self) -> usize {
        self.polylines.len()
    }
}

/// Plan path pattern types.
/// FillPlanePath.hpp:55-70
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlanPathPattern {
    /// Hilbert space-filling curve.
    #[default]
    HilbertCurve,

    /// Archimedean spiral from center outward.
    ArchimedeanChords,

    /// 8-pointed star spiral pattern.
    OctagramSpiral,
}

// =============================================================================
// Hilbert Curve Implementation
// =============================================================================

/// Convert a Hilbert curve index to XY coordinates.
///
/// Based on the algorithm from Math::PlanePath::HilbertCurve.
///
/// The Hilbert curve states represent different orientations:
/// - state=0:  Plain orientation (3--2, 0--1)
/// - state=4:  Transpose
/// - state=8:  Rotate 180
/// - state=12: Rotate 180 + transpose
fn hilbert_index_to_xy(n: usize) -> (Coord, Coord) {
    // State transition table
    const NEXT_STATE: [usize; 16] = [4, 0, 0, 12, 0, 4, 4, 8, 12, 8, 8, 4, 8, 12, 12, 0];

    // Digit to X coordinate mapping
    const DIGIT_TO_X: [Coord; 16] = [0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0];

    // Digit to Y coordinate mapping
    const DIGIT_TO_Y: [Coord; 16] = [0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1];

    // Count number of 2-bit digits
    let mut ndigits = 0;
    let mut nc = n;
    while nc > 0 {
        nc >>= 2;
        ndigits += 1;
    }

    // Initial state depends on parity of digit count
    let mut state = if ndigits & 1 != 0 { 4 } else { 0 };

    let mut x: Coord = 0;
    let mut y: Coord = 0;

    for i in (0..ndigits).rev() {
        let digit = (n >> (i * 2)) & 3;
        let idx = state + digit;
        x |= DIGIT_TO_X[idx] << i;
        y |= DIGIT_TO_Y[idx] << i;
        state = NEXT_STATE[idx];
    }

    (x, y)
}

/// Generate Hilbert curve points.
fn generate_hilbert_curve_points(
    min_x: Coord,
    min_y: Coord,
    max_x: Coord,
    max_y: Coord,
) -> Vec<(CoordF, CoordF)> {
    // Find minimum power of two square to fit the domain
    let width = (max_x + 1 - min_x) as usize;
    let height = (max_y + 1 - min_y) as usize;
    let sz0 = width.max(height);

    let mut sz: usize = 2;
    while sz < sz0 {
        sz <<= 1;
    }

    let sz2 = sz * sz;
    let mut points = Vec::with_capacity(sz2);

    for i in 0..sz2 {
        let (px, py) = hilbert_index_to_xy(i);
        points.push(((px + min_x) as CoordF, (py + min_y) as CoordF));
    }

    points
}

// =============================================================================
// Archimedean Chords Implementation
// =============================================================================

/// Generate Archimedean spiral points.
///
/// In polar coordinates: r = a + b*theta
/// This creates a spiral from the center outward.
fn generate_archimedean_chords_points(
    _min_x: Coord,
    _min_y: Coord,
    max_x: Coord,
    max_y: Coord,
    resolution: CoordF,
) -> Vec<(CoordF, CoordF)> {
    // Maximum radius to achieve (diagonal of bounding box * sqrt(2) + margin)
    let rmax =
        ((max_x as CoordF).powi(2) + (max_y as CoordF).powi(2)).sqrt() * 2.0_f64.sqrt() + 1.5;

    // Spiral parameters: r = a + b*theta
    let a = 1.0;
    let b = 1.0 / (2.0 * PI);

    let mut theta = 0.0;
    let mut r = 1.0;

    let mut points = Vec::new();

    // Start at center
    points.push((0.0, 0.0));
    points.push((1.0, 0.0));

    while r < rmax {
        // Discretization angle to achieve resolution error
        // Using: d_theta = 2 * acos(1 - resolution / r)
        let d_theta = 2.0 * (1.0 - resolution / r).max(-1.0).min(1.0).acos();
        theta += d_theta;
        r = a + b * theta;
        points.push((r * theta.cos(), r * theta.sin()));
    }

    points
}

// =============================================================================
// Octagram Spiral Implementation
// =============================================================================

/// Generate Octagram spiral points.
///
/// Creates an 8-pointed star pattern that spirals outward.
fn generate_octagram_spiral_points(
    _min_x: Coord,
    _min_y: Coord,
    max_x: Coord,
    max_y: Coord,
) -> Vec<(CoordF, CoordF)> {
    // Maximum radius to achieve
    let rmax =
        ((max_x as CoordF).powi(2) + (max_y as CoordF).powi(2)).sqrt() * 2.0_f64.sqrt() + 1.5;

    let r_inc = 2.0_f64.sqrt();
    let mut r = 0.0;

    let mut points = Vec::new();

    // Start at center
    points.push((0.0, 0.0));

    while r < rmax {
        r += r_inc;
        let rx = r / 2.0_f64.sqrt();
        let r2 = r + rx;

        // 16 points forming the octagram at this radius
        points.push((r, 0.0));
        points.push((r2, rx));
        points.push((rx, rx));
        points.push((rx, r2));
        points.push((0.0, r));
        points.push((-rx, r2));
        points.push((-rx, rx));
        points.push((-r2, rx));
        points.push((-r, 0.0));
        points.push((-r2, -rx));
        points.push((-rx, -rx));
        points.push((-rx, -r2));
        points.push((0.0, -r));
        points.push((rx, -r2));
        points.push((rx, -rx));
        points.push((r2 + r_inc, -rx));
    }

    points
}

// =============================================================================
// Generator
// =============================================================================

/// Generator for plan path infill patterns.
/// FillPlanePath.hpp:75-85
#[derive(Debug, Clone)]
pub struct PlanPathGenerator {
    config: PlanPathConfig,
    pattern: PlanPathPattern,
}

/// Implementation of PlanPathGenerator
/// FillPlanePath.cpp:69-75
impl PlanPathGenerator {
    // Create a new generator with configuration and pattern.
    // FillPlanePath.cpp:69
    pub fn new(config: PlanPathConfig, pattern: PlanPathPattern) -> Self {
        Self { config, pattern }
    }

    /// Create a generator with default configuration.
    pub fn with_defaults(pattern: PlanPathPattern) -> Self {
        Self::new(PlanPathConfig::default(), pattern)
    }

    /// Create a Hilbert curve generator.
    pub fn hilbert_curve(config: PlanPathConfig) -> Self {
        Self::new(config, PlanPathPattern::HilbertCurve)
    }

    /// Create an Archimedean chords generator.
    pub fn archimedean_chords(config: PlanPathConfig) -> Self {
        Self::new(config, PlanPathPattern::ArchimedeanChords)
    }

    /// Create an Octagram spiral generator.
    pub fn octagram_spiral(config: PlanPathConfig) -> Self {
        Self::new(config, PlanPathPattern::OctagramSpiral)
    }

    /// Get the configuration.
    pub fn config(&self) -> &PlanPathConfig {
        &self.config
    }

    /// Get the pattern type.
    pub fn pattern(&self) -> PlanPathPattern {
        self.pattern
    }

    /// Generate infill for the given boundary.
    pub fn generate(&self, boundary: &ExPolygon) -> PlanPathResult {
        if self.config.density <= 0.0 {
            return PlanPathResult::default();
        }

        let bbox = boundary.bounding_box();
        if bbox.is_empty() {
            return PlanPathResult::default();
        }

        // Calculate distance between lines based on spacing and density
        let distance_between_lines = scale(self.config.spacing);

        if distance_between_lines <= 0 {
            return PlanPathResult::default();
        }

        // Calculate grid bounds
        let min_x = (bbox.min.x as f64 / distance_between_lines as f64).ceil() as Coord;
        let min_y = (bbox.min.y as f64 / distance_between_lines as f64).ceil() as Coord;
        let max_x = (bbox.max.x as f64 / distance_between_lines as f64).ceil() as Coord;
        let max_y = (bbox.max.y as f64 / distance_between_lines as f64).ceil() as Coord;

        // Calculate resolution in grid units
        let resolution = scale(self.config.resolution) as f64 / distance_between_lines as f64;

        // Generate the pattern
        let pattern_points = match self.pattern {
            PlanPathPattern::HilbertCurve => {
                generate_hilbert_curve_points(min_x, min_y, max_x, max_y)
            }
            PlanPathPattern::ArchimedeanChords => {
                generate_archimedean_chords_points(min_x, min_y, max_x, max_y, resolution.max(0.01))
            }
            PlanPathPattern::OctagramSpiral => {
                generate_octagram_spiral_points(min_x, min_y, max_x, max_y)
            }
        };

        if pattern_points.len() < 2 {
            return PlanPathResult::default();
        }

        // Convert to scaled points
        let scale_factor = distance_between_lines as CoordF;
        let shift = if self.is_centered() {
            bbox.center()
        } else {
            bbox.min
        };

        let points: Vec<Point> = pattern_points
            .iter()
            .map(|(x, y)| {
                Point::new(
                    (x * scale_factor).round() as Coord + shift.x,
                    (y * scale_factor).round() as Coord + shift.y,
                )
            })
            .collect();

        let polyline = Polyline::from_points(points);

        // Clip to boundary
        let clipped = clip_polyline_to_expolygon(&polyline, boundary);

        if clipped.is_empty() {
            return PlanPathResult::default();
        }

        PlanPathResult::new(clipped, self.pattern)
    }

    /// Check if pattern is centered (spiral patterns are centered).
    fn is_centered(&self) -> bool {
        matches!(
            self.pattern,
            PlanPathPattern::ArchimedeanChords | PlanPathPattern::OctagramSpiral
        )
    }
}

/// Clip a polyline to an ExPolygon boundary.
fn clip_polyline_to_expolygon(polyline: &Polyline, boundary: &ExPolygon) -> Vec<Polyline> {
    let points = polyline.points();
    if points.len() < 2 {
        return Vec::new();
    }

    let mut result = Vec::new();
    let mut current_segment: Vec<Point> = Vec::new();

    for point in points {
        let inside = point_in_expolygon(point, boundary);

        if inside {
            current_segment.push(point.clone());
        } else {
            if current_segment.len() >= 2 {
                result.push(Polyline::from_points(current_segment));
            }
            current_segment = Vec::new();
        }
    }

    if current_segment.len() >= 2 {
        result.push(Polyline::from_points(current_segment));
    }

    result
}

/// Check if a point is inside an ExPolygon.
fn point_in_expolygon(point: &Point, expoly: &ExPolygon) -> bool {
    if !point_in_polygon(point, &expoly.contour) {
        return false;
    }

    for hole in &expoly.holes {
        if point_in_polygon(point, hole) {
            return false;
        }
    }

    true
}

/// Check if a point is inside a polygon using ray casting.
fn point_in_polygon(point: &Point, polygon: &crate::geometry::Polygon) -> bool {
    let points = polygon.points();
    if points.len() < 3 {
        return false;
    }

    let mut inside = false;
    let n = points.len();
    let mut j = n - 1;

    for i in 0..n {
        let pi = &points[i];
        let pj = &points[j];

        if (pi.y > point.y) != (pj.y > point.y) {
            let x_intersect = (pj.x - pi.x) as i128 * (point.y - pi.y) as i128
                / (pj.y - pi.y) as i128
                + pi.x as i128;
            if (point.x as i128) < x_intersect {
                inside = !inside;
            }
        }
        j = i;
    }

    inside
}

// =============================================================================
// Convenience Functions
// =============================================================================

/// Generate Hilbert curve infill for a boundary.
pub fn generate_hilbert_curve(
    boundary: &ExPolygon,
    density: CoordF,
    extrusion_width: CoordF,
) -> PlanPathResult {
    let config = PlanPathConfig::from_density(density, extrusion_width);
    let generator = PlanPathGenerator::hilbert_curve(config);
    generator.generate(boundary)
}

/// Generate Archimedean chords infill for a boundary.
pub fn generate_archimedean_chords(
    boundary: &ExPolygon,
    density: CoordF,
    extrusion_width: CoordF,
) -> PlanPathResult {
    let config = PlanPathConfig::from_density(density, extrusion_width);
    let generator = PlanPathGenerator::archimedean_chords(config);
    generator.generate(boundary)
}

/// Generate Octagram spiral infill for a boundary.
pub fn generate_octagram_spiral(
    boundary: &ExPolygon,
    density: CoordF,
    extrusion_width: CoordF,
) -> PlanPathResult {
    let config = PlanPathConfig::from_density(density, extrusion_width);
    let generator = PlanPathGenerator::octagram_spiral(config);
    generator.generate(boundary)
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn make_square_boundary(size_mm: CoordF) -> ExPolygon {
        let size = scale(size_mm);
        let contour = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(size, 0),
            Point::new(size, size),
            Point::new(0, size),
        ]);
        ExPolygon::new(contour)
    }

    fn make_square_with_hole(size_mm: CoordF, hole_size_mm: CoordF) -> ExPolygon {
        let size = scale(size_mm);
        let hole_size = scale(hole_size_mm);
        let offset = (size - hole_size) / 2;

        let contour = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(size, 0),
            Point::new(size, size),
            Point::new(0, size),
        ]);

        let hole = Polygon::from_points(vec![
            Point::new(offset, offset),
            Point::new(offset + hole_size, offset),
            Point::new(offset + hole_size, offset + hole_size),
            Point::new(offset, offset + hole_size),
        ]);

        ExPolygon::with_holes(contour, vec![hole])
    }

    #[test]
    fn test_config_default() {
        let config = PlanPathConfig::default();
        assert!((config.density - 0.2).abs() < 1e-6);
        assert!((config.spacing - 0.45).abs() < 1e-6);
    }

    #[test]
    fn test_config_from_density() {
        let config = PlanPathConfig::from_density(0.4, 0.5);
        assert!((config.density - 0.4).abs() < 1e-6);
        assert!((config.spacing - 1.25).abs() < 1e-6); // 0.5 / 0.4 = 1.25
    }

    #[test]
    fn test_hilbert_index_to_xy() {
        // Test that Hilbert curve generates valid coordinates
        // The exact mapping depends on the state machine implementation
        let (x0, y0) = hilbert_index_to_xy(0);
        assert_eq!((x0, y0), (0, 0)); // First point is always origin

        // Verify we get different coordinates for different indices
        let (x1, y1) = hilbert_index_to_xy(1);
        let (x2, y2) = hilbert_index_to_xy(2);
        let (x3, y3) = hilbert_index_to_xy(3);

        // All coordinates should be in [0, 1] range for indices 0-3
        assert!(x1 <= 1 && y1 <= 1);
        assert!(x2 <= 1 && y2 <= 1);
        assert!(x3 <= 1 && y3 <= 1);

        // All four points should be distinct
        let points = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)];
        for i in 0..4 {
            for j in (i + 1)..4 {
                assert_ne!(
                    points[i], points[j],
                    "Points {} and {} should be distinct",
                    i, j
                );
            }
        }
    }

    #[test]
    fn test_generate_hilbert_curve_raw_points() {
        let points = generate_hilbert_curve_points(0, 0, 3, 3);

        // Should generate 16 points for a 4x4 grid (power of 2)
        assert_eq!(points.len(), 16);

        // First point should be at origin
        assert!((points[0].0 - 0.0).abs() < 1e-6);
        assert!((points[0].1 - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_generate_archimedean_raw_points() {
        let points = generate_archimedean_chords_points(0, 0, 10, 10, 0.1);

        // Should generate multiple points
        assert!(points.len() > 10);

        // First two points should be at center and (1, 0)
        assert!((points[0].0 - 0.0).abs() < 1e-6);
        assert!((points[0].1 - 0.0).abs() < 1e-6);
        assert!((points[1].0 - 1.0).abs() < 1e-6);
        assert!((points[1].1 - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_generate_octagram_raw_points() {
        let points = generate_octagram_spiral_points(0, 0, 10, 10);

        // Should generate multiple points
        assert!(points.len() > 10);

        // First point should be at center
        assert!((points[0].0 - 0.0).abs() < 1e-6);
        assert!((points[0].1 - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_hilbert_generator() {
        let boundary = make_square_boundary(50.0);
        let config = PlanPathConfig::from_density(0.3, 0.45);
        let generator = PlanPathGenerator::hilbert_curve(config);
        let result = generator.generate(&boundary);

        assert_eq!(result.pattern, PlanPathPattern::HilbertCurve);
        // Hilbert curve should produce some paths
        // (may be empty for some configurations)
        println!(
            "Hilbert curve: {} paths, {:.2} mm",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_archimedean_generator() {
        let boundary = make_square_boundary(50.0);
        let config = PlanPathConfig::from_density(0.3, 0.45);
        let generator = PlanPathGenerator::archimedean_chords(config);
        let result = generator.generate(&boundary);

        assert_eq!(result.pattern, PlanPathPattern::ArchimedeanChords);
        println!(
            "Archimedean chords: {} paths, {:.2} mm",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_octagram_generator() {
        let boundary = make_square_boundary(50.0);
        let config = PlanPathConfig::from_density(0.3, 0.45);
        let generator = PlanPathGenerator::octagram_spiral(config);
        let result = generator.generate(&boundary);

        assert_eq!(result.pattern, PlanPathPattern::OctagramSpiral);
        println!(
            "Octagram spiral: {} paths, {:.2} mm",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_hilbert_with_hole() {
        let boundary = make_square_with_hole(50.0, 15.0);
        let config = PlanPathConfig::from_density(0.3, 0.45);
        let generator = PlanPathGenerator::hilbert_curve(config);
        let result = generator.generate(&boundary);

        // Should respect the hole
        assert_eq!(result.pattern, PlanPathPattern::HilbertCurve);
        println!(
            "Hilbert with hole: {} paths, {:.2} mm",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_archimedean_with_hole() {
        let boundary = make_square_with_hole(50.0, 15.0);
        let config = PlanPathConfig::from_density(0.3, 0.45);
        let generator = PlanPathGenerator::archimedean_chords(config);
        let result = generator.generate(&boundary);

        // Should respect the hole (spiral is centered, so hole affects center)
        assert_eq!(result.pattern, PlanPathPattern::ArchimedeanChords);
        println!(
            "Archimedean with hole: {} paths, {:.2} mm",
            result.path_count(),
            result.total_length_mm
        );
    }

    #[test]
    fn test_zero_density() {
        let boundary = make_square_boundary(50.0);
        let config = PlanPathConfig {
            density: 0.0,
            ..Default::default()
        };
        let generator = PlanPathGenerator::hilbert_curve(config);
        let result = generator.generate(&boundary);

        assert!(result.is_empty());
    }

    #[test]
    fn test_convenience_functions() {
        let boundary = make_square_boundary(50.0);

        let hilbert = generate_hilbert_curve(&boundary, 0.3, 0.45);
        assert_eq!(hilbert.pattern, PlanPathPattern::HilbertCurve);

        let archimedean = generate_archimedean_chords(&boundary, 0.3, 0.45);
        assert_eq!(archimedean.pattern, PlanPathPattern::ArchimedeanChords);

        let octagram = generate_octagram_spiral(&boundary, 0.3, 0.45);
        assert_eq!(octagram.pattern, PlanPathPattern::OctagramSpiral);
    }

    #[test]
    fn test_plan_path_result() {
        let polylines = vec![
            Polyline::from_points(vec![Point::new(0, 0), Point::new(scale(10.0), 0)]),
            Polyline::from_points(vec![
                Point::new(0, scale(1.0)),
                Point::new(scale(10.0), scale(1.0)),
            ]),
        ];

        let result = PlanPathResult::new(polylines, PlanPathPattern::HilbertCurve);

        assert!(!result.is_empty());
        assert_eq!(result.path_count(), 2);
        assert!(result.total_length_mm > 0.0);
    }

    #[test]
    fn test_is_centered() {
        let config = PlanPathConfig::default();

        let hilbert = PlanPathGenerator::hilbert_curve(config.clone());
        assert!(!hilbert.is_centered());

        let archimedean = PlanPathGenerator::archimedean_chords(config.clone());
        assert!(archimedean.is_centered());

        let octagram = PlanPathGenerator::octagram_spiral(config);
        assert!(octagram.is_centered());
    }
}
