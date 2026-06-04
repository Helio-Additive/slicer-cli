//! Fuzzy skin implementation for adding random texture to perimeters.
//!
//! This module provides fuzzy skin functionality that adds random perturbations
//! to perimeter polygons, creating a textured surface finish on the printed object.
//!
//! # Overview
//!
//! Fuzzy skin works by:
//! 1. Iterating along each edge of a perimeter polygon
//! 2. Inserting new points at configurable intervals
//! 3. Displacing each point perpendicular to the edge by a random amount
//!
//! The result is a "fuzzy" or textured appearance on the outer surface of the print.
//!
//! # Configuration
//!
//! - `thickness`: Maximum displacement distance from the original surface (mm)
//! - `point_distance`: Average distance between points along edges (mm)
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/FuzzySkin.cpp`
//! - `src/libslic3r/FuzzySkin.hpp`

use crate::arachne::{ExtrusionJunction, ExtrusionLine};
use crate::geometry::{Point, Polygon, Polyline};
use crate::region_config::FuzzySkinMode;
use crate::{scale, Coord, CoordF};
use std::cell::RefCell;

thread_local! {
    /// Thread-local random number generator for fuzzy skin.
    /// Uses a simple xorshift algorithm for fast random number generation.
    static RNG: RefCell<XorShift64> = RefCell::new(XorShift64::new());
}

/// Simple xorshift64 random number generator.
/// Fast and adequate for fuzzy skin noise generation.
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    // Create a new RNG with a seed based on thread ID and time.
    fn new() -> Self {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        use std::thread;
        use std::time::{SystemTime, UNIX_EPOCH};

        let mut hasher = DefaultHasher::new();
        thread::current().id().hash(&mut hasher);
        let thread_hash = hasher.finish();

        let time_seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x5DEECE66D);

        // Combine thread ID and time for unique seed per thread
        let seed = thread_hash ^ time_seed;

        // Ensure non-zero seed
        Self {
            state: if seed == 0 { 0x5DEECE66D } else { seed },
        }
    }

    /// Generate a random u64.
    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Generate a random f64 in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64)
    }
}

/// Get a random value in [0, 1) using thread-local RNG.
fn random_value() -> f64 {
    RNG.with(|rng| rng.borrow_mut().next_f64())
}

/// Configuration for fuzzy skin generation.
#[derive(Debug, Clone)]
pub struct FuzzySkinConfig {
    /// Maximum thickness/displacement from original surface (mm).
    pub thickness: CoordF,

    /// Target distance between points along edges (mm).
    pub point_distance: CoordF,

    /// Fuzzy skin mode (which perimeters to fuzzify).
    pub mode: FuzzySkinMode,
}

impl Default for FuzzySkinConfig {
    fn default() -> Self {
        Self {
            thickness: 0.3,
            point_distance: 0.8,
            mode: FuzzySkinMode::External,
        }
    }
}

impl FuzzySkinConfig {
    // Create a new fuzzy skin config with specified parameters.
    pub fn new(thickness: CoordF, point_distance: CoordF) -> Self {
        Self {
            thickness,
            point_distance,
            mode: FuzzySkinMode::External,
        }
    }

    /// Builder: set fuzzy skin mode.
    pub fn with_mode(mut self, mode: FuzzySkinMode) -> Self {
        self.mode = mode;
        self
    }

    /// Check if fuzzy skin is enabled.
    pub fn is_enabled(&self) -> bool {
        self.mode != FuzzySkinMode::None && self.thickness > 0.0 && self.point_distance > 0.0
    }
}

/// Apply fuzzy skin to a polyline (open or closed).
///
/// # Arguments
///
/// * `points` - The points to modify (modified in place)
/// * `closed` - Whether the polyline is closed (polygon)
/// * `thickness` - Maximum displacement in scaled units
/// * `point_distance` - Target distance between points in scaled units
fn fuzzy_polyline_impl(
    points: &mut Vec<Point>,
    closed: bool,
    thickness: Coord,
    point_distance: Coord,
) {
    if points.len() < 2 {
        return;
    }

    // Hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
    let min_dist_between_points = (point_distance * 3) / 4;
    let range_random_point_dist = point_distance / 2;

    // The distance to be traversed on the line before making the first new point
    let mut dist_left_over =
        (random_value() * (min_dist_between_points as f64 / 2.0)).round() as Coord;

    let mut out = Vec::with_capacity(points.len() * 2);

    // Get the starting point index
    let start_idx = if closed { 0 } else { 1 };

    // Get p0 - for closed polygons, start with the last point
    let mut p0_idx = if closed { points.len() - 1 } else { 0 };

    for i in start_idx..points.len() {
        let p0 = points[p0_idx];
        let p1 = points[i];

        // Calculate vector from p0 to p1
        let p0p1_x = (p1.x - p0.x) as f64;
        let p0p1_y = (p1.y - p0.y) as f64;
        let p0p1_size = (p0p1_x * p0p1_x + p0p1_y * p0p1_y).sqrt();

        if p0p1_size < 1.0 {
            p0_idx = i;
            continue;
        }

        // Perpendicular vector (normalized)
        let perp_x = -p0p1_y / p0p1_size;
        let perp_y = p0p1_x / p0p1_size;

        // Insert new points along the edge
        let mut p0pa_dist = dist_left_over as f64;
        while p0pa_dist < p0p1_size {
            // Random displacement perpendicular to the edge
            let r = random_value() * (thickness as f64 * 2.0) - thickness as f64;

            // Calculate new point position
            let t = p0pa_dist / p0p1_size;
            let new_x = p0.x as f64 + p0p1_x * t + perp_x * r;
            let new_y = p0.y as f64 + p0p1_y * t + perp_y * r;

            out.push(Point::new(new_x.round() as Coord, new_y.round() as Coord));

            // Move to next point position with some randomness in spacing
            p0pa_dist +=
                min_dist_between_points as f64 + random_value() * range_random_point_dist as f64;
        }

        dist_left_over = (p0pa_dist - p0p1_size).round() as Coord;
        p0_idx = i;
    }

    // Ensure we have at least 3 points for a valid polygon
    while out.len() < 3 && points.len() >= 2 {
        let point_idx = if points.len() >= 2 {
            points.len() - 2
        } else {
            0
        };
        out.push(points[point_idx]);
        if point_idx == 0 {
            break;
        }
    }

    if out.len() >= 3 {
        *points = out;
    }
}

/// Apply fuzzy skin to a polygon.
///
/// # Arguments
///
/// * `polygon` - The polygon to fuzzify (modified in place)
/// * `config` - Fuzzy skin configuration
pub fn fuzzy_polygon(polygon: &mut Polygon, config: &FuzzySkinConfig) {
    let thickness = scale(config.thickness);
    let point_distance = scale(config.point_distance);
    let points = polygon.points_mut();
    fuzzy_polyline_impl(points, true, thickness, point_distance);
}

/// Apply fuzzy skin to a polygon with explicit parameters.
///
/// # Arguments
///
/// * `polygon` - The polygon to fuzzify (modified in place)
/// * `thickness_mm` - Maximum displacement from surface (mm)
/// * `point_distance_mm` - Target distance between points (mm)
pub fn fuzzy_polygon_params(
    polygon: &mut Polygon,
    thickness_mm: CoordF,
    point_distance_mm: CoordF,
) {
    let thickness = scale(thickness_mm);
    let point_distance = scale(point_distance_mm);
    let points = polygon.points_mut();
    fuzzy_polyline_impl(points, true, thickness, point_distance);
}

/// Apply fuzzy skin to a polyline (open path).
///
/// # Arguments
///
/// * `polyline` - The polyline to fuzzify (modified in place)
/// * `config` - Fuzzy skin configuration
pub fn fuzzy_polyline(polyline: &mut Polyline, config: &FuzzySkinConfig) {
    let thickness = scale(config.thickness);
    let point_distance = scale(config.point_distance);
    let points = polyline.points_mut();
    fuzzy_polyline_impl(points, false, thickness, point_distance);
}

/// Apply fuzzy skin to an Arachne extrusion line.
///
/// # Arguments
///
/// * `extrusion` - The extrusion line to fuzzify (modified in place)
/// * `config` - Fuzzy skin configuration
pub fn fuzzy_extrusion_line(extrusion: &mut ExtrusionLine, config: &FuzzySkinConfig) {
    let thickness = scale(config.thickness);
    let point_distance = scale(config.point_distance);
    fuzzy_extrusion_line_impl(extrusion, thickness, point_distance);
}

/// Apply fuzzy skin to an Arachne extrusion line with explicit parameters.
///
/// # Arguments
///
/// * `extrusion` - The extrusion line to fuzzify (modified in place)
/// * `thickness_mm` - Maximum displacement from surface (mm)
/// * `point_distance_mm` - Target distance between points (mm)
pub fn fuzzy_extrusion_line_params(
    extrusion: &mut ExtrusionLine,
    thickness_mm: CoordF,
    point_distance_mm: CoordF,
) {
    let thickness = scale(thickness_mm);
    let point_distance = scale(point_distance_mm);
    fuzzy_extrusion_line_impl(extrusion, thickness, point_distance);
}

fn fuzzy_extrusion_line_impl(
    extrusion: &mut ExtrusionLine,
    thickness: Coord,
    point_distance: Coord,
) {
    if extrusion.junctions.len() < 2 {
        return;
    }

    // Hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
    let min_dist_between_points = (point_distance * 3) / 4;
    let range_random_point_dist = point_distance / 2;

    // The distance to be traversed on the line before making the first new point
    let mut dist_left_over =
        (random_value() * (min_dist_between_points as f64 / 2.0)).round() as Coord;

    let mut out: Vec<ExtrusionJunction> = Vec::with_capacity(extrusion.junctions.len() * 2);

    let mut p0_idx = 0;
    for i in 0..extrusion.junctions.len() {
        let p0 = &extrusion.junctions[p0_idx];
        let p1 = &extrusion.junctions[i];

        if p0.position == p1.position {
            // Copy the first point
            out.push(ExtrusionJunction {
                position: p1.position,
                width: p1.width,
                perimeter_index: p1.perimeter_index,
            });
            continue;
        }

        // Calculate vector from p0 to p1
        let p0p1_x = (p1.position.x - p0.position.x) as f64;
        let p0p1_y = (p1.position.y - p0.position.y) as f64;
        let p0p1_size = (p0p1_x * p0p1_x + p0p1_y * p0p1_y).sqrt();

        // Perpendicular vector (normalized)
        let perp_x = -p0p1_y / p0p1_size;
        let perp_y = p0p1_x / p0p1_size;

        // Insert new points along the edge
        let mut p0pa_dist = dist_left_over as f64;
        while p0pa_dist < p0p1_size {
            // Random displacement perpendicular to the edge
            let r = random_value() * (thickness as f64 * 2.0) - thickness as f64;

            // Calculate new point position
            let t = p0pa_dist / p0p1_size;
            let new_x = p0.position.x as f64 + p0p1_x * t + perp_x * r;
            let new_y = p0.position.y as f64 + p0p1_y * t + perp_y * r;

            out.push(ExtrusionJunction {
                position: Point::new(new_x.round() as Coord, new_y.round() as Coord),
                width: p1.width,
                perimeter_index: p1.perimeter_index,
            });

            // Move to next point position with some randomness in spacing
            p0pa_dist +=
                min_dist_between_points as f64 + random_value() * range_random_point_dist as f64;
        }

        dist_left_over = (p0pa_dist - p0p1_size).round() as Coord;
        p0_idx = i;
    }

    // Ensure we have at least 3 points
    while out.len() < 3 && extrusion.junctions.len() >= 2 {
        let point_idx = if extrusion.junctions.len() >= 2 {
            extrusion.junctions.len() - 2
        } else {
            0
        };
        let j = &extrusion.junctions[point_idx];
        out.push(ExtrusionJunction {
            position: j.position,
            width: j.width,
            perimeter_index: j.perimeter_index,
        });
        if point_idx == 0 {
            break;
        }
    }

    // Connect endpoints for closed extrusion lines
    if extrusion.is_closed && out.len() >= 2 && extrusion.junctions.len() >= 2 {
        let first_pos = extrusion.junctions.first().map(|j| j.position);
        let last_pos = extrusion.junctions.last().map(|j| j.position);
        if first_pos == last_pos {
            if let (Some(first), Some(last)) = (out.first().cloned(), out.last().cloned()) {
                if let Some(first_mut) = out.first_mut() {
                    first_mut.position = last.position;
                }
            }
        }
    }

    if out.len() >= 3 {
        extrusion.junctions = out;
    }
}

/// Determine if a perimeter should have fuzzy skin applied.
///
/// # Arguments
///
/// * `mode` - The fuzzy skin mode setting
/// * `layer_idx` - The layer index (0-based)
/// * `perimeter_idx` - The perimeter index (0 = outermost)
/// * `is_contour` - Whether this is a contour (outer boundary) or hole (inner)
///
/// # Returns
///
/// True if fuzzy skin should be applied to this perimeter.
pub fn should_fuzzify(
    mode: FuzzySkinMode,
    layer_idx: usize,
    perimeter_idx: usize,
    is_contour: bool,
) -> bool {
    // Don't fuzzify first layer for better bed adhesion
    if layer_idx == 0 {
        return false;
    }

    match mode {
        FuzzySkinMode::None => false,
        FuzzySkinMode::External => {
            // Only external perimeters (perimeter_idx == 0) and only contours (not holes)
            perimeter_idx == 0 && is_contour
        }
        FuzzySkinMode::All => {
            // All perimeters, both contours and holes
            true
        }
    }
}

/// Apply fuzzy skin to a polygon if conditions are met.
///
/// # Arguments
///
/// * `polygon` - The polygon to potentially fuzzify
/// * `config` - Fuzzy skin configuration
/// * `layer_idx` - Layer index (0-based)
/// * `perimeter_idx` - Perimeter index (0 = outermost)
/// * `is_contour` - Whether this is a contour or hole
///
/// # Returns
///
/// A new polygon (fuzzified if applicable, or a clone if not).
pub fn apply_fuzzy_skin_polygon(
    polygon: &Polygon,
    config: &FuzzySkinConfig,
    layer_idx: usize,
    perimeter_idx: usize,
    is_contour: bool,
) -> Polygon {
    if should_fuzzify(config.mode, layer_idx, perimeter_idx, is_contour) && config.is_enabled() {
        let mut fuzzified = polygon.clone();
        fuzzy_polygon(&mut fuzzified, config);
        fuzzified
    } else {
        polygon.clone()
    }
}

/// Apply fuzzy skin to an extrusion line if conditions are met.
///
/// # Arguments
///
/// * `extrusion` - The extrusion line to potentially fuzzify
/// * `config` - Fuzzy skin configuration
/// * `layer_idx` - Layer index (0-based)
/// * `perimeter_idx` - Perimeter index (0 = outermost)
/// * `is_contour` - Whether this is a contour or hole
///
/// # Returns
///
/// A new extrusion line (fuzzified if applicable, or a clone if not).
pub fn apply_fuzzy_skin_extrusion(
    extrusion: &ExtrusionLine,
    config: &FuzzySkinConfig,
    layer_idx: usize,
    perimeter_idx: usize,
    is_contour: bool,
) -> ExtrusionLine {
    if should_fuzzify(config.mode, layer_idx, perimeter_idx, is_contour) && config.is_enabled() {
        let mut fuzzified = extrusion.clone();
        fuzzy_extrusion_line(&mut fuzzified, config);
        fuzzified
    } else {
        extrusion.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square(size_mm: CoordF) -> Polygon {
        let s = scale(size_mm);
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(s, 0),
            Point::new(s, s),
            Point::new(0, s),
        ])
    }

    fn make_line() -> Polyline {
        let s = scale(10.0);
        Polyline::from_points(vec![Point::new(0, 0), Point::new(s, 0)])
    }

    #[test]
    fn test_fuzzy_skin_config_default() {
        let config = FuzzySkinConfig::default();
        assert!((config.thickness - 0.3).abs() < 1e-6);
        assert!((config.point_distance - 0.8).abs() < 1e-6);
        assert_eq!(config.mode, FuzzySkinMode::External);
        assert!(config.is_enabled());
    }

    #[test]
    fn test_fuzzy_skin_config_disabled() {
        let config = FuzzySkinConfig::default().with_mode(FuzzySkinMode::None);
        assert!(!config.is_enabled());
    }

    #[test]
    fn test_fuzzy_polygon_adds_points() {
        let original = make_square(10.0);
        let original_count = original.points().len();

        let mut fuzzified = original.clone();
        let config = FuzzySkinConfig::new(0.3, 0.8);
        fuzzy_polygon(&mut fuzzified, &config);

        // Fuzzy skin should add more points
        assert!(fuzzified.points().len() > original_count);
    }

    #[test]
    fn test_fuzzy_polygon_displacement() {
        let original = make_square(10.0);
        let mut fuzzified = original.clone();
        let config = FuzzySkinConfig::new(0.3, 0.8);
        fuzzy_polygon(&mut fuzzified, &config);

        // Points should be displaced but within thickness bounds
        // This is a weak test - just checking the polygon is still valid
        assert!(fuzzified.points().len() >= 3);
    }

    #[test]
    fn test_fuzzy_polyline() {
        let original = make_line();
        let original_count = original.points().len();

        let mut fuzzified = original.clone();
        let config = FuzzySkinConfig::new(0.3, 0.8);
        fuzzy_polyline(&mut fuzzified, &config);

        // Should add points (10mm line with 0.8mm spacing should have ~12 points)
        assert!(fuzzified.points().len() > original_count);
    }

    #[test]
    fn test_should_fuzzify_first_layer() {
        // First layer should never be fuzzified
        assert!(!should_fuzzify(FuzzySkinMode::External, 0, 0, true));
        assert!(!should_fuzzify(FuzzySkinMode::All, 0, 0, true));
    }

    #[test]
    fn test_should_fuzzify_external_mode() {
        // External mode: only outer perimeter contours
        assert!(should_fuzzify(FuzzySkinMode::External, 1, 0, true));
        assert!(!should_fuzzify(FuzzySkinMode::External, 1, 0, false)); // hole
        assert!(!should_fuzzify(FuzzySkinMode::External, 1, 1, true)); // inner perimeter
    }

    #[test]
    fn test_should_fuzzify_all_mode() {
        // All mode: everything (except first layer)
        assert!(should_fuzzify(FuzzySkinMode::All, 1, 0, true));
        assert!(should_fuzzify(FuzzySkinMode::All, 1, 0, false));
        assert!(should_fuzzify(FuzzySkinMode::All, 1, 1, true));
        assert!(should_fuzzify(FuzzySkinMode::All, 1, 2, false));
    }

    #[test]
    fn test_should_fuzzify_none_mode() {
        // None mode: nothing
        assert!(!should_fuzzify(FuzzySkinMode::None, 1, 0, true));
        assert!(!should_fuzzify(FuzzySkinMode::None, 5, 0, true));
    }

    #[test]
    fn test_apply_fuzzy_skin_polygon() {
        let original = make_square(10.0);
        let config = FuzzySkinConfig::default();

        // Layer 0 should not be fuzzified
        let result = apply_fuzzy_skin_polygon(&original, &config, 0, 0, true);
        assert_eq!(result.points().len(), original.points().len());

        // Layer 1 external contour should be fuzzified
        let result = apply_fuzzy_skin_polygon(&original, &config, 1, 0, true);
        assert!(result.points().len() > original.points().len());

        // Layer 1 inner perimeter should not be fuzzified (External mode)
        let result = apply_fuzzy_skin_polygon(&original, &config, 1, 1, true);
        assert_eq!(result.points().len(), original.points().len());
    }

    #[test]
    fn test_fuzzy_extrusion_line() {
        let original = ExtrusionLine {
            junctions: vec![
                ExtrusionJunction {
                    position: Point::new(0, 0),
                    width: scale(0.4),
                    perimeter_index: 0,
                },
                ExtrusionJunction {
                    position: Point::new(scale(10.0), 0),
                    width: scale(0.4),
                    perimeter_index: 0,
                },
            ],
            inset_idx: 0,
            is_odd: false,
            is_closed: false,
        };

        let mut fuzzified = original.clone();
        let config = FuzzySkinConfig::new(0.3, 0.8);
        fuzzy_extrusion_line(&mut fuzzified, &config);

        // Should add more junctions
        assert!(fuzzified.junctions.len() > original.junctions.len());
    }

    #[test]
    fn test_random_value_range() {
        // Test that random values are in [0, 1)
        for _ in 0..100 {
            let v = random_value();
            assert!(v >= 0.0 && v < 1.0);
        }
    }

    #[test]
    fn test_xorshift_not_zero() {
        let mut rng = XorShift64::new();
        // Ensure it produces non-zero values
        let mut seen_nonzero = false;
        for _ in 0..10 {
            if rng.next_u64() != 0 {
                seen_nonzero = true;
                break;
            }
        }
        assert!(seen_nonzero);
    }

    #[test]
    fn test_fuzzy_preserves_minimum_points() {
        // A very small polygon should still have at least 3 points
        let tiny = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(50, 100),
        ]);
        let mut fuzzified = tiny.clone();
        let config = FuzzySkinConfig::new(0.001, 10.0); // Large point distance
        fuzzy_polygon(&mut fuzzified, &config);

        assert!(fuzzified.points().len() >= 3);
    }
}
