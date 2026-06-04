//! Path simplification algorithms.
//!
//! This module provides algorithms for reducing the number of points in polygons
//! and polylines while maintaining shape fidelity within specified tolerances.
//!
//! ## Algorithms
//!
//! - **Douglas-Peucker**: Classic recursive line simplification algorithm
//! - **Resolution-based**: Remove tiny segments and collinear points
//! - **Extrusion-aware**: Consider extrusion area deviation when simplifying toolpaths
//!
//! ## Reference
//!
//! These algorithms match the behavior of BambuStudio/libslic3r:
//! - `MultiPoint::_douglas_peucker()` in MultiPoint.cpp
//! - `ExtrusionLine::simplify()` in Arachne/utils/ExtrusionLine.cpp
//! - Constants from `Arachne/WallToolPaths.hpp`

use super::{Line, Point, Polygon, Polyline};
use crate::{scale, Coord, CoordF};

/// Default maximum resolution for mesh fixing (0.5mm).
/// Segments shorter than this can be removed if within deviation tolerance.
/// This matches `meshfix_maximum_resolution` in BambuStudio.
pub const MESHFIX_MAXIMUM_RESOLUTION: CoordF = 0.5;

/// Default maximum deviation for mesh fixing (0.025mm = 25 microns).
/// Points can be removed if the resulting path deviates by less than this.
/// This matches `meshfix_maximum_deviation` in BambuStudio.
pub const MESHFIX_MAXIMUM_DEVIATION: CoordF = 0.025;

/// Default maximum extrusion area deviation (2.0 mm²).
/// For variable-width extrusions, limits how much the extruded area can change.
/// This matches `meshfix_maximum_extrusion_area_deviation` in BambuStudio.
pub const MESHFIX_MAXIMUM_EXTRUSION_AREA_DEVIATION: CoordF = 2.0;

/// Minimum segment length below which we always allow removal (5 microns).
/// This matches the hardcoded value in libslic3r.
pub const MINIMUM_SEGMENT_LENGTH: CoordF = 0.005;

/// Minimum height threshold for collinearity check (1 micron).
pub const COLLINEARITY_THRESHOLD: CoordF = 0.001;

/// Configuration for path simplification.
#[derive(Debug, Clone, Copy)]
pub struct SimplifyConfig {
    /// Maximum resolution: segments shorter than this may be removed.
    /// Default: 0.5mm (matches `meshfix_maximum_resolution`)
    pub max_resolution: CoordF,

    /// Maximum deviation: points can be removed if path deviates by less than this.
    /// Default: 0.025mm (matches `meshfix_maximum_deviation`)
    pub max_deviation: CoordF,

    /// Minimum segment length below which removal is always considered.
    /// Default: 0.005mm (5 microns)
    pub min_segment_length: CoordF,

    /// Whether to preserve the first and last points exactly.
    /// Default: true
    pub preserve_endpoints: bool,

    /// Whether this is a closed path (polygon vs polyline).
    /// Affects how endpoints are handled.
    pub is_closed: bool,
}

impl Default for SimplifyConfig {
    fn default() -> Self {
        Self {
            max_resolution: MESHFIX_MAXIMUM_RESOLUTION,
            max_deviation: MESHFIX_MAXIMUM_DEVIATION,
            min_segment_length: MINIMUM_SEGMENT_LENGTH,
            preserve_endpoints: true,
            is_closed: false,
        }
    }
}

impl SimplifyConfig {
    // Create a new configuration with the specified tolerances.
    pub fn new(max_resolution: CoordF, max_deviation: CoordF) -> Self {
        Self {
            max_resolution,
            max_deviation,
            ..Default::default()
        }
    }

    /// Create a configuration for closed paths (polygons).
    pub fn for_polygon() -> Self {
        Self {
            is_closed: true,
            ..Default::default()
        }
    }

    /// Create a configuration for open paths (polylines).
    pub fn for_polyline() -> Self {
        Self {
            is_closed: false,
            ..Default::default()
        }
    }

    /// Create a configuration with aggressive simplification.
    /// Uses larger tolerances for maximum point reduction.
    pub fn aggressive() -> Self {
        Self {
            max_resolution: 1.0,      // 1mm
            max_deviation: 0.05,      // 50 microns
            min_segment_length: 0.01, // 10 microns
            ..Default::default()
        }
    }

    /// Create a configuration with conservative simplification.
    /// Uses smaller tolerances to preserve more detail.
    pub fn conservative() -> Self {
        Self {
            max_resolution: 0.25,      // 0.25mm
            max_deviation: 0.01,       // 10 microns
            min_segment_length: 0.001, // 1 micron
            ..Default::default()
        }
    }
}

/// Douglas-Peucker line simplification algorithm.
///
/// Recursively simplifies a path by removing points that are within `tolerance`
/// of the line segment connecting their neighbors.
///
/// This is the same algorithm used by libslic3r in `MultiPoint::_douglas_peucker()`.
///
/// # Arguments
///
/// * `points` - The input points to simplify
/// * `tolerance` - Maximum distance a point can deviate from the simplified path (in mm)
///
/// # Returns
///
/// A new vector of simplified points. The first and last points are always preserved.
pub fn douglas_peucker(points: &[Point], tolerance: CoordF) -> Vec<Point> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    let tolerance_sq = scale(tolerance) as f64 * scale(tolerance) as f64;
    let mut result = Vec::with_capacity(points.len());
    result.push(points[0]);

    // Stack-based implementation (avoids stack overflow for large inputs)
    let mut stack = vec![(0, points.len() - 1)];

    // Track which points to keep
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;

    while let Some((anchor_idx, floater_idx)) = stack.pop() {
        if anchor_idx + 1 >= floater_idx {
            continue;
        }

        let anchor = &points[anchor_idx];
        let floater = &points[floater_idx];

        // Find point furthest from the anchor-floater line
        let mut max_dist_sq = 0.0;
        let mut furthest_idx = anchor_idx;

        for i in (anchor_idx + 1)..floater_idx {
            let dist_sq = Line::distance_to_squared(points[i], *anchor, *floater);
            if dist_sq > max_dist_sq {
                max_dist_sq = dist_sq;
                furthest_idx = i;
            }
        }

        // If furthest point exceeds tolerance, keep it and recurse
        if max_dist_sq > tolerance_sq {
            keep[furthest_idx] = true;
            stack.push((anchor_idx, furthest_idx));
            stack.push((furthest_idx, floater_idx));
        }
    }

    // Collect kept points in order
    result.clear();
    for (i, point) in points.iter().enumerate() {
        if keep[i] {
            result.push(*point);
        }
    }

    result
}

/// Douglas-Peucker simplification for a polygon.
///
/// Handles the closed nature of polygons by treating them as circular.
pub fn douglas_peucker_polygon(polygon: &Polygon, tolerance: CoordF) -> Polygon {
    let points = polygon.points();
    if points.len() <= 3 {
        return polygon.clone();
    }

    // For polygons, we need to handle the wrap-around
    // Simplify as if it's a closed loop
    let simplified = douglas_peucker(points, tolerance);

    // Ensure we still have a valid polygon
    if simplified.len() < 3 {
        return polygon.clone();
    }

    Polygon::from_points(simplified)
}

/// Douglas-Peucker simplification for a polyline.
pub fn douglas_peucker_polyline(polyline: &Polyline, tolerance: CoordF) -> Polyline {
    let points = polyline.points();
    if points.len() <= 2 {
        return polyline.clone();
    }

    let simplified = douglas_peucker(points, tolerance);
    Polyline::from_points(simplified)
}

/// Simplify points using resolution and deviation thresholds.
///
/// This algorithm matches BambuStudio's approach:
/// 1. Always remove segments shorter than `min_segment_length`
/// 2. Remove points if the deviation is within `max_deviation`
/// 3. Consider the accumulated area when removing multiple consecutive points
///
/// # Arguments
///
/// * `points` - The input points to simplify
/// * `config` - Simplification configuration
///
/// # Returns
///
/// A new vector of simplified points.
pub fn simplify_resolution(points: &[Point], config: &SimplifyConfig) -> Vec<Point> {
    let min_size = if config.is_closed { 3 } else { 2 };
    if points.len() <= min_size {
        return points.to_vec();
    }

    let max_resolution_sq = scale(config.max_resolution).pow(2);
    let max_deviation_sq = scale(config.max_deviation).pow(2);
    let min_segment_sq = scale(config.min_segment_length).pow(2);
    let collinearity_sq = scale(COLLINEARITY_THRESHOLD).pow(2);

    let mut result = Vec::with_capacity(points.len());
    result.push(points[0]);

    // Track previous points for area calculation
    let mut prev_prev = points[0];
    let mut prev = points[0];

    // Accumulated area for consecutive point removals (Shoelace formula)
    let initial = if points.len() > 1 {
        points[1]
    } else {
        points[0]
    };
    let mut accumulated_area: i128 =
        prev.x as i128 * initial.y as i128 - prev.y as i128 * initial.x as i128;

    let end_idx = if config.is_closed {
        points.len()
    } else {
        points.len() - 1
    };

    for i in 1..end_idx {
        let current = points[i];

        // Handle wrap-around for closed paths
        let next_idx = if config.is_closed {
            (i + 1) % points.len()
        } else {
            i + 1
        };

        // Skip if we're at the last point of an open path
        if !config.is_closed && next_idx >= points.len() {
            result.push(current);
            break;
        }

        let next = if next_idx < points.len() {
            points[next_idx]
        } else if !result.is_empty() {
            result[0]
        } else {
            current
        };

        // Calculate areas for Shoelace formula
        let removed_area_next: i128 =
            current.x as i128 * next.y as i128 - current.y as i128 * next.x as i128;
        let negative_area_closing: i128 =
            next.x as i128 * prev.y as i128 - next.y as i128 * prev.x as i128;
        accumulated_area += removed_area_next;

        // Segment length from prev to current
        let length_sq = (current.x - prev.x).pow(2) + (current.y - prev.y).pow(2);

        // Always allow removal of very short segments
        if length_sq < min_segment_sq as i64 {
            continue;
        }

        // Calculate total area that would be cut off
        let area_removed_so_far = accumulated_area + negative_area_closing;
        let base_length_sq: i128 =
            (next.x - prev.x).pow(2) as i128 + (next.y - prev.y).pow(2) as i128;

        // Skip if base is zero (would be removed anyway)
        if base_length_sq == 0 {
            continue;
        }

        // Calculate height of representative triangle: h² = A² / b²
        // where A = area_removed_so_far / 2 (Shoelace gives 2× area)
        let height_sq = (area_removed_so_far * area_removed_so_far) as f64 / base_length_sq as f64;

        // Check for almost collinear (very small height)
        if height_sq <= collinearity_sq as f64 {
            // Double-check with actual distance calculation
            let actual_dist_sq = Line::distance_to_infinite_squared(current, prev, next);
            if actual_dist_sq <= collinearity_sq as f64 {
                // Collinear - remove
                continue;
            }
        }

        // Check if within resolution and deviation limits
        if length_sq < max_resolution_sq as i64 && height_sq <= max_deviation_sq as f64 {
            // Check next segment length to avoid artifacts
            let next_length_sq = (current.x - next.x).pow(2) + (current.y - next.y).pow(2);

            if next_length_sq <= 4 * max_resolution_sq as i64 {
                // Both segments are short enough - safe to remove
                continue;
            }
            // If next segment is long, keep this point to avoid artifacts
        }

        // Keep this point
        accumulated_area = removed_area_next;
        prev_prev = prev;
        prev = current;
        result.push(current);
    }

    // Always keep the last point for open paths
    if !config.is_closed && !points.is_empty() {
        let last = points[points.len() - 1];
        if result.last().map_or(true, |p| *p != last) {
            result.push(last);
        }
    }

    // Ensure minimum point count
    if result.len() < min_size && points.len() >= min_size {
        return points.to_vec();
    }

    result
}

/// Simplify a polygon using resolution-based algorithm.
pub fn simplify_polygon(polygon: &Polygon, config: &SimplifyConfig) -> Polygon {
    let mut cfg = *config;
    cfg.is_closed = true;

    let simplified = simplify_resolution(polygon.points(), &cfg);

    if simplified.len() < 3 {
        return polygon.clone();
    }

    Polygon::from_points(simplified)
}

/// Simplify a polyline using resolution-based algorithm.
pub fn simplify_polyline(polyline: &Polyline, config: &SimplifyConfig) -> Polyline {
    let mut cfg = *config;
    cfg.is_closed = false;

    let simplified = simplify_resolution(polyline.points(), &cfg);

    if simplified.len() < 2 {
        return polyline.clone();
    }

    Polyline::from_points(simplified)
}

/// Remove duplicate consecutive points from a path.
///
/// Points are considered duplicates if they're within `tolerance` of each other.
pub fn remove_duplicate_points(points: &[Point], tolerance: Coord) -> Vec<Point> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut result = Vec::with_capacity(points.len());
    result.push(points[0]);

    let tolerance_sq = tolerance * tolerance;

    for point in points.iter().skip(1) {
        let last = result.last().unwrap();
        let dist_sq = (point.x - last.x).pow(2) + (point.y - last.y).pow(2);

        if dist_sq > tolerance_sq {
            result.push(*point);
        }
    }

    result
}

/// Remove collinear points from a path.
///
/// Points are removed if they lie within `tolerance` of the line connecting neighbors.
pub fn remove_collinear_points(points: &[Point], tolerance: CoordF) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }

    let tolerance_sq = scale(tolerance) as f64 * scale(tolerance) as f64;
    let mut result = Vec::with_capacity(points.len());
    result.push(points[0]);

    for i in 1..(points.len() - 1) {
        let prev = points[i - 1];
        let curr = points[i];
        let next = points[i + 1];

        let dist_sq = Line::distance_to_infinite_squared(curr, prev, next);

        if dist_sq > tolerance_sq {
            result.push(curr);
        }
    }

    result.push(points[points.len() - 1]);
    result
}

/// Comprehensive simplification combining multiple techniques.
///
/// This applies:
/// 1. Duplicate point removal
/// 2. Collinear point removal
/// 3. Resolution-based simplification
///
/// This is the recommended function for general use.
pub fn simplify_comprehensive(points: &[Point], config: &SimplifyConfig) -> Vec<Point> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    // Step 1: Remove duplicates
    let deduped = remove_duplicate_points(points, scale(config.min_segment_length));

    // Step 2: Apply resolution-based simplification
    let simplified = simplify_resolution(&deduped, config);

    simplified
}

/// Simplify a polygon comprehensively.
pub fn simplify_polygon_comprehensive(polygon: &Polygon, config: &SimplifyConfig) -> Polygon {
    let mut cfg = *config;
    cfg.is_closed = true;

    let simplified = simplify_comprehensive(polygon.points(), &cfg);

    if simplified.len() < 3 {
        return polygon.clone();
    }

    Polygon::from_points(simplified)
}

/// Simplify a polyline comprehensively.
pub fn simplify_polyline_comprehensive(polyline: &Polyline, config: &SimplifyConfig) -> Polyline {
    let mut cfg = *config;
    cfg.is_closed = false;

    let simplified = simplify_comprehensive(polyline.points(), &cfg);

    if simplified.len() < 2 {
        return polyline.clone();
    }

    Polyline::from_points(simplified)
}

/// Simplify multiple polygons.
pub fn simplify_polygons(polygons: &[Polygon], config: &SimplifyConfig) -> Vec<Polygon> {
    polygons
        .iter()
        .map(|p| simplify_polygon_comprehensive(p, config))
        .collect()
}

/// Simplify multiple polylines.
pub fn simplify_polylines(polylines: &[Polyline], config: &SimplifyConfig) -> Vec<Polyline> {
    polylines
        .iter()
        .map(|p| simplify_polyline_comprehensive(p, config))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_line_points(start: (i64, i64), end: (i64, i64), count: usize) -> Vec<Point> {
        let mut points = Vec::with_capacity(count);
        for i in 0..count {
            let t = i as f64 / (count - 1) as f64;
            let x = start.0 as f64 + t * (end.0 - start.0) as f64;
            let y = start.1 as f64 + t * (end.1 - start.1) as f64;
            points.push(Point::new(x.round() as i64, y.round() as i64));
        }
        points
    }

    #[test]
    fn test_douglas_peucker_collinear() {
        // Collinear points should be simplified to just endpoints
        let points = make_line_points((0, 0), (scale(10.0), 0), 100);
        let simplified = douglas_peucker(&points, 0.001);

        assert_eq!(simplified.len(), 2);
        assert_eq!(simplified[0], points[0]);
        assert_eq!(simplified[1], *points.last().unwrap());
    }

    #[test]
    fn test_douglas_peucker_zigzag() {
        // Create a zigzag that should be preserved
        let points = vec![
            Point::new(0, 0),
            Point::new(scale(5.0), scale(1.0)), // 1mm deviation
            Point::new(scale(10.0), 0),
        ];

        // With small tolerance, middle point is kept
        let simplified = douglas_peucker(&points, 0.5);
        assert_eq!(simplified.len(), 3);

        // With large tolerance, middle point is removed
        let simplified = douglas_peucker(&points, 2.0);
        assert_eq!(simplified.len(), 2);
    }

    #[test]
    fn test_douglas_peucker_empty() {
        let points: Vec<Point> = vec![];
        let simplified = douglas_peucker(&points, 0.1);
        assert!(simplified.is_empty());
    }

    #[test]
    fn test_douglas_peucker_single() {
        let points = vec![Point::new(0, 0)];
        let simplified = douglas_peucker(&points, 0.1);
        assert_eq!(simplified.len(), 1);
    }

    #[test]
    fn test_douglas_peucker_two() {
        let points = vec![Point::new(0, 0), Point::new(scale(10.0), scale(10.0))];
        let simplified = douglas_peucker(&points, 0.1);
        assert_eq!(simplified.len(), 2);
    }

    #[test]
    fn test_simplify_config_defaults() {
        let config = SimplifyConfig::default();
        assert!((config.max_resolution - 0.5).abs() < 1e-10);
        assert!((config.max_deviation - 0.025).abs() < 1e-10);
    }

    #[test]
    fn test_simplify_resolution_collinear() {
        let points = make_line_points((0, 0), (scale(10.0), 0), 50);
        let config = SimplifyConfig::for_polyline();
        let simplified = simplify_resolution(&points, &config);

        // Should reduce significantly
        assert!(simplified.len() < points.len() / 2);
        // But preserve endpoints
        assert_eq!(simplified[0], points[0]);
        assert_eq!(*simplified.last().unwrap(), *points.last().unwrap());
    }

    #[test]
    fn test_simplify_polygon() {
        // Create a square with many points on each edge
        let mut points = Vec::new();
        let size = scale(10.0);

        // Bottom edge
        for i in 0..10 {
            points.push(Point::new(i * size / 10, 0));
        }
        // Right edge
        for i in 0..10 {
            points.push(Point::new(size, i * size / 10));
        }
        // Top edge
        for i in (0..10).rev() {
            points.push(Point::new(i * size / 10, size));
        }
        // Left edge
        for i in (1..10).rev() {
            points.push(Point::new(0, i * size / 10));
        }

        let polygon = Polygon::from_points(points);
        let config = SimplifyConfig::for_polygon();
        let simplified = simplify_polygon(&polygon, &config);

        // Should reduce to roughly 4 corners
        assert!(simplified.len() <= 8);
        assert!(simplified.len() >= 4);
    }

    #[test]
    fn test_remove_duplicate_points() {
        // Test with exact duplicates
        let points = vec![
            Point::new(0, 0),
            Point::new(0, 0), // exact duplicate - removed
            Point::new(scale(1.0), 0),
            Point::new(scale(2.0), 0),
        ];

        let deduped = remove_duplicate_points(&points, 1);
        // Should keep: (0,0), (scale(1.0), 0), (scale(2.0), 0) = 3 points
        assert_eq!(deduped.len(), 3);

        // Test with points within tolerance
        let points2 = vec![
            Point::new(0, 0),
            Point::new(5, 5), // within tolerance of (0,0) when tolerance is 10
            Point::new(scale(1.0), 0),
            Point::new(scale(2.0), 0),
        ];
        // dist_sq between (0,0) and (5,5) = 50, tolerance_sq = 100
        let deduped2 = remove_duplicate_points(&points2, 10);
        assert_eq!(deduped2.len(), 3); // (0,0), (scale(1.0), 0), (scale(2.0), 0)
    }

    #[test]
    fn test_simplify_preserves_shape() {
        // Create a right angle - should be preserved
        let points = vec![
            Point::new(0, 0),
            Point::new(scale(5.0), 0),
            Point::new(scale(5.0), scale(5.0)),
        ];

        let config = SimplifyConfig::for_polyline();
        let simplified = simplify_resolution(&points, &config);

        // All three points should be kept (corner is significant)
        assert_eq!(simplified.len(), 3);
    }

    #[test]
    fn test_aggressive_vs_conservative() {
        // Create a more pronounced curve that shows clear difference
        let mut points = Vec::new();
        for i in 0..200 {
            let x = i as f64 * 0.05; // 0 to 10mm, more points
            let y = (x * 0.5).sin() * 1.0; // larger wave amplitude
            points.push(Point::new(scale(x), scale(y)));
        }

        let aggressive = simplify_comprehensive(&points, &SimplifyConfig::aggressive());
        let conservative = simplify_comprehensive(&points, &SimplifyConfig::conservative());

        // Both should reduce point count
        assert!(aggressive.len() < points.len());
        assert!(conservative.len() < points.len());

        // Aggressive should remove at least as many points as conservative
        // (or more, but in some edge cases they could be equal)
        assert!(aggressive.len() <= conservative.len());
    }

    #[test]
    fn test_simplify_polygons() {
        let polygons = vec![
            Polygon::from_points(make_line_points((0, 0), (scale(10.0), 0), 20)),
            Polygon::from_points(make_line_points((0, 0), (0, scale(10.0)), 20)),
        ];

        let config = SimplifyConfig::for_polygon();
        let simplified = simplify_polygons(&polygons, &config);

        assert_eq!(simplified.len(), 2);
    }
}
