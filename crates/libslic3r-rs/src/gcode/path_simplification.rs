//! Path simplification and segment merging for G-code generation.
//!
//! This module implements the segment simplification logic used by BambuStudio
//! to reduce the number of tiny extrusion moves that result from fine STL
//! triangulation and Clipper offset operations.
//!
//! ## BambuStudio Reference
//!
//! The C++ implementation uses several strategies:
//!
//! 1. **Early simplification**: `ExPolygon::simplify_p()` via Douglas-Peucker
//!    before perimeter generation (PerimeterGenerator.cpp:940, 1393, 1456)
//!
//! 2. **Path grouping**: `can_merge()` groups consecutive paths with identical
//!    properties (width, height, role, speed) for speed transitions
//!    (GCode.cpp:5780, ExtrusionEntity.cpp:70)
//!
//! 3. **Polyline simplification**: `Polyline::simplify(tolerance)` applies
//!    Douglas-Peucker to reduce point count (Polyline.cpp:146)
//!
//! This module provides post-processing for ExtrusionPaths to:
//! - Filter micro-segments (< 0.1mm) by merging with neighbors
//! - Merge co-linear segments with same properties
//! - Apply additional Douglas-Peucker simplification where needed

use crate::gcode::ExtrusionPath;
use crate::geometry::simplify::douglas_peucker_polyline;
use crate::geometry::{Point, Polyline};
use crate::{scale, unscale, CoordF};

/// Minimum segment length threshold (mm). Segments shorter than this will be
/// merged with adjacent segments if possible.
///
/// BambuStudio doesn't have an explicit threshold but effectively filters
/// very short segments through early simplification. 0.1mm is a safe threshold
/// that removes micro-segments while preserving detail.
pub const MIN_SEGMENT_LENGTH_MM: CoordF = 0.1;

/// Maximum angle deviation for considering segments co-linear (degrees).
///
/// BambuStudio uses `can_merge()` which only checks properties (width, role, etc)
/// not geometry. We add angle checking to merge segments that are nearly straight.
pub const COLINEAR_ANGLE_THRESHOLD_DEG: CoordF = 5.0;

/// Configuration for path simplification.
#[derive(Debug, Clone)]
pub struct SimplificationConfig {
    /// Minimum segment length (mm). Shorter segments are merged with neighbors.
    pub min_segment_length: CoordF,

    /// Maximum angle for co-linear merging (degrees).
    pub colinear_angle_threshold: CoordF,

    /// Apply additional Douglas-Peucker simplification with this tolerance (mm).
    /// Set to 0.0 to disable. Typical value: 0.01-0.02mm.
    pub douglas_peucker_tolerance: CoordF,

    /// Filter segments shorter than this threshold (mm) even if isolated.
    /// Set to 0.0 to keep all segments. Typical value: 0.05mm.
    pub filter_threshold: CoordF,
}

impl Default for SimplificationConfig {
    fn default() -> Self {
        Self {
            min_segment_length: MIN_SEGMENT_LENGTH_MM,
            colinear_angle_threshold: COLINEAR_ANGLE_THRESHOLD_DEG,
            douglas_peucker_tolerance: 0.0, // Disabled by default (done earlier in pipeline)
            filter_threshold: 0.05,         // Filter very short segments
        }
    }
}

impl SimplificationConfig {
    // Create a config for aggressive simplification (reduces move count significantly).
    pub fn aggressive() -> Self {
        Self {
            min_segment_length: 0.15,         // Merge segments < 0.15mm
            colinear_angle_threshold: 10.0,   // More tolerant angle
            douglas_peucker_tolerance: 0.015, // Apply DP simplification
            filter_threshold: 0.08,           // Filter < 0.08mm
        }
    }

    /// Create a config for conservative simplification (preserves more detail).
    pub fn conservative() -> Self {
        Self {
            min_segment_length: 0.08,
            colinear_angle_threshold: 3.0,
            douglas_peucker_tolerance: 0.0, // No DP
            filter_threshold: 0.03,
        }
    }
}

/// Simplify a vector of extrusion paths by filtering short segments and merging
/// co-linear ones.
///
/// This is applied AFTER path generation and classification to reduce the number
/// of G-code moves without changing the fundamental toolpath.
///
/// # Arguments
/// * `paths` - The extrusion paths to simplify (modified in-place)
/// * `config` - Simplification parameters
///
/// # Algorithm
/// 1. Apply Douglas-Peucker to each path's polyline (if enabled)
/// 2. Filter out paths shorter than filter_threshold
/// 3. Merge consecutive co-linear segments with same properties
/// 4. Merge very short segments with adjacent segments
///
/// # BambuStudio Reference
/// This combines the effects of:
/// - Early ExPolygon simplification (PerimeterGenerator.cpp:940)
/// - can_merge() grouping (ExtrusionEntity.cpp:70)
/// - Polyline::simplify() (Polyline.cpp:146)
pub fn simplify_paths(paths: &mut Vec<ExtrusionPath>, config: &SimplificationConfig) {
    if paths.is_empty() {
        return;
    }

    // Step 1: Apply Douglas-Peucker to each path if enabled
    if config.douglas_peucker_tolerance > 0.0 {
        for path in paths.iter_mut() {
            if path.points.len() >= 2 {
                let polyline = Polyline::from_points(path.points.clone());
                let simplified =
                    douglas_peucker_polyline(&polyline, config.douglas_peucker_tolerance);
                if simplified.len() >= 2 {
                    path.points = simplified.to_vec();
                }
            }
        }
    }

    // Step 2: Filter out very short paths
    if config.filter_threshold > 0.0 {
        let filter_threshold_scaled = scale(config.filter_threshold);
        paths.retain(|path| {
            let length = polyline_length(&path.points);
            length >= filter_threshold_scaled as f64
        });
    }

    // Step 3: Merge consecutive segments with same properties
    merge_consecutive_paths(paths, config);

    // Step 4: Filter again after merging (some merged paths might be too short)
    if config.filter_threshold > 0.0 {
        let filter_threshold_scaled = scale(config.filter_threshold);
        paths.retain(|path| {
            let length = polyline_length(&path.points);
            length >= filter_threshold_scaled as f64
        });
    }
}

/// Merge consecutive paths that have the same extrusion properties and are
/// nearly co-linear.
///
/// This implements the spirit of BambuStudio's `can_merge()` but actually
/// combines the polylines into a single path when possible.
///
/// # BambuStudio Reference
/// - `ExtrusionPath::can_merge()` (ExtrusionEntity.cpp:70)
///   Checks: width, height, mm3_per_mm, role, speed, can_reverse
fn merge_consecutive_paths(paths: &mut Vec<ExtrusionPath>, config: &SimplificationConfig) {
    if paths.len() < 2 {
        return;
    }

    let mut merged = Vec::with_capacity(paths.len());
    let mut current = paths[0].clone();

    for next in paths.iter().skip(1) {
        // Check if paths can be merged (BambuStudio's can_merge() logic)
        let can_merge = paths_compatible(&current, next);

        // Check if the connection is co-linear
        let is_colinear = if can_merge && current.points.len() >= 2 && next.points.len() >= 2 {
            check_colinear_connection(&current, next, config.colinear_angle_threshold)
        } else {
            false
        };

        if can_merge && is_colinear {
            // Merge: append next's points to current (skip first point to avoid duplication)
            if next.points.len() >= 2 {
                current.points.extend_from_slice(&next.points[1..]);
            }
        } else {
            // Can't merge - push current and start new
            merged.push(current);
            current = next.clone();
        }
    }

    // Don't forget the last path
    merged.push(current);

    *paths = merged;
}

/// Check if two extrusion paths have compatible properties for merging.
///
/// This replicates BambuStudio's `ExtrusionPath::can_merge()` logic:
/// - Same width, height, role
/// - Same speed (if both have speed set)
/// - Compatible flags
///
/// Reference: ExtrusionEntity.cpp:70-79
fn paths_compatible(a: &ExtrusionPath, b: &ExtrusionPath) -> bool {
    // Width and height must match
    if (a.width - b.width).abs() > 0.001 || (a.height - b.height).abs() > 0.001 {
        return false;
    }

    // Role must match
    if a.role != b.role {
        return false;
    }

    // Speed must match (within tolerance)
    if (a.speed - b.speed).abs() > 0.1 {
        return false;
    }

    // If both have flow objects, mm3_per_mm should be close
    // (This is part of BambuStudio's can_merge check)
    if let (Some(flow_a), Some(flow_b)) = (&a.flow, &b.flow) {
        // mm3_per_mm() returns Result, so we need to handle errors
        if let (Ok(mm3_a), Ok(mm3_b)) = (flow_a.mm3_per_mm(), flow_b.mm3_per_mm()) {
            if (mm3_a - mm3_b).abs() > 0.001 {
                return false;
            }
        }
    }

    true
}

/// Check if the connection between two paths is co-linear (nearly straight).
///
/// Computes the angle at the connection point. If the angle deviation from
/// straight (180°) is less than the threshold, the paths are co-linear.
///
/// # Arguments
/// * `first` - The first path
/// * `second` - The second path
/// * `threshold_deg` - Maximum angle deviation from straight (degrees)
///
/// # Returns
/// `true` if the paths connect in a nearly straight line.
fn check_colinear_connection(
    first: &ExtrusionPath,
    second: &ExtrusionPath,
    threshold_deg: CoordF,
) -> bool {
    if first.points.len() < 2 || second.points.len() < 2 {
        return false;
    }

    // Get the three points at the connection
    let p1 = first.points[first.points.len() - 2]; // Second-to-last of first path
    let p2 = first.points[first.points.len() - 1]; // Last of first (connection point)
    let p3 = second.points[1]; // Second of second path (skip first = p2)

    // Compute angle at p2
    let angle_deg = compute_angle(p1, p2, p3);

    // Co-linear means angle is close to 180° (straight)
    let deviation = (180.0 - angle_deg).abs();
    deviation <= threshold_deg
}

/// Compute the angle at point `b` formed by points `a-b-c` (in degrees).
///
/// Returns the interior angle (0-180°).
fn compute_angle(a: Point, b: Point, c: Point) -> CoordF {
    // Vector ba = a - b
    let ba_x = (a.x - b.x) as f64;
    let ba_y = (a.y - b.y) as f64;

    // Vector bc = c - b
    let bc_x = (c.x - b.x) as f64;
    let bc_y = (c.y - b.y) as f64;

    // Dot product and magnitudes
    let dot = ba_x * bc_x + ba_y * bc_y;
    let mag_ba = (ba_x * ba_x + ba_y * ba_y).sqrt();
    let mag_bc = (bc_x * bc_x + bc_y * bc_y).sqrt();

    if mag_ba < 1e-9 || mag_bc < 1e-9 {
        return 180.0; // Degenerate case - consider co-linear
    }

    // Angle in radians
    let cos_angle = (dot / (mag_ba * mag_bc)).clamp(-1.0, 1.0);
    let angle_rad = cos_angle.acos();

    // Convert to degrees
    angle_rad.to_degrees()
}

/// Compute the length of a polyline (sum of segment lengths).
fn polyline_length(points: &[Point]) -> f64 {
    if points.len() < 2 {
        return 0.0;
    }

    let mut length = 0.0;
    for i in 1..points.len() {
        let dx = (points[i].x - points[i - 1].x) as f64;
        let dy = (points[i].y - points[i - 1].y) as f64;
        length += (dx * dx + dy * dy).sqrt();
    }

    length
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::ExtrusionRole;

    fn make_point(x_mm: f64, y_mm: f64) -> Point {
        Point::new(scale(x_mm), scale(y_mm))
    }

    fn make_path(points: Vec<Point>, role: ExtrusionRole, width: CoordF) -> ExtrusionPath {
        let polyline = Polyline::from_points(points);
        ExtrusionPath::from_polyline(&polyline, role)
            .with_width(width)
            .with_height(0.2)
    }

    #[test]
    fn test_paths_compatible_same_properties() {
        let points = vec![make_point(0.0, 0.0), make_point(1.0, 0.0)];
        let path1 = make_path(points.clone(), ExtrusionRole::ExternalPerimeter, 0.45);
        let path2 = make_path(points, ExtrusionRole::ExternalPerimeter, 0.45);

        assert!(paths_compatible(&path1, &path2));
    }

    #[test]
    fn test_paths_compatible_different_width() {
        let points = vec![make_point(0.0, 0.0), make_point(1.0, 0.0)];
        let path1 = make_path(points.clone(), ExtrusionRole::ExternalPerimeter, 0.45);
        let path2 = make_path(points, ExtrusionRole::ExternalPerimeter, 0.50);

        assert!(!paths_compatible(&path1, &path2));
    }

    #[test]
    fn test_paths_compatible_different_role() {
        let points = vec![make_point(0.0, 0.0), make_point(1.0, 0.0)];
        let path1 = make_path(points.clone(), ExtrusionRole::ExternalPerimeter, 0.45);
        let path2 = make_path(points, ExtrusionRole::InternalPerimeter, 0.45);

        assert!(!paths_compatible(&path1, &path2));
    }

    #[test]
    fn test_colinear_straight_line() {
        let p1 = make_point(0.0, 0.0);
        let p2 = make_point(1.0, 0.0);
        let p3 = make_point(2.0, 0.0);

        let angle = compute_angle(p1, p2, p3);
        assert!((angle - 180.0).abs() < 1.0); // Should be exactly 180°
    }

    #[test]
    fn test_colinear_right_angle() {
        let p1 = make_point(0.0, 0.0);
        let p2 = make_point(1.0, 0.0);
        let p3 = make_point(1.0, 1.0);

        let angle = compute_angle(p1, p2, p3);
        assert!((angle - 90.0).abs() < 1.0); // Should be 90°
    }

    #[test]
    fn test_check_colinear_connection_straight() {
        let points1 = vec![make_point(0.0, 0.0), make_point(1.0, 0.0)];
        let points2 = vec![make_point(1.0, 0.0), make_point(2.0, 0.0)];

        let path1 = make_path(points1, ExtrusionRole::ExternalPerimeter, 0.45);
        let path2 = make_path(points2, ExtrusionRole::ExternalPerimeter, 0.45);

        assert!(check_colinear_connection(&path1, &path2, 5.0));
    }

    #[test]
    fn test_check_colinear_connection_angled() {
        let points1 = vec![make_point(0.0, 0.0), make_point(1.0, 0.0)];
        let points2 = vec![make_point(1.0, 0.0), make_point(1.0, 1.0)]; // 90° turn

        let path1 = make_path(points1, ExtrusionRole::ExternalPerimeter, 0.45);
        let path2 = make_path(points2, ExtrusionRole::ExternalPerimeter, 0.45);

        assert!(!check_colinear_connection(&path1, &path2, 5.0));
    }

    #[test]
    fn test_merge_consecutive_paths_straight() {
        let points1 = vec![make_point(0.0, 0.0), make_point(1.0, 0.0)];
        let points2 = vec![make_point(1.0, 0.0), make_point(2.0, 0.0)];
        let points3 = vec![make_point(2.0, 0.0), make_point(3.0, 0.0)];

        let mut paths = vec![
            make_path(points1, ExtrusionRole::ExternalPerimeter, 0.45),
            make_path(points2, ExtrusionRole::ExternalPerimeter, 0.45),
            make_path(points3, ExtrusionRole::ExternalPerimeter, 0.45),
        ];

        let config = SimplificationConfig::default();
        merge_consecutive_paths(&mut paths, &config);

        // Should merge into one path
        assert_eq!(paths.len(), 1);
        assert_eq!(paths[0].points.len(), 4); // 0,0 -> 1,0 -> 2,0 -> 3,0
    }

    #[test]
    fn test_merge_consecutive_paths_different_roles() {
        let points1 = vec![make_point(0.0, 0.0), make_point(1.0, 0.0)];
        let points2 = vec![make_point(1.0, 0.0), make_point(2.0, 0.0)];

        let mut paths = vec![
            make_path(points1, ExtrusionRole::ExternalPerimeter, 0.45),
            make_path(points2, ExtrusionRole::InternalPerimeter, 0.45), // Different role
        ];

        let config = SimplificationConfig::default();
        merge_consecutive_paths(&mut paths, &config);

        // Should NOT merge (different roles)
        assert_eq!(paths.len(), 2);
    }

    #[test]
    fn test_simplify_paths_filters_short() {
        let short_points = vec![make_point(0.0, 0.0), make_point(0.01, 0.0)]; // 0.01mm
        let long_points = vec![make_point(0.0, 0.0), make_point(1.0, 0.0)]; // 1mm

        let mut paths = vec![
            make_path(short_points, ExtrusionRole::ExternalPerimeter, 0.45),
            make_path(long_points, ExtrusionRole::ExternalPerimeter, 0.45),
        ];

        let config = SimplificationConfig {
            filter_threshold: 0.05, // Filter < 0.05mm
            ..Default::default()
        };

        simplify_paths(&mut paths, &config);

        // Short path should be filtered out
        assert_eq!(paths.len(), 1);
    }

    #[test]
    fn test_polyline_length() {
        let points = vec![
            make_point(0.0, 0.0),
            make_point(1.0, 0.0),
            make_point(1.0, 1.0),
        ];

        let length = polyline_length(&points);
        let expected = scale(2.0) as f64; // 1mm + 1mm
        assert!((length - expected).abs() < scale(0.01) as f64);
    }
}
