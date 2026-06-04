//! Variable-width extrusion path generation.
//!
//! Converts ThickPolylines (from medial axis computation) into ExtrusionPaths
//! with per-segment variable LINE_WIDTH annotations. This is critical for gap fill
//! where the path width varies continuously along its length.
//!
//! Port of BambuStudio's `VariableWidth.cpp` (1–230 lines).
//!
//! ## Algorithm
//!
//! 1. Walk along each ThickPolyline segment by segment
//! 2. Track the running average width for the current extrusion path
//! 3. When the width at a vertex deviates from the current path's average
//!    by more than `TOLERANCE` (0.05mm by default), start a new path
//! 4. Each resulting ExtrusionPath has a single `width` value (the average
//!    width of its constituent segments)
//!
//! This produces many short extrusion paths for tapered gaps, each annotated
//! with `;LINE_WIDTH:` in the G-code output — matching the reference behavior
//! where gap fill has 138 unique LINE_WIDTH values on Layer 0 alone.

use crate::flow::Flow;
use crate::gcode::{ExtrusionPath, ExtrusionRole};
use crate::geometry::{Point, Polyline, ThickPolyline, ThickPolylines};
use crate::CoordF;

/// Default tolerance for width variation within a single extrusion path (mm).
/// When the width at a vertex deviates from the current segment average by more
/// than this amount, a new extrusion path is started.
///
/// BambuStudio uses 0.05mm (EPSILON in VariableWidth.cpp).
pub const WIDTH_TOLERANCE: CoordF = 0.05;

/// Minimum extrusion path length in mm. Paths shorter than this are discarded
/// to avoid tiny extrusion moves that cause firmware issues.
pub const MIN_PATH_LENGTH_MM: CoordF = 0.05;

/// Minimum width for an extrusion (mm). Widths below this are clamped.
pub const MIN_EXTRUSION_WIDTH: CoordF = 0.01;

/// Configuration for variable-width conversion.
#[derive(Debug, Clone)]
pub struct VariableWidthConfig {
    /// Maximum width deviation before splitting into a new path (mm).
    pub width_tolerance: CoordF,
    /// Minimum path length to emit (mm).
    pub min_path_length: CoordF,
    /// Layer height for flow calculations (mm).
    pub layer_height: CoordF,
    /// Nozzle diameter for flow calculations (mm).
    pub nozzle_diameter: CoordF,
    /// Extrusion speed (mm/s).
    pub speed: CoordF,
    /// The extrusion role to assign to generated paths.
    pub role: ExtrusionRole,
}

impl Default for VariableWidthConfig {
    fn default() -> Self {
        Self {
            width_tolerance: WIDTH_TOLERANCE,
            min_path_length: MIN_PATH_LENGTH_MM,
            layer_height: 0.2,
            nozzle_diameter: 0.4,
            speed: 0.0,
            role: ExtrusionRole::GapFill,
        }
    }
}

impl VariableWidthConfig {
    // Create a config for gap fill with the given parameters.
    pub fn for_gap_fill(layer_height: CoordF, nozzle_diameter: CoordF, speed: CoordF) -> Self {
        Self {
            layer_height,
            nozzle_diameter,
            speed,
            role: ExtrusionRole::GapFill,
            ..Default::default()
        }
    }

    /// Set the extrusion role.
    pub fn with_role(mut self, role: ExtrusionRole) -> Self {
        self.role = role;
        self
    }

    /// Set the width tolerance.
    pub fn with_tolerance(mut self, tolerance: CoordF) -> Self {
        self.width_tolerance = tolerance;
        self
    }
}

/// A single variable-width extrusion segment with its computed width.
#[derive(Debug, Clone)]
pub struct VariableWidthSegment {
    /// Points defining this segment path.
    pub points: Vec<Point>,
    /// Average width of this segment (mm).
    pub width: CoordF,
}

/// Convert a set of ThickPolylines into ExtrusionPaths with variable LINE_WIDTH.
///
/// This is the main entry point, equivalent to BambuStudio's `variable_width()` function
/// in `VariableWidth.cpp`.
///
/// Each ThickPolyline is split into segments of roughly uniform width, and each
/// segment becomes an ExtrusionPath with the appropriate width.
///
/// # Arguments
/// * `thick_polylines` - ThickPolylines from medial axis computation
/// * `config` - Variable width conversion parameters
///
/// # Returns
/// A vector of ExtrusionPaths, each with a specific width value.
pub fn variable_width(
    thick_polylines: &ThickPolylines,
    config: &VariableWidthConfig,
) -> Vec<ExtrusionPath> {
    let mut result = Vec::new();

    for tp in thick_polylines {
        let paths = variable_width_single(tp, config);
        result.extend(paths);
    }

    result
}

/// Convert a single ThickPolyline into one or more ExtrusionPaths.
///
/// Walks along the polyline, accumulating segments into paths of roughly
/// uniform width. When the width at a vertex deviates from the current
/// path average by more than `config.width_tolerance`, a new path is started.
///
/// BambuStudio reference: VariableWidth.cpp lines 20–150
fn variable_width_single(
    thick_polyline: &ThickPolyline,
    config: &VariableWidthConfig,
) -> Vec<ExtrusionPath> {
    if thick_polyline.len() < 2 {
        return vec![];
    }

    let segments = split_by_width(thick_polyline, config.width_tolerance);

    let mut paths = Vec::new();

    for seg in segments {
        if seg.points.len() < 2 {
            continue;
        }

        let width = seg.width.max(MIN_EXTRUSION_WIDTH);

        // Check minimum path length
        let length_mm = polyline_length_mm(&seg.points);
        if length_mm < config.min_path_length {
            continue;
        }

        // Create the polyline
        let polyline = Polyline::from_points(seg.points);

        // Build the extrusion path
        let mut path = ExtrusionPath::from_polyline(&polyline, config.role)
            .with_width(width)
            .with_height(config.layer_height)
            .with_speed(config.speed);

        // If we have a valid nozzle diameter, compute a proper Flow object
        if config.nozzle_diameter > 0.0 && config.layer_height > 0.0 {
            if let Ok(flow) = Flow::new(width, config.layer_height, config.nozzle_diameter) {
                path = path.with_flow_object(flow);
            }
        }

        paths.push(path);
    }

    paths
}

/// Split a ThickPolyline into segments of roughly uniform width.
///
/// This is the core splitting logic. It walks along the polyline and whenever
/// the width at a vertex deviates from the current segment's average by more
/// than `tolerance`, it starts a new segment.
///
/// BambuStudio splits when:
///   |width_at_vertex - segment_avg_width| > EPSILON (0.05mm)
///
/// The segments overlap by one point at boundaries to ensure continuity.
fn split_by_width(tp: &ThickPolyline, tolerance: CoordF) -> Vec<VariableWidthSegment> {
    debug_assert!(tp.len() >= 2);
    debug_assert_eq!(tp.points.len(), tp.widths.len());

    let mut segments: Vec<VariableWidthSegment> = Vec::new();

    // Current segment being built
    let mut cur_points: Vec<Point> = Vec::new();
    let mut cur_width_sum: CoordF = 0.0;
    let mut cur_count: usize = 0;

    // Start with the first point
    cur_points.push(tp.points[0]);
    cur_width_sum += tp.widths[0];
    cur_count += 1;

    for i in 1..tp.len() {
        let w = tp.widths[i];
        let cur_avg = cur_width_sum / cur_count as CoordF;

        // BambuStudio: check if the width at this vertex deviates from the
        // current segment average by more than tolerance
        let deviation = (w - cur_avg).abs();

        if deviation > tolerance && cur_points.len() >= 2 {
            // Finalize current segment
            let avg_width = cur_width_sum / cur_count as CoordF;
            segments.push(VariableWidthSegment {
                points: cur_points.clone(),
                width: avg_width,
            });

            // Start a new segment, overlapping by one point for continuity
            // Use the previous point as the start of the new segment
            cur_points.clear();
            cur_points.push(tp.points[i - 1]);
            cur_width_sum = tp.widths[i - 1];
            cur_count = 1;
        }

        // Add current point to segment
        cur_points.push(tp.points[i]);
        cur_width_sum += w;
        cur_count += 1;
    }

    // Finalize the last segment
    if cur_points.len() >= 2 {
        let avg_width = cur_width_sum / cur_count as CoordF;
        segments.push(VariableWidthSegment {
            points: cur_points,
            width: avg_width,
        });
    }

    segments
}

/// Compute the length of a point sequence in mm.
fn polyline_length_mm(points: &[Point]) -> CoordF {
    let mut total = 0.0_f64;
    for i in 1..points.len() {
        let dx = (points[i].x - points[i - 1].x) as f64;
        let dy = (points[i].y - points[i - 1].y) as f64;
        total += (dx * dx + dy * dy).sqrt();
    }
    total / crate::SCALING_FACTOR
}

/// Convert ThickPolylines to ExtrusionPaths using a Flow object for width/height.
///
/// This is a convenience wrapper that creates a VariableWidthConfig from the
/// given flow parameters.
///
/// BambuStudio reference: PerimeterGenerator.cpp line 1359:
///   `variable_width(polylines, erGapFill, this->solid_infill_flow, gap_fill.entities);`
pub fn variable_width_from_flow(
    thick_polylines: &ThickPolylines,
    role: ExtrusionRole,
    flow: &Flow,
    speed: CoordF,
) -> Vec<ExtrusionPath> {
    let config = VariableWidthConfig {
        width_tolerance: WIDTH_TOLERANCE,
        min_path_length: MIN_PATH_LENGTH_MM,
        layer_height: flow.height(),
        nozzle_diameter: flow.nozzle_diameter(),
        speed,
        role,
    };
    variable_width(thick_polylines, &config)
}

/// Filter out very short ThickPolylines that wouldn't produce meaningful extrusion.
///
/// BambuStudio filters polylines shorter than `max_width` before processing.
pub fn filter_short_polylines(polylines: &mut ThickPolylines, min_length_mm: CoordF) {
    polylines.retain(|tp| tp.length_mm() >= min_length_mm);
}

/// Merge nearly-collinear adjacent paths that have similar widths.
///
/// After splitting, we may end up with many tiny paths that could be merged
/// if their widths are similar enough. This post-processing step reduces
/// the number of extrusion moves while maintaining width accuracy.
pub fn merge_similar_paths(paths: &mut Vec<ExtrusionPath>, width_merge_tolerance: CoordF) {
    if paths.len() < 2 {
        return;
    }

    let mut merged = Vec::with_capacity(paths.len());
    let mut i = 0;

    while i < paths.len() {
        let mut current = paths[i].clone();
        let mut j = i + 1;

        while j < paths.len() {
            let next = &paths[j];

            // Check if widths are similar enough to merge
            let width_diff = (current.width - next.width).abs();
            if width_diff > width_merge_tolerance {
                break;
            }

            // Check if paths are connected (last point of current == first point of next)
            let current_last = current.last_point();
            let next_first = next.first_point();

            if let (Some(cl), Some(nf)) = (current_last, next_first) {
                let dx = (cl.x - nf.x).abs() as CoordF;
                let dy = (cl.y - nf.y).abs() as CoordF;
                let dist_mm = (dx * dx + dy * dy).sqrt() / crate::SCALING_FACTOR;

                if dist_mm < 0.01 {
                    // Merge: append next's points (skip first since it overlaps)
                    let next_points = &next.points;
                    if next_points.len() > 1 {
                        current.points.extend_from_slice(&next_points[1..]);
                    }
                    // Average the widths
                    current.width = (current.width + next.width) / 2.0;
                    j += 1;
                    continue;
                }
            }

            break;
        }

        merged.push(current);
        i = j;
    }

    *paths = merged;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use crate::scale;

    fn s(mm: f64) -> i64 {
        scale(mm)
    }

    fn make_config() -> VariableWidthConfig {
        VariableWidthConfig {
            width_tolerance: 0.05,
            min_path_length: 0.05,
            layer_height: 0.2,
            nozzle_diameter: 0.4,
            speed: 30.0,
            role: ExtrusionRole::GapFill,
        }
    }

    #[test]
    fn test_variable_width_empty() {
        let config = make_config();
        let result = variable_width(&vec![], &config);
        assert!(result.is_empty());
    }

    #[test]
    fn test_variable_width_single_point() {
        let config = make_config();
        let tp = ThickPolyline::from_points_and_widths(vec![Point::new(0, 0)], vec![0.4]);
        let result = variable_width(&vec![tp], &config);
        assert!(result.is_empty()); // Single point can't form a path
    }

    #[test]
    fn test_variable_width_uniform() {
        let config = make_config();
        let tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(5.0), 0),
                Point::new(s(10.0), 0),
            ],
            vec![0.4, 0.4, 0.4],
        );
        let result = variable_width(&vec![tp], &config);

        // Uniform width should produce a single path
        assert_eq!(result.len(), 1);
        assert!((result[0].width - 0.4).abs() < 0.01);
        assert_eq!(result[0].points.len(), 3);
    }

    #[test]
    fn test_variable_width_split_at_large_change() {
        let config = make_config();
        // Width changes from 0.1 to 0.5 — should split
        let tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(5.0), 0),
                Point::new(s(10.0), 0),
                Point::new(s(15.0), 0),
            ],
            vec![0.1, 0.12, 0.5, 0.52],
        );
        let result = variable_width(&vec![tp], &config);

        // Should produce at least 2 paths due to the width jump
        assert!(
            result.len() >= 2,
            "Expected at least 2 paths, got {}",
            result.len()
        );

        // First path should be narrow, second should be wide
        let first_width = result[0].width;
        let last_width = result.last().unwrap().width;
        assert!(
            first_width < 0.3,
            "First path width {:.3} should be < 0.3",
            first_width
        );
        assert!(
            last_width > 0.3,
            "Last path width {:.3} should be > 0.3",
            last_width
        );
    }

    #[test]
    fn test_variable_width_gradual_taper() {
        let config = make_config();
        // Gradual taper from 0.1 to 0.7 over many points
        let n = 20;
        let mut points = Vec::with_capacity(n);
        let mut widths = Vec::with_capacity(n);
        for i in 0..n {
            points.push(Point::new(s(i as f64 * 1.0), 0));
            widths.push(0.1 + 0.6 * (i as f64) / (n as f64 - 1.0));
        }
        let tp = ThickPolyline::from_points_and_widths(points, widths);
        let result = variable_width(&vec![tp], &config);

        // Should produce multiple paths, each with moderate width variation
        assert!(
            result.len() >= 2,
            "Expected multiple paths for gradual taper, got {}",
            result.len()
        );

        // Widths should be monotonically non-decreasing (since source is monotonic)
        for i in 1..result.len() {
            assert!(
                result[i].width >= result[i - 1].width - 0.1,
                "Path {} width {:.3} is much less than path {} width {:.3}",
                i,
                result[i].width,
                i - 1,
                result[i - 1].width,
            );
        }
    }

    #[test]
    fn test_variable_width_respects_min_length() {
        let config = VariableWidthConfig {
            min_path_length: 1.0, // 1mm minimum
            ..make_config()
        };

        // Create a very short polyline (0.1mm)
        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(0.05), 0)],
            vec![0.4, 0.4],
        );
        let result = variable_width(&vec![tp], &config);
        assert!(
            result.is_empty(),
            "Should filter out paths shorter than min_length"
        );
    }

    #[test]
    fn test_variable_width_multiple_polylines() {
        let config = make_config();
        let tp1 = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(5.0), 0)],
            vec![0.3, 0.3],
        );
        let tp2 = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(s(10.0), 0),
                Point::new(s(15.0), 0),
                Point::new(s(20.0), 0),
            ],
            vec![0.5, 0.5, 0.5],
        );
        let result = variable_width(&vec![tp1, tp2], &config);

        // Should produce at least 2 paths (one per polyline)
        assert!(result.len() >= 2);
    }

    #[test]
    fn test_variable_width_role_propagation() {
        let config = VariableWidthConfig {
            role: ExtrusionRole::GapFill,
            ..make_config()
        };

        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.4, 0.4],
        );
        let result = variable_width(&vec![tp], &config);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].role, ExtrusionRole::GapFill);
    }

    #[test]
    fn test_split_by_width_uniform() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(5.0), 0),
                Point::new(s(10.0), 0),
            ],
            vec![0.4, 0.41, 0.39],
        );

        let segments = split_by_width(&tp, 0.05);
        assert_eq!(
            segments.len(),
            1,
            "Uniform width should produce single segment"
        );
        assert_eq!(segments[0].points.len(), 3);
    }

    #[test]
    fn test_split_by_width_step_change() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(5.0), 0),
                Point::new(s(10.0), 0),
                Point::new(s(15.0), 0),
            ],
            vec![0.2, 0.21, 0.5, 0.51],
        );

        let segments = split_by_width(&tp, 0.05);
        assert!(
            segments.len() >= 2,
            "Step change should split into at least 2 segments, got {}",
            segments.len()
        );
    }

    #[test]
    fn test_polyline_length_mm() {
        let points = vec![Point::new(0, 0), Point::new(s(10.0), 0)];
        let len = polyline_length_mm(&points);
        assert!((len - 10.0).abs() < 0.001, "Expected 10mm, got {:.3}", len);
    }

    #[test]
    fn test_filter_short_polylines() {
        let mut polylines = vec![
            ThickPolyline::from_points_and_widths(
                vec![Point::new(0, 0), Point::new(s(10.0), 0)],
                vec![0.4, 0.4],
            ),
            ThickPolyline::from_points_and_widths(
                vec![Point::new(0, 0), Point::new(s(0.01), 0)],
                vec![0.4, 0.4],
            ),
        ];

        filter_short_polylines(&mut polylines, 0.1);
        assert_eq!(
            polylines.len(),
            1,
            "Should filter out polyline shorter than 0.1mm"
        );
        assert!((polylines[0].length_mm() - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_variable_width_from_flow() {
        let flow = Flow::new(0.4, 0.2, 0.4).unwrap();
        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.4, 0.4],
        );
        let result = variable_width_from_flow(&vec![tp], ExtrusionRole::GapFill, &flow, 30.0);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].role, ExtrusionRole::GapFill);
    }

    #[test]
    fn test_merge_similar_paths() {
        // Two connected paths with similar widths should merge
        let path1 = ExtrusionPath::from_polyline(
            &Polyline::from_points(vec![Point::new(0, 0), Point::new(s(5.0), 0)]),
            ExtrusionRole::GapFill,
        )
        .with_width(0.4);

        let path2 = ExtrusionPath::from_polyline(
            &Polyline::from_points(vec![Point::new(s(5.0), 0), Point::new(s(10.0), 0)]),
            ExtrusionRole::GapFill,
        )
        .with_width(0.41);

        let mut paths = vec![path1, path2];
        merge_similar_paths(&mut paths, 0.05);

        assert_eq!(paths.len(), 1, "Similar connected paths should merge");
        assert_eq!(paths[0].points.len(), 3);
    }

    #[test]
    fn test_merge_different_widths_no_merge() {
        let path1 = ExtrusionPath::from_polyline(
            &Polyline::from_points(vec![Point::new(0, 0), Point::new(s(5.0), 0)]),
            ExtrusionRole::GapFill,
        )
        .with_width(0.2);

        let path2 = ExtrusionPath::from_polyline(
            &Polyline::from_points(vec![Point::new(s(5.0), 0), Point::new(s(10.0), 0)]),
            ExtrusionRole::GapFill,
        )
        .with_width(0.6);

        let mut paths = vec![path1, path2];
        merge_similar_paths(&mut paths, 0.05);

        assert_eq!(
            paths.len(),
            2,
            "Paths with different widths should not merge"
        );
    }

    #[test]
    fn test_variable_width_realistic_gap() {
        // Simulate a realistic tapered gap: width starts at 0.1, peaks at 0.35, returns to 0.1
        let config = make_config();
        let n = 30;
        let mut points = Vec::with_capacity(n);
        let mut widths = Vec::with_capacity(n);

        for i in 0..n {
            let t = i as f64 / (n as f64 - 1.0);
            let x = t * 15.0; // 15mm long gap
            let w = 0.1 + 0.25 * (std::f64::consts::PI * t).sin(); // Sinusoidal width
            points.push(Point::new(s(x), 0));
            widths.push(w);
        }

        let tp = ThickPolyline::from_points_and_widths(points, widths);
        let result = variable_width(&vec![tp], &config);

        // Should produce multiple paths with varying widths
        assert!(
            result.len() >= 2,
            "Realistic tapered gap should produce multiple paths, got {}",
            result.len()
        );

        // Collect unique widths (rounded to 0.01)
        let unique_widths: std::collections::HashSet<i32> = result
            .iter()
            .map(|p| (p.width * 100.0).round() as i32)
            .collect();
        assert!(
            unique_widths.len() >= 2,
            "Should have multiple distinct widths, got {}",
            unique_widths.len()
        );

        // Total length should be close to the original 15mm
        let total_len: CoordF = result.iter().map(|p| p.length_mm()).sum();
        assert!(
            total_len > 10.0 && total_len < 20.0,
            "Total length {:.1}mm should be roughly 15mm",
            total_len
        );
    }
}
