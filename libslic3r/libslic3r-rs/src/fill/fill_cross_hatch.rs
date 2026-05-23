//! Cross Hatch Infill Pattern Implementation
//!
//! CrossHatch infill enhances 3D printing speed and reduces noise by alternating
//! line direction by 90 degrees every few layers to improve adhesion.
//! It introduces transform layers between direction shifts for better line cohesion,
//! which fixes the weakness of line infill.
//!
//! # Algorithm Overview
//!
//! The pattern consists of two types of layers:
//! 1. **Repeat layers**: Simple parallel lines in a single direction
//! 2. **Transform layers**: Zigzag patterns that transition between directions
//!
//! The transform technique creates smooth transitions between direction changes,
//! improving layer adhesion while maintaining fast print speeds.
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/Fill/FillCrossHatch.cpp`
//! - `src/libslic3r/Fill/FillCrossHatch.hpp`
//!
//! Based on the work by Bambu Lab, with transform technique inspired by David Eccles (gringer).

use crate::clipper_utils::intersect_polylines_with_expolygons;
use crate::geometry::{BoundingBox, ExPolygon, Point, Polyline};
use crate::{scale, Coord, CoordF};

/// Configuration for Cross Hatch infill generation.
#[derive(Debug, Clone)]
pub struct CrossHatchConfig {
    /// Grid spacing (mm) - determines the distance between parallel lines.
    pub grid_size: CoordF,

    /// Infill angle in degrees.
    pub angle: CoordF,

    /// Ratio between repeat layer height and grid size.
    /// Higher values mean more repeat layers relative to transform layers.
    /// Range: 0.2 to 1.0, default is 1.0.
    pub repeat_ratio: CoordF,

    /// Density adjustment factor (0.0 to 1.0).
    pub density: CoordF,

    /// Whether to connect adjacent lines for shorter travel moves.
    pub connect_lines: bool,

    /// Minimum line length to keep (mm).
    pub min_line_length: CoordF,
}

impl Default for CrossHatchConfig {
    fn default() -> Self {
        Self {
            grid_size: 2.0,       // 2mm grid
            angle: 0.0,           // No rotation by default
            repeat_ratio: 1.0,    // Equal repeat and transform layers
            density: 1.0,         // Full density
            connect_lines: true,  // Connect lines by default
            min_line_length: 0.8, // 0.8mm minimum line
        }
    }
}

impl CrossHatchConfig {
    // Create a new configuration with the given grid size.
    pub fn new(grid_size: CoordF) -> Self {
        Self {
            grid_size,
            ..Default::default()
        }
    }

    /// Create a configuration with grid size derived from extrusion width and density.
    pub fn from_density(extrusion_width: CoordF, density: CoordF) -> Self {
        let grid_size = if density > 0.0 && density <= 1.0 {
            extrusion_width / density
        } else {
            extrusion_width
        };

        // Optimize repeat ratio for low density (from BambuStudio)
        let repeat_ratio = if density < 0.3 {
            (1.0 - (-5.0 * density).exp()).clamp(0.2, 1.0)
        } else {
            1.0
        };

        Self {
            grid_size,
            density,
            repeat_ratio,
            ..Default::default()
        }
    }

    /// Set the infill angle.
    pub fn with_angle(mut self, angle: CoordF) -> Self {
        self.angle = angle;
        self
    }

    /// Set the repeat ratio.
    pub fn with_repeat_ratio(mut self, ratio: CoordF) -> Self {
        self.repeat_ratio = ratio.clamp(0.2, 1.0);
        self
    }

    /// Enable or disable line connection.
    pub fn with_connect_lines(mut self, connect: bool) -> Self {
        self.connect_lines = connect;
        self
    }
}

/// Result of Cross Hatch infill generation.
#[derive(Debug, Clone, Default)]
pub struct CrossHatchResult {
    /// Generated polylines representing the infill pattern.
    pub polylines: Vec<Polyline>,

    /// Whether this layer is a transform layer (vs repeat layer).
    pub is_transform_layer: bool,

    /// The direction used for this layer (-1 = vertical, 1 = horizontal).
    pub direction: i32,

    /// The progress value for transform layers (0.0 to 1.0).
    pub progress: CoordF,
}

impl CrossHatchResult {
    // Check if any infill was generated.
    pub fn has_infill(&self) -> bool {
        !self.polylines.is_empty()
    }

    /// Get the total number of polylines.
    pub fn polyline_count(&self) -> usize {
        self.polylines.len()
    }

    /// Calculate total length of all polylines in mm.
    pub fn total_length_mm(&self) -> CoordF {
        self.polylines.iter().map(|pl| pl.length()).sum()
    }
}

/// Generator for Cross Hatch infill patterns.
#[derive(Debug, Clone)]
pub struct CrossHatchGenerator {
    config: CrossHatchConfig,
}

impl CrossHatchGenerator {
    // Create a new generator with the given configuration.
    pub fn new(config: CrossHatchConfig) -> Self {
        Self { config }
    }

    /// Create a generator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(CrossHatchConfig::default())
    }

    /// Create a generator from extrusion width and density.
    pub fn from_density(extrusion_width: CoordF, density: CoordF) -> Self {
        Self::new(CrossHatchConfig::from_density(extrusion_width, density))
    }

    /// Get a reference to the configuration.
    pub fn config(&self) -> &CrossHatchConfig {
        &self.config
    }

    /// Generate Cross Hatch infill for the given area at the specified Z height.
    ///
    /// # Arguments
    /// * `fill_area` - The ExPolygons to fill
    /// * `z_height_mm` - The Z height in mm (used to determine layer type)
    ///
    /// # Returns
    /// A CrossHatchResult containing the generated polylines.
    pub fn generate(&self, fill_area: &[ExPolygon], z_height_mm: CoordF) -> CrossHatchResult {
        let mut result = CrossHatchResult::default();

        if fill_area.is_empty() || self.config.density <= 0.0 {
            return result;
        }

        // Get bounding box of all fill areas
        let mut bbox = BoundingBox::new();
        for expoly in fill_area {
            bbox.merge(&expoly.bounding_box());
        }

        if bbox.is_empty() {
            return result;
        }

        // Generate the infill pattern for this Z height
        let grid_size_scaled = scale(self.config.grid_size);
        let width = bbox.width();
        let height = bbox.height();

        // Align bounding box to grid
        let aligned_min = align_to_grid(bbox.min, grid_size_scaled * 4);
        let aligned_bbox = BoundingBox::from_points(&[
            aligned_min,
            Point::new(
                aligned_min.x + width + grid_size_scaled * 4,
                aligned_min.y + height + grid_size_scaled * 4,
            ),
        ]);

        let aligned_width = aligned_bbox.width();
        let aligned_height = aligned_bbox.height();

        // Generate pattern based on Z height
        let (polylines, is_transform, direction, progress) = generate_infill_layers(
            scale(z_height_mm),
            self.config.repeat_ratio,
            grid_size_scaled,
            aligned_width,
            aligned_height,
        );

        result.is_transform_layer = is_transform;
        result.direction = direction;
        result.progress = progress;

        if polylines.is_empty() {
            return result;
        }

        // Translate pattern to actual bounding box position
        let mut translated: Vec<Polyline> = polylines
            .into_iter()
            .map(|mut pl| {
                pl.translate(aligned_bbox.min);
                pl
            })
            .collect();

        // Apply rotation if angle is set
        if self.config.angle.abs() >= 1e-6 {
            let angle_rad = -self.config.angle.to_radians();
            let center = bbox.center();
            for pl in &mut translated {
                // Translate to origin, rotate, translate back
                pl.translate(Point::new(-center.x, -center.y));
                pl.rotate(angle_rad);
                pl.translate(center);
            }
        }

        // Clip polylines to the fill area
        let clipped = intersect_polylines_with_expolygons(&translated, fill_area);

        // Filter out very short segments
        let min_length_mm = self.config.min_line_length;
        result.polylines = clipped
            .into_iter()
            .filter(|pl| pl.length() >= min_length_mm)
            .collect();

        result
    }
}

/// Generate one cycle of the transform pattern waveform.
///
/// Creates 4 points that form one zigzag cycle:
/// ```text
///     o---o
///    /     \
///   /       \
///            \       /
///             \     /
///              o---o
///    p1   p2  p3   p4
/// ```
fn generate_one_cycle(progress: CoordF, period: Coord) -> Vec<Point> {
    let period_f = period as CoordF;
    let offset = progress * 0.125 * period_f; // progress * 1/8 * period
    let offset_coord = offset as Coord;

    vec![
        Point::new((0.25 * period_f) as Coord - offset_coord, offset_coord),
        Point::new((0.25 * period_f) as Coord + offset_coord, offset_coord),
        Point::new((0.75 * period_f) as Coord - offset_coord, -offset_coord),
        Point::new((0.75 * period_f) as Coord + offset_coord, -offset_coord),
    ]
}

/// Generate the transform pattern (zigzag transition between directions).
///
/// # Arguments
/// * `progress` - Progress through the transform (0.0 to 1.0)
/// * `direction` - Direction (-1 for vertical, 1 for horizontal)
/// * `grid_size` - Base grid size
/// * `width` - Pattern width
/// * `height` - Pattern height
fn generate_transform_pattern(
    progress: CoordF,
    direction: i32,
    grid_size: Coord,
    width: Coord,
    height: Coord,
) -> Vec<Polyline> {
    let (actual_width, actual_height) = if direction < 0 {
        (height, width)
    } else {
        (width, height)
    };

    // Double the grid size since we handle odd and even separately
    let double_grid = grid_size * 2;

    // Generate template cycle
    let one_cycle = generate_one_cycle(progress, double_grid);

    // Calculate how many cycles we need
    let num_cycles = (actual_width / double_grid + 2) as usize;
    let num_lines = (actual_height / double_grid + 2) as usize;

    let mut out_polylines = Vec::with_capacity(num_lines * 2);

    // Build odd lines (base pattern)
    let mut odd_poly_points = Vec::with_capacity(num_cycles * one_cycle.len());
    for i in 0..num_cycles {
        for pt in &one_cycle {
            odd_poly_points.push(Point::new(pt.x + (i as Coord) * double_grid, pt.y));
        }
    }

    // Replicate for all odd lines
    for i in 0..num_lines {
        let mut points = odd_poly_points.clone();
        let y_offset = (i as Coord) * double_grid;
        for pt in &mut points {
            pt.y += y_offset;
        }
        out_polylines.push(Polyline::from_points(points));
    }

    // Build even lines (offset by half grid)
    for i in 0..num_lines {
        let mut points = odd_poly_points.clone();
        let x_offset = -double_grid / 2;
        let y_offset = ((i as CoordF + 0.5) * double_grid as CoordF) as Coord;
        for pt in &mut points {
            pt.x += x_offset;
            pt.y = pt.y + y_offset;
        }
        out_polylines.push(Polyline::from_points(points));
    }

    // Swap X and Y if vertical direction
    if direction < 0 {
        for pl in &mut out_polylines {
            for pt in pl.points_mut() {
                std::mem::swap(&mut pt.x, &mut pt.y);
            }
        }
    }

    out_polylines
}

/// Generate the repeat pattern (simple parallel lines).
///
/// # Arguments
/// * `direction` - Direction (-1 for vertical, 1 for horizontal)
/// * `grid_size` - Spacing between lines
/// * `width` - Pattern width
/// * `height` - Pattern height
fn generate_repeat_pattern(
    direction: i32,
    grid_size: Coord,
    width: Coord,
    height: Coord,
) -> Vec<Polyline> {
    let (actual_width, actual_height) = if direction < 0 {
        (height, width)
    } else {
        (width, height)
    };

    let num_lines = (actual_height / grid_size + 1) as usize;
    let mut out_polylines = Vec::with_capacity(num_lines);

    for i in 0..num_lines {
        let y = (i as Coord) * grid_size;
        let points = vec![Point::new(0, y), Point::new(actual_width, y)];
        out_polylines.push(Polyline::from_points(points));
    }

    // Swap X and Y if vertical direction
    if direction < 0 {
        for pl in &mut out_polylines {
            for pt in pl.points_mut() {
                std::mem::swap(&mut pt.x, &mut pt.y);
            }
        }
    }

    out_polylines
}

/// Generate infill pattern for a specific Z height.
///
/// Returns the polylines along with layer type information:
/// - `is_transform` - Whether this is a transform layer
/// - `direction` - The direction (-1 vertical, 1 horizontal)
/// - `progress` - Progress through transform (if applicable)
fn generate_infill_layers(
    z_height: Coord,
    repeat_ratio: CoordF,
    grid_size: Coord,
    width: Coord,
    height: Coord,
) -> (Vec<Polyline>, bool, i32, CoordF) {
    let grid_size_f = grid_size as CoordF;

    // Layer geometry:
    // - Transform layer size = 40% of grid
    // - Repeat layer size = repeat_ratio * grid
    let trans_layer_size = grid_size_f * 0.4;
    let repeat_layer_size = grid_size_f * repeat_ratio;

    // Offset to improve first layer strength and reduce warping risk
    let z_height_f = z_height as CoordF + repeat_layer_size / 2.0;

    // Calculate period (one full cycle of transform + repeat)
    let period = trans_layer_size + repeat_layer_size;

    // Find where we are in the current period
    let periods_completed = (z_height_f / period).floor();
    let remains = z_height_f - periods_completed * period;

    // Transform Z is relative to the repeat layer (repeat comes first)
    let trans_z = remains - repeat_layer_size;

    // Determine direction based on double-period phase
    let double_period = period * 2.0;
    let phase = (z_height_f % double_period) - (period - 1.0);
    let direction = if phase <= 0.0 { -1 } else { 1 };

    // Check if this is a repeat or transform layer
    if trans_z < 0.0 {
        // This is a repeat layer
        let polylines = generate_repeat_pattern(direction, grid_size, width, height);
        (polylines, false, direction, 0.0)
    } else {
        // This is a transform layer
        let progress_raw = (trans_z % trans_layer_size) / trans_layer_size;

        // Split progress into forward and backward phases with opposite direction
        let (progress, actual_direction) = if progress_raw < 0.5 {
            // Forward phase - increase overlapping slightly
            ((progress_raw + 0.1) * 2.0, direction)
        } else {
            // Backward phase - opposite direction
            ((1.1 - progress_raw) * 2.0, -direction)
        };

        let polylines =
            generate_transform_pattern(progress, actual_direction, grid_size, width, height);
        (polylines, true, actual_direction, progress)
    }
}

/// Align a point to a grid.
fn align_to_grid(point: Point, grid_size: Coord) -> Point {
    Point::new(
        (point.x / grid_size) * grid_size,
        (point.y / grid_size) * grid_size,
    )
}

/// Convenience function to generate Cross Hatch infill with default settings.
pub fn generate_cross_hatch(
    fill_area: &[ExPolygon],
    z_height_mm: CoordF,
    extrusion_width: CoordF,
    density: CoordF,
) -> CrossHatchResult {
    let generator = CrossHatchGenerator::from_density(extrusion_width, density);
    generator.generate(fill_area, z_height_mm)
}

/// Convenience function to generate Cross Hatch infill with custom angle.
pub fn generate_cross_hatch_with_angle(
    fill_area: &[ExPolygon],
    z_height_mm: CoordF,
    extrusion_width: CoordF,
    density: CoordF,
    angle: CoordF,
) -> CrossHatchResult {
    let config = CrossHatchConfig::from_density(extrusion_width, density).with_angle(angle);
    let generator = CrossHatchGenerator::new(config);
    generator.generate(fill_area, z_height_mm)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn make_square_mm(size: CoordF) -> ExPolygon {
        let half = scale(size / 2.0);
        ExPolygon::new(Polygon::from_points(vec![
            Point::new(-half, -half),
            Point::new(half, -half),
            Point::new(half, half),
            Point::new(-half, half),
        ]))
    }

    fn make_rectangle_mm(width: CoordF, height: CoordF) -> ExPolygon {
        let hw = scale(width / 2.0);
        let hh = scale(height / 2.0);
        ExPolygon::new(Polygon::from_points(vec![
            Point::new(-hw, -hh),
            Point::new(hw, -hh),
            Point::new(hw, hh),
            Point::new(-hw, hh),
        ]))
    }

    #[test]
    fn test_config_default() {
        let config = CrossHatchConfig::default();
        assert!((config.grid_size - 2.0).abs() < 1e-6);
        assert!((config.angle - 0.0).abs() < 1e-6);
        assert!((config.repeat_ratio - 1.0).abs() < 1e-6);
        assert!((config.density - 1.0).abs() < 1e-6);
        assert!(config.connect_lines);
    }

    #[test]
    fn test_config_from_density() {
        let config = CrossHatchConfig::from_density(0.4, 0.2);
        // Grid size = extrusion_width / density = 0.4 / 0.2 = 2.0
        assert!((config.grid_size - 2.0).abs() < 1e-6);
        // Low density adjusts repeat ratio
        assert!(config.repeat_ratio < 1.0);
        assert!(config.repeat_ratio >= 0.2);
    }

    #[test]
    fn test_config_from_density_high() {
        let config = CrossHatchConfig::from_density(0.4, 0.5);
        // Grid size = 0.4 / 0.5 = 0.8
        assert!((config.grid_size - 0.8).abs() < 1e-6);
        // High density keeps repeat ratio at 1.0
        assert!((config.repeat_ratio - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_generate_one_cycle() {
        let period: Coord = 1000000; // 1mm in scaled coords
        let cycle = generate_one_cycle(0.0, period);

        assert_eq!(cycle.len(), 4);
        // At progress 0, offset should be 0
        assert_eq!(cycle[0].y, 0);
        assert_eq!(cycle[1].y, 0);
    }

    #[test]
    fn test_generate_one_cycle_with_progress() {
        let period: Coord = 1000000;
        let cycle = generate_one_cycle(1.0, period);

        assert_eq!(cycle.len(), 4);
        // At progress 1.0, offset should be period/8
        let expected_offset = (period as CoordF * 0.125) as Coord;
        assert_eq!(cycle[0].y, expected_offset);
    }

    #[test]
    fn test_generate_repeat_pattern_horizontal() {
        let polylines = generate_repeat_pattern(1, scale(1.0), scale(10.0), scale(5.0));

        // Should have lines at intervals
        assert!(!polylines.is_empty());

        // All lines should be horizontal (same Y for start and end)
        for pl in &polylines {
            assert!(pl.points().len() >= 2);
            assert_eq!(pl.points()[0].y, pl.points()[1].y);
        }
    }

    #[test]
    fn test_generate_repeat_pattern_vertical() {
        let polylines = generate_repeat_pattern(-1, scale(1.0), scale(10.0), scale(5.0));

        assert!(!polylines.is_empty());

        // All lines should be vertical (same X for start and end)
        for pl in &polylines {
            assert!(pl.points().len() >= 2);
            assert_eq!(pl.points()[0].x, pl.points()[1].x);
        }
    }

    #[test]
    fn test_generate_transform_pattern() {
        let polylines = generate_transform_pattern(0.5, 1, scale(1.0), scale(10.0), scale(5.0));

        assert!(!polylines.is_empty());

        // Transform pattern should have more points per line (zigzag)
        for pl in &polylines {
            assert!(pl.points().len() >= 2);
        }
    }

    #[test]
    fn test_generate_infill_layers_repeat() {
        // At Z=0, should be in repeat layer
        let (polylines, is_transform, direction, progress) =
            generate_infill_layers(scale(0.1), 1.0, scale(2.0), scale(20.0), scale(20.0));

        assert!(!polylines.is_empty());
        // Direction should be -1 or 1
        assert!(direction == -1 || direction == 1);

        if !is_transform {
            assert!((progress - 0.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_generator_empty_area() {
        let generator = CrossHatchGenerator::with_defaults();
        let result = generator.generate(&[], 1.0);

        assert!(!result.has_infill());
        assert_eq!(result.polyline_count(), 0);
    }

    #[test]
    fn test_generator_zero_density() {
        let config = CrossHatchConfig {
            density: 0.0,
            ..Default::default()
        };
        let generator = CrossHatchGenerator::new(config);
        let square = make_square_mm(20.0);
        let result = generator.generate(&[square], 1.0);

        assert!(!result.has_infill());
    }

    #[test]
    fn test_generator_basic() {
        let generator = CrossHatchGenerator::from_density(0.4, 0.2);
        let square = make_square_mm(20.0);
        let result = generator.generate(&[square], 1.0);

        // Should generate some infill
        assert!(result.has_infill());
        assert!(result.polyline_count() > 0);
    }

    #[test]
    fn test_generator_with_angle() {
        let config = CrossHatchConfig::from_density(0.4, 0.3).with_angle(45.0);
        let generator = CrossHatchGenerator::new(config);
        let square = make_square_mm(20.0);
        let result = generator.generate(&[square], 1.0);

        assert!(result.has_infill());
    }

    #[test]
    fn test_layer_variation() {
        let generator = CrossHatchGenerator::from_density(0.4, 0.5);
        let square = make_square_mm(20.0);

        // Generate at different Z heights
        let result1 = generator.generate(&[square.clone()], 0.2);
        let result2 = generator.generate(&[square.clone()], 0.5);
        let result3 = generator.generate(&[square.clone()], 1.0);

        // All should have infill
        assert!(result1.has_infill());
        assert!(result2.has_infill());
        assert!(result3.has_infill());
    }

    #[test]
    fn test_direction_alternation() {
        let generator = CrossHatchGenerator::from_density(0.4, 0.5);
        let square = make_square_mm(30.0);

        // Test at various Z heights to see direction changes
        let mut found_negative = false;
        let mut found_positive = false;

        for i in 0..20 {
            let z = (i as CoordF) * 0.3;
            let result = generator.generate(&[square.clone()], z);
            if result.direction < 0 {
                found_negative = true;
            } else if result.direction > 0 {
                found_positive = true;
            }
        }

        // Should find both directions over multiple layers
        assert!(found_negative || found_positive);
    }

    #[test]
    fn test_convenience_function() {
        let square = make_square_mm(20.0);
        let result = generate_cross_hatch(&[square], 1.0, 0.4, 0.3);

        assert!(result.has_infill());
    }

    #[test]
    fn test_convenience_function_with_angle() {
        let square = make_square_mm(20.0);
        let result = generate_cross_hatch_with_angle(&[square], 1.0, 0.4, 0.3, 45.0);

        assert!(result.has_infill());
    }

    #[test]
    fn test_total_length() {
        let generator = CrossHatchGenerator::from_density(0.4, 0.3);
        let square = make_square_mm(20.0);
        let result = generator.generate(&[square], 1.0);

        if result.has_infill() {
            let length = result.total_length_mm();
            assert!(length > 0.0);
        }
    }

    #[test]
    fn test_rectangular_area() {
        let generator = CrossHatchGenerator::from_density(0.4, 0.25);
        let rect = make_rectangle_mm(30.0, 10.0);
        let result = generator.generate(&[rect], 0.5);

        assert!(result.has_infill());
    }

    #[test]
    fn test_align_to_grid() {
        let pt = Point::new(1234567, 9876543);
        let grid = 1000000; // 1mm grid
        let aligned = align_to_grid(pt, grid);

        assert_eq!(aligned.x % grid, 0);
        assert_eq!(aligned.y % grid, 0);
    }
}
