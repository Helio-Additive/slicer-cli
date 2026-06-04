//! Arachne variable-width perimeter generation module.
//!
//! This module provides variable-width perimeter generation, which adapts the
//! extrusion width based on local geometry. This is particularly useful for:
//! - Thin walls that don't fit a whole number of perimeters
//! - Narrow features that would otherwise be skipped or poorly printed
//! - Improving surface quality by avoiding gaps
//!
//! # Overview
//!
//! The Arachne algorithm (originally from CuraEngine, adopted by PrusaSlicer
//! and BambuStudio) uses a skeletal trapezoidation approach to generate
//! perimeters that vary in width along their length.
//!
//! This implementation provides a simplified version that:
//! 1. Detects thin regions where standard perimeters would collapse
//! 2. Generates variable-width paths for those regions
//! 3. Falls back to standard fixed-width perimeters for normal geometry
//!
//! # BambuStudio Reference
//!
//! This module corresponds to `src/libslic3r/Arachne/` directory.

pub mod beading;
pub mod beading_factory;
pub mod beading_strategies;
pub mod distributed_beading;
pub mod half_edge;
pub mod junction;
pub mod limited_beading;
pub mod line;
pub mod polyline_stitcher;
pub mod skeletal_trapezoidation;

pub use beading::{BeadingCalculator, BeadingResult, BeadingStrategy};
pub use beading_factory::{BeadingStrategyFactory, BeadingStrategyType};
pub use half_edge::{HalfEdge, HalfEdgeGraph, HalfEdgeNode};
pub use junction::{ExtrusionJunction, ExtrusionJunctions};
pub use line::{ExtrusionLine, VariableWidthLines};
pub use polyline_stitcher::stitch_polylines;
pub use skeletal_trapezoidation::{SkeletonEdge, SkeletonGraph, SkeletonNode};

use crate::clipper_utils::{shrink, OffsetJoinType};
use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use crate::{scale, Coord, CoordF};

/// Configuration for Arachne variable-width perimeter generation.
#[derive(Debug, Clone)]
pub struct ArachneConfig {
    /// Nominal (preferred) bead/extrusion width for outer wall (mm).
    pub bead_width_outer: CoordF,

    /// Nominal (preferred) bead/extrusion width for inner walls (mm).
    pub bead_width_inner: CoordF,

    /// Minimum allowed bead width (mm).
    /// Features narrower than this will not be printed.
    pub min_bead_width: CoordF,

    /// Minimum feature size to print (mm).
    /// Features smaller than this are considered unprintable.
    pub min_feature_size: CoordF,

    /// Maximum number of perimeter walls.
    pub wall_count: usize,

    /// How far to inset the outer wall from the actual outline (mm).
    /// This can improve adhesion between walls.
    pub wall_0_inset: CoordF,

    /// Wall transition length - how long the transition between different
    /// wall counts should be (mm).
    pub wall_transition_length: CoordF,

    /// The angle (in degrees) at which walls transition from N to N+1 walls.
    pub wall_transition_angle: CoordF,

    /// Whether to print thin walls (features that don't fit normal perimeters).
    pub print_thin_walls: bool,

    /// Join type for offset operations.
    pub join_type: OffsetJoinType,

    /// Beading strategy for distributing wall widths.
    pub beading_strategy: BeadingStrategy,
}

impl Default for ArachneConfig {
    fn default() -> Self {
        Self {
            bead_width_outer: 0.45,
            bead_width_inner: 0.45,
            min_bead_width: 0.1,
            min_feature_size: 0.1,
            wall_count: 3,
            wall_0_inset: 0.0,
            wall_transition_length: 0.4,
            wall_transition_angle: 10.0,
            print_thin_walls: true,
            join_type: OffsetJoinType::Miter,
            beading_strategy: BeadingStrategy::Distributed,
        }
    }
}

impl ArachneConfig {
    // Create a new config with specified wall count and bead width.
    pub fn new(wall_count: usize, bead_width: CoordF) -> Self {
        Self {
            bead_width_outer: bead_width,
            bead_width_inner: bead_width,
            wall_count,
            ..Default::default()
        }
    }

    /// Set different widths for outer and inner walls.
    pub fn with_wall_widths(mut self, outer: CoordF, inner: CoordF) -> Self {
        self.bead_width_outer = outer;
        self.bead_width_inner = inner;
        self
    }

    /// Enable or disable thin wall printing.
    pub fn with_thin_walls(mut self, enabled: bool) -> Self {
        self.print_thin_walls = enabled;
        self
    }

    /// Set minimum bead width.
    pub fn with_min_bead_width(mut self, width: CoordF) -> Self {
        self.min_bead_width = width;
        self
    }

    /// Builder: set beading strategy.
    pub fn with_beading_strategy(mut self, strategy: BeadingStrategy) -> Self {
        self.beading_strategy = strategy;
        self
    }

    /// Create a beading calculator from this config.
    pub fn beading_calculator(&self) -> BeadingCalculator {
        BeadingCalculator::new(
            self.beading_strategy,
            self.bead_width_outer,
            self.bead_width_inner,
            self.min_bead_width,
            self.wall_count,
        )
    }
}

/// Result of Arachne wall generation.
#[derive(Debug, Clone)]
pub struct ArachneResult {
    /// The generated variable-width wall toolpaths.
    /// Organized by wall index (0 = outer, 1 = first inner, etc.)
    pub toolpaths: Vec<VariableWidthLines>,

    /// The inner contour remaining after all walls.
    /// This is the area available for infill.
    pub inner_contour: ExPolygons,

    /// Thin wall fills (center lines for features too thin for normal walls).
    pub thin_fills: VariableWidthLines,
}

impl ArachneResult {
    // Create a new empty result.
    pub fn new() -> Self {
        Self {
            toolpaths: Vec::new(),
            inner_contour: Vec::new(),
            thin_fills: Vec::new(),
        }
    }

    /// Check if any toolpaths were generated.
    pub fn has_toolpaths(&self) -> bool {
        self.toolpaths.iter().any(|lines| !lines.is_empty())
    }

    /// Get total number of extrusion lines across all walls.
    pub fn total_line_count(&self) -> usize {
        self.toolpaths
            .iter()
            .map(|lines| lines.len())
            .sum::<usize>()
            + self.thin_fills.len()
    }

    /// Get all toolpaths flattened into a single vector.
    pub fn all_toolpaths(&self) -> VariableWidthLines {
        let mut all = Vec::new();
        for lines in &self.toolpaths {
            all.extend(lines.clone());
        }
        all.extend(self.thin_fills.clone());
        all
    }

    /// Get outer wall toolpaths only.
    pub fn outer_walls(&self) -> Option<&VariableWidthLines> {
        self.toolpaths.first()
    }

    /// Get inner wall toolpaths only.
    pub fn inner_walls(&self) -> Vec<&VariableWidthLines> {
        self.toolpaths.iter().skip(1).collect()
    }
}

impl Default for ArachneResult {
    fn default() -> Self {
        Self::new()
    }
}

/// Arachne variable-width wall toolpath generator.
///
/// This generator creates perimeter toolpaths with variable extrusion width,
/// adapting to local geometry for better print quality, especially in thin
/// sections.
pub struct ArachneGenerator {
    config: ArachneConfig,
}

impl ArachneGenerator {
    // Create a new Arachne generator with the given configuration.
    pub fn new(config: ArachneConfig) -> Self {
        Self { config }
    }

    /// Create a generator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(ArachneConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &ArachneConfig {
        &self.config
    }

    /// Get mutable access to the configuration.
    pub fn config_mut(&mut self) -> &mut ArachneConfig {
        &mut self.config
    }

    /// Generate variable-width wall toolpaths for the given outline.
    ///
    /// # Arguments
    /// * `outline` - The ExPolygons representing the slice to generate walls for
    ///
    /// # Returns
    /// An ArachneResult containing the generated toolpaths and remaining infill area.
    pub fn generate(&self, outline: &[ExPolygon]) -> ArachneResult {
        let mut result = ArachneResult::new();

        if outline.is_empty() || self.config.wall_count == 0 {
            result.inner_contour = outline.to_vec();
            return result;
        }

        // Initialize toolpaths for each wall level
        result.toolpaths = vec![Vec::new(); self.config.wall_count];

        let mut current_area = outline.to_vec();

        // Generate each wall level
        for wall_idx in 0..self.config.wall_count {
            let is_outer = wall_idx == 0;
            let bead_width = if is_outer {
                self.config.bead_width_outer
            } else {
                self.config.bead_width_inner
            };

            // Calculate offset distance
            let offset = if wall_idx == 0 {
                bead_width / 2.0 + self.config.wall_0_inset
            } else {
                bead_width
            };

            // Shrink to get wall centerlines
            let wall_area = shrink(&current_area, offset, self.config.join_type);

            if wall_area.is_empty() {
                // No more room for walls
                // Check for thin walls if enabled
                if self.config.print_thin_walls && !current_area.is_empty() {
                    let thin_fills = self.generate_thin_walls(&current_area, wall_idx);
                    result.thin_fills.extend(thin_fills);
                }
                break;
            }

            // Generate wall toolpaths from the offset area
            let walls = self.generate_walls_from_area(&wall_area, bead_width, wall_idx);
            result.toolpaths[wall_idx] = walls;

            // Check for thin regions between current and next wall
            if self.config.print_thin_walls {
                let thin_fills =
                    self.detect_thin_regions(&current_area, &wall_area, bead_width, wall_idx);
                result.thin_fills.extend(thin_fills);
            }

            current_area = wall_area;
        }

        // The remaining area is the inner contour for infill
        // Shrink by half bead width to get the actual infill boundary
        let inner_offset = self.config.bead_width_inner / 2.0;
        result.inner_contour = shrink(&current_area, inner_offset, self.config.join_type);

        result
    }

    /// Generate wall toolpaths from an offset area.
    fn generate_walls_from_area(
        &self,
        area: &[ExPolygon],
        bead_width: CoordF,
        wall_idx: usize,
    ) -> VariableWidthLines {
        let mut lines = Vec::new();
        let width_scaled = scale(bead_width);

        for expoly in area {
            // Generate contour wall
            if !expoly.contour.is_empty() && expoly.contour.len() >= 3 {
                let line = self.polygon_to_extrusion_line(&expoly.contour, width_scaled, wall_idx);
                lines.push(line);
            }

            // Generate hole walls
            for hole in &expoly.holes {
                if !hole.is_empty() && hole.len() >= 3 {
                    let line = self.polygon_to_extrusion_line(hole, width_scaled, wall_idx);
                    lines.push(line);
                }
            }
        }

        lines
    }

    /// Convert a polygon to a constant-width extrusion line.
    fn polygon_to_extrusion_line(
        &self,
        polygon: &Polygon,
        width: Coord,
        wall_idx: usize,
    ) -> ExtrusionLine {
        ExtrusionLine::from_polygon(polygon, width, wall_idx)
    }

    /// Generate thin wall fills for regions that can't fit normal perimeters.
    fn generate_thin_walls(&self, area: &[ExPolygon], wall_idx: usize) -> VariableWidthLines {
        let mut thin_fills = Vec::new();

        // For thin regions, we generate center lines with variable width
        // based on the local thickness of the region

        for expoly in area {
            // Approximate the medial axis by taking the center between inward offsets
            let thin_lines = self.compute_thin_wall_paths(expoly, wall_idx);
            thin_fills.extend(thin_lines);
        }

        thin_fills
    }

    /// Compute thin wall paths using a simplified medial axis approach.
    fn compute_thin_wall_paths(&self, expoly: &ExPolygon, wall_idx: usize) -> VariableWidthLines {
        let mut result = Vec::new();

        let min_width = scale(self.config.min_bead_width);
        let max_width = scale(self.config.bead_width_inner);

        // Try progressively smaller offsets to find the skeleton
        let mut offset = self.config.bead_width_inner / 4.0;
        let min_offset = self.config.min_feature_size / 2.0;

        while offset >= min_offset {
            let shrunk = shrink(&[expoly.clone()], offset, self.config.join_type);

            if shrunk.is_empty() {
                // This offset causes collapse - we've found a thin region
                // Generate a center line with width = 2 * offset
                let width = scale(offset * 2.0).max(min_width).min(max_width);

                // Use the contour as an approximation of the center path
                // (In a full implementation, this would use proper Voronoi/MAT)
                if expoly.contour.len() >= 3 {
                    let mut line = ExtrusionLine::from_polygon(&expoly.contour, width, wall_idx);
                    line.is_odd = true; // Mark as thin wall
                    result.push(line);
                }
                break;
            }

            offset /= 2.0;
        }

        result
    }

    /// Detect thin regions between two wall levels.
    fn detect_thin_regions(
        &self,
        outer_area: &[ExPolygon],
        inner_area: &[ExPolygon],
        _bead_width: CoordF,
        wall_idx: usize,
    ) -> VariableWidthLines {
        let mut thin_fills = Vec::new();

        // Check if there are regions in outer_area that don't have corresponding
        // regions in inner_area (i.e., they collapsed during the offset)

        // Simple heuristic: if inner_area has fewer polygons than outer_area,
        // some regions collapsed and may need thin fills

        if outer_area.len() > inner_area.len() {
            // Some regions collapsed - try to generate thin fills
            // This is a simplified approach; full Arachne uses proper
            // skeletal trapezoidation

            for outer_poly in outer_area {
                let has_matching_inner = inner_area.iter().any(|inner| {
                    // Check if this outer polygon has a corresponding inner polygon
                    // by checking if their bounding boxes overlap significantly
                    let outer_bbox = outer_poly.bounding_box();
                    let inner_bbox = inner.bounding_box();

                    // Simple overlap check
                    outer_bbox.min.x < inner_bbox.max.x
                        && outer_bbox.max.x > inner_bbox.min.x
                        && outer_bbox.min.y < inner_bbox.max.y
                        && outer_bbox.max.y > inner_bbox.min.y
                });

                if !has_matching_inner {
                    // This outer polygon collapsed - generate thin fill
                    let thin_lines = self.compute_thin_wall_paths(outer_poly, wall_idx);
                    thin_fills.extend(thin_lines);
                }
            }
        }

        thin_fills
    }
}

impl Default for ArachneGenerator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Convenience function to generate Arachne walls with default config.
pub fn generate_arachne_walls(outline: &[ExPolygon], wall_count: usize) -> ArachneResult {
    let config = ArachneConfig {
        wall_count,
        ..Default::default()
    };
    ArachneGenerator::new(config).generate(outline)
}

/// Convenience function to generate Arachne walls with custom bead width.
pub fn generate_arachne_walls_with_width(
    outline: &[ExPolygon],
    wall_count: usize,
    bead_width: CoordF,
) -> ArachneResult {
    let config = ArachneConfig::new(wall_count, bead_width);
    ArachneGenerator::new(config).generate(outline)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_mm(x: f64, y: f64, size: f64) -> ExPolygon {
        let poly = Polygon::rectangle(
            Point::new(scale(x), scale(y)),
            Point::new(scale(x + size), scale(y + size)),
        );
        poly.into()
    }

    fn make_thin_rectangle_mm(x: f64, y: f64, width: f64, height: f64) -> ExPolygon {
        let poly = Polygon::rectangle(
            Point::new(scale(x), scale(y)),
            Point::new(scale(x + width), scale(y + height)),
        );
        poly.into()
    }

    #[test]
    fn test_arachne_config_default() {
        let config = ArachneConfig::default();
        assert_eq!(config.wall_count, 3);
        assert!((config.bead_width_outer - 0.45).abs() < 0.001);
    }

    #[test]
    fn test_arachne_config_builder() {
        let config = ArachneConfig::new(4, 0.4)
            .with_thin_walls(true)
            .with_min_bead_width(0.15);

        assert_eq!(config.wall_count, 4);
        assert!((config.bead_width_outer - 0.4).abs() < 0.001);
        assert!(config.print_thin_walls);
        assert!((config.min_bead_width - 0.15).abs() < 0.001);
    }

    #[test]
    fn test_arachne_generator_empty_input() {
        let generator = ArachneGenerator::with_defaults();
        let result = generator.generate(&[]);

        assert!(!result.has_toolpaths());
        assert!(result.inner_contour.is_empty());
    }

    #[test]
    fn test_arachne_generator_simple_square() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let config = ArachneConfig::new(3, 0.45);
        let generator = ArachneGenerator::new(config);

        let result = generator.generate(&[square]);

        assert!(result.has_toolpaths());
        assert_eq!(result.toolpaths.len(), 3); // 3 wall levels

        // Each wall level should have at least one line (the square contour)
        for (idx, lines) in result.toolpaths.iter().enumerate() {
            assert!(!lines.is_empty(), "Wall level {} should have lines", idx);
        }

        // Should have inner contour for infill
        assert!(!result.inner_contour.is_empty());

        println!(
            "Generated {} wall levels with {} total lines",
            result.toolpaths.len(),
            result.total_line_count()
        );
    }

    #[test]
    fn test_arachne_generator_thin_feature() {
        // Create a thin rectangle that can't fit 3 full perimeters
        // Width = 1.0mm, with 0.45mm walls, only ~1 wall fits
        let thin_rect = make_thin_rectangle_mm(0.0, 0.0, 1.0, 20.0);
        let config = ArachneConfig::new(3, 0.45).with_thin_walls(true);
        let generator = ArachneGenerator::new(config);

        let result = generator.generate(&[thin_rect]);

        // Should have some toolpaths (at least outer wall or thin fills)
        assert!(result.has_toolpaths() || !result.thin_fills.is_empty());

        println!(
            "Thin feature: {} regular lines, {} thin fills",
            result.toolpaths.iter().map(|l| l.len()).sum::<usize>(),
            result.thin_fills.len()
        );
    }

    #[test]
    fn test_arachne_generator_zero_walls() {
        let square = make_square_mm(0.0, 0.0, 10.0);
        let config = ArachneConfig {
            wall_count: 0,
            ..Default::default()
        };
        let generator = ArachneGenerator::new(config);

        let result = generator.generate(&[square]);

        assert!(!result.has_toolpaths());
        // Inner contour should be the original outline
        assert!(!result.inner_contour.is_empty());
    }

    #[test]
    fn test_arachne_result_accessors() {
        let square = make_square_mm(0.0, 0.0, 20.0);
        let generator = ArachneGenerator::with_defaults();
        let result = generator.generate(&[square]);

        // Test outer walls accessor
        let outer = result.outer_walls();
        assert!(outer.is_some());
        assert!(!outer.unwrap().is_empty());

        // Test inner walls accessor
        let inner = result.inner_walls();
        assert!(!inner.is_empty()); // Should have 2 inner wall levels

        // Test all_toolpaths
        let all = result.all_toolpaths();
        assert!(!all.is_empty());
    }

    #[test]
    fn test_arachne_config_with_beading_strategy() {
        let config =
            ArachneConfig::new(3, 0.45).with_beading_strategy(BeadingStrategy::InwardDistributed);

        assert_eq!(config.beading_strategy, BeadingStrategy::InwardDistributed);

        // Test that beading calculator is created correctly
        let calc = config.beading_calculator();
        assert_eq!(calc.strategy(), BeadingStrategy::InwardDistributed);
    }

    #[test]
    fn test_arachne_beading_calculator_integration() {
        let config = ArachneConfig::new(3, 0.45)
            .with_beading_strategy(BeadingStrategy::Distributed)
            .with_min_bead_width(0.1);

        let calc = config.beading_calculator();

        // Test beading calculation for various thicknesses
        let result_1bead = calc.calculate(0.4);
        assert!(result_1bead.is_valid);
        assert_eq!(result_1bead.bead_count, 1);

        let result_2beads = calc.calculate(0.9);
        assert!(result_2beads.is_valid);
        assert_eq!(result_2beads.bead_count, 2);

        let result_too_thin = calc.calculate(0.05);
        assert!(!result_too_thin.is_valid);
    }

    #[test]
    fn test_arachne_extrusion_line_properties() {
        let square = make_square_mm(0.0, 0.0, 10.0);
        let generator = ArachneGenerator::with_defaults();
        let result = generator.generate(&[square]);

        if let Some(outer_lines) = result.outer_walls() {
            for line in outer_lines {
                // All outer walls should be closed loops
                assert!(line.is_closed);
                // All should be marked as external
                assert!(line.is_external());
                // Should have reasonable width
                let avg_width = line.average_width_mm();
                assert!(avg_width >= 0.1 && avg_width <= 1.0);
            }
        }
    }

    #[test]
    fn test_convenience_functions() {
        let square = make_square_mm(0.0, 0.0, 15.0);

        let result1 = generate_arachne_walls(&[square.clone()], 2);
        assert_eq!(result1.toolpaths.len(), 2);

        let result2 = generate_arachne_walls_with_width(&[square], 3, 0.4);
        assert_eq!(result2.toolpaths.len(), 3);
    }
}
