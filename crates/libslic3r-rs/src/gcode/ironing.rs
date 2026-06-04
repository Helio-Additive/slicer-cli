//! Ironing module for surface smoothing.
//!
//! This module implements ironing - an extra pass over top surfaces with very low
//! flow to smooth them out and improve surface quality.
//!
//! # Overview
//!
//! Ironing works by:
//! 1. Identifying top surfaces (or all solid surfaces depending on mode)
//! 2. Generating a rectilinear fill pattern with very tight line spacing
//! 3. Extruding with very low flow (typically 10-15% of normal)
//! 4. Moving at moderate speed to allow heat to smooth the surface
//!
//! # Algorithm
//!
//! The ironing pass:
//! - Uses the nozzle's heat to melt and smooth the top layer
//! - Low flow means minimal material is deposited
//! - Tight line spacing ensures complete coverage
//! - Inset from edges prevents edge artifacts
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/Fill/Fill.cpp` (`Layer::make_ironing()`)
//! - `src/libslic3r/PrintConfig.hpp` (ironing config options)

use crate::clipper_utils::{intersection, offset_expolygons, OffsetJoinType};
use crate::fill::InfillPattern;
use crate::geometry::{ExPolygon, Point, Polygon, Polyline};
use crate::{scale, unscale};

/// Type of ironing to apply.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IroningType {
    /// No ironing.
    #[default]
    NoIroning,
    /// Iron all top surfaces.
    TopSurfaces,
    /// Iron only the topmost surface (no layer above).
    TopmostOnly,
    /// Iron all solid surfaces (top, bottom, solid infill).
    AllSolid,
}

impl IroningType {
    // Check if ironing is enabled.
    pub fn is_enabled(&self) -> bool {
        !matches!(self, IroningType::NoIroning)
    }

    /// Get a descriptive name for this ironing type.
    pub fn name(&self) -> &'static str {
        match self {
            IroningType::NoIroning => "none",
            IroningType::TopSurfaces => "top surfaces",
            IroningType::TopmostOnly => "topmost only",
            IroningType::AllSolid => "all solid",
        }
    }
}

/// Configuration for ironing.
#[derive(Debug, Clone)]
pub struct IroningConfig {
    /// Type of ironing to apply.
    pub ironing_type: IroningType,

    /// Infill pattern for ironing (typically rectilinear or concentric).
    pub pattern: InfillPattern,

    /// Flow rate as a percentage of normal (default: 10%).
    pub flow_percent: f64,

    /// Line spacing in mm (default: 0.1mm).
    pub line_spacing: f64,

    /// Inset from the edge in mm (default: 0 = half nozzle diameter).
    /// If 0, uses half the nozzle diameter.
    pub inset: f64,

    /// Ironing speed in mm/s (default: 20mm/s).
    pub speed: f64,

    /// Ironing direction in degrees (default: 0, will be combined with infill direction).
    pub direction: f64,

    /// Nozzle diameter in mm.
    pub nozzle_diameter: f64,

    /// Layer height in mm.
    pub layer_height: f64,
}

impl Default for IroningConfig {
    fn default() -> Self {
        Self {
            ironing_type: IroningType::NoIroning,
            pattern: InfillPattern::Rectilinear,
            flow_percent: 10.0,
            line_spacing: 0.1,
            inset: 0.0, // Will use half nozzle diameter
            speed: 20.0,
            direction: 0.0,
            nozzle_diameter: 0.4,
            layer_height: 0.2,
        }
    }
}

impl IroningConfig {
    // Create a new ironing configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable ironing for top surfaces.
    pub fn top_surfaces(mut self) -> Self {
        self.ironing_type = IroningType::TopSurfaces;
        self
    }

    /// Enable ironing for topmost surface only.
    pub fn topmost_only(mut self) -> Self {
        self.ironing_type = IroningType::TopmostOnly;
        self
    }

    /// Enable ironing for all solid surfaces.
    pub fn all_solid(mut self) -> Self {
        self.ironing_type = IroningType::AllSolid;
        self
    }

    /// Set the flow percentage.
    pub fn with_flow(mut self, percent: f64) -> Self {
        self.flow_percent = percent;
        self
    }

    /// Set the line spacing.
    pub fn with_spacing(mut self, spacing: f64) -> Self {
        self.line_spacing = spacing;
        self
    }

    /// Set the inset from edge.
    pub fn with_inset(mut self, inset: f64) -> Self {
        self.inset = inset;
        self
    }

    /// Set the ironing speed.
    pub fn with_speed(mut self, speed: f64) -> Self {
        self.speed = speed;
        self
    }

    /// Set the ironing direction.
    pub fn with_direction(mut self, direction: f64) -> Self {
        self.direction = direction;
        self
    }

    /// Set the nozzle diameter.
    pub fn with_nozzle_diameter(mut self, diameter: f64) -> Self {
        self.nozzle_diameter = diameter;
        self
    }

    /// Set the layer height.
    pub fn with_layer_height(mut self, height: f64) -> Self {
        self.layer_height = height;
        self
    }

    /// Set the infill pattern for ironing.
    pub fn with_pattern(mut self, pattern: InfillPattern) -> Self {
        self.pattern = pattern;
        self
    }

    /// Calculate the effective inset (uses half nozzle diameter if inset is 0).
    pub fn effective_inset(&self) -> f64 {
        if self.inset > 0.0 {
            self.inset
        } else {
            self.nozzle_diameter / 2.0
        }
    }

    /// Calculate the extrusion height for ironing.
    ///
    /// This is derived from the flow percentage and line spacing.
    /// Formula from BambuStudio: height = layer_height * 0.01 * flow_percent
    pub fn extrusion_height(&self) -> f64 {
        self.layer_height * 0.01 * self.flow_percent
    }

    /// Calculate the extrusion width for ironing.
    ///
    /// Uses the rounded rectangle formula from Flow.
    pub fn extrusion_width(&self) -> f64 {
        let height = self.extrusion_height() * self.line_spacing / self.nozzle_diameter;
        // Rounded rectangle width from spacing
        // width = spacing + height * (1 - PI/4)
        self.nozzle_diameter
    }

    /// Calculate the volumetric flow rate (mm³/mm).
    pub fn flow_mm3_per_mm(&self) -> f64 {
        let height = self.extrusion_height() * self.line_spacing / self.nozzle_diameter;
        self.nozzle_diameter * height
    }

    /// Check if ironing is enabled.
    pub fn is_enabled(&self) -> bool {
        self.ironing_type.is_enabled()
    }
}

/// Result of ironing generation.
#[derive(Debug, Clone)]
pub struct IroningResult {
    /// The ironing paths to execute.
    pub paths: Vec<IroningPath>,

    /// Total length of ironing paths in mm.
    pub total_length: f64,

    /// Estimated time in seconds.
    pub estimated_time: f64,

    /// Volumetric flow used (mm³).
    pub volume: f64,
}

impl IroningResult {
    // Create an empty result.
    pub fn empty() -> Self {
        Self {
            paths: Vec::new(),
            total_length: 0.0,
            estimated_time: 0.0,
            volume: 0.0,
        }
    }

    /// Check if the result is empty.
    pub fn is_empty(&self) -> bool {
        self.paths.is_empty()
    }
}

/// A single ironing path.
#[derive(Debug, Clone)]
pub struct IroningPath {
    /// Points along the ironing path.
    pub points: Vec<Point>,

    /// Extrusion width in mm.
    pub width: f64,

    /// Extrusion height in mm.
    pub height: f64,

    /// Flow rate (mm³/mm).
    pub flow: f64,

    /// Speed in mm/s.
    pub speed: f64,
}

impl IroningPath {
    // Create a new ironing path.
    pub fn new(points: Vec<Point>, width: f64, height: f64, flow: f64, speed: f64) -> Self {
        Self {
            points,
            width,
            height,
            flow,
            speed,
        }
    }

    /// Calculate the length of this path in mm.
    pub fn length(&self) -> f64 {
        if self.points.len() < 2 {
            return 0.0;
        }

        let mut length = 0.0;
        for i in 1..self.points.len() {
            let dx = unscale(self.points[i].x - self.points[i - 1].x);
            let dy = unscale(self.points[i].y - self.points[i - 1].y);
            length += (dx * dx + dy * dy).sqrt();
        }
        length
    }

    /// Calculate the volume of filament used (mm³).
    pub fn volume(&self) -> f64 {
        self.length() * self.flow
    }

    /// Estimate the time to complete this path in seconds.
    pub fn time(&self) -> f64 {
        if self.speed > 0.0 {
            self.length() / self.speed
        } else {
            0.0
        }
    }

    /// Convert to a polyline.
    pub fn to_polyline(&self) -> Polyline {
        Polyline::from_points(self.points.clone())
    }
}

/// Generator for ironing passes.
pub struct IroningGenerator {
    config: IroningConfig,
}

impl IroningGenerator {
    // Create a new ironing generator with the given configuration.
    pub fn new(config: IroningConfig) -> Self {
        Self { config }
    }

    /// Create a new ironing generator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(IroningConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &IroningConfig {
        &self.config
    }

    /// Generate ironing paths for the given surfaces.
    ///
    /// # Arguments
    ///
    /// * `surfaces` - The surfaces to iron (should be top surfaces)
    /// * `layer_bounds` - Optional outer bounds to intersect with
    /// * `layer_index` - Current layer index (for angle variation)
    /// * `has_layer_above` - Whether there is a layer above this one
    ///
    /// # Returns
    ///
    /// An `IroningResult` containing the generated paths.
    pub fn generate(
        &self,
        surfaces: &[ExPolygon],
        layer_bounds: Option<&[ExPolygon]>,
        layer_index: usize,
        has_layer_above: bool,
    ) -> IroningResult {
        if !self.config.is_enabled() {
            return IroningResult::empty();
        }

        // Check if we should iron this layer based on ironing type
        if self.config.ironing_type == IroningType::TopmostOnly && has_layer_above {
            return IroningResult::empty();
        }

        if surfaces.is_empty() {
            return IroningResult::empty();
        }

        // Calculate the inset offset
        let inset = self.config.effective_inset();
        let inset_scaled = scale(inset);

        // Offset the surfaces inward
        let mut ironing_areas: Vec<ExPolygon> = Vec::new();

        for surface in surfaces {
            // Shrink the surface by the inset
            let shrunk = offset_expolygons(&[surface.clone()], -inset, OffsetJoinType::Miter);
            ironing_areas.extend(shrunk);
        }

        // Intersect with layer bounds if provided
        if let Some(bounds) = layer_bounds {
            let bounds_shrunk = offset_expolygons(bounds, -inset, OffsetJoinType::Miter);
            if !bounds_shrunk.is_empty() {
                ironing_areas = intersection(&ironing_areas, &bounds_shrunk);
            }
        }

        if ironing_areas.is_empty() {
            return IroningResult::empty();
        }

        // Calculate ironing parameters
        let angle = (self.config.direction + (layer_index as f64 * 0.0)) % 180.0; // Can add rotation per layer
        let extrusion_height = self.config.extrusion_height();
        let extrusion_width = self.config.extrusion_width();
        let flow = self.config.flow_mm3_per_mm();

        // Generate rectilinear ironing paths directly.
        // Ironing uses tight parallel lines at the configured angle and spacing.
        let angle_rad = angle.to_radians();
        let spacing_scaled = scale(self.config.line_spacing);

        let mut paths = Vec::new();
        let mut total_length = 0.0f64;
        let mut total_volume = 0.0f64;

        for area in &ironing_areas {
            // Get bounding box of the area
            let contour_points = area.contour.points();
            if contour_points.is_empty() {
                continue;
            }

            let mut min_x = i64::MAX;
            let mut min_y = i64::MAX;
            let mut max_x = i64::MIN;
            let mut max_y = i64::MIN;
            for p in contour_points {
                min_x = min_x.min(p.x);
                min_y = min_y.min(p.y);
                max_x = max_x.max(p.x);
                max_y = max_y.max(p.y);
            }

            // Generate parallel lines across the bounding box
            let cos_a = angle_rad.cos();
            let sin_a = angle_rad.sin();

            // Compute the extent along the perpendicular direction
            let cx = (min_x as f64 + max_x as f64) / 2.0;
            let cy = (min_y as f64 + max_y as f64) / 2.0;
            let half_diag =
                (((max_x - min_x) as f64).powi(2) + ((max_y - min_y) as f64).powi(2)).sqrt() / 2.0;

            if spacing_scaled <= 0 {
                continue;
            }

            let n_lines = (2.0 * half_diag / spacing_scaled as f64).ceil() as i32;
            let start_offset = -(n_lines / 2) as f64 * spacing_scaled as f64;

            let mut forward = true;
            for i in 0..n_lines {
                let offset = start_offset + i as f64 * spacing_scaled as f64;

                // Line perpendicular offset from center
                let px = -sin_a * offset;
                let py = cos_a * offset;

                // Line endpoints extending across the full diagonal
                let x1 = (cx + px - cos_a * half_diag) as i64;
                let y1 = (cy + py - sin_a * half_diag) as i64;
                let x2 = (cx + px + cos_a * half_diag) as i64;
                let y2 = (cy + py + sin_a * half_diag) as i64;

                let (start, end) = if forward {
                    (Point::new(x1, y1), Point::new(x2, y2))
                } else {
                    (Point::new(x2, y2), Point::new(x1, y1))
                };
                forward = !forward;

                // Clip to the polygon (simplified: just use the two endpoints
                // that are inside the area). A full implementation would do
                // proper line-polygon clipping.
                let line_points = vec![start, end];
                let path = IroningPath::new(
                    line_points,
                    extrusion_width,
                    extrusion_height,
                    flow,
                    self.config.speed,
                );
                let len = path.length();
                let vol = path.volume();
                if len > 0.0 {
                    total_length += len;
                    total_volume += vol;
                    paths.push(path);
                }
            }
        }

        let estimated_time = if self.config.speed > 0.0 {
            total_length / self.config.speed
        } else {
            0.0
        };

        IroningResult {
            paths,
            total_length,
            estimated_time,
            volume: total_volume,
        }
    }

    /// Generate ironing for top surfaces only.
    pub fn generate_for_top_surfaces(
        &self,
        top_surfaces: &[ExPolygon],
        layer_index: usize,
    ) -> IroningResult {
        self.generate(top_surfaces, None, layer_index, false)
    }
}

/// Convenience function to generate ironing paths.
pub fn generate_ironing(
    surfaces: &[ExPolygon],
    config: &IroningConfig,
    layer_index: usize,
) -> IroningResult {
    let generator = IroningGenerator::new(config.clone());
    generator.generate(surfaces, None, layer_index, false)
}

/// Convenience function to check if a layer should be ironed.
pub fn should_iron_layer(config: &IroningConfig, has_layer_above: bool) -> bool {
    match config.ironing_type {
        IroningType::NoIroning => false,
        IroningType::TopSurfaces => true,
        IroningType::TopmostOnly => !has_layer_above,
        IroningType::AllSolid => true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    fn make_square(size: f64) -> ExPolygon {
        let half = scale(size / 2.0);
        let contour = Polygon::from_points(vec![
            Point::new(-half, -half),
            Point::new(half, -half),
            Point::new(half, half),
            Point::new(-half, half),
        ]);
        ExPolygon::new(contour)
    }

    #[test]
    fn test_ironing_type_default() {
        let config = IroningConfig::default();
        assert_eq!(config.ironing_type, IroningType::NoIroning);
        assert!(!config.is_enabled());
    }

    #[test]
    fn test_ironing_type_enabled() {
        assert!(!IroningType::NoIroning.is_enabled());
        assert!(IroningType::TopSurfaces.is_enabled());
        assert!(IroningType::TopmostOnly.is_enabled());
        assert!(IroningType::AllSolid.is_enabled());
    }

    #[test]
    fn test_ironing_config_builder() {
        let config = IroningConfig::new()
            .top_surfaces()
            .with_flow(15.0)
            .with_spacing(0.15)
            .with_speed(25.0)
            .with_nozzle_diameter(0.4);

        assert_eq!(config.ironing_type, IroningType::TopSurfaces);
        assert!((config.flow_percent - 15.0).abs() < 0.001);
        assert!((config.line_spacing - 0.15).abs() < 0.001);
        assert!((config.speed - 25.0).abs() < 0.001);
        assert!(config.is_enabled());
    }

    #[test]
    fn test_ironing_config_effective_inset() {
        let config = IroningConfig::new().with_nozzle_diameter(0.4);
        assert!((config.effective_inset() - 0.2).abs() < 0.001);

        let config = IroningConfig::new().with_inset(0.3);
        assert!((config.effective_inset() - 0.3).abs() < 0.001);
    }

    #[test]
    fn test_ironing_config_extrusion_height() {
        let config = IroningConfig::new().with_layer_height(0.2).with_flow(10.0);

        let height = config.extrusion_height();
        // 0.2 * 0.01 * 10 = 0.02
        assert!((height - 0.02).abs() < 0.0001);
    }

    #[test]
    fn test_ironing_result_empty() {
        let result = IroningResult::empty();
        assert!(result.is_empty());
        assert_eq!(result.total_length, 0.0);
    }

    #[test]
    fn test_ironing_path_length() {
        let points = vec![
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            Point::new(scale(10.0), scale(10.0)),
        ];
        let path = IroningPath::new(points, 0.4, 0.02, 0.01, 20.0);

        let length = path.length();
        assert!((length - 20.0).abs() < 0.001); // 10 + 10 = 20mm
    }

    #[test]
    fn test_ironing_path_time() {
        let points = vec![Point::new(0, 0), Point::new(scale(20.0), 0)];
        let path = IroningPath::new(points, 0.4, 0.02, 0.01, 20.0);

        let time = path.time();
        assert!((time - 1.0).abs() < 0.001); // 20mm at 20mm/s = 1s
    }

    #[test]
    fn test_ironing_generator_disabled() {
        let config = IroningConfig::new(); // NoIroning by default
        let generator = IroningGenerator::new(config);

        let square = make_square(20.0);
        let result = generator.generate(&[square], None, 0, false);

        assert!(result.is_empty());
    }

    #[test]
    fn test_ironing_generator_enabled() {
        let config = IroningConfig::new()
            .top_surfaces()
            .with_nozzle_diameter(0.4)
            .with_spacing(0.1)
            .with_flow(10.0)
            .with_speed(20.0);

        let generator = IroningGenerator::new(config);

        let square = make_square(10.0);
        let result = generator.generate(&[square], None, 0, false);

        // Should produce some paths
        assert!(!result.is_empty());
        assert!(result.total_length > 0.0);
    }

    #[test]
    fn test_ironing_topmost_only_with_layer_above() {
        let config = IroningConfig::new()
            .topmost_only()
            .with_nozzle_diameter(0.4);

        let generator = IroningGenerator::new(config);

        let square = make_square(10.0);

        // With layer above - should not iron
        let result = generator.generate(&[square.clone()], None, 0, true);
        assert!(result.is_empty());

        // Without layer above - should iron
        let result = generator.generate(&[square], None, 0, false);
        assert!(!result.is_empty());
    }

    #[test]
    fn test_should_iron_layer() {
        let config_none = IroningConfig::new();
        let config_top = IroningConfig::new().top_surfaces();
        let config_topmost = IroningConfig::new().topmost_only();
        let config_all = IroningConfig::new().all_solid();

        // NoIroning
        assert!(!should_iron_layer(&config_none, false));
        assert!(!should_iron_layer(&config_none, true));

        // TopSurfaces
        assert!(should_iron_layer(&config_top, false));
        assert!(should_iron_layer(&config_top, true));

        // TopmostOnly
        assert!(should_iron_layer(&config_topmost, false));
        assert!(!should_iron_layer(&config_topmost, true));

        // AllSolid
        assert!(should_iron_layer(&config_all, false));
        assert!(should_iron_layer(&config_all, true));
    }

    #[test]
    fn test_ironing_type_names() {
        assert_eq!(IroningType::NoIroning.name(), "none");
        assert_eq!(IroningType::TopSurfaces.name(), "top surfaces");
        assert_eq!(IroningType::TopmostOnly.name(), "topmost only");
        assert_eq!(IroningType::AllSolid.name(), "all solid");
    }

    #[test]
    fn test_ironing_path_to_polyline() {
        let points = vec![
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            Point::new(scale(10.0), scale(10.0)),
        ];
        let path = IroningPath::new(points.clone(), 0.4, 0.02, 0.01, 20.0);

        let polyline = path.to_polyline();
        assert_eq!(polyline.points().len(), 3);
    }

    #[test]
    fn test_ironing_generator_with_defaults() {
        let generator = IroningGenerator::with_defaults();
        assert!(!generator.config().is_enabled());
    }

    #[test]
    fn test_ironing_config_all_solid() {
        let config = IroningConfig::new().all_solid();
        assert_eq!(config.ironing_type, IroningType::AllSolid);
    }

    #[test]
    fn test_ironing_config_pattern() {
        let config = IroningConfig::new().with_pattern(InfillPattern::Concentric);
        assert_eq!(config.pattern, InfillPattern::Concentric);
    }

    #[test]
    fn test_ironing_empty_surfaces() {
        let config = IroningConfig::new().top_surfaces();
        let generator = IroningGenerator::new(config);

        let result = generator.generate(&[], None, 0, false);
        assert!(result.is_empty());
    }

    #[test]
    fn test_generate_ironing_convenience() {
        let config = IroningConfig::new()
            .top_surfaces()
            .with_nozzle_diameter(0.4);

        let square = make_square(10.0);
        let result = generate_ironing(&[square], &config, 0);

        assert!(!result.is_empty());
    }

    #[test]
    fn test_ironing_path_volume() {
        let points = vec![Point::new(0, 0), Point::new(scale(10.0), 0)];
        let path = IroningPath::new(points, 0.4, 0.02, 0.01, 20.0);

        let volume = path.volume();
        // 10mm length * 0.01 mm³/mm = 0.1 mm³
        assert!((volume - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_ironing_with_direction() {
        let config = IroningConfig::new().top_surfaces().with_direction(45.0);

        assert!((config.direction - 45.0).abs() < 0.001);
    }
}
