//! Spiral vase mode for continuous Z movement.
//!
//! This module implements spiral vase mode similar to BambuStudio's SpiralVase:
//! - Converts layer-by-layer printing into continuous spiral movement
//! - Interpolates Z height throughout each layer's perimeter
//! - Optionally smooths XY coordinates with the previous layer
//! - Handles transition from solid bottom layers to spiral mode

use crate::geometry::{Point, PointF};
use crate::ExtrusionRole;

/// Configuration for spiral vase mode.
#[derive(Debug, Clone)]
pub struct SpiralVaseConfig {
    /// Enable spiral vase mode.
    pub enabled: bool,
    /// Number of solid bottom layers before starting spiral.
    pub bottom_solid_layers: u32,
    /// Enable smooth spiral (interpolate XY with previous layer).
    pub smooth_spiral: bool,
    /// Maximum XY smoothing distance in mm.
    pub max_xy_smoothing: f64,
    /// Layer height in mm.
    pub layer_height: f64,
    /// First layer height in mm.
    pub first_layer_height: f64,
}

impl Default for SpiralVaseConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            bottom_solid_layers: 1,
            smooth_spiral: true,
            max_xy_smoothing: 0.2,
            layer_height: 0.2,
            first_layer_height: 0.3,
        }
    }
}

impl SpiralVaseConfig {
    // Create a new spiral vase config.
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable spiral vase mode.
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set number of solid bottom layers.
    pub fn with_bottom_solid_layers(mut self, layers: u32) -> Self {
        self.bottom_solid_layers = layers;
        self
    }

    /// Enable smooth spiral interpolation.
    pub fn with_smooth_spiral(mut self, smooth: bool) -> Self {
        self.smooth_spiral = smooth;
        self
    }

    /// Set maximum XY smoothing distance.
    pub fn with_max_xy_smoothing(mut self, max: f64) -> Self {
        self.max_xy_smoothing = max;
        self
    }

    /// Set layer height.
    pub fn with_layer_height(mut self, height: f64) -> Self {
        self.layer_height = height;
        self
    }

    /// Set first layer height.
    pub fn with_first_layer_height(mut self, height: f64) -> Self {
        self.first_layer_height = height;
        self
    }

    /// Check if a layer should be spiralized.
    pub fn should_spiralize(&self, layer_index: u32) -> bool {
        self.enabled && layer_index >= self.bottom_solid_layers
    }

    /// Get the Z height at the start of a layer.
    pub fn layer_z(&self, layer_index: u32) -> f64 {
        if layer_index == 0 {
            self.first_layer_height
        } else {
            self.first_layer_height + (layer_index as f64) * self.layer_height
        }
    }
}

/// A point in the spiral with interpolated Z.
#[derive(Debug, Clone, Copy)]
pub struct SpiralPoint {
    /// X coordinate in mm.
    pub x: f64,
    /// Y coordinate in mm.
    pub y: f64,
    /// Interpolated Z coordinate in mm.
    pub z: f64,
    /// Extrusion amount (E value).
    pub e: f64,
}

impl SpiralPoint {
    // Create a new spiral point.
    pub fn new(x: f64, y: f64, z: f64, e: f64) -> Self {
        Self { x, y, z, e }
    }

    /// Create from a Point with Z and E.
    pub fn from_point(point: &Point, z: f64, e: f64) -> Self {
        Self {
            x: crate::unscale(point.x),
            y: crate::unscale(point.y),
            z,
            e,
        }
    }

    /// Create from a PointF with Z and E.
    pub fn from_pointf(point: &PointF, z: f64, e: f64) -> Self {
        Self {
            x: point.x,
            y: point.y,
            z,
            e,
        }
    }

    /// Distance to another point in XY plane.
    pub fn xy_distance(&self, other: &SpiralPoint) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// 3D distance to another point.
    pub fn distance_3d(&self, other: &SpiralPoint) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Interpolate between two points.
    pub fn lerp(&self, other: &SpiralPoint, t: f64) -> SpiralPoint {
        SpiralPoint {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
            z: self.z + (other.z - self.z) * t,
            e: self.e + (other.e - self.e) * t,
        }
    }
}

/// Represents a layer's perimeter for spiral processing.
#[derive(Debug, Clone)]
pub struct SpiralLayer {
    /// Points forming the perimeter loop.
    pub points: Vec<SpiralPoint>,
    /// Total path length in mm.
    pub total_length: f64,
    /// Cumulative lengths at each point.
    pub cumulative_lengths: Vec<f64>,
}

impl SpiralLayer {
    // Create a new spiral layer from points.
    pub fn new(points: Vec<SpiralPoint>) -> Self {
        let mut cumulative_lengths = Vec::with_capacity(points.len());
        let mut total_length = 0.0;

        cumulative_lengths.push(0.0);
        for i in 1..points.len() {
            let dist = points[i - 1].xy_distance(&points[i]);
            total_length += dist;
            cumulative_lengths.push(total_length);
        }

        Self {
            points,
            total_length,
            cumulative_lengths,
        }
    }

    /// Create from XY points with uniform Z.
    pub fn from_xy_points(xy_points: &[PointF], z: f64) -> Self {
        let points: Vec<SpiralPoint> = xy_points
            .iter()
            .map(|p| SpiralPoint::from_pointf(p, z, 0.0))
            .collect();
        Self::new(points)
    }

    /// Get the point at a given fraction along the path (0.0 to 1.0).
    pub fn point_at_fraction(&self, fraction: f64) -> Option<SpiralPoint> {
        if self.points.is_empty() || self.total_length <= 0.0 {
            return None;
        }

        let target_length = fraction.clamp(0.0, 1.0) * self.total_length;

        // Find the segment containing this length
        for i in 1..self.points.len() {
            if self.cumulative_lengths[i] >= target_length {
                let segment_start = self.cumulative_lengths[i - 1];
                let segment_length = self.cumulative_lengths[i] - segment_start;

                if segment_length > 0.0 {
                    let t = (target_length - segment_start) / segment_length;
                    return Some(self.points[i - 1].lerp(&self.points[i], t));
                } else {
                    return Some(self.points[i - 1]);
                }
            }
        }

        self.points.last().copied()
    }

    /// Check if this layer is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Get the number of points.
    pub fn len(&self) -> usize {
        self.points.len()
    }
}

/// Spiral vase processor that converts layer perimeters to spiral moves.
#[derive(Debug)]
pub struct SpiralVase {
    /// Configuration.
    config: SpiralVaseConfig,
    /// Previous layer for smoothing.
    previous_layer: Option<SpiralLayer>,
    /// Whether spiral mode is currently active.
    active: bool,
    /// Whether this is a transition layer (first spiral layer).
    transition_layer: bool,
}

impl SpiralVase {
    // Create a new spiral vase processor.
    pub fn new(config: SpiralVaseConfig) -> Self {
        Self {
            config,
            previous_layer: None,
            active: false,
            transition_layer: false,
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(SpiralVaseConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &SpiralVaseConfig {
        &self.config
    }

    /// Check if spiral mode is active.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Enable spiral mode for subsequent layers.
    pub fn enable(&mut self) {
        if !self.active {
            self.transition_layer = true;
        }
        self.active = true;
    }

    /// Disable spiral mode.
    pub fn disable(&mut self) {
        self.active = false;
        self.transition_layer = false;
    }

    /// Process a layer and convert to spiral moves.
    ///
    /// Returns the processed spiral points, or None if this layer should not be spiralized.
    pub fn process_layer(
        &mut self,
        layer_index: u32,
        perimeter_points: &[PointF],
        layer_z: f64,
        next_layer_z: f64,
    ) -> Option<Vec<SpiralPoint>> {
        // Check if we should spiralize this layer
        if !self.config.should_spiralize(layer_index) {
            self.previous_layer = None;
            return None;
        }

        // Enable spiral mode if this is the first spiral layer
        if layer_index == self.config.bottom_solid_layers {
            self.enable();
        }

        if perimeter_points.is_empty() {
            return None;
        }

        // Create the current layer
        let current_layer = SpiralLayer::from_xy_points(perimeter_points, layer_z);

        if current_layer.is_empty() {
            return None;
        }

        // Calculate Z range for this layer
        let z_start = layer_z;
        let z_end = next_layer_z;
        let z_range = z_end - z_start;

        // Process points with Z interpolation
        let mut result = Vec::with_capacity(current_layer.points.len());
        let total_length = current_layer.total_length;

        if total_length <= 0.0 {
            return None;
        }

        for (i, point) in current_layer.points.iter().enumerate() {
            // Calculate progress along the perimeter (0.0 to 1.0)
            let progress = if i == 0 {
                0.0
            } else {
                current_layer.cumulative_lengths[i] / total_length
            };

            // Interpolate Z based on progress
            let interpolated_z = if self.transition_layer {
                // For transition layer, ramp Z from 0 to layer_height
                z_start + progress * z_range * 0.5
            } else {
                z_start + progress * z_range
            };

            // Apply XY smoothing with previous layer if enabled
            let (final_x, final_y) = if self.config.smooth_spiral {
                self.smooth_xy(point, progress)
            } else {
                (point.x, point.y)
            };

            result.push(SpiralPoint::new(final_x, final_y, interpolated_z, 0.0));
        }

        // Store current layer for next iteration's smoothing
        self.previous_layer = Some(current_layer);
        self.transition_layer = false;

        Some(result)
    }

    /// Smooth XY coordinates with the previous layer.
    fn smooth_xy(&self, current_point: &SpiralPoint, progress: f64) -> (f64, f64) {
        if let Some(ref prev_layer) = self.previous_layer {
            if let Some(prev_point) = prev_layer.point_at_fraction(progress) {
                let dx = prev_point.x - current_point.x;
                let dy = prev_point.y - current_point.y;
                let distance = (dx * dx + dy * dy).sqrt();

                if distance <= self.config.max_xy_smoothing && distance > 0.0 {
                    // Interpolate towards previous layer position
                    let blend = 0.5; // 50% blend
                    let new_x = current_point.x + dx * blend;
                    let new_y = current_point.y + dy * blend;
                    return (new_x, new_y);
                }
            }
        }

        (current_point.x, current_point.y)
    }

    /// Calculate extrusion amounts for spiral points.
    pub fn calculate_extrusion(
        &self,
        points: &mut [SpiralPoint],
        flow_mm3_per_mm: f64,
        filament_area: f64,
    ) {
        if points.is_empty() || filament_area <= 0.0 {
            return;
        }

        let e_per_mm = flow_mm3_per_mm / filament_area;
        let mut cumulative_e = 0.0;

        points[0].e = 0.0;

        for i in 1..points.len() {
            let dist = points[i - 1].distance_3d(&points[i]);
            cumulative_e += dist * e_per_mm;
            points[i].e = cumulative_e;
        }
    }

    /// Reset the processor state.
    pub fn reset(&mut self) {
        self.previous_layer = None;
        self.active = false;
        self.transition_layer = false;
    }
}

impl Default for SpiralVase {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Result of spiral vase processing.
#[derive(Debug, Clone)]
pub struct SpiralResult {
    /// Processed spiral points.
    pub points: Vec<SpiralPoint>,
    /// Whether this is a transition layer.
    pub is_transition: bool,
    /// Starting Z height.
    pub z_start: f64,
    /// Ending Z height.
    pub z_end: f64,
}

impl SpiralResult {
    // Create a new spiral result.
    pub fn new(points: Vec<SpiralPoint>, is_transition: bool, z_start: f64, z_end: f64) -> Self {
        Self {
            points,
            is_transition,
            z_start,
            z_end,
        }
    }

    /// Get the total path length.
    pub fn total_length(&self) -> f64 {
        if self.points.len() < 2 {
            return 0.0;
        }

        let mut length = 0.0;
        for i in 1..self.points.len() {
            length += self.points[i - 1].distance_3d(&self.points[i]);
        }
        length
    }

    /// Get the total extrusion amount.
    pub fn total_extrusion(&self) -> f64 {
        self.points.last().map(|p| p.e).unwrap_or(0.0)
    }

    /// Check if this result is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

/// Check if a layer's paths are suitable for vase mode.
///
/// Vase mode requires exactly one external perimeter loop with no infill or inner walls.
pub fn is_vase_mode_compatible(roles: &[ExtrusionRole], is_closed_loop: bool) -> bool {
    if !is_closed_loop {
        return false;
    }

    // Count external perimeters
    let external_count = roles
        .iter()
        .filter(|r| **r == ExtrusionRole::ExternalPerimeter)
        .count();

    // Should have exactly one external perimeter and no other extrusion types
    // (except possibly gap fill which is acceptable)
    let other_count = roles
        .iter()
        .filter(|r| **r != ExtrusionRole::ExternalPerimeter && **r != ExtrusionRole::GapFill)
        .count();

    external_count == 1 && other_count == 0
}

/// Extract the external perimeter loop from layer paths for vase mode.
pub fn extract_vase_perimeter(paths: &[(Vec<PointF>, ExtrusionRole)]) -> Option<Vec<PointF>> {
    for (points, role) in paths {
        if *role == ExtrusionRole::ExternalPerimeter && !points.is_empty() {
            return Some(points.clone());
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_points() -> Vec<PointF> {
        vec![
            PointF::new(0.0, 0.0),
            PointF::new(10.0, 0.0),
            PointF::new(10.0, 10.0),
            PointF::new(0.0, 10.0),
            PointF::new(0.0, 0.0), // Close the loop
        ]
    }

    #[test]
    fn test_spiral_vase_config_default() {
        let config = SpiralVaseConfig::default();
        assert!(!config.enabled);
        assert_eq!(config.bottom_solid_layers, 1);
        assert!(config.smooth_spiral);
    }

    #[test]
    fn test_spiral_vase_config_builder() {
        let config = SpiralVaseConfig::new()
            .with_enabled(true)
            .with_bottom_solid_layers(3)
            .with_smooth_spiral(false)
            .with_layer_height(0.15);

        assert!(config.enabled);
        assert_eq!(config.bottom_solid_layers, 3);
        assert!(!config.smooth_spiral);
        assert_eq!(config.layer_height, 0.15);
    }

    #[test]
    fn test_should_spiralize() {
        let config = SpiralVaseConfig::new()
            .with_enabled(true)
            .with_bottom_solid_layers(2);

        assert!(!config.should_spiralize(0));
        assert!(!config.should_spiralize(1));
        assert!(config.should_spiralize(2));
        assert!(config.should_spiralize(5));
    }

    #[test]
    fn test_spiral_point_distance() {
        let p1 = SpiralPoint::new(0.0, 0.0, 0.0, 0.0);
        let p2 = SpiralPoint::new(3.0, 4.0, 0.0, 0.0);

        assert!((p1.xy_distance(&p2) - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_spiral_point_lerp() {
        let p1 = SpiralPoint::new(0.0, 0.0, 0.0, 0.0);
        let p2 = SpiralPoint::new(10.0, 20.0, 1.0, 2.0);

        let mid = p1.lerp(&p2, 0.5);
        assert!((mid.x - 5.0).abs() < 0.001);
        assert!((mid.y - 10.0).abs() < 0.001);
        assert!((mid.z - 0.5).abs() < 0.001);
        assert!((mid.e - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_spiral_layer_creation() {
        let points = make_square_points();
        let layer = SpiralLayer::from_xy_points(&points, 0.2);

        assert_eq!(layer.len(), 5);
        assert!(layer.total_length > 0.0);
        // Perimeter of 10x10 square = 40mm
        assert!((layer.total_length - 40.0).abs() < 0.001);
    }

    #[test]
    fn test_spiral_layer_point_at_fraction() {
        let points = vec![PointF::new(0.0, 0.0), PointF::new(10.0, 0.0)];
        let layer = SpiralLayer::from_xy_points(&points, 0.2);

        let start = layer.point_at_fraction(0.0).unwrap();
        assert!((start.x - 0.0).abs() < 0.001);

        let mid = layer.point_at_fraction(0.5).unwrap();
        assert!((mid.x - 5.0).abs() < 0.001);

        let end = layer.point_at_fraction(1.0).unwrap();
        assert!((end.x - 10.0).abs() < 0.001);
    }

    #[test]
    fn test_spiral_vase_process_before_bottom_layers() {
        let config = SpiralVaseConfig::new()
            .with_enabled(true)
            .with_bottom_solid_layers(2);

        let mut vase = SpiralVase::new(config);
        let points = make_square_points();

        // Layer 0 should not be spiralized
        let result = vase.process_layer(0, &points, 0.2, 0.4);
        assert!(result.is_none());

        // Layer 1 should not be spiralized
        let result = vase.process_layer(1, &points, 0.4, 0.6);
        assert!(result.is_none());
    }

    #[test]
    fn test_spiral_vase_process_spiral_layer() {
        let config = SpiralVaseConfig::new()
            .with_enabled(true)
            .with_bottom_solid_layers(1)
            .with_smooth_spiral(false);

        let mut vase = SpiralVase::new(config);
        let points = make_square_points();

        // Layer 1 should be spiralized (after 1 bottom layer)
        let result = vase.process_layer(1, &points, 0.4, 0.6);
        assert!(result.is_some());

        let spiral_points = result.unwrap();
        assert_eq!(spiral_points.len(), points.len());

        // First point should be at layer start Z
        assert!((spiral_points[0].z - 0.4).abs() < 0.01);

        // Last point should be near layer end Z
        // (not exactly at end because it's the transition layer)
    }

    #[test]
    fn test_spiral_vase_z_interpolation() {
        let config = SpiralVaseConfig::new()
            .with_enabled(true)
            .with_bottom_solid_layers(0)
            .with_smooth_spiral(false);

        let mut vase = SpiralVase::new(config);

        // Simple line from (0,0) to (10,0)
        let points = vec![PointF::new(0.0, 0.0), PointF::new(10.0, 0.0)];

        // First call to get past transition layer
        let _ = vase.process_layer(0, &points, 0.0, 0.2);

        // Second call for normal spiral
        let result = vase.process_layer(1, &points, 0.2, 0.4);
        assert!(result.is_some());

        let spiral_points = result.unwrap();

        // Start should be at z_start
        assert!((spiral_points[0].z - 0.2).abs() < 0.001);

        // End should be at z_end
        assert!((spiral_points[1].z - 0.4).abs() < 0.001);
    }

    #[test]
    fn test_calculate_extrusion() {
        let config = SpiralVaseConfig::default();
        let vase = SpiralVase::new(config);

        let mut points = vec![
            SpiralPoint::new(0.0, 0.0, 0.0, 0.0),
            SpiralPoint::new(10.0, 0.0, 0.0, 0.0),
        ];

        // flow_mm3_per_mm = 0.1 mm³/mm, filament_area = 2.405 mm² (1.75mm filament)
        let filament_area = std::f64::consts::PI * (1.75_f64 / 2.0).powi(2);
        vase.calculate_extrusion(&mut points, 0.1, filament_area);

        assert_eq!(points[0].e, 0.0);
        assert!(points[1].e > 0.0);

        // E should be approximately 10mm * 0.1 / filament_area
        let expected_e = 10.0 * 0.1 / filament_area;
        assert!((points[1].e - expected_e).abs() < 0.001);
    }

    #[test]
    fn test_is_vase_mode_compatible() {
        // Single external perimeter, closed loop - compatible
        let roles = vec![ExtrusionRole::ExternalPerimeter];
        assert!(is_vase_mode_compatible(&roles, true));

        // Not a closed loop - incompatible
        assert!(!is_vase_mode_compatible(&roles, false));

        // Multiple types - incompatible
        let roles = vec![ExtrusionRole::ExternalPerimeter, ExtrusionRole::Perimeter];
        assert!(!is_vase_mode_compatible(&roles, true));

        // External perimeter + gap fill - compatible
        let roles = vec![ExtrusionRole::ExternalPerimeter, ExtrusionRole::GapFill];
        assert!(is_vase_mode_compatible(&roles, true));
    }

    #[test]
    fn test_spiral_result() {
        let points = vec![
            SpiralPoint::new(0.0, 0.0, 0.0, 0.0),
            SpiralPoint::new(10.0, 0.0, 0.2, 0.5),
        ];

        let result = SpiralResult::new(points, false, 0.0, 0.2);

        assert!(!result.is_empty());
        assert!((result.total_extrusion() - 0.5).abs() < 0.001);
        assert!(result.total_length() > 10.0); // 3D distance is slightly more than 10
    }

    #[test]
    fn test_spiral_vase_reset() {
        let config = SpiralVaseConfig::new().with_enabled(true);
        let mut vase = SpiralVase::new(config);

        vase.enable();
        assert!(vase.is_active());

        vase.reset();
        assert!(!vase.is_active());
    }
}
