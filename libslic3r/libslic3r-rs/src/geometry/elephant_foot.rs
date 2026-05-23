//! Elephant foot compensation for first layer shrinkage.
//!
//! This module implements elephant foot compensation similar to BambuStudio's
//! ElephantFootCompensation. The first layer of a 3D print often spreads out
//! more than intended due to:
//! - Bed adhesion squishing the filament
//! - Higher first layer temperature
//! - Closer nozzle distance
//!
//! This compensation shrinks the first layer contours to counteract the spreading.

use crate::clipper_utils::{offset_polygon, OffsetJoinType};
use crate::geometry::{ExPolygon, Polygon};
use crate::{scale, Coord};

/// Configuration for elephant foot compensation.
#[derive(Debug, Clone)]
pub struct ElephantFootConfig {
    /// Compensation amount in mm (how much to shrink the first layer).
    pub compensation: f64,
    /// Minimum contour width to preserve (in mm).
    /// Contours narrower than this won't be compensated to avoid disappearing.
    pub min_contour_width: f64,
    /// Whether compensation is enabled.
    pub enabled: bool,
    /// Apply compensation to external perimeters only.
    pub external_only: bool,
    /// Smoothing factor for the compensation (0.0 = sharp, 1.0 = smooth).
    pub smoothing: f64,
}

impl Default for ElephantFootConfig {
    fn default() -> Self {
        Self {
            compensation: 0.2,
            min_contour_width: 0.4,
            enabled: true,
            external_only: true,
            smoothing: 0.5,
        }
    }
}

impl ElephantFootConfig {
    // Create a new elephant foot config.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the compensation amount in mm.
    pub fn with_compensation(mut self, compensation: f64) -> Self {
        self.compensation = compensation.max(0.0);
        self
    }

    /// Set the minimum contour width in mm.
    pub fn with_min_contour_width(mut self, width: f64) -> Self {
        self.min_contour_width = width.max(0.0);
        self
    }

    /// Enable or disable compensation.
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set whether to apply to external perimeters only.
    pub fn with_external_only(mut self, external_only: bool) -> Self {
        self.external_only = external_only;
        self
    }

    /// Set the smoothing factor.
    pub fn with_smoothing(mut self, smoothing: f64) -> Self {
        self.smoothing = smoothing.clamp(0.0, 1.0);
        self
    }

    /// Create config from external perimeter flow width.
    pub fn from_flow(external_perimeter_width: f64, compensation: f64) -> Self {
        Self {
            compensation,
            min_contour_width: external_perimeter_width * 0.5,
            enabled: compensation > 0.0,
            external_only: true,
            smoothing: 0.5,
        }
    }

    /// Get the compensation as scaled coordinate.
    pub fn scaled_compensation(&self) -> Coord {
        scale(self.compensation)
    }

    /// Get the minimum contour width as scaled coordinate.
    pub fn scaled_min_contour_width(&self) -> Coord {
        scale(self.min_contour_width)
    }
}

/// Apply elephant foot compensation to a polygon.
///
/// This shrinks the polygon by the compensation amount, but ensures that
/// narrow features are preserved by checking against the minimum contour width.
///
/// # Arguments
/// * `polygon` - The polygon to compensate
/// * `compensation` - The amount to shrink in mm
/// * `min_contour_width` - Minimum width to preserve in mm
///
/// # Returns
/// The compensated polygon, or the original if compensation would eliminate it.
pub fn compensate_polygon(
    polygon: &Polygon,
    compensation: f64,
    min_contour_width: f64,
) -> Option<Polygon> {
    if compensation <= 0.0 {
        return Some(polygon.clone());
    }

    // Check if the polygon is wide enough to compensate
    if !can_compensate_polygon(polygon, compensation, min_contour_width) {
        // Return original polygon if it's too narrow
        return Some(polygon.clone());
    }

    // Shrink the polygon by the compensation amount
    let offset_amount = -compensation;
    let results = offset_polygon(polygon, offset_amount, OffsetJoinType::Round);

    // Return the first result's contour if any (there should be at most one for a simple shrink)
    results.into_iter().next().map(|ep| ep.contour)
}

/// Apply elephant foot compensation to an ExPolygon.
///
/// Compensates both the outer contour and holes appropriately.
///
/// # Arguments
/// * `expolygon` - The ExPolygon to compensate
/// * `compensation` - The amount to shrink in mm
/// * `min_contour_width` - Minimum width to preserve in mm
///
/// # Returns
/// The compensated ExPolygon, or the original if compensation would eliminate it.
/// Apply elephant foot compensation to a single ExPolygon.
///
/// BambuStudio only compensates the **outer contour** — holes (internal features
/// like cabin spaces, smokestack openings, etc.) are NOT shrunk because the
/// elephant foot effect only occurs at the build-plate contact edge.
///
/// The approach:
/// 1. Shrink the outer contour only (using `offset_polygon`)
/// 2. Re-intersect the shrunk contour with the original holes to preserve them
/// 3. Return all resulting ExPolygons (shrinking can split the contour)
///
/// Previously this used `offset_expolygon` which shrinks the entire ExPolygon
/// including holes, causing holes to close and dramatically reducing perimeter
/// count (e.g. from 11 perimeters to 2 on the 3DBenchy layer 0).
///
/// # Arguments
/// * `expolygon` - The ExPolygon to compensate
/// * `compensation` - The amount to shrink in mm
/// * `min_contour_width` - Minimum width to preserve in mm
///
/// # Returns
/// All compensated ExPolygons (may be more than one if the shrink splits the shape).
pub fn compensate_expolygon(
    expolygon: &ExPolygon,
    compensation: f64,
    min_contour_width: f64,
) -> Vec<ExPolygon> {
    if compensation <= 0.0 {
        return vec![expolygon.clone()];
    }

    // Check if the expolygon is wide enough
    if !can_compensate_expolygon(expolygon, compensation, min_contour_width) {
        return vec![expolygon.clone()];
    }

    // Step 1: Shrink ONLY the outer contour (not holes).
    // offset_polygon returns ExPolygons from offsetting a single polygon.
    let offset_amount = -compensation;
    let shrunk_contours = offset_polygon(&expolygon.contour, offset_amount, OffsetJoinType::Round);

    if shrunk_contours.is_empty() {
        // Shrink eliminated the contour entirely — return original unchanged
        return vec![expolygon.clone()];
    }

    // Step 2: Re-apply the original holes to each shrunk contour piece.
    // This preserves internal features (holes) that the elephant foot should not affect.
    if expolygon.holes.is_empty() {
        // No holes — just return the shrunk contours as-is
        return shrunk_contours;
    }

    let mut results = Vec::new();
    for shrunk in &shrunk_contours {
        // Build ExPolygon with the shrunk contour and original holes
        // Only include holes that are actually inside this contour piece
        let mut holes = Vec::new();
        for hole in &expolygon.holes {
            // Check if hole's bounding box is inside the shrunk contour's bounding box
            let hole_bb = hole.bounding_box();
            let contour_bb = shrunk.contour.bounding_box();
            if hole_bb.min.x >= contour_bb.min.x
                && hole_bb.max.x <= contour_bb.max.x
                && hole_bb.min.y >= contour_bb.min.y
                && hole_bb.max.y <= contour_bb.max.y
            {
                holes.push(hole.clone());
            }
        }

        let mut ep = ExPolygon::new(shrunk.contour.clone());
        for hole in holes {
            ep.add_hole(hole);
        }
        results.push(ep);
    }

    if results.is_empty() {
        vec![expolygon.clone()]
    } else {
        results
    }
}

/// Apply elephant foot compensation to multiple ExPolygons.
///
/// # Arguments
/// * `expolygons` - The ExPolygons to compensate
/// * `compensation` - The amount to shrink in mm
/// * `min_contour_width` - Minimum width to preserve in mm
///
/// # Returns
/// The compensated ExPolygons.
pub fn compensate_expolygons(
    expolygons: &[ExPolygon],
    compensation: f64,
    min_contour_width: f64,
) -> Vec<ExPolygon> {
    if compensation <= 0.0 {
        return expolygons.to_vec();
    }

    let mut results = Vec::new();

    for expoly in expolygons {
        let compensated = compensate_expolygon(expoly, compensation, min_contour_width);
        results.extend(compensated);
    }

    results
}

/// Check if a polygon can be compensated without disappearing.
///
/// This checks if the polygon has sufficient width to survive the compensation.
fn can_compensate_polygon(polygon: &Polygon, compensation: f64, min_contour_width: f64) -> bool {
    // A polygon can be compensated if its minimum "width" is at least
    // 2 * compensation + min_contour_width
    // (compensation on both sides plus minimum remaining width)

    let min_required_width = 2.0 * compensation + min_contour_width;

    // Estimate the polygon's minimum width using bounding box
    // This is a conservative estimate
    let bbox = polygon.bounding_box();
    let bbox_width = crate::unscale(bbox.max.x - bbox.min.x);
    let bbox_height = crate::unscale(bbox.max.y - bbox.min.y);
    let min_dimension = bbox_width.min(bbox_height);

    min_dimension >= min_required_width
}

/// Check if an ExPolygon can be compensated without disappearing.
fn can_compensate_expolygon(
    expolygon: &ExPolygon,
    compensation: f64,
    min_contour_width: f64,
) -> bool {
    can_compensate_polygon(&expolygon.contour, compensation, min_contour_width)
}

/// Elephant foot compensator that processes first layer geometry.
#[derive(Debug)]
pub struct ElephantFootCompensator {
    /// Configuration.
    config: ElephantFootConfig,
}

impl ElephantFootCompensator {
    // Create a new compensator with the given configuration.
    pub fn new(config: ElephantFootConfig) -> Self {
        Self { config }
    }

    /// Create a compensator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(ElephantFootConfig::default())
    }

    /// Create a compensator from flow parameters.
    pub fn from_flow(external_perimeter_width: f64, compensation: f64) -> Self {
        Self::new(ElephantFootConfig::from_flow(
            external_perimeter_width,
            compensation,
        ))
    }

    /// Get the configuration.
    pub fn config(&self) -> &ElephantFootConfig {
        &self.config
    }

    /// Check if compensation is enabled.
    pub fn is_enabled(&self) -> bool {
        self.config.enabled && self.config.compensation > 0.0
    }

    /// Compensate a single polygon.
    pub fn compensate_polygon(&self, polygon: &Polygon) -> Option<Polygon> {
        if !self.is_enabled() {
            return Some(polygon.clone());
        }

        compensate_polygon(
            polygon,
            self.config.compensation,
            self.config.min_contour_width,
        )
    }

    /// Compensate a single ExPolygon, returning all resulting pieces.
    pub fn compensate_expolygon(&self, expolygon: &ExPolygon) -> Vec<ExPolygon> {
        if !self.is_enabled() {
            return vec![expolygon.clone()];
        }

        compensate_expolygon(
            expolygon,
            self.config.compensation,
            self.config.min_contour_width,
        )
    }

    /// Compensate multiple ExPolygons.
    pub fn compensate_expolygons(&self, expolygons: &[ExPolygon]) -> Vec<ExPolygon> {
        if !self.is_enabled() {
            return expolygons.to_vec();
        }

        compensate_expolygons(
            expolygons,
            self.config.compensation,
            self.config.min_contour_width,
        )
    }

    /// Process first layer slices with elephant foot compensation.
    ///
    /// This is the main entry point for compensating first layer geometry.
    pub fn process_first_layer(&self, slices: &[ExPolygon]) -> Vec<ExPolygon> {
        self.compensate_expolygons(slices)
    }
}

impl Default for ElephantFootCompensator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Calculate the recommended compensation amount based on first layer parameters.
///
/// # Arguments
/// * `first_layer_height` - First layer height in mm
/// * `nozzle_diameter` - Nozzle diameter in mm
/// * `first_layer_flow_ratio` - First layer flow ratio (e.g., 1.0 for 100%)
///
/// # Returns
/// Recommended compensation amount in mm
pub fn calculate_compensation(
    first_layer_height: f64,
    nozzle_diameter: f64,
    first_layer_flow_ratio: f64,
) -> f64 {
    // The elephant foot effect is proportional to how much the first layer
    // is squished. A thinner first layer relative to nozzle size means more squish.

    // Base compensation is about 20% of the expected spread
    let squish_ratio = (nozzle_diameter - first_layer_height) / nozzle_diameter;
    let base_compensation = squish_ratio * nozzle_diameter * 0.5;

    // Adjust for flow ratio - higher flow means more spread
    let flow_adjustment = (first_layer_flow_ratio - 1.0) * nozzle_diameter * 0.3;

    // Total compensation, clamped to reasonable values
    (base_compensation + flow_adjustment).clamp(0.0, nozzle_diameter * 0.5)
}

/// Calculate the elephant foot spacing for perimeter detection.
///
/// This returns the spacing value used to detect narrow parts where
/// elephant foot compensation cannot be applied.
///
/// Based on BambuStudio's Flow::scaled_elephant_foot_spacing()
pub fn elephant_foot_spacing(extrusion_width: f64, extrusion_spacing: f64) -> f64 {
    // Enable some perimeter squish (INSET_OVERLAP_TOLERANCE)
    // Allow 0.2x external perimeter spacing overlap for elephant foot compensation
    0.5 * (extrusion_width + 0.6 * extrusion_spacing)
}

/// Calculate scaled elephant foot spacing.
pub fn scaled_elephant_foot_spacing(extrusion_width: f64, extrusion_spacing: f64) -> Coord {
    scale(elephant_foot_spacing(extrusion_width, extrusion_spacing))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

    fn make_square_polygon(size_mm: f64) -> Polygon {
        let half = scale(size_mm / 2.0);
        Polygon::from_points(vec![
            Point::new(-half, -half),
            Point::new(half, -half),
            Point::new(half, half),
            Point::new(-half, half),
        ])
    }

    fn make_square_expolygon(size_mm: f64) -> ExPolygon {
        ExPolygon::new(make_square_polygon(size_mm))
    }

    #[test]
    fn test_config_default() {
        let config = ElephantFootConfig::default();
        assert!(config.enabled);
        assert_eq!(config.compensation, 0.2);
        assert_eq!(config.min_contour_width, 0.4);
    }

    #[test]
    fn test_config_builder() {
        let config = ElephantFootConfig::new()
            .with_compensation(0.3)
            .with_min_contour_width(0.5)
            .with_enabled(false)
            .with_smoothing(0.8);

        assert!(!config.enabled);
        assert_eq!(config.compensation, 0.3);
        assert_eq!(config.min_contour_width, 0.5);
        assert_eq!(config.smoothing, 0.8);
    }

    #[test]
    fn test_config_from_flow() {
        let config = ElephantFootConfig::from_flow(0.45, 0.15);

        assert!(config.enabled);
        assert_eq!(config.compensation, 0.15);
        assert_eq!(config.min_contour_width, 0.225); // 0.45 * 0.5
    }

    #[test]
    fn test_compensate_polygon_disabled() {
        let polygon = make_square_polygon(10.0);
        let result = compensate_polygon(&polygon, 0.0, 0.4);

        assert!(result.is_some());
        assert_eq!(result.unwrap().points().len(), polygon.points().len());
    }

    #[test]
    fn test_compensate_polygon_large_square() {
        let polygon = make_square_polygon(10.0);
        let result = compensate_polygon(&polygon, 0.2, 0.4);

        assert!(result.is_some());
        let compensated = result.unwrap();

        // The compensated polygon should be smaller
        let original_bbox = polygon.bounding_box();
        let compensated_bbox = compensated.bounding_box();

        let original_width = original_bbox.max.x - original_bbox.min.x;
        let compensated_width = compensated_bbox.max.x - compensated_bbox.min.x;

        assert!(compensated_width < original_width);
    }

    #[test]
    fn test_compensate_expolygons() {
        let expolys = vec![make_square_expolygon(10.0), make_square_expolygon(5.0)];
        let result = compensate_expolygons(&expolys, 0.5, 0.8);
        // Both squares should survive a 0.5mm shrink (they're 10mm and 5mm wide).
        // Each input may produce one or more output ExPolygons if the shrink
        // splits the shape, so we expect at least 2.
        assert!(result.len() >= 2);
    }

    #[test]
    fn test_can_compensate_large_polygon() {
        let polygon = make_square_polygon(10.0);
        assert!(can_compensate_polygon(&polygon, 0.2, 0.4));
    }

    #[test]
    fn test_can_compensate_small_polygon() {
        // A 0.5mm square with 0.2mm compensation and 0.4mm min width
        // would need 0.8mm minimum, so it can't be compensated
        let polygon = make_square_polygon(0.5);
        assert!(!can_compensate_polygon(&polygon, 0.2, 0.4));
    }

    #[test]
    fn test_compensator_creation() {
        let compensator = ElephantFootCompensator::with_defaults();
        assert!(compensator.is_enabled());
    }

    #[test]
    fn test_compensator_disabled() {
        let config = ElephantFootConfig::new().with_enabled(false);
        let compensator = ElephantFootCompensator::new(config);

        assert!(!compensator.is_enabled());

        let polygon = make_square_polygon(10.0);
        let result = compensator.compensate_polygon(&polygon);

        // Should return original when disabled
        assert!(result.is_some());
    }

    #[test]
    fn test_compensator_from_flow() {
        let compensator = ElephantFootCompensator::from_flow(0.45, 0.2);

        assert!(compensator.is_enabled());
        assert_eq!(compensator.config().compensation, 0.2);
    }

    #[test]
    fn test_process_first_layer() {
        let compensator = ElephantFootCompensator::with_defaults();
        let slices = vec![make_square_expolygon(10.0), make_square_expolygon(15.0)];

        let results = compensator.process_first_layer(&slices);

        // Each input ExPolygon may produce one or more output ExPolygons
        // (if the shrink splits the shape), so we expect at least 2.
        assert!(results.len() >= 2);
    }

    #[test]
    fn test_calculate_compensation() {
        // Standard first layer: 0.2mm height, 0.4mm nozzle, 100% flow
        let compensation = calculate_compensation(0.2, 0.4, 1.0);

        // Should be a reasonable value
        assert!(compensation > 0.0);
        assert!(compensation < 0.2); // Less than half nozzle diameter
    }

    #[test]
    fn test_calculate_compensation_higher_flow() {
        let normal = calculate_compensation(0.2, 0.4, 1.0);
        let high_flow = calculate_compensation(0.2, 0.4, 1.2);

        // Higher flow should result in more compensation
        assert!(high_flow > normal);
    }

    #[test]
    fn test_elephant_foot_spacing() {
        let spacing = elephant_foot_spacing(0.45, 0.4);

        // Should be approximately 0.5 * (0.45 + 0.6 * 0.4) = 0.5 * 0.69 = 0.345
        assert!((spacing - 0.345).abs() < 0.01);
    }

    #[test]
    fn test_scaled_elephant_foot_spacing() {
        let spacing = scaled_elephant_foot_spacing(0.45, 0.4);

        // Should be scaled version
        assert!(spacing > 0);
        assert_eq!(spacing, scale(elephant_foot_spacing(0.45, 0.4)));
    }

    #[test]
    fn test_zero_compensation() {
        let config = ElephantFootConfig::new().with_compensation(0.0);
        let compensator = ElephantFootCompensator::new(config);

        // Should not be enabled with zero compensation
        assert!(!compensator.is_enabled());
    }

    #[test]
    fn test_negative_compensation_clamped() {
        let config = ElephantFootConfig::new().with_compensation(-0.5);

        // Negative values should be clamped to 0
        assert_eq!(config.compensation, 0.0);
    }
}
