//! Adaptive Layer Heights Implementation
//!
//! This module implements adaptive layer height computation that varies layer
//! height based on the surface slope of the mesh. Steep surfaces get thinner
//! layers for better detail, while flat surfaces can use thicker layers for speed.
//!
//! # Algorithm Overview
//!
//! Based on the work of Florens Wasserfall (@platch):
//! "Adaptive Slicing for the FDM Process Revisited"
//! DOI: 10.1109/COASE.2017.8256074
//!
//! The algorithm works by:
//! 1. Collecting all mesh faces and their Z-spans and normal angles
//! 2. Sorting faces by their minimum Z coordinate
//! 3. For each layer, finding all faces that intersect the current Z range
//! 4. Computing the maximum layer height that maintains surface quality
//!
//! # Surface Error Metrics
//!
//! Several error metrics are supported (matching BambuStudio):
//! - Triangle area error (Vojtech's formula) - default
//! - Topographic lines distance (Cura-style)
//! - Surface roughness (constant stepping along surface)
//! - Wasserfall's original formula
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/SlicingAdaptive.hpp`
//! - `src/libslic3r/SlicingAdaptive.cpp`

use crate::slicing::SlicingParams;
use crate::triangle_mesh::TriangleMesh;
use crate::CoordF;

/// Surface roughness constant from Wasserfall's paper.
/// Describes the volumetric error at the surface induced by stacking
/// elliptic extrusion threads.
const SURFACE_CONST: CoordF = 0.18403;

/// Small epsilon for floating point comparisons.
const EPSILON: CoordF = 1e-6;

/// Information about a mesh face relevant for adaptive slicing.
#[derive(Debug, Clone, Copy)]
pub struct FaceZ {
    /// Z-span of the face (min_z, max_z).
    pub z_span: (CoordF, CoordF),

    /// Cosine of the normal vector towards the Z axis (|n.z|).
    /// 0 = vertical face, 1 = horizontal face.
    pub n_cos: CoordF,

    /// Sine of the normal vector towards the Z axis (sqrt(n.x² + n.y²)).
    /// 1 = vertical face, 0 = horizontal face.
    pub n_sin: CoordF,
}

impl FaceZ {
    // Create a new FaceZ from Z-span and normal components.
    pub fn new(z_min: CoordF, z_max: CoordF, normal_z: CoordF, normal_xy_mag: CoordF) -> Self {
        Self {
            z_span: (z_min, z_max),
            n_cos: normal_z.abs(),
            n_sin: normal_xy_mag,
        }
    }

    /// Check if this face is horizontal (normal pointing up/down).
    pub fn is_horizontal(&self) -> bool {
        self.z_span.0 == self.z_span.1
    }

    /// Check if this face intersects the given Z range.
    pub fn intersects_z_range(&self, z_min: CoordF, z_max: CoordF) -> bool {
        self.z_span.0 < z_max && self.z_span.1 > z_min
    }
}

/// Error metric for computing layer height from surface slope.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SlopeErrorMetric {
    /// Constant error measured as triangle area (Vojtech's formula).
    /// This is the default and provides good balance.
    #[default]
    TriangleArea,

    /// Constant stepping in horizontal direction (Cura-style).
    /// Good for topographic-style prints.
    TopographicDistance,

    /// Constant stepping along the surface (Perez/Pandey).
    /// Matches "surface roughness" metric.
    SurfaceRoughness,

    /// Original Wasserfall formula from his paper.
    Wasserfall,
}

/// Configuration for adaptive layer height computation.
#[derive(Debug, Clone)]
pub struct AdaptiveHeightsConfig {
    /// Minimum layer height (mm).
    pub min_layer_height: CoordF,

    /// Maximum layer height (mm).
    pub max_layer_height: CoordF,

    /// Default/target layer height (mm).
    pub layer_height: CoordF,

    /// Quality factor (0.0 = highest quality/thin layers, 1.0 = lowest quality/thick layers).
    pub quality: CoordF,

    /// Error metric to use for slope-based height calculation.
    pub error_metric: SlopeErrorMetric,
}

impl Default for AdaptiveHeightsConfig {
    fn default() -> Self {
        Self {
            min_layer_height: 0.07,
            max_layer_height: 0.3,
            layer_height: 0.2,
            quality: 0.5,
            error_metric: SlopeErrorMetric::TriangleArea,
        }
    }
}

impl AdaptiveHeightsConfig {
    // Create a new configuration with the given layer heights.
    pub fn new(min_height: CoordF, max_height: CoordF, default_height: CoordF) -> Self {
        Self {
            min_layer_height: min_height,
            max_layer_height: max_height,
            layer_height: default_height,
            ..Default::default()
        }
    }

    /// Create a configuration from slicing parameters.
    pub fn from_slicing_params(params: &SlicingParams) -> Self {
        Self {
            min_layer_height: params.min_layer_height,
            max_layer_height: params.max_layer_height,
            layer_height: params.layer_height,
            ..Default::default()
        }
    }

    /// Set the quality factor.
    pub fn with_quality(mut self, quality: CoordF) -> Self {
        self.quality = quality.clamp(0.0, 1.0);
        self
    }

    /// Set the error metric.
    pub fn with_error_metric(mut self, metric: SlopeErrorMetric) -> Self {
        self.error_metric = metric;
        self
    }

    /// Compute the maximum surface deviation based on quality factor.
    ///
    /// Quality 0.0 = use min_layer_height as deviation
    /// Quality 0.5 = use layer_height as deviation
    /// Quality 1.0 = use max_layer_height as deviation
    pub fn max_surface_deviation(&self) -> CoordF {
        let delta_min = self.min_layer_height;
        let delta_mid = self.layer_height;
        let delta_max = self.max_layer_height;

        if self.quality < 0.5 {
            lerp(delta_min, delta_mid, 2.0 * self.quality)
        } else {
            lerp(delta_max, delta_mid, 2.0 * (1.0 - self.quality))
        }
    }
}

/// Linear interpolation.
fn lerp(a: CoordF, b: CoordF, t: CoordF) -> CoordF {
    a + (b - a) * t
}

/// Compute the maximum layer height for a given face based on its slope.
///
/// # Arguments
/// * `face` - The face to compute height for
/// * `max_deviation` - Maximum allowed surface deviation
/// * `metric` - The error metric to use
fn layer_height_from_slope(
    face: &FaceZ,
    max_deviation: CoordF,
    metric: SlopeErrorMetric,
) -> CoordF {
    match metric {
        SlopeErrorMetric::TriangleArea => {
            // Vojtech's formula: constant error measured as triangle area
            // with clamping to roughness at 90 degrees
            let clamped = max_deviation / 0.184;
            if face.n_cos > EPSILON {
                clamped.min(1.44 * max_deviation * (face.n_sin / face.n_cos).sqrt())
            } else {
                clamped
            }
        }
        SlopeErrorMetric::TopographicDistance => {
            // Cura-style: constant stepping in horizontal direction
            if face.n_cos > EPSILON {
                max_deviation * face.n_sin / face.n_cos
            } else {
                CoordF::MAX
            }
        }
        SlopeErrorMetric::SurfaceRoughness => {
            // Constant stepping along the surface
            max_deviation * face.n_sin
        }
        SlopeErrorMetric::Wasserfall => {
            // Original Wasserfall formula from his paper
            max_deviation / (SURFACE_CONST + 0.5 * face.n_cos)
        }
    }
}

/// Result of adaptive layer height computation.
#[derive(Debug, Clone)]
pub struct AdaptiveLayerHeight {
    /// Bottom Z coordinate of the layer.
    pub bottom_z: CoordF,

    /// Top Z coordinate of the layer (print Z).
    pub top_z: CoordF,

    /// Slice Z coordinate (where the slicing plane is).
    pub slice_z: CoordF,

    /// Computed layer height.
    pub height: CoordF,
}

impl AdaptiveLayerHeight {
    // Create a new layer height entry.
    pub fn new(bottom_z: CoordF, height: CoordF) -> Self {
        let top_z = bottom_z + height;
        Self {
            bottom_z,
            top_z,
            slice_z: (bottom_z + top_z) / 2.0,
            height,
        }
    }
}

/// Adaptive slicing engine that computes variable layer heights.
pub struct AdaptiveSlicing {
    /// Configuration for adaptive heights.
    config: AdaptiveHeightsConfig,

    /// Collected faces sorted by Z-span.
    faces: Vec<FaceZ>,
}

impl AdaptiveSlicing {
    // Create a new adaptive slicing engine.
    pub fn new(config: AdaptiveHeightsConfig) -> Self {
        Self {
            config,
            faces: Vec::new(),
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(AdaptiveHeightsConfig::default())
    }

    /// Clear all collected face data.
    pub fn clear(&mut self) {
        self.faces.clear();
    }

    /// Get the configuration.
    pub fn config(&self) -> &AdaptiveHeightsConfig {
        &self.config
    }

    /// Prepare the adaptive slicer by collecting face data from a mesh.
    ///
    /// This should be called before computing layer heights.
    pub fn prepare(&mut self, mesh: &TriangleMesh) {
        self.clear();

        if mesh.is_empty() {
            return;
        }

        let vertices = mesh.vertices();
        let indices = mesh.indices();

        self.faces.reserve(indices.len());

        for tri in indices {
            let v0 = &vertices[tri.indices[0] as usize];
            let v1 = &vertices[tri.indices[1] as usize];
            let v2 = &vertices[tri.indices[2] as usize];

            // Compute face Z-span
            let z_min = v0.z.min(v1.z).min(v2.z);
            let z_max = v0.z.max(v1.z).max(v2.z);

            // Compute face normal
            let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
            let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];

            // Cross product
            let nx = e1[1] * e2[2] - e1[2] * e2[1];
            let ny = e1[2] * e2[0] - e1[0] * e2[2];
            let nz = e1[0] * e2[1] - e1[1] * e2[0];

            // Normalize
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len < EPSILON {
                continue; // Degenerate triangle
            }

            let nz_normalized = nz / len;
            let nxy_magnitude = (nx * nx + ny * ny).sqrt() / len;

            self.faces.push(FaceZ::new(
                z_min as CoordF,
                z_max as CoordF,
                nz_normalized as CoordF,
                nxy_magnitude as CoordF,
            ));
        }

        // Sort faces by their Z-span (lexicographically by (z_min, z_max))
        self.faces
            .sort_by(|a, b| a.z_span.partial_cmp(&b.z_span).unwrap());
    }

    /// Compute the next layer height starting from the given print_z.
    ///
    /// # Arguments
    /// * `print_z` - The top surface of the previous layer
    /// * `current_facet` - Index hint for face iteration (updated on return)
    ///
    /// # Returns
    /// The computed layer height.
    pub fn next_layer_height(&self, print_z: CoordF, current_facet: &mut usize) -> CoordF {
        let mut height = self.config.max_layer_height;
        let max_deviation = self.config.max_surface_deviation();

        // Find all faces intersecting the current slice region
        let mut ordered_id = *current_facet;
        let mut first_hit = false;

        while ordered_id < self.faces.len() {
            let face = &self.faces[ordered_id];

            // Face's minimum is higher than print_z -> end loop
            if face.z_span.0 >= print_z {
                break;
            }

            // Face's maximum is higher than print_z -> this face intersects
            if face.z_span.1 > print_z {
                // First event?
                if !first_hit {
                    first_hit = true;
                    *current_facet = ordered_id;
                }

                // Skip touching faces which could otherwise cause small cusp values
                if face.z_span.1 < print_z + EPSILON {
                    ordered_id += 1;
                    continue;
                }

                // Compute cusp-height for this face and store minimum
                let face_height =
                    layer_height_from_slope(face, max_deviation, self.config.error_metric);
                height = height.min(face_height);
            }

            ordered_id += 1;
        }

        // Lower height limit due to printer capabilities
        height = height.max(self.config.min_layer_height);

        // Check for sloped faces inside the determined layer and correct height if necessary
        if height > self.config.min_layer_height {
            while ordered_id < self.faces.len() {
                let face = &self.faces[ordered_id];

                // Face's minimum is higher than print_z + height -> end loop
                if face.z_span.0 >= print_z + height {
                    break;
                }

                // Skip touching faces
                if face.z_span.1 < print_z + EPSILON {
                    ordered_id += 1;
                    continue;
                }

                // Compute cusp-height for this face
                let reduced_height =
                    layer_height_from_slope(face, max_deviation, self.config.error_metric);

                let z_diff = face.z_span.0 - print_z;
                if reduced_height < z_diff {
                    // The face's slope limits the layer height so much that
                    // the lowest point of the face is already above the new layer height.
                    // Limit layer height so this face is just above the new layer.
                    height = z_diff.max(self.config.min_layer_height);
                } else if reduced_height < height {
                    height = reduced_height;
                }

                ordered_id += 1;
            }

            // Apply minimum height limit again
            height = height.max(self.config.min_layer_height);
        }

        height
    }

    /// Find the distance to the next horizontal facet in Z direction.
    ///
    /// This helps consider horizontal object features in slice thickness.
    pub fn horizontal_facet_distance(&self, z: CoordF) -> CoordF {
        for face in &self.faces {
            // Face's minimum is higher than max forward distance -> end loop
            if face.z_span.0 > z + self.config.max_layer_height {
                break;
            }

            // Horizontal facet (min_z == max_z) above current Z
            if face.z_span.0 > z && face.is_horizontal() {
                return face.z_span.0 - z;
            }
        }

        // Return max layer height or distance to object top
        self.config.max_layer_height
    }

    /// Compute all layer heights for slicing the mesh.
    ///
    /// # Arguments
    /// * `mesh` - The mesh to slice
    /// * `first_layer_height` - Height of the first layer
    ///
    /// # Returns
    /// A vector of AdaptiveLayerHeight entries.
    pub fn compute_layer_heights(
        &mut self,
        mesh: &TriangleMesh,
        first_layer_height: CoordF,
    ) -> Vec<AdaptiveLayerHeight> {
        self.prepare(mesh);

        if self.faces.is_empty() {
            return Vec::new();
        }

        let bb = mesh.compute_bounding_box();
        if !bb.is_defined() {
            return Vec::new();
        }

        let min_z = bb.min.z as CoordF;
        let max_z = bb.max.z as CoordF;

        let mut heights = Vec::new();
        let mut z = min_z;
        let mut current_facet = 0usize;

        // First layer with fixed height
        if z < max_z {
            let height = first_layer_height.min(max_z - z);
            heights.push(AdaptiveLayerHeight::new(z, height));
            z += height;
        }

        // Subsequent layers with adaptive height
        while z < max_z {
            let height = self.next_layer_height(z, &mut current_facet);
            let clamped_height = height.min(max_z - z);
            heights.push(AdaptiveLayerHeight::new(z, clamped_height));
            z += clamped_height;
        }

        heights
    }

    /// Compute layer heights with a quality-based approach.
    ///
    /// Quality ranges from 0.0 (highest quality, thinnest layers) to
    /// 1.0 (lowest quality, thickest layers).
    pub fn compute_layer_heights_with_quality(
        &mut self,
        mesh: &TriangleMesh,
        first_layer_height: CoordF,
        quality: CoordF,
    ) -> Vec<AdaptiveLayerHeight> {
        self.config.quality = quality.clamp(0.0, 1.0);
        self.compute_layer_heights(mesh, first_layer_height)
    }
}

impl Default for AdaptiveSlicing {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Convenience function to compute adaptive layer heights for a mesh.
pub fn compute_adaptive_heights(
    mesh: &TriangleMesh,
    params: &SlicingParams,
) -> Vec<AdaptiveLayerHeight> {
    let config = AdaptiveHeightsConfig::from_slicing_params(params);
    let mut slicer = AdaptiveSlicing::new(config);
    slicer.compute_layer_heights(mesh, params.first_layer_height)
}

/// Convenience function to compute adaptive layer heights with quality setting.
pub fn compute_adaptive_heights_with_quality(
    mesh: &TriangleMesh,
    params: &SlicingParams,
    quality: CoordF,
) -> Vec<AdaptiveLayerHeight> {
    let config = AdaptiveHeightsConfig::from_slicing_params(params).with_quality(quality);
    let mut slicer = AdaptiveSlicing::new(config);
    slicer.compute_layer_heights(mesh, params.first_layer_height)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_default() {
        let config = AdaptiveHeightsConfig::default();
        assert!((config.min_layer_height - 0.07).abs() < 1e-6);
        assert!((config.max_layer_height - 0.3).abs() < 1e-6);
        assert!((config.layer_height - 0.2).abs() < 1e-6);
        assert!((config.quality - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_config_max_surface_deviation() {
        let config = AdaptiveHeightsConfig::default();

        // At quality 0.5, deviation should be close to layer_height
        let dev = config.max_surface_deviation();
        assert!((dev - config.layer_height).abs() < 1e-6);

        // At quality 0.0, deviation should be min_layer_height
        let config_high_quality = config.clone().with_quality(0.0);
        let dev_high = config_high_quality.max_surface_deviation();
        assert!((dev_high - config.min_layer_height).abs() < 1e-6);

        // At quality 1.0, deviation should be max_layer_height
        let config_low_quality = config.clone().with_quality(1.0);
        let dev_low = config_low_quality.max_surface_deviation();
        assert!((dev_low - config.max_layer_height).abs() < 1e-6);
    }

    #[test]
    fn test_face_z_horizontal() {
        let face = FaceZ::new(1.0, 1.0, 1.0, 0.0);
        assert!(face.is_horizontal());

        let face2 = FaceZ::new(1.0, 2.0, 0.5, 0.866);
        assert!(!face2.is_horizontal());
    }

    #[test]
    fn test_face_z_intersects() {
        let face = FaceZ::new(1.0, 3.0, 0.5, 0.866);

        assert!(face.intersects_z_range(0.0, 2.0)); // Overlaps bottom
        assert!(face.intersects_z_range(2.0, 4.0)); // Overlaps top
        assert!(face.intersects_z_range(1.5, 2.5)); // Inside
        assert!(!face.intersects_z_range(3.5, 4.0)); // Above
        assert!(!face.intersects_z_range(0.0, 0.5)); // Below
    }

    #[test]
    fn test_layer_height_from_slope_horizontal() {
        // Horizontal face (normal pointing up)
        let face = FaceZ::new(1.0, 1.0, 1.0, 0.0);
        let max_dev = 0.2;

        // For horizontal faces with n_sin=0:
        // - TriangleArea: min(clamped, 1.44 * max_dev * sqrt(0/1)) = min(clamped, 0) = 0
        // - SurfaceRoughness: max_dev * n_sin = max_dev * 0 = 0
        // - Wasserfall gives a positive result
        let h1 = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::TriangleArea);
        let h2 = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::SurfaceRoughness);
        let h3 = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::Wasserfall);

        // TriangleArea and SurfaceRoughness give 0 for perfectly horizontal
        // (because there's no stairstepping on horizontal surfaces)
        assert!((h1 - 0.0).abs() < 1e-6);
        assert!((h2 - 0.0).abs() < 1e-6);
        // Wasserfall metric gives a reasonable positive value
        assert!(h3 > 0.0);
    }

    #[test]
    fn test_layer_height_from_slope_vertical() {
        // Vertical face (normal pointing sideways)
        let face = FaceZ::new(0.0, 10.0, 0.0, 1.0);
        let max_dev = 0.2;

        let h = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::TriangleArea);

        // Vertical surface should be clamped
        assert!(h > 0.0);
        assert!(h <= max_dev / 0.184 + 1e-6); // Clamped value
    }

    #[test]
    fn test_layer_height_from_slope_45_degrees() {
        // 45-degree face
        let n_cos = (0.5_f64).sqrt() as CoordF;
        let n_sin = (0.5_f64).sqrt() as CoordF;
        let face = FaceZ::new(0.0, 10.0, n_cos, n_sin);
        let max_dev = 0.2;

        let h = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::TriangleArea);

        // Should give a reasonable height between min and max
        assert!(h > 0.0);
        assert!(h < 10.0); // Reasonable upper bound
    }

    #[test]
    fn test_adaptive_slicing_empty_mesh() {
        let mesh = TriangleMesh::new();
        let mut slicer = AdaptiveSlicing::with_defaults();

        let heights = slicer.compute_layer_heights(&mesh, 0.3);
        assert!(heights.is_empty());
    }

    #[test]
    fn test_adaptive_slicing_cube() {
        let mesh = TriangleMesh::cube(10.0);
        let mut slicer = AdaptiveSlicing::with_defaults();

        let heights = slicer.compute_layer_heights(&mesh, 0.3);

        // Should have multiple layers
        assert!(!heights.is_empty());

        // First layer should have the specified height
        assert!((heights[0].height - 0.3).abs() < 1e-6);

        // All heights should be within bounds
        for h in &heights {
            assert!(h.height >= slicer.config().min_layer_height - EPSILON);
            assert!(h.height <= slicer.config().max_layer_height + EPSILON);
        }

        // Layers should be contiguous
        for i in 1..heights.len() {
            assert!((heights[i].bottom_z - heights[i - 1].top_z).abs() < 1e-6);
        }
    }

    #[test]
    fn test_adaptive_slicing_quality_variation() {
        let mesh = TriangleMesh::cube(10.0);

        // High quality (thin layers)
        let mut slicer_high =
            AdaptiveSlicing::new(AdaptiveHeightsConfig::default().with_quality(0.0));
        let heights_high = slicer_high.compute_layer_heights(&mesh, 0.3);

        // Low quality (thick layers)
        let mut slicer_low =
            AdaptiveSlicing::new(AdaptiveHeightsConfig::default().with_quality(1.0));
        let heights_low = slicer_low.compute_layer_heights(&mesh, 0.3);

        // High quality should have more layers (or equal)
        assert!(heights_high.len() >= heights_low.len());
    }

    #[test]
    fn test_lerp() {
        assert!((lerp(0.0, 10.0, 0.0) - 0.0).abs() < 1e-6);
        assert!((lerp(0.0, 10.0, 1.0) - 10.0).abs() < 1e-6);
        assert!((lerp(0.0, 10.0, 0.5) - 5.0).abs() < 1e-6);
        assert!((lerp(2.0, 8.0, 0.25) - 3.5).abs() < 1e-6);
    }

    #[test]
    fn test_convenience_functions() {
        let mesh = TriangleMesh::cube(10.0);
        let params = SlicingParams::default();

        let heights = compute_adaptive_heights(&mesh, &params);
        assert!(!heights.is_empty());

        let heights_quality = compute_adaptive_heights_with_quality(&mesh, &params, 0.3);
        assert!(!heights_quality.is_empty());
    }

    #[test]
    fn test_adaptive_layer_height_struct() {
        let layer = AdaptiveLayerHeight::new(1.0, 0.2);

        assert!((layer.bottom_z - 1.0).abs() < 1e-6);
        assert!((layer.top_z - 1.2).abs() < 1e-6);
        assert!((layer.slice_z - 1.1).abs() < 1e-6);
        assert!((layer.height - 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_horizontal_facet_distance() {
        let mesh = TriangleMesh::cube(10.0);
        let mut slicer = AdaptiveSlicing::with_defaults();
        slicer.prepare(&mesh);

        let distance = slicer.horizontal_facet_distance(0.0);

        // Should return at most max_layer_height
        assert!(distance <= slicer.config().max_layer_height + EPSILON);
        assert!(distance > 0.0);
    }

    #[test]
    fn test_error_metrics() {
        let face = FaceZ::new(0.0, 10.0, 0.5, 0.866); // ~60 degree slope
        let max_dev = 0.2;

        let h_triangle = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::TriangleArea);
        let h_topo = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::TopographicDistance);
        let h_rough = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::SurfaceRoughness);
        let h_wasser = layer_height_from_slope(&face, max_dev, SlopeErrorMetric::Wasserfall);

        // All should give positive values
        assert!(h_triangle > 0.0);
        assert!(h_topo > 0.0);
        assert!(h_rough > 0.0);
        assert!(h_wasser > 0.0);

        // They may differ in value
        // Just ensure they're in a reasonable range
        assert!(h_triangle < 10.0);
        assert!(h_topo < 10.0);
        assert!(h_rough < 10.0);
        assert!(h_wasser < 10.0);
    }
}
