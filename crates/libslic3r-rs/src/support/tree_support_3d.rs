//! Tree Support 3D - Branch generation and support element propagation.
//!
//! This module implements the core tree support algorithm that grows branches
//! from overhang points down to the build plate while avoiding collisions.
//! It is a Rust port of BambuStudio's `TreeSupport3D.cpp`.
//!
//! # Algorithm Overview
//!
//! 1. **Generate Initial Areas**: Sample overhang regions to create support tips
//! 2. **Create Layer Pathing**: Propagate support elements downward layer by layer
//! 3. **Merge Branches**: Combine nearby branches to reduce material usage
//! 4. **Set Points on Areas**: Determine final branch center positions
//! 5. **Organic Smoothing** (optional): Smooth branches and avoid collisions
//! 6. **Draw Branches**: Convert support elements to printable geometry
//!
//! # BambuStudio Reference
//!
//! - `Support/TreeSupport3D.cpp`
//! - `Support/TreeSupport3D.hpp`

use crate::clipper_utils::{self, OffsetJoinType};
use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use crate::support::organic_smooth::{
    smooth_move_bounds, OrganicSmoothConfig, OrganicSmoothResult,
};
use crate::support::tree_model_volumes::{point_inside_polygons, AvoidanceType, TreeModelVolumes};
use crate::support::tree_support_settings::{
    LineStatus, SupportElement, SupportElementState, TreeSupportSettings,
};
use crate::support::{SupportConfig, SupportLayer};
use crate::{scale, unscale, Coord};

/// Minimum area threshold for valid support areas (in scaled units squared).
const TINY_AREA_THRESHOLD: f64 = 1000.0; // ~1mm² in scaled units

/// Default connection length for support sampling (5mm in scaled units).
const DEFAULT_CONNECT_LENGTH: Coord = 5_000_000;

/// Line information with position and status.
pub type LineInformation = Vec<(Point, LineStatus)>;

/// Collection of line information.
pub type LineInformations = Vec<LineInformation>;

/// Support elements per layer.
pub type SupportElements = Vec<SupportElement>;

/// All support elements indexed by layer.
pub type LayerSupportElements = Vec<SupportElements>;

/// Result of branch generation.
#[derive(Debug, Clone)]
pub struct TreeSupport3DResult {
    /// Support layers with polygons for each layer.
    pub layers: Vec<SupportLayer>,
    /// Total number of branches generated.
    pub branch_count: usize,
    /// Total number of tips (contact points).
    pub tip_count: usize,
}

/// Configuration for Tree Support 3D generation.
#[derive(Debug, Clone)]
pub struct TreeSupport3DConfig {
    /// Base tree support settings.
    pub settings: TreeSupportSettings,
    /// Whether to enable roof generation.
    pub roof_enabled: bool,
    /// Number of roof layers.
    pub num_roof_layers: usize,
    /// Minimum area for support regions (mm²).
    pub minimum_support_area: f64,
    /// Minimum area for roof regions (mm²).
    pub minimum_roof_area: f64,
    /// Support offset from model.
    pub support_offset: Coord,
    /// Branch distance (spacing between tips).
    pub branch_distance: Coord,
    /// Top rate for branch thickening.
    pub top_rate: f64,
}

impl Default for TreeSupport3DConfig {
    fn default() -> Self {
        Self {
            settings: TreeSupportSettings::default(),
            roof_enabled: true,
            num_roof_layers: 3,
            minimum_support_area: 1.0,
            minimum_roof_area: 1.0,
            support_offset: 0,
            branch_distance: scale(1.0),
            top_rate: 15.0,
        }
    }
}

impl TreeSupport3DConfig {
    // Create config from SupportConfig.
    pub fn from_support_config(config: &SupportConfig) -> Self {
        let mut result = Self::default();
        result.roof_enabled = config.support_roof;
        result.num_roof_layers = config.top_interface_layers;
        result.minimum_support_area = config.min_area;
        result.settings.xy_distance = scale(config.xy_distance);
        result.settings.support_rests_on_model = !config.buildplate_only;
        result.settings.branch_radius = scale(config.tree_branch_diameter / 2.0);
        result.settings.min_radius = scale(config.tree_tip_diameter / 2.0);
        result.branch_distance = scale(config.tree_branch_diameter);
        result
    }
}

/// Tree Support 3D generator.
#[derive(Debug)]
pub struct TreeSupport3D {
    config: TreeSupport3DConfig,
    volumes: TreeModelVolumes,
    /// Support elements per layer (built from top down).
    move_bounds: LayerSupportElements,
    /// Number of layers.
    num_layers: usize,
}

impl TreeSupport3D {
    // Create a new Tree Support 3D generator.
    pub fn new(config: TreeSupport3DConfig, volumes: TreeModelVolumes) -> Self {
        let num_layers = volumes.layer_count();
        Self {
            config,
            volumes,
            move_bounds: vec![Vec::new(); num_layers],
            num_layers,
        }
    }

    /// Generate tree supports from overhang regions.
    ///
    /// # Arguments
    /// * `overhangs` - Overhang polygons for each layer
    ///
    /// # Returns
    /// Tree support result with support layers.
    pub fn generate(&mut self, overhangs: &[Vec<Polygon>]) -> TreeSupport3DResult {
        // Generate initial support areas (tips)
        self.generate_initial_areas(overhangs);

        // Create layer pathing (propagate downward)
        self.create_layer_pathing();

        // Set points on areas (determine branch centers)
        self.set_points_on_areas();

        // Convert to support layers
        self.create_support_layers()
    }

    /// Generate tree supports using 3D mesh extrusion for accurate branch geometry.
    ///
    /// This method creates a 3D tube mesh for each branch and slices it to get
    /// accurate per-layer polygons, providing better geometric fidelity than
    /// the simple circle-based rasterization used by `generate()`.
    ///
    /// # Arguments
    /// * `overhangs` - Overhang polygons for each layer
    ///
    /// # Returns
    /// Tree support result with support layers derived from mesh slicing.
    pub fn generate_with_mesh(&mut self, overhangs: &[Vec<Polygon>]) -> TreeSupport3DResult {
        self.generate_with_mesh_internal(overhangs, None)
    }

    /// Generate tree supports using 3D mesh extrusion with organic smoothing.
    ///
    /// This method creates a 3D tube mesh for each branch after applying organic
    /// smoothing to create smooth, collision-free branch paths.
    ///
    /// # Arguments
    /// * `overhangs` - Overhang polygons for each layer
    /// * `model_outlines` - Model outlines per layer for collision detection during smoothing
    ///
    /// # Returns
    /// Tree support result with smoothed branches.
    pub fn generate_with_organic_smoothing(
        &mut self,
        overhangs: &[Vec<Polygon>],
        model_outlines: &[ExPolygons],
    ) -> TreeSupport3DResult {
        self.generate_with_mesh_internal(overhangs, Some(model_outlines))
    }

    /// Internal implementation for mesh-based generation with optional organic smoothing.
    fn generate_with_mesh_internal(
        &mut self,
        overhangs: &[Vec<Polygon>],
        model_outlines: Option<&[ExPolygons]>,
    ) -> TreeSupport3DResult {
        use crate::support::branch_mesh::{
            generate_branch_mesh, slice_branch_mesh, BranchMeshConfig,
        };

        // : Same as regular generate
        self.generate_initial_areas(overhangs);
        self.create_layer_pathing();
        self.set_points_on_areas();

        // Apply organic smoothing if model outlines provided
        if let Some(outlines) = model_outlines {
            self.apply_organic_smoothing(outlines);
        }

        // Generate branch mesh and slice it
        let mesh_config = BranchMeshConfig::default();
        let mesh_result =
            generate_branch_mesh(&self.move_bounds, &self.config.settings, mesh_config);

        if mesh_result.mesh.is_empty() {
            // Fall back to simple circle-based generation
            return self.create_support_layers();
        }

        // Get layer Z heights and heights
        let layer_zs: Vec<f64> = (0..self.num_layers)
            .map(|i| unscale(self.config.settings.get_actual_z(i)))
            .collect();
        let layer_heights: Vec<f64> = (0..self.num_layers)
            .map(|i| {
                if i == 0 {
                    unscale(self.config.settings.layer_height)
                } else {
                    unscale(self.config.settings.layer_height)
                }
            })
            .collect();

        // Slice the branch mesh
        let sliced_polys = slice_branch_mesh(&mesh_result.mesh, &layer_zs, &layer_heights);

        // Convert sliced polygons to support layers
        let mut layers = Vec::with_capacity(self.num_layers);
        let mut tip_count = 0;

        // Count tips from elements
        for layer_idx in 0..self.num_layers {
            for element in &self.move_bounds[layer_idx] {
                if !element.state.bits.deleted && element.state.distance_to_top == 0 {
                    tip_count += 1;
                }
            }
        }

        for layer_idx in 0..self.num_layers {
            let z = unscale(self.config.settings.get_actual_z(layer_idx));
            let height = unscale(self.config.settings.layer_height);

            // Get sliced polygons for this layer
            let support_regions = if layer_idx < sliced_polys.len() {
                sliced_polys[layer_idx].clone()
            } else {
                Vec::new()
            };

            // Determine interface regions based on proximity to tips
            let mut interface_regions = Vec::new();
            let elements = &self.move_bounds[layer_idx];

            // Check if any element at this layer is an interface
            let has_interface = elements.iter().any(|e| {
                !e.state.bits.deleted
                    && e.state.bits.supports_roof
                    && e.state.distance_to_top < self.config.num_roof_layers as u32
            });

            layers.push(SupportLayer {
                layer_id: layer_idx,
                z,
                height,
                support_regions,
                interface_regions,
                is_interface: has_interface,
                overhang_regions: Vec::new(),
            });
        }

        TreeSupport3DResult {
            layers,
            branch_count: mesh_result.branch_count,
            tip_count,
        }
    }

    /// Get a reference to the internal move_bounds (support elements per layer).
    /// Useful for external processing like organic smoothing.
    pub fn move_bounds(&self) -> &LayerSupportElements {
        &self.move_bounds
    }

    /// Get a mutable reference to the internal move_bounds.
    pub fn move_bounds_mut(&mut self) -> &mut LayerSupportElements {
        &mut self.move_bounds
    }

    /// Get the configuration.
    pub fn config(&self) -> &TreeSupport3DConfig {
        &self.config
    }

    /// Get the number of layers.
    pub fn num_layers(&self) -> usize {
        self.num_layers
    }

    /// Get a reference to the TreeModelVolumes.
    pub fn volumes(&self) -> &TreeModelVolumes {
        &self.volumes
    }

    /// Apply organic smoothing to the current move_bounds.
    ///
    /// This smooths branch positions and resolves collisions with the model.
    /// Should be called after `set_points_on_areas()` and before mesh generation.
    ///
    /// # Arguments
    /// * `model_outlines` - Model outlines per layer for collision detection
    ///
    /// # Returns
    /// Organic smoothing result with iteration count and convergence status.
    pub fn apply_organic_smoothing(
        &mut self,
        model_outlines: &[ExPolygons],
    ) -> OrganicSmoothResult {
        let config = OrganicSmoothConfig::default();
        smooth_move_bounds(
            &mut self.move_bounds,
            &self.config.settings,
            model_outlines,
            config,
        )
    }

    /// Apply organic smoothing with custom configuration.
    ///
    /// # Arguments
    /// * `model_outlines` - Model outlines per layer for collision detection
    /// * `config` - Custom organic smoothing configuration
    ///
    /// # Returns
    /// Organic smoothing result with iteration count and convergence status.
    pub fn apply_organic_smoothing_with_config(
        &mut self,
        model_outlines: &[ExPolygons],
        config: OrganicSmoothConfig,
    ) -> OrganicSmoothResult {
        smooth_move_bounds(
            &mut self.move_bounds,
            &self.config.settings,
            model_outlines,
            config,
        )
    }

    /// Generate initial support areas from overhangs.
    fn generate_initial_areas(&mut self, overhangs: &[Vec<Polygon>]) {
        let z_distance_delta = self.config.settings.z_distance_top_layers + 1;
        let _min_xy_dist = self.config.settings.xy_distance > self.config.settings.xy_min_distance;

        // Calculate connection length based on config
        let connect_length = (self.config.settings.support_line_width as f64 * 100.0
            / self.config.top_rate) as Coord
            + (2 * self.config.settings.min_radius - self.config.settings.support_line_width)
                .max(0);

        // Process each layer with overhangs
        for layer_idx in z_distance_delta..self.num_layers.min(overhangs.len()) {
            let overhang_idx = layer_idx;
            if overhangs[overhang_idx].is_empty() {
                continue;
            }

            let support_layer_idx = layer_idx.saturating_sub(z_distance_delta);
            let overhang = &overhangs[overhang_idx];

            // Get relevant forbidden areas
            let relevant_forbidden = if self.config.settings.support_rests_on_model {
                self.volumes
                    .get_collision(self.config.settings.min_radius, support_layer_idx)
            } else {
                self.volumes.get_avoidance(
                    self.config.settings.min_radius,
                    support_layer_idx,
                    AvoidanceType::Fast,
                    false,
                )
            };

            // Offset forbidden area slightly for numerical stability
            let relevant_forbidden = offset_polygons_simple(&relevant_forbidden, scale(0.005));

            // Calculate safe overhang area
            let overhang_safe = safe_offset_inc(
                overhang,
                self.config.support_offset,
                &relevant_forbidden,
                self.config.settings.min_radius * 2 + self.config.settings.xy_min_distance,
            );

            // Filter small areas
            let overhang_filtered: Vec<Polygon> = overhang_safe
                .into_iter()
                .filter(|p| polygon_area(p) >= scale(self.config.minimum_support_area) as f64)
                .collect();

            // Sample overhang areas to create support tips
            for polygon in &overhang_filtered {
                self.sample_overhang_area(
                    polygon,
                    false, // not roof (for now)
                    support_layer_idx,
                    connect_length,
                );
            }
        }
    }

    /// Sample an overhang area to create support tips.
    fn sample_overhang_area(
        &mut self,
        polygon: &Polygon,
        is_roof: bool,
        layer_idx: usize,
        connect_length: Coord,
    ) {
        // Sample points along the polygon boundary
        let points = sample_polygon_points(polygon, connect_length);

        for point in points {
            // Determine the line status for this point
            let status = self.get_avoidance_status(point, layer_idx);

            if status == LineStatus::Invalid {
                continue;
            }

            // Create support element state
            let mut state = SupportElementState::new(
                layer_idx,
                point,
                unscale(self.config.settings.min_radius),
            );
            state.bits.to_buildplate = matches!(
                status,
                LineStatus::ToBuildPlate | LineStatus::ToBuildPlateSafe
            );
            state.bits.to_model_gracious = matches!(
                status,
                LineStatus::ToModelGracious | LineStatus::ToModelGraciousSafe
            );
            state.bits.can_use_safe_radius = matches!(
                status,
                LineStatus::ToBuildPlateSafe | LineStatus::ToModelGraciousSafe
            );
            state.bits.supports_roof = is_roof;
            state.target_height = layer_idx;
            state.target_position = point;
            state.next_position = point;

            // Create influence area (circle around the point)
            let influence_area = create_circle_polygon(point, self.config.settings.min_radius);

            // Add to move_bounds for this layer
            let element = SupportElement::new(state, vec![influence_area]);
            self.move_bounds[layer_idx].push(element);
        }
    }

    /// Get the avoidance status for a point at a given layer.
    fn get_avoidance_status(&self, point: Point, layer_idx: usize) -> LineStatus {
        let radius = self.config.settings.min_radius;
        let _min_xy_dist = self.config.settings.xy_distance > self.config.settings.xy_min_distance;

        // Check FastSafe avoidance first (best case - can reach build plate safely)
        let avoidance_fast_safe =
            self.volumes
                .get_avoidance(radius, layer_idx, AvoidanceType::FastSafe, false);
        if !point_inside_polygons(point, &avoidance_fast_safe) {
            return LineStatus::ToBuildPlateSafe;
        }

        // Check Fast avoidance (can reach build plate)
        let avoidance_fast =
            self.volumes
                .get_avoidance(radius, layer_idx, AvoidanceType::Fast, false);
        if !point_inside_polygons(point, &avoidance_fast) {
            return LineStatus::ToBuildPlate;
        }

        // If support can rest on model, check model avoidances
        if self.config.settings.support_rests_on_model {
            let avoidance_model_safe =
                self.volumes
                    .get_avoidance(radius, layer_idx, AvoidanceType::FastSafe, true);
            if !point_inside_polygons(point, &avoidance_model_safe) {
                return LineStatus::ToModelGraciousSafe;
            }

            let avoidance_model =
                self.volumes
                    .get_avoidance(radius, layer_idx, AvoidanceType::Fast, true);
            if !point_inside_polygons(point, &avoidance_model) {
                return LineStatus::ToModelGracious;
            }

            let collision = self.volumes.get_collision(radius, layer_idx);
            if !point_inside_polygons(point, &collision) {
                return LineStatus::ToModel;
            }
        }

        LineStatus::Invalid
    }

    /// Create layer pathing - propagate support elements downward.
    fn create_layer_pathing(&mut self) {
        // Process from top to bottom
        for layer_idx in (1..self.num_layers).rev() {
            let prev_layer = &self.move_bounds[layer_idx];
            if prev_layer.is_empty() {
                continue;
            }

            // Increase areas for each element and propagate to layer below
            let mut new_elements = Vec::new();

            for (parent_idx, element) in prev_layer.iter().enumerate() {
                if element.state.bits.deleted {
                    continue;
                }

                // Propagate element state down one layer
                let propagated_state = element.state.propagate_down();

                // Calculate new influence area
                let new_influence = self.increase_area_one_layer(
                    &element.influence_area,
                    &propagated_state,
                    layer_idx - 1,
                );

                if new_influence.is_empty() {
                    // Element cannot continue - collision
                    continue;
                }

                // Create new element with parent reference
                let mut new_element = SupportElement::with_parents(
                    propagated_state,
                    vec![parent_idx as i32],
                    new_influence,
                );

                new_elements.push(new_element);
            }

            // Merge nearby elements on this layer
            let merged_elements = self.merge_influence_areas(new_elements, layer_idx - 1);

            // Add to move_bounds
            self.move_bounds[layer_idx - 1] = merged_elements;
        }
    }

    /// Increase the influence area by one layer (move downward).
    fn increase_area_one_layer(
        &self,
        influence_area: &[Polygon],
        state: &SupportElementState,
        layer_idx: usize,
    ) -> Vec<Polygon> {
        // Get the maximum movement distance for this layer
        let max_move = if state.bits.use_min_xy_dist {
            self.config.settings.maximum_move_distance_slow
        } else {
            self.config.settings.maximum_move_distance
        };

        // Offset the influence area by the movement distance
        let expanded = offset_polygons_simple(influence_area, max_move);

        if expanded.is_empty() {
            return Vec::new();
        }

        // Get collision/avoidance for this layer
        let radius = state.get_radius(&self.config.settings);
        let avoidance_type = if state.bits.can_use_safe_radius {
            AvoidanceType::FastSafe
        } else {
            AvoidanceType::Fast
        };
        let to_model = !state.bits.to_buildplate && self.config.settings.support_rests_on_model;

        let forbidden = self
            .volumes
            .get_avoidance(radius, layer_idx, avoidance_type, to_model);

        // Subtract forbidden areas from expanded influence
        let result = difference_polygons(&expanded, &forbidden);

        result
    }

    /// Merge nearby influence areas to reduce branch count.
    fn merge_influence_areas(
        &self,
        mut elements: Vec<SupportElement>,
        layer_idx: usize,
    ) -> Vec<SupportElement> {
        if elements.len() <= 1 {
            return elements;
        }

        // Simple merging strategy: merge elements whose influence areas overlap
        let mut merged = Vec::new();
        let mut used = vec![false; elements.len()];

        for i in 0..elements.len() {
            if used[i] {
                continue;
            }

            let mut current = elements[i].clone();
            used[i] = true;

            // Find all elements that overlap with current
            for j in (i + 1)..elements.len() {
                if used[j] {
                    continue;
                }

                // Check if influence areas overlap
                if influence_areas_overlap(&current.influence_area, &elements[j].influence_area) {
                    // Merge the elements
                    current = merge_support_elements(current, elements[j].clone());
                    used[j] = true;
                }
            }

            merged.push(current);
        }

        merged
    }

    /// Set the final result points on all support elements.
    fn set_points_on_areas(&mut self) {
        // Process from bottom to top
        for layer_idx in 0..self.num_layers {
            for elem_idx in 0..self.move_bounds[layer_idx].len() {
                let element = &self.move_bounds[layer_idx][elem_idx];

                if element.state.bits.deleted {
                    continue;
                }

                // If result is not set, determine it
                if !element.state.result_on_layer_is_set() {
                    // Find the best point inside the influence area
                    let best_point = if !element.influence_area.is_empty() {
                        move_inside_if_outside(&element.influence_area, element.state.next_position)
                    } else {
                        element.state.next_position
                    };

                    // We need to modify the element, so do it through the vector
                    self.move_bounds[layer_idx][elem_idx].state.result_on_layer = Some(best_point);
                }

                // Propagate to parents
                let result_point = self.move_bounds[layer_idx][elem_idx]
                    .state
                    .result_on_layer
                    .unwrap_or(self.move_bounds[layer_idx][elem_idx].state.next_position);
                let parents = self.move_bounds[layer_idx][elem_idx].parents.clone();

                if layer_idx + 1 < self.num_layers {
                    for &parent_idx in &parents {
                        if parent_idx >= 0 {
                            let parent_idx = parent_idx as usize;
                            if parent_idx < self.move_bounds[layer_idx + 1].len() {
                                let parent = &mut self.move_bounds[layer_idx + 1][parent_idx];
                                if !parent.state.result_on_layer_is_set() {
                                    parent.state.result_on_layer = Some(move_inside_if_outside(
                                        &parent.influence_area,
                                        result_point,
                                    ));
                                }
                                parent.state.bits.marked = true;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Convert support elements to support layers.
    fn create_support_layers(&self) -> TreeSupport3DResult {
        let mut layers = Vec::with_capacity(self.num_layers);
        let mut branch_count = 0;
        let mut tip_count = 0;

        for layer_idx in 0..self.num_layers {
            let elements = &self.move_bounds[layer_idx];

            let mut support_regions = Vec::new();
            let mut interface_regions = Vec::new();

            for element in elements {
                if element.state.bits.deleted {
                    continue;
                }

                branch_count += 1;
                if element.state.distance_to_top == 0 {
                    tip_count += 1;
                }

                // Get the radius for this element
                let radius = element.state.get_radius(&self.config.settings);

                // Create circle at the result point
                if let Some(result_point) = element.state.result_on_layer {
                    let circle = create_circle_polygon(result_point, radius);

                    // Determine if this is an interface layer
                    let is_interface = element.state.bits.supports_roof
                        && element.state.distance_to_top < self.config.num_roof_layers as u32;

                    if is_interface {
                        interface_regions.push(ExPolygon::new(circle));
                    } else {
                        support_regions.push(ExPolygon::new(circle));
                    }
                }
            }

            // Get Z height for this layer
            let z = self.config.settings.get_actual_z(layer_idx);
            let height = self.config.settings.layer_height;

            layers.push(SupportLayer {
                layer_id: layer_idx,
                z: unscale(z),
                height: unscale(height),
                support_regions,
                interface_regions,
                is_interface: false,
                overhang_regions: Vec::new(),
            });
        }

        TreeSupport3DResult {
            layers,
            branch_count,
            tip_count,
        }
    }
}

// ============================================================================
// Helper functions
// ============================================================================

/// Create a circle polygon at a given center with given radius.
fn create_circle_polygon(center: Point, radius: Coord) -> Polygon {
    const NUM_SEGMENTS: usize = 24;
    let mut points = Vec::with_capacity(NUM_SEGMENTS);

    for i in 0..NUM_SEGMENTS {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (NUM_SEGMENTS as f64);
        let x = center.x + (radius as f64 * angle.cos()) as Coord;
        let y = center.y + (radius as f64 * angle.sin()) as Coord;
        points.push(Point::new(x, y));
    }

    Polygon::from_points(points)
}

/// Sample points along a polygon boundary at given spacing.
fn sample_polygon_points(polygon: &Polygon, spacing: Coord) -> Vec<Point> {
    let mut points = Vec::new();
    let poly_points = polygon.points();

    if poly_points.is_empty() {
        return points;
    }

    let mut accumulated_dist: Coord = 0;

    for i in 0..poly_points.len() {
        let p1 = poly_points[i];
        let p2 = poly_points[(i + 1) % poly_points.len()];

        let dx = p2.x - p1.x;
        let dy = p2.y - p1.y;
        let segment_length = ((dx as f64).powi(2) + (dy as f64).powi(2)).sqrt() as Coord;

        if segment_length == 0 {
            continue;
        }

        let mut pos: Coord = 0;
        while pos < segment_length {
            let remaining_to_next = spacing - accumulated_dist;
            if pos + remaining_to_next <= segment_length {
                pos += remaining_to_next;
                accumulated_dist = 0;

                // Calculate point at this position
                let t = pos as f64 / segment_length as f64;
                let x = p1.x + (dx as f64 * t) as Coord;
                let y = p1.y + (dy as f64 * t) as Coord;
                points.push(Point::new(x, y));
            } else {
                accumulated_dist += segment_length - pos;
                break;
            }
        }
    }

    // Always include at least the first point if polygon is non-empty
    if points.is_empty() && !poly_points.is_empty() {
        points.push(poly_points[0]);
    }

    points
}

/// Simple polygon offset (grow/shrink).
fn offset_polygons_simple(polygons: &[Polygon], delta: Coord) -> Vec<Polygon> {
    if polygons.is_empty() || delta == 0 {
        return polygons.to_vec();
    }

    match clipper_utils::offset_polygons(polygons, delta as f64, OffsetJoinType::Round) {
        expolygons => {
            // Extract contours
            expolygons.iter().map(|ex| ex.contour.clone()).collect()
        }
    }
}

/// Safe offset that respects forbidden areas.
fn safe_offset_inc(
    polygons: &[Polygon],
    offset: Coord,
    forbidden: &[Polygon],
    min_offset: Coord,
) -> Vec<Polygon> {
    if polygons.is_empty() {
        return Vec::new();
    }

    // Apply offset
    let expanded = offset_polygons_simple(polygons, offset);

    // Subtract forbidden areas
    let result = difference_polygons(&expanded, forbidden);

    // Filter by minimum area
    result
        .into_iter()
        .filter(|p| polygon_area(p) >= (min_offset as f64).powi(2))
        .collect()
}

/// Difference between two polygon sets.
fn difference_polygons(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
    if subject.is_empty() {
        return Vec::new();
    }
    if clip.is_empty() {
        return subject.to_vec();
    }

    // Convert to ExPolygons for clipper operations
    let subject_ex: Vec<ExPolygon> = subject.iter().map(|p| ExPolygon::new(p.clone())).collect();
    let clip_ex: Vec<ExPolygon> = clip.iter().map(|p| ExPolygon::new(p.clone())).collect();

    let result = clipper_utils::difference(&subject_ex, &clip_ex);

    // Extract contours
    result.iter().map(|ex| ex.contour.clone()).collect()
}

/// Calculate the area of a polygon (in scaled units squared).
fn polygon_area(polygon: &Polygon) -> f64 {
    polygon.area().abs()
}

/// Check if two influence areas overlap.
fn influence_areas_overlap(area1: &[Polygon], area2: &[Polygon]) -> bool {
    if area1.is_empty() || area2.is_empty() {
        return false;
    }

    // Convert to ExPolygons for intersection check
    let ex1: Vec<ExPolygon> = area1.iter().map(|p| ExPolygon::new(p.clone())).collect();
    let ex2: Vec<ExPolygon> = area2.iter().map(|p| ExPolygon::new(p.clone())).collect();

    let intersection = clipper_utils::intersection(&ex1, &ex2);

    !intersection.is_empty()
}

/// Merge two support elements into one.
fn merge_support_elements(mut elem1: SupportElement, elem2: SupportElement) -> SupportElement {
    // Merge parents
    for parent in elem2.parents {
        if !elem1.parents.contains(&parent) {
            elem1.parents.push(parent);
        }
    }

    // Union influence areas
    let ex1: Vec<ExPolygon> = elem1
        .influence_area
        .iter()
        .map(|p| ExPolygon::new(p.clone()))
        .collect();
    let ex2: Vec<ExPolygon> = elem2
        .influence_area
        .iter()
        .map(|p| ExPolygon::new(p.clone()))
        .collect();

    let unioned = clipper_utils::union(&ex1, &ex2);
    elem1.influence_area = unioned.iter().map(|ex| ex.contour.clone()).collect();

    // Merge state flags
    elem1.state.bits.to_buildplate =
        elem1.state.bits.to_buildplate || elem2.state.bits.to_buildplate;
    elem1.state.bits.to_model_gracious =
        elem1.state.bits.to_model_gracious || elem2.state.bits.to_model_gracious;
    elem1.state.bits.supports_roof =
        elem1.state.bits.supports_roof || elem2.state.bits.supports_roof;

    // Use the larger radius
    if elem2.state.radius > elem1.state.radius {
        elem1.state.radius = elem2.state.radius;
    }

    elem1
}

/// Move a point inside the given polygons if it's currently outside.
fn move_inside_if_outside(polygons: &[Polygon], point: Point) -> Point {
    if polygons.is_empty() {
        return point;
    }

    // Check if point is inside any polygon
    for polygon in polygons {
        if polygon.contains_point(&point) {
            return point;
        }
    }

    // Point is outside - find the nearest point on the polygon boundary
    let mut best_point = point;
    let mut best_dist_sq = i64::MAX;

    for polygon in polygons {
        let poly_points = polygon.points();
        for i in 0..poly_points.len() {
            let p1 = poly_points[i];
            let p2 = poly_points[(i + 1) % poly_points.len()];

            // Find closest point on segment p1-p2
            let closest = closest_point_on_segment(point, p1, p2);
            let dist_sq = (closest.x - point.x).pow(2) + (closest.y - point.y).pow(2);

            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_point = closest;
            }
        }
    }

    best_point
}

/// Find the closest point on a line segment to a given point.
fn closest_point_on_segment(point: Point, seg_start: Point, seg_end: Point) -> Point {
    let dx = seg_end.x - seg_start.x;
    let dy = seg_end.y - seg_start.y;
    let length_sq = dx * dx + dy * dy;

    if length_sq == 0 {
        return seg_start;
    }

    // Project point onto line segment
    let t = ((point.x - seg_start.x) * dx + (point.y - seg_start.y) * dy) as f64 / length_sq as f64;
    let t = t.clamp(0.0, 1.0);

    Point::new(
        seg_start.x + (dx as f64 * t) as Coord,
        seg_start.y + (dy as f64 * t) as Coord,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::support::tree_model_volumes::TreeModelVolumesConfig;

    fn make_square_mm(size: f64) -> Polygon {
        let half = scale(size / 2.0);
        Polygon::from_points(vec![
            Point::new(-half, -half),
            Point::new(half, -half),
            Point::new(half, half),
            Point::new(-half, half),
        ])
    }

    #[test]
    fn test_create_circle_polygon() {
        let center = Point::new(0, 0);
        let radius = scale(1.0);
        let circle = create_circle_polygon(center, radius);

        assert_eq!(circle.points().len(), 24);

        // Check all points are approximately radius distance from center
        // Allow 1% tolerance for floating-point rounding in integer coordinates
        for point in circle.points() {
            let dist = ((point.x as f64).powi(2) + (point.y as f64).powi(2)).sqrt();
            let expected = radius as f64;
            let tolerance = expected * 0.01; // 1% tolerance
            assert!(
                (dist - expected).abs() < tolerance,
                "Point distance {} should be close to {}",
                dist,
                expected
            );
        }
    }

    #[test]
    fn test_sample_polygon_points() {
        let square = make_square_mm(10.0);
        let spacing = scale(2.0); // 2mm spacing
        let points = sample_polygon_points(&square, spacing);

        // Should have multiple points along the perimeter
        assert!(!points.is_empty());
        // Perimeter is 40mm, with 2mm spacing should get ~20 points
        assert!(points.len() >= 15 && points.len() <= 25);
    }

    #[test]
    fn test_sample_polygon_points_small_polygon() {
        let square = make_square_mm(1.0);
        let spacing = scale(5.0); // 5mm spacing, larger than polygon
        let points = sample_polygon_points(&square, spacing);

        // Should still have at least one point
        assert!(!points.is_empty());
    }

    #[test]
    fn test_offset_polygons_simple() {
        let square = make_square_mm(10.0);
        let grown = offset_polygons_simple(&[square.clone()], scale(1.0));

        assert!(!grown.is_empty());
        // Grown polygon should have larger area
        let original_area = polygon_area(&square);
        let grown_area = polygon_area(&grown[0]);
        assert!(grown_area > original_area);
    }

    #[test]
    fn test_difference_polygons() {
        let large = make_square_mm(10.0);
        let small = make_square_mm(5.0);

        let diff = difference_polygons(&[large], &[small]);

        // Result should exist (ring shape)
        // Note: The result might be represented differently depending on clipper
        // For a simple case like this, we just verify it's not empty or the same as original
        assert!(!diff.is_empty() || diff.len() > 1);
    }

    #[test]
    fn test_polygon_area() {
        let square = make_square_mm(10.0);
        let area = polygon_area(&square);

        // 10mm x 10mm = 100mm², in scaled units
        let expected = (scale(10.0) as f64).powi(2);
        let tolerance = expected * 0.01; // 1% tolerance
        assert!(
            (area - expected).abs() < tolerance,
            "Area {} should be close to {}",
            area,
            expected
        );
    }

    #[test]
    fn test_closest_point_on_segment() {
        let seg_start = Point::new(0, 0);
        let seg_end = Point::new(scale(10.0), 0);

        // Point directly above middle of segment
        let point = Point::new(scale(5.0), scale(5.0));
        let closest = closest_point_on_segment(point, seg_start, seg_end);

        assert_eq!(closest.x, scale(5.0));
        assert_eq!(closest.y, 0);
    }

    #[test]
    fn test_closest_point_on_segment_before_start() {
        let seg_start = Point::new(scale(10.0), 0);
        let seg_end = Point::new(scale(20.0), 0);

        // Point before segment start
        let point = Point::new(0, 0);
        let closest = closest_point_on_segment(point, seg_start, seg_end);

        // Should clamp to start
        assert_eq!(closest.x, seg_start.x);
        assert_eq!(closest.y, seg_start.y);
    }

    #[test]
    fn test_closest_point_on_segment_after_end() {
        let seg_start = Point::new(0, 0);
        let seg_end = Point::new(scale(10.0), 0);

        // Point after segment end
        let point = Point::new(scale(20.0), 0);
        let closest = closest_point_on_segment(point, seg_start, seg_end);

        // Should clamp to end
        assert_eq!(closest.x, seg_end.x);
        assert_eq!(closest.y, seg_end.y);
    }

    #[test]
    fn test_move_inside_if_outside_already_inside() {
        let square = make_square_mm(10.0);
        let point = Point::new(0, 0); // Center of square

        let result = move_inside_if_outside(&[square], point);

        // Point was inside, should be unchanged
        assert_eq!(result.x, point.x);
        assert_eq!(result.y, point.y);
    }

    #[test]
    fn test_tree_support_3d_config_default() {
        let config = TreeSupport3DConfig::default();

        assert!(config.roof_enabled);
        assert_eq!(config.num_roof_layers, 3);
        assert!(config.minimum_support_area > 0.0);
    }

    #[test]
    fn test_tree_support_3d_config_from_support_config() {
        let mut support_config = SupportConfig::default();
        support_config.support_roof = false;
        support_config.top_interface_layers = 5;
        support_config.tree_branch_diameter = 3.0;

        let config = TreeSupport3DConfig::from_support_config(&support_config);

        assert!(!config.roof_enabled);
        assert_eq!(config.num_roof_layers, 5);
    }

    #[test]
    fn test_tree_support_3d_new() {
        let config = TreeSupport3DConfig::default();
        let volumes_config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(volumes_config);

        let generator = TreeSupport3D::new(config, volumes);

        assert_eq!(generator.num_layers, 0); // No layers in empty volumes
    }

    #[test]
    fn test_merge_support_elements() {
        let state1 = SupportElementState::new(5, Point::new(0, 0), 1.0);
        let state2 = SupportElementState::new(5, Point::new(scale(2.0), 0), 1.5);

        let elem1 = SupportElement::with_parents(
            state1,
            vec![0],
            vec![create_circle_polygon(Point::new(0, 0), scale(1.0))],
        );
        let elem2 = SupportElement::with_parents(
            state2,
            vec![1],
            vec![create_circle_polygon(Point::new(scale(2.0), 0), scale(1.0))],
        );

        let merged = merge_support_elements(elem1, elem2);

        // Should have both parents
        assert_eq!(merged.parents.len(), 2);
        // Should have the larger radius
        assert!((merged.state.radius - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_influence_areas_overlap_yes() {
        let circle1 = create_circle_polygon(Point::new(0, 0), scale(2.0));
        let circle2 = create_circle_polygon(Point::new(scale(1.0), 0), scale(2.0)); // Overlapping

        assert!(influence_areas_overlap(&[circle1], &[circle2]));
    }

    #[test]
    fn test_influence_areas_overlap_no() {
        let circle1 = create_circle_polygon(Point::new(0, 0), scale(1.0));
        let circle2 = create_circle_polygon(Point::new(scale(10.0), 0), scale(1.0)); // Far apart

        assert!(!influence_areas_overlap(&[circle1], &[circle2]));
    }

    #[test]
    fn test_tree_support_3d_result() {
        let result = TreeSupport3DResult {
            layers: Vec::new(),
            branch_count: 10,
            tip_count: 5,
        };

        assert_eq!(result.branch_count, 10);
        assert_eq!(result.tip_count, 5);
    }

    #[test]
    fn test_line_information_types() {
        let info: LineInformation = vec![
            (Point::new(0, 0), LineStatus::ToBuildPlate),
            (Point::new(100, 100), LineStatus::ToModel),
        ];

        assert_eq!(info.len(), 2);
        assert_eq!(info[0].1, LineStatus::ToBuildPlate);
        assert_eq!(info[1].1, LineStatus::ToModel);
    }
}
