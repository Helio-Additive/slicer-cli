//! Organic Smoothing - Branch smoothing and collision avoidance for tree supports.
//!
//! This module implements the organic smoothing algorithm that refines tree support
//! branch positions after initial generation. It uses Laplacian smoothing combined
//! with collision avoidance to create smooth, printable branch paths.
//!
//! # Algorithm Overview
//!
//! 1. **Collision Detection**: For each branch position (sphere), check distance to model
//! 2. **Collision Avoidance**: Push branches away from model surfaces
//! 3. **Laplacian Smoothing**: Average positions with neighbors for smooth curves
//! 4. **Iteration**: Repeat until convergence or max iterations reached
//!
//! # BambuStudio Reference
//!
//! - `Support/TreeSupport3D.cpp` - `organic_smooth_branches_avoid_collisions()`
//! - Uses AABB tree for efficient line-sphere distance queries
//!
//! # Integration with Branch Mesh
//!
//! The organic smoothing output can be wired into the branch mesh generation:
//!
//! 1. Build spheres from `move_bounds` using `build_spheres_from_move_bounds()`
//! 2. Run organic smoothing with `OrganicSmoother::smooth()`
//! 3. Apply smoothed positions back using `apply_smoothed_positions_to_move_bounds()`
//! 4. Generate branch mesh from updated `move_bounds`

use crate::geometry::{ExPolygon, ExPolygons, Point, PointF, Polygon};
use crate::support::tree_support_settings::{SupportElement, TreeSupportSettings};
use crate::{scale, unscale, Coord, CoordF};
use std::collections::HashMap;

/// Configuration for organic smoothing.
#[derive(Debug, Clone)]
pub struct OrganicSmoothConfig {
    /// Maximum distance to nudge for collision avoidance (mm).
    pub max_nudge_collision: CoordF,
    /// Maximum distance to nudge for smoothing (mm).
    pub max_nudge_smoothing: CoordF,
    /// Extra gap to maintain from collisions (mm).
    pub collision_extra_gap: CoordF,
    /// Smoothing factor for Laplacian smoothing (0.0 - 1.0).
    pub smoothing_factor: f64,
    /// Maximum number of smoothing iterations.
    pub max_iterations: usize,
    /// Convergence threshold - stop if fewer than this many spheres moved.
    pub convergence_threshold: usize,
    /// Layer height for Z calculations (mm).
    pub layer_height: CoordF,
}

impl Default for OrganicSmoothConfig {
    fn default() -> Self {
        Self {
            max_nudge_collision: 0.5,
            max_nudge_smoothing: 0.2,
            collision_extra_gap: 0.1,
            smoothing_factor: 0.5,
            max_iterations: 100,
            convergence_threshold: 0,
            layer_height: 0.2,
        }
    }
}

/// A sphere representing a branch position for collision/smoothing.
#[derive(Debug, Clone)]
pub struct CollisionSphere {
    /// Current 3D position (x, y in mm, z is layer height).
    pub position: Point3F,
    /// Radius of the sphere (mm).
    pub radius: CoordF,
    /// Index of element below in the tree (for smoothing).
    pub element_below_id: Option<usize>,
    /// Indices of parent elements above (for smoothing).
    pub parent_ids: Vec<usize>,
    /// Whether this sphere is locked (tips and roots are locked).
    pub locked: bool,
    /// Layer index.
    pub layer_idx: usize,
    /// Previous position for Laplacian smoothing.
    prev_position: Point3F,
    /// Last collision point.
    last_collision: Option<Point3F>,
    /// Last collision depth.
    last_collision_depth: CoordF,
    /// Minimum Z for collision search.
    pub min_z: CoordF,
    /// Maximum Z for collision search.
    pub max_z: CoordF,
}

/// 3D point with floating point coordinates.
#[derive(Debug, Clone, Copy, Default)]
pub struct Point3F {
    pub x: CoordF,
    pub y: CoordF,
    pub z: CoordF,
}

impl Point3F {
    pub fn new(x: CoordF, y: CoordF, z: CoordF) -> Self {
        Self { x, y, z }
    }

    pub fn to_2d(&self) -> PointF {
        PointF::new(self.x, self.y)
    }

    pub fn from_2d(p: PointF, z: CoordF) -> Self {
        Self::new(p.x, p.y, z)
    }

    pub fn distance_xy(&self, other: &Point3F) -> CoordF {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn distance(&self, other: &Point3F) -> CoordF {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

impl CollisionSphere {
    // Create a new collision sphere.
    pub fn new(
        position: Point3F,
        radius: CoordF,
        layer_idx: usize,
        element_below_id: Option<usize>,
        parent_ids: Vec<usize>,
    ) -> Self {
        // Spheres at tips (no parents) and roots (no element below but has layer > 0) are locked
        let locked = parent_ids.is_empty() || (element_below_id.is_none() && layer_idx > 0);

        Self {
            position,
            radius,
            element_below_id,
            parent_ids,
            locked,
            layer_idx,
            prev_position: position,
            last_collision: None,
            last_collision_depth: 0.0,
            min_z: position.z - radius,
            max_z: position.z + radius,
        }
    }

    /// Update the Z search bounds based on tree structure.
    pub fn update_z_bounds(&mut self, min_z_below: CoordF, max_z_above: CoordF) {
        self.min_z = self.min_z.max(min_z_below);
        self.max_z = self.max_z.min(max_z_above);
        // Also limit by sphere radius
        self.min_z = self.min_z.max(self.position.z - self.radius);
        self.max_z = self.max_z.min(self.position.z + self.radius);
    }

    /// Back up current position for smoothing.
    fn backup_position(&mut self) {
        self.prev_position = self.position;
    }

    /// Reset collision state for new iteration.
    fn reset_collision(&mut self) {
        self.last_collision = None;
        self.last_collision_depth = -f64::MAX;
    }
}

/// Cached collision data for a single layer.
#[derive(Debug, Clone, Default)]
pub struct LayerCollisionCache {
    /// Line segments representing model boundary at this layer.
    pub lines: Vec<(PointF, PointF)>,
    /// Minimum element radius that uses this layer.
    pub min_element_radius: CoordF,
    /// Collision radius used for computing this cache.
    pub collision_radius: Coord,
}

impl LayerCollisionCache {
    pub fn new() -> Self {
        Self {
            lines: Vec::new(),
            min_element_radius: CoordF::MAX,
            collision_radius: 0,
        }
    }

    /// Build collision lines from layer outlines.
    pub fn from_outlines(outlines: &ExPolygons, collision_radius: Coord) -> Self {
        let mut cache = Self::new();
        cache.collision_radius = collision_radius;

        for expoly in outlines {
            // Add contour lines
            Self::add_polygon_lines(&expoly.contour, &mut cache.lines);
            // Add hole lines
            for hole in &expoly.holes {
                Self::add_polygon_lines(hole, &mut cache.lines);
            }
        }

        cache
    }

    fn add_polygon_lines(polygon: &Polygon, lines: &mut Vec<(PointF, PointF)>) {
        let points = polygon.points();
        if points.len() < 2 {
            return;
        }

        for i in 0..points.len() {
            let p1 = points[i];
            let p2 = points[(i + 1) % points.len()];
            lines.push((
                PointF::new(unscale(p1.x), unscale(p1.y)),
                PointF::new(unscale(p2.x), unscale(p2.y)),
            ));
        }
    }

    /// Check if this cache is empty.
    pub fn is_empty(&self) -> bool {
        self.lines.is_empty()
    }

    /// Find the closest point on any line to the given point.
    /// Returns (distance, closest_point) or None if no lines.
    pub fn closest_point(&self, point: PointF) -> Option<(CoordF, PointF)> {
        if self.lines.is_empty() {
            return None;
        }

        let mut min_dist = CoordF::MAX;
        let mut closest = PointF::new(0.0, 0.0);

        for (p1, p2) in &self.lines {
            let (dist, pt) = closest_point_on_segment(point, *p1, *p2);
            if dist < min_dist {
                min_dist = dist;
                closest = pt;
            }
        }

        Some((min_dist, closest))
    }
}

/// Find the closest point on a line segment to a given point.
fn closest_point_on_segment(point: PointF, seg_start: PointF, seg_end: PointF) -> (CoordF, PointF) {
    let dx = seg_end.x - seg_start.x;
    let dy = seg_end.y - seg_start.y;

    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-12 {
        // Degenerate segment (point)
        let dist = ((point.x - seg_start.x).powi(2) + (point.y - seg_start.y).powi(2)).sqrt();
        return (dist, seg_start);
    }

    // Project point onto line
    let t = ((point.x - seg_start.x) * dx + (point.y - seg_start.y) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);

    let closest = PointF::new(seg_start.x + t * dx, seg_start.y + t * dy);

    let dist = ((point.x - closest.x).powi(2) + (point.y - closest.y).powi(2)).sqrt();
    (dist, closest)
}

/// Organic smoother for tree support branches.
#[derive(Debug)]
pub struct OrganicSmoother {
    config: OrganicSmoothConfig,
    /// Collision spheres representing branch positions.
    spheres: Vec<CollisionSphere>,
    /// Layer collision caches.
    layer_caches: Vec<LayerCollisionCache>,
    /// Mapping from (layer_idx, element_idx) to sphere index.
    sphere_index: HashMap<(usize, usize), usize>,
}

impl OrganicSmoother {
    // Create a new organic smoother.
    pub fn new(config: OrganicSmoothConfig) -> Self {
        Self {
            config,
            spheres: Vec::new(),
            layer_caches: Vec::new(),
            sphere_index: HashMap::new(),
        }
    }

    /// Add a collision sphere for a branch element.
    pub fn add_sphere(
        &mut self,
        position: Point3F,
        radius: CoordF,
        layer_idx: usize,
        element_idx: usize,
        element_below_id: Option<usize>,
        parent_ids: Vec<usize>,
    ) {
        let sphere_idx = self.spheres.len();
        self.spheres.push(CollisionSphere::new(
            position,
            radius,
            layer_idx,
            element_below_id,
            parent_ids,
        ));
        self.sphere_index
            .insert((layer_idx, element_idx), sphere_idx);
    }

    /// Set the layer collision caches.
    pub fn set_layer_caches(&mut self, caches: Vec<LayerCollisionCache>) {
        self.layer_caches = caches;
    }

    /// Build layer collision caches from layer outlines.
    pub fn build_layer_caches(&mut self, layer_outlines: &[ExPolygons]) {
        self.layer_caches = layer_outlines
            .iter()
            .map(|outlines| LayerCollisionCache::from_outlines(outlines, 0))
            .collect();
    }

    /// Update Z bounds for all spheres based on tree structure.
    pub fn update_z_bounds(&mut self) {
        // First pass: propagate min_z upward (from children to parents)
        for i in 0..self.spheres.len() {
            if let Some(below_idx) = self.spheres[i].element_below_id {
                // Find the sphere below in the index
                let layer_below = self.spheres[i].layer_idx.saturating_sub(1);
                if let Some(&sphere_below_idx) = self.sphere_index.get(&(layer_below, below_idx)) {
                    let min_z_below = self.spheres[sphere_below_idx].min_z;
                    self.spheres[i].min_z = self.spheres[i].min_z.max(min_z_below);
                }
            }
        }

        // Second pass: propagate max_z downward (from parents to children)
        for i in (0..self.spheres.len()).rev() {
            let parent_ids = self.spheres[i].parent_ids.clone();
            if !parent_ids.is_empty() {
                let layer_above = self.spheres[i].layer_idx + 1;
                let mut min_max_z = CoordF::MAX;
                for &parent_idx in &parent_ids {
                    if let Some(&sphere_parent_idx) =
                        self.sphere_index.get(&(layer_above, parent_idx))
                    {
                        min_max_z = min_max_z.min(self.spheres[sphere_parent_idx].max_z);
                    }
                }
                if min_max_z < CoordF::MAX {
                    self.spheres[i].max_z = self.spheres[i].max_z.min(min_max_z);
                }
            }

            // Final bounds limiting by radius
            self.spheres[i].min_z = self.spheres[i]
                .min_z
                .max(self.spheres[i].position.z - self.spheres[i].radius);
            self.spheres[i].max_z = self.spheres[i]
                .max_z
                .min(self.spheres[i].position.z + self.spheres[i].radius);
        }
    }

    /// Run the organic smoothing algorithm.
    ///
    /// Returns the number of iterations performed.
    pub fn smooth(&mut self) -> usize {
        self.update_z_bounds();

        for iter in 0..self.config.max_iterations {
            // Backup positions for Laplacian smoothing
            for sphere in &mut self.spheres {
                sphere.backup_position();
                sphere.reset_collision();
            }

            let mut num_moved = 0;

            // Process each sphere
            for sphere_idx in 0..self.spheres.len() {
                if self.spheres[sphere_idx].locked {
                    continue;
                }

                // Check collision with model
                if self.check_and_resolve_collision(sphere_idx) {
                    num_moved += 1;
                }

                // Apply Laplacian smoothing
                self.apply_laplacian_smoothing(sphere_idx);
            }

            // Check for convergence
            if num_moved <= self.config.convergence_threshold {
                return iter + 1;
            }
        }

        self.config.max_iterations
    }

    /// Check collision for a sphere and nudge it away if needed.
    /// Returns true if the sphere was moved due to collision.
    fn check_and_resolve_collision(&mut self, sphere_idx: usize) -> bool {
        let sphere = &self.spheres[sphere_idx];

        // Determine which layers to check based on Z bounds
        let min_layer = self.z_to_layer_idx(sphere.min_z);
        let max_layer = self.z_to_layer_idx(sphere.max_z);

        let mut max_collision_depth = -CoordF::MAX;
        let mut collision_point = None;

        // Check each relevant layer
        for layer_idx in min_layer..=max_layer {
            if layer_idx >= self.layer_caches.len() {
                continue;
            }

            let cache = &self.layer_caches[layer_idx];
            if cache.is_empty() {
                continue;
            }

            // Calculate effective radius at this Z height
            let layer_z = layer_idx as CoordF * self.config.layer_height;
            let dz = layer_z - sphere.position.z;
            let r_squared = sphere.radius * sphere.radius - dz * dz;

            if r_squared <= 0.0 {
                continue;
            }

            let effective_radius = r_squared.sqrt();

            // Find closest point on model boundary
            let point_2d = sphere.position.to_2d();
            if let Some((dist, closest)) = cache.closest_point(point_2d) {
                let collision_depth = effective_radius - dist;
                if collision_depth > max_collision_depth {
                    max_collision_depth = collision_depth;
                    collision_point = Some(Point3F::from_2d(closest, layer_z));
                }
            }
        }

        // Update sphere collision info
        self.spheres[sphere_idx].last_collision_depth = max_collision_depth;
        self.spheres[sphere_idx].last_collision = collision_point;

        // Nudge away from collision if needed
        if max_collision_depth > 0.0 {
            if let Some(collision_pt) = collision_point {
                let sphere = &mut self.spheres[sphere_idx];

                // Calculate nudge direction (away from collision)
                let dx = sphere.position.x - collision_pt.x;
                let dy = sphere.position.y - collision_pt.y;
                let dist = (dx * dx + dy * dy).sqrt();

                if dist > 1e-10 {
                    // Normalize and scale
                    let nudge_dist = (max_collision_depth + self.config.collision_extra_gap)
                        .min(self.config.max_nudge_collision);

                    sphere.position.x += dx / dist * nudge_dist;
                    sphere.position.y += dy / dist * nudge_dist;
                }
            }
            return max_collision_depth > 1e-6;
        }

        false
    }

    /// Apply Laplacian smoothing to a sphere.
    fn apply_laplacian_smoothing(&mut self, sphere_idx: usize) {
        let sphere = &self.spheres[sphere_idx];
        if sphere.locked {
            return;
        }

        let mut avg_x = 0.0;
        let mut avg_y = 0.0;
        let mut total_weight = 0.0;

        // Average with parents (above)
        let parent_ids = sphere.parent_ids.clone();
        let layer_above = sphere.layer_idx + 1;
        for &parent_idx in &parent_ids {
            if let Some(&parent_sphere_idx) = self.sphere_index.get(&(layer_above, parent_idx)) {
                let parent = &self.spheres[parent_sphere_idx];
                let weight = sphere.radius; // Weight by radius
                avg_x += weight * parent.prev_position.x;
                avg_y += weight * parent.prev_position.y;
                total_weight += weight;
            }
        }

        // Average with element below
        if let Some(below_idx) = sphere.element_below_id {
            let layer_below = sphere.layer_idx.saturating_sub(1);
            if let Some(&below_sphere_idx) = self.sphere_index.get(&(layer_below, below_idx)) {
                let below = &self.spheres[below_sphere_idx];
                let weight = total_weight.max(sphere.radius); // Balance with parents
                avg_x += weight * below.prev_position.x;
                avg_y += weight * below.prev_position.y;
                total_weight += weight;
            }
        }

        if total_weight < 1e-10 {
            return;
        }

        avg_x /= total_weight;
        avg_y /= total_weight;

        // Apply smoothing
        let sphere = &mut self.spheres[sphere_idx];
        let old_x = sphere.position.x;
        let old_y = sphere.position.y;

        let new_x =
            (1.0 - self.config.smoothing_factor) * old_x + self.config.smoothing_factor * avg_x;
        let new_y =
            (1.0 - self.config.smoothing_factor) * old_y + self.config.smoothing_factor * avg_y;

        // Limit the smoothing nudge distance
        let shift_x = new_x - old_x;
        let shift_y = new_y - old_y;
        let shift_dist = (shift_x * shift_x + shift_y * shift_y).sqrt();

        if shift_dist > self.config.max_nudge_smoothing {
            let scale = self.config.max_nudge_smoothing / shift_dist;
            sphere.position.x = old_x + shift_x * scale;
            sphere.position.y = old_y + shift_y * scale;
        } else {
            sphere.position.x = new_x;
            sphere.position.y = new_y;
        }
    }

    /// Convert Z coordinate to layer index.
    fn z_to_layer_idx(&self, z: CoordF) -> usize {
        if z <= 0.0 {
            return 0;
        }
        (z / self.config.layer_height).floor() as usize
    }

    /// Get the smoothed positions of all spheres.
    pub fn get_smoothed_positions(&self) -> Vec<(usize, usize, Point3F)> {
        self.spheres
            .iter()
            .map(|s| (s.layer_idx, 0, s.position)) // Note: element_idx not stored
            .collect()
    }

    /// Get the positions as scaled Points for each layer.
    pub fn get_layer_positions(&self) -> HashMap<usize, Vec<Point>> {
        let mut result: HashMap<usize, Vec<Point>> = HashMap::new();

        for sphere in &self.spheres {
            let point = Point::new(scale(sphere.position.x), scale(sphere.position.y));
            result.entry(sphere.layer_idx).or_default().push(point);
        }

        result
    }

    /// Get sphere count.
    pub fn sphere_count(&self) -> usize {
        self.spheres.len()
    }

    /// Get a reference to the spheres.
    pub fn spheres(&self) -> &[CollisionSphere] {
        &self.spheres
    }

    /// Get a mutable reference to a sphere by index.
    pub fn sphere_mut(&mut self, idx: usize) -> Option<&mut CollisionSphere> {
        self.spheres.get_mut(idx)
    }
}

/// Result of organic smoothing.
#[derive(Debug, Clone)]
pub struct OrganicSmoothResult {
    /// Number of iterations performed.
    pub iterations: usize,
    /// Final positions per layer (layer_idx -> positions).
    pub positions: HashMap<usize, Vec<Point>>,
    /// Whether smoothing converged before max iterations.
    pub converged: bool,
}

/// Smooth tree support branches using organic smoothing.
///
/// This is a high-level function that takes branch positions and model outlines,
/// applies organic smoothing, and returns the smoothed positions.
pub fn smooth_branches(
    branch_positions: &[(usize, Point3F, CoordF, Option<usize>, Vec<usize>)],
    layer_outlines: &[ExPolygons],
    config: OrganicSmoothConfig,
) -> OrganicSmoothResult {
    let max_iterations = config.max_iterations;
    let mut smoother = OrganicSmoother::new(config);

    // Add all spheres
    for (idx, (layer_idx, position, radius, below_id, parent_ids)) in
        branch_positions.iter().enumerate()
    {
        smoother.add_sphere(
            *position,
            *radius,
            *layer_idx,
            idx,
            *below_id,
            parent_ids.clone(),
        );
    }

    // Build collision caches
    smoother.build_layer_caches(layer_outlines);

    // Run smoothing
    let iterations = smoother.smooth();

    // Get results
    let positions = smoother.get_layer_positions();
    let converged = iterations < max_iterations;

    OrganicSmoothResult {
        iterations,
        positions,
        converged,
    }
}

/// Information about a sphere built from a support element.
/// Used to map smoothed positions back to move_bounds.
#[derive(Debug, Clone)]
pub struct SphereMapping {
    /// Layer index in move_bounds.
    pub layer_idx: usize,
    /// Element index within the layer.
    pub element_idx: usize,
    /// Index of the sphere in the smoother.
    pub sphere_idx: usize,
}

/// Result of building spheres from move_bounds.
#[derive(Debug)]
pub struct SphereBuildResult {
    /// The organic smoother with spheres added.
    pub smoother: OrganicSmoother,
    /// Mapping from sphere index to move_bounds location.
    pub mappings: Vec<SphereMapping>,
    /// Linear data layers (cumulative count of elements up to each layer).
    pub linear_data_layers: Vec<usize>,
}

/// Build collision spheres from move_bounds support elements.
///
/// This function extracts branch positions from the support elements and creates
/// collision spheres for organic smoothing. It mimics the C++ implementation's
/// `elements_with_link_down` construction.
///
/// # Arguments
/// * `move_bounds` - Support elements per layer (from TreeSupport3D)
/// * `settings` - Tree support settings for radius and Z calculations
/// * `config` - Organic smoothing configuration
///
/// # Returns
/// SphereBuildResult containing the smoother with spheres and mapping information.
pub fn build_spheres_from_move_bounds(
    move_bounds: &[Vec<SupportElement>],
    settings: &TreeSupportSettings,
    config: OrganicSmoothConfig,
) -> SphereBuildResult {
    let mut smoother = OrganicSmoother::new(config);
    let mut mappings = Vec::new();
    let mut linear_data_layers = Vec::new();

    // Build downward links: for each element, find which element below it connects to
    // In BambuStudio, this is done by tracking parent relationships
    let mut map_downwards_old: Vec<(usize, usize, usize)> = Vec::new(); // (layer, elem_idx, child_idx)
    let mut map_downwards_new: Vec<(usize, usize, usize)> = Vec::new();

    linear_data_layers.push(0);

    for layer_idx in 0..move_bounds.len() {
        let layer = &move_bounds[layer_idx];
        map_downwards_new.clear();

        // Sort old mappings for binary search
        map_downwards_old.sort_by_key(|x| (x.0, x.1));

        for elem_idx in 0..layer.len() {
            let elem = &layer[elem_idx];

            // Skip deleted elements
            if elem.state.bits.deleted {
                continue;
            }

            // Skip elements without result_on_layer set
            let position = match elem.state.result_on_layer {
                Some(p) => p,
                None => continue,
            };

            // Find child (element below that links to this one)
            let mut child_idx: Option<usize> = None;
            if layer_idx > 0 {
                // Search in map_downwards_old for this element
                for &(_, parent_elem, child) in &map_downwards_old {
                    if parent_elem == elem_idx {
                        child_idx = Some(child);
                        break;
                    }
                }
            }

            // Build parent links for elements in layer above
            if layer_idx + 1 < move_bounds.len() {
                let layer_above = &move_bounds[layer_idx + 1];
                for &parent_idx in &elem.parents {
                    let parent_idx = parent_idx as usize;
                    if parent_idx < layer_above.len() {
                        let parent = &layer_above[parent_idx];
                        if parent.state.result_on_layer_is_set() {
                            map_downwards_new.push((layer_idx + 1, parent_idx, elem_idx));
                        }
                    }
                }
            }

            // Get position and radius
            let z = unscale(settings.get_actual_z(layer_idx));
            let radius = unscale(elem.state.get_radius(settings));
            let pos_3f = Point3F::new(unscale(position.x), unscale(position.y), z);

            // Determine if this sphere is locked (tips and roots are locked)
            // Tips: no parents, Roots: no child and not at layer 0
            let is_tip = elem.parents.is_empty();
            let is_root = child_idx.is_none() && layer_idx > 0;
            let locked = is_tip || is_root;

            // Get parent indices for smoothing
            let parent_ids: Vec<usize> = elem.parents.iter().map(|&p| p as usize).collect();

            let sphere_idx = smoother.sphere_count();
            smoother.add_sphere(pos_3f, radius, layer_idx, elem_idx, child_idx, parent_ids);

            // Mark locked status
            if let Some(sphere) = smoother.sphere_mut(sphere_idx) {
                sphere.locked = locked;
            }

            mappings.push(SphereMapping {
                layer_idx,
                element_idx: elem_idx,
                sphere_idx,
            });
        }

        std::mem::swap(&mut map_downwards_old, &mut map_downwards_new);
        linear_data_layers.push(mappings.len());
    }

    SphereBuildResult {
        smoother,
        mappings,
        linear_data_layers,
    }
}

/// Apply smoothed positions from the organic smoother back to move_bounds.
///
/// This function takes the smoothed sphere positions and updates the corresponding
/// `result_on_layer` fields in the support elements.
///
/// # Arguments
/// * `smoother` - The organic smoother after running smoothing
/// * `mappings` - Sphere to move_bounds mappings from build_spheres_from_move_bounds
/// * `move_bounds` - Mutable reference to support elements per layer
///
/// # Returns
/// Number of positions updated.
pub fn apply_smoothed_positions_to_move_bounds(
    smoother: &OrganicSmoother,
    mappings: &[SphereMapping],
    move_bounds: &mut [Vec<SupportElement>],
) -> usize {
    let spheres = smoother.spheres();
    let mut updated = 0;

    for mapping in mappings {
        if mapping.sphere_idx >= spheres.len() {
            continue;
        }

        let sphere = &spheres[mapping.sphere_idx];

        if mapping.layer_idx >= move_bounds.len() {
            continue;
        }

        let layer = &mut move_bounds[mapping.layer_idx];
        if mapping.element_idx >= layer.len() {
            continue;
        }

        let element = &mut layer[mapping.element_idx];

        // Update result_on_layer with smoothed position
        let new_point = Point::new(scale(sphere.position.x), scale(sphere.position.y));
        element.state.result_on_layer = Some(new_point);
        updated += 1;
    }

    updated
}

/// High-level function to apply organic smoothing to move_bounds.
///
/// This is a convenience function that combines building spheres, running
/// smoothing, and applying the results back to move_bounds.
///
/// # Arguments
/// * `move_bounds` - Mutable reference to support elements per layer
/// * `settings` - Tree support settings
/// * `layer_outlines` - Model outlines per layer for collision detection
/// * `config` - Organic smoothing configuration
///
/// # Returns
/// OrganicSmoothResult with iteration count and convergence status.
pub fn smooth_move_bounds(
    move_bounds: &mut [Vec<SupportElement>],
    settings: &TreeSupportSettings,
    layer_outlines: &[ExPolygons],
    config: OrganicSmoothConfig,
) -> OrganicSmoothResult {
    let max_iterations = config.max_iterations;

    // Build spheres from move_bounds
    let mut build_result = build_spheres_from_move_bounds(move_bounds, settings, config);

    if build_result.smoother.sphere_count() == 0 {
        return OrganicSmoothResult {
            iterations: 0,
            positions: HashMap::new(),
            converged: true,
        };
    }

    // Build collision caches from layer outlines
    build_result.smoother.build_layer_caches(layer_outlines);

    // Run smoothing
    let iterations = build_result.smoother.smooth();
    let converged = iterations < max_iterations;

    // Apply smoothed positions back to move_bounds
    let _updated = apply_smoothed_positions_to_move_bounds(
        &build_result.smoother,
        &build_result.mappings,
        move_bounds,
    );

    // Get final positions
    let positions = build_result.smoother.get_layer_positions();

    OrganicSmoothResult {
        iterations,
        positions,
        converged,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::support::tree_support_settings::SupportElementState;

    #[test]
    fn test_point3f_new() {
        let p = Point3F::new(1.0, 2.0, 3.0);
        assert!((p.x - 1.0).abs() < 0.001);
        assert!((p.y - 2.0).abs() < 0.001);
        assert!((p.z - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_point3f_to_2d() {
        let p = Point3F::new(1.0, 2.0, 3.0);
        let p2d = p.to_2d();
        assert!((p2d.x - 1.0).abs() < 0.001);
        assert!((p2d.y - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_point3f_distance_xy() {
        let p1 = Point3F::new(0.0, 0.0, 0.0);
        let p2 = Point3F::new(3.0, 4.0, 10.0);
        let dist = p1.distance_xy(&p2);
        assert!((dist - 5.0).abs() < 0.001); // 3-4-5 triangle
    }

    #[test]
    fn test_point3f_distance() {
        let p1 = Point3F::new(0.0, 0.0, 0.0);
        let p2 = Point3F::new(1.0, 0.0, 0.0);
        let dist = p1.distance(&p2);
        assert!((dist - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_collision_sphere_new() {
        let sphere = CollisionSphere::new(Point3F::new(1.0, 2.0, 3.0), 0.5, 5, Some(3), vec![1, 2]);

        assert!((sphere.position.x - 1.0).abs() < 0.001);
        assert!((sphere.radius - 0.5).abs() < 0.001);
        assert_eq!(sphere.layer_idx, 5);
        assert!(!sphere.locked); // Has both below and parents, not locked
    }

    #[test]
    fn test_collision_sphere_locked_tip() {
        let sphere = CollisionSphere::new(
            Point3F::new(1.0, 2.0, 3.0),
            0.5,
            5,
            Some(3),
            vec![], // No parents = tip
        );
        assert!(sphere.locked);
    }

    #[test]
    fn test_collision_sphere_locked_root() {
        let sphere = CollisionSphere::new(
            Point3F::new(1.0, 2.0, 3.0),
            0.5,
            5,
            None, // No element below
            vec![1, 2],
        );
        assert!(sphere.locked); // layer > 0 and no below = root-like
    }

    #[test]
    fn test_closest_point_on_segment() {
        let p = PointF::new(0.0, 1.0);
        let seg_start = PointF::new(-1.0, 0.0);
        let seg_end = PointF::new(1.0, 0.0);

        let (dist, closest) = closest_point_on_segment(p, seg_start, seg_end);

        assert!((dist - 1.0).abs() < 0.001);
        assert!((closest.x - 0.0).abs() < 0.001);
        assert!((closest.y - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_closest_point_on_segment_endpoint() {
        let p = PointF::new(-2.0, 0.0);
        let seg_start = PointF::new(-1.0, 0.0);
        let seg_end = PointF::new(1.0, 0.0);

        let (dist, closest) = closest_point_on_segment(p, seg_start, seg_end);

        assert!((dist - 1.0).abs() < 0.001);
        assert!((closest.x - (-1.0)).abs() < 0.001);
    }

    #[test]
    fn test_layer_collision_cache_empty() {
        let cache = LayerCollisionCache::new();
        assert!(cache.is_empty());
        assert!(cache.closest_point(PointF::new(0.0, 0.0)).is_none());
    }

    #[test]
    fn test_organic_smooth_config_default() {
        let config = OrganicSmoothConfig::default();
        assert!((config.smoothing_factor - 0.5).abs() < 0.001);
        assert_eq!(config.max_iterations, 100);
    }

    #[test]
    fn test_organic_smoother_new() {
        let config = OrganicSmoothConfig::default();
        let smoother = OrganicSmoother::new(config);
        assert_eq!(smoother.sphere_count(), 0);
    }

    #[test]
    fn test_organic_smoother_add_sphere() {
        let config = OrganicSmoothConfig::default();
        let mut smoother = OrganicSmoother::new(config);

        smoother.add_sphere(Point3F::new(1.0, 2.0, 0.2), 0.5, 0, 0, None, vec![0]);

        assert_eq!(smoother.sphere_count(), 1);
    }

    #[test]
    fn test_organic_smoother_z_to_layer() {
        let config = OrganicSmoothConfig {
            layer_height: 0.2,
            ..Default::default()
        };
        let smoother = OrganicSmoother::new(config);

        assert_eq!(smoother.z_to_layer_idx(0.0), 0);
        assert_eq!(smoother.z_to_layer_idx(0.1), 0);
        assert_eq!(smoother.z_to_layer_idx(0.2), 1);
        assert_eq!(smoother.z_to_layer_idx(0.5), 2);
    }

    #[test]
    fn test_organic_smooth_result() {
        let result = OrganicSmoothResult {
            iterations: 10,
            positions: HashMap::new(),
            converged: true,
        };

        assert_eq!(result.iterations, 10);
        assert!(result.converged);
    }

    #[test]
    fn test_smooth_branches_empty() {
        let config = OrganicSmoothConfig::default();
        let branch_positions: Vec<(usize, Point3F, CoordF, Option<usize>, Vec<usize>)> = vec![];
        let layer_outlines: Vec<ExPolygons> = vec![];

        let result = smooth_branches(&branch_positions, &layer_outlines, config);

        assert!(result.converged);
        assert!(result.positions.is_empty());
    }

    #[test]
    fn test_smooth_branches_single_locked() {
        let config = OrganicSmoothConfig {
            max_iterations: 10,
            ..Default::default()
        };

        // Single tip (locked)
        let branch_positions = vec![(
            0,
            Point3F::new(5.0, 5.0, 0.2),
            0.5,
            None,
            vec![], // No parents = tip = locked
        )];
        let layer_outlines: Vec<ExPolygons> = vec![vec![]];

        let result = smooth_branches(&branch_positions, &layer_outlines, config);

        // Should converge immediately since only locked sphere
        assert!(result.converged);
    }

    #[test]
    fn test_sphere_mapping() {
        let mapping = SphereMapping {
            layer_idx: 5,
            element_idx: 3,
            sphere_idx: 10,
        };
        assert_eq!(mapping.layer_idx, 5);
        assert_eq!(mapping.element_idx, 3);
        assert_eq!(mapping.sphere_idx, 10);
    }

    #[test]
    fn test_build_spheres_from_move_bounds_empty() {
        let move_bounds: Vec<Vec<SupportElement>> = vec![];
        let settings = TreeSupportSettings::default();
        let config = OrganicSmoothConfig::default();

        let result = build_spheres_from_move_bounds(&move_bounds, &settings, config);

        assert_eq!(result.smoother.sphere_count(), 0);
        assert!(result.mappings.is_empty());
        assert_eq!(result.linear_data_layers.len(), 1); // Just the initial 0
    }

    #[test]
    fn test_build_spheres_from_move_bounds_no_result_on_layer() {
        // Elements without result_on_layer set should be skipped
        let state = SupportElementState::default();
        let element = SupportElement::new(state, vec![]);
        let move_bounds = vec![vec![element]];
        let settings = TreeSupportSettings::default();
        let config = OrganicSmoothConfig::default();

        let result = build_spheres_from_move_bounds(&move_bounds, &settings, config);

        // Element has no result_on_layer, so no sphere created
        assert_eq!(result.smoother.sphere_count(), 0);
        assert!(result.mappings.is_empty());
    }

    #[test]
    fn test_build_spheres_from_move_bounds_with_element() {
        // Create element with result_on_layer set
        let mut state = SupportElementState::default();
        state.result_on_layer = Some(Point::new(scale(5.0), scale(10.0)));
        let element = SupportElement::new(state, vec![]);

        let move_bounds = vec![vec![element]];
        let mut settings = TreeSupportSettings::default();
        settings.known_z = vec![scale(0.2)]; // Layer 0 at 0.2mm

        let config = OrganicSmoothConfig::default();

        let result = build_spheres_from_move_bounds(&move_bounds, &settings, config);

        assert_eq!(result.smoother.sphere_count(), 1);
        assert_eq!(result.mappings.len(), 1);
        assert_eq!(result.mappings[0].layer_idx, 0);
        assert_eq!(result.mappings[0].element_idx, 0);
        assert_eq!(result.mappings[0].sphere_idx, 0);

        // Check sphere position
        let spheres = result.smoother.spheres();
        assert!((spheres[0].position.x - 5.0).abs() < 0.001);
        assert!((spheres[0].position.y - 10.0).abs() < 0.001);
    }

    #[test]
    fn test_apply_smoothed_positions_to_move_bounds_empty() {
        let config = OrganicSmoothConfig::default();
        let smoother = OrganicSmoother::new(config);
        let mappings: Vec<SphereMapping> = vec![];
        let mut move_bounds: Vec<Vec<SupportElement>> = vec![];

        let updated =
            apply_smoothed_positions_to_move_bounds(&smoother, &mappings, &mut move_bounds);

        assert_eq!(updated, 0);
    }

    #[test]
    fn test_apply_smoothed_positions_to_move_bounds() {
        let config = OrganicSmoothConfig::default();
        let mut smoother = OrganicSmoother::new(config);

        // Add a sphere
        smoother.add_sphere(Point3F::new(15.0, 20.0, 0.2), 0.5, 0, 0, None, vec![]);

        let mappings = vec![SphereMapping {
            layer_idx: 0,
            element_idx: 0,
            sphere_idx: 0,
        }];

        // Create element with initial position
        let mut state = SupportElementState::default();
        state.result_on_layer = Some(Point::new(scale(5.0), scale(10.0)));
        let element = SupportElement::new(state, vec![]);
        let mut move_bounds = vec![vec![element]];

        let updated =
            apply_smoothed_positions_to_move_bounds(&smoother, &mappings, &mut move_bounds);

        assert_eq!(updated, 1);

        // Check position was updated
        let new_pos = move_bounds[0][0].state.result_on_layer.unwrap();
        assert!((unscale(new_pos.x) - 15.0).abs() < 0.001);
        assert!((unscale(new_pos.y) - 20.0).abs() < 0.001);
    }

    #[test]
    fn test_smooth_move_bounds_empty() {
        let mut move_bounds: Vec<Vec<SupportElement>> = vec![];
        let settings = TreeSupportSettings::default();
        let layer_outlines: Vec<ExPolygons> = vec![];
        let config = OrganicSmoothConfig::default();

        let result = smooth_move_bounds(&mut move_bounds, &settings, &layer_outlines, config);

        assert!(result.converged);
        assert_eq!(result.iterations, 0);
    }

    #[test]
    fn test_smooth_move_bounds_single_element() {
        // Create element with result_on_layer set
        let mut state = SupportElementState::default();
        state.result_on_layer = Some(Point::new(scale(5.0), scale(10.0)));
        let element = SupportElement::new(state, vec![]);

        let mut move_bounds = vec![vec![element]];
        let mut settings = TreeSupportSettings::default();
        settings.known_z = vec![scale(0.2)];

        let layer_outlines: Vec<ExPolygons> = vec![vec![]];
        let config = OrganicSmoothConfig {
            max_iterations: 10,
            ..Default::default()
        };

        let result = smooth_move_bounds(&mut move_bounds, &settings, &layer_outlines, config);

        // Single locked element should converge quickly
        assert!(result.converged);

        // Position should be preserved (locked tip)
        let pos = move_bounds[0][0].state.result_on_layer.unwrap();
        assert!((unscale(pos.x) - 5.0).abs() < 0.01);
        assert!((unscale(pos.y) - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_sphere_build_result_linear_data_layers() {
        // Create elements across multiple layers
        let mut state1 = SupportElementState::default();
        state1.result_on_layer = Some(Point::new(scale(1.0), scale(1.0)));
        let elem1 = SupportElement::new(state1, vec![]);

        let mut state2 = SupportElementState::default();
        state2.result_on_layer = Some(Point::new(scale(2.0), scale(2.0)));
        let mut elem2 = SupportElement::new(state2, vec![]);
        elem2.parents = vec![0]; // Points to elem in layer above

        let mut state3 = SupportElementState::default();
        state3.result_on_layer = Some(Point::new(scale(3.0), scale(3.0)));
        let mut elem3 = SupportElement::new(state3, vec![]);
        elem3.parents = vec![0]; // Points to elem2

        let move_bounds = vec![vec![elem1], vec![elem2], vec![elem3]];
        let mut settings = TreeSupportSettings::default();
        settings.known_z = vec![scale(0.2), scale(0.4), scale(0.6)];

        let config = OrganicSmoothConfig::default();
        let result = build_spheres_from_move_bounds(&move_bounds, &settings, config);

        assert_eq!(result.smoother.sphere_count(), 3);
        assert_eq!(result.mappings.len(), 3);

        // linear_data_layers should track cumulative counts
        // [0, count_after_layer0, count_after_layer1, count_after_layer2]
        assert_eq!(result.linear_data_layers.len(), 4);
        assert_eq!(result.linear_data_layers[0], 0);
        assert_eq!(result.linear_data_layers[1], 1);
        assert_eq!(result.linear_data_layers[2], 2);
        assert_eq!(result.linear_data_layers[3], 3);
    }
}
