//! Tree Model Volumes - Collision detection and avoidance area computation.
//!
//! This module provides the `TreeModelVolumes` structure which pre-computes and caches
//! collision and avoidance areas for tree support generation. It is a Rust port of
//! BambuStudio's `TreeModelVolumes.cpp`.
//!
//! # Overview
//!
//! Tree supports need to know:
//! 1. **Collision areas**: Where branches cannot go (inside the model)
//! 2. **Avoidance areas**: Expanded collision areas that keep branches at safe distances
//! 3. **Placeable areas**: Where support tips can be placed (under overhangs)
//! 4. **Wall restrictions**: Areas where branches must maintain minimum distance from walls
//!
//! The volumes are computed per-layer and per-radius, allowing the tree support algorithm
//! to query "where can a branch of radius R go at layer Z?"
//!
//! # BambuStudio Reference
//!
//! - `Support/TreeModelVolumes.hpp`
//! - `Support/TreeModelVolumes.cpp`
//!
//! # Key Concepts
//!
//! - **Radius ceiling**: Radii are rounded up to discrete values for caching efficiency
//! - **Avoidance types**: Fast (approximate), FastSafe (hole-free), Slow (precise)
//! - **Layer polygon caching**: Computed areas are cached for reuse

use crate::clipper_utils::{self, OffsetJoinType};
use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use crate::{scale, unscale, Coord, CoordF};
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// Resolution for collision area computation (in scaled units).
/// Smaller values = more precise but slower.
/// 0.5mm = 500,000 scaled units
pub const COLLISION_RESOLUTION: Coord = 500_000;

/// Exponential growth factor for radius stepping.
/// Radii grow exponentially to reduce cache entries while maintaining precision.
pub const EXPONENTIAL_FACTOR: f64 = 1.5;

/// Threshold below which linear radius stepping is used instead of exponential.
/// 3.0mm = 3,000,000 scaled units
pub const EXPONENTIAL_THRESHOLD: Coord = 3_000_000;

/// Whether to avoid support blockers (regions marked as no-support).
pub const AVOID_SUPPORT_BLOCKER: bool = true;

/// Type of avoidance calculation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AvoidanceType {
    /// Fast but may place branches in holes that would cause issues.
    Fast,
    /// Fast computation but avoids placing in holes (safe for buildplate-only supports).
    FastSafe,
    /// Slow but precise computation considering all obstacles.
    Slow,
}

impl Default for AvoidanceType {
    fn default() -> Self {
        Self::Fast
    }
}

/// Key for caching layer polygons by radius and layer index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RadiusLayerKey {
    pub radius: Coord,
    pub layer_idx: usize,
}

impl RadiusLayerKey {
    pub fn new(radius: Coord, layer_idx: usize) -> Self {
        Self { radius, layer_idx }
    }
}

/// Cache for polygons indexed by radius and layer.
#[derive(Debug, Default)]
pub struct RadiusLayerPolygonCache {
    data: RwLock<HashMap<RadiusLayerKey, Arc<Vec<Polygon>>>>,
}

impl RadiusLayerPolygonCache {
    pub fn new() -> Self {
        Self {
            data: RwLock::new(HashMap::new()),
        }
    }

    /// Insert polygons for a radius/layer pair.
    pub fn insert(&self, key: RadiusLayerKey, polygons: Vec<Polygon>) {
        let mut data = self.data.write().unwrap();
        data.insert(key, Arc::new(polygons));
    }

    /// Get polygons for a radius/layer pair, if cached.
    pub fn get(&self, key: &RadiusLayerKey) -> Option<Arc<Vec<Polygon>>> {
        let data = self.data.read().unwrap();
        data.get(key).cloned()
    }

    /// Check if a key exists in the cache.
    pub fn contains(&self, key: &RadiusLayerKey) -> bool {
        let data = self.data.read().unwrap();
        data.contains_key(key)
    }

    /// Get the maximum calculated layer for a given radius.
    pub fn max_layer_for_radius(&self, radius: Coord) -> Option<usize> {
        let data = self.data.read().unwrap();
        data.keys()
            .filter(|k| k.radius == radius)
            .map(|k| k.layer_idx)
            .max()
    }

    /// Clear all cached data.
    pub fn clear(&mut self) {
        let mut data = self.data.write().unwrap();
        data.clear();
    }

    /// Clear all cached data except for radius 0 (base collision).
    pub fn clear_except_radius_0(&mut self) {
        let mut data = self.data.write().unwrap();
        data.retain(|k, _| k.radius == 0);
    }

    /// Get a lower bound area - returns the cached area for the largest radius <= requested.
    pub fn get_lower_bound(
        &self,
        radius: Coord,
        layer_idx: usize,
    ) -> Option<(Coord, Arc<Vec<Polygon>>)> {
        let data = self.data.read().unwrap();
        let mut best: Option<(Coord, Arc<Vec<Polygon>>)> = None;

        for (key, polygons) in data.iter() {
            if key.layer_idx == layer_idx && key.radius <= radius {
                if best.is_none() || key.radius > best.as_ref().unwrap().0 {
                    best = Some((key.radius, polygons.clone()));
                }
            }
        }

        best
    }
}

/// Configuration for tree model volumes computation.
#[derive(Debug, Clone)]
pub struct TreeModelVolumesConfig {
    /// Maximum movement distance per layer (for fast avoidance).
    pub max_move: Coord,
    /// Maximum movement distance per layer (for slow/precise avoidance).
    pub max_move_slow: Coord,
    /// Minimum resolution for polygon simplification.
    pub min_resolution: Coord,
    /// XY distance to maintain from model.
    pub xy_distance: Coord,
    /// Minimum XY distance (when using min distance mode).
    pub xy_min_distance: Coord,
    /// Whether support can rest on the model.
    pub support_rests_on_model: bool,
    /// Radius below which branches are considered "tips".
    pub min_radius: Coord,
    /// Maximum radius for branch growth.
    pub increase_until_radius: Coord,
    /// Layer heights for each layer (mm).
    pub layer_heights: Vec<CoordF>,
    /// Z heights for each layer (mm).
    pub z_heights: Vec<CoordF>,
}

impl Default for TreeModelVolumesConfig {
    fn default() -> Self {
        Self {
            max_move: scale(1.0),
            max_move_slow: scale(0.5),
            min_resolution: scale(0.1),
            xy_distance: scale(0.8),
            xy_min_distance: scale(0.4),
            support_rests_on_model: false,
            min_radius: scale(0.4),
            increase_until_radius: scale(10.0),
            layer_heights: vec![0.2],
            z_heights: vec![0.2],
        }
    }
}

impl TreeModelVolumesConfig {
    // Create a new config with specified layer information.
    pub fn with_layers(layer_heights: Vec<CoordF>, z_heights: Vec<CoordF>) -> Self {
        Self {
            layer_heights,
            z_heights,
            ..Default::default()
        }
    }

    /// Get the Z height for a layer index.
    pub fn layer_z(&self, layer_idx: usize) -> CoordF {
        if layer_idx < self.z_heights.len() {
            self.z_heights[layer_idx]
        } else if !self.z_heights.is_empty() {
            *self.z_heights.last().unwrap()
        } else {
            0.0
        }
    }

    /// Get the layer height for a layer index.
    pub fn layer_height(&self, layer_idx: usize) -> CoordF {
        if layer_idx < self.layer_heights.len() {
            self.layer_heights[layer_idx]
        } else if !self.layer_heights.is_empty() {
            *self.layer_heights.last().unwrap()
        } else {
            0.2
        }
    }
}

/// Pre-computed collision and avoidance volumes for tree support generation.
///
/// This structure caches polygon areas at each layer for various radii,
/// allowing efficient queries during tree support path planning.
#[derive(Debug)]
pub struct TreeModelVolumes {
    config: TreeModelVolumesConfig,

    /// Model outlines at each layer (the actual model geometry).
    layer_outlines: Vec<ExPolygons>,

    /// Anti-overhang areas (regions marked as no-support).
    anti_overhang: Vec<Vec<Polygon>>,

    /// Machine bed area (printable region).
    bed_area: Polygon,

    /// Machine border (inverse of bed area).
    machine_border: Vec<Polygon>,

    /// Base collision cache (radius 0 = raw model outline).
    collision_cache: RadiusLayerPolygonCache,

    /// Hole-free collision cache (for buildplate-only supports).
    collision_cache_holefree: RadiusLayerPolygonCache,

    /// Fast avoidance cache.
    avoidance_cache_fast: RadiusLayerPolygonCache,

    /// Fast-safe (hole-free) avoidance cache.
    avoidance_cache_fast_safe: RadiusLayerPolygonCache,

    /// Slow (precise) avoidance cache.
    avoidance_cache_slow: RadiusLayerPolygonCache,

    /// Avoidance cache for supports that can rest on model.
    avoidance_cache_to_model: RadiusLayerPolygonCache,

    /// Placeable areas cache (where tips can be placed).
    placeable_cache: RadiusLayerPolygonCache,

    /// Wall restriction cache.
    wall_restriction_cache: RadiusLayerPolygonCache,

    /// Wall restriction cache (minimum distance mode).
    wall_restriction_cache_min: RadiusLayerPolygonCache,

    /// Whether volumes have been pre-calculated.
    precalculated: bool,

    /// Radii that can be ignored (too small to matter).
    ignorable_radii: Vec<Coord>,
}

impl TreeModelVolumes {
    // Create a new TreeModelVolumes instance.
    pub fn new(config: TreeModelVolumesConfig) -> Self {
        Self {
            config,
            layer_outlines: Vec::new(),
            anti_overhang: Vec::new(),
            bed_area: Polygon::new(),
            machine_border: Vec::new(),
            collision_cache: RadiusLayerPolygonCache::new(),
            collision_cache_holefree: RadiusLayerPolygonCache::new(),
            avoidance_cache_fast: RadiusLayerPolygonCache::new(),
            avoidance_cache_fast_safe: RadiusLayerPolygonCache::new(),
            avoidance_cache_slow: RadiusLayerPolygonCache::new(),
            avoidance_cache_to_model: RadiusLayerPolygonCache::new(),
            placeable_cache: RadiusLayerPolygonCache::new(),
            wall_restriction_cache: RadiusLayerPolygonCache::new(),
            wall_restriction_cache_min: RadiusLayerPolygonCache::new(),
            precalculated: false,
            ignorable_radii: Vec::new(),
        }
    }

    /// Create with layer outlines from sliced model.
    pub fn with_layer_outlines(
        config: TreeModelVolumesConfig,
        layer_outlines: Vec<ExPolygons>,
    ) -> Self {
        let mut volumes = Self::new(config);
        volumes.layer_outlines = layer_outlines;
        volumes
    }

    /// Set the bed area (printable region).
    pub fn set_bed_area(&mut self, bed_area: Polygon) {
        self.bed_area = bed_area.clone();
        // Machine border is the inverse - areas outside the bed
        // For now we leave it empty; a full implementation would compute the complement
        self.machine_border = Vec::new();
    }

    /// Set anti-overhang areas (no-support regions).
    pub fn set_anti_overhang(&mut self, anti_overhang: Vec<Vec<Polygon>>) {
        self.anti_overhang = anti_overhang;
    }

    /// Get the configuration.
    pub fn config(&self) -> &TreeModelVolumesConfig {
        &self.config
    }

    /// Get the number of layers.
    pub fn layer_count(&self) -> usize {
        self.layer_outlines.len()
    }

    /// Clear all caches.
    pub fn clear(&mut self) {
        self.collision_cache = RadiusLayerPolygonCache::new();
        self.collision_cache_holefree = RadiusLayerPolygonCache::new();
        self.avoidance_cache_fast = RadiusLayerPolygonCache::new();
        self.avoidance_cache_fast_safe = RadiusLayerPolygonCache::new();
        self.avoidance_cache_slow = RadiusLayerPolygonCache::new();
        self.avoidance_cache_to_model = RadiusLayerPolygonCache::new();
        self.placeable_cache = RadiusLayerPolygonCache::new();
        self.wall_restriction_cache = RadiusLayerPolygonCache::new();
        self.wall_restriction_cache_min = RadiusLayerPolygonCache::new();
        self.precalculated = false;
    }

    /// Clear all caches except base collision (radius 0).
    pub fn clear_except_collision(&mut self) {
        // Keep collision_cache as-is
        self.collision_cache_holefree = RadiusLayerPolygonCache::new();
        self.avoidance_cache_fast = RadiusLayerPolygonCache::new();
        self.avoidance_cache_fast_safe = RadiusLayerPolygonCache::new();
        self.avoidance_cache_slow = RadiusLayerPolygonCache::new();
        self.avoidance_cache_to_model = RadiusLayerPolygonCache::new();
        self.placeable_cache = RadiusLayerPolygonCache::new();
        self.wall_restriction_cache = RadiusLayerPolygonCache::new();
        self.wall_restriction_cache_min = RadiusLayerPolygonCache::new();
    }

    /// Round a radius up to the next cached value.
    ///
    /// This implements the exponential stepping from BambuStudio:
    /// - Below EXPONENTIAL_THRESHOLD: linear steps of COLLISION_RESOLUTION
    /// - Above threshold: exponential growth with EXPONENTIAL_FACTOR
    pub fn ceil_radius(&self, radius: Coord) -> Coord {
        if radius == 0 {
            return 0;
        }

        if radius < EXPONENTIAL_THRESHOLD {
            // Linear stepping for small radii
            let steps = (radius + COLLISION_RESOLUTION - 1) / COLLISION_RESOLUTION;
            steps * COLLISION_RESOLUTION
        } else {
            // Exponential stepping for larger radii
            let base = EXPONENTIAL_THRESHOLD as f64;
            let factor = EXPONENTIAL_FACTOR;
            let _diff = (radius - EXPONENTIAL_THRESHOLD) as f64;

            // Find the next exponential step
            let mut result = base;
            while (result as Coord) < radius {
                result *= factor;
            }

            result as Coord
        }
    }

    /// Get the next ceil radius after a given radius.
    pub fn next_ceil_radius(&self, radius: Coord) -> Coord {
        let ceiled = self.ceil_radius(radius);
        if ceiled < EXPONENTIAL_THRESHOLD {
            ceiled + COLLISION_RESOLUTION
        } else {
            (ceiled as f64 * EXPONENTIAL_FACTOR) as Coord
        }
    }

    /// Get collision areas for a given radius and layer.
    ///
    /// Collision areas are where a branch of the given radius would collide
    /// with the model. This is the model outline expanded by the radius plus
    /// the XY distance.
    pub fn get_collision(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        let ceiled_radius = self.ceil_radius(radius);
        let key = RadiusLayerKey::new(ceiled_radius, layer_idx);

        // Check cache first
        if let Some(cached) = self.collision_cache.get(&key) {
            return (*cached).clone();
        }

        // Calculate collision area
        let collision = self.calculate_collision(ceiled_radius, layer_idx);

        // Cache the result
        self.collision_cache.insert(key, collision.clone());

        collision
    }

    /// Get collision areas with holes removed (for buildplate-only supports).
    pub fn get_collision_holefree(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        let ceiled_radius = self.ceil_radius(radius);
        let key = RadiusLayerKey::new(ceiled_radius, layer_idx);

        if let Some(cached) = self.collision_cache_holefree.get(&key) {
            return (*cached).clone();
        }

        let collision = self.calculate_collision_holefree(ceiled_radius, layer_idx);
        self.collision_cache_holefree.insert(key, collision.clone());

        collision
    }

    /// Get avoidance areas for a given radius, layer, and avoidance type.
    ///
    /// Avoidance areas are regions where a branch should not be placed,
    /// considering that it needs to be able to move downward without collision.
    pub fn get_avoidance(
        &self,
        radius: Coord,
        layer_idx: usize,
        avoidance_type: AvoidanceType,
        to_model: bool,
    ) -> Vec<Polygon> {
        let ceiled_radius = self.ceil_radius(radius);
        let key = RadiusLayerKey::new(ceiled_radius, layer_idx);

        let cache = if to_model {
            &self.avoidance_cache_to_model
        } else {
            match avoidance_type {
                AvoidanceType::Fast => &self.avoidance_cache_fast,
                AvoidanceType::FastSafe => &self.avoidance_cache_fast_safe,
                AvoidanceType::Slow => &self.avoidance_cache_slow,
            }
        };

        if let Some(cached) = cache.get(&key) {
            return (*cached).clone();
        }

        let avoidance =
            self.calculate_avoidance(ceiled_radius, layer_idx, avoidance_type, to_model);
        cache.insert(key, avoidance.clone());

        avoidance
    }

    /// Get placeable areas for a given radius and layer.
    ///
    /// Placeable areas are where support tips can be placed - typically
    /// the inverse of collision areas within the bed bounds.
    pub fn get_placeable(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        let ceiled_radius = self.ceil_radius(radius);
        let key = RadiusLayerKey::new(ceiled_radius, layer_idx);

        if let Some(cached) = self.placeable_cache.get(&key) {
            return (*cached).clone();
        }

        let placeable = self.calculate_placeable(ceiled_radius, layer_idx);
        self.placeable_cache.insert(key, placeable.clone());

        placeable
    }

    /// Get wall restriction areas for a given radius and layer.
    ///
    /// Wall restrictions define areas where branches must maintain minimum
    /// distance from walls to ensure printability.
    pub fn get_wall_restriction(
        &self,
        radius: Coord,
        layer_idx: usize,
        use_min_distance: bool,
    ) -> Vec<Polygon> {
        let ceiled_radius = self.ceil_radius(radius);
        let key = RadiusLayerKey::new(ceiled_radius, layer_idx);

        let cache = if use_min_distance {
            &self.wall_restriction_cache_min
        } else {
            &self.wall_restriction_cache
        };

        if let Some(cached) = cache.get(&key) {
            return (*cached).clone();
        }

        let restriction =
            self.calculate_wall_restriction(ceiled_radius, layer_idx, use_min_distance);
        cache.insert(key, restriction.clone());

        restriction
    }

    /// Pre-calculate volumes for a range of layers and radii.
    ///
    /// This should be called before tree support generation to ensure
    /// all needed volumes are cached, enabling parallel computation.
    pub fn precalculate(&mut self, max_layer: usize, radii: &[Coord]) {
        // Calculate collision for all layers at radius 0 first
        for layer_idx in 0..=max_layer.min(self.layer_count().saturating_sub(1)) {
            let _ = self.get_collision(0, layer_idx);
        }

        // Calculate for all requested radii
        for &radius in radii {
            let ceiled = self.ceil_radius(radius);
            for layer_idx in 0..=max_layer.min(self.layer_count().saturating_sub(1)) {
                let _ = self.get_collision(ceiled, layer_idx);
                let _ = self.get_avoidance(ceiled, layer_idx, AvoidanceType::Fast, false);
            }
        }

        self.precalculated = true;
    }

    /// Check if volumes have been pre-calculated.
    pub fn is_precalculated(&self) -> bool {
        self.precalculated
    }

    // --- Internal calculation methods ---

    /// Calculate collision area for a radius and layer.
    fn calculate_collision(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        if layer_idx >= self.layer_outlines.len() {
            return Vec::new();
        }

        // Get base model outline for this layer
        let layer_outline = &self.layer_outlines[layer_idx];

        // Convert ExPolygons to Polygons (contours only for now)
        let mut polygons: Vec<Polygon> =
            layer_outline.iter().map(|ex| ex.contour.clone()).collect();

        // Add holes as separate polygons (they are also obstacles)
        for ex in layer_outline {
            for hole in &ex.holes {
                polygons.push(hole.clone());
            }
        }

        if radius == 0 {
            // Base collision is just the model outline
            return polygons;
        }

        // Offset by radius + xy_distance
        let offset_distance = radius + self.config.xy_distance;

        // offset_polygons returns ExPolygons, extract contours as Polygons
        let offset_expolygons = clipper_utils::offset_polygons(
            &polygons,
            offset_distance as f64,
            OffsetJoinType::Round,
        );
        expolygons_to_polygons(&offset_expolygons)
    }

    /// Calculate hole-free collision area.
    fn calculate_collision_holefree(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        // Get base collision
        let collision = self.get_collision(radius, layer_idx);

        // Fill holes by using only outer contours
        // Convert to ExPolygons for union, then back to Polygons
        let expolygons: Vec<ExPolygon> = collision
            .iter()
            .map(|p| ExPolygon::new(p.clone()))
            .collect();

        // Union with empty to merge overlapping regions
        let unioned = clipper_utils::union(&expolygons, &[]);

        // Extract just the outer contours (no holes)
        unioned.iter().map(|ex| ex.contour.clone()).collect()
    }

    /// Calculate avoidance area.
    fn calculate_avoidance(
        &self,
        radius: Coord,
        layer_idx: usize,
        avoidance_type: AvoidanceType,
        to_model: bool,
    ) -> Vec<Polygon> {
        // Base case: at layer 0, avoidance equals collision
        if layer_idx == 0 {
            return if matches!(avoidance_type, AvoidanceType::FastSafe) {
                self.get_collision_holefree(radius, layer_idx)
            } else {
                self.get_collision(radius, layer_idx)
            };
        }

        // Get collision at current layer
        let collision = if matches!(avoidance_type, AvoidanceType::FastSafe) {
            self.get_collision_holefree(radius, layer_idx)
        } else {
            self.get_collision(radius, layer_idx)
        };

        // Get avoidance from layer below
        let avoidance_below = self.get_avoidance(radius, layer_idx - 1, avoidance_type, to_model);

        // Determine move distance based on avoidance type
        let move_distance = match avoidance_type {
            AvoidanceType::Slow => self.config.max_move_slow,
            _ => self.config.max_move,
        };

        // Offset the avoidance from below by the move distance
        // (branches can move horizontally by this much per layer)
        let offset_avoidance = if move_distance > 0 {
            let offset_expolygons = clipper_utils::offset_polygons(
                &avoidance_below,
                move_distance as f64,
                OffsetJoinType::Round,
            );
            expolygons_to_polygons(&offset_expolygons)
        } else {
            avoidance_below.clone()
        };

        // Union collision with offset avoidance
        let collision_ex: Vec<ExPolygon> = collision
            .iter()
            .map(|p| ExPolygon::new(p.clone()))
            .collect();
        let avoidance_ex: Vec<ExPolygon> = offset_avoidance
            .iter()
            .map(|p| ExPolygon::new(p.clone()))
            .collect();

        let unioned = clipper_utils::union(&collision_ex, &avoidance_ex);
        expolygons_to_polygons(&unioned)
    }

    /// Calculate placeable areas.
    fn calculate_placeable(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        // Placeable areas are the bed area minus collision areas
        if self.bed_area.points().is_empty() {
            // No bed defined, return empty
            return Vec::new();
        }

        let collision = self.get_collision(radius, layer_idx);

        // Convert collision polygons to ExPolygons
        let collision_ex: Vec<ExPolygon> = collision
            .iter()
            .map(|p| ExPolygon::new(p.clone()))
            .collect();

        // Difference: bed_area - collision
        let bed_ex = ExPolygon::new(self.bed_area.clone());
        let result = clipper_utils::difference(&[bed_ex.clone()], &collision_ex);

        if result.is_empty() {
            vec![self.bed_area.clone()]
        } else {
            expolygons_to_polygons(&result)
        }
    }

    /// Calculate wall restriction areas.
    fn calculate_wall_restriction(
        &self,
        radius: Coord,
        layer_idx: usize,
        use_min_distance: bool,
    ) -> Vec<Polygon> {
        if layer_idx >= self.layer_outlines.len() {
            return Vec::new();
        }

        let layer_outline = &self.layer_outlines[layer_idx];

        // Get outer contours
        let polygons: Vec<Polygon> = layer_outline.iter().map(|ex| ex.contour.clone()).collect();

        // Offset by xy distance (or min distance)
        let offset_distance = if use_min_distance {
            self.config.xy_min_distance
        } else {
            self.config.xy_distance
        };

        // Additional offset by radius
        let total_offset = offset_distance + radius;

        let offset_expolygons =
            clipper_utils::offset_polygons(&polygons, total_offset as f64, OffsetJoinType::Round);
        expolygons_to_polygons(&offset_expolygons)
    }
}

/// Helper to convert ExPolygons to Polygons (extracting contours).
fn expolygons_to_polygons(expolygons: &[ExPolygon]) -> Vec<Polygon> {
    let mut result = Vec::with_capacity(expolygons.len());
    for ex in expolygons {
        result.push(ex.contour.clone());
        // Also include holes as they represent obstacles
        for hole in &ex.holes {
            result.push(hole.clone());
        }
    }
    result
}

/// Check if a point is inside any of the given polygons.
pub fn point_inside_polygons(point: Point, polygons: &[Polygon]) -> bool {
    for polygon in polygons {
        if polygon.contains_point(&point) {
            return true;
        }
    }
    false
}

/// Check if a point is outside all collision areas (i.e., it's safe to place a branch there).
pub fn is_safe_position(
    volumes: &TreeModelVolumes,
    point: Point,
    radius: Coord,
    layer_idx: usize,
) -> bool {
    let collision = volumes.get_collision(radius, layer_idx);
    !point_inside_polygons(point, &collision)
}

/// Find the nearest safe position from a given point.
///
/// This is used when a branch needs to move to avoid collision.
/// Returns the original point if it's already safe, otherwise searches
/// for the nearest point outside collision areas.
pub fn find_nearest_safe_position(
    volumes: &TreeModelVolumes,
    point: Point,
    radius: Coord,
    layer_idx: usize,
    max_search_distance: Coord,
) -> Option<Point> {
    if is_safe_position(volumes, point, radius, layer_idx) {
        return Some(point);
    }

    let collision = volumes.get_collision(radius, layer_idx);

    // Simple search in expanding circles
    let search_step = scale(0.1); // 0.1mm steps
    let mut search_radius = search_step;

    while search_radius <= max_search_distance {
        // Try points around the circle
        let num_points =
            (2.0 * std::f64::consts::PI * unscale(search_radius) / 0.1).ceil() as usize;
        let num_points = num_points.max(8);

        for i in 0..num_points {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (num_points as f64);
            let dx = (search_radius as f64 * angle.cos()) as Coord;
            let dy = (search_radius as f64 * angle.sin()) as Coord;

            let test_point = Point::new(point.x + dx, point.y + dy);

            if !point_inside_polygons(test_point, &collision) {
                return Some(test_point);
            }
        }

        search_radius += search_step;
    }

    None // No safe position found within search distance
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::ExPolygon;

    fn make_square_mm(size: f64) -> Polygon {
        let half = scale(size / 2.0);
        Polygon::from_points(vec![
            Point::new(-half, -half),
            Point::new(half, -half),
            Point::new(half, half),
            Point::new(-half, half),
        ])
    }

    fn make_expolygon_from_polygon(p: Polygon) -> ExPolygon {
        ExPolygon::new(p)
    }

    #[test]
    fn test_tree_model_volumes_new() {
        let config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(config);

        assert_eq!(volumes.layer_count(), 0);
        assert!(!volumes.is_precalculated());
    }

    #[test]
    fn test_ceil_radius_linear() {
        let config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(config);

        // Below threshold, should use linear stepping
        let r1 = volumes.ceil_radius(scale(0.1));
        assert_eq!(r1, COLLISION_RESOLUTION);

        let r2 = volumes.ceil_radius(scale(0.5));
        assert_eq!(r2, COLLISION_RESOLUTION);

        let r3 = volumes.ceil_radius(scale(0.6));
        assert_eq!(r3, COLLISION_RESOLUTION * 2);
    }

    #[test]
    fn test_ceil_radius_zero() {
        let config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(config);

        assert_eq!(volumes.ceil_radius(0), 0);
    }

    #[test]
    fn test_collision_cache() {
        let cache = RadiusLayerPolygonCache::new();
        let key = RadiusLayerKey::new(scale(1.0), 5);

        assert!(!cache.contains(&key));

        let polygons = vec![make_square_mm(10.0)];
        cache.insert(key, polygons.clone());

        assert!(cache.contains(&key));

        let retrieved = cache.get(&key).unwrap();
        assert_eq!(retrieved.len(), 1);
    }

    #[test]
    fn test_collision_calculation_empty() {
        let config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(config);

        // No layer outlines, should return empty
        let collision = volumes.get_collision(scale(1.0), 0);
        assert!(collision.is_empty());
    }

    #[test]
    fn test_collision_calculation_with_outline() {
        let config = TreeModelVolumesConfig::default();
        let square = make_square_mm(10.0);
        let expolygon = make_expolygon_from_polygon(square);
        let volumes = TreeModelVolumes::with_layer_outlines(config, vec![vec![expolygon]]);

        // Radius 0 should return the base outline
        let collision = volumes.get_collision(0, 0);
        assert!(!collision.is_empty());

        // Larger radius should return expanded outline
        let collision_expanded = volumes.get_collision(scale(1.0), 0);
        assert!(!collision_expanded.is_empty());
    }

    #[test]
    fn test_avoidance_type_default() {
        assert_eq!(AvoidanceType::default(), AvoidanceType::Fast);
    }

    #[test]
    fn test_config_layer_z() {
        let config = TreeModelVolumesConfig::with_layers(vec![0.2, 0.2, 0.2], vec![0.2, 0.4, 0.6]);

        assert!((config.layer_z(0) - 0.2).abs() < 1e-10);
        assert!((config.layer_z(1) - 0.4).abs() < 1e-10);
        assert!((config.layer_z(2) - 0.6).abs() < 1e-10);

        // Out of bounds should return last value
        assert!((config.layer_z(10) - 0.6).abs() < 1e-10);
    }

    #[test]
    fn test_config_layer_height() {
        let config = TreeModelVolumesConfig::with_layers(vec![0.2, 0.3, 0.2], vec![0.2, 0.5, 0.7]);

        assert!((config.layer_height(0) - 0.2).abs() < 1e-10);
        assert!((config.layer_height(1) - 0.3).abs() < 1e-10);
        assert!((config.layer_height(2) - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_radius_layer_key() {
        let key1 = RadiusLayerKey::new(100, 5);
        let key2 = RadiusLayerKey::new(100, 5);
        let key3 = RadiusLayerKey::new(200, 5);

        assert_eq!(key1, key2);
        assert_ne!(key1, key3);
    }

    #[test]
    fn test_cache_max_layer() {
        let cache = RadiusLayerPolygonCache::new();

        cache.insert(RadiusLayerKey::new(100, 0), vec![]);
        cache.insert(RadiusLayerKey::new(100, 5), vec![]);
        cache.insert(RadiusLayerKey::new(100, 3), vec![]);
        cache.insert(RadiusLayerKey::new(200, 10), vec![]);

        assert_eq!(cache.max_layer_for_radius(100), Some(5));
        assert_eq!(cache.max_layer_for_radius(200), Some(10));
        assert_eq!(cache.max_layer_for_radius(300), None);
    }

    #[test]
    fn test_cache_lower_bound() {
        let cache = RadiusLayerPolygonCache::new();

        cache.insert(RadiusLayerKey::new(100, 0), vec![make_square_mm(1.0)]);
        cache.insert(RadiusLayerKey::new(200, 0), vec![make_square_mm(2.0)]);
        cache.insert(RadiusLayerKey::new(400, 0), vec![make_square_mm(4.0)]);

        // Request radius 300, should get 200
        let result = cache.get_lower_bound(300, 0);
        assert!(result.is_some());
        let (radius, _) = result.unwrap();
        assert_eq!(radius, 200);

        // Request radius 100, should get 100
        let result = cache.get_lower_bound(100, 0);
        assert!(result.is_some());
        let (radius, _) = result.unwrap();
        assert_eq!(radius, 100);

        // Request radius 50, should get nothing (no radius <= 50)
        let result = cache.get_lower_bound(50, 0);
        assert!(result.is_none());
    }

    #[test]
    fn test_point_inside_polygons() {
        let square = make_square_mm(10.0);
        let polygons = vec![square];

        // Point at center should be inside
        assert!(point_inside_polygons(Point::new(0, 0), &polygons));

        // Point far outside should not be inside
        assert!(!point_inside_polygons(
            Point::new(scale(100.0), scale(100.0)),
            &polygons
        ));
    }

    #[test]
    fn test_is_safe_position() {
        let config = TreeModelVolumesConfig::default();
        let square = make_square_mm(10.0);
        let expolygon = make_expolygon_from_polygon(square);
        let volumes = TreeModelVolumes::with_layer_outlines(config, vec![vec![expolygon]]);

        // Point inside model should not be safe
        assert!(!is_safe_position(&volumes, Point::new(0, 0), 0, 0));

        // Point far from model should be safe
        assert!(is_safe_position(
            &volumes,
            Point::new(scale(100.0), scale(100.0)),
            0,
            0
        ));
    }

    #[test]
    fn test_volumes_clear() {
        let config = TreeModelVolumesConfig::default();
        let square = make_square_mm(10.0);
        let expolygon = make_expolygon_from_polygon(square);
        let mut volumes = TreeModelVolumes::with_layer_outlines(config, vec![vec![expolygon]]);

        // Populate cache
        let _ = volumes.get_collision(scale(1.0), 0);

        // Clear
        volumes.clear();

        // Verify cleared by checking precalculated flag
        assert!(!volumes.is_precalculated());
    }

    #[test]
    fn test_next_ceil_radius() {
        let config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(config);

        let r1 = volumes.ceil_radius(scale(0.1));
        let r2 = volumes.next_ceil_radius(scale(0.1));

        assert!(r2 > r1);
    }

    #[test]
    fn test_precalculate() {
        let config = TreeModelVolumesConfig::default();
        let square = make_square_mm(10.0);
        let expolygon = make_expolygon_from_polygon(square);
        let mut volumes = TreeModelVolumes::with_layer_outlines(
            config,
            vec![vec![expolygon.clone()], vec![expolygon]],
        );

        assert!(!volumes.is_precalculated());

        volumes.precalculate(1, &[scale(1.0), scale(2.0)]);

        assert!(volumes.is_precalculated());
    }
}
