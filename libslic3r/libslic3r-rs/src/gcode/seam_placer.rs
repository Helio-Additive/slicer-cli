//! Seam Placer - Intelligent seam placement for perimeter loops.
//!
//! This module provides advanced seam placement algorithms that determine
//! optimal starting points for perimeter extrusion loops. It mirrors
//! BambuStudio's `GCode/SeamPlacer.cpp` implementation.
//!
//! # Features
//!
//! - **Visibility-based scoring**: Prefers seam positions that are less visible
//! - **Overhang avoidance**: Avoids placing seams on overhanging regions
//! - **Corner preference**: Hides seams in concave corners when possible
//! - **Layer alignment**: Aligns seams across layers for cleaner vertical seam lines
//! - **Enforcer/Blocker support**: Respects user-painted seam preferences
//!
//! # Seam Position Modes
//!
//! - `Aligned`: Tries to align seams vertically across layers (default)
//! - `Nearest`: Places seam nearest to previous position
//! - `Random`: Randomizes seam position for less visible seam line
//! - `Rear`: Places seam at rear of print (highest Y)
//! - `Hidden`: Actively seeks concave corners to hide seams
//!
//! # Algorithm Overview
//!
//! 1. **Candidate Generation**: For each perimeter loop, generate seam candidates
//!    at each vertex plus oversampled points along edges near enforcers.
//!
//! 2. **Visibility Calculation**: Optionally raycast from mesh surface to determine
//!    visibility of each candidate point.
//!
//! 3. **Overhang Detection**: Calculate overhang amount at each candidate using
//!    layer-to-layer comparison.
//!
//! 4. **Embedded Distance**: Calculate distance inside merged regions to prefer
//!    hidden points (e.g., multi-material joins).
//!
//! 5. **Seam Selection**: Score candidates and select optimal seam point.
//!
//! 6. **Alignment**: Optionally align seams across layers using spatial queries.
//!
//! # BambuStudio Reference
//!
//! - `GCode/SeamPlacer.hpp` - Header with data structures
//! - `GCode/SeamPlacer.cpp` - Implementation (~1500 lines)
//!
//! Key structures from C++:
//! - `SeamCandidate` - Per-vertex seam candidate with attributes
//! - `Perimeter` - Metadata for a perimeter loop
//! - `SeamComparator` - Comparison logic for candidate scoring
//! - `GlobalModelInfo` - Mesh-wide visibility data

use crate::edge_grid::EdgeGrid;
use crate::geometry::{Point, PointF, Polygon};
use crate::{scale, unscale, Coord};
use std::f64::consts::PI;

// ============================================================================
// Configuration
// ============================================================================

/// Seam position preference mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SeamPositionMode {
    /// Align seams vertically across layers.
    #[default]
    Aligned,
    /// Place seam nearest to previous/current position.
    Nearest,
    /// Randomize seam position (deterministically based on geometry).
    Random,
    /// Place seam at rear of print (highest Y coordinate).
    Rear,
    /// Actively hide seams in concave corners.
    Hidden,
}

/// Whether a seam point is enforced, blocked, or neutral.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum EnforcedBlockedSeamPoint {
    /// User has blocked this point from being a seam.
    Blocked = 0,
    /// Neutral - no user preference.
    #[default]
    Neutral = 1,
    /// User has enforced this point as a preferred seam location.
    Enforced = 2,
}

/// Configuration for the seam placer.
#[derive(Clone, Debug)]
pub struct SeamPlacerConfig {
    /// Seam position preference mode.
    pub seam_position: SeamPositionMode,

    /// Minimum arm length for angle calculations (mm).
    pub min_arm_length: f64,

    /// Sharp angle threshold for snapping (radians).
    pub sharp_angle_threshold: f64,

    /// Overhang angle threshold (radians from vertical).
    pub overhang_angle_threshold: f64,

    /// Importance of angle in scoring (relative to visibility).
    pub angle_importance: f64,

    /// Distance for oversampling near enforcers (mm).
    pub enforcer_oversampling_distance: f64,

    /// Score tolerance for seam alignment.
    pub seam_align_score_tolerance: f64,

    /// Distance factor for alignment search (multiplied by flow width).
    pub seam_align_tolerable_dist_factor: f64,

    /// Minimum seams needed for alignment string.
    pub seam_align_minimum_string_seams: usize,

    /// Whether to avoid placing seams on overhangs.
    pub avoid_overhangs: bool,

    /// Whether to prefer hidden (embedded) points.
    pub prefer_hidden_points: bool,

    /// Embedded distance threshold for "hidden" classification (mm).
    pub embedded_distance_threshold: f64,
}

impl Default for SeamPlacerConfig {
    fn default() -> Self {
        Self {
            seam_position: SeamPositionMode::Aligned,
            min_arm_length: 0.5,
            sharp_angle_threshold: 55.0 * PI / 180.0,
            overhang_angle_threshold: 45.0 * PI / 180.0,
            angle_importance: 0.6,
            enforcer_oversampling_distance: 0.2,
            seam_align_score_tolerance: 0.3,
            seam_align_tolerable_dist_factor: 4.0,
            seam_align_minimum_string_seams: 6,
            avoid_overhangs: true,
            prefer_hidden_points: true,
            embedded_distance_threshold: 0.5,
        }
    }
}

impl SeamPlacerConfig {
    // Create configuration for nearest seam mode.
    pub fn nearest() -> Self {
        Self {
            seam_position: SeamPositionMode::Nearest,
            angle_importance: 1.0, // Higher for nearest mode
            ..Default::default()
        }
    }

    /// Create configuration for aligned seam mode.
    pub fn aligned() -> Self {
        Self {
            seam_position: SeamPositionMode::Aligned,
            ..Default::default()
        }
    }

    /// Create configuration for random seam mode.
    pub fn random() -> Self {
        Self {
            seam_position: SeamPositionMode::Random,
            ..Default::default()
        }
    }

    /// Create configuration for rear seam mode.
    pub fn rear() -> Self {
        Self {
            seam_position: SeamPositionMode::Rear,
            ..Default::default()
        }
    }

    /// Create configuration for hidden seam mode.
    pub fn hidden() -> Self {
        Self {
            seam_position: SeamPositionMode::Hidden,
            angle_importance: 1.2, // Emphasize corners
            ..Default::default()
        }
    }
}

// ============================================================================
// Seam Candidate Data Structures
// ============================================================================

/// A perimeter loop with its seam metadata.
#[derive(Clone, Debug)]
pub struct Perimeter {
    /// Start index in the candidates vector.
    pub start_index: usize,
    /// End index (inclusive) in the candidates vector.
    pub end_index: usize,
    /// Selected seam index within the perimeter.
    pub seam_index: usize,
    /// Flow/extrusion width for this perimeter.
    pub flow_width: f64,
    /// Whether the seam position has been finalized.
    pub finalized: bool,
    /// Final seam position (may differ from candidate position).
    pub final_seam_position: Option<Point3f>,
    /// Whether this is an external perimeter.
    pub is_external: bool,
    /// Layer index this perimeter belongs to.
    pub layer_index: usize,
}

impl Perimeter {
    // Create a new perimeter.
    pub fn new(start_index: usize, end_index: usize, flow_width: f64) -> Self {
        Self {
            start_index,
            end_index,
            seam_index: start_index,
            flow_width,
            finalized: false,
            final_seam_position: None,
            is_external: true,
            layer_index: 0,
        }
    }

    /// Get the number of candidates in this perimeter.
    pub fn len(&self) -> usize {
        if self.end_index >= self.start_index {
            self.end_index - self.start_index + 1
        } else {
            0
        }
    }

    /// Check if the perimeter has no candidates.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// 3D floating-point position.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point3f {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3f {
    // Create a new 3D point.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Create from 2D point with Z coordinate.
    pub fn from_2d(p: Point, z: f64) -> Self {
        Self {
            x: unscale(p.x),
            y: unscale(p.y),
            z,
        }
    }

    /// Convert to 2D point (drops Z).
    pub fn to_2d(&self) -> PointF {
        PointF::new(self.x, self.y)
    }

    /// Convert to scaled 2D point.
    pub fn to_2d_scaled(&self) -> Point {
        Point::new_scale(self.x, self.y)
    }

    /// Calculate squared distance to another point.
    pub fn distance_squared(&self, other: &Point3f) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let dz = other.z - self.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Calculate distance to another point.
    pub fn distance(&self, other: &Point3f) -> f64 {
        self.distance_squared(other).sqrt()
    }

    /// Calculate 2D distance (ignoring Z).
    pub fn distance_2d(&self, other: &Point3f) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Get the XY components as a tuple.
    pub fn xy(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}

/// A candidate point for seam placement.
#[derive(Clone, Debug)]
pub struct SeamCandidate {
    /// 3D position of the candidate.
    pub position: Point3f,
    /// Index of the perimeter this candidate belongs to.
    pub perimeter_index: usize,
    /// Visibility score (0 = hidden, higher = more visible).
    pub visibility: f64,
    /// Overhang amount (0 = no overhang, higher = more overhang).
    pub overhang: f64,
    /// Distance inside merged regions (negative = inside print).
    pub embedded_distance: f64,
    /// Local counter-clockwise angle at this vertex.
    pub local_ccw_angle: f64,
    /// Enforced/blocked/neutral status.
    pub point_type: EnforcedBlockedSeamPoint,
    /// Whether this is the central point of an enforced segment.
    pub central_enforcer: bool,
    /// Whether this candidate is suitable for scarf seam.
    pub enable_scarf_seam: bool,
    /// Extra overhang penalty from surrounding points.
    pub extra_overhang: f64,
}

impl SeamCandidate {
    // Create a new seam candidate.
    pub fn new(position: Point3f, perimeter_index: usize, local_ccw_angle: f64) -> Self {
        Self {
            position,
            perimeter_index,
            visibility: 0.0,
            overhang: 0.0,
            embedded_distance: 0.0,
            local_ccw_angle,
            point_type: EnforcedBlockedSeamPoint::Neutral,
            central_enforcer: false,
            enable_scarf_seam: false,
            extra_overhang: 0.0,
        }
    }

    /// Create a candidate with enforced status.
    pub fn enforced(position: Point3f, perimeter_index: usize, local_ccw_angle: f64) -> Self {
        let mut candidate = Self::new(position, perimeter_index, local_ccw_angle);
        candidate.point_type = EnforcedBlockedSeamPoint::Enforced;
        candidate
    }

    /// Create a candidate with blocked status.
    pub fn blocked(position: Point3f, perimeter_index: usize, local_ccw_angle: f64) -> Self {
        let mut candidate = Self::new(position, perimeter_index, local_ccw_angle);
        candidate.point_type = EnforcedBlockedSeamPoint::Blocked;
        candidate
    }

    /// Check if this is a concave corner (good for hiding seams).
    pub fn is_concave(&self) -> bool {
        self.local_ccw_angle < 0.0
    }

    /// Check if this is a convex corner.
    pub fn is_convex(&self) -> bool {
        self.local_ccw_angle > 0.0
    }

    /// Get the penalty for this candidate's angle.
    pub fn angle_penalty(&self) -> f64 {
        compute_angle_penalty(self.local_ccw_angle)
    }
}

// ============================================================================
// Layer Seam Data
// ============================================================================

/// Seam data for a single layer.
#[derive(Clone, Debug, Default)]
pub struct LayerSeams {
    /// All perimeters in this layer.
    pub perimeters: Vec<Perimeter>,
    /// All seam candidates for all perimeters.
    pub candidates: Vec<SeamCandidate>,
    /// Z height of this layer.
    pub z_height: f64,
    /// Layer index.
    pub layer_index: usize,
}

impl LayerSeams {
    // Create a new empty layer seams structure.
    pub fn new(z_height: f64, layer_index: usize) -> Self {
        Self {
            perimeters: Vec::new(),
            candidates: Vec::new(),
            z_height,
            layer_index,
        }
    }

    /// Add a perimeter with its candidates.
    pub fn add_perimeter(
        &mut self,
        polygon: &Polygon,
        flow_width: f64,
        is_external: bool,
        config: &SeamPlacerConfig,
    ) {
        let start_index = self.candidates.len();

        // Generate candidates for each vertex
        let candidates =
            generate_candidates_for_polygon(polygon, self.z_height, self.perimeters.len(), config);

        if candidates.is_empty() {
            return;
        }

        let end_index = start_index + candidates.len() - 1;
        self.candidates.extend(candidates);

        let mut perimeter = Perimeter::new(start_index, end_index, flow_width);
        perimeter.is_external = is_external;
        perimeter.layer_index = self.layer_index;
        self.perimeters.push(perimeter);
    }

    /// Get the perimeter containing a candidate index.
    pub fn perimeter_for_candidate(&self, candidate_idx: usize) -> Option<&Perimeter> {
        self.perimeters
            .iter()
            .find(|p| candidate_idx >= p.start_index && candidate_idx <= p.end_index)
    }

    /// Get mutable perimeter containing a candidate index.
    pub fn perimeter_for_candidate_mut(&mut self, candidate_idx: usize) -> Option<&mut Perimeter> {
        self.perimeters
            .iter_mut()
            .find(|p| candidate_idx >= p.start_index && candidate_idx <= p.end_index)
    }
}

// ============================================================================
// Seam Placer
// ============================================================================

/// The main seam placer that computes optimal seam positions.
#[derive(Clone, Debug)]
pub struct SeamPlacer {
    /// Configuration.
    config: SeamPlacerConfig,
    /// Seam data per layer.
    layers: Vec<LayerSeams>,
    /// Whether initialization has been completed.
    initialized: bool,
}

impl SeamPlacer {
    // Create a new seam placer with the given configuration.
    pub fn new(config: SeamPlacerConfig) -> Self {
        Self {
            config,
            layers: Vec::new(),
            initialized: false,
        }
    }

    /// Create a seam placer with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(SeamPlacerConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &SeamPlacerConfig {
        &self.config
    }

    /// Set the seam position mode.
    pub fn set_seam_position(&mut self, mode: SeamPositionMode) {
        self.config.seam_position = mode;
    }

    /// Initialize the seam placer with layer outlines.
    ///
    /// This processes all layers to compute seam candidates and their attributes.
    pub fn init(&mut self, layer_outlines: &[LayerOutline]) {
        self.layers.clear();
        self.initialized = false;

        // Generate candidates for each layer
        for (layer_idx, outline) in layer_outlines.iter().enumerate() {
            let mut layer_seams = LayerSeams::new(outline.z_height, layer_idx);

            for perimeter in &outline.perimeters {
                layer_seams.add_perimeter(
                    &perimeter.polygon,
                    perimeter.flow_width,
                    perimeter.is_external,
                    &self.config,
                );
            }

            self.layers.push(layer_seams);
        }

        // Calculate overhangs by comparing layers
        self.calculate_overhangs();

        // Calculate embedded distances
        self.calculate_embedded_distances(layer_outlines);

        // Pick initial seam points
        self.pick_seam_points();

        // Optionally align seams across layers
        if self.config.seam_position == SeamPositionMode::Aligned {
            self.align_seam_points();
        }

        self.initialized = true;
    }

    /// Initialize with simple polygon layers (convenience method).
    pub fn init_simple(&mut self, layers: &[(f64, Vec<Polygon>)], flow_width: f64) {
        let outlines: Vec<_> = layers
            .iter()
            .map(|(z, polygons)| LayerOutline {
                z_height: *z,
                perimeters: polygons
                    .iter()
                    .map(|p| PerimeterOutline {
                        polygon: p.clone(),
                        flow_width,
                        is_external: true,
                    })
                    .collect(),
                layer_regions: Vec::new(),
            })
            .collect();

        self.init(&outlines);
    }

    /// Get the seam position for a perimeter loop.
    ///
    /// Returns the optimal seam point index within the polygon.
    pub fn get_seam_point(
        &self,
        layer_index: usize,
        polygon: &Polygon,
        current_pos: Option<Point>,
    ) -> usize {
        if !self.initialized || layer_index >= self.layers.len() {
            // Fall back to simple seam placement
            return self.fallback_seam_point(polygon, current_pos);
        }

        let layer = &self.layers[layer_index];

        // Find matching perimeter
        let polygon_center = polygon.centroid();
        let best_perimeter = layer.perimeters.iter().min_by_key(|p| {
            if p.start_index >= layer.candidates.len() {
                return i64::MAX;
            }
            let candidate = &layer.candidates[p.start_index];
            let center = candidate.position.to_2d_scaled();
            polygon_center.distance_squared(&center) as i64
        });

        match best_perimeter {
            Some(perimeter) => {
                // For nearest mode, recalculate based on current position
                if self.config.seam_position == SeamPositionMode::Nearest {
                    if let Some(pos) = current_pos {
                        return self.find_nearest_seam(layer, perimeter, pos);
                    }
                }

                // Return the pre-calculated seam index
                let relative_index = perimeter.seam_index - perimeter.start_index;
                relative_index.min(polygon.len().saturating_sub(1))
            }
            None => self.fallback_seam_point(polygon, current_pos),
        }
    }

    /// Get the 3D seam position for a perimeter.
    pub fn get_seam_position_3d(
        &self,
        layer_index: usize,
        perimeter_index: usize,
    ) -> Option<Point3f> {
        if layer_index >= self.layers.len() {
            return None;
        }

        let layer = &self.layers[layer_index];
        let perimeter = layer.perimeters.get(perimeter_index)?;

        if perimeter.finalized {
            perimeter.final_seam_position
        } else {
            layer
                .candidates
                .get(perimeter.seam_index)
                .map(|c| c.position)
        }
    }

    /// Get statistics about seam placement.
    pub fn stats(&self) -> SeamPlacerStats {
        let total_perimeters: usize = self.layers.iter().map(|l| l.perimeters.len()).sum();
        let total_candidates: usize = self.layers.iter().map(|l| l.candidates.len()).sum();
        let finalized_count = self
            .layers
            .iter()
            .flat_map(|l| l.perimeters.iter())
            .filter(|p| p.finalized)
            .count();

        SeamPlacerStats {
            layer_count: self.layers.len(),
            total_perimeters,
            total_candidates,
            finalized_count,
            seam_position_mode: self.config.seam_position,
        }
    }

    // ========================================================================
    // Private Methods
    // ========================================================================

    /// Calculate overhangs by comparing adjacent layers.
    fn calculate_overhangs(&mut self) {
        if self.layers.len() < 2 {
            return;
        }

        for layer_idx in 1..self.layers.len() {
            // Get previous layer's outline for comparison
            let prev_candidates: Vec<_> = self.layers[layer_idx - 1]
                .candidates
                .iter()
                .map(|c| c.position)
                .collect();

            if prev_candidates.is_empty() {
                continue;
            }

            // For each candidate in current layer, check if it's over previous layer
            let layer = &mut self.layers[layer_idx];
            for candidate in &mut layer.candidates {
                let pos = candidate.position;

                // Find nearest point in previous layer
                let min_dist = prev_candidates
                    .iter()
                    .map(|p| pos.distance_2d(p))
                    .fold(f64::INFINITY, f64::min);

                // If distance is greater than flow width, it's an overhang
                let flow_width = layer
                    .perimeters
                    .get(candidate.perimeter_index)
                    .map(|p| p.flow_width)
                    .unwrap_or(0.4);

                if min_dist > flow_width * 0.5 {
                    candidate.overhang = (min_dist / flow_width).min(1.0);
                }
            }
        }
    }

    /// Calculate embedded distances using edge grids.
    fn calculate_embedded_distances(&mut self, layer_outlines: &[LayerOutline]) {
        for (layer_idx, outline) in layer_outlines.iter().enumerate() {
            if layer_idx >= self.layers.len() || outline.layer_regions.is_empty() {
                continue;
            }

            // Build edge grid from layer regions
            let polygons: Vec<_> = outline.layer_regions.iter().cloned().collect();
            if polygons.is_empty() {
                continue;
            }

            let grid = EdgeGrid::from_polygons(&polygons, scale(0.5));

            // Calculate embedded distance for each candidate
            let layer = &mut self.layers[layer_idx];
            for candidate in &mut layer.candidates {
                let point = candidate.position.to_2d_scaled();

                // Use signed distance from edge grid
                // Negative = inside, Positive = outside
                let sdf = grid.signed_distance_bilinear(&point);
                if sdf < f32::MAX {
                    candidate.embedded_distance = unscale(sdf as i64);
                }
            }
        }
    }

    /// Pick seam points for all perimeters.
    fn pick_seam_points(&mut self) {
        let comparator =
            SeamComparator::new(self.config.seam_position, self.config.angle_importance);

        for layer in &mut self.layers {
            for perimeter in &mut layer.perimeters {
                if perimeter.is_empty() {
                    continue;
                }

                match self.config.seam_position {
                    SeamPositionMode::Random => {
                        pick_random_seam_point(&layer.candidates, perimeter, &comparator);
                    }
                    SeamPositionMode::Rear => {
                        pick_rear_seam_point(&layer.candidates, perimeter);
                    }
                    _ => {
                        pick_best_seam_point(&layer.candidates, perimeter, &comparator);
                    }
                }
            }
        }
    }

    /// Align seam points across layers.
    ///
    /// Ported from BambuStudio's `SeamPlacer::align_seam_points`. The algorithm:
    /// 1. Gathers all seam positions (layer_idx, candidate_idx) across the object.
    /// 2. Sorts them by quality (best first) using the SeamComparator.
    /// 3. Starting from the best unfinalized seam, searches upward and downward
    ///    through layers to build a "seam string" -- a cluster of nearby, quality-
    ///    acceptable seam points that can be aligned into a vertical seam line.
    /// 4. If the string is long enough (>= seam_align_minimum_string_seams), each
    ///    member's seam position is adjusted toward the string's weighted average
    ///    XY position, with sharp-angle points snapping more strongly to their
    ///    original positions.
    fn align_seam_points(&mut self) {
        if self.layers.len() < self.config.seam_align_minimum_string_seams {
            return;
        }

        let comparator =
            SeamComparator::new(self.config.seam_position, self.config.angle_importance);

        // Step 1: Gather all seam (layer_idx, candidate_idx) pairs.
        // For each perimeter, we record its currently chosen seam_index.
        let mut seams: Vec<(usize, usize)> = Vec::new();
        for layer_idx in 0..self.layers.len() {
            for perim in &self.layers[layer_idx].perimeters {
                seams.push((layer_idx, perim.seam_index));
            }
        }

        if seams.is_empty() {
            return;
        }

        // Step 2: Sort seams by quality (best first).
        // We need to clone candidate data for sorting since we can't borrow
        // self.layers immutably while also mutating later.
        seams.sort_by(|left, right| {
            let a = &self.layers[left.0].candidates[left.1];
            let b = &self.layers[right.0].candidates[right.1];
            if comparator.is_first_better(a, b) {
                std::cmp::Ordering::Less
            } else if comparator.is_first_better(b, a) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });

        // Step 3: For each seam (best first), find its seam string.
        let mut global_index = 0;
        while global_index < seams.len() {
            let (start_layer_idx, start_seam_idx) = seams[global_index];
            global_index += 1;

            // Skip already finalized perimeters
            let perim_finalized = self.layers[start_layer_idx]
                .candidates
                .get(start_seam_idx)
                .and_then(|c| {
                    self.layers[start_layer_idx]
                        .perimeters
                        .get(c.perimeter_index)
                })
                .map(|p| p.finalized)
                .unwrap_or(true);

            if perim_finalized {
                continue;
            }

            // Find the seam string starting from this point
            let mut seam_string =
                self.find_seam_string(start_layer_idx, start_seam_idx, &comparator);

            // Try alternative starting points within the string to find longer strings
            // (ported from C++: tries every 20th member as alternative start)
            let step_size = 1 + seam_string.len() / 20;
            let string_copy = seam_string.clone();
            for alt_start in (0..string_copy.len()).step_by(step_size) {
                let alt_layer = string_copy[alt_start].0;
                // Use that layer's perimeter's seam_index as the start
                if let Some(candidate) = self.layers[alt_layer]
                    .candidates
                    .get(string_copy[alt_start].1)
                {
                    let perim_seam_idx = self.layers[alt_layer]
                        .perimeters
                        .get(candidate.perimeter_index)
                        .map(|p| p.seam_index)
                        .unwrap_or(string_copy[alt_start].1);

                    let alt_string = self.find_seam_string(alt_layer, perim_seam_idx, &comparator);
                    if alt_string.len() > seam_string.len() {
                        seam_string = alt_string;
                    }
                }
            }

            if seam_string.len() < self.config.seam_align_minimum_string_seams {
                continue;
            }

            // Sort string by layer index
            seam_string.sort_by_key(|&(layer_idx, _)| layer_idx);

            // Retry current seam since the string may have consumed its perimeter
            // via an alternative path
            global_index -= 1;

            // Step 4: Align the string.
            // Compute weighted average XY position along the string.
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut total_weight = 0.0;

            for &(layer_idx, cand_idx) in &seam_string {
                if let Some(candidate) = self.layers[layer_idx].candidates.get(cand_idx) {
                    let w = 1.0 / (0.1 + compute_angle_penalty(candidate.local_ccw_angle));
                    sum_x += w * candidate.position.x;
                    sum_y += w * candidate.position.y;
                    total_weight += w;
                }
            }

            if total_weight <= 0.0 {
                continue;
            }

            let avg_x = sum_x / total_weight;
            let avg_y = sum_y / total_weight;

            // Apply alignment: interpolate between candidate position and average,
            // with sharp-angle points snapping more strongly to their original pos.
            for &(layer_idx, cand_idx) in &seam_string {
                let z = self.layers[layer_idx].z_height;
                let (final_pos, perim_idx) = {
                    let candidate = match self.layers[layer_idx].candidates.get(cand_idx) {
                        Some(c) => c,
                        None => continue,
                    };
                    // t controls how much to keep original vs fitted position.
                    // Sharp angles (|angle| >= threshold) snap strongly to original.
                    let t = (candidate.local_ccw_angle.abs() / self.config.sharp_angle_threshold)
                        .powi(3)
                        .min(1.0);

                    let fitted_x = avg_x;
                    let fitted_y = avg_y;

                    let final_x = t * candidate.position.x + (1.0 - t) * fitted_x;
                    let final_y = t * candidate.position.y + (1.0 - t) * fitted_y;

                    (Point3f::new(final_x, final_y, z), candidate.perimeter_index)
                };

                if let Some(perimeter) = self.layers[layer_idx].perimeters.get_mut(perim_idx) {
                    perimeter.seam_index = cand_idx;
                    perimeter.final_seam_position = Some(final_pos);
                    perimeter.finalized = true;
                }
            }
        }
    }

    /// Find a seam string starting from a given seam point, searching both
    /// upward and downward through layers.
    ///
    /// Ported from BambuStudio's `SeamPlacer::find_seam_string`.
    fn find_seam_string(
        &self,
        start_layer: usize,
        start_cand: usize,
        comparator: &SeamComparator,
    ) -> Vec<(usize, usize)> {
        let mut seam_string = vec![(start_layer, start_cand)];
        let max_dist = self.config.seam_align_tolerable_dist_factor
            * self.layers[start_layer]
                .candidates
                .get(start_cand)
                .and_then(|c| self.layers[start_layer].perimeters.get(c.perimeter_index))
                .map(|p| p.flow_width)
                .unwrap_or(0.4);

        // Search upward, then downward
        for direction in &[1i32, -1i32] {
            let mut prev_layer = start_layer;
            let mut prev_cand = start_cand;
            let mut next_layer = start_layer as i64 + *direction as i64;

            loop {
                if next_layer < 0 || next_layer >= self.layers.len() as i64 {
                    break;
                }
                let nl = next_layer as usize;

                let prev_pos = match self.layers[prev_layer].candidates.get(prev_cand) {
                    Some(c) => c.position,
                    None => break,
                };

                // Project position to the next layer's Z
                let projected = Point3f::new(prev_pos.x, prev_pos.y, self.layers[nl].z_height);

                match self.find_next_seam_in_layer(nl, &projected, max_dist, comparator) {
                    Some(found_cand) => {
                        seam_string.push((nl, found_cand));
                        prev_layer = nl;
                        prev_cand = found_cand;
                    }
                    None => break,
                }

                next_layer += *direction as i64;
            }
        }

        seam_string
    }

    /// Find the best seam candidate in a layer near a projected position.
    ///
    /// Ported from BambuStudio's `SeamPlacer::find_next_seam_in_layer`.
    /// Returns the candidate index if a suitable one is found.
    fn find_next_seam_in_layer(
        &self,
        layer_idx: usize,
        projected_position: &Point3f,
        max_distance: f64,
        comparator: &SeamComparator,
    ) -> Option<usize> {
        let layer = &self.layers[layer_idx];
        if layer.candidates.is_empty() {
            return None;
        }

        // Find all candidates within max_distance (2D) of the projected position
        let mut best_nearby_idx: Option<usize> = None;
        let mut nearest_idx: Option<usize> = None;
        let mut nearest_dist_sq = f64::INFINITY;

        for (idx, candidate) in layer.candidates.iter().enumerate() {
            let dist = candidate.position.distance_2d(projected_position);
            if dist > max_distance {
                continue;
            }

            // Check if this perimeter is already finalized
            let perim = match layer.perimeters.get(candidate.perimeter_index) {
                Some(p) => p,
                None => continue,
            };
            if perim.finalized {
                continue;
            }

            // Track nearest
            let dist_sq = dist * dist;
            if dist_sq < nearest_dist_sq {
                nearest_dist_sq = dist_sq;
                nearest_idx = Some(idx);
            }

            // Track best
            match best_nearby_idx {
                Some(bi) => {
                    if comparator.is_first_better(candidate, &layer.candidates[bi]) {
                        best_nearby_idx = Some(idx);
                    }
                }
                None => {
                    best_nearby_idx = Some(idx);
                }
            }
        }

        let nearest_idx = nearest_idx?;

        // Get the perimeter's current seam for this nearest point
        let nearest_perim_idx = layer.candidates[nearest_idx].perimeter_index;
        let next_layer_seam_idx = layer.perimeters[nearest_perim_idx].seam_index;

        // Check for central enforcer
        if let Some(seam_candidate) = layer.candidates.get(next_layer_seam_idx) {
            if seam_candidate.central_enforcer {
                let enforcer_dist = seam_candidate.position.distance_2d(projected_position);
                if enforcer_dist < 3.0 * max_distance {
                    return Some(next_layer_seam_idx);
                }
            }
        }

        // Try nearest first
        if let Some(seam_at) = layer.candidates.get(next_layer_seam_idx) {
            if comparator.is_first_not_much_worse(&layer.candidates[nearest_idx], seam_at) {
                return Some(nearest_idx);
            }
        }

        // Try best nearby
        if let Some(bi) = best_nearby_idx {
            if let Some(seam_at) = layer.candidates.get(next_layer_seam_idx) {
                if comparator.is_first_not_much_worse(&layer.candidates[bi], seam_at) {
                    return Some(bi);
                }
            }
        }

        None
    }

    /// Find nearest seam point for nearest mode.
    fn find_nearest_seam(
        &self,
        layer: &LayerSeams,
        perimeter: &Perimeter,
        current_pos: Point,
    ) -> usize {
        let current_pos_f = Point3f::from_2d(current_pos, layer.z_height);
        let _comparator =
            SeamComparator::new(SeamPositionMode::Nearest, self.config.angle_importance);

        let mut best_idx = perimeter.start_index;
        let mut best_score = f64::INFINITY;

        for idx in perimeter.start_index..=perimeter.end_index {
            if let Some(candidate) = layer.candidates.get(idx) {
                let distance = candidate.position.distance_2d(&current_pos_f);
                let angle_penalty = candidate.angle_penalty() * self.config.angle_importance;
                let score = distance + angle_penalty;

                // Also consider enforced/blocked status
                if candidate.point_type == EnforcedBlockedSeamPoint::Blocked {
                    continue;
                }
                if candidate.point_type == EnforcedBlockedSeamPoint::Enforced {
                    return idx - perimeter.start_index;
                }

                if score < best_score {
                    best_score = score;
                    best_idx = idx;
                }
            }
        }

        best_idx - perimeter.start_index
    }

    /// Fallback seam point selection when not initialized.
    fn fallback_seam_point(&self, polygon: &Polygon, current_pos: Option<Point>) -> usize {
        if polygon.is_empty() {
            return 0;
        }

        match self.config.seam_position {
            SeamPositionMode::Nearest => {
                if let Some(pos) = current_pos {
                    polygon
                        .points()
                        .iter()
                        .enumerate()
                        .min_by_key(|(_, p)| p.distance_squared(&pos) as i64)
                        .map(|(i, _)| i)
                        .unwrap_or(0)
                } else {
                    0
                }
            }
            SeamPositionMode::Rear => polygon
                .points()
                .iter()
                .enumerate()
                .max_by_key(|(_, p)| p.y)
                .map(|(i, _)| i)
                .unwrap_or(0),
            SeamPositionMode::Random => {
                // Deterministic "random" based on geometry
                let hash = polygon
                    .points()
                    .first()
                    .map(|p| ((p.x.wrapping_mul(73856093)) ^ (p.y.wrapping_mul(19349663))) as usize)
                    .unwrap_or(0);
                hash % polygon.len().max(1)
            }
            SeamPositionMode::Hidden | SeamPositionMode::Aligned => {
                // Find sharpest concave corner
                find_best_corner(polygon)
            }
        }
    }
}

impl Default for SeamPlacer {
    fn default() -> Self {
        Self::with_defaults()
    }
}

// ============================================================================
// Input Data Structures
// ============================================================================

/// A perimeter outline for seam placement.
#[derive(Clone, Debug)]
pub struct PerimeterOutline {
    /// The perimeter polygon.
    pub polygon: Polygon,
    /// Flow/extrusion width.
    pub flow_width: f64,
    /// Whether this is an external perimeter.
    pub is_external: bool,
}

/// Layer outline data for seam placement initialization.
#[derive(Clone, Debug)]
pub struct LayerOutline {
    /// Z height of this layer.
    pub z_height: f64,
    /// Perimeter loops at this layer.
    pub perimeters: Vec<PerimeterOutline>,
    /// Merged layer regions for embedded distance calculation.
    pub layer_regions: Vec<Polygon>,
}

// ============================================================================
// Statistics
// ============================================================================

/// Statistics about seam placement.
#[derive(Clone, Debug)]
pub struct SeamPlacerStats {
    /// Number of layers processed.
    pub layer_count: usize,
    /// Total number of perimeters.
    pub total_perimeters: usize,
    /// Total number of seam candidates.
    pub total_candidates: usize,
    /// Number of perimeters with finalized seams.
    pub finalized_count: usize,
    /// Seam position mode used.
    pub seam_position_mode: SeamPositionMode,
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Gaussian function for smooth falloff.
fn gauss(value: f64, mean_x: f64, mean_value: f64, falloff_speed: f64) -> f64 {
    let shifted = value - mean_x;
    let denominator = falloff_speed * shifted * shifted + 1.0;
    let exponent = 1.0 / denominator;
    mean_value * (exponent.exp() - 1.0) / (std::f64::consts::E - 1.0)
}

/// Compute angle penalty for seam placement.
///
/// Uses a combination of gaussian and sigmoid to penalize convex corners
/// more than concave ones.
fn compute_angle_penalty(ccw_angle: f64) -> f64 {
    // Gaussian + sigmoid combination
    // Concave angles (negative) have lower penalty
    // Convex angles (positive) have higher penalty
    gauss(ccw_angle, 0.0, 1.0, 3.0) + 1.0 / (2.0 + (-ccw_angle).exp())
}

/// Generate seam candidates for a polygon.
fn generate_candidates_for_polygon(
    polygon: &Polygon,
    z_height: f64,
    perimeter_index: usize,
    config: &SeamPlacerConfig,
) -> Vec<SeamCandidate> {
    let points = polygon.points();
    if points.len() < 3 {
        return Vec::new();
    }

    let n = points.len();
    let mut candidates = Vec::with_capacity(n);

    // Pre-calculate segment lengths
    let lengths: Vec<f64> = (0..n)
        .map(|i| {
            let next = (i + 1) % n;
            unscale(points[i].distance(&points[next]) as Coord)
        })
        .collect();

    // Calculate angles at each vertex
    let angles = calculate_polygon_angles(&points, &lengths, config.min_arm_length);

    for (i, &point) in points.iter().enumerate() {
        let pos = Point3f::from_2d(point, z_height);
        let angle = angles.get(i).copied().unwrap_or(0.0);
        candidates.push(SeamCandidate::new(pos, perimeter_index, angle));
    }

    candidates
}

/// Calculate angles at polygon vertices.
fn calculate_polygon_angles(points: &[Point], lengths: &[f64], min_arm_length: f64) -> Vec<f64> {
    let n = points.len();
    if n < 3 {
        return vec![0.0; n];
    }

    let mut result = vec![0.0; n];

    let mut idx_prev = 0;
    let mut idx_next = 0;
    let mut distance_to_prev = 0.0;
    let mut distance_to_next = 0.0;

    // Initialize prev index far enough back
    while distance_to_prev < min_arm_length {
        idx_prev = if idx_prev == 0 { n - 1 } else { idx_prev - 1 };
        distance_to_prev += lengths[idx_prev];
        if idx_prev == 0 && distance_to_prev < min_arm_length {
            break; // Polygon too small
        }
    }

    for idx_curr in 0..n {
        // Pull idx_prev to current as much as possible
        while distance_to_prev - lengths[idx_prev] > min_arm_length {
            distance_to_prev -= lengths[idx_prev];
            idx_prev = (idx_prev + 1) % n;
        }

        // Push idx_next forward as needed
        while distance_to_next < min_arm_length {
            distance_to_next += lengths[idx_next];
            idx_next = (idx_next + 1) % n;
        }

        // Calculate angle
        let p0 = &points[idx_prev];
        let p1 = &points[idx_curr];
        let p2 = &points[idx_next];

        result[idx_curr] =
            angle_between_vectors((p1.x - p0.x, p1.y - p0.y), (p2.x - p1.x, p2.y - p1.y));

        // Advance
        let curr_distance = lengths[idx_curr];
        distance_to_prev += curr_distance;
        distance_to_next -= curr_distance;
    }

    result
}

/// Calculate angle between two vectors (in radians).
fn angle_between_vectors(v1: (Coord, Coord), v2: (Coord, Coord)) -> f64 {
    let v1f = (v1.0 as f64, v1.1 as f64);
    let v2f = (v2.0 as f64, v2.1 as f64);

    let cross = v1f.0 * v2f.1 - v1f.1 * v2f.0;
    let dot = v1f.0 * v2f.0 + v1f.1 * v2f.1;

    cross.atan2(dot)
}

/// Find the best corner for seam placement in a polygon.
fn find_best_corner(polygon: &Polygon) -> usize {
    let points = polygon.points();
    if points.len() < 3 {
        return 0;
    }

    let n = points.len();
    let mut best_idx = 0;
    let mut best_score = f64::MAX;

    for i in 0..n {
        let prev = &points[(i + n - 1) % n];
        let curr = &points[i];
        let next = &points[(i + 1) % n];

        // Calculate cross product (negative = concave)
        let v1 = (curr.x - prev.x, curr.y - prev.y);
        let v2 = (next.x - curr.x, next.y - curr.y);
        let cross = (v1.0 as f64) * (v2.1 as f64) - (v1.1 as f64) * (v2.0 as f64);

        // Score: prefer concave corners (negative cross product)
        let score = if cross < 0.0 {
            cross.abs() * -1.0 // Negative score for concave
        } else {
            cross.abs() // Positive score for convex
        };

        if score < best_score {
            best_score = score;
            best_idx = i;
        }
    }

    best_idx
}

// ============================================================================
// Seam Comparator
// ============================================================================

/// Comparator for seam candidates.
struct SeamComparator {
    mode: SeamPositionMode,
    angle_importance: f64,
}

impl SeamComparator {
    fn new(mode: SeamPositionMode, angle_importance: f64) -> Self {
        Self {
            mode,
            angle_importance,
        }
    }

    /// Check if candidate `a` is better than candidate `b`.
    fn is_first_better(&self, a: &SeamCandidate, b: &SeamCandidate) -> bool {
        // Central enforcers have priority in aligned mode
        if self.mode == SeamPositionMode::Aligned && a.central_enforcer != b.central_enforcer {
            return a.central_enforcer;
        }

        // Enforced/blocked discrimination (top priority)
        if a.point_type != b.point_type {
            return a.point_type > b.point_type;
        }

        // Avoid overhangs
        if a.overhang > 0.0 || b.overhang > 0.0 {
            if (a.overhang - b.overhang).abs() > 0.1 {
                return a.overhang < b.overhang;
            }
        }

        // Prefer hidden points (embedded more than threshold inside)
        if a.embedded_distance < -0.5 && b.embedded_distance > -0.5 {
            return true;
        }
        if b.embedded_distance < -0.5 && a.embedded_distance > -0.5 {
            return false;
        }

        // Rear mode: prefer higher Y
        if self.mode == SeamPositionMode::Rear && (a.position.y - b.position.y).abs() > 0.01 {
            return a.position.y > b.position.y;
        }

        // Calculate penalties
        let penalty_a = a.overhang
            + a.visibility
            + self.angle_importance * compute_angle_penalty(a.local_ccw_angle)
            + a.extra_overhang;

        let penalty_b = b.overhang
            + b.visibility
            + self.angle_importance * compute_angle_penalty(b.local_ccw_angle)
            + b.extra_overhang;

        penalty_a < penalty_b
    }

    /// Check if candidate `a` is not much worse than candidate `b`.
    fn is_first_not_much_worse(&self, a: &SeamCandidate, b: &SeamCandidate) -> bool {
        // Central enforcers have priority in aligned mode
        if self.mode == SeamPositionMode::Aligned && a.central_enforcer != b.central_enforcer {
            return a.central_enforcer;
        }

        // Enforced always acceptable
        if a.point_type == EnforcedBlockedSeamPoint::Enforced {
            return true;
        }

        // Blocked never acceptable
        if a.point_type == EnforcedBlockedSeamPoint::Blocked {
            return false;
        }

        if a.point_type != b.point_type {
            return a.point_type > b.point_type;
        }

        // Avoid significant overhang differences
        if (a.overhang > 0.0 || b.overhang > 0.0) && (a.overhang - b.overhang).abs() > 0.1 {
            return a.overhang < b.overhang;
        }

        // Prefer hidden points
        if a.embedded_distance < -0.5 && b.embedded_distance > -0.5 {
            return true;
        }
        if b.embedded_distance < -0.5 && a.embedded_distance > -0.5 {
            return false;
        }

        // Random mode: always acceptable
        if self.mode == SeamPositionMode::Random {
            return true;
        }

        // Rear mode: slight tolerance
        if self.mode == SeamPositionMode::Rear {
            return a.position.y + 0.3 * 5.0 > b.position.y;
        }

        // Calculate penalties with tolerance
        let penalty_a = a.overhang
            + a.visibility
            + self.angle_importance * compute_angle_penalty(a.local_ccw_angle)
            + a.extra_overhang;

        let penalty_b = b.overhang
            + b.visibility
            + self.angle_importance * compute_angle_penalty(b.local_ccw_angle)
            + b.extra_overhang;

        penalty_a <= penalty_b || penalty_a - penalty_b < 0.3 // Score tolerance
    }

    /// Check if two candidates are similar in quality.
    fn are_similar(&self, a: &SeamCandidate, b: &SeamCandidate) -> bool {
        self.is_first_not_much_worse(a, b) && self.is_first_not_much_worse(b, a)
    }
}

/// Pick the best seam point for a perimeter.
fn pick_best_seam_point(
    candidates: &[SeamCandidate],
    perimeter: &mut Perimeter,
    comparator: &SeamComparator,
) {
    let mut best_idx = perimeter.start_index;

    for idx in perimeter.start_index..=perimeter.end_index {
        if let (Some(candidate), Some(best)) = (candidates.get(idx), candidates.get(best_idx)) {
            if comparator.is_first_better(candidate, best) {
                best_idx = idx;
            }
        }
    }

    perimeter.seam_index = best_idx;
}

/// Pick rear seam point (highest Y coordinate).
fn pick_rear_seam_point(candidates: &[SeamCandidate], perimeter: &mut Perimeter) {
    let mut best_idx = perimeter.start_index;
    let mut best_y = f64::NEG_INFINITY;

    for idx in perimeter.start_index..=perimeter.end_index {
        if let Some(candidate) = candidates.get(idx) {
            // Skip blocked points
            if candidate.point_type == EnforcedBlockedSeamPoint::Blocked {
                continue;
            }
            // Enforced points win
            if candidate.point_type == EnforcedBlockedSeamPoint::Enforced {
                perimeter.seam_index = idx;
                return;
            }

            if candidate.position.y > best_y {
                best_y = candidate.position.y;
                best_idx = idx;
            }
        }
    }

    perimeter.seam_index = best_idx;
}

/// Pick a random seam point (deterministically based on geometry).
fn pick_random_seam_point(
    candidates: &[SeamCandidate],
    perimeter: &mut Perimeter,
    comparator: &SeamComparator,
) {
    if perimeter.is_empty() {
        return;
    }

    // Collect viable candidates
    struct Viable {
        index: usize,
        edge_length: f64,
    }

    let mut viables: Vec<Viable> = Vec::new();
    let mut viable_example_index = perimeter.start_index;

    // Deterministic pseudo-random based on first point
    let seed_pos = candidates
        .get(perimeter.start_index)
        .map(|c| c.position)
        .unwrap_or_default();
    let rand = {
        let v = seed_pos.x * 12.9898 + seed_pos.y * 78.233 + seed_pos.z * 133.3333;
        let r = (v.sin() * 43758.5453).abs();
        r - r.floor()
    };

    for idx in perimeter.start_index..=perimeter.end_index {
        let candidate = match candidates.get(idx) {
            Some(c) => c,
            None => continue,
        };
        let example = match candidates.get(viable_example_index) {
            Some(c) => c,
            None => continue,
        };

        if comparator.are_similar(candidate, example) {
            // Calculate edge length to next point
            let next_idx = if idx == perimeter.end_index {
                perimeter.start_index
            } else {
                idx + 1
            };
            let next_pos = candidates
                .get(next_idx)
                .map(|c| c.position)
                .unwrap_or(candidate.position);
            let edge_length = candidate.position.distance(&next_pos);

            viables.push(Viable {
                index: idx,
                edge_length,
            });
        } else if comparator.is_first_not_much_worse(example, candidate) {
            // Current example is better, skip this candidate
        } else {
            // This candidate is better, restart
            viable_example_index = idx;
            viables.clear();

            let next_idx = if idx == perimeter.end_index {
                perimeter.start_index
            } else {
                idx + 1
            };
            let next_pos = candidates
                .get(next_idx)
                .map(|c| c.position)
                .unwrap_or(candidate.position);
            let edge_length = candidate.position.distance(&next_pos);

            viables.push(Viable {
                index: idx,
                edge_length,
            });
        }
    }

    if viables.is_empty() {
        perimeter.seam_index = perimeter.start_index;
        return;
    }

    // Pick random point based on edge lengths
    let total_len: f64 = viables.iter().map(|v| v.edge_length).sum();
    let mut picked_len = total_len * rand;

    let mut selected_idx = 0;
    for viable in &viables {
        if picked_len <= viable.edge_length {
            selected_idx = viable.index;
            break;
        }
        picked_len -= viable.edge_length;
        selected_idx = viable.index;
    }

    perimeter.seam_index = selected_idx;
    perimeter.finalized = true;

    // Calculate exact position along the edge
    if let Some(candidate) = candidates.get(selected_idx) {
        let next_idx = if selected_idx == perimeter.end_index {
            perimeter.start_index
        } else {
            selected_idx + 1
        };

        if let Some(next) = candidates.get(next_idx) {
            let edge = Point3f::new(
                next.position.x - candidate.position.x,
                next.position.y - candidate.position.y,
                next.position.z - candidate.position.z,
            );
            let edge_len = candidate.position.distance(&next.position);
            if edge_len > 0.0 {
                let t = (picked_len / edge_len).clamp(0.0, 1.0);
                perimeter.final_seam_position = Some(Point3f::new(
                    candidate.position.x + edge.x * t,
                    candidate.position.y + edge.y * t,
                    candidate.position.z + edge.z * t,
                ));
            } else {
                perimeter.final_seam_position = Some(candidate.position);
            }
        } else {
            perimeter.final_seam_position = Some(candidate.position);
        }
    }
}

// ============================================================================
// Convenience Functions
// ============================================================================

/// Place seam on a polygon using the specified mode.
pub fn place_seam(polygon: &Polygon, mode: SeamPositionMode, current_pos: Option<Point>) -> usize {
    let placer = SeamPlacer::new(SeamPlacerConfig {
        seam_position: mode,
        ..Default::default()
    });
    placer.fallback_seam_point(polygon, current_pos)
}

/// Find the best seam index for a single polygon using angle-based scoring.
///
/// This is the primary entry point for use from `perimeter_generator.rs`.
/// It computes per-vertex angles, scores each candidate using the
/// `angle_importance * angle_penalty` formula, biases toward rear (high Y),
/// and optionally aligns with a previous layer's seam position.
///
/// # Arguments
/// * `polygon` - The perimeter polygon (assumed CCW or will be treated as-is).
/// * `prev_seam_pos` - If provided, biases toward this XY position for
///   inter-layer alignment. Pass `None` for the first layer.
/// * `config` - Seam placer configuration. Use `&SeamPlacerConfig::default()`
///   for aligned mode with BambuStudio X1C defaults.
///
/// # Returns
/// The index of the best seam point within the polygon.
pub fn find_best_seam_index(
    polygon: &Polygon,
    prev_seam_pos: Option<Point>,
    config: &SeamPlacerConfig,
) -> usize {
    let points = polygon.points();
    if points.len() < 3 {
        return 0;
    }

    let n = points.len();

    // Compute segment lengths
    let lengths: Vec<f64> = (0..n)
        .map(|i| {
            let next = (i + 1) % n;
            unscale(points[i].distance(&points[next]) as Coord)
        })
        .collect();

    // Compute per-vertex angles
    let angles = calculate_polygon_angles(points, &lengths, config.min_arm_length);

    // Score each vertex. Lower penalty = better seam point.
    let mut best_idx = 0;
    let mut best_penalty = f64::INFINITY;

    // Compute polygon bounding box center-Y for rear bias normalization
    let max_y = points.iter().map(|p| p.y).max().unwrap_or(0);
    let min_y = points.iter().map(|p| p.y).min().unwrap_or(0);
    let y_range = (max_y - min_y) as f64;

    for i in 0..n {
        let angle = angles[i];
        let angle_penalty = config.angle_importance * compute_angle_penalty(angle);

        // Rear bias: prefer higher Y. Normalize to [0, 1] range and invert
        // so that high-Y points get low penalty.
        let rear_penalty = if y_range > 0.0 {
            1.0 - ((points[i].y - min_y) as f64 / y_range)
        } else {
            0.0
        };

        // Alignment bias: if we have a previous seam position, prefer points
        // close to it. This gives inter-layer seam alignment.
        let alignment_penalty = if let Some(prev_pos) = prev_seam_pos {
            let dx = unscale(points[i].x - prev_pos.x);
            let dy = unscale(points[i].y - prev_pos.y);
            let dist = (dx * dx + dy * dy).sqrt();
            // Use gaussian falloff: nearby points get low penalty
            1.0 - gauss(dist, 0.0, 1.0, 0.005)
        } else {
            0.0
        };

        // Combined penalty: angle is primary, rear and alignment are secondary
        let penalty = angle_penalty + 0.3 * rear_penalty + alignment_penalty;

        if penalty < best_penalty {
            best_penalty = penalty;
            best_idx = i;
        }
    }

    best_idx
}

/// Create a seam placer and initialize with layers.
pub fn create_seam_placer(
    layers: &[(f64, Vec<Polygon>)],
    flow_width: f64,
    mode: SeamPositionMode,
) -> SeamPlacer {
    let mut placer = SeamPlacer::new(SeamPlacerConfig {
        seam_position: mode,
        ..Default::default()
    });
    placer.init_simple(layers, flow_width);
    placer
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square(size: f64) -> Polygon {
        let s = scale(size);
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(s, 0),
            Point::new(s, s),
            Point::new(0, s),
        ])
    }

    fn make_concave_shape() -> Polygon {
        // L-shaped polygon with a concave corner
        let s = scale(10.0);
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(s, 0),
            Point::new(s, s / 2),
            Point::new(s / 2, s / 2), // Concave corner
            Point::new(s / 2, s),
            Point::new(0, s),
        ])
    }

    #[test]
    fn test_seam_placer_config_default() {
        let config = SeamPlacerConfig::default();
        assert_eq!(config.seam_position, SeamPositionMode::Aligned);
        assert!(config.angle_importance > 0.0);
    }

    #[test]
    fn test_seam_placer_config_modes() {
        assert_eq!(
            SeamPlacerConfig::nearest().seam_position,
            SeamPositionMode::Nearest
        );
        assert_eq!(
            SeamPlacerConfig::aligned().seam_position,
            SeamPositionMode::Aligned
        );
        assert_eq!(
            SeamPlacerConfig::random().seam_position,
            SeamPositionMode::Random
        );
        assert_eq!(
            SeamPlacerConfig::rear().seam_position,
            SeamPositionMode::Rear
        );
        assert_eq!(
            SeamPlacerConfig::hidden().seam_position,
            SeamPositionMode::Hidden
        );
    }

    #[test]
    fn test_point3f_operations() {
        let p1 = Point3f::new(1.0, 2.0, 3.0);
        let p2 = Point3f::new(4.0, 6.0, 3.0);

        assert!((p1.distance(&p2) - 5.0).abs() < 0.001);
        assert!((p1.distance_2d(&p2) - 5.0).abs() < 0.001);

        let p2d = p1.to_2d();
        assert!((p2d.x - 1.0).abs() < 0.001);
        assert!((p2d.y - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_seam_candidate_creation() {
        let candidate = SeamCandidate::new(Point3f::new(1.0, 2.0, 0.2), 0, -0.5);
        assert!(candidate.is_concave());
        assert!(!candidate.is_convex());
        assert_eq!(candidate.point_type, EnforcedBlockedSeamPoint::Neutral);

        let enforced = SeamCandidate::enforced(Point3f::new(1.0, 2.0, 0.2), 0, 0.5);
        assert_eq!(enforced.point_type, EnforcedBlockedSeamPoint::Enforced);

        let blocked = SeamCandidate::blocked(Point3f::new(1.0, 2.0, 0.2), 0, 0.5);
        assert_eq!(blocked.point_type, EnforcedBlockedSeamPoint::Blocked);
    }

    #[test]
    fn test_perimeter_creation() {
        let perimeter = Perimeter::new(0, 9, 0.4);
        assert_eq!(perimeter.len(), 10);
        assert!(!perimeter.is_empty());
        assert_eq!(perimeter.flow_width, 0.4);
        assert!(!perimeter.finalized);
    }

    #[test]
    fn test_layer_seams_add_perimeter() {
        let mut layer = LayerSeams::new(0.2, 0);
        let square = make_square(10.0);
        let config = SeamPlacerConfig::default();

        layer.add_perimeter(&square, 0.4, true, &config);

        assert_eq!(layer.perimeters.len(), 1);
        assert_eq!(layer.candidates.len(), 4);
    }

    #[test]
    fn test_compute_angle_penalty() {
        // The penalty function uses gaussian + sigmoid
        // Concave angles (negative) should have lower penalty than convex (positive)
        let concave_penalty = compute_angle_penalty(-0.5);
        let convex_penalty = compute_angle_penalty(0.5);
        let flat_penalty = compute_angle_penalty(0.0);

        // Concave should be better (lower penalty) than convex
        assert!(concave_penalty < convex_penalty);
        // All penalties should be positive
        assert!(concave_penalty > 0.0);
        assert!(convex_penalty > 0.0);
        assert!(flat_penalty > 0.0);
    }

    #[test]
    fn test_gauss_function() {
        // Peak at mean
        let peak = gauss(0.0, 0.0, 1.0, 1.0);
        let off_peak = gauss(1.0, 0.0, 1.0, 1.0);

        assert!(peak > off_peak);
        assert!(peak > 0.0);
    }

    #[test]
    fn test_seam_placer_fallback_nearest() {
        let placer = SeamPlacer::new(SeamPlacerConfig::nearest());
        let square = make_square(10.0);
        let current_pos = Point::new_scale(9.0, 9.0); // Near top-right

        let seam_idx = placer.fallback_seam_point(&square, Some(current_pos));
        // Should be vertex 2 (top-right corner at 10, 10)
        assert_eq!(seam_idx, 2);
    }

    #[test]
    fn test_seam_placer_fallback_rear() {
        let placer = SeamPlacer::new(SeamPlacerConfig::rear());
        let square = make_square(10.0);

        let seam_idx = placer.fallback_seam_point(&square, None);
        // Should be vertex 2 or 3 (highest Y = 10)
        assert!(seam_idx == 2 || seam_idx == 3);
    }

    #[test]
    fn test_seam_placer_fallback_hidden() {
        let placer = SeamPlacer::new(SeamPlacerConfig::hidden());
        let concave = make_concave_shape();

        let seam_idx = placer.fallback_seam_point(&concave, None);
        // Should prefer the concave corner (index 3)
        assert_eq!(seam_idx, 3);
    }

    #[test]
    fn test_seam_placer_init_simple() {
        let mut placer = SeamPlacer::with_defaults();
        let square = make_square(10.0);
        let layers = vec![
            (0.2, vec![square.clone()]),
            (0.4, vec![square.clone()]),
            (0.6, vec![square]),
        ];

        placer.init_simple(&layers, 0.4);

        let stats = placer.stats();
        assert_eq!(stats.layer_count, 3);
        assert_eq!(stats.total_perimeters, 3);
        assert!(stats.total_candidates >= 12); // At least 4 per layer
    }

    #[test]
    fn test_seam_placer_get_seam_point() {
        let mut placer = SeamPlacer::new(SeamPlacerConfig::nearest());
        let square = make_square(10.0);
        let layers = vec![(0.2, vec![square.clone()]), (0.4, vec![square.clone()])];

        placer.init_simple(&layers, 0.4);

        let current_pos = Point::new_scale(0.5, 0.5); // Near origin
        let seam_idx = placer.get_seam_point(0, &square, Some(current_pos));
        // Should be vertex 0 (at origin)
        assert_eq!(seam_idx, 0);
    }

    #[test]
    fn test_seam_comparator_enforced_blocked() {
        let comparator = SeamComparator::new(SeamPositionMode::Aligned, 0.6);

        let neutral = SeamCandidate::new(Point3f::new(0.0, 0.0, 0.0), 0, 0.0);
        let enforced = SeamCandidate::enforced(Point3f::new(0.0, 0.0, 0.0), 0, 0.0);
        let blocked = SeamCandidate::blocked(Point3f::new(0.0, 0.0, 0.0), 0, 0.0);

        // Enforced > Neutral > Blocked
        assert!(comparator.is_first_better(&enforced, &neutral));
        assert!(comparator.is_first_better(&neutral, &blocked));
        assert!(comparator.is_first_better(&enforced, &blocked));
    }

    #[test]
    fn test_seam_comparator_overhang() {
        let comparator = SeamComparator::new(SeamPositionMode::Aligned, 0.6);

        let mut no_overhang = SeamCandidate::new(Point3f::new(0.0, 0.0, 0.0), 0, 0.0);
        let mut with_overhang = SeamCandidate::new(Point3f::new(0.0, 0.0, 0.0), 0, 0.0);
        with_overhang.overhang = 0.5;

        // No overhang is better
        assert!(comparator.is_first_better(&no_overhang, &with_overhang));
    }

    #[test]
    fn test_seam_comparator_embedded() {
        let comparator = SeamComparator::new(SeamPositionMode::Aligned, 0.6);

        let mut hidden = SeamCandidate::new(Point3f::new(0.0, 0.0, 0.0), 0, 0.0);
        hidden.embedded_distance = -1.0; // Inside

        let exposed = SeamCandidate::new(Point3f::new(0.0, 0.0, 0.0), 0, 0.0);

        // Hidden is better
        assert!(comparator.is_first_better(&hidden, &exposed));
    }

    #[test]
    fn test_place_seam_convenience() {
        let square = make_square(10.0);

        let rear_idx = place_seam(&square, SeamPositionMode::Rear, None);
        assert!(rear_idx == 2 || rear_idx == 3); // Top corners

        let nearest_idx = place_seam(&square, SeamPositionMode::Nearest, Some(Point::zero()));
        assert_eq!(nearest_idx, 0); // Origin corner
    }

    #[test]
    fn test_create_seam_placer_convenience() {
        let square = make_square(10.0);
        let layers = vec![(0.2, vec![square.clone()]), (0.4, vec![square])];

        let placer = create_seam_placer(&layers, 0.4, SeamPositionMode::Aligned);
        let stats = placer.stats();

        assert_eq!(stats.layer_count, 2);
        assert_eq!(stats.seam_position_mode, SeamPositionMode::Aligned);
    }

    #[test]
    fn test_angle_calculation() {
        let angles = calculate_polygon_angles(
            &[
                Point::new(0, 0),
                Point::new(scale(10.0), 0),
                Point::new(scale(10.0), scale(10.0)),
                Point::new(0, scale(10.0)),
            ],
            &[10.0, 10.0, 10.0, 10.0],
            0.5,
        );

        // Square corners should have ~90 degree angles (π/2 radians)
        for angle in &angles {
            assert!((*angle - std::f64::consts::FRAC_PI_2).abs() < 0.2);
        }
    }

    #[test]
    fn test_find_best_corner_concave() {
        let concave = make_concave_shape();
        let best = find_best_corner(&concave);
        // Should find the concave corner
        assert_eq!(best, 3);
    }

    #[test]
    fn test_seam_placer_empty_polygon() {
        let placer = SeamPlacer::with_defaults();
        let empty = Polygon::new();

        let idx = placer.fallback_seam_point(&empty, None);
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_seam_placer_single_point() {
        let placer = SeamPlacer::with_defaults();
        let single = Polygon::from_points(vec![Point::new(0, 0)]);

        let idx = placer.fallback_seam_point(&single, None);
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_random_seam_deterministic() {
        let placer = SeamPlacer::new(SeamPlacerConfig::random());
        let square = make_square(10.0);

        // Same polygon should give same result
        let idx1 = placer.fallback_seam_point(&square, None);
        let idx2 = placer.fallback_seam_point(&square, None);
        assert_eq!(idx1, idx2);
    }

    #[test]
    fn test_seam_placer_stats() {
        let mut placer = SeamPlacer::with_defaults();
        let square = make_square(10.0);
        let layers = vec![
            (0.2, vec![square.clone(), square.clone()]),
            (0.4, vec![square]),
        ];

        placer.init_simple(&layers, 0.4);
        let stats = placer.stats();

        assert_eq!(stats.layer_count, 2);
        assert_eq!(stats.total_perimeters, 3);
        assert!(stats.total_candidates >= 12);
    }

    #[test]
    fn test_get_seam_position_3d() {
        let mut placer = SeamPlacer::with_defaults();
        let square = make_square(10.0);
        let layers = vec![(0.2, vec![square])];

        placer.init_simple(&layers, 0.4);

        let pos = placer.get_seam_position_3d(0, 0);
        assert!(pos.is_some());

        let p = pos.unwrap();
        assert!((p.z - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_layer_outline_creation() {
        let outline = LayerOutline {
            z_height: 0.2,
            perimeters: vec![PerimeterOutline {
                polygon: make_square(10.0),
                flow_width: 0.4,
                is_external: true,
            }],
            layer_regions: vec![make_square(10.0)],
        };

        assert_eq!(outline.z_height, 0.2);
        assert_eq!(outline.perimeters.len(), 1);
        assert_eq!(outline.layer_regions.len(), 1);
    }
}
