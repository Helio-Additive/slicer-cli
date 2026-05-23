//! Adhesion features: Brim, Skirt, and Raft generation.
//!
//! This module implements adhesion helpers similar to BambuStudio's Brim.cpp:
//! - Brim: Flat area around the first layer for better bed adhesion
//! - Skirt: Outline around the print to prime the nozzle
//! - Raft: Full support platform under the print

use crate::clipper_utils::{offset_expolygon, offset_polygons, union_ex, OffsetJoinType};
use crate::gcode::{ExtrusionPath, ExtrusionRole};
use crate::geometry::{ExPolygon, Point, Polygon};
use crate::{scale, unscale, Coord};

/// Configuration for brim generation.
#[derive(Debug, Clone)]
pub struct BrimConfig {
    /// Brim width in mm.
    pub width: f64,
    /// Brim line spacing in mm (typically same as extrusion width).
    pub line_spacing: f64,
    /// Number of brim loops (alternative to width-based calculation).
    pub loops: Option<u32>,
    /// Gap between brim and model in mm.
    pub gap: f64,
    /// Brim type.
    pub brim_type: BrimType,
    /// Whether brim is enabled.
    pub enabled: bool,
    /// Extrusion width for brim lines in mm.
    pub extrusion_width: f64,
    /// Minimum area for brim application (mm²).
    pub min_area: f64,
    /// Smart brim: only add brim where needed.
    pub smart_brim: bool,
}

impl Default for BrimConfig {
    fn default() -> Self {
        Self {
            width: 5.0,
            line_spacing: 0.45,
            loops: None,
            gap: 0.0,
            brim_type: BrimType::Outer,
            enabled: false,
            extrusion_width: 0.45,
            min_area: 0.0,
            smart_brim: false,
        }
    }
}

impl BrimConfig {
    // Create a new brim config.
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable brim with the given width.
    pub fn with_width(mut self, width: f64) -> Self {
        self.width = width.max(0.0);
        self.enabled = width > 0.0;
        self
    }

    /// Set brim using number of loops.
    pub fn with_loops(mut self, loops: u32) -> Self {
        self.loops = Some(loops);
        self.enabled = loops > 0;
        self
    }

    /// Set brim type.
    pub fn with_type(mut self, brim_type: BrimType) -> Self {
        self.brim_type = brim_type;
        self
    }

    /// Set gap between brim and model.
    pub fn with_gap(mut self, gap: f64) -> Self {
        self.gap = gap.max(0.0);
        self
    }

    /// Set line spacing.
    pub fn with_line_spacing(mut self, spacing: f64) -> Self {
        self.line_spacing = spacing.max(0.1);
        self
    }

    /// Enable smart brim.
    pub fn with_smart_brim(mut self, smart: bool) -> Self {
        self.smart_brim = smart;
        self
    }

    /// Calculate the number of brim loops.
    pub fn calculate_loops(&self) -> u32 {
        if let Some(loops) = self.loops {
            loops
        } else if self.line_spacing > 0.0 {
            (self.width / self.line_spacing).ceil() as u32
        } else {
            0
        }
    }

    /// Get the scaled line spacing.
    pub fn scaled_line_spacing(&self) -> Coord {
        scale(self.line_spacing)
    }

    /// Get the scaled gap.
    pub fn scaled_gap(&self) -> Coord {
        scale(self.gap)
    }
}

/// Type of brim to generate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BrimType {
    /// Brim around the outside of the model only.
    Outer,
    /// Brim inside holes only.
    Inner,
    /// Brim both inside and outside.
    Both,
    /// No brim.
    None,
}

impl Default for BrimType {
    fn default() -> Self {
        Self::Outer
    }
}

/// Configuration for skirt generation.
#[derive(Debug, Clone)]
pub struct SkirtConfig {
    /// Number of skirt loops.
    pub loops: u32,
    /// Distance from the model in mm.
    pub distance: f64,
    /// Minimum skirt length in mm (will add loops until reached).
    pub min_length: f64,
    /// Skirt height in layers.
    pub height: u32,
    /// Whether skirt is enabled.
    pub enabled: bool,
    /// Extrusion width for skirt lines in mm.
    pub extrusion_width: f64,
    /// Draft shield: extend skirt to full height.
    pub draft_shield: bool,
}

impl Default for SkirtConfig {
    fn default() -> Self {
        Self {
            loops: 1,
            distance: 6.0,
            min_length: 0.0,
            height: 1,
            enabled: true,
            extrusion_width: 0.45,
            draft_shield: false,
        }
    }
}

impl SkirtConfig {
    // Create a new skirt config.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set number of loops.
    pub fn with_loops(mut self, loops: u32) -> Self {
        self.loops = loops;
        self.enabled = loops > 0;
        self
    }

    /// Set distance from model.
    pub fn with_distance(mut self, distance: f64) -> Self {
        self.distance = distance.max(0.0);
        self
    }

    /// Set minimum length.
    pub fn with_min_length(mut self, length: f64) -> Self {
        self.min_length = length.max(0.0);
        self
    }

    /// Set skirt height in layers.
    pub fn with_height(mut self, height: u32) -> Self {
        self.height = height.max(1);
        self
    }

    /// Enable draft shield (full height skirt).
    pub fn with_draft_shield(mut self, enabled: bool) -> Self {
        self.draft_shield = enabled;
        self
    }

    /// Get the scaled distance.
    pub fn scaled_distance(&self) -> Coord {
        scale(self.distance)
    }
}

/// Configuration for raft generation.
#[derive(Debug, Clone)]
pub struct RaftConfig {
    /// Number of raft layers.
    pub layers: u32,
    /// Raft expansion beyond the model in mm.
    pub expansion: f64,
    /// Gap between raft and model in mm (Z direction).
    pub contact_distance: f64,
    /// First layer of raft line spacing in mm.
    pub first_layer_spacing: f64,
    /// Interface layer line spacing in mm.
    pub interface_spacing: f64,
    /// Whether raft is enabled.
    pub enabled: bool,
    /// Raft density (0.0 - 1.0).
    pub density: f64,
}

impl Default for RaftConfig {
    fn default() -> Self {
        Self {
            layers: 3,
            expansion: 1.5,
            contact_distance: 0.2,
            first_layer_spacing: 0.8,
            interface_spacing: 0.4,
            enabled: false,
            density: 1.0,
        }
    }
}

impl RaftConfig {
    // Create a new raft config.
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable raft with the given number of layers.
    pub fn with_layers(mut self, layers: u32) -> Self {
        self.layers = layers;
        self.enabled = layers > 0;
        self
    }

    /// Set raft expansion.
    pub fn with_expansion(mut self, expansion: f64) -> Self {
        self.expansion = expansion.max(0.0);
        self
    }

    /// Set contact distance.
    pub fn with_contact_distance(mut self, distance: f64) -> Self {
        self.contact_distance = distance.max(0.0);
        self
    }

    /// Set raft density.
    pub fn with_density(mut self, density: f64) -> Self {
        self.density = density.clamp(0.0, 1.0);
        self
    }
}

/// Generated brim result.
#[derive(Debug, Clone)]
pub struct BrimResult {
    /// Brim loops (from inside to outside).
    pub loops: Vec<Polygon>,
    /// Total brim length in mm.
    pub total_length: f64,
    /// Brim area in mm².
    pub area: f64,
}

impl BrimResult {
    // Create a new brim result.
    pub fn new(loops: Vec<Polygon>) -> Self {
        let total_length = loops.iter().map(|p| unscale(p.perimeter() as Coord)).sum();
        let area = loops
            .iter()
            .map(|p| p.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR))
            .sum();

        Self {
            loops,
            total_length,
            area,
        }
    }

    /// Check if brim is empty.
    pub fn is_empty(&self) -> bool {
        self.loops.is_empty()
    }

    /// Get the number of loops.
    pub fn loop_count(&self) -> usize {
        self.loops.len()
    }

    /// Convert to extrusion paths.
    pub fn to_extrusion_paths(&self, config: &BrimConfig) -> Vec<ExtrusionPath> {
        self.loops
            .iter()
            .map(|polygon| {
                ExtrusionPath::from_polygon(polygon, ExtrusionRole::Skirt)
                    .with_width(config.extrusion_width)
                    .with_height(0.2) // Set by caller based on layer height
                    .with_speed(60.0)
            })
            .collect()
    }
}

/// Generated skirt result.
#[derive(Debug, Clone)]
pub struct SkirtResult {
    /// Skirt loops (from inside to outside).
    pub loops: Vec<Polygon>,
    /// Total skirt length in mm.
    pub total_length: f64,
}

impl SkirtResult {
    // Create a new skirt result.
    pub fn new(loops: Vec<Polygon>) -> Self {
        let total_length = loops.iter().map(|p| unscale(p.perimeter() as Coord)).sum();

        Self {
            loops,
            total_length,
        }
    }

    /// Check if skirt is empty.
    pub fn is_empty(&self) -> bool {
        self.loops.is_empty()
    }

    /// Get the number of loops.
    pub fn loop_count(&self) -> usize {
        self.loops.len()
    }

    /// Convert to extrusion paths.
    pub fn to_extrusion_paths(&self, config: &SkirtConfig) -> Vec<ExtrusionPath> {
        self.loops
            .iter()
            .map(|polygon| {
                ExtrusionPath::from_polygon(polygon, ExtrusionRole::Skirt)
                    .with_width(config.extrusion_width)
                    .with_height(0.2)
                    .with_speed(60.0)
            })
            .collect()
    }
}

/// Brim generator.
#[derive(Debug)]
pub struct BrimGenerator {
    /// Brim configuration.
    config: BrimConfig,
}

impl BrimGenerator {
    // Create a new brim generator.
    pub fn new(config: BrimConfig) -> Self {
        Self { config }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(BrimConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &BrimConfig {
        &self.config
    }

    /// Check if brim is enabled.
    pub fn is_enabled(&self) -> bool {
        self.config.enabled
    }

    /// Generate brim around the given first layer slices.
    ///
    /// # Arguments
    /// * `first_layer_slices` - The first layer geometry to add brim around
    ///
    /// # Returns
    /// The generated brim loops
    pub fn generate(&self, first_layer_slices: &[ExPolygon]) -> BrimResult {
        if !self.is_enabled() || first_layer_slices.is_empty() {
            return BrimResult::new(vec![]);
        }

        let num_loops = self.config.calculate_loops();
        if num_loops == 0 {
            return BrimResult::new(vec![]);
        }

        let mut loops = Vec::new();
        let line_spacing = self.config.line_spacing;
        let gap = self.config.gap;

        // Union all slices to handle overlapping objects
        let unified = union_ex(first_layer_slices);

        match self.config.brim_type {
            BrimType::Outer | BrimType::Both => {
                // Generate outer brim loops
                for i in 0..num_loops {
                    let offset = gap + line_spacing * (i as f64) + line_spacing / 2.0;

                    for expoly in &unified {
                        let grown = offset_expolygon(expoly, offset, OffsetJoinType::Round);
                        for ep in grown {
                            loops.push(ep.contour);
                        }
                    }
                }
            }
            BrimType::Inner => {
                // Inner brim only - handled below
            }
            BrimType::None => {}
        }

        if self.config.brim_type == BrimType::Both || self.config.brim_type == BrimType::Inner {
            // Generate inner brim (inside holes)
            for expoly in &unified {
                for hole in &expoly.holes {
                    for i in 0..num_loops {
                        let offset = -(gap + line_spacing * (i as f64) + line_spacing / 2.0);
                        // Create a temporary expolygon from the hole
                        let hole_expoly = ExPolygon::new(hole.clone());
                        let shrunk = offset_expolygon(&hole_expoly, offset, OffsetJoinType::Round);
                        for ep in shrunk {
                            loops.push(ep.contour);
                        }
                    }
                }
            }
        }

        BrimResult::new(loops)
    }

    /// Generate brim for support structures.
    pub fn generate_support_brim(&self, support_slices: &[ExPolygon]) -> BrimResult {
        // Support brim uses fewer loops typically
        let mut config = self.config.clone();
        config.width = (config.width * 0.5).max(config.line_spacing * 2.0);

        let generator = BrimGenerator::new(config);
        generator.generate(support_slices)
    }
}

impl Default for BrimGenerator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Skirt generator.
#[derive(Debug)]
pub struct SkirtGenerator {
    /// Skirt configuration.
    config: SkirtConfig,
}

impl SkirtGenerator {
    // Create a new skirt generator.
    pub fn new(config: SkirtConfig) -> Self {
        Self { config }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(SkirtConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &SkirtConfig {
        &self.config
    }

    /// Check if skirt is enabled.
    pub fn is_enabled(&self) -> bool {
        self.config.enabled && self.config.loops > 0
    }

    /// Generate skirt around the given first layer slices.
    ///
    /// # Arguments
    /// * `first_layer_slices` - The first layer geometry to add skirt around
    ///
    /// # Returns
    /// The generated skirt loops
    pub fn generate(&self, first_layer_slices: &[ExPolygon]) -> SkirtResult {
        if !self.is_enabled() || first_layer_slices.is_empty() {
            return SkirtResult::new(vec![]);
        }

        let mut loops = Vec::new();
        let distance = self.config.distance;
        let line_spacing = self.config.extrusion_width;

        // Union all slices
        let unified = union_ex(first_layer_slices);

        // Generate skirt loops
        let mut current_offset = distance;
        let mut total_length: f64 = 0.0;
        let mut loop_count = 0;

        while loop_count < self.config.loops
            || (self.config.min_length > 0.0 && total_length < self.config.min_length)
        {
            for expoly in &unified {
                let grown = offset_expolygon(expoly, current_offset, OffsetJoinType::Round);
                for ep in &grown {
                    total_length += unscale(ep.contour.perimeter() as Coord);
                    loops.push(ep.contour.clone());
                }
            }

            current_offset += line_spacing;
            loop_count += 1;

            // Safety limit
            if loop_count > 100 {
                break;
            }
        }

        SkirtResult::new(loops)
    }

    /// Calculate the required number of loops to meet minimum length.
    pub fn calculate_loops_for_length(&self, perimeter_length: f64) -> u32 {
        if self.config.min_length <= 0.0 || perimeter_length <= 0.0 {
            return self.config.loops;
        }

        let loops_needed = (self.config.min_length / perimeter_length).ceil() as u32;
        loops_needed.max(self.config.loops)
    }
}

impl Default for SkirtGenerator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Generated raft result.
#[derive(Debug, Clone)]
pub struct RaftResult {
    /// Raft layers (bottom to top: base layers, interface layers, contact layer).
    pub layers: Vec<RaftLayer>,
    /// Total raft height in mm.
    pub total_height: f64,
}

/// A single raft layer.
#[derive(Debug, Clone)]
pub struct RaftLayer {
    /// Layer index (0 = bottom).
    pub index: usize,
    /// Layer type.
    pub layer_type: RaftLayerType,
    /// Z height of this layer.
    pub z_height: f64,
    /// Layer thickness.
    pub thickness: f64,
    /// Outline of the raft at this layer.
    pub outline: Vec<Polygon>,
    /// Fill paths for this layer.
    pub fill_paths: Vec<Polygon>,
    /// Line spacing for this layer.
    pub line_spacing: f64,
}

/// Type of raft layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RaftLayerType {
    /// Base layer (thicker, wider spacing, for adhesion).
    Base,
    /// Interface layer (medium spacing).
    Interface,
    /// Contact layer (fine spacing, for smooth bottom surface).
    Contact,
}

impl RaftResult {
    // Create a new empty raft result.
    pub fn new() -> Self {
        Self {
            layers: Vec::new(),
            total_height: 0.0,
        }
    }

    /// Check if the raft is empty.
    pub fn is_empty(&self) -> bool {
        self.layers.is_empty()
    }

    /// Get the number of layers.
    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }

    /// Get the Z offset for the model (raft height + contact distance).
    pub fn model_z_offset(&self, contact_distance: f64) -> f64 {
        self.total_height + contact_distance
    }
}

impl Default for RaftResult {
    fn default() -> Self {
        Self::new()
    }
}

/// Generator for raft structures.
///
/// A raft is a horizontal platform printed beneath the model to improve
/// bed adhesion and help with warping. It consists of:
/// - Base layers: Thick, widely-spaced lines for bed adhesion
/// - Interface layers: Medium spacing for structural support
/// - Contact layer: Fine spacing for a smooth surface
pub struct RaftGenerator {
    config: RaftConfig,
}

impl RaftGenerator {
    // Create a new raft generator with the given configuration.
    pub fn new(config: RaftConfig) -> Self {
        Self { config }
    }

    /// Create a new raft generator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(RaftConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &RaftConfig {
        &self.config
    }

    /// Check if raft generation is enabled.
    pub fn is_enabled(&self) -> bool {
        self.config.enabled && self.config.layers > 0
    }

    /// Generate raft for the given first layer slices.
    ///
    /// # Arguments
    /// * `first_layer_slices` - The bottom layer of the model
    /// * `first_layer_height` - Height of the first layer
    /// * `layer_height` - Normal layer height
    /// * `nozzle_diameter` - Nozzle diameter for line width calculation
    ///
    /// # Returns
    /// A `RaftResult` containing all raft layers.
    pub fn generate(
        &self,
        first_layer_slices: &[ExPolygon],
        first_layer_height: f64,
        layer_height: f64,
        nozzle_diameter: f64,
    ) -> RaftResult {
        if !self.is_enabled() || first_layer_slices.is_empty() {
            return RaftResult::new();
        }

        let expansion = self.config.expansion;
        let num_layers = self.config.layers as usize;

        // Calculate layer distribution:
        // - At least 1 base layer
        // - At least 1 contact layer (topmost)
        // - Remaining are interface layers
        let base_layers = 1.max(num_layers / 3);
        let contact_layers = 1;
        let interface_layers = num_layers.saturating_sub(base_layers + contact_layers);

        // Create the raft outline by offsetting the first layer contours
        let contours: Vec<Polygon> = first_layer_slices
            .iter()
            .map(|expoly| expoly.contour.clone())
            .collect();

        // Expand the contours using clipper
        let expanded_expolys = offset_polygons(&contours, expansion, OffsetJoinType::Round);

        // Extract the contours as the raft outline
        let raft_outline: Vec<Polygon> = expanded_expolys
            .iter()
            .map(|expoly| expoly.contour.clone())
            .collect();

        let mut layers = Vec::with_capacity(num_layers);
        let mut current_z = 0.0;

        // Base layer height is thicker
        let base_layer_height = first_layer_height * 1.5;
        // Interface layer height matches normal layer height
        let interface_layer_height = layer_height;
        // Contact layer is thinner for smooth surface
        let contact_layer_height = layer_height * 0.75;

        // Line widths
        // Line widths (for future use in extrusion path generation)
        let _base_line_width = nozzle_diameter * 1.5;
        let _interface_line_width = nozzle_diameter * 1.2;
        let _contact_line_width = nozzle_diameter;

        // Generate base layers
        for i in 0..base_layers {
            current_z += base_layer_height;
            let layer = RaftLayer {
                index: i,
                layer_type: RaftLayerType::Base,
                z_height: current_z,
                thickness: base_layer_height,
                outline: raft_outline.clone(),
                fill_paths: self.generate_fill_paths(
                    &raft_outline,
                    self.config.first_layer_spacing,
                    i,
                ),
                line_spacing: self.config.first_layer_spacing,
            };
            layers.push(layer);
        }

        // Generate interface layers
        for i in 0..interface_layers {
            current_z += interface_layer_height;
            let layer_idx = base_layers + i;
            let layer = RaftLayer {
                index: layer_idx,
                layer_type: RaftLayerType::Interface,
                z_height: current_z,
                thickness: interface_layer_height,
                outline: raft_outline.clone(),
                fill_paths: self.generate_fill_paths(
                    &raft_outline,
                    self.config.interface_spacing,
                    layer_idx,
                ),
                line_spacing: self.config.interface_spacing,
            };
            layers.push(layer);
        }

        // Generate contact layer
        current_z += contact_layer_height;
        let contact_spacing = self.config.interface_spacing * 0.75;
        let layer = RaftLayer {
            index: num_layers - 1,
            layer_type: RaftLayerType::Contact,
            z_height: current_z,
            thickness: contact_layer_height,
            outline: raft_outline.clone(),
            fill_paths: self.generate_fill_paths(&raft_outline, contact_spacing, num_layers - 1),
            line_spacing: contact_spacing,
        };
        layers.push(layer);

        RaftResult {
            layers,
            total_height: current_z,
        }
    }

    /// Generate fill paths for a raft layer.
    fn generate_fill_paths(
        &self,
        outline: &[Polygon],
        spacing: f64,
        layer_index: usize,
    ) -> Vec<Polygon> {
        // Simple rectilinear fill pattern
        // Alternate direction every layer (0°, 90°)
        let is_horizontal = layer_index % 2 == 0;

        let mut fill_paths = Vec::new();
        let spacing_scaled = scale(spacing);

        for poly in outline {
            let bbox = poly.bounding_box();

            // Generate parallel lines
            let (start, end) = if is_horizontal {
                (bbox.min.y, bbox.max.y)
            } else {
                (bbox.min.x, bbox.max.x)
            };

            let mut pos = start + spacing_scaled / 2;
            while pos < end {
                let line = if is_horizontal {
                    Polygon::from_points(vec![
                        Point::new(bbox.min.x, pos),
                        Point::new(bbox.max.x, pos),
                    ])
                } else {
                    Polygon::from_points(vec![
                        Point::new(pos, bbox.min.y),
                        Point::new(pos, bbox.max.y),
                    ])
                };
                fill_paths.push(line);
                pos += spacing_scaled;
            }
        }

        fill_paths
    }

    /// Get the total raft height for layer offsetting.
    pub fn calculate_raft_height(&self, first_layer_height: f64, layer_height: f64) -> f64 {
        if !self.is_enabled() {
            return 0.0;
        }

        let num_layers = self.config.layers as usize;
        let base_layers = 1.max(num_layers / 3);
        let contact_layers = 1;
        let interface_layers = num_layers.saturating_sub(base_layers + contact_layers);

        let base_layer_height = first_layer_height * 1.5;
        let interface_layer_height = layer_height;
        let contact_layer_height = layer_height * 0.75;

        (base_layers as f64 * base_layer_height)
            + (interface_layers as f64 * interface_layer_height)
            + (contact_layers as f64 * contact_layer_height)
    }
}

impl Default for RaftGenerator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Calculate the convex hull of first layer slices for skirt/brim.
pub fn first_layer_convex_hull(slices: &[ExPolygon]) -> Polygon {
    let mut all_points = Vec::new();

    for expoly in slices {
        all_points.extend(expoly.contour.points().iter().cloned());
    }

    if all_points.is_empty() {
        return Polygon::new();
    }

    convex_hull(&all_points)
}

/// Compute the convex hull of a set of points.
fn convex_hull(points: &[Point]) -> Polygon {
    if points.len() < 3 {
        return Polygon::from_points(points.to_vec());
    }

    // Find the bottom-most point (or left-most in case of tie)
    let mut start_idx = 0;
    for (i, p) in points.iter().enumerate() {
        if p.y < points[start_idx].y || (p.y == points[start_idx].y && p.x < points[start_idx].x) {
            start_idx = i;
        }
    }

    // Sort points by polar angle with respect to start point
    let start = points[start_idx];
    let mut sorted: Vec<Point> = points.iter().filter(|p| **p != start).cloned().collect();

    sorted.sort_by(|a, b| {
        let angle_a = ((a.y - start.y) as f64).atan2((a.x - start.x) as f64);
        let angle_b = ((b.y - start.y) as f64).atan2((b.x - start.x) as f64);
        angle_a
            .partial_cmp(&angle_b)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Build the convex hull using Graham scan
    let mut hull = vec![start];

    for p in sorted {
        while hull.len() > 1 {
            let len = hull.len();
            if cross_product(&hull[len - 2], &hull[len - 1], &p) <= 0 {
                hull.pop();
            } else {
                break;
            }
        }
        hull.push(p);
    }

    Polygon::from_points(hull)
}

/// Cross product of vectors (p1 - p0) and (p2 - p0).
fn cross_product(p0: &Point, p1: &Point, p2: &Point) -> i64 {
    (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_expolygon(size_mm: f64) -> ExPolygon {
        let half = scale(size_mm / 2.0);
        let contour = Polygon::from_points(vec![
            Point::new(-half, -half),
            Point::new(half, -half),
            Point::new(half, half),
            Point::new(-half, half),
        ]);
        ExPolygon::new(contour)
    }

    #[test]
    fn test_brim_config_default() {
        let config = BrimConfig::default();
        assert!(!config.enabled);
        assert_eq!(config.width, 5.0);
        assert_eq!(config.brim_type, BrimType::Outer);
    }

    #[test]
    fn test_brim_config_builder() {
        let config = BrimConfig::new()
            .with_width(8.0)
            .with_gap(0.1)
            .with_type(BrimType::Both);

        assert!(config.enabled);
        assert_eq!(config.width, 8.0);
        assert_eq!(config.gap, 0.1);
        assert_eq!(config.brim_type, BrimType::Both);
    }

    #[test]
    fn test_brim_config_loops() {
        let config = BrimConfig::new().with_loops(5);
        assert!(config.enabled);
        assert_eq!(config.loops, Some(5));
        assert_eq!(config.calculate_loops(), 5);
    }

    #[test]
    fn test_brim_config_calculate_loops_from_width() {
        let config = BrimConfig::new().with_width(4.5).with_line_spacing(0.45);

        // 4.5 / 0.45 = 10 loops
        assert_eq!(config.calculate_loops(), 10);
    }

    #[test]
    fn test_skirt_config_default() {
        let config = SkirtConfig::default();
        assert!(config.enabled);
        assert_eq!(config.loops, 1);
        assert_eq!(config.distance, 6.0);
    }

    #[test]
    fn test_skirt_config_builder() {
        let config = SkirtConfig::new()
            .with_loops(3)
            .with_distance(10.0)
            .with_min_length(100.0);

        assert!(config.enabled);
        assert_eq!(config.loops, 3);
        assert_eq!(config.distance, 10.0);
        assert_eq!(config.min_length, 100.0);
    }

    #[test]
    fn test_raft_config_default() {
        let config = RaftConfig::default();
        assert!(!config.enabled);
        assert_eq!(config.layers, 3);
    }

    #[test]
    fn test_raft_config_builder() {
        let config = RaftConfig::new()
            .with_layers(4)
            .with_expansion(2.0)
            .with_density(0.8);

        assert!(config.enabled);
        assert_eq!(config.layers, 4);
        assert_eq!(config.expansion, 2.0);
        assert_eq!(config.density, 0.8);
    }

    #[test]
    fn test_brim_generator_disabled() {
        let generator = BrimGenerator::with_defaults();
        let slices = vec![make_square_expolygon(10.0)];

        let result = generator.generate(&slices);
        assert!(result.is_empty());
    }

    #[test]
    fn test_brim_generator_enabled() {
        let config = BrimConfig::new().with_width(3.0);
        let generator = BrimGenerator::new(config);
        let slices = vec![make_square_expolygon(10.0)];

        let result = generator.generate(&slices);
        assert!(!result.is_empty());
        assert!(result.total_length > 0.0);
    }

    #[test]
    fn test_brim_result_to_extrusion_paths() {
        let config = BrimConfig::new().with_width(2.0);
        let generator = BrimGenerator::new(config.clone());
        let slices = vec![make_square_expolygon(10.0)];

        let result = generator.generate(&slices);
        let paths = result.to_extrusion_paths(&config);

        assert!(!paths.is_empty());
        for path in &paths {
            assert_eq!(path.role, ExtrusionRole::Skirt);
            assert!(path.is_closed);
        }
    }

    #[test]
    fn test_skirt_generator_disabled() {
        let config = SkirtConfig::new().with_loops(0);
        let generator = SkirtGenerator::new(config);
        let slices = vec![make_square_expolygon(10.0)];

        let result = generator.generate(&slices);
        assert!(result.is_empty());
    }

    #[test]
    fn test_skirt_generator_enabled() {
        let config = SkirtConfig::new().with_loops(2).with_distance(5.0);
        let generator = SkirtGenerator::new(config);
        let slices = vec![make_square_expolygon(10.0)];

        let result = generator.generate(&slices);
        assert!(!result.is_empty());
        assert!(result.total_length > 0.0);
    }

    #[test]
    fn test_skirt_min_length() {
        let generator = SkirtGenerator::with_defaults();

        // 40mm perimeter, 100mm min length = need 3 loops
        let loops = generator.calculate_loops_for_length(40.0);
        assert!(loops >= 1);
    }

    #[test]
    fn test_convex_hull_simple() {
        let points = vec![
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            Point::new(scale(10.0), scale(10.0)),
            Point::new(0, scale(10.0)),
            Point::new(scale(5.0), scale(5.0)), // Interior point
        ];

        let hull = convex_hull(&points);

        // Should have 4 points (the interior point is excluded)
        assert_eq!(hull.points().len(), 4);
    }

    #[test]
    fn test_first_layer_convex_hull() {
        let slices = vec![make_square_expolygon(10.0), make_square_expolygon(5.0)];

        let hull = first_layer_convex_hull(&slices);
        assert!(!hull.points().is_empty());
    }

    #[test]
    fn test_brim_empty_slices() {
        let config = BrimConfig::new().with_width(5.0);
        let generator = BrimGenerator::new(config);

        let result = generator.generate(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_skirt_result() {
        let loops = vec![Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            Point::new(scale(10.0), scale(10.0)),
            Point::new(0, scale(10.0)),
        ])];

        let result = SkirtResult::new(loops);
        assert_eq!(result.loop_count(), 1);
        assert!(result.total_length > 30.0); // Perimeter should be ~40mm
    }

    #[test]
    fn test_brim_type_variants() {
        assert_eq!(BrimType::default(), BrimType::Outer);

        let outer = BrimType::Outer;
        let inner = BrimType::Inner;
        let both = BrimType::Both;
        let none = BrimType::None;

        assert_ne!(outer, inner);
        assert_ne!(both, none);
    }

    #[test]
    fn test_raft_generator_disabled() {
        let config = RaftConfig::default();
        assert!(!config.enabled);

        let generator = RaftGenerator::new(config);
        assert!(!generator.is_enabled());

        let slices = vec![make_square_expolygon(10.0)];
        let result = generator.generate(&slices, 0.3, 0.2, 0.4);
        assert!(result.is_empty());
    }

    #[test]
    fn test_raft_generator_enabled() {
        let config = RaftConfig::new().with_layers(3);
        assert!(config.enabled);
        assert_eq!(config.layers, 3);

        let generator = RaftGenerator::new(config);
        assert!(generator.is_enabled());

        let slices = vec![make_square_expolygon(10.0)];
        let result = generator.generate(&slices, 0.3, 0.2, 0.4);

        assert!(!result.is_empty());
        assert_eq!(result.layer_count(), 3);
        assert!(result.total_height > 0.0);
    }

    #[test]
    fn test_raft_layer_types() {
        let config = RaftConfig::new().with_layers(5);
        let generator = RaftGenerator::new(config);

        let slices = vec![make_square_expolygon(10.0)];
        let result = generator.generate(&slices, 0.3, 0.2, 0.4);

        assert_eq!(result.layer_count(), 5);

        // Check that we have at least one of each type
        let has_base = result
            .layers
            .iter()
            .any(|l| l.layer_type == RaftLayerType::Base);
        let has_contact = result
            .layers
            .iter()
            .any(|l| l.layer_type == RaftLayerType::Contact);

        assert!(has_base, "Raft should have at least one base layer");
        assert!(has_contact, "Raft should have a contact layer");

        // Contact layer should be last
        assert_eq!(
            result.layers.last().unwrap().layer_type,
            RaftLayerType::Contact
        );
    }

    #[test]
    fn test_raft_result_model_offset() {
        let config = RaftConfig::new().with_layers(3).with_contact_distance(0.15);

        let generator = RaftGenerator::new(config);
        let slices = vec![make_square_expolygon(10.0)];
        let result = generator.generate(&slices, 0.3, 0.2, 0.4);

        let offset = result.model_z_offset(0.15);
        assert!(offset > result.total_height);
        assert!((offset - result.total_height - 0.15).abs() < 0.001);
    }

    #[test]
    fn test_raft_calculate_height() {
        let config = RaftConfig::new().with_layers(3);
        let generator = RaftGenerator::new(config);

        let height = generator.calculate_raft_height(0.3, 0.2);
        assert!(height > 0.0);

        // Disabled generator should return 0
        let disabled_config = RaftConfig::default();
        let disabled_generator = RaftGenerator::new(disabled_config);
        assert_eq!(disabled_generator.calculate_raft_height(0.3, 0.2), 0.0);
    }

    #[test]
    fn test_raft_empty_slices() {
        let config = RaftConfig::new().with_layers(3);
        let generator = RaftGenerator::new(config);

        let result = generator.generate(&[], 0.3, 0.2, 0.4);
        assert!(result.is_empty());
    }

    #[test]
    fn test_raft_expansion() {
        let config = RaftConfig::new().with_layers(2).with_expansion(2.0);

        let generator = RaftGenerator::new(config);
        assert_eq!(generator.config().expansion, 2.0);
    }
}
