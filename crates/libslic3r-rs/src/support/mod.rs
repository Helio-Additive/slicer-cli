//! Support structure generation.
//!
//! This module provides functionality for generating support structures
//! for 3D printing, including:
//! - Overhang detection (areas exceeding the overhang threshold angle)
//! - Support region calculation (projecting overhangs to build plate)
//! - Normal supports (grid/lines pattern)
//! - Tree supports (organic branching structures)
//! - Support interface layers (dense top/bottom layers)
//!
//! # Algorithm Overview
//!
//! 1. **Overhang Detection**: For each layer, compute the difference between
//!    the current layer's contours and the lower layer's contours (expanded
//!    by the overhang threshold). Areas not supported from below are overhangs.
//!
//! 2. **Support Region Calculation**: Project overhang regions downward,
//!    accumulating support areas layer by layer until reaching the build plate
//!    or existing model geometry.
//!
//! 3. **Support Trimming**: Offset support regions away from the model by
//!    the XY distance to prevent fusion.
//!
//! 4. **Pattern Generation**: Fill support regions with the selected pattern
//!    (grid, lines, honeycomb, etc.).
//!
//! # Tree Support 3D
//!
//! Tree supports use an organic branching algorithm that:
//! - Places support tips at overhang points
//! - Grows branches downward while avoiding model collisions
//! - Merges nearby branches to reduce material usage
//! - Optionally lands on model surfaces (not just build plate)
//!
//! Key submodules for tree support:
//! - `tree_model_volumes`: Collision detection and avoidance computation
//! - `tree_support_settings`: Configuration and support element state
//! - `tree_support_3d`: Main tree support generation algorithm

pub mod branch_mesh;
pub mod organic_smooth;
pub mod tree_model_volumes;
pub mod tree_support_3d;
pub mod tree_support_settings;

// Re-export tree support types for convenience
pub use branch_mesh::{
    build_branch_paths, generate_branch_mesh, slice_branch_mesh, BranchMeshBuilder,
    BranchMeshConfig, BranchMeshResult, BranchPath, BranchPathElement, Point3D,
};
pub use organic_smooth::{
    apply_smoothed_positions_to_move_bounds, build_spheres_from_move_bounds, smooth_move_bounds,
    OrganicSmoothConfig, OrganicSmoothResult, OrganicSmoother, SphereBuildResult, SphereMapping,
};
pub use tree_model_volumes::{TreeModelVolumes, TreeModelVolumesConfig};
pub use tree_support_3d::{TreeSupport3D, TreeSupport3DConfig, TreeSupport3DResult};
pub use tree_support_settings::TreeSupportSettings;

use crate::clipper_utils::{self, OffsetJoinType};
use crate::geometry::{ExPolygon, ExPolygons, Point, PointF, Polygon};
use crate::{scale, unscale, Coord, CoordF};

/// Support generation configuration parameters
#[derive(Debug, Clone)]
pub struct SupportConfig {
    /// Enable support generation
    pub enabled: bool,
    /// Support pattern type
    pub pattern: SupportPattern,
    /// Support density (0.0 - 1.0)
    pub density: f64,
    /// Z distance between support and model (mm)
    pub z_distance: f64,
    /// XY distance between support and model (mm)
    pub xy_distance: f64,
    /// Number of interface layers at top of support
    pub top_interface_layers: usize,
    /// Number of interface layers at bottom of support
    pub bottom_interface_layers: usize,
    /// Interface layer density (0.0 - 1.0)
    pub interface_density: f64,
    /// Overhang angle threshold (degrees) - areas steeper than this need support
    pub overhang_angle: f64,
    /// Minimum overhang area to generate support (mm²)
    pub min_area: f64,
    /// Support on build plate only (don't support on top of model)
    pub buildplate_only: bool,
    /// Support type (normal or tree)
    pub support_type: SupportType,
    /// Extrusion width for support (mm)
    pub extrusion_width: f64,
    /// Enable support roof (dense interface at top)
    pub support_roof: bool,
    /// Enable support floor (dense interface at bottom)
    pub support_floor: bool,
    /// Support line spacing (mm) - derived from density and width if not set
    pub line_spacing: Option<f64>,
    /// Tree support branch angle (degrees)
    pub tree_branch_angle: f64,
    /// Tree support branch diameter (mm)
    pub tree_branch_diameter: f64,
    /// Tree support tip diameter (mm)
    pub tree_tip_diameter: f64,
}

impl Default for SupportConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            pattern: SupportPattern::Grid,
            density: 0.15,
            z_distance: 0.2,
            xy_distance: 0.8,
            top_interface_layers: 3,
            bottom_interface_layers: 0,
            interface_density: 0.8,
            overhang_angle: 45.0,
            min_area: 1.0,
            buildplate_only: false,
            support_type: SupportType::Normal,
            extrusion_width: 0.4,
            support_roof: true,
            support_floor: false,
            line_spacing: None,
            tree_branch_angle: 40.0,
            tree_branch_diameter: 2.0,
            tree_tip_diameter: 0.8,
        }
    }
}

impl SupportConfig {
    // Create a new support config with supports enabled
    pub fn enabled() -> Self {
        Self {
            enabled: true,
            ..Default::default()
        }
    }

    /// Calculate the support expansion based on overhang angle
    /// This is how much we expand the lower layer to determine what's supported
    pub fn overhang_expansion(&self, layer_height: CoordF) -> CoordF {
        // tan(angle) = horizontal / vertical
        // horizontal = vertical * tan(angle)
        let angle_rad = self.overhang_angle.to_radians();
        layer_height * angle_rad.tan()
    }

    /// Calculate line spacing from density
    pub fn effective_line_spacing(&self) -> CoordF {
        self.line_spacing.unwrap_or_else(|| {
            // spacing = width / density
            self.extrusion_width / self.density
        })
    }

    /// Builder method to set pattern
    pub fn with_pattern(mut self, pattern: SupportPattern) -> Self {
        self.pattern = pattern;
        self
    }

    /// Builder method to set density
    pub fn with_density(mut self, density: f64) -> Self {
        self.density = density.clamp(0.05, 1.0);
        self
    }

    /// Builder method to set overhang angle
    pub fn with_overhang_angle(mut self, angle: f64) -> Self {
        self.overhang_angle = angle.clamp(0.0, 90.0);
        self
    }

    /// Builder method to enable tree supports
    pub fn tree_support(mut self) -> Self {
        self.support_type = SupportType::Tree;
        self
    }

    /// Builder method to set buildplate only
    pub fn buildplate_only(mut self, value: bool) -> Self {
        self.buildplate_only = value;
        self
    }
}

/// Support pattern types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SupportPattern {
    /// Grid pattern (cross-hatch)
    Grid,
    /// Lines pattern (parallel lines)
    Lines,
    /// Triangles pattern
    Triangles,
    /// Honeycomb pattern
    Honeycomb,
    /// Gyroid pattern (3D minimal surface)
    Gyroid,
    /// Lightning pattern (sparse tree-like)
    Lightning,
}

/// Support type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SupportType {
    /// Normal/Classic support (vertical pillars)
    Normal,
    /// Tree support (organic branching structures)
    Tree,
    /// Organic support (Bambu-style with smoother branches)
    Organic,
}

/// Represents a single layer of support structure
#[derive(Debug, Clone)]
pub struct SupportLayer {
    /// Layer index (matches model layer index)
    pub layer_id: usize,
    /// Z height of this support layer (mm)
    pub z: CoordF,
    /// Layer height/thickness (mm)
    pub height: CoordF,
    /// Support regions (areas that need support infill)
    pub support_regions: ExPolygons,
    /// Interface regions (dense top/bottom support areas)
    pub interface_regions: ExPolygons,
    /// Whether this is an interface layer (top or bottom of support)
    pub is_interface: bool,
    /// Overhang regions that initiated support at this layer
    pub overhang_regions: ExPolygons,
}

impl SupportLayer {
    // Create a new empty support layer
    pub fn new(layer_id: usize, z: CoordF, height: CoordF) -> Self {
        Self {
            layer_id,
            z,
            height,
            support_regions: Vec::new(),
            interface_regions: Vec::new(),
            is_interface: false,
            overhang_regions: Vec::new(),
        }
    }

    /// Check if this layer has any support
    pub fn is_empty(&self) -> bool {
        self.support_regions.is_empty() && self.interface_regions.is_empty()
    }

    /// Get total support area (mm²)
    pub fn total_area(&self) -> CoordF {
        let support_area: CoordF = self
            .support_regions
            .iter()
            .map(|p| {
                let area_scaled = p.area().abs();
                // area is in scaled² units, need to unscale twice
                area_scaled / (crate::SCALING_FACTOR * crate::SCALING_FACTOR)
            })
            .sum();
        let interface_area: CoordF = self
            .interface_regions
            .iter()
            .map(|p| {
                let area_scaled = p.area().abs();
                area_scaled / (crate::SCALING_FACTOR * crate::SCALING_FACTOR)
            })
            .sum();
        support_area + interface_area
    }
}

/// Support generator - main entry point for support generation
#[derive(Debug)]
pub struct SupportGenerator {
    config: SupportConfig,
}

impl SupportGenerator {
    // Create a new support generator with the given configuration
    pub fn new(config: SupportConfig) -> Self {
        Self { config }
    }

    /// Create a support generator with default configuration (supports enabled)
    pub fn enabled() -> Self {
        Self::new(SupportConfig::enabled())
    }

    /// Get the configuration
    pub fn config(&self) -> &SupportConfig {
        &self.config
    }

    /// Get mutable configuration
    pub fn config_mut(&mut self) -> &mut SupportConfig {
        &mut self.config
    }

    /// Generate support structures for a set of layer slices.
    ///
    /// # Arguments
    /// * `layer_slices` - Vector of (z_height, layer_height, slices) for each layer
    ///
    /// # Returns
    /// Vector of SupportLayer, one for each input layer (may be empty if no support needed)
    pub fn generate(&self, layer_slices: &[(CoordF, CoordF, ExPolygons)]) -> Vec<SupportLayer> {
        if !self.config.enabled || layer_slices.is_empty() {
            return layer_slices
                .iter()
                .enumerate()
                .map(|(i, (z, h, _))| SupportLayer::new(i, *z, *h))
                .collect();
        }

        // Dispatch based on support type
        match self.config.support_type {
            SupportType::Tree | SupportType::Organic => self.generate_tree_support(layer_slices),
            SupportType::Normal => self.generate_normal_support(layer_slices),
        }
    }

    /// Generate normal (column-based) support structures.
    fn generate_normal_support(
        &self,
        layer_slices: &[(CoordF, CoordF, ExPolygons)],
    ) -> Vec<SupportLayer> {
        // Step 1: Detect overhangs for each layer
        let overhangs = self.detect_overhangs(layer_slices);

        // Step 2: Generate support regions by projecting overhangs down
        let support_regions = self.project_supports(&overhangs, layer_slices);

        // Step 3: Trim supports from model and mark interface layers
        let support_layers = self.finalize_supports(support_regions, &overhangs, layer_slices);

        support_layers
    }

    /// Generate tree support structures using TreeSupport3D.
    fn generate_tree_support(
        &self,
        layer_slices: &[(CoordF, CoordF, ExPolygons)],
    ) -> Vec<SupportLayer> {
        use tree_model_volumes::{TreeModelVolumes, TreeModelVolumesConfig};
        use tree_support_3d::{TreeSupport3D, TreeSupport3DConfig};

        // Step 1: Detect overhangs for each layer
        let overhangs = self.detect_overhangs(layer_slices);

        // Check if there are any overhangs at all
        let has_overhangs = overhangs.iter().any(|o| !o.is_empty());
        if !has_overhangs {
            return layer_slices
                .iter()
                .enumerate()
                .map(|(i, (z, h, _))| SupportLayer::new(i, *z, *h))
                .collect();
        }

        // Step 2: Prepare layer information for TreeModelVolumes
        let layer_heights: Vec<CoordF> = layer_slices.iter().map(|(_, h, _)| *h).collect();
        let z_heights: Vec<CoordF> = layer_slices.iter().map(|(z, _, _)| *z).collect();
        let layer_outlines: Vec<ExPolygons> =
            layer_slices.iter().map(|(_, _, s)| s.clone()).collect();

        // Step 3: Create TreeModelVolumes configuration
        let mut volumes_config = TreeModelVolumesConfig::with_layers(layer_heights, z_heights);
        volumes_config.xy_distance = scale(self.config.xy_distance);
        volumes_config.xy_min_distance = scale(self.config.xy_distance / 2.0);
        volumes_config.support_rests_on_model = !self.config.buildplate_only;
        volumes_config.min_radius = scale(self.config.tree_tip_diameter / 2.0);
        volumes_config.increase_until_radius = scale(self.config.tree_branch_diameter);

        // Step 4: Create TreeModelVolumes with layer outlines
        let volumes = TreeModelVolumes::with_layer_outlines(volumes_config, layer_outlines);

        // Step 5: Create TreeSupport3D configuration
        let tree_config = TreeSupport3DConfig::from_support_config(&self.config);

        // Step 6: Create TreeSupport3D generator and run
        let mut tree_support = TreeSupport3D::new(tree_config, volumes);

        // Convert overhangs from ExPolygons to Vec<Polygon> for TreeSupport3D API
        let overhang_polygons: Vec<Vec<Polygon>> = overhangs
            .iter()
            .map(|expolys| expolys.iter().map(|ex| ex.contour.clone()).collect())
            .collect();

        // Step 7: Generate tree support
        let result = tree_support.generate(&overhang_polygons);

        // Step 8: Post-process - mark interface layers and merge with overhangs
        let mut support_layers = result.layers;

        // Mark interface layers based on proximity to overhangs
        for (i, layer) in support_layers.iter_mut().enumerate() {
            layer.overhang_regions = overhangs.get(i).cloned().unwrap_or_default();

            // Check if this is a top interface layer (close to overhang)
            let is_top_interface = self.config.support_roof
                && !layer.support_regions.is_empty()
                && self.is_near_overhang(i, &overhangs);

            if is_top_interface {
                // Move support regions to interface regions for denser infill
                layer.interface_regions = layer.support_regions.clone();
                layer.support_regions.clear();
                layer.is_interface = true;
            }
        }

        support_layers
    }

    /// Check if a layer is near an overhang (within interface layer count).
    fn is_near_overhang(&self, layer_idx: usize, overhangs: &[ExPolygons]) -> bool {
        let interface_layers = self.config.top_interface_layers;

        for offset in 0..=interface_layers {
            let check_idx = layer_idx + offset;
            if check_idx < overhangs.len() && !overhangs[check_idx].is_empty() {
                return true;
            }
        }
        false
    }

    /// Detect overhang regions for each layer.
    ///
    /// An overhang is an area of the current layer that is not supported by
    /// the layer below (within the overhang angle tolerance).
    fn detect_overhangs(&self, layer_slices: &[(CoordF, CoordF, ExPolygons)]) -> Vec<ExPolygons> {
        let mut overhangs = Vec::with_capacity(layer_slices.len());

        for (i, (_z, height, slices)) in layer_slices.iter().enumerate() {
            if i == 0 {
                // First layer has no overhangs (on build plate)
                overhangs.push(Vec::new());
                continue;
            }

            let (_, _, lower_slices) = &layer_slices[i - 1];

            // Calculate expansion based on overhang angle (in mm)
            let expansion = self.config.overhang_expansion(*height);

            // Expand lower layer by the overhang threshold
            let lower_expanded = if expansion > 0.0 {
                clipper_utils::offset_expolygons(lower_slices, expansion, OffsetJoinType::Round)
            } else {
                lower_slices.clone()
            };

            // Overhangs = current layer - expanded lower layer
            let overhang_expolygons = clipper_utils::difference(slices, &lower_expanded);

            // Filter small overhangs (min_area is in mm²)
            let min_area_scaled =
                self.config.min_area * crate::SCALING_FACTOR * crate::SCALING_FACTOR;
            let layer_overhangs: ExPolygons = overhang_expolygons
                .into_iter()
                .filter(|p| p.area().abs() >= min_area_scaled)
                .collect();

            overhangs.push(layer_overhangs);
        }

        overhangs
    }

    /// Project overhang regions downward to create support regions.
    ///
    /// For each overhang, project it down through all layers until it hits
    /// the build plate or (if not buildplate_only) model geometry.
    fn project_supports(
        &self,
        overhangs: &[ExPolygons],
        layer_slices: &[(CoordF, CoordF, ExPolygons)],
    ) -> Vec<ExPolygons> {
        let num_layers = layer_slices.len();
        let mut support_regions: Vec<ExPolygons> = vec![Vec::new(); num_layers];

        // Process from top to bottom, accumulating support regions
        for layer_idx in (1..num_layers).rev() {
            // Start with overhangs at this layer
            let mut current_support = overhangs[layer_idx].clone();

            // Add any support coming from above
            if layer_idx + 1 < num_layers && !support_regions[layer_idx + 1].is_empty() {
                current_support =
                    clipper_utils::union(&current_support, &support_regions[layer_idx + 1]);
            }

            if current_support.is_empty() {
                continue;
            }

            // Trim support from model geometry
            let (_, _, layer_geometry) = &layer_slices[layer_idx];

            // Expand model by XY distance (in mm)
            let expanded_model = clipper_utils::offset_expolygons(
                layer_geometry,
                self.config.xy_distance,
                OffsetJoinType::Round,
            );

            // Remove model area from support
            let trimmed = clipper_utils::difference(&current_support, &expanded_model);
            support_regions[layer_idx] = trimmed;

            // If buildplate_only, check if support can continue down
            if self.config.buildplate_only && layer_idx > 0 {
                let (_, _, lower_geometry) = &layer_slices[layer_idx - 1];

                // Check what support would land on model below
                let support_on_model =
                    clipper_utils::intersection(&support_regions[layer_idx], lower_geometry);

                if !support_on_model.is_empty() {
                    // Remove parts that would land on model
                    let valid_support =
                        clipper_utils::difference(&support_regions[layer_idx], lower_geometry);
                    support_regions[layer_idx] = valid_support;
                }
            }
        }

        support_regions
    }

    /// Finalize support layers, marking interface regions and trimming.
    fn finalize_supports(
        &self,
        support_regions: Vec<ExPolygons>,
        overhangs: &[ExPolygons],
        layer_slices: &[(CoordF, CoordF, ExPolygons)],
    ) -> Vec<SupportLayer> {
        let num_layers = layer_slices.len();
        let mut support_layers = Vec::with_capacity(num_layers);

        for (i, (z, height, _)) in layer_slices.iter().enumerate() {
            let mut layer = SupportLayer::new(i, *z, *height);
            layer.overhang_regions = overhangs[i].clone();

            if support_regions[i].is_empty() {
                support_layers.push(layer);
                continue;
            }

            // Determine if this is an interface layer
            let is_top_interface = self.config.support_roof
                && !overhangs[i].is_empty()
                && i > 0
                && !support_regions.get(i - 1).map_or(true, |r| r.is_empty());

            let is_bottom_interface = self.config.support_floor
                && i + 1 < num_layers
                && support_regions.get(i + 1).map_or(true, |r| r.is_empty())
                && !support_regions[i].is_empty();

            // Count consecutive interface layers from top
            let layers_from_top = if is_top_interface {
                let mut count = 0;
                for j in (0..i).rev() {
                    if overhangs.get(j + 1).map_or(true, |o| o.is_empty()) {
                        break;
                    }
                    count += 1;
                    if count >= self.config.top_interface_layers {
                        break;
                    }
                }
                count
            } else {
                self.config.top_interface_layers + 1
            };

            layer.is_interface =
                layers_from_top < self.config.top_interface_layers || is_bottom_interface;

            if layer.is_interface {
                layer.interface_regions = support_regions[i].clone();
            } else {
                layer.support_regions = support_regions[i].clone();
            }

            support_layers.push(layer);
        }

        support_layers
    }
}

impl Default for SupportGenerator {
    fn default() -> Self {
        Self::new(SupportConfig::default())
    }
}

/// Detect bridges in a layer by analyzing unsupported areas.
///
/// A bridge is an area that spans a gap between two supported regions,
/// where the print can potentially bridge across without support.
pub fn detect_bridges(
    layer_polygons: &ExPolygons,
    lower_layer_polygons: &ExPolygons,
) -> Vec<Bridge> {
    if layer_polygons.is_empty() || lower_layer_polygons.is_empty() {
        return Vec::new();
    }

    let mut bridges = Vec::new();

    // Find unsupported areas
    let unsupported = clipper_utils::difference(layer_polygons, lower_layer_polygons);

    for expolygon in unsupported {
        // Check if this unsupported area has anchors on both sides
        let bbox = expolygon.bounding_box();
        let width = unscale(bbox.width()) as f64;
        let height = unscale(bbox.height()) as f64;

        // Simple heuristic: if area is longer than wide, it might be a bridge
        let (length, angle) = if width > height {
            (width, 0.0)
        } else {
            (height, 90.0)
        };

        // Check if there are anchors (intersections with lower layer)
        let expanded = clipper_utils::offset_expolygon(&expolygon, 0.1, OffsetJoinType::Round);

        let anchors = clipper_utils::intersection(&expanded, lower_layer_polygons);

        if anchors.len() >= 2 {
            // Has at least two anchor points - could be a bridge
            bridges.push(Bridge {
                angle,
                length,
                area: expolygon,
            });
        }
    }

    bridges
}

/// Calculate the optimal bridge angle for an unsupported area.
///
/// Finds the direction that minimizes the unsupported span length
/// by analyzing potential anchor points on the surrounding geometry.
pub fn calculate_bridge_angle(bridge_area: &ExPolygon, anchors: &[Polygon]) -> f64 {
    if anchors.is_empty() {
        return 0.0;
    }

    let centroid = bridge_area.centroid();
    let mut best_angle = 0.0;
    let mut best_score = f64::MAX;

    // Test angles in 15-degree increments
    for angle_deg in (0..180).step_by(15) {
        let angle_rad = (angle_deg as f64).to_radians();
        let dir = PointF::new(angle_rad.cos(), angle_rad.sin());

        // Calculate distances to anchors in this direction
        let mut min_dist_pos = f64::MAX;
        let mut min_dist_neg = f64::MAX;

        for anchor in anchors {
            for point in anchor.points() {
                let to_point =
                    PointF::new(unscale(point.x - centroid.x), unscale(point.y - centroid.y));
                let dot = to_point.x * dir.x + to_point.y * dir.y;

                if dot > 0.0 {
                    min_dist_pos = min_dist_pos.min(dot);
                } else {
                    min_dist_neg = min_dist_neg.min(-dot);
                }
            }
        }

        // Score is the maximum span in this direction
        let span = min_dist_pos + min_dist_neg;
        if span < best_score && min_dist_pos < f64::MAX && min_dist_neg < f64::MAX {
            best_score = span;
            best_angle = angle_deg as f64;
        }
    }

    best_angle
}

/// Represents a detected bridge area
#[derive(Debug, Clone)]
pub struct Bridge {
    /// The bridge angle (direction of bridging, in degrees)
    pub angle: f64,
    /// Bridge length (span in the bridge direction, in mm)
    pub length: f64,
    /// The bridge area polygon
    pub area: ExPolygon,
}

impl Bridge {
    // Create a new bridge with the given parameters
    pub fn new(angle: f64, length: f64, area: ExPolygon) -> Self {
        Self {
            angle,
            length,
            area,
        }
    }

    /// Get the bridge direction as a unit vector
    pub fn direction(&self) -> PointF {
        let rad = self.angle.to_radians();
        PointF::new(rad.cos(), rad.sin())
    }
}

/// Tree support branch node
#[derive(Debug, Clone)]
pub struct TreeBranch {
    /// Position of this branch node (in scaled coordinates)
    pub position: Point,
    /// Z height of this node (mm)
    pub z: CoordF,
    /// Radius at this node (mm)
    pub radius: CoordF,
    /// Parent branch index (None for root nodes)
    pub parent: Option<usize>,
    /// Child branch indices
    pub children: Vec<usize>,
}

/// Tree support generator for organic branching supports
#[derive(Debug)]
pub struct TreeSupportGenerator {
    config: SupportConfig,
    branches: Vec<TreeBranch>,
}

impl TreeSupportGenerator {
    // Create a new tree support generator
    pub fn new(config: SupportConfig) -> Self {
        Self {
            config,
            branches: Vec::new(),
        }
    }

    /// Generate tree supports from overhang points.
    ///
    /// # Arguments
    /// * `overhang_points` - Vector of (z_height, points) for each layer's overhangs
    /// * `layer_slices` - Model geometry at each layer for collision avoidance
    pub fn generate(
        &mut self,
        overhang_points: &[(CoordF, Vec<Point>)],
        layer_slices: &[(CoordF, CoordF, ExPolygons)],
    ) -> Vec<SupportLayer> {
        self.branches.clear();

        // Create tip branches at overhang points
        for (z, points) in overhang_points {
            for point in points {
                self.branches.push(TreeBranch {
                    position: *point,
                    z: *z,
                    radius: self.config.tree_tip_diameter / 2.0,
                    parent: None,
                    children: Vec::new(),
                });
            }
        }

        // Grow branches downward, avoiding collisions and merging nearby branches
        self.grow_branches(layer_slices);

        // Convert branches to support layers
        self.branches_to_layers(layer_slices)
    }

    /// Grow branches downward toward the build plate
    fn grow_branches(&mut self, layer_slices: &[(CoordF, CoordF, ExPolygons)]) {
        if self.branches.is_empty() || layer_slices.is_empty() {
            return;
        }

        let branch_angle_rad = self.config.tree_branch_angle.to_radians();
        let max_horizontal_per_layer = branch_angle_rad.tan();

        // Sort branches by Z height (highest first)
        let mut branch_indices: Vec<usize> = (0..self.branches.len()).collect();
        branch_indices.sort_by(|&a, &b| {
            self.branches[b]
                .z
                .partial_cmp(&self.branches[a].z)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Process each branch, growing it downward
        for branch_idx in branch_indices {
            let branch = &self.branches[branch_idx];
            if branch.z <= 0.0 {
                continue; // Already at build plate
            }

            // Find the layer below this branch
            let current_layer_idx = layer_slices
                .iter()
                .position(|(z, _, _)| *z >= branch.z)
                .unwrap_or(0);

            if current_layer_idx == 0 {
                continue;
            }

            // Grow down one layer at a time
            let mut current_pos = branch.position;
            let mut current_z = branch.z;
            let mut current_radius = branch.radius;

            for layer_idx in (0..current_layer_idx).rev() {
                let (layer_z, _layer_height, geometry) = &layer_slices[layer_idx];
                let dz = current_z - layer_z;
                let max_move = scale(dz * max_horizontal_per_layer);

                // Try to move toward center/build plate while avoiding model
                let new_pos =
                    self.find_safe_position(current_pos, max_move, current_radius, geometry);

                // Increase radius as we go down
                let radius_growth =
                    (self.config.tree_branch_diameter - self.config.tree_tip_diameter) / 2.0
                        * (dz / current_z);
                current_radius =
                    (current_radius + radius_growth).min(self.config.tree_branch_diameter / 2.0);

                current_pos = new_pos;
                current_z = *layer_z;

                if current_z <= 0.0 {
                    break;
                }
            }
        }
    }

    /// Find a safe position for branch growth that avoids model geometry
    fn find_safe_position(
        &self,
        current: Point,
        max_move: Coord,
        radius: CoordF,
        geometry: &ExPolygons,
    ) -> Point {
        if geometry.is_empty() {
            return current; // No obstacles
        }

        let radius_with_gap = radius + self.config.xy_distance;

        // Check if current position is safe
        let is_safe = |pos: &Point| -> bool {
            for ex in geometry {
                if ex.contains_point(pos) {
                    return false;
                }
                // Check distance from contour
                let dist = ex.distance_to_point(pos);
                if dist < radius_with_gap {
                    return false;
                }
            }
            true
        };

        if is_safe(&current) {
            return current;
        }

        // Try to move away from obstacles
        // Simple approach: try 8 directions
        let directions = [
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1),
            (0, -1),
            (1, -1),
        ];

        for &(dx, dy) in &directions {
            let new_pos = Point::new(current.x + dx * max_move, current.y + dy * max_move);
            if is_safe(&new_pos) {
                return new_pos;
            }
        }

        // Couldn't find safe position, return current
        current
    }

    /// Convert branch structure to support layers
    fn branches_to_layers(
        &self,
        layer_slices: &[(CoordF, CoordF, ExPolygons)],
    ) -> Vec<SupportLayer> {
        let mut support_layers: Vec<SupportLayer> = layer_slices
            .iter()
            .enumerate()
            .map(|(i, (z, h, _))| SupportLayer::new(i, *z, *h))
            .collect();

        // Rasterize branches to each layer
        for branch in &self.branches {
            // Find layer for this branch
            let layer_idx = layer_slices
                .iter()
                .position(|(z, _, _)| (*z - branch.z).abs() < 0.001)
                .unwrap_or(0);

            if layer_idx < support_layers.len() {
                // Create a circle polygon for this branch
                let circle = Polygon::circle(branch.position, scale(branch.radius), 32);
                support_layers[layer_idx]
                    .support_regions
                    .push(ExPolygon::from(circle));
            }
        }

        // Merge overlapping support regions
        for layer in &mut support_layers {
            if layer.support_regions.len() > 1 {
                layer.support_regions = clipper_utils::union_ex(&layer.support_regions);
            }
        }

        support_layers
    }
}

/// Sample points from overhang regions for tree support generation
pub fn sample_overhang_points(overhangs: &[ExPolygon], spacing: CoordF) -> Vec<Point> {
    let mut points = Vec::new();
    let spacing_scaled = scale(spacing);

    for overhang in overhangs {
        let bbox = overhang.bounding_box();

        // Grid sampling within bounding box
        let mut y = bbox.min.y;
        while y <= bbox.max.y {
            let mut x = bbox.min.x;
            while x <= bbox.max.x {
                let point = Point::new(x, y);
                if overhang.contains_point(&point) {
                    points.push(point);
                }
                x += spacing_scaled;
            }
            y += spacing_scaled;
        }
    }

    points
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn make_square_mm(size: CoordF) -> ExPolygon {
        let half = scale(size / 2.0);
        ExPolygon::square(Point::new(half, half), half)
    }

    #[test]
    fn test_support_config_default() {
        let config = SupportConfig::default();
        assert!(!config.enabled);
        assert!((config.overhang_angle - 45.0).abs() < 0.001);
        assert!((config.density - 0.15).abs() < 0.001);
    }

    #[test]
    fn test_support_config_enabled() {
        let config = SupportConfig::enabled();
        assert!(config.enabled);
    }

    #[test]
    fn test_support_config_builder() {
        let config = SupportConfig::enabled()
            .with_density(0.3)
            .with_overhang_angle(60.0)
            .with_pattern(SupportPattern::Honeycomb);

        assert!(config.enabled);
        assert!((config.density - 0.3).abs() < 0.001);
        assert!((config.overhang_angle - 60.0).abs() < 0.001);
        assert_eq!(config.pattern, SupportPattern::Honeycomb);
    }

    #[test]
    fn test_overhang_expansion() {
        let config = SupportConfig::default();
        let expansion = config.overhang_expansion(0.2);
        // At 45 degrees, expansion should equal layer height
        assert!((expansion - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_support_layer_new() {
        let layer = SupportLayer::new(5, 1.0, 0.2);
        assert_eq!(layer.layer_id, 5);
        assert!((layer.z - 1.0).abs() < 0.001);
        assert!((layer.height - 0.2).abs() < 0.001);
        assert!(layer.is_empty());
    }

    #[test]
    fn test_support_generator_disabled() {
        let gen = SupportGenerator::default();
        let slices = vec![
            (0.2, 0.2, vec![make_square_mm(10.0)]),
            (0.4, 0.2, vec![make_square_mm(20.0)]),
        ];

        let layers = gen.generate(&slices);
        assert_eq!(layers.len(), 2);
        assert!(layers[0].is_empty());
        assert!(layers[1].is_empty());
    }

    #[test]
    fn test_support_generator_no_overhang() {
        let gen = SupportGenerator::enabled();
        // Stack of same-size squares - no overhang
        let slices = vec![
            (0.2, 0.2, vec![make_square_mm(10.0)]),
            (0.4, 0.2, vec![make_square_mm(10.0)]),
            (0.6, 0.2, vec![make_square_mm(10.0)]),
        ];

        let layers = gen.generate(&slices);
        assert_eq!(layers.len(), 3);
        // No overhangs, so no support
        for layer in &layers {
            assert!(layer.support_regions.is_empty());
        }
    }

    #[test]
    fn test_support_generator_with_overhang() {
        let gen = SupportGenerator::new(SupportConfig {
            enabled: true,
            min_area: 0.1,
            ..Default::default()
        });

        // Small base, large top - should have overhang
        let small = ExPolygon::from(Polygon::rectangle(
            Point::new(scale(4.0), scale(4.0)),
            Point::new(scale(6.0), scale(6.0)),
        ));
        let large = make_square_mm(10.0);

        let slices = vec![(0.2, 0.2, vec![small]), (0.4, 0.2, vec![large])];

        let layers = gen.generate(&slices);
        assert_eq!(layers.len(), 2);

        // Second layer should have overhang detection
        assert!(!layers[1].overhang_regions.is_empty());
    }

    #[test]
    fn test_bridge_detection_no_bridge() {
        let current = vec![make_square_mm(10.0)];
        let lower = vec![make_square_mm(10.0)];

        let bridges = detect_bridges(&current, &lower);
        assert!(bridges.is_empty());
    }

    #[test]
    fn test_bridge_struct() {
        let bridge = Bridge::new(45.0, 10.0, make_square_mm(5.0));

        assert!((bridge.angle - 45.0).abs() < 0.001);
        assert!((bridge.length - 10.0).abs() < 0.001);

        let dir = bridge.direction();
        let expected_x = 45.0_f64.to_radians().cos();
        let expected_y = 45.0_f64.to_radians().sin();
        assert!((dir.x - expected_x).abs() < 0.001);
        assert!((dir.y - expected_y).abs() < 0.001);
    }

    #[test]
    fn test_tree_branch() {
        let branch = TreeBranch {
            position: Point::new(scale(5.0), scale(5.0)),
            z: 1.0,
            radius: 0.4,
            parent: None,
            children: vec![],
        };

        assert!((branch.z - 1.0).abs() < 0.001);
        assert!((branch.radius - 0.4).abs() < 0.001);
        assert!(branch.parent.is_none());
    }

    #[test]
    fn test_sample_overhang_points() {
        let overhang = make_square_mm(10.0);
        let points = sample_overhang_points(&[overhang.clone()], 2.0);

        // Should have multiple sample points
        assert!(!points.is_empty());

        // All points should be within the overhang
        let overhang_check = make_square_mm(10.0);
        for point in &points {
            assert!(overhang_check.contains_point(point));
        }
    }

    #[test]
    fn test_effective_line_spacing() {
        let config = SupportConfig {
            density: 0.2,
            extrusion_width: 0.4,
            line_spacing: None,
            ..Default::default()
        };

        let spacing = config.effective_line_spacing();
        // spacing = width / density = 0.4 / 0.2 = 2.0
        assert!((spacing - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_effective_line_spacing_override() {
        let config = SupportConfig {
            density: 0.2,
            extrusion_width: 0.4,
            line_spacing: Some(3.0),
            ..Default::default()
        };

        let spacing = config.effective_line_spacing();
        assert!((spacing - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_support_type_variants() {
        assert_eq!(SupportType::Normal, SupportType::Normal);
        assert_ne!(SupportType::Normal, SupportType::Tree);
        assert_ne!(SupportType::Tree, SupportType::Organic);
    }

    #[test]
    fn test_support_pattern_variants() {
        let patterns = [
            SupportPattern::Grid,
            SupportPattern::Lines,
            SupportPattern::Triangles,
            SupportPattern::Honeycomb,
            SupportPattern::Gyroid,
            SupportPattern::Lightning,
        ];

        for (i, p1) in patterns.iter().enumerate() {
            for (j, p2) in patterns.iter().enumerate() {
                if i == j {
                    assert_eq!(p1, p2);
                } else {
                    assert_ne!(p1, p2);
                }
            }
        }
    }

    #[test]
    fn test_tree_support_config() {
        let config = SupportConfig::enabled().tree_support();
        assert!(config.enabled);
        assert_eq!(config.support_type, SupportType::Tree);
    }

    #[test]
    fn test_tree_support_generator_no_overhang() {
        let config = SupportConfig {
            enabled: true,
            support_type: SupportType::Tree,
            ..Default::default()
        };
        let gen = SupportGenerator::new(config);

        // Stack of same-size squares - no overhang
        let slices = vec![
            (0.2, 0.2, vec![make_square_mm(10.0)]),
            (0.4, 0.2, vec![make_square_mm(10.0)]),
            (0.6, 0.2, vec![make_square_mm(10.0)]),
        ];

        let layers = gen.generate(&slices);
        assert_eq!(layers.len(), 3);
        // No overhangs, so no support
        for layer in &layers {
            assert!(layer.support_regions.is_empty());
        }
    }

    #[test]
    fn test_tree_support_generator_with_overhang() {
        let config = SupportConfig {
            enabled: true,
            support_type: SupportType::Tree,
            min_area: 0.1,
            tree_branch_diameter: 2.0,
            tree_tip_diameter: 0.8,
            ..Default::default()
        };
        let gen = SupportGenerator::new(config);

        // Small base, large top - should have overhang
        let small = ExPolygon::from(Polygon::rectangle(
            Point::new(scale(4.0), scale(4.0)),
            Point::new(scale(6.0), scale(6.0)),
        ));
        let large = make_square_mm(10.0);

        let slices = vec![(0.2, 0.2, vec![small]), (0.4, 0.2, vec![large])];

        let layers = gen.generate(&slices);
        assert_eq!(layers.len(), 2);

        // Second layer should have overhang detection
        assert!(!layers[1].overhang_regions.is_empty());
    }

    #[test]
    fn test_organic_support_type() {
        let config = SupportConfig {
            enabled: true,
            support_type: SupportType::Organic,
            ..Default::default()
        };
        let gen = SupportGenerator::new(config);

        // Simple test - should use tree support path
        let slices = vec![
            (0.2, 0.2, vec![make_square_mm(10.0)]),
            (0.4, 0.2, vec![make_square_mm(10.0)]),
        ];

        let layers = gen.generate(&slices);
        assert_eq!(layers.len(), 2);
    }

    #[test]
    fn test_is_near_overhang() {
        let config = SupportConfig {
            enabled: true,
            top_interface_layers: 2,
            ..Default::default()
        };
        let gen = SupportGenerator::new(config);

        // Create overhangs array with overhang only at layer 3
        let overhangs: Vec<ExPolygons> = vec![
            vec![],                    // layer 0
            vec![],                    // layer 1
            vec![],                    // layer 2
            vec![make_square_mm(5.0)], // layer 3 - has overhang
            vec![],                    // layer 4
        ];

        // Layer 1 is within 2 layers of layer 3 overhang
        assert!(gen.is_near_overhang(1, &overhangs));
        // Layer 2 is within 2 layers of layer 3 overhang
        assert!(gen.is_near_overhang(2, &overhangs));
        // Layer 3 has the overhang
        assert!(gen.is_near_overhang(3, &overhangs));
        // Layer 0 is too far from layer 3
        assert!(!gen.is_near_overhang(0, &overhangs));
    }

    #[test]
    fn test_tree_support_buildplate_only() {
        let config = SupportConfig {
            enabled: true,
            support_type: SupportType::Tree,
            buildplate_only: true,
            ..Default::default()
        };
        let gen = SupportGenerator::new(config);

        assert!(gen.config().buildplate_only);

        let slices = vec![
            (0.2, 0.2, vec![make_square_mm(10.0)]),
            (0.4, 0.2, vec![make_square_mm(10.0)]),
        ];

        let layers = gen.generate(&slices);
        assert_eq!(layers.len(), 2);
    }
}
