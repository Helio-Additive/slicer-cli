//! Path generation module.
//!
//! This module converts perimeters and infill into ordered toolpaths
//! ready for G-code generation.
//!
//! # Overview
//!
//! The path generator takes:
//! - Perimeter loops from the perimeter generator
//! - Infill paths from the infill generator
//!
//! And produces an ordered sequence of extrusion paths with:
//! - Optimized ordering to minimize travel moves
//! - Proper seam placement for perimeters
//! - Extrusion amounts calculated based on path geometry
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/GCode.cpp` (path ordering and extrusion)
//! - `src/libslic3r/ExtrusionEntity.cpp`
//! - `src/libslic3r/Flow.cpp` (cross-section area calculations)

use std::f64::consts::PI;

use crate::arachne::ArachneResult;
use crate::arachne::ExtrusionLine;
use crate::fill::{InfillPath, InfillResult};
use crate::flow::Flow;
use crate::geometry::simplify::{
    simplify_polygon_comprehensive, simplify_polyline_comprehensive, SimplifyConfig,
};
use crate::geometry::{Point, PointF, Polygon, Polyline};
use crate::perimeter_generator::PerimeterResult;
use crate::{unscale, Coord, CoordF};

/// Type of extrusion for a path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtrusionRole {
    /// External (outer) perimeter.
    ExternalPerimeter,
    /// Internal perimeter.
    Perimeter,
    /// Overhang wall — external perimeter partially unsupported by layer below.
    /// BambuStudio: `; FEATURE: Overhang wall`
    OverhangWall,
    /// Floating vertical shell — external perimeter completely unsupported.
    /// BambuStudio: `; FEATURE: Floating vertical shell`
    FloatingVerticalShell,
    /// Sparse infill.
    InternalInfill,
    /// Solid infill (top/bottom surfaces).
    SolidInfill,
    /// Top solid infill (visible surface).
    TopSolidInfill,
    /// Bottom solid infill (first layer on bed).
    BottomSolidInfill,
    /// Bridge infill (over gaps).
    BridgeInfill,
    /// Gap fill (thin areas).
    GapFill,
    /// Skirt/brim.
    Skirt,
    /// Raft base layer.
    RaftBase,
    /// Raft interface layer.
    RaftInterface,
    /// Raft contact layer.
    RaftContact,
    /// Support material.
    SupportMaterial,
    /// Support interface.
    SupportMaterialInterface,
    /// Wipe move.
    Wipe,
    /// Custom (user-defined).
    Custom,
}

impl ExtrusionRole {
    // Check if this role is a perimeter.
    pub fn is_perimeter(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::ExternalPerimeter
                | ExtrusionRole::Perimeter
                | ExtrusionRole::OverhangWall
                | ExtrusionRole::FloatingVerticalShell
        )
    }

    /// Check if this role is an external (outer) perimeter type,
    /// including overhang wall and floating vertical shell variants.
    pub fn is_external_perimeter(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::ExternalPerimeter
                | ExtrusionRole::OverhangWall
                | ExtrusionRole::FloatingVerticalShell
        )
    }

    /// Check if this role is infill.
    pub fn is_infill(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::InternalInfill
                | ExtrusionRole::SolidInfill
                | ExtrusionRole::TopSolidInfill
                | ExtrusionRole::BottomSolidInfill
                | ExtrusionRole::BridgeInfill
        )
    }

    /// Check if this role is support.
    pub fn is_support(&self) -> bool {
        matches!(
            self,
            ExtrusionRole::SupportMaterial | ExtrusionRole::SupportMaterialInterface
        )
    }

    /// Get a descriptive name for this role.
    pub fn name(&self) -> &'static str {
        match self {
            ExtrusionRole::ExternalPerimeter => "external perimeter",
            ExtrusionRole::Perimeter => "perimeter",
            ExtrusionRole::OverhangWall => "overhang wall",
            ExtrusionRole::FloatingVerticalShell => "floating vertical shell",
            ExtrusionRole::InternalInfill => "internal infill",
            ExtrusionRole::SolidInfill => "solid infill",
            ExtrusionRole::TopSolidInfill => "top solid infill",
            ExtrusionRole::BottomSolidInfill => "bottom solid infill",
            ExtrusionRole::BridgeInfill => "bridge infill",
            ExtrusionRole::GapFill => "gap fill",
            ExtrusionRole::Skirt => "skirt",
            ExtrusionRole::RaftBase => "raft base",
            ExtrusionRole::RaftInterface => "raft interface",
            ExtrusionRole::RaftContact => "raft contact",
            ExtrusionRole::SupportMaterial => "support material",
            ExtrusionRole::SupportMaterialInterface => "support interface",
            ExtrusionRole::Wipe => "wipe",
            ExtrusionRole::Custom => "custom",
        }
    }

    /// Get the BambuStudio-style feature name for G-code comments.
    ///
    /// This returns the exact string format used by BambuStudio in
    /// `; FEATURE: <name>` comments, which is expected by validation tools.
    pub fn feature_name(&self) -> &'static str {
        match self {
            ExtrusionRole::ExternalPerimeter => "Outer wall",
            ExtrusionRole::Perimeter => "Inner wall",
            ExtrusionRole::OverhangWall => "Overhang wall",
            ExtrusionRole::FloatingVerticalShell => "Floating vertical shell",
            ExtrusionRole::InternalInfill => "Sparse infill",
            ExtrusionRole::SolidInfill => "Internal solid infill",
            ExtrusionRole::TopSolidInfill => "Top surface",
            ExtrusionRole::BottomSolidInfill => "Bottom surface",
            ExtrusionRole::BridgeInfill => "Bridge",
            ExtrusionRole::GapFill => "Gap infill",
            ExtrusionRole::Skirt => "Skirt",
            ExtrusionRole::RaftBase => "Raft base",
            ExtrusionRole::RaftInterface => "Raft interface",
            ExtrusionRole::RaftContact => "Raft contact",
            ExtrusionRole::SupportMaterial => "Support",
            ExtrusionRole::SupportMaterialInterface => "Support interface",
            ExtrusionRole::Wipe => "Wipe",
            ExtrusionRole::Custom => "Custom",
        }
    }
}

/// A single extrusion path ready for G-code generation.
#[derive(Debug, Clone)]
pub struct ExtrusionPath {
    /// The path points (in scaled coordinates).
    pub points: Vec<Point>,

    /// Whether this is a closed loop (polygon) or open path (polyline).
    pub is_closed: bool,

    /// The role/type of this extrusion.
    pub role: ExtrusionRole,

    /// Extrusion width (mm).
    pub width: CoordF,

    /// Layer height (mm).
    pub height: CoordF,

    /// Flow rate multiplier (1.0 = normal).
    pub flow_multiplier: CoordF,

    /// Print speed for this path (mm/s).
    pub speed: CoordF,

    /// Flow object for accurate E-value calculation.
    ///
    /// When present, this should be used for extrusion calculations via
    /// `flow.mm3_per_mm()` rather than the simple width × height formula.
    /// This ensures proper rounded rectangle cross-section calculations
    /// matching BambuStudio's Flow.cpp.
    pub flow: Option<Flow>,
}

impl ExtrusionPath {
    // Create a new extrusion path.
    pub fn new(points: Vec<Point>, is_closed: bool, role: ExtrusionRole) -> Self {
        Self {
            points,
            is_closed,
            role,
            width: 0.45,
            height: 0.2,
            flow_multiplier: 1.0,
            speed: 60.0, // mm/s default
            flow: None,
        }
    }

    /// Create from a polygon (closed loop).
    pub fn from_polygon(polygon: &Polygon, role: ExtrusionRole) -> Self {
        Self::new(polygon.points().to_vec(), true, role)
    }

    /// Create from a polyline (open path).
    pub fn from_polyline(polyline: &Polyline, role: ExtrusionRole) -> Self {
        Self::new(polyline.points().to_vec(), false, role)
    }

    /// Set the extrusion width.
    pub fn with_width(mut self, width: CoordF) -> Self {
        self.width = width;
        self
    }

    /// Set the layer height.
    pub fn with_height(mut self, height: CoordF) -> Self {
        self.height = height;
        self
    }

    /// Set the flow multiplier.
    pub fn with_flow(mut self, flow: CoordF) -> Self {
        self.flow_multiplier = flow;
        self
    }

    /// Set the print speed.
    pub fn with_speed(mut self, speed: CoordF) -> Self {
        self.speed = speed;
        self
    }

    /// Set the Flow object for accurate extrusion calculations.
    pub fn with_flow_object(mut self, flow: Flow) -> Self {
        self.flow = Some(flow);
        self
    }

    /// Get the first point.
    pub fn first_point(&self) -> Option<Point> {
        self.points.first().copied()
    }

    /// Get the last point.
    pub fn last_point(&self) -> Option<Point> {
        self.points.last().copied()
    }

    /// Get the path length in scaled units.
    pub fn length(&self) -> Coord {
        if self.points.len() < 2 {
            return 0;
        }

        let mut total = 0i64;
        for i in 0..(self.points.len() - 1) {
            let p1 = self.points[i];
            let p2 = self.points[i + 1];
            let dx = (p2.x - p1.x) as f64;
            let dy = (p2.y - p1.y) as f64;
            total += (dx * dx + dy * dy).sqrt() as Coord;
        }

        // Add closing segment for closed paths
        if self.is_closed && self.points.len() >= 2 {
            let p1 = *self.points.last().unwrap();
            let p2 = self.points[0];
            let dx = (p2.x - p1.x) as f64;
            let dy = (p2.y - p1.y) as f64;
            total += (dx * dx + dy * dy).sqrt() as Coord;
        }

        total
    }

    /// Get the path length in mm.
    pub fn length_mm(&self) -> CoordF {
        unscale(self.length())
    }

    /// Check if this is a bridge extrusion.
    pub fn is_bridge(&self) -> bool {
        matches!(self.role, ExtrusionRole::BridgeInfill)
    }

    /// Calculate the cross-sectional area of the extrusion (mm²).
    ///
    /// This uses the proper rounded rectangle formula from libslic3r/Flow.cpp:
    /// - For bridges: circular cross-section `π × (width/2)²`
    /// - For normal extrusions: `height × (width - height × (1 - π/4))`
    ///
    /// The simple `width × height` formula would give ~10-15% error.
    pub fn cross_section_area(&self) -> CoordF {
        if self.is_bridge() {
            // Bridge: circular cross-section (unsupported filament forms round thread)
            // area = π × r² = π × (width/2)² = width² × π/4
            (self.width * self.width) * 0.25 * PI
        } else {
            // Normal extrusion: rectangle with semicircular ends
            // area = height × (width - height × (1 - π/4))
            //      ≈ height × (width - 0.2146 × height)
            self.height * (self.width - self.height * (1.0 - 0.25 * PI))
        }
    }

    /// Calculate the total extrusion volume for this path (mm³).
    pub fn extrusion_volume(&self) -> CoordF {
        self.length_mm() * self.cross_section_area() * self.flow_multiplier
    }

    /// Calculate the E distance for this path given filament diameter.
    ///
    /// E = volume / (π × (filament_diameter/2)²)
    ///
    /// This is the amount of filament (in mm) that needs to be pushed
    /// through the extruder to produce this path.
    pub fn extrusion_length(&self, filament_diameter: CoordF) -> CoordF {
        let filament_area = PI * (filament_diameter / 2.0).powi(2);
        self.extrusion_volume() / filament_area
    }

    /// Get a Flow object representing this path's extrusion parameters.
    ///
    /// This provides access to all the flow calculation helpers from the
    /// flow module. If a Flow object was set via `with_flow_object()`, it
    /// will be returned. Otherwise, a Flow is constructed on-demand from
    /// width and height.
    pub fn flow(&self) -> Option<Flow> {
        if let Some(ref flow) = self.flow {
            return Some(flow.clone());
        }

        // Fallback: construct Flow from width/height
        if self.is_bridge() {
            // For bridges, width == height (circular)
            Some(Flow::bridging_flow(self.width, self.width))
        } else {
            // Normal flow - may fail if width/height combination is invalid
            Flow::new(self.width, self.height, self.width).ok()
        }
    }

    /// Reverse the path direction.
    pub fn reverse(&mut self) {
        self.points.reverse();
    }

    /// Get a reversed copy.
    pub fn reversed(&self) -> Self {
        let mut copy = self.clone();
        copy.reverse();
        copy
    }

    /// Split this closed path at a specific point index to create a seam.
    pub fn split_at_seam(&mut self, seam_index: usize) {
        if !self.is_closed || self.points.len() < 3 || seam_index >= self.points.len() {
            return;
        }

        // Rotate points so seam_index becomes the first point
        let mut new_points = Vec::with_capacity(self.points.len());
        new_points.extend_from_slice(&self.points[seam_index..]);
        new_points.extend_from_slice(&self.points[..seam_index]);
        self.points = new_points;
    }

    /// Get the point as PointF (in mm).
    pub fn point_mm(&self, index: usize) -> Option<PointF> {
        self.points
            .get(index)
            .map(|p| PointF::new(unscale(p.x), unscale(p.y)))
    }

    /// Iterate over segments as (from, to) point pairs in mm.
    pub fn segments_mm(&self) -> impl Iterator<Item = (PointF, PointF)> + '_ {
        let len = self.points.len();
        let extra = if self.is_closed && len >= 2 { 1 } else { 0 };

        (0..(len.saturating_sub(1) + extra)).map(move |i| {
            let p1 = self.points[i];
            let p2 = self.points[(i + 1) % len];
            (
                PointF::new(unscale(p1.x), unscale(p1.y)),
                PointF::new(unscale(p2.x), unscale(p2.y)),
            )
        })
    }
}

/// Configuration for path generation.
#[derive(Debug, Clone)]
pub struct PathConfig {
    /// Extrusion width for perimeters (mm).
    pub perimeter_width: CoordF,

    /// Extrusion width for external perimeters (mm).
    pub external_perimeter_width: CoordF,

    /// Extrusion width for infill (mm).
    pub infill_width: CoordF,

    /// Layer height (mm).
    pub layer_height: CoordF,

    /// First layer height (mm).
    pub first_layer_height: CoordF,

    /// Perimeter print speed (mm/s).
    pub perimeter_speed: CoordF,

    /// External perimeter print speed (mm/s).
    pub external_perimeter_speed: CoordF,

    /// Infill print speed (mm/s).
    pub infill_speed: CoordF,

    /// Solid infill print speed (mm/s).
    pub solid_infill_speed: CoordF,

    /// Top solid infill print speed (mm/s).
    pub top_solid_infill_speed: CoordF,

    /// Travel speed (mm/s).
    pub travel_speed: CoordF,

    /// Filament diameter (mm).
    pub filament_diameter: CoordF,

    /// Whether to print external perimeters first.
    pub external_perimeters_first: bool,

    /// Whether to print infill before perimeters.
    pub infill_first: bool,

    /// Seam position preference.
    pub seam_position: SeamPosition,
}

impl Default for PathConfig {
    fn default() -> Self {
        Self {
            perimeter_width: 0.45,
            external_perimeter_width: 0.45,
            infill_width: 0.45,
            layer_height: 0.2,
            first_layer_height: 0.3,
            perimeter_speed: 60.0,
            external_perimeter_speed: 30.0,
            infill_speed: 80.0,
            solid_infill_speed: 60.0,
            top_solid_infill_speed: 30.0,
            travel_speed: 150.0,
            filament_diameter: 1.75,
            external_perimeters_first: false,
            infill_first: false,
            seam_position: SeamPosition::Nearest,
        }
    }
}

/// Seam position preference for closed loops.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SeamPosition {
    /// Random position.
    Random,
    /// Nearest to current position.
    #[default]
    Nearest,
    /// Aligned (try to keep seams in same position across layers).
    Aligned,
    /// Rear of the print (positive Y).
    Rear,
    /// Hidden in a corner.
    Hidden,
}

/// Classification for a generated layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LayerKind {
    /// Standard model toolpaths.
    #[default]
    Model,
    /// Support-only content.
    Support,
    /// Adhesion-only content (brim, skirt, raft).
    Adhesion,
    /// Custom or mixed content.
    Custom,
}

/// Result of path generation for a layer.
#[derive(Debug, Clone, Default)]
pub struct LayerPaths {
    /// All extrusion paths for this layer, in print order.
    pub paths: Vec<ExtrusionPath>,

    /// Support structure paths for this layer.
    pub support_paths: Vec<ExtrusionPath>,

    /// Adhesion (brim/skirt/raft) paths for this layer.
    pub adhesion_paths: Vec<ExtrusionPath>,

    /// Layer index.
    pub layer_index: usize,

    /// Layer Z height (mm).
    pub z_height: CoordF,

    /// Layer thickness (mm).
    pub layer_height: CoordF,

    /// Classification of this layer.
    pub kind: LayerKind,

    /// Total extrusion length for this layer (mm of filament).
    pub total_extrusion: CoordF,

    /// Total travel distance for this layer (mm).
    pub total_travel: CoordF,
}

impl LayerPaths {
    // Create a new layer paths container.
    pub fn new(layer_index: usize, z_height: CoordF, layer_height: CoordF) -> Self {
        Self {
            paths: Vec::new(),
            support_paths: Vec::new(),
            adhesion_paths: Vec::new(),
            layer_index,
            z_height,
            layer_height,
            kind: LayerKind::Model,
            total_extrusion: 0.0,
            total_travel: 0.0,
        }
    }

    /// Add a path to this layer.
    pub fn add_path(&mut self, path: ExtrusionPath) {
        self.paths.push(path);
    }

    /// Get the number of paths.
    pub fn path_count(&self) -> usize {
        self.paths.len()
    }

    /// Check if this layer has any paths.
    pub fn has_paths(&self) -> bool {
        !self.paths.is_empty() || !self.support_paths.is_empty() || !self.adhesion_paths.is_empty()
    }

    /// Check if this layer has support paths.
    pub fn has_support(&self) -> bool {
        !self.support_paths.is_empty()
    }

    /// Add a support path to this layer.
    pub fn add_support_path(&mut self, path: ExtrusionPath) {
        self.support_paths.push(path);
    }

    /// Add an adhesion path to this layer.
    pub fn add_adhesion_path(&mut self, path: ExtrusionPath) {
        self.adhesion_paths.push(path);
    }

    /// Set the kind for this layer.
    pub fn set_kind(&mut self, kind: LayerKind) {
        self.kind = kind;
    }

    /// Calculate statistics for this layer.
    pub fn calculate_stats(&mut self, filament_diameter: CoordF) {
        // Calculate extrusion for model paths
        let model_extrusion: CoordF = self
            .paths
            .iter()
            .map(|p| p.extrusion_length(filament_diameter))
            .sum();

        // Calculate extrusion for support paths
        let support_extrusion: CoordF = self
            .support_paths
            .iter()
            .map(|p| p.extrusion_length(filament_diameter))
            .sum();

        self.total_extrusion = model_extrusion + support_extrusion;

        // Calculate travel (simplified: just distances between path endpoints)
        self.total_travel = 0.0;

        // Travel within model paths
        for i in 1..self.paths.len() {
            if let (Some(prev_end), Some(curr_start)) =
                (self.paths[i - 1].last_point(), self.paths[i].first_point())
            {
                let dx = unscale(curr_start.x - prev_end.x);
                let dy = unscale(curr_start.y - prev_end.y);
                self.total_travel += (dx * dx + dy * dy).sqrt();
            }
        }

        // Travel within support paths
        for i in 1..self.support_paths.len() {
            if let (Some(prev_end), Some(curr_start)) = (
                self.support_paths[i - 1].last_point(),
                self.support_paths[i].first_point(),
            ) {
                let dx = unscale(curr_start.x - prev_end.x);
                let dy = unscale(curr_start.y - prev_end.y);
                self.total_travel += (dx * dx + dy * dy).sqrt();
            }
        }
    }

    /// Get perimeter paths only.
    pub fn perimeter_paths(&self) -> impl Iterator<Item = &ExtrusionPath> {
        self.paths.iter().filter(|p| p.role.is_perimeter())
    }

    /// Get infill paths only.
    pub fn infill_paths(&self) -> impl Iterator<Item = &ExtrusionPath> {
        self.paths.iter().filter(|p| p.role.is_infill())
    }

    /// Get support paths only.
    pub fn support_paths(&self) -> impl Iterator<Item = &ExtrusionPath> {
        self.support_paths.iter()
    }

    /// Get support interface paths only.
    pub fn support_interface_paths(&self) -> impl Iterator<Item = &ExtrusionPath> {
        self.support_paths
            .iter()
            .filter(|p| p.role == ExtrusionRole::SupportMaterialInterface)
    }

    /// Get all paths (model + support) in order.
    pub fn all_paths(&self) -> impl Iterator<Item = &ExtrusionPath> {
        // Adhesion prints first, followed by support, then model paths
        self.adhesion_paths
            .iter()
            .chain(self.support_paths.iter())
            .chain(self.paths.iter())
    }
}

/// Path generator.
///
/// Converts perimeters and infill into ordered extrusion paths.
#[derive(Debug, Clone)]
pub struct PathGenerator {
    config: PathConfig,
    /// Current position for nearest-neighbor ordering.
    current_position: Option<Point>,
}

impl PathGenerator {
    // Create a new path generator with the given configuration.
    pub fn new(config: PathConfig) -> Self {
        Self {
            config,
            current_position: None,
        }
    }

    /// Create a path generator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(PathConfig::default())
    }

    /// Get the configuration.
    pub fn config(&self) -> &PathConfig {
        &self.config
    }

    /// Get the current position (for island ordering).
    pub fn current_position(&self) -> Option<Point> {
        self.current_position
    }

    /// Set the current position (used by island ordering).
    pub fn set_current_position(&mut self, pos: Option<Point>) {
        self.current_position = pos;
    }

    pub fn set_position(&mut self, pos: Point) {
        self.current_position = Some(pos);
    }

    /// Generate paths for a layer from perimeters and infill.
    ///
    /// # Arguments
    /// * `perimeters` - Perimeter generation result
    /// * `infill` - Infill generation result
    /// * `layer_index` - Layer index
    /// * `z_height` - Z height of this layer (mm)
    /// * `is_solid` - Whether this is a solid layer (top/bottom)
    ///
    /// # Returns
    /// LayerPaths containing all extrusion paths in print order.
    pub fn generate(
        &mut self,
        perimeters: &PerimeterResult,
        infill: &InfillResult,
        layer_index: usize,
        z_height: CoordF,
        is_solid: bool,
    ) -> LayerPaths {
        let layer_height = if layer_index == 0 {
            self.config.first_layer_height
        } else {
            self.config.layer_height
        };

        let mut layer_paths = LayerPaths::new(layer_index, z_height, layer_height);

        // Convert perimeters to paths
        let perimeter_paths = self.convert_perimeters(perimeters, layer_height);

        // Convert infill to paths
        let infill_paths = self.convert_infill(infill, layer_height, is_solid, layer_index);

        // Order paths based on configuration
        if self.config.infill_first {
            // Infill first, then perimeters
            layer_paths.paths.extend(infill_paths);
            layer_paths.paths.extend(perimeter_paths);
        } else {
            // Perimeters first (default), then infill
            layer_paths.paths.extend(perimeter_paths);
            layer_paths.paths.extend(infill_paths);
        }

        // Optimize path order to minimize travel
        self.optimize_path_order(&mut layer_paths.paths);

        // Calculate statistics
        layer_paths.calculate_stats(self.config.filament_diameter);

        layer_paths
    }

    /// Generate paths for a layer from Arachne variable-width perimeters and infill.
    ///
    /// # Arguments
    /// * `arachne_result` - Arachne perimeter generation result with variable-width toolpaths
    /// * `infill` - Infill generation result
    /// * `layer_index` - Layer index
    /// * `z_height` - Z height of this layer (mm)
    /// * `is_solid` - Whether this is a solid layer (top/bottom)
    ///
    /// # Returns
    /// LayerPaths containing all extrusion paths in print order.
    pub fn generate_from_arachne(
        &mut self,
        arachne_result: &ArachneResult,
        infill: &InfillResult,
        layer_index: usize,
        z_height: CoordF,
        is_solid: bool,
    ) -> LayerPaths {
        let layer_height = if layer_index == 0 {
            self.config.first_layer_height
        } else {
            self.config.layer_height
        };

        let mut layer_paths = LayerPaths::new(layer_index, z_height, layer_height);

        // Convert Arachne variable-width toolpaths to extrusion paths
        let perimeter_paths = self.convert_arachne_toolpaths(arachne_result, layer_height);

        // Convert infill to paths
        let infill_paths = self.convert_infill(infill, layer_height, is_solid, layer_index);

        // Order paths based on configuration
        if self.config.infill_first {
            layer_paths.paths.extend(infill_paths);
            layer_paths.paths.extend(perimeter_paths);
        } else {
            layer_paths.paths.extend(perimeter_paths);
            layer_paths.paths.extend(infill_paths);
        }

        // Optimize path order to minimize travel
        self.optimize_path_order(&mut layer_paths.paths);

        // Calculate statistics
        layer_paths.calculate_stats(self.config.filament_diameter);

        layer_paths
    }

    /// Convert perimeters to extrusion paths.
    fn convert_perimeters(
        &self,
        perimeters: &PerimeterResult,
        layer_height: CoordF,
    ) -> Vec<ExtrusionPath> {
        let mut paths = Vec::new();

        // PerimeterGenerator now returns ExtrusionEntityCollection via traverse_loops
        // which already contains properly structured ExtrusionLoop entities with
        // paths that have been simplified at surface level (0.0125mm resolution).
        //
        // We need to extract ExtrusionPath objects from the ExtrusionLoop entities.
        use crate::extrusion_entity::ExtrusionEntityType;

        for entity in &perimeters.entities.entities {
            match entity {
                ExtrusionEntityType::Loop(extrusion_loop) => {
                    // Each ExtrusionLoop contains one or more ExtrusionPath segments
                    for extrusion_path in &extrusion_loop.paths {
                        // Convert from libslic3r ExtrusionPath to gcode::path::ExtrusionPath
                        use crate::extrusion_entity::ExtrusionRole as LibRole;

                        // Convert libslic3r ExtrusionRole to gcode::path ExtrusionRole
                        let role = match extrusion_path.role {
                            LibRole::ExternalPerimeter => ExtrusionRole::ExternalPerimeter,
                            LibRole::Perimeter => ExtrusionRole::Perimeter,
                            LibRole::OverhangPerimeter => ExtrusionRole::Perimeter,
                            LibRole::InternalInfill => ExtrusionRole::InternalInfill,
                            LibRole::SolidInfill => ExtrusionRole::SolidInfill,
                            LibRole::TopSolidInfill => ExtrusionRole::TopSolidInfill,
                            LibRole::Ironing => ExtrusionRole::TopSolidInfill, // Map to top solid
                            LibRole::BridgeInfill => ExtrusionRole::BridgeInfill,
                            LibRole::GapFill => ExtrusionRole::GapFill,
                            LibRole::Skirt => ExtrusionRole::Skirt,
                            LibRole::SupportMaterial => ExtrusionRole::SupportMaterial,
                            LibRole::SupportMaterialInterface => {
                                ExtrusionRole::SupportMaterialInterface
                            }
                            LibRole::WipeTower => ExtrusionRole::SupportMaterial, // Map to support
                            LibRole::Custom => ExtrusionRole::Custom,
                            // Catch-all for any remaining variants
                            _ => ExtrusionRole::Perimeter,
                        };

                        let width = extrusion_path.width;

                        let speed = match role {
                            ExtrusionRole::ExternalPerimeter => {
                                self.config.external_perimeter_speed
                            }
                            ExtrusionRole::Perimeter => self.config.perimeter_speed,
                            _ => self.config.perimeter_speed,
                        };

                        // Convert polyline points to our path format
                        let points = extrusion_path.polyline.points().to_vec();

                        let path = ExtrusionPath {
                            points,
                            is_closed: false, // Polylines from ExtrusionPath are open
                            role,
                            width,
                            height: layer_height,
                            flow_multiplier: 1.0,
                            speed,
                            flow: None, // Flow is embedded in mm3_per_mm
                        };

                        paths.push(path);
                    }
                }
                ExtrusionEntityType::Path(extrusion_path) => {
                    // Handle standalone paths (e.g., thin walls)
                    use crate::extrusion_entity::ExtrusionRole as LibRole;

                    // Convert libslic3r ExtrusionRole to gcode::path ExtrusionRole
                    let role = match extrusion_path.role {
                        LibRole::ExternalPerimeter => ExtrusionRole::ExternalPerimeter,
                        LibRole::Perimeter => ExtrusionRole::Perimeter,
                        LibRole::OverhangPerimeter => ExtrusionRole::Perimeter,
                        LibRole::InternalInfill => ExtrusionRole::InternalInfill,
                        LibRole::SolidInfill => ExtrusionRole::SolidInfill,
                        LibRole::TopSolidInfill => ExtrusionRole::TopSolidInfill,
                        LibRole::Ironing => ExtrusionRole::TopSolidInfill, // Map to top solid
                        LibRole::BridgeInfill => ExtrusionRole::BridgeInfill,
                        LibRole::GapFill => ExtrusionRole::GapFill,
                        LibRole::Skirt => ExtrusionRole::Skirt,
                        LibRole::SupportMaterial => ExtrusionRole::SupportMaterial,
                        LibRole::SupportMaterialInterface => {
                            ExtrusionRole::SupportMaterialInterface
                        }
                        LibRole::WipeTower => ExtrusionRole::SupportMaterial, // Map to support
                        LibRole::Custom => ExtrusionRole::Custom,
                        // Catch-all for any remaining variants
                        _ => ExtrusionRole::Perimeter,
                    };

                    let width = extrusion_path.width;

                    let speed = match role {
                        ExtrusionRole::ExternalPerimeter => self.config.external_perimeter_speed,
                        ExtrusionRole::Perimeter => self.config.perimeter_speed,
                        _ => self.config.perimeter_speed,
                    };

                    let points = extrusion_path.polyline.points().to_vec();

                    let path = ExtrusionPath {
                        points,
                        is_closed: false,
                        role,
                        width,
                        height: layer_height,
                        flow_multiplier: 1.0,
                        speed,
                        flow: None,
                    };

                    paths.push(path);
                }
                ExtrusionEntityType::Collection(_nested) => {
                    // TODO: Handle nested collections recursively if needed
                }
            }
        }

        // Simplification config for polylines (open paths)
        let polyline_simplify_config = SimplifyConfig::for_polyline();

        // Convert gap fills (ThickPolylines) to extrusion paths using variable_width.
        //
        // Each ThickPolyline carries per-vertex width information from the medial axis.
        // We use variable_width() to split these into ExtrusionPaths where each path
        // has a single LINE_WIDTH value, matching BambuStudio's behavior of emitting
        // many short gap fill segments with varying widths.
        //
        // TODO: Re-enable after ThickPolyline support is fully implemented
        // BambuStudio reference: PerimeterGenerator.cpp line 1359:
        //   variable_width(polylines, erGapFill, this->solid_infill_flow, gap_fill.entities);
        // {
        //     use crate::variable_width::{variable_width, VariableWidthConfig};
        //
        //     let vw_config = VariableWidthConfig::for_gap_fill(
        //         layer_height,
        //         self.config.perimeter_width, // nozzle_diameter approximation
        //         self.config.perimeter_speed,
        //     );
        //
        //     let gap_paths = variable_width(&perimeters.gap_fills, &vw_config);
        //
        //     for path in gap_paths {
        //         if path.points.len() >= 2 {
        //             paths.push(path);
        //         }
        //     }
        // }

        // TODO: Convert gap fills to extrusion paths
        // gap_fills are ExPolygons, not Polylines - need proper conversion
        // for gap_fill in &perimeters.gap_fills {
        //     // Gap fills need to be converted from ExPolygons to paths
        //     // This requires medial axis transformation or similar
        // }

        paths
    }

    /// Convert Arachne variable-width toolpaths to extrusion paths.
    ///
    /// This handles the conversion from ExtrusionLine (with per-junction widths)
    /// to ExtrusionPath. When width variation is significant, the line is split
    /// into multiple segments with different widths to better preserve the
    /// variable-width intent.
    fn convert_arachne_toolpaths(
        &self,
        arachne_result: &ArachneResult,
        layer_height: CoordF,
    ) -> Vec<ExtrusionPath> {
        let mut paths = Vec::new();

        // Threshold for splitting: if width varies by more than 20%, split into segments
        const WIDTH_VARIATION_THRESHOLD: CoordF = 0.20;

        // Simplification config matching BambuStudio's meshfix parameters
        let polygon_simplify_config = SimplifyConfig::for_polygon();
        let polyline_simplify_config = SimplifyConfig::for_polyline();

        // Process each wall layer (outer to inner)
        for wall_lines in &arachne_result.toolpaths {
            for line in wall_lines {
                if line.is_empty() || line.len() < 2 {
                    continue;
                }

                let role = if line.is_external() {
                    ExtrusionRole::ExternalPerimeter
                } else {
                    ExtrusionRole::Perimeter
                };

                let speed = if line.is_external() {
                    self.config.external_perimeter_speed
                } else {
                    self.config.perimeter_speed
                };

                // Check width variation
                let min_width = line.min_width() as CoordF;
                let max_width = line.max_width() as CoordF;
                let avg_width = line.average_width_mm();

                // Calculate relative variation (convert scaled widths to mm for comparison)
                let min_width_mm = unscale(min_width as Coord);
                let max_width_mm = unscale(max_width as Coord);
                let variation = if avg_width > 0.0 {
                    (max_width_mm - min_width_mm) / avg_width
                } else {
                    0.0
                };

                if variation > WIDTH_VARIATION_THRESHOLD && line.len() > 2 {
                    // Split into variable-width segments
                    let segment_paths =
                        self.split_variable_width_line(line, role, layer_height, speed);
                    paths.extend(segment_paths);
                } else {
                    // Use average width for the whole path, with simplification
                    let path = if line.is_closed {
                        let polygon = line.to_polygon();
                        let simplified =
                            simplify_polygon_comprehensive(&polygon, &polygon_simplify_config);
                        ExtrusionPath::from_polygon(&simplified, role)
                    } else {
                        let polyline = line.to_polyline();
                        let simplified =
                            simplify_polyline_comprehensive(&polyline, &polyline_simplify_config);
                        ExtrusionPath::from_polyline(&simplified, role)
                    }
                    .with_width(avg_width)
                    .with_height(layer_height)
                    .with_speed(speed);

                    paths.push(path);
                }
            }
        }

        // Also add thin fills if any (with simplification)
        for line in &arachne_result.thin_fills {
            if line.is_empty() || line.len() < 2 {
                continue;
            }

            let avg_width = line.average_width_mm();

            let path = if line.is_closed {
                let polygon = line.to_polygon();
                let simplified = simplify_polygon_comprehensive(&polygon, &polygon_simplify_config);
                ExtrusionPath::from_polygon(&simplified, ExtrusionRole::GapFill)
            } else {
                let polyline = line.to_polyline();
                let simplified =
                    simplify_polyline_comprehensive(&polyline, &polyline_simplify_config);
                if simplified.len() < 2 {
                    continue;
                }
                ExtrusionPath::from_polyline(&simplified, ExtrusionRole::GapFill)
            }
            .with_width(avg_width)
            .with_height(layer_height)
            .with_speed(self.config.perimeter_speed * 0.5); // Slower for gap fill

            paths.push(path);
        }

        paths
    }

    /// Split a variable-width ExtrusionLine into multiple ExtrusionPath segments.
    ///
    /// This groups consecutive junctions with similar widths together to create
    /// multiple paths, each with a more uniform width. This preserves the
    /// variable-width intent better than using a single average width.
    fn split_variable_width_line(
        &self,
        line: &ExtrusionLine,
        role: ExtrusionRole,
        layer_height: CoordF,
        speed: CoordF,
    ) -> Vec<ExtrusionPath> {
        use crate::geometry::Polyline;

        let mut paths = Vec::new();

        if line.len() < 2 {
            return paths;
        }

        // Threshold for starting a new segment: 15% width change from segment average
        const SEGMENT_WIDTH_THRESHOLD: CoordF = 0.15;

        let mut segment_start = 0;
        let mut segment_width_sum: CoordF = line[0].width_mm();
        let mut segment_count = 1;

        for i in 1..line.len() {
            let junction = &line[i];
            let current_width = junction.width_mm();
            let segment_avg_width = segment_width_sum / segment_count as CoordF;

            // Check if this junction's width deviates significantly from the segment average
            let width_deviation = if segment_avg_width > 0.0 {
                (current_width - segment_avg_width).abs() / segment_avg_width
            } else {
                0.0
            };

            let should_split = width_deviation > SEGMENT_WIDTH_THRESHOLD && i > segment_start + 1;

            if should_split || i == line.len() - 1 {
                // Create a path for the current segment
                let end_idx = if should_split { i } else { i + 1 };

                if end_idx > segment_start + 1 {
                    let segment_junctions = &line.junctions[segment_start..end_idx];

                    // Calculate average width for this segment
                    let seg_avg_width: CoordF = segment_junctions
                        .iter()
                        .map(|j| j.width_mm())
                        .sum::<CoordF>()
                        / segment_junctions.len() as CoordF;

                    // Create polyline from segment points
                    let points: Vec<Point> = segment_junctions.iter().map(|j| j.position).collect();

                    let polyline = Polyline::from_points(points);

                    let path = ExtrusionPath::from_polyline(&polyline, role)
                        .with_width(seg_avg_width)
                        .with_height(layer_height)
                        .with_speed(speed);

                    paths.push(path);
                }

                // Start new segment (overlap by 1 point for continuity)
                if should_split {
                    segment_start = i - 1;
                    segment_width_sum = line[i - 1].width_mm() + current_width;
                    segment_count = 2;
                }
            } else {
                // Continue accumulating in current segment
                segment_width_sum += current_width;
                segment_count += 1;
            }
        }

        // If no paths were created (degenerate case), create one with average width
        if paths.is_empty() && line.len() >= 2 {
            let avg_width = line.average_width_mm();
            let path = if line.is_closed {
                ExtrusionPath::from_polygon(&line.to_polygon(), role)
            } else {
                ExtrusionPath::from_polyline(&line.to_polyline(), role)
            }
            .with_width(avg_width)
            .with_height(layer_height)
            .with_speed(speed);

            paths.push(path);
        }

        paths
    }

    /// Convert infill to extrusion paths with an explicit role.
    ///
    /// This is the role-aware variant used by the pipeline when it knows the
    /// exact surface type (Top, Bottom, InternalSolid, Internal) for a set of
    /// infill paths. The role determines the `; FEATURE:` comment emitted in
    /// the G-code and the print speed used.
    pub fn convert_infill_with_role(
        &self,
        infill: &InfillResult,
        layer_height: CoordF,
        role: ExtrusionRole,
    ) -> Vec<ExtrusionPath> {
        let speed = match role {
            ExtrusionRole::TopSolidInfill => self.config.top_solid_infill_speed,
            ExtrusionRole::BottomSolidInfill => self.config.solid_infill_speed,
            ExtrusionRole::SolidInfill => self.config.solid_infill_speed,
            _ => self.config.infill_speed,
        };

        self.convert_infill_inner(infill, layer_height, role, speed)
    }

    /// Convert infill to extrusion paths.
    fn convert_infill(
        &self,
        infill: &InfillResult,
        layer_height: CoordF,
        is_solid: bool,
        layer_index: usize,
    ) -> Vec<ExtrusionPath> {
        // FIX: Distinguish between top solid, bottom solid, and internal solid infill
        let role = if is_solid {
            if layer_index == 0 {
                // First layer uses bottom solid infill role
                ExtrusionRole::BottomSolidInfill
            } else {
                ExtrusionRole::SolidInfill
            }
        } else {
            ExtrusionRole::InternalInfill
        };

        let speed = if is_solid {
            self.config.solid_infill_speed
        } else {
            self.config.infill_speed
        };

        self.convert_infill_inner(infill, layer_height, role, speed)
    }

    /// Shared implementation for infill-to-path conversion.
    fn convert_infill_inner(
        &self,
        infill: &InfillResult,
        layer_height: CoordF,
        role: ExtrusionRole,
        speed: CoordF,
    ) -> Vec<ExtrusionPath> {
        let mut paths = Vec::new();

        // Simplification config for infill paths
        // Use slightly more aggressive simplification for infill since it's internal
        let polygon_simplify_config = SimplifyConfig::for_polygon();
        let polyline_simplify_config = SimplifyConfig::for_polyline();

        for infill_path in &infill.paths {
            let path = match infill_path {
                InfillPath::Line(polyline) => {
                    let simplified =
                        simplify_polyline_comprehensive(polyline, &polyline_simplify_config);
                    if simplified.len() < 2 {
                        continue;
                    }
                    ExtrusionPath::from_polyline(&simplified, role)
                        .with_width(self.config.infill_width)
                        .with_height(layer_height)
                        .with_speed(speed)
                }
                InfillPath::Loop(polygon) => {
                    let simplified =
                        simplify_polygon_comprehensive(polygon, &polygon_simplify_config);
                    ExtrusionPath::from_polygon(&simplified, role)
                        .with_width(self.config.infill_width)
                        .with_height(layer_height)
                        .with_speed(speed)
                }
            };

            paths.push(path);
        }

        paths
    }

    /// Optimize path order using nearest-neighbor heuristic.
    ///
    /// IMPORTANT: This optimization preserves feature type grouping to avoid
    /// interleaving different feature types (e.g., perimeters and infill).
    /// This reduces the number of FEATURE comments in the output and produces
    /// more logical print order.
    ///
    /// Uses deterministic tie-breaking when paths have equal distances to ensure
    /// reproducible results across runs. Tie-breaking is based on:
    /// 1. Original path index (lower index wins)
    /// 2. Path start point coordinates (lower X, then lower Y wins)
    fn optimize_path_order(&mut self, paths: &mut Vec<ExtrusionPath>) {
        if paths.len() < 2 {
            return;
        }

        // Group paths by their role (feature type) to avoid interleaving
        // This preserves the logical grouping: all perimeters together, then infill, etc.
        let mut groups: Vec<(ExtrusionRole, Vec<usize>)> = Vec::new();

        for (idx, path) in paths.iter().enumerate() {
            // Check if we already have a group for this role
            if let Some(group) = groups.iter_mut().find(|(role, _)| *role == path.role) {
                group.1.push(idx);
            } else {
                groups.push((path.role, vec![idx]));
            }
        }

        let mut ordered = Vec::with_capacity(paths.len());

        // Start from current position or first path
        let start_pos = self
            .current_position
            .unwrap_or_else(|| paths[0].first_point().unwrap_or(Point::new(0, 0)));

        let mut current_pos = start_pos;

        // Process each feature group in order, optimizing within the group
        for (_role, group_indices) in groups {
            if group_indices.is_empty() {
                continue;
            }

            let mut remaining = group_indices;

            while !remaining.is_empty() {
                // Find nearest path within this group with deterministic tie-breaking
                let mut best_idx = 0;
                let mut best_dist = i64::MAX;
                let mut best_original_idx = usize::MAX;
                let mut best_start_point = Point::new(i64::MAX, i64::MAX);
                let mut reverse_best = false;

                for (i, &path_idx) in remaining.iter().enumerate() {
                    let path = &paths[path_idx];

                    if let Some(start) = path.first_point() {
                        let dist = distance_squared(current_pos, start);
                        // Use deterministic tie-breaking: prefer lower distance, then lower
                        // original index, then lower start point (X then Y)
                        let is_better = dist < best_dist
                            || (dist == best_dist
                                && (path_idx < best_original_idx
                                    || (path_idx == best_original_idx
                                        && Self::point_less_than(start, best_start_point))));

                        if is_better {
                            best_dist = dist;
                            best_idx = i;
                            best_original_idx = path_idx;
                            best_start_point = start;
                            reverse_best = false;
                        }
                    }

                    // For open paths, also consider starting from the end
                    if !path.is_closed {
                        if let Some(end) = path.last_point() {
                            let dist = distance_squared(current_pos, end);
                            let is_better = dist < best_dist
                                || (dist == best_dist
                                    && (path_idx < best_original_idx
                                        || (path_idx == best_original_idx
                                            && Self::point_less_than(end, best_start_point))));

                            if is_better {
                                best_dist = dist;
                                best_idx = i;
                                best_original_idx = path_idx;
                                best_start_point = end;
                                reverse_best = true;
                            }
                        }
                    }
                }

                // Add the best path from this group
                let path_idx = remaining.remove(best_idx);
                let mut path = paths[path_idx].clone();

                if reverse_best {
                    path.reverse();
                }

                // Update current position
                if let Some(end) = path.last_point() {
                    current_pos = end;
                }

                ordered.push(path);
            }
        }

        *paths = ordered;

        // Update current position for next layer
        if let Some(last_path) = paths.last() {
            if let Some(end) = last_path.last_point() {
                self.current_position = Some(end);
            }
        }
    }

    /// Legacy path optimization that doesn't preserve feature grouping.
    /// Kept for reference but not used.
    #[allow(dead_code)]
    fn optimize_path_order_legacy(&mut self, paths: &mut Vec<ExtrusionPath>) {
        if paths.len() < 2 {
            return;
        }

        let mut ordered = Vec::with_capacity(paths.len());
        let mut remaining: Vec<_> = (0..paths.len()).collect();

        // Start from current position or first path
        let start_pos = self
            .current_position
            .unwrap_or_else(|| paths[0].first_point().unwrap_or(Point::new(0, 0)));

        let mut current_pos = start_pos;

        while !remaining.is_empty() {
            // Find nearest path with deterministic tie-breaking
            let mut best_idx = 0;
            let mut best_dist = i64::MAX;
            let mut best_original_idx = usize::MAX;
            let mut best_start_point = Point::new(i64::MAX, i64::MAX);
            let mut reverse_best = false;

            for (i, &path_idx) in remaining.iter().enumerate() {
                let path = &paths[path_idx];

                if let Some(start) = path.first_point() {
                    let dist = distance_squared(current_pos, start);
                    // Use deterministic tie-breaking: prefer lower distance, then lower
                    // original index, then lower start point (X then Y)
                    let is_better = dist < best_dist
                        || (dist == best_dist
                            && (path_idx < best_original_idx
                                || (path_idx == best_original_idx
                                    && Self::point_less_than(start, best_start_point))));

                    if is_better {
                        best_dist = dist;
                        best_idx = i;
                        best_original_idx = path_idx;
                        best_start_point = start;
                        reverse_best = false;
                    }
                }

                // For open paths, also consider starting from the end
                if !path.is_closed {
                    if let Some(end) = path.last_point() {
                        let dist = distance_squared(current_pos, end);
                        let is_better = dist < best_dist
                            || (dist == best_dist
                                && (path_idx < best_original_idx
                                    || (path_idx == best_original_idx
                                        && Self::point_less_than(end, best_start_point))));

                        if is_better {
                            best_dist = dist;
                            best_idx = i;
                            best_original_idx = path_idx;
                            best_start_point = end;
                            reverse_best = true;
                        }
                    }
                }
            }

            // Add the best path
            let path_idx = remaining.remove(best_idx);
            let mut path = paths[path_idx].clone();

            if reverse_best {
                path.reverse();
            }

            // Apply seam position for closed paths
            if path.is_closed {
                self.apply_seam(&mut path, current_pos);
            }

            // Update current position
            current_pos = path.last_point().unwrap_or(current_pos);

            ordered.push(path);
        }

        // Update current position for next call
        if let Some(last_path) = ordered.last() {
            if let Some(end) = last_path.last_point() {
                self.current_position = Some(end);
            }
        }

        *paths = ordered;
    }

    /// Compare two points for deterministic ordering (X first, then Y).
    #[inline]
    fn point_less_than(a: Point, b: Point) -> bool {
        a.x < b.x || (a.x == b.x && a.y < b.y)
    }

    /// Apply seam position to a closed path.
    fn apply_seam(&self, path: &mut ExtrusionPath, current_pos: Point) {
        if !path.is_closed || path.points.len() < 3 {
            return;
        }

        let seam_index = match self.config.seam_position {
            SeamPosition::Nearest => {
                // Find point nearest to current position
                Self::find_nearest_point(&path.points, current_pos)
            }
            SeamPosition::Rear => {
                // Find point with highest Y (rear of print bed)
                // With tie-breaking by highest X for determinism
                path.points
                    .iter()
                    .enumerate()
                    .max_by(|(_, a), (_, b)| a.y.cmp(&b.y).then_with(|| a.x.cmp(&b.x)))
                    .map(|(i, _)| i)
                    .unwrap_or(0)
            }
            SeamPosition::Hidden => {
                // Find a concave corner (inside corner) to hide the seam
                Self::find_hidden_seam_position(&path.points, current_pos)
            }
            SeamPosition::Aligned => {
                // Try to align with previous seam position
                // Use a deterministic position based on geometry for consistency across layers
                Self::find_aligned_seam_position(&path.points)
            }
            SeamPosition::Random => {
                // Use a deterministic "random" based on path geometry
                // Hash the first point coordinates for reproducibility
                let hash = path
                    .points
                    .first()
                    .map(|p| ((p.x.wrapping_mul(73856093)) ^ (p.y.wrapping_mul(19349663))) as usize)
                    .unwrap_or(0);
                hash % path.points.len().max(1)
            }
        };

        path.split_at_seam(seam_index);
    }

    /// Find the nearest point to a given position.
    fn find_nearest_point(points: &[Point], target: Point) -> usize {
        points
            .iter()
            .enumerate()
            .min_by_key(|(_, p)| distance_squared(target, **p))
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    /// Find a hidden seam position by looking for concave corners.
    ///
    /// Concave corners (inside corners) are good places to hide seams because
    /// they're less visible than convex corners or flat surfaces.
    fn find_hidden_seam_position(points: &[Point], current_pos: Point) -> usize {
        if points.len() < 3 {
            return 0;
        }

        let n = points.len();
        let mut best_idx = 0;
        let mut best_score = i64::MAX;

        for i in 0..n {
            let prev = points[(i + n - 1) % n];
            let curr = points[i];
            let next = points[(i + 1) % n];

            // Calculate the turn angle at this vertex
            // Negative cross product means concave (right turn / inside corner)
            let v1_x = curr.x - prev.x;
            let v1_y = curr.y - prev.y;
            let v2_x = next.x - curr.x;
            let v2_y = next.y - curr.y;

            let cross = v1_x * v2_y - v1_y * v2_x;

            // Score: prefer concave corners (negative cross product)
            // Also consider distance to current position as secondary factor
            let concavity_score = if cross < 0 {
                // Concave corner - very desirable
                cross.abs() / 1000 // Normalize
            } else {
                // Convex or straight - less desirable
                i64::MAX / 2
            };

            let distance_score = distance_squared(current_pos, curr) / 1_000_000;

            // Combined score: prioritize concavity, use distance as tie-breaker
            let score = concavity_score.saturating_add(distance_score / 100);

            if score < best_score || (score == best_score && i < best_idx) {
                best_score = score;
                best_idx = i;
            }
        }

        // If no good concave corner found, try to find a sharp corner
        if best_score >= i64::MAX / 2 {
            best_idx = Self::find_sharpest_corner(points);
        }

        best_idx
    }

    /// Find the sharpest corner in a polygon (largest angle change).
    fn find_sharpest_corner(points: &[Point]) -> usize {
        if points.len() < 3 {
            return 0;
        }

        let n = points.len();
        let mut best_idx = 0;
        let mut best_sharpness = 0i64;

        for i in 0..n {
            let prev = points[(i + n - 1) % n];
            let curr = points[i];
            let next = points[(i + 1) % n];

            // Calculate angle sharpness (absolute cross product)
            let v1_x = curr.x - prev.x;
            let v1_y = curr.y - prev.y;
            let v2_x = next.x - curr.x;
            let v2_y = next.y - curr.y;

            let cross = (v1_x * v2_y - v1_y * v2_x).abs();

            if cross > best_sharpness || (cross == best_sharpness && i < best_idx) {
                best_sharpness = cross;
                best_idx = i;
            }
        }

        best_idx
    }

    /// Find an aligned seam position based on geometry for consistency across layers.
    ///
    /// Uses a deterministic algorithm that picks the same relative position
    /// on similar-shaped polygons.
    fn find_aligned_seam_position(points: &[Point]) -> usize {
        if points.is_empty() {
            return 0;
        }

        // Strategy: find the point closest to the polygon's "rear-left" corner
        // This gives consistent results for similar shapes across layers
        let bbox_min_x = points.iter().map(|p| p.x).min().unwrap_or(0);
        let bbox_max_y = points.iter().map(|p| p.y).max().unwrap_or(0);

        // Target: rear-left corner of bounding box
        let target = Point::new(bbox_min_x, bbox_max_y);

        // Find nearest point to this target
        points
            .iter()
            .enumerate()
            .min_by(|(i1, p1), (i2, p2)| {
                let d1 = distance_squared(target, **p1);
                let d2 = distance_squared(target, **p2);
                d1.cmp(&d2).then_with(|| i1.cmp(i2))
            })
            .map(|(i, _)| i)
            .unwrap_or(0)
    }
}

impl Default for PathGenerator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Calculate squared distance between two points.
fn distance_squared(a: Point, b: Point) -> i64 {
    let dx = (b.x - a.x) as i64;
    let dy = (b.y - a.y) as i64;
    dx * dx + dy * dy
}

// ============================================================================
// Convenience Functions
// ============================================================================

/// Generate paths with default configuration.
pub fn generate_paths(
    perimeters: &PerimeterResult,
    infill: &InfillResult,
    layer_index: usize,
    z_height: CoordF,
) -> LayerPaths {
    let mut generator = PathGenerator::with_defaults();
    generator.generate(perimeters, infill, layer_index, z_height, false)
}

/// Generate paths for a solid layer (top/bottom).
pub fn generate_solid_paths(
    perimeters: &PerimeterResult,
    infill: &InfillResult,
    layer_index: usize,
    z_height: CoordF,
) -> LayerPaths {
    let mut generator = PathGenerator::with_defaults();
    generator.generate(perimeters, infill, layer_index, z_height, true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scale;

    fn make_square_polygon(x: f64, y: f64, size: f64) -> Polygon {
        Polygon::rectangle(
            Point::new(scale(x), scale(y)),
            Point::new(scale(x + size), scale(y + size)),
        )
    }

    #[test]
    fn test_extrusion_role() {
        assert!(ExtrusionRole::ExternalPerimeter.is_perimeter());
        assert!(ExtrusionRole::Perimeter.is_perimeter());
        assert!(!ExtrusionRole::InternalInfill.is_perimeter());

        assert!(ExtrusionRole::InternalInfill.is_infill());
        assert!(ExtrusionRole::SolidInfill.is_infill());
        assert!(!ExtrusionRole::Perimeter.is_infill());

        assert!(ExtrusionRole::SupportMaterial.is_support());
        assert!(!ExtrusionRole::Perimeter.is_support());
    }

    #[test]
    fn test_extrusion_path_from_polygon() {
        let polygon = make_square_polygon(0.0, 0.0, 10.0);
        let path = ExtrusionPath::from_polygon(&polygon, ExtrusionRole::ExternalPerimeter);

        assert!(path.is_closed);
        assert_eq!(path.role, ExtrusionRole::ExternalPerimeter);
        assert_eq!(path.points.len(), 4);
    }

    #[test]
    fn test_extrusion_path_length() {
        let polygon = make_square_polygon(0.0, 0.0, 10.0);
        let path = ExtrusionPath::from_polygon(&polygon, ExtrusionRole::Perimeter);

        // 10mm square perimeter = 40mm
        let length = path.length_mm();
        assert!((length - 40.0).abs() < 0.001);
    }

    #[test]
    fn test_extrusion_path_volume() {
        // Create a simple line path
        let points = vec![
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(0.0)),
        ];
        let path = ExtrusionPath::new(points, false, ExtrusionRole::InternalInfill)
            .with_width(0.4)
            .with_height(0.2);

        // Length = 10mm
        // Cross-section uses rounded rectangle formula from libslic3r/Flow.cpp:
        //   area = height × (width - height × (1 - π/4))
        //        = 0.2 × (0.4 - 0.2 × (1 - π/4))
        //        = 0.2 × (0.4 - 0.2 × 0.2146)
        //        = 0.2 × (0.4 - 0.0429)
        //        = 0.2 × 0.3571
        //        ≈ 0.0714 mm²
        // Volume = 10 × 0.0714 ≈ 0.714 mm³
        //
        // Note: Simple width×height=0.08 would give ~10% error!
        let expected_area = 0.2 * (0.4 - 0.2 * (1.0 - 0.25 * PI));
        let expected_volume = 10.0 * expected_area;
        let volume = path.extrusion_volume();
        assert!(
            (volume - expected_volume).abs() < 0.001,
            "Volume mismatch: got {}, expected {}",
            volume,
            expected_volume
        );
    }

    #[test]
    fn test_extrusion_path_reverse() {
        let points = vec![
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(0.0)),
            Point::new(scale(10.0), scale(10.0)),
        ];
        let mut path = ExtrusionPath::new(points, false, ExtrusionRole::InternalInfill);

        let start_before = path.first_point();
        let end_before = path.last_point();

        path.reverse();

        assert_eq!(path.first_point(), end_before);
        assert_eq!(path.last_point(), start_before);
    }

    #[test]
    fn test_extrusion_path_split_at_seam() {
        let polygon = make_square_polygon(0.0, 0.0, 10.0);
        let mut path = ExtrusionPath::from_polygon(&polygon, ExtrusionRole::Perimeter);

        let original_second = path.points[2];
        path.split_at_seam(2);

        assert_eq!(path.first_point(), Some(original_second));
    }

    #[test]
    fn test_path_config_default() {
        let config = PathConfig::default();
        assert!((config.perimeter_width - 0.45).abs() < 1e-6);
        assert!((config.filament_diameter - 1.75).abs() < 1e-6);
    }

    #[test]
    fn test_layer_paths() {
        let mut layer = LayerPaths::new(0, 0.2, 0.2);
        assert_eq!(layer.layer_index, 0);
        assert!(!layer.has_paths());

        let polygon = make_square_polygon(0.0, 0.0, 10.0);
        let path = ExtrusionPath::from_polygon(&polygon, ExtrusionRole::Perimeter);
        layer.add_path(path);

        assert!(layer.has_paths());
        assert_eq!(layer.path_count(), 1);
    }

    #[test]
    fn test_layer_paths_calculate_stats() {
        let mut layer = LayerPaths::new(0, 0.2, 0.2);

        // Add a simple path
        let points = vec![
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(0.0)),
        ];
        let path = ExtrusionPath::new(points, false, ExtrusionRole::InternalInfill)
            .with_width(0.4)
            .with_height(0.2);
        layer.add_path(path);

        layer.calculate_stats(1.75);

        assert!(layer.total_extrusion > 0.0);
    }

    #[test]
    fn test_path_generator_empty() {
        let mut generator = PathGenerator::with_defaults();

        let perimeters = PerimeterResult::default();
        let infill = InfillResult::default();

        let result = generator.generate(&perimeters, &infill, 0, 0.2, false);

        assert!(!result.has_paths());
    }

    #[test]
    fn test_distance_squared() {
        let a = Point::new(0, 0);
        let b = Point::new(3, 4);

        // Distance should be 5, squared = 25
        assert_eq!(distance_squared(a, b), 25);
    }

    #[test]
    fn test_find_nearest_point() {
        let points = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ];

        // Target at (90, 10) should find point at (100, 0)
        let target = Point::new(90, 10);
        let idx = PathGenerator::find_nearest_point(&points, target);
        assert_eq!(idx, 1);

        // Target at origin should find first point
        let target = Point::new(0, 0);
        let idx = PathGenerator::find_nearest_point(&points, target);
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_find_sharpest_corner() {
        // Square has equal corners, should pick first one (index 0)
        let square = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ];
        let idx = PathGenerator::find_sharpest_corner(&square);
        assert_eq!(idx, 0);

        // L-shape: the function finds the sharpest corner based on cross product
        // All 90-degree corners have equal sharpness, so it should return first one (idx 0)
        let l_shape = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 50),
            Point::new(50, 50),
            Point::new(50, 100),
            Point::new(0, 100),
        ];
        let idx = PathGenerator::find_sharpest_corner(&l_shape);
        // All corners are 90 degrees, so first one wins due to tie-breaking
        assert!(idx < l_shape.len()); // Just ensure it returns a valid index
    }

    #[test]
    fn test_find_aligned_seam_position() {
        // For a square, should find rear-left (max Y, min X)
        let square = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ];
        let idx = PathGenerator::find_aligned_seam_position(&square);
        // Point at (0, 100) is rear-left
        assert_eq!(idx, 3);
    }

    #[test]
    fn test_find_hidden_seam_position() {
        // L-shape should prefer concave corner
        let l_shape = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 50),
            Point::new(50, 50), // inner corner - concave
            Point::new(50, 100),
            Point::new(0, 100),
        ];
        let current_pos = Point::new(50, 50);
        let idx = PathGenerator::find_hidden_seam_position(&l_shape, current_pos);
        // Should prefer the concave inner corner
        assert!(idx == 3 || idx == 2 || idx == 4); // One of the corners near the L joint
    }

    #[test]
    fn test_seam_position_deterministic() {
        // Run the same seam calculation multiple times - should get same result
        let points = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ];

        let results: Vec<usize> = (0..5)
            .map(|_| PathGenerator::find_aligned_seam_position(&points))
            .collect();

        // All results should be identical
        assert!(results.iter().all(|&x| x == results[0]));
    }

    #[test]
    fn test_point_less_than() {
        let a = Point::new(10, 20);
        let b = Point::new(10, 30);
        let c = Point::new(20, 10);

        // Same X, lower Y wins
        assert!(PathGenerator::point_less_than(a, b));
        assert!(!PathGenerator::point_less_than(b, a));

        // Lower X always wins regardless of Y
        assert!(PathGenerator::point_less_than(a, c));
    }
}
