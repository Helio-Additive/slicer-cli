//! Multi-material integration module.
//!
//! This module wires together the Tool Ordering and Wipe Tower components
//! to coordinate multi-extruder printing operations.
//!
//! # Overview
//!
//! When printing with multiple filaments, the slicing pipeline needs to:
//! 1. Determine which extruder to use for each feature on each layer
//! 2. Calculate the optimal order of tool changes to minimize flush/purge waste
//! 3. Generate wipe tower geometry to prime/purge during tool changes
//! 4. Insert tool change G-code at appropriate points
//!
//! This module provides the `MultiMaterialCoordinator` which orchestrates
//! these operations.
//!
//! # BambuStudio Reference
//!
//! This corresponds to the interaction between:
//! - `GCode/ToolOrdering.cpp` - Tool change sequence optimization
//! - `GCode/WipeTower.cpp` - Wipe tower geometry generation
//! - `GCode.cpp` - G-code generation with multi-material support

use super::tool_ordering::{
    CustomGCodeItem, ExtrusionRoleType, FilamentChangeMode, FilamentChangeStats, FlushMatrix,
    ToolOrdering, ToolOrderingConfig,
};
use super::wipe_tower::{FilamentParameters, ToolChangeResult, WipeTower, WipeTowerConfig};
use crate::geometry::{ExPolygon, Point, Polygon};
use crate::{scale, CoordF};

/// Configuration for multi-material printing.
#[derive(Debug, Clone)]
pub struct MultiMaterialConfig {
    /// Number of filaments/extruders
    pub num_filaments: usize,

    /// Nozzle diameter for each extruder (mm)
    pub nozzle_diameters: Vec<f64>,

    /// Filament density for each extruder (g/cm³)
    pub filament_densities: Vec<f64>,

    /// Whether each filament is soluble (for supports)
    pub filament_soluble: Vec<bool>,

    /// Whether each filament is designated for support material
    pub filament_is_support: Vec<bool>,

    /// Filament type names (e.g., "PLA", "PETG", "TPU")
    pub filament_types: Vec<String>,

    /// Filament color strings (e.g., "#FF0000" for red)
    pub filament_colors: Vec<String>,

    /// Flush/purge volume matrix (from_filament -> to_filament)
    pub flush_matrix: FlushMatrix,

    /// Per-filament flush multipliers
    pub flush_multipliers: Vec<f32>,

    /// Whether to generate a prime/wipe tower
    pub enable_prime_tower: bool,

    /// Wipe tower position X (mm)
    pub wipe_tower_x: f64,

    /// Wipe tower position Y (mm)
    pub wipe_tower_y: f64,

    /// Wipe tower width (mm)
    pub wipe_tower_width: f64,

    /// Wipe tower rotation angle (degrees)
    pub wipe_tower_rotation: f64,

    /// Wipe tower brim width (mm)
    pub wipe_tower_brim_width: f64,

    /// Whether infill should be printed first (before perimeters)
    pub infill_first: bool,

    /// Default extruder for perimeters (0-indexed)
    pub default_perimeter_extruder: usize,

    /// Default extruder for infill (0-indexed)
    pub default_infill_extruder: usize,

    /// Default extruder for solid infill (0-indexed)
    pub default_solid_infill_extruder: usize,

    /// Default extruder for support material (0-indexed)
    pub default_support_extruder: usize,

    /// Default extruder for support interface (0-indexed)
    pub default_support_interface_extruder: usize,

    /// Travel speed for non-printing moves (mm/s)
    pub travel_speed: f64,

    /// Maximum print speed (mm/s)
    pub max_print_speed: f64,

    /// First layer speed multiplier
    pub first_layer_speed_factor: f64,
}

impl Default for MultiMaterialConfig {
    fn default() -> Self {
        Self {
            num_filaments: 1,
            nozzle_diameters: vec![0.4],
            filament_densities: vec![1.24], // PLA density
            filament_soluble: vec![false],
            filament_is_support: vec![false],
            filament_types: vec!["PLA".to_string()],
            filament_colors: vec!["#FFFFFF".to_string()],
            flush_matrix: FlushMatrix::new(1, 50.0),
            flush_multipliers: vec![1.0f32],
            enable_prime_tower: false,
            wipe_tower_x: 180.0,
            wipe_tower_y: 180.0,
            wipe_tower_width: 60.0,
            wipe_tower_rotation: 0.0,
            wipe_tower_brim_width: 2.0,
            infill_first: false,
            default_perimeter_extruder: 0,
            default_infill_extruder: 0,
            default_solid_infill_extruder: 0,
            default_support_extruder: 0,
            default_support_interface_extruder: 0,
            travel_speed: 150.0,
            max_print_speed: 200.0,
            first_layer_speed_factor: 0.5,
        }
    }
}

impl MultiMaterialConfig {
    // Create a new multi-material configuration.
    pub fn new(num_filaments: usize) -> Self {
        let mut config = Self::default();
        config.num_filaments = num_filaments;
        config.nozzle_diameters = vec![0.4; num_filaments];
        config.filament_densities = vec![1.24; num_filaments];
        config.filament_soluble = vec![false; num_filaments];
        config.filament_is_support = vec![false; num_filaments];
        config.filament_types = vec!["PLA".to_string(); num_filaments];
        config.filament_colors = (0..num_filaments)
            .map(|i| default_filament_color(i))
            .collect();
        config.flush_matrix = FlushMatrix::new(num_filaments, 50.0);
        config.flush_multipliers = vec![1.0f32; num_filaments];

        // Enable prime tower if more than one filament
        config.enable_prime_tower = num_filaments > 1;

        config
    }

    /// Check if multi-material mode is active.
    pub fn is_multi_material(&self) -> bool {
        self.num_filaments > 1
    }

    /// Get flush volume between two filaments.
    pub fn get_flush_volume(&self, from: usize, to: usize) -> f32 {
        let base_volume = self.flush_matrix.get(from, to);
        let multiplier = self.flush_multipliers.get(to).copied().unwrap_or(1.0);
        base_volume * multiplier
    }

    /// Convert to ToolOrderingConfig.
    pub fn to_tool_ordering_config(&self) -> ToolOrderingConfig {
        ToolOrderingConfig {
            num_filaments: self.num_filaments,
            nozzle_diameters: self.nozzle_diameters.clone(),
            filament_densities: self.filament_densities.clone(),
            filament_soluble: self.filament_soluble.clone(),
            filament_is_support: self.filament_is_support.clone(),
            filament_types: self.filament_types.clone(),
            filament_colors: self.filament_colors.clone(),
            flush_matrix: self.flush_matrix.clone(),
            flush_multipliers: self.flush_multipliers.clone(),
            enable_prime_tower: self.enable_prime_tower,
            infill_first: self.infill_first,
            ..Default::default()
        }
    }

    /// Convert to WipeTowerConfig.
    pub fn to_wipe_tower_config(&self, max_print_z: f64) -> WipeTowerConfig {
        WipeTowerConfig {
            pos_x: self.wipe_tower_x as f32,
            pos_y: self.wipe_tower_y as f32,
            width: self.wipe_tower_width as f32,
            rotation_angle: self.wipe_tower_rotation as f32,
            brim_width: self.wipe_tower_brim_width as f32,
            travel_speed: self.travel_speed as f32,
            max_speed: self.max_print_speed as f32,
            first_layer_speed: (self.max_print_speed * self.first_layer_speed_factor) as f32,
            height: max_print_z as f32,
            is_multi_extruder: self.num_filaments > 1,
            ..Default::default()
        }
    }

    /// Create FilamentParameters for each filament.
    pub fn to_filament_params(&self) -> Vec<FilamentParameters> {
        (0..self.num_filaments)
            .map(|i| FilamentParameters {
                material: self.filament_types.get(i).cloned().unwrap_or_default(),
                is_soluble: self.filament_soluble.get(i).copied().unwrap_or(false),
                is_support: self.filament_is_support.get(i).copied().unwrap_or(false),
                nozzle_diameter: self.nozzle_diameters.get(i).copied().unwrap_or(0.4) as f32,
                ..Default::default()
            })
            .collect()
    }
}

/// Default filament color for a given index.
fn default_filament_color(index: usize) -> String {
    const COLORS: &[&str] = &[
        "#FFFFFF", // White
        "#000000", // Black
        "#FF0000", // Red
        "#00FF00", // Green
        "#0000FF", // Blue
        "#FFFF00", // Yellow
        "#FF00FF", // Magenta
        "#00FFFF", // Cyan
        "#FFA500", // Orange
        "#800080", // Purple
    ];
    COLORS
        .get(index % COLORS.len())
        .unwrap_or(&"#FFFFFF")
        .to_string()
}

/// Layer information for multi-material coordination.
#[derive(Debug, Clone)]
pub struct MultiMaterialLayer {
    /// Layer index
    pub layer_idx: usize,

    /// Z height (mm)
    pub print_z: CoordF,

    /// Layer height (mm)
    pub layer_height: CoordF,

    /// Extruders used on this layer (in order)
    pub extruders: Vec<usize>,

    /// Tool changes for this layer
    pub tool_changes: Vec<ToolChange>,

    /// Wipe tower results for this layer (if any)
    pub wipe_tower_results: Vec<ToolChangeResult>,

    /// Whether this layer has a skirt
    pub has_skirt: bool,

    /// Custom G-code events for this layer
    pub custom_gcodes: Vec<CustomGCodeItem>,
}

impl MultiMaterialLayer {
    // Create a new multi-material layer.
    pub fn new(layer_idx: usize, print_z: CoordF, layer_height: CoordF) -> Self {
        Self {
            layer_idx,
            print_z,
            layer_height,
            extruders: Vec::new(),
            tool_changes: Vec::new(),
            wipe_tower_results: Vec::new(),
            has_skirt: false,
            custom_gcodes: Vec::new(),
        }
    }

    /// Check if this layer has any tool changes.
    pub fn has_tool_changes(&self) -> bool {
        !self.tool_changes.is_empty()
    }

    /// Get the number of tool changes on this layer.
    pub fn num_tool_changes(&self) -> usize {
        self.tool_changes.len()
    }
}

/// A single tool change event.
#[derive(Debug, Clone)]
pub struct ToolChange {
    /// Previous extruder
    pub from_extruder: usize,

    /// New extruder
    pub to_extruder: usize,

    /// Flush/purge volume required (mm³)
    pub flush_volume: f32,

    /// Z height where the tool change occurs
    pub print_z: CoordF,

    /// Whether this is a nozzle change (same extruder, different nozzle)
    pub is_nozzle_change: bool,
}

impl ToolChange {
    // Create a new tool change.
    pub fn new(from: usize, to: usize, flush_volume: f32, print_z: CoordF) -> Self {
        Self {
            from_extruder: from,
            to_extruder: to,
            flush_volume,
            print_z,
            is_nozzle_change: false,
        }
    }
}

/// Result of multi-material planning.
#[derive(Debug, Clone)]
pub struct MultiMaterialPlan {
    /// Layers with tool ordering and wipe tower data
    pub layers: Vec<MultiMaterialLayer>,

    /// Total flush/purge volume (mm³)
    pub total_flush_volume: f32,

    /// Total number of tool changes
    pub total_tool_changes: usize,

    /// Filament change statistics
    pub stats: FilamentChangeStats,

    /// Wipe tower bounding box (if generated)
    pub wipe_tower_bounds: Option<WipeTowerBounds>,

    /// First extruder used in the print
    pub first_extruder: Option<usize>,

    /// Last extruder used in the print
    pub last_extruder: Option<usize>,

    /// All extruders used in the print
    pub all_extruders: Vec<usize>,
}

impl Default for MultiMaterialPlan {
    fn default() -> Self {
        Self::new()
    }
}

impl MultiMaterialPlan {
    // Create a new empty plan.
    pub fn new() -> Self {
        Self {
            layers: Vec::new(),
            total_flush_volume: 0.0f32,
            total_tool_changes: 0,
            stats: FilamentChangeStats::new(),
            wipe_tower_bounds: None,
            first_extruder: None,
            last_extruder: None,
            all_extruders: Vec::new(),
        }
    }

    /// Check if the plan has any tool changes.
    pub fn has_tool_changes(&self) -> bool {
        self.total_tool_changes > 0
    }

    /// Get layer by index.
    pub fn get_layer(&self, layer_idx: usize) -> Option<&MultiMaterialLayer> {
        self.layers.iter().find(|l| l.layer_idx == layer_idx)
    }

    /// Get mutable layer by index.
    pub fn get_layer_mut(&mut self, layer_idx: usize) -> Option<&mut MultiMaterialLayer> {
        self.layers.iter_mut().find(|l| l.layer_idx == layer_idx)
    }
}

/// Wipe tower bounding box.
#[derive(Debug, Clone, Copy)]
pub struct WipeTowerBounds {
    pub min_x: CoordF,
    pub min_y: CoordF,
    pub max_x: CoordF,
    pub max_y: CoordF,
    pub height: CoordF,
}

impl WipeTowerBounds {
    // Create a new wipe tower bounds.
    pub fn new(min_x: CoordF, min_y: CoordF, max_x: CoordF, max_y: CoordF, height: CoordF) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
            height,
        }
    }

    /// Get the width of the wipe tower.
    pub fn width(&self) -> CoordF {
        self.max_x - self.min_x
    }

    /// Get the depth of the wipe tower.
    pub fn depth(&self) -> CoordF {
        self.max_y - self.min_y
    }

    /// Convert to a polygon (for collision detection).
    pub fn to_polygon(&self) -> Polygon {
        Polygon::from(vec![
            Point::new(scale(self.min_x), scale(self.min_y)),
            Point::new(scale(self.max_x), scale(self.min_y)),
            Point::new(scale(self.max_x), scale(self.max_y)),
            Point::new(scale(self.min_x), scale(self.max_y)),
        ])
    }

    /// Convert to an ExPolygon.
    pub fn to_expolygon(&self) -> ExPolygon {
        ExPolygon::from(self.to_polygon())
    }
}

/// Coordinates multi-material printing operations.
///
/// This struct brings together tool ordering and wipe tower generation
/// to plan and execute multi-extruder prints.
#[derive(Debug)]
pub struct MultiMaterialCoordinator {
    config: MultiMaterialConfig,
    tool_ordering: ToolOrdering,
    wipe_tower: Option<WipeTower>,
    initialized: bool,
}

impl MultiMaterialCoordinator {
    // Create a new multi-material coordinator.
    pub fn new(config: MultiMaterialConfig) -> Self {
        let tool_ordering_config = config.to_tool_ordering_config();
        let tool_ordering = ToolOrdering::new(tool_ordering_config);

        Self {
            config,
            tool_ordering,
            wipe_tower: None,
            initialized: false,
        }
    }

    /// Create a coordinator for single-material printing (no-op mode).
    pub fn single_material() -> Self {
        Self::new(MultiMaterialConfig::default())
    }

    /// Check if multi-material mode is active.
    pub fn is_multi_material(&self) -> bool {
        self.config.is_multi_material()
    }

    /// Get the configuration.
    pub fn config(&self) -> &MultiMaterialConfig {
        &self.config
    }

    /// Get the tool ordering.
    pub fn tool_ordering(&self) -> &ToolOrdering {
        &self.tool_ordering
    }

    /// Get mutable tool ordering.
    pub fn tool_ordering_mut(&mut self) -> &mut ToolOrdering {
        &mut self.tool_ordering
    }

    /// Get the wipe tower (if any).
    pub fn wipe_tower(&self) -> Option<&WipeTower> {
        self.wipe_tower.as_ref()
    }

    /// Get mutable wipe tower.
    pub fn wipe_tower_mut(&mut self) -> Option<&mut WipeTower> {
        self.wipe_tower.as_mut()
    }

    /// Initialize layers from print Z heights.
    ///
    /// # Arguments
    /// * `layer_z_heights` - Vector of (z_height, layer_height) for each layer
    pub fn initialize_layers(&mut self, layer_z_heights: &[(CoordF, CoordF)]) {
        let z_heights: Vec<f64> = layer_z_heights.iter().map(|(z, _)| *z).collect();
        self.tool_ordering.initialize_layers(z_heights);
        self.initialized = true;
    }

    /// Add extruder usage for a layer.
    ///
    /// # Arguments
    /// * `print_z` - Z height of the layer
    /// * `extruder` - Extruder index (0-based)
    /// * `role` - Extrusion role type
    pub fn add_extruder_to_layer(
        &mut self,
        print_z: CoordF,
        extruder: usize,
        role: ExtrusionRoleType,
    ) {
        // Determine if this is support material
        let is_support = matches!(
            role,
            ExtrusionRoleType::SupportMaterial | ExtrusionRoleType::SupportMaterialInterface
        );
        let is_interface = matches!(role, ExtrusionRoleType::SupportMaterialInterface);

        if is_support {
            self.tool_ordering.add_support_extruder_to_layer(
                print_z,
                extruder as u32,
                is_interface,
            );
        } else {
            self.tool_ordering
                .add_extruder_to_layer(print_z, extruder as u32, true);
        }
    }

    /// Plan tool changes and wipe tower for all layers.
    ///
    /// This should be called after all extruder usages have been added.
    ///
    /// # Arguments
    /// * `max_print_z` - Maximum Z height of the print
    ///
    /// # Returns
    /// A MultiMaterialPlan containing all tool changes and wipe tower data.
    pub fn plan(&mut self, max_print_z: CoordF) -> MultiMaterialPlan {
        if !self.config.is_multi_material() {
            return MultiMaterialPlan::new();
        }

        // Step 1: Handle "don't care" extruders (layers with no explicit extruder)
        self.tool_ordering.handle_dontcare_extruders(None);

        // Step 2: Reorder extruders to minimize flush volume
        self.tool_ordering.reorder_extruders_for_minimum_flush();

        // Step 3: Mark skirt layers
        self.tool_ordering.mark_skirt_layers();

        // Step 4: Fill wipe tower partitions
        // Use 0.0 as object_bottom_z since we want tower from the start
        self.tool_ordering.fill_wipe_tower_partitions(0.0);

        // Step 5: Collect statistics
        self.tool_ordering
            .collect_extruder_statistics(self.config.enable_prime_tower);

        // Step 6: Initialize wipe tower if enabled
        if self.config.enable_prime_tower {
            self.initialize_wipe_tower(max_print_z);

            // Step 7: Plan wipe tower for each tool change
            self.plan_wipe_tower_toolchanges();
        }

        // Step 8: Build the multi-material plan
        self.build_plan()
    }

    /// Initialize the wipe tower.
    fn initialize_wipe_tower(&mut self, max_print_z: CoordF) {
        let wipe_config = self.config.to_wipe_tower_config(max_print_z);
        let filament_params = self.config.to_filament_params();

        // Get initial tool (default to 0)
        let initial_tool = self
            .tool_ordering
            .first_extruder()
            .map(|e| e as usize)
            .unwrap_or(0);

        let mut wipe_tower = WipeTower::new(wipe_config, initial_tool, self.config.num_filaments);

        // Set filament parameters for each extruder
        for (idx, params) in filament_params.into_iter().enumerate() {
            wipe_tower.set_extruder(idx, params);
        }

        self.wipe_tower = Some(wipe_tower);
    }

    /// Plan wipe tower tool changes.
    fn plan_wipe_tower_toolchanges(&mut self) {
        let wipe_tower = match self.wipe_tower.as_mut() {
            Some(wt) => wt,
            None => return,
        };

        let layer_tools: Vec<_> = self.tool_ordering.layer_tools().to_vec();
        let num_layers = layer_tools.len();
        let flush_matrix = self.config.flush_matrix.clone();
        let flush_multipliers = self.config.flush_multipliers.clone();

        for (layer_idx, layer_tool) in layer_tools.iter().enumerate() {
            let print_z = layer_tool.print_z;
            let layer_height = layer_tool.wipe_tower_layer_height;

            // Count tool changes for this layer
            let extruders: Vec<_> = layer_tool.extruders.iter().copied().collect();
            let max_tool_changes = if extruders.len() > 1 {
                extruders.len() - 1
            } else {
                0
            };

            let is_first_layer = layer_idx == 0;
            let is_last_layer = layer_idx == num_layers - 1;

            // Set layer on wipe tower
            wipe_tower.set_layer(
                print_z as f32,
                layer_height as f32,
                max_tool_changes,
                is_first_layer,
                is_last_layer,
            );

            // Plan tool changes for this layer
            for i in 1..extruders.len() {
                let old_tool = extruders[i - 1] as usize;
                let new_tool = extruders[i] as usize;

                if old_tool != new_tool {
                    // Calculate flush volume locally to avoid borrow issues
                    let base_volume = flush_matrix.get(old_tool, new_tool);
                    let multiplier = flush_multipliers.get(new_tool).copied().unwrap_or(1.0);
                    let flush_vol = base_volume * multiplier;

                    wipe_tower.plan_toolchange(
                        print_z as f32,
                        layer_height as f32,
                        old_tool,
                        new_tool,
                        flush_vol, // wipe_volume_ec
                        flush_vol, // wipe_volume_nc (same for now)
                        flush_vol, // purge_volume
                    );
                }
            }
        }

        // Plan the complete tower
        wipe_tower.plan_tower();
    }

    /// Build the final multi-material plan.
    fn build_plan(&self) -> MultiMaterialPlan {
        let mut plan = MultiMaterialPlan::new();

        // Copy layer data from tool ordering
        for (idx, layer_tool) in self.tool_ordering.layer_tools().iter().enumerate() {
            let mut mm_layer = MultiMaterialLayer::new(
                idx,
                layer_tool.print_z,
                layer_tool.wipe_tower_layer_height,
            );

            mm_layer.extruders = layer_tool.extruders.iter().map(|&e| e as usize).collect();
            mm_layer.has_skirt = layer_tool.has_skirt;

            if let Some(ref gcode) = layer_tool.custom_gcode {
                mm_layer.custom_gcodes.push(gcode.clone());
            }

            // Build tool changes
            let extruders: Vec<_> = mm_layer.extruders.clone();
            for i in 1..extruders.len() {
                let from = extruders[i - 1];
                let to = extruders[i];

                if from != to {
                    let flush_vol = self.config.get_flush_volume(from, to);
                    mm_layer.tool_changes.push(ToolChange::new(
                        from,
                        to,
                        flush_vol,
                        layer_tool.print_z,
                    ));

                    plan.total_flush_volume += flush_vol as f32;
                    plan.total_tool_changes += 1;
                }
            }

            plan.layers.push(mm_layer);
        }

        // Set extruder info
        plan.first_extruder = self.tool_ordering.first_extruder().map(|e| e as usize);
        plan.last_extruder = self.tool_ordering.last_extruder().map(|e| e as usize);
        plan.all_extruders = self
            .tool_ordering
            .all_extruders()
            .iter()
            .map(|&e| e as usize)
            .collect();

        // Get statistics
        plan.stats = self
            .tool_ordering
            .get_filament_change_stats(FilamentChangeMode::MultiExtBest)
            .clone();

        // Get wipe tower bounds
        if let Some(ref wipe_tower) = self.wipe_tower {
            let bbox = wipe_tower.get_bounding_box();
            let height = wipe_tower.get_height();

            plan.wipe_tower_bounds = Some(WipeTowerBounds::new(
                bbox.min.x,
                bbox.min.y,
                bbox.max.x,
                bbox.max.y,
                height as CoordF,
            ));
        }

        plan
    }

    /// Generate wipe tower G-code for all layers.
    ///
    /// # Returns
    /// Vector of tool change results per layer.
    pub fn generate_wipe_tower(&mut self) -> Vec<Vec<ToolChangeResult>> {
        if let Some(ref mut wipe_tower) = self.wipe_tower {
            wipe_tower.generate()
        } else {
            Vec::new()
        }
    }

    /// Get the extruder to use for a given role and layer.
    ///
    /// # Arguments
    /// * `print_z` - Z height of the layer
    /// * `role` - Extrusion role type
    ///
    /// # Returns
    /// The extruder index to use.
    pub fn get_extruder_for_role(&self, print_z: CoordF, role: ExtrusionRoleType) -> usize {
        if let Some(layer_tools) = self.tool_ordering.layer_tools_for_layer(print_z) {
            // Use defaults from config (1-indexed in tool_ordering API)
            let wall_filament = (self.config.default_perimeter_extruder + 1) as u32;
            let sparse_infill_filament = (self.config.default_infill_extruder + 1) as u32;
            let solid_infill_filament = (self.config.default_solid_infill_extruder + 1) as u32;
            layer_tools.extruder_for_role(
                role,
                wall_filament,
                sparse_infill_filament,
                solid_infill_filament,
            ) as usize
        } else {
            // Fall back to defaults
            match role {
                ExtrusionRoleType::Perimeter
                | ExtrusionRoleType::ExternalPerimeter
                | ExtrusionRoleType::OverhangPerimeter => self.config.default_perimeter_extruder,

                ExtrusionRoleType::InternalInfill | ExtrusionRoleType::GapFill => {
                    self.config.default_infill_extruder
                }

                ExtrusionRoleType::SolidInfill
                | ExtrusionRoleType::TopSolidInfill
                | ExtrusionRoleType::BridgeInfill => self.config.default_solid_infill_extruder,

                ExtrusionRoleType::SupportMaterial | ExtrusionRoleType::SupportTransition => {
                    self.config.default_support_extruder
                }

                ExtrusionRoleType::SupportMaterialInterface => {
                    self.config.default_support_interface_extruder
                }

                _ => 0,
            }
        }
    }

    /// Check if a tool change is needed between two extrusion roles.
    ///
    /// # Arguments
    /// * `print_z` - Z height
    /// * `current_extruder` - Current extruder
    /// * `role` - Next extrusion role
    ///
    /// # Returns
    /// Some(new_extruder) if a tool change is needed, None otherwise.
    pub fn needs_tool_change(
        &self,
        print_z: CoordF,
        current_extruder: usize,
        role: ExtrusionRoleType,
    ) -> Option<usize> {
        let target_extruder = self.get_extruder_for_role(print_z, role);

        if target_extruder != current_extruder {
            Some(target_extruder)
        } else {
            None
        }
    }

    /// Get flush volume for a tool change.
    pub fn get_flush_volume(&self, from: usize, to: usize) -> f32 {
        self.config.get_flush_volume(from, to)
    }

    /// Get wipe tower avoidance polygon for travel planning.
    ///
    /// Returns a polygon that should be avoided when planning travel moves.
    pub fn get_wipe_tower_avoidance(&self) -> Option<ExPolygon> {
        if let Some(ref wipe_tower) = self.wipe_tower {
            let bbox = wipe_tower.get_bounding_box();

            // Add a small margin for safety
            let margin = 2.0; // mm
            let bounds = WipeTowerBounds::new(
                bbox.min.x - margin,
                bbox.min.y - margin,
                bbox.max.x + margin,
                bbox.max.y + margin,
                wipe_tower.get_height() as CoordF,
            );

            Some(bounds.to_expolygon())
        } else {
            None
        }
    }
}

impl Default for MultiMaterialCoordinator {
    fn default() -> Self {
        Self::single_material()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_multi_material_config_default() {
        let config = MultiMaterialConfig::default();
        assert_eq!(config.num_filaments, 1);
        assert!(!config.is_multi_material());
        assert!(!config.enable_prime_tower);
    }

    #[test]
    fn test_multi_material_config_new() {
        let config = MultiMaterialConfig::new(4);
        assert_eq!(config.num_filaments, 4);
        assert!(config.is_multi_material());
        assert!(config.enable_prime_tower);
        assert_eq!(config.nozzle_diameters.len(), 4);
        assert_eq!(config.filament_colors.len(), 4);
    }

    #[test]
    fn test_default_filament_colors() {
        assert_eq!(default_filament_color(0), "#FFFFFF");
        assert_eq!(default_filament_color(1), "#000000");
        assert_eq!(default_filament_color(2), "#FF0000");
        // Test wrap-around
        assert_eq!(default_filament_color(10), "#FFFFFF");
    }

    #[test]
    fn test_coordinator_single_material() {
        let coord = MultiMaterialCoordinator::single_material();
        assert!(!coord.is_multi_material());
        assert!(coord.wipe_tower().is_none());
    }

    #[test]
    fn test_coordinator_multi_material() {
        let config = MultiMaterialConfig::new(2);
        let coord = MultiMaterialCoordinator::new(config);
        assert!(coord.is_multi_material());
    }

    #[test]
    fn test_tool_change() {
        let tc = ToolChange::new(0, 1, 50.0, 0.3);
        assert_eq!(tc.from_extruder, 0);
        assert_eq!(tc.to_extruder, 1);
        assert_eq!(tc.flush_volume, 50.0);
        assert!(!tc.is_nozzle_change);
    }

    #[test]
    fn test_multi_material_layer() {
        let layer = MultiMaterialLayer::new(0, 0.2, 0.2);
        assert_eq!(layer.layer_idx, 0);
        assert!(!layer.has_tool_changes());
        assert_eq!(layer.num_tool_changes(), 0);
    }

    #[test]
    fn test_multi_material_plan() {
        let plan = MultiMaterialPlan::new();
        assert!(!plan.has_tool_changes());
        assert!(plan.layers.is_empty());
    }

    #[test]
    fn test_wipe_tower_bounds() {
        let bounds = WipeTowerBounds::new(100.0, 100.0, 160.0, 180.0, 50.0);
        assert_eq!(bounds.width(), 60.0);
        assert_eq!(bounds.depth(), 80.0);

        let polygon = bounds.to_polygon();
        assert_eq!(polygon.points().len(), 4);
    }

    #[test]
    fn test_flush_volume_calculation() {
        let mut config = MultiMaterialConfig::new(2);
        config.flush_matrix.set(0, 1, 100.0);
        config.flush_multipliers = vec![1.0f32, 1.5f32];

        assert_eq!(config.get_flush_volume(0, 1), 150.0f32); // 100 * 1.5
    }

    #[test]
    fn test_to_tool_ordering_config() {
        let config = MultiMaterialConfig::new(3);
        let tool_config = config.to_tool_ordering_config();
        assert_eq!(tool_config.num_filaments, 3);
        assert!(tool_config.enable_prime_tower);
    }

    #[test]
    fn test_to_filament_params() {
        let mut config = MultiMaterialConfig::new(2);
        config.filament_types = vec!["PLA".to_string(), "PETG".to_string()];
        config.filament_soluble = vec![false, true];

        let params = config.to_filament_params();
        assert_eq!(params.len(), 2);
        assert_eq!(params[0].material, "PLA");
        assert!(!params[0].is_soluble);
        assert_eq!(params[1].material, "PETG");
        assert!(params[1].is_soluble);
    }

    #[test]
    fn test_coordinator_initialize_and_plan() {
        let config = MultiMaterialConfig::new(2);
        let mut coord = MultiMaterialCoordinator::new(config);

        // Initialize layers
        let layers = vec![(0.2, 0.2), (0.4, 0.2), (0.6, 0.2)];
        coord.initialize_layers(&layers);

        // Add extruder usage
        coord.add_extruder_to_layer(0.2, 0, ExtrusionRoleType::Perimeter);
        coord.add_extruder_to_layer(0.2, 1, ExtrusionRoleType::InternalInfill);
        coord.add_extruder_to_layer(0.4, 0, ExtrusionRoleType::Perimeter);
        coord.add_extruder_to_layer(0.6, 1, ExtrusionRoleType::Perimeter);

        // Plan
        let plan = coord.plan(0.6);

        // Should have layers
        assert!(!plan.layers.is_empty());
    }

    #[test]
    fn test_get_extruder_for_role() {
        let mut config = MultiMaterialConfig::new(2);
        config.default_perimeter_extruder = 0;
        config.default_infill_extruder = 1;
        config.default_support_extruder = 1;

        let coord = MultiMaterialCoordinator::new(config);

        // Without layer tools initialized, falls back to defaults
        assert_eq!(
            coord.get_extruder_for_role(0.2, ExtrusionRoleType::Perimeter),
            0
        );
        assert_eq!(
            coord.get_extruder_for_role(0.2, ExtrusionRoleType::InternalInfill),
            1
        );
        assert_eq!(
            coord.get_extruder_for_role(0.2, ExtrusionRoleType::SupportMaterial),
            1
        );
    }

    #[test]
    fn test_needs_tool_change() {
        let mut config = MultiMaterialConfig::new(2);
        config.default_perimeter_extruder = 0;
        config.default_infill_extruder = 1;

        let coord = MultiMaterialCoordinator::new(config);

        // Current extruder 0, perimeter role -> no change needed
        assert!(coord
            .needs_tool_change(0.2, 0, ExtrusionRoleType::Perimeter)
            .is_none());

        // Current extruder 0, infill role -> change to 1
        assert_eq!(
            coord.needs_tool_change(0.2, 0, ExtrusionRoleType::InternalInfill),
            Some(1)
        );
    }
}
