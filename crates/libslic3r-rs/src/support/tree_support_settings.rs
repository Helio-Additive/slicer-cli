//! Tree Support Settings - Configuration and support element state management.
//!
//! This module provides the configuration structures and support element state types
//! for tree support generation. It is a Rust port of BambuStudio's `TreeSupportCommon.hpp`.
//!
//! # Overview
//!
//! Tree supports use a state machine approach where each support element tracks:
//! - Its current position and target position
//! - Distance to the top of the support structure
//! - Radius that grows as branches merge and descend
//! - Movement constraints and avoidance settings
//!
//! # BambuStudio Reference
//!
//! - `Support/TreeSupportCommon.hpp`
//! - `Support/TreeSupport3D.hpp`

use crate::geometry::Point;
use crate::{scale, unscale, Coord, CoordF};

/// Circle resolution for tree support branches.
/// Higher values = smoother circles but more polygons.
pub const TREE_CIRCLE_RESOLUTION: usize = 25;

/// Interface preference for how support interfaces interact with support areas.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InterfacePreference {
    /// Interface areas overwrite support areas.
    #[default]
    InterfaceAreaOverwritesSupport,
    /// Support areas overwrite interface areas.
    SupportAreaOverwritesInterface,
    /// Interface lines overwrite support lines.
    InterfaceLinesOverwriteSupport,
    /// Support lines overwrite interface lines.
    SupportLinesOverwriteInterface,
    /// No preference.
    Nothing,
}

/// Settings for tree support mesh groups.
///
/// This mirrors `TreeSupportMeshGroupSettings` from BambuStudio and contains
/// all the settings that affect tree support generation for a group of meshes.
#[derive(Debug, Clone)]
pub struct TreeSupportMeshGroupSettings {
    /// Layer height in scaled units.
    pub layer_height: Coord,
    /// Resolution for polygon operations.
    pub resolution: Coord,
    /// Minimum feature size to preserve.
    pub min_feature_size: Coord,
    /// Maximum overhang angle before support is needed (degrees).
    pub support_angle: f64,
    /// Support line width.
    pub support_line_width: Coord,
    /// Support roof line width.
    pub support_roof_line_width: Coord,
    /// Whether bottom interface is enabled.
    pub support_bottom_enable: bool,
    /// Height of bottom interface layers.
    pub support_bottom_height: Coord,
    /// Whether support is only on build plate.
    pub support_material_buildplate_only: bool,
    /// XY distance from model.
    pub support_xy_distance: Coord,
    /// XY distance on first layer.
    pub support_xy_distance_1st_layer: Coord,
    /// XY distance over overhangs.
    pub support_xy_distance_overhang: Coord,
    /// Z distance at top of support.
    pub support_top_distance: Coord,
    /// Z distance at bottom of support.
    pub support_bottom_distance: Coord,
    /// Skip height for interface layers.
    pub support_interface_skip_height: Coord,
    /// Whether roof interface is enabled.
    pub support_roof_enable: bool,
    /// Number of roof layers.
    pub support_roof_layers: usize,
    /// Whether floor interface is enabled.
    pub support_floor_enable: bool,
    /// Number of floor layers.
    pub support_floor_layers: usize,
    /// Minimum area for roof generation.
    pub minimum_roof_area: f64,
    /// Angles for roof pattern.
    pub support_roof_angles: Vec<f64>,
    /// Support line spacing.
    pub support_line_spacing: Coord,
    /// Offset at bottom of support.
    pub support_bottom_offset: Coord,
    /// Number of support wall loops.
    pub support_wall_count: usize,
    /// Distance between roof lines.
    pub support_roof_line_distance: Coord,
    /// Minimum area for support regions.
    pub minimum_support_area: Coord,
    /// Minimum area for bottom interface.
    pub minimum_bottom_area: Coord,
    /// General support offset.
    pub support_offset: Coord,
    /// Tree branch angle (degrees).
    pub support_tree_angle: f64,
    /// Rate of branch diameter increase per angle.
    pub support_tree_branch_diameter_angle: f64,
    /// Distance between branches.
    pub support_tree_branch_distance: Coord,
    /// Starting branch diameter.
    pub support_tree_branch_diameter: Coord,
    /// Slow angle for branches (degrees).
    pub support_tree_angle_slow: f64,
    /// Maximum diameter increase from merges when supporting to model.
    pub support_tree_max_diameter_increase_by_merges_when_support_to_model: Coord,
    /// Minimum height to start supporting to model.
    pub support_tree_min_height_to_model: Coord,
    /// Branch diameter at build plate.
    pub support_tree_bp_diameter: Coord,
    /// Rate at which branches reach full thickness.
    pub support_tree_top_rate: f64,
    /// Tip diameter for tree branches.
    pub support_tree_tip_diameter: Coord,
}

impl Default for TreeSupportMeshGroupSettings {
    fn default() -> Self {
        Self {
            layer_height: scale(0.2),
            resolution: scale(0.025),
            min_feature_size: scale(0.1),
            support_angle: 50.0,
            support_line_width: scale(0.4),
            support_roof_line_width: scale(0.4),
            support_bottom_enable: false,
            support_bottom_height: scale(0.8),
            support_material_buildplate_only: false,
            support_xy_distance: scale(0.8),
            support_xy_distance_1st_layer: scale(0.8),
            support_xy_distance_overhang: scale(0.4),
            support_top_distance: scale(0.2),
            support_bottom_distance: scale(0.2),
            support_interface_skip_height: scale(0.2),
            support_roof_enable: true,
            support_roof_layers: 3,
            support_floor_enable: false,
            support_floor_layers: 0,
            minimum_roof_area: 1.0, // mm²
            support_roof_angles: vec![0.0],
            support_line_spacing: scale(2.0),
            support_bottom_offset: scale(0.0),
            support_wall_count: 1,
            support_roof_line_distance: scale(0.4),
            minimum_support_area: scale(1.0),
            minimum_bottom_area: scale(1.0),
            support_offset: scale(0.0),
            support_tree_angle: 40.0,
            support_tree_branch_diameter_angle: 5.0,
            support_tree_branch_distance: scale(1.0),
            support_tree_branch_diameter: scale(2.0),
            support_tree_angle_slow: 25.0,
            support_tree_max_diameter_increase_by_merges_when_support_to_model: scale(1.0),
            support_tree_min_height_to_model: scale(0.5),
            support_tree_bp_diameter: scale(7.5),
            support_tree_top_rate: 15.0,
            support_tree_tip_diameter: scale(0.8),
        }
    }
}

/// Main tree support settings derived from mesh group settings.
///
/// These are the computed/derived settings used during tree support generation.
#[derive(Debug, Clone)]
pub struct TreeSupportSettings {
    /// Branch angle in radians.
    pub angle: f64,
    /// Slow branch angle in radians.
    pub angle_slow: f64,
    /// Known Z heights for each layer.
    pub known_z: Vec<Coord>,
    /// Whether support material is soluble.
    pub soluble: bool,
    /// Support line width.
    pub support_line_width: Coord,
    /// Layer height.
    pub layer_height: Coord,
    /// Base branch radius.
    pub branch_radius: Coord,
    /// Minimum branch radius (tip).
    pub min_radius: Coord,
    /// Maximum horizontal movement per layer.
    pub maximum_move_distance: Coord,
    /// Maximum horizontal movement per layer (slow mode).
    pub maximum_move_distance_slow: Coord,
    /// Number of bottom interface layers.
    pub support_bottom_layers: usize,
    /// Number of tip layers (before branch thickening).
    pub tip_layers: usize,
    /// Branch radius increase per layer.
    pub branch_radius_increase_per_layer: f64,
    /// Maximum radius increase when supporting to model.
    pub max_to_model_radius_increase: Coord,
    /// Minimum distance-to-top before supporting to model.
    pub min_dtt_to_model: usize,
    /// Radius threshold for increase.
    pub increase_radius_until_radius: Coord,
    /// Layer threshold for increase.
    pub increase_radius_until_layer: usize,
    /// Whether support can rest on the model.
    pub support_rests_on_model: bool,
    /// XY distance from model.
    pub xy_distance: Coord,
    /// Build plate branch radius.
    pub bp_radius: Coord,
    /// Layer index where BP radius starts.
    pub layer_start_bp_radius: usize,
    /// BP radius increase per layer.
    pub bp_radius_increase_per_layer: f64,
    /// Minimum XY distance.
    pub xy_min_distance: Coord,
    /// Z distance in layers at top.
    pub z_distance_top_layers: usize,
    /// Z distance in layers at bottom.
    pub z_distance_bottom_layers: usize,
    /// Skip layers for performance.
    pub performance_interface_skip_layers: usize,
    /// Roof pattern angles.
    pub support_roof_angles: Vec<f64>,
    /// Resolution for polygon operations.
    pub resolution: Coord,
    /// Roof line distance.
    pub support_roof_line_distance: Coord,
    /// Interface preference setting.
    pub interface_preference: InterfacePreference,
    /// Original mesh group settings.
    pub settings: TreeSupportMeshGroupSettings,
    /// Minimum feature size.
    pub min_feature_size: Coord,
    /// Raft layer Z heights.
    pub raft_layers: Vec<CoordF>,
}

impl TreeSupportSettings {
    // Create settings from mesh group settings.
    pub fn new(settings: TreeSupportMeshGroupSettings) -> Self {
        let angle = settings.support_tree_angle.to_radians();
        let angle_slow = settings.support_tree_angle_slow.to_radians();

        // Calculate maximum move distance based on angle
        // tan(angle) * layer_height gives horizontal distance per layer
        let layer_height_f = unscale(settings.layer_height);
        let max_move = scale(layer_height_f * angle.tan());
        let max_move_slow = scale(layer_height_f * angle_slow.tan());

        // Calculate branch radius from diameter
        let branch_radius = settings.support_tree_branch_diameter / 2;
        let min_radius = settings.support_tree_tip_diameter / 2;
        let bp_radius = settings.support_tree_bp_diameter / 2;

        // Calculate tip layers (layers before branch starts thickening)
        let tip_layers = if min_radius < branch_radius {
            let radius_diff = unscale(branch_radius - min_radius);
            let increase_per_layer = radius_diff / settings.support_tree_top_rate;
            (radius_diff / increase_per_layer).ceil() as usize
        } else {
            0
        };

        // Calculate radius increase per layer
        let branch_radius_increase_per_layer = if tip_layers > 0 {
            unscale(branch_radius - min_radius) / tip_layers as f64
        } else {
            0.0
        };

        // Calculate increase_radius_until values
        let increase_radius_until_radius = branch_radius;
        let increase_radius_until_layer = tip_layers;

        // Z distance in layers
        let z_distance_top_layers =
            (unscale(settings.support_top_distance) / layer_height_f).ceil() as usize;
        let z_distance_bottom_layers =
            (unscale(settings.support_bottom_distance) / layer_height_f).ceil() as usize;

        Self {
            angle,
            angle_slow,
            known_z: Vec::new(),
            soluble: false,
            support_line_width: settings.support_line_width,
            layer_height: settings.layer_height,
            branch_radius,
            min_radius,
            maximum_move_distance: max_move,
            maximum_move_distance_slow: max_move_slow,
            support_bottom_layers: if settings.support_bottom_enable {
                (unscale(settings.support_bottom_height) / layer_height_f).ceil() as usize
            } else {
                0
            },
            tip_layers,
            branch_radius_increase_per_layer,
            max_to_model_radius_increase: settings
                .support_tree_max_diameter_increase_by_merges_when_support_to_model,
            min_dtt_to_model: (unscale(settings.support_tree_min_height_to_model) / layer_height_f)
                .ceil() as usize,
            increase_radius_until_radius,
            increase_radius_until_layer,
            support_rests_on_model: !settings.support_material_buildplate_only,
            xy_distance: settings.support_xy_distance,
            bp_radius,
            layer_start_bp_radius: 0,
            bp_radius_increase_per_layer: 0.0,
            xy_min_distance: settings.support_xy_distance_overhang,
            z_distance_top_layers,
            z_distance_bottom_layers,
            performance_interface_skip_layers: 0,
            support_roof_angles: settings.support_roof_angles.clone(),
            resolution: settings.resolution,
            support_roof_line_distance: settings.support_roof_line_distance,
            interface_preference: InterfacePreference::default(),
            settings,
            min_feature_size: scale(0.1),
            raft_layers: Vec::new(),
        }
    }

    /// Get the radius for a given effective distance-to-top.
    pub fn get_radius(&self, effective_dtt: usize, elephant_foot_increases: f64) -> Coord {
        let base_radius = if effective_dtt < self.tip_layers {
            // In tip region, radius grows from min to branch radius
            self.min_radius + scale(self.branch_radius_increase_per_layer * effective_dtt as f64)
        } else {
            self.branch_radius
        };

        // Add elephant foot compensation if any
        let elephant_foot_extra = scale(elephant_foot_increases * 0.1); // 0.1mm per increase

        base_radius + elephant_foot_extra
    }

    /// Get the recommended minimum radius for printability.
    pub fn recommended_min_radius(&self) -> Coord {
        // Minimum radius should be at least half the line width
        (self.support_line_width / 2).max(self.min_radius)
    }

    /// Get Z height for a layer index.
    pub fn get_actual_z(&self, layer_idx: usize) -> Coord {
        if layer_idx < self.known_z.len() {
            self.known_z[layer_idx]
        } else if !self.known_z.is_empty() {
            *self.known_z.last().unwrap()
        } else {
            // Estimate based on layer height
            self.layer_height * (layer_idx as Coord + 1)
        }
    }

    /// Set Z heights from layer information.
    pub fn set_actual_z(&mut self, z_heights: Vec<Coord>) {
        self.known_z = z_heights;
    }
}

impl Default for TreeSupportSettings {
    fn default() -> Self {
        Self::new(TreeSupportMeshGroupSettings::default())
    }
}

/// Settings for area increase operations during branch propagation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AreaIncreaseSettings {
    /// Speed of area increase.
    pub increase_speed: Coord,
    /// Type of avoidance to use.
    pub avoidance_type: AvoidanceTypeCompact,
    /// Whether to increase radius.
    pub increase_radius: bool,
    /// Whether to suppress errors.
    pub no_error: bool,
    /// Whether to use minimum distance mode.
    pub use_min_distance: bool,
    /// Whether movement is allowed.
    pub allow_move: bool,
}

/// Compact avoidance type for storage efficiency.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AvoidanceTypeCompact {
    #[default]
    Fast,
    FastSafe,
    Slow,
}

/// Bit flags for support element state.
#[derive(Debug, Clone, Copy, Default)]
pub struct SupportElementStateBits {
    /// Element tries to reach the build plate.
    pub to_buildplate: bool,
    /// Branch can rest on flat surface (build plate or model).
    pub to_model_gracious: bool,
    /// Use minimum XY distance.
    pub use_min_xy_dist: bool,
    /// Supports a roof interface.
    pub supports_roof: bool,
    /// Can use safe radius (hole-free avoidance).
    pub can_use_safe_radius: bool,
    /// Skip ovalisation when generating final circles.
    pub skip_ovalisation: bool,
    /// Marked as lost (debugging).
    pub lost: bool,
    /// Marked as very lost (debugging).
    pub verylost: bool,
    /// Marked for deletion.
    pub deleted: bool,
    /// General purpose visited marker.
    pub marked: bool,
}

/// State of a support element during tree generation.
#[derive(Debug, Clone)]
pub struct SupportElementState {
    /// Bit flags.
    pub bits: SupportElementStateBits,
    /// Element type identifier.
    pub element_type: i32,
    /// Current radius.
    pub radius: CoordF,
    /// Print Z height.
    pub print_z: f32,
    /// Target layer height (where this element wants to support).
    pub target_height: usize,
    /// Target position to support.
    pub target_position: Point,
    /// Next suggested position.
    pub next_position: Point,
    /// Current layer index.
    pub layer_idx: usize,
    /// Effective radius considering growth history.
    pub effective_radius_height: u32,
    /// Distance to topmost layer of this branch.
    pub distance_to_top: u32,
    /// Resulting center point for final circle.
    pub result_on_layer: Option<Point>,
    /// Extra radius from merging with non-BP branches.
    pub increased_to_model_radius: Coord,
    /// Counter for elephant foot increases.
    pub elephant_foot_increases: f64,
    /// Don't move until this DTT is reached.
    pub dont_move_until: u32,
    /// Last area increase settings used.
    pub last_area_increase: AreaIncreaseSettings,
    /// Roof layers not yet added due to movement.
    pub missing_roof_layers: u32,
}

impl Default for SupportElementState {
    fn default() -> Self {
        Self {
            bits: SupportElementStateBits::default(),
            element_type: 0,
            radius: 0.0,
            print_z: 0.0,
            target_height: 0,
            target_position: Point::new(0, 0),
            next_position: Point::new(0, 0),
            layer_idx: 0,
            effective_radius_height: 0,
            distance_to_top: 0,
            result_on_layer: None,
            increased_to_model_radius: 0,
            elephant_foot_increases: 0.0,
            dont_move_until: 0,
            last_area_increase: AreaIncreaseSettings::default(),
            missing_roof_layers: 0,
        }
    }
}

impl SupportElementState {
    // Create a new support element state.
    pub fn new(layer_idx: usize, position: Point, radius: CoordF) -> Self {
        Self {
            layer_idx,
            target_position: position,
            next_position: position,
            radius,
            ..Default::default()
        }
    }

    /// Check if result_on_layer is set.
    pub fn result_on_layer_is_set(&self) -> bool {
        self.result_on_layer.is_some()
    }

    /// Reset result_on_layer.
    pub fn result_on_layer_reset(&mut self) {
        self.result_on_layer = None;
    }

    /// Propagate state down one layer.
    pub fn propagate_down(&self) -> Self {
        let mut dst = self.clone();
        dst.distance_to_top += 1;
        dst.layer_idx = dst.layer_idx.saturating_sub(1);
        dst.result_on_layer_reset();
        dst.bits.skip_ovalisation = false;
        dst
    }

    /// Get effective distance-to-top for radius calculation.
    pub fn get_effective_dtt(&self, settings: &TreeSupportSettings) -> usize {
        if (self.effective_radius_height as usize) < settings.increase_radius_until_layer {
            if (self.distance_to_top as usize) < settings.increase_radius_until_layer {
                self.distance_to_top as usize
            } else {
                settings.increase_radius_until_layer
            }
        } else {
            self.effective_radius_height as usize
        }
    }

    /// Get the radius this element will have.
    pub fn get_radius(&self, settings: &TreeSupportSettings) -> Coord {
        settings.get_radius(
            self.get_effective_dtt(settings),
            self.elephant_foot_increases,
        )
    }

    /// Get the collision radius (may be smaller than actual radius).
    pub fn get_collision_radius(&self, settings: &TreeSupportSettings) -> Coord {
        settings.get_radius(
            self.effective_radius_height as usize,
            self.elephant_foot_increases,
        )
    }
}

/// Parent indices for support elements.
/// Uses a small vector optimization for common cases.
pub type ParentIndices = Vec<i32>;

/// A complete support element with influence area.
#[derive(Debug, Clone)]
pub struct SupportElement {
    /// Element state.
    pub state: SupportElementState,
    /// Parent element indices in the layer above.
    pub parents: ParentIndices,
    /// Influence area polygons.
    pub influence_area: Vec<crate::geometry::Polygon>,
}

impl SupportElement {
    // Create a new support element with state and influence area.
    pub fn new(state: SupportElementState, influence_area: Vec<crate::geometry::Polygon>) -> Self {
        Self {
            state,
            parents: Vec::new(),
            influence_area,
        }
    }

    /// Create with parents.
    pub fn with_parents(
        state: SupportElementState,
        parents: ParentIndices,
        influence_area: Vec<crate::geometry::Polygon>,
    ) -> Self {
        Self {
            state,
            parents,
            influence_area,
        }
    }

    /// Get the radius for this element.
    pub fn get_radius(&self, settings: &TreeSupportSettings) -> Coord {
        self.state.get_radius(settings)
    }

    /// Get the collision radius for this element.
    pub fn get_collision_radius(&self, settings: &TreeSupportSettings) -> Coord {
        self.state.get_collision_radius(settings)
    }
}

/// Line status for tree support path segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineStatus {
    /// Invalid or unknown status.
    Invalid,
    /// Line goes to model.
    ToModel,
    /// Line goes to model graciously (can land on flat surface).
    ToModelGracious,
    /// Line goes to model graciously and safely.
    ToModelGraciousSafe,
    /// Line goes to build plate.
    ToBuildPlate,
    /// Line goes to build plate safely.
    ToBuildPlateSafe,
}

impl Default for LineStatus {
    fn default() -> Self {
        Self::Invalid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mesh_group_settings_default() {
        let settings = TreeSupportMeshGroupSettings::default();

        assert_eq!(settings.layer_height, scale(0.2));
        assert!((settings.support_tree_angle - 40.0).abs() < 1e-10);
        assert!(settings.support_roof_enable);
    }

    #[test]
    fn test_tree_support_settings_new() {
        let mesh_settings = TreeSupportMeshGroupSettings::default();
        let settings = TreeSupportSettings::new(mesh_settings);

        // Angle should be converted to radians
        assert!((settings.angle - 40.0_f64.to_radians()).abs() < 1e-10);

        // Max move should be derived from angle and layer height
        assert!(settings.maximum_move_distance > 0);
        assert!(settings.maximum_move_distance_slow > 0);
        assert!(settings.maximum_move_distance > settings.maximum_move_distance_slow);
    }

    #[test]
    fn test_get_radius_tip() {
        let settings = TreeSupportSettings::default();

        // At tip (dtt=0), should get min radius
        let r0 = settings.get_radius(0, 0.0);
        assert_eq!(r0, settings.min_radius);

        // As dtt increases, radius should increase
        let r1 = settings.get_radius(1, 0.0);
        assert!(r1 >= r0);
    }

    #[test]
    fn test_get_radius_with_elephant_foot() {
        let settings = TreeSupportSettings::default();

        let r_no_ef = settings.get_radius(0, 0.0);
        let r_with_ef = settings.get_radius(0, 1.0);

        // Elephant foot should increase radius
        assert!(r_with_ef > r_no_ef);
    }

    #[test]
    fn test_support_element_state_new() {
        let state = SupportElementState::new(5, Point::new(100, 200), 1.5);

        assert_eq!(state.layer_idx, 5);
        assert_eq!(state.target_position, Point::new(100, 200));
        assert!((state.radius - 1.5).abs() < 1e-10);
    }

    #[test]
    fn test_support_element_state_propagate_down() {
        let state = SupportElementState::new(5, Point::new(100, 200), 1.5);
        let propagated = state.propagate_down();

        assert_eq!(propagated.layer_idx, 4);
        assert_eq!(propagated.distance_to_top, 1);
        assert!(!propagated.result_on_layer_is_set());
    }

    #[test]
    fn test_support_element_state_result_on_layer() {
        let mut state = SupportElementState::default();

        assert!(!state.result_on_layer_is_set());

        state.result_on_layer = Some(Point::new(50, 50));
        assert!(state.result_on_layer_is_set());

        state.result_on_layer_reset();
        assert!(!state.result_on_layer_is_set());
    }

    #[test]
    fn test_support_element_new() {
        let state = SupportElementState::default();
        let polygons = vec![];
        let element = SupportElement::new(state, polygons);

        assert!(element.parents.is_empty());
        assert!(element.influence_area.is_empty());
    }

    #[test]
    fn test_support_element_with_parents() {
        let state = SupportElementState::default();
        let parents = vec![0, 1, 2];
        let polygons = vec![];
        let element = SupportElement::with_parents(state, parents, polygons);

        assert_eq!(element.parents.len(), 3);
    }

    #[test]
    fn test_interface_preference_default() {
        let pref = InterfacePreference::default();
        assert_eq!(pref, InterfacePreference::InterfaceAreaOverwritesSupport);
    }

    #[test]
    fn test_line_status_default() {
        let status = LineStatus::default();
        assert_eq!(status, LineStatus::Invalid);
    }

    #[test]
    fn test_area_increase_settings_default() {
        let settings = AreaIncreaseSettings::default();

        assert_eq!(settings.increase_speed, 0);
        assert!(!settings.increase_radius);
        assert!(!settings.no_error);
        assert!(!settings.use_min_distance);
        assert!(!settings.allow_move);
    }

    #[test]
    fn test_state_bits_default() {
        let bits = SupportElementStateBits::default();

        assert!(!bits.to_buildplate);
        assert!(!bits.to_model_gracious);
        assert!(!bits.use_min_xy_dist);
        assert!(!bits.supports_roof);
        assert!(!bits.can_use_safe_radius);
        assert!(!bits.skip_ovalisation);
        assert!(!bits.lost);
        assert!(!bits.verylost);
        assert!(!bits.deleted);
        assert!(!bits.marked);
    }

    #[test]
    fn test_recommended_min_radius() {
        let settings = TreeSupportSettings::default();
        let min_r = settings.recommended_min_radius();

        // Should be at least min_radius
        assert!(min_r >= settings.min_radius);
        // Should be at least half line width
        assert!(min_r >= settings.support_line_width / 2);
    }

    #[test]
    fn test_get_actual_z() {
        let mut settings = TreeSupportSettings::default();
        settings.known_z = vec![scale(0.2), scale(0.4), scale(0.6)];

        assert_eq!(settings.get_actual_z(0), scale(0.2));
        assert_eq!(settings.get_actual_z(1), scale(0.4));
        assert_eq!(settings.get_actual_z(2), scale(0.6));

        // Out of bounds returns last
        assert_eq!(settings.get_actual_z(10), scale(0.6));
    }

    #[test]
    fn test_get_actual_z_empty() {
        let settings = TreeSupportSettings::default();

        // With empty known_z, should estimate based on layer height
        let z = settings.get_actual_z(0);
        assert_eq!(z, settings.layer_height);

        let z2 = settings.get_actual_z(4);
        assert_eq!(z2, settings.layer_height * 5);
    }
}
