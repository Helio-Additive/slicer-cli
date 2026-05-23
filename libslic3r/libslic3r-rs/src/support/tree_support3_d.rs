//! Tree support 3D algorithm types.
//!
//! C++ Reference:
//! - Support/TreeSupport3D.hpp
//! - Support/TreeSupport3D.cpp
//!
//! This module defines the data structures for the 3D tree support algorithm
//! by Thomas Rahm (based on CuraEngine). The algorithm uses support elements
//! with area-based collision avoidance, growing from tips downward and merging
//! branches.
//!
//! NOTE: This file is not currently wired into the module tree. The active
//! 3D tree support implementation lives in `support/tree_support_3d.rs`.

use crate::geometry::{ExPolygons, Point};
use crate::Coord;

/// Settings for how support element areas increase as they grow downward.
///
/// TreeSupport3D.hpp: struct AreaIncreaseSettings
#[derive(Debug, Clone, Default)]
pub struct AreaIncreaseSettings {
    pub increase_speed: Coord,
    pub increase_radius: bool,
    pub no_error: bool,
    pub use_min_distance: bool,
    pub do_move: bool,
}

impl AreaIncreaseSettings {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Bit flags for support element state.
///
/// TreeSupport3D.hpp: struct SupportElementStateBits
#[derive(Debug, Clone, Default)]
pub struct SupportElementStateBits {
    pub to_buildplate: bool,
    pub to_model_gracious: bool,
    pub to_model_gracious_small: bool,
    pub use_min_xy_dist: bool,
    pub supports_roof: bool,
    pub can_use_safe_radius: bool,
    pub skip_ovalisation: bool,
    pub deleted: bool,
    pub marked: bool,
}

impl SupportElementStateBits {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Full state of a support element during tree growth.
///
/// TreeSupport3D.hpp: struct SupportElementState
#[derive(Debug, Clone, Default)]
pub struct SupportElementState {
    pub target_height: Coord,
    pub target_position: Point,
    pub next_position: Point,
    pub layer_idx: usize,
    pub effective_radius_height: Coord,
    pub distance_to_top: usize,
    pub elephant_foot_increases: usize,
    pub increase_settings: AreaIncreaseSettings,
    pub bits: SupportElementStateBits,
    pub result_on_layer: ExPolygons,
}

impl SupportElementState {
    pub fn new() -> Self {
        Self::default()
    }
}

/// A support element combining state with its computed area polygon.
///
/// TreeSupport3D.hpp: struct SupportElement
#[derive(Debug, Clone, Default)]
pub struct SupportElement {
    pub state: SupportElementState,
    pub area: ExPolygons,
    pub parents: Vec<usize>,
}

impl SupportElement {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Compute the support element radius for a given configuration.
///
/// TreeSupport3D.hpp: support_element_radius()
pub fn support_element_radius(
    branch_radius: f64,
    distance_to_top: usize,
    tip_layers: usize,
    diameter_angle_scale: f64,
    branch_radius_increase_per_layer: f64,
) -> f64 {
    if distance_to_top <= tip_layers {
        return branch_radius;
    }
    let layers_above_tip = (distance_to_top - tip_layers) as f64;
    branch_radius + layers_above_tip * branch_radius_increase_per_layer * diameter_angle_scale
}

/// Compute the collision radius for a support element.
///
/// TreeSupport3D.hpp: support_element_collision_radius()
pub fn support_element_collision_radius(
    element_radius: f64,
    xy_distance: f64,
) -> f64 {
    element_radius + xy_distance
}

/// Reset the result_on_layer field for a support element.
///
/// TreeSupport3D.hpp: result_on_layer_reset()
pub fn result_on_layer_reset(element: &mut SupportElementState) {
    element.result_on_layer.clear();
}

/// Show/log a tree support error message.
///
/// TreeSupport3D.hpp: tree_supports_show_error()
pub fn tree_supports_show_error(_message: &str) {
    #[cfg(debug_assertions)]
    eprintln!("Tree support 3D error: {}", _message);
}
