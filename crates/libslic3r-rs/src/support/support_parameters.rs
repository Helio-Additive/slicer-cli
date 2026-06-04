//! Support generation parameters.
//!
//! C++ Reference:
//! - Support/SupportParameters.hpp
//!
//! This module defines the `SupportParameters` struct that aggregates all
//! configuration values needed for support generation. In the C++ code it
//! is initialized from `PrintObject` and derives flow rates, spacings,
//! angles, and style/pattern settings.
//!
//! NOTE: This file is not currently wired into the module tree. The active
//! support implementation lives in `support/mod.rs` with its own config types.

/// Support generation parameters derived from print configuration.
///
/// SupportParameters.hpp: struct SupportParameters
#[derive(Debug, Clone)]
pub struct SupportParameters {
    pub soluble_interface: bool,
    pub soluble_interface_non_soluble_base: bool,
    pub has_top_contacts: bool,
    pub has_bottom_contacts: bool,
    pub num_top_interface_layers: usize,
    pub num_bottom_interface_layers: usize,
    pub num_top_base_interface_layers: usize,
    pub num_bottom_base_interface_layers: usize,
    pub can_merge_support_regions: bool,
    pub support_layer_height_min: f64,
    pub gap_xy: f64,
    pub gap_xy_first_layer: f64,
    pub base_angle: f32,
    pub interface_angle: f32,
    pub interface_spacing: f64,
    pub interface_density: f64,
    pub raft_interface_density: f64,
    pub support_spacing: f64,
    pub support_density: f64,
    pub with_sheath: bool,
    pub independent_layer_height: bool,
    pub raft_angle_1st_layer: f32,
    pub raft_angle_base: f32,
    pub raft_angle_interface: f32,
    pub support_extrusion_width: f64,
    pub tree_branch_diameter_double_wall_area_scaled: f64,
    pub enable_support_ironing: bool,
    pub ironing_line_spacing: f64,
    pub ironing_flow_percent: f64,
    pub ironing_speed: f64,
    pub ironing_angle: f64,
    pub ironing_inset: f64,
}

impl SupportParameters {
    /// Create default support parameters.
    ///
    /// In the C++ code this constructor requires a PrintObject reference.
    /// This default provides zero/false values for all fields.
    pub fn new() -> Self {
        Self::default()
    }

    pub fn has_contacts(&self) -> bool {
        self.has_top_contacts || self.has_bottom_contacts
    }

    pub fn has_interfaces(&self) -> bool {
        self.num_top_interface_layers + self.num_bottom_interface_layers > 0
    }

    pub fn has_base_interfaces(&self) -> bool {
        self.num_top_base_interface_layers + self.num_bottom_base_interface_layers > 0
    }

    pub fn raft_interface_angle(&self, interface_id: usize) -> f32 {
        self.raft_angle_interface
            + if interface_id & 1 != 0 {
                -std::f32::consts::FRAC_PI_4
            } else {
                std::f32::consts::FRAC_PI_4
            }
    }
}

impl Default for SupportParameters {
    fn default() -> Self {
        Self {
            soluble_interface: false,
            soluble_interface_non_soluble_base: false,
            has_top_contacts: false,
            has_bottom_contacts: false,
            num_top_interface_layers: 0,
            num_bottom_interface_layers: 0,
            num_top_base_interface_layers: 0,
            num_bottom_base_interface_layers: 0,
            can_merge_support_regions: false,
            support_layer_height_min: 0.01,
            gap_xy: 0.0,
            gap_xy_first_layer: 0.0,
            base_angle: 0.0,
            interface_angle: 0.0,
            interface_spacing: 0.0,
            interface_density: 0.0,
            raft_interface_density: 0.0,
            support_spacing: 0.0,
            support_density: 0.0,
            with_sheath: false,
            independent_layer_height: false,
            raft_angle_1st_layer: 0.0,
            raft_angle_base: 0.0,
            raft_angle_interface: 0.0,
            support_extrusion_width: 0.0,
            tree_branch_diameter_double_wall_area_scaled: 0.0,
            enable_support_ironing: false,
            ironing_line_spacing: 0.0,
            ironing_flow_percent: 0.0,
            ironing_speed: 0.0,
            ironing_angle: 0.0,
            ironing_inset: 0.0,
        }
    }
}
