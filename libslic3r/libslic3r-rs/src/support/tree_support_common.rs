//! Common types for tree support generation.
//!
//! C++ Reference:
//! - Support/TreeSupportCommon.hpp
//!
//! Shared enums, settings, and utilities used by both 2D and 3D tree support
//! algorithms.
//!
//! NOTE: This file is not currently wired into the module tree. The active
//! tree support implementation lives in `support/tree_support_3d.rs` and
//! `support/tree_support_settings.rs`.

use crate::geometry::ExPolygons;

/// Preference for where to place support interface.
///
/// TreeSupportCommon.hpp: enum InterfacePreference
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterfacePreference {
    /// Always place interface.
    InterfaceAreaOverwritesSupport,
    /// Place interface only where needed.
    SupportAreaOverwritesInterface,
    /// Place interface and support independently.
    InterfacesAndSupportAreIndependent,
    /// No interface.
    Nothing,
}

impl Default for InterfacePreference {
    fn default() -> Self {
        Self::InterfaceAreaOverwritesSupport
    }
}

/// Priority for support interface placement.
///
/// TreeSupportCommon.hpp: enum support_interface_priority
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SupportInterfacePriority {
    AlwaysInterface,
    PreferInterface,
    PreferSupport,
    NeverInterface,
}

impl Default for SupportInterfacePriority {
    fn default() -> Self {
        Self::PreferInterface
    }
}

/// Status of a line during tree support line generation.
///
/// TreeSupportCommon.hpp: enum LineStatus
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineStatus {
    Invalid,
    ToModelGracious,
    ToModelGraciousSmall,
    ToBuildPlate,
    ToModelBut,
}

impl Default for LineStatus {
    fn default() -> Self {
        Self::Invalid
    }
}

/// Tree support mesh group settings.
///
/// TreeSupportCommon.hpp: class TreeSupportMeshGroupSettings
#[derive(Debug, Clone, Default)]
pub struct TreeSupportMeshGroupSettings {
    pub layer_height: f64,
    pub support_angle: f64,
    pub support_line_width: f64,
    pub support_roof_enable: bool,
    pub support_floor_enable: bool,
    pub support_roof_height: f64,
    pub support_floor_height: f64,
    pub support_xy_distance: f64,
    pub support_top_distance: f64,
    pub support_bottom_distance: f64,
}

impl TreeSupportMeshGroupSettings {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Aggregated tree support settings.
///
/// TreeSupportCommon.hpp: class TreeSupportSettings
#[derive(Debug, Clone, Default)]
pub struct TreeSupportSettings {
    pub branch_radius: f64,
    pub max_radius: f64,
    pub tip_layers: usize,
    pub branch_radius_increase_per_layer: f64,
    pub max_move_distance: f64,
    pub max_move_distance_slow: f64,
    pub support_bottom_layers: usize,
    pub support_roof_layers: usize,
    pub layer_height: f64,
    pub xy_distance: f64,
    pub bp_radius: f64,
    pub diameter_angle_scale: f64,
    pub min_radius: f64,
    pub support_rest_preference: InterfacePreference,
}

impl TreeSupportSettings {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Interface placement helper.
///
/// TreeSupportCommon.hpp: class InterfacePlacer
#[derive(Debug, Clone, Default)]
pub struct InterfacePlacer {
    pub interface_preference: InterfacePreference,
    pub support_polygons: ExPolygons,
    pub roof_polygons: ExPolygons,
}

impl InterfacePlacer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add roof polygons for a layer.
    /// TreeSupportCommon.hpp: add_roof()
    pub fn add_roof(&mut self, polygons: ExPolygons) {
        self.roof_polygons.extend(polygons);
    }

    /// Add roof polygons without checking bounds.
    /// TreeSupportCommon.hpp: add_roof_unguarded()
    pub fn add_roof_unguarded(&mut self, polygons: ExPolygons) {
        self.roof_polygons.extend(polygons);
    }

    /// Add roof polygons for build-plate-only support.
    /// TreeSupportCommon.hpp: add_roof_build_plate()
    pub fn add_roof_build_plate(&mut self, polygons: ExPolygons) {
        self.roof_polygons.extend(polygons);
    }
}

/// Show/log a tree support error message.
///
/// TreeSupportCommon.hpp: tree_supports_show_error()
pub fn tree_supports_show_error(_message: &str) {
    // In the C++ code this shows an error dialog. Here we just log.
    #[cfg(debug_assertions)]
    eprintln!("Tree support error: {}", _message);
}

/// Compute the ceiling layer index for a given z height.
///
/// TreeSupportCommon.hpp: layer_idx_ceil()
pub fn layer_idx_ceil(z: f64, layer_height: f64) -> usize {
    if layer_height <= 0.0 {
        return 0;
    }
    (z / layer_height).ceil() as usize
}
