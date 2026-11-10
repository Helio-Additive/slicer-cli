//! Print region configuration and Flow generation.
//!
//! Faithful 1:1 port of BambuStudio's `src/libslic3r/PrintRegion.cpp`.
//! A `PrintRegion` represents a group of volumes sharing the same print
//! settings (including the same assigned extruder(s)).
//!
//! Reference: BambuStudio/src/libslic3r/PrintRegion.cpp
//!
//! Porting notes (divergences forced by the current Rust crate state, see the
//! module-level documentation and the parity ledger):
//!   * C++ `PrintConfig::nozzle_diameter` / `filament_diameter` are
//!     `ConfigOptionFloats` (per-extruder vectors). The Rust `PrintConfig`
//!     models `nozzle_diameter` as a scalar, so `PrintRegion::flow`'s
//!     `get_at(extruder(role) - 1)` collapses onto a direct read; the
//!     `nozzle_dmr_avg` / `bridging_height_avg` helpers still take `&[f64]`
//!     slices (with the libslic3r `get_at` zeroth-element fallback).
//!   * `PrintRegion::flow(&PrintObject, ...)` keeps the C++ signature
//!     (PrintRegion.cpp:21), reaching `object.print().config()` and
//!     `object.config()` over the Arc-stamped config hierarchy. Its body is
//!     factored into the crate-private `flow_from_configs` so that
//!     `LayerRegion::flow` / `bridging_flow` (which hold config Arcs instead
//!     of a `&PrintObject`) can share it.

use crate::flow::{Flow, FlowRole};
use crate::region_config::PrintRegionConfig;
use crate::CoordF;

/// A PrintRegion object represents a group of volumes to print
/// sharing the same config (including the same assigned extruder(s))
/// Print.hpp:110-158
#[derive(Clone, Debug)]
pub struct PrintRegion {
    /// Print.hpp:153
    config: PrintRegionConfig,
    /// Print.hpp:154
    config_hash: usize,
    /// Identifier of this PrintRegion in the list of Print::m_print_regions.
    /// Print.hpp:155
    print_region_id: i32,
    /// Print.hpp:156
    print_object_region_id: i32,
}

impl PrintRegion {
    // Print.hpp:115 / PrintRegion ctor from config.
    pub fn new(config: PrintRegionConfig) -> Self {
        let config_hash = Self::hash_config(&config);
        Self {
            config,
            config_hash,
            print_region_id: -1,
            print_object_region_id: -1,
        }
    }

    // Print.hpp:117 : PrintRegion(const PrintRegionConfig &config, const size_t config_hash, int print_object_region_id = -1)
    pub fn with_ids(
        config: PrintRegionConfig,
        config_hash: usize,
        print_object_region_id: i32,
    ) -> Self {
        Self {
            config,
            config_hash,
            print_region_id: -1,
            print_object_region_id,
        }
    }

    /// Print.hpp:124
    pub fn config(&self) -> &PrintRegionConfig {
        &self.config
    }

    /// Print.hpp:125
    pub fn config_hash(&self) -> usize {
        self.config_hash
    }

    /// Identifier of this PrintRegion in the list of Print::m_print_regions.
    /// Print.hpp:127
    pub fn print_region_id(&self) -> i32 {
        self.print_region_id
    }

    /// Print.hpp:128
    pub fn print_object_region_id(&self) -> i32 {
        self.print_object_region_id
    }

    // 1-based extruder identifier for this region and role.
    // PrintRegion.cpp:7
    pub fn extruder(&self, role: FlowRole) -> Result<u32, String> {
        region_extruder(&self.config, role)
    }

    // PrintRegion.cpp:21
    // Faithful port of:
    //   Flow PrintRegion::flow(const PrintObject &object, FlowRole role, double layer_height, bool first_layer) const
    // (C++ declares the default argument `first_layer = false`, Print.hpp:131;
    // Rust callers pass it explicitly.)
    pub fn flow(
        &self,
        object: &crate::print_object::PrintObject,
        role: FlowRole,
        layer_height: CoordF,
        first_layer: bool,
    ) -> Result<Flow, String> {
        // PrintRegion.cpp:23
        // C++: const PrintConfig &print_config = object.print()->config();
        let print_config = object.print().config();
        // PrintRegion.cpp:24-49 — shared with LayerRegion::flow/bridging_flow,
        // which reach the same configs through their stored Arc snapshots.
        flow_from_configs(
            role,
            layer_height,
            first_layer,
            print_config.initial_layer_line_width,
            object.config().line_width,
            print_config.nozzle_diameter,
            &self.config,
        )
    }

    // PrintRegion.cpp:52
    // Average diameter of nozzles participating on extruding this region.
    pub fn nozzle_dmr_avg(&self, nozzle_diameters: &[CoordF]) -> CoordF {
        // PrintRegion.cpp:54
        (get_at(nozzle_diameters, self.config.wall_filament.wrapping_sub(1))
            + get_at(nozzle_diameters, self.config.sparse_infill_filament.wrapping_sub(1))
            + get_at(nozzle_diameters, self.config.solid_infill_filament.wrapping_sub(1)))
            / 3.
    }

    // PrintRegion.cpp:59
    // Average diameter of nozzles participating on extruding this region.
    pub fn bridging_height_avg(&self, nozzle_diameters: &[CoordF]) -> CoordF {
        // PrintRegion.cpp:61
        self.nozzle_dmr_avg(nozzle_diameters) * self.config.bridge_flow_ratio.sqrt()
    }

    // PrintRegion.cpp:64
    // Collect 0-based extruder indices used to print this region's object.
    //
    // Static helper. The C++ derives `num_extruders` from
    // `print_config.filament_diameter.size()`; the Rust `PrintConfig` models
    // `filament_diameter` as a scalar, so `num_extruders` is taken from the
    // supplied `num_extruders` argument (the caller passes the number of
    // configured printer extruders). The branch predicates use the equivalent
    // Rust `PrintRegionConfig` field names:
    //   wall_loops            -> perimeters
    //   sparse_infill_density -> fill_density
    //   top_shell_layers      -> top_solid_layers
    //   bottom_shell_layers   -> bottom_solid_layers
    pub fn collect_object_printing_extruders_static(
        num_extruders: i32,
        region_config: &PrintRegionConfig,
        has_brim: bool,
        object_extruders: &mut Vec<u32>,
    ) {
        // These checks reflect the same logic used in the GUI for enabling/disabling extruder selection fields.
        // BBS
        // PrintRegion.cpp:69
        let emplace_extruder = |extruder_id: i32, object_extruders: &mut Vec<u32>| {
            // PrintRegion.cpp:70
            let i = std::cmp::max(0, extruder_id - 1);
            // PrintRegion.cpp:71
            object_extruders.push(if i >= num_extruders { 0 } else { i as u32 });
        };
        // PrintRegion.cpp:73
        if region_config.perimeters > 0 || has_brim {
            // PrintRegion.cpp:74
            emplace_extruder(region_config.wall_filament as i32, object_extruders);
        }
        // PrintRegion.cpp:75
        if region_config.fill_density > 0.0 {
            // PrintRegion.cpp:76
            emplace_extruder(region_config.sparse_infill_filament as i32, object_extruders);
        }
        // PrintRegion.cpp:77
        if region_config.top_solid_layers > 0 || region_config.bottom_solid_layers > 0 {
            // PrintRegion.cpp:78
            emplace_extruder(region_config.solid_infill_filament as i32, object_extruders);
        }
    }

    // PrintRegion.cpp:81
    // Collect 0-based extruder indices used to print this region's object.
    //
    // Member overload. C++ reads `print.config()` and `print.has_brim()`; the
    // Rust caller supplies `num_extruders` (number of configured printer
    // extruders, == `print.config().filament_diameter.size()`) and `has_brim`
    // directly. The `#ifndef NDEBUG` asserts validate that each region's
    // filament index is within range.
    pub fn collect_object_printing_extruders(
        &self,
        num_extruders: i32,
        has_brim: bool,
        object_extruders: &mut Vec<u32>,
    ) {
        // PrintRegion, if used by some PrintObject, shall have all the extruders set to an existing printer extruder.
        // If not, then there must be something wrong with the Print::apply() function.
        // PrintRegion.cpp:85 (#ifndef NDEBUG)
        // BBS
        // PrintRegion.cpp:88
        debug_assert!(self.config().wall_filament as i32 <= num_extruders);
        // PrintRegion.cpp:89
        debug_assert!(self.config().sparse_infill_filament as i32 <= num_extruders);
        // PrintRegion.cpp:90
        debug_assert!(self.config().solid_infill_filament as i32 <= num_extruders);
        // PrintRegion.cpp:92
        Self::collect_object_printing_extruders_static(
            num_extruders,
            self.config(),
            has_brim,
            object_extruders,
        );
    }

    // Print.hpp:143 : void set_config(const PrintRegionConfig &config)
    pub fn set_config(&mut self, config: PrintRegionConfig) {
        // Print.hpp:143
        self.config_hash = Self::hash_config(&config);
        self.config = config;
    }

    // Local stand-in for `PrintRegionConfig::hash()` (ConfigBase::hash()).
    // The Rust `PrintRegionConfig` does not yet expose a faithful config hash;
    // a stable hash over the change-relevant fields is used so that
    // `config_hash` mirrors C++ usage (change detection). NOTE: this is NOT
    // byte-identical to the C++ ConfigBase::hash() and must not be relied on
    // for cross-language equality. See parity ledger.
    fn hash_config(config: &PrintRegionConfig) -> usize {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        let mut hasher = DefaultHasher::new();
        config.perimeters.hash(&mut hasher);
        ((config.fill_density * 1000.0) as u32).hash(&mut hasher);
        hasher.finish() as usize
    }
}

// PrintRegion.cpp:7-19 — the body of `PrintRegion::extruder`, factored over a
// raw `PrintRegionConfig` so that `flow_from_configs` (which has no
// `&PrintRegion`) can reproduce the PrintRegion.cpp:48 role check.
pub(crate) fn region_extruder(
    config: &PrintRegionConfig,
    role: FlowRole,
) -> Result<u32, String> {
    // PrintRegion.cpp:9
    let extruder: usize;
    // PrintRegion.cpp:10
    if role == FlowRole::Perimeter || role == FlowRole::ExternalPerimeter {
        // PrintRegion.cpp:11
        extruder = config.wall_filament;
    // PrintRegion.cpp:12
    } else if role == FlowRole::Infill {
        // PrintRegion.cpp:13
        extruder = config.sparse_infill_filament;
    // PrintRegion.cpp:14
    } else if role == FlowRole::SolidInfill || role == FlowRole::TopSolidInfill {
        // PrintRegion.cpp:15
        extruder = config.solid_infill_filament;
    // PrintRegion.cpp:16
    } else {
        // PrintRegion.cpp:17
        return Err("Unknown role".to_string());
    }
    // PrintRegion.cpp:18
    Ok(extruder as u32)
}

// PrintRegion.cpp:24-49 — the config-level body of `PrintRegion::flow`,
// factored out so that `LayerRegion::flow` / `bridging_flow` (which reach the
// configs through their stored Arc snapshots rather than a `&PrintObject`)
// can share it. Crate-private: the public entry points are
// `PrintRegion::flow(&PrintObject, ...)` and the `LayerRegion` members.
//   * `initial_layer_line_width` = print_config.initial_layer_line_width
//   * `object_line_width`        = object.config().line_width
//   * `nozzle_diameter`          = print_config.nozzle_diameter (the C++
//     per-extruder `get_at(extruder(role) - 1)` collapses onto this crate's
//     scalar field; the extruder(role) role check is still evaluated for its
//     Unknown-role error semantics).
pub(crate) fn flow_from_configs(
    role: FlowRole,
    layer_height: CoordF,
    first_layer: bool,
    initial_layer_line_width: CoordF,
    object_line_width: CoordF,
    nozzle_diameter: CoordF,
    region_config: &PrintRegionConfig,
) -> Result<Flow, String> {
    // PrintRegion.cpp:24
    let mut config_width: CoordF;
    // Get extrusion width from configuration.
    // (might be an absolute value, or a percent value, or zero for auto)
    // PrintRegion.cpp:27
    if first_layer && initial_layer_line_width > 0.0 {
        // PrintRegion.cpp:28
        config_width = initial_layer_line_width;
    // PrintRegion.cpp:29
    } else if role == FlowRole::ExternalPerimeter {
        // PrintRegion.cpp:30
        config_width = region_config.outer_wall_line_width;
    // PrintRegion.cpp:31
    } else if role == FlowRole::Perimeter {
        // PrintRegion.cpp:32
        config_width = region_config.inner_wall_line_width;
    // PrintRegion.cpp:33
    } else if role == FlowRole::Infill {
        // PrintRegion.cpp:34
        config_width = region_config.sparse_infill_line_width;
    // PrintRegion.cpp:35
    } else if role == FlowRole::SolidInfill {
        // PrintRegion.cpp:36
        config_width = region_config.internal_solid_infill_line_width;
    // PrintRegion.cpp:37
    } else if role == FlowRole::TopSolidInfill {
        // PrintRegion.cpp:38
        config_width = region_config.top_surface_line_width;
    // PrintRegion.cpp:39
    } else {
        // PrintRegion.cpp:40
        return Err("Unknown role".to_string());
    }

    // PrintRegion.cpp:43
    if config_width == 0.0 {
        // PrintRegion.cpp:44
        config_width = object_line_width;
    }

    // Get the configured nozzle_diameter for the extruder associated to the flow role requested.
    // Here this->extruder(role) - 1 may underflow to MAX_INT, but then the get_at() will follback to zero'th element, so everything is all right.
    // PrintRegion.cpp:48
    let _ = region_extruder(region_config, role)?;
    // C++ casts nozzle_diameter to `float` (PrintRegion.cpp:48) and layer_height
    // to `float(layer_height)` (PrintRegion.cpp:49) — `new_from_config_width`
    // takes `float nozzle_diameter, float height` (Flow.hpp:107). Reproduce the
    // f32 narrowing locally before widening back to the crate's CoordF (f64).
    let nozzle_diameter = nozzle_diameter as f32;
    let height = layer_height as f32;
    // PrintRegion.cpp:49
    Flow::new_from_config_width(
        role,
        config_width,
        nozzle_diameter as CoordF,
        height as CoordF,
    )
    .map_err(|e| format!("{:?}", e))
}

// `get_at` reproduces Slic3r::ConfigOptionVector::get_at: the value at index
// `i`, or the zeroth element when `i` is out of range (and `0` when empty).
// ConfigOptionVector models. See PrintRegion.cpp:48 commentary.
#[inline]
fn get_at(values: &[CoordF], i: usize) -> CoordF {
    if values.is_empty() {
        0.
    } else if i < values.len() {
        values[i]
    } else {
        values[0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_print_region_new() {
        let config = PrintRegionConfig::default();
        let region = PrintRegion::new(config);
        assert_eq!(region.print_region_id(), -1);
        assert_eq!(region.config().perimeters, 3);
    }

    #[test]
    fn test_extruder_assignment() {
        let mut config = PrintRegionConfig::default();
        config.wall_filament = 1;
        config.sparse_infill_filament = 2;
        config.solid_infill_filament = 3;
        let region = PrintRegion::new(config);

        assert_eq!(region.extruder(FlowRole::ExternalPerimeter).unwrap(), 1);
        assert_eq!(region.extruder(FlowRole::Perimeter).unwrap(), 1);
        assert_eq!(region.extruder(FlowRole::Infill).unwrap(), 2);
        assert_eq!(region.extruder(FlowRole::SolidInfill).unwrap(), 3);
        assert_eq!(region.extruder(FlowRole::TopSolidInfill).unwrap(), 3);
        // Roles outside the C++ switch throw "Unknown role".
        assert!(region.extruder(FlowRole::SupportMaterial).is_err());
    }

    #[test]
    fn test_flow_generation() {
        // Exercises the shared config-level core directly; the public
        // `PrintRegion::flow(&PrintObject, ...)` is a thin reader over it.
        let config = PrintRegionConfig::default();
        let flow = flow_from_configs(FlowRole::Perimeter, 0.2, false, 0.0, 0.45, 0.4, &config)
            .expect("Flow creation failed");

        assert!(flow.width() > 0.0);
        assert!((flow.height() - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_nozzle_diameter_avg() {
        let mut config = PrintRegionConfig::default();
        config.wall_filament = 1;
        config.sparse_infill_filament = 1;
        config.solid_infill_filament = 1;
        let region = PrintRegion::new(config);

        let nozzles = vec![0.4, 0.6, 0.8];
        let avg = region.nozzle_dmr_avg(&nozzles);
        assert!((avg - 0.4).abs() < 0.01); // all use extruder 1 (index 0)
    }

    #[test]
    fn test_collect_object_printing_extruders() {
        let mut config = PrintRegionConfig::default();
        config.wall_filament = 1;
        config.sparse_infill_filament = 2;
        config.solid_infill_filament = 3;
        config.perimeters = 2;
        config.fill_density = 0.15;
        config.top_solid_layers = 3;
        config.bottom_solid_layers = 3;
        let region = PrintRegion::new(config);

        let mut extruders = Vec::new();
        region.collect_object_printing_extruders(8, false, &mut extruders);
        // 0-based indices: wall(1)->0, sparse(2)->1, solid(3)->2
        assert_eq!(extruders, vec![0u32, 1u32, 2u32]);
    }
}
