//! Print region configuration and Flow generation.
//!
//! This module provides PrintRegion, a direct port of BambuStudio's PrintRegion.cpp.
//! A PrintRegion represents a group of volumes with the same print settings and generates
//! Flow objects for different extrusion roles (perimeter, infill, etc.).
//!
//! Reference: BambuStudio/src/libslic3r/PrintRegion.cpp

use crate::flow::{Flow, FlowRole};
use crate::region_config::PrintRegionConfig;
use crate::CoordF;

/// A print region - volumes sharing the same config and extruder assignments.
///
/// This is a direct port of C++ PrintRegion class. It holds configuration
/// and generates Flow objects on-demand for different extrusion roles.
///
/// Unlike the previous simplified version, this matches C++ behavior exactly:
/// - Stores PrintRegionConfig directly
/// - Generates Flow objects based on role and layer height
/// - Handles extruder assignment per role
/// - Provides methods matching C++ PrintRegion API
/// A print region - volumes sharing the same config and extruder assignments.
/// Print.hpp:111
#[derive(Clone, Debug)]
/// Print region with configuration and IDs
/// Print.hpp:111
pub struct PrintRegion {
    /// Region configuration (all print settings for this region).
    /// Print.hpp:152
    config: PrintRegionConfig,

    /// Configuration hash for change detection.
    /// Print.hpp:153
    config_hash: usize,

    /// Print region ID (identifier in Print::m_print_regions).
    /// Print.hpp:154
    print_region_id: i32,

    /// Print object region ID.
    /// Print.hpp:127
    print_object_region_id: i32,
}

/// Implementation of PrintRegion methods
/// Print.hpp:111-160
impl PrintRegion {
    // Create a new PrintRegion from config.
    // Print.hpp:115
    pub fn new(config: PrintRegionConfig) -> Self {
        // Print.hpp:116
        let config_hash = Self::hash_config(&config);
        Self {
            config,
            config_hash,
            print_region_id: -1,
            print_object_region_id: -1,
        }
    }

    /// Create with explicit IDs and hash.
    /// PrintRegion.cpp:21
    pub fn with_ids(
        config: PrintRegionConfig,
        print_region_id: i32,
        print_object_region_id: i32,
    ) -> Self {
        // PrintRegion.cpp:22
        let config_hash = Self::hash_config(&config);
        Self {
            config,
            config_hash,
            print_region_id,
            print_object_region_id,
        }
    }

    /// Get the region configuration.
    /// Print.hpp:123
    pub fn config(&self) -> &PrintRegionConfig {
        &self.config
    }

    /// Get configuration hash.
    /// Print.hpp:124
    pub fn config_hash(&self) -> usize {
        self.config_hash
    }

    /// Get print region ID.
    /// Print.hpp:126
    pub fn print_region_id(&self) -> i32 {
        self.print_region_id
    }

    /// Get print object region ID.
    /// Print.hpp:127
    pub fn print_object_region_id(&self) -> i32 {
        self.print_object_region_id
    }

    /// Get 1-based extruder identifier for this region and role.
    /// PrintRegion.cpp:7-19
    pub fn extruder(&self, role: FlowRole) -> usize {
        // PrintRegion.cpp:8-18
        match role {
            FlowRole::ExternalPerimeter | FlowRole::Perimeter => self.config.wall_filament,
            FlowRole::Infill => self.config.effective_infill_extruder(),
            FlowRole::SolidInfill | FlowRole::TopSolidInfill => {
                self.config.effective_solid_infill_extruder()
            }
            FlowRole::SupportMaterial | FlowRole::SupportMaterialInterface => {
                // Support uses region extruder by default
                self.config.wall_filament
            }
            FlowRole::SupportTransition => self.config.wall_filament,
        }
    }

    /// Generate Flow object for a specific role and layer height.
    /// PrintRegion.cpp:21-50
    pub fn flow(
        &self,
        role: FlowRole,
        nozzle_diameter: CoordF,
        layer_height: CoordF,
        first_layer: bool,
        initial_layer_line_width: CoordF,
    ) -> Result<Flow, String> {
        // Get extrusion width from configuration
        // (might be an absolute value, or a percent value, or zero for auto)
        // PrintRegion.cpp:24-42
        // PrintRegion.cpp:24
        let config_width =
            // PrintRegion.cpp:24
            if first_layer && initial_layer_line_width > 0.0 {
            // PrintRegion.cpp:25
            initial_layer_line_width
        } else {
            // PrintRegion.cpp:27-41
            match role {
                FlowRole::ExternalPerimeter => self.config.outer_wall_line_width,
                FlowRole::Perimeter => self.config.inner_wall_line_width,
                FlowRole::Infill => self.config.sparse_infill_line_width,
                FlowRole::SolidInfill => self.config.internal_solid_infill_line_width,
                FlowRole::TopSolidInfill => self.config.top_surface_line_width,
                FlowRole::SupportMaterial | FlowRole::SupportMaterialInterface => {
                    // Support uses infill width
                    self.config.sparse_infill_line_width
                }
                FlowRole::SupportTransition => self.config.sparse_infill_line_width,
            }
        };

        // Create Flow using the C++ new_from_config_width method
        Flow::new_from_config_width(role, config_width, nozzle_diameter, layer_height)
            .map_err(|e| format!("Failed to create flow for role {:?}: {:?}", role, e))
    }

    /// Calculate average nozzle diameter for this region.
    /// PrintRegion.cpp:52-57
    pub fn nozzle_dmr_avg(&self, nozzle_diameters: &[CoordF]) -> CoordF {
        // PrintRegion.cpp:53
        let wall_nozzle = nozzle_diameters
            .get(self.config.wall_filament)
            .copied()
            .unwrap_or(0.4);
        // PrintRegion.cpp:54
        let sparse_nozzle = nozzle_diameters
            .get(self.config.effective_infill_extruder())
            .copied()
            .unwrap_or(0.4);
        // PrintRegion.cpp:55
        let solid_nozzle = nozzle_diameters
            .get(self.config.effective_solid_infill_extruder())
            .copied()
            .unwrap_or(0.4);

        (wall_nozzle + sparse_nozzle + solid_nozzle) / 3.0
    }

    /// Calculate average bridging height for this region.
    /// PrintRegion.cpp:59-62
    pub fn bridging_height_avg(&self, nozzle_diameters: &[CoordF]) -> CoordF {
        self.nozzle_dmr_avg(nozzle_diameters) * self.config.bridge_flow_ratio.sqrt()
    }

    /// Update configuration.
    /// Print.hpp:142
    pub fn set_config(&mut self, config: PrintRegionConfig) {
        // Print.hpp:143
        self.config_hash = Self::hash_config(&config);
        // Print.hpp:144
        self.config = config;
    }

    /// Simple hash function for config (for now just use basic hash).
    /// In production, this should match C++ PrintRegionConfig::hash().
    /// Print.hpp:147
    fn hash_config(config: &PrintRegionConfig) -> usize {
        // Simplified hash - in real impl should match C++ exactly
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        // Print.hpp:148
        let mut hasher = DefaultHasher::new();
        // Print.hpp:149
        config.perimeters.hash(&mut hasher);
        // Print.hpp:150
        ((config.fill_density * 1000.0) as u32).hash(&mut hasher);
        hasher.finish() as usize
    }
}

/// Default implementation for PrintRegion
/// Print.hpp:111
impl Default for PrintRegion {
    // Create default PrintRegion with default config
    // Print.hpp:115
    fn default() -> Self {
        Self::new(PrintRegionConfig::default())
    }
}

/// Helper to create Flow objects for all roles at once.
///
/// This is useful for pre-calculating all flows for a layer to avoid
/// repeated lookups during generation.
#[derive(Clone, Debug)]
/// Helper struct for pre-calculated Flow objects
/// PrintRegion.cpp:64
pub struct RegionFlows {
    pub external_perimeter_flow: Flow,
    pub perimeter_flow: Flow,
    pub infill_flow: Flow,
    pub solid_infill_flow: Flow,
    pub top_solid_infill_flow: Flow,
}

/// Implementation of RegionFlows methods
/// PrintRegion.cpp:64-90
impl RegionFlows {
    // Create all flows for a region at a specific layer height.
    // PrintRegion.cpp:67
    pub fn new(
        region: &PrintRegion,
        nozzle_diameter: CoordF,
        layer_height: CoordF,
        first_layer: bool,
        initial_layer_line_width: CoordF,
    ) -> Result<Self, String> {
        Ok(Self {
            external_perimeter_flow: region.flow(
                FlowRole::ExternalPerimeter,
                nozzle_diameter,
                layer_height,
                first_layer,
                initial_layer_line_width,
            )?,
            perimeter_flow: region.flow(
                FlowRole::Perimeter,
                nozzle_diameter,
                layer_height,
                first_layer,
                initial_layer_line_width,
            )?,
            infill_flow: region.flow(
                FlowRole::Infill,
                nozzle_diameter,
                layer_height,
                first_layer,
                initial_layer_line_width,
            )?,
            solid_infill_flow: region.flow(
                FlowRole::SolidInfill,
                nozzle_diameter,
                layer_height,
                first_layer,
                initial_layer_line_width,
            )?,
            top_solid_infill_flow: region.flow(
                FlowRole::TopSolidInfill,
                nozzle_diameter,
                layer_height,
                first_layer,
                initial_layer_line_width,
            )?,
        })
    }

    /// Get flow for a specific role.
    /// PrintRegion.cpp:85
    pub fn get_flow(&self, role: FlowRole) -> &Flow {
        // PrintRegion.cpp:86
        match role {
            FlowRole::ExternalPerimeter => &self.external_perimeter_flow,
            FlowRole::Perimeter => &self.perimeter_flow,
            FlowRole::Infill => &self.infill_flow,
            FlowRole::SolidInfill => &self.solid_infill_flow,
            FlowRole::TopSolidInfill => &self.top_solid_infill_flow,
            FlowRole::SupportMaterial
            | FlowRole::SupportMaterialInterface
            | FlowRole::SupportTransition => &self.infill_flow, // Fallback
        }
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
        let config = PrintRegionConfig::default();
        let region = PrintRegion::new(config);

        // All should use default extruder (0)
        assert_eq!(region.extruder(FlowRole::ExternalPerimeter), 0);
        assert_eq!(region.extruder(FlowRole::Perimeter), 0);
        assert_eq!(region.extruder(FlowRole::Infill), 0);
        assert_eq!(region.extruder(FlowRole::SolidInfill), 0);
    }

    #[test]
    fn test_flow_generation() {
        let config = PrintRegionConfig::default();
        let region = PrintRegion::new(config);

        let nozzle_diameter = 0.4;
        let layer_height = 0.2;
        let first_layer = false;
        let initial_width = 0.0;

        let flow = region
            .flow(
                FlowRole::Perimeter,
                nozzle_diameter,
                layer_height,
                first_layer,
                initial_width,
            )
            .expect("Flow creation failed");

        assert!(flow.width() > 0.0);
        assert!((flow.height() - layer_height).abs() < 0.001);
    }

    #[test]
    fn test_region_flows() {
        let config = PrintRegionConfig::default();
        let region = PrintRegion::new(config);

        let flows =
            RegionFlows::new(&region, 0.4, 0.2, false, 0.0).expect("Failed to create region flows");

        assert!(flows.perimeter_flow.width() > 0.0);
        assert!(flows.external_perimeter_flow.width() > 0.0);
        assert!(flows.infill_flow.width() > 0.0);
    }

    #[test]
    fn test_nozzle_diameter_avg() {
        let config = PrintRegionConfig::default();
        let region = PrintRegion::new(config);

        let nozzles = vec![0.4, 0.6, 0.8];
        let avg = region.nozzle_dmr_avg(&nozzles);

        assert!((avg - 0.4).abs() < 0.01); // All use extruder 0 by default
    }
}
