//! Extruder state management for G-code generation
//!
//! C++ Reference: Extruder.hpp, Extruder.cpp
//!
//! This module tracks the state of an extruder during G-code generation,
//! including E axis position, retraction state, and filament parameters.

use crate::print_config::PrintConfig;
use std::f64::consts::PI;

/// Static shared state for multi-material single-extruder machines
/// Extruder.cpp:5-6
static mut SHARE_E: [f64; 2] = [0.0, 0.0];
static mut SHARE_RETRACTED: [f64; 2] = [0.0, 0.0];

/// Extruder state tracker
/// Extruder.hpp:11-88
#[derive(Debug)]
pub struct Extruder {
    // Print-wide global ID of this extruder
    // Extruder.hpp:75
    id: u32,

    // Reference to G-code configuration
    // Extruder.hpp:73-74
    config: *const PrintConfig,

    // Current state of the extruder axis (may be reset if using relative E)
    // Extruder.hpp:76-77
    e: f64,

    // Current state of the extruder tachometer for statistics
    // Extruder.hpp:78-79
    absolute_e: f64,

    // Current positive amount of retraction
    // Extruder.hpp:80-81
    retracted: f64,

    // Extra amount of priming on deretraction
    // Extruder.hpp:82-83
    pub restart_extra: f64,

    // Cached E per mm³ conversion factor
    // Extruder.hpp:84
    e_per_mm3: f64,

    // Whether this is a shared extruder for multi-material
    // Extruder.hpp:87
    share_extruder: bool,
}

impl Extruder {
    // Create a new extruder with the given ID and configuration
    // Extruder.cpp:8-18
    pub fn new(id: u32, config: *const PrintConfig, share_extruder: bool) -> Self {
        let mut extruder = Self {
            id,
            config,
            e: 0.0,
            absolute_e: 0.0,
            retracted: 0.0,
            restart_extra: 0.0,
            e_per_mm3: 0.0,
            share_extruder,
        };

        // Reset to initial state
        // Extruder.cpp:14
        extruder.reset();

        // Cache E per mm³ conversion factor
        // Extruder.cpp:15-17
        extruder.e_per_mm3 = extruder.filament_flow_ratio();
        extruder.e_per_mm3 /= extruder.filament_crossection();

        extruder
    }

    // Reset extruder state
    // Extruder.hpp:17-27
    pub fn reset(&mut self) {
        if self.share_extruder {
            // Reset shared extruder state
            // Extruder.hpp:19-21
            unsafe {
                SHARE_E = [0.0, 0.0];
                SHARE_RETRACTED = [0.0, 0.0];
            }
        } else {
            // Reset single extruder state
            // Extruder.hpp:22-24
            self.e = 0.0;
            self.retracted = 0.0;
        }

        // Reset common state
        // Extruder.hpp:25-26
        self.restart_extra = 0.0;
        self.absolute_e = 0.0;
    }

    // Get the extruder ID
    // Extruder.hpp:29
    pub fn id(&self) -> u32 {
        self.id
    }

    // Get the actual extruder ID (mapped through filament map)
    // Extruder.cpp:20-26
    // TODO: Implement filament_map support in PrintConfig
    pub fn extruder_id(&self) -> usize {
        // Stub: Return self.id for now until filament_map is added to PrintConfig
        self.id as usize
    }

    // Extrude or retract filament
    ///
    // # Arguments
    // * `de` - Amount to extrude (positive) or retract (negative)
    ///
    // # Returns
    // The actual amount extruded/retracted
    ///
    // Extruder.cpp:28-48
    pub fn extrude(&mut self, de: f64) -> f64 {
        let config = unsafe { &*self.config };

        if self.share_extruder {
            // Shared extruder mode
            // Extruder.cpp:30-38
            let extruder_id = self.extruder_id();
            unsafe {
                if config.use_relative_e {
                    SHARE_E[extruder_id] = 0.0;
                }
                SHARE_E[extruder_id] += de;
                self.absolute_e += de;
                if de < 0.0 {
                    SHARE_RETRACTED[extruder_id] -= de;
                }
            }
        } else {
            // Single extruder mode
            // Extruder.cpp:39-47
            if config.use_relative_e {
                self.e = 0.0;
            }
            self.e += de;
            self.absolute_e += de;
            if de < 0.0 {
                self.retracted -= de;
            }
        }

        de
    }

    // Retract filament by the specified length
    ///
    // If already retracted by the same or greater amount, this is a no-op.
    ///
    // # Arguments
    // * `length` - Amount to retract
    // * `restart_extra` - Extra length for unretraction
    ///
    // # Returns
    // The actual amount retracted
    ///
    // Extruder.cpp:50-81
    pub fn retract(&mut self, length: f64, restart_extra: f64) -> f64 {
        let config = unsafe { &*self.config };

        if self.share_extruder {
            // Shared extruder retraction
            // Extruder.cpp:52-65
            let extruder_id = self.extruder_id();
            unsafe {
                if config.use_relative_e {
                    SHARE_E[extruder_id] = 0.0;
                }
                let to_retract = (length - SHARE_RETRACTED[extruder_id]).max(0.0);
                self.restart_extra = restart_extra;
                if to_retract > 0.0 {
                    SHARE_E[extruder_id] -= to_retract;
                    self.absolute_e -= to_retract;
                    SHARE_RETRACTED[extruder_id] += to_retract;
                }
                to_retract
            }
        } else {
            // Single extruder retraction
            // Extruder.cpp:66-80
            if config.use_relative_e {
                self.e = 0.0;
            }
            let to_retract = (length - self.retracted).max(0.0);
            self.restart_extra = restart_extra;
            if to_retract > 0.0 {
                self.e -= to_retract;
                self.absolute_e -= to_retract;
                self.retracted += to_retract;
            }
            to_retract
        }
    }

    // Unretract filament (reverse retraction)
    ///
    // # Returns
    // The amount of filament unretracted
    ///
    // Extruder.cpp:83-97
    pub fn unretract(&mut self) -> f64 {
        if self.share_extruder {
            // Shared extruder unretraction
            // Extruder.cpp:85-91
            let extruder_id = self.extruder_id();
            let de = unsafe { SHARE_RETRACTED[extruder_id] } + self.restart_extra;
            self.extrude(de);
            unsafe {
                SHARE_RETRACTED[extruder_id] = 0.0;
            }
            self.restart_extra = 0.0;
            de
        } else {
            // Single extruder unretraction
            // Extruder.cpp:92-96
            let de = self.retracted + self.restart_extra;
            self.extrude(de);
            self.retracted = 0.0;
            self.restart_extra = 0.0;
            de
        }
    }

    // Get current E position
    // Extruder.hpp:35
    pub fn e(&self) -> f64 {
        if self.share_extruder {
            let extruder_id = self.extruder_id();
            unsafe { SHARE_E[extruder_id] }
        } else {
            self.e
        }
    }

    // Reset E position to zero
    // Extruder.hpp:36
    pub fn reset_e(&mut self) {
        self.e = 0.0;
        let extruder_id = self.extruder_id();
        unsafe {
            SHARE_E[extruder_id] = 0.0;
        }
    }

    // Calculate E axis movement per mm of extrusion
    // Extruder.hpp:37
    pub fn e_per_mm(&self, mm3_per_mm: f64) -> f64 {
        mm3_per_mm * self.e_per_mm3
    }

    // Get E per mm³ conversion factor
    // Extruder.hpp:38
    pub fn e_per_mm3(&self) -> f64 {
        self.e_per_mm3
    }

    // Get total extruded volume in mm³
    // Extruder.cpp:99-108
    pub fn extruded_volume(&self) -> f64 {
        self.used_filament() * self.filament_crossection()
    }

    // Get total used filament length in mm
    // Extruder.cpp:110-120
    pub fn used_filament(&self) -> f64 {
        if self.share_extruder {
            // FIXME: need to count retracted length for share-extruder machine
            // Extruder.cpp:113-114
            self.absolute_e
        } else {
            // Extruder.cpp:116
            self.absolute_e + self.retracted
        }
    }

    // Get the filament diameter in mm
    // Extruder.cpp:129-132
    // TODO: Implement filament_diameter array in PrintConfig
    pub fn filament_diameter(&self) -> f64 {
        // Stub: Return standard 1.75mm for now
        1.75
    }

    // Get filament cross-sectional area in mm²
    // Extruder.hpp:43
    pub fn filament_crossection(&self) -> f64 {
        let d = self.filament_diameter();
        d * d * 0.25 * PI
    }

    // Get the filament density in g/cm³
    // Extruder.cpp:134-137
    // TODO: Implement filament_density array in PrintConfig
    pub fn filament_density(&self) -> f64 {
        // Stub: Return PLA density (1.24 g/cm³) for now
        1.24
    }

    // Get the filament cost per kg
    // Extruder.cpp:139-142
    // TODO: Implement filament_cost array in PrintConfig
    pub fn filament_cost(&self) -> f64 {
        // Stub: Return $20/kg for now
        20.0
    }

    // Get filament flow ratio (extrusion multiplier)
    // Get the flow ratio multiplier
    // Extruder.cpp:144-147
    // TODO: Implement filament_flow_ratio array in PrintConfig
    pub fn filament_flow_ratio(&self) -> f64 {
        // Stub: Return 1.0 (100% flow) for now
        1.0
    }

    // Get retract before wipe as a factor (0.0 to 1.0)
    // Extruder.cpp:142-146
    // TODO: Implement retract_before_wipe array in PrintConfig
    pub fn retract_before_wipe(&self) -> f64 {
        // Stub: Return 0 (no retract before wipe) for now
        0.0
    }

    // Get the retraction length in mm
    // Extruder.cpp:149-152
    // TODO: Implement retraction_length array in PrintConfig
    pub fn retraction_length(&self) -> f64 {
        // Stub: Return 0.4mm for now
        0.4
    }

    // Get retract lift (Z hop) in mm
    // Extruder.cpp:153-156
    // TODO: Implement z_hop array in PrintConfig
    pub fn retract_lift(&self) -> f64 {
        // Stub: Return 0 (no Z hop) for now
        0.0
    }

    // Get the retraction speed in mm/s
    // Extruder.cpp:154-157
    // TODO: Implement retraction_speed array in PrintConfig
    pub fn retraction_speed(&self) -> f64 {
        // Stub: Return 40mm/s for now
        40.0
    }

    // Get deretraction speed in mm/s
    // Extruder.cpp:159-162
    // TODO: Implement deretraction_speed array in PrintConfig
    pub fn deretraction_speed(&self) -> f64 {
        // Stub: Return same as retraction speed for now
        self.retraction_speed()
    }

    // Get extra restart length after retraction in mm
    // Get extra length to add during unretraction
    // Extruder.cpp:164-167
    // TODO: Implement retract_restart_extra array in PrintConfig
    pub fn retract_restart_extra(&self) -> f64 {
        // Stub: Return 0 for now
        0.0
    }

    // Get retraction length for tool change in mm
    // Extruder.cpp:169-172
    // TODO: Implement retract_length_toolchange array in PrintConfig
    pub fn retract_length_toolchange(&self) -> f64 {
        // Stub: Return 0 for now
        0.0
    }

    // Get extra restart length after tool change in mm
    // Extruder.cpp:174-177
    // TODO: Implement retract_restart_extra_toolchange array in PrintConfig
    pub fn retract_restart_extra_toolchange(&self) -> f64 {
        // Stub: Return 0 for now
        0.0
    }

    // Check if this is a shared extruder
    // Extruder.hpp:62
    pub fn is_share_extruder(&self) -> bool {
        self.share_extruder
    }

    // Get single extruder retracted length
    // Extruder.hpp:63
    pub fn get_single_retracted_length(&self) -> f64 {
        self.retracted
    }

    // Get shared extruder retracted length
    // Extruder.hpp:64
    pub fn get_share_retracted_length(&self) -> f64 {
        let extruder_id = self.extruder_id();
        unsafe { SHARE_RETRACTED[extruder_id] }
    }
}

/// Comparison operators for Extruder
/// Extruder.hpp:91-94
impl PartialEq for Extruder {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for Extruder {}

impl PartialOrd for Extruder {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Extruder {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.id.cmp(&other.id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Create a mock GCodeConfig for testing
    fn mock_config() -> GCodeConfig {
        GCodeConfig {
            filament_map: vec![1, 2],
            use_relative_e_distances: false,
            filament_diameter: vec![1.75, 1.75],
            filament_density: vec![1.24, 1.24],
            filament_cost: vec![20.0, 20.0],
            filament_flow_ratio: vec![1.0, 1.0],
            retract_before_wipe: vec![0.0, 0.0],
            retraction_length: vec![0.8, 0.8],
            z_hop: vec![0.0, 0.0],
            retraction_speed: vec![40.0, 40.0],
            deretraction_speed: vec![0.0, 0.0],
            retract_restart_extra: vec![0.0, 0.0],
            retract_length_toolchange: vec![10.0, 10.0],
            retract_restart_extra_toolchange: vec![0.0, 0.0],
        }
    }

    #[test]
    fn test_extruder_creation() {
        let config = mock_config();
        let extruder = Extruder::new(0, &config, false);

        assert_eq!(extruder.id(), 0);
        assert_eq!(extruder.e(), 0.0);
        assert!(!extruder.is_share_extruder());
    }

    #[test]
    fn test_extrude() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);

        let extruded = extruder.extrude(10.0);
        assert_eq!(extruded, 10.0);
        assert_eq!(extruder.e(), 10.0);
    }

    #[test]
    fn test_retract() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);

        // Extrude first
        extruder.extrude(10.0);

        // Retract
        let retracted = extruder.retract(2.0, 0.1);
        assert_eq!(retracted, 2.0);
    }

    #[test]
    fn test_unretract() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);

        // Extrude, retract, then unretract
        extruder.extrude(10.0);
        extruder.retract(2.0, 0.1);
        let unretracted = extruder.unretract();

        assert_eq!(unretracted, 2.1); // 2.0 retracted + 0.1 extra
    }

    #[test]
    fn test_reset() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);

        extruder.extrude(10.0);
        extruder.retract(2.0, 0.1);

        extruder.reset();

        assert_eq!(extruder.e(), 0.0);
    }

    #[test]
    fn test_filament_crossection() {
        let config = mock_config();
        let extruder = Extruder::new(0, &config, false);

        let crossection = extruder.filament_crossection();
        let expected = 1.75 * 1.75 * 0.25 * PI;

        assert!((crossection - expected).abs() < 0.001);
    }

    #[test]
    fn test_extruder_ordering() {
        let config = mock_config();
        let e1 = Extruder::new(0, &config, false);
        let e2 = Extruder::new(1, &config, false);

        assert!(e1 < e2);
        assert_eq!(e1, e1);
        assert_ne!(e1, e2);
    }

    #[test]
    fn test_used_filament() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);

        extruder.extrude(10.0);
        extruder.retract(2.0, 0.0);

        // Used filament = absolute_e + retracted = 8.0 + 2.0 = 10.0
        assert_eq!(extruder.used_filament(), 10.0);
    }

    #[test]
    fn test_e_per_mm() {
        let config = mock_config();
        let extruder = Extruder::new(0, &config, false);

        let e = extruder.e_per_mm(0.5);
        assert!(e > 0.0);
    }
}
