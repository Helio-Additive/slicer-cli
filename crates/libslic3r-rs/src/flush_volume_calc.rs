//! Flush volume calculator.
//!
//! Calculates the amount of filament to flush during tool changes,
//! mirroring BambuStudio's FlushVolCalc.cpp.

/// Derive traits for FlushVolumeCalculator
/// FlushVolCalc.hpp:8-9
#[derive(Clone, Debug)]
/// Calculator for flush volumes during tool changes
/// FlushVolCalc.hpp:10-30
pub struct FlushVolumeCalculator {
    /// Base flush volume in mm³
    /// FlushVolCalc.hpp:12-13
    base_flush_volume: f64,
    /// Multiplier for similar materials
    /// FlushVolCalc.hpp:14-15
    similar_material_multiplier: f64,
    /// Multiplier for different materials
    /// FlushVolCalc.hpp:16-17
    different_material_multiplier: f64,
}

/// Implementation of flush volume calculation methods
/// FlushVolCalc.cpp:15-120
impl FlushVolumeCalculator {
    // Create a new flush volume calculator with default values
    // FlushVolCalc.cpp:18-25
    pub fn new() -> Self {
        // Initialize flush volume calculator with default values
        // FlushVolCalc.cpp:19-23
        Self {
            base_flush_volume: 100.0,
            similar_material_multiplier: 1.0,
            different_material_multiplier: 3.0,
        }
    }

    /// Calculate flush volume for a tool change
    /// FlushVolCalc.cpp:28-38
    pub fn calculate(&self, from_filament: &FilamentInfo, to_filament: &FilamentInfo) -> f64 {
        // Check if filaments are similar and use appropriate multiplier
        // FlushVolCalc.cpp:29-34
        if self.are_similar(from_filament, to_filament) {
            self.base_flush_volume * self.similar_material_multiplier
        } else {
            self.base_flush_volume * self.different_material_multiplier
        }
    }

    /// Check if two filaments are similar enough to reduce flushing
    /// FlushVolCalc.cpp:41-48
    fn are_similar(&self, a: &FilamentInfo, b: &FilamentInfo) -> bool {
        // Compare filament type and temperature difference
        // FlushVolCalc.cpp:42-45
        a.filament_type == b.filament_type
            && (a.print_temperature - b.print_temperature).abs() < 20.0
    }

    /// Set base flush volume
    /// FlushVolCalc.cpp:51-56
    pub fn with_base_volume(mut self, volume: f64) -> Self {
        // Update base flush volume field
        // FlushVolCalc.cpp:52
        self.base_flush_volume = volume;
        // Return self for method chaining
        // FlushVolCalc.cpp:53
        self
    }
}

/// Default trait implementation for FlushVolumeCalculator
/// FlushVolCalc.cpp:59-63
impl Default for FlushVolumeCalculator {
    // Create default flush volume calculator
    // FlushVolCalc.cpp:60-62
    fn default() -> Self {
        Self::new()
    }
}

/// Derive traits for FilamentInfo
/// FlushVolCalc.hpp:33-34
#[derive(Clone, Debug)]
/// Information about a filament
/// FlushVolCalc.hpp:35-55
pub struct FilamentInfo {
    /// Filament type (e.g., "PLA", "PETG", "ABS").
    pub filament_type: String,
    /// Brand/color identifier.
    pub color: String,
    /// Print temperature in Celsius.
    pub print_temperature: f64,
    /// Bed temperature in Celsius.
    pub bed_temperature: f64,
}
