//! Custom G-code handling.
//!
//! Mirrors BambuStudio's `CustomGCode` namespace.
//! Handles custom G-code logic for color changes, pauses, tool changes, etc.

use serde::{Deserialize, Serialize};

/// Derive traits for CustomGCodeType enum
/// CustomGCode.hpp:10-12
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
/// Enumeration of custom G-code types
/// CustomGCode.hpp:15-25
pub enum CustomGCodeType {
    ColorChange,
    PausePrint,
    ToolChange,
    Template,
    Custom,
    Unknown,
}

/// Derive traits for CustomGCodeItem struct
/// CustomGCode.hpp:28-30
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
/// Custom G-code item with position and metadata
/// CustomGCode.hpp:32-45
pub struct CustomGCodeItem {
    pub print_z: f64,
    pub type_: CustomGCodeType,
    pub extruder: i32,
    pub color: String,
    pub extra: String,
}

/// Implement Eq trait for CustomGCodeItem
/// CustomGCode.cpp:15-18
impl Eq for CustomGCodeItem {}

/// Implement PartialOrd for sorting by print_z
/// CustomGCode.cpp:20-25
impl PartialOrd for CustomGCodeItem {
    // Compare items by print_z coordinate
    // CustomGCode.cpp:21-23
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.print_z.partial_cmp(&other.print_z)
    }
}

/// Implement Ord for total ordering
/// CustomGCode.cpp:27-32
impl Ord for CustomGCodeItem {
    // Compare items using partial_cmp with fallback
    // CustomGCode.cpp:28-30
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// Derive traits for CustomGCodeMode enum
/// CustomGCode.hpp:48-50
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
/// Custom G-code operation mode
/// CustomGCode.hpp:52-60
pub enum CustomGCodeMode {
    Undef,
    SingleExtruder,
    MultiAsSingle,
    MultiExtruder,
}

/// Derive traits for CustomGCodeInfo struct
/// CustomGCode.hpp:63-65
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
/// Container for custom G-code information
/// CustomGCode.hpp:67-75
pub struct CustomGCodeInfo {
    pub mode: CustomGCodeMode,
    pub gcodes: Vec<CustomGCodeItem>,
}

/// Default trait implementation for CustomGCodeInfo
/// CustomGCode.cpp:35-42
impl Default for CustomGCodeInfo {
    // Create default custom G-code info with undefined mode
    // CustomGCode.cpp:36-40
    fn default() -> Self {
        Self {
            mode: CustomGCodeMode::Undef,
            gcodes: Vec::new(),
        }
    }
}

/// Implementation of CustomGCodeInfo methods
/// CustomGCode.cpp:45-95
impl CustomGCodeInfo {
    // Return pairs of <print_z, extruder ID> sorted by increasing print_z
    // CustomGCode.cpp:48-75
    pub fn custom_tool_changes(&self, num_extruders: usize) -> Vec<(f64, usize)> {
        // Initialize empty vector for tool changes
        // CustomGCode.cpp:49
        let mut changes = Vec::new();
        // Iterate through all custom G-code items
        // CustomGCode.cpp:50-70
        for item in &self.gcodes {
            // Filter for tool change items only
            // CustomGCode.cpp:51-52
            if item.type_ == CustomGCodeType::ToolChange {
                // Determine extruder ID with bounds checking
                // CustomGCode.cpp:53-60
                let extruder_id = {
                    // Check if extruder ID is valid (positive)
                    // CustomGCode.cpp:53
                    if item.extruder > 0 {
                        // Use provided extruder, clamped to valid range
                        // CustomGCode.cpp:54-56
                        (item.extruder as usize).min(num_extruders)
                    } else {
                        // Default to extruder 1 if invalid
                        // CustomGCode.cpp:57-59
                        1 // Default to 1-based index if invalid
                    }
                };
                // Add tool change to result list
                // CustomGCode.cpp:61-62
                changes.push((item.print_z, extruder_id));
            }
        }
        // Sort by print_z coordinate in ascending order
        // CustomGCode.cpp:72-73
        changes.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        // Return sorted tool changes
        // CustomGCode.cpp:74
        changes
    }
}
