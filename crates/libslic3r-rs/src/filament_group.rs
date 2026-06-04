//! Filament group utilities.
//!
//! This module provides filament group management for multi-extruder setups,
//! mirroring BambuStudio's FilamentGroup.cpp and FilamentGroupUtils.cpp.

#[derive(Clone, Debug)]
/// A group of filaments that share similar properties
/// FilamentGroup.hpp:54-61
pub struct FilamentGroup {
    /// Group ID.
    pub id: usize,
    /// Group name.
    pub name: String,
    /// Filament indices in this group.
    pub filaments: Vec<usize>,
    /// Compatible temperature range.
    pub temp_range: (f64, f64),
}

#[derive(Clone, Debug, Default)]
/// Manager for filament groups
/// FilamentGroupUtils.hpp:20-30
pub struct FilamentGroupManager {
    groups: Vec<FilamentGroup>,
}

/// Implementation of FilamentGroupManager methods
/// FilamentGroup.cpp:20-50
impl FilamentGroupManager {
    // Create a new filament group manager
    // FilamentGroup.cpp:20-25
    pub fn new() -> Self {
        // Initialize with default values
        // FilamentGroup.cpp:22-24
        Self::default()
    }

    /// Add a filament group
    /// FilamentGroup.cpp:30-35
    pub fn add_group(&mut self, group: FilamentGroup) {
        // Push group to the groups vector
        // FilamentGroup.cpp:32-33
        self.groups.push(group);
    }

    /// Get the group containing a specific filament
    /// FilamentGroup.cpp:40-50
    pub fn get_group_for_filament(&self, filament_idx: usize) -> Option<&FilamentGroup> {
        // Search through groups to find one containing the filament
        // FilamentGroup.cpp:41-45
        self.groups
            .iter()
            .find(|g| g.filaments.contains(&filament_idx))
    }

    /// Check if two filaments are in the same group
    /// FilamentGroup.cpp:55-65
    pub fn are_compatible(&self, filament_a: usize, filament_b: usize) -> bool {
        // Find groups for both filaments and compare IDs
        // FilamentGroup.cpp:56-60
        self.get_group_for_filament(filament_a)
            .and_then(|ga| {
                self.get_group_for_filament(filament_b)
                    .map(|gb| ga.id == gb.id)
            })
            .unwrap_or(false)
    }
}

/// Calculate flush volume between two filaments
/// FilamentGroupUtils.cpp:100-120
pub fn calculate_flush_volume(_from_filament: usize, _to_filament: usize) -> f64 {
    // TODO: Implement flush volume calculation based on filament properties
    // FilamentGroupUtils.cpp:102-118
    100.0 // Default 100mm³
}
