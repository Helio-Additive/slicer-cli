//! Custom G-code insertion support
//!
//! C++ Reference: CustomGCode.hpp, CustomGCode.cpp
//!
//! This module handles custom G-code insertions at specific Z heights during printing,
//! including color changes, pause points, tool changes, and custom G-code snippets.

use serde::{Deserialize, Serialize};

/// Type of custom G-code insertion
/// CustomGCode.hpp:13-21
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Type {
    // Color change (M600)
    // CustomGCode.hpp:15
    ColorChange,

    // Pause print
    // CustomGCode.hpp:16
    PausePrint,

    // Tool change
    // CustomGCode.hpp:17
    ToolChange,

    // Template G-code
    // CustomGCode.hpp:18
    Template,

    // Custom user G-code
    // CustomGCode.hpp:19
    Custom,

    // Unknown type
    // CustomGCode.hpp:20
    Unknown,
}

/// Single custom G-code item
/// CustomGCode.hpp:23-64
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Item {
    // Z height at which to insert the G-code
    // CustomGCode.hpp:36
    pub print_z: f64,

    // Type of custom G-code
    // CustomGCode.hpp:37
    #[serde(rename = "type")]
    pub gcode_type: Type,

    // Extruder number (1-based)
    // CustomGCode.hpp:38-40
    pub extruder: i32,

    // Color string (hex format) or message for pause
    // CustomGCode.hpp:41-42
    pub color: String,

    // Extra data (custom G-code text or pause message)
    // CustomGCode.hpp:43-45
    #[serde(default)]
    pub extra: String,
}

impl Item {
    // Create a new custom G-code item
    // CustomGCode.hpp:23
    pub fn new(print_z: f64, gcode_type: Type, extruder: i32, color: String) -> Self {
        Self {
            print_z,
            gcode_type,
            extruder,
            color,
            extra: String::new(),
        }
    }

    // Create a new item with extra data
    pub fn new_with_extra(
        print_z: f64,
        gcode_type: Type,
        extruder: i32,
        color: String,
        extra: String,
    ) -> Self {
        Self {
            print_z,
            gcode_type,
            extruder,
            color,
            extra,
        }
    }
}

impl Ord for Item {
    // Sort by Z height
    // CustomGCode.hpp:25
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.print_z
            .partial_cmp(&other.print_z)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

impl PartialOrd for Item {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for Item {}

/// Mode for custom G-code handling
/// CustomGCode.hpp:66-73
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Mode {
    // Undefined mode
    // CustomGCode.hpp:68
    Undef,

    // Single extruder printer
    // CustomGCode.hpp:69
    SingleExtruder,

    // Multiple extruder printer used as single extruder
    // CustomGCode.hpp:70-72
    MultiAsSingle,

    // Multiple extruder printer
    // CustomGCode.hpp:73
    MultiExtruder,
}

/// String constants for mode serialization
/// CustomGCode.hpp:76-78
pub const SINGLE_EXTRUDER_MODE: &str = "SingleExtruder";
pub const MULTI_AS_SINGLE_MODE: &str = "MultiAsSingle";
pub const MULTI_EXTRUDER_MODE: &str = "MultiExtruder";

/// Information about custom G-code for a print
/// CustomGCode.hpp:80-102
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Info {
    // Mode for custom G-code handling
    // CustomGCode.hpp:82
    #[serde(default)]
    pub mode: Mode,

    // List of custom G-code items
    // CustomGCode.hpp:83
    #[serde(default)]
    pub gcodes: Vec<Item>,
}

impl Info {
    // Create new empty custom G-code info
    // CustomGCode.hpp:80
    pub fn new() -> Self {
        Self {
            mode: Mode::Undef,
            gcodes: Vec::new(),
        }
    }

    // Create with specified mode
    pub fn new_with_mode(mode: Mode) -> Self {
        Self {
            mode,
            gcodes: Vec::new(),
        }
    }

    // Sort items by Z height
    pub fn sort(&mut self) {
        self.gcodes.sort();
    }
}

impl Default for Info {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for Mode {
    fn default() -> Self {
        Mode::Undef
    }
}

/// Check and update mode for custom G-code based on items
///
/// If mode is undefined, determines the appropriate mode by examining
/// the custom G-code items.
///
/// CustomGCode.cpp:40-56
pub fn check_mode_for_custom_gcode_per_print_z(info: &mut Info) {
    // Already has a defined mode
    // CustomGCode.cpp:42-43
    if info.mode != Mode::Undef {
        return;
    }

    // Check items to determine mode
    // CustomGCode.cpp:45-55
    let mut is_single_extruder = true;

    for item in &info.gcodes {
        // ToolChange means MultiAsSingle mode
        // CustomGCode.cpp:48-51
        if item.gcode_type == Type::ToolChange {
            info.mode = Mode::MultiAsSingle;
            return;
        }

        // ColorChange with extruder > 1 means not single extruder
        // CustomGCode.cpp:52-53
        if item.gcode_type == Type::ColorChange && item.extruder > 1 {
            is_single_extruder = false;
        }
    }

    // Set mode based on extruder usage
    // CustomGCode.cpp:55
    info.mode = if is_single_extruder {
        Mode::SingleExtruder
    } else {
        Mode::MultiExtruder
    };
}

/// Extract tool change events from custom G-code
///
/// Returns pairs of (print_z, extruder_id) sorted by Z height.
/// The extruder ID is 1-based.
///
/// # Arguments
/// * `custom_gcode_per_print_z` - Custom G-code information
/// * `num_extruders` - Number of extruders in printer
///
/// CustomGCode.cpp:58-69
pub fn custom_tool_changes(
    custom_gcode_per_print_z: &Info,
    num_extruders: usize,
) -> Vec<(f64, u32)> {
    let mut tool_changes = Vec::new();

    // Extract ToolChange items
    // CustomGCode.cpp:61-67
    for custom_gcode in &custom_gcode_per_print_z.gcodes {
        if custom_gcode.gcode_type == Type::ToolChange {
            // Clamp extruder ID to valid range
            // CustomGCode.cpp:63-65
            debug_assert!(custom_gcode.extruder >= 0);
            let extruder_id = if custom_gcode.extruder as usize > num_extruders {
                1
            } else {
                custom_gcode.extruder as u32
            };

            tool_changes.push((custom_gcode.print_z, extruder_id));
        }
    }

    tool_changes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_item_ordering() {
        let item1 = Item::new(1.0, Type::ColorChange, 1, "#FF0000".to_string());
        let item2 = Item::new(2.0, Type::ColorChange, 1, "#00FF00".to_string());

        assert!(item1 < item2);
        assert_eq!(item1.cmp(&item1), std::cmp::Ordering::Equal);
    }

    #[test]
    fn test_item_equality() {
        let item1 = Item::new(1.0, Type::ColorChange, 1, "#FF0000".to_string());
        let item2 = Item::new(1.0, Type::ColorChange, 1, "#FF0000".to_string());
        let item3 = Item::new(2.0, Type::ColorChange, 1, "#FF0000".to_string());

        assert_eq!(item1, item2);
        assert_ne!(item1, item3);
    }

    #[test]
    fn test_info_creation() {
        let info = Info::new();
        assert_eq!(info.mode, Mode::Undef);
        assert!(info.gcodes.is_empty());

        let info2 = Info::new_with_mode(Mode::SingleExtruder);
        assert_eq!(info2.mode, Mode::SingleExtruder);
    }

    #[test]
    fn test_check_mode_single_extruder() {
        let mut info = Info::new();
        info.gcodes
            .push(Item::new(1.0, Type::ColorChange, 1, "#FF0000".to_string()));
        info.gcodes
            .push(Item::new(2.0, Type::PausePrint, 1, "Check".to_string()));

        check_mode_for_custom_gcode_per_print_z(&mut info);
        assert_eq!(info.mode, Mode::SingleExtruder);
    }

    #[test]
    fn test_check_mode_multi_extruder() {
        let mut info = Info::new();
        info.gcodes
            .push(Item::new(1.0, Type::ColorChange, 1, "#FF0000".to_string()));
        info.gcodes
            .push(Item::new(2.0, Type::ColorChange, 2, "#00FF00".to_string()));

        check_mode_for_custom_gcode_per_print_z(&mut info);
        assert_eq!(info.mode, Mode::MultiExtruder);
    }

    #[test]
    fn test_check_mode_multi_as_single() {
        let mut info = Info::new();
        info.gcodes
            .push(Item::new(1.0, Type::ToolChange, 2, "".to_string()));

        check_mode_for_custom_gcode_per_print_z(&mut info);
        assert_eq!(info.mode, Mode::MultiAsSingle);
    }

    #[test]
    fn test_check_mode_already_defined() {
        let mut info = Info::new_with_mode(Mode::SingleExtruder);
        info.gcodes
            .push(Item::new(1.0, Type::ToolChange, 2, "".to_string()));

        check_mode_for_custom_gcode_per_print_z(&mut info);
        // Should NOT change from SingleExtruder
        assert_eq!(info.mode, Mode::SingleExtruder);
    }

    #[test]
    fn test_custom_tool_changes() {
        let mut info = Info::new();
        info.gcodes
            .push(Item::new(1.0, Type::ToolChange, 2, "".to_string()));
        info.gcodes
            .push(Item::new(2.0, Type::ColorChange, 1, "#FF0000".to_string()));
        info.gcodes
            .push(Item::new(3.0, Type::ToolChange, 3, "".to_string()));

        let changes = custom_tool_changes(&info, 4);

        assert_eq!(changes.len(), 2);
        assert_eq!(changes[0], (1.0, 2));
        assert_eq!(changes[1], (3.0, 3));
    }

    #[test]
    fn test_custom_tool_changes_clamp() {
        let mut info = Info::new();
        // Extruder 5 is out of range (only 2 extruders)
        info.gcodes
            .push(Item::new(1.0, Type::ToolChange, 5, "".to_string()));

        let changes = custom_tool_changes(&info, 2);

        assert_eq!(changes.len(), 1);
        assert_eq!(changes[0], (1.0, 1)); // Clamped to extruder 1
    }

    #[test]
    fn test_info_sorting() {
        let mut info = Info::new();
        info.gcodes
            .push(Item::new(3.0, Type::ColorChange, 1, "#FF0000".to_string()));
        info.gcodes
            .push(Item::new(1.0, Type::ColorChange, 1, "#00FF00".to_string()));
        info.gcodes
            .push(Item::new(2.0, Type::ColorChange, 1, "#0000FF".to_string()));

        info.sort();

        assert_eq!(info.gcodes[0].print_z, 1.0);
        assert_eq!(info.gcodes[1].print_z, 2.0);
        assert_eq!(info.gcodes[2].print_z, 3.0);
    }

    #[test]
    fn test_type_equality() {
        assert_eq!(Type::ColorChange, Type::ColorChange);
        assert_ne!(Type::ColorChange, Type::ToolChange);
    }

    #[test]
    fn test_mode_constants() {
        assert_eq!(SINGLE_EXTRUDER_MODE, "SingleExtruder");
        assert_eq!(MULTI_AS_SINGLE_MODE, "MultiAsSingle");
        assert_eq!(MULTI_EXTRUDER_MODE, "MultiExtruder");
    }
}
