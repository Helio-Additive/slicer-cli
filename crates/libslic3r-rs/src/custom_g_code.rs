//! Custom G-code insertion support
//!
//! C++ Reference: CustomGCode.hpp, CustomGCode.cpp
//!
//! 1:1 line-by-line port of `src/libslic3r/CustomGCode.{hpp,cpp}` from BambuStudio.
//! This module handles custom G-code insertions at specific Z heights during printing,
//! including color changes, pause points, tool changes, and custom G-code snippets.

use serde::{Deserialize, Serialize};

// CustomGCode.hpp:14-22
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Type {
    // CustomGCode.hpp:16
    ColorChange,
    // CustomGCode.hpp:17
    PausePrint,
    // CustomGCode.hpp:18
    ToolChange,
    // CustomGCode.hpp:19
    Template,
    // CustomGCode.hpp:20
    Custom,
    // CustomGCode.hpp:21
    Unknown,
}

// CustomGCode.hpp:24-65
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Item {
    // CustomGCode.hpp:37
    pub print_z: f64,
    // CustomGCode.hpp:38
    #[serde(rename = "type")]
    pub gcode_type: Type,
    // CustomGCode.hpp:39-41
    // Informative value for ColorChangeCode and ToolChangeCode
    // "gcode" == ColorChangeCode   => M600 will be applied for "extruder" extruder
    // "gcode" == ToolChangeCode    => for whole print tool will be switched to "extruder" extruder
    pub extruder: i32,
    // CustomGCode.hpp:42-43
    // if gcode is equal to PausePrintCode,
    // this field is used for save a short message shown on Printer display
    pub color: String,
    // CustomGCode.hpp:44-46
    // this field is used for the extra data like :
    // - G-code text for the Type::Custom
    // - message text for the Type::PausePrint
    #[serde(default)]
    pub extra: String,
}

impl Item {
    // Convenience constructor mirroring C++ aggregate initialization
    // `Item{ print_z, type, extruder, color }` (CustomGCode.cpp:28, in #if 0 block)
    // CustomGCode.hpp:24
    pub fn new(print_z: f64, gcode_type: Type, extruder: i32, color: String) -> Self {
        Self {
            print_z,
            gcode_type,
            extruder,
            color,
            extra: String::new(),
        }
    }

    // Convenience constructor with the `extra` field populated.
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

    // CustomGCode.hpp:47-64
    pub fn from_json(j: &serde_json::Value) -> Self {
        let mut item = Item {
            print_z: 0.0,
            gcode_type: Type::Unknown,
            extruder: 0,
            color: String::new(),
            extra: String::new(),
        };
        // CustomGCode.hpp:48-49
        let type_str: String = j["type"].as_str().unwrap().to_string();
        // CustomGCode.hpp:50-55
        let str2type: std::collections::HashMap<&str, Type> = [
            ("ColorChange", Type::ColorChange),
            ("PausePrint", Type::PausePrint),
            ("ToolChange", Type::ToolChange),
            ("Template", Type::Template),
            ("Custom", Type::Custom),
            ("Unknown", Type::Unknown),
        ]
        .into_iter()
        .collect();
        // CustomGCode.hpp:56
        item.gcode_type = Type::Unknown;
        // CustomGCode.hpp:57-58
        if let Some(&t) = str2type.get(type_str.as_str()) {
            item.gcode_type = t;
        }
        // CustomGCode.hpp:59
        item.print_z = j["print_z"].as_f64().unwrap();
        // CustomGCode.hpp:60
        item.color = j["color"].as_str().unwrap().to_string();
        // CustomGCode.hpp:61
        item.extruder = j["extruder"].as_i64().unwrap() as i32;
        // CustomGCode.hpp:62-63
        if j.get("extra").is_some() {
            item.extra = j["extra"].as_str().unwrap().to_string();
        }
        item
    }
}

// CustomGCode.hpp:26 (operator<)
impl Ord for Item {
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

// CustomGCode.hpp:27-35 (operator==, operator!=)
impl PartialEq for Item {
    fn eq(&self, rhs: &Self) -> bool {
        (rhs.print_z == self.print_z)
            && (rhs.gcode_type == self.gcode_type)
            && (rhs.extruder == self.extruder)
            && (rhs.color == self.color)
            && (rhs.extra == self.extra)
    }
}

impl Eq for Item {}

// CustomGCode.hpp:67-75
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Mode {
    // CustomGCode.hpp:69
    Undef,
    // CustomGCode.hpp:70 Single extruder printer preset is selected
    SingleExtruder,
    // CustomGCode.hpp:71-73 Multiple extruder printer preset is selected, but
    // this mode works just for Single extruder print
    // (The same extruder is assigned to all ModelObjects and ModelVolumes).
    MultiAsSingle,
    // CustomGCode.hpp:74 Multiple extruder printer preset is selected
    MultiExtruder,
}

// string anlogue of custom_code_per_height mode
// CustomGCode.hpp:78
pub const SINGLE_EXTRUDER_MODE: &str = "SingleExtruder";
// CustomGCode.hpp:79
pub const MULTI_AS_SINGLE_MODE: &str = "MultiAsSingle";
// CustomGCode.hpp:80
pub const MULTI_EXTRUDER_MODE: &str = "MultiExtruder";

// CustomGCode.hpp:82-111
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Info {
    // CustomGCode.hpp:84
    #[serde(default)]
    pub mode: Mode,
    // CustomGCode.hpp:85
    #[serde(default)]
    pub gcodes: Vec<Item>,
}

impl Info {
    // Convenience constructor matching the default member initializers
    // (CustomGCode.hpp:84 `Mode mode = Undef;`).
    pub fn new() -> Self {
        Self {
            mode: Mode::Undef,
            gcodes: Vec::new(),
        }
    }

    pub fn new_with_mode(mode: Mode) -> Self {
        Self {
            mode,
            gcodes: Vec::new(),
        }
    }

    pub fn sort(&mut self) {
        self.gcodes.sort();
    }

    // CustomGCode.hpp:94-110
    pub fn from_json(j: &serde_json::Value) -> Self {
        let mut info = Info {
            mode: Mode::Undef,
            gcodes: Vec::new(),
        };
        // CustomGCode.hpp:95-97
        let mut mode_str = String::new();
        if j.get("mode").is_some() {
            mode_str = j["mode"].as_str().unwrap().to_string();
        }
        // CustomGCode.hpp:98-101
        if mode_str == "SingleExtruder" {
            info.mode = Mode::SingleExtruder;
        } else if mode_str == "MultiAsSingle" {
            info.mode = Mode::MultiAsSingle;
        } else if mode_str == "MultiExtruder" {
            info.mode = Mode::MultiExtruder;
        } else {
            info.mode = Mode::Undef;
        }

        // CustomGCode.hpp:103
        let j_gcodes = &j["gcodes"];
        // CustomGCode.hpp:104 (gcodes.reserve(j_gcodes.size())) — Vec growth handles this.
        info.gcodes.reserve(j_gcodes.as_array().map_or(0, |a| a.len()));
        // CustomGCode.hpp:105-109
        for jj in j_gcodes.as_array().unwrap() {
            let item = Item::from_json(jj);
            info.gcodes.push(item);
        }
        info
    }
}

// CustomGCode.hpp:87-91 (operator==, operator!=)
impl PartialEq for Info {
    fn eq(&self, rhs: &Self) -> bool {
        (rhs.mode == self.mode) && (rhs.gcodes == self.gcodes)
    }
}

impl Eq for Info {}

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

//BBS: useless config and function
// update_custom_gcode_per_print_z_from_config is wrapped in `#if 0` in C++
// (CustomGCode.cpp:11-37) and the header declaration is commented out
// (CustomGCode.hpp:116-117), so it is intentionally not ported.

// If information for custom Gcode per print Z was imported from older Slicer, mode will be undefined.
// So, we should set CustomGCode::Info.mode should be updated considering code values from items.
// CustomGCode.cpp:41-58
pub fn check_mode_for_custom_gcode_per_print_z(info: &mut Info) {
    // CustomGCode.cpp:43-44
    if info.mode != Mode::Undef {
        return;
    }

    // CustomGCode.cpp:46
    let mut is_single_extruder = true;
    // CustomGCode.cpp:47-55
    for item in &info.gcodes {
        // CustomGCode.cpp:49-52
        if item.gcode_type == Type::ToolChange {
            info.mode = Mode::MultiAsSingle;
            return;
        }
        // CustomGCode.cpp:53-54
        if item.gcode_type == Type::ColorChange && item.extruder > 1 {
            is_single_extruder = false;
        }
    }

    // CustomGCode.cpp:57
    info.mode = if is_single_extruder {
        Mode::SingleExtruder
    } else {
        Mode::MultiExtruder
    };
}

// Return pairs of <print_z, 1-based extruder ID> sorted by increasing print_z from custom_gcode_per_print_z.
// print_z corresponds to the first layer printed with the new extruder.
// CustomGCode.cpp:62-72
pub fn custom_tool_changes(
    custom_gcode_per_print_z: &Info,
    num_extruders: usize,
) -> Vec<(f64, u32)> {
    // CustomGCode.cpp:64
    let mut custom_tool_changes: Vec<(f64, u32)> = Vec::new();
    // CustomGCode.cpp:65-70
    for custom_gcode in &custom_gcode_per_print_z.gcodes {
        if custom_gcode.gcode_type == Type::ToolChange {
            // If extruder count in PrinterSettings was changed, use default (0) extruder for extruders, more than num_extruders
            // CustomGCode.cpp:68
            debug_assert!(custom_gcode.extruder >= 0);
            // CustomGCode.cpp:69
            custom_tool_changes.push((
                custom_gcode.print_z,
                if (custom_gcode.extruder as usize) > num_extruders {
                    1
                } else {
                    custom_gcode.extruder as u32
                },
            ));
        }
    }
    // CustomGCode.cpp:71
    custom_tool_changes
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

    #[test]
    fn test_item_from_json() {
        let j = serde_json::json!({
            "type": "ColorChange",
            "print_z": 1.5,
            "color": "#FF0000",
            "extruder": 2
        });
        let item = Item::from_json(&j);
        assert_eq!(item.gcode_type, Type::ColorChange);
        assert_eq!(item.print_z, 1.5);
        assert_eq!(item.color, "#FF0000");
        assert_eq!(item.extruder, 2);
        assert_eq!(item.extra, "");
    }

    #[test]
    fn test_item_from_json_unknown_type() {
        let j = serde_json::json!({
            "type": "Bogus",
            "print_z": 0.2,
            "color": "",
            "extruder": 0,
            "extra": "hello"
        });
        let item = Item::from_json(&j);
        // Unrecognized type falls back to Unknown (CustomGCode.hpp:56-58)
        assert_eq!(item.gcode_type, Type::Unknown);
        assert_eq!(item.extra, "hello");
    }

    #[test]
    fn test_info_from_json() {
        let j = serde_json::json!({
            "mode": "MultiExtruder",
            "gcodes": [
                {"type": "ToolChange", "print_z": 1.0, "color": "", "extruder": 2},
                {"type": "ColorChange", "print_z": 2.0, "color": "#00FF00", "extruder": 1}
            ]
        });
        let info = Info::from_json(&j);
        assert_eq!(info.mode, Mode::MultiExtruder);
        assert_eq!(info.gcodes.len(), 2);
        assert_eq!(info.gcodes[0].gcode_type, Type::ToolChange);
        assert_eq!(info.gcodes[1].gcode_type, Type::ColorChange);
    }
}
