//! Compile-time technology flags and feature switches
//!
//! This module provides compile-time constants that control various
//! debugging and feature flags, mirroring BambuStudio's Technologies.hpp.
//! Since this is a pure library rewrite (no GUI), the flags retain the
//! exact same values as the C++ `#define`s.
//!
//! C++ Reference: Technologies.hpp

//=============
// debug techs
//=============
// Technologies.hpp:4-6

/// Shows camera target in the 3D scene
/// Technologies.hpp:8
pub const ENABLE_SHOW_CAMERA_TARGET: bool = false;

/// Log debug messages to console when changing selection
/// Technologies.hpp:10
pub const ENABLE_SELECTION_DEBUG_OUTPUT: bool = false;

/// Renders a small sphere in the center of the bounding box of the current selection when no gizmo is active
/// Technologies.hpp:12
pub const ENABLE_RENDER_SELECTION_CENTER: bool = false;

// Shows an imgui dialog with camera related data
//#define ENABLE_CAMERA_STATISTICS 0// by ctrl +shift +space quick key
// Technologies.hpp:13-14

/// Render the picking pass instead of the main scene (use [T] key to toggle between regular rendering and picking pass only rendering)
/// Technologies.hpp:16
pub const ENABLE_RENDER_PICKING_PASS: bool = false;

/// Enable extracting thumbnails from selected gcode and save them as png files
/// Technologies.hpp:18
pub const ENABLE_THUMBNAIL_GENERATOR_DEBUG: bool = false;

/// Disable synchronization of unselected instances
/// Technologies.hpp:20
pub const DISABLE_INSTANCES_SYNCH: bool = false;

/// Use wxDataViewRender instead of wxDataViewCustomRenderer
/// Technologies.hpp:22
pub const ENABLE_NONCUSTOM_DATA_VIEW_RENDERING: bool = false;

/// Enable G-Code viewer statistics imgui dialog
/// Technologies.hpp:24
pub const ENABLE_GCODE_VIEWER_STATISTICS: bool = false;

/// Enable G-Code viewer comparison between toolpaths height and width detected from gcode and calculated at gcode generation
/// Technologies.hpp:26
pub const ENABLE_GCODE_VIEWER_DATA_CHECKING: bool = false;

/// Enable project dirty state manager debug window
/// Technologies.hpp:28
pub const ENABLE_PROJECT_DIRTY_STATE_DEBUG_WINDOW: bool = false;

/// Enable rendering of objects using environment map
/// Technologies.hpp:32
pub const ENABLE_ENVIRONMENT_MAP: bool = false;

/// Enable smoothing of objects normals
/// Technologies.hpp:34
pub const ENABLE_SMOOTH_NORMALS: bool = false;

/// Enable rendering markers for options in preview as fixed screen size points
/// Technologies.hpp:36
pub const ENABLE_FIXED_SCREEN_SIZE_POINT_MARKERS: bool = true;

/// Enable style editor in develop mode
/// Technologies.hpp:39
pub const ENABLE_IMGUI_STYLE_EDITOR: bool = false;

/// Enable rework of Reload from disk command
/// Technologies.hpp:42
pub const ENABLE_RELOAD_FROM_DISK_REWORK: bool = true;

//====================
// 2.4.0.beta1 techs
//====================
// Technologies.hpp:44-46

/// 2.4.0.beta1 feature set enabled
/// Technologies.hpp:47
pub const ENABLE_2_4_0_BETA1: bool = true;

/// Enable rendering modifiers and similar objects always as transparent
/// Technologies.hpp:50
pub const ENABLE_MODIFIERS_ALWAYS_TRANSPARENT: bool = true && ENABLE_2_4_0_BETA1;

//====================
// 2.4.0.beta2 techs
//====================
// Technologies.hpp:53-55

/// 2.4.0.beta2 feature set enabled
/// Technologies.hpp:56
pub const ENABLE_2_4_0_BETA2: bool = true;

/// Enable modified ImGuiWrapper::slider_float() to create a compound widget where
/// an additional button can be used to set the keyboard focus into the slider
/// to allow the user to type in the desired value
/// Technologies.hpp:61
pub const ENABLE_ENHANCED_IMGUI_SLIDER_FLOAT: bool = true && ENABLE_2_4_0_BETA2;

/// Enable fit print volume command for circular printbeds
/// Technologies.hpp:63
pub const ENABLE_ENHANCED_PRINT_VOLUME_FIT: bool = true && ENABLE_2_4_0_BETA2;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tech_flags() {
        // Debug / GUI-related flags are disabled, matching the C++ defaults.
        assert!(!ENABLE_SHOW_CAMERA_TARGET);
        assert!(!ENABLE_RENDER_SELECTION_CENTER);
        assert!(!ENABLE_ENVIRONMENT_MAP);

        // Enabled flags.
        assert!(ENABLE_FIXED_SCREEN_SIZE_POINT_MARKERS);
        assert!(ENABLE_RELOAD_FROM_DISK_REWORK);

        // Feature-set flags.
        assert!(ENABLE_2_4_0_BETA1);
        assert!(ENABLE_2_4_0_BETA2);
        assert!(ENABLE_MODIFIERS_ALWAYS_TRANSPARENT);
        assert!(ENABLE_ENHANCED_IMGUI_SLIDER_FLOAT);
        assert!(ENABLE_ENHANCED_PRINT_VOLUME_FIT);
    }
}
