//! Compile-time technology flags and feature switches
//!
//! This module provides compile-time constants that control various
//! debugging and feature flags, mirroring BambuStudio's Technologies.hpp.
//! Since this is a pure library rewrite (no GUI), most flags are disabled.
//!
//! C++ Reference: Technologies.hpp

/// Shows camera target in the 3D scene (GUI only)
/// Technologies.hpp:8
pub const ENABLE_SHOW_CAMERA_TARGET: bool = false;

/// Log debug messages to console when changing selection (GUI only)
/// Technologies.hpp:10
pub const ENABLE_SELECTION_DEBUG_OUTPUT: bool = false;

/// Renders a small sphere in the center of the bounding box (GUI only)
/// Technologies.hpp:12
pub const ENABLE_RENDER_SELECTION_CENTER: bool = false;

/// Render the picking pass instead of the main scene (GUI only)
/// Technologies.hpp:16
pub const ENABLE_RENDER_PICKING_PASS: bool = false;

/// Enable extracting thumbnails from selected gcode
/// Technologies.hpp:18
pub const ENABLE_THUMBNAIL_GENERATOR_DEBUG: bool = false;

/// Disable synchronization of unselected instances
/// Technologies.hpp:20
pub const DISABLE_INSTANCES_SYNCH: bool = false;

/// Use wxDataViewRender instead of wxDataViewCustomRenderer (GUI only)
/// Technologies.hpp:22
pub const ENABLE_NONCUSTOM_DATA_VIEW_RENDERING: bool = false;

/// Enable G-Code viewer statistics imgui dialog (GUI only)
/// Technologies.hpp:24
pub const ENABLE_GCODE_VIEWER_STATISTICS: bool = false;

/// Enable G-Code viewer comparison between toolpaths
/// Technologies.hpp:26
pub const ENABLE_GCODE_VIEWER_DATA_CHECKING: bool = false;

/// Enable project dirty state manager debug window (GUI only)
/// Technologies.hpp:28
pub const ENABLE_PROJECT_DIRTY_STATE_DEBUG_WINDOW: bool = false;

/// Enable rendering of objects using environment map (GUI only)
/// Technologies.hpp:32
pub const ENABLE_ENVIRONMENT_MAP: bool = false;

/// Enable smoothing of objects normals
/// Technologies.hpp:34
pub const ENABLE_SMOOTH_NORMALS: bool = false;

/// Enable rendering markers for options in preview (GUI only)
/// Technologies.hpp:36
pub const ENABLE_FIXED_SCREEN_SIZE_POINT_MARKERS: bool = true;

/// Enable style editor in develop mode (GUI only)
/// Technologies.hpp:39
pub const ENABLE_IMGUI_STYLE_EDITOR: bool = false;

/// Enable rework of Reload from disk command
/// Technologies.hpp:42
pub const ENABLE_RELOAD_FROM_DISK_REWORK: bool = true;

/// 2.4.0.beta1 feature set enabled
/// Technologies.hpp:47
pub const ENABLE_2_4_0_BETA1: bool = true;

/// Enable rendering modifiers and similar objects always as transparent (GUI only)
/// Technologies.hpp:50
pub const ENABLE_MODIFIERS_ALWAYS_TRANSPARENT: bool = ENABLE_2_4_0_BETA1;

/// Check if a specific technology flag is enabled (runtime query)
/// Technologies.hpp (utility function)
#[inline]
pub fn is_tech_enabled(tech_name: &str) -> bool {
    match tech_name {
        "SHOW_CAMERA_TARGET" => ENABLE_SHOW_CAMERA_TARGET,
        "SELECTION_DEBUG_OUTPUT" => ENABLE_SELECTION_DEBUG_OUTPUT,
        "RENDER_SELECTION_CENTER" => ENABLE_RENDER_SELECTION_CENTER,
        "RENDER_PICKING_PASS" => ENABLE_RENDER_PICKING_PASS,
        "THUMBNAIL_GENERATOR_DEBUG" => ENABLE_THUMBNAIL_GENERATOR_DEBUG,
        "GCODE_VIEWER_STATISTICS" => ENABLE_GCODE_VIEWER_STATISTICS,
        "GCODE_VIEWER_DATA_CHECKING" => ENABLE_GCODE_VIEWER_DATA_CHECKING,
        "PROJECT_DIRTY_STATE_DEBUG_WINDOW" => ENABLE_PROJECT_DIRTY_STATE_DEBUG_WINDOW,
        "ENVIRONMENT_MAP" => ENABLE_ENVIRONMENT_MAP,
        "SMOOTH_NORMALS" => ENABLE_SMOOTH_NORMALS,
        "FIXED_SCREEN_SIZE_POINT_MARKERS" => ENABLE_FIXED_SCREEN_SIZE_POINT_MARKERS,
        "IMGUI_STYLE_EDITOR" => ENABLE_IMGUI_STYLE_EDITOR,
        "RELOAD_FROM_DISK_REWORK" => ENABLE_RELOAD_FROM_DISK_REWORK,
        "2_4_0_BETA1" => ENABLE_2_4_0_BETA1,
        "MODIFIERS_ALWAYS_TRANSPARENT" => ENABLE_MODIFIERS_ALWAYS_TRANSPARENT,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tech_flags() {
        // Most GUI-related flags should be disabled in library-only build
        assert!(!ENABLE_SHOW_CAMERA_TARGET);
        assert!(!ENABLE_RENDER_SELECTION_CENTER);
        assert!(!ENABLE_ENVIRONMENT_MAP);

        // Feature flags should be enabled
        assert!(ENABLE_2_4_0_BETA1);
        assert!(ENABLE_RELOAD_FROM_DISK_REWORK);
    }

    #[test]
    fn test_tech_query() {
        assert!(is_tech_enabled("2_4_0_BETA1"));
        assert!(!is_tech_enabled("SHOW_CAMERA_TARGET"));
        assert!(!is_tech_enabled("UNKNOWN_TECH"));
    }
}
