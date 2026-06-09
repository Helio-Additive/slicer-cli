//! Compatibility re-export of the `GCodeEditor.{hpp,cpp}` value-types.
//!
//! The canonical 1:1 port lives in [`crate::gcode::g_code_editor`] (snake_case of the C++
//! filename `GCodeEditor`). This module is kept as a thin re-export so existing dependents
//! (`cooling_buffer`, `smoothing`) that import via `crate::gcode::gcode_editor::*` continue to
//! resolve to the single canonical definition without duplication.
//!
//! The previous fake/stub `GCodeEditor`, `EditorLayer`, `EditorGCode` types and the standalone
//! free functions (`reset`, `write_layer_gcode`, `slowdown_to_minimum_feedrate`, ...) were
//! removed: the real (functional) `GCodeEditor` method logic is implemented in
//! `crate::gcode::cooling::GCodeEditorState`. See the BLOCKED note in `g_code_editor.rs`.

pub use crate::gcode::g_code_editor::{
    AdjustableFeatureType, CoolingLine, CoolingLineType, CoolingSlowdownLogicType,
    PerExtruderAdjustments,
};
