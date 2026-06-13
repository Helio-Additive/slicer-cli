//! G-code generation module.
//!
//! This module provides types and functions for generating G-code from
//! sliced layers, mirroring BambuStudio's GCode and GCodeWriter classes.

pub mod arc_fitting;
pub mod avoid_crossing_perimeters;
pub mod compare;
pub mod conflict_checker;
pub mod cooling;
pub mod cooling_buffer;
pub mod curve_analyzer;
pub mod custom_gcode;
pub mod exporter;
pub mod extruder;
pub mod g_code_editor;
pub mod g_code_processor;
pub mod gcode_processor;
mod generator;
pub mod ironing;
pub mod multi_material;
mod path;
pub mod placeholder_parser;
pub mod post_processor;
pub mod pressure_equalizer;
pub mod print_extents;
pub mod reader;
pub mod retract_crossing;
pub mod retract_when_crossing_perimeters;
pub mod seam_placer;
pub mod smoothing;
pub mod spiral_vase;
pub mod thumbnail_data;
pub mod timelapse_pos_picker;
pub mod tool_order_utils;
pub mod tool_ordering;
pub mod validation;
pub mod wipe_tower;
mod writer;

pub use arc_fitting::{
    fit_arcs, fit_arcs_to_points, ArcDirection, ArcFitter, ArcFittingConfig, ArcFittingStats,
    FittedArc, PathSegment,
};
pub use avoid_crossing_perimeters::{
    init_boundary, init_boundary_with_merge_points, AvoidCrossingPerimeters, Boundary,
    ConvertBBoxToPolyline,
};
pub use compare::{
    compare_exact_lines, compare_gcode, compare_gcode_files, ComparisonConfig, ComparisonResult,
    ExactComparisonResult, ExtrusionMode, ExtrusionTracker, GCodeComparator, GCodeMove,
    LayerComparison, LayerInfo, MoveComparison, ParsedGCode,
};
pub use cooling::{
    estimate_layer_time, CoolingBuffer, CoolingConfig, CoolingMove, CoolingResult,
    GCodeEditorState, PerExtruderAdjustments,
};
pub use exporter::{
    change_layer, emit_layer_object_labels, emit_layer_postamble, emit_layer_preamble,
    emit_object_end_label, emit_object_start_label, extrude_collection, extrude_entity,
    extrude_infill, extrude_loop, extrude_multi_path, extrude_path, extrude_path_with_arc_fitting,
    extrude_perimeters, extrude_support, retract, set_extruder, travel_to, unretract, wipe,
    GCodeLayerState,
};
pub use generator::{process_gcode_template, GCode, GCodeHeader, GCodeStats};
pub use ironing::{
    generate_ironing, should_iron_layer, IroningConfig, IroningGenerator, IroningPath,
    IroningResult, IroningType,
};
pub use multi_material::{
    MultiMaterialConfig, MultiMaterialCoordinator, MultiMaterialLayer, MultiMaterialPlan,
    ToolChange, WipeTowerBounds,
};
pub use path::{
    generate_paths, generate_solid_paths, ExtrusionPath, ExtrusionRole, LayerKind, LayerPaths,
    PathConfig, PathGenerator, SeamPosition,
};
pub use placeholder_parser::{PlaceholderParser, PrintContext};
pub use pressure_equalizer::{PressureEqualizer, PressureEqualizerConfig, PressureEqualizerStats};
pub use retract_crossing::{RetractCrossingConfig, RetractDecision, RetractWhenCrossingPerimeters};
pub use seam_placer::{
    create_seam_placer, find_best_seam_index, place_seam, EnforcedBlockedSeamPoint, LayerOutline,
    LayerSeams, Perimeter, PerimeterOutline, Point3f, SeamCandidate, SeamPlacer, SeamPlacerConfig,
    SeamPlacerStats, SeamPositionMode,
};
pub use spiral_vase::{spiral_vase_helpers, SpiralPoint, SpiralVase};
pub use tool_ordering::{
    calculate_flush_volume, find_optimal_ordering_exhaustive, generate_all_orderings,
    optimize_extruder_sequence, CustomGCodeItem, CustomGCodeType, ExtrusionRoleType,
    FilamentChangeMode, FilamentChangeStats, FilamentMapMode, FlushMatrix, LayerTools,
    ToolOrdering, ToolOrderingConfig, WipingExtrusions,
};
pub use validation::{
    validate_gcode_files, validate_gcode_files_with_config, FeatureStats, FeatureType,
    IssueCategory, IssueSeverity, LayerValidation, ReportFormat, ScoreBreakdown, ValidationConfig,
    ValidationIssue, ValidationReport, ValidationSummary,
};
pub use wipe_tower::{
    align_ceil, align_floor, align_round, is_valid_gcode, BedShape, BlockDepthInfo, BoxCoordinates,
    Extrusion, FilamentParameters, GCodeFlavor as WipeTowerGCodeFlavor, LimitFlow,
    NozzleChangeResult, ToolChangeInfo, ToolChangeResult, Vec2f, WipeShape, WipeTower,
    WipeTowerBlock, WipeTowerConfig, WipeTowerLayerInfo, WipeTowerWriter,
};
pub use writer::{format_gcode_value, GCodeWriter};

/// G-code command types.
#[derive(Clone, Debug, PartialEq)]
pub enum GCodeCommand {
    /// G0 - Rapid move (travel)
    RapidMove {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        f: Option<f64>,
    },
    /// G1 - Linear move (extrusion)
    LinearMove {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        e: Option<f64>,
        f: Option<f64>,
    },
    /// G2 - Clockwise arc
    ArcCW {
        x: f64,
        y: f64,
        i: f64,
        j: f64,
        e: Option<f64>,
        f: Option<f64>,
    },
    /// G3 - Counter-clockwise arc
    ArcCCW {
        x: f64,
        y: f64,
        i: f64,
        j: f64,
        e: Option<f64>,
        f: Option<f64>,
    },
    /// G3 - Helical counter-clockwise arc (travel with Z movement).
    /// Used for spiral lift during retraction. Traces a circle in XY
    /// while simultaneously changing Z height.
    ///
    /// BambuStudio reference: GCodeWriter.cpp `_spiral_travel_to_z()` line 661
    /// Example: `G3 Z0.6 I-0.86 J-0.861 P1 F60000`
    HelicalArcCCW {
        /// Target Z height (absolute)
        z: f64,
        /// X offset to arc center from current position
        i: f64,
        /// Y offset to arc center from current position
        j: f64,
        /// Number of full revolutions (typically 1)
        p: u32,
        /// Feedrate (mm/min)
        f: Option<f64>,
    },
    /// G17 - Select XY plane for arc interpolation.
    /// Required before helical arcs to ensure the firmware interprets
    /// I/J as XY-plane offsets.
    SelectXYPlane,
    /// G28 - Home
    Home { x: bool, y: bool, z: bool },
    /// G90 - Absolute positioning
    AbsolutePositioning,
    /// G91 - Relative positioning
    RelativePositioning,
    /// G92 - Set position
    SetPosition {
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        e: Option<f64>,
    },
    /// M82 - Absolute extrusion
    AbsoluteExtrusion,
    /// M83 - Relative extrusion
    RelativeExtrusion,
    /// M104 - Set extruder temperature (no wait)
    SetExtruderTemp { s: u32 },
    /// M109 - Set extruder temperature and wait
    SetExtruderTempWait { s: u32 },
    /// M140 - Set bed temperature (no wait)
    SetBedTemp { s: u32 },
    /// M190 - Set bed temperature and wait
    SetBedTempWait { s: u32 },
    /// M106 - Set fan speed
    SetFanSpeed { s: u32 },
    /// M107 - Fan off
    FanOff,
    /// Comment
    Comment(String),
    /// Raw G-code line
    Raw(String),
}

impl GCodeCommand {
    // Convert the command to a G-code string.
    pub fn to_gcode(&self) -> String {
        match self {
            GCodeCommand::RapidMove { x, y, z, f } => {
                let mut cmd = String::from("G0");
                if let Some(v) = x {
                    cmd.push_str(&format!(" X{}", writer::format_gcode_value(*v, 3)));
                }
                if let Some(v) = y {
                    cmd.push_str(&format!(" Y{}", writer::format_gcode_value(*v, 3)));
                }
                if let Some(v) = z {
                    cmd.push_str(&format!(" Z{}", writer::format_gcode_value(*v, 3)));
                }
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::LinearMove { x, y, z, e, f } => {
                let mut cmd = String::from("G1");
                if let Some(v) = x {
                    cmd.push_str(&format!(" X{}", writer::format_gcode_value(*v, 3)));
                }
                if let Some(v) = y {
                    cmd.push_str(&format!(" Y{}", writer::format_gcode_value(*v, 3)));
                }
                if let Some(v) = z {
                    cmd.push_str(&format!(" Z{}", writer::format_gcode_value(*v, 3)));
                }
                if let Some(v) = e {
                    cmd.push_str(&format!(" E{}", writer::format_gcode_value(*v, 5)));
                }
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::ArcCW { x, y, i, j, e, f } => {
                let mut cmd = format!("G2 X{:.3} Y{:.3} I{:.3} J{:.3}", x, y, i, j);
                if let Some(v) = e {
                    cmd.push_str(&format!(" E{:.5}", v));
                }
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::ArcCCW { x, y, i, j, e, f } => {
                let mut cmd = format!("G3 X{:.3} Y{:.3} I{:.3} J{:.3}", x, y, i, j);
                if let Some(v) = e {
                    cmd.push_str(&format!(" E{:.5}", v));
                }
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::HelicalArcCCW { z, i, j, p, f } => {
                let mut cmd = format!(
                    "G3 Z{} I{} J{} P{}",
                    writer::format_gcode_value(*z, 3),
                    writer::format_gcode_value(*i, 3),
                    writer::format_gcode_value(*j, 3),
                    p
                );
                if let Some(v) = f {
                    cmd.push_str(&format!(" F{:.0}", v));
                }
                cmd
            }
            GCodeCommand::SelectXYPlane => "G17".to_string(),
            GCodeCommand::Home { x, y, z } => {
                let mut cmd = String::from("G28");
                if *x {
                    cmd.push_str(" X");
                }
                if *y {
                    cmd.push_str(" Y");
                }
                if *z {
                    cmd.push_str(" Z");
                }
                cmd
            }
            GCodeCommand::AbsolutePositioning => "G90".to_string(),
            GCodeCommand::RelativePositioning => "G91".to_string(),
            GCodeCommand::SetPosition { x, y, z, e } => {
                let mut cmd = String::from("G92");
                if let Some(v) = x {
                    cmd.push_str(&format!(" X{:.3}", v));
                }
                if let Some(v) = y {
                    cmd.push_str(&format!(" Y{:.3}", v));
                }
                if let Some(v) = z {
                    cmd.push_str(&format!(" Z{:.3}", v));
                }
                if let Some(v) = e {
                    cmd.push_str(&format!(" E{:.5}", v));
                }
                cmd
            }
            GCodeCommand::AbsoluteExtrusion => "M82".to_string(),
            GCodeCommand::RelativeExtrusion => "M83".to_string(),
            GCodeCommand::SetExtruderTemp { s } => format!("M104 S{}", s),
            GCodeCommand::SetExtruderTempWait { s } => format!("M109 S{}", s),
            GCodeCommand::SetBedTemp { s } => format!("M140 S{}", s),
            GCodeCommand::SetBedTempWait { s } => format!("M190 S{}", s),
            GCodeCommand::SetFanSpeed { s } => format!("M106 S{}", s),
            GCodeCommand::FanOff => "M107".to_string(),
            GCodeCommand::Comment(text) => format!("; {}", text),
            GCodeCommand::Raw(line) => line.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rapid_move() {
        let cmd = GCodeCommand::RapidMove {
            x: Some(10.0),
            y: Some(20.0),
            z: None,
            f: Some(3000.0),
        };
        assert_eq!(cmd.to_gcode(), "G0 X10.000 Y20.000 F3000");
    }

    #[test]
    fn test_linear_move() {
        let cmd = GCodeCommand::LinearMove {
            x: Some(10.0),
            y: Some(20.0),
            z: None,
            e: Some(1.5),
            f: Some(1200.0),
        };
        assert_eq!(cmd.to_gcode(), "G1 X10.000 Y20.000 E1.50000 F1200");
    }

    #[test]
    fn test_temperature_commands() {
        assert_eq!(
            GCodeCommand::SetExtruderTemp { s: 200 }.to_gcode(),
            "M104 S200"
        );
        assert_eq!(
            GCodeCommand::SetExtruderTempWait { s: 210 }.to_gcode(),
            "M109 S210"
        );
        assert_eq!(GCodeCommand::SetBedTemp { s: 60 }.to_gcode(), "M140 S60");
        assert_eq!(
            GCodeCommand::SetBedTempWait { s: 65 }.to_gcode(),
            "M190 S65"
        );
    }

    #[test]
    fn test_fan_commands() {
        assert_eq!(GCodeCommand::SetFanSpeed { s: 255 }.to_gcode(), "M106 S255");
        assert_eq!(GCodeCommand::FanOff.to_gcode(), "M107");
    }

    #[test]
    fn test_comment() {
        let cmd = GCodeCommand::Comment("Layer 1".to_string());
        assert_eq!(cmd.to_gcode(), "; Layer 1");
    }
}
