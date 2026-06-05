//! Calibration module for printer tuning and testing.
//!
//! This module provides structures and functions for various printer calibration routines:
//! - Pressure Advance (PA) calibration (line and pattern methods)
//! - Flow rate calibration
//! - Temperature towers
//! - Retraction towers
//! - Volumetric speed towers
//! - VFA (Vertical Fine Artifacts) towers
//!
//! C++ Reference:
//! - Calib.hpp
//! - Calib.cpp
//!
//! ## Architecture
//!
//! The calibration system consists of:
//! 1. **Data structures** - Configuration and result types
//! 2. **Base classes** - CalibPressureAdvance with common drawing primitives
//! 3. **Derived classes** - Specific calibration implementations (Line, Pattern, Tower)
//!
//! ## Note on Implementation
//!
//! This is a faithful 1:1 port of `Calib.cpp`. The pure-math primitives
//! (`e_per_mm`, `convert_number_to_string`, `get_distance`, `to_radians`,
//! `speed_adjust`, `number_spacing`, `delta_scale_bed_ext`) are ported exactly.
//!
//! The G-code-emitting members (`move_to`, `draw_digit`, `draw_number`,
//! `draw_line`, `draw_box`, `find_optimal_PA_speed`,
//! `CalibPressureAdvanceLine::*`, `CalibPressureAdvancePattern::*`) are BLOCKED:
//! they require BambuStudio's `DynamicPrintConfig` config-reflection API
//! (`option<ConfigOptionFloatsNullable>(...)->get_at(...)`, `get_abs_value(...)`,
//! `apply(...)`, `full_print_config()`), the string-returning `GCodeWriter`
//! (`retract()`/`travel_to_xy()`/`unretract()`/`extrude_to_xy()`/`set_speed()`/
//! `set_pressure_advance()`/`travel_to_z()`/`apply_print_config()`/
//! `set_xy_offset()`/`set_extruders()`/...), the `GCode` gcodegen wrapper
//! (`config()`/`writer()`), and `Model::plates_custom_gcodes` /
//! `CustomGCode::Info`. None of these exist in the Rust crate in the form
//! required for a faithful translation, so those symbols are intentionally left
//! unported rather than faked. See the parity ledger for status.

use crate::flow::Flow;
use crate::geometry::{BoundingBoxF, PointF};
use crate::{Error, Result};
use serde::{Deserialize, Serialize};

/// Dynamic print configuration (placeholder for full implementation)
/// PrintConfig.hpp
///
/// NOTE: BambuStudio's `DynamicPrintConfig` is a reflective key/value config
/// store accessed via `option<T>(name)->get_at(idx)` / `get_abs_value(name)`.
/// The Rust crate models configuration with concrete typed structs
/// (`crate::print_config::PrintConfig`), so this remains a placeholder until the
/// reflective config layer is ported. The G-code generation members of this
/// module are blocked on it.
#[derive(Debug, Clone, Default)]
pub struct DynamicPrintConfig {
    // TODO: Port full reflective configuration system (DynamicPrintConfig).
    _placeholder: (),
}

// ============================================================================
// Enums
// ============================================================================

/// Calibration mode selection
/// Calib.hpp:12-22
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CalibMode {
    /// No calibration
    /// Calib.hpp:13
    None = 0,

    /// Pressure Advance line test
    /// Calib.hpp:14
    PALine,

    /// Pressure Advance pattern test
    /// Calib.hpp:15
    PAPattern,

    /// Pressure Advance tower test
    /// Calib.hpp:16
    PATower,

    /// Automatic Pressure Advance line test
    /// Calib.hpp:17
    AutoPALine,

    /// Flow rate calibration
    /// Calib.hpp:18
    FlowRate,

    /// Temperature tower calibration
    /// Calib.hpp:19
    TempTower,

    /// Volumetric speed tower
    /// Calib.hpp:20
    VolSpeedTower,

    /// VFA (Vertical Fine Artifacts) tower
    /// Calib.hpp:21
    VFATower,

    /// Retraction tower calibration
    /// Calib.hpp:22
    RetractionTower,
}

impl Default for CalibMode {
    fn default() -> Self {
        CalibMode::None
    }
}

/// Calibration state machine states
/// Calib.hpp:24-32
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CalibState {
    /// Initial state
    /// Calib.hpp:25
    Start = 0,

    /// Preset selection
    /// Calib.hpp:26
    Preset,

    /// Calibration in progress
    /// Calib.hpp:27
    Calibration,

    /// Coarse calibration save
    /// Calib.hpp:28
    CoarseSave,

    /// Fine calibration phase
    /// Calib.hpp:29
    FineCalibration,

    /// Save calibration results
    /// Calib.hpp:30
    Save,

    /// Calibration finished
    /// Calib.hpp:31
    Finish,
}

impl Default for CalibState {
    fn default() -> Self {
        CalibState::Start
    }
}

/// Flow ratio calibration type
/// Calib.hpp:44-47
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FlowRatioCalibrationType {
    /// Complete calibration from scratch
    /// Calib.hpp:45
    CompleteCalibration = 0,

    /// Fine-tuning of existing calibration
    /// Calib.hpp:46
    FineCalibration,
}

impl Default for FlowRatioCalibrationType {
    fn default() -> Self {
        FlowRatioCalibrationType::CompleteCalibration
    }
}

/// Calibration result confidence level
/// Calib.hpp:116-120
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CalibResult {
    /// Calibration successful
    /// Calib.hpp:117
    Success = 0,

    /// Calibration uncertain/problem detected
    /// Calib.hpp:118
    Problem = 1,

    /// Calibration failed
    /// Calib.hpp:119
    Failed = 2,
}

/// Digit drawing mode for calibration labels
/// Calib.hpp:198 (nested enum in CalibPressureAdvance)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DrawDigitMode {
    /// Draw digits from left to right
    /// Calib.hpp:198
    LeftToRight,

    /// Draw digits from bottom to top
    /// Calib.hpp:198
    BottomToTop,
}

impl Default for DrawDigitMode {
    fn default() -> Self {
        DrawDigitMode::LeftToRight
    }
}

/// Extruder type for calibration
/// Referenced in X1CCalibInfo but defined elsewhere
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ExtruderType {
    /// Direct drive extruder
    DirectDrive,
    /// Bowden extruder
    Bowden,
}

/// Nozzle volume type
/// Referenced in X1CCalibInfo but defined elsewhere
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NozzleVolumeType {
    /// Standard nozzle volume
    Standard,
    /// High flow nozzle
    HighFlow,
}

/// Bed type for calibration
/// Referenced in CaliPresetInfo but defined elsewhere
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BedType {
    /// Cool plate
    CoolPlate,
    /// Engineering plate
    EngineeringPlate,
    /// High temp plate
    HighTempPlate,
    /// Textured PEI plate
    TexturedPEI,
}

// ============================================================================
// Data Structures
// ============================================================================

/// Calibration parameters
/// Calib.hpp:34-42
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibParams {
    /// Extruder ID to calibrate
    /// Calib.hpp:37
    pub extruder_id: usize,

    /// Starting value for calibration range
    /// Calib.hpp:38
    pub start: f64,

    /// Ending value for calibration range
    /// Calib.hpp:38
    pub end: f64,

    /// Step size between test values
    /// Calib.hpp:38
    pub step: f64,

    /// Whether to print value labels on the test
    /// Calib.hpp:39
    pub print_numbers: bool,

    /// Calibration mode
    /// Calib.hpp:40
    pub mode: CalibMode,
}

impl Default for CalibParams {
    /// Create default calibration parameters
    /// Calib.hpp:36
    fn default() -> Self {
        Self {
            extruder_id: 0,
            start: 0.0,
            end: 0.0,
            step: 0.0,
            print_numbers: false,
            mode: CalibMode::None,
        }
    }
}

/// X1C printer calibration information (single extruder/filament combo)
/// Calib.hpp:51-68
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct X1CCalibInfo {
    /// Extruder ID
    /// Calib.hpp:52
    pub extruder_id: usize,

    /// Tray ID in AMS
    /// Calib.hpp:53
    pub tray_id: i32,

    /// AMS unit ID
    /// Calib.hpp:54
    pub ams_id: usize,

    /// Slot ID in AMS
    /// Calib.hpp:55
    pub slot_id: usize,

    /// Bed temperature
    /// Calib.hpp:56
    pub bed_temp: i32,

    /// Extruder type
    /// Calib.hpp:57
    pub extruder_type: ExtruderType,

    /// Nozzle volume type
    /// Calib.hpp:58
    pub nozzle_volume_type: NozzleVolumeType,

    /// Nozzle temperature
    /// Calib.hpp:59
    pub nozzle_temp: i32,

    /// Nozzle diameter in mm
    /// Calib.hpp:60
    pub nozzle_diameter: f32,

    /// Filament ID string
    /// Calib.hpp:61
    pub filament_id: String,

    /// Setting ID string
    /// Calib.hpp:62
    pub setting_id: String,

    /// Maximum volumetric speed (mm³/s)
    /// Calib.hpp:63
    pub max_volumetric_speed: f32,

    /// Flow rate ratio (for flow calibration)
    /// Calib.hpp:64
    pub flow_rate: f32,
}

impl Default for X1CCalibInfo {
    fn default() -> Self {
        Self {
            extruder_id: 0,
            tray_id: 0,
            ams_id: 0,
            slot_id: 0,
            bed_temp: 0,
            extruder_type: ExtruderType::DirectDrive,
            nozzle_volume_type: NozzleVolumeType::Standard,
            nozzle_temp: 0,
            nozzle_diameter: 0.4,
            filament_id: String::new(),
            setting_id: String::new(),
            max_volumetric_speed: 0.0,
            flow_rate: 0.98,
        }
    }
}

/// Collection of calibration info for X1C printer
/// Calib.hpp:67-71
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct X1CCalibInfos {
    /// Vector of calibration data for each extruder/filament
    /// Calib.hpp:69
    pub calib_datas: Vec<X1CCalibInfo>,

    /// Current calibration mode
    /// Calib.hpp:70
    pub cali_mode: CalibMode,
}

/// Calibration preset information
/// Calib.hpp:73-100
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CaliPresetInfo {
    /// Tray ID
    /// Calib.hpp:75
    pub tray_id: i32,

    /// Extruder ID
    /// Calib.hpp:76
    pub extruder_id: usize,

    /// Nozzle volume type
    /// Calib.hpp:77
    pub nozzle_volume_type: NozzleVolumeType,

    /// Bed type
    /// Calib.hpp:78
    pub bed_type: BedType,

    /// Nozzle diameter in mm
    /// Calib.hpp:79
    pub nozzle_diameter: f32,

    /// Nozzle position ID (-1 means no position tracking)
    /// Calib.hpp:80
    pub nozzle_pos_id: i32,

    /// Nozzle serial number
    /// Calib.hpp:81
    pub nozzle_sn: String,

    /// Filament ID
    /// Calib.hpp:82
    pub filament_id: String,

    /// Setting ID
    /// Calib.hpp:83
    pub setting_id: String,

    /// Preset name
    /// Calib.hpp:84
    pub name: String,
}

impl Default for CaliPresetInfo {
    fn default() -> Self {
        Self {
            tray_id: 0,
            extruder_id: 0,
            nozzle_volume_type: NozzleVolumeType::Standard,
            bed_type: BedType::CoolPlate,
            nozzle_diameter: 0.4,
            nozzle_pos_id: -1,
            nozzle_sn: String::new(),
            filament_id: String::new(),
            setting_id: String::new(),
            name: String::new(),
        }
    }
}

/// Printer calibration information
/// Calib.hpp:102-108
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrinterCaliInfo {
    /// Device ID
    /// Calib.hpp:104
    pub dev_id: String,

    /// Whether calibration is finished
    /// Calib.hpp:105
    pub cali_finished: bool,

    /// Cached flow ratio value
    /// Calib.hpp:106
    pub cache_flow_ratio: f32,

    /// Selected presets for calibration
    /// Calib.hpp:107
    pub selected_presets: Vec<CaliPresetInfo>,

    /// Flow rate calibration type
    /// Calib.hpp:108
    pub cache_flow_rate_calibration_type: FlowRatioCalibrationType,
}

impl Default for PrinterCaliInfo {
    fn default() -> Self {
        Self {
            dev_id: String::new(),
            cali_finished: true,
            cache_flow_ratio: 0.0,
            selected_presets: Vec::new(),
            cache_flow_rate_calibration_type: FlowRatioCalibrationType::CompleteCalibration,
        }
    }
}

/// Pressure Advance calibration result
/// Calib.hpp:110-134
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PACalibResult {
    /// Extruder ID
    /// Calib.hpp:121
    pub extruder_id: usize,

    /// Nozzle volume type
    /// Calib.hpp:122
    pub nozzle_volume_type: NozzleVolumeType,

    /// Tray ID
    /// Calib.hpp:123
    pub tray_id: usize,

    /// AMS unit ID
    /// Calib.hpp:124
    pub ams_id: usize,

    /// Slot ID
    /// Calib.hpp:125
    pub slot_id: usize,

    /// Calibration index (-1 means default)
    /// Calib.hpp:126
    pub cali_idx: i32,

    /// Nozzle position ID (-1 means no position)
    /// Calib.hpp:127
    pub nozzle_pos_id: i32,

    /// Nozzle diameter
    /// Calib.hpp:128
    pub nozzle_diameter: f32,

    /// Nozzle serial number
    /// Calib.hpp:129
    pub nozzle_sn: String,

    /// Filament ID
    /// Calib.hpp:130
    pub filament_id: String,

    /// Setting ID
    /// Calib.hpp:131
    pub setting_id: String,

    /// Preset name
    /// Calib.hpp:132
    pub name: String,

    /// Pressure advance K value
    /// Calib.hpp:133
    pub k_value: f32,

    /// Pressure advance N coefficient
    /// Calib.hpp:134
    pub n_coef: f32,

    /// Confidence level (0: success, 1: uncertain, 2: failed)
    /// Calib.hpp:135
    pub confidence: i32,
}

impl Default for PACalibResult {
    fn default() -> Self {
        Self {
            extruder_id: 0,
            nozzle_volume_type: NozzleVolumeType::Standard,
            tray_id: 0,
            ams_id: 0,
            slot_id: 0,
            cali_idx: -1,
            nozzle_pos_id: -1,
            nozzle_diameter: 0.4,
            nozzle_sn: String::new(),
            filament_id: String::new(),
            setting_id: String::new(),
            name: String::new(),
            k_value: 0.0,
            n_coef: 0.0,
            confidence: -1,
        }
    }
}

/// Pressure Advance calibration index information
/// Calib.hpp:137-148
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PACalibIndexInfo {
    /// Extruder ID
    /// Calib.hpp:138
    pub extruder_id: usize,

    /// Nozzle volume type
    /// Calib.hpp:139
    pub nozzle_volume_type: NozzleVolumeType,

    /// Tray ID
    /// Calib.hpp:140
    pub tray_id: usize,

    /// AMS unit ID
    /// Calib.hpp:141
    pub ams_id: usize,

    /// Slot ID
    /// Calib.hpp:142
    pub slot_id: usize,

    /// Calibration index (-1 means default)
    /// Calib.hpp:143
    pub cali_idx: i32,

    /// Nozzle position ID (-1 means no position)
    /// Calib.hpp:144
    pub nozzle_pos_id: i32,

    /// Nozzle diameter
    /// Calib.hpp:145
    pub nozzle_diameter: f32,

    /// Nozzle serial number
    /// Calib.hpp:146
    pub nozzle_sn: String,

    /// Filament ID
    /// Calib.hpp:147
    pub filament_id: String,
}

impl Default for PACalibIndexInfo {
    fn default() -> Self {
        Self {
            extruder_id: 0,
            nozzle_volume_type: NozzleVolumeType::Standard,
            tray_id: 0,
            ams_id: 0,
            slot_id: 0,
            cali_idx: -1,
            nozzle_pos_id: -1,
            nozzle_diameter: 0.4,
            nozzle_sn: String::new(),
            filament_id: String::new(),
        }
    }
}

/// Pressure Advance calibration extruder information
/// Calib.hpp:150-160
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PACalibExtruderInfo {
    /// Extruder ID
    /// Calib.hpp:151
    pub extruder_id: usize,

    /// Nozzle volume type
    /// Calib.hpp:152
    pub nozzle_volume_type: NozzleVolumeType,

    /// Nozzle position ID (-1 means no position)
    /// Calib.hpp:153
    pub nozzle_pos_id: i32,

    /// Nozzle diameter
    /// Calib.hpp:154
    pub nozzle_diameter: f32,

    /// Nozzle serial number
    /// Calib.hpp:155
    pub nozzle_sn: String,

    /// Filament ID (empty string means no filament)
    /// Calib.hpp:156
    pub filament_id: String,

    /// Whether to use extruder ID for matching
    /// Calib.hpp:157
    pub use_extruder_id: bool,

    /// Whether to use nozzle volume type for matching
    /// Calib.hpp:158
    pub use_nozzle_volume_type: bool,
}

impl Default for PACalibExtruderInfo {
    fn default() -> Self {
        Self {
            extruder_id: 0,
            nozzle_volume_type: NozzleVolumeType::Standard,
            nozzle_pos_id: -1,
            nozzle_diameter: 0.4,
            nozzle_sn: String::new(),
            filament_id: String::new(),
            use_extruder_id: true,
            use_nozzle_volume_type: true,
        }
    }
}

/// Pressure Advance calibration tab information
/// Calib.hpp:162-167
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PACalibTabInfo {
    /// Nozzle diameter for PA calibration tab
    /// Calib.hpp:163
    pub pa_calib_tab_nozzle_dia: f32,

    /// Extruder ID
    /// Calib.hpp:164
    pub extruder_id: usize,

    /// Nozzle volume type
    /// Calib.hpp:165
    pub nozzle_volume_type: NozzleVolumeType,
}

impl Default for PACalibTabInfo {
    fn default() -> Self {
        Self {
            pa_calib_tab_nozzle_dia: 0.4,
            extruder_id: 0,
            nozzle_volume_type: NozzleVolumeType::Standard,
        }
    }
}

/// Flow ratio calibration result
/// Calib.hpp:169-177
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlowRatioCalibResult {
    /// Tray ID
    /// Calib.hpp:171
    pub tray_id: i32,

    /// Nozzle diameter
    /// Calib.hpp:172
    pub nozzle_diameter: f32,

    /// Filament ID
    /// Calib.hpp:173
    pub filament_id: String,

    /// Setting ID
    /// Calib.hpp:174
    pub setting_id: String,

    /// Calibrated flow ratio
    /// Calib.hpp:175
    pub flow_ratio: f32,

    /// Confidence level (0: success, 1: uncertain, 2: failed)
    /// Calib.hpp:176
    pub confidence: i32,
}

impl Default for FlowRatioCalibResult {
    fn default() -> Self {
        Self {
            tray_id: 0,
            nozzle_diameter: 0.4,
            filament_id: String::new(),
            setting_id: String::new(),
            flow_ratio: 1.0,
            confidence: -1,
        }
    }
}

/// Optional arguments for drawing calibration boxes
/// Calib.hpp:179-188
#[derive(Debug, Clone, Copy)]
pub struct DrawBoxOptArgs {
    /// Whether the box should be filled
    /// Calib.hpp:183
    pub is_filled: bool,

    /// Number of perimeters to draw
    /// Calib.hpp:184
    pub num_perimeters: i32,

    /// Layer height in mm
    /// Calib.hpp:185
    pub height: f64,

    /// Line width in mm
    /// Calib.hpp:186
    pub line_width: f64,

    /// Print speed in mm/s
    /// Calib.hpp:187
    pub speed: f64,
}

impl DrawBoxOptArgs {
    /// Create DrawBoxOptArgs with specified parameters
    /// Calib.hpp:181
    pub fn new(num_perimeters: i32, height: f64, line_width: f64, speed: f64) -> Self {
        Self {
            is_filled: false,
            num_perimeters,
            height,
            line_width,
            speed,
        }
    }
}

impl Default for DrawBoxOptArgs {
    /// Create default DrawBoxOptArgs
    /// Calib.hpp:182
    fn default() -> Self {
        Self {
            is_filled: false,
            num_perimeters: 0,
            height: 0.0,
            line_width: 0.0,
            speed: 0.0,
        }
    }
}

/// Suggested configuration for PA pattern calibration
/// Calib.hpp:272-282
#[derive(Debug, Clone)]
pub struct SuggestedConfigCalibPAPattern {
    /// Float parameter pairs (name, value)
    /// Calib.hpp:273
    pub float_pairs: Vec<(String, f64)>,

    /// Float array parameter pairs (name, values)
    /// Calib.hpp:275
    pub floats_pairs: Vec<(String, Vec<f64>)>,

    /// Nozzle ratio parameter pairs (name, ratio as percentage)
    /// Calib.hpp:277
    pub nozzle_ratio_pairs: Vec<(String, f64)>,

    /// Integer parameter pairs (name, value)
    /// Calib.hpp:279
    pub int_pairs: Vec<(String, i32)>,
}

impl Default for SuggestedConfigCalibPAPattern {
    /// Create default suggested configuration
    /// Calib.hpp:273-281
    fn default() -> Self {
        Self {
            float_pairs: vec![
                ("initial_layer_print_height".to_string(), 0.25),
                ("layer_height".to_string(), 0.2),
            ],
            floats_pairs: vec![("initial_layer_speed".to_string(), vec![30.0])],
            nozzle_ratio_pairs: vec![
                ("line_width".to_string(), 112.5),
                ("initial_layer_line_width".to_string(), 140.0),
            ],
            int_pairs: vec![
                ("skirt_loops".to_string(), 0),
                ("wall_loops".to_string(), 3),
            ],
        }
    }
}

// ============================================================================
// Base Calibration Class
// ============================================================================

/// Base class for Pressure Advance calibration with common drawing primitives.
///
/// The pure-math primitives (`e_per_mm`, `convert_number_to_string`,
/// `get_distance`, `to_radians`, `speed_adjust`, `number_spacing`,
/// `delta_scale_bed_ext`) are ported faithfully. The G-code-emitting
/// primitives (`move_to`, `draw_digit`, `draw_number`, `draw_line`,
/// `draw_box`) and `find_optimal_PA_speed` are BLOCKED on the string-returning
/// `GCodeWriter` and `DynamicPrintConfig` reflection (see member docs).
///
/// Calib.hpp:193-235
#[derive(Debug, Clone)]
pub struct CalibPressureAdvance {
    /// Last position for movement calculations
    /// Calib.hpp:213
    pub last_pos: (f64, f64, f64),

    /// Print configuration
    /// Calib.hpp:214
    pub config: DynamicPrintConfig,

    /// Encroachment factor for digit drawing
    /// Calib.hpp:216
    pub encroachment: f64,

    /// Digit drawing mode
    /// Calib.hpp:217
    pub draw_digit_mode: DrawDigitMode,

    /// Length of digit segments
    /// Calib.hpp:218
    pub digit_segment_len: f64,

    /// Gap between digit segments
    /// Calib.hpp:219
    pub digit_gap_len: f64,

    /// Maximum number length for labels
    /// Calib.hpp:220
    pub max_number_len: usize,
}

impl CalibPressureAdvance {
    /// Create a new CalibPressureAdvance instance
    /// Calib.hpp:193-195
    pub fn new(config: DynamicPrintConfig) -> Self {
        Self {
            last_pos: (0.0, 0.0, 0.0),
            config,
            encroachment: 1.0 / 3.0,
            draw_digit_mode: DrawDigitMode::LeftToRight,
            digit_segment_len: 2.0,
            digit_gap_len: 1.0,
            max_number_len: 5,
        }
    }

    /// Find optimal PA test speed based on volumetric limits
    ///
    /// **BLOCKED:** Faithful translation requires the reflective
    /// `DynamicPrintConfig` API
    /// (`option<ConfigOptionFloatsNullable>("filament_max_volumetric_speed")
    /// ->get_at(filament_idx)`, `..."nozzle_diameter"->get_at(extruder_id)`,
    /// `..."outer_wall_speed"->get_at(extruder_id)`), which is not ported.
    ///
    /// Calib.hpp:191
    /// Calib.cpp:9-18
    pub fn find_optimal_pa_speed(
        _config: &DynamicPrintConfig,
        _line_width: f64,
        _layer_height: f64,
        _extruder_id: usize,
        _filament_idx: usize,
    ) -> Result<f32> {
        // BLOCKED on DynamicPrintConfig reflective option API. C++ Calib.cpp:9-18:
        //   const double general_suggested_min_speed   = 100.0;
        //   double filament_max_volumetric_speed =
        //       config.option<ConfigOptionFloatsNullable>("filament_max_volumetric_speed")->get_at(filament_idx);
        //   Flow pattern_line = Flow(line_width, layer_height,
        //       config.option<ConfigOptionFloatsNullable>("nozzle_diameter")->get_at(extruder_id));
        //   auto pa_speed = std::min(std::max(general_suggested_min_speed,
        //       config.option<ConfigOptionFloatsNullable>("outer_wall_speed")->get_at(extruder_id)),
        //       filament_max_volumetric_speed / pattern_line.mm3_per_mm());
        //   return std::floor(pa_speed);
        Err(Error::Config(
            "CalibPressureAdvance::find_optimal_PA_speed blocked on DynamicPrintConfig reflective option API"
                .to_string(),
        ))
    }

    /// Compute extrusion length per mm of travel for the given line geometry.
    /// Calib.hpp:204
    /// Calib.cpp:33-39
    pub fn e_per_mm(
        &self,
        line_width: f64,
        layer_height: f64,
        nozzle_diameter: f32,
        filament_diameter: f32,
        print_flow_ratio: f32,
    ) -> Result<f64> {
        // Calib.cpp:35
        let line_flow = Flow::new(line_width, layer_height, nozzle_diameter as f64)?;
        // Calib.cpp:36
        let filament_area =
            std::f64::consts::PI * (filament_diameter as f64 / 2.0).powi(2);

        // Calib.cpp:38
        Ok(line_flow.mm3_per_mm()? / filament_area * print_flow_ratio as f64)
    }

    /// Convert a number to its trailing-zero-stripped decimal string.
    /// Calib.hpp:201
    /// Calib.cpp:41-48
    pub fn convert_number_to_string(&self, num: f64) -> String {
        // Calib.cpp:43 — std::to_string(double) formats with %f (6 decimals).
        let mut s_number = format!("{:.6}", num);
        // Calib.cpp:44 — erase trailing zeros.
        if let Some(pos) = s_number.rfind(|c| c != '0') {
            s_number.truncate(pos + 1);
        }
        // Calib.cpp:45 — erase a trailing '.' if it is now last.
        if let Some(pos) = s_number.rfind(|c| c != '.') {
            s_number.truncate(pos + 1);
        }

        // Calib.cpp:47
        s_number
    }

    /// Convert degrees to radians
    /// Calib.hpp:215 — `double to_radians(double degrees) const { return degrees * M_PI / 180; }`
    pub fn to_radians(&self, degrees: f64) -> f64 {
        degrees * std::f64::consts::PI / 180.0
    }

    /// Get distance between two points
    /// Calib.hpp:216
    /// Calib.cpp:205 — `return std::hypot((to.x() - from.x()), (to.y() - from.y()));`
    pub fn get_distance(&self, from: PointF, to: PointF) -> f64 {
        (to.x - from.x).hypot(to.y - from.y)
    }

    /// Adjust speed for G-code output (convert mm/s to mm/min)
    /// Calib.hpp:204 — `double speed_adjust(int speed) const { return speed * 60; }`
    pub fn speed_adjust(&self, speed: i32) -> i32 {
        speed * 60
    }

    /// Calculate number spacing for label positioning
    /// Calib.hpp:206 — `double number_spacing() const { return m_digit_segment_len + m_digit_gap_len; }`
    pub fn number_spacing(&self) -> f64 {
        self.digit_segment_len + self.digit_gap_len
    }

    /// Scale bed extents for delta printers
    /// Calib.hpp:200 — `void delta_scale_bed_ext(BoundingBoxf &bed_ext) const { bed_ext.scale(1.0f / 1.41421f); }`
    pub fn delta_scale_bed_ext(&self, bed_ext: &mut BoundingBoxF) {
        // BoundingBoxf::scale(factor) multiplies both min and max by factor.
        // 1.0f / 1.41421f computed in f32 to match the C++ literal precision.
        let scale = (1.0f32 / 1.41421f32) as f64;
        bed_ext.min.x *= scale;
        bed_ext.min.y *= scale;
        bed_ext.max.x *= scale;
        bed_ext.max.y *= scale;
    }
}

// ============================================================================
// Derived Calibration Classes
// ============================================================================

/// Pressure Advance line test calibration.
///
/// Generates a series of horizontal lines with varying PA values and speeds
/// to visualize the effect of pressure advance settings.
///
/// **PARTIAL:** `delta_modify_start` and `set_speed` are ported; the
/// G-code-emitting members (`generate_test`, `print_pa_lines`, `is_delta`) are
/// BLOCKED on the `GCode` gcodegen wrapper and the string-returning
/// `GCodeWriter` (see member docs).
///
/// Calib.hpp:237-274
#[derive(Debug, Clone)]
pub struct CalibPressureAdvanceLine {
    /// Base calibration functionality
    pub base: CalibPressureAdvance,

    /// Nozzle diameter in mm
    /// Calib.hpp:251
    pub nozzle_diameter: f64,

    /// Slow speed for PA test (mm/s)
    /// Calib.hpp:252
    pub slow_speed: f64,

    /// Fast speed for PA test (mm/s)
    /// Calib.hpp:252
    pub fast_speed: f64,

    /// Layer height in mm
    /// Calib.hpp:254
    pub height_layer: f64,

    /// Line width in mm
    /// Calib.hpp:255
    pub line_width: f64,

    /// Thin line width in mm (for fine details)
    /// Calib.hpp:256
    pub thin_line_width: f64,

    /// Number line width in mm (for labels)
    /// Calib.hpp:257
    pub number_line_width: f64,

    /// Vertical spacing between lines
    /// Calib.hpp:258
    pub space_y: f64,

    /// Length of short line segments
    /// Calib.hpp:260
    pub length_short: f64,

    /// Length of long line segments
    /// Calib.hpp:260
    pub length_long: f64,

    /// Whether to draw value labels
    /// Calib.hpp:261
    pub draw_numbers: bool,
}

impl CalibPressureAdvanceLine {
    /// Create a new PA line calibration
    /// Calib.hpp:222
    /// Calib.cpp:407-413
    pub fn new(config: DynamicPrintConfig) -> Self {
        Self {
            base: CalibPressureAdvance::new(config),
            nozzle_diameter: 0.4,
            slow_speed: 20.0,
            fast_speed: 100.0,
            height_layer: 0.2,
            line_width: 0.6,
            thin_line_width: 0.44,
            number_line_width: 0.48,
            space_y: 3.5,
            length_short: 20.0,
            length_long: 40.0,
            draw_numbers: true,
        }
    }

    /// Generate PA line test G-code
    ///
    /// **BLOCKED:** Faithful translation requires the `GCode` gcodegen wrapper
    /// (`config()`/`writer()`), the string-returning `GCodeWriter`
    /// (`set_pressure_advance`, `travel_to_xy`, `extrude_to_xy`, `set_speed`, ...)
    /// and `get_extents(printable_area.values)` — none ported in the required
    /// form. See Calib.cpp:415-431 / print_pa_lines Calib.cpp:435-490.
    ///
    /// Calib.hpp:243
    /// Calib.cpp:415-431
    pub fn generate_test(
        &mut self,
        _start_pa: f64,
        _step_pa: f64,
        _count: usize,
    ) -> Result<String> {
        Err(Error::Config(
            "CalibPressureAdvanceLine::generate_test blocked on GCode gcodegen + string GCodeWriter"
                .to_string(),
        ))
    }

    /// Check if printer is delta type
    ///
    /// **BLOCKED:** C++ Calib.cpp:433 reads
    /// `mp_gcodegen->config().printable_area.values.size() > 4`, which requires
    /// the `GCode` gcodegen wrapper and the reflective config. Not ported.
    ///
    /// Calib.hpp:253
    /// Calib.cpp:433
    pub fn is_delta(&self) -> bool {
        // BLOCKED on GCode gcodegen / printable_area config.
        false
    }

    /// Set speed parameters
    /// Calib.hpp:245-249
    pub fn set_speed(&mut self, fast: f64, slow: f64) {
        // Calib.hpp:247-248 — m_slow_speed = slow; m_fast_speed = fast;
        self.slow_speed = slow;
        self.fast_speed = fast;
    }

    /// Shift the start position for delta printers.
    /// Calib.hpp:259
    /// Calib.cpp:492-496
    pub fn delta_modify_start(&self, startx: &mut f64, starty: &mut f64, count: i32) {
        // Calib.cpp:494-495
        *startx = -*startx;
        *starty = -(count as f64 * self.space_y) / 2.0;
    }
}

/// Pressure Advance pattern test calibration.
///
/// Generates a grid of test patterns with varying PA values, using
/// corner features to visualize the effect of pressure advance.
///
/// **PARTIAL:** The G-code-emitting members and config-derived dimension
/// getters are BLOCKED on `DynamicPrintConfig` reflection, the string-returning
/// `GCodeWriter`, and `Model`/`CustomGCode` injection (see member docs).
///
/// Calib.hpp:289-360
#[derive(Debug, Clone)]
pub struct CalibPressureAdvancePattern {
    /// Base calibration functionality
    pub base: CalibPressureAdvance,

    /// Calibration parameters
    /// Calib.hpp:322
    pub params: CalibParams,

    /// Starting point offset
    /// Calib.hpp:325
    pub starting_point: (f64, f64, f64),

    /// Whether starting point is fixed
    /// Calib.hpp:326
    pub is_start_point_fixed: bool,

    /// Handle XY size for manipulation
    /// Calib.hpp:328
    pub handle_xy_size: f64,

    /// Spacing between handles
    /// Calib.hpp:329
    pub handle_spacing: f64,

    /// Number of layers to print
    /// Calib.hpp:330
    pub num_layers: usize,

    /// Side length of test walls
    /// Calib.hpp:332
    pub wall_side_length: f64,

    /// Corner angle for test patterns
    /// Calib.hpp:333
    pub corner_angle: i32,

    /// Spacing between patterns
    /// Calib.hpp:334
    pub pattern_spacing: f64,

    /// Horizontal padding for labels
    /// Calib.hpp:336
    pub glyph_padding_horizontal: f64,

    /// Vertical padding for labels
    /// Calib.hpp:337
    pub glyph_padding_vertical: f64,
}

impl CalibPressureAdvancePattern {
    /// Create a new PA pattern calibration
    /// Calib.hpp:294
    /// Calib.cpp:498-504
    ///
    /// NOTE: the C++ ctor also calls `refresh_setup(config, is_bbl_machine,
    /// model, origin)` (Calib.cpp:503), which is BLOCKED on `DynamicPrintConfig`
    /// reflection, `Model`, and the `GCodeWriter` setup chain. That call is
    /// omitted here; the field initializers below mirror the C++ member defaults.
    pub fn new(params: CalibParams, config: DynamicPrintConfig) -> Self {
        Self {
            base: CalibPressureAdvance {
                // Calib.cpp:501 — this->m_draw_digit_mode = DrawDigitMode::Bottom_To_Top;
                draw_digit_mode: DrawDigitMode::BottomToTop,
                ..CalibPressureAdvance::new(config)
            },
            params,
            starting_point: (0.0, 0.0, 0.0),
            is_start_point_fixed: false,
            handle_xy_size: 5.0,
            handle_spacing: 2.0,
            num_layers: 4,
            wall_side_length: 30.0,
            corner_angle: 90,
            pattern_spacing: 2.0,
            glyph_padding_horizontal: 1.0,
            glyph_padding_vertical: 1.0,
        }
    }

    /// Generate custom G-codes for PA pattern test
    ///
    /// **BLOCKED:** Faithful translation requires `DynamicPrintConfig`
    /// reflection, the string-returning `GCodeWriter`, `Model` /
    /// `Model::plates_custom_gcodes`, and `CustomGCode::Info`/`Item` injection —
    /// none ported in the required form.
    ///
    /// Calib.hpp:302
    /// Calib.cpp:506-656
    pub fn generate_custom_gcodes(&mut self) -> Result<()> {
        Err(Error::Config(
            "CalibPressureAdvancePattern::generate_custom_gcodes blocked on DynamicPrintConfig + string GCodeWriter + Model custom gcode injection"
                .to_string(),
        ))
    }

    /// Set starting position offset
    /// Calib.hpp:297
    /// Calib.cpp:650-654
    pub fn set_start_offset(&mut self, offset: (f64, f64, f64)) {
        self.starting_point = offset;
        self.is_start_point_fixed = true;
    }

    /// Get starting position offset
    /// Calib.hpp:298
    /// Calib.cpp:656
    pub fn get_start_offset(&self) -> (f64, f64, f64) {
        self.starting_point
    }

    /// Get handle size
    /// Calib.hpp:289
    pub fn handle_xy_size(&self) -> f64 {
        self.handle_xy_size
    }

    /// Get handle spacing
    /// Calib.hpp:290
    pub fn handle_spacing(&self) -> f64 {
        self.handle_spacing
    }

    /// Get total print size X
    ///
    /// **BLOCKED:** C++ `print_size_x() = object_size_x() + pattern_shift()`
    /// (Calib.hpp:298); both `object_size_x()` (Calib.cpp:734) and
    /// `pattern_shift()` (Calib.cpp:806) read `m_config` through the reflective
    /// `DynamicPrintConfig` option API (`line_width()`, `wall_count()`,
    /// `line_spacing_first_layer()`, ...). Not ported.
    ///
    /// Calib.hpp:298
    pub fn print_size_x(&self) -> f64 {
        unimplemented!(
            "CalibPressureAdvancePattern::print_size_x blocked on DynamicPrintConfig option API"
        )
    }

    /// Get total print size Y
    ///
    /// **BLOCKED:** C++ `print_size_y() = object_size_y()` (Calib.hpp:299),
    /// where `object_size_y()` (Calib.cpp:740) reads `m_config` through the
    /// reflective `DynamicPrintConfig` option API. Not ported.
    ///
    /// Calib.hpp:299
    pub fn print_size_y(&self) -> f64 {
        unimplemented!(
            "CalibPressureAdvancePattern::print_size_y blocked on DynamicPrintConfig option API"
        )
    }

    /// Get maximum layer Z height
    ///
    /// **BLOCKED:** C++ `max_layer_z() = height_first_layer() +
    /// ((m_num_layers - 1) * height_layer())` (Calib.hpp:300), where
    /// `height_first_layer()` / `height_layer()` read
    /// `m_config.option<ConfigOptionFloat>("initial_layer_print_height"|"layer_height")->value`.
    /// Not ported.
    ///
    /// Calib.hpp:300
    pub fn max_layer_z(&self) -> f64 {
        unimplemented!(
            "CalibPressureAdvancePattern::max_layer_z blocked on DynamicPrintConfig option API"
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calib_mode_default() {
        assert_eq!(CalibMode::default(), CalibMode::None);
    }

    #[test]
    fn test_calib_state_default() {
        assert_eq!(CalibState::default(), CalibState::Start);
    }

    #[test]
    fn test_flow_ratio_calibration_type_default() {
        assert_eq!(
            FlowRatioCalibrationType::default(),
            FlowRatioCalibrationType::CompleteCalibration
        );
    }

    #[test]
    fn test_calib_params_default() {
        let params = CalibParams::default();
        assert_eq!(params.extruder_id, 0);
        assert_eq!(params.mode, CalibMode::None);
        assert!(!params.print_numbers);
    }

    #[test]
    fn test_x1c_calib_info_default() {
        let info = X1CCalibInfo::default();
        assert_eq!(info.extruder_id, 0);
        assert_eq!(info.nozzle_diameter, 0.4);
        assert_eq!(info.flow_rate, 0.98);
    }

    #[test]
    fn test_pa_calib_result_default() {
        let result = PACalibResult::default();
        assert_eq!(result.k_value, 0.0);
        assert_eq!(result.n_coef, 0.0);
        assert_eq!(result.confidence, -1);
    }

    #[test]
    fn test_draw_box_opt_args() {
        let args = DrawBoxOptArgs::new(3, 0.2, 0.4, 50.0);
        assert_eq!(args.num_perimeters, 3);
        assert_eq!(args.height, 0.2);
        assert_eq!(args.line_width, 0.4);
        assert_eq!(args.speed, 50.0);
        assert!(!args.is_filled);
    }

    #[test]
    fn test_calib_pressure_advance_to_radians() {
        let calib = CalibPressureAdvance::new(DynamicPrintConfig::default());
        let radians = calib.to_radians(180.0);
        assert!((radians - std::f64::consts::PI).abs() < 1e-10);
    }

    #[test]
    fn test_calib_pressure_advance_get_distance() {
        let calib = CalibPressureAdvance::new(DynamicPrintConfig::default());
        let p1 = PointF::new(0.0, 0.0);
        let p2 = PointF::new(3.0, 4.0);
        let distance = calib.get_distance(p1, p2);
        assert!((distance - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_calib_pressure_advance_speed_adjust() {
        let calib = CalibPressureAdvance::new(DynamicPrintConfig::default());
        assert_eq!(calib.speed_adjust(50), 3000); // 50 mm/s * 60 = 3000 mm/min
    }

    #[test]
    fn test_calib_pressure_advance_number_spacing() {
        let calib = CalibPressureAdvance::new(DynamicPrintConfig::default());
        assert_eq!(calib.number_spacing(), 3.0); // 2.0 + 1.0
    }

    #[test]
    fn test_calib_pressure_advance_line_new() {
        let line = CalibPressureAdvanceLine::new(DynamicPrintConfig::default());
        assert_eq!(line.height_layer, 0.2);
        assert_eq!(line.line_width, 0.6);
        assert!(line.draw_numbers);
    }

    #[test]
    fn test_calib_pressure_advance_line_set_speed() {
        let mut line = CalibPressureAdvanceLine::new(DynamicPrintConfig::default());
        line.set_speed(120.0, 30.0);
        assert_eq!(line.fast_speed, 120.0);
        assert_eq!(line.slow_speed, 30.0);
    }

    #[test]
    fn test_calib_pressure_advance_pattern_new() {
        let params = CalibParams::default();
        let pattern = CalibPressureAdvancePattern::new(params, DynamicPrintConfig::default());
        assert_eq!(pattern.num_layers, 4);
        assert_eq!(pattern.wall_side_length, 30.0);
        assert_eq!(pattern.corner_angle, 90);
    }

    #[test]
    fn test_calib_pressure_advance_pattern_offset() {
        let params = CalibParams::default();
        let mut pattern = CalibPressureAdvancePattern::new(params, DynamicPrintConfig::default());
        pattern.set_start_offset((10.0, 20.0, 0.5));
        let offset = pattern.get_start_offset();
        assert_eq!(offset.0, 10.0);
        assert_eq!(offset.1, 20.0);
        assert_eq!(offset.2, 0.5);
    }

    #[test]
    fn test_suggested_config_default() {
        let config = SuggestedConfigCalibPAPattern::default();
        assert_eq!(config.float_pairs.len(), 2);
        assert_eq!(config.floats_pairs.len(), 1);
        assert_eq!(config.nozzle_ratio_pairs.len(), 2);
        assert_eq!(config.int_pairs.len(), 2);
    }

    #[test]
    fn test_convert_number_to_string() {
        // Calib.cpp:41-48 — strips trailing zeros, then a trailing '.'.
        let calib = CalibPressureAdvance::new(DynamicPrintConfig::default());
        assert_eq!(calib.convert_number_to_string(0.02), "0.02");
        assert_eq!(calib.convert_number_to_string(0.0), "0");
        assert_eq!(calib.convert_number_to_string(1.0), "1");
        assert_eq!(calib.convert_number_to_string(0.125), "0.125");
        assert_eq!(calib.convert_number_to_string(10.5), "10.5");
    }

    #[test]
    fn test_e_per_mm() {
        // Calib.cpp:33-39 — mm3_per_mm / filament_area * print_flow_ratio.
        let calib = CalibPressureAdvance::new(DynamicPrintConfig::default());
        let line_width = 0.45;
        let layer_height = 0.2;
        let nozzle_diameter = 0.4_f32;
        let filament_diameter = 1.75_f32;
        let print_flow_ratio = 0.98_f32;

        let e = calib
            .e_per_mm(
                line_width,
                layer_height,
                nozzle_diameter,
                filament_diameter,
                print_flow_ratio,
            )
            .expect("e_per_mm");

        let flow = crate::flow::Flow::new(line_width, layer_height, nozzle_diameter as f64).unwrap();
        let filament_area =
            std::f64::consts::PI * (filament_diameter as f64 / 2.0).powi(2);
        let expected = flow.mm3_per_mm().unwrap() / filament_area * print_flow_ratio as f64;
        assert!((e - expected).abs() < 1e-12);
    }

    #[test]
    fn test_delta_modify_start() {
        // Calib.cpp:492-496 — startx = -startx; starty = -(count * m_space_y) / 2.
        let line = CalibPressureAdvanceLine::new(DynamicPrintConfig::default());
        let mut startx = 12.0;
        let mut starty = 99.0;
        line.delta_modify_start(&mut startx, &mut starty, 10);
        assert_eq!(startx, -12.0);
        assert_eq!(starty, -(10.0 * line.space_y) / 2.0);
    }

    #[test]
    fn test_delta_scale_bed_ext() {
        // Calib.hpp:200 — bed_ext.scale(1.0f / 1.41421f).
        let calib = CalibPressureAdvance::new(DynamicPrintConfig::default());
        let mut bbox =
            BoundingBoxF::from_points_minmax(PointF::new(0.0, 0.0), PointF::new(200.0, 200.0));
        calib.delta_scale_bed_ext(&mut bbox);
        let scale = (1.0f32 / 1.41421f32) as f64;
        assert!((bbox.max.x - 200.0 * scale).abs() < 1e-9);
        assert!((bbox.max.y - 200.0 * scale).abs() < 1e-9);
    }
}
