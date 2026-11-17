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
//! `find_optimal_PA_speed`, `SuggestedConfigCalibPAPattern` and the configuration and geometry
//! members of `CalibPressureAdvancePattern` read the reflective `DynamicPrintConfig`
//! (`print_config::config_def`, re-exported here as [`FullDynamicPrintConfig`]) and are compared
//! with the pin by `tests/calib_native_parity.rs`.
//!
//! The G-code-emitting members (`move_to`, `draw_digit`, `draw_number`, `draw_line`, `draw_box`,
//! `CalibPressureAdvanceLine::{generate_test, print_pa_lines}`,
//! `CalibPressureAdvancePattern::{_refresh_writer, generate_custom_gcodes}`) write through the
//! faithful writer (`g_code_writer.rs`, set up with `apply_print_config`) and, for the line test,
//! the writer of a `g_code::gcode_class::GCode`; `tests/calib_native_parity.rs` compares their
//! G-code with the pin's.
//!
//! The struct `DynamicPrintConfig` below is the earlier placeholder, still imported by other
//! modules; the calibration classes do not use it.

use crate::config::{
    ConfigBase, ConfigOptionFloat, ConfigOptionFloats, ConfigOptionFloatsNullable, ConfigOptionInt, ConfigOptionResolverTyped,
};
use crate::flow::Flow;
use crate::g_code_writer::GCodeWriter;
use crate::geometry::{BoundingBoxF, PointF};
/// The reflective `DynamicPrintConfig` of PrintConfig.hpp, which the calibration classes read
/// (`m_config.option<...>(key)->get_at(i)`, `get_abs_value(key)`).
pub use crate::print_config::config_def::DynamicPrintConfig as FullDynamicPrintConfig;
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

impl SuggestedConfigCalibPAPattern {
    /// Calib.hpp:276 `static constexpr double nozzle_diameter_0_2 = 0.2;`
    pub const NOZZLE_DIAMETER_0_2: f64 = 0.2;
    /// Calib.hpp:277 `static constexpr double nozzle_compare_epsilon = 1e-6;`
    pub const NOZZLE_COMPARE_EPSILON: f64 = 1e-6;

    /// Calib.hpp:279-282 `double initial_layer_print_height(double nozzle_diameter) const`
    pub fn initial_layer_print_height(&self, nozzle_diameter: f64) -> f64 {
        if (nozzle_diameter - Self::NOZZLE_DIAMETER_0_2).abs() < Self::NOZZLE_COMPARE_EPSILON {
            0.2
        } else {
            0.25
        }
    }

    /// Calib.hpp:284-287 `std::vector<std::pair<std::string, double>> float_pairs(double nozzle_diameter) const`
    pub fn float_pairs(&self, nozzle_diameter: f64) -> Vec<(String, f64)> {
        vec![
            ("initial_layer_print_height".to_string(), self.initial_layer_print_height(nozzle_diameter)),
            ("layer_height".to_string(), 0.2),
        ]
    }

    /// Calib.hpp:295 `const std::pair<std::string, BrimType> brim_pair{"brim_type", BrimType::btNoBrim};`
    pub fn brim_pair(&self) -> (String, crate::print_config::BrimType) {
        ("brim_type".to_string(), crate::print_config::BrimType::NoBrim)
    }
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

/// `std::min(a, b)` of doubles: `(b < a) ? b : a`.
#[inline]
fn std_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `std::max(a, b)` of doubles: `(a < b) ? b : a`.
#[inline]
fn std_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// `config.option<ConfigOptionFloatsNullable>(key)->get_at(idx)` (also read when the store holds the
/// non-nullable class of the same type code, which the C++ cast reads the same way).
fn floats_nullable_at(config: &FullDynamicPrintConfig, key: &str, idx: usize) -> Result<f64> {
    if let Some(o) = config.option_typed::<ConfigOptionFloatsNullable>(key) {
        return Ok(*o.get_at(idx));
    }
    if let Some(o) = config.option_typed::<ConfigOptionFloats>(key) {
        return Ok(*o.get_at(idx));
    }
    Err(Error::Config(format!("calib: no floats option {key}")))
}

/// `config.option<ConfigOptionFloats>(key)->get_at(idx)` (or the nullable class).
fn floats_at(config: &FullDynamicPrintConfig, key: &str, idx: usize) -> Result<f64> {
    floats_nullable_at(config, key, idx)
}

/// `config.option<ConfigOptionFloats>(key)->values` (or the nullable class).
fn floats_values(config: &FullDynamicPrintConfig, key: &str) -> Result<Vec<f64>> {
    if let Some(o) = config.option_typed::<ConfigOptionFloatsNullable>(key) {
        return Ok(o.values.clone());
    }
    if let Some(o) = config.option_typed::<ConfigOptionFloats>(key) {
        return Ok(o.values.clone());
    }
    Err(Error::Config(format!("calib: no floats option {key}")))
}

// ============================================================================
// Base Calibration Class
// ============================================================================

/// Base class for Pressure Advance calibration with common drawing primitives.
///
/// Calib.hpp:193-235
#[derive(Clone)]
pub struct CalibPressureAdvance {
    /// Last position for movement calculations
    /// Calib.hpp:213
    pub last_pos: (f64, f64, f64),

    /// Print configuration (`m_config`)
    /// Calib.hpp:233
    pub config: FullDynamicPrintConfig,

    /// Calib.hpp:230 `bool m_is_bbl_bowden = false;`
    pub is_bbl_bowden: bool,

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
    pub fn new(config: FullDynamicPrintConfig) -> Self {
        Self {
            last_pos: (0.0, 0.0, 0.0),
            config,
            is_bbl_bowden: false,
            encroachment: 1.0 / 3.0,
            draw_digit_mode: DrawDigitMode::LeftToRight,
            digit_segment_len: 2.0,
            digit_gap_len: 1.0,
            max_number_len: 5,
        }
    }

    /// Find optimal PA test speed based on volumetric limits
    /// Calib.hpp:198
    /// Calib.cpp:9-18
    pub fn find_optimal_pa_speed(
        config: &FullDynamicPrintConfig,
        line_width: f64,
        layer_height: f64,
        extruder_id: usize,
        filament_idx: usize,
    ) -> Result<f32> {
        // Calib.cpp:11
        let general_suggested_min_speed = 100.0;
        // Calib.cpp:12
        let filament_max_volumetric_speed = floats_nullable_at(config, "filament_max_volumetric_speed", filament_idx)?;
        // Calib.cpp:13 — Flow(line_width, layer_height, nozzle_diameter)
        let pattern_line = Flow::new(line_width, layer_height, floats_nullable_at(config, "nozzle_diameter", extruder_id)?)?;
        // Calib.cpp:14-15 — std::min(std::max(...), ...)
        let pa_speed = std_min(
            std_max(general_suggested_min_speed, floats_nullable_at(config, "outer_wall_speed", extruder_id)?),
            filament_max_volumetric_speed / pattern_line.mm3_per_mm()?,
        );
        // Calib.cpp:17
        Ok(pa_speed.floor() as f32)
    }

    /// Calib.hpp:200 `void set_bbl_bowden_mode() { m_is_bbl_bowden = true; }`
    pub fn set_bbl_bowden_mode(&mut self) {
        self.is_bbl_bowden = true;
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
        // Calib.cpp:35 — Flow(line_width, layer_height, nozzle_diameter); nozzle_diameter is float.
        let line_flow = Flow::new(line_width, layer_height, nozzle_diameter as f64)?;
        // Calib.cpp:36 — M_PI * std::pow(filament_diameter / 2, 2).
        // filament_diameter is `float`, so `filament_diameter / 2` is evaluated in float
        // precision (the int literal 2 promotes to float); std::pow then promotes to double.
        let filament_area =
            std::f64::consts::PI * ((filament_diameter / 2.0f32) as f64).powi(2);

        // Calib.cpp:38 — line_flow.mm3_per_mm() / filament_area * print_flow_ratio.
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
    /// Calib.hpp:212 — `double speed_adjust(int speed) const { return speed * 60; };`
    /// C++ computes `speed * 60` in `int`, then returns it promoted to `double`.
    pub fn speed_adjust(&self, speed: i32) -> f64 {
        (speed * 60) as f64
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

    /// Calib.cpp:20-31 `std::string CalibPressureAdvance::move_to(Vec2d pt, GCodeWriter &writer,
    /// std::string comment)`
    pub fn move_to(&mut self, pt: PointF, writer: &mut GCodeWriter, comment: &str) -> String {
        // Calib.cpp:22
        let mut gcode = String::new();

        // Calib.cpp:24-26
        gcode += &writer.retract(false);
        gcode += &writer.travel_to_xy(pt, comment);
        gcode += &writer.unretract(0.0);

        // Calib.cpp:28
        self.last_pos = (pt.x, pt.y, 0.0);

        // Calib.cpp:30
        gcode
    }

    /// Calib.cpp:50-185 `std::string CalibPressureAdvance::draw_digit(double startx, double starty,
    /// char c, CalibPressureAdvance::DrawDigitMode mode, double line_width, double e_per_mm,
    /// GCodeWriter &writer)`
    #[allow(clippy::too_many_arguments)]
    pub fn draw_digit(
        &mut self,
        startx: f64,
        starty: f64,
        c: u8,
        mode: DrawDigitMode,
        line_width: f64,
        e_per_mm: f64,
        writer: &mut GCodeWriter,
    ) -> String {
        // Calib.cpp:53-54
        let len = self.digit_segment_len;
        let gap = line_width / 2.0;

        // Calib.cpp:56-57
        let d_e = e_per_mm * len;
        let two_d_e = d_e * 2.0;

        // Calib.cpp:59-62
        let (p0, p1, p2, p3, p4, p5);
        let (p0_5, p4_5);
        let (gap_p0_toward_p3, gap_p2_toward_p3);
        let dot_direction;

        // Calib.cpp:64
        if mode == DrawDigitMode::BottomToTop {
            //  1-------2-------5
            //  |       |       |
            //  |       |       |
            //  0-------3-------4
            // Calib.cpp:69-76 (`startx + len * 2` fused)
            p0 = PointF::new(startx, starty);
            p0_5 = PointF::new(startx, starty + len / 2.0);
            p1 = PointF::new(startx, starty + len);
            p2 = PointF::new(startx + len, starty + len);
            p3 = PointF::new(startx + len, starty);
            p4 = PointF::new(len.mul_add(2.0, startx), starty);
            p4_5 = PointF::new(len.mul_add(2.0, startx), starty + len / 2.0);
            p5 = PointF::new(len.mul_add(2.0, startx), starty + len);

            // Calib.cpp:78-79
            gap_p0_toward_p3 = PointF::new(p0.x + gap, p0.y + 0.0);
            gap_p2_toward_p3 = PointF::new(p2.x + 0.0, p2.y + gap);

            // Calib.cpp:81
            dot_direction = PointF::new(-len / 2.0, 0.0);
        } else {
            //  0-------1
            //  |       |
            //  3-------2
            //  |       |
            //  4-------5
            // Calib.cpp:88-95 (`starty - len * 2` fused)
            p0 = PointF::new(startx, starty);
            p0_5 = PointF::new(startx + len / 2.0, starty);
            p1 = PointF::new(startx + len, starty);
            p2 = PointF::new(startx + len, starty - len);
            p3 = PointF::new(startx, starty - len);
            p4 = PointF::new(startx, (-len).mul_add(2.0, starty));
            p4_5 = PointF::new(startx + len / 2.0, (-len).mul_add(2.0, starty));
            p5 = PointF::new(startx + len, (-len).mul_add(2.0, starty));

            // Calib.cpp:97-98
            gap_p0_toward_p3 = PointF::new(p0.x - 0.0, p0.y - gap);
            gap_p2_toward_p3 = PointF::new(p2.x - gap, p2.y - 0.0);

            // Calib.cpp:100
            dot_direction = PointF::new(0.0, len / 2.0);
        }

        // Calib.cpp:103
        let mut gcode = String::new();

        // Calib.cpp:105-182
        match c {
            b'0' => {
                gcode += &self.move_to(p0, writer, "Glyph: 0");
                gcode += &writer.extrude_to_xy(p1, d_e, "", false);
                gcode += &writer.extrude_to_xy(p5, two_d_e, "", false);
                gcode += &writer.extrude_to_xy(p4, d_e, "", false);
                gcode += &writer.extrude_to_xy(gap_p0_toward_p3, two_d_e, "", false);
            }
            b'1' => {
                gcode += &self.move_to(p0_5, writer, "Glyph: 1");
                gcode += &writer.extrude_to_xy(p4_5, two_d_e, "", false);
            }
            b'2' => {
                gcode += &self.move_to(p0, writer, "Glyph: 2");
                gcode += &writer.extrude_to_xy(p1, d_e, "", false);
                gcode += &writer.extrude_to_xy(p2, d_e, "", false);
                gcode += &writer.extrude_to_xy(p3, d_e, "", false);
                gcode += &writer.extrude_to_xy(p4, d_e, "", false);
                gcode += &writer.extrude_to_xy(p5, d_e, "", false);
            }
            b'3' => {
                gcode += &self.move_to(p0, writer, "Glyph: 3");
                gcode += &writer.extrude_to_xy(p1, d_e, "", false);
                gcode += &writer.extrude_to_xy(p5, two_d_e, "", false);
                gcode += &writer.extrude_to_xy(p4, d_e, "", false);
                gcode += &self.move_to(gap_p2_toward_p3, writer, "");
                gcode += &writer.extrude_to_xy(p3, d_e, "", false);
            }
            b'4' => {
                gcode += &self.move_to(p0, writer, "Glyph: 4");
                gcode += &writer.extrude_to_xy(p3, d_e, "", false);
                gcode += &writer.extrude_to_xy(p2, d_e, "", false);
                gcode += &self.move_to(p1, writer, "");
                gcode += &writer.extrude_to_xy(p5, two_d_e, "", false);
            }
            b'5' => {
                gcode += &self.move_to(p1, writer, "Glyph: 5");
                gcode += &writer.extrude_to_xy(p0, d_e, "", false);
                gcode += &writer.extrude_to_xy(p3, d_e, "", false);
                gcode += &writer.extrude_to_xy(p2, d_e, "", false);
                gcode += &writer.extrude_to_xy(p5, d_e, "", false);
                gcode += &writer.extrude_to_xy(p4, d_e, "", false);
            }
            b'6' => {
                gcode += &self.move_to(p1, writer, "Glyph: 6");
                gcode += &writer.extrude_to_xy(p0, d_e, "", false);
                gcode += &writer.extrude_to_xy(p4, two_d_e, "", false);
                gcode += &writer.extrude_to_xy(p5, d_e, "", false);
                gcode += &writer.extrude_to_xy(p2, d_e, "", false);
                gcode += &writer.extrude_to_xy(p3, d_e, "", false);
            }
            b'7' => {
                gcode += &self.move_to(p0, writer, "Glyph: 7");
                gcode += &writer.extrude_to_xy(p1, d_e, "", false);
                gcode += &writer.extrude_to_xy(p5, two_d_e, "", false);
            }
            b'8' => {
                gcode += &self.move_to(p2, writer, "Glyph: 8");
                gcode += &writer.extrude_to_xy(p3, d_e, "", false);
                gcode += &writer.extrude_to_xy(p4, d_e, "", false);
                gcode += &writer.extrude_to_xy(p5, d_e, "", false);
                gcode += &writer.extrude_to_xy(p1, two_d_e, "", false);
                gcode += &writer.extrude_to_xy(p0, d_e, "", false);
                gcode += &writer.extrude_to_xy(p3, d_e, "", false);
            }
            b'9' => {
                gcode += &self.move_to(p5, writer, "Glyph: 9");
                gcode += &writer.extrude_to_xy(p1, two_d_e, "", false);
                gcode += &writer.extrude_to_xy(p0, d_e, "", false);
                gcode += &writer.extrude_to_xy(p3, d_e, "", false);
                gcode += &writer.extrude_to_xy(p2, d_e, "", false);
            }
            b'.' => {
                gcode += &self.move_to(p4_5, writer, "Glyph: .");
                gcode += &writer.extrude_to_xy(PointF::new(p4_5.x + dot_direction.x, p4_5.y + dot_direction.y), d_e, "", false);
            }
            _ => {}
        }

        // Calib.cpp:184
        gcode
    }

    /// Calib.cpp:187-203 `std::string CalibPressureAdvance::draw_number(double startx, double starty,
    /// double value, CalibPressureAdvance::DrawDigitMode mode, double line_width, double e_per_mm,
    /// double speed, GCodeWriter &writer)`
    #[allow(clippy::too_many_arguments)]
    pub fn draw_number(
        &mut self,
        startx: f64,
        starty: f64,
        value: f64,
        mode: DrawDigitMode,
        line_width: f64,
        e_per_mm: f64,
        speed: f64,
        writer: &mut GCodeWriter,
    ) -> String {
        // Calib.cpp:190-192
        let s_number = self.convert_number_to_string(value);
        let mut gcode = String::new();
        gcode += &writer.set_speed(speed, "", "");

        // Calib.cpp:194-200
        let bytes = s_number.as_bytes();
        for i in 0..bytes.len() {
            if i > self.max_number_len {
                break;
            }
            match mode {
                // `starty + i * number_spacing()` (the sum fused)
                DrawDigitMode::BottomToTop => {
                    gcode += &self.draw_digit(startx, (i as f64).mul_add(self.number_spacing(), starty), bytes[i], mode, line_width, e_per_mm, writer)
                }
                _ => gcode += &self.draw_digit((i as f64).mul_add(self.number_spacing(), startx), starty, bytes[i], mode, line_width, e_per_mm, writer),
            }
        }

        // Calib.cpp:202
        gcode
    }

    /// Calib.cpp:207-224 `std::string CalibPressureAdvance::draw_line(GCodeWriter &writer, Vec2d to_pt,
    /// double line_width, double layer_height, double speed, const std::string &comment)`
    pub fn draw_line(
        &mut self,
        writer: &mut GCodeWriter,
        to_pt: PointF,
        line_width: f64,
        layer_height: f64,
        speed: f64,
        comment: &str,
    ) -> Result<String> {
        // Calib.cpp:209-211 (the doubles of the options passed to the float parameters)
        let e_per_mm = self.e_per_mm(
            line_width,
            layer_height,
            floats_nullable_at(&self.config, "nozzle_diameter", 0)? as f32,
            floats_at(&self.config, "filament_diameter", 0)? as f32,
            floats_nullable_at(&self.config, "filament_flow_ratio", 0)? as f32,
        )?;

        // Calib.cpp:213-214
        let length = self.get_distance(PointF::new(self.last_pos.0, self.last_pos.1), to_pt);
        let d_e = e_per_mm * length;

        // Calib.cpp:216
        let mut gcode = String::new();

        // Calib.cpp:218-219
        gcode += &writer.set_speed(speed, "", "");
        gcode += &writer.extrude_to_xy(to_pt, d_e, comment, false);

        // Calib.cpp:221
        self.last_pos = (to_pt.x, to_pt.y, 0.0);

        // Calib.cpp:223
        Ok(gcode)
    }

    /// Calib.cpp:226-405 `std::string CalibPressureAdvance::draw_box(GCodeWriter &writer, double min_x,
    /// double min_y, double size_x, double size_y, DrawBoxOptArgs opt_args)`
    pub fn draw_box(
        &mut self,
        writer: &mut GCodeWriter,
        min_x: f64,
        min_y: f64,
        size_x: f64,
        size_y: f64,
        mut opt_args: DrawBoxOptArgs,
    ) -> Result<String> {
        // Calib.cpp:228
        let mut gcode = String::new();

        // Calib.cpp:230-233
        let mut x = min_x;
        let mut y = min_y;
        let max_x = min_x + size_x;
        let max_y = min_y + size_y;

        // Calib.cpp:235  `opt_args.line_width - opt_args.height * (1 - M_PI / 4)` (fused)
        let spacing = (-opt_args.height).mul_add(1.0 - std::f64::consts::PI / 4.0, opt_args.line_width);

        // Calib.cpp:237-241  if number of perims exceeds size of box, reduce it to max; this is the
        // equivalent of number of perims for concentric fill (std::min of the doubles, then the int)
        let max_perimeters = std_min(
            (size_x * self.to_radians(45.0).sin()).floor() / (spacing / self.to_radians(45.0).sin()),
            (size_y * self.to_radians(45.0).sin()).floor() / (spacing / self.to_radians(45.0).sin()),
        ) as i32;

        // Calib.cpp:243
        opt_args.num_perimeters = opt_args.num_perimeters.min(max_perimeters);

        // Calib.cpp:245
        gcode += &self.move_to(PointF::new(min_x, min_y), writer, "Move to box start");

        // Calib.cpp:247-251  DrawLineOptArgs line_opt_args(*this);
        let line_arg_height = opt_args.height;
        let line_arg_line_width = opt_args.line_width;
        let line_arg_speed = opt_args.speed;
        let mut comment: &str;

        // Calib.cpp:253
        for i in 0..opt_args.num_perimeters {
            // Calib.cpp:254-258  after first perimeter, step inwards to start next perimeter
            if i != 0 {
                x += spacing;
                y += spacing;
                gcode += &self.move_to(PointF::new(x, y), writer, "Step inwards to print next perimeter");
            }

            // Calib.cpp:260-262  `size_y - i * spacing * 2` (fused)
            y += (-(i as f64 * spacing)).mul_add(2.0, size_y);
            comment = "Draw perimeter (up)";
            gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;

            // Calib.cpp:264-266
            x += (-(i as f64 * spacing)).mul_add(2.0, size_x);
            comment = "Draw perimeter (right)";
            gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;

            // Calib.cpp:268-270
            y -= (-(i as f64 * spacing)).mul_add(2.0, size_y);
            comment = "Draw perimeter (down)";
            gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;

            // Calib.cpp:272-274
            x -= (-(i as f64 * spacing)).mul_add(2.0, size_x);
            comment = "Draw perimeter (left)";
            gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
        }

        // Calib.cpp:277
        if !opt_args.is_filled {
            return Ok(gcode);
        }

        // Calib.cpp:279-280  create box infill
        let spacing_45 = spacing / self.to_radians(45.0).sin();

        // Calib.cpp:282  `(spacing * (n - 1)) + (line_width * (1 - m_encroachment))` (the first
        // product fused)
        let bound_modifier =
            spacing.mul_add((opt_args.num_perimeters - 1) as f64, opt_args.line_width * (1.0 - self.encroachment));
        // Calib.cpp:283-288
        let x_min_bound = min_x + bound_modifier;
        let x_max_bound = max_x - bound_modifier;
        let y_min_bound = min_y + bound_modifier;
        let y_max_bound = max_y - bound_modifier;
        let x_count = ((x_max_bound - x_min_bound) / spacing_45).floor() as i32;
        let y_count = ((y_max_bound - y_min_bound) / spacing_45).floor() as i32;

        // Calib.cpp:290-291  (std::fmod is the remainder of the truncated quotient: Rust's `%`)
        let mut x_remainder = (x_max_bound - x_min_bound) % spacing_45;
        let mut y_remainder = (y_max_bound - y_min_bound) % spacing_45;

        // Calib.cpp:293-294
        x = x_min_bound;
        y = y_min_bound;

        // Calib.cpp:296
        gcode += &self.move_to(PointF::new(x, y), writer, "Move to fill start");

        // Calib.cpp:298-299  this isn't the most robust way, but less expensive than finding line
        // intersections (the bound is evaluated every iteration: the remainders change in the loop)
        let mut i: i32 = 0;
        while i < x_count + y_count + if x_remainder + y_remainder >= spacing_45 { 1 } else { 0 } {
            // Calib.cpp:300
            if i < x_count.min(y_count) {
                if i % 2 == 0 {
                    // Calib.cpp:302-309
                    x += spacing_45;
                    y = y_min_bound;
                    gcode += &self.move_to(PointF::new(x, y), writer, "Fill: Step right");

                    y += x - x_min_bound;
                    x = x_min_bound;
                    comment = "Fill: Print up/left";
                    gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
                } else {
                    // Calib.cpp:311-318
                    y += spacing_45;
                    x = x_min_bound;
                    gcode += &self.move_to(PointF::new(x, y), writer, "Fill: Step up");

                    x += y - y_min_bound;
                    y = y_min_bound;
                    comment = "Fill: Print down/right";
                    gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
                }
            } else if i < x_count.max(y_count) {
                // Calib.cpp:320
                if x_count > y_count {
                    // box is wider than tall
                    if i % 2 == 0 {
                        // Calib.cpp:323-330
                        x += spacing_45;
                        y = y_min_bound;
                        gcode += &self.move_to(PointF::new(x, y), writer, "Fill: Step right");

                        x -= y_max_bound - y_min_bound;
                        y = y_max_bound;
                        comment = "Fill: Print up/left";
                        gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
                    } else {
                        // Calib.cpp:332-345
                        if i == y_count {
                            x += spacing_45 - y_remainder;
                            y_remainder = 0.0;
                        } else {
                            x += spacing_45;
                        }
                        y = y_max_bound;
                        gcode += &self.move_to(PointF::new(x, y), writer, "Fill: Step right");

                        x += y_max_bound - y_min_bound;
                        y = y_min_bound;
                        comment = "Fill: Print down/right";
                        gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
                    }
                } else {
                    // box is taller than wide
                    if i % 2 == 0 {
                        // Calib.cpp:350-363
                        x = x_max_bound;
                        if i == x_count {
                            y += spacing_45 - x_remainder;
                            x_remainder = 0.0;
                        } else {
                            y += spacing_45;
                        }
                        gcode += &self.move_to(PointF::new(x, y), writer, "Fill: Step up");

                        x = x_min_bound;
                        y += x_max_bound - x_min_bound;
                        comment = "Fill: Print up/left";
                        gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
                    } else {
                        // Calib.cpp:365-372
                        x = x_min_bound;
                        y += spacing_45;
                        gcode += &self.move_to(PointF::new(x, y), writer, "Fill: Step up");

                        x = x_max_bound;
                        y -= x_max_bound - x_min_bound;
                        comment = "Fill: Print down/right";
                        gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
                    }
                }
            } else {
                // Calib.cpp:375
                if i % 2 == 0 {
                    // Calib.cpp:377-388
                    x = x_max_bound;
                    if i == x_count {
                        y += spacing_45 - x_remainder;
                    } else {
                        y += spacing_45;
                    }
                    gcode += &self.move_to(PointF::new(x, y), writer, "Fill: Step up");

                    x -= y_max_bound - y;
                    y = y_max_bound;
                    comment = "Fill: Print up/left";
                    gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
                } else {
                    // Calib.cpp:390-400
                    if i == y_count {
                        x += spacing_45 - y_remainder;
                    } else {
                        x += spacing_45;
                    }
                    y = y_max_bound;
                    gcode += &self.move_to(PointF::new(x, y), writer, "Fill: Step right");

                    y -= x_max_bound - x;
                    x = x_max_bound;
                    comment = "Fill: Print down/right";
                    gcode += &self.draw_line(writer, PointF::new(x, y), line_arg_line_width, line_arg_height, line_arg_speed, comment)?;
                }
            }
            i += 1;
        }

        // Calib.cpp:404
        Ok(gcode)
    }
}

// ============================================================================
// Derived Calibration Classes
// ============================================================================

/// Pressure Advance line test calibration.
///
/// Generates a series of horizontal lines with varying PA values and speeds
/// to visualize the effect of pressure advance settings, through the writer of the `GCode`
/// generator it is made for (`mp_gcodegen`).
///
/// Calib.hpp:237-274
pub struct CalibPressureAdvanceLine<'g, 'a> {
    /// Base calibration functionality
    pub base: CalibPressureAdvance,

    /// Calib.hpp:261 `GCode *mp_gcodegen;`
    mp_gcodegen: &'g mut crate::g_code::gcode_class::GCode<'a>,

    /// Nozzle diameter in mm
    /// Calib.hpp:263
    pub nozzle_diameter: f64,

    /// Slow speed for PA test (mm/s)
    /// Calib.hpp:264
    /// FIDELITY-NOTE: the C++ leaves m_slow_speed / m_fast_speed uninitialized until set_speed();
    /// they start at set_speed()'s defaults here.
    pub slow_speed: f64,

    /// Fast speed for PA test (mm/s)
    /// Calib.hpp:264
    pub fast_speed: f64,

    /// Layer height in mm
    /// Calib.hpp:266
    pub height_layer: f64,

    /// Line width in mm
    /// Calib.hpp:267
    pub line_width: f64,

    /// Thin line width in mm (for fine details)
    /// Calib.hpp:268
    pub thin_line_width: f64,

    /// Number line width in mm (for labels)
    /// Calib.hpp:269
    pub number_line_width: f64,

    /// Vertical spacing between lines
    /// Calib.hpp:270
    pub space_y: f64,

    /// Length of short line segments
    /// Calib.hpp:272
    pub length_short: f64,

    /// Length of long line segments
    /// Calib.hpp:272
    pub length_long: f64,

    /// Whether to draw value labels
    /// Calib.hpp:273
    pub draw_numbers: bool,
}

impl<'g, 'a> CalibPressureAdvanceLine<'g, 'a> {
    /// Calib.cpp:407-413 `CalibPressureAdvanceLine::CalibPressureAdvanceLine(GCode *gcodegen)`
    pub fn new(gcodegen: &'g mut crate::g_code::gcode_class::GCode<'a>) -> Result<Self> {
        // Calib.cpp:408  CalibPressureAdvance(gcodegen->config()) (`m_config.apply(config)` of the
        // full configuration: every option of it), m_nozzle_diameter(config().nozzle_diameter.get_at(0))
        let config = gcodegen.config().clone();
        let nozzle_diameter = floats_nullable_at(&config, "nozzle_diameter", 0)?;
        let initial_layer_print_height = config
            .option_typed::<ConfigOptionFloat>("initial_layer_print_height")
            .ok_or_else(|| Error::Config("calib: no initial_layer_print_height".to_string()))?
            .value;
        let mut this = Self {
            base: CalibPressureAdvance::new(config),
            mp_gcodegen: gcodegen,
            nozzle_diameter,
            slow_speed: 20.0,
            fast_speed: 100.0,
            // Calib.hpp:266-273  the member initializers
            height_layer: 0.2,
            line_width: 0.6,
            thin_line_width: 0.44,
            number_line_width: 0.48,
            space_y: 3.5,
            length_short: 20.0,
            length_long: 40.0,
            draw_numbers: true,
        };
        // Calib.cpp:410
        this.line_width = if this.nozzle_diameter < 0.51 { this.nozzle_diameter * 1.5 } else { this.nozzle_diameter * 1.05 };
        // Calib.cpp:411
        this.height_layer = initial_layer_print_height;
        // Calib.cpp:412
        this.thin_line_width = this.nozzle_diameter;
        this.number_line_width = this.thin_line_width;
        Ok(this)
    }

    /// Calib.cpp:415-431 `std::string CalibPressureAdvanceLine::generate_test(double start_pa /*= 0*/,
    /// double step_pa /*= 0.002*/, int count /*= 10*/)`
    pub fn generate_test(&mut self, start_pa: f64, step_pa: f64, mut count: i32) -> Result<String> {
        // Calib.cpp:417  BoundingBoxf bed_ext = get_extents(mp_gcodegen->config().printable_area.values);
        let printable_area = printable_area_of(self.mp_gcodegen.config())?;
        let mut bed_ext = BoundingBoxF::new();
        for p in &printable_area {
            bed_ext.merge_point(*p);
        }
        // Calib.cpp:418
        if self.is_delta() {
            self.base.delta_scale_bed_ext(&mut bed_ext);
        }

        // Calib.cpp:420-422  (BoundingBoxBase::size(): max - min)
        let w = bed_ext.max.x - bed_ext.min.x;
        let h = bed_ext.max.y - bed_ext.min.y;
        // Calib.cpp:423
        count = count.min(((h - 10.0) / self.space_y) as i32);

        // Calib.cpp:425  40 + std::min(w - 120.0, 0.0)
        self.length_long = 40.0 + std_min(w - 120.0, 0.0);

        // Calib.cpp:427  `w - m_length_short * 2 - ...` (the first difference fused)
        let startx = bed_ext.min.x + ((-self.length_short).mul_add(2.0, w) - self.length_long - 20.0) / 2.0;
        // Calib.cpp:428  `h - count * m_space_y` (fused)
        let starty = bed_ext.min.y + (-(count as f64)).mul_add(self.space_y, h) / 2.0;

        // Calib.cpp:430
        self.print_pa_lines(startx, starty, start_pa, step_pa, count)
    }

    /// Calib.hpp:250 `const double &line_width()`
    pub fn line_width(&self) -> f64 {
        self.line_width
    }

    /// Calib.hpp:251 `const double &height_layer()`
    pub fn height_layer(&self) -> f64 {
        self.height_layer
    }

    /// Calib.cpp:433 `bool CalibPressureAdvanceLine::is_delta() const { return
    /// mp_gcodegen->config().printable_area.values.size() > 4; }`
    pub fn is_delta(&self) -> bool {
        printable_area_of(self.mp_gcodegen.config()).map(|v| v.len()).unwrap_or(0) > 4
    }

    /// Calib.hpp:253 `bool &draw_numbers()`
    pub fn draw_numbers(&mut self) -> &mut bool {
        &mut self.draw_numbers
    }

    /// Set speed parameters
    /// Calib.hpp:244-248 `void set_speed(double fast = 100.0, double slow = 20.0)`
    pub fn set_speed(&mut self, fast: f64, slow: f64) {
        // Calib.hpp:246-247
        self.slow_speed = slow;
        self.fast_speed = fast;
    }

    /// Calib.cpp:435-490 `std::string CalibPressureAdvanceLine::print_pa_lines(double start_x, double
    /// start_y, double start_pa, double step_pa, int num)`
    fn print_pa_lines(&mut self, start_x: f64, start_y: f64, start_pa: f64, step_pa: f64, num: i32) -> Result<String> {
        // Calib.cpp:437-438  auto &writer = mp_gcodegen->writer(); const auto &config = mp_gcodegen->config();
        // Calib.cpp:440-441  filament_diameter.get_at(0); the ConfigOptionFloat print_flow_ratio
        let filament_diameter = floats_at(self.mp_gcodegen.config(), "filament_diameter", 0)?;
        let print_flow_ratio = self
            .mp_gcodegen
            .config()
            .option_typed::<ConfigOptionFloat>("print_flow_ratio")
            .ok_or_else(|| Error::Config("calib: no print_flow_ratio".to_string()))?
            .value;

        // Calib.cpp:443-445  (the doubles passed to the float parameters)
        let e_per_mm = self.base.e_per_mm(
            self.line_width,
            self.height_layer,
            self.nozzle_diameter as f32,
            filament_diameter as f32,
            print_flow_ratio as f32,
        )?;
        let _thin_e_per_mm = self.base.e_per_mm(
            self.thin_line_width,
            self.height_layer,
            self.nozzle_diameter as f32,
            filament_diameter as f32,
            print_flow_ratio as f32,
        )?;
        let number_e_per_mm = self.base.e_per_mm(
            self.number_line_width,
            self.height_layer,
            self.nozzle_diameter as f32,
            filament_diameter as f32,
            print_flow_ratio as f32,
        )?;

        // Calib.cpp:447-448  speed_adjust(int): the double speeds truncated
        let fast = self.base.speed_adjust(self.fast_speed as i32);
        let slow = self.base.speed_adjust(self.slow_speed as i32);
        // Calib.cpp:449-450
        let mut gcode = String::new();
        gcode += &self.mp_gcodegen.m_writer.travel_to_z(self.height_layer, "");
        let y_pos = start_y;

        // Calib.cpp:453-456  prime line
        let prime_x = start_x;
        let writer = &mut self.mp_gcodegen.m_writer;
        gcode += &self.base.move_to(PointF::new(prime_x, (num as f64).mul_add(self.space_y, y_pos)), writer, "");
        gcode += &writer.set_speed(slow, "", "");
        gcode += &writer.extrude_to_xy(PointF::new(prime_x, y_pos), e_per_mm * self.space_y * num as f64 * 1.2, "", false);

        // Calib.cpp:458-467  (`y_pos + i * m_space_y`, `start_pa + i * step_pa` fused)
        for i in 0..num {
            let fi = i as f64;
            gcode += &writer.set_pressure_advance(fi.mul_add(step_pa, start_pa), self.base.is_bbl_bowden);
            gcode += &self.base.move_to(PointF::new(start_x, fi.mul_add(self.space_y, y_pos)), writer, "");
            gcode += &writer.set_speed(slow, "", "");
            gcode += &writer.extrude_to_xy(
                PointF::new(start_x + self.length_short, fi.mul_add(self.space_y, y_pos)),
                e_per_mm * self.length_short,
                "",
                false,
            );
            gcode += &writer.set_speed(fast, "", "");
            gcode += &writer.extrude_to_xy(
                PointF::new(start_x + self.length_short + self.length_long, fi.mul_add(self.space_y, y_pos)),
                e_per_mm * self.length_long,
                "",
                false,
            );
            gcode += &writer.set_speed(slow, "", "");
            gcode += &writer.extrude_to_xy(
                PointF::new(start_x + self.length_short + self.length_long + self.length_short, fi.mul_add(self.space_y, y_pos)),
                e_per_mm * self.length_short,
                "",
                false,
            );
        }
        // Calib.cpp:468
        gcode += &writer.set_pressure_advance(if self.base.is_bbl_bowden { 0.4 } else { 0.0 }, self.base.is_bbl_bowden);

        // Calib.cpp:470
        if self.draw_numbers {
            // Calib.cpp:471-477  Orca: skip drawing indicator lines

            // Calib.cpp:479-481
            let box_start_x = start_x + self.length_short + self.length_long + self.length_short;
            let mut default_box_opt_args = DrawBoxOptArgs::new(2, self.height_layer, self.line_width, fast);
            default_box_opt_args.is_filled = true;
            // Calib.cpp:482
            gcode += &self.base.draw_box(
                writer,
                box_start_x,
                start_y - self.space_y,
                self.base.number_spacing() * 8.0,
                (num + 1) as f64 * self.space_y,
                default_box_opt_args,
            )?;
            // Calib.cpp:483
            gcode += &writer.travel_to_z(self.height_layer * 2.0, "");
            // Calib.cpp:484-487  (`y_pos + i * m_space_y`, `start_pa + i * step_pa` fused)
            let mut i = 0;
            while i < num {
                let fi = i as f64;
                gcode += &self.base.draw_number(
                    box_start_x + 3.0 + self.line_width,
                    fi.mul_add(self.space_y, y_pos) + self.space_y / 2.0,
                    fi.mul_add(step_pa, start_pa),
                    self.base.draw_digit_mode,
                    self.number_line_width,
                    number_e_per_mm,
                    3600.0,
                    writer,
                );
                i += 2;
            }
        }
        // Calib.cpp:489
        Ok(gcode)
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

/// `config.printable_area.values`: the points of the ConfigOptionPoints `printable_area`.
fn printable_area_of(config: &FullDynamicPrintConfig) -> Result<Vec<PointF>> {
    config
        .option_typed::<crate::config::ConfigOptionPoints>("printable_area")
        .map(|o| o.values.clone())
        .ok_or_else(|| Error::Config("calib: no printable_area".to_string()))
}


/// Pressure Advance pattern test calibration.
///
/// Generates a grid of test patterns with varying PA values, using
/// corner features to visualize the effect of pressure advance.
///
/// Calib.cpp:498-806.
///
/// Calib.hpp:302-374
pub struct CalibPressureAdvancePattern {
    /// Base calibration functionality
    pub base: CalibPressureAdvance,

    /// Calibration parameters (`m_params`)
    /// Calib.hpp:358
    pub params: CalibParams,

    /// Starting point offset (`m_starting_point`)
    /// Calib.hpp:361
    pub starting_point: (f64, f64, f64),

    /// Whether starting point is fixed
    /// Calib.hpp:362
    pub is_start_point_fixed: bool,

    /// Calib.hpp:364 `const double m_handle_xy_size{5};`
    pub handle_xy_size: f64,

    /// Calib.hpp:365 `const double m_handle_spacing{2};`
    pub handle_spacing: f64,

    /// Calib.hpp:366 `const int m_num_layers{4};`
    pub num_layers: i32,

    /// Calib.hpp:368 `const double m_wall_side_length{30.0};`
    pub wall_side_length: f64,

    /// Calib.hpp:369 `const int m_corner_angle{90};`
    pub corner_angle: i32,

    /// Calib.hpp:370 `const int m_pattern_spacing{2};`
    pub pattern_spacing: i32,

    /// Calib.hpp:372 `const double m_glyph_padding_horizontal{1};`
    pub glyph_padding_horizontal: f64,

    /// Calib.hpp:373 `const double m_glyph_padding_vertical{1};`
    pub glyph_padding_vertical: f64,

    /// Calib.hpp:360 `GCodeWriter m_writer;`
    pub writer: GCodeWriter,
}

impl CalibPressureAdvancePattern {
    /// The members with their initializers (Calib.hpp:358-373) and `m_draw_digit_mode =
    /// DrawDigitMode::Bottom_To_Top` (Calib.cpp:501), before `refresh_setup`.
    fn with_defaults(params: CalibParams, config: FullDynamicPrintConfig) -> Self {
        Self {
            base: CalibPressureAdvance { draw_digit_mode: DrawDigitMode::BottomToTop, ..CalibPressureAdvance::new(config) },
            params,
            starting_point: (0.0, 0.0, 0.0),
            is_start_point_fixed: false,
            handle_xy_size: 5.0,
            handle_spacing: 2.0,
            num_layers: 4,
            wall_side_length: 30.0,
            corner_angle: 90,
            pattern_spacing: 2,
            glyph_padding_horizontal: 1.0,
            glyph_padding_vertical: 1.0,
            writer: GCodeWriter::new(),
        }
    }

    /// Calib.hpp:305 / Calib.cpp:498-504
    /// `CalibPressureAdvancePattern(const Calib_Params &params, const DynamicPrintConfig &config,
    /// bool is_bbl_machine, Model &model, const Vec3d &origin)`
    pub fn new(
        params: CalibParams,
        config: &FullDynamicPrintConfig,
        is_bbl_machine: bool,
        model: &crate::model::Model,
        origin: &crate::geometry::geometry::Vec3d,
    ) -> Result<Self> {
        // Calib.cpp:499 — m_params(params), CalibPressureAdvance(config)
        let mut this = Self::with_defaults(params, config.clone());
        // Calib.cpp:503
        this.refresh_setup(config, is_bbl_machine, model, origin)?;
        Ok(this)
    }

    /// Calib.cpp:506-656 `void CalibPressureAdvancePattern::generate_custom_gcodes(const
    /// DynamicPrintConfig &config, bool is_bbl_machine, Model &model, const Vec3d &origin)`
    pub fn generate_custom_gcodes(
        &mut self,
        config: &FullDynamicPrintConfig,
        is_bbl_machine: bool,
        model: &mut crate::model::Model,
        origin: &crate::geometry::geometry::Vec3d,
    ) -> Result<()> {
        use crate::custom_g_code::{Info, Item, Mode, Type};
        // Calib.cpp:508-509
        if model.objects.is_empty() {
            return Ok(());
        }
        // Calib.cpp:510-511
        let mut gcode = String::new();
        gcode += "; start pressure advance pattern for layer\n";

        // Calib.cpp:513
        self.refresh_setup(config, is_bbl_machine, model, origin)?;

        // Calib.cpp:515-516
        self.writer.set_first_layer(true);
        self.writer.reset_last_acceleration();

        // Calib.cpp:518-520
        let sp = self.starting_point;
        gcode += &self.base.move_to(PointF::new(sp.0, sp.1), &mut self.writer, "Move to start XY position");
        gcode += &self.writer.travel_to_z(self.height_first_layer(), "Move to start Z position");
        gcode += &self.writer.set_pressure_advance(if self.base.is_bbl_bowden { 0.4 } else { self.params.start }, self.base.is_bbl_bowden);

        // Calib.cpp:522  (speed_adjust(int): the double speed truncated)
        let default_box_opt_args = DrawBoxOptArgs::new(
            self.wall_count(),
            self.height_first_layer(),
            self.line_width_first_layer(),
            self.base.speed_adjust(self.speed_first_layer() as i32),
        );

        // Calib.cpp:524-525  create anchoring frame
        let (print_size_x, frame_size_y) = (self.print_size_x(), self.frame_size_y());
        gcode += &self.base.draw_box(&mut self.writer, sp.0, sp.1, print_size_x, frame_size_y, default_box_opt_args)?;

        // Calib.cpp:527-532  create tab for numbers (`... + m_glyph_padding_vertical * 2` fused)
        let mut draw_box_opt_args = default_box_opt_args;
        draw_box_opt_args.is_filled = true;
        draw_box_opt_args.num_perimeters = self.wall_count();
        let tab_y = sp.1 + self.frame_size_y() + self.line_spacing_first_layer();
        let tab_size_x = self.glyph_tab_max_x() - sp.0;
        let tab_size_y = self.glyph_padding_vertical.mul_add(2.0, self.max_numbering_height() + self.line_spacing_first_layer());
        gcode += &self.base.draw_box(&mut self.writer, sp.0, tab_y, tab_size_x, tab_size_y, draw_box_opt_args)?;

        // Calib.cpp:534-535
        let mut gcode_items: Vec<Item> = Vec::new();
        let num_patterns = self.get_num_patterns(); // "cache" for use in loops

        // Calib.cpp:537-538  draw pressure advance pattern
        for i in 0..self.num_layers {
            // Calib.cpp:539-540  (`height_first_layer() + (i * height_layer())` fused)
            let layer_height = (i as f64).mul_add(self.height_layer(), self.height_first_layer());
            let zhop_height = layer_height + self.height_layer();

            // Calib.cpp:542
            if i > 0 {
                // Calib.cpp:543-548  FIDELITY-NOTE: `CustomGCode::Item item;` leaves `extruder`
                // uninitialized; 0 here.
                gcode += "; end pressure advance pattern for layer\n";
                let item = Item::new_with_extra(
                    ((i - 1) as f64).mul_add(self.height_layer(), self.height_first_layer()),
                    Type::Custom,
                    0,
                    String::new(),
                    gcode.clone(),
                );
                gcode_items.push(item);

                // Calib.cpp:550-551  reset for next layer contents
                gcode = String::new();
                gcode += "; start pressure advance pattern for layer\n";

                // Calib.cpp:553-556
                self.writer.reset_last_acceleration();
                self.writer.set_first_layer(false);
                gcode += &self.writer.travel_to_z(layer_height, "Move to layer height");
                gcode += &self.writer.reset_e(false);
            }

            // Calib.cpp:559-560  line numbering
            if i == 1 {
                gcode += &self.writer.set_pressure_advance(if self.base.is_bbl_bowden { 0.4 } else { self.params.start }, self.base.is_bbl_bowden);

                // Calib.cpp:562-564  (the doubles passed to the float parameters)
                let number_e_per_mm = self.base.e_per_mm(
                    self.line_width(),
                    self.height_layer(),
                    floats_nullable_at(&self.base.config, "nozzle_diameter", 0)? as f32,
                    floats_at(&self.base.config, "filament_diameter", 0)? as f32,
                    floats_nullable_at(&self.base.config, "filament_flow_ratio", 0)? as f32,
                )?;

                // Calib.cpp:566-570  glyph on every other line (`m_params.start + (j *
                // m_params.step)` fused)
                let mut j = 0;
                while j < num_patterns {
                    let start_x = self.glyph_start_x(j);
                    let start_y = sp.1 + self.frame_size_y() + self.glyph_padding_vertical + self.line_width();
                    let value = (j as f64).mul_add(self.params.step, self.params.start);
                    let (mode, line_width, speed) = (self.base.draw_digit_mode, self.line_width(), self.speed_first_layer());
                    gcode += &self.base.draw_number(start_x, start_y, value, mode, line_width, number_e_per_mm, speed, &mut self.writer);
                    j += 2;
                }
            }

            // Calib.cpp:573-574
            if i == 0 {
                gcode += &self.writer.set_pressure_advance(self.params.start, self.base.is_bbl_bowden);
            }

            // Calib.cpp:576-578
            let mut to_x = sp.0 + self.pattern_shift();
            let mut to_y = sp.1;
            let mut side_length = self.wall_side_length;

            // Calib.cpp:580-586  shrink first layer to fit inside frame
            if i == 0 {
                // `line_spacing_first_layer() * (wall_count() - 1) + (line_width_first_layer() * (1 -
                // m_encroachment))` (the first product fused)
                let inset = self
                    .line_spacing_first_layer()
                    .mul_add((self.wall_count() - 1) as f64, self.line_width_first_layer() * (1.0 - self.base.encroachment));
                let shrink = inset / (self.base.to_radians(self.corner_angle as f64) / 2.0).sin();
                side_length = self.wall_side_length - shrink;
                // `to_x += shrink * std::sin(...)` (fused)
                to_x = shrink.mul_add(
                    (self.base.to_radians(90.0) - self.base.to_radians(self.corner_angle as f64) / 2.0).sin(),
                    to_x,
                );
                to_y += self
                    .line_spacing_first_layer()
                    .mul_add((self.wall_count() - 1) as f64, self.line_width_first_layer() * (1.0 - self.base.encroachment));
            }

            // Calib.cpp:588-589
            let initial_x = to_x;
            let initial_y = to_y;

            // Calib.cpp:591-593
            gcode += &self.writer.travel_to_z(zhop_height, "z-hop before move");
            gcode += &self.base.move_to(PointF::new(to_x, to_y), &mut self.writer, "Move to pattern start");
            gcode += &self.writer.travel_to_z(layer_height, "undo z-hop");

            // Calib.cpp:595
            for j in 0..num_patterns {
                // Calib.cpp:596-597  increment pressure advance
                gcode += &self.writer.set_pressure_advance((j as f64).mul_add(self.params.step, self.params.start), self.base.is_bbl_bowden);

                // Calib.cpp:599
                for k in 0..self.wall_count() {
                    // Calib.cpp:600-601  (`to_x += cos(...) * side_length` fused)
                    let half_angle = self.base.to_radians(self.corner_angle as f64) / 2.0;
                    to_x = half_angle.cos().mul_add(side_length, to_x);
                    to_y = half_angle.sin().mul_add(side_length, to_y);

                    // Calib.cpp:603-606  don't use line_width_first_layer so results are
                    // consistent across all layers
                    let draw_line_arg_height = if i == 0 { self.height_first_layer() } else { self.height_layer() };
                    let draw_line_arg_line_width = self.line_width();
                    let draw_line_arg_speed = if i == 0 {
                        self.base.speed_adjust(self.speed_first_layer() as i32)
                    } else {
                        self.base.speed_adjust(self.speed_perimeter() as i32)
                    };
                    let draw_line_arg_comment = "Print pattern wall";
                    // Calib.cpp:607
                    gcode += &self.base.draw_line(
                        &mut self.writer,
                        PointF::new(to_x, to_y),
                        draw_line_arg_line_width,
                        draw_line_arg_height,
                        draw_line_arg_speed,
                        draw_line_arg_comment,
                    )?;

                    // Calib.cpp:609-610  (`to_x -= cos(...) * side_length` fused)
                    to_x = (-half_angle.cos()).mul_add(side_length, to_x);
                    to_y = half_angle.sin().mul_add(side_length, to_y);

                    // Calib.cpp:612
                    gcode += &self.base.draw_line(
                        &mut self.writer,
                        PointF::new(to_x, to_y),
                        draw_line_arg_line_width,
                        draw_line_arg_height,
                        draw_line_arg_speed,
                        draw_line_arg_comment,
                    )?;

                    // Calib.cpp:614
                    to_y = initial_y;
                    if k != self.wall_count() - 1 {
                        // Calib.cpp:615-620  perimeters not done yet. move to next perimeter
                        to_x += self.line_spacing_angle();
                        gcode += &self.writer.travel_to_z(zhop_height, "z-hop before move");
                        gcode += &self.base.move_to(PointF::new(to_x, to_y), &mut self.writer, "Move to start next pattern wall");
                        gcode += &self.writer.travel_to_z(layer_height, "undo z-hop");
                    } else if j != num_patterns - 1 {
                        // Calib.cpp:621-626  patterns not done yet. move to next pattern
                        to_x += self.pattern_spacing as f64 + self.line_width();
                        gcode += &self.writer.travel_to_z(zhop_height, "z-hop before move");
                        gcode += &self.base.move_to(PointF::new(to_x, to_y), &mut self.writer, "Move to next pattern");
                        gcode += &self.writer.travel_to_z(layer_height, "undo z-hop");
                    } else if i != self.num_layers - 1 {
                        // Calib.cpp:627-633  layers not done yet. move back to start
                        to_x = initial_x;
                        gcode += &self.writer.travel_to_z(zhop_height, "z-hop before move");
                        gcode += &self.base.move_to(PointF::new(to_x, to_y), &mut self.writer, "Move back to start position");
                        gcode += &self.writer.travel_to_z(layer_height, "undo z-hop");
                        gcode += &self.writer.reset_e(false); // reset extruder before printing placeholder cube to avoid
                    } else {
                        // everything done
                    }
                }
            }
        }

        // Calib.cpp:640-641
        gcode += &self.writer.set_pressure_advance(if self.base.is_bbl_bowden { 0.4 } else { self.params.start }, self.base.is_bbl_bowden);
        gcode += "; end pressure advance pattern for layer\n";

        // Calib.cpp:643-647  (`extruder` left 0, see above)
        let item = Item::new_with_extra(self.max_layer_z(), Type::Custom, 0, String::new(), gcode);
        gcode_items.push(item);

        // Calib.cpp:649-651
        let mut info = Info::new();
        info.mode = Mode::SingleExtruder;
        info.gcodes = gcode_items;

        // Calib.cpp:653
        let plate = model.curr_plate_index;
        model.plates_custom_gcodes.insert(plate, info);
        Ok(())
    }

    /// Set starting position offset
    /// Calib.hpp:317
    /// Calib.cpp:658-662
    pub fn set_start_offset(&mut self, offset: (f64, f64, f64)) {
        self.starting_point = offset;
        self.is_start_point_fixed = true;
    }

    /// Get starting position offset
    /// Calib.hpp:318
    /// Calib.cpp:664
    pub fn get_start_offset(&self) -> (f64, f64, f64) {
        self.starting_point
    }

    /// Calib.hpp:308 `double handle_xy_size() const`
    pub fn handle_xy_size(&self) -> f64 {
        self.handle_xy_size
    }

    /// Calib.hpp:309 `double handle_spacing() const`
    pub fn handle_spacing(&self) -> f64 {
        self.handle_spacing
    }

    /// Calib.hpp:310 `double print_size_x() const { return object_size_x() + pattern_shift(); }`
    pub fn print_size_x(&self) -> f64 {
        self.object_size_x() + self.pattern_shift()
    }

    /// Calib.hpp:312 `double print_size_y() const { return object_size_y(); }`
    pub fn print_size_y(&self) -> f64 {
        self.object_size_y()
    }

    /// Calib.hpp:313 `double max_layer_z() const { return height_first_layer() + ((m_num_layers - 1) *
    /// height_layer()); }` (the sum fused)
    pub fn max_layer_z(&self) -> f64 {
        ((self.num_layers - 1) as f64).mul_add(self.height_layer(), self.height_first_layer())
    }

    /// Calib.hpp:321 `m_config.option<ConfigOptionFloatsNullable>("initial_layer_speed")->get_at(m_params.extruder_id)`
    pub fn speed_first_layer(&self) -> f64 {
        floats_nullable_at(&self.base.config, "initial_layer_speed", self.params.extruder_id).expect("initial_layer_speed")
    }

    /// Calib.hpp:322 `m_config.option<ConfigOptionFloatsNullable>("outer_wall_speed")->get_at(m_params.extruder_id)`
    pub fn speed_perimeter(&self) -> f64 {
        floats_nullable_at(&self.base.config, "outer_wall_speed", self.params.extruder_id).expect("outer_wall_speed")
    }

    /// Calib.hpp:323 `m_config.get_abs_value("initial_layer_line_width")`
    pub fn line_width_first_layer(&self) -> f64 {
        self.base.config.get_abs_value("initial_layer_line_width").expect("initial_layer_line_width")
    }

    /// Calib.hpp:324 `m_config.get_abs_value("line_width")`
    pub fn line_width(&self) -> f64 {
        self.base.config.get_abs_value("line_width").expect("line_width")
    }

    /// Calib.hpp:325 `m_config.option<ConfigOptionInt>("wall_loops")->value`
    pub fn wall_count(&self) -> i32 {
        self.base.config.option_typed::<ConfigOptionInt>("wall_loops").expect("wall_loops").value
    }

    /// Calib.cpp:666-674 `void refresh_setup(const DynamicPrintConfig &config, bool is_bbl_machine,
    /// const Model &model, const Vec3d &origin)`
    fn refresh_setup(
        &mut self,
        config: &FullDynamicPrintConfig,
        is_bbl_machine: bool,
        model: &crate::model::Model,
        origin: &crate::geometry::geometry::Vec3d,
    ) -> Result<()> {
        // Calib.cpp:668-670
        self.base.config = config.clone();
        let object = &model.objects[0];
        self.base.config.apply(object.config.get(), true).map_err(|e| Error::Config(format!("{e:?}")))?;
        self.base.config.apply(object.volumes[0].config.get(), true).map_err(|e| Error::Config(format!("{e:?}")))?;
        // Calib.cpp:672
        self._refresh_starting_point(model);
        // Calib.cpp:673
        self._refresh_writer(is_bbl_machine, model, origin)?;
        Ok(())
    }

    /// Calib.cpp:676-685 `void _refresh_starting_point(const Model &model)`
    fn _refresh_starting_point(&mut self, model: &crate::model::Model) {
        // Calib.cpp:678
        if self.is_start_point_fixed {
            return;
        }
        // Calib.cpp:680-681
        let obj = &model.objects[0];
        let bbox = obj.instance_bounding_box_of(&obj.instances[0], false);
        // Calib.cpp:683-684
        self.starting_point = (bbox.min.x, bbox.max.y, 0.0);
        self.starting_point.1 += self.handle_spacing;
    }

    /// Calib.cpp:687-732 `void _refresh_writer(bool is_bbl_machine, const Model &model, const Vec3d
    /// &origin)`
    fn _refresh_writer(&mut self, _is_bbl_machine: bool, model: &crate::model::Model, origin: &crate::geometry::geometry::Vec3d) -> Result<()> {
        // Calib.cpp:689-695  Build a full config with all defaults first, then override with
        // m_config. A bare PrintConfig() has empty vectors for many options (filament_volume_map,
        // filament_self_index, etc.), which causes crashes in GCodeWriter if those keys are absent
        // from m_config.
        let mut full_defaults = FullDynamicPrintConfig::full_print_config();
        full_defaults.apply(&self.base.config, true).map_err(|e| Error::Config(format!("{e:?}")))?;

        // Calib.cpp:697-700  PrintConfig print_config; print_config.apply(full_defaults, true);
        // m_writer.apply_print_config(print_config);
        // FIDELITY-NOTE: the writer takes the full configuration, whose PrintConfig members are those
        // `print_config.apply(full_defaults, true)` copies.
        let print_config = full_defaults;
        self.writer.apply_print_config(&print_config);
        // Calib.cpp:701
        self.writer.set_xy_offset(origin.x, origin.y);
        // Calib.cpp:702  //m_writer.set_is_bbl_machine(is_bbl_machine);

        // Calib.cpp:704-707
        let object = &model.objects[0];
        let model_extruder_id = object.volumes[0].extruder_id(object);
        let filament_id: u32 = if model_extruder_id > 0 { (model_extruder_id - 1) as u32 } else { 0 };
        self.writer.set_extruders(vec![filament_id]);
        self.writer.set_extruder(filament_id, 0);

        // Calib.cpp:709-720  `(unsigned int) floor(value + 0.5)`
        let to_uints = |key: &str| -> Result<Vec<u32>> {
            Ok(floats_values(&print_config, key)?.iter().map(|&value| (value + 0.5).floor() as u32).collect())
        };
        let travel_accelerations = to_uints("travel_acceleration")?;
        let short_travel_accelerations = to_uints("travel_short_distance_acceleration")?;
        let first_layer_travel_accelerations = to_uints("initial_layer_travel_acceleration")?;
        // Calib.cpp:721-723
        self.writer.set_travel_acceleration(&travel_accelerations);
        self.writer.set_travel_short_acceleration(&short_travel_accelerations);
        self.writer.set_first_layer_travel_acceleration(&first_layer_travel_accelerations);

        // Calib.cpp:725-731  (get_process_config_idx of the PrintConfig: of its GCodeConfig members,
        // which the writer's config holds)
        let nozzle_idx = crate::extruder::get_process_config_idx(&self.writer.config, filament_id);
        let default_acceleration = floats_values(&print_config, "default_acceleration")?;
        let outer_wall_acceleration = floats_values(&print_config, "outer_wall_acceleration")?;
        if nozzle_idx < default_acceleration.len() && nozzle_idx < outer_wall_acceleration.len() {
            let default_acc = default_acceleration[nozzle_idx];
            let outer_acc = outer_wall_acceleration[nozzle_idx];
            if default_acc > 0.0 && outer_acc > 0.0 {
                self.writer.set_acceleration((outer_acc + 0.5).floor() as u32);
            }
        }
        Ok(())
    }

    /// Calib.hpp:332 `m_config.option<ConfigOptionFloat>("initial_layer_print_height")->value`
    pub fn height_first_layer(&self) -> f64 {
        self.base.config.option_typed::<ConfigOptionFloat>("initial_layer_print_height").expect("initial_layer_print_height").value
    }

    /// Calib.hpp:333 `m_config.option<ConfigOptionFloat>("layer_height")->value`
    pub fn height_layer(&self) -> f64 {
        self.base.config.option_typed::<ConfigOptionFloat>("layer_height").expect("layer_height").value
    }

    /// Calib.hpp:334 `const int get_num_patterns() const { return std::ceil((m_params.end -
    /// m_params.start) / m_params.step + 1); }`
    pub fn get_num_patterns(&self) -> i32 {
        ((self.params.end - self.params.start) / self.params.step + 1.0).ceil() as i32
    }

    /// Calib.hpp:342 `line_width() - height_layer() * (1 - M_PI / 4)` (the difference fused)
    pub fn line_spacing(&self) -> f64 {
        (-self.height_layer()).mul_add(1.0 - std::f64::consts::PI / 4.0, self.line_width())
    }

    /// Calib.hpp:343 `line_width_first_layer() - height_first_layer() * (1 - M_PI / 4)` (fused)
    pub fn line_spacing_first_layer(&self) -> f64 {
        (-self.height_first_layer()).mul_add(1.0 - std::f64::consts::PI / 4.0, self.line_width_first_layer())
    }

    /// Calib.hpp:344 `line_spacing() / std::sin(to_radians(m_corner_angle) / 2)`
    pub fn line_spacing_angle(&self) -> f64 {
        self.line_spacing() / (self.base.to_radians(self.corner_angle as f64) / 2.0).sin()
    }

    /// Calib.cpp:734-738 `double object_size_x() const` (the sums fused left to right)
    pub fn object_size_x(&self) -> f64 {
        let np = self.get_num_patterns();
        let wc = self.wall_count();
        let p1 = (wc - 1) as f64 * self.line_spacing_angle();
        let p2 = (np - 1) as f64 * (self.pattern_spacing as f64 + self.line_width());
        let s1 = (np as f64).mul_add(p1, p2);
        let s2 = (self.base.to_radians(self.corner_angle as f64) / 2.0).cos().mul_add(self.wall_side_length, s1);
        self.line_spacing_first_layer().mul_add(wc as f64, s2)
    }

    /// Calib.cpp:740-743 `double object_size_y() const` (the sums fused left to right)
    pub fn object_size_y(&self) -> f64 {
        let x = (self.base.to_radians(self.corner_angle as f64) / 2.0).sin() * self.wall_side_length;
        let s1 = 2.0f64.mul_add(x, self.max_numbering_height());
        let s2 = self.glyph_padding_vertical.mul_add(2.0, s1);
        s2 + self.line_width_first_layer()
    }

    /// Calib.hpp:348 `std::sin(to_radians(double(m_corner_angle) / 2)) * m_wall_side_length * 2`
    pub fn frame_size_y(&self) -> f64 {
        self.base.to_radians(self.corner_angle as f64 / 2.0).sin() * self.wall_side_length * 2.0
    }

    /// Calib.cpp:745-768 `double glyph_start_x(int pattern_i = 0) const`
    pub fn glyph_start_x(&self, pattern_i: i32) -> f64 {
        let wc = self.wall_count();
        let lsa = self.line_spacing_angle();
        // Calib.cpp:749-758 — the int products `pattern_i * (wall_count() - 1)` and
        // `pattern_i * m_pattern_spacing`; the sums with a double product fused
        let t1 = self.starting_point.0 + self.pattern_shift();
        let t2 = ((pattern_i * (wc - 1)) as f64).mul_add(lsa, t1);
        let t3 = (pattern_i as f64).mul_add(self.line_width(), t2);
        let mut x = t3 + (pattern_i * self.pattern_spacing) as f64;
        // Calib.cpp:761
        x += wc as f64 * lsa / 2.0;
        // Calib.cpp:765
        x -= self.glyph_length_x() / 2.0;
        x
    }

    /// Calib.cpp:770-774 `double glyph_length_x() const { return line_width() + (2 * m_digit_segment_len); }`
    pub fn glyph_length_x(&self) -> f64 {
        2.0f64.mul_add(self.base.digit_segment_len, self.line_width())
    }

    /// Calib.cpp:776-787 `double glyph_tab_max_x() const`
    pub fn glyph_tab_max_x(&self) -> f64 {
        // Calib.cpp:779-780
        let num = self.get_num_patterns();
        let max_num = if num % 2 == 0 { num - 1 } else { num };
        // Calib.cpp:783
        let padding = self.glyph_start_x(0) - self.starting_point.0;
        // Calib.cpp:785-786
        self.glyph_start_x(max_num - 1) + (self.glyph_length_x() - self.line_width() / 2.0) + padding
    }

    /// Calib.cpp:789-804 `double max_numbering_height() const`
    pub fn max_numbering_height(&self) -> f64 {
        // Calib.cpp:791-792
        let mut most_characters: u64 = 0;
        let num_patterns = self.get_num_patterns();
        // Calib.cpp:795-799 — `m_params.start + (i * m_params.step)` (the sum fused)
        let mut i: u64 = 0;
        while (i as i64) < num_patterns as i64 {
            let s_number = self.base.convert_number_to_string((i as f64).mul_add(self.params.step, self.params.start));
            if (s_number.len() as u64) > most_characters {
                most_characters = s_number.len() as u64;
            }
            i += 2;
        }
        // Calib.cpp:801
        most_characters = most_characters.min(self.base.max_number_len as u64);
        // Calib.cpp:803 — size_t arithmetic, the sum fused
        (most_characters as f64).mul_add(self.base.digit_segment_len, most_characters.wrapping_sub(1) as f64 * self.base.digit_gap_len)
    }

    /// Calib.cpp:806 `(wall_count() - 1) * line_spacing_first_layer() + line_width_first_layer() +
    /// m_glyph_padding_horizontal` (the first sum fused)
    pub fn pattern_shift(&self) -> f64 {
        ((self.wall_count() - 1) as f64).mul_add(self.line_spacing_first_layer(), self.line_width_first_layer()) + self.glyph_padding_horizontal
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
        let calib = CalibPressureAdvance::new(FullDynamicPrintConfig::default());
        let radians = calib.to_radians(180.0);
        assert!((radians - std::f64::consts::PI).abs() < 1e-10);
    }

    #[test]
    fn test_calib_pressure_advance_get_distance() {
        let calib = CalibPressureAdvance::new(FullDynamicPrintConfig::default());
        let p1 = PointF::new(0.0, 0.0);
        let p2 = PointF::new(3.0, 4.0);
        let distance = calib.get_distance(p1, p2);
        assert!((distance - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_calib_pressure_advance_speed_adjust() {
        let calib = CalibPressureAdvance::new(FullDynamicPrintConfig::default());
        assert_eq!(calib.speed_adjust(50), 3000.0); // 50 mm/s * 60 = 3000 mm/min
    }

    #[test]
    fn test_calib_pressure_advance_number_spacing() {
        let calib = CalibPressureAdvance::new(FullDynamicPrintConfig::default());
        assert_eq!(calib.number_spacing(), 3.0); // 2.0 + 1.0
    }

    /// A GCode generator on the full default configuration, as `CalibPressureAdvanceLine` takes
    /// one (Calib.cpp:407).
    fn default_gcodegen<'a>() -> crate::g_code::gcode_class::GCode<'a> {
        let config = FullDynamicPrintConfig::full_print_config();
        let mut g = crate::g_code::gcode_class::GCode::new();
        g.m_config = config.clone();
        g.m_writer.apply_print_config(&config);
        g.m_writer.set_extruders(vec![0]);
        g.m_writer.set_extruder(0, 0);
        g
    }

    #[test]
    fn test_calib_pressure_advance_line_new() {
        let mut g = default_gcodegen();
        let line = CalibPressureAdvanceLine::new(&mut g).unwrap();
        // Calib.cpp:410-411: the line width from the nozzle diameter (0.4 by default), the layer
        // height from initial_layer_print_height (0.2 by default).
        assert_eq!(line.line_width, 0.4 * 1.5);
        assert_eq!(line.height_layer, 0.2);
        assert!(line.draw_numbers);
    }

    #[test]
    fn test_calib_pressure_advance_line_set_speed() {
        let mut g = default_gcodegen();
        let mut line = CalibPressureAdvanceLine::new(&mut g).unwrap();
        line.set_speed(120.0, 30.0);
        assert_eq!(line.fast_speed, 120.0);
        assert_eq!(line.slow_speed, 30.0);
    }

    #[test]
    fn test_calib_pressure_advance_pattern_new() {
        let params = CalibParams::default();
        let pattern = CalibPressureAdvancePattern::with_defaults(params, FullDynamicPrintConfig::default());
        assert_eq!(pattern.num_layers, 4);
        assert_eq!(pattern.wall_side_length, 30.0);
        assert_eq!(pattern.corner_angle, 90);
    }

    #[test]
    fn test_calib_pressure_advance_pattern_offset() {
        let params = CalibParams::default();
        let mut pattern = CalibPressureAdvancePattern::with_defaults(params, FullDynamicPrintConfig::default());
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
        let calib = CalibPressureAdvance::new(FullDynamicPrintConfig::default());
        assert_eq!(calib.convert_number_to_string(0.02), "0.02");
        assert_eq!(calib.convert_number_to_string(0.0), "0");
        assert_eq!(calib.convert_number_to_string(1.0), "1");
        assert_eq!(calib.convert_number_to_string(0.125), "0.125");
        assert_eq!(calib.convert_number_to_string(10.5), "10.5");
    }

    #[test]
    fn test_e_per_mm() {
        // Calib.cpp:33-39 — mm3_per_mm / filament_area * print_flow_ratio.
        let calib = CalibPressureAdvance::new(FullDynamicPrintConfig::default());
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
        let mut g = default_gcodegen();
        let line = CalibPressureAdvanceLine::new(&mut g).unwrap();
        let mut startx = 12.0;
        let mut starty = 99.0;
        line.delta_modify_start(&mut startx, &mut starty, 10);
        assert_eq!(startx, -12.0);
        assert_eq!(starty, -(10.0 * line.space_y) / 2.0);
    }

    #[test]
    fn test_delta_scale_bed_ext() {
        // Calib.hpp:200 — bed_ext.scale(1.0f / 1.41421f).
        let calib = CalibPressureAdvance::new(FullDynamicPrintConfig::default());
        let mut bbox =
            BoundingBoxF::from_points_minmax(PointF::new(0.0, 0.0), PointF::new(200.0, 200.0));
        calib.delta_scale_bed_ext(&mut bbox);
        let scale = (1.0f32 / 1.41421f32) as f64;
        assert!((bbox.max.x - 200.0 * scale).abs() < 1e-9);
        assert!((bbox.max.y - 200.0 * scale).abs() < 1e-9);
    }
}
