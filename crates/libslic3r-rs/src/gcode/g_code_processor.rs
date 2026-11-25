//! Faithful 1:1 port of `GCode/GCodeProcessor.cpp` / `.hpp` (BambuStudio).
//!
//! This module mirrors the C++ subdir layout (`GCode/GCodeProcessor.cpp` ->
//! `gcode/g_code_processor.rs`). The full `GCodeProcessor` state machine in
//! C++ is ~6800 lines and is deeply coupled to `PrintConfig`/`DynamicPrintConfig`,
//! `MachineEnvelopeConfig`, `MultiNozzleUtils`, file I/O, regex producer
//! detection and threading. This file ports the self-contained, byte-exact
//! critical pieces that drive G-code time / feedrate / filament-usage parity:
//!
//!  * the firmware-kinematics math (`estimated_acceleration_distance`,
//!    `intersection_distance`, `speed_from_distance`, `max_allowable_speed`,
//!    `acceleration_time_from_distance`),
//!  * the trapezoid / time-block planner (`Trapezoid`, `TimeBlock`, `TimeMachine`,
//!    `planner_forward_pass_kernel`, `planner_reverse_pass_kernel`,
//!    `recalculate_trapezoids`),
//!  * the static reserved/custom tag tables and reserved-tag scanners,
//!  * the static gcode queries (`get_gcode_last_filament`, `get_last_z_from_gcode`,
//!    `get_last_position_from_gcode`),
//!  * the comment parse helpers (`get_object_label_id`, `get_z_height`),
//!  * the `UsedFilaments` cache structures and pure cache methods,
//!  * the `CommandProcessor` command trie,
//!  * the result/statistics data structures and their `reset()`.
//!
//! Blocked symbols (require not-yet-threaded config or unported deps; see the
//! port report) are listed at the bottom and NOT stubbed here.
//!
//! Conventions: `coord_t` -> i64, `coordf_t` -> f64 (none here; this file is all
//! `float`/`f32` matching C++). Line refs use `// GCodeProcessor.cpp:NNN`.

#![allow(dead_code)]
#![allow(clippy::needless_range_loop)]

use std::collections::BTreeMap;
use std::collections::HashMap;

use crate::custom_g_code::Type as CustomGCodeType;
use crate::extrusion_entity::ExtrusionRole;
use crate::g_code_reader::GCodeReader;

/// `static_cast<size_t>(ExtrusionRole::erCount)` — number of ExtrusionRole
/// values (ExtrusionEntity.hpp:45-70, last is `erCount`). The Rust
/// `ExtrusionRole` enum mirrors C++ None..Mixed (22 variants); `erCount` == 22.
const EXTRUSION_ROLE_COUNT: usize = 22;

/// `ExtrusionRole` does not derive `Ord`/`Hash` in this crate, but C++ keys
/// `std::map<ExtrusionRole, ...>` by the underlying enum value. We key our maps
/// by the `u8` discriminant to preserve the C++ ordering/identity semantics.
type RoleKey = u8;

#[inline]
fn role_key(role: ExtrusionRole) -> RoleKey {
    role as u8
}

// GCodeProcessor.cpp:27
const DEFAULT_TOOLPATH_WIDTH: f32 = 0.4;
/// PrintConfig.hpp:795-796 `AUX_FAN_STANDARD_GCODE_INDEX`, `AUX_FAN_OPTIONAL_GCODE_INDEX`
const AUX_FAN_STANDARD_GCODE_INDEX: i32 = 2;
const AUX_FAN_OPTIONAL_GCODE_INDEX: i32 = 10;
// GCodeProcessor.cpp:28
const DEFAULT_TOOLPATH_HEIGHT: f32 = 0.2;

// GCodeProcessor.cpp:30
const INCHES_TO_MM: f32 = 25.4;
// GCodeProcessor.cpp:31
const MMMIN_TO_MMSEC: f32 = 1.0 / 60.0;
// GCodeProcessor.cpp:32  0.0125mm tolerance for drawing arc
const DRAW_ARC_TOLERANCE: f32 = 0.0125;

// GCodeProcessor.cpp:34  Prusa Firmware 1_75mm_MK2
const DEFAULT_ACCELERATION: f32 = 1500.0;
// GCodeProcessor.cpp:35  Prusa Firmware 1_75mm_MK2
const DEFAULT_RETRACT_ACCELERATION: f32 = 1500.0;
// GCodeProcessor.cpp:36
const DEFAULT_TRAVEL_ACCELERATION: f32 = 1250.0;

// GCodeProcessor.cpp:38
const MIN_EXTRUDERS_COUNT: usize = 5;
// GCodeProcessor.cpp:39
const DEFAULT_FILAMENT_DIAMETER: f32 = 1.75;
// GCodeProcessor.cpp:40
const DEFAULT_FILAMENT_HRC: i32 = 0;
// GCodeProcessor.cpp:41
const DEFAULT_FILAMENT_DENSITY: f32 = 1.245;
// GCodeProcessor.cpp:42
const DEFAULT_FILAMENT_COST: f32 = 29.99;
// GCodeProcessor.cpp:43
const DEFAULT_FILAMENT_VITRIFICATION_TEMPERATURE: i32 = 0;

/// `sqr(x)` from `libslic3r.h`: `x * x`.
#[inline]
fn sqr(x: f32) -> f32 {
    x * x
}

/// PI used by `process_role_cache` (`PI` from `libslic3r.h`).
const PI: f64 = std::f64::consts::PI;

// ===========================================================================
// GCodeProcessor.hpp:31-45  enum class EMoveType : unsigned char
// ===========================================================================
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EMoveType {
    Noop,
    Retract,
    Unretract,
    Seam,
    ToolChange,  // Tool_change
    ColorChange, // Color_change
    PausePrint,  // Pause_Print
    CustomGCode, // Custom_GCode
    Travel,
    Wipe,
    Extrude,
    // Count
}

impl EMoveType {
    /// `static_cast<size_t>(EMoveType::Count)`
    pub const COUNT: usize = 11;
}

impl Default for EMoveType {
    fn default() -> Self {
        EMoveType::Noop
    }
}

// ===========================================================================
// GCodeProcessor.hpp:47-58  enum SkipType + skip_type_map
// ===========================================================================
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SkipType {
    StTimelapse,
    StHeadWrapDetect,
    StOther,
    StNone,
}

impl Default for SkipType {
    fn default() -> Self {
        SkipType::StNone
    }
}

/// GCodeProcessor.hpp:55-58 `skip_type_map`
pub fn skip_type_map(key: &str) -> Option<SkipType> {
    match key {
        "timelapse" => Some(SkipType::StTimelapse),
        "head_wrap_detect" => Some(SkipType::StHeadWrapDetect),
        _ => None,
    }
}

// ===========================================================================
// GCodeProcessor.hpp:59-121  struct PrintEstimatedStatistics
// ===========================================================================
/// GCodeProcessor.hpp:61-66  enum class ETimeMode : unsigned char
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ETimeMode {
    Normal,
    Stealth,
    // Count
}

impl ETimeMode {
    /// `static_cast<size_t>(ETimeMode::Count)`
    pub const COUNT: usize = 2;
}

impl Default for ETimeMode {
    fn default() -> Self {
        ETimeMode::Normal
    }
}

/// GCodeProcessor.hpp:68-89  struct PrintEstimatedStatistics::Mode
#[derive(Debug, Clone, Default)]
pub struct Mode {
    pub time: f32,
    pub prepare_time: f32,
    pub custom_gcode_times: Vec<(CustomGCodeType, (f32, f32))>,
    pub moves_times: Vec<(EMoveType, f32)>,
    pub roles_times: Vec<(ExtrusionRole, f32)>,
    pub layers_times: Vec<f32>,
    /// GCodeProcessor.hpp:77-80 `std::vector<float> preview_layers_times;`: like layers_times but one
    /// entry per slider tick, filled only for mixed-color sublayers.
    pub preview_layers_times: Vec<f32>,
}

impl Mode {
    /// GCodeProcessor.hpp:77-88  void reset()
    pub fn reset(&mut self) {
        self.time = 0.0;
        self.prepare_time = 0.0;
        self.custom_gcode_times.clear();
        self.moves_times.clear();
        self.roles_times.clear();
        self.layers_times.clear();
        self.preview_layers_times.clear();
    }
}

/// GCodeProcessor.hpp:59-121  struct PrintEstimatedStatistics
#[derive(Debug, Clone, Default)]
pub struct PrintEstimatedStatistics {
    pub volumes_per_color_change: Vec<f64>,
    pub model_volumes_per_extruder: BTreeMap<usize, f64>,
    pub wipe_tower_volumes_per_extruder: BTreeMap<usize, f64>,
    pub support_volumes_per_extruder: BTreeMap<usize, f64>,
    pub total_volumes_per_extruder: BTreeMap<usize, f64>,
    /// BBS: the flush amount of every filament
    pub flush_per_filament: BTreeMap<usize, f64>,
    /// keyed by `ExtrusionRole as u8` (see `RoleKey`).
    pub used_filaments_per_role: BTreeMap<RoleKey, (f64, f64)>,

    pub modes: [Mode; ETimeMode::COUNT],
    /// GCodeProcessor.hpp:105-106
    pub load_time_per_filament: BTreeMap<usize, f64>,
    pub unload_time_per_filament: BTreeMap<usize, f64>,
    pub total_flush_filament_changes: u32,
    pub total_filament_changes: u32,
}

impl PrintEstimatedStatistics {
    /// GCodeProcessor.hpp:104  PrintEstimatedStatistics() { reset(); }
    pub fn new() -> Self {
        let mut s = Self::default();
        s.reset();
        s
    }

    /// GCodeProcessor.hpp:106-120  void reset()
    pub fn reset(&mut self) {
        // for (auto m : modes) { m.reset(); }
        // NOTE: faithful to C++: the original iterates over *copies* (`auto m`),
        // so this loop is a no-op on the stored modes; preserved exactly.
        for mut m in self.modes.clone() {
            m.reset();
        }
        self.volumes_per_color_change.clear();
        self.wipe_tower_volumes_per_extruder.clear();
        self.model_volumes_per_extruder.clear();
        self.support_volumes_per_extruder.clear();
        self.total_volumes_per_extruder.clear();
        self.flush_per_filament.clear();
        self.used_filaments_per_role.clear();
        self.load_time_per_filament.clear();
        self.unload_time_per_filament.clear();
        self.total_flush_filament_changes = 0;
        self.total_filament_changes = 0;
    }
}

// ===========================================================================
// GCodeProcessor.hpp:123-137  struct ConflictResult / ConflictResultOpt
// ===========================================================================
/// GCodeProcessor.hpp:123-135  struct ConflictResult
///
/// The C++ holds `const void* _obj1/_obj2` (nullptr means wipe tower); these
/// opaque pointers are not representable across the FFI boundary and are not
/// load-bearing for parity, so they are omitted here.
#[derive(Debug, Clone)]
pub struct ConflictResult {
    pub obj_name1: String, // _objName1
    pub obj_name2: String, // _objName2
    pub height: f32,       // _height
    pub layer: i32,
}

impl ConflictResult {
    /// GCodeProcessor.hpp:131-133  ctor
    pub fn new(obj_name1: String, obj_name2: String, height: f32) -> Self {
        Self {
            obj_name1,
            obj_name2,
            height,
            layer: -1,
        }
    }
}

impl Default for ConflictResult {
    /// GCodeProcessor.hpp:134  ConflictResult() = default; with `int layer = -1;`
    fn default() -> Self {
        Self {
            obj_name1: String::new(),
            obj_name2: String::new(),
            height: 0.0,
            layer: -1,
        }
    }
}

/// GCodeProcessor.hpp:137  using ConflictResultOpt = std::optional<ConflictResult>;
pub type ConflictResultOpt = Option<ConflictResult>;

// ===========================================================================
// GCodeProcessor.hpp:139-151  struct GCodeCheckResult
// ===========================================================================
#[derive(Debug, Clone, Default)]
pub struct GCodeCheckResult {
    /// 0 means succeed, 0b0001 multi extruder printable area error, 0b0010
    /// multi extruder printable height error, 0b0100 plate printable area
    /// error, 0b1000 plate printable height error, 0b10000 wrapping detection
    /// area error, (1<<10) filament map error
    pub error_code: i32,
    /// printable_area extruder_id to <filament_id - object_label_id>
    pub print_area_error_infos: BTreeMap<i32, Vec<(i32, i32)>>,
    /// printable_height extruder_id to <filament_id - object_label_id>
    pub print_height_error_infos: BTreeMap<i32, Vec<(i32, i32)>>,
}

impl GCodeCheckResult {
    /// GCodeProcessor.hpp:146-150  void reset()
    pub fn reset(&mut self) {
        self.error_code = 0;
        self.print_area_error_infos.clear();
        self.print_height_error_infos.clear();
    }
}

// ===========================================================================
// GCodeProcessor.hpp:153-163  struct FilamentPrintableResult
// ===========================================================================
#[derive(Debug, Clone, Default)]
pub struct FilamentPrintableResult {
    pub conflict_filament: Vec<i32>,
    pub plate_name: String,
}

impl FilamentPrintableResult {
    /// GCodeProcessor.hpp:157  FilamentPrintableResult(){};
    pub fn new() -> Self {
        Self::default()
    }

    /// GCodeProcessor.hpp:158  ctor from conflict_filament + plate_name
    pub fn with_conflicts(conflict_filament: Vec<i32>, plate_name: String) -> Self {
        Self {
            conflict_filament,
            plate_name,
        }
    }

    /// GCodeProcessor.hpp:159-161  bool has_value() const
    pub fn has_value(&self) -> bool {
        !self.conflict_filament.is_empty()
    }

    /// GCodeProcessor.cpp:175-179  void FilamentPrintableResult::reset()
    pub fn reset(&mut self) {
        self.conflict_filament.clear();
        self.plate_name = String::new();
    }
}

// ===========================================================================
// GCodeProcessor.hpp:235-240  GCodeProcessorResult::SliceWarning
// (`GCodeProcessorResult` itself is not yet fully ported; its nested POD types
// live here so dependents — e.g. `Format/bbs_3mf.cpp`'s `PlateData::warnings`
// — can be ported faithfully.)
// ===========================================================================
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SliceWarning {
    /// 0: normal tips, 1: warning; 2: error  GCodeProcessor.hpp:236
    pub level: i32,
    /// enum string  GCodeProcessor.hpp:237
    pub msg: String,
    /// error code for studio  GCodeProcessor.hpp:238
    pub error_code: String,
    /// extra msg info  GCodeProcessor.hpp:239
    pub params: Vec<String>,
}

// ===========================================================================
// GCodeProcessor.hpp:242-247  GCodeProcessorResult::FilamentUseInfo
// ===========================================================================
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FilamentUseInfo {
    /// GCodeProcessor.hpp:244  int filament_id = 0;
    pub filament_id: i32,
    /// GCodeProcessor.hpp:245  bool use_for_object{false};
    pub use_for_object: bool,
    /// GCodeProcessor.hpp:246  bool use_for_support{false};
    pub use_for_support: bool,
}

// ===========================================================================
// GCodeProcessor.hpp:167-175  GCodeProcessorResult::FilamentSequenceHash
// Hash functor for `std::unordered_map<std::vector<unsigned int>, ...>` keys;
// in Rust the corresponding maps use the default `HashMap` hasher (identical
// map semantics — key equality is still full vector equality). The C++ hash
// function is preserved here for reference/parity tooling.
// ===========================================================================
pub fn filament_sequence_hash(layer_filament: &[u32]) -> u64 {
    // GCodeProcessor.hpp:169-173
    let mut key: u64 = 0;
    for &f in layer_filament {
        key |= 1u64.wrapping_shl(f);
    }
    key
}

// ===========================================================================
// GCodeProcessor.hpp:504-513  struct ThermalIndex
// ===========================================================================
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalIndex {
    pub max: f32,
    pub min: f32,
    pub mean: f32,
    pub is_null: bool,
}

impl ThermalIndex {
    /// GCodeProcessor.hpp:510  ThermalIndex() : min(-200), max(-200), mean(-200), isNull(true) {}
    pub fn new() -> Self {
        Self {
            min: -200.0,
            max: -200.0,
            mean: -200.0,
            is_null: true,
        }
    }

    /// GCodeProcessor.hpp:512  ThermalIndex(minVal, maxVal, meanVal)
    pub fn with_values(min_val: f32, max_val: f32, mean_val: f32) -> Self {
        Self {
            min: min_val,
            max: max_val,
            mean: mean_val,
            is_null: false,
        }
    }
}

impl Default for ThermalIndex {
    fn default() -> Self {
        Self::new()
    }
}

// ===========================================================================
// GCodeProcessor.hpp:182-193  GCodeProcessorResult::SettingsIds
// ===========================================================================
#[derive(Debug, Clone, Default)]
pub struct SettingsIds {
    pub print: String,
    pub filament: Vec<String>,
    pub printer: String,
}

impl SettingsIds {
    /// GCodeProcessor.hpp:188-192  void reset()
    pub fn reset(&mut self) {
        self.print.clear();
        self.filament.clear();
        self.printer.clear();
    }
}

// ===========================================================================
// GCodeProcessor.hpp:516-546 (private) — units / positioning / cached position
// ===========================================================================
/// GCodeProcessor.hpp:520-524  enum class EUnits : unsigned char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EUnits {
    Millimeters,
    Inches,
}

impl Default for EUnits {
    fn default() -> Self {
        EUnits::Millimeters
    }
}

/// GCodeProcessor.hpp:526-530  enum class EPositioningType : unsigned char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EPositioningType {
    Absolute,
    Relative,
}

impl Default for EPositioningType {
    fn default() -> Self {
        EPositioningType::Absolute
    }
}

/// GCodeProcessor.hpp:516  using AxisCoords = std::array<double, 4>;
pub type AxisCoords = [f64; 4];

/// GCodeProcessor.hpp:532-538  struct CachedPosition
#[derive(Debug, Clone)]
pub struct CachedPosition {
    pub position: AxisCoords, // mm
    pub feedrate: f32,        // mm/s
}

impl CachedPosition {
    /// GCodeProcessor.cpp:228-232  void GCodeProcessor::CachedPosition::reset()
    pub fn reset(&mut self) {
        // std::fill(position.begin(), position.end(), FLT_MAX);
        self.position = [f32::MAX as f64; 4];
        // NOTE: faithful: C++ fills the AxisCoords (doubles) with FLT_MAX, not
        // DBL_MAX, so the value is exactly the float maximum widened to double.
        self.feedrate = f32::MAX;
    }
}

impl Default for CachedPosition {
    fn default() -> Self {
        let mut p = CachedPosition {
            position: [0.0; 4],
            feedrate: 0.0,
        };
        p.reset();
        p
    }
}

/// GCodeProcessor.hpp:540-546  struct CpColor
#[derive(Debug, Clone, Default)]
pub struct CpColor {
    pub counter: u8,
    pub current: u8,
}

impl CpColor {
    /// GCodeProcessor.cpp:234-238  void GCodeProcessor::CpColor::reset()
    pub fn reset(&mut self) {
        self.counter = 0;
        self.current = 0;
    }
}

// ===========================================================================
// Static kinematics helpers — GCodeProcessor.cpp:119-148
// ===========================================================================

/// GCodeProcessor.cpp:119-122
fn estimated_acceleration_distance(initial_rate: f32, target_rate: f32, acceleration: f32) -> f32 {
    if acceleration == 0.0 {
        0.0
    } else {
        (sqr(target_rate) - sqr(initial_rate)) / (2.0 * acceleration)
    }
}

/// GCodeProcessor.cpp:124-127
fn intersection_distance(
    initial_rate: f32,
    final_rate: f32,
    acceleration: f32,
    distance: f32,
) -> f32 {
    if acceleration == 0.0 {
        0.0
    } else {
        if crate::faithful_gate("TIME_FLOAT_MODEL") {
            // `(2a) * d - sqr(i)` is one fused multiply-subtract on arm64.
            ((2.0 * acceleration).mul_add(distance, -sqr(initial_rate)) + sqr(final_rate))
                / (4.0 * acceleration)
        } else {
            (2.0 * acceleration * distance - sqr(initial_rate) + sqr(final_rate))
                / (4.0 * acceleration)
        }
    }
}

/// GCodeProcessor.cpp:129-134
fn speed_from_distance(initial_feedrate: f32, distance: f32, acceleration: f32) -> f32 {
    // to avoid invalid negative numbers due to numerical errors
    // clang on arm64 contracts `sqr(i) + (2a) * d` into one fused multiply-add.
    let value = if crate::faithful_gate("TIME_FLOAT_MODEL") {
        f32::max(0.0, (2.0 * acceleration).mul_add(distance, sqr(initial_feedrate)))
    } else {
        f32::max(0.0, sqr(initial_feedrate) + 2.0 * acceleration * distance)
    };
    value.sqrt()
}

/// GCodeProcessor.cpp:136-143
///
/// Calculates the maximum allowable speed at this point when you must be able
/// to reach target_velocity using the acceleration within the allotted distance.
fn max_allowable_speed(acceleration: f32, target_velocity: f32, distance: f32) -> f32 {
    // to avoid invalid negative numbers due to numerical errors
    // clang on arm64 contracts `sqr(t) - (2a) * d` into one fused multiply-subtract.
    let value = if crate::faithful_gate("TIME_FLOAT_MODEL") {
        f32::max(0.0, (-(2.0 * acceleration)).mul_add(distance, sqr(target_velocity)))
    } else {
        f32::max(0.0, sqr(target_velocity) - 2.0 * acceleration * distance)
    };
    value.sqrt()
}

/// GCodeProcessor.cpp:145-148
fn acceleration_time_from_distance(initial_feedrate: f32, distance: f32, acceleration: f32) -> f32 {
    if acceleration != 0.0 {
        (speed_from_distance(initial_feedrate, distance, acceleration) - initial_feedrate)
            / acceleration
    } else {
        0.0
    }
}

// ===========================================================================
// Comment-parse helpers — GCodeProcessor.cpp:150-173
// ===========================================================================

/// GCodeProcessor.cpp:150-161
pub fn get_object_label_id(comment_1: &str) -> i32 {
    // std::string comment(comment_1);
    let comment = comment_1;
    // auto pos = comment.find(":");
    let pos = comment.find(':');
    // std::string num_str = comment.substr(pos + 1);
    // C++ string::substr(npos+1) == substr(0) when find returns npos; faithful:
    let num_str = match pos {
        Some(p) => &comment[p + 1..],
        None => {
            // npos == size_t(-1); npos + 1 == 0, so substr(0) == whole string.
            comment
        }
    };
    // int id = -1; try { id = stoi(num_str); } catch (...) {}
    parse_stoi(num_str).unwrap_or(-1)
}

/// GCodeProcessor.cpp:163-173
pub fn get_z_height(comment_1: &str) -> f32 {
    let comment = comment_1;
    let pos = comment.find(':');
    let num_str = match pos {
        Some(p) => &comment[p + 1..],
        None => comment,
    };
    // float print_z = 0.0f; try { print_z = stof(num_str); } catch (...) {}
    parse_stof(num_str).unwrap_or(0.0)
}

/// Mirror of `std::stoi`: parse a leading (whitespace-skipped, signed) base-10
/// integer prefix; throws (returns None) if no conversion could be performed.
fn parse_stoi(s: &str) -> Option<i32> {
    let bytes = s.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() && (bytes[i] as char).is_whitespace() {
        i += 1;
    }
    let start = i;
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        i += 1;
    }
    let digits_start = i;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
    }
    if i == digits_start {
        return None; // std::invalid_argument
    }
    s[start..i].parse::<i32>().ok()
}

/// Mirror of `std::stof`: parse a leading floating-point prefix; throws
/// (returns None) if no conversion could be performed.
fn parse_stof(s: &str) -> Option<f32> {
    // Gate PROCESSOR_STOF_PIN (faithful): `std::stof` as libc++ runs it (strtof, then out_of_range when
    // errno is ERANGE): it also reads "inf"/"infinity"/"nan" and hexadecimal floats, and an overflow to
    // infinity throws (get_z_height's catch then keeps 0). Checked against the pin by
    // tests/g_code_processor_helpers_native_parity.rs. =0: the earlier decimal-only reader below.
    if crate::faithful_gate("PROCESSOR_STOF_PIN") {
        return parse_stof_pin(s);
    }
    let trimmed = s.trim_start();
    // Find the longest leading prefix that parses as a float. std::stof parses
    // the leading numeric token and ignores trailing characters.
    let bytes = trimmed.as_bytes();
    let mut end = 0usize;
    let mut seen_digit = false;
    let mut seen_dot = false;
    let mut seen_e = false;
    if end < bytes.len() && (bytes[end] == b'+' || bytes[end] == b'-') {
        end += 1;
    }
    while end < bytes.len() {
        let c = bytes[end];
        if c.is_ascii_digit() {
            seen_digit = true;
            end += 1;
        } else if c == b'.' && !seen_dot && !seen_e {
            seen_dot = true;
            end += 1;
        } else if (c == b'e' || c == b'E') && seen_digit && !seen_e {
            seen_e = true;
            end += 1;
            if end < bytes.len() && (bytes[end] == b'+' || bytes[end] == b'-') {
                end += 1;
            }
        } else {
            break;
        }
    }
    if !seen_digit {
        return None;
    }
    trimmed[..end].parse::<f32>().ok()
}

/// `std::stof(str)` (libc++: `strtof` over the C string, `invalid_argument` when nothing is read,
/// `out_of_range` when errno is ERANGE): leading C-locale white space, a sign, then "inf"/"infinity",
/// "nan", a hexadecimal float (`0x` digits, '.', binary exponent `p`) or a decimal float. None for the
/// two exceptions. An underflow keeps what the decimal reader gives (the pin's vectors agree with it for
/// the smallest normal and a subnormal input).
fn parse_stof_pin(s: &str) -> Option<f32> {
    let b = s.as_bytes();
    let mut i = 0usize;
    while i < b.len() && matches!(b[i], b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r') {
        i += 1;
    }
    let mut neg = false;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        neg = b[i] == b'-';
        i += 1;
    }
    let rest = &b[i..];
    let lower_starts = |w: &[u8]| rest.len() >= w.len() && rest[..w.len()].eq_ignore_ascii_case(w);
    let signed = |v: f32| if neg { -v } else { v };
    if lower_starts(b"inf") {
        return Some(signed(f32::INFINITY));
    }
    if lower_starts(b"nan") {
        return Some(signed(f32::NAN));
    }
    // Hexadecimal float: "0x" then at least one hex digit (before or after the point).
    if rest.len() >= 2 && rest[0] == b'0' && (rest[1] == b'x' || rest[1] == b'X') {
        let mut j = 2usize;
        let mut mant: u64 = 0;
        let mut exp: i64 = 0;
        let mut any = false;
        let mut sticky = false;
        let mut seen_dot = false;
        while j < rest.len() {
            let c = rest[j];
            if c == b'.' && !seen_dot {
                seen_dot = true;
                j += 1;
                continue;
            }
            let d = match (c as char).to_digit(16) {
                Some(d) => d as u64,
                None => break,
            };
            any = true;
            if mant >> 56 == 0 {
                mant = (mant << 4) | d;
                if seen_dot {
                    exp -= 4;
                }
            } else {
                sticky |= d != 0;
                if !seen_dot {
                    exp += 4;
                }
            }
            j += 1;
        }
        if any {
            if j < rest.len() && (rest[j] == b'p' || rest[j] == b'P') {
                let mut k = j + 1;
                let mut eneg = false;
                if k < rest.len() && (rest[k] == b'+' || rest[k] == b'-') {
                    eneg = rest[k] == b'-';
                    k += 1;
                }
                let ds = k;
                let mut e: i64 = 0;
                while k < rest.len() && rest[k].is_ascii_digit() {
                    e = (e * 10 + (rest[k] - b'0') as i64).min(1 << 20);
                    k += 1;
                }
                if k > ds {
                    exp += if eneg { -e } else { e };
                }
            }
            if sticky {
                mant |= 1;
            }
            // One rounding to float (u64 -> f32), then an exact scaling by a power of two in double.
            let v = ((mant as f32) as f64 * 2f64.powi(exp.clamp(-2000, 2000) as i32)) as f32;
            if v.is_infinite() {
                return None; // ERANGE: std::out_of_range
            }
            return Some(signed(v));
        }
        // "0x" without digits: strtof reads the "0".
        return Some(signed(0.0));
    }
    // Decimal: digits, an optional point and digits (at least one digit), an exponent only when digits
    // follow it.
    let mut j = 0usize;
    let mut digits = 0usize;
    while j < rest.len() && rest[j].is_ascii_digit() {
        j += 1;
        digits += 1;
    }
    if j < rest.len() && rest[j] == b'.' {
        j += 1;
        while j < rest.len() && rest[j].is_ascii_digit() {
            j += 1;
            digits += 1;
        }
    }
    if digits == 0 {
        return None; // std::invalid_argument
    }
    if j < rest.len() && (rest[j] == b'e' || rest[j] == b'E') {
        let mut k = j + 1;
        if k < rest.len() && (rest[k] == b'+' || rest[k] == b'-') {
            k += 1;
        }
        if k < rest.len() && rest[k].is_ascii_digit() {
            while k < rest.len() && rest[k].is_ascii_digit() {
                k += 1;
            }
            j = k;
        }
    }
    let text = std::str::from_utf8(&rest[..j]).ok()?;
    let v = text.parse::<f32>().ok()?;
    if v.is_infinite() {
        return None; // ERANGE: std::out_of_range
    }
    Some(signed(v))
}

/// GCodeProcessor.cpp:3135-3170  `parse_number<float>`: `std::from_chars`, which
/// takes no leading '+' and must consume the whole string.
fn parse_number_f32(sv: &str) -> Option<f32> {
    if sv.is_empty() || sv.starts_with('+') {
        return None;
    }
    sv.parse::<f32>().ok()
}

/// GCodeProcessor.cpp:2982-3016 (template) — `parse_number<int>` specialization.
///
/// Returns Some(out) only when the *entire* string parses as an int (matching
/// `from_chars` consuming the whole input). Used by `get_gcode_last_filament`.
fn parse_number_i32(sv: &str) -> Option<i32> {
    // std::from_chars: optional leading '-', no leading '+', no whitespace,
    // and must consume the whole string view.
    if sv.is_empty() {
        return None;
    }
    let bytes = sv.as_bytes();
    let mut i = 0usize;
    if bytes[0] == b'-' {
        i = 1;
    }
    if i >= bytes.len() {
        return None;
    }
    for &b in &bytes[i..] {
        if !b.is_ascii_digit() {
            return None;
        }
    }
    sv.parse::<i32>().ok()
}

// ===========================================================================
// CommandProcessor — GCodeProcessor.cpp:181-226 / hpp:422-437
// ===========================================================================
//
// The C++ `CommandProcessor` is a char-trie that dispatches gcode commands to
// `std::function` handlers bound to `&GCodeProcessor`. The handler closures
// borrow the processor mutably, which cannot be modeled as stored
// `Box<dyn Fn>` fields in Rust without unsafe aliasing. We therefore port the
// *trie structure and matching algorithm* faithfully (this is the
// parity-relevant, deterministic part: which registered command, if any,
// matches a given input and whether early-quit triggers), and expose the match
// result as the registered command id. The actual dispatch to processor
// methods is performed by the (blocked) `GCodeProcessor` driver.

/// GCodeProcessor.hpp:426-430  struct TrieNode
struct TrieNode {
    /// Index into the registered-handler table; `None` == nullptr handler.
    handler: Option<usize>,
    children: HashMap<u8, Box<TrieNode>>,
    /// stop matching, trigger handle immediately
    early_quit: bool,
}

impl TrieNode {
    fn new() -> Self {
        TrieNode {
            handler: None,
            children: HashMap::new(),
            early_quit: false,
        }
    }
}

/// GCodeProcessor.hpp:422-437  class CommandProcessor
pub struct CommandProcessor {
    root: Box<TrieNode>,
}

impl CommandProcessor {
    /// GCodeProcessor.cpp:181-184  CommandProcessor::CommandProcessor()
    pub fn new() -> Self {
        CommandProcessor {
            root: Box::new(TrieNode::new()),
        }
    }

    /// GCodeProcessor.cpp:186-206  register_command
    ///
    /// `handler_id` is the index of the command's handler (replacing the C++
    /// `command_handler_t` function pointer, which is not portable here).
    pub fn register_command(&mut self, str: &str, handler_id: usize, early_quit: bool) {
        // TrieNode* node = root.get();
        let mut node: &mut TrieNode = &mut self.root;
        // for (char ch : str)
        for &ch in str.as_bytes() {
            node = node
                .children
                .entry(ch)
                .or_insert_with(|| Box::new(TrieNode::new()));
        }
        // if (node->handler != nullptr) { assert(false); } // duplicated command
        debug_assert!(node.handler.is_none(), "duplicated command");
        node.handler = Some(handler_id);
        node.early_quit = early_quit;
    }

    /// GCodeProcessor.cpp:208-226  process_comand
    ///
    /// Returns `Some(handler_id)` if a registered handler matched (the caller
    /// should invoke that handler), else `None`.
    pub fn process_comand(&self, cmd: &str) -> Option<usize> {
        // TrieNode* node = root.get();
        let mut node: &TrieNode = &self.root;
        // for (char ch : cmd)
        for &ch in cmd.as_bytes() {
            // if (node->early_quit && node->handler) { handler(line); return true; }
            if node.early_quit && node.handler.is_some() {
                return node.handler;
            }
            // auto iter = node->children.find(ch);
            match node.children.get(&ch) {
                None => return None,
                Some(next) => node = next,
            }
        }
        // if (!node || !node->handler) return false;
        node.handler
    }
}

impl Default for CommandProcessor {
    fn default() -> Self {
        Self::new()
    }
}

// ===========================================================================
// FeedrateProfile / Trapezoid / TimeBlock — hpp:549-595, cpp:240-286
// ===========================================================================

/// GCodeProcessor.hpp:549-554  struct FeedrateProfile
#[derive(Debug, Clone, Copy, Default)]
pub struct FeedrateProfile {
    pub entry: f32,  // mm/s
    pub cruise: f32, // mm/s
    pub exit: f32,   // mm/s
}

/// GCodeProcessor.hpp:556-566  struct Trapezoid
#[derive(Debug, Clone, Copy, Default)]
pub struct Trapezoid {
    pub accelerate_until: f32, // mm
    pub decelerate_after: f32, // mm
    pub cruise_feedrate: f32,  // mm/sec
}

impl Trapezoid {
    /// GCodeProcessor.cpp:240-243
    pub fn acceleration_time(&self, entry_feedrate: f32, acceleration: f32) -> f32 {
        acceleration_time_from_distance(entry_feedrate, self.accelerate_until, acceleration)
    }

    /// GCodeProcessor.cpp:245-248
    pub fn cruise_time(&self) -> f32 {
        if self.cruise_feedrate != 0.0 {
            self.cruise_distance() / self.cruise_feedrate
        } else {
            0.0
        }
    }

    /// GCodeProcessor.cpp:250-253
    pub fn deceleration_time(&self, distance: f32, acceleration: f32) -> f32 {
        acceleration_time_from_distance(
            self.cruise_feedrate,
            distance - self.decelerate_after,
            -acceleration,
        )
    }

    /// GCodeProcessor.cpp:255-258
    pub fn cruise_distance(&self) -> f32 {
        self.decelerate_after - self.accelerate_until
    }
}

/// GCodeProcessor.hpp:570-575  TimeBlock::Flags
#[derive(Debug, Clone, Copy, Default)]
pub struct TimeBlockFlags {
    pub recalculate: bool,
    pub nominal_length: bool,
    pub prepare_stage: bool,
}

/// GCodeProcessor.hpp:568-595  struct TimeBlock
#[derive(Debug, Clone)]
pub struct TimeBlock {
    pub move_type: EMoveType,
    pub role: ExtrusionRole,
    pub skippable_type: SkipType,
    /// index of the related move vertex, assigned during gcode process
    pub move_id: u32,
    pub g1_line_id: u32,
    pub layer_id: u32,
    pub distance: f32,        // mm
    pub acceleration: f32,    // mm/s^2
    pub max_entry_speed: f32, // mm/s
    pub safe_feedrate: f32,   // mm/s
    pub flags: TimeBlockFlags,
    pub feedrate_profile: FeedrateProfile,
    pub trapezoid: Trapezoid,
}

impl Default for TimeBlock {
    fn default() -> Self {
        TimeBlock {
            move_type: EMoveType::Noop,
            role: ExtrusionRole::None,
            skippable_type: SkipType::StNone,
            move_id: 0,
            g1_line_id: 0,
            layer_id: 0,
            distance: 0.0,
            acceleration: 0.0,
            max_entry_speed: 0.0,
            safe_feedrate: 0.0,
            flags: TimeBlockFlags::default(),
            feedrate_profile: FeedrateProfile::default(),
            trapezoid: Trapezoid::default(),
        }
    }
}

impl TimeBlock {
    /// GCodeProcessor.cpp:260-279  void TimeBlock::calculate_trapezoid()
    pub fn calculate_trapezoid(&mut self) {
        self.trapezoid.cruise_feedrate = self.feedrate_profile.cruise;

        let mut accelerate_distance = f32::max(
            0.0,
            estimated_acceleration_distance(
                self.feedrate_profile.entry,
                self.feedrate_profile.cruise,
                self.acceleration,
            ),
        );
        let decelerate_distance = f32::max(
            0.0,
            estimated_acceleration_distance(
                self.feedrate_profile.cruise,
                self.feedrate_profile.exit,
                -self.acceleration,
            ),
        );
        let mut cruise_distance = self.distance - accelerate_distance - decelerate_distance;

        // Not enough space to reach the nominal feedrate.
        // This means no cruising, and we'll have to use intersection_distance() to
        // calculate when to abort acceleration and start braking in order to reach
        // the exit_feedrate exactly at the end of this block.
        if cruise_distance < 0.0 {
            accelerate_distance = intersection_distance(
                self.feedrate_profile.entry,
                self.feedrate_profile.exit,
                self.acceleration,
                self.distance,
            )
            .clamp(0.0, self.distance);
            cruise_distance = 0.0;
            self.trapezoid.cruise_feedrate = speed_from_distance(
                self.feedrate_profile.entry,
                accelerate_distance,
                self.acceleration,
            );
        }

        self.trapezoid.accelerate_until = accelerate_distance;
        self.trapezoid.decelerate_after = accelerate_distance + cruise_distance;
    }

    /// GCodeProcessor.cpp:281-286  float TimeBlock::time() const
    pub fn time(&self) -> f32 {
        self.trapezoid
            .acceleration_time(self.feedrate_profile.entry, self.acceleration)
            + self.trapezoid.cruise_time()
            + self
                .trapezoid
                .deceleration_time(self.distance, self.acceleration)
    }
}

// ===========================================================================
// Planner kernels — GCodeProcessor.cpp:338-407
// ===========================================================================

/// GCodeProcessor.cpp:338-356  planner_forward_pass_kernel
fn planner_forward_pass_kernel(prev: &mut TimeBlock, curr: &mut TimeBlock) {
    // If the previous block is an acceleration block, but it is not long enough to
    // complete the full speed change within the block, we need to adjust the entry
    // speed accordingly. Entry speeds have already been reset, maximized, and
    // reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No
    // need to recheck.
    if !prev.flags.nominal_length {
        if prev.feedrate_profile.entry < curr.feedrate_profile.entry {
            let entry_speed = f32::min(
                curr.feedrate_profile.entry,
                max_allowable_speed(-prev.acceleration, prev.feedrate_profile.entry, prev.distance),
            );
            // Check for junction speed change
            if curr.feedrate_profile.entry != entry_speed {
                curr.feedrate_profile.entry = entry_speed;
                curr.flags.recalculate = true;
            }
        }
    }
}

/// GCodeProcessor.cpp:358-373  planner_reverse_pass_kernel
fn planner_reverse_pass_kernel(curr: &mut TimeBlock, next: &TimeBlock) {
    // If entry speed is already at the maximum entry speed, no need to recheck.
    // Block is cruising. If not, block in state of acceleration or deceleration.
    // Reset entry speed to maximum and check for maximum allowable speed reductions
    // to ensure maximum possible planned speed.
    if curr.feedrate_profile.entry != curr.max_entry_speed {
        // If nominal length true, max junction speed is guaranteed to be reached.
        // Only compute for max allowable speed if block is decelerating and nominal
        // length is false.
        if !curr.flags.nominal_length && curr.max_entry_speed > next.feedrate_profile.entry {
            curr.feedrate_profile.entry = f32::min(
                curr.max_entry_speed,
                max_allowable_speed(-curr.acceleration, next.feedrate_profile.entry, curr.distance),
            );
        } else {
            curr.feedrate_profile.entry = curr.max_entry_speed;
        }

        curr.flags.recalculate = true;
    }
}

/// GCodeProcessor.cpp:375-407  recalculate_trapezoids
fn recalculate_trapezoids(blocks: &mut [TimeBlock]) {
    // GCodeProcessor.TimeBlock* curr = nullptr; *next = nullptr;
    // Indices are used instead of raw pointers; `usize::MAX` == nullptr.
    let mut curr: usize = usize::MAX;
    let mut next: usize = usize::MAX;

    for i in 0..blocks.len() {
        curr = next;
        next = i;

        if curr != usize::MAX {
            // Recalculate if current block entry or exit junction speed has changed.
            if blocks[curr].flags.recalculate || blocks[next].flags.recalculate {
                // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                let exit = blocks[next].feedrate_profile.entry;
                let block = &mut blocks[curr];
                block.feedrate_profile.exit = exit;
                block.calculate_trapezoid();
                // curr->trapezoid = block.trapezoid; (same object here)
                block.flags.recalculate = false; // Reset current only to ensure next trapezoid is computed
            }
        }
    }

    // Last/newest block in buffer. Always recalculated.
    if next != usize::MAX {
        let safe = blocks[next].safe_feedrate;
        let block = &mut blocks[next];
        block.feedrate_profile.exit = safe;
        block.calculate_trapezoid();
        block.flags.recalculate = false;
    }
}

// ===========================================================================
// TimeMachine — hpp:599-700, cpp:288-520
// ===========================================================================

/// GCodeProcessor.hpp:601-616  TimeMachine::State
#[derive(Debug, Clone, Default)]
pub struct State {
    pub feedrate: f32,                // mm/s
    pub safe_feedrate: f32,           // mm/s
    pub axis_feedrate: AxisCoords,    // mm/s
    pub abs_axis_feedrate: AxisCoords, // mm/s
    /// BBS: unit vector of enter speed in x-y-z space.
    pub enter_direction: [f32; 3],
    /// BBS: unit vector of exit speed in x-y-z space.
    pub exit_direction: [f32; 3],
}

impl State {
    /// GCodeProcessor.cpp:288-297  void TimeMachine::State::reset()
    pub fn reset(&mut self) {
        self.feedrate = 0.0;
        self.safe_feedrate = 0.0;
        self.axis_feedrate = [0.0, 0.0, 0.0, 0.0];
        self.abs_axis_feedrate = [0.0, 0.0, 0.0, 0.0];
        // BBS
        self.enter_direction = [0.0, 0.0, 0.0];
        self.exit_direction = [0.0, 0.0, 0.0];
    }
}

/// GCodeProcessor.hpp:618-625  TimeMachine::CustomGCodeTime
#[derive(Debug, Clone, Default)]
pub struct CustomGCodeTime {
    pub needed: bool,
    pub cache: f32,
    pub times: Vec<(CustomGCodeType, f32)>,
}

impl CustomGCodeTime {
    /// GCodeProcessor.cpp:299-304  void TimeMachine::CustomGCodeTime::reset()
    pub fn reset(&mut self) {
        self.needed = false;
        self.cache = 0.0;
        self.times = Vec::new();
    }
}

/// TIMEDBG_BLOCKS (diagnostics only): set while the replayed processor of
/// TIMEPROC_REPLAY runs, so only its blocks are dumped.
pub static TIMEDBG_BLOCKS_ARMED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// GCodeProcessor.hpp:627-631  TimeMachine::G1LinesCacheItem
#[derive(Debug, Clone, Copy, Default)]
pub struct G1LinesCacheItem {
    pub id: u32,
    pub elapsed_time: f32,
}

/// GCodeProcessor.hpp:645-649  TimeMachine::StopTime
#[derive(Debug, Clone, Copy, Default)]
pub struct StopTime {
    pub g1_line_id: u32,
    pub elapsed_time: f32,
}

/// GCodeProcessor.hpp:666  using AdditionalBufferBlock = std::pair<ExtrusionRole,float>;
pub type AdditionalBufferBlock = (ExtrusionRole, f32);
/// GCodeProcessor.hpp:667  using AdditionalBuffer = std::vector<AdditionalBufferBlock>;
pub type AdditionalBuffer = Vec<AdditionalBufferBlock>;

/// GCodeProcessor.hpp:599-700  struct TimeMachine
#[derive(Debug, Clone)]
pub struct TimeMachine {
    pub enabled: bool,
    pub acceleration: f32,     // mm/s^2
    pub max_acceleration: f32, // hard limit clamp, mm/s^2
    pub retract_acceleration: f32,
    pub max_retract_acceleration: f32,
    pub travel_acceleration: f32,
    pub max_travel_acceleration: f32,
    pub extrude_factor_override_percentage: f32,
    pub time: f32, // s
    pub stop_times: Vec<StopTime>,
    pub line_m73_main_mask: String,
    pub line_m73_stop_mask: String,
    pub curr: State,
    pub prev: State,
    pub gcode_time: CustomGCodeTime,
    pub blocks: Vec<TimeBlock>,
    pub g1_times_cache: Vec<G1LinesCacheItem>,
    pub moves_time: [f32; EMoveType::COUNT],
    pub roles_time: [f32; EXTRUSION_ROLE_COUNT],
    pub layers_time: Vec<f32>,
    /// BBS: prepare stage time before print model
    pub prepare_time: f32,
    /// GCodeProcessor.hpp:668  AdditionalBuffer m_additional_time_buffer;
    m_additional_time_buffer: AdditionalBuffer,
}

impl Default for TimeMachine {
    fn default() -> Self {
        TimeMachine {
            enabled: false,
            acceleration: 0.0,
            max_acceleration: 0.0,
            retract_acceleration: 0.0,
            max_retract_acceleration: 0.0,
            travel_acceleration: 0.0,
            max_travel_acceleration: 0.0,
            extrude_factor_override_percentage: 1.0,
            time: 0.0,
            stop_times: Vec::new(),
            line_m73_main_mask: String::new(),
            line_m73_stop_mask: String::new(),
            curr: State::default(),
            prev: State::default(),
            gcode_time: CustomGCodeTime::default(),
            blocks: Vec::new(),
            g1_times_cache: Vec::new(),
            moves_time: [0.0; EMoveType::COUNT],
            roles_time: [0.0; EXTRUSION_ROLE_COUNT],
            layers_time: Vec::new(),
            prepare_time: 0.0,
            m_additional_time_buffer: Vec::new(),
        }
    }
}

impl TimeMachine {
    /// GCodeProcessor.cpp:416-434  merge_adjacent_addtional_time_blocks
    pub fn merge_adjacent_addtional_time_blocks(buffer: &AdditionalBuffer) -> AdditionalBuffer {
        let mut merged: AdditionalBuffer = Vec::new();
        if buffer.is_empty() {
            return merged;
        }

        // auto current_block = buffer.front();
        let mut current_block = buffer[0];
        for idx in 1..buffer.len() {
            let next_block = buffer[idx];
            if current_block.0 == next_block.0 {
                current_block.1 += next_block.1;
            } else {
                merged.push(current_block);
                current_block = next_block;
            }
        }
        merged.push(current_block);
        merged
    }

    /// GCodeProcessor.cpp:306-328  void TimeMachine::reset()
    pub fn reset(&mut self) {
        self.enabled = false;
        self.acceleration = 0.0;
        self.max_acceleration = 0.0;
        self.retract_acceleration = 0.0;
        self.max_retract_acceleration = 0.0;
        self.travel_acceleration = 0.0;
        self.max_travel_acceleration = 0.0;
        self.extrude_factor_override_percentage = 1.0;
        self.time = 0.0;
        self.stop_times = Vec::new();
        self.curr.reset();
        self.prev.reset();
        self.gcode_time.reset();
        self.blocks = Vec::new();
        self.g1_times_cache = Vec::new();
        self.moves_time.iter_mut().for_each(|t| *t = 0.0);
        self.roles_time.iter_mut().for_each(|t| *t = 0.0);
        self.layers_time = Vec::new();
        self.prepare_time = 0.0;
        self.m_additional_time_buffer.clear();
    }

    /// GCodeProcessor.cpp:330-336  simulate_st_synchronize
    ///
    /// `block_handler` accepts (block, total_time) — mirrors the C++
    /// `block_handler_t = std::function<void(const TimeBlock&, const float)>`.
    pub fn simulate_st_synchronize<F: FnMut(&TimeBlock, f32)>(
        &mut self,
        additional_time: f32,
        target_role: ExtrusionRole,
        block_handler: F,
    ) {
        if !self.enabled {
            return;
        }
        self.calculate_time(0, additional_time, target_role, block_handler);
    }

    /// GCodeProcessor.cpp:437-520  void TimeMachine::calculate_time(...)
    pub fn calculate_time<F: FnMut(&TimeBlock, f32)>(
        &mut self,
        keep_last_n_blocks: usize,
        additional_time: f32,
        target_role: ExtrusionRole,
        mut block_handler: F,
    ) {
        if !self.enabled {
            return;
        }
        if self.blocks.len() < 2 {
            if additional_time > 0.0 {
                self.m_additional_time_buffer
                    .push((target_role, additional_time));
            }
            return;
        }

        debug_assert!(keep_last_n_blocks <= self.blocks.len());

        let mut additional_buffer: AdditionalBuffer = self.m_additional_time_buffer.clone();
        if additional_time > 0.0 {
            additional_buffer.push((target_role, additional_time));
        }
        additional_buffer = Self::merge_adjacent_addtional_time_blocks(&additional_buffer);

        // forward_pass
        for i in 0..self.blocks.len() - 1 {
            let (left, right) = self.blocks.split_at_mut(i + 1);
            planner_forward_pass_kernel(&mut left[i], &mut right[0]);
        }

        // reverse_pass
        let mut i = self.blocks.len() as i64 - 1;
        while i > 0 {
            let (left, right) = self.blocks.split_at_mut(i as usize);
            planner_reverse_pass_kernel(&mut left[(i - 1) as usize], &right[0]);
            i -= 1;
        }

        recalculate_trapezoids(&mut self.blocks);

        let n_blocks_process = self.blocks.len() - keep_last_n_blocks;
        let mut additional_buffer_idx = 0usize;

        for i in 0..n_blocks_process {
            // const TimeBlock& block = blocks[i];
            let mut block_time = self.blocks[i].time();

            if additional_buffer_idx < additional_buffer.len() {
                let buf_role = additional_buffer[additional_buffer_idx].0;
                let buf_time = additional_buffer[additional_buffer_idx].1;
                let is_valid_block =
                    (buf_role == ExtrusionRole::None) || (buf_role == self.blocks[i].role);
                if is_valid_block {
                    block_time += buf_time;
                    additional_buffer_idx += 1;
                }
            }

            self.time += block_time;
            // TIMEDBG_BLOCKS (diagnostics only): the planner's blocks as the C++
            // twin probe prints them, while `TIMEDBG_BLOCKS_ARMED` is set (the
            // replay of TIMEPROC_REPLAY sets it).
            if TIMEDBG_BLOCKS_ARMED.load(std::sync::atomic::Ordering::Relaxed) {
                use std::io::Write;
                // A LineWriter: the static is never dropped, so nothing else
                // would flush the last blocks.
                static OUT: std::sync::OnceLock<Option<std::sync::Mutex<std::io::LineWriter<std::fs::File>>>> =
                    std::sync::OnceLock::new();
                let out = OUT.get_or_init(|| {
                    std::env::var("TIMEDBG_BLOCKS")
                        .ok()
                        .and_then(|p| std::fs::File::create(p).ok())
                        .map(|f| std::sync::Mutex::new(std::io::LineWriter::new(f)))
                });
                if let Some(out) = out {
                    let b = &self.blocks[i];
                    let _ = writeln!(
                        out.lock().unwrap(),
                        "{} {} {} {:e} {:e} {:e} {:e} {:e} {:e} {:e}",
                        b.g1_line_id,
                        b.move_type as i32,
                        b.role as i32,
                        b.distance,
                        b.acceleration,
                        b.feedrate_profile.entry,
                        b.feedrate_profile.cruise,
                        b.feedrate_profile.exit,
                        b.time(),
                        block_time
                    );
                }
            }
            block_handler(&self.blocks[i], self.time);
            self.gcode_time.cache += block_time;
            // BBS: don't calculate travel of start gcode into travel time
            let block_move_type = self.blocks[i].move_type;
            let block_prepare_stage = self.blocks[i].flags.prepare_stage;
            let block_role = self.blocks[i].role;
            let block_layer_id = self.blocks[i].layer_id;
            let block_g1_line_id = self.blocks[i].g1_line_id;
            if !block_prepare_stage || block_move_type != EMoveType::Travel {
                self.moves_time[block_move_type as usize] += block_time;
            }
            self.roles_time[block_role as usize] += block_time;
            if block_layer_id as usize >= self.layers_time.len() {
                let curr_size = self.layers_time.len();
                self.layers_time.resize(block_layer_id as usize, 0.0);
                for j in curr_size..self.layers_time.len() {
                    self.layers_time[j] = 0.0;
                }
            }
            self.layers_time[(block_layer_id - 1) as usize] += block_time;
            // BBS
            if block_prepare_stage {
                self.prepare_time += block_time;
            }

            if !self.g1_times_cache.is_empty()
                && self.g1_times_cache.last().unwrap().id == block_g1_line_id
            {
                self.g1_times_cache.last_mut().unwrap().elapsed_time = self.time;
            } else {
                self.g1_times_cache.push(G1LinesCacheItem {
                    id: block_g1_line_id,
                    elapsed_time: self.time,
                });
            }
            // update times for remaining time to printer stop placeholders
            // std::lower_bound on stop_times by g1_line_id
            let it = self
                .stop_times
                .partition_point(|t| t.g1_line_id < block_g1_line_id);
            if it != self.stop_times.len() && self.stop_times[it].g1_line_id == block_g1_line_id {
                self.stop_times[it].elapsed_time = self.time;
            }
        }

        self.m_additional_time_buffer.clear();
        if additional_buffer_idx < additional_buffer.len() {
            self.m_additional_time_buffer
                .extend_from_slice(&additional_buffer[additional_buffer_idx..]);
        }

        if keep_last_n_blocks != 0 {
            self.blocks.drain(0..n_blocks_process);
        } else {
            self.blocks.clear();
        }
    }
}

// ===========================================================================
// UsedFilaments — hpp:702-742, cpp:1330-1484
// ===========================================================================
//
// Cache structures + the pure cache methods are ported faithfully. The methods
// that read processor state (`process_*_cache(GCodeProcessor*)` reading
// `get_filament_id()`, `m_extrusion_role`, `m_result.filament_diameters`, ...)
// are blocked behind the full processor; their logic is reproduced here as
// free functions taking the needed values explicitly so it is testable and
// reusable by the (blocked) driver.

/// GCodeProcessor.hpp:702-742  struct UsedFilaments (filaments per ColorChange)
#[derive(Debug, Clone, Default)]
pub struct UsedFilaments {
    pub color_change_cache: f64,
    pub volumes_per_color_change: Vec<f64>,

    pub model_extrude_cache: f64,
    pub model_volumes_per_filament: BTreeMap<usize, f64>,

    pub wipe_tower_cache: f64,
    pub wipe_tower_volumes_per_filament: BTreeMap<usize, f64>,

    pub support_volume_cache: f64,
    pub support_volumes_per_filament: BTreeMap<usize, f64>,

    /// BBS: the flush amount of every filament
    pub flush_per_filament: BTreeMap<usize, f64>,

    pub total_volume_cache: f64,
    pub total_volumes_per_filament: BTreeMap<usize, f64>,

    pub role_cache: f64,
    /// keyed by `ExtrusionRole as u8` (see `RoleKey`).
    pub filaments_per_role: BTreeMap<RoleKey, (f64, f64)>,
}

impl UsedFilaments {
    /// GCodeProcessor.cpp:1330-1351  void UsedFilaments::reset()
    pub fn reset(&mut self) {
        self.color_change_cache = 0.0;
        self.volumes_per_color_change = Vec::new();

        self.model_extrude_cache = 0.0;
        self.model_volumes_per_filament.clear();

        self.flush_per_filament.clear();

        self.role_cache = 0.0;
        self.filaments_per_role.clear();

        self.wipe_tower_cache = 0.0;
        self.wipe_tower_volumes_per_filament.clear();

        self.support_volume_cache = 0.0;
        self.support_volumes_per_filament.clear();

        self.total_volume_cache = 0.0;
        self.total_volumes_per_filament.clear();
    }

    /// GCodeProcessor.cpp:1353-1358  increase_support_caches
    pub fn increase_support_caches(&mut self, extruded_volume: f64) {
        self.support_volume_cache += extruded_volume;
        self.role_cache += extruded_volume;
        self.total_volume_cache += extruded_volume;
    }

    /// GCodeProcessor.cpp:1360-1366  increase_model_caches
    pub fn increase_model_caches(&mut self, extruded_volume: f64) {
        self.color_change_cache += extruded_volume;
        self.model_extrude_cache += extruded_volume;
        self.role_cache += extruded_volume;
        self.total_volume_cache += extruded_volume;
    }

    /// GCodeProcessor.cpp:1368-1373  increase_wipe_tower_caches
    pub fn increase_wipe_tower_caches(&mut self, extruded_volume: f64) {
        self.wipe_tower_cache += extruded_volume;
        self.role_cache += extruded_volume;
        self.total_volume_cache += extruded_volume;
    }

    /// GCodeProcessor.cpp:1375-1381  process_color_change_cache
    pub fn process_color_change_cache(&mut self) {
        if self.color_change_cache != 0.0 {
            self.volumes_per_color_change.push(self.color_change_cache);
            self.color_change_cache = 0.0;
        }
    }

    /// GCodeProcessor.cpp:1384-1396  process_total_volume_cache
    ///
    /// `active_filament_id` is `processor->get_filament_id()`.
    pub fn process_total_volume_cache(&mut self, active_filament_id: i32) {
        if self.total_volume_cache != 0.0 {
            if active_filament_id != -1 {
                *self
                    .total_volumes_per_filament
                    .entry(active_filament_id as usize)
                    .or_insert(0.0) += self.total_volume_cache;
            }
            self.total_volume_cache = 0.0;
        }
    }

    /// GCodeProcessor.cpp:1398-1410  process_model_cache
    pub fn process_model_cache(&mut self, active_filament_id: i32) {
        if self.model_extrude_cache != 0.0 {
            if active_filament_id != -1 {
                *self
                    .model_volumes_per_filament
                    .entry(active_filament_id as usize)
                    .or_insert(0.0) += self.model_extrude_cache;
            }
            self.model_extrude_cache = 0.0;
        }
    }

    /// GCodeProcessor.cpp:1412-1424  process_wipe_tower_cache
    pub fn process_wipe_tower_cache(&mut self, active_filament_id: i32) {
        if self.wipe_tower_cache != 0.0 {
            if active_filament_id != -1 {
                *self
                    .wipe_tower_volumes_per_filament
                    .entry(active_filament_id as usize)
                    .or_insert(0.0) += self.wipe_tower_cache;
            }
            self.wipe_tower_cache = 0.0;
        }
    }

    /// GCodeProcessor.cpp:1426-1438  process_support_cache
    ///
    /// `active_filament_id` is `processor->get_filament_id(false)`.
    pub fn process_support_cache(&mut self, active_filament_id: i32) {
        if self.support_volume_cache != 0.0 {
            if active_filament_id != -1 {
                *self
                    .support_volumes_per_filament
                    .entry(active_filament_id as usize)
                    .or_insert(0.0) += self.support_volume_cache;
            }
            self.support_volume_cache = 0.0;
        }
    }

    /// GCodeProcessor.cpp:1440-1454  update_flush_per_filament
    pub fn update_flush_per_filament(&mut self, filament_id: usize, flush_volume: f32) {
        if flush_volume != 0.0 {
            self.role_cache += flush_volume as f64;
            *self.flush_per_filament.entry(filament_id).or_insert(0.0) += flush_volume as f64;
            *self
                .total_volumes_per_filament
                .entry(filament_id)
                .or_insert(0.0) += flush_volume as f64;
        }
    }

    /// GCodeProcessor.cpp:1456-1474  process_role_cache
    ///
    /// `filament_diameter` is `m_result.filament_diameters[get_filament_id()]`,
    /// `filament_density` is `m_result.filament_densities[get_filament_id()]`,
    /// `active_role` is `m_extrusion_role`.
    pub fn process_role_cache(
        &mut self,
        filament_diameter: f32,
        filament_density: f32,
        active_role: ExtrusionRole,
    ) {
        if self.role_cache != 0.0 {
            let mut filament: (f64, f64) = (0.0, 0.0);

            // double s = PI * sqr(0.5 * filament_diameter);
            // GCodeProcessor.cpp:1512: `0.5` is a double literal, so the float diameter is
            // promoted and the half and its square are taken in double (R953 gate
            // PROCESSOR_ROLE_CACHE_F64, faithful; =0 gives the earlier float arm, which differs
            // for a diameter whose half squared is not exact in float, e.g. 2.85f).
            let s = if crate::faithful_gate("PROCESSOR_ROLE_CACHE_F64") {
                let half = 0.5 * filament_diameter as f64;
                PI * (half * half)
            } else {
                PI * sqr(0.5 * filament_diameter) as f64
            };
            filament.0 = self.role_cache / s * 0.001;
            filament.1 = self.role_cache * filament_density as f64 * 0.001;

            match self.filaments_per_role.get_mut(&role_key(active_role)) {
                Some(entry) => {
                    entry.0 += filament.0;
                    entry.1 += filament.1;
                }
                None => {
                    self.filaments_per_role.insert(role_key(active_role), filament);
                }
            }
            self.role_cache = 0.0;
        }
    }
}

// ===========================================================================
// GCodeProcessor::ETags / CustomETags — hpp:445-484, cpp:48-83
// ===========================================================================

impl UsedFilaments {
    /// GCodeProcessor.cpp:1526-1534 `void GCodeProcessor::UsedFilaments::process_caches(GCodeProcessor*
    /// processor)`. What the C++ reads from the processor is passed in: the active filament (forced
    /// and not), the diameter and density of the active filament, the current extrusion role.
    pub fn process_caches(
        &mut self,
        active_filament_id: i32,
        active_filament_id_support: i32,
        diameter: f32,
        density: f32,
        role: ExtrusionRole,
    ) {
        self.process_color_change_cache();
        self.process_model_cache(active_filament_id);
        self.process_role_cache(diameter, density, role);
        self.process_wipe_tower_cache(active_filament_id);
        self.process_support_cache(active_filament_id_support);
        self.process_total_volume_cache(active_filament_id);
    }
}

/// GCodeProcessor.hpp:445-470  enum class ETags : unsigned char
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ETags {
    Role,
    WipeStart,
    WipeEnd,
    Height,
    Width,
    LayerChange,
    ColorChange,
    PausePrint,
    CustomCode,
    FirstLineM73Placeholder,
    LastLineM73Placeholder,
    EstimatedPrintingTimePlaceholder,
    TotalLayerNumberPlaceholder,
    WipeTowerStart,
    WipeTowerEnd,
    UsedFilamentWeightPlaceholder,
    UsedFilamentVolumePlaceholder,
    UsedFilamentLengthPlaceholder,
    MachineStartGCodeEnd,
    MachineEndGCodeStart,
    NozzleChangeStart,
    NozzleChangeEnd,
    CpToolchangeWipe,
}

/// GCodeProcessor.hpp:472-481  enum class CustomETags : unsigned char
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CustomETags {
    FlushStart,
    FlushEnd,
    VflushStart,
    VflushEnd,
    SkippableStart,
    SkippableEnd,
    SkippableType,
}

/// GCodeProcessor.cpp:48-72  GCodeProcessor::ReservedTags
pub const RESERVED_TAGS: [&str; 23] = [
    " FEATURE: ",
    " WIPE_START",
    " WIPE_END",
    " LAYER_HEIGHT: ",
    " LINE_WIDTH: ",
    " CHANGE_LAYER",
    " COLOR_CHANGE",
    " PAUSE_PRINTING",
    " CUSTOM_GCODE",
    "_GP_FIRST_LINE_M73_PLACEHOLDER",
    "_GP_LAST_LINE_M73_PLACEHOLDER",
    "_GP_ESTIMATED_PRINTING_TIME_PLACEHOLDER",
    "_GP_TOTAL_LAYER_NUMBER_PLACEHOLDER",
    " WIPE_TOWER_START",
    " WIPE_TOWER_END",
    "_GP_FILAMENT_USED_WEIGHT_PLACEHOLDER",
    "_GP_FILAMENT_USED_VOLUME_PLACEHOLDER",
    "_GP_FILAMENT_USED_LENGTH_PLACEHOLDER",
    " MACHINE_START_GCODE_END",
    " MACHINE_END_GCODE_START",
    " NOZZLE_CHANGE_START",
    " NOZZLE_CHANGE_END",
    " CP_TOOLCHANGE_WIPE",
];

/// GCodeProcessor.cpp:74-82  GCodeProcessor::CustomTags
pub const CUSTOM_TAGS: [&str; 7] = [
    " FLUSH_START",
    " FLUSH_END",
    " VFLUSH_START",
    " VFLUSH_END",
    " SKIPPABLE_START",
    " SKIPPABLE_END",
    " SKIPTYPE: ",
];

/// GCodeProcessor.hpp:483  reserved_tag(ETags)
pub fn reserved_tag(tag: ETags) -> &'static str {
    RESERVED_TAGS[tag as usize]
}

/// GCodeProcessor.hpp:484  custom_tags(CustomETags)
pub fn custom_tags(tag: CustomETags) -> &'static str {
    CUSTOM_TAGS[tag as usize]
}

/// GCodeProcessor.cpp:85  const float GCodeProcessor::Wipe_Width = 0.05f;
pub const WIPE_WIDTH: f32 = 0.05;
/// GCodeProcessor.cpp:86  const float GCodeProcessor::Wipe_Height = 0.05f;
pub const WIPE_HEIGHT: f32 = 0.05;

/// GCodeProcessor.cpp:88  bool GCodeProcessor::s_IsBBLPrinter = true;
pub static mut S_IS_BBL_PRINTER: bool = true;

// ===========================================================================
// Static reserved-tag scanners — GCodeProcessor.cpp:1575-1625
// ===========================================================================

/// GCodeProcessor.cpp:1575-1596  contains_reserved_tag
///
/// Returns `Some(found_tag)` if any reserved tag is found, else `None`.
pub fn contains_reserved_tag(gcode: &str) -> Option<String> {
    let mut ret: Option<String> = None;

    let mut parser = GCodeReader::new();
    parser.parse_buffer(gcode, |parser, line| {
        // std::string comment = line.raw();
        let comment_full = line.raw();
        // if (comment.length() > 2 && comment.front() == ';')
        if comment_full.len() > 2 && comment_full.as_bytes()[0] == b';' {
            // comment = comment.substr(1);
            let comment = &comment_full[1..];
            for s in RESERVED_TAGS.iter() {
                if comment.starts_with(s) {
                    ret = Some(comment.to_string());
                    parser.quit_parsing();
                    return;
                }
            }
        }
    });

    ret
}

/// GCodeProcessor.cpp:1598-1625  contains_reserved_tags
///
/// Pushes found tags into the returned vec (up to `max_count`); returns `true`
/// when any tag was found.
pub fn contains_reserved_tags(gcode: &str, max_count: u32, found_tag: &mut Vec<String>) -> bool {
    // max_count = std::max(max_count, 1U);
    let max_count = max_count.max(1);

    let mut ret = false;

    // CNumericLocalesSetter locales_setter; — locale is fixed-C in Rust parsing.

    let mut parser = GCodeReader::new();
    parser.parse_buffer(gcode, |parser, line| {
        let comment_full = line.raw();
        if comment_full.len() > 2 && comment_full.as_bytes()[0] == b';' {
            let comment = &comment_full[1..];
            for s in RESERVED_TAGS.iter() {
                if comment.starts_with(s) {
                    ret = true;
                    found_tag.push(comment.to_string());
                    if found_tag.len() == max_count as usize {
                        parser.quit_parsing();
                        return;
                    }
                }
            }
        }
    });

    ret
}

// ===========================================================================
// Static gcode queries — GCodeProcessor.cpp:3018-3222
// ===========================================================================

/// GCodeProcessor.cpp:3018-3050  get_gcode_last_filament
pub fn get_gcode_last_filament(gcode_str: &str) -> i32 {
    let bytes = gcode_str.as_bytes();
    let str_size = bytes.len() as i64;
    let mut start_index: i64 = 0;
    let mut end_index: i64 = 0;
    let mut out_filament: i32 = -1;
    while end_index < str_size {
        if bytes[end_index as usize] != b'\n' {
            end_index += 1;
            continue;
        }

        if end_index > start_index {
            // std::string line_str = gcode_str.substr(start_index, end_index - start_index);
            let mut line_str = &gcode_str[start_index as usize..end_index as usize];
            // line_str.erase(0, line_str.find_first_not_of(" "));
            // line_str.erase(line_str.find_last_not_of(" ") + 1);
            line_str = trim_spaces(line_str);
            // if (line_str.empty() || line_str[0] != 'T')
            if line_str.is_empty() || line_str.as_bytes()[0] != b'T' {
                start_index = end_index + 1;
                end_index = start_index;
                continue;
            }

            // if (parse_number(line_str.substr(1), out) && out >= 0 && out < 255)
            if let Some(out) = parse_number_i32(&line_str[1..]) {
                if out >= 0 && out < 255 {
                    out_filament = out;
                }
            }
        }

        start_index = end_index + 1;
        end_index = start_index;
    }

    out_filament
}

/// `std::string::erase(0, find_first_not_of(" "))` then
/// `erase(find_last_not_of(" ") + 1)` — trim leading/trailing ASCII spaces only.
fn trim_spaces(s: &str) -> &str {
    s.trim_matches(' ')
}

/// BBS: get last z position from gcode
/// GCodeProcessor.cpp:3052-3105  get_last_z_from_gcode
///
/// Returns `Some(z)` if a Z value was parsed, else `None`. Faithfully preserves
/// the C++ `char* end = c + sizeof(z_sub.c_str())` quirk: `sizeof` on a `const
/// char*` is the pointer size (8 on 64-bit), so parsing is bounded to at most 8
/// bytes past the start of the substring.
pub fn get_last_z_from_gcode(gcode_str: &str) -> Option<f64> {
    let bytes = gcode_str.as_bytes();
    let str_size = bytes.len() as i64;
    let mut start_index: i64 = 0;
    let mut end_index: i64 = 0;
    let mut is_z_changed = false;
    let mut z: f64 = 0.0;
    while end_index < str_size {
        // find a full line
        if bytes[end_index as usize] != b'\n' {
            end_index += 1;
            continue;
        }
        // parse the line
        if end_index > start_index {
            let raw = &gcode_str[start_index as usize..end_index as usize];
            // erase leading " ", trailing ";", trailing " "
            let line_str = trim_leading_spaces_trailing_semis_spaces(raw);

            // command which may have z movement
            if line_str.len() > 4
                && (line_str.starts_with("G0 ")
                    || line_str.starts_with("G1 ")
                    || line_str.starts_with("G2 ")
                    || line_str.starts_with("G3 "))
            {
                // auto z_pos = line_str.find(" Z");
                if let Some(z_pos) = line_str.find(" Z") {
                    if z_pos + 2 < line_str.len() {
                        // std::string z_sub = line_str.substr(z_pos + 2);
                        let z_sub = &line_str[z_pos + 2..];
                        // char* c = &z_sub[0]; char* end = c + sizeof(z_sub.c_str());
                        // sizeof(const char*) == 8 on 64-bit => bound to 8 bytes.
                        // double temp_z; fast_float::from_chars(c, end, temp_z);
                        if let Some((temp_z, consumed)) = fast_float_from_chars_f64(z_sub, 8) {
                            // if (pend != c && is_end_of_word(*pend))
                            if consumed > 0 && is_end_of_word(z_sub.as_bytes().get(consumed).copied())
                            {
                                z = temp_z;
                                is_z_changed = true;
                            }
                        }
                    }
                }
            }
        }
        // loop to handle next line
        start_index = end_index + 1;
        end_index = start_index;
    }
    if is_z_changed {
        Some(z)
    } else {
        None
    }
}

/// GCodeProcessor.cpp:3264-3379  `static bool get_last_position_from_gcode(const std::string
/// &gcode_str, Vec3f &pos)`: each axis of `pos` the moves of the text set is overwritten with the
/// last value; true when one was. Preserves the `sizeof(const char*)` 8-byte parse bound (see
/// `get_last_z_from_gcode`).
pub fn get_last_position_from_gcode(gcode_str: &str, pos: &mut [f32; 3]) -> bool {
    // auto parse_G387 = [](const std::string& line_str) -> int
    fn parse_g387(line_str: &str) -> i32 {
        if !line_str.starts_with("G387 ") {
            return 0;
        }
        if line_str.contains("J1") {
            -1 // min
        } else if line_str.contains("J-1") {
            1 // max
        } else {
            0
        }
    }

    let bytes = gcode_str.as_bytes();
    let str_size = bytes.len() as i64;
    let mut start_index: i64 = 0;
    let mut end_index: i64 = 0;
    let mut is_z_changed = false;
    let mut pre_pos: [f32; 3] = [0.0, 0.0, 0.0];
    let mut pre_pos_valid: [i32; 3] = [0, 0, 0];
    while end_index < str_size {
        if bytes[end_index as usize] != b'\n' {
            end_index += 1;
            continue;
        }
        if end_index > start_index {
            let raw = &gcode_str[start_index as usize..end_index as usize];
            let line_str = trim_leading_spaces_trailing_semis_spaces(raw);

            if line_str.len() > 5
                && (line_str.starts_with("G0 ")
                    || line_str.starts_with("G1 ")
                    || line_str.starts_with("G2 ")
                    || line_str.starts_with("G3 ")
                    || line_str.starts_with("G387 "))
            {
                let g387_j = parse_g387(line_str);
                // X
                {
                    if let Some(z_pos) = line_str.find(" X") {
                        if z_pos + 2 < line_str.len() {
                            let z_sub = &line_str[z_pos + 2..];
                            if let Some((temp_z, consumed)) = fast_float_from_chars(z_sub, 8) {
                                if consumed > 0
                                    && is_end_of_word(z_sub.as_bytes().get(consumed).copied())
                                {
                                    let mut x = temp_z;
                                    is_z_changed = true;
                                    if g387_j != 0 && pre_pos_valid[0] != 0 {
                                        x = if g387_j == -1 {
                                            pre_pos[0].min(x)
                                        } else {
                                            pre_pos[0].max(x)
                                        };
                                    }
                                    pos[0] = x;
                                    pre_pos[0] = x;
                                    pre_pos_valid[0] = 1;
                                }
                            }
                        }
                    }
                }
                // Y
                {
                    if let Some(z_pos) = line_str.find(" Y") {
                        if z_pos + 2 < line_str.len() {
                            let z_sub = &line_str[z_pos + 2..];
                            if let Some((temp_z, consumed)) = fast_float_from_chars(z_sub, 8) {
                                if consumed > 0
                                    && is_end_of_word(z_sub.as_bytes().get(consumed).copied())
                                {
                                    let mut y = temp_z;
                                    is_z_changed = true;
                                    if g387_j != 0 && pre_pos_valid[1] != 0 {
                                        y = if g387_j == -1 {
                                            pre_pos[1].min(y)
                                        } else {
                                            pre_pos[1].max(y)
                                        };
                                    }
                                    pos[1] = y;
                                    pre_pos[1] = y;
                                    pre_pos_valid[1] = 1;
                                }
                            }
                        }
                    }
                }
                // Z
                {
                    if let Some(z_pos) = line_str.find(" Z") {
                        if z_pos + 2 < line_str.len() {
                            let z_sub = &line_str[z_pos + 2..];
                            if let Some((temp_z, consumed)) = fast_float_from_chars(z_sub, 8) {
                                if consumed > 0
                                    && is_end_of_word(z_sub.as_bytes().get(consumed).copied())
                                {
                                    let mut zz = temp_z;
                                    is_z_changed = true;
                                    if g387_j != 0 && pre_pos_valid[2] != 0 {
                                        zz = if g387_j == -1 {
                                            pre_pos[2].min(zz)
                                        } else {
                                            pre_pos[2].max(zz)
                                        };
                                    }
                                    pos[2] = zz;
                                    pre_pos[2] = zz;
                                    pre_pos_valid[2] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        start_index = end_index + 1;
        end_index = start_index;
    }
    is_z_changed
}

/// `is_end_of_word` lambda: `c == ' ' || '\t' || '\r' || '\n' || 0 || ';'`.
/// `None` represents the C string's NUL terminator (`*pend == 0`).
fn is_end_of_word(c: Option<u8>) -> bool {
    match c {
        None => true, // past end == NUL terminator (0)
        Some(b) => b == b' ' || b == b'\t' || b == b'\r' || b == b'\n' || b == 0 || b == b';',
    }
}

/// Faithful stand-in for `fast_float::from_chars(c, c+max_len, out)` over an
/// f32: parse the longest float prefix within the first `max_len` bytes of
/// `s`. Returns `(value, consumed_bytes)`; `consumed == 0` means no parse
/// (`pend == c`).
fn fast_float_from_chars(s: &str, max_len: usize) -> Option<(f32, usize)> {
    let bytes = s.as_bytes();
    let limit = bytes.len().min(max_len);
    let mut end = 0usize;
    let mut seen_digit = false;
    let mut seen_dot = false;
    let mut seen_e = false;
    if end < limit && (bytes[end] == b'+' || bytes[end] == b'-') {
        end += 1;
    }
    while end < limit {
        let c = bytes[end];
        if c.is_ascii_digit() {
            seen_digit = true;
            end += 1;
        } else if c == b'.' && !seen_dot && !seen_e {
            seen_dot = true;
            end += 1;
        } else if (c == b'e' || c == b'E') && seen_digit && !seen_e {
            seen_e = true;
            end += 1;
            if end < limit && (bytes[end] == b'+' || bytes[end] == b'-') {
                end += 1;
            }
        } else {
            break;
        }
    }
    if !seen_digit {
        return Some((0.0, 0)); // pend == c
    }
    match s[..end].parse::<f32>() {
        Ok(v) => Some((v, end)),
        Err(_) => Some((0.0, 0)),
    }
}

/// f64 variant of `fast_float_from_chars` for `double temp_z` (used by
/// `get_last_z_from_gcode`, whose `z` is a `double&`).
fn fast_float_from_chars_f64(s: &str, max_len: usize) -> Option<(f64, usize)> {
    let bytes = s.as_bytes();
    let limit = bytes.len().min(max_len);
    let mut end = 0usize;
    let mut seen_digit = false;
    let mut seen_dot = false;
    let mut seen_e = false;
    if end < limit && (bytes[end] == b'+' || bytes[end] == b'-') {
        end += 1;
    }
    while end < limit {
        let c = bytes[end];
        if c.is_ascii_digit() {
            seen_digit = true;
            end += 1;
        } else if c == b'.' && !seen_dot && !seen_e {
            seen_dot = true;
            end += 1;
        } else if (c == b'e' || c == b'E') && seen_digit && !seen_e {
            seen_e = true;
            end += 1;
            if end < limit && (bytes[end] == b'+' || bytes[end] == b'-') {
                end += 1;
            }
        } else {
            break;
        }
    }
    if !seen_digit {
        return Some((0.0, 0)); // pend == c
    }
    match s[..end].parse::<f64>() {
        Ok(v) => Some((v, end)),
        Err(_) => Some((0.0, 0)),
    }
}

/// C++: `line_str.erase(0, find_first_not_of(" "))` (leading spaces),
/// then `erase(find_last_not_of(";") + 1)` (trailing ';'),
/// then `erase(find_last_not_of(" ") + 1)` (trailing spaces). Applied in order.
fn trim_leading_spaces_trailing_semis_spaces(s: &str) -> &str {
    let s = s.trim_start_matches(' ');
    let s = s.trim_end_matches(';');
    s.trim_end_matches(' ')
}

// ===========================================================================
// EMovePathType — ArcFitter.hpp:9-16
// ===========================================================================
/// ArcFitter.hpp:9-16  enum class EMovePathType : unsigned char
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EMovePathType {
    NoopMove,    // Noop_move
    LinearMove,  // Linear_move
    ArcMoveCw,   // Arc_move_cw
    ArcMoveCcw,  // Arc_move_ccw
    // Count
}

impl Default for EMovePathType {
    fn default() -> Self {
        EMovePathType::NoopMove
    }
}

// ===========================================================================
// GCodeProcessorResult::MoveVertex — GCodeProcessor.hpp:195-233
// ===========================================================================
/// GCodeProcessor.hpp:195-233  struct MoveVertex
///
/// C++ uses `Vec3f` (Eigen) for position fields; here a plain `[f32; 3]` is
/// used (faithful value semantics, no nalgebra dependency required).
#[derive(Debug, Clone)]
pub struct MoveVertex {
    pub move_type: EMoveType, // type
    pub extrusion_role: ExtrusionRole,
    pub move_path_type: EMovePathType,
    pub extruder_id: u8,
    pub cp_color_id: u8,

    pub gcode_id: u32,
    pub delta_extruder: f32, // mm
    pub feedrate: f32,       // mm/s
    pub width: f32,          // mm
    pub height: f32,         // mm
    pub mm3_per_mm: f32,
    pub fan_speed: f32,    // percentage
    /// GCodeProcessor.hpp:221 `float additional_fan_speed{ 0.0f };` (auxiliary/side fan, M106 P2)
    pub additional_fan_speed: f32,
    pub temperature: f32,  // Celsius degrees
    pub layer_duration: f32, // s (layer id before finalize)
    pub thermal_index_min: f32,
    pub thermal_index_max: f32,
    pub thermal_index_mean: f32,

    /// prefix sum of time, assigned during finalize() — one per ETimeMode.
    pub time: [f32; 2],

    pub position: [f32; 3],            // mm
    pub arc_center_position: [f32; 3], // mm
    pub interpolation_points: Vec<[f32; 3]>,
    pub object_label_id: i32,
    pub print_z: f32,
}

impl Default for MoveVertex {
    fn default() -> Self {
        MoveVertex {
            move_type: EMoveType::Noop,
            extrusion_role: ExtrusionRole::None,
            move_path_type: EMovePathType::NoopMove,
            extruder_id: 0,
            cp_color_id: 0,
            gcode_id: 0,
            delta_extruder: 0.0,
            feedrate: 0.0,
            width: 0.0,
            height: 0.0,
            mm3_per_mm: 0.0,
            fan_speed: 0.0,
            additional_fan_speed: 0.0,
            temperature: 0.0,
            layer_duration: 0.0,
            thermal_index_min: 0.0,
            thermal_index_max: 0.0,
            thermal_index_mean: 0.0,
            time: [0.0, 0.0],
            position: [0.0, 0.0, 0.0],
            arc_center_position: [0.0, 0.0, 0.0],
            interpolation_points: Vec::new(),
            object_label_id: -1,
            print_z: 0.0,
        }
    }
}

impl MoveVertex {
    /// GCodeProcessor.hpp:225  float volumetric_rate() const
    pub fn volumetric_rate(&self) -> f32 {
        self.feedrate * self.mm3_per_mm
    }

    /// GCodeProcessor.hpp:239-241  bool is_arc_move_with_interpolation_points() const
    pub fn is_arc_move_with_interpolation_points(&self) -> bool {
        (self.move_path_type == EMovePathType::ArcMoveCcw || self.move_path_type == EMovePathType::ArcMoveCw)
            && !self.interpolation_points.is_empty()
    }

    /// GCodeProcessor.hpp:230-232  bool is_arc_move() const
    pub fn is_arc_move(&self) -> bool {
        self.move_path_type == EMovePathType::ArcMoveCcw
            || self.move_path_type == EMovePathType::ArcMoveCw
    }
}

// ===========================================================================
// GCodeProcessorResult — GCodeProcessor.hpp:181-355
// ===========================================================================
/// GCodeProcessor.hpp:181-355  struct GCodeProcessorResult
///
/// Only the fields used by the time / filament-usage parity path are modeled.
/// Fields that require unported deps (NozzleType,
/// MultiNozzleUtils, mutex, etc.) are omitted; see the BLOCKED notes in
/// `finalize`/`apply_config`. The `print_time` / `filament_used_mm` /
/// `filament_used_g` / `filament_used_mm3` accessors are thin parity-facing
/// summaries (not C++ Result fields) that the caller reads.
#[derive(Debug, Clone)]
pub struct GCodeProcessorResult {
    pub filename: String,
    pub id: u32,
    pub moves: Vec<MoveVertex>,
    pub lines_ends: Vec<usize>,
    pub toolpath_outside: bool,
    pub label_object_enabled: bool,
    pub long_retraction_when_cut: bool,
    pub is_helio_gcode: bool,
    pub timelapse_warning_code: i32,
    /// GCodeProcessor.hpp:287 `bool support_traditional_timelapse{true};`
    pub support_traditional_timelapse: bool,
    pub printable_height: f32,
    // GCodeProcessor.hpp:273-276. XY coordinates are unscaled doubles.
    pub printable_area: Vec<(f64, f64)>,
    pub bed_exclude_area: Vec<(f64, f64)>,
    pub wrapping_exclude_area: Vec<(f64, f64)>,
    // GCodeProcessor.hpp:295-297. Machine-limit rows are keyed by extruder and nozzle variant.
    pub extruder_types: Vec<crate::extruder::ExtruderType>,
    pub printer_extruder_id: Vec<i32>,
    pub printer_extruder_variant: Vec<String>,
    pub settings_ids: SettingsIds,
    pub filaments_count: usize,
    pub extruder_colors: Vec<String>,
    pub filament_diameters: Vec<f32>,
    pub required_nozzle_hrc: Vec<i32>,
    pub filament_densities: Vec<f32>,
    pub filament_costs: Vec<f32>,
    pub filament_vitrification_temperature: Vec<i32>,
    pub print_statistics: PrintEstimatedStatistics,
    pub spiral_vase_layers: Vec<(f32, (usize, usize))>,
    pub warnings: Vec<SliceWarning>,
    pub skippable_part_time: HashMap<SkipType, f32>,
    pub used_filaments: Vec<FilamentUseInfo>,
    pub initial_layer_time: f32,

    // --- parity-facing summary fields (filled by finalize) ---
    /// Total estimated print time (s) — `print_statistics.modes[Normal].time`.
    pub print_time: f32,
    /// Total filament used (mm of filament).
    pub filament_used_mm: f64,
    /// Total filament used (g).
    pub filament_used_g: f64,
    /// Total filament volume used (mm^3).
    pub filament_used_mm3: f64,
}

impl Default for GCodeProcessorResult {
    fn default() -> Self {
        let mut r = GCodeProcessorResult {
            filename: String::new(),
            id: 0,
            moves: Vec::new(),
            lines_ends: Vec::new(),
            toolpath_outside: false,
            label_object_enabled: false,
            long_retraction_when_cut: false,
            is_helio_gcode: false,
            timelapse_warning_code: 0,
            support_traditional_timelapse: true,
            printable_height: 0.0,
            printable_area: Vec::new(),
            bed_exclude_area: Vec::new(),
            wrapping_exclude_area: Vec::new(),
            extruder_types: Vec::new(),
            printer_extruder_id: Vec::new(),
            printer_extruder_variant: Vec::new(),
            settings_ids: SettingsIds::default(),
            filaments_count: 0,
            extruder_colors: Vec::new(),
            filament_diameters: Vec::new(),
            required_nozzle_hrc: Vec::new(),
            filament_densities: Vec::new(),
            filament_costs: Vec::new(),
            filament_vitrification_temperature: Vec::new(),
            print_statistics: PrintEstimatedStatistics::new(),
            spiral_vase_layers: Vec::new(),
            warnings: Vec::new(),
            skippable_part_time: HashMap::new(),
            used_filaments: Vec::new(),
            initial_layer_time: 0.0,
            print_time: 0.0,
            filament_used_mm: 0.0,
            filament_used_g: 0.0,
            filament_used_mm3: 0.0,
        };
        r.reset();
        r
    }
}

impl GCodeProcessorResult {
    /// GCodeProcessor.cpp:1516-1555  void GCodeProcessorResult::reset()
    /// (non-`ENABLE_GCODE_VIEWER_STATISTICS` branch)
    pub fn reset(&mut self) {
        self.moves.clear();
        self.lines_ends.clear();
        self.toolpath_outside = false;
        self.is_helio_gcode = false;
        self.label_object_enabled = false;
        self.long_retraction_when_cut = false;
        self.timelapse_warning_code = 0;
        // GCodeProcessor.cpp:1589
        self.support_traditional_timelapse = true;
        // GCodeProcessor.cpp:1577-1590.
        self.printable_area.clear();
        self.bed_exclude_area.clear();
        self.wrapping_exclude_area.clear();
        self.printable_height = 0.0;
        self.settings_ids.reset();
        self.filaments_count = 0;
        self.extruder_colors = Vec::new();
        self.filament_diameters = vec![DEFAULT_FILAMENT_DIAMETER; MIN_EXTRUDERS_COUNT];
        self.required_nozzle_hrc = vec![DEFAULT_FILAMENT_HRC; MIN_EXTRUDERS_COUNT];
        self.filament_densities = vec![DEFAULT_FILAMENT_DENSITY; MIN_EXTRUDERS_COUNT];
        self.filament_costs = vec![DEFAULT_FILAMENT_COST; MIN_EXTRUDERS_COUNT];
        // GCodeProcessor.cpp:1594-1597 resets the other four material vectors here.
        // apply_config replaces the vitrification temperatures for the next print.
        self.spiral_vase_layers = Vec::new();
        self.skippable_part_time.clear();
        self.warnings.clear();
    }
}

// ===========================================================================
// MachineEnvelopeConfig — PrintConfig.hpp machine_max_* / machine_min_*
// ===========================================================================
/// `MachineEnvelopeConfig` (PrintConfig.hpp): the machine motion limits `TimeProcessor` reads, each
/// a `ConfigOptionFloats(Nullable)` indexed `extruder_id * 2 + mode` (speeds, accelerations) or
/// `mode` (jerks, minimum rates); `get_option_value` falls back to the last value past the end and
/// to 0 on an empty array. The values are kept as the floats `get_option_value` reads them as.
#[derive(Debug, Clone, Default)]
pub struct MachineLimits {
    pub machine_max_acceleration_x: Vec<f32>,
    pub machine_max_acceleration_y: Vec<f32>,
    pub machine_max_acceleration_z: Vec<f32>,
    pub machine_max_acceleration_e: Vec<f32>,
    pub machine_max_acceleration_extruding: Vec<f32>,
    pub machine_max_acceleration_retracting: Vec<f32>,
    pub machine_max_acceleration_travel: Vec<f32>,
    pub machine_max_speed_x: Vec<f32>,
    pub machine_max_speed_y: Vec<f32>,
    pub machine_max_speed_z: Vec<f32>,
    pub machine_max_speed_e: Vec<f32>,
    pub machine_max_jerk_x: Vec<f32>,
    pub machine_max_jerk_y: Vec<f32>,
    pub machine_max_jerk_z: Vec<f32>,
    pub machine_max_jerk_e: Vec<f32>,
    pub machine_min_extruding_rate: Vec<f32>,
    pub machine_min_travel_rate: Vec<f32>,
}

impl MachineLimits {
    /// `MachineEnvelopeConfig()`: the defaults of the definitions (PrintConfig.cpp:4968-5131, per
    /// mode: normal, stealth). MACHINE_LIMITS_DEFAULTS (on); =0 gives empty arrays (no limit), as the
    /// port had before.
    pub fn machine_envelope_config() -> Self {
        if !crate::faithful_gate("MACHINE_LIMITS_DEFAULTS") {
            return MachineLimits::default();
        }
        MachineLimits {
            // name, max_feedrate,  max_acceleration, max_jerk (PrintConfig.cpp:4974-4980)
            machine_max_speed_x: vec![500.0, 200.0],
            machine_max_speed_y: vec![500.0, 200.0],
            machine_max_speed_z: vec![12.0, 12.0],
            machine_max_speed_e: vec![120.0, 120.0],
            machine_max_acceleration_x: vec![1000.0, 1000.0],
            machine_max_acceleration_y: vec![1000.0, 1000.0],
            machine_max_acceleration_z: vec![500.0, 200.0],
            machine_max_acceleration_e: vec![5000.0, 5000.0],
            machine_max_jerk_x: vec![10.0, 10.0],
            machine_max_jerk_y: vec![10.0, 10.0],
            machine_max_jerk_z: vec![0.2f64 as f32, 0.4f64 as f32],
            machine_max_jerk_e: vec![2.5, 2.5],
            // PrintConfig.cpp:5074-5130
            machine_min_extruding_rate: vec![0.0, 0.0],
            machine_min_travel_rate: vec![0.0, 0.0],
            machine_max_acceleration_extruding: vec![1500.0, 1250.0],
            machine_max_acceleration_retracting: vec![1500.0, 1250.0],
            machine_max_acceleration_travel: vec![1500.0, 1250.0],
        }
    }
}

/// GCodeProcessor.cpp:115-119 `get_option_value(const ConfigOptionFloatsNullable& option, size_t id)`
/// over the limits' floats.
fn ml_get(option: &[f32], id: usize) -> f32 {
    if option.is_empty() {
        0.0
    } else if id < option.len() {
        option[id]
    } else {
        *option.last().unwrap()
    }
}

/// GCodeProcessor.cpp:103-107 `set_option_value(ConfigOptionFloatsNullable& option, size_t id, float
/// value)` over the limits' floats.
fn ml_set(option: &mut [f32], id: usize, value: f32) {
    if id < option.len() {
        option[id] = value;
    }
}

// ===========================================================================
// GCodeProcessor::TimeProcessor — hpp:810-871, cpp:522-537
// ===========================================================================
/// GCodeProcessor.hpp:810-871  struct TimeProcessor
///
/// Only the time-relevant members are modeled. `post_process`,
/// `handle_offsets_*` and the pre-cooling machinery are blocked (unported deps).
/// See BLOCKED notes.
#[derive(Debug, Clone)]
pub struct TimeProcessor {
    pub extruder_unloaded: bool,
    pub machine_envelope_processing_enabled: bool,
    /// MachineEnvelopeConfig machine_limits (GCodeProcessor.hpp:818): the defaults of
    /// `MachineEnvelopeConfig()` (reset), the configuration's under the Marlin and
    /// Klipper flavors (`GCodeProcessor::apply_config`, cpp:2027-2033).
    pub machine_limits: MachineLimits,
    pub filament_load_times: f32,
    pub filament_unload_times: f32,
    pub extruder_change_times: f32,
    pub hotend_change_times: f32,
    /// GCodeProcessor.hpp `prepare_compensation_time`.
    pub prepare_compensation_time: f32,
    pub machines: [TimeMachine; ETimeMode::COUNT],
}

impl TimeProcessor {
    /// GCodeProcessor.hpp:826-833  Planner constants
    const PLANNER_QUEUE_SIZE: usize = 64;
    const PLANNER_REFRESH_THRESHOLD: usize = Self::PLANNER_QUEUE_SIZE * 4;

    /// GCodeProcessor.cpp:571-1300  `TimeProcessor::post_process`, the part that
    /// writes the `M73` lines: before every G1, G2, G3 and `;VG1` line of the
    /// finished G-code, `process_line_move` (:816-885, called at :1019-1032)
    /// writes the pair <percent, remaining minutes> when it changed, and the time
    /// to the next printer stop. The times are the `g1_times_cache` the planner
    /// filled; its ids come from `m_g1_line_id`, which G0 and G28 advance too,
    /// while the counter here counts G1/G2/G3/`;VG1` lines of the text only
    /// (`g1_lines_counter`, :997) — as in C++.
    ///
    /// Not ported here: the placeholders (the port writes those lines itself),
    /// the filament and extruder blocks, the pre-heating injection and the
    /// line-offset bookkeeping of the same function.
    pub fn post_process(&self, gcode: &str) -> String {
        self.post_process_first_pass(gcode, None)
    }

    /// GCodeProcessor.cpp:571-1032  the first pass of `TimeProcessor::post_process`
    /// (`gcode_time_handler` over `gcode_process`): the `M73` lines before the moves, and, when
    /// `offsets` is given, the `offsets` vector the pass fills (:1014-1015, :1028-1031): per input
    /// line id (1-based, :929-931) the number of lines inserted before it.
    pub fn post_process_first_pass(&self, gcode: &str, mut offsets: Option<&mut Vec<(u32, u32)>>) -> String {
        use crate::g_code_reader::GCodeLine;
        let machines = &self.machines;
        // :589-592
        let time_in_minutes = |time_in_seconds: f32| -> i32 { ((time_in_seconds + 0.5f32) / 60.0f32) as i32 };
        // :594-597
        let time_in_last_minute = |time_in_seconds: f32| -> f32 { time_in_seconds / 60.0f32 };
        // :599-605, :607-611
        let format_line_m73_main = |mask: &str, percent: i32, time: i32| -> String {
            mask.replacen("%s", &percent.to_string(), 1).replacen("%s", &time.to_string(), 1)
        };
        let format_line_m73_stop_int =
            |mask: &str, time: i32| -> String { mask.replacen("%s", &time.to_string(), 1) };
        // :621-629
        let format_time_float =
            |time: f32| -> String { crate::locales_utils::float_to_string_decimal_point(time as f64, 2) };
        let format_line_m73_stop_float =
            |mask: &str, time: f32| -> String { mask.replacen("%s", &format_time_float(time), 1) };

        // :645-655 — last exported pair and stop time per machine.
        let mut last_exported_main: Vec<(i32, i32)> =
            machines.iter().map(|m| (0, time_in_minutes(m.time))).collect();
        let mut last_exported_stop: Vec<i32> = machines.iter().map(|m| time_in_minutes(m.time)).collect();
        // :812-814
        let mut g1_times_cache_it: Vec<usize> = vec![0; machines.len()];
        // :997
        let mut g1_lines_counter: usize = 0;

        let mut out = String::with_capacity(gcode.len() + gcode.len() / 64);
        // :929-931 — `line_id` counts the input lines from 1.
        let mut line_id: u32 = 0;
        for gcode_line in gcode.split_inclusive('\n') {
            line_id += 1;
            if GCodeLine::cmd_is_str(gcode_line, "G1")
                || GCodeLine::cmd_is_str(gcode_line, "G2")
                || GCodeLine::cmd_is_str(gcode_line, "G3")
                || GCodeLine::cmd_start_with(gcode_line, ";VG1")
            {
                // :825 `exported_lines_count`
                let out_len_before = out.len();
                // process_line_move(gcode_buffer, g1_lines_counter++)
                for i in 0..machines.len() {
                    let machine = &machines[i];
                    if !machine.enabled {
                        continue;
                    }
                    // Skip all machine.g1_times_cache below g1_lines_counter.
                    let cache = &machine.g1_times_cache;
                    let it = &mut g1_times_cache_it[i];
                    while *it < cache.len() && (cache[*it].id as usize) < g1_lines_counter {
                        *it += 1;
                    }
                    if *it < cache.len() && cache[*it].id as usize == g1_lines_counter {
                        let elapsed = cache[*it].elapsed_time;
                        let to_export_main = (
                            (100.0f32 * elapsed / machine.time) as i32,
                            time_in_minutes(machine.time - elapsed),
                        );
                        if last_exported_main[i] != to_export_main {
                            out.push_str(&format_line_m73_main(
                                &machine.line_m73_main_mask,
                                to_export_main.0,
                                to_export_main.1,
                            ));
                            last_exported_main[i] = to_export_main;
                        }
                        // export remaining time to next printer stop (:845-879)
                        let upper_bound = |value: f32| -> usize {
                            machine.stop_times.partition_point(|t| !(value < t.elapsed_time))
                        };
                        let it_stop = upper_bound(elapsed);
                        if it_stop < machine.stop_times.len() {
                            let stop_elapsed = machine.stop_times[it_stop].elapsed_time;
                            let to_export_stop = time_in_minutes(stop_elapsed - elapsed);
                            if last_exported_stop[i] != to_export_stop {
                                if to_export_stop > 0 {
                                    out.push_str(&format_line_m73_stop_int(
                                        &machine.line_m73_stop_mask,
                                        to_export_stop,
                                    ));
                                    last_exported_stop[i] = to_export_stop;
                                } else {
                                    let mut is_last = false;
                                    let next_it = *it + 1;
                                    is_last |= next_it == cache.len();
                                    if next_it != cache.len() {
                                        let next_elapsed = cache[next_it].elapsed_time;
                                        let next_it_stop = upper_bound(next_elapsed);
                                        is_last |= next_it_stop != it_stop;
                                        let time_float_str =
                                            format_time_float(time_in_last_minute(stop_elapsed - elapsed));
                                        let next_time_float_str =
                                            format_time_float(time_in_last_minute(stop_elapsed - next_elapsed));
                                        is_last |= time_float_str.parse::<f64>().unwrap_or(0.0) > 0.0
                                            && next_time_float_str.parse::<f64>().unwrap_or(0.0) == 0.0;
                                    }
                                    if is_last {
                                        if it_stop == machine.stop_times.len() - 1 {
                                            out.push_str(&format_line_m73_stop_int(
                                                &machine.line_m73_stop_mask,
                                                to_export_stop,
                                            ));
                                        } else {
                                            out.push_str(&format_line_m73_stop_float(
                                                &machine.line_m73_stop_mask,
                                                time_in_last_minute(stop_elapsed - elapsed),
                                            ));
                                        }
                                        last_exported_stop[i] = to_export_stop;
                                    }
                                }
                            }
                        }
                    }
                }
                g1_lines_counter += 1;
                // :1028-1031 — every exported line ends in '\n' (the masks end in one).
                if let Some(offs) = offsets.as_deref_mut() {
                    let extra_lines_count = out[out_len_before..].bytes().filter(|&b| b == b'\n').count() as u32;
                    if extra_lines_count > 0 {
                        offs.push((line_id, extra_lines_count));
                    }
                }
            }
            out.push_str(gcode_line);
        }
        out
    }

    /// GCodeProcessor.cpp:522-537  void TimeProcessor::reset()
    pub fn reset(&mut self) {
        self.extruder_unloaded = true;
        self.machine_envelope_processing_enabled = false;
        // GCodeProcessor.cpp:526  machine_limits = MachineEnvelopeConfig();
        self.machine_limits = MachineLimits::machine_envelope_config();
        self.filament_load_times = 0.0;
        self.filament_unload_times = 0.0;
        self.extruder_change_times = 0.0;
        self.hotend_change_times = 0.0;

        for i in 0..ETimeMode::COUNT {
            self.machines[i].reset();
        }
        self.machines[ETimeMode::Normal as usize].enabled = true;
    }
}

impl Default for TimeProcessor {
    fn default() -> Self {
        let mut tp = TimeProcessor {
            extruder_unloaded: true,
            machine_envelope_processing_enabled: false,
            machine_limits: MachineLimits::machine_envelope_config(),
            filament_load_times: 0.0,
            filament_unload_times: 0.0,
            extruder_change_times: 0.0,
            hotend_change_times: 0.0,
            prepare_compensation_time: 0.0,
            machines: [TimeMachine::default(), TimeMachine::default()],
        };
        tp.reset();
        tp
    }
}

// ===========================================================================
// Command dispatch ids — mirrors register_commands (GCodeProcessor.cpp:1639)
// ===========================================================================
/// Handler ids registered with `CommandProcessor`. The C++ binds
/// `std::function` closures; we register stable ids and dispatch on them.
#[repr(usize)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Cmd {
    G0,
    G1,
    G2G3,
    G4,
    G10,
    G11,
    G20,
    G21,
    G22,
    G23,
    G28,
    G29,
    G90,
    G91,
    G92,
    M1,
    M82,
    M83,
    M104,
    M106,
    M107,
    M108,
    M109,
    M132,
    M135,
    M140,
    M190,
    M191,
    M201,
    M203,
    M204,
    M205,
    M221,
    M400,
    M401,
    M402,
    M566,
    M702,
    M1020,
    T,
    Sync,
    M622,
    M623,
    VM104,
    VM109,
    VG1,
}

const CMD_TABLE: &[(&str, Cmd, bool)] = &[
    ("G0", Cmd::G0, false),
    ("G1", Cmd::G1, false),
    ("G2", Cmd::G2G3, false),
    ("G3", Cmd::G2G3, false),
    ("G4", Cmd::G4, false),
    ("G10", Cmd::G10, false),
    ("G11", Cmd::G11, false),
    ("G20", Cmd::G20, false),
    ("G21", Cmd::G21, false),
    ("G22", Cmd::G22, false),
    ("G23", Cmd::G23, false),
    ("G28", Cmd::G28, false),
    ("G29", Cmd::G29, false),
    ("G90", Cmd::G90, false),
    ("G91", Cmd::G91, false),
    ("G92", Cmd::G92, false),
    ("M1", Cmd::M1, false),
    ("M82", Cmd::M82, false),
    ("M83", Cmd::M83, false),
    ("M104", Cmd::M104, false),
    ("M106", Cmd::M106, false),
    ("M107", Cmd::M107, false),
    ("M108", Cmd::M108, false),
    ("M109", Cmd::M109, false),
    ("M132", Cmd::M132, false),
    ("M135", Cmd::M135, false),
    ("M140", Cmd::M140, false),
    ("M190", Cmd::M190, false),
    ("M191", Cmd::M191, false),
    ("M201", Cmd::M201, false),
    ("M203", Cmd::M203, false),
    ("M204", Cmd::M204, false),
    ("M205", Cmd::M205, false),
    ("M221", Cmd::M221, false),
    ("M400", Cmd::M400, false),
    ("M401", Cmd::M401, false),
    ("M402", Cmd::M402, false),
    ("M566", Cmd::M566, false),
    ("M702", Cmd::M702, false),
    ("M1020", Cmd::M1020, false),
    ("T", Cmd::T, true),
    ("SYNC", Cmd::Sync, false),
    // GCodeProcessor.cpp:1762-1763 — M622 / M623 (PROCESS_G4_G29, below).
    ("M622", Cmd::M622, false),
    ("M623", Cmd::M623, false),
    // GCodeProcessor.cpp:1757-1758 — VM104 / VM109 (the virtual temperature commands of `;V...`
    // comments).
    ("VM104", Cmd::VM104, false),
    ("VM109", Cmd::VM109, false),
    // GCodeProcessor.cpp:1756 — VG1 (the virtual move of `;VG1` comments).
    ("VG1", Cmd::VG1, false),
];

fn cmd_from_id(id: usize) -> Cmd {
    // Safe: ids are produced only by register; bounded by CMD_TABLE entries.
    debug_assert!(id < CMD_TABLE.len() || id < 64);
    // Reconstruct from discriminant.
    unsafe { std::mem::transmute::<usize, Cmd>(id) }
}

// ===========================================================================
// GCodeProcessor (state-machine driver) — hpp:440-1211, cpp:1627-...
// ===========================================================================
/// GCodeFlavor subset used by the handlers. The C++ `m_flavor` defaults to
/// `gcfRepRapSprinter`; only the comparisons in the ported handlers matter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GCodeFlavor {
    RepRapSprinter,
    RepRapFirmware,
    RepetierFirmware, // gcfRepetier
    MarlinLegacy,
    MarlinFirmware,
    Klipper,
    Sailfish,
    MakerWare,
    Smoothie,
    /// gcfMach3 (set by Cura's `FLAVOR:Mach3`, `process_cura_tags`).
    Mach3,
    Other,
}

/// GCodeProcessor.hpp:440-1211  class GCodeProcessor (state machine).
///
/// Not modelled: `OptionsZCorrector` (with `custom_gcode_per_print_z`), the
/// filament lists and the pre-heating context of `post_process`, the data
/// checkers' members (ENABLE_GCODE_VIEWER_DATA_CHECKING is off at the pin).
pub struct GCodeProcessor {
    m_command_processor: CommandProcessor,
    m_units: EUnits,
    m_global_positioning_type: EPositioningType,
    m_e_local_positioning_type: EPositioningType,
    m_flavor: GCodeFlavor,

    m_start_position: AxisCoords, // mm  [X,Y,Z,E]
    m_end_position: AxisCoords,   // mm
    m_origin: AxisCoords,         // mm
    m_cached_position: CachedPosition,

    m_wiping: bool,
    m_flushing: bool,
    m_virtual_flushing: bool,
    m_wipe_tower: bool,
    m_skippable: bool,
    m_skippable_type: SkipType,
    m_object_label_id: i32,
    m_print_z: f32,
    m_remaining_volume: Vec<f32>,

    // BBS: x, y offset for gcode generated (default 0)
    m_x_offset: f64,
    m_y_offset: f64,
    // BBS: arc move related data
    m_move_path_type: EMovePathType,
    m_arc_center: [f32; 3],
    m_interpolation_points: Vec<[f32; 3]>,

    m_line_id: u32,
    m_last_line_id: u32,
    m_feedrate: f32, // mm/s
    m_width: f32,    // mm
    m_height: f32,   // mm
    m_forced_width: f32,
    m_forced_height: f32,
    m_mm3_per_mm: f32,
    m_fan_speed: f32, // percentage
    /// GCodeProcessor.hpp:1215 `float m_additional_fan_speed;` (auxiliary/side fan, M106 P2)
    m_additional_fan_speed: f32,
    m_extrusion_role: ExtrusionRole,

    m_filament_maps: Vec<i32>,
    m_physical_extruder_map: Vec<i32>,
    m_last_filament_id: Vec<u8>,
    m_filament_id: Vec<u8>,
    m_extruder_id: u8,
    m_extruder_colors: Vec<u8>,
    m_extruder_temps: Vec<f32>,
    m_thermal_index: ThermalIndex,
    m_is_helio_gcode: bool,
    m_highest_bed_temp: i32,
    m_extruded_last_z: f32,
    m_first_layer_height: f32, // mm
    m_zero_layer_height: f32,  // mm
    m_processing_start_custom_gcode: bool,
    /// GCodeProcessor.hpp `m_pa_line_calibration`.
    m_pa_line_calibration: bool,
    /// `process_tags`' function-static `prev_role` (GCodeProcessor.cpp:3383).
    m_prev_role: ExtrusionRole,
    /// GCodeProcessor.hpp `m_nozzle_volume`.
    m_nozzle_volume: Vec<f32>,
    /// GCodeProcessor.hpp `m_handle_hotend_as_extruder`.
    m_handle_hotend_as_extruder: bool,
    /// GCodeProcessor.hpp `m_nozzle_group_result`.
    m_nozzle_group_result: Option<crate::multi_nozzle_utils::LayeredNozzleGroupResult>,
    /// GCodeProcessor.hpp:1172 `m_nozzle_status_recorder`.
    m_nozzle_status_recorder: crate::multi_nozzle_utils::NozzleStatusRecorder,
    m_g1_line_id: u32,
    m_layer_id: u32,
    m_cp_color: CpColor,
    m_seams_count: i32,
    m_detect_layer_based_on_tag: bool,
    m_measure_g29_time: bool,
    /// GCodeProcessor.hpp:1206 `SeamsDetector m_seams_detector;`
    m_seams_detector: SeamsDetector,
    /// GCodeProcessor.hpp:1233 `EProducer m_producer;`
    m_producer: EProducer,
    /// GCodeProcessor.hpp m_extruder_offsets, one Vec3f per filament.
    m_extruder_offsets: Vec<[f32; 3]>,

    m_time_processor: TimeProcessor,
    m_used_filaments: UsedFilaments,

    m_result: GCodeProcessorResult,
}

impl GCodeProcessor {
    /// GCodeProcessor.cpp:1627-1637  GCodeProcessor::GCodeProcessor()
    pub fn new() -> Self {
        let mut p = GCodeProcessor {
            m_command_processor: CommandProcessor::new(),
            m_units: EUnits::Millimeters,
            m_global_positioning_type: EPositioningType::Absolute,
            m_e_local_positioning_type: EPositioningType::Absolute,
            m_flavor: GCodeFlavor::RepRapSprinter,
            m_start_position: [0.0; 4],
            m_end_position: [0.0; 4],
            m_origin: [0.0; 4],
            m_cached_position: CachedPosition::default(),
            m_wiping: false,
            m_flushing: false,
            m_virtual_flushing: false,
            m_wipe_tower: false,
            m_skippable: false,
            m_skippable_type: SkipType::StNone,
            m_object_label_id: -1,
            m_print_z: 0.0,
            m_remaining_volume: vec![0.0, 0.0],
            m_x_offset: 0.0,
            m_y_offset: 0.0,
            m_move_path_type: EMovePathType::NoopMove,
            m_arc_center: [0.0; 3],
            m_interpolation_points: Vec::new(),
            m_line_id: 0,
            m_last_line_id: 0,
            m_feedrate: 0.0,
            m_width: 0.0,
            m_height: 0.0,
            m_forced_width: 0.0,
            m_forced_height: 0.0,
            m_mm3_per_mm: 0.0,
            m_fan_speed: 0.0,
            m_additional_fan_speed: 0.0,
            m_extrusion_role: ExtrusionRole::None,
            m_filament_maps: Vec::new(),
            m_physical_extruder_map: Vec::new(),
            m_last_filament_id: Vec::new(),
            m_filament_id: Vec::new(),
            m_extruder_id: 0xff,
            m_extruder_colors: Vec::new(),
            m_extruder_temps: Vec::new(),
            m_thermal_index: ThermalIndex::new(),
            m_is_helio_gcode: false,
            m_highest_bed_temp: 0,
            m_extruded_last_z: 0.0,
            m_first_layer_height: 0.0,
            m_zero_layer_height: 0.0,
            m_processing_start_custom_gcode: false,
            m_pa_line_calibration: false,
            m_prev_role: ExtrusionRole::None,
            m_nozzle_volume: vec![0.0, 0.0],
            m_handle_hotend_as_extruder: false,
            m_nozzle_group_result: None,
            m_nozzle_status_recorder: crate::multi_nozzle_utils::NozzleStatusRecorder::new(),
            m_g1_line_id: 0,
            m_layer_id: 0,
            m_cp_color: CpColor::default(),
            m_seams_count: 0,
            m_detect_layer_based_on_tag: false,
            m_measure_g29_time: false,
            m_seams_detector: SeamsDetector::default(),
            m_producer: EProducer::Unknown,
            m_extruder_offsets: vec![[0.0; 3]; MIN_EXTRUDERS_COUNT],
            m_time_processor: TimeProcessor::default(),
            m_used_filaments: UsedFilaments::default(),
            m_result: GCodeProcessorResult::default(),
        };
        p.reset();
        // GCodeProcessor.cpp:1631-1634  m73 masks
        p.m_time_processor.machines[ETimeMode::Normal as usize].line_m73_main_mask =
            "M73 P%s R%s\n".to_string();
        p.m_time_processor.machines[ETimeMode::Normal as usize].line_m73_stop_mask =
            "M73 C%s\n".to_string();
        p.m_time_processor.machines[ETimeMode::Stealth as usize].line_m73_main_mask =
            "M73 Q%s S%s\n".to_string();
        p.m_time_processor.machines[ETimeMode::Stealth as usize].line_m73_stop_mask =
            "M73 D%s\n".to_string();
        p.register_commands();
        p
    }

    /// GCodeProcessor.cpp:1639-1723  void GCodeProcessor::register_commands()
    fn register_commands(&mut self) {
        // !!! registered command must be upper case (and a lowercase alias)
        for &(cmd, id, early_quit) in CMD_TABLE.iter() {
            self.m_command_processor
                .register_command(cmd, id as usize, early_quit);
            let lower = cmd.to_lowercase();
            if lower != cmd {
                self.m_command_processor
                    .register_command(&lower, id as usize, early_quit);
            }
        }
    }

    /// GCodeProcessor.cpp:2434-2524  void GCodeProcessor::reset()
    pub fn reset(&mut self) {
        self.m_units = EUnits::Millimeters;
        self.m_global_positioning_type = EPositioningType::Absolute;
        self.m_e_local_positioning_type = EPositioningType::Absolute;
        self.m_extruder_offsets = vec![[0.0; 3]; MIN_EXTRUDERS_COUNT];
        self.m_flavor = GCodeFlavor::RepRapSprinter;

        self.m_start_position = [0.0, 0.0, 0.0, 0.0];
        self.m_end_position = [0.0, 0.0, 0.0, 0.0];
        self.m_origin = [0.0, 0.0, 0.0, 0.0];
        self.m_cached_position.reset();
        self.m_wiping = false;
        self.m_flushing = false;
        self.m_virtual_flushing = false;
        self.m_skippable = false;
        self.m_skippable_type = SkipType::StNone;
        self.m_wipe_tower = false;
        self.m_remaining_volume = vec![0.0, 0.0];
        self.m_move_path_type = EMovePathType::NoopMove;
        self.m_arc_center = [0.0; 3];

        self.m_line_id = 0;
        self.m_last_line_id = 0;
        self.m_feedrate = 0.0;
        self.m_width = 0.0;
        self.m_height = 0.0;
        self.m_forced_width = 0.0;
        self.m_forced_height = 0.0;
        self.m_mm3_per_mm = 0.0;
        self.m_fan_speed = 0.0;
        // GCodeProcessor.cpp:2586
        self.m_additional_fan_speed = 0.0;

        self.m_extrusion_role = ExtrusionRole::None;

        self.m_is_helio_gcode = false;
        // m_filament_id = { -1, -1 } (unsigned char)
        self.m_filament_id = vec![0xff, 0xff];
        self.m_last_filament_id = vec![0xff, 0xff];
        self.m_extruder_id = 0xff;
        self.m_extruder_colors = (0..MIN_EXTRUDERS_COUNT as u8).collect();
        self.m_extruder_temps = vec![0.0; MIN_EXTRUDERS_COUNT];

        self.m_physical_extruder_map.clear();

        self.m_thermal_index = ThermalIndex::with_values(0.0, 0.0, 0.0);
        self.m_highest_bed_temp = 0;

        self.m_extruded_last_z = 0.0;
        self.m_zero_layer_height = 0.0;
        self.m_first_layer_height = 0.0;
        self.m_processing_start_custom_gcode = false;
        self.m_pa_line_calibration = false;
        // :2560, :2640
        self.m_nozzle_volume = vec![0.0, 0.0];
        self.m_nozzle_group_result = None;
        self.m_g1_line_id = 0;
        self.m_layer_id = 0;
        self.m_cp_color.reset();

        self.m_time_processor.reset();
        self.m_used_filaments.reset();

        self.m_result.reset();
        // m_result.id = ++s_result_id;  (s_result_id is process-global in C++)
        self.m_result.id = 1;

        self.m_detect_layer_based_on_tag = false;
        self.m_seams_count = 0;
        // GCodeProcessor.cpp:2624
        self.m_producer = EProducer::Unknown;
    }

    /// GCodeProcessor.cpp:2429-2432  void GCodeProcessor::enable_stealth_time_estimator(bool)
    pub fn enable_stealth_time_estimator(&mut self, enabled: bool) {
        self.m_time_processor.machines[ETimeMode::Stealth as usize].enabled = enabled;
    }

    /// GCodeProcessor.hpp:1336 `void set_xy_offset(double x, double y) { m_x_offset = x; m_y_offset =
    /// y; }` (BBS: set offset for gcode writer)
    pub fn set_xy_offset(&mut self, x: f64, y: f64) {
        self.m_x_offset = x;
        self.m_y_offset = y;
    }

    /// Faithful port of the machine-limit / acceleration parts of
    /// `GCodeProcessor::apply_config(const PrintConfig&)` (cpp:1908-1995).
    ///
    /// Only the time-relevant config is consumed here:
    ///  * `m_flavor` (cpp:1908),
    ///  * `m_time_processor.machine_limits` (cpp:1964-1970) — populated when the
    ///    flavor is Marlin-legacy / Marlin-firmware / Klipper; for Marlin-legacy
    ///    the travel acceleration mirrors the extruding value (cpp:1966-1969),
    ///  * `machines[i].max_acceleration / acceleration / *_retract / *_travel`
    ///    (cpp:1985-1995).
    ///
    /// GCodeProcessor.cpp:2030-2036, 2055-2066. The envelope retains both timing
    /// modes. Per-axis speed and acceleration can also vary by nozzle row.
    pub fn apply_config(&mut self, config: &crate::print_config::PrintConfig) {
        use crate::print_config::GCodeFlavor as CfgFlavor;

        // GCodeProcessor.cpp:1908  m_flavor = config.gcode_flavor;
        self.m_flavor = match config.gcode_flavor {
            CfgFlavor::MarlinLegacy => GCodeFlavor::MarlinLegacy,
            CfgFlavor::Marlin => GCodeFlavor::MarlinFirmware,
            CfgFlavor::Klipper => GCodeFlavor::Klipper,
            CfgFlavor::RepRapSprinter => GCodeFlavor::RepRapSprinter,
            CfgFlavor::RepRapFirmware => GCodeFlavor::RepRapFirmware,
            CfgFlavor::Repetier => GCodeFlavor::RepetierFirmware,
            CfgFlavor::MakerWare => GCodeFlavor::MakerWare,
            CfgFlavor::Sailfish => GCodeFlavor::Sailfish,
            CfgFlavor::Smoothie => GCodeFlavor::Smoothie,
            _ => GCodeFlavor::Other,
        };

        // GCodeProcessor.cpp:1964-1970 — machine_limits only for Marlin/Klipper.
        if matches!(
            self.m_flavor,
            GCodeFlavor::MarlinLegacy | GCodeFlavor::MarlinFirmware | GCodeFlavor::Klipper
        ) {
            // `machine_limits = reinterpret_cast<const MachineEnvelopeConfig&>(config)`: the
            // per-axis rows retain normal and stealth values for each configured nozzle.
            let ml = &mut self.m_time_processor.machine_limits;
            ml.machine_max_acceleration_x = config.machine_limit_values("machine_max_acceleration_x", config.machine_max_acceleration_x);
            ml.machine_max_acceleration_y = config.machine_limit_values("machine_max_acceleration_y", config.machine_max_acceleration_y);
            ml.machine_max_acceleration_z = config.machine_limit_values("machine_max_acceleration_z", config.machine_max_acceleration_z);
            ml.machine_max_acceleration_e = config.machine_limit_values("machine_max_acceleration_e", config.machine_max_acceleration_e);
            ml.machine_max_acceleration_extruding = config.machine_limit_values("machine_max_acceleration_extruding", config.machine_max_acceleration_extruding);
            ml.machine_max_acceleration_retracting = config.machine_limit_values("machine_max_acceleration_retracting", config.machine_max_acceleration_retracting);
            ml.machine_max_acceleration_travel = config.machine_limit_values("machine_max_acceleration_travel", config.machine_max_acceleration_travel);
            ml.machine_max_speed_x = config.machine_limit_values("machine_max_speed_x", config.machine_max_speed_x);
            ml.machine_max_speed_y = config.machine_limit_values("machine_max_speed_y", config.machine_max_speed_y);
            ml.machine_max_speed_z = config.machine_limit_values("machine_max_speed_z", config.machine_max_speed_z);
            ml.machine_max_speed_e = config.machine_limit_values("machine_max_speed_e", config.machine_max_speed_e);
            ml.machine_max_jerk_x = config.machine_limit_values("machine_max_jerk_x", config.machine_max_jerk_x);
            ml.machine_max_jerk_y = config.machine_limit_values("machine_max_jerk_y", config.machine_max_jerk_y);
            ml.machine_max_jerk_z = config.machine_limit_values("machine_max_jerk_z", config.machine_max_jerk_z);
            ml.machine_max_jerk_e = config.machine_limit_values("machine_max_jerk_e", config.machine_max_jerk_e);
            ml.machine_min_extruding_rate = config.machine_limit_values("machine_min_extruding_rate", config.machine_min_extruding_rate);
            ml.machine_min_travel_rate = config.machine_limit_values("machine_min_travel_rate", config.machine_min_travel_rate);

            // GCodeProcessor.cpp:2030-2036 — legacy Marlin has no separate travel
            // acceleration; it uses the 'extruding' value instead.
            if self.m_flavor == GCodeFlavor::MarlinLegacy {
                ml.machine_max_acceleration_travel = ml.machine_max_acceleration_extruding.clone();
            }
        }

        // GCodeProcessor.cpp:2038-2040. With the gate disabled the empty table
        // retains the pre-R955 lookup result (row zero).
        if crate::faithful_gate("PROCESSOR_MACHINE_CONFIG_ROW") {
            self.m_result.printer_extruder_id.clone_from(&config.printer_extruder_id);
            self.m_result.printer_extruder_variant.clone_from(&config.printer_extruder_variant);
            self.m_result.extruder_types = config.extruder_types();
        }

        // PROCESS_FILAMENT_CHANGE (on): what process_filament_change reads
        // (GCodeProcessor.cpp:1970-2023, 2045-2053, 2074-2079). =0 leaves the
        // processor single-filament with no tool-change time, as before R847.
        if crate::faithful_gate("PROCESS_FILAMENT_CHANGE") {
            // :1970-1972 — filament_count = filament_diameter.values.size().
            let filament_count = config.num_filaments();
            self.m_result.filaments_count = filament_count;
            // :1975-1977
            self.m_nozzle_volume = config.nozzle_volumes.iter().map(|v| *v as f32).collect();
            // :2002
            self.m_handle_hotend_as_extruder = config.handle_hotend_as_extruder;
            // :2006-2023 — the per-filament vectors, `get_at(i)` falling back to
            // the first value.
            fn get_at<T: Copy>(rows: &[T], i: usize, fallback: T) -> T {
                rows.get(i).or_else(|| rows.first()).copied().unwrap_or(fallback)
            }
            // GCodeProcessor.cpp:2016-2022. The one-based map is padded with the
            // master extruder. ConfigOptionPoints::get_at falls back to row zero.
            self.m_extruder_offsets = if let Some(first) = config.extruder_offsets.first()
                .filter(|_| crate::faithful_gate("PROCESSOR_FILAMENT_OFFSETS"))
            {
                (0..filament_count).map(|i| {
                    let id = config.filament_map.get(i).copied().or(config.master_extruder_id).unwrap_or(1);
                    let row = config.extruder_offsets.get((id as usize).wrapping_sub(1)).unwrap_or(first);
                    [row[0] as f32, row[1] as f32, 0.0]
                }).collect()
            } else {
                vec![[config.extruder_offset_x as f32, config.extruder_offset_y as f32, 0.0]; filament_count]
            };
            self.m_extruder_colors = (0..filament_count).map(|i| i as u8).collect();
            self.m_result.filament_diameters = (0..filament_count)
                .map(|i| get_at(&config.filament_diameters, i, config.filament_diameter) as f32)
                .collect();
            self.m_result.filament_densities = (0..filament_count)
                .map(|i| get_at(&config.filament_densities, i, config.filament_density) as f32)
                .collect();
            // GCodeProcessor.cpp:2024-2027. ConfigOptionVector::get_at uses row zero
            // past the end. A scalar-only config uses the defaults from PrintConfig.cpp
            // (3000-3006, 3624-3628, 3682-3688) and its saved filament price.
            if crate::faithful_gate("PROCESSOR_FILAMENT_MATERIAL_INFO") {
                self.m_result.required_nozzle_hrc = (0..filament_count)
                    .map(|i| get_at(&config.required_nozzle_hrc, i, 0))
                    .collect();
                // The native result holds ints, but apply_config first casts the option to float.
                self.m_result.filament_vitrification_temperature = (0..filament_count)
                    .map(|i| get_at(&config.temperature_vitrification, i, 100) as f32 as i32)
                    .collect();
                self.m_result.filament_costs = (0..filament_count)
                    .map(|i| get_at(&config.filament_costs, i, config.filament_cost) as f32)
                    .collect();
            }
            self.m_extruder_temps.resize(filament_count, 0.0);
            // GCodeProcessor.cpp:2042-2051, PrintConfig.cpp:1085-1108. The
            // selected AMS has its own load/unload times, including zero.
            let (loads, unloads) = if crate::faithful_gate("PROCESSOR_AMS_TIMES") && config.default_ams_type >= 0 {
                let get_float = |key: &str| config.ams_filament_times.get(key).copied();
                (
                    crate::print_config::config_def::gather_ams_times(false, get_float),
                    crate::print_config::config_def::gather_ams_times(true, get_float),
                )
            } else {
                (Vec::new(), Vec::new())
            };
            let (_, load, unload) = crate::multi_nozzle_utils::resolve_ams_load_unload(
                &loads,
                &unloads,
                config.default_ams_type,
                config.machine_load_filament_time as f32,
                config.machine_unload_filament_time as f32,
            );
            self.m_time_processor.filament_load_times = load;
            self.m_time_processor.filament_unload_times = unload;
            self.m_time_processor.extruder_change_times = config.machine_switch_extruder_time as f32;
            self.m_time_processor.hotend_change_times = config.machine_hotend_change_time as f32;
            // :2074-2079
            if !config.filament_map.is_empty() {
                self.m_filament_maps = config.filament_map.iter().map(|v| v - 1).collect();
            }
            self.ensure_nozzle_group_result(self.m_result.filaments_count as i32);
        }

        // GCodeProcessor.cpp:2072, 2089-2091. Copy the complete profile
        // polygons on reapplication, including an explicitly empty polygon.
        if crate::faithful_gate("PROCESSOR_PRINTABLE_GEOMETRY") {
            self.m_result.printable_height = config.printable_height as f32;
            self.m_result.printable_area.clone_from(&config.printable_area);
            self.m_result.bed_exclude_area.clone_from(&config.bed_exclude_area);
            self.m_result.wrapping_exclude_area.clone_from(&config.wrapping_exclude_area);
        }

        // GCodeProcessor.cpp:2081-2087. Each typed profile replaces the
        // detection setting, including an ordinary profile after a spiral one.
        if crate::faithful_gate("PROCESSOR_LAYER_TAG_PROFILES") {
            self.m_detect_layer_based_on_tag = config.spiral_vase || config.has_scarf_joint_seam;
        }

        // GCodeProcessor.cpp:2057-2059 — m_first_layer_height from `initial_layer_print_height`,
        // which the startup paths are flattened to and the first `Z_HEIGHT` tag compares with.
        // PROCESSOR_FIRST_LAYER_HEIGHT (on); =0 leaves it 0.
        if crate::faithful_gate("PROCESSOR_FIRST_LAYER_HEIGHT") {
            self.m_first_layer_height = config.first_layer_height.abs() as f32;
        }

        // GCodeProcessor.cpp:2054 — prepare_compensation_time (PROCESS_G4_G29).
        if crate::faithful_gate("PROCESS_G4_G29") {
            self.m_time_processor.prepare_compensation_time =
                config.machine_prepare_compensation_time as f32;
        }

        // GCodeProcessor.cpp:1985-1995 — per-mode machine acceleration setup.
        for i in 0..ETimeMode::COUNT {
            let max_acceleration = self.get_axis_max_acceleration_extruding(i);
            let max_retract_acceleration = self.get_machine_limit_retract_acceleration(i);
            let max_travel_acceleration = self.get_machine_limit_travel_acceleration(i);
            let m = &mut self.m_time_processor.machines[i];
            m.max_acceleration = max_acceleration;
            m.acceleration = if max_acceleration > 0.0 {
                max_acceleration
            } else {
                DEFAULT_ACCELERATION
            };
            m.max_retract_acceleration = max_retract_acceleration;
            m.retract_acceleration = if max_retract_acceleration > 0.0 {
                max_retract_acceleration
            } else {
                DEFAULT_RETRACT_ACCELERATION
            };
            m.max_travel_acceleration = max_travel_acceleration;
            m.travel_acceleration = if max_travel_acceleration > 0.0 {
                max_travel_acceleration
            } else {
                DEFAULT_TRAVEL_ACCELERATION
            };
        }
    }

    /// `get_option_value(machine_max_acceleration_extruding, mode)` (cpp:1986).
    fn get_axis_max_acceleration_extruding(&self, mode: usize) -> f32 {
        ml_get(&self.m_time_processor.machine_limits.machine_max_acceleration_extruding, mode)
    }

    /// `get_option_value(machine_max_acceleration_retracting, mode)` (cpp:1989).
    fn get_machine_limit_retract_acceleration(&self, mode: usize) -> f32 {
        ml_get(&self.m_time_processor.machine_limits.machine_max_acceleration_retracting, mode)
    }

    /// `get_option_value(machine_max_acceleration_travel, mode)` (cpp:1992).
    fn get_machine_limit_travel_acceleration(&self, mode: usize) -> f32 {
        ml_get(&self.m_time_processor.machine_limits.machine_max_acceleration_travel, mode)
    }

    // ----- accessors -----

    /// GCodeProcessor.cpp:2898-2901 `std::string GCodeProcessor::get_time_dhm(ETimeMode mode) const`:
    /// `short_time(get_time_dhms(m_time_processor.machines[mode].time))`, "N/A" for a mode not below
    /// `ETimeMode::Count` (the Rust enum has no such value, so that branch cannot be reached).
    pub fn get_time_dhm(&self, mode: ETimeMode) -> String {
        if (mode as usize) < ETimeMode::COUNT {
            crate::utils::short_time(&crate::utils::get_time_dhms(self.m_time_processor.machines[mode as usize].time))
        } else {
            String::from("N/A")
        }
    }

    /// GCodeProcessor.hpp `const Result& get_result() const`.
    pub fn result(&self) -> &GCodeProcessorResult {
        &self.m_result
    }

    pub fn result_mut(&mut self) -> &mut GCodeProcessorResult {
        &mut self.m_result
    }

    // ----- filament / extruder id helpers (cpp:6348-6377) -----

    /// GCodeProcessor.cpp:6372-6377  int get_extruder_id(bool force_initialize)
    fn get_extruder_id(&self, force_initialize: bool) -> i32 {
        if self.m_extruder_id == 0xff {
            return if force_initialize { 0 } else { -1 };
        }
        self.m_extruder_id as i32
    }

    /// GCodeProcessor.cpp:6348-6358  int get_filament_id(bool force_initialize)
    fn get_filament_id(&self, force_initialize: bool) -> i32 {
        let extruder_id = self.get_extruder_id(force_initialize);
        if extruder_id == -1 {
            return if force_initialize { 0 } else { -1 };
        }
        if self.m_filament_id[extruder_id as usize] == 0xff {
            return if force_initialize { 0 } else { -1 };
        }
        self.m_filament_id[extruder_id as usize] as i32
    }

    /// GCodeProcessor.cpp:6360-6370  int get_last_filament_id(bool force_initialize)
    fn get_last_filament_id(&self, force_initialize: bool) -> i32 {
        let extruder_id = self.get_extruder_id(force_initialize);
        if extruder_id == -1 {
            return if force_initialize { 0 } else { -1 };
        }
        if self.m_last_filament_id[extruder_id as usize] == 0xff {
            return if force_initialize { 0 } else { -1 };
        }
        self.m_last_filament_id[extruder_id as usize] as i32
    }

    /// GCodeProcessor.cpp:6596-6610. Machine limits follow the nozzle currently
    /// mounted in the active extruder. The C++ signature also accepts a filament ID.
    fn get_machine_config_idx(&self, _filament_idx: i32) -> i32 {
        let extruder_id = self.get_extruder_id(true);
        let Some(group) = &self.m_nozzle_group_result else { return 0 };
        let Some(&extruder_type) = self.m_result.extruder_types.get(extruder_id as usize) else { return 0 };
        let variants = &self.m_result.printer_extruder_variant;
        let ids = &self.m_result.printer_extruder_id;
        // A single row always resolves to zero. An incomplete table cannot be
        // indexed as a pair (PrintConfig.cpp:995 requires equal lengths).
        if variants.len() <= 1 || variants.len() != ids.len() {
            return 0;
        }
        let nozzle_id = self.m_nozzle_status_recorder.get_nozzle_in_extruder(extruder_id);
        let Some(nozzle) = group.get_nozzle_from_id(nozzle_id) else { return 0 };
        crate::print_config::config_def::get_config_index_base(
            crate::print_config::enums::NozzleVolumeType::from_i32(nozzle.volume_type as i32),
            extruder_type, extruder_id + 1, variants, ids,
        )
    }

    // ----- machine-limit getters (cpp:6022-6128) -----
    // These read `m_time_processor.machine_limits`, populated by `apply_config`
    // (cpp:1964-1995). When the limits are not present (no config applied, or a
    // non-Marlin/Klipper flavor) they fall back exactly like the C++
    // `get_option_value` on empty arrays (0.0 / passthrough feedrate).

    /// GCodeProcessor.cpp:6248-6254 — machine_min_extruding_rate empty → feedrate,
    /// else max(feedrate, min_extruding_rate).
    fn minimum_feedrate(&self, mode: ETimeMode, feedrate: f32) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if ml.machine_min_extruding_rate.is_empty() {
            return feedrate;
        }
        feedrate.max(ml_get(&ml.machine_min_extruding_rate, mode as usize))
    }

    /// GCodeProcessor.cpp:6256-6262 — machine_min_travel_rate empty → feedrate,
    /// else max(feedrate, min_travel_rate).
    fn minimum_travel_feedrate(&self, mode: ETimeMode, feedrate: f32) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if ml.machine_min_travel_rate.is_empty() {
            return feedrate;
        }
        feedrate.max(ml_get(&ml.machine_min_travel_rate, mode as usize))
    }

    /// GCodeProcessor.cpp:6038-6049 — per-axis max feedrate (mm/s).
    /// `axis` is the `Axis` discriminant (X=0,Y=1,Z=2,E=3); other axes → 0.0.
    fn get_axis_max_feedrate(&self, mode: ETimeMode, axis: usize, extruder_id: i32) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        let matched_pos = (extruder_id * 2) as usize;
        match axis {
            0 => ml_get(&ml.machine_max_speed_x, matched_pos + mode as usize),
            1 => ml_get(&ml.machine_max_speed_y, matched_pos + mode as usize),
            2 => ml_get(&ml.machine_max_speed_z, matched_pos + mode as usize),
            3 => ml_get(&ml.machine_max_speed_e, matched_pos + mode as usize),
            _ => 0.0,
        }
    }

    /// GCodeProcessor.cpp:6051-6062 — per-axis max acceleration (mm/s^2).
    fn get_axis_max_acceleration(&self, mode: ETimeMode, axis: usize, extruder_id: i32) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        let matched_pos = (extruder_id * 2) as usize;
        match axis {
            0 => ml_get(&ml.machine_max_acceleration_x, matched_pos + mode as usize),
            1 => ml_get(&ml.machine_max_acceleration_y, matched_pos + mode as usize),
            2 => ml_get(&ml.machine_max_acceleration_z, matched_pos + mode as usize),
            3 => ml_get(&ml.machine_max_acceleration_e, matched_pos + mode as usize),
            _ => 0.0,
        }
    }

    /// GCodeProcessor.cpp:6064-6074 — per-axis max jerk (mm/s).
    fn get_axis_max_jerk(&self, mode: ETimeMode, axis: usize) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        match axis {
            0 => ml_get(&ml.machine_max_jerk_x, mode as usize),
            1 => ml_get(&ml.machine_max_jerk_y, mode as usize),
            2 => ml_get(&ml.machine_max_jerk_z, mode as usize),
            3 => ml_get(&ml.machine_max_jerk_e, mode as usize),
            _ => 0.0,
        }
    }

    /// GCodeProcessor.cpp:6076-6081 — (x,y,z) max jerk.
    fn get_xyz_max_jerk(&self, mode: ETimeMode) -> [f32; 3] {
        let ml = &self.m_time_processor.machine_limits;
        [
            ml_get(&ml.machine_max_jerk_x, mode as usize),
            ml_get(&ml.machine_max_jerk_y, mode as usize),
            ml_get(&ml.machine_max_jerk_z, mode as usize),
        ]
    }

    /// GCodeProcessor.cpp:6083-6087  get_retract_acceleration
    fn get_retract_acceleration(&self, mode: ETimeMode) -> f32 {
        let id = mode as usize;
        if id < self.m_time_processor.machines.len() {
            self.m_time_processor.machines[id].retract_acceleration
        } else {
            DEFAULT_RETRACT_ACCELERATION
        }
    }

    /// GCodeProcessor.cpp:6089-6097  set_retract_acceleration
    fn set_retract_acceleration(&mut self, mode: ETimeMode, value: f32) {
        let id = mode as usize;
        if id < self.m_time_processor.machines.len() {
            let m = &mut self.m_time_processor.machines[id];
            m.retract_acceleration = if m.max_retract_acceleration == 0.0 {
                value
            } else {
                value.min(m.max_retract_acceleration)
            };
        }
    }

    /// GCodeProcessor.cpp:6099-6103  get_acceleration
    fn get_acceleration(&self, mode: ETimeMode) -> f32 {
        let id = mode as usize;
        if id < self.m_time_processor.machines.len() {
            self.m_time_processor.machines[id].acceleration
        } else {
            DEFAULT_ACCELERATION
        }
    }

    /// GCodeProcessor.cpp:6105-6113  set_acceleration
    fn set_acceleration(&mut self, mode: ETimeMode, value: f32) {
        let id = mode as usize;
        if id < self.m_time_processor.machines.len() {
            let m = &mut self.m_time_processor.machines[id];
            m.acceleration = if m.max_acceleration == 0.0 {
                value
            } else {
                value.min(m.max_acceleration)
            };
        }
    }

    /// GCodeProcessor.cpp:6115-6119  get_travel_acceleration
    fn get_travel_acceleration(&self, mode: ETimeMode) -> f32 {
        let id = mode as usize;
        if id < self.m_time_processor.machines.len() {
            self.m_time_processor.machines[id].travel_acceleration
        } else {
            DEFAULT_TRAVEL_ACCELERATION
        }
    }

    /// GCodeProcessor.cpp:6121-6128  set_travel_acceleration
    fn set_travel_acceleration(&mut self, mode: ETimeMode, value: f32) {
        let id = mode as usize;
        if id < self.m_time_processor.machines.len() {
            let m = &mut self.m_time_processor.machines[id];
            m.travel_acceleration = if m.max_travel_acceleration == 0.0 {
                value
            } else {
                value.min(m.max_travel_acceleration)
            };
        }
    }

    // ----- top-level processing -----

    /// GCodeProcessor.cpp:2645-2651  void GCodeProcessor::process_buffer(const std::string&)
    pub fn process_buffer(&mut self, buffer: &str) {
        let mut parser = GCodeReader::new();
        // The C++ lambda captures `this` and calls process_gcode_line(line, false).
        // GCodeReader's callback can't borrow self mutably while parser is borrowed,
        // so collect parsed lines first (faithful: same line stream, same order).
        let mut lines: Vec<crate::g_code_reader::GCodeLine> = Vec::new();
        parser.parse_buffer(buffer, |_reader, line| {
            lines.push(line.clone());
        });
        for line in &lines {
            self.process_gcode_line(line, false);
        }
    }

    /// Thin convenience wrapper used by the caller (mirrors `process_buffer`).
    pub fn process_gcode(&mut self, gcode: &str) {
        self.process_buffer(gcode);
        self.finalize(false);
    }

    /// GCodeProcessor.cpp:2920-2964  void GCodeProcessor::process_gcode_line(line, producers_enabled)
    pub fn process_gcode_line(&mut self, line: &crate::g_code_reader::GCodeLine, producers_enabled: bool) {
        self.m_line_id += 1;

        // update start position
        self.m_start_position = self.m_end_position;

        let cmd = line.cmd().to_string();
        // OrcaSlicer: Klipper SET_VELOCITY_LIMIT
        if self.m_flavor == GCodeFlavor::Klipper && cmd.eq_ignore_ascii_case("SET_VELOCITY_LIMIT") {
            self.process_set_velocity_limit(line);
            return;
        }

        if cmd.len() > 1 {
            if let Some(handler_id) = self.m_command_processor.process_comand(&cmd) {
                self.dispatch(cmd_from_id(handler_id), line);
            }
        } else {
            let comment = line.raw();
            if comment.len() > 2 && comment.as_bytes()[0] == b';' {
                let comment_content = &comment[1..];
                let first = comment_content.as_bytes()[0];
                if first == b'V' || first == b'v' {
                    // ";V{cmd}" — re-parse the comment body as a gcode line.
                    let body = comment_content.to_string();
                    let mut reader = GCodeReader::new();
                    let mut new_line: Option<crate::g_code_reader::GCodeLine> = None;
                    reader.parse_line_str(&body, |_r, gline| {
                        new_line = Some(gline.clone());
                    });
                    if let Some(nl) = new_line {
                        let ncmd = nl.cmd().to_string();
                        if let Some(handler_id) = self.m_command_processor.process_comand(&ncmd) {
                            self.dispatch(cmd_from_id(handler_id), &nl);
                        }
                    }
                } else {
                    // BLOCKED(deps): process_tags (tag/role/layer parsing) needs the
                    // producer-detection + tag pipeline; not ported here.
                    let _ = producers_enabled;
                    self.process_tags(comment_content, producers_enabled);
                }
            }
        }
    }

    /// Dispatch a matched command id to its handler (replaces the C++
    /// `std::function` bound in register_commands).
    fn dispatch(&mut self, cmd: Cmd, line: &crate::g_code_reader::GCodeLine) {
        match cmd {
            Cmd::G0 => self.process_g0(line),
            Cmd::G1 => self.process_g1(line),
            Cmd::G2G3 => self.process_g2_g3(line),
            Cmd::G4 => self.process_g4(line),
            Cmd::G10 => self.process_g10(line),
            Cmd::G11 => self.process_g11(line),
            Cmd::G20 => self.process_g20(line),
            Cmd::G21 => self.process_g21(line),
            Cmd::G22 => self.process_g22(line),
            Cmd::G23 => self.process_g23(line),
            Cmd::G28 => self.process_g28(line),
            Cmd::G29 => self.process_g29(line),
            Cmd::G90 => self.process_g90(line),
            Cmd::G91 => self.process_g91(line),
            Cmd::G92 => self.process_g92(line),
            Cmd::M1 => self.process_m1(line),
            Cmd::M82 => self.process_m82(line),
            Cmd::M83 => self.process_m83(line),
            Cmd::M104 => self.process_m104(line),
            Cmd::M106 => self.process_m106(line),
            Cmd::M107 => self.process_m107(line),
            Cmd::M108 => self.process_m108(line),
            Cmd::M109 => self.process_m109(line),
            Cmd::M132 => self.process_m132(line),
            Cmd::M135 => self.process_m135(line),
            Cmd::M140 => self.process_m140(line),
            Cmd::M190 => self.process_m190(line),
            Cmd::M191 => self.process_m191(line),
            Cmd::M201 => self.process_m201(line),
            Cmd::M203 => self.process_m203(line),
            Cmd::M204 => self.process_m204(line),
            Cmd::M205 => self.process_m205(line),
            Cmd::M221 => self.process_m221(line),
            Cmd::M400 => self.process_m400(line),
            Cmd::M401 => self.process_m401(line),
            Cmd::M402 => self.process_m402(line),
            Cmd::M566 => self.process_m566(line),
            Cmd::M702 => self.process_m702(line),
            Cmd::M1020 => self.process_m1020(line),
            Cmd::T => self.process_t(line),
            Cmd::Sync => self.process_sync(line),
            Cmd::M622 => self.process_m622(line),
            Cmd::M623 => self.process_m623(line),
            Cmd::VM104 => self.process_vm104(line),
            Cmd::VM109 => self.process_vm109(line),
            Cmd::VG1 => self.process_vg1(line),
        }
    }

    /// GCodeProcessor.cpp:3381-3632  process_tags.
    ///
    /// PROCESS_TAGS (on); =0 ignores every tag, as the port did before R846.
    /// Not ported: the color change, pause and custom G-code tags (:3525-3608),
    /// whose `custom_gcode_per_print_z` and `OptionsZCorrector` are missing; those
    /// tags are ignored. The producers' tags run only with `producers_enabled`
    /// (G-code loaded from a file), never for the G-code the port writes.
    fn process_tags(&mut self, comment: &str, producers_enabled: bool) {
        if !crate::faithful_gate("PROCESS_TAGS") {
            return;
        }
        // :3385-3387 — producers tags
        if producers_enabled && self.process_producers_tags(comment) {
            return;
        }

        // :3389-3396 — extrusion role tag
        if let Some(rest) = comment.strip_prefix(reserved_tag(ETags::Role)) {
            self.set_extrusion_role(crate::extrusion_entity::string_to_role(rest));
            // :3391-3392 (the detector's state feeds only the seam moves of the preview).
            if self.m_extrusion_role == ExtrusionRole::ExternalPerimeter {
                self.m_seams_detector.activate(true);
            }
            self.m_processing_start_custom_gcode =
                self.m_extrusion_role == ExtrusionRole::Custom && self.m_g1_line_id == 0;
            return;
        }

        if comment.starts_with(" PA_LINE_CALIBRATION_START") {
            self.m_pa_line_calibration = true;
            return;
        }

        if comment.starts_with(" PA_LINE_CALIBRATION_END") {
            self.m_pa_line_calibration = false;
            return;
        }

        // :3408-3418 — ; OBJECT_ID start / end
        if comment.starts_with(" start printing object") {
            self.m_object_label_id = get_object_label_id(comment);
            return;
        }
        if comment.starts_with(" stop printing object") {
            self.m_object_label_id = -1;
            return;
        }

        // :3420-3436 — ; Z_HEIGHT:
        if comment.starts_with(" Z_HEIGHT:") {
            let print_z = get_z_height(comment);
            if self.m_layer_id == 1
                && print_z > 0.0
                && (print_z - self.m_first_layer_height).abs() > EPSILON as f32
            {
                self.m_first_layer_height = print_z;
                for mv in self.m_result.moves.iter_mut() {
                    mv.position[2] = print_z;
                    mv.arc_center_position[2] = print_z;
                    for point in mv.interpolation_points.iter_mut() {
                        point[2] = print_z;
                    }
                }
            }
            self.m_print_z = print_z;
            return;
        }

        // :3438-3448 — wipe start / end
        if comment.starts_with(reserved_tag(ETags::WipeStart)) {
            self.m_wiping = true;
            return;
        }
        if comment.starts_with(reserved_tag(ETags::WipeEnd)) {
            self.m_wiping = false;
            return;
        }

        // :3450-3459 — wipe tower start / end
        if comment.starts_with(reserved_tag(ETags::WipeTowerStart)) {
            self.m_wipe_tower = true;
            return;
        }
        if comment.starts_with(reserved_tag(ETags::WipeTowerEnd)) {
            self.m_wipe_tower = false;
            let active_filament_id = self.get_filament_id(true);
            self.m_used_filaments.process_wipe_tower_cache(active_filament_id);
            return;
        }

        // :3462-3478 — skippable start / end / type
        if comment.starts_with(custom_tags(CustomETags::SkippableStart)) {
            self.m_skippable = true;
            return;
        }
        if comment.starts_with(custom_tags(CustomETags::SkippableEnd)) {
            self.m_skippable = false;
            self.m_skippable_type = SkipType::StNone;
            return;
        }
        if let Some(r#type) = comment.strip_prefix(custom_tags(CustomETags::SkippableType)) {
            self.set_skippable_type(r#type);
            return;
        }

        // :3480-3506 — flush start / end, virtual flush start / end
        if comment.starts_with(custom_tags(CustomETags::FlushStart)) {
            self.m_prev_role = self.m_extrusion_role;
            self.set_extrusion_role(ExtrusionRole::Flush);
            self.m_flushing = true;
            return;
        }
        if comment.starts_with(custom_tags(CustomETags::FlushEnd)) {
            self.set_extrusion_role(self.m_prev_role);
            self.m_flushing = false;
            return;
        }
        if comment.starts_with(custom_tags(CustomETags::VflushStart)) {
            self.m_prev_role = self.m_extrusion_role;
            self.set_extrusion_role(ExtrusionRole::Flush);
            self.m_virtual_flushing = true;
            return;
        }
        if comment.starts_with(custom_tags(CustomETags::VflushEnd)) {
            self.set_extrusion_role(self.m_prev_role);
            self.m_virtual_flushing = false;
            return;
        }

        // :3510-3523 — height and width tags (no producers: always read)
        // PROCESSOR_PARSE_NUMBER (on): `parse_number` as the C++ compiles it here (the value it
        // reads is kept even when the text does not end with the number). =0 keeps the value only
        // when the whole text parses.
        let faithful_parse_number = crate::faithful_gate("PROCESSOR_PARSE_NUMBER");
        if let Some(rest) = comment.strip_prefix(reserved_tag(ETags::Height)) {
            if faithful_parse_number {
                let _ = parse_number(rest, &mut self.m_forced_height);
            } else if let Some(v) = parse_number_f32(rest) {
                self.m_forced_height = v;
            }
            return;
        }
        if let Some(rest) = comment.strip_prefix(reserved_tag(ETags::Width)) {
            if faithful_parse_number {
                let _ = parse_number(rest, &mut self.m_forced_width);
            } else if let Some(v) = parse_number_f32(rest) {
                self.m_forced_width = v;
            }
            return;
        }

        // :3610-3624 — layer change tag
        if comment == reserved_tag(ETags::LayerChange) {
            self.m_layer_id += 1;
            if self.m_detect_layer_based_on_tag {
                if self.m_result.moves.is_empty() || self.m_result.spiral_vase_layers.is_empty() {
                    self.m_result.spiral_vase_layers.push((f32::MAX, (0, 0)));
                } else {
                    let move_id =
                        self.m_result.moves.len() - 1 - self.m_seams_count.max(0) as usize;
                    if let Some(last) = self.m_result.spiral_vase_layers.last_mut() {
                        (last.1).1 = move_id;
                    }
                    self.m_result.spiral_vase_layers.push((f32::MAX, (move_id, move_id)));
                }
            }
        }
    }

    /// GCodeProcessor.cpp:6223-6232  set_extrusion_role.
    fn set_extrusion_role(&mut self, role: ExtrusionRole) {
        // m_used_filaments.process_role_cache(this): the current filament's
        // diameter and density, the role before this one.
        let active_filament_id = self.get_filament_id(true);
        let diameter = self
            .m_result
            .filament_diameters
            .get(active_filament_id as usize)
            .or(self.m_result.filament_diameters.last())
            .copied()
            .unwrap_or(DEFAULT_FILAMENT_DIAMETER);
        let density = self
            .m_result
            .filament_densities
            .get(active_filament_id as usize)
            .or(self.m_result.filament_densities.last())
            .copied()
            .unwrap_or(DEFAULT_FILAMENT_DENSITY);
        let active_role = self.m_extrusion_role;
        self.m_used_filaments.process_role_cache(diameter, density, active_role);
        if role == ExtrusionRole::FloatingVerticalShell {
            self.m_extrusion_role = ExtrusionRole::SolidInfill;
        } else {
            self.m_extrusion_role = role;
        }
    }

    /// GCodeProcessor.cpp:6234-6246  set_skippable_type.
    fn set_skippable_type(&mut self, r#type: &str) {
        if !self.m_skippable {
            self.m_skippable_type = SkipType::StNone;
            return;
        }
        self.m_skippable_type = skip_type_map(r#type).unwrap_or(SkipType::StOther);
    }

    /// GCodeProcessor.cpp:5252-5255  process_M1 — simulate st_synchronize.
    fn process_m1(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.simulate_st_synchronize(0.0, ExtrusionRole::None);
    }

    /// GCodeProcessor.cpp:3931-3934  process_G0 -> process_G1
    fn process_g0(&mut self, line: &crate::g_code_reader::GCodeLine) {
        self.process_g1(line);
    }

    /// GCodeProcessor.cpp:4723-5119  void GCodeProcessor::process_G2_G3(line)
    ///
    /// Faithful port of the time-estimate path. The arc is fed to the planner as a
    /// single TimeBlock whose `distance` is the full 3D arc length (`delta_xyz`),
    /// with a centripetal-acceleration cruise clamp and X-Y-plane resultant
    /// feedrate/acceleration projection (cpp:4905-5066). The arc interpolation
    /// points (`arc_interpolation`) only feed `store_move_vertex` for
    /// visualization and do NOT affect the time block, so they are omitted.
    fn process_g2_g3(&mut self, line: &crate::g_code_reader::GCodeLine) {
        use crate::g_code_reader::Axis;

        let filament_id = self.get_filament_id(true);
        let filament_diameter = if (filament_id as usize) < self.m_result.filament_diameters.len() {
            self.m_result.filament_diameters[filament_id as usize]
        } else {
            *self.m_result.filament_diameters.last().unwrap()
        };
        let filament_radius = 0.5 * filament_diameter;
        let area_filament_cross_section = (PI as f32) * sqr(filament_radius);

        // absolute_position lambda (cpp:4731-4754) — handles I/J relative to start.
        let global_relative = self.m_global_positioning_type == EPositioningType::Relative;
        let e_relative = self.m_e_local_positioning_type == EPositioningType::Relative;
        let units_inches = self.m_units == EUnits::Inches;
        let start_position = self.m_start_position;
        let origin = self.m_origin;
        // ARC_ABS_POS_DOUBLE (on unless =0; see GCODE_FLAVOR_KEY for why): the lambda returns `m_start_position[..] + ret`,
        // an `AxisCoords` double plus a float (GCodeProcessor.cpp:4912-4935), so
        // an axis the line does not name keeps its double and its delta is 0. The
        // port returned f32: the E of a spiral lift (`G3 Z.. I.. J.. P1`) came
        // back rounded, the delta was not 0 and the arc was typed Extrude with
        // the current role instead of Travel. Unset, it rounds through f32.
        let abs_double = crate::faithful_gate("ARC_ABS_POS_DOUBLE");
        let absolute_position = |axis: Axis, lineg: &crate::g_code_reader::GCodeLine| -> f64 {
            let mut is_relative = global_relative;
            if axis == Axis::E {
                is_relative |= e_relative;
            }
            let v = if lineg.has(axis) {
                let length_scale = if units_inches { INCHES_TO_MM } else { 1.0 };
                let ret = (lineg.value(axis) * length_scale) as f64;
                match axis {
                    Axis::I => start_position[Axis::X as usize] + ret,
                    Axis::J => start_position[Axis::Y as usize] + ret,
                    _ => {
                        let base = if is_relative {
                            start_position[axis as usize]
                        } else {
                            origin[axis as usize]
                        };
                        base + ret
                    }
                }
            } else {
                match axis {
                    Axis::I => start_position[Axis::X as usize],
                    Axis::J => start_position[Axis::Y as usize],
                    _ => start_position[axis as usize],
                }
            };
            if abs_double { v } else { v as f32 as f64 }
        };

        self.m_g1_line_id += 1;
        // enable processing of lines M201/M203/M204/M205 (cpp:4790)
        self.m_time_processor.machine_envelope_processing_enabled = true;

        // get axes positions from line (X..=E) (cpp:4793-4795)
        for a in 0..=(Axis::E as usize) {
            let ax = match a {
                0 => Axis::X,
                1 => Axis::Y,
                2 => Axis::Z,
                _ => Axis::E,
            };
            self.m_end_position[a] = absolute_position(ax, line);
        }
        // G2/G3 with no I and J — invalid (cpp:4797-4798)
        if !line.has(Axis::I) && !line.has(Axis::J) {
            return;
        }
        // P mode validity check (cpp:4800-4804)
        if line.has(Axis::P)
            && (self.m_start_position[Axis::X as usize] != self.m_end_position[Axis::X as usize]
                || self.m_start_position[Axis::Y as usize] != self.m_end_position[Axis::Y as usize]
                || (line.p() as i32) != 1)
        {
            return;
        }

        // arc center (cpp:4806)
        self.m_arc_center = [
            absolute_position(Axis::I, line) as f32,
            absolute_position(Axis::J, line) as f32,
            self.m_start_position[Axis::Z as usize] as f32,
        ];
        // G2 = CW, G3 = CCW (cpp:4807-4809)
        let cmd = line.cmd();
        let is_g2 = cmd.as_bytes().get(1) == Some(&b'2');
        self.m_move_path_type = if is_g2 {
            EMovePathType::ArcMoveCw
        } else {
            EMovePathType::ArcMoveCcw
        };
        let is_ccw = self.m_move_path_type == EMovePathType::ArcMoveCcw;

        let start_point = nalgebra::Vector3::new(
            self.m_start_position[Axis::X as usize] as f32,
            self.m_start_position[Axis::Y as usize] as f32,
            self.m_start_position[Axis::Z as usize] as f32,
        );
        let end_point = nalgebra::Vector3::new(
            self.m_end_position[Axis::X as usize] as f32,
            self.m_end_position[Axis::Y as usize] as f32,
            self.m_end_position[Axis::Z as usize] as f32,
        );
        let center = nalgebra::Vector3::new(self.m_arc_center[0], self.m_arc_center[1], self.m_arc_center[2]);

        use crate::circle::ArcSegment;
        use crate::circle::Circle;

        let float_model_arc = crate::faithful_gate("TIME_FLOAT_MODEL");
        // arc length (cpp:4814-4817)
        let arc_length = if !line.has(Axis::P) {
            ArcSegment::calc_arc_length(start_point, end_point, center, is_ccw)
        } else {
            if float_model_arc {
                // cpp:4999: `((int)line.p()) * 2 * PI * norm` with the double PI
                // of libslic3r.h: an int times a double times a float.
                (((line.p() as i32) * 2) as f64 * PI * (start_point - center).norm() as f64) as f32
            } else {
                (line.p() as i32) as f32 * 2.0 * (PI as f32) * (start_point - center).norm()
            }
        };
        // tangential directions (cpp:4821-4822)
        let start_dir = Circle::calc_tangential_vector(start_point, center, is_ccw);
        let end_dir = Circle::calc_tangential_vector(end_point, center, is_ccw);

        // updates feedrate from line (cpp:4824-4826)
        if line.has_f() {
            self.m_feedrate = line.f() * MMMIN_TO_MMSEC;
        }

        // movement deltas (cpp:4828-4832)
        let mut delta_pos = [0.0f64; 4];
        for a in 0..=(Axis::E as usize) {
            delta_pos[a] = self.m_end_position[a] - self.m_start_position[a];
        }

        // no displacement (cpp:4834-4836)
        if arc_length == 0.0 && delta_pos[Axis::Z as usize] == 0.0 {
            return;
        }

        let de = delta_pos[Axis::E as usize];
        let r#type = if de == 0.0 {
            EMoveType::Travel
        } else {
            EMoveType::Extrude
        };

        // delta_xyz = sqrt(arc_length^2 + dz^2) (cpp:4841)
        let dz = delta_pos[Axis::Z as usize] as f32;
        let delta_xyz = if float_model_arc {
            // cpp:5023: sqr(float) + sqr(double), root taken in double.
            {
                let dz64 = delta_pos[Axis::Z as usize];
                (sqr(arc_length) as f64 + dz64 * dz64).sqrt() as f32
            }
        } else {
            (sqr(arc_length) + sqr(dz)).sqrt()
        };

        // extrude width/height + filament caches (cpp:4842-4903)
        // PROCESSOR_ARC_DOUBLE (on): `delta_pos[E]` is a double (AxisCoords), so the filament volume
        // and the widths of an arc are computed in double and stored as floats
        // (GCodeProcessor.cpp:5025, 5065-5072). =0 computes them in float.
        let arc_double = crate::faithful_gate("PROCESSOR_ARC_DOUBLE");
        if r#type == EMoveType::Extrude {
            let volume_extruded_filament =
                if arc_double { (area_filament_cross_section as f64 * de) as f32 } else { area_filament_cross_section * de as f32 };
            let area_toolpath_cross_section = volume_extruded_filament / delta_xyz;

            match self.m_extrusion_role {
                ExtrusionRole::SupportMaterial
                | ExtrusionRole::SupportMaterialInterface
                | ExtrusionRole::SupportTransition => {
                    self.m_used_filaments
                        .increase_support_caches(volume_extruded_filament as f64);
                }
                ExtrusionRole::WipeTower => {
                    self.m_used_filaments
                        .increase_wipe_tower_caches(volume_extruded_filament as f64);
                }
                _ => {
                    self.m_used_filaments
                        .increase_model_caches(volume_extruded_filament as f64);
                }
            }
            self.m_mm3_per_mm = area_toolpath_cross_section;

            if self.m_forced_height > 0.0 {
                self.m_height = self.m_forced_height;
            } else if self.m_end_position[Axis::Z as usize] as f32 > self.m_extruded_last_z + EPSILON {
                self.m_height = self.m_end_position[Axis::Z as usize] as f32 - self.m_extruded_last_z;
            }
            if self.m_height == 0.0 {
                self.m_height = DEFAULT_TOOLPATH_HEIGHT;
            }
            if self.m_end_position[Axis::Z as usize] == 0.0 {
                self.m_end_position[Axis::Z as usize] = self.m_height as f64;
            }
            self.m_extruded_last_z = self.m_end_position[Axis::Z as usize] as f32;

            if self.m_forced_width > 0.0 {
                self.m_width = self.m_forced_width;
            } else if self.m_extrusion_role == ExtrusionRole::ExternalPerimeter {
                self.m_width = if arc_double {
                    //BBS: cross section: rectangle
                    (de * ((PI * sqr(1.05f32 * filament_radius) as f64) as f32) as f64 / (delta_xyz * self.m_height) as f64) as f32
                } else {
                    de as f32 * (PI as f32 * sqr(1.05 * filament_radius)) / (delta_xyz * self.m_height)
                };
            } else if self.m_extrusion_role == ExtrusionRole::BridgeInfill
                || self.m_extrusion_role == ExtrusionRole::None
            {
                self.m_width = if arc_double {
                    //BBS: cross section: circle
                    (filament_diameter as f64 * (de / delta_xyz as f64).sqrt()) as f32
                } else {
                    filament_diameter * (de as f32 / delta_xyz).sqrt()
                };
            } else {
                self.m_width = if arc_double {
                    //BBS: cross section: rectangle + 2 semicircles
                    (de * ((PI * sqr(filament_radius) as f64) as f32) as f64 / (delta_xyz * self.m_height) as f64
                        + (((1.0 - 0.25 * PI) as f32) * self.m_height) as f64) as f32
                } else {
                    de as f32 * (PI as f32 * sqr(filament_radius)) / (delta_xyz * self.m_height)
                        + (1.0 - 0.25 * PI as f32) * self.m_height
                };
            }
            if self.m_width == 0.0 {
                self.m_width = DEFAULT_TOOLPATH_WIDTH;
            }
            self.m_width = self.m_width.min(2.0f32.max(4.0 * self.m_height));
        }

        // time estimate section (cpp:4905-5066) -------------------------------
        let inv_distance = 1.0 / delta_xyz;
        let radius = ArcSegment::calc_arc_radius(start_point, center);

        for i in 0..ETimeMode::COUNT {
            let mode = if i == 0 {
                ETimeMode::Normal
            } else {
                ETimeMode::Stealth
            };
            if !self.m_time_processor.machines[i].enabled {
                continue;
            }

            // curr.feedrate (cpp:4919-4921)
            let mut feedrate = if r#type == EMoveType::Travel {
                self.minimum_travel_feedrate(mode, self.m_feedrate)
            } else {
                self.minimum_feedrate(mode, self.m_feedrate)
            };

            let enter_direction = [start_dir[0], start_dir[1], start_dir[2]];
            let exit_direction = [end_dir[0], end_dir[1], end_dir[2]];
            let prev_exit_direction = self.m_time_processor.machines[i].prev.exit_direction;
            let prev_feedrate = self.m_time_processor.machines[i].prev.feedrate;
            let prev_safe_feedrate = self.m_time_processor.machines[i].prev.safe_feedrate;
            let blocks_empty = self.m_time_processor.machines[i].blocks.is_empty();
            let extrude_factor =
                self.m_time_processor.machines[i].extrude_factor_override_percentage;

            let mut block = TimeBlock::default();
            block.move_type = r#type;
            block.skippable_type = self.m_skippable_type;
            block.role = if r#type != EMoveType::Travel
                || self.m_extrusion_role == ExtrusionRole::Custom
            {
                self.m_extrusion_role
            } else {
                ExtrusionRole::None
            };
            block.distance = delta_xyz;
            block.move_id = self.m_result.moves.len() as u32;
            block.g1_line_id = self.m_g1_line_id;
            block.layer_id = 1u32.max(self.m_layer_id);
            block.flags.prepare_stage = self.m_processing_start_custom_gcode;

            // centripetal-acceleration cruise clamp (cpp:4941-4943)
            let centripetal_acceleration = self.get_acceleration(mode);
            let max_feedrate_by_centri_acc =
                (centripetal_acceleration * radius).sqrt() / (arc_length * inv_distance);
            feedrate = feedrate.min(max_feedrate_by_centri_acc);

            // block cruise feedrate — X-Y resultant projection (cpp:4945-4968)
            // GCodeProcessor.cpp:5135 and :5173-5179: the arc branch never
            // assigns `curr.axis_feedrate[E]` (it only multiplies what the last
            // move left there) and never resets `curr.safe_feedrate` to the cruise
            // feedrate (it only lowers what the last move left). After a retract
            // (`G1 E-.02 F1800`) the spiral lift therefore inherits the retract's
            // E feedrate and its safe feedrate.
            let arc_stale_state = crate::faithful_gate("TIME_ARC_STALE_STATE");
            // The axis feedrates are doubles (GCodeProcessor.hpp:639-640); see
            // process_g1 for what TIME_FLOAT_MODEL follows.
            let mut axis_feedrate = [0.0f64; 4];
            let mut abs_axis_feedrate = [0.0f64; 4];
            if arc_stale_state {
                axis_feedrate[Axis::E as usize] =
                    self.m_time_processor.machines[i].curr.axis_feedrate[Axis::E as usize];
                if !float_model_arc {
                    axis_feedrate[Axis::E as usize] = axis_feedrate[Axis::E as usize] as f32 as f64;
                }
            }
            let mut min_feedrate_factor = 1.0f32;
            for a in 0..=(Axis::E as usize) {
                if a == Axis::X as usize || a == Axis::Y as usize {
                    axis_feedrate[a] = (feedrate * arc_length * inv_distance) as f64;
                } else if a == Axis::Z as usize {
                    axis_feedrate[a] = if float_model_arc {
                        feedrate as f64 * delta_pos[a] * inv_distance as f64
                    } else {
                        (feedrate * delta_pos[a] as f32 * inv_distance) as f64
                    };
                } else {
                    // E axis (cpp:5135): curr.axis_feedrate[E] *= extrude_factor.
                    axis_feedrate[a] = if float_model_arc {
                        axis_feedrate[a] * extrude_factor as f64
                    } else {
                        (axis_feedrate[a] as f32 * extrude_factor) as f64
                    };
                }
                abs_axis_feedrate[a] = axis_feedrate[a].abs();
                if abs_axis_feedrate[a] != 0.0 {
                    let axis_max_feedrate = self.get_axis_max_feedrate(
                        mode,
                        a,
                        self.get_machine_config_idx(self.get_filament_id(true)),
                    );
                    if axis_max_feedrate != 0.0 {
                        min_feedrate_factor = if float_model_arc {
                            min_feedrate_factor
                                .min((axis_max_feedrate as f64 / abs_axis_feedrate[a]) as f32)
                        } else {
                            min_feedrate_factor
                                .min(axis_max_feedrate / abs_axis_feedrate[a] as f32)
                        };
                    }
                }
            }
            feedrate *= min_feedrate_factor;
            block.feedrate_profile.cruise = feedrate;
            if min_feedrate_factor < 1.0 {
                for a in 0..=(Axis::E as usize) {
                    if float_model_arc {
                        axis_feedrate[a] *= min_feedrate_factor as f64;
                        abs_axis_feedrate[a] *= min_feedrate_factor as f64;
                    } else {
                        axis_feedrate[a] = (axis_feedrate[a] as f32 * min_feedrate_factor) as f64;
                        abs_axis_feedrate[a] =
                            (abs_axis_feedrate[a] as f32 * min_feedrate_factor) as f64;
                    }
                }
            }

            // block acceleration — X-Y resultant projection (cpp:4970-4988)
            let acceleration = if r#type == EMoveType::Travel {
                self.get_travel_acceleration(mode)
            } else {
                self.get_acceleration(mode)
            };
            let mut min_acc_factor = 1.0f32;
            for a in 0..=(Axis::Z as usize) {
                // cpp:5157-5170: `axis_acc` is an AxisCoords (double); the Z term
                // multiplies a float by a double.
                let axis_acc: f64 = if a == Axis::X as usize || a == Axis::Y as usize {
                    (acceleration * arc_length * inv_distance) as f64
                } else if float_model_arc {
                    acceleration as f64 * delta_pos[a].abs() * inv_distance as f64
                } else {
                    (acceleration * (delta_pos[a] as f32).abs() * inv_distance) as f64
                };
                if axis_acc != 0.0 {
                    let axis_max_acceleration = self.get_axis_max_acceleration(
                        mode,
                        a,
                        self.get_machine_config_idx(self.get_filament_id(true)),
                    );
                    if axis_max_acceleration != 0.0 && axis_acc > axis_max_acceleration as f64 {
                        min_acc_factor = if float_model_arc {
                            min_acc_factor.min((axis_max_acceleration as f64 / axis_acc) as f32)
                        } else {
                            min_acc_factor.min(axis_max_acceleration / axis_acc as f32)
                        };
                    }
                }
            }
            block.acceleration = acceleration * min_acc_factor;

            // block exit feedrate (cpp:5173-5179)
            let mut safe_feedrate = if arc_stale_state {
                self.m_time_processor.machines[i].curr.safe_feedrate
            } else {
                block.feedrate_profile.cruise
            };
            for a in 0..=(Axis::E as usize) {
                let axis_max_jerk = self.get_axis_max_jerk(mode, a);
                if abs_axis_feedrate[a] > axis_max_jerk as f64 {
                    safe_feedrate = safe_feedrate.min(axis_max_jerk);
                }
            }
            block.feedrate_profile.exit = safe_feedrate;

            const PREVIOUS_FEEDRATE_THRESHOLD: f32 = 0.0001;
            // block entry feedrate (cpp:4998-5043)
            let mut vmax_junction = safe_feedrate;
            if !blocks_empty && prev_feedrate > PREVIOUS_FEEDRATE_THRESHOLD {
                vmax_junction = prev_feedrate.min(block.feedrate_profile.cruise);

                let mut limited = false;
                let exit_direction_unit = normalized3(prev_exit_direction);
                let enter_direction_unit = normalized3(enter_direction);
                let mut k_min = 10000.0f32;

                // a == X branch only (cpp:5013-5027)
                let jerk_v = [
                    (enter_direction_unit[0] - exit_direction_unit[0]).abs(),
                    (enter_direction_unit[1] - exit_direction_unit[1]).abs(),
                    (enter_direction_unit[2] - exit_direction_unit[2]).abs(),
                ];
                let max_xyz_jerk_v = self.get_xyz_max_jerk(mode);
                for idx in 0..3 {
                    if jerk_v[idx] > 0.0 {
                        limited = true;
                        let k = max_xyz_jerk_v[idx] / jerk_v[idx];
                        if k < k_min {
                            k_min = k;
                        }
                    }
                }
                if limited {
                    vmax_junction = k_min;
                }

                let vmax_junction_threshold = vmax_junction * 0.99;
                if prev_safe_feedrate > vmax_junction_threshold
                    && safe_feedrate > vmax_junction_threshold
                {
                    vmax_junction = safe_feedrate;
                }
            }

            let v_allowable =
                max_allowable_speed(-block.acceleration, safe_feedrate, block.distance);
            block.feedrate_profile.entry = vmax_junction.min(v_allowable);
            block.max_entry_speed = vmax_junction;
            block.flags.nominal_length = block.feedrate_profile.cruise <= v_allowable;
            block.flags.recalculate = true;
            block.safe_feedrate = safe_feedrate;

            block.calculate_trapezoid();

            // updates previous + push block (cpp:5057-5059)
            {
                let machine = &mut self.m_time_processor.machines[i];
                machine.curr.feedrate = feedrate;
                machine.curr.safe_feedrate = safe_feedrate;
                machine.curr.axis_feedrate = axis_feedrate;
                machine.curr.abs_axis_feedrate = abs_axis_feedrate;
                machine.curr.enter_direction = enter_direction;
                machine.curr.exit_direction = exit_direction;
                machine.prev = machine.curr.clone();
                machine.blocks.push(block);
            }

            if self.m_time_processor.machines[i].blocks.len()
                > TimeProcessor::PLANNER_REFRESH_THRESHOLD
            {
                self.run_calculate_time(i, TimeProcessor::PLANNER_QUEUE_SIZE, 0.0, ExtrusionRole::None);
            }
        }

        // BLOCKED(deps): m_seams_detector + spiral_vase_layers side-effects.

        // store move (cpp:5118)
        let path_type = self.m_move_path_type;
        self.store_move_vertex(r#type, path_type);
    }

    /// The time processor, for `TimeProcessor::post_process`.
    pub fn time_processor(&self) -> &TimeProcessor {
        &self.m_time_processor
    }

    /// GCodeProcessor.cpp:2888-2891  get_time.
    pub fn get_time(&self, mode: ETimeMode) -> f32 {
        if (mode as usize) < ETimeMode::COUNT {
            self.m_time_processor.machines[mode as usize].time
        } else {
            0.0
        }
    }

    /// GCodeProcessor.cpp:2893-2896  get_prepare_time.
    pub fn get_prepare_time(&self, mode: ETimeMode) -> f32 {
        if (mode as usize) < ETimeMode::COUNT {
            self.m_time_processor.machines[mode as usize].prepare_time
        } else {
            0.0
        }
    }

    /// GCodeProcessor.cpp:5304-5312  process_G4 — dwell: `S` seconds, else `P`
    /// milliseconds (`has_value('S') || has_value('P')` reads `P` only without
    /// `S`). PROCESS_G4_G29 (on); =0 ignores the dwell.
    fn process_g4(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("PROCESS_G4_G29") {
            return;
        }
        let (mut value_s, mut value_p) = (0.0f32, 0.0f32);
        let found = if let Some(s) = line.has_value(b'S') {
            value_s = s;
            true
        } else if let Some(p) = line.has_value(b'P') {
            value_p = p;
            true
        } else {
            false
        };
        if found {
            value_s += value_p * 0.001;
            self.simulate_st_synchronize(value_s, ExtrusionRole::None);
        }
    }

    /// GCodeProcessor.cpp:5776-5785  process_M622 — `M622 J1` opens a block
    /// whose G29 is timed (PROCESS_G4_G29).
    fn process_m622(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("PROCESS_G4_G29") {
            return;
        }
        if let Some(value_j) = line.has_value(b'J') {
            let interger_j = value_j.round() as i32;
            if interger_j == 1 && !self.m_measure_g29_time {
                self.m_measure_g29_time = true;
            }
        }
    }

    /// GCodeProcessor.cpp:5787-5791  process_M623.
    fn process_m623(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("PROCESS_G4_G29") {
            return;
        }
        if self.m_measure_g29_time {
            self.m_measure_g29_time = false;
        }
    }

    /// GCodeProcessor.cpp:5144-5148  process_G10 — store retract move.
    fn process_g10(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.store_move_vertex(EMoveType::Retract, EMovePathType::NoopMove);
    }

    /// GCodeProcessor.cpp:5150-5154  process_G11 — store unretract move.
    fn process_g11(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.store_move_vertex(EMoveType::Unretract, EMovePathType::NoopMove);
    }

    /// GCodeProcessor.cpp:5156-5159  process_G20 — units = inches.
    fn process_g20(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.m_units = EUnits::Inches;
    }

    /// GCodeProcessor.cpp:5161-5164  process_G21 — units = millimeters.
    fn process_g21(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.m_units = EUnits::Millimeters;
    }

    /// GCodeProcessor.cpp:5166-5170  process_G22 — store retract move.
    fn process_g22(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.store_move_vertex(EMoveType::Retract, EMovePathType::NoopMove);
    }

    /// GCodeProcessor.cpp:5172-5176  process_G23 — store unretract move.
    fn process_g23(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.store_move_vertex(EMoveType::Unretract, EMovePathType::NoopMove);
    }

    /// GCodeProcessor.cpp:5178-5202  process_G28 — home to origin via synthetic G1.
    fn process_g28(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let cmd = line.cmd();
        let mut new_line_raw = cmd.to_string();
        let mut found = false;
        // G28_HAS_CHAR (on unless =0; see GCODE_FLAVOR_KEY for why): `line.has('X')` is the `char` overload, which looks
        // for the letter in the raw line (GCodeReader.cpp:225-245), value or not
        // (GCodeProcessor.cpp:5365-5376). `G28 Z P0 T250` has a Z without a
        // number, which the axis mask does not record: the port took it for a
        // bare G28 and homes X and Y as well. Unset, it asks the mask.
        let by_char = crate::faithful_gate("G28_HAS_CHAR");
        let has = |c: u8, a: crate::g_code_reader::Axis| {
            if by_char { line.has_char(c) } else { line.has(a) }
        };
        if has(b'X', crate::g_code_reader::Axis::X) {
            new_line_raw += " X0";
            found = true;
        }
        if has(b'Y', crate::g_code_reader::Axis::Y) {
            new_line_raw += " Y0";
            found = true;
        }
        if has(b'Z', crate::g_code_reader::Axis::Z) {
            new_line_raw += " Z0";
            found = true;
        }
        if !found {
            new_line_raw += " X0  Y0  Z0";
        }
        let mut reader = GCodeReader::new();
        let mut new_gline: Option<crate::g_code_reader::GCodeLine> = None;
        reader.parse_line_str(&new_line_raw, |_r, gline| {
            new_gline = Some(gline.clone());
        });
        if let Some(nl) = new_gline {
            self.process_g1(&nl);
        }
    }

    /// GCodeProcessor.cpp:5315-5324  process_G29 — bed levelling takes
    /// `machine_prepare_compensation_time`; on a Bambu Lab printer only inside an
    /// `M622 J1` block. PROCESS_G4_G29 (on); =0 ignores it.
    fn process_g29(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("PROCESS_G4_G29") {
            return;
        }
        // SAFETY: S_IS_BBL_PRINTER is only written during configuration.
        if unsafe { S_IS_BBL_PRINTER } {
            if self.m_measure_g29_time {
                let t = self.m_time_processor.prepare_compensation_time;
                self.simulate_st_synchronize(t, ExtrusionRole::None);
            }
        } else {
            let t = self.m_time_processor.prepare_compensation_time;
            self.simulate_st_synchronize(t, ExtrusionRole::None);
        }
    }

    /// GCodeProcessor.cpp:5204-5207  process_G90 — absolute positioning.
    fn process_g90(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.m_global_positioning_type = EPositioningType::Absolute;
    }

    /// GCodeProcessor.cpp:5209-5212  process_G91 — relative positioning.
    fn process_g91(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.m_global_positioning_type = EPositioningType::Relative;
    }

    /// GCodeProcessor.cpp:5214-5250  process_G92 — set position / origin.
    fn process_g92(&mut self, line: &crate::g_code_reader::GCodeLine) {
        use crate::g_code_reader::Axis;
        let lengths_scale_factor = if self.m_units == EUnits::Inches {
            INCHES_TO_MM
        } else {
            1.0
        };
        let mut any_found = false;

        if line.has_x() {
            self.m_origin[Axis::X as usize] =
                self.m_end_position[Axis::X as usize] - (line.x() * lengths_scale_factor) as f64;
            any_found = true;
        }
        if line.has_y() {
            self.m_origin[Axis::Y as usize] =
                self.m_end_position[Axis::Y as usize] - (line.y() * lengths_scale_factor) as f64;
            any_found = true;
        }
        if line.has_z() {
            self.m_origin[Axis::Z as usize] =
                self.m_end_position[Axis::Z as usize] - (line.z() * lengths_scale_factor) as f64;
            any_found = true;
        }
        if line.has_e() {
            self.m_end_position[Axis::E as usize] = (line.e() * lengths_scale_factor) as f64;
            any_found = true;
        } else {
            self.simulate_st_synchronize(0.0, ExtrusionRole::None);
        }

        if !any_found && !line.has_unknown_axis() {
            for a in 0..=(Axis::E as usize) {
                self.m_origin[a] = self.m_end_position[a];
            }
        }
    }

    /// GCodeProcessor.cpp:5257-5260  process_M82 — extruder absolute mode.
    fn process_m82(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.m_e_local_positioning_type = EPositioningType::Absolute;
    }

    /// GCodeProcessor.cpp:5262-5265  process_M83 — extruder relative mode.
    fn process_m83(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.m_e_local_positioning_type = EPositioningType::Relative;
    }

    /// GCodeProcessor.cpp:5267-5286  process_M104 — set extruder temperature.
    fn process_m104(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let filament_id = self.get_filament_id(true);
        if let Some(phy_extruder_id_temp) = line.has_value(b'T') {
            let phy_extruder_id_temp_int = phy_extruder_id_temp.round() as i32;
            if let Some(s_temp) = line.has_value(b'S') {
                if let Some(extruder_index) = self
                    .m_physical_extruder_map
                    .iter()
                    .position(|&v| v == phy_extruder_id_temp_int)
                {
                    let extruder_index = extruder_index as i32;
                    for _ii in 0..self.m_filament_maps.len() {
                        if let Some(filament_index) =
                            self.m_filament_maps.iter().position(|&v| v == extruder_index)
                        {
                            if filament_index > 0 && filament_index < self.m_extruder_temps.len() {
                                self.m_extruder_temps[filament_index] = s_temp;
                            }
                        }
                    }
                }
            }
        } else if let Some(s_temp) = line.has_value(b'S') {
            self.m_extruder_temps[filament_id as usize] = s_temp;
        }
    }

    /// GCodeProcessor.cpp:5293-5305  process_M106 — set fan speed.
    fn process_m106(&mut self, line: &crate::g_code_reader::GCodeLine) {
        //BBS: for Bambu machine ,we both use M106 P1 and M106 to indicate the part cooling fan
        //So we must not ignore M106 P1
        if !line.has_p() || (line.has_p() && line.p() == 1.0) {
            // The absence of P means the print cooling fan, so ignore anything else.
            if let Some(new_fan_speed) = line.has_value(b'S') {
                self.m_fan_speed = (100.0 / 255.0) * new_fan_speed;
            } else {
                self.m_fan_speed = 100.0;
            }
        } else if crate::faithful_gate("PROCESSOR_AUX_FAN")
            && line.has_p()
            && (line.p() == AUX_FAN_STANDARD_GCODE_INDEX as f32 || line.p() == AUX_FAN_OPTIONAL_GCODE_INDEX as f32)
        {
            // GCodeProcessor.cpp:5497-5507 — auxiliary/side cooling fan. The preview only has a single
            // channel for them, so with two independently driven fans the last one written wins.
            // PROCESSOR_AUX_FAN (on); =0 ignores it.
            if let Some(new_fan_speed) = line.has_value(b'S') {
                self.m_additional_fan_speed = (100.0 / 255.0) * new_fan_speed;
            } else {
                self.m_additional_fan_speed = 100.0;
            }
        }
    }

    /// GCodeProcessor.cpp:5510-5518  process_M107 — disable fan; without P or P1 the part fan, P2 /
    /// P10 the auxiliary side fans. PROCESSOR_AUX_FAN (on); =0 turns the part fan off whatever P is.
    fn process_m107(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("PROCESSOR_AUX_FAN") {
            self.m_fan_speed = 0.0;
            return;
        }
        // M107: disable fan; without P or P1 = part fan, P2 / P10 = auxiliary side fans
        if !line.has_p() || (line.has_p() && line.p() == 1.0) {
            self.m_fan_speed = 0.0;
        } else if line.has_p()
            && (line.p() == AUX_FAN_STANDARD_GCODE_INDEX as f32 || line.p() == AUX_FAN_OPTIONAL_GCODE_INDEX as f32)
        {
            self.m_additional_fan_speed = 0.0;
        }
    }

    /// GCodeProcessor.cpp:5520-5532  process_M108 — the tool change of Sailfish.
    fn process_m108(&mut self, line: &crate::g_code_reader::GCodeLine) {
        // These M-codes are used by Sailfish to change active tool.
        // They have to be processed otherwise toolchanges will be unrecognised
        if self.m_flavor != GCodeFlavor::Sailfish {
            return;
        }

        let cmd = line.raw().to_string();
        if let Some(pos) = cmd.find('T') {
            self.process_t_command(&cmd[pos..], -1);
        }
    }

    /// GCodeProcessor.cpp:5326-5342  process_M109 — set extruder temp + wait.
    fn process_m109(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let filament_id = self.get_filament_id(true);
        if let Some(new_temp) = line.has_value(b'R') {
            if let Some(val) = line.has_value(b'T') {
                let eid = val as usize;
                if eid < self.m_extruder_temps.len() {
                    self.m_extruder_temps[eid] = new_temp;
                }
            } else {
                self.m_extruder_temps[filament_id as usize] = new_temp;
            }
        } else if let Some(new_temp) = line.has_value(b'S') {
            self.m_extruder_temps[filament_id as usize] = new_temp;
        }
    }

    /// GCodeProcessor.cpp:5349-5365  process_M132 — recall home offsets.
    fn process_m132(&mut self, line: &crate::g_code_reader::GCodeLine) {
        use crate::g_code_reader::Axis;
        if line.has(Axis::X) {
            self.m_origin[Axis::X as usize] = 0.0;
        }
        if line.has(Axis::Y) {
            self.m_origin[Axis::Y as usize] = 0.0;
        }
        if line.has(Axis::Z) {
            self.m_origin[Axis::Z as usize] = 0.0;
        }
        if line.has(Axis::E) {
            self.m_origin[Axis::E as usize] = 0.0;
        }
    }

    // BLOCKED(deps): process_M135 (MakerWare tool change) — only for gcfMakerWare.
    /// GCodeProcessor.cpp:5562-5574  process_M135 — the tool change of MakerWare.
    fn process_m135(&mut self, line: &crate::g_code_reader::GCodeLine) {
        // These M-codes are used by MakerWare to change active tool.
        // They have to be processed otherwise toolchanges will be unrecognised
        if self.m_flavor != GCodeFlavor::MakerWare {
            return;
        }

        let cmd = line.raw().to_string();
        if let Some(pos) = cmd.find('T') {
            self.process_t_command(&cmd[pos..], -1);
        }
    }

    /// GCodeProcessor.cpp:5381-5386  process_M140 — set bed temperature.
    fn process_m140(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if let Some(new_temp) = line.has_value(b'S') {
            let nt = new_temp as i32;
            self.m_highest_bed_temp = if self.m_highest_bed_temp < nt {
                nt
            } else {
                self.m_highest_bed_temp
            };
        }
    }

    /// GCodeProcessor.cpp:5388-5393  process_M190 — wait bed temperature.
    fn process_m190(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if let Some(new_temp) = line.has_value(b'S') {
            let nt = new_temp as i32;
            self.m_highest_bed_temp = if self.m_highest_bed_temp < nt {
                nt
            } else {
                self.m_highest_bed_temp
            };
        }
    }

    /// GCodeProcessor.cpp:5395-5402  process_M191 — wait chamber temperature.
    fn process_m191(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let wait_chamber_temp_time = 720.0;
        if let Some(chamber_temp) = line.has_value(b'S') {
            if chamber_temp > 40.0 {
                self.simulate_st_synchronize(wait_chamber_temp_time, ExtrusionRole::None);
            }
        }
    }

    /// GCodeProcessor.cpp:5600-5617  process_M201 — max printing acceleration: the
    /// entries of the first `size() / 2` extruders the loop reaches (its index steps
    /// by 2 below `size() / 2`), the Normal mode's and, with envelope processing,
    /// the Stealth mode's. MACHINE_LIMITS (on); =0 ignores the line.
    fn process_m201(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("MACHINE_LIMITS") {
            return;
        }
        // see http://reprap.org/wiki/G-code#M201:_Set_max_printing_acceleration
        let factor = if !matches!(self.m_flavor, GCodeFlavor::RepRapSprinter | GCodeFlavor::RepRapFirmware)
            && self.m_units == EUnits::Inches
        {
            INCHES_TO_MM
        } else {
            1.0
        };
        let envelope = self.m_time_processor.machine_envelope_processing_enabled;
        let ml = &mut self.m_time_processor.machine_limits;
        let indx_limit = ml.machine_max_acceleration_x.len() / 2;
        let mut index = 0usize;
        while index < indx_limit {
            for i in 0..ETimeMode::COUNT {
                if i == ETimeMode::Normal as usize || envelope {
                    if line.has_x() {
                        ml_set(&mut ml.machine_max_acceleration_x, index + i, line.x() * factor);
                    }
                    if line.has_y() {
                        ml_set(&mut ml.machine_max_acceleration_y, index + i, line.y() * factor);
                    }
                    if line.has_z() {
                        ml_set(&mut ml.machine_max_acceleration_z, index + i, line.z() * factor);
                    }
                    if line.has_e() {
                        ml_set(&mut ml.machine_max_acceleration_e, index + i, line.e() * factor);
                    }
                }
            }
            index += 2;
        }
    }

    /// GCodeProcessor.cpp:5619-5649  process_M203 — maximum feedrate, mm/s on
    /// Marlin, Smoothie and Klipper, mm/min elsewhere (MACHINE_LIMITS, as M201).
    fn process_m203(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("MACHINE_LIMITS") {
            return;
        }
        // see http://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate
        if self.m_flavor == GCodeFlavor::RepetierFirmware {
            return;
        }
        // http://smoothieware.org/supported-g-codes
        let factor = if matches!(
            self.m_flavor,
            GCodeFlavor::MarlinLegacy | GCodeFlavor::MarlinFirmware | GCodeFlavor::Smoothie | GCodeFlavor::Klipper
        ) {
            1.0
        } else {
            MMMIN_TO_MMSEC
        };
        let envelope = self.m_time_processor.machine_envelope_processing_enabled;
        let ml = &mut self.m_time_processor.machine_limits;
        //BBS:
        let indx_limit = ml.machine_max_speed_x.len() / 2;
        let mut index = 0usize;
        while index < indx_limit {
            for i in 0..ETimeMode::COUNT {
                if i == ETimeMode::Normal as usize || envelope {
                    if line.has_x() {
                        ml_set(&mut ml.machine_max_speed_x, index + i, line.x() * factor);
                    }
                    if line.has_y() {
                        ml_set(&mut ml.machine_max_speed_y, index + i, line.y() * factor);
                    }
                    if line.has_z() {
                        ml_set(&mut ml.machine_max_speed_z, index + i, line.z() * factor);
                    }
                    if line.has_e() {
                        ml_set(&mut ml.machine_max_speed_e, index + i, line.e() * factor);
                    }
                }
            }
            index += 2;
        }
    }

    /// GCodeProcessor.cpp:5456-5483  process_M204 — set default acceleration.
    fn process_m204(&mut self, line: &crate::g_code_reader::GCodeLine) {
        for i in 0..ETimeMode::COUNT {
            let mode = if i == 0 {
                ETimeMode::Normal
            } else {
                ETimeMode::Stealth
            };
            if mode == ETimeMode::Normal
                || self.m_time_processor.machine_envelope_processing_enabled
            {
                if let Some(value) = line.has_value(b'S') {
                    // Legacy acceleration format.
                    self.set_acceleration(mode, value);
                    self.set_travel_acceleration(mode, value);
                    if let Some(t) = line.has_value(b'T') {
                        self.set_retract_acceleration(mode, t);
                    }
                } else {
                    // New acceleration format.
                    if let Some(p) = line.has_value(b'P') {
                        self.set_acceleration(mode, p);
                    }
                    if let Some(r) = line.has_value(b'R') {
                        self.set_retract_acceleration(mode, r);
                    }
                    if let Some(t) = line.has_value(b'T') {
                        self.set_travel_acceleration(mode, t);
                    }
                }
            }
        }
    }

    /// GCodeProcessor.cpp:5680-5708  process_M205 — jerk and minimum rates, the
    /// Normal mode's and, with envelope processing, the Stealth mode's
    /// (MACHINE_LIMITS).
    fn process_m205(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("MACHINE_LIMITS") {
            return;
        }
        let envelope = self.m_time_processor.machine_envelope_processing_enabled;
        let ml = &mut self.m_time_processor.machine_limits;
        for i in 0..ETimeMode::COUNT {
            if i == ETimeMode::Normal as usize || envelope {
                if line.has_x() {
                    let max_jerk = line.x();
                    ml_set(&mut ml.machine_max_jerk_x, i, max_jerk);
                    ml_set(&mut ml.machine_max_jerk_y, i, max_jerk);
                }
                if line.has_y() {
                    ml_set(&mut ml.machine_max_jerk_y, i, line.y());
                }
                if line.has_z() {
                    ml_set(&mut ml.machine_max_jerk_z, i, line.z());
                }
                if line.has_e() {
                    ml_set(&mut ml.machine_max_jerk_e, i, line.e());
                }
                if let Some(value) = line.has_value(b'S') {
                    ml_set(&mut ml.machine_min_extruding_rate, i, value);
                }
                if let Some(value) = line.has_value(b'T') {
                    ml_set(&mut ml.machine_min_travel_rate, i, value);
                }
            }
        }
    }

    /// GCodeProcessor.cpp:5515-5561  process_SET_VELOCITY_LIMIT (Klipper).
    /// BLOCKED(config): writes jerk/speed limit arrays (empty/unthreaded) and
    /// acceleration. We port only the ACCEL component (which sets acceleration
    /// on the machines, a real effect); jerk/velocity writes are no-ops.
    fn process_set_velocity_limit(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let raw = line.raw();
        // KLIPPER_SVL_CPP (on): GCodeProcessor.cpp:5710-5762 as written: SQUARE_CORNER_VELOCITY sets
        // the X and Y jerks, ACCEL the accelerations, VELOCITY the X and Y speeds, each found by
        // `std::regex_search` and read by `std::stof` (0 when it throws). =0 reads ACCEL only, from
        // the first occurrence of the key.
        if crate::faithful_gate("KLIPPER_SVL_CPP") {
            // handle SQUARE_CORNER_VELOCITY
            if let Some(jerk) = klipper_regex_value(raw, "SQUARE_CORNER_VELOCITY") {
                let ml = &mut self.m_time_processor.machine_limits;
                for i in 0..ETimeMode::COUNT {
                    ml_set(&mut ml.machine_max_jerk_x, i, jerk);
                    ml_set(&mut ml.machine_max_jerk_y, i, jerk);
                }
            }
            if let Some(accl) = klipper_regex_value(raw, "ACCEL") {
                for i in 0..ETimeMode::COUNT {
                    let mode = if i == 0 { ETimeMode::Normal } else { ETimeMode::Stealth };
                    self.set_acceleration(mode, accl);
                    self.set_travel_acceleration(mode, accl);
                }
            }
            if let Some(speed) = klipper_regex_value(raw, "VELOCITY") {
                let ml = &mut self.m_time_processor.machine_limits;
                for i in 0..ETimeMode::COUNT {
                    ml_set(&mut ml.machine_max_speed_x, i, speed);
                    ml_set(&mut ml.machine_max_speed_y, i, speed);
                }
            }
            return;
        }
        if let Some(accl) = parse_klipper_kv(raw, "ACCEL") {
            for i in 0..ETimeMode::COUNT {
                let mode = if i == 0 {
                    ETimeMode::Normal
                } else {
                    ETimeMode::Stealth
                };
                self.set_acceleration(mode, accl);
                self.set_travel_acceleration(mode, accl);
            }
        }
    }

    /// GCodeProcessor.cpp:5563-5573  process_M221 — extrude factor override %.
    fn process_m221(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let value_s = line.has_value(b'S');
        let value_t = line.has_value(b'T');
        if let Some(mut s) = value_s {
            if value_t.is_none() {
                s *= 0.01;
                for i in 0..ETimeMode::COUNT {
                    self.m_time_processor.machines[i].extrude_factor_override_percentage = s;
                }
            }
        }
    }

    /// GCodeProcessor.cpp:5593-5601  process_M400 — BBS dwell -> st_synchronize.
    fn process_m400(&mut self, line: &crate::g_code_reader::GCodeLine) {
        // PROCESSOR_M400_CPP (on): `line.has_value('S', value_s) || line.has_value('P', value_p)` reads
        // P only without S, and `value_s += value_p * 0.001` multiplies by the double 0.001
        // (GCodeProcessor.cpp:5808-5811). =0 adds both in float.
        if crate::faithful_gate("PROCESSOR_M400_CPP") {
            let mut value_s = 0.0f32;
            let mut value_p = 0.0f32;
            let has = match line.has_value(b'S') {
                Some(v) => {
                    value_s = v;
                    true
                }
                None => match line.has_value(b'P') {
                    Some(v) => {
                        value_p = v;
                        true
                    }
                    None => false,
                },
            };
            if has {
                value_s = (value_s as f64 + value_p as f64 * 0.001) as f32;
                self.simulate_st_synchronize(value_s, ExtrusionRole::None);
            }
            return;
        }
        let value_s = line.has_value(b'S');
        let value_p = line.has_value(b'P');
        if value_s.is_some() || value_p.is_some() {
            let mut s = value_s.unwrap_or(0.0);
            s += value_p.unwrap_or(0.0) * 0.001;
            self.simulate_st_synchronize(s, ExtrusionRole::None);
        }
    }

    /// GCodeProcessor.cpp:5814-5823  process_M401 — Repetier: store the position.
    fn process_m401(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        if self.m_flavor != GCodeFlavor::RepetierFirmware {
            return;
        }

        for a in 0..=3usize {
            self.m_cached_position.position[a] = self.m_start_position[a];
        }
        self.m_cached_position.feedrate = self.m_feedrate;
    }

    /// GCodeProcessor.cpp:5825-5855  process_M402 — Repetier: go to the stored position. `line.has(a)`
    /// with an `unsigned char` axis calls `has(char)` (a character of code 0..2, never in a line),
    /// and `line.has_value(4, p)` looks for the character of code 4: neither matches, as in C++.
    fn process_m402(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if self.m_flavor != GCodeFlavor::RepetierFirmware {
            return;
        }

        // see for reference:
        // https://github.com/repetier/Repetier-Firmware/blob/master/src/ArduinoAVR/Repetier/Printer.cpp
        // void Printer::GoToMemoryPosition(bool x, bool y, bool z, bool e, float feed)

        let has_xyz = !(line.has_char(b'X') || line.has_char(b'Y') || line.has_char(b'Z'));

        let mut p: f32;
        for a in 0..=2u8 {
            if has_xyz || line.has_char(a) {
                p = self.m_cached_position.position[a as usize] as f32;
                if p != f32::MAX {
                    self.m_start_position[a as usize] = p as f64;
                }
            }
        }

        p = self.m_cached_position.position[3] as f32;
        if p != f32::MAX {
            self.m_start_position[3] = p as f64;
        }

        p = match line.has_value(4) {
            Some(v) => v,
            None => self.m_cached_position.feedrate,
        };

        if p != f32::MAX {
            self.m_feedrate = p;
        }
    }

    /// GCodeProcessor.cpp:5857-5872  process_M566 — instantaneous speed change,
    /// mm/min, both modes (MACHINE_LIMITS).
    fn process_m566(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("MACHINE_LIMITS") {
            return;
        }
        let ml = &mut self.m_time_processor.machine_limits;
        for i in 0..ETimeMode::COUNT {
            if line.has_x() {
                ml_set(&mut ml.machine_max_jerk_x, i, line.x() * MMMIN_TO_MMSEC);
            }
            if line.has_y() {
                ml_set(&mut ml.machine_max_jerk_y, i, line.y() * MMMIN_TO_MMSEC);
            }
            if line.has_z() {
                ml_set(&mut ml.machine_max_jerk_z, i, line.z() * MMMIN_TO_MMSEC);
            }
            if line.has_e() {
                ml_set(&mut ml.machine_max_jerk_e, i, line.e() * MMMIN_TO_MMSEC);
            }
        }
    }

    /// GCodeProcessor.cpp:5663-5673  process_M702 — MMU unload at end of print.
    fn process_m702(&mut self, line: &crate::g_code_reader::GCodeLine) {
        // C++ checks line.has('C'); 'C' is not a recognized axis char in the
        // Rust reader's has(Axis) API, but has_char covers it.
        // R953 gate PROCESS_M702_PIN (faithful): GCodeProcessor.cpp:5874-5886 as the pin writes it:
        // `int filament_id = get_filament_id();` before the test, the unload time from
        // get_filament_unload_time(filament_id) and `unload_time_per_filament[filament_id] +=` it
        // (std::map::operator[] creates the entry even when the time is 0); =0 the earlier arm
        // below, which leaves the statistic out.
        if crate::faithful_gate("PROCESS_M702_PIN") {
            let filament_id = self.get_filament_id(true);
            if line.has_char(b'C') {
                self.m_time_processor.extruder_unloaded = true;
                let unload_time = self.get_filament_unload_time(filament_id.max(0) as usize);
                if filament_id >= 0 {
                    *self
                        .m_result
                        .print_statistics
                        .unload_time_per_filament
                        .entry(filament_id as usize)
                        .or_insert(0.0) += unload_time as f64;
                }
                self.simulate_st_synchronize(unload_time, ExtrusionRole::None);
            }
            return;
        }
        if line.has_char(b'C') {
            self.m_time_processor.extruder_unloaded = true;
            // get_filament_unload_time => extruder_unloaded ? 0 : filament_unload_times
            let t = if self.m_time_processor.extruder_unloaded {
                0.0
            } else {
                self.m_time_processor.filament_unload_times
            };
            self.simulate_st_synchronize(t, ExtrusionRole::None);
        }
    }

    // BLOCKED(deps): process_M1020 (Select Tool) + process_T need the
    // MultiNozzleUtils nozzle group / filament-change pipeline. The C++ updates
    // m_extruder_id / m_filament_id from tool selection. Not ported (default
    // single-extruder path keeps m_extruder_id at its initialized value).
    /// GCodeProcessor.cpp:5919-5958  process_M1020.
    fn process_m1020(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("PROCESS_FILAMENT_CHANGE") {
            return;
        }
        let mut eid: i32 = -1;
        if let Some(val) = line.has_value(b'S') {
            eid = val as i32;
        } else if line.raw().len() > 5 {
            // :5929 — `line.raw().substr(7)`, then `std::stoi`.
            let filament_id_str = line.raw().get(7..).unwrap_or("");
            if !filament_id_str.is_empty() {
                if let Some(v) = parse_stoi(filament_id_str) {
                    eid = v;
                }
            }
        }
        if eid == -1 {
            return;
        }
        if eid < 0 || eid > 254 {
            // :5942-5945 — an error is logged.
        } else {
            if eid as usize >= self.m_result.filaments_count {
                return;
            }
            let nozzle_id = line.has_value(b'H').map(|v| v as i32).unwrap_or(-1);
            self.process_filament_change(eid, nozzle_id);
        }
    }

    /// GCodeProcessor.cpp:5909-5917  process_T(line).
    fn process_t(&mut self, line: &crate::g_code_reader::GCodeLine) {
        if !crate::faithful_gate("PROCESS_FILAMENT_CHANGE") {
            return;
        }
        let nozzle_id = line.has_value(b'H').map(|v| v as i32).unwrap_or(-1);
        let cmd = line.cmd().to_string();
        self.process_t_command(&cmd, nozzle_id);
    }

    /// GCodeProcessor.cpp:5960-5984  process_T(command, nozzle_id).
    fn process_t_command(&mut self, command: &str, nozzle_id: i32) {
        if command.len() > 1 {
            let parsed = parse_number_i32(&command[1..]);
            let eid = parsed.unwrap_or(0);
            if parsed.is_none() || eid < 0 || eid > 254 {
                // :5966-5969 — T255, T1000 and T1100 are special commands on a
                // Bambu Lab machine and cost no time.
                if matches!(self.m_flavor, GCodeFlavor::MarlinLegacy | GCodeFlavor::MarlinFirmware)
                    && (command == "Tx"
                        || command == "Tc"
                        || command == "T?"
                        || eid == 1000
                        || eid == 1100
                        || eid == 255)
                {
                    return;
                }
                // :5971-5973 — an error is logged.
            } else {
                if eid as usize >= self.m_result.filaments_count {
                    return;
                }
                self.process_filament_change(eid, nozzle_id);
            }
        }
    }

    /// GCodeProcessor.cpp:5993-6052  ensure_nozzle_group_result: without a
    /// nozzle group result from the print, one nozzle per extruder of
    /// `filament_map`.
    fn ensure_nozzle_group_result(&mut self, min_filament_count: i32) {
        use crate::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleInfo};
        if self.m_nozzle_group_result.is_some() {
            return;
        }
        let mut filament_count = min_filament_count.max(1);
        filament_count = filament_count.max(self.m_filament_maps.len() as i32);

        let mut filament_map = self.m_filament_maps.clone();
        if filament_map.is_empty() {
            filament_map = vec![0; filament_count as usize];
        } else if (filament_map.len() as i32) < filament_count {
            let front = filament_map[0];
            filament_map.resize(filament_count as usize, front);
        }

        let min_value = *filament_map.iter().min().unwrap();
        if min_value >= 1 {
            for value in filament_map.iter_mut() {
                *value -= 1;
            }
        }
        for value in filament_map.iter_mut() {
            *value = (*value).max(0);
        }

        let mut max_extruder_id = *filament_map.iter().max().unwrap();
        max_extruder_id = max_extruder_id.max(0);

        let nozzle_diameter =
            crate::utils::format_diameter_to_str_default(DEFAULT_TOOLPATH_WIDTH as f64);
        let mut nozzle_list: Vec<NozzleInfo> = Vec::with_capacity(max_extruder_id as usize + 1);
        for extruder_id in 0..=max_extruder_id {
            let mut info = NozzleInfo::default();
            info.diameter = nozzle_diameter.clone();
            info.volume_type = crate::extruder::NozzleVolumeType::NvtStandard;
            info.extruder_id = extruder_id;
            info.group_id = extruder_id;
            nozzle_list.push(info);
        }

        let filament_nozzle_map: Vec<i32> =
            (0..filament_count as usize).map(|i| filament_map[i]).collect();
        let used_filaments: Vec<u32> = (0..filament_count as u32).collect();

        if let Some(result) =
            LayeredNozzleGroupResult::create(&filament_nozzle_map, &nozzle_list, &used_filaments)
        {
            self.m_nozzle_group_result = Some(result);
        }
    }

    /// GCodeProcessor.cpp:6357-6361  get_filament_load_time.
    fn get_filament_load_time(&self, _extruder_id: usize) -> f32 {
        if self.m_time_processor.extruder_unloaded {
            0.0
        } else {
            self.m_time_processor.filament_load_times
        }
    }

    /// GCodeProcessor.cpp:6363-6367  get_filament_unload_time.
    fn get_filament_unload_time(&self, _extruder_id: usize) -> f32 {
        if self.m_time_processor.extruder_unloaded {
            0.0
        } else {
            self.m_time_processor.filament_unload_times
        }
    }

    /// GCodeProcessor.cpp:6369-6373  get_extruder_change_time.
    fn get_extruder_change_time(&self, _extruder_id: usize) -> f32 {
        self.m_time_processor.extruder_change_times
    }

    /// GCodeProcessor.cpp:6054-6125  process_filament_change.
    fn process_filament_change(&mut self, id: i32, nozzle_id: i32) {
        let prev_extruder_id = self.get_extruder_id(false);
        let prev_filament_id = self.get_filament_id(false);
        let mut extra_time: f32 = 0.0;

        if prev_filament_id == id && nozzle_id == -1 {
            return;
        }

        if prev_extruder_id != -1 {
            self.m_last_filament_id[prev_extruder_id as usize] = prev_filament_id as u8;
        }

        let Some(group) = self.m_nozzle_group_result.as_ref() else {
            return;
        };
        let mut target_nozzle_info = None;
        if nozzle_id != -1 {
            target_nozzle_info = group.get_nozzle_from_id(nozzle_id);
        }
        if target_nozzle_info.is_none() {
            let used_nozzles = group.get_nozzles_for_filament(id);
            if used_nozzles.is_empty() {
                return;
            }
            target_nozzle_info = used_nozzles.into_iter().next();
        }
        let Some(target_nozzle_info) = target_nozzle_info else {
            return;
        };

        let new_extruder_id = target_nozzle_info.extruder_id;
        let old_extruder_id = prev_extruder_id;

        let new_nozzle_id_in_extruder = target_nozzle_info.group_id;
        let old_nozzle_id_in_extruder =
            self.m_nozzle_status_recorder.get_nozzle_in_extruder(new_extruder_id);

        let new_filament_id = id;
        let old_filament_in_nozzle =
            self.m_nozzle_status_recorder.get_filament_in_nozzle(new_nozzle_id_in_extruder);
        let old_filament_in_extruder =
            self.m_nozzle_status_recorder.get_filament_in_nozzle(old_nozzle_id_in_extruder);

        let nozzle_switch_on_target_extruder =
            new_nozzle_id_in_extruder != old_nozzle_id_in_extruder;
        let nozzle_in_extruder_change =
            !self.m_handle_hotend_as_extruder && nozzle_switch_on_target_extruder;
        let filament_in_nozzle_change = new_filament_id != old_filament_in_nozzle;
        let extruder_change = (new_extruder_id != old_extruder_id)
            || (self.m_handle_hotend_as_extruder && nozzle_switch_on_target_extruder);

        if extruder_change && old_extruder_id != -1 {
            extra_time += self.get_extruder_change_time(new_extruder_id as usize);
        }
        if nozzle_in_extruder_change || filament_in_nozzle_change {
            if old_filament_in_extruder >= 0 {
                let unload_time = self.get_filament_unload_time(old_filament_in_extruder as usize);
                extra_time += unload_time;
                *self
                    .m_result
                    .print_statistics
                    .unload_time_per_filament
                    .entry(old_filament_in_extruder as usize)
                    .or_insert(0.0) += unload_time as f64;
            }
            self.m_time_processor.extruder_unloaded = false;
            let load_time = self.get_filament_load_time(new_filament_id as usize);
            extra_time += load_time;
            if new_filament_id >= 0 {
                *self
                    .m_result
                    .print_statistics
                    .load_time_per_filament
                    .entry(new_filament_id as usize)
                    .or_insert(0.0) += load_time as f64;
            }
            if filament_in_nozzle_change && old_filament_in_nozzle != -1 {
                self.m_result.print_statistics.total_flush_filament_changes += 1;
            }
        }
        if prev_filament_id != -1 {
            self.m_result.print_statistics.total_filament_changes += 1;
        }

        self.process_filaments_tool_change();

        if new_extruder_id != -1 {
            self.m_filament_id[new_extruder_id as usize] = new_filament_id as u8;
        }
        self.m_extruder_id = new_extruder_id as u8;

        self.m_nozzle_status_recorder.set_nozzle_status(
            new_nozzle_id_in_extruder,
            new_filament_id,
            new_extruder_id,
        );

        self.m_cp_color.current = self.m_extruder_colors[new_filament_id as usize];
        // store tool change move
        self.store_move_vertex(EMoveType::ToolChange, EMovePathType::NoopMove);

        // construct a new time block to handle filament change
        for i in 0..ETimeMode::COUNT {
            if !self.m_time_processor.machines[i].enabled {
                continue;
            }
            let mut block = TimeBlock::default();
            block.skippable_type = self.m_skippable_type;
            block.move_id = (self.m_result.moves.len() - 1) as u32;
            block.role = ExtrusionRole::Flush;
            block.move_type = EMoveType::ToolChange;
            block.layer_id = self.m_layer_id.max(1);
            block.g1_line_id = self.m_g1_line_id;
            block.flags.prepare_stage = self.m_processing_start_custom_gcode;
            block.distance = 0.0;
            block.calculate_trapezoid();
            self.m_time_processor.machines[i].blocks.push(block);
        }

        self.simulate_st_synchronize(extra_time, ExtrusionRole::Flush);
    }

    /// GCodeProcessor.cpp:6131-6145  process_filaments(CustomGCode::ToolChange).
    fn process_filaments_tool_change(&mut self) {
        let active_filament_id = self.get_filament_id(true);
        let active_filament_id_support = self.get_filament_id(false);
        self.m_used_filaments.process_model_cache(active_filament_id);
        self.m_used_filaments.process_support_cache(active_filament_id_support);
        self.m_used_filaments.process_total_volume_cache(active_filament_id);
        // BBS: reset remaining filament
        let last_extruder_id = self.get_extruder_id(true) as usize;
        if last_extruder_id < self.m_remaining_volume.len() {
            self.m_remaining_volume[last_extruder_id] =
                self.m_nozzle_volume.get(last_extruder_id).copied().unwrap_or(0.0);
        }
    }

    /// GCodeProcessor.cpp:5676-5692  process_SYNC — flush/prepare time.
    fn process_sync(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let time_role_int = match line.has_value(b'R') {
            Some(r) => r.round() as i32,
            // absence of 'R' interpreted as a flush command.
            None => 1,
        };
        if let Some(time) = line.has_value(b'T') {
            if time_role_int == 1 {
                self.simulate_st_synchronize(time, ExtrusionRole::Flush);
            } else {
                self.simulate_st_synchronize(time, ExtrusionRole::None);
            }
        }
    }

    /// GCodeProcessor.cpp:6199-6210  void GCodeProcessor::simulate_st_synchronize(...)
    fn simulate_st_synchronize(&mut self, additional_time: f32, target_role: ExtrusionRole) {
        for i in 0..ETimeMode::COUNT {
            // machine.simulate_st_synchronize(additional_time, target_role, handler)
            self.run_calculate_time(i, 0, additional_time, target_role);
        }
    }

    /// Shared driver for `TimeMachine::calculate_time` + `handle_time_block`.
    /// `handle_time_block` writes into `m_result.moves[block.move_id].time` and
    /// `m_result.skippable_part_time`; we run the planner against a detached
    /// blocks list and apply the handler here (the borrow of `m_result` and the
    /// machine cannot overlap inside one closure in Rust).
    fn run_calculate_time(
        &mut self,
        machine_idx: usize,
        keep_last_n_blocks: usize,
        additional_time: f32,
        target_role: ExtrusionRole,
    ) {
        if !self.m_time_processor.machines[machine_idx].enabled {
            return;
        }
        // Collect (move_id, time, skippable_type, block_time) emitted by the
        // handler, then apply to m_result after the machine borrow ends.
        let mut emitted: Vec<(usize, f32, SkipType, f32)> = Vec::new();
        {
            let machine = &mut self.m_time_processor.machines[machine_idx];
            machine.calculate_time(
                keep_last_n_blocks,
                additional_time,
                target_role,
                |block, time| {
                    emitted.push((block.move_id as usize, time, block.skippable_type, block.time()));
                },
            );
        }
        // GCodeProcessor.cpp:409-414  handle_time_block
        for (move_id, time, skippable_type, block_time) in emitted {
            if skippable_type != SkipType::StNone {
                *self
                    .m_result
                    .skippable_part_time
                    .entry(skippable_type)
                    .or_insert(0.0) += block_time;
            }
            if move_id < self.m_result.moves.len() {
                // PROCESSOR_MOVE_TIME_INT (on): the C++ handlers are lambdas `(const TimeBlock& block,
                // int time)` wrapped in `block_handler_t` (`std::function<void(const TimeBlock&, const
                // float)>`, GCodeProcessor.hpp:699), so the time a move stores is truncated to whole
                // seconds (GCodeProcessor.cpp:2807, 6388, 6422). =0 stores it whole.
                let time = if crate::faithful_gate("PROCESSOR_MOVE_TIME_INT") { (time as i32) as f32 } else { time };
                self.m_result.moves[move_id].time[machine_idx] = time;
            }
        }
    }

    /// GCodeProcessor.cpp:3936-4337  void GCodeProcessor::process_G1(line)
    fn process_g1(&mut self, line: &crate::g_code_reader::GCodeLine) {
        use crate::g_code_reader::Axis;
        // PROCESSOR_HELIO_COMMENT (on): the thermal index of the line's comment (it only reaches the
        // moves of the result). =0 leaves it alone.
        if crate::faithful_gate("PROCESSOR_HELIO_COMMENT") {
            self.process_helioadditive_comment(line);
        }

        let filament_id = self.get_filament_id(true);
        let _last_filament_id = self.get_last_filament_id(false);
        let filament_diameter = if (filament_id as usize) < self.m_result.filament_diameters.len() {
            self.m_result.filament_diameters[filament_id as usize]
        } else {
            *self.m_result.filament_diameters.last().unwrap()
        };
        let filament_radius = 0.5 * filament_diameter;
        let area_filament_cross_section = (PI as f32) * sqr(filament_radius);

        // absolute_position lambda (cpp:3945-3957)
        let global_relative = self.m_global_positioning_type == EPositioningType::Relative;
        let e_relative = self.m_e_local_positioning_type == EPositioningType::Relative;
        let units_inches = self.m_units == EUnits::Inches;
        let start_position = self.m_start_position;
        let origin = self.m_origin;
        let absolute_position = |axis: usize, lineg1: &crate::g_code_reader::GCodeLine| -> f64 {
            let mut is_relative = global_relative;
            if axis == Axis::E as usize {
                is_relative |= e_relative;
            }
            let ax = match axis {
                0 => Axis::X,
                1 => Axis::Y,
                2 => Axis::Z,
                _ => Axis::E,
            };
            if lineg1.has(ax) {
                let length_scale = if units_inches { INCHES_TO_MM } else { 1.0 };
                let ret = (lineg1.value(ax) * length_scale) as f64;
                if is_relative {
                    start_position[axis] + ret
                } else {
                    origin[axis] + ret
                }
            } else {
                start_position[axis]
            }
        };

        self.m_g1_line_id += 1;
        // enable processing of lines M201/M203/M204/M205
        self.m_time_processor.machine_envelope_processing_enabled = true;

        // updates axes positions from line (X..=E)
        for a in 0..=(Axis::E as usize) {
            self.m_end_position[a] = absolute_position(a, line);
        }

        // updates feedrate from line, if present
        if line.has_f() {
            self.m_feedrate = line.f() * MMMIN_TO_MMSEC;
        }

        // calculates movement deltas
        let mut max_abs_delta: f32 = 0.0;
        let mut delta_pos: AxisCoords = [0.0; 4];
        for a in 0..=(Axis::E as usize) {
            delta_pos[a] = self.m_end_position[a] - self.m_start_position[a];
            max_abs_delta = max_abs_delta.max((delta_pos[a] as f32).abs());
        }

        // no displacement, return
        if max_abs_delta == 0.0 {
            return;
        }

        let dx = delta_pos[Axis::X as usize];
        let dy = delta_pos[Axis::Y as usize];
        let dz = delta_pos[Axis::Z as usize];
        let de = delta_pos[Axis::E as usize];

        // move_type lambda (cpp:3959-3976)
        let move_type = |dp: &AxisCoords| -> EMoveType {
            let (px, py, pz, pe) = (
                dp[Axis::X as usize],
                dp[Axis::Y as usize],
                dp[Axis::Z as usize],
                dp[Axis::E as usize],
            );
            if self.m_wiping {
                EMoveType::Wipe
            } else if pe < 0.0 {
                if px != 0.0 || py != 0.0 || pz != 0.0 {
                    EMoveType::Travel
                } else {
                    EMoveType::Retract
                }
            } else if pe > 0.0 {
                if px == 0.0 && py == 0.0 {
                    if pz == 0.0 {
                        EMoveType::Unretract
                    } else {
                        EMoveType::Travel
                    }
                } else if px != 0.0 || py != 0.0 {
                    EMoveType::Extrude
                } else {
                    EMoveType::Noop
                }
            } else if px != 0.0 || py != 0.0 || pz != 0.0 {
                EMoveType::Travel
            } else {
                EMoveType::Noop
            }
        };

        let r#type = move_type(&delta_pos);
        if r#type == EMoveType::Extrude {
            // C++: float delta_xyz = std::sqrt(sqr(dpX)+sqr(dpY)+sqr(dpZ));
            // delta_pos is std::array<double,4>, so sqr/sum/sqrt are double.
            let delta_xyz = (dx * dx + dy * dy + dz * dz).sqrt() as f32;
            // C++: float vol = area_filament_cross_section * dpE; (float*double->double)
            let volume_extruded_filament = (area_filament_cross_section as f64 * de) as f32;
            let area_toolpath_cross_section = volume_extruded_filament / delta_xyz;

            if self.m_extrusion_role == ExtrusionRole::SupportMaterial
                || self.m_extrusion_role == ExtrusionRole::SupportMaterialInterface
                || self.m_extrusion_role == ExtrusionRole::SupportTransition
            {
                self.m_used_filaments
                    .increase_support_caches(volume_extruded_filament as f64);
            } else if self.m_extrusion_role == ExtrusionRole::WipeTower {
                self.m_used_filaments
                    .increase_wipe_tower_caches(volume_extruded_filament as f64);
            } else {
                self.m_used_filaments
                    .increase_model_caches(volume_extruded_filament as f64);
            }
            self.m_mm3_per_mm = area_toolpath_cross_section;

            if self.m_forced_height > 0.0 {
                self.m_height = self.m_forced_height;
            } else if self.m_end_position[Axis::Z as usize]
                > (self.m_extruded_last_z + EPSILON) as f64
            {
                self.m_height =
                    (self.m_end_position[Axis::Z as usize] - self.m_extruded_last_z as f64) as f32;
            }
            if self.m_height == 0.0 {
                self.m_height = DEFAULT_TOOLPATH_HEIGHT;
            }
            if self.m_end_position[Axis::Z as usize] == 0.0 {
                self.m_end_position[Axis::Z as usize] = self.m_height as f64;
            }
            self.m_extruded_last_z = self.m_end_position[Axis::Z as usize] as f32;
            // m_options_z_corrector.update(m_height); — BLOCKED(deps): no-op.

            if self.m_forced_width > 0.0 {
                self.m_width = self.m_forced_width;
            } else if self.m_extrusion_role == ExtrusionRole::ExternalPerimeter {
                // C++: m_width = delta_pos[E] * static_cast<float>(M_PI*sqr(1.05f*fr))
                //               / (delta_xyz * m_height);
                // delta_pos[E] is double; the cast constant is float; division promotes
                // to double; the result is cast to float on assignment.
                let c1 = (PI * sqr(1.05 * filament_radius) as f64) as f32;
                self.m_width =
                    (de * c1 as f64 / (delta_xyz * self.m_height) as f64) as f32;
            } else if self.m_extrusion_role == ExtrusionRole::BridgeInfill
                || self.m_extrusion_role == ExtrusionRole::None
            {
                // C++: m_width = (float)filament_diameters[id] * std::sqrt(dpE/delta_xyz);
                // dpE/delta_xyz is double; sqrt is double; float*double -> double -> float.
                let fd = self.m_result.filament_diameters[filament_id as usize];
                self.m_width = (fd as f64 * (de / delta_xyz as f64).sqrt()) as f32;
            } else {
                // C++: m_width = delta_pos[E] * static_cast<float>(M_PI*sqr(fr))
                //               / (delta_xyz * m_height)
                //               + static_cast<float>(1.0 - 0.25*M_PI) * m_height;
                let c2 = (PI * sqr(filament_radius) as f64) as f32;
                let c3 = (1.0 - 0.25 * PI) as f32;
                self.m_width = (de * c2 as f64 / (delta_xyz * self.m_height) as f64
                    + (c3 * self.m_height) as f64) as f32;
            }
            if self.m_width == 0.0 {
                self.m_width = DEFAULT_TOOLPATH_WIDTH;
            }
            self.m_width = self.m_width.min(2.0_f32.max(4.0 * self.m_height));
        } else if r#type == EMoveType::Unretract && self.m_flushing {
            // BLOCKED(deps): flushing path needs get_extruder_id + remaining_volume
            // bookkeeping. Ported faithfully below using current state.
            let extruder_id = self.get_extruder_id(true).max(0) as usize;
            let volume_flushed_filament = area_filament_cross_section * de as f32;
            let last_filament_id = self.get_last_filament_id(false);
            if extruder_id < self.m_remaining_volume.len()
                && self.m_remaining_volume[extruder_id] > volume_flushed_filament
            {
                if last_filament_id != -1 {
                    self.m_used_filaments
                        .update_flush_per_filament(last_filament_id as usize, volume_flushed_filament);
                }
                self.m_remaining_volume[extruder_id] -= volume_flushed_filament;
            } else if extruder_id < self.m_remaining_volume.len() {
                let rem = self.m_remaining_volume[extruder_id];
                if last_filament_id != -1 {
                    self.m_used_filaments
                        .update_flush_per_filament(last_filament_id as usize, rem);
                }
                self.m_used_filaments.update_flush_per_filament(
                    filament_id as usize,
                    volume_flushed_filament - rem,
                );
                self.m_remaining_volume[extruder_id] = 0.0;
            }
        }

        // time estimate section -------------------------------------------------
        // C++ move_length: float sq_xyz_length = sqr(dpX)+sqr(dpY)+sqr(dpZ); (double->float)
        //   return (sq_xyz_length > 0) ? std::sqrt(sq_xyz_length) : std::abs(dpE);
        let sq_xyz_length = (dx * dx + dy * dy + dz * dz) as f32;
        let distance = if sq_xyz_length > 0.0 {
            sq_xyz_length.sqrt()
        } else {
            de.abs() as f32
        };
        let inv_distance = 1.0 / distance;
        let is_extrusion_only_move = dx == 0.0 && dy == 0.0 && dz == 0.0 && de != 0.0;

        for i in 0..ETimeMode::COUNT {
            let mode = if i == 0 {
                ETimeMode::Normal
            } else {
                ETimeMode::Stealth
            };
            if !self.m_time_processor.machines[i].enabled {
                continue;
            }

            // curr.feedrate (cpp:4108-4110)
            let mut feedrate = if de == 0.0 {
                self.minimum_travel_feedrate(mode, self.m_feedrate)
            } else {
                self.minimum_feedrate(mode, self.m_feedrate)
            };

            // enter/exit direction (cpp:4112-4117)
            let mut enter_direction = [dx as f32, dy as f32, dz as f32];
            let norm =
                (enter_direction[0].powi(2) + enter_direction[1].powi(2) + enter_direction[2].powi(2)).sqrt();
            if !is_extrusion_only_move {
                enter_direction = [enter_direction[0] / norm, enter_direction[1] / norm, enter_direction[2] / norm];
            }
            let exit_direction = enter_direction;
            let prev_exit_direction = self.m_time_processor.machines[i].prev.exit_direction;
            let prev_feedrate = self.m_time_processor.machines[i].prev.feedrate;
            let prev_safe_feedrate = self.m_time_processor.machines[i].prev.safe_feedrate;
            let blocks_empty = self.m_time_processor.machines[i].blocks.is_empty();
            let extrude_factor =
                self.m_time_processor.machines[i].extrude_factor_override_percentage;

            let mut block = TimeBlock::default();
            block.move_type = r#type;
            block.skippable_type = self.m_skippable_type;
            block.role = if r#type != EMoveType::Travel
                || self.m_extrusion_role == ExtrusionRole::Custom
            {
                self.m_extrusion_role
            } else {
                ExtrusionRole::None
            };
            block.distance = distance;
            block.move_id = self.m_result.moves.len() as u32;
            block.g1_line_id = self.m_g1_line_id;
            block.layer_id = 1u32.max(self.m_layer_id);
            block.flags.prepare_stage = self.m_processing_start_custom_gcode;

            let float_model_g1 = crate::faithful_gate("TIME_FLOAT_MODEL");
            // block acceleration (cpp:4131-4135)
            let mut acceleration = if r#type == EMoveType::Travel {
                self.get_travel_acceleration(mode)
            } else if is_extrusion_only_move {
                self.get_retract_acceleration(mode)
            } else {
                self.get_acceleration(mode)
            };

            // centripetal acceleration limit (cpp:4137-4161)
            if (prev_exit_direction[0] != 0.0 || prev_exit_direction[1] != 0.0)
                && (enter_direction[0] != 0.0 || enter_direction[1] != 0.0)
                && !is_extrusion_only_move
            {
                let mut v1 = [prev_exit_direction[0], prev_exit_direction[1], 0.0f32];
                normalize3(&mut v1);
                let mut v2 = [enter_direction[0], enter_direction[1], 0.0f32];
                normalize3(&mut v2);
                let norm_diff = ((v2[0] - v1[0]).powi(2)
                    + (v2[1] - v1[1]).powi(2)
                    + (v2[2] - v1[2]).powi(2))
                .sqrt();
                if norm_diff < 0.5 && norm_diff > 0.00001 {
                    let float_model = crate::faithful_gate("TIME_FLOAT_MODEL");
                    // GCodeProcessor.cpp:4336-4337: float products summed in one
                    // expression, which clang on arm64 contracts into a fused
                    // multiply-add (the left product is the fused one).
                    let (dot, cross) = if float_model {
                        (
                            v1[0].mul_add(v2[0], v1[1] * v2[1]),
                            v1[0].mul_add(v2[1], -(v1[1] * v2[0])),
                        )
                    } else {
                        (v1[0] * v2[0] + v1[1] * v2[1], v1[0] * v2[1] - v1[1] * v2[0])
                    };
                    let angle = (cross as f64).atan2(dot as f64) as f32;
                    // C++: float sin_theta_2 = sqrt((1.0f - cos(angle)) * 0.5f);
                    // (GCodeProcessor.cpp:4339). `angle` is a float and libc++'s
                    // <math.h> declares `float cos(float)` / `float sqrt(float)` in
                    // the global namespace, so the whole expression is single
                    // precision: `1 - cosf(angle)` cancels for the small angles this
                    // branch takes (norm_diff < 0.5), which a double cosine does not
                    // reproduce.
                    let sin_theta_2 = if float_model {
                        ((1.0f32 - angle.cos()) * 0.5f32).sqrt()
                    } else {
                        ((1.0 - (angle as f64).cos()) * 0.5).sqrt() as f32
                    };
                    // C++: float r = sqrt(sqr(dpX)+sqr(dpY)) * 0.5 / sin_theta_2;
                    // sqr(double); 0.5 is double; division in double -> float.
                    let r = ((dx * dx + dy * dy).sqrt() * 0.5 / sin_theta_2 as f64) as f32;
                    feedrate = feedrate.min((acceleration * r).sqrt());
                }
            }

            // block cruise feedrate (cpp:4163-4186)
            // `curr.axis_feedrate` / `abs_axis_feedrate` are AxisCoords, arrays of
            // double (GCodeProcessor.hpp:548, :639-640), and `delta_pos` is one
            // too: `curr.feedrate * delta_pos[a] * inv_distance` is evaluated in
            // double, and `std::min<float>(factor, max / abs)` rounds a double
            // quotient. TIME_FLOAT_MODEL follows that; without it the port keeps
            // its single-precision arithmetic.
            let mut axis_feedrate = [0.0f64; 4];
            let mut abs_axis_feedrate = [0.0f64; 4];
            let mut min_feedrate_factor = 1.0f32;
            for a in 0..=(Axis::E as usize) {
                axis_feedrate[a] = if float_model_g1 {
                    feedrate as f64 * delta_pos[a] * inv_distance as f64
                } else {
                    (feedrate * delta_pos[a] as f32 * inv_distance) as f64
                };
                if a == Axis::E as usize {
                    axis_feedrate[a] = if float_model_g1 {
                        axis_feedrate[a] * extrude_factor as f64
                    } else {
                        (axis_feedrate[a] as f32 * extrude_factor) as f64
                    };
                }
                abs_axis_feedrate[a] = axis_feedrate[a].abs();
                if abs_axis_feedrate[a] != 0.0 {
                    let axis_max_feedrate = self.get_axis_max_feedrate(
                        mode,
                        a,
                        self.get_machine_config_idx(self.get_filament_id(true)),
                    );
                    if axis_max_feedrate != 0.0 {
                        min_feedrate_factor = if float_model_g1 {
                            min_feedrate_factor
                                .min((axis_max_feedrate as f64 / abs_axis_feedrate[a]) as f32)
                        } else {
                            min_feedrate_factor
                                .min(axis_max_feedrate / abs_axis_feedrate[a] as f32)
                        };
                    }
                }
            }
            feedrate *= min_feedrate_factor;
            block.feedrate_profile.cruise = feedrate;
            if min_feedrate_factor < 1.0 {
                for a in 0..=(Axis::E as usize) {
                    if float_model_g1 {
                        axis_feedrate[a] *= min_feedrate_factor as f64;
                        abs_axis_feedrate[a] *= min_feedrate_factor as f64;
                    } else {
                        axis_feedrate[a] = (axis_feedrate[a] as f32 * min_feedrate_factor) as f64;
                        abs_axis_feedrate[a] =
                            (abs_axis_feedrate[a] as f32 * min_feedrate_factor) as f64;
                    }
                }
            }

            // axis-limited acceleration (cpp:4189-4194)
            // C++ has NO `axis_max_acceleration != 0.0` guard: when the limit is
            // 0 and the candidate is > 0, acceleration is clamped to 0/(x) == 0.
            for a in 0..=(Axis::E as usize) {
                let axis_max_acceleration = self.get_axis_max_acceleration(
                    mode,
                    a,
                    self.get_machine_config_idx(self.get_filament_id(true)),
                );
                if float_model_g1 {
                    // cpp:4374-4375: float * double * float, compared and divided
                    // in double.
                    if acceleration as f64 * delta_pos[a].abs() * inv_distance as f64
                        > axis_max_acceleration as f64
                    {
                        acceleration = (axis_max_acceleration as f64
                            / (delta_pos[a].abs() * inv_distance as f64))
                            as f32;
                    }
                } else if acceleration * (delta_pos[a] as f32).abs() * inv_distance
                    > axis_max_acceleration
                {
                    acceleration = axis_max_acceleration / ((delta_pos[a] as f32).abs() * inv_distance);
                }
            }
            block.acceleration = acceleration;

            // block exit feedrate (cpp:4198-4207)
            // C++ has NO `axis_max_jerk != 0.0` guard: when the jerk limit is 0 and
            // abs_axis_feedrate > 0, safe_feedrate is clamped to min(.,0) == 0.
            let mut safe_feedrate = block.feedrate_profile.cruise;
            for a in 0..=(Axis::E as usize) {
                let axis_max_jerk = self.get_axis_max_jerk(mode, a);
                if abs_axis_feedrate[a] > axis_max_jerk as f64 {
                    safe_feedrate = safe_feedrate.min(axis_max_jerk);
                }
            }
            block.feedrate_profile.exit = safe_feedrate;

            const PREVIOUS_FEEDRATE_THRESHOLD: f32 = 0.0001;
            // block entry feedrate (cpp:4211-4252)
            let mut vmax_junction = safe_feedrate;
            if !blocks_empty && prev_feedrate > PREVIOUS_FEEDRATE_THRESHOLD {
                vmax_junction = prev_feedrate.min(block.feedrate_profile.cruise);

                let mut limited = false;
                let exit_direction_unit = normalized3(prev_exit_direction);
                let enter_direction_unit = normalized3(enter_direction);
                let mut k_min = 10000.0f32;

                // a == X branch (Y/Z continue) — cpp:4221-4240
                let jerk_v = [
                    (enter_direction_unit[0] - exit_direction_unit[0]).abs(),
                    (enter_direction_unit[1] - exit_direction_unit[1]).abs(),
                    (enter_direction_unit[2] - exit_direction_unit[2]).abs(),
                ];
                let max_xyz_jerk_v = self.get_xyz_max_jerk(mode);
                for idx in 0..3 {
                    if jerk_v[idx] > 0.0 {
                        limited = true;
                        let k = max_xyz_jerk_v[idx] / jerk_v[idx];
                        if k < k_min {
                            k_min = k;
                        }
                    }
                }
                if limited {
                    vmax_junction = k_min;
                }

                let vmax_junction_threshold = vmax_junction * 0.99;
                if prev_safe_feedrate > vmax_junction_threshold
                    && safe_feedrate > vmax_junction_threshold
                {
                    vmax_junction = safe_feedrate;
                }
            }

            let v_allowable = max_allowable_speed(-acceleration, safe_feedrate, block.distance);
            block.feedrate_profile.entry = vmax_junction.min(v_allowable);
            block.max_entry_speed = vmax_junction;
            block.flags.nominal_length = block.feedrate_profile.cruise <= v_allowable;
            block.flags.recalculate = true;
            block.safe_feedrate = safe_feedrate;

            block.calculate_trapezoid();

            // updates previous + push block
            {
                let machine = &mut self.m_time_processor.machines[i];
                machine.curr.feedrate = feedrate;
                machine.curr.safe_feedrate = safe_feedrate;
                machine.curr.axis_feedrate = axis_feedrate;
                machine.curr.abs_axis_feedrate = abs_axis_feedrate;
                machine.curr.enter_direction = enter_direction;
                machine.curr.exit_direction = exit_direction;
                machine.prev = machine.curr.clone();
                machine.blocks.push(block);
            }

            if self.m_time_processor.machines[i].blocks.len()
                > TimeProcessor::PLANNER_REFRESH_THRESHOLD
            {
                self.run_calculate_time(
                    i,
                    TimeProcessor::PLANNER_QUEUE_SIZE,
                    0.0,
                    ExtrusionRole::None,
                );
            }
        }

        // GCodeProcessor.cpp:4459-4515 — the seams detector and the spiral vase layers.
        // PROCESSOR_SEAMS (on): what they change is the moves of the result (the seam moves, the
        // layer ranges), not the times. =0 leaves both out.
        if crate::faithful_gate("PROCESSOR_SEAMS") {
            self.process_g1_seams_and_spiral_layers(r#type, filament_id, &delta_pos);
        }

        // store move
        self.store_move_vertex(r#type, EMovePathType::NoopMove);
    }

    /// GCodeProcessor.cpp:4521-4903 `void GCodeProcessor::process_VG1(const GCodeReader::GCodeLine&
    /// line)`: the virtual move of a `;VG1` comment. It is `process_G1` without the thermal index of
    /// the comment, the seams and the spiral vase layers, with the virtual flush (`m_virtual_flushing`)
    /// in place of the real one, the centripetal limit taken with the extrusion acceleration and on
    /// extrusion-only moves too, and the junction speed of the jerk model per axis (X, Y, Z together,
    /// then E) of the earlier planner.
    fn process_vg1(&mut self, line: &crate::g_code_reader::GCodeLine) {
        use crate::g_code_reader::Axis;
        let filament_id = self.get_filament_id(true);
        let _last_filament_id = self.get_last_filament_id(false);
        let filament_diameter = if (filament_id as usize) < self.m_result.filament_diameters.len() {
            self.m_result.filament_diameters[filament_id as usize]
        } else {
            *self.m_result.filament_diameters.last().unwrap()
        };
        let filament_radius = 0.5 * filament_diameter;
        let area_filament_cross_section = (PI as f32) * sqr(filament_radius);

        // absolute_position lambda (cpp:3945-3957)
        let global_relative = self.m_global_positioning_type == EPositioningType::Relative;
        let e_relative = self.m_e_local_positioning_type == EPositioningType::Relative;
        let units_inches = self.m_units == EUnits::Inches;
        let start_position = self.m_start_position;
        let origin = self.m_origin;
        let absolute_position = |axis: usize, lineg1: &crate::g_code_reader::GCodeLine| -> f64 {
            let mut is_relative = global_relative;
            if axis == Axis::E as usize {
                is_relative |= e_relative;
            }
            let ax = match axis {
                0 => Axis::X,
                1 => Axis::Y,
                2 => Axis::Z,
                _ => Axis::E,
            };
            if lineg1.has(ax) {
                let length_scale = if units_inches { INCHES_TO_MM } else { 1.0 };
                let ret = (lineg1.value(ax) * length_scale) as f64;
                if is_relative {
                    start_position[axis] + ret
                } else {
                    origin[axis] + ret
                }
            } else {
                start_position[axis]
            }
        };

        self.m_g1_line_id += 1;
        // enable processing of lines M201/M203/M204/M205
        self.m_time_processor.machine_envelope_processing_enabled = true;

        // updates axes positions from line (X..=E)
        for a in 0..=(Axis::E as usize) {
            self.m_end_position[a] = absolute_position(a, line);
        }

        // updates feedrate from line, if present
        if line.has_f() {
            self.m_feedrate = line.f() * MMMIN_TO_MMSEC;
        }

        // calculates movement deltas
        let mut max_abs_delta: f32 = 0.0;
        let mut delta_pos: AxisCoords = [0.0; 4];
        for a in 0..=(Axis::E as usize) {
            delta_pos[a] = self.m_end_position[a] - self.m_start_position[a];
            max_abs_delta = max_abs_delta.max((delta_pos[a] as f32).abs());
        }

        // no displacement, return
        if max_abs_delta == 0.0 {
            return;
        }

        let dx = delta_pos[Axis::X as usize];
        let dy = delta_pos[Axis::Y as usize];
        let dz = delta_pos[Axis::Z as usize];
        let de = delta_pos[Axis::E as usize];

        // move_type lambda (cpp:3959-3976)
        let move_type = |dp: &AxisCoords| -> EMoveType {
            let (px, py, pz, pe) = (
                dp[Axis::X as usize],
                dp[Axis::Y as usize],
                dp[Axis::Z as usize],
                dp[Axis::E as usize],
            );
            if self.m_wiping {
                EMoveType::Wipe
            } else if pe < 0.0 {
                if px != 0.0 || py != 0.0 || pz != 0.0 {
                    EMoveType::Travel
                } else {
                    EMoveType::Retract
                }
            } else if pe > 0.0 {
                if px == 0.0 && py == 0.0 {
                    if pz == 0.0 {
                        EMoveType::Unretract
                    } else {
                        EMoveType::Travel
                    }
                } else if px != 0.0 || py != 0.0 {
                    EMoveType::Extrude
                } else {
                    EMoveType::Noop
                }
            } else if px != 0.0 || py != 0.0 || pz != 0.0 {
                EMoveType::Travel
            } else {
                EMoveType::Noop
            }
        };

        let r#type = move_type(&delta_pos);
        if r#type == EMoveType::Extrude {
            // C++: float delta_xyz = std::sqrt(sqr(dpX)+sqr(dpY)+sqr(dpZ));
            // delta_pos is std::array<double,4>, so sqr/sum/sqrt are double.
            let delta_xyz = (dx * dx + dy * dy + dz * dz).sqrt() as f32;
            // C++: float vol = area_filament_cross_section * dpE; (float*double->double)
            let volume_extruded_filament = (area_filament_cross_section as f64 * de) as f32;
            let area_toolpath_cross_section = volume_extruded_filament / delta_xyz;

            if self.m_extrusion_role == ExtrusionRole::SupportMaterial
                || self.m_extrusion_role == ExtrusionRole::SupportMaterialInterface
                || self.m_extrusion_role == ExtrusionRole::SupportTransition
            {
                self.m_used_filaments
                    .increase_support_caches(volume_extruded_filament as f64);
            } else if self.m_extrusion_role == ExtrusionRole::WipeTower {
                self.m_used_filaments
                    .increase_wipe_tower_caches(volume_extruded_filament as f64);
            } else {
                self.m_used_filaments
                    .increase_model_caches(volume_extruded_filament as f64);
            }
            self.m_mm3_per_mm = area_toolpath_cross_section;

            if self.m_forced_height > 0.0 {
                self.m_height = self.m_forced_height;
            } else if self.m_end_position[Axis::Z as usize]
                > (self.m_extruded_last_z + EPSILON) as f64
            {
                self.m_height =
                    (self.m_end_position[Axis::Z as usize] - self.m_extruded_last_z as f64) as f32;
            }
            if self.m_height == 0.0 {
                self.m_height = DEFAULT_TOOLPATH_HEIGHT;
            }
            if self.m_end_position[Axis::Z as usize] == 0.0 {
                self.m_end_position[Axis::Z as usize] = self.m_height as f64;
            }
            self.m_extruded_last_z = self.m_end_position[Axis::Z as usize] as f32;
            // m_options_z_corrector.update(m_height); — BLOCKED(deps): no-op.

            if self.m_forced_width > 0.0 {
                self.m_width = self.m_forced_width;
            } else if self.m_extrusion_role == ExtrusionRole::ExternalPerimeter {
                // C++: m_width = delta_pos[E] * static_cast<float>(M_PI*sqr(1.05f*fr))
                //               / (delta_xyz * m_height);
                // delta_pos[E] is double; the cast constant is float; division promotes
                // to double; the result is cast to float on assignment.
                let c1 = (PI * sqr(1.05 * filament_radius) as f64) as f32;
                self.m_width =
                    (de * c1 as f64 / (delta_xyz * self.m_height) as f64) as f32;
            } else if self.m_extrusion_role == ExtrusionRole::BridgeInfill
                || self.m_extrusion_role == ExtrusionRole::None
            {
                // C++: m_width = (float)filament_diameters[id] * std::sqrt(dpE/delta_xyz);
                // dpE/delta_xyz is double; sqrt is double; float*double -> double -> float.
                let fd = self.m_result.filament_diameters[filament_id as usize];
                self.m_width = (fd as f64 * (de / delta_xyz as f64).sqrt()) as f32;
            } else {
                // C++: m_width = delta_pos[E] * static_cast<float>(M_PI*sqr(fr))
                //               / (delta_xyz * m_height)
                //               + static_cast<float>(1.0 - 0.25*M_PI) * m_height;
                let c2 = (PI * sqr(filament_radius) as f64) as f32;
                let c3 = (1.0 - 0.25 * PI) as f32;
                self.m_width = (de * c2 as f64 / (delta_xyz * self.m_height) as f64
                    + (c3 * self.m_height) as f64) as f32;
            }
            if self.m_width == 0.0 {
                self.m_width = DEFAULT_TOOLPATH_WIDTH;
            }
            self.m_width = self.m_width.min(2.0_f32.max(4.0 * self.m_height));
        } else if r#type == EMoveType::Unretract && self.m_virtual_flushing {
            // BLOCKED(deps): flushing path needs get_extruder_id + remaining_volume
            // bookkeeping. Ported faithfully below using current state.
            let extruder_id = self.get_extruder_id(true).max(0) as usize;
            let volume_flushed_filament = area_filament_cross_section * de as f32;
            let last_filament_id = self.get_last_filament_id(false);
            if extruder_id < self.m_remaining_volume.len()
                && self.m_remaining_volume[extruder_id] > volume_flushed_filament
            {
                if last_filament_id != -1 {
                    self.m_used_filaments
                        .update_flush_per_filament(last_filament_id as usize, volume_flushed_filament);
                }
                self.m_remaining_volume[extruder_id] -= volume_flushed_filament;
            } else if extruder_id < self.m_remaining_volume.len() {
                let rem = self.m_remaining_volume[extruder_id];
                if last_filament_id != -1 {
                    self.m_used_filaments
                        .update_flush_per_filament(last_filament_id as usize, rem);
                }
                self.m_used_filaments.update_flush_per_filament(
                    filament_id as usize,
                    volume_flushed_filament - rem,
                );
                self.m_remaining_volume[extruder_id] = 0.0;
            }
        }

        if line.has_f() {
            self.m_feedrate = line.f() * MMMIN_TO_MMSEC;
        }

        // time estimate section -------------------------------------------------
        // C++ move_length: float sq_xyz_length = sqr(dpX)+sqr(dpY)+sqr(dpZ); (double->float)
        //   return (sq_xyz_length > 0) ? std::sqrt(sq_xyz_length) : std::abs(dpE);
        let sq_xyz_length = (dx * dx + dy * dy + dz * dz) as f32;
        let distance = if sq_xyz_length > 0.0 {
            sq_xyz_length.sqrt()
        } else {
            de.abs() as f32
        };
        let inv_distance = 1.0 / distance;
        let is_extrusion_only_move = dx == 0.0 && dy == 0.0 && dz == 0.0 && de != 0.0;

        for i in 0..ETimeMode::COUNT {
            let mode = if i == 0 {
                ETimeMode::Normal
            } else {
                ETimeMode::Stealth
            };
            if !self.m_time_processor.machines[i].enabled {
                continue;
            }

            // curr.feedrate (cpp:4108-4110)
            let mut feedrate = if de == 0.0 {
                self.minimum_travel_feedrate(mode, self.m_feedrate)
            } else {
                self.minimum_feedrate(mode, self.m_feedrate)
            };

            // enter/exit direction (cpp:4112-4117)
            let mut enter_direction = [dx as f32, dy as f32, dz as f32];
            let norm =
                (enter_direction[0].powi(2) + enter_direction[1].powi(2) + enter_direction[2].powi(2)).sqrt();
            if !is_extrusion_only_move {
                enter_direction = [enter_direction[0] / norm, enter_direction[1] / norm, enter_direction[2] / norm];
            }
            let exit_direction = enter_direction;
            let prev_exit_direction = self.m_time_processor.machines[i].prev.exit_direction;
            let prev_feedrate = self.m_time_processor.machines[i].prev.feedrate;
            let prev_safe_feedrate = self.m_time_processor.machines[i].prev.safe_feedrate;
            let blocks_empty = self.m_time_processor.machines[i].blocks.is_empty();
            let extrude_factor =
                self.m_time_processor.machines[i].extrude_factor_override_percentage;

            let mut block = TimeBlock::default();
            block.move_type = r#type;
            block.skippable_type = self.m_skippable_type;
            block.role = if r#type != EMoveType::Travel
                || self.m_extrusion_role == ExtrusionRole::Custom
            {
                self.m_extrusion_role
            } else {
                ExtrusionRole::None
            };
            block.distance = distance;
            block.move_id = self.m_result.moves.len() as u32;
            block.g1_line_id = self.m_g1_line_id;
            block.layer_id = 1u32.max(self.m_layer_id);
            block.flags.prepare_stage = self.m_processing_start_custom_gcode;

            let float_model_g1 = crate::faithful_gate("TIME_FLOAT_MODEL");
            // block acceleration (cpp:4131-4135)
            let mut acceleration = if r#type == EMoveType::Travel {
                self.get_travel_acceleration(mode)
            } else if is_extrusion_only_move {
                self.get_retract_acceleration(mode)
            } else {
                self.get_acceleration(mode)
            };

            // centripetal acceleration limit (cpp:4137-4161)
            if (prev_exit_direction[0] != 0.0 || prev_exit_direction[1] != 0.0)
                && (enter_direction[0] != 0.0 || enter_direction[1] != 0.0)
            {
                let mut v1 = [prev_exit_direction[0], prev_exit_direction[1], 0.0f32];
                normalize3(&mut v1);
                let mut v2 = [enter_direction[0], enter_direction[1], 0.0f32];
                normalize3(&mut v2);
                let norm_diff = ((v2[0] - v1[0]).powi(2)
                    + (v2[1] - v1[1]).powi(2)
                    + (v2[2] - v1[2]).powi(2))
                .sqrt();
                if norm_diff < 0.5 && norm_diff > 0.00001 {
                    let float_model = crate::faithful_gate("TIME_FLOAT_MODEL");
                    // GCodeProcessor.cpp:4336-4337: float products summed in one
                    // expression, which clang on arm64 contracts into a fused
                    // multiply-add (the left product is the fused one).
                    let (dot, cross) = if float_model {
                        (
                            v1[0].mul_add(v2[0], v1[1] * v2[1]),
                            v1[0].mul_add(v2[1], -(v1[1] * v2[0])),
                        )
                    } else {
                        (v1[0] * v2[0] + v1[1] * v2[1], v1[0] * v2[1] - v1[1] * v2[0])
                    };
                    let angle = (cross as f64).atan2(dot as f64) as f32;
                    // C++: float sin_theta_2 = sqrt((1.0f - cos(angle)) * 0.5f);
                    // (GCodeProcessor.cpp:4339). `angle` is a float and libc++'s
                    // <math.h> declares `float cos(float)` / `float sqrt(float)` in
                    // the global namespace, so the whole expression is single
                    // precision: `1 - cosf(angle)` cancels for the small angles this
                    // branch takes (norm_diff < 0.5), which a double cosine does not
                    // reproduce.
                    let sin_theta_2 = if float_model {
                        ((1.0f32 - angle.cos()) * 0.5f32).sqrt()
                    } else {
                        ((1.0 - (angle as f64).cos()) * 0.5).sqrt() as f32
                    };
                    // C++: float r = sqrt(sqr(dpX)+sqr(dpY)) * 0.5 / sin_theta_2;
                    // sqr(double); 0.5 is double; division in double -> float.
                    let r = ((dx * dx + dy * dy).sqrt() * 0.5 / sin_theta_2 as f64) as f32;
                    let acc = self.get_acceleration(mode);
                    feedrate = feedrate.min((acc * r).sqrt());
                }
            }

            // block cruise feedrate (cpp:4163-4186)
            // `curr.axis_feedrate` / `abs_axis_feedrate` are AxisCoords, arrays of
            // double (GCodeProcessor.hpp:548, :639-640), and `delta_pos` is one
            // too: `curr.feedrate * delta_pos[a] * inv_distance` is evaluated in
            // double, and `std::min<float>(factor, max / abs)` rounds a double
            // quotient. TIME_FLOAT_MODEL follows that; without it the port keeps
            // its single-precision arithmetic.
            let mut axis_feedrate = [0.0f64; 4];
            let mut abs_axis_feedrate = [0.0f64; 4];
            let mut min_feedrate_factor = 1.0f32;
            for a in 0..=(Axis::E as usize) {
                axis_feedrate[a] = if float_model_g1 {
                    feedrate as f64 * delta_pos[a] * inv_distance as f64
                } else {
                    (feedrate * delta_pos[a] as f32 * inv_distance) as f64
                };
                if a == Axis::E as usize {
                    axis_feedrate[a] = if float_model_g1 {
                        axis_feedrate[a] * extrude_factor as f64
                    } else {
                        (axis_feedrate[a] as f32 * extrude_factor) as f64
                    };
                }
                abs_axis_feedrate[a] = axis_feedrate[a].abs();
                if abs_axis_feedrate[a] != 0.0 {
                    let axis_max_feedrate = self.get_axis_max_feedrate(
                        mode,
                        a,
                        self.get_machine_config_idx(self.get_filament_id(true)),
                    );
                    if axis_max_feedrate != 0.0 {
                        min_feedrate_factor = if float_model_g1 {
                            min_feedrate_factor
                                .min((axis_max_feedrate as f64 / abs_axis_feedrate[a]) as f32)
                        } else {
                            min_feedrate_factor
                                .min(axis_max_feedrate / abs_axis_feedrate[a] as f32)
                        };
                    }
                }
            }
            feedrate *= min_feedrate_factor;
            block.feedrate_profile.cruise = feedrate;
            if min_feedrate_factor < 1.0 {
                for a in 0..=(Axis::E as usize) {
                    if float_model_g1 {
                        axis_feedrate[a] *= min_feedrate_factor as f64;
                        abs_axis_feedrate[a] *= min_feedrate_factor as f64;
                    } else {
                        axis_feedrate[a] = (axis_feedrate[a] as f32 * min_feedrate_factor) as f64;
                        abs_axis_feedrate[a] =
                            (abs_axis_feedrate[a] as f32 * min_feedrate_factor) as f64;
                    }
                }
            }

            // axis-limited acceleration (cpp:4189-4194)
            // C++ has NO `axis_max_acceleration != 0.0` guard: when the limit is
            // 0 and the candidate is > 0, acceleration is clamped to 0/(x) == 0.
            for a in 0..=(Axis::E as usize) {
                let axis_max_acceleration = self.get_axis_max_acceleration(
                    mode,
                    a,
                    self.get_machine_config_idx(self.get_filament_id(true)),
                );
                if float_model_g1 {
                    // cpp:4374-4375: float * double * float, compared and divided
                    // in double.
                    if acceleration as f64 * delta_pos[a].abs() * inv_distance as f64
                        > axis_max_acceleration as f64
                    {
                        acceleration = (axis_max_acceleration as f64
                            / (delta_pos[a].abs() * inv_distance as f64))
                            as f32;
                    }
                } else if acceleration * (delta_pos[a] as f32).abs() * inv_distance
                    > axis_max_acceleration
                {
                    acceleration = axis_max_acceleration / ((delta_pos[a] as f32).abs() * inv_distance);
                }
            }
            block.acceleration = acceleration;

            // block exit feedrate (cpp:4198-4207)
            // C++ has NO `axis_max_jerk != 0.0` guard: when the jerk limit is 0 and
            // abs_axis_feedrate > 0, safe_feedrate is clamped to min(.,0) == 0.
            let mut safe_feedrate = block.feedrate_profile.cruise;
            for a in 0..=(Axis::E as usize) {
                let axis_max_jerk = self.get_axis_max_jerk(mode, a);
                if abs_axis_feedrate[a] > axis_max_jerk as f64 {
                    safe_feedrate = safe_feedrate.min(axis_max_jerk);
                }
            }
            block.feedrate_profile.exit = safe_feedrate;

            const PREVIOUS_FEEDRATE_THRESHOLD: f32 = 0.0001;
            // block entry feedrate (cpp:4211-4252)
            let mut vmax_junction = safe_feedrate;
            if !blocks_empty && prev_feedrate > PREVIOUS_FEEDRATE_THRESHOLD {
                let cruise = block.feedrate_profile.cruise;
                let prev_speed_larger = prev_feedrate > cruise;
                let smaller_speed_factor = if prev_speed_larger { cruise / prev_feedrate } else { prev_feedrate / cruise };
                // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
                vmax_junction = if prev_speed_larger { cruise } else { prev_feedrate };

                let mut v_factor = 1.0f32;
                let mut limited = false;

                let prev_axis_feedrate = self.m_time_processor.machines[i].prev.axis_feedrate;
                for a in 0..=(Axis::E as usize) {
                    // Limit an axis. We have to differentiate coasting from the reversal of an axis movement, or a full stop.
                    if a == Axis::X as usize {
                        let mut exit_v = [
                            prev_feedrate * prev_exit_direction[0],
                            prev_feedrate * prev_exit_direction[1],
                            prev_feedrate * prev_exit_direction[2],
                        ];
                        if prev_speed_larger {
                            for c in exit_v.iter_mut() {
                                *c *= smaller_speed_factor;
                            }
                        }
                        let entry_v = [cruise * enter_direction[0], cruise * enter_direction[1], cruise * enter_direction[2]];
                        let mut jerk_v = [
                            (entry_v[0] - exit_v[0]).abs(),
                            (entry_v[1] - exit_v[1]).abs(),
                            (entry_v[2] - exit_v[2]).abs(),
                        ];
                        let max_xyz_jerk_v = self.get_xyz_max_jerk(mode);

                        for idx in 0..3 {
                            if jerk_v[idx] > max_xyz_jerk_v[idx] {
                                v_factor *= max_xyz_jerk_v[idx] / jerk_v[idx];
                                for c in jerk_v.iter_mut() {
                                    *c *= v_factor;
                                }
                                limited = true;
                            }
                        }
                    } else if a == Axis::Y as usize || a == Axis::Z as usize {
                        continue;
                    } else {
                        let mut v_exit = prev_axis_feedrate[a] as f32;
                        let mut v_entry = axis_feedrate[a] as f32;

                        if prev_speed_larger {
                            v_exit *= smaller_speed_factor;
                        }

                        if limited {
                            v_exit *= v_factor;
                            v_entry *= v_factor;
                        }

                        // Calculate the jerk depending on whether the axis is coasting in the same direction or reversing a direction.
                        let jerk = if v_exit > v_entry {
                            if v_entry > 0.0 || v_exit < 0.0 {
                                // coasting
                                v_exit - v_entry
                            } else {
                                // axis reversal
                                v_exit.max(-v_entry)
                            }
                        } else if v_entry < 0.0 || v_exit > 0.0 {
                            // v_exit <= v_entry, coasting
                            v_entry - v_exit
                        } else {
                            // axis reversal
                            (-v_exit).max(v_entry)
                        };

                        let axis_max_jerk = self.get_axis_max_jerk(mode, a);
                        if jerk > axis_max_jerk {
                            v_factor *= axis_max_jerk / jerk;
                            limited = true;
                        }
                    }
                }

                if limited {
                    vmax_junction *= v_factor;
                }

                // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
                // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
                let vmax_junction_threshold = vmax_junction * 0.99f32;

                // Not coasting. The machine will stop and start the movements anyway, better to start the segment from start.
                if prev_safe_feedrate > vmax_junction_threshold && safe_feedrate > vmax_junction_threshold {
                    vmax_junction = safe_feedrate;
                }
            }

            let v_allowable = max_allowable_speed(-acceleration, safe_feedrate, block.distance);
            block.feedrate_profile.entry = vmax_junction.min(v_allowable);
            block.max_entry_speed = vmax_junction;
            block.flags.nominal_length = block.feedrate_profile.cruise <= v_allowable;
            block.flags.recalculate = true;
            block.safe_feedrate = safe_feedrate;

            block.calculate_trapezoid();

            // updates previous + push block
            {
                let machine = &mut self.m_time_processor.machines[i];
                machine.curr.feedrate = feedrate;
                machine.curr.safe_feedrate = safe_feedrate;
                machine.curr.axis_feedrate = axis_feedrate;
                machine.curr.abs_axis_feedrate = abs_axis_feedrate;
                machine.curr.enter_direction = enter_direction;
                machine.curr.exit_direction = exit_direction;
                machine.prev = machine.curr.clone();
                machine.blocks.push(block);
            }

            if self.m_time_processor.machines[i].blocks.len()
                > TimeProcessor::PLANNER_REFRESH_THRESHOLD
            {
                self.run_calculate_time(
                    i,
                    TimeProcessor::PLANNER_QUEUE_SIZE,
                    0.0,
                    ExtrusionRole::None,
                );
            }
        }

        // store move
        self.store_move_vertex(r#type, EMovePathType::NoopMove);
    }

    /// GCodeProcessor.cpp:4459-4515, the tail of `process_G1` before the move is stored: the seam of
    /// an outer wall loop (a `Seam` move where the loop closes) and the layer ranges of a spiral vase.
    fn process_g1_seams_and_spiral_layers(&mut self, r#type: EMoveType, filament_id: i32, delta_pos: &AxisCoords) {
        use crate::g_code_reader::Axis;
        let off = self.m_extruder_offsets[filament_id as usize];
        let back_position = |this: &Self| -> [f32; 3] { this.m_result.moves.last().map(|m| m.position).unwrap_or([0.0; 3]) };
        if self.m_seams_detector.is_active() {
            // check for seam starting vertex
            if r#type == EMoveType::Extrude
                && self.m_extrusion_role == ExtrusionRole::ExternalPerimeter
                && !self.m_seams_detector.has_first_vertex()
            {
                //BBS: m_result.moves.back().position has plate offset, must minus plate offset before calculate the real seam position
                let b = back_position(self);
                let real_first_pos =
                    [(b[0] as f64 - self.m_x_offset) as f32, (b[1] as f64 - self.m_y_offset) as f32, b[2]];
                self.m_seams_detector
                    .set_first_vertex([real_first_pos[0] - off[0], real_first_pos[1] - off[1], real_first_pos[2] - off[2]]);
            } else if r#type == EMoveType::Extrude
                && self.m_extrusion_role == ExtrusionRole::ExternalPerimeter
                && self.m_detect_layer_based_on_tag
            {
                let b = back_position(self);
                let real_last_pos =
                    [(b[0] as f64 - self.m_x_offset) as f32, (b[1] as f64 - self.m_y_offset) as f32, b[2]];
                let new_pos = [real_last_pos[0] - off[0], real_last_pos[1] - off[1], real_last_pos[2] - off[2]];
                // We may have sloped loop, drop any previous start pos if we have z increment
                let first_vertex = self.m_seams_detector.get_first_vertex();
                if new_pos[2] > first_vertex.unwrap()[2] {
                    self.m_seams_detector.set_first_vertex(new_pos);
                }
            }
            // check for seam ending vertex and store the resulting move
            else if (r#type != EMoveType::Extrude
                || (self.m_extrusion_role != ExtrusionRole::ExternalPerimeter
                    && self.m_extrusion_role != ExtrusionRole::OverhangPerimeter))
                && self.m_seams_detector.has_first_vertex()
            {
                let curr_pos = [
                    self.m_end_position[Axis::X as usize] as f32,
                    self.m_end_position[Axis::Y as usize] as f32,
                    self.m_end_position[Axis::Z as usize] as f32,
                ];
                //BBS: m_result.moves.back().position has plate offset, must minus plate offset before calculate the real seam position
                let b = back_position(self);
                let real_last_pos =
                    [(b[0] as f64 - self.m_x_offset) as f32, (b[1] as f64 - self.m_y_offset) as f32, b[2]];
                let new_pos = [real_last_pos[0] - off[0], real_last_pos[1] - off[1], real_last_pos[2] - off[2]];
                let first_vertex = self.m_seams_detector.get_first_vertex().unwrap();
                // the threshold value = 0.0625f == 0.25 * 0.25 is arbitrary, we may find some smarter condition later
                let d = [new_pos[0] - first_vertex[0], new_pos[1] - first_vertex[1], new_pos[2] - first_vertex[2]];
                if d[0] * d[0] + d[1] * d[1] + d[2] * d[2] < 0.0625f32 {
                    let mid = [
                        0.5f32 * (new_pos[0] + first_vertex[0]),
                        0.5f32 * (new_pos[1] + first_vertex[1]),
                        0.5f32 * (new_pos[2] + first_vertex[2]),
                    ];
                    self.m_end_position[Axis::X as usize] = mid[0] as f64;
                    self.m_end_position[Axis::Y as usize] = mid[1] as f64;
                    self.m_end_position[Axis::Z as usize] = mid[2] as f64;
                    self.store_move_vertex(EMoveType::Seam, EMovePathType::NoopMove);
                    self.m_end_position[Axis::X as usize] = curr_pos[0] as f64;
                    self.m_end_position[Axis::Y as usize] = curr_pos[1] as f64;
                    self.m_end_position[Axis::Z as usize] = curr_pos[2] as f64;
                }

                self.m_seams_detector.activate(false);
            }
        } else if r#type == EMoveType::Extrude && self.m_extrusion_role == ExtrusionRole::ExternalPerimeter {
            self.m_seams_detector.activate(true);
            let plate_offset = [self.m_x_offset as f32, self.m_y_offset as f32, 0.0f32];
            let b = back_position(self);
            self.m_seams_detector.set_first_vertex([
                b[0] - off[0] - plate_offset[0],
                b[1] - off[1] - plate_offset[1],
                b[2] - off[2] - plate_offset[2],
            ]);
        }

        if self.m_detect_layer_based_on_tag && !self.m_result.spiral_vase_layers.is_empty() {
            if delta_pos[Axis::Z as usize] >= 0.0 && r#type == EMoveType::Extrude {
                let current_z = self.m_end_position[Axis::Z as usize] as f32;
                // replace layer height placeholder with correct value
                let last = self.m_result.spiral_vase_layers.last_mut().unwrap();
                if last.0 == f32::MAX {
                    last.0 = current_z;
                } else {
                    last.0 = last.0.max(current_z);
                }
            }
            if !self.m_result.moves.is_empty() {
                let v = self.m_result.moves.len() - 1 - self.m_seams_count as usize;
                (self.m_result.spiral_vase_layers.last_mut().unwrap().1).1 = v;
            }
        }
    }

    /// GCodeProcessor.cpp:5935-5995  void GCodeProcessor::store_move_vertex(type, path_type)
    fn store_move_vertex(&mut self, r#type: EMoveType, path_type: EMovePathType) {
        use crate::g_code_reader::Axis;
        let filament_id = self.get_filament_id(true);
        self.m_last_line_id = if r#type == EMoveType::ColorChange
            || r#type == EMoveType::PausePrint
            || r#type == EMoveType::CustomGCode
        {
            self.m_line_id + 1
        } else if r#type == EMoveType::Seam {
            self.m_last_line_id
        } else {
            self.m_line_id
        };

        // BBS: apply plate's and extruder's offset to arc interpolation points
        if path_type == EMovePathType::ArcMoveCw || path_type == EMovePathType::ArcMoveCcw {
            let off = self.m_extruder_offsets[filament_id as usize];
            for ip in self.m_interpolation_points.iter_mut() {
                ip[0] = ip[0] + self.m_x_offset as f32 + off[0];
                ip[1] = ip[1] + self.m_y_offset as f32 + off[1];
                ip[2] = (if self.m_processing_start_custom_gcode {
                    self.m_first_layer_height
                } else {
                    ip[2]
                }) + off[2];
            }
        }

        let off = self.m_extruder_offsets[filament_id as usize];
        let position = [
            self.m_end_position[Axis::X as usize] as f32 + self.m_x_offset as f32 + off[0],
            self.m_end_position[Axis::Y as usize] as f32 + self.m_y_offset as f32 + off[1],
            (if self.m_processing_start_custom_gcode {
                self.m_first_layer_height
            } else {
                self.m_end_position[Axis::Z as usize] as f32
            }) + off[2],
        ];
        let arc_center_position = [
            self.m_arc_center[0] + self.m_x_offset as f32 + off[0],
            self.m_arc_center[1] + self.m_y_offset as f32 + off[1],
            self.m_arc_center[2] + off[2],
        ];

        let move_vertex = MoveVertex {
            move_type: r#type,
            extrusion_role: self.m_extrusion_role,
            move_path_type: path_type,
            extruder_id: filament_id as u8,
            cp_color_id: self.m_cp_color.current,
            gcode_id: self.m_last_line_id,
            delta_extruder: (self.m_end_position[Axis::E as usize]
                - self.m_start_position[Axis::E as usize]) as f32,
            feedrate: self.m_feedrate,
            width: self.m_width,
            height: self.m_height,
            mm3_per_mm: self.m_mm3_per_mm,
            fan_speed: self.m_fan_speed,
            additional_fan_speed: self.m_additional_fan_speed,
            temperature: self.m_extruder_temps[filament_id as usize],
            layer_duration: self.m_layer_id as f32,
            thermal_index_min: self.m_thermal_index.min,
            thermal_index_max: self.m_thermal_index.max,
            thermal_index_mean: self.m_thermal_index.mean,
            time: [0.0, 0.0],
            position,
            arc_center_position,
            interpolation_points: self.m_interpolation_points.clone(),
            object_label_id: self.m_object_label_id,
            print_z: self.m_print_z,
        };
        self.m_result.moves.push(move_vertex);

        if r#type == EMoveType::Seam {
            self.m_seams_count += 1;
        }

        // stores stop time placeholders for later use
        if r#type == EMoveType::ColorChange || r#type == EMoveType::PausePrint {
            for i in 0..ETimeMode::COUNT {
                let machine = &mut self.m_time_processor.machines[i];
                if !machine.enabled {
                    continue;
                }
                machine.stop_times.push(StopTime {
                    g1_line_id: self.m_g1_line_id,
                    elapsed_time: 0.0,
                });
            }
        }
    }

    /// GCodeProcessor.cpp:2653-2731  void GCodeProcessor::finalize(bool post_process)
    pub fn finalize(&mut self, _post_process: bool) {
        // update width/height of wipe moves
        for move_v in self.m_result.moves.iter_mut() {
            if move_v.move_type == EMoveType::Wipe {
                move_v.width = WIPE_WIDTH;
                move_v.height = WIPE_HEIGHT;
            }
        }

        // process the time blocks
        for i in 0..ETimeMode::COUNT {
            self.run_calculate_time(i, 0, 0.0, ExtrusionRole::None);
            let gcode_time = &mut self.m_time_processor.machines[i].gcode_time;
            if gcode_time.needed && gcode_time.cache != 0.0 {
                gcode_time
                    .times
                    .push((CustomGCodeType::ColorChange, gcode_time.cache));
            }
        }

        // m_used_filaments.process_caches(this);
        self.process_used_filaments_caches();

        self.update_estimated_times_stats();

        // prepare_time = roles_times for erCustom (cpp:2677-2683)
        let normal_mode = &self.m_result.print_statistics.modes[ETimeMode::Normal as usize];
        let prepare_time = normal_mode
            .roles_times
            .iter()
            .find(|(role, _)| *role == ExtrusionRole::Custom)
            .map(|(_, t)| *t)
            .unwrap_or(0.0);
        let layer_times = normal_mode.layers_times.clone();
        self.m_result.initial_layer_time = if !layer_times.is_empty() {
            (layer_times[0] - prepare_time).max(0.0)
        } else {
            0.0
        };

        // BLOCKED(deps): post_process / TimeProcessContext / MultiNozzleUtils +
        // PreCoolingInjector — not ported (require unported config/nozzle deps).

        // update layer_duration for each move (cpp:2718-2725)
        for mv in self.m_result.moves.iter_mut() {
            let layer_id = mv.layer_duration as usize;
            if layer_times.len() > layer_id.wrapping_sub(1) && layer_id > 0 {
                mv.layer_duration = if layer_id == 1 {
                    (layer_times[layer_id - 1] - prepare_time).max(0.0)
                } else {
                    layer_times[layer_id - 1]
                };
            } else {
                mv.layer_duration = 0.0;
            }
        }

        // BLOCKED(deps): update_slice_warnings — needs config/bed warnings.

        // ---- parity-facing summary fields (not C++ Result members) ----
        self.m_result.print_time =
            self.m_result.print_statistics.modes[ETimeMode::Normal as usize].time;
        // total filament volume = sum of total_volumes_per_extruder (mm^3)
        let total_mm3: f64 = self
            .m_result
            .print_statistics
            .total_volumes_per_extruder
            .values()
            .sum();
        self.m_result.filament_used_mm3 = total_mm3;
        // length (mm) and grams from filament cross-section / density (use
        // filament 0 diameter/density as the representative value).
        let diameter = *self
            .m_result
            .filament_diameters
            .first()
            .unwrap_or(&DEFAULT_FILAMENT_DIAMETER);
        let density = *self
            .m_result
            .filament_densities
            .first()
            .unwrap_or(&DEFAULT_FILAMENT_DENSITY);
        let cross_section = PI * sqr(0.5 * diameter) as f64;
        self.m_result.filament_used_mm = if cross_section != 0.0 {
            total_mm3 / cross_section
        } else {
            0.0
        };
        self.m_result.filament_used_g = total_mm3 * density as f64 * 0.001;
    }

    /// GCodeProcessor.cpp:1476-1484  UsedFilaments::process_caches(this)
    fn process_used_filaments_caches(&mut self) {
        let active_filament_id = self.get_filament_id(true);
        let active_filament_id_support = self.get_filament_id(false);
        let diameter = if (active_filament_id as usize) < self.m_result.filament_diameters.len() {
            self.m_result.filament_diameters[active_filament_id as usize]
        } else {
            *self
                .m_result
                .filament_diameters
                .last()
                .unwrap_or(&DEFAULT_FILAMENT_DIAMETER)
        };
        let density = if (active_filament_id as usize) < self.m_result.filament_densities.len() {
            self.m_result.filament_densities[active_filament_id as usize]
        } else {
            *self
                .m_result
                .filament_densities
                .last()
                .unwrap_or(&DEFAULT_FILAMENT_DENSITY)
        };
        let role = self.m_extrusion_role;
        self.m_used_filaments
            .process_caches(active_filament_id, active_filament_id_support, diameter, density, role);
    }

    /// GCodeProcessor.cpp:6212-6237  void GCodeProcessor::update_estimated_times_stats()
    fn update_estimated_times_stats(&mut self) {
        self.update_mode(ETimeMode::Normal);
        if self.m_time_processor.machines[ETimeMode::Stealth as usize].enabled {
            self.update_mode(ETimeMode::Stealth);
        } else {
            self.m_result.print_statistics.modes[ETimeMode::Stealth as usize].reset();
        }

        let uf = &self.m_used_filaments;
        let ps = &mut self.m_result.print_statistics;
        ps.volumes_per_color_change = uf.volumes_per_color_change.clone();
        ps.model_volumes_per_extruder = uf.model_volumes_per_filament.clone();
        ps.wipe_tower_volumes_per_extruder = uf.wipe_tower_volumes_per_filament.clone();
        ps.support_volumes_per_extruder = uf.support_volumes_per_filament.clone();
        ps.flush_per_filament = uf.flush_per_filament.clone();
        ps.used_filaments_per_role = uf.filaments_per_role.clone();
        ps.total_volumes_per_extruder = uf.total_volumes_per_filament.clone();
    }

    /// update_mode lambda (cpp:6214-6222)
    fn update_mode(&mut self, mode: ETimeMode) {
        let id = mode as usize;
        let machine = &self.m_time_processor.machines[id];
        let time = machine.time;
        let prepare_time = machine.prepare_time;

        // custom_gcode_times (cpp:2748-2761, include_remaining=true)
        let mut custom_gcode_times: Vec<(CustomGCodeType, (f32, f32))> = Vec::new();
        let mut total_time = 0.0f32;
        for &(t, time_v) in machine.gcode_time.times.iter() {
            let remaining = machine.time - total_time;
            custom_gcode_times.push((t, (time_v, remaining)));
            total_time += time_v;
        }

        // moves_times (cpp:2763-2774)
        let mut moves_times: Vec<(EMoveType, f32)> = Vec::new();
        for (i, &t) in machine.moves_time.iter().enumerate() {
            if t > 0.0 {
                moves_times.push((emove_type_from_index(i), t));
            }
        }

        // roles_times (cpp:2776-2787)
        let mut roles_times: Vec<(ExtrusionRole, f32)> = Vec::new();
        for (i, &t) in machine.roles_time.iter().enumerate() {
            if t > 0.0 {
                roles_times.push((extrusion_role_from_index(i), t));
            }
        }

        let layers_times = machine.layers_time.clone();

        // TIMEDBG: env-gated ground-truth dump of the estimator's per-role /
        // per-move-type time breakdown, to localize the rust-vs-native gap.
        if std::env::var("TIMEDBG").is_ok() && id == ETimeMode::Normal as usize {
            eprintln!("TIMEDBG mode=Normal total={:.1}s ({:.1}min)", time, time / 60.0);
            let mut rt: Vec<_> = roles_times.clone();
            rt.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
            for (role, t) in &rt {
                eprintln!("TIMEDBG role {:?} = {:.1}s", role, t);
            }
            for (mv, t) in &moves_times {
                eprintln!("TIMEDBG move {:?} = {:.1}s", mv, t);
            }
        }

        let data = &mut self.m_result.print_statistics.modes[id];
        data.time = time;
        data.prepare_time = prepare_time;
        data.custom_gcode_times = custom_gcode_times;
        data.moves_times = moves_times;
        data.roles_times = roles_times;
        data.layers_times = layers_times;
    }

    /// Convenience accessor mirroring the divergent module's `print_time()`.
    pub fn print_time(&self) -> f32 {
        self.m_result.print_time
    }
}

impl Default for GCodeProcessor {
    fn default() -> Self {
        Self::new()
    }
}

/// `EPSILON` from `libslic3r.h`.
const EPSILON: f32 = 1e-4;

/// `static_cast<EMoveType>(i)` — index → EMoveType (cpp:2770).
fn emove_type_from_index(i: usize) -> EMoveType {
    match i {
        0 => EMoveType::Noop,
        1 => EMoveType::Retract,
        2 => EMoveType::Unretract,
        3 => EMoveType::Seam,
        4 => EMoveType::ToolChange,
        5 => EMoveType::ColorChange,
        6 => EMoveType::PausePrint,
        7 => EMoveType::CustomGCode,
        8 => EMoveType::Travel,
        9 => EMoveType::Wipe,
        _ => EMoveType::Extrude,
    }
}

/// `static_cast<ExtrusionRole>(i)` — index → ExtrusionRole (cpp:2783).
fn extrusion_role_from_index(i: usize) -> ExtrusionRole {
    match i {
        0 => ExtrusionRole::None,
        1 => ExtrusionRole::Perimeter,
        2 => ExtrusionRole::ExternalPerimeter,
        3 => ExtrusionRole::OverhangPerimeter,
        4 => ExtrusionRole::InternalInfill,
        5 => ExtrusionRole::SolidInfill,
        6 => ExtrusionRole::FloatingVerticalShell,
        7 => ExtrusionRole::TopSolidInfill,
        8 => ExtrusionRole::BottomSurface,
        9 => ExtrusionRole::Ironing,
        10 => ExtrusionRole::BridgeInfill,
        11 => ExtrusionRole::GapFill,
        12 => ExtrusionRole::Skirt,
        13 => ExtrusionRole::Brim,
        14 => ExtrusionRole::SupportMaterial,
        15 => ExtrusionRole::SupportMaterialInterface,
        16 => ExtrusionRole::SupportTransition,
        17 => ExtrusionRole::SupportIroning,
        18 => ExtrusionRole::WipeTower,
        19 => ExtrusionRole::Custom,
        20 => ExtrusionRole::Flush,
        _ => ExtrusionRole::Mixed,
    }
}

/// 3D in-place normalize (Eigen `Vec3f::normalize()`); zero vector stays zero.
// FIDELITY-NOTE(F1): Eigen `normalize()` divides unconditionally and yields NaN
// for a zero-length vector; we guard `n != 0.0` and leave a zero vector unchanged.
// This only differs on degenerate zero-direction inputs.
fn normalize3(v: &mut [f32; 3]) {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n != 0.0 {
        v[0] /= n;
        v[1] /= n;
        v[2] /= n;
    }
}

/// 3D normalize returning a new vector (Eigen `Vec3f::normalized()`).
fn normalized3(v: [f32; 3]) -> [f32; 3] {
    let mut out = v;
    normalize3(&mut out);
    out
}

/// Parse a Klipper `KEY=<float>` token from a raw line (mirrors the regex
/// `\sKEY\s*=\s*([0-9]*\.*[0-9]*)`). Returns the parsed value if present.
/// `std::regex_search(raw, matches, std::regex("\\s<KEY>\\s*=\\s*([0-9]*\\.*[0-9]*)"))` then
/// `std::stof(matches[1])` (GCodeProcessor.cpp:5713-5759): None when nothing matches, the value when
/// it does, 0 when `stof` throws on the group.
fn klipper_regex_value(raw: &str, key: &str) -> Option<f32> {
    let s = raw.as_bytes();
    let is_space = |c: u8| matches!(c, b' ' | b'\t' | b'\n' | b'\r' | 0x0b | 0x0c);
    for i in 0..s.len() {
        if !is_space(s[i]) || !s[i + 1..].starts_with(key.as_bytes()) {
            continue;
        }
        let mut j = i + 1 + key.len();
        while j < s.len() && is_space(s[j]) {
            j += 1;
        }
        if j >= s.len() || s[j] != b'=' {
            continue;
        }
        j += 1;
        while j < s.len() && is_space(s[j]) {
            j += 1;
        }
        let start = j;
        while j < s.len() && s[j].is_ascii_digit() {
            j += 1;
        }
        while j < s.len() && s[j] == b'.' {
            j += 1;
        }
        while j < s.len() && s[j].is_ascii_digit() {
            j += 1;
        }
        let r = crate::config::strtod_c(&s[start..j]);
        return Some(if r.consumed == 0 { 0.0 } else { r.value as f32 });
    }
    None
}

fn parse_klipper_kv(raw: &str, key: &str) -> Option<f32> {
    let idx = raw.find(key)?;
    // Require whitespace before the key (the `\s` in the regex).
    if idx == 0 {
        return None;
    }
    let prev = raw.as_bytes()[idx - 1];
    if prev != b' ' && prev != b'\t' {
        return None;
    }
    let after = &raw[idx + key.len()..];
    let after = after.trim_start();
    let after = after.strip_prefix('=')?;
    let after = after.trim_start();
    // capture [0-9]*\.*[0-9]*
    let mut end = 0usize;
    let bytes = after.as_bytes();
    while end < bytes.len() && (bytes[end].is_ascii_digit() || bytes[end] == b'.') {
        end += 1;
    }
    after[..end].parse::<f32>().ok()
}


// ===========================================================================
// GCodeProcessor.cpp:97-119 — the option helpers of the machine limits
// ===========================================================================

/// GCodeProcessor.cpp:97-119 `static void set_option_value(...)` / `static float
/// get_option_value(...)`: the two overloads of each, on `ConfigOptionFloats` and
/// `ConfigOptionFloatsNullable`.
pub trait MachineLimitOption {
    /// GCodeProcessor.cpp:97-107 `set_option_value(option, id, value)`
    fn set_option_value(&mut self, id: usize, value: f32);
    /// GCodeProcessor.cpp:109-119 `get_option_value(option, id)`: 0 when empty, the last value
    /// past the end.
    fn get_option_value(&self, id: usize) -> f32;
}

impl MachineLimitOption for crate::config::ConfigOptionFloats {
    /// GCodeProcessor.cpp:97-101 `static void set_option_value(ConfigOptionFloats& option, size_t id,
    /// float value)`
    fn set_option_value(&mut self, id: usize, value: f32) {
        if id < self.values.len() {
            self.values[id] = value as f64;
        }
    }

    /// GCodeProcessor.cpp:109-113 `static float get_option_value(const ConfigOptionFloats& option,
    /// size_t id)`
    fn get_option_value(&self, id: usize) -> f32 {
        if self.values.is_empty() {
            0.0
        } else if id < self.values.len() {
            self.values[id] as f32
        } else {
            *self.values.last().unwrap() as f32
        }
    }
}

impl MachineLimitOption for crate::config::ConfigOptionFloatsNullable {
    /// GCodeProcessor.cpp:103-107 `static void set_option_value(ConfigOptionFloatsNullable& option,
    /// size_t id, float value)`
    fn set_option_value(&mut self, id: usize, value: f32) {
        if id < self.values.len() {
            self.values[id] = value as f64;
        }
    }

    /// GCodeProcessor.cpp:115-119 `static float get_option_value(const ConfigOptionFloatsNullable&
    /// option, size_t id)`
    fn get_option_value(&self, id: usize) -> f32 {
        if self.values.is_empty() {
            0.0
        } else if id < self.values.len() {
            self.values[id] as f32
        } else {
            *self.values.last().unwrap() as f32
        }
    }
}

/// GCodeProcessor.cpp:2649-2652 `static inline const char* skip_whitespaces(const char *begin, const char
/// *end)`: the first position of `s[begin..end]` that is not a blank or a tab.
pub fn skip_whitespaces(s: &[u8], mut begin: usize, end: usize) -> usize {
    while begin != end && (s[begin] == b' ' || s[begin] == b'\t') {
        begin += 1;
    }
    begin
}

/// GCodeProcessor.cpp:2654-2657 `static inline const char* remove_eols(const char *begin, const char
/// *end)`: the end of `s[begin..end]` without its trailing line ends.
pub fn remove_eols(s: &[u8], begin: usize, mut end: usize) -> usize {
    while begin != end && (s[end - 1] == b'\r' || s[end - 1] == b'\n') {
        end -= 1;
    }
    end
}

/// The number types `parse_number` reads (GCodeProcessor.cpp:3129-3170): what the conversion leaves
/// in `out` (None: `out` is not written) and whether the number spanned the whole input.
pub trait ParseNumber: Sized + Copy {
    fn parse_number_impl(sv: &str) -> (Option<Self>, bool);
}

impl ParseNumber for f32 {
    /// The legacy branch (the macOS standard library has no `std::from_chars` for floating point):
    /// `out = string_to_double_decimal_point(str, &read); return str.size() == read;` (the value is
    /// written whatever was read; nothing read leaves 0).
    fn parse_number_impl(sv: &str) -> (Option<f32>, bool) {
        let (v, read) = crate::locales_utils::string_to_double_decimal_point(sv);
        (Some(v as f32), sv.len() == read)
    }
}

impl ParseNumber for i32 {
    /// `std::from_chars(first, last, int&)`: an optional '-', decimal digits; `out` is written when
    /// digits were read and the value fits, and the result is true only when they span the input.
    fn parse_number_impl(sv: &str) -> (Option<i32>, bool) {
        let b = sv.as_bytes();
        let mut i = 0usize;
        if i < b.len() && b[i] == b'-' {
            i += 1;
        }
        let digits_start = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        if i == digits_start {
            // std::errc::invalid_argument
            return (None, false);
        }
        match sv[..i].parse::<i32>() {
            Ok(v) => (Some(v), i == b.len()),
            // std::errc::result_out_of_range
            Err(_) => (None, false),
        }
    }
}

/// GCodeProcessor.cpp:3137-3170 `template<typename T> static inline bool parse_number(const
/// std::string_view sv, T &out)`: true when the number was parsed into `out` and spanned the whole
/// input string.
pub fn parse_number<T: ParseNumber>(sv: &str, out: &mut T) -> bool {
    let (v, whole) = T::parse_number_impl(sv);
    if let Some(v) = v {
        *out = v;
    }
    v.is_some() && whole
}

// ===========================================================================
// GCodeProcessor.hpp:389-450 namespace ExtruderPreHeating
// ===========================================================================

/// GCodeProcessor.hpp:415-448 `struct ExtruderUsageBlcok`: the usage of an extruder in a section,
/// between two extruder change blocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExtruderUsageBlcok {
    pub extruder_id: i32,
    pub start_id: u32,
    pub end_id: u32,
    pub start_filament: i32,
    pub end_filament: i32,
    pub start_nozzle_id: i32,
    pub end_nozzle_id: i32,
    pub post_extrusion_start_id: u32,
    pub post_extrusion_end_id: u32,
    pub ignore_cooling_before_tower: bool,
}

impl Default for ExtruderUsageBlcok {
    /// GCodeProcessor.hpp:447 `ExtruderUsageBlcok() = default;` with the member initializers.
    fn default() -> Self {
        ExtruderUsageBlcok {
            extruder_id: -1,
            start_id: u32::MAX,
            end_id: u32::MAX,
            start_filament: -1,
            end_filament: -1,
            start_nozzle_id: -1,
            end_nozzle_id: -1,
            post_extrusion_start_id: u32::MAX,
            post_extrusion_end_id: u32::MAX,
            ignore_cooling_before_tower: false,
        }
    }
}

impl ExtruderUsageBlcok {
    /// GCodeProcessor.hpp:428-433 `void initialize_step_1(int extruder_id_, int start_id_, int
    /// start_filament_, int start_nozzle_id_)`
    pub fn initialize_step_1(&mut self, extruder_id: i32, start_id: i32, start_filament: i32, start_nozzle_id: i32) {
        self.extruder_id = extruder_id;
        self.start_id = start_id as u32;
        self.start_filament = start_filament;
        self.start_nozzle_id = start_nozzle_id;
    }

    /// GCodeProcessor.hpp:434-436 `void initialize_step_2(int post_extrusion_start_id_)`
    pub fn initialize_step_2(&mut self, post_extrusion_start_id: i32) {
        self.post_extrusion_start_id = post_extrusion_start_id as u32;
    }

    /// GCodeProcessor.hpp:437-442 `void initialize_step_3(int end_id_, int end_filament_, int
    /// post_extrusion_end_id_, int end_nozzle_id_)`
    pub fn initialize_step_3(&mut self, end_id: i32, end_filament: i32, post_extrusion_end_id: i32, end_nozzle_id: i32) {
        self.end_id = end_id as u32;
        self.end_filament = end_filament;
        self.post_extrusion_end_id = post_extrusion_end_id as u32;
        self.end_nozzle_id = end_nozzle_id;
    }

    /// GCodeProcessor.hpp:443-445 `void reset() { *this = ExtruderUsageBlcok(); }`
    pub fn reset(&mut self) {
        *self = ExtruderUsageBlcok::default();
    }
}

/// GCodeProcessor.hpp:785-799 `struct FilamentVariantRows`: variant-expanded filament rows with
/// their labels.
#[derive(Debug, Clone, Default)]
pub struct FilamentVariantRows {
    pub nozzle_temp: Vec<i32>,
    pub pre_cooling_temp: Vec<i32>,
    pub preheat_temperature_delta: Vec<f64>,
    pub extruder_variant: Vec<String>,
    pub self_index: Vec<i32>,
    // Indexed by filament id: row of the filament's statically assigned variant, used when the nozzle is unknown.
    pub default_row: Vec<i32>,
}

impl FilamentVariantRows {
    /// GCodeProcessor.hpp:794-798 `template<class T> static T value_at(const std::vector<T>& values,
    /// int row)`: the same fallback as `ConfigOptionVector::get_at()` for arrays shorter than the
    /// row count.
    pub fn value_at<T: Clone + Default>(values: &[T], row: i32) -> T {
        if values.is_empty() {
            T::default()
        } else {
            values[if (row as usize) < values.len() { row as usize } else { 0 }].clone()
        }
    }
}

// ===========================================================================
// GCodeProcessor.hpp:389-397, 800-878, 929-1040 / GCodeProcessor.cpp:571-1300, 6611-7203:
// the pre-heating / pre-cooling pass of TimeProcessor::post_process (R946)
// ===========================================================================

/// GCodeProcessor.hpp:389-397 `struct ExtruderPreHeating::FilamentUsageBlock`: `[lower_gcode_id,
/// upper_gcode_id)` uses the filament; the upper id is set when the next block is found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FilamentUsageBlock {
    pub filament_id: i32,
    pub extruder_id: i32,
    pub nozzle_id: i32,
    pub lower_gcode_id: u32,
    pub upper_gcode_id: u32,
}

impl FilamentUsageBlock {
    /// GCodeProcessor.hpp:396 `FilamentUsageBlock(int filament_id_, int extruder_id_, int nozzle_id_,
    /// unsigned int lower_gcode_id_, unsigned int upper_gcode_id_)`
    pub fn new(filament_id: i32, extruder_id: i32, nozzle_id: i32, lower_gcode_id: u32, upper_gcode_id: u32) -> Self {
        FilamentUsageBlock { filament_id, extruder_id, nozzle_id, lower_gcode_id, upper_gcode_id }
    }
}

/// GCodeProcessor.hpp:868-876 `enum TimeProcessor::InsertLineType`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsertLineType {
    PlaceholderReplace,
    TimePredict,
    FilamentChangePredict,
    ExtruderChangePredict,
    PreCooling,
    PreHeating,
}

/// GCodeProcessor.hpp:879 `using InsertedLinesMap = std::map<unsigned int, std::vector<std::pair<std::string,
/// InsertLineType>>>` (first key is the line id, second the content).
pub type InsertedLinesMap = std::collections::BTreeMap<u32, Vec<(String, InsertLineType)>>;

/// The two members of `GCodeProcessorResult::MoveVertex` the pre-cooling pass reads
/// (GCodeProcessor.hpp MoveVertex `gcode_id`, `time`), copied out of the result so the pass does not
/// clone the whole vertex list.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveVertexTime {
    pub gcode_id: u32,
    pub time: [f32; 2],
}

/// GCodeProcessor.hpp:800-863 `struct TimeProcessContext`: the members the pre-heating pass reads
/// (the placeholders the pass also replaces are written by the port's header itself).
#[derive(Debug, Clone)]
pub struct TimeProcessContext {
    pub filament_types: Vec<String>,
    pub filament_pre_heating_enabled: Vec<u8>,
    pub physical_extruder_map: Vec<i32>,
    pub nozzle_group_result: crate::multi_nozzle_utils::LayeredNozzleGroupResult,
    pub cooling_rate: Vec<f64>,
    pub cooling_rate_on_rack: Vec<f64>,
    pub heating_rate: Vec<f64>,
    pub filament_min_temperature_when_ec: Vec<f64>,
    pub inject_time_threshold: f32,
    pub enable_pre_heating: bool,
    pub handle_hotend_as_extruder: bool,
    pub has_filament_switcher: bool,
    pub extruder_max_nozzle_count: Vec<i32>,
    pub extruder_types: Vec<crate::extruder::ExtruderType>,
    pub nozzle_diameter: Vec<f64>,
    pub filament_variant_rows: FilamentVariantRows,
}

impl TimeProcessContext {
    /// The context `GCodeProcessor::finalize(true)` builds (GCodeProcessor.cpp:2831-2853) from what
    /// `GCodeProcessor::apply_config(const PrintConfig&)` stored (GCodeProcessor.cpp:1979-2013):
    /// `inject_time_threshold` is the constant 0 of :2832, the nozzle group result the print's
    /// (`DoExport::init_gcode_processor`, GCode.cpp:2193-2204 -> `initialize_from_context`).
    pub fn from_config(
        config: &crate::print_config::config_def::DynamicPrintConfig,
        nozzle_group_result: crate::multi_nozzle_utils::LayeredNozzleGroupResult,
    ) -> Self {
        use crate::config::ConfigOptionResolver;
        use crate::gcode::wipe_tower::{cfg_bools, cfg_floats, cfg_ints, cfg_strings};
        use crate::extruder::{ExtruderType, NozzleVolumeType};
        let filament_count = cfg_floats(config, "filament_diameter").len();
        let get_at_i = |v: &[i32], i: usize| -> i32 {
            if v.is_empty() { 0 } else if i < v.len() { v[i] } else { v[0] }
        };
        // GCodeProcessor.cpp:1979-1986 `m_filament_variant_rows`
        let mut rows = FilamentVariantRows {
            nozzle_temp: cfg_ints(config, "nozzle_temperature"),
            pre_cooling_temp: cfg_ints(config, "filament_pre_cooling_temperature"),
            preheat_temperature_delta: cfg_floats(config, "filament_preheat_temperature_delta"),
            extruder_variant: cfg_strings(config, "filament_extruder_variant"),
            self_index: cfg_ints(config, "filament_self_index"),
            default_row: vec![0; filament_count],
        };
        // PrintConfig.cpp:485-492 get_filament_config_idx(config, idx), with get_extruder_index
        // (PrintConfig.cpp:92-98) over filament_map.
        let filament_volume_map = cfg_ints(config, "filament_volume_map");
        let filament_map = cfg_ints(config, "filament_map");
        let extruder_type = cfg_ints(config, "extruder_type");
        for idx in 0..filament_count {
            let volume_type = NozzleVolumeType::from_i32(get_at_i(&filament_volume_map, idx));
            let extruder_index = if idx < filament_map.len() { (filament_map[idx] - 1) as usize } else { 0 };
            let et = ExtruderType::from_i32(get_at_i(&extruder_type, extruder_index));
            rows.default_row[idx] = crate::extruder::get_config_index_base(
                volume_type,
                et,
                idx as i32 + 1,
                &rows.extruder_variant,
                &rows.self_index,
            );
        }
        let filament_type = cfg_strings(config, "filament_type");
        let pre_heating_enabled = cfg_bools(config, "filament_pre_heating_enabled");
        TimeProcessContext {
            // GCodeProcessor.cpp:1988-1990 `config.filament_type.get_at(idx)`
            filament_types: (0..filament_count)
                .map(|i| if i < filament_type.len() { filament_type[i].clone() } else { filament_type.first().cloned().unwrap_or_default() })
                .collect(),
            // :1992-1994
            filament_pre_heating_enabled: (0..filament_count)
                .map(|i| {
                    let v = if i < pre_heating_enabled.len() { pre_heating_enabled[i] } else { pre_heating_enabled.first().copied().unwrap_or(false) };
                    if v { 1 } else { 0 }
                })
                .collect(),
            physical_extruder_map: cfg_ints(config, "physical_extruder_map"),
            nozzle_group_result,
            cooling_rate: cfg_floats(config, "hotend_cooling_rate"),
            cooling_rate_on_rack: cfg_floats(config, "hotend_cooling_rate_on_rack"),
            heating_rate: cfg_floats(config, "hotend_heating_rate"),
            filament_min_temperature_when_ec: cfg_floats(config, "min_temperature_when_ec"),
            inject_time_threshold: 0.0,
            enable_pre_heating: config.opt_bool("enable_pre_heating"),
            handle_hotend_as_extruder: config.opt_bool("handle_hotend_as_extruder"),
            has_filament_switcher: config.opt_bool("has_filament_switcher"),
            extruder_max_nozzle_count: cfg_ints(config, "extruder_max_nozzle_count"),
            // GCodeProcessor.cpp `m_result.extruder_types` = config.extruder_type.values
            extruder_types: extruder_type.iter().map(|&v| ExtruderType::from_i32(v)).collect(),
            nozzle_diameter: cfg_floats(config, "nozzle_diameter"),
            filament_variant_rows: rows,
        }
    }
}

/// GCodeProcessor.hpp:931-944 `struct PreCoolingInjector::ExtruderFreeBlock`
#[derive(Debug, Clone, Copy, Default)]
pub struct ExtruderFreeBlock {
    pub free_lower_gcode_id: u32,
    pub free_upper_gcode_id: u32,
    pub partial_free_lower_id: u32,
    pub partial_free_upper_id: u32,
    pub last_filament_id: i32,
    pub next_filament_id: i32,
    pub last_nozzle_id: i32,
    pub next_nozzle_id: i32,
    pub extruder_id: i32,
    pub idle_chain_key: i32,
    pub ignore_cooling_before_tower: bool,
}

/// GCodeProcessor.hpp:929-1040 `class PreCoolingInjector`
pub struct PreCoolingInjector<'a> {
    m_extruder_free_blocks: Vec<ExtruderFreeBlock>,
    moves: &'a [MoveVertexTime],
    ctx: &'a TimeProcessContext,
    valid_machine_id: usize,
    skippable_blocks: &'a [(u32, u32)],
    machine_start_gcode_end_id: u32,
    machine_end_gcode_start_id: u32,
}

impl<'a> PreCoolingInjector<'a> {
    /// GCodeProcessor.hpp:948-990 the constructor (GCodeProcessor.cpp:1216-1237 passes the
    /// context's members).
    pub fn new(
        moves: &'a [MoveVertexTime],
        ctx: &'a TimeProcessContext,
        valid_machine_id: usize,
        skippable_blocks: &'a [(u32, u32)],
        machine_start_gcode_end_id: u32,
        machine_end_gcode_start_id: u32,
    ) -> Self {
        PreCoolingInjector {
            m_extruder_free_blocks: Vec::new(),
            moves,
            ctx,
            valid_machine_id,
            skippable_blocks,
            machine_start_gcode_end_id,
            machine_end_gcode_start_id,
        }
    }

    /// GCodeProcessor.cpp:6611-6626 `filament_variant_row(filament_id, nozzle_id)`
    pub fn filament_variant_row(&self, filament_id: i32, nozzle_id: i32) -> i32 {
        let rows = &self.ctx.filament_variant_rows;
        if filament_id < 0 || filament_id >= rows.default_row.len() as i32 {
            return -1;
        }
        if nozzle_id >= 0 && !rows.extruder_variant.is_empty() && rows.extruder_variant.len() == rows.self_index.len() {
            if let Some(nozzle) = self.ctx.nozzle_group_result.get_nozzle_from_id(nozzle_id) {
                if nozzle.extruder_id >= 0 && nozzle.extruder_id < self.ctx.extruder_types.len() as i32 {
                    let row = crate::extruder::get_config_index_base(
                        nozzle.volume_type,
                        self.ctx.extruder_types[nozzle.extruder_id as usize],
                        filament_id + 1,
                        &rows.extruder_variant,
                        &rows.self_index,
                    );
                    // get_config_index_base() returns 0 when the filament owns no row at all.
                    if row >= 0 && row < rows.self_index.len() as i32 && rows.self_index[row as usize] == filament_id + 1 {
                        return row;
                    }
                }
            }
        }
        rows.default_row[filament_id as usize]
    }

    /// GCodeProcessor.cpp:6628-6689 `process_pre_cooling_and_heating(inserted_operation_lines)`
    pub fn process_pre_cooling_and_heating(&self, inserted_operation_lines: &mut InsertedLinesMap) {
        let ctx = self.ctx;
        let rows = &ctx.filament_variant_rows;
        // :6630 (is_multiple_nozzle is captured by get_nozzle_temp but not read)
        // :6631-6640
        let get_nozzle_temp = |filament_id: i32, nozzle_id: i32, from_or_to: bool, consider_preheat_temperature_delta: bool| -> i32 {
            let row = self.filament_variant_row(filament_id, nozzle_id);
            if row < 0 {
                return if from_or_to { 140 } else { 0 }; // default temp for unknown filament
            }
            let temp = FilamentVariantRows::value_at(&rows.nozzle_temp, row) as f64;
            if consider_preheat_temperature_delta {
                (temp - FilamentVariantRows::value_at(&rows.preheat_temperature_delta, row)) as i32
            } else {
                temp as i32
            }
        };
        // :6644-6648
        let has_mixed_extruder_types =
            ctx.extruder_types.len() > 1 && ctx.extruder_types.windows(2).any(|w| w[0] != w[1]);
        let first_nozzle_dia: f32 = if ctx.nozzle_diameter.is_empty() { 0.4 } else { ctx.nozzle_diameter[0] as f32 };
        let switcher_temp_offset: f32 =
            if (first_nozzle_dia as f64) >= 0.6 - crate::libslic3r::EPSILON { 40.0 } else { 20.0 };
        // :6653-6659
        let wants_pre_heating = |filament_id: i32| -> bool {
            if !ctx.handle_hotend_as_extruder {
                return true;
            }
            if filament_id < 0 || filament_id >= ctx.filament_pre_heating_enabled.len() as i32 {
                return true;
            }
            ctx.filament_pre_heating_enabled[filament_id as usize] != 0
        };

        // :6661-6664
        let mut per_extruder_free_blocks: std::collections::BTreeMap<i32, Vec<ExtruderFreeBlock>> =
            std::collections::BTreeMap::new();
        for block in &self.m_extruder_free_blocks {
            per_extruder_free_blocks.entry(block.idle_chain_key).or_default().push(*block);
        }

        // :6666-6688
        for (_extruder_id, extruder_free_blcoks) in per_extruder_free_blocks.iter() {
            let n = extruder_free_blcoks.len();
            for (k, iter) in extruder_free_blcoks.iter().enumerate() {
                let is_end = k + 1 == n;
                let is_begin = k == 0;
                let apply_pre_cooling = !is_begin || !ctx.handle_hotend_as_extruder;
                let apply_pre_heating = if is_end { false } else { wants_pre_heating(iter.next_filament_id) };
                let curr_temp: f32 = if iter.last_filament_id == -1 && ctx.handle_hotend_as_extruder {
                    0.0
                } else {
                    get_nozzle_temp(iter.last_filament_id, iter.last_nozzle_id, true, false) as f32
                };
                let mut target_temp: f32 =
                    get_nozzle_temp(iter.next_filament_id, iter.next_nozzle_id, false, !iter.ignore_cooling_before_tower) as f32;
                if ctx.has_filament_switcher && has_mixed_extruder_types && apply_pre_heating {
                    let print_temp = get_nozzle_temp(iter.next_filament_id, iter.next_nozzle_id, false, false) as f32;
                    target_temp = target_temp.min(print_temp - switcher_temp_offset);
                }
                self.inject_cooling_heating_command(
                    inserted_operation_lines,
                    iter,
                    curr_temp,
                    target_temp,
                    apply_pre_cooling,
                    apply_pre_heating,
                );
            }
        }
    }

    /// GCodeProcessor.cpp:6691-6697 `build_extruder_free_blocks(filament_usage_blocks, extruder_usage_blocks)`
    pub fn build_extruder_free_blocks(
        &mut self,
        filament_usage_blocks: &[FilamentUsageBlock],
        extruder_usage_blocks: &[ExtruderUsageBlcok],
    ) {
        if extruder_usage_blocks.len() <= 1 {
            self.build_by_filament_blocks(filament_usage_blocks);
        } else {
            self.build_by_extruder_blocks(filament_usage_blocks, extruder_usage_blocks);
        }
    }

    /// The `std::sort` of :7045-7047 / :7104-7106 / :7157-7159 by (free_lower, free_upper). The
    /// comparison is a strict order on the two ids; blocks equal in both (not met on the jobs here)
    /// keep their order.
    fn sort_free_blocks(&mut self) {
        self.m_extruder_free_blocks.sort_by(|a, b| {
            (a.free_lower_gcode_id, a.free_upper_gcode_id).cmp(&(b.free_lower_gcode_id, b.free_upper_gcode_id))
        });
    }

    /// GCodeProcessor.cpp:6699-6996 `inject_cooling_heating_command(inserted_operation_lines, block,
    /// curr_temp, target_temp, pre_cooling, pre_heating)`
    pub fn inject_cooling_heating_command(
        &self,
        inserted_operation_lines: &mut InsertedLinesMap,
        block: &ExtruderFreeBlock,
        mut curr_temp: f32,
        target_temp: f32,
        pre_cooling: bool,
        pre_heating: bool,
    ) {
        let ctx = self.ctx;
        let rows = &ctx.filament_variant_rows;
        let moves = self.moves;
        let vm = self.valid_machine_id;
        // :6703-6706
        let get_valid_extruder_id = |last_nozzle_id: i32| -> i32 {
            ctx.nozzle_group_result.get_nozzle_from_id(last_nozzle_id).map_or(0, |n| n.extruder_id)
        };
        // :6708-6715
        let get_filament_nozzle_temp = |idx: i32, nozzle_id: i32| -> i32 {
            let row = self.filament_variant_row(idx, nozzle_id);
            if row >= 0 { FilamentVariantRows::value_at(&rows.nozzle_temp, row) } else { 0 }
        };
        let get_filament_pre_cooling_temp = |idx: i32, nozzle_id: i32| -> i32 {
            let row = self.filament_variant_row(idx, nozzle_id);
            if row >= 0 { FilamentVariantRows::value_at(&rows.pre_cooling_temp, row) } else { 0 }
        };
        // :6717-6722
        let is_pre_cooling_valid = |idx: i32, nozzle_id: i32| -> bool {
            if idx < 0 {
                return false;
            }
            let pre_cooling_temp = get_filament_pre_cooling_temp(idx, nozzle_id);
            pre_cooling_temp > 0 && pre_cooling_temp < get_filament_nozzle_temp(idx, nozzle_id)
        };
        // :6724-6729
        let get_partial_free_cooling_thres = |idx: i32, nozzle_id: i32| -> f32 {
            if idx < 0 {
                return 30.0;
            }
            let temp_in_tower = get_filament_nozzle_temp(idx, nozzle_id) as f32;
            temp_in_tower - get_filament_pre_cooling_temp(idx, nozzle_id) as f32
        };
        // :6730-6734
        let get_min_temperature_when_ec = |idx: i32| -> f32 {
            if idx < 0 || idx >= ctx.filament_min_temperature_when_ec.len() as i32 {
                return -1.0;
            }
            ctx.filament_min_temperature_when_ec[idx as usize] as f32
        };
        // :6736-6738 / std::lower_bound(moves, gcode_id, a.gcode_id < gcode_id)
        let lower_bound_gcode = |gcode_id: u32| -> usize { moves.partition_point(|a| a.gcode_id < gcode_id) };
        // :6740-6751
        let skippable_blocks = self.skippable_blocks;
        let find_skip_block = |gcode_id: u32| -> Option<(u32, u32)> {
            let it = skippable_blocks.partition_point(|block| !(gcode_id < block.0));
            if it != 0 {
                let candidate = skippable_blocks[it - 1];
                if gcode_id >= candidate.0 && gcode_id <= candidate.1 {
                    return Some(candidate);
                }
            }
            None
        };
        // :6753-6763
        let adjust_insert_gcode_id_backward = |mut gcode_id: u32| -> u32 {
            while gcode_id > 0 {
                let Some(skip_block) = find_skip_block(gcode_id) else {
                    return gcode_id;
                };
                if skip_block.0 == 0 {
                    return 0;
                }
                gcode_id = skip_block.0 - 1;
            }
            0
        };
        // :6764-6774
        let adjust_insert_gcode_id_forward = |mut gcode_id: u32| -> u32 {
            while gcode_id < u32::MAX {
                let Some(skip_block) = find_skip_block(gcode_id) else {
                    return gcode_id;
                };
                if skip_block.1 == u32::MAX {
                    return 0;
                }
                gcode_id = skip_block.1 + 1;
            }
            0
        };

        // :6776-6777
        if (!pre_cooling && !pre_heating) || block.free_upper_gcode_id <= block.free_lower_gcode_id {
            return;
        }

        // :6780-6787
        let apply_cooling_when_partial_free = is_pre_cooling_valid(block.last_filament_id, block.last_nozzle_id)
            && pre_cooling
            && block.partial_free_upper_id > block.partial_free_lower_id;
        let free_lower_gcode_id =
            if pre_cooling { adjust_insert_gcode_id_forward(block.free_lower_gcode_id) } else { block.free_lower_gcode_id };
        if pre_cooling && (free_lower_gcode_id == 0 || free_lower_gcode_id >= block.free_upper_gcode_id) {
            return;
        }

        // :6793-6798
        let mut cooling_insert_gcode_id = free_lower_gcode_id;
        if pre_cooling && ctx.handle_hotend_as_extruder {
            let before_skip_blocks = adjust_insert_gcode_id_backward(block.free_lower_gcode_id);
            if before_skip_blocks != 0 {
                cooling_insert_gcode_id = before_skip_blocks;
            }
        }

        // :6800-6810
        let move_iter_lower = lower_bound_gcode(free_lower_gcode_id);
        let mut move_iter_upper = lower_bound_gcode(block.free_upper_gcode_id); // closed iter
        if move_iter_lower == moves.len() || move_iter_upper == 0 {
            return;
        }
        move_iter_upper -= 1;
        let complete_free_time_gap: f32 = if move_iter_lower == 0 {
            moves[move_iter_upper].time[vm]
        } else {
            moves[move_iter_upper].time[vm] - moves[move_iter_lower - 1].time[vm]
        };

        // :6813-6822
        let partial_free_move_lower = lower_bound_gcode(block.partial_free_lower_id);
        let mut partial_free_move_upper = lower_bound_gcode(block.partial_free_upper_id); // closed iter
        if partial_free_move_lower == moves.len() || partial_free_move_upper == 0 {
            return;
        }
        partial_free_move_upper -= 1;
        let partial_free_time_gap: f32 = if partial_free_move_lower == 0 {
            moves[partial_free_move_upper].time[vm]
        } else {
            moves[partial_free_move_upper].time[vm] - moves[partial_free_move_lower - 1].time[vm]
        };

        // :6824-6831
        if move_iter_lower >= move_iter_upper {
            return;
        }
        if apply_cooling_when_partial_free && partial_free_time_gap + complete_free_time_gap < ctx.inject_time_threshold {
            return;
        }
        if !apply_cooling_when_partial_free && complete_free_time_gap < ctx.inject_time_threshold {
            return;
        }

        // :6833-6838
        let extruder_id = get_valid_extruder_id(block.last_nozzle_id);
        let ext_heating_rate = ctx.heating_rate[extruder_id as usize] as f32;
        let ext_cooling_rate_post_extrusion = ctx.cooling_rate[extruder_id as usize] as f32;
        let ext_cooling_rate_when_free = ctx.cooling_rate_on_rack[extruder_id as usize] as f32;

        // :6840-6885 add_M104_lines / format_line_M104
        let add_m104_lines = |inserted_operation_lines: &mut InsertedLinesMap,
                              gcode_id: u32,
                                  target_extruder: i32,
                                  target_temp: i32,
                                  target_filament: i32,
                                  skippable: bool,
                                  next_filament_idx: i32,
                                  next_nozzle_id: i32,
                                  is_heating: bool,
                                  r#type: InsertLineType,
                                  comment: &str| {
            let mut buffer: Vec<String> = Vec::new();
            if skippable {
                let support_dynamic_nozzle_map = ctx.nozzle_group_result.is_support_dynamic_nozzle_map();
                let mut m632_line = format!("M632 S{}", next_filament_idx);
                if support_dynamic_nozzle_map {
                    m632_line += &format!(" H{}", next_nozzle_id);
                }
                if ctx.extruder_max_nozzle_count[target_extruder as usize] > 1 {
                    m632_line += " N R";
                }
                m632_line += " W\n";
                buffer.push(m632_line);
            }
            if !is_heating {
                buffer.push("M400\n".to_string());
            }
            let mut m104_line = String::from("M104");
            if ctx.handle_hotend_as_extruder {
                m104_line += &format!(" I{}", if target_filament == -1 { next_filament_idx } else { target_filament });
            } else if target_extruder != -1 {
                m104_line += &format!(" T{}", ctx.physical_extruder_map[target_extruder as usize]);
            }
            m104_line += &format!(" S{}", target_temp);
            m104_line += " N0"; // N0 means the gcode is generated by slicer
            m104_line += if is_heating { " W" } else { " C" };
            if !comment.is_empty() {
                m104_line += " ;";
                m104_line += comment;
            }
            m104_line.push('\n');
            buffer.push(m104_line);
            if skippable {
                buffer.push("M633\n".to_string());
            }
            let entry = inserted_operation_lines.entry(gcode_id).or_default();
            for line in buffer {
                entry.push((line, r#type));
            }
        };

        // :6887-6896 add_M106_full_part_fan — written into the same map after add_M104_lines.
        // :6898-6907
        let room_temperature: f32 = 25.0;
        let get_filament_min_temperature_when_ec = |filament_id: i32, nozzle_id: i32| -> f32 {
            let min_temperature: f32 = if ctx.handle_hotend_as_extruder { 0.0 } else { room_temperature };
            if self.filament_variant_row(filament_id, nozzle_id) < 0 {
                return min_temperature;
            }
            let idle_floor = get_min_temperature_when_ec(filament_id);
            if idle_floor >= 0.0 {
                return min_temperature.max(idle_floor);
            }
            min_temperature
        };

        // :6909-6929
        if apply_cooling_when_partial_free {
            let min_temperature = get_filament_min_temperature_when_ec(block.last_filament_id, block.last_nozzle_id);
            let mut cooling_budget = get_partial_free_cooling_thres(block.last_filament_id, block.last_nozzle_id);
            let command_only_active = ctx.handle_hotend_as_extruder
                && block.last_filament_id >= 0
                && block.last_filament_id < ctx.filament_types.len() as i32
                && ctx.filament_types[block.last_filament_id as usize] == "PVA";
            if !command_only_active {
                cooling_budget = cooling_budget.min(partial_free_time_gap * ext_cooling_rate_post_extrusion);
            }
            let max_cooling_temp = curr_temp.min(cooling_budget);
            curr_temp = min_temperature.max(curr_temp - max_cooling_temp);
            add_m104_lines(
                inserted_operation_lines,
                block.partial_free_lower_id,
                extruder_id,
                curr_temp as i32,
                block.last_filament_id,
                false,
                block.next_filament_id,
                block.next_nozzle_id,
                false,
                InsertLineType::PreCooling,
                "Multi extruder pre cooling in post extrusion",
            );
            if command_only_active {
                inserted_operation_lines
                    .entry(block.partial_free_lower_id)
                    .or_default()
                    .push(("M106 S255 ;Multi extruder pre cooling fan on\n".to_string(), InsertLineType::PreCooling));
            }
        }

        // :6931-6945
        if pre_cooling && !pre_heating {
            let temp_floor = get_filament_min_temperature_when_ec(block.last_filament_id, block.last_nozzle_id) as i32;
            let mut cooling_temp = temp_floor;
            if self.filament_variant_row(block.next_filament_id, block.next_nozzle_id) >= 0 {
                let delta_temp = (complete_free_time_gap * ext_cooling_rate_when_free) as i32;
                cooling_temp = temp_floor.max(curr_temp as i32 - delta_temp);
            }
            if cooling_temp >= curr_temp as i32 {
                return;
            }
            add_m104_lines(
                inserted_operation_lines,
                cooling_insert_gcode_id,
                extruder_id,
                cooling_temp,
                block.last_filament_id,
                false,
                block.next_filament_id,
                block.next_nozzle_id,
                false,
                InsertLineType::PreCooling,
                "Multi extruder pre cooling",
            );
            return;
        }
        // :6946-6966 std::upper_bound(first, last, t, time < a.time[vm])
        let upper_bound_time = |first: usize, last: usize, t: f32| -> usize {
            first + moves[first..last].partition_point(|a| !(t < a.time[vm]))
        };
        if !pre_cooling && pre_heating {
            if target_temp <= curr_temp {
                return;
            }
            let heating_start_time = moves[move_iter_upper].time[vm] - (target_temp - curr_temp) / ext_heating_rate;
            let mut heating_move_iter = upper_bound_time(move_iter_lower, move_iter_upper + 1, heating_start_time);
            if heating_move_iter == move_iter_lower {
                let heating_gcode_id = adjust_insert_gcode_id_backward(block.free_lower_gcode_id);
                if heating_gcode_id == 0 || heating_gcode_id < block.free_lower_gcode_id {
                    return;
                }
                add_m104_lines(
                    inserted_operation_lines,
                    heating_gcode_id,
                    extruder_id,
                    target_temp as i32,
                    block.next_filament_id,
                    true,
                    block.next_filament_id,
                    block.next_nozzle_id,
                    true,
                    InsertLineType::PreHeating,
                    "Multi extruder pre heating",
                );
            } else {
                heating_move_iter -= 1;
                let heating_gcode_id = adjust_insert_gcode_id_backward(moves[heating_move_iter].gcode_id);
                if heating_gcode_id == 0 || heating_gcode_id < block.free_lower_gcode_id {
                    return;
                }
                add_m104_lines(
                    inserted_operation_lines,
                    heating_gcode_id,
                    extruder_id,
                    target_temp as i32,
                    block.next_filament_id,
                    true,
                    block.next_filament_id,
                    block.next_nozzle_id,
                    true,
                    InsertLineType::PreHeating,
                    "Multi extruder pre heating",
                );
            }
            return;
        }
        // :6967-6996 perform cooling first and then perform heating
        let cooling_thres = get_filament_min_temperature_when_ec(block.last_filament_id, block.last_nozzle_id);
        // :6970 — one expression: clang fuses `curr*hr + target*cr` (the left product into the add)
        // and the subtraction of `(gap*cr)*hr`.
        let mid_temp = {
            let t1 = target_temp * ext_cooling_rate_when_free;
            let s = curr_temp.mul_add(ext_heating_rate, t1);
            let g = complete_free_time_gap * ext_cooling_rate_when_free;
            let num = (-g).mul_add(ext_heating_rate, s);
            cooling_thres.max(num / (ext_cooling_rate_when_free + ext_heating_rate))
        };
        let heating_temp = target_temp - mid_temp;
        let heating_start_time = moves[move_iter_upper].time[vm] - heating_temp / ext_heating_rate;
        let mut heating_move_iter = upper_bound_time(move_iter_lower, move_iter_upper + 1, heating_start_time);
        if heating_move_iter == move_iter_lower {
            return;
        }
        heating_move_iter -= 1;
        let heating_gcode_id = adjust_insert_gcode_id_backward(moves[heating_move_iter].gcode_id);
        if heating_gcode_id <= free_lower_gcode_id {
            return;
        }
        // :6981-6988 std::upper_bound(lower, upper + 1, heating_gcode_id, gcode_id < move.gcode_id)
        let adjusted_heating_move_iter =
            move_iter_lower + moves[move_iter_lower..move_iter_upper + 1].partition_point(|m| !(heating_gcode_id < m.gcode_id));
        if adjusted_heating_move_iter == move_iter_lower {
            return;
        }
        heating_move_iter = adjusted_heating_move_iter - 1;

        // :6990-6995
        let real_cooling_time = moves[heating_move_iter].time[vm] - moves[move_iter_lower].time[vm];
        let real_delta_temp = (real_cooling_time * ext_cooling_rate_when_free) as i32;
        let cooling_temp = (cooling_thres as i32).max(curr_temp as i32 - real_delta_temp);
        add_m104_lines(
            inserted_operation_lines,
            cooling_insert_gcode_id,
            extruder_id,
            cooling_temp,
            block.last_filament_id,
            false,
            block.next_filament_id,
            block.next_nozzle_id,
            false,
            InsertLineType::PreCooling,
            "Multi extruder pre cooling",
        );
        add_m104_lines(
            inserted_operation_lines,
            heating_gcode_id,
            extruder_id,
            target_temp as i32,
            block.next_filament_id,
            true,
            block.next_filament_id,
            block.next_nozzle_id,
            true,
            InsertLineType::PreHeating,
            "Multi extruder pre heating",
        );
    }

    /// GCodeProcessor.cpp:6998-7048 `build_by_filament_blocks(filament_usage_blocks)`
    pub fn build_by_filament_blocks(&mut self, filament_usage_blocks: &[FilamentUsageBlock]) {
        self.m_extruder_free_blocks.clear();
        let handle_hotend_as_extruder = self.ctx.handle_hotend_as_extruder;
        let mut per_group_usage_blocks: std::collections::BTreeMap<i32, Vec<FilamentUsageBlock>> =
            std::collections::BTreeMap::new();
        for block in filament_usage_blocks {
            let group_key = if handle_hotend_as_extruder { block.filament_id } else { block.extruder_id };
            per_group_usage_blocks.entry(group_key).or_default().push(*block);
        }
        let start_filament_block = FilamentUsageBlock::new(-1, -1, -1, 0, self.machine_start_gcode_end_id);
        let end_filament_block = FilamentUsageBlock::new(-1, -1, -1, self.machine_end_gcode_start_id, u32::MAX);
        for blocks in per_group_usage_blocks.values_mut() {
            blocks.insert(0, start_filament_block);
            blocks.push(end_filament_block);
        }
        for (&group_key, filament_blocks) in per_group_usage_blocks.iter() {
            for w in filament_blocks.windows(2) {
                let (iter, niter) = (&w[0], &w[1]);
                let mut block = ExtruderFreeBlock {
                    free_lower_gcode_id: iter.upper_gcode_id,
                    last_filament_id: iter.filament_id,
                    last_nozzle_id: iter.nozzle_id,
                    free_upper_gcode_id: niter.lower_gcode_id,
                    next_filament_id: niter.filament_id,
                    next_nozzle_id: niter.nozzle_id,
                    ..Default::default()
                };
                if block.last_nozzle_id == -1 {
                    block.last_nozzle_id = block.next_nozzle_id;
                }
                block.extruder_id = group_key;
                block.idle_chain_key = group_key;
                block.partial_free_lower_id = block.free_lower_gcode_id;
                block.partial_free_upper_id = block.free_lower_gcode_id;
                self.m_extruder_free_blocks.push(block);
            }
        }
        for block in self.m_extruder_free_blocks.iter_mut() {
            block.ignore_cooling_before_tower = true;
        }
        self.sort_free_blocks();
    }

    /// GCodeProcessor.cpp:7050-7160 `build_by_extruder_blocks(filament_usage_blocks, extruder_usage_blocks)`
    pub fn build_by_extruder_blocks(
        &mut self,
        filament_usage_blocks: &[FilamentUsageBlock],
        extruder_usage_blocks: &[ExtruderUsageBlcok],
    ) {
        self.m_extruder_free_blocks.clear();
        // :7061-7108
        if self.ctx.handle_hotend_as_extruder {
            let mut per_filament_usage_blocks: std::collections::BTreeMap<i32, Vec<FilamentUsageBlock>> =
                std::collections::BTreeMap::new();
            for filament_span in filament_usage_blocks {
                per_filament_usage_blocks.entry(filament_span.filament_id).or_default().push(*filament_span);
            }
            let start_filament_block = FilamentUsageBlock::new(-1, -1, -1, 0, self.machine_start_gcode_end_id);
            let end_filament_block = FilamentUsageBlock::new(-1, -1, -1, self.machine_end_gcode_start_id, u32::MAX);
            for spans in per_filament_usage_blocks.values_mut() {
                spans.insert(0, start_filament_block);
                spans.push(end_filament_block);
            }
            for (&filament_id, filament_spans) in per_filament_usage_blocks.iter() {
                for w in filament_spans.windows(2) {
                    let (prev, next) = (&w[0], &w[1]);
                    let mut block = ExtruderFreeBlock {
                        free_lower_gcode_id: prev.upper_gcode_id,
                        last_filament_id: prev.filament_id,
                        last_nozzle_id: prev.nozzle_id,
                        free_upper_gcode_id: next.lower_gcode_id,
                        next_filament_id: next.filament_id,
                        next_nozzle_id: next.nozzle_id,
                        ..Default::default()
                    };
                    if block.last_nozzle_id == -1 {
                        block.last_nozzle_id = block.next_nozzle_id;
                    }
                    block.idle_chain_key = filament_id;
                    block.extruder_id =
                        self.ctx.nozzle_group_result.get_nozzle_from_id(block.last_nozzle_id).map_or(0, |n| n.extruder_id);
                    block.partial_free_lower_id = block.free_lower_gcode_id;
                    block.partial_free_upper_id = block.free_lower_gcode_id;
                    block.ignore_cooling_before_tower = true;
                    self.attach_nc_bounds_if_gap_crosses_nozzle_change(&mut block, prev, next, extruder_usage_blocks);
                    self.m_extruder_free_blocks.push(block);
                }
            }
            self.sort_free_blocks();
            return;
        }

        // :7110-7129
        let mut per_extruder_usage_blocks: std::collections::BTreeMap<i32, Vec<ExtruderUsageBlcok>> =
            std::collections::BTreeMap::new();
        for block in extruder_usage_blocks {
            per_extruder_usage_blocks.entry(block.extruder_id).or_default().push(*block);
        }
        for (&extruder_id, blocks) in per_extruder_usage_blocks.iter_mut() {
            let mut start_filament_block = ExtruderUsageBlcok::default();
            start_filament_block.initialize_step_1(extruder_id, 0, -1, -1);
            start_filament_block.initialize_step_2(self.machine_start_gcode_end_id as i32);
            start_filament_block.initialize_step_3(
                self.machine_start_gcode_end_id as i32,
                -1,
                self.machine_start_gcode_end_id as i32,
                -1,
            );
            let mut end_filament_block = ExtruderUsageBlcok::default();
            end_filament_block.initialize_step_1(extruder_id, self.machine_end_gcode_start_id as i32, -1, -1);
            end_filament_block.initialize_step_2(i32::MAX);
            end_filament_block.initialize_step_3(i32::MAX, -1, i32::MAX, -1);
            blocks.insert(0, start_filament_block);
            blocks.push(end_filament_block);
        }
        // :7132-7155
        for (&extruder_id, blocks) in per_extruder_usage_blocks.iter() {
            for w in blocks.windows(2) {
                let (iter, niter) = (&w[0], &w[1]);
                let mut block = ExtruderFreeBlock {
                    free_lower_gcode_id: iter.end_id,
                    last_filament_id: iter.end_filament,
                    last_nozzle_id: iter.end_nozzle_id,
                    free_upper_gcode_id: niter.start_id,
                    next_filament_id: niter.start_filament,
                    next_nozzle_id: niter.start_nozzle_id,
                    ..Default::default()
                };
                if block.last_nozzle_id == -1 {
                    block.last_nozzle_id = block.next_nozzle_id;
                }
                block.extruder_id = extruder_id;
                block.idle_chain_key = extruder_id;
                block.partial_free_lower_id = iter.post_extrusion_start_id;
                block.partial_free_upper_id = iter.post_extrusion_end_id;
                block.ignore_cooling_before_tower = niter.ignore_cooling_before_tower;
                self.m_extruder_free_blocks.push(block);
            }
        }
        self.sort_free_blocks();
    }

    /// GCodeProcessor.cpp:7162-7202 `attach_nc_bounds_if_gap_crosses_nozzle_change(idle, prev, next, nc_blocks)`
    pub fn attach_nc_bounds_if_gap_crosses_nozzle_change(
        &self,
        idle: &mut ExtruderFreeBlock,
        prev: &FilamentUsageBlock,
        next: &FilamentUsageBlock,
        nc_blocks: &[ExtruderUsageBlcok],
    ) {
        let overlaps = |nc: &ExtruderUsageBlcok, fil: &FilamentUsageBlock| -> bool {
            fil.filament_id >= 0 && nc.start_id < fil.upper_gcode_id && nc.end_id > fil.lower_gcode_id
        };
        for nc in nc_blocks {
            if nc.end_filament != prev.filament_id || nc.end_id >= self.machine_end_gcode_start_id || !overlaps(nc, prev) {
                continue;
            }
            if nc.post_extrusion_start_id < nc.post_extrusion_end_id {
                idle.partial_free_lower_id = nc.post_extrusion_start_id;
                idle.partial_free_upper_id = nc.post_extrusion_end_id;
            }
            if nc.end_id < idle.free_upper_gcode_id {
                idle.free_lower_gcode_id = nc.end_id;
            }
            if nc.end_nozzle_id >= 0 {
                idle.last_nozzle_id = nc.end_nozzle_id;
            }
            break;
        }
        for nc in nc_blocks {
            if nc.start_filament != next.filament_id || nc.start_id <= self.machine_start_gcode_end_id {
                continue;
            }
            if nc.start_id < prev.upper_gcode_id || nc.start_id >= next.lower_gcode_id || nc.end_id < next.lower_gcode_id {
                continue;
            }
            idle.ignore_cooling_before_tower = nc.ignore_cooling_before_tower;
            if nc.start_nozzle_id >= 0 {
                idle.next_nozzle_id = nc.start_nozzle_id;
            }
            break;
        }
    }
}

/// `std::istringstream >> int` over `s`: leading whitespace, an optional sign and digits; `None`
/// when no digit follows (the stream's fail state). Returns the value and the rest of the string.
fn istream_int(s: &str) -> Option<(i32, &str)> {
    let t = s.trim_start_matches([' ', '\t', '\n', '\r', '\x0b', '\x0c']);
    let b = t.as_bytes();
    let mut k = 0;
    if k < b.len() && (b[k] == b'+' || b[k] == b'-') {
        k += 1;
    }
    let digits_start = k;
    while k < b.len() && b[k].is_ascii_digit() {
        k += 1;
    }
    if k == digits_start {
        return None;
    }
    let v: i64 = t[..k].parse().ok()?;
    if v < i32::MIN as i64 || v > i32::MAX as i64 {
        return None;
    }
    Some((v as i32, &t[k..]))
}

/// GCodeProcessor.cpp:1033-1043 / 1050-1060 / 1069-1079: after the filament id, `while (str >> param)`
/// reads characters (skipping whitespace) until an `H`, then `str >> nozzle_id`; -1 when absent or
/// unreadable.
fn istream_nozzle_after(rest: &str) -> i32 {
    let mut chars = rest.char_indices().filter(|(_, c)| !c.is_whitespace());
    if let Some((i, _)) = chars.find(|(_, c)| *c == 'H') {
        return istream_int(&rest[i + 1..]).map_or(-1, |(v, _)| v);
    }
    -1
}

/// GCodeProcessor.cpp:904-922 `handle_nozzle_change_line`: the regex
/// `OF(\d+)\s+NF(\d+)\s+ON(\d+)\s+NN(\d+)` searched in the line.
fn nozzle_change_fields(line: &str) -> Option<(i32, i32, i32, i32)> {
    let b = line.as_bytes();
    let digits = |mut k: usize| -> Option<(i32, usize)> {
        let s = k;
        while k < b.len() && b[k].is_ascii_digit() {
            k += 1;
        }
        if k == s {
            return None;
        }
        line[s..k].parse::<i32>().ok().map(|v| (v, k))
    };
    let ws = |mut k: usize| -> Option<usize> {
        let s = k;
        while k < b.len() && (b[k] as char).is_ascii_whitespace() {
            k += 1;
        }
        if k == s { None } else { Some(k) }
    };
    let lit = |k: usize, l: &str| -> Option<usize> { if line[k..].starts_with(l) { Some(k + l.len()) } else { None } };
    for start in 0..b.len() {
        let r = (|| {
            let k = lit(start, "OF")?;
            let (of, k) = digits(k)?;
            let k = ws(k)?;
            let k = lit(k, "NF")?;
            let (nf, k) = digits(k)?;
            let k = ws(k)?;
            let k = lit(k, "ON")?;
            let (on, k) = digits(k)?;
            let k = ws(k)?;
            let k = lit(k, "NN")?;
            let (nn, _) = digits(k)?;
            Some((of, nf, on, nn))
        })();
        if r.is_some() {
            return r;
        }
    }
    None
}

/// GCodeProcessor.cpp:923-932 `handle_toolchange_wipe_line`: the regex `CT(\d)(?:\s+FL(\d))?`.
fn toolchange_wipe_fields(line: &str) -> Option<(bool, bool)> {
    let b = line.as_bytes();
    for start in 0..b.len().saturating_sub(2) {
        if b[start] == b'C' && b[start + 1] == b'T' && b[start + 2].is_ascii_digit() {
            let is_contact = b[start + 2] != b'0';
            let mut k = start + 3;
            let s = k;
            while k < b.len() && (b[k] as char).is_ascii_whitespace() {
                k += 1;
            }
            let mut is_first_layer = false;
            if k > s && k + 2 < b.len() + 0 && b[k] == b'F' && b[k + 1] == b'L' && b[k + 2].is_ascii_digit() {
                is_first_layer = b[k + 2] != b'0';
            }
            return Some((is_contact, is_first_layer));
        }
    }
    None
}

impl TimeProcessor {
    /// GCodeProcessor.cpp:571-1300 `TimeProcessor::post_process(filename, moves, lines_ends, context)`
    /// with the pre-heating pass: the first pass (M73 lines, :1012-1124) and its block collection, the
    /// offsets (:1307-1365, applied by reading the ids off the first pass's output, which is what
    /// `handle_offsets_of_first_process` computes), the moves' time fill (:1129-1143), the remaining
    /// filament changes (:1150-1169, written when `write_filament_change_predict`), the extruder blocks'
    /// initialization (:1175-1199), the PreCoolingInjector (:1204-1240) and the second pass (:1242-1281).
    /// `moves` are the processor's moves (`gcode_id`, `time`) in the first pass's input numbering.
    pub fn post_process_with_context(
        &self,
        gcode: &str,
        moves_in: &[MoveVertexTime],
        context: &TimeProcessContext,
        write_filament_change_predict: bool,
    ) -> String {
        use crate::g_code_reader::GCodeLine;
        // first pass: the M73 lines and their offsets
        let mut offsets: Vec<(u32, u32)> = Vec::new();
        let text1 = self.post_process_first_pass(gcode, Some(&mut offsets));

        // :1318-1327 moves' gcode ids moved by the inserted lines
        let mut moves: Vec<MoveVertexTime> = moves_in.to_vec();
        {
            let mut curr_offset_id = 0usize;
            let mut total_offset = 0u32;
            for mv in moves.iter_mut() {
                while curr_offset_id < offsets.len() && offsets[curr_offset_id].0 <= mv.gcode_id {
                    total_offset += offsets[curr_offset_id].1;
                    curr_offset_id += 1;
                }
                mv.gcode_id += total_offset;
            }
        }

        // The block collection of `gcode_time_handler` (:1001-1121), over the first pass's output: an
        // input line L sits at L + (lines inserted at or before L) there, which is the id
        // `handle_offsets_of_first_process` gives every block (:1342-1364).
        let reserved = |t: ETags| -> String { format!(";{}", reserved_tag(t)) };
        let tag_machine_start_end = reserved_tag(ETags::MachineStartGCodeEnd);
        let tag_machine_end_start = reserved_tag(ETags::MachineEndGCodeStart);
        let tag_skippable_start = custom_tags(CustomETags::SkippableStart);
        let tag_skippable_end = custom_tags(CustomETags::SkippableEnd);
        // GCodeProcessor.cpp:82-83 CustomTags FilamentChangeGCodeStart / End
        let tag_fc_start = " FilamentChangeGCodeStart";
        let tag_fc_end = " FilamentChangeGCodeEnd";
        let nozzle_change_start = reserved(ETags::NozzleChangeStart);
        let nozzle_change_end = reserved(ETags::NozzleChangeEnd);
        let cp_toolchange_wipe = reserved(ETags::CpToolchangeWipe);
        let layer_change = reserved(ETags::LayerChange);
        let ngr = &context.nozzle_group_result;

        let mut machine_start_gcode_end_line_id: u32 = u32::MAX;
        let mut machine_end_gcode_start_line_id: u32 = u32::MAX;
        let mut skippable_blocks: Vec<(u32, u32)> = Vec::new();
        let mut current_layer_id: i32 = 0;
        let mut filament_blocks: Vec<FilamentUsageBlock> = Vec::new();
        let mut extruder_blocks: Vec<ExtruderUsageBlcok> = vec![ExtruderUsageBlcok::default()];
        let mut temp_construct_block = ExtruderUsageBlcok::default();

        let mut line_id: u32 = 0;
        for gcode_line in text1.split_inclusive('\n') {
            line_id += 1;
            // :675-787 process_placeholders: the tags it records (the placeholders themselves are
            // already replaced in the port's header).
            let line = gcode_line.strip_suffix('\n').unwrap_or(gcode_line);
            if line.len() > 1 {
                let l = &line[1..];
                if l == tag_machine_start_end {
                    machine_start_gcode_end_line_id = line_id;
                } else if l == tag_machine_end_start {
                    machine_end_gcode_start_line_id = line_id;
                } else if l == tag_skippable_start {
                    skippable_blocks.push((line_id, 0));
                } else if l == tag_skippable_end {
                    if let Some(b) = skippable_blocks.last_mut() {
                        b.1 = line_id;
                    }
                } else if context.handle_hotend_as_extruder && l == tag_fc_start {
                    skippable_blocks.push((line_id, 0));
                } else if context.handle_hotend_as_extruder && l == tag_fc_end && !skippable_blocks.is_empty() {
                    skippable_blocks.last_mut().unwrap().1 = line_id;
                }
            }
            // :957-975 handle_filament_change
            let mut handle_filament_change = |filament_id: i32, line_id: u32, mut nozzle_id: i32| {
                if machine_start_gcode_end_line_id == u32::MAX && line_id < machine_start_gcode_end_line_id
                    || machine_end_gcode_start_line_id != u32::MAX && line_id > machine_end_gcode_start_line_id
                {
                    return;
                }
                if let Some(b) = filament_blocks.last_mut() {
                    b.upper_gcode_id = line_id;
                }
                if nozzle_id == -1 {
                    nozzle_id = ngr.get_nozzle_id(filament_id, current_layer_id);
                }
                // :972 `get_nozzle_from_id(nozzle_id)->extruder_id` (dereferenced unchecked in C++).
                let extruder_id = ngr.get_nozzle_from_id(nozzle_id).map_or(0, |n| n.extruder_id);
                filament_blocks.push(FilamentUsageBlock::new(filament_id, extruder_id, nozzle_id, line_id, u32::MAX));
            };
            // :983-1121 (the G1/G2/G3/;VG1 branch wrote the M73 lines in the first pass)
            if GCodeLine::cmd_is_str(gcode_line, "G1")
                || GCodeLine::cmd_is_str(gcode_line, "G2")
                || GCodeLine::cmd_is_str(gcode_line, "G3")
                || GCodeLine::cmd_start_with(gcode_line, ";VG1")
            {
            } else if GCodeLine::cmd_start_with(gcode_line, "T") {
                let skips = gcode_line.len() - gcode_line.trim_start_matches([' ', '\t']).len();
                if let Some((fid, rest)) = istream_int(&gcode_line[skips + 1..]) {
                    if (0..255).contains(&fid) {
                        handle_filament_change(fid, line_id, istream_nozzle_after(rest));
                    }
                }
            } else if GCodeLine::cmd_start_with(gcode_line, ";VT") {
                let skips = gcode_line.len() - gcode_line.trim_start_matches([' ', '\t']).len();
                if let Some((fid, rest)) = istream_int(&gcode_line[skips + 3..]) {
                    if (0..255).contains(&fid) {
                        handle_filament_change(fid, line_id, istream_nozzle_after(rest));
                    }
                }
            } else if GCodeLine::cmd_start_with(gcode_line, "M1020") {
                if let Some(s_pos) = gcode_line.find('S') {
                    if let Some((fid, rest)) = istream_int(&gcode_line[s_pos + 1..]) {
                        if (0..255).contains(&fid) {
                            handle_filament_change(fid, line_id, istream_nozzle_after(rest));
                        }
                    }
                }
            } else if GCodeLine::cmd_start_with(gcode_line, &nozzle_change_start) {
                if let Some(b) = extruder_blocks.last_mut() {
                    b.initialize_step_2(line_id as i32);
                }
            } else if GCodeLine::cmd_start_with(gcode_line, &nozzle_change_end) {
                let (prev_filament, next_filament, prev_nozzle_id, next_nozzle_id) =
                    nozzle_change_fields(gcode_line).unwrap_or((-1, -1, -1, -1));
                // :915-919 (not handle_hotend_as_extruder: the nozzle's extruder; else its group)
                let extruder_id = if nozzle_change_fields(gcode_line).is_some() {
                    let info = ngr.get_nozzle_from_id(next_nozzle_id);
                    if context.handle_hotend_as_extruder {
                        info.map_or(-1, |n| n.group_id)
                    } else {
                        info.map_or(-1, |n| n.extruder_id)
                    }
                } else {
                    -1
                };
                if let Some(b) = extruder_blocks.last_mut() {
                    b.initialize_step_3(line_id as i32, prev_filament, line_id as i32, prev_nozzle_id);
                }
                temp_construct_block.initialize_step_1(extruder_id, line_id as i32, next_filament, next_nozzle_id);
                extruder_blocks.push(temp_construct_block);
                temp_construct_block.reset();
            } else if GCodeLine::cmd_start_with(gcode_line, &cp_toolchange_wipe) {
                let (is_contact, is_first_layer) = toolchange_wipe_fields(gcode_line).unwrap_or((false, false));
                if let Some(b) = extruder_blocks.last_mut() {
                    b.ignore_cooling_before_tower = is_contact || is_first_layer;
                }
            } else if GCodeLine::cmd_start_with(gcode_line, &layer_change) {
                current_layer_id += 1;
            }
        }

        // :1129-1143 If not initialized, use the time from the previous move.
        for k in 1..moves.len() {
            if moves[k].time[0] == 0.0 && moves[k].time[1] == 0.0 {
                moves[k].time = moves[k - 1].time;
            }
        }

        let mut inserted_operation_lines: InsertedLinesMap = InsertedLinesMap::new();

        // :1150-1169 the remaining filament changes
        if write_filament_change_predict {
            let mut curr_filament = -1;
            let mut total_filament_count = 0;
            for fb in &filament_blocks {
                if curr_filament != -1 && curr_filament != fb.filament_id {
                    total_filament_count += 1;
                }
                curr_filament = fb.filament_id;
            }
            curr_filament = -1;
            let mut curr_filament_change_num = 0;
            for fb in &filament_blocks {
                if curr_filament != -1 && curr_filament != fb.filament_id {
                    curr_filament_change_num += 1;
                    inserted_operation_lines.entry(fb.lower_gcode_id).or_default().push((
                        format!("M73 E{}\n", total_filament_count - curr_filament_change_num),
                        InsertLineType::FilamentChangePredict,
                    ));
                }
                curr_filament = fb.filament_id;
            }
        }

        // :1171-1173
        if let Some(b) = filament_blocks.last_mut() {
            b.upper_gcode_id = machine_end_gcode_start_line_id;
        }

        // :1175-1199
        if !extruder_blocks.is_empty() {
            let mut first_filament = 0;
            let mut last_filament = 0;
            if !filament_blocks.is_empty() {
                first_filament = filament_blocks[0].filament_id;
                last_filament = filament_blocks.last().unwrap().filament_id;
            }
            {
                let nozzle_info = ngr.get_first_nozzle_for_filament(first_filament);
                let extruder_id = if context.handle_hotend_as_extruder {
                    nozzle_info.as_ref().map_or(-1, |n| n.group_id)
                } else {
                    nozzle_info.as_ref().map_or(-1, |n| n.extruder_id)
                };
                let start_nozzle_id = nozzle_info.as_ref().map_or(-1, |n| n.group_id);
                extruder_blocks[0].initialize_step_1(
                    extruder_id,
                    machine_start_gcode_end_line_id as i32,
                    first_filament,
                    start_nozzle_id,
                );
            }
            extruder_blocks.last_mut().unwrap().initialize_step_2(machine_end_gcode_start_line_id as i32);
            let mut last_nozzle_id = -1;
            if !filament_blocks.is_empty() {
                last_nozzle_id = filament_blocks.last().unwrap().nozzle_id;
            }
            extruder_blocks.last_mut().unwrap().initialize_step_3(
                machine_end_gcode_start_line_id as i32,
                last_filament,
                machine_end_gcode_start_line_id as i32,
                last_nozzle_id,
            );
        }
        // :1201-1202 `extruder_change_info` is filled and not read again.

        // :1204-1240
        if context.enable_pre_heating {
            let mut valid_machine_id = 0usize;
            for i in 0..ETimeMode::COUNT {
                if self.machines[i].enabled {
                    valid_machine_id = i;
                    break;
                }
            }
            let mut pre_cooling_injector = PreCoolingInjector::new(
                &moves,
                context,
                valid_machine_id,
                &skippable_blocks,
                machine_start_gcode_end_line_id,
                machine_end_gcode_start_line_id,
            );
            pre_cooling_injector.build_extruder_free_blocks(&filament_blocks, &extruder_blocks);
            pre_cooling_injector.process_pre_cooling_and_heating(&mut inserted_operation_lines);
        }

        // :1242-1281 the second pass: the lines are appended after the line of their id; the
        // iterator only moves on when a line id equals its key.
        let mut out = String::with_capacity(text1.len() + inserted_operation_lines.len() * 48);
        let mut pre_operation_iter = inserted_operation_lines.iter().peekable();
        let mut line_id: u32 = 0;
        for gcode_line in text1.split_inclusive('\n') {
            line_id += 1;
            out.push_str(gcode_line);
            if let Some((&key, elems)) = pre_operation_iter.peek() {
                if line_id == key {
                    if !gcode_line.ends_with('\n') {
                        out.push('\n');
                    }
                    for (s, t) in elems.iter() {
                        match t {
                            InsertLineType::PreCooling | InsertLineType::PreHeating => {
                                if context.enable_pre_heating {
                                    out.push_str(s);
                                }
                            }
                            InsertLineType::FilamentChangePredict => out.push_str(s),
                            _ => {}
                        }
                    }
                    pre_operation_iter.next();
                }
            }
        }
        out
    }
}

// ===========================================================================
// GCodeProcessor.hpp:1043-1063 class SeamsDetector
// ===========================================================================

/// GCodeProcessor.hpp:1043-1063 `class SeamsDetector`
#[derive(Debug, Clone, Default)]
pub struct SeamsDetector {
    m_active: bool,
    m_first_vertex: Option<[f32; 3]>,
}

impl SeamsDetector {
    /// GCodeProcessor.hpp:1049-1055 `void activate(bool active)`
    pub fn activate(&mut self, active: bool) {
        if self.m_active != active {
            self.m_active = active;
            if self.m_active {
                self.m_first_vertex = None;
            }
        }
    }

    /// GCodeProcessor.hpp:1057 `std::optional<Vec3f> get_first_vertex() const`
    pub fn get_first_vertex(&self) -> Option<[f32; 3]> {
        self.m_first_vertex
    }

    /// GCodeProcessor.hpp:1058 `void set_first_vertex(const Vec3f& vertex)`
    pub fn set_first_vertex(&mut self, vertex: [f32; 3]) {
        self.m_first_vertex = Some(vertex);
    }

    /// GCodeProcessor.hpp:1060 `bool is_active() const`
    pub fn is_active(&self) -> bool {
        self.m_active
    }

    /// GCodeProcessor.hpp:1061 `bool has_first_vertex() const`
    pub fn has_first_vertex(&self) -> bool {
        self.m_first_vertex.is_some()
    }
}

// ===========================================================================
// GCodeProcessor.hpp:1098-1166 struct DataChecker (ENABLE_GCODE_VIEWER_DATA_CHECKING)
// ===========================================================================

/// GCodeProcessor.hpp:1100-1105 `struct DataChecker::Error`
#[derive(Debug, Clone, Copy)]
pub struct DataCheckerError {
    pub value: f32,
    pub tag_value: f32,
    pub role: ExtrusionRole,
}

/// GCodeProcessor.hpp:1098-1165 `struct DataChecker`: the values the G-code tags declare against
/// those the processor computes.
#[derive(Debug, Clone)]
pub struct DataChecker {
    pub r#type: String,
    pub threshold: f32,
    pub last_tag_value: f32,
    pub count: u32,
    pub errors: Vec<DataCheckerError>,
}

impl DataChecker {
    /// GCodeProcessor.hpp:1113-1115 `DataChecker(const std::string& type, float threshold)`
    pub fn new(r#type: &str, threshold: f32) -> Self {
        DataChecker { r#type: r#type.to_string(), threshold, last_tag_value: 0.0, count: 0, errors: Vec::new() }
    }

    /// GCodeProcessor.hpp:1117-1125 `void update(float value, ExtrusionRole role)`
    pub fn update(&mut self, value: f32, role: ExtrusionRole) {
        if role != ExtrusionRole::Custom {
            self.count += 1;
            if self.last_tag_value != 0.0 && (value - self.last_tag_value).abs() / self.last_tag_value > self.threshold {
                self.errors.push(DataCheckerError { value, tag_value: self.last_tag_value, role });
            }
        }
    }

    /// GCodeProcessor.hpp:1127 `void reset()`
    pub fn reset(&mut self) {
        self.last_tag_value = 0.0;
        self.errors.clear();
        self.count = 0;
    }

    /// GCodeProcessor.hpp:1129-1139 `std::pair<float, float> get_min() const`
    pub fn get_min(&self) -> (f32, f32) {
        let mut delta_min = f32::MAX;
        let mut perc_min = 0.0f32;
        for e in &self.errors {
            if delta_min > e.value - e.tag_value {
                delta_min = e.value - e.tag_value;
                perc_min = 100.0 * delta_min / e.tag_value;
            }
        }
        (delta_min, perc_min)
    }

    /// GCodeProcessor.hpp:1141-1151 `std::pair<float, float> get_max() const`
    pub fn get_max(&self) -> (f32, f32) {
        let mut delta_max = -f32::MAX;
        let mut perc_max = 0.0f32;
        for e in &self.errors {
            if delta_max < e.value - e.tag_value {
                delta_max = e.value - e.tag_value;
                perc_max = 100.0 * delta_max / e.tag_value;
            }
        }
        (delta_max, perc_max)
    }

    /// GCodeProcessor.hpp:1153-1161 `void output() const`: the text the C++ writes to `std::cout`.
    pub fn output(&self) -> String {
        let mut out = String::new();
        if !self.errors.is_empty() {
            out += &format!("{}:\n", self.r#type);
            out += &format!(
                "Errors: {} ({}%)\n",
                self.errors.len(),
                crate::locales_utils::general_format((100.0f32 * self.errors.len() as f32 / self.count as f32) as f64, 6)
            );
            let (min, perc_min) = self.get_min();
            let (max, perc_max) = self.get_max();
            out += &format!(
                "min: {}({}%) - max: {}({}%)\n",
                crate::locales_utils::general_format(min as f64, 6),
                crate::locales_utils::general_format(perc_min as f64, 6),
                crate::locales_utils::general_format(max as f64, 6),
                crate::locales_utils::general_format(perc_max as f64, 6)
            );
        }
        out
    }
}

// ===========================================================================
// GCodeProcessor.hpp:1219-1234 enum class EProducer, GCodeProcessor.cpp:1618-1631 Producers
// ===========================================================================

/// GCodeProcessor.hpp:1219-1231 `enum class EProducer`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EProducer {
    Unknown,
    BambuStudio,
    Slic3rPE,
    Slic3r,
    SuperSlicer,
    Cura,
    Simplify3D,
    CraftWare,
    IdeaMaker,
    KissSlicer,
}

/// GCodeProcessor.cpp:1618-1631 `const std::vector<std::pair<GCodeProcessor::EProducer, std::string>>
/// GCodeProcessor::Producers` (`SLIC3R_APP_NAME` is "BambuStudio"; the other producers are commented
/// out at the pin).
pub const PRODUCERS: [(EProducer, &str); 2] =
    [(EProducer::BambuStudio, "BambuStudio"), (EProducer::BambuStudio, "generated by BambuStudio")];

/// GCodeProcessor.cpp:1633 `unsigned int GCodeProcessor::s_result_id = 0;`
static S_RESULT_ID: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// GCodeProcessor.hpp:1336-1355 `static ThermalIndex parse_helioadditive_comment(const std::string
/// comment, bool& is_helio)`: the thermal index of a `;helioadditive=` comment, matched by
/// `\bti\.max=(-?[0-9]*\.?[0-9]+),ti\.min=(-?[0-9]*\.?[0-9]+),ti\.mean=(-?[0-9]*\.?[0-9]+)\b`.
pub fn parse_helioadditive_comment(comment: &str, is_helio: &mut bool) -> ThermalIndex {
    if comment.contains(";helioadditive=") {
        match helio_regex_search(comment.as_bytes()) {
            Some((max_s, min_s, mean_s)) => {
                // std::stof(..) * 100.0: a float times a double, stored as a float.
                let stof = |s: &str| -> f32 { crate::config::strtod_c(s.as_bytes()).value as f32 };
                let max_val = (stof(max_s) as f64 * 100.0) as f32;
                let min_val = (stof(min_s) as f64 * 100.0) as f32;
                let mean_val = (stof(mean_s) as f64 * 100.0) as f32;
                *is_helio = true;
                ThermalIndex::with_values(min_val, max_val, mean_val)
            }
            None => ThermalIndex::new(),
        }
    } else {
        ThermalIndex::new()
    }
}

/// `std::regex_search` (ECMAScript) of the thermal index pattern: the leftmost match, its three
/// groups. `-?[0-9]*\.?[0-9]+` backtracks: the greedy `[0-9]*` gives back digits so that `[0-9]+`
/// matches at least one; the `\b` after the third number needs a word character before the end of
/// the match and a non-word one (or the end) after it.
fn helio_regex_search(s: &[u8]) -> Option<(&str, &str, &str)> {
    let is_word = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
    // `-?[0-9]*\.?[0-9]+` at `i`: every end the pattern can reach, longest first (the order the
    // ECMAScript backtracking tries them).
    let number_ends = |i: usize| -> Vec<usize> {
        let mut out = Vec::new();
        let mut j = i;
        if j < s.len() && s[j] == b'-' {
            j += 1;
        }
        let digits_start = j;
        let mut k = j;
        while k < s.len() && s[k].is_ascii_digit() {
            k += 1;
        }
        // Greedy `[0-9]*` takes k-digits_start digits, then gives back one at a time.
        let mut star = k;
        loop {
            // optional '.'
            let mut after_dot = vec![];
            if star < s.len() && s[star] == b'.' {
                after_dot.push(star + 1);
            }
            after_dot.push(star);
            for &p in &after_dot {
                let mut q = p;
                while q < s.len() && s[q].is_ascii_digit() {
                    q += 1;
                }
                // `[0-9]+` greedy: longest first, at least one digit.
                let mut e = q;
                while e > p {
                    out.push(e);
                    e -= 1;
                }
            }
            if star == digits_start {
                break;
            }
            star -= 1;
        }
        out
    };
    let lit = |i: usize, t: &[u8]| -> Option<usize> {
        if s.len() >= i + t.len() && &s[i..i + t.len()] == t {
            Some(i + t.len())
        } else {
            None
        }
    };
    for start in 0..s.len() {
        // `\b` before "ti": the previous character is not a word character ('t' is one).
        if start > 0 && is_word(s[start - 1]) {
            continue;
        }
        let Some(a) = lit(start, b"ti.max=") else { continue };
        for e1 in number_ends(a) {
            let Some(b) = lit(e1, b",ti.min=") else { continue };
            for e2 in number_ends(b) {
                let Some(c) = lit(e2, b",ti.mean=") else { continue };
                for e3 in number_ends(c) {
                    // `\b`: s[e3-1] is a digit (a word character), so the next one must not be.
                    if e3 < s.len() && is_word(s[e3]) {
                        continue;
                    }
                    let st = |x: usize, y: usize| std::str::from_utf8(&s[x..y]).unwrap_or("");
                    return Some((st(a, e1), st(b, e2), st(c, e3)));
                }
            }
        }
    }
    None
}

impl TimeMachine {
    /// GCodeProcessor.cpp:416-421 `void GCodeProcessor::TimeMachine::handle_time_block(const TimeBlock&
    /// block, float time, int activate_machine_idx, GCodeProcessorResult& result)`
    pub fn handle_time_block(block: &TimeBlock, time: f32, activate_machine_idx: usize, result: &mut GCodeProcessorResult) {
        if block.skippable_type != SkipType::StNone {
            *result.skippable_part_time.entry(block.skippable_type).or_insert(0.0) += block.time();
        }
        result.moves[block.move_id as usize].time[activate_machine_idx] = time;
    }
}

impl GCodeProcessor {
    /// GCodeProcessor.hpp:546 `bool is_helio_gcode()`
    pub fn is_helio_gcode(&self) -> bool {
        self.m_is_helio_gcode
    }

    /// GCodeProcessor.hpp:1304-1306 `bool is_stealth_time_estimator_enabled() const`
    pub fn is_stealth_time_estimator_enabled(&self) -> bool {
        self.m_time_processor.machines[ETimeMode::Stealth as usize].enabled
    }

    /// GCodeProcessor.hpp:1307 `void enable_machine_envelope_processing(bool enabled)`
    pub fn enable_machine_envelope_processing(&mut self, enabled: bool) {
        self.m_time_processor.machine_envelope_processing_enabled = enabled;
    }

    /// GCodeProcessor.hpp:1339-1341 `void detect_layer_based_on_tag(bool enabled)` (Orca: if true,
    /// only change new layer if ETags::Layer_Change occurs).
    pub fn detect_layer_based_on_tag(&mut self, enabled: bool) {
        self.m_detect_layer_based_on_tag = enabled;
    }

    /// GCodeProcessor.cpp:2763-2777 `void GCodeProcessor::initialize(const std::string& filename)`:
    /// the streaming interface's start; the first move is a dummy move.
    pub fn initialize(&mut self, filename: &str) {
        // process gcode
        self.m_result.filename = filename.to_string();
        // m_result.id = ++s_result_id;
        self.m_result.id = S_RESULT_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
        // 1st move must be a dummy move
        self.m_result.moves.push(MoveVertex::default());
    }

    /// GCodeProcessor.cpp:2779-2783 `void GCodeProcessor::initialize_from_context(const
    /// std::shared_ptr<MultiNozzleUtils::NozzleGroupResultBase>& nozzle_group_result)`
    pub fn initialize_from_context(&mut self, nozzle_group_result: Option<crate::multi_nozzle_utils::LayeredNozzleGroupResult>) {
        self.m_nozzle_group_result = nozzle_group_result;
    }

    /// The processor's `m_nozzle_group_result` (the one `GCodeProcessor::finalize(true)` hands to the
    /// time context, GCodeProcessor.cpp:2833) and its moves' `gcode_id` / `time`, for
    /// `TimeProcessor::post_process_with_context`.
    pub fn time_process_inputs(&self) -> (Option<&crate::multi_nozzle_utils::LayeredNozzleGroupResult>, Vec<MoveVertexTime>) {
        (
            self.m_nozzle_group_result.as_ref(),
            self.m_result.moves.iter().map(|m| MoveVertexTime { gcode_id: m.gcode_id, time: m.time }).collect(),
        )
    }

    /// GCodeProcessor.cpp:2903-2916 `std::vector<std::pair<CustomGCode::Type, std::pair<float,
    /// float>>> GCodeProcessor::get_custom_gcode_times(PrintEstimatedStatistics::ETimeMode mode, bool
    /// include_remaining) const`
    pub fn get_custom_gcode_times(&self, mode: ETimeMode, include_remaining: bool) -> Vec<(CustomGCodeType, (f32, f32))> {
        let mut ret: Vec<(CustomGCodeType, (f32, f32))> = Vec::new();
        if (mode as usize) < ETimeMode::COUNT {
            let machine = &self.m_time_processor.machines[mode as usize];
            let mut total_time = 0.0f32;
            for &(r#type, time) in &machine.gcode_time.times {
                let remaining = if include_remaining { machine.time - total_time } else { 0.0 };
                ret.push((r#type, (time, remaining)));
                total_time += time;
            }
        }
        ret
    }

    /// GCodeProcessor.cpp:2918-2929 `std::vector<std::pair<EMoveType, float>>
    /// GCodeProcessor::get_moves_time(PrintEstimatedStatistics::ETimeMode mode) const`
    pub fn get_moves_time(&self, mode: ETimeMode) -> Vec<(EMoveType, f32)> {
        let mut ret: Vec<(EMoveType, f32)> = Vec::new();
        if (mode as usize) < ETimeMode::COUNT {
            let moves_time = &self.m_time_processor.machines[mode as usize].moves_time;
            for i in 0..moves_time.len() {
                let time = moves_time[i];
                if time > 0.0 {
                    ret.push((emove_type_from_index(i), time));
                }
            }
        }
        ret
    }

    /// GCodeProcessor.cpp:2931-2942 `std::vector<std::pair<ExtrusionRole, float>>
    /// GCodeProcessor::get_roles_time(PrintEstimatedStatistics::ETimeMode mode) const`
    pub fn get_roles_time(&self, mode: ETimeMode) -> Vec<(ExtrusionRole, f32)> {
        let mut ret: Vec<(ExtrusionRole, f32)> = Vec::new();
        if (mode as usize) < ETimeMode::COUNT {
            let roles_time = &self.m_time_processor.machines[mode as usize].roles_time;
            for i in 0..roles_time.len() {
                let time = roles_time[i];
                if time > 0.0 {
                    ret.push((extrusion_role_from_index(i), time));
                }
            }
        }
        ret
    }

    /// GCodeProcessor.cpp:2977-2982 `std::vector<float>
    /// GCodeProcessor::get_layers_time(PrintEstimatedStatistics::ETimeMode mode) const`
    pub fn get_layers_time(&self, mode: ETimeMode) -> Vec<f32> {
        if (mode as usize) < ETimeMode::COUNT {
            self.m_time_processor.machines[mode as usize].layers_time.clone()
        } else {
            Vec::new()
        }
    }

    /// GCodeProcessor.cpp:3122-3126 `void GCodeProcessor::process_helioadditive_comment(const
    /// GCodeReader::GCodeLine &line)`: the thermal index of a comment added by the helio processing
    /// server.
    pub fn process_helioadditive_comment(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let comment = line.raw();
        self.m_thermal_index = parse_helioadditive_comment(comment, &mut self.m_is_helio_gcode);
    }

    /// GCodeProcessor.cpp:3643-3658 `bool GCodeProcessor::process_producers_tags(const std::string_view
    /// comment)`
    fn process_producers_tags(&mut self, comment: &str) -> bool {
        match self.m_producer {
            EProducer::Slic3rPE | EProducer::Slic3r | EProducer::SuperSlicer | EProducer::BambuStudio => {
                self.process_bambuslicer_tags(comment)
            }
            EProducer::Cura => self.process_cura_tags(comment),
            EProducer::Simplify3D => self.process_simplify3d_tags(comment),
            EProducer::CraftWare => self.process_craftware_tags(comment),
            EProducer::IdeaMaker => self.process_ideamaker_tags(comment),
            EProducer::KissSlicer => self.process_kissslicer_tags(comment),
            _ => false,
        }
    }

    /// GCodeProcessor.cpp:3660-3663 `bool GCodeProcessor::process_bambuslicer_tags(const
    /// std::string_view comment)`
    fn process_bambuslicer_tags(&mut self, _comment: &str) -> bool {
        false
    }

    /// GCodeProcessor.cpp:3665-3737 `bool GCodeProcessor::process_cura_tags(const std::string_view
    /// comment)`
    fn process_cura_tags(&mut self, comment: &str) -> bool {
        // TYPE -> extrusion role
        let mut tag = "TYPE:";
        if let Some(pos) = comment.find(tag) {
            let r#type = &comment[pos + tag.len()..];
            match r#type {
                "SKIRT" => self.set_extrusion_role(ExtrusionRole::Skirt),
                "WALL-OUTER" => self.set_extrusion_role(ExtrusionRole::ExternalPerimeter),
                "WALL-INNER" => self.set_extrusion_role(ExtrusionRole::Perimeter),
                "SKIN" => self.set_extrusion_role(ExtrusionRole::SolidInfill),
                "FILL" => self.set_extrusion_role(ExtrusionRole::InternalInfill),
                "SUPPORT" => self.set_extrusion_role(ExtrusionRole::SupportMaterial),
                "SUPPORT-INTERFACE" => self.set_extrusion_role(ExtrusionRole::SupportMaterialInterface),
                "PRIME-TOWER" => self.set_extrusion_role(ExtrusionRole::WipeTower),
                _ => {
                    self.set_extrusion_role(ExtrusionRole::None);
                    // BOOST_LOG_TRIVIAL(warning) << "GCodeProcessor found unknown extrusion role: " << type;
                }
            }

            if self.m_extrusion_role == ExtrusionRole::ExternalPerimeter {
                self.m_seams_detector.activate(true);
            }

            return true;
        }

        // flavor
        tag = "FLAVOR:";
        if let Some(pos) = comment.find(tag) {
            let flavor = &comment[pos + tag.len()..];
            match flavor {
                "BFB" => self.m_flavor = GCodeFlavor::MarlinLegacy, // is this correct ?
                "Mach3" => self.m_flavor = GCodeFlavor::Mach3,
                "Makerbot" => self.m_flavor = GCodeFlavor::MakerWare,
                "UltiGCode" => self.m_flavor = GCodeFlavor::MarlinLegacy, // is this correct ?
                "Marlin(Volumetric)" => self.m_flavor = GCodeFlavor::MarlinLegacy, // is this correct ?
                "Griffin" => self.m_flavor = GCodeFlavor::MarlinLegacy, // is this correct ?
                "Repetier" => self.m_flavor = GCodeFlavor::RepetierFirmware,
                "RepRap" => self.m_flavor = GCodeFlavor::RepRapFirmware,
                "Marlin" => self.m_flavor = GCodeFlavor::MarlinLegacy,
                _ => {} // BOOST_LOG_TRIVIAL(warning) << "GCodeProcessor found unknown flavor: " << flavor;
            }

            return true;
        }

        // layer
        tag = "LAYER:";
        if comment.contains(tag) {
            self.m_layer_id += 1;
            return true;
        }

        false
    }

    /// GCodeProcessor.cpp:3739-3879 `bool GCodeProcessor::process_simplify3d_tags(const
    /// std::string_view comment)`
    fn process_simplify3d_tags(&mut self, comment: &str) -> bool {
        // extrusion roles

        // in older versions the comments did not contain the key 'feature'
        let mut cmt = comment;
        if cmt.find(" feature") == Some(0) {
            cmt = &cmt[8..];
        }

        // `cmt.find(x) == 0`
        let at0 = |cmt: &str, x: &str| cmt.find(x) == Some(0);

        // ; skirt
        if at0(cmt, " skirt") {
            self.set_extrusion_role(ExtrusionRole::Skirt);
            return true;
        }

        // ; outer perimeter
        if at0(cmt, " outer perimeter") {
            self.set_extrusion_role(ExtrusionRole::ExternalPerimeter);
            self.m_seams_detector.activate(true);
            return true;
        }

        // ; inner perimeter
        if at0(cmt, " inner perimeter") {
            self.set_extrusion_role(ExtrusionRole::Perimeter);
            return true;
        }

        // ; gap fill
        if at0(cmt, " gap fill") {
            self.set_extrusion_role(ExtrusionRole::GapFill);
            return true;
        }

        // ; infill
        if at0(cmt, " infill") {
            self.set_extrusion_role(ExtrusionRole::InternalInfill);
            return true;
        }

        // ; solid layer
        if at0(cmt, " solid layer") {
            self.set_extrusion_role(ExtrusionRole::SolidInfill);
            return true;
        }

        // ; bridge
        if at0(cmt, " bridge") {
            self.set_extrusion_role(ExtrusionRole::BridgeInfill);
            return true;
        }

        // ; support
        if at0(cmt, " support") {
            self.set_extrusion_role(ExtrusionRole::SupportMaterial);
            return true;
        }

        // ; dense support
        if at0(cmt, " dense support") {
            self.set_extrusion_role(ExtrusionRole::SupportMaterialInterface);
            return true;
        }

        // ; prime pillar
        if at0(cmt, " prime pillar") {
            self.set_extrusion_role(ExtrusionRole::WipeTower);
            return true;
        }

        // ; ooze shield
        if at0(cmt, " ooze shield") {
            self.set_extrusion_role(ExtrusionRole::None); // Missing mapping
            return true;
        }

        // ; raft
        if at0(cmt, " raft") {
            self.set_extrusion_role(ExtrusionRole::SupportMaterial);
            return true;
        }

        // ; internal single extrusion
        if at0(cmt, " internal single extrusion") {
            self.set_extrusion_role(ExtrusionRole::None); // Missing mapping
            return true;
        }

        // geometry
        // ; tool
        let mut tag = " tool";
        if at0(cmt, tag) {
            let data = &cmt[tag.len()..];
            // `substr(start + 1, end != npos ? end - start - 1 : npos)`; `find_first_of(' ', npos)`
            // is npos.
            let sub = |start: usize, end: Option<usize>| -> &str {
                match end {
                    Some(e) => &data[start + 1..e],
                    None => &data[start + 1..],
                }
            };
            let h_start = data.find('H');
            let h_end = h_start.and_then(|h| data[h..].find(' ').map(|p| p + h));
            let w_start = data.find('W');
            let w_end = w_start.and_then(|w| data[w..].find(' ').map(|p| p + w));
            if let Some(h_start) = h_start {
                // BOOST_LOG_TRIVIAL(error) on an invalid value for Height
                let _ = parse_number(sub(h_start, h_end), &mut self.m_forced_height);
            }
            if let Some(w_start) = w_start {
                // BOOST_LOG_TRIVIAL(error) on an invalid value for Width
                let _ = parse_number(sub(w_start, w_end), &mut self.m_forced_width);
            }

            return true;
        }

        // ; layer
        tag = " layer";
        if at0(cmt, tag) {
            // skip lines "; layer end"
            let data = &cmt[tag.len()..];
            if !data.contains("end") {
                self.m_layer_id += 1;
            }

            return true;
        }

        false
    }

    /// GCodeProcessor.cpp:3881-3929 `bool GCodeProcessor::process_craftware_tags(const
    /// std::string_view comment)`
    fn process_craftware_tags(&mut self, comment: &str) -> bool {
        // segType -> extrusion role
        let tag = "segType:";
        if let Some(pos) = comment.find(tag) {
            let r#type = &comment[pos + tag.len()..];
            match r#type {
                "Skirt" => self.set_extrusion_role(ExtrusionRole::Skirt),
                "Perimeter" => self.set_extrusion_role(ExtrusionRole::ExternalPerimeter),
                "HShell" => self.set_extrusion_role(ExtrusionRole::None),
                "InnerHair" => self.set_extrusion_role(ExtrusionRole::None),
                "Loop" => self.set_extrusion_role(ExtrusionRole::None),
                "Infill" => self.set_extrusion_role(ExtrusionRole::InternalInfill),
                "Raft" => self.set_extrusion_role(ExtrusionRole::Skirt),
                "Support" => self.set_extrusion_role(ExtrusionRole::SupportMaterial),
                "SupportTouch" => self.set_extrusion_role(ExtrusionRole::SupportMaterial),
                "SoftSupport" => self.set_extrusion_role(ExtrusionRole::SupportMaterialInterface),
                "Pillar" => self.set_extrusion_role(ExtrusionRole::WipeTower),
                _ => {
                    self.set_extrusion_role(ExtrusionRole::None);
                    // BOOST_LOG_TRIVIAL(warning) << "GCodeProcessor found unknown extrusion role: " << type;
                }
            }

            if self.m_extrusion_role == ExtrusionRole::ExternalPerimeter {
                self.m_seams_detector.activate(true);
            }

            return true;
        }

        // layer
        if comment.find(" Layer #") == Some(0) {
            self.m_layer_id += 1;
            return true;
        }

        false
    }

    /// GCodeProcessor.cpp:3931-3990 `bool GCodeProcessor::process_ideamaker_tags(const
    /// std::string_view comment)`
    fn process_ideamaker_tags(&mut self, comment: &str) -> bool {
        // TYPE -> extrusion role
        let mut tag = "TYPE:";
        if let Some(pos) = comment.find(tag) {
            let r#type = &comment[pos + tag.len()..];
            match r#type {
                "RAFT" => self.set_extrusion_role(ExtrusionRole::Skirt),
                "WALL-OUTER" => self.set_extrusion_role(ExtrusionRole::ExternalPerimeter),
                "WALL-INNER" => self.set_extrusion_role(ExtrusionRole::Perimeter),
                "SOLID-FILL" => self.set_extrusion_role(ExtrusionRole::SolidInfill),
                "FILL" => self.set_extrusion_role(ExtrusionRole::InternalInfill),
                "BRIDGE" => self.set_extrusion_role(ExtrusionRole::BridgeInfill),
                "SUPPORT" => self.set_extrusion_role(ExtrusionRole::SupportMaterial),
                _ => {
                    self.set_extrusion_role(ExtrusionRole::None);
                    // BOOST_LOG_TRIVIAL(warning) << "GCodeProcessor found unknown extrusion role: " << type;
                }
            }

            if self.m_extrusion_role == ExtrusionRole::ExternalPerimeter {
                self.m_seams_detector.activate(true);
            }

            return true;
        }

        // geometry
        // width
        tag = "WIDTH:";
        if let Some(pos) = comment.find(tag) {
            // BOOST_LOG_TRIVIAL(error) on an invalid value for Width
            let _ = parse_number(&comment[pos + tag.len()..], &mut self.m_forced_width);
            return true;
        }

        // height
        tag = "HEIGHT:";
        if let Some(pos) = comment.find(tag) {
            // BOOST_LOG_TRIVIAL(error) on an invalid value for Height
            let _ = parse_number(&comment[pos + tag.len()..], &mut self.m_forced_height);
            return true;
        }

        // layer
        if comment.find("LAYER:") == Some(0) {
            self.m_layer_id += 1;
            return true;
        }

        false
    }

    /// GCodeProcessor.cpp:3992-4098 `bool GCodeProcessor::process_kissslicer_tags(const
    /// std::string_view comment)`
    fn process_kissslicer_tags(&mut self, comment: &str) -> bool {
        // extrusion roles
        let at0 = |x: &str| comment.find(x) == Some(0);

        // ; 'Raft Path'
        if at0(" 'Raft Path'") {
            self.set_extrusion_role(ExtrusionRole::Skirt);
            return true;
        }

        // ; 'Support Interface Path'
        if at0(" 'Support Interface Path'") {
            self.set_extrusion_role(ExtrusionRole::SupportMaterialInterface);
            return true;
        }

        // ; 'Travel/Ironing Path'
        if at0(" 'Travel/Ironing Path'") {
            self.set_extrusion_role(ExtrusionRole::Ironing);
            return true;
        }

        // ; 'Support (may Stack) Path'
        if at0(" 'Support (may Stack) Path'") {
            self.set_extrusion_role(ExtrusionRole::SupportMaterial);
            return true;
        }

        // ; 'Perimeter Path'
        if at0(" 'Perimeter Path'") {
            self.set_extrusion_role(ExtrusionRole::ExternalPerimeter);
            self.m_seams_detector.activate(true);
            return true;
        }

        // ; 'Pillar Path'
        if at0(" 'Pillar Path'") {
            self.set_extrusion_role(ExtrusionRole::None);
            return true;
        }

        // ; 'Destring/Wipe/Jump Path'
        if at0(" 'Destring/Wipe/Jump Path'") {
            self.set_extrusion_role(ExtrusionRole::None);
            return true;
        }

        // ; 'Prime Pillar Path'
        if at0(" 'Prime Pillar Path'") {
            self.set_extrusion_role(ExtrusionRole::None);
            return true;
        }

        // ; 'Loop Path'
        if at0(" 'Loop Path'") {
            self.set_extrusion_role(ExtrusionRole::None);
            return true;
        }

        // ; 'Crown Path'
        if at0(" 'Crown Path'") {
            self.set_extrusion_role(ExtrusionRole::None);
            return true;
        }

        // ; 'Solid Path'
        if at0(" 'Solid Path'") {
            self.set_extrusion_role(ExtrusionRole::None);
            return true;
        }

        // ; 'Stacked Sparse Infill Path'
        if at0(" 'Stacked Sparse Infill Path'") {
            self.set_extrusion_role(ExtrusionRole::InternalInfill);
            return true;
        }

        // ; 'Sparse Infill Path'
        if at0(" 'Sparse Infill Path'") {
            self.set_extrusion_role(ExtrusionRole::SolidInfill);
            return true;
        }

        // geometry

        // layer
        if at0(" BEGIN_LAYER_") {
            self.m_layer_id += 1;
            return true;
        }

        false
    }

    /// GCodeProcessor.cpp:4100-4111 `bool GCodeProcessor::detect_producer(const std::string_view
    /// comment)`
    pub fn detect_producer(&mut self, comment: &str) -> bool {
        for &(id, search_string) in PRODUCERS.iter() {
            if comment.contains(search_string) {
                self.m_producer = id;
                //BOOST_LOG_TRIVIAL(info) << "Detected gcode producer: " << search_string;
                return true;
            }
        }
        false
    }

    /// GCodeProcessor.cpp:5482-5485 `void GCodeProcessor::process_VM104(const GCodeReader::GCodeLine&
    /// line)`
    fn process_vm104(&mut self, line: &crate::g_code_reader::GCodeLine) {
        self.process_m104(line);
    }

    /// GCodeProcessor.cpp:5539-5542 `void GCodeProcessor::process_VM109(const GCodeReader::GCodeLine&
    /// line)`
    fn process_vm109(&mut self, line: &crate::g_code_reader::GCodeLine) {
        self.process_m109(line);
    }

    /// GCodeProcessor.cpp:5986-5991 `void GCodeProcessor::init_filament_maps_when_import_only_gcode()`
    pub fn init_filament_maps_when_import_only_gcode(&mut self) {
        if self.m_filament_maps.is_empty() {
            // EnforcerBlockerType::ExtruderMax
            self.m_filament_maps = vec![1; crate::triangle_selector::EnforcerBlockerType::extruder_max().0 as usize];
        }
    }

    /// GCodeProcessor.cpp:6375-6378 `float GCodeProcessor::get_hotend_change_time()`
    pub fn get_hotend_change_time(&self) -> f32 {
        self.m_time_processor.hotend_change_times
    }

    /// GCodeProcessor.cpp:6380-6399 `void GCodeProcessor::process_custom_gcode_time(CustomGCode::Type
    /// code)`
    pub fn process_custom_gcode_time(&mut self, code: CustomGCodeType) {
        for i in 0..ETimeMode::COUNT {
            if !self.m_time_processor.machines[i].enabled {
                continue;
            }

            self.m_time_processor.machines[i].gcode_time.needed = true;
            //FIXME this simulates st_synchronize! is it correct?
            // The estimated time may be longer than the real print time.
            // machine.simulate_st_synchronize(0, erNone, handle_time_block)
            self.run_calculate_time(i, 0, 0.0, ExtrusionRole::None);
            let gcode_time = &mut self.m_time_processor.machines[i].gcode_time;
            if gcode_time.cache != 0.0 {
                gcode_time.times.push((code, gcode_time.cache));
                gcode_time.cache = 0.0;
            }
        }
    }

    /// GCodeProcessor.cpp:6401-6414 `void GCodeProcessor::process_filaments(CustomGCode::Type code)`
    pub fn process_filaments(&mut self, code: CustomGCodeType) {
        if code == CustomGCodeType::ColorChange {
            self.m_used_filaments.process_color_change_cache();
        }

        if code == CustomGCodeType::ToolChange {
            let active_filament_id = self.get_filament_id(true);
            let active_filament_id_support = self.get_filament_id(false);
            self.m_used_filaments.process_model_cache(active_filament_id);
            self.m_used_filaments.process_support_cache(active_filament_id_support);
            self.m_used_filaments.process_total_volume_cache(active_filament_id);
            //BBS: reset remaining filament
            let last_extruder_id = self.get_extruder_id(true) as usize;
            self.m_remaining_volume[last_extruder_id] = self.m_nozzle_volume[last_extruder_id];
        }
    }

    /// GCodeProcessor.cpp:6456-6507 `void GCodeProcessor::update_preview_layers_times_stats()`
    pub fn update_preview_layers_times_stats(&mut self) {
        // Tag-based layers (spiral vase / scarf) replace the renderer's layer list, so the per-run
        // partition below would not line up with the slider ticks; leave those to layers_times.
        if self.m_detect_layer_based_on_tag {
            return;
        }

        for mode_idx in 0..ETimeMode::COUNT {
            if self.m_result.print_statistics.modes[mode_idx].layers_times.is_empty() {
                continue;
            }

            // Slice time by the same consecutive extrusion-Z runs the renderer builds the slider ticks from
            // (a new run starts whenever the extrusion Z changes by more than EPSILON).
            let mut run_times: Vec<f32> = Vec::new();
            let mut prev_time = 0.0f32; // previous move's cumulative time (move.time is a prefix sum)
            let mut last_z = f32::NAN;
            let mut pending = 0.0f32; // pre-print time before the first extrusion run
            for mv in &self.m_result.moves {
                let current_time = prev_time.max(mv.time[mode_idx]);
                let dt = current_time - prev_time;
                prev_time = current_time;
                if mv.move_type == EMoveType::Extrude
                    && (run_times.is_empty() || (mv.position[2] - last_z).abs() > EPSILON)
                {
                    run_times.push(0.0);
                    last_z = mv.position[2];
                }
                if run_times.is_empty() {
                    pending += dt; // before the first extrusion; folded into the first run below
                } else {
                    *run_times.last_mut().unwrap() += dt;
                }
            }

            let data = &mut self.m_result.print_statistics.modes[mode_idx];
            // Only mixed-color sublayers make the runs finer than the logical layers; otherwise layers_times
            // already lines up with the slider, so leave preview empty and let the slider reuse it.
            if run_times.len() <= data.layers_times.len() {
                continue;
            }

            // Fold the pre-print time (and any rounding) into the first run so the prefix sum the slider
            // builds from these durations ends exactly at the estimated total time.
            run_times[0] += pending;
            let mut sum = 0.0f32;
            for &t in &run_times {
                sum += t;
            }
            run_times[0] += 0.0f32.max(data.time - sum);
            data.preview_layers_times = run_times;
        }
    }

    /// GCodeProcessor.cpp:6509-6563 `void GCodeProcessor::update_slice_warnings()` (BBS: ugly code...)
    pub fn update_slice_warnings(&mut self) {
        self.m_result.warnings.clear();

        let mut used_filaments: Vec<usize> = Vec::with_capacity(self.m_used_filaments.total_volumes_per_filament.len());
        for (&k, _) in &self.m_used_filaments.total_volumes_per_filament {
            used_filaments.push(k);
        }
        // assert(!used_filaments.empty()) is compiled out (NDEBUG).
        let _ = used_filaments;
        let mut warning = SliceWarning::default();

        // bbs:timelapse checker
        warning.params.clear();
        warning.level = 1;
        if !self.m_result.support_traditional_timelapse {
            warning.level = 2;
            warning.msg = NOT_SUPPORT_TRADITIONAL_TIMELAPSE.to_string();
            warning.error_code = "10018003".to_string();
            self.m_result.warnings.push(warning.clone());

            // Compatible with older version for A series
            warning.level = 3;
            warning.error_code = "1000C003".to_string();
            self.m_result.warnings.push(warning.clone());
        }

        if self.m_result.timelapse_warning_code != 0 {
            if self.m_result.timelapse_warning_code & 1 != 0 {
                warning.level = 1;
                warning.msg = NOT_GENERATE_TIMELAPSE.to_string();
                warning.error_code = "10014001".to_string();
                self.m_result.warnings.push(warning.clone());
            }
            if (self.m_result.timelapse_warning_code >> 1) & 1 != 0 {
                warning.level = 1;
                warning.msg = NOT_GENERATE_TIMELAPSE.to_string();
                warning.error_code = "10014002".to_string();
                self.m_result.warnings.push(warning.clone());
            }
            if (self.m_result.timelapse_warning_code >> 2) & 1 != 0 {
                warning.level = 2;
                warning.msg = SMOOTH_TIMELAPSE_WITHOUT_PRIME_TOWER.to_string();
                warning.error_code = "10018004".to_string();
                self.m_result.warnings.push(warning.clone());
            }
        }

        self.m_result.warnings.shrink_to_fit();
    }
}

/// GCodeProcessor.hpp:16-18 `#define NOT_SUPPORT_TRADITIONAL_TIMELAPSE`, `NOT_GENERATE_TIMELAPSE`,
/// `SMOOTH_TIMELAPSE_WITHOUT_PRIME_TOWER` (the message keys of the slice warnings).
pub const NOT_SUPPORT_TRADITIONAL_TIMELAPSE: &str = "not_support_traditional_timelapse";
pub const NOT_GENERATE_TIMELAPSE: &str = "not_generate_timelapse";
pub const SMOOTH_TIMELAPSE_WITHOUT_PRIME_TOWER: &str = "smooth_timelapse_without_prime_tower";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serialized_filament_offsets_preserve_scalar_configs() {
        use crate::print_config::PrintConfig;
        let mut config = PrintConfig::default();
        config.filament_diameters = vec![1.75, 1.75];
        config.extruder_offset_x = -1.25;
        config.extruder_offset_y = 3.5;
        let legacy = serde_json::to_value(&config).unwrap();
        assert!(legacy.get("extruder_offsets").is_none() && legacy.get("master_extruder_id").is_none());
        let mut config: PrintConfig = serde_json::from_value(legacy).unwrap();
        let mut processor = GCodeProcessor::new();
        processor.apply_config(&config);
        assert_eq!(processor.m_extruder_offsets, vec![[-1.25, 3.5, 0.0]; 2]);

        config.set_deserialize("extruder_offset", "0x0,12.25x-3.5");
        config.set_deserialize("master_extruder_id", "2");
        let config: PrintConfig = serde_json::from_value(serde_json::to_value(config).unwrap()).unwrap();
        processor.apply_config(&config);
        assert_eq!(processor.m_extruder_offsets, vec![[12.25, -3.5, 0.0]; 2]);
        assert_eq!([config.extruder_offset_x, config.extruder_offset_y], [-1.25, 3.5]);
    }

    /// GCodeProcessor.cpp:2016-2022. Each filament selects its physical
    /// extruder's XY offset. A short map is extended with master_extruder_id.
    #[test]
    fn mapped_filament_offsets_match_native() {
        use crate::print_config::PrintConfig;
        let vectors = include_str!("../../tests/data/g_code_processor/filament_offsets.txt");
        let mut cases = 0;
        for line in vectors.lines().filter(|line| line.starts_with("rows ")) {
            let fields: Vec<_> = line.split_whitespace().collect();
            let expected: Vec<_> = fields[4..].iter().map(|s| u32::from_str_radix(s, 16).unwrap()).collect();
            for direct in [false, true] {
                let mut config = PrintConfig::default();
                config.filament_diameters = vec![1.75, 1.75];
                let mapping = if fields[2] == "-" { "" } else { fields[2] };
                for (key, value) in [("extruder_offset", fields[1]), ("filament_map", mapping), ("master_extruder_id", fields[3])] {
                    if direct {
                        config.apply_key_value(key, value);
                    } else {
                        config.set_deserialize(key, value);
                    }
                }
                let mut processor = GCodeProcessor::new();
                processor.apply_config(&config);
                let offsets: Vec<_> = processor.m_extruder_offsets.iter()
                    .flat_map(|p| [p[0].to_bits(), p[1].to_bits()]).collect();
                assert_eq!(offsets, expected, "{line}, direct={direct}");
            }
            cases += 1;
        }
        assert_eq!(cases, 48);
    }

    /// GCodeProcessor.cpp:2030-2036, 2055-2066, 6248-6262. Compare initialized
    /// machine limits and minimum feedrates in both modes for every firmware
    /// that applies the configured machine envelope.
    #[test]
    fn profile_motion_limits_match_native() {
        use crate::print_config::PrintConfig;
        let vectors = include_str!("../../tests/data/g_code_processor/motion_limits.txt");
        let mut cases = 0;
        for line in vectors.lines().filter(|line| line.starts_with("rows ")) {
            let mut fields = line.split_whitespace().skip(1);
            let flavor = fields.next().unwrap();
            let key = fields.next().unwrap();
            let encoded = fields.next().unwrap();
            let values = if encoded == "-" { "" } else { encoded };
            for key_value in [false, true] {
                let mut config = PrintConfig::default();
                assert!(config.set_deserialize("gcode_flavor", flavor));
                for (name, value) in [
                    ("machine_max_acceleration_extruding", "1100,700"),
                    ("machine_max_acceleration_retracting", "2100,800"),
                    ("machine_max_acceleration_travel", "3100,900"),
                    ("machine_min_extruding_rate", "10,8"),
                    ("machine_min_travel_rate", "11,9"),
                    (key, values),
                ] {
                    if key_value {
                        assert!(config.apply_key_value(name, value), "{name}");
                    } else {
                        assert!(config.set_deserialize(name, value), "{name}");
                    }
                }
                let mut processor = GCodeProcessor::new();
                processor.apply_config(&config);
                let mut expected = fields.clone();
                for mode in [ETimeMode::Normal, ETimeMode::Stealth] {
                    let machine = &processor.m_time_processor.machines[mode as usize];
                    for value in [machine.max_acceleration, machine.max_retract_acceleration,
                                  machine.max_travel_acceleration, processor.minimum_feedrate(mode, 9.5),
                                  processor.minimum_travel_feedrate(mode, 9.5)] {
                        let word = expected.next().unwrap();
                        if word == "nil" {
                            assert!(value.is_nan(), "{line} {mode:?}");
                        } else {
                            assert_eq!(value.to_bits(), u32::from_str_radix(word, 16).unwrap(), "{line} {mode:?}");
                        }
                    }
                }
                assert!(expected.next().is_none());
            }
            cases += 1;
        }
        assert_eq!(cases, 75);
    }

    /// GCodeProcessor.cpp:6264-6303. Six lookups per profile option cover both
    /// timing modes and in-range and missing nozzle rows.
    #[test]
    fn profile_axis_limits_match_native() {
        let vectors = include_str!("../../tests/data/g_code_processor/axis_limits.txt");
        let mut cases = 0;
        for line in vectors.lines().filter(|line| line.starts_with("rows ")) {
            let mut fields = line.split_whitespace();
            fields.next();
            let key = fields.next().unwrap();
            let encoded = fields.next().unwrap();
            let value = if encoded == "-" { "" } else { encoded };
            let axis = "xyze".find(key.chars().last().unwrap()).unwrap();

            for use_key_value in [false, true] {
                let mut config = crate::print_config::PrintConfig::default();
                config.gcode_flavor = crate::print_config::GCodeFlavor::Marlin;
                if use_key_value {
                    assert!(config.apply_key_value(key, value));
                } else {
                    assert!(config.set_deserialize(key, value));
                }
                let mut processor = GCodeProcessor::new();
                processor.apply_config(&config);
                let mut expected = fields.clone();
                for row in 0..3 {
                    for mode in [ETimeMode::Normal, ETimeMode::Stealth] {
                        let limit = if key.starts_with("machine_max_speed_") {
                            processor.get_axis_max_feedrate(mode, axis, row)
                        } else if key.starts_with("machine_max_acceleration_") {
                            processor.get_axis_max_acceleration(mode, axis, row)
                        } else {
                            processor.get_axis_max_jerk(mode, axis)
                        };
                        let bits = expected.next().unwrap();
                        if bits == "nil" {
                            assert!(limit.is_nan(), "{line}, row {row}, mode {mode:?}");
                        } else {
                            assert_eq!(limit.to_bits(), u32::from_str_radix(bits, 16).unwrap(), "{line}, row {row}, mode {mode:?}");
                        }
                    }
                }
                assert!(expected.next().is_none());
            }
            cases += 1;
        }
        assert_eq!(cases, 60);
    }

    /// The pin's GCodeProcessor.cpp:6596-6610 over the states emitted by
    /// tests/data/g_code_processor/gen.cpp --machine-rows.
    #[test]
    fn machine_config_rows_match_native() {
        use crate::extruder::NozzleVolumeType::{NvtHighFlow, NvtStandard, NvtTPUHighFlow};
        use crate::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleInfo};

        let mut config = crate::print_config::PrintConfig::default();
        config.printer_extruder_id = vec![2, 1, 1, 2];
        config.printer_extruder_variant = ["Bowden Standard", "Direct Drive Standard",
                                          "Direct Drive High Flow", "Bowden High Flow"].map(str::to_owned).to_vec();
        config.extruder_type = vec![0, 1];
        let nozzles: Vec<_> = [(0, NvtStandard), (0, NvtHighFlow), (1, NvtHighFlow), (0, NvtTPUHighFlow)]
            .into_iter().enumerate().map(|(id, (extruder_id, volume_type))| NozzleInfo {
                group_id: id as i32, extruder_id, volume_type, diameter: "0.4".into(), installed: true,
            }).collect();
        let group = LayeredNozzleGroupResult::create(&[0, 1, 2, 3], &nozzles, &[0, 1, 2, 3]).unwrap();
        let vectors = include_str!("../../tests/data/g_code_processor/machine_rows.txt");
        assert_eq!(vectors.lines().count(), 144);
        for line in vectors.lines() {
            let values: Vec<i32> = line.split_whitespace().map(|s| s.parse().unwrap()).collect();
            let [grouped, extruder, nozzle, filament, expected] = values[..] else { panic!("bad vector {line}") };
            let mut p = GCodeProcessor::new();
            p.apply_config(&config);
            p.initialize_from_context((grouped == 1).then(|| group.clone()));
            p.m_extruder_id = extruder as u8;
            if nozzle >= 0 {
                let extruder_id = p.get_extruder_id(true);
                p.m_nozzle_status_recorder.set_nozzle_status(nozzle, 0, extruder_id);
            }
            assert_eq!(p.get_machine_config_idx(filament), expected, "{line}");
        }
    }

    /// R953: `GCodeProcessor::get_time_dhm` against the pin. The vectors
    /// (tests/data/g_code_processor_used_filaments/g_code_processor_time_dhm.txt, written by gen.cpp
    /// beside them, which includes the pin's GCodeProcessor.cpp) give per line the mode, the machine's
    /// time as float bits and the pin's string, for 324 times in both modes.
    #[test]
    fn get_time_dhm_native_r953() {
        let path = format!(
            "{}/tests/data/g_code_processor_used_filaments/g_code_processor_time_dhm.txt",
            env!("CARGO_MANIFEST_DIR")
        );
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        let mut n = 0usize;
        let mut bad: Vec<String> = Vec::new();
        for line in text.lines() {
            let t: Vec<&str> = line.splitn(4, ' ').collect();
            assert_eq!(t[0], "T");
            let mode = if t[1] == "0" { ETimeMode::Normal } else { ETimeMode::Stealth };
            let time = f32::from_bits(u32::from_str_radix(t[2], 16).unwrap());
            let mut p = GCodeProcessor::new();
            p.m_time_processor.machines[mode as usize].time = time;
            let got = p.get_time_dhm(mode);
            n += 1;
            if got != t[3] {
                bad.push(format!("mode {} time {time}: port {got:?} pin {:?}", t[1], t[3]));
            }
        }
        assert!(n >= 600, "only {n} lines");
        assert!(bad.is_empty(), "{} of {n} differ:\n{}", bad.len(), bad.join("\n"));
    }

    /// R953 PROCESS_M702_PIN: `M702 C` unloads the active filament (GCodeProcessor.cpp:5874-5886).
    /// With no extruder selected `get_filament_id()` is 0 (forced, :6565-6575), the unload time is 0
    /// because `extruder_unloaded` is set first (:5880, :6363-6367), and `unload_time_per_filament[0]
    /// += 0` still creates the entry (std::map::operator[]). A plain `M702` changes nothing.
    #[test]
    fn process_m702_records_the_unload_entry_r953() {
        let mut p = GCodeProcessor::new();
        p.process_buffer("M702\n");
        assert!(p.m_result.print_statistics.unload_time_per_filament.is_empty());
        // TimeProcessor::reset starts with extruder_unloaded = true (GCodeProcessor.cpp:556).
        assert!(p.m_time_processor.extruder_unloaded);
        p.process_buffer("M702 C\n");
        assert!(p.m_time_processor.extruder_unloaded);
        if crate::faithful_gate("PROCESS_M702_PIN") {
            assert_eq!(p.m_result.print_statistics.unload_time_per_filament.get(&0), Some(&0.0));
            assert_eq!(p.m_result.print_statistics.unload_time_per_filament.len(), 1);
        } else {
            assert!(p.m_result.print_statistics.unload_time_per_filament.is_empty());
        }
    }

    /// The `producer` lines of tests/data/g_code_processor/g_code_processor.txt.gz (gen.cpp, which
    /// includes the pin's GCodeProcessor.cpp): `process_producers_tags` with `m_producer` set, from a
    /// fresh processor per comment. The C++ detects no producer but BambuStudio at the pin, so the
    /// tag parsers of the other producers are reached only this way.
    #[test]
    fn test_producer_tags_native() {
        use std::io::Read;
        let path = format!("{}/tests/data/g_code_processor/g_code_processor.txt.gz", env!("CARGO_MANIFEST_DIR"));
        let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        let mut text = String::new();
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
        let unhex = |t: &str| -> String {
            if t == "-" {
                return String::new();
            }
            let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).unwrap()).collect();
            String::from_utf8(b).unwrap()
        };
        // PrintConfig.hpp:40-43 `enum GCodeFlavor`
        let flavor_id = |f: GCodeFlavor| -> i32 {
            match f {
                GCodeFlavor::MarlinLegacy => 0,
                GCodeFlavor::Klipper => 1,
                GCodeFlavor::RepRapSprinter => 2,
                GCodeFlavor::RepRapFirmware => 3,
                GCodeFlavor::RepetierFirmware => 4,
                GCodeFlavor::MakerWare => 6,
                GCodeFlavor::MarlinFirmware => 7,
                GCodeFlavor::Sailfish => 8,
                GCodeFlavor::Mach3 => 9,
                GCodeFlavor::Smoothie => 11,
                GCodeFlavor::Other => -1,
            }
        };
        let producers = [
            EProducer::Unknown,
            EProducer::BambuStudio,
            EProducer::Slic3rPE,
            EProducer::Slic3r,
            EProducer::SuperSlicer,
            EProducer::Cura,
            EProducer::Simplify3D,
            EProducer::CraftWare,
            EProducer::IdeaMaker,
            EProducer::KissSlicer,
        ];
        let mut n = 0usize;
        let mut bad: Vec<String> = Vec::new();
        for line in text.lines().filter(|l| l.starts_with("producer ")) {
            let t: Vec<&str> = line.split(' ').collect();
            let id: usize = t[1].parse().unwrap();
            let comment = unhex(t[2]);
            let mut p = GCodeProcessor::new();
            p.m_producer = producers[id];
            p.m_forced_width = 0.0;
            p.m_forced_height = 0.0;
            let r = p.process_producers_tags(&comment);
            let got = format!(
                "producer {} {} {} {} {} {:08x} {:08x} {} {}",
                id,
                t[2],
                r as i32,
                p.m_extrusion_role as u8,
                p.m_layer_id,
                p.m_forced_width.to_bits(),
                p.m_forced_height.to_bits(),
                flavor_id(p.m_flavor),
                p.m_seams_detector.is_active() as i32
            );
            n += 1;
            if got != line {
                bad.push(format!("  want {line}\n  got  {got}"));
            }
        }
        assert!(n > 800, "only {n} producer lines");
        assert!(bad.is_empty(), "{} of {} producer lines differ:\n{}", bad.len(), n, bad.join("\n"));
    }

    #[test]
    fn test_kinematics_helpers() {
        // estimated_acceleration_distance with zero acceleration => 0
        assert_eq!(estimated_acceleration_distance(10.0, 20.0, 0.0), 0.0);
        // (20^2 - 10^2) / (2*5) = (400-100)/10 = 30
        assert_eq!(estimated_acceleration_distance(10.0, 20.0, 5.0), 30.0);
        // speed_from_distance clamps negatives
        assert_eq!(speed_from_distance(0.0, -100.0, 1.0), 0.0);
        // max_allowable_speed clamps negatives
        assert_eq!(max_allowable_speed(1.0, 0.0, 100.0), 0.0);
    }

    #[test]
    fn test_trapezoid_and_time() {
        let mut b = TimeBlock {
            distance: 10.0,
            acceleration: 100.0,
            feedrate_profile: FeedrateProfile {
                entry: 0.0,
                cruise: 30.0,
                exit: 0.0,
            },
            ..Default::default()
        };
        b.calculate_trapezoid();
        let t = b.time();
        assert!(t > 0.0);
    }

    #[test]
    fn test_command_processor_trie() {
        let mut cp = CommandProcessor::new();
        cp.register_command("G1", 7, false);
        cp.register_command("M73", 42, false);
        assert_eq!(cp.process_comand("G1"), Some(7));
        assert_eq!(cp.process_comand("M73"), Some(42));
        assert_eq!(cp.process_comand("G2"), None);
        assert_eq!(cp.process_comand("X"), None);
    }

    #[test]
    fn test_command_processor_early_quit() {
        let mut cp = CommandProcessor::new();
        // Register "T" with early_quit so any "T..." matches at the T node.
        cp.register_command("T", 1, true);
        assert_eq!(cp.process_comand("T0"), Some(1));
        assert_eq!(cp.process_comand("T12"), Some(1));
    }

    #[test]
    fn test_get_object_label_id() {
        assert_eq!(get_object_label_id(" OBJECT_ID: 42"), 42);
        assert_eq!(get_object_label_id("no colon"), -1);
    }

    #[test]
    fn test_get_z_height() {
        assert!((get_z_height("Z_HEIGHT: 1.25") - 1.25).abs() < 1e-6);
        assert_eq!(get_z_height("nope"), 0.0);
    }

    #[test]
    fn test_parse_number_i32() {
        assert_eq!(parse_number_i32("0"), Some(0));
        assert_eq!(parse_number_i32("254"), Some(254));
        assert_eq!(parse_number_i32("-3"), Some(-3));
        assert_eq!(parse_number_i32("1.5"), None);
        assert_eq!(parse_number_i32("12a"), None);
        assert_eq!(parse_number_i32("+5"), None); // from_chars rejects leading '+'
    }

    #[test]
    fn test_get_gcode_last_filament() {
        let g = "G1 X1\nT0\nG1 X2\nT3\nG1 X3\n";
        assert_eq!(get_gcode_last_filament(g), 3);
        let none = "G1 X1\nG1 X2\n";
        assert_eq!(get_gcode_last_filament(none), -1);
    }

    #[test]
    fn test_get_last_z_from_gcode() {
        let g = "G1 X1 Y1 Z0.2\nG1 X2 Z0.4\nG1 X3\n";
        assert_eq!(get_last_z_from_gcode(g), Some(0.4));
        let none = "G1 X1 Y1\n";
        assert_eq!(get_last_z_from_gcode(none), None);
    }

    #[test]
    fn test_get_last_position_from_gcode() {
        let g = "G1 X1 Y2 Z0.2\nG1 X3 Y4 Z0.4\n";
        let mut p = [0.0f32; 3];
        assert!(get_last_position_from_gcode(g, &mut p));
        assert!((p[0] - 3.0).abs() < 1e-6);
        assert!((p[1] - 4.0).abs() < 1e-6);
        assert!((p[2] - 0.4).abs() < 1e-6);
    }

    #[test]
    fn test_reserved_tags_table() {
        assert_eq!(reserved_tag(ETags::Role), " FEATURE: ");
        assert_eq!(reserved_tag(ETags::CpToolchangeWipe), " CP_TOOLCHANGE_WIPE");
        assert_eq!(custom_tags(CustomETags::SkippableType), " SKIPTYPE: ");
    }

    #[test]
    fn test_contains_reserved_tag() {
        let g = "; FEATURE: Outer wall\nG1 X1\n";
        assert_eq!(
            contains_reserved_tag(g).as_deref(),
            Some(" FEATURE: Outer wall")
        );
        let none = "G1 X1\n; just a comment\n";
        assert_eq!(contains_reserved_tag(none), None);
    }

    #[test]
    fn test_used_filaments_caches() {
        let mut uf = UsedFilaments::default();
        uf.increase_model_caches(100.0);
        assert_eq!(uf.model_extrude_cache, 100.0);
        assert_eq!(uf.total_volume_cache, 100.0);
        uf.process_model_cache(2);
        assert_eq!(*uf.model_volumes_per_filament.get(&2).unwrap(), 100.0);
        assert_eq!(uf.model_extrude_cache, 0.0);
    }

    #[test]
    fn test_cached_position_reset() {
        let mut cp = CachedPosition::default();
        cp.reset();
        assert_eq!(cp.feedrate, f32::MAX);
        assert_eq!(cp.position[0], f32::MAX as f64);
    }
}
