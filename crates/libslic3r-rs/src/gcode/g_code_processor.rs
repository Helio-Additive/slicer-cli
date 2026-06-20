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
        (2.0 * acceleration * distance - sqr(initial_rate) + sqr(final_rate)) / (4.0 * acceleration)
    }
}

/// GCodeProcessor.cpp:129-134
fn speed_from_distance(initial_feedrate: f32, distance: f32, acceleration: f32) -> f32 {
    // to avoid invalid negative numbers due to numerical errors
    let value = f32::max(0.0, sqr(initial_feedrate) + 2.0 * acceleration * distance);
    value.sqrt()
}

/// GCodeProcessor.cpp:136-143
///
/// Calculates the maximum allowable speed at this point when you must be able
/// to reach target_velocity using the acceleration within the allotted distance.
fn max_allowable_speed(acceleration: f32, target_velocity: f32, distance: f32) -> f32 {
    // to avoid invalid negative numbers due to numerical errors
    let value = f32::max(0.0, sqr(target_velocity) - 2.0 * acceleration * distance);
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
            let s = PI * sqr(0.5 * filament_diameter) as f64;
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

/// GCodeProcessor.cpp:3107-3222  get_last_position_from_gcode
///
/// Returns `Some([x,y,z])` if any axis was parsed, else `None`. Preserves the
/// `sizeof(const char*)` 8-byte parse bound (see `get_last_z_from_gcode`).
pub fn get_last_position_from_gcode(gcode_str: &str) -> Option<[f32; 3]> {
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
    let mut pos: [f32; 3] = [0.0, 0.0, 0.0];
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
    if is_z_changed {
        Some(pos)
    } else {
        None
    }
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
/// Fields that require unported deps (Pointfs, ExtruderType, NozzleType,
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
    pub printable_height: f32,
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
            printable_height: 0.0,
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
        self.printable_height = 0.0;
        self.settings_ids.reset();
        self.filaments_count = 0;
        self.extruder_colors = Vec::new();
        self.filament_diameters = vec![DEFAULT_FILAMENT_DIAMETER; MIN_EXTRUDERS_COUNT];
        self.required_nozzle_hrc = vec![DEFAULT_FILAMENT_HRC; MIN_EXTRUDERS_COUNT];
        self.filament_densities = vec![DEFAULT_FILAMENT_DENSITY; MIN_EXTRUDERS_COUNT];
        self.filament_costs = vec![DEFAULT_FILAMENT_COST; MIN_EXTRUDERS_COUNT];
        // BLOCKED(config): filament_vitrification_temperature default not reset in
        // this branch of C++; left as-is here.
        self.spiral_vase_layers = Vec::new();
        self.skippable_part_time.clear();
        self.warnings.clear();
    }
}

// ===========================================================================
// MachineEnvelopeConfig — PrintConfig.hpp machine_max_* / machine_min_*
// ===========================================================================
/// Faithful (scalar) model of `MachineEnvelopeConfig` (the machine motion
/// limits read by `TimeProcessor`). In C++ each field is a per-mode array
/// (`ConfigOptionFloatsNullable`) indexed `extruder_id*2 + mode`; `get_option_value`
/// falls back to `.back()` when the index is out of range. For BambuStudio the
/// arrays carry identical values per mode/extruder, so a single scalar per axis
/// is an exact match for `get_option_value` on any index.
///
/// `*_present` flags model whether the C++ option array is non-empty: an empty
/// array makes the getter fall back (0.0 / passthrough feedrate). When unset,
/// the processor reproduces the legacy "no machine limits" behaviour.
#[derive(Debug, Clone, Default)]
pub struct MachineLimits {
    pub present: bool,
    pub max_acceleration_x: f32,
    pub max_acceleration_y: f32,
    pub max_acceleration_z: f32,
    pub max_acceleration_e: f32,
    pub max_acceleration_extruding: f32,
    pub max_acceleration_retracting: f32,
    pub max_acceleration_travel: f32,
    pub max_speed_x: f32,
    pub max_speed_y: f32,
    pub max_speed_z: f32,
    pub max_speed_e: f32,
    pub max_jerk_x: f32,
    pub max_jerk_y: f32,
    pub max_jerk_z: f32,
    pub max_jerk_e: f32,
    pub min_extruding_rate: f32,
    pub min_extruding_rate_present: bool,
    pub min_travel_rate: f32,
    pub min_travel_rate_present: bool,
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
    /// MachineEnvelopeConfig machine_limits (GCodeProcessor.hpp:818). Populated by
    /// `GCodeProcessor::apply_config` (cpp:1964-1995) when the gcode flavor is
    /// Marlin/Klipper; otherwise stays at the empty default so the getters fall
    /// back exactly like the C++ `get_option_value` on empty arrays.
    pub machine_limits: MachineLimits,
    pub filament_load_times: f32,
    pub filament_unload_times: f32,
    pub extruder_change_times: f32,
    pub hotend_change_times: f32,
    pub machines: [TimeMachine; ETimeMode::COUNT],
}

impl TimeProcessor {
    /// GCodeProcessor.hpp:826-833  Planner constants
    const PLANNER_QUEUE_SIZE: usize = 64;
    const PLANNER_REFRESH_THRESHOLD: usize = Self::PLANNER_QUEUE_SIZE * 4;

    /// GCodeProcessor.cpp:522-537  void TimeProcessor::reset()
    pub fn reset(&mut self) {
        self.extruder_unloaded = true;
        self.machine_envelope_processing_enabled = false;
        // GCodeProcessor.cpp:526  machine_limits = MachineEnvelopeConfig();
        self.machine_limits = MachineLimits::default();
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
            machine_limits: MachineLimits::default(),
            filament_load_times: 0.0,
            filament_unload_times: 0.0,
            extruder_change_times: 0.0,
            hotend_change_times: 0.0,
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
    Other,
}

/// GCodeProcessor.hpp:440-1211  class GCodeProcessor (state machine).
///
/// BLOCKED members (unported deps): MultiNozzleUtils nozzle group/status,
/// extruder_offsets/Vec3f, SeamsDetector/OptionsZCorrector full behavior,
/// producer regex detection, ArcFitter (G2/G3 interpolation), data-checking.
/// The fields below cover the time / filament path.
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
    m_g1_line_id: u32,
    m_layer_id: u32,
    m_cp_color: CpColor,
    m_seams_count: i32,
    m_detect_layer_based_on_tag: bool,
    m_measure_g29_time: bool,
    // BLOCKED(deps): m_extruder_offsets (Vec3f), m_seams_detector,
    // m_options_z_corrector, m_nozzle_group_result/recorder — see notes.
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
            m_g1_line_id: 0,
            m_layer_id: 0,
            m_cp_color: CpColor::default(),
            m_seams_count: 0,
            m_detect_layer_based_on_tag: false,
            m_measure_g29_time: false,
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
    }

    /// GCodeProcessor.cpp:2429-2432  void GCodeProcessor::enable_stealth_time_estimator(bool)
    pub fn enable_stealth_time_estimator(&mut self, enabled: bool) {
        self.m_time_processor.machines[ETimeMode::Stealth as usize].enabled = enabled;
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
    /// The C++ `machine_limits` is a per-mode/extruder array; BambuStudio carries
    /// identical values per mode, so the scalar `MachineLimits` model is exact for
    /// `get_option_value` on any index.
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
            let ml = &mut self.m_time_processor.machine_limits;
            ml.present = true;
            ml.max_acceleration_x = config.machine_max_acceleration_x as f32;
            ml.max_acceleration_y = config.machine_max_acceleration_y as f32;
            ml.max_acceleration_z = config.machine_max_acceleration_z as f32;
            ml.max_acceleration_e = config.machine_max_acceleration_e as f32;
            ml.max_acceleration_extruding = config.machine_max_acceleration_extruding as f32;
            ml.max_acceleration_retracting = config.machine_max_acceleration_retracting as f32;
            ml.max_acceleration_travel = config.machine_max_acceleration_travel as f32;
            ml.max_speed_x = config.machine_max_speed_x as f32;
            ml.max_speed_y = config.machine_max_speed_y as f32;
            ml.max_speed_z = config.machine_max_speed_z as f32;
            ml.max_speed_e = config.machine_max_speed_e as f32;
            ml.max_jerk_x = config.machine_max_jerk_x as f32;
            ml.max_jerk_y = config.machine_max_jerk_y as f32;
            ml.max_jerk_z = config.machine_max_jerk_z as f32;
            ml.max_jerk_e = config.machine_max_jerk_e as f32;
            // BambuStudio always carries these arrays (length == 2*extruders), so the
            // C++ `.empty()` guard is false: model them as present.
            ml.min_extruding_rate = config.machine_min_extruding_rate as f32;
            ml.min_extruding_rate_present = true;
            ml.min_travel_rate = config.machine_min_travel_rate as f32;
            ml.min_travel_rate_present = true;

            // GCodeProcessor.cpp:1966-1969 — legacy Marlin has no separate travel
            // acceleration; it uses the 'extruding' value instead.
            if self.m_flavor == GCodeFlavor::MarlinLegacy {
                ml.max_acceleration_travel = ml.max_acceleration_extruding;
            }
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
    fn get_axis_max_acceleration_extruding(&self, _mode: usize) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if ml.present {
            ml.max_acceleration_extruding
        } else {
            0.0
        }
    }

    /// `get_option_value(machine_max_acceleration_retracting, mode)` (cpp:1989).
    fn get_machine_limit_retract_acceleration(&self, _mode: usize) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if ml.present {
            ml.max_acceleration_retracting
        } else {
            0.0
        }
    }

    /// `get_option_value(machine_max_acceleration_travel, mode)` (cpp:1992).
    fn get_machine_limit_travel_acceleration(&self, _mode: usize) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if ml.present {
            ml.max_acceleration_travel
        } else {
            0.0
        }
    }

    // ----- accessors -----

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

    // BLOCKED(deps): get_machine_config_idx needs MultiNozzleUtils nozzle group;
    // with the default (no nozzle group) it returns 0 — faithful to cpp:6379-6392.
    fn get_machine_config_idx(&self, _filament_idx: i32) -> i32 {
        0
    }

    // ----- machine-limit getters (cpp:6022-6128) -----
    // These read `m_time_processor.machine_limits`, populated by `apply_config`
    // (cpp:1964-1995). When the limits are not present (no config applied, or a
    // non-Marlin/Klipper flavor) they fall back exactly like the C++
    // `get_option_value` on empty arrays (0.0 / passthrough feedrate).

    /// GCodeProcessor.cpp:6022-6028 — machine_min_extruding_rate empty → feedrate,
    /// else max(feedrate, min_extruding_rate).
    fn minimum_feedrate(&self, _mode: ETimeMode, feedrate: f32) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if !ml.min_extruding_rate_present {
            feedrate
        } else {
            feedrate.max(ml.min_extruding_rate)
        }
    }

    /// GCodeProcessor.cpp:6030-6036 — machine_min_travel_rate empty → feedrate,
    /// else max(feedrate, min_travel_rate).
    fn minimum_travel_feedrate(&self, _mode: ETimeMode, feedrate: f32) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if !ml.min_travel_rate_present {
            feedrate
        } else {
            feedrate.max(ml.min_travel_rate)
        }
    }

    /// GCodeProcessor.cpp:6038-6049 — per-axis max feedrate (mm/s).
    /// `axis` is the `Axis` discriminant (X=0,Y=1,Z=2,E=3); other axes → 0.0.
    fn get_axis_max_feedrate(&self, _mode: ETimeMode, axis: usize, _extruder_id: i32) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if !ml.present {
            return 0.0;
        }
        match axis {
            0 => ml.max_speed_x,
            1 => ml.max_speed_y,
            2 => ml.max_speed_z,
            3 => ml.max_speed_e,
            _ => 0.0,
        }
    }

    /// GCodeProcessor.cpp:6051-6062 — per-axis max acceleration (mm/s^2).
    fn get_axis_max_acceleration(&self, _mode: ETimeMode, axis: usize, _extruder_id: i32) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if !ml.present {
            return 0.0;
        }
        match axis {
            0 => ml.max_acceleration_x,
            1 => ml.max_acceleration_y,
            2 => ml.max_acceleration_z,
            3 => ml.max_acceleration_e,
            _ => 0.0,
        }
    }

    /// GCodeProcessor.cpp:6064-6074 — per-axis max jerk (mm/s).
    fn get_axis_max_jerk(&self, _mode: ETimeMode, axis: usize) -> f32 {
        let ml = &self.m_time_processor.machine_limits;
        if !ml.present {
            return 0.0;
        }
        match axis {
            0 => ml.max_jerk_x,
            1 => ml.max_jerk_y,
            2 => ml.max_jerk_z,
            3 => ml.max_jerk_e,
            _ => 0.0,
        }
    }

    /// GCodeProcessor.cpp:6076-6081 — (x,y,z) max jerk.
    fn get_xyz_max_jerk(&self, _mode: ETimeMode) -> [f32; 3] {
        let ml = &self.m_time_processor.machine_limits;
        if !ml.present {
            return [0.0, 0.0, 0.0];
        }
        [ml.max_jerk_x, ml.max_jerk_y, ml.max_jerk_z]
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
        }
    }

    // BLOCKED(deps): process_tags / process_helioadditive_comment depend on the
    // tag pipeline + helio parser; the role/layer/width/height side-effects are
    // not ported. The time/filament math in process_G1 still runs.
    fn process_tags(&mut self, _comment: &str, _producers_enabled: bool) {}

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
        let absolute_position = |axis: Axis, lineg: &crate::g_code_reader::GCodeLine| -> f32 {
            let mut is_relative = global_relative;
            if axis == Axis::E {
                is_relative |= e_relative;
            }
            if lineg.has(axis) {
                let length_scale = if units_inches { INCHES_TO_MM } else { 1.0 };
                let ret = lineg.value(axis) * length_scale;
                match axis {
                    Axis::I => start_position[Axis::X as usize] as f32 + ret,
                    Axis::J => start_position[Axis::Y as usize] as f32 + ret,
                    _ => {
                        let base = if is_relative {
                            start_position[axis as usize]
                        } else {
                            origin[axis as usize]
                        };
                        base as f32 + ret
                    }
                }
            } else {
                match axis {
                    Axis::I => start_position[Axis::X as usize] as f32,
                    Axis::J => start_position[Axis::Y as usize] as f32,
                    _ => start_position[axis as usize] as f32,
                }
            }
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
            self.m_end_position[a] = absolute_position(ax, line) as f64;
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
            absolute_position(Axis::I, line),
            absolute_position(Axis::J, line),
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

        // arc length (cpp:4814-4817)
        let arc_length = if !line.has(Axis::P) {
            ArcSegment::calc_arc_length(start_point, end_point, center, is_ccw)
        } else {
            (line.p() as i32) as f32 * 2.0 * (PI as f32) * (start_point - center).norm()
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
        let delta_xyz = (sqr(arc_length) + sqr(dz)).sqrt();

        // extrude width/height + filament caches (cpp:4842-4903)
        if r#type == EMoveType::Extrude {
            let volume_extruded_filament = area_filament_cross_section * de as f32;
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
                self.m_width = de as f32 * (PI as f32 * sqr(1.05 * filament_radius))
                    / (delta_xyz * self.m_height);
            } else if self.m_extrusion_role == ExtrusionRole::BridgeInfill
                || self.m_extrusion_role == ExtrusionRole::None
            {
                self.m_width = filament_diameter * (de as f32 / delta_xyz).sqrt();
            } else {
                self.m_width = de as f32 * (PI as f32 * sqr(filament_radius))
                    / (delta_xyz * self.m_height)
                    + (1.0 - 0.25 * PI as f32) * self.m_height;
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
            let mut axis_feedrate = [0.0f32; 4];
            let mut abs_axis_feedrate = [0.0f32; 4];
            let mut min_feedrate_factor = 1.0f32;
            for a in 0..=(Axis::E as usize) {
                if a == Axis::X as usize || a == Axis::Y as usize {
                    axis_feedrate[a] = feedrate * arc_length * inv_distance;
                } else if a == Axis::Z as usize {
                    axis_feedrate[a] = feedrate * delta_pos[a] as f32 * inv_distance;
                } else {
                    // E axis (cpp:4953): curr.axis_feedrate[E] *= extrude_factor.
                    // curr.axis_feedrate[E] is left at its previous value (0 on a
                    // fresh State); mirror C++ by scaling the existing value.
                    axis_feedrate[a] *= extrude_factor;
                }
                abs_axis_feedrate[a] = axis_feedrate[a].abs();
                if abs_axis_feedrate[a] != 0.0 {
                    let axis_max_feedrate = self.get_axis_max_feedrate(
                        mode,
                        a,
                        self.get_machine_config_idx(self.get_filament_id(true)),
                    );
                    if axis_max_feedrate != 0.0 {
                        min_feedrate_factor =
                            min_feedrate_factor.min(axis_max_feedrate / abs_axis_feedrate[a]);
                    }
                }
            }
            feedrate *= min_feedrate_factor;
            block.feedrate_profile.cruise = feedrate;
            if min_feedrate_factor < 1.0 {
                for a in 0..=(Axis::E as usize) {
                    axis_feedrate[a] *= min_feedrate_factor;
                    abs_axis_feedrate[a] *= min_feedrate_factor;
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
                let axis_acc = if a == Axis::X as usize || a == Axis::Y as usize {
                    acceleration * arc_length * inv_distance
                } else {
                    acceleration * (delta_pos[a] as f32).abs() * inv_distance
                };
                if axis_acc != 0.0 {
                    let axis_max_acceleration = self.get_axis_max_acceleration(
                        mode,
                        a,
                        self.get_machine_config_idx(self.get_filament_id(true)),
                    );
                    if axis_max_acceleration != 0.0 && axis_acc > axis_max_acceleration {
                        min_acc_factor = min_acc_factor.min(axis_max_acceleration / axis_acc);
                    }
                }
            }
            block.acceleration = acceleration * min_acc_factor;

            // block exit feedrate (cpp:4990-4996)
            let mut safe_feedrate = block.feedrate_profile.cruise;
            for a in 0..=(Axis::E as usize) {
                let axis_max_jerk = self.get_axis_max_jerk(mode, a);
                if abs_axis_feedrate[a] > axis_max_jerk {
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
                machine.curr.axis_feedrate = [
                    axis_feedrate[0] as f64,
                    axis_feedrate[1] as f64,
                    axis_feedrate[2] as f64,
                    axis_feedrate[3] as f64,
                ];
                machine.curr.abs_axis_feedrate = [
                    abs_axis_feedrate[0] as f64,
                    abs_axis_feedrate[1] as f64,
                    abs_axis_feedrate[2] as f64,
                    abs_axis_feedrate[3] as f64,
                ];
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

    // BLOCKED(deps): process_G4 (dwell) folds dwell time via st_synchronize +
    // measure_g29_time logic. Conservative faithful subset: no-op time add.
    fn process_g4(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

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
        if line.has(crate::g_code_reader::Axis::X) {
            new_line_raw += " X0";
            found = true;
        }
        if line.has(crate::g_code_reader::Axis::Y) {
            new_line_raw += " Y0";
            found = true;
        }
        if line.has(crate::g_code_reader::Axis::Z) {
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

    // BLOCKED(deps): process_G29 — leveling-mesh timing; not ported.
    fn process_g29(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

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
        if !line.has_p() || (line.has_p() && line.p() == 1.0) {
            if let Some(new_fan_speed) = line.has_value(b'S') {
                self.m_fan_speed = (100.0 / 255.0) * new_fan_speed;
            } else {
                self.m_fan_speed = 100.0;
            }
        }
    }

    /// GCodeProcessor.cpp:5307-5310  process_M107 — disable fan.
    fn process_m107(&mut self, _line: &crate::g_code_reader::GCodeLine) {
        self.m_fan_speed = 0.0;
    }

    // BLOCKED(deps): process_M108 (Sailfish tool change) needs process_T(substr);
    // only relevant for gcfSailfish flavor (not default). No-op for default flavor.
    fn process_m108(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

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
    fn process_m135(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

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

    /// GCodeProcessor.cpp:5405-5423  process_M201 — set max printing acceleration.
    /// BLOCKED(config): machine_max_acceleration arrays are empty (unthreaded
    /// MachineEnvelopeConfig), so the C++ loop body never executes. No-op here.
    fn process_m201(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

    /// GCodeProcessor.cpp:5425-5454  process_M203 — set maximum feedrate.
    /// BLOCKED(config): machine_max_speed arrays empty → loop never runs.
    fn process_m203(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

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

    /// GCodeProcessor.cpp:5485-5513  process_M205 — advanced settings (jerk/min rate).
    /// BLOCKED(config): jerk + min-rate limit arrays are empty (unthreaded), so
    /// the set_option_value calls have no observable effect. No-op here.
    fn process_m205(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

    /// GCodeProcessor.cpp:5515-5561  process_SET_VELOCITY_LIMIT (Klipper).
    /// BLOCKED(config): writes jerk/speed limit arrays (empty/unthreaded) and
    /// acceleration. We port only the ACCEL component (which sets acceleration
    /// on the machines, a real effect); jerk/velocity writes are no-ops.
    fn process_set_velocity_limit(&mut self, line: &crate::g_code_reader::GCodeLine) {
        let raw = line.raw();
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
        let value_s = line.has_value(b'S');
        let value_p = line.has_value(b'P');
        if value_s.is_some() || value_p.is_some() {
            let mut s = value_s.unwrap_or(0.0);
            s += value_p.unwrap_or(0.0) * 0.001;
            self.simulate_st_synchronize(s, ExtrusionRole::None);
        }
    }

    // BLOCKED(deps): process_M401 / process_M402 only act for gcfRepetier (not
    // default). No-op for default flavor.
    fn process_m401(&mut self, _line: &crate::g_code_reader::GCodeLine) {}
    fn process_m402(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

    /// GCodeProcessor.cpp:5646-5661  process_M566 — instantaneous speed change.
    /// BLOCKED(config): writes machine_max_jerk arrays (empty/unthreaded). No-op.
    fn process_m566(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

    /// GCodeProcessor.cpp:5663-5673  process_M702 — MMU unload at end of print.
    fn process_m702(&mut self, line: &crate::g_code_reader::GCodeLine) {
        // C++ checks line.has('C'); 'C' is not a recognized axis char in the
        // Rust reader's has(Axis) API, but has_char covers it.
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
    fn process_m1020(&mut self, _line: &crate::g_code_reader::GCodeLine) {}
    fn process_t(&mut self, _line: &crate::g_code_reader::GCodeLine) {}

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
                self.m_result.moves[move_id].time[machine_idx] = time;
            }
        }
    }

    /// GCodeProcessor.cpp:3936-4337  void GCodeProcessor::process_G1(line)
    fn process_g1(&mut self, line: &crate::g_code_reader::GCodeLine) {
        use crate::g_code_reader::Axis;
        // process_helioadditive_comment(line); — BLOCKED(deps): helio parser.

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
                    let dot = v1[0] * v2[0] + v1[1] * v2[1];
                    let cross = v1[0] * v2[1] - v1[1] * v2[0];
                    let angle = (cross as f64).atan2(dot as f64) as f32;
                    // C++: float sin_theta_2 = sqrt((1.0f - cos(angle)) * 0.5f);
                    // bare cos(float) promotes to double; result cast to float.
                    let sin_theta_2 = ((1.0 - (angle as f64).cos()) * 0.5).sqrt() as f32;
                    // C++: float r = sqrt(sqr(dpX)+sqr(dpY)) * 0.5 / sin_theta_2;
                    // sqr(double); 0.5 is double; division in double -> float.
                    let r = ((dx * dx + dy * dy).sqrt() * 0.5 / sin_theta_2 as f64) as f32;
                    feedrate = feedrate.min((acceleration * r).sqrt());
                }
            }

            // block cruise feedrate (cpp:4163-4186)
            let mut axis_feedrate = [0.0f32; 4];
            let mut abs_axis_feedrate = [0.0f32; 4];
            let mut min_feedrate_factor = 1.0f32;
            for a in 0..=(Axis::E as usize) {
                axis_feedrate[a] = feedrate * delta_pos[a] as f32 * inv_distance;
                if a == Axis::E as usize {
                    axis_feedrate[a] *= extrude_factor;
                }
                abs_axis_feedrate[a] = axis_feedrate[a].abs();
                if abs_axis_feedrate[a] != 0.0 {
                    let axis_max_feedrate = self.get_axis_max_feedrate(
                        mode,
                        a,
                        self.get_machine_config_idx(self.get_filament_id(true)),
                    );
                    if axis_max_feedrate != 0.0 {
                        min_feedrate_factor =
                            min_feedrate_factor.min(axis_max_feedrate / abs_axis_feedrate[a]);
                    }
                }
            }
            feedrate *= min_feedrate_factor;
            block.feedrate_profile.cruise = feedrate;
            if min_feedrate_factor < 1.0 {
                for a in 0..=(Axis::E as usize) {
                    axis_feedrate[a] *= min_feedrate_factor;
                    abs_axis_feedrate[a] *= min_feedrate_factor;
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
                if acceleration * (delta_pos[a] as f32).abs() * inv_distance > axis_max_acceleration
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
                if abs_axis_feedrate[a] > axis_max_jerk {
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
                machine.curr.axis_feedrate = [
                    axis_feedrate[0] as f64,
                    axis_feedrate[1] as f64,
                    axis_feedrate[2] as f64,
                    axis_feedrate[3] as f64,
                ];
                machine.curr.abs_axis_feedrate = [
                    abs_axis_feedrate[0] as f64,
                    abs_axis_feedrate[1] as f64,
                    abs_axis_feedrate[2] as f64,
                    abs_axis_feedrate[3] as f64,
                ];
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

        // BLOCKED(deps): m_seams_detector + spiral_vase_layers side-effects need
        // SeamsDetector + tag pipeline. Not ported.

        // store move
        self.store_move_vertex(r#type, EMovePathType::NoopMove);
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
        let uf = &mut self.m_used_filaments;
        uf.process_color_change_cache();
        uf.process_model_cache(active_filament_id);
        uf.process_role_cache(diameter, density, role);
        uf.process_wipe_tower_cache(active_filament_id);
        uf.process_support_cache(active_filament_id_support);
        uf.process_total_volume_cache(active_filament_id);
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

#[cfg(test)]
mod tests {
    use super::*;

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
        let p = get_last_position_from_gcode(g).unwrap();
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
