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
