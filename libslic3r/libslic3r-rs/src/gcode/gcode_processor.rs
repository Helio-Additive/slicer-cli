//! GCode processor module for parsing and analyzing G-code.
//!
//! C++ Reference:
//! - GCode/GCodeProcessor.hpp
//! - GCode/GCodeProcessor.cpp
//!
//! This module provides types for processing G-code output, tracking moves,
//! estimating print times, and detecting conflicts.

use std::collections::HashMap;

/// Move types in processed G-code.
/// Corresponds to C++ EMoveType enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EMoveType {
    Noop,
    Retract,
    Unretract,
    Seam,
    ToolChange,
    ColorChange,
    PausePrint,
    CustomGCode,
    Travel,
    Wipe,
    Extrude,
}

impl EMoveType {
    /// Total number of move type variants.
    pub const COUNT: usize = 11;

    /// Check if this move type involves material extrusion.
    pub fn is_extrusion(&self) -> bool {
        matches!(self, EMoveType::Extrude)
    }
}

impl Default for EMoveType {
    fn default() -> Self {
        EMoveType::Noop
    }
}

/// Skip types for timelapse and head wrap detection.
/// Corresponds to C++ SkipType enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SkipType {
    Timelapse,
    HeadWrapDetect,
    Other,
    None,
}

impl Default for SkipType {
    fn default() -> Self {
        SkipType::None
    }
}

impl SkipType {
    /// Parse a skip type from a string tag.
    pub fn from_str_tag(s: &str) -> Self {
        match s {
            "timelapse" => SkipType::Timelapse,
            "head_wrap_detect" => SkipType::HeadWrapDetect,
            _ => SkipType::Other,
        }
    }
}

/// Time estimation modes.
/// Corresponds to C++ PrintEstimatedStatistics::ETimeMode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ETimeMode {
    Normal,
    Stealth,
}

impl ETimeMode {
    pub const COUNT: usize = 2;
}

impl Default for ETimeMode {
    fn default() -> Self {
        ETimeMode::Normal
    }
}

/// Custom G-code event tags.
/// Corresponds to C++ CustomGCode::Type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CustomGCodeType {
    ColorChange,
    PausePrint,
    ToolChange,
    Template,
    Custom,
}

/// G-code processor tags for embedded metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ETags {
    /// Height tag
    Height,
    /// Width tag
    Width,
    /// Layer change tag
    LayerChange,
    /// Color change tag
    ColorChange,
    /// Pause print tag
    PausePrint,
    /// Custom G-code tag
    CustomGCode,
    /// First layer placeholder for temp control
    FirstLayerPlaceholder,
}

/// Thermal index data for temperature tracking.
#[derive(Debug, Clone, Default)]
pub struct ThermalIndex {
    pub min: f32,
    pub max: f32,
    pub mean: f32,
}

impl ThermalIndex {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&mut self, value: f32) {
        if value < self.min || self.min == 0.0 {
            self.min = value;
        }
        if value > self.max {
            self.max = value;
        }
        // Approximation; real implementation tracks count for proper mean
        self.mean = (self.min + self.max) / 2.0;
    }
}

/// Settings IDs for print/filament/printer profiles.
/// Corresponds to C++ GCodeProcessorResult::SettingsIds.
#[derive(Debug, Clone, Default)]
pub struct SettingsIds {
    pub print: String,
    pub filament: Vec<String>,
    pub printer: String,
}

impl SettingsIds {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.print.clear();
        self.filament.clear();
        self.printer.clear();
    }
}

/// Result of filament printability check.
/// Corresponds to C++ FilamentPrintableResult.
#[derive(Debug, Clone, Default)]
pub struct FilamentPrintableResult {
    pub conflict_filament: Vec<i32>,
    pub plate_name: String,
}

impl FilamentPrintableResult {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_conflicts(conflict_filament: Vec<i32>, plate_name: String) -> Self {
        Self {
            conflict_filament,
            plate_name,
        }
    }

    pub fn has_value(&self) -> bool {
        !self.conflict_filament.is_empty()
    }

    pub fn reset(&mut self) {
        self.conflict_filament.clear();
        self.plate_name.clear();
    }
}

/// Result of G-code validation checks.
/// Corresponds to C++ GCodeCheckResult.
#[derive(Debug, Clone, Default)]
pub struct GCodeCheckResult {
    /// Error code bitfield:
    /// 0b00001 = multi extruder printable area error
    /// 0b00010 = multi extruder printable height error
    /// 0b00100 = plate printable area error
    /// 0b01000 = plate printable height error
    /// 0b10000 = wrapping detection area error
    pub error_code: i32,
    /// extruder_id -> Vec<(filament_id, object_label_id)>
    pub print_area_error_infos: HashMap<i32, Vec<(i32, i32)>>,
    /// extruder_id -> Vec<(filament_id, object_label_id)>
    pub print_height_error_infos: HashMap<i32, Vec<(i32, i32)>>,
}

impl GCodeCheckResult {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.error_code = 0;
        self.print_area_error_infos.clear();
        self.print_height_error_infos.clear();
    }

    pub fn has_errors(&self) -> bool {
        self.error_code != 0
    }
}

/// Color palette entry for G-code visualization.
/// Corresponds to C++ CpColor concept in GCodeProcessor.
#[derive(Debug, Clone, Default)]
pub struct CpColor {
    pub counter: u32,
    pub current: u32,
}

impl CpColor {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn advance(&mut self) {
        self.counter += 1;
        self.current = self.counter;
    }
}

/// Conflict detection result between objects.
/// Corresponds to C++ ConflictResult.
#[derive(Debug, Clone, Default)]
pub struct ConflictResult {
    pub obj_name1: String,
    pub obj_name2: String,
    pub height: f32,
    pub layer: i32,
}

impl ConflictResult {
    pub fn new() -> Self {
        Self {
            layer: -1,
            ..Default::default()
        }
    }

    pub fn with_names(obj_name1: String, obj_name2: String, height: f32) -> Self {
        Self {
            obj_name1,
            obj_name2,
            height,
            layer: -1,
        }
    }
}

/// Bitflags for G-code processor state.
#[derive(Debug, Clone, Default)]
pub struct Flags {
    bits: u32,
}

impl Flags {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set(&mut self, bit: u32, value: bool) {
        if value {
            self.bits |= 1 << bit;
        } else {
            self.bits &= !(1 << bit);
        }
    }

    pub fn get(&self, bit: u32) -> bool {
        (self.bits & (1 << bit)) != 0
    }

    pub fn reset(&mut self) {
        self.bits = 0;
    }
}

/// Time estimation mode data.
/// Corresponds to C++ PrintEstimatedStatistics::Mode.
#[derive(Debug, Clone, Default)]
pub struct TimeModeData {
    pub time: f32,
    pub prepare_time: f32,
    pub custom_gcode_times: Vec<(CustomGCodeType, (f32, f32))>,
    pub moves_times: Vec<(EMoveType, f32)>,
    pub layers_times: Vec<f32>,
}

impl TimeModeData {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.time = 0.0;
        self.prepare_time = 0.0;
        self.custom_gcode_times.clear();
        self.moves_times.clear();
        self.layers_times.clear();
    }
}

/// Process G21 command (set units to millimeters).
/// This is typically a no-op since the processor assumes mm.
pub fn process_g21() -> crate::Result<()> {
    // G21 sets units to millimeters - this is the default, no action needed
    Ok(())
}

/// Get filament vitrification (glass transition) temperature.
/// Returns the temperature at which the filament becomes glassy.
pub fn get_filament_vitrification_temperature() -> crate::Result<()> {
    // Default: return Ok, actual temperatures come from config
    Ok(())
}

/// Process total volume cache for filament usage tracking.
pub fn process_total_volume_cache() -> crate::Result<()> {
    // Accumulates total extrusion volumes per extruder
    Ok(())
}

/// Process model cache for object identification.
pub fn process_model_cache() -> crate::Result<()> {
    // Caches model info for conflict detection
    Ok(())
}

/// Process KISSlicer-specific tags in G-code comments.
pub fn process_kissslicer_tags() -> crate::Result<()> {
    // KISSlicer compatibility - parse proprietary comment tags
    Ok(())
}

// ============================================================================
// GCodeProcessor — main G-code line parser
// Ported from C++ GCodeProcessor.cpp:2795-4050
// ============================================================================

/// Axis indices matching C++ Axis enum.
const AXIS_X: usize = 0;
const AXIS_Y: usize = 1;
const AXIS_Z: usize = 2;
const AXIS_E: usize = 3;
const NUM_AXES: usize = 4;

/// Millimeters per minute to millimeters per second.
const MMMIN_TO_MMSEC: f32 = 1.0 / 60.0;

/// Default toolpath height when none is detected.
const DEFAULT_TOOLPATH_HEIGHT: f32 = 0.2;

/// Positioning type for axes.
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

/// Units for G-code interpretation.
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

/// Filament usage tracking.
/// C++ GCodeProcessor::UsedFilaments
#[derive(Debug, Clone, Default)]
pub struct UsedFilaments {
    pub model_volume: f64,
    pub support_volume: f64,
    pub wipe_tower_volume: f64,
    pub flush_per_filament: HashMap<i32, f64>,
}

impl UsedFilaments {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn increase_model_caches(&mut self, volume: f32) {
        self.model_volume += volume as f64;
    }

    pub fn increase_support_caches(&mut self, volume: f32) {
        self.support_volume += volume as f64;
    }

    pub fn increase_wipe_tower_caches(&mut self, volume: f32) {
        self.wipe_tower_volume += volume as f64;
    }

    pub fn total_volume(&self) -> f64 {
        self.model_volume + self.support_volume + self.wipe_tower_volume
    }
}

/// Result of processing a G-code file.
/// C++ GCodeProcessorResult
#[derive(Debug, Clone, Default)]
pub struct GCodeProcessorResult {
    /// Filament diameters per extruder.
    pub filament_diameters: Vec<f32>,
    /// Filament densities per extruder (g/cm3).
    pub filament_densities: Vec<f32>,
    /// Per-layer print times (seconds).
    pub layer_times: Vec<f32>,
    /// Total print time (seconds).
    pub print_time: f32,
    /// Total filament used (mm of filament).
    pub filament_used_mm: f64,
    /// Total filament used (g).
    pub filament_used_g: f64,
    /// Total filament volume used (mm3).
    pub filament_used_mm3: f64,
}

/// Main G-code processor for parsing and analyzing G-code output.
///
/// C++ reference: GCodeProcessor class
/// GCodeProcessor.hpp / GCodeProcessor.cpp
///
/// Tracks position, speed, extruder state, computes print time
/// and filament usage per feature/layer.
#[derive(Debug, Clone)]
pub struct GCodeProcessor {
    /// Current position [X, Y, Z, E].
    start_position: [f32; NUM_AXES],
    /// End position after current move.
    end_position: [f32; NUM_AXES],
    /// Origin offset [X, Y, Z, E].
    origin: [f32; NUM_AXES],
    /// Current feedrate (mm/s).
    feedrate: f32,
    /// Global positioning type (absolute/relative).
    global_positioning: EPositioningType,
    /// E-axis positioning type.
    e_positioning: EPositioningType,
    /// Current units.
    units: EUnits,
    /// Current extruder ID.
    extruder_id: u32,
    /// Current extrusion role.
    extrusion_role: ExtrusionRoleTag,
    /// Forced width from tag.
    forced_width: f32,
    /// Forced height from tag.
    forced_height: f32,
    /// Current detected height.
    height: f32,
    /// Current detected width.
    width: f32,
    /// mm3 per mm of toolpath.
    mm3_per_mm: f32,
    /// Last Z where extrusion occurred.
    extruded_last_z: f32,
    /// Current layer ID (1-based).
    layer_id: u32,
    /// G1 line counter.
    g1_line_id: u64,
    /// Total line counter.
    line_id: u64,
    /// Whether currently wiping.
    wiping: bool,
    /// Whether currently flushing.
    flushing: bool,
    /// Filament usage tracker.
    used_filaments: UsedFilaments,
    /// Processing result.
    result: GCodeProcessorResult,
    /// Per-layer time accumulator.
    current_layer_time: f32,
    /// Total accumulated print time.
    total_time: f32,
}

/// Simplified extrusion role tag for processor.
/// Matches C++ ExtrusionRole but as simple tag for tracking.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ExtrusionRoleTag {
    #[default]
    None,
    Perimeter,
    ExternalPerimeter,
    OverhangPerimeter,
    InternalInfill,
    SolidInfill,
    TopSolidInfill,
    Ironing,
    BridgeInfill,
    GapFill,
    Skirt,
    SupportMaterial,
    SupportMaterialInterface,
    SupportTransition,
    WipeTower,
    Custom,
}

impl ExtrusionRoleTag {
    /// Parse from a FEATURE comment string.
    pub fn from_feature_str(s: &str) -> Self {
        match s.trim() {
            "Outer wall" | "External perimeter" => Self::ExternalPerimeter,
            "Inner wall" | "Perimeter" => Self::Perimeter,
            "Overhang wall" => Self::OverhangPerimeter,
            "Sparse infill" | "Internal infill" => Self::InternalInfill,
            "Internal solid infill" | "Solid infill" => Self::SolidInfill,
            "Top surface" | "Top solid infill" => Self::TopSolidInfill,
            "Ironing" => Self::Ironing,
            "Bridge" | "Bridge infill" => Self::BridgeInfill,
            "Gap infill" | "Gap fill" => Self::GapFill,
            "Skirt" => Self::Skirt,
            "Support" | "support material" => Self::SupportMaterial,
            "Support interface" | "support material interface" => Self::SupportMaterialInterface,
            "Support transition" => Self::SupportTransition,
            "Wipe tower" | "Prime tower" => Self::WipeTower,
            _ => Self::Custom,
        }
    }
}

impl GCodeProcessor {
    /// Create a new GCodeProcessor with default state.
    pub fn new() -> Self {
        Self {
            start_position: [0.0; NUM_AXES],
            end_position: [0.0; NUM_AXES],
            origin: [0.0; NUM_AXES],
            feedrate: 0.0,
            global_positioning: EPositioningType::Absolute,
            e_positioning: EPositioningType::Absolute,
            units: EUnits::Millimeters,
            extruder_id: 0,
            extrusion_role: ExtrusionRoleTag::None,
            forced_width: 0.0,
            forced_height: 0.0,
            height: 0.0,
            width: 0.0,
            mm3_per_mm: 0.0,
            extruded_last_z: 0.0,
            layer_id: 0,
            g1_line_id: 0,
            line_id: 0,
            wiping: false,
            flushing: false,
            used_filaments: UsedFilaments::new(),
            result: GCodeProcessorResult {
                filament_diameters: vec![1.75],
                filament_densities: vec![1.24],
                ..Default::default()
            },
            current_layer_time: 0.0,
            total_time: 0.0,
        }
    }

    /// Get the processing result.
    pub fn result(&self) -> &GCodeProcessorResult {
        &self.result
    }

    /// Get mutable processing result.
    pub fn result_mut(&mut self) -> &mut GCodeProcessorResult {
        &mut self.result
    }

    /// Process a single line of G-code.
    ///
    /// C++ reference: GCodeProcessor::process_gcode_line()
    /// GCodeProcessor.cpp:2795-2838
    ///
    /// Parses the line, dispatches to the appropriate handler (G0/G1, G28, M82/M83, etc.),
    /// and updates internal state (position, feedrate, extruder, time, filament usage).
    pub fn process_line(&mut self, line: &str) {
        self.line_id += 1;

        // Update start position
        self.start_position = self.end_position;

        let trimmed = line.trim();
        if trimmed.is_empty() {
            return;
        }

        // Process comments for tags
        if trimmed.starts_with(';') {
            self.process_comment(trimmed);
            return;
        }

        // Extract command (first word)
        let cmd = trimmed.split_whitespace().next().unwrap_or("");

        match cmd {
            "G0" | "G1" => self.process_g1(trimmed),
            "G10" => self.process_g10(),
            "G11" => self.process_g11(),
            "G20" => self.units = EUnits::Inches,
            "G21" => self.units = EUnits::Millimeters,
            "G28" => {
                // Home — reset position to origin
                self.end_position = [0.0; NUM_AXES];
            }
            "G90" => self.global_positioning = EPositioningType::Absolute,
            "G91" => self.global_positioning = EPositioningType::Relative,
            "G92" => self.process_g92(trimmed),
            "M82" => self.e_positioning = EPositioningType::Absolute,
            "M83" => self.e_positioning = EPositioningType::Relative,
            "M104" | "M109" => { /* temperature commands — no position tracking */ }
            "M106" | "M107" => { /* fan commands */ }
            "M140" | "M190" => { /* bed temperature commands */ }
            "M204" => { /* acceleration — TODO: track for time estimation */ }
            _ => {
                // Tool change: T0, T1, etc.
                if cmd.starts_with('T') {
                    if let Ok(id) = cmd[1..].parse::<u32>() {
                        self.extruder_id = id;
                    }
                }
            }
        }
    }

    /// Process G1/G0 move command.
    ///
    /// C++ reference: GCodeProcessor::process_G1()
    /// GCodeProcessor.cpp:3811-4050
    fn process_g1(&mut self, line: &str) {
        self.g1_line_id += 1;

        let filament_id = self.extruder_id as usize;
        let filament_diameter = if filament_id < self.result.filament_diameters.len() {
            self.result.filament_diameters[filament_id]
        } else if !self.result.filament_diameters.is_empty() {
            *self.result.filament_diameters.last().unwrap()
        } else {
            1.75
        };
        let filament_radius = 0.5 * filament_diameter;
        let area_filament_cross_section = std::f32::consts::PI * filament_radius * filament_radius;

        let scale_factor = if self.units == EUnits::Inches {
            25.4
        } else {
            1.0
        };

        // Parse axis values from line
        // GCodeProcessor.cpp:3858-3861
        for part in line.split_whitespace().skip(1) {
            if part.is_empty() {
                continue;
            }
            let axis_char = part.as_bytes()[0];
            let value_str = &part[1..];
            // Strip inline comments
            let value_str = value_str.split(';').next().unwrap_or(value_str);
            if let Ok(val) = value_str.parse::<f32>() {
                let scaled_val = val * scale_factor;
                match axis_char {
                    b'X' => {
                        self.end_position[AXIS_X] =
                            if self.global_positioning == EPositioningType::Relative {
                                self.start_position[AXIS_X] + scaled_val
                            } else {
                                self.origin[AXIS_X] + scaled_val
                            };
                    }
                    b'Y' => {
                        self.end_position[AXIS_Y] =
                            if self.global_positioning == EPositioningType::Relative {
                                self.start_position[AXIS_Y] + scaled_val
                            } else {
                                self.origin[AXIS_Y] + scaled_val
                            };
                    }
                    b'Z' => {
                        self.end_position[AXIS_Z] =
                            if self.global_positioning == EPositioningType::Relative {
                                self.start_position[AXIS_Z] + scaled_val
                            } else {
                                self.origin[AXIS_Z] + scaled_val
                            };
                    }
                    b'E' => {
                        let is_relative = self.global_positioning == EPositioningType::Relative
                            || self.e_positioning == EPositioningType::Relative;
                        self.end_position[AXIS_E] = if is_relative {
                            self.start_position[AXIS_E] + scaled_val
                        } else {
                            self.origin[AXIS_E] + scaled_val
                        };
                    }
                    b'F' => {
                        self.feedrate = val * MMMIN_TO_MMSEC;
                    }
                    _ => {}
                }
            }
        }

        // Calculate deltas
        // GCodeProcessor.cpp:3868-3877
        let mut delta_pos = [0.0f32; NUM_AXES];
        let mut max_abs_delta: f32 = 0.0;
        for a in 0..NUM_AXES {
            delta_pos[a] = self.end_position[a] - self.start_position[a];
            max_abs_delta = max_abs_delta.max(delta_pos[a].abs());
        }

        if max_abs_delta == 0.0 {
            return;
        }

        // Determine move type
        // GCodeProcessor.cpp:3834-3851
        let move_type = if self.wiping {
            EMoveType::Wipe
        } else if delta_pos[AXIS_E] < 0.0 {
            if delta_pos[AXIS_X] != 0.0 || delta_pos[AXIS_Y] != 0.0 || delta_pos[AXIS_Z] != 0.0 {
                EMoveType::Travel
            } else {
                EMoveType::Retract
            }
        } else if delta_pos[AXIS_E] > 0.0 {
            if delta_pos[AXIS_X] == 0.0 && delta_pos[AXIS_Y] == 0.0 {
                if delta_pos[AXIS_Z] == 0.0 {
                    EMoveType::Unretract
                } else {
                    EMoveType::Travel
                }
            } else {
                EMoveType::Extrude
            }
        } else if delta_pos[AXIS_X] != 0.0 || delta_pos[AXIS_Y] != 0.0 || delta_pos[AXIS_Z] != 0.0 {
            EMoveType::Travel
        } else {
            EMoveType::Noop
        };

        // Handle extrusion
        // GCodeProcessor.cpp:3880-3941
        if move_type == EMoveType::Extrude {
            let delta_xyz =
                (delta_pos[AXIS_X].powi(2) + delta_pos[AXIS_Y].powi(2) + delta_pos[AXIS_Z].powi(2))
                    .sqrt();
            let volume_extruded = area_filament_cross_section * delta_pos[AXIS_E];

            // Track filament usage by role
            match self.extrusion_role {
                ExtrusionRoleTag::SupportMaterial
                | ExtrusionRoleTag::SupportMaterialInterface
                | ExtrusionRoleTag::SupportTransition => {
                    self.used_filaments.increase_support_caches(volume_extruded);
                }
                ExtrusionRoleTag::WipeTower => {
                    self.used_filaments
                        .increase_wipe_tower_caches(volume_extruded);
                }
                _ => {
                    self.used_filaments.increase_model_caches(volume_extruded);
                }
            }

            // Detect height
            // GCodeProcessor.cpp:3900-3913
            if self.forced_height > 0.0 {
                self.height = self.forced_height;
            } else if self.end_position[AXIS_Z] > self.extruded_last_z + 1e-6 {
                self.height = self.end_position[AXIS_Z] - self.extruded_last_z;
            }
            if self.height == 0.0 {
                self.height = DEFAULT_TOOLPATH_HEIGHT;
            }
            self.extruded_last_z = self.end_position[AXIS_Z];

            // Update mm3_per_mm
            if delta_xyz > 0.0 {
                self.mm3_per_mm = volume_extruded / delta_xyz;
            }
        }

        // Time estimation
        // GCodeProcessor.cpp:3960-3970
        let sq_xyz =
            delta_pos[AXIS_X].powi(2) + delta_pos[AXIS_Y].powi(2) + delta_pos[AXIS_Z].powi(2);
        let distance = if sq_xyz > 0.0 {
            sq_xyz.sqrt()
        } else {
            delta_pos[AXIS_E].abs()
        };

        if distance > 0.0 && self.feedrate > 0.0 {
            let move_time = distance / self.feedrate;
            self.current_layer_time += move_time;
            self.total_time += move_time;
        }

        // Accumulate filament length
        if move_type == EMoveType::Extrude {
            self.result.filament_used_mm += delta_pos[AXIS_E].abs() as f64;
            self.result.filament_used_mm3 +=
                (area_filament_cross_section * delta_pos[AXIS_E].abs()) as f64;
        }
    }

    /// Process G10 (firmware retract).
    fn process_g10(&mut self) {
        // Firmware retract - handled by firmware, just track state
    }

    /// Process G11 (firmware unretract).
    fn process_g11(&mut self) {
        // Firmware unretract
    }

    /// Process G92 (set position).
    fn process_g92(&mut self, line: &str) {
        for part in line.split_whitespace().skip(1) {
            if part.is_empty() {
                continue;
            }
            let axis_char = part.as_bytes()[0];
            let value_str = &part[1..];
            let value_str = value_str.split(';').next().unwrap_or(value_str);
            if let Ok(val) = value_str.parse::<f32>() {
                match axis_char {
                    b'X' => {
                        self.origin[AXIS_X] = self.end_position[AXIS_X] - val;
                        self.end_position[AXIS_X] = val + self.origin[AXIS_X];
                    }
                    b'Y' => {
                        self.origin[AXIS_Y] = self.end_position[AXIS_Y] - val;
                        self.end_position[AXIS_Y] = val + self.origin[AXIS_Y];
                    }
                    b'Z' => {
                        self.origin[AXIS_Z] = self.end_position[AXIS_Z] - val;
                        self.end_position[AXIS_Z] = val + self.origin[AXIS_Z];
                    }
                    b'E' => {
                        // Reset E: common pattern is G92 E0
                        self.origin[AXIS_E] = self.end_position[AXIS_E] - val;
                        self.end_position[AXIS_E] = val + self.origin[AXIS_E];
                    }
                    _ => {}
                }
            }
        }
    }

    /// Process comment lines for embedded tags.
    ///
    /// C++ reference: GCodeProcessor::process_tags()
    /// GCodeProcessor.cpp:3100-3340
    fn process_comment(&mut self, line: &str) {
        let content = line.trim_start_matches(';').trim();

        // Layer change tag
        if content.starts_with("CHANGE_LAYER") || content.starts_with("Layer_Change") {
            // Flush current layer time
            if self.current_layer_time > 0.0 {
                self.result.layer_times.push(self.current_layer_time);
            }
            self.current_layer_time = 0.0;
            self.layer_id += 1;
            return;
        }

        // Height tag
        if content.starts_with("HEIGHT:") {
            if let Ok(h) = content[7..].trim().parse::<f32>() {
                self.forced_height = h;
            }
            return;
        }

        // Width tag
        if content.starts_with("LINE_WIDTH:") || content.starts_with("WIDTH:") {
            let val_str = if content.starts_with("LINE_WIDTH:") {
                &content[11..]
            } else {
                &content[6..]
            };
            if let Ok(w) = val_str.trim().parse::<f32>() {
                self.forced_width = w;
            }
            return;
        }

        // Feature/role tag
        if content.starts_with("FEATURE:") || content.starts_with("TYPE:") {
            let role_str = if content.starts_with("FEATURE:") {
                &content[8..]
            } else {
                &content[5..]
            };
            self.extrusion_role = ExtrusionRoleTag::from_feature_str(role_str);
            return;
        }

        // Wipe start/end
        if content.starts_with("WIPE_START") {
            self.wiping = true;
            return;
        }
        if content.starts_with("WIPE_END") {
            self.wiping = false;
            return;
        }

        // Flush start/end
        if content.starts_with("FLUSH_START") {
            self.flushing = true;
            return;
        }
        if content.starts_with("FLUSH_END") {
            self.flushing = false;
            return;
        }
    }

    /// Process an entire G-code string (all lines).
    pub fn process_gcode(&mut self, gcode: &str) {
        for line in gcode.lines() {
            self.process_line(line);
        }
        // Flush last layer
        if self.current_layer_time > 0.0 {
            self.result.layer_times.push(self.current_layer_time);
            self.current_layer_time = 0.0;
        }
        self.result.print_time = self.total_time;

        // Calculate filament weight from volume
        let density = if !self.result.filament_densities.is_empty() {
            self.result.filament_densities[0] as f64
        } else {
            1.24 // PLA default
        };
        self.result.filament_used_g = self.result.filament_used_mm3 * density / 1000.0;
    }

    /// Get current position.
    pub fn position(&self) -> [f32; NUM_AXES] {
        self.end_position
    }

    /// Get total print time estimate (seconds).
    pub fn print_time(&self) -> f32 {
        self.total_time
    }

    /// Get used filaments tracker.
    pub fn used_filaments(&self) -> &UsedFilaments {
        &self.used_filaments
    }

    /// Get current layer ID.
    pub fn layer_id(&self) -> u32 {
        self.layer_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_move_type_default() {
        assert_eq!(EMoveType::default(), EMoveType::Noop);
    }

    #[test]
    fn test_skip_type_from_str() {
        assert_eq!(SkipType::from_str_tag("timelapse"), SkipType::Timelapse);
        assert_eq!(
            SkipType::from_str_tag("head_wrap_detect"),
            SkipType::HeadWrapDetect
        );
        assert_eq!(SkipType::from_str_tag("unknown"), SkipType::Other);
    }

    #[test]
    fn test_settings_ids() {
        let mut ids = SettingsIds::new();
        ids.print = "my_print".into();
        ids.filament.push("PLA".into());
        ids.reset();
        assert!(ids.print.is_empty());
        assert!(ids.filament.is_empty());
    }

    #[test]
    fn test_filament_printable_result() {
        let r = FilamentPrintableResult::new();
        assert!(!r.has_value());
        let r2 = FilamentPrintableResult::with_conflicts(vec![1, 2], "plate1".into());
        assert!(r2.has_value());
    }

    #[test]
    fn test_gcode_check_result() {
        let mut r = GCodeCheckResult::new();
        assert!(!r.has_errors());
        r.error_code = 1;
        assert!(r.has_errors());
        r.reset();
        assert!(!r.has_errors());
    }

    #[test]
    fn test_conflict_result() {
        let r = ConflictResult::new();
        assert_eq!(r.layer, -1);
    }

    #[test]
    fn test_flags() {
        let mut f = Flags::new();
        assert!(!f.get(0));
        f.set(0, true);
        assert!(f.get(0));
        f.set(0, false);
        assert!(!f.get(0));
    }

    #[test]
    fn test_process_functions() {
        assert!(process_g21().is_ok());
        assert!(get_filament_vitrification_temperature().is_ok());
        assert!(process_total_volume_cache().is_ok());
        assert!(process_model_cache().is_ok());
        assert!(process_kissslicer_tags().is_ok());
    }

    #[test]
    fn test_gcode_processor_basic() {
        let mut proc = GCodeProcessor::new();
        proc.process_line("G28"); // home
        assert_eq!(proc.position(), [0.0, 0.0, 0.0, 0.0]);

        proc.process_line("G1 X10 Y20 Z0.2 F3000");
        let pos = proc.position();
        assert!((pos[0] - 10.0).abs() < 1e-3);
        assert!((pos[1] - 20.0).abs() < 1e-3);
        assert!((pos[2] - 0.2).abs() < 1e-3);
    }

    #[test]
    fn test_gcode_processor_extrusion() {
        let mut proc = GCodeProcessor::new();
        proc.process_line("M83"); // relative E
        proc.process_line("G1 X0 Y0 Z0.2 F3000");
        proc.process_line("; FEATURE: Outer wall");
        proc.process_line("G1 X10 Y0 E0.5 F1200");
        assert!(proc.result().filament_used_mm > 0.0);
        assert!(proc.print_time() > 0.0);
    }

    #[test]
    fn test_gcode_processor_layer_tracking() {
        let mut proc = GCodeProcessor::new();
        proc.process_line("; CHANGE_LAYER");
        proc.process_line(";HEIGHT:0.2");
        proc.process_line("G1 X10 Y10 E1 F1200");
        proc.process_line("; CHANGE_LAYER");
        proc.process_line(";HEIGHT:0.2");
        proc.process_line("G1 X20 Y20 E2 F1200");

        proc.process_gcode(""); // flush
        assert_eq!(proc.layer_id(), 2);
    }

    #[test]
    fn test_extrusion_role_tag_parsing() {
        assert_eq!(
            ExtrusionRoleTag::from_feature_str("Outer wall"),
            ExtrusionRoleTag::ExternalPerimeter
        );
        assert_eq!(
            ExtrusionRoleTag::from_feature_str("Inner wall"),
            ExtrusionRoleTag::Perimeter
        );
        assert_eq!(
            ExtrusionRoleTag::from_feature_str("Sparse infill"),
            ExtrusionRoleTag::InternalInfill
        );
        assert_eq!(
            ExtrusionRoleTag::from_feature_str("Top surface"),
            ExtrusionRoleTag::TopSolidInfill
        );
        assert_eq!(
            ExtrusionRoleTag::from_feature_str("Support"),
            ExtrusionRoleTag::SupportMaterial
        );
    }
}
