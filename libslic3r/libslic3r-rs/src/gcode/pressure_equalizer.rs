//! Pressure Equalizer module.
//!
//! This module implements pressure equalization for G-code, which smooths out
//! rapid changes in volumetric extrusion rate. This helps prevent pressure
//! spikes in the extruder that can cause artifacts like blobs and zits.
//!
//! # Overview
//!
//! The pressure equalizer works by:
//! 1. Parsing G-code line by line into a circular buffer
//! 2. Calculating volumetric extrusion rates for each segment
//! 3. Adjusting feedrates to limit the slope of extrusion rate changes
//! 4. Optionally splitting long segments to achieve smoother transitions
//!
//! # Algorithm
//!
//! For each extruding segment, we calculate:
//! - Volumetric extrusion rate = filament_area × feedrate × (extrusion_length / travel_length)
//!
//! We then limit how fast this rate can change by:
//! - Going backward through the buffer, limiting rate increases (negative slope)
//! - Going forward through the buffer, limiting rate decreases (positive slope)
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/GCode/PressureEqualizer.hpp`
//! - `src/libslic3r/GCode/PressureEqualizer.cpp`

use std::f32::consts::PI;

use crate::gcode::ExtrusionRole;

/// Configuration for the pressure equalizer.
#[derive(Debug, Clone)]
pub struct PressureEqualizerConfig {
    /// Filament diameter in mm. Used to calculate filament cross-section area.
    pub filament_diameter: f32,

    /// Maximum positive slope of volumetric extrusion rate (mm³/s²).
    /// Controls how fast the rate can increase.
    pub max_volumetric_rate_slope_positive: f32,

    /// Maximum negative slope of volumetric extrusion rate (mm³/s²).
    /// Controls how fast the rate can decrease.
    pub max_volumetric_rate_slope_negative: f32,

    /// Maximum segment length for splitting long segments (mm).
    pub max_segment_length: f32,

    /// Whether to use relative E distances.
    pub use_relative_e: bool,

    /// Size of the circular buffer for lookahead.
    pub buffer_size: usize,
}

impl Default for PressureEqualizerConfig {
    fn default() -> Self {
        Self {
            filament_diameter: 1.75,
            // Default slope: ~1.8 mm³/s² (from BambuStudio comments)
            // This corresponds to changing from 20mm/s to 60mm/s over 2 seconds
            // for a 0.45mm × 0.2mm extrusion
            max_volumetric_rate_slope_positive: 1.8,
            max_volumetric_rate_slope_negative: 1.8,
            max_segment_length: 20.0,
            use_relative_e: false,
            buffer_size: 100,
        }
    }
}

impl PressureEqualizerConfig {
    // Create a new configuration with the given filament diameter.
    pub fn new(filament_diameter: f32) -> Self {
        Self {
            filament_diameter,
            ..Default::default()
        }
    }

    /// Set the maximum volumetric rate slopes (both positive and negative).
    pub fn with_max_slope(mut self, slope: f32) -> Self {
        self.max_volumetric_rate_slope_positive = slope;
        self.max_volumetric_rate_slope_negative = slope;
        self
    }

    /// Set the maximum positive volumetric rate slope.
    pub fn with_max_positive_slope(mut self, slope: f32) -> Self {
        self.max_volumetric_rate_slope_positive = slope;
        self
    }

    /// Set the maximum negative volumetric rate slope.
    pub fn with_max_negative_slope(mut self, slope: f32) -> Self {
        self.max_volumetric_rate_slope_negative = slope;
        self
    }

    /// Set whether to use relative E distances.
    pub fn with_relative_e(mut self, relative: bool) -> Self {
        self.use_relative_e = relative;
        self
    }

    /// Calculate filament cross-section area.
    pub fn filament_cross_section(&self) -> f32 {
        let r = self.filament_diameter / 2.0;
        PI * r * r
    }
}

/// Type of G-code line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GCodeLineType {
    Invalid,
    Noop,
    Other,
    Retract,
    Unretract,
    ToolChange,
    Move,
    Extrude,
}

/// Per-role slope configuration.
#[derive(Debug, Clone, Copy)]
struct RoleSlope {
    positive: f32,
    negative: f32,
}

impl RoleSlope {
    fn new(positive: f32, negative: f32) -> Self {
        Self { positive, negative }
    }

    fn unlimited() -> Self {
        Self {
            positive: 0.0,
            negative: 0.0,
        }
    }
}

/// A parsed G-code line with extrusion information.
#[derive(Debug, Clone)]
struct GCodeLine {
    /// Type of this line.
    line_type: GCodeLineType,

    /// Raw G-code text.
    raw: String,

    /// Whether this line has been modified and needs to be regenerated.
    modified: bool,

    /// Position at the start of this move [X, Y, Z, E, F].
    pos_start: [f32; 5],

    /// Position at the end of this move [X, Y, Z, E, F].
    pos_end: [f32; 5],

    /// Which axes were provided on this line [X, Y, Z, E, F].
    pos_provided: [bool; 5],

    /// Current extruder ID.
    extruder_id: usize,

    /// Extrusion role for this segment.
    extrusion_role: ExtrusionRole,

    /// Volumetric extrusion rate (mm³/min).
    volumetric_rate: f32,

    /// Volumetric extrusion rate at the start of this segment.
    volumetric_rate_start: f32,

    /// Volumetric extrusion rate at the end of this segment.
    volumetric_rate_end: f32,

    /// Maximum positive slope for this segment.
    max_slope_positive: f32,

    /// Maximum negative slope for this segment.
    max_slope_negative: f32,
}

impl Default for GCodeLine {
    fn default() -> Self {
        Self {
            line_type: GCodeLineType::Invalid,
            raw: String::new(),
            modified: false,
            pos_start: [0.0; 5],
            pos_end: [0.0; 5],
            pos_provided: [false; 5],
            extruder_id: 0,
            extrusion_role: ExtrusionRole::Perimeter,
            volumetric_rate: 0.0,
            volumetric_rate_start: 0.0,
            volumetric_rate_end: 0.0,
            max_slope_positive: 0.0,
            max_slope_negative: 0.0,
        }
    }
}

impl GCodeLine {
    // Check if this line is moving in XY.
    fn moving_xy(&self) -> bool {
        (self.pos_end[0] - self.pos_start[0]).abs() > f32::EPSILON
            || (self.pos_end[1] - self.pos_start[1]).abs() > f32::EPSILON
    }

    /// Check if this line is extruding.
    fn extruding(&self) -> bool {
        self.moving_xy() && self.pos_end[3] > self.pos_start[3]
    }

    /// Calculate XY distance squared.
    fn dist_xy2(&self) -> f32 {
        let dx = self.pos_end[0] - self.pos_start[0];
        let dy = self.pos_end[1] - self.pos_start[1];
        dx * dx + dy * dy
    }

    /// Calculate XYZ distance squared.
    fn dist_xyz2(&self) -> f32 {
        let dx = self.pos_end[0] - self.pos_start[0];
        let dy = self.pos_end[1] - self.pos_start[1];
        let dz = self.pos_end[2] - self.pos_start[2];
        dx * dx + dy * dy + dz * dz
    }

    /// Calculate XY distance.
    fn dist_xy(&self) -> f32 {
        self.dist_xy2().sqrt()
    }

    /// Calculate XYZ distance.
    fn dist_xyz(&self) -> f32 {
        self.dist_xyz2().sqrt()
    }

    /// Get the feedrate (F value).
    fn feedrate(&self) -> f32 {
        self.pos_end[4]
    }

    /// Calculate the time for this segment (in minutes, since F is mm/min).
    fn time(&self) -> f32 {
        let dist = self.dist_xyz();
        if dist > f32::EPSILON && self.feedrate() > f32::EPSILON {
            dist / self.feedrate()
        } else {
            0.0
        }
    }

    /// Calculate the average volumetric correction factor.
    fn volumetric_correction_avg(&self) -> f32 {
        if self.volumetric_rate > f32::EPSILON {
            let avg = 0.5 * (self.volumetric_rate_start + self.volumetric_rate_end)
                / self.volumetric_rate;
            avg.clamp(0.0, 1.0)
        } else {
            1.0
        }
    }

    /// Calculate the corrected time for this segment.
    fn time_corrected(&self) -> f32 {
        self.time() * self.volumetric_correction_avg()
    }
}

/// Statistics about pressure equalization.
#[derive(Debug, Clone, Default)]
pub struct PressureEqualizerStats {
    /// Minimum volumetric extrusion rate seen.
    pub volumetric_rate_min: f32,

    /// Maximum volumetric extrusion rate seen.
    pub volumetric_rate_max: f32,

    /// Average volumetric extrusion rate (weighted by length).
    pub volumetric_rate_avg: f32,

    /// Total extrusion length processed.
    pub extrusion_length: f32,
}

impl PressureEqualizerStats {
    // Reset statistics.
    pub fn reset(&mut self) {
        self.volumetric_rate_min = f32::MAX;
        self.volumetric_rate_max = 0.0;
        self.volumetric_rate_avg = 0.0;
        self.extrusion_length = 0.0;
    }

    /// Update statistics with a new segment.
    fn update(&mut self, rate: f32, length: f32) {
        self.volumetric_rate_min = self.volumetric_rate_min.min(rate);
        self.volumetric_rate_max = self.volumetric_rate_max.max(rate);
        self.volumetric_rate_avg += rate * length;
        self.extrusion_length += length;
    }

    /// Finalize the average calculation.
    pub fn finalize(&mut self) {
        if self.extrusion_length > 0.0 {
            self.volumetric_rate_avg /= self.extrusion_length;
        }
    }
}

/// Number of extrusion roles to track.
const NUM_EXTRUSION_ROLES: usize = 18;

/// Convert ExtrusionRole to index.
fn role_to_index(role: ExtrusionRole) -> usize {
    match role {
        ExtrusionRole::ExternalPerimeter => 0,
        ExtrusionRole::Perimeter => 1,
        ExtrusionRole::OverhangWall => 2,
        ExtrusionRole::FloatingVerticalShell => 3,
        ExtrusionRole::InternalInfill => 4,
        ExtrusionRole::SolidInfill => 5,
        ExtrusionRole::TopSolidInfill => 6,
        ExtrusionRole::BottomSolidInfill => 7,
        ExtrusionRole::BridgeInfill => 8,
        ExtrusionRole::GapFill => 9,
        ExtrusionRole::Skirt => 10,
        ExtrusionRole::RaftBase => 11,
        ExtrusionRole::RaftInterface => 12,
        ExtrusionRole::RaftContact => 13,
        ExtrusionRole::SupportMaterial => 14,
        ExtrusionRole::SupportMaterialInterface => 15,
        ExtrusionRole::Wipe => 16,
        ExtrusionRole::Custom => 17,
    }
}

/// Pressure equalizer for smoothing volumetric extrusion rate changes.
///
/// This processes G-code through a circular buffer, analyzing extrusion rates
/// and adjusting feedrates to prevent rapid pressure changes.
pub struct PressureEqualizer {
    config: PressureEqualizerConfig,

    /// Per-role slope limits.
    role_slopes: [RoleSlope; NUM_EXTRUSION_ROLES],

    /// Filament cross-section areas per extruder.
    filament_cross_sections: Vec<f32>,

    /// Current position [X, Y, Z, E, F].
    current_pos: [f32; 5],

    /// Current extruder index.
    current_extruder: usize,

    /// Current extrusion role.
    current_role: ExtrusionRole,

    /// Whether currently retracted.
    retracted: bool,

    /// Circular buffer of G-code lines.
    buffer: Vec<GCodeLine>,

    /// Current position in the circular buffer.
    buffer_pos: usize,

    /// Number of items in the circular buffer.
    buffer_items: usize,

    /// Output buffer for processed G-code.
    output: String,

    /// Statistics.
    stats: PressureEqualizerStats,

    /// Line index for debugging.
    line_idx: usize,
}

impl PressureEqualizer {
    // Create a new pressure equalizer with the given configuration.
    pub fn new(config: PressureEqualizerConfig) -> Self {
        let cross_section = config.filament_cross_section();

        // Initialize per-role slopes
        let default_slope = RoleSlope::new(
            config.max_volumetric_rate_slope_positive * 60.0 * 60.0, // Convert to mm³/min²
            config.max_volumetric_rate_slope_negative * 60.0 * 60.0,
        );

        let mut role_slopes = [default_slope; NUM_EXTRUSION_ROLES];

        // Don't regulate pressure in bridge infill (needs consistent flow for bridging)
        role_slopes[role_to_index(ExtrusionRole::BridgeInfill)] = RoleSlope::unlimited();

        // Don't regulate pressure in gap fill (thin features need precise flow)
        role_slopes[role_to_index(ExtrusionRole::GapFill)] = RoleSlope::unlimited();

        let buffer_size = config.buffer_size;

        Self {
            config,
            role_slopes,
            filament_cross_sections: vec![cross_section],
            current_pos: [0.0; 5],
            current_extruder: 0,
            current_role: ExtrusionRole::Perimeter,
            retracted: true, // Expect first command to fill nozzle
            buffer: vec![GCodeLine::default(); buffer_size],
            buffer_pos: 0,
            buffer_items: 0,
            output: String::with_capacity(4096),
            stats: PressureEqualizerStats::default(),
            line_idx: 0,
        }
    }

    /// Reset the equalizer state.
    pub fn reset(&mut self) {
        self.current_pos = [0.0; 5];
        self.current_extruder = 0;
        self.current_role = ExtrusionRole::Perimeter;
        self.retracted = true;
        self.buffer_pos = 0;
        self.buffer_items = 0;
        self.output.clear();
        self.stats.reset();
        self.line_idx = 0;

        for line in &mut self.buffer {
            *line = GCodeLine::default();
        }
    }

    /// Add a filament cross-section for an additional extruder.
    pub fn add_extruder(&mut self, filament_diameter: f32) {
        let r = filament_diameter / 2.0;
        self.filament_cross_sections.push(PI * r * r);
    }

    /// Process G-code input and return the equalized output.
    ///
    /// If `flush` is true, all buffered lines will be output.
    pub fn process(&mut self, gcode: &str, flush: bool) -> String {
        self.output.clear();

        // Process each line
        for line in gcode.lines() {
            // If buffer is full, push out the oldest line
            if self.buffer_items == self.config.buffer_size {
                let head_idx = self.buffer_idx_head();
                self.output_gcode_line(head_idx);
            } else {
                self.buffer_items += 1;
            }

            // Process this line into the buffer
            let idx = self.buffer_pos;
            self.buffer_pos = self.buffer_idx_next(self.buffer_pos);

            if !self.process_line(line, idx) {
                // Line should be skipped (e.g., role marker comment)
                self.buffer_pos = idx;
                self.buffer_items -= 1;
            }
        }

        // Flush remaining lines if requested
        if flush {
            let mut idx = self.buffer_idx_head();
            while self.buffer_items > 0 {
                self.output_gcode_line(idx);
                idx = self.buffer_idx_next(idx);
                self.buffer_items -= 1;
            }
            self.buffer_pos = 0;
            self.stats.finalize();
        }

        std::mem::take(&mut self.output)
    }

    /// Get the current statistics.
    pub fn stats(&self) -> &PressureEqualizerStats {
        &self.stats
    }

    /// Get the buffer head index (oldest item).
    fn buffer_idx_head(&self) -> usize {
        let size = self.config.buffer_size;
        let idx = self.buffer_pos + size - self.buffer_items;
        if idx >= size {
            idx - size
        } else {
            idx
        }
    }

    /// Get the next buffer index.
    fn buffer_idx_next(&self, idx: usize) -> usize {
        let next = idx + 1;
        if next >= self.config.buffer_size {
            next - self.config.buffer_size
        } else {
            next
        }
    }

    /// Get the previous buffer index.
    fn buffer_idx_prev(&self, idx: usize) -> usize {
        if idx == 0 {
            self.config.buffer_size - 1
        } else {
            idx - 1
        }
    }

    /// Process a single G-code line into the buffer.
    /// Returns false if the line should be skipped.
    fn process_line(&mut self, line: &str, buf_idx: usize) -> bool {
        const EXTRUSION_ROLE_TAG: &str = ";_EXTRUSION_ROLE:";

        // Check for extrusion role marker
        if let Some(role_str) = line.strip_prefix(EXTRUSION_ROLE_TAG) {
            if let Ok(role_num) = role_str.trim().parse::<u32>() {
                self.current_role = role_from_number(role_num);
            }
            self.line_idx += 1;
            return false; // Don't buffer this line
        }

        // Initialize buffer entry
        self.buffer[buf_idx].line_type = GCodeLineType::Other;
        self.buffer[buf_idx].modified = false;
        self.buffer[buf_idx].raw = line.to_string();
        self.buffer[buf_idx].pos_start = self.current_pos;
        self.buffer[buf_idx].pos_end = self.current_pos;
        self.buffer[buf_idx].pos_provided = [false; 5];
        self.buffer[buf_idx].volumetric_rate = 0.0;
        self.buffer[buf_idx].volumetric_rate_start = 0.0;
        self.buffer[buf_idx].volumetric_rate_end = 0.0;
        self.buffer[buf_idx].max_slope_positive = 0.0;
        self.buffer[buf_idx].max_slope_negative = 0.0;
        self.buffer[buf_idx].extrusion_role = self.current_role;

        // Parse the line
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with(';') {
            self.buffer[buf_idx].line_type = GCodeLineType::Other;
            self.line_idx += 1;
            return true;
        }

        let mut chars = trimmed.chars().peekable();

        match chars.next() {
            Some('G') | Some('g') => {
                // Parse G-code number
                let gcode_str: String = chars.by_ref().take_while(|c| c.is_ascii_digit()).collect();
                if let Ok(gcode) = gcode_str.parse::<u32>() {
                    self.parse_gcode(gcode, &mut chars, buf_idx);
                }
            }
            Some('M') | Some('m') => {
                // M-codes don't need special handling for pressure equalization
                self.buffer[buf_idx].line_type = GCodeLineType::Other;
            }
            Some('T') | Some('t') => {
                // Tool change
                let extruder_str: String = chars.take_while(|c| c.is_ascii_digit()).collect();
                if let Ok(new_extruder) = extruder_str.parse::<usize>() {
                    if new_extruder != self.current_extruder {
                        self.current_extruder = new_extruder;
                        self.retracted = true;
                        self.buffer[buf_idx].line_type = GCodeLineType::ToolChange;
                    } else {
                        self.buffer[buf_idx].line_type = GCodeLineType::Noop;
                    }
                }
            }
            _ => {
                self.buffer[buf_idx].line_type = GCodeLineType::Other;
            }
        }

        self.buffer[buf_idx].extruder_id = self.current_extruder;
        self.buffer[buf_idx].pos_end = self.current_pos;

        // Adjust volumetric rates
        self.adjust_volumetric_rate();

        self.line_idx += 1;
        true
    }

    /// Parse a G-code command.
    fn parse_gcode(
        &mut self,
        gcode: u32,
        chars: &mut std::iter::Peekable<std::str::Chars>,
        buf_idx: usize,
    ) {
        match gcode {
            0 | 1 => {
                // G0/G1: Linear move
                let mut new_pos = self.current_pos;
                let mut changed = [false; 5];

                // Skip whitespace
                while chars.peek().map(|c| c.is_whitespace()).unwrap_or(false) {
                    chars.next();
                }

                // Parse axis values
                while let Some(&c) = chars.peek() {
                    if c == ';' {
                        break; // Comment
                    }

                    let axis = c.to_ascii_uppercase();
                    chars.next();

                    let axis_idx = match axis {
                        'X' => Some(0),
                        'Y' => Some(1),
                        'Z' => Some(2),
                        'E' => Some(3),
                        'F' => Some(4),
                        _ => None,
                    };

                    if let Some(idx) = axis_idx {
                        // Parse the value
                        let value_str: String = chars
                            .by_ref()
                            .take_while(|c| {
                                c.is_ascii_digit() || *c == '.' || *c == '-' || *c == '+'
                            })
                            .collect();

                        if let Ok(value) = value_str.parse::<f32>() {
                            self.buffer[buf_idx].pos_provided[idx] = true;
                            new_pos[idx] = if idx == 3 && self.config.use_relative_e {
                                self.current_pos[idx] + value
                            } else {
                                value
                            };
                            changed[idx] =
                                (new_pos[idx] - self.current_pos[idx]).abs() > f32::EPSILON;
                        }
                    }

                    // Skip whitespace
                    while chars.peek().map(|c| c.is_whitespace()).unwrap_or(false) {
                        chars.next();
                    }
                }

                // Determine move type
                if changed[3] {
                    // E axis changed
                    let e_diff = new_pos[3] - self.current_pos[3];
                    if e_diff < 0.0 {
                        // Retraction
                        self.buffer[buf_idx].line_type = GCodeLineType::Retract;
                        self.retracted = true;
                    } else if !changed[0] && !changed[1] && !changed[2] {
                        // Unretract (E increases but no XYZ move)
                        self.buffer[buf_idx].line_type = GCodeLineType::Unretract;
                        self.retracted = false;
                    } else {
                        // Extrusion move
                        self.buffer[buf_idx].line_type = GCodeLineType::Extrude;

                        // Calculate volumetric extrusion rate
                        let diff_x = new_pos[0] - self.current_pos[0];
                        let diff_y = new_pos[1] - self.current_pos[1];
                        let diff_z = new_pos[2] - self.current_pos[2];
                        let diff_e = new_pos[3] - self.current_pos[3];

                        let len2 = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
                        if len2 > f32::EPSILON && new_pos[4] > f32::EPSILON {
                            // volumetric rate = A_filament × F × L_e / L_xyz [mm³/min]
                            let cross_section = self
                                .filament_cross_sections
                                .get(self.current_extruder)
                                .copied()
                                .unwrap_or(self.filament_cross_sections[0]);

                            let rate =
                                cross_section * new_pos[4] * ((diff_e * diff_e) / len2).sqrt();

                            self.buffer[buf_idx].volumetric_rate = rate;
                            self.buffer[buf_idx].volumetric_rate_start = rate;
                            self.buffer[buf_idx].volumetric_rate_end = rate;

                            self.stats.update(rate, len2.sqrt());
                        }
                    }
                } else if changed[0] || changed[1] || changed[2] {
                    // Movement without extrusion
                    self.buffer[buf_idx].line_type = GCodeLineType::Move;
                }

                self.current_pos = new_pos;
            }
            10 | 22 => {
                // G10/G22: Firmware retract
                self.buffer[buf_idx].line_type = GCodeLineType::Retract;
                self.retracted = true;
            }
            11 | 23 => {
                // G11/G23: Firmware unretract
                self.buffer[buf_idx].line_type = GCodeLineType::Unretract;
                self.retracted = false;
            }
            92 => {
                // G92: Set position
                // Skip whitespace
                while chars.peek().map(|c| c.is_whitespace()).unwrap_or(false) {
                    chars.next();
                }

                while let Some(&c) = chars.peek() {
                    if c == ';' {
                        break;
                    }

                    let axis = c.to_ascii_uppercase();
                    chars.next();

                    let axis_idx = match axis {
                        'X' => Some(0),
                        'Y' => Some(1),
                        'Z' => Some(2),
                        'E' => Some(3),
                        _ => None,
                    };

                    if let Some(idx) = axis_idx {
                        let value_str: String = chars
                            .by_ref()
                            .take_while(|c| {
                                c.is_ascii_digit() || *c == '.' || *c == '-' || *c == '+'
                            })
                            .collect();

                        let value = value_str.parse::<f32>().unwrap_or(0.0);
                        self.current_pos[idx] = value;
                    }

                    while chars.peek().map(|c| c.is_whitespace()).unwrap_or(false) {
                        chars.next();
                    }
                }

                self.buffer[buf_idx].line_type = GCodeLineType::Other;
            }
            _ => {
                self.buffer[buf_idx].line_type = GCodeLineType::Other;
            }
        }
    }

    /// Adjust volumetric rates in the buffer to limit slope changes.
    fn adjust_volumetric_rate(&mut self) {
        if self.buffer_items < 2 {
            return;
        }

        let idx_head = self.buffer_idx_head();
        let idx_tail = self.buffer_idx_prev(if self.buffer_pos == 0 {
            self.config.buffer_size - 1
        } else {
            self.buffer_pos - 1
        });

        // Check if the last move is extruding
        if idx_tail == idx_head || !self.buffer[idx_tail].extruding() {
            return;
        }

        // Initialize per-role feedrates
        let mut feedrate_per_role = [f32::MAX; NUM_EXTRUSION_ROLES];
        feedrate_per_role[role_to_index(self.buffer[idx_tail].extrusion_role)] =
            self.buffer[idx_tail].volumetric_rate_start;

        // Go backward from tail to head, limiting rate increases
        let mut idx = idx_tail;
        let mut modified = true;

        while modified && idx != idx_head {
            let idx_prev = self.buffer_idx_prev(idx);

            // Skip non-extruding moves
            let mut scan_idx = idx_prev;
            while !self.buffer[scan_idx].extruding() && scan_idx != idx_head {
                scan_idx = self.buffer_idx_prev(scan_idx);
            }

            if !self.buffer[scan_idx].extruding() {
                break;
            }

            let rate_succ = self.buffer[idx].volumetric_rate_start;
            idx = scan_idx;

            // For each role, limit the rate
            for i_role in 1..NUM_EXTRUSION_ROLES {
                let rate_slope = self.role_slopes[i_role].negative;
                if rate_slope == 0.0 {
                    continue;
                }

                let mut rate_end = feedrate_per_role[i_role];
                let line_role = role_to_index(self.buffer[idx].extrusion_role);

                if i_role == line_role && rate_succ < rate_end {
                    rate_end = rate_succ;
                }

                if self.buffer[idx].volumetric_rate_end > rate_end {
                    self.buffer[idx].volumetric_rate_end = rate_end;
                    self.buffer[idx].modified = true;
                } else if i_role == line_role {
                    rate_end = self.buffer[idx].volumetric_rate_end;
                } else if rate_end == f32::MAX {
                    continue;
                }

                let rate_start = rate_end + rate_slope * self.buffer[idx].time_corrected();
                if rate_start < self.buffer[idx].volumetric_rate_start {
                    self.buffer[idx].volumetric_rate_start = rate_start;
                    self.buffer[idx].max_slope_negative = rate_slope;
                    self.buffer[idx].modified = true;
                }

                feedrate_per_role[i_role] = if i_role == line_role {
                    self.buffer[idx].volumetric_rate_start
                } else {
                    rate_start
                };
            }

            modified = idx != idx_prev;
        }

        // Go forward from head to tail, limiting rate decreases
        feedrate_per_role = [f32::MAX; NUM_EXTRUSION_ROLES];
        feedrate_per_role[role_to_index(self.buffer[idx].extrusion_role)] =
            self.buffer[idx].volumetric_rate_end;

        while idx != idx_tail {
            let idx_next = self.buffer_idx_next(idx);

            // Skip non-extruding moves
            let mut scan_idx = idx_next;
            while !self.buffer[scan_idx].extruding() && scan_idx != idx_tail {
                scan_idx = self.buffer_idx_next(scan_idx);
            }

            if !self.buffer[scan_idx].extruding() {
                break;
            }

            let rate_prec = self.buffer[idx].volumetric_rate_end;
            idx = scan_idx;

            for i_role in 1..NUM_EXTRUSION_ROLES {
                let rate_slope = self.role_slopes[i_role].positive;
                if rate_slope == 0.0 {
                    continue;
                }

                let mut rate_start = feedrate_per_role[i_role];
                let line_role = role_to_index(self.buffer[idx].extrusion_role);

                if i_role == line_role && rate_prec < rate_start {
                    rate_start = rate_prec;
                }

                if self.buffer[idx].volumetric_rate_start > rate_start {
                    self.buffer[idx].volumetric_rate_start = rate_start;
                    self.buffer[idx].modified = true;
                } else if i_role == line_role {
                    rate_start = self.buffer[idx].volumetric_rate_start;
                } else if rate_start == f32::MAX {
                    continue;
                }

                let rate_end = if rate_slope == 0.0 {
                    f32::MAX
                } else {
                    rate_start + rate_slope * self.buffer[idx].time_corrected()
                };

                if rate_end < self.buffer[idx].volumetric_rate_end {
                    self.buffer[idx].volumetric_rate_end = rate_end;
                    self.buffer[idx].max_slope_positive = rate_slope;
                    self.buffer[idx].modified = true;
                }

                feedrate_per_role[i_role] = if i_role == line_role {
                    self.buffer[idx].volumetric_rate_end
                } else {
                    rate_end
                };
            }
        }
    }

    /// Output a G-code line, possibly modified.
    fn output_gcode_line(&mut self, idx: usize) {
        // Extract all needed data from the buffer line first to avoid borrow issues
        let modified = self.buffer[idx].modified;
        let raw = self.buffer[idx].raw.clone();

        if !modified {
            // Output unmodified
            self.output.push_str(&raw);
            self.output.push('\n');
            return;
        }

        // Extract all needed values from the line
        let line_pos_start = self.buffer[idx].pos_start;
        let line_pos_end = self.buffer[idx].pos_end;
        let line_pos_provided = self.buffer[idx].pos_provided;
        let line_feedrate = self.buffer[idx].feedrate();
        let line_volumetric_rate = self.buffer[idx].volumetric_rate;
        let line_volumetric_rate_start = self.buffer[idx].volumetric_rate_start;
        let line_volumetric_rate_end = self.buffer[idx].volumetric_rate_end;
        let line_volumetric_correction_avg = self.buffer[idx].volumetric_correction_avg();
        let line_max_slope_positive = self.buffer[idx].max_slope_positive;
        let line_max_slope_negative = self.buffer[idx].max_slope_negative;
        let l = self.buffer[idx].dist_xyz();

        // Line was modified - need to regenerate with adjusted feedrate
        let comment_str = raw.find(';').map(|i| raw[i..].to_string());
        let comment = comment_str.as_deref();

        let n_segments = ((l / self.config.max_segment_length).ceil() as usize).max(1);

        if n_segments == 1 {
            // Just update this segment's feedrate
            let new_feedrate = line_feedrate * line_volumetric_correction_avg;
            self.push_line_to_output_values(
                &line_pos_start,
                &line_pos_end,
                &line_pos_provided,
                new_feedrate,
                comment,
            );
        } else {
            // Need to split into multiple segments
            let accelerating = line_volumetric_rate_start < line_volumetric_rate_end;

            // Calculate feedrates at start and end
            let feed_start = if line_volumetric_rate > f32::EPSILON {
                line_volumetric_rate_start * line_feedrate / line_volumetric_rate
            } else {
                line_feedrate
            };
            let feed_end = if line_volumetric_rate > f32::EPSILON {
                line_volumetric_rate_end * line_feedrate / line_volumetric_rate
            } else {
                line_feedrate
            };
            let feed_avg = 0.5 * (feed_start + feed_end);

            // Calculate acceleration time and distance
            let max_slope = if accelerating {
                line_max_slope_positive
            } else {
                line_max_slope_negative
            };

            let t_total = if feed_avg > f32::EPSILON {
                l / feed_avg
            } else {
                0.0
            };
            let t_acc = if max_slope > f32::EPSILON {
                0.5 * (line_volumetric_rate_start + line_volumetric_rate_end) / max_slope
            } else {
                t_total
            };

            let (l_acc, l_steady, actual_segments) = if t_acc < t_total {
                let l_acc_calc = t_acc * feed_avg;
                let l_steady_calc = l - l_acc_calc;
                if l_steady_calc < 0.5 * self.config.max_segment_length {
                    (l, 0.0, n_segments)
                } else {
                    let seg = (l_acc_calc / self.config.max_segment_length).ceil() as usize;
                    (l_acc_calc, l_steady_calc, seg.max(1))
                }
            } else {
                (l, 0.0, n_segments)
            };

            // Calculate segment positions
            let mut pos_start = line_pos_start;
            let pos_end = line_pos_end;

            if l_steady > f32::EPSILON {
                if accelerating {
                    // Accelerating: split into acceleration segments, then steady
                    let t = l_acc / l;
                    for i in 1..=actual_segments {
                        let seg_t = i as f32 / actual_segments as f32 * t;
                        let mut seg_end = [0.0f32; 5];
                        for j in 0..4 {
                            seg_end[j] =
                                line_pos_start[j] + (pos_end[j] - line_pos_start[j]) * seg_t;
                        }
                        seg_end[4] = feed_start
                            + (feed_end - feed_start) * (seg_t - 0.5 / actual_segments as f32);

                        self.push_segment(&pos_start, &seg_end, &line_pos_provided, comment);
                        pos_start = seg_end;
                    }

                    // Steady segment
                    self.push_segment(&pos_start, &line_pos_end, &line_pos_provided, None);
                } else {
                    // Decelerating: steady segment first, then deceleration
                    let t_steady = l_steady / l;
                    let mut steady_end = [0.0f32; 5];
                    for j in 0..4 {
                        steady_end[j] =
                            line_pos_start[j] + (pos_end[j] - line_pos_start[j]) * t_steady;
                    }
                    steady_end[4] = feed_start;

                    self.push_segment(&pos_start, &steady_end, &line_pos_provided, comment);
                    pos_start = steady_end;

                    // Deceleration segments
                    for i in 1..=actual_segments {
                        let seg_t = t_steady + (1.0 - t_steady) * i as f32 / actual_segments as f32;
                        let mut seg_end = [0.0f32; 5];
                        for j in 0..4 {
                            seg_end[j] =
                                line_pos_start[j] + (pos_end[j] - line_pos_start[j]) * seg_t;
                        }
                        let feed_t =
                            (seg_t - 0.5 / actual_segments as f32 - t_steady) / (1.0 - t_steady);
                        seg_end[4] = feed_start + (feed_end - feed_start) * feed_t;

                        self.push_segment(&pos_start, &seg_end, &line_pos_provided, None);
                        pos_start = seg_end;
                    }
                }
            } else {
                // No steady segment, just split evenly
                for i in 1..=actual_segments {
                    let t = i as f32 / actual_segments as f32;
                    let mut seg_end = [0.0f32; 5];
                    for j in 0..4 {
                        seg_end[j] = line_pos_start[j] + (pos_end[j] - line_pos_start[j]) * t;
                    }
                    let feed_t = (t - 0.5 / actual_segments as f32).clamp(0.0, 1.0);
                    seg_end[4] = feed_start + (feed_end - feed_start) * feed_t;

                    let c = if i == 1 { comment } else { None };
                    self.push_segment(&pos_start, &seg_end, &line_pos_provided, c);
                    pos_start = seg_end;
                }
            }
        }
    }

    /// Push a line segment to output.
    fn push_segment(
        &mut self,
        start: &[f32; 5],
        end: &[f32; 5],
        provided: &[bool; 5],
        comment: Option<&str>,
    ) {
        self.output.push_str("G1");

        for (i, axis) in ['X', 'Y', 'Z'].iter().enumerate() {
            if provided[i] {
                self.output.push_str(&format!(" {}{:.3}", axis, end[i]));
            }
        }

        // E axis
        let e_value = if self.config.use_relative_e {
            end[3] - start[3]
        } else {
            end[3]
        };
        self.output.push_str(&format!(" E{:.5}", e_value));

        // F axis (feedrate)
        self.output.push_str(&format!(" F{:.0}", end[4]));

        if let Some(c) = comment {
            self.output.push_str(c);
        }

        self.output.push('\n');
    }

    /// Push a line to output with a specific feedrate (using extracted values).
    fn push_line_to_output_values(
        &mut self,
        pos_start: &[f32; 5],
        pos_end: &[f32; 5],
        pos_provided: &[bool; 5],
        feedrate: f32,
        comment: Option<&str>,
    ) {
        self.output.push_str("G1");

        for (i, axis) in ['X', 'Y', 'Z'].iter().enumerate() {
            if pos_provided[i] {
                self.output.push_str(&format!(" {}{:.3}", axis, pos_end[i]));
            }
        }

        // E axis
        let e_value = if self.config.use_relative_e {
            pos_end[3] - pos_start[3]
        } else {
            pos_end[3]
        };
        self.output.push_str(&format!(" E{:.5}", e_value));

        // Feedrate
        self.output.push_str(&format!(" F{:.0}", feedrate));

        if let Some(c) = comment {
            self.output.push_str(c);
        }

        self.output.push('\n');
    }
}

/// Convert a numeric role code to ExtrusionRole.
fn role_from_number(num: u32) -> ExtrusionRole {
    match num {
        0 => ExtrusionRole::Perimeter,
        1 => ExtrusionRole::ExternalPerimeter,
        2 => ExtrusionRole::InternalInfill,
        3 => ExtrusionRole::SolidInfill,
        4 => ExtrusionRole::TopSolidInfill,
        5 => ExtrusionRole::BridgeInfill,
        6 => ExtrusionRole::GapFill,
        7 => ExtrusionRole::Skirt,
        8 => ExtrusionRole::SupportMaterial,
        9 => ExtrusionRole::SupportMaterialInterface,
        10 => ExtrusionRole::Wipe,
        _ => ExtrusionRole::Custom,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_default() {
        let config = PressureEqualizerConfig::default();
        assert!((config.filament_diameter - 1.75).abs() < 0.01);
        assert!(config.max_volumetric_rate_slope_positive > 0.0);
        assert!(config.max_volumetric_rate_slope_negative > 0.0);
    }

    #[test]
    fn test_config_cross_section() {
        let config = PressureEqualizerConfig::new(1.75);
        let area = config.filament_cross_section();
        let expected = PI * (1.75 / 2.0) * (1.75 / 2.0);
        assert!((area - expected).abs() < 0.0001);
    }

    #[test]
    fn test_equalizer_creation() {
        let config = PressureEqualizerConfig::default();
        let eq = PressureEqualizer::new(config);
        assert_eq!(eq.buffer_items, 0);
        assert_eq!(eq.current_extruder, 0);
    }

    #[test]
    fn test_equalizer_reset() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);
        eq.current_extruder = 1;
        eq.buffer_items = 5;
        eq.reset();
        assert_eq!(eq.current_extruder, 0);
        assert_eq!(eq.buffer_items, 0);
    }

    #[test]
    fn test_process_empty() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);
        let result = eq.process("", true);
        assert!(result.is_empty());
    }

    #[test]
    fn test_process_simple_move() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);
        let gcode = "G1 X10 Y10 F1000\n";
        let result = eq.process(gcode, true);
        assert!(result.contains("G1"));
    }

    #[test]
    fn test_process_comment() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);
        let gcode = "; This is a comment\n";
        let result = eq.process(gcode, true);
        assert!(result.contains("comment"));
    }

    #[test]
    fn test_process_extrusion() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);

        // Setup initial position
        let gcode = "G92 E0\nG1 X0 Y0 Z0.2 F1000\nG1 X10 Y0 E1 F1000\n";
        let result = eq.process(gcode, true);
        assert!(result.contains("G1"));
        assert!(result.contains("E"));
    }

    #[test]
    fn test_role_from_number() {
        assert_eq!(role_from_number(0), ExtrusionRole::Perimeter);
        assert_eq!(role_from_number(1), ExtrusionRole::ExternalPerimeter);
        assert_eq!(role_from_number(5), ExtrusionRole::BridgeInfill);
        assert_eq!(role_from_number(999), ExtrusionRole::Custom);
    }

    #[test]
    fn test_gcode_line_distances() {
        let mut line = GCodeLine::default();
        line.pos_start = [0.0, 0.0, 0.0, 0.0, 1000.0];
        line.pos_end = [3.0, 4.0, 0.0, 0.5, 1000.0];

        assert!((line.dist_xy() - 5.0).abs() < 0.001);
        assert!((line.dist_xyz() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_gcode_line_extruding() {
        let mut line = GCodeLine::default();
        line.pos_start = [0.0, 0.0, 0.0, 0.0, 1000.0];
        line.pos_end = [10.0, 0.0, 0.0, 1.0, 1000.0];
        line.line_type = GCodeLineType::Extrude;

        assert!(line.moving_xy());
        assert!(line.extruding());
    }

    #[test]
    fn test_gcode_line_time() {
        let mut line = GCodeLine::default();
        line.pos_start = [0.0, 0.0, 0.0, 0.0, 1000.0];
        line.pos_end = [100.0, 0.0, 0.0, 1.0, 1000.0]; // 100mm at 1000mm/min = 0.1 min

        let time = line.time();
        assert!((time - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_stats() {
        let mut stats = PressureEqualizerStats::default();
        stats.reset();
        stats.update(100.0, 10.0);
        stats.update(200.0, 20.0);
        stats.finalize();

        assert!((stats.volumetric_rate_min - 100.0).abs() < 0.001);
        assert!((stats.volumetric_rate_max - 200.0).abs() < 0.001);
        // Weighted avg: (100*10 + 200*20) / 30 = 5000/30 = 166.67
        assert!((stats.volumetric_rate_avg - 166.67).abs() < 0.01);
    }

    #[test]
    fn test_role_to_index() {
        assert_eq!(role_to_index(ExtrusionRole::ExternalPerimeter), 0);
        assert_eq!(role_to_index(ExtrusionRole::Perimeter), 1);
        assert_eq!(role_to_index(ExtrusionRole::BottomSolidInfill), 5);
        assert_eq!(role_to_index(ExtrusionRole::BridgeInfill), 6);
        assert_eq!(role_to_index(ExtrusionRole::GapFill), 7);
    }

    #[test]
    fn test_config_builder() {
        let config = PressureEqualizerConfig::new(2.85)
            .with_max_slope(2.0)
            .with_relative_e(true);

        assert!((config.filament_diameter - 2.85).abs() < 0.01);
        assert!((config.max_volumetric_rate_slope_positive - 2.0).abs() < 0.01);
        assert!((config.max_volumetric_rate_slope_negative - 2.0).abs() < 0.01);
        assert!(config.use_relative_e);
    }

    #[test]
    fn test_buffer_index_operations() {
        let config = PressureEqualizerConfig {
            buffer_size: 10,
            ..Default::default()
        };
        let eq = PressureEqualizer::new(config);

        assert_eq!(eq.buffer_idx_next(0), 1);
        assert_eq!(eq.buffer_idx_next(9), 0);
        assert_eq!(eq.buffer_idx_prev(0), 9);
        assert_eq!(eq.buffer_idx_prev(5), 4);
    }

    #[test]
    fn test_process_role_marker() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);

        let gcode = ";_EXTRUSION_ROLE:1\nG1 X10 Y10 E1 F1000\n";
        let result = eq.process(gcode, true);

        // Role marker should not be in output
        assert!(!result.contains("_EXTRUSION_ROLE"));
    }

    #[test]
    fn test_process_tool_change() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);

        let gcode = "T0\nT1\nT1\n"; // First T0 is noop (already 0), T1 is change, second T1 is noop
        let _ = eq.process(gcode, true);

        // Just verify it doesn't crash
    }

    #[test]
    fn test_volumetric_correction_avg() {
        let mut line = GCodeLine::default();
        line.volumetric_rate = 100.0;
        line.volumetric_rate_start = 80.0;
        line.volumetric_rate_end = 100.0;

        let correction = line.volumetric_correction_avg();
        // avg = (80 + 100) / 2 / 100 = 0.9
        assert!((correction - 0.9).abs() < 0.001);
    }

    #[test]
    fn test_process_g92() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);

        let gcode = "G92 X0 Y0 Z0 E0\nG1 X10 Y10 E1 F1000\n";
        let result = eq.process(gcode, true);

        assert!(result.contains("G92"));
    }

    #[test]
    fn test_firmware_retract() {
        let config = PressureEqualizerConfig::default();
        let mut eq = PressureEqualizer::new(config);

        let gcode = "G10\nG11\n";
        let result = eq.process(gcode, true);

        assert!(result.contains("G10"));
        assert!(result.contains("G11"));
    }

    #[test]
    fn test_add_extruder() {
        let config = PressureEqualizerConfig::new(1.75);
        let mut eq = PressureEqualizer::new(config);

        assert_eq!(eq.filament_cross_sections.len(), 1);

        eq.add_extruder(2.85);
        assert_eq!(eq.filament_cross_sections.len(), 2);

        let expected_area = PI * (2.85 / 2.0) * (2.85 / 2.0);
        assert!((eq.filament_cross_sections[1] - expected_area).abs() < 0.0001);
    }
}
