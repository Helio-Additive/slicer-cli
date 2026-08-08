//! Cooling buffer for layer time-based fan control and speed adjustments.
//!
//! This module implements cooling strategies similar to BambuStudio's CoolingBuffer:
//! - Calculates layer print time and adjusts speeds to meet minimum layer time
//! - Controls fan speed based on layer time thresholds
//! - Supports per-extruder cooling adjustments

use crate::print_config::PerExtruderCoolingConfig;
use crate::{CoordF, ExtrusionRole};

/// Epsilon for floating point comparisons.
/// Reference: libslic3r.h:52 `static constexpr double EPSILON = 1e-4;`
/// IMPORTANT: must be 1e-4 (not 1e-6). The non-proportional slowdown span-find
/// loop compares `line.feedrate > feedrate - EPSILON`. With feedrates ~300 mm/s
/// in f32, `300.0 - 1e-6 == 300.0` (1e-6 is below the f32 ULP at 300), so the
/// span never advances and zero lines get slowed. 1e-4 matches C++ CoolingBuffer.
const EPSILON: f32 = 1e-4;

/// Feature types that can be adjusted during cooling slowdown
/// Reference: GCodeEditor.hpp:17-35
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdjustableFeatureType {
    None = 0,
    ExternalPerimeters = 1 << 0,
    FirstInternalPerimeters = 1 << 1,
}

impl AdjustableFeatureType {
    pub const NONE: Self = Self::None;
    pub const EXTERNAL_PERIMETERS: Self = Self::ExternalPerimeters;
    pub const FIRST_INTERNAL_PERIMETERS: Self = Self::FirstInternalPerimeters;
}

impl std::ops::BitOr for AdjustableFeatureType {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self::Output {
        match (self as u32) | (rhs as u32) {
            0 => Self::None,
            1 => Self::ExternalPerimeters,
            2 => Self::FirstInternalPerimeters,
            3 => Self::FirstInternalPerimeters, // Combined
            _ => Self::None,
        }
    }
}

impl std::ops::BitAnd for AdjustableFeatureType {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self::Output {
        match (self as u32) & (rhs as u32) {
            0 => Self::None,
            1 => Self::ExternalPerimeters,
            2 => Self::FirstInternalPerimeters,
            _ => Self::None,
        }
    }
}

/// Cooling slowdown logic type
/// Reference: GCodeEditor.hpp (CoolingSlowdownLogicType)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoolingSlowdownLogic {
    /// Uniform cooling (original logic)
    UniformCooling,
    /// Consistent surface quality (slow internals first)
    ConsistentSurface,
}

/// Configuration for cooling behavior.
#[derive(Debug, Clone)]
pub struct CoolingConfig {
    /// Minimum layer time in seconds. Layers printing faster will be slowed down.
    pub min_layer_time: f64,
    /// Maximum layer time for full fan speed (seconds).
    pub max_layer_time: f64,
    /// Minimum print speed when slowing down for cooling (mm/s).
    pub min_print_speed: f64,
    /// Enable fan if layer time is below this threshold (seconds).
    pub fan_below_layer_time: f64,
    /// Fan speed for layers below threshold (0.0 - 1.0).
    pub fan_speed: f64,
    /// Disable fan for first N layers.
    pub disable_fan_first_layers: u32,
    /// Enable bridge fan override.
    pub bridge_fan_override: bool,
    /// Fan speed for bridges (0.0 - 1.0).
    pub bridge_fan_speed: f64,
    /// Enable overhang fan override.
    pub overhang_fan_override: bool,
    /// Fan speed for overhangs (0.0 - 1.0).
    pub overhang_fan_speed: f64,
    /// Slowdown method: proportional vs binary.
    pub slowdown_proportional: bool,
    /// Full fan speed threshold (layer time in seconds).
    pub full_fan_speed_layer_time: f64,
    /// Cooling slowdown logic type.
    pub slowdown_logic: CoolingSlowdownLogic,
    /// Distance before perimeter end for transition zone (mm).
    pub perimeter_transition_distance: f64,
}

impl Default for CoolingConfig {
    fn default() -> Self {
        Self {
            min_layer_time: 5.0,
            max_layer_time: 60.0,
            min_print_speed: 10.0,
            fan_below_layer_time: 60.0,
            fan_speed: 1.0,
            disable_fan_first_layers: 1,
            bridge_fan_override: true,
            bridge_fan_speed: 1.0,
            overhang_fan_override: false,
            overhang_fan_speed: 0.5,
            slowdown_proportional: false,
            full_fan_speed_layer_time: 0.0,
            slowdown_logic: CoolingSlowdownLogic::UniformCooling,
            perimeter_transition_distance: 5.0,
        }
    }
}

impl CoolingConfig {
    // Create a new cooling config with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set minimum layer time.
    pub fn with_min_layer_time(mut self, time: f64) -> Self {
        self.min_layer_time = time;
        self
    }

    /// Set minimum print speed.
    pub fn with_min_print_speed(mut self, speed: f64) -> Self {
        self.min_print_speed = speed;
        self
    }

    /// Set fan speed (0.0 - 1.0).
    pub fn with_fan_speed(mut self, speed: f64) -> Self {
        self.fan_speed = speed.clamp(0.0, 1.0);
        self
    }

    /// Disable fan for first N layers.
    pub fn with_disable_fan_first_layers(mut self, layers: u32) -> Self {
        self.disable_fan_first_layers = layers;
        self
    }

    /// Enable/disable bridge fan override.
    pub fn with_bridge_fan(mut self, enabled: bool, speed: f64) -> Self {
        self.bridge_fan_override = enabled;
        self.bridge_fan_speed = speed.clamp(0.0, 1.0);
        self
    }
}

/// Represents a single move/extrusion segment for cooling calculations.
/// Reference: GCodeEditor.hpp:37-133
#[derive(Debug, Clone)]
pub struct CoolingMove {
    /// Length of the move in mm.
    pub length: f64,
    /// Original feedrate in mm/s.
    pub feedrate: f64,
    /// Whether this is a travel move (non-extrusion).
    pub is_travel: bool,
    /// Whether this can be slowed down.
    pub can_slowdown: bool,
    /// Extrusion role for this move.
    pub role: Option<ExtrusionRole>,
    /// Time to execute this move at original speed (seconds).
    pub time: f64,
    /// Maximum time when slowed to minimum speed (seconds).
    pub time_max: f64,
    /// Adjusted feedrate after cooling slowdown (mm/s).
    pub adjusted_feedrate: f64,
    /// Whether this move has been slowed down.
    pub slowdown: bool,

    // ConsistentSurface fields
    /// Adjustable portion of length (mm).
    pub adjustable_length: f64,
    /// Non-adjustable portion of length (mm).
    pub non_adjustable_length: f64,
    /// Adjustable portion of time (seconds).
    pub adjustable_time: f64,
    /// Non-adjustable portion of time (seconds).
    pub non_adjustable_time: f64,
    /// Maximum adjustable time (seconds).
    pub adjustable_time_max: f64,
    /// Whether this is an external perimeter.
    pub is_external_perimeter: bool,
    /// Whether this is a first internal perimeter.
    pub is_first_internal_perimeter: bool,
}

impl CoolingMove {
    /// Create a new cooling move
    /// Reference: GCodeEditor.hpp:50-53
    pub fn new(length: f64, feedrate: f64, is_travel: bool, role: Option<ExtrusionRole>) -> Self {
        let time = if feedrate > 0.0 {
            length / feedrate
        } else {
            0.0
        };
        let can_slowdown = !is_travel && role != Some(ExtrusionRole::BridgeInfill);

        // Calculate time_max (time at minimum speed, typically 10 mm/s)
        // C++: time_max = length / min_print_speed
        let min_speed = 10.0; // Default min_print_speed (mm/s)
        let time_max = if can_slowdown && length > 0.0 {
            length / min_speed
        } else {
            time
        };

        Self {
            length,
            feedrate,
            is_travel,
            can_slowdown,
            role,
            time,
            time_max,
            adjusted_feedrate: feedrate,
            slowdown: false,
            adjustable_length: 0.0,
            non_adjustable_length: 0.0,
            adjustable_time: 0.0,
            non_adjustable_time: 0.0,
            adjustable_time_max: 0.0,
            is_external_perimeter: role == Some(ExtrusionRole::ExternalPerimeter),
            is_first_internal_perimeter: false, // Set by caller if needed
        }
    }

    /// Create a travel move.
    pub fn travel(length: f64, feedrate: f64) -> Self {
        Self::new(length, feedrate, true, None)
    }

    /// Create an extrusion move.
    pub fn extrusion(length: f64, feedrate: f64, role: ExtrusionRole) -> Self {
        Self::new(length, feedrate, false, Some(role))
    }

    /// Calculate time at current adjusted feedrate.
    pub fn adjusted_time(&self) -> f64 {
        if self.adjusted_feedrate > 0.0 {
            self.length / self.adjusted_feedrate
        } else {
            self.time
        }
    }

    /// Check if move is adjustable for given feature types
    /// Reference: GCodeEditor.hpp:78-90
    pub fn adjustable_for_features(&self, features: AdjustableFeatureType) -> bool {
        if !self.can_slowdown || self.adjustable_time >= self.adjustable_time_max {
            return false;
        }

        if self.is_external_perimeter {
            return features == AdjustableFeatureType::EXTERNAL_PERIMETERS
                || features
                    == (AdjustableFeatureType::EXTERNAL_PERIMETERS
                        | AdjustableFeatureType::FIRST_INTERNAL_PERIMETERS);
        }

        if self.is_first_internal_perimeter {
            return features == AdjustableFeatureType::FIRST_INTERNAL_PERIMETERS
                || features
                    == (AdjustableFeatureType::EXTERNAL_PERIMETERS
                        | AdjustableFeatureType::FIRST_INTERNAL_PERIMETERS);
        }

        true
    }
}

/// Per-extruder adjustments for cooling.
/// Reference: GCodeEditor.hpp:139-402
#[derive(Debug, Clone)]
pub struct PerExtruderAdjustments {
    /// Extruder index.
    pub extruder_id: usize,
    /// All moves for this extruder.
    pub moves: Vec<CoolingMove>,
    /// Total extrusion time (excluding travels).
    pub extrusion_time: f64,
    /// Total travel time.
    pub travel_time: f64,
    /// Time that can be slowed down.
    pub slowdown_time: f64,
    /// Slowdown factor applied (1.0 = no slowdown).
    pub slowdown_factor: f64,

    // Advanced cooling fields
    /// Is cooling slow down enabled for this extruder?
    pub cooling_slow_down_enabled: bool,
    /// Slow down if layer time is below this (seconds).
    pub slow_down_layer_time: f32,
    /// Minimum print speed when slowing down (mm/s).
    pub slow_down_min_speed: f32,
    /// Cooling slowdown logic type.
    pub cooling_slowdown_logic: CoolingSlowdownLogic,
    /// Distance before perimeter end for transition (mm).
    pub cooling_perimeter_transition_distance: f32,

    // For non-proportional slowdown
    /// Number of adjustable lines at start of moves.
    pub n_lines_adjustable: usize,
    /// Non-adjustable time of remaining lines.
    pub time_non_adjustable: f32,
    /// Current total time for this extruder.
    pub time_total: f32,
    /// Maximum time when slowed to minimum.
    pub time_maximum: f32,

    // Temporaries for processing
    /// Beginning index for current processing span.
    pub idx_line_begin: usize,
    /// End index for current processing span.
    pub idx_line_end: usize,
}

impl PerExtruderAdjustments {
    /// Create new per-extruder adjustments
    pub fn new(extruder_id: usize) -> Self {
        Self {
            extruder_id,
            moves: Vec::new(),
            extrusion_time: 0.0,
            travel_time: 0.0,
            slowdown_time: 0.0,
            slowdown_factor: 1.0,
            cooling_slow_down_enabled: true,
            slow_down_layer_time: 5.0,
            slow_down_min_speed: 10.0,
            cooling_slowdown_logic: CoolingSlowdownLogic::UniformCooling,
            cooling_perimeter_transition_distance: 5.0,
            n_lines_adjustable: 0,
            time_non_adjustable: 0.0,
            time_total: 0.0,
            time_maximum: 0.0,
            idx_line_begin: 0,
            idx_line_end: 0,
        }
    }

    /// Add a move to this extruder's list.
    pub fn add_move(&mut self, mov: CoolingMove) {
        if mov.is_travel {
            self.travel_time += mov.time;
        } else {
            self.extrusion_time += mov.time;
            if mov.can_slowdown {
                self.slowdown_time += mov.time;
            }
        }
        self.moves.push(mov);
    }

    /// Get total time at original speeds.
    pub fn total_time(&self) -> f64 {
        self.extrusion_time + self.travel_time
    }

    /// Get total time after adjustments.
    pub fn adjusted_total_time(&self) -> f64 {
        self.moves.iter().map(|m| m.adjusted_time()).sum()
    }

    /// Apply slowdown factor to all eligible moves.
    pub fn apply_slowdown(&mut self, factor: f64, min_speed: f64) {
        self.slowdown_factor = factor;
        for mov in &mut self.moves {
            if mov.can_slowdown {
                mov.adjusted_feedrate = (mov.feedrate / factor).max(min_speed);
            }
        }
    }

    /// Calculate elapsed time total
    /// Reference: GCodeEditor.hpp:140-145
    pub fn elapsed_time_total(&self) -> f32 {
        self.moves.iter().map(|m| m.time as f32).sum()
    }

    /// Calculate maximum time after slowdown
    /// Reference: GCodeEditor.hpp:148-160
    pub fn maximum_time_after_slowdown(&self, slowdown_external: bool) -> f32 {
        let mut time_total = 0.0f32;
        for mov in &self.moves {
            if self.adjustable_move(mov, slowdown_external) {
                if mov.time_max == f64::MAX {
                    return f32::MAX;
                }
                time_total += mov.time_max as f32;
            } else {
                time_total += mov.time as f32;
            }
        }
        time_total
    }

    /// Check if move is adjustable
    fn adjustable_move(&self, mov: &CoolingMove, slowdown_external: bool) -> bool {
        mov.can_slowdown
            && (slowdown_external || !mov.is_external_perimeter)
            && mov.time < mov.time_max
    }

    /// Calculate adjustable time
    /// Reference: GCodeEditor.hpp:163-170
    pub fn adjustable_time(&self, slowdown_external: bool) -> f32 {
        self.moves
            .iter()
            .filter(|m| self.adjustable_move(m, slowdown_external))
            .map(|m| m.time as f32)
            .sum()
    }

    /// Calculate non-adjustable time
    /// Reference: GCodeEditor.hpp:173-180
    pub fn non_adjustable_time(&self, slowdown_external: bool) -> f32 {
        self.moves
            .iter()
            .filter(|m| !self.adjustable_move(m, slowdown_external))
            .map(|m| m.time as f32)
            .sum()
    }

    /// Slow down to minimum feedrate
    /// Reference: GCodeEditor.hpp:184-197
    pub fn slowdown_to_minimum_feedrate(&mut self, slowdown_external: bool) -> f32 {
        let mut time_total = 0.0f32;
        for mov in &mut self.moves {
            let is_adjustable = mov.can_slowdown
                && (slowdown_external || !mov.is_external_perimeter)
                && mov.time < mov.time_max;
            if is_adjustable {
                debug_assert!(mov.time_max >= 0.0 && mov.time_max < f64::MAX);
                mov.slowdown = true;
                mov.time = mov.time_max;
                mov.feedrate = mov.length / mov.time;
            }
            time_total += mov.time as f32;
        }
        time_total
    }

    /// Slow down proportionally by factor
    /// Reference: GCodeEditor.hpp:201-212
    pub fn slow_down_proportional(&mut self, factor: f32, slowdown_external: bool) -> f32 {
        debug_assert!(factor >= 1.0);
        let mut time_total = 0.0f32;
        for mov in &mut self.moves {
            let is_adjustable = mov.can_slowdown
                && (slowdown_external || !mov.is_external_perimeter)
                && mov.time < mov.time_max;
            if is_adjustable {
                mov.slowdown = true;
                mov.time = mov.time_max.min(mov.time * factor as f64);
                mov.feedrate = mov.length / mov.time;
            }
            time_total += mov.time as f32;
        }
        time_total
    }

    /// Sort lines by decreasing feedrate
    /// Reference: GCodeEditor.hpp:216-225
    pub fn sort_lines_by_decreasing_feedrate(&mut self) {
        self.moves.sort_by(|a, b| {
            let adj_a = a.can_slowdown;
            let adj_b = b.can_slowdown;
            if adj_a == adj_b {
                b.feedrate
                    .partial_cmp(&a.feedrate)
                    .unwrap_or(std::cmp::Ordering::Equal)
            } else if adj_a {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        });

        self.n_lines_adjustable = self.moves.iter().take_while(|m| m.can_slowdown).count();
        self.time_non_adjustable = self.moves[self.n_lines_adjustable..]
            .iter()
            .map(|m| m.time as f32)
            .sum();
    }

    /// Calculate time stretch when slowing to feedrate
    /// Reference: GCodeEditor.hpp:230-239
    pub fn time_stretch_when_slowing_down_to_feedrate(&self, min_feedrate: f32) -> f32 {
        let mut time_stretch = 0.0f32;
        debug_assert!(self.slow_down_min_speed < min_feedrate + EPSILON);
        for i in 0..self.n_lines_adjustable {
            let mov = &self.moves[i];
            if mov.feedrate as f32 > min_feedrate {
                time_stretch += mov.time as f32 * (mov.feedrate as f32 / min_feedrate - 1.0);
            }
        }
        time_stretch
    }

    /// Slow down to feedrate
    /// Reference: GCodeEditor.hpp:244-254
    pub fn slow_down_to_feedrate(&mut self, min_feedrate: f32) {
        debug_assert!(self.slow_down_min_speed < min_feedrate + EPSILON);
        for i in 0..self.n_lines_adjustable {
            let mov = &mut self.moves[i];
            if mov.feedrate as f32 > min_feedrate {
                mov.time *= (mov.feedrate as f32 / min_feedrate).max(1.0) as f64;
                mov.feedrate = min_feedrate as f64;
                mov.slowdown = true;
            }
        }
    }

    /// Time stretch for specific features (ConsistentSurface)
    /// Reference: GCodeEditor.hpp:264-272
    pub fn time_stretch_when_slowing_down_to_feedrate_for_features(
        &self,
        min_feedrate: f32,
        features: AdjustableFeatureType,
    ) -> f32 {
        let mut time_stretch = 0.0f32;
        for i in 0..self.n_lines_adjustable {
            let mov = &self.moves[i];
            if mov.adjustable_for_features(features) && mov.feedrate as f32 > min_feedrate {
                time_stretch +=
                    mov.adjustable_time as f32 * (mov.feedrate as f32 / min_feedrate - 1.0);
            }
        }
        time_stretch
    }

    /// Slow down to feedrate for specific features
    /// Reference: GCodeEditor.hpp:276-286
    pub fn slow_down_to_feedrate_for_features(
        &mut self,
        min_feedrate: f32,
        features: AdjustableFeatureType,
    ) {
        for i in 0..self.n_lines_adjustable {
            let mov = &mut self.moves[i];
            if mov.adjustable_for_features(features) && mov.feedrate as f32 > min_feedrate {
                mov.adjustable_time = mov.adjustable_length / min_feedrate as f64;
                mov.time = mov.adjustable_time + mov.non_adjustable_time;
                mov.feedrate = min_feedrate as f64;
                mov.slowdown = true;
            }
        }
    }

    /// Create non-adjustable segments at perimeter ends
    /// Reference: GCodeEditor.hpp:324-361
    pub fn create_non_adjustable_segments(&mut self, non_adjustable_length: f32) {
        if non_adjustable_length <= 0.0 {
            return;
        }

        let mut accumulated_length = 0.0f32;

        // Process in reverse to accumulate from end of perimeters
        for i in (0..self.moves.len()).rev() {
            let mov = &mut self.moves[i];

            // Reset at perimeter boundaries
            if !mov.can_slowdown {
                accumulated_length = 0.0;
                continue;
            }

            // Initialize adjustable fields if not set
            if mov.adjustable_length == 0.0 && mov.length > 0.0 {
                mov.adjustable_length = mov.length;
                mov.adjustable_time = mov.time;
                mov.adjustable_time_max = mov.time_max;
            }

            let remaining = non_adjustable_length - accumulated_length;
            if remaining > 0.0 && mov.adjustable_length > 0.0 {
                let convert_length = mov.adjustable_length.min(remaining as f64);
                let convert_ratio = convert_length / mov.adjustable_length;

                mov.non_adjustable_length += convert_length;
                mov.non_adjustable_time += mov.adjustable_time * convert_ratio;
                mov.adjustable_length -= convert_length;
                mov.adjustable_time -= mov.adjustable_time * convert_ratio;
                mov.adjustable_time_max =
                    if mov.adjustable_length > 0.0 && self.slow_down_min_speed > 0.0 {
                        mov.adjustable_length / self.slow_down_min_speed as f64
                    } else {
                        0.0
                    };

                accumulated_length += convert_length as f32;
            } else {
                accumulated_length += mov.length as f32;
            }
        }
    }
}

/// Cooling buffer that manages layer cooling and fan control.
#[derive(Debug)]
pub struct CoolingBuffer {
    /// Cooling configuration.
    config: CoolingConfig,
}

impl CoolingBuffer {
    // Create a new cooling buffer with the given configuration.
    pub fn new(config: CoolingConfig) -> Self {
        Self { config }
    }

    /// Create a cooling buffer with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(CoolingConfig::default())
    }

    /// Get the cooling configuration.
    pub fn config(&self) -> &CoolingConfig {
        &self.config
    }

    /// Calculate the slowdown factor needed to meet minimum layer time.
    ///
    /// Returns the factor by which print speeds should be divided.
    /// A factor of 1.0 means no slowdown needed.
    /// Binary search to find feedrate that achieves target time stretch
    /// Reference: CoolingBuffer.cpp:1-60
    fn new_feedrate_to_reach_time_stretch(
        adjustments: &[&PerExtruderAdjustments],
        min_feedrate: f32,
        time_stretch: f32,
        max_iter: usize,
    ) -> f32 {
        let mut new_feedrate = min_feedrate;
        let mut current_min = min_feedrate;

        for _iter in 0..max_iter {
            let mut nomin = 0.0f32;
            let mut denom = time_stretch;

            for adj in adjustments {
                debug_assert!(adj.slow_down_min_speed < current_min + EPSILON);
                for i in 0..adj.n_lines_adjustable {
                    let mov = &adj.moves[i];
                    if mov.feedrate as f32 > current_min {
                        nomin += mov.time as f32 * mov.feedrate as f32;
                        denom += mov.time as f32;
                    }
                }
            }

            debug_assert!(denom > 0.0);
            if denom < 0.0 {
                return min_feedrate;
            }

            new_feedrate = nomin / denom;
            debug_assert!(new_feedrate > current_min - EPSILON);

            if new_feedrate < current_min + EPSILON {
                break;
            }

            // Check if any line would be slower than new_feedrate
            let mut needs_retry = false;
            for adj in adjustments {
                for i in 0..adj.n_lines_adjustable {
                    let mov = &adj.moves[i];
                    if mov.feedrate as f32 > current_min && (mov.feedrate as f32) < new_feedrate {
                        needs_retry = true;
                        break;
                    }
                }
                if needs_retry {
                    break;
                }
            }

            if !needs_retry {
                break;
            }

            current_min = new_feedrate;
        }

        new_feedrate
    }

    /// Proportional slowdown algorithm
    /// Reference: CoolingBuffer.cpp:63-119
    fn extruder_range_slow_down_proportional(
        adjustments: &mut [&mut PerExtruderAdjustments],
        elapsed_time_total0: f32,
        elapsed_time_before_slowdown: f32,
        slow_down_layer_time: f32,
    ) -> f32 {
        let mut total_after_slowdown = elapsed_time_before_slowdown;

        // Check if we can meet target by slowing only non-external perimeters
        let mut max_time_nep = elapsed_time_total0;
        for adj in adjustments.iter() {
            max_time_nep += adj.maximum_time_after_slowdown(false);
        }

        if max_time_nep > slow_down_layer_time {
            // Slow down only non-external perimeters
            let mut non_adjustable_time = elapsed_time_total0;
            for adj in adjustments.iter() {
                non_adjustable_time += adj.non_adjustable_time(false);
            }

            // Linear programming: iterate up to 5 times
            for _iter in 0..5 {
                let factor = (slow_down_layer_time - non_adjustable_time)
                    / (total_after_slowdown - non_adjustable_time);
                debug_assert!(factor > 1.0);

                total_after_slowdown = elapsed_time_total0;
                for adj in adjustments.iter_mut() {
                    total_after_slowdown += adj.slow_down_proportional(factor, false);
                }

                if total_after_slowdown > 0.95 * slow_down_layer_time {
                    break;
                }
            }
        } else {
            // Need to slow down everything
            // First max out non-external perimeters
            for adj in adjustments.iter_mut() {
                adj.slowdown_to_minimum_feedrate(false);
            }

            // Then slow down external perimeters proportionally
            let mut non_adjustable_time = elapsed_time_total0;
            for adj in adjustments.iter() {
                non_adjustable_time += adj.non_adjustable_time(true);
            }

            for _iter in 0..5 {
                let factor = (slow_down_layer_time - non_adjustable_time)
                    / (total_after_slowdown - non_adjustable_time);
                debug_assert!(factor > 1.0);

                total_after_slowdown = elapsed_time_total0;
                for adj in adjustments.iter_mut() {
                    total_after_slowdown += adj.slow_down_proportional(factor, true);
                }

                if total_after_slowdown > 0.95 * slow_down_layer_time {
                    break;
                }
            }
        }

        total_after_slowdown
    }

    /// ConsistentSurface slowdown algorithm (two-phase)
    /// Reference: CoolingBuffer.cpp:122-207
    fn extruder_range_slow_down_consistent_surface(
        adjustments: &mut [&mut PerExtruderAdjustments],
        time_stretch: f32,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        if time_stretch <= 0.0 {
            return 0.0;
        }

        // Sort by slow_down_min_speed (highest first)
        let mut by_min_speed: Vec<usize> = (0..adjustments.len()).collect();
        by_min_speed.sort_by(|&a, &b| {
            adjustments[b]
                .slow_down_min_speed
                .partial_cmp(&adjustments[a].slow_down_min_speed)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Find highest adjustable feedrate
        let mut feedrate = 0.0f32;
        for &idx in &by_min_speed {
            let adj = &adjustments[idx];
            for i in 0..adj.n_lines_adjustable {
                let mov = &adj.moves[i];
                if mov.adjustable_for_features(additional_slowdown_features)
                    && mov.feedrate as f32 > feedrate
                {
                    feedrate = mov.feedrate as f32;
                }
            }
        }

        if feedrate == 0.0 {
            return time_stretch; // No adjustable features
        }

        let mut remaining_stretch = time_stretch;
        let mut processed = 0;

        while processed < by_min_speed.len() {
            let current_idx = by_min_speed[processed];
            let feedrate_limit = adjustments[current_idx].slow_down_min_speed;

            // Calculate max time stretch at this feedrate limit
            let mut time_stretch_max = 0.0f32;
            for i in processed..by_min_speed.len() {
                let idx = by_min_speed[i];
                time_stretch_max += adjustments[idx]
                    .time_stretch_when_slowing_down_to_feedrate_for_features(
                        feedrate_limit,
                        additional_slowdown_features,
                    );
            }

            if time_stretch_max >= remaining_stretch {
                // Binary search for exact feedrate
                let mut feedrate_high = feedrate;
                let mut feedrate_low = feedrate_limit;

                for _iter in 0..20 {
                    let feedrate_mid = (feedrate_high + feedrate_low) / 2.0;
                    let mut stretch = 0.0f32;
                    for i in processed..by_min_speed.len() {
                        let idx = by_min_speed[i];
                        stretch += adjustments[idx]
                            .time_stretch_when_slowing_down_to_feedrate_for_features(
                                feedrate_mid,
                                additional_slowdown_features,
                            );
                    }

                    if stretch < remaining_stretch {
                        feedrate_high = feedrate_mid;
                    } else {
                        feedrate_low = feedrate_mid;
                    }

                    if (stretch - remaining_stretch).abs() < 0.01 {
                        break;
                    }
                }

                // Apply the slowdown
                for i in processed..by_min_speed.len() {
                    let idx = by_min_speed[i];
                    adjustments[idx].slow_down_to_feedrate_for_features(
                        feedrate_low,
                        additional_slowdown_features,
                    );
                }

                return 0.0; // Time stretch achieved
            } else {
                // Slow down to minimum for this tier
                remaining_stretch -= time_stretch_max;
                for i in processed..by_min_speed.len() {
                    let idx = by_min_speed[i];
                    adjustments[idx].slow_down_to_feedrate_for_features(
                        feedrate_limit,
                        additional_slowdown_features,
                    );
                }
            }

            // Skip to next speed tier
            processed += 1;
            while processed < by_min_speed.len() {
                let next_idx = by_min_speed[processed];
                if adjustments[next_idx].slow_down_min_speed < feedrate_limit - EPSILON {
                    break;
                }
                processed += 1;
            }
        }

        remaining_stretch
    }

    /// Non-proportional slowdown algorithm (equalize feedrates)
    /// Reference: CoolingBuffer.cpp:210-287
    fn extruder_range_slow_down_non_proportional(
        adjustments: &mut [&mut PerExtruderAdjustments],
        mut time_stretch: f32,
    ) {
        // Sort by slow_down_min_speed (highest first)
        let mut by_min_speed: Vec<usize> = (0..adjustments.len()).collect();
        by_min_speed.sort_by(|&a, &b| {
            adjustments[b]
                .slow_down_min_speed
                .partial_cmp(&adjustments[a].slow_down_min_speed)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Find highest adjustable feedrate
        let mut feedrate = 0.0f32;
        for &idx in &by_min_speed {
            let adj = &mut adjustments[idx];
            adj.idx_line_begin = 0;
            adj.idx_line_end = 0;
            if adj.n_lines_adjustable > 0 {
                let first_feedrate = adj.moves[0].feedrate as f32;
                if first_feedrate > feedrate {
                    feedrate = first_feedrate;
                }
            }
        }

        debug_assert!(feedrate > 0.0);

        loop {
            // Find span of lines with feedrate close to current feedrate
            for &idx in &by_min_speed {
                let adj = &mut adjustments[idx];
                adj.idx_line_end = adj.idx_line_begin;
                while adj.idx_line_end < adj.n_lines_adjustable
                    && adj.moves[adj.idx_line_end].feedrate as f32 > feedrate - EPSILON
                {
                    adj.idx_line_end += 1;
                }
            }

            // Find next highest feedrate
            let mut feedrate_next = 0.0f32;
            for &idx in &by_min_speed {
                let adj = &adjustments[idx];
                if adj.idx_line_end < adj.n_lines_adjustable {
                    let next_feedrate = adj.moves[adj.idx_line_end].feedrate as f32;
                    if next_feedrate > feedrate_next {
                        feedrate_next = next_feedrate;
                    }
                }
            }

            // Process each speed tier
            let mut tier_idx = 0;
            while tier_idx < by_min_speed.len() {
                let current_idx = by_min_speed[tier_idx];
                let min_speed = adjustments[current_idx].slow_down_min_speed;

                if min_speed == 0.0 {
                    // All adjustable speeds are now at same speed, uniformly slow down
                    let mut time_adjustable = 0.0f32;
                    for i in tier_idx..by_min_speed.len() {
                        let idx = by_min_speed[i];
                        time_adjustable += adjustments[idx].adjustable_time(true);
                    }
                    let rate = (time_adjustable + time_stretch) / time_adjustable;
                    for i in tier_idx..by_min_speed.len() {
                        let idx = by_min_speed[i];
                        adjustments[idx].slow_down_proportional(rate, true);
                    }
                    return;
                } else {
                    let feedrate_limit = feedrate_next.max(min_speed);

                    // Calculate time stretch available
                    let mut time_stretch_max = 0.0f32;
                    for i in tier_idx..by_min_speed.len() {
                        let idx = by_min_speed[i];
                        time_stretch_max += adjustments[idx]
                            .time_stretch_when_slowing_down_to_feedrate(feedrate_limit);
                    }

                    if time_stretch_max >= time_stretch {
                        // Binary search for exact feedrate
                        let adj_refs: Vec<&PerExtruderAdjustments> = by_min_speed[tier_idx..]
                            .iter()
                            .map(|&idx| &*adjustments[idx] as &PerExtruderAdjustments)
                            .collect();
                        let final_feedrate = Self::new_feedrate_to_reach_time_stretch(
                            &adj_refs,
                            feedrate_limit,
                            time_stretch,
                            20,
                        );
                        for i in tier_idx..by_min_speed.len() {
                            let idx = by_min_speed[i];
                            adjustments[idx].slow_down_to_feedrate(final_feedrate);
                        }
                        return;
                    } else {
                        time_stretch -= time_stretch_max;
                        for i in tier_idx..by_min_speed.len() {
                            let idx = by_min_speed[i];
                            adjustments[idx].slow_down_to_feedrate(feedrate_limit);
                        }
                    }
                }

                // Skip to next speed tier
                tier_idx += 1;
                while tier_idx < by_min_speed.len() {
                    let next_idx = by_min_speed[tier_idx];
                    if adjustments[next_idx].slow_down_min_speed < min_speed - EPSILON {
                        break;
                    }
                    tier_idx += 1;
                }
            }

            if feedrate_next == 0.0 {
                break;
            }

            for &idx in &by_min_speed {
                let adj = &mut adjustments[idx];
                adj.idx_line_begin = adj.idx_line_end;
            }
            feedrate = feedrate_next;
        }
    }

    /// Main layer slowdown calculation with algorithm selection
    /// Reference: CoolingBuffer.cpp:290-420
    pub fn calculate_layer_slowdown(
        &self,
        per_extruder_adjustments: &mut [PerExtruderAdjustments],
    ) -> f32 {
        // Calculate total layer time
        let total_time: f64 = per_extruder_adjustments
            .iter()
            .map(|adj| adj.total_time())
            .sum();

        // If we're already at or above minimum layer time, no slowdown needed
        if total_time >= self.config.min_layer_time {
            return 1.0;
        }

        // Calculate how much time can be slowed down
        let total_slowdown_time: f64 = per_extruder_adjustments
            .iter()
            .map(|adj| adj.slowdown_time)
            .sum();

        let fixed_time = total_time - total_slowdown_time;

        // If nothing can be slowed down, return 1.0
        if total_slowdown_time <= 0.0 {
            return 1.0;
        }

        // Calculate required slowdown factor
        let target_slowdown_time = self.config.min_layer_time - fixed_time;
        let slowdown_factor = if target_slowdown_time > 0.0 {
            target_slowdown_time / total_slowdown_time
        } else {
            1.0
        };

        // The slowdown factor is how much longer we need the slowable time to be
        // So if we need 2x the time, we divide speed by 2 (factor = 2)
        let factor = slowdown_factor.max(1.0);

        // Calculate the maximum factor based on minimum print speed
        // We need to check all moves to find the limiting factor
        let mut max_factor = f64::MAX;
        for adj in per_extruder_adjustments.iter() {
            for mov in &adj.moves {
                if mov.can_slowdown && mov.feedrate > 0.0 {
                    let move_max_factor = mov.feedrate / self.config.min_print_speed;
                    max_factor = max_factor.min(move_max_factor);
                }
            }
        }

        // Clamp the factor
        let final_factor = factor.min(max_factor).max(1.0);

        // Apply the slowdown to all extruders
        for adj in per_extruder_adjustments.iter_mut() {
            adj.apply_slowdown(final_factor, self.config.min_print_speed);
        }

        // Sort extruders by slow_down_layer_time
        let mut by_slowdown_time: Vec<usize> = Vec::new();
        let mut elapsed_time_total0 = 0.0f32;

        // Collect adjustable extruders
        for (idx, adj) in per_extruder_adjustments.iter_mut().enumerate() {
            adj.time_total = adj.elapsed_time_total();
            adj.time_maximum = adj.maximum_time_after_slowdown(true);

            if adj.cooling_slow_down_enabled && !adj.moves.is_empty() {
                by_slowdown_time.push(idx);

                // For ConsistentSurface, prepare non-adjustable segments
                if adj.cooling_slowdown_logic == CoolingSlowdownLogic::ConsistentSurface {
                    // Initialize adjustable fields
                    for mov in &mut adj.moves {
                        if mov.can_slowdown {
                            mov.adjustable_length = mov.length;
                            mov.adjustable_time = mov.time;
                            mov.adjustable_time_max = mov.time_max;
                        }
                    }
                    adj.create_non_adjustable_segments(adj.cooling_perimeter_transition_distance);
                }

                if !self.config.slowdown_proportional {
                    adj.sort_lines_by_decreasing_feedrate();
                }
            } else {
                elapsed_time_total0 += adj.elapsed_time_total();
            }
        }

        // Sort by slow_down_layer_time
        by_slowdown_time.sort_by(|&a, &b| {
            per_extruder_adjustments[a]
                .slow_down_layer_time
                .partial_cmp(&per_extruder_adjustments[b].slow_down_layer_time)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Process each extruder tier
        for cur_begin in 0..by_slowdown_time.len() {
            let begin_idx = by_slowdown_time[cur_begin];

            // Calculate current total time
            let mut total = elapsed_time_total0;
            for &idx in &by_slowdown_time[cur_begin..] {
                total += per_extruder_adjustments[idx].time_total;
            }

            let slow_down_layer_time =
                per_extruder_adjustments[begin_idx].slow_down_layer_time * 1.001;

            if total > slow_down_layer_time {
                // No adjustment needed
                continue;
            }

            // Check if we can reach target by slowing down
            let mut max_time = elapsed_time_total0;
            for &idx in &by_slowdown_time[cur_begin..] {
                max_time += per_extruder_adjustments[idx].time_maximum;
            }

            if max_time > slow_down_layer_time {
                let time_stretch = slow_down_layer_time - total;

                // Collect mutable references for this range
                let mut adj_ptrs: Vec<&mut PerExtruderAdjustments> = by_slowdown_time[cur_begin..]
                    .iter()
                    .map(|&idx| &mut per_extruder_adjustments[idx] as *mut PerExtruderAdjustments)
                    .map(|ptr| unsafe { &mut *ptr })
                    .collect();

                // Choose algorithm based on cooling logic
                let first_logic = per_extruder_adjustments[begin_idx].cooling_slowdown_logic;

                if first_logic == CoolingSlowdownLogic::ConsistentSurface {
                    // Two-phase ConsistentSurface slowdown
                    let remaining = Self::extruder_range_slow_down_consistent_surface(
                        &mut adj_ptrs,
                        time_stretch,
                        AdjustableFeatureType::NONE,
                    );

                    if remaining > 0.0 {
                        Self::extruder_range_slow_down_consistent_surface(
                            &mut adj_ptrs,
                            remaining,
                            AdjustableFeatureType::EXTERNAL_PERIMETERS
                                | AdjustableFeatureType::FIRST_INTERNAL_PERIMETERS,
                        );
                    }
                } else if self.config.slowdown_proportional {
                    // Proportional slowdown
                    Self::extruder_range_slow_down_proportional(
                        &mut adj_ptrs,
                        elapsed_time_total0,
                        total,
                        slow_down_layer_time,
                    );
                } else {
                    // Non-proportional slowdown
                    Self::extruder_range_slow_down_non_proportional(&mut adj_ptrs, time_stretch);
                }
            } else {
                // Slow down to maximum
                for &idx in &by_slowdown_time[cur_begin..] {
                    per_extruder_adjustments[idx].slowdown_to_minimum_feedrate(true);
                }
            }

            elapsed_time_total0 += per_extruder_adjustments[begin_idx].elapsed_time_total();
        }

        elapsed_time_total0
    }

    /// Calculate fan speed for a given layer.
    ///
    /// Returns fan speed as a value from 0.0 to 1.0.
    pub fn calculate_fan_speed(&self, layer_index: u32, layer_time: f64) -> f64 {
        // Disable fan for first layers
        if layer_index < self.config.disable_fan_first_layers {
            return 0.0;
        }

        // If layer time is above threshold, no fan needed
        if layer_time >= self.config.fan_below_layer_time {
            return 0.0;
        }

        // Interpolate fan speed based on layer time
        if layer_time <= self.config.full_fan_speed_layer_time {
            // Full fan speed for very fast layers
            self.config.fan_speed
        } else {
            // Linear interpolation between full fan and no fan
            let range = self.config.fan_below_layer_time - self.config.full_fan_speed_layer_time;
            if range > 0.0 {
                let t = (self.config.fan_below_layer_time - layer_time) / range;
                t * self.config.fan_speed
            } else {
                self.config.fan_speed
            }
        }
    }

    /// Get fan speed for bridges.
    pub fn bridge_fan_speed(&self) -> Option<f64> {
        if self.config.bridge_fan_override {
            Some(self.config.bridge_fan_speed)
        } else {
            None
        }
    }

    /// Get fan speed for overhangs.
    pub fn overhang_fan_speed(&self) -> Option<f64> {
        if self.config.overhang_fan_override {
            Some(self.config.overhang_fan_speed)
        } else {
            None
        }
    }

    /// Process a layer's moves and apply cooling adjustments.
    ///
    /// Returns the adjusted moves and the calculated fan speed.
    pub fn process_layer(
        &self,
        layer_index: u32,
        moves: Vec<CoolingMove>,
        extruder_id: u32,
    ) -> CoolingResult {
        let mut adjustments = vec![PerExtruderAdjustments::new(extruder_id as usize)];

        for mov in moves {
            adjustments[0].add_move(mov);
        }

        let original_time = adjustments[0].total_time();
        // Skip expensive slowdown calculation if layer time already exceeds minimum
        let slowdown_factor = if original_time >= self.config.min_layer_time {
            1.0f32
        } else {
            self.calculate_layer_slowdown(&mut adjustments)
        };
        let adjusted_time = if slowdown_factor > 1.0 {
            adjustments[0].adjusted_total_time()
        } else {
            original_time
        };
        let fan_speed = self.calculate_fan_speed(layer_index, adjusted_time);

        CoolingResult {
            moves: adjustments.into_iter().next().unwrap().moves,
            original_time,
            adjusted_time,
            slowdown_factor: slowdown_factor as f64,
            fan_speed,
        }
    }
}

impl Default for CoolingBuffer {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Result of cooling processing for a layer.
#[derive(Debug, Clone)]
pub struct CoolingResult {
    /// Adjusted moves with updated feedrates.
    pub moves: Vec<CoolingMove>,
    /// Original layer time in seconds.
    pub original_time: f64,
    /// Adjusted layer time in seconds.
    pub adjusted_time: f64,
    /// Slowdown factor applied.
    pub slowdown_factor: f64,
    /// Calculated fan speed (0.0 - 1.0).
    pub fan_speed: f64,
}

impl CoolingResult {
    // Check if any slowdown was applied.
    pub fn has_slowdown(&self) -> bool {
        self.slowdown_factor > 1.0
    }

    /// Check if fan is enabled.
    pub fn fan_enabled(&self) -> bool {
        self.fan_speed > 0.0
    }

    /// Get fan speed as percentage (0-100).
    pub fn fan_speed_percent(&self) -> u32 {
        (self.fan_speed * 100.0).round() as u32
    }
}

/// Estimate layer time from path lengths and feedrates.
pub fn estimate_layer_time(
    path_lengths: &[CoordF],
    feedrates: &[CoordF],
    travel_length: CoordF,
    travel_feedrate: CoordF,
) -> f64 {
    let extrusion_time: f64 = path_lengths
        .iter()
        .zip(feedrates.iter())
        .map(|(&len, &feed)| if feed > 0.0 { len / feed } else { 0.0 })
        .sum();

    let travel_time = if travel_feedrate > 0.0 {
        travel_length / travel_feedrate
    } else {
        0.0
    };

    extrusion_time + travel_time
}

// ============================================================================
// G-code Text Post-Processor (CoolingBuffer)
// ============================================================================
// Port of BambuStudio's GCodeEditor::parse_layer_gcode() + write_layer_gcode()
// Architecture: Takes raw G-code text with cooling markers, parses moves,
// applies slowdown via calculate_layer_slowdown(), rewrites speeds and fan.

/// A parsed G-code line for cooling post-processing.
/// Tracks byte offsets into the original G-code string.
/// Reference: GCodeEditor.hpp:38-135
#[derive(Debug, Clone)]
pub(crate) struct CoolingLine {
    line_type: u32,
    line_start: usize,
    line_end: usize,
    length: f32,
    feedrate: f32,
    time: f32,
    time_max: f32,
    slowdown: bool,
    origin_feedrate: f32,
    origin_time_max: f32,
    object_id: i32,
    cooling_node_id: i32,
    outwall_smooth_mark: bool,
    perimeter_index: Option<u16>,
    adjustable_length: f32,
    non_adjustable_length: f32,
    adjustable_time: f32,
    non_adjustable_time: f32,
    adjustable_time_max: f32,
}

impl CoolingLine {
    const TYPE_SET_TOOL: u32 = 1 << 0;
    const TYPE_EXTRUDE_END: u32 = 1 << 1;
    const TYPE_OVERHANG_FAN_START: u32 = 1 << 2;
    const TYPE_OVERHANG_FAN_END: u32 = 1 << 3;
    const TYPE_G0: u32 = 1 << 4;
    const TYPE_G1: u32 = 1 << 5;
    const TYPE_ADJUSTABLE: u32 = 1 << 6;
    const TYPE_EXTERNAL_PERIMETER: u32 = 1 << 7;
    const TYPE_HAS_F: u32 = 1 << 8;
    const TYPE_WIPE: u32 = 1 << 9;
    const TYPE_G4: u32 = 1 << 10;
    const TYPE_G92: u32 = 1 << 11;
    const TYPE_G2: u32 = 1 << 12;
    const TYPE_G3: u32 = 1 << 13;
    const TYPE_FORCE_RESUME_FAN: u32 = 1 << 14;
    const TYPE_SET_FAN_CHANGING_LAYER: u32 = 1 << 15;
    const TYPE_OBJECT_START: u32 = 1 << 16;
    const TYPE_OBJECT_END: u32 = 1 << 17;
    const TYPE_SET_FAN_CHANGING_FILAMENT: u32 = 1 << 18;
    const TYPE_NOT_SET_FAN_CHANGING_FILAMENT: u32 = 1 << 19;
    const TYPE_INTERNAL_PERIMETER: u32 = 1 << 20;
    const TYPE_FIRST_INTERNAL_PERIMETER: u32 = 1 << 21;

    fn new(line_start: usize, line_end: usize) -> Self {
        Self {
            line_type: 0,
            line_start,
            line_end,
            length: 0.0,
            feedrate: 0.0,
            time: 0.0,
            time_max: 0.0,
            slowdown: false,
            origin_feedrate: 0.0,
            origin_time_max: 0.0,
            object_id: -1,
            cooling_node_id: -1,
            outwall_smooth_mark: false,
            perimeter_index: None,
            adjustable_length: 0.0,
            non_adjustable_length: 0.0,
            adjustable_time: 0.0,
            non_adjustable_time: 0.0,
            adjustable_time_max: 0.0,
        }
    }

    fn adjustable(&self) -> bool {
        (self.line_type & Self::TYPE_ADJUSTABLE) != 0 && self.time < self.time_max
    }

    /// Check if this line is adjustable given the additional feature flags.
    /// `additional_flags` is a bitmask: bit 0 = ExternalPerimeters, bit 1 = FirstInternalPerimeters
    fn adjustable_for_features(&self, additional_flags: u32) -> bool {
        if (self.line_type & Self::TYPE_ADJUSTABLE) == 0
            || self.adjustable_time >= self.adjustable_time_max
        {
            return false;
        }
        if (self.line_type & Self::TYPE_EXTERNAL_PERIMETER) != 0 {
            return (additional_flags & 1) != 0; // ExternalPerimeters
        }
        if (self.line_type & Self::TYPE_FIRST_INTERNAL_PERIMETER) != 0 {
            return (additional_flags & 2) != 0; // FirstInternalPerimeters
        }
        true
    }
}

/// Calculate arc length for G2/G3 arc moves.
/// Reference: ArcSegment::calc_arc_length() (GCodeEditor.cpp:239-245)
fn calc_arc_length(start: (f32, f32), end: (f32, f32), center: (f32, f32), is_ccw: bool) -> f32 {
    let radius = ((start.0 - center.0).powi(2) + (start.1 - center.1).powi(2)).sqrt();
    if radius < 1e-10 {
        return 0.0;
    }
    let angle_start = (start.1 - center.1).atan2(start.0 - center.0);
    let angle_end = (end.1 - center.1).atan2(end.0 - center.0);
    let mut angle_swept = if is_ccw {
        angle_end - angle_start
    } else {
        angle_start - angle_end
    };
    if angle_swept < 0.0 {
        angle_swept += 2.0 * std::f32::consts::PI;
    }
    if angle_swept < 1e-10 {
        let dx = end.0 - start.0;
        let dy = end.1 - start.1;
        if dx * dx + dy * dy < 1e-10 {
            angle_swept = 2.0 * std::f32::consts::PI;
        }
    }
    radius * angle_swept
}

/// Per-extruder data for the G-code post-processor.
/// Full version matching C++ PerExtruderAdjustments (GCodeEditor.hpp:137-414)
pub struct PostProcAdjustments {
    extruder_id: u32,
    cooling_slow_down_enabled: bool,
    lines: Vec<CoolingLine>,
    slow_down_min_speed: f32,
    slow_down_layer_time: f32,
    cooling_slowdown_logic: i32,
    cooling_perimeter_transition_distance: f32,
    n_lines_adjustable: usize,
    time_non_adjustable: f32,
    time_total: f32,
    time_maximum: f32,
    idx_line_begin: usize,
    idx_line_end: usize,
}

impl PostProcAdjustments {
    fn elapsed_time_total(&self) -> f32 {
        self.lines.iter().map(|l| l.time).sum()
    }

    fn maximum_time_after_slowdown_bool(&self, slowdown_external: bool) -> f32 {
        let mut t = 0.0f32;
        for line in &self.lines {
            let adj = (line.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0
                && (slowdown_external
                    || (line.line_type & CoolingLine::TYPE_EXTERNAL_PERIMETER) == 0)
                && line.time < line.time_max;
            if adj {
                if line.time_max == f32::MAX {
                    return f32::MAX;
                }
                t += line.time_max;
            } else {
                t += line.time;
            }
        }
        t
    }

    fn non_adjustable_time_bool(&self, slowdown_external: bool) -> f32 {
        self.lines
            .iter()
            .filter(|l| {
                !((l.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0
                    && (slowdown_external
                        || (l.line_type & CoolingLine::TYPE_EXTERNAL_PERIMETER) == 0)
                    && l.time < l.time_max)
            })
            .map(|l| l.time)
            .sum()
    }

    fn adjustable_time_bool(&self, slowdown_external: bool) -> f32 {
        self.lines
            .iter()
            .filter(|l| {
                (l.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0
                    && (slowdown_external
                        || (l.line_type & CoolingLine::TYPE_EXTERNAL_PERIMETER) == 0)
                    && l.time < l.time_max
            })
            .map(|l| l.time)
            .sum()
    }

    fn slowdown_to_minimum_feedrate_bool(&mut self, slowdown_external: bool) -> f32 {
        let mut time_total = 0.0f32;
        for line in &mut self.lines {
            let adj = (line.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0
                && (slowdown_external
                    || (line.line_type & CoolingLine::TYPE_EXTERNAL_PERIMETER) == 0)
                && line.time < line.time_max;
            if adj {
                line.slowdown = true;
                line.time = line.time_max;
                if line.length > 0.0 {
                    line.feedrate = line.length / line.time;
                }
            }
            time_total += line.time;
        }
        time_total
    }

    fn slow_down_proportional(&mut self, factor: f32, slowdown_external: bool) -> f32 {
        let mut time_total = 0.0f32;
        for line in &mut self.lines {
            let adj = (line.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0
                && (slowdown_external
                    || (line.line_type & CoolingLine::TYPE_EXTERNAL_PERIMETER) == 0)
                && line.time < line.time_max;
            if adj {
                line.slowdown = true;
                line.time = line.time_max.min(line.time * factor);
                line.feedrate = line.length / line.time;
            }
            time_total += line.time;
        }
        time_total
    }

    fn sort_lines_by_decreasing_feedrate(&mut self) {
        self.lines.sort_by(|a, b| {
            let adj_a = a.adjustable();
            let adj_b = b.adjustable();
            if adj_a == adj_b {
                b.feedrate
                    .partial_cmp(&a.feedrate)
                    .unwrap_or(std::cmp::Ordering::Equal)
            } else if adj_a {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        });
        self.n_lines_adjustable = self.lines.iter().take_while(|l| l.adjustable()).count();
        self.time_non_adjustable = self.lines[self.n_lines_adjustable..]
            .iter()
            .map(|l| l.time)
            .sum();
    }

    fn time_stretch_when_slowing_down_to_feedrate(&self, min_feedrate: f32) -> f32 {
        let mut stretch = 0.0f32;
        for i in 0..self.n_lines_adjustable {
            let line = &self.lines[i];
            if line.feedrate > min_feedrate {
                stretch += line.time * (line.feedrate / min_feedrate - 1.0);
            }
        }
        stretch
    }

    fn time_stretch_when_slowing_down_to_feedrate_features(
        &self,
        min_feedrate: f32,
        additional: u32,
    ) -> f32 {
        let mut stretch = 0.0f32;
        for i in 0..self.n_lines_adjustable {
            let line = &self.lines[i];
            if line.adjustable_for_features(additional) && line.feedrate > min_feedrate {
                stretch += line.adjustable_time * (line.feedrate / min_feedrate - 1.0);
            }
        }
        stretch
    }

    fn slow_down_to_feedrate(&mut self, min_feedrate: f32) {
        for i in 0..self.n_lines_adjustable {
            let line = &mut self.lines[i];
            if line.feedrate > min_feedrate {
                line.time *= (line.feedrate / min_feedrate).max(1.0);
                line.feedrate = min_feedrate;
                line.slowdown = true;
            }
        }
    }

    fn slow_down_to_feedrate_features(&mut self, min_feedrate: f32, additional: u32) {
        for i in 0..self.n_lines_adjustable {
            let line = &mut self.lines[i];
            if line.adjustable_for_features(additional) && line.feedrate > min_feedrate {
                line.adjustable_time = line.adjustable_length / min_feedrate;
                line.time = line.adjustable_time + line.non_adjustable_time;
                line.feedrate = min_feedrate;
                line.slowdown = true;
            }
        }
    }

    fn slowdown_to_minimum_feedrate(&mut self) {
        for line in &mut self.lines {
            if line.adjustable() {
                line.slowdown = true;
                line.time = line.time_max;
                if line.length > 0.0 {
                    line.feedrate = line.length / line.time;
                }
            }
        }
    }

    fn create_non_adjustable_segments(&mut self, non_adjustable_length: f32) {
        if non_adjustable_length <= 0.0 {
            return;
        }
        let mut accumulated = 0.0f32;
        let sdms = self.slow_down_min_speed;
        for i in (0..self.lines.len()).rev() {
            let line = &mut self.lines[i];
            if (line.line_type & CoolingLine::TYPE_ADJUSTABLE) == 0
                || (line.line_type & CoolingLine::TYPE_EXTRUDE_END) != 0
            {
                accumulated = 0.0;
                continue;
            }
            if line.adjustable_length == 0.0 && line.length > 0.0 {
                line.adjustable_length = line.length;
                line.adjustable_time = line.time;
                line.adjustable_time_max = line.time_max;
            }
            let remaining = non_adjustable_length - accumulated;
            if remaining > 0.0 && line.adjustable_length > 0.0 {
                let convert = line.adjustable_length.min(remaining);
                let ratio = convert / line.adjustable_length;
                line.non_adjustable_length += convert;
                line.non_adjustable_time += line.adjustable_time * ratio;
                line.adjustable_length -= convert;
                line.adjustable_time -= line.adjustable_time * ratio;
                line.adjustable_time_max = if line.adjustable_length > 0.0 && sdms > 0.0 {
                    line.adjustable_length / sdms
                } else {
                    0.0
                };
                accumulated += convert;
            } else {
                accumulated += line.length;
            }
        }
    }
}

/// Parse a G-code axis value: given "X123.456 Y..." and axis 'X', returns 123.456
/// R226 gate for the faithful fan-interpolation floor (see GCodeEditor.cpp:430).
fn zsmooth_fan_floor() -> bool {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *G.get_or_init(|| crate::faithful_gate("ZSMOOTH_FAITHFUL"))
}

fn parse_axis(line: &str, axis: char) -> Option<f32> {
    let bytes = line.as_bytes();
    let axis_byte = axis as u8;
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == axis_byte {
            i += 1;
            let start = i;
            while i < bytes.len()
                && (bytes[i] == b'-' || bytes[i] == b'.' || bytes[i].is_ascii_digit())
            {
                i += 1;
            }
            if start < i {
                return line[start..i].parse().ok();
            }
        }
        i += 1;
    }
    None
}

/// Fan speed format for G-code output.
fn format_set_fan(fan_speed: i32) -> String {
    if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        // GCodeWriter::set_fan (GCodeWriter.cpp:862-894), default flavor:
        // speed==0 -> "M106 S0"; else "M106 S" << 255.0*speed/100.0 (ostream
        // %g -> fractional, e.g. 79% -> 201.45).
        if fan_speed <= 0 {
            return "M106 S0\n".to_string();
        }
        let s = 255.0f64 * fan_speed as f64 / 100.0;
        let mut t = format!("{:.6}", s);
        if t.contains('.') {
            t = t.trim_end_matches('0').trim_end_matches('.').to_string();
        }
        return format!("M106 S{}\n", t);
    }
    if fan_speed > 0 {
        let s_val = ((fan_speed as f32) * 255.0 / 100.0).round() as i32;
        format!("M106 S{}\n", s_val.min(255).max(0))
    } else {
        "M107\n".to_string()
    }
}

/// Format M106 P2 Sxxx for additional (auxiliary) fan.
/// GCodeWriter.cpp:907 — `(int)(255.0 * speed / 100.0)`, a TRUNCATION. R647:
/// this rounded instead, so the first configured value that lands on a .5
/// boundary diverged (70% → 178.5: C++ 178, ours 179).
fn format_set_additional_fan(fan_speed: i32) -> String {
    let s_val = (255.0 * fan_speed as f64 / 100.0) as i32;
    format!("M106 P2 S{}\n", s_val.min(255).max(0))
}

/// Fan set type, matching C++ SetFanType enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SetFanType {
    ChangingLayer,
    ChangingFilament,
    ImmediatelyApply,
}

/// Stateful G-code editor for cooling post-processing.
/// Accumulates support layer G-code and processes on flush.
/// Reference: GCodeEditor class (GCodeEditor.hpp:424-481)
pub struct GCodeEditorState {
    /// Accumulated G-code from non-flushed layers (support layers)
    pub m_gcode: String,
    /// Current position: X, Y, Z, E, F, I, J
    pub m_current_pos: [f32; 7],
    /// Current known fan speed or -1 if not known yet
    pub m_fan_speed: i32,
    /// Current additional (auxiliary) fan speed or -1
    pub m_additional_fan_speed: i32,
    /// Cached current fan speed for overhang logic
    pub m_current_fan_speed: i32,
    /// Current extruder index
    pub m_current_extruder: u32,
    /// Parse-phase extruder tracker
    pub m_parse_gcode_extruder: u32,
    /// Flag: fan speed needs to be emitted at layer change marker
    pub m_set_fan_changing_layer: bool,
    /// Flag: additional fan speed needs to be emitted at layer change
    pub m_set_addition_fan_changing_layer: bool,
    /// Flag: fan setting for filament change is active
    pub m_set_fan_changing_filament_start: bool,
}

impl Default for GCodeEditorState {
    fn default() -> Self {
        Self {
            m_gcode: String::new(),
            m_current_pos: [0.0; 7],
            m_fan_speed: -1,
            m_additional_fan_speed: -1,
            m_current_fan_speed: -1,
            m_current_extruder: 0,
            m_parse_gcode_extruder: 0,
            m_set_fan_changing_layer: false,
            m_set_addition_fan_changing_layer: false,
            m_set_fan_changing_filament_start: true,
        }
    }
}

impl GCodeEditorState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset position state.
    /// Reference: GCodeEditor::reset()
    pub fn reset(&mut self, position: [f32; 3], default_feedrate: f32) {
        self.m_current_pos = [
            position[0],
            position[1],
            position[2],
            0.0,
            default_feedrate,
            0.0,
            0.0,
        ];
        self.m_fan_speed = -1;
        self.m_additional_fan_speed = -1;
        self.m_current_fan_speed = -1;
    }

    /// Accumulate or flush layer G-code.
    /// Reference: GCodeEditor::process_layer() (GCodeEditor.cpp:71-95)
    ///
    /// When `flush` is false, just accumulates the gcode for later.
    /// When `flush` is true, processes all accumulated + current gcode.
    pub fn process_layer(
        &mut self,
        gcode: &str,
        layer_id: usize,
        extruder_configs: &[PerExtruderCoolingConfig],
        cooling_logic_proportional: bool,
        auxiliary_fan: bool,
        toolchange_prefix: &str,
        use_relative_e: bool,
        object_label: &[i32],
        flush: bool,
        spiral_vase: bool,
    ) -> String {
        // Accumulate
        if self.m_gcode.is_empty() {
            self.m_gcode = gcode.to_string();
        } else {
            self.m_gcode.push_str(gcode);
        }

        if !flush {
            return String::new();
        }

        // Flush: process accumulated gcode
        let full_gcode = std::mem::take(&mut self.m_gcode);

        // Parse into per-extruder adjustments
        let (per_extruder_adjustments, not_set_additional_fan) = self.parse_layer_gcode(
            &full_gcode,
            extruder_configs,
            toolchange_prefix,
            use_relative_e,
            object_label,
            spiral_vase,
            layer_id > 0,
        );

        // Calculate slowdown
        let mut adjustments = per_extruder_adjustments;
        let layer_time =
            calculate_layer_slowdown_postproc(&mut adjustments, cooling_logic_proportional);

        // Write output
        self.write_layer_gcode(
            &full_gcode,
            not_set_additional_fan,
            layer_id,
            layer_time,
            &adjustments,
            extruder_configs,
            auxiliary_fan,
            toolchange_prefix,
        )
    }

    /// Two-phase variant, phase 1 (GCode.cpp:3396-3417 pipeline 1: parse →
    /// calculate_layer_slowdown, NO write). Used by the z-direction outwall
    /// smoothing path: all layers are parsed first, the SmoothCalculator runs
    /// over the collected wall nodes, then `write_parsed_layer` rewrites each
    /// layer with the (possibly smoothing-clamped) feedrates.
    #[allow(clippy::too_many_arguments)]
    pub fn process_layer_parse_only(
        &mut self,
        gcode: &str,
        layer_id: usize,
        extruder_configs: &[PerExtruderCoolingConfig],
        cooling_logic_proportional: bool,
        toolchange_prefix: &str,
        use_relative_e: bool,
        object_label: &[i32],
        spiral_vase: bool,
    ) -> ParsedLayer {
        let full_gcode = gcode.to_string();
        let (per_extruder_adjustments, not_set_additional_fan) = self.parse_layer_gcode(
            &full_gcode,
            extruder_configs,
            toolchange_prefix,
            use_relative_e,
            object_label,
            spiral_vase,
            layer_id > 0,
        );
        let mut adjustments = per_extruder_adjustments;
        let layer_time =
            calculate_layer_slowdown_postproc(&mut adjustments, cooling_logic_proportional);
        ParsedLayer {
            gcode: full_gcode,
            adjustments,
            not_set_additional_fan,
            layer_time,
            layer_id,
        }
    }

    /// Two-phase variant, phase 2 (GCode.cpp write_gocde filter).
    pub fn write_parsed_layer(
        &mut self,
        parsed: &ParsedLayer,
        extruder_configs: &[PerExtruderCoolingConfig],
        auxiliary_fan: bool,
        toolchange_prefix: &str,
    ) -> String {
        self.write_layer_gcode(
            &parsed.gcode,
            parsed.not_set_additional_fan,
            parsed.layer_id,
            parsed.layer_time,
            &parsed.adjustments,
            extruder_configs,
            auxiliary_fan,
            toolchange_prefix,
        )
    }

    /// Parse layer G-code into per-extruder CoolingLine vectors.
    /// Reference: GCodeEditor::parse_layer_gcode() (GCodeEditor.cpp:100-349)
    fn parse_layer_gcode(
        &mut self,
        gcode: &str,
        extruder_configs: &[PerExtruderCoolingConfig],
        toolchange_prefix: &str,
        use_relative_e: bool,
        _object_label: &[i32],
        _spiral_vase: bool,
        _join_z_smooth: bool,
    ) -> (Vec<PostProcAdjustments>, bool) {
        let num_extruders = extruder_configs.len().max(1);
        // BambuStudio CoolingBuffer: first layer (layer_id == 0) skips speed slowdown.
        // _join_z_smooth = layer_id > 0; when false we are on the first layer.
        let is_first_layer = !_join_z_smooth;
        let mut per_extruder_adjustments: Vec<PostProcAdjustments> = (0..num_extruders)
            .map(|i| {
                let cfg = &extruder_configs[i.min(extruder_configs.len() - 1)];
                PostProcAdjustments {
                    extruder_id: i as u32,
                    // First layer: disable slowdown so initial_layer_speed is preserved.
                    cooling_slow_down_enabled: if is_first_layer {
                        false
                    } else {
                        cfg.slow_down_for_layer_cooling
                    },
                    lines: Vec::new(),
                    slow_down_min_speed: cfg.slow_down_min_speed,
                    slow_down_layer_time: cfg.slow_down_layer_time,
                    cooling_slowdown_logic: cfg.cooling_slowdown_logic,
                    cooling_perimeter_transition_distance: cfg
                        .cooling_perimeter_transition_distance,
                    n_lines_adjustable: 0,
                    time_non_adjustable: 0.0,
                    time_total: 0.0,
                    time_maximum: 0.0,
                    idx_line_begin: 0,
                    idx_line_end: 0,
                }
            })
            .collect();

        let mut not_set_additional_fan = false;
        let mut current_extruder = self.m_parse_gcode_extruder as usize;
        if current_extruder >= num_extruders {
            current_extruder = 0;
        }
        let mut adj_idx = current_extruder.min(num_extruders - 1);
        let mut active_speed_modifier: Option<usize> = None;
        let mut not_join_cooling = false;
        // GCodeEditor.cpp:133-140 — outwall smooth-mark state
        let mut object_id: i32 = -1;
        let mut cooling_node_id: i32 = -1;
        let current_pos = &mut self.m_current_pos;

        let bytes = gcode.as_bytes();
        let mut line_start_off = 0usize;

        while line_start_off < bytes.len() {
            let mut line_end_off = line_start_off;
            while line_end_off < bytes.len() && bytes[line_end_off] != b'\n' {
                line_end_off += 1;
            }
            let sline_end = line_end_off;
            if line_end_off < bytes.len() {
                line_end_off += 1; // include \n
            }

            let sline = &gcode[line_start_off..sline_end];
            let mut cl = CoolingLine::new(line_start_off, line_end_off);

            // Identify line type
            if sline.starts_with("G0 ") {
                cl.line_type = CoolingLine::TYPE_G0;
            } else if sline.starts_with("G1 ") {
                cl.line_type = CoolingLine::TYPE_G1;
            } else if sline.starts_with("G92 ") {
                cl.line_type = CoolingLine::TYPE_G92;
            } else if sline.starts_with("G2 ") {
                cl.line_type = CoolingLine::TYPE_G2;
            } else if sline.starts_with("G3 ") {
                cl.line_type = CoolingLine::TYPE_G3;
            } else if sline.starts_with("; OBJECT_ID: ") {
                // GCodeEditor.cpp:163-166
                object_id = sline["; OBJECT_ID: ".len()..].trim().parse().unwrap_or(-1);
            } else if sline.starts_with("; COOLING_NODE: ") {
                // GCodeEditor.cpp:166-169
                cooling_node_id = sline["; COOLING_NODE: ".len()..]
                    .trim()
                    .parse()
                    .unwrap_or(-1);
            } else if sline.contains(";not reset fan") {
                not_set_additional_fan = true;
            }

            if cl.line_type != 0 {
                // Parse axis values: X, Y, Z, E, F, I, J
                let mut new_pos = *current_pos;
                if let Some(x) = parse_axis(sline, 'X') {
                    new_pos[0] = x;
                }
                if let Some(y) = parse_axis(sline, 'Y') {
                    new_pos[1] = y;
                }
                if let Some(z) = parse_axis(sline, 'Z') {
                    new_pos[2] = z;
                }
                if let Some(e) = parse_axis(sline, 'E') {
                    new_pos[3] = e;
                }
                if let Some(f) = parse_axis(sline, 'F') {
                    new_pos[4] = f / 60.0; // mm/min to mm/s
                    if (cl.line_type & CoolingLine::TYPE_G92) == 0 {
                        cl.line_type |= CoolingLine::TYPE_HAS_F;
                    }
                }
                // I and J are relative offsets from current position to arc center
                // Reference: GCodeEditor.cpp:186-199
                if let Some(i_val) = parse_axis(sline, 'I') {
                    new_pos[5] = current_pos[0] + i_val;
                }
                if let Some(j_val) = parse_axis(sline, 'J') {
                    new_pos[6] = current_pos[1] + j_val;
                }

                let is_external = sline.contains(";_EXTERNAL_PERIMETER");
                let is_wipe = sline.contains(";_WIPE");

                if is_wipe {
                    cl.line_type |= CoolingLine::TYPE_WIPE;
                }

                let is_new_speed_modifier =
                    sline.contains(";_EXTRUDE_SET_SPEED") && !is_wipe && !not_join_cooling;
                if is_new_speed_modifier {
                    cl.line_type |= CoolingLine::TYPE_ADJUSTABLE;
                    active_speed_modifier = Some(per_extruder_adjustments[adj_idx].lines.len());
                }

                if is_external {
                    cl.line_type |= CoolingLine::TYPE_EXTERNAL_PERIMETER;
                    // Don't slowdown external perimeters if config says so
                    let ecfg = &extruder_configs[adj_idx.min(extruder_configs.len() - 1)];
                    if ecfg.no_slow_down_for_cooling_on_outwalls {
                        cl.line_type &= !CoolingLine::TYPE_ADJUSTABLE;
                    }
                    // GCodeEditor.cpp:222-227 mark_node_pos + :44-52
                    // record_wall_lines — native records line_idx and stamps the
                    // SAME line on the next iteration; collapsed to a direct set.
                    if (cl.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0
                        && _join_z_smooth
                        && !_spiral_vase
                        && cooling_node_id != -1
                    {
                        if let Some(object_idx) =
                            _object_label.iter().position(|&l| l == object_id)
                        {
                            cl.outwall_smooth_mark = true;
                            cl.object_id = object_idx as i32;
                            cl.cooling_node_id = cooling_node_id;
                        }
                    }
                }

                // Check for internal perimeter markers (ConsistentSurface)
                if sline.contains(";_INTERNAL_PERIMETER") {
                    cl.line_type |= CoolingLine::TYPE_INTERNAL_PERIMETER;
                }
                if sline.contains(";_FIRST_INTERNAL_PERIMETER") {
                    cl.line_type |= CoolingLine::TYPE_FIRST_INTERNAL_PERIMETER;
                }

                if (cl.line_type & CoolingLine::TYPE_G92) == 0 {
                    // G0/G1/G2/G3: Calculate duration
                    if use_relative_e {
                        current_pos[3] = 0.0;
                    }

                    let dif = [
                        new_pos[0] - current_pos[0],
                        new_pos[1] - current_pos[1],
                        new_pos[2] - current_pos[2],
                        new_pos[3] - current_pos[3],
                    ];

                    // Arc length for G2/G3, chord for G0/G1
                    let dxy2 = if (cl.line_type & CoolingLine::TYPE_G2) != 0
                        || (cl.line_type & CoolingLine::TYPE_G3) != 0
                    {
                        let start = (current_pos[0], current_pos[1]);
                        let end = (new_pos[0], new_pos[1]);
                        let center = (new_pos[5], new_pos[6]);
                        let is_ccw = (cl.line_type & CoolingLine::TYPE_G3) != 0;
                        let arc_len = calc_arc_length(start, end, center, is_ccw);
                        arc_len * arc_len
                    } else {
                        dif[0] * dif[0] + dif[1] * dif[1]
                    };

                    let dxyz2 = dxy2 + dif[2] * dif[2];

                    if dxyz2 > 0.0 {
                        cl.length = dxyz2.sqrt();
                    } else if dif[3].abs() > 0.0 {
                        cl.length = dif[3].abs();
                    }

                    cl.feedrate = new_pos[4];
                    cl.origin_feedrate = new_pos[4];

                    if cl.length > 0.0 && cl.feedrate > 0.0 {
                        cl.time = cl.length / cl.feedrate;
                    }
                    if cl.feedrate == 0.0 {
                        cl.time = 0.0;
                    }

                    let adj = &per_extruder_adjustments[adj_idx];
                    cl.time_max = cl.time;
                    if (cl.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0
                        || active_speed_modifier.is_some()
                    {
                        cl.time_max = if adj.slow_down_min_speed == 0.0 {
                            f32::MAX
                        } else {
                            cl.time.max(cl.length / adj.slow_down_min_speed)
                        };
                    }
                    cl.origin_time_max = cl.time_max;

                    // Merge into active speed modifier if inside adjustable block
                    if !is_new_speed_modifier {
                        if let Some(sm_local_idx) = active_speed_modifier {
                            let lines = &mut per_extruder_adjustments[adj_idx].lines;
                            if sm_local_idx < lines.len()
                                && (cl.line_type
                                    & (CoolingLine::TYPE_G1
                                        | CoolingLine::TYPE_G2
                                        | CoolingLine::TYPE_G3))
                                    != 0
                            {
                                let sm = &mut lines[sm_local_idx];
                                sm.length += cl.length;
                                sm.time += cl.time;
                                if sm.time_max != f32::MAX {
                                    if cl.time_max == f32::MAX {
                                        sm.time_max = f32::MAX;
                                    } else {
                                        sm.time_max += cl.time_max;
                                    }
                                    sm.origin_time_max = sm.time_max;
                                }
                                cl.line_type = 0;
                            }
                        }
                    }
                }
                *current_pos = new_pos;
            } else if sline.starts_with("; Slow Down Start") {
                not_join_cooling = true;
            } else if sline.starts_with("; Slow Down End") {
                not_join_cooling = false;
            } else if sline.starts_with(";_EXTRUDE_END") {
                cl.line_type = CoolingLine::TYPE_EXTRUDE_END;
                active_speed_modifier = None;
            } else if sline.starts_with(toolchange_prefix) && !toolchange_prefix.is_empty() {
                // Tool change line
                if let Ok(new_ext) = sline[toolchange_prefix.len()..].trim().parse::<u32>() {
                    if (new_ext as usize) < num_extruders && new_ext as usize != current_extruder {
                        cl.line_type = CoolingLine::TYPE_SET_TOOL;
                        current_extruder = new_ext as usize;
                        adj_idx = current_extruder;
                    }
                }
            } else if sline.starts_with(";_OVERHANG_FAN_START") {
                cl.line_type = CoolingLine::TYPE_OVERHANG_FAN_START;
            } else if sline.starts_with(";_OVERHANG_FAN_END") {
                cl.line_type = CoolingLine::TYPE_OVERHANG_FAN_END;
            } else if sline.starts_with("G4 ") {
                cl.line_type = CoolingLine::TYPE_G4;
                if let Some(s) = parse_axis(sline, 'S') {
                    cl.time = s;
                    cl.time_max = s;
                } else if let Some(p) = parse_axis(sline, 'P') {
                    cl.time = p * 0.001;
                    cl.time_max = p * 0.001;
                }
                cl.origin_time_max = cl.time_max;
            } else if sline.starts_with(";_FORCE_RESUME_FAN_SPEED") {
                cl.line_type = CoolingLine::TYPE_FORCE_RESUME_FAN;
            } else if sline.starts_with(";_SET_FAN_SPEED_CHANGING_LAYER") {
                cl.line_type = CoolingLine::TYPE_SET_FAN_CHANGING_LAYER;
            } else if sline.starts_with("M624") {
                cl.line_type = CoolingLine::TYPE_OBJECT_START;
            } else if sline.starts_with("M625") {
                cl.line_type = CoolingLine::TYPE_OBJECT_END;
            } else if sline.contains(";set fan changing filament") {
                cl.line_type = CoolingLine::TYPE_SET_FAN_CHANGING_FILAMENT;
            } else if sline.contains(";not set fan changing filament") {
                cl.line_type = CoolingLine::TYPE_NOT_SET_FAN_CHANGING_FILAMENT;
            }

            if cl.line_type != 0 {
                let is_speed_mod = (cl.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0
                    && (cl.line_type
                        & (CoolingLine::TYPE_G0
                            | CoolingLine::TYPE_G1
                            | CoolingLine::TYPE_G2
                            | CoolingLine::TYPE_G3))
                        != 0;
                per_extruder_adjustments[adj_idx].lines.push(cl);
                if is_speed_mod {
                    active_speed_modifier = Some(per_extruder_adjustments[adj_idx].lines.len() - 1);
                }
            }

            line_start_off = line_end_off;
        }

        self.m_parse_gcode_extruder = current_extruder as u32;
        (per_extruder_adjustments, not_set_additional_fan)
    }

    /// Write the output G-code with adjusted feedrates and fan commands.
    /// Reference: GCodeEditor::write_layer_gcode() (GCodeEditor.cpp:352-642)
    fn write_layer_gcode(
        &mut self,
        gcode: &str,
        not_set_additional_fan: bool,
        layer_id: usize,
        layer_time: f32,
        per_extruder_adjustments: &[PostProcAdjustments],
        extruder_configs: &[PerExtruderCoolingConfig],
        auxiliary_fan: bool,
        toolchange_prefix: &str,
    ) -> String {
        if gcode.is_empty() {
            return String::new();
        }

        // Flatten and sort all lines by position
        let mut all_lines: Vec<&CoolingLine> = Vec::new();
        for adj in per_extruder_adjustments {
            for line in &adj.lines {
                all_lines.push(line);
            }
        }
        all_lines.sort_by_key(|l| l.line_start);

        let mut new_gcode = String::with_capacity(gcode.len() * 2);
        let mut overhang_fan_control = false;
        let mut overhang_fan_speed = 0i32;
        let mut pre_start_overhang_fan_time = 0.0f32;

        // change_extruder_set_fan lambda equivalent
        // Reference: GCodeEditor.cpp:393-460
        let compute_fan =
            |state: &mut GCodeEditorState,
             set_type: SetFanType,
             new_gcode: &mut String,
             overhang_fan_control: &mut bool,
             overhang_fan_speed: &mut i32,
             pre_start_overhang_fan_time: &mut f32,
             not_set_additional_fan: bool,
             auxiliary_fan: bool,
             extruder_configs: &[PerExtruderCoolingConfig]| {
                let ext = state.m_current_extruder as usize;
                let cfg = &extruder_configs[ext.min(extruder_configs.len() - 1)];

                let fan_min_speed = cfg.fan_min_speed;
                let mut fan_speed_new = if cfg.reduce_fan_stop_start_freq {
                    fan_min_speed
                } else {
                    0
                };
                let mut additional_fan_speed_new = cfg.additional_cooling_fan_speed;
                let mut close_fan_first = cfg.close_fan_the_first_x_layers;
                let full_fan_speed_layer = cfg.full_fan_speed_layer;

                if close_fan_first <= 0 && full_fan_speed_layer > 0 {
                    close_fan_first = 1;
                }

                if (layer_id as i32) >= close_fan_first {
                    let fan_max_speed = cfg.fan_max_speed;
                    let slow_down_lt = cfg.slow_down_layer_time;
                    let fan_cooling_lt = cfg.fan_cooling_layer_time;

                    if layer_time < slow_down_lt {
                        fan_speed_new = fan_max_speed;
                    } else if layer_time < fan_cooling_lt {
                        if zsmooth_fan_floor() {
                            // R226: native is `int(floor(t*min + (1-t)*max) + 0.5)`
                            // (GCodeEditor.cpp:430) — the +0.5 is DEAD after
                            // floor(): floor yields an integer, +0.5 truncates
                            // back. Effective op = floor, not round (79.9 → 79;
                            // rust's round gave 80 → the 1272× S201.45-vs-S204
                            // duty class). t is float math widened to double.
                            let t = ((layer_time - slow_down_lt)
                                / (fan_cooling_lt - slow_down_lt))
                                as f64;
                            fan_speed_new = (t * fan_min_speed as f64
                                + (1.0 - t) * fan_max_speed as f64)
                                .floor() as i32;
                        } else {
                            let t =
                                (layer_time - slow_down_lt) / (fan_cooling_lt - slow_down_lt);
                            fan_speed_new = (t * fan_min_speed as f32
                                + (1.0 - t) * fan_max_speed as f32
                                + 0.5) as i32;
                        }
                    }

                    *overhang_fan_speed = cfg.overhang_fan_speed;

                    if (layer_id as i32) >= close_fan_first
                        && (layer_id as i32 + 1) < full_fan_speed_layer
                    {
                        let factor = (layer_id as i32 + 1 - close_fan_first) as f32
                            / (full_fan_speed_layer - close_fan_first) as f32;
                        fan_speed_new =
                            (fan_speed_new as f32 * factor + 0.5).clamp(0.0, 255.0) as i32;
                        *overhang_fan_speed =
                            (*overhang_fan_speed as f32 * factor + 0.5).clamp(0.0, 255.0) as i32;
                    }

                    *overhang_fan_control = *overhang_fan_speed > fan_speed_new;
                } else {
                    *overhang_fan_control = false;
                    *overhang_fan_speed = 0;
                    fan_speed_new = 0;
                    additional_fan_speed_new = cfg.first_x_layer_fan_speed;
                }

                if std::env::var("FANDBG").is_ok() && layer_id < 6 {
                    eprintln!(
                        "FANDBG l={} lt={:.2} min={} max={} cool_lt={} slow_lt={} close={} new={}",
                        layer_id, layer_time, cfg.fan_min_speed, cfg.fan_max_speed,
                        cfg.fan_cooling_layer_time, cfg.slow_down_layer_time,
                        close_fan_first, fan_speed_new
                    );
                }
                if fan_speed_new != state.m_fan_speed {
                    state.m_fan_speed = fan_speed_new;
                    state.m_current_fan_speed = fan_speed_new;
                    match set_type {
                        SetFanType::ImmediatelyApply => {
                            new_gcode.push_str(&format_set_fan(state.m_fan_speed));
                        }
                        SetFanType::ChangingLayer => {
                            state.m_set_fan_changing_layer = true;
                        }
                        SetFanType::ChangingFilament => {}
                    }
                }

                if additional_fan_speed_new != state.m_additional_fan_speed {
                    state.m_additional_fan_speed = additional_fan_speed_new;
                    match set_type {
                        SetFanType::ImmediatelyApply
                            if state.m_set_fan_changing_filament_start && auxiliary_fan =>
                        {
                            new_gcode
                                .push_str(&format_set_additional_fan(state.m_additional_fan_speed));
                        }
                        SetFanType::ChangingLayer if !not_set_additional_fan => {
                            state.m_set_addition_fan_changing_layer = true;
                        }
                        _ => {}
                    }
                }

                *pre_start_overhang_fan_time = if *overhang_fan_control {
                    cfg.pre_start_fan_time
                } else {
                    0.0
                };
            };

        let mut current_feedrate = 0i32;
        self.m_set_fan_changing_layer = false;
        self.m_set_addition_fan_changing_layer = false;

        // Initial fan calculation
        compute_fan(
            self,
            SetFanType::ChangingLayer,
            &mut new_gcode,
            &mut overhang_fan_control,
            &mut overhang_fan_speed,
            &mut pre_start_overhang_fan_time,
            not_set_additional_fan,
            auxiliary_fan,
            extruder_configs,
        );

        // Overhang fan pre-start lookahead
        // Reference: GCodeEditor.cpp:469-494
        let mut cumulative_time = 0.0f32;
        let mut search_time = 0.0f32;
        let mut j = 0usize;

        let mut pos = 0usize;

        for i in 0..all_lines.len() {
            let line = all_lines[i];

            // Pre-start fan lookahead
            if pre_start_overhang_fan_time > 0.0 && overhang_fan_speed > self.m_fan_speed {
                cumulative_time += line.time;
                if j < i {
                    j = i;
                }
                if search_time < cumulative_time {
                    search_time = cumulative_time;
                }

                while search_time - cumulative_time < pre_start_overhang_fan_time
                    && j < all_lines.len()
                    && overhang_fan_control
                    && self.m_current_fan_speed < overhang_fan_speed
                {
                    let line_iter = all_lines[j];
                    if (line_iter.line_type & CoolingLine::TYPE_FORCE_RESUME_FAN) != 0 {
                        break;
                    }
                    search_time += line_iter.time;
                    if (line_iter.line_type & CoolingLine::TYPE_OVERHANG_FAN_START) != 0 {
                        self.m_current_fan_speed = overhang_fan_speed;
                        new_gcode.push_str(&format_set_fan(overhang_fan_speed));
                        break;
                    }
                    j += 1;
                }
            }

            let line_start_bytes = &gcode[line.line_start..line.line_end];

            // Append text between parsed lines
            if line.line_start > pos {
                new_gcode.push_str(&gcode[pos..line.line_start]);
            }

            if (line.line_type & CoolingLine::TYPE_SET_FAN_CHANGING_FILAMENT) != 0 {
                self.m_set_fan_changing_filament_start = true;
            } else if (line.line_type & CoolingLine::TYPE_NOT_SET_FAN_CHANGING_FILAMENT) != 0 {
                self.m_set_fan_changing_filament_start = false;
            } else if (line.line_type & CoolingLine::TYPE_SET_TOOL) != 0 {
                // Tool change
                let sline = &gcode[line.line_start..line.line_end];
                if let Ok(new_ext) = sline
                    .trim_start_matches(toolchange_prefix)
                    .trim()
                    .parse::<u32>()
                {
                    if new_ext != self.m_current_extruder {
                        self.m_current_extruder = new_ext;
                        compute_fan(
                            self,
                            SetFanType::ChangingFilament,
                            &mut new_gcode,
                            &mut overhang_fan_control,
                            &mut overhang_fan_speed,
                            &mut pre_start_overhang_fan_time,
                            not_set_additional_fan,
                            auxiliary_fan,
                            extruder_configs,
                        );
                        cumulative_time = 0.0;
                        search_time = 0.0;
                    }
                }
                new_gcode.push_str(line_start_bytes);
            } else if (line.line_type & CoolingLine::TYPE_OVERHANG_FAN_START) != 0 {
                if overhang_fan_control && self.m_current_fan_speed < overhang_fan_speed {
                    self.m_current_fan_speed = overhang_fan_speed;
                    new_gcode.push_str(&format_set_fan(overhang_fan_speed));
                }
            } else if (line.line_type & CoolingLine::TYPE_OVERHANG_FAN_END) != 0 {
                if overhang_fan_control {
                    self.m_current_fan_speed = self.m_fan_speed;
                    new_gcode.push_str(&format_set_fan(self.m_fan_speed));
                }
            } else if (line.line_type & CoolingLine::TYPE_FORCE_RESUME_FAN) != 0 {
                if self.m_current_fan_speed != -1 {
                    new_gcode.push_str(&format_set_fan(self.m_current_fan_speed));
                }
                if self.m_additional_fan_speed != -1
                    && self.m_set_fan_changing_filament_start
                    && auxiliary_fan
                {
                    new_gcode.push_str(&format_set_additional_fan(self.m_additional_fan_speed));
                }
            } else if (line.line_type & CoolingLine::TYPE_SET_FAN_CHANGING_LAYER) != 0 {
                if self.m_current_fan_speed != -1 && self.m_set_fan_changing_layer {
                    new_gcode.push_str(&format_set_fan(self.m_current_fan_speed));
                    self.m_set_fan_changing_layer = false;
                }
                if self.m_additional_fan_speed != -1
                    && self.m_set_addition_fan_changing_layer
                    && auxiliary_fan
                {
                    new_gcode.push_str(&format_set_additional_fan(self.m_additional_fan_speed));
                    self.m_set_addition_fan_changing_layer = false;
                }
            } else if (line.line_type & CoolingLine::TYPE_EXTRUDE_END) != 0 {
                // Strip marker
            } else if (line.line_type
                & (CoolingLine::TYPE_ADJUSTABLE
                    | CoolingLine::TYPE_EXTERNAL_PERIMETER
                    | CoolingLine::TYPE_WIPE
                    | CoolingLine::TYPE_HAS_F))
                != 0
            {
                // Line with feedrate that may need adjustment
                let sline = line_start_bytes.trim_end_matches('\n');

                // Find comment start
                let end_pos = sline.find(';').unwrap_or(sline.len());

                if let Some(f_offset) = sline[..end_pos].find(" F") {
                    let fpos = f_offset + 2;
                    let new_feedrate = if line.slowdown {
                        (60.0 * line.feedrate + 0.5) as i32
                    } else {
                        let f_str = &sline[fpos..];
                        let f_end = f_str
                            .find(|c: char| !c.is_ascii_digit() && c != '.' && c != '-')
                            .unwrap_or(f_str.len());
                        f_str[..f_end]
                            .parse::<f64>()
                            .unwrap_or(current_feedrate as f64) as i32
                    };

                    let modify = line.slowdown && new_feedrate != current_feedrate;
                    let remove = !line.slowdown
                        && new_feedrate == current_feedrate
                        && (line.line_type
                            & (CoolingLine::TYPE_ADJUSTABLE
                                | CoolingLine::TYPE_EXTERNAL_PERIMETER
                                | CoolingLine::TYPE_WIPE))
                            == 0
                        && line.length > 0.0;

                    if new_feedrate == current_feedrate {
                        if (line.line_type
                            & (CoolingLine::TYPE_ADJUSTABLE
                                | CoolingLine::TYPE_EXTERNAL_PERIMETER
                                | CoolingLine::TYPE_WIPE))
                            != 0
                            || line.length == 0.0
                        {
                            // Skip entire line
                        } else if remove {
                            // Remove F param
                            let clean = strip_cooling_markers(sline);
                            let without_f = remove_f_param(&clean);
                            if !without_f.is_empty() && without_f != "G1" && without_f != "G0" {
                                new_gcode.push_str(&without_f);
                                new_gcode.push('\n');
                            }
                        } else {
                            let clean = strip_cooling_markers(sline);
                            let without_f = remove_f_param(&clean);
                            if !without_f.is_empty() && without_f != "G1" && without_f != "G0" {
                                new_gcode.push_str(&without_f);
                                new_gcode.push('\n');
                            }
                        }
                    } else if modify {
                        current_feedrate = new_feedrate;
                        new_gcode.push_str(&sline[..f_offset + 2]);
                        new_gcode.push_str(&new_feedrate.to_string());
                        // Skip old F value
                        let after_f = &sline[fpos..];
                        let f_end = after_f
                            .find(|c: char| c == ' ' || c == ';' || c == '\n')
                            .unwrap_or(after_f.len());
                        // R643: `rest` stopped at `end_pos` (the `;`), silently
                        // dropping the inline comment. C++'s cooling buffer
                        // rewrites the F value in place and keeps the remainder of
                        // the line, comment included. Measured cost of truncating:
                        // Majora carried 12,249 F-bearing lines with an inline
                        // comment in C++ against 18 in ours, while comment-less
                        // lines matched at 7,482 vs 7,475 — the whole gap was here.
                        let rest = &after_f[f_end..];
                        let clean_rest = strip_cooling_markers_str(rest);
                        if !clean_rest.is_empty() {
                            new_gcode.push_str(&clean_rest);
                        }
                        new_gcode.push('\n');
                    } else {
                        // R643: this truncated at `;` and the comment above said
                        // so outright. C++ keeps the comment here too — the only
                        // thing its cooling buffer strips is its own `;_` markers.
                        current_feedrate = new_feedrate;
                        let clean = strip_cooling_markers(sline);
                        new_gcode.push_str(clean.trim_end());
                        new_gcode.push('\n');
                    }
                } else {
                    let clean = strip_cooling_markers(sline);
                    if !clean.is_empty() {
                        new_gcode.push_str(&clean);
                        new_gcode.push('\n');
                    }
                }
            } else if (line.line_type & CoolingLine::TYPE_OBJECT_START) != 0 {
                new_gcode.push_str(line_start_bytes);
                if pre_start_overhang_fan_time > 0.0 && self.m_current_fan_speed > self.m_fan_speed
                {
                    new_gcode.push_str(&format_set_fan(self.m_current_fan_speed));
                }
            } else if (line.line_type & CoolingLine::TYPE_OBJECT_END) != 0 {
                if pre_start_overhang_fan_time > 0.0 && self.m_current_fan_speed > self.m_fan_speed
                {
                    new_gcode.push_str(&format_set_fan(self.m_fan_speed));
                }
                new_gcode.push_str(line_start_bytes);
            } else {
                new_gcode.push_str(line_start_bytes);
            }

            pos = line.line_end;
        }

        // Append remaining text
        if pos < gcode.len() {
            new_gcode.push_str(&gcode[pos..]);
        }

        new_gcode
    }
}

/// Calculate layer slowdown for the post-processor adjustments.
/// Reference: CoolingBuffer::calculate_layer_slowdown() (CoolingBuffer.cpp:259-350)
/// Parsed-but-unwritten layer for the two-phase cooling path.
pub struct ParsedLayer {
    pub gcode: String,
    pub adjustments: Vec<PostProcAdjustments>,
    pub not_set_additional_fan: bool,
    pub layer_time: f32,
    pub layer_id: usize,
}

/// Smoothing.cpp:5-43 `SmoothCalculator::build_node` over the live
/// PostProcAdjustments types (smoothing.rs's own copy consumes the parallel
/// g_code_editor port; the data structs are shared).
pub fn build_node_postproc(
    wall_collection: &mut Vec<crate::gcode::smoothing::OutwallCollection>,
    object_label: &[i32],
    per_extruder_adjustments: &[PostProcAdjustments],
) {
    use crate::gcode::smoothing::{CoolingNode, OutwallCollection};
    if per_extruder_adjustments.is_empty() {
        return;
    }
    for object_idx in 0..object_label.len() {
        let mut object_level = OutwallCollection::new();
        object_level.object_id = object_label[object_idx];
        wall_collection.push(object_level);
    }
    for (extruder_idx, extruder_adjustments) in per_extruder_adjustments.iter().enumerate() {
        for (line_idx, line) in extruder_adjustments.lines.iter().enumerate() {
            if line.outwall_smooth_mark {
                let nodes = &mut wall_collection[line.object_id as usize].cooling_nodes;
                nodes
                    .entry(line.cooling_node_id)
                    .or_insert_with(CoolingNode::new);
                let node = nodes.get_mut(&line.cooling_node_id).unwrap();
                if (line.line_type & CoolingLine::TYPE_EXTERNAL_PERIMETER) != 0 {
                    node.outwall_line.push((line_idx as i32, extruder_idx as i32));
                    if node.max_feedrate < line.feedrate {
                        node.max_feedrate = line.feedrate;
                        node.filter_feedrate = node.max_feedrate;
                    }
                }
            }
        }
    }
}

/// Smoothing.cpp:45-62 `exclude_participate_in_speed_slowdown` over
/// PostProcAdjustments: clamp outwall lines to the smoothed filter_feedrate,
/// drop them from further cooling adjustment, recompute their times.
fn exclude_participate_postproc(
    lines_pos: &[(i32, i32)],
    per_extruder_adjustments: &mut [PostProcAdjustments],
    node: &crate::gcode::smoothing::CoolingNode,
) -> f64 {
    let apply_speed = node.max_feedrate > 0.0 && node.filter_feedrate > 0.0;
    let mut rate = node.rate;
    if apply_speed {
        rate = node.filter_feedrate as f64 / node.max_feedrate as f64;
    }
    for &(line_pos, extruder_pos) in lines_pos {
        let line = &mut per_extruder_adjustments[extruder_pos as usize].lines[line_pos as usize];
        if apply_speed && line.feedrate > node.filter_feedrate {
            line.feedrate = node.filter_feedrate;
            line.slowdown = true;
        }
        line.line_type &= !CoolingLine::TYPE_ADJUSTABLE;
        if line.feedrate == 0.0 || line.length == 0.0 {
            line.time = 0.0;
        } else {
            line.time = line.length / line.feedrate;
        }
    }
    rate
}

/// Smoothing.cpp:64-80 `SmoothCalculator::recaculate_layer_time` over
/// PostProcAdjustments. Mirrors the C++ `std::map::operator[]` loop that
/// self-densifies sparse node keys (bound re-read each iteration).
pub fn recalculate_layer_time_postproc(
    smoother: &mut crate::gcode::smoothing::SmoothCalculator,
    layer_id: usize,
    per_extruder_adjustments: &mut [PostProcAdjustments],
) -> f32 {
    use crate::gcode::smoothing::CoolingNode;
    for obj_id in 0..smoother.layers_wall_collection[layer_id].len() {
        let mut node_id: usize = 0;
        while node_id < smoother.layers_wall_collection[layer_id][obj_id].cooling_nodes.len() {
            let node = smoother.layers_wall_collection[layer_id][obj_id]
                .cooling_nodes
                .entry(node_id as i32)
                .or_insert_with(CoolingNode::new)
                .clone();
            let rate = exclude_participate_postproc(
                &node.outwall_line,
                per_extruder_adjustments,
                &node,
            );
            let stored = smoother.layers_wall_collection[layer_id][obj_id]
                .cooling_nodes
                .get_mut(&(node_id as i32))
                .unwrap();
            stored.rate = rate;
            node_id += 1;
        }
    }

    let mut layer_time = 0.0f32;
    for extruder in per_extruder_adjustments.iter() {
        for line in &extruder.lines {
            layer_time += line.time;
        }
    }
    layer_time
}

fn calculate_layer_slowdown_postproc(
    per_extruder_adjustments: &mut [PostProcAdjustments],
    cooling_logic_proportional: bool,
) -> f32 {
    // Sort extruders by slow_down_layer_time (increasing)
    // Reference: CoolingBuffer.cpp:259-350
    let mut by_slowdown_time: Vec<usize> = Vec::new();
    let mut elapsed_time_total0 = 0.0f32;

    for (idx, adj) in per_extruder_adjustments.iter_mut().enumerate() {
        adj.time_total = adj.elapsed_time_total();
        adj.time_maximum = adj.maximum_time_after_slowdown_bool(true);

        if adj.cooling_slow_down_enabled && !adj.lines.is_empty() {
            by_slowdown_time.push(idx);

            // For ConsistentSurface, prepare non-adjustable segments
            if adj.cooling_slowdown_logic == 1 {
                for line in &mut adj.lines {
                    if (line.line_type & CoolingLine::TYPE_ADJUSTABLE) != 0 {
                        line.adjustable_length = line.length;
                        line.adjustable_time = line.time;
                        line.adjustable_time_max = line.time_max;
                    }
                }
                let dist = adj.cooling_perimeter_transition_distance;
                adj.create_non_adjustable_segments(dist);
            }

            if !cooling_logic_proportional {
                adj.sort_lines_by_decreasing_feedrate();
            }
        } else {
            elapsed_time_total0 += adj.elapsed_time_total();
        }
    }

    // Sort by increasing slow_down_layer_time
    by_slowdown_time.sort_by(|&a, &b| {
        per_extruder_adjustments[a]
            .slow_down_layer_time
            .partial_cmp(&per_extruder_adjustments[b].slow_down_layer_time)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    for cur_begin in 0..by_slowdown_time.len() {
        let begin_idx = by_slowdown_time[cur_begin];
        let mut total = elapsed_time_total0;
        for &idx in &by_slowdown_time[cur_begin..] {
            total += per_extruder_adjustments[idx].time_total;
        }

        let slow_down_layer_time = per_extruder_adjustments[begin_idx].slow_down_layer_time * 1.001;

        if total > slow_down_layer_time {
            // No adjustment needed for this tier
        } else {
            let mut max_time = elapsed_time_total0;
            for &idx in &by_slowdown_time[cur_begin..] {
                max_time += per_extruder_adjustments[idx].time_maximum;
            }

            if max_time > slow_down_layer_time {
                let time_stretch = slow_down_layer_time - total;
                let first_logic = per_extruder_adjustments[begin_idx].cooling_slowdown_logic;

                if first_logic == 1 {
                    // ConsistentSurface: two-phase slowdown
                    // slow non-visible features
                    let remaining = consistent_surface_slowdown(
                        per_extruder_adjustments,
                        &by_slowdown_time[cur_begin..],
                        time_stretch,
                        0, // None: slow only non-visible features
                    );
                    // slow external + first internal perimeters
                    if remaining > 0.0 {
                        consistent_surface_slowdown(
                            per_extruder_adjustments,
                            &by_slowdown_time[cur_begin..],
                            remaining,
                            3, // ExternalPerimeters(1) | FirstInternalPerimeters(2)
                        );
                    }
                } else if cooling_logic_proportional {
                    // Proportional slowdown
                    proportional_slowdown(
                        per_extruder_adjustments,
                        &by_slowdown_time[cur_begin..],
                        elapsed_time_total0,
                        total,
                        slow_down_layer_time,
                    );
                } else {
                    // Non-proportional slowdown
                    non_proportional_slowdown(
                        per_extruder_adjustments,
                        &by_slowdown_time[cur_begin..],
                        time_stretch,
                    );
                }
            } else {
                // Slow everything to maximum
                for &idx in &by_slowdown_time[cur_begin..] {
                    per_extruder_adjustments[idx].slowdown_to_minimum_feedrate_bool(true);
                }
            }
        }

        elapsed_time_total0 += per_extruder_adjustments[begin_idx].elapsed_time_total();
    }

    // Sort all lines back by position for output
    for adj in per_extruder_adjustments.iter_mut() {
        adj.lines.sort_by_key(|l| l.line_start);
    }

    elapsed_time_total0
}

/// Proportional slowdown algorithm for post-processor.
/// Reference: CoolingBuffer.cpp:59-102
fn proportional_slowdown(
    adjustments: &mut [PostProcAdjustments],
    indices: &[usize],
    elapsed_time_total0: f32,
    elapsed_time_before_slowdown: f32,
    slow_down_layer_time: f32,
) -> f32 {
    let mut total_after = elapsed_time_before_slowdown;

    // Check if non-external perimeter slowdown is sufficient
    let mut max_time_nep = elapsed_time_total0;
    for &idx in indices {
        max_time_nep += adjustments[idx].maximum_time_after_slowdown_bool(false);
    }

    if max_time_nep > slow_down_layer_time {
        // Slow only non-external perimeters
        let mut non_adj_time = elapsed_time_total0;
        for &idx in indices {
            non_adj_time += adjustments[idx].non_adjustable_time_bool(false);
        }
        for _iter in 0..5 {
            let factor = (slow_down_layer_time - non_adj_time) / (total_after - non_adj_time);
            total_after = elapsed_time_total0;
            for &idx in indices {
                total_after += adjustments[idx].slow_down_proportional(factor, false);
            }
            if total_after > 0.95 * slow_down_layer_time {
                break;
            }
        }
    } else {
        // Slow everything: first max out non-external, then slow external proportionally
        for &idx in indices {
            adjustments[idx].slowdown_to_minimum_feedrate_bool(false);
        }
        let mut non_adj_time = elapsed_time_total0;
        for &idx in indices {
            non_adj_time += adjustments[idx].non_adjustable_time_bool(true);
        }
        for _iter in 0..5 {
            let factor = (slow_down_layer_time - non_adj_time) / (total_after - non_adj_time);
            total_after = elapsed_time_total0;
            for &idx in indices {
                total_after += adjustments[idx].slow_down_proportional(factor, true);
            }
            if total_after > 0.95 * slow_down_layer_time {
                break;
            }
        }
    }

    total_after
}

/// Non-proportional slowdown for post-processor (equalize feedrates).
/// Reference: CoolingBuffer.cpp:188-256
fn non_proportional_slowdown(
    adjustments: &mut [PostProcAdjustments],
    indices: &[usize],
    mut time_stretch: f32,
) {
    // Sort by slow_down_min_speed descending
    let mut by_min_speed: Vec<usize> = indices.to_vec();
    by_min_speed.sort_by(|&a, &b| {
        adjustments[b]
            .slow_down_min_speed
            .partial_cmp(&adjustments[a].slow_down_min_speed)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Find highest adjustable feedrate
    let mut feedrate = 0.0f32;
    for &idx in &by_min_speed {
        adjustments[idx].idx_line_begin = 0;
        adjustments[idx].idx_line_end = 0;
        if adjustments[idx].n_lines_adjustable > 0 {
            let f = adjustments[idx].lines[0].feedrate;
            if f > feedrate {
                feedrate = f;
            }
        }
    }

    let mut loop_guard = 0;
    loop {
        loop_guard += 1;
        if loop_guard > 1000 {
            break; // Safety: prevent infinite loop
        }
        // Find span of lines with feedrate near current
        for &idx in &by_min_speed {
            let adj = &mut adjustments[idx];
            adj.idx_line_end = adj.idx_line_begin;
            while adj.idx_line_end < adj.n_lines_adjustable
                && adj.lines[adj.idx_line_end].feedrate > feedrate - EPSILON
            {
                adj.idx_line_end += 1;
            }
        }

        // Find next highest feedrate
        let mut feedrate_next = 0.0f32;
        for &idx in &by_min_speed {
            let adj = &adjustments[idx];
            if adj.idx_line_end < adj.n_lines_adjustable {
                let f = adj.lines[adj.idx_line_end].feedrate;
                if f > feedrate_next {
                    feedrate_next = f;
                }
            }
        }

        // Process each speed tier
        let mut tier_idx = 0;
        while tier_idx < by_min_speed.len() {
            let current_adj_idx = by_min_speed[tier_idx];
            let min_speed = adjustments[current_adj_idx].slow_down_min_speed;

            if min_speed == 0.0 {
                let mut time_adjustable = 0.0f32;
                for i in tier_idx..by_min_speed.len() {
                    time_adjustable += adjustments[by_min_speed[i]].adjustable_time_bool(true);
                }
                let rate = (time_adjustable + time_stretch) / time_adjustable;
                for i in tier_idx..by_min_speed.len() {
                    adjustments[by_min_speed[i]].slow_down_proportional(rate, true);
                }
                return;
            } else {
                let feedrate_limit = feedrate_next.max(min_speed);
                let mut time_stretch_max = 0.0f32;
                for i in tier_idx..by_min_speed.len() {
                    time_stretch_max += adjustments[by_min_speed[i]]
                        .time_stretch_when_slowing_down_to_feedrate(feedrate_limit);
                }
                if time_stretch_max >= time_stretch {
                    // Binary search for exact feedrate
                    let mut f_low = feedrate_limit;
                    let mut f_high = feedrate;
                    for _ in 0..20 {
                        let f_mid = (f_low + f_high) / 2.0;
                        let mut s = 0.0f32;
                        for i in tier_idx..by_min_speed.len() {
                            s += adjustments[by_min_speed[i]]
                                .time_stretch_when_slowing_down_to_feedrate(f_mid);
                        }
                        if s < time_stretch {
                            f_high = f_mid;
                        } else {
                            f_low = f_mid;
                        }
                        if (s - time_stretch).abs() < 0.01 {
                            break;
                        }
                    }
                    for i in tier_idx..by_min_speed.len() {
                        adjustments[by_min_speed[i]].slow_down_to_feedrate(f_low);
                    }
                    return;
                } else {
                    time_stretch -= time_stretch_max;
                    for i in tier_idx..by_min_speed.len() {
                        adjustments[by_min_speed[i]].slow_down_to_feedrate(feedrate_limit);
                    }
                }
            }

            // Skip to next speed tier
            let current_min = adjustments[by_min_speed[tier_idx]].slow_down_min_speed;
            tier_idx += 1;
            while tier_idx < by_min_speed.len() {
                if adjustments[by_min_speed[tier_idx]].slow_down_min_speed < current_min - EPSILON {
                    break;
                }
                tier_idx += 1;
            }
        }

        if feedrate_next == 0.0 {
            break;
        }

        for &idx in &by_min_speed {
            adjustments[idx].idx_line_begin = adjustments[idx].idx_line_end;
        }
        feedrate = feedrate_next;
    }
}

/// ConsistentSurface two-phase slowdown for post-processor.
/// Reference: CoolingBuffer.cpp:108-184
fn consistent_surface_slowdown(
    adjustments: &mut [PostProcAdjustments],
    indices: &[usize],
    time_stretch: f32,
    additional_features: u32,
) -> f32 {
    if time_stretch <= 0.0 {
        return 0.0;
    }

    // Sort by slow_down_min_speed descending
    let mut by_min_speed: Vec<usize> = indices.to_vec();
    by_min_speed.sort_by(|&a, &b| {
        adjustments[b]
            .slow_down_min_speed
            .partial_cmp(&adjustments[a].slow_down_min_speed)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Find highest adjustable feedrate
    let mut feedrate = 0.0f32;
    for &idx in &by_min_speed {
        for i in 0..adjustments[idx].n_lines_adjustable {
            let line = &adjustments[idx].lines[i];
            if line.adjustable_for_features(additional_features) && line.feedrate > feedrate {
                feedrate = line.feedrate;
            }
        }
    }

    if feedrate == 0.0 {
        return time_stretch;
    }

    let mut remaining = time_stretch;
    let mut processed = 0;

    while processed < by_min_speed.len() {
        let feedrate_limit = adjustments[by_min_speed[processed]].slow_down_min_speed;

        let mut time_stretch_max = 0.0f32;
        for i in processed..by_min_speed.len() {
            time_stretch_max += adjustments[by_min_speed[i]]
                .time_stretch_when_slowing_down_to_feedrate_features(
                    feedrate_limit,
                    additional_features,
                );
        }

        if time_stretch_max >= remaining {
            // Binary search
            let mut f_high = feedrate;
            let mut f_low = feedrate_limit;
            for _iter in 0..20 {
                let f_mid = (f_high + f_low) / 2.0;
                let mut stretch = 0.0f32;
                for i in processed..by_min_speed.len() {
                    stretch += adjustments[by_min_speed[i]]
                        .time_stretch_when_slowing_down_to_feedrate_features(
                            f_mid,
                            additional_features,
                        );
                }
                if stretch < remaining {
                    f_high = f_mid;
                } else {
                    f_low = f_mid;
                }
                if (stretch - remaining).abs() < 0.01 {
                    break;
                }
            }
            for i in processed..by_min_speed.len() {
                adjustments[by_min_speed[i]]
                    .slow_down_to_feedrate_features(f_low, additional_features);
            }
            return 0.0;
        } else {
            remaining -= time_stretch_max;
            for i in processed..by_min_speed.len() {
                adjustments[by_min_speed[i]]
                    .slow_down_to_feedrate_features(feedrate_limit, additional_features);
            }
        }

        // Skip to next tier
        let current_min = adjustments[by_min_speed[processed]].slow_down_min_speed;
        processed += 1;
        while processed < by_min_speed.len() {
            if adjustments[by_min_speed[processed]].slow_down_min_speed < current_min - EPSILON {
                break;
            }
            processed += 1;
        }
    }

    remaining
}

/// Legacy wrapper: process a single layer's G-code (backwards compatible).
/// Creates a temporary GCodeEditorState and processes in one shot.
pub fn process_layer_gcode(
    gcode: &str,
    layer_id: usize,
    min_layer_time: f32,
    min_print_speed: f32,
    fan_min_speed: i32,
    fan_max_speed: i32,
    _slow_down_layer_time: f32,
    fan_cooling_layer_time: f32,
    close_fan_first_layers: usize,
) -> String {
    if gcode.is_empty() {
        return String::new();
    }

    let cfg = PerExtruderCoolingConfig {
        fan_min_speed,
        fan_max_speed,
        slow_down_for_layer_cooling: true,
        slow_down_layer_time: min_layer_time,
        slow_down_min_speed: min_print_speed,
        fan_cooling_layer_time,
        close_fan_the_first_x_layers: close_fan_first_layers as i32,
        ..PerExtruderCoolingConfig::default()
    };

    let mut state = GCodeEditorState::new();
    state.process_layer(
        gcode,
        layer_id,
        &[cfg],
        false, // cooling_logic_proportional
        false, // auxiliary_fan
        "T",   // toolchange_prefix
        true,  // use_relative_e
        &[],   // object_label
        true,  // flush
        false, // spiral_vase
    )
}

/// Strip cooling marker comments from a G-code line
fn strip_cooling_markers(line: &str) -> String {
    line.replace(";_EXTRUDE_SET_SPEED", "")
        .replace(";_EXTERNAL_PERIMETER", "")
        .replace(";_WIPE", "")
        .replace(";_INTERNAL_PERIMETER", "")
        .replace(";_FIRST_INTERNAL_PERIMETER", "")
        .trim_end()
        .to_string()
}

/// Strip cooling markers from a string fragment
fn strip_cooling_markers_str(s: &str) -> String {
    s.replace(";_EXTRUDE_SET_SPEED", "")
        .replace(";_EXTERNAL_PERIMETER", "")
        .replace(";_WIPE", "")
        .replace(";_INTERNAL_PERIMETER", "")
        .replace(";_FIRST_INTERNAL_PERIMETER", "")
        .trim()
        .to_string()
}

/// Remove the F parameter from a G-code line
fn remove_f_param(line: &str) -> String {
    if let Some(f_start) = line.find(" F") {
        let after = &line[f_start + 2..];
        let f_end = after
            .find(|c: char| c == ' ' || c == ';' || c == '\n')
            .unwrap_or(after.len());
        let mut result = String::with_capacity(line.len());
        result.push_str(line[..f_start].trim_end());
        if f_start + 2 + f_end < line.len() {
            result.push_str(&line[f_start + 2 + f_end..]);
        }
        result
    } else {
        line.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cooling_config_default() {
        let config = CoolingConfig::default();
        assert_eq!(config.min_layer_time, 5.0);
        assert_eq!(config.min_print_speed, 10.0);
        assert_eq!(config.fan_speed, 1.0);
    }

    #[test]
    fn test_cooling_config_builder() {
        let config = CoolingConfig::new()
            .with_min_layer_time(10.0)
            .with_min_print_speed(15.0)
            .with_fan_speed(0.8);

        assert_eq!(config.min_layer_time, 10.0);
        assert_eq!(config.min_print_speed, 15.0);
        assert_eq!(config.fan_speed, 0.8);
    }

    #[test]
    fn test_cooling_move_creation() {
        let travel = CoolingMove::travel(10.0, 100.0);
        assert!(travel.is_travel);
        assert!(!travel.can_slowdown);
        assert!((travel.time - 0.1).abs() < 0.001);

        let extrusion = CoolingMove::extrusion(20.0, 50.0, ExtrusionRole::Perimeter);
        assert!(!extrusion.is_travel);
        assert!(extrusion.can_slowdown);
        assert!((extrusion.time - 0.4).abs() < 0.001);
    }

    #[test]
    fn test_bridge_cannot_slowdown() {
        let bridge = CoolingMove::extrusion(10.0, 30.0, ExtrusionRole::BridgeInfill);
        assert!(!bridge.can_slowdown);
    }

    #[test]
    fn test_per_extruder_adjustments() {
        let mut adj = PerExtruderAdjustments::new(0);
        adj.add_move(CoolingMove::travel(10.0, 100.0));
        adj.add_move(CoolingMove::extrusion(20.0, 50.0, ExtrusionRole::Perimeter));

        assert!((adj.travel_time - 0.1).abs() < 0.001);
        assert!((adj.extrusion_time - 0.4).abs() < 0.001);
        assert!((adj.total_time() - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_slowdown_calculation_no_slowdown_needed() {
        let config = CoolingConfig {
            min_layer_time: 5.0,
            min_print_speed: 10.0,
            ..Default::default()
        };
        let buffer = CoolingBuffer::new(config);

        // Create a layer that takes 10 seconds (above minimum)
        let mut adj = PerExtruderAdjustments::new(0);
        adj.add_move(CoolingMove::extrusion(
            500.0,
            50.0,
            ExtrusionRole::Perimeter,
        )); // 10 seconds

        let mut adjustments = vec![adj];
        let factor = buffer.calculate_layer_slowdown(&mut adjustments);

        assert!((factor - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_slowdown_calculation_slowdown_needed() {
        let config = CoolingConfig {
            min_layer_time: 10.0,
            min_print_speed: 10.0,
            ..Default::default()
        };
        let buffer = CoolingBuffer::new(config);

        // Create a layer that takes 5 seconds (below minimum)
        let mut adj = PerExtruderAdjustments::new(0);
        adj.add_move(CoolingMove::extrusion(
            250.0,
            50.0,
            ExtrusionRole::Perimeter,
        )); // 5 seconds

        let mut adjustments = vec![adj];
        let factor = buffer.calculate_layer_slowdown(&mut adjustments);

        // Should slow down by factor of 2 to reach 10 seconds
        assert!(factor > 1.0);
        assert!((factor - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_fan_speed_first_layer_disabled() {
        let config = CoolingConfig {
            disable_fan_first_layers: 2,
            ..Default::default()
        };
        let buffer = CoolingBuffer::new(config);

        assert_eq!(buffer.calculate_fan_speed(0, 1.0), 0.0);
        assert_eq!(buffer.calculate_fan_speed(1, 1.0), 0.0);
        assert!(buffer.calculate_fan_speed(2, 1.0) > 0.0);
    }

    #[test]
    fn test_fan_speed_based_on_layer_time() {
        let config = CoolingConfig {
            disable_fan_first_layers: 0,
            fan_below_layer_time: 60.0,
            full_fan_speed_layer_time: 15.0,
            fan_speed: 1.0,
            ..Default::default()
        };
        let buffer = CoolingBuffer::new(config);

        // Above threshold - no fan
        assert_eq!(buffer.calculate_fan_speed(5, 100.0), 0.0);

        // Below full fan threshold - full fan
        assert_eq!(buffer.calculate_fan_speed(5, 10.0), 1.0);

        // In between - interpolated
        let speed = buffer.calculate_fan_speed(5, 37.5);
        assert!(speed > 0.0 && speed < 1.0);
    }

    #[test]
    fn test_process_layer() {
        let config = CoolingConfig {
            min_layer_time: 10.0,
            min_print_speed: 10.0,
            disable_fan_first_layers: 0,
            fan_below_layer_time: 60.0,
            ..Default::default()
        };
        let buffer = CoolingBuffer::new(config);

        let moves = vec![
            CoolingMove::extrusion(250.0, 50.0, ExtrusionRole::Perimeter), // 5 seconds
        ];

        let result = buffer.process_layer(5, moves, 0);

        assert!(result.has_slowdown());
        assert!(result.adjusted_time >= 10.0 - 0.1);
        assert!(result.fan_enabled());
    }

    #[test]
    fn test_bridge_fan_speed() {
        let config = CoolingConfig {
            bridge_fan_override: true,
            bridge_fan_speed: 0.8,
            ..Default::default()
        };
        let buffer = CoolingBuffer::new(config);

        assert_eq!(buffer.bridge_fan_speed(), Some(0.8));

        let config_disabled = CoolingConfig {
            bridge_fan_override: false,
            ..Default::default()
        };
        let buffer_disabled = CoolingBuffer::new(config_disabled);

        assert_eq!(buffer_disabled.bridge_fan_speed(), None);
    }

    #[test]
    fn test_cooling_result() {
        let result = CoolingResult {
            moves: vec![],
            original_time: 5.0,
            adjusted_time: 10.0,
            slowdown_factor: 2.0,
            fan_speed: 0.75,
        };

        assert!(result.has_slowdown());
        assert!(result.fan_enabled());
        assert_eq!(result.fan_speed_percent(), 75);
    }

    #[test]
    fn test_estimate_layer_time() {
        let path_lengths = vec![100.0, 200.0];
        let feedrates = vec![50.0, 100.0];
        let travel_length = 50.0;
        let travel_feedrate = 100.0;

        let time = estimate_layer_time(&path_lengths, &feedrates, travel_length, travel_feedrate);

        // 100/50 + 200/100 + 50/100 = 2 + 2 + 0.5 = 4.5 seconds
        assert!((time - 4.5).abs() < 0.001);
    }

    /// Test proportional slowdown algorithm
    /// Reference: CoolingBuffer.cpp:63-119
    #[test]
    fn test_proportional_slowdown() {
        let mut config = CoolingConfig::default();
        config.slowdown_proportional = true;
        config.min_layer_time = 10.0;
        config.min_print_speed = 10.0;
        let buffer = CoolingBuffer::new(config);

        // Create a layer that takes 5 seconds
        let mut adj = PerExtruderAdjustments::new(0);
        adj.cooling_slow_down_enabled = true;
        adj.slow_down_layer_time = 10.0;
        adj.slow_down_min_speed = 10.0;
        adj.add_move(CoolingMove::extrusion(
            250.0,
            50.0,
            ExtrusionRole::Perimeter,
        ));

        let mut adjustments = vec![adj];
        let final_time = buffer.calculate_layer_slowdown(&mut adjustments);

        // Should reach ~10 seconds
        assert!(final_time >= 9.5);
        assert!(final_time <= 10.5);
    }

    /// Test non-proportional slowdown algorithm (equalize feedrates)
    /// Reference: CoolingBuffer.cpp:210-287
    #[test]
    fn test_non_proportional_slowdown() {
        let mut config = CoolingConfig::default();
        config.slowdown_proportional = false;
        config.min_layer_time = 10.0;
        config.min_print_speed = 10.0;
        let buffer = CoolingBuffer::new(config);

        // Create moves with different speeds
        let mut adj = PerExtruderAdjustments::new(0);
        adj.cooling_slow_down_enabled = true;
        adj.slow_down_layer_time = 10.0;
        adj.slow_down_min_speed = 10.0;
        adj.add_move(CoolingMove::extrusion(
            100.0,
            100.0,
            ExtrusionRole::Perimeter,
        )); // 1s
        adj.add_move(CoolingMove::extrusion(
            200.0,
            50.0,
            ExtrusionRole::InternalInfill,
        )); // 4s

        let mut adjustments = vec![adj];
        let final_time = buffer.calculate_layer_slowdown(&mut adjustments);

        // Should reach target by slowing fast moves first
        assert!(final_time >= 9.5);
        assert!(final_time <= 10.5);
    }

    /// Test ConsistentSurface slowdown (two-phase)
    /// Reference: CoolingBuffer.cpp:122-207
    #[test]
    fn test_consistent_surface_slowdown() {
        let mut config = CoolingConfig::default();
        config.slowdown_logic = CoolingSlowdownLogic::ConsistentSurface;
        config.min_layer_time = 15.0;
        config.min_print_speed = 10.0;
        config.perimeter_transition_distance = 5.0;
        let buffer = CoolingBuffer::new(config);

        // Create external and internal moves
        let mut adj = PerExtruderAdjustments::new(0);
        adj.cooling_slow_down_enabled = true;
        adj.slow_down_layer_time = 15.0;
        adj.slow_down_min_speed = 10.0;
        adj.cooling_slowdown_logic = CoolingSlowdownLogic::ConsistentSurface;

        let mut external = CoolingMove::extrusion(100.0, 50.0, ExtrusionRole::ExternalPerimeter);
        external.is_external_perimeter = true;
        adj.add_move(external);

        adj.add_move(CoolingMove::extrusion(
            200.0,
            50.0,
            ExtrusionRole::Perimeter,
        )); // Internal

        let mut adjustments = vec![adj];
        let final_time = buffer.calculate_layer_slowdown(&mut adjustments);

        // Should prioritize slowing internal moves first
        assert!(final_time >= 14.5);
        assert!(final_time <= 15.5);
    }

    /// Test binary search for time stretch target
    /// Reference: CoolingBuffer.cpp:1-60
    #[test]
    fn test_binary_search_feedrate() {
        let mut adj1 = PerExtruderAdjustments::new(0);
        adj1.slow_down_min_speed = 10.0;
        adj1.n_lines_adjustable = 2;
        adj1.add_move(CoolingMove::extrusion(
            100.0,
            50.0,
            ExtrusionRole::Perimeter,
        ));
        adj1.add_move(CoolingMove::extrusion(
            100.0,
            40.0,
            ExtrusionRole::Perimeter,
        ));

        let adjustments = vec![&adj1];
        let feedrate = CoolingBuffer::new_feedrate_to_reach_time_stretch(
            &adjustments,
            15.0,
            2.0, // Need 2 more seconds
            20,
        );

        // Should find a feedrate between min and current speeds
        assert!(feedrate >= 15.0);
        assert!(feedrate <= 50.0);
    }

    /// Test multi-extruder coordination
    /// Reference: CoolingBuffer.cpp:290-420
    #[test]
    fn test_multi_extruder_slowdown() {
        let mut config = CoolingConfig::default();
        config.min_layer_time = 10.0;
        config.min_print_speed = 10.0;
        let buffer = CoolingBuffer::new(config);

        // Extruder 0: 3 seconds
        let mut adj0 = PerExtruderAdjustments::new(0);
        adj0.cooling_slow_down_enabled = true;
        adj0.slow_down_layer_time = 10.0;
        adj0.slow_down_min_speed = 10.0;
        adj0.add_move(CoolingMove::extrusion(
            150.0,
            50.0,
            ExtrusionRole::Perimeter,
        ));

        // Extruder 1: 2 seconds
        let mut adj1 = PerExtruderAdjustments::new(1);
        adj1.cooling_slow_down_enabled = true;
        adj1.slow_down_layer_time = 10.0;
        adj1.slow_down_min_speed = 10.0;
        adj1.add_move(CoolingMove::extrusion(
            100.0,
            50.0,
            ExtrusionRole::Perimeter,
        ));

        let mut adjustments = vec![adj0, adj1];
        let final_time = buffer.calculate_layer_slowdown(&mut adjustments);

        // Total should reach 10 seconds (both extruders combined)
        assert!(final_time >= 9.5);
        assert!(final_time <= 10.5);
    }

    /// Test non-adjustable segment creation
    /// Reference: GCodeEditor.hpp:324-361
    #[test]
    fn test_non_adjustable_segments() {
        let mut adj = PerExtruderAdjustments::new(0);
        adj.slow_down_min_speed = 10.0;

        // Add moves that form a perimeter
        for _ in 0..5 {
            adj.add_move(CoolingMove::extrusion(20.0, 50.0, ExtrusionRole::Perimeter));
        }

        // Create 10mm non-adjustable zone at perimeter end
        adj.create_non_adjustable_segments(10.0);

        // Check that last moves have non-adjustable portions
        let total_non_adjustable: f64 = adj.moves.iter().map(|m| m.non_adjustable_length).sum();

        assert!(total_non_adjustable >= 9.5);
        assert!(total_non_adjustable <= 10.5);
    }

    /// Test adjustable feature type filtering
    /// Reference: GCodeEditor.hpp:78-90
    #[test]
    fn test_adjustable_feature_types() {
        let mut external = CoolingMove::extrusion(100.0, 50.0, ExtrusionRole::ExternalPerimeter);
        external.is_external_perimeter = true;
        external.adjustable_time = 2.0;
        external.adjustable_time_max = 10.0;

        // External should not be adjustable without flag
        assert!(!external.adjustable_for_features(AdjustableFeatureType::NONE));

        // External should be adjustable with flag
        assert!(external.adjustable_for_features(AdjustableFeatureType::EXTERNAL_PERIMETERS));

        let mut internal = CoolingMove::extrusion(100.0, 50.0, ExtrusionRole::Perimeter);
        internal.adjustable_time = 2.0;
        internal.adjustable_time_max = 10.0;

        // Internal should always be adjustable
        assert!(internal.adjustable_for_features(AdjustableFeatureType::NONE));
        assert!(internal.adjustable_for_features(AdjustableFeatureType::EXTERNAL_PERIMETERS));
    }

    /// Test sorting by decreasing feedrate
    /// Reference: GCodeEditor.hpp:216-225
    #[test]
    fn test_sort_by_feedrate() {
        let mut adj = PerExtruderAdjustments::new(0);
        adj.add_move(CoolingMove::extrusion(
            100.0,
            30.0,
            ExtrusionRole::Perimeter,
        ));
        adj.add_move(CoolingMove::extrusion(
            100.0,
            50.0,
            ExtrusionRole::Perimeter,
        ));
        adj.add_move(CoolingMove::extrusion(
            100.0,
            40.0,
            ExtrusionRole::Perimeter,
        ));
        adj.add_move(CoolingMove::travel(50.0, 100.0)); // Non-adjustable

        adj.sort_lines_by_decreasing_feedrate();

        // Adjustable moves should be first, sorted by feedrate
        assert_eq!(adj.n_lines_adjustable, 3);
        assert!(adj.moves[0].feedrate >= adj.moves[1].feedrate);
        assert!(adj.moves[1].feedrate >= adj.moves[2].feedrate);
        assert!(adj.moves[3].is_travel); // Travel at end
    }

    /// Test time stretch calculation
    /// Reference: GCodeEditor.hpp:230-239
    #[test]
    fn test_time_stretch_calculation() {
        let mut adj = PerExtruderAdjustments::new(0);
        adj.slow_down_min_speed = 10.0;
        adj.n_lines_adjustable = 2;
        adj.add_move(CoolingMove::extrusion(
            100.0,
            50.0,
            ExtrusionRole::Perimeter,
        )); // 2s
        adj.add_move(CoolingMove::extrusion(
            100.0,
            40.0,
            ExtrusionRole::Perimeter,
        )); // 2.5s

        // Calculate stretch when slowing to 20 mm/s
        let stretch = adj.time_stretch_when_slowing_down_to_feedrate(20.0);

        // 50->20 gives 2*(50/20 - 1) = 3s extra
        // 40->20 gives 2.5*(40/20 - 1) = 2.5s extra
        // Total: 5.5s extra
        assert!((stretch - 5.5).abs() < 0.1);
    }

    /// Test maximum time calculation
    /// Reference: GCodeEditor.hpp:148-160
    #[test]
    fn test_maximum_time_after_slowdown() {
        let mut adj = PerExtruderAdjustments::new(0);
        adj.slow_down_min_speed = 10.0;

        let mut mov1 = CoolingMove::extrusion(100.0, 50.0, ExtrusionRole::Perimeter);
        mov1.time_max = 10.0; // 100mm at 10mm/s = 10s
        adj.add_move(mov1);

        let mut mov2 = CoolingMove::extrusion(50.0, 25.0, ExtrusionRole::Perimeter);
        mov2.time_max = 5.0;
        adj.add_move(mov2);

        let max_time = adj.maximum_time_after_slowdown(true);

        // Should be 10 + 5 = 15 seconds
        assert!((max_time - 15.0).abs() < 0.1);
    }
}
