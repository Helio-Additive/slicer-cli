//! GCode editor module for cooling-related G-code manipulation.
//!
//! C++ Reference:
//! - GCode/GCodeEditor.hpp
//! - GCode/GCodeEditor.cpp
//!
//! This module provides types for analyzing and adjusting G-code lines
//! during the cooling pass, including feedrate slowdown and per-extruder adjustments.

use std::collections::HashMap;

/// Feature types that can be adjusted during cooling slowdown.
/// Used by ConsistentSurface logic to control which features are slowed first.
/// Corresponds to C++ AdjustableFeatureType.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdjustableFeatureType(u32);

impl AdjustableFeatureType {
    pub const NONE: Self = Self(0);
    pub const EXTERNAL_PERIMETERS: Self = Self(1 << 0);
    pub const FIRST_INTERNAL_PERIMETERS: Self = Self(1 << 1);

    pub fn contains(&self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    pub fn is_none(&self) -> bool {
        self.0 == 0
    }
}

impl std::ops::BitOr for AdjustableFeatureType {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl std::ops::BitAnd for AdjustableFeatureType {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self {
        Self(self.0 & rhs.0)
    }
}

impl Default for AdjustableFeatureType {
    fn default() -> Self {
        Self::NONE
    }
}

/// Cooling slowdown logic type for an extruder.
/// Corresponds to C++ `CoolingSlowdownLogicType` (PrintConfig.hpp:262-265).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoolingSlowdownLogicType {
    /// Default: slow down all features equally.
    CslUniformCooling = 0,
    /// Prioritize slowing infill/internal perimeters first.
    CslConsistentSurface = 1,
}

impl Default for CoolingSlowdownLogicType {
    fn default() -> Self {
        Self::CslUniformCooling
    }
}

/// Type flags for cooling line classification.
/// Corresponds to C++ CoolingLine::Type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoolingLineType(u32);

#[allow(dead_code)]
impl CoolingLineType {
    pub const SET_TOOL: u32 = 1 << 0;
    pub const EXTRUDE_END: u32 = 1 << 1;
    pub const OVERHANG_FAN_START: u32 = 1 << 2;
    pub const OVERHANG_FAN_END: u32 = 1 << 3;
    pub const G0: u32 = 1 << 4;
    pub const G1: u32 = 1 << 5;
    pub const ADJUSTABLE: u32 = 1 << 6;
    pub const EXTERNAL_PERIMETER: u32 = 1 << 7;
    pub const HAS_F: u32 = 1 << 8;
    pub const WIPE: u32 = 1 << 9;
    pub const G4: u32 = 1 << 10;
    pub const G92: u32 = 1 << 11;
    pub const G2: u32 = 1 << 12;
    pub const G3: u32 = 1 << 13;
    pub const FORCE_RESUME_FAN: u32 = 1 << 14;
    pub const SET_FAN_CHANGING_LAYER: u32 = 1 << 15;
    pub const OBJECT_START: u32 = 1 << 16;
    pub const OBJECT_END: u32 = 1 << 17;
    pub const SET_FAN_CHANGING_FILAMENT: u32 = 1 << 18;
    pub const NOT_SET_FAN_CHANGING_FILAMENT: u32 = 1 << 19;
    pub const INTERNAL_PERIMETER: u32 = 1 << 20;
    pub const FIRST_INTERNAL_PERIMETER: u32 = 1 << 21;
}

/// A single line of G-code annotated with cooling information.
/// Corresponds to C++ CoolingLine.
#[derive(Debug, Clone)]
pub struct CoolingLine {
    pub line_type: u32,
    pub line_start: usize,
    pub line_end: usize,
    pub length: f32,
    pub feedrate: f32,
    pub origin_feedrate: f32,
    pub time: f32,
    pub time_max: f32,
    pub slowdown: bool,
    pub origin_time_max: f32,
    pub outwall_smooth_mark: bool,
    pub object_id: i32,
    pub cooling_node_id: i32,
    // ConsistentSurface fields
    pub adjustable_length: f32,
    pub non_adjustable_length: f32,
    pub adjustable_time: f32,
    pub non_adjustable_time: f32,
    pub adjustable_time_max: f32,
    pub perimeter_index: Option<u16>,
}

impl CoolingLine {
    pub fn new(line_type: u32, line_start: usize, line_end: usize) -> Self {
        Self {
            line_type,
            line_start,
            line_end,
            length: 0.0,
            feedrate: 0.0,
            origin_feedrate: 0.0,
            time: 0.0,
            time_max: 0.0,
            slowdown: false,
            origin_time_max: 0.0,
            outwall_smooth_mark: false,
            object_id: -1,
            cooling_node_id: -1,
            adjustable_length: 0.0,
            non_adjustable_length: 0.0,
            adjustable_time: 0.0,
            non_adjustable_time: 0.0,
            adjustable_time_max: 0.0,
            perimeter_index: None,
        }
    }

    /// Check if this line is adjustable (legacy method).
    pub fn adjustable_legacy(&self, slowdown_external_perimeters: bool) -> bool {
        (self.line_type & CoolingLineType::ADJUSTABLE) != 0
            && ((self.line_type & CoolingLineType::EXTERNAL_PERIMETER) == 0
                || slowdown_external_perimeters)
            && self.time < self.time_max
    }

    /// Check if this line is adjustable (simple check).
    pub fn adjustable(&self) -> bool {
        (self.line_type & CoolingLineType::ADJUSTABLE) != 0 && self.time < self.time_max
    }

    /// Check if adjustable with ConsistentSurface feature type control.
    pub fn adjustable_with_features(&self, features: AdjustableFeatureType) -> bool {
        if (self.line_type & CoolingLineType::ADJUSTABLE) == 0
            || self.adjustable_time >= self.adjustable_time_max
        {
            return false;
        }

        if (self.line_type & CoolingLineType::EXTERNAL_PERIMETER) != 0 {
            return features.contains(AdjustableFeatureType::EXTERNAL_PERIMETERS);
        }

        if (self.line_type & CoolingLineType::FIRST_INTERNAL_PERIMETER) != 0 {
            return features.contains(AdjustableFeatureType::FIRST_INTERNAL_PERIMETERS);
        }

        true
    }

    /// Total time including adjustable and non-adjustable parts.
    pub fn total_time(&self) -> f32 {
        self.adjustable_time + self.non_adjustable_time
    }

    /// Total length including adjustable and non-adjustable parts.
    pub fn total_length(&self) -> f32 {
        self.adjustable_length + self.non_adjustable_length
    }

    /// Maximum total time after slowdown.
    pub fn total_time_max(&self) -> f32 {
        self.adjustable_time_max + self.non_adjustable_time
    }
}

/// Per-extruder cooling adjustments.
/// Corresponds to C++ PerExtruderAdjustments.
#[derive(Debug, Clone)]
pub struct PerExtruderAdjustments {
    // Extruder, for which the G-code will be adjusted. // GCodeEditor.hpp:389
    pub extruder_id: u32,
    // Is the cooling slow down logic enabled for this extruder's material? // GCodeEditor.hpp:391
    pub cooling_slow_down_enabled: bool,
    // Slow down the print down to slow_down_min_speed if the total layer time is below slow_down_layer_time. // GCodeEditor.hpp:393
    pub slow_down_layer_time: f32,
    // Minimum print speed allowed for this extruder. // GCodeEditor.hpp:395
    pub slow_down_min_speed: f32,
    // Cooling slowdown logic type for this extruder (Uniform or ConsistentSurface) // GCodeEditor.hpp:398
    pub cooling_slowdown_logic: CoolingSlowdownLogicType,
    // Distance before perimeters where speed transitions back to normal // GCodeEditor.hpp:400
    pub cooling_perimeter_transition_distance: f32,
    // Parsed lines. // GCodeEditor.hpp:403
    pub lines: Vec<CoolingLine>,
    // Number of adjustable lines, at the start of lines. // GCodeEditor.hpp:406
    pub n_lines_adjustable: usize,
    // Non-adjustable time of lines starting with n_lines_adjustable. // GCodeEditor.hpp:408
    pub time_non_adjustable: f32,
    // Current total time for this extruder. // GCodeEditor.hpp:410
    pub time_total: f32,
    // Maximum time for this extruder, when the maximum slow down is applied. // GCodeEditor.hpp:412
    pub time_maximum: f32,
    // Temporaries for processing the slow down. Both thresholds go from 0 to n_lines_adjustable. // GCodeEditor.hpp:415
    pub idx_line_begin: usize,
    pub idx_line_end: usize,
}

impl PerExtruderAdjustments {
    pub fn new() -> Self {
        Self {
            extruder_id: 0,
            cooling_slow_down_enabled: false,
            slow_down_layer_time: 0.0,
            slow_down_min_speed: 0.0,
            cooling_slowdown_logic: CoolingSlowdownLogicType::CslUniformCooling,
            cooling_perimeter_transition_distance: 5.0,
            lines: Vec::new(),
            n_lines_adjustable: 0,
            time_non_adjustable: 0.0,
            time_total: 0.0,
            time_maximum: 0.0,
            idx_line_begin: 0,
            idx_line_end: 0,
        }
    }

    /// Total elapsed time for this extruder.
    pub fn elapsed_time_total(&self) -> f32 {
        self.lines.iter().map(|l| l.time).sum()
    }

    /// Maximum time after slowing all adjustable lines to minimum feedrate.
    pub fn maximum_time_after_slowdown(&self, slowdown_external_perimeters: bool) -> f32 {
        let mut time_total = 0.0f32;
        for line in &self.lines {
            if line.adjustable_legacy(slowdown_external_perimeters) {
                if line.time_max == f32::MAX {
                    return f32::MAX;
                }
                time_total += line.time_max;
            } else {
                time_total += line.time;
            }
        }
        time_total
    }

    /// Calculate the adjustable portion of total time.
    pub fn adjustable_time(&self, slowdown_external_perimeters: bool) -> f32 {
        self.lines
            .iter()
            .filter(|l| l.adjustable_legacy(slowdown_external_perimeters))
            .map(|l| l.time)
            .sum()
    }

    /// Calculate the non-adjustable portion of total time.
    pub fn non_adjustable_time(&self, slowdown_external_perimeters: bool) -> f32 {
        self.lines
            .iter()
            .filter(|l| !l.adjustable_legacy(slowdown_external_perimeters))
            .map(|l| l.time)
            .sum()
    }

    /// Slow down all adjustable lines to their minimum feedrate.
    pub fn slowdown_to_minimum_feedrate(&mut self, slowdown_external_perimeters: bool) -> f32 {
        let mut time_total = 0.0f32;
        for line in &mut self.lines {
            if line.adjustable_legacy(slowdown_external_perimeters) {
                debug_assert!(line.time_max >= 0.0 && line.time_max < f32::MAX);
                line.slowdown = true;
                line.time = line.time_max;
                if line.time > 0.0 {
                    line.feedrate = line.length / line.time;
                }
            }
            time_total += line.time;
        }
        time_total
    }

    /// Slow down adjustable lines proportionally by a factor.
    pub fn slow_down_proportional(
        &mut self,
        factor: f32,
        slowdown_external_perimeters: bool,
    ) -> f32 {
        debug_assert!(factor >= 1.0);
        let mut time_total = 0.0f32;
        for line in &mut self.lines {
            if line.adjustable_legacy(slowdown_external_perimeters) {
                line.slowdown = true;
                line.time = line.time_max.min(line.time * factor);
                if line.time > 0.0 {
                    line.feedrate = line.length / line.time;
                }
            }
            time_total += line.time;
        }
        time_total
    }

    /// Sort lines: adjustable first, higher feedrate first.
    pub fn sort_lines_by_decreasing_feedrate(&mut self) {
        self.lines.sort_by(|l1, l2| {
            let adj1 = l1.adjustable();
            let adj2 = l2.adjustable();
            if adj1 == adj2 {
                l2.feedrate
                    .partial_cmp(&l1.feedrate)
                    .unwrap_or(std::cmp::Ordering::Equal)
            } else if adj1 {
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

    /// Calculate the time stretch when slowing down to a given min feedrate.
    pub fn time_stretch_when_slowing_down_to_feedrate(&self, min_feedrate: f32) -> f32 {
        let mut time_stretch = 0.0f32;
        for i in 0..self.n_lines_adjustable {
            let line = &self.lines[i];
            if line.feedrate > min_feedrate {
                time_stretch += line.time * (line.feedrate / min_feedrate - 1.0);
            }
        }
        time_stretch
    }

    /// Slow down all adjustable lines to the given min feedrate.
    pub fn slow_down_to_feedrate(&mut self, min_feedrate: f32) {
        for i in 0..self.n_lines_adjustable {
            let line = &mut self.lines[i];
            if line.feedrate > min_feedrate {
                line.time *= (line.feedrate / min_feedrate).max(1.0);
                line.feedrate = min_feedrate;
                line.slowdown = true;
            }
        }
    }

    /// Collect total time from all lines.
    pub fn collection_line_times_of_extruder(&self) -> f32 {
        self.lines.iter().map(|l| l.time).sum()
    }

    // --- ConsistentSurface cooling methods --- // GCodeEditor.hpp:267

    // Calculate the maximum time stretch when slowing down to min_feedrate,
    // considering only features allowed by additional_slowdown_features. // GCodeEditor.hpp:269-280
    pub fn time_stretch_when_slowing_down_to_feedrate_features(
        &self,
        min_feedrate: f32,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        let mut time_stretch = 0.0f32;
        for i in 0..self.n_lines_adjustable {
            let line = &self.lines[i];
            if line.adjustable_with_features(additional_slowdown_features)
                && line.feedrate > min_feedrate
            {
                time_stretch += line.adjustable_time * (line.feedrate / min_feedrate - 1.0);
            }
        }
        time_stretch
    }

    // Slow down all lines matching the feature type to min_feedrate. // GCodeEditor.hpp:282-294
    pub fn slow_down_to_feedrate_features(
        &mut self,
        min_feedrate: f32,
        additional_slowdown_features: AdjustableFeatureType,
    ) {
        for i in 0..self.n_lines_adjustable {
            let line = &mut self.lines[i];
            if line.adjustable_with_features(additional_slowdown_features)
                && line.feedrate > min_feedrate
            {
                line.adjustable_time = line.adjustable_length / min_feedrate;
                line.time = line.adjustable_time + line.non_adjustable_time;
                line.feedrate = min_feedrate;
                line.slowdown = true;
            }
        }
    }

    // Calculate maximum time after slowdown for features matching the type. // GCodeEditor.hpp:296-310
    pub fn maximum_time_after_slowdown_features(
        &self,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        let mut time_total = 0.0f32;
        for line in &self.lines {
            if line.adjustable_with_features(additional_slowdown_features) {
                if line.adjustable_time_max == f32::MAX {
                    return f32::MAX;
                }
                time_total += line.adjustable_time_max + line.non_adjustable_time;
            } else {
                time_total += line.time;
            }
        }
        time_total
    }

    // Calculate adjustable time for features matching the type. // GCodeEditor.hpp:312-321
    pub fn adjustable_time_for_features(
        &self,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        let mut time_total = 0.0f32;
        for line in &self.lines {
            if line.adjustable_with_features(additional_slowdown_features) {
                time_total += line.adjustable_time;
            }
        }
        time_total
    }

    // Slow down to minimum feedrate for features matching the type. // GCodeEditor.hpp:323-338
    pub fn slowdown_to_minimum_feedrate_features(
        &mut self,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        let mut time_total = 0.0f32;
        for line in &mut self.lines {
            if line.adjustable_with_features(additional_slowdown_features) {
                line.slowdown = true;
                line.adjustable_time = line.adjustable_time_max;
                line.time = line.adjustable_time + line.non_adjustable_time;
                if line.adjustable_length > 0.0 {
                    line.feedrate = line.adjustable_length / line.adjustable_time;
                }
            }
            time_total += line.time;
        }
        time_total
    }

    // Create non-adjustable segments at the end of perimeter loops for transition smoothing.
    // This preserves speed in the last 'non_adjustable_length' mm of each perimeter. // GCodeEditor.hpp:340-384
    pub fn create_non_adjustable_segments(&mut self, non_adjustable_length: f32) {
        if non_adjustable_length <= 0.0 {
            return;
        }

        // Process lines in reverse to accumulate length from the end of each perimeter loop
        let mut accumulated_length = 0.0f32;
        for line in self.lines.iter_mut().rev() {
            // Reset accumulator at perimeter boundaries (non-adjustable lines or different feature types)
            if (line.line_type & CoolingLineType::ADJUSTABLE) == 0
                || (line.line_type & CoolingLineType::EXTRUDE_END) != 0
            {
                accumulated_length = 0.0;
                continue;
            }

            // Initialize adjustable fields if not set
            if line.adjustable_length == 0.0 && line.length > 0.0 {
                line.adjustable_length = line.length;
                line.adjustable_time = line.time;
                line.adjustable_time_max = line.time_max;
            }

            let remaining_non_adjustable = non_adjustable_length - accumulated_length;
            if remaining_non_adjustable > 0.0 && line.adjustable_length > 0.0 {
                let convert_length = line.adjustable_length.min(remaining_non_adjustable);
                let convert_ratio = convert_length / line.adjustable_length;

                line.non_adjustable_length += convert_length;
                line.non_adjustable_time += line.adjustable_time * convert_ratio;
                line.adjustable_length -= convert_length;
                line.adjustable_time -= line.adjustable_time * convert_ratio;
                line.adjustable_time_max =
                    if line.adjustable_length > 0.0 && self.slow_down_min_speed > 0.0 {
                        line.adjustable_length / self.slow_down_min_speed
                    } else {
                        0.0
                    };

                accumulated_length += convert_length;
            } else {
                accumulated_length += line.length;
            }
        }
    }

    // --- End ConsistentSurface cooling methods --- // GCodeEditor.hpp:386
}

/// G-code editor state for managing layer editing.
/// Corresponds to parts of C++ GCode class related to editing.
#[derive(Debug, Clone, Default)]
pub struct GCodeEditor {
    pub current_layer_index: usize,
    pub current_object_id: i32,
}

impl GCodeEditor {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.current_layer_index = 0;
        self.current_object_id = -1;
    }
}

/// Layer data for the G-code editor.
#[derive(Debug, Clone, Default)]
pub struct EditorLayer {
    pub index: usize,
    pub z: f64,
    pub height: f64,
}

impl EditorLayer {
    pub fn new() -> Self {
        Self::default()
    }
}

/// G-code container for the editor.
#[derive(Debug, Clone, Default)]
pub struct EditorGCode {
    pub lines: Vec<String>,
}

impl EditorGCode {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Sort cooling lines by decreasing feedrate (standalone function).
pub fn sort_lines_by_decreasing_feedrate(adjustments: &mut PerExtruderAdjustments) {
    adjustments.sort_lines_by_decreasing_feedrate();
}

/// Reset the G-code editor state.
pub fn reset(editor: &mut GCodeEditor) {
    editor.reset();
}

/// Write layer G-code (placeholder for actual layer writing).
pub fn write_layer_gcode() -> crate::Result<()> {
    // Actual layer writing is done by the exporter module
    Ok(())
}

/// Slow down adjustable lines to minimum feedrate.
pub fn slowdown_to_minimum_feedrate(
    adjustments: &mut PerExtruderAdjustments,
    slowdown_external_perimeters: bool,
) -> f32 {
    adjustments.slowdown_to_minimum_feedrate(slowdown_external_perimeters)
}

/// Slow down adjustable lines proportionally.
pub fn slow_down_proportional(
    adjustments: &mut PerExtruderAdjustments,
    factor: f32,
    slowdown_external_perimeters: bool,
) -> f32 {
    adjustments.slow_down_proportional(factor, slowdown_external_perimeters)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cooling_line_new() {
        let line = CoolingLine::new(CoolingLineType::G1 | CoolingLineType::ADJUSTABLE, 0, 10);
        assert!(!line.slowdown);
        assert_eq!(line.object_id, -1);
    }

    #[test]
    fn test_cooling_line_adjustable() {
        let mut line = CoolingLine::new(CoolingLineType::G1 | CoolingLineType::ADJUSTABLE, 0, 10);
        line.time = 1.0;
        line.time_max = 2.0;
        assert!(line.adjustable());

        line.time = 2.0;
        assert!(!line.adjustable());
    }

    #[test]
    fn test_per_extruder_adjustments() {
        let mut adj = PerExtruderAdjustments::new();
        let mut line = CoolingLine::new(CoolingLineType::G1 | CoolingLineType::ADJUSTABLE, 0, 10);
        line.time = 1.0;
        line.time_max = 3.0;
        line.length = 10.0;
        line.feedrate = 10.0;
        adj.lines.push(line);

        assert!((adj.elapsed_time_total() - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_adjustable_feature_type() {
        let feat = AdjustableFeatureType::EXTERNAL_PERIMETERS
            | AdjustableFeatureType::FIRST_INTERNAL_PERIMETERS;
        assert!(feat.contains(AdjustableFeatureType::EXTERNAL_PERIMETERS));
        assert!(!AdjustableFeatureType::NONE.contains(AdjustableFeatureType::EXTERNAL_PERIMETERS));
    }

    #[test]
    fn test_editor_reset() {
        let mut editor = GCodeEditor::new();
        editor.current_layer_index = 5;
        editor.reset();
        assert_eq!(editor.current_layer_index, 0);
    }
}
