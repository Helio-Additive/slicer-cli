//! Faithful 1:1 port of BambuStudio `GCode/GCodeEditor.{hpp,cpp}`.
//!
//! A standalone G-code filter, to control cooling of the print.
//! The G-code is processed per layer. Once a layer is collected, fan start / stop commands are edited
//! and the print is modified to stretch over a minimum layer time.
//!
//! Status: the header value-types (`AdjustableFeatureType`, `CoolingSlowdownLogicType`,
//! `CoolingLine`, `PerExtruderAdjustments`) are ported line-by-line below. The `GCodeEditor`
//! class methods (`reset`, `process_layer`, `parse_layer_gcode`, `write_layer_gcode`) require the
//! not-yet-ported BambuStudio `GCode` class and the per-extruder `FullPrintConfig`
//! (`OPT.get_at(extruder_id)` accessors). The functional equivalent of those methods is already
//! implemented in `crate::gcode::cooling::GCodeEditorState` against a decoupled config
//! representation. See module notes / PORT_LEDGER for the blocked symbols.
//!
//! C++ Reference:
//! - GCode/GCodeEditor.hpp
//! - GCode/GCodeEditor.cpp

// Feature types that can be adjusted during cooling slowdown
// Used by ConsistentSurface logic to control which features are slowed first
// GCodeEditor.hpp:20  enum class AdjustableFeatureType : uint32_t
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdjustableFeatureType(u32);

impl AdjustableFeatureType {
    // GCodeEditor.hpp:21  None                    = 0,
    pub const NONE: Self = Self(0);
    // GCodeEditor.hpp:22  ExternalPerimeters      = 1 << 0,
    pub const EXTERNAL_PERIMETERS: Self = Self(1 << 0);
    // GCodeEditor.hpp:23  FirstInternalPerimeters = 1 << 1,
    pub const FIRST_INTERNAL_PERIMETERS: Self = Self(1 << 1);

    // Helper mirroring `(a & b) != AdjustableFeatureType::None` comparison.
    pub fn contains(&self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    // Helper mirroring `!a` (GCodeEditor.hpp:34-36).
    pub fn is_none(&self) -> bool {
        self.0 == 0
    }
}

// GCodeEditor.hpp:26  inline AdjustableFeatureType operator|(AdjustableFeatureType a, AdjustableFeatureType b)
impl std::ops::BitOr for AdjustableFeatureType {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

// GCodeEditor.hpp:30  inline AdjustableFeatureType operator&(AdjustableFeatureType a, AdjustableFeatureType b)
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
/// Corresponds to C++ `CoolingSlowdownLogicType` (PrintConfig.hpp), default `cslUniformCooling`.
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
/// Corresponds to C++ `CoolingLine::Type` (GCodeEditor.hpp:40-69).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoolingLineType(u32);

#[allow(dead_code)]
impl CoolingLineType {
    // GCodeEditor.hpp:41  TYPE_SET_TOOL           = 1 << 0,
    pub const SET_TOOL: u32 = 1 << 0;
    // GCodeEditor.hpp:42  TYPE_EXTRUDE_END        = 1 << 1,
    pub const EXTRUDE_END: u32 = 1 << 1;
    // GCodeEditor.hpp:43  TYPE_OVERHANG_FAN_START = 1 << 2,
    pub const OVERHANG_FAN_START: u32 = 1 << 2;
    // GCodeEditor.hpp:44  TYPE_OVERHANG_FAN_END   = 1 << 3,
    pub const OVERHANG_FAN_END: u32 = 1 << 3;
    // GCodeEditor.hpp:45  TYPE_G0                 = 1 << 4,
    pub const G0: u32 = 1 << 4;
    // GCodeEditor.hpp:46  TYPE_G1                 = 1 << 5,
    pub const G1: u32 = 1 << 5;
    // GCodeEditor.hpp:47  TYPE_ADJUSTABLE         = 1 << 6,
    pub const ADJUSTABLE: u32 = 1 << 6;
    // GCodeEditor.hpp:48  TYPE_EXTERNAL_PERIMETER = 1 << 7,
    pub const EXTERNAL_PERIMETER: u32 = 1 << 7;
    // GCodeEditor.hpp:50  TYPE_HAS_F = 1 << 8, (The line sets a feedrate.)
    pub const HAS_F: u32 = 1 << 8;
    // GCodeEditor.hpp:51  TYPE_WIPE  = 1 << 9,
    pub const WIPE: u32 = 1 << 9;
    // GCodeEditor.hpp:52  TYPE_G4    = 1 << 10,
    pub const G4: u32 = 1 << 10;
    // GCodeEditor.hpp:53  TYPE_G92   = 1 << 11,
    pub const G92: u32 = 1 << 11;
    // BBS: add G2 G3 type
    // GCodeEditor.hpp:55  TYPE_G2                     = 1 << 12,
    pub const G2: u32 = 1 << 12;
    // GCodeEditor.hpp:56  TYPE_G3                     = 1 << 13,
    pub const G3: u32 = 1 << 13;
    // GCodeEditor.hpp:57  TYPE_FORCE_RESUME_FAN       = 1 << 14,
    pub const FORCE_RESUME_FAN: u32 = 1 << 14;
    // GCodeEditor.hpp:58  TYPE_SET_FAN_CHANGING_LAYER = 1 << 15,
    pub const SET_FAN_CHANGING_LAYER: u32 = 1 << 15;
    // GCodeEditor.hpp:59  TYPE_OBJECT_START           = 1 << 16,
    pub const OBJECT_START: u32 = 1 << 16;
    // GCodeEditor.hpp:60  TYPE_OBJECT_END             = 1 << 17,
    pub const OBJECT_END: u32 = 1 << 17;
    // GCodeEditor.hpp:61  TYPE_SET_FAN_CHANGING_FILAMENT = 1 << 18,
    pub const SET_FAN_CHANGING_FILAMENT: u32 = 1 << 18;
    // GCodeEditor.hpp:62  TYPE_NOT_SET_FAN_CHANGING_FILAMENT = 1 << 19,
    pub const NOT_SET_FAN_CHANGING_FILAMENT: u32 = 1 << 19;
    // Internal perimeter types for ConsistentSurface cooling logic
    // GCodeEditor.hpp:64  TYPE_INTERNAL_PERIMETER       = 1 << 20,
    pub const INTERNAL_PERIMETER: u32 = 1 << 20;
    // GCodeEditor.hpp:65  TYPE_FIRST_INTERNAL_PERIMETER = 1 << 21,
    pub const FIRST_INTERNAL_PERIMETER: u32 = 1 << 21;
    // Ironing fan speed control
    // GCodeEditor.hpp:67  TYPE_IRONING_FAN_START        = 1 << 22,
    pub const IRONING_FAN_START: u32 = 1 << 22;
    // GCodeEditor.hpp:68  TYPE_IRONING_FAN_END          = 1 << 23,
    pub const IRONING_FAN_END: u32 = 1 << 23;
}

/// A single line of G-code annotated with cooling information.
/// Corresponds to C++ `CoolingLine` (GCodeEditor.hpp:38-138).
/// NOTE: C++ field `type` is named `line_type` here (`type` is a Rust keyword).
#[derive(Debug, Clone)]
pub struct CoolingLine {
    // GCodeEditor.hpp:105  size_t type;
    pub line_type: u32,
    // Start of this line at the G-code snippet. // GCodeEditor.hpp:107
    pub line_start: usize,
    // End of this line at the G-code snippet. // GCodeEditor.hpp:109
    pub line_end: usize,
    // XY Euclidian length of this segment. // GCodeEditor.hpp:111
    pub length: f32,
    // Current feedrate, possibly adjusted. // GCodeEditor.hpp:113
    pub feedrate: f32,
    // Current duration of this segment. // GCodeEditor.hpp:115
    pub time: f32,
    // Maximum duration of this segment. // GCodeEditor.hpp:117
    pub time_max: f32,
    // If marked with the "slowdown" flag, the line has been slowed down. // GCodeEditor.hpp:119
    pub slowdown: bool,
    // Current feedrate, possibly adjusted. // GCodeEditor.hpp:121
    pub origin_feedrate: f32,
    // GCodeEditor.hpp:122  float origin_time_max = 0;
    pub origin_time_max: f32,
    // GCodeEditor.hpp:125  bool  outwall_smooth_mark = false;
    pub outwall_smooth_mark: bool,
    // GCodeEditor.hpp:126  int   object_id = -1;
    pub object_id: i32,
    // GCodeEditor.hpp:127  int   cooling_node_id = -1;
    pub cooling_node_id: i32,
    // For ConsistentSurface logic - split adjustable vs non-adjustable portions
    // GCodeEditor.hpp:130  float adjustable_length = 0.f;
    pub adjustable_length: f32,
    // GCodeEditor.hpp:131  float non_adjustable_length = 0.f;
    pub non_adjustable_length: f32,
    // GCodeEditor.hpp:132  float adjustable_time = 0.f;
    pub adjustable_time: f32,
    // GCodeEditor.hpp:133  float non_adjustable_time = 0.f;
    pub non_adjustable_time: f32,
    // GCodeEditor.hpp:134  float adjustable_time_max = 0.f;
    pub adjustable_time_max: f32,
    // Perimeter index: 0 = external, 1 = first internal, 2+ = deeper internal
    // GCodeEditor.hpp:137  std::optional<uint16_t> perimeter_index;
    pub perimeter_index: Option<u16>,
}

impl CoolingLine {
    // GCodeEditor.hpp:71  CoolingLine(unsigned int type, size_t line_start, size_t line_end)
    pub fn new(line_type: u32, line_start: usize, line_end: usize) -> Self {
        // GCodeEditor.hpp:72  : type(type), line_start(line_start), line_end(line_end), length(0.f), feedrate(0.f),
        //   origin_feedrate(0.f), time(0.f), time_max(0.f), slowdown(false)
        Self {
            line_type,
            line_start,
            line_end,
            length: 0.0,
            feedrate: 0.0,
            time: 0.0,
            time_max: 0.0,
            slowdown: false,
            origin_feedrate: 0.0,
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

    // Legacy method - used by existing code // GCodeEditor.hpp:76
    // bool adjustable(bool slowdown_external_perimeters) const
    pub fn adjustable_legacy(&self, slowdown_external_perimeters: bool) -> bool {
        // GCodeEditor.hpp:78  return (this->type & TYPE_ADJUSTABLE) &&
        //   (!(this->type & TYPE_EXTERNAL_PERIMETER) || slowdown_external_perimeters) &&
        //   this->time < this->time_max;
        (self.line_type & CoolingLineType::ADJUSTABLE) != 0
            && ((self.line_type & CoolingLineType::EXTERNAL_PERIMETER) == 0
                || slowdown_external_perimeters)
            && self.time < self.time_max
    }

    // GCodeEditor.hpp:81  bool adjustable() const
    pub fn adjustable(&self) -> bool {
        // return (this->type & TYPE_ADJUSTABLE) && this->time < this->time_max;
        (self.line_type & CoolingLineType::ADJUSTABLE) != 0 && self.time < self.time_max
    }

    // New method for ConsistentSurface logic - allows fine-grained control over which features are adjustable
    // GCodeEditor.hpp:84  bool adjustable(AdjustableFeatureType additional_slowdown_features) const
    pub fn adjustable_with_features(&self, additional_slowdown_features: AdjustableFeatureType) -> bool {
        // GCodeEditor.hpp:85
        if (self.line_type & CoolingLineType::ADJUSTABLE) == 0
            || self.adjustable_time >= self.adjustable_time_max
        {
            // GCodeEditor.hpp:86  return false;
            return false;
        }

        // GCodeEditor.hpp:89  if (this->type & TYPE_EXTERNAL_PERIMETER)
        if (self.line_type & CoolingLineType::EXTERNAL_PERIMETER) != 0 {
            // GCodeEditor.hpp:90  return (additional_slowdown_features & AdjustableFeatureType::ExternalPerimeters) != AdjustableFeatureType::None;
            return (additional_slowdown_features & AdjustableFeatureType::EXTERNAL_PERIMETERS)
                != AdjustableFeatureType::NONE;
        }

        // GCodeEditor.hpp:93  if (this->type & TYPE_FIRST_INTERNAL_PERIMETER)
        if (self.line_type & CoolingLineType::FIRST_INTERNAL_PERIMETER) != 0 {
            // GCodeEditor.hpp:94  return (additional_slowdown_features & AdjustableFeatureType::FirstInternalPerimeters) != AdjustableFeatureType::None;
            return (additional_slowdown_features & AdjustableFeatureType::FIRST_INTERNAL_PERIMETERS)
                != AdjustableFeatureType::NONE;
        }

        // GCodeEditor.hpp:97  return true;
        true
    }

    // Time calculations for ConsistentSurface logic
    // GCodeEditor.hpp:101  inline float total_time() const
    pub fn total_time(&self) -> f32 {
        self.adjustable_time + self.non_adjustable_time
    }
    // GCodeEditor.hpp:102  inline float total_length() const
    pub fn total_length(&self) -> f32 {
        self.adjustable_length + self.non_adjustable_length
    }
    // GCodeEditor.hpp:103  inline float total_time_max() const
    pub fn total_time_max(&self) -> f32 {
        self.adjustable_time_max + self.non_adjustable_time
    }
}

/// Per-extruder cooling adjustments.
/// Corresponds to C++ `PerExtruderAdjustments` (GCodeEditor.hpp:140-417).
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
    // GCodeEditor.hpp:416  size_t idx_line_end   = 0;
    pub idx_line_end: usize,
}

impl Default for PerExtruderAdjustments {
    fn default() -> Self {
        Self::new()
    }
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

    // Calculate the total elapsed time per this extruder, adjusted for the slowdown.
    // GCodeEditor.hpp:143  float elapsed_time_total() const
    pub fn elapsed_time_total(&self) -> f32 {
        // GCodeEditor.hpp:145  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:146  for (const CoolingLine &line : lines) time_total += line.time;
        for line in &self.lines {
            time_total += line.time;
        }
        // GCodeEditor.hpp:147  return time_total;
        time_total
    }

    // Calculate the total elapsed time when slowing down
    // to the minimum extrusion feed rate defined for the current material.
    // GCodeEditor.hpp:151  float maximum_time_after_slowdown(bool slowdown_external_perimeters) const
    pub fn maximum_time_after_slowdown(&self, slowdown_external_perimeters: bool) -> f32 {
        // GCodeEditor.hpp:153  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:154-161
        for line in &self.lines {
            if line.adjustable_legacy(slowdown_external_perimeters) {
                // GCodeEditor.hpp:156  if (line.time_max == FLT_MAX) return FLT_MAX;
                if line.time_max == f32::MAX {
                    return f32::MAX;
                } else {
                    // GCodeEditor.hpp:159  time_total += line.time_max;
                    time_total += line.time_max;
                }
            } else {
                // GCodeEditor.hpp:161  time_total += line.time;
                time_total += line.time;
            }
        }
        // GCodeEditor.hpp:162  return time_total;
        time_total
    }

    // Calculate the adjustable part of the total time.
    // GCodeEditor.hpp:165  float adjustable_time(bool slowdown_external_perimeters) const
    pub fn adjustable_time(&self, slowdown_external_perimeters: bool) -> f32 {
        // GCodeEditor.hpp:167  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:168-169
        for line in &self.lines {
            if line.adjustable_legacy(slowdown_external_perimeters) {
                time_total += line.time;
            }
        }
        // GCodeEditor.hpp:170  return time_total;
        time_total
    }

    // Calculate the non-adjustable part of the total time.
    // GCodeEditor.hpp:173  float non_adjustable_time(bool slowdown_external_perimeters) const
    pub fn non_adjustable_time(&self, slowdown_external_perimeters: bool) -> f32 {
        // GCodeEditor.hpp:175  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:176-177
        for line in &self.lines {
            if !line.adjustable_legacy(slowdown_external_perimeters) {
                time_total += line.time;
            }
        }
        // GCodeEditor.hpp:178  return time_total;
        time_total
    }

    // Slow down the adjustable extrusions to the minimum feedrate allowed for the current extruder material.
    // Used by both proportional and non-proportional slow down.
    // GCodeEditor.hpp:182  float slowdown_to_minimum_feedrate(bool slowdown_external_perimeters)
    pub fn slowdown_to_minimum_feedrate(&mut self, slowdown_external_perimeters: bool) -> f32 {
        // GCodeEditor.hpp:184  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:185-193
        for line in &mut self.lines {
            if line.adjustable_legacy(slowdown_external_perimeters) {
                // GCodeEditor.hpp:187  assert(line.time_max >= 0.f && line.time_max < FLT_MAX);
                debug_assert!(line.time_max >= 0.0 && line.time_max < f32::MAX);
                // GCodeEditor.hpp:188  line.slowdown = true;
                line.slowdown = true;
                // GCodeEditor.hpp:189  line.time     = line.time_max;
                line.time = line.time_max;
                // GCodeEditor.hpp:190  line.feedrate = line.length / line.time;
                line.feedrate = line.length / line.time;
            }
            // GCodeEditor.hpp:192  time_total += line.time;
            time_total += line.time;
        }
        // GCodeEditor.hpp:194  return time_total;
        time_total
    }

    // Slow down each adjustable G-code line proportionally by a factor.
    // Used by the proportional slow down.
    // GCodeEditor.hpp:198  float slow_down_proportional(float factor, bool slowdown_external_perimeters)
    pub fn slow_down_proportional(&mut self, factor: f32, slowdown_external_perimeters: bool) -> f32 {
        // GCodeEditor.hpp:200  assert(factor >= 1.f);
        debug_assert!(factor >= 1.0);
        // GCodeEditor.hpp:201  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:202-208
        for line in &mut self.lines {
            if line.adjustable_legacy(slowdown_external_perimeters) {
                // GCodeEditor.hpp:204  line.slowdown = true;
                line.slowdown = true;
                // GCodeEditor.hpp:205  line.time     = std::min(line.time_max, line.time * factor);
                line.time = line.time_max.min(line.time * factor);
                // GCodeEditor.hpp:206  line.feedrate = line.length / line.time;
                line.feedrate = line.length / line.time;
            }
            // GCodeEditor.hpp:208  time_total += line.time;
            time_total += line.time;
        }
        // GCodeEditor.hpp:210  return time_total;
        time_total
    }

    // Sort the lines, adjustable first, higher feedrate first.
    // Used by non-proportional slow down.
    // GCodeEditor.hpp:215  void sort_lines_by_decreasing_feedrate()
    pub fn sort_lines_by_decreasing_feedrate(&mut self) {
        // GCodeEditor.hpp:217-221  std::sort(... [](const CoolingLine &l1, const CoolingLine &l2) {
        //     bool adj1 = l1.adjustable();
        //     bool adj2 = l2.adjustable();
        //     return (adj1 == adj2) ? l1.feedrate > l2.feedrate : adj1;
        // });
        self.lines.sort_by(|l1, l2| {
            let adj1 = l1.adjustable();
            let adj2 = l2.adjustable();
            if adj1 == adj2 {
                // l1.feedrate > l2.feedrate (descending)
                l2.feedrate
                    .partial_cmp(&l1.feedrate)
                    .unwrap_or(std::cmp::Ordering::Equal)
            } else if adj1 {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        });

        // GCodeEditor.hpp:222  for (n_lines_adjustable = 0; n_lines_adjustable < lines.size() && this->lines[n_lines_adjustable].adjustable(); ++n_lines_adjustable);
        self.n_lines_adjustable = 0;
        while self.n_lines_adjustable < self.lines.len()
            && self.lines[self.n_lines_adjustable].adjustable()
        {
            self.n_lines_adjustable += 1;
        }
        // GCodeEditor.hpp:224  time_non_adjustable = 0.f;
        self.time_non_adjustable = 0.0;
        // GCodeEditor.hpp:225  for (size_t i = n_lines_adjustable; i < lines.size(); ++i) time_non_adjustable += lines[i].time;
        for i in self.n_lines_adjustable..self.lines.len() {
            self.time_non_adjustable += self.lines[i].time;
        }
    }

    // Calculate the maximum time stretch when slowing down to min_feedrate.
    // Slowdown to min_feedrate shall be allowed for this extruder's material.
    // Used by non-proportional slow down.
    // GCodeEditor.hpp:231  float time_stretch_when_slowing_down_to_feedrate(float min_feedrate) const
    pub fn time_stretch_when_slowing_down_to_feedrate(&self, min_feedrate: f32) -> f32 {
        // GCodeEditor.hpp:233  float time_stretch = 0.f;
        let mut time_stretch = 0.0f32;
        // GCodeEditor.hpp:234  assert(this->slow_down_min_speed < min_feedrate + EPSILON);
        // GCodeEditor.hpp:235-238
        for i in 0..self.n_lines_adjustable {
            let line = &self.lines[i];
            // GCodeEditor.hpp:237  if (line.feedrate > min_feedrate) time_stretch += line.time * (line.feedrate / min_feedrate - 1.f);
            if line.feedrate > min_feedrate {
                time_stretch += line.time * (line.feedrate / min_feedrate - 1.0);
            }
        }
        // GCodeEditor.hpp:239  return time_stretch;
        time_stretch
    }

    // Slow down all adjustable lines down to min_feedrate.
    // Slowdown to min_feedrate shall be allowed for this extruder's material.
    // Used by non-proportional slow down.
    // GCodeEditor.hpp:245  void slow_down_to_feedrate(float min_feedrate)
    pub fn slow_down_to_feedrate(&mut self, min_feedrate: f32) {
        // GCodeEditor.hpp:247  assert(this->slow_down_min_speed < min_feedrate + EPSILON);
        // GCodeEditor.hpp:248-255
        for i in 0..self.n_lines_adjustable {
            let line = &mut self.lines[i];
            // GCodeEditor.hpp:250  if (line.feedrate > min_feedrate)
            if line.feedrate > min_feedrate {
                // GCodeEditor.hpp:251  line.time *= std::max(1.f, line.feedrate / min_feedrate);
                line.time *= (line.feedrate / min_feedrate).max(1.0);
                // GCodeEditor.hpp:252  line.feedrate = min_feedrate;
                line.feedrate = min_feedrate;
                // GCodeEditor.hpp:253  line.slowdown = true;
                line.slowdown = true;
            }
        }
    }

    // collect lines time
    // GCodeEditor.hpp:259  float collection_line_times_of_extruder()
    pub fn collection_line_times_of_extruder(&self) -> f32 {
        // GCodeEditor.hpp:260  float times = 0;
        let mut times = 0.0f32;
        // GCodeEditor.hpp:261-263
        for line in &self.lines {
            times += line.time;
        }
        // GCodeEditor.hpp:264  return times;
        times
    }

    // --- ConsistentSurface cooling methods --- // GCodeEditor.hpp:267

    // Calculate the maximum time stretch when slowing down to min_feedrate,
    // considering only features allowed by additional_slowdown_features.
    // GCodeEditor.hpp:271  float time_stretch_when_slowing_down_to_feedrate(float min_feedrate, AdjustableFeatureType additional_slowdown_features) const
    pub fn time_stretch_when_slowing_down_to_feedrate_features(
        &self,
        min_feedrate: f32,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        // GCodeEditor.hpp:273  float time_stretch = 0.f;
        let mut time_stretch = 0.0f32;
        // GCodeEditor.hpp:274-278
        for i in 0..self.n_lines_adjustable {
            let line = &self.lines[i];
            // GCodeEditor.hpp:276  if (line.adjustable(additional_slowdown_features) && line.feedrate > min_feedrate)
            if line.adjustable_with_features(additional_slowdown_features) && line.feedrate > min_feedrate {
                // GCodeEditor.hpp:277  time_stretch += line.adjustable_time * (line.feedrate / min_feedrate - 1.f);
                time_stretch += line.adjustable_time * (line.feedrate / min_feedrate - 1.0);
            }
        }
        // GCodeEditor.hpp:279  return time_stretch;
        time_stretch
    }

    // Slow down all lines matching the feature type to min_feedrate.
    // GCodeEditor.hpp:283  void slow_down_to_feedrate(float min_feedrate, AdjustableFeatureType additional_slowdown_features)
    pub fn slow_down_to_feedrate_features(
        &mut self,
        min_feedrate: f32,
        additional_slowdown_features: AdjustableFeatureType,
    ) {
        // GCodeEditor.hpp:285-293
        for i in 0..self.n_lines_adjustable {
            let line = &mut self.lines[i];
            // GCodeEditor.hpp:287  if (line.adjustable(additional_slowdown_features) && line.feedrate > min_feedrate)
            if line.adjustable_with_features(additional_slowdown_features) && line.feedrate > min_feedrate {
                // GCodeEditor.hpp:288  line.adjustable_time = line.adjustable_length / min_feedrate;
                line.adjustable_time = line.adjustable_length / min_feedrate;
                // GCodeEditor.hpp:289  line.time = line.adjustable_time + line.non_adjustable_time;
                line.time = line.adjustable_time + line.non_adjustable_time;
                // GCodeEditor.hpp:290  line.feedrate = min_feedrate;
                line.feedrate = min_feedrate;
                // GCodeEditor.hpp:291  line.slowdown = true;
                line.slowdown = true;
            }
        }
    }

    // Calculate maximum time after slowdown for features matching the type.
    // GCodeEditor.hpp:297  float maximum_time_after_slowdown(AdjustableFeatureType additional_slowdown_features) const
    pub fn maximum_time_after_slowdown_features(
        &self,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        // GCodeEditor.hpp:299  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:300-308
        for line in &self.lines {
            if line.adjustable_with_features(additional_slowdown_features) {
                // GCodeEditor.hpp:302  if (line.adjustable_time_max == FLT_MAX) return FLT_MAX;
                if line.adjustable_time_max == f32::MAX {
                    return f32::MAX;
                }
                // GCodeEditor.hpp:304  time_total += line.adjustable_time_max + line.non_adjustable_time;
                time_total += line.adjustable_time_max + line.non_adjustable_time;
            } else {
                // GCodeEditor.hpp:306  time_total += line.time;
                time_total += line.time;
            }
        }
        // GCodeEditor.hpp:309  return time_total;
        time_total
    }

    // Calculate adjustable time for features matching the type.
    // GCodeEditor.hpp:313  float adjustable_time_for_features(AdjustableFeatureType additional_slowdown_features) const
    pub fn adjustable_time_for_features(
        &self,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        // GCodeEditor.hpp:315  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:316-320
        for line in &self.lines {
            // GCodeEditor.hpp:317  if (line.adjustable(additional_slowdown_features))
            if line.adjustable_with_features(additional_slowdown_features) {
                // GCodeEditor.hpp:318  time_total += line.adjustable_time;
                time_total += line.adjustable_time;
            }
        }
        // GCodeEditor.hpp:320  return time_total;
        time_total
    }

    // Slow down to minimum feedrate for features matching the type.
    // GCodeEditor.hpp:324  float slowdown_to_minimum_feedrate(AdjustableFeatureType additional_slowdown_features)
    pub fn slowdown_to_minimum_feedrate_features(
        &mut self,
        additional_slowdown_features: AdjustableFeatureType,
    ) -> f32 {
        // GCodeEditor.hpp:326  float time_total = 0.f;
        let mut time_total = 0.0f32;
        // GCodeEditor.hpp:327-335
        for line in &mut self.lines {
            // GCodeEditor.hpp:328  if (line.adjustable(additional_slowdown_features))
            if line.adjustable_with_features(additional_slowdown_features) {
                // GCodeEditor.hpp:329  line.slowdown = true;
                line.slowdown = true;
                // GCodeEditor.hpp:330  line.adjustable_time = line.adjustable_time_max;
                line.adjustable_time = line.adjustable_time_max;
                // GCodeEditor.hpp:331  line.time = line.adjustable_time + line.non_adjustable_time;
                line.time = line.adjustable_time + line.non_adjustable_time;
                // GCodeEditor.hpp:332  if (line.adjustable_length > 0)
                if line.adjustable_length > 0.0 {
                    // GCodeEditor.hpp:333  line.feedrate = line.adjustable_length / line.adjustable_time;
                    line.feedrate = line.adjustable_length / line.adjustable_time;
                }
            }
            // GCodeEditor.hpp:335  time_total += line.time;
            time_total += line.time;
        }
        // GCodeEditor.hpp:337  return time_total;
        time_total
    }

    // Create non-adjustable segments at the end of perimeter loops for transition smoothing.
    // This preserves speed in the last 'non_adjustable_length' mm of each perimeter.
    // GCodeEditor.hpp:342  void create_non_adjustable_segments(float non_adjustable_length)
    pub fn create_non_adjustable_segments(&mut self, non_adjustable_length: f32) {
        // GCodeEditor.hpp:344  if (non_adjustable_length <= 0) return;
        if non_adjustable_length <= 0.0 {
            return;
        }

        // Process lines in reverse to accumulate length from the end of each perimeter loop
        // GCodeEditor.hpp:348  float accumulated_length = 0.f;
        let mut accumulated_length = 0.0f32;
        // GCodeEditor.hpp:349  for (auto it = lines.rbegin(); it != lines.rend(); ++it)
        let slow_down_min_speed = self.slow_down_min_speed;
        for line in self.lines.iter_mut().rev() {
            // Reset accumulator at perimeter boundaries (non-adjustable lines or different feature types)
            // GCodeEditor.hpp:353  if (!(line.type & CoolingLine::TYPE_ADJUSTABLE) || (line.type & CoolingLine::TYPE_EXTRUDE_END))
            if (line.line_type & CoolingLineType::ADJUSTABLE) == 0
                || (line.line_type & CoolingLineType::EXTRUDE_END) != 0
            {
                // GCodeEditor.hpp:355  accumulated_length = 0.f;
                accumulated_length = 0.0;
                // GCodeEditor.hpp:356  continue;
                continue;
            }

            // Initialize adjustable fields if not set
            // GCodeEditor.hpp:360  if (line.adjustable_length == 0.f && line.length > 0.f)
            if line.adjustable_length == 0.0 && line.length > 0.0 {
                // GCodeEditor.hpp:361  line.adjustable_length = line.length;
                line.adjustable_length = line.length;
                // GCodeEditor.hpp:362  line.adjustable_time = line.time;
                line.adjustable_time = line.time;
                // GCodeEditor.hpp:363  line.adjustable_time_max = line.time_max;
                line.adjustable_time_max = line.time_max;
            }

            // GCodeEditor.hpp:366  float remaining_non_adjustable = non_adjustable_length - accumulated_length;
            let remaining_non_adjustable = non_adjustable_length - accumulated_length;
            // GCodeEditor.hpp:367  if (remaining_non_adjustable > 0.f && line.adjustable_length > 0.f)
            if remaining_non_adjustable > 0.0 && line.adjustable_length > 0.0 {
                // GCodeEditor.hpp:368  float convert_length = std::min(line.adjustable_length, remaining_non_adjustable);
                let convert_length = line.adjustable_length.min(remaining_non_adjustable);
                // GCodeEditor.hpp:369  float convert_ratio = convert_length / line.adjustable_length;
                let convert_ratio = convert_length / line.adjustable_length;

                // GCodeEditor.hpp:371  line.non_adjustable_length += convert_length;
                line.non_adjustable_length += convert_length;
                // GCodeEditor.hpp:372  line.non_adjustable_time += line.adjustable_time * convert_ratio;
                line.non_adjustable_time += line.adjustable_time * convert_ratio;
                // GCodeEditor.hpp:373  line.adjustable_length -= convert_length;
                line.adjustable_length -= convert_length;
                // GCodeEditor.hpp:374  line.adjustable_time -= line.adjustable_time * convert_ratio;
                line.adjustable_time -= line.adjustable_time * convert_ratio;
                // GCodeEditor.hpp:375-377  line.adjustable_time_max = (line.adjustable_length > 0.f && slow_down_min_speed > 0.f)
                //     ? line.adjustable_length / slow_down_min_speed : 0.f;
                line.adjustable_time_max = if line.adjustable_length > 0.0 && slow_down_min_speed > 0.0 {
                    line.adjustable_length / slow_down_min_speed
                } else {
                    0.0
                };

                // GCodeEditor.hpp:379  accumulated_length += convert_length;
                accumulated_length += convert_length;
            } else {
                // GCodeEditor.hpp:381  accumulated_length += line.length;
                accumulated_length += line.length;
            }
        }
    }

    // --- End ConsistentSurface cooling methods --- // GCodeEditor.hpp:386
}

// ---------------------------------------------------------------------------
// GCodeEditor class (GCodeEditor.hpp:427-484, GCodeEditor.cpp:19-672)
//
// BLOCKED: The `GCodeEditor` class itself (ctor `GCodeEditor(GCode &gcodegen)` and methods
// `reset`, `process_layer`, `parse_layer_gcode`, `write_layer_gcode`) cannot be faithfully
// ported here because they depend on:
//   * the BambuStudio `GCode` class (`gcodegen.config()`, `gcodegen.writer()`,
//     `writer().toolchange_prefix()`, `writer().extruders()`, `writer().get_position()`),
//     which is not yet ported (the crate's `gcode::GCode` is a different generator abstraction);
//   * the per-extruder `FullPrintConfig` accessors (`m_config.OPT.get_at(extruder_id)`).
//     The crate's `PrintConfig` is a flat single-value struct and is missing several options
//     used by `write_layer_gcode` (`close_additional_fan_first_x_layers`,
//     `additional_fan_full_speed_layer`, `first_x_layer_part_fan_speed`, `ironing_fan_speed`).
//
// The functional, byte-exact equivalent of these methods is implemented in
// `crate::gcode::cooling::GCodeEditorState` (`reset` / `process_layer` / `parse_layer_gcode`
// / `write_layer_gcode`) against a decoupled `PerExtruderCoolingConfig` representation that
// does not require the not-yet-ported `GCode`/`FullPrintConfig` types. Once those are ported,
// a 1:1 `GCodeEditor` wrapper can be reinstated here.
// ---------------------------------------------------------------------------

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
    fn test_sort_lines_by_decreasing_feedrate() {
        let mut adj = PerExtruderAdjustments::new();
        for (fr, t, tmax) in [(100.0f32, 1.0f32, 2.0f32), (200.0, 1.0, 2.0)] {
            let mut line =
                CoolingLine::new(CoolingLineType::G1 | CoolingLineType::ADJUSTABLE, 0, 10);
            line.feedrate = fr;
            line.time = t;
            line.time_max = tmax;
            line.length = fr * t;
            adj.lines.push(line);
        }
        adj.sort_lines_by_decreasing_feedrate();
        assert_eq!(adj.n_lines_adjustable, 2);
        assert_eq!(adj.lines[0].feedrate, 200.0);
    }
}
