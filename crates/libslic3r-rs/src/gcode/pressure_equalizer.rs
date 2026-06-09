//! Faithful 1:1 port of `src/libslic3r/GCode/PressureEqualizer.{cpp,hpp}` from
//! BambuStudio's libslic3r.
//!
//! Processes a G-code. Finds changes in the volumetric extrusion speed and
//! adjusts the transitions between these paths to limit fast changes in the
//! volumetric extrusion speed.
//!
//! Translation notes:
//!   * `coord_t -> i64`, `coordf_t -> f64`. The C++ here uses `float` for all
//!     positional / volumetric state — we mirror that with `f32`.
//!   * The C++ class indexes its per-role slope arrays directly by the integer
//!     value of `Slic3r::ExtrusionRole`, and reads the `;_EXTRUSION_ROLE:N`
//!     comment as that same raw integer. We therefore use the faithful
//!     `crate::extrusion_entity::ExtrusionRole` (a `#[repr(u8)]` mirror of the
//!     C++ enum, same ordering) and index by `role as usize`.
//!   * `m_config` (a `const GCodeConfig *`) is represented by
//!     [`PressureEqualizerConfig`], which carries exactly the config values the
//!     C++ reads: the per-extruder filament diameters, the two volumetric rate
//!     slope limits (mm^3/s^2), and `use_relative_e_distances`.
//!
//! BambuStudio Reference:
//!   - `src/libslic3r/GCode/PressureEqualizer.hpp`
//!   - `src/libslic3r/GCode/PressureEqualizer.cpp`

use crate::extrusion_entity::ExtrusionRole;
use crate::locales_utils::string_to_double_decimal_point;

// PressureEqualizer.hpp:57 — enum { numExtrusionRoles = erSupportMaterialInterface + 1 };
// erSupportMaterialInterface == 15 (see crate::extrusion_entity::ExtrusionRole),
// so numExtrusionRoles == 16.
const NUM_EXTRUSION_ROLES: usize = ExtrusionRole::SupportMaterialInterface as usize + 1;

/// Carrier for the `const Slic3r::GCodeConfig *m_config` reference used by the
/// PressureEqualizer. Holds exactly the config values the C++ reads.
///
/// PressureEqualizer.cpp:13-14 / .hpp:49
#[derive(Debug, Clone)]
pub struct PressureEqualizerConfig {
    /// `m_config->filament_diameter.values` — filament diameter (mm) per extruder.
    pub filament_diameter: Vec<f64>,
    /// `m_config->max_volumetric_extrusion_rate_slope_positive.value` (mm^3/s^2).
    pub max_volumetric_extrusion_rate_slope_positive: f64,
    /// `m_config->max_volumetric_extrusion_rate_slope_negative.value` (mm^3/s^2).
    pub max_volumetric_extrusion_rate_slope_negative: f64,
    /// `m_config->use_relative_e_distances.value`.
    pub use_relative_e_distances: bool,
}

impl Default for PressureEqualizerConfig {
    fn default() -> Self {
        Self {
            filament_diameter: vec![1.75],
            max_volumetric_extrusion_rate_slope_positive: 1.8,
            max_volumetric_extrusion_rate_slope_negative: 1.8,
            use_relative_e_distances: false,
        }
    }
}

impl PressureEqualizerConfig {
    /// Create a configuration for a single extruder with the given filament diameter.
    pub fn new(filament_diameter: f64) -> Self {
        Self {
            filament_diameter: vec![filament_diameter],
            ..Default::default()
        }
    }

    /// Set the maximum volumetric rate slopes (both positive and negative).
    pub fn with_max_slope(mut self, slope: f64) -> Self {
        self.max_volumetric_extrusion_rate_slope_positive = slope;
        self.max_volumetric_extrusion_rate_slope_negative = slope;
        self
    }

    /// Set the maximum positive volumetric rate slope.
    pub fn with_max_positive_slope(mut self, slope: f64) -> Self {
        self.max_volumetric_extrusion_rate_slope_positive = slope;
        self
    }

    /// Set the maximum negative volumetric rate slope.
    pub fn with_max_negative_slope(mut self, slope: f64) -> Self {
        self.max_volumetric_extrusion_rate_slope_negative = slope;
        self
    }

    /// Set whether to use relative E distances.
    pub fn with_relative_e(mut self, relative: bool) -> Self {
        self.use_relative_e_distances = relative;
        self
    }
}

// PressureEqualizer.hpp:75-85 — enum GCodeLineType
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

// PressureEqualizer.hpp:26-44 — struct Statistics
#[derive(Debug, Clone)]
pub struct PressureEqualizerStats {
    /// Minimum volumetric extrusion rate seen.
    pub volumetric_extrusion_rate_min: f32,
    /// Maximum volumetric extrusion rate seen.
    pub volumetric_extrusion_rate_max: f32,
    /// Average volumetric extrusion rate (length-weighted accumulator).
    pub volumetric_extrusion_rate_avg: f32,
    /// Total extrusion length processed.
    pub extrusion_length: f32,
}

impl Default for PressureEqualizerStats {
    fn default() -> Self {
        let mut s = Self {
            volumetric_extrusion_rate_min: 0.0,
            volumetric_extrusion_rate_max: 0.0,
            volumetric_extrusion_rate_avg: 0.0,
            extrusion_length: 0.0,
        };
        s.reset();
        s
    }
}

impl PressureEqualizerStats {
    // PressureEqualizer.hpp:28-33 — void reset()
    pub fn reset(&mut self) {
        self.volumetric_extrusion_rate_min = f32::MAX;
        self.volumetric_extrusion_rate_max = 0.0;
        self.volumetric_extrusion_rate_avg = 0.0;
        self.extrusion_length = 0.0;
    }

    // PressureEqualizer.hpp:34-39 — void update(float volumetric_extrusion_rate, float length)
    fn update(&mut self, volumetric_extrusion_rate: f32, length: f32) {
        self.volumetric_extrusion_rate_min =
            self.volumetric_extrusion_rate_min.min(volumetric_extrusion_rate);
        self.volumetric_extrusion_rate_max =
            self.volumetric_extrusion_rate_max.max(volumetric_extrusion_rate);
        self.volumetric_extrusion_rate_avg += volumetric_extrusion_rate * length;
        self.extrusion_length += length;
    }
}

// PressureEqualizer.hpp:53-56 — struct ExtrusionRateSlope { float positive; float negative; }
#[derive(Debug, Clone, Copy)]
struct ExtrusionRateSlope {
    positive: f32,
    negative: f32,
}

// PressureEqualizer.hpp:87-155 — struct GCodeLine
#[derive(Debug, Clone)]
struct GCodeLine {
    // PressureEqualizer.hpp:122
    type_: GCodeLineType,

    // PressureEqualizer.hpp:124-126 — raw text + its length.
    raw: Vec<u8>,
    raw_length: usize,
    // PressureEqualizer.hpp:129
    modified: bool,

    // PressureEqualizer.hpp:134-137 — X,Y,Z,E,F state and which axes were provided.
    pos_start: [f32; 5],
    pos_end: [f32; 5],
    pos_provided: [bool; 5],

    // PressureEqualizer.hpp:140
    #[allow(dead_code)]
    extruder_id: usize,
    // PressureEqualizer.hpp:142
    extrusion_role: ExtrusionRole,

    // PressureEqualizer.hpp:145-149
    volumetric_extrusion_rate: f32,
    volumetric_extrusion_rate_start: f32,
    volumetric_extrusion_rate_end: f32,

    // PressureEqualizer.hpp:153-154
    max_volumetric_extrusion_rate_slope_positive: f32,
    max_volumetric_extrusion_rate_slope_negative: f32,
}

impl Default for GCodeLine {
    // PressureEqualizer.hpp:89-97 — GCodeLine()
    fn default() -> Self {
        Self {
            type_: GCodeLineType::Invalid,
            raw: Vec::new(),
            raw_length: 0,
            modified: false,
            // The C++ ctor leaves pos_start/pos_end/pos_provided uninitialised;
            // process_line() always fully sets them before use. Zero them here.
            pos_start: [0.0; 5],
            pos_end: [0.0; 5],
            pos_provided: [false; 5],
            extruder_id: 0,
            extrusion_role: ExtrusionRole::None,
            volumetric_extrusion_rate: 0.0,
            volumetric_extrusion_rate_start: 0.0,
            volumetric_extrusion_rate_end: 0.0,
            max_volumetric_extrusion_rate_slope_positive: 0.0,
            max_volumetric_extrusion_rate_slope_negative: 0.0,
        }
    }
}

impl GCodeLine {
    // PressureEqualizer.hpp:99
    fn moving_xy(&self) -> bool {
        (self.pos_end[0] - self.pos_start[0]).abs() > 0.0
            || (self.pos_end[1] - self.pos_start[1]).abs() > 0.0
    }

    // PressureEqualizer.hpp:100
    #[allow(dead_code)]
    fn moving_z(&self) -> bool {
        (self.pos_end[2] - self.pos_start[2]).abs() > 0.0
    }

    // PressureEqualizer.hpp:101
    fn extruding(&self) -> bool {
        self.moving_xy() && self.pos_end[3] > self.pos_start[3]
    }

    // PressureEqualizer.hpp:102
    #[allow(dead_code)]
    fn retracting(&self) -> bool {
        self.pos_end[3] < self.pos_start[3]
    }

    // PressureEqualizer.hpp:103
    #[allow(dead_code)]
    fn deretracting(&self) -> bool {
        !self.moving_xy() && self.pos_end[3] > self.pos_start[3]
    }

    // PressureEqualizer.hpp:105
    #[allow(dead_code)]
    fn dist_xy2(&self) -> f32 {
        (self.pos_end[0] - self.pos_start[0]) * (self.pos_end[0] - self.pos_start[0])
            + (self.pos_end[1] - self.pos_start[1]) * (self.pos_end[1] - self.pos_start[1])
    }

    // PressureEqualizer.hpp:106
    fn dist_xyz2(&self) -> f32 {
        (self.pos_end[0] - self.pos_start[0]) * (self.pos_end[0] - self.pos_start[0])
            + (self.pos_end[1] - self.pos_start[1]) * (self.pos_end[1] - self.pos_start[1])
            + (self.pos_end[2] - self.pos_start[2]) * (self.pos_end[2] - self.pos_start[2])
    }

    // PressureEqualizer.hpp:107
    #[allow(dead_code)]
    fn dist_xy(&self) -> f32 {
        self.dist_xy2().sqrt()
    }

    // PressureEqualizer.hpp:108
    fn dist_xyz(&self) -> f32 {
        self.dist_xyz2().sqrt()
    }

    // PressureEqualizer.hpp:109
    #[allow(dead_code)]
    fn dist_e(&self) -> f32 {
        (self.pos_end[3] - self.pos_start[3]).abs()
    }

    // PressureEqualizer.hpp:111
    fn feedrate(&self) -> f32 {
        self.pos_end[4]
    }

    // PressureEqualizer.hpp:112
    fn time(&self) -> f32 {
        self.dist_xyz() / self.feedrate()
    }

    // PressureEqualizer.hpp:113
    #[allow(dead_code)]
    fn time_inv(&self) -> f32 {
        self.feedrate() / self.dist_xyz()
    }

    // PressureEqualizer.hpp:114-119
    fn volumetric_correction_avg(&self) -> f32 {
        let avg_correction = 0.5
            * (self.volumetric_extrusion_rate_start + self.volumetric_extrusion_rate_end)
            / self.volumetric_extrusion_rate;
        debug_assert!(avg_correction > 0.0);
        debug_assert!(avg_correction <= 1.000_000_01);
        avg_correction
    }

    // PressureEqualizer.hpp:120
    fn time_corrected(&self) -> f32 {
        self.time() * self.volumetric_correction_avg()
    }
}

/// Processes a G-code. Finds changes in the volumetric extrusion speed and
/// adjusts the transitions between these paths to limit fast changes in the
/// volumetric extrusion speed.
///
/// PressureEqualizer.hpp:12-208 — class PressureEqualizer
pub struct PressureEqualizer {
    // PressureEqualizer.hpp:46
    m_stat: PressureEqualizerStats,

    // PressureEqualizer.hpp:49 — keeps the reference, does not own the config.
    m_config: PressureEqualizerConfig,

    // PressureEqualizer.hpp:58-60
    m_max_volumetric_extrusion_rate_slopes: [ExtrusionRateSlope; NUM_EXTRUSION_ROLES],
    m_max_volumetric_extrusion_rate_slope_positive: f32,
    m_max_volumetric_extrusion_rate_slope_negative: f32,
    // PressureEqualizer.hpp:62
    m_max_segment_length: f32,

    // PressureEqualizer.hpp:66
    m_filament_crossections: Vec<f32>,

    // PressureEqualizer.hpp:70-73 — X,Y,Z,E,F
    m_current_pos: [f32; 5],
    m_current_extruder: usize,
    m_current_extrusion_role: ExtrusionRole,
    m_retracted: bool,

    // PressureEqualizer.hpp:158-164
    circular_buffer: Vec<GCodeLine>,
    circular_buffer_pos: usize,
    circular_buffer_size: usize,
    circular_buffer_items: usize,

    // PressureEqualizer.hpp:167-168
    output_buffer: Vec<u8>,
    output_buffer_length: usize,

    // PressureEqualizer.hpp:171
    line_idx: usize,
}

impl PressureEqualizer {
    // PressureEqualizer.cpp:13-17 — PressureEqualizer(const Slic3r::GCodeConfig *config)
    pub fn new(config: PressureEqualizerConfig) -> Self {
        let mut pe = Self {
            m_stat: PressureEqualizerStats::default(),
            m_config: config,
            m_max_volumetric_extrusion_rate_slopes: [ExtrusionRateSlope {
                positive: 0.0,
                negative: 0.0,
            }; NUM_EXTRUSION_ROLES],
            m_max_volumetric_extrusion_rate_slope_positive: 0.0,
            m_max_volumetric_extrusion_rate_slope_negative: 0.0,
            m_max_segment_length: 0.0,
            m_filament_crossections: Vec::new(),
            m_current_pos: [0.0; 5],
            m_current_extruder: 0,
            m_current_extrusion_role: ExtrusionRole::None,
            m_retracted: false,
            circular_buffer: Vec::new(),
            circular_buffer_pos: 0,
            circular_buffer_size: 0,
            circular_buffer_items: 0,
            output_buffer: Vec::new(),
            output_buffer_length: 0,
            line_idx: 0,
        };
        pe.reset();
        pe
    }

    // PressureEqualizer.cpp:23-72 — void reset()
    pub fn reset(&mut self) {
        self.circular_buffer_pos = 0;
        self.circular_buffer_size = 100;
        self.circular_buffer_items = 0;
        self.circular_buffer
            .splice(.., std::iter::repeat(GCodeLine::default()).take(self.circular_buffer_size));

        // Preallocate some data, so that output_buffer.data() will return an empty string.
        self.output_buffer.clear();
        self.output_buffer.resize(32, 0);
        self.output_buffer_length = 0;

        self.m_current_extruder = 0;
        // Zero the position of the XYZE axes + the current feed
        self.m_current_pos = [0.0; 5];
        self.m_current_extrusion_role = ExtrusionRole::None;
        // Expect the first command to fill the nozzle (deretract).
        self.m_retracted = true;

        // Calculate filamet crossections for the multiple extruders.
        self.m_filament_crossections.clear();
        for i in 0..self.m_config.filament_diameter.len() {
            let r = self.m_config.filament_diameter[i];
            let a = 0.25f64 * std::f64::consts::PI * r * r;
            self.m_filament_crossections.push(a as f32);
        }

        self.m_max_segment_length = 20.0;
        // Volumetric rate of a 0.45mm x 0.2mm extrusion at 60mm/s XY movement: 0.45*0.2*60*60=5.4*60 = 324 mm^3/min
        // Volumetric rate of a 0.45mm x 0.2mm extrusion at 20mm/s XY movement: 0.45*0.2*20*60=1.8*60 = 108 mm^3/min
        // Slope of the volumetric rate, changing from 20mm/s to 60mm/s over 2 seconds: (5.4-1.8)*60*60/2=60*60*1.8 = 6480 mm^3/min^2 = 1.8 mm^3/s^2
        self.m_max_volumetric_extrusion_rate_slope_positive =
            (self.m_config.max_volumetric_extrusion_rate_slope_positive * 60.0 * 60.0) as f32;
        self.m_max_volumetric_extrusion_rate_slope_negative =
            (self.m_config.max_volumetric_extrusion_rate_slope_negative * 60.0 * 60.0) as f32;

        for i in 0..NUM_EXTRUSION_ROLES {
            self.m_max_volumetric_extrusion_rate_slopes[i].negative =
                self.m_max_volumetric_extrusion_rate_slope_negative;
            self.m_max_volumetric_extrusion_rate_slopes[i].positive =
                self.m_max_volumetric_extrusion_rate_slope_positive;
        }

        // Don't regulate the pressure in infill.
        self.m_max_volumetric_extrusion_rate_slopes[ExtrusionRole::BridgeInfill as usize].negative =
            0.0;
        self.m_max_volumetric_extrusion_rate_slopes[ExtrusionRole::BridgeInfill as usize].positive =
            0.0;
        // Don't regulate the pressure in gap fill.
        self.m_max_volumetric_extrusion_rate_slopes[ExtrusionRole::GapFill as usize].negative = 0.0;
        self.m_max_volumetric_extrusion_rate_slopes[ExtrusionRole::GapFill as usize].positive = 0.0;

        self.m_stat.reset();
        self.line_idx = 0;
    }

    /// Get the current statistics.
    pub fn stats(&self) -> &PressureEqualizerStats {
        &self.m_stat
    }

    // PressureEqualizer.hpp:23 — size_t get_output_buffer_length() const
    pub fn get_output_buffer_length(&self) -> usize {
        self.output_buffer_length
    }

    // PressureEqualizer.cpp:74-129 — const char* process(const char *szGCode, bool flush)
    //
    // The C++ returns a `const char*` into the internal output_buffer. We return
    // the produced output as a `String` (the bytes pushed for this call).
    pub fn process(&mut self, sz_gcode: &str, flush: bool) -> String {
        // Reset length of the output_buffer.
        self.output_buffer_length = 0;

        // if (szGCode != 0) {  — always true for a &str.
        {
            let bytes = sz_gcode.as_bytes();
            let mut p: usize = 0;
            while p < bytes.len() && bytes[p] != 0 {
                // Find end of the line.
                let mut endl = p;
                // Slic3r always generates end of lines in a Unix style.
                while endl < bytes.len() && bytes[endl] != 0 && bytes[endl] != b'\n' {
                    endl += 1;
                }
                if self.circular_buffer_items == self.circular_buffer_size {
                    // Buffer is full. Push out the oldest line.
                    let pos = self.circular_buffer_pos;
                    self.output_gcode_line(pos);
                } else {
                    self.circular_buffer_items += 1;
                }
                // Process a G-code line, store it into the provided GCodeLine object.
                let idx_tail = self.circular_buffer_pos;
                self.circular_buffer_pos = self.circular_buffer_idx_next(self.circular_buffer_pos);
                if !self.process_line(&bytes[p..endl], endl - p, idx_tail) {
                    // The line has to be forgotten. It contains comment marks, which shall be
                    // filtered out of the target g-code.
                    self.circular_buffer_pos = idx_tail;
                    self.circular_buffer_items -= 1;
                }
                p = endl;
                if p < bytes.len() && bytes[p] == b'\n' {
                    p += 1;
                }
            }
        }

        if flush {
            // Flush the remaining valid lines of the circular buffer.
            let mut idx = self.circular_buffer_idx_head();
            while self.circular_buffer_items > 0 {
                self.output_gcode_line(idx);
                idx += 1;
                if idx == self.circular_buffer_size {
                    idx = 0;
                }
                self.circular_buffer_items -= 1;
            }
            // Reset the index pointer.
            debug_assert!(self.circular_buffer_items == 0);
            self.circular_buffer_pos = 0;

            // #if 1
            // Statistics (the C++ prints these to stdout).
            if self.m_stat.extrusion_length > 0.0 {
                self.m_stat.volumetric_extrusion_rate_avg /= self.m_stat.extrusion_length;
            }
            self.m_stat.reset();
            // #endif
        }

        // return output_buffer.data();
        String::from_utf8_lossy(&self.output_buffer[..self.output_buffer_length]).into_owned()
    }

    // PressureEqualizer.cpp:169-359 — bool process_line(const char *line, const size_t len, GCodeLine &buf)
    fn process_line(&mut self, line: &[u8], len: usize, buf_idx: usize) -> bool {
        // PressureEqualizer.cpp:171
        const EXTRUSION_ROLE_TAG: &[u8] = b";_EXTRUSION_ROLE:";

        if line.len() >= EXTRUSION_ROLE_TAG.len()
            && &line[..EXTRUSION_ROLE_TAG.len()] == EXTRUSION_ROLE_TAG
        {
            // line += strlen(EXTRUSION_ROLE_TAG);
            let rest = &line[EXTRUSION_ROLE_TAG.len()..];
            // int role = atoi(line);
            let role = atoi(rest);
            // m_current_extrusion_role = ExtrusionRole(role);
            self.m_current_extrusion_role = extrusion_role_from_int(role);
            self.line_idx += 1;
            return false;
        }

        // Set the type, copy the line to the buffer.
        self.circular_buffer[buf_idx].type_ = GCodeLineType::Other;
        self.circular_buffer[buf_idx].modified = false;
        // buf.raw holds the line text plus a trailing NUL (as in C++).
        {
            let buf = &mut self.circular_buffer[buf_idx];
            buf.raw.clear();
            buf.raw.extend_from_slice(&line[..len]);
            buf.raw.push(0);
            buf.raw_length = len;
        }

        self.circular_buffer[buf_idx].pos_start = self.m_current_pos;
        self.circular_buffer[buf_idx].pos_end = self.m_current_pos;
        self.circular_buffer[buf_idx].pos_provided = [false; 5];

        self.circular_buffer[buf_idx].volumetric_extrusion_rate = 0.0;
        self.circular_buffer[buf_idx].volumetric_extrusion_rate_start = 0.0;
        self.circular_buffer[buf_idx].volumetric_extrusion_rate_end = 0.0;
        self.circular_buffer[buf_idx].max_volumetric_extrusion_rate_slope_positive = 0.0;
        self.circular_buffer[buf_idx].max_volumetric_extrusion_rate_slope_negative = 0.0;
        self.circular_buffer[buf_idx].extrusion_role = self.m_current_extrusion_role;

        // Parse the G-code line, store the result into the buf.
        // switch (toupper(*line ++))
        let mut cur: usize = 0;
        let first = if cur < line.len() {
            (line[cur] as char).to_ascii_uppercase()
        } else {
            '\0'
        };
        cur += 1;
        match first {
            'G' => {
                let gcode = parse_int(line, &mut cur);
                eatws(line, &mut cur);
                match gcode {
                    0 | 1 => {
                        // G0, G1: A FFF 3D printer does not make a difference between the two.
                        let mut new_pos: [f32; 5] = self.m_current_pos;
                        let mut changed = [false; 5];
                        while !is_eol(char_at(line, cur)) {
                            let axis = (char_at(line, cur) as char).to_ascii_uppercase();
                            cur += 1;
                            let i: i32 = match axis {
                                'X' | 'Y' | 'Z' => (axis as i32) - ('X' as i32),
                                'E' => 3,
                                'F' => 4,
                                _ => {
                                    debug_assert!(false);
                                    -1
                                }
                            };
                            if i == -1 {
                                panic!(
                                    "GCode::PressureEqualizer: Invalid axis for G0/G1: {}",
                                    axis
                                );
                            }
                            let i = i as usize;
                            self.circular_buffer[buf_idx].pos_provided[i] = true;
                            new_pos[i] = parse_float(line, &mut cur);
                            if i == 3 && self.m_config.use_relative_e_distances {
                                new_pos[i] += self.m_current_pos[i];
                            }
                            changed[i] = new_pos[i] != self.m_current_pos[i];
                            eatws(line, &mut cur);
                        }
                        if changed[3] {
                            // Extrusion, retract or unretract.
                            let diff = new_pos[3] - self.m_current_pos[3];
                            if diff < 0.0 {
                                self.circular_buffer[buf_idx].type_ = GCodeLineType::Retract;
                                self.m_retracted = true;
                            } else if !changed[0] && !changed[1] && !changed[2] {
                                // assert(m_retracted);
                                self.circular_buffer[buf_idx].type_ = GCodeLineType::Unretract;
                                self.m_retracted = false;
                            } else {
                                debug_assert!(changed[0] || changed[1]);
                                // Moving in XY plane.
                                self.circular_buffer[buf_idx].type_ = GCodeLineType::Extrude;
                                // Calculate the volumetric extrusion rate.
                                let mut diff4 = [0.0f32; 4];
                                for i in 0..4 {
                                    diff4[i] = new_pos[i] - self.m_current_pos[i];
                                }
                                // volumetric extrusion rate = A_filament * F_xyz * L_e / L_xyz [mm^3/min]
                                let len2 =
                                    diff4[0] * diff4[0] + diff4[1] * diff4[1] + diff4[2] * diff4[2];
                                let rate = self.m_filament_crossections[self.m_current_extruder]
                                    * new_pos[4]
                                    * ((diff4[3] * diff4[3]) / len2).sqrt();
                                self.circular_buffer[buf_idx].volumetric_extrusion_rate = rate;
                                self.circular_buffer[buf_idx].volumetric_extrusion_rate_start = rate;
                                self.circular_buffer[buf_idx].volumetric_extrusion_rate_end = rate;
                                self.m_stat.update(rate, len2.sqrt());
                                // The C++ prints a warning for extremely low flow rates
                                // (rate < 40.f); we omit the stdout side-effect.
                            }
                        } else if changed[0] || changed[1] || changed[2] {
                            // Moving without extrusion.
                            self.circular_buffer[buf_idx].type_ = GCodeLineType::Move;
                        }
                        self.m_current_pos = new_pos;
                    }
                    92 => {
                        // G92 : Set Position
                        // Set a logical coordinate position to a new value without actually moving the machine motors.
                        // Which axes to set?
                        let mut set = false;
                        while !is_eol(char_at(line, cur)) {
                            let axis = (char_at(line, cur) as char).to_ascii_uppercase();
                            cur += 1;
                            match axis {
                                'X' | 'Y' | 'Z' => {
                                    let idx = ((axis as i32) - ('X' as i32)) as usize;
                                    self.m_current_pos[idx] = if !is_ws_or_eol(char_at(line, cur)) {
                                        parse_float(line, &mut cur)
                                    } else {
                                        0.0
                                    };
                                    set = true;
                                }
                                'E' => {
                                    self.m_current_pos[3] = if !is_ws_or_eol(char_at(line, cur)) {
                                        parse_float(line, &mut cur)
                                    } else {
                                        0.0
                                    };
                                    set = true;
                                }
                                _ => {
                                    panic!(
                                        "GCode::PressureEqualizer: Incorrect axis in a G92 G-code: {}",
                                        axis
                                    );
                                }
                            }
                            eatws(line, &mut cur);
                        }
                        debug_assert!(set);
                    }
                    10 | 22 => {
                        // Firmware retract.
                        self.circular_buffer[buf_idx].type_ = GCodeLineType::Retract;
                        self.m_retracted = true;
                    }
                    11 | 23 => {
                        // Firmware unretract.
                        self.circular_buffer[buf_idx].type_ = GCodeLineType::Unretract;
                        self.m_retracted = false;
                    }
                    _ => {
                        // Ignore the rest.
                    }
                }
            }
            'M' => {
                let _mcode = parse_int(line, &mut cur);
                eatws(line, &mut cur);
                // Ignore the rest of the M-codes.
            }
            'T' => {
                // Activate an extruder head.
                let new_extruder = parse_int(line, &mut cur) as usize;
                if new_extruder != self.m_current_extruder {
                    self.m_current_extruder = new_extruder;
                    self.m_retracted = true;
                    self.circular_buffer[buf_idx].type_ = GCodeLineType::ToolChange;
                } else {
                    self.circular_buffer[buf_idx].type_ = GCodeLineType::Noop;
                }
            }
            _ => {}
        }

        self.circular_buffer[buf_idx].extruder_id = self.m_current_extruder;
        self.circular_buffer[buf_idx].pos_end = self.m_current_pos;

        self.adjust_volumetric_rate();
        self.line_idx += 1;
        true
    }

    // PressureEqualizer.cpp:361-457 — void output_gcode_line(GCodeLine &line)
    fn output_gcode_line(&mut self, idx: usize) {
        if !self.circular_buffer[idx].modified {
            let raw = self.circular_buffer[idx].raw.clone();
            let raw_length = self.circular_buffer[idx].raw_length;
            self.push_to_output(&raw, raw_length, true);
            return;
        }

        // The line was modified.
        // Find the comment.
        let comment: Option<Vec<u8>> = {
            let raw = &self.circular_buffer[idx].raw;
            let mut c = 0usize;
            while c < raw.len() && raw[c] != b';' && raw[c] != 0 {
                c += 1;
            }
            if c < raw.len() && raw[c] == b';' {
                // Comment is the NUL-terminated tail of raw starting at the ';'.
                let mut v = Vec::new();
                let mut k = c;
                while k < raw.len() && raw[k] != 0 {
                    v.push(raw[k]);
                    k += 1;
                }
                Some(v)
            } else {
                None
            }
        };
        let mut comment: Option<&[u8]> = comment.as_deref();

        // Emit the line with lowered extrusion rates.
        let l2 = self.circular_buffer[idx].dist_xyz2();
        let l = l2.sqrt();
        let mut n_segments = (l / self.m_max_segment_length).ceil() as usize;
        if n_segments == 1 {
            // Just update this segment.
            let new_feedrate = self.circular_buffer[idx].feedrate()
                * self.circular_buffer[idx].volumetric_correction_avg();
            self.push_line_to_output(idx, new_feedrate, comment);
        } else {
            let accelerating = self.circular_buffer[idx].volumetric_extrusion_rate_start
                < self.circular_buffer[idx].volumetric_extrusion_rate_end;
            // Update the initial and final feed rate values.
            {
                let line = &mut self.circular_buffer[idx];
                line.pos_start[4] = line.volumetric_extrusion_rate_start * line.pos_end[4]
                    / line.volumetric_extrusion_rate;
                line.pos_end[4] = line.volumetric_extrusion_rate_end * line.pos_end[4]
                    / line.volumetric_extrusion_rate;
            }
            let feed_avg =
                0.5 * (self.circular_buffer[idx].pos_start[4] + self.circular_buffer[idx].pos_end[4]);
            // Limiting volumetric extrusion rate slope for this segment.
            let max_volumetric_extrusion_rate_slope = if accelerating {
                self.circular_buffer[idx].max_volumetric_extrusion_rate_slope_positive
            } else {
                self.circular_buffer[idx].max_volumetric_extrusion_rate_slope_negative
            };
            // Total time for the segment, corrected for the possibly lowered volumetric feed rate,
            // if accelerating / decelerating over the complete segment.
            let t_total = self.circular_buffer[idx].dist_xyz() / feed_avg;
            // Time of the acceleration / deceleration part of the segment, if accelerating / decelerating
            // with the maximum volumetric extrusion rate slope.
            let t_acc = 0.5
                * (self.circular_buffer[idx].volumetric_extrusion_rate_start
                    + self.circular_buffer[idx].volumetric_extrusion_rate_end)
                / max_volumetric_extrusion_rate_slope;
            // NOTE: The C++ declares `float l_acc = l;` and `float l_steady = 0.f;`
            // here, and then *shadows* them inside the `if (t_acc < t_total)` block
            // with new locals of the same name. As a result the outer `l_acc`/
            // `l_steady` retain their initial values (`l` and `0`) for the rest of
            // this function, and the recomputed-`nSegments` is the only effect that
            // escapes the block. We reproduce that bug-for-bug behaviour faithfully.
            let l_acc: f32 = l;
            let l_steady: f32 = 0.0;
            if t_acc < t_total {
                // One may achieve higher print speeds if part of the segment is not speed limited.
                let mut l_acc_inner = t_acc * feed_avg; // shadows outer l_acc (no effect outside)
                let mut l_steady_inner = l - l_acc_inner; // shadows outer l_steady (no effect outside)
                if l_steady_inner < 0.5 * self.m_max_segment_length {
                    l_acc_inner = l;
                    l_steady_inner = 0.0;
                    let _ = (l_acc_inner, l_steady_inner);
                } else {
                    n_segments = (l_acc_inner / self.m_max_segment_length).ceil() as usize;
                    let _ = (l_acc_inner, l_steady_inner);
                }
            }
            let mut pos_start: [f32; 5] = self.circular_buffer[idx].pos_start;
            let mut pos_end: [f32; 5] = self.circular_buffer[idx].pos_end;
            let mut pos_end2: [f32; 4] = [0.0; 4];
            if l_steady > 0.0 {
                // There will be a steady feed segment emitted.
                if accelerating {
                    // Prepare the final steady feed rate segment.
                    pos_end2.copy_from_slice(&pos_end[..4]);
                    let t = l_acc / l;
                    for i in 0..4 {
                        pos_end[i] = pos_start[i] + (pos_end[i] - pos_start[i]) * t;
                        self.circular_buffer[idx].pos_provided[i] = true;
                    }
                } else {
                    // Emit the steady feed rate segment.
                    let t = l_steady / l;
                    for i in 0..4 {
                        let v = pos_start[i] + (pos_end[i] - pos_start[i]) * t;
                        self.circular_buffer[idx].pos_end[i] = v;
                        self.circular_buffer[idx].pos_provided[i] = true;
                    }
                    self.push_line_to_output(idx, pos_start[4], comment);
                    comment = None;
                    let new_start = self.circular_buffer[idx].pos_end;
                    self.circular_buffer[idx].pos_start = new_start;
                    pos_start = new_start;
                }
            }
            // Split the segment into pieces.
            for i in 1..n_segments {
                let t = (i as f32) / (n_segments as f32);
                for j in 0..4 {
                    let v = pos_start[j] + (pos_end[j] - pos_start[j]) * t;
                    self.circular_buffer[idx].pos_end[j] = v;
                    self.circular_buffer[idx].pos_provided[j] = true;
                }
                // Interpolate the feed rate at the center of the segment.
                let fr = pos_start[4]
                    + (pos_end[4] - pos_start[4]) * ((i as f32) - 0.5) / (n_segments as f32);
                self.push_line_to_output(idx, fr, comment);
                comment = None;
                let new_start = self.circular_buffer[idx].pos_end;
                self.circular_buffer[idx].pos_start = new_start;
            }
            if l_steady > 0.0 && accelerating {
                for i in 0..4 {
                    self.circular_buffer[idx].pos_end[i] = pos_end2[i];
                    self.circular_buffer[idx].pos_provided[i] = true;
                }
                self.push_line_to_output(idx, pos_end[4], comment);
            }
        }
    }

    // PressureEqualizer.cpp:459-567 — void adjust_volumetric_rate()
    fn adjust_volumetric_rate(&mut self) {
        if self.circular_buffer_items < 2 {
            return;
        }

        // Go back from the current circular_buffer_pos and lower the feedtrate to decrease the slope of the extrusion rate changes.
        let idx_head = self.circular_buffer_idx_head();
        let idx_tail = self.circular_buffer_idx_prev(self.circular_buffer_idx_tail());
        let mut idx = idx_tail;
        if idx == idx_head || !self.circular_buffer[idx].extruding() {
            // Nothing to do, the last move is not extruding.
            return;
        }

        let mut feedrate_per_extrusion_role = [f32::MAX; NUM_EXTRUSION_ROLES];
        feedrate_per_extrusion_role
            [self.circular_buffer[idx].extrusion_role as usize] =
            self.circular_buffer[idx].volumetric_extrusion_rate_start;

        // PressureEqualizer.cpp:477 — bool modified = true;
        // The C++ `modified = false/true` updates inside the loop are commented
        // out (cpp:508/516), so `modified` stays true for the whole loop; we keep
        // the variable and condition for fidelity but it is never reassigned.
        let modified = true;
        while modified && idx != idx_head {
            let mut idx_prev = self.circular_buffer_idx_prev(idx);
            while !self.circular_buffer[idx_prev].extruding() && idx_prev != idx_head {
                idx_prev = self.circular_buffer_idx_prev(idx_prev);
            }
            if !self.circular_buffer[idx_prev].extruding() {
                break;
            }
            // Volumetric extrusion rate at the start of the succeding segment.
            let rate_succ = self.circular_buffer[idx].volumetric_extrusion_rate_start;
            // What is the gradient of the extrusion rate between idx_prev and idx?
            idx = idx_prev;
            let line_role = self.circular_buffer[idx].extrusion_role as usize;
            for i_role in 1..NUM_EXTRUSION_ROLES {
                let rate_slope = self.m_max_volumetric_extrusion_rate_slopes[i_role].negative;
                if rate_slope == 0.0 {
                    // The negative rate is unlimited.
                    continue;
                }
                let mut rate_end = feedrate_per_extrusion_role[i_role];
                if i_role == line_role && rate_succ < rate_end {
                    // Limit by the succeeding volumetric flow rate.
                    rate_end = rate_succ;
                }
                if self.circular_buffer[idx].volumetric_extrusion_rate_end > rate_end {
                    self.circular_buffer[idx].volumetric_extrusion_rate_end = rate_end;
                    self.circular_buffer[idx].modified = true;
                } else if i_role == line_role {
                    rate_end = self.circular_buffer[idx].volumetric_extrusion_rate_end;
                } else if rate_end == f32::MAX {
                    // The rate for ExtrusionRole iRole is unlimited.
                    continue;
                } else {
                    // Use the original, 'floating' extrusion rate as a starting point for the limiter.
                }
                // modified = false;
                let rate_start = rate_end + rate_slope * self.circular_buffer[idx].time_corrected();
                if rate_start < self.circular_buffer[idx].volumetric_extrusion_rate_start {
                    // Limit the volumetric extrusion rate at the start of this segment due to a segment
                    // of ExtrusionType iRole, which will be extruded in the future.
                    self.circular_buffer[idx].volumetric_extrusion_rate_start = rate_start;
                    self.circular_buffer[idx].max_volumetric_extrusion_rate_slope_negative =
                        rate_slope;
                    self.circular_buffer[idx].modified = true;
                    // modified = true;
                }
                feedrate_per_extrusion_role[i_role] = if i_role == line_role {
                    self.circular_buffer[idx].volumetric_extrusion_rate_start
                } else {
                    rate_start
                };
            }
        }

        // Go forward and adjust the feedrate to decrease the slope of the extrusion rate changes.
        for i in 0..NUM_EXTRUSION_ROLES {
            feedrate_per_extrusion_role[i] = f32::MAX;
        }
        feedrate_per_extrusion_role
            [self.circular_buffer[idx].extrusion_role as usize] =
            self.circular_buffer[idx].volumetric_extrusion_rate_end;

        debug_assert!(self.circular_buffer[idx].extruding());
        while idx != idx_tail {
            let mut idx_next = self.circular_buffer_idx_next(idx);
            while !self.circular_buffer[idx_next].extruding() && idx_next != idx_tail {
                idx_next = self.circular_buffer_idx_next(idx_next);
            }
            if !self.circular_buffer[idx_next].extruding() {
                break;
            }
            let rate_prec = self.circular_buffer[idx].volumetric_extrusion_rate_end;
            // What is the gradient of the extrusion rate between idx_prev and idx?
            idx = idx_next;
            let line_role = self.circular_buffer[idx].extrusion_role as usize;
            for i_role in 1..NUM_EXTRUSION_ROLES {
                let rate_slope = self.m_max_volumetric_extrusion_rate_slopes[i_role].positive;
                if rate_slope == 0.0 {
                    // The positive rate is unlimited.
                    continue;
                }
                let mut rate_start = feedrate_per_extrusion_role[i_role];
                if i_role == line_role && rate_prec < rate_start {
                    rate_start = rate_prec;
                }
                if self.circular_buffer[idx].volumetric_extrusion_rate_start > rate_start {
                    self.circular_buffer[idx].volumetric_extrusion_rate_start = rate_start;
                    self.circular_buffer[idx].modified = true;
                } else if i_role == line_role {
                    rate_start = self.circular_buffer[idx].volumetric_extrusion_rate_start;
                } else if rate_start == f32::MAX {
                    // The rate for ExtrusionRole iRole is unlimited.
                    continue;
                } else {
                    // Use the original, 'floating' extrusion rate as a starting point for the limiter.
                }
                let rate_end = if rate_slope == 0.0 {
                    f32::MAX
                } else {
                    rate_start + rate_slope * self.circular_buffer[idx].time_corrected()
                };
                if rate_end < self.circular_buffer[idx].volumetric_extrusion_rate_end {
                    // Limit the volumetric extrusion rate at the start of this segment due to a segment
                    // of ExtrusionType iRole, which was extruded before.
                    self.circular_buffer[idx].volumetric_extrusion_rate_end = rate_end;
                    self.circular_buffer[idx].max_volumetric_extrusion_rate_slope_positive =
                        rate_slope;
                    self.circular_buffer[idx].modified = true;
                }
                feedrate_per_extrusion_role[i_role] = if i_role == line_role {
                    self.circular_buffer[idx].volumetric_extrusion_rate_end
                } else {
                    rate_end
                };
            }
        }
    }

    // PressureEqualizer.cpp:569-576 — void push_axis_to_output(const char axis, const float value, bool add_eol)
    fn push_axis_to_output(&mut self, axis: char, value: f32, add_eol: bool) {
        // sprintf(buf, (axis == 'E') ? " %c%.3f" : " %c%.5f", axis, value);
        let s = if axis == 'E' {
            format!(" {}{:.3}", axis, value)
        } else {
            format!(" {}{:.5}", axis, value)
        };
        let bytes = s.into_bytes();
        let len = bytes.len();
        self.push_to_output(&bytes, len, add_eol);
    }

    // PressureEqualizer.cpp:578-608 — void push_to_output(const char *text, const size_t len, bool add_eol)
    fn push_to_output(&mut self, text: &[u8], len: usize, add_eol: bool) {
        // New length of the output buffer content.
        let mut len_new = self.output_buffer_length + len + 1;
        if add_eol {
            len_new += 1;
        }

        // Resize the output buffer to a power of 2 higher than the required memory.
        if self.output_buffer.len() < len_new {
            // Compute the next highest power of 2 of 32-bit v
            // http://graphics.stanford.edu/~seander/bithacks.html
            let mut v = len_new as u32;
            v -= 1;
            v |= v >> 1;
            v |= v >> 2;
            v |= v >> 4;
            v |= v >> 8;
            v |= v >> 16;
            v += 1;
            self.output_buffer.resize(v as usize, 0);
        }

        // Copy the text to the output.
        if len != 0 {
            self.output_buffer[self.output_buffer_length..self.output_buffer_length + len]
                .copy_from_slice(&text[..len]);
            self.output_buffer_length += len;
        }
        if add_eol {
            self.output_buffer[self.output_buffer_length] = b'\n';
            self.output_buffer_length += 1;
        }
        self.output_buffer[self.output_buffer_length] = 0;
    }

    // PressureEqualizer.cpp:610-621 — void push_line_to_output(const GCodeLine &line, const float new_feedrate, const char *comment)
    fn push_line_to_output(&mut self, idx: usize, new_feedrate: f32, comment: Option<&[u8]>) {
        self.push_to_output(b"G1", 2, false);
        for i in 0..3i32 {
            if self.circular_buffer[idx].pos_provided[i as usize] {
                let v = self.circular_buffer[idx].pos_end[i as usize];
                self.push_axis_to_output(
                    char::from(b'X' + (i as u8)),
                    v,
                    false,
                );
            }
        }
        let e_value = if self.m_config.use_relative_e_distances {
            self.circular_buffer[idx].pos_end[3] - self.circular_buffer[idx].pos_start[3]
        } else {
            self.circular_buffer[idx].pos_end[3]
        };
        self.push_axis_to_output('E', e_value, false);
        // if (line.pos_provided[4] || fabs(line.feedrate() - new_feedrate) > 1e-5)
        self.push_axis_to_output('F', new_feedrate, false);
        // output comment and EOL
        match comment {
            Some(c) => {
                let len = c.len();
                let owned = c.to_vec();
                self.push_to_output(&owned, len, true);
            }
            None => {
                self.push_to_output(&[], 0, true);
            }
        }
    }

    // PressureEqualizer.hpp:187-192 — size_t circular_buffer_idx_head() const
    fn circular_buffer_idx_head(&self) -> usize {
        let mut idx = self.circular_buffer_pos + self.circular_buffer_size - self.circular_buffer_items;
        if idx >= self.circular_buffer_size {
            idx -= self.circular_buffer_size;
        }
        idx
    }

    // PressureEqualizer.hpp:194 — size_t circular_buffer_idx_tail() const
    fn circular_buffer_idx_tail(&self) -> usize {
        self.circular_buffer_pos
    }

    // PressureEqualizer.hpp:196-201 — size_t circular_buffer_idx_prev(size_t idx) const
    fn circular_buffer_idx_prev(&self, mut idx: usize) -> usize {
        idx += self.circular_buffer_size - 1;
        if idx >= self.circular_buffer_size {
            idx -= self.circular_buffer_size;
        }
        idx
    }

    // PressureEqualizer.hpp:203-207 — size_t circular_buffer_idx_next(size_t idx) const
    fn circular_buffer_idx_next(&self, mut idx: usize) -> usize {
        idx += 1;
        if idx >= self.circular_buffer_size {
            idx -= self.circular_buffer_size;
        }
        idx
    }
}

// PressureEqualizer.cpp:132 — static inline bool is_ws(const char c)
fn is_ws(c: u8) -> bool {
    c == b' ' || c == b'\t'
}
// PressureEqualizer.cpp:134 — static inline bool is_eol(const char c)
fn is_eol(c: u8) -> bool {
    c == 0 || c == b'\r' || c == b'\n' || c == b';'
}
// PressureEqualizer.cpp:136 — static inline bool is_ws_or_eol(const char c)
fn is_ws_or_eol(c: u8) -> bool {
    is_ws(c) || is_eol(c)
}

// Read the byte at `cur`, or NUL if past the end (the C++ line is NUL-terminated).
fn char_at(line: &[u8], cur: usize) -> u8 {
    if cur < line.len() {
        line[cur]
    } else {
        0
    }
}

// PressureEqualizer.cpp:139-143 — static void eatws(const char *&line)
fn eatws(line: &[u8], cur: &mut usize) {
    while is_ws(char_at(line, *cur)) {
        *cur += 1;
    }
}

// PressureEqualizer.cpp:147-155 — static inline int parse_int(const char *&line)
//
// `strtol(line, &endptr, 10)` consumes an optional sign and a run of base-10
// digits (skipping leading whitespace). The resulting `endptr` must point at a
// whitespace or end-of-line, otherwise the C++ throws.
fn parse_int(line: &[u8], cur: &mut usize) -> i32 {
    let (result, endptr) = strtol(line, *cur);
    // `endptr == NULL` cannot happen with strtol; only validate the trailing char.
    if !is_ws_or_eol(char_at(line, endptr)) {
        panic!("PressureEqualizer: Error parsing an int");
    }
    *cur = endptr;
    result as i32
}

// PressureEqualizer.cpp:159-167 — static inline float parse_float(const char *&line)
fn parse_float(line: &[u8], cur: &mut usize) -> f32 {
    // float result = string_to_double_decimal_point(line, &endptr);
    let tail = match std::str::from_utf8(&line[*cur..]) {
        Ok(s) => s,
        Err(_) => panic!("PressureEqualizer: Error parsing a float"),
    };
    let (result, consumed) = string_to_double_decimal_point(tail);
    let endptr = *cur + consumed;
    if !is_ws_or_eol(char_at(line, endptr)) {
        panic!("PressureEqualizer: Error parsing a float");
    }
    *cur = endptr;
    result as f32
}

// `strtol(base 10)` semantics: skip leading whitespace, parse optional sign and
// digits, return the value and the index just past the consumed characters.
fn strtol(line: &[u8], start: usize) -> (i64, usize) {
    let mut i = start;
    // strtol skips leading whitespace (isspace), but PressureEqualizer always
    // calls it right after the command letter so the prefix is digits/sign.
    while i < line.len() && (line[i] == b' ' || line[i] == b'\t') {
        i += 1;
    }
    let begin_digits;
    let mut neg = false;
    if i < line.len() && (line[i] == b'+' || line[i] == b'-') {
        neg = line[i] == b'-';
        i += 1;
    }
    begin_digits = i;
    let mut value: i64 = 0;
    while i < line.len() && line[i].is_ascii_digit() {
        value = value * 10 + (line[i] - b'0') as i64;
        i += 1;
    }
    if i == begin_digits {
        // No digits consumed: strtol returns 0 and endptr == original line.
        return (0, start);
    }
    (if neg { -value } else { value }, i)
}

// `atoi(line)` — parse a leading int, ignoring any trailing characters.
// PressureEqualizer.cpp:175 — int role = atoi(line);
fn atoi(line: &[u8]) -> i32 {
    strtol(line, 0).0 as i32
}

// Map the raw integer parsed from `;_EXTRUSION_ROLE:N` (or the `T` index) to the
// C++ `Slic3r::ExtrusionRole`, mirroring `ExtrusionRole(role)`.
// PressureEqualizer.cpp:176
fn extrusion_role_from_int(role: i32) -> ExtrusionRole {
    match role {
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
        21 => ExtrusionRole::Mixed,
        // Out-of-range values would be UB in C++; clamp to None defensively.
        _ => ExtrusionRole::None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_num_extrusion_roles() {
        // numExtrusionRoles = erSupportMaterialInterface + 1
        assert_eq!(NUM_EXTRUSION_ROLES, 16);
        assert_eq!(ExtrusionRole::SupportMaterialInterface as usize, 15);
    }

    #[test]
    fn test_bridge_gapfill_indices() {
        assert_eq!(ExtrusionRole::BridgeInfill as usize, 10);
        assert_eq!(ExtrusionRole::GapFill as usize, 11);
    }

    #[test]
    fn test_filament_crossection() {
        // a = 0.25 * PI * d^2 with d = 1.75
        let eq = PressureEqualizer::new(PressureEqualizerConfig::new(1.75));
        let expected = (0.25f64 * std::f64::consts::PI * 1.75 * 1.75) as f32;
        assert!((eq.m_filament_crossections[0] - expected).abs() < 1e-6);
    }

    #[test]
    fn test_slope_conversion_and_unlimited_roles() {
        let eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        // 1.8 * 60 * 60 = 6480
        assert!(
            (eq.m_max_volumetric_extrusion_rate_slopes[ExtrusionRole::Perimeter as usize].positive
                - 6480.0)
                .abs()
                < 1e-3
        );
        assert_eq!(
            eq.m_max_volumetric_extrusion_rate_slopes[ExtrusionRole::BridgeInfill as usize].positive,
            0.0
        );
        assert_eq!(
            eq.m_max_volumetric_extrusion_rate_slopes[ExtrusionRole::GapFill as usize].negative,
            0.0
        );
    }

    #[test]
    fn test_process_empty() {
        let mut eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        let result = eq.process("", true);
        assert!(result.is_empty());
    }

    #[test]
    fn test_process_simple_move() {
        let mut eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        let result = eq.process("G1 X10 Y10 F1000\n", true);
        assert!(result.contains("G1"));
    }

    #[test]
    fn test_process_comment_kept() {
        let mut eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        let result = eq.process("; This is a comment\n", true);
        assert!(result.contains("comment"));
    }

    #[test]
    fn test_process_role_marker_filtered() {
        let mut eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        let result = eq.process(";_EXTRUSION_ROLE:1\nG1 X10 Y10 E1 F1000\n", true);
        assert!(!result.contains("_EXTRUSION_ROLE"));
    }

    #[test]
    fn test_process_g92() {
        let mut eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        let result = eq.process("G92 X0 Y0 Z0 E0\nG1 X10 Y10 E1 F1000\n", true);
        assert!(result.contains("G92"));
    }

    #[test]
    fn test_firmware_retract() {
        let mut eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        let result = eq.process("G10\nG11\n", true);
        assert!(result.contains("G10"));
        assert!(result.contains("G11"));
    }

    #[test]
    fn test_tool_change() {
        let mut eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        let _ = eq.process("T0\nT1\nT1\n", true);
    }

    #[test]
    fn test_idx_helpers() {
        let eq = PressureEqualizer::new(PressureEqualizerConfig::default());
        assert_eq!(eq.circular_buffer_idx_next(0), 1);
        assert_eq!(eq.circular_buffer_idx_next(99), 0);
        assert_eq!(eq.circular_buffer_idx_prev(0), 99);
        assert_eq!(eq.circular_buffer_idx_prev(5), 4);
    }

    #[test]
    fn test_extrusion_role_from_int() {
        assert_eq!(extrusion_role_from_int(0), ExtrusionRole::None);
        assert_eq!(extrusion_role_from_int(1), ExtrusionRole::Perimeter);
        assert_eq!(extrusion_role_from_int(2), ExtrusionRole::ExternalPerimeter);
        assert_eq!(extrusion_role_from_int(10), ExtrusionRole::BridgeInfill);
        assert_eq!(extrusion_role_from_int(11), ExtrusionRole::GapFill);
    }

    #[test]
    fn test_parse_helpers() {
        let line = b"42 ";
        let mut cur = 0usize;
        assert_eq!(parse_int(line, &mut cur), 42);
        let line = b"-3.5\n";
        let mut cur = 0usize;
        let f = parse_float(line, &mut cur);
        assert!((f - (-3.5)).abs() < 1e-6);
    }
}
