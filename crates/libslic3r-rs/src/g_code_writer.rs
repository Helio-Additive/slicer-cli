//! G-code writer.
//!
//! 1:1 line-by-line port of `GCodeWriter.cpp` / `GCodeWriter.hpp`
//! (BambuStudio src/libslic3r). `coord_t` -> `i64`, `coordf_t` -> `f64`.
//! This file faithfully mirrors the control flow, constants, rounding and edge
//! cases of the C++ source. wasm-safe: no system/dylib dependencies.
//!
//! Dependency note: the C++ `GCodeWriter::config` is a full `GCodeConfig`
//! (a huge `ConfigOption`-based config class). The Rust crate models the subset
//! of `GCodeConfig` the writer + `Extruder` need in `crate::extruder::GCodeConfig`
//! with `ConfigOptionVector`-faithful `get_at` semantics. `apply_print_config`'s
//! `this->config.apply(print_config, true)` requires the not-yet-ported
//! `ConfigBase` merge machinery; see the note on `apply_print_config` below.

// GCodeWriter.cpp:1   #include "GCodeWriter.hpp"
// GCodeWriter.cpp:2   #include "CustomGCode.hpp"
use crate::extruder::{get_process_config_idx, Extruder, GCodeConfig};
use crate::geometry::{Vec2d, Vec3d};
use crate::print_config::GCodeFlavor;
use crate::libslic3r::EPSILON;
use std::f64::consts::PI;
use std::fmt::Write as _;

// GCodeWriter.cpp:13   #define FLAVOR_IS(val) this->config.gcode_flavor == val
// GCodeWriter.cpp:14   #define FLAVOR_IS_NOT(val) this->config.gcode_flavor != val
// Implemented inline as `self.config.gcode_flavor == GCodeFlavor::...`.

// GCodeWriter.hpp:15   enum class LiftType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiftType {
    // GCodeWriter.hpp:16
    NormalLift,
    // GCodeWriter.hpp:17
    SlopeLift,
    // GCodeWriter.hpp:18
    SpiralLift,
}

// GCodeWriter.hpp:21   class GCodeWriter
pub struct GCodeWriter {
    // GCodeWriter.hpp:23   GCodeConfig config;
    pub config: GCodeConfig,
    // GCodeWriter.hpp:24   bool multiple_extruders;
    pub multiple_extruders: bool,

    // GCodeWriter.hpp:141  std::vector<Extruder> m_filament_extruders;
    // Extruders are sorted by their ID, so that binary search is possible.
    m_filament_extruders: Vec<Extruder>,
    // GCodeWriter.hpp:142  bool m_single_extruder_multi_material;
    m_single_extruder_multi_material: bool,
    // GCodeWriter.hpp:143  std::vector<Extruder*> m_curr_filament_extruder;
    // C++ holds raw pointers into m_filament_extruders; we hold indices into it.
    m_curr_filament_extruder: [Option<usize>; 2],
    // GCodeWriter.hpp:144  int m_curr_extruder_id;
    m_curr_extruder_id: i32,
    // GCodeWriter.hpp:145  unsigned int m_last_acceleration;
    m_last_acceleration: u32,
    // GCodeWriter.hpp:148  unsigned int m_max_acceleration;
    m_max_acceleration: u32,
    // GCodeWriter.hpp:149  double m_last_jerk;
    m_last_jerk: f64,
    // GCodeWriter.hpp:150  double m_max_jerk;
    m_max_jerk: f64,
    // GCodeWriter.hpp:152  unsigned int m_last_additional_fan_speed;
    #[allow(dead_code)]
    m_last_additional_fan_speed: u32,
    // GCodeWriter.hpp:153  int m_last_bed_temperature;
    m_last_bed_temperature: i32,
    // GCodeWriter.hpp:154  bool m_last_bed_temperature_reached;
    m_last_bed_temperature_reached: bool,
    // GCodeWriter.hpp:155  double m_lifted;
    m_lifted: f64,
    // GCodeWriter.hpp:158  double m_to_lift;
    m_to_lift: f64,
    // GCodeWriter.hpp:159  LiftType m_to_lift_type;
    m_to_lift_type: LiftType,
    // GCodeWriter.hpp:160  Vec3d m_pos = Vec3d::Zero();
    m_pos: Vec3d,
    // GCodeWriter.hpp:164  bool m_is_current_pos_clear = false;
    m_is_current_pos_clear: bool,
    // GCodeWriter.hpp:166  double m_x_offset{ 0 };
    m_x_offset: f64,
    // GCodeWriter.hpp:167  double m_y_offset{ 0 };
    m_y_offset: f64,
    // GCodeWriter.hpp:168  double m_current_speed{ 0 };
    m_current_speed: f64,
    // GCodeWriter.hpp:169  bool m_is_bbl_printer = false;
    m_is_bbl_printer: bool,
    // GCodeWriter.hpp:171  std::string m_gcode_label_objects_start;
    m_gcode_label_objects_start: String,
    // GCodeWriter.hpp:172  std::string m_gcode_label_objects_end;
    m_gcode_label_objects_end: String,
    // GCodeWriter.hpp:174  bool m_is_first_layer{false};
    m_is_first_layer: bool,
    // GCodeWriter.hpp:175  unsigned int m_acceleration{0};
    m_acceleration: u32,
    // GCodeWriter.hpp:176  std::vector<unsigned int> m_travel_accelerations;
    m_travel_accelerations: Vec<u32>,
    // GCodeWriter.hpp:177  std::vector<unsigned int> m_travel_short_accelerations;
    m_travel_short_accelerations: Vec<u32>,
    // GCodeWriter.hpp:178  std::vector<unsigned int> m_first_layer_travel_accelerations;
    m_first_layer_travel_accelerations: Vec<u32>,
}

// GCodeWriter.hpp:129  static const bool full_gcode_comment;
// GCodeWriter.cpp:18    const bool GCodeWriter::full_gcode_comment = false;
pub const FULL_GCODE_COMMENT: bool = false;

// GCodeWriter.hpp:131  static const double slope_threshold;
// GCodeWriter.cpp:19    const double GCodeWriter::slope_threshold = 3 * PI / 180;
pub const SLOPE_THRESHOLD: f64 = 3.0 * PI / 180.0;

impl Default for GCodeWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl GCodeWriter {
    // GCodeWriter.hpp:26-36  GCodeWriter() :
    pub fn new() -> Self {
        GCodeWriter {
            config: GCodeConfig::default(),
            // GCodeWriter.hpp:27  multiple_extruders(false)
            multiple_extruders: false,
            m_filament_extruders: Vec::new(),
            // GCodeWriter.hpp:29  m_single_extruder_multi_material(false)
            m_single_extruder_multi_material: false,
            // GCodeWriter.hpp:27  m_curr_filament_extruder{ nullptr,nullptr }
            m_curr_filament_extruder: [None, None],
            // GCodeWriter.hpp:28  m_curr_extruder_id (-1)
            m_curr_extruder_id: -1,
            // GCodeWriter.hpp:30  m_last_acceleration(0)
            m_last_acceleration: 0,
            // GCodeWriter.hpp:30  m_max_acceleration(0)
            m_max_acceleration: 0,
            // GCodeWriter.hpp:31  m_last_jerk(0)
            m_last_jerk: 0.0,
            // GCodeWriter.hpp:31  m_max_jerk(0)
            m_max_jerk: 0.0,
            m_last_additional_fan_speed: 0,
            m_last_bed_temperature: 0,
            // GCodeWriter.hpp:32  m_last_bed_temperature_reached(true)
            m_last_bed_temperature_reached: true,
            // GCodeWriter.hpp:33  m_lifted(0)
            m_lifted: 0.0,
            // GCodeWriter.hpp:34  m_to_lift(0)
            m_to_lift: 0.0,
            // GCodeWriter.hpp:35  m_to_lift_type(LiftType::NormalLift)
            m_to_lift_type: LiftType::NormalLift,
            // GCodeWriter.hpp:160  m_pos = Vec3d::Zero()
            m_pos: Vec3d::zero(),
            // GCodeWriter.hpp:164  m_is_current_pos_clear = false
            m_is_current_pos_clear: false,
            // GCodeWriter.hpp:166-167  m_x_offset{0}; m_y_offset{0}
            m_x_offset: 0.0,
            m_y_offset: 0.0,
            // GCodeWriter.hpp:168  m_current_speed{0}
            m_current_speed: 0.0,
            // GCodeWriter.hpp:169  m_is_bbl_printer = false
            m_is_bbl_printer: false,
            m_gcode_label_objects_start: String::new(),
            m_gcode_label_objects_end: String::new(),
            // GCodeWriter.hpp:174  m_is_first_layer{false}
            m_is_first_layer: false,
            // GCodeWriter.hpp:175  m_acceleration{0}
            m_acceleration: 0,
            m_travel_accelerations: Vec::new(),
            m_travel_short_accelerations: Vec::new(),
            m_first_layer_travel_accelerations: Vec::new(),
        }
    }

    // GCodeWriter.hpp:37-38  Extruder* filament(size_t extruder_id)
    pub fn filament_by_id(&self, extruder_id: usize) -> Option<&Extruder> {
        // GCodeWriter.hpp:37  assert(extruder_id < m_curr_filament_extruder.size());
        debug_assert!(extruder_id < self.m_curr_filament_extruder.len());
        self.m_curr_filament_extruder[extruder_id].map(|i| &self.m_filament_extruders[i])
    }

    // GCodeWriter.hpp:39-40  Extruder* filament()
    pub fn filament(&self) -> Option<&Extruder> {
        // GCodeWriter.hpp:39  if (m_curr_extruder_id == -1) return nullptr;
        if self.m_curr_extruder_id == -1 {
            return None;
        }
        self.m_curr_filament_extruder[self.m_curr_extruder_id as usize]
            .map(|i| &self.m_filament_extruders[i])
    }

    // Mutable variant of filament(); used where the C++ calls filament()->extrude()/retract().
    fn filament_mut(&mut self) -> Option<&mut Extruder> {
        if self.m_curr_extruder_id == -1 {
            return None;
        }
        let idx = self.m_curr_filament_extruder[self.m_curr_extruder_id as usize]?;
        Some(&mut self.m_filament_extruders[idx])
    }

    // GCodeWriter.hpp:42  int get_curr_extruder_id() const
    pub fn get_curr_extruder_id(&self) -> i32 {
        self.m_curr_extruder_id
    }

    // GCodeWriter.cpp:21  void GCodeWriter::apply_print_config(const PrintConfig &print_config)
    //
    // BLOCKED: the C++ first does `this->config.apply(print_config, true)` (a full
    // ConfigBase option-merge that copies every overlapping option from PrintConfig
    // into GCodeConfig). That merge machinery (ConfigBase::apply, t_config_option_keys)
    // is not yet ported, and the Rust `print_config::PrintConfig` is a divergent
    // simplified type whose option layout differs from GCodeConfig. The remaining
    // lines (single_extruder_multi_material / max accel / max jerk derivation) read
    // `.values.front()` of vector options that the Rust `print_config::PrintConfig`
    // does not expose as ConfigOptionVectors. This method is therefore left as a
    // documented stub-free port surface: callers must populate `self.config`
    // directly until ConfigBase::apply is ported. See PORT_LEDGER status="partial".

    // GCodeWriter.cpp:32  void GCodeWriter::set_extruders(std::vector<unsigned int> extruder_ids)
    pub fn set_extruders(&mut self, mut extruder_ids: Vec<u32>) {
        // GCodeWriter.cpp:34  std::sort(extruder_ids.begin(), extruder_ids.end());
        extruder_ids.sort();
        // GCodeWriter.cpp:35  m_filament_extruders.clear();
        self.m_filament_extruders.clear();
        // GCodeWriter.cpp:36  m_filament_extruders.reserve(extruder_ids.size());
        self.m_filament_extruders.reserve(extruder_ids.len());
        // GCodeWriter.cpp:37-38  for (unsigned int extruder_id : extruder_ids)
        //     m_filament_extruders.emplace_back(Extruder(extruder_id, &this->config, config.single_extruder_multi_material.value));
        for extruder_id in &extruder_ids {
            let e = Extruder::new(
                *extruder_id,
                &self.config as *const GCodeConfig,
                self.m_single_extruder_multi_material,
            );
            self.m_filament_extruders.push(e);
        }

        // GCodeWriter.cpp:40-42  we enable support for multiple extruder if any extruder
        //     greater than 0 is used [...]
        // GCodeWriter.cpp:43  this->multiple_extruders = (*std::max_element(...)) > 0;
        self.multiple_extruders = *extruder_ids.iter().max().unwrap() > 0;
    }

    // GCodeWriter.hpp:47  const std::vector<Extruder>& extruders() const
    pub fn extruders(&self) -> &Vec<Extruder> {
        &self.m_filament_extruders
    }

    // GCodeWriter.hpp:48-54  std::vector<unsigned int> extruder_ids() const
    pub fn extruder_ids(&self) -> Vec<u32> {
        // GCodeWriter.hpp:49
        let mut out: Vec<u32> = Vec::new();
        // GCodeWriter.hpp:50
        out.reserve(self.m_filament_extruders.len());
        // GCodeWriter.hpp:51-52
        for e in &self.m_filament_extruders {
            out.push(e.id());
        }
        // GCodeWriter.hpp:53
        out
    }

    // GCodeWriter.cpp:46  std::string GCodeWriter::preamble()
    pub fn preamble(&mut self) -> String {
        // GCodeWriter.cpp:48
        let mut gcode = String::new();

        // GCodeWriter.cpp:50  if (FLAVOR_IS_NOT(gcfMakerWare))
        if self.config.gcode_flavor != GCodeFlavor::MakerWare {
            // GCodeWriter.cpp:51
            gcode.push_str("G90\n");
            // GCodeWriter.cpp:52
            gcode.push_str("G21\n");
        }
        // GCodeWriter.cpp:54-61
        if self.config.gcode_flavor == GCodeFlavor::RepRapSprinter
            || self.config.gcode_flavor == GCodeFlavor::RepRapFirmware
            || self.config.gcode_flavor == GCodeFlavor::MarlinLegacy
            || self.config.gcode_flavor == GCodeFlavor::Marlin
            || self.config.gcode_flavor == GCodeFlavor::Teacup
            || self.config.gcode_flavor == GCodeFlavor::Repetier
            || self.config.gcode_flavor == GCodeFlavor::Smoothie
            || self.config.gcode_flavor == GCodeFlavor::Klipper
        {
            // GCodeWriter.cpp:63
            if self.config.use_relative_e_distances {
                // GCodeWriter.cpp:64
                gcode.push_str("M83 ; use relative distances for extrusion\n");
            } else {
                // GCodeWriter.cpp:66
                gcode.push_str("M82 ; use absolute distances for extrusion\n");
            }
            // GCodeWriter.cpp:68
            gcode.push_str(&self.reset_e(true));
        }

        // GCodeWriter.cpp:71
        gcode
    }

    // GCodeWriter.cpp:74  std::string GCodeWriter::postamble() const
    pub fn postamble(&self) -> String {
        // GCodeWriter.cpp:76
        let mut gcode = String::new();
        // GCodeWriter.cpp:77  if (FLAVOR_IS(gcfMachinekit))
        if self.config.gcode_flavor == GCodeFlavor::Machinekit {
            // GCodeWriter.cpp:78
            gcode.push_str("M2 ; end of program\n");
        }
        // GCodeWriter.cpp:79
        gcode
    }

    // GCodeWriter.cpp:82  std::string GCodeWriter::set_temperature(unsigned int temperature, bool wait, int tool) const
    pub fn set_temperature(&self, temperature: u32, wait: bool, tool: i32) -> String {
        // GCodeWriter.cpp:84  if (wait && (FLAVOR_IS(gcfMakerWare) || FLAVOR_IS(gcfSailfish)))
        if wait
            && (self.config.gcode_flavor == GCodeFlavor::MakerWare
                || self.config.gcode_flavor == GCodeFlavor::Sailfish)
        {
            // GCodeWriter.cpp:85
            return String::new();
        }

        // GCodeWriter.cpp:87  std::string code, comment;
        let code: &str;
        let comment: &str;
        // GCodeWriter.cpp:88  if (wait && FLAVOR_IS_NOT(gcfTeacup) && FLAVOR_IS_NOT(gcfRepRapFirmware))
        if wait
            && self.config.gcode_flavor != GCodeFlavor::Teacup
            && self.config.gcode_flavor != GCodeFlavor::RepRapFirmware
        {
            // GCodeWriter.cpp:89-90
            code = "M109";
            comment = "set nozzle temperature and wait for it to be reached";
        } else {
            // GCodeWriter.cpp:92  if (FLAVOR_IS(gcfRepRapFirmware)) // M104 is deprecated
            if self.config.gcode_flavor == GCodeFlavor::RepRapFirmware {
                // GCodeWriter.cpp:93
                code = "G10";
            } else {
                // GCodeWriter.cpp:95
                code = "M104";
            }
            // GCodeWriter.cpp:97
            comment = "set nozzle temperature";
        }

        // GCodeWriter.cpp:100  std::ostringstream gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:101  gcode << code << " ";
        let _ = write!(gcode, "{} ", code);
        // GCodeWriter.cpp:102  if (FLAVOR_IS(gcfMach3) || FLAVOR_IS(gcfMachinekit))
        if self.config.gcode_flavor == GCodeFlavor::Mach3
            || self.config.gcode_flavor == GCodeFlavor::Machinekit
        {
            // GCodeWriter.cpp:103
            gcode.push('P');
        } else {
            // GCodeWriter.cpp:105
            gcode.push('S');
        }
        // GCodeWriter.cpp:107  gcode << temperature;
        let _ = write!(gcode, "{}", temperature);
        // GCodeWriter.cpp:108  bool multiple_tools = this->multiple_extruders && ! m_single_extruder_multi_material;
        let multiple_tools = self.multiple_extruders && !self.m_single_extruder_multi_material;
        // GCodeWriter.cpp:109  if (tool != -1 && (multiple_tools || FLAVOR_IS(gcfMakerWare) || FLAVOR_IS(gcfSailfish)))
        if tool != -1
            && (multiple_tools
                || self.config.gcode_flavor == GCodeFlavor::MakerWare
                || self.config.gcode_flavor == GCodeFlavor::Sailfish)
        {
            // GCodeWriter.cpp:110  if (FLAVOR_IS(gcfRepRapFirmware))
            if self.config.gcode_flavor == GCodeFlavor::RepRapFirmware {
                // GCodeWriter.cpp:111  gcode << " P" << tool;
                let _ = write!(gcode, " P{}", tool);
            } else {
                // GCodeWriter.cpp:113  gcode << " T" << tool;
                let _ = write!(gcode, " T{}", tool);
            }
        }
        // GCodeWriter.cpp:116  gcode << " ; " << comment << "\n";
        let _ = write!(gcode, " ; {}\n", comment);

        // GCodeWriter.cpp:118  if ((FLAVOR_IS(gcfTeacup) || FLAVOR_IS(gcfRepRapFirmware)) && wait)
        if (self.config.gcode_flavor == GCodeFlavor::Teacup
            || self.config.gcode_flavor == GCodeFlavor::RepRapFirmware)
            && wait
        {
            // GCodeWriter.cpp:119
            gcode.push_str("M116 ; wait for temperature to be reached\n");
        }

        // GCodeWriter.cpp:121
        gcode
    }

    // GCodeWriter.cpp:125  std::string GCodeWriter::set_bed_temperature(int temperature, bool wait)
    pub fn set_bed_temperature(&mut self, temperature: i32, wait: bool) -> String {
        // GCodeWriter.cpp:127  if (temperature == m_last_bed_temperature && (! wait || m_last_bed_temperature_reached))
        if temperature == self.m_last_bed_temperature
            && (!wait || self.m_last_bed_temperature_reached)
        {
            // GCodeWriter.cpp:128
            return String::new();
        }

        // GCodeWriter.cpp:130
        self.m_last_bed_temperature = temperature;
        // GCodeWriter.cpp:131
        self.m_last_bed_temperature_reached = wait;

        // GCodeWriter.cpp:133  std::string code, comment;
        let code: &str;
        let comment: &str;
        // GCodeWriter.cpp:134  std::ostringstream gcode;
        let mut gcode = String::new();

        // GCodeWriter.cpp:136
        if wait {
            // GCodeWriter.cpp:137-138
            code = "M190";
            comment = "set bed temperature and wait for it to be reached";
        } else {
            // GCodeWriter.cpp:141-142
            code = "M140";
            comment = "set bed temperature";
        }

        // GCodeWriter.cpp:145  gcode << code << " S" << temperature << " ; " << comment << "\n";
        let _ = write!(gcode, "{} S{} ; {}\n", code, temperature, comment);
        // GCodeWriter.cpp:146
        gcode
    }

    // GCodeWriter.cpp:149  std::string GCodeWriter::set_chamber_temperature(int temperature, bool wait)
    pub fn set_chamber_temperature(&mut self, temperature: i32, wait: bool) -> String {
        // GCodeWriter.cpp:151  std::string code, comment;
        let code: &str;
        let comment: &str;
        // GCodeWriter.cpp:152  std::ostringstream gcode;
        let mut gcode = String::new();

        // GCodeWriter.cpp:154
        if wait {
            // GCodeWriter.cpp:156
            gcode.push_str("M106 P2 S255 \n");
            // GCodeWriter.cpp:157
            let _ = write!(
                gcode,
                "M191 S{} ;set chamber_temperature and wait for it to be reached\n",
                temperature
            );
            // GCodeWriter.cpp:158
            gcode.push_str("M106 P2 S0 \n");
        } else {
            // GCodeWriter.cpp:161-162
            code = "M141";
            comment = "set chamber_temperature";
            // GCodeWriter.cpp:163  gcode << code << " S" << temperature << ";" << comment << "\n";
            let _ = write!(gcode, "{} S{};{}\n", code, temperature, comment);
        }
        // GCodeWriter.cpp:165
        gcode
    }

    // GCodeWriter.cpp:168  void GCodeWriter::set_acceleration(unsigned int acceleration)
    pub fn set_acceleration(&mut self, acceleration: u32) {
        // GCodeWriter.cpp:170
        self.m_acceleration = acceleration;
    }

    // GCodeWriter.cpp:173  void GCodeWriter::set_travel_acceleration(const std::vector<unsigned int>& accelerations)
    pub fn set_travel_acceleration_vec(&mut self, accelerations: &[u32]) {
        // GCodeWriter.cpp:175
        self.m_travel_accelerations = accelerations.to_vec();
    }

    // GCodeWriter.cpp:178  void GCodeWriter::set_travel_short_acceleration(const std::vector<unsigned int>& accelerations)
    pub fn set_travel_short_acceleration(&mut self, accelerations: &[u32]) {
        // GCodeWriter.cpp:180
        self.m_travel_short_accelerations = accelerations.to_vec();
    }

    // GCodeWriter.cpp:183  void GCodeWriter::reset_last_acceleration()
    pub fn reset_last_acceleration(&mut self) {
        // GCodeWriter.cpp:185
        self.m_last_acceleration = 0;
    }

    // GCodeWriter.hpp:64  std::vector<unsigned int>& get_travel_acceleration()
    pub fn get_travel_acceleration(&mut self) -> &mut Vec<u32> {
        &mut self.m_travel_accelerations
    }

    // GCodeWriter.hpp:65  std::vector<unsigned int>& get_travel_short_acceleration()
    pub fn get_travel_short_acceleration(&mut self) -> &mut Vec<u32> {
        &mut self.m_travel_short_accelerations
    }

    // GCodeWriter.cpp:188  void GCodeWriter::set_first_layer_travel_acceleration(const std::vector<unsigned int> &travel_accelerations)
    pub fn set_first_layer_travel_acceleration(&mut self, travel_accelerations: &[u32]) {
        // GCodeWriter.cpp:190
        self.m_first_layer_travel_accelerations = travel_accelerations.to_vec();
    }

    // GCodeWriter.cpp:193  void GCodeWriter::set_first_layer(bool is_first_layer)
    pub fn set_first_layer(&mut self, is_first_layer: bool) {
        // GCodeWriter.cpp:195
        self.m_is_first_layer = is_first_layer;
    }

    // GCodeWriter.cpp:198  std::string GCodeWriter::set_extrude_acceleration()
    fn set_extrude_acceleration(&mut self) -> String {
        // GCodeWriter.cpp:200
        self.set_acceleration_impl(self.m_acceleration)
    }

    // GCodeWriter.cpp:203  std::string GCodeWriter::set_travel_acceleration()
    fn set_travel_acceleration(&mut self) -> String {
        // GCodeWriter.cpp:205
        self.set_travel_acceleration_impl(false)
    }

    // GCodeWriter.cpp:208  std::string GCodeWriter::set_travel_acceleration(bool use_short_travel_acceleration)
    fn set_travel_acceleration_impl(&mut self, use_short_travel_acceleration: bool) -> String {
        // GCodeWriter.cpp:210  std::vector<unsigned int> travel_accelerations = m_is_first_layer ? m_first_layer_travel_accelerations : m_travel_accelerations;
        let travel_accelerations = if self.m_is_first_layer {
            self.m_first_layer_travel_accelerations.clone()
        } else {
            self.m_travel_accelerations.clone()
        };
        // GCodeWriter.cpp:211  if (travel_accelerations.empty())
        if travel_accelerations.is_empty() {
            // GCodeWriter.cpp:212
            return String::new();
        }

        // GCodeWriter.cpp:214  Extruder *cur_filament = filament();
        // GCodeWriter.cpp:215  if (!cur_filament)
        let extruder_id = match self.filament() {
            None => return String::new(), // GCodeWriter.cpp:216
            // GCodeWriter.cpp:218  unsigned int extruder_id = cur_filament->extruder_id();
            Some(cur_filament) => cur_filament.extruder_id(),
        };

        // GCodeWriter.cpp:221-223  Use short travel acceleration if requested and available
        if use_short_travel_acceleration
            && (extruder_id as usize) < self.m_travel_short_accelerations.len()
            && self.m_travel_short_accelerations[extruder_id as usize] > 0
        {
            // GCodeWriter.cpp:224
            return self.set_acceleration_impl(self.m_travel_short_accelerations[extruder_id as usize]);
        }

        // GCodeWriter.cpp:227
        let accel = travel_accelerations[extruder_id as usize];
        self.set_acceleration_impl(accel)
    }

    // GCodeWriter.cpp:230  std::string GCodeWriter::set_acceleration_impl(unsigned int acceleration)
    fn set_acceleration_impl(&mut self, mut acceleration: u32) -> String {
        // GCodeWriter.cpp:232-234  Clamp the acceleration to the allowed maximum.
        if self.m_max_acceleration > 0 && acceleration > self.m_max_acceleration {
            acceleration = self.m_max_acceleration;
        }

        // GCodeWriter.cpp:236  if (acceleration == 0 || acceleration == m_last_acceleration)
        if acceleration == 0 || acceleration == self.m_last_acceleration {
            // GCodeWriter.cpp:237
            return String::new();
        }

        // GCodeWriter.cpp:239
        self.m_last_acceleration = acceleration;

        // GCodeWriter.cpp:241  std::ostringstream gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:242  if (FLAVOR_IS(gcfRepetier))
        if self.config.gcode_flavor == GCodeFlavor::Repetier {
            // GCodeWriter.cpp:244  M201: Set max printing acceleration
            let _ = write!(gcode, "M201 X{} Y{}", acceleration, acceleration);
            // GCodeWriter.cpp:246
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; adjust acceleration");
            }
            // GCodeWriter.cpp:247
            gcode.push('\n');
            // GCodeWriter.cpp:249  M202: Set max travel acceleration
            let _ = write!(gcode, "M202 X{} Y{}", acceleration, acceleration);
        } else if self.config.gcode_flavor == GCodeFlavor::RepRapFirmware {
            // GCodeWriter.cpp:252  M204: Set default acceleration
            let _ = write!(gcode, "M204 P{}", acceleration);
        } else if self.config.gcode_flavor == GCodeFlavor::Marlin {
            // GCodeWriter.cpp:253-256  new MarlinFirmware with separated print/retraction/travel acceleration.
            let _ = write!(gcode, "M204 P{}", acceleration);
        } else if self.config.gcode_flavor == GCodeFlavor::Klipper
            && self.config.accel_to_decel_enable
        {
            // GCodeWriter.cpp:258
            let _ = write!(
                gcode,
                "SET_VELOCITY_LIMIT ACCEL_TO_DECEL={}",
                acceleration as f64 * self.config.accel_to_decel_factor / 100.0
            );
            // GCodeWriter.cpp:259
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; adjust ACCEL_TO_DECEL");
            }
            // GCodeWriter.cpp:260
            let _ = write!(gcode, "\nM204 S{}", acceleration);
        } else {
            // GCodeWriter.cpp:264  M204: Set default acceleration
            let _ = write!(gcode, "M204 S{}", acceleration);
        }
        // GCodeWriter.cpp:267
        if FULL_GCODE_COMMENT {
            gcode.push_str(" ; adjust acceleration");
        }
        // GCodeWriter.cpp:268
        gcode.push('\n');

        // GCodeWriter.cpp:270
        gcode
    }

    // GCodeWriter.cpp:273  std::string GCodeWriter::set_pressure_advance(double pa, bool is_bbl_bowden) const
    pub fn set_pressure_advance(&self, pa: f64, is_bbl_bowden: bool) -> String {
        // GCodeWriter.cpp:275  std::ostringstream gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:276  if (pa < 0) return gcode.str();
        if pa < 0.0 {
            return gcode;
        }
        // GCodeWriter.cpp:277  if (false) { // todo: bbl printer
        #[allow(clippy::overly_complex_bool_expr)]
        if false {
            // GCodeWriter.cpp:279  OrcaSlicer: set L1000 to use linear model
            let _ = write!(
                gcode,
                "M400\n M900 K{} L1000 M10 ; Override pressure advance value\n",
                format_setprecision_4(pa)
            );
        } else {
            // GCodeWriter.cpp:281  if (this->config.gcode_flavor == gcfKlipper)
            if self.config.gcode_flavor == GCodeFlavor::Klipper {
                // GCodeWriter.cpp:282
                let _ = write!(
                    gcode,
                    "SET_PRESSURE_ADVANCE ADVANCE={}; Override pressure advance value\n",
                    format_setprecision_4(pa)
                );
            } else if self.config.gcode_flavor == GCodeFlavor::RepRapFirmware {
                // GCodeWriter.cpp:284
                let _ = write!(
                    gcode,
                    "M572 D0 S{}; Override pressure advance value\n",
                    format_setprecision_4(pa)
                );
            } else if is_bbl_bowden {
                // GCodeWriter.cpp:286
                let _ = write!(
                    gcode,
                    "M400\n M901 P0.75 K{}; Override pressure advance value\n",
                    format_setprecision_4(pa)
                );
            } else {
                // GCodeWriter.cpp:288
                let _ = write!(
                    gcode,
                    "M400\n M900 K{}; Override pressure advance value\n",
                    format_setprecision_4(pa)
                );
            }
        }
        // GCodeWriter.cpp:290
        gcode
    }

    // GCodeWriter.cpp:293  std::string GCodeWriter::set_jerk_xy(double jerk)
    pub fn set_jerk_xy(&mut self, mut jerk: f64) -> String {
        // GCodeWriter.cpp:295-296  Clamp the jerk to the allowed maximum.
        if self.m_max_jerk > 0.0 && jerk > self.m_max_jerk {
            jerk = self.m_max_jerk;
        }

        // GCodeWriter.cpp:298  if (jerk < 0.01 || is_approx(jerk, m_last_jerk)) return std::string();
        if jerk < 0.01 || crate::geometry::geometry::is_approx(jerk, self.m_last_jerk) {
            return String::new();
        }

        // GCodeWriter.cpp:300
        self.m_last_jerk = jerk;

        // GCodeWriter.cpp:302  std::ostringstream gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:303  if (FLAVOR_IS(gcfKlipper))
        if self.config.gcode_flavor == GCodeFlavor::Klipper {
            // GCodeWriter.cpp:304
            let _ = write!(gcode, "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY={}", ostream_double(jerk));
        } else {
            // GCodeWriter.cpp:306
            let _ = write!(gcode, "M205 X{} Y{}", ostream_double(jerk), ostream_double(jerk));
        }

        // GCodeWriter.cpp:308
        if FULL_GCODE_COMMENT {
            gcode.push_str(" ; adjust jerk");
        }
        // GCodeWriter.cpp:309
        gcode.push('\n');

        // GCodeWriter.cpp:311
        gcode
    }

    // GCodeWriter.cpp:314  std::string GCodeWriter::reset_e(bool force)
    pub fn reset_e(&mut self, force: bool) -> String {
        // GCodeWriter.cpp:316-318
        if self.config.gcode_flavor == GCodeFlavor::Mach3
            || self.config.gcode_flavor == GCodeFlavor::MakerWare
            || self.config.gcode_flavor == GCodeFlavor::Sailfish
        {
            // GCodeWriter.cpp:319
            return String::new();
        }

        // GCodeWriter.cpp:321  if (m_curr_extruder_id!=-1 && m_curr_filament_extruder[m_curr_extruder_id] != nullptr)
        if self.m_curr_extruder_id != -1
            && self.m_curr_filament_extruder[self.m_curr_extruder_id as usize].is_some()
        {
            let idx = self.m_curr_filament_extruder[self.m_curr_extruder_id as usize].unwrap();
            // GCodeWriter.cpp:322  if (m_curr_filament_extruder[m_curr_extruder_id]->E() == 0. && !force)
            if self.m_filament_extruders[idx].e() == 0.0 && !force {
                // GCodeWriter.cpp:323
                return String::new();
            }
            // GCodeWriter.cpp:324  m_curr_filament_extruder[m_curr_extruder_id]->reset_E();
            self.m_filament_extruders[idx].reset_e();
        }

        // GCodeWriter.cpp:327  if (!this->config.use_relative_e_distances)
        if !self.config.use_relative_e_distances {
            // GCodeWriter.cpp:328  std::ostringstream gcode;
            let mut gcode = String::new();
            // GCodeWriter.cpp:329
            gcode.push_str("G92 E0");
            // GCodeWriter.cpp:331
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; reset extrusion distance");
            }
            // GCodeWriter.cpp:332
            gcode.push('\n');
            // GCodeWriter.cpp:333
            gcode
        } else {
            // GCodeWriter.cpp:335
            String::new()
        }
    }

    // GCodeWriter.cpp:339  std::string GCodeWriter::update_progress(unsigned int num, unsigned int tot, bool allow_100) const
    pub fn update_progress(&self, num: u32, tot: u32, allow_100: bool) -> String {
        // GCodeWriter.cpp:341  if (FLAVOR_IS_NOT(gcfMakerWare) && FLAVOR_IS_NOT(gcfSailfish))
        if self.config.gcode_flavor != GCodeFlavor::MakerWare
            && self.config.gcode_flavor != GCodeFlavor::Sailfish
        {
            // GCodeWriter.cpp:342
            return String::new();
        }

        // GCodeWriter.cpp:344  unsigned int percent = (unsigned int)floor(100.0 * num / tot + 0.5);
        let mut percent = (100.0 * num as f64 / tot as f64 + 0.5).floor() as u32;
        // GCodeWriter.cpp:345  if (!allow_100) percent = std::min(percent, (unsigned int)99);
        if !allow_100 {
            percent = percent.min(99);
        }

        // GCodeWriter.cpp:347  std::ostringstream gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:348  gcode << "M73 P" << percent;
        let _ = write!(gcode, "M73 P{}", percent);
        // GCodeWriter.cpp:350
        if FULL_GCODE_COMMENT {
            gcode.push_str(" ; update progress");
        }
        // GCodeWriter.cpp:351
        gcode.push('\n');
        // GCodeWriter.cpp:352
        gcode
    }

    // GCodeWriter.cpp:355  std::string GCodeWriter::toolchange_prefix() const
    pub fn toolchange_prefix(&self) -> String {
        // GCodeWriter.cpp:357-358
        if self.config.gcode_flavor == GCodeFlavor::MakerWare {
            "M135 T".to_string()
        } else if self.config.gcode_flavor == GCodeFlavor::Sailfish {
            "M108 T".to_string()
        } else {
            "T".to_string()
        }
    }

    // Helper port of `Slic3r::lower_bound_by_predicate` (libslic3r.h:220) over the
    // sorted m_filament_extruders, returning the first index `i` with
    // !(e[i].id() < filament_id). Returns len() if none.
    fn lower_bound_filament(&self, filament_id: u32) -> usize {
        // libslic3r.h:222-237
        let mut first: usize = 0;
        let last: usize = self.m_filament_extruders.len();
        let mut count = last - first;
        while count > 0 {
            let step = count / 2;
            let it = first + step;
            if self.m_filament_extruders[it].id() < filament_id {
                first = it + 1;
                count -= step + 1;
            } else {
                count = step;
            }
        }
        first
    }

    // GCodeWriter.cpp:361  std::string GCodeWriter::toolchange(unsigned int filament_id, unsigned int nozzle_id)
    pub fn toolchange(&mut self, filament_id: u32, nozzle_id: u32) -> String {
        // GCodeWriter.cpp:363-364  set the new extruder
        let filament_extruder_idx = self.lower_bound_filament(filament_id);
        // GCodeWriter.cpp:365  assert(filament_extruder_iter != m_filament_extruders.end() && filament_extruder_iter->id() == filament_id);
        debug_assert!(
            filament_extruder_idx != self.m_filament_extruders.end_index()
                && self.m_filament_extruders[filament_extruder_idx].id() == filament_id
        );
        // GCodeWriter.cpp:366  m_curr_extruder_id = filament_extruder_iter->extruder_id();
        self.m_curr_extruder_id =
            self.m_filament_extruders[filament_extruder_idx].extruder_id() as i32;
        // GCodeWriter.cpp:367  m_curr_filament_extruder[m_curr_extruder_id] = &*filament_extruder_iter;
        self.m_curr_filament_extruder[self.m_curr_extruder_id as usize] = Some(filament_extruder_idx);

        // GCodeWriter.cpp:369-371  return the toolchange command
        let mut gcode = String::new();
        // GCodeWriter.cpp:372  if (this->multiple_extruders)
        if self.multiple_extruders {
            // GCodeWriter.cpp:374  if (this->m_is_bbl_printer)
            if self.m_is_bbl_printer {
                // GCodeWriter.cpp:375
                let _ = write!(gcode, "M1020 S{} H{}", filament_id, nozzle_id);
            } else {
                // GCodeWriter.cpp:377
                let _ = write!(gcode, "{}{}", self.toolchange_prefix(), filament_id);
            }
            // GCodeWriter.cpp:379-380
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; change extruder");
            }
            // GCodeWriter.cpp:381
            gcode.push('\n');
            // GCodeWriter.cpp:382
            let reset = self.reset_e(true);
            gcode.push_str(&reset);
        }
        // GCodeWriter.cpp:384
        gcode
    }

    // GCodeWriter.cpp:387  std::string GCodeWriter::set_speed(double F, const std::string &comment, const std::string &cooling_marker)
    pub fn set_speed(&mut self, f: f64, comment: &str, cooling_marker: &str) -> String {
        // GCodeWriter.cpp:389  assert(F > 0.);
        debug_assert!(f > 0.0);
        // GCodeWriter.cpp:390  assert(F < 100000.);
        debug_assert!(f < 100000.0);
        // GCodeWriter.cpp:391
        self.m_current_speed = f;
        // GCodeWriter.cpp:392  GCodeG1Formatter w;
        let mut w = GCodeG1Formatter::new();
        // GCodeWriter.cpp:393
        w.emit_f(f);
        // GCodeWriter.cpp:395
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:396
        w.emit_string(cooling_marker);
        // GCodeWriter.cpp:397
        w.string()
    }

    // GCodeWriter.hpp:81  double get_current_speed()
    pub fn get_current_speed(&self) -> f64 {
        self.m_current_speed
    }

    // Helper: travel_speed feedrate for the current filament (used many times).
    // GCodeWriter.cpp:416 (and others)
    //   this->config.travel_speed.get_at(get_process_config_idx(this->config, filament()->id())) * 60.0
    fn travel_feedrate(&self) -> f64 {
        let id = self.filament().unwrap().id();
        self.config
            .travel_speed
            .get_at(get_process_config_idx(&self.config, id))
            * 60.0
    }

    // GCodeWriter.cpp:400  std::string GCodeWriter::travel_to_xy(const Vec2d &point, const std::string &comment)
    pub fn travel_to_xy(&mut self, point: Vec2d, comment: &str) -> String {
        // GCodeWriter.cpp:402
        self.travel_to_xy_impl(point, comment, false)
    }

    // GCodeWriter.cpp:405  std::string GCodeWriter::travel_to_xy(const Vec2d &point, const std::string &comment, bool use_short_travel_acceleration)
    pub fn travel_to_xy_impl(
        &mut self,
        point: Vec2d,
        comment: &str,
        use_short_travel_acceleration: bool,
    ) -> String {
        // GCodeWriter.cpp:407  m_pos(0) = point(0);
        self.m_pos.x = point.x();
        // GCodeWriter.cpp:408  m_pos(1) = point(1);
        self.m_pos.y = point.y();

        // GCodeWriter.cpp:410
        self.set_current_position_clear(true);
        // GCodeWriter.cpp:412  Vec2d point_on_plate = { point(0) - m_x_offset, point(1) - m_y_offset };
        let point_on_plate = Vec2d::new(point.x() - self.m_x_offset, point.y() - self.m_y_offset);

        // GCodeWriter.cpp:414  GCodeG1Formatter w;
        let mut w = GCodeG1Formatter::new();
        // GCodeWriter.cpp:415
        w.emit_xy(point_on_plate);
        // GCodeWriter.cpp:416
        w.emit_f(self.travel_feedrate());
        // GCodeWriter.cpp:418
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:419
        let accel = self.set_travel_acceleration_impl(use_short_travel_acceleration);
        accel + &w.string()
    }

    // GCodeWriter.cpp:425  std::string GCodeWriter::lazy_lift(LiftType lift_type, bool spiral_vase, bool tool_change)
    pub fn lazy_lift(&mut self, lift_type: LiftType, spiral_vase: bool, tool_change: bool) -> String {
        // GCodeWriter.cpp:428  double target_lift = 0;
        let mut target_lift = 0.0;
        {
            // GCodeWriter.cpp:431  int extruder_id = filament()->extruder_id();
            let extruder_id = self.filament().unwrap().extruder_id();
            // GCodeWriter.cpp:432  int filament_id = filament()->id();
            let filament_id = self.filament().unwrap().id();
            // GCodeWriter.cpp:433  double above = this->config.retract_lift_above.get_at(extruder_id);
            let above = self.config.retract_lift_above.get_at(extruder_id as usize);
            // GCodeWriter.cpp:434  double below = this->config.retract_lift_below.get_at(extruder_id);
            let below = self.config.retract_lift_below.get_at(extruder_id as usize);
            // GCodeWriter.cpp:435  if (m_pos.z() >= above && m_pos.z() <= below)
            if self.m_pos.z >= above && self.m_pos.z <= below {
                // GCodeWriter.cpp:436  target_lift = this->config.z_hop.get_at(filament_id);
                target_lift = self.config.z_hop.get_at(filament_id as usize);
                // GCodeWriter.cpp:437  if (tool_change && this->config.prime_tower_lift_height.value > 0) target_lift = ...
                if tool_change && self.config.prime_tower_lift_height > 0.0 {
                    target_lift = self.config.prime_tower_lift_height;
                }
            }
        }
        // GCodeWriter.cpp:441  if (m_lifted == 0 && m_to_lift == 0 && target_lift > 0)
        if self.m_lifted == 0.0 && self.m_to_lift == 0.0 && target_lift > 0.0 {
            // GCodeWriter.cpp:442
            if spiral_vase {
                // GCodeWriter.cpp:443
                self.m_lifted = target_lift;
                // GCodeWriter.cpp:444
                return self._travel_to_z(self.m_pos.z + target_lift, "lift Z", tool_change);
            } else {
                // GCodeWriter.cpp:447
                self.m_to_lift = target_lift;
                // GCodeWriter.cpp:448
                self.m_to_lift_type = lift_type;
            }
        }
        // GCodeWriter.cpp:451
        String::new()
    }

    // GCodeWriter.cpp:456  std::string GCodeWriter::eager_lift(const LiftType type, bool tool_change)
    pub fn eager_lift(&mut self, lift_type: LiftType, tool_change: bool) -> String {
        // GCodeWriter.cpp:458  std::string lift_move;
        let mut lift_move = String::new();
        // GCodeWriter.cpp:459  double target_lift = 0;
        let mut target_lift = 0.0;
        {
            // GCodeWriter.cpp:462  int extruder_id = filament()->extruder_id();
            let extruder_id = self.filament().unwrap().extruder_id();
            // GCodeWriter.cpp:463  int filament_id = filament()->id();
            let filament_id = self.filament().unwrap().id();
            // GCodeWriter.cpp:464  double above = this->config.retract_lift_above.get_at(extruder_id);
            let above = self.config.retract_lift_above.get_at(extruder_id as usize);
            // GCodeWriter.cpp:465  double below = this->config.retract_lift_below.get_at(extruder_id);
            let below = self.config.retract_lift_below.get_at(extruder_id as usize);
            // GCodeWriter.cpp:466
            if self.m_pos.z >= above && self.m_pos.z <= below {
                // GCodeWriter.cpp:467
                target_lift = self.config.z_hop.get_at(filament_id as usize);
                // GCodeWriter.cpp:468
                if tool_change && self.config.prime_tower_lift_height > 0.0 {
                    target_lift = self.config.prime_tower_lift_height;
                }
            }
        }

        // GCodeWriter.cpp:472  double to_lift = target_lift - m_lifted;
        let to_lift = target_lift - self.m_lifted;
        // GCodeWriter.cpp:473  if (to_lift < EPSILON)
        if to_lift < EPSILON {
            // GCodeWriter.cpp:474
            return lift_move;
        }

        // GCodeWriter.cpp:478  if (type == LiftType::SpiralLift && this->is_current_position_clear())
        if lift_type == LiftType::SpiralLift && self.is_current_position_clear() {
            // GCodeWriter.cpp:479
            if to_lift > 0.0 {
                // GCodeWriter.cpp:480  double radius = to_lift / (2 * PI * atan(GCodeWriter::slope_threshold));
                let radius = to_lift / (2.0 * PI * SLOPE_THRESHOLD.atan());
                // GCodeWriter.cpp:483  Vec2d ij_offset = { radius, 0 };
                let ij_offset = Vec2d::new(radius, 0.0);
                // GCodeWriter.cpp:484
                lift_move =
                    self._spiral_travel_to_z(self.m_pos.z + to_lift, ij_offset, "spiral lift Z", tool_change);
            }
        }
        // GCodeWriter.cpp:488  else if (to_lift > 0)
        else if to_lift > 0.0 {
            // GCodeWriter.cpp:489
            lift_move = self._travel_to_z(self.m_pos.z + to_lift, "normal lift Z", tool_change);
        }
        // GCodeWriter.cpp:491
        self.m_lifted = target_lift;
        // GCodeWriter.cpp:492
        self.m_to_lift = 0.0;
        // GCodeWriter.cpp:493
        lift_move
    }

    // GCodeWriter.cpp:496  std::string GCodeWriter::travel_to_xyz(const Vec3d &point, const std::string &comment)
    pub fn travel_to_xyz(&mut self, point: Vec3d, comment: &str) -> String {
        // GCodeWriter.cpp:498
        self.travel_to_xyz_impl(point, comment, false)
    }

    // GCodeWriter.cpp:501  std::string GCodeWriter::travel_to_xyz(const Vec3d &point, const std::string &comment, bool use_short_travel_acceleration)
    pub fn travel_to_xyz_impl(
        &mut self,
        point: Vec3d,
        comment: &str,
        use_short_travel_acceleration: bool,
    ) -> String {
        // GCodeWriter.cpp:513  Vec3d dest_point = point;
        let mut dest_point = point;
        // GCodeWriter.cpp:515  if (std::abs(m_to_lift) > EPSILON)
        if self.m_to_lift.abs() > EPSILON {
            // GCodeWriter.cpp:516  assert(std::abs(m_lifted) < EPSILON);
            debug_assert!(self.m_lifted.abs() < EPSILON);
            // GCodeWriter.cpp:520-521  if ((!this->is_current_position_clear() || m_pos != dest_point) && m_to_lift + m_pos(2) > point(2))
            if (!self.is_current_position_clear() || self.m_pos != dest_point)
                && self.m_to_lift + self.m_pos.z > point.z
            {
                // GCodeWriter.cpp:522  m_lifted = m_to_lift + m_pos(2) - point(2);
                self.m_lifted = self.m_to_lift + self.m_pos.z - point.z;
                // GCodeWriter.cpp:523  dest_point(2) = m_to_lift + m_pos(2);
                dest_point.z = self.m_to_lift + self.m_pos.z;
            }
            // GCodeWriter.cpp:525  m_to_lift = 0.;
            self.m_to_lift = 0.0;

            // GCodeWriter.cpp:527  std::string slop_move;
            let mut slop_move = String::new();
            // GCodeWriter.cpp:529  Vec3d source = { m_pos(0) - m_x_offset, m_pos(1) - m_y_offset, m_pos(2) };
            let source = Vec3d::new(
                self.m_pos.x - self.m_x_offset,
                self.m_pos.y - self.m_y_offset,
                self.m_pos.z,
            );
            // GCodeWriter.cpp:530  Vec3d target = { dest_point(0) - m_x_offset, dest_point(1) - m_y_offset, dest_point(2) };
            let target = Vec3d::new(
                dest_point.x - self.m_x_offset,
                dest_point.y - self.m_y_offset,
                dest_point.z,
            );
            // GCodeWriter.cpp:531  Vec3d delta = target - source;
            let delta = target - source;
            // GCodeWriter.cpp:532  Vec2d delta_no_z = { delta(0), delta(1) };
            let delta_no_z = Vec2d::new(delta.x, delta.y);
            // GCodeWriter.cpp:535  if (delta(2) > 0 && delta_no_z.norm() != 0.0f)
            if delta.z > 0.0 && delta_no_z.length() != 0.0 {
                // GCodeWriter.cpp:537  if (m_to_lift_type == LiftType::SpiralLift && this->is_current_position_clear())
                if self.m_to_lift_type == LiftType::SpiralLift && self.is_current_position_clear() {
                    // GCodeWriter.cpp:539  double radius = delta(2) / (2 * PI * atan(GCodeWriter::slope_threshold));
                    let radius = delta.z / (2.0 * PI * SLOPE_THRESHOLD.atan());
                    // GCodeWriter.cpp:540  Vec2d ij_offset = radius * delta_no_z.normalized();
                    let mut ij_offset = delta_no_z.normalize() * radius;
                    // GCodeWriter.cpp:541  ij_offset = { -ij_offset(1), ij_offset(0) };
                    ij_offset = Vec2d::new(-ij_offset.y(), ij_offset.x());
                    // GCodeWriter.cpp:542
                    slop_move = self._spiral_travel_to_z_default(target.z, ij_offset, "spiral lift Z");
                }
                // GCodeWriter.cpp:545-547  else if (m_to_lift_type == LiftType::SlopeLift && is_current_position_clear() && atan2(delta(2), delta_no_z.norm()) < slope_threshold)
                else if self.m_to_lift_type == LiftType::SlopeLift
                    && self.is_current_position_clear()
                    && delta.z.atan2(delta_no_z.length()) < SLOPE_THRESHOLD
                {
                    // GCodeWriter.cpp:551  Vec2d temp = delta_no_z.normalized() * delta(2) / tan(GCodeWriter::slope_threshold);
                    let temp = delta_no_z.normalize() * delta.z / SLOPE_THRESHOLD.tan();
                    // GCodeWriter.cpp:552  Vec3d slope_top_point = Vec3d(temp(0), temp(1), delta(2)) + source;
                    let slope_top_point = Vec3d::new(temp.x(), temp.y(), delta.z) + source;
                    // GCodeWriter.cpp:553  GCodeG1Formatter w0;
                    let mut w0 = GCodeG1Formatter::new();
                    // GCodeWriter.cpp:554
                    w0.emit_xyz(slope_top_point);
                    // GCodeWriter.cpp:555
                    w0.emit_f(self.travel_feedrate());
                    // GCodeWriter.cpp:557
                    w0.emit_comment(FULL_GCODE_COMMENT, "slope lift Z");
                    // GCodeWriter.cpp:558
                    slop_move = w0.string();
                }
                // GCodeWriter.cpp:560  else if (m_to_lift_type == LiftType::NormalLift)
                else if self.m_to_lift_type == LiftType::NormalLift {
                    // GCodeWriter.cpp:561
                    slop_move = self._travel_to_z_default(target.z, "normal lift Z");
                }
            }

            // GCodeWriter.cpp:565  std::string xy_z_move;
            let xy_z_move: String;
            {
                // GCodeWriter.cpp:567  GCodeG1Formatter w0;
                let mut w0 = GCodeG1Formatter::new();
                // GCodeWriter.cpp:568  if (this->is_current_position_clear())
                if self.is_current_position_clear() {
                    // GCodeWriter.cpp:569
                    w0.emit_xyz(target);
                    // GCodeWriter.cpp:570
                    w0.emit_f(self.travel_feedrate());
                    // GCodeWriter.cpp:571
                    w0.emit_comment(FULL_GCODE_COMMENT, comment);
                    // GCodeWriter.cpp:572
                    xy_z_move = w0.string();
                } else {
                    // GCodeWriter.cpp:575  w0.emit_xy(Vec2d(target.x(), target.y()));
                    w0.emit_xy(Vec2d::new(target.x, target.y));
                    // GCodeWriter.cpp:576
                    w0.emit_f(self.travel_feedrate());
                    // GCodeWriter.cpp:577
                    w0.emit_comment(FULL_GCODE_COMMENT, comment);
                    // GCodeWriter.cpp:578  xy_z_move = w0.string() + _travel_to_z(target.z(), comment);
                    let s = w0.string();
                    let z = self._travel_to_z_default(target.z, comment);
                    xy_z_move = s + &z;
                }
            }
            // GCodeWriter.cpp:581  m_pos = dest_point;
            self.m_pos = dest_point;
            // GCodeWriter.cpp:582
            self.set_current_position_clear(true);
            // GCodeWriter.cpp:583
            let accel = self.set_travel_acceleration_impl(use_short_travel_acceleration);
            return accel + &slop_move + &xy_z_move;
        }
        // GCodeWriter.cpp:585  else if (!this->will_move_z(point(2)))
        else if !self.will_move_z(point.z) {
            // GCodeWriter.cpp:586  double nominal_z = m_pos(2) - m_lifted;
            let nominal_z = self.m_pos.z - self.m_lifted;
            // GCodeWriter.cpp:587  m_lifted -= (point(2) - nominal_z);
            self.m_lifted -= point.z - nominal_z;
            // GCodeWriter.cpp:590  if (std::abs(m_lifted) < EPSILON)
            if self.m_lifted.abs() < EPSILON {
                // GCodeWriter.cpp:591
                self.m_lifted = 0.0;
            }
            // GCodeWriter.cpp:593
            self.set_current_position_clear(true);
            // GCodeWriter.cpp:594  return this->travel_to_xy(to_2d(point), std::string(), use_short_travel_acceleration);
            return self.travel_to_xy_impl(
                Vec2d::new(point.x, point.y),
                "",
                use_short_travel_acceleration,
            );
        } else {
            // GCodeWriter.cpp:599  m_lifted = 0;
            self.m_lifted = 0.0;
        }

        // GCodeWriter.cpp:603  Vec3d point_on_plate = { dest_point(0) - m_x_offset, dest_point(1) - m_y_offset, dest_point(2) };
        let point_on_plate = Vec3d::new(
            dest_point.x - self.m_x_offset,
            dest_point.y - self.m_y_offset,
            dest_point.z,
        );
        // GCodeWriter.cpp:604  std::string out_string;
        let out_string: String;
        // GCodeWriter.cpp:605  GCodeG1Formatter w;
        // GCodeWriter.cpp:606  if (!this->is_current_position_clear())
        if !self.is_current_position_clear() {
            let mut w = GCodeG1Formatter::new();
            // GCodeWriter.cpp:609  force to move xy first then z after filament change
            w.emit_xy(Vec2d::new(point_on_plate.x, point_on_plate.y));
            // GCodeWriter.cpp:610
            w.emit_f(self.travel_feedrate());
            // GCodeWriter.cpp:611
            w.emit_comment(FULL_GCODE_COMMENT, comment);
            // GCodeWriter.cpp:612  out_string = w.string() + _travel_to_z(point_on_plate.z(), comment);
            let s = w.string();
            let z = self._travel_to_z_default(point_on_plate.z, comment);
            out_string = s + &z;
        } else {
            // GCodeWriter.cpp:614  GCodeG1Formatter w;
            let mut w = GCodeG1Formatter::new();
            // GCodeWriter.cpp:615
            w.emit_xyz(point_on_plate);
            // GCodeWriter.cpp:616
            w.emit_f(self.travel_feedrate());
            // GCodeWriter.cpp:617
            w.emit_comment(FULL_GCODE_COMMENT, comment);
            // GCodeWriter.cpp:618
            out_string = w.string();
        }

        // GCodeWriter.cpp:621  m_pos = dest_point;
        self.m_pos = dest_point;
        // GCodeWriter.cpp:622
        self.set_current_position_clear(true);
        // GCodeWriter.cpp:623
        let accel = self.set_travel_acceleration_impl(use_short_travel_acceleration);
        accel + &out_string
    }

    // GCodeWriter.cpp:626  std::string GCodeWriter::travel_to_z(double z, const std::string &comment)
    pub fn travel_to_z(&mut self, z: f64, comment: &str) -> String {
        // GCodeWriter.cpp:631  if (!this->will_move_z(z))
        if !self.will_move_z(z) {
            // GCodeWriter.cpp:632  double nominal_z = m_pos(2) - m_lifted;
            let nominal_z = self.m_pos.z - self.m_lifted;
            // GCodeWriter.cpp:633  m_lifted -= (z - nominal_z);
            self.m_lifted -= z - nominal_z;
            // GCodeWriter.cpp:634  if (std::abs(m_lifted) < EPSILON)
            if self.m_lifted.abs() < EPSILON {
                // GCodeWriter.cpp:635
                self.m_lifted = 0.0;
            }
            // GCodeWriter.cpp:636
            return String::new();
        }

        // GCodeWriter.cpp:641  m_lifted = 0;
        self.m_lifted = 0.0;
        // GCodeWriter.cpp:642  return set_travel_acceleration() + this->_travel_to_z(z, comment);
        let accel = self.set_travel_acceleration();
        let z_move = self._travel_to_z_default(z, comment);
        accel + &z_move
    }

    // GCodeWriter.cpp:645  std::string GCodeWriter::_travel_to_z(double z, const std::string &comment, bool tool_change)
    // Default `tool_change=false` (GCodeWriter.hpp:180).
    fn _travel_to_z_default(&mut self, z: f64, comment: &str) -> String {
        self._travel_to_z(z, comment, false)
    }

    fn _travel_to_z(&mut self, z: f64, comment: &str, tool_change: bool) -> String {
        // GCodeWriter.cpp:647  m_pos(2) = z;
        self.m_pos.z = z;

        // GCodeWriter.cpp:649  double speed = this->config.travel_speed_z.get_at(get_process_config_idx(this->config, filament()->id()));
        let id = self.filament().unwrap().id();
        let mut speed = self
            .config
            .travel_speed_z
            .get_at(get_process_config_idx(&self.config, id));
        // GCodeWriter.cpp:650  if (speed == 0.)
        if speed == 0.0 {
            // GCodeWriter.cpp:651
            speed = self
                .config
                .travel_speed
                .get_at(get_process_config_idx(&self.config, id));
        }
        // GCodeWriter.cpp:652  if (tool_change && this->config.prime_tower_lift_speed.value>0)
        if tool_change && self.config.prime_tower_lift_speed > 0.0 {
            // GCodeWriter.cpp:653
            speed = self.config.prime_tower_lift_speed; // lift speed
        }
        // GCodeWriter.cpp:655  GCodeG1Formatter w;
        let mut w = GCodeG1Formatter::new();
        // GCodeWriter.cpp:656
        w.emit_z(z);
        // GCodeWriter.cpp:657
        w.emit_f(speed * 60.0);
        // GCodeWriter.cpp:659
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:660  return set_travel_acceleration() + w.string();
        let accel = self.set_travel_acceleration();
        accel + &w.string()
    }

    // GCodeWriter.cpp:663  std::string GCodeWriter::_spiral_travel_to_z(double z, const Vec2d &ij_offset, const std::string &comment, bool tool_change)
    // Default `tool_change=false` (GCodeWriter.hpp:181).
    fn _spiral_travel_to_z_default(&mut self, z: f64, ij_offset: Vec2d, comment: &str) -> String {
        self._spiral_travel_to_z(z, ij_offset, comment, false)
    }

    fn _spiral_travel_to_z(
        &mut self,
        z: f64,
        ij_offset: Vec2d,
        comment: &str,
        tool_change: bool,
    ) -> String {
        // GCodeWriter.cpp:665  m_pos(2) = z;
        self.m_pos.z = z;

        // GCodeWriter.cpp:667
        let id = self.filament().unwrap().id();
        let mut speed = self
            .config
            .travel_speed_z
            .get_at(get_process_config_idx(&self.config, id));
        // GCodeWriter.cpp:668  if (speed == 0.)
        if speed == 0.0 {
            // GCodeWriter.cpp:669
            speed = self
                .config
                .travel_speed
                .get_at(get_process_config_idx(&self.config, id));
        }
        // GCodeWriter.cpp:670  if (tool_change && this->config.prime_tower_lift_speed.value>0)
        if tool_change && self.config.prime_tower_lift_speed > 0.0 {
            // GCodeWriter.cpp:671
            speed = self.config.prime_tower_lift_speed; // lift speed
        }
        // GCodeWriter.cpp:673  std::string output = "G17\n";
        let output = "G17\n".to_string();
        // GCodeWriter.cpp:674  GCodeG2G3Formatter w(true);
        let mut w = GCodeG2G3Formatter::new(true);
        // GCodeWriter.cpp:675
        w.emit_z(z);
        // GCodeWriter.cpp:676
        w.emit_ij(ij_offset);
        // GCodeWriter.cpp:677
        w.emit_string(" P1 ");
        // GCodeWriter.cpp:678
        w.emit_f(speed * 60.0);
        // GCodeWriter.cpp:679
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:680  return set_travel_acceleration() + output + w.string();
        let accel = self.set_travel_acceleration();
        accel + &output + &w.string()
    }

    // GCodeWriter.cpp:683  bool GCodeWriter::will_move_z(double z) const
    pub fn will_move_z(&self, z: f64) -> bool {
        // GCodeWriter.cpp:687  if (m_lifted > 0)
        if self.m_lifted > 0.0 {
            // GCodeWriter.cpp:688  double nominal_z = m_pos(2) - m_lifted;
            let nominal_z = self.m_pos.z - self.m_lifted;
            // GCodeWriter.cpp:689  if (z >= nominal_z - EPSILON && z <= m_pos(2) + EPSILON)
            if z >= nominal_z - EPSILON && z <= self.m_pos.z + EPSILON {
                // GCodeWriter.cpp:690
                return false;
            }
        }
        // GCodeWriter.cpp:694  else if (std::abs(m_pos(2) - z) < EPSILON)
        else if (self.m_pos.z - z).abs() < EPSILON {
            // GCodeWriter.cpp:695
            return false;
        }
        // GCodeWriter.cpp:697
        true
    }

    // GCodeWriter.cpp:700  std::string GCodeWriter::extrude_to_xy(const Vec2d &point, double dE, const std::string &comment, bool force_no_extrusion)
    pub fn extrude_to_xy(
        &mut self,
        point: Vec2d,
        d_e: f64,
        comment: &str,
        force_no_extrusion: bool,
    ) -> String {
        // GCodeWriter.cpp:702  m_pos(0) = point(0);
        self.m_pos.x = point.x();
        // GCodeWriter.cpp:703  m_pos(1) = point(1);
        self.m_pos.y = point.y();

        // GCodeWriter.cpp:705  if (!force_no_extrusion)
        if !force_no_extrusion {
            // GCodeWriter.cpp:706  filament()->extrude(dE);
            self.filament_mut().unwrap().extrude(d_e);
        }

        // GCodeWriter.cpp:709  Vec2d point_on_plate = { point(0) - m_x_offset, point(1) - m_y_offset };
        let point_on_plate = Vec2d::new(point.x() - self.m_x_offset, point.y() - self.m_y_offset);

        // GCodeWriter.cpp:711  GCodeG1Formatter w;
        let mut w = GCodeG1Formatter::new();
        // GCodeWriter.cpp:712
        w.emit_xy(point_on_plate);
        // GCodeWriter.cpp:713  if (!force_no_extrusion)
        if !force_no_extrusion {
            // GCodeWriter.cpp:714  w.emit_e(filament()->E());
            w.emit_e(self.filament().unwrap().e());
        }
        // GCodeWriter.cpp:716
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:717  return set_extrude_acceleration() + w.string();
        let accel = self.set_extrude_acceleration();
        accel + &w.string()
    }

    // GCodeWriter.cpp:723  std::string GCodeWriter::extrude_arc_to_xy(const Vec2d& point, const Vec2d& center_offset, double dE, const bool is_ccw, const std::string& comment, bool force_no_extrusion)
    pub fn extrude_arc_to_xy(
        &mut self,
        point: Vec2d,
        center_offset: Vec2d,
        d_e: f64,
        is_ccw: bool,
        comment: &str,
        force_no_extrusion: bool,
    ) -> String {
        // GCodeWriter.cpp:725  m_pos(0) = point(0);
        self.m_pos.x = point.x();
        // GCodeWriter.cpp:726  m_pos(1) = point(1);
        self.m_pos.y = point.y();
        // GCodeWriter.cpp:727  if (!force_no_extrusion)
        if !force_no_extrusion {
            // GCodeWriter.cpp:728
            self.filament_mut().unwrap().extrude(d_e);
        }

        // GCodeWriter.cpp:730  Vec2d point_on_plate = { point(0) - m_x_offset, point(1) - m_y_offset };
        let point_on_plate = Vec2d::new(point.x() - self.m_x_offset, point.y() - self.m_y_offset);

        // GCodeWriter.cpp:732  GCodeG2G3Formatter w(is_ccw);
        let mut w = GCodeG2G3Formatter::new(is_ccw);
        // GCodeWriter.cpp:733
        w.emit_xy(point_on_plate);
        // GCodeWriter.cpp:734
        w.emit_ij(center_offset);
        // GCodeWriter.cpp:735  if (!force_no_extrusion)
        if !force_no_extrusion {
            // GCodeWriter.cpp:736
            w.emit_e(self.filament().unwrap().e());
        }
        // GCodeWriter.cpp:738
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:739  return set_extrude_acceleration() + w.string();
        let accel = self.set_extrude_acceleration();
        accel + &w.string()
    }

    // GCodeWriter.cpp:742  std::string GCodeWriter::extrude_to_xyz(const Vec3d &point, double dE, const std::string &comment, bool force_no_extrusion)
    pub fn extrude_to_xyz(
        &mut self,
        point: Vec3d,
        d_e: f64,
        comment: &str,
        force_no_extrusion: bool,
    ) -> String {
        // GCodeWriter.cpp:744  m_pos = point;
        self.m_pos = point;
        // GCodeWriter.cpp:745  m_lifted = 0;
        self.m_lifted = 0.0;
        // GCodeWriter.cpp:746  if (!force_no_extrusion)
        if !force_no_extrusion {
            // GCodeWriter.cpp:747
            self.filament_mut().unwrap().extrude(d_e);
        }

        // GCodeWriter.cpp:750  Vec3d point_on_plate = { point(0) - m_x_offset, point(1) - m_y_offset, point(2) };
        let point_on_plate = Vec3d::new(
            point.x - self.m_x_offset,
            point.y - self.m_y_offset,
            point.z,
        );

        // GCodeWriter.cpp:752  GCodeG1Formatter w;
        let mut w = GCodeG1Formatter::new();
        // GCodeWriter.cpp:753
        w.emit_xyz(point_on_plate);
        // GCodeWriter.cpp:754  if (!force_no_extrusion)
        if !force_no_extrusion {
            // GCodeWriter.cpp:755
            w.emit_e(self.filament().unwrap().e());
        }
        // GCodeWriter.cpp:757
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:758  return set_extrude_acceleration() + w.string();
        let accel = self.set_extrude_acceleration();
        accel + &w.string()
    }

    // GCodeWriter.cpp:761  std::string GCodeWriter::retract(bool before_wipe)
    pub fn retract(&mut self, before_wipe: bool) -> String {
        // GCodeWriter.cpp:763  double factor = before_wipe ? filament()->retract_before_wipe() : 1.;
        let factor = if before_wipe {
            self.filament().unwrap().retract_before_wipe()
        } else {
            1.0
        };
        // GCodeWriter.cpp:764  assert(factor >= 0. && factor <= 1. + EPSILON);
        debug_assert!(factor >= 0.0 && factor <= 1.0 + EPSILON);
        // GCodeWriter.cpp:765-769
        let length = factor * self.filament().unwrap().retraction_length();
        let restart_extra = factor * self.filament().unwrap().retract_restart_extra();
        self._retract(length, restart_extra, "retract")
    }

    // GCodeWriter.cpp:772  std::string GCodeWriter::retract_for_toolchange(bool before_wipe)
    pub fn retract_for_toolchange(&mut self, before_wipe: bool) -> String {
        // GCodeWriter.cpp:774  double factor = before_wipe ? filament()->retract_before_wipe() : 1.;
        let factor = if before_wipe {
            self.filament().unwrap().retract_before_wipe()
        } else {
            1.0
        };
        // GCodeWriter.cpp:775  assert(factor >= 0. && factor <= 1. + EPSILON);
        debug_assert!(factor >= 0.0 && factor <= 1.0 + EPSILON);
        // GCodeWriter.cpp:776-780
        let length = factor * self.filament().unwrap().retract_length_toolchange();
        let restart_extra = factor * self.filament().unwrap().retract_restart_extra_toolchange();
        self._retract(length, restart_extra, "retract for toolchange")
    }

    // GCodeWriter.cpp:783  std::string GCodeWriter::_retract(double length, double restart_extra, const std::string &comment)
    fn _retract(&mut self, mut length: f64, restart_extra: f64, comment: &str) -> String {
        // GCodeWriter.cpp:785  std::string gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:786  if (config.use_firmware_retraction)
        if self.config.use_firmware_retraction {
            // GCodeWriter.cpp:787
            length = 1.0;
        }
        // GCodeWriter.cpp:788  if (double dE = filament()->retract(length, restart_extra); dE != 0)
        let d_e = self.filament_mut().unwrap().retract(length, restart_extra);
        if d_e != 0.0 {
            // GCodeWriter.cpp:790  add firmware retraction
            if self.config.use_firmware_retraction {
                // GCodeWriter.cpp:791  gcode = FLAVOR_IS(gcfMachinekit) ? "G22 ;retract" : "G10 ;retract \n";
                gcode = if self.config.gcode_flavor == GCodeFlavor::Machinekit {
                    "G22 ;retract".to_string()
                } else {
                    "G10 ;retract \n".to_string()
                };
            } else {
                // GCodeWriter.cpp:795  GCodeG1Formatter w;
                let mut w = GCodeG1Formatter::new();
                // GCodeWriter.cpp:796  w.emit_e(filament()->E());
                w.emit_e(self.filament().unwrap().e());
                // GCodeWriter.cpp:797  w.emit_f(filament()->retract_speed() * 60.);
                w.emit_f(self.filament().unwrap().retract_speed() as f64 * 60.0);
                // GCodeWriter.cpp:799
                w.emit_comment(FULL_GCODE_COMMENT, comment);
                // GCodeWriter.cpp:800
                gcode = w.string();
            }
        }

        // GCodeWriter.cpp:804  if (FLAVOR_IS(gcfMakerWare))
        if self.config.gcode_flavor == GCodeFlavor::MakerWare {
            // GCodeWriter.cpp:805
            gcode.push_str("M103 ; extruder off\n");
        }

        // GCodeWriter.cpp:807
        gcode
    }

    // GCodeWriter.cpp:810  std::string GCodeWriter::unretract(float extra_retract)
    pub fn unretract(&mut self, extra_retract: f32) -> String {
        // GCodeWriter.cpp:812  std::string gcode;
        let mut gcode = String::new();

        // GCodeWriter.cpp:814  if (FLAVOR_IS(gcfMakerWare))
        if self.config.gcode_flavor == GCodeFlavor::MakerWare {
            // GCodeWriter.cpp:815
            gcode = "M101 ; extruder on\n".to_string();
        }

        // GCodeWriter.cpp:817  if (double dE = filament()->unretract(); dE != 0)
        let d_e = self.filament_mut().unwrap().unretract();
        if d_e != 0.0 {
            // GCodeWriter.cpp:818  if (config.use_firmware_retraction)
            if self.config.use_firmware_retraction {
                // GCodeWriter.cpp:819
                gcode += if self.config.gcode_flavor == GCodeFlavor::Machinekit {
                    "G23 ;unretract \n"
                } else {
                    "G11 ;unretract \n"
                };
                // GCodeWriter.cpp:820
                let reset = self.reset_e(false);
                gcode += &reset;
            } else {
                // GCodeWriter.cpp:825  use G1 instead of G0 because G0 will blend the restart with the previous travel move
                let mut w = GCodeG1Formatter::new();
                // GCodeWriter.cpp:826  w.emit_e(filament()->E()+extra_retract);
                w.emit_e(self.filament().unwrap().e() + extra_retract as f64);
                // GCodeWriter.cpp:827  w.emit_f(filament()->deretract_speed() * 60.);
                w.emit_f(self.filament().unwrap().deretract_speed() as f64 * 60.0);
                // GCodeWriter.cpp:829
                w.emit_comment(FULL_GCODE_COMMENT, " ; unretract");
                // GCodeWriter.cpp:830
                gcode += &w.string();
            }
        }

        // GCodeWriter.cpp:834
        gcode
    }

    // GCodeWriter.cpp:837  double GCodeWriter::get_extruder_retracted_length(const int filament_id)
    pub fn get_extruder_retracted_length(&self, filament_id: i32) -> f64 {
        // GCodeWriter.cpp:839  double res = 0.0;
        let mut res = 0.0;
        // GCodeWriter.cpp:840  auto filament_extruder_iter = ... lower_bound_by_predicate(...)
        let filament_extruder_idx = self.lower_bound_filament(filament_id as u32);
        // GCodeWriter.cpp:841  assert(...)
        debug_assert!(
            filament_extruder_idx != self.m_filament_extruders.end_index()
                && self.m_filament_extruders[filament_extruder_idx].id() == filament_id as u32
        );

        // GCodeWriter.cpp:843  if (filament_extruder_iter->is_share_extruder())
        if self.m_filament_extruders[filament_extruder_idx].is_share_extruder() {
            // GCodeWriter.cpp:844
            res = self.m_filament_extruders[filament_extruder_idx].get_share_retracted_length();
        } else {
            // GCodeWriter.cpp:846
            res = self.m_filament_extruders[filament_extruder_idx].get_single_retracted_length();
        }

        // GCodeWriter.cpp:848
        res
    }

    // GCodeWriter.cpp:851  std::string GCodeWriter::unlift()
    pub fn unlift(&mut self) -> String {
        // GCodeWriter.cpp:853  std::string gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:854  if (m_lifted > 0)
        if self.m_lifted > 0.0 {
            // GCodeWriter.cpp:855  gcode += this->_travel_to_z(m_pos(2) - m_lifted, "restore layer Z");
            let z_move = self._travel_to_z_default(self.m_pos.z - self.m_lifted, "restore layer Z");
            gcode += &z_move;
            // GCodeWriter.cpp:856
            self.m_lifted = 0.0;
        }
        // GCodeWriter.cpp:858  m_to_lift = 0.;
        self.m_to_lift = 0.0;
        // GCodeWriter.cpp:859
        gcode
    }

    // GCodeWriter.cpp:862  std::string GCodeWriter::set_fan(const GCodeFlavor gcode_flavor, unsigned int speed)
    pub fn set_fan_static(gcode_flavor: GCodeFlavor, speed: u32) -> String {
        // GCodeWriter.cpp:864  std::ostringstream gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:865  if (speed == 0)
        if speed == 0 {
            // GCodeWriter.cpp:866
            match gcode_flavor {
                // GCodeWriter.cpp:867-868
                GCodeFlavor::Teacup => gcode.push_str("M106 S0"),
                // GCodeWriter.cpp:869-871
                GCodeFlavor::MakerWare | GCodeFlavor::Sailfish => gcode.push_str("M127"),
                // GCodeWriter.cpp:872-873
                _ => gcode.push_str("M106 S0"),
            }
            // GCodeWriter.cpp:875
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; disable fan");
            }
            // GCodeWriter.cpp:877
            gcode.push('\n');
        } else {
            // GCodeWriter.cpp:879
            match gcode_flavor {
                // GCodeWriter.cpp:880-882
                GCodeFlavor::MakerWare | GCodeFlavor::Sailfish => gcode.push_str("M126"),
                // GCodeWriter.cpp:883-885
                GCodeFlavor::Mach3 | GCodeFlavor::Machinekit => {
                    let _ = write!(gcode, "M106 P{}", ostream_double(255.0 * speed as f64 / 100.0));
                }
                // GCodeWriter.cpp:886-887
                _ => {
                    let _ = write!(gcode, "M106 S{}", ostream_double(255.0 * speed as f64 / 100.0));
                }
            }
            // GCodeWriter.cpp:889
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; enable fan");
            }
            // GCodeWriter.cpp:891
            gcode.push('\n');
        }
        // GCodeWriter.cpp:893
        gcode
    }

    // GCodeWriter.cpp:896  std::string GCodeWriter::set_fan(unsigned int speed) const
    pub fn set_fan(&self, speed: u32) -> String {
        // GCodeWriter.cpp:899
        GCodeWriter::set_fan_static(self.config.gcode_flavor, speed)
    }

    // GCodeWriter.cpp:903  std::string GCodeWriter::set_additional_fan(unsigned int speed)
    pub fn set_additional_fan(speed: u32) -> String {
        // GCodeWriter.cpp:905  std::ostringstream gcode;
        let mut gcode = String::new();

        // GCodeWriter.cpp:907  gcode << "M106 " << "P2 " << "S" << (int)(255.0 * speed / 100.0);
        let _ = write!(gcode, "M106 P2 S{}", (255.0 * speed as f64 / 100.0) as i32);
        // GCodeWriter.cpp:908
        if FULL_GCODE_COMMENT {
            // GCodeWriter.cpp:909  if (speed == 0)
            if speed == 0 {
                // GCodeWriter.cpp:910
                gcode.push_str(" ; disable additional fan ");
            } else {
                // GCodeWriter.cpp:912
                gcode.push_str(" ; enable additional fan ");
            }
        }
        // GCodeWriter.cpp:914
        gcode.push('\n');
        // GCodeWriter.cpp:915
        gcode
    }

    // GCodeWriter.cpp:918  std::string GCodeWriter::set_exhaust_fan(int speed, bool add_eol)
    pub fn set_exhaust_fan(speed: i32, add_eol: bool) -> String {
        // GCodeWriter.cpp:920  std::ostringstream gcode;
        let mut gcode = String::new();
        // GCodeWriter.cpp:921  gcode << "M106" << " P3" << " S" << (int)(speed / 100.0 * 255);
        let _ = write!(gcode, "M106 P3 S{}", (speed as f64 / 100.0 * 255.0) as i32);

        // GCodeWriter.cpp:923  if(add_eol)
        if add_eol {
            // GCodeWriter.cpp:924
            gcode.push('\n');
        }
        // GCodeWriter.cpp:925
        gcode
    }

    // GCodeWriter.hpp:116  void set_object_start_str(std::string start_string)
    pub fn set_object_start_str(&mut self, start_string: String) {
        self.m_gcode_label_objects_start = start_string;
    }
    // GCodeWriter.hpp:117  bool empty_object_start_str()
    pub fn empty_object_start_str(&self) -> bool {
        self.m_gcode_label_objects_start.is_empty()
    }
    // GCodeWriter.hpp:118  void set_object_end_str(std::string end_string)
    pub fn set_object_end_str(&mut self, end_string: String) {
        self.m_gcode_label_objects_end = end_string;
    }
    // GCodeWriter.hpp:119  bool empty_object_end_str()
    pub fn empty_object_end_str(&self) -> bool {
        self.m_gcode_label_objects_end.is_empty()
    }

    // GCodeWriter.cpp:928  void GCodeWriter::add_object_start_labels(std::string& gcode)
    pub fn add_object_start_labels(&mut self, gcode: &mut String) {
        // GCodeWriter.cpp:930  if (!m_gcode_label_objects_start.empty())
        if !self.m_gcode_label_objects_start.is_empty() {
            // GCodeWriter.cpp:931
            gcode.push_str(&self.m_gcode_label_objects_start);
            // GCodeWriter.cpp:932
            self.m_gcode_label_objects_start = String::new();
        }
    }

    // GCodeWriter.cpp:936  void GCodeWriter::add_object_end_labels(std::string& gcode)
    pub fn add_object_end_labels(&mut self, gcode: &mut String) {
        // GCodeWriter.cpp:938  if (!m_gcode_label_objects_end.empty())
        if !self.m_gcode_label_objects_end.is_empty() {
            // GCodeWriter.cpp:939
            gcode.push_str(&self.m_gcode_label_objects_end);
            // GCodeWriter.cpp:940
            self.m_gcode_label_objects_end = String::new();
        }
    }

    // GCodeWriter.cpp:944  void GCodeWriter::add_object_change_labels(std::string& gcode)
    pub fn add_object_change_labels(&mut self, gcode: &mut String) {
        // GCodeWriter.cpp:946
        self.add_object_end_labels(gcode);
        // GCodeWriter.cpp:947
        self.add_object_start_labels(gcode);
    }

    // GCodeWriter.cpp:950  std::string GCodeWriter::set_extruder(unsigned int filament_id, unsigned int nozzle_id)
    pub fn set_extruder(&mut self, filament_id: u32, nozzle_id: u32) -> String {
        // GCodeWriter.cpp:952  auto filament_ext_it = ... lower_bound_by_predicate(...)
        let filament_ext_idx = self.lower_bound_filament(filament_id);
        // GCodeWriter.cpp:953  unsigned int extruder_id = nozzle_id>0;
        let _extruder_id: u32 = (nozzle_id > 0) as u32;
        // GCodeWriter.cpp:954  assert(...)
        debug_assert!(
            filament_ext_idx != self.m_filament_extruders.end_index()
                && self.m_filament_extruders[filament_ext_idx].id() == filament_id
        );
        // GCodeWriter.cpp:956  return this->need_toolchange(filament_id) ? this->toolchange(filament_id,nozzle_id) : "";
        if self.need_toolchange(filament_id) {
            self.toolchange(filament_id, nozzle_id)
        } else {
            String::new()
        }
    }

    // GCodeWriter.cpp:959  void GCodeWriter::init_extruder(unsigned int filament_id, unsigned int nozzle_id)
    pub fn init_extruder(&mut self, filament_id: u32, nozzle_id: u32) {
        // GCodeWriter.cpp:961  if (m_curr_extruder_id == -1 && filament_id != -1)
        // NOTE: filament_id is unsigned in C++; the `filament_id != -1` compares against
        // (unsigned)-1 == u32::MAX. Preserved faithfully.
        if self.m_curr_extruder_id == -1 && filament_id != u32::MAX {
            // GCodeWriter.cpp:962  auto filament_extruder_iter = ... lower_bound_by_predicate(...)
            let filament_extruder_idx = self.lower_bound_filament(filament_id);
            // GCodeWriter.cpp:963  assert(...)
            debug_assert!(
                filament_extruder_idx != self.m_filament_extruders.end_index()
                    && self.m_filament_extruders[filament_extruder_idx].id() == filament_id
            );
            // GCodeWriter.cpp:964  m_curr_extruder_id = nozzle_id>0;
            self.m_curr_extruder_id = (nozzle_id > 0) as i32;
            // GCodeWriter.cpp:965  m_curr_filament_extruder[m_curr_extruder_id] = &*filament_extruder_iter;
            self.m_curr_filament_extruder[self.m_curr_extruder_id as usize] =
                Some(filament_extruder_idx);
        }
    }

    // GCodeWriter.cpp:969  bool GCodeWriter::need_toolchange(unsigned int filament_id) const
    pub fn need_toolchange(&self, filament_id: u32) -> bool {
        // GCodeWriter.cpp:971  return filament()==nullptr || filament()->id()!=filament_id;
        match self.filament() {
            None => true,
            Some(f) => f.id() != filament_id,
        }
    }

    // GCodeWriter.hpp:101  Vec3d get_position() const
    pub fn get_position(&self) -> Vec3d {
        self.m_pos
    }
    // GCodeWriter.hpp:102  void set_position(Vec3d& in)
    pub fn set_position(&mut self, in_pos: Vec3d) {
        self.m_pos = in_pos;
    }

    // GCodeWriter.hpp:105  void set_xy_offset(double x, double y)
    pub fn set_xy_offset(&mut self, x: f64, y: f64) {
        self.m_x_offset = x;
        self.m_y_offset = y;
    }
    // GCodeWriter.hpp:106  Vec2f get_xy_offset()
    pub fn get_xy_offset(&self) -> (f32, f32) {
        (self.m_x_offset as f32, self.m_y_offset as f32)
    }

    // GCodeWriter.hpp:125  void set_current_position_clear(bool clear)
    pub fn set_current_position_clear(&mut self, clear: bool) {
        self.m_is_current_pos_clear = clear;
    }
    // GCodeWriter.hpp:126  bool is_current_position_clear() const
    pub fn is_current_position_clear(&self) -> bool {
        self.m_is_current_pos_clear
    }
    // GCodeWriter.hpp:127  void set_is_bbl_printer(bool is_bbl_printer)
    pub fn set_is_bbl_printer(&mut self, is_bbl_printer: bool) {
        self.m_is_bbl_printer = is_bbl_printer;
    }
}

// Helper to mirror std::ostringstream `<<` default formatting of a double in C++
// (no fixed precision, "%g"-like shortest round-trippable). Rust's default {}
// for f64 already prints the shortest round-trip representation, matching this
// for the integral/short-decimal values used here (jerk, fan percentages).
fn ostream_double(v: f64) -> String {
    // C++ ostream prints integral doubles without a trailing ".0" (e.g. 255).
    if v == v.trunc() && v.is_finite() {
        format!("{}", v as i64)
    } else {
        format!("{}", v)
    }
}

// Helper to mirror `std::setprecision(4)` (without std::fixed) on a stream, which
// sets the total significant-digit precision to 4 ("%.4g"-like). Used by
// set_pressure_advance (GCodeWriter.cpp:279-288).
fn format_setprecision_4(v: f64) -> String {
    // std::setprecision sets significant digits (default float format).
    let s = format!("{:.*e}", 3, v); // 4 significant digits in scientific form
    // Convert back from scientific to plain "%g"-style by parsing; for the small
    // positive PA values in use this matches "%.4g".
    let parsed: f64 = s.parse().unwrap_or(v);
    // Print with up to 4 significant digits, trimming as %g does.
    let g = format!("{:.4}", parsed);
    let trimmed = g.trim_end_matches('0').trim_end_matches('.');
    if trimmed.is_empty() {
        "0".to_string()
    } else {
        trimmed.to_string()
    }
}

// Small helper to mirror `iter == container.end()` checks used by the asserts.
trait EndIndex {
    fn end_index(&self) -> usize;
}
impl EndIndex for Vec<Extruder> {
    fn end_index(&self) -> usize {
        self.len()
    }
}

// ===========================================================================
// GCodeFormatter / GCodeG1Formatter / GCodeG2G3Formatter (GCodeWriter.hpp:185-290,
// GCodeWriter.cpp:974-1031)
// ===========================================================================

// GCodeWriter.hpp:201-202
//   static constexpr const int XYZF_EXPORT_DIGITS = 3;
//   static constexpr const int E_EXPORT_DIGITS    = 5;
const XYZF_EXPORT_DIGITS: usize = 3;
const E_EXPORT_DIGITS: usize = 5;

// GCodeWriter.hpp:260  static constexpr const size_t buflen = 256;
const BUFLEN: usize = 256;

// GCodeWriter.hpp:185  class GCodeFormatter
pub struct GCodeFormatter {
    // GCodeWriter.hpp:261  char buf[buflen];
    buf: [u8; BUFLEN],
    // GCodeWriter.hpp:263  std::to_chars_result ptr_err; (we track the write cursor index)
    ptr: usize,
}

impl GCodeFormatter {
    // GCodeWriter.hpp:187-190  GCodeFormatter()
    pub fn new() -> Self {
        // GCodeWriter.hpp:188  this->buf_end = buf + buflen;
        // GCodeWriter.hpp:189  this->ptr_err.ptr = this->buf;
        GCodeFormatter {
            buf: [0u8; BUFLEN],
            ptr: 0,
        }
    }

    // GCodeWriter.cpp:974  void GCodeFormatter::emit_axis(const char axis, const double v, size_t digits)
    pub fn emit_axis(&mut self, axis: u8, v: f64, digits: usize) {
        // GCodeWriter.cpp:975  assert(digits <= 9);
        debug_assert!(digits <= 9);
        // GCodeWriter.cpp:976  static constexpr const std::array<int,10> pow_10{...};
        const POW_10: [i64; 10] = [
            1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000,
        ];
        // GCodeWriter.cpp:977  *ptr_err.ptr++ = ' '; *ptr_err.ptr++ = axis;
        self.buf[self.ptr] = b' ';
        self.ptr += 1;
        self.buf[self.ptr] = axis;
        self.ptr += 1;

        // GCodeWriter.cpp:979  char *base_ptr = this->ptr_err.ptr;
        let base_ptr = self.ptr;
        // GCodeWriter.cpp:980  auto v_int = int64_t(std::round(v * pow_10[digits]));
        let v_int = (v * POW_10[digits] as f64).round() as i64;
        // GCodeWriter.cpp:983-988  write integer digits (to_chars / karma::generate).
        let int_str = v_int.to_string();
        for &b in int_str.as_bytes() {
            self.buf[self.ptr] = b;
            self.ptr += 1;
        }
        // GCodeWriter.cpp:989  size_t writen_digits = (ptr - base_ptr) - (v_int < 0 ? 1 : 0);
        let writen_digits = (self.ptr - base_ptr) - if v_int < 0 { 1 } else { 0 };
        // GCodeWriter.cpp:990  if (writen_digits < digits)
        if writen_digits < digits {
            // GCodeWriter.cpp:992  size_t remaining_digits = digits - writen_digits;
            let remaining_digits = digits - writen_digits;
            // GCodeWriter.cpp:994-995  Move all newly inserted chars by remaining_digits.
            //   for (from_ptr = ptr-1, to_ptr = from_ptr+remaining; from_ptr >= ptr-writen_digits; --to_ptr,--from_ptr) *to_ptr=*from_ptr;
            {
                let mut from_ptr: isize = self.ptr as isize - 1;
                let mut to_ptr: isize = from_ptr + remaining_digits as isize;
                let stop: isize = self.ptr as isize - writen_digits as isize;
                while from_ptr >= stop {
                    self.buf[to_ptr as usize] = self.buf[from_ptr as usize];
                    to_ptr -= 1;
                    from_ptr -= 1;
                }
            }
            // GCodeWriter.cpp:997  memset(ptr - writen_digits, '0', remaining_digits);
            for i in 0..remaining_digits {
                self.buf[self.ptr - writen_digits + i] = b'0';
            }
            // GCodeWriter.cpp:998  ptr += remaining_digits;
            self.ptr += remaining_digits;
        }

        // GCodeWriter.cpp:1002-1003  Move all newly inserted chars by one for a decimal point.
        //   for (to_ptr = ptr, from_ptr = to_ptr-1; from_ptr >= ptr-digits; --to_ptr,--from_ptr) *to_ptr=*from_ptr;
        {
            let mut to_ptr: isize = self.ptr as isize;
            let mut from_ptr: isize = to_ptr - 1;
            let stop: isize = self.ptr as isize - digits as isize;
            while from_ptr >= stop {
                self.buf[to_ptr as usize] = self.buf[from_ptr as usize];
                to_ptr -= 1;
                from_ptr -= 1;
            }
        }

        // GCodeWriter.cpp:1005  *(ptr - digits) = '.';
        self.buf[self.ptr - digits] = b'.';
        // GCodeWriter.cpp:1006-1010  for (i=0;i<digits;++i){ if(*ptr!='0')break; ptr--; }
        for _ in 0..digits {
            if self.buf[self.ptr] != b'0' {
                break;
            }
            self.ptr -= 1;
        }
        // GCodeWriter.cpp:1011-1012  if (*ptr == '.') ptr--;
        if self.buf[self.ptr] == b'.' {
            self.ptr -= 1;
        }
        // GCodeWriter.cpp:1013-1014  if ((ptr+1)==base_ptr || *ptr=='-') *(++ptr) = '0';
        if (self.ptr + 1) == base_ptr || self.buf[self.ptr] == b'-' {
            self.ptr += 1;
            self.buf[self.ptr] = b'0';
        }
        // GCodeWriter.cpp:1015  ptr++;
        self.ptr += 1;
    }

    // GCodeWriter.hpp:214  void emit_xy(const Vec2d &point)
    pub fn emit_xy(&mut self, point: Vec2d) {
        // GCodeWriter.hpp:215
        self.emit_axis(b'X', point.x(), XYZF_EXPORT_DIGITS);
        // GCodeWriter.hpp:216
        self.emit_axis(b'Y', point.y(), XYZF_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:219  void emit_xyz(const Vec3d &point)
    pub fn emit_xyz(&mut self, point: Vec3d) {
        // GCodeWriter.hpp:220
        self.emit_axis(b'X', point.x, XYZF_EXPORT_DIGITS);
        // GCodeWriter.hpp:221
        self.emit_axis(b'Y', point.y, XYZF_EXPORT_DIGITS);
        // GCodeWriter.hpp:222
        self.emit_z(point.z);
    }

    // GCodeWriter.hpp:225  void emit_z(const double z)
    pub fn emit_z(&mut self, z: f64) {
        // GCodeWriter.hpp:226
        self.emit_axis(b'Z', z, XYZF_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:229  void emit_e(double v)
    pub fn emit_e(&mut self, v: f64) {
        // GCodeWriter.hpp:230
        self.emit_axis(b'E', v, E_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:233  void emit_f(double speed)
    pub fn emit_f(&mut self, speed: f64) {
        // GCodeWriter.hpp:234
        self.emit_axis(b'F', speed, XYZF_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:237  void emit_ij(const Vec2d &point)
    pub fn emit_ij(&mut self, point: Vec2d) {
        // GCodeWriter.hpp:238
        self.emit_axis(b'I', point.x(), XYZF_EXPORT_DIGITS);
        // GCodeWriter.hpp:239
        self.emit_axis(b'J', point.y(), XYZF_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:242  void emit_string(const std::string &s)
    pub fn emit_string(&mut self, s: &str) {
        // GCodeWriter.hpp:243-244  strncpy(ptr, s.c_str(), s.size()); ptr += s.size();
        for &b in s.as_bytes() {
            self.buf[self.ptr] = b;
            self.ptr += 1;
        }
    }

    // GCodeWriter.hpp:247  void emit_comment(bool allow_comments, const std::string &comment)
    pub fn emit_comment(&mut self, allow_comments: bool, comment: &str) {
        // GCodeWriter.hpp:248  if (allow_comments && ! comment.empty())
        if allow_comments && !comment.is_empty() {
            // GCodeWriter.hpp:249  *ptr++ = ' '; *ptr++ = ';'; *ptr++ = ' ';
            self.buf[self.ptr] = b' ';
            self.ptr += 1;
            self.buf[self.ptr] = b';';
            self.ptr += 1;
            self.buf[self.ptr] = b' ';
            self.ptr += 1;
            // GCodeWriter.hpp:250
            self.emit_string(comment);
        }
    }

    // GCodeWriter.hpp:254  std::string string()
    pub fn string(&mut self) -> String {
        // GCodeWriter.hpp:255  *ptr++ = '\n';
        self.buf[self.ptr] = b'\n';
        self.ptr += 1;
        // GCodeWriter.hpp:256  return std::string(this->buf, ptr - buf);
        String::from_utf8_lossy(&self.buf[0..self.ptr]).into_owned()
    }
}

impl Default for GCodeFormatter {
    fn default() -> Self {
        Self::new()
    }
}

// GCodeWriter.hpp:266  class GCodeG1Formatter : public GCodeFormatter
pub struct GCodeG1Formatter {
    inner: GCodeFormatter,
}

impl GCodeG1Formatter {
    // GCodeWriter.hpp:268-273  GCodeG1Formatter()
    pub fn new() -> Self {
        let mut inner = GCodeFormatter::new();
        // GCodeWriter.hpp:269  this->buf[0] = 'G';
        inner.buf[0] = b'G';
        // GCodeWriter.hpp:270  this->buf[1] = '1';
        inner.buf[1] = b'1';
        // GCodeWriter.hpp:272  this->ptr_err.ptr = this->buf + 2;
        inner.ptr = 2;
        GCodeG1Formatter { inner }
    }
    pub fn emit_xy(&mut self, point: Vec2d) {
        self.inner.emit_xy(point)
    }
    pub fn emit_xyz(&mut self, point: Vec3d) {
        self.inner.emit_xyz(point)
    }
    pub fn emit_z(&mut self, z: f64) {
        self.inner.emit_z(z)
    }
    pub fn emit_e(&mut self, v: f64) {
        self.inner.emit_e(v)
    }
    pub fn emit_f(&mut self, speed: f64) {
        self.inner.emit_f(speed)
    }
    pub fn emit_string(&mut self, s: &str) {
        self.inner.emit_string(s)
    }
    pub fn emit_comment(&mut self, allow_comments: bool, comment: &str) {
        self.inner.emit_comment(allow_comments, comment)
    }
    pub fn string(&mut self) -> String {
        self.inner.string()
    }
}

impl Default for GCodeG1Formatter {
    fn default() -> Self {
        Self::new()
    }
}

// GCodeWriter.hpp:279  class GCodeG2G3Formatter : public GCodeFormatter
pub struct GCodeG2G3Formatter {
    inner: GCodeFormatter,
}

impl GCodeG2G3Formatter {
    // GCodeWriter.hpp:281-286  GCodeG2G3Formatter(bool is_ccw)
    pub fn new(is_ccw: bool) -> Self {
        let mut inner = GCodeFormatter::new();
        // GCodeWriter.hpp:282  this->buf[0] = 'G';
        inner.buf[0] = b'G';
        // GCodeWriter.hpp:283  this->buf[1] = is_ccw ? '3' : '2';
        inner.buf[1] = if is_ccw { b'3' } else { b'2' };
        // GCodeWriter.hpp:285  this->ptr_err.ptr = this->buf + 2;
        inner.ptr = 2;
        GCodeG2G3Formatter { inner }
    }
    pub fn emit_xy(&mut self, point: Vec2d) {
        self.inner.emit_xy(point)
    }
    pub fn emit_z(&mut self, z: f64) {
        self.inner.emit_z(z)
    }
    pub fn emit_e(&mut self, v: f64) {
        self.inner.emit_e(v)
    }
    pub fn emit_f(&mut self, speed: f64) {
        self.inner.emit_f(speed)
    }
    pub fn emit_ij(&mut self, point: Vec2d) {
        self.inner.emit_ij(point)
    }
    pub fn emit_string(&mut self, s: &str) {
        self.inner.emit_string(s)
    }
    pub fn emit_comment(&mut self, allow_comments: bool, comment: &str) {
        self.inner.emit_comment(allow_comments, comment)
    }
    pub fn string(&mut self) -> String {
        self.inner.string()
    }
}
