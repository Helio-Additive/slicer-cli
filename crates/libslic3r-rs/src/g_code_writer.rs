//! G-code writer.
//!
//! Line-by-line port of `GCodeWriter.cpp` / `GCodeWriter.hpp` (Bambu Studio src/libslic3r at the
//! pin). `coord_t` -> `i64`, `coordf_t` -> `f64`. The control flow, constants, rounding and edge
//! cases follow the C++ source. wasm-safe: no system/dylib dependencies.
//!
//! The C++ `GCodeWriter::config` is a `GCodeConfig`; this port's `GCodeConfig`
//! (`crate::extruder::GCodeConfig`) holds the members the writer and the `Extruder` read, with
//! `ConfigOptionVector`-faithful `get_at` semantics, and `apply` / `apply_only` as `ConfigBase`
//! does them (Config.hpp:2584, Config.cpp:454-493).
//!
//! C++ overloads that share a name are, past the first, methods of the small traits next to the
//! struct (`GCodeWriterFilamentMut`, `GCodeWriterTravelToXyShort`, ...), each named after the C++
//! function it ports.

// GCodeWriter.cpp:1   #include "GCodeWriter.hpp"
// GCodeWriter.cpp:2   #include "CustomGCode.hpp"
use crate::extruder::{get_process_config_idx, Extruder, GCodeConfig};
use crate::geometry::{Vec2d, Vec3d};
use crate::libslic3r::EPSILON;
use crate::print_config::GCodeFlavor;
use std::f64::consts::PI;
use std::fmt::Write as _;

// GCodeWriter.cpp:13   #define FLAVOR_IS(val) this->config.gcode_flavor == val
// GCodeWriter.cpp:14   #define FLAVOR_IS_NOT(val) this->config.gcode_flavor != val
// Implemented inline as `self.config.gcode_flavor == GCodeFlavor::...`.

// GCodeWriter.hpp:15-19   enum class LiftType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiftType {
    // GCodeWriter.hpp:16
    NormalLift,
    // GCodeWriter.hpp:17
    SlopeLift,
    // GCodeWriter.hpp:18
    SpiralLift,
}

// GCodeWriter.hpp:21-24   enum class TemperatureCommandType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TemperatureCommandType {
    // GCodeWriter.hpp:22
    Default,
    // GCodeWriter.hpp:23
    LayerChange,
}

// GCodeWriter.hpp:26   class GCodeWriter
pub struct GCodeWriter {
    // GCodeWriter.hpp:28  GCodeConfig config;
    // FIDELITY-NOTE: boxed so that its address, which every `Extruder` of the writer keeps
    // (`Extruder(extruder_id, &this->config, ...)`, GCodeWriter.cpp:78), stays the same when the
    // writer is moved; the C++ writer is not moved once its extruders are set.
    pub config: Box<GCodeConfig>,
    // GCodeWriter.hpp:29  bool multiple_extruders;
    pub multiple_extruders: bool,

    // GCodeWriter.hpp:153  std::vector<Extruder> m_filament_extruders;
    // Extruders are sorted by their ID, so that binary search is possible.
    m_filament_extruders: Vec<Extruder>,
    // GCodeWriter.hpp:154  bool m_single_extruder_multi_material;
    m_single_extruder_multi_material: bool,
    // GCodeWriter.hpp:155  std::vector<Extruder*> m_curr_filament_extruder;
    // The C++ holds pointers into m_filament_extruders; this port holds indices into it.
    m_curr_filament_extruder: [Option<usize>; 2],
    // GCodeWriter.hpp:156  int m_curr_extruder_id;
    m_curr_extruder_id: i32,
    // GCodeWriter.hpp:157  size_t m_current_process_config_idx;
    m_current_process_config_idx: usize,
    // GCodeWriter.hpp:158  unsigned int m_last_acceleration;
    m_last_acceleration: u32,
    // GCodeWriter.hpp:159-161  unsigned int m_max_acceleration;
    // Limit for setting the acceleration, to respect the machine limits set for the Marlin firmware.
    // If set to zero, the limit is not in action.
    m_max_acceleration: u32,
    // GCodeWriter.hpp:162  double m_last_jerk;
    m_last_jerk: f64,
    // GCodeWriter.hpp:163  double m_max_jerk;
    m_max_jerk: f64,
    // GCodeWriter.hpp:165  unsigned int m_last_additional_fan_speed;
    #[allow(dead_code)]
    m_last_additional_fan_speed: u32,
    // GCodeWriter.hpp:166  int m_last_bed_temperature;
    // FIDELITY-NOTE: the C++ constructor leaves it uninitialized (GCodeWriter.hpp:38 comments it
    // out of the initializer list); 0 here.
    m_last_bed_temperature: i32,
    // GCodeWriter.hpp:167  bool m_last_bed_temperature_reached;
    m_last_bed_temperature_reached: bool,
    // GCodeWriter.hpp:168  double m_lifted;
    m_lifted: f64,
    // GCodeWriter.hpp:171  double m_to_lift;
    m_to_lift: f64,
    // GCodeWriter.hpp:172  LiftType m_to_lift_type;
    m_to_lift_type: LiftType,
    // GCodeWriter.hpp:173  Vec3d m_pos = Vec3d::Zero();
    m_pos: Vec3d,
    // GCodeWriter.hpp:174-177  bool m_is_current_pos_clear = false;
    m_is_current_pos_clear: bool,
    // GCodeWriter.hpp:178  bool m_avoid_z_descent_travel = false;
    m_avoid_z_descent_travel: bool,
    // GCodeWriter.hpp:180  double m_x_offset{ 0 };
    m_x_offset: f64,
    // GCodeWriter.hpp:181  double m_y_offset{ 0 };
    m_y_offset: f64,
    // GCodeWriter.hpp:183  Vec2d m_bed_min{ 0., 0. };
    m_bed_min: Vec2d,
    // GCodeWriter.hpp:184  Vec2d m_bed_max{ 0., 0. };
    m_bed_max: Vec2d,
    // GCodeWriter.hpp:185  bool m_bed_bbox_valid{ false };
    m_bed_bbox_valid: bool,
    // GCodeWriter.hpp:189  std::vector<Vec2d> m_extruder_bed_min;
    m_extruder_bed_min: Vec<Vec2d>,
    // GCodeWriter.hpp:190  std::vector<Vec2d> m_extruder_bed_max;
    m_extruder_bed_max: Vec<Vec2d>,
    // GCodeWriter.hpp:191  std::vector<char> m_extruder_bed_valid;
    m_extruder_bed_valid: Vec<u8>,
    // GCodeWriter.hpp:192  double m_current_speed{ 0 };
    m_current_speed: f64,
    // GCodeWriter.hpp:193  bool m_is_bbl_printer = false;
    m_is_bbl_printer: bool,
    // GCodeWriter.hpp:195  std::string m_gcode_label_objects_start;
    m_gcode_label_objects_start: String,
    // GCodeWriter.hpp:196  std::string m_gcode_label_objects_end;
    m_gcode_label_objects_end: String,
    // GCodeWriter.hpp:198  bool m_is_first_layer{false};
    m_is_first_layer: bool,
    // GCodeWriter.hpp:199  unsigned int m_acceleration{0};
    m_acceleration: u32,
    // GCodeWriter.hpp:200  std::vector<unsigned int> m_travel_accelerations;
    m_travel_accelerations: Vec<u32>,
    // GCodeWriter.hpp:201  std::vector<unsigned int> m_travel_short_accelerations;
    m_travel_short_accelerations: Vec<u32>,
    // GCodeWriter.hpp:202  std::vector<unsigned int> m_first_layer_travel_accelerations;
    m_first_layer_travel_accelerations: Vec<u32>,
}

// GCodeWriter.hpp:139  static const bool full_gcode_comment;
// GCodeWriter.cpp:18    const bool GCodeWriter::full_gcode_comment = false;
pub const FULL_GCODE_COMMENT: bool = false;

// GCodeWriter.hpp:141  static const double slope_threshold;
// GCodeWriter.cpp:19    const double GCodeWriter::slope_threshold = 3 * PI / 180;
pub const SLOPE_THRESHOLD: f64 = 3.0 * PI / 180.0;

impl Default for GCodeWriter {
    fn default() -> Self {
        Self::new()
    }
}

/// The option `key` of the `PrintConfig` the writer applies: the configuration's own, or the
/// default of its definition when the configuration does not hold it (the C++ `PrintConfig` is a
/// static class that holds every member).
fn print_config_option<'c>(
    print_config: &'c crate::print_config::config_def::DynamicPrintConfig,
    key: &str,
) -> &'c dyn crate::config::ConfigOption {
    use crate::config::ConfigOptionResolver;
    match print_config.option(key) {
        Some(o) => o,
        None => crate::print_config::config_def::static_print_config_defaults("PrintConfig")
            .option(key)
            .unwrap_or_else(|| panic!("PrintConfig has no member {}", key)),
    }
}

/// `ConfigOptionFloats::values` of a member of the `PrintConfig`.
fn print_config_floats(print_config: &crate::print_config::config_def::DynamicPrintConfig, key: &str) -> Vec<f64> {
    print_config_option(print_config, key)
        .vector_values_any()
        .and_then(|a| a.downcast_ref::<Vec<f64>>())
        .cloned()
        .unwrap_or_else(|| panic!("PrintConfig::{} is not a ConfigOptionFloats", key))
}

impl GCodeWriter {
    // GCodeWriter.hpp:31-42  GCodeWriter() :
    pub fn new() -> Self {
        GCodeWriter {
            // GCodeWriter.hpp:28  GCodeConfig config; (its default constructor: the defaults of the
            // definitions)
            config: Box::new(GCodeConfig::new()),
            // GCodeWriter.hpp:32  multiple_extruders(false)
            multiple_extruders: false,
            m_filament_extruders: Vec::new(),
            // GCodeWriter.hpp:35  m_single_extruder_multi_material(false)
            m_single_extruder_multi_material: false,
            // GCodeWriter.hpp:32  m_curr_filament_extruder{ nullptr,nullptr }
            m_curr_filament_extruder: [None, None],
            // GCodeWriter.hpp:33  m_curr_extruder_id (-1)
            m_curr_extruder_id: -1,
            // GCodeWriter.hpp:34  m_current_process_config_idx(0)
            m_current_process_config_idx: 0,
            // GCodeWriter.hpp:36  m_last_acceleration(0), m_max_acceleration(0)
            m_last_acceleration: 0,
            m_max_acceleration: 0,
            // GCodeWriter.hpp:37  m_last_jerk(0), m_max_jerk(0)
            m_last_jerk: 0.0,
            m_max_jerk: 0.0,
            m_last_additional_fan_speed: 0,
            m_last_bed_temperature: 0,
            // GCodeWriter.hpp:38  m_last_bed_temperature_reached(true)
            m_last_bed_temperature_reached: true,
            // GCodeWriter.hpp:39  m_lifted(0)
            m_lifted: 0.0,
            // GCodeWriter.hpp:40  m_to_lift(0)
            m_to_lift: 0.0,
            // GCodeWriter.hpp:41  m_to_lift_type(LiftType::NormalLift)
            m_to_lift_type: LiftType::NormalLift,
            // GCodeWriter.hpp:173  m_pos = Vec3d::Zero()
            m_pos: Vec3d::zero(),
            // GCodeWriter.hpp:177  m_is_current_pos_clear = false
            m_is_current_pos_clear: false,
            // GCodeWriter.hpp:178  m_avoid_z_descent_travel = false
            m_avoid_z_descent_travel: false,
            // GCodeWriter.hpp:180-181  m_x_offset{0}; m_y_offset{0}
            m_x_offset: 0.0,
            m_y_offset: 0.0,
            // GCodeWriter.hpp:183-185
            m_bed_min: Vec2d::new(0.0, 0.0),
            m_bed_max: Vec2d::new(0.0, 0.0),
            m_bed_bbox_valid: false,
            m_extruder_bed_min: Vec::new(),
            m_extruder_bed_max: Vec::new(),
            m_extruder_bed_valid: Vec::new(),
            // GCodeWriter.hpp:192  m_current_speed{0}
            m_current_speed: 0.0,
            // GCodeWriter.hpp:193  m_is_bbl_printer = false
            m_is_bbl_printer: false,
            m_gcode_label_objects_start: String::new(),
            m_gcode_label_objects_end: String::new(),
            // GCodeWriter.hpp:198  m_is_first_layer{false}
            m_is_first_layer: false,
            // GCodeWriter.hpp:199  m_acceleration{0}
            m_acceleration: 0,
            m_travel_accelerations: Vec::new(),
            m_travel_short_accelerations: Vec::new(),
            m_first_layer_travel_accelerations: Vec::new(),
        }
    }

    // GCodeWriter.hpp:43  Extruder* filament(size_t extruder_id)
    pub fn filament_at(&self, extruder_id: usize) -> Option<&Extruder> {
        <GCodeWriter as GCodeWriterFilamentAt>::filament(self, extruder_id)
    }

    // GCodeWriter.hpp:46  const Extruder* filament() const
    pub fn filament(&self) -> Option<&Extruder> {
        // if(m_curr_extruder_id==-1) return nullptr;
        if self.m_curr_extruder_id == -1 {
            return None;
        }
        // return m_curr_filament_extruder[m_curr_extruder_id];
        self.m_curr_filament_extruder[self.m_curr_extruder_id as usize].map(|i| &self.m_filament_extruders[i])
    }

    // GCodeWriter.hpp:45  Extruder* filament(): the mutable one, where the C++ calls
    // filament()->extrude() / retract().
    fn filament_mut(&mut self) -> Option<&mut Extruder> {
        <GCodeWriter as GCodeWriterFilamentMut>::filament(self)
    }

    // GCodeWriter.hpp:48  int get_curr_extruder_id() const
    pub fn get_curr_extruder_id(&self) -> i32 {
        self.m_curr_extruder_id
    }

    // GCodeWriter.cpp:21-67  void GCodeWriter::apply_print_config(const PrintConfig &print_config)
    //
    // FIDELITY-NOTE: the C++ takes the print's static `PrintConfig`; this port takes the
    // configuration as a `DynamicPrintConfig` (the members it does not hold read as the defaults of
    // their definitions). `this->config.apply(print_config, true)` copies the members of
    // `GCodeConfig` the configuration holds, as it does for the static class.
    pub fn apply_print_config(&mut self, print_config: &crate::print_config::config_def::DynamicPrintConfig) {
        use crate::config::{ConfigOptionPoints, ConfigOptionPointsGroups};
        // GCodeWriter.cpp:23
        self.config
            .apply(print_config, true)
            .expect("GCodeWriter::apply_print_config(): config.apply(print_config, true)");
        // GCodeWriter.cpp:24-25  BBS: cache the bed printable-area bounding box so spiral lift can
        // keep its arc inside the bed.
        self.m_bed_bbox_valid = false;
        // GCodeWriter.cpp:26
        let bed_pts: Vec<Vec2d> = print_config_option(print_config, "printable_area")
            .as_any()
            .downcast_ref::<ConfigOptionPoints>()
            .expect("PrintConfig::printable_area is a ConfigOptionPoints")
            .values
            .clone();
        // GCodeWriter.cpp:27
        if bed_pts.len() >= 3 {
            // GCodeWriter.cpp:28-29
            let mut bmin = bed_pts[0];
            let mut bmax = bed_pts[0];
            // GCodeWriter.cpp:30-33
            for p in &bed_pts {
                bmin = Vec2d::new(bmin.x.min(p.x), bmin.y.min(p.y));
                bmax = Vec2d::new(bmax.x.max(p.x), bmax.y.max(p.y));
            }
            // GCodeWriter.cpp:34-36
            self.m_bed_min = bmin;
            self.m_bed_max = bmax;
            self.m_bed_bbox_valid = true;
        }
        // GCodeWriter.cpp:38-42  BBS: cache each physical nozzle's own printable-area bounding box.
        self.m_extruder_bed_min.clear();
        self.m_extruder_bed_max.clear();
        self.m_extruder_bed_valid.clear();
        // GCodeWriter.cpp:43
        let ext_areas: Vec<Vec<Vec2d>> = print_config_option(print_config, "extruder_printable_area")
            .as_any()
            .downcast_ref::<ConfigOptionPointsGroups>()
            .expect("PrintConfig::extruder_printable_area is a ConfigOptionPointsGroups")
            .values
            .clone();
        // GCodeWriter.cpp:44-46
        self.m_extruder_bed_min.resize(ext_areas.len(), Vec2d::new(0.0, 0.0));
        self.m_extruder_bed_max.resize(ext_areas.len(), Vec2d::new(0.0, 0.0));
        self.m_extruder_bed_valid.resize(ext_areas.len(), 0);
        // GCodeWriter.cpp:47
        for (i, pts) in ext_areas.iter().enumerate() {
            // GCodeWriter.cpp:49-50
            if pts.len() < 3 {
                continue;
            }
            // GCodeWriter.cpp:51-52
            let mut bmin = pts[0];
            let mut bmax = pts[0];
            // GCodeWriter.cpp:53-56
            for p in pts {
                bmin = Vec2d::new(bmin.x.min(p.x), bmin.y.min(p.y));
                bmax = Vec2d::new(bmax.x.max(p.x), bmax.y.max(p.y));
            }
            // GCodeWriter.cpp:57-59
            self.m_extruder_bed_min[i] = bmin;
            self.m_extruder_bed_max[i] = bmax;
            self.m_extruder_bed_valid[i] = 1;
        }
        // GCodeWriter.cpp:61
        self.m_single_extruder_multi_material = print_config_option(print_config, "single_extruder_multi_material")
            .get_bool()
            .expect("PrintConfig::single_extruder_multi_material");
        // GCodeWriter.cpp:62-64
        let gcode_flavor = crate::extruder::gcode_flavor_from_int(
            print_config_option(print_config, "gcode_flavor").get_int().expect("PrintConfig::gcode_flavor"),
        );
        let is_marlin = gcode_flavor == GCodeFlavor::MarlinLegacy
            || gcode_flavor == GCodeFlavor::Marlin
            || gcode_flavor == GCodeFlavor::Klipper;
        // GCodeWriter.cpp:65  m_max_acceleration = std::lrint(is_marlin ? print_config.machine_max_acceleration_extruding.values.front() : 0);
        let max_acc = if is_marlin {
            print_config_floats(print_config, "machine_max_acceleration_extruding")[0]
        } else {
            0.0
        };
        self.m_max_acceleration = max_acc.round_ties_even() as i64 as u32;
        // GCodeWriter.cpp:66  m_max_jerk = std::lrint(is_marlin ? std::min(print_config.machine_max_jerk_x.values.front(), print_config.machine_max_jerk_y.values.front()) : 0);
        let max_jerk = if is_marlin {
            let jx = print_config_floats(print_config, "machine_max_jerk_x")[0];
            let jy = print_config_floats(print_config, "machine_max_jerk_y")[0];
            // std::min(a, b): b < a ? b : a
            if jy < jx {
                jy
            } else {
                jx
            }
        } else {
            0.0
        };
        self.m_max_jerk = max_jerk.round_ties_even() as i64 as f64;
    }

    // GCodeWriter.cpp:69-86  void GCodeWriter::set_extruders(std::vector<unsigned int> extruder_ids)
    pub fn set_extruders(&mut self, mut extruder_ids: Vec<u32>) {
        // GCodeWriter.cpp:71
        extruder_ids.sort();
        // GCodeWriter.cpp:72
        self.m_curr_extruder_id = -1;
        // GCodeWriter.cpp:73
        self.m_current_process_config_idx = 0;
        // GCodeWriter.cpp:74  std::fill(m_curr_filament_extruder.begin(), m_curr_filament_extruder.end(), nullptr);
        for e in self.m_curr_filament_extruder.iter_mut() {
            *e = None;
        }
        // GCodeWriter.cpp:75
        self.m_filament_extruders.clear();
        // GCodeWriter.cpp:76
        self.m_filament_extruders.reserve(extruder_ids.len());
        // GCodeWriter.cpp:77-78  for (unsigned int extruder_id : extruder_ids)
        //     m_filament_extruders.emplace_back(Extruder(extruder_id, &this->config, config.single_extruder_multi_material.value));
        let config_ptr: *const GCodeConfig = &*self.config;
        for &extruder_id in &extruder_ids {
            let e = Extruder::new(extruder_id, config_ptr, self.config.single_extruder_multi_material);
            self.m_filament_extruders.push(e);
        }

        // GCodeWriter.cpp:80-83  we enable support for multiple extruder if any extruder greater
        // than 0 is used (even if prints only uses that one) since we need to output Tx commands
        // first extruder has index 0
        self.multiple_extruders = *extruder_ids.iter().max().unwrap() > 0;

        // GCodeWriter.cpp:85
        self.update_exclusive_nozzles();
    }

    // GCodeWriter.cpp:88-98  void GCodeWriter::update_exclusive_nozzles()
    pub fn update_exclusive_nozzles(&mut self) {
        // GCodeWriter.cpp:90
        let nozzle_map = self.config.filament_nozzle_map.values.clone();
        // GCodeWriter.cpp:91  std::map<int, int> filaments_per_nozzle;
        let mut filaments_per_nozzle: std::collections::BTreeMap<i32, i32> = std::collections::BTreeMap::new();
        // GCodeWriter.cpp:92-94
        for e in &self.m_filament_extruders {
            if (e.id() as usize) < nozzle_map.len() {
                *filaments_per_nozzle.entry(nozzle_map[e.id() as usize]).or_insert(0) += 1;
            }
        }

        // GCodeWriter.cpp:96-97
        for e in self.m_filament_extruders.iter_mut() {
            let exclusive = (e.id() as usize) < nozzle_map.len()
                && *filaments_per_nozzle.entry(nozzle_map[e.id() as usize]).or_insert(0) == 1;
            e.set_exclusive_nozzle(exclusive);
        }
    }

    // GCodeWriter.hpp:55  const std::vector<Extruder>& extruders() const
    pub fn extruders(&self) -> &Vec<Extruder> {
        &self.m_filament_extruders
    }

    // GCodeWriter.hpp:56-62  std::vector<unsigned int> extruder_ids() const
    pub fn extruder_ids(&self) -> Vec<u32> {
        // GCodeWriter.hpp:57
        let mut out: Vec<u32> = Vec::new();
        // GCodeWriter.hpp:58
        out.reserve(self.m_filament_extruders.len());
        // GCodeWriter.hpp:59-60
        for e in &self.m_filament_extruders {
            out.push(e.id());
        }
        // GCodeWriter.hpp:61
        out
    }

    // GCodeWriter.cpp:100-126  std::string GCodeWriter::preamble()
    pub fn preamble(&mut self) -> String {
        // GCodeWriter.cpp:102
        let mut gcode = String::new();

        // GCodeWriter.cpp:104-107
        if self.config.gcode_flavor != GCodeFlavor::MakerWare {
            gcode.push_str("G90\n");
            gcode.push_str("G21\n");
        }
        // GCodeWriter.cpp:108-116
        if self.config.gcode_flavor == GCodeFlavor::RepRapSprinter
            || self.config.gcode_flavor == GCodeFlavor::RepRapFirmware
            || self.config.gcode_flavor == GCodeFlavor::MarlinLegacy
            || self.config.gcode_flavor == GCodeFlavor::Marlin
            || self.config.gcode_flavor == GCodeFlavor::Teacup
            || self.config.gcode_flavor == GCodeFlavor::Repetier
            || self.config.gcode_flavor == GCodeFlavor::Smoothie
            || self.config.gcode_flavor == GCodeFlavor::Klipper
        {
            // GCodeWriter.cpp:117-121
            if self.config.use_relative_e_distances {
                gcode.push_str("M83 ; use relative distances for extrusion\n");
            } else {
                gcode.push_str("M82 ; use absolute distances for extrusion\n");
            }
            // GCodeWriter.cpp:122
            gcode.push_str(&self.reset_e(true));
        }

        // GCodeWriter.cpp:125
        gcode
    }

    // GCodeWriter.cpp:128-134  std::string GCodeWriter::postamble() const
    pub fn postamble(&self) -> String {
        // GCodeWriter.cpp:130
        let mut gcode = String::new();
        // GCodeWriter.cpp:131-132
        if self.config.gcode_flavor == GCodeFlavor::Machinekit {
            gcode.push_str("M2 ; end of program\n");
        }
        // GCodeWriter.cpp:133
        gcode
    }

    // GCodeWriter.cpp:136-180  std::string GCodeWriter::set_temperature(unsigned int temperature,
    //     bool wait, int tool, TemperatureCommandType command_type) const
    pub fn set_temperature(&self, temperature: u32, wait: bool, tool: i32, command_type: TemperatureCommandType) -> String {
        // GCodeWriter.cpp:139-140
        if wait
            && (self.config.gcode_flavor == GCodeFlavor::MakerWare || self.config.gcode_flavor == GCodeFlavor::Sailfish)
        {
            return String::new();
        }

        // GCodeWriter.cpp:142  std::string code, comment;
        let code: &str;
        let comment: &str;
        // GCodeWriter.cpp:143
        if wait
            && self.config.gcode_flavor != GCodeFlavor::Teacup
            && self.config.gcode_flavor != GCodeFlavor::RepRapFirmware
        {
            // GCodeWriter.cpp:144-145
            code = "M109";
            comment = "set nozzle temperature and wait for it to be reached";
        } else {
            // GCodeWriter.cpp:147-151  M104 is deprecated on RepRapFirmware
            if self.config.gcode_flavor == GCodeFlavor::RepRapFirmware {
                code = "G10";
            } else {
                code = "M104";
            }
            // GCodeWriter.cpp:152
            comment = "set nozzle temperature";
        }

        // GCodeWriter.cpp:155-156
        let mut gcode = String::new();
        gcode.push_str(code);
        // GCodeWriter.cpp:157-158
        if command_type == TemperatureCommandType::LayerChange && code == "M104" {
            gcode.push_str(" M");
        }
        // GCodeWriter.cpp:159
        gcode.push(' ');
        // GCodeWriter.cpp:160-164
        if self.config.gcode_flavor == GCodeFlavor::Mach3 || self.config.gcode_flavor == GCodeFlavor::Machinekit {
            gcode.push('P');
        } else {
            gcode.push('S');
        }
        // GCodeWriter.cpp:165
        let _ = write!(gcode, "{}", temperature);
        // GCodeWriter.cpp:166
        let multiple_tools = self.multiple_extruders && !self.m_single_extruder_multi_material;
        // GCodeWriter.cpp:167
        if tool != -1
            && (multiple_tools
                || self.config.gcode_flavor == GCodeFlavor::MakerWare
                || self.config.gcode_flavor == GCodeFlavor::Sailfish)
        {
            // GCodeWriter.cpp:168-172
            if self.config.gcode_flavor == GCodeFlavor::RepRapFirmware {
                let _ = write!(gcode, " P{}", tool);
            } else {
                let _ = write!(gcode, " T{}", tool);
            }
        }
        // GCodeWriter.cpp:174
        let _ = write!(gcode, " ; {}\n", comment);

        // GCodeWriter.cpp:176-177
        if (self.config.gcode_flavor == GCodeFlavor::Teacup || self.config.gcode_flavor == GCodeFlavor::RepRapFirmware)
            && wait
        {
            gcode.push_str("M116 ; wait for temperature to be reached\n");
        }

        // GCodeWriter.cpp:179
        gcode
    }

    // GCodeWriter.cpp:182-205  std::string GCodeWriter::set_bed_temperature(int temperature, bool wait)
    pub fn set_bed_temperature(&mut self, temperature: i32, wait: bool) -> String {
        // GCodeWriter.cpp:185-186
        if temperature == self.m_last_bed_temperature && (!wait || self.m_last_bed_temperature_reached) {
            return String::new();
        }

        // GCodeWriter.cpp:188-189
        self.m_last_bed_temperature = temperature;
        self.m_last_bed_temperature_reached = wait;

        // GCodeWriter.cpp:191-192
        let code: &str;
        let comment: &str;
        let mut gcode = String::new();

        // GCodeWriter.cpp:194-201
        if wait {
            code = "M190";
            comment = "set bed temperature and wait for it to be reached";
        } else {
            code = "M140";
            comment = "set bed temperature";
        }

        // GCodeWriter.cpp:203
        let _ = write!(gcode, "{} S{} ; {}\n", code, temperature, comment);
        // GCodeWriter.cpp:204
        gcode
    }

    // GCodeWriter.cpp:207-224  std::string GCodeWriter::set_chamber_temperature(int temperature, bool wait)
    pub fn set_chamber_temperature(&mut self, temperature: i32, wait: bool) -> String {
        // GCodeWriter.cpp:209-210
        let code: &str;
        let comment: &str;
        let mut gcode = String::new();

        // GCodeWriter.cpp:212
        if wait {
            // GCodeWriter.cpp:214
            gcode.push_str("M106 P2 S255 \n");
            // GCodeWriter.cpp:215
            let _ = write!(gcode, "M191 S{} ;set chamber_temperature and wait for it to be reached\n", temperature);
            // GCodeWriter.cpp:216
            gcode.push_str("M106 P2 S0 \n");
        } else {
            // GCodeWriter.cpp:219-221
            code = "M141";
            comment = "set chamber_temperature";
            let _ = write!(gcode, "{} S{};{}\n", code, temperature, comment);
        }
        // GCodeWriter.cpp:223
        gcode
    }

    // GCodeWriter.cpp:226-229  void GCodeWriter::set_acceleration(unsigned int acceleration)
    pub fn set_acceleration(&mut self, acceleration: u32) {
        self.m_acceleration = acceleration;
    }

    // GCodeWriter.cpp:231-234  void GCodeWriter::set_travel_acceleration(const std::vector<unsigned int>& accelerations)
    pub fn set_travel_acceleration(&mut self, accelerations: &[u32]) {
        self.m_travel_accelerations = accelerations.to_vec();
    }

    // GCodeWriter.cpp:236-239  void GCodeWriter::set_travel_short_acceleration(const std::vector<unsigned int>& accelerations)
    pub fn set_travel_short_acceleration(&mut self, accelerations: &[u32]) {
        self.m_travel_short_accelerations = accelerations.to_vec();
    }

    // GCodeWriter.cpp:241-244  void GCodeWriter::reset_last_acceleration()
    pub fn reset_last_acceleration(&mut self) {
        self.m_last_acceleration = 0;
    }

    // GCodeWriter.hpp:73  std::vector<unsigned int> &get_travel_acceleration()
    pub fn get_travel_acceleration(&mut self) -> &mut Vec<u32> {
        &mut self.m_travel_accelerations
    }

    // GCodeWriter.hpp:74  std::vector<unsigned int> &get_travel_short_acceleration()
    pub fn get_travel_short_acceleration(&mut self) -> &mut Vec<u32> {
        &mut self.m_travel_short_accelerations
    }

    // GCodeWriter.cpp:246-249  void GCodeWriter::set_first_layer_travel_acceleration(const std::vector<unsigned int> &travel_accelerations)
    pub fn set_first_layer_travel_acceleration(&mut self, travel_accelerations: &[u32]) {
        self.m_first_layer_travel_accelerations = travel_accelerations.to_vec();
    }

    // GCodeWriter.cpp:251-254  void GCodeWriter::set_first_layer(bool is_first_layer)
    pub fn set_first_layer(&mut self, is_first_layer: bool) {
        self.m_is_first_layer = is_first_layer;
    }

    // GCodeWriter.cpp:256-259  std::string GCodeWriter::set_extrude_acceleration()
    fn set_extrude_acceleration(&mut self) -> String {
        self.set_acceleration_impl(self.m_acceleration)
    }

    // GCodeWriter.cpp:288-329  std::string GCodeWriter::set_acceleration_impl(unsigned int acceleration)
    fn set_acceleration_impl(&mut self, mut acceleration: u32) -> String {
        // GCodeWriter.cpp:290-292  Clamp the acceleration to the allowed maximum.
        if self.m_max_acceleration > 0 && acceleration > self.m_max_acceleration {
            acceleration = self.m_max_acceleration;
        }

        // GCodeWriter.cpp:294-295
        if acceleration == 0 || acceleration == self.m_last_acceleration {
            return String::new();
        }

        // GCodeWriter.cpp:297
        self.m_last_acceleration = acceleration;

        // GCodeWriter.cpp:299
        let mut gcode = String::new();
        // GCodeWriter.cpp:300
        if self.config.gcode_flavor == GCodeFlavor::Repetier {
            // GCodeWriter.cpp:301-302  M201: Set max printing acceleration
            let _ = write!(gcode, "M201 X{} Y{}", acceleration, acceleration);
            // GCodeWriter.cpp:304
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; adjust acceleration");
            }
            // GCodeWriter.cpp:305
            gcode.push('\n');
            // GCodeWriter.cpp:306-307  M202: Set max travel acceleration
            let _ = write!(gcode, "M202 X{} Y{}", acceleration, acceleration);
        } else if self.config.gcode_flavor == GCodeFlavor::RepRapFirmware {
            // GCodeWriter.cpp:309-310  M204: Set default acceleration
            let _ = write!(gcode, "M204 P{}", acceleration);
        } else if self.config.gcode_flavor == GCodeFlavor::Marlin {
            // GCodeWriter.cpp:312-314  This is new MarlinFirmware with separated
            // print/retraction/travel acceleration. Use M204 P, we don't want to override travel acc
            // by M204 S (which is deprecated anyway).
            let _ = write!(gcode, "M204 P{}", acceleration);
        } else if self.config.gcode_flavor == GCodeFlavor::Klipper && self.config.accel_to_decel_enable {
            // GCodeWriter.cpp:316  gcode << ... << acceleration * this->config.accel_to_decel_factor / 100;
            // (an unsigned int times the percent's double, over 100: a double the stream prints at
            // its default precision)
            let _ = write!(
                gcode,
                "SET_VELOCITY_LIMIT ACCEL_TO_DECEL={}",
                ostream_double(acceleration as f64 * self.config.accel_to_decel_factor / 100.0)
            );
            // GCodeWriter.cpp:317
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; adjust ACCEL_TO_DECEL");
            }
            // GCodeWriter.cpp:318
            let _ = write!(gcode, "\nM204 S{}", acceleration);
            // Set max accel to decel to half of acceleration
        } else {
            // GCodeWriter.cpp:321-322  M204: Set default acceleration
            let _ = write!(gcode, "M204 S{}", acceleration);
        }
        // GCodeWriter.cpp:325
        if FULL_GCODE_COMMENT {
            gcode.push_str(" ; adjust acceleration");
        }
        // GCodeWriter.cpp:326
        gcode.push('\n');

        // GCodeWriter.cpp:328
        gcode
    }

    // GCodeWriter.cpp:331-349  std::string GCodeWriter::set_pressure_advance(double pa, bool is_bbl_bowden) const
    pub fn set_pressure_advance(&self, pa: f64, is_bbl_bowden: bool) -> String {
        // GCodeWriter.cpp:333
        let mut gcode = String::new();
        // GCodeWriter.cpp:334
        if pa < 0.0 {
            return gcode;
        }
        // GCodeWriter.cpp:335  if (false) { // todo: bbl printer
        #[allow(clippy::overly_complex_bool_expr)]
        if false {
            // GCodeWriter.cpp:336-337  OrcaSlicer: set L1000 to use linear model
            let _ = write!(gcode, "M400\n M900 K{} L1000 M10 ; Override pressure advance value\n", format_setprecision_4(pa));
        } else {
            // GCodeWriter.cpp:339-346
            if self.config.gcode_flavor == GCodeFlavor::Klipper {
                let _ = write!(gcode, "SET_PRESSURE_ADVANCE ADVANCE={}; Override pressure advance value\n", format_setprecision_4(pa));
            } else if self.config.gcode_flavor == GCodeFlavor::RepRapFirmware {
                let _ = write!(gcode, "M572 D0 S{}; Override pressure advance value\n", format_setprecision_4(pa));
            } else if is_bbl_bowden {
                let _ = write!(gcode, "M400\n M901 P0.75 K{}; Override pressure advance value\n", format_setprecision_4(pa));
            } else {
                let _ = write!(gcode, "M400\n M900 K{}; Override pressure advance value\n", format_setprecision_4(pa));
            }
        }
        // GCodeWriter.cpp:348
        gcode
    }

    // GCodeWriter.cpp:351-370  std::string GCodeWriter::set_jerk_xy(double jerk)
    pub fn set_jerk_xy(&mut self, mut jerk: f64) -> String {
        // GCodeWriter.cpp:353-354  Clamp the jerk to the allowed maximum.
        if self.m_max_jerk > 0.0 && jerk > self.m_max_jerk {
            jerk = self.m_max_jerk;
        }

        // GCodeWriter.cpp:356
        if jerk < 0.01 || crate::geometry::geometry::is_approx(jerk, self.m_last_jerk) {
            return String::new();
        }

        // GCodeWriter.cpp:358
        self.m_last_jerk = jerk;

        // GCodeWriter.cpp:360-364
        let mut gcode = String::new();
        if self.config.gcode_flavor == GCodeFlavor::Klipper {
            let _ = write!(gcode, "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY={}", ostream_double(jerk));
        } else {
            let _ = write!(gcode, "M205 X{} Y{}", ostream_double(jerk), ostream_double(jerk));
        }

        // GCodeWriter.cpp:366
        if FULL_GCODE_COMMENT {
            gcode.push_str(" ; adjust jerk");
        }
        // GCodeWriter.cpp:367
        gcode.push('\n');

        // GCodeWriter.cpp:369
        gcode
    }

    // GCodeWriter.cpp:372-395  std::string GCodeWriter::reset_e(bool force)
    pub fn reset_e(&mut self, force: bool) -> String {
        // GCodeWriter.cpp:374-377
        if self.config.gcode_flavor == GCodeFlavor::Mach3
            || self.config.gcode_flavor == GCodeFlavor::MakerWare
            || self.config.gcode_flavor == GCodeFlavor::Sailfish
        {
            return String::new();
        }

        // GCodeWriter.cpp:379
        if self.m_curr_extruder_id != -1 {
            if let Some(idx) = self.m_curr_filament_extruder[self.m_curr_extruder_id as usize] {
                // GCodeWriter.cpp:380-381
                if self.m_filament_extruders[idx].e() == 0.0 && !force {
                    return String::new();
                }
                // GCodeWriter.cpp:382
                self.m_filament_extruders[idx].reset_e();
            }
        }

        // GCodeWriter.cpp:385
        if !self.config.use_relative_e_distances {
            // GCodeWriter.cpp:386-391
            let mut gcode = String::new();
            gcode.push_str("G92 E0");
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; reset extrusion distance");
            }
            gcode.push('\n');
            gcode
        } else {
            // GCodeWriter.cpp:393
            String::new()
        }
    }

    // GCodeWriter.cpp:397-411  std::string GCodeWriter::update_progress(unsigned int num, unsigned int tot, bool allow_100) const
    pub fn update_progress(&self, num: u32, tot: u32, allow_100: bool) -> String {
        // GCodeWriter.cpp:399-400
        if self.config.gcode_flavor != GCodeFlavor::MakerWare && self.config.gcode_flavor != GCodeFlavor::Sailfish {
            return String::new();
        }

        // GCodeWriter.cpp:402  unsigned int percent = (unsigned int)floor(100.0 * num / tot + 0.5);
        let mut percent = (100.0 * num as f64 / tot as f64 + 0.5).floor() as u32;
        // GCodeWriter.cpp:403
        if !allow_100 {
            percent = percent.min(99);
        }

        // GCodeWriter.cpp:405-410
        let mut gcode = String::new();
        let _ = write!(gcode, "M73 P{}", percent);
        if FULL_GCODE_COMMENT {
            gcode.push_str(" ; update progress");
        }
        gcode.push('\n');
        gcode
    }

    // GCodeWriter.cpp:413-417  std::string GCodeWriter::toolchange_prefix() const
    pub fn toolchange_prefix(&self) -> String {
        (if self.config.gcode_flavor == GCodeFlavor::MakerWare {
            "M135 T"
        } else if self.config.gcode_flavor == GCodeFlavor::Sailfish {
            "M108 T"
        } else {
            "T"
        })
        .to_string()
    }

    // `Slic3r::lower_bound_by_predicate` (libslic3r.h) over the sorted m_filament_extruders with
    // the predicate `e.id() < filament_id`: the index of the first extruder whose id is not less
    // than `filament_id`, or len() when there is none.
    fn lower_bound_filament(&self, filament_id: u32) -> usize {
        let mut first: usize = 0;
        let mut count = self.m_filament_extruders.len();
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

    // GCodeWriter.cpp:419-444  std::string GCodeWriter::toolchange(unsigned int filament_id, unsigned int nozzle_id)
    pub fn toolchange(&mut self, filament_id: u32, nozzle_id: u32) -> String {
        // GCodeWriter.cpp:421-423  set the new extruder
        let filament_extruder_iter = self.lower_bound_filament(filament_id);
        debug_assert!(
            filament_extruder_iter != self.m_filament_extruders.len()
                && self.m_filament_extruders[filament_extruder_iter].id() == filament_id
        );
        // GCodeWriter.cpp:424
        self.m_curr_extruder_id = self.m_filament_extruders[filament_extruder_iter].extruder_id() as i32;
        // GCodeWriter.cpp:425
        self.m_curr_filament_extruder[self.m_curr_extruder_id as usize] = Some(filament_extruder_iter);
        // GCodeWriter.cpp:426
        self.m_current_process_config_idx = get_process_config_idx(&self.config, filament_id);

        // GCodeWriter.cpp:428-430  return the toolchange command; if we are running a
        // single-extruder setup, just set the extruder and return nothing
        let mut gcode = String::new();
        // GCodeWriter.cpp:431
        if self.multiple_extruders {
            // GCodeWriter.cpp:433-436  BBS
            if self.m_is_bbl_printer {
                let _ = write!(gcode, "M1020 S{} H{}", filament_id, nozzle_id);
            } else {
                let _ = write!(gcode, "{}{}", self.toolchange_prefix(), filament_id);
            }
            // GCodeWriter.cpp:438-439
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; change extruder");
            }
            // GCodeWriter.cpp:440
            gcode.push('\n');
            // GCodeWriter.cpp:441
            let reset = self.reset_e(true);
            gcode.push_str(&reset);
        }
        // GCodeWriter.cpp:443
        gcode
    }

    // GCodeWriter.cpp:446-457  std::string GCodeWriter::set_speed(double F, const std::string &comment, const std::string &cooling_marker)
    pub fn set_speed(&mut self, f: f64, comment: &str, cooling_marker: &str) -> String {
        // GCodeWriter.cpp:448-449
        debug_assert!(f > 0.0);
        debug_assert!(f < 100000.0);
        // GCodeWriter.cpp:450
        self.m_current_speed = f;
        // GCodeWriter.cpp:451-456
        let mut w = GCodeG1Formatter::new();
        w.emit_f(f);
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        w.emit_string(cooling_marker);
        w.string()
    }

    // GCodeWriter.hpp:90  double get_current_speed()
    pub fn get_current_speed(&self) -> f64 {
        self.m_current_speed
    }

    // GCodeWriter.cpp:459-462  std::string GCodeWriter::travel_to_xy(const Vec2d &point, const std::string &comment)
    pub fn travel_to_xy(&mut self, point: Vec2d, comment: &str) -> String {
        <GCodeWriter as GCodeWriterTravelToXyShort>::travel_to_xy(self, point, comment, false)
    }

    // GCodeWriter.cpp:481-511  std::string GCodeWriter::lazy_lift(LiftType lift_type, bool spiral_vase, bool tool_change)
    //  If this method is called more than once before calling unlift(), it will not perform
    //  subsequent lifts, even if Z was raised manually (i.e. with travel_to_z()) and thus _lifted
    //  was reduced.
    pub fn lazy_lift(&mut self, lift_type: LiftType, spiral_vase: bool, tool_change: bool) -> String {
        // GCodeWriter.cpp:486-487  check whether the above/below conditions are met
        let mut target_lift = 0.0;
        {
            // GCodeWriter.cpp:490-493  BBS
            let extruder_id = GCodeWriter::filament(self).unwrap().extruder_id() as i32;
            let filament_id = GCodeWriter::filament(self).unwrap().id() as i32;
            let above = self.config.retract_lift_above.get_at(extruder_id as usize);
            let below = self.config.retract_lift_below.get_at(extruder_id as usize);
            // GCodeWriter.cpp:494
            if self.m_pos.z >= above && self.m_pos.z <= below {
                // GCodeWriter.cpp:495
                target_lift = self.config.z_hop.get_at(filament_id as usize);
                // GCodeWriter.cpp:496
                if tool_change && self.config.prime_tower_lift_height > 0.0 {
                    target_lift = self.config.prime_tower_lift_height;
                }
            }
        }
        // GCodeWriter.cpp:500  BBS
        if self.m_lifted == 0.0 && self.m_to_lift == 0.0 && target_lift > 0.0 {
            // GCodeWriter.cpp:501-504
            if spiral_vase {
                self.m_lifted = target_lift;
                return self._travel_to_z(self.m_pos.z + target_lift, "lift Z", tool_change);
            } else {
                // GCodeWriter.cpp:506-507
                self.m_to_lift = target_lift;
                self.m_to_lift_type = lift_type;
            }
        }
        // GCodeWriter.cpp:510
        String::new()
    }

    // GCodeWriter.cpp:513-535  bool GCodeWriter::spiral_arc_within_bed(const Vec2d &center, double radius) const
    // BBS: return true if the full spiral-lift circle (centre, radius) stays inside the bed
    // printable area. When the bed bounding box is unknown, keep the legacy behavior (no block).
    fn spiral_arc_within_bed(&self, center: &Vec2d, radius: f64) -> bool {
        // GCodeWriter.cpp:520-523  Prefer the printable area of the head that performs the lift.
        let mut bmin = self.m_bed_min;
        let mut bmax = self.m_bed_max;
        let mut valid = self.m_bed_bbox_valid;
        let eid = self.m_curr_extruder_id;
        // GCodeWriter.cpp:524-528
        if eid >= 0 && eid < self.m_extruder_bed_valid.len() as i32 && self.m_extruder_bed_valid[eid as usize] != 0 {
            bmin = self.m_extruder_bed_min[eid as usize];
            bmax = self.m_extruder_bed_max[eid as usize];
            valid = true;
        }
        // GCodeWriter.cpp:529-530
        if !valid {
            return true;
        }
        // GCodeWriter.cpp:531-534
        center.x - radius >= bmin.x
            && center.x + radius <= bmax.x
            && center.y - radius >= bmin.y
            && center.y + radius <= bmax.y
    }

    // GCodeWriter.cpp:537-595  std::string GCodeWriter::eager_lift(const LiftType type, bool tool_change)
    // BBS: immediately execute an undelayed lift move with a spiral lift pattern designed
    // specifically for subsequent gcode injection (e.g. timelapse)
    pub fn eager_lift(&mut self, lift_type: LiftType, tool_change: bool) -> String {
        // GCodeWriter.cpp:541-542
        let mut lift_move = String::new();
        let mut target_lift = 0.0;
        {
            // GCodeWriter.cpp:545-548  BBS
            let extruder_id = GCodeWriter::filament(self).unwrap().extruder_id() as i32;
            let filament_id = GCodeWriter::filament(self).unwrap().id() as i32;
            let above = self.config.retract_lift_above.get_at(extruder_id as usize);
            let below = self.config.retract_lift_below.get_at(extruder_id as usize);
            // GCodeWriter.cpp:549-552
            if self.m_pos.z >= above && self.m_pos.z <= below {
                target_lift = self.config.z_hop.get_at(filament_id as usize);
                if tool_change && self.config.prime_tower_lift_height > 0.0 {
                    target_lift = self.config.prime_tower_lift_height;
                }
            }
        }

        // GCodeWriter.cpp:555-557
        let to_lift = target_lift - self.m_lifted;
        if to_lift < EPSILON {
            return lift_move;
        }

        // GCodeWriter.cpp:559-560  BBS: spiral lift only safe with known position
        if lift_type == LiftType::SpiralLift && self.is_current_position_clear() {
            // GCodeWriter.cpp:561
            if to_lift > 0.0 {
                // GCodeWriter.cpp:562
                let radius = to_lift / (2.0 * PI * SLOPE_THRESHOLD.atan());
                // GCodeWriter.cpp:563-569  static spiral alignment when no move in x,y plane. The
                // spiral centre sits a radius away from the current point; try the opposite /
                // orthogonal directions when the legacy +X centre pushes the arc off the bed.
                let cur_xy = Vec2d::new(self.m_pos.x - self.m_x_offset, self.m_pos.y - self.m_y_offset);
                // GCodeWriter.cpp:570
                let dirs: [Vec2d; 4] =
                    [Vec2d::new(1.0, 0.0), Vec2d::new(-1.0, 0.0), Vec2d::new(0.0, 1.0), Vec2d::new(0.0, -1.0)];
                // GCodeWriter.cpp:571-572
                let mut found = false;
                let mut ij_offset = Vec2d::new(radius, 0.0);
                // GCodeWriter.cpp:573-580
                for d in &dirs {
                    let off = Vec2d::new(radius * d.x, radius * d.y);
                    let center = Vec2d::new(cur_xy.x + off.x, cur_xy.y + off.y);
                    if self.spiral_arc_within_bed(&center, radius) {
                        ij_offset = off;
                        found = true;
                        break;
                    }
                }
                // GCodeWriter.cpp:581-585
                if found {
                    lift_move = self._spiral_travel_to_z(self.m_pos.z + to_lift, ij_offset, "spiral lift Z", tool_change);
                } else {
                    // no in-bed spiral direction, fall back to a plain vertical lift
                    lift_move = self._travel_to_z(self.m_pos.z + to_lift, "normal lift Z", tool_change);
                }
            }
        }
        // GCodeWriter.cpp:588-591  BBS: if position is unknown use normal lift
        else if to_lift > 0.0 {
            lift_move = self._travel_to_z(self.m_pos.z + to_lift, "normal lift Z", tool_change);
        }
        // GCodeWriter.cpp:592-594
        self.m_lifted = target_lift;
        self.m_to_lift = 0.0;
        lift_move
    }

    // GCodeWriter.cpp:597-600  std::string GCodeWriter::travel_to_xyz(const Vec3d &point, const std::string &comment)
    pub fn travel_to_xyz(&mut self, point: Vec3d, comment: &str) -> String {
        <GCodeWriter as GCodeWriterTravelToXyzShort>::travel_to_xyz(self, point, comment, false)
    }

    // GCodeWriter.cpp:746-763  std::string GCodeWriter::travel_to_z(double z, const std::string &comment)
    pub fn travel_to_z(&mut self, z: f64, comment: &str) -> String {
        // GCodeWriter.cpp:748-757  If target Z is lower than current Z but higher than nominal Z
        // we don't perform the move but we only adjust the nominal Z by reducing the lift amount
        // that will be used for unlift.
        if !self.will_move_z(z) {
            let nominal_z = self.m_pos.z - self.m_lifted;
            self.m_lifted -= z - nominal_z;
            if self.m_lifted.abs() < EPSILON {
                self.m_lifted = 0.0;
            }
            return String::new();
        }

        // GCodeWriter.cpp:759-762  In all the other cases, we perform an actual Z move and cancel
        // the lift.
        self.m_lifted = 0.0;
        let accel = <GCodeWriter as GCodeWriterSetTravelAccelerationNow>::set_travel_acceleration(self);
        accel + &self._travel_to_z(z, comment, false)
    }

    // GCodeWriter.cpp:765-781  std::string GCodeWriter::_travel_to_z(double z, const std::string &comment, bool tool_change)
    fn _travel_to_z(&mut self, z: f64, comment: &str, tool_change: bool) -> String {
        // GCodeWriter.cpp:767
        self.m_pos.z = z;

        // GCodeWriter.cpp:769-771
        let mut speed = self.config.travel_speed_z.get_at(self.m_current_process_config_idx);
        if speed == 0.0 {
            speed = self.config.travel_speed.get_at(self.m_current_process_config_idx);
        }
        // GCodeWriter.cpp:772-774
        if tool_change && self.config.prime_tower_lift_speed > 0.0 {
            speed = self.config.prime_tower_lift_speed; // lift speed
        }
        // GCodeWriter.cpp:775-779
        let mut w = GCodeG1Formatter::new();
        w.emit_z(z);
        w.emit_f(speed * 60.0);
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:780
        let accel = <GCodeWriter as GCodeWriterSetTravelAccelerationNow>::set_travel_acceleration(self);
        accel + &w.string()
    }

    // GCodeWriter.cpp:783-801  std::string GCodeWriter::_spiral_travel_to_z(double z, const Vec2d &ij_offset, const std::string &comment, bool tool_change)
    fn _spiral_travel_to_z(&mut self, z: f64, ij_offset: Vec2d, comment: &str, tool_change: bool) -> String {
        // GCodeWriter.cpp:785
        self.m_pos.z = z;

        // GCodeWriter.cpp:787-789
        let mut speed = self.config.travel_speed_z.get_at(self.m_current_process_config_idx);
        if speed == 0.0 {
            speed = self.config.travel_speed.get_at(self.m_current_process_config_idx);
        }
        // GCodeWriter.cpp:790-792
        if tool_change && self.config.prime_tower_lift_speed > 0.0 {
            speed = self.config.prime_tower_lift_speed; // lift speed
        }
        // GCodeWriter.cpp:793
        let output = "G17\n";
        // GCodeWriter.cpp:794-799
        let mut w = GCodeG2G3Formatter::new(true);
        w.emit_z(z);
        w.emit_ij(ij_offset);
        w.emit_string(" P1 ");
        w.emit_f(speed * 60.0);
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:800
        let accel = <GCodeWriter as GCodeWriterSetTravelAccelerationNow>::set_travel_acceleration(self);
        accel + output + &w.string()
    }

    // GCodeWriter.cpp:803-818  bool GCodeWriter::will_move_z(double z) const
    pub fn will_move_z(&self, z: f64) -> bool {
        // GCodeWriter.cpp:805-811  If target Z is lower than current Z but higher than nominal Z
        // we don't perform an actual Z move.
        if self.m_lifted > 0.0 {
            let nominal_z = self.m_pos.z - self.m_lifted;
            if z >= nominal_z - EPSILON && z <= self.m_pos.z + EPSILON {
                return false;
            }
        }
        // GCodeWriter.cpp:812-816  BBS. Dont move z if it is the same as target z
        else if (self.m_pos.z - z).abs() < EPSILON {
            return false;
        }
        // GCodeWriter.cpp:817
        true
    }

    // GCodeWriter.cpp:820-838  std::string GCodeWriter::extrude_to_xy(const Vec2d &point, double dE, const std::string &comment, bool force_no_extrusion)
    pub fn extrude_to_xy(&mut self, point: Vec2d, d_e: f64, comment: &str, force_no_extrusion: bool) -> String {
        // GCodeWriter.cpp:822-823
        self.m_pos.x = point.x;
        self.m_pos.y = point.y;

        // GCodeWriter.cpp:825-826
        if !force_no_extrusion {
            self.filament_mut().unwrap().extrude(d_e);
        }

        // GCodeWriter.cpp:828-829  BBS: take plate offset into consider
        let point_on_plate = Vec2d::new(point.x - self.m_x_offset, point.y - self.m_y_offset);

        // GCodeWriter.cpp:831-836
        let mut w = GCodeG1Formatter::new();
        w.emit_xy(point_on_plate);
        if !force_no_extrusion {
            w.emit_e(GCodeWriter::filament(self).unwrap().e());
        }
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:837
        let accel = self.set_extrude_acceleration();
        accel + &w.string()
    }

    // GCodeWriter.cpp:840-860  std::string GCodeWriter::extrude_arc_to_xy(const Vec2d& point, const Vec2d& center_offset, double dE, const bool is_ccw, const std::string& comment, bool force_no_extrusion)
    // BBS: generate G2 or G3 extrude which moves by arc; point is end point which means X and Y
    // axis, center_offset is I and J axis
    pub fn extrude_arc_to_xy(
        &mut self,
        point: Vec2d,
        center_offset: Vec2d,
        d_e: f64,
        is_ccw: bool,
        comment: &str,
        force_no_extrusion: bool,
    ) -> String {
        // GCodeWriter.cpp:845-848
        self.m_pos.x = point.x;
        self.m_pos.y = point.y;
        if !force_no_extrusion {
            self.filament_mut().unwrap().extrude(d_e);
        }

        // GCodeWriter.cpp:850
        let point_on_plate = Vec2d::new(point.x - self.m_x_offset, point.y - self.m_y_offset);

        // GCodeWriter.cpp:852-858
        let mut w = GCodeG2G3Formatter::new(is_ccw);
        w.emit_xy(point_on_plate);
        w.emit_ij(center_offset);
        if !force_no_extrusion {
            w.emit_e(GCodeWriter::filament(self).unwrap().e());
        }
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:859
        let accel = self.set_extrude_acceleration();
        accel + &w.string()
    }

    // GCodeWriter.cpp:862-879  std::string GCodeWriter::extrude_to_xyz(const Vec3d &point, double dE, const std::string &comment, bool force_no_extrusion)
    pub fn extrude_to_xyz(&mut self, point: Vec3d, d_e: f64, comment: &str, force_no_extrusion: bool) -> String {
        // GCodeWriter.cpp:864-867
        self.m_pos = point;
        self.m_lifted = 0.0;
        if !force_no_extrusion {
            self.filament_mut().unwrap().extrude(d_e);
        }

        // GCodeWriter.cpp:869-870  BBS: take plate offset into consider
        let point_on_plate = Vec3d::new(point.x - self.m_x_offset, point.y - self.m_y_offset, point.z);

        // GCodeWriter.cpp:872-877
        let mut w = GCodeG1Formatter::new();
        w.emit_xyz(point_on_plate);
        if !force_no_extrusion {
            w.emit_e(GCodeWriter::filament(self).unwrap().e());
        }
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:878
        let accel = self.set_extrude_acceleration();
        accel + &w.string()
    }

    // GCodeWriter.cpp:881-890  std::string GCodeWriter::retract(bool before_wipe)
    pub fn retract(&mut self, before_wipe: bool) -> String {
        // GCodeWriter.cpp:883-884
        let factor = if before_wipe { GCodeWriter::filament(self).unwrap().retract_before_wipe() } else { 1.0 };
        debug_assert!(factor >= 0.0 && factor <= 1.0 + EPSILON);
        // GCodeWriter.cpp:885-889
        let length = factor * GCodeWriter::filament(self).unwrap().retraction_length();
        let restart_extra = factor * GCodeWriter::filament(self).unwrap().retract_restart_extra();
        self._retract(length, restart_extra, "retract")
    }

    // GCodeWriter.cpp:892-901  std::string GCodeWriter::retract_for_toolchange(bool before_wipe)
    pub fn retract_for_toolchange(&mut self, before_wipe: bool) -> String {
        // GCodeWriter.cpp:894-895
        let factor = if before_wipe { GCodeWriter::filament(self).unwrap().retract_before_wipe() } else { 1.0 };
        debug_assert!(factor >= 0.0 && factor <= 1.0 + EPSILON);
        // GCodeWriter.cpp:896-900
        let length = factor * GCodeWriter::filament(self).unwrap().retract_length_toolchange();
        let restart_extra = factor * GCodeWriter::filament(self).unwrap().retract_restart_extra_toolchange();
        self._retract(length, restart_extra, "retract for toolchange")
    }

    // GCodeWriter.cpp:903-928  std::string GCodeWriter::_retract(double length, double restart_extra, const std::string &comment)
    fn _retract(&mut self, mut length: f64, restart_extra: f64, comment: &str) -> String {
        // GCodeWriter.cpp:905
        let mut gcode = String::new();
        // GCodeWriter.cpp:906-907
        if self.config.use_firmware_retraction {
            length = 1.0;
        }
        // GCodeWriter.cpp:908
        let d_e = self.filament_mut().unwrap().retract(length, restart_extra);
        if d_e != 0.0 {
            // GCodeWriter.cpp:909-911  add firmware retraction
            if self.config.use_firmware_retraction {
                gcode = if self.config.gcode_flavor == GCodeFlavor::Machinekit {
                    "G22 ;retract".to_string()
                } else {
                    "G10 ;retract \n".to_string()
                };
            } else {
                // GCodeWriter.cpp:914-920  BBS
                let mut w = GCodeG1Formatter::new();
                w.emit_e(GCodeWriter::filament(self).unwrap().e());
                w.emit_f(GCodeWriter::filament(self).unwrap().retract_speed() as f64 * 60.0);
                w.emit_comment(FULL_GCODE_COMMENT, comment);
                gcode = w.string();
            }
        }

        // GCodeWriter.cpp:924-925
        if self.config.gcode_flavor == GCodeFlavor::MakerWare {
            gcode.push_str("M103 ; extruder off\n");
        }

        // GCodeWriter.cpp:927
        gcode
    }

    // GCodeWriter.cpp:930-955  std::string GCodeWriter::unretract(float extra_retract)
    pub fn unretract(&mut self, extra_retract: f32) -> String {
        // GCodeWriter.cpp:932
        let mut gcode = String::new();

        // GCodeWriter.cpp:934-935
        if self.config.gcode_flavor == GCodeFlavor::MakerWare {
            gcode = "M101 ; extruder on\n".to_string();
        }

        // GCodeWriter.cpp:937
        let d_e = self.filament_mut().unwrap().unretract();
        if d_e != 0.0 {
            // GCodeWriter.cpp:938-940
            if self.config.use_firmware_retraction {
                gcode += if self.config.gcode_flavor == GCodeFlavor::Machinekit {
                    "G23 ;unretract \n"
                } else {
                    "G11 ;unretract \n"
                };
                let reset = self.reset_e(false);
                gcode += &reset;
            } else {
                // GCodeWriter.cpp:943-950  BBS: use G1 instead of G0 because G0 will blend the
                // restart with the previous travel move
                let mut w = GCodeG1Formatter::new();
                // `filament()->E()+extra_retract`: a double plus the float widened
                w.emit_e(GCodeWriter::filament(self).unwrap().e() + extra_retract as f64);
                w.emit_f(GCodeWriter::filament(self).unwrap().deretract_speed() as f64 * 60.0);
                w.emit_comment(FULL_GCODE_COMMENT, " ; unretract");
                gcode += &w.string();
            }
        }

        // GCodeWriter.cpp:954
        gcode
    }

    // GCodeWriter.cpp:957-969  double GCodeWriter::get_extruder_retracted_length(const int filament_id)
    pub fn get_extruder_retracted_length(&self, filament_id: i32) -> f64 {
        // GCodeWriter.cpp:959-961
        let res;
        let filament_extruder_iter = self.lower_bound_filament(filament_id as u32);
        debug_assert!(
            filament_extruder_iter != self.m_filament_extruders.len()
                && self.m_filament_extruders[filament_extruder_iter].id() == filament_id as u32
        );

        // GCodeWriter.cpp:963-966
        if self.m_filament_extruders[filament_extruder_iter].is_share_extruder() {
            res = self.m_filament_extruders[filament_extruder_iter].get_share_retracted_length();
        } else {
            res = self.m_filament_extruders[filament_extruder_iter].get_single_retracted_length();
        }

        // GCodeWriter.cpp:968
        res
    }

    // GCodeWriter.cpp:971-980  std::string GCodeWriter::unlift()
    pub fn unlift(&mut self) -> String {
        // GCodeWriter.cpp:973
        let mut gcode = String::new();
        // GCodeWriter.cpp:974-977
        if self.m_lifted > 0.0 {
            let z_move = self._travel_to_z(self.m_pos.z - self.m_lifted, "restore layer Z", false);
            gcode += &z_move;
            self.m_lifted = 0.0;
        }
        // GCodeWriter.cpp:978
        self.m_to_lift = 0.0;
        // GCodeWriter.cpp:979
        gcode
    }

    // GCodeWriter.cpp:1016-1020  std::string GCodeWriter::set_fan(unsigned int speed) const
    pub fn set_fan(&self, speed: u32) -> String {
        // GCodeWriter.cpp:1018-1019  BBS
        <GCodeWriter as GCodeWriterSetFanFlavor>::set_fan(self.config.gcode_flavor, speed)
    }

    // GCodeWriter.cpp:1022-1038  std::string GCodeWriter::set_additional_fan(int gcode_index, unsigned int speed)
    // BBS: set the speed of one auxiliary part cooling fan, addressed by its M106 P index
    pub fn set_additional_fan(gcode_index: i32, speed: u32) -> String {
        // GCodeWriter.cpp:1025
        let mut gcode = String::new();

        // GCodeWriter.cpp:1027  const int fan_speed = (int)(255.0 * speed / 100.0);
        let fan_speed = (255.0 * speed as f64 / 100.0) as i32;
        // GCodeWriter.cpp:1028
        let _ = write!(gcode, "M106 P{} S{}", gcode_index, fan_speed);
        // GCodeWriter.cpp:1029-1034
        if FULL_GCODE_COMMENT {
            if speed == 0 {
                gcode.push_str(" ; disable additional fan ");
            } else {
                gcode.push_str(" ; enable additional fan ");
            }
        }
        // GCodeWriter.cpp:1035
        gcode.push('\n');

        // GCodeWriter.cpp:1037
        gcode
    }

    // GCodeWriter.cpp:1040-1048  std::string GCodeWriter::set_exhaust_fan(int speed, bool add_eol)
    pub fn set_exhaust_fan(speed: i32, add_eol: bool) -> String {
        // GCodeWriter.cpp:1042-1043  gcode << "M106" << " P3" << " S" << (int)(speed / 100.0 * 255);
        let mut gcode = String::new();
        let _ = write!(gcode, "M106 P3 S{}", (speed as f64 / 100.0 * 255.0) as i32);

        // GCodeWriter.cpp:1045-1046
        if add_eol {
            gcode.push('\n');
        }
        // GCodeWriter.cpp:1047
        gcode
    }

    // GCodeWriter.hpp:125  void set_object_start_str(std::string start_string)
    pub fn set_object_start_str(&mut self, start_string: String) {
        self.m_gcode_label_objects_start = start_string;
    }
    // GCodeWriter.hpp:126  bool empty_object_start_str()
    pub fn empty_object_start_str(&self) -> bool {
        self.m_gcode_label_objects_start.is_empty()
    }
    // GCodeWriter.hpp:127  void set_object_end_str(std::string end_string)
    pub fn set_object_end_str(&mut self, end_string: String) {
        self.m_gcode_label_objects_end = end_string;
    }
    // GCodeWriter.hpp:128  bool empty_object_end_str()
    pub fn empty_object_end_str(&self) -> bool {
        self.m_gcode_label_objects_end.is_empty()
    }

    // GCodeWriter.cpp:1050-1056  void GCodeWriter::add_object_start_labels(std::string& gcode)
    pub fn add_object_start_labels(&mut self, gcode: &mut String) {
        if !self.m_gcode_label_objects_start.is_empty() {
            gcode.push_str(&self.m_gcode_label_objects_start);
            self.m_gcode_label_objects_start = String::new();
        }
    }

    // GCodeWriter.cpp:1058-1064  void GCodeWriter::add_object_end_labels(std::string& gcode)
    pub fn add_object_end_labels(&mut self, gcode: &mut String) {
        if !self.m_gcode_label_objects_end.is_empty() {
            gcode.push_str(&self.m_gcode_label_objects_end);
            self.m_gcode_label_objects_end = String::new();
        }
    }

    // GCodeWriter.cpp:1066-1070  void GCodeWriter::add_object_change_labels(std::string& gcode)
    pub fn add_object_change_labels(&mut self, gcode: &mut String) {
        self.add_object_end_labels(gcode);
        self.add_object_start_labels(gcode);
    }

    // GCodeWriter.cpp:1072-1079  std::string GCodeWriter::set_extruder(unsigned int filament_id, unsigned int nozzle_id)
    pub fn set_extruder(&mut self, filament_id: u32, nozzle_id: u32) -> String {
        // GCodeWriter.cpp:1074-1076
        let filament_ext_it = self.lower_bound_filament(filament_id);
        let _extruder_id: u32 = (nozzle_id > 0) as u32;
        debug_assert!(
            filament_ext_it != self.m_filament_extruders.len()
                && self.m_filament_extruders[filament_ext_it].id() == filament_id
        );
        // GCodeWriter.cpp:1077-1078  TODO: optmize here, pass extruder_id to toolchange
        if self.need_toolchange(filament_id) {
            self.toolchange(filament_id, nozzle_id)
        } else {
            String::new()
        }
    }

    // GCodeWriter.cpp:1081-1090  void GCodeWriter::init_extruder(unsigned int filament_id, unsigned int nozzle_id)
    pub fn init_extruder(&mut self, filament_id: u32, nozzle_id: u32) {
        // GCodeWriter.cpp:1083  `filament_id != -1` compares the unsigned id with (unsigned)-1.
        if self.m_curr_extruder_id == -1 && filament_id != u32::MAX {
            // GCodeWriter.cpp:1084-1085
            let filament_extruder_iter = self.lower_bound_filament(filament_id);
            debug_assert!(
                filament_extruder_iter != self.m_filament_extruders.len()
                    && self.m_filament_extruders[filament_extruder_iter].id() == filament_id
            );
            // GCodeWriter.cpp:1086
            self.m_curr_extruder_id = (nozzle_id > 0) as i32;
            // GCodeWriter.cpp:1087
            self.m_curr_filament_extruder[self.m_curr_extruder_id as usize] = Some(filament_extruder_iter);
            // GCodeWriter.cpp:1088
            self.m_current_process_config_idx = get_process_config_idx(&self.config, filament_id);
        }
    }

    // GCodeWriter.cpp:1092-1095  bool GCodeWriter::need_toolchange(unsigned int filament_id) const
    pub fn need_toolchange(&self, filament_id: u32) -> bool {
        match GCodeWriter::filament(self) {
            None => true,
            Some(f) => f.id() != filament_id,
        }
    }

    // GCodeWriter.hpp:110  Vec3d get_position() const
    pub fn get_position(&self) -> Vec3d {
        self.m_pos
    }
    // GCodeWriter.hpp:111  void set_position(Vec3d& in)
    pub fn set_position(&mut self, in_pos: Vec3d) {
        self.m_pos = in_pos;
    }

    // GCodeWriter.hpp:114  void set_xy_offset(double x, double y)
    pub fn set_xy_offset(&mut self, x: f64, y: f64) {
        self.m_x_offset = x;
        self.m_y_offset = y;
    }
    // GCodeWriter.hpp:115  Vec2f get_xy_offset()
    pub fn get_xy_offset(&self) -> (f32, f32) {
        (self.m_x_offset as f32, self.m_y_offset as f32)
    }

    // GCodeWriter.hpp:134  void set_current_position_clear(bool clear)
    pub fn set_current_position_clear(&mut self, clear: bool) {
        self.m_is_current_pos_clear = clear;
    }
    // GCodeWriter.hpp:135  bool is_current_position_clear() const
    pub fn is_current_position_clear(&self) -> bool {
        self.m_is_current_pos_clear
    }
    // GCodeWriter.hpp:136  void set_is_bbl_printer(bool is_bbl_printer)
    pub fn set_is_bbl_printer(&mut self, is_bbl_printer: bool) {
        self.m_is_bbl_printer = is_bbl_printer;
    }
    // GCodeWriter.hpp:137  void set_avoid_z_descent_travel(bool v)
    pub fn set_avoid_z_descent_travel(&mut self, v: bool) {
        self.m_avoid_z_descent_travel = v;
    }

    /// The private members, as tests/g_code_writer_native_parity.rs compares them with the C++
    /// writer's; not a member of the C++ class.
    #[doc(hidden)]
    pub fn parity_members(&self) -> GCodeWriterMembers {
        GCodeWriterMembers {
            lifted: self.m_lifted,
            to_lift: self.m_to_lift,
            to_lift_type: self.m_to_lift_type,
            curr_extruder_id: self.m_curr_extruder_id,
            current_process_config_idx: self.m_current_process_config_idx,
            last_acceleration: self.m_last_acceleration,
            max_acceleration: self.m_max_acceleration,
            last_jerk: self.m_last_jerk,
            max_jerk: self.m_max_jerk,
            is_current_pos_clear: self.m_is_current_pos_clear,
            current_speed: self.m_current_speed,
            last_bed_temperature: self.m_last_bed_temperature,
            last_bed_temperature_reached: self.m_last_bed_temperature_reached,
            single_extruder_multi_material: self.m_single_extruder_multi_material,
            bed_bbox_valid: self.m_bed_bbox_valid,
            bed_min: self.m_bed_min,
            bed_max: self.m_bed_max,
            extruder_bed_valid: self.m_extruder_bed_valid.clone(),
            extruder_bed_min: self.m_extruder_bed_min.clone(),
            extruder_bed_max: self.m_extruder_bed_max.clone(),
        }
    }
}

/// The private members of a `GCodeWriter` (`GCodeWriter::parity_members`).
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct GCodeWriterMembers {
    pub lifted: f64,
    pub to_lift: f64,
    pub to_lift_type: LiftType,
    pub curr_extruder_id: i32,
    pub current_process_config_idx: usize,
    pub last_acceleration: u32,
    pub max_acceleration: u32,
    pub last_jerk: f64,
    pub max_jerk: f64,
    pub is_current_pos_clear: bool,
    pub current_speed: f64,
    pub last_bed_temperature: i32,
    pub last_bed_temperature_reached: bool,
    pub single_extruder_multi_material: bool,
    pub bed_bbox_valid: bool,
    pub bed_min: Vec2d,
    pub bed_max: Vec2d,
    pub extruder_bed_valid: Vec<u8>,
    pub extruder_bed_min: Vec<Vec2d>,
    pub extruder_bed_max: Vec<Vec2d>,
}

// ---------------------------------------------------------------------------
// The overloads of GCodeWriter past the first of each name.
// ---------------------------------------------------------------------------

/// GCodeWriter.hpp:43  `Extruder* filament(size_t extruder_id)`
pub trait GCodeWriterFilamentAtMut {
    fn filament(&mut self, extruder_id: usize) -> Option<&mut Extruder>;
}

/// GCodeWriter.hpp:44  `const Extruder* filament(size_t extruder_id) const`
pub trait GCodeWriterFilamentAt {
    fn filament(&self, extruder_id: usize) -> Option<&Extruder>;
}

/// GCodeWriter.hpp:45  `Extruder* filament()`
pub trait GCodeWriterFilamentMut {
    fn filament(&mut self) -> Option<&mut Extruder>;
}

/// GCodeWriter.cpp:464-479  `std::string travel_to_xy(const Vec2d &point, const std::string &comment, bool use_short_travel_acceleration)`
pub trait GCodeWriterTravelToXyShort {
    fn travel_to_xy(&mut self, point: Vec2d, comment: &str, use_short_travel_acceleration: bool) -> String;
}

/// GCodeWriter.cpp:602-744  `std::string travel_to_xyz(const Vec3d &point, const std::string &comment, bool use_short_travel_acceleration)`
pub trait GCodeWriterTravelToXyzShort {
    fn travel_to_xyz(&mut self, point: Vec3d, comment: &str, use_short_travel_acceleration: bool) -> String;
}

/// GCodeWriter.cpp:261-264  `std::string set_travel_acceleration()` (private)
trait GCodeWriterSetTravelAccelerationNow {
    fn set_travel_acceleration(&mut self) -> String;
}

/// GCodeWriter.cpp:266-286  `std::string set_travel_acceleration(bool use_short_travel_acceleration)` (private)
trait GCodeWriterSetTravelAccelerationShort {
    fn set_travel_acceleration(&mut self, use_short_travel_acceleration: bool) -> String;
}

/// GCodeWriter.cpp:982-1014  `static std::string set_fan(const GCodeFlavor gcode_flavor, unsigned int speed)`
pub trait GCodeWriterSetFanFlavor {
    fn set_fan(gcode_flavor: GCodeFlavor, speed: u32) -> String;
}

impl GCodeWriterFilamentAtMut for GCodeWriter {
    // GCodeWriter.hpp:43
    fn filament(&mut self, extruder_id: usize) -> Option<&mut Extruder> {
        debug_assert!(extruder_id < self.m_curr_filament_extruder.len());
        let idx = self.m_curr_filament_extruder[extruder_id]?;
        Some(&mut self.m_filament_extruders[idx])
    }
}

impl GCodeWriterFilamentAt for GCodeWriter {
    // GCodeWriter.hpp:44
    fn filament(&self, extruder_id: usize) -> Option<&Extruder> {
        debug_assert!(extruder_id < self.m_curr_filament_extruder.len());
        self.m_curr_filament_extruder[extruder_id].map(|i| &self.m_filament_extruders[i])
    }
}

impl GCodeWriterFilamentMut for GCodeWriter {
    // GCodeWriter.hpp:45
    fn filament(&mut self) -> Option<&mut Extruder> {
        if self.m_curr_extruder_id == -1 {
            return None;
        }
        let idx = self.m_curr_filament_extruder[self.m_curr_extruder_id as usize]?;
        Some(&mut self.m_filament_extruders[idx])
    }
}

impl GCodeWriterTravelToXyShort for GCodeWriter {
    // GCodeWriter.cpp:464-479
    fn travel_to_xy(&mut self, point: Vec2d, comment: &str, use_short_travel_acceleration: bool) -> String {
        // GCodeWriter.cpp:466-467
        self.m_pos.x = point.x;
        self.m_pos.y = point.y;

        // GCodeWriter.cpp:469
        self.set_current_position_clear(true);
        // GCodeWriter.cpp:470-471  BBS: take plate offset into consider
        let point_on_plate = Vec2d::new(point.x - self.m_x_offset, point.y - self.m_y_offset);

        // GCodeWriter.cpp:473-477
        let mut w = GCodeG1Formatter::new();
        w.emit_xy(point_on_plate);
        w.emit_f(self.config.travel_speed.get_at(self.m_current_process_config_idx) * 60.0);
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        // GCodeWriter.cpp:478
        let accel =
            <GCodeWriter as GCodeWriterSetTravelAccelerationShort>::set_travel_acceleration(self, use_short_travel_acceleration);
        accel + &w.string()
    }
}

impl GCodeWriterTravelToXyzShort for GCodeWriter {
    // GCodeWriter.cpp:602-744
    fn travel_to_xyz(&mut self, point: Vec3d, comment: &str, use_short_travel_acceleration: bool) -> String {
        // GCodeWriter.cpp:604-607  FIXME: This function was not being used when travel_speed_z was
        // separated (bd6badf). Calculation of feedrate was not updated accordingly. If you want to
        // use this function, fix it first.

        // GCodeWriter.cpp:609-612  If target Z is lower than current Z but higher than nominal Z we
        // don't perform the Z move but we only move in the XY plane and adjust the nominal Z by
        // reducing the lift amount that will be used for unlift.
        // GCodeWriter.cpp:614
        let mut dest_point = point;
        // GCodeWriter.cpp:615-616  BBS: a z_hop need to be handle when travel
        if self.m_to_lift.abs() > EPSILON {
            debug_assert!(self.m_lifted.abs() < EPSILON);
            // GCodeWriter.cpp:618-625  BBS: don't need to do real lift if the current position is
            // absolutely same with target.
            if (!self.is_current_position_clear() || self.m_pos != dest_point) && self.m_to_lift + self.m_pos.z > point.z {
                self.m_lifted = self.m_to_lift + self.m_pos.z - point.z;
                dest_point.z = self.m_to_lift + self.m_pos.z;
            }
            // GCodeWriter.cpp:626
            self.m_to_lift = 0.0;

            // GCodeWriter.cpp:628
            let mut slop_move = String::new();
            // GCodeWriter.cpp:629-631  BBS: minus plate offset
            let source = Vec3d::new(self.m_pos.x - self.m_x_offset, self.m_pos.y - self.m_y_offset, self.m_pos.z);
            let target = Vec3d::new(dest_point.x - self.m_x_offset, dest_point.y - self.m_y_offset, dest_point.z);
            // GCodeWriter.cpp:632-633
            let delta = target - source;
            let delta_no_z = Vec2d::new(delta.x, delta.y);
            // GCodeWriter.cpp:634-636  BBS: don'need slope travel because we don't know where is the
            // source position the first time; also don't need to do slope move or spiral lift if
            // x-y distance is absolute zero
            if delta.z > 0.0 && delta_no_z.length() != 0.0 {
                // GCodeWriter.cpp:637-638  BBS: SpiralLift
                if self.m_to_lift_type == LiftType::SpiralLift && self.is_current_position_clear() {
                    // GCodeWriter.cpp:639
                    let radius = delta.z / (2.0 * PI * SLOPE_THRESHOLD.atan());
                    // GCodeWriter.cpp:640-645  The spiral centre sits perpendicular to the travel
                    // direction; keep the arc inside the bed.
                    let mut perp = delta_no_z.normalize() * radius;
                    perp = Vec2d::new(-perp.y, perp.x);
                    // GCodeWriter.cpp:646-648
                    let source_xy = Vec2d::new(source.x, source.y);
                    let mut found = false;
                    let mut ij_offset = perp;
                    // GCodeWriter.cpp:649-655
                    if self.spiral_arc_within_bed(&Vec2d::new(source_xy.x + perp.x, source_xy.y + perp.y), radius) {
                        ij_offset = perp;
                        found = true;
                    } else if self.spiral_arc_within_bed(&Vec2d::new(source_xy.x - perp.x, source_xy.y - perp.y), radius) {
                        ij_offset = Vec2d::new(-perp.x, -perp.y);
                        found = true;
                    }
                    // GCodeWriter.cpp:656-660
                    if found {
                        slop_move = self._spiral_travel_to_z(target.z, ij_offset, "spiral lift Z", false);
                    } else {
                        // no in-bed spiral direction, fall back to a plain vertical lift
                        slop_move = self._travel_to_z(target.z, "normal lift Z", false);
                    }
                }
                // GCodeWriter.cpp:662-665  BBS: SlopeLift
                else if self.m_to_lift_type == LiftType::SlopeLift
                    && self.is_current_position_clear()
                    && delta.z.atan2(delta_no_z.length()) < SLOPE_THRESHOLD
                {
                    // GCodeWriter.cpp:666-669  BBS: check whether we can make a travel like
                    //   _____
                    //  /       to make the z list early to avoid to hit some warping place when travel is long.
                    let temp = delta_no_z.normalize() * delta.z / SLOPE_THRESHOLD.tan();
                    // GCodeWriter.cpp:670
                    let slope_top_point = Vec3d::new(temp.x, temp.y, delta.z) + source;
                    // GCodeWriter.cpp:671-676
                    let mut w0 = GCodeG1Formatter::new();
                    w0.emit_xyz(slope_top_point);
                    w0.emit_f(self.config.travel_speed.get_at(self.m_current_process_config_idx) * 60.0);
                    w0.emit_comment(FULL_GCODE_COMMENT, "slope lift Z");
                    slop_move = w0.string();
                }
                // GCodeWriter.cpp:678-680
                else if self.m_to_lift_type == LiftType::NormalLift {
                    slop_move = self._travel_to_z(target.z, "normal lift Z", false);
                }
            }

            // GCodeWriter.cpp:683
            let xy_z_move: String;
            {
                // GCodeWriter.cpp:685
                let mut w0 = GCodeG1Formatter::new();
                // GCodeWriter.cpp:686
                if self.is_current_position_clear() {
                    // GCodeWriter.cpp:687-690
                    w0.emit_xyz(target);
                    w0.emit_f(self.config.travel_speed.get_at(self.m_current_process_config_idx) * 60.0);
                    w0.emit_comment(FULL_GCODE_COMMENT, comment);
                    xy_z_move = w0.string();
                } else {
                    // GCodeWriter.cpp:693-696
                    w0.emit_xy(Vec2d::new(target.x, target.y));
                    w0.emit_f(self.config.travel_speed.get_at(self.m_current_process_config_idx) * 60.0);
                    w0.emit_comment(FULL_GCODE_COMMENT, comment);
                    let s = w0.string();
                    xy_z_move = s + &self._travel_to_z(target.z, comment, false);
                }
            }
            // GCodeWriter.cpp:699-701
            self.m_pos = dest_point;
            self.set_current_position_clear(true);
            let accel = <GCodeWriter as GCodeWriterSetTravelAccelerationShort>::set_travel_acceleration(
                self,
                use_short_travel_acceleration,
            );
            return accel + &slop_move + &xy_z_move;
        }
        // GCodeWriter.cpp:703
        else if !self.will_move_z(point.z) {
            // GCodeWriter.cpp:704-709  In case that z_hop == layer_height we could end up with
            // almost zero in_m_lifted and a retract could be skipped
            let nominal_z = self.m_pos.z - self.m_lifted;
            self.m_lifted -= point.z - nominal_z;
            if self.m_lifted.abs() < EPSILON {
                self.m_lifted = 0.0;
            }
            // GCodeWriter.cpp:710-712  BBS
            self.set_current_position_clear(true);
            return <GCodeWriter as GCodeWriterTravelToXyShort>::travel_to_xy(
                self,
                Vec2d::new(point.x, point.y),
                "",
                use_short_travel_acceleration,
            );
        } else {
            // GCodeWriter.cpp:714-718  In all the other cases, we perform an actual XYZ move and
            // cancel the lift.
            self.m_lifted = 0.0;
        }

        // GCodeWriter.cpp:720-721  BBS: take plate offset into consider
        let point_on_plate = Vec3d::new(dest_point.x - self.m_x_offset, dest_point.y - self.m_y_offset, dest_point.z);
        // GCodeWriter.cpp:722-723
        let out_string: String;
        // GCodeWriter.cpp:724-725
        if !self.is_current_position_clear() || (self.m_avoid_z_descent_travel && point_on_plate.z < self.m_pos.z - EPSILON) {
            // GCodeWriter.cpp:727-732  Split XY + Z: required after filament change (position
            // unknown), or when mixed sub-layer Z must descend to avoid diagonal collision.
            let mut w = GCodeG1Formatter::new();
            w.emit_xy(Vec2d::new(point_on_plate.x, point_on_plate.y));
            w.emit_f(self.config.travel_speed.get_at(self.m_current_process_config_idx) * 60.0);
            w.emit_comment(FULL_GCODE_COMMENT, comment);
            let s = w.string();
            out_string = s + &self._travel_to_z(point_on_plate.z, comment, false);
        } else {
            // GCodeWriter.cpp:734-738
            let mut w = GCodeG1Formatter::new();
            w.emit_xyz(point_on_plate);
            w.emit_f(self.config.travel_speed.get_at(self.m_current_process_config_idx) * 60.0);
            w.emit_comment(FULL_GCODE_COMMENT, comment);
            out_string = w.string();
        }

        // GCodeWriter.cpp:741-743
        self.m_pos = dest_point;
        self.set_current_position_clear(true);
        let accel =
            <GCodeWriter as GCodeWriterSetTravelAccelerationShort>::set_travel_acceleration(self, use_short_travel_acceleration);
        accel + &out_string
    }
}

impl GCodeWriterSetTravelAccelerationNow for GCodeWriter {
    // GCodeWriter.cpp:261-264
    fn set_travel_acceleration(&mut self) -> String {
        <GCodeWriter as GCodeWriterSetTravelAccelerationShort>::set_travel_acceleration(self, false)
    }
}

impl GCodeWriterSetTravelAccelerationShort for GCodeWriter {
    // GCodeWriter.cpp:266-286
    fn set_travel_acceleration(&mut self, use_short_travel_acceleration: bool) -> String {
        // GCodeWriter.cpp:268
        let travel_accelerations = if self.m_is_first_layer {
            self.m_first_layer_travel_accelerations.clone()
        } else {
            self.m_travel_accelerations.clone()
        };
        // GCodeWriter.cpp:269-270
        if travel_accelerations.is_empty() {
            return String::new();
        }

        // GCodeWriter.cpp:272-274
        let extruder_id = match GCodeWriter::filament(self) {
            None => return String::new(),
            // GCodeWriter.cpp:276
            Some(cur_filament) => cur_filament.extruder_id() as usize,
        };

        // GCodeWriter.cpp:278-283  Use short travel acceleration if requested and available
        if use_short_travel_acceleration
            && extruder_id < self.m_travel_short_accelerations.len()
            && self.m_travel_short_accelerations[extruder_id] > 0
        {
            return self.set_acceleration_impl(self.m_travel_short_accelerations[extruder_id]);
        }

        // GCodeWriter.cpp:285
        self.set_acceleration_impl(travel_accelerations[extruder_id])
    }
}

impl GCodeWriterSetFanFlavor for GCodeWriter {
    // GCodeWriter.cpp:982-1014
    fn set_fan(gcode_flavor: GCodeFlavor, speed: u32) -> String {
        // GCodeWriter.cpp:984
        let mut gcode = String::new();
        // GCodeWriter.cpp:985
        if speed == 0 {
            // GCodeWriter.cpp:986-994
            match gcode_flavor {
                GCodeFlavor::Teacup => gcode.push_str("M106 S0"),
                GCodeFlavor::MakerWare | GCodeFlavor::Sailfish => gcode.push_str("M127"),
                _ => gcode.push_str("M106 S0"),
            }
            // GCodeWriter.cpp:995-996
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; disable fan");
            }
            // GCodeWriter.cpp:997
            gcode.push('\n');
        } else {
            // GCodeWriter.cpp:999-1008
            match gcode_flavor {
                GCodeFlavor::MakerWare | GCodeFlavor::Sailfish => gcode.push_str("M126"),
                GCodeFlavor::Mach3 | GCodeFlavor::Machinekit => {
                    let _ = write!(gcode, "M106 P{}", ostream_double(255.0 * speed as f64 / 100.0));
                }
                _ => {
                    let _ = write!(gcode, "M106 S{}", ostream_double(255.0 * speed as f64 / 100.0));
                }
            }
            // GCodeWriter.cpp:1009-1010
            if FULL_GCODE_COMMENT {
                gcode.push_str(" ; enable fan");
            }
            // GCodeWriter.cpp:1011
            gcode.push('\n');
        }
        // GCodeWriter.cpp:1013
        gcode
    }
}

// `std::ostringstream << double` with the stream's defaults (`std::defaultfloat`, precision 6),
// which is `printf("%.*g", precision, v)`: at most `precision` significant digits, scientific
// notation when the decimal exponent is < -4 or >= precision, trailing zeros (and a bare decimal
// point) trimmed.
fn format_g(v: f64, precision: usize) -> String {
    if !v.is_finite() {
        // %g prints "inf"/"nan"; not expected here but keep it total.
        return format!("{}", v);
    }
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".to_string() } else { "0".to_string() };
    }
    // %g uses P = max(precision, 1) significant digits, and chooses between fixed and scientific
    // on the decimal exponent of the value rounded to P significant digits.
    let p = precision.max(1);
    let sci = format!("{:.*e}", p - 1, v);
    let exp: i32 = sci.split(['e', 'E']).nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);
    if exp < -4 || exp >= p as i32 {
        // Scientific style "%e" with (P-1) digits after the point, then trimmed; the exponent has
        // at least two digits.
        let (mant, e) = sci.split_once('e').unwrap_or((sci.as_str(), "0"));
        let mant_trimmed = if mant.contains('.') { mant.trim_end_matches('0').trim_end_matches('.') } else { mant };
        let e_num: i32 = e.parse().unwrap_or(0);
        let sign = if e_num < 0 { '-' } else { '+' };
        format!("{}e{}{:02}", mant_trimmed, sign, e_num.abs())
    } else {
        // Fixed style "%f" with (P - 1 - exp) digits after the point, then trimmed.
        let frac_digits = (p as i32 - 1 - exp).max(0) as usize;
        let fixed = format!("{:.*}", frac_digits, v);
        if fixed.contains('.') {
            fixed.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            fixed
        }
    }
}

// `std::ostringstream << double` at the default precision of 6 (`%.6g`): set_jerk_xy
// (GCodeWriter.cpp:362/364), set_fan (GCodeWriter.cpp:1005/1007), set_acceleration_impl
// (GCodeWriter.cpp:316).
pub(crate) fn ostream_double(v: f64) -> String {
    format_g(v, 6)
}

// `std::setprecision(4)` (without std::fixed) on a stream: `%.4g`. set_pressure_advance
// (GCodeWriter.cpp:337-346).
fn format_setprecision_4(v: f64) -> String {
    format_g(v, 4)
}

// ===========================================================================
// GCodeFormatter / GCodeG1Formatter / GCodeG2G3Formatter (GCodeWriter.hpp:209-314,
// GCodeWriter.cpp:1097-1154)
// ===========================================================================

// GCodeWriter.hpp:225-226
//   static constexpr const int XYZF_EXPORT_DIGITS = 3;
//   static constexpr const int E_EXPORT_DIGITS    = 5;
const XYZF_EXPORT_DIGITS: usize = 3;
const E_EXPORT_DIGITS: usize = 5;

// GCodeWriter.hpp:284  static constexpr const size_t buflen = 256;
const BUFLEN: usize = 256;

// GCodeWriter.hpp:209  class GCodeFormatter
pub struct GCodeFormatter {
    // GCodeWriter.hpp:285  char buf[buflen];
    buf: [u8; BUFLEN],
    // GCodeWriter.hpp:287  std::to_chars_result ptr_err; (we track the write cursor index)
    ptr: usize,
}

impl GCodeFormatter {
    // GCodeWriter.hpp:211-214  GCodeFormatter()
    pub fn new() -> Self {
        // GCodeWriter.hpp:212  this->buf_end = buf + buflen;
        // GCodeWriter.hpp:213  this->ptr_err.ptr = this->buf;
        GCodeFormatter {
            buf: [0u8; BUFLEN],
            ptr: 0,
        }
    }

    // GCodeWriter.cpp:1097  void GCodeFormatter::emit_axis(const char axis, const double v, size_t digits)
    pub fn emit_axis(&mut self, axis: u8, v: f64, digits: usize) {
        // GCodeWriter.cpp:1098  assert(digits <= 9);
        debug_assert!(digits <= 9);
        // GCodeWriter.cpp:1099  static constexpr const std::array<int,10> pow_10{...};
        const POW_10: [i64; 10] = [
            1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000,
        ];
        // GCodeWriter.cpp:1100  *ptr_err.ptr++ = ' '; *ptr_err.ptr++ = axis;
        self.buf[self.ptr] = b' ';
        self.ptr += 1;
        self.buf[self.ptr] = axis;
        self.ptr += 1;

        // GCodeWriter.cpp:1102  char *base_ptr = this->ptr_err.ptr;
        let base_ptr = self.ptr;
        // GCodeWriter.cpp:1103  auto v_int = int64_t(std::round(v * pow_10[digits]));
        let v_int = (v * POW_10[digits] as f64).round() as i64;
        // GCodeWriter.cpp:1106-1111  write integer digits (to_chars / karma::generate).
        let int_str = v_int.to_string();
        for &b in int_str.as_bytes() {
            self.buf[self.ptr] = b;
            self.ptr += 1;
        }
        // GCodeWriter.cpp:1112  size_t writen_digits = (ptr - base_ptr) - (v_int < 0 ? 1 : 0);
        let writen_digits = (self.ptr - base_ptr) - if v_int < 0 { 1 } else { 0 };
        // GCodeWriter.cpp:1113  if (writen_digits < digits)
        if writen_digits < digits {
            // GCodeWriter.cpp:1115  size_t remaining_digits = digits - writen_digits;
            let remaining_digits = digits - writen_digits;
            // GCodeWriter.cpp:1117-1118  Move all newly inserted chars by remaining_digits.
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
            // GCodeWriter.cpp:1120  memset(ptr - writen_digits, '0', remaining_digits);
            for i in 0..remaining_digits {
                self.buf[self.ptr - writen_digits + i] = b'0';
            }
            // GCodeWriter.cpp:1121  ptr += remaining_digits;
            self.ptr += remaining_digits;
        }

        // GCodeWriter.cpp:1125-1126  Move all newly inserted chars by one for a decimal point.
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

        // GCodeWriter.cpp:1128  *(ptr - digits) = '.';
        self.buf[self.ptr - digits] = b'.';
        // GCodeWriter.cpp:1129-1133  for (i=0;i<digits;++i){ if(*ptr!='0')break; ptr--; }
        for _ in 0..digits {
            if self.buf[self.ptr] != b'0' {
                break;
            }
            self.ptr -= 1;
        }
        // GCodeWriter.cpp:1134-1135  if (*ptr == '.') ptr--;
        if self.buf[self.ptr] == b'.' {
            self.ptr -= 1;
        }
        // GCodeWriter.cpp:1136-1137  if ((ptr+1)==base_ptr || *ptr=='-') *(++ptr) = '0';
        if (self.ptr + 1) == base_ptr || self.buf[self.ptr] == b'-' {
            self.ptr += 1;
            self.buf[self.ptr] = b'0';
        }
        // GCodeWriter.cpp:1138  ptr++;
        self.ptr += 1;
    }

    // GCodeWriter.hpp:238  void emit_xy(const Vec2d &point)
    pub fn emit_xy(&mut self, point: Vec2d) {
        // GCodeWriter.hpp:239
        self.emit_axis(b'X', point.x(), XYZF_EXPORT_DIGITS);
        // GCodeWriter.hpp:240
        self.emit_axis(b'Y', point.y(), XYZF_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:243  void emit_xyz(const Vec3d &point)
    pub fn emit_xyz(&mut self, point: Vec3d) {
        // GCodeWriter.hpp:244
        self.emit_axis(b'X', point.x, XYZF_EXPORT_DIGITS);
        // GCodeWriter.hpp:245
        self.emit_axis(b'Y', point.y, XYZF_EXPORT_DIGITS);
        // GCodeWriter.hpp:246
        self.emit_z(point.z);
    }

    // GCodeWriter.hpp:249  void emit_z(const double z)
    pub fn emit_z(&mut self, z: f64) {
        // GCodeWriter.hpp:250
        self.emit_axis(b'Z', z, XYZF_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:253  void emit_e(double v)
    pub fn emit_e(&mut self, v: f64) {
        // GCodeWriter.hpp:254
        self.emit_axis(b'E', v, E_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:257  void emit_f(double speed)
    pub fn emit_f(&mut self, speed: f64) {
        // GCodeWriter.hpp:258
        self.emit_axis(b'F', speed, XYZF_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:261  void emit_ij(const Vec2d &point)
    pub fn emit_ij(&mut self, point: Vec2d) {
        // GCodeWriter.hpp:262
        self.emit_axis(b'I', point.x(), XYZF_EXPORT_DIGITS);
        // GCodeWriter.hpp:263
        self.emit_axis(b'J', point.y(), XYZF_EXPORT_DIGITS);
    }

    // GCodeWriter.hpp:266  void emit_string(const std::string &s)
    pub fn emit_string(&mut self, s: &str) {
        // GCodeWriter.hpp:267-268  strncpy(ptr, s.c_str(), s.size()); ptr += s.size();
        for &b in s.as_bytes() {
            self.buf[self.ptr] = b;
            self.ptr += 1;
        }
    }

    // GCodeWriter.hpp:271  void emit_comment(bool allow_comments, const std::string &comment)
    pub fn emit_comment(&mut self, allow_comments: bool, comment: &str) {
        // GCodeWriter.hpp:272  if (allow_comments && ! comment.empty())
        if allow_comments && !comment.is_empty() {
            // GCodeWriter.hpp:273  *ptr++ = ' '; *ptr++ = ';'; *ptr++ = ' ';
            self.buf[self.ptr] = b' ';
            self.ptr += 1;
            self.buf[self.ptr] = b';';
            self.ptr += 1;
            self.buf[self.ptr] = b' ';
            self.ptr += 1;
            // GCodeWriter.hpp:274
            self.emit_string(comment);
        }
    }

    // GCodeWriter.hpp:278  std::string string()
    pub fn string(&mut self) -> String {
        // GCodeWriter.hpp:279  *ptr++ = '\n';
        self.buf[self.ptr] = b'\n';
        self.ptr += 1;
        // GCodeWriter.hpp:280  return std::string(this->buf, ptr - buf);
        String::from_utf8_lossy(&self.buf[0..self.ptr]).into_owned()
    }
}

impl Default for GCodeFormatter {
    fn default() -> Self {
        Self::new()
    }
}

// GCodeWriter.hpp:290  class GCodeG1Formatter : public GCodeFormatter
pub struct GCodeG1Formatter {
    inner: GCodeFormatter,
}

impl GCodeG1Formatter {
    // GCodeWriter.hpp:292-297  GCodeG1Formatter()
    pub fn new() -> Self {
        let mut inner = GCodeFormatter::new();
        // GCodeWriter.hpp:293  this->buf[0] = 'G';
        inner.buf[0] = b'G';
        // GCodeWriter.hpp:294  this->buf[1] = '1';
        inner.buf[1] = b'1';
        // GCodeWriter.hpp:296  this->ptr_err.ptr = this->buf + 2;
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

// GCodeWriter.hpp:303  class GCodeG2G3Formatter : public GCodeFormatter
pub struct GCodeG2G3Formatter {
    inner: GCodeFormatter,
}

impl GCodeG2G3Formatter {
    // GCodeWriter.hpp:305-310  GCodeG2G3Formatter(bool is_ccw)
    pub fn new(is_ccw: bool) -> Self {
        let mut inner = GCodeFormatter::new();
        // GCodeWriter.hpp:306  this->buf[0] = 'G';
        inner.buf[0] = b'G';
        // GCodeWriter.hpp:307  this->buf[1] = is_ccw ? '3' : '2';
        inner.buf[1] = if is_ccw { b'3' } else { b'2' };
        // GCodeWriter.hpp:309  this->ptr_err.ptr = this->buf + 2;
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
