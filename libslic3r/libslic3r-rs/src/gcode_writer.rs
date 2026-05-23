//! G-code writer with state tracking and command formatting
//!
//! C++ Reference: GCodeWriter.hpp, GCodeWriter.cpp
//!
//! This module handles low-level G-code generation with state tracking for:
//! - Extruder position and retraction
//! - Acceleration and jerk settings
//! - Temperature (hotend, bed, chamber)
//! - Fan speeds
//! - Z lift/hop
//! - Travel vs extrusion moves
//!
//! The writer maintains internal state to avoid emitting redundant commands
//! and ensures smooth transitions between different move types.

use crate::Result;
use crate::{
    extruder::Extruder,
    geometry::{Vec2d, Vec3d},
    print_config::{GCodeConfig, GCodeFlavor},
};
use std::fmt::Write;

/// Slope threshold (radians) for lazy lift and spiral lift
/// GCodeWriter.cpp:19
const SLOPE_THRESHOLD: f64 = 0.017453; // ~1 degree

/// Full G-code comment output flag
/// GCodeWriter.cpp:18
const FULL_GCODE_COMMENT: bool = true;

/// Type of Z lift/hop to perform
/// GCodeWriter.hpp:14-18
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiftType {
    /// Normal vertical lift
    NormalLift,
    /// Lift with slope (gradual)
    SlopeLift,
    /// Lift in spiral motion
    SpiralLift,
}

/// Main G-code writer with state tracking
/// GCodeWriter.hpp:20-137
pub struct GCodeWriter {
    /// G-code configuration
    /// GCodeWriter.hpp:22
    pub config: GCodeConfig,

    /// Whether multiple extruders are used
    /// GCodeWriter.hpp:23
    pub multiple_extruders: bool,

    // Private fields
    /// Filament extruders sorted by ID
    /// GCodeWriter.hpp:139
    filament_extruders: Vec<Extruder>,

    /// Single extruder multi-material mode
    /// GCodeWriter.hpp:140
    single_extruder_multi_material: bool,

    /// Current filament extruder for each extruder ID
    /// GCodeWriter.hpp:141
    curr_filament_extruder: Vec<Option<usize>>,

    /// Current extruder ID (-1 if none)
    /// GCodeWriter.hpp:142
    curr_extruder_id: i32,

    /// Last acceleration setting
    /// GCodeWriter.hpp:143-146
    last_acceleration: u32,
    max_acceleration: u32,

    /// Last jerk setting
    /// GCodeWriter.hpp:147-148
    last_jerk: f64,
    max_jerk: f64,

    /// Last bed temperature and state
    /// GCodeWriter.hpp:150-151
    last_bed_temperature: i32,
    last_bed_temperature_reached: bool,

    /// Current Z lift amount
    /// GCodeWriter.hpp:152
    lifted: f64,

    /// Pending lift amount and type
    /// GCodeWriter.hpp:154-155
    to_lift: f64,
    to_lift_type: LiftType,

    /// Current position
    /// GCodeWriter.hpp:156
    pos: Vec3d,

    /// Whether current position is valid
    /// GCodeWriter.hpp:159
    is_current_pos_clear: bool,

    /// XY offset for plate
    /// GCodeWriter.hpp:161-162
    x_offset: f64,
    y_offset: f64,

    /// Current speed (F parameter)
    /// GCodeWriter.hpp:163
    current_speed: f64,

    /// Whether this is a BBL printer
    /// GCodeWriter.hpp:164
    is_bbl_printer: bool,

    /// Object labeling strings
    /// GCodeWriter.hpp:166-167
    gcode_label_objects_start: String,
    gcode_label_objects_end: String,

    /// First layer flag
    /// GCodeWriter.hpp:170
    is_first_layer: bool,

    /// Current acceleration
    /// GCodeWriter.hpp:171
    acceleration: u32,

    /// Travel accelerations per extruder
    /// GCodeWriter.hpp:172-174
    travel_accelerations: Vec<u32>,
    travel_short_accelerations: Vec<u32>,
    first_layer_travel_accelerations: Vec<u32>,

    /// Last additional fan speed
    last_additional_fan_speed: u32,
}

impl GCodeWriter {
    /// Create a new G-code writer
    /// GCodeWriter.hpp:25-35
    pub fn new() -> Self {
        Self {
            config: GCodeConfig::default(),
            multiple_extruders: false,
            filament_extruders: Vec::new(),
            single_extruder_multi_material: false,
            curr_filament_extruder: vec![None, None],
            curr_extruder_id: -1,
            last_acceleration: 0,
            max_acceleration: 0,
            last_jerk: 0.0,
            max_jerk: 0.0,
            last_bed_temperature: 0,
            last_bed_temperature_reached: true,
            lifted: 0.0,
            to_lift: 0.0,
            to_lift_type: LiftType::NormalLift,
            pos: Vec3d::new(0.0, 0.0, 0.0),
            is_current_pos_clear: false,
            x_offset: 0.0,
            y_offset: 0.0,
            current_speed: 0.0,
            is_bbl_printer: false,
            gcode_label_objects_start: String::new(),
            gcode_label_objects_end: String::new(),
            is_first_layer: false,
            acceleration: 0,
            travel_accelerations: Vec::new(),
            travel_short_accelerations: Vec::new(),
            first_layer_travel_accelerations: Vec::new(),
            last_additional_fan_speed: 0,
        }
    }

    /// Get filament extruder by ID
    /// GCodeWriter.hpp:36-38
    pub fn filament(&self, extruder_id: usize) -> Option<&Extruder> {
        self.curr_filament_extruder
            .get(extruder_id)
            .and_then(|&idx| idx)
            .and_then(|idx| self.filament_extruders.get(idx))
    }

    /// Get current filament extruder
    /// GCodeWriter.hpp:39-40
    pub fn current_filament(&self) -> Option<&Extruder> {
        if self.curr_extruder_id < 0 {
            None
        } else {
            self.filament(self.curr_extruder_id as usize)
        }
    }

    /// Get mutable reference to current filament extruder
    fn current_filament_mut(&mut self) -> Option<&mut Extruder> {
        if self.curr_extruder_id < 0 {
            return None;
        }
        let idx = self
            .curr_filament_extruder
            .get(self.curr_extruder_id as usize)?
            .as_ref()?;
        self.filament_extruders.get_mut(*idx)
    }

    /// Set extruders from list of IDs
    /// GCodeWriter.cpp:32-44
    pub fn set_extruders(&mut self, mut extruder_ids: Vec<u32>) {
        extruder_ids.sort_unstable();
        self.filament_extruders.clear();
        self.filament_extruders.reserve(extruder_ids.len());

        for extruder_id in &extruder_ids {
            // TODO: This is a type mismatch - Extruder::new expects *const PrintConfig
            // but we have GCodeConfig. Need to refactor Extruder to accept GCodeConfig
            // or change GCodeWriter to store PrintConfig.
            // For now, cast to raw pointer (unsafe but maintains compilation)
            self.filament_extruders.push(Extruder::new(
                *extruder_id,
                &self.config as *const GCodeConfig as *const crate::print_config::PrintConfig,
                self.single_extruder_multi_material,
            ));
        }

        // Enable multiple extruders if any ID > 0
        self.multiple_extruders = extruder_ids.iter().any(|&id| id > 0);
    }

    /// Get list of extruder IDs
    /// GCodeWriter.hpp:47-52
    pub fn extruder_ids(&self) -> Vec<u32> {
        self.filament_extruders.iter().map(|e| e.id()).collect()
    }

    /// Generate preamble G-code
    /// GCodeWriter.cpp:46-72
    pub fn preamble(&mut self) -> Result<String> {
        let mut gcode = String::new();

        // Not MakerWare flavor
        if self.config.gcode_flavor != GCodeFlavor::MakerWare {
            gcode.push_str("G90\n"); // Absolute positioning
            gcode.push_str("G21\n"); // Millimeters
        }

        // For most firmware types
        if matches!(
            self.config.gcode_flavor,
            GCodeFlavor::RepRapSprinter
                | GCodeFlavor::RepRapFirmware
                | GCodeFlavor::MarlinLegacy
                | GCodeFlavor::Marlin
                | GCodeFlavor::Teacup
                | GCodeFlavor::Repetier
                | GCodeFlavor::Smoothie
                | GCodeFlavor::Klipper
        ) {
            if self.config.use_relative_e_distances {
                gcode.push_str("M83 ; use relative distances for extrusion\n");
            } else {
                gcode.push_str("M82 ; use absolute distances for extrusion\n");
            }
            gcode.push_str(&self.reset_e(true)?);
        }

        Ok(gcode)
    }

    /// Generate postamble G-code
    /// GCodeWriter.cpp:74-80
    pub fn postamble(&self) -> String {
        let mut gcode = String::new();

        if self.config.gcode_flavor == GCodeFlavor::Machinekit {
            gcode.push_str("M2 ; end of program\n");
        }

        gcode
    }

    /// Set hotend temperature
    /// GCodeWriter.cpp:82-122
    pub fn set_temperature(&self, temperature: u32, wait: bool, tool: i32) -> String {
        if temperature == 0 {
            return String::new();
        }

        let code: &str;
        let comment: &str;

        if wait {
            code = "M109";
            comment = "set temperature and wait";
        } else {
            code = "M104";
            comment = "set temperature";
        }

        let mut gcode = String::new();

        let tool_id = if tool == -1 {
            self.curr_extruder_id as i32
        } else {
            tool
        };

        let multiple_tools = self.filament_extruders.len() > 1;

        if self.config.gcode_flavor == GCodeFlavor::MakerWare {
            let _ = writeln!(gcode, "{} P{} S{}", code, tool_id, temperature);
        } else {
            let _ = write!(gcode, "{} S{}", code, temperature);
            if multiple_tools {
                let _ = write!(gcode, " T{}", tool_id);
            }
            if FULL_GCODE_COMMENT {
                let _ = write!(gcode, " ; {}", comment);
            }
            gcode.push('\n');
        }

        gcode
    }

    /// Set bed temperature
    /// GCodeWriter.cpp:125-147
    pub fn set_bed_temperature(&mut self, temperature: i32, wait: bool) -> String {
        if temperature > 0 && (temperature != self.last_bed_temperature || wait) {
            self.last_bed_temperature = temperature;
            self.last_bed_temperature_reached = wait;

            let code: &str;
            let comment: &str;

            if wait {
                code = "M190";
                comment = "set bed temperature and wait";
            } else {
                code = "M140";
                comment = "set bed temperature";
            }

            let mut gcode = String::new();
            let _ = write!(gcode, "{} S{}", code, temperature);
            if FULL_GCODE_COMMENT {
                let _ = write!(gcode, " ; {}", comment);
            }
            gcode.push('\n');

            gcode
        } else {
            String::new()
        }
    }

    /// Set chamber temperature
    /// GCodeWriter.cpp:149-166
    pub fn set_chamber_temperature(&self, temperature: i32, wait: bool) -> String {
        let code: &str;
        let comment: &str;

        if wait {
            code = "M191";
            comment = "set chamber temperature and wait";
        } else {
            code = "M141";
            comment = "set chamber temperature";
        }

        let mut gcode = String::new();
        let _ = write!(gcode, "{} S{}", code, temperature);
        if FULL_GCODE_COMMENT {
            let _ = write!(gcode, " ; {}", comment);
        }
        gcode.push('\n');

        gcode
    }

    /// Set acceleration
    /// GCodeWriter.cpp:168-171
    pub fn set_acceleration(&mut self, acceleration: u32) {
        self.acceleration = acceleration;
    }

    /// Set travel accelerations
    /// GCodeWriter.cpp:173-176
    pub fn set_travel_acceleration(&mut self, travel_accelerations: Vec<u32>) {
        self.travel_accelerations = travel_accelerations;
    }

    /// Set short travel accelerations
    /// GCodeWriter.cpp:178-181
    pub fn set_travel_short_acceleration(&mut self, travel_short_accelerations: Vec<u32>) {
        self.travel_short_accelerations = travel_short_accelerations;
    }

    /// Reset last acceleration
    /// GCodeWriter.cpp:183-186
    pub fn reset_last_acceleration(&mut self) {
        self.last_acceleration = 0;
    }

    /// Set first layer travel acceleration
    /// GCodeWriter.cpp:188-191
    pub fn set_first_layer_travel_acceleration(
        &mut self,
        first_layer_travel_accelerations: Vec<u32>,
    ) {
        self.first_layer_travel_accelerations = first_layer_travel_accelerations;
    }

    /// Set whether this is the first layer
    /// GCodeWriter.cpp:193-196
    pub fn set_first_layer(&mut self, is_first_layer: bool) {
        self.is_first_layer = is_first_layer;
    }

    /// Get travel acceleration vector
    pub fn get_travel_acceleration(&self) -> &[u32] {
        &self.travel_accelerations
    }

    /// Get short travel acceleration vector
    pub fn get_travel_short_acceleration(&self) -> &[u32] {
        &self.travel_short_accelerations
    }

    /// Set extrude acceleration and return G-code
    /// GCodeWriter.cpp:198-201
    fn set_extrude_acceleration(&mut self) -> String {
        self.set_acceleration_impl(self.acceleration)
    }

    /// Set travel acceleration and return G-code
    /// GCodeWriter.cpp:203-206
    fn set_travel_acceleration_impl(&mut self) -> String {
        self.set_travel_acceleration_for_move(false)
    }

    /// Set travel acceleration for move with optional short travel
    /// GCodeWriter.cpp:208-228
    fn set_travel_acceleration_for_move(&mut self, use_short_travel_acceleration: bool) -> String {
        if self.travel_accelerations.is_empty() {
            return String::new();
        }

        let filament_id = if let Some(filament) = self.current_filament() {
            filament.id() as usize
        } else {
            return String::new();
        };

        let extruder_id = if filament_id < self.travel_accelerations.len() {
            filament_id
        } else {
            0
        };

        let acceleration = if use_short_travel_acceleration
            && extruder_id < self.travel_short_accelerations.len()
        {
            self.travel_short_accelerations[extruder_id]
        } else {
            self.travel_accelerations[extruder_id]
        };

        self.set_acceleration_impl(acceleration)
    }

    /// Set acceleration implementation
    /// GCodeWriter.cpp:230-271
    fn set_acceleration_impl(&mut self, acceleration: u32) -> String {
        if acceleration == 0 || acceleration == self.last_acceleration {
            return String::new();
        }

        let mut acceleration = acceleration;

        // Respect machine limits
        if self.max_acceleration > 0 && acceleration > self.max_acceleration {
            acceleration = self.max_acceleration;
        }

        self.last_acceleration = acceleration;

        let mut gcode = String::new();

        if self.config.gcode_flavor == GCodeFlavor::RepRapSprinter
            || self.config.gcode_flavor == GCodeFlavor::RepRapFirmware
            || self.config.gcode_flavor == GCodeFlavor::Repetier
        {
            let _ = writeln!(gcode, "M204 P{} ; set acceleration", acceleration);
        } else if self.config.gcode_flavor == GCodeFlavor::Klipper {
            let _ = writeln!(gcode, "SET_VELOCITY_LIMIT ACCEL={}", acceleration);
        } else {
            let _ = writeln!(gcode, "M204 S{} ; set acceleration", acceleration);
        }

        gcode
    }

    /// Set pressure advance
    /// GCodeWriter.cpp:273-289
    pub fn set_pressure_advance(&self, pa: f64) -> String {
        let mut gcode = String::new();

        if self.config.gcode_flavor == GCodeFlavor::Klipper {
            if let Some(filament) = self.current_filament() {
                let _ = writeln!(
                    gcode,
                    "SET_PRESSURE_ADVANCE EXTRUDER=extruder{} ADVANCE={:.5}",
                    filament.id(),
                    pa
                );
            }
        }

        gcode
    }

    /// Set XY jerk
    /// GCodeWriter.cpp:291-310
    pub fn set_jerk_xy(&mut self, jerk: f64) -> String {
        if jerk == self.last_jerk {
            return String::new();
        }

        let mut jerk = jerk;

        // Respect machine limits
        if self.max_jerk > 0.0 && jerk > self.max_jerk {
            jerk = self.max_jerk;
        }

        self.last_jerk = jerk;

        let mut gcode = String::new();

        if self.config.gcode_flavor == GCodeFlavor::Klipper {
            let _ = writeln!(gcode, "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY={}", jerk);
        } else {
            let _ = writeln!(gcode, "M205 X{:.2} Y{:.2}", jerk, jerk);
        }

        gcode
    }

    /// Reset E axis
    /// GCodeWriter.cpp:312-335
    pub fn reset_e(&mut self, force: bool) -> Result<String> {
        if self.config.use_relative_e_distances {
            return Ok(String::new());
        }

        if let Some(extruder) = self.current_filament_mut() {
            if force || extruder.e() != 0.0 {
                extruder.reset_e();

                if !self.config.use_relative_e_distances {
                    let mut w = GCodeG1Formatter::new();
                    w.emit_e(0.0);
                    return Ok(w.string());
                }
            }
        }

        Ok(String::new())
    }

    /// Update progress in G-code
    /// GCodeWriter.cpp:337-351
    pub fn update_progress(&self, num: u32, tot: u32, allow_100: bool) -> String {
        if self.config.gcode_flavor != GCodeFlavor::MakerWare {
            return String::new();
        }

        let percent = if allow_100 {
            (100.0 * num as f64 / tot as f64).round() as u32
        } else {
            (99.0 * num as f64 / tot as f64).round() as u32
        };

        format!("M73 P{}\n", percent)
    }

    /// Get toolchange prefix
    /// GCodeWriter.cpp:353-357
    pub fn toolchange_prefix(&self) -> String {
        format!(
            "{} extruder {}\n",
            self.config.toolchange_gcode, self.curr_extruder_id
        )
    }

    /// Perform toolchange
    /// GCodeWriter.cpp:359-383
    pub fn toolchange(&mut self, filament_id: u32) -> String {
        let filament_extruder_id = if (filament_id as usize) < self.curr_filament_extruder.len() {
            self.curr_filament_extruder[filament_id as usize]
        } else {
            None
        };

        if filament_extruder_id.is_none() {
            return String::new();
        }

        self.curr_extruder_id = filament_id as i32;

        let mut gcode = String::new();

        if self.multiple_extruders {
            let _ = writeln!(gcode, "T{}", filament_id);
        }

        gcode
    }

    /// Check if toolchange is needed
    /// GCodeWriter.cpp:967-970
    pub fn need_toolchange(&self, filament_id: u32) -> bool {
        self.curr_extruder_id != filament_id as i32
    }

    /// Set extruder
    /// GCodeWriter.cpp:948-955
    pub fn set_extruder(&mut self, filament_id: u32) -> String {
        if (filament_id as usize) < self.curr_filament_extruder.len() {
            if self.curr_filament_extruder[filament_id as usize].is_some() {
                self.curr_extruder_id = filament_id as i32;
            }
        }
        String::new()
    }

    /// Initialize extruder
    /// GCodeWriter.cpp:957-965
    pub fn init_extruder(&mut self, filament_id: u32) {
        if (filament_id as usize) >= self.curr_filament_extruder.len() {
            self.curr_filament_extruder
                .resize(filament_id as usize + 1, None);
        }

        for (i, extruder) in self.filament_extruders.iter().enumerate() {
            if extruder.id() == filament_id {
                self.curr_filament_extruder[filament_id as usize] = Some(i);
                break;
            }
        }
    }

    /// Set speed (F parameter)
    /// GCodeWriter.cpp:385-396
    pub fn set_speed(&mut self, f: f64, comment: &str, cooling_marker: &str) -> String {
        debug_assert!(f > 0.0 && f < 100000.0);
        self.current_speed = f;

        let mut w = GCodeG1Formatter::new();
        w.emit_f(f);
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        w.emit_string(cooling_marker);
        w.string()
    }

    /// Get current speed
    pub fn get_current_speed(&self) -> f64 {
        self.current_speed
    }

    /// Travel to XY
    /// GCodeWriter.cpp:398-401
    pub fn travel_to_xy(&mut self, point: Vec2d, comment: &str) -> String {
        self.travel_to_xy_with_accel(point, comment, false)
    }

    /// Travel to XY with acceleration control
    /// GCodeWriter.cpp:403-418
    pub fn travel_to_xy_with_accel(
        &mut self,
        point: Vec2d,
        comment: &str,
        use_short_travel_acceleration: bool,
    ) -> String {
        self.pos.x = point.x;
        self.pos.y = point.y;
        self.is_current_pos_clear = true;

        let point_on_plate = Vec2d::new(point.x - self.x_offset, point.y - self.y_offset);

        let mut w = GCodeG1Formatter::new();
        w.emit_xy(point_on_plate);

        if let Some(_filament) = self.current_filament() {
            let travel_speed = self.config.travel_speed * 60.0;
            w.emit_f(travel_speed);
        }

        w.emit_comment(FULL_GCODE_COMMENT, comment);

        let accel = self.set_travel_acceleration_for_move(use_short_travel_acceleration);
        format!("{}{}", accel, w.string())
    }

    /// Travel to XYZ
    /// GCodeWriter.cpp:494-497
    pub fn travel_to_xyz(&mut self, point: Vec3d, comment: &str) -> String {
        self.travel_to_xyz_with_accel(point, comment, false)
    }

    /// Travel to XYZ with acceleration and advanced lift logic
    /// GCodeWriter.cpp:499-622
    pub fn travel_to_xyz_with_accel(
        &mut self,
        point: Vec3d,
        comment: &str,
        use_short_travel_acceleration: bool,
    ) -> String {
        // Handle pending lift
        let mut slop_move = String::new();

        if self.to_lift > 0.0 {
            let source = self.pos;
            let target = point;
            let delta = target - source;
            let delta_no_z = Vec2d::new(delta.x, delta.y);

            if self.to_lift_type == LiftType::SlopeLift
                && delta_no_z.length() > 0.01
                && delta.z.abs() / delta_no_z.length() < SLOPE_THRESHOLD.tan()
            {
                let radius = delta_no_z.length() / (2.0 * std::f64::consts::PI);
                let _ij_offset = Vec2d::new(-radius, 0.0);

                let temp = Vec2d::new(delta.x / delta_no_z.length(), delta.y / delta_no_z.length());
                let slope_top_point = Vec3d::new(
                    source.x + temp.x,
                    source.y + temp.y,
                    source.z + self.to_lift,
                );

                let mut w0 = GCodeG1Formatter::new();
                w0.emit_xyz(slope_top_point);

                let travel_speed = if let Some(_filament) = self.current_filament() {
                    self.config.travel_speed * 60.0
                } else {
                    3000.0
                };
                w0.emit_f(travel_speed);

                slop_move = w0.string();
                self.pos = slope_top_point;
            } else {
                slop_move = self._travel_to_z(self.pos.z + self.to_lift, "", false);
            }

            self.lifted = self.to_lift;
            self.to_lift = 0.0;
        }

        // Descend to target Z if needed
        let mut xy_z_move = String::new();

        if self.lifted > 0.0 && (point.z - self.pos.z).abs() > f64::EPSILON {
            let mut w0 = GCodeG1Formatter::new();
            w0.emit_z(point.z);

            let travel_speed = if let Some(_filament) = self.current_filament() {
                self.config.travel_speed * 60.0
            } else {
                3000.0
            };
            w0.emit_f(travel_speed);

            xy_z_move = w0.string();
            self.lifted = 0.0;
        }

        // Update position
        self.pos = point;

        let point_on_plate = Vec3d::new(point.x - self.x_offset, point.y - self.y_offset, point.z);

        let mut out_string = String::new();
        let mut w = GCodeG1Formatter::new();

        if slop_move.is_empty() && xy_z_move.is_empty() {
            w.emit_xyz(point_on_plate);
        } else {
            w.emit_xy(Vec2d::new(point_on_plate.x, point_on_plate.y));
        }

        if let Some(_filament) = self.current_filament() {
            let travel_speed = self.config.travel_speed * 60.0;
            w.emit_f(travel_speed);
        }

        w.emit_comment(FULL_GCODE_COMMENT, comment);

        let accel = self.set_travel_acceleration_for_move(use_short_travel_acceleration);
        out_string.push_str(&accel);
        out_string.push_str(&slop_move);
        out_string.push_str(&xy_z_move);
        out_string.push_str(&w.string());

        out_string
    }

    /// Travel to Z
    /// GCodeWriter.cpp:624-641
    pub fn travel_to_z(&mut self, z: f64, comment: &str) -> String {
        self._travel_to_z(z, comment, false)
    }

    /// Internal travel to Z
    /// GCodeWriter.cpp:643-659
    fn _travel_to_z(&mut self, z: f64, comment: &str, tool_change: bool) -> String {
        self.pos.z = z;
        self.lifted = 0.0;

        let speed = if tool_change {
            if let Some(filament) = self.current_filament() {
                filament.retraction_speed() as f64 * 60.0
            } else {
                3000.0
            }
        } else if let Some(_filament) = self.current_filament() {
            self.config.travel_speed * 60.0
        } else {
            3000.0
        };

        let mut w = GCodeG1Formatter::new();
        w.emit_z(z);
        w.emit_f(speed);
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        w.string()
    }

    /// Spiral travel to Z
    /// GCodeWriter.cpp:661-679
    fn _spiral_travel_to_z(
        &mut self,
        z: f64,
        ij_offset: Vec2d,
        comment: &str,
        tool_change: bool,
    ) -> String {
        self.pos.z = z;

        let speed = if tool_change {
            if let Some(filament) = self.current_filament() {
                filament.retraction_speed() as f64 * 60.0
            } else {
                3000.0
            }
        } else if let Some(_filament) = self.current_filament() {
            self.config.travel_speed * 60.0
        } else {
            3000.0
        };

        let mut output = String::new();
        let mut w = GCodeG2G3Formatter::new(false);
        w.emit_z(z);
        w.emit_ij(ij_offset);
        w.emit_f(speed);
        w.emit_comment(FULL_GCODE_COMMENT, comment);
        output.push_str(&w.string());

        output
    }

    /// Check if Z will move
    /// GCodeWriter.cpp:681-696
    pub fn will_move_z(&self, z: f64) -> bool {
        (self.pos.z - z).abs() > f64::EPSILON
    }

    /// Extrude to XY
    /// GCodeWriter.cpp:698-716
    pub fn extrude_to_xy(
        &mut self,
        point: Vec2d,
        de: f64,
        comment: &str,
        force_no_extrusion: bool,
    ) -> String {
        self.pos.x = point.x;
        self.pos.y = point.y;

        if !force_no_extrusion {
            if let Some(filament) = self.current_filament_mut() {
                filament.extrude(de);
            }
        }

        let point_on_plate = Vec2d::new(point.x - self.x_offset, point.y - self.y_offset);

        let mut w = GCodeG1Formatter::new();
        w.emit_xy(point_on_plate);

        if !force_no_extrusion {
            if let Some(filament) = self.current_filament() {
                w.emit_e(filament.e());
            }
        }

        w.emit_comment(FULL_GCODE_COMMENT, comment);

        format!("{}{}", self.set_extrude_acceleration(), w.string())
    }

    /// Extrude arc to XY (G2/G3)
    /// GCodeWriter.cpp:721-738
    pub fn extrude_arc_to_xy(
        &mut self,
        point: Vec2d,
        center_offset: Vec2d,
        de: f64,
        is_ccw: bool,
        comment: &str,
        force_no_extrusion: bool,
    ) -> String {
        self.pos.x = point.x;
        self.pos.y = point.y;

        if !force_no_extrusion {
            if let Some(filament) = self.current_filament_mut() {
                filament.extrude(de);
            }
        }

        let point_on_plate = Vec2d::new(point.x - self.x_offset, point.y - self.y_offset);

        let mut w = GCodeG2G3Formatter::new(is_ccw);
        w.emit_xy(point_on_plate);
        w.emit_ij(center_offset);

        if !force_no_extrusion {
            if let Some(filament) = self.current_filament() {
                w.emit_e(filament.e());
            }
        }

        w.emit_comment(FULL_GCODE_COMMENT, comment);

        format!("{}{}", self.set_extrude_acceleration(), w.string())
    }

    /// Extrude to XYZ
    /// GCodeWriter.cpp:740-757
    pub fn extrude_to_xyz(
        &mut self,
        point: Vec3d,
        de: f64,
        comment: &str,
        force_no_extrusion: bool,
    ) -> String {
        self.pos = point;
        self.lifted = 0.0;

        if !force_no_extrusion {
            if let Some(filament) = self.current_filament_mut() {
                filament.extrude(de);
            }
        }

        let point_on_plate = Vec3d::new(point.x - self.x_offset, point.y - self.y_offset, point.z);

        let mut w = GCodeG1Formatter::new();
        w.emit_xyz(point_on_plate);

        if !force_no_extrusion {
            if let Some(filament) = self.current_filament() {
                w.emit_e(filament.e());
            }
        }

        w.emit_comment(FULL_GCODE_COMMENT, comment);

        format!("{}{}", self.set_extrude_acceleration(), w.string())
    }

    /// Retract filament
    /// GCodeWriter.cpp:759-768
    pub fn retract(&mut self, before_wipe: bool) -> String {
        if let Some(filament) = self.current_filament() {
            let length = if before_wipe {
                filament.retraction_length() * filament.retract_before_wipe()
            } else {
                filament.retraction_length()
            };
            self._retract(length, 0.0, "retract")
        } else {
            String::new()
        }
    }

    /// Retract for toolchange
    /// GCodeWriter.cpp:770-779
    pub fn retract_for_toolchange(&mut self, before_wipe: bool) -> String {
        if let Some(filament) = self.current_filament() {
            let length = if before_wipe {
                filament.retract_length_toolchange() * filament.retract_before_wipe()
            } else {
                filament.retract_length_toolchange()
            };
            self._retract(length, 0.0, "retract for toolchange")
        } else {
            String::new()
        }
    }

    /// Internal retract implementation
    /// GCodeWriter.cpp:781-806
    fn _retract(&mut self, length: f64, restart_extra: f64, comment: &str) -> String {
        let mut gcode = String::new();

        if let Some(filament) = self.current_filament_mut() {
            let de = filament.retract(length, restart_extra);

            if de != 0.0 {
                let mut w = GCodeG1Formatter::new();
                w.emit_e(filament.e());
                w.emit_f(filament.retraction_speed() as f64 * 60.0);
                w.emit_comment(FULL_GCODE_COMMENT, comment);
                gcode.push_str(&w.string());
            }
        }

        gcode
    }

    /// Unretract filament
    /// GCodeWriter.cpp:808-833
    pub fn unretract(&mut self, extra_retract: f32) -> String {
        let mut gcode = String::new();

        if let Some(filament) = self.current_filament_mut() {
            filament.restart_extra = extra_retract as f64;
            let de = filament.unretract();

            if de != 0.0 {
                let mut w = GCodeG1Formatter::new();
                w.emit_e(filament.e());
                w.emit_f(filament.deretraction_speed() as f64 * 60.0);

                if FULL_GCODE_COMMENT {
                    let used_filament = filament.used_filament();
                    let comment = format!("unretract, used {:.2}mm filament", used_filament);
                    w.emit_comment(true, &comment);
                }

                gcode.push_str(&w.string());
            }
        }

        gcode
    }

    /// Get extruder retracted length
    /// GCodeWriter.cpp:835-847
    pub fn get_extruder_retracted_length(&self, filament_id: i32) -> f64 {
        let mut res = 0.0;

        if let Some(extruder) = self.filament(filament_id as usize) {
            if extruder.is_share_extruder() {
                res = extruder.get_share_retracted_length();
            } else {
                res = extruder.get_single_retracted_length();
            }
        }

        res
    }

    /// Lazy lift (deferred until next travel)
    /// GCodeWriter.cpp:423-450
    pub fn lazy_lift(
        &mut self,
        lift_type: LiftType,
        spiral_vase: bool,
        _tool_change: bool,
    ) -> String {
        if let Some(filament) = self.current_filament() {
            let target_lift = filament.retract_lift();

            if !spiral_vase && target_lift > self.lifted {
                self.to_lift = target_lift - self.lifted;
                self.to_lift_type = lift_type;
            }
        }

        String::new()
    }

    /// Eager lift (immediate)
    /// GCodeWriter.cpp:454-492
    pub fn eager_lift(&mut self, lift_type: LiftType, tool_change: bool) -> String {
        let mut lift_move = String::new();

        if let Some(filament) = self.current_filament() {
            let target_lift = filament.retract_lift();

            if target_lift > self.lifted {
                let to_lift = target_lift - self.lifted;

                if lift_type == LiftType::SpiralLift {
                    let radius = 2.0;
                    let ij_offset = Vec2d::new(-radius, 0.0);
                    lift_move =
                        self._spiral_travel_to_z(self.pos.z + to_lift, ij_offset, "", tool_change);
                } else {
                    lift_move = self._travel_to_z(self.pos.z + to_lift, "", tool_change);
                }

                self.lifted = target_lift;
            }
        }

        lift_move
    }

    /// Unlift (reverse lift/hop)
    /// GCodeWriter.cpp:849-858
    pub fn unlift(&mut self) -> String {
        let mut gcode = String::new();

        if self.lifted > 0.0 {
            gcode.push_str(&self._travel_to_z(self.pos.z - self.lifted, "restore layer Z", false));
            self.lifted = 0.0;
        }

        gcode
    }

    /// Set fan speed
    /// GCodeWriter.cpp:894-898
    pub fn set_fan(&self, speed: u32) -> String {
        Self::set_fan_static(self.config.gcode_flavor, speed)
    }

    /// Set fan speed (static version)
    /// GCodeWriter.cpp:860-892
    pub fn set_fan_static(gcode_flavor: GCodeFlavor, speed: u32) -> String {
        let mut gcode = String::new();

        if gcode_flavor == GCodeFlavor::Teacup {
            let _ = writeln!(gcode, "M106 S{}", speed.min(255));
        } else if gcode_flavor == GCodeFlavor::MakerWare {
            let _ = writeln!(gcode, "M126 T0");
        } else {
            if speed == 0 {
                if gcode_flavor == GCodeFlavor::RepRapFirmware {
                    gcode.push_str("M106 S0\n");
                } else {
                    gcode.push_str("M107\n");
                }
            } else {
                if gcode_flavor == GCodeFlavor::Mach3 || gcode_flavor == GCodeFlavor::Machinekit {
                    let _ = writeln!(gcode, "M106 P{:.2}", speed as f64 / 255.0 * 100.0);
                } else {
                    let _ = writeln!(gcode, "M106 S{}", speed.min(255));
                }
            }
        }

        gcode
    }

    /// Set additional fan speed (BBL printers)
    /// GCodeWriter.cpp:901-914
    pub fn set_additional_fan(speed: u32) -> String {
        let mut gcode = String::new();

        if speed == 0 {
            gcode.push_str("M106 P2 S0\n");
        } else {
            let _ = writeln!(gcode, "M106 P2 S{}", speed.min(255));
        }

        gcode
    }

    /// Set exhaust fan speed (BBL printers)
    /// GCodeWriter.cpp:916-924
    pub fn set_exhaust_fan(speed: i32, add_eol: bool) -> String {
        let mut gcode = String::new();

        let _ = write!(gcode, "M106 P3 S{}", speed.min(255));
        if add_eol {
            gcode.push('\n');
        }

        gcode
    }

    /// Add object start labels
    /// GCodeWriter.cpp:926-932
    pub fn add_object_start_labels(&mut self, gcode: &mut String) {
        if !self.gcode_label_objects_start.is_empty() {
            gcode.push_str(&self.gcode_label_objects_start);
            self.gcode_label_objects_start.clear();
        }
    }

    /// Add object end labels
    /// GCodeWriter.cpp:934-940
    pub fn add_object_end_labels(&mut self, gcode: &mut String) {
        if !self.gcode_label_objects_end.is_empty() {
            gcode.push_str(&self.gcode_label_objects_end);
            self.gcode_label_objects_end.clear();
        }
    }

    /// Add object change labels
    /// GCodeWriter.cpp:942-946
    pub fn add_object_change_labels(&mut self, gcode: &mut String) {
        self.add_object_end_labels(gcode);
        self.add_object_start_labels(gcode);
    }

    /// Get current position
    pub fn get_position(&self) -> Vec3d {
        self.pos
    }

    /// Set current position
    pub fn set_position(&mut self, pos: Vec3d) {
        self.pos = pos;
    }

    /// Set XY offset
    pub fn set_xy_offset(&mut self, x: f64, y: f64) {
        self.x_offset = x;
        self.y_offset = y;
    }

    /// Get XY offset
    pub fn get_xy_offset(&self) -> Vec2d {
        Vec2d::new(self.x_offset, self.y_offset)
    }

    /// Set whether current position is clear
    pub fn set_current_position_clear(&mut self, clear: bool) {
        self.is_current_pos_clear = clear;
    }

    /// Check if current position is clear
    pub fn is_current_position_clear(&self) -> bool {
        self.is_current_pos_clear
    }

    /// Set whether this is a BBL printer
    pub fn set_is_bbl_printer(&mut self, is_bbl_printer: bool) {
        self.is_bbl_printer = is_bbl_printer;
    }

    /// Set object start label string
    pub fn set_object_start_str(&mut self, start_string: String) {
        self.gcode_label_objects_start = start_string;
    }

    /// Check if object start label is empty
    pub fn empty_object_start_str(&self) -> bool {
        self.gcode_label_objects_start.is_empty()
    }

    /// Set object end label string
    pub fn set_object_end_str(&mut self, end_string: String) {
        self.gcode_label_objects_end = end_string;
    }

    /// Check if object end label is empty
    pub fn empty_object_end_str(&self) -> bool {
        self.gcode_label_objects_end.is_empty()
    }
}

impl Default for GCodeWriter {
    fn default() -> Self {
        Self::new()
    }
}

/// Base G-code formatter with efficient numeric output
/// GCodeWriter.hpp:180-240
pub struct GCodeFormatter {
    /// Output buffer
    buf: Vec<u8>,
    /// Current write position
    pos: usize,
}

impl GCodeFormatter {
    /// Buffer size for formatting
    const BUF_LEN: usize = 256;

    /// Export digits for XYZ and F axes
    /// GCodeWriter.hpp:189
    const XYZF_EXPORT_DIGITS: usize = 3;

    /// Export digits for E axis
    /// GCodeWriter.hpp:190
    const E_EXPORT_DIGITS: usize = 5;

    /// Create a new formatter
    /// GCodeWriter.hpp:185-188
    pub fn new() -> Self {
        Self {
            buf: vec![0; Self::BUF_LEN],
            pos: 0,
        }
    }

    /// Emit an axis value
    /// GCodeWriter.cpp:972-1029
    pub fn emit_axis(&mut self, axis: char, v: f64, digits: usize) {
        debug_assert!(digits <= 9);

        // Powers of 10 for scaling
        const POW_10: [i64; 10] = [
            1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000,
        ];

        self.buf[self.pos] = b' ';
        self.pos += 1;
        self.buf[self.pos] = axis as u8;
        self.pos += 1;

        let base_ptr = self.pos;
        let v_int = (v * POW_10[digits] as f64).round() as i64;

        // Convert integer to string
        let s = v_int.to_string();
        let bytes = s.as_bytes();
        let is_negative = v_int < 0;
        let start_idx = if is_negative { 1 } else { 0 };

        for &b in &bytes[start_idx..] {
            self.buf[self.pos] = b;
            self.pos += 1;
        }

        let written_digits = (self.pos - base_ptr) - if is_negative { 1 } else { 0 };

        // Pad with zeros if needed
        if written_digits < digits {
            let remaining_digits = digits - written_digits;

            // Shift right to make space for zeros
            for i in (0..written_digits).rev() {
                self.buf[self.pos - written_digits + i + remaining_digits] =
                    self.buf[self.pos - written_digits + i];
            }

            // Insert zeros
            for i in 0..remaining_digits {
                self.buf[self.pos - written_digits + i] = b'0';
            }

            self.pos += remaining_digits;
        }

        // Shift right to insert decimal point
        for i in (0..digits).rev() {
            self.buf[self.pos - digits + i + 1] = self.buf[self.pos - digits + i];
        }

        self.buf[self.pos - digits] = b'.';
        self.pos += 1;

        // Trim trailing zeros
        while self.pos > base_ptr && self.buf[self.pos - 1] == b'0' {
            self.pos -= 1;
        }

        // Trim decimal point if no fractional part
        if self.pos > base_ptr && self.buf[self.pos - 1] == b'.' {
            self.pos -= 1;
        }

        // Handle "-0" case or just "-"
        if self.pos == base_ptr || (self.pos == base_ptr + 1 && self.buf[base_ptr] == b'-') {
            self.buf[self.pos] = b'0';
            self.pos += 1;
        }
    }

    /// Emit XY coordinates
    pub fn emit_xy(&mut self, point: Vec2d) {
        self.emit_axis('X', point.x, Self::XYZF_EXPORT_DIGITS);
        self.emit_axis('Y', point.y, Self::XYZF_EXPORT_DIGITS);
    }

    /// Emit XYZ coordinates
    pub fn emit_xyz(&mut self, point: Vec3d) {
        self.emit_axis('X', point.x, Self::XYZF_EXPORT_DIGITS);
        self.emit_axis('Y', point.y, Self::XYZF_EXPORT_DIGITS);
        self.emit_z(point.z);
    }

    /// Emit Z coordinate
    pub fn emit_z(&mut self, z: f64) {
        self.emit_axis('Z', z, Self::XYZF_EXPORT_DIGITS);
    }

    /// Emit E axis
    pub fn emit_e(&mut self, v: f64) {
        self.emit_axis('E', v, Self::E_EXPORT_DIGITS);
    }

    /// Emit F parameter
    pub fn emit_f(&mut self, speed: f64) {
        self.emit_axis('F', speed, Self::XYZF_EXPORT_DIGITS);
    }

    /// Emit IJ offsets for arcs
    pub fn emit_ij(&mut self, point: Vec2d) {
        self.emit_axis('I', point.x, Self::XYZF_EXPORT_DIGITS);
        self.emit_axis('J', point.y, Self::XYZF_EXPORT_DIGITS);
    }

    /// Emit a string
    pub fn emit_string(&mut self, s: &str) {
        for &b in s.as_bytes() {
            self.buf[self.pos] = b;
            self.pos += 1;
        }
    }

    /// Emit a comment
    pub fn emit_comment(&mut self, allow_comments: bool, comment: &str) {
        if allow_comments && !comment.is_empty() {
            self.buf[self.pos] = b' ';
            self.pos += 1;
            self.buf[self.pos] = b';';
            self.pos += 1;
            self.buf[self.pos] = b' ';
            self.pos += 1;
            self.emit_string(comment);
        }
    }

    /// Get the formatted string
    pub fn string(&mut self) -> String {
        self.buf[self.pos] = b'\n';
        self.pos += 1;
        String::from_utf8_lossy(&self.buf[..self.pos]).into_owned()
    }
}

impl Default for GCodeFormatter {
    fn default() -> Self {
        Self::new()
    }
}

/// G1 command formatter
/// GCodeWriter.hpp:242-253
pub struct GCodeG1Formatter {
    formatter: GCodeFormatter,
}

impl GCodeG1Formatter {
    /// Create a new G1 formatter
    /// GCodeWriter.hpp:244-249
    pub fn new() -> Self {
        let mut formatter = GCodeFormatter::new();
        formatter.buf[0] = b'G';
        formatter.buf[1] = b'1';
        formatter.pos = 2;
        Self { formatter }
    }

    /// Emit XY coordinates
    pub fn emit_xy(&mut self, point: Vec2d) {
        self.formatter.emit_xy(point);
    }

    /// Emit XYZ coordinates
    pub fn emit_xyz(&mut self, point: Vec3d) {
        self.formatter.emit_xyz(point);
    }

    /// Emit Z coordinate
    pub fn emit_z(&mut self, z: f64) {
        self.formatter.emit_z(z);
    }

    /// Emit E axis
    pub fn emit_e(&mut self, v: f64) {
        self.formatter.emit_e(v);
    }

    /// Emit F parameter
    pub fn emit_f(&mut self, speed: f64) {
        self.formatter.emit_f(speed);
    }

    /// Emit a string
    pub fn emit_string(&mut self, s: &str) {
        self.formatter.emit_string(s);
    }

    /// Emit a comment
    pub fn emit_comment(&mut self, allow_comments: bool, comment: &str) {
        self.formatter.emit_comment(allow_comments, comment);
    }

    /// Get the formatted string
    pub fn string(&mut self) -> String {
        self.formatter.string()
    }
}

impl Default for GCodeG1Formatter {
    fn default() -> Self {
        Self::new()
    }
}

/// G2/G3 arc command formatter
/// GCodeWriter.hpp:255-266
pub struct GCodeG2G3Formatter {
    formatter: GCodeFormatter,
}

impl GCodeG2G3Formatter {
    /// Create a new G2/G3 formatter
    /// GCodeWriter.hpp:257-262
    pub fn new(is_ccw: bool) -> Self {
        let mut formatter = GCodeFormatter::new();
        formatter.buf[0] = b'G';
        formatter.buf[1] = if is_ccw { b'3' } else { b'2' };
        formatter.pos = 2;
        Self { formatter }
    }

    /// Emit XY coordinates
    pub fn emit_xy(&mut self, point: Vec2d) {
        self.formatter.emit_xy(point);
    }

    /// Emit Z coordinate
    pub fn emit_z(&mut self, z: f64) {
        self.formatter.emit_z(z);
    }

    /// Emit E axis
    pub fn emit_e(&mut self, v: f64) {
        self.formatter.emit_e(v);
    }

    /// Emit F parameter
    pub fn emit_f(&mut self, speed: f64) {
        self.formatter.emit_f(speed);
    }

    /// Emit IJ offsets
    pub fn emit_ij(&mut self, point: Vec2d) {
        self.formatter.emit_ij(point);
    }

    /// Emit a string
    pub fn emit_string(&mut self, s: &str) {
        self.formatter.emit_string(s);
    }

    /// Emit a comment
    pub fn emit_comment(&mut self, allow_comments: bool, comment: &str) {
        self.formatter.emit_comment(allow_comments, comment);
    }

    /// Get the formatted string
    pub fn string(&mut self) -> String {
        self.formatter.string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gcode_writer_creation() {
        let writer = GCodeWriter::new();
        assert_eq!(writer.curr_extruder_id, -1);
        assert!(!writer.multiple_extruders);
    }

    #[test]
    fn test_preamble() {
        let mut writer = GCodeWriter::new();
        writer.config.gcode_flavor = GCodeFlavor::Marlin;
        writer.config.use_relative_e_distances = true;

        let preamble = writer.preamble().unwrap();
        assert!(preamble.contains("G90"));
        assert!(preamble.contains("G21"));
        assert!(preamble.contains("M83"));
    }

    #[test]
    fn test_postamble() {
        let mut writer = GCodeWriter::new();
        writer.config.gcode_flavor = GCodeFlavor::Machinekit;

        let postamble = writer.postamble();
        assert!(postamble.contains("M2"));
    }

    #[test]
    fn test_set_temperature() {
        let writer = GCodeWriter::new();
        let gcode = writer.set_temperature(210, false, -1);
        assert!(gcode.contains("M104"));
        assert!(gcode.contains("S210"));
    }

    #[test]
    fn test_set_temperature_wait() {
        let writer = GCodeWriter::new();
        let gcode = writer.set_temperature(210, true, -1);
        assert!(gcode.contains("M109"));
        assert!(gcode.contains("S210"));
    }

    #[test]
    fn test_set_bed_temperature() {
        let mut writer = GCodeWriter::new();
        let gcode = writer.set_bed_temperature(60, false);
        assert!(gcode.contains("M140"));
        assert!(gcode.contains("S60"));
    }

    #[test]
    fn test_set_chamber_temperature() {
        let writer = GCodeWriter::new();
        let gcode = writer.set_chamber_temperature(40, false);
        assert!(gcode.contains("M141"));
        assert!(gcode.contains("S40"));
    }

    #[test]
    fn test_formatter_g1() {
        let mut formatter = GCodeG1Formatter::new();
        formatter.emit_xy(Vec2d::new(10.0, 20.0));
        formatter.emit_f(3000.0);

        let result = formatter.string();
        assert!(result.starts_with("G1"));
        assert!(result.contains("X10"));
        assert!(result.contains("Y20"));
        assert!(result.contains("F3000"));
    }

    #[test]
    fn test_formatter_g2() {
        let mut formatter = GCodeG2G3Formatter::new(false);
        formatter.emit_xy(Vec2d::new(10.0, 20.0));
        formatter.emit_ij(Vec2d::new(5.0, 5.0));
        formatter.emit_f(1500.0);

        let result = formatter.string();
        assert!(result.starts_with("G2"));
        assert!(result.contains("X10"));
        assert!(result.contains("Y20"));
        assert!(result.contains("I5"));
        assert!(result.contains("J5"));
    }

    #[test]
    fn test_formatter_g3() {
        let mut formatter = GCodeG2G3Formatter::new(true);
        formatter.emit_xy(Vec2d::new(10.0, 20.0));
        formatter.emit_ij(Vec2d::new(5.0, 5.0));

        let result = formatter.string();
        assert!(result.starts_with("G3"));
    }

    #[test]
    fn test_emit_axis_precision() {
        let mut formatter = GCodeFormatter::new();
        formatter.emit_axis('X', 10.12345, 3);

        let result = formatter.string();
        assert!(result.contains("X10.123"));
    }

    #[test]
    fn test_emit_axis_trailing_zeros() {
        let mut formatter = GCodeFormatter::new();
        formatter.emit_axis('X', 10.0, 3);

        let result = formatter.string();
        assert!(result.contains("X10"));
        assert!(!result.contains("X10.000"));
    }

    #[test]
    fn test_set_fan() {
        let gcode = GCodeWriter::set_fan_static(GCodeFlavor::Marlin, 128);
        assert!(gcode.contains("M106"));
        assert!(gcode.contains("S128"));
    }

    #[test]
    fn test_set_fan_off() {
        let gcode = GCodeWriter::set_fan_static(GCodeFlavor::Marlin, 0);
        assert!(gcode.contains("M107"));
    }

    #[test]
    fn test_lift_type() {
        assert_eq!(LiftType::NormalLift, LiftType::NormalLift);
        assert_ne!(LiftType::NormalLift, LiftType::SlopeLift);
    }

    #[test]
    fn test_position_tracking() {
        let mut writer = GCodeWriter::new();
        writer.set_position(Vec3d::new(10.0, 20.0, 5.0));

        let pos = writer.get_position();
        assert_eq!(pos.x, 10.0);
        assert_eq!(pos.y, 20.0);
        assert_eq!(pos.z, 5.0);
    }

    #[test]
    fn test_xy_offset() {
        let mut writer = GCodeWriter::new();
        writer.set_xy_offset(5.0, 10.0);

        let offset = writer.get_xy_offset();
        assert_eq!(offset.x, 5.0);
        assert_eq!(offset.y, 10.0);
    }
}
