//! Wipe Tower Module for Multi-Material Printing
//!
//! This module implements wipe tower generation for multi-material 3D printing,
//! porting the functionality from BambuStudio's `GCode/WipeTower.cpp`.
//!
//! The wipe tower is a sacrificial structure printed alongside the main model
//! that serves several purposes:
//! - Purging old filament during tool changes
//! - Priming the new filament before continuing the print
//! - Stabilizing extrusion after filament switches
//!
//! ## Key Concepts
//!
//! - **Tool Change**: Switching from one filament to another
//! - **Ramming**: Fast extrusion to push out old filament
//! - **Wiping**: Back-and-forth movements to clean the nozzle
//! - **Purge Volume**: Amount of filament needed to fully transition colors
//!
//! ## Reference
//!
//! - `BambuStudio/src/libslic3r/GCode/WipeTower.hpp`
//! - `BambuStudio/src/libslic3r/GCode/WipeTower.cpp`

use std::collections::HashMap;
use std::f32::consts::PI;

use crate::geometry::{BoundingBoxF, Line, Point, PointF, Polygon, Polyline};

// ============================================================================
// Constants
// ============================================================================

/// Resolution for wipe tower paths (mm)
const WIPE_TOWER_RESOLUTION: f32 = 0.1;

/// Default overlap for wipe tower wall infill
const WIPE_TOWER_WALL_INFILL_OVERLAP: f32 = 0.0;

/// Small epsilon for floating point comparisons
const WT_EPSILON: f32 = 1e-4;

/// Width to nozzle diameter ratio
const WIDTH_TO_NOZZLE_RATIO: f32 = 1.25;

/// Default flat iron area
const FLAT_IRON_AREA: f32 = 4.0;

/// Default flat iron speed (mm/min)
const FLAT_IRON_SPEED: f32 = 10.0 * 60.0;

/// Minimum depth per height mapping for tower stability
const MIN_DEPTH_PER_HEIGHT: &[(f32, f32)] = &[
    (50.0, 10.0),
    (100.0, 15.0),
    (150.0, 20.0),
    (200.0, 25.0),
    (250.0, 30.0),
    (300.0, 35.0),
];

// ============================================================================
// Types and Enums
// ============================================================================

/// G-code flavor for different printer types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GCodeFlavor {
    #[default]
    Marlin,
    RepRap,
    Klipper,
    Smoothie,
    Mach3,
}

/// Flow limiting mode during ramming
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitFlow {
    /// No flow limiting
    None,
    /// Limit based on print flow
    LimitPrintFlow,
    /// Limit based on ramming flow
    LimitRammingFlow,
    /// Limit based on nozzle change ramming flow
    LimitRammingFlowNC,
}

/// Wipe tower shape direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WipeShape {
    #[default]
    Normal,
    Reversed,
}

/// Bed shape type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BedShape {
    #[default]
    Rectangular,
    Circular,
    Custom,
}

// ============================================================================
// Core Data Structures
// ============================================================================

/// 2D vector for wipe tower coordinates
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vec2f {
    pub x: f32,
    pub y: f32,
}

impl Vec2f {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    pub fn norm(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn normalized(&self) -> Self {
        let n = self.norm();
        if n > 0.0 {
            Self::new(self.x / n, self.y / n)
        } else {
            *self
        }
    }

    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    pub fn rotate(&self, angle: f32) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Self::new(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a,
        )
    }
}

impl std::ops::Add for Vec2f {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y)
    }
}

impl std::ops::AddAssign for Vec2f {
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl std::ops::Sub for Vec2f {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y)
    }
}

impl std::ops::Mul<f32> for Vec2f {
    type Output = Self;
    fn mul(self, scalar: f32) -> Self {
        Self::new(self.x * scalar, self.y * scalar)
    }
}

impl std::ops::Neg for Vec2f {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl From<PointF> for Vec2f {
    fn from(p: PointF) -> Self {
        Self::new(p.x as f32, p.y as f32)
    }
}

impl From<Vec2f> for PointF {
    fn from(v: Vec2f) -> Self {
        PointF::new(v.x as f64, v.y as f64)
    }
}

/// Box coordinates for wipe tower regions
#[derive(Debug, Clone, Copy)]
pub struct BoxCoordinates {
    /// Left-down corner
    pub ld: Vec2f,
    /// Left-up corner
    pub lu: Vec2f,
    /// Right-down corner
    pub rd: Vec2f,
    /// Right-up corner
    pub ru: Vec2f,
}

impl BoxCoordinates {
    pub fn new(left: f32, bottom: f32, width: f32, height: f32) -> Self {
        Self {
            ld: Vec2f::new(left, bottom),
            lu: Vec2f::new(left, bottom + height),
            rd: Vec2f::new(left + width, bottom),
            ru: Vec2f::new(left + width, bottom + height),
        }
    }

    pub fn from_pos(pos: Vec2f, width: f32, height: f32) -> Self {
        Self::new(pos.x, pos.y, width, height)
    }

    pub fn translate(&mut self, shift: Vec2f) {
        self.ld += shift;
        self.lu += shift;
        self.rd += shift;
        self.ru += shift;
    }

    pub fn expand(&mut self, offset: f32) {
        self.ld += Vec2f::new(-offset, -offset);
        self.lu += Vec2f::new(-offset, offset);
        self.rd += Vec2f::new(offset, -offset);
        self.ru += Vec2f::new(offset, offset);
    }

    pub fn expand_xy(&mut self, offset_x: f32, offset_y: f32) {
        self.ld += Vec2f::new(-offset_x, -offset_y);
        self.lu += Vec2f::new(-offset_x, offset_y);
        self.rd += Vec2f::new(offset_x, -offset_y);
        self.ru += Vec2f::new(offset_x, offset_y);
    }

    pub fn width(&self) -> f32 {
        self.rd.x - self.ld.x
    }

    pub fn height(&self) -> f32 {
        self.lu.y - self.ld.y
    }
}

/// Extrusion record for path preview
#[derive(Debug, Clone)]
pub struct Extrusion {
    /// End position of this extrusion
    pub pos: Vec2f,
    /// Width of the extrusion (0 for travel moves)
    pub width: f32,
    /// Current extruder index
    pub tool: usize,
}

impl Extrusion {
    pub fn new(pos: Vec2f, width: f32, tool: usize) -> Self {
        Self { pos, width, tool }
    }
}

/// Result of a nozzle change operation
#[derive(Debug, Clone, Default)]
pub struct NozzleChangeResult {
    /// G-code for the nozzle change
    pub gcode: String,
    /// Start position (rotated)
    pub start_pos: Vec2f,
    /// End position (rotated)
    pub end_pos: Vec2f,
    /// Original start position (not rotated)
    pub origin_start_pos: Vec2f,
    /// Path for wiping
    pub wipe_path: Vec<Vec2f>,
    /// Whether this is an extruder change
    pub is_extruder_change: bool,
}

/// Result of a tool change operation
#[derive(Debug, Clone, Default)]
pub struct ToolChangeResult {
    /// Print height of this tool change
    pub print_z: f32,
    /// Layer height
    pub layer_height: f32,
    /// G-code section
    pub gcode: String,
    /// Extrusion records for path preview
    pub extrusions: Vec<Extrusion>,
    /// Initial position
    pub start_pos: Vec2f,
    /// Final position
    pub end_pos: Vec2f,
    /// Time elapsed during this tool change
    pub elapsed_time: f32,
    /// Is this a priming extrusion?
    pub priming: bool,
    /// Is this an actual tool change?
    pub is_tool_change: bool,
    /// Position where tool change started
    pub tool_change_start_pos: Vec2f,
    /// Wipe path for G-code generator
    pub wipe_path: Vec<Vec2f>,
    /// Purge volume used
    pub purge_volume: f32,
    /// Initial tool index
    pub initial_tool: i32,
    /// New tool index
    pub new_tool: i32,
    /// Whether finish layer comes before tool change
    pub is_finish_first: bool,
    /// Result of nozzle change if applicable
    pub nozzle_change_result: NozzleChangeResult,
}

impl ToolChangeResult {
    // Calculate total extrusion length in the XY plane
    pub fn total_extrusion_length_in_plane(&self) -> f32 {
        let mut e_length = 0.0f32;
        for i in 1..self.extrusions.len() {
            let e = &self.extrusions[i];
            if e.width > 0.0 {
                let v = e.pos - self.extrusions[i - 1].pos;
                e_length += v.norm();
            }
        }
        e_length
    }
}

/// Parameters for a single filament
#[derive(Debug, Clone)]
pub struct FilamentParameters {
    /// Material type (e.g., "PLA", "ABS")
    pub material: String,
    /// Adhesiveness category
    pub category: i32,
    /// Is this filament soluble?
    pub is_soluble: bool,
    /// Is this a support filament?
    pub is_support: bool,
    /// Nozzle temperature
    pub nozzle_temperature: i32,
    /// Initial layer nozzle temperature
    pub nozzle_temperature_initial_layer: i32,
    /// Ramming line width multiplier
    pub ramming_line_width_multiplicator: f32,
    /// Ramming step multiplier
    pub ramming_step_multiplicator: f32,
    /// Maximum extrusion speed (mm/s)
    pub max_e_speed: f32,
    /// Ramming speeds (mm/s)
    pub ramming_speed: Vec<f32>,
    /// Nozzle diameter
    pub nozzle_diameter: f32,
    /// Filament cross-sectional area
    pub filament_area: f32,
    /// Retraction length
    pub retract_length: f32,
    /// Retraction speed
    pub retract_speed: f32,
    /// Wipe distance
    pub wipe_dist: f32,
    /// Maximum ramming speed for (extruder change, nozzle change)
    pub max_e_ramming_speed: (f32, f32),
    /// Ramming travel time (extruder change, nozzle change)
    pub ramming_travel_time: (f32, f32),
    /// Pre-cooling time tables
    pub precool_t: (Vec<f32>, Vec<f32>),
    /// Pre-cooling time tables for first layer
    pub precool_t_first_layer: (Vec<f32>, Vec<f32>),
    /// Pre-cooling target temperatures
    pub precool_target_temp: (i32, i32),
    /// Filament cooling time before tower
    pub filament_cooling_before_tower: f32,
}

impl Default for FilamentParameters {
    fn default() -> Self {
        Self {
            material: "PLA".to_string(),
            category: 0,
            is_soluble: false,
            is_support: false,
            nozzle_temperature: 200,
            nozzle_temperature_initial_layer: 210,
            ramming_line_width_multiplicator: 1.0,
            ramming_step_multiplicator: 1.0,
            max_e_speed: f32::MAX,
            ramming_speed: vec![],
            nozzle_diameter: 0.4,
            filament_area: PI * 0.4375 * 0.4375, // 1.75mm filament
            retract_length: 0.8,
            retract_speed: 35.0,
            wipe_dist: 1.0,
            max_e_ramming_speed: (0.0, 0.0),
            ramming_travel_time: (0.0, 0.0),
            precool_t: (vec![], vec![]),
            precool_t_first_layer: (vec![], vec![]),
            precool_target_temp: (0, 0),
            filament_cooling_before_tower: 0.0,
        }
    }
}

/// Information about a single tool change
#[derive(Debug, Clone)]
pub struct ToolChangeInfo {
    /// Old tool index
    pub old_tool: usize,
    /// New tool index
    pub new_tool: usize,
    /// Required depth for this tool change
    pub required_depth: f32,
    /// Depth used for ramming
    pub ramming_depth: f32,
    /// Position of first wipe line
    pub first_wipe_line: f32,
    /// Volume to wipe
    pub wipe_volume: f32,
    /// Length to wipe
    pub wipe_length: f32,
    /// Depth for nozzle change
    pub nozzle_change_depth: f32,
    /// Length for nozzle change
    pub nozzle_change_length: f32,
    /// Purge volume
    pub purge_volume: f32,
}

impl ToolChangeInfo {
    pub fn new(old_tool: usize, new_tool: usize) -> Self {
        Self {
            old_tool,
            new_tool,
            required_depth: 0.0,
            ramming_depth: 0.0,
            first_wipe_line: 0.0,
            wipe_volume: 0.0,
            wipe_length: 0.0,
            nozzle_change_depth: 0.0,
            nozzle_change_length: 0.0,
            purge_volume: 0.0,
        }
    }
}

/// Information about a single layer in the wipe tower
#[derive(Debug, Clone)]
pub struct WipeTowerLayerInfo {
    /// Z height
    pub z: f32,
    /// Layer height
    pub height: f32,
    /// Depth of this layer
    pub depth: f32,
    /// Extra spacing factor
    pub extra_spacing: f32,
    /// Whether this layer has extruder fill
    pub extruder_fill: bool,
    /// Tool changes in this layer
    pub tool_changes: Vec<ToolChangeInfo>,
}

impl WipeTowerLayerInfo {
    pub fn new(z: f32, height: f32) -> Self {
        Self {
            z,
            height,
            depth: 0.0,
            extra_spacing: 1.0,
            extruder_fill: false,
            tool_changes: vec![],
        }
    }

    /// Calculate total depth for all tool changes
    pub fn toolchanges_depth(&self) -> f32 {
        self.tool_changes.iter().map(|tc| tc.required_depth).sum()
    }
}

/// Block of wipe tower for multi-extruder support
#[derive(Debug, Clone, Default)]
pub struct WipeTowerBlock {
    /// Block ID
    pub block_id: i32,
    /// Filament adhesiveness category
    pub filament_adhesiveness_category: i32,
    /// Depth per layer
    pub layer_depths: Vec<f32>,
    /// Solid infill flags per layer
    pub solid_infill: Vec<bool>,
    /// Finish depth per layer
    pub finish_depth: Vec<f32>,
    /// Total depth
    pub depth: f32,
    /// Starting depth
    pub start_depth: f32,
    /// Current depth
    pub cur_depth: f32,
    /// Last filament change ID
    pub last_filament_change_id: i32,
    /// Last nozzle change ID
    pub last_nozzle_change_id: i32,
}

/// Depth information for a block
#[derive(Debug, Clone, Default)]
pub struct BlockDepthInfo {
    /// Category
    pub category: i32,
    /// Depth
    pub depth: f32,
    /// Nozzle change depth
    pub nozzle_change_depth: f32,
}

// ============================================================================
// Wipe Tower Configuration
// ============================================================================

/// Configuration for the wipe tower
#[derive(Debug, Clone)]
pub struct WipeTowerConfig {
    /// X position of wipe tower
    pub pos_x: f32,
    /// Y position of wipe tower
    pub pos_y: f32,
    /// Width of wipe tower
    pub width: f32,
    /// Depth of wipe tower (calculated)
    pub depth: f32,
    /// Maximum height of wipe tower
    pub height: f32,
    /// Brim width
    pub brim_width: f32,
    /// Rotation angle (degrees)
    pub rotation_angle: f32,
    /// Whether this is a single extruder multi-material setup
    pub semm: bool,
    /// G-code flavor
    pub gcode_flavor: GCodeFlavor,
    /// Travel speed (mm/s)
    pub travel_speed: f32,
    /// First layer speed (mm/s)
    pub first_layer_speed: f32,
    /// Maximum print speed (mm/s)
    pub max_speed: f32,
    /// Bridging parameter
    pub bridging: f32,
    /// Whether to skip sparse layers
    pub no_sparse_layers: bool,
    /// Enable timelapse printing
    pub enable_timelapse_print: bool,
    /// Enable wrapping detection
    pub enable_wrapping_detection: bool,
    /// Number of wrapping detection layers
    pub wrapping_detection_layers: i32,
    /// Whether this is a multi-extruder setup
    pub is_multi_extruder: bool,
    /// Use gap wall
    pub use_gap_wall: bool,
    /// Use rib wall
    pub use_rib_wall: bool,
    /// Extra rib length
    pub extra_rib_length: f32,
    /// Rib width
    pub rib_width: f32,
    /// Use fillet corners
    pub use_fillet: bool,
    /// Extra spacing factor
    pub extra_spacing: f32,
    /// Enable tower framework
    pub tower_framework: bool,
    /// Flat ironing enabled
    pub flat_ironing: bool,
    /// Bed shape type
    pub bed_shape: BedShape,
    /// Bed width
    pub bed_width: f32,
    /// Bed bottom-left corner
    pub bed_bottom_left: Vec2f,
    /// Normal accelerations per extruder
    pub normal_accels: Vec<u32>,
    /// First layer normal accelerations
    pub first_layer_normal_accels: Vec<u32>,
    /// Travel accelerations per extruder
    pub travel_accels: Vec<u32>,
    /// First layer travel accelerations
    pub first_layer_travel_accels: Vec<u32>,
    /// Maximum acceleration
    pub max_accel: u32,
    /// Enable accel-to-decel
    pub accel_to_decel_enable: bool,
    /// Accel-to-decel factor
    pub accel_to_decel_factor: f32,
    /// Printable height per extruder
    pub printable_height: Vec<f32>,
    /// Physical extruder mapping
    pub physical_extruder_map: Vec<i32>,
    /// Filament change length per filament
    pub filament_change_length: Vec<f32>,
    /// Filament change length for nozzle change
    pub filament_change_length_nc: Vec<f32>,
    /// Hotend heating rates
    pub hotend_heating_rate: Vec<f32>,
    /// First layer flow ratio
    pub first_layer_flow_ratio: f32,
}

impl Default for WipeTowerConfig {
    fn default() -> Self {
        Self {
            pos_x: 170.0,
            pos_y: 125.0,
            width: 60.0,
            depth: 0.0,
            height: 0.0,
            brim_width: 2.0,
            rotation_angle: 0.0,
            semm: false,
            gcode_flavor: GCodeFlavor::Marlin,
            travel_speed: 150.0,
            first_layer_speed: 30.0,
            max_speed: 100.0,
            bridging: 10.0,
            no_sparse_layers: false,
            enable_timelapse_print: false,
            enable_wrapping_detection: false,
            wrapping_detection_layers: 0,
            is_multi_extruder: false,
            use_gap_wall: false,
            use_rib_wall: false,
            extra_rib_length: 0.0,
            rib_width: 0.0,
            use_fillet: false,
            extra_spacing: 1.0,
            tower_framework: false,
            flat_ironing: false,
            bed_shape: BedShape::Rectangular,
            bed_width: 256.0,
            bed_bottom_left: Vec2f::zero(),
            normal_accels: vec![500],
            first_layer_normal_accels: vec![500],
            travel_accels: vec![1000],
            first_layer_travel_accels: vec![1000],
            max_accel: 5000,
            accel_to_decel_enable: false,
            accel_to_decel_factor: 0.5,
            printable_height: vec![300.0],
            physical_extruder_map: vec![0],
            filament_change_length: vec![20.0],
            filament_change_length_nc: vec![20.0],
            hotend_heating_rate: vec![2.0],
            first_layer_flow_ratio: 1.0,
        }
    }
}

// ============================================================================
// Wipe Tower Writer
// ============================================================================

/// G-code writer specifically for wipe tower operations
#[derive(Debug, Clone)]
pub struct WipeTowerWriter {
    /// Start position
    start_pos: Vec2f,
    /// Current position
    current_pos: Vec2f,
    /// Wipe path points
    wipe_path: Vec<Vec2f>,
    /// Current Z height
    current_z: f32,
    /// Current feedrate
    current_feedrate: f32,
    /// Current tool index
    current_tool: usize,
    /// Layer height
    layer_height: f32,
    /// Extrusion flow rate
    extrusion_flow: f32,
    /// Preview suppression flag
    preview_suppressed: bool,
    /// Generated G-code
    gcode: String,
    /// Extrusion records
    extrusions: Vec<Extrusion>,
    /// Elapsed time
    elapsed_time: f32,
    /// Internal rotation angle
    internal_angle: f32,
    /// Y shift
    y_shift: f32,
    /// Wipe tower width
    wipe_tower_width: f32,
    /// Wipe tower depth
    wipe_tower_depth: f32,
    /// Last fan speed
    last_fan_speed: u32,
    /// Current temperature
    current_temp: i32,
    /// Default analyzer line width
    default_analyzer_line_width: f32,
    /// Used filament length
    used_filament_length: f32,
    /// G-code flavor
    gcode_flavor: GCodeFlavor,
    /// Is first layer
    is_first_layer: bool,
    /// Normal accelerations
    normal_accelerations: Vec<u32>,
    /// First layer normal accelerations
    first_layer_normal_accelerations: Vec<u32>,
    /// Travel accelerations
    travel_accelerations: Vec<u32>,
    /// First layer travel accelerations
    first_layer_travel_accelerations: Vec<u32>,
    /// Maximum acceleration
    max_acceleration: u32,
    /// Last acceleration value
    last_acceleration: u32,
    /// Filament map
    filament_map: Vec<i32>,
    /// Accel-to-decel enable
    accel_to_decel_enable: bool,
    /// Accel-to-decel factor
    accel_to_decel_factor: f32,
}

impl WipeTowerWriter {
    // Create a new wipe tower writer
    pub fn new(
        layer_height: f32,
        perimeter_width: f32,
        gcode_flavor: GCodeFlavor,
        filament_parameters: &[FilamentParameters],
    ) -> Self {
        let extrusion_flow = Self::calculate_extrusion_flow(layer_height, perimeter_width);

        Self {
            start_pos: Vec2f::zero(),
            current_pos: Vec2f::zero(),
            wipe_path: vec![],
            current_z: 0.0,
            current_feedrate: 0.0,
            current_tool: 0,
            layer_height,
            extrusion_flow,
            preview_suppressed: false,
            gcode: String::new(),
            extrusions: vec![],
            elapsed_time: 0.0,
            internal_angle: 0.0,
            y_shift: 0.0,
            wipe_tower_width: 0.0,
            wipe_tower_depth: 0.0,
            last_fan_speed: 0,
            current_temp: 0,
            default_analyzer_line_width: perimeter_width,
            used_filament_length: 0.0,
            gcode_flavor,
            is_first_layer: false,
            normal_accelerations: vec![],
            first_layer_normal_accelerations: vec![],
            travel_accelerations: vec![],
            first_layer_travel_accelerations: vec![],
            max_acceleration: 0,
            last_acceleration: 0,
            filament_map: vec![],
            accel_to_decel_enable: false,
            accel_to_decel_factor: 0.5,
        }
    }

    /// Calculate extrusion flow based on layer height and width
    fn calculate_extrusion_flow(layer_height: f32, perimeter_width: f32) -> f32 {
        // Cross-section area using rounded rectangle formula
        let area = layer_height * (perimeter_width - layer_height * (1.0 - PI / 4.0));
        // Filament area (1.75mm diameter)
        let filament_area = PI * 0.875 * 0.875;
        area / filament_area
    }

    /// Set initial position
    pub fn set_initial_position(&mut self, pos: Vec2f, internal_angle: f32, y_shift: f32) {
        self.start_pos = pos;
        self.current_pos = pos;
        self.internal_angle = internal_angle;
        self.y_shift = y_shift;
    }

    /// Set current tool
    pub fn set_initial_tool(&mut self, tool: usize) {
        self.current_tool = tool;
    }

    /// Set Z height
    pub fn set_z(&mut self, z: f32) {
        self.current_z = z;
    }

    /// Set extrusion flow
    pub fn set_extrusion_flow(&mut self, flow: f32) {
        self.extrusion_flow = flow;
    }

    /// Set Y shift
    pub fn set_y_shift(&mut self, y_shift: f32) {
        let delta = y_shift - self.y_shift;
        self.current_pos.y += delta;
        self.y_shift = y_shift;
    }

    /// Set wipe tower dimensions
    pub fn set_wipe_tower_dimensions(&mut self, width: f32, depth: f32) {
        self.wipe_tower_width = width;
        self.wipe_tower_depth = depth;
    }

    /// Set first layer flag
    pub fn set_first_layer(&mut self, is_first: bool) {
        self.is_first_layer = is_first;
    }

    /// Disable linear advance
    pub fn disable_linear_advance(&mut self) {
        match self.gcode_flavor {
            GCodeFlavor::Marlin => {
                self.gcode.push_str("M900 K0\n");
            }
            GCodeFlavor::Klipper => {
                self.gcode.push_str("SET_PRESSURE_ADVANCE ADVANCE=0\n");
            }
            _ => {}
        }
    }

    /// Suppress preview output
    pub fn suppress_preview(&mut self) {
        self.preview_suppressed = true;
    }

    /// Resume preview output
    pub fn resume_preview(&mut self) {
        self.preview_suppressed = false;
    }

    /// Set feedrate
    pub fn feedrate(&mut self, f: f32) -> &mut Self {
        if (self.current_feedrate - f).abs() > WT_EPSILON {
            self.gcode.push_str(&format!("G1 F{:.0}\n", f));
            self.current_feedrate = f;
        }
        self
    }

    /// Get generated G-code
    pub fn gcode(&self) -> &str {
        &self.gcode
    }

    /// Get extrusions
    pub fn extrusions(&self) -> &[Extrusion] {
        &self.extrusions
    }

    /// Get current X position
    pub fn x(&self) -> f32 {
        self.current_pos.x
    }

    /// Get current Y position
    pub fn y(&self) -> f32 {
        self.current_pos.y
    }

    /// Get current position
    pub fn pos(&self) -> Vec2f {
        self.current_pos
    }

    /// Get start position (rotated)
    pub fn start_pos_rotated(&self) -> Vec2f {
        self.rotate(self.start_pos)
    }

    /// Get current position (rotated)
    pub fn pos_rotated(&self) -> Vec2f {
        self.rotate(self.current_pos)
    }

    /// Get elapsed time
    pub fn elapsed_time(&self) -> f32 {
        self.elapsed_time
    }

    /// Get and reset used filament length
    pub fn get_and_reset_used_filament_length(&mut self) -> f32 {
        let temp = self.used_filament_length;
        self.used_filament_length = 0.0;
        temp
    }

    /// Get wipe path
    pub fn wipe_path(&self) -> &[Vec2f] {
        &self.wipe_path
    }

    /// Travel to position (no extrusion)
    pub fn travel(&mut self, x: f32, y: f32) -> &mut Self {
        self.travel_to(Vec2f::new(x, y))
    }

    /// Travel to position
    pub fn travel_to(&mut self, target: Vec2f) -> &mut Self {
        let rotated = self.rotate(target);
        self.gcode.push_str(&format!(
            "G0 X{:.3} Y{:.3}\n",
            rotated.x,
            rotated.y + self.y_shift
        ));

        if !self.preview_suppressed {
            self.extrusions
                .push(Extrusion::new(target, 0.0, self.current_tool));
        }

        let dx = target.x - self.current_pos.x;
        let dy = target.y - self.current_pos.y;
        let len = (dx * dx + dy * dy).sqrt();
        if self.current_feedrate > 0.0 {
            self.elapsed_time += len / self.current_feedrate * 60.0;
        }

        self.current_pos = target;
        self
    }

    /// Extrude to position
    pub fn extrude(&mut self, x: f32, y: f32) -> &mut Self {
        let dx = x - self.current_pos.x;
        let dy = y - self.current_pos.y;
        self.extrude_explicit(
            x,
            y,
            (dx * dx + dy * dy).sqrt() * self.extrusion_flow,
            self.default_analyzer_line_width,
            false,
        )
    }

    /// Extrude to position with explicit parameters
    pub fn extrude_explicit(
        &mut self,
        x: f32,
        y: f32,
        e: f32,
        width: f32,
        limit_flow: bool,
    ) -> &mut Self {
        let target = Vec2f::new(x, y);
        let rotated = self.rotate(target);

        let dx = x - self.current_pos.x;
        let dy = y - self.current_pos.y;
        let len = (dx * dx + dy * dy).sqrt();

        self.gcode.push_str(&format!(
            "G1 X{:.3} Y{:.3} E{:.5}\n",
            rotated.x,
            rotated.y + self.y_shift,
            e
        ));

        if !self.preview_suppressed && width > 0.0 {
            self.extrusions
                .push(Extrusion::new(target, width, self.current_tool));
        }

        if self.current_feedrate > 0.0 {
            self.elapsed_time += len / self.current_feedrate * 60.0;
        }

        self.used_filament_length += e;
        self.current_pos = target;
        self
    }

    /// Extrude a rectangle
    pub fn rectangle(&mut self, box_coords: &BoxCoordinates) -> &mut Self {
        // Find closest corner
        let corners = [box_coords.ld, box_coords.lu, box_coords.ru, box_coords.rd];
        let mut closest_idx = 0;
        let mut min_dist = f32::MAX;

        for (i, corner) in corners.iter().enumerate() {
            let d = (self.current_pos - *corner).norm();
            if d < min_dist {
                min_dist = d;
                closest_idx = i;
            }
        }

        // Extrude around the rectangle starting from closest corner
        for i in 0..=4 {
            let idx = (closest_idx + i) % 4;
            if i == 0 {
                self.travel_to(corners[idx]);
            } else {
                self.extrude(corners[idx].x, corners[idx].y);
            }
        }

        self
    }

    /// Fill a box with back-and-forth extrusion
    pub fn rectangle_fill_box(&mut self, box_coords: &BoxCoordinates, spacing: f32) -> &mut Self {
        let width = box_coords.width();
        let height = box_coords.height();
        let num_lines = (height / spacing).floor() as i32;

        if num_lines < 1 {
            return self;
        }

        let actual_spacing = height / num_lines as f32;
        let mut y = box_coords.ld.y + actual_spacing / 2.0;
        let mut left_to_right = true;

        for _ in 0..num_lines {
            let (start_x, end_x) = if left_to_right {
                (box_coords.ld.x, box_coords.rd.x)
            } else {
                (box_coords.rd.x, box_coords.ld.x)
            };

            self.travel(start_x, y);
            self.extrude(end_x, y);

            y += actual_spacing;
            left_to_right = !left_to_right;
        }

        self
    }

    /// Add a line
    pub fn line(&mut self, from: Vec2f, to: Vec2f) -> &mut Self {
        self.travel_to(from);
        self.extrude(to.x, to.y)
    }

    /// Load filament
    pub fn load(&mut self, e: f32, f: f32) -> &mut Self {
        if e > 0.0 {
            self.gcode.push_str(&format!("G1 E{:.5} F{:.0}\n", e, f));
            self.used_filament_length += e;
            self.elapsed_time += e.abs() / f * 60.0;
        }
        self
    }

    /// Retract filament
    pub fn retract(&mut self, e: f32, f: f32) -> &mut Self {
        self.load(-e, f)
    }

    /// Z hop
    pub fn z_hop(&mut self, hop: f32, f: f32) -> &mut Self {
        let z_str = super::writer::format_gcode_value((self.current_z + hop) as f64, 3);
        self.gcode.push_str(&format!("G1 Z{} F{:.0}\n", z_str, f));
        self.elapsed_time += hop.abs() / f * 60.0;
        self
    }

    /// Reset Z hop
    pub fn z_hop_reset(&mut self, f: f32) -> &mut Self {
        let z_str = super::writer::format_gcode_value(self.current_z as f64, 3);
        self.gcode.push_str(&format!("G1 Z{} F{:.0}\n", z_str, f));
        self
    }

    /// Set tool
    pub fn set_tool(&mut self, tool: usize) -> &mut Self {
        self.gcode.push_str(&format!("T{}\n", tool));
        self.current_tool = tool;
        self
    }

    /// Set extruder temperature
    pub fn set_extruder_temp(&mut self, temp: i32, wait: bool) -> &mut Self {
        let cmd = if wait { "M109" } else { "M104" };
        self.gcode.push_str(&format!("{} S{}\n", cmd, temp));
        self.current_temp = temp;
        self
    }

    /// Wait for time
    pub fn wait(&mut self, seconds: f32) -> &mut Self {
        if seconds > 0.0 {
            self.gcode
                .push_str(&format!("G4 P{}\n", (seconds * 1000.0) as i32));
            self.elapsed_time += seconds;
        }
        self
    }

    /// Speed override
    pub fn speed_override(&mut self, percent: i32) -> &mut Self {
        self.gcode.push_str(&format!("M220 S{}\n", percent));
        self
    }

    /// Set fan speed
    pub fn set_fan(&mut self, speed: u32) -> &mut Self {
        if speed != self.last_fan_speed {
            if speed == 0 {
                self.gcode.push_str("M107\n");
            } else {
                self.gcode.push_str(&format!("M106 S{}\n", speed));
            }
            self.last_fan_speed = speed;
        }
        self
    }

    /// Reset extruder position
    pub fn reset_extruder(&mut self) -> &mut Self {
        self.gcode.push_str("G92 E0\n");
        self
    }

    /// Append raw G-code
    pub fn append(&mut self, gcode: &str) -> &mut Self {
        self.gcode.push_str(gcode);
        self
    }

    /// Add comment
    pub fn comment(&mut self, text: &str) -> &mut Self {
        self.gcode.push_str(&format!("; {}\n", text));
        self
    }

    /// Add wipe point
    pub fn add_wipe_point(&mut self, pos: Vec2f) -> &mut Self {
        self.wipe_path.push(self.rotate(pos));
        self
    }

    /// Set normal acceleration
    pub fn set_normal_acceleration(&mut self) -> &mut Self {
        let accels = if self.is_first_layer {
            &self.first_layer_normal_accelerations
        } else {
            &self.normal_accelerations
        };

        if !accels.is_empty() {
            let idx = self.current_tool.min(accels.len() - 1);
            let acc = accels[idx].min(self.max_acceleration);
            if acc != self.last_acceleration {
                self.gcode.push_str(&format!("M204 S{}\n", acc));
                if self.accel_to_decel_enable {
                    let decel = (acc as f32 * self.accel_to_decel_factor) as u32;
                    self.gcode.push_str(&format!("M204 T{}\n", decel));
                }
                self.last_acceleration = acc;
            }
        }
        self
    }

    /// Set travel acceleration
    pub fn set_travel_acceleration(&mut self) -> &mut Self {
        let accels = if self.is_first_layer {
            &self.first_layer_travel_accelerations
        } else {
            &self.travel_accelerations
        };

        if !accels.is_empty() {
            let idx = self.current_tool.min(accels.len() - 1);
            let acc = accels[idx].min(self.max_acceleration);
            if acc != self.last_acceleration {
                self.gcode.push_str(&format!("M204 S{}\n", acc));
                self.last_acceleration = acc;
            }
        }
        self
    }

    /// Rotate a point by the internal angle
    fn rotate(&self, pt: Vec2f) -> Vec2f {
        if self.internal_angle.abs() < WT_EPSILON {
            pt
        } else {
            let angle = self.internal_angle * PI / 180.0;
            pt.rotate(angle)
        }
    }
}

// ============================================================================
// Wipe Tower
// ============================================================================

/// Main wipe tower generator
#[derive(Debug, Clone)]
pub struct WipeTower {
    /// Configuration
    config: WipeTowerConfig,
    /// Position
    pos: Vec2f,
    /// Calculated depth
    depth: f32,
    /// Current Z position
    z_pos: f32,
    /// Current layer height
    layer_height: f32,
    /// Current tool
    current_tool: usize,
    /// Filament parameters per extruder
    filament_params: Vec<FilamentParameters>,
    /// Layer plan
    plan: Vec<WipeTowerLayerInfo>,
    /// Current layer iterator index
    layer_idx: usize,
    /// First layer index
    first_layer_idx: Option<usize>,
    /// Number of layer changes
    num_layer_changes: u32,
    /// Number of tool changes
    num_tool_changes: u32,
    /// Whether to print brim
    print_brim: bool,
    /// Current wipe shape direction
    current_shape: WipeShape,
    /// Depth traversed in current layer
    depth_traversed: f32,
    /// Whether current layer is finished
    current_layer_finished: bool,
    /// Left to right direction flag
    left_to_right: bool,
    /// Extra spacing factor
    extra_spacing: f32,
    /// TPU fixed spacing
    tpu_fixed_spacing: f32,
    /// Used filament length per extruder
    used_filament_length: Vec<f32>,
    /// Perimeter width
    perimeter_width: f32,
    /// Nozzle change perimeter width
    nozzle_change_perimeter_width: f32,
    /// Extrusion flow
    extrusion_flow: f32,
    /// Y shift
    y_shift: f32,
    /// Internal rotation angle
    internal_rotation: f32,
    /// Real brim width
    brim_width_real: f32,
    /// Old temperature
    old_temperature: i32,
    /// Maximum color changes
    max_color_changes: usize,
    /// Outer wall polygons per Z height
    outer_wall: HashMap<i32, Vec<Polyline>>,
    /// Wall skip points
    wall_skip_points: Vec<Vec2f>,
    /// Wipe tower blocks (for multi-extruder)
    wipe_tower_blocks: Vec<WipeTowerBlock>,
    /// All layer depth info
    all_layers_depth: Vec<Vec<BlockDepthInfo>>,
    /// Last block ID
    last_block_id: i32,
    /// Current block pointer
    cur_block_idx: Option<usize>,
    /// Block infill gap widths
    block_infill_gap_width: HashMap<i32, (f32, f32)>,
    /// Nozzle change result
    nozzle_change_result: NozzleChangeResult,
    /// Last layer IDs per nozzle
    last_layer_id: Vec<i32>,
    /// Has TPU filament
    has_tpu_filament: bool,
    /// Need reverse travel
    need_reverse_travel: bool,
    /// Rib length
    rib_length: f32,
    /// Rib offset
    rib_offset: Vec2f,
    /// Filament map
    filament_map: Vec<i32>,
    /// Used filament IDs
    used_filament_ids: Vec<i32>,
    /// Filament categories
    filament_categories: Vec<i32>,
    /// Adhesion enabled
    adhesion: bool,
    /// Is multiple nozzle setup
    is_multiple_nozzle: bool,
}

impl WipeTower {
    // Create a new wipe tower
    pub fn new(config: WipeTowerConfig, initial_tool: usize, num_filaments: usize) -> Self {
        let perimeter_width = config.width * WIDTH_TO_NOZZLE_RATIO / config.width.max(1.0);
        let nozzle_change_perimeter_width = perimeter_width;

        // Calculate extrusion flow
        let extrusion_flow = {
            let area = 0.2 * (perimeter_width - 0.2 * (1.0 - PI / 4.0));
            let filament_area = PI * 0.875 * 0.875;
            area / filament_area
        };

        Self {
            pos: Vec2f::new(config.pos_x, config.pos_y),
            depth: config.depth,
            z_pos: 0.0,
            layer_height: 0.2,
            current_tool: initial_tool,
            filament_params: vec![FilamentParameters::default(); num_filaments],
            plan: vec![],
            layer_idx: 0,
            first_layer_idx: None,
            num_layer_changes: 0,
            num_tool_changes: 0,
            print_brim: true,
            current_shape: WipeShape::Normal,
            depth_traversed: 0.0,
            current_layer_finished: false,
            left_to_right: true,
            extra_spacing: config.extra_spacing,
            tpu_fixed_spacing: 0.0,
            used_filament_length: vec![0.0; num_filaments],
            perimeter_width,
            nozzle_change_perimeter_width,
            extrusion_flow,
            y_shift: 0.0,
            internal_rotation: config.rotation_angle,
            brim_width_real: config.brim_width,
            old_temperature: 0,
            max_color_changes: 0,
            outer_wall: HashMap::new(),
            wall_skip_points: vec![],
            wipe_tower_blocks: vec![],
            all_layers_depth: vec![],
            last_block_id: 0,
            cur_block_idx: None,
            block_infill_gap_width: HashMap::new(),
            nozzle_change_result: NozzleChangeResult::default(),
            last_layer_id: vec![-1; num_filaments],
            has_tpu_filament: false,
            need_reverse_travel: false,
            rib_length: 0.0,
            rib_offset: Vec2f::zero(),
            filament_map: (0..num_filaments as i32).collect(),
            used_filament_ids: vec![],
            filament_categories: vec![],
            adhesion: true,
            is_multiple_nozzle: false,
            config,
        }
    }

    /// Set extruder parameters
    pub fn set_extruder(&mut self, idx: usize, params: FilamentParameters) {
        if idx >= self.filament_params.len() {
            self.filament_params
                .resize(idx + 1, FilamentParameters::default());
        }
        self.filament_params[idx] = params;
    }

    /// Set filament map
    pub fn set_filament_map(&mut self, map: Vec<i32>) {
        self.filament_map = map;
    }

    /// Set has TPU filament
    pub fn set_has_tpu_filament(&mut self, has_tpu: bool) {
        self.has_tpu_filament = has_tpu;
    }

    /// Check if has TPU filament
    pub fn has_tpu_filament(&self) -> bool {
        self.has_tpu_filament
    }

    /// Set layer parameters
    pub fn set_layer(
        &mut self,
        z: f32,
        layer_height: f32,
        max_tool_changes: usize,
        is_first_layer: bool,
        is_last_layer: bool,
    ) {
        self.z_pos = z;
        self.layer_height = layer_height;
        self.max_color_changes = max_tool_changes;

        // Find the layer in the plan
        self.layer_idx = self
            .plan
            .iter()
            .position(|l| (l.z - z).abs() < WT_EPSILON)
            .unwrap_or(0);

        self.depth_traversed = 0.0;
        self.current_layer_finished = false;

        // Update perimeter width based on layer
        if is_first_layer {
            // Slightly wider on first layer for adhesion
            self.perimeter_width =
                self.config.width * WIDTH_TO_NOZZLE_RATIO / self.config.width.max(1.0) * 1.05;
        } else {
            self.perimeter_width =
                self.config.width * WIDTH_TO_NOZZLE_RATIO / self.config.width.max(1.0);
        }

        // Calculate extrusion flow for this layer
        let area = layer_height * (self.perimeter_width - layer_height * (1.0 - PI / 4.0));
        let filament_area = PI * 0.875 * 0.875;
        self.extrusion_flow = area / filament_area;

        self.num_layer_changes += 1;
    }

    /// Get tower width
    pub fn width(&self) -> f32 {
        self.config.width
    }

    /// Get tower depth
    pub fn get_depth(&self) -> f32 {
        self.depth
    }

    /// Get brim width
    pub fn get_brim_width(&self) -> f32 {
        self.brim_width_real
    }

    /// Get tower height
    pub fn get_height(&self) -> f32 {
        self.config.height
    }

    /// Get current position
    pub fn position(&self) -> Vec2f {
        self.pos
    }

    /// Check if finished
    pub fn finished(&self) -> bool {
        self.layer_idx >= self.plan.len()
    }

    /// Check if current layer is finished
    pub fn layer_finished(&self) -> bool {
        self.current_layer_finished
    }

    /// Get used filament lengths
    pub fn get_used_filament(&self) -> &[f32] {
        &self.used_filament_length
    }

    /// Get number of tool changes
    pub fn get_number_of_toolchanges(&self) -> u32 {
        self.num_tool_changes
    }

    /// Get bounding box
    pub fn get_bounding_box(&self) -> BoundingBoxF {
        BoundingBoxF::from_coords(
            self.pos.x as f64,
            self.pos.y as f64,
            (self.pos.x + self.config.width) as f64,
            (self.pos.y + self.depth) as f64,
        )
    }

    /// Convert volume to extrusion length
    fn volume_to_length(&self, volume: f32, line_width: f32, layer_height: f32) -> f32 {
        let area = layer_height * (line_width - layer_height * (1.0 - PI / 4.0));
        volume / area
    }

    /// Convert extrusion length to volume
    fn length_to_volume(&self, length: f32, line_width: f32, layer_height: f32) -> f32 {
        let area = layer_height * (line_width - layer_height * (1.0 - PI / 4.0));
        length * area
    }

    /// Extrusion flow for nozzle change
    fn nozzle_change_extrusion_flow(&self, layer_height: f32) -> f32 {
        let area =
            layer_height * (self.nozzle_change_perimeter_width - layer_height * (1.0 - PI / 4.0));
        let filament_area = PI * 0.875 * 0.875;
        area / filament_area
    }

    /// Check if two filaments are in the same extruder
    fn is_same_extruder(&self, filament1: usize, filament2: usize) -> bool {
        if filament1 >= self.filament_map.len() || filament2 >= self.filament_map.len() {
            return false;
        }
        self.filament_map[filament1] == self.filament_map[filament2]
    }

    /// Check if two filaments use the same nozzle
    fn is_same_nozzle(&self, filament1: usize, filament2: usize) -> bool {
        // For single-extruder multi-material, all filaments use the same nozzle
        if self.config.semm {
            return true;
        }
        self.is_same_extruder(filament1, filament2)
    }

    /// Check if ramming is needed for tool change
    fn is_need_ramming(&self, old_tool: usize, new_tool: usize) -> bool {
        // Ramming is needed when changing to a different extruder
        !self.is_same_extruder(old_tool, new_tool) || !self.is_same_nozzle(old_tool, new_tool)
    }

    /// Check if filament is TPU
    fn is_tpu_filament(&self, filament_id: usize) -> bool {
        if filament_id >= self.filament_params.len() {
            return false;
        }
        self.filament_params[filament_id].material.to_uppercase() == "TPU"
    }

    /// Get minimum depth by height
    pub fn get_limit_depth_by_height(max_height: f32) -> f32 {
        for &(height, depth) in MIN_DEPTH_PER_HEIGHT {
            if max_height <= height {
                return depth;
            }
        }
        // Linear extrapolation for heights beyond the table
        let last = MIN_DEPTH_PER_HEIGHT.last().unwrap();
        let rate = last.1 / last.0;
        max_height * rate
    }

    /// Get auto brim width by height
    pub fn get_auto_brim_by_height(max_height: f32) -> f32 {
        // Brim width scales with tower height for stability
        (max_height / 50.0).clamp(1.0, 5.0)
    }

    /// Plan a tool change
    pub fn plan_toolchange(
        &mut self,
        z: f32,
        layer_height: f32,
        old_tool: usize,
        new_tool: usize,
        wipe_volume_ec: f32,
        wipe_volume_nc: f32,
        purge_volume: f32,
    ) {
        // Ensure z is not below the last planned layer
        assert!(self.plan.is_empty() || self.plan.last().unwrap().z <= z + WT_EPSILON);

        let wipe_volume = if self.is_same_extruder(old_tool, new_tool)
            && !self.is_same_nozzle(old_tool, new_tool)
        {
            wipe_volume_nc
        } else {
            wipe_volume_ec
        };

        // Add new layer if needed
        if self.plan.is_empty() || self.plan.last().unwrap().z + WT_EPSILON < z {
            self.plan.push(WipeTowerLayerInfo::new(z, layer_height));
        }

        // Record first layer with actual tool changes
        if self.first_layer_idx.is_none() && (!self.config.no_sparse_layers || old_tool != new_tool)
        {
            self.first_layer_idx = Some(self.plan.len() - 1);
        }

        // No actual tool change
        if old_tool == new_tool {
            return;
        }

        // Calculate depth for this tool change
        let width = self.config.width - 2.0 * self.perimeter_width;
        if width <= WT_EPSILON {
            return;
        }

        let length_to_extrude =
            self.volume_to_length(wipe_volume, self.perimeter_width, layer_height);
        let mut depth = (length_to_extrude / width).ceil() * self.perimeter_width;

        // Add nozzle change depth if needed
        let mut nozzle_change_depth = 0.0;
        let mut nozzle_change_length = 0.0;

        if self.is_need_ramming(old_tool, new_tool) {
            let filament_change_length = if !self.is_same_extruder(old_tool, new_tool) {
                self.config
                    .filament_change_length
                    .get(old_tool)
                    .copied()
                    .unwrap_or(20.0)
            } else {
                self.config
                    .filament_change_length_nc
                    .get(old_tool)
                    .copied()
                    .unwrap_or(20.0)
            };

            let e_flow = self.nozzle_change_extrusion_flow(layer_height);
            let length = filament_change_length / e_flow;
            let nozzle_change_line_count = (length
                / (self.config.width - 2.0 * self.nozzle_change_perimeter_width))
                .ceil() as i32;
            nozzle_change_depth =
                nozzle_change_line_count as f32 * self.nozzle_change_perimeter_width;
            nozzle_change_length = length;
            depth += nozzle_change_depth;
        }

        let mut tool_change = ToolChangeInfo::new(old_tool, new_tool);
        tool_change.required_depth = depth;
        tool_change.wipe_volume = wipe_volume;
        tool_change.wipe_length = length_to_extrude;
        tool_change.nozzle_change_depth = nozzle_change_depth;
        tool_change.nozzle_change_length = nozzle_change_length;
        tool_change.purge_volume = purge_volume;

        self.plan.last_mut().unwrap().tool_changes.push(tool_change);
        self.num_tool_changes += 1;
    }

    /// Plan the entire tower
    pub fn plan_tower(&mut self) {
        // Calculate maximum depth needed
        let mut max_depth = 0.0f32;
        for info in &self.plan {
            max_depth = max_depth.max(info.toolchanges_depth());
        }

        let min_wipe_tower_depth = Self::get_limit_depth_by_height(self.config.height);

        // Calculate extra spacing if tower is too thin
        if self.config.enable_wrapping_detection && max_depth < WT_EPSILON {
            max_depth = 15.0; // Default wrapping detection depth
        }

        if self.config.enable_timelapse_print && max_depth < WT_EPSILON {
            max_depth = min_wipe_tower_depth;
        }

        if max_depth + WT_EPSILON < min_wipe_tower_depth && !self.has_tpu_filament {
            self.extra_spacing = min_wipe_tower_depth / max_depth;
        } else {
            self.extra_spacing = 1.0;
        }

        // Apply spacing to layers
        let perimeter_width = self.perimeter_width;
        let config_width = self.config.width;
        let extra_spacing = self.extra_spacing;

        for (idx, info) in self.plan.iter_mut().enumerate() {
            if idx == 0 && extra_spacing > 1.0 + WT_EPSILON {
                // Solid fill for first layer
                info.extra_spacing = 1.0;
                for tc in &mut info.tool_changes {
                    let layer_height = info.height;
                    let area = layer_height * (perimeter_width - layer_height * (1.0 - PI / 4.0));
                    let x_to_wipe = tc.wipe_volume / area;
                    let line_len = config_width - 2.0 * perimeter_width;
                    let x_to_wipe_new = (x_to_wipe * extra_spacing / line_len).floor() * line_len;
                    let x_to_wipe_new = x_to_wipe_new.max(x_to_wipe);

                    let line_count = ((x_to_wipe_new - WT_EPSILON) / line_len).ceil() as i32;
                    let nozzle_change_line_count =
                        ((tc.nozzle_change_depth + WT_EPSILON) / perimeter_width) as i32;

                    tc.required_depth =
                        (line_count + nozzle_change_line_count) as f32 * perimeter_width;
                    tc.wipe_volume = x_to_wipe_new / x_to_wipe * tc.wipe_volume;
                    tc.wipe_length = x_to_wipe_new;
                }
            } else {
                info.extra_spacing = extra_spacing;
                for tc in &mut info.tool_changes {
                    tc.required_depth *= extra_spacing;
                }
            }
        }

        // Calculate final tower depth
        self.depth = 0.0;
        for info in &self.plan {
            self.depth = self.depth.max(info.toolchanges_depth());
        }

        // Ensure minimum depth
        self.depth = self.depth.max(min_wipe_tower_depth);
    }

    /// Generate the wipe tower
    pub fn generate(&mut self) -> Vec<Vec<ToolChangeResult>> {
        let mut results: Vec<Vec<ToolChangeResult>> = Vec::new();

        // Plan the tower first
        self.plan_tower();

        // Collect layer info to avoid borrow issues
        let layer_data: Vec<(f32, f32, usize, Vec<usize>)> = self
            .plan
            .iter()
            .map(|info| {
                let new_tools: Vec<usize> =
                    info.tool_changes.iter().map(|tc| tc.new_tool).collect();
                (info.z, info.height, info.tool_changes.len(), new_tools)
            })
            .collect();

        let num_layers = layer_data.len();

        for (layer_idx, (z, height, num_tool_changes, new_tools)) in
            layer_data.into_iter().enumerate()
        {
            let mut layer_results: Vec<ToolChangeResult> = Vec::new();

            // Set layer
            let is_first_layer = layer_idx == 0;
            let is_last_layer = layer_idx == num_layers - 1;
            self.set_layer(z, height, num_tool_changes, is_first_layer, is_last_layer);

            // Generate tool changes for this layer
            for new_tool in new_tools {
                let result = self.tool_change(new_tool);
                layer_results.push(result);
            }

            // Finish the layer
            let finish_result = self.finish_layer();
            layer_results.push(finish_result);

            results.push(layer_results);
        }

        results
    }

    /// Perform a tool change
    pub fn tool_change(&mut self, new_tool: usize) -> ToolChangeResult {
        let old_tool = self.current_tool;
        let layer_info = &self.plan[self.layer_idx];

        // Find the tool change info
        let tc_info = layer_info
            .tool_changes
            .iter()
            .find(|tc| tc.new_tool == new_tool)
            .cloned()
            .unwrap_or_else(|| ToolChangeInfo::new(old_tool, new_tool));

        let wipe_depth = tc_info.required_depth;
        let wipe_length = tc_info.wipe_length;
        let purge_volume = tc_info.purge_volume;
        let nozzle_change_depth = tc_info.nozzle_change_depth;

        // Create cleaning box
        let cleaning_box = BoxCoordinates::new(
            self.perimeter_width,
            self.depth_traversed + self.perimeter_width,
            self.config.width - 2.0 * self.perimeter_width,
            wipe_depth - self.perimeter_width,
        );

        // Create writer
        let mut writer = WipeTowerWriter::new(
            self.layer_height,
            self.perimeter_width,
            self.config.gcode_flavor,
            &self.filament_params,
        );

        writer.set_initial_position(
            Vec2f::new(self.perimeter_width, self.depth_traversed),
            self.internal_rotation,
            self.y_shift,
        );
        writer.set_initial_tool(old_tool);
        writer.set_z(self.z_pos);
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_wipe_tower_dimensions(self.config.width, self.depth);
        writer.set_first_layer(self.layer_idx == 0);

        let is_first_layer = self.layer_idx == 0;
        let feedrate = if is_first_layer {
            self.config.first_layer_speed * 60.0
        } else {
            self.config.travel_speed * 60.0
        };

        // Travel to start position
        writer.feedrate(feedrate);

        // Comment for tool change
        writer.comment(&format!("Tool change from T{} to T{}", old_tool, new_tool));

        // Retract before tool change
        if old_tool < self.filament_params.len() {
            let retract = self.filament_params[old_tool].retract_length;
            let retract_speed = self.filament_params[old_tool].retract_speed * 60.0;
            if retract > 0.0 {
                writer.retract(retract, retract_speed);
            }
        }

        // Perform ramming/wiping if needed
        if self.is_need_ramming(old_tool, new_tool) {
            self.toolchange_unload(&mut writer, &cleaning_box);
        }

        // Actual tool change
        writer.set_tool(new_tool);
        self.current_tool = new_tool;

        // Load new filament
        if new_tool < self.filament_params.len() {
            let load = self.filament_params[new_tool].retract_length;
            let load_speed = self.filament_params[new_tool].retract_speed * 60.0;
            if load > 0.0 {
                writer.load(load, load_speed);
            }
        }

        // Wipe
        self.toolchange_wipe(&mut writer, &cleaning_box, wipe_length);

        // Update state
        self.depth_traversed += wipe_depth;
        self.left_to_right = !self.left_to_right;

        // Construct result
        self.construct_tcr(&writer, false, old_tool, false, true, purge_volume)
    }

    /// Unload filament during tool change
    fn toolchange_unload(&self, writer: &mut WipeTowerWriter, cleaning_box: &BoxCoordinates) {
        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;
        let line_width = self.perimeter_width;

        // Simple ramming pattern - back and forth
        let y = cleaning_box.ld.y + line_width / 2.0;
        writer.travel(xl, y);

        // Ram by extruding quickly back and forth
        let ram_length = (xr - xl).min(20.0);
        writer.feedrate(self.config.max_speed * 60.0);
        writer.extrude(xl + ram_length, y);
        writer.extrude(xl, y);
    }

    /// Wipe during tool change
    fn toolchange_wipe(
        &self,
        writer: &mut WipeTowerWriter,
        cleaning_box: &BoxCoordinates,
        wipe_length: f32,
    ) {
        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;
        let line_len = xr - xl;

        if line_len <= 0.0 {
            return;
        }

        let num_lines = (wipe_length / line_len).ceil() as i32;
        let dy = self.perimeter_width;
        let mut y = cleaning_box.ld.y + dy / 2.0;
        let wipe_speed = self.config.max_speed * 60.0 * 0.6; // 60% of max speed for wiping

        writer.feedrate(wipe_speed);

        for i in 0..num_lines {
            let (x_start, x_end) = if i % 2 == 0 { (xl, xr) } else { (xr, xl) };

            writer.travel(x_start, y);
            writer.extrude(x_end, y);

            y += dy;
            if y > cleaning_box.lu.y - dy / 2.0 {
                break;
            }
        }

        // Add wipe path for post-processing
        writer.add_wipe_point(Vec2f::new(xl, y));
        writer.add_wipe_point(Vec2f::new(xr, y));
    }

    /// Finish the current layer
    pub fn finish_layer(&mut self) -> ToolChangeResult {
        let is_first_layer = self.layer_idx == 0;

        // Create writer
        let mut writer = WipeTowerWriter::new(
            self.layer_height,
            self.perimeter_width,
            self.config.gcode_flavor,
            &self.filament_params,
        );

        writer.set_initial_position(
            Vec2f::new(self.perimeter_width, self.depth_traversed),
            self.internal_rotation,
            self.y_shift,
        );
        writer.set_initial_tool(self.current_tool);
        writer.set_z(self.z_pos);
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_wipe_tower_dimensions(self.config.width, self.depth);
        writer.set_first_layer(is_first_layer);

        let feedrate = if is_first_layer {
            self.config.first_layer_speed * 60.0
        } else {
            self.config.travel_speed * 60.0
        };

        writer.feedrate(feedrate);

        // Fill remaining depth
        let fill_box = BoxCoordinates::new(
            self.perimeter_width,
            self.depth_traversed,
            self.config.width - 2.0 * self.perimeter_width,
            self.depth - self.depth_traversed - self.perimeter_width,
        );

        // Sparse infill if there's remaining space
        if fill_box.height() > self.perimeter_width {
            let sparse_factor = if is_first_layer {
                1.0
            } else {
                self.extra_spacing
            };
            let spacing = self.perimeter_width * sparse_factor;
            writer.rectangle_fill_box(&fill_box, spacing);
        }

        // Draw outer perimeter
        let wt_box = BoxCoordinates::new(0.0, 0.0, self.config.width, self.depth);

        // Only draw perimeter if this is first layer or we need it
        if is_first_layer || !self.config.no_sparse_layers {
            writer.rectangle(&wt_box);
        }

        // Print brim on first layer
        if is_first_layer && self.print_brim && self.brim_width_real > 0.0 {
            let brim_spacing = self.perimeter_width * 0.9;
            let num_loops = (self.brim_width_real / brim_spacing).ceil() as i32;

            for i in 1..=num_loops {
                let offset = i as f32 * brim_spacing;
                let mut brim_box = wt_box;
                brim_box.expand(offset);
                writer.rectangle(&brim_box);
            }

            self.print_brim = false;
        }

        self.current_layer_finished = true;

        self.construct_tcr(&writer, false, self.current_tool, true, false, 0.0)
    }

    /// Construct a ToolChangeResult from writer state
    fn construct_tcr(
        &mut self,
        writer: &WipeTowerWriter,
        priming: bool,
        old_tool: usize,
        is_finish: bool,
        is_tool_change: bool,
        purge_volume: f32,
    ) -> ToolChangeResult {
        // Track filament usage
        if old_tool < self.used_filament_length.len() {
            self.used_filament_length[old_tool] += writer.used_filament_length;
        }

        ToolChangeResult {
            print_z: self.z_pos,
            layer_height: self.layer_height,
            gcode: writer.gcode().to_string(),
            extrusions: writer.extrusions().to_vec(),
            start_pos: Vec2f::new(
                self.pos.x + writer.start_pos_rotated().x,
                self.pos.y + writer.start_pos_rotated().y,
            ),
            end_pos: Vec2f::new(
                self.pos.x + writer.pos_rotated().x,
                self.pos.y + writer.pos_rotated().y,
            ),
            elapsed_time: writer.elapsed_time(),
            priming,
            is_tool_change,
            tool_change_start_pos: Vec2f::new(
                self.pos.x + writer.start_pos_rotated().x,
                self.pos.y + writer.start_pos_rotated().y,
            ),
            wipe_path: writer
                .wipe_path()
                .iter()
                .map(|p| Vec2f::new(self.pos.x + p.x, self.pos.y + p.y))
                .collect(),
            purge_volume,
            initial_tool: old_tool as i32,
            new_tool: self.current_tool as i32,
            is_finish_first: is_finish,
            nozzle_change_result: self.nozzle_change_result.clone(),
        }
    }

    /// Prime the wipe tower (initial priming)
    pub fn prime(&mut self, tools_to_prime: &[usize]) -> Vec<ToolChangeResult> {
        let mut results = Vec::new();

        for &tool in tools_to_prime {
            let mut writer = WipeTowerWriter::new(
                self.layer_height,
                self.perimeter_width,
                self.config.gcode_flavor,
                &self.filament_params,
            );

            writer.set_initial_position(
                Vec2f::new(self.perimeter_width, 0.0),
                self.internal_rotation,
                self.y_shift,
            );
            writer.set_initial_tool(tool);
            writer.set_z(self.z_pos);

            // Simple priming line
            let prime_length = self.config.width - 2.0 * self.perimeter_width;
            writer.feedrate(self.config.first_layer_speed * 60.0);
            writer.travel(self.perimeter_width, self.perimeter_width);
            writer.extrude(self.perimeter_width + prime_length, self.perimeter_width);

            let result = self.construct_tcr(&writer, true, tool, false, false, 0.0);
            results.push(result);
        }

        results
    }
}

// ============================================================================
// Utility Functions (faithful free-function ports from WipeTower.cpp)
// ============================================================================

// WipeTower.cpp:27
// inline float align_round(float value, float base)
// {
//     return std::round(value / base) * base;
// }
#[inline]
pub fn align_round(value: f32, base: f32) -> f32 {
    (value / base).round() * base
}

// WipeTower.cpp:32
// inline float align_ceil(float value, float base)
// {
//     return std::ceil(value / base) * base;
// }
#[inline]
pub fn align_ceil(value: f32, base: f32) -> f32 {
    (value / base).ceil() * base
}

// WipeTower.cpp:37
// inline float align_floor(float value, float base)
// {
//     return std::floor((value) / base) * base;
// }
#[inline]
pub fn align_floor(value: f32, base: f32) -> f32 {
    (value / base).floor() * base
}

// WipeTower.cpp:42
// static bool is_valid_gcode(const std::string &gcode)
// Walks the gcode string line by line, trimming spaces; returns true as soon as
// a non-empty line is found that does not start with ';'.
pub fn is_valid_gcode(gcode: &str) -> bool {
    // WipeTower.cpp:48-66 — iterate over '\n'-terminated lines, skipping the
    // trailing partial line (the C++ loop only inspects a line when it sees the
    // terminating '\n', so a final line without a newline is ignored).
    let bytes = gcode.as_bytes();
    let str_size = bytes.len();
    let mut start_index = 0usize;
    let mut end_index = 0usize;
    let mut is_valid = false;
    while end_index < str_size {
        if bytes[end_index] != b'\n' {
            end_index += 1;
            continue;
        }

        if end_index > start_index {
            // WipeTower.cpp:55-57 — substr then erase leading/trailing spaces.
            let line_str = &gcode[start_index..end_index];
            let trimmed = line_str.trim_matches(' ');
            if !trimmed.is_empty() && trimmed.as_bytes()[0] != b';' {
                is_valid = true;
                break;
            }
        }

        start_index = end_index + 1;
        end_index = start_index;
    }

    is_valid
}

// WipeTower.cpp:259
// Polygon generate_rectange(const Line &line, coord_t offset)
// Builds an oriented rectangle of the given half-width `offset` around `line`.
// Faithful 1:1 port reusing the crate `Line`/`Point`/`Polygon` primitives.
// `coord_t` -> `i64`, `coordf_t`/`double` -> `f64`.
pub fn generate_rectange(line: &Line, offset: i64) -> Polygon {
    // WipeTower.cpp:261-262
    let p1 = line.a;
    let p2 = line.b;

    // WipeTower.cpp:264-265
    let dx = (p2.x() - p1.x()) as f64;
    let dy = (p2.y() - p1.y()) as f64;

    // WipeTower.cpp:267
    let length = (dx * dx + dy * dy).sqrt();

    // WipeTower.cpp:269-270
    let ux = dx / length;
    let uy = dy / length;

    // WipeTower.cpp:272-273
    let vx = -uy;
    let vy = ux;

    // WipeTower.cpp:275-276
    let ox = vx * offset as f64;
    let oy = vy * offset as f64;

    // WipeTower.cpp:278-283
    // Points rect; rect.resize(4);
    // rect[0] = {p1.x() + ox, p1.y() + oy};
    // rect[1] = {p1.x() - ox, p1.y() - oy};
    // rect[2] = {p2.x() - ox, p2.y() - oy};
    // rect[3] = {p2.x() + ox, p2.y() + oy};
    // NOTE: C++ assigns f64 expressions into coord_t (i64) Point components,
    // which truncates toward zero. Mirror that with `as i64`.
    let mut rect: Vec<Point> = Vec::with_capacity(4);
    rect.push(Point::new(
        (p1.x() as f64 + ox) as i64,
        (p1.y() as f64 + oy) as i64,
    ));
    rect.push(Point::new(
        (p1.x() as f64 - ox) as i64,
        (p1.y() as f64 - oy) as i64,
    ));
    rect.push(Point::new(
        (p2.x() as f64 - ox) as i64,
        (p2.y() as f64 - oy) as i64,
    ));
    rect.push(Point::new(
        (p2.x() as f64 + ox) as i64,
        (p2.y() as f64 + oy) as i64,
    ));

    // WipeTower.cpp:284-285
    Polygon::from_points(rect)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec2f_operations() {
        let v1 = Vec2f::new(3.0, 4.0);
        let v2 = Vec2f::new(1.0, 2.0);

        assert!((v1.norm() - 5.0).abs() < 1e-6);
        assert!((v1.dot(&v2) - 11.0).abs() < 1e-6);

        let sum = v1 + v2;
        assert!((sum.x - 4.0).abs() < 1e-6);
        assert!((sum.y - 6.0).abs() < 1e-6);

        let diff = v1 - v2;
        assert!((diff.x - 2.0).abs() < 1e-6);
        assert!((diff.y - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_box_coordinates() {
        let box_coords = BoxCoordinates::new(0.0, 0.0, 10.0, 5.0);

        assert!((box_coords.ld.x - 0.0).abs() < 1e-6);
        assert!((box_coords.ld.y - 0.0).abs() < 1e-6);
        assert!((box_coords.ru.x - 10.0).abs() < 1e-6);
        assert!((box_coords.ru.y - 5.0).abs() < 1e-6);
        assert!((box_coords.width() - 10.0).abs() < 1e-6);
        assert!((box_coords.height() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_box_coordinates_expand() {
        let mut box_coords = BoxCoordinates::new(5.0, 5.0, 10.0, 10.0);
        box_coords.expand(2.0);

        assert!((box_coords.ld.x - 3.0).abs() < 1e-6);
        assert!((box_coords.ld.y - 3.0).abs() < 1e-6);
        assert!((box_coords.ru.x - 17.0).abs() < 1e-6);
        assert!((box_coords.ru.y - 17.0).abs() < 1e-6);
    }

    #[test]
    fn test_wipe_tower_config_default() {
        let config = WipeTowerConfig::default();

        assert!((config.width - 60.0).abs() < 1e-6);
        assert!(config.travel_speed > 0.0);
        assert!(config.first_layer_speed > 0.0);
    }

    #[test]
    fn test_filament_parameters_default() {
        let params = FilamentParameters::default();

        assert_eq!(params.material, "PLA");
        assert!(!params.is_soluble);
        assert!(!params.is_support);
        assert!(params.nozzle_diameter > 0.0);
    }

    #[test]
    fn test_wipe_tower_creation() {
        let config = WipeTowerConfig::default();
        let tower = WipeTower::new(config, 0, 2);

        assert_eq!(tower.current_tool, 0);
        assert_eq!(tower.filament_params.len(), 2);
        // A tower with no plan is technically "finished" (no layers to process)
        assert!(tower.finished());
    }

    #[test]
    fn test_wipe_tower_plan_toolchange() {
        let config = WipeTowerConfig::default();
        let mut tower = WipeTower::new(config, 0, 2);

        tower.plan_toolchange(0.2, 0.2, 0, 1, 50.0, 30.0, 0.0);

        assert_eq!(tower.plan.len(), 1);
        assert_eq!(tower.plan[0].tool_changes.len(), 1);
        assert_eq!(tower.plan[0].tool_changes[0].old_tool, 0);
        assert_eq!(tower.plan[0].tool_changes[0].new_tool, 1);
    }

    #[test]
    fn test_wipe_tower_plan_multiple_layers() {
        let config = WipeTowerConfig::default();
        let mut tower = WipeTower::new(config, 0, 3);

        tower.plan_toolchange(0.2, 0.2, 0, 1, 50.0, 30.0, 0.0);
        tower.plan_toolchange(0.4, 0.2, 1, 2, 50.0, 30.0, 0.0);
        tower.plan_toolchange(0.6, 0.2, 2, 0, 50.0, 30.0, 0.0);

        assert_eq!(tower.plan.len(), 3);
        assert_eq!(tower.num_tool_changes, 3);
    }

    #[test]
    fn test_wipe_tower_limit_depth() {
        assert!(WipeTower::get_limit_depth_by_height(50.0) >= 10.0);
        assert!(WipeTower::get_limit_depth_by_height(100.0) >= 15.0);
        assert!(WipeTower::get_limit_depth_by_height(200.0) >= 25.0);
    }

    #[test]
    fn test_wipe_tower_writer() {
        let mut writer = WipeTowerWriter::new(
            0.2,
            0.4,
            GCodeFlavor::Marlin,
            &[FilamentParameters::default()],
        );

        writer.set_initial_position(Vec2f::new(0.0, 0.0), 0.0, 0.0);
        writer.set_z(0.2);
        writer.feedrate(1500.0);
        writer.travel(10.0, 10.0);
        writer.extrude(20.0, 10.0);

        let gcode = writer.gcode();
        assert!(gcode.contains("G0"));
        assert!(gcode.contains("G1"));
        assert!(gcode.contains("F1500"));
    }

    #[test]
    fn test_wipe_tower_writer_rectangle() {
        let mut writer = WipeTowerWriter::new(
            0.2,
            0.4,
            GCodeFlavor::Marlin,
            &[FilamentParameters::default()],
        );

        writer.set_initial_position(Vec2f::new(0.0, 0.0), 0.0, 0.0);
        writer.set_z(0.2);
        writer.feedrate(1500.0);

        let box_coords = BoxCoordinates::new(0.0, 0.0, 10.0, 10.0);
        writer.rectangle(&box_coords);

        assert!(writer.extrusions().len() >= 4);
    }

    #[test]
    fn test_tool_change_result() {
        let mut result = ToolChangeResult::default();
        result
            .extrusions
            .push(Extrusion::new(Vec2f::new(0.0, 0.0), 0.4, 0));
        result
            .extrusions
            .push(Extrusion::new(Vec2f::new(10.0, 0.0), 0.4, 0));
        result
            .extrusions
            .push(Extrusion::new(Vec2f::new(10.0, 10.0), 0.4, 0));

        let length = result.total_extrusion_length_in_plane();
        assert!((length - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_wipe_tower_layer_info() {
        let mut layer = WipeTowerLayerInfo::new(0.2, 0.2);

        let mut tc1 = ToolChangeInfo::new(0, 1);
        tc1.required_depth = 5.0;
        let mut tc2 = ToolChangeInfo::new(1, 2);
        tc2.required_depth = 7.0;

        layer.tool_changes.push(tc1);
        layer.tool_changes.push(tc2);

        assert!((layer.toolchanges_depth() - 12.0).abs() < 1e-6);
    }

    #[test]
    fn test_align_functions() {
        assert!((align_round(5.3, 1.0) - 5.0).abs() < 1e-6);
        assert!((align_round(5.6, 1.0) - 6.0).abs() < 1e-6);
        assert!((align_ceil(5.1, 1.0) - 6.0).abs() < 1e-6);
        assert!((align_floor(5.9, 1.0) - 5.0).abs() < 1e-6);

        assert!((align_round(5.3, 0.5) - 5.5).abs() < 1e-6);
        assert!((align_ceil(5.3, 0.5) - 5.5).abs() < 1e-6);
        assert!((align_floor(5.3, 0.5) - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_is_valid_gcode() {
        assert!(is_valid_gcode("G0 X10 Y10\n"));
        assert!(is_valid_gcode("; comment\nG1 X20 Y20\n"));
        assert!(!is_valid_gcode("; just a comment\n"));
        assert!(!is_valid_gcode("  ; another comment\n"));
        assert!(!is_valid_gcode(""));
    }

    #[test]
    fn test_extrusion() {
        let extrusion = Extrusion::new(Vec2f::new(10.0, 20.0), 0.4, 0);

        assert!((extrusion.pos.x - 10.0).abs() < 1e-6);
        assert!((extrusion.pos.y - 20.0).abs() < 1e-6);
        assert!((extrusion.width - 0.4).abs() < 1e-6);
        assert_eq!(extrusion.tool, 0);
    }

    #[test]
    fn test_wipe_tower_is_same_extruder() {
        let config = WipeTowerConfig::default();
        let mut tower = WipeTower::new(config, 0, 4);

        // Default map: each filament has its own extruder
        assert!(tower.is_same_extruder(0, 0));
        assert!(!tower.is_same_extruder(0, 1));

        // Set custom map where filaments 0 and 2 share extruder 0
        tower.set_filament_map(vec![0, 1, 0, 1]);
        assert!(tower.is_same_extruder(0, 2));
        assert!(tower.is_same_extruder(1, 3));
        assert!(!tower.is_same_extruder(0, 1));
    }

    #[test]
    fn test_wipe_tower_generate_simple() {
        let mut config = WipeTowerConfig::default();
        config.height = 10.0;

        let mut tower = WipeTower::new(config, 0, 2);
        tower.plan_toolchange(0.2, 0.2, 0, 1, 50.0, 30.0, 0.0);

        let results = tower.generate();

        assert!(!results.is_empty());
        assert!(!results[0].is_empty());

        // Check that we have G-code
        for layer_results in &results {
            for result in layer_results {
                assert!(!result.gcode.is_empty());
            }
        }
    }

    #[test]
    fn test_nozzle_change_result_default() {
        let result = NozzleChangeResult::default();

        assert!(result.gcode.is_empty());
        assert!((result.start_pos.x - 0.0).abs() < 1e-6);
        assert!((result.end_pos.y - 0.0).abs() < 1e-6);
        assert!(!result.is_extruder_change);
    }

    #[test]
    fn test_wipe_shape_default() {
        let shape = WipeShape::default();
        assert_eq!(shape, WipeShape::Normal);
    }

    #[test]
    fn test_bed_shape_default() {
        let shape = BedShape::default();
        assert_eq!(shape, BedShape::Rectangular);
    }

    #[test]
    fn test_gcode_flavor_default() {
        let flavor = GCodeFlavor::default();
        assert_eq!(flavor, GCodeFlavor::Marlin);
    }

    #[test]
    fn test_vec2f_rotate() {
        let v = Vec2f::new(1.0, 0.0);
        let rotated = v.rotate(std::f32::consts::FRAC_PI_2);

        assert!((rotated.x - 0.0).abs() < 1e-5);
        assert!((rotated.y - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_vec2f_normalized() {
        let v = Vec2f::new(3.0, 4.0);
        let n = v.normalized();

        assert!((n.norm() - 1.0).abs() < 1e-6);
        assert!((n.x - 0.6).abs() < 1e-6);
        assert!((n.y - 0.8).abs() < 1e-6);
    }

    #[test]
    fn test_vec2f_zero() {
        let v = Vec2f::zero();
        assert!((v.x - 0.0).abs() < 1e-6);
        assert!((v.y - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_vec2f_neg() {
        let v = Vec2f::new(3.0, -4.0);
        let neg = -v;

        assert!((neg.x - (-3.0)).abs() < 1e-6);
        assert!((neg.y - 4.0).abs() < 1e-6);
    }

    #[test]
    fn test_wipe_tower_volume_length_conversion() {
        let config = WipeTowerConfig::default();
        let tower = WipeTower::new(config, 0, 1);

        let volume = 10.0; // mm³
        let line_width = 0.4;
        let layer_height = 0.2;

        let length = tower.volume_to_length(volume, line_width, layer_height);
        let back_to_volume = tower.length_to_volume(length, line_width, layer_height);

        assert!((back_to_volume - volume).abs() < 0.01);
    }

    #[test]
    fn test_wipe_tower_auto_brim() {
        let brim_50 = WipeTower::get_auto_brim_by_height(50.0);
        let brim_100 = WipeTower::get_auto_brim_by_height(100.0);
        let brim_250 = WipeTower::get_auto_brim_by_height(250.0);

        assert!(brim_50 >= 1.0);
        assert!(brim_100 >= brim_50);
        assert!(brim_250 <= 5.0);
    }

    #[test]
    fn test_wipe_tower_set_layer() {
        let config = WipeTowerConfig::default();
        let mut tower = WipeTower::new(config, 0, 2);

        tower.plan_toolchange(0.2, 0.2, 0, 1, 50.0, 30.0, 0.0);
        tower.plan_tower();

        tower.set_layer(0.2, 0.2, 1, true, false);

        assert!((tower.z_pos - 0.2).abs() < 1e-6);
        assert!((tower.layer_height - 0.2).abs() < 1e-6);
    }

    #[test]
    fn test_wipe_tower_getters() {
        let mut config = WipeTowerConfig::default();
        config.width = 60.0;
        config.height = 100.0;
        config.brim_width = 3.0;

        let tower = WipeTower::new(config, 0, 2);

        assert!((tower.width() - 60.0).abs() < 1e-6);
        assert!((tower.get_height() - 100.0).abs() < 1e-6);
        assert!((tower.get_brim_width() - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_wipe_tower_prime() {
        let config = WipeTowerConfig::default();
        let mut tower = WipeTower::new(config, 0, 2);

        tower.z_pos = 0.2;
        tower.layer_height = 0.2;

        let results = tower.prime(&[0, 1]);

        assert_eq!(results.len(), 2);
        for result in &results {
            assert!(result.priming);
            assert!(!result.gcode.is_empty());
        }
    }
}
