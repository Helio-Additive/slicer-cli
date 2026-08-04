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

/// WipeTower.cpp:26
/// `static const std::map<float, float> nozzle_diameter_to_nozzle_change_width
///      {{0.2f, 0.5f}, {0.4f, 1.0f}, {0.6f, 1.2f}, {0.8f, 1.4f}};`
/// Note this is NOT `nozzle_diameter * Width_To_Nozzle_Ratio`: for the common 0.4
/// nozzle the nozzle-change lines are 1.0mm wide, twice the 0.5mm tower perimeter.
const NOZZLE_DIAMETER_TO_NOZZLE_CHANGE_WIDTH: [(f32, f32); 4] =
    [(0.2, 0.5), (0.4, 1.0), (0.6, 1.2), (0.8, 1.4)];

/// WipeTower.cpp:1901 — `nozzle_diameter_to_nozzle_change_width.at(nozzle_diameter)`.
/// C++ uses `std::map::at`, i.e. an EXACT float key; an unlisted diameter throws.
/// We keep slicing instead and fall back to the width-to-nozzle ratio.
fn nozzle_change_width_for_nozzle(nozzle_diameter: f32) -> f32 {
    for (d, w) in NOZZLE_DIAMETER_TO_NOZZLE_CHANGE_WIDTH {
        if (nozzle_diameter - d).abs() < 1e-6 {
            return w;
        }
    }
    nozzle_diameter * WIDTH_TO_NOZZLE_RATIO
}

/// Default wipe tower depth used for wrapping-detection / timelapse layers.
// WipeTower.cpp:1559 — const double wrapping_wipe_tower_depth = 10;
const WRAPPING_WIPE_TOWER_DEPTH: f32 = 10.0;

/// Default flat iron area
const FLAT_IRON_AREA: f32 = 4.0;

/// Default flat iron speed (mm/min)
const FLAT_IRON_SPEED: f32 = 10.0 * 60.0;

/// Minimum depth per height mapping for tower stability
// WipeTower.cpp:1562-1564 — WipeTower::min_depth_per_height (std::map ordered by
// key): {{5,5},{100,20},{250,40},{350,60}}.
const MIN_DEPTH_PER_HEIGHT: &[(f32, f32)] = &[(5.0, 5.0), (100.0, 20.0), (250.0, 40.0), (350.0, 60.0)];

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
    /// Depth of this layer.
    ///
    /// Plays the role of C++'s `m_layer_info->depth`, which
    /// `update_all_layer_depth` forces to the FULL tower depth on every layer
    /// when timelapse is on. `finish_layer_new`'s fallback fill box uses this.
    pub depth: f32,
    /// The depth ALLOCATED to this layer by its tool changes — C++'s
    /// `block.layer_depths[m_cur_layer_id]`. `finish_block`'s fill box and its
    /// block-full skip use this, NOT `depth`. R510: these were one field, so any
    /// value fixed one caller and broke the other.
    pub alloc_depth: f32,
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
            alloc_depth: 0.0,
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
        _filament_parameters: &[FilamentParameters],
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
    // WipeTower.cpp:665-672 — set width/depth/internal_angle, then
    // m_start_pos = rotate(pos); m_current_pos = pos (UNrotated). Here the
    // reduced port passes (pos, internal_angle, y_shift); width/depth are set
    // separately via set_wipe_tower_dimensions.
    pub fn set_initial_position(&mut self, pos: Vec2f, internal_angle: f32, y_shift: f32) {
        self.internal_angle = internal_angle;
        self.y_shift = y_shift;
        self.start_pos = self.rotate(pos);
        self.current_pos = pos;
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
    // WipeTower.cpp:682-686 — m_current_pos.y() -= shift - m_y_shift; then
    // m_y_shift = shift.
    pub fn set_y_shift(&mut self, y_shift: f32) {
        self.current_pos.y -= y_shift - self.y_shift;
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
    // WipeTower.cpp:688-697 — Klipper: SET_PRESSURE_ADVANCE ADVANCE=0;
    // RepRapFirmware: "M572 D<tool> S0"; otherwise (Marlin/etc.): "M900 K0".
    pub fn disable_linear_advance(&mut self) {
        match self.gcode_flavor {
            GCodeFlavor::Klipper => {
                self.gcode.push_str("SET_PRESSURE_ADVANCE ADVANCE=0\n");
            }
            GCodeFlavor::RepRap => {
                self.gcode
                    .push_str(&format!("M572 D{} S0\n", self.current_tool));
            }
            _ => {
                self.gcode.push_str("M900 K0\n");
            }
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
    // WipeTower.cpp:710-717 — emit "G1 F<round(f)>" when the feedrate actually
    // changes. set_format_F (WipeTower.cpp:1480-1485) prints int(floor(f+0.5)).
    pub fn feedrate(&mut self, f: f32) -> &mut Self {
        if f != self.current_feedrate {
            let fi = (f + 0.5).floor() as i64;
            self.gcode.push_str(&format!("G1 F{}\n", fi));
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
    // WipeTower.cpp:724 — m_start_pos is already stored rotated.
    pub fn start_pos_rotated(&self) -> Vec2f {
        self.start_pos
    }

    /// Get current position (rotated)
    // WipeTower.cpp:725 — rotate(m_current_pos).
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
        // WipeTower.cpp:766-771 — gcode uses rot.x()/rot.y() directly; the
        // y_shift is already folded into rotate() (WipeTower.cpp:1495), so it
        // must NOT be added again here.
        let rotated = self.rotate(target);
        // WipeTower.cpp:766 emits travels as `G1` (e==0), NOT `G0` — the export
        // integration's `transform_gcode` only rewrites `G1 ` moves into bed
        // coordinates, so a `G0` travel would leak the tower-local position.
        self.gcode
            .push_str(&format!("G1 X{:.3} Y{:.3}\n", rotated.x, rotated.y));

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
        _limit_flow: bool,
    ) -> &mut Self {
        let target = Vec2f::new(x, y);
        let rotated = self.rotate(target);

        let dx = x - self.current_pos.x;
        let dy = y - self.current_pos.y;
        let len = (dx * dx + dy * dy).sqrt();

        // WipeTower.cpp:766-771 — y_shift is already folded into rotate(), so do
        // not add it again to the emitted Y coordinate. set_format_X/Y print 3
        // decimals, set_format_E prints 4 (WipeTower.cpp:1461-1478).
        self.gcode
            .push_str(&format!("G1 X{:.3} Y{:.3} E{:.4}\n", rotated.x, rotated.y, e));

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
    // WipeTower.cpp:1000-1006 — rectangle(box) -> rectangle(box.ld, w, h, f)
    // with w = ru.x - lu.x, h = ru.y - rd.y.
    // WipeTower.cpp:902-925 — the rectangle(ld, width, height) implementation.
    pub fn rectangle(&mut self, box_coords: &BoxCoordinates) -> &mut Self {
        let ld = box_coords.ld;
        let width = box_coords.ru.x - box_coords.lu.x;
        let height = box_coords.ru.y - box_coords.rd.y;

        // WipeTower.cpp:904-908 — corners are ld, ld+(w,0), ld+(w,h), ld+(0,h).
        let corners = [
            ld,
            ld + Vec2f::new(width, 0.0),
            ld + Vec2f::new(width, height),
            ld + Vec2f::new(0.0, height),
        ];

        // WipeTower.cpp:909-913 — choose the closest corner via axis comparisons.
        let mut index_of_closest = 0usize;
        if self.x() - ld.x > ld.x + width - self.x() {
            // closer to the right
            index_of_closest = 1;
        }
        if self.y() - ld.y > ld.y + height - self.y() {
            // closer to the top
            index_of_closest = if index_of_closest == 0 { 3 } else { 2 };
        }

        // WipeTower.cpp:915-916 — travel to the closest corner (axis-aligned).
        self.travel(corners[index_of_closest].x, self.y());
        self.travel(self.x(), corners[index_of_closest].y);

        // WipeTower.cpp:918-923 — extrude around the rectangle.
        let mut i = index_of_closest;
        loop {
            i += 1;
            if i == 4 {
                i = 0;
            }
            self.extrude(corners[i].x, corners[i].y);
            if i == index_of_closest {
                break;
            }
        }

        self
    }

    /// Fill a box with back-and-forth extrusion
    pub fn rectangle_fill_box(&mut self, box_coords: &BoxCoordinates, spacing: f32) -> &mut Self {
        let _width = box_coords.width();
        let height = box_coords.height();
        let num_lines = (height / spacing).floor() as i32;

        if num_lines < 1 {
            return self;
        }

        let actual_spacing = height / num_lines as f32;
        let mut y = box_coords.ld.y + actual_spacing / 2.0;
        let mut left_to_right = true;
        // FillBase-style zig-zag. C++'s equivalent solid branch
        // (WipeTower.cpp:3619-3623) writes
        //     writer.extrude(writer.x(), y, feedrate).extrude(i % 2 ? left : right, y);
        // so the step in Y is EXTRUDED. Gated with the purge connectors (R498).
        let connector = crate::faithful_gate("TOWER_WIPE_CONNECTOR");
        let mut first = true;

        for _ in 0..num_lines {
            let (start_x, end_x) = if left_to_right {
                (box_coords.ld.x, box_coords.rd.x)
            } else {
                (box_coords.rd.x, box_coords.ld.x)
            };

            if connector {
                if first {
                    self.travel(start_x, y);
                    first = false;
                } else {
                    let x = self.x();
                    self.extrude(x, y);
                }
            } else {
                self.travel(start_x, y);
            }
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
    // WipeTower.cpp:1060-1071 — emit "G1" plus E (when e != 0) plus F (when
    // f != 0 and f != current feedrate). Early-out for a no-op.
    pub fn load(&mut self, e: f32, f: f32) -> &mut Self {
        if e == 0.0 && (f == 0.0 || f == self.current_feedrate) {
            return self;
        }
        self.gcode.push_str("G1");
        if e != 0.0 {
            // WipeTower.cpp:1476-1478 — set_format_E prints 4 decimals.
            self.gcode.push_str(&format!(" E{:.4}", e));
        }
        if f != 0.0 && f != self.current_feedrate {
            self.gcode.push_str(&format!(" F{:.0}", f));
        }
        self.gcode.push('\n');
        self
    }

    /// Retract filament
    // WipeTower.cpp:1073-1074 — retract(e) == load(-e).
    pub fn retract(&mut self, e: f32, f: f32) -> &mut Self {
        self.load(-e, f)
    }

    /// Z hop
    // WipeTower.cpp:1094-1101 — "G1 Z<z+hop>" plus F (only when f != 0 and
    // f != current feedrate). No elapsed-time bookkeeping in C++.
    pub fn z_hop(&mut self, hop: f32, f: f32) -> &mut Self {
        let z_str = super::writer::format_gcode_value((self.current_z + hop) as f64, 3);
        self.gcode.push_str(&format!("G1 Z{}", z_str));
        if f != 0.0 && f != self.current_feedrate {
            self.gcode.push_str(&format!(" F{:.0}", f));
        }
        self.gcode.push('\n');
        self
    }

    /// Reset Z hop
    // WipeTower.cpp:1104-1105 — z_hop_reset(f) == z_hop(0, f).
    pub fn z_hop_reset(&mut self, f: f32) -> &mut Self {
        self.z_hop(0.0, f)
    }

    /// Set tool
    // WipeTower.cpp:1126-1130 — this only updates the writer's notion of the
    // current tool and outputs nothing; the actual "Tn" command is inserted by
    // the caller / post-processor.
    pub fn set_tool(&mut self, tool: usize) -> &mut Self {
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
    // WipeTower.cpp:1140-1146 — "G4 S<time>" with 3 decimal places; early-out
    // when time == 0. (No elapsed-time bookkeeping in C++.)
    pub fn wait(&mut self, seconds: f32) -> &mut Self {
        if seconds == 0.0 {
            return self;
        }
        let s = super::writer::format_gcode_value(seconds as f64, 3);
        self.gcode.push_str(&format!("G4 S{}\n", s));
        self
    }

    /// Speed override
    pub fn speed_override(&mut self, percent: i32) -> &mut Self {
        self.gcode.push_str(&format!("M220 S{}\n", percent));
        self
    }

    /// Set fan speed
    // WipeTower.cpp:1207-1217 — `speed` is a percentage; M106 takes a PWM value
    // (255 * speed / 100). M107 turns the fan off.
    pub fn set_fan(&mut self, speed: u32) -> &mut Self {
        if speed != self.last_fan_speed {
            if speed == 0 {
                self.gcode.push_str("M107\n");
            } else {
                let pwm = (255.0 * speed as f64 / 100.0) as u32;
                self.gcode.push_str(&format!("M106 S{}\n", pwm));
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
    // WipeTower.cpp:1368-1374 — pick first-layer vs normal accel list; bail out
    // if empty; index by the extruder for the current tool (here current_tool,
    // since this reduced port has no multi-nozzle group result), then emit.
    pub fn set_normal_acceleration(&mut self) -> &mut Self {
        let accels = if self.is_first_layer {
            &self.first_layer_normal_accelerations
        } else {
            &self.normal_accelerations
        };
        if accels.is_empty() {
            return self;
        }
        let acc = accels[self.current_tool.min(accels.len() - 1)];
        self.set_acceleration_impl(acc);
        self
    }

    /// Set travel acceleration
    // WipeTower.cpp:1376-1384
    pub fn set_travel_acceleration(&mut self) -> &mut Self {
        let accels = if self.is_first_layer {
            &self.first_layer_travel_accelerations
        } else {
            &self.travel_accelerations
        };
        if accels.is_empty() {
            return self;
        }
        let acc = accels[self.current_tool.min(accels.len() - 1)];
        self.set_acceleration_impl(acc);
        self
    }

    /// Emit an acceleration command, flavor dependent.
    // WipeTower.cpp:1385-1420
    fn set_acceleration_impl(&mut self, acceleration: u32) {
        // WipeTower.cpp:1387-1388 — clamp to max only when a max is set (>0).
        let mut acceleration = acceleration;
        if self.max_acceleration > 0 && acceleration > self.max_acceleration {
            acceleration = self.max_acceleration;
        }

        // WipeTower.cpp:1390-1391 — nothing to emit.
        if acceleration == 0 || acceleration == self.last_acceleration {
            return;
        }

        // WipeTower.cpp:1393
        self.last_acceleration = acceleration;

        // WipeTower.cpp:1396-1418 — flavor-dependent gcode. This reduced port's
        // GCodeFlavor enum lacks Repetier/RepRapFirmware/MarlinFirmware
        // distinctions; map RepRap -> "M204 P", Klipper -> SET_VELOCITY_LIMIT
        // (when accel_to_decel), everything else -> "M204 S".
        match self.gcode_flavor {
            GCodeFlavor::RepRap => {
                self.gcode.push_str(&format!("M204 P{}\n", acceleration));
            }
            GCodeFlavor::Klipper if self.accel_to_decel_enable => {
                let a2d = (acceleration as f32 * self.accel_to_decel_factor / 100.0) as i64;
                self.gcode
                    .push_str(&format!("SET_VELOCITY_LIMIT ACCEL_TO_DECEL={}\n", a2d));
                self.gcode.push_str(&format!("M204 S{}\n", acceleration));
            }
            _ => {
                self.gcode.push_str(&format!("M204 S{}\n", acceleration));
            }
        }
    }

    /// Rotate a point by the internal angle
    // WipeTower.cpp:1492-1500 — translate to tower center (applying m_y_shift),
    // rotate by m_internal_angle, then translate back. The angle/cos/sin are
    // computed unconditionally in C++ (no early-out for ~0 angle).
    fn rotate(&self, pt: Vec2f) -> Vec2f {
        let x = pt.x - self.wipe_tower_width / 2.0;
        let y = pt.y + self.y_shift - self.wipe_tower_depth / 2.0;
        let angle = self.internal_angle * (PI / 180.0);
        let c = angle.cos() as f64;
        let s = angle.sin() as f64;
        let px = x as f64;
        let py = y as f64;
        Vec2f::new(
            (px * c - py * s) as f32 + self.wipe_tower_width / 2.0,
            (px * s + py * c) as f32 + self.wipe_tower_depth / 2.0,
        )
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
        // WipeTower.hpp:499-501 — these are member initializers (defaults), the C++
        // constructor (WipeTower.cpp:1725) does NOT recompute them from the tower
        // width. m_perimeter_width = 0.4f * Width_To_Nozzle_Ratio, same for the
        // nozzle-change width, and m_extrusion_flow = 0.038f.
        let perimeter_width = 0.4 * WIDTH_TO_NOZZLE_RATIO;
        let nozzle_change_perimeter_width = 0.4 * WIDTH_TO_NOZZLE_RATIO;
        let extrusion_flow = 0.038;

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
    /// WipeTower.cpp:1807 — WipeTower::set_extruder(idx, config).
    pub fn set_extruder(&mut self, idx: usize, params: FilamentParameters) {
        if idx >= self.filament_params.len() {
            self.filament_params
                .resize(idx + 1, FilamentParameters::default());
        }
        self.filament_params[idx] = params;

        // WipeTower.cpp:1900-1901 — both widths are recomputed here from the
        // extruder's nozzle diameter; the values set in the constructor are only
        // the member-initialiser defaults (WipeTower.hpp:499-501).
        //   m_perimeter_width               = nozzle_diameter * Width_To_Nozzle_Ratio;
        //   m_nozzle_change_perimeter_width = nozzle_diameter_to_nozzle_change_width.at(nozzle_diameter);
        // R475: we never did this, so the nozzle-change block used the PERIMETER
        // width (0.5 for a 0.4 nozzle) where C++ uses 1.0 — half the line width, so
        // twice as many lines to cover the same depth, each at roughly half the flow.
        if crate::faithful_gate("WT_NOZZLE_CHANGE_WIDTH") {
            let nd = self.filament_params[idx].nozzle_diameter;
            if std::env::var_os("WT_WIDTH_DEBUG").is_some() {
                eprintln!("WT_WIDTH: set_extruder idx={idx} nd={nd} -> ncw={}", nozzle_change_width_for_nozzle(nd));
            }
            self.perimeter_width = nd * WIDTH_TO_NOZZLE_RATIO;
            self.nozzle_change_perimeter_width = nozzle_change_width_for_nozzle(nd);
        }
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
    // WipeTower.hpp:222 — set_layer(print_z, layer_height, max_tool_changes,
    // is_first_layer, is_last_layer). NOTE: in C++ `max_tool_changes` and
    // `is_last_layer` are accepted but NOT used in the body; we keep them in the
    // signature for call-site compatibility.
    pub fn set_layer(
        &mut self,
        print_z: f32,
        layer_height: f32,
        _max_tool_changes: usize,
        is_first_layer: bool,
        _is_last_layer: bool,
    ) {
        // WipeTower.hpp:234-237
        self.z_pos = print_z;
        self.layer_height = layer_height;
        self.depth_traversed = 0.0;
        self.current_layer_finished = false;
        // WipeTower.hpp:238-239 — m_current_shape = SHAPE_NORMAL (the reversed
        // alternative is commented out in C++).
        self.current_shape = WipeShape::Normal;
        // WipeTower.hpp:240-244
        if is_first_layer {
            self.num_layer_changes = 0;
            self.num_tool_changes = 0;
        } else {
            self.num_layer_changes += 1;
        }

        // WipeTower.hpp:247 — m_extrusion_flow = extrusion_flow(layer_height);
        // extrusion_flow(lh) = lh * (m_perimeter_width - lh*(1 - PI/4)) / filament_area
        // (WipeTower.hpp:285). filament_area = PI * 0.875^2 (filament dia 1.75).
        let filament_area = PI * 0.875 * 0.875;
        self.extrusion_flow =
            layer_height * (self.perimeter_width - layer_height * (1.0 - PI / 4.0)) / filament_area;
        // R463 (WT_WIDTH_DEBUG=1): C++ emits a CONSTANT `; LINE_WIDTH: 0.500000` for the
        // tower and an E-per-mm of 0.05433, which this formula reproduces exactly at
        // h=0.3 / perimeter_width=0.5. Our measured tower E-per-mm is 0.04850. Print what
        // the fields actually hold at the moment the flow is computed.
        if std::env::var_os("WT_WIDTH_DEBUG").is_some() {
            eprintln!(
                "WT_WIDTH: set_layer z={:.3} layer_height={:.4} perimeter_width={:.6}                  nozzle_change_perimeter_width={:.6} => extrusion_flow={:.6}",
                print_z, layer_height, self.perimeter_width,
                self.nozzle_change_perimeter_width, self.extrusion_flow
            );
        }

        // WipeTower.hpp:249-250 — advance the layer-info iterator to the plan
        // entry whose z is at (or just below) print_z. Equivalent index search.
        self.layer_idx = self
            .plan
            .iter()
            .position(|l| (l.z - print_z).abs() < WT_EPSILON)
            .unwrap_or(self.layer_idx);
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
    // WipeTower.hpp:534-536 — std::max(0.f, volume / area)
    fn volume_to_length(&self, volume: f32, line_width: f32, layer_height: f32) -> f32 {
        0.0_f32.max(volume / (layer_height * (line_width - layer_height * (1.0 - PI / 4.0))))
    }

    /// Convert extrusion length to volume
    // WipeTower.hpp:538-541 — std::max(0.f, length * area)
    fn length_to_volume(&self, length: f32, line_width: f32, layer_height: f32) -> f32 {
        0.0_f32.max(length * (layer_height * (line_width - layer_height * (1.0 - PI / 4.0))))
    }

    /// Extrusion flow for nozzle change
    // WipeTower.hpp:287-292 — negative layer_height returns the current
    // m_extrusion_flow, otherwise compute from the nozzle-change perimeter width.
    fn nozzle_change_extrusion_flow(&self, layer_height: f32) -> f32 {
        if layer_height < 0.0 {
            return self.extrusion_flow;
        }
        let filament_area = PI * 0.875 * 0.875;
        layer_height * (self.nozzle_change_perimeter_width - layer_height * (1.0 - PI / 4.0))
            / filament_area
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
    // WipeTower.cpp:1566-1600
    pub fn get_limit_depth_by_height(max_height: f32) -> f32 {
        // WipeTower.cpp:1568
        let mut min_wipe_tower_depth = 0.0_f32;
        let table = MIN_DEPTH_PER_HEIGHT;
        // WipeTower.cpp:1569-1570 — iterate over the (ordered) map.
        let mut i = 0usize;
        while i < table.len() {
            let curr = table[i];

            // WipeTower.cpp:1574-1577 — height lower than first member.
            if curr.0 >= max_height {
                min_wipe_tower_depth = curr.1;
                break;
            }

            // WipeTower.cpp:1579 — ++iter
            i += 1;

            // WipeTower.cpp:1582-1585 — curr was the last member.
            if i == table.len() {
                min_wipe_tower_depth = curr.1;
                break;
            }

            // WipeTower.cpp:1588-1597 — between current and next: linear interp.
            let next = table[i];
            if next.0 > max_height {
                let height_diff = next.0 - curr.0;
                let depth_diff = next.1 - curr.1;
                min_wipe_tower_depth = curr.1 + (max_height - curr.0) / height_diff * depth_diff;
                break;
            }
        }
        min_wipe_tower_depth
    }

    /// Get auto brim width by height
    // WipeTower.cpp:1602-1605
    pub fn get_auto_brim_by_height(max_height: f32) -> f32 {
        if max_height < 100.0 {
            return max_height / 100.0 * 8.0;
        }
        8.0
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
        // WipeTower.cpp:2874 — refuses to add a layer below the last one.
        assert!(self.plan.is_empty() || self.plan.last().unwrap().z <= z + WT_EPSILON);

        // WipeTower.cpp:2876-2877 — if we moved to a new layer, add it first.
        if self.plan.is_empty() || self.plan.last().unwrap().z + WT_EPSILON < z {
            self.plan.push(WipeTowerLayerInfo::new(z, layer_height));
        }

        // WipeTower.cpp:2879-2880 — record first layer with actual tool changes.
        if self.first_layer_idx.is_none() && (!self.config.no_sparse_layers || old_tool != new_tool)
        {
            self.first_layer_idx = Some(self.plan.len() - 1);
        }

        // WipeTower.cpp:2882-2883 — new layer without toolchanges, done.
        if old_tool == new_tool {
            return;
        }

        // WipeTower.cpp:2886-2887
        let mut depth = 0.0_f32;
        let width = self.config.width - 2.0 * self.perimeter_width;

        // WipeTower.cpp:2890-2891 — if the wipe tower width is too small, the
        // depth would be infinity. (C++ compares against EPSILON == 1e-4.)
        if width <= WT_EPSILON {
            return;
        }

        // WipeTower.cpp:2892-2893 — layer_id is the last plan index. In this
        // reduced port is_same_extruder/is_same_nozzle ignore layer_id.
        let wipe_volume = if self.is_same_extruder(old_tool, new_tool)
            && !self.is_same_nozzle(old_tool, new_tool)
        {
            wipe_volume_nc
        } else {
            wipe_volume_ec
        };

        // WipeTower.cpp:2911-2913
        let length_to_extrude =
            self.volume_to_length(wipe_volume, self.perimeter_width, layer_height);
        depth += (length_to_extrude / width).ceil() * self.perimeter_width;

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

        // WipeTower.cpp:2926-2929 — assemble the ToolChange and append it.
        // (C++ does NOT touch m_num_tool_changes here; that counter is reset in
        // set_layer and incremented during the actual tool_change pass.)
        self.plan.last_mut().unwrap().tool_changes.push(tool_change);
    }

    /// Plan the entire tower
    // WipeTower.cpp:2933-3032
    pub fn plan_tower(&mut self) {
        // WipeTower.cpp:2937-2939 — calculate extra spacing.
        let mut max_depth = 0.0f32;
        for info in &self.plan {
            max_depth = max_depth.max(info.toolchanges_depth());
        }

        // WipeTower.cpp:2941
        let min_wipe_tower_depth = Self::get_limit_depth_by_height(self.config.height);

        // WipeTower.cpp:2944-2945
        if self.config.enable_wrapping_detection && max_depth < WT_EPSILON {
            max_depth = WRAPPING_WIPE_TOWER_DEPTH;
        }

        // WipeTower.cpp:2947-2948
        if self.config.enable_timelapse_print && max_depth < WT_EPSILON {
            max_depth = min_wipe_tower_depth;
        }

        if max_depth + WT_EPSILON < min_wipe_tower_depth && !self.has_tpu_filament {
            self.extra_spacing = min_wipe_tower_depth / max_depth;
        } else {
            self.extra_spacing = 1.0;
        }
        if std::env::var_os("WT_WIDTH_DEBUG").is_some() {
            eprintln!(
                "WT_PLAN: max_depth(toolchanges)={:.3} min_wipe_tower_depth={:.3} height={:.3} has_tpu={} -> extra_spacing={:.4}",
                max_depth, min_wipe_tower_depth, self.config.height, self.has_tpu_filament, self.extra_spacing
            );
            let ncd: f32 = self.plan.iter().flat_map(|l| l.tool_changes.iter()).map(|t| t.nozzle_change_depth).sum();
            let rd: f32 = self.plan.iter().flat_map(|l| l.tool_changes.iter()).map(|t| t.required_depth).sum();
            let n = self.plan.iter().map(|l| l.tool_changes.len()).sum::<usize>();
            eprintln!("WT_PLAN: tool_changes={n} sum(nozzle_change_depth)={ncd:.2} sum(required_depth)={rd:.2} ncw={:.3}", self.nozzle_change_perimeter_width);
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
                // WipeTower.cpp:2980-2984
                info.extra_spacing = extra_spacing;
                for tc in &mut info.tool_changes {
                    tc.required_depth *= extra_spacing;
                    let area =
                        info.height * (perimeter_width - info.height * (1.0 - PI / 4.0));
                    // volume_to_length: std::max(0, wipe_volume / area)
                    tc.wipe_length = 0.0_f32.max(tc.wipe_volume / area);
                }
            }
        }

        // WipeTower.cpp:2989-2992 — reset depths.
        self.depth = 0.0;
        for layer in self.plan.iter_mut() {
            layer.depth = 0.0;
        }

        // WipeTower.cpp:2994-3017 — back-to-front depth propagation.
        let mut max_depth_for_all = 0.0_f32;
        let plan_len = self.plan.len();
        for layer_index in (0..plan_len).rev() {
            let mut this_layer_depth =
                self.plan[layer_index].depth.max(self.plan[layer_index].toolchanges_depth());
            // WipeTower.cpp:2998-2999
            if self.config.enable_wrapping_detection
                && (layer_index as i32) < self.config.wrapping_detection_layers
                && this_layer_depth < WT_EPSILON
            {
                this_layer_depth = WRAPPING_WIPE_TOWER_DEPTH;
            }
            // WipeTower.cpp:3001-3002
            if self.config.enable_timelapse_print && this_layer_depth < WT_EPSILON {
                this_layer_depth = min_wipe_tower_depth;
            }

            self.plan[layer_index].depth = this_layer_depth;

            // WipeTower.cpp:3006-3007
            if this_layer_depth > self.depth - self.perimeter_width {
                self.depth = this_layer_depth + self.perimeter_width;
            }

            // WipeTower.cpp:3009-3013 — propagate downwards.
            for i in (0..layer_index).rev() {
                if self.plan[i].depth - this_layer_depth < 2.0 * self.perimeter_width {
                    self.plan[i].depth = this_layer_depth;
                }
            }

            // WipeTower.cpp:3015-3016
            if self.config.enable_timelapse_print && layer_index == 0 {
                max_depth_for_all = self.plan[0].depth;
            }
        }

        // WipeTower.cpp:3019-3025
        if self.config.enable_wrapping_detection {
            for i in (0..self.config.wrapping_detection_layers as usize).rev() {
                if i < plan_len
                    && plan_len <= self.config.wrapping_detection_layers as usize
                    && self.plan[i].depth < WRAPPING_WIPE_TOWER_DEPTH
                {
                    self.plan[i].depth = WRAPPING_WIPE_TOWER_DEPTH;
                }
            }
        }

        // R510: freeze the per-layer ALLOCATION before the timelapse override
        // below rewrites `depth`. C++ keeps these as two separate values —
        // `block.layer_depths[cur]` (the allocation) and `m_layer_info->depth`
        // (which timelapse forces to the full tower depth).
        for i in 0..plan_len {
            self.plan[i].alloc_depth = self.plan[i].depth;
        }

        // WipeTower.cpp:3027-3031
        if self.config.enable_timelapse_print {
            for i in (0..plan_len).rev() {
                self.plan[i].depth = max_depth_for_all;
            }
        }

    }

    /// Generate the wipe tower — port of `WipeTower::generate` (WipeTower.cpp:4774).
    ///
    /// FIDELITY-NOTE(R479): **BambuStudio does not call this function.**
    /// `Print::_make_wipe_tower` (Print.cpp:3157) calls `wipe_tower.generate_new(...)`,
    /// the newer block-based tower. `generate()` survives in the C++ tree but is dead
    /// for this pipeline, and we ported the dead one. The block-based path is:
    ///   WipeTower.cpp:4208  generate_wipe_tower_blocks()
    ///   WipeTower.cpp:4417  plan_tower_new()
    ///   WipeTower.cpp:4564  generate_new()
    ///   WipeTower.cpp:3487  finish_layer_new(extrude_perimeter, extrude_fill, extrude_fill_wall)
    ///   WipeTower.cpp:3597  generate_support_wall_new(.., m_use_rib_wall, extrude_perimeter, m_use_gap_wall)
    /// driven by `prime_tower_rib_wall` / `prime_tower_skip_points` /
    /// `prime_tower_enable_framework` / `prime_tower_infill_gap` (0 / 1 / 0 / 100% on
    /// the Majora plate).
    ///
    /// Measured consequences on Majora, all consistent with running the wrong
    /// generator (see the R479 commit for the full numbers):
    ///  - C++ emits TWO separate `; FEATURE: Prime tower` blocks per layer in the
    ///    sparse top band -- an outer rectangle and one inset by a perimeter width,
    ///    each with its own travel/wipe/Z-hop. We emit one, so z178-197 comes out at
    ///    E-rat 0.558 (rust 582.3 vs bambu 1042.9 over the same 63 layers).
    ///  - The bulk band z0-178 goes the other way at 1.053, netting the +4.5% overall.
    ///  - C++ writes 3,655 `; WIPE_TOWER_START` / `WIPE_TOWER_END` markers (consumed
    ///    by GCodeProcessor); this path emits none.
    /// The tower's flow, footprint, extra_spacing and tool-change count are already
    /// exact (R475-R478), so porting the block-based generator is what remains.
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

        // WipeTower.cpp:3271 (tool_change_new):
        //   box_coordinates cleaning_box(Vec2f(m_perimeter_width, block->cur_depth),
        //                                m_wipe_tower_width - 2 * m_perimeter_width,
        //                                wipe_depth - nozzle_change_depth);
        // The height is the toolchange's allocated depth MINUS the nozzle-change
        // depth (zero for Majora — nozzle_change emits nothing, R502), not minus
        // the perimeter width. Ours was 0.5 mm short: box 5.000 against C++'s
        // 5.500 while both allocate 5.5 per toolchange. At dy = 0.5 that is
        // exactly one purge stroke per toolchange, and 34.5 mm x 2,723 toolchanges
        // = 94,000 mm — the entire 94,632 mm purge deficit (R506).
        let box_height = if crate::faithful_gate("TOWER_CLEANING_BOX") {
            wipe_depth - nozzle_change_depth
        } else {
            wipe_depth - self.perimeter_width
        };
        let cleaning_box = BoxCoordinates::new(
            self.perimeter_width,
            self.depth_traversed + self.perimeter_width,
            self.config.width - 2.0 * self.perimeter_width,
            box_height,
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

        // WipeTower.cpp:3270-3272 (tool_change_new) — the GCodeProcessor reserved
        // block markers. R530: these are real gcode CONTENT that C++ emits and we
        // did not; GCodeProcessor consumes them to segment the preview. Counts are
        // exact against C++ because our tool-change count already matches (2,723).
        writer.append(";--------------------\n; CP TOOLCHANGE START\n");

        // Comment for tool change
        writer.comment(&format!("Tool change from T{} to T{}", old_tool, new_tool));

        // WipeTower.cpp:3288 — `;` + reserved_tag(Wipe_Tower_Start). The tag string
        // itself carries a leading space (GCodeProcessor.cpp:63), so the emitted
        // line is exactly `; WIPE_TOWER_START`.
        writer.append("; WIPE_TOWER_START\n");

        // R466 — WipeTower.cpp:2288 `toolchange_Unload`: "BBS: toolchange unload is
        // done in change_filament_gcode", and its whole body is `#if 0`. C++ emits NO
        // retract here; the template's own `G1 E-[old_retract_length_toolchange]`
        // does it. Emitting one anyway added a 0.8mm retract C++ never has and, paired
        // with the load below, left the template's 2.0mm toolchange retraction only
        // 40% repaid entering the tower (R465).
        if crate::faithful_gate("WT_TOOLCHANGE_RETRACT_LEGACY")
            && std::env::var_os("WT_TOOLCHANGE_RETRACT_LEGACY").is_some()
        {
            if old_tool < self.filament_params.len() {
                let retract = self.filament_params[old_tool].retract_length;
                let retract_speed = self.filament_params[old_tool].retract_speed * 60.0;
                if retract > 0.0 {
                    writer.retract(retract, retract_speed);
                }
            }
        }

        // Perform ramming/wiping if needed
        if self.is_need_ramming(old_tool, new_tool) {
            self.toolchange_unload(&mut writer, &cleaning_box);
        }

        // Actual tool change. Faithful to WipeTower.cpp:2466: emit the
        // `[change_filament_gcode]` PLACEHOLDER, which the export step
        // (`wipe_tower_integration`/`emit_tower_tcr`) substitutes with the
        // evaluated `change_filament_gcode` template — or with a bare `Tn` when
        // that template is empty (GCode.cpp:754). Then update the writer's tool
        // state via the (output-less) set_tool.
        writer.append("[change_filament_gcode]\n");
        writer.set_tool(new_tool);
        self.current_tool = new_tool;

        // R466 — WipeTower.cpp:2460 `toolchange_Load`: "BBS: tool load is done in
        // change_filament_gcode", body `#if 0`. C++ emits NO load here either; the
        // unretract that repays the template's toolchange retraction is emitted by
        // GCode::append_tcr (see `emit_tower_tcr`), and it is
        // `retract_length_toolchange` (2.0), not `retract_length` (0.8).
        if crate::faithful_gate("WT_TOOLCHANGE_LOAD_LEGACY")
            && std::env::var_os("WT_TOOLCHANGE_LOAD_LEGACY").is_some()
        {
            if new_tool < self.filament_params.len() {
                let load = self.filament_params[new_tool].retract_length;
                let load_speed = self.filament_params[new_tool].retract_speed * 60.0;
                if load > 0.0 {
                    writer.load(load, load_speed);
                }
            }
        }

        // Wipe
        self.toolchange_wipe(&mut writer, &cleaning_box, wipe_length);

        // WipeTower.cpp:3328 — closes the Wipe_Tower_Start block opened above.
        writer.append("; WIPE_TOWER_END\n");

        // WipeTower.cpp:3341-3343 — closes the CP TOOLCHANGE block.
        writer.append("; CP TOOLCHANGE END\n;------------------\n\n\n");

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
        // WipeTower.cpp:3960-3961 (toolchange_wipe_new) — the reserved tag carries
        // TWO suffixes, not one:
        //   ";" + reserved_tag(CP_TOOLCHANGE_WIPE)
        //       + " CT" + to_string(solid_tool_toolchange)
        //       + " FL" + to_string(is_first_layer())
        // `std::to_string(bool)` renders "0"/"1", so the real emitted lines are
        // `; CP_TOOLCHANGE_WIPE CT0 FL0` (2,720 in the C++ Majora output) and
        // `... CT0 FL1` (3, the first layer's tool changes).
        //
        // CT is hard-zero here because this port has no solid-toolchange path at
        // all: R506 measured `solid_tool_toolchange` as ZERO for Majora and our
        // `tool_change` takes no such parameter. If that branch is ever ported,
        // this tag must start reporting it.
        writer.append(&format!(
            "; CP_TOOLCHANGE_WIPE CT0 FL{}\n",
            if self.layer_idx == 0 { 1 } else { 0 }
        ));

        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;
        let line_len = xr - xl;

        if line_len <= 0.0 {
            return;
        }

        let num_lines = (wipe_length / line_len).ceil() as i32;
        let dy = self.perimeter_width;
        // WipeTower.cpp:1980-1997 — `get_next_pos` returns `cleaning_box.ld +
        // pos_offset` for the layer_id % 4 == 0 case, i.e. the purge starts AT
        // the bottom edge of the cleaning box, not half a line-pitch above it.
        // Starting at `ld.y + dy/2` costs exactly one stroke per toolchange:
        // C++ emits 11 strokes + 11 connectors = 379.5 mm per toolchange
        // (measured 379.0 over 2,723 toolchanges / 59,906 lines), we emitted
        // ~10 + ~10 = 344.0 mm. 34.5 mm x 2,723 = 94,000 mm, which is the whole
        // 94,632 mm purge deficit (R506).
        let mut y = if std::env::var_os("TOWER_PURGE_START").is_some() {
            cleaning_box.ld.y
        } else {
            cleaning_box.ld.y + dy / 2.0
        };
        if std::env::var_os("WTWL").is_some() {
            use std::sync::atomic::{AtomicUsize, Ordering};
            static N: AtomicUsize = AtomicUsize::new(0);
            static SUMW: AtomicUsize = AtomicUsize::new(0);
            static SUML: AtomicUsize = AtomicUsize::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed);
            SUMW.fetch_add((wipe_length * 1000.0) as usize, Ordering::Relaxed);
            SUML.fetch_add(num_lines.max(0) as usize, Ordering::Relaxed);
            if n < 4 || n % 900 == 0 {
                eprintln!(
                    "[WTWL] n={} wipe_length={:.3} line_len={:.3} num_lines={} box_h={:.3} ld_y={:.3} lu_y={:.3} avg_wl={:.3} avg_lines={:.3}",
                    n,
                    wipe_length,
                    line_len,
                    num_lines,
                    cleaning_box.lu.y - cleaning_box.ld.y,
                    cleaning_box.ld.y,
                    cleaning_box.lu.y,
                    SUMW.load(Ordering::Relaxed) as f32 / 1000.0 / (n + 1) as f32,
                    SUML.load(Ordering::Relaxed) as f32 / (n + 1) as f32
                );
            }
        }
        let wipe_speed = self.config.max_speed * 60.0 * 0.6; // 60% of max speed for wiping

        writer.feedrate(wipe_speed);

        // WipeTower.cpp:4145-4149 (toolchange_wipe_new) — the step to the next
        // purge line is EXTRUDED, not travelled:
        //     if (is_from_up) writer.extrude(writer.x(), writer.y() - dy);
        //     else            writer.extrude(writer.x(), writer.y() + dy);
        //     m_left_to_right = !m_left_to_right;
        // i.e. the purge is one continuous serpentine. We travelled the step, so
        // every one of those connectors was missing from our tower: R497's
        // histogram found C++ writes 27,273 segments of 0.5 mm (13,636 mm) where
        // we write ZERO segments at or below 1 mm.
        if crate::faithful_gate("TOWER_WIPE_CONNECTOR") {
            let mut left_to_right = true;
            let mut first = true;
            // WipeTower.cpp:4079-4116 — when `m_use_gap_wall` is set (which is
            // `prime_tower_skip_points`, plumbed in R499), the FIRST purge line of
            // every toolchange opens with an ironing pass: extrude a short run
            // (`ironing_length = 3.`, :4073), retract, travel back 1.5x and
            // forward again, un-retract, then extrude the REST of the way to the
            // far edge. That is R497's unexplained pair of segment classes —
            // 4,854 of 3.0 mm and 2,723 of 31.0 mm (34.0 - 3.0, one per
            // toolchange). The retract/un-retract pair is E-neutral.
            // NOT ported: `spiral_flat_ironing` (`m_flat_ironing`, :1768), which
            // needs `filament_tower_ironing_area`; the non-flat branch is taken
            // for this profile.
            let ironing_length = 3.0_f32;
            let iron = self.config.use_gap_wall;
            // C++: m_filpar[m_current_tool].retract_length / .retract_speed * 60
            let (retract_len, retract_spd) = self
                .filament_params
                .get(self.current_tool)
                .map_or((0.0, 0.0), |p| (p.retract_length, p.retract_speed * 60.0));
            for i in 0..num_lines {
                let (x_start, x_end) = if left_to_right { (xl, xr) } else { (xr, xl) };
                if first {
                    writer.travel(x_start, y);
                    first = false;
                }
                if i == 0 && iron {
                    let x0 = writer.x();
                    let dx = x_end - x0;
                    let il = if dx.abs() < ironing_length { dx.abs() } else { ironing_length };
                    let dir = if dx >= 0.0 { 1.0 } else { -1.0 };
                    writer.extrude(x0 + dir * il, y);
                    let x1 = writer.x();
                    writer.retract(retract_len, retract_spd);
                    writer.travel(x1 - dir * 1.5 * il, y);
                    writer.travel(x1, y);
                    writer.retract(-retract_len, retract_spd);
                }
                writer.extrude(x_end, y);
                y += dy;
                if y > cleaning_box.lu.y - dy / 2.0 {
                    break;
                }
                let x = writer.x();
                writer.extrude(x, y);
                left_to_right = !left_to_right;
            }
        } else {
            for i in 0..num_lines {
                let (x_start, x_end) = if i % 2 == 0 { (xl, xr) } else { (xr, xl) };

                writer.travel(x_start, y);
                writer.extrude(x_end, y);

                y += dy;
                if y > cleaning_box.lu.y - dy / 2.0 {
                    break;
                }
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

        // WipeTower.cpp:3550 (finish_layer_new) — unconditional, right after the
        // writer setup / set_for_wipe_tower_writer. R531: the second half of the
        // reserved-tag port (R530 did tool_change). `finish_layer` runs exactly
        // once per layer, so this adds 656 pairs on Majora.
        writer.append("; WIPE_TOWER_START\n");

        // Fill the remaining depth OF THIS LAYER. WipeTower.cpp:2697-2699:
        //   fill_box_y = m_layer_info->toolchanges_depth() + m_perimeter_width;
        //   fill_box(.., m_wipe_tower_width - 2*m_perimeter_width,
        //            m_layer_info->depth - fill_box_y)
        // R440: this used `self.depth` — the GLOBAL max tower depth over all
        // layers — so every layer filled the full rectangle instead of only the
        // depth reserved for it. That is why our per-layer sweep E was constant
        // (144.7) while C++'s varies (144→89→130), and why our tower swept 1.52×
        // C++'s path length at matching flow.
        // R510: two depths, matching C++'s two values.
        //   `depth`       == m_layer_info->depth        -> finish_layer_new's box
        //   `alloc_depth` == block.layer_depths[cur]    -> finish_block's box+skip
        let layer_depth = self
            .plan
            .get(self.layer_idx)
            .map(|l| l.depth)
            .unwrap_or(self.depth);
        let alloc_depth = self
            .plan
            .get(self.layer_idx)
            .map(|l| l.alloc_depth)
            .unwrap_or(self.depth);
        let fill_box_y = self
            .plan
            .get(self.layer_idx)
            .map(|l| l.toolchanges_depth())
            .unwrap_or(self.depth_traversed)
            + self.perimeter_width;
        // WipeTower.cpp:3570-3577 — with a single block (which is every layer of
        // Majora, probed R494) `multi_block_fill` is false and the fill box is
        // the WHOLE layer box:
        //     fill_box_depth = m_layer_info->depth - 2 * m_perimeter_width;
        //     fill_boxes.emplace_back(Vec2f(m_perimeter_width, m_perimeter_width),
        //                             m_wipe_tower_width - 2 * m_perimeter_width,
        //                             fill_box_depth);
        // NOT the leftover above the toolchanges, which is what the dead
        // `finish_layer` used and what we inherited. R493 applied the sparse
        // branch to the leftover box and undershot by 63,147 mm as a result.
        // R500: `TOWER_SPARSE_GRID` bundled four independent changes, so toggling
        // it moved the tower by their SUM and hid which one was mis-sized. It is
        // now the master switch, with three per-behaviour overrides that default
        // to it and can be forced on/off individually:
        //   TOWER_FILL_BOX  — C++'s fill box (whole layer box on no-toolchange
        //                     layers, finish_block's cur_depth box otherwise)
        //   TOWER_FILL_RECT — finish_block's always-on inner rectangle
        //   TOWER_FILL_GRID — the sparse-vs-solid grid branch
        // R506: the tower set now lands at 0.9947 (from 1.045) with every other
        // feature unchanged, so the master and all its per-behaviour knobs are
        // DEFAULT-ON. Set any of them to "0" to disable individually.
        let sparse_grid = crate::faithful_gate("TOWER_SPARSE_GRID");
        let knob = |name: &str| match std::env::var(name) {
            Ok(v) => v != "0",
            Err(_) => sparse_grid,
        };
        // R503: probing C++ shows `finish_layer_new` receives extrude_fill=FALSE on
        // 653 of its 656 calls — the dominant call site (WipeTower.cpp:4746) passes
        // a literal `false`:
        //     if (wall_idx != -1) {
        //         if (layer.tool_changes.empty())
        //             finish_layer_new(only_generate_wall ? false : true, false, false);
        // so on nearly every layer it draws ONLY the outer wall, no fill. C++'s
        // finish-layer fill therefore comes almost entirely from `finish_block`,
        // which runs on ~206 TOOL-CHANGE layers. We laid a fill on every layer,
        // which is the 4.50x fill excess (R502). R497 had the layer condition
        // exactly backwards — it suppressed the fill on tool-change layers.
        let fill_only_on_toolchange_layers = knob("TOWER_FILL_ONLY_TC");
        let fill_box_faithful = knob("TOWER_FILL_BOX");
        let inner_rect = knob("TOWER_FILL_RECT");
        let grid_faithful = knob("TOWER_FILL_GRID");
        // WipeTower.cpp:4703 — generate_new calls finish_layer_new ONLY when the
        // layer has no tool change (`wall_idx == -1`); layers WITH tool changes
        // are finished by finish_block (:3733), whose fill box runs from the
        // depth already consumed by the toolchanges up to the block's allocation
        // for this layer:
        //     box_coordinates(Vec2f(m_perimeter_width, block.cur_depth),
        //                     m_wipe_tower_width - 2*m_perimeter_width,
        //                     block.start_depth + block.layer_depths[id]
        //                         - block.cur_depth - m_perimeter_width)
        // With one block that is our existing leftover box. R496 measured that
        // serving both layer classes from one emitter is what splits the tower
        // error in two opposed halves.
        let layer_has_toolchange = self
            .plan
            .get(self.layer_idx)
            .map_or(false, |l| !l.tool_changes.is_empty());
        let fill_box = if fill_box_faithful && !layer_has_toolchange {
            BoxCoordinates::new(
                self.perimeter_width,
                self.perimeter_width,
                self.config.width - 2.0 * self.perimeter_width,
                layer_depth - 2.0 * self.perimeter_width,
            )
        } else {
            // finish_block's box (:3751) is measured against the block's
            // ALLOCATION for this layer, not the plan depth.
            BoxCoordinates::new(
                self.perimeter_width,
                fill_box_y,
                self.config.width - 2.0 * self.perimeter_width,
                alloc_depth - fill_box_y,
            )
        };

        // WipeTower.cpp:3585-3644 (finish_layer_new). The fill inside the tower
        // box is SOLID only when the NEXT layer contains a toolchange involving
        // a soluble filament, or on the first layer with adhesion:
        //   solid_infill = any_of((m_layer_info+1)->tool_changes, [](tch){
        //       return m_filpar[tch.new_tool].is_soluble || m_filpar[tch.old_tool].is_soluble; });
        //   solid_infill |= first_layer && m_adhesion;
        // Otherwise C++ writes the sparse "CP EMPTY GRID": an inverse-U up the
        // left edge plus vertical strokes spaced `m_bridging` (10 mm) apart —
        // a small fraction of a solid fill's material.
        //
        // We had no sparse branch at all, so every layer without a toolchange
        // got a full solid fill. Measured on Majora: +121,162 mm of excess
        // tower path across the 209 layers where C++ emits an empty grid, e.g.
        // at z23.10 we laid 76 full-width strokes at 0.5 pitch (3,004 mm) where
        // C++ laid 1,158 mm total (R493).
        //
        // FIDELITY-NOTE (R493): this branch alone is NOT sufficient and is
        // therefore OPT-IN (`TOWER_SPARSE_GRID=1`). The tower's per-layer path
        // length carries two opposed errors that partly cancel: +121,162 mm on
        // C++'s empty-grid layers and -68,580 mm on all the others (net +52,582,
        // the tower's 1.045). Enabling only the sparse branch takes the tower
        // from 1.045 to 0.887, because C++ in multi-block mode emits ONE grid
        // PER BLOCK (`fill_boxes` built from `block.layer_depths[m_cur_layer_id]`,
        // WipeTower.cpp:3562-3577) plus a `rectangle_fill_box` wall per box,
        // while we emit a single grid over the whole layer box: measured -63,147
        // mm on those same layers. Landing this needs the block structure
        // (generate_wipe_tower_blocks:4268, plan_tower_new:4477,
        // update_all_layer_depth:4237, finish_block:3733, finish_block_solid:3842),
        // which is also the source of the -68,580 mm shortfall elsewhere.
        let solid_infill = {
            let soluble_next = self
                .plan
                .get(self.layer_idx + 1)
                .map_or(false, |l| {
                    l.tool_changes.iter().any(|tch| {
                        let sol = |t: usize| {
                            self.filament_params.get(t).map_or(false, |p| p.is_soluble)
                        };
                        sol(tch.new_tool) || sol(tch.old_tool)
                    })
                });
            soluble_next || (is_first_layer && self.adhesion)
        };

        // C++: dy = fill_box.lu.y() - fill_box.ld.y() - m_perimeter_width
        let dy = fill_box.height() - self.perimeter_width;
        let mut left = fill_box.lu.x + 2.0 * self.perimeter_width;
        let mut right = fill_box.ru.x - 2.0 * self.perimeter_width;

        // finish_block:3759 always lays the inner perimeter of the sparse
        // section first — `writer.rectangle_fill_box(this, fill_box, ...)`, which
        // is a rectangle OUTLINE walked from the nearest corner, not a fill.
        // finish_layer_new gates the same call on `extrude_fill_wall`.
        if inner_rect && layer_has_toolchange && fill_box.height() > self.perimeter_width {
            writer.rectangle(&fill_box);
        }
        if std::env::var_os("WTFILLCNT").is_some() {
            // R505: count how often our finish-layer fill actually runs, and how
            // often each guard rejects it, to compare against C++'s dispatch
            // (656 layers, 592 block iterations, 386 full-skips, 206 passed).
            use std::sync::atomic::{AtomicUsize, Ordering};
            static SEEN: AtomicUsize = AtomicUsize::new(0);
            static SKIP_NO_TC: AtomicUsize = AtomicUsize::new(0);
            static SKIP_FULL: AtomicUsize = AtomicUsize::new(0);
            SEEN.fetch_add(1, Ordering::Relaxed);
            if !layer_has_toolchange {
                use std::sync::atomic::AtomicUsize as A2;
                static SHOWN: A2 = A2::new(0);
                if SHOWN.fetch_add(1, Ordering::Relaxed) < 5 {
                    eprintln!(
                        "[WTNOTC] z={:.2} layer_depth={:.3} fill_box_h={:.3} dy={:.3} tc_depth={:.3}",
                        self.z_pos,
                        layer_depth,
                        fill_box.height(),
                        dy,
                        self.plan.get(self.layer_idx).map_or(-1.0, |l| l.toolchanges_depth())
                    );
                }
            }
            if fill_only_on_toolchange_layers && !layer_has_toolchange {
                SKIP_NO_TC.fetch_add(1, Ordering::Relaxed);
            } else if !(dy > self.perimeter_width) {
                SKIP_FULL.fetch_add(1, Ordering::Relaxed);
            }
            if self.layer_idx + 1 >= self.plan.len() {
                eprintln!(
                    "[WTFILLCNT] seen={} skip_no_tc={} skip_full={} passed={}",
                    SEEN.load(Ordering::Relaxed),
                    SKIP_NO_TC.load(Ordering::Relaxed),
                    SKIP_FULL.load(Ordering::Relaxed),
                    SEEN.load(Ordering::Relaxed)
                        - SKIP_NO_TC.load(Ordering::Relaxed)
                        - SKIP_FULL.load(Ordering::Relaxed)
                );
            }
        }
        if fill_only_on_toolchange_layers && !layer_has_toolchange {
            // C++ draws only the wall here.
        } else if !grid_faithful {
            // Pre-R493 behaviour: always a solid zig-zag over the whole
            // remaining layer box. Kept as the default while the block port is
            // incomplete — see the FIDELITY-NOTE above.
            if fill_box.height() > self.perimeter_width {
                let sparse_factor = if is_first_layer { 1.0 } else { self.extra_spacing };
                let spacing = self.perimeter_width * sparse_factor;
                writer.rectangle_fill_box(&fill_box, spacing);
            }
        } else if dy > self.perimeter_width {
            // WipeTower.cpp:3604-3607 — this branch is C++'s
            // `extrude_fill && dy > m_perimeter_width`, and it is where the
            // `CP EMPTY GRID` block opens. R533: measured with the R505
            // `WTFILLCNT` probe, our guard passes on 207 layers against C++'s 209
            // emissions — the same constant −2 seen across every tool-change
            // counter (R531/R532), not a branch mismatch.
            //
            // C++ also emits `.comment_with_value(" layer #", m_num_layer_changes + 1)`
            // here. That counter has no exact counterpart on our side, so it is
            // deliberately NOT emitted rather than guessed (R528) — the reserved
            // tag itself is what GCodeProcessor consumes.
            writer.append(";--------------------\n; CP EMPTY GRID START\n");
            if solid_infill {
                let mut sparse_factor = 1.5_f32;
                if is_first_layer {
                    // the infill should touch perimeters
                    left -= self.perimeter_width;
                    right += self.perimeter_width;
                    sparse_factor = 1.0;
                }
                let mut y = fill_box.ld.y + self.perimeter_width;
                let n = (dy / (self.perimeter_width * sparse_factor)) as i32;
                if n > 1 {
                    let spacing = (dy - self.perimeter_width) / (n - 1) as f32;
                    for i in 0..n {
                        let x = writer.x();
                        writer.extrude(x, y);
                        writer.extrude(if i % 2 != 0 { left } else { right }, y);
                        y += spacing;
                    }
                    let x = writer.x();
                    writer.extrude(x, fill_box.lu.y);
                }
            } else {
                // Extrude an inverse U at the left of the region and the sparse infill.
                writer.extrude(
                    fill_box.lu.x + self.perimeter_width * 2.0,
                    fill_box.lu.y,
                );
                let n = 1 + ((right - left) / self.config.bridging) as i32;
                let dx = (right - left) / n as f32;
                for i in 1..=n {
                    let x = left + dx * i as f32;
                    let cy = writer.y();
                    writer.travel(x, cy);
                    writer.extrude(x, if i % 2 != 0 { fill_box.rd.y } else { fill_box.ru.y });
                }
            }

            // WipeTower.cpp:3643-3644 — closes the block, with C++'s separator
            // and its seven trailing blank lines.
            writer.append("; CP EMPTY GRID END\n;------------------\n\n\n\n\n\n\n\n");
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

        // WipeTower.cpp:3721 — closes the block opened above, just before the
        // material accounting.
        writer.append("; WIPE_TOWER_END\n");

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

        // WipeTower.cpp:1521-1522 — start_pos = writer.start_pos_rotated();
        // end_pos = priming ? writer.pos() : writer.pos_rotated().
        // (This reduced port keeps its own convention of offsetting by self.pos
        // to make the positions absolute.)
        let start_pos = Vec2f::new(
            self.pos.x + writer.start_pos_rotated().x,
            self.pos.y + writer.start_pos_rotated().y,
        );
        let end_pos_raw = if priming {
            writer.pos()
        } else {
            writer.pos_rotated()
        };
        let end_pos = Vec2f::new(self.pos.x + end_pos_raw.x, self.pos.y + end_pos_raw.y);

        ToolChangeResult {
            print_z: self.z_pos,
            layer_height: self.layer_height,
            gcode: writer.gcode().to_string(),
            extrusions: writer.extrusions().to_vec(),
            start_pos,
            end_pos,
            elapsed_time: writer.elapsed_time(),
            priming,
            is_tool_change,
            // WipeTower.cpp:1529 — is_tool_change ? start_pos : Vec2f(0,0).
            tool_change_start_pos: if is_tool_change {
                start_pos
            } else {
                Vec2f::zero()
            },
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
        // WipeTower.cpp:2871 — plan_toolchange does NOT bump m_num_tool_changes;
        // it records the change on the layer's tool_changes list instead. The
        // counter is managed in set_layer / the tool_change pass.
        let total_tool_changes: usize =
            tower.plan.iter().map(|l| l.tool_changes.len()).sum();
        assert_eq!(total_tool_changes, 3);
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
        // Travels are emitted as G1 (WipeTower.cpp:766), so both the travel and
        // the extrude are G1 moves.
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
        // WipeTower.cpp:1602-1605 — below 100mm the brim scales linearly to 8mm,
        // at/above 100mm it is capped at 8mm.
        let brim_50 = WipeTower::get_auto_brim_by_height(50.0);
        let brim_100 = WipeTower::get_auto_brim_by_height(100.0);
        let brim_250 = WipeTower::get_auto_brim_by_height(250.0);

        assert!((brim_50 - 4.0).abs() < 1e-6);
        assert!((brim_100 - 8.0).abs() < 1e-6);
        assert!((brim_250 - 8.0).abs() < 1e-6);
        assert!(brim_100 >= brim_50);
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
