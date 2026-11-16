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
    /// `gcfMarlinLegacy` ("marlin")
    #[default]
    Marlin,
    /// `gcfRepRapFirmware` ("reprapfirmware")
    RepRap,
    /// `gcfKlipper`
    Klipper,
    /// `gcfSmoothie`
    Smoothie,
    /// `gcfMach3`
    Mach3,
    /// The other C++ flavors (PrintConfig.hpp `enum GCodeFlavor`).
    RepRapSprinter,
    Repetier,
    Teacup,
    MakerWare,
    MarlinFirmware,
    Sailfish,
    Machinekit,
    NoExtrusion,
}

impl GCodeFlavor {
    /// The flavor of a `gcode_flavor` config value (PrintConfig.cpp
    /// `s_keys_map_GCodeFlavor`).
    pub fn from_config_value(v: &str) -> Self {
        match v {
            "reprap" => GCodeFlavor::RepRapSprinter,
            "reprapfirmware" => GCodeFlavor::RepRap,
            "repetier" => GCodeFlavor::Repetier,
            "teacup" => GCodeFlavor::Teacup,
            "makerware" => GCodeFlavor::MakerWare,
            "marlin" => GCodeFlavor::Marlin,
            "marlin2" => GCodeFlavor::MarlinFirmware,
            "klipper" => GCodeFlavor::Klipper,
            "sailfish" => GCodeFlavor::Sailfish,
            "smoothie" => GCodeFlavor::Smoothie,
            "mach3" => GCodeFlavor::Mach3,
            "machinekit" => GCodeFlavor::Machinekit,
            "no-extrusion" => GCodeFlavor::NoExtrusion,
            _ => GCodeFlavor::Marlin,
        }
    }
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
    /// WipeTower.hpp:65 `bool zero_flush_idle_cooling{false}`
    pub zero_flush_idle_cooling: bool,
    /// WipeTower.hpp:66 `bool heater_off_only{false}`
    pub heater_off_only: bool,
    /// WipeTower.hpp:67 `float precool_target_temp{0}`
    pub precool_target_temp: f32,
    /// WipeTower.hpp:68 `int precool_filament_id{-1}`. `Default` gives 0; the
    /// C++ default is what `NozzleChangeResult::new_cpp()` gives.
    pub precool_filament_id: i32,
}

impl NozzleChangeResult {
    /// WipeTower.hpp:54-69 — a value-initialised `NozzleChangeResult` with the
    /// member initialisers of the C++ struct (`precool_filament_id{-1}`).
    pub fn new_cpp() -> Self {
        Self { precool_filament_id: -1, ..Default::default() }
    }
}

/// Result of a tool change operation
#[derive(Debug, Clone, Default)]
pub struct ToolChangeResult {
    /// Print height of this tool change
    pub print_z: f32,
    /// Layer height
    pub layer_height: f32,
    /// Tower extrusion width (`WipeTower::m_perimeter_width`).
    ///
    /// R534: carried on the result so the export side can emit C++'s
    /// `; LINE_WIDTH:` analyzer tag (`WipeTowerWriter` ctor, WipeTower.cpp:642)
    /// without reaching back into the tower.
    pub perimeter_width: f32,
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
    /// This result is the outer wall drawn on its own at the start of the layer
    /// (`only_generate_out_wall`, which `generate_new` puts in front of the
    /// layer's results under timelapse, WipeTower.cpp:8072-8073).
    pub is_out_wall_only: bool,
    /// WipeTower.hpp:117 `bool is_contact = false`
    pub is_contact: bool,
    /// WipeTower.hpp:121 `bool side_scrape = false`
    pub side_scrape: bool,
    /// WipeTower.hpp:127 `float side_scrape_z_low = 0.f`
    pub side_scrape_z_low: f32,
    /// WipeTower.hpp:128 `float side_scrape_depth = 0.f`
    pub side_scrape_depth: f32,
    /// WipeTower.hpp:129 `float side_scrape_min_height_above_base = 0.f`
    pub side_scrape_min_height_above_base: f32,
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
    /// WipeTower.hpp:369 `float perimeter_width{0.f}`: this filament's tower
    /// line width; 0 falls back to the tower-wide width
    /// (`filament_perimeter_width`).
    pub perimeter_width: f32,
    /// WipeTower.hpp:370 `float nozzle_change_perimeter_width{0.f}`
    pub nozzle_change_perimeter_width: f32,
    /// WipeTower.hpp:372 `float feedrate_factor{1.f}`
    pub feedrate_factor: f32,
    /// WipeTower.hpp:383 `bool idle_travel_cooling_enabled = true`
    pub idle_travel_cooling_enabled: bool,
    /// WipeTower.hpp:384 `float flat_iron_area`
    pub flat_iron_area: f32,
    /// WipeTower.hpp:385 `float filament_tower_interface_print_temp`
    pub filament_tower_interface_print_temp: f32,
    /// WipeTower.hpp:386 `float filament_tower_interface_pre_extrusion_dist = 0`
    pub filament_tower_interface_pre_extrusion_dist: f32,
    /// WipeTower.hpp:387 `float filament_tower_interface_pre_extrusion_length = 0`
    pub filament_tower_interface_pre_extrusion_length: f32,
    /// WipeTower.hpp:388 `std::vector<double> speed_factor_curve`
    pub speed_factor_curve: Vec<f64>,
    /// WipeTower.hpp:389 `float filament_petg_pre_extrusion_offset_dist = 0`
    pub filament_petg_pre_extrusion_offset_dist: f32,
}

impl FilamentParameters {
    /// WipeTower.hpp:345-390 — `FilamentParameters()` with the C++ member
    /// initialisers (the members without one are value-initialised to 0 by the
    /// `push_back(FilamentParameters())` of `set_extruder`, WipeTower.cpp:2020).
    /// `Default` keeps the values this port has always used.
    pub fn new_cpp() -> Self {
        Self {
            material: "PLA".to_string(),
            category: 0,
            is_soluble: false,
            is_support: false,
            nozzle_temperature: 0,
            nozzle_temperature_initial_layer: 0,
            ramming_line_width_multiplicator: 1.0,
            ramming_step_multiplicator: 1.0,
            max_e_speed: f32::MAX,
            ramming_speed: vec![],
            nozzle_diameter: 0.0,
            filament_area: 0.0,
            retract_length: 0.0,
            retract_speed: 0.0,
            wipe_dist: 0.0,
            max_e_ramming_speed: (0.0, 0.0),
            ramming_travel_time: (0.0, 0.0),
            precool_t: (vec![], vec![]),
            precool_t_first_layer: (vec![], vec![]),
            precool_target_temp: (0, 0),
            filament_cooling_before_tower: 0.0,
            perimeter_width: 0.0,
            nozzle_change_perimeter_width: 0.0,
            feedrate_factor: 1.0,
            idle_travel_cooling_enabled: true,
            flat_iron_area: 0.0,
            filament_tower_interface_print_temp: 0.0,
            filament_tower_interface_pre_extrusion_dist: 0.0,
            filament_tower_interface_pre_extrusion_length: 0.0,
            speed_factor_curve: vec![],
            filament_petg_pre_extrusion_offset_dist: 0.0,
        }
    }
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
            perimeter_width: 0.0,
            nozzle_change_perimeter_width: 0.0,
            feedrate_factor: 1.0,
            idle_travel_cooling_enabled: true,
            flat_iron_area: 0.0,
            filament_tower_interface_print_temp: 0.0,
            filament_tower_interface_pre_extrusion_dist: 0.0,
            filament_tower_interface_pre_extrusion_length: 0.0,
            speed_factor_curve: vec![],
            filament_petg_pre_extrusion_offset_dist: 0.0,
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
            // TOWER_EXTRUDER_FILL_CPP (on): `bool extruder_fill{true}`
            // (WipeTower.hpp:916); the print clears it on the timelapse layers
            // without tool changes (Print.cpp:4332-4334). =0 false everywhere,
            // which left every finish_block without its grid.
            extruder_fill: crate::faithful_gate("TOWER_EXTRUDER_FILL_CPP"),
            tool_changes: vec![],
        }
    }

    /// Calculate total depth for all tool changes
    pub fn toolchanges_depth(&self) -> f32 {
        self.tool_changes.iter().map(|tc| tc.required_depth).sum()
    }
}

// ============================================================================
// Wall-gap geometry helpers (R623)
//
// These are the leaf functions of C++'s LIVE wall-gap chain:
//     generate_support_wall_new (WipeTower.cpp:5030)
//       -> contrust_gap_for_skip_points (:595)
//         -> remove_points_from_polygon (:510)
//       -> WipeTowerWriter::generate_path (:1249)
//
// R622 established that the OTHER chain — `generate_support_wall` (:5081) and
// `remove_points_from_segment` (:298) — is dead; both of its call sites (:3661,
// :4977) are commented out. These three helpers belong to the live side.
//
// They are ported here as pure functions with unit tests, ahead of
// `remove_points_from_polygon` itself, because that function is meaningless
// until they are correct and they can be checked in isolation. Nothing calls
// them yet, so this is byte-neutral by construction.
// ============================================================================

/// 2D cross product. WipeTower.cpp uses Slic3r's `cross2`.
fn cross2(a: Vec2f, b: Vec2f) -> f64 {
    a.x as f64 * b.y as f64 - a.y as f64 * b.x as f64
}

/// Intersect the ray `a + t*v1` (t >= 0) with the segment `b..c`.
// WipeTower.cpp:230-246 (ray_intersetion_line — C++'s spelling, kept).
// Returns the hit point only when t1 >= 0 (forward along the ray) and
// 0 <= t2 <= 1 (within the segment).
pub fn ray_intersetion_line(a: Vec2f, v1: Vec2f, b: Vec2f, c: Vec2f) -> Option<Vec2f> {
    // TOWER_RAY_CROSS2_CPP (on): `cross2` of two Vec2f is a float, one
    // contracted multiply-add (Point.hpp:92-96), promoted to double for the
    // divisions; =0 the products in double this port used.
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *G.get_or_init(|| crate::faithful_gate("TOWER_RAY_CROSS2_CPP")) {
        let v2 = c - b;
        let denom = cross2_f32(v1, v2) as f64;
        if denom.abs() < crate::libslic3r::EPSILON {
            return None;
        }
        let v12 = a - b;
        let nume_a = cross2_f32(v2, v12) as f64;
        let nume_b = cross2_f32(v1, v12) as f64;
        let t1 = nume_a / denom;
        let t2 = nume_b / denom;
        if t1 >= 0.0 && t2 >= 0.0 && t2 <= 1.0 {
            // `a + t1 * v1`: the double t1 converted to float.
            return Some(a + v1 * (t1 as f32));
        }
        return None;
    }
    let v2 = Vec2f::new(c.x - b.x, c.y - b.y);
    let denom = cross2(v1, v2);
    if denom.abs() < WT_EPSILON as f64 {
        return None;
    }
    let v12 = Vec2f::new(a.x - b.x, a.y - b.y);
    let t1 = cross2(v2, v12) / denom;
    let t2 = cross2(v1, v12) / denom;
    if t1 >= 0.0 && t2 >= 0.0 && t2 <= 1.0 {
        return Some(Vec2f::new(
            a.x + t1 as f32 * v1.x,
            a.y + t1 as f32 * v1.y,
        ));
    }
    None
}

/// A point on the wall polygon, tagged with the gap it belongs to.
// WipeTower.cpp's PointWithFlag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointWithFlag {
    pub pos: Vec2f,
    /// Index of the skip point that opened this gap, or -1 for an ordinary vertex.
    pub pair_idx: i32,
    pub is_forward: bool,
}

/// Where a gap boundary lands on the polygon.
// WipeTower.cpp's IntersectionInfo.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntersectionInfo {
    pub pos: Vec2f,
    pub idx: i32,
    pub pair_idx: i32,
    pub dis_from_idx: f32,
    pub is_forward: bool,
}

impl Default for IntersectionInfo {
    fn default() -> Self {
        Self { pos: Vec2f::zero(), idx: 0, pair_idx: -1, dis_from_idx: 0.0, is_forward: false }
    }
}

fn dist(a: Vec2f, b: Vec2f) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    (dx * dx + dy * dy).sqrt()
}

/// Walk `offset` mm along the polygon from `start_point`, forwards or backwards.
// WipeTower.cpp:337-397 (move_point_along_polygon). The two directions are NOT
// mirror images in C++ — the backward branch measures `dis_from_idx` as
// `segmentLength - remainingDistance` and steps from `points[(i+1) % mod]`,
// while the forward branch measures it as `remainingDistance` from `points[i]`.
// Both are reproduced literally rather than folded together.
pub fn move_point_along_polygon(
    points: &[Vec2f],
    start_point: Vec2f,
    start_idx: usize,
    offset: f32,
    forward: bool,
    pair_idx: i32,
) -> IntersectionInfo {
    let mut remaining = offset;
    let m = points.len();
    let mut res = IntersectionInfo { pair_idx, ..Default::default() };
    if m == 0 {
        return res;
    }
    let next = (start_idx + 1) % m;

    if forward {
        remaining -= dist(points[next], start_point);
        if remaining <= 0.0 {
            let d = dist(points[next], start_point);
            let dir = if d > 0.0 {
                Vec2f::new((points[next].x - start_point.x) / d, (points[next].y - start_point.y) / d)
            } else {
                Vec2f::zero()
            };
            res.idx = start_idx as i32;
            res.pos = Vec2f::new(start_point.x + dir.x * offset, start_point.y + dir.y * offset);
            res.dis_from_idx = dist(points[start_idx], res.pos);
            return res;
        }
        let mut i = (start_idx + 1) % m;
        while i != start_idx {
            let j = (i + 1) % m;
            let seg = dist(points[j], points[i]);
            if remaining <= seg {
                let ratio = if seg > 0.0 { remaining / seg } else { 0.0 };
                res.idx = i as i32;
                res.pos = Vec2f::new(
                    points[i].x + ratio * (points[j].x - points[i].x),
                    points[i].y + ratio * (points[j].y - points[i].y),
                );
                res.dis_from_idx = remaining;
                return res;
            }
            remaining -= seg;
            i = (i + 1) % m;
        }
        res.idx = ((start_idx + m - 1) % m) as i32;
        res.pos = points[start_idx];
        res.dis_from_idx = dist(res.pos, points[res.idx as usize]);
    } else {
        remaining -= dist(points[start_idx], start_point);
        if remaining <= 0.0 {
            let d = dist(points[next], points[start_idx]);
            let dir = if d > 0.0 {
                Vec2f::new((points[next].x - points[start_idx].x) / d, (points[next].y - points[start_idx].y) / d)
            } else {
                Vec2f::zero()
            };
            res.idx = start_idx as i32;
            res.pos = Vec2f::new(start_point.x - dir.x * offset, start_point.y - dir.y * offset);
            res.dis_from_idx = dist(res.pos, points[start_idx]);
            return res;
        }
        let mut i = (start_idx + m - 1) % m;
        while i != start_idx {
            let j = (i + 1) % m;
            let seg = dist(points[j], points[i]);
            if remaining <= seg {
                let ratio = if seg > 0.0 { remaining / seg } else { 0.0 };
                res.idx = i as i32;
                res.pos = Vec2f::new(
                    points[j].x - ratio * (points[j].x - points[i].x),
                    points[j].y - ratio * (points[j].y - points[i].y),
                );
                res.dis_from_idx = seg - remaining;
                return res;
            }
            remaining -= seg;
            i = (i + m - 1) % m;
        }
        res.idx = start_idx as i32;
        res.pos = points[res.idx as usize];
        res.dis_from_idx = 0.0;
    }
    res
}

/// Tag an existing vertex, or splice a new one in after `idx`.
// WipeTower.cpp:399-413 (insert_points). C++ compares with squaredNorm against
// EPSILON, i.e. a squared tolerance; reproduced as such rather than as a
// distance comparison.
pub fn insert_points(
    pl: &mut Vec<PointWithFlag>,
    idx: usize,
    pos: Vec2f,
    pair_idx: i32,
    is_forward: bool,
) {
    if pl.is_empty() {
        return;
    }
    let next = (idx + 1) % pl.len();
    let sq = |a: Vec2f, b: Vec2f| {
        let dx = a.x - b.x;
        let dy = a.y - b.y;
        dx * dx + dy * dy
    };
    if sq(pos, pl[idx].pos) < WT_EPSILON {
        pl[idx].pair_idx = pair_idx;
        pl[idx].is_forward = is_forward;
    } else if sq(pos, pl[next].pos) < WT_EPSILON {
        pl[next].pair_idx = pair_idx;
        pl[next].is_forward = is_forward;
    } else {
        pl.insert(idx + 1, PointWithFlag { pos, pair_idx, is_forward });
    }
}

/// Polygon -> Polyline, CLOSING the ring.
// Polygon.hpp:224-231 (to_polyline) — the first point is pushed again at the
// end. R624: omitting that closing point made an ungapped wall 3 sides long
// instead of 4; caught by `gaps_actually_remove_length`, which measured the
// "whole" baseline as SHORTER than the gapped wall.
fn to_polyline(polygon: &crate::geometry::Polygon) -> crate::geometry::Polyline {
    let mut pl = crate::geometry::Polyline::new();
    let pts = polygon.points();
    pl.points.reserve(pts.len() + 1);
    for p in pts {
        pl.points.push(*p);
    }
    if let Some(first) = pts.first() {
        pl.points.push(*first);
    }
    pl
}

/// Insert three extra vertices on the polygon edge nearest the anchor.
// WipeTower.cpp:417-506 (add_extra_point). The anchor is (bbox centre x, bbox
// min y); the edge whose MIDPOINT is closest to it gets `offset_to_a`, `mid`
// and `offset_to_b` spliced in after its start vertex, so the ring gains a
// vertex exactly at the middle of that edge plus one either side.
//
// `range` is clamped to 0.9 * the shorter half-edge (`:471`) so the inserted
// points cannot overshoot the edge.
pub fn add_extra_point(polygon: &crate::geometry::Polygon, scale_range: f64) -> crate::geometry::Polygon {
    use crate::geometry::Point;
    let pts = polygon.points();
    if pts.len() < 2 {
        return polygon.clone();
    }
    let bbox = polygon.bounding_box();
    // WipeTower.cpp:426 — X at the bbox centre, Y at the bbox bottom.
    let anchor = Vec2f::new(
        ((bbox.min.x + bbox.max.x) as f64 / 2.0) as f32,
        bbox.min.y as f32,
    );

    // :429-446 — the edge whose midpoint is nearest the anchor.
    let mut closest_edge_idx = 0usize;
    let mut min_dist_sq = f32::MAX;
    for i in 0..pts.len() {
        let a = Vec2f::new(pts[i].x as f32, pts[i].y as f32);
        let b_i = pts[(i + 1) % pts.len()];
        let b = Vec2f::new(b_i.x as f32, b_i.y as f32);
        let mid = Vec2f::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        let dx = anchor.x - mid.x;
        let dy = anchor.y - mid.y;
        let d = dx * dx + dy * dy;
        if d < min_dist_sq {
            min_dist_sq = d;
            closest_edge_idx = i;
        }
    }

    let a_i = pts[closest_edge_idx];
    let b_i = pts[(closest_edge_idx + 1) % pts.len()];
    let a = Vec2f::new(a_i.x as f32, a_i.y as f32);
    let b = Vec2f::new(b_i.x as f32, b_i.y as f32);
    let mid = Vec2f::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);

    let mut dir_to_a = Vec2f::new(a.x - mid.x, a.y - mid.y);
    let mut dir_to_b = Vec2f::new(b.x - mid.x, b.y - mid.y);
    let len_a = (dir_to_a.x * dir_to_a.x + dir_to_a.y * dir_to_a.y).sqrt();
    let len_b = (dir_to_b.x * dir_to_b.x + dir_to_b.y * dir_to_b.y).sqrt();
    // :464 — a degenerate edge is left alone.
    if len_a < WT_EPSILON || len_b < WT_EPSILON {
        return polygon.clone();
    }
    dir_to_a = Vec2f::new(dir_to_a.x / len_a, dir_to_a.y / len_a);
    dir_to_b = Vec2f::new(dir_to_b.x / len_b, dir_to_b.y / len_b);

    // :471-472 — clamp so the offsets stay on the edge.
    let max_range = len_a.min(len_b) * 0.9;
    let range = (scale_range as f32).min(max_range);

    let to_int = |p: Vec2f| Point::new(p.x.round() as i64, p.y.round() as i64);
    let off_a = to_int(Vec2f::new(mid.x + dir_to_a.x * range, mid.y + dir_to_a.y * range));
    let mid_i = to_int(mid);
    let off_b = to_int(Vec2f::new(mid.x + dir_to_b.x * range, mid.y + dir_to_b.y * range));

    // :494-503 — rebuild, splicing the trio in after the edge's start vertex.
    let mut res = crate::geometry::Polygon::new();
    for (i, p) in pts.iter().enumerate() {
        res.points_mut().push(*p);
        if i == closest_edge_idx {
            res.points_mut().push(off_a);
            res.points_mut().push(mid_i);
            res.points_mut().push(off_b);
        }
    }
    res
}

/// Break the wall polygon open at every skip point.
// WipeTower.cpp:510-593 (remove_points_from_polygon). Returns the surviving
// polyline runs plus `insert_skip_pg`, the full ring INCLUDING the inserted gap
// boundaries, which C++ hands back for the wipe path.
//
// Shape of the algorithm: densify the ring (add_extra_point), rotate it to start
// at the vertex nearest the anchor, then for each skip point cast a horizontal
// ray (inward from whichever side the point sits on) to find where it meets the
// ring; walk `range` forwards and backwards from that hit to get the gap's two
// boundaries; splice those in as tagged vertices; finally walk the ring emitting
// runs and jumping across each tagged pair.
pub fn remove_points_from_polygon(
    polygon_ori: &crate::geometry::Polygon,
    skip_points: &[Vec2f],
    range: f64,
    wt_width: f32,
) -> (Vec<crate::geometry::Polyline>, crate::geometry::Polygon) {
    use crate::geometry::{Point, Polygon, Polyline};
    let polygon = add_extra_point(polygon_ori, crate::scaled(range) as f64);
    let mut insert_skip_pg = Polygon::new();
    if polygon.points().len() < 2 {
        insert_skip_pg = polygon.clone();
        return (vec![to_polyline(&polygon)], insert_skip_pg);
    }

    // :517 — anchor at (bbox centre x, bbox min y), same as add_extra_point's.
    let bbox = polygon.bounding_box();
    let anchor = Point::new((bbox.min.x + bbox.max.x) / 2, bbox.min.y);

    // :519-526 — rotate the ring to start at the nearest vertex. C++ takes
    // split_at_index (which repeats the start vertex at the end) and pops it.
    let idx = polygon.closest_point_index(&anchor);
    let tmp = polygon.split_at_index(idx);
    let mut points: Vec<Vec2f> = tmp
        .points
        .iter()
        .map(|p| Vec2f::new(unscale_d(p.x) as f32, unscale_d(p.y) as f32))
        .collect();
    points.pop();
    if points.is_empty() {
        return (Vec::new(), polygon);
    }

    // :528-545 — one ray per skip point; the first edge it meets defines the gap.
    let mut inter_info: Vec<IntersectionInfo> = Vec::new();
    for (i, sp) in skip_points.iter().enumerate() {
        let is_left = sp.x.abs() < wt_width / 2.0;
        let ray = if is_left { Vec2f::new(-1.0, 0.0) } else { Vec2f::new(1.0, 0.0) };
        for j in 0..points.len() {
            let p1 = points[j];
            let p2 = points[(j + 1) % points.len()];
            if let Some(inter_pos) = ray_intersetion_line(*sp, ray, p1, p2) {
                let mut forward =
                    move_point_along_polygon(&points, inter_pos, j, range as f32, true, i as i32);
                let mut backward =
                    move_point_along_polygon(&points, inter_pos, j, range as f32, false, i as i32);
                forward.is_forward = true;
                backward.is_forward = false;
                inter_info.push(backward);
                inter_info.push(forward);
                break;
            }
        }
    }

    // :547-552 — seed the tagged ring, then splice the boundaries in from the
    // BACK so earlier indices stay valid.
    let mut new_pl: Vec<PointWithFlag> = points
        .iter()
        .map(|p| PointWithFlag { pos: *p, pair_idx: -1, is_forward: false })
        .collect();
    inter_info.sort_by(|l, r| {
        if l.idx == r.idx {
            l.dis_from_idx
                .partial_cmp(&r.dis_from_idx)
                .unwrap_or(std::cmp::Ordering::Equal)
        } else {
            l.idx.cmp(&r.idx)
        }
    });
    for info in inter_info.iter().rev() {
        insert_points(
            &mut new_pl,
            info.idx.max(0) as usize,
            info.pos,
            info.pair_idx,
            info.is_forward,
        );
    }

    // :554-557 — the ring WITH the gap boundaries, returned for the wipe path.
    for p in &new_pl {
        insert_skip_pg
            .points_mut()
            .push(Point::new(scaled_vec2f(p.pos.x), scaled_vec2f(p.pos.y)));
    }

    // :559-588 — walk the ring, emitting runs and jumping across tagged pairs.
    let mut result: Vec<Polyline> = Vec::new();
    let beg = 0usize;
    let mut skip = true;
    let mut i = beg;
    let mut pl = Polyline::new();
    let n = new_pl.len();
    if n == 0 {
        return (result, insert_skip_pg);
    }
    loop {
        if skip || new_pl[i].pair_idx == -1 {
            pl.points.push(Point::new(
                scaled_vec2f(new_pl[i].pos.x),
                scaled_vec2f(new_pl[i].pos.y),
            ));
            i = (i + 1) % n;
            skip = false;
        } else {
            if !pl.points.is_empty() {
                pl.points.push(Point::new(
                    scaled_vec2f(new_pl[i].pos.x),
                    scaled_vec2f(new_pl[i].pos.y),
                ));
                result.push(std::mem::replace(&mut pl, Polyline::new()));
            }
            // Jump to this gap's partner, following any nested gap opened on the
            // way (:577-580).
            let mut left = new_pl[i].pair_idx;
            let mut j = (i + 1) % n;
            while j != beg && new_pl[j].pair_idx != left {
                if new_pl[j].pair_idx != -1 && !new_pl[j].is_forward {
                    left = new_pl[j].pair_idx;
                }
                j = (j + 1) % n;
            }
            i = j;
            skip = true;
        }
        if i == beg {
            break;
        }
    }
    // :585-588
    if !pl.points.is_empty() {
        if new_pl[i].pair_idx == -1 {
            pl.points.push(Point::new(
                scaled_vec2f(new_pl[i].pos.x),
                scaled_vec2f(new_pl[i].pos.y),
            ));
        }
        result.push(pl);
    }
    (result, insert_skip_pg)
}

/// Wall polyline runs for a layer's skip points.
// WipeTower.cpp:595-607 (contrust_gap_for_skip_points — C++'s spelling, kept).
// With no skip points the whole ring is one run.
pub fn contrust_gap_for_skip_points(
    polygon: &crate::geometry::Polygon,
    skip_points: &[Vec2f],
    wt_width: f32,
    gap_length: f64,
) -> (Vec<crate::geometry::Polyline>, crate::geometry::Polygon) {
    if skip_points.is_empty() {
        return (vec![to_polyline(polygon)], polygon.clone());
    }
    remove_points_from_polygon(polygon, skip_points, gap_length, wt_width)
}

/// `scaled(const Vec2f &)` (Point.hpp): the float coordinate divided by the
/// scaling factor as a float, then truncated. TOWER_SCALED_VEC2F (on): so; =0
/// divides in double (`crate::scaled`), which puts 35.0 at 3499999 and a skip
/// point at x = 35 to the right of the wall, where its ray finds no edge.
fn scaled_vec2f(v: f32) -> crate::Coord {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *G.get_or_init(|| crate::faithful_gate("TOWER_SCALED_VEC2F")) {
        (v / crate::libslic3r::SCALING_FACTOR as f32) as crate::Coord
    } else {
        crate::scaled(v as f64)
    }
}

/// The tower's outer wall as a rectangle, counter-clockwise from `ld`.
// WipeTower.cpp:610-618 (generate_rectange_polygon — C++'s spelling, kept).
pub fn generate_rectange_polygon(wt_box_min: Vec2f, wt_box_max: Vec2f) -> crate::geometry::Polygon {
    use crate::geometry::Point;
    let mut res = crate::geometry::Polygon::new();
    let s = |x: f32, y: f32| Point::new(scaled_vec2f(x), scaled_vec2f(y));
    res.points_mut().push(s(wt_box_min.x, wt_box_min.y));
    res.points_mut().push(s(wt_box_max.x, wt_box_min.y));
    res.points_mut().push(s(wt_box_max.x, wt_box_max.y));
    res.points_mut().push(s(wt_box_min.x, wt_box_max.y));
    res
}

// ============================================================================
// Free functions of WipeTower.cpp (and WipeTower.hpp:978-1000)
// ============================================================================

/// WipeTower.cpp:32-39 `nozzle_change_width_for_diameter`: the nozzle-change
/// line width of the table entry whose diameter is nearest (the first of two
/// equally near ones).
pub fn nozzle_change_width_for_diameter(nozzle_diameter: f32) -> f32 {
    let table = &NOZZLE_DIAMETER_TO_NOZZLE_CHANGE_WIDTH;
    let mut best = 0usize;
    for i in 1..table.len() {
        if (table[i].0 - nozzle_diameter).abs() < (table[best].0 - nozzle_diameter).abs() {
            best = i;
        }
    }
    table[best].1
}

/// The configuration the C++ functions of this file take (`const PrintConfig&`):
/// a full print configuration (every option present).
pub type WipeTowerPrintConfig = crate::print_config::config_def::DynamicPrintConfig;

/// `config.<key>.value` of an enum option, as its integer.
fn cfg_enum(config: &WipeTowerPrintConfig, key: &str) -> i32 {
    use crate::config::ConfigOptionResolver;
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
}

/// `config.<key>.value` of a float, percent or float-or-percent option.
fn cfg_float(config: &WipeTowerPrintConfig, key: &str) -> f64 {
    *config.opt_float(key)
}

/// `config.<key>.values` of a float vector option (nullable or not).
fn cfg_floats(config: &WipeTowerPrintConfig, key: &str) -> Vec<f64> {
    use crate::config::ConfigOptionResolver;
    let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionFloats>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionFloatsNullable>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionPercents>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionPercentsNullable>() {
        return o.values.clone();
    }
    panic!("option {key} is not a float vector")
}

/// `config.<key>.values` of an int vector option (nullable or not).
fn cfg_ints(config: &WipeTowerPrintConfig, key: &str) -> Vec<i32> {
    use crate::config::ConfigOptionResolver;
    let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionInts>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionIntsNullable>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionEnumsGeneric>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionEnumsGenericNullable>() {
        return o.values.clone();
    }
    panic!("option {key} is not an int vector")
}

/// `config.<key>.values` of a bool vector option (as the C++ unsigned chars).
fn cfg_bools(config: &WipeTowerPrintConfig, key: &str) -> Vec<bool> {
    use crate::config::ConfigOptionResolver;
    let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionBools>() {
        return o.values.iter().map(|&v| v != 0).collect();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionBoolsNullable>() {
        return o.values.iter().map(|&v| v != 0).collect();
    }
    panic!("option {key} is not a bool vector")
}

/// `config.<key>.values` of a string vector option.
fn cfg_strings(config: &WipeTowerPrintConfig, key: &str) -> Vec<String> {
    use crate::config::ConfigOptionResolver;
    let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
    opt.as_any()
        .downcast_ref::<crate::config::ConfigOptionStrings>()
        .unwrap_or_else(|| panic!("option {key} is not a string vector"))
        .values
        .clone()
}

/// Config.hpp:681 `get_at(i)`: `values[i]`, the first value past the end.
fn get_at<T: Clone>(values: &[T], i: usize) -> T {
    if i < values.len() { values[i].clone() } else { values[0].clone() }
}

/// WipeTower.cpp:41-45 `wipe_tower_sparse_layers_skipped`.
pub fn wipe_tower_sparse_layers_skipped(config: &WipeTowerPrintConfig) -> bool {
    // TimelapseType::tlSmooth == 1 (PrintConfig.hpp)
    config.opt_bool("wipe_tower_no_sparse_layers")
        && cfg_enum(config, "timelapse_type") != 1
        && !config.opt_bool("enable_wrapping_detection")
}

/// WipeTower.cpp:47-50 `wipe_tower_layer_is_sparse`.
pub fn wipe_tower_layer_is_sparse(layer_tool_changes: &[ToolChangeResult]) -> bool {
    layer_tool_changes.len() == 1 && layer_tool_changes[0].initial_tool == layer_tool_changes[0].new_tool
}

/// WipeTower.cpp:52-59 `prime_tower_side_scrape_margin`.
pub fn prime_tower_side_scrape_margin(config: &WipeTowerPrintConfig) -> f64 {
    if !config.opt_bool("prime_tower_side_scrape") {
        return 0.0;
    }
    0.0_f64.max(cfg_float(config, "prime_tower_side_scrape_wall_gap") + cfg_float(config, "prime_tower_side_scrape_length"))
}

/// WipeTower.cpp:61-71 `compute_compacted_wipe_tower_z`.
pub fn compute_compacted_wipe_tower_z(tool_changes: &[Vec<ToolChangeResult>]) -> Vec<f32> {
    let mut tower_z = vec![0.0_f32; tool_changes.len()];
    let mut last = 0.0_f32;
    for i in 0..tool_changes.len() {
        if !tool_changes[i].is_empty() && !wipe_tower_layer_is_sparse(&tool_changes[i]) {
            last += tool_changes[i][0].layer_height;
        }
        tower_z[i] = last;
    }
    tower_z
}

/// WipeTower.cpp:73-101 `parseDoubles`: the comma separated numbers of a
/// `filament_prime_tower_speed_factor_curve`, each clamped to [0.01, 5]; empty
/// when one of them does not parse.
pub fn parse_doubles(str: &str) -> Vec<f64> {
    let mut result: Vec<f64> = Vec::new();
    if str.is_empty() {
        return result;
    }
    // :78-84 — the C-locale number after leading blanks; nothing read throws.
    let to_double = |token: &str| -> Option<f64> {
        let first = token.find(|c: char| c != ' ' && c != '\t');
        let (value, pos) = match first {
            None => (0.0, 0usize),
            Some(f) => crate::locales_utils::string_to_double_decimal_point(&token[f..]),
        };
        if pos == 0 {
            return None;
        }
        Some(value)
    };
    let mut start = 0usize;
    let mut ok = true;
    while let Some(off) = str[start..].find(',') {
        let end = start + off;
        match to_double(&str[start..end]) {
            Some(v) => result.push(v),
            None => {
                ok = false;
                break;
            }
        }
        start = end + 1;
    }
    if ok {
        match to_double(&str[start..]) {
            Some(v) => result.push(v),
            None => ok = false,
        }
    }
    if !ok {
        // :92-94 — warning logged, built-in default.
        return Vec::new();
    }
    for v in &mut result {
        if *v > 5.0 {
            *v = 5.0;
        }
        if *v < 0.01 {
            *v = 0.01;
        }
    }
    result
}

/// `unscaled(Point)` as a double vector.
#[inline]
fn unscaled_pt(p: &Point) -> (f64, f64) {
    (p.x as f64 * crate::libslic3r::SCALING_FACTOR, p.y as f64 * crate::libslic3r::SCALING_FACTOR)
}

/// `scaled(Vec2d)`: `(v / SCALING_FACTOR).cast<coord_t>()`, truncating.
#[inline]
fn scaled_vec2d(x: f64, y: f64) -> Point {
    Point::new(
        (x / crate::libslic3r::SCALING_FACTOR) as crate::Coord,
        (y / crate::libslic3r::SCALING_FACTOR) as crate::Coord,
    )
}

/// `scaled(Vec2f)`: the float vector divided by the scaling factor as a float,
/// then truncated (see `scaled_vec2f`).
#[inline]
fn scaled_vec2f_pt(v: Vec2f) -> Point {
    Point::new(scaled_vec2f(v.x), scaled_vec2f(v.y))
}

/// `unscaled<float>(Point)`: each coordinate as a float times the float
/// scaling factor.
#[inline]
fn unscaled_f(p: &Point) -> Vec2f {
    let sf = crate::libslic3r::SCALING_FACTOR as f32;
    Vec2f::new(p.x as f32 * sf, p.y as f32 * sf)
}

/// TOWER_UNSCALE_MUL_CPP (on): `unscale(Point)` / `unscaled(Point)` multiply by
/// SCALING_FACTOR (Point.hpp:560-577) where the reduced pipeline divided by
/// 1e5 (`crate::unscale`), which differs in the last bit; =0 divides.
fn unscale_mul_gate() -> bool {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *G.get_or_init(|| crate::faithful_gate("TOWER_UNSCALE_MUL_CPP"))
}

/// `unscale(coord)` as a double (see `unscale_mul_gate`).
#[inline]
fn unscale_d(v: crate::Coord) -> f64 {
    if unscale_mul_gate() {
        v as f64 * crate::libslic3r::SCALING_FACTOR
    } else {
        crate::unscale(v)
    }
}

/// `Point(double, double)`: each coordinate `lrint`-ed (Point.hpp:179).
#[inline]
fn point_from_f64(x: f64, y: f64) -> Point {
    Point::new(x.round_ties_even() as crate::Coord, y.round_ties_even() as crate::Coord)
}

/// `std::sin(x)` and `std::cos(x)` of one float, as clang compiles the two side
/// by side: LLVM folds the pair into one sincos call where the platform has one
/// (Darwin `__sincosf_stret`, glibc `sincosf`), which can differ from separate
/// `sinf` / `cosf` in the last bit. Kept out of line so that the pair stays in
/// one block and Rust's LLVM folds it the same way.
#[inline(never)]
fn sincos_f32(x: f32) -> (f32, f32) {
    (x.sin(), x.cos())
}

/// `sin(x)` and `cos(x)` of one double side by side (see `sincos_f32`).
#[inline(never)]
fn sincos_f64(x: f64) -> (f64, f64) {
    (x.sin(), x.cos())
}

/// The corner geometry shared by `chamfer_polygon`, `WipeTower::rounding_polygon`
/// and `rounding_rectangle` (WipeTower.cpp:155-168): the unscaled corner `b`,
/// the unit directions into and out of it, the two edge lengths, and the
/// float cosine of the turn.
fn polygon_corner(polygon: &Polygon, i: usize) -> ((f64, f64), (f64, f64), (f64, f64), f64, f64, f32) {
    let pts = polygon.points();
    let m = pts.len() as i32;
    let a = unscaled_pt(&pts[((i as i32 - 1 + m) % m) as usize]);
    let b = unscaled_pt(&pts[i]);
    let c = unscaled_pt(&pts[(i + 1) % pts.len()]);
    let ab_len = ((a.0 - b.0) * (a.0 - b.0) + (a.1 - b.1) * (a.1 - b.1)).sqrt();
    let bc_len = ((b.0 - c.0) * (b.0 - c.0) + (b.1 - c.1) * (b.1 - c.1)).sqrt();
    let ab = ((b.0 - a.0) / ab_len, (b.1 - a.1) / ab_len);
    let bc = ((c.0 - b.0) / bc_len, (c.1 - b.1) / bc_len);
    let cosangle = (ab.0 * bc.0 + ab.1 * bc.1) as f32;
    (b, ab, bc, ab_len, bc_len, cosangle)
}

/// WipeTower.cpp:147-179 `chamfer_polygon(polygon, chamfer_dis = 2, angle_tol = 30°)`.
pub fn chamfer_polygon(polygon: &Polygon, chamfer_dis: f64, angle_tol: f64) -> Polygon {
    if polygon.points().len() < 3 {
        return polygon.clone();
    }
    let mut res = Polygon::new();
    let cos_angle_tol = angle_tol.cos().abs();
    for i in 0..polygon.points().len() {
        let (b, ab, bc, ab_len, bc_len, cosangle) = polygon_corner(polygon, i);
        if (cosangle.abs() as f64) < cos_angle_tol {
            // :169 — std::min({double...}) stored as float.
            let real_chamfer_dis = chamfer_dis.min(ab_len / 2.1).min(bc_len / 2.1) as f32 as f64;
            let left = (b.0 - ab.0 * real_chamfer_dis, b.1 - ab.1 * real_chamfer_dis);
            let right = (b.0 + bc.0 * real_chamfer_dis, b.1 + bc.1 * real_chamfer_dis);
            res.points_mut().push(scaled_vec2d(left.0, left.1));
            res.points_mut().push(scaled_vec2d(right.0, right.1));
        } else {
            res.points_mut().push(polygon.points()[i]);
        }
    }
    res
}

/// The arc points of WipeTower.cpp:222-238 / 280-296: `arc_fit_size` points
/// from the start of the arc, at float angles.
fn push_arc_points(res: &mut Polygon, arc: &crate::circle::ArcSegment) {
    const ARC_FIT_SIZE: i32 = 20; // WipeTower.cpp:27
    let n = ARC_FIT_SIZE;
    for j in 0..n {
        // :228 — `float cur_angle = theta0 + (float)j/n * angle`: the float
        // ratio promoted, one contracted double multiply-add, stored as float.
        let mut cur_angle = ((j as f32 / n as f32) as f64).mul_add(arc.angle_radians, arc.polar_start_theta) as f32;
        if cur_angle as f64 > 2.0 * std::f64::consts::PI {
            cur_angle = (cur_angle as f64 - 2.0 * std::f64::consts::PI) as f32;
        } else if cur_angle < 0.0 {
            cur_angle = (cur_angle as f64 + 2.0 * std::f64::consts::PI) as f32;
        }
        // :234 — `arc.center + Point{radius * std::cos(cur_angle), ...}`: the
        // float overloads of cos / sin, the products in double, lrint-ed.
        let (sin_a, cos_a) = sincos_f32(cur_angle);
        let d = point_from_f64(arc.circle.radius * cos_a as f64, arc.circle.radius * sin_a as f64);
        res.points_mut().push(Point::new(arc.circle.center.x + d.x, arc.circle.center.y + d.y));
    }
}

/// WipeTower.cpp:249-304 `rounding_rectangle(polygon, rounding = 2, angle_tol = 30°)`.
pub fn rounding_rectangle(polygon: &Polygon, rounding: f64, angle_tol: f64) -> Polygon {
    if polygon.points().len() < 3 {
        return polygon.clone();
    }
    let mut res = Polygon::new();
    let cos_angle_tol = angle_tol.cos().abs();
    for i in 0..polygon.points().len() {
        let (b, ab, bc, ab_len, bc_len, cosangle) = polygon_corner(polygon, i);
        let cosangle = cosangle.clamp(-1.0, 1.0);
        let is_ccw = cross2_f64(ab, bc) > 0.0;
        if (cosangle.abs() as f64) < cos_angle_tol {
            let real_rounding_dis = rounding.min(ab_len / 2.1).min(bc_len / 2.1) as f32 as f64;
            let left = (b.0 - ab.0 * real_rounding_dis, b.1 - ab.1 * real_rounding_dis);
            let right = (b.0 + bc.0 * real_rounding_dis, b.1 + bc.1 * real_rounding_dis);
            let center = b;
            let radius = real_rounding_dis;
            let arc = crate::circle::ArcSegment::with_parameters(
                scaled_vec2d(center.0, center.1),
                (radius / crate::libslic3r::SCALING_FACTOR) as crate::Coord as f64,
                scaled_vec2d(left.0, left.1),
                scaled_vec2d(right.0, right.1),
                if is_ccw { crate::circle::ArcDirection::CounterClockwise } else { crate::circle::ArcDirection::Clockwise },
            );
            push_arc_points(&mut res, &arc);
            res.points_mut().push(scaled_vec2d(right.0, right.1));
        } else {
            res.points_mut().push(polygon.points()[i]);
        }
    }
    res
}

/// Point.hpp:92-96 `cross2` of two double vectors: `a.x*b.y - a.y*b.x`, which
/// clang contracts to one fused multiply-add.
#[inline]
fn cross2_f64(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0.mul_add(b.1, -(a.1 * b.0))
}

/// Point.hpp:92-96 `cross2` of two float vectors, in float, contracted.
#[inline]
fn cross2_f32(a: Vec2f, b: Vec2f) -> f32 {
    a.x.mul_add(b.y, -(a.y * b.x))
}

/// WipeTower.cpp:323-327 `scale_polygon`.
pub fn scale_polygon(points: &[Vec2f]) -> Polygon {
    let mut res = Polygon::new();
    for p in points {
        res.points_mut().push(scaled_vec2f_pt(*p));
    }
    res
}

/// WipeTower.cpp:328-333 `unscale_polygon`.
pub fn unscale_polygon(polygon: &Polygon) -> Vec<Vec2f> {
    polygon.points().iter().map(unscaled_f).collect()
}

/// WipeTower.cpp:364-372 `struct Segment` (the arc data of a segment is kept
/// by the writer's own segment lists; here only the straight part).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    pub start: Vec2f,
    pub end: Vec2f,
}

impl Segment {
    pub fn new(s: Vec2f, e: Vec2f) -> Self {
        Self { start: s, end: e }
    }
    /// WipeTower.cpp:371
    pub fn is_valid(&self) -> bool {
        self.start.y < self.end.y
    }
}

/// WipeTower.cpp:374-396 `remove_points_from_segment`.
pub fn remove_points_from_segment(segment: &Segment, skip_points: &[Vec2f], range: f64) -> Vec<Segment> {
    let mut result: Vec<Segment> = vec![*segment];
    let x = segment.start.x;
    for point in skip_points {
        let mut new_result: Vec<Segment> = Vec::new();
        for seg in &result {
            // float + double comparisons are done in double
            if point.y as f64 + range <= seg.start.y as f64 || point.y as f64 - range >= seg.end.y as f64 {
                new_result.push(*seg);
            } else {
                if point.y as f64 - range > seg.start.y as f64 {
                    new_result.push(Segment::new(Vec2f::new(x, seg.start.y), Vec2f::new(x, (point.y as f64 - range) as f32)));
                }
                if point.y as f64 + range < seg.end.y as f64 {
                    new_result.push(Segment::new(Vec2f::new(x, (point.y as f64 + range) as f32), Vec2f::new(x, seg.end.y)));
                }
            }
        }
        result = new_result;
    }
    result.retain(|seg| seg.is_valid());
    result
}

/// WipeTower.cpp:4660-4685 `polylines_extreme_at`: the outermost crossing of
/// the polylines with the cut `along` (of the wall pair `across_is_x`).
pub fn polylines_extreme_at(pls: &[Polyline], across_is_x: bool, positive_side: bool, along: f32, out_across: &mut f32) -> bool {
    let pick = |v: Vec2f, i: usize| if i == 0 { v.x } else { v.y };
    let i_across = if across_is_x { 0 } else { 1 };
    let i_along = if across_is_x { 1 } else { 0 };
    let mut found = false;
    let mut extreme = if positive_side { -f32::MAX } else { f32::MAX };
    for pl in pls {
        let pts = &pl.points;
        let mut i = 0usize;
        while i + 1 < pts.len() {
            let a = unscaled_f(&pts[i]);
            let b = unscaled_f(&pts[i + 1]);
            i += 1;
            // segment does not straddle the cut
            if (pick(a, i_along) - along) * (pick(b, i_along) - along) > 0.0 {
                continue;
            }
            // segment parallel to the cut, ignore
            if (pick(b, i_along) - pick(a, i_along)).abs() < crate::libslic3r::EPSILON as f32 {
                continue;
            }
            let t = (along - pick(a, i_along)) / (pick(b, i_along) - pick(a, i_along));
            // a + t * (b - a): one contracted multiply-add.
            let c = t.mul_add(pick(b, i_across) - pick(a, i_across), pick(a, i_across));
            // std::max / std::min: (a < b) ? b : a / (b < a) ? b : a
            extreme = if positive_side {
                if extreme < c { c } else { extreme }
            } else if c < extreme {
                c
            } else {
                extreme
            };
            found = true;
        }
    }
    if found {
        *out_across = extreme;
    }
    found
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
    /// WipeTower.hpp:420 `int block_key{0}`: packed (adhesiveness category,
    /// nozzle diameter, lane) key of the band (`encode_block_key`).
    pub block_key: i32,
    /// WipeTower.hpp:424 `std::vector<WipeTowerLayerType> layers_type`
    pub layers_type: Vec<WipeTowerLayerType>,
    /// WipeTower.hpp:425 `std::vector<float> layer_heights`
    pub layer_heights: Vec<f32>,
    /// WipeTower.hpp:426 `std::vector<char> layer_reserved`
    pub layer_reserved: Vec<i8>,
    /// WipeTower.hpp:429 `std::vector<int> layer_fill_filament`
    pub layer_fill_filament: Vec<i32>,
}

impl WipeTowerBlock {
    /// WipeTower.hpp:415-435 — a value-initialised `WipeTowerBlock` with the
    /// C++ member initialisers (`finish_depth{0}` is a one-element vector,
    /// `last_filament_change_id{-1}`, `last_nozzle_change_id{-1}`).
    pub fn new_cpp() -> Self {
        Self {
            finish_depth: vec![0.0],
            last_filament_change_id: -1,
            last_nozzle_change_id: -1,
            ..Default::default()
        }
    }
}

/// WipeTower.hpp:413 `enum class WipeTowerLayerType : unsigned char { Normal,
/// Contact, Solid, Contact_UP }`. A contact layer is solid and printed slower.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WipeTowerLayerType {
    #[default]
    Normal,
    Contact,
    Solid,
    ContactUp,
}

/// WipeTower.hpp:527-533 `struct MergeLadderGroup`: one step of a merge
/// ladder, the printed steps `[begin_step, top_step]` printing together at
/// `top_step`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MergeLadderGroup {
    pub begin_step: i32,
    pub top_step: i32,
    pub filament_id: i32,
    pub height: f32,
}

impl Default for MergeLadderGroup {
    fn default() -> Self {
        Self { begin_step: 0, top_step: 0, filament_id: -1, height: 0.0 }
    }
}

/// WipeTower.hpp:619-624 `struct SideScrapeZ`: the vertical extent of one
/// side scrape (see `ToolChangeResult::side_scrape_z_low`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SideScrapeZ {
    pub z_low: f32,
    pub depth: f32,
    pub min_height_above_base: f32,
}

/// Depth information for a block. C++ `BlockDepthInfo{}` has `category{-1}`
/// (WipeTower.hpp:439); `Default` here gives 0, `BlockDepthInfo::new_cpp()` -1.
#[derive(Debug, Clone, Default)]
pub struct BlockDepthInfo {
    /// Category
    pub category: i32,
    /// Depth
    pub depth: f32,
    /// Nozzle change depth
    pub nozzle_change_depth: f32,
}

impl BlockDepthInfo {
    /// WipeTower.hpp:437-442 with its member initialisers.
    pub fn new_cpp() -> Self {
        Self { category: -1, depth: 0.0, nozzle_change_depth: 0.0 }
    }
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
    /// Smooth timelapse is on (C++ `m_enable_timelapse_print`), as
    /// `update_all_layer_depth` reads it to set every layer's depth to the
    /// tower depth (WipeTower.cpp:5541-5546). `enable_timelapse_print` above
    /// is the opt-in that changes this port's plan; this one only tells
    /// `layer_info_depth` what C++ reads.
    pub timelapse_smooth: bool,
    /// `only_generate_wall` of `generate_new` (WipeTower.cpp:7818): under
    /// timelapse the outer wall is a result of its own at the head of the layer
    /// and the finish of the layer draws none.
    pub only_generate_wall: bool,
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
    /// `m_enable_arc_fitting` (WipeTower.cpp, from `enable_arc_fitting`).
    pub enable_arc_fitting: bool,
    /// Per-filament adhesiveness category (`filament_adhesiveness_category`).
    /// WipeTower.cpp:1850 — `m_filpar[idx].category = config...get_at(idx)`.
    /// One tower BLOCK is created per DISTINCT value. R617.
    pub filament_categories: Vec<i32>,
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
            timelapse_smooth: false,
            only_generate_wall: false,
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
            enable_arc_fitting: false,
            filament_categories: vec![],
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

/// R808: `TOWER_WRITER_CPP=0` restores the pre-R808 always-`X Y E` writer lines
/// (and separate `G1 F` speed lines).
/// R808 purge-row port gate (see `toolchange_wipe`).
fn tower_wipe_cpp() -> bool {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *G.get_or_init(|| crate::faithful_gate("TOWER_WIPE_CPP"))
}

fn tower_writer_cpp() -> bool {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *G.get_or_init(|| crate::faithful_gate("TOWER_WRITER_CPP"))
}

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
    /// R808: per-filament area / max E speed for the faithful emitter
    /// (WipeTower.cpp:748 width, :776 flow limit).
    filpar_area: Vec<f32>,
    filpar_max_e_speed: Vec<f32>,
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
    /// `m_enable_arc_fitting` of the C++ writer (its constructor argument).
    enable_arc_fitting: bool,
    /// Last acceleration value
    last_acceleration: u32,
    /// Filament map
    filament_map: Vec<i32>,
    /// Accel-to-decel enable
    accel_to_decel_enable: bool,
    /// Accel-to-decel factor
    accel_to_decel_factor: f32,
    /// The writer behaves as the C++ one in every method (made by `new_cpp`,
    /// the faithful generator); the reduced pipeline's writers (`new`) keep
    /// this port's own forms.
    cpp_raw: bool,
    /// WipeTower.cpp:1590 `m_line_width`
    line_width: f32,
    /// WipeTower.cpp:1578 `m_layer_id = -1`
    layer_id: i32,
    /// WipeTower.cpp:1579 `m_handle_hotend_as_extruder = false`
    handle_hotend_as_extruder: bool,
    /// WipeTower.cpp:1577 `m_multi_nozzle_group_result{nullptr}`
    multi_nozzle_group_result: Option<std::sync::Arc<crate::multi_nozzle_utils::LayeredNozzleGroupResult>>,
    /// WipeTower.cpp:1580 `m_physical_extruder_map`
    physical_extruder_map: Vec<i32>,
    /// `m_filpar[tool].max_e_ramming_speed` (WipeTower.cpp:875-876).
    filpar_max_e_ramming_speed: Vec<(f32, f32)>,
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
            filpar_area: filament_parameters.iter().map(|p| p.filament_area).collect(),
            filpar_max_e_speed: filament_parameters.iter().map(|p| p.max_e_speed).collect(),
            gcode_flavor,
            is_first_layer: false,
            normal_accelerations: vec![],
            first_layer_normal_accelerations: vec![],
            travel_accelerations: vec![],
            first_layer_travel_accelerations: vec![],
            max_acceleration: 0,
            enable_arc_fitting: false,
            last_acceleration: 0,
            filament_map: vec![],
            accel_to_decel_enable: false,
            accel_to_decel_factor: 0.5,
            cpp_raw: false,
            line_width: perimeter_width,
            layer_id: -1,
            handle_hotend_as_extruder: false,
            multi_nozzle_group_result: None,
            physical_extruder_map: vec![],
            filpar_max_e_ramming_speed: filament_parameters.iter().map(|p| p.max_e_ramming_speed).collect(),
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

    /// Install the acceleration lists.
    // WipeTower.cpp:1356-1360 — the public setters, called from
    // `set_for_wipe_tower_writer` (:2663-2667). C++ overloads the names (a
    // vector-taking public setter and a no-arg private emitter); Rust cannot
    // overload, so the emitters keep the C++ name and these carry a `_values`
    // suffix. R615: without these the lists stayed empty and every emitter
    // early-returned.
    pub fn set_normal_acceleration_values(&mut self, accelerations: Vec<u32>) {
        self.normal_accelerations = accelerations;
    }

    // WipeTower.cpp:1357
    pub fn set_first_layer_normal_acceleration_values(&mut self, accelerations: Vec<u32>) {
        self.first_layer_normal_accelerations = accelerations;
    }

    // WipeTower.cpp:1358
    pub fn set_travel_acceleration_values(&mut self, accelerations: Vec<u32>) {
        self.travel_accelerations = accelerations;
    }

    // WipeTower.cpp:1359
    pub fn set_first_layer_travel_acceleration_values(&mut self, accelerations: Vec<u32>) {
        self.first_layer_travel_accelerations = accelerations;
    }

    // WipeTower.cpp:1360
    pub fn set_max_acceleration(&mut self, acceleration: u32) {
        self.max_acceleration = acceleration;
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

    /// Travel to position, writing the feedrate ON the move line.
    // WipeTower.cpp:884 — `travel(const Vec2f &dest, float f = 0.f)` forwards to
    // extrude_explicit(dest.x, dest.y, 0.f, f, ...), which appends set_format_F(f)
    // to the same `G1`. R626: `generate_path`'s gap travels were emitting a
    // standalone `G1 F600` line AND a bare `G1 X.. Y..` — two lines where C++
    // writes one, measured as 5,011 standalone `G1 F600` against C++'s 2,485.
    pub fn travel_to_f(&mut self, target: Vec2f, f: f32) -> &mut Self {
        if self.cpp_raw {
            return self.travel_xy_f(target.x, target.y, f);
        }
        if tower_writer_cpp() {
            return self.extrude_explicit_f(target.x, target.y, 0.0, f, false, true);
        }
        if f <= 0.0 || f == self.current_feedrate {
            // Nothing to fold in; C++'s set_format_F is skipped on f == 0 and the
            // move is identical to a plain travel.
            return self.travel_to(target);
        }
        let rotated = self.rotate(target);
        self.gcode.push_str(&format!(
            "G1 X{:.3} Y{:.3} F{:.0}\n",
            rotated.x, rotated.y, f
        ));
        self.current_feedrate = f;

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

    /// Travel to position
    pub fn travel_to(&mut self, target: Vec2f) -> &mut Self {
        if self.cpp_raw {
            return self.travel_xy_f(target.x, target.y, 0.0);
        }
        if tower_writer_cpp() {
            return self.extrude_explicit_f(target.x, target.y, 0.0, 0.0, false, true);
        }
        // WipeTower.cpp:766-771 — gcode uses rot.x()/rot.y() directly; the
        // y_shift is already folded into rotate() (WipeTower.cpp:1495), so it
        // must NOT be added again here.
        let rotated = self.rotate(target);
        // WipeTower.cpp:760-763 — the acceleration is emitted immediately before
        // the move, chosen by whether the move extrudes. C++ routes travels
        // through `extrude_explicit(.., e = 0)`, so a travel always takes the
        // travel branch; our travel writer is separate, so it takes it directly.
        if crate::faithful_gate("TOWER_ACCEL_CPP") {
            self.set_travel_acceleration();
        }
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
        if self.cpp_raw {
            return self.extrude_xy_f(x, y, 0.0, LimitFlow::LimitPrintFlow);
        }
        if tower_writer_cpp() {
            return self.extrude_f(x, y, 0.0);
        }
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
        if self.cpp_raw {
            let lf = if _limit_flow { LimitFlow::LimitPrintFlow } else { LimitFlow::None };
            return self.extrude_explicit_lf(x, y, e, 0.0, false, lf);
        }
        if tower_writer_cpp() {
            return self.extrude_explicit_f(x, y, e, 0.0, false, _limit_flow);
        }
        let target = Vec2f::new(x, y);
        let rotated = self.rotate(target);

        let dx = x - self.current_pos.x;
        let dy = y - self.current_pos.y;
        let len = (dx * dx + dy * dy).sqrt();

        // WipeTower.cpp:760-763 — same acceleration prologue as the travel writer,
        // but here the branch is live: e == 0 takes the travel acceleration, any
        // other value takes the normal (printing) one.
        if crate::faithful_gate("TOWER_ACCEL_CPP") {
            if e == 0.0 {
                self.set_travel_acceleration();
            } else {
                self.set_normal_acceleration();
            }
        }
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

    /// WipeTower.cpp:730-797 `extrude_explicit(x, y, e, f, record_length,
    /// limit_flow)` — the ONE emitter every native travel/extrude goes through:
    /// an axis only when it changes (compared AFTER rotation), E only when
    /// non-zero, F inline as `%d` only when it changes (flow-limited by
    /// max_e_speed), and the feedrate register updated. The post-transform
    /// (GCode.cpp:1096) re-inserts X/Y, so rust's always-`X Y E` lines came
    /// out as "G1  X..   E.." against native "G1  X..  E..", and its separate
    /// `G1 F` lines had no native twin (Majora: 0 of 67k tower extrusion
    /// lines matched @R807). R808.
    pub fn extrude_explicit_f(
        &mut self,
        x: f32,
        y: f32,
        e: f32,
        f: f32,
        record_length: bool,
        limit_flow: bool,
    ) -> &mut Self {
        if self.cpp_raw {
            let lf = if limit_flow { LimitFlow::LimitPrintFlow } else { LimitFlow::None };
            return self.extrude_explicit_lf(x, y, e, f, record_length, lf);
        }
        if (x - self.current_pos.x).abs() <= WT_EPSILON
            && (y - self.current_pos.y).abs() < WT_EPSILON
            && e == 0.0
            && (f == 0.0 || f == self.current_feedrate)
        {
            return self;
        }
        let dx = x - self.current_pos.x;
        let dy = y - self.current_pos.y;
        let len = (dx * dx + dy * dy).sqrt();
        if record_length {
            self.used_filament_length += e;
        }
        let rotated_current = self.pos_rotated();
        let target = Vec2f::new(x, y);
        let rot = self.rotate(target);
        if !self.preview_suppressed {
            if e > 0.0 && len > 0.0 {
                let fa = self
                    .filpar_area
                    .get(self.current_tool)
                    .copied()
                    .unwrap_or(PI * 0.875 * 0.875);
                let mut width = e * fa / (len * self.layer_height);
                width += self.layer_height * (1.0 - PI / 4.0);
                self.extrusions
                    .push(Extrusion::new(target, width, self.current_tool));
            } else if e == 0.0 {
                self.extrusions
                    .push(Extrusion::new(target, 0.0, self.current_tool));
            }
        }
        if crate::faithful_gate("TOWER_ACCEL_CPP") {
            if e == 0.0 {
                self.set_travel_acceleration();
            } else {
                self.set_normal_acceleration();
            }
        }
        let mut line = String::from("G1");
        if (rot.x - rotated_current.x).abs() > WT_EPSILON {
            line.push_str(&format!(" X{:.3}", rot.x));
        }
        if (rot.y - rotated_current.y).abs() > WT_EPSILON {
            line.push_str(&format!(" Y{:.3}", rot.y));
        }
        if e != 0.0 {
            line.push_str(&format!(" E{:.4}", e));
        }
        let mut f = f;
        if f != 0.0 && f != self.current_feedrate {
            if limit_flow {
                let denom = if len == 0.0 { e.abs() } else { len };
                let e_speed = e / (denom / f * 60.0);
                let tmp = self
                    .filpar_max_e_speed
                    .get(self.current_tool)
                    .copied()
                    .unwrap_or(f32::MAX);
                f /= (e_speed / tmp).max(1.0);
            }
            line.push_str(&format!(" F{}", (f + 0.5).floor() as i64));
            self.current_feedrate = f;
        }
        self.current_pos = target;
        if self.current_feedrate > 0.0 {
            let denom = if len == 0.0 { e.abs() } else { len };
            self.elapsed_time += denom / self.current_feedrate * 60.0;
        }
        line.push('\n');
        self.gcode.push_str(&line);
        self
    }

    /// WipeTower.cpp:888 `extrude(x, y, f)` — E from the extrusion flow, F inline.
    pub fn extrude_f(&mut self, x: f32, y: f32, f: f32) -> &mut Self {
        let dx = x - self.current_pos.x;
        let dy = y - self.current_pos.y;
        let e = (dx * dx + dy * dy).sqrt() * self.extrusion_flow;
        self.extrude_explicit_f(x, y, e, f, false, true)
    }

    /// WipeTower.cpp:881 `travel(x, y, f)`.
    pub fn travel_f(&mut self, x: f32, y: f32, f: f32) -> &mut Self {
        self.extrude_explicit_f(x, y, 0.0, f, false, true)
    }


    /// Extrude a rectangle
    // WipeTower.cpp:1000-1006 — rectangle(box) -> rectangle(box.ld, w, h, f)
    // with w = ru.x - lu.x, h = ru.y - rd.y.
    // WipeTower.cpp:902-925 — the rectangle(ld, width, height) implementation.
    /// Extrude one fitted arc as `G2`/`G3`.
    // WipeTower.cpp:798-872 (extrude_arc_explicit) reached via :894 extrude_arc.
    // The centre is emitted as an I/J OFFSET from the current position, both
    // rotated into tower space, and E is the arc's own length times the
    // extrusion flow — not the chord's. R628.
    //
    // The LimitFlow branch (:855-864) is omitted: it only fires for ramming
    // (LimitRammingFlow / LimitRammingFlowNC), and generate_path calls
    // extrude_arc with the default LimitPrintFlow on a wall, where
    // `max_e_speed` is not the binding constraint. Stated rather than assumed.
    pub fn extrude_arc(&mut self, arc: &crate::circle::ArcSegment, f: f32) -> &mut Self {
        if self.cpp_raw {
            return self.extrude_arc_explicit(arc, f, false, LimitFlow::LimitPrintFlow);
        }
        let x = crate::unscale(arc.end_point.x) as f32;
        let y = crate::unscale(arc.end_point.y) as f32;
        let len = crate::unscale(arc.length as i64) as f32;
        let e = len * self.extrusion_flow;
        // :804-806 — neither an extrusion nor a travel.
        if len < WT_EPSILON && e == 0.0 && (f == 0.0 || f == self.current_feedrate) {
            return self;
        }

        let rotated_current_pos = self.pos_rotated();
        let rot = self.rotate(Vec2f::new(x, y));

        // :845 — CCW is G3, CW is G2.
        let code = if arc.direction == crate::circle::ArcDirection::CounterClockwise {
            "G3"
        } else {
            "G2"
        };
        // :846 — the centre offset is relative to the CURRENT position.
        let centre = self.rotate(Vec2f::new(
            crate::unscale(arc.circle.center.x) as f32,
            crate::unscale(arc.circle.center.y) as f32,
        ));
        let i = centre.x - rotated_current_pos.x;
        let j = centre.y - rotated_current_pos.y;

        self.gcode.push_str(&format!(
            "{} X{:.3} Y{:.3} I{:.3} J{:.3}",
            code, rot.x, rot.y, i, j
        ));
        // :853
        if e != 0.0 {
            self.gcode.push_str(&format!(" E{:.4}", e));
        }
        // :854-865
        if f != 0.0 && f != self.current_feedrate {
            if tower_writer_cpp() {
                // set_format_F: " F%d" of floor(f+0.5), and the register moves.
                self.gcode.push_str(&format!(" F{}", (f + 0.5).floor() as i64));
                self.current_feedrate = f;
            } else {
                if tower_writer_cpp() {
                    // set_format_F: " F%d" of floor(f+0.5), and the register moves.
                    self.gcode.push_str(&format!(" F{}", (f + 0.5).floor() as i64));
                    self.current_feedrate = f;
                } else {
                    self.gcode.push_str(&format!(" F{:.0}", f));
                }
            }
            self.current_feedrate = f;
        }
        self.gcode.push('\n');

        if !self.preview_suppressed && e > 0.0 && len > 0.0 {
            self.extrusions.push(Extrusion::new(
                Vec2f::new(x, y),
                self.default_analyzer_line_width,
                self.current_tool,
            ));
        }

        // :866-870
        self.current_pos = Vec2f::new(x, y);
        if self.current_feedrate > 0.0 {
            self.elapsed_time += len / self.current_feedrate * 60.0;
        }
        self.used_filament_length += e;
        self
    }

    /// Emit a set of polyline runs, retracting across the gaps between them.
    // WipeTower.cpp:1249-1309 (WipeTowerWriter::generate_path) — LINEAR branch.
    //
    // C++ branches on `m_enable_arc_fitting` and Majora sets it, so the real
    // path runs `simplify_by_fitting_arc` and can emit G2/G3. That is NOT ported
    // here: R622 measured C++ emitting 2,719 tower arcs against our 0, and doing
    // the retract triple without the arcs would trade one divergence for another
    // (R622's rule). This port takes the segments straight from the polyline
    // points, which is what C++'s else-branch (`simplify` + `reset_to_linear_move`)
    // produces; the arc residual is left to its own round and is stated as an
    // open item rather than hidden.
    //
    // The important part is :1294-1308: the START segment is chosen by PROXIMITY
    // to the current position (`get_closet_idx`), not by list order, and every
    // subsequent segment whose start is away from the pen gets
    //     retract(retract_length, retract_speed)
    //     travel(start, 600.)
    //     retract(-retract_length, retract_speed)
    // before it is extruded. That triple is the gap-wall pattern R620 measured as
    // entirely absent from our output.
    pub fn generate_path(
        &mut self,
        pls: &[crate::geometry::Polyline],
        feedrate: f32,
        retract_length: f32,
        retract_speed: f32,
    ) -> &mut Self {
        if self.cpp_raw {
            let mut pls = pls.to_vec();
            self.generate_path_cpp(&mut pls, feedrate, retract_length, retract_speed, false);
            return self;
        }
        // :1265-1272 — fit arcs (or flatten) BEFORE building the segment list.
        // Majora sets "enable_arc_fitting": "1", and C++ emits 2,719 G2/G3 inside
        // tower toolchange blocks from the fillet-rounded wall
        // ("prime_tower_fillet_wall": "1") against our 0 before this (R622/R628).
        // WIPE_TOWER_RESOLUTION is 0.1mm (:20); C++ scales it by its own
        // SCALING_FACTOR, so the scaled value is expressed in OUR units here
        // rather than copied as a literal (the two codebases scale by 1e6 and
        // 1e5 respectively — R596/R600's units warning).
        let mut fitted: Vec<crate::geometry::Polyline> = pls.to_vec();
        // OPT-IN, not default-on. C++ does call simplify_by_fitting_arc here
        // (:1266), so enabling it is the faithful shape — but measured on Majora
        // it produces ZERO arcs and removes 1,184 lines for zero matched gain,
        // because the wall is a plain rectangle: `rib_wall` is 0, and C++'s own
        // comment at :5059 ("rectangle_wall do nothing") shows the fillet
        // rounding is skipped in that case. R628 also established that C++'s
        // 2,719 tower "arcs" are NOT wall arcs at all — they are the spiral
        // Z-lift `G3 Z.7 I1.217 J0 P1` in the toolchange gcode, one per tool
        // change. Left opt-in until there is a fixture where it pays.
        // TOWER_ARC_FITTING_CPP (on, R838): what was measured above was with
        // the gaps of the skip points missing (R838's float scaling); with
        // them the simplification is what makes the wall the oracle's (it
        // drops `add_extra_point`'s collinear points). Arc fitting when
        // `enable_arc_fitting`, else `simplify` (:1265-1272). =0 neither.
        let arc_fitting = crate::faithful_gate("TOWER_ARC_FITTING_CPP");
        // The WHOLE simplification step is gated, not just the arc branch: C++
        // runs one or the other unconditionally (:1265-1272), but measured on
        // Majora EITHER branch removes lines for zero matched gain — the arc
        // branch found no arcs (the wall is a plain rectangle) and the linear
        // branch merely drops collinear points C++ apparently keeps. Until the
        // tolerance is understood (the two codebases scale by 1e6 and 1e5, so
        // `SCALED_WIPE_TOWER_RESOLUTION` is not transferable as a literal —
        // R596/R600), running neither reproduces R627's verified output.
        if arc_fitting && self.enable_arc_fitting {
            for pl in &mut fitted {
                pl.simplify_by_fitting_arc(crate::scaled(0.1) as f64);
            }
        } else if arc_fitting {
            for pl in &mut fitted {
                pl.simplify(crate::scaled(0.1) as f64);
            }
        }

        // :1274-1289 — flatten into segments, honouring each fitting_result span:
        // a Linear_move contributes one segment per point pair, an arc contributes
        // ONE segment carrying its arc data.
        let mut segments: Vec<(Vec2f, Vec2f, Option<crate::circle::ArcSegment>)> = Vec::new();
        // `unscaled<float>(p)`: float coordinate times the float scaling factor.
        let to_v = |p: &crate::geometry::Point| {
            if unscale_mul_gate() {
                unscaled_f(p)
            } else {
                Vec2f::new(crate::unscale(p.x) as f32, crate::unscale(p.y) as f32)
            }
        };
        for pl in &fitted {
            if pl.points.len() < 2 {
                continue;
            }
            if pl.fitting_result.is_empty() {
                for w in pl.points.windows(2) {
                    segments.push((to_v(&w[0]), to_v(&w[1]), None));
                }
                continue;
            }
            for fr in &pl.fitting_result {
                if fr.path_type == crate::arc_fitter::EMovePathType::LinearMove {
                    for k in fr.start_point_index..fr.end_point_index {
                        segments.push((to_v(&pl.points[k]), to_v(&pl.points[k + 1]), None));
                    }
                } else {
                    let beg = fr.start_point_index;
                    let end = fr.end_point_index;
                    segments.push((
                        to_v(&pl.points[beg]),
                        to_v(&pl.points[end]),
                        Some(fr.arc_data.clone()),
                    ));
                }
            }
        }
        // :1290-1291
        if segments.is_empty() {
            return self;
        }

        // :1251-1264 (get_closet_idx) — nearest segment START to the pen.
        let anchor = self.current_pos;
        let mut index_of_closest = 0usize;
        let mut min_d = f32::MAX;
        for (i, seg) in segments.iter().enumerate() {
            let dx = seg.0.x - anchor.x;
            let dy = seg.0.y - anchor.y;
            let d = dx * dx + dy * dy;
            if d < min_d {
                min_d = d;
                index_of_closest = i;
            }
        }

        // :1293-1295 — travel to it and lay the first segment.
        // TOWER_PATH_F_INLINE (on): `extrude(segments[i].end, feedrate)` writes
        // the feedrate on the move (:1295, :1307). =0 writes a `G1 F` line
        // before it.
        let f_inline = crate::faithful_gate("TOWER_PATH_F_INLINE");
        self.travel_to(segments[index_of_closest].0);
        if !f_inline {
            self.feedrate(feedrate);
        }
        match segments[index_of_closest].2.clone() {
            Some(arc) => {
                self.extrude_arc(&arc, feedrate);
            }
            None if f_inline => {
                self.extrude_f(segments[index_of_closest].1.x, segments[index_of_closest].1.y, feedrate);
            }
            None => {
                self.extrude(segments[index_of_closest].1.x, segments[index_of_closest].1.y);
            }
        }

        // :1296-1308 — walk the ring; retract across any real jump.
        let n = segments.len();
        let mut i = index_of_closest;
        loop {
            i = (i + 1) % n;
            if i == index_of_closest {
                break;
            }
            let dx = segments[i].0.x - self.current_pos.x;
            let dy = segments[i].0.y - self.current_pos.y;
            let len = (dx * dx + dy * dy).sqrt();
            if len > WT_EPSILON {
                self.retract(retract_length, retract_speed);
                // :1304 — `travel(segments[i].start, 600.)`, feedrate INLINE on
                // the move, not a separate `G1 F600` line (R626).
                self.travel_to_f(segments[i].0, 600.0);
                self.retract(-retract_length, retract_speed);
            }
            if !f_inline {
                self.feedrate(feedrate);
            }
            // :1307 — `segments[i].is_arc ? extrude_arc(...) : extrude(...)`.
            match segments[i].2.clone() {
                Some(arc) => {
                    self.extrude_arc(&arc, feedrate);
                }
                None if f_inline => {
                    self.extrude_f(segments[i].1.x, segments[i].1.y, feedrate);
                }
                None => {
                    self.extrude(segments[i].1.x, segments[i].1.y);
                }
            }
        }
        self
    }

    pub fn rectangle(&mut self, box_coords: &BoxCoordinates) -> &mut Self {
        if self.cpp_raw {
            return self.rectangle_f(box_coords, 0.0);
        }
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

    /// WipeTower.cpp:1138-1190 `polygon(const Polygon &wall_polygon, const float f)`:
    /// the closed polygon as segments, begun at the segment whose start is
    /// nearest the pen (a travel to it), each segment extruded with `f`.
    ///
    /// The simplification at :1140-1147 (`simplify(WT_SIMPLIFY_TOLERANCE_SCALED)`,
    /// then arc fitting or another `simplify`) is left out as `generate_path`
    /// leaves it out (R628); the polygons drawn here are offsets of the
    /// rectangular wall, on which it changes nothing.
    pub fn polygon(&mut self, wall_polygon: &crate::geometry::Polygon, f: f32) -> &mut Self {
        if self.cpp_raw {
            return self.polygon_f(wall_polygon, f);
        }
        let pl = to_polyline(wall_polygon);
        // unscaled<float>(p): the coordinate as float times float(SCALING_FACTOR).
        let sf = crate::libslic3r::SCALING_FACTOR as f32;
        let to_v = |p: &crate::geometry::Point| Vec2f::new(p.x as f32 * sf, p.y as f32 * sf);
        let segments: Vec<(Vec2f, Vec2f)> = pl
            .points
            .windows(2)
            .map(|w| (to_v(&w[0]), to_v(&w[1])))
            .collect();
        // :1176-1177
        if segments.is_empty() {
            return self;
        }
        // :1149-1161 get_closet_idx — nearest segment start to the pen.
        let anchor = self.current_pos;
        let mut index_of_closest = 0usize;
        let mut min_distance = f32::MAX;
        for (i, seg) in segments.iter().enumerate() {
            let dx = seg.0.x - anchor.x;
            let dy = seg.0.y - anchor.y;
            let distance = dx * dx + dy * dy;
            if distance < min_distance {
                min_distance = distance;
                index_of_closest = i;
            }
        }
        // :1179-1188
        let mut i = index_of_closest;
        self.travel_to(segments[i].0);
        self.extrude_f(segments[i].1.x, segments[i].1.y, f);
        loop {
            i = (i + 1) % segments.len();
            if i == index_of_closest {
                break;
            }
            self.extrude_f(segments[i].1.x, segments[i].1.y, f);
        }
        self
    }


    /// WipeTower.cpp:1053-1085 and 1111-1130 `rectangle_fill_box(wipe_tower,
    /// fill_box, finish_rect_wipe_path, f)`: the rectangle around a fill box,
    /// walked from the nearest corner with `f` on its moves, or a single line
    /// when its sides would overlap; the wipe from where it ends to the next
    /// corner is put into `finish_rect_wipe_path`. `line_width` is the
    /// writer's own line width (`m_line_width`).
    ///
    /// `need_thick_bridge_flow` (:1055, 1088) is false for slabs of 0.2 mm and
    /// more (WipeTower.cpp:8526-8530); below that it asks the floating area of
    /// the block, which this port does not keep, so the bridge flow is not
    /// written.
    pub fn rectangle_fill_box_wipe(
        &mut self,
        fill_box: &BoxCoordinates,
        finish_rect_wipe_path: &mut Vec<Vec2f>,
        f: f32,
        line_width: f32,
    ) -> &mut Self {
        let width = fill_box.rd.x - fill_box.ld.x;
        let height = fill_box.ru.y - fill_box.rd.y;
        if height > line_width - WT_EPSILON {
            let ld = fill_box.ld;
            let corners = [
                ld,
                ld + Vec2f::new(width, 0.0),
                ld + Vec2f::new(width, height),
                ld + Vec2f::new(0.0, height),
            ];
            let mut index_of_closest = 0usize;
            if self.x() - ld.x > ld.x + width - self.x() {
                index_of_closest = 1;
            }
            if self.y() - ld.y > ld.y + height - self.y() {
                index_of_closest = if index_of_closest == 0 { 3 } else { 2 };
            }
            let cy = self.y();
            self.travel(corners[index_of_closest].x, cy);
            let cx = self.x();
            self.travel(cx, corners[index_of_closest].y);
            let mut i = index_of_closest;
            loop {
                i += 1;
                if i == 4 {
                    i = 0;
                }
                self.extrude_f(corners[i].x, corners[i].y, f);
                if i == index_of_closest {
                    break;
                }
            }
            let pos = self.pos();
            let target = if pos == fill_box.ld {
                fill_box.rd
            } else if pos == fill_box.rd {
                fill_box.ru
            } else if pos == fill_box.ru {
                fill_box.lu
            } else {
                fill_box.ld
            };
            finish_rect_wipe_path.push(pos);
            finish_rect_wipe_path.push(target);
        } else if height > WT_EPSILON {
            // :1086-1109 line(wipe_tower, ld, rd)
            let (mut p0, mut p1) = (fill_box.ld, fill_box.rd);
            if (self.x() - p0.x).abs() > (self.x() - p1.x).abs() {
                std::mem::swap(&mut p0, &mut p1);
            }
            let cy = self.y();
            self.travel(p0.x, cy);
            let cx = self.x();
            self.travel(cx, p0.y);
            self.extrude(p1.x, p1.y);
            let pos = self.pos();
            let target = if pos == fill_box.ld { fill_box.rd } else { fill_box.ld };
            finish_rect_wipe_path.push(pos);
            finish_rect_wipe_path.push(target);
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
        if self.cpp_raw {
            return self.load_cpp(e, f);
        }
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
        if self.cpp_raw {
            return self.z_hop_cpp(hop, f);
        }
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
        if self.cpp_raw {
            return self.wait_cpp(seconds);
        }
        if seconds == 0.0 {
            return self;
        }
        let s = super::writer::format_gcode_value(seconds as f64, 3);
        self.gcode.push_str(&format!("G4 S{}\n", s));
        self
    }

    /// Speed override
    // WipeTower.cpp:1149-1153 — `M220 S<percent>`.
    pub fn speed_override(&mut self, percent: i32) -> &mut Self {
        self.gcode.push_str(&format!("M220 S{}\n", percent));
        self
    }

    /// Let the firmware back up the active speed override value.
    // WipeTower.cpp:1156-1163 `speed_override_backup`. C++ guards this on
    // `gcfMarlinLegacy || gcfMarlinFirmware` (its "BBL machine don't support speed
    // backup" comment refers to the OTHER flavors); Majora's reference output does
    // contain `M220 B`, so this profile's flavor qualifies. R602 emits it
    // unconditionally and VERIFIES the count against C++ rather than plumbing a
    // flavor field for a fixture that only exercises one flavor.
    pub fn speed_override_backup(&mut self) -> &mut Self {
        if self.cpp_raw {
            return self.speed_override_backup_cpp();
        }
        self.gcode.push_str("M220 B\n");
        self
    }

    /// Let the firmware restore the active speed override value.
    // WipeTower.cpp:1165-1172 `speed_override_restore`.
    pub fn speed_override_restore(&mut self) -> &mut Self {
        if self.cpp_raw {
            return self.speed_override_restore_cpp();
        }
        self.gcode.push_str("M220 R\n");
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

    /// WipeTower.cpp `flush_planner_queue` — `G4 S0`, a zero dwell that drains
    /// the motion planner. R645.
    pub fn flush_planner_queue(&mut self) -> &mut Self {
        self.gcode.push_str("G4 S0\n");
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
        let acc = if let Some(g) = &self.multi_nozzle_group_result {
            // WipeTower.cpp:1519-1520
            accels[g.get_extruder_id(self.current_tool as i32, self.layer_id) as usize]
        } else if self.cpp_raw {
            // WipeTower.cpp:1517-1518 — no group result, no acceleration.
            return self;
        } else {
            accels[self.current_tool.min(accels.len() - 1)]
        };
        self.set_acceleration_impl(acc);
        self
    }

    /// Set travel acceleration
    // WipeTower.cpp:1523-1531
    pub fn set_travel_acceleration(&mut self) -> &mut Self {
        let accels = if self.is_first_layer {
            &self.first_layer_travel_accelerations
        } else {
            &self.travel_accelerations
        };
        if accels.is_empty() {
            return self;
        }
        let acc = if let Some(g) = &self.multi_nozzle_group_result {
            accels[g.get_extruder_id(self.current_tool as i32, self.layer_id) as usize]
        } else if self.cpp_raw {
            return self;
        } else {
            accels[self.current_tool.min(accels.len() - 1)]
        };
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

        if self.cpp_raw {
            // WipeTower.cpp:1542-1566
            match self.gcode_flavor {
                GCodeFlavor::Repetier => {
                    self.gcode.push_str(&format!("M201 X{} Y{}\nM202 X{} Y{}\n", acceleration, acceleration, acceleration, acceleration));
                }
                GCodeFlavor::RepRap | GCodeFlavor::MarlinFirmware => {
                    self.gcode.push_str(&format!("M204 P{}\n", acceleration));
                }
                GCodeFlavor::Klipper if self.accel_to_decel_enable => {
                    // `acceleration * m_accel_to_decel_factor / 100` is a float,
                    // streamed with the default 6 significant digits.
                    let v = acceleration as f32 * self.accel_to_decel_factor / 100.0;
                    self.gcode.push_str(&format!("SET_VELOCITY_LIMIT ACCEL_TO_DECEL={}\nM204 S{}\n", ostream_float(v as f64), acceleration));
                }
                _ => {
                    self.gcode.push_str(&format!("M204 S{}\n", acceleration));
                }
            }
            return;
        }
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
        if self.cpp_raw {
            // WipeTower.cpp:1641-1649: the float angle in double, double
            // cos / sin, each coordinate one contracted multiply-add.
            let px = pt.x - self.wipe_tower_width / 2.0;
            // `pt.y() += m_y_shift - m_wipe_tower_depth / 2.f`
            let py = pt.y + (self.y_shift - self.wipe_tower_depth / 2.0);
            let angle = (self.internal_angle * (std::f64::consts::PI / 180.0) as f32) as f64;
            let (s, c) = sincos_f64(angle);
            return Vec2f::new(
                (px as f64).mul_add(c, -(py as f64 * s)) as f32 + self.wipe_tower_width / 2.0,
                (px as f64).mul_add(s, py as f64 * c) as f32 + self.wipe_tower_depth / 2.0,
            );
        }
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

/// `std::to_string(float)`: the value promoted to double, `%f`.
fn std_to_string_f32(v: f32) -> String {
    format!("{:.6}", v as f64)
}

/// `Slic3r::float_to_string_decimal_point(v, precision)` (LocalesUtils.cpp:65-87,
/// the stringstream branch): fixed notation, `precision` decimals.
fn float_to_string_decimal_point(v: f64, precision: usize) -> String {
    format!("{:.*}", precision, v)
}

/// The writer's segment of a fitted polyline (WipeTower.cpp:364-372 `Segment`
/// with its arc).
#[derive(Debug, Clone)]
struct WriterSegment {
    start: Vec2f,
    end: Vec2f,
    arc: Option<crate::circle::ArcSegment>,
}

/// WipeTower.cpp:1163-1175 / 1404-1419: the segments of fitted polylines: one
/// per point pair of a linear span, one per arc.
fn fitted_segments(pls: &[Polyline], skip_short: bool) -> Vec<WriterSegment> {
    let mut segments: Vec<WriterSegment> = Vec::new();
    for pl in pls {
        if skip_short && pl.points.len() < 2 {
            continue;
        }
        for fr in &pl.fitting_result {
            if fr.path_type == crate::arc_fitter::EMovePathType::LinearMove {
                for j in fr.start_point_index..fr.end_point_index {
                    segments.push(WriterSegment { start: unscaled_f(&pl.points[j]), end: unscaled_f(&pl.points[j + 1]), arc: None });
                }
            } else {
                let beg = fr.start_point_index;
                let end = fr.end_point_index;
                segments.push(WriterSegment {
                    start: unscaled_f(&pl.points[beg]),
                    end: unscaled_f(&pl.points[end]),
                    arc: Some(fr.arc_data.clone()),
                });
            }
        }
    }
    segments
}

/// WipeTower.cpp:28 `SCALED_WIPE_TOWER_RESOLUTION` = `WIPE_TOWER_RESOLUTION /
/// SCALING_FACTOR` in double.
fn scaled_wipe_tower_resolution() -> f64 {
    0.1 / crate::libslic3r::SCALING_FACTOR
}

/// WipeTower.cpp:26 `WT_SIMPLIFY_TOLERANCE_SCALED` = `0.001 / SCALING_FACTOR`.
fn wt_simplify_tolerance_scaled() -> f64 {
    0.001 / crate::libslic3r::SCALING_FACTOR
}

impl WipeTowerWriter {
    /// WipeTower.cpp:698-720 — the C++ constructor: the analyzer header (layer
    /// height, role, line width), the position at `FLT_MAX`, no extrusion flow.
    /// The writer it makes behaves as the C++ one in every method
    /// (`cpp_raw`); `new` is the reduced pipeline's.
    pub fn new_cpp(
        layer_height: f32,
        line_width: f32,
        gcode_flavor: GCodeFlavor,
        filament_parameters: &[FilamentParameters],
        enable_arc_fitting: bool,
    ) -> Self {
        let mut w = Self::new(layer_height, line_width, gcode_flavor, filament_parameters);
        w.cpp_raw = true;
        w.current_pos = Vec2f::new(f32::MAX, f32::MAX);
        w.current_z = 0.0;
        w.current_feedrate = 0.0;
        w.extrusion_flow = 0.0;
        w.line_width = line_width;
        w.enable_arc_fitting = enable_arc_fitting;
        w.current_temp = -1;
        w.layer_id = -1;
        w.gcode.push_str(&format!(";{}{}\n", " LAYER_HEIGHT: ", std_to_string_f32(layer_height)));
        w.gcode.push_str(&format!(";{}{}\n", " FEATURE: ", "Prime tower"));
        w.change_analyzer_line_width(line_width);
        w
    }

    /// WipeTower.cpp:722-728
    pub fn change_analyzer_line_width(&mut self, line_width: f32) -> &mut Self {
        self.gcode.push_str(&format!(";{}{}\n", " LINE_WIDTH: ", std_to_string_f32(line_width)));
        self
    }

    /// WipeTower.cpp:730-736
    pub fn change_analyzer_layer_height(&mut self, layer_height: f32) -> &mut Self {
        self.gcode.push_str(&format!(";{}{}\n", " LAYER_HEIGHT: ", std_to_string_f32(layer_height)));
        self
    }

    /// WipeTower.cpp:750-757 `set_initial_position(pos, width, depth, internal_angle)`.
    pub fn set_initial_position_wd(&mut self, pos: Vec2f, width: f32, depth: f32, internal_angle: f32) -> &mut Self {
        self.wipe_tower_width = width;
        self.wipe_tower_depth = depth;
        self.internal_angle = internal_angle;
        self.start_pos = self.rotate(pos);
        self.current_pos = pos;
        self
    }

    /// `m_current_feedrate` (read by the native parity test).
    pub fn current_feedrate(&self) -> f32 {
        self.current_feedrate
    }

    /// WipeTower.cpp:767-768
    pub fn set_line_width(&mut self, line_width: f32) -> &mut Self {
        self.line_width = line_width;
        self
    }

    /// WipeTower.cpp:778 `y_shift()`.
    pub fn y_shift(&self) -> f32 {
        self.y_shift
    }

    /// WipeTower.cpp:819 `point_rotated`.
    pub fn point_rotated(&self, pt: Vec2f) -> Vec2f {
        self.rotate(pt)
    }

    /// The filament parameter of the current tool the C++ writer reads through
    /// its `m_filpar` reference.
    fn cur_filament_area(&self) -> f32 {
        self.filpar_area.get(self.current_tool).copied().unwrap_or(PI * 0.875 * 0.875)
    }

    /// WipeTower.cpp:824-889 `extrude_explicit(x, y, e, f, record_length,
    /// limit_flow)`, as C++ does it.
    pub fn extrude_explicit_lf(&mut self, x: f32, y: f32, e: f32, f: f32, record_length: bool, limit_flow: LimitFlow) -> &mut Self {
        let eps = crate::libslic3r::EPSILON as f32;
        if (x - self.current_pos.x).abs() <= eps
            && (y - self.current_pos.y).abs() < eps
            && e == 0.0
            && (f == 0.0 || f == self.current_feedrate)
        {
            return self;
        }
        let dx = x - self.current_pos.x;
        let dy = y - self.current_pos.y;
        // std::sqrt(dx*dx + dy*dy): one contracted multiply-add.
        let len = dx.mul_add(dx, dy * dy).sqrt();
        if record_length {
            self.used_filament_length += e;
        }
        let rotated_current_pos = self.pos_rotated();
        let rot = self.rotate(Vec2f::new(x, y));
        if !self.preview_suppressed && e > 0.0 && len > 0.0 {
            let mut width = e * self.cur_filament_area() / (len * self.layer_height);
            // `width += m_layer_height * float(1. - M_PI / 4.)`: contracted.
            width = self.layer_height.mul_add((1.0 - std::f64::consts::PI / 4.0) as f32, width);
            if self.extrusions.last().map_or(true, |l| l.pos != rotated_current_pos) {
                self.extrusions.push(Extrusion::new(rotated_current_pos, 0.0, self.current_tool));
            }
            self.extrusions.push(Extrusion::new(rot, width, self.current_tool));
        }
        if e == 0.0 {
            self.set_travel_acceleration();
        } else {
            self.set_normal_acceleration();
        }
        self.gcode.push_str("G1");
        if (rot.x - rotated_current_pos.x).abs() > eps {
            let s = self.set_format_x(rot.x);
            self.gcode.push_str(&s);
        }
        if (rot.y - rotated_current_pos.y).abs() > eps {
            let s = self.set_format_y(rot.y);
            self.gcode.push_str(&s);
        }
        if e != 0.0 {
            let s = Self::set_format_e(e);
            self.gcode.push_str(&s);
        }
        let mut f = f;
        if f != 0.0 && f != self.current_feedrate {
            if limit_flow != LimitFlow::None {
                let e_speed = e / ((if len == 0.0 { e.abs() } else { len }) / f * 60.0);
                let mut tmp = self.filpar_max_e_speed.get(self.current_tool).copied().unwrap_or(f32::MAX);
                if limit_flow == LimitFlow::LimitRammingFlow {
                    tmp = self.filpar_max_e_ramming_speed.get(self.current_tool).map_or(f32::MAX, |p| p.0);
                } else if limit_flow == LimitFlow::LimitRammingFlowNC {
                    tmp = self.filpar_max_e_ramming_speed.get(self.current_tool).map_or(f32::MAX, |p| p.1);
                }
                f /= 1.0_f32.max(e_speed / tmp);
            }
            let s = self.set_format_f(f);
            self.gcode.push_str(&s);
        }
        self.current_pos.x = x;
        self.current_pos.y = y;
        // `m_elapsed_time += x / m_current_feedrate * 60.f`: contracted.
        self.elapsed_time = ((if len == 0.0 { e.abs() } else { len }) / self.current_feedrate).mul_add(60.0, self.elapsed_time);
        self.gcode.push('\n');
        self
    }

    /// WipeTower.cpp:892-967 `extrude_arc_explicit(arc, f, record_length,
    /// limit_flow)`.
    pub fn extrude_arc_explicit(&mut self, arc: &crate::circle::ArcSegment, f: f32, record_length: bool, limit_flow: LimitFlow) -> &mut Self {
        // (float)unscale(arc.end_point): the double product, then float.
        let x = (arc.end_point.x as f64 * crate::libslic3r::SCALING_FACTOR) as f32;
        let y = (arc.end_point.y as f64 * crate::libslic3r::SCALING_FACTOR) as f32;
        // unscaled<float>(arc.length): float(length) * float(SCALING_FACTOR)
        let len = arc.length as f32 * crate::libslic3r::SCALING_FACTOR as f32;
        let e = len * self.extrusion_flow;
        if len < crate::libslic3r::EPSILON as f32 && e == 0.0 && (f == 0.0 || f == self.current_feedrate) {
            return self;
        }
        if record_length {
            self.used_filament_length += e;
        }
        let rotated_current_pos = self.pos_rotated();
        let rot = self.rotate(Vec2f::new(x, y));
        if !self.preview_suppressed && e > 0.0 && len > 0.0 {
            let mut width = e * self.cur_filament_area() / (len * self.layer_height);
            // `width += m_layer_height * float(1. - M_PI / 4.)`: contracted.
            width = self.layer_height.mul_add((1.0 - std::f64::consts::PI / 4.0) as f32, width);
            if self.extrusions.last().map_or(true, |l| l.pos != rotated_current_pos) {
                self.extrusions.push(Extrusion::new(rotated_current_pos, 0.0, self.current_tool));
            }
            let n = 20; // arc_fit_size
            for j in 0..n {
                let mut cur_angle = ((j as f32 / n as f32) as f64).mul_add(arc.angle_radians, arc.polar_start_theta) as f32;
                if cur_angle as f64 > 2.0 * std::f64::consts::PI {
                    cur_angle = (cur_angle as f64 - 2.0 * std::f64::consts::PI) as f32;
                } else if cur_angle < 0.0 {
                    cur_angle = (cur_angle as f64 + 2.0 * std::f64::consts::PI) as f32;
                }
                let (sin_a, cos_a) = sincos_f32(cur_angle);
                let d = point_from_f64(arc.circle.radius * cos_a as f64, arc.circle.radius * sin_a as f64);
                let tmp = Point::new(arc.circle.center.x + d.x, arc.circle.center.y + d.y);
                let p = self.rotate(unscaled_f(&tmp));
                self.extrusions.push(Extrusion::new(p, width, self.current_tool));
            }
            self.extrusions.push(Extrusion::new(rot, width, self.current_tool));
        }
        if e == 0.0 {
            self.set_travel_acceleration();
        } else {
            self.set_normal_acceleration();
        }
        self.gcode.push_str(if arc.direction == crate::circle::ArcDirection::CounterClockwise { "G3" } else { "G2" });
        let c = self.rotate(unscaled_f(&arc.circle.center));
        let center_offset = Vec2f::new(c.x - rotated_current_pos.x, c.y - rotated_current_pos.y);
        let s = self.set_format_x(rot.x);
        self.gcode.push_str(&s);
        let s = self.set_format_y(rot.y);
        self.gcode.push_str(&s);
        self.gcode.push_str(&Self::set_format_i(center_offset.x));
        self.gcode.push_str(&Self::set_format_j(center_offset.y));
        if e != 0.0 {
            self.gcode.push_str(&Self::set_format_e(e));
        }
        let mut f = f;
        if f != 0.0 && f != self.current_feedrate {
            if limit_flow != LimitFlow::None {
                let e_speed = e / ((if len == 0.0 { e.abs() } else { len }) / f * 60.0);
                let mut tmp = self.filpar_max_e_speed.get(self.current_tool).copied().unwrap_or(f32::MAX);
                if limit_flow == LimitFlow::LimitRammingFlow {
                    tmp = self.filpar_max_e_ramming_speed.get(self.current_tool).map_or(f32::MAX, |p| p.0);
                } else if limit_flow == LimitFlow::LimitRammingFlowNC {
                    tmp = self.filpar_max_e_ramming_speed.get(self.current_tool).map_or(f32::MAX, |p| p.1);
                }
                f /= 1.0_f32.max(e_speed / tmp);
            }
            let s = self.set_format_f(f);
            self.gcode.push_str(&s);
        }
        self.current_pos.x = x;
        self.current_pos.y = y;
        // `m_elapsed_time += x / m_current_feedrate * 60.f`: contracted.
        self.elapsed_time = ((if len == 0.0 { e.abs() } else { len }) / self.current_feedrate).mul_add(60.0, self.elapsed_time);
        self.gcode.push('\n');
        self
    }

    /// WipeTower.cpp:975-979 `travel(x, y, f)` / `travel(dest, f)`.
    pub fn travel_xy_f(&mut self, x: f32, y: f32, f: f32) -> &mut Self {
        self.extrude_explicit_lf(x, y, 0.0, f, false, LimitFlow::LimitPrintFlow)
    }

    /// WipeTower.cpp:986-1011 `travel_xyz(dest, z, f)`: a travel to `dest` and
    /// the absolute Z `z`, written verbatim.
    pub fn travel_xyz(&mut self, dest: Vec2f, z: &str, f: f32) -> &mut Self {
        let rotated_current_pos = self.pos_rotated();
        let rot = self.rotate(dest);
        self.set_travel_acceleration();
        self.gcode.push_str("G1");
        let eps = crate::libslic3r::EPSILON as f32;
        if (rot.x - rotated_current_pos.x).abs() > eps {
            let s = self.set_format_x(rot.x);
            self.gcode.push_str(&s);
        }
        if (rot.y - rotated_current_pos.y).abs() > eps {
            let s = self.set_format_y(rot.y);
            self.gcode.push_str(&s);
        }
        self.gcode.push_str(" Z");
        self.gcode.push_str(z);
        if f != 0.0 && f != self.current_feedrate {
            let s = self.set_format_f(f);
            self.gcode.push_str(&s);
        }
        self.current_pos.x = dest.x;
        self.current_pos.y = dest.y;
        let dx = rot.x - rotated_current_pos.x;
        let dy = rot.y - rotated_current_pos.y;
        let len = dx.mul_add(dx, dy * dy).sqrt();
        if self.current_feedrate > 0.0 {
            self.elapsed_time = (len / self.current_feedrate).mul_add(60.0, self.elapsed_time);
        }
        self.gcode.push('\n');
        self
    }

    /// WipeTower.cpp:1014-1019 `extrude(x, y, f, limit_flow)`.
    pub fn extrude_xy_f(&mut self, x: f32, y: f32, f: f32, limit_flow: LimitFlow) -> &mut Self {
        let dx = x - self.current_pos.x;
        let dy = y - self.current_pos.y;
        self.extrude_explicit_lf(x, y, dx.mul_add(dx, dy * dy).sqrt() * self.extrusion_flow, f, false, limit_flow)
    }

    /// WipeTower.cpp:1028-1051 `rectangle(ld, width, height, f)`.
    pub fn rectangle_ld(&mut self, ld: Vec2f, width: f32, height: f32, f: f32) -> &mut Self {
        let corners = [ld, ld + Vec2f::new(width, 0.0), ld + Vec2f::new(width, height), ld + Vec2f::new(0.0, height)];
        let mut index_of_closest = 0usize;
        if self.x() - ld.x > ld.x + width - self.x() {
            index_of_closest = 1;
        }
        if self.y() - ld.y > ld.y + height - self.y() {
            index_of_closest = if index_of_closest == 0 { 3 } else { 2 };
        }
        let y = self.y();
        self.travel_xy_f(corners[index_of_closest].x, y, 0.0);
        let x = self.x();
        self.travel_xy_f(x, corners[index_of_closest].y, 0.0);
        let mut i = index_of_closest;
        loop {
            i += 1;
            if i == 4 {
                i = 0;
            }
            self.extrude_xy_f(corners[i].x, corners[i].y, f, LimitFlow::LimitPrintFlow);
            if i == index_of_closest {
                break;
            }
        }
        self
    }

    /// WipeTower.cpp:1053-1091 `rectangle_fill_box(wipe_tower, ld, width,
    /// height, f)`.
    pub fn rectangle_fill_box_ld(&mut self, wipe_tower: &WipeTower, ld: Vec2f, width: f32, height: f32, f: f32) -> &mut Self {
        let need_change_flow = wipe_tower.need_thick_bridge_flow(ld.y, self.layer_height);
        let saved_flow = self.extrusion_flow;
        let corners = [ld, ld + Vec2f::new(width, 0.0), ld + Vec2f::new(width, height), ld + Vec2f::new(0.0, height)];
        let mut index_of_closest = 0usize;
        if self.x() - ld.x > ld.x + width - self.x() {
            index_of_closest = 1;
        }
        if self.y() - ld.y > ld.y + height - self.y() {
            index_of_closest = if index_of_closest == 0 { 3 } else { 2 };
        }
        let y = self.y();
        self.travel_xy_f(corners[index_of_closest].x, y, 0.0);
        let x = self.x();
        self.travel_xy_f(x, corners[index_of_closest].y, 0.0);
        let mut i = index_of_closest;
        let mut flow_changed = false;
        loop {
            i += 1;
            if i == 4 {
                i = 0;
            }
            if need_change_flow {
                if i == 1 {
                    self.set_extrusion_flow(wipe_tower.extrusion_flow_for(0.2, self.line_width));
                    self.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", format!("{:.6}", 0.2_f64)));
                    flow_changed = true;
                } else if i == 2 && flow_changed {
                    self.set_extrusion_flow(saved_flow);
                    let lh = self.layer_height;
                    self.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", std_to_string_f32(lh)));
                }
            }
            self.extrude_xy_f(corners[i].x, corners[i].y, f, LimitFlow::LimitPrintFlow);
            if i == index_of_closest {
                break;
            }
        }
        self
    }

    /// WipeTower.cpp:1092-1109 `line(wipe_tower, p0, p1, f)`.
    pub fn line_wt(&mut self, wipe_tower: &WipeTower, p0: Vec2f, p1: Vec2f, f: f32) -> &mut Self {
        let need_change_flow = wipe_tower.need_thick_bridge_flow(p0.y, self.layer_height);
        let saved_flow = self.extrusion_flow;
        if need_change_flow {
            self.set_extrusion_flow(wipe_tower.extrusion_flow_for(0.2, self.line_width));
            self.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", format!("{:.6}", 0.2_f64)));
        }
        let (mut p0, mut p1) = (p0, p1);
        if (self.x() - p0.x).abs() > (self.x() - p1.x).abs() {
            std::mem::swap(&mut p0, &mut p1);
        }
        let y = self.y();
        self.travel_xy_f(p0.x, y, 0.0);
        let x = self.x();
        self.travel_xy_f(x, p0.y, 0.0);
        self.extrude_xy_f(p1.x, p1.y, f, LimitFlow::LimitPrintFlow);
        if need_change_flow {
            self.set_extrusion_flow(saved_flow);
            let lh = self.layer_height;
            self.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", std_to_string_f32(lh)));
        }
        self
    }

    /// WipeTower.cpp:1111-1130 `rectangle_fill_box(wipe_tower, fill_box,
    /// finish_rect_wipe_path, f)`.
    pub fn rectangle_fill_box_wt(&mut self, wipe_tower: &WipeTower, fill_box: &BoxCoordinates, finish_rect_wipe_path: &mut Vec<Vec2f>, f: f32) -> &mut Self {
        let width = fill_box.rd.x - fill_box.ld.x;
        let height = fill_box.ru.y - fill_box.rd.y;
        if height > self.line_width - WIPE_TOWER_WT_EPSILON {
            self.rectangle_fill_box_ld(wipe_tower, fill_box.ld, width, height, f);
            let pos = self.pos();
            let target = if pos == fill_box.ld {
                fill_box.rd
            } else if pos == fill_box.rd {
                fill_box.ru
            } else if pos == fill_box.ru {
                fill_box.lu
            } else {
                fill_box.ld
            };
            finish_rect_wipe_path.push(pos);
            finish_rect_wipe_path.push(target);
        } else if height > WIPE_TOWER_WT_EPSILON {
            self.line_wt(wipe_tower, fill_box.ld, fill_box.rd, 0.0);
            let pos = self.pos();
            let target = if pos == fill_box.ld { fill_box.rd } else { fill_box.ld };
            finish_rect_wipe_path.push(pos);
            finish_rect_wipe_path.push(target);
        }
        self
    }

    /// WipeTower.cpp:1131-1137 `rectangle(box, f)`.
    pub fn rectangle_f(&mut self, b: &BoxCoordinates, f: f32) -> &mut Self {
        self.rectangle_ld(Vec2f::new(b.ld.x, b.ld.y), b.ru.x - b.lu.x, b.ru.y - b.rd.y, f)
    }

    /// WipeTower.cpp:1138-1189 `polygon(wall_polygon, f)`.
    pub fn polygon_f(&mut self, wall_polygon: &Polygon, f: f32) -> &mut Self {
        let mut pl = to_polyline(wall_polygon);
        pl.simplify(wt_simplify_tolerance_scaled());
        if self.enable_arc_fitting {
            pl.simplify_by_fitting_arc(scaled_wipe_tower_resolution());
        } else {
            pl.simplify(scaled_wipe_tower_resolution());
            pl.reset_to_linear_move();
        }
        let segments = fitted_segments(std::slice::from_ref(&pl), false);
        if segments.is_empty() {
            return self;
        }
        let index_of_closest = self.closest_segment(&segments);
        let mut i = index_of_closest;
        self.travel_xy_f(segments[i].start.x, segments[i].start.y, 0.0);
        self.extrude_segment(&segments[i], f);
        loop {
            i = (i + 1) % segments.len();
            if i == index_of_closest {
                break;
            }
            self.extrude_segment(&segments[i], f);
        }
        self
    }

    /// WipeTower.cpp:1149-1161 `get_closet_idx`.
    fn closest_segment(&self, corners: &[WriterSegment]) -> usize {
        let anchor = self.current_pos;
        let mut closest_index: i32 = -1;
        let mut min_distance = f32::MAX;
        for (i, c) in corners.iter().enumerate() {
            let dx = c.start.x - anchor.x;
            let dy = c.start.y - anchor.y;
            let distance = dx * dx + dy * dy;
            if distance < min_distance {
                min_distance = distance;
                closest_index = i as i32;
            }
        }
        closest_index as usize
    }

    /// `segments[i].is_arc ? extrude_arc(arcsegment, f) : extrude(end, f)`.
    fn extrude_segment(&mut self, seg: &WriterSegment, f: f32) {
        match &seg.arc {
            Some(arc) => {
                self.extrude_arc_explicit(arc, f, false, LimitFlow::LimitPrintFlow);
            }
            None => {
                self.extrude_xy_f(seg.end.x, seg.end.y, f, LimitFlow::LimitPrintFlow);
            }
        }
    }

    /// WipeTower.cpp:1191-1202 `load(e, f)`, as C++ does it.
    pub fn load_cpp(&mut self, e: f32, f: f32) -> &mut Self {
        if e == 0.0 && (f == 0.0 || f == self.current_feedrate) {
            return self;
        }
        self.gcode.push_str("G1");
        if e != 0.0 {
            self.gcode.push_str(&Self::set_format_e(e));
        }
        if f != 0.0 && f != self.current_feedrate {
            let s = self.set_format_f(f);
            self.gcode.push_str(&s);
        }
        self.gcode.push('\n');
        self
    }

    /// WipeTower.cpp:1204-1205 `retract(e, f)`, as C++ does it.
    pub fn retract_cpp(&mut self, e: f32, f: f32) -> &mut Self {
        self.load_cpp(-e, f)
    }

    /// WipeTower.cpp:1208-1222 `load_move_x_advanced`.
    pub fn load_move_x_advanced(&mut self, farthest_x: f32, loading_dist: f32, loading_speed: f32, max_x_speed: f32) -> &mut Self {
        let time = (loading_dist / loading_speed).abs();
        let mut x_distance = (farthest_x - self.x()).abs();
        let mut x_speed = x_distance / time;
        if x_speed > max_x_speed {
            x_distance = max_x_speed * time;
            x_speed = max_x_speed;
        }
        // x() + (farthest_x > x() ? 1 : -1) * x_distance: one contracted
        // multiply-add.
        let end_point = (if farthest_x > self.x() { 1.0_f32 } else { -1.0 }).mul_add(x_distance, self.x());
        let y = self.y();
        self.extrude_explicit_lf(end_point, y, loading_dist, x_speed * 60.0, false, LimitFlow::None)
    }

    /// WipeTower.cpp:1225-1232 `z_hop(hop, f)`, as C++ does it.
    pub fn z_hop_cpp(&mut self, hop: f32, f: f32) -> &mut Self {
        let z = Self::set_format_z(self.current_z + hop);
        self.gcode.push_str("G1");
        self.gcode.push_str(&z);
        if f != 0.0 && f != self.current_feedrate {
            let s = self.set_format_f(f);
            self.gcode.push_str(&s);
        }
        self.gcode.push('\n');
        self
    }

    /// WipeTower.cpp:1240-1245 `ram`.
    pub fn ram(&mut self, x1: f32, x2: f32, dy: f32, e0: f32, e: f32, f: f32) -> &mut Self {
        let y = self.current_pos.y + dy;
        self.extrude_explicit_lf(x1, y, e0, f, true, LimitFlow::None);
        let y = self.current_pos.y;
        self.extrude_explicit_lf(x2, y, e, 0.0, true, LimitFlow::None);
        self
    }

    /// WipeTower.cpp:1250-1255 `cool`.
    pub fn cool(&mut self, x1: f32, x2: f32, e1: f32, e2: f32, f: f32) -> &mut Self {
        let y = self.current_pos.y;
        self.extrude_explicit_lf(x1, y, e1, f, false, LimitFlow::None);
        let y = self.current_pos.y;
        self.extrude_explicit_lf(x2, y, e2, 0.0, false, LimitFlow::None);
        self
    }

    /// WipeTower.cpp:1271-1277 `wait(time)`, as C++ does it.
    pub fn wait_cpp(&mut self, time: f32) -> &mut Self {
        if time == 0.0 {
            return self;
        }
        self.gcode.push_str(&format!("G4 S{}\n", float_to_string_decimal_point(time as f64, 3)));
        self
    }

    /// WipeTower.cpp:1287-1293 `speed_override_backup`, as C++ does it.
    pub fn speed_override_backup_cpp(&mut self) -> &mut Self {
        if matches!(self.gcode_flavor, GCodeFlavor::Marlin | GCodeFlavor::MarlinFirmware) {
            self.gcode.push_str("M220 B\n");
        }
        self
    }

    /// WipeTower.cpp:1296-1302 `speed_override_restore`, as C++ does it.
    pub fn speed_override_restore_cpp(&mut self) -> &mut Self {
        if matches!(self.gcode_flavor, GCodeFlavor::Marlin | GCodeFlavor::MarlinFirmware) {
            self.gcode.push_str("M220 R\n");
        }
        self
    }

    /// WipeTower.cpp:1305-1316 `set_extruder_trimpot` (BBS: no trimpot control).
    pub fn set_extruder_trimpot(&mut self, _current: i32) -> &mut Self {
        self
    }

    /// WipeTower.cpp:1331-1335 `comment_with_value`.
    pub fn comment_with_value(&mut self, comment: &str, value: i32) -> &mut Self {
        self.gcode.push_str(&format!(";{}{}\n", comment, value));
        self
    }

    /// WipeTower.cpp:1363-1366 `add_wipe_point(x, y)`.
    pub fn add_wipe_point_xy(&mut self, x: f32, y: f32) -> &mut Self {
        self.add_wipe_point(Vec2f::new(x, y))
    }

    /// WipeTower.cpp:1368-1379 `add_wipe_path(polygon, wipe_dist)`.
    pub fn add_wipe_path(&mut self, polygon: &Polygon, wipe_dist: f64) -> &mut Self {
        let mut wipe_dist = wipe_dist;
        let closest_idx = polygon.closest_point_index(&scaled_vec2f_pt(self.current_pos));
        let mut wipe_path = polygon.split_at_index(closest_idx);
        wipe_path.points.reverse();
        for i in 0..wipe_path.points.len() {
            if wipe_dist < crate::libslic3r::EPSILON {
                break;
            }
            let p = unscaled_f(&wipe_path.points[i]);
            self.add_wipe_point(p);
            if i != 0 {
                let a = unscaled_pt(&wipe_path.points[i]);
                let b = unscaled_pt(&wipe_path.points[i - 1]);
                wipe_dist -= ((a.0 - b.0) * (a.0 - b.0) + (a.1 - b.1) * (a.1 - b.1)).sqrt();
            }
        }
        self
    }

    /// WipeTower.cpp:1380-1440 `generate_path(pls, feedrate, retract_length,
    /// retract_speed, used_fillet)`, as C++ does it.
    pub fn generate_path_cpp(&mut self, pls: &mut [Polyline], feedrate: f32, retract_length: f32, retract_speed: f32, _used_fillet: bool) {
        if self.enable_arc_fitting {
            for pl in pls.iter_mut() {
                pl.simplify_by_fitting_arc(scaled_wipe_tower_resolution());
            }
        } else {
            for pl in pls.iter_mut() {
                pl.simplify(scaled_wipe_tower_resolution());
                pl.reset_to_linear_move();
            }
        }
        let segments = fitted_segments(pls, true);
        if segments.is_empty() {
            return;
        }
        let index_of_closest = self.closest_segment(&segments);
        let mut i = index_of_closest;
        self.travel_xy_f(segments[i].start.x, segments[i].start.y, 0.0);
        self.extrude_segment(&segments[i], feedrate);
        loop {
            i = (i + 1) % segments.len();
            if i == index_of_closest {
                break;
            }
            let dx = segments[i].start.x - self.current_pos.x;
            let dy = segments[i].start.y - self.current_pos.y;
            let len = (dx * dx + dy * dy).sqrt();
            if len as f64 > crate::libslic3r::EPSILON {
                self.retract_cpp(retract_length, retract_speed);
                self.travel_xy_f(segments[i].start.x, segments[i].start.y, 600.0);
                self.retract_cpp(-retract_length, retract_speed);
            }
            self.extrude_segment(&segments[i], feedrate);
        }
    }

    /// WipeTower.cpp:1441-1457 `spiral_flat_ironing`.
    pub fn spiral_flat_ironing(&mut self, center: Vec2f, area: f32, step_length: f32, feedrate: f32) {
        let edge_length = area.sqrt();
        let mut box_max = center + Vec2f::new(step_length, step_length);
        let mut box_min = center - Vec2f::new(step_length, step_length);
        let mut n = (edge_length / step_length / 2.0).ceil() as i32;
        if n <= 0 {
            return;
        }
        while n > 0 {
            n -= 1;
            let y = self.current_pos.y;
            self.travel_xy_f(box_max.x, y, feedrate);
            let x = self.current_pos.x;
            self.travel_xy_f(x, box_max.y, feedrate);
            let y = self.current_pos.y;
            self.travel_xy_f(box_min.x, y, feedrate);
            let x = self.current_pos.x;
            self.travel_xy_f(x, box_min.y, feedrate);
            box_max += Vec2f::new(step_length, step_length);
            box_min = box_min - Vec2f::new(step_length, step_length);
        }
    }

    /// WipeTower.cpp:1459-1480 `format_line_M104`.
    pub fn format_line_m104(&mut self, target_temp: i32, tool: i32, is_heating: bool, wait_for_moves: bool, comment: &str) -> &mut Self {
        let mut buffer = String::new();
        if wait_for_moves && !is_heating {
            buffer.push_str("M400\n");
        }
        buffer.push_str("M104");
        if !self.handle_hotend_as_extruder {
            let mapped_extruder = self.group_extruder_id(tool, self.layer_id);
            if mapped_extruder < 0 || mapped_extruder as usize >= self.physical_extruder_map.len() {
                return self;
            }
            buffer.push_str(&format!(" T{}", self.physical_extruder_map[mapped_extruder as usize]));
        } else {
            buffer.push_str(&format!(" I{}", tool));
        }
        buffer.push_str(&format!(" S{} N0", target_temp));
        buffer.push_str(if is_heating { " W" } else { " C" });
        if !comment.is_empty() {
            buffer.push_str(&format!(" ;{}", comment));
        }
        buffer.push('\n');
        self.append(&buffer);
        self
    }

    /// WipeTower.cpp:1482-1499 `format_line_M109`.
    pub fn format_line_m109(&mut self, target_temp: i32, tool: i32, comment: &str) -> &mut Self {
        let mut buffer = String::from("M109");
        if self.multi_nozzle_group_result.is_none() {
            return self;
        }
        if !self.handle_hotend_as_extruder {
            let mapped_extruder = self.group_extruder_id(tool, self.layer_id);
            if mapped_extruder < 0 || mapped_extruder as usize >= self.physical_extruder_map.len() {
                return self;
            }
            buffer.push_str(&format!(" T{}", self.physical_extruder_map[mapped_extruder as usize]));
        } else {
            buffer.push_str(&format!(" I{}", tool));
        }
        buffer.push_str(&format!(" S{} N0", target_temp));
        if !comment.is_empty() {
            buffer.push_str(&format!(" ;{}", comment));
        }
        buffer.push('\n');
        self.append(&buffer);
        self
    }

    /// `m_multi_nozzle_group_result->get_extruder_id(tool, layer_id)`; the
    /// reduced pipeline has none and answers from `filament_map`.
    fn group_extruder_id(&self, tool: i32, layer_id: i32) -> i32 {
        match &self.multi_nozzle_group_result {
            Some(g) => g.get_extruder_id(tool, layer_id),
            None => self.filament_map.get(tool.max(0) as usize).copied().unwrap_or(0),
        }
    }

    /// WipeTower.cpp:1503 `set_first_layer_normal_acceleration`.
    pub fn set_first_layer_normal_acceleration(&mut self, accelerations: Vec<u32>) {
        self.first_layer_normal_accelerations = accelerations;
    }

    /// WipeTower.cpp:1507 `set_accel_to_decel_enable`.
    pub fn set_accel_to_decel_enable(&mut self, enable: bool) {
        self.accel_to_decel_enable = enable;
    }

    /// WipeTower.cpp:1508 `set_accel_to_decel_factor`.
    pub fn set_accel_to_decel_factor(&mut self, factor: f32) {
        self.accel_to_decel_factor = factor;
    }

    /// WipeTower.cpp:1509 `set_layer_id`.
    pub fn set_layer_id(&mut self, layer_id: i32) {
        self.layer_id = layer_id;
    }

    /// WipeTower.cpp:1510 `set_handle_hotend_as_extruder`.
    pub fn set_handle_hotend_as_extruder(&mut self, handle_hotend_as_extruder: bool) {
        self.handle_hotend_as_extruder = handle_hotend_as_extruder;
    }

    /// WipeTower.cpp:1511 `set_multi_nozzle_group_result`.
    pub fn set_multi_nozzle_group_result(&mut self, multi_nozzle_group_result: Option<std::sync::Arc<crate::multi_nozzle_utils::LayeredNozzleGroupResult>>) {
        self.multi_nozzle_group_result = multi_nozzle_group_result;
    }

    /// WipeTower.cpp:1512 `set_physical_extruder_map`.
    pub fn set_physical_extruder_map(&mut self, physical_extruder_map: Vec<i32>) {
        self.physical_extruder_map = physical_extruder_map;
    }

    /// WipeTower.cpp:1610-1614 `set_format_X`: also moves the current X.
    fn set_format_x(&mut self, x: f32) -> String {
        self.current_pos.x = x;
        format!(" X{}", float_to_string_decimal_point(x as f64, 3))
    }

    /// WipeTower.cpp:1616-1619 `set_format_Y`.
    fn set_format_y(&mut self, y: f32) -> String {
        self.current_pos.y = y;
        format!(" Y{}", float_to_string_decimal_point(y as f64, 3))
    }

    /// WipeTower.cpp:1621-1623 `set_format_Z`.
    fn set_format_z(z: f32) -> String {
        format!(" Z{}", float_to_string_decimal_point(z as f64, 3))
    }

    /// WipeTower.cpp:1625-1627 `set_format_E`.
    fn set_format_e(e: f32) -> String {
        format!(" E{}", float_to_string_decimal_point(e as f64, 4))
    }

    /// WipeTower.cpp:1629-1634 `set_format_F`: `" F%d"` of `int(floor(f +
    /// 0.5f))`; also sets the current feedrate.
    fn set_format_f(&mut self, f: f32) -> String {
        self.current_feedrate = f;
        format!(" F{}", (f + 0.5).floor() as i32)
    }

    /// WipeTower.cpp:1635 `set_format_I`.
    fn set_format_i(i: f32) -> String {
        format!(" I{}", float_to_string_decimal_point(i as f64, 3))
    }

    /// WipeTower.cpp:1636 `set_format_J`.
    fn set_format_j(j: f32) -> String {
        format!(" J{}", float_to_string_decimal_point(j as f64, 3))
    }
}

/// `std::ostream << double` with the default format (6 significant digits,
/// `%g`).
fn ostream_float(v: f64) -> String {
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".to_string() } else { "0".to_string() };
    }
    if !v.is_finite() {
        return if v.is_nan() { "nan".to_string() } else if v > 0.0 { "inf".to_string() } else { "-inf".to_string() };
    }
    let e = format!("{:.5e}", v);
    let (mant, exp) = e.split_once('e').unwrap();
    let exp: i32 = exp.parse().unwrap();
    if exp < -4 || exp >= 6 {
        let mant = mant.trim_end_matches('0').trim_end_matches('.');
        format!("{}e{}{:02}", mant, if exp < 0 { '-' } else { '+' }, exp.abs())
    } else {
        let s = format!("{:.*}", (5 - exp).max(0) as usize, v);
        if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s }
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
    outer_wall: std::collections::BTreeMap<i32, Vec<Polyline>>,
    /// Wall skip points
    /// Per-LAYER wall skip points — the positions at which the tower's outer wall
    /// is broken into gaps. WipeTower.hpp:524 declares
    /// `std::vector<std::vector<Vec2f>>`; ours was a flat `Vec<Vec2f>` (never
    /// populated, so the wrong shape had gone unnoticed). R621.
    wall_skip_points: Vec<Vec<Vec2f>>,
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

    // ------------------------------------------------------------------
    // The rest of the C++ members (WipeTower.hpp), named after them. They hold
    // the C++ member-initialiser values unless `WipeTower::new_from_config`
    // (the C++ constructor) or a setter sets them; the reduced pipeline
    // (print.rs) leaves them there, which keeps every feature they switch on
    // off for it.
    // ------------------------------------------------------------------
    /// The reduced pipeline's interface (print.rs, through `new`): results
    /// offset by the tower position, used filament counted in `construct_tcr`.
    /// `new_from_config` clears it: the results are then the C++ ones.
    absolute_positions: bool,
    /// WipeTower.hpp:408 `m_multi_diameter_print`
    multi_diameter_print: bool,
    /// WipeTower.hpp:409 `m_per_nozzle_tower_params`
    per_nozzle_tower_params: bool,
    /// WipeTower.hpp:411 `m_multi_nozzle_group_result` (a pointer in C++; the
    /// tower keeps its own copy here). `None` is the reduced pipeline, which
    /// answers the extruder / nozzle questions from `filament_map`.
    multi_nozzle_group_result: Option<std::sync::Arc<crate::multi_nozzle_utils::LayeredNozzleGroupResult>>,
    /// WipeTower.hpp:464 `m_lane_assignment`
    lane_assignment: Vec<HashMap<i32, i32>>,
    /// WipeTower.hpp:467 `m_block_slab_height`
    block_slab_height: Vec<HashMap<i32, f32>>,
    /// WipeTower.hpp:469 `m_reserved_blocks`
    reserved_blocks: Vec<std::collections::HashSet<i32>>,
    /// WipeTower.hpp:472 `m_layer_slab_height`
    layer_slab_height: Vec<f32>,
    /// WipeTower.hpp:473 `m_layer_skipped`
    layer_skipped: Vec<i8>,
    /// WipeTower.hpp:477 `m_wall_filament`
    wall_filament: Vec<i32>,
    /// WipeTower.hpp:478 `m_wall_slab_height`
    wall_slab_height: Vec<f32>,
    /// WipeTower.hpp:479 `m_wall_reserved`
    wall_reserved: Vec<i8>,
    /// WipeTower.hpp:482 `m_merged_slabs_active{false}`
    merged_slabs_active: bool,
    /// WipeTower.hpp:484 `m_foundation_required{true}`
    foundation_required: bool,
    /// WipeTower.hpp:696 `m_slice_used_filaments`
    slice_used_filaments: i32,
    /// WipeTower.hpp:706 `m_out_wall_only_z = -1.f`
    out_wall_only_z: f32,
    /// WipeTower.hpp:707 `m_out_wall_only_width = 0.f`
    out_wall_only_width: f32,
    /// WipeTower.hpp:710 `m_wipe_tower_brim_width` (the configured value, which
    /// `plan_tower_new` resolves when it is negative).
    wipe_tower_brim_width_cpp: f32,
    /// WipeTower.hpp:719-720 `m_travel_speed`, `m_first_layer_speed` (mm/s).
    travel_speed: f32,
    first_layer_speed: f32,
    /// WipeTower.hpp:722 `m_origin`
    origin: Vec2f,
    /// WipeTower.hpp:724 `m_filaments_change_length` ([0] extruder change,
    /// [1] nozzle change), doubles as in C++.
    filaments_change_length: (Vec<f64>, Vec<f64>),
    /// WipeTower.hpp:725 `m_cur_layer_id`
    cur_layer_id: usize,
    /// WipeTower.hpp:738 `m_enable_tower_interface_features{false}`
    enable_tower_interface_features: bool,
    /// WipeTower.hpp:749 `m_sparse_layers_skipped`
    sparse_layers_skipped: bool,
    /// WipeTower.hpp:753 `m_gcode_flavor`
    gcode_flavor: GCodeFlavor,
    /// WipeTower.hpp:755-761 the acceleration lists and limits.
    normal_accels: Vec<u32>,
    first_layer_normal_accels: Vec<u32>,
    travel_accels: Vec<u32>,
    first_layer_travel_accels: Vec<u32>,
    max_accels: u32,
    accel_to_decel_enable: bool,
    accel_to_decel_factor: f32,
    /// WipeTower.hpp:762 `m_enable_arc_fitting = true`
    enable_arc_fitting: bool,
    /// WipeTower.hpp:763-764 `m_hotend_heating_rate`, `m_hotend_cooling_rate`
    hotend_heating_rate: Vec<f64>,
    hotend_cooling_rate: Vec<f64>,
    /// WipeTower.hpp:765 `m_shared_print_bed`
    shared_print_bed: Vec<Polygon>,
    /// WipeTower.hpp:768-774 bed properties.
    bed_shape: BedShape,
    bed_width: f32,
    bed_bottom_left: Vec2f,
    /// WipeTower.hpp:776 `m_first_layer_flow_ratio`
    first_layer_flow_ratio: f32,
    /// WipeTower.hpp:779-781
    tower_grid_nozzle_diameter: f32,
    tower_grid_reference_nozzle: f32,
    tower_grid_layer_height: f32,
    /// WipeTower.hpp:804 `m_max_speed = 5400.f` (mm/min)
    max_speed: f32,
    /// WipeTower.hpp:805 `m_tail_speed_factor = 1.f`
    tail_speed_factor: f32,
    /// WipeTower.hpp:806 `m_tail_slowdown_volume = 10.f`
    tail_slowdown_volume: f32,
    /// WipeTower.hpp:809 `m_printable_height` (doubles)
    printable_height: Vec<f64>,
    /// WipeTower.hpp:840-842
    flat_ironing: bool,
    line_ironing: bool,
    ironing_speed: f32,
    /// WipeTower.hpp:843-860 the side scrape.
    enable_side_scrape: bool,
    side_scrape_on_left_right: bool,
    side_scrape_length: f32,
    side_scrape_height: f32,
    side_scrape_speed: f32,
    side_scrape_wall_gap: f32,
    side_scrape_point_spacing: f32,
    side_scrape_along_length: f32,
    side_scrape_seq: u32,
    /// WipeTower.hpp:861 `m_machine_max_speed = 500.f * 60.f`
    machine_max_speed: f32,
    /// WipeTower.hpp:862-866
    contact_ironing: bool,
    has_filament_switcher: bool,
    contact_speed: f32,
    handle_hotend_as_extruder: bool,
    physical_extruder_map: Vec<i32>,
    /// WipeTower.hpp:698-700, 728-737 the configuration flags under their C++
    /// names (the reduced pipeline keeps them in `config`).
    enable_wrapping_detection: bool,
    wrapping_detection_layers: i32,
    enable_timelapse_print: bool,
    semm: bool,
    wipe_tower_width: f32,
    wipe_tower_height: f32,
    wipe_tower_rotation_angle: f32,
    is_multi_extruder: bool,
    use_gap_wall: bool,
    use_rib_wall: bool,
    rib_width: f32,
    extra_rib_length: f32,
    used_fillet: bool,
    tower_framework: bool,
    bridging: f32,
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
            outer_wall: std::collections::BTreeMap::new(),
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
            // WipeTower.cpp:1850 — one entry per filament; get_filament_category
            // falls back to 0 past the end, so an empty list means "all one
            // category", which is also the PrintConfig default. R617.
            filament_categories: config.filament_categories.clone(),
            adhesion: true,
            is_multiple_nozzle: false,
            absolute_positions: true,
            multi_diameter_print: false,
            per_nozzle_tower_params: false,
            multi_nozzle_group_result: None,
            lane_assignment: vec![],
            block_slab_height: vec![],
            reserved_blocks: vec![],
            layer_slab_height: vec![],
            layer_skipped: vec![],
            wall_filament: vec![],
            wall_slab_height: vec![],
            wall_reserved: vec![],
            merged_slabs_active: false,
            foundation_required: true,
            slice_used_filaments: 0,
            out_wall_only_z: -1.0,
            out_wall_only_width: 0.0,
            wipe_tower_brim_width_cpp: config.brim_width,
            travel_speed: config.travel_speed,
            first_layer_speed: config.first_layer_speed,
            origin: Vec2f::zero(),
            filaments_change_length: (
                config.filament_change_length.iter().map(|&v| v as f64).collect(),
                config.filament_change_length_nc.iter().map(|&v| v as f64).collect(),
            ),
            cur_layer_id: 0,
            enable_tower_interface_features: false,
            sparse_layers_skipped: config.no_sparse_layers,
            gcode_flavor: config.gcode_flavor,
            normal_accels: config.normal_accels.clone(),
            first_layer_normal_accels: config.first_layer_normal_accels.clone(),
            travel_accels: config.travel_accels.clone(),
            first_layer_travel_accels: config.first_layer_travel_accels.clone(),
            max_accels: config.max_accel,
            accel_to_decel_enable: config.accel_to_decel_enable,
            accel_to_decel_factor: config.accel_to_decel_factor,
            enable_arc_fitting: config.enable_arc_fitting,
            hotend_heating_rate: config.hotend_heating_rate.iter().map(|&v| v as f64).collect(),
            hotend_cooling_rate: vec![],
            shared_print_bed: vec![],
            bed_shape: config.bed_shape,
            bed_width: config.bed_width,
            bed_bottom_left: config.bed_bottom_left,
            first_layer_flow_ratio: config.first_layer_flow_ratio,
            tower_grid_nozzle_diameter: 0.0,
            tower_grid_reference_nozzle: 0.0,
            tower_grid_layer_height: 0.0,
            max_speed: config.max_speed * 60.0,
            tail_speed_factor: 1.0,
            tail_slowdown_volume: 10.0,
            printable_height: config.printable_height.iter().map(|&v| v as f64).collect(),
            flat_ironing: config.flat_ironing,
            line_ironing: true,
            ironing_speed: 10.0 * 60.0,
            enable_side_scrape: false,
            side_scrape_on_left_right: false,
            side_scrape_length: 2.0,
            side_scrape_height: 1.5,
            side_scrape_speed: 1400.0 * 60.0,
            side_scrape_wall_gap: 0.2,
            side_scrape_point_spacing: 1.0,
            side_scrape_along_length: 2.0,
            side_scrape_seq: 0,
            machine_max_speed: 500.0 * 60.0,
            contact_ironing: false,
            has_filament_switcher: false,
            contact_speed: 20.0 * 60.0,
            handle_hotend_as_extruder: false,
            physical_extruder_map: config.physical_extruder_map.clone(),
            enable_wrapping_detection: config.enable_wrapping_detection,
            wrapping_detection_layers: config.wrapping_detection_layers,
            enable_timelapse_print: config.timelapse_smooth,
            semm: config.semm,
            wipe_tower_width: config.width,
            wipe_tower_height: config.height,
            wipe_tower_rotation_angle: config.rotation_angle,
            is_multi_extruder: config.is_multi_extruder,
            use_gap_wall: config.use_gap_wall,
            use_rib_wall: config.use_rib_wall,
            rib_width: config.rib_width,
            extra_rib_length: config.extra_rib_length,
            used_fillet: config.use_fillet,
            tower_framework: config.tower_framework,
            bridging: config.bridging,
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
            if crate::probe_enabled("WT_WIDTH_DEBUG") {
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
        // (WipeTower.hpp:285).
        let filament_area = self.filament_area();
        self.extrusion_flow =
            layer_height * self.flow_spacing(self.perimeter_width, layer_height) / filament_area;
        // R463 (WT_WIDTH_DEBUG=1): C++ emits a CONSTANT `; LINE_WIDTH: 0.500000` for the
        // tower and an E-per-mm of 0.05433, which this formula reproduces exactly at
        // h=0.3 / perimeter_width=0.5. Our measured tower E-per-mm is 0.04850. Print what
        // the fields actually hold at the moment the flow is computed.
        if crate::probe_enabled("WT_WIDTH_DEBUG") {
            eprintln!(
                "WT_WIDTH: set_layer z={:.3} layer_height={:.4} perimeter_width={:.6}                  nozzle_change_perimeter_width={:.6} => extrusion_flow={:.6}",
                print_z, layer_height, self.perimeter_width,
                self.nozzle_change_perimeter_width, self.extrusion_flow
            );
        }

        if self.is_cpp_instance() {
            // WipeTower.hpp:273-274 — advance m_layer_info while it is below
            // print_z (WT_EPSILON = 1e-3) and not the last layer.
            while !self.plan.is_empty()
                && self.plan[self.layer_idx].z < print_z - WIPE_TOWER_WT_EPSILON
                && self.layer_idx + 1 != self.plan.len()
            {
                self.layer_idx += 1;
            }
            return;
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

    /// WipeTower.hpp:238-242 `set_last_layer_extruder_fill`.
    pub fn set_last_layer_extruder_fill(&mut self, extruder_fill: bool) {
        if let Some(last) = self.plan.last_mut() {
            last.extruder_fill = extruder_fill;
        }
    }

    /// `set_last_layer_extruder_fill(false)` as the print calls it right after
    /// planning each layer whose `wipe_tower_partitions` is 0 (Print.cpp:4333),
    /// done after the plan: the partitions of a layer are its tool changes or
    /// those of any layer above it (ToolOrdering.cpp:1213-1228), so the layers
    /// above the last tool change.
    pub fn clear_extruder_fill_above_tool_changes(&mut self) {
        for layer in self.plan.iter_mut().rev() {
            if !layer.tool_changes.is_empty() {
                break;
            }
            layer.extruder_fill = false;
        }
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
        0.0_f32.max(volume / (layer_height * self.flow_spacing(line_width, layer_height)))
    }

    /// `line_width - layer_height * (1.f - float(M_PI) / 4.f)` as the
    /// WipeTower.hpp inline helpers write it: clang contracts it into one
    /// multiply-add. TOWER_FLOW_FMA_CPP (on) does so for the reduced pipeline
    /// too; =0 keeps its unfused form (the C++ generator always fuses).
    fn flow_spacing(&self, line_width: f32, layer_height: f32) -> f32 {
        static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        if self.is_cpp_instance() || *G.get_or_init(|| crate::faithful_gate("TOWER_FLOW_FMA_CPP")) {
            (-layer_height).mul_add(1.0 - PI / 4.0, line_width)
        } else {
            line_width - layer_height * (1.0 - PI / 4.0)
        }
    }

    /// Convert extrusion length to volume
    // WipeTower.hpp:538-541 — std::max(0.f, length * area)
    fn length_to_volume(&self, length: f32, line_width: f32, layer_height: f32) -> f32 {
        0.0_f32.max(length * (layer_height * self.flow_spacing(line_width, layer_height)))
    }

    /// Extrusion flow for nozzle change
    // WipeTower.hpp:287-292 — negative layer_height returns the current
    // m_extrusion_flow, otherwise compute from the nozzle-change perimeter width.
    fn nozzle_change_extrusion_flow(&self, layer_height: f32) -> f32 {
        if layer_height < 0.0 {
            return self.extrusion_flow;
        }
        let filament_area = self.filament_area();
        layer_height * self.flow_spacing(self.nozzle_change_perimeter_width, layer_height) / filament_area
    }

    /// WipeTower.cpp:8563-8566 `is_same_extruder(filament_id_1, filament_id_2,
    /// layer_id)`: from the nozzle group result; without one (the reduced
    /// pipeline) from `filament_map`.
    fn is_same_extruder(&self, filament1: usize, filament2: usize, layer_id: i32) -> bool {
        if let Some(g) = &self.multi_nozzle_group_result {
            return g.are_filaments_same_extruder(filament1 as i32, filament2 as i32, layer_id);
        }
        if filament1 >= self.filament_map.len() || filament2 >= self.filament_map.len() {
            return false;
        }
        self.filament_map[filament1] == self.filament_map[filament2]
    }

    /// WipeTower.cpp:8568-8571 `is_same_nozzle`, as `is_same_extruder`.
    fn is_same_nozzle(&self, filament1: usize, filament2: usize, layer_id: i32) -> bool {
        if let Some(g) = &self.multi_nozzle_group_result {
            return g.are_filaments_same_nozzle(filament1 as i32, filament2 as i32, layer_id);
        }
        // For single-extruder multi-material, all filaments use the same nozzle
        if self.config.semm {
            return true;
        }
        self.is_same_extruder(filament1, filament2, layer_id)
    }

    /// WipeTower.cpp:8559-8562 `is_need_ramming`, as `is_same_extruder`.
    fn is_need_ramming(&self, old_tool: usize, new_tool: usize, layer_id: i32) -> bool {
        if let Some(g) = &self.multi_nozzle_group_result {
            return !g.are_filaments_same_nozzle(old_tool as i32, new_tool as i32, layer_id);
        }
        // Ramming is needed when changing to a different extruder
        !self.is_same_extruder(old_tool, new_tool, layer_id) || !self.is_same_nozzle(old_tool, new_tool, layer_id)
    }

    /// WipeTower.cpp:3324-3327 `is_tpu_filament`: `material == "TPU"`.
    /// TOWER_TPU_EXACT_CPP (on): the exact comparison; =0 the case-insensitive
    /// one with a range check this port used.
    fn is_tpu_filament(&self, filament_id: usize) -> bool {
        if crate::faithful_gate("TOWER_TPU_EXACT_CPP") {
            return self.filament_params[filament_id].material == "TPU";
        }
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
                // TOWER_LIMIT_DEPTH_FMA_CPP (on): `base + (h - h0) / dh * dd` is one
                // contracted multiply-add (WipeTower.cpp:1747); =0 unfused.
                min_wipe_tower_depth = if crate::faithful_gate("TOWER_LIMIT_DEPTH_FMA_CPP") {
                    ((max_height - curr.0) / height_diff).mul_add(depth_diff, curr.1)
                } else {
                    curr.1 + (max_height - curr.0) / height_diff * depth_diff
                };
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
        if self.is_cpp_instance() {
            // WipeTower.cpp:3119-3181 — the C++ `plan_toolchange(z_par,
            // layer_height_par, old_tool, new_tool, wipe_volume_ec,
            // wipe_volume_nc, prime_volume)` (its live `#else` arm).
            let (z_par, layer_height_par, old_tool, new_tool, prime_volume) =
                (z, layer_height, old_tool as u32, new_tool as u32, purge_volume);
            let purge_volume = prime_volume;
            // :3122 — assert(m_plan.empty() || m_plan.back().z <= z_par + WT_EPSILON) (compiled out).
            if self.plan.is_empty() || self.plan.last().unwrap().z + WIPE_TOWER_WT_EPSILON < z_par {
                self.plan.push(WipeTowerLayerInfo::new(z_par, layer_height_par));
            }
            if self.first_layer_idx.is_none() && (!self.sparse_layers_skipped || old_tool != new_tool) {
                self.first_layer_idx = Some(self.plan.len() - 1);
            }
            if old_tool == new_tool {
                return;
            }
            let mut depth = 0.0_f32;
            let new_tool_width = self.filament_perimeter_width(new_tool as i32);
            let old_tool_nc_width = self.filament_nozzle_change_width(old_tool as i32);
            let width = self.wipe_tower_width - 2.0 * new_tool_width;
            // BBS: if the wipe tower width is too small, the depth will be infinity
            if width as f64 <= crate::libslic3r::EPSILON {
                return;
            }
            let layer_id = self.plan.len() as i32 - 1;
            let wipe_volume = if self.is_same_extruder(old_tool as usize, new_tool as usize, layer_id)
                && !self.is_same_nozzle(old_tool as usize, new_tool as usize, layer_id)
            {
                wipe_volume_nc
            } else {
                wipe_volume_ec
            };
            // :3161-3179 (`#else`)
            let length_to_extrude = self.volume_to_length(wipe_volume, new_tool_width, layer_height_par);
            depth = (length_to_extrude / width).ceil().mul_add(new_tool_width, depth);
            let filament_change_length: f32 = if !self.is_same_extruder(old_tool as usize, new_tool as usize, layer_id) {
                self.filaments_change_length.0[old_tool as usize] as f32
            } else {
                self.filaments_change_length.1[old_tool as usize] as f32
            };
            let mut nozzle_change_depth = 0.0_f32;
            let mut nozzle_change_length = 0.0_f32;
            if self.is_need_ramming(old_tool as usize, new_tool as usize, layer_id) {
                let k = 1.0_f32 - (std::f64::consts::PI as f32) / 4.0;
                let e_flow = (layer_height_par * (-layer_height_par).mul_add(k, old_tool_nc_width)
                    / self.filament_params[old_tool as usize].filament_area) as f64;
                let length = filament_change_length as f64 / e_flow;
                let nozzle_change_line_count = (length / (self.wipe_tower_width - 2.0 * old_tool_nc_width) as f64).ceil() as i32;
                nozzle_change_depth = nozzle_change_line_count as f32 * old_tool_nc_width;
                depth += nozzle_change_depth;
                nozzle_change_length = length as f32;
            }
            let mut tool_change = ToolChangeInfo::new(old_tool as usize, new_tool as usize);
            tool_change.required_depth = depth;
            tool_change.wipe_volume = wipe_volume;
            tool_change.wipe_length = length_to_extrude;
            tool_change.purge_volume = purge_volume;
            tool_change.nozzle_change_depth = nozzle_change_depth;
            tool_change.nozzle_change_length = nozzle_change_length;
            self.plan.last_mut().unwrap().tool_changes.push(tool_change);
    
            return;
        }
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
        let wipe_volume = if self.is_same_extruder(old_tool, new_tool, (self.plan.len() - 1) as i32)
            && !self.is_same_nozzle(old_tool, new_tool, (self.plan.len() - 1) as i32)
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

        if self.is_need_ramming(old_tool, new_tool, (self.plan.len() - 1) as i32) {
            let filament_change_length = if !self.is_same_extruder(old_tool, new_tool, (self.plan.len() - 1) as i32) {
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
    /// `filament_area()` (WipeTower.hpp): the current filament's cross-section.
    /// TOWER_FILAMENT_AREA_CPP (on): from its parameters, which the print fills
    /// as `float((M_PI/4.f) * pow(diameter, 2))` (WipeTower.cpp:2047); =0:
    /// `PI * 0.875 * 0.875` in float, one unit in the last place above it,
    /// which turns 1.90165 mm of filament on a 35 mm wall line into E1.9016.
    fn filament_area(&self) -> f32 {
        if crate::faithful_gate("TOWER_FILAMENT_AREA_CPP") {
            if let Some(p) = self.filament_params.get(self.current_tool) {
                return p.filament_area;
            }
        }
        PI * 0.875 * 0.875
    }

    /// WipeTower.hpp:325-328 `extrusion_spacing_for`: centre-to-centre spacing
    /// of adjacent extrusions of the given width and height.
    fn extrusion_spacing_for(&self, layer_height: f32, line_width: f32) -> f32 {
        self.flow_spacing(line_width, layer_height)
    }

    /// `m_wipe_tower_brim_width`: the configured width, or the one derived from
    /// the tower height when it is negative (WipeTower.cpp:7258).
    fn wipe_tower_brim_width(&self) -> f32 {
        if self.is_cpp_instance() {
            return self.wipe_tower_brim_width_cpp;
        }
        if self.config.brim_width < 0.0 {
            Self::get_auto_brim_by_height(self.config.height)
        } else {
            self.config.brim_width
        }
    }

    /// WipeTower.cpp:4056-4075 `brim_chamfer_loops_num`: the first layer
    /// carries the full brim, every layer above it one loop less, capped by the
    /// maximum chamfer width.
    fn brim_chamfer_loops_num(
        &self,
        layer_height: f32,
        first_layer: bool,
        dist_to_first: i32,
        line_width: f32,
    ) -> i32 {
        let line_width = if line_width <= 0.0 { self.perimeter_width } else { line_width };
        let spacing = self.extrusion_spacing_for(layer_height, line_width);
        if spacing <= crate::libslic3r::EPSILON as f32 {
            return 0;
        }
        let mut loops_num = ((self.wipe_tower_brim_width() + spacing / 2.0) / spacing) as i32;
        if !first_layer {
            if dist_to_first < 0 {
                return 0;
            }
            // WipeTower.hpp:670 WT_MAX_CHAMFER_WIDTH = 3.f
            let chamfer_loops_num = (3.0_f32 / spacing) as i32;
            loops_num = loops_num.min(chamfer_loops_num) - dist_to_first;
        }
        loops_num.max(0)
    }

    /// WipeTower.cpp:6074-6081 `first_printed_layer_idx`.
    fn first_printed_layer_idx(&self) -> i32 {
        let first = self.first_layer_idx.map_or(0, |i| i as i32);
        let mut printed = first;
        while printed < self.plan.len() as i32 && self.layer_is_skipped(printed) {
            printed += 1;
        }
        if printed < self.plan.len() as i32 { printed } else { first }
    }

    /// WipeTower.hpp:813-824 `is_first_layer_for_slab`: whether an extrusion of
    /// `slab_height` topping out at the current Z sits on the bed.
    fn is_first_layer_for_slab(&self, slab_height: f32) -> bool {
        if self.plan.is_empty() || self.layer_idx >= self.plan.len() {
            return false;
        }
        if Some(self.layer_idx) == self.first_layer_idx {
            return true;
        }
        let first = self.first_layer_idx.unwrap_or(0);
        if first >= self.plan.len() {
            return false;
        }
        let bed = self.plan[first].z - self.plan[first].height;
        self.z_pos - slab_height <= bed + WT_EPSILON
    }

    /// WipeTower.hpp:829-838 `calc_tower_feedrate(filament_id, extrusion_flow,
    /// first_layer)`.
    fn calc_tower_feedrate(&self, filament_id: usize, extrusion_flow: f32, first_layer: bool) -> f32 {
        if first_layer {
            return (self.first_layer_speed * 60.0).min(self.max_speed);
        }
        let max_e_speed = self.filament_params.get(filament_id).map_or(f32::MAX, |p| p.max_e_speed);
        if self.handle_hotend_as_extruder {
            let mut feedrate = 60.0 * max_e_speed / extrusion_flow;
            feedrate *= self.filament_params[filament_id].feedrate_factor;
            return feedrate.min(self.machine_max_speed);
        }
        (60.0 * max_e_speed / extrusion_flow).min(self.max_speed)
    }

    /// `m_layer_info->depth` as C++ reads it: under smooth timelapse
    /// `update_all_layer_depth` sets every layer to the tower depth
    /// (WipeTower.cpp:5541-5546). This port keeps the per-layer depth in
    /// `plan[i].depth` for its fill box (R509), so the C++ value is given here.
    fn layer_info_depth(&self, layer_idx: usize) -> f32 {
        if self.config.timelapse_smooth {
            return self.depth;
        }
        self.plan.get(layer_idx).map_or(self.depth, |l| l.depth)
    }

    /// WipeTower.cpp:2879-2898 `align_perimeter`.
    fn align_perimeter(&self, perimeter_box: &BoxCoordinates) -> BoxCoordinates {
        let mut aligned_box = *perimeter_box;
        let eps = crate::libslic3r::EPSILON;
        let spacing = self.extra_spacing * self.perimeter_width;
        // `float up = lu(1) - m_perimeter_width - EPSILON;`: the float
        // difference, then the double EPSILON, stored as float.
        let mut up = ((perimeter_box.lu.y - self.perimeter_width) as f64 - eps) as f32;
        up = align_ceil(up, spacing);
        up += self.perimeter_width;
        up = up.min(self.depth);
        let mut down = ((perimeter_box.ld.y - self.perimeter_width) as f64 + eps) as f32;
        down = align_floor(down, spacing);
        down += self.perimeter_width;
        down = down.max(-self.y_shift);
        aligned_box.lu.y = up;
        aligned_box.ru.y = up;
        aligned_box.ld.y = down;
        aligned_box.rd.y = down;
        aligned_box
    }

    /// `m_outer_wall[m_z_pos].push_back(pl)`: the map is keyed by Z; the key
    /// here is the bit pattern of the positive float Z, which sorts as Z does.
    fn record_outer_wall(&mut self, pl: Polyline) {
        self.outer_wall.entry(self.z_pos.to_bits() as i32).or_default().push(pl);
    }

    /// WipeTower.hpp:223-228 `get_bbx`: the extents of the walls recorded on
    /// the lowest layer (the first layer's wall and brim loops), unscaled, in
    /// the tower's own coordinates; a box at the origin when none was recorded.
    pub fn get_bbx(&self) -> ((f64, f64), (f64, f64)) {
        let Some(first) = self.outer_wall.keys().min() else {
            return ((0.0, 0.0), (0.0, 0.0));
        };
        let mut min = (i64::MAX, i64::MAX);
        let mut max = (i64::MIN, i64::MIN);
        for pl in &self.outer_wall[first] {
            for p in &pl.points {
                min.0 = min.0.min(p.x);
                min.1 = min.1.min(p.y);
                max.0 = max.0.max(p.x);
                max.1 = max.1.max(p.y);
            }
        }
        if min.0 > max.0 {
            return ((0.0, 0.0), (0.0, 0.0));
        }
        // TOWER_BBX_UNSCALE_CPP (on): `unscale(Point)` is the coordinate times
        // SCALING_FACTOR (Point.hpp:574-577); =0 divides by 1e5.
        static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        if self.is_cpp_instance() || *G.get_or_init(|| crate::faithful_gate("TOWER_BBX_UNSCALE_CPP")) {
            let u = |v: i64| v as f64 * crate::libslic3r::SCALING_FACTOR;
            return ((u(min.0), u(min.1)), (u(max.0), u(max.1)));
        }
        (
            (crate::unscale(min.0), crate::unscale(min.1)),
            (crate::unscale(max.0), crate::unscale(max.1)),
        )
    }

    /// Adhesiveness category of a filament, 0 past the end of the list.
    // WipeTower.cpp:4204-4209 — the out-of-range fallback is C++'s, not a guess:
    //   if (filament_id >= m_filament_categories.size()) return 0;
    fn get_filament_category(&self, filament_id: usize) -> i32 {
        if filament_id >= self.filament_categories.len() {
            return 0;
        }
        self.filament_categories[filament_id]
    }

    /// Index of the block for a category, creating it when asked.
    // WipeTower.cpp:4161-4179 (get_block_by_category). C++ returns a pointer;
    // returning an index instead keeps the borrow checker happy while leaving
    // the create-on-miss semantics identical.
    fn get_block_by_category_reduced(&mut self, category: i32, create: bool) -> Option<usize> {
        if let Some(idx) = self
            .wipe_tower_blocks
            .iter()
            .position(|b| b.filament_adhesiveness_category == category)
        {
            return Some(idx);
        }
        if create {
            let block = WipeTowerBlock {
                block_id: self.wipe_tower_blocks.len() as i32,
                filament_adhesiveness_category: category,
                ..Default::default()
            };
            self.wipe_tower_blocks.push(block);
            return Some(self.wipe_tower_blocks.len() - 1);
        }
        None
    }

    /// Accumulate one tool change's depth into the current layer's category bucket.
    // WipeTower.cpp:4182-4202 (add_depth_to_block). C++ takes filament_id but
    // never reads it, so it is omitted here rather than carried as a dead arg.
    fn add_depth_to_block_reduced(
        &mut self,
        category: i32,
        depth: f32,
        is_nozzle_change: bool,
    ) {
        let layer_depth = &mut self.all_layers_depth[self.layer_idx];
        if let Some(e) = layer_depth.iter_mut().find(|i| i.category == category) {
            e.depth += depth;
            if is_nozzle_change {
                e.nozzle_change_depth += depth;
            }
            return;
        }
        let mut new_block = BlockDepthInfo {
            category,
            depth,
            nozzle_change_depth: 0.0,
        };
        if is_nozzle_change {
            new_block.nozzle_change_depth += depth;
        }
        layer_depth.push(new_block);
    }

    /// Build the per-category tower blocks from the plan.
    // WipeTower.cpp:4268-4315 — steps 1 to 3 of generate_wipe_tower_blocks.
    //
    // R617: this is the population that R614 found missing — `wipe_tower_blocks`
    // was declared, initialised `vec![]`, and never written, which is why
    // `finish_block`/`finish_block_solid` (both of which take a &WipeTowerBlock)
    // could not be ported at all.
    //
    // Steps 1-3 only. C++'s step 4 (:4316-4323) then REWRITES `m_plan[i].depth`
    // from the blocks, with a reverse-cumulative max over layer_depths; that is a
    // real geometry change entangled with the two-depth problem (R509/R510), so it
    // is kept out of this round deliberately — see TOWER_BLOCK_PLAN_DEPTH below.
    // The `add_solid_flag` pass (:4335-4390) is also omitted: it classifies layers
    // via WipeTowerLayerType, an enum this port does not have (we carry
    // `solid_infill: Vec<bool>` instead), so porting it needs the enum first.
    fn generate_wipe_tower_blocks_reduced(&mut self) {
        // 1. accumulate every tool change's depth into its category, per layer.
        self.all_layers_depth.clear();
        self.all_layers_depth.resize(self.plan.len(), Vec::new());
        let saved_layer_idx = self.layer_idx;
        for layer_id in 0..self.plan.len() {
            self.layer_idx = layer_id;
            let changes: Vec<(usize, usize, f32, f32)> = self.plan[layer_id]
                .tool_changes
                .iter()
                .map(|tc| {
                    (
                        tc.old_tool,
                        tc.new_tool,
                        tc.required_depth,
                        tc.nozzle_change_depth,
                    )
                })
                .collect();
            for (old_tool, new_tool, required_depth, nozzle_change_depth) in changes {
                if !self.is_need_ramming(old_tool, new_tool, self.layer_idx as i32) {
                    let cat = self.get_filament_category(new_tool);
                    self.add_depth_to_block_reduced(cat, required_depth, false);
                } else {
                    let old_cat = self.get_filament_category(old_tool);
                    self.add_depth_to_block_reduced(old_cat, nozzle_change_depth, true);
                    let new_cat = self.get_filament_category(new_tool);
                    self.add_depth_to_block_reduced(
                        new_cat,
                        required_depth - nozzle_change_depth,
                        false,
                    );
                }
            }
        }
        self.layer_idx = saved_layer_idx;

        // 2. collapse each layer's buckets into category -> depth.
        let num_layers = self.all_layers_depth.len();
        let per_layer: Vec<Vec<(i32, f32)>> = self
            .all_layers_depth
            .iter()
            .map(|blocks| blocks.iter().map(|b| (b.category, b.depth)).collect())
            .collect();

        // 3. build the blocks.
        self.wipe_tower_blocks.clear();
        for (layer_id, cats) in per_layer.iter().enumerate() {
            for &(category, depth) in cats {
                let idx = match self.get_block_by_category_reduced(category, true) {
                    Some(i) => i,
                    None => continue,
                };
                let block = &mut self.wipe_tower_blocks[idx];
                if block.layer_depths.is_empty() {
                    block.layer_depths = vec![0.0; num_layers];
                    block.finish_depth = vec![0.0; num_layers];
                    block.solid_infill = vec![false; num_layers];
                }
                block.depth = block.depth.max(depth);
                block.layer_depths[layer_id] = depth;
            }
        }

        // 4a. Propagate each layer's depth DOWNWARD as a running max.
        // WipeTower.cpp:4316-4323 — walking layers from the top:
        //     block.layer_depths[i] = max(block.layer_depths[i], block.layer_depths[i+1]);
        // R618: this is what makes `finish_block` fire at all. Without it,
        // `layer_depths[i]` is exactly the sum of layer i's required depths, and
        // `cur_depth` advances by exactly that sum, so generate()'s skip test
        // (`cur_depth + EPS >= start_depth + layer_depths[i] - perimeter_width`)
        // reduces to `slack == -perimeter_width` on EVERY layer — measured at a
        // constant -0.500 across all 656. Raising a sparse layer's depth to the
        // max of the layers above it is what leaves room for a finish block.
        //
        // C++ folds a second statement into the same loop —
        // `m_plan[layer_id].depth = sum of block.layer_depths[layer_id]` — which
        // rewrites the PLAN depths and is a real geometry change entangled with
        // the two-depth problem (R509/R510). That half stays out; this half only
        // touches `layer_depths`, which nothing but the block machinery reads.
        if crate::faithful_gate("TOWER_BLOCK_LAYER_DEPTH_MAX") {
            for layer_id in (0..num_layers.saturating_sub(1)).rev() {
                for block in &mut self.wipe_tower_blocks {
                    if layer_id + 1 < block.layer_depths.len() {
                        block.layer_depths[layer_id] =
                            block.layer_depths[layer_id].max(block.layer_depths[layer_id + 1]);
                    }
                }
            }
        }

        if crate::probe_enabled("TOWER_BLOCKS_DEBUG") {
            eprintln!(
                "TOWER_BLOCKS: layers={} blocks={} categories={:?} depths={:?}",
                num_layers,
                self.wipe_tower_blocks.len(),
                self.wipe_tower_blocks
                    .iter()
                    .map(|b| b.filament_adhesiveness_category)
                    .collect::<Vec<_>>(),
                self.wipe_tower_blocks
                    .iter()
                    .map(|b| b.depth)
                    .collect::<Vec<_>>(),
            );
        }
    }

    /// Lay the blocks out along the tower depth.
    // WipeTower.cpp:4237-4258 (update_all_layer_depth) — BLOCK SIDE ONLY.
    //
    // C++ also multiplies every `m_plan[i].depth` by `m_extra_spacing` from inside
    // this loop (i.e. once per block, which is why it is a loop-body statement and
    // not a separate pass), and under timelapse/wrapping overwrites all plan depths
    // with `m_wipe_tower_depth`. Both are plan-side geometry changes; on Majora
    // `extra_spacing` is 1.0 and timelapse is opt-in (R509), so they are no-ops
    // here — porting them belongs with the two-depth separation (R509/R510), not
    // with the block layout. R618.
    fn update_all_layer_depth_reduced(&mut self) {
        let start_offset = self.perimeter_width;
        let mut start_depth = start_offset;
        let extra_spacing = self.extra_spacing;
        let mut tower_depth = 0.0f32;
        for block in &mut self.wipe_tower_blocks {
            block.depth *= extra_spacing;
            block.start_depth = start_depth;
            start_depth += block.depth;
            tower_depth += block.depth;
            for layer_depth in &mut block.layer_depths {
                *layer_depth *= extra_spacing;
            }
        }
        let _ = tower_depth;
    }

    /// Charge one tool change's consumed depth to its block.
    // WipeTower.cpp:3333 (`block->cur_depth += wipe_depth - nozzle_change_depth`,
    // plus `last_filament_change_id`) and :3479 (`+= nozzle_change_depth`, plus
    // `last_nozzle_change_id`). The two together advance `cur_depth` by the full
    // `wipe_depth` per tool change, which is the same quantity our single
    // `depth_traversed` cursor tracks — but C++ keeps it PER BLOCK, and
    // `finish_block` reads it to fill only the depth the tool changes left over.
    // R618.
    fn charge_tool_change_to_block(
        &mut self,
        old_tool: usize,
        new_tool: usize,
        wipe_depth: f32,
        nozzle_change_depth: f32,
    ) {
        if nozzle_change_depth > 0.0 {
            let old_cat = self.get_filament_category(old_tool);
            if let Some(idx) = self.get_block_by_category_reduced(old_cat, false) {
                let block = &mut self.wipe_tower_blocks[idx];
                block.cur_depth += nozzle_change_depth;
                block.last_nozzle_change_id = old_tool as i32;
            }
        }
        let new_cat = self.get_filament_category(new_tool);
        if let Some(idx) = self.get_block_by_category_reduced(new_cat, false) {
            let block = &mut self.wipe_tower_blocks[idx];
            block.cur_depth += wipe_depth - nozzle_change_depth;
            block.last_filament_change_id = new_tool as i32;
        }
    }

    /// Would C++ emit a `finish_block` for this block on the current layer?
    // WipeTower.cpp:4751 — generate() SKIPS the block when the tool changes have
    // already filled it:
    //   if (block.cur_depth + EPSILON >= block.start_depth
    //       + block.layer_depths[m_cur_layer_id] - m_perimeter_width) continue;
    // This is the mechanism behind C++'s {1:386, 2:264, 3:6} finish-block
    // histogram: on the 1-buckets the block is already full. R618.
    fn block_needs_finish(&self, block: &WipeTowerBlock) -> bool {
        let layer_depth = block
            .layer_depths
            .get(self.layer_idx)
            .copied()
            .unwrap_or(0.0);
        block.cur_depth + WT_EPSILON
            < block.start_depth + layer_depth - self.perimeter_width
    }

    /// Build the wall skip points for every layer.
    // WipeTower.cpp:3135-3142 (get_all_wall_skip_points), called from
    // plan_tower_new:4559 under `if (m_use_gap_wall)`. R621.
    fn get_all_wall_skip_points_reduced(&mut self) {
        self.wall_skip_points.clear();
        self.wall_skip_points.resize(self.plan.len(), Vec::new());
        for layer_id in 0..self.plan.len() {
            self.get_wall_skip_points_reduced(layer_id);
        }
    }

    /// Wall skip points for one layer.
    // WipeTower.cpp:3145-3186 (get_wall_skip_points), in its Majora-reduced form.
    //
    // Two of C++'s constructs are deliberately omitted, each because Majora's own
    // config or plan makes them inert — not as a shortcut:
    //
    //  * The `solid_toolchange && m_enable_tower_interface_features` branch
    //    (:3190) and the per-block pass that follows it. The 3MF sets
    //    `"enable_tower_interface_features": "0"` (read R620), so both are dead.
    //    They are also the ONLY readers of `block.layers_type`, which is why this
    //    port does not need the WipeTowerLayerType enum that still blocks
    //    finish_block_solid.
    //  * The ramming branch's `is_valid_last_layer` zeroing (:3159). Measured on
    //    Majora: `sum(nozzle_change_depth) = 0.00` across all 2,721 tool changes,
    //    so nozzle_change_depth is already zero and the zeroing is a no-op. The
    //    ramming structure itself is kept below so the shape matches C++.
    //
    // `infill_gap_width` is `get_block_gap_width(new_filament, false)` (:5226),
    // which for a block with no ramming is `extra_width + m_perimeter_width`
    // (:4439) with `extra_width = (m_extra_spacing - 1) * m_perimeter_width`
    // (:4433). Majora's extra_spacing is exactly 1.0 (probed WT_PLAN), so
    // extra_width is 0 and the gap width is exactly `perimeter_width` — which is
    // also C++'s own fallback when the category is absent from the map (:5232).
    // The same 1.0 makes `m_plan[layer_id].extra_spacing * infill_gap_width`
    // collapse to `infill_gap_width`; we hold a global extra_spacing, not a
    // per-layer one, so that factor is stated here rather than faked as a field.
    fn get_wall_skip_points_reduced(&mut self, layer_id: usize) {
        use std::collections::HashMap;
        let mut cur_block_depth: HashMap<i32, f32> = HashMap::new();

        let changes: Vec<(usize, usize, f32, f32)> = self.plan[layer_id]
            .tool_changes
            .iter()
            .map(|tc| {
                (
                    tc.old_tool,
                    tc.new_tool,
                    tc.required_depth,
                    tc.nozzle_change_depth,
                )
            })
            .collect();

        let width = self.config.width;
        let infill_gap_width = self.perimeter_width;

        for (old_tool, new_tool, required_depth, nozzle_change_depth) in changes {
            let wipe_depth = required_depth - nozzle_change_depth;

            let new_cat = self.get_filament_category(new_tool);
            let new_start = match self.get_block_by_category_reduced(new_cat, false) {
                Some(idx) => self.wipe_tower_blocks[idx].start_depth,
                // WipeTower.cpp:3157 — `if (!block) continue;`
                None => continue,
            };

            let mut process_depth = *cur_block_depth.entry(new_cat).or_insert(new_start);

            // WipeTower.cpp:3161-3173 — inert on Majora (nozzle_change_depth == 0)
            // but kept so the shape matches C++.
            if self.is_need_ramming(old_tool, new_tool, self.layer_idx as i32) {
                let old_cat = self.get_filament_category(old_tool);
                if new_cat == old_cat {
                    process_depth += nozzle_change_depth;
                } else {
                    let old_start = match self.get_block_by_category_reduced(old_cat, false) {
                        Some(idx) => self.wipe_tower_blocks[idx].start_depth,
                        None => continue,
                    };
                    let e = cur_block_depth.entry(old_cat).or_insert(old_start);
                    *e += nozzle_change_depth;
                }
            }

            // WipeTower.cpp:3177-3184 — the corner rotates every four layers.
            let res = match layer_id % 4 {
                0 => Vec2f::new(0.0, process_depth),
                1 => Vec2f::new(width, process_depth + wipe_depth - infill_gap_width),
                2 => Vec2f::new(width, process_depth),
                _ => Vec2f::new(0.0, process_depth + wipe_depth - infill_gap_width),
            };
            self.wall_skip_points[layer_id].push(res);

            // WipeTower.cpp:3187
            cur_block_depth.insert(new_cat, process_depth + wipe_depth);
        }
    }

    /// Reset per-layer block bookkeeping.
    // WipeTower.cpp:4219-4226 (reset_block_status), called at the top of each
    // layer in generate().
    fn reset_block_status(&mut self) {
        for block in &mut self.wipe_tower_blocks {
            block.cur_depth = block.start_depth;
            block.last_filament_change_id = -1;
            block.last_nozzle_change_id = -1;
        }
    }

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
        if crate::probe_enabled("WT_WIDTH_DEBUG") {
            eprintln!(
                "WT_PLAN: max_depth(toolchanges)={:.3} min_wipe_tower_depth={:.3} height={:.3} has_tpu={} -> extra_spacing={:.4}",
                max_depth, min_wipe_tower_depth, self.config.height, self.has_tpu_filament, self.extra_spacing
            );
            let ncd: f32 = self.plan.iter().flat_map(|l| l.tool_changes.iter()).map(|t| t.nozzle_change_depth).sum();
            let rd: f32 = self.plan.iter().flat_map(|l| l.tool_changes.iter()).map(|t| t.required_depth).sum();
            let n = self.plan.iter().map(|l| l.tool_changes.len()).sum::<usize>();
            eprintln!("WT_PLAN: tool_changes={n} sum(nozzle_change_depth)={ncd:.2} sum(required_depth)={rd:.2} ncw={:.3}", self.nozzle_change_perimeter_width);
        }

        // WipeTower.cpp:4483/4494 — plan_tower_new calls generate_wipe_tower_blocks
        // once the plan's per-tool-change depths are known. R617.
        if crate::faithful_gate("TOWER_BLOCKS_CPP") {
            self.generate_wipe_tower_blocks_reduced();
            // WipeTower.cpp:4557 — plan_tower_new lays the blocks out once the
            // depths are known. Without this every block keeps start_depth 0, so
            // reset_block_status would rewind cur_depth to 0 instead of to the
            // block's origin. R618.
            self.update_all_layer_depth_reduced();
        }

        // WipeTower.cpp:4559 — `if (m_use_gap_wall) get_all_wall_skip_points();`,
        // immediately after update_all_layer_depth (:4557), because the skip
        // points are measured from each block's start_depth. R621.
        if crate::faithful_gate("TOWER_SKIP_POINTS_CPP") && self.config.use_gap_wall {
            self.get_all_wall_skip_points_reduced();
            if crate::probe_enabled("TOWER_SKIP_POINTS_CENSUS") {
                let total: usize = self.wall_skip_points.iter().map(|v| v.len()).sum();
                let layers_with = self.wall_skip_points.iter().filter(|v| !v.is_empty()).count();
                eprintln!(
                    "TOWER_SKIP_POINTS: layers={} with_points={} total={}",
                    self.wall_skip_points.len(),
                    layers_with,
                    total
                );
            }
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

        // TOWER_GENERATE_NEW_CPP (on): the layer structure of `generate_new`
        // (WipeTower.cpp:7760-8106): the finish of a layer with tool changes
        // (`finish_layer_new` without fill) goes with the tool change of the
        // layer's wall filament, before the first one when the layer starts
        // with it; what is left of each block is filled by `finish_block` and
        // merged into the tool change of its filament (`merge_tcr`); only a
        // layer without tool changes keeps a finish of its own, with its fill.
        // =0 finishes every layer at its end with fill, wall and brim.
        let generate_new = crate::faithful_gate("TOWER_GENERATE_NEW_CPP");
        // :7769-7774, 7779 — the tool the plan starts with, then the wall
        // filament of the whole tower.
        let wall_filament = if generate_new {
            let start_tool = self
                .plan
                .iter()
                .find(|l| !l.tool_changes.is_empty())
                .map_or(self.current_tool, |l| l.tool_changes[0].old_tool);
            self.get_wall_filament_for_all_layer_reduced(start_tool)
        } else {
            -1
        };

        for (layer_idx, (z, height, num_tool_changes, new_tools)) in
            layer_data.into_iter().enumerate()
        {
            // WipeTower.cpp:4652 — reset_block_status() at the top of every layer.
            if crate::faithful_gate("TOWER_BLOCKS_CPP") {
                self.reset_block_status();
            }

            let mut layer_results: Vec<ToolChangeResult> = Vec::new();

            // Set layer
            let is_first_layer = layer_idx == 0;
            let is_last_layer = layer_idx == num_layers - 1;
            // TOWER_LAYER_COUNT_CPP (on): WipeTower.cpp:7787 — generate_new calls
            // set_layer with `is_first_layer` false on every layer, so the layer
            // counter of the grid's `; layer #N` is 1 on the first layer. =0
            // resets it to 0 there.
            let layer_count_cpp = generate_new && crate::faithful_gate("TOWER_LAYER_COUNT_CPP");
            self.set_layer(
                z,
                height,
                num_tool_changes,
                is_first_layer && !layer_count_cpp,
                is_last_layer,
            );

            // WipeTower.cpp:7840-7841, 7854-7856 — under `only_generate_wall` the
            // wall is drawn first, by the tool the layer starts with.
            let timelapse_wall = if self.config.only_generate_wall {
                Some(self.only_generate_out_wall_reduced())
            } else {
                None
            };

            if generate_new {
                self.generate_new_layer(&mut layer_results, wall_filament);
                // WipeTower.cpp:8072-8073
                if let Some(wall) = timelapse_wall {
                    if !wall.gcode.is_empty() {
                        layer_results.insert(0, wall);
                    }
                }
                results.push(layer_results);
                let _ = new_tools;
                continue;
            }

            // Generate tool changes for this layer
            for new_tool in new_tools {
                let result = self.tool_change_reduced(new_tool);
                layer_results.push(result);
            }

            // WipeTower.cpp:4749-4752 — C++ walks the blocks here and emits a
            // finish_block tcr for each one the tool changes did NOT fill. R618
            // measures how often that fires BEFORE porting the ~90-line emitter,
            // so the block count can be checked against C++'s 276 in advance.
            if crate::probe_enabled("TOWER_FINISH_BLOCK_CENSUS") {
                let n = self
                    .wipe_tower_blocks
                    .iter()
                    .filter(|b| self.block_needs_finish(b))
                    .count();
                let b = &self.wipe_tower_blocks[0];
                eprintln!(
                    "TOWER_FINISH_BLOCK: layer={} would_emit={} start={:.3} cur={:.3} layer_depth={:.3} pw={:.3} slack={:.3}",
                    layer_idx,
                    n,
                    b.start_depth,
                    b.cur_depth,
                    b.layer_depths.get(self.layer_idx).copied().unwrap_or(0.0),
                    self.perimeter_width,
                    b.start_depth + b.layer_depths.get(self.layer_idx).copied().unwrap_or(0.0)
                        - self.perimeter_width
                        - b.cur_depth,
                );
            }

            // Finish the layer
            let finish_result = self.finish_layer_reduced();
            layer_results.push(finish_result);
            // WipeTower.cpp:8072-8073 — layer_result.insert(begin, timelapse_wall).
            if let Some(wall) = timelapse_wall {
                if !wall.gcode.is_empty() {
                    layer_results.insert(0, wall);
                }
            }

            results.push(layer_results);
        }

        results
    }

    /// The tool changes and finishes of one layer as `generate_new` makes them
    /// (WipeTower.cpp:7818-8070, without skipped layers, merged slabs, solid
    /// or contact bands and reserved walls, none of which this port has).
    fn generate_new_layer(&mut self, layer_result: &mut Vec<ToolChangeResult>, wall_filament: i32) {
        let layer_id = self.layer_idx;
        let tool_changes: Vec<(usize, usize)> = self.plan[layer_id]
            .tool_changes
            .iter()
            .map(|tc| (tc.old_tool, tc.new_tool))
            .collect();
        let extruder_fill = self.plan[layer_id].extruder_fill;
        // :7818-7822
        let only_generate_wall = self.config.only_generate_wall;
        let wall_idx = self.get_wall_filament_for_layer_reduced(layer_id, wall_filament);
        let extrude_outer_wall = !only_generate_wall;
        let mut finish_layer_tcr: Option<ToolChangeResult> = None;
        // :7832-7858 — a layer without tool changes.
        if wall_idx == -1 {
            finish_layer_tcr = Some(self.finish_layer_new_reduced(extrude_outer_wall, extruder_fill, true));
        }
        // :7862-7891
        let mut insert_finish_layer_idx: Option<usize> = None;
        for (i, &(old_tool, new_tool)) in tool_changes.iter().enumerate() {
            let finish_first = i == 0 && old_tool as i32 == wall_idx;
            if finish_first {
                finish_layer_tcr = Some(self.finish_layer_new_reduced(extrude_outer_wall, false, false));
            }
            layer_result.push(self.tool_change_reduced(new_tool));
            if !finish_first && new_tool as i32 == wall_idx {
                finish_layer_tcr = Some(self.finish_layer_new_reduced(extrude_outer_wall, false, false));
                insert_finish_layer_idx = Some(i);
            }
        }
        // :7894-8052 — what the tool changes left of each block.
        if wall_idx != -1 {
            for bi in 0..self.wipe_tower_blocks.len() {
                let block = self.wipe_tower_blocks[bi].clone();
                if !self.block_needs_finish(&block) {
                    continue;
                }
                let mut finish_layer_filament = self.get_block_fill_filament_reduced(layer_id, &block, wall_idx);
                if finish_layer_filament < 0 {
                    finish_layer_filament = layer_result
                        .first()
                        .map_or(wall_idx, |r| r.new_tool);
                }
                // :7947-7966 can_carry_band
                let categories = self.filament_categories.clone();
                let block_category = block.filament_adhesiveness_category;
                let own_category_of = move |f: i32| {
                    // get_filament_category: 0 past the end of the list.
                    f >= 0 && categories.get(f as usize).copied().unwrap_or(0) == block_category
                };
                let layer_finished = self.layer_finished();
                let last_filament_change_id = block.last_filament_change_id;
                let can_carry_band = |f: i32, results: &[ToolChangeResult], fin: &Option<ToolChangeResult>| {
                    if f < 0 {
                        return false;
                    }
                    if results.iter().any(|r| r.new_tool == f) {
                        return true;
                    }
                    if (last_filament_change_id == -1 || own_category_of(f))
                        && results.iter().any(|r| r.initial_tool == f)
                    {
                        return true;
                    }
                    layer_finished && fin.as_ref().map_or(false, |t| t.new_tool == f)
                };
                if !can_carry_band(finish_layer_filament, layer_result, &finish_layer_tcr) {
                    // :7967-8005 — every filament may fill here.
                    let mut fallback = layer_result.first().map_or(-1, |r| r.new_tool);
                    if fallback < 0 && can_carry_band(wall_idx, layer_result, &finish_layer_tcr) {
                        fallback = wall_idx;
                    }
                    if fallback < 0 {
                        continue;
                    }
                    finish_layer_filament = fallback;
                }
                let finish_block_tcr = self.finish_block_reduced(bi, finish_layer_filament as usize, extruder_fill);
                // :8018-8020
                let cur = self.wipe_tower_blocks[bi].cur_depth;
                if let Some(d) = self.wipe_tower_blocks[bi].finish_depth.get_mut(layer_id) {
                    *d = cur;
                }
                // :8022-8051
                let mut has_inserted = false;
                if let Some(k) = layer_result.iter().position(|r| r.new_tool == finish_layer_filament) {
                    layer_result[k] = self.merge_tcr_reduced(&layer_result[k], &finish_block_tcr);
                    has_inserted = true;
                }
                let own_category_fill = own_category_of(finish_layer_filament);
                if (block.last_filament_change_id == -1 || own_category_fill) && !has_inserted {
                    if let Some(k) = layer_result.iter().position(|r| r.initial_tool == finish_layer_filament) {
                        layer_result[k] = self.merge_tcr_reduced(&finish_block_tcr, &layer_result[k]);
                        has_inserted = true;
                    }
                }
                if !has_inserted && is_valid_gcode(&finish_block_tcr.gcode) {
                    if let Some(fin) = finish_layer_tcr.as_ref() {
                        if self.layer_finished() && fin.new_tool == finish_layer_filament {
                            finish_layer_tcr = Some(self.merge_tcr_reduced(fin, &finish_block_tcr));
                        }
                    }
                }
            }
        }
        // :8054-8070
        if layer_result.is_empty() {
            if let Some(fin) = finish_layer_tcr {
                if only_generate_wall || (is_valid_gcode(&fin.gcode) && !fin.extrusions.is_empty()) {
                    layer_result.push(fin);
                }
            }
        } else if let Some(fin) = finish_layer_tcr {
            if is_valid_gcode(&fin.gcode) {
                match insert_finish_layer_idx {
                    None => layer_result[0] = self.merge_tcr_reduced(&fin, &layer_result[0]),
                    Some(i) => layer_result[i] = self.merge_tcr_reduced(&layer_result[i], &fin),
                }
            }
        }
    }

    /// Install the tower-wide writer settings on a freshly built writer.
    // WipeTower.cpp:2661-2670 (set_for_wipe_tower_writer). Only the acceleration
    // group is ported here; the multi-nozzle group result and accel-to-decel
    // knobs have no equivalent in this reduced port, and first-layer/layer-id are
    // already set by the individual call sites.
    fn set_for_wipe_tower_writer(&self, writer: &mut WipeTowerWriter) {
        if self.is_cpp_instance() {
            // WipeTower.cpp:2900-2914
            writer.set_normal_acceleration_values(self.normal_accels.clone());
            writer.set_travel_acceleration_values(self.travel_accels.clone());
            writer.set_first_layer_normal_acceleration_values(self.first_layer_normal_accels.clone());
            writer.set_first_layer_travel_acceleration_values(self.first_layer_travel_accels.clone());
            writer.set_max_acceleration(self.max_accels);
            writer.set_multi_nozzle_group_result(self.multi_nozzle_group_result.clone());
            writer.set_accel_to_decel_enable(self.accel_to_decel_enable);
            writer.set_accel_to_decel_factor(self.accel_to_decel_factor);
            writer.set_first_layer(self.is_first_layer());
            writer.set_layer_id(self.cur_layer_id as i32);
            writer.set_handle_hotend_as_extruder(self.handle_hotend_as_extruder);
            writer.set_physical_extruder_map(self.physical_extruder_map.clone());
            return;
        }
        if !crate::faithful_gate("TOWER_ACCEL_CPP") {
            return;
        }
        writer.set_normal_acceleration_values(self.config.normal_accels.clone());
        writer.set_travel_acceleration_values(self.config.travel_accels.clone());
        writer.set_first_layer_normal_acceleration_values(
            self.config.first_layer_normal_accels.clone(),
        );
        writer.set_first_layer_travel_acceleration_values(
            self.config.first_layer_travel_accels.clone(),
        );
        writer.set_max_acceleration(self.config.max_accel);
        writer.enable_arc_fitting = self.config.enable_arc_fitting;
    }

    /// Perform a tool change
    pub fn tool_change_reduced(&mut self, new_tool: usize) -> ToolChangeResult {
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
        // WipeTower.cpp:2661 — every writer C++ builds is passed through
        // set_for_wipe_tower_writer before use (R615).
        self.set_for_wipe_tower_writer(&mut writer);

        writer.set_initial_position(
            if tower_wipe_cpp() { self.get_next_pos_reduced(&cleaning_box, wipe_length) } else { Vec2f::new(self.perimeter_width, self.depth_traversed) },
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
        if !tower_wipe_cpp() {
            writer.feedrate(feedrate);
        }

        // WipeTower.cpp:3270-3272 (tool_change_new) — the GCodeProcessor reserved
        // block markers. R530: these are real gcode CONTENT that C++ emits and we
        // did not; GCodeProcessor consumes them to segment the preview. Counts are
        // exact against C++ because our tool-change count already matches (2,723).
        writer.append(";--------------------\n; CP TOOLCHANGE START\n");

        // R602: C++ follows the marker with a fixed four-line header and the speed
        // override backup/reset (WipeTower.cpp:3272-3282, mirrored at :2068-2077):
        //     .comment_with_value(" toolchange #", m_num_tool_changes + 1)
        //     "; material : <old> -> <new>"
        //     ";--------------------"
        //     speed_override_backup()      -> M220 B
        //     speed_override(100)          -> M220 S100
        // We emitted none of it, and instead emitted `; Tool change from Tx to Ty`,
        // which C++ never writes. File-wide that cost 8,169 `M220` lines (C++ 8,171,
        // we had 2 — `speed_override` was defined but never called) plus two comment
        // lines per tool change.
        if crate::faithful_gate("WT_TOOLCHANGE_HEADER_CPP") {
            // C++ comment is zero-based +1, i.e. the 1-based index of THIS change.
            writer.append(&format!("; toolchange #{}\n", self.num_tool_changes + 1));
            let mat = |t: usize| -> String {
                self.filament_params
                    .get(t)
                    .map(|f| f.material.clone())
                    .unwrap_or_else(|| "(NONE)".to_string())
            };
            writer.append(&format!("; material : {} -> {}\n", mat(old_tool), mat(new_tool)));
            writer.append(";--------------------\n");
            writer.speed_override_backup();
            writer.speed_override(100);
        } else {
            writer.comment(&format!("Tool change from T{} to T{}", old_tool, new_tool));
        }

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
            && crate::probe_enabled("WT_TOOLCHANGE_RETRACT_LEGACY")
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
        if self.is_need_ramming(old_tool, new_tool, self.layer_idx as i32) {
            self.toolchange_unload_reduced(&mut writer, &cleaning_box);
        }

        // Actual tool change. Faithful to WipeTower.cpp:2466: emit the
        // `[change_filament_gcode]` PLACEHOLDER, which the export step
        // (`g_code::append_tcr`) substitutes with the
        // evaluated `change_filament_gcode` template — or with a bare `Tn` when
        // that template is empty (GCode.cpp:754). Then update the writer's tool
        // state via the (output-less) set_tool.
        writer.append("[change_filament_gcode]\n");
        writer.set_tool(new_tool);
        self.current_tool = new_tool;

        // R466 — WipeTower.cpp:2460 `toolchange_Load`: "BBS: tool load is done in
        // change_filament_gcode", body `#if 0`. C++ emits NO load here either; the
        // unretract that repays the template's toolchange retraction is emitted by
        // GCode::append_tcr (see `g_code::append_tcr`), and it is
        // `retract_length_toolchange` (2.0), not `retract_length` (0.8).
        if crate::faithful_gate("WT_TOOLCHANGE_LOAD_LEGACY")
            && crate::probe_enabled("WT_TOOLCHANGE_LOAD_LEGACY")
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
        self.toolchange_wipe_reduced(&mut writer, &cleaning_box, wipe_length);

        // WipeTower.cpp:3328 — closes the Wipe_Tower_Start block opened above.
        writer.append("; WIPE_TOWER_END\n");

        // R602: WipeTower.cpp:3329 `++m_num_tool_changes` (mirrored at :2162). This
        // port never incremented the counter at all -- it was reset and read but
        // never advanced -- so every block printed `; toolchange #1`. Unconditional
        // because C++ is unconditional and the counter has no other consumer:
        // `get_number_of_toolchanges` is dead, so with the header gate off this is
        // still byte-neutral.
        self.num_tool_changes += 1;

        // R602: WipeTower.cpp:3337 `speed_override_restore()` — `M220 R`, paired with
        // the `M220 B` at the head. C++ places it between `; WIPE_TOWER_END` and the
        // `G1 F30000` / `G4 S0` / `G92 E0` trailer, verified against the reference
        // output at cpp_majora_new.gcode:7094-7099.
        if crate::faithful_gate("WT_TOOLCHANGE_HEADER_CPP") {
            writer.speed_override_restore();
            // WipeTower.cpp:2172-2174 — the trailer this comment has named since
            // it was written, but which was never emitted:
            //     writer.feedrate(m_travel_speed * 60.f)
            //           .flush_planner_queue()
            //           .reset_extruder()
            // R645 measured the cost by classifying `G92 E0` by preceding marker:
            // C++ emits 2,723 of them right after `; WIPE_TOWER_END` and we emitted
            // ZERO, while the timelapse block matched exactly at 656 — so the
            // deficit was entirely here, three lines per tool change.
            let travel_f = self.config.travel_speed * 60.0;
            writer
                .feedrate(travel_f as f32)
                .flush_planner_queue()
                .reset_extruder();
        }

        // WipeTower.cpp:3341-3343 — closes the CP TOOLCHANGE block.
        writer.append("; CP TOOLCHANGE END\n;------------------\n\n\n");

        // Update state
        self.depth_traversed += wipe_depth;
        // WipeTower.cpp:3333/:3479 — the same advance, but charged PER BLOCK so
        // finish_block can fill only what is left over. R618.
        if crate::faithful_gate("TOWER_BLOCKS_CPP") {
            self.charge_tool_change_to_block(
                old_tool,
                new_tool,
                wipe_depth,
                nozzle_change_depth,
            );
        }
        if !tower_wipe_cpp() {
            self.left_to_right = !self.left_to_right;
        }

        // Construct result
        self.construct_tcr(&writer, false, old_tool, false, true, purge_volume, false, -1.0)
    }

    /// Unload filament during tool change
    fn toolchange_unload_reduced(&self, writer: &mut WipeTowerWriter, cleaning_box: &BoxCoordinates) {
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
    /// R808: `TOWER_WIPE_CPP=0` restores the pre-R808 purge (bottom-up rows from
    /// ld.y + dy/2, one `G1 F` speed line, no start-corner rotation).
    fn toolchange_wipe_reduced(
        &mut self,
        writer: &mut WipeTowerWriter,
        cleaning_box: &BoxCoordinates,
        wipe_length: f32,
    ) {
        if tower_wipe_cpp() {
            self.toolchange_wipe_cpp(writer, cleaning_box, wipe_length);
        } else {
            self.toolchange_wipe_legacy(writer, cleaning_box, wipe_length);
        }
    }

    /// WipeTower.cpp:5300-5330 `get_next_pos` — the purge start corner rotates
    /// with the layer index: ld, rd+top, rd, ld+top (top = line_count*dy), and
    /// it is where the tcr's start_pos (the tower-entry travel target) lands.
    fn get_next_pos_reduced(&self, cleaning_box: &BoxCoordinates, wipe_length: f32) -> Vec2f {
        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;
        let line_count = (wipe_length / (xr - xl)) as i32;
        let dy = self.layer_extra_spacing() * self.perimeter_width;
        let y_offset = line_count as f32 * dy;
        match self.layer_idx % 4 {
            0 => cleaning_box.ld,
            1 => Vec2f::new(cleaning_box.rd.x, cleaning_box.rd.y + y_offset),
            2 => cleaning_box.rd,
            _ => Vec2f::new(cleaning_box.ld.x, cleaning_box.ld.y + y_offset),
        }
    }

    /// `m_layer_info->extra_spacing` (1.0 when the plan has no entry).
    fn layer_extra_spacing(&self) -> f32 {
        self.plan
            .get(self.layer_idx)
            .map(|l| l.extra_spacing)
            .unwrap_or(1.0)
    }

    /// WipeTower.cpp:3958-4160 `toolchange_wipe_new` (single-nozzle, no
    /// interface/cooling features): rows alternate direction, walk up on even
    /// layers and down on odd ones, speeds step through WipeSpeedMap with the F
    /// inline on the row's extrude, and the first row carries the gap-wall
    /// ironing (3 mm extrude, retract, back 1.5x at F600, forward at F240,
    /// deretract). R808.
    fn toolchange_wipe_cpp(
        &mut self,
        writer: &mut WipeTowerWriter,
        cleaning_box: &BoxCoordinates,
        wipe_length: f32,
    ) {
        let first = self.layer_idx == 0;
        let pw = self.perimeter_width;
        writer.set_extrusion_flow(
            self.extrusion_flow * if first { self.config.first_layer_flow_ratio } else { 1.0 },
        );
        writer.append(&format!("; CP_TOOLCHANGE_WIPE CT0 FL{}\n", if first { 1 } else { 0 }));
        if !self.nozzle_change_result.gcode.is_empty() {
            writer.append(&format!("; LINE_WIDTH: {:.6}\n", pw));
        }
        if first {
            writer.append(&format!(
                "; LINE_WIDTH: {:.6}\n",
                self.config.first_layer_flow_ratio * pw
            ));
        }
        let (retract_length, retract_speed) = self
            .filament_params
            .get(self.current_tool)
            .map_or((0.0, 0.0), |p| (p.retract_length, p.retract_speed * 60.0));
        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;
        let overlap = WIPE_TOWER_WALL_INFILL_OVERLAP * pw;
        let mut x_to_wipe = wipe_length;
        let dy = if first { pw } else { self.layer_extra_spacing() * pw };
        let max_speed = self.config.max_speed * 60.0;
        let target_speed = if first {
            (self.config.first_layer_speed * 60.0).min(max_speed)
        } else {
            max_speed
        };
        let speed_map = [
            0.33 * target_speed,
            0.375 * target_speed,
            0.458 * target_speed,
            0.875 * target_speed,
            target_speed.min(0.875 * target_speed + 50.0),
        ];
        let mut wipe_speed = speed_map[0];
        self.left_to_right = (self.layer_idx + 3) % 4 >= 2;
        let is_from_up = self.layer_idx % 2 == 1;
        let mut i = 0usize;
        loop {
            if i < speed_map.len() {
                wipe_speed = speed_map[i];
            }
            // need_thick_bridge_flow() is false for layer_height >= 0.2; the
            // floating-area branch below that is not ported.
            let mut ironing_length = 3.0_f32;
            if i == 0 && self.config.use_gap_wall {
                if self.left_to_right {
                    let dx = xr + overlap - writer.x();
                    if dx.abs() < ironing_length {
                        ironing_length = dx.abs();
                    }
                    let (x, y) = (writer.x(), writer.y());
                    writer.extrude_f(x + ironing_length, y, wipe_speed);
                    writer.retract(retract_length, retract_speed);
                    let x = writer.x();
                    writer.travel_f(x - 1.5 * ironing_length, y, 600.0);
                    let x = writer.x();
                    writer.travel_f(x + 1.5 * ironing_length, y, 240.0);
                    writer.retract(-retract_length, retract_speed);
                    let y = writer.y();
                    writer.extrude_f(xr + overlap, y, wipe_speed);
                } else {
                    let dx = xl - overlap - writer.x();
                    if dx.abs() < ironing_length {
                        ironing_length = dx.abs();
                    }
                    let (x, y) = (writer.x(), writer.y());
                    writer.extrude_f(x - ironing_length, y, wipe_speed);
                    writer.retract(retract_length, retract_speed);
                    let x = writer.x();
                    writer.travel_f(x + 1.5 * ironing_length, y, 600.0);
                    let x = writer.x();
                    writer.travel_f(x - 1.5 * ironing_length, y, 240.0);
                    writer.retract(-retract_length, retract_speed);
                    let y = writer.y();
                    writer.extrude_f(xl - overlap, y, wipe_speed);
                }
            } else {
                let y = writer.y();
                if self.left_to_right {
                    writer.extrude_f(xr + overlap, y, wipe_speed);
                } else {
                    writer.extrude_f(xl - overlap, y, wipe_speed);
                }
            }
            let y = writer.y();
            if !is_from_up && (y + dy - WT_EPSILON > cleaning_box.lu.y - pw) {
                break;
            }
            if is_from_up && (y - dy + WT_EPSILON) < cleaning_box.ld.y {
                break;
            }
            x_to_wipe -= xr - xl;
            if x_to_wipe < WT_EPSILON {
                break;
            }
            let x = writer.x();
            if is_from_up {
                writer.extrude_f(x, y - dy, 0.0);
            } else {
                writer.extrude_f(x, y + dy, 0.0);
            }
            self.left_to_right = !self.left_to_right;
            i += 1;
        }
        let (x, y) = (writer.x(), writer.y());
        writer.add_wipe_point(Vec2f::new(x, y));
        writer.add_wipe_point(Vec2f::new(
            if !self.left_to_right { self.config.width } else { 0.0 },
            y,
        ));
        if let Some(last) = self.plan.get(self.layer_idx).and_then(|l| l.tool_changes.last()) {
            if self.current_tool != last.new_tool {
                self.left_to_right = !self.left_to_right;
            }
        }
        writer.set_extrusion_flow(self.extrusion_flow);
        if first {
            writer.append(&format!("; LINE_WIDTH: {:.6}\n", pw));
        }
    }


    fn toolchange_wipe_legacy(
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
        // WipeTower.cpp:3962-3963 — `if (!m_nozzle_change_result.gcode.empty())
        // writer.change_analyzer_line_width(m_perimeter_width);`. R502 measured our
        // nozzle-change tower gcode as always empty and the C++ reference emits this
        // line ZERO times on Majora, so the guard is expected to stay closed; it is
        // ported anyway because a future nozzle-change path must carry the tag.
        if !self.nozzle_change_result.gcode.is_empty() {
            writer.append(&format!("; LINE_WIDTH: {:.6}\n", self.perimeter_width));
        }
        // WipeTower.cpp:3965-3968 — "BBS: add the note for gcode-check, when the flow
        // changed, the width should follow the change". `reserved_tag(ETags::Width)`
        // is `" LINE_WIDTH: "` (GCodeProcessor.cpp:54), i.e. the same analyzer tag the
        // object paths already use, and the value is the FIRST-LAYER flow width.
        if self.layer_idx == 0 {
            writer.append(&format!(
                "; LINE_WIDTH: {:.6}\n",
                self.config.first_layer_flow_ratio * self.perimeter_width
            ));
        }

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
        let mut y = if crate::probe_enabled("TOWER_PURGE_START") {
            cleaning_box.ld.y
        } else {
            cleaning_box.ld.y + dy / 2.0
        };
        if crate::probe_enabled("WTWL") {
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
                    // WipeTower.cpp:4085-4094 — both travels carry an explicit
                    // feedrate: 600 out, 240 back (the non-flat-ironing branch).
                    // R627: we omitted both, so `current_feedrate` stayed at
                    // retract_speed and the CLOSING retract's
                    // `f != m_current_feedrate` test suppressed its suffix —
                    // 2,721 bare `G1 E0.8000` against C++'s zero, one per tool
                    // change, all inside tower toolchange blocks (attributed).
                    writer.retract(retract_len, retract_spd);
                    writer.travel_to_f(Vec2f::new(x1 - dir * 1.5 * il, y), 600.0);
                    writer.travel_to_f(Vec2f::new(x1, y), 240.0);
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

        // WipeTower.cpp:4156-4158 — the closing half of the pair above: the extrusion
        // flow is reset to `m_extrusion_flow`, so the analyzer width has to be put
        // back to the plain perimeter width. First layer only, matching C++'s guard.
        // (Our early `line_len <= 0.0` return above skips this, as it skips the whole
        // wipe; C++ has no such guard but also never reaches a degenerate box here.)
        if self.layer_idx == 0 {
            writer.append(&format!("; LINE_WIDTH: {:.6}\n", self.perimeter_width));
        }
    }

    /// The outer wall as `generate_support_wall_new` draws it
    /// (WipeTower.cpp:8314-8369): the rectangle, broken at this layer's skip
    /// points when the gap wall is on. Returns the polygon the wipe path is
    /// taken from (`insert_skip_polygon`).
    fn draw_outer_wall(
        &self,
        writer: &mut WipeTowerWriter,
        wt_box: &BoxCoordinates,
        feedrate: f32,
    ) -> crate::geometry::Polygon {
        let wt_box = *wt_box;
        let wall_polygon = generate_rectange_polygon(wt_box.ld, wt_box.ru);
        let mut insert_skip_polygon = wall_polygon.clone();
            // WipeTower.cpp:3664 — finish_layer_new hands the wall to
            // generate_support_wall_new, which under `skip_points`
            // (= m_use_gap_wall = "prime_tower_skip_points": "1") breaks the ring
            // at this layer's skip points and emits the runs through
            // generate_path, retracting across each gap. R625.
            //
            // `rib_wall` only selects the polygon SHAPE and Majora sets
            // "prime_tower_rib_wall": "0" (R622), so the shape is the plain
            // rectangle either way; only the gapping differs.
            let skip = self
                .wall_skip_points
                .get(self.layer_idx)
                .map(|v| v.as_slice())
                .unwrap_or(&[]);
            if crate::faithful_gate("TOWER_WALL_GAPS_CPP")
                && self.config.use_gap_wall
                && !skip.is_empty()
            {
                // :5067 — the gap length is 2.5 * m_perimeter_width.
                let (runs, skip_polygon) = contrust_gap_for_skip_points(
                    &wall_polygon,
                    skip,
                    self.config.width,
                    2.5 * self.perimeter_width as f64,
                );
                let rl = self.filament_params[self.current_tool].retract_length;
                let rs = self.filament_params[self.current_tool].retract_speed * 60.0;
                writer.generate_path(&runs, feedrate, rl, rs);
                insert_skip_polygon = skip_polygon;
            } else {
                writer.rectangle(&wt_box);
            }
        insert_skip_polygon
    }

    /// `WipeTower::only_generate_out_wall(true)` (WipeTower.cpp:8215-8280): the
    /// outer wall as a result of its own, drawn by the tool that is loaded when
    /// the layer starts.
    pub fn only_generate_out_wall_reduced(&mut self) -> ToolChangeResult {
        let old_tool = self.current_tool;
        let is_first_layer = self.layer_idx == 0;

        let mut writer = WipeTowerWriter::new(
            self.layer_height,
            self.perimeter_width,
            self.config.gcode_flavor,
            &self.filament_params,
        );
        self.set_for_wipe_tower_writer(&mut writer);

        // :8239-8243 — fill_box, and the corner the writer starts from.
        // TOWER_OUT_WALL_CPP (on): `m_layer_info->depth` as C++ reads it (the
        // full tower depth under smooth timelapse), the feedrate of
        // `calc_tower_feedrate` and no feedrate line of its own (the first
        // extrusion carries it, :8264). =0: the port's per-layer depth, the
        // travel speed and a `G1 F` line.
        let out_wall_cpp = crate::faithful_gate("TOWER_OUT_WALL_CPP");
        let layer_depth = if out_wall_cpp {
            self.layer_info_depth(self.layer_idx)
        } else {
            self.plan.get(self.layer_idx).map(|l| l.depth).unwrap_or(self.depth)
        };
        let fill_box_y = self
            .plan
            .get(self.layer_idx)
            .map(|l| l.toolchanges_depth())
            .unwrap_or(self.depth_traversed)
            + self.perimeter_width;
        let fill_box = BoxCoordinates::new(
            self.perimeter_width,
            fill_box_y,
            self.config.width - 2.0 * self.perimeter_width,
            layer_depth - fill_box_y,
        );
        writer.set_initial_position(
            if self.left_to_right { fill_box.ru } else { fill_box.lu },
            self.internal_rotation,
            self.y_shift,
        );
        writer.set_initial_tool(self.current_tool);
        writer.set_z(self.z_pos);
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_wipe_tower_dimensions(self.config.width, self.depth);
        writer.set_first_layer(is_first_layer);

        // :8238 — calc_tower_feedrate, as finish_layer has it.
        let feedrate = if out_wall_cpp {
            let first = self.is_first_layer_for_slab(self.layer_height);
            self.calc_tower_feedrate(self.current_tool, self.extrusion_flow, first)
        } else if is_first_layer {
            self.config.first_layer_speed * 60.0
        } else {
            self.config.travel_speed * 60.0
        };
        if !out_wall_cpp {
            writer.feedrate(feedrate);
        }

        // :8251
        writer.append("; WIPE_TOWER_START\n");

        // :8255-8259 — in the new mode under timelapse the wall takes the whole
        // tower depth.
        let wt_box = BoxCoordinates::new(0.0, 0.0, self.config.width, self.depth);
        // :8264
        let outer_wall = self.draw_outer_wall(&mut writer, &wt_box, feedrate);
        // :8265 — m_outer_wall[m_z_pos].push_back(to_polyline(outer_wall)).
        if crate::faithful_gate("TOWER_BRIM_CHAMFER_CPP") {
            self.record_outer_wall(to_polyline(&outer_wall));
        }

        // :8271 — writer.add_wipe_path(outer_wall, m_filpar[m_current_tool].wipe_dist)
        // (WipeTower.cpp:1368-1379).
        {
            let mut wipe_dist = self.filament_params[self.current_tool].wipe_dist as f64;
            let cur = writer.pos();
            let scaled_cur = crate::geometry::Point::new(
                crate::scale(cur.x as f64),
                crate::scale(cur.y as f64),
            );
            let closest_idx = outer_wall.closest_point_index(&scaled_cur);
            let mut wipe_path = outer_wall.split_at_index(closest_idx);
            wipe_path.points.reverse();
            let pts = wipe_path.points.clone();
            for i in 0..pts.len() {
                if wipe_dist < crate::libslic3r::EPSILON {
                    break;
                }
                let sf = crate::libslic3r::SCALING_FACTOR as f32;
                writer.add_wipe_point(Vec2f::new(pts[i].x as f32 * sf, pts[i].y as f32 * sf));
                if i != 0 {
                    let dx = unscale_d(pts[i].x) - unscale_d(pts[i - 1].x);
                    let dy = unscale_d(pts[i].y) - unscale_d(pts[i - 1].y);
                    wipe_dist -= (dx * dx + dy * dy).sqrt();
                }
            }
        }
        // :8272
        writer.append("; WIPE_TOWER_END\n");

        let mut tcr = self.construct_tcr(&writer, false, old_tool, true, false, 0.0, false, -1.0);
        tcr.is_out_wall_only = true;
        tcr
    }

    /// Finish the current layer
    pub fn finish_layer_reduced(&mut self) -> ToolChangeResult {
        let is_first_layer = self.layer_idx == 0;

        // Create writer
        let mut writer = WipeTowerWriter::new(
            self.layer_height,
            self.perimeter_width,
            self.config.gcode_flavor,
            &self.filament_params,
        );
        // WipeTower.cpp:2661 — every writer C++ builds is passed through
        // set_for_wipe_tower_writer before use (R615).
        self.set_for_wipe_tower_writer(&mut writer);

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
        if crate::probe_enabled("WTFILLCNT") {
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

            // WipeTower.cpp:3643-3644 — closes the block. C++'s literal is
            //     "; CP EMPTY GRID END\n" ";------------------\n\n\n\n\n\n\n"
            // i.e. SEVEN newlines after the separator text, which terminate the
            // separator line and leave SIX blank lines. R616: we emitted eight,
            // so every EMPTY GRID close carried one extra blank line — measured
            // on Majora as C++ {6 blanks: 209} against our {7 blanks: 207}.
            writer.append("; CP EMPTY GRID END\n;------------------\n\n\n\n\n\n\n");
        }

        // Draw outer perimeter
        let wt_box = BoxCoordinates::new(0.0, 0.0, self.config.width, self.depth);

        // Only draw perimeter if this is first layer or we need it.
        // WipeTower.cpp:7821 — `extrude_outer_wall = !only_generate_wall && ..`:
        // under timelapse the wall is the layer's first result.
        let extrude_perimeter =
            (is_first_layer || !self.config.no_sparse_layers) && !self.config.only_generate_wall;
        let drawn_wall = if extrude_perimeter {
            Some(self.draw_outer_wall(&mut writer, &wt_box, feedrate))
        } else {
            None
        };

        // TOWER_BRIM_CHAMFER_CPP (on): the brim and its chamfer as
        // finish_layer_new draws them (WipeTower.cpp:4235-4242, 4268-4272,
        // 4333-4371): loops offset from the outer wall by the extrusion spacing,
        // the full brim on the first layer and one loop less on every layer
        // above it, each loop recorded with the wall for `get_bbx`. =0 draws
        // rectangles 0.9 line widths apart on the first layer only.
        if crate::faithful_gate("TOWER_BRIM_CHAMFER_CPP") {
            let wall_height = self.layer_height;
            let wall_width = self.perimeter_width;
            let perimeter_flow = self.extrusion_flow;
            let wall_first_layer = self.is_first_layer_for_slab(wall_height);
            let chamfer_feedrate =
                self.calc_tower_feedrate(self.current_tool, perimeter_flow, wall_first_layer);
            // :4237-4242 — one block: the layer's own depth plus a line width.
            let mut wipe_tower_depth = self.depth;
            if self.wipe_tower_blocks.len() == 1 {
                wipe_tower_depth = self.layer_info_depth(self.layer_idx) + self.perimeter_width;
            }
            let wall_box = self.align_perimeter(&BoxCoordinates::new(
                0.0,
                0.0,
                self.config.width,
                wipe_tower_depth,
            ));
            // :4266 — generate_support_wall_new returns the wall polygon, drawn
            // or not (:8356).
            let mut outer_wall = match drawn_wall {
                Some(w) => {
                    // :4268-4272
                    let mut shifted = to_polyline(&w);
                    let dy = crate::scaled(self.y_shift as f64);
                    for p in &mut shifted.points {
                        p.y += dy;
                    }
                    self.record_outer_wall(shifted);
                    w
                }
                None => generate_rectange_polygon(wall_box.ld, wall_box.ru),
            };
            let _ = self.draw_brim_chamfer(
                &mut writer,
                &mut outer_wall,
                wall_height,
                wall_width,
                wall_first_layer,
                chamfer_feedrate,
            );
            if is_first_layer {
                self.print_brim = false;
            }
        } else if is_first_layer && self.print_brim && self.brim_width_real > 0.0 {
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

        self.construct_tcr(&writer, false, self.current_tool, true, false, 0.0, false, -1.0)
    }


    /// WipeTower.cpp:4333-4371: the brim on the tower's first layer and its
    /// chamfer above, loops offset from `outer_wall` by the extrusion spacing,
    /// each drawn and recorded with the walls. `outer_wall` ends as the last
    /// loop; returns the number of loops.
    fn draw_brim_chamfer(
        &mut self,
        writer: &mut WipeTowerWriter,
        outer_wall: &mut crate::geometry::Polygon,
        wall_height: f32,
        wall_width: f32,
        wall_first_layer: bool,
        feedrate: f32,
    ) -> i32 {
        // :4336-4344
        let spacing = self.extrusion_spacing_for(wall_height, wall_width);
        let dist_to_1st = self.layer_idx as i32 - self.first_printed_layer_idx();
        let mut loops_num =
            self.brim_chamfer_loops_num(wall_height, wall_first_layer, dist_to_1st, wall_width);
        if !wall_first_layer && self.layer_info_depth(self.layer_idx) != self.layer_info_depth(0) {
            loops_num = 0;
        }
        if loops_num > 0 {
            // :4347-4349 — the wall the loops surround has this layer's wall
            // width.
            let drawn_wall_width = wall_width;
            let first_loop_step =
                self.extrusion_spacing_for(wall_height, 0.5 * (drawn_wall_width + wall_width));
            for i in 0..loops_num {
                let step = if i == 0 { first_loop_step } else { spacing };
                // scaled(float): float / float(SCALING_FACTOR), truncated.
                let delta = (step / crate::libslic3r::SCALING_FACTOR as f32) as i64 as f64;
                let expanded = crate::clipper_utils::offset_polygons_clib(
                    std::slice::from_ref(&*outer_wall),
                    delta,
                    crate::clipper_utils::OffsetJoinType::Miter,
                );
                if expanded.is_empty() {
                    break;
                }
                *outer_wall = expanded[0].clone();
                writer.polygon(outer_wall, feedrate);
                self.record_outer_wall(to_polyline(outer_wall));
            }
            if wall_first_layer {
                // :4364-4367
                self.brim_width_real =
                    first_loop_step + (loops_num - 1) as f32 * spacing + spacing / 2.0;
            }
        }
        loops_num
    }

    /// WipeTower.cpp:1368-1379 `add_wipe_path(polygon, wipe_dist)`: from the
    /// polygon point nearest the pen, backwards along the polygon for
    /// `wipe_dist`.
    fn add_wipe_path(writer: &mut WipeTowerWriter, polygon: &crate::geometry::Polygon, wipe_dist: f64) {
        let mut wipe_dist = wipe_dist;
        let cur = writer.pos();
        let scaled_cur =
            crate::geometry::Point::new(crate::scale(cur.x as f64), crate::scale(cur.y as f64));
        let closest_idx = polygon.closest_point_index(&scaled_cur);
        let mut wipe_path = polygon.split_at_index(closest_idx);
        wipe_path.points.reverse();
        let pts = wipe_path.points.clone();
        for i in 0..pts.len() {
            if wipe_dist < crate::libslic3r::EPSILON {
                break;
            }
            let sf = crate::libslic3r::SCALING_FACTOR as f32;
            writer.add_wipe_point(Vec2f::new(pts[i].x as f32 * sf, pts[i].y as f32 * sf));
            if i != 0 {
                let dx = unscale_d(pts[i].x) - unscale_d(pts[i - 1].x);
                let dy = unscale_d(pts[i].y) - unscale_d(pts[i - 1].y);
                wipe_dist -= (dx * dx + dy * dy).sqrt();
            }
        }
    }

    /// The sparse grid or solid fill of a fill box, shared by
    /// `finish_layer_new` (WipeTower.cpp:4171-4219) and `finish_block`
    /// (:4447-4500): a travel into the box, the grid markers with the layer
    /// number, and the fill; the sparse grid replaces the wipe path.
    fn draw_fill_grid(
        &self,
        writer: &mut WipeTowerWriter,
        fill_box: &BoxCoordinates,
        line_width: f32,
        first_layer: bool,
        feedrate: f32,
        finish_rect_wipe_path: &mut Vec<Vec2f>,
    ) {
        let dy = fill_box.lu.y - fill_box.ld.y - line_width;
        let mut left = fill_box.lu.x + 2.0 * line_width;
        let mut right = fill_box.ru.x - 2.0 * line_width;
        if !(dy > line_width) {
            return;
        }
        writer.travel_to(fill_box.ld + Vec2f::new(line_width * 2.0, 0.0));
        writer.append(";--------------------\n; CP EMPTY GRID START\n");
        // comment_with_value(" layer #", m_num_layer_changes + 1)
        writer.append(&format!("; layer #{}\n", self.num_layer_changes + 1));
        // Is there a soluble filament wiped/rammed at the next layer? If so,
        // the infill should not be sparse.
        let soluble_next = self.plan.get(self.layer_idx + 1).map_or(false, |l| {
            l.tool_changes.iter().any(|tch| {
                let sol = |t: usize| self.filament_params.get(t).map_or(false, |p| p.is_soluble);
                sol(tch.new_tool) || sol(tch.old_tool)
            })
        });
        let solid_infill = soluble_next || (first_layer && self.adhesion);
        if solid_infill {
            let mut sparse_factor = 1.5_f32;
            if first_layer {
                left -= line_width;
                right += line_width;
                sparse_factor = 1.0;
            }
            let mut y = fill_box.ld.y + line_width;
            let n = (dy / (line_width * sparse_factor)) as i32;
            let spacing = if n > 1 { (dy - line_width) / (n - 1) as f32 } else { 0.0 };
            for i in 0..n {
                let x = writer.x();
                writer.extrude_f(x, y, feedrate);
                writer.extrude(if i % 2 != 0 { left } else { right }, y);
                y += spacing;
            }
            let x = writer.x();
            writer.extrude(x, fill_box.lu.y);
        } else {
            // An inverse U at the left of the region and the sparse infill.
            writer.extrude_f(fill_box.lu.x + line_width * 2.0, fill_box.lu.y, feedrate);
            let n = 1 + ((right - left) / self.config.bridging) as i32;
            let dx = if n > 0 { (right - left) / n as f32 } else { 0.0 };
            for i in 1..=n {
                let x = left + dx * i as f32;
                let cy = writer.y();
                writer.travel(x, cy);
                writer.extrude(x, if i % 2 != 0 { fill_box.rd.y } else { fill_box.ru.y });
            }
            finish_rect_wipe_path.clear();
            finish_rect_wipe_path.push(writer.pos());
            finish_rect_wipe_path.push(Vec2f::new(
                left + dx * n as f32,
                if n % 2 != 0 { fill_box.ru.y } else { fill_box.rd.y },
            ));
        }
        writer.append("; CP EMPTY GRID END\n;------------------\n\n\n\n\n\n\n");
    }

    /// The wipe at the end of a finish without a wall (WipeTower.cpp:4378-4385,
    /// 4505-4511): from the pen to the next corner of the wall box, or along
    /// the last rectangle or grid line when the pen ended it.
    fn add_finish_wipe(writer: &mut WipeTowerWriter, wt_box: &BoxCoordinates, finish_rect_wipe_path: &[Vec2f]) {
        let pos = writer.pos();
        let mut target = if pos == wt_box.ld {
            wt_box.rd
        } else if pos == wt_box.rd {
            wt_box.ru
        } else if pos == wt_box.ru {
            wt_box.lu
        } else {
            wt_box.ld
        };
        if finish_rect_wipe_path.len() == 2 && finish_rect_wipe_path[0] == pos {
            target = finish_rect_wipe_path[1];
        }
        writer.add_wipe_point(pos);
        writer.add_wipe_point(target);
    }

    /// WipeTower.cpp:4077-4396 `finish_layer_new` for this port's towers: one
    /// block or several without merged fills, no merged slabs, side scrape or
    /// capped widths. The fill box of the layer (its rectangle when
    /// `extrude_fill_wall`, the grid when `extrude_fill`), the outer wall when
    /// `extrude_perimeter`, the brim or chamfer, and the wipe.
    pub fn finish_layer_new_reduced(
        &mut self,
        extrude_perimeter: bool,
        extrude_fill: bool,
        extrude_fill_wall: bool,
    ) -> ToolChangeResult {
        self.current_layer_finished = true;
        let wall_height = self.layer_height;
        let wall_width = self.perimeter_width;
        // extrusion_flow_for(m_layer_height, wall_width): the layer's flow.
        let wall_flow = self.extrusion_flow;
        let mut writer = WipeTowerWriter::new(
            self.layer_height,
            wall_width,
            self.config.gcode_flavor,
            &self.filament_params,
        );
        writer.set_extrusion_flow(wall_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        self.set_for_wipe_tower_writer(&mut writer);
        let first_layer = self.is_first_layer_for_slab(self.layer_height);
        writer.set_first_layer(first_layer);
        writer.append("; WIPE_TOWER_START\n");
        let feedrate = self.calc_tower_feedrate(self.current_tool, wall_flow, first_layer);

        let toolchanges_on_layer = self
            .plan
            .get(self.layer_idx)
            .map_or(false, |l| l.toolchanges_depth() > WT_EPSILON);
        let mut finish_rect_wipe_path: Vec<Vec2f> = Vec::new();
        // :4107-4135 — one fill box over the whole tower, or over the band of
        // the one block.
        let mut fill_box_depth = self.depth - 2.0 * wall_width;
        if self.wipe_tower_blocks.len() == 1 {
            let band = self.wipe_tower_blocks[0]
                .layer_depths
                .get(self.layer_idx)
                .copied()
                .unwrap_or(self.wipe_tower_blocks[0].depth);
            fill_box_depth = band - 2.0 * wall_width;
        }
        let fill_box = BoxCoordinates::new(
            wall_width,
            wall_width,
            self.config.width - 2.0 * wall_width,
            fill_box_depth,
        );
        writer.set_initial_position(
            if self.left_to_right { fill_box.ru } else { fill_box.lu },
            self.internal_rotation,
            self.y_shift,
        );
        writer.set_wipe_tower_dimensions(self.config.width, self.depth);
        // :4157-4219
        if extrude_fill_wall && (fill_box.ru.y - fill_box.rd.y > WT_EPSILON) {
            writer.rectangle_fill_box_wipe(&fill_box, &mut finish_rect_wipe_path, feedrate, wall_width);
        }
        if extrude_fill {
            self.draw_fill_grid(
                &mut writer,
                &fill_box,
                wall_width,
                first_layer,
                feedrate,
                &mut finish_rect_wipe_path,
            );
        }

        // :4226-4233
        let perimeter_flow = self.extrusion_flow;
        writer.set_extrusion_flow(perimeter_flow);
        let wall_first_layer = self.is_first_layer_for_slab(wall_height);
        writer.set_first_layer(wall_first_layer);
        let feedrate = self.calc_tower_feedrate(self.current_tool, perimeter_flow, wall_first_layer);
        // :4237-4242
        let mut wipe_tower_depth = self.depth;
        if self.wipe_tower_blocks.len() == 1 {
            wipe_tower_depth = self.layer_info_depth(self.layer_idx) + self.perimeter_width;
        }
        let wt_box =
            self.align_perimeter(&BoxCoordinates::new(0.0, 0.0, self.config.width, wipe_tower_depth));
        // :4266-4272
        let mut outer_wall = if extrude_perimeter {
            let w = self.draw_outer_wall(&mut writer, &wt_box, feedrate);
            let mut shifted = to_polyline(&w);
            let dy = crate::scaled(self.y_shift as f64);
            for p in &mut shifted.points {
                p.y += dy;
            }
            self.record_outer_wall(shifted);
            w
        } else {
            generate_rectange_polygon(wt_box.ld, wt_box.ru)
        };
        let loops_num = self.draw_brim_chamfer(
            &mut writer,
            &mut outer_wall,
            wall_height,
            wall_width,
            wall_first_layer,
            feedrate,
        );
        if self.layer_idx == 0 {
            self.print_brim = false;
        }
        // :4374-4385
        if extrude_perimeter || loops_num > 0 {
            let wipe_dist = self.filament_params[self.current_tool].wipe_dist as f64;
            Self::add_wipe_path(&mut writer, &outer_wall, wipe_dist);
        } else {
            Self::add_finish_wipe(&mut writer, &wt_box, &finish_rect_wipe_path);
        }
        writer.append("; WIPE_TOWER_END\n");
        let _ = toolchanges_on_layer;
        self.construct_tcr(&writer, false, self.current_tool, true, false, 0.0, false, -1.0)
    }

    /// WipeTower.cpp:4398-4529 `finish_block`: what is left of the block on
    /// this layer after its tool changes, filled by `filament_id` — the
    /// rectangle around it and, with `extrude_fill`, the grid — then the wipe.
    fn finish_block_reduced(&mut self, block_idx: usize, filament_id: usize, extrude_fill: bool) -> ToolChangeResult {
        let block = self.wipe_tower_blocks[block_idx].clone();
        let fill_width = self.perimeter_width;
        // `block.layer_heights` is not kept: every slab is the layer height.
        let slab_height = self.layer_height;
        // slab_height * (fill_width - slab_height * (1 - pi/4)) / filament_area:
        // the layer's flow.
        let filament_e_flow = self.extrusion_flow;
        let mut writer = WipeTowerWriter::new(
            slab_height,
            fill_width,
            self.config.gcode_flavor,
            &self.filament_params,
        );
        writer.set_extrusion_flow(filament_e_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(filament_id);
        self.set_for_wipe_tower_writer(&mut writer);
        let first_layer = self.is_first_layer_for_slab(slab_height);
        writer.set_first_layer(first_layer);
        writer.append("; WIPE_TOWER_START\n");
        let feedrate = self.calc_tower_feedrate(filament_id, filament_e_flow, first_layer);
        // :4422-4426
        let band = block.layer_depths.get(self.layer_idx).copied().unwrap_or(block.depth);
        let fill_box = BoxCoordinates::new(
            fill_width,
            block.cur_depth,
            self.config.width - 2.0 * fill_width,
            block.start_depth + band - block.cur_depth - fill_width,
        );
        writer.set_initial_position(
            if self.left_to_right { fill_box.ru } else { fill_box.lu },
            self.internal_rotation,
            self.y_shift,
        );
        writer.set_wipe_tower_dimensions(self.config.width, self.depth);
        let mut finish_rect_wipe_path: Vec<Vec2f> = Vec::new();
        // :4437-4445
        if fill_box.ru.y - fill_box.rd.y > WT_EPSILON {
            writer.rectangle_fill_box_wipe(&fill_box, &mut finish_rect_wipe_path, feedrate, fill_width);
        }
        // :4447-4501
        if extrude_fill {
            self.draw_fill_grid(
                &mut writer,
                &fill_box,
                fill_width,
                first_layer,
                feedrate,
                &mut finish_rect_wipe_path,
            );
        }
        // :4503-4513
        let wt_box = self.align_perimeter(&BoxCoordinates::new(
            0.0,
            0.0,
            self.config.width,
            self.layer_info_depth(self.layer_idx) + self.perimeter_width,
        ));
        Self::add_finish_wipe(&mut writer, &wt_box, &finish_rect_wipe_path);
        writer.append("; WIPE_TOWER_END\n");
        // construct_block_tcr (WipeTower.cpp:1686-1706).
        let mut tcr = self.construct_tcr(&writer, false, filament_id, true, false, 0.0, false, -1.0);
        tcr.initial_tool = filament_id as i32;
        tcr.new_tool = filament_id as i32;
        tcr.is_tool_change = false;
        tcr.tool_change_start_pos = Vec2f::zero();
        tcr
    }

    /// WipeTower.cpp:3353-3427 `merge_tcr`: `second` appended to `first`, with
    /// a travel between them unless `second` is a tool change that starts
    /// where its tool change starts (that travel is made by the G-code side).
    /// The tower G-code here is in tower coordinates and the positions on the
    /// result on the bed, and the analyzer lines the C++ writer puts at the
    /// head of every result are written by the export for the first part only,
    /// so the second part's are put in here.
    fn merge_tcr_reduced(&self, first: &ToolChangeResult, second: &ToolChangeResult) -> ToolChangeResult {
        let mut out = first.clone();
        let d = Vec2f::new(first.end_pos.x - second.start_pos.x, first.end_pos.y - second.start_pos.y);
        if (d.x * d.x + d.y * d.y).sqrt() > crate::libslic3r::EPSILON as f32 {
            let local = Vec2f::new(second.start_pos.x - self.pos.x, second.start_pos.y - self.pos.y);
            let travel_gcode = format!(
                "G1 X{:.3} Y{:.3} F{:.0}\n",
                local.x,
                local.y,
                self.config.max_speed * 60.0
            );
            let is_approx = |a: f32, b: f32| ((a as f64) - (b as f64)).abs() < crate::libslic3r::EPSILON;
            let need_insert_travel = !(second.is_tool_change
                && is_approx(second.start_pos.x, second.tool_change_start_pos.x)
                && is_approx(second.start_pos.y, second.tool_change_start_pos.y));
            if need_insert_travel {
                out.gcode.push_str(&travel_gcode);
            }
        }
        // The analyzer lines of the second writer (WipeTower.cpp:713-719).
        out.gcode.push_str(&format!(
            "; LAYER_HEIGHT: {:.6}\n; FEATURE: Prime tower\n; LINE_WIDTH: {:.6}\n",
            second.layer_height, second.perimeter_width
        ));
        out.gcode.push_str(&second.gcode);
        out.extrusions.extend(second.extrusions.iter().cloned());
        out.end_pos = second.end_pos;
        out.wipe_path = second.wipe_path.clone();
        out.initial_tool = first.initial_tool;
        out.new_tool = second.new_tool;
        let has_nozzle_change_action = |r: &NozzleChangeResult| !r.gcode.is_empty();
        if has_nozzle_change_action(&first.nozzle_change_result) {
            out.nozzle_change_result = first.nozzle_change_result.clone();
        } else if has_nozzle_change_action(&second.nozzle_change_result) {
            out.nozzle_change_result = second.nozzle_change_result.clone();
        }
        if first.is_tool_change {
            out.is_tool_change = true;
            out.tool_change_start_pos = first.tool_change_start_pos;
        } else if second.is_tool_change {
            out.is_tool_change = true;
            out.tool_change_start_pos = second.tool_change_start_pos;
        } else {
            out.is_tool_change = false;
        }
        out.purge_volume += second.purge_volume;
        out
    }

    /// WipeTower.cpp:7370-7435 `get_wall_filament_for_all_layer`: the filament
    /// of the most-visited category that visits the most layers. Every
    /// filament prints the base layers here (no merged slabs).
    fn get_wall_filament_for_all_layer_reduced(&self, start_tool: usize) -> i32 {
        use std::collections::{BTreeMap, HashSet};
        let mut category_counts: BTreeMap<i32, i32> = BTreeMap::new();
        let mut filament_counts: BTreeMap<usize, i32> = BTreeMap::new();
        let mut current_tool = start_tool;
        for layer in &self.plan {
            if layer.tool_changes.is_empty() {
                *filament_counts.entry(current_tool).or_insert(0) += 1;
                *category_counts.entry(self.get_filament_category(current_tool)).or_insert(0) += 1;
                continue;
            }
            let mut used_tools: HashSet<usize> = HashSet::new();
            let mut used_category: HashSet<i32> = HashSet::new();
            for (i, tc) in layer.tool_changes.iter().enumerate() {
                if i == 0 {
                    *filament_counts.entry(tc.old_tool).or_insert(0) += 1;
                    *category_counts.entry(self.get_filament_category(tc.old_tool)).or_insert(0) += 1;
                    used_tools.insert(tc.old_tool);
                    used_category.insert(self.get_filament_category(tc.old_tool));
                }
                let new_cat = self.get_filament_category(tc.new_tool);
                if !used_category.contains(&new_cat) {
                    *category_counts.entry(new_cat).or_insert(0) += 1;
                }
                if !used_tools.contains(&tc.new_tool) {
                    *filament_counts.entry(tc.new_tool).or_insert(0) += 1;
                }
                used_tools.insert(tc.new_tool);
                used_category.insert(new_cat);
            }
            current_tool = layer.tool_changes.last().map_or(current_tool, |tc| tc.new_tool);
        }
        let mut selected_category = -1;
        let mut selected_count = 0;
        for (&cat, &count) in &category_counts {
            if count > selected_count {
                selected_category = cat;
                selected_count = count;
            }
        }
        let mut filament_id = -1;
        let mut filament_count = 0;
        for (&f, &count) in &filament_counts {
            if self.get_filament_category(f) == selected_category && count > filament_count {
                filament_id = f as i32;
                filament_count = count;
            }
        }
        filament_id
    }

    /// WipeTower.cpp:7451-7499 `get_wall_filament_for_layer`: -1 on a layer
    /// without tool changes; else the wall filament when the layer loads it,
    /// else the first filament of its category the layer has. No merged fills
    /// and no printable-height limits here, so every filament fits and is
    /// valid on its last layer.
    fn get_wall_filament_for_layer_reduced(&self, layer_id: usize, wall_filament: i32) -> i32 {
        let layer = &self.plan[layer_id];
        if layer.tool_changes.is_empty() {
            return -1;
        }
        let cat = |t: usize| self.filament_params.get(t).map_or(0, |p| p.category);
        let wall_cat = if wall_filament >= 0 { cat(wall_filament as usize) } else { i32::MIN };
        let mut candidate_id = -1;
        for (idx, tc) in layer.tool_changes.iter().enumerate() {
            if idx == 0 {
                if tc.old_tool as i32 == wall_filament {
                    return wall_filament;
                }
                if cat(tc.old_tool) == wall_cat {
                    candidate_id = tc.old_tool as i32;
                }
            }
            if tc.new_tool as i32 == wall_filament {
                return wall_filament;
            }
            if candidate_id == -1 && cat(tc.new_tool) == wall_cat {
                candidate_id = tc.new_tool as i32;
            }
        }
        if candidate_id != -1 {
            return candidate_id;
        }
        layer.tool_changes[0].new_tool as i32
    }

    /// WipeTower.cpp:7501-7533 `block_fill_filament_base` and 7535-7583
    /// `get_block_fill_filament` without merged fills or per-slab filaments:
    /// the last filament the layer purges into the block, else the one that
    /// rams out of it, else the one the layer starts with when it belongs to
    /// the block, else the wall filament.
    fn get_block_fill_filament_reduced(&self, layer_id: usize, block: &WipeTowerBlock, wall_filament: i32) -> i32 {
        let layer = &self.plan[layer_id];
        let key = block.filament_adhesiveness_category;
        let mut last_filament_change = -1;
        let mut last_nozzle_change = -1;
        for tc in &layer.tool_changes {
            if self.get_filament_category(tc.new_tool) == key {
                last_filament_change = tc.new_tool as i32;
            }
            if tc.nozzle_change_depth > crate::libslic3r::EPSILON as f32
                && self.is_need_ramming(tc.old_tool, tc.new_tool, layer_id as i32)
                && self.get_filament_category(tc.old_tool) == key
            {
                last_nozzle_change = tc.old_tool as i32;
            }
        }
        let mut fill_filament =
            if last_filament_change != -1 { last_filament_change } else { last_nozzle_change };
        if fill_filament == -1
            && !layer.tool_changes.is_empty()
            && self.get_filament_category(layer.tool_changes[0].old_tool) == key
        {
            fill_filament = layer.tool_changes[0].old_tool as i32;
        }
        if fill_filament == -1 {
            fill_filament = wall_filament;
        }
        fill_filament
    }

    /// WipeTower.cpp:1655-1684 `construct_tcr(writer, priming, old_tool,
    /// is_finish, is_tool_change, purge_volume, is_contact, slab_height)`.
    ///
    /// `absolute_positions` is the reduced pipeline's interface (print.rs): it
    /// takes the positions offset by the tower position and the filament used
    /// by the writer counted here. Built by `new_from_config` the result is the
    /// C++ one, in the tower's own frame.
    #[allow(clippy::too_many_arguments)]
    fn construct_tcr(
        &mut self,
        writer: &WipeTowerWriter,
        priming: bool,
        old_tool: usize,
        is_finish: bool,
        is_tool_change: bool,
        purge_volume: f32,
        is_contact: bool,
        slab_height: f32,
    ) -> ToolChangeResult {
        let (origin, absolute) = if self.absolute_positions {
            // Track filament usage (the reduced pipeline's bookkeeping).
            if old_tool < self.used_filament_length.len() {
                self.used_filament_length[old_tool] += writer.used_filament_length;
            }
            (self.pos, true)
        } else {
            (Vec2f::zero(), false)
        };
        // :1671-1672 — start_pos = writer.start_pos_rotated();
        // end_pos = priming ? writer.pos() : writer.pos_rotated().
        let start_pos = origin + writer.start_pos_rotated();
        let end_pos_raw = if priming { writer.pos() } else { writer.pos_rotated() };
        let end_pos = origin + end_pos_raw;

        ToolChangeResult {
            print_z: self.z_pos,
            // :1669
            layer_height: if slab_height > 0.0 { slab_height } else { self.layer_height },
            perimeter_width: self.perimeter_width,
            gcode: writer.gcode().to_string(),
            extrusions: writer.extrusions().to_vec(),
            start_pos,
            end_pos,
            elapsed_time: writer.elapsed_time(),
            priming,
            is_tool_change,
            // :1679 — is_tool_change ? start_pos : Vec2f(0,0).
            tool_change_start_pos: if is_tool_change { start_pos } else { Vec2f::zero() },
            wipe_path: if absolute {
                writer.wipe_path().iter().map(|p| origin + *p).collect()
            } else {
                writer.wipe_path().to_vec()
            },
            purge_volume,
            initial_tool: old_tool as i32,
            new_tool: self.current_tool as i32,
            is_finish_first: is_finish,
            is_out_wall_only: false,
            is_contact,
            nozzle_change_result: self.nozzle_change_result.clone(),
            ..Default::default()
        }
    }

    /// WipeTower.cpp:1686-1706 `construct_block_tcr`.
    fn construct_block_tcr(
        &self,
        writer: &WipeTowerWriter,
        priming: bool,
        filament_id: usize,
        is_finish: bool,
        purge_volume: f32,
        slab_height: f32,
    ) -> ToolChangeResult {
        let origin = if self.absolute_positions { self.pos } else { Vec2f::zero() };
        let start_pos = origin + writer.start_pos_rotated();
        let end_pos = origin + if priming { writer.pos() } else { writer.pos_rotated() };
        ToolChangeResult {
            priming,
            initial_tool: filament_id as i32,
            new_tool: filament_id as i32,
            print_z: self.z_pos,
            layer_height: if slab_height > 0.0 { slab_height } else { self.layer_height },
            perimeter_width: self.perimeter_width,
            elapsed_time: writer.elapsed_time(),
            start_pos,
            end_pos,
            gcode: writer.gcode().to_string(),
            extrusions: writer.extrusions().to_vec(),
            wipe_path: writer.wipe_path().iter().map(|p| origin + *p).collect(),
            is_finish_first: is_finish,
            is_tool_change: false,
            tool_change_start_pos: Vec2f::zero(),
            purge_volume,
            nozzle_change_result: NozzleChangeResult::new_cpp(),
            ..Default::default()
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
            // WipeTower.cpp:2661 (R615).
            self.set_for_wipe_tower_writer(&mut writer);

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

            let result = self.construct_tcr(&writer, true, tool, false, false, 0.0, false, -1.0);
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
    // WipeTower.cpp:337-338
    let p1 = line.a;
    let p2 = line.b;
    // :340-341
    let dx = (p2.x() - p1.x()) as f64;
    let dy = (p2.y() - p1.y()) as f64;
    // :343 — std::sqrt(dx * dx + dy * dy), contracted.
    let length = dx.mul_add(dx, dy * dy).sqrt();
    // :345-349
    let ux = dx / length;
    let uy = dy / length;
    let vx = -uy;
    let vy = ux;
    // :351-352
    let ox = vx * offset as f64;
    let oy = vy * offset as f64;
    // :354-360 — `rect[i] = {double, double}` is Point(double, double), which
    // lrint-s each coordinate (Point.hpp:179).
    let rect = vec![
        point_from_f64(p1.x() as f64 + ox, p1.y() as f64 + oy),
        point_from_f64(p1.x() as f64 - ox, p1.y() as f64 - oy),
        point_from_f64(p2.x() as f64 - ox, p2.y() as f64 - oy),
        point_from_f64(p2.x() as f64 + ox, p2.y() as f64 + oy),
    ];
    Polygon::from_points(rect)
}

// ============================================================================
// Tests
// ============================================================================

/// WipeTower.hpp:668 `static constexpr float WT_EPSILON = 1e-3f` (the module's
/// `WT_EPSILON` above is libslic3r's `EPSILON`, 1e-4).
const WIPE_TOWER_WT_EPSILON: f32 = 1e-3;
/// WipeTower.hpp:670 `static constexpr float WT_MAX_CHAMFER_WIDTH = 3.f`
const WT_MAX_CHAMFER_WIDTH: f32 = 3.0;

/// PrintConfig.cpp `get_extruder_index(config, filament_id)`.
fn cfg_get_extruder_index(config: &WipeTowerPrintConfig, filament_id: u32) -> i32 {
    let filament_map = cfg_ints(config, "filament_map");
    if (filament_id as usize) < filament_map.len() {
        return get_at(&filament_map, filament_id as usize) - 1;
    }
    0
}

/// PrintConfig.cpp:494-502 `get_process_config_idx(config, filament_id)`.
fn cfg_get_process_config_idx(config: &WipeTowerPrintConfig, filament_id: u32) -> usize {
    use crate::config::ConfigEnum;
    use crate::print_config::enums::{ExtruderType, NozzleVolumeType};
    let volume_type = <NozzleVolumeType as ConfigEnum>::from_int(get_at(&cfg_ints(config, "filament_volume_map"), filament_id as usize));
    let extruder_id = cfg_get_extruder_index(config, filament_id);
    let extruder_type = <ExtruderType as ConfigEnum>::from_int(get_at(&cfg_ints(config, "extruder_type"), extruder_id as usize));
    let print_extruder_id = cfg_ints(config, "printer_extruder_id");
    let variant_list = cfg_strings(config, "printer_extruder_variant");
    crate::print_config::config_def::get_config_index_base(volume_type, extruder_type, extruder_id + 1, &variant_list, &print_extruder_id) as usize
}

/// PrintConfig.cpp:454-469 `nozzle_diameter_for_filament(config, filament_id)`.
fn cfg_nozzle_diameter_for_filament(config: &WipeTowerPrintConfig, filament_id: u32) -> f32 {
    if filament_id != 0 {
        let fil_idx = (filament_id - 1) as usize;
        let selected = cfg_strings(config, "filament_diameter_map");
        if fil_idx < selected.len() && !selected[fil_idx].is_empty() && !selected[fil_idx].contains(';') {
            let (diameter, _) = crate::locales_utils::string_to_double_decimal_point(&selected[fil_idx]);
            if diameter > 0.0 {
                return diameter as f32;
            }
        }
    }
    let nozzle_idx = if filament_id == 0 { 0 } else { (filament_id - 1) as usize };
    get_at(&cfg_floats(config, "nozzle_diameter"), nozzle_idx) as f32
}

impl WipeTower {
    // ------------------------------------------------------------------
    // WipeTower.hpp inline members
    // ------------------------------------------------------------------

    /// WipeTower.hpp:28 `never_skip_tag`.
    pub fn never_skip_tag() -> &'static str {
        "_GCODE_WIPE_TOWER_NEVER_SKIP_TAG"
    }

    /// WipeTower.hpp:199
    pub fn set_foundation_required(&mut self, required: bool) {
        self.foundation_required = required;
    }

    /// WipeTower.hpp:207
    pub fn set_shared_print_bed(&mut self, bed: Vec<Polygon>) {
        self.shared_print_bed = bed;
    }

    /// WipeTower.hpp:229-232 `get_outer_wall`: the walls recorded per Z, in Z order.
    pub fn get_outer_wall(&self) -> Vec<(f32, Vec<Polyline>)> {
        self.outer_wall.iter().map(|(k, v)| (f32::from_bits(*k as u32), v.clone())).collect()
    }

    /// WipeTower.hpp:234
    pub fn get_layer_height(&self) -> f32 {
        self.layer_height
    }

    /// WipeTower.hpp:235
    pub fn get_rib_length(&self) -> f32 {
        self.rib_length
    }

    /// WipeTower.hpp:236
    pub fn get_rib_width(&self) -> f32 {
        self.rib_width
    }

    /// WipeTower.hpp:306-310 `extrusion_flow(layer_height = -1)`.
    fn extrusion_flow(&self, layer_height: f32) -> f32 {
        if layer_height < 0.0 {
            return self.extrusion_flow;
        }
        layer_height * self.flow_spacing(self.perimeter_width, layer_height) / self.filament_area()
    }

    /// WipeTower.hpp:319-323 `extrusion_flow_for(layer_height, line_width)`.
    fn extrusion_flow_for(&self, layer_height: f32, line_width: f32) -> f32 {
        if layer_height < 0.0 {
            return self.extrusion_flow;
        }
        layer_height * self.extrusion_spacing_for(layer_height, line_width) / self.filament_area()
    }

    /// WipeTower.hpp:332
    pub fn get_extrusion_flow(&self) -> f32 {
        self.extrusion_flow
    }

    /// WipeTower.hpp:393
    pub fn set_used_filament_ids(&mut self, used_filament_ids: Vec<i32>) {
        self.used_filament_ids = used_filament_ids;
    }

    /// WipeTower.hpp:396
    pub fn set_multi_diameter_print(&mut self, multi_diameter: bool) {
        self.multi_diameter_print = multi_diameter;
    }

    /// WipeTower.hpp:397
    pub fn set_tower_grid_reference_nozzle(&mut self, nozzle_diameter: f32) {
        self.tower_grid_reference_nozzle = nozzle_diameter;
    }

    /// WipeTower.hpp:398
    pub fn set_tower_grid_layer_height(&mut self, layer_height: f32) {
        self.tower_grid_layer_height = layer_height;
    }

    /// WipeTower.hpp:401
    pub fn set_per_nozzle_tower_params(&mut self, per_nozzle: bool) {
        self.per_nozzle_tower_params = per_nozzle;
    }

    /// WipeTower.hpp:405
    pub fn set_filament_categories(&mut self, filament_categories: Vec<i32>) {
        self.filament_categories = filament_categories;
    }

    /// WipeTower.hpp:406 (the tower keeps a copy of the group result).
    pub fn set_nozzle_group_result(&mut self, multi_nozzle_group_result: crate::multi_nozzle_utils::LayeredNozzleGroupResult) {
        self.multi_nozzle_group_result = Some(std::sync::Arc::new(multi_nozzle_group_result));
    }

    /// WipeTower.hpp:445-449 `encode_block_key`.
    pub fn encode_block_key(category: i32, nozzle_diameter: f32, lane: i32) -> i32 {
        let diam_units = 0x3F.min((nozzle_diameter / 0.05 + 0.5) as i32);
        (0.max(category) << 10) | (diam_units << 4) | (lane & 0xF)
    }

    /// WipeTower.hpp:452 `decode_block_category`.
    pub fn decode_block_category(block_key: i32) -> i32 {
        block_key >> 10
    }

    /// WipeTower.hpp:456 `get_gap_key`.
    fn get_gap_key(&self, filament_id: i32) -> i32 {
        self.get_block_key(filament_id, -1)
    }

    /// WipeTower.hpp:487-490 `layer_is_skipped`.
    fn layer_is_skipped(&self, layer_id: i32) -> bool {
        layer_id >= 0 && (layer_id as usize) < self.layer_skipped.len() && self.layer_skipped[layer_id as usize] != 0
    }

    /// WipeTower.hpp:492-496 `layer_slab_height`.
    fn layer_slab_height(&self, layer_id: i32, fallback: f32) -> f32 {
        if layer_id >= 0
            && (layer_id as usize) < self.layer_slab_height.len()
            && self.layer_slab_height[layer_id as usize] > 0.0
        {
            self.layer_slab_height[layer_id as usize]
        } else {
            fallback
        }
    }

    /// WipeTower.hpp:582-590 `slab_height_for`.
    fn slab_height_for(&self, filament_id: i32, layer_id: i32, fallback: f32) -> f32 {
        if layer_id >= 0 && (layer_id as usize) < self.block_slab_height.len() {
            if let Some(&h) = self.block_slab_height[layer_id as usize].get(&self.get_block_key(filament_id, layer_id)) {
                if h > 0.0 {
                    return h;
                }
            }
        }
        self.layer_slab_height(layer_id, fallback)
    }

    /// WipeTower.hpp:593-596 `filament_extrusion_height`.
    fn filament_extrusion_height(&self, filament_id: i32) -> f32 {
        self.slab_height_for(filament_id, self.cur_layer_id as i32, self.layer_height)
    }

    /// WipeTower.hpp:653 `get_rib_offset`.
    pub fn get_rib_offset(&self) -> Vec2f {
        self.rib_offset
    }

    /// WipeTower.hpp:676-679 `filament_perimeter_width`.
    fn filament_perimeter_width(&self, filament_id: i32) -> f32 {
        if filament_id >= 0
            && (filament_id as usize) < self.filament_params.len()
            && self.filament_params[filament_id as usize].perimeter_width > 0.0
        {
            self.filament_params[filament_id as usize].perimeter_width
        } else {
            self.perimeter_width
        }
    }

    /// WipeTower.hpp:680-683 `filament_nozzle_change_width`.
    fn filament_nozzle_change_width(&self, filament_id: i32) -> f32 {
        if filament_id >= 0
            && (filament_id as usize) < self.filament_params.len()
            && self.filament_params[filament_id as usize].nozzle_change_perimeter_width > 0.0
        {
            self.filament_params[filament_id as usize].nozzle_change_perimeter_width
        } else {
            self.nozzle_change_perimeter_width
        }
    }

    /// WipeTower.hpp:685-689 `nozzle_based_width`.
    fn nozzle_based_width(&self, filament_id: i32) -> f32 {
        if filament_id >= 0
            && (filament_id as usize) < self.filament_params.len()
            && self.filament_params[filament_id as usize].nozzle_diameter > 0.0
        {
            return self.filament_params[filament_id as usize].nozzle_diameter * WIDTH_TO_NOZZLE_RATIO;
        }
        self.filament_perimeter_width(filament_id)
    }

    /// WipeTower.hpp:692-694 `capped_wall_grid_width`.
    fn capped_wall_grid_width(&self, filament_id: i32) -> f32 {
        // std::min(a, b): (b < a) ? b : a
        let a = self.nozzle_based_width(filament_id);
        let b = self.filament_perimeter_width(filament_id);
        if b < a { b } else { a }
    }

    /// WipeTower.hpp:810 `is_first_layer`.
    fn is_first_layer(&self) -> bool {
        self.is_first_layer_for_slab(self.layer_height)
    }

    /// WipeTower.hpp:852 `keeps_outer_walls_unshrunk`.
    fn keeps_outer_walls_unshrunk(&self) -> bool {
        self.enable_side_scrape
    }

    // ------------------------------------------------------------------
    // Small members of WipeTower.cpp
    // ------------------------------------------------------------------

    /// WipeTower.cpp:5427-5438 `get_block_key`.
    fn get_block_key(&self, filament_id: i32, layer_id: i32) -> i32 {
        let category = if filament_id >= 0 && (filament_id as usize) < self.filament_categories.len() {
            self.filament_categories[filament_id as usize]
        } else {
            0
        };
        let diameter = if filament_id >= 0 && (filament_id as usize) < self.filament_params.len() {
            self.filament_params[filament_id as usize].nozzle_diameter
        } else {
            0.0
        };
        let mut lane = 0;
        if layer_id >= 0 && (layer_id as usize) < self.lane_assignment.len() {
            if let Some(&l) = self.lane_assignment[layer_id as usize].get(&filament_id) {
                lane = l;
            }
        }
        Self::encode_block_key(category, diameter, lane)
    }

    /// WipeTower.cpp:5440-5460 `get_block_by_category(block_key, create)`: the
    /// index of the band of that key in `wipe_tower_blocks` (a pointer in C++).
    fn get_block_by_category(&mut self, block_key: i32, create: bool) -> Option<usize> {
        if let Some(idx) = self.wipe_tower_blocks.iter().position(|b| b.block_key == block_key) {
            return Some(idx);
        }
        if create {
            let mut new_block = WipeTowerBlock::new_cpp();
            new_block.block_id = self.wipe_tower_blocks.len() as i32;
            new_block.block_key = block_key;
            new_block.filament_adhesiveness_category = block_key >> 10;
            self.wipe_tower_blocks.push(new_block);
            return Some(self.wipe_tower_blocks.len() - 1);
        }
        None
    }

    /// `get_block_by_category` that only looks (the const use of the C++ one).
    fn find_block_by_category(&self, block_key: i32) -> Option<usize> {
        self.wipe_tower_blocks.iter().position(|b| b.block_key == block_key)
    }

    /// WipeTower.cpp:5462-5482 `add_depth_to_block`.
    fn add_depth_to_block(&mut self, _filament_id: i32, block_key: i32, depth: f32, is_nozzle_change: bool) {
        let layer_depth = &mut self.all_layers_depth[self.cur_layer_id];
        if let Some(item) = layer_depth.iter_mut().find(|item| item.category == block_key) {
            item.depth += depth;
            if is_nozzle_change {
                item.nozzle_change_depth += depth;
            }
        } else {
            let mut new_block = BlockDepthInfo::new_cpp();
            new_block.category = block_key;
            new_block.depth = depth;
            if is_nozzle_change {
                new_block.nozzle_change_depth += depth;
            }
            layer_depth.push(new_block);
        }
    }

    /// WipeTower.cpp:5499-5510 `set_nozzle_last_layer_id`.
    fn set_nozzle_last_layer_id(&mut self) {
        for idx in 0..self.plan.len() {
            for i in 0..self.plan[idx].tool_changes.len() {
                let old_tool = self.plan[idx].tool_changes[i].old_tool as i32;
                let new_tool = self.plan[idx].tool_changes[i].new_tool as i32;
                if old_tool >= 0 {
                    let e = self.get_extruder_id(old_tool, idx as i32);
                    self.last_layer_id[e as usize] = idx as i32;
                }
                let e = self.get_extruder_id(new_tool, idx as i32);
                self.last_layer_id[e as usize] = idx as i32;
            }
        }
    }

    /// WipeTower.cpp:5512-5515 `set_first_layer_flow_ratio`.
    pub fn set_first_layer_flow_ratio(&mut self, flow_ratio: f32) {
        self.first_layer_flow_ratio = flow_ratio;
    }

    /// WipeTower.cpp:5832-5837 `has_multiple_nozzle_diameters`.
    fn has_multiple_nozzle_diameters(&self) -> bool {
        self.multi_diameter_print
    }

    /// WipeTower.cpp:6083-6087 `merged_fill_active`.
    fn merged_fill_active(&self) -> bool {
        !self.plan.is_empty()
            && self.has_multiple_nozzle_diameters()
            && !self.sparse_layers_skipped
            && !self.enable_timelapse_print
            && !self.enable_wrapping_detection
    }

    /// WipeTower.cpp:6091-6113 `same_category_filler`.
    fn same_category_filler(&self, layer: &WipeTowerLayerInfo, block_key: i32) -> i32 {
        let group_key = block_key & !0xF;
        let block_category = Self::decode_block_category(block_key);
        let mut group_filler = -1;
        let mut category_filler = -1;
        let mut consider = |filament: i32| {
            if filament < 0 {
                return;
            }
            if self.get_gap_key(filament) == group_key {
                group_filler = filament;
            } else if category_filler < 0 && self.get_filament_category(filament as usize) == block_category {
                category_filler = filament;
            }
        };
        for tool_change in &layer.tool_changes {
            consider(tool_change.old_tool as i32);
            consider(tool_change.new_tool as i32);
        }
        if group_filler != -1 { group_filler } else { category_filler }
    }

    /// WipeTower.cpp:6115-6121 `loaded_filament`.
    fn loaded_filament(&self, layer_id: i32) -> i32 {
        let mut l = layer_id.min(self.plan.len() as i32) - 1;
        while l >= 0 {
            if !self.plan[l as usize].tool_changes.is_empty() {
                return self.plan[l as usize].tool_changes.last().unwrap().new_tool as i32;
            }
            l -= 1;
        }
        -1
    }

    /// WipeTower.cpp:6513-6520 `collect_printed_steps`.
    fn collect_printed_steps(&self, printed: &mut Vec<i32>) {
        printed.clear();
        let first_layer = self.first_layer_idx.map_or(0, |i| i as i32);
        for layer_id in first_layer..self.plan.len() as i32 {
            if !self.layer_is_skipped(layer_id) {
                printed.push(layer_id);
            }
        }
    }

    /// WipeTower.cpp:6522-6528 `tower_bed_z`.
    fn tower_bed_z(&self) -> f64 {
        if self.plan.is_empty() {
            return 0.0;
        }
        let first = self.first_layer_idx.unwrap_or(0);
        (self.plan[first].z - self.plan[first].height) as f64
    }

    /// WipeTower.cpp:5222-5226 `min_layer_height_for_filament`.
    fn min_layer_height_for_filament(&self, filament_id: i32) -> f32 {
        let diameter = if filament_id >= 0 && (filament_id as usize) < self.filament_params.len() {
            self.filament_params[filament_id as usize].nozzle_diameter
        } else {
            0.0
        };
        if diameter > 0.0 {
            crate::adaptive_layer_merge::adaptive_layer_height_range(diameter as f64).min_height as f32
        } else {
            0.0
        }
    }

    /// WipeTower.cpp:5228-5232 `max_layer_height_for_filament`.
    fn max_layer_height_for_filament(&self, filament_id: i32) -> f32 {
        let diameter = if filament_id >= 0 && (filament_id as usize) < self.filament_params.len() {
            self.filament_params[filament_id as usize].nozzle_diameter
        } else {
            0.0
        };
        if diameter > 0.0 {
            crate::adaptive_layer_merge::adaptive_layer_height_range(diameter as f64).max_height as f32
        } else {
            f32::MAX
        }
    }

    /// WipeTower.cpp:5234-5238 `filament_can_print_height`.
    fn filament_can_print_height(&self, filament_id: i32, height: f32) -> bool {
        height + WIPE_TOWER_WT_EPSILON >= self.min_layer_height_for_filament(filament_id)
            && height <= self.max_layer_height_for_filament(filament_id) + WIPE_TOWER_WT_EPSILON
    }

    /// WipeTower.cpp:5240-5247 `preferred_band_size_for_filament`.
    fn preferred_band_size_for_filament(&self, filament_id: i32) -> usize {
        let diameter = if filament_id >= 0 && (filament_id as usize) < self.filament_params.len() {
            self.filament_params[filament_id as usize].nozzle_diameter
        } else {
            0.0
        };
        let reference = if self.tower_grid_reference_nozzle > 0.0 {
            self.tower_grid_reference_nozzle
        } else {
            self.tower_grid_nozzle_diameter
        };
        crate::adaptive_layer_merge::adaptive_layer_band_size(diameter as f64, reference as f64, self.tower_grid_layer_height as f64)
    }

    /// WipeTower.cpp:5249-5254 `preferred_height_for_filament`.
    fn preferred_height_for_filament(&self, filament_id: i32) -> f32 {
        if self.tower_grid_layer_height <= WIPE_TOWER_WT_EPSILON {
            return self.min_layer_height_for_filament(filament_id);
        }
        self.preferred_band_size_for_filament(filament_id) as f32 * self.tower_grid_layer_height
    }

    /// WipeTower.cpp:7360-7368 `can_print_base_layers`.
    fn can_print_base_layers(&self, filament_id: i32) -> bool {
        if !self.merged_slabs_active {
            return true;
        }
        for layer_id in 0..self.plan.len() as i32 {
            if !self.layer_is_skipped(layer_id)
                && !self.filament_can_print_height(filament_id, self.layer_slab_height(layer_id, self.plan[layer_id as usize].height))
            {
                return false;
            }
        }
        true
    }

    /// WipeTower.cpp:7437-7449 `band_fill_depth`.
    fn band_fill_depth(&self, block: &WipeTowerBlock, layer_id: i32) -> f32 {
        if layer_id >= 0 && (layer_id as usize) < block.layer_depths.len() {
            return block.layer_depths[layer_id as usize];
        }
        block.depth
    }

    /// WipeTower.cpp:8539-8545 `is_valid_last_layer`.
    fn is_valid_last_layer(&self, tool: i32, layer_id: i32, layer_z: f64) -> bool {
        let extruder_id = self.get_extruder_id(tool, layer_id);
        if extruder_id < 0 || extruder_id as usize >= self.printable_height.len() {
            return true;
        }
        if self.last_layer_id[extruder_id as usize] == layer_id && layer_z > self.printable_height[extruder_id as usize] {
            return false;
        }
        true
    }

    /// WipeTower.cpp:8546-8557 `get_block_gap_width`.
    fn get_block_gap_width(&self, tool: i32, is_nozzlechangle: bool) -> f32 {
        let gap_key = self.get_gap_key(tool);
        match self.block_infill_gap_width.get(&gap_key) {
            None => {
                if is_nozzlechangle {
                    self.filament_nozzle_change_width(tool)
                } else {
                    self.filament_perimeter_width(tool)
                }
            }
            Some(&(first, second)) => {
                if is_nozzlechangle { second } else { first }
            }
        }
    }

    /// WipeTower.cpp:8573 `get_nozzle_id`.
    fn get_nozzle_id(&self, filament_id: i32, layer_id: i32) -> i32 {
        match &self.multi_nozzle_group_result {
            Some(g) => g.get_nozzle_id(filament_id, layer_id),
            // The reduced pipeline: one nozzle per extruder of `filament_map`.
            None => self.filament_map.get(filament_id.max(0) as usize).copied().unwrap_or(0),
        }
    }

    /// WipeTower.cpp:8575-8577 `get_extruder_id`.
    fn get_extruder_id(&self, filament_id: i32, layer_id: i32) -> i32 {
        match &self.multi_nozzle_group_result {
            Some(g) => g.get_extruder_id(filament_id, layer_id),
            None => self.filament_map.get(filament_id.max(0) as usize).copied().unwrap_or(0),
        }
    }

    /// WipeTower.cpp:3329-3332 `is_petg_filament`.
    fn is_petg_filament(&self, filament_id: i32) -> bool {
        self.filament_params[filament_id as usize].material == "PETG"
    }

    /// WipeTower.cpp:3334-3339 `is_need_reverse_travel`.
    fn is_need_reverse_travel(&self, filament_id: i32, extruder_change: bool) -> bool {
        let p = &self.filament_params[filament_id as usize];
        let eps = crate::libslic3r::EPSILON;
        if extruder_change {
            return p.ramming_travel_time.0 as f64 > eps && self.filaments_change_length.0[filament_id as usize] > eps;
        }
        p.ramming_travel_time.1 as f64 > eps && self.filaments_change_length.1[filament_id as usize] > eps
    }

    /// WipeTower.cpp:3344-3351 `first_toolchange_to_nonsoluble_nonsupport`.
    fn first_toolchange_to_nonsoluble_nonsupport(&self, tool_changes: &[ToolChangeInfo]) -> i32 {
        for (idx, tc) in tool_changes.iter().enumerate() {
            let p = &self.filament_params[tc.new_tool];
            if !p.is_soluble && !p.is_support {
                return idx as i32;
            }
        }
        -1
    }

    /// WipeTower.cpp:8476-8524 `get_floating_area`.
    fn get_floating_area(&self, start_pos_y: &mut f32, end_pos_y: &mut f32) -> bool {
        let Some(cur) = self.cur_block_idx else {
            return false;
        };
        let block = &self.wipe_tower_blocks[cur];
        if !self.merged_slabs_active {
            if self.layer_idx == 0 || self.layer_idx - 1 == 0 {
                return false;
            }
            *end_pos_y = block.start_depth + block.depth - self.perimeter_width;
            *start_pos_y = block.finish_depth[self.cur_layer_id - 1];
            return true;
        }
        let mut below = -1i32;
        let mut layer_id = self.cur_layer_id as i32 - 1;
        while layer_id >= 0 {
            if self.layer_is_skipped(layer_id) {
                layer_id -= 1;
                continue;
            }
            let purge = (layer_id as usize) < self.block_slab_height.len()
                && self.block_slab_height[layer_id as usize].contains_key(&block.block_key);
            let fill = (layer_id as usize) < block.layer_reserved.len()
                && block.layer_reserved[layer_id as usize] == 0
                && self.band_fill_depth(block, layer_id) > 2.0 * self.perimeter_width;
            if purge || fill {
                below = layer_id;
                break;
            }
            layer_id -= 1;
        }
        if below < 0 {
            return false;
        }
        let layer_depth = self.band_fill_depth(block, self.cur_layer_id as i32);
        *end_pos_y = block.start_depth + layer_depth - self.perimeter_width;
        *start_pos_y = block.finish_depth[below as usize];
        true
    }

    /// WipeTower.cpp:8526-8537 `need_thick_bridge_flow`.
    fn need_thick_bridge_flow(&self, pos_y: f32, slab_height: f32) -> bool {
        if slab_height >= 0.2 {
            return false;
        }
        let mut y_min = 0.0_f32;
        let mut y_max = 0.0_f32;
        if self.get_floating_area(&mut y_min, &mut y_max) {
            return pos_y > y_min && pos_y < y_max;
        }
        false
    }

    /// WipeTower.cpp:2201-2207 `wipe_line_spacing`.
    fn wipe_line_spacing(&self, solid_toolchange: bool) -> f32 {
        let tool_width = self.filament_perimeter_width(self.current_tool as i32);
        if solid_toolchange || self.is_first_layer() {
            return tool_width;
        }
        self.plan[self.layer_idx].extra_spacing * self.get_block_gap_width(self.current_tool as i32, false)
    }

    /// WipeTower.cpp:1761-1780 `move_box_inside_box(box1, box2, scaled_offset)`.
    pub fn move_box_inside_box(box1: &crate::geometry::BoundingBox, box2: &crate::geometry::BoundingBox, scaled_offset: i32) -> Vec2f {
        let mut res = Vec2f::zero();
        let off = scaled_offset as crate::Coord;
        let s1 = box1.size();
        let s2 = box2.size();
        if s1.x >= s2.x - 2 * off || s1.y >= s2.y - 2 * off {
            return res;
        }
        let uf = |v: crate::Coord| v as f32 * crate::libslic3r::SCALING_FACTOR as f32;
        if box1.max.x > box2.max.x - off {
            res.x = uf((box2.max.x - off) - box1.max.x);
        } else if box1.min.x < box2.min.x + off {
            res.x = uf((box2.min.x + off) - box1.min.x);
        }
        if box1.max.y > box2.max.y - off {
            res.y = uf((box2.max.y - off) - box1.max.y);
        } else if box1.min.y < box2.min.y + off {
            res.y = uf((box2.min.y + off) - box1.min.y);
        }
        res
    }

    /// WipeTower.cpp:181-247 `rounding_polygon(polygon, rounding = 2, angle_tol = 30°)`.
    pub fn rounding_polygon(polygon: &Polygon, rounding: f64, angle_tol: f64) -> Polygon {
        if polygon.points().len() < 3 {
            return polygon.clone();
        }
        let mut res = Polygon::new();
        let cos_angle_tol = angle_tol.cos().abs();
        for i in 0..polygon.points().len() {
            let (b, ab, bc, ab_len, bc_len, cosangle) = polygon_corner(polygon, i);
            let cosangle = cosangle.clamp(-1.0, 1.0);
            let is_ccw = cross2_f64(ab, bc) > 0.0;
            if (cosangle.abs() as f64) < cos_angle_tol {
                let real_rounding_dis = rounding.min(ab_len / 2.1).min(bc_len / 2.1) as f32 as f64;
                let left = (b.0 - ab.0 * real_rounding_dis, b.1 - ab.1 * real_rounding_dis);
                let right = (b.0 + bc.0 * real_rounding_dis, b.1 + bc.1 * real_rounding_dis);
                // :211 — float half_angle = std::acos(cosangle) / 2.f (float acos).
                let half_angle = cosangle.acos() / 2.0;
                // :214-216 — Vec2d dir = (right - left).normalized(), turned left.
                let dx = right.0 - left.0;
                let dy = right.1 - left.1;
                let n2 = dx * dx + dy * dy;
                let (nx, ny) = if n2 > 0.0 { let n = n2.sqrt(); (dx / n, dy / n) } else { (dx, dy) };
                let mut dir = (-ny, nx);
                if !is_ccw {
                    dir = (-dir.0, -dir.1);
                }
                // :217 — `double dis = real_rounding_dis / sin(half_angle)`: the
                // float overload of sin, a float division, stored as double.
                let dis = ((real_rounding_dis as f32) / half_angle.sin()) as f64;
                let center = (b.0 + dir.0 * dis, b.1 + dir.1 * dis);
                let radius = ((left.0 - center.0) * (left.0 - center.0) + (left.1 - center.1) * (left.1 - center.1)).sqrt();
                let arc = crate::circle::ArcSegment::with_parameters(
                    scaled_vec2d(center.0, center.1),
                    (radius / crate::libslic3r::SCALING_FACTOR) as crate::Coord as f64,
                    scaled_vec2d(left.0, left.1),
                    scaled_vec2d(right.0, right.1),
                    if is_ccw { crate::circle::ArcDirection::CounterClockwise } else { crate::circle::ArcDirection::Clockwise },
                );
                push_arc_points(&mut res, &arc);
                res.points_mut().push(scaled_vec2d(right.0, right.1));
            } else {
                res.points_mut().push(polygon.points()[i]);
            }
        }
        crate::multi_point::remove_duplicate_points(res.points_mut());
        res
    }

    /// WipeTower.cpp:1782-1822 `rib_section(width, depth, rib_length, rib_width, fillet_wall)`.
    pub fn rib_section(width: f32, depth: f32, rib_length: f32, rib_width: f32, fillet_wall: bool) -> Polygon {
        let mut pts = vec![Point::new(0, 0); 16];
        let theta = (width / depth).atan();
        let (sintheta, costheta) = sincos_f32(theta);
        let w = rib_width / 2.0;
        // float sqrt of width*width + depth*depth, contracted.
        let diag = width.mul_add(width, depth * depth).sqrt();
        let l = (rib_length - diag) / 2.0;
        let diag_dir1 = Vec2f::new(width, depth).normalized();
        let diag_dir1_perp = Vec2f::new(-diag_dir1.y, diag_dir1.x);
        let diag_dir2 = Vec2f::new(-width, depth).normalized();
        let diag_dir2_perp = Vec2f::new(-diag_dir2.y, diag_dir2.x);
        let p = [Vec2f::new(0.0, 0.0), Vec2f::new(width, 0.0), Vec2f::new(width, depth), Vec2f::new(0.0, depth)];
        let s = scaled_vec2f_pt;
        pts[0] = s(Vec2f::new(p[0].x, p[0].y + w / sintheta));
        pts[1] = s((p[0] - diag_dir1 * l) + diag_dir1_perp * w);
        pts[2] = s((p[0] - diag_dir1 * l) - diag_dir1_perp * w);
        pts[3] = s(Vec2f::new(p[0].x + w / costheta, p[0].y));

        pts[4] = s(Vec2f::new(p[1].x - w / costheta, p[1].y));
        pts[5] = s((p[1] - diag_dir2 * l) + diag_dir2_perp * w);
        pts[6] = s((p[1] - diag_dir2 * l) - diag_dir2_perp * w);
        pts[7] = s(Vec2f::new(p[1].x, p[1].y + w / sintheta));

        pts[8] = s(Vec2f::new(p[2].x, p[2].y - w / sintheta));
        pts[9] = s((p[2] + diag_dir1 * l) - diag_dir1_perp * w);
        pts[10] = s((p[2] + diag_dir1 * l) + diag_dir1_perp * w);
        pts[11] = s(Vec2f::new(p[2].x - w / costheta, p[2].y));

        pts[12] = s(Vec2f::new(p[3].x + w / costheta, p[3].y));
        pts[13] = s((p[3] + diag_dir2 * l) - diag_dir2_perp * w);
        pts[14] = s((p[3] + diag_dir2 * l) + diag_dir2_perp * w);
        pts[15] = s(Vec2f::new(p[3].x, p[3].y - w / sintheta));
        let mut res = Polygon::from_points(pts);
        crate::multi_point::remove_duplicate_points(res.points_mut());
        if fillet_wall {
            res = Self::rounding_polygon(&res, 2.0, 30.0 / 180.0 * std::f64::consts::PI);
        }
        res
    }

    /// WipeTower.cpp:1824-1852 `its_make_rib_tower`: the mesh of the ribbed
    /// tower (`TriangleMesh::its`; the C++ fills `its` without the stats). The
    /// caps come from `Triangulation::triangulate`, whose port is blocked
    /// (triangulation.rs), so they are empty until it lands.
    pub fn its_make_rib_tower(width: f32, depth: f32, height: f32, rib_length: f32, rib_width: f32, fillet_wall: bool) -> crate::normal_utils::indexed_triangle_set {
        let mut res = crate::normal_utils::indexed_triangle_set::default();
        let bottom = Self::rib_section(width, depth, rib_length, rib_width, fillet_wall);
        let top = Self::rib_section(width, depth, width.mul_add(width, depth * depth).sqrt(), rib_width, fillet_wall);
        let offset = bottom.points().len() as i32;
        let eps = crate::scaled(crate::libslic3r::EPSILON) as f64;
        if bottom.area() < eps || top.area() < eps || bottom.points().len() != top.points().len() {
            return res;
        }
        let faces_bottom = crate::triangulation::triangulate_polygon(&bottom).unwrap_or_default();
        let faces_top = crate::triangulation::triangulate_polygon(&top).unwrap_or_default();
        for t in &faces_bottom {
            res.indices.push(crate::normal_utils::Vec3crd::new(t[1], t[0], t[2]));
        }
        for t in &faces_top {
            res.indices.push(crate::normal_utils::Vec3crd::new(t[0] + offset, t[1] + offset, t[2] + offset));
        }
        let uf = |v: crate::Coord| v as f32 * crate::libslic3r::SCALING_FACTOR as f32;
        for p in bottom.points() {
            res.vertices.push(crate::normal_utils::Vec3f::new(uf(p.x), uf(p.y), 0.0));
        }
        for p in top.points() {
            res.vertices.push(crate::normal_utils::Vec3f::new(uf(p.x), uf(p.y), height));
        }
        for i in 0..offset {
            let a = i;
            let b = (i + 1) % offset;
            let c = i + offset;
            let d = b + offset;
            res.indices.push(crate::normal_utils::Vec3crd::new(a, b, c));
            res.indices.push(crate::normal_utils::Vec3crd::new(d, c, b));
        }
        res
    }

    /// WipeTower.cpp:1854-1876 `its_make_rib_brim` (caps as in
    /// `its_make_rib_tower`).
    pub fn its_make_rib_brim(brim: &Polygon, layer_height: f32) -> crate::normal_utils::indexed_triangle_set {
        let mut res = crate::normal_utils::indexed_triangle_set::default();
        if brim.area() < crate::scaled(crate::libslic3r::EPSILON) as f64 {
            return res;
        }
        let offset = brim.points().len() as i32;
        let faces = crate::triangulation::triangulate_polygon(brim).unwrap_or_default();
        for t in &faces {
            res.indices.push(crate::normal_utils::Vec3crd::new(t[1], t[0], t[2]));
        }
        for t in &faces {
            res.indices.push(crate::normal_utils::Vec3crd::new(t[0] + offset, t[1] + offset, t[2] + offset));
        }
        let uf = |v: crate::Coord| v as f32 * crate::libslic3r::SCALING_FACTOR as f32;
        for p in brim.points() {
            res.vertices.push(crate::normal_utils::Vec3f::new(uf(p.x), uf(p.y), 0.0));
        }
        for p in brim.points() {
            res.vertices.push(crate::normal_utils::Vec3f::new(uf(p.x), uf(p.y), layer_height));
        }
        for i in 0..offset {
            let a = i;
            let b = (i + 1) % offset;
            let c = i + offset;
            let d = b + offset;
            res.indices.push(crate::normal_utils::Vec3crd::new(a, b, c));
            res.indices.push(crate::normal_utils::Vec3crd::new(d, c, b));
        }
        res
    }
}

impl WipeTower {
    /// Whether this tower was built by `new_from_config` (the C++ constructor)
    /// and so runs the C++ generator; the reduced pipeline's towers come from
    /// `new`.
    fn is_cpp_instance(&self) -> bool {
        !self.absolute_positions
    }

    /// WipeTower.cpp:1879-2013 — the C++ constructor `WipeTower(config,
    /// plate_idx, plate_origin, initial_tool, wipe_tower_height,
    /// slice_used_filaments)`. `config` is a full print configuration. The
    /// tower it makes gives its results in its own frame, as C++ does.
    pub fn new_from_config(
        config: &WipeTowerPrintConfig,
        plate_idx: i32,
        plate_origin: (f64, f64, f64),
        initial_tool: usize,
        wipe_tower_height: f32,
        slice_used_filaments: &[u32],
    ) -> Self {
        let process_idx = cfg_get_process_config_idx(config, initial_tool as u32);
        let wipe_tower_x = cfg_floats(config, "wipe_tower_x");
        let wipe_tower_y = cfg_floats(config, "wipe_tower_y");
        let nozzle_diameter = cfg_floats(config, "nozzle_diameter");
        let gcode_flavor = {
            use crate::config::ConfigOptionResolver;
            let opt = config.option("gcode_flavor").expect("gcode_flavor");
            GCodeFlavor::from_config_value(&opt.serialize().unwrap_or_default())
        };
        // :1888-1898 and the reduced pipeline's view of the same values.
        let mut cfg = WipeTowerConfig::default();
        cfg.pos_x = get_at(&wipe_tower_x, plate_idx as usize) as f32;
        cfg.pos_y = get_at(&wipe_tower_y, plate_idx as usize) as f32;
        cfg.width = cfg_float(config, "prime_tower_width") as f32;
        cfg.depth = 0.0;
        cfg.height = wipe_tower_height;
        cfg.brim_width = cfg_float(config, "prime_tower_brim_width") as f32;
        cfg.rotation_angle = cfg_float(config, "wipe_tower_rotation_angle") as f32;
        cfg.semm = config.opt_bool("single_extruder_multi_material");
        cfg.gcode_flavor = gcode_flavor;
        cfg.travel_speed = get_at(&cfg_floats(config, "travel_speed"), process_idx) as f32;
        cfg.bridging = 10.0;
        cfg.no_sparse_layers = wipe_tower_sparse_layers_skipped(config);
        cfg.timelapse_smooth = cfg_enum(config, "timelapse_type") == 1;
        cfg.enable_wrapping_detection = config.opt_bool("enable_wrapping_detection");
        cfg.wrapping_detection_layers = (config.opt_int("wrapping_detection_layers") != 0
            && config.opt::<crate::config::ConfigOptionPoints>("wrapping_exclude_area").map_or(0, |o| o.values.len()) > 2)
            as i32;
        cfg.is_multi_extruder = nozzle_diameter.len() > 1;
        cfg.use_gap_wall = config.opt_bool("prime_tower_skip_points");
        cfg.use_rib_wall = config.opt_bool("prime_tower_rib_wall");
        cfg.extra_rib_length = cfg_float(config, "prime_tower_extra_rib_length") as f32;
        cfg.rib_width = cfg_float(config, "prime_tower_rib_width") as f32;
        cfg.use_fillet = config.opt_bool("prime_tower_fillet_wall");
        cfg.extra_spacing = cfg_float(config, "prime_tower_infill_gap") as f32 / 100.0;
        cfg.tower_framework = config.opt_bool("prime_tower_enable_framework");
        cfg.max_speed = cfg_float(config, "prime_tower_max_speed") as f32;
        cfg.accel_to_decel_enable = config.opt_bool("accel_to_decel_enable");
        cfg.accel_to_decel_factor = cfg_float(config, "accel_to_decel_factor") as f32;
        cfg.enable_arc_fitting = config.opt_bool("enable_arc_fitting");
        cfg.flat_ironing = config.opt_bool("prime_tower_flat_ironing");
        cfg.physical_extruder_map = cfg_ints(config, "physical_extruder_map");
        cfg.enable_timelapse_print = cfg.timelapse_smooth;
        cfg.only_generate_wall = false;
        cfg.filament_categories = vec![];

        let mut wt = WipeTower::new(cfg, initial_tool, 0);
        wt.absolute_positions = false;
        wt.filament_params.clear();
        wt.used_filament_length.clear();
        wt.last_layer_id.clear();
        wt.filament_map.clear();
        wt.old_temperature = -1;
        wt.tpu_fixed_spacing = 2.0;
        wt.internal_rotation = 0.0;
        wt.extra_spacing = wt.config.extra_spacing;
        wt.wipe_tower_brim_width_cpp = wt.config.brim_width;
        wt.brim_width_real = 0.0;
        wt.depth = 0.0;
        wt.layer_height = 0.0;
        wt.extrusion_flow = 0.038;
        wt.perimeter_width = 0.4 * WIDTH_TO_NOZZLE_RATIO;
        wt.nozzle_change_perimeter_width = 0.4 * WIDTH_TO_NOZZLE_RATIO;
        wt.sparse_layers_skipped = wt.config.no_sparse_layers;
        wt.slice_used_filaments = slice_used_filaments.len() as i32;
        wt.max_speed = cfg_float(config, "prime_tower_max_speed") as f32 * 60.0;
        wt.tail_speed_factor = (cfg_float(config, "prime_tower_tail_speed_factor") as f32 / 100.0).clamp(0.1, 1.0);
        wt.tail_slowdown_volume = 0.0_f32.max(cfg_float(config, "prime_tower_tail_slowdown_volume") as f32);
        wt.printable_height = cfg_floats(config, "extruder_printable_height");
        wt.flat_ironing = config.opt_bool("prime_tower_flat_ironing");
        wt.line_ironing = config.opt_bool("prime_tower_line_ironing");
        wt.ironing_speed = cfg_float(config, "prime_tower_ironing_speed") as f32 * 60.0;
        wt.enable_side_scrape = config.opt_bool("prime_tower_side_scrape");
        // sssfLeftRight = 0 (PrintConfig.hpp:251-252 `SideScrapeFace`).
        wt.side_scrape_on_left_right = cfg_enum(config, "prime_tower_side_scrape_face") == 0;
        wt.side_scrape_length = cfg_float(config, "prime_tower_side_scrape_length") as f32;
        wt.side_scrape_height = cfg_float(config, "prime_tower_side_scrape_height") as f32;
        wt.side_scrape_speed = cfg_float(config, "prime_tower_side_scrape_speed") as f32 * 60.0;
        wt.side_scrape_wall_gap = cfg_float(config, "prime_tower_side_scrape_wall_gap") as f32;
        wt.side_scrape_point_spacing = cfg_float(config, "prime_tower_side_scrape_point_spacing") as f32;
        wt.side_scrape_along_length = cfg_float(config, "prime_tower_side_scrape_along_length") as f32;
        wt.enable_tower_interface_features = config.opt_bool("enable_tower_interface_features");
        wt.physical_extruder_map = cfg_ints(config, "physical_extruder_map");
        wt.enable_arc_fitting = config.opt_bool("enable_arc_fitting");
        wt.has_filament_switcher = config.opt_bool("has_filament_switcher");
        wt.handle_hotend_as_extruder = config.opt_bool("handle_hotend_as_extruder");
        wt.travel_speed = wt.config.travel_speed;
        wt.gcode_flavor = gcode_flavor;
        // :1931-1940
        wt.contact_speed = 20.0 * 60.0;
        wt.filaments_change_length = (cfg_floats(config, "filament_change_length"), cfg_floats(config, "filament_change_length_nc"));
        wt.hotend_heating_rate = cfg_floats(config, "hotend_heating_rate");
        wt.hotend_cooling_rate = cfg_floats(config, "hotend_cooling_rate");
        {
            let mx = cfg_floats(config, "machine_max_speed_x");
            let my = cfg_floats(config, "machine_max_speed_y");
            let max_x = if mx.is_empty() { 500.0_f32 } else { mx[0] as f32 };
            let max_y = if my.is_empty() { 500.0_f32 } else { my[0] as f32 };
            wt.machine_max_speed = max_x.min(max_y) * 60.0;
        }
        // :1941
        wt.flat_ironing = wt.flat_ironing && wt.use_gap_wall;
        // :1942-1967 the accelerations, `(unsigned int) floor(value + 0.5)`.
        let round = |v: f64| (v + 0.5).floor() as u32;
        wt.normal_accels.clear();
        let prime_tower_acceleration = cfg_float(config, "prime_tower_acceleration");
        let default_acceleration = cfg_floats(config, "default_acceleration");
        if prime_tower_acceleration > 0.0 {
            let tower_accel = round(prime_tower_acceleration);
            for _ in 0..default_acceleration.len() {
                wt.normal_accels.push(tower_accel);
            }
        } else {
            for v in &default_acceleration {
                wt.normal_accels.push(round(*v));
            }
        }
        wt.first_layer_normal_accels = cfg_floats(config, "initial_layer_acceleration").iter().map(|v| round(*v)).collect();
        wt.travel_accels = cfg_floats(config, "travel_acceleration").iter().map(|v| round(*v)).collect();
        wt.first_layer_travel_accels = cfg_floats(config, "initial_layer_travel_acceleration").iter().map(|v| round(*v)).collect();
        // :1967 — values.front() converted to unsigned.
        wt.max_accels = cfg_floats(config, "machine_max_acceleration_extruding")[0] as u32;
        wt.accel_to_decel_enable = wt.config.accel_to_decel_enable;
        wt.accel_to_decel_factor = wt.config.accel_to_decel_factor;
        // :1972-1975
        let default_speed = 60.0_f32;
        wt.first_layer_speed = get_at(&cfg_floats(config, "initial_layer_speed"), process_idx) as f32;
        if wt.first_layer_speed == 0.0 {
            wt.first_layer_speed = default_speed / 2.0;
        }
        wt.config.first_layer_speed = wt.first_layer_speed;
        // :1990-2009 the bed.
        let bed_points = config.opt::<crate::config::ConfigOptionPoints>("printable_area").map(|o| o.values.clone()).unwrap_or_default();
        let (mut minx, mut miny, mut maxx, mut maxy) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in &bed_points {
            minx = minx.min(p.x);
            miny = miny.min(p.y);
            maxx = maxx.max(p.x);
            maxy = maxy.max(p.y);
        }
        wt.bed_width = (maxx - minx) as f32;
        wt.bed_shape = if bed_points.len() == 4 { BedShape::Rectangular } else { BedShape::Circular };
        if wt.bed_shape == BedShape::Circular {
            let r2 = (wt.bed_width as f64 / 2.0).powf(2.0);
            let lim2 = (wt.bed_width as f64 / 10.0).powf(2.0);
            let center = ((minx + maxx) / 2.0, (miny + maxy) / 2.0);
            for pt in &bed_points {
                if ((pt.x - center.0).powf(2.0) + (pt.y - center.1).powf(2.0) - r2).abs() > lim2 {
                    wt.bed_shape = BedShape::Custom;
                    break;
                }
            }
        }
        wt.bed_bottom_left = if wt.bed_shape == BedShape::Rectangular {
            Vec2f::new(bed_points[0].x as f32, bed_points[0].y as f32)
        } else {
            Vec2f::zero()
        };
        // :2010-2012
        wt.last_layer_id = vec![-1; nozzle_diameter.len()];
        wt.origin = Vec2f::new(plate_origin.0 as f32, plate_origin.1 as f32);
        wt.is_multiple_nozzle = cfg_ints(config, "extruder_max_nozzle_count").iter().any(|&e| e > 1);
        // The members whose C++ name the reduced pipeline keeps in `config`.
        wt.enable_wrapping_detection = wt.config.enable_wrapping_detection;
        wt.wrapping_detection_layers = wt.config.wrapping_detection_layers;
        wt.enable_timelapse_print = wt.config.timelapse_smooth;
        wt.semm = wt.config.semm;
        wt.wipe_tower_width = wt.config.width;
        wt.wipe_tower_height = wt.config.height;
        wt.wipe_tower_rotation_angle = wt.config.rotation_angle;
        wt.is_multi_extruder = wt.config.is_multi_extruder;
        wt.use_gap_wall = wt.config.use_gap_wall;
        wt.use_rib_wall = wt.config.use_rib_wall;
        wt.rib_width = wt.config.rib_width;
        wt.extra_rib_length = wt.config.extra_rib_length;
        wt.used_fillet = wt.config.use_fillet;
        wt.tower_framework = wt.config.tower_framework;
        wt.bridging = 10.0;
        wt.pos = Vec2f::new(wt.config.pos_x, wt.config.pos_y);
        wt.current_tool = initial_tool;
        // WipeTower.hpp:726 `m_nozzle_change_result` value-initialised.
        wt.nozzle_change_result = NozzleChangeResult::new_cpp();
        wt
    }

    /// WipeTower.cpp:2017-2170 — the C++ `set_extruder(idx, config, config_idx)`
    /// (`set_extruder` keeps the reduced pipeline's signature).
    pub fn set_extruder_config(&mut self, idx: usize, config: &WipeTowerPrintConfig, config_idx: usize) {
        use crate::config::ConfigOptionResolver;
        self.filament_params.push(FilamentParameters::new_cpp());
        let floats = |key: &str| cfg_floats(config, key);
        let ints = |key: &str| cfg_ints(config, key);
        // Config.hpp:888 / 1054 `is_nil(idx)`: `values[idx]` unchecked in C++;
        // past the end (undefined there) it reads as not nil here.
        let nil_f = |v: &[f64], i: usize| v.get(i).is_some_and(|x| x.is_nan());
        let nil_i = |v: &[i32], i: usize| v.get(i).is_some_and(|x| *x == i32::MAX);
        let p = &mut self.filament_params[idx];
        p.material = get_at(&cfg_strings(config, "filament_type"), idx);
        p.is_soluble = get_at(&cfg_bools(config, "filament_soluble"), idx);
        p.is_support = get_at(&cfg_bools(config, "filament_is_support"), idx);
        p.nozzle_temperature = get_at(&ints("nozzle_temperature"), config_idx);
        p.nozzle_temperature_initial_layer = get_at(&ints("nozzle_temperature_initial_layer"), config_idx);
        p.category = get_at(&ints("filament_adhesiveness_category"), idx);
        p.flat_iron_area = get_at(&floats("filament_tower_ironing_area"), idx) as f32;
        // :2047 — float((M_PI/4.f) * pow(diameter, 2)): M_PI is a double.
        p.filament_area = (std::f64::consts::PI / 4.0 * get_at(&floats("filament_diameter"), idx).powf(2.0)) as f32;
        let mut nozzle_diameter = cfg_nozzle_diameter_for_filament(config, (idx + 1) as u32);
        if nozzle_diameter <= 0.0 {
            nozzle_diameter = get_at(&floats("nozzle_diameter"), idx) as f32;
        }
        p.nozzle_diameter = nozzle_diameter;
        let ptlw = cfg_float(config, "prime_tower_line_width");
        let ptrlw = cfg_float(config, "prime_tower_ramming_line_width");
        let ptff = cfg_float(config, "prime_tower_feedrate_factor");
        let line_width = if self.per_nozzle_tower_params {
            crate::print_config::config_def::multi_nozzle_prime_tower_line_width(nozzle_diameter as f64, ptlw) as f32
        } else {
            ptlw as f32
        };
        let ramming_line_width = if self.per_nozzle_tower_params {
            crate::print_config::config_def::multi_nozzle_prime_tower_ramming_line_width(nozzle_diameter as f64, ptrlw) as f32
        } else {
            ptrlw as f32
        };
        let feedrate_factor = if self.per_nozzle_tower_params {
            crate::print_config::config_def::multi_nozzle_prime_tower_feedrate_factor(nozzle_diameter as f64, ptff) as f32
        } else {
            ptff as f32
        };
        let p = &mut self.filament_params[idx];
        p.perimeter_width = if line_width > 0.0 { line_width } else { nozzle_diameter * WIDTH_TO_NOZZLE_RATIO };
        p.nozzle_change_perimeter_width =
            if ramming_line_width > 0.0 { ramming_line_width } else { nozzle_change_width_for_diameter(nozzle_diameter) };
        p.feedrate_factor = 0.01_f32.max(feedrate_factor);
        if self.tower_grid_nozzle_diameter <= 0.0 || nozzle_diameter < self.tower_grid_nozzle_diameter {
            self.tower_grid_nozzle_diameter = nozzle_diameter;
            self.perimeter_width = self.filament_params[idx].perimeter_width;
            self.nozzle_change_perimeter_width = self.filament_params[idx].nozzle_change_perimeter_width;
        }
        let idx_filament_area = self.filament_params[idx].filament_area;
        let max_vol_speed = get_at(&floats("filament_max_volumetric_speed"), config_idx) as f32;
        if max_vol_speed != 0.0 {
            self.filament_params[idx].max_e_speed = max_vol_speed / idx_filament_area;
        }
        {
            let rvs = floats("filament_ramming_volumetric_speed");
            let mut ramming_vol_speed = get_at(&rvs, config_idx) as f32;
            if nil_f(&rvs, config_idx) || (get_at(&rvs, config_idx) - -1.0).abs() < crate::libslic3r::EPSILON {
                ramming_vol_speed = max_vol_speed;
            }
            self.filament_params[idx].max_e_ramming_speed.0 = ramming_vol_speed / idx_filament_area;
            let rvs_nc = floats("filament_ramming_volumetric_speed_nc");
            let mut ramming_vol_speed_nc = get_at(&rvs_nc, config_idx) as f32;
            if nil_f(&rvs_nc, config_idx) || (get_at(&rvs_nc, config_idx) - -1.0).abs() < crate::libslic3r::EPSILON {
                ramming_vol_speed_nc = max_vol_speed;
            }
            self.filament_params[idx].max_e_ramming_speed.1 = ramming_vol_speed_nc / idx_filament_area;
        }
        {
            let extruder_count = self.multi_nozzle_group_result.as_ref().map_or(1, |g| g.get_extruder_count()).max(0) as usize;
            let p = &mut self.filament_params[idx];
            p.precool_t.0.resize(extruder_count, 0.0);
            p.precool_t_first_layer.0.resize(extruder_count, 0.0);
            p.precool_t.1.resize(extruder_count, 0.0);
            p.precool_t_first_layer.1.resize(extruder_count, 0.0);
            p.precool_target_temp.0 = 0;
            p.precool_target_temp.1 = 0;
            let nti = ints("nozzle_temperature_initial_layer");
            let nt = ints("nozzle_temperature");
            let nozzle_temp_first_layer = if nil_i(&nti, config_idx) { -1.0 } else { get_at(&nti, config_idx) as f32 };
            let nozzle_temp_other_layer = if nil_i(&nt, config_idx) { -1.0 } else { get_at(&nt, config_idx) as f32 };
            let hotend_cooling_rates = floats("hotend_cooling_rate");
            let pct = ints("filament_pre_cooling_temperature");
            let pct_nc = ints("filament_pre_cooling_temperature_nc");
            let enable_pre_heating = config.opt_bool("enable_pre_heating");
            let is_need_precooling = |extruder_change: bool| -> bool {
                let res = enable_pre_heating;
                if extruder_change {
                    return res && !nil_i(&pct, config_idx) && get_at(&pct, config_idx) != 0;
                }
                res && !nil_i(&pct_nc, config_idx) && get_at(&pct_nc, config_idx) != 0
            };
            if is_need_precooling(true) {
                for i in 0..p.precool_t.0.len() {
                    if nil_f(&hotend_cooling_rates, i) {
                        continue;
                    }
                    let t = get_at(&pct, config_idx) as f32;
                    p.precool_t.0[i] = 0.0_f32.max(nozzle_temp_other_layer - t) / hotend_cooling_rates[i] as f32;
                    p.precool_t_first_layer.0[i] = 0.0_f32.max(nozzle_temp_first_layer - t) / hotend_cooling_rates[i] as f32;
                }
                p.precool_target_temp.0 = get_at(&pct, config_idx);
            }
            if is_need_precooling(false) {
                for i in 0..p.precool_t.1.len() {
                    if nil_f(&hotend_cooling_rates, i) {
                        continue;
                    }
                    let t = get_at(&pct_nc, config_idx) as f32;
                    p.precool_t.1[i] = 0.0_f32.max(nozzle_temp_other_layer - t) / hotend_cooling_rates[i] as f32;
                    p.precool_t_first_layer.1[i] = 0.0_f32.max(nozzle_temp_first_layer - t) / hotend_cooling_rates[i] as f32;
                }
                p.precool_target_temp.1 = get_at(&pct_nc, config_idx);
            }
        }
        {
            let p = &mut self.filament_params[idx];
            p.ramming_travel_time = (0.0, 0.0);
            let rtt = floats("filament_ramming_travel_time");
            let rtt_nc = floats("filament_ramming_travel_time_nc");
            if !nil_f(&rtt, config_idx) {
                p.ramming_travel_time.0 = get_at(&rtt, config_idx) as f32;
            }
            if !nil_f(&rtt_nc, config_idx) {
                p.ramming_travel_time.1 = get_at(&rtt_nc, config_idx) as f32;
            }
            p.idle_travel_cooling_enabled = get_at(&cfg_bools(config, "filament_idle_travel_cooling_enabled"), idx);
            let filament_curve_str = get_at(&cfg_strings(config, "filament_prime_tower_speed_factor_curve"), idx);
            p.speed_factor_curve = parse_doubles(&filament_curve_str);
        }
        if self.used_filament_length.len() < idx + 1 {
            self.used_filament_length.resize(idx + 1, 0.0);
        }
        let enable_tif = config.opt_bool("enable_tower_interface_features");
        let p = &mut self.filament_params[idx];
        p.retract_length = get_at(&floats("retraction_length"), config_idx) as f32;
        p.retract_speed = get_at(&floats("retraction_speed"), config_idx) as f32;
        p.wipe_dist = get_at(&floats("wipe_distance"), config_idx) as f32;
        p.filament_cooling_before_tower = get_at(&floats("filament_cooling_before_tower"), config_idx) as f32;
        p.filament_petg_pre_extrusion_offset_dist = get_at(&floats("filament_tower_interface_pre_extrusion_dist"), idx) as f32;
        if enable_tif {
            let tipt = ints("filament_tower_interface_print_temp");
            p.filament_tower_interface_print_temp = if get_at(&tipt, idx) == -1 {
                get_at(&ints("nozzle_temperature_range_high"), idx) as f32
            } else {
                get_at(&tipt, idx) as f32
            };
            p.filament_tower_interface_pre_extrusion_dist = get_at(&floats("filament_tower_interface_pre_extrusion_dist"), idx) as f32;
            p.filament_tower_interface_pre_extrusion_length = get_at(&floats("filament_tower_interface_pre_extrusion_length"), idx) as f32;
        } else {
            p.filament_tower_interface_print_temp = get_at(&ints("nozzle_temperature"), config_idx) as f32;
            p.filament_tower_interface_pre_extrusion_dist = 0.0;
            p.filament_tower_interface_pre_extrusion_length = 0.0;
        }
        let _ = config.option("filament_type");
    }

    /// WipeTower.cpp:2172-2184 `set_single_diameter_parameters`. C++ throws a
    /// RuntimeError when no filament carries the nozzle; this returns it.
    pub fn set_single_diameter_parameters(&mut self, nozzle_diameter: f32) -> Result<(), String> {
        for filament in &self.filament_params {
            if ((filament.nozzle_diameter - nozzle_diameter).abs() as f64) < crate::libslic3r::EPSILON {
                self.tower_grid_nozzle_diameter = filament.nozzle_diameter;
                self.perimeter_width = filament.perimeter_width;
                self.nozzle_change_perimeter_width = filament.nozzle_change_perimeter_width;
                return Ok(());
            }
        }
        Err(format!(
            "Single-diameter layering nozzle {:.6} is not used by any filament of the wipe tower.",
            nozzle_diameter as f64
        ))
    }
}

// ============================================================================
// The C++ generator: tool changes (WipeTower.cpp:2209-2878, 3542-4055, 4660-5221)
// ============================================================================
/// `std::min(a, b)` on floats: `(b < a) ? b : a`.
#[inline]
fn fa_min(a: f32, b: f32) -> f32 {
    if b < a { b } else { a }
}

/// `std::max(a, b)` on floats: `(a < b) ? b : a`.
#[inline]
fn fa_max(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}

/// `std::clamp(v, lo, hi)`: `(v < lo) ? lo : (hi < v) ? hi : v`.
#[inline]
fn fa_clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}

/// The `format_nozzle_change_line` lambdas of WipeTower.cpp:2448-2453,
/// 3755-3762, 3924-3932, 3946-3954: `;<NOZZLE_CHANGE_START|END> OF.. NF..`
/// (with the nozzle ids `ON.. NN..` when given).
fn fa_nozzle_change_line(start: bool, old_filament_id: i32, new_filament_id: i32, nozzle_ids: Option<(i32, i32)>) -> String {
    let tag = if start { " NOZZLE_CHANGE_START" } else { " NOZZLE_CHANGE_END" };
    match nozzle_ids {
        Some((old_nozzle_id, new_nozzle_id)) => {
            format!(";{} OF{} NF{} ON{} NN{}\n", tag, old_filament_id, new_filament_id, old_nozzle_id, new_nozzle_id)
        }
        None => format!(";{} OF{} NF{}\n", tag, old_filament_id, new_filament_id),
    }
}

/// `(unsigned int)(-1)`: the "unload only" tool of the C++ tool changes.
const FA_NO_TOOL: usize = u32::MAX as usize;

impl WipeTower {
    /// WipeTower.cpp:2209-2261 `get_next_pos(cleaning_box, wipe_length,
    /// solid_toolchange)`: where the purge zigzag starts, rotating with the
    /// layer through ld, rd + top, rd, ld + top; with the contact / PETG
    /// pre-extrusion moved outwards and kept on the shared bed.
    fn get_next_pos(&self, cleaning_box: &BoxCoordinates, wipe_length: f32, solid_toolchange: bool) -> Vec2f {
        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;
        let dy = self.wipe_line_spacing(solid_toolchange);
        let box_depth = cleaning_box.lu.y - cleaning_box.ld.y;
        let mut y_offset = fa_max(0.0, box_depth - dy);
        if !solid_toolchange && xr > xl {
            let line_count = 1.max((wipe_length / (xr - xl)).ceil() as i32);
            // N rows contain N - 1 vertical steps (:2218-2220).
            y_offset = fa_min((line_count - 1) as f32 * dy, y_offset);
        }
        let pos_offset = Vec2f::new(0.0, self.depth_traversed);

        let index = self.cur_layer_id % 4;
        let offset = Vec2f::new(0.0, 0.0);
        let neg_offset = Vec2f::new(-offset.x, -offset.y);
        let mut res = Vec2f::zero();
        match index % 4 {
            0 => res = (offset + cleaning_box.ld) + pos_offset,
            1 => res = ((neg_offset + cleaning_box.rd) + pos_offset) + Vec2f::new(0.0, y_offset),
            2 => res = (neg_offset + cleaning_box.rd) + pos_offset,
            3 => res = ((offset + cleaning_box.ld) + pos_offset) + Vec2f::new(0.0, y_offset),
            _ => {}
        }
        let is_contact_pre_extrusion = solid_toolchange && self.enable_tower_interface_features;
        let is_petg_pre_extrusion =
            !is_contact_pre_extrusion && self.is_petg_filament(self.current_tool as i32) && self.has_filament_switcher;
        if is_contact_pre_extrusion || is_petg_pre_extrusion {
            let mut stop_pos = res;
            let p = &self.filament_params[self.current_tool];
            let filament_tower_interface_pre_extrusion_dist = if is_petg_pre_extrusion {
                p.filament_petg_pre_extrusion_offset_dist
            } else {
                p.filament_tower_interface_pre_extrusion_dist
            };
            // :2250-2251 — unscaled(get_extents(m_shared_print_bed)), translated by
            // -(m_wipe_tower_pos + m_rib_offset) (float vector cast to double).
            let (mut min_x, mut max_x) = (0.0_f64, 0.0_f64);
            let mut first = true;
            for pg in &self.shared_print_bed {
                for p in pg.points() {
                    if first {
                        min_x = p.x as f64;
                        max_x = p.x as f64;
                        first = false;
                    } else {
                        min_x = min_x.min(p.x as f64);
                        max_x = max_x.max(p.x as f64);
                    }
                }
            }
            let sf = crate::libslic3r::SCALING_FACTOR;
            let shift = Vec2f::new(-self.pos.x - self.rib_offset.x, -self.pos.y - self.rib_offset.y);
            let bbx_min = min_x * sf + shift.x as f64;
            let bbx_max = max_x * sf + shift.x as f64;
            if stop_pos.x < self.wipe_tower_width / 2.0 {
                stop_pos = Vec2f::new(stop_pos.x - filament_tower_interface_pre_extrusion_dist, stop_pos.y);
            } else {
                stop_pos = Vec2f::new(stop_pos.x + filament_tower_interface_pre_extrusion_dist, stop_pos.y);
            }
            if (stop_pos.x as f64) < bbx_min {
                stop_pos.x = bbx_min as f32;
            }
            if (stop_pos.x as f64) > bbx_max {
                stop_pos.x = bbx_max as f32;
            }
            res = stop_pos;
        }
        res
    }

    /// The Y shift both tool changes hand to their writer (WipeTower.cpp:2300,
    /// 3609, 3775, 3994): `m_y_shift + (tool valid && SHAPE_REVERSED ?
    /// depth - toolchanges_depth : 0)`.
    fn fa_writer_y_shift(&self, tool_valid: bool) -> f32 {
        let extra = if tool_valid && self.current_shape == WipeShape::Reversed {
            let layer = &self.plan[self.layer_idx];
            layer.depth - layer.toolchanges_depth()
        } else {
            0.0
        };
        self.y_shift + extra
    }

    /// WipeTower.cpp:2263-2422 `tool_change(tool, extrude_perimeter,
    /// first_toolchange_to_nonsoluble)`: the legacy tool change (only the
    /// compiled-out legacy `generate` calls it in the pin).
    pub fn tool_change(&mut self, tool: usize, extrude_perimeter: bool, _first_toolchange_to_nonsoluble: bool) -> ToolChangeResult {
        let old_tool = self.current_tool;

        let mut wipe_depth = 0.0_f32;
        let mut wipe_length = 0.0_f32;
        let mut purge_volume = 0.0_f32;
        let mut nozzle_change_depth = 0.0_f32;
        // Finds this toolchange info
        if tool != FA_NO_TOOL {
            for b in &self.plan[self.layer_idx].tool_changes {
                if b.new_tool == tool {
                    wipe_length = b.wipe_length;
                    wipe_depth = b.required_depth;
                    purge_volume = b.purge_volume;
                    nozzle_change_depth = b.nozzle_change_depth;
                    break;
                }
            }
        }

        let cleaning_box = BoxCoordinates::from_pos(
            Vec2f::new(self.perimeter_width, self.perimeter_width),
            self.wipe_tower_width - 2.0 * self.perimeter_width,
            if tool != FA_NO_TOOL {
                wipe_depth + self.depth_traversed - self.perimeter_width
            } else {
                self.depth - self.perimeter_width
            },
        );

        let mut writer = WipeTowerWriter::new_cpp(
            self.layer_height,
            self.perimeter_width,
            self.gcode_flavor,
            &self.filament_params,
            self.enable_arc_fitting,
        );
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        writer.set_y_shift(self.fa_writer_y_shift(tool != FA_NO_TOOL));
        writer.append(";--------------------\n; CP TOOLCHANGE START\n");
        writer.comment_with_value(" toolchange #", (self.num_tool_changes + 1) as i32);

        self.set_for_wipe_tower_writer(&mut writer);

        if tool != FA_NO_TOOL {
            let from = if self.current_tool < self.filament_params.len() {
                self.filament_params[self.current_tool].material.clone()
            } else {
                "(NONE)".to_string()
            };
            writer.append(&format!("; material : {} -> {}\n", from, self.filament_params[tool].material));
            writer.append(";--------------------\n");
        }

        writer.speed_override_backup();
        writer.speed_override(100);

        let feedrate = if self.is_first_layer() {
            fa_min(self.first_layer_speed * 60.0, 5400.0)
        } else {
            self.calc_tower_feedrate(self.current_tool, self.extrusion_flow, self.is_first_layer())
        };

        if tool != FA_NO_TOOL {
            writer.append(";");
            writer.append(" WIPE_TOWER_START");
            writer.append("\n");
            let current_material = self.filament_params[self.current_tool].material.clone();
            let new_temperature = if self.is_first_layer() {
                self.filament_params[tool].nozzle_temperature_initial_layer
            } else {
                self.filament_params[tool].nozzle_temperature
            };
            self.toolchange_unload(&mut writer, &cleaning_box, &current_material, new_temperature);
            let new_material = self.filament_params[tool].material.clone();
            self.toolchange_change(&mut writer, tool, &new_material);
            self.toolchange_load(&mut writer, &cleaning_box);

            if self.is_multi_extruder && self.is_tpu_filament(tool) {
                let dy = 2.0 * self.perimeter_width;
                let mut nozzle_change_speed = 60.0 * self.filament_params[tool].max_e_speed / self.extrusion_flow;
                nozzle_change_speed = (nozzle_change_speed as f64 * 0.25) as f32;

                let xl = cleaning_box.ld.x;
                let xr = cleaning_box.rd.x;

                let start_pos = self.nozzle_change_result.start_pos + Vec2f::new(0.0, self.perimeter_width);
                let mut left_to_right = true;
                let tpu_travel_length = 5.0_f64;
                let e_flow = self.extrusion_flow(self.layer_height) as f64;
                let length = tpu_travel_length / e_flow;
                let tpu_line_count = (length / (self.wipe_tower_width - 2.0 * self.perimeter_width) as f64 + 1.0) as i32;

                writer.travel_xy_f(start_pos.x, start_pos.y, 0.0);

                let mut i = 0;
                loop {
                    if left_to_right {
                        let y = writer.y();
                        writer.travel_xy_f(xr - self.perimeter_width, y, nozzle_change_speed);
                    } else {
                        let y = writer.y();
                        writer.travel_xy_f(xl + self.perimeter_width, y, nozzle_change_speed);
                    }
                    if i == tpu_line_count - 1 {
                        break;
                    }
                    let (x, y) = (writer.x(), writer.y());
                    writer.travel_xy_f(x, y + dy, 0.0);
                    left_to_right = !left_to_right;
                    i += 1;
                }
            }

            let initial_position = self.get_next_pos(&cleaning_box, wipe_length, false);
            writer.set_initial_position_wd(initial_position, self.wipe_tower_width, self.depth, self.internal_rotation);

            if extrude_perimeter {
                let layer = &self.plan[self.layer_idx];
                let mut wt_box = BoxCoordinates::from_pos(
                    Vec2f::new(
                        0.0,
                        if self.current_shape == WipeShape::Reversed { layer.toolchanges_depth() - layer.depth } else { 0.0 },
                    ),
                    self.wipe_tower_width,
                    layer.depth + self.perimeter_width,
                );
                // align the perimeter
                let mut pos = initial_position;
                match self.cur_layer_id % 4 {
                    0 => pos = wt_box.ld,
                    1 => pos = wt_box.rd,
                    2 => pos = wt_box.ru,
                    3 => pos = wt_box.lu,
                    _ => {}
                }
                writer.set_initial_position_wd(pos, self.wipe_tower_width, self.depth, self.internal_rotation);

                wt_box = self.align_perimeter(&wt_box);
                writer.rectangle_f(&wt_box, feedrate);
            }

            writer.travel_xy_f(initial_position.x, initial_position.y, 0.0);

            // Wipe the newly loaded filament until the end of the assigned wipe area.
            self.toolchange_wipe(&mut writer, &cleaning_box, wipe_length);

            writer.append(";");
            writer.append(" WIPE_TOWER_END");
            writer.append("\n");
            self.num_tool_changes += 1;
        } else {
            let current_material = self.filament_params[self.current_tool].material.clone();
            let t = self.filament_params[self.current_tool].nozzle_temperature;
            self.toolchange_unload(&mut writer, &cleaning_box, &current_material, t);
        }

        self.depth_traversed += wipe_depth - nozzle_change_depth;

        writer.speed_override_restore();
        writer.feedrate(self.travel_speed * 60.0);
        writer.flush_planner_queue();
        writer.reset_extruder();
        writer.append("; CP TOOLCHANGE END\n;------------------\n\n\n");

        // Ask our writer about how much material was consumed:
        if self.current_tool < self.used_filament_length.len() {
            let used = writer.get_and_reset_used_filament_length();
            self.used_filament_length[self.current_tool] += used;
        }

        self.construct_tcr(&writer, false, old_tool, false, true, purge_volume, false, -1.0)
    }

    /// WipeTower.cpp:2424-2551 `nozzle_change(old_filament_id,
    /// new_filament_id)`. Compiled out (`#if 0`) in the pin: nothing calls it.
    #[allow(dead_code)]
    pub fn nozzle_change(&mut self, old_filament_id: i32, new_filament_id: i32) -> NozzleChangeResult {
        let mut wipe_depth = 0.0_f32;
        let mut _wipe_length = 0.0_f32;
        let mut _purge_volume = 0.0_f32;
        let mut nozzle_change_line_count: i32 = 0;

        // Finds this toolchange info
        if new_filament_id as u32 != u32::MAX {
            for b in &self.plan[self.layer_idx].tool_changes {
                if b.new_tool == new_filament_id as usize {
                    _wipe_length = b.wipe_length;
                    wipe_depth = b.required_depth;
                    _purge_volume = b.purge_volume;
                    if self.has_tpu_filament() {
                        nozzle_change_line_count =
                            (((b.nozzle_change_depth + WIPE_TOWER_WT_EPSILON) / self.nozzle_change_perimeter_width) / 2.0) as i32;
                    } else {
                        nozzle_change_line_count =
                            ((b.nozzle_change_depth + WIPE_TOWER_WT_EPSILON) / self.nozzle_change_perimeter_width) as i32;
                    }
                    break;
                }
            }
        }

        let mut nozzle_change_speed = 60.0 * self.filament_params[self.current_tool].max_e_speed / self.extrusion_flow;
        if self.is_tpu_filament(self.current_tool) {
            nozzle_change_speed = (nozzle_change_speed as f64 * 0.25) as f32;
        }

        let mut writer = WipeTowerWriter::new_cpp(
            self.layer_height,
            self.perimeter_width,
            self.gcode_flavor,
            &self.filament_params,
            self.enable_arc_fitting,
        );
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_y_shift(self.fa_writer_y_shift(new_filament_id as u32 != u32::MAX));
        writer.append(&fa_nozzle_change_line(true, old_filament_id, new_filament_id, None));

        self.set_for_wipe_tower_writer(&mut writer);

        let cleaning_box = BoxCoordinates::from_pos(
            Vec2f::new(self.perimeter_width, self.perimeter_width),
            self.wipe_tower_width - 2.0 * self.perimeter_width,
            if new_filament_id as u32 != u32::MAX {
                wipe_depth + self.depth_traversed - self.perimeter_width
            } else {
                self.depth - self.perimeter_width
            },
        );

        let initial_position = cleaning_box.ld + Vec2f::new(0.0, self.depth_traversed);
        writer.set_initial_position_wd(initial_position, self.wipe_tower_width, self.depth, self.internal_rotation);

        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;

        let mut dy = self.plan[self.layer_idx].extra_spacing * self.perimeter_width;
        if self.has_tpu_filament() {
            dy = 2.0 * self.perimeter_width;
        }

        self.left_to_right = true;

        let mut i = 0;
        loop {
            let y = writer.y();
            if self.left_to_right {
                writer.extrude_xy_f(xr, y, nozzle_change_speed, LimitFlow::LimitPrintFlow);
            } else {
                writer.extrude_xy_f(xl, y, nozzle_change_speed, LimitFlow::LimitPrintFlow);
            }
            if writer.y() - crate::libslic3r::EPSILON as f32 > cleaning_box.lu.y {
                break; // in case next line would not fit
            }
            if i == nozzle_change_line_count - 1 {
                break;
            }
            // stepping to the next line:
            let (x, y) = (writer.x(), writer.y());
            writer.extrude_xy_f(x, y + dy, 0.0, LimitFlow::LimitPrintFlow);
            self.left_to_right = !self.left_to_right;
            i += 1;
        }

        writer.set_extrusion_flow(self.extrusion_flow); // Reset the extrusion flow.

        self.depth_traversed += nozzle_change_line_count as f32 * dy;

        let mut result = NozzleChangeResult::new_cpp();

        if self.is_tpu_filament(self.current_tool) {
            let mut left_to_right = !self.left_to_right;
            let tpu_travel_length = 5.0_f64;
            let e_flow = self.extrusion_flow(self.layer_height) as f64;
            let length = tpu_travel_length / e_flow;
            let tpu_line_count = (length / (self.wipe_tower_width - 2.0 * self.perimeter_width) as f64 + 1.0) as i32;

            let (x, y) = (writer.x(), writer.y());
            writer.travel_xy_f(x, y - self.perimeter_width, 0.0);

            let mut i = 0;
            loop {
                let y = writer.y();
                if left_to_right {
                    writer.travel_xy_f(xr - self.perimeter_width, y, nozzle_change_speed);
                } else {
                    writer.travel_xy_f(xl + self.perimeter_width, y, nozzle_change_speed);
                }
                if i == tpu_line_count - 1 {
                    break;
                }
                let (x, y) = (writer.x(), writer.y());
                writer.travel_xy_f(x, y - dy, 0.0);
                left_to_right = !left_to_right;
                i += 1;
            }
        } else {
            result.wipe_path.push(writer.pos());
            if self.left_to_right {
                result.wipe_path.push(Vec2f::new(0.0, writer.y()));
            } else {
                result.wipe_path.push(Vec2f::new(self.wipe_tower_width, writer.y()));
            }
        }

        writer.append(&fa_nozzle_change_line(false, old_filament_id, new_filament_id, None));

        result.start_pos = writer.start_pos_rotated();
        result.end_pos = writer.pos();
        result.gcode = writer.gcode().to_string();
        result
    }

    /// WipeTower.cpp:2553-2690 `toolchange_Unload`: the whole body is compiled
    /// out (`#if 0`, BBS: the unload is done in change_filament_gcode).
    fn toolchange_unload(&mut self, _writer: &mut WipeTowerWriter, _cleaning_box: &BoxCoordinates, _current_material: &str, _new_temperature: i32) {}

    /// WipeTower.cpp:2692-2726 `toolchange_Change`: the filament placeholders,
    /// the new tool in the writer, the flow of the new tool.
    fn toolchange_change(&mut self, writer: &mut WipeTowerWriter, new_tool: usize, _new_material: &str) {
        // Ask the writer about how much of the old filament we consumed:
        if self.current_tool < self.used_filament_length.len() {
            let used = writer.get_and_reset_used_filament_length();
            self.used_filament_length[self.current_tool] += used;
        }

        // This is where we want to place the custom gcodes. We will use placeholders for this.
        // These will be substituted by the actual gcodes when the gcode is generated.
        writer.append("[filament_end_gcode]\n");
        writer.append("[change_filament_gcode]\n");

        // The toolchange Tn command will be inserted later, only in case that the user does
        // not provide a custom toolchange gcode.
        writer.set_tool(new_tool); // This outputs nothing, the writer just needs to know the tool has changed.
        writer.append("[filament_start_gcode]\n");

        writer.flush_planner_queue();
        self.current_tool = new_tool;
        self.extrusion_flow = self.extrusion_flow(self.layer_height);
    }

    /// WipeTower.cpp:2728-2758 `toolchange_Load`: compiled out (`#if 0`, BBS:
    /// the load is done in change_filament_gcode).
    fn toolchange_load(&mut self, _writer: &mut WipeTowerWriter, _cleaning_box: &BoxCoordinates) {}

    /// WipeTower.cpp:2760-2877 `toolchange_Wipe(writer, cleaning_box,
    /// wipe_length)`: the legacy purge zigzag.
    fn toolchange_wipe(&mut self, writer: &mut WipeTowerWriter, cleaning_box: &BoxCoordinates, wipe_volume: f32) {
        let wipe_length = wipe_volume;
        // Increase flow on first layer, slow down print.
        let first_layer = self.is_first_layer();
        writer.set_extrusion_flow(self.extrusion_flow * if first_layer { self.first_layer_flow_ratio } else { 1.0 });
        writer.append("; CP TOOLCHANGE WIPE\n");

        // BBS: add the note for gcode-check, when the flow changed, the width should follow the change
        if first_layer {
            writer.append(&format!(";{}{}\n", " LINE_WIDTH: ", std_to_string_f32(self.first_layer_flow_ratio * self.perimeter_width)));
        }

        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;

        let mut x_to_wipe = wipe_length;
        let dy = self.plan[self.layer_idx].extra_spacing * self.perimeter_width;

        let target_speed = if first_layer { fa_min(self.first_layer_speed * 60.0, 4800.0) } else { 4800.0_f32 };
        let mut wipe_speed = 0.33_f32 * target_speed;

        self.left_to_right = (self.cur_layer_id + 3) % 4 >= 2;
        let is_from_up = self.cur_layer_id % 2 == 1;

        let mut need_change_flow = false;
        let eps = crate::libslic3r::EPSILON as f32;
        // now the wiping itself:
        let mut i = 0;
        loop {
            if i != 0 {
                if wipe_speed < 0.34_f32 * target_speed {
                    wipe_speed = 0.375_f32 * target_speed;
                } else if (wipe_speed as f64) < 0.377 * target_speed as f64 {
                    wipe_speed = 0.458_f32 * target_speed;
                } else if wipe_speed < 0.46_f32 * target_speed {
                    wipe_speed = 0.875_f32 * target_speed;
                } else {
                    wipe_speed = fa_min(target_speed, wipe_speed + 50.0);
                }
            }

            // BBS: check the bridging area and use the bridge flow
            if need_change_flow || self.need_thick_bridge_flow(writer.y(), self.layer_height) {
                writer.set_extrusion_flow(self.extrusion_flow(0.2));
                writer.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", format!("{:.6}", 0.2_f64)));
                need_change_flow = true;
            }

            let y = writer.y();
            if self.left_to_right {
                writer.extrude_xy_f(xr, y, wipe_speed, LimitFlow::LimitPrintFlow);
            } else {
                writer.extrude_xy_f(xl, y, wipe_speed, LimitFlow::LimitPrintFlow);
            }

            // BBS: recover the flow in non-bridging area
            if need_change_flow {
                writer.set_extrusion_flow(self.extrusion_flow);
                writer.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", std_to_string_f32(self.layer_height)));
            }

            if !is_from_up && (writer.y() - eps > cleaning_box.lu.y) {
                break; // in case next line would not fit
            }
            if is_from_up && (writer.y() + eps < cleaning_box.ld.y) {
                break;
            }

            x_to_wipe -= xr - xl;
            if x_to_wipe < WIPE_TOWER_WT_EPSILON {
                break;
            }
            // stepping to the next line:
            let (x, y) = (writer.x(), writer.y());
            if is_from_up {
                writer.extrude_xy_f(x, y - dy, 0.0, LimitFlow::LimitPrintFlow);
            } else {
                writer.extrude_xy_f(x, y + dy, 0.0, LimitFlow::LimitPrintFlow);
            }

            self.left_to_right = !self.left_to_right;
            i += 1;
        }

        // BBS: modify the wipe_path after toolchange
        let (x, y) = (writer.x(), writer.y());
        writer.add_wipe_point_xy(x, y);
        writer.add_wipe_point_xy(if !self.left_to_right { self.wipe_tower_width } else { 0.0 }, y);

        if self.layer_idx < self.plan.len() {
            if let Some(last) = self.plan[self.layer_idx].tool_changes.last() {
                if self.current_tool != last.new_tool {
                    self.left_to_right = !self.left_to_right;
                }
            }
        }

        writer.set_extrusion_flow(self.extrusion_flow); // Reset the extrusion flow.
        // BBS: add the note for gcode-check when the flow changed
        if first_layer {
            writer.append(&format!(";{}{}\n", " LINE_WIDTH: ", std_to_string_f32(self.perimeter_width)));
        }
    }

    /// WipeTower.cpp:3542-3725 `tool_change_new(new_tool, solid_toolchange,
    /// solid_nozzlechange)`: the nozzle change of the outgoing filament (ramming,
    /// idle travel or heater off), then the purge of the new one in its band,
    /// with the side scrape when it is on.
    fn tool_change_new(&mut self, new_tool: usize, solid_toolchange: bool, solid_nozzlechange: bool) -> ToolChangeResult {
        self.nozzle_change_result = NozzleChangeResult::new_cpp();
        let layer_id = self.cur_layer_id as i32;
        let eps = crate::libslic3r::EPSILON;
        if self.is_need_ramming(self.current_tool, new_tool, layer_id) {
            let mut use_idle_travel = false;
            let mut fc_length = 0.0_f32;
            if self.handle_hotend_as_extruder {
                fc_length = if !self.is_same_extruder(self.current_tool, new_tool, layer_id) {
                    self.filaments_change_length.0[self.current_tool] as f32
                } else {
                    self.filaments_change_length.1[self.current_tool] as f32
                };
                use_idle_travel = (fc_length as f64) <= eps
                    && !self.is_first_layer()
                    && self.filament_params[self.current_tool].idle_travel_cooling_enabled;
            }
            if use_idle_travel {
                if self.is_valid_last_layer(self.current_tool as i32, layer_id, self.z_pos as f64) {
                    self.nozzle_change_result = self.idle_travel_cooling(self.current_tool as i32, new_tool as i32);
                }
            } else if self.handle_hotend_as_extruder && (fc_length as f64) <= eps && !self.is_first_layer() {
                if self.is_valid_last_layer(self.current_tool as i32, layer_id, self.z_pos as f64) {
                    self.nozzle_change_result = self.heater_off_nozzle_change(self.current_tool as i32, new_tool as i32);
                }
            } else {
                let hotend_change = self.is_same_extruder(self.current_tool, new_tool, layer_id);
                if self.is_valid_last_layer(self.current_tool as i32, layer_id, self.z_pos as f64) {
                    self.nozzle_change_result =
                        self.ramming(self.current_tool as i32, new_tool as i32, solid_nozzlechange, !hotend_change);
                }
            }
        }

        let old_tool = self.current_tool;
        let mut wipe_depth = 0.0_f32;
        let mut wipe_length = 0.0_f32;
        let mut purge_volume = 0.0_f32;
        let mut nozzle_change_depth = 0.0_f32;

        if new_tool != FA_NO_TOOL {
            for b in &self.plan[self.layer_idx].tool_changes {
                if b.new_tool == new_tool {
                    wipe_length = b.wipe_length;
                    wipe_depth = b.required_depth;
                    purge_volume = b.purge_volume;
                    nozzle_change_depth = b.nozzle_change_depth;
                    break;
                }
            }
        }
        self.current_tool = new_tool;
        self.extrusion_flow = self.extrusion_flow(self.layer_height);
        let block_key = self.get_block_key(new_tool as i32, layer_id);
        let Some(block_idx) = self.get_block_by_category(block_key, false) else {
            // :3586-3594 — C++ logs and throws InvalidArgument.
            panic!(
                "WipeTower: missing block for planned tool change, layer={}, planned={}->{}, block_key={}",
                self.cur_layer_id, old_tool, new_tool, block_key
            );
        };
        self.current_tool = new_tool;
        self.cur_block_idx = Some(block_idx);
        let new_tool_width = self.filament_perimeter_width(new_tool as i32);
        // The purge sits on the merged slab of this band, which may span several base layers.
        let slab_height = self.slab_height_for(new_tool as i32, layer_id, self.layer_height);
        self.extrusion_flow = self.extrusion_flow_for(slab_height, new_tool_width);
        let cleaning_box = BoxCoordinates::from_pos(
            Vec2f::new(new_tool_width, self.wipe_tower_blocks[block_idx].cur_depth),
            self.wipe_tower_width - 2.0 * new_tool_width,
            wipe_depth - nozzle_change_depth,
        );

        let mut writer =
            WipeTowerWriter::new_cpp(slab_height, new_tool_width, self.gcode_flavor, &self.filament_params, self.enable_arc_fitting);
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        writer.set_y_shift(self.fa_writer_y_shift(new_tool != FA_NO_TOOL));
        writer.append(";--------------------\n; CP TOOLCHANGE START\n");
        writer.comment_with_value(" toolchange #", (self.num_tool_changes + 1) as i32);

        self.set_for_wipe_tower_writer(&mut writer);
        let first_layer = self.is_first_layer_for_slab(slab_height);
        writer.set_first_layer(first_layer);

        if new_tool != FA_NO_TOOL {
            let from = if old_tool < self.filament_params.len() {
                self.filament_params[old_tool].material.clone()
            } else {
                "(NONE)".to_string()
            };
            writer.append(&format!("; material : {} -> {}\n", from, self.filament_params[new_tool].material));
            writer.append(";--------------------\n");
        }

        writer.speed_override_backup();
        writer.speed_override(100);

        let mut did_side_scrape = false;
        let mut scrape_start_local = Vec2f::new(0.0, 0.0);
        let mut scrape_z = SideScrapeZ::default();

        // Ram the hot material out of the melt zone, retract the filament into the cooling tubes and let it cool.
        if new_tool != FA_NO_TOOL {
            let initial_position = self.get_next_pos(&cleaning_box, wipe_length, solid_toolchange);
            writer.set_initial_position_wd(initial_position, self.wipe_tower_width, self.depth, self.internal_rotation);

            writer.append(";");
            writer.append(" WIPE_TOWER_START");
            writer.append("\n");
            let current_material = self.filament_params[self.current_tool].material.clone();
            let new_temperature = if first_layer {
                self.filament_params[new_tool].nozzle_temperature_initial_layer
            } else {
                self.filament_params[new_tool].nozzle_temperature
            };
            self.toolchange_unload(&mut writer, &cleaning_box, &current_material, new_temperature);
            let new_material = self.filament_params[new_tool].material.clone();
            self.toolchange_change(&mut writer, new_tool, &new_material);
            // toolchange_Change() resets the flow to the tower wide default; the purge keeps the new
            // filament's own line width and the slab height of its band (:3639-3642).
            self.extrusion_flow = self.extrusion_flow_for(slab_height, new_tool_width);
            self.toolchange_load(&mut writer, &cleaning_box);
            // :3644-3676 compiled out (# if 0).

            // Diagonal side scrape: after the tool change, before the wipe line, scrape the oozed
            // filament stub onto the tower side wall (travel only). Skipped on the first layer.
            if self.enable_side_scrape
                && !first_layer
                && self.toolchange_side_scrape(
                    &mut writer,
                    &cleaning_box,
                    initial_position,
                    Some(&mut scrape_start_local),
                    Some(&mut scrape_z),
                )
            {
                // The E deretract is emitted here (after the scrape) instead of in filament_start_gcode.
                writer.append("[wipe_tower_deretract_gcode]\n");
                did_side_scrape = true;
            }

            self.toolchange_wipe_new(&mut writer, &cleaning_box, wipe_length, solid_toolchange);

            writer.append(";");
            writer.append(" WIPE_TOWER_END");
            writer.append("\n");
            self.num_tool_changes += 1;
        } else {
            let current_material = self.filament_params[self.current_tool].material.clone();
            let t = self.filament_params[self.current_tool].nozzle_temperature;
            self.toolchange_unload(&mut writer, &cleaning_box, &current_material, t);
        }

        {
            let block = &mut self.wipe_tower_blocks[block_idx];
            block.cur_depth += wipe_depth - nozzle_change_depth;
            block.last_filament_change_id = new_tool as i32;
        }

        writer.speed_override_restore();
        writer.feedrate(self.travel_speed * 60.0);
        writer.flush_planner_queue();
        writer.reset_extruder();
        writer.append("; CP TOOLCHANGE END\n;------------------\n\n\n");

        // Ask our writer about how much material was consumed:
        if self.current_tool < self.used_filament_length.len() {
            let used = writer.get_and_reset_used_filament_length();
            self.used_filament_length[self.current_tool] += used;
        }

        let mut result =
            self.construct_tcr(&writer, false, old_tool, false, true, purge_volume, solid_toolchange, slab_height);
        result.side_scrape = did_side_scrape;
        if did_side_scrape {
            result.side_scrape_z_low = scrape_z.z_low;
            result.side_scrape_depth = scrape_z.depth;
            result.side_scrape_min_height_above_base = scrape_z.min_height_above_base;
            // Route every travel that brings the nozzle to the tower straight to the scrape start.
            let origin = if self.absolute_positions { self.pos } else { Vec2f::zero() };
            let rotated_scrape_start = origin + writer.point_rotated(scrape_start_local);
            result.start_pos = rotated_scrape_start;
            result.tool_change_start_pos = rotated_scrape_start;
        }
        result
    }

    /// WipeTower.cpp:3727-3920 `ramming(old_filament_id, new_filament_id,
    /// solid_infill, extruder_change)`: the outgoing filament rammed into its
    /// band at the nozzle-change line width, with the nozzle change commands
    /// around it and, when configured, the reverse travel.
    fn ramming(&mut self, old_filament_id: i32, new_filament_id: i32, solid_change: bool, extruder_change: bool) -> NozzleChangeResult {
        let solid_infill = solid_change;
        let format_line_m106 = || String::from("M106 S255\n");
        let format_line_m633 = || String::from("M633\n");
        let format_line_m632 = |filament_id: i32, nozzle_id: i32| {
            let mut buffer = format!("M632 S{}", filament_id);
            if nozzle_id >= 0 {
                buffer.push_str(&format!(" H{}", nozzle_id));
            }
            buffer.push_str(" M N\n");
            buffer
        };
        let layer_id = self.cur_layer_id as i32;
        let eps = crate::libslic3r::EPSILON;

        let mut nozzle_change_line_count: i32 = 0;
        // The ramming lines are extruded by the outgoing filament, so they follow its own nozzle.
        let nc_width = self.filament_nozzle_change_width(old_filament_id);
        let old_tool_width = self.filament_perimeter_width(old_filament_id);
        let x_offset = old_tool_width + (nc_width - old_tool_width) / 2.0;
        let nozzle_change_box_width = self.wipe_tower_width - 2.0 * x_offset;
        let mut nozzle_change_depth = 0.0_f32;
        if new_filament_id as u32 != u32::MAX {
            for b in &self.plan[self.layer_idx].tool_changes {
                if b.new_tool == new_filament_id as usize {
                    nozzle_change_line_count = (b.nozzle_change_length / nozzle_change_box_width).ceil() as i32;
                    nozzle_change_depth = b.nozzle_change_depth;
                    break;
                }
            }
        }
        let nozzle_ids = |s: &Self| (s.get_nozzle_id(old_filament_id, layer_id), s.get_nozzle_id(new_filament_id, layer_id));

        // The ramming lands in the outgoing filament's band, which carries its own merged slab height.
        let ramming_slab_height = self.slab_height_for(old_filament_id, layer_id, self.layer_height);
        let nz_extrusion_flow = self.extrusion_flow_for(ramming_slab_height, nc_width);
        let mut writer =
            WipeTowerWriter::new_cpp(ramming_slab_height, nc_width, self.gcode_flavor, &self.filament_params, self.enable_arc_fitting);
        writer.set_extrusion_flow(nz_extrusion_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        writer.set_y_shift(self.fa_writer_y_shift(new_filament_id as u32 != u32::MAX));
        self.set_for_wipe_tower_writer(&mut writer);
        let first_layer = self.is_first_layer_for_slab(ramming_slab_height);
        writer.set_first_layer(first_layer);

        let block_key = self.get_block_key(old_filament_id, layer_id);
        let Some(block_idx) = self.get_block_by_category(block_key, false) else {
            // :3774-3777 — assert(false); return NozzleChangeResult();
            return NozzleChangeResult::new_cpp();
        };
        self.cur_block_idx = Some(block_idx);

        let mut dy = if first_layer {
            nc_width
        } else {
            self.plan[self.layer_idx].extra_spacing * self.get_block_gap_width(self.current_tool as i32, true)
        };
        let cleaning_box = BoxCoordinates::from_pos(
            Vec2f::new(x_offset, self.wipe_tower_blocks[block_idx].cur_depth + (nc_width - old_tool_width) / 2.0),
            nozzle_change_box_width,
            nozzle_change_depth,
        );
        let initial_position = cleaning_box.ld;
        writer.set_initial_position_wd(initial_position, self.wipe_tower_width, self.depth, self.internal_rotation);

        let dynamic_map = self.multi_nozzle_group_result.as_ref().map_or(false, |g| g.is_support_dynamic_nozzle_map());

        // --- Nozzle change preamble: notify firmware + precool ---
        writer.append(&fa_nozzle_change_line(true, old_filament_id, new_filament_id, Some(nozzle_ids(self))));
        if !extruder_change {
            let new_nozzle_id = if dynamic_map { self.get_nozzle_id(new_filament_id, layer_id) } else { -1 };
            writer.append(&format_line_m632(new_filament_id, new_nozzle_id));
            if self.filament_params[self.current_tool].precool_target_temp.1 != 0 {
                writer.format_line_m104(
                    self.filament_params[self.current_tool].precool_target_temp.1,
                    self.current_tool as i32,
                    false,
                    true,
                    "Wipe tower nozzle change pre cooling",
                );
                writer.append(&format_line_m106());
            }
            writer.append(&format_line_m633());
        }

        let mut result = NozzleChangeResult::new_cpp();

        if nozzle_change_line_count > 0 {
            let p = &self.filament_params[self.current_tool];
            let max_e_ramming_speed = if extruder_change { p.max_e_ramming_speed.0 } else { p.max_e_ramming_speed.1 };
            let mut nozzle_change_speed = 60.0 * max_e_ramming_speed / nz_extrusion_flow;
            if solid_infill {
                nozzle_change_speed = fa_min(40.0 * 60.0, nozzle_change_speed);
            }
            let mut bridge_speed =
                fa_min(60.0 * max_e_ramming_speed / self.extrusion_flow_for(0.2, nc_width), nozzle_change_speed);

            let xl = cleaning_box.ld.x;
            let xr = cleaning_box.rd.x;
            dy = if solid_infill { nc_width } else { dy };
            if solid_infill {
                nozzle_change_line_count = (eps
                    + ((cleaning_box.ru.y - cleaning_box.rd.y + (nc_width - old_tool_width) / 2.0) / nc_width) as f64)
                    .floor() as i32;
            }
            self.left_to_right = true;
            let mut need_change_flow = false;
            let ramming_length = nozzle_change_line_count as f32 * (xr - xl);
            let extruder_id = self.get_extruder_id(self.current_tool as i32, layer_id) as usize;
            let p = &self.filament_params[self.current_tool];
            let precool_t = if extruder_change { p.precool_t.0[extruder_id] } else { p.precool_t.1[extruder_id] };
            let precool_t_first_layer =
                if extruder_change { p.precool_t_first_layer.0[extruder_id] } else { p.precool_t_first_layer.1[extruder_id] };
            let mut per_cooling_max_speed = nozzle_change_speed;
            // PVA-only: keep volumetric ram speed (do not stretch to precool_t).
            let command_only_active = self.handle_hotend_as_extruder && p.material == "PVA";
            if extruder_change && !command_only_active {
                if first_layer && (precool_t_first_layer as f64) > eps {
                    per_cooling_max_speed = ramming_length / precool_t_first_layer * 60.0;
                } else if (precool_t as f64) > eps {
                    per_cooling_max_speed = ramming_length / precool_t * 60.0;
                }
            } // BBS:nozzle change does not require forcing a cooldown to a specific temperature.
            if nozzle_change_speed > per_cooling_max_speed {
                nozzle_change_speed = per_cooling_max_speed;
            }
            if bridge_speed > per_cooling_max_speed {
                bridge_speed = per_cooling_max_speed;
            }
            let limit_ramming = if extruder_change { LimitFlow::LimitRammingFlow } else { LimitFlow::LimitRammingFlowNC };
            let mut i = 0;
            loop {
                if self.need_thick_bridge_flow(writer.pos().y, ramming_slab_height) {
                    writer.set_extrusion_flow(self.extrusion_flow_for(0.2, nc_width));
                    writer.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", format!("{:.6}", 0.2_f64)));
                    need_change_flow = true;
                }
                let y = writer.y();
                let f = if need_change_flow { bridge_speed } else { nozzle_change_speed };
                if self.left_to_right {
                    writer.extrude_xy_f(xr, y, f, limit_ramming);
                } else {
                    writer.extrude_xy_f(xl, y, f, limit_ramming);
                }
                if i == nozzle_change_line_count - 1 {
                    break;
                }
                if ((writer.y() + dy - cleaning_box.ru.y) + (nc_width + old_tool_width) / 2.0) > eps as f32 {
                    break;
                }
                if need_change_flow {
                    writer.set_extrusion_flow(self.extrusion_flow_for(ramming_slab_height, nc_width));
                    writer.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", std_to_string_f32(ramming_slab_height)));
                    need_change_flow = false;
                }
                let (x, y) = (writer.x(), writer.y());
                writer.extrude_xy_f(x, y + dy, nozzle_change_speed, limit_ramming);
                self.left_to_right = !self.left_to_right;
                i += 1;
            }
            if need_change_flow {
                writer.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", std_to_string_f32(ramming_slab_height)));
            }

            writer.set_extrusion_flow(nz_extrusion_flow);
            {
                let block = &mut self.wipe_tower_blocks[block_idx];
                block.cur_depth += nozzle_change_depth;
                block.last_nozzle_change_id = old_filament_id;
            }
            // --- Post-ramming: re-arm nozzle change for travel phase ---
            if !extruder_change {
                let new_nozzle_id = if dynamic_map { self.get_nozzle_id(new_filament_id, layer_id) } else { -1 };
                writer.append(&format_line_m632(new_filament_id, new_nozzle_id));
            }

            if self.is_need_reverse_travel(self.current_tool as i32, extruder_change) {
                let mut left_to_right = !self.left_to_right;
                let tpu_line_count = nozzle_change_line_count;
                nozzle_change_speed *= 2.0; // due to nozzle change 2 perimeter
                let p = &self.filament_params[self.current_tool];
                let ramming_travel_time = if extruder_change { p.ramming_travel_time.0 } else { p.ramming_travel_time.1 };
                let mut need_reverse_travel_dis = ramming_travel_time * nozzle_change_speed / 60.0;
                // xr - xl - 2 * w: the doubling is exact, so the contraction changes nothing.
                let real_travel_dis = tpu_line_count as f32 * (xr - xl - 2.0 * old_tool_width);
                if real_travel_dis < need_reverse_travel_dis {
                    nozzle_change_speed *= real_travel_dis / need_reverse_travel_dis;
                }
                let (x, y) = (writer.x(), writer.y());
                writer.travel_xy_f(x, y + dy / 2.0, 0.0);

                let mut i = 0;
                loop {
                    need_reverse_travel_dis -= xr - xl - 2.0 * old_tool_width;
                    let mut offset_dis = 0.0_f32;
                    if need_reverse_travel_dis < 0.0 {
                        offset_dis = -need_reverse_travel_dis;
                    }
                    let y = writer.y();
                    if left_to_right {
                        writer.travel_xy_f(xr - old_tool_width - offset_dis, y, nozzle_change_speed);
                    } else {
                        writer.travel_xy_f(xl + old_tool_width + offset_dis, y, nozzle_change_speed);
                    }
                    if (need_reverse_travel_dis as f64) < eps {
                        break;
                    }
                    if i == tpu_line_count - 1 {
                        break;
                    }
                    let (x, y) = (writer.x(), writer.y());
                    writer.travel_xy_f(x, y - dy, 0.0);
                    left_to_right = !left_to_right;
                    i += 1;
                }
            } else {
                result.wipe_path.push(writer.pos_rotated());
                if self.left_to_right {
                    result.wipe_path.push(Vec2f::new(0.0, writer.pos_rotated().y));
                } else {
                    result.wipe_path.push(Vec2f::new(self.wipe_tower_width, writer.pos_rotated().y));
                }
            }
            if !extruder_change {
                writer.append(&format_line_m633());
            }
        }

        writer.append(&fa_nozzle_change_line(false, old_filament_id, new_filament_id, Some(nozzle_ids(self))));

        result.start_pos = writer.start_pos_rotated();
        result.origin_start_pos = initial_position;
        result.end_pos = writer.pos_rotated();
        result.gcode = writer.gcode().to_string();
        result.is_extruder_change = extruder_change;
        result
    }

    /// WipeTower.cpp:3922-3942 `heater_off_nozzle_change`: only the nozzle
    /// change markers; the heater is switched off by the G-code side.
    fn heater_off_nozzle_change(&mut self, old_filament_id: i32, new_filament_id: i32) -> NozzleChangeResult {
        let layer_id = self.cur_layer_id as i32;
        let ids = (self.get_nozzle_id(old_filament_id, layer_id), self.get_nozzle_id(new_filament_id, layer_id));
        let mut result = NozzleChangeResult::new_cpp();
        result.heater_off_only = true;
        // This path is only entered when the nozzles differ.
        result.is_extruder_change = true;
        result.precool_filament_id = old_filament_id;
        result.gcode = fa_nozzle_change_line(true, old_filament_id, new_filament_id, Some(ids))
            + &fa_nozzle_change_line(false, old_filament_id, new_filament_id, Some(ids));
        result
    }

    /// WipeTower.cpp:3944-4054 `idle_travel_cooling`: travel moves over the
    /// band of the incoming filament while the outgoing nozzle cools, timed to
    /// the cooling when there is a precool target.
    fn idle_travel_cooling(&mut self, old_filament_id: i32, new_filament_id: i32) -> NozzleChangeResult {
        let layer_id = self.cur_layer_id as i32;
        let eps = crate::libslic3r::EPSILON;
        let ids = (self.get_nozzle_id(old_filament_id, layer_id), self.get_nozzle_id(new_filament_id, layer_id));
        let format_line_m633 = || String::from("M633\n");
        let format_line_m632 = |filament_id: i32| format!("M632 S{} D\n", filament_id);

        let mut wipe_depth = 0.0_f32;
        let mut wipe_length = 0.0_f32;
        for b in &self.plan[self.layer_idx].tool_changes {
            if b.new_tool == new_filament_id as usize {
                wipe_length = b.wipe_length;
                wipe_depth = b.required_depth;
                break;
            }
        }

        let Some(block_idx) = self.get_block_by_category(self.get_block_key(new_filament_id, layer_id), false) else {
            return NozzleChangeResult::new_cpp();
        };

        let dy = self.plan[self.layer_idx].extra_spacing * self.get_block_gap_width(new_filament_id, false);
        let xl = self.filament_perimeter_width(new_filament_id);
        let xr = self.wipe_tower_width - xl;
        let line_len = xr - xl;

        let line_count_by_length = if (line_len as f64) > eps { 1.max((wipe_length / line_len).ceil() as i32) } else { 0 };
        let line_count_by_depth = if (dy as f64) > eps { (wipe_depth / dy).floor() as i32 } else { 0 };
        let line_count = line_count_by_length.min(1.max(line_count_by_depth));
        if line_count <= 0 {
            return NozzleChangeResult::new_cpp();
        }

        let extruder_change = !self.is_same_extruder(old_filament_id as usize, new_filament_id as usize, layer_id);
        let p = &self.filament_params[old_filament_id as usize];
        let precool_target = if extruder_change { p.precool_target_temp.0 } else { p.precool_target_temp.1 } as f32;
        let max_travel_speed = self.travel_speed * 60.0;
        let mut travel_speed = max_travel_speed;

        if precool_target > 0.0 {
            let extruder_id = self.get_nozzle_id(old_filament_id, layer_id);
            let mut cooling_rate = if extruder_id >= 0 && (extruder_id as usize) < self.hotend_cooling_rate.len() {
                self.hotend_cooling_rate[extruder_id as usize] as f32
            } else {
                2.0
            };
            if (cooling_rate as f64) < eps {
                cooling_rate = 2.0;
            }
            let current_temp = self.filament_params[old_filament_id as usize].nozzle_temperature as f32;
            if current_temp > precool_target {
                let cooling_time = (current_temp - precool_target) / cooling_rate;
                let total_travel_len = line_count as f32 * line_len;
                if (total_travel_len as f64) > eps && (cooling_time as f64) > eps {
                    travel_speed = fa_min(max_travel_speed, total_travel_len / cooling_time * 60.0);
                }
            }
        }

        let mut writer = WipeTowerWriter::new_cpp(
            self.layer_height,
            self.perimeter_width,
            self.gcode_flavor,
            &self.filament_params,
            self.enable_arc_fitting,
        );
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        writer.set_y_shift(self.fa_writer_y_shift(new_filament_id as u32 != u32::MAX));
        writer.append(&fa_nozzle_change_line(true, old_filament_id, new_filament_id, Some(ids)));
        self.set_for_wipe_tower_writer(&mut writer);

        // Alternate the entry corner every layer, matching the get_next_pos pattern.
        let start_at_left = (self.cur_layer_id % 4 == 0) || (self.cur_layer_id % 4 == 3);
        let start_at_bottom = (self.cur_layer_id % 4 == 0) || (self.cur_layer_id % 4 == 2);
        let depth_span = (line_count - 1) as f32 * dy;

        let cur_depth = self.wipe_tower_blocks[block_idx].cur_depth;
        let start_pos = Vec2f::new(
            if start_at_left { xl } else { xr },
            if start_at_bottom { cur_depth } else { cur_depth + depth_span },
        );
        writer.set_initial_position_wd(start_pos, self.wipe_tower_width, self.depth, self.internal_rotation);

        if precool_target > 0.0 {
            writer.format_line_m104(precool_target as i32, old_filament_id, false, true, "Wipe tower idle travel pre cooling");
        } else {
            writer.format_line_m104(0, old_filament_id, false, true, "Wipe tower idle travel heater off");
        }

        // --- Wrap idle-travel moves with nozzle change notify (always for idle travel) ---
        writer.append(&format_line_m632(new_filament_id));

        let mut moving_right = start_at_left;
        let row_step = if start_at_bottom { dy } else { -dy };
        for i in 0..line_count {
            let y = writer.y();
            writer.travel_xy_f(if moving_right { xr } else { xl }, y, travel_speed);
            if i < line_count - 1 {
                let (x, y) = (writer.x(), writer.y());
                writer.travel_xy_f(x, y + row_step, 0.0);
            }
            moving_right = !moving_right;
        }

        writer.append(&format_line_m633());

        writer.append(&fa_nozzle_change_line(false, old_filament_id, new_filament_id, Some(ids)));

        let mut result = NozzleChangeResult::new_cpp();
        result.start_pos = writer.start_pos_rotated();
        result.origin_start_pos = start_pos;
        result.end_pos = writer.pos_rotated();
        result.gcode = writer.gcode().to_string();
        result.is_extruder_change = extruder_change;
        result.zero_flush_idle_cooling = true;
        result.precool_target_temp = precool_target;
        result.precool_filament_id = old_filament_id;
        result
    }

    /// WipeTower.cpp:4687-4708 `outer_wall_extreme_at`: the outermost wall
    /// coordinate at `along` on the stored layer whose Z is nearest `z`
    /// (within half a layer height; the later of two equally near ones).
    fn outer_wall_extreme_at(&self, z: f32, across_is_x: bool, positive_side: bool, along: f32, out_across: &mut f32) -> bool {
        if self.outer_wall.is_empty() {
            return false;
        }
        let tol = self.layer_height * 0.5 + crate::libslic3r::EPSILON as f32;
        let mut pls: Option<&Vec<Polyline>> = None;
        let mut best_dz = tol;
        // m_outer_wall.lower_bound(z - tol), then while key <= z + tol (the map
        // is ordered by Z; the keys here are the bits of positive floats).
        for (key, value) in self.outer_wall.iter() {
            let kz = f32::from_bits(*key as u32);
            if kz < z - tol {
                continue;
            }
            if !(kz <= z + tol) {
                break;
            }
            let dz = (kz - z).abs();
            if dz <= best_dz {
                best_dz = dz;
                pls = Some(value);
            }
        }
        let Some(pls) = pls else {
            return false;
        };
        polylines_extreme_at(pls, across_is_x, positive_side, along, out_across)
    }

    /// WipeTower.cpp:4710-4767 `side_wall_straight_along_range`: the straight
    /// run of a wall, grown outwards from its middle sample, pulled in by one
    /// sample step.
    fn side_wall_straight_along_range(&self, across_is_x: bool, positive_side: bool, along_begin: &mut f32, along_end: &mut f32) -> bool {
        let along_extent = if across_is_x { self.depth } else { self.wipe_tower_width };
        if along_extent <= crate::libslic3r::EPSILON as f32 {
            return false;
        }

        // std::lround: half away from zero.
        let n = 32.max((along_extent as f64 / 0.25).round() as i32);
        let step = along_extent / n as f32;

        let mut wall = vec![0.0_f32; (n + 1) as usize];
        let mut wall_valid = vec![0_i8; (n + 1) as usize];
        for i in 0..=n {
            let along = step * i as f32;
            let mut q = 0.0_f32;
            for z in [self.z_pos - self.layer_height, self.z_pos] {
                if !self.outer_wall_extreme_at(z, across_is_x, positive_side, along, &mut q) {
                    continue;
                }
                let iu = i as usize;
                wall[iu] = if wall_valid[iu] != 0 {
                    if positive_side { fa_max(wall[iu], q) } else { fa_min(wall[iu], q) }
                } else {
                    q
                };
                wall_valid[iu] = 1;
            }
        }

        let mid = n / 2;
        let mut seed = -1;
        let mut d = 0;
        while d <= n && seed < 0 {
            if mid - d >= 0 && wall_valid[(mid - d) as usize] != 0 {
                seed = mid - d;
            } else if mid + d <= n && wall_valid[(mid + d) as usize] != 0 {
                seed = mid + d;
            }
            d += 1;
        }
        if seed < 0 {
            return false;
        }
        let edge = wall[seed as usize];

        let tol = fa_max(0.02, 0.05 * self.perimeter_width);

        let mut lo = seed;
        let mut hi = seed;
        while lo > 0 && wall_valid[(lo - 1) as usize] != 0 && (wall[(lo - 1) as usize] - edge).abs() <= tol {
            lo -= 1;
        }
        while hi < n && wall_valid[(hi + 1) as usize] != 0 && (wall[(hi + 1) as usize] - edge).abs() <= tol {
            hi += 1;
        }

        // step * lo + step / step * hi - step: one contracted multiply-add each.
        *along_begin = fa_min(step.mul_add(lo as f32, step), step * seed as f32);
        *along_end = fa_max(step.mul_add(hi as f32, -step), step * seed as f32);
        true
    }

    /// WipeTower.cpp:4769-4811 `side_scrape_z_floor`: the top of the highest
    /// layer still carrying a brim or chamfer loop, plus a clearance.
    fn side_scrape_z_floor(&self) -> f32 {
        let clearance = 0.4_f32;

        if self.plan.is_empty() {
            return self.layer_height + clearance;
        }

        let first = 0.max(self.first_printed_layer_idx()) as usize;

        let mut flared_top = first;
        let wall_filament = self.get_wall_filament_for_all_layer();
        let bed_z = self.tower_bed_z() as f32;
        for i in first..self.plan.len() {
            if self.layer_is_skipped(i as i32) {
                continue;
            }
            // A layer whose wall is merged upwards draws no loops; the group above carries them.
            if i < self.wall_reserved.len() && self.wall_reserved[i] != 0 {
                continue;
            }
            let mut wf = self.get_wall_filament_for_layer(&self.plan[i], i as i32, wall_filament);
            if wf < 0 {
                wf = wall_filament;
            }
            let line_width = self.filament_perimeter_width(wf);
            let wall_height = if i < self.wall_slab_height.len() && self.wall_slab_height[i] > 0.0 {
                self.wall_slab_height[i]
            } else {
                self.layer_slab_height(i as i32, self.plan[i].height)
            };
            let anchor = i == first || self.plan[i].z - wall_height <= bed_z + WIPE_TOWER_WT_EPSILON;
            if self.brim_chamfer_loops_num(wall_height, anchor, (i - first) as i32, line_width) <= 0 {
                break;
            }
            flared_top = i;
        }

        self.plan[flared_top].z + clearance
    }

    /// WipeTower.cpp:4813-4997 `toolchange_side_scrape`: after the tool change,
    /// a travel-only diagonal lift along the outside of one tower wall that
    /// drags the ooze stub onto it, then a short move along the wall and back to
    /// the wipe start. False (nothing emitted) where the layer leaves no room.
    fn toolchange_side_scrape(
        &mut self,
        writer: &mut WipeTowerWriter,
        _cleaning_box: &BoxCoordinates,
        initial_position: Vec2f,
        out_scrape_start: Option<&mut Vec2f>,
        out_z: Option<&mut SideScrapeZ>,
    ) -> bool {
        let eps = crate::libslic3r::EPSILON as f32;
        let across_is_x = self.side_scrape_on_left_right;
        let across_extent = if across_is_x { self.wipe_tower_width } else { self.depth };

        // Advance the sequence up front so that a blocked layer hands the next attempt over.
        let slot = self.side_scrape_seq;
        self.side_scrape_seq = self.side_scrape_seq.wrapping_add(1);

        // Clamp the diagonal run as a sanity bound.
        let length = fa_min(self.side_scrape_length, across_extent / 2.0 - self.perimeter_width);
        if length <= eps || self.side_scrape_height <= eps {
            return false;
        }

        // positive@slot0, negative@slot0, positive@slot1, ...
        let positive_side = slot % 2 == 0;
        let outward = if positive_side { 1.0_f32 } else { -1.0 };

        let y_shift = writer.y_shift();
        let to_writer = |across: f32, along: f32| {
            let mut p = if across_is_x { Vec2f::new(across, along) } else { Vec2f::new(along, across) };
            p.y -= y_shift;
            p
        };

        let mut straight_lo = 0.0_f32;
        let mut straight_hi = 0.0_f32;
        if !self.side_wall_straight_along_range(across_is_x, positive_side, &mut straight_lo, &mut straight_hi) {
            return false;
        }
        let margin = fa_min(self.perimeter_width, 0.25 * (straight_hi - straight_lo));
        let along_lo = straight_lo + margin;
        let along_hi = fa_max(along_lo, straight_hi - margin);
        let span = along_hi - along_lo;
        let spacing = fa_max(0.1, self.side_scrape_point_spacing);
        let points_per_side = ((span as f64 / spacing as f64).round() as i32).clamp(1, 64);
        let along_slot = ((slot / 2) % points_per_side as u32) as i32;

        let along0 = along_lo + (along_slot as f32 + 0.5) * span / points_per_side as f32;
        let scrape_speed = self.side_scrape_speed;

        let z_top = self.z_pos; // current layer top
        // Top of the tower's own first layer, i.e. the tower base the floor below is expressed relative to.
        let mut z_base = self.layer_height;
        match self.first_layer_idx {
            Some(f) if f < self.plan.len() => z_base = self.plan[f].z,
            _ => {
                if !self.plan.is_empty() {
                    z_base = self.plan[0].z;
                }
            }
        }
        let z_floor = self.side_scrape_z_floor();
        let z_low = fa_max(self.z_pos - self.side_scrape_height, z_floor);
        // Close to the tower base there is no vertical room left.
        if z_top - z_low <= eps {
            return false;
        }

        let initial_along = if across_is_x { initial_position.y + y_shift } else { initial_position.x };
        let smear = fa_min(self.side_scrape_along_length, span);
        let mut smear_dir = if initial_along >= along0 { 1.0_f32 } else { -1.0 };
        // along0 + smear_dir * smear: smear_dir is +-1, the contraction changes nothing.
        let mut along1 = fa_clamp(smear_dir.mul_add(smear, along0), along_lo, along_hi);
        // If clamped to the same position (span boundary), try the opposite direction once.
        if (along1 - along0).abs() <= eps {
            smear_dir = -smear_dir;
            along1 = fa_clamp(smear_dir.mul_add(smear, along0), along_lo, along_hi);
        }

        // Reference wall coordinate for the whole scrape, taken from the stored wall only.
        let mut wall_found = false;
        let mut wall_across = 0.0_f32;
        {
            let a = fa_min(along0, along1);
            let b = fa_max(along0, along1);
            let ns = 2.max(((b - a) as f64 / 0.25).round() as i32);
            for i in 0..=ns {
                let at = a + (b - a) * i as f32 / ns as f32;
                for z in [self.z_pos - self.layer_height, self.z_pos] {
                    let mut q = 0.0_f32;
                    if !self.outer_wall_extreme_at(z, across_is_x, positive_side, at, &mut q) {
                        continue;
                    }
                    wall_across = if wall_found {
                        if positive_side { fa_max(wall_across, q) } else { fa_min(wall_across, q) }
                    } else {
                        q
                    };
                    wall_found = true;
                }
            }
        }
        // Without stored wall data there is nothing to place the scrape against.
        if !wall_found {
            return false;
        }

        // outward is +-1: the contractions change nothing.
        let scrape_across = outward.mul_add(self.side_scrape_wall_gap, wall_across);
        let start_across = outward.mul_add(length, scrape_across);

        if let Some(out) = out_scrape_start {
            *out = to_writer(start_across, along0);
        }
        if let Some(out) = out_z {
            out.z_low = z_low;
            out.depth = z_top - z_low;
            out.min_height_above_base = fa_max(0.0, z_floor - z_base);
        }

        writer.append(";--------------------\n; CP TOOLCHANGE SIDE SCRAPE\n");
        // 1) Position outside the wall at the layer top (also the after-toolchange landing point).
        let p = to_writer(start_across, along0);
        writer.travel_xy_f(p.x, p.y, scrape_speed);
        // 2) Descend below the already-printed top surface, staying outside the wall.
        writer.travel_xyz(to_writer(start_across, along0), "[wipe_tower_scrape_z_low]", scrape_speed);
        // 3) Diagonal scrape: rise back to the layer top while approaching the wall.
        writer.travel_xyz(to_writer(scrape_across, along0), "[wipe_tower_scrape_z_top]", scrape_speed);
        // 4) Short empty travel along the wall, following the wall contour a fixed gap outside it.
        if (along1 - along0).abs() > eps {
            let hug_across = |s: &Self, at: f32| -> f32 {
                let mut found = false;
                let mut q = if positive_side { -f32::MAX } else { f32::MAX };
                for z in [s.z_pos - s.layer_height, s.z_pos] {
                    let mut w = 0.0_f32;
                    if !s.outer_wall_extreme_at(z, across_is_x, positive_side, at, &mut w) {
                        continue;
                    }
                    q = if positive_side { fa_max(q, w) } else { fa_min(q, w) };
                    found = true;
                }
                // Where this layer stored no wall data, keep the straight reference rather than risk cutting in.
                if !found {
                    return scrape_across;
                }
                outward.mul_add(s.side_scrape_wall_gap, q)
            };
            let ns = 1.max(((along1 - along0).abs() as f64 / 0.5).round() as i32);
            for i in 1..=ns {
                let at = along0 + (along1 - along0) * i as f32 / ns as f32;
                let p = to_writer(hug_across(self, at), at);
                writer.travel_xy_f(p.x, p.y, scrape_speed);
            }
        }
        // 5) Travel to the wipe start position so the wipe line begins as usual.
        writer.travel_xy_f(initial_position.x, initial_position.y, scrape_speed);
        writer.append(";--------------------\n");

        true
    }

    /// WipeTower.cpp:4999-5220 `toolchange_wipe_new(writer, cleaning_box,
    /// wipe_length, solid_tool_toolchange)`: the purge zigzag of the new
    /// filament on its band's slab, the opening ironing of the gap wall, the
    /// speed ramp of the speed-factor curve.
    fn toolchange_wipe_new(&mut self, writer: &mut WipeTowerWriter, cleaning_box: &BoxCoordinates, wipe_length: f32, solid_toolchange: bool) {
        let solid_tool_toolchange = solid_toolchange;
        let eps = crate::libslic3r::EPSILON;
        let tool = self.current_tool;
        let tool_width = self.filament_perimeter_width(tool as i32);
        let wipe_slab_height = self.slab_height_for(tool as i32, self.cur_layer_id as i32, self.layer_height);
        let first_layer = self.is_first_layer_for_slab(wipe_slab_height);
        writer.set_first_layer(first_layer);
        writer.set_extrusion_flow(self.extrusion_flow * if first_layer { self.first_layer_flow_ratio } else { 1.0 });
        writer.append(&format!(
            ";{} CT{} FL{}\n",
            " CP_TOOLCHANGE_WIPE", solid_tool_toolchange as i32, first_layer as i32
        ));
        if !self.nozzle_change_result.gcode.is_empty() {
            // The nozzle change block spliced in above left its own width/height tags behind.
            writer.change_analyzer_line_width(tool_width);
            writer.change_analyzer_layer_height(wipe_slab_height);
        }

        // BBS: add the note for gcode-check, when the flow changed, the width should follow the change
        if first_layer {
            writer.append(&format!(";{}{}\n", " LINE_WIDTH: ", std_to_string_f32(self.first_layer_flow_ratio * tool_width)));
        }

        let retract_length = self.filament_params[tool].retract_length;
        let retract_speed = self.filament_params[tool].retract_speed * 60.0;
        let xl = cleaning_box.ld.x;
        let xr = cleaning_box.rd.x;
        let mut should_flat_ironging = self.flat_ironing;
        let mut should_line_ironing = self.line_ironing;
        if !self.contact_ironing && solid_tool_toolchange {
            should_flat_ironging = false;
            should_line_ironing = false;
        }
        let _should_cooling_before_tower = !solid_tool_toolchange;
        // interface layer heating print tower, then cooling print object
        let should_cooling_before_object = false;
        let mut cooling_begin_line = 2;
        let mut x_to_wipe = wipe_length;
        let dy = self.wipe_line_spacing(solid_tool_toolchange);
        x_to_wipe = if solid_tool_toolchange { f32::MAX } else { x_to_wipe };
        let mut target_speed = if first_layer {
            fa_min(self.first_layer_speed * 60.0, self.max_speed)
        } else {
            self.calc_tower_feedrate(tool, self.extrusion_flow, false)
        };
        if solid_tool_toolchange {
            target_speed = self.contact_speed;
        }
        let mut wipe_speed_map: Vec<f32> = Vec::new();
        let filament_curve = &self.filament_params[tool].speed_factor_curve;
        if !filament_curve.is_empty() {
            for &v in filament_curve {
                wipe_speed_map.push((v * target_speed as f64) as f32);
            }
            const K_MIN_SPEED_MAP_SIZE: usize = 5;
            if wipe_speed_map.len() < K_MIN_SPEED_MAP_SIZE {
                let back = *wipe_speed_map.last().unwrap();
                wipe_speed_map.resize(K_MIN_SPEED_MAP_SIZE, back);
            }
        } else {
            wipe_speed_map = vec![
                0.33 * target_speed,
                0.375 * target_speed,
                0.458 * target_speed,
                0.875 * target_speed,
                // 0.875f * target_speed + 50.f: one contracted multiply-add.
                fa_min(target_speed, 0.875_f32.mul_add(target_speed, 50.0)),
            ];
        }
        let mut wipe_speed = wipe_speed_map[0];

        self.left_to_right = (self.cur_layer_id + 3) % 4 >= 2;

        let is_from_up = self.cur_layer_id % 2 == 1;

        let estimate_time_kernel = |n: i32| -> f32 {
            let one_line_len = xr - xl;
            let m = &wipe_speed_map;
            let time = if n <= 1 {
                one_line_len / m[0]
            } else if n <= 2 {
                one_line_len / m[0] + one_line_len / m[1]
            } else if n <= 3 {
                one_line_len / m[0] + one_line_len / m[1] + one_line_len / m[2]
            } else if n <= 4 {
                one_line_len / m[0] + one_line_len / m[1] + one_line_len / m[2] + one_line_len / m[3]
            } else {
                let mut t = one_line_len / m[0] + one_line_len / m[1] + one_line_len / m[2] + one_line_len / m[3];
                t += (n - 4) as f32 * one_line_len / m[4];
                t
            };
            time * 60.0
        };
        let estimate_wipe_time = |begin_line: i32| -> f32 {
            let mut n = (x_to_wipe / (xr - xl)).ceil() as i32;
            if solid_tool_toolchange {
                n = ((cleaning_box.lu.y - cleaning_box.ld.y) / dy) as i32;
            }
            let total_time = estimate_time_kernel(n);
            let beg_time = if n <= 0 { 0.0 } else { estimate_time_kernel(begin_line) };
            total_time - beg_time
        };

        let should_heating = (self.filament_params[tool].filament_cooling_before_tower as f64) > eps
            && !solid_tool_toolchange
            && !first_layer;
        let add_m104_by_requirement = |s: &Self, writer: &mut WipeTowerWriter| {
            if (s.filament_params[s.current_tool].filament_cooling_before_tower as f64) < eps {
                return;
            }
            if !should_heating {
                return;
            }
            let p = &s.filament_params[s.current_tool];
            let target_temp = if first_layer { p.nozzle_temperature_initial_layer } else { p.nozzle_temperature } as f32;
            writer.format_line_m104(target_temp as i32, s.current_tool as i32, true, true, "Wipe tower reheat before wipe");
        };
        let mut speed_factor = 1.0_f32;
        // should_heating: no additional heating time is required (:5099-5108).
        if should_cooling_before_object {
            let n = ((cleaning_box.lu.y - cleaning_box.ld.y) / dy) as i32;
            let extruder_id = self.get_extruder_id(tool as i32, self.cur_layer_id as i32);
            let p = &self.filament_params[tool];
            let cooling_time = (p.filament_tower_interface_print_temp - p.nozzle_temperature as f32)
                / self.hotend_cooling_rate[extruder_id as usize] as f32;
            if n < 2 {
                let estimate_time = estimate_wipe_time(0);
                speed_factor = if estimate_time > cooling_time { 1.0 } else { estimate_time / cooling_time };
                cooling_begin_line = 0;
            } else {
                let estimate_time = estimate_wipe_time(2);
                speed_factor = if estimate_time > cooling_time { 1.0 } else { estimate_time / cooling_time };
                cooling_begin_line = 2;
            }
            wipe_speed *= speed_factor;
        }

        let mut i: i32 = 0;
        loop {
            if (i as usize) < wipe_speed_map.len() {
                wipe_speed = wipe_speed_map[i as usize] * speed_factor;
            }

            let need_change_flow = self.need_thick_bridge_flow(writer.y(), wipe_slab_height);
            // BBS: check the bridging area and use the bridge flow
            if need_change_flow {
                writer.set_extrusion_flow(self.extrusion_flow_for(0.2, tool_width));
                writer.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", format!("{:.6}", 0.2_f64)));
            }
            let flat_iron_area = self.filament_params[tool].flat_iron_area;
            let mut ironing_length = 3.0_f32;

            if should_cooling_before_object && i == cooling_begin_line {
                let p = &self.filament_params[tool];
                writer.format_line_m104(
                    p.nozzle_temperature,
                    tool as i32,
                    p.nozzle_temperature as f32 > p.filament_tower_interface_print_temp,
                    true,
                    "Wipe tower pre object cooling",
                );
            }

            if i == 0 && self.use_gap_wall {
                // BBS: add ironing after extruding start
                if self.left_to_right {
                    if should_line_ironing {
                        // xr + 0.0 * width - x, in double.
                        let dx = (xr as f64 - writer.pos().x as f64) as f32;
                        if dx.abs() < ironing_length {
                            ironing_length = dx.abs();
                        }
                        let (x, y) = (writer.x(), writer.y());
                        writer.extrude_xy_f(x + ironing_length, y, wipe_speed, LimitFlow::LimitPrintFlow);
                        writer.retract(retract_length, retract_speed);
                        let (x, y) = (writer.x(), writer.y());
                        writer.travel_xy_f((x as f64 - 1.5 * ironing_length as f64) as f32, y, 600.0);
                        if should_flat_ironging {
                            let (x, y) = (writer.x(), writer.y());
                            writer.travel_xy_f(x + 0.5 * ironing_length, y, 240.0);
                            let pos = Vec2f::new(writer.x() + 1.0 * ironing_length, writer.y());
                            let wp = writer.pos();
                            writer.spiral_flat_ironing(wp, flat_iron_area, tool_width, self.ironing_speed);
                            writer.travel_xy_f(pos.x, pos.y, wipe_speed);
                        } else {
                            let (x, y) = (writer.x(), writer.y());
                            writer.travel_xy_f((x as f64 + 1.5 * ironing_length as f64) as f32, y, 240.0);
                        }
                        writer.retract(-retract_length, retract_speed);
                    }
                    add_m104_by_requirement(self, writer);
                    let y = writer.y();
                    writer.extrude_xy_f(xr, y, wipe_speed, LimitFlow::LimitPrintFlow);
                } else {
                    if should_line_ironing {
                        let dx = (xl as f64 - writer.pos().x as f64) as f32;
                        if dx.abs() < ironing_length {
                            ironing_length = dx.abs();
                        }
                        let (x, y) = (writer.x(), writer.y());
                        writer.extrude_xy_f(x - ironing_length, y, wipe_speed, LimitFlow::LimitPrintFlow);
                        writer.retract(retract_length, retract_speed);
                        let (x, y) = (writer.x(), writer.y());
                        writer.travel_xy_f((x as f64 + 1.5 * ironing_length as f64) as f32, y, 600.0);
                        if should_flat_ironging {
                            let (x, y) = (writer.x(), writer.y());
                            writer.travel_xy_f(x - 0.5 * ironing_length, y, 240.0);
                            let pos = Vec2f::new(writer.x() - 1.0 * ironing_length, writer.y());
                            let wp = writer.pos();
                            writer.spiral_flat_ironing(wp, flat_iron_area, tool_width, self.ironing_speed);
                            writer.travel_xy_f(pos.x, pos.y, wipe_speed);
                        } else {
                            let (x, y) = (writer.x(), writer.y());
                            writer.travel_xy_f((x as f64 - 1.5 * ironing_length as f64) as f32, y, 240.0);
                        }
                        writer.retract(-retract_length, retract_speed);
                    }
                    add_m104_by_requirement(self, writer);
                    let y = writer.y();
                    writer.extrude_xy_f(xl, y, wipe_speed, LimitFlow::LimitPrintFlow);
                }
            } else {
                if i == 0 {
                    add_m104_by_requirement(self, writer);
                }
                let y = writer.y();
                if self.left_to_right {
                    writer.extrude_xy_f(xr, y, wipe_speed, LimitFlow::LimitPrintFlow);
                } else {
                    writer.extrude_xy_f(xl, y, wipe_speed, LimitFlow::LimitPrintFlow);
                }
            }

            // BBS: recover the flow in non-bridging area
            if need_change_flow {
                writer.set_extrusion_flow(self.extrusion_flow);
                writer.append(&format!(";{}{}\n", " LAYER_HEIGHT: ", std_to_string_f32(wipe_slab_height)));
            }

            if !is_from_up && (writer.y() + dy - eps as f32 > cleaning_box.lu.y - tool_width) {
                break; // in case next line would not fit
            }
            // The top of the clean box cannot have wiring, but the bottom can.
            if is_from_up && (writer.y() - dy + eps as f32) < cleaning_box.ld.y {
                break;
            }

            x_to_wipe -= xr - xl;
            if x_to_wipe < WIPE_TOWER_WT_EPSILON {
                break;
            }
            // stepping to the next line:
            let (x, y) = (writer.x(), writer.y());
            if is_from_up {
                writer.extrude_xy_f(x, y - dy, 0.0, LimitFlow::LimitPrintFlow);
            } else {
                writer.extrude_xy_f(x, y + dy, 0.0, LimitFlow::LimitPrintFlow);
            }

            self.left_to_right = !self.left_to_right;
            i += 1;
        }

        let (x, y) = (writer.x(), writer.y());
        writer.add_wipe_point_xy(x, y);
        writer.add_wipe_point_xy(if !self.left_to_right { self.wipe_tower_width } else { 0.0 }, y);

        if self.layer_idx < self.plan.len() {
            if let Some(last) = self.plan[self.layer_idx].tool_changes.last() {
                if self.current_tool != last.new_tool {
                    self.left_to_right = !self.left_to_right;
                }
            }
        }

        writer.set_extrusion_flow(self.extrusion_flow); // Reset the extrusion flow.
        // BBS: add the note for gcode-check when the flow changed
        if first_layer {
            writer.append(&format!(";{}{}\n", " LINE_WIDTH: ", std_to_string_f32(tool_width)));
        }
    }
}

// ============================================================================
// The C++ generator: layer finishes and walls (WipeTower.cpp:2916-3084, 4077-4659, 8215-8475)
// ============================================================================
// ----------------------------------------------------------------------------
// The ClipperUtils calls of the layer finishes, on the faithful ClipperUtils
// internals of `crate::clipper_utils` (raw_offset, offset_paths, clipper_do,
// _clipper_ex, expolygons_offset_pt, clipper_do_polytree).
// ----------------------------------------------------------------------------

/// `ClipperUtils::SinglePathProvider(points)` / `PolygonsProvider`: one path per
/// polygon, the coordinates as `cInt`.
fn clipper_path_of(points: &[Point]) -> crate::clipper::Path {
    points.iter().map(|p| crate::clipper::IntPoint::new(p.x as crate::clipper::cInt, p.y as crate::clipper::cInt)).collect()
}

/// `PolygonsProvider(polygons)`.
fn clipper_paths_of(polygons: &[Polygon]) -> crate::clipper::Paths {
    polygons.iter().map(|p| clipper_path_of(p.points())).collect()
}

/// `ExPolygonsProvider(expolygons)`: the contour, then the holes, of each.
fn clipper_paths_of_ex(expolygons: &[crate::geometry::ExPolygon]) -> crate::clipper::Paths {
    let mut out = crate::clipper::Paths::new();
    for ex in expolygons {
        out.push(clipper_path_of(ex.contour.points()));
        for h in &ex.holes {
            out.push(clipper_path_of(h.points()));
        }
    }
    out
}

/// `to_polygons(ClipperLib::Paths)` (ClipperUtils.cpp): one polygon per path.
fn clipper_polygons_of(paths: &crate::clipper::Paths) -> Vec<Polygon> {
    paths
        .iter()
        .map(|p| Polygon::from_points(p.iter().map(|q| Point::new(q.x as crate::Coord, q.y as crate::Coord)).collect()))
        .collect()
}

/// ClipperUtils.cpp:410-411 `offset(const Polygon &, const float delta)` with the
/// default jtMiter / 3: a raw offset of the one path, no union.
fn clipper_offset_polygon(polygon: &Polygon, delta: f32) -> Vec<Polygon> {
    let paths = vec![clipper_path_of(polygon.points())];
    clipper_polygons_of(&crate::clipper_utils::raw_offset(
        &paths,
        delta as f64,
        crate::clipper::JoinType::jtMiter,
        3.0,
        crate::clipper::EndType::etClosedPolygon,
    ))
}

/// ClipperUtils.cpp:413-414 `offset(const Polygons &, const float delta)` with the
/// default jtMiter / 3.
fn clipper_offset_polygons(polygons: &[Polygon], delta: f32) -> Vec<Polygon> {
    clipper_polygons_of(&crate::clipper_utils::offset_paths::<crate::clipper::Paths>(
        &clipper_paths_of(polygons),
        delta as f64,
        crate::clipper::JoinType::jtMiter,
        3.0,
    ))
}

/// ClipperUtils.cpp:721-722 `union_(const Polygons &)`: `_clipper(ctUnion, ...)`,
/// the flat union paths, pftNonZero.
fn clipper_union_polygons(polygons: &[Polygon]) -> Vec<Polygon> {
    clipper_polygons_of(&crate::clipper_utils::clipper_do::<crate::clipper::Paths>(
        crate::clipper::ClipType::ctUnion,
        &clipper_paths_of(polygons),
        &crate::clipper::Paths::new(),
        crate::clipper::PolyFillType::pftNonZero,
    ))
}

/// ClipperUtils.cpp:741-742 `diff_ex(const Polygons &, const Polygons &)`.
fn clipper_diff_ex_polygons(subject: &[Polygon], clip: &[Polygon]) -> Vec<crate::geometry::ExPolygon> {
    crate::clipper_utils::_clipper_ex(
        crate::clipper::ClipType::ctDifference,
        &clipper_paths_of(subject),
        &clipper_paths_of(clip),
        crate::clipper_utils::ApplySafetyOffset::No,
        crate::clipper::PolyFillType::pftNonZero,
    )
}

/// ClipperUtils.cpp:570-571 `offset_ex(const ExPolygons &, const float delta)` with
/// the default jtMiter / 3.
fn clipper_offset_ex_expolygons(expolygons: &[crate::geometry::ExPolygon], delta: f32) -> Vec<crate::geometry::ExPolygon> {
    let rings: Vec<(crate::clipper::Path, crate::clipper::Paths)> = expolygons
        .iter()
        .map(|ex| (clipper_path_of(ex.contour.points()), ex.holes.iter().map(|h| clipper_path_of(h.points())).collect()))
        .collect();
    crate::clipper_utils::poly_tree_to_ex_polygons(&crate::clipper_utils::expolygons_offset_pt(
        &rings,
        delta as f64,
        crate::clipper::JoinType::jtMiter,
        3.0,
    ))
}

/// ClipperUtils.cpp:815-816 `union_ex(const ExPolygons &)`.
fn clipper_union_ex_expolygons(expolygons: &[crate::geometry::ExPolygon]) -> Vec<crate::geometry::ExPolygon> {
    crate::clipper_utils::poly_tree_to_ex_polygons(&crate::clipper_utils::clipper_do_polytree(
        crate::clipper::ClipType::ctUnion,
        &clipper_paths_of_ex(expolygons),
        &crate::clipper::Paths::new(),
        crate::clipper::PolyFillType::pftNonZero,
    ))
}

/// `get_extents(const Points-holders &)` (BoundingBox.cpp): the box of every
/// point, (0,0)-(0,0) when there are none.
fn extents_of_points<'a>(points: impl Iterator<Item = &'a Point>) -> (Point, Point) {
    let mut min = Point::new(0, 0);
    let mut max = Point::new(0, 0);
    let mut first = true;
    for p in points {
        if first {
            min = *p;
            max = *p;
            first = false;
        } else {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
        }
    }
    (min, max)
}

/// `Point::new_scale(const Vec2f &)` (Point.hpp:187): each coordinate divided by
/// the scaling factor in double (`scale_`), then truncated.
fn point_new_scale_vec2f(v: Vec2f) -> Point {
    Point::new(
        (v.x as f64 / crate::libslic3r::SCALING_FACTOR) as crate::Coord,
        (v.y as f64 / crate::libslic3r::SCALING_FACTOR) as crate::Coord,
    )
}

/// `scaled(float)` (Point.hpp:536-541): the float divided by the float scaling
/// factor, truncated to coord_t.
fn scaled_f32(v: f32) -> crate::Coord {
    (v / crate::libslic3r::SCALING_FACTOR as f32) as crate::Coord
}

impl WipeTower {
    /// WipeTower.cpp:2916-3083 `finish_layer(extrude_perimeter, extruder_fill)`.
    /// Compiled out in the pin (`#if 0`, :2915-3084): kept as the C++ text has
    /// it, nothing calls it.
    #[allow(dead_code)]
    pub fn finish_layer(&mut self, extrude_perimeter: bool, extruder_fill: bool) -> ToolChangeResult {
        debug_assert!(!self.layer_finished());
        self.current_layer_finished = true;

        let old_tool = self.current_tool;

        let mut writer =
            WipeTowerWriter::new_cpp(self.layer_height, self.perimeter_width, self.gcode_flavor, &self.filament_params, self.enable_arc_fitting);
        writer.set_extrusion_flow(self.extrusion_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        let reversed_shift =
            if self.current_shape == WipeShape::Reversed { self.plan[self.layer_idx].toolchanges_depth() } else { 0.0 };
        writer.set_y_shift(self.y_shift - reversed_shift);

        self.set_for_wipe_tower_writer(&mut writer);

        writer.append("; WIPE_TOWER_START\n");

        // Slow down on the 1st layer.
        let first_layer = self.is_first_layer();
        // BBS: speed up perimeter speed to 90mm/s for non-first layer (std::min: (b < a) ? b : a)
        let min = |a: f32, b: f32| if b < a { b } else { a };
        let feedrate = if first_layer {
            min(self.first_layer_speed * 60.0, 5400.0)
        } else {
            min(60.0 * self.filament_params[self.current_tool].max_e_speed / self.extrusion_flow, 5400.0)
        };
        let fill_box_y = self.plan[self.layer_idx].toolchanges_depth() + self.perimeter_width;
        let fill_box = BoxCoordinates::from_pos(
            Vec2f::new(self.perimeter_width, fill_box_y),
            self.wipe_tower_width - 2.0 * self.perimeter_width,
            self.plan[self.layer_idx].depth - fill_box_y,
        );

        let p = if self.left_to_right { fill_box.ru } else { fill_box.lu };
        writer.set_initial_position_wd(p, self.wipe_tower_width, self.depth, self.internal_rotation);

        let toolchanges_on_layer = self.plan[self.layer_idx].toolchanges_depth() > WIPE_TOWER_WT_EPSILON;

        // inner perimeter of the sparse section, if there is space for it:
        if fill_box.ru.y - fill_box.rd.y > self.perimeter_width - WIPE_TOWER_WT_EPSILON {
            writer.rectangle_fill_box_ld(self, fill_box.ld, fill_box.rd.x - fill_box.ld.x, fill_box.ru.y - fill_box.rd.y, feedrate);
        }

        // Extrude infill to support the material to be printed above.
        let dy = fill_box.lu.y - fill_box.ld.y - self.perimeter_width;
        let mut left = fill_box.lu.x + 2.0 * self.perimeter_width;
        let mut right = fill_box.ru.x - 2.0 * self.perimeter_width;
        let mut finish_rect_wipe_path: Vec<Vec2f> = Vec::new();
        if extruder_fill && dy > self.perimeter_width {
            let p = fill_box.ld + Vec2f::new(self.perimeter_width * 2.0, 0.0);
            writer.travel_xy_f(p.x, p.y, 0.0);
            writer.append(";--------------------\n; CP EMPTY GRID START\n");
            writer.comment_with_value(" layer #", self.num_layer_changes.wrapping_add(1) as i32);

            // Is there a soluble filament wiped/rammed at the next layer?
            let mut solid_infill = if self.layer_idx + 1 == self.plan.len() {
                false
            } else {
                self.plan[self.layer_idx + 1]
                    .tool_changes
                    .iter()
                    .any(|tch| self.filament_params[tch.new_tool].is_soluble || self.filament_params[tch.old_tool].is_soluble)
            };
            solid_infill |= first_layer && self.adhesion;

            if solid_infill {
                let mut sparse_factor = 1.5_f32;
                if first_layer {
                    left -= self.perimeter_width;
                    right += self.perimeter_width;
                    sparse_factor = 1.0;
                }
                let mut y = fill_box.ld.y + self.perimeter_width;
                let n = (dy / (self.perimeter_width * sparse_factor)) as i32;
                let spacing = (dy - self.perimeter_width) / (n - 1) as f32;
                for i in 0..n {
                    let x = writer.x();
                    writer.extrude_xy_f(x, y, feedrate, LimitFlow::LimitPrintFlow);
                    writer.extrude_xy_f(if i % 2 != 0 { left } else { right }, y, 0.0, LimitFlow::LimitPrintFlow);
                    y = y + spacing;
                }
                let x = writer.x();
                writer.extrude_xy_f(x, fill_box.lu.y, 0.0, LimitFlow::LimitPrintFlow);
            } else {
                // Extrude an inverse U at the left of the region and the sparse infill.
                let p = fill_box.lu + Vec2f::new(self.perimeter_width * 2.0, 0.0);
                writer.extrude_xy_f(p.x, p.y, feedrate, LimitFlow::LimitPrintFlow);

                let n = 1 + ((right - left) / self.bridging) as i32;
                let dx = (right - left) / n as f32;
                for i in 1..=n {
                    let x = dx.mul_add(i as f32, left);
                    let y = writer.y();
                    writer.travel_xy_f(x, y, 0.0);
                    writer.extrude_xy_f(x, if i % 2 != 0 { fill_box.rd.y } else { fill_box.ru.y }, 0.0, LimitFlow::LimitPrintFlow);
                }
                // BBS: add wipe_path for this case: only with finish rectangle
                finish_rect_wipe_path.push(writer.pos());
                finish_rect_wipe_path.push(Vec2f::new(dx.mul_add(n as f32, left), if n % 2 != 0 { fill_box.ru.y } else { fill_box.rd.y }));
            }

            writer.append("; CP EMPTY GRID END\n;------------------\n\n\n\n\n\n\n");
        }

        // outer perimeter (always):
        let mut wipe_tower_depth = self.plan[self.layer_idx].depth + self.perimeter_width;
        if self.keeps_outer_walls_unshrunk() {
            wipe_tower_depth = self.depth;
        }
        let mut wt_box = BoxCoordinates::from_pos(Vec2f::new(0.0, reversed_shift), self.wipe_tower_width, wipe_tower_depth);
        wt_box = self.align_perimeter(&wt_box);
        if extrude_perimeter {
            writer.rectangle_f(&wt_box, feedrate);
        }

        // brim chamfer: m_perimeter_width - m_layer_height * float(1. - M_PI_4),
        // one contracted multiply-add.
        let spacing = (-self.layer_height).mul_add((1.0 - std::f64::consts::FRAC_PI_4) as f32, self.perimeter_width);
        // How many perimeters shall the brim have?
        let mut loops_num = ((self.wipe_tower_brim_width_cpp + spacing / 2.0) / spacing) as i32;
        let max_chamfer_width = 3.0_f32;
        if !first_layer {
            // stop print chamfer if depth changes
            if self.plan[self.layer_idx].depth != self.plan[0].depth {
                loops_num = 0;
            } else {
                // limit max chamfer width to 3 mm
                let chamfer_loops_num = (max_chamfer_width / spacing) as i32;
                let dist_to_1st = self.layer_idx as i32 - self.first_printed_layer_idx();
                loops_num = loops_num.min(chamfer_loops_num) - dist_to_1st;
            }
        }

        if loops_num > 0 {
            let mut bx = wt_box;
            for _ in 0..loops_num as usize {
                bx.expand(spacing);
                writer.rectangle_f(&bx, feedrate);
            }

            if first_layer {
                self.brim_width_real = wt_box.ld.x - bx.ld.x + spacing / 2.0;
            }
            wt_box = bx;
        }

        // Now prepare future wipe. box contains rectangle that was extruded last (ccw).
        let pos = writer.pos();
        let mut target = if pos == wt_box.ld {
            wt_box.rd
        } else if pos == wt_box.rd {
            wt_box.ru
        } else if pos == wt_box.ru {
            wt_box.lu
        } else {
            wt_box.ld
        };
        // BBS: add wipe_path for this case: only with finish rectangle
        if finish_rect_wipe_path.len() == 2 && finish_rect_wipe_path[0] == writer.pos() {
            target = finish_rect_wipe_path[1];
        }
        let pos = writer.pos();
        writer.add_wipe_point(pos).add_wipe_point(target);

        writer.append("; WIPE_TOWER_END\n");

        if !self.sparse_layers_skipped || toolchanges_on_layer {
            if self.current_tool < self.used_filament_length.len() {
                let used = writer.get_and_reset_used_filament_length();
                self.used_filament_length[self.current_tool] += used;
            }
        }

        self.construct_tcr(&writer, false, old_tool, true, false, 0.0, false, -1.0)
    }

    /// WipeTower.cpp:4077-4396 `finish_layer_new(extrude_perimeter, extrude_fill,
    /// extrude_fill_wall, wall_height, wall_reserved)`.
    pub fn finish_layer_new(
        &mut self,
        extrude_perimeter: bool,
        extrude_fill: bool,
        extrude_fill_wall: bool,
        wall_height: f32,
        wall_reserved: bool,
    ) -> ToolChangeResult {
        debug_assert!(!self.layer_finished());
        self.current_layer_finished = true;

        let mut wall_height = wall_height;
        if wall_height <= 0.0 {
            wall_height = self.layer_height;
        }
        let wall_width = self.filament_perimeter_width(self.current_tool as i32);
        let wall_flow = self.extrusion_flow_for(self.layer_height, wall_width);

        // :4088-4092
        let mut writer =
            WipeTowerWriter::new_cpp(self.layer_height, wall_width, self.gcode_flavor, &self.filament_params, self.enable_arc_fitting);
        writer.set_extrusion_flow(wall_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        let reversed_shift =
            if self.current_shape == WipeShape::Reversed { self.plan[self.layer_idx].toolchanges_depth() } else { 0.0 };
        writer.set_y_shift(self.y_shift - reversed_shift);

        self.set_for_wipe_tower_writer(&mut writer);

        writer.append("; WIPE_TOWER_START\n");

        // Slow down on the 1st layer.
        let first_layer = self.is_first_layer();
        let mut feedrate = self.calc_tower_feedrate(self.current_tool, wall_flow, self.is_first_layer());

        let toolchanges_on_layer = self.plan[self.layer_idx].toolchanges_depth() > WIPE_TOWER_WT_EPSILON;

        let mut finish_rect_wipe_path: Vec<Vec2f> = Vec::new();
        let multi_block_fill = self.wipe_tower_blocks.len() > 1 && (extrude_fill_wall || extrude_fill);

        // :4111-4135 the fill boxes.
        let cur_layer_id = self.cur_layer_id;
        let mut fill_boxes: Vec<BoxCoordinates> = Vec::new();
        if multi_block_fill {
            for block in &self.wipe_tower_blocks {
                if cur_layer_id < block.layer_reserved.len() && block.layer_reserved[cur_layer_id] != 0 {
                    continue;
                }
                let block_fill_height = self.band_fill_depth(block, cur_layer_id as i32) - 2.0 * wall_width;
                if block_fill_height <= WIPE_TOWER_WT_EPSILON {
                    continue;
                }
                fill_boxes.push(BoxCoordinates::from_pos(
                    Vec2f::new(wall_width, block.start_depth + wall_width),
                    self.wipe_tower_width - 2.0 * wall_width,
                    block_fill_height,
                ));
            }
        }
        if fill_boxes.is_empty() && !multi_block_fill {
            let mut fill_box_depth = self.depth - 2.0 * wall_width;
            if self.wipe_tower_blocks.len() == 1 {
                fill_box_depth = self.band_fill_depth(&self.wipe_tower_blocks[0], cur_layer_id as i32) - 2.0 * wall_width;
            }
            fill_boxes.push(BoxCoordinates::from_pos(
                Vec2f::new(wall_width, wall_width),
                self.wipe_tower_width - 2.0 * wall_width,
                fill_box_depth,
            ));
        }

        if !fill_boxes.is_empty() {
            let p = if self.left_to_right { fill_boxes[0].ru } else { fill_boxes[0].lu };
            writer.set_initial_position_wd(p, self.wipe_tower_width, self.depth, self.internal_rotation);
        } else {
            writer.set_initial_position_wd(Vec2f::new(wall_width, wall_width), self.wipe_tower_width, self.depth, self.internal_rotation);
        }

        // :4143-4146
        let mut solid_infill = if self.layer_idx + 1 == self.plan.len() {
            false
        } else {
            self.plan[self.layer_idx + 1]
                .tool_changes
                .iter()
                .any(|tch| self.filament_params[tch.new_tool].is_soluble || self.filament_params[tch.old_tool].is_soluble)
        };
        solid_infill |= first_layer && self.adhesion;

        // :4150-4151
        let capped_width = self.capped_wall_grid_width(self.current_tool as i32);
        let grid_narrowed = capped_width + WIPE_TOWER_WT_EPSILON < wall_width;

        for i in 0..fill_boxes.len() {
            let fill_box = fill_boxes[i];
            if i > 0 {
                let p = if self.left_to_right { fill_box.ru } else { fill_box.lu };
                writer.travel_xy_f(p.x, p.y, 0.0);
            }

            if extrude_fill_wall && (fill_box.ru.y - fill_box.rd.y > WIPE_TOWER_WT_EPSILON) {
                // The rectangle around the grid follows the capped width too, except on the first layer.
                let narrow_rect = grid_narrowed && !first_layer;
                if narrow_rect {
                    let flow = self.extrusion_flow_for(self.layer_height, capped_width);
                    writer.set_line_width(capped_width);
                    writer.set_extrusion_flow(flow);
                    writer.change_analyzer_line_width(capped_width);
                }
                writer.rectangle_fill_box_wt(self, &fill_box, &mut finish_rect_wipe_path, feedrate);
                if narrow_rect {
                    writer.set_line_width(wall_width);
                    writer.set_extrusion_flow(wall_flow);
                    writer.change_analyzer_line_width(wall_width);
                }
            }
            // Extrude infill to support the material to be printed above.
            let dy = fill_box.lu.y - fill_box.ld.y - wall_width;
            let mut left = fill_box.lu.x + 2.0 * wall_width;
            let mut right = fill_box.ru.x - 2.0 * wall_width;

            if extrude_fill && dy > wall_width {
                let p = fill_box.ld + Vec2f::new(wall_width * 2.0, 0.0);
                writer.travel_xy_f(p.x, p.y, 0.0);
                writer.append(";--------------------\n; CP EMPTY GRID START\n");
                writer.comment_with_value(" layer #", self.num_layer_changes.wrapping_add(1) as i32);

                if solid_infill {
                    let mut sparse_factor = 1.5_f32; // 1=solid, 2=every other line, etc.
                    if first_layer {
                        // the infill should touch perimeters
                        left -= wall_width;
                        right += wall_width;
                        sparse_factor = 1.0;
                    }
                    let mut y = fill_box.ld.y + wall_width;
                    let n = (dy / (wall_width * sparse_factor)) as i32;
                    // A single line has no spacing to distribute.
                    let spacing = if n > 1 { (dy - wall_width) / (n - 1) as f32 } else { 0.0 };
                    for i in 0..n {
                        let x = writer.x();
                        writer.extrude_xy_f(x, y, feedrate, LimitFlow::LimitPrintFlow);
                        writer.extrude_xy_f(if i % 2 != 0 { left } else { right }, y, 0.0, LimitFlow::LimitPrintFlow);
                        y = y + spacing;
                    }
                    let x = writer.x();
                    writer.extrude_xy_f(x, fill_box.lu.y, 0.0, LimitFlow::LimitPrintFlow);
                } else {
                    // Extrude an inverse U at the left of the region and the sparse infill.
                    if grid_narrowed {
                        let flow = self.extrusion_flow_for(self.layer_height, capped_width);
                        writer.set_extrusion_flow(flow);
                        writer.change_analyzer_line_width(capped_width);
                    }
                    let p = fill_box.lu + Vec2f::new(wall_width * 2.0, 0.0);
                    writer.extrude_xy_f(p.x, p.y, feedrate, LimitFlow::LimitPrintFlow);

                    // A box narrower than the inset leaves nothing to bridge over.
                    let n = 1 + ((right - left) / self.bridging) as i32;
                    let dx = if n > 0 { (right - left) / n as f32 } else { 0.0 };
                    for i in 1..=n {
                        // left + dx * i: one contracted multiply-add.
                        let x = dx.mul_add(i as f32, left);
                        let y = writer.y();
                        writer.travel_xy_f(x, y, 0.0);
                        writer.extrude_xy_f(x, if i % 2 != 0 { fill_box.rd.y } else { fill_box.ru.y }, 0.0, LimitFlow::LimitPrintFlow);
                    }
                    if grid_narrowed {
                        writer.set_extrusion_flow(wall_flow);
                        writer.change_analyzer_line_width(wall_width);
                    }

                    finish_rect_wipe_path.clear();
                    // BBS: add wipe_path for this case: only with finish rectangle
                    finish_rect_wipe_path.push(writer.pos());
                    finish_rect_wipe_path.push(Vec2f::new(
                        dx.mul_add(n as f32, left),
                        if n % 2 != 0 { fill_box.ru.y } else { fill_box.rd.y },
                    ));
                }

                writer.append("; CP EMPTY GRID END\n;------------------\n\n\n\n\n\n\n");
            }
        }

        // :4226-4234 the perimeter flow of the wall slab.
        let perimeter_flow = self.extrusion_flow_for(wall_height, wall_width);
        writer.set_extrusion_flow(perimeter_flow);
        if extrude_perimeter && (wall_height - self.layer_height).abs() > WIPE_TOWER_WT_EPSILON {
            writer.change_analyzer_layer_height(wall_height);
        }
        let wall_first_layer = self.is_first_layer_for_slab(wall_height);
        writer.set_first_layer(wall_first_layer);
        feedrate = self.calc_tower_feedrate(self.current_tool, perimeter_flow, wall_first_layer);

        // :4238-4245 outer perimeter (always):
        let mut wipe_tower_depth = self.depth;
        if self.wipe_tower_blocks.len() == 1 && !self.keeps_outer_walls_unshrunk() {
            wipe_tower_depth = self.plan[self.layer_idx].depth + self.perimeter_width;
        }
        let mut wt_box = BoxCoordinates::from_pos(Vec2f::new(0.0, 0.0), self.wipe_tower_width, wipe_tower_depth);
        wt_box = self.align_perimeter(&wt_box);

        // :4255-4268
        let outer_wall_narrowed = extrude_perimeter && grid_narrowed && !wall_first_layer;
        let mut drawn_wall_width = wall_width;
        if outer_wall_narrowed {
            drawn_wall_width = capped_width;
        } else if !extrude_perimeter && (self.out_wall_only_z - self.z_pos).abs() < WIPE_TOWER_WT_EPSILON {
            drawn_wall_width = self.out_wall_only_width;
        }
        if outer_wall_narrowed {
            let flow = self.extrusion_flow_for(wall_height, capped_width);
            writer.set_extrusion_flow(flow);
            writer.change_analyzer_line_width(capped_width);
        }
        let (use_rib_wall, use_gap_wall) = (self.use_rib_wall, self.use_gap_wall);
        let mut outer_wall = self.generate_support_wall_new(
            &mut writer,
            &wt_box,
            feedrate as f64,
            wall_first_layer,
            use_rib_wall,
            extrude_perimeter,
            use_gap_wall,
        );
        if outer_wall_narrowed {
            writer.set_extrusion_flow(perimeter_flow);
            writer.change_analyzer_line_width(wall_width);
        }
        if extrude_perimeter {
            let mut shift_polyline = to_polyline(&outer_wall);
            shift_polyline.translate(Point::new(0, scaled_f32(self.y_shift)));
            self.record_outer_wall(shift_polyline);
        }

        // :4287-4331 first layer: fill the voids inside the outer wall (side scrape only).
        if extrude_perimeter && wall_first_layer && self.enable_side_scrape {
            let step = self.extrusion_spacing_for(wall_height, wall_width);
            let mut purged: Vec<Polygon> = Vec::new();
            let perimeter_width = self.perimeter_width;
            let wipe_tower_width = self.wipe_tower_width;
            let mut add_band = |y0: f32, y1: f32| {
                if y1 - y0 <= WIPE_TOWER_WT_EPSILON {
                    return;
                }
                let band = BoxCoordinates::from_pos(Vec2f::new(perimeter_width, y0), wipe_tower_width - 2.0 * perimeter_width, y1 - y0);
                purged.push(Polygon::from_points(vec![
                    point_new_scale_vec2f(band.ld),
                    point_new_scale_vec2f(band.rd),
                    point_new_scale_vec2f(band.ru),
                    point_new_scale_vec2f(band.lu),
                ]));
            };
            if self.wipe_tower_blocks.len() > 1 {
                for block in &self.wipe_tower_blocks {
                    add_band(block.start_depth, block.start_depth + self.band_fill_depth(block, cur_layer_id as i32) - perimeter_width);
                }
            } else {
                for fb in &fill_boxes {
                    add_band(fb.ld.y, fb.lu.y);
                }
            }
            let scaled_step = scaled_f32(step);
            let purged_grown = if purged.is_empty() { Vec::new() } else { clipper_offset_polygons(&purged, scaled_step as f32) };
            let mut voids = clipper_diff_ex_polygons(&clipper_offset_polygon(&outer_wall, (-scaled_step) as f32), &purged_grown);
            // `scaled<double>(step)`: the float quotient as a double.
            let scaled_step_d = (step / crate::libslic3r::SCALING_FACTOR as f32) as f64;
            let min_island_area = scaled_step_d * scaled_step_d;
            crate::ex_polygon::remove_small_and_small_holes(&mut voids, min_island_area);
            let mut guard = 0;
            while guard < 200 && !voids.is_empty() {
                for island in &voids {
                    writer.polygon_f(&island.contour, feedrate);
                    for hole in &island.holes {
                        writer.polygon_f(hole, feedrate);
                    }
                }
                voids = clipper_union_ex_expolygons(&clipper_offset_ex_expolygons(&voids, (-scaled_step) as f32));
                crate::ex_polygon::remove_small_and_small_holes(&mut voids, min_island_area);
                guard += 1;
            }
        }

        // :4336-4345 brim chamfer
        let spacing = self.extrusion_spacing_for(wall_height, wall_width);
        let dist_to_1st = self.layer_idx as i32 - self.first_printed_layer_idx();
        let mut loops_num = self.brim_chamfer_loops_num(wall_height, wall_first_layer, dist_to_1st, wall_width);
        if wall_reserved {
            loops_num = 0;
        }
        if !wall_first_layer && self.plan[self.layer_idx].depth != self.plan[0].depth {
            loops_num = 0;
        }

        if loops_num > 0 {
            // The surrounded wall may be narrower than the loops, so the first loop moves in to stay against it.
            let first_loop_step = self.extrusion_spacing_for(wall_height, 0.5 * (drawn_wall_width + wall_width));
            for i in 0..loops_num as usize {
                let expanded = clipper_offset_polygon(&outer_wall, scaled_f32(if i == 0 { first_loop_step } else { spacing }) as f32);
                if expanded.is_empty() {
                    break;
                }
                outer_wall = expanded.into_iter().next().unwrap();
                writer.polygon_f(&outer_wall, feedrate);
                self.record_outer_wall(to_polyline(&outer_wall));
            }

            if wall_first_layer {
                // first_loop_step + (loops_num - 1) * spacing + spacing / 2.f: the first
                // sum is one contracted multiply-add.
                self.brim_width_real = ((loops_num - 1) as f32).mul_add(spacing, first_loop_step) + spacing / 2.0;
            }
        }

        if extrude_perimeter || loops_num > 0 {
            let wipe_dist = self.filament_params[self.current_tool].wipe_dist;
            writer.add_wipe_path(&outer_wall, wipe_dist as f64);
        } else {
            // Now prepare future wipe. box contains rectangle that was extruded last (ccw).
            let pos = writer.pos();
            let mut target = if pos == wt_box.ld {
                wt_box.rd
            } else if pos == wt_box.rd {
                wt_box.ru
            } else if pos == wt_box.ru {
                wt_box.lu
            } else {
                wt_box.ld
            };
            // BBS: add wipe_path for this case: only with finish rectangle
            if finish_rect_wipe_path.len() == 2 && finish_rect_wipe_path[0] == writer.pos() {
                target = finish_rect_wipe_path[1];
            }
            let pos = writer.pos();
            writer.add_wipe_point(pos).add_wipe_point(target);
        }
        writer.append("; WIPE_TOWER_END\n");

        // Ask our writer about how much material was consumed.
        if !self.sparse_layers_skipped || toolchanges_on_layer {
            if self.current_tool < self.used_filament_length.len() {
                let used = writer.get_and_reset_used_filament_length();
                self.used_filament_length[self.current_tool] += used;
            }
        }

        self.nozzle_change_result = NozzleChangeResult::new_cpp();
        let current_tool = self.current_tool;
        self.construct_tcr(&writer, false, current_tool, true, false, 0.0, false, wall_height)
    }

    /// WipeTower.cpp:4398-4529 `finish_block(block, filament_id, extrude_fill)`:
    /// the block is `wipe_tower_blocks[block_idx]` (nothing here writes a block).
    fn finish_block(&mut self, block_idx: usize, filament_id: i32, extrude_fill: bool) -> ToolChangeResult {
        let block = self.wipe_tower_blocks[block_idx].clone();
        let fid = filament_id as usize;
        // A block can be topped up by a filament of a different diameter than the one that owns it.
        let fill_width = self.filament_perimeter_width(filament_id);
        let slab_height = if block.layer_heights.is_empty() { self.layer_height } else { block.layer_heights[self.cur_layer_id] };
        // slab_height * (fill_width - slab_height * (1 - pi/4)) / area: the inner
        // difference is one contracted multiply-add.
        let filament_e_flow =
            slab_height * (-slab_height).mul_add(1.0 - std::f32::consts::PI / 4.0, fill_width) / self.filament_params[fid].filament_area;

        let mut writer = WipeTowerWriter::new_cpp(slab_height, fill_width, self.gcode_flavor, &self.filament_params, self.enable_arc_fitting);
        writer.set_extrusion_flow(filament_e_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(fid);
        let reversed_shift =
            if self.current_shape == WipeShape::Reversed { self.plan[self.layer_idx].toolchanges_depth() } else { 0.0 };
        writer.set_y_shift(self.y_shift - reversed_shift);

        self.set_for_wipe_tower_writer(&mut writer);
        // Slow down on the 1st layer.
        let first_layer = self.is_first_layer_for_slab(slab_height);
        writer.set_first_layer(first_layer);

        writer.append("; WIPE_TOWER_START\n");

        let feedrate = self.calc_tower_feedrate(fid, filament_e_flow, first_layer);

        // :4424
        let fill_box = BoxCoordinates::from_pos(
            Vec2f::new(fill_width, block.cur_depth),
            self.wipe_tower_width - 2.0 * fill_width,
            block.start_depth + self.band_fill_depth(&block, self.cur_layer_id as i32) - block.cur_depth - fill_width,
        );

        let p = if self.left_to_right { fill_box.ru } else { fill_box.lu };
        writer.set_initial_position_wd(p, self.wipe_tower_width, self.depth, self.internal_rotation);

        let toolchanges_on_layer = self.plan[self.layer_idx].toolchanges_depth() > WIPE_TOWER_WT_EPSILON;

        let mut finish_rect_wipe_path: Vec<Vec2f> = Vec::new();
        let capped_width = self.capped_wall_grid_width(filament_id);
        let grid_narrowed = capped_width + WIPE_TOWER_WT_EPSILON < fill_width;
        let capped_flow = slab_height * self.extrusion_spacing_for(slab_height, capped_width) / self.filament_params[fid].filament_area;
        // inner perimeter of the sparse section, if there is space for it:
        if fill_box.ru.y - fill_box.rd.y > WIPE_TOWER_WT_EPSILON {
            let narrow_rect = grid_narrowed && !first_layer;
            if narrow_rect {
                writer.set_line_width(capped_width);
                writer.set_extrusion_flow(capped_flow);
                writer.change_analyzer_line_width(capped_width);
            }
            writer.rectangle_fill_box_wt(self, &fill_box, &mut finish_rect_wipe_path, feedrate);
            if narrow_rect {
                writer.set_line_width(fill_width);
                writer.set_extrusion_flow(filament_e_flow);
                writer.change_analyzer_line_width(fill_width);
            }
        }

        // Extrude infill to support the material to be printed above.
        let dy = fill_box.lu.y - fill_box.ld.y - fill_width;
        let mut left = fill_box.lu.x + 2.0 * fill_width;
        let mut right = fill_box.ru.x - 2.0 * fill_width;
        if extrude_fill && dy > fill_width {
            let p = fill_box.ld + Vec2f::new(fill_width * 2.0, 0.0);
            writer.travel_xy_f(p.x, p.y, 0.0);
            writer.append(";--------------------\n; CP EMPTY GRID START\n");
            writer.comment_with_value(" layer #", self.num_layer_changes.wrapping_add(1) as i32);

            // Is there a soluble filament wiped/rammed at the next layer?
            let mut solid_infill = if self.layer_idx + 1 == self.plan.len() {
                false
            } else {
                self.plan[self.layer_idx + 1]
                    .tool_changes
                    .iter()
                    .any(|tch| self.filament_params[tch.new_tool].is_soluble || self.filament_params[tch.old_tool].is_soluble)
            };
            solid_infill |= first_layer && self.adhesion;

            if solid_infill {
                let mut sparse_factor = 1.5_f32;
                if first_layer {
                    left -= fill_width;
                    right += fill_width;
                    sparse_factor = 1.0;
                }
                let mut y = fill_box.ld.y + fill_width;
                let n = (dy / (fill_width * sparse_factor)) as i32;
                let spacing = if n > 1 { (dy - fill_width) / (n - 1) as f32 } else { 0.0 };
                for i in 0..n {
                    let x = writer.x();
                    writer.extrude_xy_f(x, y, feedrate, LimitFlow::LimitPrintFlow);
                    writer.extrude_xy_f(if i % 2 != 0 { left } else { right }, y, 0.0, LimitFlow::LimitPrintFlow);
                    y = y + spacing;
                }
                let x = writer.x();
                writer.extrude_xy_f(x, fill_box.lu.y, 0.0, LimitFlow::LimitPrintFlow);
            } else {
                // Extrude an inverse U at the left of the region and the sparse infill.
                if grid_narrowed {
                    writer.set_extrusion_flow(capped_flow);
                    writer.change_analyzer_line_width(capped_width);
                }
                let p = fill_box.lu + Vec2f::new(fill_width * 2.0, 0.0);
                writer.extrude_xy_f(p.x, p.y, feedrate, LimitFlow::LimitPrintFlow);

                let n = 1 + ((right - left) / self.bridging) as i32;
                let dx = if n > 0 { (right - left) / n as f32 } else { 0.0 };
                for i in 1..=n {
                    let x = dx.mul_add(i as f32, left);
                    let y = writer.y();
                    writer.travel_xy_f(x, y, 0.0);
                    writer.extrude_xy_f(x, if i % 2 != 0 { fill_box.rd.y } else { fill_box.ru.y }, 0.0, LimitFlow::LimitPrintFlow);
                }
                if grid_narrowed {
                    writer.set_extrusion_flow(filament_e_flow);
                    writer.change_analyzer_line_width(fill_width);
                }
                finish_rect_wipe_path.clear();
                finish_rect_wipe_path.push(writer.pos());
                finish_rect_wipe_path.push(Vec2f::new(dx.mul_add(n as f32, left), if n % 2 != 0 { fill_box.ru.y } else { fill_box.rd.y }));
            }

            writer.append("; CP EMPTY GRID END\n;------------------\n\n\n\n\n\n\n");
        }

        // :4509-4518 outer perimeter (always): the future wipe.
        let mut wt_box =
            BoxCoordinates::from_pos(Vec2f::new(0.0, 0.0), self.wipe_tower_width, self.plan[self.layer_idx].depth + self.perimeter_width);
        wt_box = self.align_perimeter(&wt_box);

        let pos = writer.pos();
        let mut target = if pos == wt_box.ld {
            wt_box.rd
        } else if pos == wt_box.rd {
            wt_box.ru
        } else if pos == wt_box.ru {
            wt_box.lu
        } else {
            wt_box.ld
        };
        if finish_rect_wipe_path.len() == 2 && finish_rect_wipe_path[0] == writer.pos() {
            target = finish_rect_wipe_path[1];
        }
        let pos = writer.pos();
        writer.add_wipe_point(pos).add_wipe_point(target);

        writer.append("; WIPE_TOWER_END\n");

        if !self.sparse_layers_skipped || toolchanges_on_layer {
            if fid < self.used_filament_length.len() {
                let used = writer.get_and_reset_used_filament_length();
                self.used_filament_length[fid] += used;
            }
        }

        self.construct_block_tcr(&writer, false, fid, true, 0.0, slab_height)
    }

    /// WipeTower.cpp:4531-4656 `finish_block_solid(block, filament_id,
    /// extrude_fill, layer_type)`; the block is `wipe_tower_blocks[block_idx]`.
    fn finish_block_solid(&mut self, block_idx: usize, filament_id: i32, _extrude_fill: bool, layer_type: WipeTowerLayerType) -> ToolChangeResult {
        let block = self.wipe_tower_blocks[block_idx].clone();
        let fid = filament_id as usize;
        let filament_area_val = self.filament_params[fid].filament_area;
        let fill_width = self.filament_perimeter_width(filament_id);
        // :4537-4545
        let slab_height = if block.layer_heights.is_empty() { self.layer_height } else { block.layer_heights[self.cur_layer_id] };
        let mut extrusion_height = slab_height;
        let k = 1.0 - std::f32::consts::PI / 4.0;
        let mut e_flow = slab_height * (-slab_height).mul_add(k, fill_width) / filament_area_val;
        let e_flow_02 = 0.2 * (-0.2_f32).mul_add(k, fill_width) / filament_area_val;
        if !self.merged_slabs_active
            && self.cur_layer_id > 1
            && block.layers_type[self.cur_layer_id - 1] == WipeTowerLayerType::Normal
            && e_flow < e_flow_02
        {
            extrusion_height = 0.2;
            e_flow = e_flow_02;
        }

        let mut writer =
            WipeTowerWriter::new_cpp(extrusion_height, fill_width, self.gcode_flavor, &self.filament_params, self.enable_arc_fitting);
        writer.set_extrusion_flow(e_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(fid);
        let reversed_shift =
            if self.current_shape == WipeShape::Reversed { self.plan[self.layer_idx].toolchanges_depth() } else { 0.0 };
        writer.set_y_shift(self.y_shift - reversed_shift);

        self.set_for_wipe_tower_writer(&mut writer);
        let first_layer = self.is_first_layer_for_slab(slab_height);
        writer.set_first_layer(first_layer);

        writer.append("; WIPE_TOWER_START\n");

        let mut feedrate = self.calc_tower_feedrate(fid, e_flow, first_layer);
        if layer_type == WipeTowerLayerType::Contact || layer_type == WipeTowerLayerType::ContactUp {
            feedrate = self.contact_speed;
        }
        // :4565-4566
        let fill_box = BoxCoordinates::from_pos(
            Vec2f::new(fill_width, block.cur_depth),
            self.wipe_tower_width - 2.0 * fill_width,
            block.start_depth + self.band_fill_depth(&block, self.cur_layer_id as i32) - block.cur_depth - fill_width,
        );
        let toolchanges_on_layer = self.plan[self.layer_idx].toolchanges_depth() > WIPE_TOWER_WT_EPSILON;
        let dy = fill_box.lu.y - fill_box.ld.y;
        // (dy + 0.25 * fill_width) / fill_width + 1 in double; the sum is one
        // contracted multiply-add.
        let n = (0.25_f64.mul_add(fill_width as f64, dy as f64) / fill_width as f64 + 1.0) as i32;
        let spacing = fill_width;
        let mut initial_pos = Vec2f::new(0.0, 0.0);
        let mut up_to_down = false;
        // set initial pos
        {
            let index = self.cur_layer_id % 4;
            let gird_depth = spacing * (n - 1) as f32;
            match index % 4 {
                0 => {
                    initial_pos = fill_box.ld;
                    self.left_to_right = true;
                    up_to_down = false;
                }
                1 => {
                    initial_pos = Vec2f::new(fill_box.rd.x, fill_box.rd.y + gird_depth);
                    self.left_to_right = false;
                    up_to_down = true;
                }
                2 => {
                    initial_pos = fill_box.rd;
                    self.left_to_right = false;
                    up_to_down = false;
                }
                3 => {
                    initial_pos = Vec2f::new(fill_box.ld.x, gird_depth + fill_box.ld.y);
                    self.left_to_right = true;
                    up_to_down = true;
                }
                _ => {}
            }
        }
        // Extrude infill to support the material to be printed above.
        let left = fill_box.lu.x;
        let right = fill_box.ru.x;
        {
            writer.append(";--------------------\n; CP EMPTY GRID START\n");
            writer.comment_with_value(" layer #", self.num_layer_changes.wrapping_add(1) as i32);
            let is_full_block = ((block.cur_depth - block.start_depth).abs() as f64) < crate::libslic3r::EPSILON;
            let ct = self.current_tool;
            if is_full_block && layer_type == WipeTowerLayerType::Contact && self.enable_tower_interface_features {
                let mut stop_pos = initial_pos;
                let filament_tower_interface_pre_extrusion_dist = self.filament_params[ct].filament_tower_interface_pre_extrusion_dist;
                // :4614-4615 — unscaled(get_extents(m_shared_print_bed)), translated by
                // (-m_wipe_tower_pos - m_rib_offset) cast to double.
                let (bmin, bmax) = extents_of_points(self.shared_print_bed.iter().flat_map(|p| p.points().iter()));
                let sf = crate::libslic3r::SCALING_FACTOR;
                let shift = Vec2f::new(-self.pos.x - self.rib_offset.x, -self.pos.y - self.rib_offset.y);
                let bb_min_x = bmin.x as f64 * sf + shift.x as f64;
                let bb_max_x = bmax.x as f64 * sf + shift.x as f64;
                if stop_pos.x < self.wipe_tower_width / 2.0 {
                    stop_pos = Vec2f::new(stop_pos.x - filament_tower_interface_pre_extrusion_dist, stop_pos.y);
                } else {
                    stop_pos = Vec2f::new(stop_pos.x + filament_tower_interface_pre_extrusion_dist, stop_pos.y);
                }
                if (stop_pos.x as f64) < bb_min_x {
                    stop_pos.x = bb_min_x as f32;
                }
                if (stop_pos.x as f64) > bb_max_x {
                    stop_pos.x = bb_max_x as f32;
                }
                initial_pos = stop_pos;
                let p = &self.filament_params[ct];
                if p.filament_tower_interface_print_temp != p.nozzle_temperature as f32 {
                    writer.format_line_m109(p.filament_tower_interface_print_temp as i32, ct as i32, "Wipe tower interface layer enter");
                }
                // (the C++ indentation suggests this is under the if above; it is not)
                writer.retract_cpp(-p.filament_tower_interface_pre_extrusion_length - 2.0, 100.0);
            }
            writer.set_initial_position_wd(initial_pos, self.wipe_tower_width, self.depth, self.internal_rotation);

            for i in 0..n {
                let y = writer.y();
                writer.extrude_xy_f(if self.left_to_right { right } else { left }, y, feedrate, LimitFlow::LimitPrintFlow);
                if i == n - 1 {
                    let pos = writer.pos();
                    let y = writer.y();
                    writer.add_wipe_point(pos).add_wipe_point(Vec2f::new(if self.left_to_right { left } else { right }, y));
                    break;
                }
                self.left_to_right = !self.left_to_right;
                let x = writer.x();
                // writer.y() + spacing * (up_to_down ? -1 : 1): one contracted multiply-add.
                let y = spacing.mul_add(if up_to_down { -1.0 } else { 1.0 }, writer.y());
                writer.extrude_xy_f(x, y, feedrate, LimitFlow::LimitPrintFlow);
            }
            let p = &self.filament_params[ct];
            if layer_type == WipeTowerLayerType::Contact
                && self.enable_tower_interface_features
                && p.filament_tower_interface_print_temp != p.nozzle_temperature as f32
            {
                writer.format_line_m104(
                    p.nozzle_temperature,
                    ct as i32,
                    p.nozzle_temperature as f32 > p.filament_tower_interface_print_temp,
                    true,
                    "Wipe tower interface layer leave restore",
                );
            }
            writer.append("; CP EMPTY GRID END\n;------------------\n\n\n\n\n\n\n");
        }

        writer.append("; WIPE_TOWER_END\n");

        if !self.sparse_layers_skipped || toolchanges_on_layer {
            if fid < self.used_filament_length.len() {
                let used = writer.get_and_reset_used_filament_length();
                self.used_filament_length[fid] += used;
            }
        }

        self.construct_block_tcr(&writer, false, fid, true, 0.0, slab_height)
    }

    /// WipeTower.cpp:8215-8280 `only_generate_out_wall(is_new_mode)`.
    pub fn only_generate_out_wall(&mut self, is_new_mode: bool) -> ToolChangeResult {
        let old_tool = self.current_tool;

        // Drawn like the regular outer wall in finish_layer_new(): capped width, except on the first layer.
        let first_layer = self.is_first_layer();
        let full_width = self.filament_perimeter_width(self.current_tool as i32);
        let capped = self.capped_wall_grid_width(self.current_tool as i32);
        let wall_width = if !first_layer && capped + WIPE_TOWER_WT_EPSILON < full_width { capped } else { full_width };
        let wall_flow = self.extrusion_flow_for(self.layer_height, wall_width);
        self.out_wall_only_z = self.z_pos;
        self.out_wall_only_width = wall_width;

        let mut writer =
            WipeTowerWriter::new_cpp(self.layer_height, wall_width, self.gcode_flavor, &self.filament_params, self.enable_arc_fitting);
        writer.set_extrusion_flow(wall_flow);
        writer.set_z(self.z_pos);
        writer.set_initial_tool(self.current_tool);
        let reversed_shift =
            if self.current_shape == WipeShape::Reversed { self.plan[self.layer_idx].toolchanges_depth() } else { 0.0 };
        writer.set_y_shift(self.y_shift - reversed_shift);

        self.set_for_wipe_tower_writer(&mut writer);

        // Slow down on the 1st layer.
        let feedrate = self.calc_tower_feedrate(self.current_tool, self.extrusion_flow, self.is_first_layer());
        let fill_box_y = self.plan[self.layer_idx].toolchanges_depth() + self.perimeter_width;
        let fill_box = BoxCoordinates::from_pos(
            Vec2f::new(self.perimeter_width, fill_box_y),
            self.wipe_tower_width - 2.0 * self.perimeter_width,
            self.plan[self.layer_idx].depth - fill_box_y,
        );

        let p = if self.left_to_right { fill_box.ru } else { fill_box.lu };
        writer.set_initial_position_wd(p, self.wipe_tower_width, self.depth, self.internal_rotation);

        let toolchanges_on_layer = self.plan[self.layer_idx].toolchanges_depth() > WIPE_TOWER_WT_EPSILON;

        writer.append("; WIPE_TOWER_START\n");
        // outer perimeter (always):
        let mut wipe_tower_depth = self.plan[self.layer_idx].depth + self.perimeter_width;
        if is_new_mode && (self.enable_timelapse_print || self.enable_wrapping_detection || self.keeps_outer_walls_unshrunk()) {
            wipe_tower_depth = self.depth;
        }
        let mut wt_box = BoxCoordinates::from_pos(Vec2f::new(0.0, reversed_shift), self.wipe_tower_width, wipe_tower_depth);
        wt_box = self.align_perimeter(&wt_box);
        let (use_rib_wall, use_gap_wall) = (self.use_rib_wall, self.use_gap_wall);
        let outer_wall =
            self.generate_support_wall_new(&mut writer, &wt_box, feedrate as f64, first_layer, use_rib_wall, true, use_gap_wall);
        self.record_outer_wall(to_polyline(&outer_wall));

        let wipe_dist = self.filament_params[self.current_tool].wipe_dist;
        writer.add_wipe_path(&outer_wall, wipe_dist as f64);
        writer.append("; WIPE_TOWER_END\n");

        if !self.sparse_layers_skipped || toolchanges_on_layer {
            if self.current_tool < self.used_filament_length.len() {
                let used = writer.get_and_reset_used_filament_length();
                self.used_filament_length[self.current_tool] += used;
            }
        }

        self.construct_tcr(&writer, false, old_tool, true, false, 0.0, false, -1.0)
    }

    /// WipeTower.cpp:8282-8312 `generate_rib_polygon(wt_box)`.
    pub fn generate_rib_polygon(&mut self, wt_box: &BoxCoordinates) -> Polygon {
        // float |max - cur| / max * max_len
        let get_current_layer_rib_len = |cur_height: f32, max_height: f32, max_len: f32| -> f32 { (max_height - cur_height).abs() / max_height * max_len };
        let diagonal_width: crate::Coord = scaled_f32(self.rib_width) / 2;
        let a = self.wipe_tower_width;
        let b = self.depth;
        let mut line_1 = Line { a: point_new_scale_vec2f(Vec2f::new(0.0, 0.0)), b: point_new_scale_vec2f(Vec2f::new(a, b)) };
        let mut line_2 = Line { a: point_new_scale_vec2f(Vec2f::new(a, 0.0)), b: point_new_scale_vec2f(Vec2f::new(0.0, b)) };
        let line_1_length = (line_1.length() * crate::libslic3r::SCALING_FACTOR) as f32;
        let mut diagonal_extra_length = 0.0_f32.max(self.rib_length - line_1_length) / 2.0;
        diagonal_extra_length =
            scaled_f32(get_current_layer_rib_len(self.z_pos, self.wipe_tower_height, diagonal_extra_length)) as f32;
        let y_shift = Point::new(0, scaled_f32(self.y_shift));

        line_1.extend(diagonal_extra_length as f64);
        line_2.extend(diagonal_extra_length as f64);
        let neg = Point::new(-y_shift.x, -y_shift.y);
        let line_1 = line_1.translate(neg);
        let line_2 = line_2.translate(neg);

        let poly_1 = generate_rectange(&line_1, diagonal_width);
        let poly_2 = generate_rectange(&line_2, diagonal_width);
        let poly = Polygon::from_points(vec![
            point_new_scale_vec2f(wt_box.ld),
            point_new_scale_vec2f(wt_box.rd),
            point_new_scale_vec2f(wt_box.ru),
            point_new_scale_vec2f(wt_box.lu),
        ]);

        let p_1_2 = clipper_union_polygons(&[poly_1, poly_2, poly.clone()]);
        // A zero-area box or collapsed rib leaves Clipper with nothing; keep the tower rectangle.
        if p_1_2.is_empty() {
            return poly;
        }
        p_1_2.into_iter().next().unwrap()
    }

    /// WipeTower.cpp:8314-8369 `generate_support_wall_new(writer, wt_box, feedrate,
    /// first_layer, rib_wall, extrude_perimeter, skip_points)`. (The C++
    /// `get_closet_idx` lambda is defined there but never called.)
    #[allow(clippy::too_many_arguments)]
    pub fn generate_support_wall_new(
        &mut self,
        writer: &mut WipeTowerWriter,
        wt_box: &BoxCoordinates,
        feedrate: f64,
        first_layer: bool,
        rib_wall: bool,
        extrude_perimeter: bool,
        skip_points: bool,
    ) -> Polygon {
        let retract_length = self.filament_params[self.current_tool].retract_length;
        let retract_speed = self.filament_params[self.current_tool].retract_speed * 60.0;
        let mut wall_polygon = if rib_wall { self.generate_rib_polygon(wt_box) } else { generate_rectange_polygon(wt_box.ld, wt_box.ru) };
        let mut result_wall: Vec<Polyline> = Vec::new();
        let insert_skip_polygon: Polygon;
        if self.used_fillet {
            if !rib_wall && (self.y_shift as f64) > crate::libslic3r::EPSILON {
                // do nothing because the fillet will cause it to be suspended.
            } else {
                // rectangle_wall do nothing
                wall_polygon = if rib_wall {
                    WipeTower::rounding_polygon(&wall_polygon, 2.0, 30.0 / 180.0 * std::f64::consts::PI)
                } else {
                    wall_polygon
                };
                let wt_box_polygon = generate_rectange_polygon(wt_box.ld, wt_box.ru);
                // Fillet can open a gap under the tower body, so merge it back with the rectangle.
                let united = clipper_union_polygons(&[wall_polygon.clone(), wt_box_polygon.clone()]);
                if !united.is_empty() {
                    wall_polygon = united.into_iter().next().unwrap();
                } else if wall_polygon.points().is_empty() {
                    wall_polygon = wt_box_polygon;
                }
            }
        }
        if !extrude_perimeter {
            return wall_polygon;
        }

        if skip_points {
            // 2.5 * m_perimeter_width is passed as the float gap_length.
            let gap_length = (2.5 * self.perimeter_width as f64) as f32 as f64;
            let (pls, insert) =
                contrust_gap_for_skip_points(&wall_polygon, &self.wall_skip_points[self.cur_layer_id], self.wipe_tower_width, gap_length);
            result_wall = pls;
            insert_skip_polygon = insert;
        } else {
            result_wall.push(to_polyline(&wall_polygon));
            insert_skip_polygon = wall_polygon;
        }
        // generate_path simplifies `result_wall` in place, which the extents below read.
        writer.generate_path_cpp(&mut result_wall, feedrate as f32, retract_length, retract_speed, self.used_fillet);
        if first_layer {
            let (bmin, _) = extents_of_points(result_wall.iter().flat_map(|pl| pl.points.iter()));
            let sf = crate::libslic3r::SCALING_FACTOR as f32;
            self.rib_offset = Vec2f::new(-(bmin.x as f32 * sf), -(bmin.y as f32 * sf));
        }
        insert_skip_polygon
    }

    /// WipeTower.cpp:8371-8473 `generate_support_wall(writer, wt_box, feedrate,
    /// first_layer)`: the rectangular wall broken at the skip points of one side.
    pub fn generate_support_wall(&mut self, writer: &mut WipeTowerWriter, wt_box: &BoxCoordinates, feedrate: f64, _first_layer: bool) -> Polygon {
        let retract_length = self.filament_params[self.current_tool].retract_length;
        let retract_speed = self.filament_params[self.current_tool].retract_speed * 60.0;
        let feedrate = feedrate as f32;
        let eps = crate::libslic3r::EPSILON;
        let mut is_left = false;
        let mut is_right = false;
        for pt in &self.wall_skip_points[self.cur_layer_id] {
            if (pt.x.abs() as f64) < eps {
                is_left = true;
            } else if ((pt.x - self.wipe_tower_width).abs() as f64) < eps {
                is_right = true;
            }
        }
        // :8385-8393 — `Vec2f *p = nullptr; p->x();` on both or neither side: an
        // unused read through a null pointer, which the optimiser drops; nothing
        // happens.

        //  3 -------------  2
        //    |           |
        //    |           |
        //  0 -------------  1
        let cur_pos = writer.pos();
        let index_init = if (cur_pos.x - wt_box.ld.x).abs() > (cur_pos.x - wt_box.rd.x).abs() {
            if (cur_pos.y - wt_box.ld.y).abs() > (cur_pos.y - wt_box.lu.y).abs() { 2 } else { 1 }
        } else if (cur_pos.y - wt_box.ld.y).abs() > (cur_pos.y - wt_box.lu.y).abs() {
            3
        } else {
            0
        };
        let mut index: usize = index_init;

        let points = [wt_box.ld, wt_box.rd, wt_box.ru, wt_box.lu];

        writer.travel_xy_f(points[index].x, points[index].y, 0.0);
        let range = 2.5 * self.perimeter_width as f64;
        let mut extruded_nums = 0;
        while extruded_nums < 4 {
            index = (index + 1) % 4;
            if index == 2 {
                if is_right {
                    let break_segments =
                        remove_points_from_segment(&Segment::new(wt_box.rd, wt_box.ru), &self.wall_skip_points[self.cur_layer_id], range);
                    for seg in break_segments.iter() {
                        let dx = seg.start.x - writer.pos().x;
                        let dy = seg.start.y - writer.pos().y;
                        let len = dx.mul_add(dx, dy * dy).sqrt();
                        if len > 0.0 {
                            writer.retract_cpp(retract_length, retract_speed);
                            writer.travel_xy_f(seg.start.x, seg.start.y, 600.0);
                            writer.retract_cpp(-retract_length, retract_speed);
                        } else {
                            writer.travel_xy_f(seg.start.x, seg.start.y, 600.0);
                        }
                        writer.extrude_xy_f(seg.end.x, seg.end.y, feedrate, LimitFlow::LimitPrintFlow);
                    }
                    writer.travel_xy_f(wt_box.ru.x, wt_box.ru.y, feedrate);
                } else {
                    writer.extrude_xy_f(wt_box.ru.x, wt_box.ru.y, feedrate, LimitFlow::LimitPrintFlow);
                }
            } else if index == 0 {
                if is_left {
                    let break_segments =
                        remove_points_from_segment(&Segment::new(wt_box.ld, wt_box.lu), &self.wall_skip_points[self.cur_layer_id], range);
                    for seg in break_segments.iter().rev() {
                        let dx = seg.end.x - writer.pos().x;
                        let dy = seg.end.y - writer.pos().y;
                        let len = dx.mul_add(dx, dy * dy).sqrt();
                        if len > 0.0 {
                            writer.retract_cpp(retract_length, retract_speed);
                            writer.travel_xy_f(seg.end.x, seg.end.y, 600.0);
                            writer.retract_cpp(-retract_length, retract_speed);
                        } else {
                            writer.travel_xy_f(seg.end.x, seg.end.y, 600.0);
                        }
                        writer.extrude_xy_f(seg.start.x, seg.start.y, feedrate, LimitFlow::LimitPrintFlow);
                    }
                    writer.travel_xy_f(wt_box.ld.x, wt_box.ld.y, feedrate);
                } else {
                    writer.extrude_xy_f(wt_box.ld.x, wt_box.ld.y, feedrate, LimitFlow::LimitPrintFlow);
                }
            } else {
                writer.extrude_xy_f(points[index].x, points[index].y, feedrate, LimitFlow::LimitPrintFlow);
            }
            extruded_nums += 1;
        }

        Polygon::new()
    }
}

// ============================================================================
// The C++ generator: the Z ladder and merged slabs (WipeTower.cpp:5256-5426, 5843-7255)
// ============================================================================
// ============================================================================
// The Z ladder of the tower and the merged slabs (WipeTower.cpp:5256-5426,
// 5843-7255): which planned layers the tower prints at all, how thick each
// printed slab is, which purge digs a trench below itself and in which lane,
// and the separate ladders of the band fills and of the outer wall.
// ============================================================================

/// `std::max(a, b)` of C++: `(a < b) ? b : a`.
#[inline]
fn wt_c_max_f64(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// `std::min(a, b)` of C++: `(b < a) ? b : a`.
#[inline]
fn wt_c_min_f64(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// `std::max(a, b)` of C++ on floats.
#[inline]
fn wt_c_max_f32(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}

/// `std::min(a, b)` of C++ on floats.
#[inline]
fn wt_c_min_f32(a: f32, b: f32) -> f32 {
    if b < a { b } else { a }
}

impl WipeTower {
    /// WipeTower.cpp:5256-5353 `plan_band_merge_ladder`: the partition of the
    /// printed steps `[first_step, last_step]` into merge groups, each scored
    /// against a fixed nozzle band (`preferred_height`), or against the
    /// extruding nozzle's maximum layer height with one unit per slab
    /// (`prefer_max_height`); `height_ceiling`, when positive, caps a merged
    /// slab. False when no partition exists.
    #[allow(clippy::too_many_arguments)]
    fn plan_band_merge_ladder(
        &self,
        printed: &[i32],
        first_step: i32,
        last_step: i32,
        bottom_z: f64,
        visitors: &[Vec<i32>],
        reservable: &[i8],
        can_merge_top: &[i8],
        groups: &mut Vec<MergeLadderGroup>,
        preferred_height: f32,
        prefer_max_height: bool,
        height_ceiling: f32,
    ) -> bool {
        let wt_eps = WIPE_TOWER_WT_EPSILON as f64;
        groups.clear();
        if first_step > last_step {
            return true;
        }
        let span = last_step - first_step + 1;

        let mut widest = 0.0_f64;
        for filament_id in 0..self.filament_params.len() as i32 {
            widest = wt_c_max_f64(widest, self.max_layer_height_for_filament(filament_id) as f64);
        }

        // The Z a group tops out at; below the first step of the range the
        // caller tells us what is already there.
        let step_z = |step: i32| -> f64 {
            if step < first_step {
                bottom_z
            } else {
                self.plan[printed[step as usize] as usize].z as f64
            }
        };

        let unreachable = f64::MAX;
        let mut best = vec![unreachable; (span + 1) as usize];
        let mut prev = vec![-1i32; (span + 1) as usize];
        let mut chosen = vec![-1i32; (span + 1) as usize];
        best[0] = 0.0;
        for end in 1..=span {
            let top_step = first_step + end - 1;
            let mut begin = end - 1;
            while begin >= 0 {
                let merged = begin < end - 1;
                if merged && can_merge_top[top_step as usize] == 0 {
                    break;
                }
                // This iteration is the one handing printed[first_step + begin]
                // over to the group above it.
                if merged && reservable[(first_step + begin) as usize] == 0 {
                    break;
                }
                let height = step_z(top_step) - step_z(first_step + begin - 1);
                // :5291 — a block keeps its own nozzle's printable maximum.
                let span_limit = if height_ceiling > WIPE_TOWER_WT_EPSILON {
                    wt_c_min_f64(widest, height_ceiling as f64)
                } else {
                    widest
                };
                if merged && height > span_limit + wt_eps {
                    break;
                }
                if best[begin as usize] == unreachable {
                    begin -= 1;
                    continue;
                }

                let mut filament_id = -1i32;
                let mut penalty = unreachable;
                for &candidate in &visitors[top_step as usize] {
                    if !self.filament_can_print_height(candidate, height as f32) {
                        continue;
                    }
                    let target_height;
                    let candidate_penalty;
                    if prefer_max_height {
                        // :5306-5309 — distance under this nozzle's maximum, plus
                        // one per slab.
                        target_height = self.max_layer_height_for_filament(candidate) as f64;
                        candidate_penalty = 1.0 + wt_c_max_f64(0.0, target_height - height);
                    } else {
                        target_height = if preferred_height > WIPE_TOWER_WT_EPSILON {
                            preferred_height as f64
                        } else {
                            self.preferred_band_size_for_filament(candidate) as f64 * self.tower_grid_layer_height as f64
                        };
                        candidate_penalty = (height - target_height).abs();
                    }
                    if candidate_penalty + wt_eps < penalty {
                        filament_id = candidate;
                        penalty = candidate_penalty;
                    }
                }
                // :5322-5325 — a height no visiting nozzle can extrude is not a slab.
                if filament_id < 0 {
                    begin -= 1;
                    continue;
                }

                // :5327-5337
                let cost = best[begin as usize] + penalty;
                if cost <= best[end as usize] + wt_eps {
                    best[end as usize] = cost;
                    prev[end as usize] = begin;
                    chosen[end as usize] = filament_id;
                }
                begin -= 1;
            }
        }

        if prev[span as usize] < 0 {
            return false;
        }

        let mut end = span;
        while end > 0 {
            let begin = prev[end as usize];
            groups.push(MergeLadderGroup {
                begin_step: first_step + begin,
                top_step: first_step + end - 1,
                filament_id: chosen[end as usize],
                height: (step_z(first_step + end - 1) - step_z(first_step + begin - 1)) as f32,
            });
            end = prev[end as usize];
        }
        groups.reverse();
        // :5350-5351 — short groups join a neighbor only while the extruding
        // nozzle can still print the result.
        self.fold_thin_ladder_groups(visitors, reservable, can_merge_top, groups, height_ceiling);
        true
    }

    /// WipeTower.cpp:5355-5425 `fold_thin_ladder_groups`: groups thinner than
    /// the extruding nozzle's band fold into a neighbor while the result stays
    /// inside that nozzle's window (and under `height_ceiling` when set).
    fn fold_thin_ladder_groups(
        &self,
        visitors: &[Vec<i32>],
        reservable: &[i8],
        can_merge_top: &[i8],
        groups: &mut Vec<MergeLadderGroup>,
        height_ceiling: f32,
    ) {
        let span_is_reservable = |begin: i32, end: i32| -> bool {
            for step in begin..=end {
                if step < 0 || step >= reservable.len() as i32 || reservable[step as usize] == 0 {
                    return false;
                }
            }
            true
        };
        let pick_filament = |top_step: i32, height: f32| -> i32 {
            let mut best = -1i32;
            let mut best_delta = f32::MAX;
            if top_step < 0 || top_step >= visitors.len() as i32 {
                return best;
            }
            for &candidate in &visitors[top_step as usize] {
                if !self.filament_can_print_height(candidate, height) {
                    continue;
                }
                let delta = (height - self.preferred_height_for_filament(candidate)).abs();
                if delta + WIPE_TOWER_WT_EPSILON < best_delta {
                    best = candidate;
                    best_delta = delta;
                }
            }
            best
        };

        let mut folded = true;
        while folded {
            folded = false;
            let mut i = 0usize;
            while i < groups.len() {
                let preferred = self.preferred_height_for_filament(groups[i].filament_id);
                if preferred <= WIPE_TOWER_WT_EPSILON || groups[i].height + WIPE_TOWER_WT_EPSILON >= preferred {
                    i += 1;
                    continue;
                }
                // :5394-5409 — a short group can only join the group above it ...
                if i + 1 < groups.len()
                    && groups[i + 1].top_step >= 0
                    && groups[i + 1].top_step < can_merge_top.len() as i32
                    && can_merge_top[groups[i + 1].top_step as usize] != 0
                    && span_is_reservable(groups[i].begin_step, groups[i].top_step)
                    && !(height_ceiling > WIPE_TOWER_WT_EPSILON
                        && groups[i].height + groups[i + 1].height > height_ceiling + WIPE_TOWER_WT_EPSILON)
                {
                    let height = groups[i].height + groups[i + 1].height;
                    let filament = pick_filament(groups[i + 1].top_step, height);
                    if filament >= 0 {
                        groups[i + 1].begin_step = groups[i].begin_step;
                        groups[i + 1].height = height;
                        groups[i + 1].filament_id = filament;
                        groups.remove(i);
                        folded = true;
                        break;
                    }
                }
                // :5410-5423 — ... or grow downward over the group below.
                if i > 0
                    && groups[i].top_step >= 0
                    && groups[i].top_step < can_merge_top.len() as i32
                    && can_merge_top[groups[i].top_step as usize] != 0
                    && span_is_reservable(groups[i - 1].begin_step, groups[i - 1].top_step)
                    && !(height_ceiling > WIPE_TOWER_WT_EPSILON
                        && groups[i - 1].height + groups[i].height > height_ceiling + WIPE_TOWER_WT_EPSILON)
                {
                    let height = groups[i - 1].height + groups[i].height;
                    let filament = pick_filament(groups[i].top_step, height);
                    if filament >= 0 {
                        groups[i].begin_step = groups[i - 1].begin_step;
                        groups[i].height = height;
                        groups[i].filament_id = filament;
                        groups.remove(i - 1);
                        folded = true;
                        break;
                    }
                }
                i += 1;
            }
        }
    }

    /// WipeTower.cpp:5843-5875 `absorb_run_into_next_layer`: give the layers
    /// `[run_begin, run_end]` to the tool-change layer right above them, which
    /// then prints the whole stack at once. False when that layer cannot take
    /// them.
    fn absorb_run_into_next_layer(&mut self, run_begin: i32, run_end: i32) -> bool {
        let above = run_end + 1;
        if above >= self.plan.len() as i32 || self.plan[above as usize].tool_changes.is_empty() {
            return false;
        }

        let mut grown = self.plan[above as usize].height;
        for layer_id in run_begin..=run_end {
            grown += self.plan[layer_id as usize].height;
        }

        // :5859-5865 — both the incoming and (when it rams) the outgoing
        // filament have to lay the grown slab down.
        for tool_change in &self.plan[above as usize].tool_changes {
            if !self.filament_can_print_height(tool_change.new_tool as i32, grown) {
                return false;
            }
            if self.is_need_ramming(tool_change.old_tool, tool_change.new_tool, above)
                && !self.filament_can_print_height(tool_change.old_tool as i32, grown)
            {
                return false;
            }
        }

        for layer_id in run_begin..=run_end {
            self.layer_skipped[layer_id as usize] = 1;
            self.layer_slab_height[layer_id as usize] = 0.0;
        }
        self.layer_slab_height[above as usize] = grown;
        self.merged_slabs_active = true;
        true
    }

    /// WipeTower.cpp:5877-6072 `plan_tower_ladder`: which planned layers the
    /// tower prints at all and how thick each printed layer is. A run of
    /// tool-change-free layers is one material, so it is split into bands of
    /// the loaded nozzle (`split_adaptive_layer_span`), with a suffix reserved
    /// to thicken the tool-change layer above it.
    fn plan_tower_ladder(&mut self) {
        let wt_eps = WIPE_TOWER_WT_EPSILON;
        let layer_count = self.plan.len() as i32;
        self.layer_slab_height = vec![0.0; layer_count as usize];
        self.layer_skipped = vec![0; layer_count as usize];
        self.merged_slabs_active = false;
        for layer_id in 0..layer_count as usize {
            self.layer_slab_height[layer_id] = self.plan[layer_id].height;
        }

        // :5887-5891 — with one diameter every layer is printable by whatever
        // fills it; sparse-layer compaction, smooth timelapse and wrapping
        // detection rule dropping layers out.
        if !self.merged_fill_active() {
            return;
        }

        let mut loaded_filament = vec![-1i32; layer_count as usize];
        let mut loaded = -1i32;
        for info in &self.plan {
            if !info.tool_changes.is_empty() {
                loaded = info.tool_changes[0].old_tool as i32;
                break;
            }
        }
        for layer_id in 0..layer_count as usize {
            for tool_change in &self.plan[layer_id].tool_changes {
                loaded = tool_change.new_tool as i32;
            }
            loaded_filament[layer_id] = loaded;
        }

        let first_layer = self.first_layer_idx.map_or(0, |i| i as i32);
        let heights: Vec<f64> = self.plan.iter().map(|info| info.height as f64).collect();

        // The float sum of the planned heights of [bottom, top].
        let span_height = |plan: &[WipeTowerLayerInfo], bottom: usize, top: usize| -> f32 {
            let mut height = 0.0_f32;
            for layer_id in bottom..=top {
                height += plan[layer_id].height;
            }
            height
        };
        // :5938-5951 apply_group
        let apply_group = |this: &mut WipeTower, group: (usize, usize)| {
            let mut merged_height = 0.0_f32;
            for layer_id in group.0..=group.1 {
                merged_height += this.plan[layer_id].height;
            }
            for layer_id in group.0..group.1 {
                this.layer_skipped[layer_id] = 1;
                this.layer_slab_height[layer_id] = 0.0;
            }
            this.layer_slab_height[group.1] = merged_height;
            if group.0 != group.1 {
                this.merged_slabs_active = true;
            }
        };

        let mut run_begin = if self.foundation_required { first_layer + 1 } else { first_layer };
        while run_begin < layer_count {
            while run_begin < layer_count && !self.plan[run_begin as usize].tool_changes.is_empty() {
                run_begin += 1;
            }
            if run_begin >= layer_count {
                break;
            }

            let mut run_end = run_begin;
            while run_end + 1 < layer_count
                && self.plan[(run_end + 1) as usize].tool_changes.is_empty()
                && loaded_filament[(run_end + 1) as usize] == loaded_filament[run_begin as usize]
            {
                run_end += 1;
            }

            // :5925-5932 — before any real first-layer extrusion the prefix is
            // handed to the first required tool change.
            if !self.foundation_required && run_begin == first_layer && self.absorb_run_into_next_layer(run_begin, run_end) {
                run_begin = run_end + 1;
                continue;
            }

            let filament_id = loaded_filament[run_begin as usize];
            if filament_id >= 0 {
                let mut groups: Vec<(usize, usize)> = Vec::new();
                let band_size = self.preferred_band_size_for_filament(filament_id);
                let range = crate::adaptive_layer_merge::AdaptiveLayerHeightRange {
                    min_height: self.min_layer_height_for_filament(filament_id) as f64,
                    max_height: self.max_layer_height_for_filament(filament_id) as f64,
                };
                let preferred = self.preferred_height_for_filament(filament_id);
                // :5961 — stay inside the printable window.
                let ceiling = self.max_layer_height_for_filament(filament_id);
                // :5964-5987 fold_short_groups
                let fold_short_groups = |plan: &[WipeTowerLayerInfo], groups: &mut Vec<(usize, usize)>| {
                    let mut folded = true;
                    while folded {
                        folded = false;
                        let mut i = 0usize;
                        while i < groups.len() {
                            if span_height(plan, groups[i].0, groups[i].1) + wt_eps >= preferred {
                                i += 1;
                                continue;
                            }
                            if i + 1 < groups.len() && span_height(plan, groups[i].0, groups[i + 1].1) <= ceiling + wt_eps {
                                groups[i + 1].0 = groups[i].0;
                                groups.remove(i);
                                folded = true;
                                break;
                            }
                            if i > 0 && span_height(plan, groups[i - 1].0, groups[i].1) <= ceiling + wt_eps {
                                groups[i].0 = groups[i - 1].0;
                                groups.remove(i - 1);
                                folded = true;
                                break;
                            }
                            i += 1;
                        }
                    }
                };
                // :5988-6024 — reserve a suffix of the run for the tool-change
                // layer above it before the split.
                let mut suffix = run_end + 1;
                let mut suffix_limit = ceiling;
                let above = run_end + 1;
                if above < layer_count && !self.plan[above as usize].tool_changes.is_empty() {
                    let mut target = 0.0_f32;
                    suffix_limit = f32::MAX;
                    for tool_change in &self.plan[above as usize].tool_changes {
                        let new_tool = tool_change.new_tool as i32;
                        target = wt_c_max_f32(target, self.preferred_height_for_filament(new_tool));
                        suffix_limit = wt_c_min_f32(suffix_limit, self.max_layer_height_for_filament(new_tool));
                        if self.is_need_ramming(tool_change.old_tool, tool_change.new_tool, above) {
                            let old_tool = tool_change.old_tool as i32;
                            target = wt_c_max_f32(target, self.preferred_height_for_filament(old_tool));
                            suffix_limit = wt_c_min_f32(suffix_limit, self.max_layer_height_for_filament(old_tool));
                        }
                    }
                    target = wt_c_min_f32(target, suffix_limit);
                    let mut grown = self.plan[above as usize].height;
                    if grown + wt_eps < target {
                        let mut layer_id = run_end;
                        while layer_id >= run_begin {
                            let next = grown + self.plan[layer_id as usize].height;
                            if next > suffix_limit + wt_eps {
                                break;
                            }
                            grown = next;
                            suffix = layer_id;
                            if grown + wt_eps >= target {
                                break;
                            }
                            layer_id -= 1;
                        }
                        // :6021-6023 — a suffix that misses the full band is
                        // kept only when it lifts the purge off the grid step.
                        if !(suffix <= run_end && grown + wt_eps > self.plan[above as usize].height) {
                            suffix = run_end + 1;
                        }
                    }
                }
                let plan_end = suffix - 1;
                let split_ok = plan_end >= run_begin
                    && crate::adaptive_layer_merge::split_adaptive_layer_span(
                        &heights,
                        run_begin as usize,
                        plan_end as usize,
                        band_size,
                        &range,
                        &mut groups,
                    );
                if plan_end < run_begin {
                    if !self.absorb_run_into_next_layer(run_begin, run_end) && crate::probe_enabled("WT_LADDER_LOG") {
                        eprintln!(
                            "WipeTower: cannot hand layers {}..{} to the tool change above them; keeping their base layer heights",
                            run_begin, run_end
                        );
                    }
                } else if split_ok {
                    fold_short_groups(&self.plan, &mut groups);
                    let mut pulled_tail: (usize, usize) = (0, 0);
                    let mut pulled = false;
                    if !groups.is_empty() && suffix <= run_end {
                        let back = *groups.last().unwrap();
                        let tail = span_height(&self.plan, back.0, back.1);
                        let grown_tail = span_height(&self.plan, suffix as usize, run_end as usize) + self.plan[above as usize].height;
                        if tail + wt_eps < preferred && grown_tail + tail <= suffix_limit + wt_eps {
                            pulled_tail = back;
                            pulled = true;
                            suffix = back.0 as i32;
                            groups.pop();
                        }
                    }
                    for group in &groups {
                        apply_group(self, *group);
                    }
                    if suffix <= run_end && !self.absorb_run_into_next_layer(suffix, run_end) {
                        if pulled {
                            apply_group(self, pulled_tail);
                        }
                        if crate::probe_enabled("WT_LADDER_LOG") {
                            eprintln!(
                                "WipeTower: cannot thicken the tool change at layer {} with layers {}..{}; keeping their base layer heights",
                                above, suffix, run_end
                            );
                        }
                    }
                } else if !self.absorb_run_into_next_layer(run_begin, run_end) && crate::probe_enabled("WT_LADDER_LOG") {
                    eprintln!(
                        "WipeTower: cannot apply the model Z merge plan to layers {}..{} with filament {}; keeping their base layer heights",
                        run_begin, run_end, filament_id
                    );
                }
            }
            run_begin = run_end + 1;
        }
    }
}

/// WipeTower.cpp:6167-6172 `GroupEvent` of `plan_merged_slabs`: one band's
/// share of a printed step (the filaments extruding into it and the depth they
/// claim at the base layer height).
#[derive(Debug, Clone, Default)]
struct WtGroupEvent {
    step: i32,
    filaments: Vec<i32>,
    base_depth: f32,
}

/// WipeTower.cpp:6194-6199 `PlannedPurge` of `plan_merged_slabs`.
#[derive(Debug, Clone, Copy)]
struct WtPlannedPurge {
    lane: i32,
    merge: i32,
    fitted: bool,
}

impl Default for WtPlannedPurge {
    fn default() -> Self {
        WtPlannedPurge { lane: 0, merge: 1, fitted: true }
    }
}

/// WipeTower.cpp:6200-6206 `GroupPlan` of `plan_merged_slabs`.
#[derive(Debug, Clone, Default)]
struct WtGroupPlan {
    purges: Vec<WtPlannedPurge>,
    thin_layers: i32,
    depth: f32,
    lanes: i32,
}

/// WipeTower.cpp:6283-6290 `SlabChoice` of `plan_merged_slabs`.
#[derive(Debug, Clone, Copy)]
struct WtSlabChoice {
    merge: i32,
    lane: i32,
    thin: i32,
    delta: f32,
}

impl Default for WtSlabChoice {
    fn default() -> Self {
        WtSlabChoice { merge: 1, lane: -1, thin: i32::MAX, delta: f32::MAX }
    }
}

impl WtSlabChoice {
    fn valid(&self) -> bool {
        self.thin != i32::MAX
    }
}

impl WipeTower {
    /// WipeTower.cpp:6128-6511 `plan_merged_slabs`: which purge events dig a
    /// trench below themselves (a slab merged over the steps under it) and in
    /// which lane of their band each one goes. Fills `lane_assignment`,
    /// `block_slab_height` and `reserved_blocks`.
    fn plan_merged_slabs(&mut self) {
        self.lane_assignment = vec![HashMap::new(); self.plan.len()];
        self.block_slab_height = vec![HashMap::new(); self.plan.len()];
        self.reserved_blocks = vec![std::collections::HashSet::new(); self.plan.len()];

        // :6136-6137
        if !self.merged_fill_active() {
            return;
        }

        let mut printed: Vec<i32> = Vec::new();
        self.collect_printed_steps(&mut printed);
        if printed.is_empty() {
            return;
        }
        let bed_z = self.tower_bed_z();

        // :6148
        const CELL_FREE: u8 = 0;
        const CELL_PURGE: u8 = 1;
        const CELL_RESERVED: u8 = 2;
        const MAX_LANES: i32 = 4; // more lanes means a deeper tower, so keep the budget tight
        const MAX_ENCODABLE_LANES: i32 = 16; // the lane field of the block key is 4 bits wide
        const MAX_MERGE_LAYERS: i32 = 8;
        const UNFILLABLE: i32 = i32::MAX / 2;
        let step_count = printed.len() as i32;
        let wt_eps = WIPE_TOWER_WT_EPSILON;

        // :6155 — top_z: float.
        let top_z = |this: &WipeTower, step: i32| -> f32 {
            if step < 0 { bed_z as f32 } else { this.plan[printed[step as usize] as usize].z }
        };
        // :6158-6165 — a lane can host the event if the purge step is not
        // reserved and every step the slab reaches into is untouched.
        let lane_fits = |lane: &[u8], step: i32, merge_layers: i32| -> bool {
            if lane[step as usize] == CELL_RESERVED {
                return false;
            }
            for l in (step - merge_layers + 1)..step {
                if lane[l as usize] != CELL_FREE {
                    return false;
                }
            }
            true
        };

        // :6174-6192 — every filament extruding into a band on a layer, keyed
        // by group (std::map: ordered).
        let mut group_events: std::collections::BTreeMap<i32, Vec<WtGroupEvent>> = std::collections::BTreeMap::new();
        let mut category_groups: std::collections::BTreeMap<i32, std::collections::BTreeSet<i32>> = std::collections::BTreeMap::new();
        for step in 0..step_count {
            let layer_id = printed[step as usize];
            let mut on_layer: std::collections::BTreeMap<i32, WtGroupEvent> = std::collections::BTreeMap::new();
            for tool_change in &self.plan[layer_id as usize].tool_changes {
                let new_tool = tool_change.new_tool as i32;
                let old_tool = tool_change.old_tool as i32;
                {
                    let wipe = on_layer.entry(self.get_gap_key(new_tool)).or_default();
                    wipe.filaments.push(new_tool);
                    // std::max(0.f, required_depth - nozzle_change_depth)
                    let d = tool_change.required_depth - tool_change.nozzle_change_depth;
                    wipe.base_depth += if 0.0 < d { d } else { 0.0 };
                }
                category_groups.entry(self.get_filament_category(new_tool as usize)).or_default().insert(self.get_gap_key(new_tool));
                category_groups.entry(self.get_filament_category(old_tool as usize)).or_default().insert(self.get_gap_key(old_tool));
                if tool_change.nozzle_change_depth as f64 > crate::libslic3r::EPSILON
                    && self.is_need_ramming(tool_change.old_tool, tool_change.new_tool, layer_id)
                {
                    let ramming = on_layer.entry(self.get_gap_key(old_tool)).or_default();
                    ramming.filaments.push(old_tool);
                    ramming.base_depth += tool_change.nozzle_change_depth;
                }
            }
            for (key, mut event) in on_layer {
                event.step = step;
                group_events.entry(key).or_default().push(event);
            }
        }

        // :6211-6430 plan_group
        let plan_group = |this: &WipeTower, group_key: i32, events: &[WtGroupEvent], lanes_for_thinness: bool| -> WtGroupPlan {
            let filament = events[0].filaments[0];
            let min_height = this.min_layer_height_for_filament(filament);
            let max_height = this.max_layer_height_for_filament(filament);
            let preferred_height = this.preferred_height_for_filament(filament);
            // :6218-6220
            let own_nozzle_fill = category_groups
                .get(&WipeTower::decode_block_category(group_key))
                .map_or(0, |s| s.len())
                <= 1;
            let is_thin = |height: f32| height + wt_eps < preferred_height;

            // :6222-6236 — NoFill / SingleFill / MergedFill.
            const NO_FILL: i8 = 0;
            const SINGLE_FILL: i8 = 1;
            const MERGED_FILL: i8 = 2;
            let mut can_fill = vec![NO_FILL; step_count as usize];
            for step in 0..step_count {
                let layer = &this.plan[printed[step as usize] as usize];
                if this.same_category_filler(layer, group_key) >= 0 {
                    can_fill[step as usize] = MERGED_FILL;
                } else if layer.tool_changes.is_empty() {
                    let loaded = this.loaded_filament(printed[step as usize]);
                    if loaded >= 0 && this.get_filament_category(loaded as usize) == WipeTower::decode_block_category(group_key) {
                        can_fill[step as usize] = SINGLE_FILL;
                    }
                }
            }

            // :6238-6274 fill_thin_layers
            let fill_thin_layers = |from: i32, to: i32| -> i32 {
                if to <= from {
                    return 0;
                }
                let mut any_own = false;
                let mut t = 0.max(from + 1);
                while t <= to && t < step_count {
                    if can_fill[t as usize] != NO_FILL {
                        any_own = true;
                    }
                    t += 1;
                }
                if !any_own {
                    return 0;
                }
                if can_fill[to as usize] == NO_FILL {
                    return UNFILLABLE;
                }
                if !own_nozzle_fill {
                    return 0;
                }
                let mut best = vec![UNFILLABLE; (to - from + 1) as usize];
                best[0] = 0;
                for t in (from + 1)..=to {
                    if can_fill[t as usize] == NO_FILL {
                        continue;
                    }
                    let lowest = if can_fill[t as usize] == SINGLE_FILL { t - 1 } else { from };
                    let mut u = t - 1;
                    while u >= lowest {
                        let height = top_z(this, t) - top_z(this, u);
                        if height > max_height + wt_eps {
                            break;
                        }
                        if height + wt_eps < min_height || best[(u - from) as usize] >= UNFILLABLE {
                            u -= 1;
                            continue;
                        }
                        let cand = best[(u - from) as usize] + if is_thin(height) { 1 } else { 0 };
                        let cur = best[(t - from) as usize];
                        best[(t - from) as usize] = if cand < cur { cand } else { cur };
                        u -= 1;
                    }
                }
                best[(to - from) as usize]
            };
            let surface_step = |lane: &[u8], below_step: i32| -> i32 {
                let mut l = below_step;
                while l >= 0 {
                    if lane[l as usize] == CELL_PURGE {
                        return l;
                    }
                    l -= 1;
                }
                -1
            };
            // :6291-6297
            let better = |choice: &WtSlabChoice, best: &WtSlabChoice| -> bool {
                if choice.thin != best.thin {
                    return choice.thin < best.thin;
                }
                if choice.lane != best.lane {
                    return choice.lane < best.lane;
                }
                choice.delta <= best.delta + wt_eps
            };

            let mut plan = WtGroupPlan::default();
            let mut lanes: Vec<Vec<u8>> = vec![vec![CELL_FREE; step_count as usize]];
            let mut lane_depth: Vec<f32> = vec![0.0];
            for event in events {
                let step = event.step;
                let max_available = (step + 1).min(MAX_MERGE_LAYERS);
                let mut in_use = WtSlabChoice::default();
                let mut fresh = WtSlabChoice::default();
                let mut unreachable_merge = 1i32;
                let mut unreachable_delta = f32::MAX;
                for candidate in 1..=max_available {
                    let height = top_z(this, step) - top_z(this, step - candidate);
                    // :6320-6323
                    if height > max_height + wt_eps {
                        break;
                    }
                    if height + wt_eps < min_height {
                        continue;
                    }
                    let delta = (height - preferred_height).abs();
                    if delta <= unreachable_delta + wt_eps {
                        unreachable_merge = candidate;
                        unreachable_delta = delta;
                    }
                    let below = step - candidate;
                    let slab_thin = if is_thin(height) { 1 } else { 0 };
                    // :6333-6339 — a fresh lane starts at the bed.
                    let fresh_fill = fill_thin_layers(-1, below);
                    if fresh_fill < UNFILLABLE {
                        let choice = WtSlabChoice { merge: candidate, lane: -1, thin: slab_thin + fresh_fill, delta };
                        if better(&choice, &fresh) {
                            fresh = choice;
                        }
                    }
                    for idx in 0..lanes.len() as i32 {
                        if !lane_fits(&lanes[idx as usize], step, candidate) {
                            continue;
                        }
                        let fill = fill_thin_layers(surface_step(&lanes[idx as usize], below), below);
                        if fill >= UNFILLABLE {
                            continue;
                        }
                        let choice = WtSlabChoice { merge: candidate, lane: idx, thin: slab_thin + fill, delta };
                        if better(&choice, &in_use) {
                            in_use = choice;
                        }
                    }
                }

                // :6352-6354
                let lanes_left = (lanes.len() as i32) < MAX_LANES;
                let open_lane = lanes_for_thinness && in_use.valid() && fresh.valid() && lanes_left && fresh.thin < in_use.thin;
                let mut purge = WtPlannedPurge::default();
                let mut lane = -1i32;
                if in_use.valid() && !open_lane {
                    purge.merge = in_use.merge;
                    lane = in_use.lane;
                }
                if lane < 0 {
                    // :6360-6368
                    if !fresh.valid() && crate::probe_enabled("WT_LADDER_LOG") {
                        eprintln!(
                            "WipeTower: no slab of group {} on layer {} can be topped up with its own material",
                            group_key, printed[step as usize]
                        );
                    }
                    purge.merge = if fresh.valid() { fresh.merge } else { unreachable_merge };
                    if !(open_lane && fresh.valid()) {
                        for candidate in 0..lanes.len() as i32 {
                            if lane_fits(&lanes[candidate as usize], step, purge.merge) {
                                lane = candidate;
                                break;
                            }
                        }
                        // :6375-6398 — stay on a lane whose purge cell is free at
                        // the thickest height it can host.
                        if lane < 0 {
                            for candidate in 0..lanes.len() as i32 {
                                if lanes[candidate as usize][step as usize] == CELL_RESERVED {
                                    continue;
                                }
                                let mut fit_merge = 0i32;
                                for m in 1..=max_available {
                                    if !lane_fits(&lanes[candidate as usize], step, m) {
                                        break;
                                    }
                                    let height = top_z(this, step) - top_z(this, step - m);
                                    if height > max_height + wt_eps {
                                        break;
                                    }
                                    if height + wt_eps < min_height {
                                        continue;
                                    }
                                    fit_merge = m;
                                }
                                // A free cell is not a licence to print below the nozzle minimum.
                                if fit_merge == 0 {
                                    continue;
                                }
                                purge.merge = fit_merge;
                                lane = candidate;
                                break;
                            }
                        }
                    }
                    if lane < 0 && lanes_left && (open_lane || fresh.valid()) {
                        lanes.push(vec![CELL_FREE; step_count as usize]);
                        lane_depth.push(0.0);
                        lane = lanes.len() as i32 - 1;
                    }
                }
                if lane < 0 {
                    // :6405-6436 — the last resort.
                    purge.fitted = false;
                    let single = top_z(this, step) - top_z(this, step - 1);
                    let single_ok = single + wt_eps >= min_height && single <= max_height + wt_eps;
                    if unreachable_delta < f32::MAX {
                        for candidate in 0..lanes.len() as i32 {
                            if lane_fits(&lanes[candidate as usize], step, unreachable_merge) {
                                purge.merge = unreachable_merge;
                                lane = candidate;
                                break;
                            }
                        }
                    }
                    if lane < 0 && single_ok {
                        purge.merge = 1;
                        for candidate in 0..lanes.len() as i32 {
                            if lanes[candidate as usize][step as usize] != CELL_RESERVED {
                                lane = candidate;
                                break;
                            }
                        }
                    }
                    if lane < 0 && unreachable_delta < f32::MAX {
                        purge.merge = unreachable_merge;
                    } else if lane < 0 && !single_ok && crate::probe_enabled("WT_LADDER_LOG") {
                        eprintln!(
                            "WipeTower: group {} on layer {} has no slab inside {}..{} mm",
                            group_key, printed[step as usize], min_height, max_height
                        );
                    }
                    if lane < 0 && (lanes.len() as i32) < MAX_ENCODABLE_LANES {
                        lanes.push(vec![CELL_FREE; step_count as usize]);
                        lane_depth.push(0.0);
                        lane = lanes.len() as i32 - 1;
                    }
                    if lane < 0 {
                        lane = 0; // The lane bits of the block key are exhausted; share the first one.
                    }
                }
                purge.lane = lane;

                // :6440-6455
                let below = step - purge.merge;
                let slab_height = top_z(this, step) - top_z(this, below);
                let fill = fill_thin_layers(surface_step(&lanes[lane as usize], below), below);
                plan.thin_layers += (if is_thin(slab_height) { 1 } else { 0 }) + if fill < UNFILLABLE { fill } else { 0 };
                let lane_state = &mut lanes[lane as usize];
                for l in (below + 1)..step {
                    lane_state[l as usize] = CELL_RESERVED;
                }
                lane_state[step as usize] = CELL_PURGE;
                let base_height = this.plan[printed[step as usize] as usize].height;
                let footprint = if slab_height as f64 > crate::libslic3r::EPSILON {
                    event.base_depth * base_height / slab_height
                } else {
                    event.base_depth
                };
                lane_depth[lane as usize] = wt_c_max_f32(lane_depth[lane as usize], footprint);
                plan.purges.push(purge);
            }
            plan.lanes = lanes.len() as i32;
            for depth in &lane_depth {
                plan.depth += *depth;
            }
            plan
        };

        // :6463-6510
        for (&group_key, events) in &group_events {
            // One block per material and diameter (WipeTower.cpp:6466-6471).
            let few_lanes = plan_group(self, group_key, events, false);
            let chosen = &few_lanes;
            for (idx, event) in events.iter().enumerate() {
                let purge = chosen.purges[idx];
                let step = event.step;
                let layer_id = printed[step as usize];
                let block_key = (group_key & !0xF) | purge.lane;
                let slab_height = top_z(self, step) - top_z(self, step - purge.merge);
                for &filament in &event.filaments {
                    self.lane_assignment[layer_id as usize].insert(filament, purge.lane);
                }
                self.block_slab_height[layer_id as usize].insert(block_key, slab_height);
                for l in (step - purge.merge + 1)..step {
                    self.reserved_blocks[printed[l as usize] as usize].insert(block_key);
                }
                if purge.merge > 1 {
                    self.merged_slabs_active = true;
                }
                if !purge.fitted && crate::probe_enabled("WT_LADDER_LOG") {
                    eprintln!(
                        "WipeTower: no lane left for a merged purge on layer {}, falling back to the tower layer height",
                        layer_id
                    );
                }
            }
        }
    }

    /// WipeTower.cpp:6530-6558 `collect_tower_visitors`: the filaments visiting
    /// the tower on each printed step (those of its tool changes, or the loaded
    /// one on a layer without), sorted by nozzle diameter. The C++ collects
    /// them in a `std::unordered_set<int>` and sorts that with `std::sort`, so
    /// the order of equal diameters is libc++'s: emulated with
    /// `LibcxxUnorderedMap` (the identity hash of `std::hash<int>`) and
    /// `libcxx_sort`.
    fn collect_tower_visitors(&self, printed: &[i32], visitors: &mut Vec<Vec<i32>>) {
        visitors.clear();
        visitors.resize(printed.len(), Vec::new());

        let mut loaded = -1i32;
        for info in &self.plan {
            if !info.tool_changes.is_empty() {
                loaded = info.tool_changes[0].old_tool as i32;
                break;
            }
        }
        let mut loaded_filament = vec![-1i32; self.plan.len()];
        for layer_id in 0..self.plan.len() {
            for tool_change in &self.plan[layer_id].tool_changes {
                loaded = tool_change.new_tool as i32;
            }
            loaded_filament[layer_id] = loaded;
        }

        for step in 0..printed.len() {
            let layer_id = printed[step] as usize;
            let mut seen: crate::texture_to_color::color_utils::LibcxxUnorderedMap<()> =
                crate::texture_to_color::color_utils::LibcxxUnorderedMap::new();
            for tool_change in &self.plan[layer_id].tool_changes {
                seen.insert(tool_change.old_tool);
                seen.insert(tool_change.new_tool);
            }
            if seen.is_empty() && loaded_filament[layer_id] >= 0 {
                seen.insert(loaded_filament[layer_id] as usize);
            }
            visitors[step] = seen.keys().into_iter().map(|k| k as i32).collect();
            crate::clipper::libcxx_sort(&mut visitors[step], |lhs: &i32, rhs: &i32| {
                self.filament_params[*lhs as usize].nozzle_diameter < self.filament_params[*rhs as usize].nozzle_diameter
            });
        }
    }

    /// WipeTower.cpp:6560-6641 `plan_merge_ladder`: the partition of the
    /// printed steps `[first_step, last_step]` into merge groups that merges
    /// only where a step cannot be printed on its own (every swallowed step
    /// costs one; an unprintable single step costs 1e6). False when no
    /// partition exists.
    #[allow(clippy::too_many_arguments)]
    fn plan_merge_ladder(
        &self,
        printed: &[i32],
        first_step: i32,
        last_step: i32,
        bottom_z: f64,
        visitors: &[Vec<i32>],
        reservable: &[i8],
        can_merge_top: &[i8],
        groups: &mut Vec<MergeLadderGroup>,
    ) -> bool {
        let wt_eps = WIPE_TOWER_WT_EPSILON as f64;
        groups.clear();
        if first_step > last_step {
            return true;
        }
        let span = last_step - first_step + 1;

        let mut widest = 0.0_f64;
        for filament_id in 0..self.filament_params.len() as i32 {
            widest = wt_c_max_f64(widest, self.max_layer_height_for_filament(filament_id) as f64);
        }

        let step_z = |step: i32| -> f64 {
            if step < first_step {
                bottom_z
            } else {
                self.plan[printed[step as usize] as usize].z as f64
            }
        };

        let unreachable = f64::MAX;
        // :6580-6581
        let unfit_cost = 1.0e6_f64;
        let mut best = vec![unreachable; (span + 1) as usize];
        let mut prev = vec![-1i32; (span + 1) as usize];
        let mut chosen = vec![-1i32; (span + 1) as usize];
        best[0] = 0.0;
        for end in 1..=span {
            let top_step = first_step + end - 1;
            let mut begin = end - 1;
            while begin >= 0 {
                let merged = begin < end - 1;
                if merged && can_merge_top[top_step as usize] == 0 {
                    break;
                }
                if merged && reservable[(first_step + begin) as usize] == 0 {
                    break;
                }
                let height = step_z(top_step) - step_z(first_step + begin - 1);
                if merged && height > widest + wt_eps {
                    break;
                }
                if best[begin as usize] == unreachable {
                    begin -= 1;
                    continue;
                }

                let mut filament_id = -1i32;
                let mut penalty = 0.0_f64;
                for &candidate in &visitors[top_step as usize] {
                    if self.filament_can_print_height(candidate, height as f32) {
                        filament_id = candidate;
                        break;
                    }
                }
                if filament_id < 0 {
                    // :6606-6612 — only a single step may stay on a height none
                    // of its nozzles covers.
                    if merged || visitors[top_step as usize].is_empty() {
                        begin -= 1;
                        continue;
                    }
                    filament_id = visitors[top_step as usize][0];
                    penalty = unfit_cost;
                }

                // :6614-6622
                let cost = best[begin as usize] + penalty + (end - 1 - begin) as f64;
                if cost + wt_eps < best[end as usize] {
                    best[end as usize] = cost;
                    prev[end as usize] = begin;
                    chosen[end as usize] = filament_id;
                }
                begin -= 1;
            }
        }

        if prev[span as usize] < 0 {
            return false;
        }

        let mut end = span;
        while end > 0 {
            let begin = prev[end as usize];
            groups.push(MergeLadderGroup {
                begin_step: first_step + begin,
                top_step: first_step + end - 1,
                filament_id: chosen[end as usize],
                height: (step_z(first_step + end - 1) - step_z(first_step + begin - 1)) as f32,
            });
            end = prev[end as usize];
        }
        groups.reverse();
        true
    }
}

/// WipeTower.cpp:7050-7055 `PrintedSlab` of `plan_merged_block_fill`.
#[derive(Debug, Clone, Copy)]
struct WtPrintedSlab {
    layer_id: i32,
    filament: i32,
    top_z: f32,
    bottom_z: f32,
}

impl WipeTower {
    /// WipeTower.cpp:6643-7091 `plan_merged_block_fill`: the fill that tops
    /// every band up to its full depth gets its own Z ladder over the layers
    /// between its purges (`layer_heights`, `layer_reserved`,
    /// `layer_fill_filament`, `layers_type` of the blocks), then the material
    /// interfaces of each band are marked Contact.
    ///
    /// The C++ walks `m_wipe_tower_blocks` by reference; each block is taken
    /// out of the list while it is planned (nothing called on the way reads
    /// the list) and put back, and `m_merged_slabs_active`, which nothing reads
    /// on the way either, is set when the block is put back.
    fn plan_merged_block_fill(&mut self) {
        let wt_eps = WIPE_TOWER_WT_EPSILON;
        if !self.merged_fill_active() {
            return;
        }

        let mut printed: Vec<i32> = Vec::new();
        self.collect_printed_steps(&mut printed);
        if printed.is_empty() {
            return;
        }

        let n = printed.len() as i32;
        // a band left empty is fine, the wall planner keeps the layer alive
        let reservable: Vec<i8> = vec![1; n as usize];
        let wall_filament = self.get_wall_filament_for_all_layer();

        // :6658-6669 — only a layer with a tool change or a solid interface
        // can top a group.
        let mut can_merge_top: Vec<i8> = vec![0; n as usize];
        for step in 0..n as usize {
            let layer_id = printed[step] as usize;
            can_merge_top[step] = (!self.plan[layer_id].tool_changes.is_empty()
                || self.wipe_tower_blocks.iter().any(|block| block.layers_type[layer_id] != WipeTowerLayerType::Normal))
                as i8;
        }

        // :6671-6680 — Contact beats Contact_UP beats Solid.
        let type_rank = |t: WipeTowerLayerType| -> i32 {
            match t {
                WipeTowerLayerType::Contact => 3,
                WipeTowerLayerType::ContactUp => 2,
                WipeTowerLayerType::Solid => 1,
                _ => 0,
            }
        };

        for bi in 0..self.wipe_tower_blocks.len() {
            let mut block = std::mem::take(&mut self.wipe_tower_blocks[bi]);
            let mut merged = false;
            {
                let this: &WipeTower = &*self;
                let block_key = block.block_key;
                // :6683-6692 — this band's grid stays tied to its own nozzle.
                let diameter_units = (block_key >> 4) & 0x3F;
                let block_nozzle_diameter = diameter_units as f32 * 0.05;
                let reference_nozzle = if this.tower_grid_reference_nozzle > 0.0 {
                    this.tower_grid_reference_nozzle
                } else {
                    this.tower_grid_nozzle_diameter
                };
                let preferred_block_height = crate::adaptive_layer_merge::adaptive_layer_band_size(
                    block_nozzle_diameter as f64,
                    reference_nozzle as f64,
                    this.tower_grid_layer_height as f64,
                ) as f32
                    * this.tower_grid_layer_height;
                let block_category = WipeTower::decode_block_category(block_key);
                // :6697-6707 — the slab still has to be a height this band's own
                // nozzle can lay down.
                let mut block_height_ceiling = 0.0_f32;
                for id in 0..this.filament_params.len() as i32 {
                    if this.get_filament_category(id as usize) != block_category {
                        continue;
                    }
                    if (this.filament_params[id as usize].nozzle_diameter - block_nozzle_diameter).abs() > 0.026 {
                        continue;
                    }
                    block_height_ceiling = wt_c_max_f32(block_height_ceiling, this.max_layer_height_for_filament(id));
                }
                if block_height_ceiling <= wt_eps {
                    block_height_ceiling = 0.7 * block_nozzle_diameter;
                }
                let mut category_is_tpu = false;
                for id in 0..this.filament_params.len() {
                    if this.get_filament_category(id) == block_category && this.is_tpu_filament(id) {
                        category_is_tpu = true;
                        break;
                    }
                }
                // :6714-6730
                let mut band_has_tpu_ramming = false;
                if category_is_tpu {
                    let mut layer_id = 0usize;
                    while layer_id < this.plan.len() && !band_has_tpu_ramming {
                        for tool_change in &this.plan[layer_id].tool_changes {
                            let old_tool = tool_change.old_tool;
                            if tool_change.nozzle_change_depth as f64 > crate::libslic3r::EPSILON
                                && this.is_need_ramming(old_tool, tool_change.new_tool, layer_id as i32)
                                && this.is_tpu_filament(old_tool)
                                && this.get_block_key(old_tool as i32, layer_id as i32) == block_key
                            {
                                band_has_tpu_ramming = true;
                                break;
                            }
                        }
                        layer_id += 1;
                    }
                }
                // :6734-6737
                let prefer_max_fill = category_is_tpu && ((block_key & 0xF) > 0 || band_has_tpu_ramming);
                let prefer_own_fill = band_has_tpu_ramming;

                // :6745-6751 — plan against the one filament that tops this band up.
                let mut block_visitors: Vec<Vec<i32>> = vec![Vec::new(); n as usize];
                for step in 0..n as usize {
                    let layer_id = printed[step];
                    let filler = this.block_fill_filament_base(&this.plan[layer_id as usize], layer_id, &block, wall_filament);
                    if filler >= 0 {
                        block_visitors[step] = vec![filler];
                    }
                }

                // :6756-6762 — steps the purge plan already decided on.
                let mut fixed: Vec<i8> = vec![0; n as usize];
                for step in 0..n as usize {
                    let layer_id = printed[step] as usize;
                    if block.layer_reserved[layer_id] != 0
                        || this.block_slab_height[layer_id].contains_key(&block_key)
                        || block.layer_depths[layer_id] <= 2.0 * this.perimeter_width
                    {
                        fixed[step] = 1;
                    }
                }

                // :6774-6795 filaments_on_step
                let filaments_on_step = |step: i32, other_category_only: bool| -> Vec<i32> {
                    let mut visitors: Vec<i32> = Vec::new();
                    let mut add = |filament: i32| {
                        if filament < 0 {
                            return;
                        }
                        if other_category_only
                            && this.get_filament_category(filament as usize) == WipeTower::decode_block_category(block_key)
                        {
                            return;
                        }
                        if !visitors.contains(&filament) {
                            visitors.push(filament);
                        }
                    };
                    let layer_id = printed[step as usize];
                    let layer = &this.plan[layer_id as usize];
                    if layer.tool_changes.is_empty() {
                        add(this.loaded_filament(layer_id));
                    } else {
                        add(layer.tool_changes[0].old_tool as i32);
                        for tool_change in &layer.tool_changes {
                            add(tool_change.new_tool as i32);
                        }
                    }
                    visitors
                };
                // :6796-6846 apply_fill_groups
                let apply_fill_groups = |block: &mut WipeTowerBlock,
                                         band_top_z: &mut f64,
                                         merged: &mut bool,
                                         groups: &[MergeLadderGroup],
                                         foreign_support: bool,
                                         keep_sparse: bool| {
                    let mut surface = *band_top_z;
                    for group in groups {
                        let top_layer = printed[group.top_step as usize];
                        let mut height = group.height;
                        // :6802-6812 — the support nozzle may already be extruding
                        // a thicker slab of its own on this layer.
                        if foreign_support
                            && group.filament_id >= 0
                            && height + wt_eps < this.preferred_height_for_filament(group.filament_id)
                        {
                            let carrier = this.slab_height_for(group.filament_id, top_layer, height);
                            let bottom = this.plan[top_layer as usize].z - carrier;
                            if carrier > height + wt_eps
                                && bottom as f64 + wt_eps as f64 >= surface
                                && (block_height_ceiling <= wt_eps || carrier <= block_height_ceiling + wt_eps)
                            {
                                height = carrier;
                            }
                        }
                        let bottom_z = this.plan[top_layer as usize].z - height;
                        for step in 0..n as usize {
                            let layer_id = printed[step];
                            let z = this.plan[layer_id as usize].z;
                            if z <= bottom_z + wt_eps {
                                continue;
                            }
                            if z > this.plan[top_layer as usize].z + wt_eps {
                                break;
                            }
                            if layer_id == top_layer {
                                continue;
                            }
                            if (layer_id as usize) < this.block_slab_height.len()
                                && this.block_slab_height[layer_id as usize].contains_key(&block.block_key)
                            {
                                continue;
                            }
                            block.layer_reserved[layer_id as usize] = 1;
                            // :6829-6833 — a same-material support run stays sparse.
                            if !keep_sparse
                                && type_rank(block.layers_type[layer_id as usize]) > type_rank(block.layers_type[top_layer as usize])
                            {
                                block.layers_type[top_layer as usize] = block.layers_type[layer_id as usize];
                            }
                        }
                        if keep_sparse {
                            block.layers_type[top_layer as usize] = WipeTowerLayerType::Normal;
                        }
                        // :6837-6839
                        if (top_layer as usize) < block.layer_fill_filament.len() && group.filament_id >= 0 {
                            block.layer_fill_filament[top_layer as usize] = group.filament_id;
                        }
                        block.layer_heights[top_layer as usize] = height;
                        if height > this.plan[top_layer as usize].height + wt_eps {
                            *merged = true;
                        }
                        surface = this.plan[top_layer as usize].z as f64;
                    }
                    if !groups.is_empty() {
                        *band_top_z = surface;
                    }
                };
                // :6847-6886 plan_fill_span
                let plan_fill_span = |block: &mut WipeTowerBlock,
                                      band_top_z: &mut f64,
                                      merged: &mut bool,
                                      begin: i32,
                                      end: i32,
                                      visitors: &[Vec<i32>],
                                      preferred: f32,
                                      foreign_support: bool| {
                    let mut end = end;
                    while end >= begin && (end >= n || visitors[end as usize].is_empty()) {
                        end -= 1;
                    }
                    if begin > end {
                        return;
                    }
                    let mut groups: Vec<MergeLadderGroup> = Vec::new();
                    let capped_preferred =
                        if block_height_ceiling > wt_eps { wt_c_min_f32(preferred, block_height_ceiling) } else { preferred };
                    if this.plan_band_merge_ladder(
                        &printed,
                        begin,
                        end,
                        *band_top_z,
                        visitors,
                        &reservable,
                        &can_merge_top,
                        &mut groups,
                        capped_preferred,
                        prefer_max_fill,
                        block_height_ceiling,
                    ) {
                        apply_fill_groups(block, band_top_z, merged, &groups, foreign_support, false);
                    } else {
                        if crate::probe_enabled("WT_LADDER_LOG") {
                            eprintln!(
                                "WipeTower: cannot merge the fill of band {} over printed layers {}..{}; keeping their base layer heights",
                                block.block_key, printed[begin as usize], printed[end as usize]
                            );
                        }
                        *band_top_z = this.plan[printed[end as usize] as usize].z as f64;
                    }
                };
                // :6894-6900 filaments_of
                let filaments_of = |step: i32, category: i32| -> Vec<i32> {
                    let mut out: Vec<i32> = Vec::new();
                    for filament in filaments_on_step(step, false) {
                        if this.get_filament_category(filament as usize) == category {
                            out.push(filament);
                        }
                    }
                    out
                };
                // :6901-6910 mark_fill_interface
                let mark_fill_interface =
                    |block: &mut WipeTowerBlock, groups: &[MergeLadderGroup], category: i32, prev_category: i32, meets_band: bool| {
                        if groups.is_empty() {
                            return;
                        }
                        let first_layer = printed[groups[0].top_step as usize];
                        let last_layer = printed[groups[groups.len() - 1].top_step as usize];
                        if prev_category >= 0 && prev_category != category {
                            block.layers_type[first_layer as usize] = WipeTowerLayerType::Contact;
                        }
                        if meets_band && category != block_category {
                            block.layers_type[last_layer as usize] = WipeTowerLayerType::Contact;
                        }
                    };

                // :6766 — the top surface of this band.
                let mut band_top_z = this.tower_bed_z();
                let mut run_begin = 0i32;
                while run_begin < n {
                    if fixed[run_begin as usize] != 0 {
                        let layer_id = printed[run_begin as usize];
                        // :6770-6773 — a purge brings the band's surface to its Z;
                        // a reserved trench does not.
                        if this.block_slab_height[layer_id as usize].contains_key(&block_key) {
                            band_top_z = this.plan[layer_id as usize].z as f64;
                        }
                        run_begin += 1;
                        continue;
                    }
                    // :6776-6782 — a group spans layers of one depth.
                    let run_depth = block.layer_depths[printed[run_begin as usize] as usize];
                    let mut run_end = run_begin;
                    while run_end + 1 < n
                        && fixed[(run_end + 1) as usize] == 0
                        && (block.layer_depths[printed[(run_end + 1) as usize] as usize] - run_depth).abs() <= wt_eps
                    {
                        run_end += 1;
                    }
                    let next_run = run_end + 1;
                    if prefer_max_fill {
                        // :6912-6923
                        let mut prev_category = -1i32;
                        if run_begin > 0 {
                            let mut step = run_begin - 1;
                            while step >= 0 && fixed[step as usize] != 0 {
                                if this.block_slab_height[printed[step as usize] as usize].contains_key(&block_key) {
                                    prev_category = block_category;
                                    break;
                                }
                                step -= 1;
                            }
                        }
                        let meets_band = next_run < n
                            && fixed[next_run as usize] != 0
                            && (block.layer_reserved[printed[next_run as usize] as usize] != 0
                                || this.block_slab_height[printed[next_run as usize] as usize].contains_key(&block_key));
                        let mut cursor = run_begin;
                        while cursor <= run_end {
                            let mut categories: Vec<i32> = Vec::new();
                            for step in cursor..=run_end {
                                for filament in filaments_on_step(step, false) {
                                    let category = this.get_filament_category(filament as usize);
                                    if !categories.contains(&category) {
                                        categories.push(category);
                                    }
                                }
                            }
                            let mut best_cat = -1i32;
                            let mut best_end = cursor - 1;
                            let mut best_max_h = -1.0_f32;
                            let mut own_end = cursor - 1;
                            for &category in &categories {
                                let mut cat_max_h = 0.0_f32;
                                for step in cursor..=run_end {
                                    for filament in filaments_of(step, category) {
                                        cat_max_h = wt_c_max_f32(cat_max_h, this.max_layer_height_for_filament(filament));
                                    }
                                }
                                if block_height_ceiling > wt_eps {
                                    cat_max_h = wt_c_min_f32(cat_max_h, block_height_ceiling);
                                }
                                let mut surface = band_top_z;
                                let mut next = cursor;
                                let mut end = cursor - 1;
                                while next <= run_end {
                                    let mut farthest = -1i32;
                                    for step in next..=run_end {
                                        let fils = filaments_of(step, category);
                                        if fils.is_empty() {
                                            continue;
                                        }
                                        let height = this.plan[printed[step as usize] as usize].z as f64 - surface;
                                        if height > cat_max_h as f64 + wt_eps as f64 {
                                            break;
                                        }
                                        let mut swallows = false;
                                        for below in next..step {
                                            if this.plan[printed[below as usize] as usize].z as f64 > surface + wt_eps as f64 {
                                                swallows = true;
                                            }
                                        }
                                        if swallows && can_merge_top[step as usize] == 0 {
                                            continue;
                                        }
                                        let can_print = fils.iter().any(|&filament| this.filament_can_print_height(filament, height as f32));
                                        if !can_print {
                                            continue;
                                        }
                                        farthest = step;
                                    }
                                    if farthest < 0 {
                                        break;
                                    }
                                    end = farthest;
                                    surface = this.plan[printed[farthest as usize] as usize].z as f64;
                                    next = farthest + 1;
                                }
                                if category == block_category && end >= cursor {
                                    own_end = end;
                                }
                                if end > best_end || (end == best_end && cat_max_h > best_max_h + wt_eps) {
                                    best_end = end;
                                    best_cat = category;
                                    best_max_h = cat_max_h;
                                }
                            }
                            // :6970-6975 — under the TPU ramming strip take that
                            // category for the whole reach of its nozzle.
                            if prefer_own_fill && own_end >= cursor {
                                best_cat = block_category;
                                best_end = own_end;
                            }
                            if best_cat < 0 || best_end < cursor {
                                let mut foreign: Vec<Vec<i32>> = vec![Vec::new(); n as usize];
                                foreign[cursor as usize] = filaments_on_step(cursor, false);
                                plan_fill_span(&mut block, &mut band_top_z, &mut merged, cursor, cursor, &foreign, 0.0, true);
                                cursor += 1;
                                prev_category = -1;
                                continue;
                            }
                            let mut visitors: Vec<Vec<i32>> = vec![Vec::new(); n as usize];
                            for step in cursor..=best_end {
                                visitors[step as usize] = filaments_of(step, best_cat);
                            }
                            let mut groups: Vec<MergeLadderGroup> = Vec::new();
                            if this.plan_band_merge_ladder(
                                &printed,
                                cursor,
                                best_end,
                                band_top_z,
                                &visitors,
                                &reservable,
                                &can_merge_top,
                                &mut groups,
                                0.0,
                                true,
                                block_height_ceiling,
                            ) {
                                apply_fill_groups(&mut block, &mut band_top_z, &mut merged, &groups, true, true);
                                mark_fill_interface(&mut block, &groups, best_cat, prev_category, best_end == run_end && meets_band);
                            } else {
                                if crate::probe_enabled("WT_LADDER_LOG") {
                                    eprintln!(
                                        "WipeTower: cannot merge the fill of band {} over printed layers {}..{}; keeping their base layer heights",
                                        block_key, printed[cursor as usize], printed[best_end as usize]
                                    );
                                }
                                band_top_z = this.plan[printed[best_end as usize] as usize].z as f64;
                            }
                            prev_category = best_cat;
                            cursor = best_end + 1;
                        }
                    } else {
                        // :7004-7019
                        let mut cursor = run_begin;
                        while cursor <= run_end {
                            let own = !block_visitors[cursor as usize].is_empty();
                            let mut seg_end = cursor;
                            while seg_end + 1 <= run_end && block_visitors[(seg_end + 1) as usize].is_empty() == !own {
                                seg_end += 1;
                            }
                            if own {
                                plan_fill_span(
                                    &mut block,
                                    &mut band_top_z,
                                    &mut merged,
                                    cursor,
                                    seg_end,
                                    &block_visitors,
                                    preferred_block_height,
                                    false,
                                );
                            } else {
                                let mut foreign: Vec<Vec<i32>> = vec![Vec::new(); n as usize];
                                for step in cursor..=seg_end {
                                    foreign[step as usize] = filaments_on_step(step, true);
                                }
                                plan_fill_span(&mut block, &mut band_top_z, &mut merged, cursor, seg_end, &foreign, 0.0, true);
                            }
                            cursor = seg_end + 1;
                        }
                    }
                    run_begin = next_run;
                }
            }
            self.wipe_tower_blocks[bi] = block;
            if merged {
                self.merged_slabs_active = true;
            }
        }

        // :7024-7090 — each band's material interfaces are Contact on both sides.
        for bi in 0..self.wipe_tower_blocks.len() {
            let mut block = std::mem::take(&mut self.wipe_tower_blocks[bi]);
            {
                let this: &WipeTower = &*self;
                let mut slabs: Vec<WtPrintedSlab> = Vec::new();
                for layer_id in 0..this.plan.len() as i32 {
                    if this.layer_is_skipped(layer_id) {
                        continue;
                    }
                    if (layer_id as usize) < block.layer_reserved.len() && block.layer_reserved[layer_id as usize] != 0 {
                        continue;
                    }
                    if this.band_fill_depth(&block, layer_id) <= 2.0 * this.perimeter_width {
                        continue;
                    }
                    let filament = if (layer_id as usize) < block.layer_fill_filament.len()
                        && block.layer_fill_filament[layer_id as usize] >= 0
                    {
                        block.layer_fill_filament[layer_id as usize]
                    } else {
                        this.block_fill_filament_base(&this.plan[layer_id as usize], layer_id, &block, wall_filament)
                    };
                    if filament < 0 {
                        continue;
                    }
                    let height = if (layer_id as usize) < block.layer_heights.len() && block.layer_heights[layer_id as usize] > wt_eps {
                        block.layer_heights[layer_id as usize]
                    } else {
                        this.plan[layer_id as usize].height
                    };
                    if height <= wt_eps {
                        continue;
                    }
                    let top_z = this.plan[layer_id as usize].z;
                    slabs.push(WtPrintedSlab { layer_id, filament, top_z, bottom_z: top_z - height });
                }
                for upper in &slabs {
                    let mut support: Option<usize> = None;
                    for (li, lower) in slabs.iter().enumerate() {
                        if lower.layer_id >= upper.layer_id || lower.top_z > upper.bottom_z + wt_eps {
                            continue;
                        }
                        if support.map_or(true, |s| lower.top_z > slabs[s].top_z) {
                            support = Some(li);
                        }
                    }
                    let Some(s) = support else {
                        continue;
                    };
                    let support = slabs[s];
                    if support.top_z + wt_eps < upper.bottom_z {
                        continue;
                    }
                    if this.get_filament_category(support.filament as usize) == this.get_filament_category(upper.filament as usize) {
                        continue;
                    }
                    block.layers_type[support.layer_id as usize] = WipeTowerLayerType::Contact;
                    block.layers_type[upper.layer_id as usize] = WipeTowerLayerType::Contact;
                }
            }
            self.wipe_tower_blocks[bi] = block;
        }
    }

    /// WipeTower.cpp:7093-7187 `plan_merged_outer_wall`: the outer wall gets
    /// its own Z ladder over every layer the tower prints; a group draws one
    /// wall at its top layer and the layers below it omit theirs. Fills
    /// `wall_filament`, `wall_slab_height` and `wall_reserved`.
    fn plan_merged_outer_wall(&mut self) {
        let layer_count = self.plan.len();
        self.wall_filament = vec![-1; layer_count];
        self.wall_slab_height = vec![0.0; layer_count];
        self.wall_reserved = vec![0; layer_count];

        if !self.merged_fill_active() {
            return;
        }

        let mut printed: Vec<i32> = Vec::new();
        self.collect_printed_steps(&mut printed);
        if printed.is_empty() {
            return;
        }
        let n = printed.len();

        // :7112-7143 — the wall keeps to the material of the tower's wall
        // filament on a layer any filament of that category visits.
        let wall_filament = self.get_wall_filament_for_all_layer();
        let wall_category = if wall_filament >= 0 { self.get_filament_category(wall_filament as usize) } else { -1 };
        let mut initial = -1i32;
        for info in &self.plan {
            if !info.tool_changes.is_empty() {
                initial = info.tool_changes[0].old_tool as i32;
                break;
            }
        }
        let mut visitors: Vec<Vec<i32>> = vec![Vec::new(); n];
        for step in 0..n {
            let layer = &self.plan[printed[step] as usize];
            let mut arrivals: Vec<i32> = Vec::new();
            let mut arrive = |filament: i32| {
                if filament >= 0 && !arrivals.contains(&filament) {
                    arrivals.push(filament);
                }
            };
            if layer.tool_changes.is_empty() {
                let loaded = self.loaded_filament(printed[step]);
                arrive(if loaded >= 0 { loaded } else { initial });
            } else {
                arrive(layer.tool_changes[0].old_tool as i32);
                for tool_change in &layer.tool_changes {
                    arrive(tool_change.new_tool as i32);
                }
            }
            let mut own_material: Vec<i32> = Vec::new();
            if arrivals.contains(&wall_filament) {
                own_material.push(wall_filament);
            }
            for &filament in &arrivals {
                if filament != wall_filament && self.get_filament_category(filament as usize) == wall_category {
                    own_material.push(filament);
                }
            }
            visitors[step] = if own_material.is_empty() { arrivals } else { own_material };
        }

        // :7150-7155 — every printed layer may hand its wall to the group
        // above, and any of them can top a group.
        let reservable: Vec<i8> = vec![1; n];
        let can_merge_top: Vec<i8> = vec![1; n];
        let bottom_z = self.tower_bed_z();
        let mut groups: Vec<MergeLadderGroup> = Vec::new();
        if !self.plan_band_merge_ladder(
            &printed,
            0,
            n as i32 - 1,
            bottom_z,
            &visitors,
            &reservable,
            &can_merge_top,
            &mut groups,
            0.0,
            false,
            0.0,
        ) {
            if crate::probe_enabled("WT_LADDER_LOG") {
                eprintln!(
                    "WipeTower: cannot lay out a wall ladder over the printed tower layers {}..{}; keeping the existing per-layer wall plan",
                    printed[0],
                    printed[n - 1]
                );
            }
            return;
        }

        for group in &groups {
            for step in group.begin_step..group.top_step {
                self.wall_reserved[printed[step as usize] as usize] = 1;
            }
            self.wall_filament[printed[group.top_step as usize] as usize] = group.filament_id;
            self.wall_slab_height[printed[group.top_step as usize] as usize] = group.height;
            if group.begin_step < group.top_step {
                self.merged_slabs_active = true;
            }
        }
    }

    /// WipeTower.cpp:7189-7254 `audit_merged_slabs`: reports every band or
    /// wall slab that claims Z a lower layer already filled. Diagnostics only:
    /// the report is printed with the probe `WT_AUDIT`.
    fn audit_merged_slabs(&self) {
        if self.plan.is_empty() || !self.merged_slabs_active {
            return;
        }
        let wt_eps = WIPE_TOWER_WT_EPSILON;
        let report = crate::probe_enabled("WT_AUDIT");

        // :7197-7206 — mirrors the fill condition of finish_layer_new().
        let band_prints = |block: &WipeTowerBlock, layer_id: i32| -> bool {
            if self.layer_is_skipped(layer_id) {
                return false;
            }
            if (layer_id as usize) < self.block_slab_height.len() && self.block_slab_height[layer_id as usize].contains_key(&block.block_key)
            {
                return true;
            }
            (layer_id as usize) < block.layer_reserved.len()
                && block.layer_reserved[layer_id as usize] == 0
                && self.band_fill_depth(block, layer_id) > 2.0 * self.perimeter_width
        };

        let mut overlaps = 0i32;
        for block in &self.wipe_tower_blocks {
            let mut prev_top = 0.0_f32;
            let mut prev_layer = -1i32;
            for layer_id in 0..self.plan.len() as i32 {
                if !band_prints(block, layer_id) {
                    continue;
                }
                let top = self.plan[layer_id as usize].z;
                let height = if (layer_id as usize) < block.layer_heights.len() && block.layer_heights[layer_id as usize] > 0.0 {
                    block.layer_heights[layer_id as usize]
                } else {
                    self.plan[layer_id as usize].height
                };
                if prev_layer >= 0 && top - height + wt_eps < prev_top {
                    overlaps += 1;
                    if report {
                        eprintln!(
                            "[wt_audit] band {} (cat={} lane={} y={}) fills ({}..{}] at layer {} h={}, but layer {} already reached {}; overlap {} mm",
                            block.block_key,
                            WipeTower::decode_block_category(block.block_key),
                            block.block_key & 0xF,
                            block.start_depth,
                            top - height,
                            top,
                            layer_id,
                            height,
                            prev_layer,
                            prev_top,
                            prev_top - (top - height)
                        );
                    }
                }
                prev_top = top;
                prev_layer = layer_id;
            }
        }

        let mut prev_wall_top = 0.0_f32;
        let mut prev_wall_layer = -1i32;
        for layer_id in 0..self.plan.len() as i32 {
            if self.layer_is_skipped(layer_id)
                || ((layer_id as usize) < self.wall_reserved.len() && self.wall_reserved[layer_id as usize] != 0)
            {
                continue;
            }
            let top = self.plan[layer_id as usize].z;
            let height = if (layer_id as usize) < self.wall_slab_height.len() && self.wall_slab_height[layer_id as usize] > 0.0 {
                self.wall_slab_height[layer_id as usize]
            } else {
                self.layer_slab_height(layer_id, self.plan[layer_id as usize].height)
            };
            if prev_wall_layer >= 0 && top - height + wt_eps < prev_wall_top {
                overlaps += 1;
                if report {
                    eprintln!(
                        "[wt_audit] outer wall draws ({}..{}] at layer {} h={}, but layer {} already reached {}; overlap {} mm",
                        top - height,
                        top,
                        layer_id,
                        height,
                        prev_wall_layer,
                        prev_wall_top,
                        prev_wall_top - (top - height)
                    );
                }
            }
            prev_wall_top = top;
            prev_wall_layer = layer_id;
        }

        if report {
            if overlaps != 0 {
                eprintln!(
                    "[wt_audit] wipe tower slab plan has {} overlapping slab(s) over {} band(s)",
                    overlaps,
                    self.wipe_tower_blocks.len()
                );
            } else {
                eprintln!(
                    "[wt_audit] wipe tower slab plan is free of overlapping slabs over {} band(s)",
                    self.wipe_tower_blocks.len()
                );
            }
        }
    }
}

// ============================================================================
// The C++ generator: planning, blocks and generate_new (WipeTower.cpp:3085-3541, 5517-5831, 7256-8111)
// ============================================================================
/// `std::unordered_map<int, V>` of libc++ as far as the order of iteration
/// goes (`__hash_table`): one singly linked list of nodes, each bucket points
/// at the node before its first one, `std::hash<int>` is the value itself,
/// the maximum load factor is 1. A unique insertion (`operator[]`, `insert`)
/// puts the node first in its bucket, a multi insertion (what the copy
/// assignment does, `__assign_multi`) last in its bucket. Used where C++ sums
/// floats in the iteration order of such a map (`calc_block_infill_gap`).
#[derive(Debug, Clone)]
struct LibcxxHashMapI32<V> {
    /// node 0 is the node before the first (`__p1_`); its key is not read
    keys: Vec<i32>,
    vals: Vec<V>,
    next: Vec<usize>,
    /// `__bucket_list_`: for each bucket the node before its first node
    buckets: Vec<usize>,
    size: usize,
}

const LIBCXX_NIL: usize = usize::MAX;

impl<V: Clone + Default> LibcxxHashMapI32<V> {
    fn new() -> Self {
        Self { keys: vec![0], vals: vec![V::default()], next: vec![LIBCXX_NIL], buckets: Vec::new(), size: 0 }
    }

    /// `std::hash<int>`: `static_cast<size_t>(v)`.
    fn hash(k: i32) -> usize {
        k as isize as usize
    }

    /// `__constrain_hash`
    fn constrain_hash(h: usize, bc: usize) -> usize {
        if bc & (bc - 1) == 0 {
            h & (bc - 1)
        } else if h < bc {
            h
        } else {
            h % bc
        }
    }

    /// `__next_prime`: the smallest prime not below `n`.
    fn next_prime(n: usize) -> usize {
        let mut p = n.max(2);
        loop {
            let mut is_prime = true;
            let mut d = 2usize;
            while d * d <= p {
                if p % d == 0 {
                    is_prime = false;
                    break;
                }
                d += 1;
            }
            if is_prime {
                return p;
            }
            p += 1;
        }
    }

    /// `__rehash(n)`, the growing branch (the only one an insertion takes).
    fn rehash(&mut self, mut n: usize) {
        if n == 1 {
            n = 2;
        } else if n & (n - 1) != 0 {
            n = Self::next_prime(n);
        }
        if n > self.buckets.len() {
            self.do_rehash(n);
        }
    }

    /// `__do_rehash(nbc)` (the keys are unique, so the multi variant moves
    /// single nodes as the unique one does).
    fn do_rehash(&mut self, nbc: usize) {
        self.buckets = vec![LIBCXX_NIL; nbc];
        let mut pp = 0usize;
        let mut cp = self.next[pp];
        if cp != LIBCXX_NIL {
            let mut chash = Self::constrain_hash(Self::hash(self.keys[cp]), nbc);
            self.buckets[chash] = pp;
            let mut phash = chash;
            pp = cp;
            cp = self.next[cp];
            while cp != LIBCXX_NIL {
                chash = Self::constrain_hash(Self::hash(self.keys[cp]), nbc);
                if chash == phash {
                    pp = cp;
                } else if self.buckets[chash] == LIBCXX_NIL {
                    self.buckets[chash] = pp;
                    pp = cp;
                    phash = chash;
                } else {
                    let np = cp;
                    self.next[pp] = self.next[np];
                    let b = self.buckets[chash];
                    self.next[np] = self.next[b];
                    self.next[b] = cp;
                }
                cp = self.next[pp];
            }
        }
    }

    /// `if (size() + 1 > bc * max_load_factor() || bc == 0) rehash(...)`.
    fn grow_for_one_more(&mut self) {
        let bc = self.buckets.len();
        if (self.size + 1) as f32 > bc as f32 * 1.0f32 || bc == 0 {
            let is_hash_power2 = bc > 2 && (bc & (bc - 1)) == 0;
            let a = 2 * bc + (!is_hash_power2) as usize;
            let b = ((self.size + 1) as f32 / 1.0f32).ceil() as usize;
            self.rehash(a.max(b));
        }
    }

    /// `find(k)`: the node of the key, or `LIBCXX_NIL`.
    fn find_node(&self, k: i32) -> usize {
        let bc = self.buckets.len();
        if bc != 0 {
            let chash = Self::constrain_hash(Self::hash(k), bc);
            let mut nd = self.buckets[chash];
            if nd != LIBCXX_NIL {
                nd = self.next[nd];
                while nd != LIBCXX_NIL
                    && (self.keys[nd] == k || Self::constrain_hash(Self::hash(self.keys[nd]), bc) == chash)
                {
                    if self.keys[nd] == k {
                        return nd;
                    }
                    nd = self.next[nd];
                }
            }
        }
        LIBCXX_NIL
    }

    /// `count(k)`
    fn count(&self, k: i32) -> usize {
        (self.find_node(k) != LIBCXX_NIL) as usize
    }

    /// A new node of the key and value, not linked yet.
    fn new_node(&mut self, k: i32, v: V) -> usize {
        let nd = self.keys.len();
        self.keys.push(k);
        self.vals.push(v);
        self.next.push(LIBCXX_NIL);
        nd
    }

    /// `operator[](k)` / `insert({k, V{}})`: the value of the key, the node
    /// put first in its bucket when it is new.
    fn entry(&mut self, k: i32) -> &mut V {
        let found = self.find_node(k);
        if found != LIBCXX_NIL {
            return &mut self.vals[found];
        }
        self.grow_for_one_more();
        let bc = self.buckets.len();
        let chash = Self::constrain_hash(Self::hash(k), bc);
        let nd = self.new_node(k, V::default());
        let pn = self.buckets[chash];
        if pn == LIBCXX_NIL {
            self.next[nd] = self.next[0];
            self.next[0] = nd;
            self.buckets[chash] = 0;
            if self.next[nd] != LIBCXX_NIL {
                let nh = Self::constrain_hash(Self::hash(self.keys[self.next[nd]]), bc);
                self.buckets[nh] = nd;
            }
        } else {
            self.next[nd] = self.next[pn];
            self.next[pn] = nd;
        }
        self.size += 1;
        &mut self.vals[nd]
    }

    /// `__node_insert_multi(nd)` for a key not in the map: the node goes last
    /// in its bucket.
    fn insert_multi(&mut self, k: i32, v: V) {
        self.grow_for_one_more();
        let bc = self.buckets.len();
        let chash = Self::constrain_hash(Self::hash(k), bc);
        // __node_insert_multi_prepare: walk to the end of the bucket's run.
        let mut pn = self.buckets[chash];
        if pn != LIBCXX_NIL {
            let mut found = false;
            while self.next[pn] != LIBCXX_NIL
                && Self::constrain_hash(Self::hash(self.keys[self.next[pn]]), bc) == chash
            {
                let eq = self.keys[self.next[pn]] == k;
                if found != eq {
                    if !found {
                        found = true;
                    } else {
                        break;
                    }
                }
                pn = self.next[pn];
            }
        }
        let nd = self.new_node(k, v);
        // __node_insert_multi_perform
        if pn == LIBCXX_NIL {
            self.next[nd] = self.next[0];
            self.next[0] = nd;
            self.buckets[chash] = 0;
            if self.next[nd] != LIBCXX_NIL {
                let nh = Self::constrain_hash(Self::hash(self.keys[self.next[nd]]), bc);
                self.buckets[nh] = nd;
            }
        } else {
            self.next[nd] = self.next[pn];
            self.next[pn] = nd;
            if self.next[nd] != LIBCXX_NIL {
                let nh = Self::constrain_hash(Self::hash(self.keys[self.next[nd]]), bc);
                if nh != chash {
                    self.buckets[nh] = nd;
                }
            }
        }
        self.size += 1;
    }

    /// The copy assignment `*this = src` (`__assign_multi`): the bucket count
    /// is kept, the nodes are emptied and the source inserted in its order.
    fn assign_from(&mut self, src: &Self) {
        let items = src.items();
        for b in self.buckets.iter_mut() {
            *b = LIBCXX_NIL;
        }
        self.keys.truncate(1);
        self.vals.truncate(1);
        self.next.truncate(1);
        self.next[0] = LIBCXX_NIL;
        self.size = 0;
        for (k, v) in items {
            self.insert_multi(k, v);
        }
    }

    /// The (key, value) pairs in iteration order.
    fn items(&self) -> Vec<(i32, V)> {
        let mut out = Vec::with_capacity(self.size);
        let mut nd = self.next[0];
        while nd != LIBCXX_NIL {
            out.push((self.keys[nd], self.vals[nd].clone()));
            nd = self.next[nd];
        }
        out
    }

    /// The node indices in iteration order.
    fn nodes_in_order(&self) -> Vec<usize> {
        let mut out = Vec::with_capacity(self.size);
        let mut nd = self.next[0];
        while nd != LIBCXX_NIL {
            out.push(nd);
            nd = self.next[nd];
        }
        out
    }
}

/// WipeTower.cpp:5739-5747 `struct BlockInfo` of `calc_block_infill_gap`.
#[derive(Debug, Clone, Copy, Default)]
struct CalcGapBlockInfo {
    has_ramming: bool,
    has_reverse_travel: bool,
    depth: f32,
    perimeter_width: f32,
    nozzle_change_perimeter_width: f32,
}

/// libslic3r.h:288-291 `is_approx<float>(value, test_value, precision =
/// EPSILON)`: the precision is the float `EPSILON`.
#[inline]
fn is_approx_float(value: f32, test_value: f32) -> bool {
    ((value as f64) - (test_value as f64)).abs() < (crate::libslic3r::EPSILON as f32) as f64
}

impl WipeTower {
    /// WipeTower.cpp:3085-3117 `set_toolchange`: the tool change of a planned
    /// layer with its depth recomputed from the slab heights and gap widths.
    fn set_toolchange(&mut self, old_tool: i32, new_tool: i32, layer_height: f32, wipe_volume: f32, purge_volume: f32, layer_id: i32) -> ToolChangeInfo {
        let new_tool_width = self.filament_perimeter_width(new_tool);
        let old_tool_nc_width = self.filament_nozzle_change_width(old_tool);
        // The wipe and the ramming live in different bands and may therefore sit on slabs of different heights.
        let wipe_height = self.slab_height_for(new_tool, layer_id, layer_height);
        let ramming_height = self.slab_height_for(old_tool, layer_id, layer_height);
        let mut depth = 0.0_f32;
        let width = self.wipe_tower_width - 2.0 * new_tool_width;
        let nozzle_change_width = self.wipe_tower_width - (old_tool_nc_width + new_tool_width);
        let length_to_extrude = self.volume_to_length(wipe_volume, new_tool_width, wipe_height);
        let toolchange_gap_width = self.get_block_gap_width(new_tool, false);
        let nozzlechange_gap_width = self.get_block_gap_width(old_tool, true);
        let filament_change_length: f32 = if !self.is_same_extruder(old_tool as usize, new_tool as usize, layer_id) {
            self.filaments_change_length.0[old_tool as usize] as f32
        } else {
            self.filaments_change_length.1[old_tool as usize] as f32
        };
        // :3099 — depth += ceil(...) * gap, contracted.
        depth = (length_to_extrude / width).ceil().mul_add(toolchange_gap_width, depth);

        let mut nozzle_change_depth = 0.0_f32;
        let mut nozzle_change_length = 0.0_f32;
        if self.is_need_ramming(old_tool as usize, new_tool as usize, layer_id) {
            // :3105 — float arithmetic (the inner a - b*c contracted), then to double.
            let k = 1.0_f32 - (std::f64::consts::PI as f32) / 4.0;
            let e_flow = (ramming_height * (-ramming_height).mul_add(k, old_tool_nc_width)
                / self.filament_params[old_tool as usize].filament_area) as f64;
            let length = filament_change_length as f64 / e_flow;
            let nozzle_change_line_count = (length / nozzle_change_width as f64).ceil() as i32;
            nozzle_change_depth = nozzle_change_line_count as f32 * nozzlechange_gap_width;
            depth += nozzle_change_depth;
            nozzle_change_length = length as f32;
        }
        let mut tool_change = ToolChangeInfo::new(old_tool as usize, new_tool as usize);
        tool_change.required_depth = depth;
        tool_change.ramming_depth = 0.0;
        tool_change.first_wipe_line = 0.0;
        tool_change.wipe_volume = wipe_volume;
        tool_change.wipe_length = length_to_extrude;
        tool_change.purge_volume = purge_volume;
        tool_change.nozzle_change_depth = nozzle_change_depth;
        tool_change.nozzle_change_length = nozzle_change_length;
        tool_change
    }


    /// WipeTower.cpp:3294-3322 `save_on_last_wipe`. Compiled out (`#if 0`) at
    /// the pin; kept as the C++ text reads.
    #[allow(dead_code)]
    fn save_on_last_wipe(&mut self) {
        self.layer_idx = 0;
        while self.layer_idx < self.plan.len() {
            let li = self.layer_idx;
            let (z, h) = (self.plan[li].z, self.plan[li].height);
            let first_z = self.plan[0].z;
            let last_z = self.plan.last().unwrap().z;
            self.set_layer(z, h, 0, z == first_z, z == last_z);
            // set_layer may move m_layer_info only forwards to the same layer.
            let li = self.layer_idx;
            if self.plan[li].tool_changes.is_empty() {
                self.layer_idx += 1;
                continue;
            }
            let idx = self.first_toolchange_to_nonsoluble_nonsupport(&self.plan[li].tool_changes.clone());
            for i in 0..self.plan[li].tool_changes.len() as i32 {
                let new_tool = self.plan[li].tool_changes[i as usize].new_tool;
                self.tool_change(new_tool, false, false);
                if i == idx {
                    let width = self.wipe_tower_width - 3.0 * self.perimeter_width;
                    let length_to_save = self.finish_layer(true, true).total_extrusion_length_in_plane();
                    let tc = self.plan[li].tool_changes[i as usize].clone();
                    let mut length_to_wipe = self.volume_to_length(tc.wipe_volume, self.perimeter_width, self.plan[li].height)
                        - tc.first_wipe_line
                        - length_to_save;
                    length_to_wipe = length_to_wipe.max(0.0);
                    let depth_to_wipe = self.perimeter_width
                        * ((length_to_wipe / width).floor() + if length_to_wipe > 0.0 { 1.0 } else { 0.0 })
                        * self.extra_spacing;
                    self.plan[li].tool_changes[i as usize].required_depth = tc.ramming_depth + depth_to_wipe;
                }
            }
            self.layer_idx += 1;
        }
    }

    /// WipeTower.cpp:3353-3427 `merge_tcr`: `second` appended to `first`, with
    /// a travel between them unless the G-code side makes it.
    fn merge_tcr(&mut self, first: &mut ToolChangeResult, second: &mut ToolChangeResult) -> ToolChangeResult {
        if first.new_tool != second.initial_tool {
            // :3356-3363 — C++ logs and throws Slic3r::InvalidArgument.
            panic!(
                "WipeTower: cannot merge discontinuous tool-change results, first={}->{}, second={}->{}, layer={}",
                first.initial_tool, first.new_tool, second.initial_tool, second.new_tool, self.cur_layer_id
            );
        }
        let mut out = first.clone();
        if (first.end_pos - second.start_pos).norm() > crate::libslic3r::EPSILON as f32 {
            let travel_gcode = format!(
                "G1 X{} Y{} F{}\n",
                float_to_string_decimal_point(second.start_pos.x as f64, 3),
                float_to_string_decimal_point(second.start_pos.y as f64, 3),
                float_to_string_decimal_point(self.max_speed as f64, 0)
            );
            let mut need_insert_travel = true;
            if second.is_tool_change
                && is_approx_float(second.start_pos.x, second.tool_change_start_pos.x)
                && is_approx_float(second.start_pos.y, second.tool_change_start_pos.y)
            {
                // will insert travel in gcode.cpp
                need_insert_travel = false;
            }
            if need_insert_travel {
                out.gcode.push_str(&travel_gcode);
            }
        }
        out.gcode.push_str(&second.gcode);
        out.extrusions.extend(second.extrusions.iter().cloned());
        out.end_pos = second.end_pos;
        out.wipe_path = second.wipe_path.clone();
        out.initial_tool = first.initial_tool;
        out.new_tool = second.new_tool;
        out.is_contact = first.is_contact || second.is_contact;
        out.side_scrape = first.side_scrape || second.side_scrape;
        if !first.side_scrape && second.side_scrape {
            out.side_scrape_z_low = second.side_scrape_z_low;
            out.side_scrape_depth = second.side_scrape_depth;
            out.side_scrape_min_height_above_base = second.side_scrape_min_height_above_base;
        }
        let has_nozzle_change_action =
            |ncr: &NozzleChangeResult| !ncr.gcode.is_empty() || ncr.heater_off_only || ncr.zero_flush_idle_cooling;
        if has_nozzle_change_action(&first.nozzle_change_result) {
            out.nozzle_change_result = first.nozzle_change_result.clone();
        } else if has_nozzle_change_action(&second.nozzle_change_result) {
            out.nozzle_change_result = second.nozzle_change_result.clone();
        }
        if first.is_tool_change {
            out.is_tool_change = true;
            out.tool_change_start_pos = first.tool_change_start_pos;
        } else if second.is_tool_change {
            out.is_tool_change = true;
            out.tool_change_start_pos = second.tool_change_start_pos;
        } else {
            out.is_tool_change = false;
        }
        // BBS
        out.purge_volume += second.purge_volume;
        out
    }

    /// WipeTower.cpp:3429-3448 `get_all_wall_skip_points`.
    fn get_all_wall_skip_points(&mut self, wall_filament: i32) {
        self.wall_skip_points.clear();
        self.wall_skip_points.resize(self.plan.len(), Vec::new());
        let mut current_tool = self.current_tool as i32;
        for i in 0..self.plan.len() as i32 {
            if self.layer_is_skipped(i) || self.plan[i as usize].depth < self.perimeter_width {
                continue;
            }
            let layer = self.plan[i as usize].clone();
            if (i as usize) >= self.wall_reserved.len() || self.wall_reserved[i as usize] == 0 {
                let mut wall_idx = self.get_wall_filament_for_layer(&layer, i, wall_filament);
                if wall_idx == -1
                    && self.wipe_tower_blocks.iter().any(|block| block.layers_type[i as usize] != WipeTowerLayerType::Normal)
                {
                    wall_idx = current_tool;
                }
                self.get_wall_skip_points(&layer, i, wall_idx);
            }
            if !layer.tool_changes.is_empty() {
                current_tool = layer.tool_changes.last().unwrap().new_tool as i32;
            }
        }
    }

    /// WipeTower.cpp:3450-3540 `get_wall_skip_points`: where the purges of a
    /// layer start, at which the outer wall leaves a gap.
    fn get_wall_skip_points(&mut self, layer: &WipeTowerLayerInfo, layer_id: i32, wall_filament: i32) {
        const PRE_ACCESS_LAYER: i32 = 4;
        let lid = layer_id as usize;
        // Only looked up, never iterated: the hash order does not matter.
        let mut cur_block_depth: HashMap<i32, f32> = HashMap::new();
        for i in 0..layer.tool_changes.len() {
            let tool_change = &layer.tool_changes[i];
            let old_filament = tool_change.old_tool;
            let new_filament = tool_change.new_tool;
            let mut nozzle_change_depth = tool_change.nozzle_change_depth;
            let wipe_depth = tool_change.required_depth - nozzle_change_depth;
            if !self.is_valid_last_layer(old_filament as i32, layer_id, self.plan[lid].z as f64) {
                nozzle_change_depth = 0.0;
            }
            let new_block_key = self.get_block_key(new_filament as i32, layer_id);
            let old_block_key = self.get_block_key(old_filament as i32, layer_id);
            let Some(block_idx) = self.find_block_by_category(new_block_key) else {
                continue;
            };
            let block_start_depth = self.wipe_tower_blocks[block_idx].start_depth;
            let mut process_depth = *cur_block_depth.entry(new_block_key).or_insert(block_start_depth);
            if self.is_need_ramming(new_filament, old_filament, layer_id) {
                if new_block_key == old_block_key {
                    process_depth += nozzle_change_depth;
                } else {
                    if !cur_block_depth.contains_key(&old_block_key) {
                        let Some(old_block) = self.find_block_by_category(old_block_key) else {
                            continue;
                        };
                        cur_block_depth.insert(old_block_key, self.wipe_tower_blocks[old_block].start_depth);
                    }
                    *cur_block_depth.get_mut(&old_block_key).unwrap() += nozzle_change_depth;
                }
            }

            let infill_gap_width = self.get_block_gap_width(new_filament as i32, false);
            let extra_spacing = self.plan[lid].extra_spacing;
            let mut res = Vec2f::zero();
            let index = layer_id % 4;
            match index % 4 {
                0 => res = Vec2f::new(0.0, process_depth),
                // process_depth + wipe_depth - extra_spacing * gap: one contracted multiply-add.
                1 => res = Vec2f::new(self.wipe_tower_width, (-extra_spacing).mul_add(infill_gap_width, process_depth + wipe_depth)),
                2 => res = Vec2f::new(self.wipe_tower_width, process_depth),
                3 => res = Vec2f::new(0.0, (-extra_spacing).mul_add(infill_gap_width, process_depth + wipe_depth)),
                _ => {}
            }
            self.wall_skip_points[lid].push(res);
            cur_block_depth.insert(new_block_key, process_depth + wipe_depth);

            let solid_toolchange = self.wipe_tower_blocks[block_idx].layers_type[lid] == WipeTowerLayerType::Contact;
            if solid_toolchange && self.enable_tower_interface_features {
                for j in 0..PRE_ACCESS_LAYER {
                    let pre_layer_id = layer_id - j;
                    if pre_layer_id < 0 {
                        break;
                    }
                    self.wall_skip_points[pre_layer_id as usize].push(res);
                }
            }
        }
        if self.enable_tower_interface_features {
            for bi in 0..self.wipe_tower_blocks.len() {
                let block = self.wipe_tower_blocks[bi].clone();
                // A band reserved for a merged slab above prints nothing here, so the wall needs no gap for it.
                if layer_id < block.layer_reserved.len() as i32 && block.layer_reserved[lid] != 0 {
                    continue;
                }
                let block_depth = cur_block_depth.get(&block.block_key).copied().unwrap_or(block.start_depth);
                if block_depth as f64 + crate::libslic3r::EPSILON
                    >= (block.start_depth + self.band_fill_depth(&block, layer_id) - self.perimeter_width) as f64
                {
                    continue;
                }
                let block_solid = block.layers_type[lid] == WipeTowerLayerType::Contact;
                let add_skip_point = block_solid && (((block_depth - block.start_depth).abs()) as f64) < crate::libslic3r::EPSILON;
                if add_skip_point {
                    let fill_filament = self.get_block_fill_filament(layer, layer_id, &block, wall_filament);
                    if fill_filament < 0 {
                        continue;
                    }
                    let fill_width = self.filament_perimeter_width(fill_filament);
                    let mut res = Vec2f::zero();
                    let index = layer_id % 4;
                    let dy_skip = self.band_fill_depth(&block, layer_id) - fill_width;
                    // (dy_skip + 0.25f * fill_width) / fill_width + 1, the sum contracted.
                    let n_skip = (0.25_f32.mul_add(fill_width, dy_skip) / fill_width + 1.0) as i32;
                    let gird_depth = fill_width * (n_skip - 1) as f32; // in sync with finish_block_solid
                    match index % 4 {
                        0 => res = Vec2f::new(0.0, block_depth),
                        1 => res = Vec2f::new(self.wipe_tower_width, block_depth + gird_depth),
                        2 => res = Vec2f::new(self.wipe_tower_width, block_depth),
                        3 => res = Vec2f::new(0.0, block_depth + gird_depth),
                        _ => {}
                    }
                    self.wall_skip_points[lid].push(res);
                    for j in 0..PRE_ACCESS_LAYER {
                        let pre_layer_id = layer_id - j;
                        if pre_layer_id < 0 {
                            break;
                        }
                        self.wall_skip_points[pre_layer_id as usize].push(res);
                    }
                }
            }
        }
    }

    /// WipeTower.cpp:5517-5549 `update_all_layer_depth`.
    fn update_all_layer_depth(&mut self, wipe_tower_depth: f32) {
        self.depth = 0.0;
        let start_offset = self.perimeter_width;
        let mut start_depth = start_offset;
        let extra_spacing = self.extra_spacing;
        for block in &mut self.wipe_tower_blocks {
            block.depth *= extra_spacing;
            block.start_depth = start_depth;
            start_depth += block.depth;
            self.depth += block.depth;
            for layer_depth in &mut block.layer_depths {
                *layer_depth *= extra_spacing;
            }
        }
        // Scale each layer once.
        for plan_info in &mut self.plan {
            plan_info.depth *= extra_spacing;
        }
        if self.depth > 0.0 {
            self.depth += start_offset;
        }
        // Keep every layer at the full tower depth when the outer walls must stay put for the diagonal
        // scrape (or when wrapping / smooth timelapse already needs a tower on every layer).
        if self.enable_wrapping_detection || self.enable_timelapse_print || self.keeps_outer_walls_unshrunk() {
            if is_approx_float(self.depth, 0.0) {
                self.depth = wipe_tower_depth;
            }
            let d = self.depth;
            for plan_info in &mut self.plan {
                plan_info.depth = d;
            }
        }
    }

    /// WipeTower.cpp:5551-5734 `generate_wipe_tower_blocks(add_solid_flag)`:
    /// the bands of the tower, their per-layer depths, the per-layer tower
    /// depth, and (with `add_solid_flag`) the solid and contact layers.
    fn generate_wipe_tower_blocks(&mut self, add_solid_flag: bool) {
        // 1. generate all layer depth
        self.all_layers_depth.clear();
        self.all_layers_depth.resize(self.plan.len(), Vec::new());
        self.cur_layer_id = 0;
        for li in 0..self.plan.len() {
            let tool_changes = self.plan[li].tool_changes.clone();
            for tool_change in &tool_changes {
                let cur = self.cur_layer_id as i32;
                if !self.is_need_ramming(tool_change.old_tool, tool_change.new_tool, cur) {
                    let key = self.get_block_key(tool_change.new_tool as i32, cur);
                    self.add_depth_to_block(tool_change.new_tool as i32, key, tool_change.required_depth, false);
                } else {
                    let key = self.get_block_key(tool_change.old_tool as i32, cur);
                    self.add_depth_to_block(tool_change.old_tool as i32, key, tool_change.nozzle_change_depth, true);
                    let key = self.get_block_key(tool_change.new_tool as i32, cur);
                    self.add_depth_to_block(tool_change.new_tool as i32, key, tool_change.required_depth - tool_change.nozzle_change_depth, false);
                }
            }
            self.cur_layer_id += 1;
        }

        // 2. generate all layer depth (looked up by key only)
        let mut all_layer_category_to_depth: Vec<HashMap<i32, f32>> = vec![HashMap::new(); self.plan.len()];
        for layer_id in 0..self.all_layers_depth.len() {
            for block in &self.all_layers_depth[layer_id] {
                all_layer_category_to_depth[layer_id].insert(block.category, block.depth);
            }
        }

        // 3. generate wipe tower block
        self.wipe_tower_blocks.clear();
        let n = all_layer_category_to_depth.len();
        for layer_id in 0..n {
            let mut ordered_keys: Vec<i32> = all_layer_category_to_depth[layer_id].keys().copied().collect();
            ordered_keys.sort_unstable();
            for key in ordered_keys {
                let value = all_layer_category_to_depth[layer_id][&key];
                let bi = self.get_block_by_category(key, true).unwrap();
                let block = &mut self.wipe_tower_blocks[bi];
                if block.layer_depths.is_empty() {
                    block.layer_depths.resize(n, 0.0);
                    block.finish_depth.resize(n, 0.0);
                    block.layers_type.resize(n, WipeTowerLayerType::Normal);
                    block.layer_heights.resize(n, 0.0);
                    block.layer_reserved.resize(n, 0);
                    block.layer_fill_filament.resize(n, -1);
                }
                // std::max(block->depth, value)
                block.depth = if block.depth < value { value } else { block.depth };
                block.layer_depths[layer_id] = value;
            }
        }
        // 4. get real depth for every layer
        let plan_len = self.plan.len();
        for layer_id in (0..plan_len).rev() {
            self.plan[layer_id].depth = 0.0;
            for block in &mut self.wipe_tower_blocks {
                if layer_id < plan_len - 1 {
                    let (a, b) = (block.layer_depths[layer_id], block.layer_depths[layer_id + 1]);
                    block.layer_depths[layer_id] = if a < b { b } else { a };
                }
                self.plan[layer_id].depth += block.layer_depths[layer_id];
            }
        }

        if self.tower_framework {
            for layer_id in 1..plan_len {
                self.plan[layer_id].depth = 0.0;
                for block in &mut self.wipe_tower_blocks {
                    block.layer_depths[layer_id] = block.layer_depths[0];
                    self.plan[layer_id].depth += block.layer_depths[layer_id];
                }
            }
        }

        // add solid infill flag
        if add_solid_flag {
            let solid_infill_layer_low: i32 = 4;
            let merged_fill = self.merged_fill_active();
            // Sets: only membership is asked, so the hash order does not matter.
            let mut layers_used_tools: Vec<std::collections::HashSet<i32>> = Vec::new();
            let mut layers_used_categories: Vec<std::collections::HashSet<i32>> = Vec::new();

            let mut first_tool: i32 = -1;
            for layer in &self.plan {
                if !layer.tool_changes.is_empty() {
                    first_tool = layer.tool_changes[0].old_tool as i32;
                    break;
                }
            }
            for layer_id in 0..plan_len as i32 {
                let info = &self.plan[layer_id as usize];
                let mut used_tools: std::collections::HashSet<i32> = std::collections::HashSet::new();
                if info.tool_changes.is_empty() {
                    used_tools.insert(self.get_block_key(first_tool, layer_id));
                } else {
                    for tool_change in &info.tool_changes {
                        used_tools.insert(self.get_block_key(tool_change.old_tool as i32, layer_id));
                        used_tools.insert(self.get_block_key(tool_change.new_tool as i32, layer_id));
                    }
                    first_tool = info.tool_changes.last().unwrap().new_tool as i32;
                }
                let mut used_categories: std::collections::HashSet<i32> = std::collections::HashSet::new();
                if layer_id > 0 && merged_fill {
                    used_categories = layers_used_categories[layer_id as usize - 1].clone();
                }
                if merged_fill && !info.tool_changes.is_empty() {
                    let mut kept: std::collections::HashSet<i32> = std::collections::HashSet::new();
                    for &category in &used_categories {
                        let column_continues = self.wipe_tower_blocks.iter().any(|block| {
                            Self::decode_block_category(block.block_key) == category
                                && layer_id < block.layer_depths.len() as i32
                                && block.layer_depths[layer_id as usize] > 2.0 * self.perimeter_width
                        });
                        if column_continues {
                            kept.insert(category);
                        }
                    }
                    used_categories = kept;
                }
                for &key in &used_tools {
                    used_categories.insert(Self::decode_block_category(key));
                }
                layers_used_tools.push(used_tools);
                layers_used_categories.push(used_categories);
            }

            let perimeter_width = self.perimeter_width;
            for block in &mut self.wipe_tower_blocks {
                let block_category = Self::decode_block_category(block.block_key);
                let block_key = block.block_key;
                let solid_has_band = |layer_id: usize| -> bool {
                    if merged_fill {
                        layers_used_categories[layer_id].contains(&block_category)
                    } else {
                        layers_used_tools[layer_id].contains(&block_key)
                    }
                };
                for layer_id in 0..n as i32 {
                    let lid = layer_id as usize;
                    // category_to_depth[block.block_key]: 0 when the key is not there.
                    let category_depth = all_layer_category_to_depth[lid].get(&block_key).copied().unwrap_or(0.0);
                    if category_depth < block.layer_depths[lid] - perimeter_width {
                        let cur_has_category = solid_has_band(lid);
                        let mut layer_count = solid_infill_layer_low;
                        while layer_count > 0 {
                            if layer_id + layer_count >= n as i32 {
                                layer_count -= 1;
                                continue;
                            }
                            let up_has_category = solid_has_band((layer_id + layer_count) as usize);
                            if cur_has_category != up_has_category {
                                block.layers_type[lid] = WipeTowerLayerType::Solid;
                                break;
                            }
                            layer_count -= 1;
                        }
                    }
                    if layer_id > 0 {
                        let cur_has_block_category = layers_used_categories[lid].contains(&block_category);
                        let pre_has_block_category = layers_used_categories[lid - 1].contains(&block_category);
                        if cur_has_block_category != pre_has_block_category {
                            block.layers_type[lid] = WipeTowerLayerType::Contact;
                        }
                        if block.layers_type[lid - 1] == WipeTowerLayerType::Contact
                            && block.layers_type[lid] != WipeTowerLayerType::Contact
                        {
                            block.layers_type[lid] = WipeTowerLayerType::ContactUp;
                        }
                    }
                }
            }
        }

        // Carry the slab plan onto the blocks now that they exist.
        for bi in 0..self.wipe_tower_blocks.len() {
            for layer_id in 0..plan_len as i32 {
                let lid = layer_id as usize;
                let key = self.wipe_tower_blocks[bi].block_key;
                let height = self.layer_slab_height(layer_id, self.plan[lid].height);
                let reserved = if self.layer_is_skipped(layer_id)
                    || (layer_id < self.reserved_blocks.len() as i32 && self.reserved_blocks[lid].contains(&key))
                {
                    1
                } else {
                    0
                };
                let mut layer_height = height;
                if layer_id < self.block_slab_height.len() as i32 {
                    if let Some(&h) = self.block_slab_height[lid].get(&key) {
                        if h > 0.0 {
                            layer_height = h;
                        }
                    }
                }
                let block = &mut self.wipe_tower_blocks[bi];
                block.layer_heights[lid] = layer_height;
                block.layer_reserved[lid] = reserved;
            }
        }
    }

    /// WipeTower.cpp:5736-5830 `calc_block_infill_gap`: the line pitch of
    /// every (category, diameter) group, then every tool change re-planned
    /// with it.
    fn calc_block_infill_gap(&mut self) {
        // Keyed by the lane-independent block key; the float sum below runs in
        // the libc++ iteration order of this map.
        let mut block_info: LibcxxHashMapI32<CalcGapBlockInfo> = LibcxxHashMapI32::new();
        let mut high_block_info: LibcxxHashMapI32<CalcGapBlockInfo> = LibcxxHashMapI32::new();
        let mut i = self.plan.len() as i32 - 1;
        while i >= 0 {
            let tool_changes = self.plan[i as usize].tool_changes.clone();
            for toolchange in &tool_changes {
                let new_tool = toolchange.new_tool as i32;
                let old_tool = toolchange.old_tool as i32;
                let old_key = self.get_gap_key(old_tool);
                let new_key = self.get_gap_key(new_tool);
                if self.is_need_ramming(old_tool as usize, new_tool as usize, i) {
                    let extruder_change = !self.is_same_extruder(new_tool as usize, old_tool as usize, i);
                    let pw = self.filament_perimeter_width(old_tool);
                    let ncw = self.filament_nozzle_change_width(old_tool);
                    let reverse = self.is_need_reverse_travel(old_tool, extruder_change);
                    let e = block_info.entry(old_key);
                    e.has_ramming = true;
                    e.perimeter_width = pw;
                    e.nozzle_change_perimeter_width = ncw;
                    if reverse {
                        e.has_reverse_travel = true;
                    }
                    e.depth += toolchange.nozzle_change_depth;
                }
                if block_info.count(new_key) == 0 {
                    block_info.entry(new_key);
                }
                let pw = self.filament_perimeter_width(new_tool);
                let ncw = self.filament_nozzle_change_width(new_tool);
                let e = block_info.entry(new_key);
                e.perimeter_width = pw;
                e.nozzle_change_perimeter_width = ncw;
                e.depth += toolchange.required_depth - toolchange.nozzle_change_depth;
            }
            for nd in block_info.nodes_in_order() {
                let k = block_info.keys[nd];
                if high_block_info.count(k) != 0 {
                    let hn = high_block_info.find_node(k);
                    if high_block_info.vals[hn].depth > block_info.vals[nd].depth {
                        block_info.vals[nd].depth = high_block_info.vals[hn].depth;
                    }
                }
            }
            high_block_info.assign_from(&block_info);
            for nd in block_info.nodes_in_order() {
                block_info.vals[nd].depth = 0.0;
            }
            if i == 0 {
                block_info.assign_from(&high_block_info);
            }
            i -= 1;
        }
        // std::accumulate in the iteration order.
        let mut max_depth = 0.0_f32;
        for (_, info) in block_info.items() {
            max_depth += info.depth;
        }
        let height_to_depth = Self::get_limit_depth_by_height(self.wipe_tower_height);
        let height_to_spacing = if max_depth > height_to_depth { 1.0 } else { height_to_depth / max_depth };

        let spacing_ratio = self.extra_spacing - 1.0;
        for (key, info) in block_info.items() {
            let band_width = if info.perimeter_width > 0.0 { info.perimeter_width } else { self.perimeter_width };
            let band_nc_width = if info.nozzle_change_perimeter_width > 0.0 {
                info.nozzle_change_perimeter_width
            } else {
                self.nozzle_change_perimeter_width
            };
            let extra_width = spacing_ratio * band_width;
            let line_gap_tol = 2.0 * band_nc_width;
            let gap = self.block_infill_gap_width.entry(key).or_insert((0.0, 0.0));
            if !info.has_ramming {
                // case1: no ramming, it can always align
                gap.1 = extra_width + band_width;
                gap.0 = gap.1;
            } else if !info.has_reverse_travel {
                // case2: has ramming, but no reverse travel
                let mut line_gap = band_nc_width + extra_width;
                if !self.use_rib_wall {
                    line_gap *= height_to_spacing;
                }
                if line_gap < line_gap_tol {
                    gap.0 = band_width + extra_width;
                    gap.1 = band_nc_width + extra_width;
                } else {
                    gap.1 = band_nc_width + extra_width;
                    gap.0 = gap.1;
                }
            } else {
                // case 3: has ramming and reverse travel
                let extra_tpu_fix_spacing = self.tpu_fixed_spacing - 1.0;
                let m = extra_tpu_fix_spacing * band_width;
                let mx = if m < extra_width { extra_width } else { m };
                let mut line_gap = band_nc_width + mx;
                if !self.use_rib_wall {
                    line_gap = height_to_spacing * line_gap;
                }
                if line_gap < line_gap_tol {
                    gap.0 = band_width + extra_width;
                    gap.1 = band_nc_width + mx;
                } else {
                    gap.1 = band_nc_width + mx;
                    gap.0 = gap.1;
                }
            }
        }

        // 2. recalculate toolchange depth
        for idx in 0..self.plan.len() {
            for t in 0..self.plan[idx].tool_changes.len() {
                let tc = self.plan[idx].tool_changes[t].clone();
                let height = self.plan[idx].height;
                self.plan[idx].tool_changes[t] =
                    self.set_toolchange(tc.old_tool as i32, tc.new_tool as i32, height, tc.wipe_volume, tc.purge_volume, idx as i32);
            }
        }
        self.extra_spacing = 1.0;
    }

    /// WipeTower.cpp:7256-7358 `plan_tower_new`: the Z ladder, the merged
    /// slabs, the gaps, the bands, the extra spacing and the merged fills.
    fn plan_tower_new(&mut self) {
        if self.wipe_tower_brim_width_cpp < 0.0 {
            self.wipe_tower_brim_width_cpp = Self::get_auto_brim_by_height(self.wipe_tower_height);
        }
        self.plan_tower_ladder();
        self.plan_merged_slabs();
        let foundation_layer = self.first_printed_layer_idx();
        self.calc_block_infill_gap();
        if self.use_rib_wall {
            // recalculate wipe_tower_with and layer's depth
            self.generate_wipe_tower_blocks(false);
            let mut max_depth = 0.0_f32;
            for t in &self.wipe_tower_blocks {
                max_depth += t.depth;
            }
            max_depth += self.perimeter_width;
            let square_width = align_ceil((max_depth * self.wipe_tower_width * self.extra_spacing).sqrt(), self.perimeter_width);
            self.wipe_tower_width = square_width;
            self.config.width = square_width;
            for idx in 0..self.plan.len() {
                for t in 0..self.plan[idx].tool_changes.len() {
                    let tc = self.plan[idx].tool_changes[t].clone();
                    let height = self.plan[idx].height;
                    self.plan[idx].tool_changes[t] =
                        self.set_toolchange(tc.old_tool as i32, tc.new_tool as i32, height, tc.wipe_volume, tc.purge_volume, idx as i32);
                }
            }
        }

        self.generate_wipe_tower_blocks(true);

        let mut max_depth = 0.0_f32;
        for block in &self.wipe_tower_blocks {
            max_depth += block.depth;
        }

        let min_wipe_tower_depth = Self::get_limit_depth_by_height(self.wipe_tower_height);

        // only for get m_extra_spacing
        {
            let eps = crate::libslic3r::EPSILON;
            if self.enable_wrapping_detection && (max_depth as f64) < eps {
                // WipeTower.hpp:35 `wrapping_wipe_tower_depth = 10.0`
                max_depth = WRAPPING_WIPE_TOWER_DEPTH;
                if self.use_rib_wall {
                    self.wipe_tower_width = max_depth;
                    self.config.width = max_depth;
                }
            }
            if self.enable_timelapse_print && (max_depth as f64) < eps {
                max_depth = min_wipe_tower_depth;
                if self.use_rib_wall {
                    self.wipe_tower_width = max_depth;
                    self.config.width = max_depth;
                }
            }
            if (max_depth as f64) + eps < min_wipe_tower_depth as f64 {
                // if enable rib_wall, there is no need to set extra_spacing
                if self.use_rib_wall {
                    let v = min_wipe_tower_depth * (2.0_f64).sqrt() as f32;
                    self.rib_length = if self.rib_length < v { v } else { self.rib_length };
                } else {
                    let v = min_wipe_tower_depth / max_depth;
                    self.extra_spacing = if v < self.extra_spacing { self.extra_spacing } else { v };
                }
            }

            for idx in 0..self.plan.len() {
                if idx as i32 == foundation_layer {
                    // apply solid fill for the first layer
                    self.plan[idx].extra_spacing = 1.0;
                    for t in 0..self.plan[idx].tool_changes.len() {
                        let tc = self.plan[idx].tool_changes[t].clone();
                        let info_height = self.plan[idx].height;
                        let new_tool_width = self.filament_perimeter_width(tc.new_tool as i32);
                        let old_tool_nc_width = self.filament_nozzle_change_width(tc.old_tool as i32);
                        let line_len = self.wipe_tower_width - 2.0 * new_tool_width;
                        let wipe_depth = (tc.required_depth - tc.nozzle_change_depth) * self.extra_spacing;
                        let wipe_line_count = wipe_depth / new_tool_width;
                        let nozzle_change_depth = tc.nozzle_change_depth * self.extra_spacing;
                        // (ncd * extra_spacing + WT_EPSILON) / width, the sum contracted, truncated.
                        let nozzle_change_line_count =
                            (tc.nozzle_change_depth.mul_add(self.extra_spacing, WIPE_TOWER_WT_EPSILON) / old_tool_nc_width) as i32;
                        let wipe_length = wipe_line_count * line_len;
                        let wipe_volume = self.length_to_volume(wipe_length, new_tool_width, info_height);
                        let nozzle_change_length =
                            nozzle_change_line_count as f32 * (self.wipe_tower_width - (old_tool_nc_width + new_tool_width));
                        let toolchange = &mut self.plan[idx].tool_changes[t];
                        toolchange.required_depth = wipe_depth + nozzle_change_depth;
                        toolchange.wipe_length = wipe_length;
                        toolchange.wipe_volume = wipe_volume;
                        toolchange.nozzle_change_length = nozzle_change_length;
                        toolchange.nozzle_change_depth = nozzle_change_depth;
                    }
                } else {
                    let es = self.extra_spacing;
                    self.plan[idx].extra_spacing = es;
                    for t in 0..self.plan[idx].tool_changes.len() {
                        let tc = self.plan[idx].tool_changes[t].clone();
                        let info_height = self.plan[idx].height;
                        let wipe_length = self.volume_to_length(
                            tc.wipe_volume,
                            self.filament_perimeter_width(tc.new_tool as i32),
                            self.slab_height_for(tc.new_tool as i32, idx as i32, info_height),
                        );
                        let toolchange = &mut self.plan[idx].tool_changes[t];
                        toolchange.required_depth *= es;
                        toolchange.nozzle_change_depth *= es;
                        toolchange.wipe_length = wipe_length;
                    }
                }
            }
        }

        self.update_all_layer_depth(max_depth);
        self.set_nozzle_last_layer_id();

        self.plan_merged_block_fill();
        self.plan_merged_outer_wall();
        self.audit_merged_slabs();

        // sqrt(d*d + w*w) of floats: the float overload, the sum contracted.
        let diagonal = self.depth.mul_add(self.depth, self.wipe_tower_width * self.wipe_tower_width).sqrt();
        self.rib_length = if self.rib_length < diagonal { diagonal } else { self.rib_length };
        self.rib_length += self.extra_rib_length;
        self.rib_length = if diagonal < self.rib_length { self.rib_length } else { diagonal };
        let m = if self.wipe_tower_width < self.depth { self.wipe_tower_width } else { self.depth };
        let half = m / 2.0;
        self.rib_width = if half < self.rib_width { half } else { self.rib_width };
    }

    /// WipeTower.cpp:7370-7435 `get_wall_filament_for_all_layer`: the filament
    /// of the most-visited category that visits the most layers, among those
    /// that can print the base layers when any can.
    fn get_wall_filament_for_all_layer(&self) -> i32 {
        use std::collections::{BTreeMap, HashSet};
        let mut category_counts: BTreeMap<i32, i32> = BTreeMap::new();
        let mut filament_counts: BTreeMap<i32, i32> = BTreeMap::new();
        let mut current_tool = self.current_tool as i32;
        for layer in &self.plan {
            if layer.tool_changes.is_empty() {
                *filament_counts.entry(current_tool).or_insert(0) += 1;
                *category_counts.entry(self.get_filament_category(current_tool as usize)).or_insert(0) += 1;
                continue;
            }
            let mut used_tools: HashSet<i32> = HashSet::new();
            let mut used_category: HashSet<i32> = HashSet::new();
            for i in 0..layer.tool_changes.len() {
                let old_tool = layer.tool_changes[i].old_tool as i32;
                let new_tool = layer.tool_changes[i].new_tool as i32;
                if i == 0 {
                    *filament_counts.entry(old_tool).or_insert(0) += 1;
                    *category_counts.entry(self.get_filament_category(old_tool as usize)).or_insert(0) += 1;
                    used_tools.insert(old_tool);
                    used_category.insert(self.get_filament_category(old_tool as usize));
                }
                if !used_category.contains(&self.get_filament_category(new_tool as usize)) {
                    *category_counts.entry(self.get_filament_category(new_tool as usize)).or_insert(0) += 1;
                }
                if !used_tools.contains(&new_tool) {
                    *filament_counts.entry(new_tool).or_insert(0) += 1;
                }
                used_tools.insert(new_tool);
                used_category.insert(self.get_filament_category(new_tool as usize));
            }
            current_tool = layer.tool_changes.last().unwrap().new_tool as i32;
        }

        // The wall spans every layer, so it has to come from a nozzle that can print the base layer height.
        if filament_counts.keys().any(|&f| self.can_print_base_layers(f)) {
            filament_counts.retain(|&f, _| self.can_print_base_layers(f));
            category_counts.clear();
            for (&f, &c) in &filament_counts {
                *category_counts.entry(self.get_filament_category(f as usize)).or_insert(0) += c;
            }
        }

        let mut selected_category = -1;
        let mut selected_count = 0;
        for (&cat, &count) in &category_counts {
            if count > selected_count {
                selected_category = cat;
                selected_count = count;
            }
        }

        let mut filament_id = -1;
        let mut filament_count = 0;
        for (&f, &count) in &filament_counts {
            if self.filament_categories[f as usize] == selected_category && count > filament_count {
                filament_id = f;
                filament_count = count;
            }
        }
        filament_id
    }

    /// WipeTower.cpp:7451-7499 `get_wall_filament_for_layer`.
    fn get_wall_filament_for_layer(&self, layer: &WipeTowerLayerInfo, layer_id: i32, wall_filament: i32) -> i32 {
        if layer.tool_changes.is_empty() {
            return -1;
        }
        if layer_id >= 0 && (layer_id as usize) < self.wall_filament.len() && self.wall_filament[layer_id as usize] >= 0 {
            return self.wall_filament[layer_id as usize];
        }
        let enforce_height_window = self.merged_fill_active();
        let fits_this_layer = |filament_id: i32| -> bool {
            !enforce_height_window || self.filament_can_print_height(filament_id, self.layer_slab_height(layer_id, layer.height))
        };
        // size_t == int: the int converted to size_t.
        let eq_tool = |tool: usize, filament: i32| tool == filament as usize;

        let mut candidate_id = -1;
        for idx in 0..layer.tool_changes.len() {
            let tc = &layer.tool_changes[idx];
            if idx == 0 {
                if eq_tool(tc.old_tool, wall_filament) && self.is_valid_last_layer(tc.old_tool as i32, layer_id, layer.z as f64) {
                    return wall_filament;
                }
                if self.filament_params[tc.old_tool].category == self.filament_params[wall_filament as usize].category
                    && self.is_valid_last_layer(tc.old_tool as i32, layer_id, layer.z as f64)
                    && fits_this_layer(tc.old_tool as i32)
                {
                    candidate_id = tc.old_tool as i32;
                }
            }
            if eq_tool(tc.new_tool, wall_filament) && fits_this_layer(wall_filament) {
                return wall_filament;
            }
            if candidate_id == -1
                && self.filament_params[tc.new_tool].category == self.filament_params[wall_filament as usize].category
                && fits_this_layer(tc.new_tool as i32)
            {
                candidate_id = tc.new_tool as i32;
            }
        }
        if candidate_id != -1 {
            return candidate_id;
        }

        let mut fallback_id = layer.tool_changes[0].new_tool as i32;
        for tool_change in &layer.tool_changes {
            if fits_this_layer(tool_change.new_tool as i32) {
                return tool_change.new_tool as i32;
            }
        }
        for tool_change in &layer.tool_changes {
            if self.filament_params[tool_change.new_tool].nozzle_diameter < self.filament_params[fallback_id as usize].nozzle_diameter {
                fallback_id = tool_change.new_tool as i32;
            }
        }
        fallback_id
    }

    /// WipeTower.cpp:7501-7533 `block_fill_filament_base`: the filament that
    /// tops a band up on a layer, chosen without regard to the slab height.
    fn block_fill_filament_base(&self, layer: &WipeTowerLayerInfo, layer_id: i32, block: &WipeTowerBlock, wall_filament: i32) -> i32 {
        let mut last_filament_change = -1;
        let mut last_nozzle_change = -1;
        for tool_change in &layer.tool_changes {
            if self.get_block_key(tool_change.new_tool as i32, layer_id) == block.block_key {
                last_filament_change = tool_change.new_tool as i32;
            }
            if tool_change.nozzle_change_depth as f64 > crate::libslic3r::EPSILON
                && self.is_need_ramming(tool_change.old_tool, tool_change.new_tool, layer_id)
                && self.is_valid_last_layer(tool_change.old_tool as i32, layer_id, layer.z as f64)
                && self.get_block_key(tool_change.old_tool as i32, layer_id) == block.block_key
            {
                last_nozzle_change = tool_change.old_tool as i32;
            }
        }

        let mut fill_filament = if last_filament_change != -1 { last_filament_change } else { last_nozzle_change };
        if fill_filament == -1
            && !layer.tool_changes.is_empty()
            && self.get_block_key(layer.tool_changes[0].old_tool as i32, layer_id) == block.block_key
        {
            fill_filament = layer.tool_changes[0].old_tool as i32;
        }
        if fill_filament == -1 && self.merged_fill_active() {
            fill_filament = self.same_category_filler(layer, block.block_key);
            // A layer without tool changes is laid down by whatever is still loaded.
            if fill_filament == -1 && layer.tool_changes.is_empty() {
                let loaded = self.loaded_filament(layer_id);
                if loaded >= 0 && self.get_filament_category(loaded as usize) == Self::decode_block_category(block.block_key) {
                    fill_filament = loaded;
                }
            }
            return fill_filament;
        }
        if fill_filament == -1 {
            fill_filament = wall_filament;
        }
        fill_filament
    }

    /// WipeTower.cpp:7535-7587 `get_block_fill_filament`.
    fn get_block_fill_filament(&self, layer: &WipeTowerLayerInfo, layer_id: i32, block: &WipeTowerBlock, wall_filament: i32) -> i32 {
        // The support under an extra TPU lane already chose one filament for the whole slab.
        if layer_id >= 0
            && layer_id < block.layer_fill_filament.len() as i32
            && block.layer_fill_filament[layer_id as usize] >= 0
        {
            let planned = block.layer_fill_filament[layer_id as usize];
            let height = block.layer_heights[layer_id as usize];
            if self.filament_can_print_height(planned, height) {
                return planned;
            }
            let category = self.get_filament_category(planned as usize);
            for tool_change in &layer.tool_changes {
                for filament in [tool_change.new_tool as i32, tool_change.old_tool as i32] {
                    if self.get_filament_category(filament as usize) == category && self.filament_can_print_height(filament, height) {
                        return filament;
                    }
                }
            }
            return planned;
        }
        let fill_filament = self.block_fill_filament_base(layer, layer_id, block, wall_filament);
        if fill_filament < 0 {
            return -1;
        }
        let height = block.layer_heights[layer_id as usize];
        if !self.filament_can_print_height(fill_filament, height) {
            let same_category_only = self.merged_fill_active();
            let can_take_over = |filament: i32| -> bool {
                self.filament_can_print_height(filament, height)
                    && (!same_category_only
                        || self.get_filament_category(filament as usize) == Self::decode_block_category(block.block_key))
            };
            for tool_change in &layer.tool_changes {
                if can_take_over(tool_change.new_tool as i32) {
                    return tool_change.new_tool as i32;
                }
                if can_take_over(tool_change.old_tool as i32) {
                    return tool_change.old_tool as i32;
                }
            }
            // :7577-7582 — C++ logs an error and lays it down with the planned filament.
        }
        fill_filament
    }
}

/// WipeTower.cpp:7589-7758 `apply_wipe_tower_tail_slowdown`: the trailing
/// extrusion moves of an assembled tower visit slowed to `speed_factor` over
/// their last `tail_volume` mm³ (all of it when `tail_volume <= 0`), splitting
/// the first move that straddles the start of that tail.
pub fn apply_wipe_tower_tail_slowdown(
    gcode: &mut String,
    tail_volume: f32,
    speed_factor: f32,
    filament_area: f32,
    initial_position: Option<Vec2f>,
    tool_change_start_position: Option<Vec2f>,
) {
    if gcode.is_empty() || filament_area <= 0.0 || speed_factor >= 1.0 - 1e-4_f32 {
        return;
    }
    let speed_factor = speed_factor.clamp(0.05, 1.0);

    // Split into lines (content only). Rejoining with '\n' reproduces the original exactly.
    let mut lines: Vec<String> = gcode.split('\n').map(|l| l.to_string()).collect();

    // Return the numeric value of the token starting with `key` on a G-code move line.
    let get_param = |line: &str, key: u8| -> Option<f32> {
        let b = line.as_bytes();
        let n = b.len();
        let mut i = 0usize;
        while i < n {
            while i < n && b[i] == b' ' {
                i += 1;
            }
            if i >= n {
                break;
            }
            let tok = i;
            while i < n && b[i] != b' ' {
                i += 1;
            }
            if b[tok] == key && i - tok >= 2 {
                return Some(crate::locales_utils::string_to_double_decimal_point(&line[tok + 1..i]).0 as f32);
            }
        }
        None
    };

    // Replace the F value of a move line, or append one if the line has no explicit feedrate.
    let set_feedrate = |line: &mut String, f: f32| {
        let fi = 1.max((f + 0.5).floor() as i32);
        let b = line.as_bytes().to_vec();
        let n = b.len();
        let mut i = 0usize;
        while i < n {
            while i < n && b[i] == b' ' {
                i += 1;
            }
            let tok = i;
            while i < n && b[i] != b' ' {
                i += 1;
            }
            if tok < n && b[tok] == b'F' {
                *line = format!("{}F{}{}", &line[..tok], fi, &line[i..]);
                return;
            }
        }
        line.push_str(&format!(" F{}", fi));
    };

    // A plain linear move made of G1/X/Y/E/F tokens only, which can be split in two.
    let is_plain_linear = |line: &str| -> bool {
        let b = line.as_bytes();
        let n = b.len();
        let mut i = 0usize;
        let mut first = true;
        while i < n {
            while i < n && b[i] == b' ' {
                i += 1;
            }
            if i >= n {
                break;
            }
            let tok = i;
            while i < n && b[i] != b' ' {
                i += 1;
            }
            if first {
                if &line[tok..i] != "G1" {
                    return false;
                }
                first = false;
            } else if !b"XYEF".contains(&b[tok]) {
                return false;
            }
        }
        true
    };

    struct Move {
        line_idx: usize,
        e: f32,
        feedrate: f32,
        splittable: bool,
        from: Vec2f,
        to: Vec2f,
    }
    let mut moves: Vec<Move> = Vec::new();
    let mut current_feedrate = 0.0_f32;
    // Mirror post_process_wipe_tower_moves(): the placeholder resets local XY to the tool-change start.
    let mut position = initial_position.unwrap_or(Vec2f::zero());
    let mut has_x_known = initial_position.is_some();
    let mut has_y_known = initial_position.is_some();
    for idx in 0..lines.len() {
        let line = lines[idx].as_str();
        if line == "[change_filament_gcode]" {
            if let Some(p) = tool_change_start_position {
                position = p;
                has_x_known = true;
                has_y_known = true;
            } else {
                has_x_known = false;
                has_y_known = false;
            }
            continue;
        }
        let Some(p) = line.find(|c: char| c != ' ') else {
            continue;
        };
        if line.as_bytes()[p] != b'G' {
            continue;
        }
        let q = line[p..].find(' ').map(|o| p + o);
        let cmd = &line[p..q.unwrap_or(line.len())];
        let is_arc = cmd == "G2" || cmd == "G3";
        if cmd != "G1" && !is_arc {
            continue;
        }
        let fval = get_param(line, b'F');
        let eval = get_param(line, b'E');
        let xval = get_param(line, b'X');
        let yval = get_param(line, b'Y');
        let (has_f, has_e, has_x, has_y) = (fval.is_some(), eval.is_some(), xval.is_some(), yval.is_some());
        let is_motion = has_x || has_y || is_arc;
        // Track the running print feedrate from motion moves and standalone "G1 F" lines,
        // but ignore pure retract/de-retract moves (E only) so retract speeds don't leak in.
        if has_f && (is_motion || (!has_e && !has_x && !has_y)) {
            current_feedrate = fval.unwrap();
        }
        let from = position;
        let from_known = has_x_known && has_y_known;
        if let Some(x) = xval {
            position.x = x;
            has_x_known = true;
        }
        if let Some(y) = yval {
            position.y = y;
            has_y_known = true;
        }
        let e = eval.unwrap_or(0.0);
        if has_e && e > 0.0 && is_motion && current_feedrate > 0.0 {
            moves.push(Move {
                line_idx: idx,
                e,
                feedrate: current_feedrate,
                splittable: from_known && !is_arc && is_plain_linear(line),
                from,
                to: position,
            });
        }
    }
    if moves.is_empty() {
        return;
    }

    let tail_len = if tail_volume <= 0.0 { f32::MAX } else { tail_volume / filament_area };
    let mut acc = 0.0_f32;
    let mut m = moves.len() as i32 - 1;
    while m >= 0 {
        if acc >= tail_len - 1e-6_f32 {
            break;
        }
        let mv = &moves[m as usize];
        let remaining = tail_len - acc;
        acc += mv.e;
        if mv.splittable && mv.e > remaining + 1e-5_f32 {
            let t_head = 1.0_f32 - remaining / mv.e;
            let d = mv.to - mv.from;
            let mid = mv.from + Vec2f::new(d.x * t_head, d.y * t_head);
            let mid_x = float_to_string_decimal_point(mid.x as f64, 3);
            let mid_y = float_to_string_decimal_point(mid.y as f64, 3);
            // A split point that prints identical to either end would leave raw tower coordinates in the G-code.
            let prints_as = |p: Vec2f| {
                mid_x == float_to_string_decimal_point(p.x as f64, 3) && mid_y == float_to_string_decimal_point(p.y as f64, 3)
            };
            // Tail too short to print on its own: keep this move at its original feedrate.
            if prints_as(mv.to) {
                break;
            }
            // Head too short to print on its own: slow the whole move below.
            if prints_as(mv.from) {
                set_feedrate(&mut lines[mv.line_idx], mv.feedrate * speed_factor);
                break;
            }
            // Only the end of this move is in the tail: its head keeps the original feedrate.
            let e_head = float_to_string_decimal_point((mv.e * t_head) as f64, 5);
            let e_tail = mv.e - crate::locales_utils::string_to_double_decimal_point(&e_head).0 as f32;
            let f_head = get_param(&lines[mv.line_idx], b'F');
            let mut head = format!("G1 X{} Y{} E{}", mid_x, mid_y, e_head);
            if let Some(fh) = f_head {
                head.push_str(&format!(" F{}", float_to_string_decimal_point(fh as f64, 3)));
            }
            let mut tail = format!(
                "G1 X{} Y{} E{}",
                float_to_string_decimal_point(mv.to.x as f64, 3),
                float_to_string_decimal_point(mv.to.y as f64, 3),
                float_to_string_decimal_point(e_tail as f64, 5)
            );
            set_feedrate(&mut tail, mv.feedrate * speed_factor);
            lines[mv.line_idx] = format!("{}\n{}", head, tail);
            break;
        }
        set_feedrate(&mut lines[mv.line_idx], mv.feedrate * speed_factor);
        m -= 1;
    }

    let mut out = String::with_capacity(gcode.len() + 16);
    for i in 0..lines.len() {
        out.push_str(&lines[i]);
        if i + 1 < lines.len() {
            out.push('\n');
        }
    }
    *gcode = out;
}

impl WipeTower {
    /// WipeTower.cpp:7760-8110 `generate_new`: the tower, planned layer by
    /// planned layer, as the tool changes and finishes of each (results in the
    /// tower's own frame).
    pub fn generate_new(&mut self, result: &mut Vec<Vec<ToolChangeResult>>) {
        if self.plan.is_empty() {
            return;
        }
        // real wipe_tower_height
        self.wipe_tower_height = self.plan.last().unwrap().z;
        self.config.height = self.wipe_tower_height;
        self.plan_tower_new();
        self.layer_idx = 0;

        if let Some(layer) = self.plan.iter().find(|l| !l.tool_changes.is_empty()) {
            self.current_tool = layer.tool_changes[0].old_tool;
        }

        for used in &mut self.used_filament_length {
            *used = 0.0; // reset used filament stats
        }

        let wall_filament = self.get_wall_filament_for_all_layer();
        if self.use_gap_wall {
            self.get_all_wall_skip_points(wall_filament);
        }

        let mut layer_result: Vec<ToolChangeResult> = Vec::new();
        let mut index = 0usize;
        let last_z = self.plan.last().unwrap().z;
        for li in 0..self.plan.len() {
            let layer = self.plan[li].clone();
            self.reset_block_status();
            self.cur_layer_id = index;
            index += 1;
            let cur = self.cur_layer_id;
            let slab = self.layer_slab_height(cur as i32, layer.height);
            self.set_layer(layer.z, slab, 0, false, layer.z == last_z);
            if self.layer_is_skipped(cur as i32) {
                // The fill front each block reached is carried over for get_floating_area().
                for block in &mut self.wipe_tower_blocks {
                    block.finish_depth[cur] =
                        if cur > 0 { block.finish_depth[cur - 1] } else { block.start_depth + block.depth };
                }
                result.push(Vec::new());
                continue;
            }
            if self.plan[self.layer_idx].depth < self.perimeter_width {
                // Same empty-layer slot as a skipped layer so later G-code stays indexed by planned layer.
                result.push(Vec::new());
                continue;
            }
            // Do not centre a shallow layer with y_shift when the scrape needs the outer walls locked.
            if self.wipe_tower_blocks.len() == 1 && !self.keeps_outer_walls_unshrunk() {
                let layer_depth = self.plan[self.layer_idx].depth;
                if layer_depth < self.depth - self.perimeter_width {
                    // align y shift to perimeter width
                    let dy = self.extra_spacing * self.perimeter_width;
                    self.y_shift = (self.depth - layer_depth) / 2.0;
                    self.y_shift = align_round(self.y_shift, dy);
                }
            }

            let mut finish_layer_tcr = ToolChangeResult::default();
            let mut timelapse_wall = ToolChangeResult::default();

            let only_generate_wall =
                self.enable_timelapse_print || (self.enable_wrapping_detection && self.slice_used_filaments <= 1);
            let mut wall_idx = self.get_wall_filament_for_layer(&layer, cur as i32, wall_filament);
            let wall_reserved = cur < self.wall_reserved.len() && self.wall_reserved[cur] != 0;
            let extrude_outer_wall = !only_generate_wall && !wall_reserved;
            let wall_height = if cur < self.wall_slab_height.len() && self.wall_slab_height[cur] > 0.0 {
                self.wall_slab_height[cur]
            } else {
                self.layer_height
            };
            // :7826-7829 — a warning is logged when no nozzle covers the wall height.

            // this layer has no tool_change
            if wall_idx == -1 {
                let need_insert_solid_infill =
                    self.wipe_tower_blocks.iter().any(|block| block.layers_type[cur] != WipeTowerLayerType::Normal);
                if need_insert_solid_infill {
                    wall_idx = self.current_tool as i32;
                } else {
                    if only_generate_wall {
                        timelapse_wall = self.only_generate_out_wall(true);
                    }
                    finish_layer_tcr =
                        self.finish_layer_new(extrude_outer_wall, layer.extruder_fill, true, wall_height, wall_reserved);
                    // Only a dense fill supports the layer above; a sparse grid has to be bridged.
                    let dense_fill = self.merged_slabs_active && layer.extruder_fill && self.adhesion && self.is_first_layer();
                    for bi in 0..self.wipe_tower_blocks.len() {
                        let block = &self.wipe_tower_blocks[bi];
                        let v = if !dense_fill || block.layer_reserved[cur] != 0 {
                            block.start_depth
                        } else {
                            block.start_depth + self.band_fill_depth(block, cur as i32)
                        };
                        self.wipe_tower_blocks[bi].finish_depth[cur] = v;
                    }
                }
            }

            // generate tool change
            let mut insert_finish_layer_idx: i32 = -1;
            if wall_idx != -1 && only_generate_wall {
                timelapse_wall = self.only_generate_out_wall(true);
            }
            // size_t == int: the int converted to size_t.
            let eq_tool = |tool: usize, filament: i32| tool == filament as usize;
            for i in 0..layer.tool_changes.len() {
                let tc = layer.tool_changes[i].clone();
                if self.current_tool != tc.old_tool {
                    // :7870-7873 — a warning is logged and the tool state realigned.
                    self.current_tool = tc.old_tool;
                }
                if i == 0 && eq_tool(tc.old_tool, wall_idx) {
                    finish_layer_tcr = self.finish_layer_new(extrude_outer_wall, false, false, wall_height, wall_reserved);
                }
                let mut solid_nozzlechange = false;
                let mut solid_toolchange = false;
                let key = self.get_block_key(tc.new_tool as i32, cur as i32);
                if let Some(b) = self.find_block_by_category(key) {
                    solid_toolchange = self.wipe_tower_blocks[b].layers_type[cur] == WipeTowerLayerType::Contact;
                }
                let key2 = self.get_block_key(tc.old_tool as i32, cur as i32);
                if let Some(b2) = self.find_block_by_category(key2) {
                    solid_nozzlechange = self.wipe_tower_blocks[b2].layers_type[cur] == WipeTowerLayerType::Contact;
                }
                let tcr = self.tool_change_new(tc.new_tool, solid_toolchange, solid_nozzlechange);
                layer_result.push(tcr);

                if i == 0 && eq_tool(tc.old_tool, wall_idx) {
                } else if eq_tool(tc.new_tool, wall_idx) {
                    finish_layer_tcr = self.finish_layer_new(extrude_outer_wall, false, false, wall_height, wall_reserved);
                    insert_finish_layer_idx = i as i32;
                }
            }

            // insert finish block
            if wall_idx != -1 {
                if layer.tool_changes.is_empty() {
                    finish_layer_tcr = self.finish_layer_new(extrude_outer_wall, false, false, wall_height, wall_reserved);
                }

                for bi in 0..self.wipe_tower_blocks.len() {
                    // The band is the trench of a merged slab printed on a layer above; nothing may go in here.
                    if self.wipe_tower_blocks[bi].layer_reserved[cur] != 0 {
                        let sd = self.wipe_tower_blocks[bi].start_depth;
                        self.wipe_tower_blocks[bi].finish_depth[cur] = sd;
                        continue;
                    }
                    let planned_depth = {
                        let block = &self.wipe_tower_blocks[bi];
                        block.start_depth + self.band_fill_depth(block, cur as i32)
                    };
                    {
                        let merged = self.merged_slabs_active;
                        let block = &mut self.wipe_tower_blocks[bi];
                        block.finish_depth[cur] = if merged {
                            // std::min(block.cur_depth, planned_depth)
                            if planned_depth < block.cur_depth { planned_depth } else { block.cur_depth }
                        } else {
                            block.start_depth + block.depth
                        };
                    }
                    if self.wipe_tower_blocks[bi].cur_depth as f64 + crate::libslic3r::EPSILON
                        >= (planned_depth - self.perimeter_width) as f64
                    {
                        continue;
                    }
                    let lt = self.wipe_tower_blocks[bi].layers_type[cur];
                    let mut block_solid = lt == WipeTowerLayerType::Contact
                        || lt == WipeTowerLayerType::ContactUp
                        || lt == WipeTowerLayerType::Solid;
                    let block_snapshot = self.wipe_tower_blocks[bi].clone();
                    let band_height = block_snapshot.layer_heights[cur];
                    let mut finish_layer_filament = self.get_block_fill_filament(&layer, cur as i32, &block_snapshot, wall_idx);
                    // No filament of this band's category visits the layer: a filament that does visit fills it.
                    if finish_layer_filament < 0 {
                        for tcr in &layer_result {
                            if self.filament_can_print_height(tcr.new_tool, band_height) {
                                finish_layer_filament = tcr.new_tool;
                                break;
                            }
                        }
                        if finish_layer_filament < 0 && wall_idx >= 0 {
                            finish_layer_filament = wall_idx;
                        }
                    }
                    if finish_layer_filament < 0 {
                        continue;
                    }
                    // Only a filament this layer's visits actually have loaded can lay the band down.
                    let can_carry_band = |this: &WipeTower,
                                          layer_result: &[ToolChangeResult],
                                          finish_layer_tcr: &ToolChangeResult,
                                          filament_id: i32|
                     -> bool {
                        if filament_id < 0 {
                            return false;
                        }
                        if layer_result.iter().any(|tcr| tcr.new_tool == filament_id) {
                            return true;
                        }
                        let block = &this.wipe_tower_blocks[bi];
                        let own_category = this.get_filament_category(filament_id as usize)
                            == Self::decode_block_category(block.block_key);
                        if (block.last_filament_change_id == -1 || own_category)
                            && layer_result.iter().any(|tcr| tcr.initial_tool == filament_id)
                        {
                            return true;
                        }
                        this.layer_finished() && finish_layer_tcr.new_tool == filament_id
                    };
                    if !can_carry_band(self, &layer_result, &finish_layer_tcr, finish_layer_filament) {
                        // A band stacked on, or under, another material does not bond to it.
                        let same_category_only = self.merged_fill_active();
                        let block_category = Self::decode_block_category(block_snapshot.block_key);
                        let may_fill = |this: &WipeTower, filament_id: i32| -> bool {
                            !same_category_only || this.get_filament_category(filament_id as usize) == block_category
                        };
                        let mut fallback = -1;
                        for tcr in &layer_result {
                            if self.filament_can_print_height(tcr.new_tool, band_height) && may_fill(self, tcr.new_tool) {
                                fallback = tcr.new_tool;
                                break;
                            }
                        }
                        if fallback < 0
                            && can_carry_band(self, &layer_result, &finish_layer_tcr, wall_idx)
                            && may_fill(self, wall_idx)
                        {
                            fallback = wall_idx;
                        }
                        if fallback < 0 && !layer_result.is_empty() && may_fill(self, layer_result.last().unwrap().new_tool) {
                            fallback = layer_result.last().unwrap().new_tool;
                        }
                        // Still nothing of this category. Fill with whoever is actually on the layer.
                        if fallback < 0 {
                            for tcr in &layer_result {
                                if self.filament_can_print_height(tcr.new_tool, band_height) {
                                    fallback = tcr.new_tool;
                                    break;
                                }
                            }
                            if fallback < 0 && can_carry_band(self, &layer_result, &finish_layer_tcr, wall_idx) {
                                fallback = wall_idx;
                            }
                            if fallback < 0 && !layer_result.is_empty() {
                                fallback = layer_result.last().unwrap().new_tool;
                            }
                            // Print the foreign fill dense so the band above still has a surface to bond to.
                            if fallback >= 0 {
                                block_solid = true;
                            }
                        }
                        // :7988-7990 — a warning is logged.
                        if fallback < 0 {
                            continue;
                        }
                        finish_layer_filament = fallback;
                    }
                    // :7994-7997 — a warning when the filament cannot print the band height.
                    // Cancel the block of the last layer
                    if !self.is_valid_last_layer(finish_layer_filament, cur as i32, layer.z as f64) {
                        continue;
                    }
                    let mut finish_block_tcr = if block_solid {
                        self.finish_block_solid(bi, finish_layer_filament, layer.extruder_fill, lt)
                    } else {
                        self.finish_block(bi, finish_layer_filament, layer.extruder_fill)
                    };
                    // A dense fill supports the complete footprint. A sparse grid does not.
                    if self.merged_slabs_active {
                        let dense_fill =
                            block_solid || (self.adhesion && self.is_first_layer_for_slab(self.wipe_tower_blocks[bi].layer_heights[cur]));
                        let block = &mut self.wipe_tower_blocks[bi];
                        block.finish_depth[cur] = if dense_fill {
                            planned_depth
                        } else if planned_depth < block.cur_depth {
                            planned_depth
                        } else {
                            block.cur_depth
                        };
                    } else {
                        let block = &mut self.wipe_tower_blocks[bi];
                        block.finish_depth[cur] = if block_solid { block.start_depth + block.depth } else { block.cur_depth };
                    }

                    let mut has_inserted = false;
                    if let Some(k) = layer_result.iter().position(|item| item.new_tool == finish_layer_filament) {
                        let merged = self.merge_tcr(&mut layer_result[k], &mut finish_block_tcr);
                        layer_result[k] = merged;
                        has_inserted = true;
                    }

                    let own_category_fill = self.get_filament_category(finish_layer_filament as usize)
                        == Self::decode_block_category(self.wipe_tower_blocks[bi].block_key);
                    if (self.wipe_tower_blocks[bi].last_filament_change_id == -1 || own_category_fill) && !has_inserted {
                        if let Some(k) = layer_result.iter().position(|item| item.initial_tool == finish_layer_filament) {
                            let merged = self.merge_tcr(&mut finish_block_tcr, &mut layer_result[k]);
                            layer_result[k] = merged;
                            has_inserted = true;
                        }
                    }

                    if !has_inserted && is_valid_gcode(&finish_block_tcr.gcode) {
                        if self.layer_finished() && finish_layer_tcr.new_tool == finish_layer_filament {
                            let merged = self.merge_tcr(&mut finish_layer_tcr, &mut finish_block_tcr);
                            finish_layer_tcr = merged;
                        }
                        // else :8045-8047 — an error is logged and the fill dropped.
                    }
                }
            }
            // record the contact layers of different categories
            if layer_result.is_empty() {
                // A visit that lays down nothing (no extrusions) is dropped unless only_generate_wall.
                if only_generate_wall || (is_valid_gcode(&finish_layer_tcr.gcode) && !finish_layer_tcr.extrusions.is_empty()) {
                    layer_result.push(std::mem::take(&mut finish_layer_tcr));
                }
            } else if is_valid_gcode(&finish_layer_tcr.gcode) {
                if insert_finish_layer_idx == -1 {
                    let merged = self.merge_tcr(&mut finish_layer_tcr, &mut layer_result[0]);
                    layer_result[0] = merged;
                } else {
                    let k = insert_finish_layer_idx as usize;
                    let merged = self.merge_tcr(&mut layer_result[k], &mut finish_layer_tcr);
                    layer_result[k] = merged;
                }
            }

            if only_generate_wall && !timelapse_wall.gcode.is_empty() {
                layer_result.insert(0, std::mem::take(&mut timelapse_wall));
            }

            // Slow down the last m_tail_slowdown_volume mm3 of every visit, only when hotends are handled as
            // extruders and never on an extrusion that sits on the bed.
            if self.handle_hotend_as_extruder && self.tail_speed_factor < 1.0 {
                let slow_visit = |this: &WipeTower, tcr: &mut ToolChangeResult| {
                    let tail_tool = if (tcr.new_tool as usize) < this.filament_params.len() {
                        tcr.new_tool as usize
                    } else {
                        this.current_tool
                    };
                    let area = if tail_tool < this.filament_params.len() { this.filament_params[tail_tool].filament_area } else { 0.0 };
                    apply_wipe_tower_tail_slowdown(
                        &mut tcr.gcode,
                        this.tail_slowdown_volume,
                        this.tail_speed_factor,
                        area,
                        Some(tcr.start_pos),
                        Some(tcr.tool_change_start_pos),
                    );
                };
                // Single-diameter: the nominal layer height is the extrusion height.
                if !self.has_multiple_nozzle_diameters() {
                    if !self.is_first_layer() {
                        for tcr in layer_result.iter_mut() {
                            slow_visit(self, tcr);
                        }
                    }
                } else {
                    // Multi-diameter: current Z minus this nozzle's own slab.
                    for tcr in layer_result.iter_mut() {
                        let tail_tool = if (tcr.new_tool as usize) < self.filament_params.len() {
                            tcr.new_tool as usize
                        } else {
                            self.current_tool
                        };
                        if self.is_first_layer_for_slab(self.filament_extrusion_height(tail_tool as i32)) {
                            continue;
                        }
                        slow_visit(self, tcr);
                    }
                }
            }

            result.push(std::mem::take(&mut layer_result));
        }
        // :8106 — assert(m_outer_wall.size() <= m_plan.size()) (compiled out).
    }
}


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
        assert!(tower.is_same_extruder(0, 0, 0));
        assert!(!tower.is_same_extruder(0, 1, 0));

        // Set custom map where filaments 0 and 2 share extruder 0
        tower.set_filament_map(vec![0, 1, 0, 1]);
        assert!(tower.is_same_extruder(0, 2, 0));
        assert!(tower.is_same_extruder(1, 3, 0));
        assert!(!tower.is_same_extruder(0, 1, 0));
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

