//! Print configuration types.
//!
//! This module provides the main configuration types for controlling
//! the slicing and printing process, mirroring BambuStudio's PrintConfig.

use crate::CoordF;
use serde::{Deserialize, Serialize};
use std::fmt;

/// G-code configuration subset.
/// This is used by GCodeWriter and contains only the fields it needs.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GCodeConfig {
    /// G-code flavor.
    pub gcode_flavor: GCodeFlavor,
    /// Travel move speed.
    pub travel_speed: CoordF,
    /// Tool change G-code.
    pub toolchange_gcode: String,
    /// Use relative E distances (default: false = absolute E).
    pub use_relative_e_distances: bool,
}

impl Default for GCodeConfig {
    fn default() -> Self {
        Self {
            gcode_flavor: GCodeFlavor::RepRapFirmware,
            travel_speed: 130.0,
            toolchange_gcode: String::new(),
            use_relative_e_distances: false,
        }
    }
}

/// Main print configuration containing global print settings.
///
/// This encompasses settings that affect the entire print, such as
/// bed size, general print speeds, and global parameters.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PrintConfig {
    // === Bed Configuration ===
    /// Bed size X (mm).
    pub bed_size_x: CoordF,
    /// Bed size Y (mm).
    pub bed_size_y: CoordF,
    /// Print origin X offset (mm).
    pub print_origin_x: CoordF,
    /// Print origin Y offset (mm).
    pub print_origin_y: CoordF,

    // === Layer Heights ===
    /// Default layer height (mm).
    pub layer_height: CoordF,
    /// First layer height (mm).
    pub first_layer_height: CoordF,

    // === Speeds (mm/s) ===
    /// Default print speed.
    pub print_speed: CoordF,
    /// Travel move speed.
    pub travel_speed: CoordF,
    /// First layer speed.
    pub first_layer_speed: CoordF,

    // === Temperatures ===
    /// Extruder temperature (°C).
    pub extruder_temperature: u32,
    /// First layer extruder temperature (°C).
    pub first_layer_extruder_temperature: u32,
    /// Bed temperature (°C).
    pub bed_temperature: u32,
    /// First layer bed temperature (°C).
    pub first_layer_bed_temperature: u32,
    /// Cool plate temperature (°C) — used for M190 in machine start sequence.
    pub cool_plate_temp: u32,

    // === Retraction ===
    /// Retraction length (mm).
    pub retract_length: CoordF,
    /// Retraction speed (mm/s).
    pub retract_speed: CoordF,
    /// Z lift on retraction (mm).
    pub retract_lift: CoordF,
    /// Minimum travel distance before retraction (mm).
    pub retract_before_travel: CoordF,
    /// Deretraction speed (mm/s). 0 means use retract_speed.
    pub deretract_speed: CoordF,
    /// Extra restart length after retraction (mm).
    pub retract_restart_extra: CoordF,
    /// Retraction length for tool changes (mm).
    pub retract_length_toolchange: CoordF,
    /// Extra restart length after tool-change retraction (mm).
    pub retract_restart_extra_toolchange: CoordF,
    /// Percentage of retraction before wipe (0-100).
    pub retract_before_wipe: CoordF,
    /// Filament density (g/cm³) — used for weight estimates.
    pub filament_density: CoordF,
    /// Filament cost (currency/kg) — used for cost estimates.
    pub filament_cost: CoordF,
    /// Filament flow ratio (extrusion multiplier, e.g. 0.98).
    pub filament_flow_ratio: CoordF,
    /// Per-filament override of `enable_overhang_speed`.
    /// C++: `filament_enable_overhang_speed` — nullable per-filament
    /// ConfigOptionBools cloned from "enable_overhang_speed"
    /// (PrintConfig.cpp:82-90 `filament_overhang_override_keys` +
    /// PrintConfig.cpp:6069-6090 `add_nullable`); default `{true}`
    /// (PrintConfig.cpp:1309). This crate models per-filament vectors as
    /// scalars, so `get_at(idx)` collapses onto a direct read.
    pub filament_enable_overhang_speed: bool,
    /// Per-filament override of `bridge_speed` (mm/s).
    /// C++: `filament_bridge_speed` — nullable per-filament
    /// ConfigOptionFloats cloned from "bridge_speed" (PrintConfig.cpp:82-90 +
    /// 6069-6090); default `{25}` (PrintConfig.cpp:1440). Scalar collapse as
    /// above.
    pub filament_bridge_speed: CoordF,

    // === Extrusion ===
    /// Nozzle diameter (mm).
    pub nozzle_diameter: CoordF,
    /// Filament diameter (mm).
    pub filament_diameter: CoordF,
    /// Per-filament colours (`filament_colour` array, e.g. `#00AE42`), one per
    /// configured filament. C++ derives the extruder count from the length of
    /// its per-filament vectors (PrintConfig.hpp filament_colour /
    /// filament_diameter); the Rust config keeps SCALAR fields for filament 0
    /// (the locked single-material paths) and carries the full arrays in these
    /// additive vectors for multi-material work. Empty = single filament.
    pub filament_colours: Vec<String>,
    /// Per-filament diameters (mm) — full `filament_diameter` array.
    /// `filament_diameter` (scalar) stays = element 0.
    pub filament_diameters: Vec<CoordF>,
    /// Per-filament densities (g/cm³) — full `filament_density` array.
    pub filament_densities: Vec<CoordF>,
    /// Extrusion multiplier (flow rate adjustment).
    pub extrusion_multiplier: CoordF,
    /// Line width of initial layer (mm). 0 = use the per-role widths.
    /// BambuStudio: `initial_layer_line_width` — lives on the PRINT config in
    /// C++ (PrintConfig.hpp:1411, coFloat) and is read by `PrintRegion::flow`
    /// (PrintRegion.cpp:27-28). The PrintObjectConfig carries a historical
    /// copy of the same JSON key; this is the faithful print-level home.
    pub initial_layer_line_width: CoordF,

    // === Skirt/Brim ===
    /// Number of skirt loops.
    pub skirt_loops: u32,
    /// Skirt distance from object (mm).
    pub skirt_distance: CoordF,
    /// Skirt minimum length (mm).
    pub skirt_min_length: CoordF,
    /// Brim width (mm), 0 = no brim.
    pub brim_width: CoordF,

    // === Raft ===
    /// Enable raft generation.
    pub raft_enabled: bool,
    /// Number of raft layers.
    pub raft_layers: u32,
    /// Raft expansion beyond the model (mm).
    pub raft_expansion: CoordF,
    /// Vertical gap between raft and model (mm).
    pub raft_contact_distance: CoordF,
    /// Base layer line spacing (mm).
    pub raft_first_layer_spacing: CoordF,
    /// Interface layer line spacing (mm).
    pub raft_interface_spacing: CoordF,
    /// Raft density (0.0 – 1.0).
    pub raft_density: CoordF,

    // === Support ===
    /// Enable support structures.
    pub support_enabled: bool,
    /// Support type (normal, tree).
    pub support_type: SupportType,
    /// Support overhang threshold angle (degrees).
    pub support_threshold_angle: CoordF,
    /// Support density (0.0 - 1.0).
    pub support_density: CoordF,

    // === Misc ===
    /// Enable spiral/vase mode.
    pub spiral_vase: bool,
    /// PrintConfig.hpp:1442: spiral_mode_smooth — interpolate XY with the previous
    /// layer so there is no seam at layer changes. Consumed by SpiralVase.
    pub spiral_mode_smooth: bool,
    /// PrintConfig.cpp:942: z_direction_outwall_speed_continuous — smooth the
    /// outer-wall speed along Z (LoopNode continuity + SmoothCalculator).
    /// Native default false; BBL profiles set 1.
    pub z_direction_outwall_speed_continuous: bool,
    /// G-code flavor.
    pub gcode_flavor: GCodeFlavor,
    /// Resolution for G-code output (mm).
    pub resolution: CoordF,

    // === Extrusion Mode ===
    /// Use relative extrusion mode (M83).
    /// When true, E values are relative (incremental) rather than absolute.
    /// Most modern printers and slicers use relative E mode (M83).
    /// BambuStudio uses relative E mode by default.
    pub use_relative_e: bool,

    // === Arc Fitting ===
    /// Enable arc fitting (G2/G3 commands).
    /// When enabled, sequences of line segments that form arcs will be
    /// converted to G2/G3 arc commands, reducing file size and improving
    /// print quality on firmware that supports arc moves.
    pub arc_fitting_enabled: bool,
    /// Arc fitting tolerance (mm).
    /// Maximum deviation allowed when fitting arcs to line segments.
    /// Smaller values produce more accurate arcs but fewer arc segments.
    pub arc_fitting_tolerance: CoordF,
    /// Minimum arc radius (mm).
    /// Arcs with smaller radii will be kept as line segments.
    pub arc_fitting_min_radius: CoordF,
    /// Maximum arc radius (mm).
    /// Very large radii are essentially straight lines and will be kept as line segments.
    pub arc_fitting_max_radius: CoordF,

    // === Z-Hop Configuration ===
    /// Z-hop type: Normal (G0), Auto, or Spiral (helical G3 arc).
    /// BambuStudio default for Bambu printers is Auto/Spiral.
    /// Reference: GCodeWriter.cpp `_spiral_travel_to_z()`
    pub z_hop_type: ZHopType,
    /// Radius of the spiral lift arc (mm). Default 0.8mm.
    /// BambuStudio computes this from current XY position and a fixed radius.
    pub spiral_lift_radius: CoordF,

    // === Travel Optimization ===
    /// Enable avoid crossing perimeters.
    /// When enabled, travel moves will be routed around perimeter walls
    /// to avoid leaving marks on the printed surface.
    pub avoid_crossing_perimeters: bool,
    /// Maximum detour percentage for avoid crossing perimeters.
    /// If the detour path is longer than this percentage of the direct path,
    /// the direct path will be used instead. Default is 200% (2x direct distance).
    pub avoid_crossing_max_detour: CoordF,
    /// Maximum volumetric extrusion rate slope (positive).
    /// Used for pressure equalization (linear advance).
    pub max_volumetric_extrusion_rate_slope_positive: CoordF,
    /// Maximum volumetric extrusion rate slope (negative).
    /// Used for pressure equalization (linear advance).
    pub max_volumetric_extrusion_rate_slope_negative: CoordF,

    // === Cooling / Fan Control ===
    /// Enable slow down for layer cooling.
    pub slow_down_for_layer_cooling: bool,
    /// Minimum layer time (seconds). Layers faster than this will be slowed.
    pub slow_down_layer_time: f64,
    /// Minimum print speed when slowing for cooling (mm/s).
    pub slow_down_min_speed: f64,
    /// Minimum fan speed (0-100%).
    pub fan_min_speed: i32,
    /// Maximum fan speed (0-100%).
    pub fan_max_speed: i32,
    /// Layer time threshold for fan interpolation (seconds).
    pub fan_cooling_layer_time: f64,
    /// Disable fan for first N layers.
    pub close_fan_the_first_x_layers: usize,

    // === Per-Extruder Cooling (Vec fields for multi-extruder) ===
    /// Per-extruder cooling configurations. If empty, a single default config is created from scalar fields above.
    pub per_extruder_cooling: Vec<PerExtruderCoolingConfig>,
    /// Whether to use proportional cooling slowdown logic.
    pub cooling_logic_proportional: bool,
    /// Whether the printer has an auxiliary fan (M106 P2).
    pub auxiliary_fan: bool,
    /// Toolchange prefix string (e.g. "T").
    pub toolchange_prefix: String,
    /// Use relative E distances (cooling buffer needs this).
    pub use_relative_e_distances_cooling: bool,

    // === Print Sequence ===
    /// Print sequence: by layer (default) or by object.
    /// BambuStudio: `print_sequence`.
    pub print_sequence: PrintSequence,

    // === Acceleration (mm/s²) ===
    /// Default acceleration for printing moves.
    /// BambuStudio: `default_acceleration`.
    pub default_acceleration: CoordF,
    /// Outer wall acceleration.
    /// BambuStudio: `outer_wall_acceleration`.
    pub outer_wall_acceleration: CoordF,
    /// Inner wall acceleration.
    /// BambuStudio: `inner_wall_acceleration`.
    pub inner_wall_acceleration: CoordF,
    /// Top surface acceleration.
    /// BambuStudio: `top_surface_acceleration`.
    pub top_surface_acceleration: CoordF,
    /// Sparse infill acceleration.
    /// BambuStudio: `sparse_infill_acceleration`.
    pub sparse_infill_acceleration: CoordF,
    /// Initial layer acceleration.
    /// BambuStudio: `initial_layer_acceleration`.
    pub initial_layer_acceleration: CoordF,
    /// Travel acceleration.
    /// BambuStudio: `travel_acceleration`.
    pub travel_acceleration: CoordF,
    /// BambuStudio: `travel_short_distance_acceleration` — used for travels
    /// shorter than retraction_minimum_travel to/within outer walls
    /// (GCode.cpp:6880-6887).
    pub travel_short_distance_acceleration: CoordF,
    /// Initial layer travel acceleration.
    /// BambuStudio: `initial_layer_travel_acceleration`.
    pub initial_layer_travel_acceleration: CoordF,

    // === Jerk (mm/s) ===
    /// Default jerk limit.
    /// BambuStudio: `default_jerk`.
    pub default_jerk: CoordF,
    /// Outer wall jerk.
    /// BambuStudio: `outer_wall_jerk`.
    pub outer_wall_jerk: CoordF,
    /// Inner wall jerk.
    /// BambuStudio: `inner_wall_jerk`.
    pub inner_wall_jerk: CoordF,
    /// Top surface jerk.
    /// BambuStudio: `top_surface_jerk`.
    pub top_surface_jerk: CoordF,
    /// Infill jerk.
    /// BambuStudio: `infill_jerk`.
    pub infill_jerk: CoordF,
    /// Initial layer jerk.
    /// BambuStudio: `initial_layer_jerk`.
    pub initial_layer_jerk: CoordF,
    /// Travel jerk.
    /// BambuStudio: `travel_jerk`.
    pub travel_jerk: CoordF,

    // === Speeds (additional) ===
    /// Outer wall speed (mm/s).
    /// BambuStudio: `outer_wall_speed`.
    pub outer_wall_speed: CoordF,
    /// Inner wall speed (mm/s).
    /// BambuStudio: `inner_wall_speed`.
    pub inner_wall_speed: CoordF,
    /// Sparse infill speed (mm/s).
    /// BambuStudio: `sparse_infill_speed`.
    pub sparse_infill_speed: CoordF,
    /// Internal solid infill speed (mm/s).
    /// BambuStudio: `internal_solid_infill_speed`.
    pub internal_solid_infill_speed: CoordF,
    /// Top surface speed (mm/s).
    /// BambuStudio: `top_surface_speed`.
    pub top_surface_speed: CoordF,
    /// Bridge speed (mm/s).
    /// BambuStudio: `bridge_speed`.
    pub bridge_speed: CoordF,
    /// Gap infill speed (mm/s).
    /// BambuStudio: `gap_infill_speed`.
    pub gap_infill_speed: CoordF,
    /// Support speed (mm/s).
    /// BambuStudio: `support_speed`.
    pub support_speed: CoordF,
    /// Support interface speed (mm/s).
    /// BambuStudio: `support_interface_speed`.
    pub support_interface_speed: CoordF,
    /// Initial layer infill speed (mm/s).
    /// BambuStudio: `initial_layer_infill_speed`.
    pub initial_layer_infill_speed: CoordF,
    /// Small perimeter speed (mm/s).
    /// BambuStudio: `small_perimeter_speed`.
    pub small_perimeter_speed: CoordF,
    /// Travel speed for Z moves (mm/s).
    /// BambuStudio: `travel_speed_z`.
    pub travel_speed_z: CoordF,

    // === Filament Settings (per-filament, first extruder) ===
    /// Filament max volumetric speed (mm³/s). 0 = unlimited.
    /// BambuStudio: `filament_max_volumetric_speed`.
    pub filament_max_volumetric_speed: CoordF,
    /// Filament type string (e.g. "PLA", "ABS", "TPU").
    /// BambuStudio: `filament_type`.
    pub filament_type: String,
    /// Filament retraction length (mm). Per-filament override.
    /// BambuStudio: `filament_retraction_length`.
    pub filament_retraction_length: CoordF,
    /// Filament retraction speed (mm/s). Per-filament override.
    /// BambuStudio: `filament_retraction_speed`.
    pub filament_retraction_speed: CoordF,
    /// Filament deretraction speed (mm/s). Per-filament override.
    /// BambuStudio: `filament_deretraction_speed`.
    pub filament_deretraction_speed: CoordF,
    /// Filament z-hop height (mm). Per-filament override.
    /// BambuStudio: `filament_z_hop`.
    pub filament_z_hop: CoordF,
    /// Filament wipe distance (mm). Per-filament override.
    /// BambuStudio: `filament_wipe_distance`.
    pub filament_wipe_distance: CoordF,
    /// Filament retraction minimum travel (mm). Per-filament override.
    /// BambuStudio: `filament_retraction_minimum_travel`.
    pub filament_retraction_minimum_travel: CoordF,
    /// Whether to retract when changing layer.
    /// BambuStudio: `filament_retract_when_changing_layer`.
    pub filament_retract_when_changing_layer: bool,
    /// Filament start G-code.
    /// BambuStudio: `filament_start_gcode`.
    pub filament_start_gcode: String,
    /// Filament end G-code.
    /// BambuStudio: `filament_end_gcode`.
    pub filament_end_gcode: String,

    // === Machine G-code ===
    /// Machine start G-code template.
    /// BambuStudio: `machine_start_gcode`.
    pub machine_start_gcode: String,
    /// Machine end G-code template.
    /// BambuStudio: `machine_end_gcode`.
    pub machine_end_gcode: String,
    /// Before layer change G-code.
    /// BambuStudio: `before_layer_change_gcode`.
    pub before_layer_change_gcode: String,
    /// Layer change G-code.
    /// BambuStudio: `layer_change_gcode`.
    pub layer_change_gcode: String,
    /// Change filament G-code.
    /// BambuStudio: `change_filament_gcode`.
    pub change_filament_gcode: String,
    /// Tool change G-code.
    /// BambuStudio: `tool_change_gcode`.
    pub tool_change_gcode: String,
    /// Machine pause G-code.
    /// BambuStudio: `machine_pause_gcode`.
    pub machine_pause_gcode: String,
    /// Printing by object G-code (between objects).
    /// BambuStudio: `printing_by_object_gcode`.
    pub printing_by_object_gcode: String,

    // === Timelapse ===
    /// Enable timelapse.
    /// BambuStudio: `enable_timelapse`.
    pub enable_timelapse: bool,
    /// Timelapse type.
    /// BambuStudio: `timelapse_type`.
    pub timelapse_type: u32,

    // === Machine Limits ===
    /// Max acceleration X (mm/s²).
    pub machine_max_acceleration_x: CoordF,
    /// Max acceleration Y (mm/s²).
    pub machine_max_acceleration_y: CoordF,
    /// Max acceleration Z (mm/s²).
    pub machine_max_acceleration_z: CoordF,
    /// Max acceleration E (mm/s²).
    pub machine_max_acceleration_e: CoordF,
    /// Max acceleration for extruding moves (mm/s²).
    pub machine_max_acceleration_extruding: CoordF,
    /// Max acceleration for retracting moves (mm/s²).
    pub machine_max_acceleration_retracting: CoordF,
    /// Max acceleration for travel moves (mm/s²).
    pub machine_max_acceleration_travel: CoordF,
    /// Max speed X (mm/s).
    pub machine_max_speed_x: CoordF,
    /// Max speed Y (mm/s).
    pub machine_max_speed_y: CoordF,
    /// Max speed Z (mm/s).
    pub machine_max_speed_z: CoordF,
    /// Max speed E (mm/s).
    pub machine_max_speed_e: CoordF,
    /// Max jerk X (mm/s).
    pub machine_max_jerk_x: CoordF,
    /// Max jerk Y (mm/s).
    pub machine_max_jerk_y: CoordF,
    /// Max jerk Z (mm/s).
    pub machine_max_jerk_z: CoordF,
    /// Max jerk E (mm/s).
    pub machine_max_jerk_e: CoordF,
    /// Minimum extruding rate (mm/s).
    pub machine_min_extruding_rate: CoordF,
    /// Minimum travel rate (mm/s).
    pub machine_min_travel_rate: CoordF,

    // === Bed Temperature Variants ===
    /// Engineering plate temperature (°C).
    pub eng_plate_temp: u32,
    /// Engineering plate initial layer temperature (°C).
    pub eng_plate_temp_initial_layer: u32,
    /// Hot plate temperature (°C).
    pub hot_plate_temp: u32,
    /// Hot plate initial layer temperature (°C).
    pub hot_plate_temp_initial_layer: u32,
    /// Cool plate initial layer temperature (°C).
    pub cool_plate_temp_initial_layer: u32,
    /// Textured plate temperature (°C).
    pub textured_plate_temp: u32,
    /// Textured plate initial layer temperature (°C).
    pub textured_plate_temp_initial_layer: u32,
    /// Current bed type string (for selecting which temp to use).
    /// BambuStudio: `curr_bed_type`.
    pub curr_bed_type: String,

    // === Nozzle Temperature ===
    /// Nozzle temperature (per-filament, °C).
    /// BambuStudio: `nozzle_temperature`.
    pub nozzle_temperature: u32,
    /// Nozzle temperature for initial layer (per-filament, °C).
    /// BambuStudio: `nozzle_temperature_initial_layer`.
    pub nozzle_temperature_initial_layer: u32,
    /// High end of the nozzle temperature range (per-filament, °C). Used as the
    /// change_filament flush temperature when `filament_flush_temp` is 0.
    /// BambuStudio: `nozzle_temperature_range_high`.
    pub nozzle_temperature_range_high: u32,
    /// Chamber temperature (°C). 0 = no heated chamber.
    /// BambuStudio: `chamber_temperatures`.
    pub chamber_temperature: u32,

    // === Long Retraction / Cut ===
    /// Enable long retraction when cut (filament cutter).
    /// BambuStudio: `enable_long_retraction_when_cut`.
    pub enable_long_retraction_when_cut: bool,
    /// Retraction distance when cut (mm).
    /// BambuStudio: `retraction_distances_when_cut`.
    pub retraction_distances_when_cut: CoordF,

    // === Wipe Tower / Prime Tower ===
    /// Enable prime/wipe tower for multi-material.
    /// BambuStudio: `enable_prime_tower`.
    pub enable_prime_tower: bool,
    /// Prime tower width (mm).
    /// BambuStudio: `prime_tower_width`.
    pub prime_tower_width: CoordF,

    // === Multi-material ===
    /// Flush into infill to reduce waste.
    /// BambuStudio: `flush_into_infill`.
    pub flush_into_infill: bool,
    /// Flush into objects to reduce waste.
    /// BambuStudio: `flush_into_objects`.
    pub flush_into_objects: bool,
    /// Flush into support to reduce waste.
    /// BambuStudio: `flush_into_support`.
    pub flush_into_support: bool,

    // === Pressure Advance ===
    /// Enable pressure advance.
    /// BambuStudio: `enable_pressure_advance`.
    pub enable_pressure_advance: bool,
    /// Pressure advance value.
    /// BambuStudio: `pressure_advance`.
    pub pressure_advance: CoordF,

    // === Printable Height ===
    /// Maximum printable height (mm). Z-axis limit.
    /// BambuStudio: `printable_height`.
    pub printable_height: CoordF,

    // === Misc Machine ===
    /// Extruder clearance height to rod (mm) for by-object printing.
    /// BambuStudio: `extruder_clearance_height_to_rod`.
    pub extruder_clearance_height_to_rod: CoordF,
    /// Extruder clearance height to lid (mm) for by-object printing.
    /// BambuStudio: `extruder_clearance_height_to_lid`.
    pub extruder_clearance_height_to_lid: CoordF,
    /// Extruder clearance max radius (mm) for by-object printing.
    /// BambuStudio: `extruder_clearance_max_radius`.
    pub extruder_clearance_max_radius: CoordF,

    // === Filename Format ===
    /// Output filename format template.
    /// BambuStudio: `filename_format`.
    pub filename_format: String,

    // === Silent Mode ===
    /// Enable silent mode (reduced stepper noise).
    /// BambuStudio: `enable_silent`.
    pub enable_silent: bool,

    // === Accel-to-Decel ===
    /// Enable accel-to-decel factor (Klipper-style).
    /// BambuStudio: `accel_to_decel_enable`.
    pub accel_to_decel_enable: bool,
    /// Accel-to-decel factor (percentage).
    /// BambuStudio: `accel_to_decel_factor`.
    pub accel_to_decel_factor: CoordF,

    // === Exclude Object ===
    /// Enable exclude object (M486/EXCLUDE_OBJECT).
    /// BambuStudio: `exclude_object`.
    pub exclude_object: bool,

    // === G-code Features ===
    /// Add line numbers to G-code.
    /// BambuStudio: `gcode_add_line_number`.
    pub gcode_add_line_number: bool,
    /// Use firmware retraction (G10/G11).
    /// BambuStudio: `use_firmware_retraction`.
    pub use_firmware_retraction: bool,

    // === Reduce Crossing Wall ===
    /// Reduce crossing wall (avoid crossing perimeters).
    /// BambuStudio: `reduce_crossing_wall`.
    pub reduce_crossing_wall: bool,
    /// Max travel detour distance.
    /// BambuStudio: `max_travel_detour_distance`.
    pub max_travel_detour_distance: CoordF,

    // === Max Print Speed ===
    /// Maximum print speed overall (mm/s). 0 = unlimited.
    /// BambuStudio: `max_print_speed`.
    pub max_print_speed: CoordF,

    // === Max Volumetric Speed ===
    /// Maximum volumetric extrusion speed (mm³/s). 0 = unlimited.
    /// BambuStudio: `max_volumetric_speed`.
    pub max_volumetric_speed: CoordF,

    // === Wall Sequence ===
    /// Wall sequence (inner/outer ordering).
    /// BambuStudio: `wall_sequence`.
    pub wall_sequence: WallSequence,

    // === Timelapse ===
    /// Time-lapse G-code template.
    /// BambuStudio: `time_lapse_gcode`.
    pub time_lapse_gcode: String,

    // === Scan First Layer ===
    /// Scan first layer (BBL printer feature).
    /// BambuStudio: `scan_first_layer`.
    pub scan_first_layer: bool,

    // === Multi-Material (additional) ===
    /// Single extruder multi-material mode.
    /// BambuStudio: `single_extruder_multi_material`.
    pub single_extruder_multi_material: bool,
    /// Support filament (1-based extruder index).
    /// BambuStudio: `support_filament`.
    pub support_filament: u32,
    /// Support interface filament (1-based extruder index).
    /// BambuStudio: `support_interface_filament`.
    pub support_interface_filament: u32,
    /// Flush volumes matrix (multi-material transition volumes).
    /// BambuStudio: `flush_volumes_matrix`.
    pub flush_volumes_matrix: Vec<f64>,
    /// Per-filament prime volume (mm³) — drives the wipe-tower reserved DEPTH
    /// (`WipeTower::plan_toolchange` wipe_volume_ec, Print.cpp:3320). NOT the
    /// flush volume (which goes into the object / stored purge). BambuStudio:
    /// `filament_prime_volume`.
    pub filament_prime_volumes: Vec<f64>,
    /// Per-filament prime volume for nozzle changes. BambuStudio:
    /// `filament_prime_volume_nc`.
    pub filament_prime_volumes_nc: Vec<f64>,

    // === Filament (additional) ===
    /// Whether filament is support material.
    /// BambuStudio: `filament_is_support`.
    pub filament_is_support: bool,
    /// Whether filament is soluble.
    /// BambuStudio: `filament_soluble`.
    pub filament_soluble: bool,
    /// Retract when changing layer.
    /// BambuStudio: `retract_when_changing_layer`.
    pub retract_when_changing_layer: bool,

    // === Wipe Tower Position ===
    /// Wipe tower X position (mm).
    /// BambuStudio: `wipe_tower_x`.
    pub wipe_tower_x: CoordF,
    /// Wipe tower Y position (mm).
    /// BambuStudio: `wipe_tower_y`.
    pub wipe_tower_y: CoordF,

    // === Wrapping Detection ===
    /// Enable wrapping detection (BBL feature).
    /// BambuStudio: `enable_wrapping_detection`.
    pub enable_wrapping_detection: bool,
    /// Wrapping detection G-code.
    /// BambuStudio: `wrapping_detection_gcode`.
    pub wrapping_detection_gcode: String,

    // === Bed Temperature Formula ===
    /// Bed temperature formula mode.
    /// BambuStudio: `bed_temperature_formula`.
    pub bed_temperature_formula: BedTempFormula,

    // === Extruder Offset ===
    /// Extruder X offset (mm). Subtracted from all G-code X coordinates.
    /// C++ reference: GCode.cpp:7089 `point_to_gcode()` subtracts extruder_offset.
    /// BambuStudio: `extruder_offset` format is "XxY" e.g. "0x2".
    pub extruder_offset_x: CoordF,
    /// Extruder Y offset (mm). Subtracted from all G-code Y coordinates.
    pub extruder_offset_y: CoordF,
}

/// Per-extruder cooling configuration.
/// Mirrors all fields from the C++ EXTRUDER_CONFIG macro block
/// (GCodeEditor.cpp:393-460).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PerExtruderCoolingConfig {
    /// Minimum fan speed (0-100).
    pub fan_min_speed: i32,
    /// Maximum fan speed (0-100).
    pub fan_max_speed: i32,
    /// Enable slow down for layer cooling.
    pub slow_down_for_layer_cooling: bool,
    /// Minimum layer time for slowdown (seconds).
    pub slow_down_layer_time: f32,
    /// Minimum print speed when slowing (mm/s).
    pub slow_down_min_speed: f32,
    /// Fan cooling layer time threshold (seconds). Below slow_down_layer_time => max fan.
    /// Between slow_down_layer_time and fan_cooling_layer_time => interpolated fan.
    pub fan_cooling_layer_time: f32,
    /// Disable fan for first N layers.
    pub close_fan_the_first_x_layers: i32,
    /// Layer at which fan reaches full speed (for ramp). 0 = disabled.
    pub full_fan_speed_layer: i32,
    /// Overhang fan speed (0-100).
    pub overhang_fan_speed: i32,
    /// If true, fan starts at fan_min_speed instead of 0 to reduce start/stop freq.
    pub reduce_fan_stop_start_freq: bool,
    /// Additional (auxiliary) cooling fan speed (0-100).
    pub additional_cooling_fan_speed: i32,
    /// Fan speed for first X layers (0-100).
    pub first_x_layer_fan_speed: i32,
    /// Pre-start time for overhang fan (seconds). Look ahead this many seconds.
    pub pre_start_fan_time: f32,
    /// Don't slow down external perimeters (outer walls) for cooling.
    pub no_slow_down_for_cooling_on_outwalls: bool,
    /// Cooling slowdown logic: 0 = UniformCooling, 1 = ConsistentSurface.
    pub cooling_slowdown_logic: i32,
    /// Perimeter transition distance for ConsistentSurface (mm).
    pub cooling_perimeter_transition_distance: f32,
}

impl Default for PerExtruderCoolingConfig {
    fn default() -> Self {
        Self {
            fan_min_speed: 100,
            fan_max_speed: 100,
            slow_down_for_layer_cooling: true,
            slow_down_layer_time: 4.0,
            slow_down_min_speed: 20.0,
            fan_cooling_layer_time: 100.0,
            close_fan_the_first_x_layers: 1,
            full_fan_speed_layer: 0,
            overhang_fan_speed: 100,
            reduce_fan_stop_start_freq: false,
            additional_cooling_fan_speed: 0,
            first_x_layer_fan_speed: 0,
            pre_start_fan_time: 0.0,
            no_slow_down_for_cooling_on_outwalls: false,
            cooling_slowdown_logic: 0,
            cooling_perimeter_transition_distance: 5.0,
        }
    }
}

impl PrintConfig {
    // Create a new PrintConfig with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Number of configured filaments/extruder slots.
    ///
    /// C++ derives this from the per-filament vector lengths
    /// (`filament_diameter.size()`, PrintConfig.hpp). The Rust config's
    /// per-filament vectors are empty for single-material configs (scalar
    /// fields = filament 0), so empty ⇒ 1.
    pub fn num_filaments(&self) -> usize {
        self.filament_diameters
            .len()
            .max(self.filament_colours.len())
            .max(1)
    }

    /// Apply all settings from another config, overwriting this one.
    /// Port of C++ DynamicPrintConfig::apply().
    pub fn apply_from(&mut self, other: &PrintConfig) {
        *self = other.clone();
    }

    /// Builder method: set layer height.
    pub fn layer_height(mut self, height: CoordF) -> Self {
        self.layer_height = height;
        self
    }

    /// Builder method: set first layer height.
    pub fn first_layer_height(mut self, height: CoordF) -> Self {
        self.first_layer_height = height;
        self
    }

    /// Builder method: set nozzle diameter.
    pub fn nozzle_diameter(mut self, diameter: CoordF) -> Self {
        self.nozzle_diameter = diameter;
        self
    }

    /// Builder method: set print speed.
    pub fn print_speed(mut self, speed: CoordF) -> Self {
        self.print_speed = speed;
        self
    }

    /// Builder method: enable/disable support.
    pub fn support(mut self, enabled: bool) -> Self {
        self.support_enabled = enabled;
        self
    }

    /// Builder method: set support type.
    pub fn support_type(mut self, support_type: SupportType) -> Self {
        self.support_type = support_type;
        self
    }

    /// Builder method: set brim width.
    pub fn brim_width(mut self, width: CoordF) -> Self {
        self.brim_width = width;
        self
    }

    /// Builder method: enable or disable raft.
    pub fn raft_enabled(mut self, enabled: bool) -> Self {
        self.raft_enabled = enabled;
        self
    }

    /// Builder method: set raft layers.
    pub fn raft_layers(mut self, layers: u32) -> Self {
        self.raft_layers = layers;
        if layers > 0 {
            self.raft_enabled = true;
        }
        self
    }

    /// Builder method: set raft expansion.
    pub fn raft_expansion(mut self, expansion: CoordF) -> Self {
        self.raft_expansion = expansion.max(0.0);
        self
    }

    /// Builder method: set raft contact distance.
    pub fn raft_contact_distance(mut self, distance: CoordF) -> Self {
        self.raft_contact_distance = distance.max(0.0);
        self
    }

    /// Builder method: set raft density.
    pub fn raft_density(mut self, density: CoordF) -> Self {
        self.raft_density = density.clamp(0.0, 1.0);
        self
    }

    /// Validate the configuration with min/max bounds from C++ PrintConfig.
    // PrintConfig.cpp:8977  std::map<std::string,std::string> validate(const FullPrintConfig &cfg, bool under_cli)
    // FIDELITY-NOTE: the C++ free `validate` operates on a FullPrintConfig / DynamicConfig
    // (ConfigOptionDef registry, get_abs_value, serialize, per-extruder *vectors*, has_enum_value)
    // none of which exist in this scalar struct model. The thresholds for the fields that DO
    // map 1:1 are mirrored here exactly; the original signature (Result<(),String> returning the
    // first error rather than a key->message map) is preserved because there is no
    // DynamicConfig/error-map model in this crate. The vector-valued C++ checks collapse to a
    // single scalar check (one nozzle / one filament).
    pub fn validate(&self) -> Result<(), String> {
        // PrintConfig.cpp:8981  --layer-height: <= 0 invalid
        if self.layer_height <= 0.0 {
            return Err(format!("invalid value {}", self.layer_height));
        }
        // PrintConfig.cpp:8984  else if (fabs(fmod(layer_height, SCALING_FACTOR)) > 1e-4) invalid
        else if (self.layer_height % crate::libslic3r::SCALING_FACTOR).abs() > 1e-4 {
            return Err(format!("invalid value {}", self.layer_height));
        }
        // PrintConfig.cpp:8989  --first-layer-height: initial_layer_print_height <= 0 invalid
        if self.first_layer_height <= 0.0 {
            return Err(format!("invalid value {}", self.first_layer_height));
        }
        // PrintConfig.cpp:8994  --filament-diameter: fd < 1 invalid
        if self.filament_diameter < 1.0 {
            return Err(format!("invalid value {}", self.filament_diameter));
        }
        // PrintConfig.cpp:9001  --nozzle-diameter: nd < 0.005 invalid
        if self.nozzle_diameter < 0.005 {
            return Err(format!("invalid value {}", self.nozzle_diameter));
        }
        // PrintConfig.cpp:9080  extruder clearance: <= 0 invalid
        if self.extruder_clearance_max_radius <= 0.0 {
            return Err(format!("invalid value {}", self.extruder_clearance_max_radius));
        }
        // PrintConfig.cpp:9083
        if self.extruder_clearance_height_to_rod <= 0.0 {
            return Err(format!("invalid value {}", self.extruder_clearance_height_to_rod));
        }
        // PrintConfig.cpp:9086
        if self.extruder_clearance_height_to_lid <= 0.0 {
            return Err(format!("invalid value {}", self.extruder_clearance_height_to_lid));
        }
        // PrintConfig.cpp:9093  --extrusion-multiplier: filament_flow_ratio em <= 0 invalid
        if self.filament_flow_ratio <= 0.0 {
            return Err(format!("invalid value {}", self.filament_flow_ratio));
        }
        Ok(())
    }
}

impl PrintConfig {
    // Builder: set relative extrusion mode (M83).
    pub fn use_relative_e(mut self, relative: bool) -> Self {
        self.use_relative_e = relative;
        self
    }

    pub fn arc_fitting(mut self, enabled: bool) -> Self {
        self.arc_fitting_enabled = enabled;
        self
    }

    pub fn arc_fitting_tolerance(mut self, tolerance: CoordF) -> Self {
        self.arc_fitting_tolerance = tolerance;
        self
    }

    pub fn arc_fitting_radius(mut self, min: CoordF, max: CoordF) -> Self {
        self.arc_fitting_min_radius = min;
        self.arc_fitting_max_radius = max;
        self
    }

    /// Enable or disable avoid crossing perimeters.
    pub fn avoid_crossing_perimeters(mut self, enabled: bool) -> Self {
        self.avoid_crossing_perimeters = enabled;
        self
    }

    /// Set the maximum detour percentage for avoid crossing perimeters.
    pub fn avoid_crossing_max_detour(mut self, percent: CoordF) -> Self {
        self.avoid_crossing_max_detour = percent;
        self
    }
}

impl Default for PrintConfig {
    fn default() -> Self {
        Self {
            // Bed
            bed_size_x: 256.0,
            bed_size_y: 256.0,
            print_origin_x: 0.0,
            print_origin_y: 0.0,

            // Layer heights
            layer_height: 0.2,
            first_layer_height: 0.2,

            // Speeds (Bambu H2D + PLA Basic reference)
            print_speed: 200.0,
            travel_speed: 1000.0,
            first_layer_speed: 50.0,

            // Temperatures (Bambu H2D + PLA Basic reference)
            extruder_temperature: 220,
            first_layer_extruder_temperature: 220,
            bed_temperature: 60,
            first_layer_bed_temperature: 60,
            cool_plate_temp: 35,

            // Retraction
            retract_length: 0.4, // Match BambuStudio H2D + PLA Basic (filament_retraction_length = 0.4)
            retract_speed: 30.0,
            retract_lift: 0.4,
            retract_before_travel: 2.0,
            deretract_speed: 0.0,
            retract_restart_extra: 0.0,
            retract_length_toolchange: 10.0,
            retract_restart_extra_toolchange: 0.0,
            retract_before_wipe: 0.0,
            filament_density: 1.24,
            filament_cost: 0.0,
            filament_flow_ratio: 1.0,
            // PrintConfig.cpp:1309 ConfigOptionBoolsNullable{ true } (cloned
            // into filament_enable_overhang_speed, PrintConfig.cpp:6069-6090)
            filament_enable_overhang_speed: true,
            // PrintConfig.cpp:1440 ConfigOptionFloatsNullable{ 25 } (cloned
            // into filament_bridge_speed)
            filament_bridge_speed: 25.0,

            // Extrusion
            nozzle_diameter: 0.4,
            filament_diameter: 1.75,
            // Empty = single filament (scalar fields are filament 0).
            filament_colours: Vec::new(),
            filament_diameters: Vec::new(),
            filament_densities: Vec::new(),
            extrusion_multiplier: 1.0,
            // PrintConfig.cpp:3004-3011 default ConfigOptionFloat(0.4)
            initial_layer_line_width: 0.4,

            // Skirt/Brim
            skirt_loops: 0, // Default to no skirt (opt-in); reference 3DBenchy has skirt_loops=0
            skirt_distance: 6.0,
            skirt_min_length: 0.0,
            brim_width: 0.0,

            // Raft
            raft_enabled: false,
            raft_layers: 0,
            raft_expansion: 3.0,
            raft_contact_distance: 0.15,
            raft_first_layer_spacing: 0.8,
            raft_interface_spacing: 0.4,
            raft_density: 1.0,

            // Support
            support_enabled: false,
            support_type: SupportType::Normal,
            support_threshold_angle: 45.0,
            support_density: 0.15,

            // Misc
            spiral_vase: false,
            z_direction_outwall_speed_continuous: false,
            spiral_mode_smooth: false,
            gcode_flavor: GCodeFlavor::Marlin,
            resolution: 0.01, // Match C++ BambuStudio

            // Extrusion Mode
            use_relative_e: true, // Match BambuStudio default (M83)

            // FINAL CONFIG: 12μ tolerance, 0.8mm min - best result (735 vs 761 ref)
            arc_fitting_enabled: true,
            arc_fitting_tolerance: 0.012,
            arc_fitting_min_radius: 0.8,
            arc_fitting_max_radius: 50.0,

            // Z-Hop
            z_hop_type: ZHopType::Auto, // Match BambuStudio default for Bambu printers
            spiral_lift_radius: 0.8,    // 0.8mm radius for spiral lift arc

            // Travel Optimization
            avoid_crossing_perimeters: true, // Enabled by default (matches BambuStudio)
            avoid_crossing_max_detour: 200.0, // 200% = 2x direct distance

            // Pressure Equalizer (Linear Advance smoothness)
            // Defaults to 0.0 (disabled)
            max_volumetric_extrusion_rate_slope_positive: 0.0,
            max_volumetric_extrusion_rate_slope_negative: 0.0,

            // Cooling defaults
            slow_down_for_layer_cooling: true,
            slow_down_layer_time: 4.0,
            slow_down_min_speed: 20.0,
            fan_min_speed: 100,
            fan_max_speed: 100,
            fan_cooling_layer_time: 100.0,
            close_fan_the_first_x_layers: 1,

            // Per-extruder cooling (empty = derive from scalar fields above)
            per_extruder_cooling: Vec::new(),
            cooling_logic_proportional: false,
            auxiliary_fan: false,
            toolchange_prefix: "T".to_string(),
            use_relative_e_distances_cooling: true,

            // Print sequence
            print_sequence: PrintSequence::ByLayer,

            // Acceleration (mm/s²) - BambuStudio X1C defaults
            default_acceleration: 10000.0,
            outer_wall_acceleration: 5000.0,
            inner_wall_acceleration: 10000.0,
            top_surface_acceleration: 5000.0,
            sparse_infill_acceleration: 10000.0,
            initial_layer_acceleration: 500.0,
            travel_acceleration: 12000.0,
            travel_short_distance_acceleration: 0.0,
            initial_layer_travel_acceleration: 9000.0,

            // Jerk (mm/s) - BambuStudio X1C defaults
            default_jerk: 9.0,
            outer_wall_jerk: 9.0,
            inner_wall_jerk: 9.0,
            top_surface_jerk: 9.0,
            infill_jerk: 9.0,
            initial_layer_jerk: 9.0,
            travel_jerk: 12.0,

            // Additional speeds (mm/s)
            outer_wall_speed: 200.0,
            inner_wall_speed: 300.0,
            sparse_infill_speed: 270.0,
            internal_solid_infill_speed: 250.0,
            top_surface_speed: 200.0,
            bridge_speed: 50.0,
            gap_infill_speed: 250.0,
            support_speed: 150.0,
            support_interface_speed: 80.0,
            initial_layer_infill_speed: 50.0,
            small_perimeter_speed: 200.0,
            travel_speed_z: 0.0, // 0 = use travel_speed

            // Filament settings
            filament_max_volumetric_speed: 0.0, // 0 = unlimited
            filament_type: "PLA".to_string(),
            filament_retraction_length: 0.8,
            filament_retraction_speed: 30.0,
            filament_deretraction_speed: 0.0, // 0 = use retraction speed
            filament_z_hop: 0.4,
            filament_wipe_distance: 1.0,
            filament_retraction_minimum_travel: 2.0,
            filament_retract_when_changing_layer: false,
            filament_start_gcode: String::new(),
            filament_end_gcode: String::new(),

            // Machine G-code
            machine_start_gcode: String::new(),
            machine_end_gcode: String::new(),
            before_layer_change_gcode: String::new(),
            layer_change_gcode: String::new(),
            change_filament_gcode: String::new(),
            tool_change_gcode: String::new(),
            machine_pause_gcode: String::new(),
            printing_by_object_gcode: String::new(),

            // Timelapse
            enable_timelapse: false,
            timelapse_type: 0,

            // Machine limits - BambuStudio X1C defaults
            machine_max_acceleration_x: 20000.0,
            machine_max_acceleration_y: 20000.0,
            machine_max_acceleration_z: 500.0,
            machine_max_acceleration_e: 5000.0,
            machine_max_acceleration_extruding: 20000.0,
            machine_max_acceleration_retracting: 5000.0,
            machine_max_acceleration_travel: 20000.0,
            machine_max_speed_x: 500.0,
            machine_max_speed_y: 500.0,
            machine_max_speed_z: 20.0,
            machine_max_speed_e: 30.0,
            machine_max_jerk_x: 9.0,
            machine_max_jerk_y: 9.0,
            machine_max_jerk_z: 3.0,
            machine_max_jerk_e: 2.5,
            machine_min_extruding_rate: 0.0,
            machine_min_travel_rate: 0.0,

            // Bed temperature variants
            eng_plate_temp: 0,
            eng_plate_temp_initial_layer: 0,
            hot_plate_temp: 60,
            hot_plate_temp_initial_layer: 60,
            cool_plate_temp_initial_layer: 35,
            textured_plate_temp: 55,
            textured_plate_temp_initial_layer: 55,
            curr_bed_type: "Hot Plate".to_string(),

            // Nozzle temperature
            nozzle_temperature: 220,
            nozzle_temperature_range_high: 240,
            nozzle_temperature_initial_layer: 220,
            chamber_temperature: 0,

            // Long retraction / cut
            enable_long_retraction_when_cut: false,
            retraction_distances_when_cut: 18.0,

            // Wipe tower / prime tower
            enable_prime_tower: false,
            prime_tower_width: 60.0,

            // Multi-material
            flush_into_infill: false,
            flush_into_objects: false,
            flush_into_support: false,

            // Pressure advance
            enable_pressure_advance: false,
            pressure_advance: 0.0,

            // Printable height
            printable_height: 256.0,

            // Extruder clearance
            extruder_clearance_height_to_rod: 36.0,
            extruder_clearance_height_to_lid: 140.0,
            extruder_clearance_max_radius: 68.0,

            // Filename format
            filename_format: "{input_filename_base}_{filament_type[0]}_{print_time}.gcode"
                .to_string(),

            // Silent mode
            enable_silent: false,

            // Accel-to-decel
            accel_to_decel_enable: false,
            accel_to_decel_factor: 50.0,

            // Exclude object
            exclude_object: true,

            // G-code features
            gcode_add_line_number: false,
            use_firmware_retraction: false,

            // Reduce crossing wall
            reduce_crossing_wall: false,
            max_travel_detour_distance: 0.0,

            // Max print speed
            max_print_speed: 0.0, // 0 = unlimited

            // Max volumetric speed
            max_volumetric_speed: 0.0, // 0 = unlimited

            // Wall sequence
            wall_sequence: WallSequence::InnerOuter,

            // Timelapse G-code
            time_lapse_gcode: String::new(),

            // Scan first layer
            scan_first_layer: false,

            // Multi-material (additional)
            single_extruder_multi_material: false,
            support_filament: 0,
            support_interface_filament: 0,
            flush_volumes_matrix: Vec::new(),
            filament_prime_volumes: Vec::new(),
            filament_prime_volumes_nc: Vec::new(),

            // Filament (additional)
            filament_is_support: false,
            filament_soluble: false,
            retract_when_changing_layer: false,

            // Wipe tower position
            wipe_tower_x: 0.0,
            wipe_tower_y: 0.0,

            // Wrapping detection
            enable_wrapping_detection: false,
            wrapping_detection_gcode: String::new(),

            // Bed temperature formula
            bed_temperature_formula: BedTempFormula::ByFirstFilament,

            // Extruder offset (0 = no offset)
            extruder_offset_x: 0.0,
            extruder_offset_y: 0.0,
        }
    }
}

impl fmt::Display for PrintConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PrintConfig(layer={:.2}mm, nozzle={:.2}mm, speed={:.0}mm/s)",
            self.layer_height, self.nozzle_diameter, self.print_speed
        )
    }
}

/// Configuration for a specific print object.
///
/// These settings can be applied per-object to override global settings.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PrintObjectConfig {
    /// Layer height for this object (mm).
    pub layer_height: CoordF,

    /// First layer height for this object (mm).
    /// PrintConfig.hpp:initial_layer_print_height
    pub first_layer_height: CoordF,

    /// Number of perimeters/shells.
    pub perimeters: u32,

    /// Number of solid top layers.
    pub top_solid_layers: u32,

    /// Number of solid bottom layers.
    pub bottom_solid_layers: u32,

    /// Infill density (0.0 - 1.0).
    pub fill_density: CoordF,

    /// Infill pattern.
    pub fill_pattern: InfillPattern,

    // === G-code Configuration ===
    /// Use relative E distances (default: false = absolute E).
    pub use_relative_e_distances: bool,

    // === Per-Feature Line Widths (mm) ===
    // 0.0 means "auto" — derive from nozzle diameter.
    // BambuStudio exposes these as explicit per-feature overrides.
    /// Default line width (mm). 0 = auto (nozzle_diameter * 1.125).
    /// BambuStudio: `line_width`.
    pub line_width: CoordF,

    /// Initial/first layer line width (mm). 0 = use default line_width.
    /// BambuStudio: `initial_layer_line_width`.
    pub initial_layer_line_width: CoordF,

    /// Outer wall / external perimeter line width (mm). 0 = use default line_width.
    /// BambuStudio: `outer_wall_line_width`.
    pub outer_wall_line_width: CoordF,

    /// Inner wall / internal perimeter line width (mm). 0 = use default line_width.
    /// BambuStudio: `inner_wall_line_width`.
    pub inner_wall_line_width: CoordF,

    /// Sparse infill line width (mm). 0 = use default line_width.
    /// BambuStudio: `sparse_infill_line_width`.
    pub sparse_infill_line_width: CoordF,

    /// Internal solid infill line width (mm). 0 = use default line_width.
    /// BambuStudio: `internal_solid_infill_line_width`.
    pub solid_infill_line_width: CoordF,

    /// Top surface line width (mm). 0 = use default line_width.
    /// BambuStudio: `top_surface_line_width`.
    pub top_surface_line_width: CoordF,

    // === Speeds (mm/s) ===
    /// Perimeter speed (mm/s).
    pub perimeter_speed: CoordF,

    /// External perimeter speed (mm/s).
    pub external_perimeter_speed: CoordF,

    /// Infill speed (mm/s).
    pub infill_speed: CoordF,

    /// Solid infill speed (mm/s).
    pub solid_infill_speed: CoordF,

    /// Top solid infill speed (mm/s).
    pub top_solid_infill_speed: CoordF,

    /// Bridge speed (mm/s).
    pub bridge_speed: CoordF,

    /// Gap fill speed (mm/s).
    pub gap_fill_speed: CoordF,

    // === Perimeter Options ===
    /// Enable thin walls detection.
    /// BambuStudio: `detect_thin_wall`.
    pub thin_walls: bool,

    /// Enable gap fill.
    pub gap_fill: bool,

    /// Detect bridging perimeters.
    pub overhangs: bool,

    /// Only use one wall on the first layer.
    /// BambuStudio: `only_one_wall_first_layer`.
    pub only_one_wall_first_layer: bool,

    /// Control whether top surfaces use only one wall (perimeter).
    /// BambuStudio: `top_one_wall_type`.
    /// - `None`: disabled (use configured wall_loops everywhere)
    /// - `TopMost`: only the topmost layer uses 1 perimeter
    /// - `AllTop`: all layers with top surfaces use 1 perimeter for top regions
    pub top_one_wall_type: TopOneWallType,

    // === Quality ===
    /// Slice closing radius (mm).
    pub slice_closing_radius: CoordF,

    /// XY size compensation (mm).
    pub xy_size_compensation: CoordF,

    /// Elephant foot compensation (mm).
    /// BambuStudio: `elefant_foot_compensation`.
    pub elephant_foot_compensation: CoordF,

    // === Infill ===
    /// Infill/perimeter overlap percentage (0.0 - 1.0).
    /// BambuStudio: `infill_wall_overlap` (as percentage string like "15%").
    pub infill_wall_overlap: CoordF,

    /// MonotonicLine top-surface wipe-connector overlap extension into the wall
    /// (0.0 - 1.0). BambuStudio: `monotonic_travel_into_wall` (percent, default 0;
    /// BBL process profiles set 45%). Consumed as gap_compensation_ratio by
    /// extrusion_entities_append_paths_with_wipe (Fill.cpp:233/636).
    pub monotonic_travel_into_wall: CoordF,

    /// Infill angle in degrees.
    /// BambuStudio: `infill_direction`.
    pub infill_angle: CoordF,

    // === Flow ===
    /// Initial layer flow ratio (multiplier, e.g. 1.0 = 100%).
    /// BambuStudio: `initial_layer_flow_ratio`.
    pub initial_layer_flow_ratio: CoordF,

    /// Top solid infill flow ratio (multiplier, e.g. 1.0 = 100%).
    /// BambuStudio: `top_solid_infill_flow_ratio`.
    pub top_solid_infill_flow_ratio: CoordF,

    /// Object-level flow ratio (multiplier, e.g. 0.98 = 98%).
    /// This is the extrusion multiplier applied to all features.
    /// BambuStudio: `print_flow_ratio`.
    pub print_flow_ratio: CoordF,

    // === Seam ===
    /// Seam position preference.
    pub seam_position: SeamPosition,

    /// Ensure seam placement away from overhangs for alignment and backing modes.
    /// C++: `((ConfigOptionBool, seam_placement_away_from_overhangs))`
    /// (PrintConfig.hpp:924); default `false` (PrintConfig.cpp:4650-4655).
    /// BambuStudio: `seam_placement_away_from_overhangs`.
    pub seam_placement_away_from_overhangs: bool,

    // === Fuzzy Skin ===
    /// Enable fuzzy skin.
    pub fuzzy_skin: bool,

    /// Fuzzy skin thickness (mm).
    pub fuzzy_skin_thickness: CoordF,

    /// Fuzzy skin point distance (mm).
    pub fuzzy_skin_point_distance: CoordF,

    // === Wipe ===
    /// Enable wipe during retraction to reduce stringing.
    pub wipe_enabled: bool,

    /// Wipe distance (mm) - how far to move while wiping.
    pub wipe_distance: CoordF,

    /// Retract before wipe percentage (0-100).
    /// How much of the retraction happens before vs during the wipe move.
    pub retract_before_wipe: CoordF,

    // === Arachne / Variable Width ===
    /// Perimeter generation mode (Classic or Arachne).
    pub perimeter_mode: PerimeterMode,

    /// Minimum bead width for Arachne mode (mm).
    /// Walls thinner than this will not be printed.
    pub arachne_min_bead_width: CoordF,

    /// Minimum feature size for Arachne mode (mm).
    /// Features smaller than this will be ignored.
    pub arachne_min_feature_size: CoordF,

    /// Wall transition length for Arachne mode (mm).
    /// Length over which wall count transitions occur.
    pub arachne_wall_transition_length: CoordF,

    // === Narrow Region Detection ===
    /// C++: PrintObjectConfig `interface_shells` (default: false).
    pub interface_shells: bool,

    /// C++: PrintConfig `spiral_mode` — duplicated here since Rust PrintObject
    /// has no back-reference to Print.
    pub spiral_vase: bool,

    /// Detect narrow internal regions and fill them with solid infill instead of
    /// sparse infill. When enabled, internal infill areas whose width is smaller
    /// than the sparse infill line spacing are automatically promoted to solid
    /// infill density. This matches BambuStudio's `detect_narrow_internal_solid_infill`
    /// behaviour and is critical for small objects where sparse infill at low density
    /// would produce no extrusion lines.
    pub detect_narrow_internal_solid_infill: bool,

    /// Minimum area for sparse infill regions (mm²).
    ///
    /// Internal (sparse) infill regions smaller than this threshold are
    /// automatically promoted to solid infill.  This is part of BambuStudio's
    /// `prepare_fill_surfaces()` step in the `prepare_infill` pipeline.
    ///
    /// A value of 0.0 disables the area-based promotion (only narrow-region
    /// detection applies, if enabled).
    ///
    /// BambuStudio config key: `minimum_sparse_infill_area` (default: 0).
    pub minimum_sparse_infill_area: CoordF,

    // === Raft (Object-level overrides) ===
    /// Number of raft layers for this object.
    /// Overrides the global raft_layers setting.
    /// BambuStudio: `raft_layers` (in PrintObjectConfig).
    pub raft_layers: u32,

    /// Raft expansion beyond the model (mm).
    /// BambuStudio: `raft_expansion` (in PrintObjectConfig).
    pub raft_expansion: CoordF,

    /// Vertical gap between raft and model (mm).
    /// BambuStudio: `raft_contact_distance` (in PrintObjectConfig).
    pub raft_contact_distance: CoordF,

    // === Support (Object-level settings) ===
    /// Enable support structures for this object.
    /// BambuStudio: `enable_support` (in PrintObjectConfig).
    pub enable_support: bool,

    /// Support type (normal, tree).
    /// BambuStudio: `support_type` (in PrintObjectConfig).
    pub support_type: SupportType,

    /// Force generation of support for the first N layers.
    /// Used for enforcing support even when overhang detection would skip it.
    /// BambuStudio: `enforce_support_layers` (in PrintObjectConfig).
    pub enforce_support_layers: u32,

    /// Support overhang threshold angle (degrees).
    /// Overhangs below this angle require support.
    /// BambuStudio: `support_threshold_angle` (in PrintObjectConfig).
    pub support_threshold_angle: CoordF,

    /// Support only on build plate (don't generate support on top of model).
    /// BambuStudio: `support_on_build_plate_only` (in PrintObjectConfig).
    pub support_on_build_plate_only: bool,

    /// Support density (0.0 - 1.0).
    /// BambuStudio: Derived from `support_base_pattern_spacing`.
    pub support_density: CoordF,

    /// Support line width (mm). 0 = use default line_width.
    /// BambuStudio: `support_line_width` (in PrintObjectConfig).
    pub support_line_width: CoordF,

    /// Support base pattern spacing (mm).
    /// Spacing between support lines in the base pattern.
    /// BambuStudio: `support_base_pattern_spacing` (in PrintObjectConfig).
    pub support_base_pattern_spacing: CoordF,

    /// Support interface top layers count.
    /// Number of interface layers at the top (touching model).
    /// BambuStudio: `support_interface_top_layers` (in PrintObjectConfig).
    pub support_interface_top_layers: u32,

    /// Support interface bottom layers count.
    /// -1 means "same as top" (C++ coInt with min -1).
    /// BambuStudio: `support_interface_bottom_layers` (in PrintObjectConfig).
    pub support_interface_bottom_layers: i32,

    /// Support interface spacing (mm).
    /// Spacing between interface lines. 0 = solid interface.
    /// BambuStudio: `support_interface_spacing` (in PrintObjectConfig).
    pub support_interface_spacing: CoordF,

    /// Support top Z distance (mm).
    /// Vertical gap between support top and model bottom.
    /// BambuStudio: `support_top_z_distance` (in PrintObjectConfig).
    pub support_top_z_distance: CoordF,

    /// Support bottom Z distance (mm).
    /// Vertical gap between model top and support growing from it.
    /// BambuStudio: `support_bottom_z_distance` (in PrintObjectConfig).
    pub support_bottom_z_distance: CoordF,

    /// Support XY distance (mm).
    /// Horizontal gap between support and model.
    /// BambuStudio: `support_object_xy_distance` (in PrintObjectConfig).
    pub support_object_xy_distance: CoordF,

    /// Support expansion (mm).
    /// Expand support base beyond detected overhang areas.
    /// BambuStudio: `support_expansion` (in PrintObjectConfig).
    pub support_expansion: CoordF,

    /// Support pattern angle (degrees).
    /// Rotate the support pattern on the horizontal plane.
    /// BambuStudio: `support_angle` (PrintConfig.hpp:931).
    /// C++ default: 0. Range [0, 359].
    pub support_angle: CoordF,

    /// XY separation between object and its support at the first layer (mm).
    /// BambuStudio: `support_object_first_layer_gap` (PrintConfig.hpp:969).
    /// C++ default: 0.2.
    pub support_object_first_layer_gap: CoordF,

    // === Support Ironing ===
    /// Enable ironing on solid support interface layers.
    /// BambuStudio: `enable_support_ironing` (PrintConfig.hpp:948).
    /// C++ default: false.
    pub enable_support_ironing: bool,

    /// Support ironing pattern (ipRectilinear or ipConcentric).
    /// BambuStudio: `support_ironing_pattern` (PrintConfig.hpp:949).
    /// C++ default: ipRectilinear.
    pub support_ironing_pattern: InfillPattern,

    /// Support ironing flow (fraction of normal flow, e.g. 0.10 = 10%).
    /// BambuStudio: `support_ironing_flow` (coPercent, PrintConfig.hpp:950).
    /// C++ default: 10 (percent) → stored as ratio 0.10.
    pub support_ironing_flow: CoordF,

    /// Support ironing line spacing (mm).
    /// BambuStudio: `support_ironing_spacing` (PrintConfig.hpp:951).
    /// C++ default: 0.1.
    pub support_ironing_spacing: CoordF,

    /// Support ironing inset from edge (mm).
    /// BambuStudio: `support_ironing_inset` (PrintConfig.hpp:952).
    /// C++ default: 0.0.
    pub support_ironing_inset: CoordF,

    /// Support ironing direction (degrees).
    /// BambuStudio: `support_ironing_direction` (PrintConfig.hpp:953).
    /// C++ default: 0.0.
    pub support_ironing_direction: CoordF,

    /// Support ironing print speed (mm/s).
    /// BambuStudio: `support_ironing_speed` (PrintConfig.hpp:954).
    /// C++ default: 20.0.
    pub support_ironing_speed: CoordF,

    // === Ironing ===
    /// Ironing type (none, top, topmost, all solid).
    /// BambuStudio: `ironing_type`.
    pub ironing_type: IroningType,
    /// Ironing flow rate (fraction, e.g. 0.15 = 15%).
    /// BambuStudio: `ironing_flow` (percentage in C++).
    pub ironing_flow: CoordF,
    /// Ironing speed (mm/s).
    /// BambuStudio: `ironing_speed`.
    pub ironing_speed: CoordF,
    /// Ironing line spacing (mm).
    /// BambuStudio: `ironing_spacing`.
    pub ironing_spacing: CoordF,
    /// Ironing pattern direction (degrees).
    /// BambuStudio: `ironing_direction`.
    pub ironing_direction: CoordF,

    // === Scarf Seam ===
    /// Scarf seam type.
    /// BambuStudio: `seam_slope_type`.
    pub scarf_seam_type: ScarfSeamType,
    /// Scarf seam start height (mm or %).
    /// BambuStudio: `seam_slope_start_height`.
    pub scarf_seam_start_height: CoordF,
    /// Scarf seam steps.
    /// BambuStudio: `seam_slope_steps`.
    pub scarf_seam_steps: u32,
    /// Whether scarf seam applies to inner walls.
    /// BambuStudio: `seam_slope_inner_walls`.
    pub scarf_seam_inner_walls: bool,
    /// Whether scarf seam applies to entire loop.
    /// BambuStudio: `seam_slope_entire_loop`.
    pub scarf_seam_entire_loop: bool,
    /// Scarf seam gap distance.
    /// BambuStudio: `seam_slope_gap`.
    pub scarf_seam_gap: CoordF,
    /// Scarf seam minimum length (mm).
    /// BambuStudio: `seam_slope_min_length`.
    pub scarf_seam_min_length: CoordF,
    /// Whether scarf is conditional (only at sharp corners).
    /// BambuStudio: `seam_slope_conditional`.
    pub scarf_seam_conditional: bool,
    /// Scarf angle threshold (degrees).
    /// BambuStudio: `scarf_angle_threshold`.
    pub scarf_angle_threshold: CoordF,
    /// Seam gap (mm). Gap before/after seam start point.
    /// BambuStudio: `seam_gap`.
    pub seam_gap: CoordF,

    // === Brim ===
    /// Brim type.
    /// BambuStudio: `brim_type`.
    pub brim_type: BrimType,
    /// Brim width (mm).
    /// BambuStudio: `brim_width`.
    pub brim_width: CoordF,
    /// Gap between brim and object (mm).
    /// BambuStudio: `brim_object_gap`.
    pub brim_object_gap: CoordF,

    // === Wall Order ===
    /// Wall/infill print order.
    /// BambuStudio: `wall_infill_order`.
    pub wall_infill_order: WallInfillOrder,
    /// Whether to print infill before walls.
    /// BambuStudio: `is_infill_first`.
    pub is_infill_first: bool,

    // === Surface Patterns ===
    /// Top surface pattern.
    /// BambuStudio: `top_surface_pattern`.
    pub top_surface_pattern: SurfacePattern,
    /// Bottom surface pattern.
    /// BambuStudio: `bottom_surface_pattern`.
    pub bottom_surface_pattern: SurfacePattern,
    /// Internal solid infill pattern.
    /// BambuStudio: `internal_solid_infill_pattern`.
    pub internal_solid_infill_pattern: InternalSolidInfillPattern,
    /// Sparse infill pattern.
    /// BambuStudio: `sparse_infill_pattern`.
    pub sparse_infill_pattern: InfillPattern,

    // === Bridge ===
    /// Bridge flow ratio (multiplier, e.g. 1.0 = 100%).
    /// BambuStudio: `bridge_flow`.
    pub bridge_flow: CoordF,
    /// Bridge angle (degrees). 0 = auto.
    /// BambuStudio: `bridge_angle`.
    pub bridge_angle: CoordF,
    /// Enable thick bridges.
    /// BambuStudio: `thick_bridges`.
    pub thick_bridges: bool,
    /// Max bridge length before support is needed (mm).
    /// BambuStudio: `max_bridge_length`.
    pub max_bridge_length: CoordF,
    /// Bottom solid infill flow ratio.
    /// BambuStudio: `bottom_solid_infill_flow_ratio`.
    pub bottom_solid_infill_flow_ratio: CoordF,

    // === Adaptive Layer Height ===
    /// Enable adaptive layer height.
    /// BambuStudio: `adaptive_layer_height`.
    pub adaptive_layer_height: bool,

    // === Layer Height Limits ===
    /// Minimum layer height (mm).
    /// BambuStudio: `min_layer_height`.
    pub min_layer_height: CoordF,
    /// Maximum layer height (mm).
    /// BambuStudio: `max_layer_height`.
    pub max_layer_height: CoordF,

    // === Fuzzy Skin (Enum) ===
    /// Fuzzy skin type (none, external, all walls).
    /// BambuStudio: `fuzzy_skin` enum.
    pub fuzzy_skin_type: FuzzySkinType,

    // === Support (additional) ===
    /// Support base pattern.
    /// BambuStudio: `support_base_pattern`.
    pub support_base_pattern: SupportBasePattern,
    /// Support interface pattern.
    /// BambuStudio: `support_interface_pattern`.
    pub support_interface_pattern: SupportInterfacePattern,
    /// Don't support bridges.
    /// BambuStudio: `bridge_no_support`.
    pub bridge_no_support: bool,
    /// Independent support layer height.
    /// BambuStudio: `independent_support_layer_height`.
    pub independent_support_layer_height: bool,
    /// Remove small overhangs from support.
    /// BambuStudio: `support_remove_small_overhang`.
    pub support_remove_small_overhang: bool,
    /// Top Z distance overrides XY distance.
    /// BambuStudio: `top_z_overrides_xy_distance`.
    pub top_z_overrides_xy_distance: bool,
    /// Support style (for tree supports).
    /// BambuStudio: `support_style`.
    pub support_style: TreeSupportStyle,
    /// Support interface loop pattern (concentric loops).
    /// BambuStudio: `support_interface_loop_pattern`.
    pub support_interface_loop_pattern: bool,
    /// Tree support branch angle (degrees).
    /// BambuStudio: `tree_support_branch_angle`.
    pub tree_support_branch_angle: CoordF,
    /// Tree support branch diameter (mm).
    /// BambuStudio: `tree_support_branch_diameter`.
    pub tree_support_branch_diameter: CoordF,
    /// Tree support branch diameter angle (degrees). Controls how much
    /// branch diameter grows per layer toward the bottom.
    /// BambuStudio: `tree_support_branch_diameter_angle`.
    /// C++ default: 5.0.
    pub tree_support_branch_diameter_angle: CoordF,
    /// Tree support branch distance (mm).
    /// BambuStudio: `tree_support_branch_distance`.
    pub tree_support_branch_distance: CoordF,
    /// Tree support wall count.
    /// BambuStudio: `tree_support_wall_count`.
    pub tree_support_wall_count: u32,
    /// Tree support with infill.
    /// BambuStudio: `tree_support_with_infill`.
    pub tree_support_with_infill: bool,
    /// Tree support brim width (mm).
    /// BambuStudio: `tree_support_brim_width`.
    pub tree_support_brim_width: CoordF,

    // === Wall Settings (additional) ===
    /// Number of wall loops.
    /// BambuStudio: `wall_loops`. Alias for `perimeters`.
    pub wall_loops: u32,
    /// Wall transition angle for Arachne (degrees).
    /// BambuStudio: `wall_transition_angle`.
    pub wall_transition_angle: CoordF,
    /// Wall transition filter deviation for Arachne.
    /// BambuStudio: `wall_transition_filter_deviation`.
    pub wall_transition_filter_deviation: CoordF,
    /// Wall distribution count for Arachne.
    /// BambuStudio: `wall_distribution_count`.
    pub wall_distribution_count: u32,
    /// Precise outer wall.
    /// BambuStudio: `precise_outer_wall`.
    pub precise_outer_wall: bool,

    // === Overhang Speed ===
    /// Enable overhang speed adjustment.
    /// BambuStudio: `enable_overhang_speed`.
    pub enable_overhang_speed: bool,
    /// Enable fan boost on overhangs/bridges (filament setting).
    /// BambuStudio: `enable_overhang_bridge_fan` (per-filament bools).
    pub enable_overhang_bridge_fan: bool,
    /// Overhang fan threshold enum index (PrintConfig.cpp:1200-1205:
    /// "0%"=0(none) "10%"=1 "25%"=2 "50%"=3 "75%"=4 "95%"=5).
    pub overhang_fan_threshold: i32,
    /// Overhang 1/4 speed (mm/s or %).
    /// BambuStudio: `overhang_1_4_speed`.
    pub overhang_1_4_speed: CoordF,
    /// Overhang 2/4 speed (mm/s or %).
    /// BambuStudio: `overhang_2_4_speed`.
    pub overhang_2_4_speed: CoordF,
    /// Overhang 3/4 speed (mm/s or %).
    /// BambuStudio: `overhang_3_4_speed`.
    pub overhang_3_4_speed: CoordF,
    /// Overhang 4/4 speed (mm/s or %).
    /// BambuStudio: `overhang_4_4_speed`.
    pub overhang_4_4_speed: CoordF,
    /// Overhang 100% (totally) speed (mm/s or %).
    /// BambuStudio: `overhang_totally_speed`. Maps to overhang degree 5
    /// (overhang_speed_key_map {5: "overhang_totally_speed"}, GCode.cpp:5354).
    pub overhang_totally_speed: CoordF,
    /// Enable speed-transition smoothing across discontinuity areas.
    /// BambuStudio: `smooth_speed_discontinuity_area` (coBool, default true).
    pub smooth_speed_discontinuity_area: bool,
    /// Smoothing coefficient for the speed ramp f(x) = coeff * x^2.
    /// BambuStudio: `smooth_coefficient`. The effective coefficient is
    /// `filament_velocity_adaptation_factor * smooth_coefficient`
    /// (GCode::set_smooth_coff); the per-filament factor is assumed 1.0.
    pub smooth_coefficient: CoordF,

    // === Infill (additional) ===
    /// Infill combination (every N layers).
    /// BambuStudio: `infill_combination`.
    pub infill_combination: bool,
    /// Sparse infill anchor length (mm).
    /// BambuStudio: `sparse_infill_anchor`.
    pub sparse_infill_anchor: CoordF,
    /// Sparse infill anchor max length (mm).
    /// BambuStudio: `sparse_infill_anchor_max`.
    pub sparse_infill_anchor_max: CoordF,
    /// Reduce infill retraction.
    /// BambuStudio: `reduce_infill_retraction`.
    pub reduce_infill_retraction: bool,

    // === Hole/Contour Compensation ===
    /// XY hole compensation (mm).
    /// BambuStudio: `xy_hole_compensation`.
    pub xy_hole_compensation: CoordF,
    /// XY contour compensation (mm).
    /// BambuStudio: `xy_contour_compensation`.
    pub xy_contour_compensation: CoordF,

    // === Slicing ===
    /// Slicing mode.
    /// BambuStudio: `slicing_mode`.
    pub slicing_mode: SlicingMode,
    /// Ensure vertical shell thickness.
    /// BambuStudio: `ensure_vertical_shell_thickness`.
    pub ensure_vertical_shell_thickness: EnsureVerticalShellThickness,
    /// Detect overhang wall.
    /// BambuStudio: `detect_overhang_wall`.
    pub detect_overhang_wall: bool,

    // === Draft Shield ===
    /// Draft shield type.
    /// BambuStudio: `draft_shield`.
    pub draft_shield: DraftShield,

    // === Raft (additional) ===
    /// Raft first layer density (0.0-1.0).
    /// BambuStudio: `raft_first_layer_density`.
    pub raft_first_layer_density: CoordF,
    /// Raft first layer expansion (mm).
    /// BambuStudio: `raft_first_layer_expansion`.
    pub raft_first_layer_expansion: CoordF,

    // === Top/Bottom Shell Thickness ===
    /// Top shell thickness (mm). Alternative to layer count.
    /// BambuStudio: `top_shell_thickness`.
    pub top_shell_thickness: CoordF,
    /// Bottom shell thickness (mm). Alternative to layer count.
    /// BambuStudio: `bottom_shell_thickness`.
    pub bottom_shell_thickness: CoordF,

    // === Elephant Foot ===
    /// Elephant foot minimum width (mm).
    /// BambuStudio: `elefant_foot_min_width`.
    pub elephant_foot_min_width: CoordF,

    // === Internal Bridge ===
    /// Internal bridge support thickness (mm).
    /// BambuStudio: `internal_bridge_support_thickness`.
    pub internal_bridge_support_thickness: CoordF,

    // === Embedding Wall Into Infill ===
    /// Embed wall into infill for better adhesion.
    /// BambuStudio: `embedding_wall_into_infill`.
    pub embedding_wall_into_infill: bool,

    // === Filter Out Gap Fill ===
    /// Filter out tiny gap fill extrusions.
    /// BambuStudio: `filter_out_gap_fill`.
    pub filter_out_gap_fill: CoordF,

    // === Skirt ===
    /// Skirt height (layers).
    /// BambuStudio: `skirt_height`.
    pub skirt_height: u32,
    /// Skirt loops.
    /// BambuStudio: `skirt_loops`.
    pub skirt_loops: u32,
    /// Skirt distance from object (mm).
    /// BambuStudio: `skirt_distance`.
    pub skirt_distance: CoordF,

    // === Z-hop ===
    /// Retraction Z-hop for this object (mm).
    /// BambuStudio: `z_hop`.
    pub z_hop: CoordF,

    // === Small Perimeter ===
    /// Small perimeter speed (mm/s).
    /// BambuStudio: `small_perimeter_speed`.
    pub small_perimeter_speed: CoordF,
    /// Small perimeter threshold (mm, length).
    /// BambuStudio: `small_perimeter_threshold`.
    pub small_perimeter_threshold: CoordF,

    // === Support Line Width ===
    /// Support interface line width (mm). 0 = use default.
    /// BambuStudio: derived from support_interface_spacing.
    pub support_interface_line_width: CoordF,

    // === Speed: Initial Layer ===
    /// Initial layer speed (mm/s).
    /// BambuStudio: `initial_layer_speed`.
    pub initial_layer_speed: CoordF,

    /// Initial layer infill speed (mm/s).
    /// BambuStudio: `initial_layer_infill_speed`.
    pub initial_layer_infill_speed: CoordF,

    // === Support Speed ===
    /// Support print speed (mm/s).
    /// BambuStudio: `support_speed`.
    pub support_speed: CoordF,
    /// Support interface print speed (mm/s).
    /// BambuStudio: `support_interface_speed`.
    pub support_interface_speed: CoordF,

    // === Detect Floating Vertical Shell ===
    /// Detect floating vertical shell regions.
    /// BambuStudio: `detect_floating_vertical_shell`.
    pub detect_floating_vertical_shell: bool,
    /// Floating vertical shell speed as a PERCENT of internal_solid_infill_speed
    /// (PrintConfig coPercent, default 80). BambuStudio: `vertical_shell_speed`;
    /// consumed by GCode.cpp:6492-6500 via get_abs_value(internal_solid_infill_speed).
    pub vertical_shell_speed: CoordF,

    // === Interlocking (InterlockingGenerator) ===
    /// Use beam interlocking between touching filaments.
    /// PrintConfig.hpp:1008 ((ConfigOptionBool, interlocking_beam))
    pub interlocking_beam: bool,
    /// The width of the interlocking structure beams (mm).
    /// PrintConfig.hpp:1009 ((ConfigOptionFloat, interlocking_beam_width))
    pub interlocking_beam_width: CoordF,
    /// Orientation of interlock beams (degrees).
    /// PrintConfig.hpp:1010 ((ConfigOptionFloat, interlocking_orientation))
    pub interlocking_orientation: CoordF,
    /// The height of the interlocking beams, in layers.
    /// PrintConfig.hpp:1011 ((ConfigOptionInt, interlocking_beam_layer_count))
    pub interlocking_beam_layer_count: i32,
    /// Distance from the filament boundary to generate the structure, in cells.
    /// PrintConfig.hpp:1012 ((ConfigOptionInt, interlocking_depth))
    pub interlocking_depth: i32,
    /// Distance from the model outside where no structure is generated, in cells.
    /// PrintConfig.hpp:1013 ((ConfigOptionInt, interlocking_boundary_avoidance))
    pub interlocking_boundary_avoidance: i32,
}

/// Helper methods for resolving per-feature line widths.
///
/// When a per-feature width is 0.0 ("auto"), we fall back to the default
/// `line_width`, and if that is also 0.0 we compute from nozzle diameter.
impl PrintObjectConfig {
    // Resolve the effective default line width given a nozzle diameter.
    // If `self.line_width` is 0 (auto), returns `nozzle_diameter * 1.125`.
    pub fn effective_line_width(&self, nozzle_diameter: CoordF) -> CoordF {
        if self.line_width > 0.0 {
            self.line_width
        } else {
            nozzle_diameter * 1.125
        }
    }

    /// Resolve initial layer line width. Falls back to default line width.
    pub fn effective_initial_layer_line_width(&self, nozzle_diameter: CoordF) -> CoordF {
        if self.initial_layer_line_width > 0.0 {
            self.initial_layer_line_width
        } else {
            self.effective_line_width(nozzle_diameter)
        }
    }

    /// Resolve outer wall (external perimeter) line width.
    /// Falls back to default line width.
    pub fn effective_outer_wall_line_width(&self, nozzle_diameter: CoordF) -> CoordF {
        if self.outer_wall_line_width > 0.0 {
            self.outer_wall_line_width
        } else {
            self.effective_line_width(nozzle_diameter)
        }
    }

    /// Resolve inner wall (internal perimeter) line width.
    /// Falls back to default line width.
    pub fn effective_inner_wall_line_width(&self, nozzle_diameter: CoordF) -> CoordF {
        if self.inner_wall_line_width > 0.0 {
            self.inner_wall_line_width
        } else {
            self.effective_line_width(nozzle_diameter)
        }
    }

    /// Resolve sparse infill line width. Falls back to default line width.
    pub fn effective_sparse_infill_line_width(&self, nozzle_diameter: CoordF) -> CoordF {
        if self.sparse_infill_line_width > 0.0 {
            self.sparse_infill_line_width
        } else {
            self.effective_line_width(nozzle_diameter)
        }
    }

    /// Resolve solid infill line width. Falls back to default line width.
    pub fn effective_solid_infill_line_width(&self, nozzle_diameter: CoordF) -> CoordF {
        if self.solid_infill_line_width > 0.0 {
            self.solid_infill_line_width
        } else {
            self.effective_line_width(nozzle_diameter)
        }
    }

    /// Resolve top surface line width. Falls back to default line width.
    pub fn effective_top_surface_line_width(&self, nozzle_diameter: CoordF) -> CoordF {
        if self.top_surface_line_width > 0.0 {
            self.top_surface_line_width
        } else {
            self.effective_line_width(nozzle_diameter)
        }
    }
}

impl PrintObjectConfig {
    // Create a new PrintObjectConfig with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder method: set number of perimeters.
    pub fn perimeters(mut self, count: u32) -> Self {
        self.perimeters = count;
        self
    }

    /// Builder method: set infill density.
    pub fn fill_density(mut self, density: CoordF) -> Self {
        self.fill_density = density;
        self
    }

    /// Builder method: set infill pattern.
    pub fn fill_pattern(mut self, pattern: InfillPattern) -> Self {
        self.fill_pattern = pattern;
        self
    }

    /// Builder method: set seam position.
    pub fn seam_position(mut self, position: SeamPosition) -> Self {
        self.seam_position = position;
        self
    }

    /// Builder method: set perimeter mode.
    pub fn perimeter_mode(mut self, mode: PerimeterMode) -> Self {
        self.perimeter_mode = mode;
        self
    }

    /// Builder method: enable Arachne mode.
    pub fn arachne(mut self) -> Self {
        self.perimeter_mode = PerimeterMode::Arachne;
        self
    }

    /// Builder method: set Arachne minimum bead width.
    pub fn arachne_min_bead_width(mut self, width: CoordF) -> Self {
        self.arachne_min_bead_width = width;
        self
    }

    /// Builder method: set Arachne minimum feature size.
    pub fn arachne_min_feature_size(mut self, size: CoordF) -> Self {
        self.arachne_min_feature_size = size;
        self
    }
}

// === Parse helpers for BambuStudio JSON values (all stored as strings) ===

pub fn parse_f64(s: &str) -> Option<f64> {
    // NOTE (R345): per-extruder configs arrive as comma-arrays
    // ("105,105,105,105,105"); this plain parse fails on the comma, so the typed
    // field silently keeps its struct default. This is a real config-load bug
    // (e.g. initial_layer_infill_speed stays 50 instead of the profile's 105),
    // but a gated first-element fix was measured BYTE-NEUTRAL: the affected
    // first-layer-infill lines diverge geometrically (cascade), so correcting
    // the speed value does not align them. Left as-is; revisit if the underlying
    // geometry is ever brought to parity.
    s.trim_end_matches('%').parse::<f64>().ok()
}

pub fn parse_pct(s: &str) -> Option<f64> {
    if s.ends_with('%') {
        s.trim_end_matches('%')
            .parse::<f64>()
            .ok()
            .map(|v| v / 100.0)
    } else {
        s.parse::<f64>().ok()
    }
}

pub fn parse_bool(s: &str) -> Option<bool> {
    Some(s == "1" || s.eq_ignore_ascii_case("true"))
}

pub fn parse_u32(s: &str) -> Option<u32> {
    s.parse::<u32>().ok()
}

pub fn parse_i32(s: &str) -> Option<i32> {
    s.parse::<i32>().ok()
}

/// Parse a C++ ConfigOptionFloatOrPercent string ("10" -> 10 mm, "10%" -> 10 percent).
pub fn parse_float_or_percent(s: &str) -> Option<crate::config::FloatOrPercent> {
    let trimmed = s.trim();
    if let Some(stripped) = trimmed.strip_suffix('%') {
        stripped
            .parse::<f64>()
            .ok()
            .map(|v| crate::config::FloatOrPercent::with(v, true))
    } else {
        trimmed
            .parse::<f64>()
            .ok()
            .map(|v| crate::config::FloatOrPercent::with(v, false))
    }
}

impl PrintConfig {
    /// Apply a key-value pair from BambuStudio project_settings JSON.
    /// Returns true if the key was recognized and applied.
    pub fn set_deserialize(&mut self, key: &str, value: &str) -> bool {
        match key {
            // === Layer Heights ===
            "layer_height" => {
                if let Some(v) = parse_f64(value) {
                    self.layer_height = v;
                }
                true
            }
            "initial_layer_print_height" => {
                if let Some(v) = parse_f64(value) {
                    self.first_layer_height = v;
                }
                true
            }

            // === Speeds ===
            "outer_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.print_speed = v;
                }
                true
            }
            "travel_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.travel_speed = v;
                }
                true
            }
            "initial_layer_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.first_layer_speed = v;
                }
                true
            }

            // === Temperatures ===
            "nozzle_temperature" => {
                if let Some(v) = parse_f64(value) {
                    self.extruder_temperature = v as u32;
                }
                true
            }
            "nozzle_temperature_initial_layer" => {
                if let Some(v) = parse_f64(value) {
                    self.first_layer_extruder_temperature = v as u32;
                }
                true
            }
            "nozzle_temperature_range_high" => {
                if let Some(v) = parse_f64(value) {
                    self.nozzle_temperature_range_high = v as u32;
                }
                true
            }

            // === Retraction ===
            "retraction_length" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_length = v;
                }
                true
            }
            "retraction_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_speed = v;
                }
                true
            }
            "z_hop" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_lift = v;
                }
                true
            }
            "retract_when_changing_layer" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_before_travel = v;
                }
                true
            }
            "deretraction_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.deretract_speed = v;
                }
                true
            }
            "retract_length_toolchange" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_length_toolchange = v;
                }
                true
            }
            "retract_before_wipe" => {
                if let Some(v) = parse_pct(value) {
                    self.retract_before_wipe = v;
                }
                true
            }

            // === Filament ===
            "filament_density" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_density = v;
                }
                true
            }
            "filament_cost" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_cost = v;
                }
                true
            }
            "filament_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_flow_ratio = v;
                }
                true
            }
            // Per-filament nullable overrides (PrintConfig.cpp:82-90); the
            // loader passes the first element of the per-filament JSON array.
            "filament_enable_overhang_speed" => {
                if let Some(v) = parse_bool(value) {
                    self.filament_enable_overhang_speed = v;
                }
                true
            }
            "filament_bridge_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_bridge_speed = v;
                }
                true
            }

            // === Extrusion ===
            "nozzle_diameter" => {
                if let Some(v) = parse_f64(value) {
                    self.nozzle_diameter = v;
                }
                true
            }
            "filament_diameter" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_diameter = v;
                }
                true
            }
            // PrintConfig.hpp:1411 — print-level option read by PrintRegion::flow
            // (PrintRegion.cpp:27-28). The PrintObjectConfig keeps its own copy
            // of this key; both are fed from the same JSON value.
            "initial_layer_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_line_width = v;
                }
                true
            }

            // === Skirt/Brim ===
            "skirt_loops" => {
                if let Some(v) = parse_u32(value) {
                    self.skirt_loops = v;
                }
                true
            }
            "skirt_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.skirt_distance = v;
                }
                true
            }

            // === Raft ===
            "raft_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.raft_layers = v;
                    self.raft_enabled = v > 0;
                }
                true
            }
            "raft_expansion" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_expansion = v;
                }
                true
            }
            "raft_contact_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_contact_distance = v;
                }
                true
            }

            // === Support ===
            "enable_support" => {
                if let Some(v) = parse_bool(value) {
                    self.support_enabled = v;
                }
                true
            }
            "support_type" => {
                if value.contains("tree") {
                    self.support_type = SupportType::Tree;
                } else if value.contains("hybrid") {
                    self.support_type = SupportType::Hybrid;
                } else {
                    self.support_type = SupportType::Normal;
                }
                true
            }
            "support_threshold_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.support_threshold_angle = v;
                }
                true
            }
            "support_base_pattern_spacing" => {
                if let Some(s) = parse_f64(value) {
                    self.support_density = if s > 0.0 { 1.0 / s } else { 0.15 };
                }
                true
            }

            // === Misc ===
            "layer_change_gcode" => {
                // machine-gcode template (parent-profile key; the CLI's
                // set_deserialize path previously dropped it — R169)
                self.layer_change_gcode = value.to_string();
                true
            }
            "before_layer_change_gcode" => {
                self.before_layer_change_gcode = value.to_string();
                true
            }
            "travel_short_distance_acceleration" => {
                if let Some(v) = parse_f64(value.split(',').next().unwrap_or(value)) {
                    self.travel_short_distance_acceleration = v;
                }
                true
            }
            "z_direction_outwall_speed_continuous" => {
                if let Some(v) = parse_bool(value) {
                    self.z_direction_outwall_speed_continuous = v;
                }
                true
            }
            "spiral_mode" => {
                if let Some(v) = parse_bool(value) {
                    self.spiral_vase = v;
                }
                true
            }
            "resolution" => {
                if let Some(v) = parse_f64(value) {
                    self.resolution = v;
                }
                true
            }
            "gcode_flavor" => {
                // Keep Marlin for now (all BambuStudio profiles use marlin)
                true
            }
            "enable_arc_fitting" => {
                if let Some(v) = parse_bool(value) {
                    self.arc_fitting_enabled = v;
                }
                true
            }
            "avoid_crossing_curled_overhangs" => {
                if let Some(v) = parse_bool(value) {
                    self.avoid_crossing_perimeters = v;
                }
                true
            }

            // === Cooling / Fan ===
            "slow_down_for_layer_cooling" => {
                if let Some(v) = parse_bool(value) {
                    self.slow_down_for_layer_cooling = v;
                }
                true
            }
            "slow_down_layer_time" => {
                if let Some(v) = parse_f64(value) {
                    self.slow_down_layer_time = v;
                }
                true
            }
            "slow_down_min_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.slow_down_min_speed = v;
                }
                true
            }
            "fan_min_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.fan_min_speed = v as i32;
                }
                true
            }
            "fan_max_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.fan_max_speed = v as i32;
                }
                true
            }
            "fan_cooling_layer_time" => {
                if let Some(v) = parse_f64(value) {
                    self.fan_cooling_layer_time = v;
                }
                true
            }
            "close_fan_the_first_x_layers" => {
                if let Some(v) = parse_f64(value) {
                    self.close_fan_the_first_x_layers = v as usize;
                }
                true
            }
            "auxiliary_fan" => {
                if let Some(v) = parse_bool(value) {
                    self.auxiliary_fan = v;
                }
                true
            }

            // === Plate temps (handled by apply_bed_temperature, but capture here too) ===
            "cool_plate_temp" => {
                if let Some(v) = parse_f64(value) {
                    self.cool_plate_temp = v as u32;
                }
                true
            }

            // === Acceleration ===
            "initial_layer_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_acceleration = v;
                }
                true
            }
            "initial_layer_travel_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_travel_acceleration = v;
                }
                true
            }
            "travel_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.travel_acceleration = v;
                }
                true
            }
            "machine_max_acceleration_extruding" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_acceleration_extruding = v;
                }
                true
            }

            // === Per-feature printing accelerations (used by GCode::_extrude's
            // "adjust acceleration" block, GCode.cpp:6393-6420). Previously these
            // keys were only handled in apply_key_value(), so the profile values
            // never reached the live PrintConfig (app_slice dispatches via
            // set_deserialize), leaving struct defaults that diverged from native.
            // sparse_infill_acceleration is a percentage of default_acceleration;
            // parse_f64 strips the trailing '%' and keeps the numeric percent. ===
            "default_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.default_acceleration = v;
                }
                true
            }
            "outer_wall_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.outer_wall_acceleration = v;
                }
                true
            }
            "inner_wall_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.inner_wall_acceleration = v;
                }
                true
            }
            "top_surface_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.top_surface_acceleration = v;
                }
                true
            }
            "sparse_infill_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_acceleration = v;
                }
                true
            }

            // === Wipe / prime tower (main.cpp _make_wipe_tower inputs) ===
            // These are consumed only by the psWipeTower phase, which is
            // gated on `enable_prime_tower && multicolour`; single-material
            // and tower-disabled jobs read but do not act on them.
            "enable_prime_tower" => {
                if let Some(v) = parse_bool(value) {
                    self.enable_prime_tower = v;
                }
                true
            }
            "prime_tower_width" => {
                if let Some(v) = parse_f64(value) {
                    self.prime_tower_width = v;
                }
                true
            }
            "wipe_tower_x" => {
                if let Some(v) = parse_f64(value) {
                    self.wipe_tower_x = v;
                }
                true
            }
            "wipe_tower_y" => {
                if let Some(v) = parse_f64(value) {
                    self.wipe_tower_y = v;
                }
                true
            }

            _ => {
                // ZSMOOTH_FAITHFUL: delegate a VETTED set of keys to
                // apply_key_value (R170 audit: the two config-application fns
                // have divergent coverage; a blanket fallback measured +119 —
                // land keys selectively against the oracle).
                if crate::faithful_gate("ZSMOOTH_FAITHFUL")
                    && matches!(
                        key,
                        "initial_layer_infill_speed"
                            | "filament_max_volumetric_speed"
                            | "vertical_shell_speed"
                            | "reduce_fan_stop_start_freq"
                            | "retraction_minimum_travel"
                            | "no_slow_down_for_cooling_on_outwalls"
                    )
                {
                    self.apply_key_value(key, value)
                } else {
                    false
                }
            }
        }
    }
}

impl PrintObjectConfig {
    /// Apply a key-value pair from BambuStudio project_settings JSON.
    /// Returns true if the key was recognized and applied.
    pub fn set_deserialize(&mut self, key: &str, value: &str) -> bool {
        match key {
            // === Layer Heights ===
            "layer_height" => {
                if let Some(v) = parse_f64(value) {
                    self.layer_height = v;
                }
                true
            }
            "initial_layer_print_height" => {
                if let Some(v) = parse_f64(value) {
                    self.first_layer_height = v;
                }
                true
            }

            // === Perimeters ===
            "wall_loops" => {
                if let Some(v) = parse_u32(value) {
                    self.perimeters = v;
                }
                true
            }
            "top_shell_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.top_solid_layers = v;
                }
                true
            }
            "bottom_shell_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.bottom_solid_layers = v;
                }
                true
            }

            // === Infill ===
            "sparse_infill_density" => {
                if let Some(v) = parse_pct(value) {
                    self.fill_density = v;
                }
                true
            }
            "sparse_infill_pattern" => {
                self.fill_pattern = match value {
                    "grid" => InfillPattern::Grid,
                    "line" | "rectilinear" => InfillPattern::Rectilinear,
                    "gyroid" => InfillPattern::Gyroid,
                    "honeycomb" => InfillPattern::Honeycomb,
                    "concentric" => InfillPattern::Concentric,
                    "cubic" => InfillPattern::Cubic,
                    "lightning" => InfillPattern::Lightning,
                    "triangles" => InfillPattern::Triangles,
                    "adaptivecubic" => InfillPattern::AdaptiveCubic,
                    _ => self.fill_pattern,
                };
                true
            }
            "monotonic_travel_into_wall" => {
                if let Some(v) = parse_pct(value) {
                    self.monotonic_travel_into_wall = v;
                }
                true
            }
            "infill_wall_overlap" => {
                if let Some(v) = parse_pct(value) {
                    self.infill_wall_overlap = v;
                }
                true
            }
            "infill_direction" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_angle = v;
                }
                true
            }
            "minimum_sparse_infill_area" => {
                if let Some(v) = parse_f64(value) {
                    self.minimum_sparse_infill_area = v;
                }
                true
            }
            "detect_narrow_internal_solid_infill" => {
                if let Some(v) = parse_bool(value) {
                    self.detect_narrow_internal_solid_infill = v;
                }
                true
            }

            // === Line Widths ===
            "line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.line_width = v;
                }
                true
            }
            "initial_layer_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_line_width = v;
                }
                true
            }
            "outer_wall_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.outer_wall_line_width = v;
                }
                true
            }
            "inner_wall_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.inner_wall_line_width = v;
                }
                true
            }
            "sparse_infill_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_line_width = v;
                }
                true
            }
            "internal_solid_infill_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.solid_infill_line_width = v;
                }
                true
            }
            "top_surface_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.top_surface_line_width = v;
                }
                true
            }

            // === Speeds ===
            "inner_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.perimeter_speed = v;
                }
                true
            }
            "outer_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.external_perimeter_speed = v;
                }
                true
            }
            "sparse_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_speed = v;
                }
                true
            }
            "internal_solid_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.solid_infill_speed = v;
                }
                true
            }
            "top_surface_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.top_solid_infill_speed = v;
                }
                true
            }
            "bridge_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.bridge_speed = v;
                }
                true
            }
            "gap_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.gap_fill_speed = v;
                }
                true
            }

            // === Overhang Speed ===
            // GCode.cpp:5348-5366 overhang_speed_key_map; values may be mm/s or
            // a `%` of the normal wall speed (resolved in the exporter via
            // get_abs_value). We keep the raw value (with `%` stripped) and the
            // exporter applies the percent semantics.
            "enable_overhang_speed" => {
                if let Some(v) = parse_bool(value) {
                    self.enable_overhang_speed = v;
                }
                true
            }
            "enable_overhang_bridge_fan" => {
                // per-filament list "1,1,1,1,1" — first element
                if let Some(v) = parse_bool(value.split(',').next().unwrap_or(value)) {
                    self.enable_overhang_bridge_fan = v;
                }
                true
            }
            "overhang_fan_threshold" => {
                // enum strings (PrintConfig.cpp:1200-1205); may be per-filament list
                let first = value.split(',').next().unwrap_or(value).trim();
                self.overhang_fan_threshold = match first {
                    "0%" => 0,
                    "10%" => 1,
                    "25%" => 2,
                    "50%" => 3,
                    "75%" => 4,
                    "95%" => 5,
                    _ => self.overhang_fan_threshold,
                };
                true
            }
            "overhang_1_4_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_1_4_speed = v;
                }
                true
            }
            "overhang_2_4_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_2_4_speed = v;
                }
                true
            }
            "overhang_3_4_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_3_4_speed = v;
                }
                true
            }
            "overhang_4_4_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_4_4_speed = v;
                }
                true
            }
            "overhang_totally_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_totally_speed = v;
                }
                true
            }
            "smooth_speed_discontinuity_area" => {
                if let Some(v) = parse_bool(value) {
                    self.smooth_speed_discontinuity_area = v;
                }
                true
            }
            "smooth_coefficient" => {
                if let Some(v) = parse_f64(value) {
                    self.smooth_coefficient = v;
                }
                true
            }

            // === Perimeter Options ===
            "detect_thin_wall" => {
                if let Some(v) = parse_bool(value) {
                    self.thin_walls = v;
                }
                true
            }
            "detect_overhang_wall" => {
                if let Some(v) = parse_bool(value) {
                    self.overhangs = v;
                }
                true
            }
            "only_one_wall_first_layer" => {
                if let Some(v) = parse_bool(value) {
                    self.only_one_wall_first_layer = v;
                }
                true
            }
            "top_one_wall_type" => {
                self.top_one_wall_type = match value {
                    "all top" => TopOneWallType::AllTop,
                    "topmost" => TopOneWallType::TopMost,
                    _ => TopOneWallType::None,
                };
                true
            }

            // === Quality ===
            "slice_closing_radius" => {
                if let Some(v) = parse_f64(value) {
                    self.slice_closing_radius = v;
                }
                true
            }
            "xy_hole_compensation" => {
                if let Some(v) = parse_f64(value) {
                    self.xy_size_compensation = v;
                }
                true
            }
            "elefant_foot_compensation" => {
                if let Some(v) = parse_f64(value) {
                    self.elephant_foot_compensation = v;
                }
                true
            }

            // === Flow ===
            "initial_layer_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_flow_ratio = v;
                }
                true
            }
            "top_solid_infill_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.top_solid_infill_flow_ratio = v;
                }
                true
            }
            "print_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.print_flow_ratio = v;
                }
                true
            }

            // === Seam ===
            "seam_position" => {
                self.seam_position = match value {
                    "aligned" => SeamPosition::Aligned,
                    "random" => SeamPosition::Random,
                    "back" | "rear" => SeamPosition::Rear,
                    "nearest" => SeamPosition::Nearest,
                    _ => self.seam_position,
                };
                true
            }
            // PrintConfig.cpp:4650 — coBool, default false.
            "seam_placement_away_from_overhangs" => {
                if let Some(v) = parse_bool(value) {
                    self.seam_placement_away_from_overhangs = v;
                }
                true
            }

            // === Fuzzy Skin ===
            "fuzzy_skin" => {
                self.fuzzy_skin = value == "1"
                    || value == "true"
                    || value == "external"
                    || value == "allwalls"
                    || value == "all";
                true
            }
            "fuzzy_skin_thickness" => {
                if let Some(v) = parse_f64(value) {
                    self.fuzzy_skin_thickness = v;
                }
                true
            }
            "fuzzy_skin_point_distance" | "fuzzy_skin_point_dist" => {
                if let Some(v) = parse_f64(value) {
                    self.fuzzy_skin_point_distance = v;
                }
                true
            }

            // === Wipe ===
            "wipe" => {
                if let Some(v) = parse_bool(value) {
                    self.wipe_enabled = v;
                }
                true
            }
            "wipe_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.wipe_distance = v;
                }
                true
            }

            // === Arachne ===
            "wall_generator" => {
                self.perimeter_mode = match value {
                    "arachne" => PerimeterMode::Arachne,
                    _ => PerimeterMode::Classic,
                };
                true
            }
            "min_bead_width" => {
                if let Some(v) = parse_f64(value) {
                    self.arachne_min_bead_width = v;
                }
                true
            }
            "min_feature_size" => {
                if let Some(v) = parse_f64(value) {
                    self.arachne_min_feature_size = v;
                }
                true
            }
            "wall_transition_length" => {
                if let Some(v) = parse_f64(value) {
                    self.arachne_wall_transition_length = v;
                }
                true
            }

            // === Misc ===
            "spiral_mode" => {
                if let Some(v) = parse_bool(value) {
                    self.spiral_vase = v;
                }
                true
            }

            // === Raft (object-level) ===
            "raft_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.raft_layers = v;
                }
                true
            }
            "raft_expansion" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_expansion = v;
                }
                true
            }
            "raft_contact_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_contact_distance = v;
                }
                true
            }

            // === Support (object-level) ===
            "enable_support" => {
                if let Some(v) = parse_bool(value) {
                    self.enable_support = v;
                }
                true
            }
            "support_type" => {
                if value.contains("tree") {
                    self.support_type = SupportType::Tree;
                } else if value.contains("hybrid") {
                    self.support_type = SupportType::Hybrid;
                } else {
                    self.support_type = SupportType::Normal;
                }
                true
            }
            "enforce_support_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.enforce_support_layers = v;
                }
                true
            }
            "support_threshold_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.support_threshold_angle = v;
                }
                true
            }
            "support_on_build_plate_only" => {
                if let Some(v) = parse_bool(value) {
                    self.support_on_build_plate_only = v;
                }
                true
            }
            "support_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.support_line_width = v;
                }
                true
            }
            "support_base_pattern_spacing" => {
                if let Some(v) = parse_f64(value) {
                    self.support_base_pattern_spacing = v;
                }
                true
            }
            "support_interface_top_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.support_interface_top_layers = v;
                }
                true
            }
            "support_interface_bottom_layers" => {
                if let Some(v) = parse_i32(value) {
                    self.support_interface_bottom_layers = v;
                }
                true
            }
            "support_interface_spacing" => {
                if let Some(v) = parse_f64(value) {
                    self.support_interface_spacing = v;
                }
                true
            }
            "support_top_z_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.support_top_z_distance = v;
                }
                true
            }
            "support_bottom_z_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.support_bottom_z_distance = v;
                }
                true
            }
            "support_object_xy_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.support_object_xy_distance = v;
                }
                true
            }
            "support_expansion" => {
                if let Some(v) = parse_f64(value) {
                    self.support_expansion = v;
                }
                true
            }
            "support_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.support_angle = v;
                }
                true
            }
            "support_object_first_layer_gap" => {
                if let Some(v) = parse_f64(value) {
                    self.support_object_first_layer_gap = v;
                }
                true
            }
            "enable_support_ironing" => {
                if let Some(v) = parse_bool(value) {
                    self.enable_support_ironing = v;
                }
                true
            }
            "support_ironing_pattern" => {
                self.support_ironing_pattern = InfillPattern::from_str_bambu(value);
                true
            }
            "support_ironing_flow" => {
                // coPercent — stored as fraction 0..1
                if let Some(v) = parse_pct(value) {
                    self.support_ironing_flow = v;
                }
                true
            }
            "support_ironing_spacing" => {
                if let Some(v) = parse_f64(value) {
                    self.support_ironing_spacing = v;
                }
                true
            }
            "support_ironing_inset" => {
                if let Some(v) = parse_f64(value) {
                    self.support_ironing_inset = v;
                }
                true
            }
            "support_ironing_direction" => {
                if let Some(v) = parse_f64(value) {
                    self.support_ironing_direction = v;
                }
                true
            }
            "support_ironing_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.support_ironing_speed = v;
                }
                true
            }

            // === Interlocking (PrintConfig.hpp:1008-1013) ===
            "interlocking_beam" => {
                if let Some(v) = parse_bool(value) {
                    self.interlocking_beam = v;
                }
                true
            }
            "interlocking_beam_width" => {
                if let Some(v) = parse_f64(value) {
                    self.interlocking_beam_width = v;
                }
                true
            }
            "interlocking_orientation" => {
                if let Some(v) = parse_f64(value) {
                    self.interlocking_orientation = v;
                }
                true
            }
            "interlocking_beam_layer_count" => {
                if let Ok(v) = value.trim().parse::<i32>() {
                    self.interlocking_beam_layer_count = v;
                }
                true
            }
            "interlocking_depth" => {
                if let Ok(v) = value.trim().parse::<i32>() {
                    self.interlocking_depth = v;
                }
                true
            }
            "interlocking_boundary_avoidance" => {
                if let Ok(v) = value.trim().parse::<i32>() {
                    self.interlocking_boundary_avoidance = v;
                }
                true
            }

            _ => {
                // ZSMOOTH_FAITHFUL: delegate a VETTED set of keys to
                // apply_key_value (R170 audit: the two config-application fns
                // have divergent coverage; a blanket fallback measured +119 —
                // land keys selectively against the oracle).
                if crate::faithful_gate("ZSMOOTH_FAITHFUL")
                    && matches!(
                        key,
                        "initial_layer_infill_speed"
                            | "filament_max_volumetric_speed"
                            | "vertical_shell_speed"
                            | "reduce_fan_stop_start_freq"
                            | "retraction_minimum_travel"
                            | "no_slow_down_for_cooling_on_outwalls"
                    )
                {
                    self.apply_key_value(key, value)
                } else {
                    false
                }
            }
        }
    }
}

impl Default for PrintObjectConfig {
    fn default() -> Self {
        Self {
            layer_height: 0.2,
            first_layer_height: 0.4,
            perimeters: 2,
            // BambuStudio reference: top_shell_layers = 5
            top_solid_layers: 5,
            // BambuStudio reference: bottom_shell_layers = 3
            bottom_solid_layers: 3,
            // BambuStudio reference: sparse_infill_density = 15%
            fill_density: 0.15,
            fill_pattern: InfillPattern::Grid,
            // Per-feature line widths matching BambuStudio reference G-code
            line_width: 0.4,
            initial_layer_line_width: 0.4,
            outer_wall_line_width: 0.4,
            inner_wall_line_width: 0.4,
            sparse_infill_line_width: 0.4,
            solid_infill_line_width: 0.4,
            top_surface_line_width: 0.4,
            // Speeds (BambuStudio reference values)
            perimeter_speed: 300.0,
            external_perimeter_speed: 200.0,
            infill_speed: 350.0,
            solid_infill_speed: 250.0,
            top_solid_infill_speed: 200.0,
            bridge_speed: 50.0,
            gap_fill_speed: 250.0,
            // Perimeter options
            thin_walls: false, // BambuStudio reference: detect_thin_wall = 0
            gap_fill: true,
            overhangs: true,
            only_one_wall_first_layer: false,
            // BambuStudio reference: top_one_wall_type = "all top"
            top_one_wall_type: TopOneWallType::AllTop,
            // Quality
            slice_closing_radius: 0.049,
            xy_size_compensation: 0.0,
            elephant_foot_compensation: 0.0,
            // Infill
            infill_wall_overlap: 0.15,
            monotonic_travel_into_wall: 0.0, // 15% default (BambuStudio default)
            infill_angle: 45.0,
            // Flow
            initial_layer_flow_ratio: 1.0,
            top_solid_infill_flow_ratio: 1.0,
            // PrintConfig.cpp:2068 — print_flow_ratio default is 1.0 (ConfigOptionFloat(1)).
            // The filament-level flow ratio (filament_flow_ratio, 0.98) is applied
            // separately via Extruder::e_per_mm3; print_flow_ratio is an additional
            // object-level multiplier that defaults to 1.0. Hard-coding 0.98 here
            // double-applied the filament flow ratio (0.98^2), under-extruding all
            // features by ~2% relative to C++.
            print_flow_ratio: 1.0,
            // Seam
            seam_position: SeamPosition::Aligned,
            // PrintConfig.cpp:4655 — ConfigOptionBool(false)
            seam_placement_away_from_overhangs: false,
            // Fuzzy skin
            fuzzy_skin: false,
            fuzzy_skin_thickness: 0.3,
            fuzzy_skin_point_distance: 0.8,
            // Wipe
            wipe_enabled: true, // Enable wipe by default (matching BambuStudio)
            wipe_distance: 1.0, // 1mm wipe distance (matching BambuStudio filament_wipe_distance)
            retract_before_wipe: 0.0, // 0% - do all retraction during wipe
            // Arachne. NOTE (R402): C++ `wall_generator` defaults to Arachne
            // (PrintConfig.cpp:5926) whereas this defaults to Classic — a latent
            // faithfulness discrepancy, but it does NOT affect any tested config
            // (Benchy/Cube set wall_generator=classic and Majora sets =arachne
            // explicitly, all via their configs). Left as Classic to avoid an
            // unverified behavioral change; revisit if a config that omits the key
            // is found to diverge from C++.
            perimeter_mode: PerimeterMode::Classic,
            arachne_min_bead_width: 0.1,
            arachne_min_feature_size: 0.1,
            arachne_wall_transition_length: 0.4,
            interface_shells: false,
            spiral_vase: false,
            detect_narrow_internal_solid_infill: true,
            // Minimum sparse infill area (mm²) — 0 = disabled (BambuStudio default)
            minimum_sparse_infill_area: 0.0,
            // Raft (object-level) - BambuStudio PrintObjectConfig defaults
            raft_layers: 0,
            raft_expansion: 3.0,
            raft_contact_distance: 0.15,
            // Support (object-level) - BambuStudio PrintObjectConfig defaults
            enable_support: false,
            support_type: SupportType::Normal,
            enforce_support_layers: 0,
            support_threshold_angle: 45.0,
            support_on_build_plate_only: false,
            support_density: 0.15,
            support_line_width: 0.0, // 0 = use default line_width
            support_base_pattern_spacing: 2.5,
            support_interface_top_layers: 2,
            // C++ default is 0 (PrintConfig.cpp:5169); -1 means "same as top".
            support_interface_bottom_layers: 0,
            support_interface_spacing: 0.5,
            support_top_z_distance: 0.2,
            support_bottom_z_distance: 0.2,
            support_object_xy_distance: 0.35,
            support_expansion: 0.0,
            // Support angle/gap/ironing (C++ PrintConfig.cpp defaults)
            support_angle: 0.0,
            support_object_first_layer_gap: 0.2,
            enable_support_ironing: false,
            support_ironing_pattern: InfillPattern::Rectilinear,
            support_ironing_flow: 0.10, // 10% (coPercent default 10 / 100)
            support_ironing_spacing: 0.1,
            support_ironing_inset: 0.0,
            support_ironing_direction: 0.0,
            support_ironing_speed: 20.0,
            use_relative_e_distances: false,

            // Ironing
            ironing_type: IroningType::NoIroning,
            ironing_flow: 0.15,
            ironing_speed: 15.0,
            ironing_spacing: 0.1,
            ironing_direction: 0.0,

            // Scarf seam
            scarf_seam_type: ScarfSeamType::None,
            scarf_seam_start_height: 0.0,
            scarf_seam_steps: 10,
            scarf_seam_inner_walls: false,
            scarf_seam_entire_loop: false,
            scarf_seam_gap: 0.0,
            scarf_seam_min_length: 0.0,
            scarf_seam_conditional: false,
            scarf_angle_threshold: 0.0,
            // PrintConfig.cpp:4657-4665 — coPercent, default 15 (% of nozzle).
            seam_gap: 15.0,

            // Brim
            brim_type: BrimType::NoBrim,
            brim_width: 0.0,
            brim_object_gap: 0.0,

            // Wall order
            wall_infill_order: WallInfillOrder::InnerOuterInfill,
            is_infill_first: false,

            // Surface patterns
            top_surface_pattern: SurfacePattern::MonotonicLine,
            bottom_surface_pattern: SurfacePattern::Monotonic,
            internal_solid_infill_pattern: InternalSolidInfillPattern::Rectilinear,
            sparse_infill_pattern: InfillPattern::Grid,

            // Bridge
            bridge_flow: 1.0,
            bridge_angle: 0.0,
            thick_bridges: false,
            max_bridge_length: 0.0, // 0 = auto
            bottom_solid_infill_flow_ratio: 1.0,

            // Adaptive layer height
            adaptive_layer_height: false,

            // Layer height limits
            min_layer_height: 0.07,
            max_layer_height: 0.0, // 0 = 75% of nozzle diameter

            // Fuzzy skin type
            fuzzy_skin_type: FuzzySkinType::None,

            // Support (additional)
            support_base_pattern: SupportBasePattern::Rectilinear,
            support_interface_pattern: SupportInterfacePattern::Rectilinear,
            bridge_no_support: false,
            independent_support_layer_height: false,
            support_remove_small_overhang: true,
            top_z_overrides_xy_distance: false,
            support_style: TreeSupportStyle::Default,
            support_interface_loop_pattern: false,
            tree_support_branch_angle: 40.0,
            tree_support_branch_diameter: 5.0,
            tree_support_branch_diameter_angle: 5.0,
            tree_support_branch_distance: 5.0,
            tree_support_wall_count: 0,
            tree_support_with_infill: false,
            tree_support_brim_width: 3.0,

            // Wall settings (additional)
            wall_loops: 2,
            wall_transition_angle: 10.0,
            wall_transition_filter_deviation: 0.25,
            wall_distribution_count: 1,
            precise_outer_wall: false,

            // Overhang speed
            // BambuStudio default is enable_overhang_speed = true (PrintConfig.cpp);
            // the prior `false` left the per-segment overhang-degree speed modulation
            // dormant. Flipped to match the reference so overhang_*_speed take effect.
            enable_overhang_speed: true,
            enable_overhang_bridge_fan: true,
            overhang_fan_threshold: 3,
            overhang_1_4_speed: 0.0,
            overhang_2_4_speed: 0.0,
            overhang_3_4_speed: 0.0,
            overhang_4_4_speed: 0.0,
            overhang_totally_speed: 0.0,
            // BambuStudio default smooth_speed_discontinuity_area = true (coBool).
            smooth_speed_discontinuity_area: true,
            // BambuStudio default smooth_coefficient (PrintConfig.cpp); resolved
            // process value for the H2D PLA profile is 4.
            smooth_coefficient: 80.0,

            // Infill (additional)
            infill_combination: false,
            sparse_infill_anchor: 2.5,
            sparse_infill_anchor_max: 12.0,
            reduce_infill_retraction: false,

            // Hole/contour compensation
            xy_hole_compensation: 0.0,
            xy_contour_compensation: 0.0,

            // Slicing
            slicing_mode: SlicingMode::Regular,
            // C++ default for ensure_vertical_shell_thickness is evtEnabled (PrintConfig.cpp:1792).
            ensure_vertical_shell_thickness: EnsureVerticalShellThickness::All,
            detect_overhang_wall: true,

            // Draft shield
            draft_shield: DraftShield::Disabled,

            // Raft (additional)
            raft_first_layer_density: 0.5,
            raft_first_layer_expansion: 2.0,

            // Shell thickness
            top_shell_thickness: 0.0,
            bottom_shell_thickness: 0.0,

            // Elephant foot
            elephant_foot_min_width: 0.0,

            // Internal bridge
            internal_bridge_support_thickness: 0.0,

            // Embedding
            embedding_wall_into_infill: false,

            // Filter gap fill
            filter_out_gap_fill: 0.0,

            // Skirt
            skirt_height: 1,
            skirt_loops: 0,
            skirt_distance: 2.0,

            // Z-hop
            z_hop: 0.4,

            // Small perimeter
            small_perimeter_speed: 200.0,
            small_perimeter_threshold: 0.0,

            // Support interface line width
            support_interface_line_width: 0.0,

            // Initial layer speed
            initial_layer_speed: 50.0,
            initial_layer_infill_speed: 50.0,

            // Support speed
            support_speed: 150.0,
            support_interface_speed: 80.0,

            // Detect floating
            detect_floating_vertical_shell: false,
            vertical_shell_speed: 80.0,

            // Interlocking (C++ defaults: PrintConfig.cpp:3665-3713)
            interlocking_beam: false,
            interlocking_beam_width: 0.8,
            interlocking_orientation: 22.5,
            interlocking_beam_layer_count: 2,
            interlocking_depth: 2,
            interlocking_boundary_avoidance: 2,
        }
    }
}

impl PrintObjectConfig {
    /// Validate the object configuration with min/max bounds from C++.
    // PrintConfig.cpp:8977  std::map<std::string,std::string> validate(const FullPrintConfig &cfg, bool under_cli)
    // FIDELITY-NOTE: the object-level portion of the C++ free `validate`. The C++ function
    // operates on the merged FullPrintConfig; only the checks whose fields exist on this struct
    // are mirrored. wall_loops/top_shell_layers/bottom_shell_layers map to perimeters /
    // top_solid_layers / bottom_solid_layers which are `u32` here (so the C++ `< 0` checks are
    // statically unreachable and are omitted). The error-map return is collapsed to the original
    // Result<(),String> (first error wins) because there is no DynamicConfig/error-map model.
    pub fn validate(&self) -> Result<(), String> {
        // PrintConfig.cpp:8981  --layer-height: <= 0 invalid
        if self.layer_height <= 0.0 {
            return Err(format!("invalid value {}", self.layer_height));
        }
        // PrintConfig.cpp:8984  else if fabs(fmod(layer_height, SCALING_FACTOR)) > 1e-4 invalid
        else if (self.layer_height % crate::libslic3r::SCALING_FACTOR).abs() > 1e-4 {
            return Err(format!("invalid value {}", self.layer_height));
        }
        // PrintConfig.cpp:8989  --first-layer-height: initial_layer_print_height <= 0 invalid
        if self.first_layer_height <= 0.0 {
            return Err(format!("invalid value {}", self.first_layer_height));
        }
        // PrintConfig.cpp:9075  --bridge-flow-ratio: bridge_flow <= 0 invalid
        if self.bridge_flow <= 0.0 {
            return Err(format!("invalid value {}", self.bridge_flow));
        }
        Ok(())
    }
}

impl fmt::Display for PrintObjectConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PrintObjectConfig(perimeters={}, infill={:.0}%, pattern={:?})",
            self.perimeters,
            self.fill_density * 100.0,
            self.fill_pattern
        )
    }
}

/// Z-hop type for retraction lift moves.
///
/// Faithful mirror of the C++ `enum ZHopType` (PrintConfig.hpp:331-336):
/// `zhtAuto = 0, zhtNormal, zhtSlope, zhtSpiral`. Variant ORDER matches the C++ enum.
/// Config keys: "Auto Lift"/"Normal Lift"/"Slope Lift"/"Spiral Lift"
/// (PrintConfig.cpp:475-480). The C++ default for the `z_hop_types` option is `zhtSpiral`
/// (PrintConfig.cpp:4442); Bambu printer presets override it to "Auto Lift".
///
/// The spiral lift produces a helical CCW arc that simultaneously lifts Z while tracing a
/// circle in XY, avoiding the vertical "zit" from a straight Z-hop. The reference G-code uses
/// `G17` (XY plane select) followed by `G3 Z{hop_z} I{i} J{j} P1` for one full revolution.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ZHopType {
    /// PrintConfig.hpp:332 — zhtAuto. Use spiral lift when available, else normal.
    /// BambuStudio uses this as the default for Bambu printers.
    Auto,
    /// PrintConfig.hpp:333 — zhtNormal. Standard vertical Z-hop using G0 Z moves.
    Normal,
    /// PrintConfig.hpp:334 — zhtSlope. Sloped lift.
    Slope,
    /// PrintConfig.hpp:335 — zhtSpiral. C++ `z_hop_types` option default.
    /// Spiral (helical) Z-hop using G3 arcs with Z component.
    /// Emits: G17 → G3 Z{z} I{i} J{j} P1 F{f}
    #[default]
    Spiral,
}

/// Support structure type.
///
/// FIDELITY-NOTE: the C++ `enum SupportType` (PrintConfig.hpp:161-163) has FOUR values
/// `stNormalAuto, stTreeAuto, stNormal(=manual), stTree(=manual)` with `is_auto()`/`is_tree()`
/// predicates and config keys `normal(auto)/tree(auto)/normal(manual)/tree(manual)`
/// (PrintConfig.cpp:322-327). This crate models support type with the coarser
/// `Normal/Tree/Hybrid` set that is shared across the `support/` subsystem
/// (support_parameters.rs, print_object.rs, model_arrange.rs); collapsing those callers
/// onto the C++ 4-value auto/manual enum is a cross-cutting structural rework and is left
/// for that subsystem rather than re-routed from this config file. The C++ default for the
/// `support_type` option is `stNormalAuto` (PrintConfig.cpp:4998); `Normal` is the closest
/// match here.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SupportType {
    /// Normal/classic support structures (C++ stNormalAuto / stNormal).
    #[default]
    Normal,
    /// Tree-style support structures (C++ stTreeAuto / stTree).
    Tree,
    /// Hybrid support (tree + normal); BBS `hybrid(auto)` handled in handle_legacy
    /// (PrintConfig.cpp:6869), not part of the C++ SupportType enum proper.
    Hybrid,
}

/// Brim type. BambuStudio: `brim_type`.
///
/// Faithful mirror of the C++ `enum BrimType` (PrintConfig.hpp:207-213):
/// `btAutoBrim, btBrimEars, btOuterOnly, btInnerOnly, btOuterAndInner, btNoBrim`.
/// Variant ORDER matches the C++ enum. Config keys per `s_keys_map_BrimType`
/// (PrintConfig.cpp:366-372). C++ option default is `btAutoBrim` (PrintConfig.cpp:1480).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BrimType {
    /// PrintConfig.hpp:208 — btAutoBrim (BBS). C++ option default.
    #[default]
    AutoBrim,
    /// PrintConfig.hpp:209 — btBrimEars (BBS).
    BrimEars,
    /// PrintConfig.hpp:210 — btOuterOnly.
    OuterOnly,
    /// PrintConfig.hpp:211 — btInnerOnly.
    InnerOnly,
    /// PrintConfig.hpp:212 — btOuterAndInner.
    OuterAndInner,
    /// PrintConfig.hpp:213 — btNoBrim.
    NoBrim,
}

impl BrimType {
    /// Parse from BambuStudio config string (s_keys_map_BrimType, PrintConfig.cpp:366-372).
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().replace(' ', "_").as_str() {
            "no_brim" => BrimType::NoBrim,
            "outer_only" => BrimType::OuterOnly,
            "inner_only" => BrimType::InnerOnly,
            "outer_and_inner" | "both" => BrimType::OuterAndInner,
            "auto_brim" | "auto" => BrimType::AutoBrim,
            "brim_ears" => BrimType::BrimEars,
            _ => BrimType::NoBrim,
        }
    }
}

/// Print sequence mode. BambuStudio: `print_sequence`.
/// Print sequence. Mirrors the C++ `enum class PrintSequence`
/// (PrintConfig.hpp:121-126): `ByLayer, ByObject, ByDefault`. C++ option default is
/// `PrintSequence::ByLayer` (PrintConfig.cpp:1557); `s_keys_map_PrintSequence`
/// (PrintConfig.cpp:279-282) defines keys only for "by layer"/"by object".
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PrintSequence {
    /// PrintConfig.hpp:122 — ByLayer. Print all objects layer by layer (C++ default).
    #[default]
    ByLayer,
    /// PrintConfig.hpp:123 — ByObject. Print each object completely before the next.
    ByObject,
    /// PrintConfig.hpp:124 — ByDefault (sentinel; no config key).
    ByDefault,
}

impl PrintSequence {
    /// Parse from BambuStudio config string (s_keys_map_PrintSequence, PrintConfig.cpp:279-282).
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().replace(' ', "_").as_str() {
            "by_object" | "by object" => PrintSequence::ByObject,
            _ => PrintSequence::ByLayer,
        }
    }
}

/// Wall/infill ordering. BambuStudio: `wall_infill_order`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum WallInfillOrder {
    /// Inner walls first, then outer, then infill.
    #[default]
    InnerOuterInfill,
    /// Outer walls first, then inner, then infill.
    OuterInnerInfill,
    /// Infill first, then inner walls, then outer.
    InfillInnerOuter,
    /// Infill first, then outer walls, then inner.
    InfillOuterInner,
    /// Inner/outer/inner sandwich pattern.
    InnerOuterInnerInfill,
}

impl WallInfillOrder {
    /// Parse from BambuStudio config string.
    pub fn from_str_bambu(s: &str) -> Self {
        match s
            .trim()
            .to_lowercase()
            .replace(' ', "_")
            .replace('/', "_")
            .as_str()
        {
            "inner_outer_inner_infill" | "inner/outer/inner" => {
                WallInfillOrder::InnerOuterInnerInfill
            }
            "outer_inner_infill" | "outer/inner" => WallInfillOrder::OuterInnerInfill,
            "infill_inner_outer" | "infill/inner/outer" => WallInfillOrder::InfillInnerOuter,
            "infill_outer_inner" | "infill/outer/inner" => WallInfillOrder::InfillOuterInner,
            _ => WallInfillOrder::InnerOuterInfill,
        }
    }
}

/// Support base pattern. BambuStudio: `support_base_pattern`.
///
/// FIDELITY-NOTE: the C++ `enum SupportMaterialPattern` (PrintConfig.hpp:138-143) has SIX values
/// `smpDefault, smpRectilinear, smpRectilinearGrid, smpHoneycomb, smpLightning, smpNone`
/// (keys: rectilinear, rectilinear-grid, honeycomb, lightning, default, hollow;
/// PrintConfig.cpp:292-298). This crate models a reduced `Rectilinear/Honeycomb/Grid` set used by
/// the `support/` subsystem; expanding it to the full C++ enum is a support-subsystem rework and
/// is not re-routed from this config file. The parser maps the C++ keys it can represent.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SupportBasePattern {
    /// Rectilinear pattern.
    #[default]
    Rectilinear,
    /// Honeycomb pattern.
    Honeycomb,
    /// Grid pattern (for tree support).
    Grid,
}

impl SupportBasePattern {
    /// Parse from BambuStudio config string.
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "honeycomb" => SupportBasePattern::Honeycomb,
            "grid" => SupportBasePattern::Grid,
            _ => SupportBasePattern::Rectilinear,
        }
    }
}

/// Support interface pattern. BambuStudio: `support_interface_pattern`.
///
/// FIDELITY-NOTE: the C++ `enum SupportMaterialInterfacePattern` (PrintConfig.hpp:156-158) has
/// FIVE values `smipAuto, smipRectilinear, smipConcentric, smipRectilinearInterlaced, smipGrid`
/// (keys: auto, rectilinear, concentric, rectilinear_interlaced, grid; PrintConfig.cpp:313-319).
/// This crate models a reduced `Rectilinear/Concentric/Grid` set used by the `support/` subsystem;
/// expanding it to the full C++ enum is a support-subsystem rework, not re-routed from here.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SupportInterfacePattern {
    /// Rectilinear pattern.
    #[default]
    Rectilinear,
    /// Concentric pattern.
    Concentric,
    /// Grid pattern.
    Grid,
}

impl SupportInterfacePattern {
    /// Parse from BambuStudio config string.
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "concentric" => SupportInterfacePattern::Concentric,
            "grid" => SupportInterfacePattern::Grid,
            _ => SupportInterfacePattern::Rectilinear,
        }
    }
}

/// Ironing type. BambuStudio: `ironing_type`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum IroningType {
    /// No ironing.
    #[default]
    NoIroning,
    /// Iron top surfaces.
    TopSurfaces,
    /// Iron topmost surface only.
    TopmostOnly,
    /// Iron all solid surfaces.
    AllSolid,
}

impl IroningType {
    /// Parse from BambuStudio config string.
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().replace(' ', "_").as_str() {
            "top" | "top_surfaces" => IroningType::TopSurfaces,
            "topmost" | "topmost_only" => IroningType::TopmostOnly,
            "solid" | "all_solid" => IroningType::AllSolid,
            _ => IroningType::NoIroning,
        }
    }
}

/// Fuzzy skin type. BambuStudio: `fuzzy_skin`.
///
/// Faithful mirror of the C++ `enum class FuzzySkinType` (PrintConfig.hpp:46-52):
/// `None, External, All, AllWalls, Disabled_fuzzy`. Variant ORDER matches the C++ enum.
/// Config keys per `s_keys_map_FuzzySkinType` (PrintConfig.cpp:183-189) distinguish
/// "all" (=All) from "allwalls" (=AllWalls).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FuzzySkinType {
    /// PrintConfig.hpp:47 — None (disabled).
    #[default]
    None,
    /// PrintConfig.hpp:48 — External (outer walls only).
    External,
    /// PrintConfig.hpp:49 — All (outer walls + holes/contours).
    All,
    /// PrintConfig.hpp:50 — AllWalls.
    AllWalls,
    /// PrintConfig.hpp:51 — Disabled_fuzzy.
    DisabledFuzzy,
}

impl FuzzySkinType {
    /// Parse from BambuStudio config string (s_keys_map_FuzzySkinType, PrintConfig.cpp:183-189).
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "external" => FuzzySkinType::External,
            "all" => FuzzySkinType::All,
            "allwalls" | "all_walls" => FuzzySkinType::AllWalls,
            "disabled_fuzzy" => FuzzySkinType::DisabledFuzzy,
            _ => FuzzySkinType::None,
        }
    }

    /// Returns true if fuzzy skin is enabled.
    pub fn is_enabled(&self) -> bool {
        !matches!(self, FuzzySkinType::None)
    }
}

/// Scarf seam type. BambuStudio: `seam_slope_type`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ScarfSeamType {
    /// Disabled.
    #[default]
    None,
    /// External perimeters only.
    External,
    /// All perimeters.
    All,
}

impl ScarfSeamType {
    /// Parse from BambuStudio config string.
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "external" => ScarfSeamType::External,
            "all" => ScarfSeamType::All,
            _ => ScarfSeamType::None,
        }
    }
}

/// Top surface pattern. BambuStudio: `top_surface_pattern`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SurfacePattern {
    /// Rectilinear.
    #[default]
    Rectilinear,
    /// Monotonic lines.
    Monotonic,
    /// Monotonic lines (variant).
    MonotonicLine,
    /// Concentric.
    Concentric,
    /// Hilbert curve.
    HilbertCurve,
    /// Archimedean chords.
    ArchimedeanChords,
    /// Octagram spiral.
    OctagramSpiral,
}

impl SurfacePattern {
    /// Parse from BambuStudio config string.
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "monotonic" => SurfacePattern::Monotonic,
            "monotonicline" | "monotonic_line" => SurfacePattern::MonotonicLine,
            "concentric" => SurfacePattern::Concentric,
            "hilbertcurve" | "hilbert_curve" => SurfacePattern::HilbertCurve,
            "archimedeanchords" | "archimedean_chords" => SurfacePattern::ArchimedeanChords,
            "octagramspiral" | "octagram_spiral" => SurfacePattern::OctagramSpiral,
            _ => SurfacePattern::Rectilinear,
        }
    }
}

/// Internal solid infill pattern. BambuStudio: `internal_solid_infill_pattern`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum InternalSolidInfillPattern {
    /// Rectilinear.
    #[default]
    Rectilinear,
    /// Monotonic.
    Monotonic,
    /// Monotonic line variant.
    MonotonicLine,
}

impl InternalSolidInfillPattern {
    /// Parse from BambuStudio config string.
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "monotonic" => InternalSolidInfillPattern::Monotonic,
            "monotonicline" | "monotonic_line" => InternalSolidInfillPattern::MonotonicLine,
            _ => InternalSolidInfillPattern::Rectilinear,
        }
    }
}

/// Draft shield type. BambuStudio: `draft_shield`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DraftShield {
    #[default]
    Disabled,
    Limited,
    Enabled,
}

/// Ensure vertical shell thickness mode. BambuStudio: `ensure_vertical_shell_thickness`.
///
/// Mirrors the C++ `enum EnsureVerticalThicknessLevel` (PrintConfig.hpp:83-87):
/// `evtDisabled(=None), evtPartial(=Limited), evtEnabled(=All)`. Ordinals match the C++ enum.
/// C++ option default is `evtEnabled` (PrintConfig.cpp:1792).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EnsureVerticalShellThickness {
    /// PrintConfig.hpp:84 — evtDisabled.
    None,
    /// PrintConfig.hpp:85 — evtPartial.
    Limited,
    /// PrintConfig.hpp:86 — evtEnabled. C++ option default.
    #[default]
    All,
}

impl EnsureVerticalShellThickness {
    /// Parse from BambuStudio config string.
    /// Keys per `s_keys_map_EnsureVerticalThicknessLevel` (PrintConfig.cpp:271-275):
    /// "disabled" (evtDisabled=None), "partial" (evtPartial=Limited), "enabled" (evtEnabled=All).
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            // PrintConfig.cpp:272  "disabled" -> evtDisabled
            "disabled" => EnsureVerticalShellThickness::None,
            // PrintConfig.cpp:273  "partial" -> evtPartial
            "limited" | "partial" => EnsureVerticalShellThickness::Limited,
            // PrintConfig.cpp:274  "enabled" -> evtEnabled
            "all" | "enabled" | "ensure_all" | "1" => EnsureVerticalShellThickness::All,
            _ => EnsureVerticalShellThickness::None,
        }
    }
}

/// Wall sequence (inner/outer ordering). BambuStudio: `wall_sequence`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum WallSequence {
    /// Inner walls first, then outer.
    #[default]
    InnerOuter,
    /// Outer walls first, then inner.
    OuterInner,
    /// Inner/outer/inner sandwich.
    InnerOuterInner,
}

/// Bed temperature formula mode. BambuStudio: `bed_temperature_formula`.
///
/// Faithful mirror of the C++ `enum class BedTempFormula` (PrintConfig.hpp:106-110):
/// `btfFirstFilament, btfHighestTemp` (exactly two values). C++ option default is
/// `btfFirstFilament` (PrintConfig.cpp:2337); keys per `s_keys_map_BedTempFormula`
/// (PrintConfig.cpp:177-180): "by_first_filament", "by_highest_temp".
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BedTempFormula {
    /// PrintConfig.hpp:107 — btfFirstFilament. C++ option default.
    #[default]
    ByFirstFilament,
    /// PrintConfig.hpp:108 — btfHighestTemp.
    ByHighestTemp,
}

/// Timelapse type. BambuStudio: `timelapse_type`.
///
/// Faithful mirror of the C++ `enum TimelapseType : int` (PrintConfig.hpp:216-219):
/// `tlTraditional = 0, tlSmooth`. C++ option default is `tlTraditional` (PrintConfig.cpp:4907);
/// keys per `s_keys_map_TimelapseType` (PrintConfig.cpp:377-380): "0" -> tlTraditional,
/// "1" -> tlSmooth.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TimeLapseType {
    /// PrintConfig.hpp:217 — tlTraditional (key "0"). C++ option default.
    #[default]
    Traditional,
    /// PrintConfig.hpp:218 — tlSmooth (key "1").
    Smooth,
}

/// Slicing mode for mesh processing. BambuStudio: `slicing_mode`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SlicingMode {
    #[default]
    Regular,
    EvenOdd,
    CloseHoles,
}

/// Tree support style.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TreeSupportStyle {
    #[default]
    Default,
    Slim,
    Strong,
    Hybrid,
    Organic,
}

impl TreeSupportStyle {
    /// Parse from BambuStudio config string.
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "slim" | "tree_slim" => TreeSupportStyle::Slim,
            "strong" | "tree_strong" => TreeSupportStyle::Strong,
            "hybrid" | "tree_hybrid" => TreeSupportStyle::Hybrid,
            "organic" | "tree_organic" => TreeSupportStyle::Organic,
            _ => TreeSupportStyle::Default,
        }
    }
}

/// G-code flavor/dialect.
///
/// Faithful mirror of the C++ `enum GCodeFlavor : unsigned char` (PrintConfig.hpp:35-38).
/// The variant ORDER (and therefore each variant's ordinal) matches the C++ enum exactly:
///   0 gcfMarlinLegacy, 1 gcfKlipper, 2 gcfRepRapSprinter, 3 gcfRepRapFirmware, 4 gcfRepetier,
///   5 gcfTeacup, 6 gcfMakerWare, 7 gcfMarlinFirmware, 8 gcfSailfish, 9 gcfMach3,
///   10 gcfMachinekit, 11 gcfSmoothie, 12 gcfNoExtrusion.
/// `Marlin` is the modern Marlin 2.x flavor (= gcfMarlinFirmware).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum GCodeFlavor {
    /// PrintConfig.hpp:36 — gcfMarlinLegacy. C++ default for `gcode_flavor`
    /// (PrintConfig.cpp:3397 `ConfigOptionEnum<GCodeFlavor>(gcfMarlinLegacy)`).
    #[default]
    MarlinLegacy,
    /// PrintConfig.hpp:36 — gcfKlipper.
    Klipper,
    /// PrintConfig.hpp:36 — gcfRepRapSprinter.
    RepRapSprinter,
    /// PrintConfig.hpp:36 — gcfRepRapFirmware.
    RepRapFirmware,
    /// PrintConfig.hpp:36 — gcfRepetier.
    Repetier,
    /// PrintConfig.hpp:36 — gcfTeacup.
    Teacup,
    /// PrintConfig.hpp:36 — gcfMakerWare (MakerBot).
    MakerWare,
    /// PrintConfig.hpp:36 — gcfMarlinFirmware (Marlin 2.x, modern).
    Marlin,
    /// PrintConfig.hpp:36 — gcfSailfish (MakerBot).
    Sailfish,
    /// PrintConfig.hpp:36 — gcfMach3 / LinuxCNC.
    Mach3,
    /// PrintConfig.hpp:36 — gcfMachinekit.
    Machinekit,
    /// PrintConfig.hpp:37 — gcfSmoothie.
    Smoothie,
    /// PrintConfig.hpp:37 — gcfNoExtrusion (CNC/laser).
    NoExtrusion,
}

/// Infill pattern type.
///
/// Faithful mirror of the C++ `enum InfillPattern : int` (PrintConfig.hpp:76-81).
/// The variant ORDER (and therefore the `as u32` ordinal of each variant)
/// matches the C++ enum exactly — `SurfaceFillParams::operator<` (Fill.cpp:89)
/// compares `unsigned(pattern)`, so the ordinals are load-bearing for the
/// fill processing order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum InfillPattern {
    /// PrintConfig.hpp:77 — ipConcentric (= InfillPattern(0)).
    Concentric,
    /// PrintConfig.hpp:77 — ipRectilinear.
    Rectilinear,
    /// PrintConfig.hpp:77 — ipGrid.
    /// (#\[default\] is a Rust-struct convenience; the C++ config default for
    /// `sparse_infill_pattern` is set per option in PrintConfig.cpp.)
    #[default]
    Grid,
    /// PrintConfig.hpp:77 — ipLine.
    Line,
    /// PrintConfig.hpp:77 — ipCubic.
    Cubic,
    /// PrintConfig.hpp:77 — ipTriangles.
    Triangles,
    /// PrintConfig.hpp:77 — ipStars (config key "tri-hexagon").
    Stars,
    /// PrintConfig.hpp:77 — ipGyroid.
    Gyroid,
    /// PrintConfig.hpp:77 — ipHoneycomb.
    Honeycomb,
    /// PrintConfig.hpp:77 — ipAdaptiveCubic.
    AdaptiveCubic,
    /// PrintConfig.hpp:77 — ipMonotonic.
    Monotonic,
    /// PrintConfig.hpp:77 — ipMonotonicLine.
    MonotonicLine,
    /// PrintConfig.hpp:77 — ipAlignedRectilinear.
    AlignedRectilinear,
    /// PrintConfig.hpp:77 — ip3DHoneycomb.
    Honeycomb3D,
    /// PrintConfig.hpp:78 — ipHilbertCurve.
    HilbertCurve,
    /// PrintConfig.hpp:78 — ipArchimedeanChords.
    ArchimedeanChords,
    /// PrintConfig.hpp:78 — ipOctagramSpiral.
    OctagramSpiral,
    /// PrintConfig.hpp:78 — ipSupportCubic.
    SupportCubic,
    /// PrintConfig.hpp:78 — ipSupportBase.
    SupportBase,
    /// PrintConfig.hpp:78 — ipConcentricInternal (BBS: internal solid infill only).
    ConcentricInternal,
    /// PrintConfig.hpp:79 — ipLightning.
    Lightning,
    /// PrintConfig.hpp:79 — ipCrossHatch.
    CrossHatch,
    /// PrintConfig.hpp:79 — ipZigZag.
    ZigZag,
    /// PrintConfig.hpp:79 — ipCrossZag.
    CrossZag,
    /// PrintConfig.hpp:79 — ipFloatingConcentric.
    FloatingConcentric,
    /// PrintConfig.hpp:79 — ipLockedZag.
    LockedZag,
    /// PrintConfig.hpp:79 — ip2DLattice.
    Lattice2D,
}

impl InfillPattern {
    /// Number of patterns; mirrors the C++ `ipCount` sentinel
    /// (PrintConfig.hpp:80). Kept as a constant instead of a Rust enum
    /// variant so that `match` arms stay exhaustive over real patterns.
    pub const COUNT: usize = 27;

    /// Parse from BambuStudio config string.
    ///
    /// Canonical keys follow `s_keys_map_InfillPattern`
    /// (PrintConfig.cpp:208-233); note the legacy quirks "zig-zag" ->
    /// ipRectilinear and "tri-hexagon" -> ipStars. A few snake_case aliases
    /// accepted by earlier revisions of this crate are kept after the
    /// canonical keys.
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            // PrintConfig.cpp:209
            "concentric" => InfillPattern::Concentric,
            // PrintConfig.cpp:210 — legacy serialization of ipRectilinear.
            "zig-zag" | "rectilinear" => InfillPattern::Rectilinear,
            // PrintConfig.cpp:211
            "grid" => InfillPattern::Grid,
            // PrintConfig.cpp:212
            "line" => InfillPattern::Line,
            // PrintConfig.cpp:213
            "cubic" => InfillPattern::Cubic,
            // PrintConfig.cpp:214
            "triangles" => InfillPattern::Triangles,
            // PrintConfig.cpp:215
            "tri-hexagon" => InfillPattern::Stars,
            // PrintConfig.cpp:216
            "gyroid" => InfillPattern::Gyroid,
            // PrintConfig.cpp:217
            "honeycomb" => InfillPattern::Honeycomb,
            // PrintConfig.cpp:218
            "adaptivecubic" | "adaptive_cubic" => InfillPattern::AdaptiveCubic,
            // PrintConfig.cpp:219
            "monotonic" => InfillPattern::Monotonic,
            // PrintConfig.cpp:220
            "monotonicline" | "monotonic_line" => InfillPattern::MonotonicLine,
            // PrintConfig.cpp:221
            "alignedrectilinear" | "aligned_rectilinear" => InfillPattern::AlignedRectilinear,
            // PrintConfig.cpp:222
            "3dhoneycomb" | "honeycomb3d" => InfillPattern::Honeycomb3D,
            // PrintConfig.cpp:223
            "hilbertcurve" | "hilbert_curve" => InfillPattern::HilbertCurve,
            // PrintConfig.cpp:224
            "archimedeanchords" | "archimedean_chords" => InfillPattern::ArchimedeanChords,
            // PrintConfig.cpp:225
            "octagramspiral" | "octagram_spiral" => InfillPattern::OctagramSpiral,
            // PrintConfig.cpp:226
            "supportcubic" | "support_cubic" => InfillPattern::SupportCubic,
            // PrintConfig.cpp:227
            "lightning" => InfillPattern::Lightning,
            // PrintConfig.cpp:228
            "crosshatch" | "cross" => InfillPattern::CrossHatch,
            // PrintConfig.cpp:229
            "zigzag" | "zig_zag" => InfillPattern::ZigZag,
            // PrintConfig.cpp:230
            "crosszag" => InfillPattern::CrossZag,
            // PrintConfig.cpp:231
            "lockedzag" => InfillPattern::LockedZag,
            // PrintConfig.cpp:232
            "2dlattice" => InfillPattern::Lattice2D,
            _ => InfillPattern::Grid,
        }
    }
}

/// Seam position preference.
///
/// Faithful mirror of the C++ `enum SeamPosition` (PrintConfig.hpp:177-179):
/// `spNearest, spAligned, spRear, spRandom`. Variant ORDER matches the C++ enum
/// (no extra variants; C++ has exactly these four). Default is `spAligned`
/// (PrintConfig.cpp:4648 `ConfigOptionEnum<SeamPosition>(spAligned)`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SeamPosition {
    /// PrintConfig.hpp:178 — spNearest. Nearest to previous layer's seam / corner.
    Nearest,
    /// PrintConfig.hpp:178 — spAligned. C++ option default.
    #[default]
    Aligned,
    /// PrintConfig.hpp:178 — spRear. Rear of the model.
    Rear,
    /// PrintConfig.hpp:178 — spRandom. Random seam position.
    Random,
}

/// Perimeter generation mode.
/// Controls whether top surfaces use only one wall (perimeter).
///
/// BambuStudio reference: `TopOneWallType` in `PrintConfig.hpp`
/// - `None`: Disabled — all layers use the configured number of perimeters.
/// - `TopMost`: Only the topmost layer (no upper slices) uses 1 perimeter.
/// - `AllTop`: All layers that contain top surfaces use 1 perimeter for the
///   top-surface regions. The topmost layer (no upper slices) also uses 1 perimeter.
///
/// Reference G-code header: `; top_one_wall_type = all top`
/// Faithful mirror of the C++ `enum class TopOneWallType` (PrintConfig.hpp:234-239):
/// `None, Alltop, Topmost`. Variant ORDER matches the C++ enum. Keys per
/// `s_keys_map_TopOneWallType` (PrintConfig.cpp:245-249): "not apply"->None, "all top"->Alltop,
/// "topmost"->Topmost. C++ option default is `Alltop` (PrintConfig.cpp:1286).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopOneWallType {
    /// PrintConfig.hpp:236 — None. Disabled; use configured wall_loops everywhere.
    None,
    /// PrintConfig.hpp:237 — Alltop. All layers with top surfaces use 1 perimeter for top
    /// regions; the topmost layer always uses 1 perimeter. C++ option default.
    #[default]
    AllTop,
    /// PrintConfig.hpp:238 — Topmost. Only the topmost layer uses 1 perimeter.
    TopMost,
}

impl TopOneWallType {
    // Returns true if this setting is enabled (not None).
    pub fn is_enabled(&self) -> bool {
        !matches!(self, TopOneWallType::None)
    }

    /// Returns true if this affects all top-surface layers (not just the topmost).
    pub fn is_all_top(&self) -> bool {
        matches!(self, TopOneWallType::AllTop)
    }

    /// Parse from BambuStudio config string.
    /// BambuStudio uses: "none", "topmost", "all top"
    pub fn from_str_bambu(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "all top" | "alltop" | "all_top" => TopOneWallType::AllTop,
            "topmost" | "top most" | "top_most" => TopOneWallType::TopMost,
            _ => TopOneWallType::None,
        }
    }
}

///
/// Controls how perimeters (walls) are generated for each layer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PerimeterMode {
    /// Classic fixed-width perimeters using polygon offset.
    /// Each perimeter has a constant extrusion width.
    #[default]
    Classic,

    /// Arachne variable-width perimeters.
    /// Adapts extrusion width based on local geometry to better fill
    /// thin walls and narrow features. This produces higher quality
    /// prints for models with thin features.
    Arachne,
}

impl PerimeterMode {
    // Returns true if this mode uses variable-width extrusion.
    pub fn is_variable_width(&self) -> bool {
        matches!(self, PerimeterMode::Arachne)
    }

    /// Returns the display name for this mode.
    pub fn name(&self) -> &'static str {
        match self {
            PerimeterMode::Classic => "Classic",
            PerimeterMode::Arachne => "Arachne",
        }
    }
}

// === Key-Value Config Serialization/Deserialization ===
//
// BambuStudio uses a simple key = value format for config files
// (project_settings.config). These methods convert to/from that format.

use std::collections::HashMap;

impl PrintConfig {
    /// Serialize to BambuStudio key-value format.
    /// Returns a HashMap of key -> value strings.
    pub fn to_key_value_map(&self) -> HashMap<String, String> {
        let mut m = HashMap::new();
        m.insert("layer_height".into(), format!("{}", self.layer_height));
        m.insert(
            "initial_layer_print_height".into(),
            format!("{}", self.first_layer_height),
        );
        m.insert(
            "nozzle_diameter".into(),
            format!("{}", self.nozzle_diameter),
        );
        m.insert(
            "filament_diameter".into(),
            format!("{}", self.filament_diameter),
        );
        m.insert("travel_speed".into(), format!("{}", self.travel_speed));
        m.insert(
            "outer_wall_speed".into(),
            format!("{}", self.outer_wall_speed),
        );
        m.insert(
            "inner_wall_speed".into(),
            format!("{}", self.inner_wall_speed),
        );
        m.insert(
            "sparse_infill_speed".into(),
            format!("{}", self.sparse_infill_speed),
        );
        m.insert(
            "internal_solid_infill_speed".into(),
            format!("{}", self.internal_solid_infill_speed),
        );
        m.insert(
            "top_surface_speed".into(),
            format!("{}", self.top_surface_speed),
        );
        m.insert("bridge_speed".into(), format!("{}", self.bridge_speed));
        m.insert(
            "gap_infill_speed".into(),
            format!("{}", self.gap_infill_speed),
        );
        m.insert(
            "initial_layer_speed".into(),
            format!("{}", self.first_layer_speed),
        );
        m.insert(
            "retraction_length".into(),
            format!("{}", self.retract_length),
        );
        m.insert("retraction_speed".into(), format!("{}", self.retract_speed));
        m.insert("z_hop".into(), format!("{}", self.retract_lift));
        m.insert(
            "deretraction_speed".into(),
            format!("{}", self.deretract_speed),
        );
        m.insert(
            "nozzle_temperature".into(),
            format!("{}", self.nozzle_temperature),
        );
        m.insert(
            "nozzle_temperature_initial_layer".into(),
            format!("{}", self.nozzle_temperature_initial_layer),
        );
        m.insert(
            "filament_flow_ratio".into(),
            format!("{}", self.filament_flow_ratio),
        );
        m.insert(
            "filament_max_volumetric_speed".into(),
            format!("{}", self.filament_max_volumetric_speed),
        );
        m.insert("filament_type".into(), self.filament_type.clone());
        m.insert(
            "enable_arc_fitting".into(),
            if self.arc_fitting_enabled {
                "1".into()
            } else {
                "0".into()
            },
        );
        m.insert(
            "spiral_mode".into(),
            if self.spiral_vase {
                "1".into()
            } else {
                "0".into()
            },
        );
        m.insert(
            "enable_support".into(),
            if self.support_enabled {
                "1".into()
            } else {
                "0".into()
            },
        );
        m.insert(
            "print_sequence".into(),
            match self.print_sequence {
                PrintSequence::ByObject => "by object".into(),
                // ByDefault is a sentinel with no C++ config key; serialize as by layer
                // (the C++ default), matching s_keys_map_PrintSequence behavior.
                PrintSequence::ByLayer | PrintSequence::ByDefault => "by layer".into(),
            },
        );
        // Acceleration
        m.insert(
            "default_acceleration".into(),
            format!("{}", self.default_acceleration),
        );
        m.insert(
            "outer_wall_acceleration".into(),
            format!("{}", self.outer_wall_acceleration),
        );
        m.insert(
            "inner_wall_acceleration".into(),
            format!("{}", self.inner_wall_acceleration),
        );
        m.insert(
            "top_surface_acceleration".into(),
            format!("{}", self.top_surface_acceleration),
        );
        m.insert(
            "sparse_infill_acceleration".into(),
            format!("{}", self.sparse_infill_acceleration),
        );
        m.insert(
            "initial_layer_acceleration".into(),
            format!("{}", self.initial_layer_acceleration),
        );
        m.insert(
            "travel_acceleration".into(),
            format!("{}", self.travel_acceleration),
        );
        // Jerk
        m.insert("default_jerk".into(), format!("{}", self.default_jerk));
        m.insert(
            "outer_wall_jerk".into(),
            format!("{}", self.outer_wall_jerk),
        );
        m.insert(
            "inner_wall_jerk".into(),
            format!("{}", self.inner_wall_jerk),
        );
        m.insert("infill_jerk".into(), format!("{}", self.infill_jerk));
        m.insert("travel_jerk".into(), format!("{}", self.travel_jerk));
        // Fan
        m.insert("fan_min_speed".into(), format!("{}", self.fan_min_speed));
        m.insert("fan_max_speed".into(), format!("{}", self.fan_max_speed));
        m.insert(
            "close_fan_the_first_x_layers".into(),
            format!("{}", self.close_fan_the_first_x_layers),
        );
        m
    }

    /// Apply settings from a key-value map (BambuStudio format).
    /// Unknown keys are silently ignored.
    pub fn apply_key_value_map(&mut self, map: &HashMap<String, String>) {
        for (key, value) in map {
            self.apply_key_value(key, value);
        }
    }

    /// Apply a single key-value setting.
    /// Returns true if the key was recognized.
    pub fn apply_key_value(&mut self, key: &str, value: &str) -> bool {
        let parse_f64 =
            |s: &str| -> Option<f64> { s.trim().trim_end_matches('%').parse::<f64>().ok() };
        let parse_u32 = |s: &str| -> Option<u32> { s.trim().parse::<u32>().ok() };
        let parse_bool =
            |s: &str| -> bool { s.trim() == "1" || s.trim().eq_ignore_ascii_case("true") };

        match key {
            "layer_height" => {
                if let Some(v) = parse_f64(value) {
                    self.layer_height = v;
                }
            }
            "initial_layer_print_height" | "first_layer_height" => {
                if let Some(v) = parse_f64(value) {
                    self.first_layer_height = v;
                }
            }
            "nozzle_diameter" => {
                if let Some(v) = parse_f64(value) {
                    self.nozzle_diameter = v;
                }
            }
            "filament_diameter" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_diameter = v;
                }
            }
            "travel_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.travel_speed = v;
                }
            }
            "outer_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.outer_wall_speed = v;
                    self.print_speed = v;
                }
            }
            "inner_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.inner_wall_speed = v;
                }
            }
            "sparse_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_speed = v;
                }
            }
            "internal_solid_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.internal_solid_infill_speed = v;
                }
            }
            "top_surface_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.top_surface_speed = v;
                }
            }
            "bridge_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.bridge_speed = v;
                }
            }
            "gap_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.gap_infill_speed = v;
                }
            }
            "initial_layer_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.first_layer_speed = v;
                }
            }
            "retraction_length" | "filament_retraction_length" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_length = v;
                    self.filament_retraction_length = v;
                }
            }
            "retraction_speed" | "filament_retraction_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_speed = v;
                    self.filament_retraction_speed = v;
                }
            }
            "deretraction_speed" | "filament_deretraction_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.deretract_speed = v;
                    self.filament_deretraction_speed = v;
                }
            }
            "z_hop" | "filament_z_hop" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_lift = v;
                    self.filament_z_hop = v;
                }
            }
            "retraction_minimum_travel" | "filament_retraction_minimum_travel" => {
                if let Some(v) = parse_f64(value) {
                    self.retract_before_travel = v;
                    self.filament_retraction_minimum_travel = v;
                }
            }
            "filament_wipe_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_wipe_distance = v;
                }
            }
            "nozzle_temperature" => {
                if let Some(v) = parse_f64(value) {
                    self.nozzle_temperature = v as u32;
                    self.extruder_temperature = v as u32;
                }
            }
            "nozzle_temperature_initial_layer" => {
                if let Some(v) = parse_f64(value) {
                    self.nozzle_temperature_initial_layer = v as u32;
                    self.first_layer_extruder_temperature = v as u32;
                }
            }
            "filament_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_flow_ratio = v;
                }
            }
            "filament_max_volumetric_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_max_volumetric_speed = v;
                }
            }
            "filament_type" => {
                self.filament_type = value.trim().to_string();
            }
            "filament_density" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_density = v;
                }
            }
            "filament_cost" => {
                if let Some(v) = parse_f64(value) {
                    self.filament_cost = v;
                }
            }
            "enable_arc_fitting" => {
                self.arc_fitting_enabled = parse_bool(value);
            }
            "spiral_mode" => {
                self.spiral_vase = parse_bool(value);
            }
            "enable_support" => {
                self.support_enabled = parse_bool(value);
            }
            "support_type" => {
                if value.contains("tree") {
                    self.support_type = SupportType::Tree;
                } else if value.contains("hybrid") {
                    self.support_type = SupportType::Hybrid;
                } else {
                    self.support_type = SupportType::Normal;
                }
            }
            "support_threshold_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.support_threshold_angle = v;
                }
            }
            "print_sequence" => {
                self.print_sequence = PrintSequence::from_str_bambu(value);
            }
            "default_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.default_acceleration = v;
                }
            }
            "outer_wall_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.outer_wall_acceleration = v;
                }
            }
            "inner_wall_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.inner_wall_acceleration = v;
                }
            }
            "top_surface_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.top_surface_acceleration = v;
                }
            }
            "sparse_infill_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_acceleration = v;
                }
            }
            "initial_layer_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_acceleration = v;
                }
            }
            "travel_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.travel_acceleration = v;
                }
            }
            "initial_layer_travel_acceleration" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_travel_acceleration = v;
                }
            }
            "default_jerk" => {
                if let Some(v) = parse_f64(value) {
                    self.default_jerk = v;
                }
            }
            "outer_wall_jerk" => {
                if let Some(v) = parse_f64(value) {
                    self.outer_wall_jerk = v;
                }
            }
            "inner_wall_jerk" => {
                if let Some(v) = parse_f64(value) {
                    self.inner_wall_jerk = v;
                }
            }
            "top_surface_jerk" => {
                if let Some(v) = parse_f64(value) {
                    self.top_surface_jerk = v;
                }
            }
            "infill_jerk" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_jerk = v;
                }
            }
            "initial_layer_jerk" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_jerk = v;
                }
            }
            "travel_jerk" => {
                if let Some(v) = parse_f64(value) {
                    self.travel_jerk = v;
                }
            }
            "fan_min_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.fan_min_speed = v as i32;
                }
            }
            "fan_max_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.fan_max_speed = v as i32;
                }
            }
            "close_fan_the_first_x_layers" => {
                if let Some(v) = parse_f64(value) {
                    self.close_fan_the_first_x_layers = v as usize;
                }
            }
            "slow_down_for_layer_cooling" => {
                self.slow_down_for_layer_cooling = parse_bool(value);
            }
            "slow_down_layer_time" => {
                if let Some(v) = parse_f64(value) {
                    self.slow_down_layer_time = v;
                }
            }
            "slow_down_min_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.slow_down_min_speed = v;
                }
            }
            "fan_cooling_layer_time" => {
                if let Some(v) = parse_f64(value) {
                    self.fan_cooling_layer_time = v;
                }
            }
            "skirt_loops" => {
                if let Some(v) = parse_u32(value) {
                    self.skirt_loops = v;
                }
            }
            "skirt_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.skirt_distance = v;
                }
            }
            "brim_width" => {
                if let Some(v) = parse_f64(value) {
                    self.brim_width = v;
                }
            }
            "raft_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.raft_layers = v;
                    self.raft_enabled = v > 0;
                }
            }
            "raft_expansion" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_expansion = v;
                }
            }
            "raft_contact_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_contact_distance = v;
                }
            }
            "resolution" => {
                if let Some(v) = parse_f64(value) {
                    self.resolution = v;
                }
            }
            "machine_start_gcode" => {
                self.machine_start_gcode = value.to_string();
            }
            "machine_end_gcode" => {
                self.machine_end_gcode = value.to_string();
            }
            "before_layer_change_gcode" => {
                self.before_layer_change_gcode = value.to_string();
            }
            "layer_change_gcode" => {
                self.layer_change_gcode = value.to_string();
            }
            "change_filament_gcode" => {
                self.change_filament_gcode = value.to_string();
            }
            "tool_change_gcode" => {
                self.tool_change_gcode = value.to_string();
            }
            "filament_start_gcode" => {
                self.filament_start_gcode = value.to_string();
            }
            "filament_end_gcode" => {
                self.filament_end_gcode = value.to_string();
            }
            "enable_timelapse" => {
                self.enable_timelapse = parse_bool(value);
            }
            "timelapse_type" => {
                if let Some(v) = parse_u32(value) {
                    self.timelapse_type = v;
                }
            }
            "enable_prime_tower" => {
                self.enable_prime_tower = parse_bool(value);
            }
            "prime_tower_width" => {
                if let Some(v) = parse_f64(value) {
                    self.prime_tower_width = v;
                }
            }
            "flush_into_infill" => {
                self.flush_into_infill = parse_bool(value);
            }
            "flush_into_objects" => {
                self.flush_into_objects = parse_bool(value);
            }
            "flush_into_support" => {
                self.flush_into_support = parse_bool(value);
            }
            "enable_pressure_advance" => {
                self.enable_pressure_advance = parse_bool(value);
            }
            "pressure_advance" => {
                if let Some(v) = parse_f64(value) {
                    self.pressure_advance = v;
                }
            }
            "printable_height" => {
                if let Some(v) = parse_f64(value) {
                    self.printable_height = v;
                }
            }
            "curr_bed_type" => {
                self.curr_bed_type = value.trim().to_string();
            }
            "cool_plate_temp" => {
                if let Some(v) = parse_f64(value) {
                    self.cool_plate_temp = v as u32;
                }
            }
            "cool_plate_temp_initial_layer" => {
                if let Some(v) = parse_f64(value) {
                    self.cool_plate_temp_initial_layer = v as u32;
                }
            }
            "hot_plate_temp" => {
                if let Some(v) = parse_f64(value) {
                    self.hot_plate_temp = v as u32;
                }
            }
            "hot_plate_temp_initial_layer" => {
                if let Some(v) = parse_f64(value) {
                    self.hot_plate_temp_initial_layer = v as u32;
                }
            }
            "eng_plate_temp" => {
                if let Some(v) = parse_f64(value) {
                    self.eng_plate_temp = v as u32;
                }
            }
            "eng_plate_temp_initial_layer" => {
                if let Some(v) = parse_f64(value) {
                    self.eng_plate_temp_initial_layer = v as u32;
                }
            }
            "textured_plate_temp" => {
                if let Some(v) = parse_f64(value) {
                    self.textured_plate_temp = v as u32;
                }
            }
            "textured_plate_temp_initial_layer" => {
                if let Some(v) = parse_f64(value) {
                    self.textured_plate_temp_initial_layer = v as u32;
                }
            }
            "chamber_temperature" | "chamber_temperatures" => {
                if let Some(v) = parse_f64(value) {
                    self.chamber_temperature = v as u32;
                }
            }
            "enable_long_retraction_when_cut" => {
                self.enable_long_retraction_when_cut = parse_bool(value);
            }
            "retraction_distances_when_cut" => {
                if let Some(v) = parse_f64(value) {
                    self.retraction_distances_when_cut = v;
                }
            }
            "reduce_crossing_wall" => {
                self.reduce_crossing_wall = parse_bool(value);
            }
            "max_travel_detour_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.max_travel_detour_distance = v;
                }
            }
            "max_print_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.max_print_speed = v;
                }
            }
            "exclude_object" => {
                self.exclude_object = parse_bool(value);
            }
            "gcode_add_line_number" => {
                self.gcode_add_line_number = parse_bool(value);
            }
            "use_firmware_retraction" => {
                self.use_firmware_retraction = parse_bool(value);
            }
            "enable_silent" => {
                self.enable_silent = parse_bool(value);
            }
            "accel_to_decel_enable" => {
                self.accel_to_decel_enable = parse_bool(value);
            }
            "accel_to_decel_factor" => {
                if let Some(v) = parse_f64(value) {
                    self.accel_to_decel_factor = v;
                }
            }
            "support_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.support_speed = v;
                }
            }
            "support_interface_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.support_interface_speed = v;
                }
            }
            "small_perimeter_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.small_perimeter_speed = v;
                }
            }
            "initial_layer_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_infill_speed = v;
                }
            }
            "filament_retract_when_changing_layer" => {
                self.filament_retract_when_changing_layer = parse_bool(value);
            }
            "auxiliary_fan" => {
                self.auxiliary_fan = parse_bool(value);
            }
            "machine_max_acceleration_x" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_acceleration_x = v;
                }
            }
            "machine_max_acceleration_y" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_acceleration_y = v;
                }
            }
            "machine_max_acceleration_z" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_acceleration_z = v;
                }
            }
            "machine_max_acceleration_e" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_acceleration_e = v;
                }
            }
            "machine_max_speed_x" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_speed_x = v;
                }
            }
            "machine_max_speed_y" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_speed_y = v;
                }
            }
            "machine_max_speed_z" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_speed_z = v;
                }
            }
            "machine_max_speed_e" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_speed_e = v;
                }
            }
            "machine_max_jerk_x" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_jerk_x = v;
                }
            }
            "machine_max_jerk_y" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_jerk_y = v;
                }
            }
            "machine_max_jerk_z" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_jerk_z = v;
                }
            }
            "machine_max_jerk_e" => {
                if let Some(v) = parse_f64(value) {
                    self.machine_max_jerk_e = v;
                }
            }
            "travel_speed_z" => {
                if let Some(v) = parse_f64(value) {
                    self.travel_speed_z = v;
                }
            }
            "max_volumetric_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.max_volumetric_speed = v;
                }
            }
            "scan_first_layer" => {
                self.scan_first_layer = parse_bool(value);
            }
            "single_extruder_multi_material" => {
                self.single_extruder_multi_material = parse_bool(value);
            }
            "support_filament" => {
                if let Some(v) = parse_u32(value) {
                    self.support_filament = v;
                }
            }
            "support_interface_filament" => {
                if let Some(v) = parse_u32(value) {
                    self.support_interface_filament = v;
                }
            }
            "filament_is_support" => {
                self.filament_is_support = parse_bool(value);
            }
            "filament_soluble" => {
                self.filament_soluble = parse_bool(value);
            }
            "retract_when_changing_layer" => {
                self.retract_when_changing_layer = parse_bool(value);
            }
            "wipe_tower_x" => {
                if let Some(v) = parse_f64(value) {
                    self.wipe_tower_x = v;
                }
            }
            "wipe_tower_y" => {
                if let Some(v) = parse_f64(value) {
                    self.wipe_tower_y = v;
                }
            }
            "enable_wrapping_detection" => {
                self.enable_wrapping_detection = parse_bool(value);
            }
            "time_lapse_gcode" => {
                self.time_lapse_gcode = value.to_string();
            }
            "wrapping_detection_gcode" => {
                self.wrapping_detection_gcode = value.to_string();
            }
            _ => return false,
        }
        true
    }
}

impl PrintObjectConfig {
    /// Apply a single key-value setting (BambuStudio format).
    /// Returns true if the key was recognized.
    pub fn apply_key_value(&mut self, key: &str, value: &str) -> bool {
        let parse_f64 =
            |s: &str| -> Option<f64> { s.trim().trim_end_matches('%').parse::<f64>().ok() };
        let parse_pct = |s: &str| -> Option<f64> {
            let s = s.trim();
            if s.ends_with('%') {
                s.trim_end_matches('%')
                    .parse::<f64>()
                    .ok()
                    .map(|v| v / 100.0)
            } else {
                s.parse::<f64>().ok()
            }
        };
        let parse_u32 = |s: &str| -> Option<u32> { s.trim().parse::<u32>().ok() };
        let parse_bool =
            |s: &str| -> bool { s.trim() == "1" || s.trim().eq_ignore_ascii_case("true") };

        match key {
            "layer_height" => {
                if let Some(v) = parse_f64(value) {
                    self.layer_height = v;
                }
            }
            "initial_layer_print_height" | "first_layer_height" => {
                if let Some(v) = parse_f64(value) {
                    self.first_layer_height = v;
                }
            }
            "wall_loops" => {
                if let Some(v) = parse_u32(value) {
                    self.perimeters = v;
                    self.wall_loops = v;
                }
            }
            "top_shell_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.top_solid_layers = v;
                }
            }
            "bottom_shell_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.bottom_solid_layers = v;
                }
            }
            "sparse_infill_density" => {
                if let Some(v) = parse_pct(value) {
                    self.fill_density = v;
                }
            }
            "sparse_infill_pattern" => {
                self.fill_pattern = InfillPattern::from_str_bambu(value);
                self.sparse_infill_pattern = InfillPattern::from_str_bambu(value);
            }
            "line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.line_width = v;
                }
            }
            "initial_layer_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_line_width = v;
                }
            }
            "outer_wall_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.outer_wall_line_width = v;
                }
            }
            "inner_wall_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.inner_wall_line_width = v;
                }
            }
            "sparse_infill_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_line_width = v;
                }
            }
            "internal_solid_infill_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.solid_infill_line_width = v;
                }
            }
            "top_surface_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.top_surface_line_width = v;
                }
            }
            "inner_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.perimeter_speed = v;
                }
            }
            "outer_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.external_perimeter_speed = v;
                }
            }
            "sparse_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_speed = v;
                }
            }
            "internal_solid_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.solid_infill_speed = v;
                }
            }
            "top_surface_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.top_solid_infill_speed = v;
                }
            }
            "bridge_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.bridge_speed = v;
                }
            }
            "gap_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.gap_fill_speed = v;
                }
            }
            "detect_thin_wall" => {
                self.thin_walls = parse_bool(value);
            }
            "detect_overhang_wall" => {
                self.overhangs = parse_bool(value);
                self.detect_overhang_wall = parse_bool(value);
            }
            "only_one_wall_first_layer" => {
                self.only_one_wall_first_layer = parse_bool(value);
            }
            "top_one_wall_type" => {
                self.top_one_wall_type = TopOneWallType::from_str_bambu(value);
            }
            "slice_closing_radius" => {
                if let Some(v) = parse_f64(value) {
                    self.slice_closing_radius = v;
                }
            }
            "xy_hole_compensation" => {
                if let Some(v) = parse_f64(value) {
                    self.xy_hole_compensation = v;
                    self.xy_size_compensation = v;
                }
            }
            "xy_contour_compensation" => {
                if let Some(v) = parse_f64(value) {
                    self.xy_contour_compensation = v;
                }
            }
            "elefant_foot_compensation" => {
                if let Some(v) = parse_f64(value) {
                    self.elephant_foot_compensation = v;
                }
            }
            "infill_wall_overlap" => {
                if let Some(v) = parse_pct(value) {
                    self.infill_wall_overlap = v;
                }
            }
            "infill_direction" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_angle = v;
                }
            }
            "initial_layer_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_flow_ratio = v;
                }
            }
            "top_solid_infill_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.top_solid_infill_flow_ratio = v;
                }
            }
            "print_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.print_flow_ratio = v;
                }
            }
            "seam_position" => {
                self.seam_position = match value.trim() {
                    "aligned" => SeamPosition::Aligned,
                    "random" => SeamPosition::Random,
                    "back" | "rear" => SeamPosition::Rear,
                    "nearest" => SeamPosition::Nearest,
                    _ => SeamPosition::Aligned,
                };
            }
            "fuzzy_skin" => {
                self.fuzzy_skin_type = FuzzySkinType::from_str_bambu(value);
                self.fuzzy_skin = self.fuzzy_skin_type.is_enabled();
            }
            "fuzzy_skin_thickness" => {
                if let Some(v) = parse_f64(value) {
                    self.fuzzy_skin_thickness = v;
                }
            }
            "fuzzy_skin_point_distance" | "fuzzy_skin_point_dist" => {
                if let Some(v) = parse_f64(value) {
                    self.fuzzy_skin_point_distance = v;
                }
            }
            "wall_generator" => {
                self.perimeter_mode = match value.trim() {
                    "arachne" => PerimeterMode::Arachne,
                    _ => PerimeterMode::Classic,
                };
            }
            "min_bead_width" => {
                if let Some(v) = parse_pct(value) {
                    self.arachne_min_bead_width = v * self.line_width.max(0.4);
                }
            }
            "min_feature_size" => {
                if let Some(v) = parse_pct(value) {
                    self.arachne_min_feature_size = v * self.line_width.max(0.4);
                }
            }
            "wall_transition_length" => {
                if let Some(v) = parse_f64(value) {
                    self.arachne_wall_transition_length = v;
                }
            }
            "wall_transition_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.wall_transition_angle = v;
                }
            }
            "wall_transition_filter_deviation" => {
                if let Some(v) = parse_f64(value) {
                    self.wall_transition_filter_deviation = v;
                }
            }
            "wall_distribution_count" => {
                if let Some(v) = parse_u32(value) {
                    self.wall_distribution_count = v;
                }
            }
            "detect_narrow_internal_solid_infill" => {
                self.detect_narrow_internal_solid_infill = parse_bool(value);
            }
            "minimum_sparse_infill_area" => {
                if let Some(v) = parse_f64(value) {
                    self.minimum_sparse_infill_area = v;
                }
            }
            "enable_support" => {
                self.enable_support = parse_bool(value);
            }
            "support_type" => {
                if value.contains("tree") {
                    self.support_type = SupportType::Tree;
                } else if value.contains("hybrid") {
                    self.support_type = SupportType::Hybrid;
                } else {
                    self.support_type = SupportType::Normal;
                }
            }
            "enforce_support_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.enforce_support_layers = v;
                }
            }
            "support_threshold_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.support_threshold_angle = v;
                }
            }
            "support_on_build_plate_only" | "support_buildplate_only" => {
                self.support_on_build_plate_only = parse_bool(value);
            }
            "support_base_pattern_spacing" => {
                if let Some(v) = parse_f64(value) {
                    self.support_base_pattern_spacing = v;
                }
            }
            "support_interface_top_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.support_interface_top_layers = v;
                }
            }
            "support_interface_bottom_layers" => {
                if let Some(v) = value.trim().parse::<i32>().ok() {
                    self.support_interface_bottom_layers = v;
                }
            }
            "support_interface_spacing" => {
                if let Some(v) = parse_f64(value) {
                    self.support_interface_spacing = v;
                }
            }
            "support_top_z_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.support_top_z_distance = v;
                }
            }
            "support_bottom_z_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.support_bottom_z_distance = v;
                }
            }
            "support_object_xy_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.support_object_xy_distance = v;
                }
            }
            "support_expansion" => {
                if let Some(v) = parse_f64(value) {
                    self.support_expansion = v;
                }
            }
            "support_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.support_angle = v;
                }
            }
            "support_object_first_layer_gap" => {
                if let Some(v) = parse_f64(value) {
                    self.support_object_first_layer_gap = v;
                }
            }
            "enable_support_ironing" => {
                self.enable_support_ironing = parse_bool(value);
            }
            "support_ironing_pattern" => {
                self.support_ironing_pattern = InfillPattern::from_str_bambu(value);
            }
            "support_ironing_flow" => {
                if let Some(v) = parse_pct(value) {
                    self.support_ironing_flow = v;
                }
            }
            "support_ironing_spacing" => {
                if let Some(v) = parse_f64(value) {
                    self.support_ironing_spacing = v;
                }
            }
            "support_ironing_inset" => {
                if let Some(v) = parse_f64(value) {
                    self.support_ironing_inset = v;
                }
            }
            "support_ironing_direction" => {
                if let Some(v) = parse_f64(value) {
                    self.support_ironing_direction = v;
                }
            }
            "support_ironing_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.support_ironing_speed = v;
                }
            }
            "support_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.support_line_width = v;
                }
            }
            "support_base_pattern" => {
                self.support_base_pattern = SupportBasePattern::from_str_bambu(value);
            }
            "support_interface_pattern" => {
                self.support_interface_pattern = SupportInterfacePattern::from_str_bambu(value);
            }
            "bridge_no_support" => {
                self.bridge_no_support = parse_bool(value);
            }
            "independent_support_layer_height" => {
                self.independent_support_layer_height = parse_bool(value);
            }
            "support_remove_small_overhang" | "support_remove_small_overhangs" => {
                self.support_remove_small_overhang = parse_bool(value);
            }
            "top_z_overrides_xy_distance" => {
                self.top_z_overrides_xy_distance = parse_bool(value);
            }
            "support_style" => {
                self.support_style = TreeSupportStyle::from_str_bambu(value);
            }
            "tree_support_branch_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.tree_support_branch_angle = v;
                }
            }
            "tree_support_branch_diameter" => {
                if let Some(v) = parse_f64(value) {
                    self.tree_support_branch_diameter = v;
                }
            }
            "tree_support_branch_diameter_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.tree_support_branch_diameter_angle = v;
                }
            }
            "tree_support_branch_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.tree_support_branch_distance = v;
                }
            }
            "tree_support_wall_count" => {
                if let Some(v) = parse_u32(value) {
                    self.tree_support_wall_count = v;
                }
            }
            "tree_support_with_infill" => {
                self.tree_support_with_infill = parse_bool(value);
            }
            "tree_support_brim_width" => {
                if let Some(v) = parse_f64(value) {
                    self.tree_support_brim_width = v;
                }
            }
            "ironing_type" => {
                self.ironing_type = IroningType::from_str_bambu(value);
            }
            "ironing_flow" => {
                if let Some(v) = parse_pct(value) {
                    self.ironing_flow = v;
                }
            }
            "ironing_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.ironing_speed = v;
                }
            }
            "ironing_spacing" => {
                if let Some(v) = parse_f64(value) {
                    self.ironing_spacing = v;
                }
            }
            "ironing_direction" => {
                if let Some(v) = parse_f64(value) {
                    self.ironing_direction = v;
                }
            }
            "seam_slope_type" => {
                self.scarf_seam_type = ScarfSeamType::from_str_bambu(value);
            }
            "seam_slope_start_height" => {
                if let Some(v) = parse_f64(value) {
                    self.scarf_seam_start_height = v;
                }
            }
            "seam_slope_steps" => {
                if let Some(v) = parse_u32(value) {
                    self.scarf_seam_steps = v;
                }
            }
            "seam_slope_inner_walls" => {
                self.scarf_seam_inner_walls = parse_bool(value);
            }
            "seam_slope_entire_loop" => {
                self.scarf_seam_entire_loop = parse_bool(value);
            }
            "seam_slope_gap" => {
                if let Some(v) = parse_f64(value) {
                    self.scarf_seam_gap = v;
                }
            }
            "seam_slope_min_length" => {
                if let Some(v) = parse_f64(value) {
                    self.scarf_seam_min_length = v;
                }
            }
            "seam_slope_conditional" => {
                self.scarf_seam_conditional = parse_bool(value);
            }
            "scarf_angle_threshold" => {
                if let Some(v) = parse_f64(value) {
                    self.scarf_angle_threshold = v;
                }
            }
            "seam_gap" => {
                if let Some(v) = parse_f64(value) {
                    self.seam_gap = v;
                }
            }
            "brim_type" => {
                self.brim_type = BrimType::from_str_bambu(value);
            }
            "brim_width" => {
                if let Some(v) = parse_f64(value) {
                    self.brim_width = v;
                }
            }
            "brim_object_gap" => {
                if let Some(v) = parse_f64(value) {
                    self.brim_object_gap = v;
                }
            }
            "wall_infill_order" => {
                self.wall_infill_order = WallInfillOrder::from_str_bambu(value);
            }
            "is_infill_first" => {
                self.is_infill_first = parse_bool(value);
            }
            "top_surface_pattern" => {
                self.top_surface_pattern = SurfacePattern::from_str_bambu(value);
            }
            "bottom_surface_pattern" => {
                self.bottom_surface_pattern = SurfacePattern::from_str_bambu(value);
            }
            "internal_solid_infill_pattern" => {
                self.internal_solid_infill_pattern =
                    InternalSolidInfillPattern::from_str_bambu(value);
            }
            "bridge_flow" => {
                if let Some(v) = parse_f64(value) {
                    self.bridge_flow = v;
                }
            }
            "bridge_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.bridge_angle = v;
                }
            }
            "thick_bridges" => {
                self.thick_bridges = parse_bool(value);
            }
            "max_bridge_length" => {
                if let Some(v) = parse_f64(value) {
                    self.max_bridge_length = v;
                }
            }
            "bottom_solid_infill_flow_ratio" => {
                if let Some(v) = parse_f64(value) {
                    self.bottom_solid_infill_flow_ratio = v;
                }
            }
            "adaptive_layer_height" => {
                self.adaptive_layer_height = parse_bool(value);
            }
            "min_layer_height" => {
                if let Some(v) = parse_f64(value) {
                    self.min_layer_height = v;
                }
            }
            "max_layer_height" => {
                if let Some(v) = parse_f64(value) {
                    self.max_layer_height = v;
                }
            }
            "raft_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.raft_layers = v;
                }
            }
            "raft_expansion" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_expansion = v;
                }
            }
            "raft_contact_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_contact_distance = v;
                }
            }
            "raft_first_layer_density" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_first_layer_density = v;
                }
            }
            "raft_first_layer_expansion" => {
                if let Some(v) = parse_f64(value) {
                    self.raft_first_layer_expansion = v;
                }
            }
            "wipe" | "filament_wipe" => {
                self.wipe_enabled = parse_bool(value);
            }
            "wipe_distance" | "filament_wipe_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.wipe_distance = v;
                }
            }
            "retract_before_wipe" | "filament_retract_before_wipe" => {
                if let Some(v) = parse_pct(value) {
                    self.retract_before_wipe = v;
                }
            }
            "enable_overhang_speed" => {
                self.enable_overhang_speed = parse_bool(value);
            }
            "overhang_1_4_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_1_4_speed = v;
                }
            }
            "overhang_2_4_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_2_4_speed = v;
                }
            }
            "overhang_3_4_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_3_4_speed = v;
                }
            }
            "overhang_4_4_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.overhang_4_4_speed = v;
                }
            }
            "infill_combination" => {
                self.infill_combination = parse_bool(value);
            }
            "sparse_infill_anchor" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_anchor = v;
                }
            }
            "sparse_infill_anchor_max" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_anchor_max = v;
                }
            }
            "reduce_infill_retraction" => {
                self.reduce_infill_retraction = parse_bool(value);
            }
            "ensure_vertical_shell_thickness" => {
                self.ensure_vertical_shell_thickness =
                    EnsureVerticalShellThickness::from_str_bambu(value);
            }
            "slicing_mode" => {
                self.slicing_mode = match value.trim() {
                    "even_odd" => SlicingMode::EvenOdd,
                    "close_holes" => SlicingMode::CloseHoles,
                    _ => SlicingMode::Regular,
                };
            }
            "precise_outer_wall" => {
                self.precise_outer_wall = parse_bool(value);
            }
            "embedding_wall_into_infill" => {
                self.embedding_wall_into_infill = parse_bool(value);
            }
            "top_shell_thickness" => {
                if let Some(v) = parse_f64(value) {
                    self.top_shell_thickness = v;
                }
            }
            "bottom_shell_thickness" => {
                if let Some(v) = parse_f64(value) {
                    self.bottom_shell_thickness = v;
                }
            }
            "elefant_foot_min_width" => {
                if let Some(v) = parse_f64(value) {
                    self.elephant_foot_min_width = v;
                }
            }
            "internal_bridge_support_thickness" => {
                if let Some(v) = parse_f64(value) {
                    self.internal_bridge_support_thickness = v;
                }
            }
            "filter_out_gap_fill" => {
                if let Some(v) = parse_f64(value) {
                    self.filter_out_gap_fill = v;
                }
            }
            "skirt_height" => {
                if let Some(v) = parse_u32(value) {
                    self.skirt_height = v;
                }
            }
            "skirt_loops" => {
                if let Some(v) = parse_u32(value) {
                    self.skirt_loops = v;
                }
            }
            "skirt_distance" => {
                if let Some(v) = parse_f64(value) {
                    self.skirt_distance = v;
                }
            }
            "z_hop" => {
                if let Some(v) = parse_f64(value) {
                    self.z_hop = v;
                }
            }
            "small_perimeter_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.small_perimeter_speed = v;
                }
            }
            "small_perimeter_threshold" => {
                if let Some(v) = parse_f64(value) {
                    self.small_perimeter_threshold = v;
                }
            }
            "initial_layer_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_speed = v;
                }
            }
            "initial_layer_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.initial_layer_infill_speed = v;
                }
            }
            "support_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.support_speed = v;
                }
            }
            "support_interface_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.support_interface_speed = v;
                }
            }
            "support_interface_loop_pattern" => {
                self.support_interface_loop_pattern = parse_bool(value);
            }
            "vertical_shell_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.vertical_shell_speed = v;
                }
            }
            "detect_floating_vertical_shell" => {
                self.detect_floating_vertical_shell = parse_bool(value);
            }
            "spiral_mode" => {
                self.spiral_vase = parse_bool(value);
            }
            // === Interlocking (PrintConfig.hpp:1008-1013) ===
            "interlocking_beam" => {
                self.interlocking_beam = parse_bool(value);
            }
            "interlocking_beam_width" => {
                if let Some(v) = parse_f64(value) {
                    self.interlocking_beam_width = v;
                }
            }
            "interlocking_orientation" => {
                if let Some(v) = parse_f64(value) {
                    self.interlocking_orientation = v;
                }
            }
            "interlocking_beam_layer_count" => {
                if let Ok(v) = value.trim().parse::<i32>() {
                    self.interlocking_beam_layer_count = v;
                }
            }
            "interlocking_depth" => {
                if let Ok(v) = value.trim().parse::<i32>() {
                    self.interlocking_depth = v;
                }
            }
            "interlocking_boundary_avoidance" => {
                if let Ok(v) = value.trim().parse::<i32>() {
                    self.interlocking_boundary_avoidance = v;
                }
            }
            _ => return false,
        }
        true
    }
}

// ===========================================================================
// Faithful 1:1 port of the self-contained helpers from BambuStudio's
// PrintConfig.cpp top section. The bulk of PrintConfig.cpp (PrintConfigDef
// and DynamicPrintConfig method implementations) is built on the Config.hpp
// class hierarchy (ConfigOptionDef, ConfigDef::add, coXxx option types,
// ConfigOptionEnum<T>, ConfigBase/DynamicConfig) which has not yet been
// ported; those symbols are listed as blocked. The helpers below depend only
// on standard string/container operations (and the already-ported BBS enums
// in crate::extruder), so they are ported here verbatim.
//
// NOTE: Several other PrintConfig.cpp helpers (get_extruder_index:92,
// get_filament_config_idx:100, get_process_config_idx:109,
// get_extruder_variant_string:528, get_config_index_base:549,
// get_nozzle_volume_type_string:563, enum_names_from_keys_map:119 for the
// NozzleVolumeType map) were already ported into crate::extruder and
// crate::multi_nozzle_utils and are not duplicated here.
// ===========================================================================

use crate::extruder::{NozzleVolumeType, NVT_MAX_NOZZLE_VOLUME_TYPE};
use std::collections::{BTreeMap, BTreeSet};

// PrintConfig.cpp:19  (anonymous namespace)
// std::set<std::string> SplitStringAndRemoveDuplicateElement(const std::string &str, const std::string &separator)
pub fn split_string_and_remove_duplicate_element(str: &str, separator: &str) -> BTreeSet<String> {
    // PrintConfig.cpp:21
    let mut result: BTreeSet<String> = BTreeSet::new();
    // PrintConfig.cpp:22  if (str.empty()) return result;
    if str.is_empty() {
        return result;
    }

    // PrintConfig.cpp:24  std::string strs = str + separator;
    let strs = format!("{}{}", str, separator);
    let strs_bytes = strs.as_bytes();
    // PrintConfig.cpp:26  size_t size = strs.size();
    let size = strs.len();

    // PrintConfig.cpp:28  for (int i = 0; i < size; ++i)
    let mut i: usize = 0;
    while i < size {
        // PrintConfig.cpp:29  pos = strs.find(separator, i);
        // std::string::find returns std::string::npos when not found.
        let pos = find_from(strs_bytes, separator.as_bytes(), i);
        // PrintConfig.cpp:30  if (pos < size)
        if let Some(pos) = pos {
            if pos < size {
                // PrintConfig.cpp:31  std::string sub_str = strs.substr(i, pos - i);
                let sub_str = strs[i..pos].to_string();
                // PrintConfig.cpp:32  result.insert(sub_str);
                result.insert(sub_str);
                // PrintConfig.cpp:33  i = pos + separator.size() - 1;
                // (the for-loop's ++i then advances past the separator)
                i = pos + separator.len() - 1;
            }
        }
        i += 1;
    }

    // PrintConfig.cpp:37
    result
}

// PrintConfig.cpp:40  (anonymous namespace)
// void ReplaceString(std::string &resource_str, const std::string &old_str, const std::string &new_str)
pub fn replace_string(resource_str: &mut String, old_str: &str, new_str: &str) {
    // PrintConfig.cpp:42  std::string::size_type pos = 0;
    let mut pos: usize = 0;
    // PrintConfig.cpp:43  size_t new_size = 0;
    let mut new_size: usize = 0;
    // PrintConfig.cpp:44  while ((pos = resource_str.find(old_str, pos + new_size)) != std::string::npos)
    loop {
        let start = pos + new_size;
        match find_from(resource_str.as_bytes(), old_str.as_bytes(), start) {
            Some(found) => {
                pos = found;
                // PrintConfig.cpp:46  resource_str.replace(pos, old_str.length(), new_str);
                resource_str.replace_range(pos..pos + old_str.len(), new_str);
                // PrintConfig.cpp:47  new_size = new_str.size();
                new_size = new_str.len();
            }
            None => break,
        }
    }
}

// Helper mirroring std::string::find(needle, from): returns the byte index of
// the first occurrence of `needle` in `haystack` at or after `from`, or None
// (std::string::npos). An empty needle matches at `from` (clamped to len),
// matching libstdc++ semantics.
fn find_from(haystack: &[u8], needle: &[u8], from: usize) -> Option<usize> {
    if from > haystack.len() {
        return None;
    }
    if needle.is_empty() {
        return Some(from);
    }
    if needle.len() > haystack.len() {
        return None;
    }
    let last = haystack.len() - needle.len();
    let mut i = from;
    while i <= last {
        if &haystack[i..i + needle.len()] == needle {
            return Some(i);
        }
        i += 1;
    }
    None
}

// PrintConfig.cpp:60
// const std::vector<std::string> filament_extruder_override_keys = { ... };
pub const FILAMENT_EXTRUDER_OVERRIDE_KEYS: [&str; 15] = [
    // floats
    "filament_retraction_length",                  // PrintConfig.cpp:62
    "filament_z_hop",                              // PrintConfig.cpp:63
    "filament_z_hop_types",                        // PrintConfig.cpp:64
    "filament_retract_lift_above",  //not in filament_options_with_variant, not used? // PrintConfig.cpp:65
    "filament_retract_lift_below",  //not in filament_options_with_variant, not used? // PrintConfig.cpp:66
    "filament_retraction_speed",                   // PrintConfig.cpp:67
    "filament_deretraction_speed",                 // PrintConfig.cpp:68
    "filament_retract_restart_extra",  //not in filament_options_with_variant, added on 20250816 // PrintConfig.cpp:69
    "filament_retraction_minimum_travel",          // PrintConfig.cpp:70
    // BBS: floats
    "filament_wipe_distance",                      // PrintConfig.cpp:72
    // bools
    "filament_retract_when_changing_layer",        // PrintConfig.cpp:74
    "filament_wipe",                               // PrintConfig.cpp:75
    // percents
    "filament_retract_before_wipe",                // PrintConfig.cpp:77
    "filament_long_retractions_when_cut",          // PrintConfig.cpp:78
    "filament_retraction_distances_when_cut",      // PrintConfig.cpp:79
];

// PrintConfig.cpp:82
// const std::vector<std::string> filament_overhang_override_keys = { ... };
pub const FILAMENT_OVERHANG_OVERRIDE_KEYS: [&str; 7] = [
    "filament_enable_overhang_speed", // PrintConfig.cpp:83
    "filament_bridge_speed",          // PrintConfig.cpp:84
    "filament_overhang_1_4_speed",    // PrintConfig.cpp:85
    "filament_overhang_2_4_speed",    // PrintConfig.cpp:86
    "filament_overhang_3_4_speed",    // PrintConfig.cpp:87
    "filament_overhang_4_4_speed",    // PrintConfig.cpp:88
    "filament_overhang_totally_speed", // PrintConfig.cpp:89
];

// PrintConfig.cpp:119  static t_config_enum_names enum_names_from_keys_map(const t_config_enum_values &enum_keys_map)
// Inverts an enum key->value map into a value-indexed name vector.
pub fn enum_names_from_keys_map(enum_keys_map: &BTreeMap<String, i32>) -> Vec<String> {
    // PrintConfig.cpp:121  t_config_enum_names names;
    // PrintConfig.cpp:122  int cnt = 0;
    let mut cnt: i32 = 0;
    // PrintConfig.cpp:123  for (const auto& kvp : enum_keys_map) cnt = std::max(cnt, kvp.second);
    for kvp in enum_keys_map.iter() {
        cnt = cnt.max(*kvp.1);
    }
    // PrintConfig.cpp:125  cnt += 1;
    cnt += 1;
    // PrintConfig.cpp:126  names.assign(cnt, "");
    let mut names: Vec<String> = vec![String::new(); cnt as usize];
    // PrintConfig.cpp:127  for (const auto& kvp : enum_keys_map) names[kvp.second] = kvp.first;
    for kvp in enum_keys_map.iter() {
        names[*kvp.1 as usize] = kvp.0.clone();
    }
    // PrintConfig.cpp:129
    names
}

// PrintConfig.cpp:489  static const t_config_enum_values s_keys_map_NozzleVolumeType = { ... };
// Reconstructed as a key->value map for use by convert_to_nvt_type and the
// nozzle-stat parsers below.
fn s_keys_map_nozzle_volume_type() -> BTreeMap<String, i32> {
    let mut m: BTreeMap<String, i32> = BTreeMap::new();
    m.insert("Standard".to_string(), NozzleVolumeType::NvtStandard as i32); // PrintConfig.cpp:490
    m.insert("High Flow".to_string(), NozzleVolumeType::NvtHighFlow as i32); // PrintConfig.cpp:491
    m.insert(
        "TPU High Flow".to_string(),
        NozzleVolumeType::NvtTPUHighFlow as i32,
    ); // PrintConfig.cpp:492
    m.insert("Hybrid".to_string(), NozzleVolumeType::NvtHybrid as i32); // PrintConfig.cpp:493
    m
}

// PrintConfig.cpp:483  static const t_config_enum_values s_keys_map_ExtruderType = { ... };
// The enum *names* (value-indexed) used by convert_to_nvt_type:
//   s_keys_names_ExtruderType = ["Direct Drive", "Bowden"]
const S_KEYS_NAMES_EXTRUDER_TYPE: [&str; 2] = ["Direct Drive", "Bowden"];

// PrintConfig.cpp:609
// std::vector<std::map<int, int>> get_extruder_ams_count(const std::vector<std::string>& strs)
pub fn get_extruder_ams_count(strs: &[String]) -> Vec<BTreeMap<i32, i32>> {
    // PrintConfig.cpp:611
    let mut extruder_ams_counts: Vec<BTreeMap<i32, i32>> = Vec::new();
    // PrintConfig.cpp:612  for (const std::string& str : strs)
    for str in strs.iter() {
        // PrintConfig.cpp:613
        let mut ams_count_info: BTreeMap<i32, i32> = BTreeMap::new();
        // PrintConfig.cpp:614  if (str.empty())
        if str.is_empty() {
            // PrintConfig.cpp:615
            extruder_ams_counts.push(ams_count_info);
            // PrintConfig.cpp:616
            continue;
        }
        // PrintConfig.cpp:618-619  boost::algorithm::split(ams_infos, str, is_any_of("|"));
        let ams_infos: Vec<&str> = str.split('|').collect();
        // PrintConfig.cpp:620  for (const std::string& ams_info : ams_infos)
        for ams_info in ams_infos.iter() {
            // PrintConfig.cpp:621-622  boost::algorithm::split(numbers, ams_info, is_any_of("#"));
            let numbers: Vec<&str> = ams_info.split('#').collect();
            // PrintConfig.cpp:623  assert(numbers.size() == 2);
            debug_assert!(numbers.len() == 2);
            // PrintConfig.cpp:624  ams_count_info.insert(make_pair(stoi(numbers[0]), stoi(numbers[1])));
            let key = stoi(numbers[0]);
            let val = stoi(numbers[1]);
            // std::map::insert keeps the first value for a duplicate key.
            ams_count_info.entry(key).or_insert(val);
        }
        // PrintConfig.cpp:626
        extruder_ams_counts.push(ams_count_info);
    }
    // PrintConfig.cpp:628
    extruder_ams_counts
}

// PrintConfig.cpp:631
// std::vector<std::map<NozzleVolumeType,int>> get_extruder_nozzle_stats(const std::vector<std::string>& strs)
pub fn get_extruder_nozzle_stats(strs: &[String]) -> Vec<BTreeMap<NozzleVolumeType, i32>> {
    // PrintConfig.cpp:633
    let mut extruder_nozzle_counts: Vec<BTreeMap<NozzleVolumeType, i32>> = Vec::new();
    let keys_map = s_keys_map_nozzle_volume_type();
    // PrintConfig.cpp:634  for (const std::string& str : strs)
    for str in strs.iter() {
        // PrintConfig.cpp:635
        let mut nozzle_count_map: BTreeMap<NozzleVolumeType, i32> = BTreeMap::new();
        // PrintConfig.cpp:636  if(str.empty())
        if str.is_empty() {
            // PrintConfig.cpp:637
            extruder_nozzle_counts.push(nozzle_count_map);
            // PrintConfig.cpp:638
            continue;
        }
        // PrintConfig.cpp:640-641  boost::algorithm::split(nozzle_infos, str, is_any_of("|"));
        let nozzle_infos: Vec<&str> = str.split('|').collect();
        // PrintConfig.cpp:642  for (auto& nozzle_info : nozzle_infos)
        for nozzle_info in nozzle_infos.iter() {
            // PrintConfig.cpp:643-644  boost::algorithm::split(attr, nozzle_info, is_any_of("#"));
            let attr: Vec<&str> = nozzle_info.split('#').collect();
            // PrintConfig.cpp:645  NozzleVolumeType volume_type = NozzleVolumeType(s_keys_map_NozzleVolumeType.at(attr[0]));
            let volume_type = NozzleVolumeType::from_i32(keys_map[attr[0]]);
            // PrintConfig.cpp:646  int nozzle_count = std::atoi(attr[1].c_str());
            let nozzle_count = atoi(attr[1]);
            // PrintConfig.cpp:647  nozzle_count_map[volume_type] = nozzle_count;
            nozzle_count_map.insert(volume_type, nozzle_count);
        }
        // PrintConfig.cpp:649
        extruder_nozzle_counts.push(nozzle_count_map);
    }
    // PrintConfig.cpp:651
    extruder_nozzle_counts
}

// PrintConfig.cpp:655
// std::vector<std::string> save_extruder_ams_count_to_string(const std::vector<std::map<int, int>> &extruder_ams_count)
pub fn save_extruder_ams_count_to_string(extruder_ams_count: &[BTreeMap<i32, i32>]) -> Vec<String> {
    // PrintConfig.cpp:657
    let mut extruder_ams_count_str: Vec<String> = Vec::new();
    // PrintConfig.cpp:658  for (size_t i = 0; i < extruder_ams_count.size(); ++i)
    for i in 0..extruder_ams_count.len() {
        // PrintConfig.cpp:659  std::ostringstream oss;
        let mut oss = String::new();
        // PrintConfig.cpp:660  const auto &item = extruder_ams_count[i];
        let item = &extruder_ams_count[i];
        // PrintConfig.cpp:661  for (auto it = item.begin(); it != item.end(); ++it)
        let mut it = item.iter().peekable();
        while let Some((k, v)) = it.next() {
            // PrintConfig.cpp:662  oss << it->first << "#" << it->second;
            oss.push_str(&format!("{}#{}", k, v));
            // PrintConfig.cpp:663  if (std::next(it) != item.end()) oss << "|";
            if it.peek().is_some() {
                oss.push('|');
            }
        }
        // PrintConfig.cpp:667  extruder_ams_count_str.push_back(oss.str());
        extruder_ams_count_str.push(oss);
    }
    // PrintConfig.cpp:669
    extruder_ams_count_str
}

// PrintConfig.cpp:672  NozzleVolumeType convert_to_nvt_type(const std::string &variant_str)
pub fn convert_to_nvt_type(variant_str: &str) -> NozzleVolumeType {
    // PrintConfig.cpp:673  const auto &ext_types = ConfigOptionEnum<ExtruderType>::get_enum_names();
    let ext_types = &S_KEYS_NAMES_EXTRUDER_TYPE;
    let keys_map = s_keys_map_nozzle_volume_type();

    // PrintConfig.cpp:675-678  trim lambda (std::string trim of " \t\r\n").
    let trim = |s: &str| -> String { s.trim_matches([' ', '\t', '\r', '\n']).to_string() };

    // PrintConfig.cpp:680  for (auto ext_type : ext_types)
    for ext_type in ext_types.iter() {
        // PrintConfig.cpp:681  size_t pos = variant_str.find(ext_type);
        // PrintConfig.cpp:682  if (pos == std::string::npos) continue;
        let pos = match find_from(variant_str.as_bytes(), ext_type.as_bytes(), 0) {
            Some(p) => p,
            None => continue,
        };

        // PrintConfig.cpp:685  std::string result = variant_str;
        let mut result = variant_str.to_string();
        // PrintConfig.cpp:686  result.erase(pos, ext_type.size());
        result.replace_range(pos..pos + ext_type.len(), "");
        // PrintConfig.cpp:687  trim(result);
        let result = trim(&result);

        // PrintConfig.cpp:689  auto iter = s_keys_map_NozzleVolumeType.find(result);
        // PrintConfig.cpp:690  if (iter != s_keys_map_NozzleVolumeType.end())
        if let Some(v) = keys_map.get(&result) {
            // PrintConfig.cpp:691  return NozzleVolumeType(iter->second);
            return NozzleVolumeType::from_i32(*v);
        }
    }

    // PrintConfig.cpp:694  return nvtHybrid;
    NozzleVolumeType::NvtHybrid
}

// PrintConfig.cpp:697
// std::vector<std::string> save_extruder_nozzle_stats_to_string(const std::vector<std::map<NozzleVolumeType,int>>& extruder_nozzle_stats)
pub fn save_extruder_nozzle_stats_to_string(
    extruder_nozzle_stats: &[BTreeMap<NozzleVolumeType, i32>],
) -> Vec<String> {
    // PrintConfig.cpp:699
    let mut extruder_nozzle_count_str: Vec<String> = Vec::new();
    // PrintConfig.cpp:700  for (size_t idx = 0; idx < extruder_nozzle_stats.size(); ++idx)
    for idx in 0..extruder_nozzle_stats.len() {
        // PrintConfig.cpp:701  std::ostringstream oss;
        let mut oss = String::new();
        // PrintConfig.cpp:702  const auto& item = extruder_nozzle_stats[idx];
        let item = &extruder_nozzle_stats[idx];
        // PrintConfig.cpp:703  for (auto it = item.begin(); it != item.end(); ++it)
        let mut it = item.iter().peekable();
        while let Some((k, v)) = it.next() {
            // PrintConfig.cpp:704  oss << get_nozzle_volume_type_string(it->first) << "#" << it->second;
            oss.push_str(&format!("{}#{}", nozzle_volume_type_string(*k), v));
            // PrintConfig.cpp:705  if (std::next(it) != item.end()) oss << "|";
            if it.peek().is_some() {
                oss.push('|');
            }
        }
        // PrintConfig.cpp:708  extruder_nozzle_count_str.emplace_back(oss.str());
        extruder_nozzle_count_str.push(oss);
    }
    // PrintConfig.cpp:710
    extruder_nozzle_count_str
}

// PrintConfig.cpp:563  std::string get_nozzle_volume_type_string(NozzleVolumeType nozzle_volume_type)
// Local copy used by save_extruder_nozzle_stats_to_string (the canonical port
// lives in crate::multi_nozzle_utils but is private there).
fn nozzle_volume_type_string(nozzle_volume_type: NozzleVolumeType) -> String {
    // s_keys_names_NozzleVolumeType is the value-indexed inversion of
    // s_keys_map_NozzleVolumeType (PrintConfig.cpp:489):
    //   ["Standard", "High Flow", "Hybrid", "TPU High Flow"]
    const S_KEYS_NAMES_NOZZLE_VOLUME_TYPE: [&str; 4] =
        ["Standard", "High Flow", "Hybrid", "TPU High Flow"];
    // PrintConfig.cpp:565  if (nozzle_volume_type > nvtMaxNozzleVolumeType) return "";
    if (nozzle_volume_type as i32) > NVT_MAX_NOZZLE_VOLUME_TYPE {
        return String::new();
    }
    // PrintConfig.cpp:569  return s_keys_names_NozzleVolumeType[nozzle_volume_type];
    S_KEYS_NAMES_NOZZLE_VOLUME_TYPE[nozzle_volume_type as usize].to_string()
}

// Faithful equivalents of std::stoi / std::atoi over the leading numeric
// prefix of a string (used by the parsers above). std::atoi returns 0 on
// failure; std::stoi throws — but the C++ call sites only ever pass valid
// integer tokens, so we mirror atoi's lenient leading-prefix behavior.
fn stoi(s: &str) -> i32 {
    parse_leading_int(s)
}

fn atoi(s: &str) -> i32 {
    parse_leading_int(s)
}

fn parse_leading_int(s: &str) -> i32 {
    let trimmed = s.trim_start();
    let bytes = trimmed.as_bytes();
    let mut end = 0;
    if end < bytes.len() && (bytes[end] == b'+' || bytes[end] == b'-') {
        end += 1;
    }
    while end < bytes.len() && bytes[end].is_ascii_digit() {
        end += 1;
    }
    trimmed[..end].parse::<i32>().unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_print_config_default() {
        let config = PrintConfig::default();
        assert!((config.layer_height - 0.2).abs() < 1e-6);
        assert!((config.nozzle_diameter - 0.4).abs() < 1e-6);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_print_config_builder() {
        let config = PrintConfig::new()
            .layer_height(0.15)
            .nozzle_diameter(0.6)
            .print_speed(60.0)
            .support(true);

        assert!((config.layer_height - 0.15).abs() < 1e-6);
        assert!((config.nozzle_diameter - 0.6).abs() < 1e-6);
        assert!((config.print_speed - 60.0).abs() < 1e-6);
        assert!(config.support_enabled);
    }

    #[test]
    fn test_print_config_validation() {
        let mut config = PrintConfig::default();
        assert!(config.validate().is_ok());

        config.layer_height = 0.0;
        assert!(config.validate().is_err());

        config.layer_height = 0.2;
        config.nozzle_diameter = -1.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_print_object_config_default() {
        let config = PrintObjectConfig::default();
        assert_eq!(config.perimeters, 2);
        assert!((config.fill_density - 0.15).abs() < 1e-6);
        assert_eq!(config.fill_pattern, InfillPattern::Grid);
    }

    #[test]
    fn test_print_object_config_builder() {
        let config = PrintObjectConfig::new()
            .perimeters(4)
            .fill_density(0.5)
            .fill_pattern(InfillPattern::Gyroid);

        assert_eq!(config.perimeters, 4);
        assert!((config.fill_density - 0.5).abs() < 1e-6);
        assert_eq!(config.fill_pattern, InfillPattern::Gyroid);
    }

    #[test]
    fn test_infill_pattern_default() {
        assert_eq!(InfillPattern::default(), InfillPattern::Grid);
    }

    #[test]
    fn test_support_type_default() {
        assert_eq!(SupportType::default(), SupportType::Normal);
    }

    #[test]
    fn test_gcode_flavor_default() {
        // C++ default for gcode_flavor is gcfMarlinLegacy (PrintConfig.cpp:3397).
        assert_eq!(GCodeFlavor::default(), GCodeFlavor::MarlinLegacy);
    }

    #[test]
    fn test_seam_position_default() {
        assert_eq!(SeamPosition::default(), SeamPosition::Aligned);
    }

    #[test]
    fn test_perimeter_mode_default() {
        assert_eq!(PerimeterMode::default(), PerimeterMode::Classic);
    }

    #[test]
    fn test_perimeter_mode_is_variable_width() {
        assert!(!PerimeterMode::Classic.is_variable_width());
        assert!(PerimeterMode::Arachne.is_variable_width());
    }

    #[test]
    fn test_print_object_config_arachne() {
        let config = PrintObjectConfig::new()
            .perimeter_mode(PerimeterMode::Arachne)
            .arachne_min_bead_width(0.15);

        assert_eq!(config.perimeter_mode, PerimeterMode::Arachne);
        assert!((config.arachne_min_bead_width - 0.15).abs() < 1e-6);
    }

    #[test]
    fn test_print_object_config_arachne_builder() {
        let config = PrintObjectConfig::new().arachne();
        assert_eq!(config.perimeter_mode, PerimeterMode::Arachne);
    }
}
