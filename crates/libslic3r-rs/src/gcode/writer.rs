//! G-code writer.
//!
//! This module provides the GCodeWriter type for building G-code output,
//! mirroring BambuStudio's GCodeWriter class.

/// Format a G-code axis value matching BambuStudio's GCodeFormatter::emit_axis.
/// Strips trailing zeros after decimal, strips leading zero before decimal for
/// values < 1.0, uses minimum digits. E.g.: -0.8 → "-.8", 0.4 → ".4", 120.3 → "120.3"
/// R206: faithful lift gate (cached).
pub fn lift_faithful_gate() -> bool {
    static GATE: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *GATE.get_or_init(|| crate::faithful_gate("ZSMOOTH_FAITHFUL"))
}

pub fn format_gcode_value(v: f64, digits: usize) -> String {
    let pow10 = 10f64.powi(digits as i32);
    let v_int = (v * pow10).round() as i64;
    let is_neg = v_int < 0;
    let abs_int = v_int.unsigned_abs();
    let s = abs_int.to_string();

    // Pad with leading zeros if needed
    let padded = if s.len() < digits {
        format!("{:0>width$}", s, width = digits)
    } else {
        s
    };

    // Insert decimal point
    let (integer_part, decimal_part) = if padded.len() <= digits {
        ("", padded.as_str())
    } else {
        padded.split_at(padded.len() - digits)
    };

    // Strip trailing zeros from decimal part
    let decimal_trimmed = decimal_part.trim_end_matches('0');

    // Build result
    let mut result = String::new();
    if is_neg {
        result.push('-');
    }
    if !integer_part.is_empty() {
        result.push_str(integer_part);
    }
    if !decimal_trimmed.is_empty() {
        result.push('.');
        result.push_str(decimal_trimmed);
    } else if integer_part.is_empty() {
        result.push('0');
    }
    result
}

use crate::circle::ArcDirection;
use crate::gcode::{GCode, GCodeCommand, GCodeStats};
use crate::geometry::PointF;
use crate::print_config::{PrintConfig, ZHopType};
use crate::CoordF;
use std::f64::consts::PI;
use std::fmt;

/// Extruder state tracker (1:1 port of Extruder class)
///
/// C++ reference: Extruder.hpp/cpp
/// Extruder.hpp:12-80
/// Extruder.cpp:1-150
#[derive(Debug, Clone)]
struct Extruder {
    /// Current E position (may be reset to 0 in relative mode)
    /// Extruder.hpp:59
    /// C++: double m_E;
    m_e: CoordF,

    /// Absolute E tachometer (always accumulates)
    /// Extruder.hpp:61
    /// C++: double m_absolute_E;
    m_absolute_e: CoordF,

    /// Current retraction amount
    /// Extruder.hpp:63
    /// C++: double m_retracted;
    m_retracted: CoordF,

    /// Extra priming on unretraction
    /// Extruder.hpp:65
    /// C++: double m_restart_extra;
    m_restart_extra: CoordF,

    /// Filament extrusion per mm³ of material
    /// Extruder.hpp:67
    /// C++: double m_e_per_mm3;
    m_e_per_mm3: CoordF,

    /// Filament diameter (mm)
    filament_diameter: CoordF,

    /// Filament flow ratio (typically 1.0)
    filament_flow_ratio: CoordF,

    /// Use relative E distances
    /// Passed from config
    use_relative_e: bool,
}

impl Extruder {
    /// Create new extruder with config
    /// Extruder.cpp:9-17
    /// C++: Extruder::Extruder(unsigned int id, GCodeConfig *config, bool share_extruder)
    /// C++: {
    /// C++:     reset();
    /// C++:     m_e_per_mm3 = this->filament_flow_ratio();
    /// C++:     m_e_per_mm3 /= this->filament_crossection();
    /// C++: }
    fn new(use_relative_e: bool, filament_diameter: CoordF, filament_flow_ratio: CoordF) -> Self {
        // Calculate filament cross-section
        // Extruder.hpp:45
        // C++: double filament_crossection() const { return this->filament_diameter() * this->filament_diameter() * 0.25 * PI; }
        let filament_crossection = filament_diameter * filament_diameter * 0.25 * PI;

        // Calculate e_per_mm3
        // Extruder.cpp:15-17
        // C++: m_e_per_mm3 = this->filament_flow_ratio();
        // C++: m_e_per_mm3 /= this->filament_crossection();
        let m_e_per_mm3 = filament_flow_ratio / filament_crossection;

        Self {
            m_e: 0.0,
            m_absolute_e: 0.0,
            m_retracted: 0.0,
            m_restart_extra: 0.0,
            m_e_per_mm3,
            filament_diameter,
            filament_flow_ratio,
            use_relative_e,
        }
    }

    /// Reset extruder state
    /// Extruder.hpp:24-32
    /// C++: void reset() {
    /// C++:     m_E             = 0;
    /// C++:     m_retracted     = 0;
    /// C++:     m_restart_extra = 0;
    /// C++:     m_absolute_E    = 0;
    /// C++: }
    fn reset(&mut self) {
        self.m_e = 0.0;
        self.m_absolute_e = 0.0;
        self.m_retracted = 0.0;
        self.m_restart_extra = 0.0;
    }

    /// Extrude by delta dE (CRITICAL: resets m_E to 0 in relative mode)
    /// Extruder.cpp:29-49
    /// C++: double Extruder::extrude(double dE)
    /// C++: {
    /// C++:     // in case of relative E distances we always reset to 0 before any output
    /// C++:     if (m_config->use_relative_e_distances)
    /// C++:         m_E = 0.;
    /// C++:     m_E          += dE;
    /// C++:     m_absolute_E += dE;
    /// C++:     if (dE < 0.)
    /// C++:         m_retracted -= dE;
    /// C++:     return dE;
    /// C++: }
    fn extrude(&mut self, de: CoordF) -> CoordF {
        // Reset m_E to 0 in relative mode (THIS IS THE KEY!)
        // Extruder.cpp:38-39
        if self.use_relative_e {
            self.m_e = 0.0;
        }

        // Accumulate delta into current E and absolute E
        // Extruder.cpp:40-41
        self.m_e += de;
        self.m_absolute_e += de;

        // Track retraction
        // Extruder.cpp:42-43
        if de < 0.0 {
            self.m_retracted -= de;
        }

        de
    }

    /// Get current E position (for G-code output)
    /// Extruder.hpp:39
    /// C++: double E() const { return m_E; }
    fn e(&self) -> CoordF {
        self.m_e
    }

    /// Reset E to 0 (for G92 E0 command)
    /// Extruder.hpp:40
    /// C++: void reset_E() { m_E = 0.; }
    fn reset_e(&mut self) {
        self.m_e = 0.0;
    }

    /// Get e_per_mm3 (filament mm per mm³ of material)
    /// Extruder.hpp:39
    /// C++: double e_per_mm3() const { return m_e_per_mm3; }
    fn e_per_mm3(&self) -> CoordF {
        self.m_e_per_mm3
    }

    /// Calculate e_per_mm for a given mm3_per_mm
    /// Extruder.hpp:38
    /// C++: double e_per_mm(double mm3_per_mm) const { return mm3_per_mm * m_e_per_mm3; }
    fn e_per_mm(&self, mm3_per_mm: CoordF) -> CoordF {
        mm3_per_mm * self.m_e_per_mm3
    }
}

/// A writer for building G-code output.
///
/// GCodeWriter maintains state about the current position, extrusion,
/// and other parameters, and provides methods for generating G-code
/// commands while tracking this state.
pub struct GCodeWriter {
    /// The G-code being built.
    gcode: GCode,

    /// Current X position (mm).
    x: CoordF,

    /// Current Y position (mm).
    y: CoordF,

    /// Current Z position (mm).
    z: CoordF,

    /// Extruder state tracker (handles E position with C++ semantics)
    /// GCodeWriter.hpp:162
    /// C++: std::vector<Extruder> m_filament_extruders;
    extruder: Extruder,

    /// Current feedrate (mm/min). This mirrors C++ `m_current_speed` — the
    /// active EXTRUSION speed set via `set_speed`. Travel moves do NOT change
    /// this (C++ `GCodeWriter::travel_to_xy` never touches `m_current_speed`),
    /// so the extrusion feedrate survives across travel moves and is not
    /// polluted by the travel speed (F60000 = 1000 mm/s).
    feedrate: CoordF,

    /// The last F value actually emitted on a G-line (mm/min). Used only to
    /// decide whether a fresh `F` token needs to be written, so we don't emit
    /// redundant `G1 F<n>` lines. Travel moves update this (so back-to-back
    /// travels at the same speed omit F) WITHOUT touching `feedrate`.
    last_emitted_f: CoordF,

    /// Whether we're in absolute positioning mode.
    absolute_positioning: bool,

    /// Whether we're in absolute extrusion mode.
    absolute_extrusion: bool,

    /// Current extruder index.
    extruder_index: usize,

    /// GCodeWriter.hpp:29 `bool multiple_extruders;` (`multiple_extruders(false)`, :32), set by
    /// `set_extruders` (GCodeWriter.cpp:80-83).
    pub multiple_extruders: bool,
    /// The sorted ids `set_extruders` was given (the ids of `m_filament_extruders`,
    /// GCodeWriter.cpp:71-78), or None while the export has not called it.
    m_extruder_ids: Option<Vec<u32>>,

    /// Whether position is known.
    position_known: bool,
    /// GCodeWriter.hpp:164 `bool m_is_current_pos_clear = false;`
    ///
    /// Distinct from `position_known`: that one records whether we have ever had
    /// a position at all and is never cleared. THIS one is C++'s "the writer
    /// knows where the extruder is right now" flag — set true when a travel
    /// completes and cleared whenever raw/custom gcode is spliced in, because
    /// the splice may move the head. C++'s own note (GCode.cpp:7477): "gcode
    /// writer doesn't know where the extruder is ... Set this flag so that
    /// normal lift will be used the first time after tool change."
    ///
    /// R629 measured the cost of not having it: the spiral-lift guard fell back
    /// to `position_known`, which is always true, so we emitted 36,407 object
    /// `G17` against C++'s 6,062. R630.
    m_is_current_pos_clear: bool,

    /// `Layer::loverhangs` (Layer.hpp:168) for every layer inside C++'s
    /// protect-z window, newest last, as `(print_z, overhangs)`. R631.
    ///
    /// `is_through_overhang` does NOT test only the current layer: GCode.cpp:6979
    /// builds `z_range = [print_z - 0.4, print_z]` and
    /// `layers_sorted_for_object` returns EVERY layer in it, so at a 0.16mm layer
    /// height the current layer plus the two below it all contribute. Testing the
    /// current layer alone (the first R631 attempt) undercounts badly — Benchy
    /// `G17` came back 338 against C++'s 2,029, because 192 of 299 layers have no
    /// overhang of their own yet sit within 0.4mm of one that does.
    layer_overhangs: std::collections::VecDeque<(CoordF, Vec<crate::geometry::ExPolygon>)>,

    /// GCode.cpp:4093-4096 `auto_lift_type`. When set, `retract()` uses this
    /// instead of consulting the travel predicate — C++ passes an explicit
    /// `LiftType` argument to `retract()` at fifteen sites, and only the travel
    /// path lets `needs_retraction` choose. R636.
    m_forced_lift_type: Option<u8>,

    /// C++'s `apply_instantly` argument to `GCode::retract` (`GCode.cpp:7097`).
    /// True routes the lift through `eager_lift` (immediate emission), false
    /// through `lazy_lift` (deferred, merged into the next `travel_to_xyz`).
    /// R637 measured 7,538 of C++'s 8,845 Majora spirals coming through the
    /// eager path. R638.
    m_apply_lift_instantly: bool,

    /// The lift type `needs_retraction` resolved for the travel about to happen
    /// (GCode.cpp:7046/:7089 write C++'s by-reference `lift_type` out-param here).
    /// 0 = NormalLift, 1 = SpiralLift, 2 = SlopeLift. R631.
    m_pending_travel_lift_type: u8,

    /// Current layer index.
    layer_index: usize,

    /// Total number of layers (for per-layer notifications).
    total_layers: usize,

    /// Current layer Z height.
    layer_z: CoordF,

    /// Retraction state.
    retracted: bool,

    /// Retraction length used.
    retraction_length: CoordF,

    /// Z lift during retraction.
    retract_lift: CoordF,

    /// Z before lift.
    z_before_lift: CoordF,

    /// R205 faithful lift state (GCodeWriter.cpp m_lifted / m_to_lift /
    /// m_to_lift_type). Only active under ZSMOOTH_FAITHFUL via lazy_lift/
    /// travel_to_xyz/unlift; inert otherwise.
    pub m_lifted: CoordF,
    pub m_to_lift: CoordF,
    /// 0 = NormalLift, 1 = SpiralLift, 2 = SlopeLift.
    pub m_to_lift_type: u8,
    /// R208: native GCode::m_nominal_z — the current layer's print z; travel
    /// destinations use this (GCode.cpp:6905 dest3d z).
    pub nominal_z: CoordF,
    /// GCode::m_need_change_layer_lift_z (GCode.cpp:6497-6498): set by
    /// change_layer so the layer's first `_extrude` travels even when it is
    /// already at the path's first point (:7611-7618).
    pub m_need_change_layer_lift_z: bool,

    /// Statistics being collected.
    stats: GCodeStats,

    /// Configuration reference.
    config: PrintConfig,

    /// Layer time tracking for cooling speed adjustment
    /// Accumulated extrusion time for the current layer (seconds)
    layer_extrusion_time: f64,
    /// Speed multiplier from cooling (1.0 = no change, >1 = slow down)
    cooling_slowdown: f64,

    /// Wipe state: accumulated extrusion path for wipe-during-retraction.
    /// C++ equivalent: GCode::m_wipe (Wipe class with path field)
    wipe_path: Vec<(CoordF, CoordF)>,
    /// "; Slow Down Start" of the path about to be extruded is already written
    /// (it precedes the role tag when the path opens a new feature).
    pub slow_down_start_written: bool,
    /// Feedrate (mm/min) of the eager lift in progress when it is a tool-change
    /// lift (`_spiral_travel_to_z(.., tool_change)`), else `None`.
    m_lift_feedrate: Option<CoordF>,
    /// WIPEPROBE: who installed the current wipe path (R807 diagnostics).
    wipe_path_src: &'static str, // (x, y) points in mm
    /// `m_config.role_base_wipe_speed` that `Wipe::wipe` reads (GCode.cpp:448);
    /// the definition's default is true (PrintConfig.cpp:6014-6020).
    m_role_base_wipe_speed: bool,
    /// `m_config.wipe_speed.value` (percent of the travel speed, GCode.cpp:449);
    /// the definition's default is 80 % (PrintConfig.cpp:6005-6013).
    m_wipe_speed_percent: CoordF,
    /// Whether wipe is enabled for this filament
    wipe_enabled: bool,
    /// R610: true only while `retract_for_toolchange` is driving `retract()`, so the
    /// TOWER_ENTRY_WHY probes can report the TOWER calls specifically. R609's
    /// decision counter was global (35,000 retracts) and could not distinguish the
    /// 2,718 tower calls from the ordinary object ones.
    tc_retract_probe: bool,
    /// The retraction under way is a tool change's (`GCode::retract(true, ..)`),
    /// which `Wipe::wipe` reads for its speed and distance.
    m_toolchange_wipe: bool,
    /// The retraction under way is the print's last one
    /// (`GCode::retract(false, true, ..)`, GCode.cpp:3566), which `Wipe::wipe`
    /// reads as `is_last` (GCode.cpp:440, 498).
    m_is_last_retraction: bool,
    /// Wipe distance (mm) — how far to move while wiping
    wipe_distance: CoordF,

    /// Last emitted travel acceleration (M204 S value), for deduplication.
    /// 0 means "not set yet" — always emit on first call.
    last_travel_accel: f64,
    /// R234: native m_last_height (GCode.cpp:6619) — per-path LAYER_HEIGHT
    /// tag register; layer changes also set it (GCode.cpp:4065).
    pub last_height_tag: f64,
    /// R227: native m_is_first_layer (GCodeWriter set_first_layer) — travel
    /// accel selection reads m_first_layer_travel_accelerations on layer 0.
    pub is_first_layer: bool,
    /// R227: native m_acceleration — feature/reset accels are STORED here and
    /// only emitted (deduped vs the shared last_travel_accel register) by the
    /// next extrusion move (GCodeWriter.cpp:168-171 set_acceleration stores;
    /// extrude_to_xy:717 prepends set_extrude_acceleration()). 0 = none.
    pending_accel: u32,
    /// R654 — the same deferral, for the collection-level feature feedrate.
    /// C++ has no collection-level `G1 F` at all: `_extrude` emits the Width tag
    /// (GCode.cpp:6607) and only then `set_speed` (:6663). Ours pre-set the F in
    /// `extrude_collection` before descending into the path, so the pair came out
    /// inverted 148,548 times (R653). Holding it here and flushing it right after
    /// the tag reproduces C++'s order without creating or destroying a line.
    pending_speed: Option<(CoordF, String)>,
    /// R656 — mirror of C++ `m_last_processor_extrusion_role == erWipeTower`
    /// (GCode.cpp:6591). `process_layer` sets the register to `erWipeTower` once
    /// per (layer, extruder) that has a wipe tower (GCode.cpp:4718-4720), and the
    /// NEXT path force-emits both the Width tag (:6605) and the Height tag
    /// (:6619) regardless of value — C++'s comment explains why: the wipe-tower
    /// writer can emit Height_Tag lines without updating `m_last_height`.
    /// One-shot: C++'s register is overwritten by the first path's own role.
    force_analyzer_tags: bool,
    /// WIDTH_TAG_FORCE_ONCE: the forced Width tag of the armed `_extrude` was
    /// already written by `width_tag_before_speed` (the same GCode.cpp:7886
    /// statement, placed ahead of the speed), so `extrude_path` must not write
    /// it a second time.
    forced_width_tag_written: bool,
    /// Per-layer context for the faithful needs_retraction (ZSMOOTH_FAITHFUL):
    /// internal-island polygons + wall boundary lines of the current layer.
    pub zsmooth_retract_ctx: Option<RetractCtx>,
    /// `WipeTowerIntegration::m_wipe_tower_bbx` (GCode.hpp:162): the tower's
    /// first-layer extents in its own coordinates (`WipeTower::get_bbx`), set by
    /// the export before the first layer.
    pub wipe_tower_bbx: Option<((f64, f64), (f64, f64))>,
    /// The length the last retraction took back (C++ `Extruder::m_retracted`,
    /// Extruder.cpp `retract`), which `unretract` gives back.
    m_retracted_length: CoordF,
    /// `Extruder::m_share_retracted` (Extruder.cpp:6, 50-60): per retraction slot (the filament's
    /// extruder, Extruder.cpp:36-46), what its filaments left retracted when the slot was last
    /// left at a tool change; read for `new_extruder_retracted_length` (GCode.cpp:982,
    /// GCodeWriter.cpp:957-969). Written by the export's tool changes only.
    pub m_share_retracted: Vec<f64>,
    /// GCode.cpp m_last_width — persistent LINE_WIDTH tag dedup state.
    last_width_tag: f32,

    /// G-code export origin (mm), subtracted from ABSOLUTE XY in write_command
    /// (C++ GCode::m_origin / point_to_gcode). FRAME_PAIR sets this to the slice
    /// center_offset so the centered-slice frame is re-placed to C++'s gcode frame.
    /// Relative I/J are NOT offset by this.
    gcode_origin_x: f64,
    gcode_origin_y: f64,
    /// R812 — GCode::m_farthest_point_timelapse for the layer being written:
    /// the farthest point in G-code millimetres, and the inline timelapse block
    /// to write once an extrusion ends within 0.5 mm of it (GCode.cpp 7946-7973).
    farthest_timelapse: Option<((f64, f64), String)>,
    farthest_timelapse_inserted: bool,

    /// Last emitted extrusion role, for `; FEATURE:` comment deduplication.
    /// C++ `GCode::m_last_extrusion_role` (GCode.hpp:538) — a PERSISTENT member
    /// so consecutive same-role entities across separate extrude calls do not
    /// re-emit the FEATURE marker. None == erNone (always emit on first).
    last_extrusion_role: Option<crate::extrusion_entity::ExtrusionRole>,
}

impl GCodeWriter {
    // Create a new GCodeWriter with default configuration.
    pub fn new() -> Self {
        Self::with_config(PrintConfig::default())
    }

    /// Create a new GCodeWriter with the given configuration.
    pub fn with_config(config: PrintConfig) -> Self {
        // The export's writer is made at its start: the previous export's placeholder parser
        // goes (GCode.cpp:2765 copies a fresh one for every export).
        crate::g_code::reset_export_placeholders();
        /// Create extruder with relative E mode and filament flow ratio from config
        /// GCodeWriter.cpp:34-42
        /// Extruder.cpp:15-17
        /// C++: m_e_per_mm3 = this->filament_flow_ratio();
        /// C++: m_e_per_mm3 /= this->filament_crossection();
        let extruder = Extruder::new(
            config.use_relative_e,
            config.filament_diameter,
            config.filament_flow_ratio,
        );

        Self {
            gcode: GCode::new(),
            x: 0.0,
            y: 0.0,
            z: 0.0,
            extruder,
            feedrate: 0.0,
            last_emitted_f: 0.0,
            absolute_positioning: true,
            absolute_extrusion: !config.use_relative_e,
            extruder_index: 0,
            multiple_extruders: false,
            m_extruder_ids: None,
            position_known: false,
            // GCodeWriter.hpp:164 — defaults to FALSE, so the first lift of the
            // print is a normal lift, not a spiral.
            m_is_current_pos_clear: false,
            layer_overhangs: std::collections::VecDeque::new(),
            // GCode.cpp:6815 `LiftType lift_type = LiftType::SpiralLift;` — the
            // value needs_retraction starts from before it decides.
            m_pending_travel_lift_type: 1,
            m_forced_lift_type: None,
            m_apply_lift_instantly: false,
            layer_index: 0,
            total_layers: 0,
            layer_z: 0.0,
            retracted: false,
            retraction_length: config.retract_length,
            retract_lift: config.retract_lift,
            z_before_lift: 0.0,
            m_lifted: 0.0,
            m_to_lift: 0.0,
            m_to_lift_type: 0,
            nominal_z: 0.0,
            m_need_change_layer_lift_z: false,
            stats: GCodeStats::default(),
            layer_extrusion_time: 0.0,
            cooling_slowdown: 1.0,
            wipe_path: Vec::new(),
            slow_down_start_written: false,
            m_lift_feedrate: None,
            wipe_path_src: "init",
            m_role_base_wipe_speed: true,
            m_wipe_speed_percent: 80.0,
            wipe_enabled: config.retract_before_wipe > 0.0 || true, // Enable wipe when retract_before_wipe > 0
            tc_retract_probe: false,
            m_toolchange_wipe: false,
            m_is_last_retraction: false,
            wipe_distance: 2.0, // Default wipe distance (mm); overridden from settings
            last_travel_accel: 0.0,
            last_height_tag: 0.0,
            is_first_layer: false,
            pending_accel: 0,
            pending_speed: None,
            force_analyzer_tags: false,
            forced_width_tag_written: false,
            zsmooth_retract_ctx: None,
            wipe_tower_bbx: None,
            m_retracted_length: 0.0,
            m_share_retracted: vec![0.0, 0.0],
            last_width_tag: 0.0,
            gcode_origin_x: 0.0,
            gcode_origin_y: 0.0,
            farthest_timelapse: None,
            farthest_timelapse_inserted: false,
            last_extrusion_role: None,
            config,
        }
    }

    /// Set the G-code export origin (mm) subtracted from absolute XY emits
    /// (C++ GCode::set_origin / m_origin). FRAME_PAIR: = slice center_offset.
    /// Arm (or clear) the inline timelapse for the layer about to be written.
    pub fn set_farthest_timelapse(&mut self, state: Option<((f64, f64), String)>) {
        self.farthest_timelapse = state;
        self.farthest_timelapse_inserted = false;
    }

    /// `m_farthest_point_timelapse.inserted_this_layer`.
    pub fn farthest_timelapse_inserted(&self) -> bool {
        self.farthest_timelapse_inserted
    }

    /// GCode.cpp 7946-7973 `check_and_insert_timelapse`, called by `_extrude`
    /// after each extruded segment with the segment's end point (slice frame,
    /// mm). `unscale(endpoint) + m_origin` is compared with the farthest point.
    pub fn check_and_insert_timelapse(&mut self, x: f64, y: f64) {
        if self.farthest_timelapse_inserted {
            return;
        }
        let Some(((fx, fy), _)) = self.farthest_timelapse.as_ref() else {
            return;
        };
        // m_origin = -gcode_origin (see write_command).
        let ex = x - self.gcode_origin_x;
        let ey = y - self.gcode_origin_y;
        if ((ex - fx) * (ex - fx) + (ey - fy) * (ey - fy)).sqrt() >= 0.5 {
            return;
        }
        let block = self.farthest_timelapse.as_ref().map(|s| s.1.clone()).unwrap_or_default();
        if !block.is_empty() {
            self.write_raw_content(&block);
            self.farthest_timelapse_inserted = true;
        }
    }

    /// `GCode::point_to_gcode` (GCode.cpp:9029-9033) on unscaled coordinates:
    /// `unscale(point) + m_origin - extruder_offset`, left to right, with
    /// `m_origin` stored here negated.
    pub fn point_to_gcode_mm(&self, x: f64, y: f64) -> (f64, f64) {
        (
            (x - self.gcode_origin_x) - self.config.extruder_offset_x,
            (y - self.gcode_origin_y) - self.config.extruder_offset_y,
        )
    }

    /// The export origin as stored here, which is `-m_origin` of the C++ GCode.
    pub fn gcode_origin(&self) -> (f64, f64) {
        (self.gcode_origin_x, self.gcode_origin_y)
    }

    pub fn set_gcode_origin(&mut self, x: f64, y: f64) {
        self.gcode_origin_x = x;
        self.gcode_origin_y = y;
    }

    /// Persistent last extrusion role for `; FEATURE:` dedup (C++
    /// `m_last_extrusion_role`). Returns the role last emitted in a FEATURE
    /// comment, or None (erNone) if none yet.
    pub fn last_extrusion_role(&self) -> Option<crate::extrusion_entity::ExtrusionRole> {
        self.last_extrusion_role
    }

    /// Update the persistent last extrusion role.
    pub fn set_last_extrusion_role(
        &mut self,
        role: Option<crate::extrusion_entity::ExtrusionRole>,
    ) {
        self.last_extrusion_role = role;
    }

    /// Get the built G-code.
    pub fn gcode(&self) -> &GCode {
        &self.gcode
    }

    /// Consume the writer and return the built G-code.
    pub fn finish(mut self) -> GCode {
        self.gcode.stats = self.stats;
        self.gcode
    }

    /// Get the current position.
    pub fn position(&self) -> PointF {
        PointF::new(self.x, self.y)
    }

    /// Get the current Z position.
    pub fn z(&self) -> CoordF {
        self.z
    }

    /// Get the current E position.
    pub fn e(&self) -> CoordF {
        self.extruder.e()
    }

    /// Get the current feedrate.
    pub fn feedrate(&self) -> CoordF {
        self.feedrate
    }

    /// Check if position is known.
    pub fn is_position_known(&self) -> bool {
        self.position_known
    }

    /// GCodeWriter.hpp:126 `bool is_current_position_clear() const`.
    ///
    /// MEASURED R630 — opt-in, NOT default-on. Enabling this alone costs Majora
    /// 1,310 matched lines (670,865 -> 669,555) to remove only 656 `G17`. It is
    /// faithful C++, but it is HALF a mechanism: C++ demotes a lift to normal
    /// when the position is unclear AND promotes travels crossing an overhang to
    /// spiral (`is_through_overhang`, GCode.cpp:6972 — not ported). Landing only
    /// the demotion removes the right lifts in the wrong places. Enable together
    /// with `LIFT_TYPE_AUTO_CPP` once R631 ports the overhang predicate, and
    /// re-measure the pair, not either half.
    ///
    /// NOTE `probe_enabled` tests for the key's PRESENCE, so `WRITER_POS_CLEAR_CPP=0`
    /// turns this ON, not off. To A/B, set or unset the variable entirely.
    pub fn is_current_position_clear(&self) -> bool {
        if !crate::opt_in_gate("WRITER_POS_CLEAR_CPP") {
            // Pre-R630 behaviour; measured better than the faithful one today.
            return self.position_known;
        }
        self.m_is_current_pos_clear
    }

    /// GCodeWriter.hpp:125 `void set_current_position_clear(bool clear)`.
    /// Called with `false` after every raw/custom gcode splice — see the five
    /// C++ sites at GCode.cpp:945, :2729, :4538, :4601 and :7480, all of which
    /// follow an inserted template (change_filament, timelapse, wrapping
    /// detection, tool change) or the start of export. R630.
    pub fn set_current_position_clear(&mut self, clear: bool) {
        self.m_is_current_pos_clear = clear;
    }

    /// Where the output stands, for `take_output_since`.
    pub(crate) fn output_mark(&self) -> usize {
        self.gcode.len()
    }

    /// Remove and return what was written since `mark`, so a caller can place
    /// the text of a writer call elsewhere in a block it assembles.
    pub(crate) fn take_output_since(&mut self, mark: usize) -> String {
        self.gcode.split_off(mark)
    }

    /// After a custom G-code block (`append_tcr`, GCode.cpp:1131-1142): the
    /// writer no longer knows where the head is, and the block's last Z, when
    /// it has one, becomes the Z of the position.
    pub(crate) fn after_custom_gcode(&mut self, last_z: Option<CoordF>) {
        self.m_is_current_pos_clear = false;
        // `is_current_position_clear` reads this one unless
        // WRITER_POS_CLEAR_CPP=1; the next travel sets both again.
        self.position_known = false;
        if let Some(z) = last_z {
            self.z = z;
        }
    }

    /// `GCode::travel_to(point, erMixed, ..)` as `append_tcr` uses it for the
    /// travel to the wipe tower (GCode.cpp:1213-1231). The planner of
    /// `reduce_crossing_wall` is disabled for this travel (:1182), so: retract
    /// when `needs_retraction` says so (a writer that is already retracted
    /// only lifts), else forget the wipe path; then `travel_to_xyz` to the
    /// nominal Z. `x`, `y` are in this writer's frame.
    pub(crate) fn travel_to_tower_point(&mut self, x: CoordF, y: CoordF) {
        use crate::extrusion_entity::ExtrusionRole;
        let from = crate::Point::new(crate::scale(self.x), crate::scale(self.y));
        let to = crate::Point::new(crate::scale(x), crate::scale(y));
        let len = ((x - self.x) * (x - self.x) + (y - self.y) * (y - self.y)).sqrt();
        if self.needs_retraction_decision(from, to, ExtrusionRole::Mixed, len) {
            self.retract();
        } else {
            self.reset_wipe_path();
        }
        self.set_travel_acceleration_for(ExtrusionRole::Mixed, len);
        let z = if self.nominal_z > 0.0 { self.nominal_z } else { self.z };
        self.travel_to_xyz(x, y, z);
    }

    /// Check if currently retracted.
    /// The length the writer's last retraction took back while it is retracted, else 0.
    pub fn retracted_length_now(&self) -> f64 {
        if self.retracted {
            self.m_retracted_length
        } else {
            0.0
        }
    }

    pub fn is_retracted(&self) -> bool {
        self.retracted
    }

    /// Decide whether a travel of the given XY length (mm) should retract.
    ///
    /// Faithful port of `GCode::needs_retraction` (GCode.cpp:6964-6970): the
    /// move retracts only when its length reaches the configured
    /// `retraction_minimum_travel` (BambuStudio `filament_retraction_minimum_travel`,
    /// stored here as `retract_before_travel`). The overhang / lift-type branches
    /// of the C++ function only *promote* the lift kind for an already-retracting
    /// move, so the threshold check is the sole gate on whether to retract at all.
    ///
    /// Returns false if the writer is already retracted (the caller would no-op).
    pub fn needs_retraction_for_travel(&self, travel_len_mm: CoordF) -> bool {
        if self.retracted {
            return false;
        }
        travel_len_mm >= self.config.retract_before_travel
    }

    /// Faithful GCode::needs_retraction (GCode.cpp:6964-7095) for the
    /// ZSMOOTH_FAITHFUL path. `from`/`to` are scaled points, `dest_role` the
    /// role about to be extruded. Branch order matches native:
    ///   1. travel < retraction_minimum_travel -> false
    ///   2. leaving an OUTER wall (last role is ext/overhang perimeter) -> true
    ///   3. reduce_infill_retraction (Auto+low-stickiness or Enabled): dest not
    ///      a perimeter && sparse density > 0 && travel inside an internal
    ///      island without crossing walls -> false
    ///   4. else true
    /// Hand the writer the layer's overhang region, as C++'s `m_layer` does.
    /// R631.
    pub fn set_layer_overhangs(&mut self, print_z: CoordF, overhangs: &[crate::geometry::ExPolygon]) {
        // GCode.cpp:6977-6981 — `const float protect_z = 0.4;`
        const PROTECT_Z: CoordF = 0.4;
        self.layer_overhangs.push_back((print_z, overhangs.to_vec()));
        while let Some((z, _)) = self.layer_overhangs.front() {
            if *z < print_z - PROTECT_Z {
                self.layer_overhangs.pop_front();
            } else {
                break;
            }
        }
    }

    /// GCode.cpp:6972-7027 `is_through_overhang`, reduced to this pipeline.
    ///
    /// C++ gathers every object layer whose print_z falls in
    /// `[print_z - 0.4, print_z]` via `layers_sorted_for_object`, walks each
    /// instance whose bounding box meets the travel, and tests the travel against
    /// that layer's `loverhangs`. Here the writer holds ONE layer's overhangs —
    /// the current one — and instances are already baked into print coordinates,
    /// so the object/instance loop and the sort by `loverhangs_bbox.area()` (a
    /// pure ordering optimisation — the result is an OR over all of them)
    /// collapse away. The remaining deviation is the 0.4mm z-window: C++ also
    /// consults the layer BELOW when its print_z is within 0.4mm. R631.
    fn is_through_overhang(&self, travel: &crate::geometry::Polyline) -> bool {
        if self.layer_overhangs.is_empty() {
            return false;
        }
        let travel_bbox = crate::geometry::BoundingBox::from_points(&travel.points);
        // R633: dump the predicate's TERMS. C++ translates the clipped travel into
        // the plate frame at GCode.cpp:7038 before testing; if our travel and our
        // overhangs live in different frames the intersection almost never fires,
        // which is what a 3.4%-vs-14.6% promotion rate looks like.
        if crate::probe_enabled("OVERHANG_PRED_CENSUS") {
            let mut ob = crate::geometry::BoundingBox::new();
            for (_z, ohs) in &self.layer_overhangs {
                for o in ohs {
                    ob.merge(&crate::geometry::get_extents(std::slice::from_ref(o)));
                }
            }
            eprintln!(
                "OHPRED travel=({},{})-({},{}) overhang_bbox=({},{})-({},{}) nlayers={} npolys={} min_travel={:.4} lift={:.4}",
                travel_bbox.min.x, travel_bbox.min.y, travel_bbox.max.x, travel_bbox.max.y,
                ob.min.x, ob.min.y, ob.max.x, ob.max.y,
                self.layer_overhangs.len(),
                self.layer_overhangs.iter().map(|(_, o)| o.len()).sum::<usize>(),
                self.config.retract_before_travel,
                self.retract_lift
            );
        }
        let subject = [travel.clone()];
        for (_z, overhangs) in &self.layer_overhangs {
            for overhang in overhangs {
                // GCode.cpp:7013-7017 — bbox reject before the clip.
                let bbox = crate::geometry::get_extents(std::slice::from_ref(overhang));
                if !bbox.intersects(&travel_bbox) {
                    continue;
                }
                if !crate::clipper_utils::intersection_pl(&subject, std::slice::from_ref(overhang))
                    .is_empty()
                {
                    return true;
                }
            }
        }
        false
    }

    /// GCode.cpp:7028-7042 — the travel is clipped to the distance a slope lift
    /// would need to reach full z_hop (`max_z_hop / tan(slope_threshold)`), and
    /// only that head segment is tested. R631.
    fn clipped_travel_for_lift(&self, from: crate::Point, to: crate::Point) -> crate::geometry::Polyline {
        let slope_threshold = 3.0 * std::f64::consts::PI / 180.0;
        let thresh_mm = self.retract_lift / slope_threshold.tan();
        let mut pl = crate::geometry::Polyline::new();
        pl.points.push(from);
        let dx = (to.x - from.x) as f64;
        let dy = (to.y - from.y) as f64;
        let len = (dx * dx + dy * dy).sqrt();
        let thresh = crate::scale(thresh_mm) as f64;
        if len > thresh && len > 0.0 {
            let r = thresh / len;
            pl.points.push(crate::Point::new(
                from.x + (dx * r) as crate::Coord,
                from.y + (dy * r) as crate::Coord,
            ));
        } else {
            pl.points.push(to);
        }
        pl
    }

    /// GCode.cpp:7044-7095 — resolve the lift type for the travel that is about
    /// to be retracted for, writing C++'s by-reference `lift_type` out-param into
    /// `m_pending_travel_lift_type`. R631.
    fn resolve_travel_lift_type(&mut self, from: crate::Point, to: crate::Point) {
        if self.config.z_hop_type == ZHopType::Auto {
            let clipped = self.clipped_travel_for_lift(from, to);
            let through = self.is_through_overhang(&clipped);
            self.m_pending_travel_lift_type = if through { 1 } else { 2 };
            // R634: log the RESULT. R633 logged only the predicate's inputs, which
            // is why it could eliminate four suspects and still not say whether the
            // shortfall is the predicate returning false or the verdict being lost
            // downstream between resolution and emission.
            if crate::probe_enabled("OVERHANG_PRED_CENSUS") {
                // R635: same fields and units as C++'s OHCPP line, so the two
                // dumps join exactly. Printed in mm — C++ scales at 1e-6 and we
                // scale at 1e5, so raw coords are NOT comparable.
                let a = clipped.points.first().copied().unwrap_or(from);
                let b = clipped.points.last().copied().unwrap_or(to);
                eprintln!(
                    "OHRUST x0={:.4} y0={:.4} x1={:.4} y1={:.4} through={}",
                    crate::unscale(a.x),
                    crate::unscale(a.y),
                    crate::unscale(b.x),
                    crate::unscale(b.y),
                    through as u8
                );
            }
        } else {
            self.m_pending_travel_lift_type = self.to_lift_type();
        }
    }

    pub fn needs_retraction_faithful(
        &mut self,
        from: crate::Point,
        to: crate::Point,
        dest_role: crate::extrusion_entity::ExtrusionRole,
        travel_len_mm: CoordF,
    ) -> bool {
        if std::env::var("RETDBG").is_ok() {
            eprintln!(
                "RETDBG-R z={:.3} role={:?} last={:?} len={:.4} from={},{} to={},{}",
                self.nominal_z, dest_role, self.last_extrusion_role, travel_len_mm,
                from.x, from.y, to.x, to.y
            );
        }
        if self.retracted {
            return false;
        }
        self.needs_retraction_decision(from, to, dest_role, travel_len_mm)
    }

    /// `GCode::needs_retraction` (GCode.cpp:8363-8496) as such, whatever the
    /// retraction state: true when the travel needs a retraction, with the lift
    /// type resolved for it.
    fn needs_retraction_decision(
        &mut self,
        from: crate::Point,
        to: crate::Point,
        dest_role: crate::extrusion_entity::ExtrusionRole,
        travel_len_mm: CoordF,
    ) -> bool {
        use crate::extrusion_entity::ExtrusionRole;
        // GCode.cpp:6966 — retraction_minimum_travel
        if travel_len_mm < self.config.retract_before_travel {
            return false;
        }
        // GCode.cpp:7044 — force retract when leaving an outer/overhang wall
        let last = self.last_extrusion_role;
        if matches!(
            last,
            Some(ExtrusionRole::ExternalPerimeter) | Some(ExtrusionRole::OverhangPerimeter)
        ) {
            // GCode.cpp:7045-7051
            self.resolve_travel_lift_type(from, to);
            return true;
        }
        // GCode.cpp:7068-7086 — reduce_infill_retraction skip
        let dest_is_perimeter = matches!(
            dest_role,
            ExtrusionRole::Perimeter
                | ExtrusionRole::ExternalPerimeter
                | ExtrusionRole::OverhangPerimeter
        );
        if !dest_is_perimeter {
            if let Some(ctx) = &self.zsmooth_retract_ctx {
                if ctx.density_ok
                    && travel_inside_internal_no_crossing(ctx, from, to)
                    && !travel_near_wall_too_long(ctx, from, to, self.config.retract_before_travel)
                {
                    return false;
                }
            }
        }
        // GCode.cpp:7087-7094
        self.resolve_travel_lift_type(from, to);
        true
    }

    /// Get the current layer index.
    pub fn layer_index(&self) -> usize {
        self.layer_index
    }

    /// Get the statistics.
    pub fn stats(&self) -> &GCodeStats {
        &self.stats
    }

    /// Set the current X position (for testing).
    #[cfg(test)]
    pub fn set_x(&mut self, x: CoordF) {
        self.x = x;
        self.position_known = true;
    }

    /// Set the current Y position (for testing).
    #[cfg(test)]
    pub fn set_y(&mut self, y: CoordF) {
        self.y = y;
        self.position_known = true;
    }

    /// Set the current Z position (for testing).
    #[cfg(test)]
    pub fn set_z(&mut self, z: CoordF) {
        self.z = z;
        self.position_known = true;
    }

    // === G-code generation methods ===

    /// Emit M204 S{accel} only when accel differs from the last-emitted value.
    /// Clamps to machine_max_acceleration_extruding (C++ GCodeWriter.cpp:230-235).
    /// Tracks state to avoid duplicate M204 lines.
    pub fn set_travel_acceleration(&mut self, accel: f64) {
        // ZSMOOTH_FAITHFUL: native GCodeWriter::set_travel_acceleration reads
        // the per-extruder `travel_acceleration` CONFIG (10000 on H2D) and
        // routes through the SAME set_acceleration_impl register as the
        // per-feature accel — travel/feature alternation re-emits M204 on
        // every switch (native: 3148x M204 S10000 on benchy). The legacy path
        // keeps the caller-passed value (hardcoded 6000) and its own register.
        let accel = if crate::faithful_gate("ZSMOOTH_FAITHFUL")
            && self.config.travel_acceleration > 0.0
        {
            // GCodeWriter.cpp:210 — first-layer travels use the
            // first-layer travel accelerations (6000 on this profile).
            if self.is_first_layer && self.config.initial_layer_travel_acceleration > 0.0 {
                self.config.initial_layer_travel_acceleration
            } else {
                self.config.travel_acceleration
            }
        } else {
            accel
        };
        if accel <= 0.0 {
            return;
        }
        // Clamp to machine limit (matching C++ set_acceleration_impl)
        let max_accel = self.config.machine_max_acceleration_extruding;
        let effective = if max_accel > 0.0 && accel > max_accel {
            max_accel
        } else {
            accel
        };
        let effective_u = effective as u32;
        if (self.last_travel_accel as u32) != effective_u {
            self.write_raw(&format!("M204 S{}", effective_u));
            self.last_travel_accel = effective;
        }
    }

    /// GCode.cpp:6880-6887 — travel acceleration selection with the SHORT
    /// variant: travels to/within outer walls shorter than
    /// retraction_minimum_travel use travel_short_distance_acceleration (250
    /// on H2D). ZSMOOTH_FAITHFUL path only; routes through the shared M204
    /// register.
    pub fn set_travel_acceleration_for(
        &mut self,
        dest_role: crate::extrusion_entity::ExtrusionRole,
        travel_len_mm: CoordF,
    ) {
        use crate::extrusion_entity::ExtrusionRole;
        let short = self.config.travel_short_distance_acceleration;
        // GCode.cpp:8272-8283: `use_short_travel_accel` is only decided off the first
        // layer (`if (!this->on_first_layer())`); on the first layer the travel takes
        // the first-layer travel acceleration (GCodeWriter.cpp:266-285).
        // GCODE_SHORT_TRAVEL_FIRST_LAYER_PIN (faithful); =0 takes the short one there too.
        let short_allowed = !(self.is_first_layer
            && crate::faithful_gate("GCODE_SHORT_TRAVEL_FIRST_LAYER_PIN"));
        let accel = if short > 0.0
            && short_allowed
            && matches!(
                dest_role,
                ExtrusionRole::ExternalPerimeter | ExtrusionRole::OverhangPerimeter
            )
            && travel_len_mm < self.config.retract_before_travel
        {
            short
        } else if self.is_first_layer && self.config.initial_layer_travel_acceleration > 0.0 {
            // GCodeWriter.cpp:210 — m_is_first_layer selects the first-layer
            // travel accelerations.
            self.config.initial_layer_travel_acceleration
        } else {
            self.config.travel_acceleration
        };
        if accel <= 0.0 {
            return;
        }
        let max_accel = self.config.machine_max_acceleration_extruding;
        let effective = if max_accel > 0.0 && accel > max_accel {
            max_accel
        } else {
            accel
        };
        let effective_u = effective as u32;
        if (self.last_travel_accel as u32) != effective_u {
            self.write_raw(&format!("M204 S{}", effective_u));
            self.last_travel_accel = effective;
        }
    }

    /// m_last_width compare-and-set (GCode.cpp:6605): returns true when the
    /// width DIFFERS from the last emitted tag (and records it).
    pub fn width_tag_changed(&mut self, width: f64) -> bool {
        let w = width as f32;
        if self.last_width_tag != w {
            self.last_width_tag = w;
            true
        } else {
            false
        }
    }

    /// Native set_acceleration_impl for the per-feature accel THROUGH THE SAME
    /// register as travel accel (ZSMOOTH_FAITHFUL path) — M204 emitted on every
    /// travel/feature alternation, deduped only against the last M204 of either
    /// kind (GCodeWriter.cpp:229-240 m_last_acceleration).
    pub fn set_feature_acceleration_shared(&mut self, accel: u32) {
        if accel == 0 {
            return;
        }
        if lift_faithful_gate() {
            // R227: native set_acceleration only STORES m_acceleration; the
            // M204 is prepended to the NEXT extrusion move (extrude_to_xy →
            // set_extrude_acceleration). Emitting eagerly here put M204 before
            // the feature F line and surfaced resets that native never prints.
            self.pending_accel = accel;
            return;
        }
        let max_accel = self.config.machine_max_acceleration_extruding;
        let effective = if max_accel > 0.0 && (accel as f64) > max_accel {
            max_accel as u32
        } else {
            accel
        };
        if (self.last_travel_accel as u32) != effective {
            self.write_raw(&format!("M204 S{}", effective));
            self.last_travel_accel = effective as f64;
        }
    }

    /// R227: GCodeWriter::set_extrude_acceleration (GCodeWriter.cpp:198-201) —
    /// emit the pending accel through the shared register, deduped. Called by
    /// every extrusion move (and the wipe segments) under the gate.
    /// R654 — hold the feature feedrate until the width tag has been written.
    /// See `pending_speed`. Overwrites any unflushed value: the collection-level
    /// pre-set is per-entity, so a second one before a flush means the first
    /// entity emitted no path and its F was never C++-visible either.
    /// The print's `role_base_wipe_speed` and `wipe_speed` (percent), which
    /// `Wipe::wipe` reads from `gcodegen.config()` (GCode.cpp:447-449).
    pub fn set_wipe_speed_config(&mut self, role_base_wipe_speed: bool, wipe_speed_percent: CoordF) {
        self.m_role_base_wipe_speed = role_base_wipe_speed;
        self.m_wipe_speed_percent = wipe_speed_percent;
    }

    /// GCodeWriter.cpp:226-229 `void GCodeWriter::set_acceleration(unsigned int
    /// acceleration)`: only stores `m_acceleration` (here `pending_accel`); the
    /// next `extrude_to_xy` writes it through `set_extrude_acceleration`
    /// (GCodeWriter.cpp:256-259), deduplicated against the last M204.
    pub fn set_acceleration(&mut self, acceleration: u32) {
        self.pending_accel = acceleration;
    }

    /// GCode.cpp:3818-3825 `void GCode::set_extrude_acceleration(bool
    /// is_first_layer)` over this writer: `initial_layer_acceleration` on the
    /// first layer, else `default_acceleration`, each `floor(v + 0.5)`.
    pub fn set_extrude_acceleration_for_layer(&mut self, is_first_layer: bool) {
        let v = if is_first_layer {
            self.config.initial_layer_acceleration
        } else {
            self.config.default_acceleration
        };
        self.set_acceleration((v + 0.5).floor() as u32);
    }

    /// GCodeWriter::reset_last_acceleration (GCodeWriter.cpp:241-244).
    pub fn reset_last_acceleration(&mut self) {
        self.last_travel_accel = 0.0;
    }

    /// R656 — arm the one-shot force-emit for the analyzer Width/Height tags.
    pub fn set_force_analyzer_tags(&mut self) {
        self.force_analyzer_tags = true;
        self.forced_width_tag_written = false;
    }

    /// Whether the force-emit is armed, without clearing it.
    pub fn peek_force_analyzer_tags(&self) -> bool {
        self.force_analyzer_tags
    }

    /// WIDTH_TAG_FORCE_ONCE: note that the armed `_extrude`'s Width tag is out.
    pub fn mark_forced_width_tag_written(&mut self) {
        self.forced_width_tag_written = true;
    }

    /// WIDTH_TAG_FORCE_ONCE: read and clear the note.
    pub fn take_forced_width_tag_written(&mut self) -> bool {
        std::mem::replace(&mut self.forced_width_tag_written, false)
    }

    /// R656 — read and clear it. Both tag guards in one `_extrude` must see the
    /// SAME value, so the caller takes it once at the top of the path.
    pub fn take_force_analyzer_tags(&mut self) -> bool {
        std::mem::replace(&mut self.force_analyzer_tags, false)
    }

    pub fn set_speed_pending(&mut self, speed: CoordF, comment: &str) {
        self.pending_speed = Some((speed, comment.to_string()));
    }

    /// Emit a pending feature feedrate, if any. Called immediately after the
    /// `; LINE_WIDTH:` tag in `extrude_path_with_arc_fitting`, which is where
    /// C++'s `set_speed` sits relative to the Width tag.
    pub fn flush_pending_speed(&mut self) {
        if let Some((speed, comment)) = self.pending_speed.take() {
            self.set_speed(speed, &comment);
        }
    }

    pub fn flush_pending_accel(&mut self) {
        if !lift_faithful_gate() || self.pending_accel == 0 {
            return;
        }
        let max_accel = self.config.machine_max_acceleration_extruding;
        let effective = if max_accel > 0.0 && (self.pending_accel as f64) > max_accel {
            max_accel as u32
        } else {
            self.pending_accel
        };
        if (self.last_travel_accel as u32) != effective {
            self.write_raw(&format!("M204 S{}", effective));
            self.last_travel_accel = effective as f64;
        }
    }

    /// Per-feature printing acceleration (M204 S value), faithful port of the
    /// "adjust acceleration" block in GCode::_extrude (GCode.cpp:6393-6420).
    ///
    /// The whole block is gated on `default_acceleration > 0`. Branch order:
    ///   1. first layer (initial_layer_acceleration > 0)
    ///   2. ExternalPerimeter / OverhangPerimeter (outer_wall_acceleration > 0)
    ///   3. top surface == TopSolidInfill (top_surface_acceleration > 0)
    ///   4. Perimeter (inner_wall_acceleration > 0)
    ///   5. InternalInfill (sparse_infill_acceleration resolved > 0)
    ///   6. else default_acceleration
    /// A feature value of 0 means "use default" (its branch is skipped), e.g.
    /// `inner_wall_acceleration = 0` falls through to default_acceleration. The
    /// bridge branch is `#if 0` in C++ (disabled), so bridges fall to default too.
    /// `sparse_infill_acceleration` is a percentage of default_acceleration
    /// (get_abs_value): the config stores e.g. "100%" as 100.0, so resolve as
    /// pct/100 * default. Returns None when default_acceleration <= 0 (no M204).
    /// R227: native resets accel to default_acceleration at the end of
    /// extrude_loop / extrude_multi_path / extrude_path when not on the first
    /// layer (GCode.cpp:5591/5676/5720, `(unsigned)(default_acceleration+0.5)`)
    /// through the shared M204 register — the next feature/travel M204 then
    /// re-emits. Gated (ZSMOOTH_FAITHFUL via lift_faithful_gate).
    pub fn reset_acceleration_default(&mut self, is_first_layer: bool) {
        if !lift_faithful_gate() || is_first_layer {
            return;
        }
        if self.config.default_acceleration <= 0.0 {
            return;
        }
        let acc = (self.config.default_acceleration + 0.5).floor() as u32;
        self.pending_accel = acc;
    }

    pub fn feature_acceleration(
        &self,
        role: crate::extrusion_entity::ExtrusionRole,
        is_first_layer: bool,
    ) -> Option<u32> {
        use crate::extrusion_entity::{is_top_surface, ExtrusionRole};
        let c = &self.config;
        if c.default_acceleration <= 0.0 {
            return None;
        }
        let acc: f64 = if is_first_layer && c.initial_layer_acceleration > 0.0 {
            c.initial_layer_acceleration
        } else if c.outer_wall_acceleration > 0.0
            && matches!(
                role,
                ExtrusionRole::ExternalPerimeter | ExtrusionRole::OverhangPerimeter
            )
        {
            c.outer_wall_acceleration
        } else if c.top_surface_acceleration > 0.0 && is_top_surface(role) {
            c.top_surface_acceleration
        } else if c.inner_wall_acceleration > 0.0 && role == ExtrusionRole::Perimeter {
            c.inner_wall_acceleration
        } else if role == ExtrusionRole::InternalInfill
            && (c.sparse_infill_acceleration / 100.0 * c.default_acceleration) > 0.0
        {
            c.sparse_infill_acceleration / 100.0 * c.default_acceleration
        } else {
            c.default_acceleration
        };
        Some((acc + 0.5).floor() as u32)
    }

    /// Perform a linear G1 Z-hop and update the writer's Z-state.
    ///
    /// Unlike `write_raw("G1 Z...")`, this method updates `self.z` and
    /// `self.z_before_lift` so that `unretract()` can correctly descend
    /// back to `layer_z` after the travel.
    ///
    /// `layer_z` is the current layer's print Z (before the hop). This is
    /// the position `unretract()` will descend to.
    ///
    /// Call after `retract_no_lift()` to complete the retract+hop sequence
    /// at layer changes.
    pub fn z_hop_linear(&mut self, layer_z: CoordF, hop_z: CoordF, feedrate: CoordF) {
        self.z = layer_z; // set writer z to the layer position before hopping
        self.z_before_lift = layer_z; // unretract will descend back to here
        let f_opt = if (feedrate - self.last_emitted_f).abs() > 0.01 {
            Some(feedrate)
        } else {
            None
        };
        self.write_command(&GCodeCommand::LinearMove {
            x: None,
            y: None,
            z: Some(hop_z),
            e: None,
            f: f_opt,
        });
        self.z = hop_z;
        // Z-hop is a travel-class move: track the emitted F but leave the
        // extrusion speed (`feedrate`) intact.
        self.last_emitted_f = feedrate;
    }

    /// R221: native m_wipe.path = the loop's SOURCE polyline points
    /// (GCode.cpp:5600-5610), not the emitted (arc-fitted) moves. The exporter
    /// installs it at loop end.
    pub fn set_wipe_path_points(&mut self, pts: Vec<(CoordF, CoordF)>) {
        self.set_wipe_path_points_from(pts, "set");
    }

    pub fn set_wipe_path_points_from(&mut self, pts: Vec<(CoordF, CoordF)>, src: &'static str) {
        self.wipe_path = pts;
        self.wipe_path_src = src;
    }

    /// R220: config accessor for exporter-side seam-gap math.
    pub fn config_ref(&self) -> &PrintConfig {
        &self.config
    }

    /// R219: native replaces m_wipe.path per _extrude (GCode.cpp:5600-5610);
    /// the accumulated path must span only the CURRENT loop/path entity.
    pub fn reset_wipe_path(&mut self) {
        self.wipe_path.clear();
    }

    /// Write a raw G-code line.
    pub fn write_raw(&mut self, line: &str) {
        let __w_t = if crate::probe_enabled("EXPPROF") { Some(std::time::Instant::now()) } else { None };
        if let Some(t) = __w_t {
            use std::sync::atomic::Ordering::Relaxed;
            crate::gcode::exporter::EXPPROF_WRITE_CALLS.fetch_add(1, Relaxed);
            let _ = t;
        }
        self.gcode.append_line(line);
    }

    /// Append raw multi-line content (already newline-terminated).
    pub fn write_raw_content(&mut self, content: &str) {
        self.gcode.append(content);
    }

    /// Write a comment.
    pub fn write_comment(&mut self, comment: &str) {
        self.gcode.append_comment(comment);
    }

    /// Write a G-code command.
    pub fn write_command(&mut self, cmd: &GCodeCommand) {
        // C++ reference: GCode.cpp:7089 point_to_gcode() subtracts (m_origin +
        // extruder_offset) from ABSOLUTE XY. m_origin = the gcode-export origin
        // (FRAME_PAIR: = center_offset, the second half of the centered-slice +
        // export-origin pair). Relative I/J are translation-invariant and are NOT
        // offset (handled below — i/j passed through unchanged).
        let ox = self.config.extruder_offset_x + self.gcode_origin_x;
        let oy = self.config.extruder_offset_y + self.gcode_origin_y;
        // POINT_TO_GCODE_ORDER (on): `unscale(point) + m_origin - extruder_offset`
        // (GCode.cpp:9029-9033), evaluated left to right; with m_origin =
        // -gcode_origin that is `(v - gcode_origin) - extruder_offset`. =0
        // subtracts their sum, which rounds differently.
        let cpp_order = crate::faithful_gate("POINT_TO_GCODE_ORDER");
        let (gx, gy, ex, ey) = (
            self.gcode_origin_x,
            self.gcode_origin_y,
            self.config.extruder_offset_x,
            self.config.extruder_offset_y,
        );
        let tx = |v: f64| if cpp_order { (v - gx) - ex } else { v - ox };
        let ty = |v: f64| if cpp_order { (v - gy) - ey } else { v - oy };
        if (ox != 0.0 || oy != 0.0)
            && matches!(
                cmd,
                GCodeCommand::LinearMove { .. }
                    | GCodeCommand::RapidMove { .. }
                    | GCodeCommand::ArcCW { .. }
                    | GCodeCommand::ArcCCW { .. }
            )
        {
            let adjusted = match cmd {
                GCodeCommand::LinearMove { x, y, z, e, f } => GCodeCommand::LinearMove {
                    x: x.map(tx),
                    y: y.map(ty),
                    z: *z,
                    e: *e,
                    f: *f,
                },
                GCodeCommand::RapidMove { x, y, z, f } => GCodeCommand::RapidMove {
                    x: x.map(tx),
                    y: y.map(ty),
                    z: *z,
                    f: *f,
                },
                GCodeCommand::ArcCW { x, y, i, j, e, f } => GCodeCommand::ArcCW {
                    x: tx(*x),
                    y: ty(*y),
                    i: *i,
                    j: *j,
                    e: *e,
                    f: *f,
                },
                GCodeCommand::ArcCCW { x, y, i, j, e, f } => GCodeCommand::ArcCCW {
                    x: tx(*x),
                    y: ty(*y),
                    i: *i,
                    j: *j,
                    e: *e,
                    f: *f,
                },
                _ => unreachable!(),
            };
            self.gcode.append_line(&adjusted.to_gcode());
        } else {
            self.gcode.append_line(&cmd.to_gcode());
        }
    }

    /// Write a G-code command with a comment.
    pub fn write_command_with_comment(&mut self, gcode: &str, comment: Option<&str>) {
        if let Some(c) = comment {
            self.gcode.append_line(&format!("{} ; {}", gcode, c));
        } else {
            self.gcode.append_line(gcode);
        }
    }

    /// Write the preamble (start G-code).
    pub fn write_preamble(&mut self) {
        self.write_comment("Generated by Slicer");
        self.write_comment("");

        // Set absolute positioning
        self.write_command(&GCodeCommand::AbsolutePositioning);
        self.absolute_positioning = true;

        // Set extrusion mode based on config (M82 = absolute, M83 = relative)
        if self.config.use_relative_e {
            self.write_command(&GCodeCommand::RelativeExtrusion);
            self.absolute_extrusion = false;
        } else {
            self.write_command(&GCodeCommand::AbsoluteExtrusion);
            self.absolute_extrusion = true;
        }

        // Reset extruder position (G92 E0)
        // GCodeWriter.cpp:62-69
        // C++: gcode << this->reset_e(true);
        self.write_command(&GCodeCommand::SetPosition {
            x: None,
            y: None,
            z: None,
            e: Some(0.0),
        });
        self.extruder.reset_e();
    }

    /// Write the end G-code.
    pub fn write_end(&mut self) {
        self.write_comment("End G-code");

        // Turn off heaters
        self.write_command(&GCodeCommand::SetExtruderTemp { s: 0 });
        self.write_command(&GCodeCommand::SetBedTemp { s: 0 });

        // Turn off fan
        self.write_command(&GCodeCommand::FanOff);

        // Home X and Y
        self.write_command(&GCodeCommand::Home {
            x: true,
            y: true,
            z: false,
        });
    }

    /// Set the bed temperature.
    pub fn set_bed_temperature(&mut self, temp: u32, wait: bool) {
        if wait {
            self.write_command(&GCodeCommand::SetBedTempWait { s: temp });
        } else {
            self.write_command(&GCodeCommand::SetBedTemp { s: temp });
        }
    }

    /// Set the extruder temperature.
    pub fn set_extruder_temperature(&mut self, temp: u32, wait: bool) {
        if wait {
            self.write_command(&GCodeCommand::SetExtruderTempWait { s: temp });
        } else {
            self.write_command(&GCodeCommand::SetExtruderTemp { s: temp });
        }
    }

    /// Set the fan speed (0-255).
    pub fn set_fan_speed(&mut self, speed: u32) {
        if speed > 0 {
            self.write_command(&GCodeCommand::SetFanSpeed { s: speed });
        } else {
            self.write_command(&GCodeCommand::FanOff);
        }
    }

    /// Home the printer.
    pub fn home(&mut self, x: bool, y: bool, z: bool) {
        self.write_command(&GCodeCommand::Home { x, y, z });
        if x {
            self.x = 0.0;
        }
        if y {
            self.y = 0.0;
        }
        if z {
            self.z = 0.0;
        }
        self.position_known = true;
    }

    /// Travel move (no extrusion).
    pub fn travel_to(&mut self, x: CoordF, y: CoordF, feedrate: Option<CoordF>) {
        let f = feedrate.unwrap_or(self.config.travel_speed * 60.0);

        // Track travel distance and time
        let dx = x - self.x;
        let dy = y - self.y;
        let dist = (dx * dx + dy * dy).sqrt();
        self.stats.travel_distance_mm += dist;
        if f > 0.0 {
            let move_time = dist * 60.0 / f;
            self.stats.print_time_seconds += move_time;
            self.layer_extrusion_time += move_time;
        }

        // BambuStudio uses G1 (not G0) for XY travel when at Z-hop height
        // with true spiral z-hop type. The reference pattern is:
        //   retract → G17 → G3 Z{hop} (spiral lift) → G1 X Y Z{hop} → G1 Z{layer} → unretract
        // For Normal/Auto linear hops (G1 Z), the XY travel does NOT include Z:
        //   retract → G1 Z{hop} → G1 X Y (no Z) → [unretract: G1 Z{layer}] → E restore
        // Reference: GCodeWriter.cpp — travel after _spiral_travel_to_z() uses G1 with Z.
        //
        // R632: this branch was UNREACHABLE until R632. `config.z_hop_type` was
        // never read from the config and sat at its `Auto` default, so the
        // `matches!(.., Spiral)` test was always false. Resolving the filament
        // override made Benchy `Spiral` and woke it up — at a cost of 82 matched
        // lines. It is also not a port of anything: C++ has no z_hop_type gate
        // here; its XY-with-Z travel comes out of `travel_to_xyz`'s combined move
        // (GCodeWriter.cpp:565-580), which we already emit. Left opt-in rather
        // than deleted, so the shape stays visible next to the C++ it approximates.
        let use_g1_travel_with_z = crate::probe_enabled("TRAVEL_G1_WITH_Z")
            && self.retracted
            && self.retract_lift > 0.0
            && matches!(self.config.z_hop_type, ZHopType::Spiral);

        if use_g1_travel_with_z {
            // Emit G1 with the current (lifted) Z to match BambuStudio exactly.
            // The reference emits: G1 X{x} Y{y} Z{hop_z}
            self.write_command(&GCodeCommand::LinearMove {
                x: Some(x),
                y: Some(y),
                z: Some(self.z),
                e: None,
                f: if (f - self.last_emitted_f).abs() > 0.01 {
                    Some(f)
                } else {
                    None
                },
            });
        } else if self.retracted && self.retract_lift > 0.0 {
            // Normal/Auto linear hop: travel without Z (unretract handles descent).
            // BambuStudio emits G1 (not G0) for travel when at z-hop height.
            self.write_command(&GCodeCommand::LinearMove {
                x: Some(x),
                y: Some(y),
                z: None,
                e: None,
                f: if (f - self.last_emitted_f).abs() > 0.01 {
                    Some(f)
                } else {
                    None
                },
            });
        } else {
            // Faithful port of GCodeWriter::travel_to_xy (GCodeWriter.cpp:405-420):
            // BambuStudio constructs a GCodeG1Formatter (emits "G1"), then
            // emit_xy(point_on_plate) + emit_f(travel_speed). There is no
            // GCodeG0Formatter in BambuStudio — XY travel is always G1, never G0.
            //   GCodeG1Formatter w;
            //   w.emit_xy(point_on_plate);
            //   w.emit_f(this->config.travel_speed... * 60.0);
            self.write_command(&GCodeCommand::LinearMove {
                x: Some(x),
                y: Some(y),
                z: None,
                e: None,
                f: if (f - self.last_emitted_f).abs() > 0.01 {
                    Some(f)
                } else {
                    None
                },
            });
        }

        self.x = x;
        self.y = y;
        // C++ travel_to_xy does NOT update m_current_speed; it only emits F via
        // the formatter. So we update the emitted-F tracker but leave `feedrate`
        // (the extrusion speed) untouched, so it survives across travels.
        self.last_emitted_f = f;
        self.position_known = true;
        // GCodeWriter.cpp:410 — travel_to_xy sets the position-clear flag. R630.
        self.m_is_current_pos_clear = true;
    }

    /// GCode.cpp:8348-8361 `GCode::to_lift_type`. zhtAuto has NO entry in the
    /// switch, so it falls to `default: return LiftType::NormalLift` — the Auto
    /// case is resolved by the CALLER, differently per call site.
    fn to_lift_type(&self) -> u8 {
        crate::g_code::lift_type_code(crate::g_code::to_lift_type(self.config.z_hop_type))
    }

    /// The lift type for a RETRACTION BEFORE A TRAVEL — GCode.cpp:7046 and
    /// :7089, both inside `needs_retraction`:
    ///
    /// ```text
    /// if (ZHopType(FILAMENT_CONFIG(z_hop_types)) == ZHopType::zhtAuto)
    ///     lift_type = is_through_overhang(clipped_travel) ? SpiralLift : SlopeLift;
    /// else
    ///     lift_type = to_lift_type(ZHopType(FILAMENT_CONFIG(z_hop_types)));
    /// ```
    ///
    /// R630: we previously hardcoded SpiralLift here, from an R206 reading taken
    /// on Benchy — whose profile is "Spiral Lift", so the constant happened to be
    /// right. Majora's profile is "Auto Lift", where C++ takes the SLOPE branch
    /// for every travel that does not cross an overhang. That single constant was
    /// the whole of the 35,751-vs-6,062 object `G17` excess R629 measured; the
    /// position-clear flag ported earlier this round only accounted for 656 of it.
    ///
    /// `is_through_overhang` (GCode.cpp:6972-7027) is not ported yet, so under
    /// Auto we take the Slope branch unconditionally; that undershoots C++ by the
    /// ~5,882 travels that DO cross an overhang. R631.
    ///
    /// MEASURED R630, opt-in until `is_through_overhang` lands: taking the Slope
    /// branch unconditionally under Auto REGRESSES both fixtures, because both
    /// profiles are "Auto Lift" and C++ resolves a large share of their travels
    /// to Spiral via the overhang test. Majora object `G17` 35,751 -> 0 against
    /// C++'s 6,062 (matched 669,555 -> 665,793); Benchy 2,040 -> 299 against
    /// C++'s 2,029 (matched 115,901 -> 112,680). The constant was wrong, but so
    /// is the naive branch — the overhang predicate IS the decision. R631.
    fn travel_lift_type(&self) -> u8 {
        if self.config.z_hop_type == ZHopType::Auto {
            if crate::faithful_gate("LIFT_TYPE_AUTO_CPP") {
                // R631: resolved per travel by needs_retraction_faithful, which
                // runs is_through_overhang exactly where C++ does.
                self.m_pending_travel_lift_type
            } else {
                1 // pre-R630 constant: SpiralLift.
            }
        } else {
            self.to_lift_type()
        }
    }

    /// GCode.cpp:4092-4096 — the lift type used by every retract site that is NOT
    /// a travel:
    ///
    /// ```cpp
    /// LiftType auto_lift_type = LiftType::NormalLift;
    /// if (z_hope_type == zhtAuto || zhtSpiral || zhtSlope)
    ///     auto_lift_type = LiftType::SpiralLift;
    /// ```
    ///
    /// R635 measured why this matters: C++'s overhang predicate returns true only
    /// 1,478 times on Majora, yet C++ emits 6,062 object `G17`. The other ~4,584
    /// come from the fifteen sites that pass `auto_lift_type` — layer change,
    /// timelapse, object change, wipe-tower entry — each an UNCONDITIONAL spiral
    /// with no overhang test at all. R636.
    pub fn auto_lift_type(&self) -> u8 {
        match self.config.z_hop_type {
            ZHopType::Auto | ZHopType::Spiral | ZHopType::Slope => 1, // SpiralLift
            ZHopType::Normal => 0,                                   // NormalLift
        }
    }

    /// GCodeWriter.cpp:456-495 `eager_lift` — immediate emission, dispatching on
    /// the lift type. The `to_lift < EPSILON` early-out at `:459` is why C++'s
    /// 7,542 forced retracts yield 7,538 spirals and not more. R638.
    fn eager_lift(&mut self, lift_type: u8) {
        let to_lift = self.retract_lift - self.m_lifted;
        if to_lift < 1e-4 {
            // GCodeWriter.cpp:459-460 — nothing to lift; C++ returns an empty move.
            return;
        }
        match lift_type {
            1 => self.eager_spiral_lift(),
            _ => {
                // GCodeWriter.cpp:475 — "if position is unknown use normal lift".
                // EAGER_LIFT_TRAVEL_ACCEL (on): `_travel_to_z` returns
                // `set_travel_acceleration() + w.string()`.
                if crate::faithful_gate("EAGER_LIFT_TRAVEL_ACCEL") {
                    self.set_travel_acceleration(0.0);
                }
                self.travel_to_z(self.z + to_lift, None);
                self.m_lifted = self.retract_lift;
                self.m_to_lift = 0.0;
            }
        }
    }

    /// `retract_for_toolchange()` with C++'s explicit `LiftType` and
    /// `apply_instantly` arguments — `GCode.cpp:747`
    /// `retract(tcr.is_tool_change && !is_nozzle_change, false, auto_lift_type, true)`.
    /// R637 measured this site at 3,443 calls on Majora. R638.
    pub fn retract_for_toolchange_with_lift(&mut self, lift_type: u8, apply_instantly: bool) {
        let prev_lt = self.m_forced_lift_type.replace(lift_type);
        let prev_ai = std::mem::replace(&mut self.m_apply_lift_instantly, apply_instantly);
        self.retract_for_toolchange();
        self.m_forced_lift_type = prev_lt;
        self.m_apply_lift_instantly = prev_ai;
    }

    /// `GCode::retract(false, false, lift_type, true, true)` as `append_tcr` calls
    /// it when leaving the wipe tower (GCode.cpp:1372): `slow_lift` reaches
    /// `eager_lift(type, tool_change)` (GCode.cpp:8522), and
    /// `_spiral_travel_to_z` then lifts at `prime_tower_lift_speed` when that is
    /// positive (GCodeWriter.cpp:790-792).
    pub fn retract_leaving_tower(&mut self, lift_type: u8, prime_tower_lift_speed: CoordF) {
        let speed = if prime_tower_lift_speed > 0.0 {
            Some(prime_tower_lift_speed * 60.0)
        } else {
            None
        };
        let prev = std::mem::replace(&mut self.m_lift_feedrate, speed);
        self.retract_with_lift_type(lift_type, true);
        self.m_lift_feedrate = prev;
    }

    /// `retract()` with C++'s explicit `LiftType` and `apply_instantly` arguments
    /// (`GCode::retract(bool, bool, LiftType, bool)`). R636/R638.
    pub fn retract_with_lift_type(&mut self, lift_type: u8, apply_instantly: bool) {
        let prev_lt = self.m_forced_lift_type.replace(lift_type);
        let prev_ai = std::mem::replace(&mut self.m_apply_lift_instantly, apply_instantly);
        self.retract();
        self.m_forced_lift_type = prev_lt;
        self.m_apply_lift_instantly = prev_ai;
    }

    /// `GCode::retract(false, is_last_retraction, lift_type, apply_instantly)`
    /// (GCode.cpp:8498-8530) with `is_last_retraction` handed to `Wipe::wipe`
    /// (GCode.cpp:8508): the last wipe of the print writes its speed line without
    /// the `;_WIPE` cooling marker (GCode.cpp:497-498), "because no more cooling
    /// layer will clean this". GCode.cpp:3566 is the one caller with `true`.
    pub fn retract_with_lift_type_is_last(
        &mut self,
        lift_type: u8,
        apply_instantly: bool,
        is_last_retraction: bool,
    ) {
        let prev = std::mem::replace(&mut self.m_is_last_retraction, is_last_retraction);
        self.retract_with_lift_type(lift_type, apply_instantly);
        self.m_is_last_retraction = prev;
    }

    /// R205: GCodeWriter::lazy_lift (GCodeWriter.cpp:425-452), non-spiral-vase,
    /// non-toolchange path. retract_lift_above/below default to always-active
    /// for this pipeline (0..huge); target = z_hop (self.retract_lift).
    pub fn lazy_lift_faithful(&mut self, lift_type: u8) {
        let target_lift = self.retract_lift;
        if self.m_lifted == 0.0 && self.m_to_lift == 0.0 && target_lift > 0.0 {
            self.m_to_lift = target_lift;
            self.m_to_lift_type = lift_type;
        }
    }

    /// R208: GCodeWriter::eager_lift SpiralLift branch (GCodeWriter.cpp:455-495)
    /// — the layer-change lift (GCode.cpp:3039 retract(..., apply_instantly)):
    /// STATIC spiral (ij = {radius, 0}), immediate emission, m_lifted set so the
    /// next unretract's unlift restores Z.
    pub fn eager_spiral_lift(&mut self) {
        let to_lift = self.retract_lift - self.m_lifted;
        if to_lift < 1e-4 {
            return;
        }
        // GCodeWriter.cpp:478 `type == LiftType::SpiralLift && this->is_current_position_clear()`
        // — "spiral lift only safe with known position"; :488 falls back to a
        // normal lift when it is not. R630.
        if !self.is_current_position_clear() {
            if crate::faithful_gate("EAGER_LIFT_TRAVEL_ACCEL") {
                self.set_travel_acceleration(0.0);
            }
            self.travel_to_z(self.z + to_lift, None);
            self.m_lifted = self.retract_lift;
            self.m_to_lift = 0.0;
            return;
        }
        let slope_threshold = 3.0 * std::f64::consts::PI / 180.0;
        let radius = to_lift / (2.0 * std::f64::consts::PI * slope_threshold.atan());
        let target_z = self.z + to_lift;
        let travel_f = self.m_lift_feedrate.unwrap_or(self.config.travel_speed * 60.0);
        // SPIRAL_LIFT_ACCEL (on): `_spiral_travel_to_z` returns
        // `set_travel_acceleration() + "G17\n" + ..` (GCodeWriter.cpp:800). =0
        // writes no acceleration.
        if crate::faithful_gate("SPIRAL_LIFT_ACCEL") {
            self.set_travel_acceleration(0.0);
        }
        self.write_command(&GCodeCommand::SelectXYPlane);
        self.write_raw(&format!(
            "G3 Z{} I{} J0 P1  F{:.0}",
            format_gcode_value(target_z, 3),
            format_gcode_value(radius, 3),
            travel_f
        ));
        self.z = target_z;
        self.last_emitted_f = travel_f;
        self.m_lifted = self.retract_lift;
        self.m_to_lift = 0.0;
    }

    /// GCodeWriter::will_move_z (GCodeWriter.cpp:683-697).
    pub fn will_move_z(&self, z: CoordF) -> bool {
        const EPSILON: CoordF = 1e-4;
        if self.m_lifted > 0.0 {
            let nominal_z = self.z - self.m_lifted;
            if z >= nominal_z - EPSILON && z <= self.z + EPSILON {
                return false;
            }
        } else if (self.z - z).abs() < EPSILON {
            return false;
        }
        true
    }

    /// GCodeWriter::unlift (GCodeWriter.cpp:851-860).
    pub fn unlift_faithful(&mut self) {
        if self.m_lifted > 0.0 {
            let target = self.z - self.m_lifted;
            self.travel_to_z(target, None);
            self.m_lifted = 0.0;
        }
        self.m_to_lift = 0.0;
    }

    /// R205: GCodeWriter::travel_to_xyz (GCodeWriter.cpp:501-612) — the lift
    /// merge: pending lazy lift becomes a slope/spiral/normal pre-move plus ONE
    /// combined XYZ travel at the lifted Z. `z` = the destination nominal Z
    /// (current layer z). Plate offsets are 0 in this pipeline.
    pub fn travel_to_xyz(&mut self, x: CoordF, y: CoordF, z: CoordF) {
        const EPSILON: CoordF = 1e-4;
        let travel_f = self.config.travel_speed * 60.0;
        if self.m_to_lift.abs() > EPSILON {
            let mut dest_z = z;
            // GCodeWriter.cpp:520 `(!this->is_current_position_clear() || m_pos != dest_point)`.
            // R630: the first conjunct was `position_known`, which is never cleared.
            let same_pos = self.is_current_position_clear()
                && (self.x - x).abs() < 1e-12
                && (self.y - y).abs() < 1e-12
                && (self.z - z).abs() < 1e-12;
            if !same_pos && self.m_to_lift + self.z > z {
                self.m_lifted = self.m_to_lift + self.z - z;
                dest_z = self.m_to_lift + self.z;
            }
            self.m_to_lift = 0.0;

            let source = (self.x, self.y, self.z);
            let delta = (x - source.0, y - source.1, dest_z - source.2);
            let delta_no_z_norm = (delta.0 * delta.0 + delta.1 * delta.1).sqrt();
            let slope_threshold = 3.0 * std::f64::consts::PI / 180.0;

            if delta.2 > 0.0 && delta_no_z_norm != 0.0 {
                // GCodeWriter.cpp:537 — `m_to_lift_type == LiftType::SpiralLift
                // && this->is_current_position_clear()`. R630: the second
                // conjunct was `position_known`, which is never cleared.
                if self.m_to_lift_type == 1 && self.is_current_position_clear() {
                    // SpiralLift (GCodeWriter.cpp:536-543): radius from climb,
                    // ij = rotate90(radius * dir).
                    let radius = delta.2 / (2.0 * std::f64::consts::PI * slope_threshold.atan());
                    let dirx = delta.0 / delta_no_z_norm;
                    let diry = delta.1 / delta_no_z_norm;
                    let (ijx, ijy) = (-radius * diry, radius * dirx);
                    self.write_command(&GCodeCommand::SelectXYPlane);
                    // native _spiral_travel_to_z: "G3 Z<z> I<i> J<j> P1  F<f>"
                    // (TWO spaces before F: emit_string("P1 ") + emit_f " F").
                    self.write_raw(&format!(
                        "G3 Z{} I{} J{} P1  F{:.0}",
                        format_gcode_value(dest_z, 3),
                        format_gcode_value(ijx, 3),
                        format_gcode_value(ijy, 3),
                        travel_f
                    ));
                    self.z = dest_z;
                    self.last_emitted_f = travel_f;
                } else if self.m_to_lift_type == 2
                    && self.is_current_position_clear()
                    && delta.2.atan2(delta_no_z_norm) < slope_threshold
                {
                    // SlopeLift (GCodeWriter.cpp:545-559): early climb to the
                    // slope top point, single G1 XYZ.
                    let t = delta.2 / slope_threshold.tan();
                    let topx = source.0 + delta.0 / delta_no_z_norm * t;
                    let topy = source.1 + delta.1 / delta_no_z_norm * t;
                    self.write_command(&GCodeCommand::LinearMove {
                        x: Some(topx),
                        y: Some(topy),
                        z: Some(dest_z),
                        e: None,
                        f: Some(travel_f),
                    });
                    self.x = topx;
                    self.y = topy;
                    self.z = dest_z;
                } else if self.m_to_lift_type == 0 {
                    self.travel_to_z(dest_z, None);
                }
            }

            // xy_z_move (GCodeWriter.cpp:565-580). When the writer does not know
            // where the head is, C++ splits the move: XY first, then Z. R630.
            if self.is_current_position_clear() {
                self.write_command(&GCodeCommand::LinearMove {
                    x: Some(x),
                    y: Some(y),
                    z: Some(dest_z),
                    e: None,
                    f: Some(travel_f),
                });
                self.x = x;
                self.y = y;
                self.z = dest_z;
                self.last_emitted_f = travel_f;
            } else {
                self.write_command(&GCodeCommand::LinearMove {
                    x: Some(x),
                    y: Some(y),
                    z: None,
                    e: None,
                    f: Some(travel_f),
                });
                self.x = x;
                self.y = y;
                self.last_emitted_f = travel_f;
                self.travel_to_z(dest_z, None);
            }
            self.position_known = true;
            // GCodeWriter.cpp:582.
            self.m_is_current_pos_clear = true;
        } else if !self.will_move_z(z) {
            // GCodeWriter.cpp:586-598.
            let nominal_z = self.z - self.m_lifted;
            self.m_lifted -= z - nominal_z;
            if self.m_lifted.abs() < EPSILON {
                self.m_lifted = 0.0;
            }
            // GCodeWriter.cpp:593 — set before the travel_to_xy delegation.
            self.m_is_current_pos_clear = true;
            self.travel_to(x, y, None);
        } else {
            // GCodeWriter.cpp:600-622 — plain XYZ, lift cancelled. C++:606 forces
            // "xy first then z after filament change" when the position is not
            // clear. R630.
            self.m_lifted = 0.0;
            if self.is_current_position_clear() {
                self.write_command(&GCodeCommand::LinearMove {
                    x: Some(x),
                    y: Some(y),
                    z: Some(z),
                    e: None,
                    f: Some(travel_f),
                });
                self.x = x;
                self.y = y;
                self.z = z;
                self.last_emitted_f = travel_f;
            } else {
                self.write_command(&GCodeCommand::LinearMove {
                    x: Some(x),
                    y: Some(y),
                    z: None,
                    e: None,
                    f: Some(travel_f),
                });
                self.x = x;
                self.y = y;
                self.last_emitted_f = travel_f;
                self.travel_to_z(z, None);
            }
            self.position_known = true;
            // GCodeWriter.cpp:622.
            self.m_is_current_pos_clear = true;
        }
    }

    /// Travel move to a specific Z height.
    pub fn travel_to_z(&mut self, z: CoordF, feedrate: Option<CoordF>) {
        let f = feedrate.unwrap_or(self.config.travel_speed * 60.0);

        // Track Z travel time
        let dz = (z - self.z).abs();
        if f > 0.0 && dz > 0.0 {
            self.stats.print_time_seconds += dz * 60.0 / f;
        }

        // Track max Z height
        if z > self.stats.max_z_height {
            self.stats.max_z_height = z;
        }

        // Faithful port of GCodeWriter::_travel_to_z (GCodeWriter.cpp:645-661):
        // BambuStudio constructs a GCodeG1Formatter (emits "G1"), then
        // emit_z(z) + emit_f(travel_speed_z). Z travel is always G1, never G0.
        //   GCodeG1Formatter w;
        //   w.emit_z(z);
        //   w.emit_f(speed * 60.0);
        self.write_command(&GCodeCommand::LinearMove {
            x: None,
            y: None,
            z: Some(z),
            e: None,
            f: if (f - self.last_emitted_f).abs() > 0.01 {
                Some(f)
            } else {
                None
            },
        });

        self.z = z;
        // Z travel: same as travel_to — emitted-F tracker only, not the
        // extrusion speed (C++ _travel_to_z does not touch m_current_speed).
        self.last_emitted_f = f;
    }

    /// Extrusion move (1:1 port of GCodeWriter::extrude_to_xy)
    ///
    /// The caller passes in a DELTA (dE), not an absolute position.
    /// The Extruder class handles the E reset logic automatically.
    ///
    /// GCodeWriter.cpp:698-716
    /// C++: std::string GCodeWriter::extrude_to_xy(const Vec2d &point, double dE, const std::string &comment, bool force_no_extrusion)
    /// C++: {
    /// C++:     m_pos(0) = point(0);
    /// C++:     m_pos(1) = point(1);
    /// C++:     if (!force_no_extrusion)
    /// C++:         filament()->extrude(dE);
    /// C++:     GCodeG1Formatter w;
    /// C++:     w.emit_xy(point_on_plate);
    /// C++:     if (!force_no_extrusion)
    /// C++:         w.emit_e(filament()->E());
    /// C++:     return set_extrude_acceleration() + w.string();
    /// C++: }
    /// Native `extrude_to_xy(..., force_no_extrusion=true)` (GCodeWriter.cpp:705,
    /// 710-711): the move is emitted WITHOUT an E word and the filament E is not
    /// advanced — the "wipe" connector of extrusion_entities_append_paths_with_wipe.
    pub fn wipe_to(&mut self, x: CoordF, y: CoordF, feedrate: Option<CoordF>) {
        let f = feedrate.unwrap_or(if self.feedrate > 0.0 {
            self.feedrate
        } else {
            self.config.print_speed * 60.0
        });
        let dx = x - self.x;
        let dy = y - self.y;
        let dist = (dx * dx + dy * dy).sqrt();
        self.stats.travel_distance_mm += dist;
        if f > 0.0 {
            let move_time = dist * 60.0 / f;
            self.stats.print_time_seconds += move_time;
            self.layer_extrusion_time += move_time;
        }
        self.write_command(&GCodeCommand::LinearMove {
            x: Some(x),
            y: Some(y),
            z: None,
            e: None,
            f: if (f - self.last_emitted_f).abs() > 0.01 {
                Some(f)
            } else {
                None
            },
        });
        if (f - self.last_emitted_f).abs() > 0.01 {
            self.last_emitted_f = f;
        }
        self.x = x;
        self.y = y;
        self.position_known = true;
    }

    pub fn extrude_to(&mut self, x: CoordF, y: CoordF, de: CoordF, feedrate: Option<CoordF>) {
        self.flush_pending_accel();
        let f = feedrate.unwrap_or(if self.feedrate > 0.0 {
            self.feedrate
        } else {
            self.config.print_speed * 60.0
        });

        // Track statistics
        self.stats.extrusion_distance_mm += de.abs();
        self.stats.filament_length_mm += de.abs();

        // Track travel distance and time
        let dx = x - self.x;
        let dy = y - self.y;
        let dist = (dx * dx + dy * dy).sqrt();
        self.stats.travel_distance_mm += dist;
        if f > 0.0 {
            let move_time = dist * 60.0 / f;
            self.stats.print_time_seconds += move_time;
            self.layer_extrusion_time += move_time;
        }

        // Call extruder.extrude(dE) - this handles E reset in relative mode!
        // GCodeWriter.cpp:703
        // C++: filament()->extrude(dE);
        self.extruder.extrude(de);

        // Emit E value from extruder.E() - already correct for relative/absolute mode!
        // GCodeWriter.cpp:710-711
        // C++: if (!force_no_extrusion)
        // C++:     w.emit_e(filament()->E());
        let e_out = self.extruder.e();

        self.write_command(&GCodeCommand::LinearMove {
            x: Some(x),
            y: Some(y),
            z: None,
            e: Some(e_out),
            f: if (f - self.last_emitted_f).abs() > 0.01 {
                Some(f)
            } else {
                None
            },
        });

        self.x = x;
        self.y = y;
        self.feedrate = f;
        self.last_emitted_f = f;
        self.position_known = true;

        // Accumulate wipe path (C++ m_wipe.path)
        if self.wipe_enabled {
            if self.wipe_path.is_empty() || self.wipe_path.last() != Some(&(x, y)) {
                if self.wipe_path.is_empty() {
                    self.wipe_path_src = "accum";
                }
                self.wipe_path.push((x, y));
            }
        }
    }

    /// Arc extrusion move (G2/G3) - 1:1 port of GCodeWriter::extrude_arc_to_xy
    ///
    /// GCodeWriter.cpp:720-738
    /// C++: std::string GCodeWriter::extrude_arc_to_xy(const Vec2d& point, const Vec2d& center_offset, double dE, const bool is_ccw, ...)
    /// C++: {
    /// C++:     m_pos(0) = point(0);
    /// C++:     m_pos(1) = point(1);
    /// C++:     if (!force_no_extrusion)
    /// C++:         filament()->extrude(dE);
    /// C++:     GCodeG2G3Formatter w(is_ccw);
    /// C++:     w.emit_xy(point_on_plate);
    /// C++:     w.emit_ij(center_offset);
    /// C++:     if (!force_no_extrusion)
    /// C++:         w.emit_e(filament()->E());
    /// C++:     return set_extrude_acceleration() + w.string();
    /// C++: }
    pub fn extrude_arc(
        &mut self,
        x: CoordF,
        y: CoordF,
        i: CoordF,
        j: CoordF,
        de: CoordF,
        direction: ArcDirection,
        feedrate: Option<CoordF>,
    ) {
        self.flush_pending_accel();
        let f = feedrate.unwrap_or(if self.feedrate > 0.0 {
            self.feedrate
        } else {
            self.config.print_speed * 60.0
        });

        // Track statistics. Mirror extrude_to so arc moves contribute to the
        // reported total filament length (generator.rs reads filament_length_mm).
        self.stats.extrusion_distance_mm += de.abs();
        self.stats.filament_length_mm += de.abs();

        // Calculate arc length for travel distance tracking
        let radius = (i * i + j * j).sqrt();
        let center_x = self.x + i;
        let center_y = self.y + j;
        let start_angle = (self.y - center_y).atan2(self.x - center_x);
        let end_angle = (y - center_y).atan2(x - center_x);
        let mut arc_angle = end_angle - start_angle;
        match direction {
            ArcDirection::CounterClockwise => {
                if arc_angle < 0.0 {
                    arc_angle += 2.0 * std::f64::consts::PI;
                }
            }
            ArcDirection::Clockwise | ArcDirection::Unknown => {
                if arc_angle > 0.0 {
                    arc_angle -= 2.0 * std::f64::consts::PI;
                }
            }
        }
        let arc_length = radius * arc_angle.abs();
        self.stats.travel_distance_mm += arc_length;

        // Call extruder.extrude(dE) - handles E reset in relative mode!
        // GCodeWriter.cpp:724
        // C++: filament()->extrude(dE);
        self.extruder.extrude(de);

        // Emit E value from extruder.E()
        // GCodeWriter.cpp:731
        // C++: w.emit_e(filament()->E());
        let e_out = self.extruder.e();

        // Write arc command
        let cmd = match direction {
            ArcDirection::Clockwise | ArcDirection::Unknown => GCodeCommand::ArcCW {
                x,
                y,
                i,
                j,
                e: Some(e_out),
                f: if (f - self.last_emitted_f).abs() > 0.01 {
                    Some(f)
                } else {
                    None
                },
            },
            ArcDirection::CounterClockwise => GCodeCommand::ArcCCW {
                x,
                y,
                i,
                j,
                e: Some(e_out),
                f: if (f - self.last_emitted_f).abs() > 0.01 {
                    Some(f)
                } else {
                    None
                },
            },
        };

        self.write_command(&cmd);

        self.x = x;
        self.y = y;
        self.feedrate = f;
        self.last_emitted_f = f;
        self.position_known = true;
    }

    /// Arc extrusion move clockwise (G2).
    pub fn extrude_arc_cw(
        &mut self,
        x: CoordF,
        y: CoordF,
        i: CoordF,
        j: CoordF,
        e: CoordF,
        feedrate: Option<CoordF>,
    ) {
        self.extrude_arc(x, y, i, j, e, ArcDirection::Clockwise, feedrate);
    }

    /// Arc extrusion move counter-clockwise (G3).
    pub fn extrude_arc_ccw(
        &mut self,
        x: CoordF,
        y: CoordF,
        i: CoordF,
        j: CoordF,
        e: CoordF,
        feedrate: Option<CoordF>,
    ) {
        self.extrude_arc(x, y, i, j, e, ArcDirection::CounterClockwise, feedrate);
    }

    /// Retract filament (1:1 port of GCodeWriter::_retract)
    ///
    /// GCodeWriter.cpp:781-811
    /// C++: std::string GCodeWriter::_retract(double length, double restart_extra, const std::string &comment)
    /// C++: {
    /// C++:     std::string gcode;
    /// C++:     if (config.use_firmware_retraction) {
    /// C++:         ...
    /// C++:     } else if (length > 0) {
    /// C++:         double dE = filament()->retract(length, restart_extra);
    /// C++:         if (dE != 0) {
    /// C++:             GCodeG1Formatter w;
    /// C++:             w.emit_e(filament()->E());
    /// C++:             w.emit_f(FILAMENT_CONFIG(retraction_speed) * 60.);
    /// C++:             gcode = w.string();
    /// C++:         }
    /// C++:     }
    /// C++:     return gcode;
    /// C++: }
    /// Retract using the TOOLCHANGE retraction length.
    ///
    /// C++ `GCodeWriter::retract_for_toolchange` / `Wipe::wipe`'s
    /// `toolchange ? retract_length_toolchange() : retraction_length()`
    /// (GCode.cpp:373-375). R608: at tower entry C++ retracts with the toolchange
    /// length and wipes along the object path just printed, emitting a wipe block
    /// whose |E| sums to exactly `retract_length_toolchange * 0.95` = 1.90 on this
    /// profile. We emitted no such block at all (measured: C++ 2,718 in-tower wipe
    /// blocks, us 0), which is the whole of the Prime-tower E/block discrepancy.
    ///
    /// Implemented by swapping the length around the existing `retract()` rather
    /// than duplicating its ~130 lines: C++ applies the toolchange length to the
    /// entire retract+wipe, so this is the same semantics with no risk to the
    /// ordinary path.
    pub fn retract_for_toolchange(&mut self) {
        // R609: TOWER_ENTRY_WHY=1 — report why this no-ops. R608 assumed the writer
        // was already `retracted` at tower entry; the emitted gcode shows no retract
        // there at all, so the blocker is one of the other guards at `retract()`'s
        // wipe branch (`wipe_enabled`, `wipe_path.len() >= 2`). Measured, not
        // inferred (R608).
        if crate::probe_enabled("TOWER_ENTRY_WHY") {
            use std::sync::atomic::{AtomicUsize, Ordering};
            static N: AtomicUsize = AtomicUsize::new(0);
            static RET: AtomicUsize = AtomicUsize::new(0);
            static NOWIPE: AtomicUsize = AtomicUsize::new(0);
            static SHORT: AtomicUsize = AtomicUsize::new(0);
            static OK: AtomicUsize = AtomicUsize::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed) + 1;
            let r = if self.retracted { RET.fetch_add(1, Ordering::Relaxed) + 1 } else { RET.load(Ordering::Relaxed) };
            let w = if !self.wipe_enabled { NOWIPE.fetch_add(1, Ordering::Relaxed) + 1 } else { NOWIPE.load(Ordering::Relaxed) };
            let s = if self.wipe_path.len() < 2 { SHORT.fetch_add(1, Ordering::Relaxed) + 1 } else { SHORT.load(Ordering::Relaxed) };
            let o = if !self.retracted && self.wipe_enabled && self.wipe_path.len() >= 2 {
                OK.fetch_add(1, Ordering::Relaxed) + 1
            } else {
                OK.load(Ordering::Relaxed)
            };
            eprintln!(
                "[TCWHY] n={n} already_retracted={r} wipe_disabled={w} path_short={s} would_wipe={o} (this call: path_len={})",
                self.wipe_path.len()
            );
        }
        let saved = self.retraction_length;
        self.retraction_length = self.config.retract_length_toolchange;
        self.tc_retract_probe = true;
        self.m_toolchange_wipe = true;
        self.retract();
        self.m_toolchange_wipe = false;
        self.tc_retract_probe = false;
        self.retraction_length = saved;
    }

    /// GCode.cpp:7117-7124 — the lift half of `GCode::retract`:
    ///
    /// ```cpp
    /// gcode += toolchange ? m_writer.retract_for_toolchange() : m_writer.retract();
    /// gcode += m_writer.reset_e();
    /// if (m_writer.filament()->retraction_length() > 0 || use_firmware_retraction) {
    ///     if (apply_instantly) gcode += m_writer.eager_lift(lift_type, toolchange);
    ///     else                 gcode += m_writer.lazy_lift(lift_type, ...);
    /// }
    /// ```
    ///
    /// R638: the crucial detail is that this block is **not** guarded by whether
    /// the retraction emitted anything. C++'s `m_writer.retract()` returns an
    /// empty string when the filament is already retracted, and the lift still
    /// runs. Ours returned early from the whole function, so an
    /// already-retracted call emitted no lift at all — which is why R637 found
    /// C++ reaching `eager_lift` 7,538 times against our single call site.
    fn emit_lift_after_retract(&mut self) {
        if lift_faithful_gate() {
            let lt = self
                .m_forced_lift_type
                .unwrap_or_else(|| self.travel_lift_type());
            if crate::probe_enabled("OVERHANG_PRED_CENSUS") {
                eprintln!("OHCONSUME lift_type={}", lt);
            }
            if self.m_apply_lift_instantly {
                // GCode.cpp:7121 `eager_lift` — immediate emission.
                self.eager_lift(lt);
            } else {
                self.lazy_lift_faithful(lt);
            }
        } else {
            self.do_z_hop();
        }
    }

    pub fn retract(&mut self) {
        if self.retracted {
            // GCode.cpp:7112-7124 — C++'s writer-level retract is the no-op here,
            // NOT the lift. Emitting the lift anyway is what R637 measured as the
            // whole remaining spiral deficit. R638.
            if crate::faithful_gate("RETRACT_LIFT_ALWAYS") {
                self.emit_lift_after_retract();
            }
            return;
        }

        let retract_speed = self.config.retract_speed * 60.0;
        let retraction_length = self.retraction_length;

        // R610: per-guard census scoped to TOWER calls only. R609 instrumented three
        // of the six sub-conditions on this path and its decision counter was global,
        // so "enabled 2,718" and "emitted 2" could not be reconciled. Enumerated from
        // the source, the guards between here and `; WIPE_START` are:
        //   (a) self.retracted            -- early return above
        //   (b) lift_faithful_gate()      -- NOT checked by R609's probe
        //   (c) self.wipe_enabled
        //   (d) self.wipe_path.len() >= 2
        //   (e) length >= 0.0
        //   (f) kept.len() >= 2 && acc > 1e-4
        if self.tc_retract_probe && crate::probe_enabled("TOWER_ENTRY_WHY") {
            use std::sync::atomic::{AtomicUsize, Ordering};
            static N: AtomicUsize = AtomicUsize::new(0);
            static NO_GATE: AtomicUsize = AtomicUsize::new(0);
            static NO_WIPE: AtomicUsize = AtomicUsize::new(0);
            static SHORT: AtomicUsize = AtomicUsize::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed) + 1;
            if !lift_faithful_gate() { NO_GATE.fetch_add(1, Ordering::Relaxed); }
            if !self.wipe_enabled { NO_WIPE.fetch_add(1, Ordering::Relaxed); }
            if self.wipe_path.len() < 2 { SHORT.fetch_add(1, Ordering::Relaxed); }
            if n % 500 == 0 || n == 1 {
                eprintln!(
                    "[TCGUARD] tower_calls={n} no_lift_gate={} no_wipe={} path_short={}",
                    NO_GATE.load(Ordering::Relaxed),
                    NO_WIPE.load(Ordering::Relaxed),
                    SHORT.load(Ordering::Relaxed)
                );
            }
        }
        // Wipe during retraction (C++ Wipe::wipe from GCode.cpp:357-433)
        // Retract filament while moving along the reversed last extrusion path
        if lift_faithful_gate() && self.wipe_enabled && self.wipe_path.len() >= 2 {
            // R222: verbatim Wipe::wipe (GCode.cpp:360-433), the whole cluster:
            // forward source-polyline walk, wipe_dist clip, per-segment
            // dE = length*(seg/wipe_dist)*0.95, current-speed wipe (set_speed
            // dedup -> usually no F line), WIPE_START/END markers.
            let length = self.retraction_length * (1.0 - self.config.retract_before_wipe / 100.0);
            if length >= 0.0 {
                // Native: wipe_distance.get_at(filament) — the per-filament
                // value (1.0 on this profile), not the writer's 2.0 default.
                // R807: the profile key is `wipe_distance` (BBS FILAMENT_CONFIG,
                // GCode.cpp:380), parsed into `config.wipe_distance_profile`;
                // `filament_wipe_distance` is only fed by a key no profile
                // carries, so every wipe was clipped at its 1.0 mm default
                // against native's 2.0 (Majora sparse len/wipe 1.000 v 2.000).
                // `WIPE_DIST_CONFIG=0` restores the old field.
                let mut wipe_dist = if crate::faithful_gate("WIPE_DIST_CONFIG") {
                    self.config
                        .wipe_distance_profile
                        .unwrap_or(self.config.filament_wipe_distance)
                } else {
                    self.config.filament_wipe_distance
                };
                // TOOLCHANGE_WIPE_SPEED (on): GCode.cpp:469-472 — a tool change
                // wipes `prime_tower_wipe_distance` when it is set.
                let toolchange_wipe =
                    self.m_toolchange_wipe && crate::faithful_gate("TOOLCHANGE_WIPE_SPEED");
                if toolchange_wipe && self.config.prime_tower_wipe_distance > 0.0 {
                    wipe_dist = self.config.prime_tower_wipe_distance;
                }
                // wipe_path = [current] + stored[1..] (forward).
                let mut pts: Vec<(CoordF, CoordF)> = vec![(self.x, self.y)];
                pts.extend(self.wipe_path.iter().skip(1).copied());
                // clip_end(total - wipe_dist): keep the first wipe_dist mm.
                let mut kept: Vec<(CoordF, CoordF)> = vec![pts[0]];
                let mut acc = 0.0;
                for i in 1..pts.len() {
                    let dx = pts[i].0 - pts[i - 1].0;
                    let dy = pts[i].1 - pts[i - 1].1;
                    let seg = (dx * dx + dy * dy).sqrt();
                    if acc + seg >= wipe_dist {
                        let ratio = (wipe_dist - acc) / seg;
                        kept.push((pts[i - 1].0 + dx * ratio, pts[i - 1].1 + dy * ratio));
                        acc = wipe_dist;
                        break;
                    }
                    kept.push(pts[i]);
                    acc += seg;
                }
                // Handle short path (GCode.cpp:401-405).
                let eff_wipe_dist = if acc < wipe_dist { acc.max(1e-4) } else { wipe_dist };
                // WIPE_CLIP_SCALED (on): Wipe::wipe works on the scaled polyline
                // (GCode.cpp:474-505): `clip_end(length() - scale_(wipe_distance))`
                // puts the new end at `(last + v * (d / |v|)).cast<coord_t>()`,
                // truncated (Polyline.cpp:52-72), and each segment's dE is
                // `length * (line.length() / wipe_dist) * 0.95` in scaled units, so
                // a one-unit shorter or longer end shows in the fifth decimal of E
                // (E1.90001, and `G1 E-.09999` for the rest). =0 clips in mm.
                let mut seg_ratios: Option<Vec<f64>> = None;
                if crate::faithful_gate("WIPE_CLIP_SCALED") {
                    let sf = crate::libslic3r::SCALING_FACTOR;
                    let mut ip: Vec<(i64, i64)> =
                        pts.iter().map(|p| (crate::scale(p.0), crate::scale(p.1))).collect();
                    let seg_len = |a: (i64, i64), b: (i64, i64)| {
                        let dx = (b.0 - a.0) as f64;
                        let dy = (b.1 - a.1) as f64;
                        (dx * dx + dy * dy).sqrt()
                    };
                    let total: f64 = ip.windows(2).map(|w| seg_len(w[0], w[1])).sum();
                    // scale_(wipe_distance): a division, as a double.
                    let mut wd = wipe_dist / sf;
                    let mut distance = total - wd;
                    while distance > 0.0 {
                        let last = ip.pop().unwrap();
                        if ip.is_empty() {
                            break;
                        }
                        let nl = *ip.last().unwrap();
                        let vx = (nl.0 - last.0) as f64;
                        let vy = (nl.1 - last.1) as f64;
                        let lsqr = vx * vx + vy * vy;
                        if lsqr > distance * distance {
                            let r = distance / lsqr.sqrt();
                            ip.push((
                                (last.0 as f64 + vx * r) as i64,
                                (last.1 as f64 + vy * r) as i64,
                            ));
                            break;
                        }
                        distance -= lsqr.sqrt();
                    }
                    if ip.len() >= 2 {
                        let len: f64 = ip.windows(2).map(|w| seg_len(w[0], w[1])).sum();
                        if len < wd {
                            wd = if len < crate::libslic3r::EPSILON { crate::libslic3r::EPSILON } else { len };
                        }
                        seg_ratios = Some(ip.windows(2).map(|w| seg_len(w[0], w[1]) / wd).collect());
                        kept = ip.iter().map(|p| (p.0 as f64 * sf, p.1 as f64 * sf)).collect();
                        acc = len * sf;
                    }
                }
                if crate::probe_enabled("WIPEPROBE") {
                    use std::sync::atomic::{AtomicUsize, Ordering};
                    static N: AtomicUsize = AtomicUsize::new(0);
                    let n = N.fetch_add(1, Ordering::Relaxed) + 1;
                    let head: Vec<String> = self.wipe_path.iter().take(3).map(|p| format!("({:.3},{:.3})", p.0, p.1)).collect();
                    eprintln!(
                        "[WIPE] n={n} src={} wp={} pts={} kept={} acc={acc:.3} wd={wipe_dist} prof={:?} cur=({:.3},{:.3}) cur_scaled=({:.4},{:.4}) head={}",
                        self.wipe_path_src, self.wipe_path.len(), pts.len(), kept.len(), self.config.wipe_distance_profile, self.x, self.y, self.x / crate::libslic3r::SCALING_FACTOR, self.y / crate::libslic3r::SCALING_FACTOR, head.join(" ")
                    );
                }
                // R609: TOWER_ENTRY_WHY=1 also reports the post-guard decision, since
                // the pre-guard census showed 2,718 calls fully enabled while only 2
                // blocks were emitted -- so the block is decided HERE, not above.
                if crate::probe_enabled("TOWER_ENTRY_WHY") {
                    use std::sync::atomic::{AtomicUsize, Ordering};
                    static N: AtomicUsize = AtomicUsize::new(0);
                    static EMIT: AtomicUsize = AtomicUsize::new(0);
                    let n = N.fetch_add(1, Ordering::Relaxed) + 1;
                    let ok = kept.len() >= 2 && acc > 1e-4;
                    let e = if ok { EMIT.fetch_add(1, Ordering::Relaxed) + 1 } else { EMIT.load(Ordering::Relaxed) };
                    if n <= 3 || n % 5000 == 0 {
                        eprintln!(
                            "[WIPEDEC] n={n} emitted={e} | wipe_dist={wipe_dist:.4} acc={acc:.4} kept={} pts={} len={length:.4}",
                            kept.len(), pts.len()
                        );
                    }
                }
                if kept.len() >= 2 && acc > 1e-4 {
                    // R227: native wipe segments are extrude_to_xy calls, so
                    // they surface the pending (reset) accel before WIPE_START's
                    // first move.
                    //
                    // R651 — right idea, WRONG SIDE OF THE MARKER. The M204 census
                    // by preceding line is unambiguous: C++ has **16,932** `M204`
                    // lines whose immediately preceding line is `; WIPE_START`
                    // (16,931 of them `M204 S10000` = default_acceleration), and we
                    // had **zero** — ours flushed one line earlier, which is why our
                    // M204s pile up after `; LINE_WIDTH` (4,491 vs C++'s 38) and
                    // after arcs (G2-preceded 9,165 vs 3,273). Same lines, wrong
                    // place. C++ emits the reset between the tag (GCode.cpp:408) and
                    // the first `extrude_to_xy` of the wipe.
                    if !crate::faithful_gate("WIPE_ACCEL_AFTER_MARKER") {
                        self.flush_pending_accel();
                    }
                    self.write_raw("; WIPE_START");
                    // R811 WIPE_ACCEL_AFTER_SPEED (faithful ON): the pending
                    // acceleration surfaces in the wipe's first extrude_to_xy,
                    // which comes after Wipe::wipe's set_speed line (GCode.cpp
                    // 496-510). Where the wipe writes no F line the two orders
                    // are the same, which is all R651 could see on Majora.
                    let accel_after_speed = crate::faithful_gate("WIPE_ACCEL_AFTER_SPEED");
                    if crate::faithful_gate("WIPE_ACCEL_AFTER_MARKER") && !accel_after_speed {
                        self.flush_pending_accel();
                    }
                    // R610: the decisive marker. Written into the stream immediately
                    // after `; WIPE_START` for TOWER calls only. If the file ends up
                    // containing ~2,718 of these, the lines ARE emitted and the
                    // counting was wrong; if it contains ~2, the tower calls never
                    // reach here and the guard census above says which one stops them.
                    if self.tc_retract_probe && crate::probe_enabled("TOWER_ENTRY_WHY") {
                        use std::sync::atomic::{AtomicUsize, Ordering};
                        static M: AtomicUsize = AtomicUsize::new(0);
                        let m = M.fetch_add(1, Ordering::Relaxed) + 1;
                        self.write_raw(&format!("; _R610_{m}"));
                    }
                    // R229: native Wipe::wipe ALWAYS emits its speed line —
                    // GCodeWriter::set_speed has NO dedup (GCodeWriter.cpp:387-399
                    // unconditionally formats G1 F), so every wipe carries
                    // `G1 F<cur>;_WIPE` (role_base_wipe_speed default true →
                    // cur speed). The R222 dedup dropped ~1 native F line per
                    // layer (the n-only G1 F12000 bucket).
                    // TOOLCHANGE_WIPE_SPEED (on): GCode.cpp:450-453 — a tool
                    // change wipes at `prime_tower_wipe_speed`, else at
                    // `prime_tower_max_speed`; `set_speed` makes that the
                    // current speed (GCodeWriter.cpp:450), which the wipe
                    // when leaving the tower then takes. =0 the current speed.
                    // GCODE_WIPE_SPEED_PIN (faithful): GCode.cpp:447-449
                    //   cur_speed = writer.get_current_speed();
                    //   wipe_speed = role_base_wipe_speed && cur_speed > EPSILON
                    //       ? cur_speed / 60
                    //       : travel_speed * wipe_speed.value / 100;
                    // and `set_speed(wipe_speed * 60)` (GCode.cpp:498), which always
                    // writes and makes it the current speed (GCodeWriter.cpp:450).
                    // Before any `set_speed` (Majora L1's first tower exit) the
                    // current speed is 0 and the wipe runs at 80 % of the travel
                    // speed (`G1 F24000`). =0 writes no F line when the current
                    // speed is 0, and the current speed itself otherwise.
                    let wipe_speed_pin = crate::faithful_gate("GCODE_WIPE_SPEED_PIN");
                    let wipe_f = if toolchange_wipe {
                        let tc = self.config.prime_tower_wipe_speed;
                        (if tc > 0.0 { tc } else { self.config.prime_tower_max_speed }) * 60.0
                    } else if wipe_speed_pin {
                        let cur_speed = self.feedrate;
                        let wipe_speed = if self.m_role_base_wipe_speed && cur_speed > crate::libslic3r::EPSILON {
                            cur_speed / 60.0
                        } else {
                            self.config.travel_speed * self.m_wipe_speed_percent / 100.0
                        };
                        wipe_speed * 60.0
                    } else {
                        self.feedrate
                    };
                    if wipe_f > 0.0 {
                        let fs = format!("{:.3}", wipe_f);
                        let fs = fs.trim_end_matches('0').trim_end_matches('.');
                        // GCode.cpp:497-498: `(enable_cooling_markers() && !is_last)
                        // ? ";_WIPE" : ""` (GCODE_LAST_WIPE_PIN, faithful).
                        if self.m_is_last_retraction && crate::faithful_gate("GCODE_LAST_WIPE_PIN") {
                            self.write_raw(&format!("G1 F{}", fs));
                        } else {
                            self.write_raw(&format!("G1 F{};_WIPE", fs));
                        }
                        self.last_emitted_f = wipe_f;
                        if toolchange_wipe || wipe_speed_pin {
                            self.feedrate = wipe_f;
                        }
                    }
                    if crate::faithful_gate("WIPE_ACCEL_AFTER_MARKER") && accel_after_speed {
                        self.flush_pending_accel();
                    }
                    let mut wiped_sum: CoordF = 0.0;
                    for i in 1..kept.len() {
                        let dx = kept[i].0 - kept[i - 1].0;
                        let dy = kept[i].1 - kept[i - 1].1;
                        let seg = (dx * dx + dy * dy).sqrt();
                        let de = match &seg_ratios {
                            Some(r) => length * r[i - 1] * 0.95,
                            None => length * (seg / eff_wipe_dist) * 0.95,
                        };
                        wiped_sum += de;
                        self.extruder.extrude(-de);
                        let e_out = self.extruder.e();
                        self.write_command(&GCodeCommand::LinearMove {
                            x: Some(kept[i].0),
                            y: Some(kept[i].1),
                            z: None,
                            e: Some(e_out),
                            f: None,
                        });
                    }
                    self.write_raw("; WIPE_END");
                    let end = *kept.last().unwrap();
                    self.x = end.0;
                    self.y = end.1;
                    // Remaining retraction (whatever the wipe didn't consume).
                    // WIPE_CLIP_SCALED: `Extruder::retract(length)` retracts
                    // `length - m_retracted`, the sum of the wipe's dE
                    // (Extruder.cpp:71-91, 101-130).
                    let wiped: CoordF = if seg_ratios.is_some() {
                        wiped_sum
                    } else {
                        length * 0.95 * (acc.min(eff_wipe_dist) / eff_wipe_dist)
                    };
                    let remaining = self.retraction_length - wiped;
                    if remaining > 1e-6 {
                        self.extruder.extrude(-remaining);
                        let e_out = self.extruder.e();
                        self.write_command(&GCodeCommand::LinearMove {
                            x: None,
                            y: None,
                            z: None,
                            e: Some(e_out),
                            f: Some(retract_speed),
                        });
                        self.last_emitted_f = retract_speed;
                    }
                }
            }
            self.wipe_path.clear();
        } else if self.wipe_enabled && self.wipe_path.len() >= 2 {
            // Build reversed wipe path from current position
            let mut wipe_pts: Vec<(CoordF, CoordF)> = Vec::new();
            wipe_pts.push((self.x, self.y)); // current position
                                             // Reverse the stored path and append.
            // R221 NOTE: native Wipe::wipe walks FORWARD ([cur]+points[1..],
            // source polyline, multi-segment dE = len*(seg/wipe_dist)*0.95,
            // role-based speed). Two re-land attempts regressed (+487/+1110):
            // the whole wipe cluster (walk + source path + segment dE + speed)
            // must land TOGETHER as a verbatim Wipe::wipe port — queued.
            for &(px, py) in self.wipe_path.iter().rev().skip(1) {
                wipe_pts.push((px, py));
            }

            // Clip to wipe_distance
            let wipe_dist = self.wipe_distance;
            let mut clipped_pts: Vec<(CoordF, CoordF)> = vec![wipe_pts[0]];
            let mut accumulated = 0.0;
            for i in 1..wipe_pts.len() {
                let dx = wipe_pts[i].0 - wipe_pts[i - 1].0;
                let dy = wipe_pts[i].1 - wipe_pts[i - 1].1;
                let seg_len = (dx * dx + dy * dy).sqrt();
                if accumulated + seg_len >= wipe_dist {
                    // Clip this segment
                    let remaining = wipe_dist - accumulated;
                    let ratio = remaining / seg_len;
                    let clip_x = wipe_pts[i - 1].0 + dx * ratio;
                    let clip_y = wipe_pts[i - 1].1 + dy * ratio;
                    clipped_pts.push((clip_x, clip_y));
                    accumulated = wipe_dist;
                    break;
                }
                clipped_pts.push(wipe_pts[i]);
                accumulated += seg_len;
            }

            if clipped_pts.len() >= 2 && accumulated > 0.001 {
                // Wipe speed: use current feedrate or configured wipe speed
                let wipe_speed = self.feedrate.max(1000.0); // At least 1000 mm/min

                self.write_raw("; WIPE_START");
                self.write_command(&GCodeCommand::LinearMove {
                    x: None,
                    y: None,
                    z: None,
                    e: None,
                    f: Some(wipe_speed),
                });

                // Distribute retraction across wipe path segments
                // C++: dE = length * (segment_length / wipe_dist) * 0.95
                let total_wipe_dist = accumulated;
                let retract_during_wipe = retraction_length * 0.95; // 95% during wipe

                for i in 1..clipped_pts.len() {
                    let dx = clipped_pts[i].0 - clipped_pts[i - 1].0;
                    let dy = clipped_pts[i].1 - clipped_pts[i - 1].1;
                    let seg_len = (dx * dx + dy * dy).sqrt();
                    let de = retract_during_wipe * (seg_len / total_wipe_dist);
                    self.extruder.extrude(-de);
                    let e_out = self.extruder.e();
                    self.write_command(&GCodeCommand::LinearMove {
                        x: Some(clipped_pts[i].0),
                        y: Some(clipped_pts[i].1),
                        z: None,
                        e: Some(e_out),
                        f: None,
                    });
                }

                self.write_raw("; WIPE_END");

                // Remaining retraction (5% not done during wipe)
                let remaining_retract = retraction_length * 0.05;
                self.extruder.extrude(-remaining_retract);
                let e_out = self.extruder.e();
                self.write_command(&GCodeCommand::LinearMove {
                    x: None,
                    y: None,
                    z: None,
                    e: Some(e_out),
                    f: Some(retract_speed),
                });

                // Update position to end of wipe path
                if let Some(&(wx, wy)) = clipped_pts.last() {
                    self.x = wx;
                    self.y = wy;
                }
            } else {
                // Wipe path too short — do simple retraction
                let de = -retraction_length;
                self.extruder.extrude(de);
                let e_out = self.extruder.e();
                self.write_command(&GCodeCommand::LinearMove {
                    x: None,
                    y: None,
                    z: None,
                    e: Some(e_out),
                    f: Some(retract_speed),
                });
            }

            // Clear wipe path after use
            self.wipe_path.clear();
        } else {
            // No wipe path — simple retraction
            let de = -retraction_length;
            self.extruder.extrude(de);
            let e_out = self.extruder.e();
            self.write_command(&GCodeCommand::LinearMove {
                x: None,
                y: None,
                z: None,
                e: Some(e_out),
                f: Some(retract_speed),
            });
            // RETRACT_F_NOTED (on): `GCodeWriter::_retract` emits the F of the
            // retraction (GCodeWriter.cpp `w.emit_f(speed * 60.)`), and that F is
            // the current one for what follows, as with the wipe's retraction
            // above. =0 leaves the previous feedrate as the current one.
            if crate::faithful_gate("RETRACT_F_NOTED") {
                self.last_emitted_f = retract_speed;
            }
        }

        self.emit_lift_after_retract();

        self.retracted = true;
        self.m_retracted_length = self.retraction_length;
        self.stats.retraction_count += 1;
    }

    /// Retract filament without z-hop lift.
    /// Used in change_layer() where z-hop is handled separately.
    pub fn retract_no_lift(&mut self) {
        if self.retracted {
            return;
        }

        let retract_speed = self.config.retract_speed * 60.0;
        let retraction_length = self.retraction_length;

        let de = -retraction_length;
        self.extruder.extrude(de);
        let e_out = self.extruder.e();
        self.write_command(&GCodeCommand::LinearMove {
            x: None,
            y: None,
            z: None,
            e: Some(e_out),
            f: Some(retract_speed),
        });

        self.retracted = true;
        self.m_retracted_length = self.retraction_length;
        self.stats.retraction_count += 1;
    }

    /// Perform Z-hop only (no filament retraction).
    ///
    /// Use this after a wipe move has already retracted the filament.
    /// This completes the retraction sequence by doing the Z-hop and
    /// marking the writer as retracted so that `unretract()` will
    /// handle the descent and filament restore.
    ///
    /// BambuStudio reference: In `_retract()`, the z-hop happens after
    /// the wipe+retract sequence. The wipe handles filament retraction
    /// along the wipe path, then z-hop lifts the nozzle.
    pub fn z_hop_only(&mut self) {
        if self.retracted {
            return;
        }

        // Emit the remaining retraction amount (after wipe partial retract).
        // BambuStudio: `retract_rest = retract_length - retract_before_wipe`
        // For simplicity, emit a small final retract to ensure full retraction.
        let retract_speed = self.config.retract_speed * 60.0;
        let remaining = self.retraction_length * 0.05;

        if remaining > 0.001 {
            // Call extruder.extrude(-remaining)
            let de = -remaining;
            self.extruder.extrude(de);
            let e_out = self.extruder.e();

            self.write_command(&GCodeCommand::LinearMove {
                x: None,
                y: None,
                z: None,
                e: Some(e_out),
                f: Some(retract_speed),
            });
        }

        self.emit_lift_after_retract();

        self.retracted = true;
        self.m_retracted_length = self.retraction_length;
        self.stats.retraction_count += 1;
    }

    /// Internal: perform Z-hop (spiral or normal) without changing retraction state.
    fn do_z_hop(&mut self) {
        if self.retract_lift > 0.0 {
            self.z_before_lift = self.z;
            let target_z = self.z + self.retract_lift;

            match self.config.z_hop_type {
                ZHopType::Spiral | ZHopType::Auto => {
                    self.spiral_travel_to_z(target_z);
                }
                // zhtNormal and zhtSlope both perform a linear lift here (the
                // slope-specific geometry is not modeled in this writer shim).
                ZHopType::Normal | ZHopType::Slope => {
                    self.travel_to_z(target_z, None);
                }
            }
        }
    }

    /// Unretract (restore) filament (1:1 port of GCodeWriter::unretract)
    ///
    /// GCodeWriter.cpp:813-825
    /// C++: std::string GCodeWriter::unretract(float extra_retract)
    /// C++: {
    /// C++:     std::string gcode;
    /// C++:     if (config.use_firmware_retraction) {
    /// C++:         gcode += FLAVOR_IS(gcfMachinekit) ? "G23 ;unretract \n" : "G11 ;unretract \n";
    /// C++:         gcode += reset_e();
    /// C++:     } else {
    /// C++:         double dE = filament()->unretract();
    /// C++:         if (dE != 0) {
    /// C++:             GCodeG1Formatter w;
    /// C++:             w.emit_e(filament()->E());
    /// C++:             w.emit_f(std::lrint(FILAMENT_CONFIG(deretraction_speed)) * 60.);
    /// C++:             gcode = w.string();
    /// C++:         }
    /// C++:     }
    /// C++:     return gcode;
    /// C++: }
    /// Sync the tracked retract state to raw gcode that already unretracted.
    ///
    /// R611: the wipe-tower block is spliced in as RAW TEXT and contains its own
    /// unretract (the `change_filament_gcode` template's `G1 E<n>`, plus R466's
    /// `G1 E{retract_length_toolchange}` trailer). The writer never sees those, so
    /// after R608's tower-entry retract its `retracted` flag stays true for the rest
    /// of the print and every subsequent `retract()` early-returns -- which is why
    /// enabling `TOWER_ENTRY_WIPE_CPP` drove after-toolchange wipes from 2,715 to 0.
    ///
    /// This clears the flag WITHOUT emitting, because the machine is already
    /// unretracted; calling `unretract()` would add a Z-unlift and an E move that C++
    /// does not have at this point. Same pattern as R475's `set_last_extrusion_role`
    /// immediately below the tower's `write_raw_content` -- raw writes change machine
    /// state the writer tracks separately, and that state has to be re-synced by hand.
    pub fn mark_unretracted_after_raw(&mut self) {
        self.retracted = false;
    }

    /// The tower block written as raw text ends with `GCode::unretract()`
    /// (GCode.cpp:1253), which unlifts before it deretracts
    /// (GCodeWriter.cpp `unlift`): the nozzle is back at the nominal Z and
    /// nothing is lifted.
    /// After a tower block written as raw text the nozzle is at the tower
    /// layer's Z (the travel to the tower took it there, GCode.cpp:790-798, and
    /// the block's own unretract unlifted), whatever Z was tracked before.
    pub fn set_nominal_z_after_raw(&mut self, z: CoordF) {
        self.z = z;
        self.m_lifted = 0.0;
        self.m_to_lift = 0.0;
    }

    pub fn mark_unlifted_after_raw(&mut self) {
        if self.m_lifted > 0.0 {
            self.z -= self.m_lifted;
            self.m_lifted = 0.0;
        }
        self.m_to_lift = 0.0;
    }

    pub fn unretract(&mut self) {
        if !self.retracted {
            return;
        }

        // Z unlift. Faithful path (R206): GCode::unretract = writer.unlift() +
        // writer.unretract() (GCode.hpp:232) — G1 Z restore from m_lifted.
        if lift_faithful_gate() {
            if self.m_lifted > 0.0 {
                let target = self.z - self.m_lifted;
                self.write_command(&GCodeCommand::LinearMove {
                    x: None,
                    y: None,
                    z: Some(target),
                    e: None,
                    f: None,
                });
                self.z = target;
                self.m_lifted = 0.0;
            }
            self.m_to_lift = 0.0;
        } else if self.retract_lift > 0.0 && self.z > self.z_before_lift {
            match self.config.z_hop_type {
                ZHopType::Spiral | ZHopType::Auto => {
                    // BambuStudio uses G1 Z (linear move) for the descent after spiral lift
                    self.write_command(&GCodeCommand::LinearMove {
                        x: None,
                        y: None,
                        z: Some(self.z_before_lift),
                        e: None,
                        f: None,
                    });
                    self.z = self.z_before_lift;
                }
                ZHopType::Normal | ZHopType::Slope => {
                    self.travel_to_z(self.z_before_lift, None);
                }
            }
        }

        let unretract_speed = self.config.deretract_speed * 60.0;

        // Call extruder.extrude(+length) to unretract
        // GCodeWriter.cpp:819
        // C++: double dE = filament()->unretract();
        // UNRETRACT_RETRACTED_LENGTH (on): `Extruder::unretract` gives back
        // what the retraction took (m_retracted), which after
        // `retract_for_toolchange` is the tool-change length, not the
        // ordinary one. =0 always gives back `retraction_length`.
        let de = if crate::faithful_gate("UNRETRACT_RETRACTED_LENGTH") && self.m_retracted_length > 0.0 {
            self.m_retracted_length
        } else {
            self.retraction_length
        };
        self.extruder.extrude(de);

        // Emit E value from extruder.E()
        // GCodeWriter.cpp:821
        // C++: w.emit_e(filament()->E());
        let e_out = self.extruder.e();

        self.write_command(&GCodeCommand::LinearMove {
            x: None,
            y: None,
            z: None,
            e: Some(e_out),
            f: Some(unretract_speed),
        });

        self.retracted = false;
    }

    /// Perform a spiral (helical) Z-hop to the target Z height.
    ///
    /// Emits a helical G3 arc that traces one full circle in XY while
    /// simultaneously lifting from the current Z to `target_z`. This produces
    /// smoother Z-hops that reduce stringing and surface marks.
    ///
    /// BambuStudio reference: GCodeWriter.cpp `_spiral_travel_to_z()` lines 661–680
    ///
    /// Output pattern:
    /// ```text
    /// G17                              ; select XY plane for arcs
    /// G3 Z{target_z} I{i} J{j} P1 F{speed}  ; helical CCW arc, 1 revolution
    /// ```
    ///
    /// The arc center offset (I, J) is computed so the arc traces a circle of
    /// `spiral_lift_radius` centered at `(current_x + radius, current_y)`.
    pub fn spiral_travel_to_z(&mut self, target_z: CoordF) {
        // Dynamic radius computation matching C++ GCodeWriter.cpp:
        // radius = to_lift / (2 * PI * atan(slope_threshold))
        // slope_threshold = 3 * PI / 180 (3 degrees)
        let to_lift = (target_z - self.z).abs();
        let slope_threshold = 3.0 * std::f64::consts::PI / 180.0;
        let radius = if to_lift > 0.001 {
            to_lift / (2.0 * std::f64::consts::PI * slope_threshold.atan())
        } else {
            0.8 // fallback
        };
        let travel_speed = self.config.travel_speed * 60.0;

        // Select XY plane for helical arc interpretation
        self.write_command(&GCodeCommand::SelectXYPlane);

        // C++ static spiral: ij_offset = { radius, 0 }
        // The center is at (current_x + radius, current_y)
        let i = radius;
        let j = 0.0;

        // Emit the helical arc: one full revolution (P1) lifting to target_z
        self.write_command(&GCodeCommand::HelicalArcCCW {
            z: target_z,
            i,
            j,
            p: 1,
            f: Some(travel_speed),
        });

        // Update internal state — XY position returns to start after full revolution
        self.z = target_z;
        // Spiral lift is a travel-class move: track the emitted F only.
        self.last_emitted_f = travel_speed;
    }

    /// Set total layer count for per-layer notifications.
    pub fn set_total_layers(&mut self, total: usize) {
        self.total_layers = total;
    }

    /// Start a new layer.
    pub fn start_layer(&mut self, layer_index: usize, z: CoordF, layer_height: CoordF) {
        // Reset layer time tracking for cooling
        self.layer_extrusion_time = 0.0;
        self.cooling_slowdown = 1.0; // Reset — will be computed at end of layer

        self.layer_index = layer_index;
        self.layer_z = z;
        self.stats.layer_count = layer_index + 1;

        // Track max Z height
        if z > self.stats.max_z_height {
            self.stats.max_z_height = z;
        }

        // BambuStudio emits "; CHANGE_LAYER" before every layer (used by validators)
        self.write_raw("; CHANGE_LAYER");
        // Cooling marker: CoolingBuffer inserts fan speed commands here
        // C++ GCode.cpp:3983
        self.write_raw(";_SET_FAN_SPEED_CHANGING_LAYER");
        // Round to avoid floating-point noise (1.9999999 → 2)
        let z_rounded = (z * 1000.0).round() / 1000.0;
        let h_rounded = (layer_height * 1000.0).round() / 1000.0;
        // Use minimal decimal representation
        let z_str = if z_rounded == z_rounded.floor() {
            format!("{:.0}", z_rounded)
        } else {
            format!("{}", z_rounded)
        };
        let h_str = if h_rounded == h_rounded.floor() {
            format!("{:.0}", h_rounded)
        } else {
            format!("{}", h_rounded)
        };
        self.write_raw(&format!("; Z_HEIGHT: {}", z_str));
        self.write_raw(&format!("; LAYER_HEIGHT: {}", h_str));

        // Layer 0: retract from nozzle load line (no wipe path available)
        // Layer 1+: retraction+wipe happens at END of previous layer (in print.rs)
        // So we only do the simple retraction for layer 0
        if layer_index == 0 && !self.retracted && self.retraction_length > 0.0 {
            let retract_speed = self.config.retract_speed * 60.0;
            let de = -self.retraction_length;
            self.extruder.extrude(de);
            let e_out = self.extruder.e();
            self.write_command(&GCodeCommand::LinearMove {
                x: None,
                y: None,
                z: None,
                e: Some(e_out),
                f: Some(retract_speed),
            });
            self.retracted = true;
            self.m_retracted_length = self.retraction_length;
            self.stats.retraction_count += 1;
        }

        // Per-layer notifications (matching BambuStudio output)
        let total_layers = if self.total_layers > 0 {
            self.total_layers
        } else {
            self.stats.layer_count.max(1)
        };
        self.write_raw(&format!(
            "; layer num/total_layer_count: {}/{}",
            layer_index + 1,
            total_layers
        ));
        self.write_raw("; update layer progress");
        self.write_raw(&format!("M73 L{}", layer_index + 1));
        self.write_raw(&format!("M991 S0 P{} ;notify layer change", layer_index));

        // M73 P/R progress estimate (percentage and remaining time)
        if total_layers > 0 {
            let pct = ((layer_index as f64 / total_layers as f64) * 100.0).round() as u32;
            let total_time_min = (self.stats.print_time_seconds / 60.0).ceil() as u32;
            let remaining_min = ((total_time_min as f64)
                * (1.0 - layer_index as f64 / total_layers as f64))
                .ceil() as u32;
            self.write_raw(&format!("M73 P{} R{}", pct.min(100), remaining_min));
        }

        // BambuStudio-specific per-layer commands
        if layer_index == 1 {
            // Layer 2: power loss recovery + model scan
            self.write_raw("; open powerlost recovery");
            self.write_raw("M1003 S1");
            self.write_raw("M976 S1 P1 ; scan model before printing 2nd layer");
            self.write_raw("M400 P100");
            // Unretract/retract for model scan
            self.write_raw(&format!(
                "G1 E{} F1800",
                format_gcode_value(self.retraction_length, 1)
            ));
            self.write_raw(&format!(
                "G1 E{} F1800",
                format_gcode_value(-self.retraction_length, 1)
            ));
        }

        // Per-layer fan control (BambuStudio turns fan on after first layer(s))
        if layer_index == 0 {
            // First layer: fan off
            self.write_raw("M106 S0");
            self.write_raw("M106 P2 S0");
        } else if layer_index == 1 {
            // Second layer: fan on
            self.write_raw("M106 S255");
            self.write_raw("M106 P2 S178");
        }

        // Travel acceleration and Z-hop (after fan, before timelapse)
        // Skip if already at hop height from end-of-previous-layer retraction
        if self.retracted && self.retract_lift > 0.0 && self.z < z + 0.01 {
            // Need to hop — we're at layer Z, not hop height
            self.write_raw("M204 S6000");
            let lift = (self.retract_lift * 0.5).max(0.2);
            let target_z = z + lift;
            self.write_command(&GCodeCommand::LinearMove {
                x: None,
                y: None,
                z: Some(target_z),
                e: None,
                f: Some(30000.0),
            });
            self.z = target_z;
        }

        // BambuStudio date marker and timelapse/skippable block
        self.write_raw(";========Date 20250206========");
        self.write_raw("; SKIPPABLE_START");
        self.write_raw("; SKIPTYPE: timelapse");
        self.write_raw("M622.1 S1 ; for prev firmware, default turned on");
        self.write_raw("M1002 judge_flag timelapse_record_flag");
        self.write_raw("M622 J1");
        self.write_raw(" ; timelapse without wipe tower");
        self.write_raw("M971 S11 C10 O0");
        self.write_raw("M1004 S5 P1  ; external shutter");
        self.write_raw("");
        self.write_raw("M623");
        self.write_raw("; SKIPPABLE_END");

        // Object ID marker (BambuStudio emits this for object tracking)
        self.write_raw("");
        self.write_raw("; OBJECT_ID: 0");

        // Z descent to layer happens in print.rs after travel to first point
        // (matching BambuStudio: travel at hop height, then G1 Z{layer_z})
    }

    /// Travel to Z using G1 (linear move). Updates internal z state.
    /// Used for descent from Z-hop to layer Z where G1 is required (matching BambuStudio).
    pub fn travel_to_z_linear(&mut self, z: CoordF) {
        self.write_command(&GCodeCommand::LinearMove {
            x: None,
            y: None,
            z: Some(z),
            e: None,
            f: None,
        });
        self.z = z;
        self.z_before_lift = z; // Prevent unretract from doing Z descent
    }

    /// Get the current layer's accumulated extrusion time (for cooling decisions).
    pub fn layer_time(&self) -> f64 {
        self.layer_extrusion_time
    }

    /// Set cooling slowdown factor for speed adjustment.
    /// Values > 1.0 mean slowdown (divide speed by this factor).
    pub fn set_cooling_slowdown(&mut self, factor: f64) {
        self.cooling_slowdown = factor;
    }

    /// Get the current layer Z height.
    pub fn get_layer_z(&self) -> CoordF {
        self.layer_z
    }

    /// Reset the extruder position.
    pub fn reset_e(&mut self) {
        self.write_command(&GCodeCommand::SetPosition {
            x: None,
            y: None,
            z: None,
            e: Some(0.0),
        });
        self.extruder.reset_e();
    }

    /// Set absolute positioning mode.
    pub fn set_absolute_positioning(&mut self, absolute: bool) {
        if absolute != self.absolute_positioning {
            if absolute {
                self.write_command(&GCodeCommand::AbsolutePositioning);
            } else {
                self.write_command(&GCodeCommand::RelativePositioning);
            }
            self.absolute_positioning = absolute;
        }
    }

    /// Set absolute extrusion mode.
    pub fn set_absolute_extrusion(&mut self, absolute: bool) {
        if absolute != self.absolute_extrusion {
            if absolute {
                self.write_command(&GCodeCommand::AbsoluteExtrusion);
            } else {
                self.write_command(&GCodeCommand::RelativeExtrusion);
            }
            self.absolute_extrusion = absolute;
        }
    }

    /// Check if a tool change is needed.
    ///
    /// C++ reference: GCodeWriter::need_toolchange()
    /// GCodeWriter.cpp:150-155
    pub fn need_toolchange(&self, new_extruder: usize) -> bool {
        new_extruder != self.extruder_index
    }

    /// GCodeWriter.cpp:69-86 `void GCodeWriter::set_extruders(std::vector<unsigned int>
    /// extruder_ids)`: the filaments the print uses, sorted; "we enable support for multiple
    /// extruder if any extruder greater than 0 is used (even if prints only uses that one) since
    /// we need to output Tx commands; first extruder has index 0".
    ///
    /// FIDELITY-NOTE: this writer keeps one `Extruder` state rather than one per filament
    /// (`m_filament_extruders`), so the ids are recorded, and `m_curr_extruder_id`, the
    /// per-extruder pointers and `update_exclusive_nozzles` have nothing to reset here.
    pub fn set_extruders(&mut self, extruder_ids: &[u32]) {
        let mut extruder_ids = extruder_ids.to_vec();
        extruder_ids.sort_unstable();
        // GCodeWriter.cpp:83 `this->multiple_extruders = (*std::max_element(...)) > 0;`
        self.multiple_extruders = extruder_ids.iter().max().copied().unwrap_or(0) > 0;
        self.m_extruder_ids = Some(extruder_ids);
    }

    /// GCodeWriter.hpp `const std::vector<Extruder>& extruders() const`: the ids of the
    /// filaments `set_extruders` installed (empty before it is called).
    pub fn extruders(&self) -> &[u32] {
        self.m_extruder_ids.as_deref().unwrap_or(&[])
    }

    /// Whether the export has called `set_extruders` (GCode.cpp:2817, 2851), so that
    /// `multiple_extruders` says what the C++ member says.
    pub fn extruders_set(&self) -> bool {
        self.m_extruder_ids.is_some()
    }

    /// GCodeWriter.hpp:29 `multiple_extruders`.
    ///
    /// WRITER_MULTIPLE_EXTRUDERS (on): the member `set_extruders` sets. =0 is the earlier stub,
    /// always false.
    pub fn has_multiple_extruders(&self) -> bool {
        if crate::faithful_gate("WRITER_MULTIPLE_EXTRUDERS") {
            self.multiple_extruders
        } else {
            false
        }
    }

    /// `GCode::m_wipe.enable` as `GCode::set_extruders` sets it (GCode.cpp:6437-6443): wipe
    /// paths are generated when any extruder of the print has `wipe` on.
    pub fn set_wipe_enable(&mut self, enable: bool) {
        self.wipe_enabled = enable;
    }

    /// Set the current extruder without emitting G-code.
    ///
    /// C++ reference: GCodeWriter::set_extruder()
    /// GCodeWriter.cpp:160-165
    pub fn set_extruder(&mut self, extruder: usize) {
        self.extruder_index = extruder;
    }

    /// Get the current extruder.
    pub fn extruder(&self) -> usize {
        self.extruder_index
    }

    /// Convert mm³/mm to filament mm/mm using extruder's e_per_mm3
    /// GCode.cpp:6081
    /// C++: double e_per_mm = m_writer.filament()->e_per_mm3() * _mm3_per_mm;
    pub fn extruder_e_per_mm(&self, mm3_per_mm: CoordF) -> CoordF {
        self.extruder.e_per_mm(mm3_per_mm)
    }

    /// Whether arc fitting (G2/G3 emission) is enabled for this print.
    /// C++ reference: GCode.cpp:6670 m_config.enable_arc_fitting,
    /// LayerRegion.cpp:790 print_config.enable_arc_fitting
    pub fn arc_fitting_enabled(&self) -> bool {
        self.config.arc_fitting_enabled
    }

    /// Whether spiral (vase) mode is enabled. Arc fitting is disabled in spiral mode.
    /// C++ reference: GCode.cpp:6672 m_config.spiral_mode,
    /// LayerRegion.cpp:789 print_config.spiral_mode
    pub fn spiral_mode(&self) -> bool {
        self.config.spiral_vase
    }

    /// Toolpath simplification resolution in mm.
    /// C++ reference: LayerRegion.cpp:791 scaled<double>(print_config.resolution.value)
    pub fn resolution(&self) -> CoordF {
        self.config.resolution
    }

    /// Get the current speed (from last set_speed call).
    ///
    /// C++ reference: GCodeWriter::get_current_speed()
    pub fn get_current_speed(&self) -> CoordF {
        self.feedrate / 60.0 // Convert mm/min to mm/s
    }

    /// Set the feedrate (speed).
    ///
    /// C++ reference: GCodeWriter::set_speed()
    /// GCodeWriter.cpp:200-210
    ///
    /// # Arguments
    /// * `speed` - Feedrate in mm/min
    /// * `comment` - Optional comment to append
    pub fn set_speed(&mut self, speed: CoordF, comment: &str) {
        // Apply cooling slowdown if active (divide speed by slowdown factor)
        let adjusted_speed = if self.cooling_slowdown > 1.001 {
            (speed / self.cooling_slowdown).max(10.0 * 60.0) // min 10mm/s = 600 mm/min
        } else {
            speed
        };
        // Update the logical extrusion speed (C++ m_current_speed) regardless of
        // whether a new F line is needed.
        self.feedrate = adjusted_speed;
        // Native GCodeWriter::set_speed (GCodeWriter.cpp:387) emits
        // UNCONDITIONALLY — a same-value `G1 F..;_EXTRUDE_SET_SPEED` line is the
        // GCodeEditor's adjustable-block opener, and the editor removes
        // redundant F lines AFTER the cooling pass consumed the markers
        // (GCodeEditor.cpp write_layer_gcode: new_feedrate == current_feedrate
        // → skip line). Deduping here starved the cooling model: layer 81 had
        // 121 adjustable blocks vs native 207 (R757). Without the gate, emit
        // only when the value last written to the gcode differs — after a
        // travel (F60000 inline), re-asserting the extrusion speed re-emits and
        // prevents the travel feedrate from leaking onto extrusion moves.
        if crate::faithful_gate("SET_SPEED_ALWAYS_EMIT")
            || (self.last_emitted_f - adjusted_speed).abs() > 0.01
        {
            self.last_emitted_f = adjusted_speed;
            // Native GCodeWriter::set_speed formats F via GCodeG1Formatter::emit_f
            // (XYZF_EXPORT_DIGITS = 3, trailing zeros/dot trimmed) — fractional
            // smooth-ramp speeds print as e.g. "F8828.637" and pass through the
            // cooling rewrite untouched (GCodeEditor only re-formats slowed
            // lines, as integers). Gated: default keeps the integer format
            // (byte-locked 147987).
            let f_str = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let t = format!("{:.3}", adjusted_speed);
                t.trim_end_matches('0').trim_end_matches('.').to_string()
            } else {
                format!("{:.0}", adjusted_speed)
            };
            let mut line = format!("G1 F{}", f_str);
            if !comment.is_empty() {
                // C++ appends cooling markers (;_EXTRUDE_SET_SPEED etc.) directly
                // without a " ; " prefix. Only add " ; " for regular comments.
                if comment.starts_with(';') {
                    // SET_SPEED_MARKER_NOSPACE (on): GCodeWriter::set_speed appends
                    // the cooling marker right after the F word
                    // (`w.emit_string(cooling_marker)`). =0 puts a space between.
                    if !crate::faithful_gate("SET_SPEED_MARKER_NOSPACE") {
                        line.push(' ');
                    }
                } else {
                    line.push_str(" ; ");
                }
                line.push_str(comment);
            }
            self.gcode.append_line(&line);
        }
    }

    /// Get a reference to the built G-code.
    ///
    /// C++ reference: GCodeWriter::gcode()
    pub fn get_gcode(&self) -> &GCode {
        &self.gcode
    }

    /// Extrude to XY position with specified E delta.
    ///
    /// C++ reference: GCodeWriter::extrude_to_xy()
    /// GCodeWriter.cpp:400-420
    ///
    /// This emits a G1 move with X, Y, and E coordinates.
    /// The E value can be negative for retraction during wipe.
    ///
    /// # Arguments
    /// * `x` - Target X position (mm)
    /// * `y` - Target Y position (mm)
    /// * `de` - E delta (change in E, can be negative)
    /// * `comment` - Optional comment
    pub fn extrude_to_xy(&mut self, x: CoordF, y: CoordF, de: f64, comment: Option<&str>) {
        let new_e = if self.absolute_extrusion {
            self.extruder.e() + de
        } else {
            de
        };

        let out_x = x - self.config.extruder_offset_x;
        let out_y = y - self.config.extruder_offset_y;
        let mut gcode = format!(
            "G1 X{} Y{} E{}",
            format_gcode_value(out_x, 3),
            format_gcode_value(out_y, 3),
            format_gcode_value(new_e, 5)
        );

        if let Some(c) = comment {
            gcode.push_str(&format!(" ; {}", c));
        }

        self.gcode.append_line(&gcode);

        self.x = x;
        self.y = y;
        if self.absolute_extrusion {
            // E tracking is handled by extruder.extrude() calls
        }
        self.position_known = true;
    }

    /// Set the XY position without emitting G-code.
    ///
    /// C++ reference: GCodeWriter::set_last_pos()
    /// GCode.cpp:1500-1505
    ///
    /// This updates the internal position tracking without moving.
    /// Used after operations that implicitly change position (like wipe).
    ///
    /// # Arguments
    /// * `x` - New X position (mm)
    /// * `y` - New Y position (mm)
    /// After raw G-code written past this writer: the feedrate its last move
    /// set becomes the one a following move is compared with. C++ writes the
    /// feedrate on every travel and the G-code editor of the cooling buffer
    /// drops it where the text before already set it (GCodeEditor.cpp); this
    /// writer drops it when writing, so it has to know what the text set.
    pub(crate) fn note_raw_feedrate(&mut self, text: &str) {
        for line in text.lines().rev() {
            let l = line.trim_start();
            if !(l.starts_with("G0 ") || l.starts_with("G1 ") || l.starts_with("G2 ") || l.starts_with("G3 ")) {
                continue;
            }
            let code = l.split(';').next().unwrap_or("");
            if let Some(f) = code
                .split_whitespace()
                .find_map(|w| w.strip_prefix('F').and_then(|v| v.parse::<f64>().ok()))
            {
                self.last_emitted_f = f;
                return;
            }
        }
    }

    /// `set_position` with only the Z changed (GCode.cpp:4803-4805).
    pub fn set_position_z(&mut self, z: CoordF) {
        self.z = z;
    }

    pub fn set_position_xy(&mut self, x: CoordF, y: CoordF) {
        self.x = x;
        self.y = y;
        self.position_known = true;
    }
}

impl Default for GCodeWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Debug for GCodeWriter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "GCodeWriter(pos=({:.3}, {:.3}, {:.3}), e={:.3}, layer={})",
            self.x,
            self.y,
            self.z,
            self.extruder.e(),
            self.layer_index
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_writer_new() {
        let writer = GCodeWriter::new();
        assert!(!writer.is_position_known());
        assert!(!writer.is_retracted());
        assert_eq!(writer.layer_index(), 0);
    }

    #[test]
    fn test_writer_preamble_relative_e() {
        // Default config uses relative E mode (M83)
        let mut writer = GCodeWriter::new();
        writer.write_preamble();

        let gcode = writer.gcode();
        assert!(gcode.content().contains("G90")); // Absolute positioning
        assert!(gcode.content().contains("M83")); // Relative extrusion (default)
        assert!(gcode.content().contains("G92")); // Reset E
    }

    #[test]
    fn test_writer_preamble_absolute_e() {
        // Test absolute E mode (M82)
        let mut config = PrintConfig::default();
        config.use_relative_e = false;
        let mut writer = GCodeWriter::with_config(config);
        writer.write_preamble();

        let gcode = writer.gcode();
        assert!(gcode.content().contains("G90")); // Absolute positioning
        assert!(gcode.content().contains("M82")); // Absolute extrusion
        assert!(gcode.content().contains("G92")); // Reset E
    }

    #[test]
    fn test_writer_travel() {
        let mut writer = GCodeWriter::new();
        writer.travel_to(10.0, 20.0, None);

        assert!(writer.is_position_known());
        assert!((writer.position().x - 10.0).abs() < 1e-6);
        assert!((writer.position().y - 20.0).abs() < 1e-6);

        let gcode = writer.gcode();
        // BambuStudio's GCodeWriter::travel_to_xy uses GCodeG1Formatter (G1),
        // never G0 — there is no GCodeG0Formatter in BambuStudio.
        assert!(gcode.content().contains("G1"));
        assert!(!gcode.content().contains("G0"));
        // GCodeFormatter::emit_axis drops trailing zeros.
        assert!(gcode.content().contains("X10 Y20"));
    }

    #[test]
    fn test_writer_extrude_relative_e() {
        // Default is relative E mode (M83)
        let mut writer = GCodeWriter::new();
        writer.write_preamble(); // Sets up relative E mode

        // The third argument is dE, as in GCodeWriter::extrude_to_xy(point, dE).
        // Extruder::extrude (Extruder.cpp:71-88) resets E before adding dE when
        // the distances are relative, so E is the last dE.
        writer.extrude_to(10.0, 10.0, 1.0, None);
        assert!((writer.e() - 1.0).abs() < 1e-6);

        writer.extrude_to(20.0, 20.0, 2.5, None);
        assert!((writer.e() - 2.5).abs() < 1e-6);

        let gcode = writer.gcode();
        assert!(gcode.content().contains("G1"));
        // In relative mode, E values are deltas
        // (GCodeFormatter::emit_axis drops trailing zeros.)
        assert!(gcode.content().contains(" E1")); // First move
        assert!(gcode.content().contains(" E2.5")); // Second move
    }

    #[test]
    fn test_writer_extrude_absolute_e() {
        // Test absolute E mode (M82)
        let mut config = PrintConfig::default();
        config.use_relative_e = false;
        let mut writer = GCodeWriter::with_config(config);
        writer.write_preamble();

        writer.extrude_to(10.0, 10.0, 1.0, None);
        assert!((writer.e() - 1.0).abs() < 1e-6);

        // dE again: with absolute distances E accumulates (Extruder.cpp:71-88).
        writer.extrude_to(20.0, 20.0, 2.5, None);
        assert!((writer.e() - 3.5).abs() < 1e-6);

        let gcode = writer.gcode();
        assert!(gcode.content().contains("G1"));
        // In absolute mode, E values are absolute positions
        assert!(gcode.content().contains(" E1"));
        assert!(gcode.content().contains(" E3.5"));
    }

    /// GCode.cpp:497-498, 3566 (GCODE_LAST_WIPE_PIN): the last wipe of the print writes its speed line
    /// without the `;_WIPE` cooling marker; any other wipe keeps it.
    #[test]
    fn test_retract_is_last_wipe_speed_without_marker() {
        let run = |is_last: bool| {
            let mut writer = GCodeWriter::new();
            writer.write_preamble();
            writer.extrude_to(10.0, 10.0, 1.0, Some(1800.0));
            writer.set_wipe_path_points(vec![(10.0, 10.0), (0.0, 10.0)]);
            writer.retract_with_lift_type_is_last(1, true, is_last);
            let content = writer.gcode().content().to_string();
            let wipe = content.split("; WIPE_START\n").nth(1).map(|s| s.to_string());
            wipe
        };
        if !(lift_faithful_gate() && crate::faithful_gate("GCODE_LAST_WIPE_PIN")) {
            return;
        }
        let last = run(true).expect("a wipe");
        let other = run(false).expect("a wipe");
        let first_line = |s: &str| s.lines().next().unwrap_or("").to_string();
        assert!(first_line(&last).starts_with("G1 F"), "last wipe: {last}");
        assert!(!first_line(&last).contains(";_WIPE"), "last wipe: {last}");
        assert!(first_line(&other).ends_with(";_WIPE"), "other wipe: {other}");
        assert_eq!(first_line(&last) + ";_WIPE", first_line(&other));
    }

    /// GCode.cpp:447-449 + 498: with no current speed yet (`m_current_speed`
    /// 0) the wipe runs at `travel_speed * wipe_speed / 100` and writes it;
    /// GCode.cpp:3818-3825 (called at :1353): the stored acceleration is the
    /// first layer's, written by the wipe's first move after the reset.
    #[test]
    fn test_wipe_speed_fallback_and_tcr_extrude_acceleration() {
        if !(lift_faithful_gate() && crate::faithful_gate("GCODE_WIPE_SPEED_PIN")) {
            return;
        }
        let mut config = PrintConfig::default();
        config.travel_speed = 500.0;
        config.initial_layer_acceleration = 500.0;
        config.default_acceleration = 10000.0;
        let mut writer = GCodeWriter::with_config(config);
        writer.write_preamble();
        writer.set_wipe_speed_config(true, 80.0);
        writer.set_extrude_acceleration_for_layer(true);
        writer.reset_last_acceleration();
        writer.set_position_xy(10.0, 10.0);
        writer.set_wipe_path_points(vec![(10.0, 10.0), (0.0, 10.0)]);
        writer.mark_unretracted_after_raw();
        writer.retract_with_lift_type(1, true);
        let content = writer.gcode().content().to_string();
        let wipe = content.split("; WIPE_START\n").nth(1).expect("a wipe").to_string();
        let mut lines = wipe.lines();
        assert_eq!(lines.next(), Some("G1 F24000;_WIPE"), "wipe: {wipe}");
        if crate::faithful_gate("WIPE_ACCEL_AFTER_MARKER") && crate::faithful_gate("WIPE_ACCEL_AFTER_SPEED") {
            assert_eq!(lines.next(), Some("M204 S500"), "wipe: {wipe}");
        }
    }

    #[test]
    fn test_writer_retract() {
        let mut writer = GCodeWriter::new();
        assert!(!writer.is_retracted());

        writer.retract();
        assert!(writer.is_retracted());

        // Retract again should be no-op
        let len_before = writer.gcode().len();
        writer.retract();
        let len_after = writer.gcode().len();
        assert_eq!(len_before, len_after);

        writer.unretract();
        assert!(!writer.is_retracted());
    }

    #[test]
    fn test_writer_start_layer() {
        let mut writer = GCodeWriter::new();
        writer.start_layer(5, 1.0, 0.2);

        assert_eq!(writer.layer_index(), 5);
        assert_eq!(writer.stats().layer_count, 6);

        let gcode = writer.gcode();
        assert!(gcode.content().contains("; Z_HEIGHT: 1"));
        assert!(gcode.content().contains("; LAYER_HEIGHT: 0.2"));
    }

    #[test]
    fn test_writer_temperatures() {
        let mut writer = GCodeWriter::new();
        writer.set_bed_temperature(60, false);
        writer.set_extruder_temperature(200, true);

        let gcode = writer.gcode();
        assert!(gcode.content().contains("M140 S60"));
        assert!(gcode.content().contains("M109 S200"));
    }

    #[test]
    fn test_writer_fan() {
        let mut writer = GCodeWriter::new();
        writer.set_fan_speed(255);
        writer.set_fan_speed(0);

        let gcode = writer.gcode();
        assert!(gcode.content().contains("M106 S255"));
        assert!(gcode.content().contains("M107"));
    }

    #[test]
    fn test_writer_home() {
        let mut writer = GCodeWriter::new();
        writer.home(true, true, true);

        assert!(writer.is_position_known());
        assert!((writer.position().x).abs() < 1e-6);
        assert!((writer.position().y).abs() < 1e-6);
        assert!((writer.z()).abs() < 1e-6);

        let gcode = writer.gcode();
        assert!(gcode.content().contains("G28"));
    }

    #[test]
    fn test_writer_finish() {
        let mut writer = GCodeWriter::new();
        writer.write_preamble();
        writer.start_layer(0, 0.3, 0.3);
        writer.extrude_to(10.0, 10.0, 1.0, None);

        let gcode = writer.finish();
        assert!(gcode.stats.layer_count > 0);
        assert!(gcode.stats.extrusion_distance_mm > 0.0);
    }

    #[test]
    fn test_writer_stats_tracking() {
        let mut writer = GCodeWriter::new();

        // Travel
        writer.travel_to(100.0, 0.0, None);
        assert!((writer.stats().travel_distance_mm - 100.0).abs() < 1e-6);

        // Extrusion
        writer.extrude_to(100.0, 100.0, 10.0, None);
        assert!((writer.stats().extrusion_distance_mm - 10.0).abs() < 1e-6);

        // Retraction
        writer.retract();
        assert_eq!(writer.stats().retraction_count, 1);
    }

    #[test]
    fn test_writer_arc_cw() {
        let mut writer = GCodeWriter::new();
        writer.travel_to(10.0, 0.0, None);

        // Arc from (10, 0) to (0, 10) with center at (0, 0)
        // I = 0 - 10 = -10, J = 0 - 0 = 0
        writer.extrude_arc_cw(0.0, 10.0, -10.0, 0.0, 1.0, Some(1200.0));

        assert!((writer.position().x - 0.0).abs() < 1e-6);
        assert!((writer.position().y - 10.0).abs() < 1e-6);
        assert!((writer.e() - 1.0).abs() < 1e-6);

        let gcode = writer.gcode();
        assert!(gcode.content().contains("G2"));
        // emit_axis strips trailing zeros / leading zero: I-10.000 -> I-10, J0.000 -> J0.
        assert!(gcode.content().contains("I-10 "));
        assert!(gcode.content().contains("J0 "));
    }

    #[test]
    fn test_writer_arc_ccw() {
        let mut writer = GCodeWriter::new();
        writer.travel_to(10.0, 0.0, None);

        // Arc from (10, 0) to (0, 10) with center at (0, 0)
        writer.extrude_arc_ccw(0.0, 10.0, -10.0, 0.0, 1.0, Some(1200.0));

        assert!((writer.position().x - 0.0).abs() < 1e-6);
        assert!((writer.position().y - 10.0).abs() < 1e-6);

        let gcode = writer.gcode();
        assert!(gcode.content().contains("G3"));
    }

    #[test]
    fn test_writer_arc_stats() {
        use crate::circle::ArcDirection;

        let mut writer = GCodeWriter::new();
        writer.travel_to(10.0, 0.0, None);

        // Quarter circle arc with radius 10
        // Expected arc length ≈ π * 10 / 2 ≈ 15.7
        let initial_travel = writer.stats().travel_distance_mm;
        writer.extrude_arc(
            0.0,
            10.0,
            -10.0,
            0.0,
            1.0,
            ArcDirection::CounterClockwise,
            None,
        );

        let arc_length = writer.stats().travel_distance_mm - initial_travel;
        let expected_length = std::f64::consts::PI * 10.0 / 2.0;
        assert!(
            (arc_length - expected_length).abs() < 0.5,
            "Arc length {} should be close to {}",
            arc_length,
            expected_length
        );
    }
}

/// Per-layer retraction context (RetractWhenCrossingPerimeters, faithful port).
/// internal_islands: expolygons of internal slices; wall_lines: their boundary
/// segments + the perimeter polylines of internal regions.
pub struct RetractCtx {
    pub internal_islands: Vec<crate::geometry::ExPolygon>,
    pub wall_lines: Vec<(crate::Point, crate::Point)>,
    pub density_ok: bool,
    /// `scale_(inner_wall_line_width > 0 ? inner_wall_line_width : nozzle_diameter)`
    /// (GCode.cpp:8478-8479), in millimetres here.
    pub wall_proximity_distance: f64,
    /// The tree over `wall_lines` (`m_aabbtree_lines_distancer`), built when
    /// the first travel asks for it.
    pub lines_distancer: std::cell::OnceCell<crate::aabb_tree_lines::LinesDistancer>,
}

fn segs_intersect(p1: crate::Point, p2: crate::Point, p3: crate::Point, p4: crate::Point) -> bool {
    fn orient(a: crate::Point, b: crate::Point, c: crate::Point) -> i32 {
        let v = (b.x - a.x) as i128 * (c.y - a.y) as i128 - (b.y - a.y) as i128 * (c.x - a.x) as i128;
        v.signum() as i32
    }
    fn on_seg(a: crate::Point, b: crate::Point, p: crate::Point) -> bool {
        p.x >= a.x.min(b.x) && p.x <= a.x.max(b.x) && p.y >= a.y.min(b.y) && p.y <= a.y.max(b.y)
    }
    let d1 = orient(p3, p4, p1);
    let d2 = orient(p3, p4, p2);
    let d3 = orient(p1, p2, p3);
    let d4 = orient(p1, p2, p4);
    if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
        return true;
    }
    (d1 == 0 && on_seg(p3, p4, p1))
        || (d2 == 0 && on_seg(p3, p4, p2))
        || (d3 == 0 && on_seg(p1, p2, p3))
        || (d4 == 0 && on_seg(p1, p2, p4))
}

/// travel_inside_internal_regions_no_wall_crossing
/// (RetractWhenCrossingPerimeters.cpp:114-118): travel fully inside an internal
/// island AND not crossing any wall line.
/// RETRACT_NEAR_WALL (on): the third condition of
/// `travel_inside_internal_regions_no_wall_crossing`
/// (RetractWhenCrossingPerimeters.cpp:190-198): a travel that runs within
/// `wall_proximity_distance` of a wall for `retraction_minimum_travel` or longer
/// keeps its retraction. =0 leaves the condition out.
fn travel_near_wall_too_long(
    ctx: &RetractCtx,
    from: crate::Point,
    to: crate::Point,
    retraction_minimum_travel: f64,
) -> bool {
    if !crate::faithful_gate("RETRACT_NEAR_WALL") {
        return false;
    }
    let distancer = ctx.lines_distancer.get_or_init(|| {
        crate::aabb_tree_lines::LinesDistancer::new(
            ctx.wall_lines
                .iter()
                .map(|&(a, b)| crate::geometry::Line::new(a, b))
                .collect(),
        )
    });
    let travel = crate::geometry::Polyline::from_points(vec![from, to]);
    // `scale_(..)` into a coord_t parameter truncates.
    let sf = crate::libslic3r::SCALING_FACTOR;
    crate::gcode::retract_when_crossing_perimeters::travel_near_perimeters_too_long(
        distancer,
        &travel,
        (ctx.wall_proximity_distance / sf) as crate::Coord,
        (retraction_minimum_travel / sf) as crate::Coord,
    )
}

fn travel_inside_internal_no_crossing(ctx: &RetractCtx, from: crate::Point, to: crate::Point) -> bool {
    // inside = both endpoints inside one island and the segment does not leave
    // it (native: diff_pl(travel, island) empty).
    let mut inside = false;
    'outer: for island in &ctx.internal_islands {
        if island.contains_point(&from) && island.contains_point(&to) {
            // segment must not cross the island boundary
            for ring in std::iter::once(&island.contour).chain(island.holes.iter()) {
                let pts = &ring.points;
                for i in 0..pts.len() {
                    let a = pts[i];
                    let b = pts[(i + 1) % pts.len()];
                    if segs_intersect(from, to, a, b) {
                        continue 'outer;
                    }
                }
            }
            inside = true;
            break;
        }
    }
    if !inside {
        return false;
    }
    // travel_cross_perimeters: no intersection with any wall line
    for &(a, b) in &ctx.wall_lines {
        if segs_intersect(from, to, a, b) {
            return false;
        }
    }
    true
}
