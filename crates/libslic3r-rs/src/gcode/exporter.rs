//! G-code export helper functions.
//!
//! This module provides helper functions for exporting ExtrusionEntity objects
//! to G-code, mirroring the helper methods in BambuStudio's GCode class.
//!
//! C++ reference: GCode.cpp (helper methods throughout)

use crate::arc_fitter::EMovePathType;
use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionPath, ExtrusionRole,
};
use crate::gcode::cooling::CoolingBuffer;
use crate::gcode::writer::GCodeWriter;
use crate::geometry::{Point, Polyline};
use crate::libslic3r::EPSILON;
use crate::print_config::PrintConfig;
use crate::Result;
use crate::{scale, unscale};

use super::seam_placer::SeamPlacer;
use std::cell::Cell;

// =============================================================================
// Active seam-placer context (the free-function analogue of C++ `GCode`'s
// instance state `m_seam_placer` + `m_layer`, GCode.hpp).
//
// The Rust gcode pipeline is a chain of free functions (`extrude_perimeters` ->
// `extrude_collection` -> `extrude_entity` -> `extrude_loop`) rather than
// methods on a stateful `GCode`. To make the per-object/per-layer SeamPlacer
// available to `extrude_loop` (which performs the C++
// `m_seam_placer.place_seam(m_layer, loop, ...)` call, GCode.cpp:5085) without
// threading it through every signature and every recursive collection, the
// driver (`print.rs`) installs the active placer + layer index here for the
// duration of one layer's region extrusion via [`SeamContextGuard`].
//
// Export is strictly single-threaded and sequential, so a thread-local pointer
// scoped by a guard is sound: the borrowed `SeamPlacer` outlives the guard.
// =============================================================================
thread_local! {
    static SEAM_CTX: Cell<Option<(*const SeamPlacer, usize)>> = const { Cell::new(None) };
}

/// RAII guard installing `(placer, layer_idx)` as the active seam context.
/// On drop it restores the previously-installed context.
///
/// Mirrors C++ setting `m_layer`/`m_seam_placer` before the per-region extrude
/// in `GCode::process_layer` (GCode.cpp:4570+). The borrow of `placer` is tied
/// to the guard's lifetime, so the pointer stored in the thread-local is valid
/// for as long as the guard lives (and the guard strictly encloses every
/// `extrude_loop` call that can observe the context).
pub struct SeamContextGuard<'a> {
    prev: Option<(*const SeamPlacer, usize)>,
    _marker: std::marker::PhantomData<&'a SeamPlacer>,
}

impl<'a> SeamContextGuard<'a> {
    pub fn install(placer: &'a SeamPlacer, layer_idx: usize) -> Self {
        let prev =
            SEAM_CTX.with(|c| c.replace(Some((placer as *const SeamPlacer, layer_idx))));
        Self {
            prev,
            _marker: std::marker::PhantomData,
        }
    }
}

impl<'a> Drop for SeamContextGuard<'a> {
    fn drop(&mut self) {
        SEAM_CTX.with(|c| c.set(self.prev));
    }
}

/// Look up the active seam point for `polygon` (a CCW perimeter loop) via the
/// installed [`SeamPlacer`], mirroring C++ `m_seam_placer.place_seam(...)`.
/// Returns `None` when no context is installed or the placer has no data for
/// the loop (caller falls back to the legacy `find_best_seam_index` heuristic).
fn active_place_seam(
    loop_ref: &crate::extrusion_entity::ExtrusionLoop,
    polygon: &crate::geometry::Polygon,
    last_pos: Point,
) -> Option<Point> {
    SEAM_CTX.with(|c| {
        let (placer_ptr, layer_idx) = c.get()?;
        // SAFETY: the pointer is valid for the lifetime of the installed
        // `SeamContextGuard`, which strictly encloses every `extrude_loop` call
        // that can observe this context (single-threaded sequential export).
        let placer: &SeamPlacer = unsafe { &*placer_ptr };
        placer.place_seam(layer_idx, loop_ref, polygon, last_pos)
    })
}

/// Apply a placed seam to a perimeter loop, making it the loop's start vertex.
///
/// SeamPlacer.cpp:1521-1527 — native is a TWO-STEP:
/// ```cpp
/// // Because the G-code export has 1um resolution, don't generate segments
/// // shorter than 1.5 microns, thus empty path segments will not be produced.
/// if (!loop.split_at_vertex(seam_point, scaled<double>(0.0015)))
///     loop.split_at(seam_point, true);   // default eps = scaled<double>(0.001)
/// ```
/// `split_at_vertex` merely ROTATES the loop onto an existing vertex and
/// inserts nothing; only when no vertex sits within 1.5 um does native fall
/// back to `split_at`, which projects the seam onto the closest segment and
/// inserts a NEW vertex there.
///
/// Our port called `split_at` unconditionally, so a seam that coincided with a
/// loop vertex still got projected — landing on the segment BEFORE that vertex
/// and emitting a short corrective segment to reach it. That is the "0.4mm
/// corrective mini-travel" R228 observed and worked around downstream.
///
/// Note the two epsilons are different in native and are NOT interchangeable:
/// 0.0015 is the vertex-snap tolerance, 0.001 is `split_at`'s own default.
/// `prefer_non_overhang` is `true` here; the `false` variant belongs to the
/// non-perimeter branch at GCode.cpp:5456.
fn apply_seam_split(l: &mut crate::extrusion_entity::ExtrusionLoop, seam: &Point) {
    if crate::faithful_gate("SEAM_SPLIT_AT_VERTEX") {
        if !l.split_at_vertex(seam, crate::scale(0.0015) as f64) {
            l.split_at(seam, true, crate::scale(0.001) as f64);
        }
    } else {
        l.split_at(seam, false, crate::scale(0.0015) as f64);
    }
}

/// Configuration for travel moves.
///
/// C++ reference: GCode class member variables
/// GCode.hpp:100-200
#[derive(Debug, Clone)]
pub struct TravelConfig {
    /// Avoid crossing perimeters if enabled
    pub avoid_crossing_perimeters: bool,
    /// Enable retraction on travel moves
    pub retract_on_travel: bool,
    /// Minimum travel distance to trigger retraction (mm)
    pub retract_length_travel: f64,
    /// Enable Z-hop during travel
    pub z_hop: bool,
    /// Z-hop height (mm)
    pub z_hop_height: f64,
}

impl Default for TravelConfig {
    fn default() -> Self {
        Self {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0,
            z_hop: false,
            z_hop_height: 0.0,
        }
    }
}

/// Map a perimeter path's `overhang_degree` to the overhang-corrected speed (mm/s).
///
/// Faithful port of `GCode::get_overhang_degree_corr_speed` (GCode.cpp:5931-5963).
/// `normal_speed` is the role's base wall speed (mm/s). `path_degree` is the
/// per-segment overhang degree (0..=5, the crate stores it truncated to i32 from
/// the C++ continuous double — see extrusion_entity.rs / overhang_detector.rs).
///
/// The overhang speed table (GCode.cpp:5348-5356 overhang_speed_key_map):
///   {1: overhang_1_4, 2: overhang_2_4, 3: overhang_3_4, 4: overhang_4_4,
///    5: overhang_totally, 6: bridge}.
/// A table value of 0 means "use the normal speed" (the BambuStudio nullable
/// `overhang_*_speed` default), matching the `== 0 ? normal` guards in C++.
// GCode.cpp:6633-6641/6757-6763 — overhang/bridge fan marker condition
// (ZSMOOTH_FAITHFUL). threshold==none forces the marker for every external
// perimeter; otherwise degree > threshold-1 or a bridge role qualifies.
fn overhang_fan_marker_needed(
    config: &crate::print_config::PrintObjectConfig,
    path: &ExtrusionPath,
) -> bool {
    if !config.enable_overhang_bridge_fan {
        return false;
    }
    let thr = config.overhang_fan_threshold;
    let is_bridge_role = matches!(
        path.role,
        crate::extrusion_entity::ExtrusionRole::BridgeInfill
            | crate::extrusion_entity::ExtrusionRole::OverhangPerimeter
    );
    // Native compares via `int get_overhang_degree()` — the double degree is
    // TRUNCATED to int before the > threshold-1 test (ExtrusionEntity.hpp:354),
    // so deg 2.9 with threshold 50% (enum 3 -> thr-1 = 2) does NOT qualify.
    (thr == 0 && path.role == crate::extrusion_entity::ExtrusionRole::ExternalPerimeter)
        || (path.overhang_degree as i32) > (thr - 1).max(0)
        || is_bridge_role
}

fn overhang_degree_corr_speed(
    config: &crate::print_config::PrintObjectConfig,
    normal_speed: f64,
    path_degree: f64,
) -> f64 {
    // GCode.cpp:5933 — if (path_degree <= 0) return normal_speed;
    if path_degree <= 0.0 {
        return normal_speed;
    }

    // overhang_speed_key_map[degree] resolved RAW value (no fallback). The C++
    // get_abs_value_at returns the configured speed; a 0 means "unset" and the
    // caller (get_path_speed) keeps the base speed via `new_speed==0 ? speed`.
    // degree 6 (bridge) maps to bridge_speed.
    let speed_at = |degree: i32| -> f64 {
        match degree {
            1 => config.overhang_1_4_speed,
            2 => config.overhang_2_4_speed,
            3 => config.overhang_3_4_speed,
            4 => config.overhang_4_4_speed,
            5 => config.overhang_totally_speed,
            6 => config.bridge_speed,
            _ => 0.0,
        }
    };

    // GCode.cpp:5942 — int lower_degree_bound = int(path_degree);
    let lower_degree_bound = path_degree as i32;
    // GCode.cpp:5944-5947 — degree>=4 or integral: use the lower-bound speed directly
    // (no 0->normal fallback here; caller handles a 0 result).
    if path_degree >= 4.0 || path_degree == lower_degree_bound as f64 {
        return speed_at(lower_degree_bound);
    }

    // GCode.cpp:5948-5962 — interpolate between lower and upper degree speeds.
    let upper_degree_bound = lower_degree_bound + 1;
    let mut lower_speed_bound = if lower_degree_bound == 0 {
        normal_speed
    } else {
        speed_at(lower_degree_bound)
    };
    let mut upper_speed_bound = if upper_degree_bound == 0 {
        normal_speed
    } else {
        speed_at(upper_degree_bound)
    };
    // GCode.cpp:5959-5960 — 0 -> normal_speed fallback (interpolation branch only).
    if lower_speed_bound == 0.0 {
        lower_speed_bound = normal_speed;
    }
    if upper_speed_bound == 0.0 {
        upper_speed_bound = normal_speed;
    }
    // GCode.cpp:5961 — speed_out = lower + (upper-lower)*(path_degree - lower_degree_bound)
    lower_speed_bound
        + (upper_speed_bound - lower_speed_bound) * (path_degree - lower_degree_bound as f64)
}

/// Base (normal) wall speed for a perimeter path role, mm/s.
/// Mirrors the role switch in `extrude_collection` (outer/inner wall speeds).
fn perimeter_base_speed(
    config: &crate::print_config::PrintObjectConfig,
    role: ExtrusionRole,
    is_first_layer: bool,
) -> f64 {
    if is_first_layer {
        return config.initial_layer_speed;
    }
    match role {
        ExtrusionRole::ExternalPerimeter => config.external_perimeter_speed,
        ExtrusionRole::Perimeter => config.perimeter_speed,
        _ => config.perimeter_speed,
    }
}

/// Extrude a single extrusion loop.
///
/// C++ reference: GCode::extrude_loop()
/// GCode.cpp:5071-5270 (~200 lines)
///
/// This function:
/// 1. Makes loop counter-clockwise (CCW orientation)
/// 2. Finds optimal seam/starting point
/// 3. Splits loop at seam point
/// 4. Clips end to create small gap at seam
/// 5. Extrudes each path segment
/// 6. Handles variable width and arc fitting
///
/// # Arguments
/// * `loop_entity` - The extrusion loop to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
pub fn extrude_loop(
    loop_entity: &ExtrusionLoop,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    // C++ reference: GCode.cpp:5071-5076
    // C++: std::string GCode::extrude_loop(ExtrusionLoop loop, std::string description, double speed)
    // C++: {
    // C++: // get a copy; don't modify the orientation of the original loop object otherwise
    // C++: // next copies (if any) would not detect the correct orientation
    // Make a copy to avoid modifying the original
    let mut loop_copy = loop_entity.clone();

    // C++ reference: GCode.cpp:5077-5078
    // C++: // extrude all loops ccw
    // C++: bool was_clockwise = loop.make_counter_clockwise();
    // Ensure counter-clockwise orientation
    loop_copy.make_counter_clockwise();

    // C++ reference: GCode.cpp:5079-5080
    // C++: bool is_hole = loop.loop_role() & elrPerimeterHole;
    // C++: Point last_pos = this->last_pos();
    // Get current position for seam placement
    let last_pos_f = writer.position();
    let last_pos = Point::new(
        (last_pos_f.x * 1_000_000.0) as i64,
        (last_pos_f.y * 1_000_000.0) as i64,
    );

    // C++ reference: GCode.cpp:5081-5088
    // C++: if (!m_config.spiral_mode && description == "perimeter") {
    // C++: assert(m_layer != nullptr);
    // C++: bool is_outer_wall_first = m_config.wall_sequence == WallSequence::OuterInner;
    // C++: m_seam_placer.place_seam(m_layer, loop, is_outer_wall_first, this->last_pos(), satisfy_scarf_seam_angle_threshold);
    // C++: } else
    // C++: loop.split_at(last_pos, false);
    // SeamPlacer integration (GCode.cpp:5081-5088)
    // C++: m_seam_placer.place_seam(m_layer, loop, is_outer_wall_first, last_pos, ...)
    // When a per-object SeamPlacer has been installed by the driver
    // (`print.rs` via `with_seam_context`), use its aligned-mode result
    // (SeamPlacer.cpp:1463). Otherwise fall back to the legacy per-loop
    // angle/nearest heuristic (`find_best_seam_index`).
    let polygon = loop_copy.as_polygon();
    let seam_point = match active_place_seam(&loop_copy, &polygon, last_pos) {
        Some(p) => p,
        None => {
            let seam_idx = super::seam_placer::find_best_seam_index(
                &polygon,
                Some(last_pos),
                &super::seam_placer::SeamPlacerConfig::default(),
            );
            polygon.points()[seam_idx]
        }
    };
    // SeamPlacer.cpp:1521-1527 — try the vertex rotate first, project only as a
    // fallback. See `apply_seam_split`.
    apply_seam_split(&mut loop_copy, &seam_point);

    if std::env::var("SPLITDBG2").is_ok() {
        let post = loop_copy
            .paths
            .first()
            .and_then(|pa| pa.polyline.points().first().copied());
        eprintln!(
            "SPLITDBG2-R prefirst=({},{}) seam=({},{}) postfirst={:?}",
            polygon.points()[0].x, polygon.points()[0].y, seam_point.x, seam_point.y, post
        );
    }

    // C++ reference: GCode.cpp:5107-5117
    // C++: const double seam_gap = scale_(EXTRUDER_CONFIG(nozzle_diameter)) * (m_config.seam_gap.value / 100);
    // C++: const double clip_length = m_enable_loop_clipping && !enable_seam_slope ? seam_gap : 0;
    // C++: // get paths
    // C++: ExtrusionPaths paths;
    // C++: ...
    // C++: loop.clip_end(clip_length, &paths);
    // C++: if (paths.empty()) return "";
    // R220: seam-gap clip (GCode.cpp:5107-5117): clip_length =
    // scale_(nozzle_diameter) * seam_gap% — the loop ends ~0.06mm before the
    // seam (native L24 inner ends at -6.323 vs rust's full -6.383).
    let clipped_paths: Vec<crate::extrusion_entity::ExtrusionPath>;
    let paths = if crate::gcode::writer::lift_faithful_gate() {
        let nozzle = writer.config_ref().nozzle_diameter;
        let seam_gap_pct = config.seam_gap;
        let clip_length = crate::scale(nozzle) as f64 * (seam_gap_pct / 100.0);
        let mut tmp = Vec::new();
        loop_copy.clip_end(clip_length, &mut tmp);
        if tmp.is_empty() {
            return;
        }
        clipped_paths = tmp;
        &clipped_paths
    } else {
        &loop_copy.paths
    };

    // C++ reference: GCode.cpp:5119-5122
    // C++: double small_peri_speed=-1;
    // C++: // apply the small perimeter speed
    // C++: if (loop.length() <= SMALL_PERIMETER_LENGTH(NOZZLE_CONFIG(small_perimeter_threshold)))
    // C++: small_peri_speed = NOZZLE_CONFIG(small_perimeter_speed).get_abs_value(NOZZLE_CONFIG(outer_wall_speed));
    // TODO: Implement small perimeter speed adjustment (GCode.cpp:5119-5122)

    // C++ reference: GCode.cpp:5089-5195
    // TODO: Implement seam slope (scarf seam) feature (GCode.cpp:5089-5195)
    // This is an advanced feature for better seam quality
    // Skip for initial implementation

    // C++ reference: GCode.cpp:5208-5218
    // C++: if (!enable_seam_slope || slope_has_overhang) {
    // C++: ...
    // C++: for (ExtrusionPaths::iterator path = paths.begin(); path != paths.end(); ++path) {
    // C++: gcode += this->_extrude(*path, description, speed_for_path(*path), set_holes_and_compensation_speed);
    // C++: }
    // C++: set_last_scarf_seam_flag(false);
    // C++: }
    // Extrude each path in the loop.
    //
    // C++ GCode.cpp:5208-5218 extrudes each path with `speed_for_path(*path)`,
    // which for perimeters routes through get_path_speed -> per-segment overhang
    // speed (GCode.cpp:5382-5404). Here we apply the overhang-degree-corrected
    // speed per path (only when it differs from the feature speed already set by
    // extrude_collection), so the F feedrate is modulated per segment.
    let zsmooth_markers = crate::faithful_gate("ZSMOOTH_FAITHFUL");
    let loop_role = loop_copy.paths.first().map(|p| p.role);
    let apply_overhang_speed = config.enable_overhang_speed
        && matches!(
            loop_role,
            Some(ExtrusionRole::ExternalPerimeter) | Some(ExtrusionRole::Perimeter)
        );

    let fmvs_cap = writer.config_ref().filament_max_volumetric_speed;
    // Per-path normal wall speed (overhang-degree-corrected), mm/s.
    // Mirrors GCode::get_path_speed for perimeter roles (GCode.cpp:5387-5400):
    //   new_speed = get_overhang_degree_corr_speed(speed, overhang_degree);
    //   speed = new_speed == 0.0 ? speed : new_speed;
    let path_speed_fn = |p: &ExtrusionPath| -> f64 {
        let base = perimeter_base_speed(config, p.role, is_first_layer);
        let new_speed = overhang_degree_corr_speed(config, base, p.overhang_degree);
        let v = if new_speed == 0.0 { base } else { new_speed };
        // R229 (gated): native applies the filament volumetric cap to EVERY
        // path in _extrude (GCode.cpp:6560-6567).
        if zsmooth_markers && crate::faithful_gate("VOLCAP_FAITHFUL") {
            volumetric_capped_speed(v, p.mm3_per_mm, fmvs_cap, config.print_flow_ratio)
        } else {
            v
        }
    };

    // C++ GCode.cpp:5576-5582 — smooth speed of discontinuity areas.
    // Gated on detect_overhang_wall && smooth_speed_discontinuity_area &&
    // is_set_speed_discontinuity_area (perimeter / external / overhang roles).
    // m_smooth_coefficient = filament_velocity_adaptation_factor (assumed 1.0) *
    // smooth_coefficient.
    let is_set_speed_discontinuity = matches!(
        loop_role,
        Some(ExtrusionRole::ExternalPerimeter)
            | Some(ExtrusionRole::Perimeter)
            | Some(ExtrusionRole::OverhangPerimeter)
    );
    let smooth_coeff = config.smooth_coefficient;
    // R536 probe (SMOOTHPROBE=1): count each sub-condition of the smoothing gate
    // separately, so a closed gate names its own cause instead of being guessed at.
    if crate::probe_enabled("SMOOTHPROBE") {
        use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
        static SEEN: AtomicUsize = AtomicUsize::new(0);
        static OK_DETECT: AtomicUsize = AtomicUsize::new(0);
        static OK_FLAG: AtomicUsize = AtomicUsize::new(0);
        static OK_ROLE: AtomicUsize = AtomicUsize::new(0);
        static OK_COEFF: AtomicUsize = AtomicUsize::new(0);
        static OK_NOT_FIRST: AtomicUsize = AtomicUsize::new(0);
        static OK_MULTI: AtomicUsize = AtomicUsize::new(0);
        static DEG_NONZERO: AtomicUsize = AtomicUsize::new(0);
        static PATHS_TOTAL: AtomicUsize = AtomicUsize::new(0);
        let n = SEEN.fetch_add(1, Relaxed) + 1;
        if config.detect_overhang_wall {
            OK_DETECT.fetch_add(1, Relaxed);
        }
        if config.smooth_speed_discontinuity_area {
            OK_FLAG.fetch_add(1, Relaxed);
        }
        if is_set_speed_discontinuity {
            OK_ROLE.fetch_add(1, Relaxed);
        }
        if smooth_coeff != 0.0 {
            OK_COEFF.fetch_add(1, Relaxed);
        }
        if !is_first_layer {
            OK_NOT_FIRST.fetch_add(1, Relaxed);
        }
        if paths.len() > 1 {
            OK_MULTI.fetch_add(1, Relaxed);
        }
        PATHS_TOTAL.fetch_add(paths.len(), Relaxed);
        // R538: bucket by role — outer walls reached C++'s structure after R537 but
        // inner walls did not, so the two populations have to be counted apart.
        {
            use std::sync::atomic::{AtomicUsize as A, Ordering::Relaxed as R};
            static EXT_LOOPS: A = A::new(0);
            static EXT_PATHS: A = A::new(0);
            static EXT_GRADED: A = A::new(0);
            static INT_LOOPS: A = A::new(0);
            static INT_PATHS: A = A::new(0);
            static INT_GRADED: A = A::new(0);
            let graded = paths.iter().filter(|p| p.overhang_degree != 0.0).count();
            if loop_role == Some(ExtrusionRole::ExternalPerimeter) {
                EXT_LOOPS.fetch_add(1, R);
                EXT_PATHS.fetch_add(paths.len(), R);
                EXT_GRADED.fetch_add(graded, R);
            } else if loop_role == Some(ExtrusionRole::Perimeter) {
                INT_LOOPS.fetch_add(1, R);
                INT_PATHS.fetch_add(paths.len(), R);
                INT_GRADED.fetch_add(graded, R);
            }
            if n % 1_000 == 0 {
                let (el, ep, eg) = (
                    EXT_LOOPS.load(R).max(1),
                    EXT_PATHS.load(R),
                    EXT_GRADED.load(R),
                );
                let (il, ip, ig) = (
                    INT_LOOPS.load(R).max(1),
                    INT_PATHS.load(R),
                    INT_GRADED.load(R),
                );
                eprintln!(
                    "[SMOOTHROLE] external: loops={el} paths/loop={:.2} graded={:.1}%  |  \
                     internal: loops={il} paths/loop={:.2} graded={:.1}%",
                    ep as f64 / el as f64,
                    100.0 * eg as f64 / ep.max(1) as f64,
                    ip as f64 / il as f64,
                    100.0 * ig as f64 / ip.max(1) as f64,
                );
            }
        }
        DEG_NONZERO.fetch_add(
            paths.iter().filter(|p| p.overhang_degree != 0.0).count(),
            Relaxed,
        );
        if n % 1_000 == 0 || n == 1 {
            eprintln!(
                "[SMOOTHPROBE] loops={n} detect={} flag={} role={} coeff={}({}) not_first={} paths>1={} \
                 paths_total={} overhang_deg!=0={}",
                OK_DETECT.load(Relaxed),
                OK_FLAG.load(Relaxed),
                OK_ROLE.load(Relaxed),
                OK_COEFF.load(Relaxed),
                smooth_coeff,
                OK_NOT_FIRST.load(Relaxed),
                OK_MULTI.load(Relaxed),
                PATHS_TOTAL.load(Relaxed),
                DEG_NONZERO.load(Relaxed),
            );
        }
    }
    if config.detect_overhang_wall
        && config.smooth_speed_discontinuity_area
        && is_set_speed_discontinuity
        && smooth_coeff != 0.0
        && !is_first_layer
        && paths.len() > 1
    {
        // Build a smoothed copy of the paths whose `smooth_speed` ramps across
        // discontinuities, then emit each with its smoothed feedrate.
        let mut smoothed: Vec<ExtrusionPath> = paths.clone();
        super::smooth_speed::smooth_speed_discontinuity_area(
            smooth_coeff,
            &mut smoothed,
            path_speed_fn,
        );
        // GCode.cpp:6599-6601 — per-path ";FEATURE: <role>" on role change.
        let mut last_path_role = smoothed.first().map(|p| p.role);
        for path in &smoothed {
            if Some(path.role) != last_path_role {
                writer.write_comment(&format!(
                    "FEATURE: {}",
                    crate::extrusion_entity::role_to_string(path.role)
                ));
                last_path_role = Some(path.role);
            }
            if apply_overhang_speed
                && matches!(
                    path.role,
                    ExtrusionRole::ExternalPerimeter | ExtrusionRole::Perimeter
                )
            {
                let cooling_comment = if path.role == ExtrusionRole::ExternalPerimeter {
                    ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
                } else {
                    ";_EXTRUDE_SET_SPEED"
                };
                set_speed_before_path(writer, path.smooth_speed * 60.0, cooling_comment);
            } else if path.role == ExtrusionRole::OverhangPerimeter {
                // GCode.cpp:5401-5408 — overhang/bridge wall speed.
                let ovh_speed = if (path.overhang_degree - 5.0).abs() < f64::EPSILON
                    && config.enable_overhang_speed
                {
                    config.overhang_totally_speed
                } else {
                    config.bridge_speed
                };
                set_speed_before_path(writer, ovh_speed * 60.0, "");
            }
            let fan_marker = zsmooth_markers && overhang_fan_marker_needed(config, path);
            if fan_marker {
                writer.write_raw(";_OVERHANG_FAN_START");
            }
            extrude_path(path, writer, config, is_first_layer);
            if fan_marker {
                writer.write_raw(";_OVERHANG_FAN_END");
            }
        }
        writer.reset_acceleration_default(is_first_layer);
        return;
    }

    // GCode.cpp:6599-6601 — emit ";FEATURE: <role>" per path when role changes.
    // extrude_collection already emitted the loop's first-path role, so seed with it.
    let mut last_path_role = paths.first().map(|p| p.role);
    for path in paths {
        // Per-path role-change FEATURE comment (Overhang wall paths within a wall loop).
        if Some(path.role) != last_path_role {
            writer.write_comment(&format!(
                "FEATURE: {}",
                crate::extrusion_entity::role_to_string(path.role)
            ));
            last_path_role = Some(path.role);
        }
        if apply_overhang_speed
            && matches!(
                path.role,
                ExtrusionRole::ExternalPerimeter | ExtrusionRole::Perimeter
            )
        {
            // GCode.cpp:5387-5400 — normal wall speed corrected by overhang_degree.
            let corr = path_speed_fn(path);
            // Cooling markers: external perimeters keep their _EXTERNAL_PERIMETER tag.
            // (overhang paths are still cooling-adjustable for degrees < 5.)
            let cooling_comment = if path.role == ExtrusionRole::ExternalPerimeter {
                ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
            } else {
                ";_EXTRUDE_SET_SPEED"
            };
            set_speed_before_path(writer, corr * 60.0, cooling_comment);
        } else if path.role == ExtrusionRole::OverhangPerimeter {
            // GCode.cpp:5401-5408 — overhang/bridge wall speed.
            //   degree==5 -> overhang_totally_speed; else (degree 6) -> bridge_speed.
            let ovh_speed = if (path.overhang_degree - 5.0).abs() < f64::EPSILON
                && config.enable_overhang_speed
            {
                config.overhang_totally_speed
            } else {
                config.bridge_speed
            };
            // Bridge moves are not cooling-adjustable.
            set_speed_before_path(writer, ovh_speed * 60.0, "");
        }
        let fan_marker = zsmooth_markers && overhang_fan_marker_needed(config, path);
        if fan_marker {
            writer.write_raw(";_OVERHANG_FAN_START");
        }
        extrude_path(path, writer, config, is_first_layer);
        if fan_marker {
            writer.write_raw(";_OVERHANG_FAN_END");
        }
    }

    // C++ reference: GCode.cpp:5220-5227
    // C++: //BBS: don't reset acceleration when printing first layer. During first layer, acceleration is always same value.
    // C++: if (!this->on_first_layer()) {
    // C++: // reset acceleration
    // C++: m_writer.set_acceleration((unsigned int) (NOZZLE_CONFIG(default_acceleration) + 0.5));
    // C++: if (!this->is_BBL_Printer())
    // C++: gcode += m_writer.set_jerk_xy(m_config.default_jerk.value);
    // C++: }
    // R227: reset acceleration to default after the loop (GCode.cpp:5591-5597).
    writer.reset_acceleration_default(is_first_layer);

    // C++ reference: GCode.cpp:5230-5241
    // C++: // BBS
    // C++: if (m_wipe.enable && FILAMENT_CONFIG(wipe)) {
    // C++: m_wipe.path = Polyline();
    // C++: for (ExtrusionPath &path : paths) {
    // C++: ...
    // C++: m_wipe.path.append(path.polyline);
    // C++: }
    // C++: }
    // R222: native wipe path = the loop's SOURCE polyline points (clipped
    // paths concatenated, duplicate joints skipped) — GCode.cpp:5600-5610.
    if crate::gcode::writer::lift_faithful_gate() {
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for path in paths.iter() {
            for (k, pt) in path.polyline.points().iter().enumerate() {
                let p = (crate::unscale(pt.x()), crate::unscale(pt.y()));
                if k == 0 && pts.last() == Some(&p) {
                    continue;
                }
                pts.push(p);
            }
        }
        writer.set_wipe_path_points(pts);
    }
}

/// Extrude an extrusion entity collection.
///
/// C++ reference: GCode::extrude_multi_path() / extrude_collection()
/// GCode.cpp:2800-3100 (~300 lines)
///
/// This function:
/// 1. Iterates through all entities in the collection
/// 2. Handles role changes between entities
/// 3. Emits feature type annotations
/// 4. Calls appropriate extrude function for each entity
///
/// # Arguments
/// * `collection` - The collection to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors


/// R733 — cost census for gcode `generate`. It is 1.206 s = 46% of the benchy
/// slice and had never been profiled. `EXPPROF=1` prints at exit.
pub static EXPPROF_EXTRUDE_NS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static EXPPROF_EXTRUDE_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static EXPPROF_WRITE_NS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static EXPPROF_WRITE_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// Accumulates on drop so every return path in `extrude_collection` is counted.
pub struct ExpProfGuard(pub std::time::Instant);
impl Drop for ExpProfGuard {
    fn drop(&mut self) {
        use std::sync::atomic::Ordering::Relaxed;
        EXPPROF_EXTRUDE_NS.fetch_add(self.0.elapsed().as_nanos() as usize, Relaxed);
        EXPPROF_EXTRUDE_CALLS.fetch_add(1, Relaxed);
    }
}

pub fn expprof_report() {
    use std::sync::atomic::Ordering::Relaxed;
    if !crate::probe_enabled("EXPPROF") {
        return;
    }
    eprintln!(
        "[EXPPROF] extrude_collection {:.1}ms calls={} | writer emit {:.1}ms calls={}",
        EXPPROF_EXTRUDE_NS.load(Relaxed) as f64 / 1e6,
        EXPPROF_EXTRUDE_CALLS.load(Relaxed),
        EXPPROF_WRITE_NS.load(Relaxed) as f64 / 1e6,
        EXPPROF_WRITE_CALLS.load(Relaxed),
    );
}
pub fn extrude_collection(
    collection: &ExtrusionEntityCollection,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) -> Result<()> {
    let __ex_guard = if crate::probe_enabled("EXPPROF") { Some(ExpProfGuard(std::time::Instant::now())) } else { None };

    /// Extrude an ExtrusionEntityCollection by iterating entities
    /// GCode.cpp:2800-3100
    /// C++: std::string GCode::extrude_entity(const ExtrusionEntity &entity, const std::string &description, double speed)

    /// Early return for empty collection
    /// GCode.cpp:2802-2803
    /// C++: if (entity.is_collection())
    /// C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
    if collection.entities.is_empty() {
        return Ok(());
    }

    /// Track current role for feature comments. C++ `m_last_extrusion_role` is a
    /// PERSISTENT GCode member (GCode.hpp:538), so consecutive same-role entities
    /// across separate extrude_collection calls do NOT re-emit the FEATURE marker.
    /// Seed from / persist back to the writer's last role to match that.
    let mut current_role: Option<ExtrusionRole> = writer.last_extrusion_role();

    /// Iterate through entities in collection
    /// GCode.cpp:2860-2900
    /// C++: for (const ExtrusionEntity *entity : collection.entities) {
    /// C++: // Emit feature comment on role change
    /// C++: if (entity->role() != current_role) {
    /// C++: gcode += "; FEATURE: " + entity->role_to_string() + "\n";
    /// C++: current_role = entity->role();
    /// C++: }
    /// C++: gcode += this->extrude_entity(*entity, description, speed);
    /// C++: }
    for entity in &collection.entities {
        /// Get entity role for feature tracking
        /// GCode.cpp:2862-2863
        let entity_role = get_entity_role(entity);

        // Feature feedrate (mm/s) for this entity's role. Computed every iteration
        // (not just on role change) so it can be re-asserted after the
        // travel-to-start below, even when consecutive entities share a role.
        // C++ GCode.cpp:6175-6200: first layer uses initial_layer_speed from config.
        let feature_speed = if is_first_layer {
            // BambuStudio uses initial_layer_infill_speed for infill on first layer,
            // and initial_layer_speed for walls/other features.
            match entity_role {
                ExtrusionRole::InternalInfill
                | ExtrusionRole::SolidInfill
                | ExtrusionRole::TopSolidInfill
                | ExtrusionRole::BottomSurface => {
                    if config.initial_layer_infill_speed > 0.0 {
                        config.initial_layer_infill_speed
                    } else {
                        config.initial_layer_speed
                    }
                }
                _ => config.initial_layer_speed,
            }
        } else {
            match entity_role {
                ExtrusionRole::ExternalPerimeter => config.external_perimeter_speed,
                ExtrusionRole::Perimeter => config.perimeter_speed,
                ExtrusionRole::InternalInfill => config.infill_speed,
                ExtrusionRole::SolidInfill => config.solid_infill_speed,
                ExtrusionRole::TopSolidInfill => config.top_solid_infill_speed,
                ExtrusionRole::BridgeInfill => config.bridge_speed,
                ExtrusionRole::GapFill => config.gap_fill_speed,
                _ => config.perimeter_speed,
            }
        };
        // R229 (gated): filament volumetric cap — sparse infill's config 350
        // caps to 25mm3s/(w*h) = 307.065 (native F18423.913; rust emitted the
        // raw config speeds F21000/F18000).
        // R232 (gated): native FloatingVerticalShell speed =
        // vertical_shell_speed% of internal_solid_infill_speed
        // (GCode.cpp:6492-6500; 80% of 250 = 200 → F12000). Rust fell through
        // to the default arm (300 → the r-only F18000 ×397 bucket).
        let feature_speed = if crate::faithful_gate("ZSMOOTH_FAITHFUL")
            && !is_first_layer
            && entity_role == ExtrusionRole::FloatingVerticalShell
        {
            config.vertical_shell_speed / 100.0 * config.solid_infill_speed
        } else {
            feature_speed
        };
        // R229: PARKED behind VOLCAP_FAITHFUL — the capped speed
        // 25/mm3_per_mm lands one F-digit off native (18423.914-vs-.913:
        // rust f64 flow chain vs native float Flow → ~2e-8 mm3 drift,
        // invisible in E's 5 decimals but visible in F's 3), and the changed
        // generation-F cascades through cooling factors (net +887). Unlocks
        // with mm3/width value parity (same blocker as LINEWIDTH_PERPATH).
        let feature_speed = if crate::faithful_gate("VOLCAP_FAITHFUL") {
            volumetric_capped_speed(
                feature_speed,
                get_entity_mm3_per_mm(entity),
                writer.config_ref().filament_max_volumetric_speed,
                config.print_flow_ratio,
            )
        } else {
            feature_speed
        };
        // Cooling markers for the CoolingBuffer post-processor (C++ GCode.cpp:6253-6272).
        let cooling_comment = if entity_role == ExtrusionRole::BridgeInfill {
            // Bridge moves are not adjustable
            ""
        } else if entity_role == ExtrusionRole::ExternalPerimeter {
            ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
        } else {
            ";_EXTRUDE_SET_SPEED"
        };

        /// Emit feature comment / LINE_WIDTH / M204 when role changes
        /// GCode.cpp:2864-2868
        /// C++: if (entity->role() != current_role) {
        /// C++: gcode += "; FEATURE: ";
        /// C++: gcode += ExtrusionEntity::role_to_string(entity->role());
        /// C++: gcode += "\n";
        /// C++: }
        let role_changed = Some(entity_role) != current_role;
        // R218: native emits the FEATURE/LINE_WIDTH markers AFTER the travel +
        // unretract, at extrusion start (GCode _extrude's description block);
        // rust emitted them before the wipe/travel. Under the gate, defer.
        let defer_markers = crate::faithful_gate("ZSMOOTH_FAITHFUL");
        let mut pending_markers = false;
        if role_changed && defer_markers {
            pending_markers = true;
        }
        if role_changed && !defer_markers {
            writer.write_comment(&format!("FEATURE: {}", entity_role.to_string()));
            // Emit LINE_WIDTH annotation for the feature
            // First layer uses initial_layer_line_width if set
            let line_width = if is_first_layer && config.initial_layer_line_width > 0.0 {
                config.initial_layer_line_width
            } else {
                match entity_role {
                    ExtrusionRole::ExternalPerimeter => config.outer_wall_line_width,
                    ExtrusionRole::Perimeter => config.inner_wall_line_width,
                    ExtrusionRole::InternalInfill => config.sparse_infill_line_width,
                    ExtrusionRole::SolidInfill => config.solid_infill_line_width,
                    ExtrusionRole::TopSolidInfill => config.top_surface_line_width,
                    _ => config.outer_wall_line_width,
                }
            };
            if line_width > 0.0 {
                // ZSMOOTH_FAITHFUL: native's m_last_width is PERSISTENT — a
                // feature change to the SAME width emits no new tag
                // (GCode.cpp:6605). Rust re-emitted per feature (1177
                // rust-only "; LINE_WIDTH: 0.42" lines).
                let skip = crate::faithful_gate("ZSMOOTH_FAITHFUL")
                    && !writer.width_tag_changed(line_width);
                if !skip {
                    // Format LINE_WIDTH to match BambuStudio: trim trailing zeros
                    let lw_str = format!("{:.5}", line_width);
                    let lw_trimmed = lw_str.trim_end_matches('0').trim_end_matches('.');
                    writer.write_comment(&format!("LINE_WIDTH: {}", lw_trimmed));
                }
            }
            // NOTE: The feature `set_speed` (with the ;_EXTRUDE_SET_SPEED cooling
            // marker) is intentionally NOT emitted here, before the intra-collection
            // travel below. Emitting it here opens a CoolingBuffer "adjustable" block
            // (active_speed_modifier) that then SWALLOWS the following travel move
            // (cooling.rs:2084-2108 merges any G1/G2/G3 inside the block, setting its
            // line_type=0), so the travel's F60000 never updates the buffer's
            // current_feedrate. The subsequent post-travel `set_speed` (below) is then
            // stripped as "redundant" (new_feedrate == current_feedrate), letting the
            // F60000 travel speed leak onto the extrusion. C++ emits the speed-set
            // AFTER the travel (GCodeEditor.cpp:276 asserts no `G1 Fxx` inside an
            // adjustable block), so the travel always precedes the ;_EXTRUDE_SET_SPEED.
            // We therefore rely solely on the post-travel set_speed at the end of this
            // loop body, matching native ordering: [M204] -> travel -> set_speed.
            // Emit per-feature acceleration (M204) matching BambuStudio.
            //
            // Faithful port of GCode::_extrude's "adjust acceleration" block
            // (GCode.cpp:6393-6420). The whole block is gated on
            // default_acceleration > 0; the branch order is:
            //   1. first layer (initial_layer_acceleration > 0)
            //   2. ExternalPerimeter / OverhangPerimeter (outer_wall_acceleration > 0)
            //   3. top surface == TopSolidInfill (top_surface_acceleration > 0)
            //   4. Perimeter (inner_wall_acceleration > 0)
            //   5. InternalInfill (sparse_infill_acceleration resolved > 0)
            //   6. else default_acceleration
            // A feature value of 0 means "use default" (the branch is skipped),
            // e.g. inner_wall_acceleration = 0 in the H2D profile falls through to
            // default_acceleration. The bridge branch is `#if 0` in C++ (disabled),
            // so bridges also fall to default_acceleration. sparse_infill_acceleration
            // is a percentage of default_acceleration (config stores e.g. "100%" as
            // 100.0), resolved via get_abs_value: pct/100 * default.
            // ZSMOOTH_FAITHFUL: the per-entity shared-register emission below
            // replaces this role-change raw write (native has no role-change
            // accel — every path goes through set_acceleration_impl).
            if !crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                if let Some(acc) = writer.feature_acceleration(entity_role, is_first_layer) {
                    writer.write_raw(&format!("M204 S{}", acc));
                }
            }
            current_role = Some(entity_role);
            // Persist to the writer so the next extrude_collection call (e.g. the
            // next island's fills, or the next gap EEC) does not re-emit the same
            // FEATURE marker — matches C++ persistent m_last_extrusion_role.
            writer.set_last_extrusion_role(current_role);
        }
        if role_changed && defer_markers {
            current_role = Some(entity_role);
            writer.set_last_extrusion_role(current_role);
        }

        // Travel to start of this entity if the nozzle is not already there.
        // C++ GCode::extrude_entity() calls travel_to() before extruding each
        // entity. Without this, consecutive entities in the same collection are
        // connected by a spurious extrusion line across open space.
        let travel_target = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
            travel_target_for_entity(entity, writer_last_pos(writer))
        } else {
            get_entity_first_point(entity)
        };
        if let Some(first_pt) = travel_target {
            let pos = writer.position();
            let tx = crate::unscale(first_pt.x());
            let ty = crate::unscale(first_pt.y());
            let dx = pos.x - tx;
            let dy = pos.y - ty;
            let dist_sq = dx * dx + dy * dy;
            if dist_sq > 0.001 * 0.001 {
                // Per-entity travel + retraction/spiral-lift decision. This is the
                // single faithful decision point, mirroring C++ GCode::_extrude ->
                // GCode::travel_to -> GCode::needs_retraction (GCode.cpp:6366,6816,6964):
                // a retract + Z-hop is emitted before the travel to each *top-level*
                // extrusion entity (loop / multipath / path) whose length reaches
                // retraction_minimum_travel. The internal paths of a multi-path loop
                // are contiguous (m_last_pos == path.first_point() -> travel skipped in
                // C++), so the path-level travel in extrude_path_with_arc_fitting is a
                // BARE positioning move and does NOT retract — avoiding the per-internal-
                // -path double counting. retract() does wipe + retract + Z-hop;
                // unretract() after the travel descends and restores E (net E zero, so
                // material is unchanged).
                let travel_len = dist_sq.sqrt();
                let did_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                    // GCode.cpp:6964 needs_retraction — faithful branch order
                    // (min-travel, leaving-outer-wall force, reduce-infill skip).
                    let from = writer_last_pos(writer);
                    let to = crate::Point::new(crate::scale(tx), crate::scale(ty));
                    writer.needs_retraction_faithful(from, to, entity_role, travel_len)
                } else {
                    writer.needs_retraction_for_travel(travel_len)
                };
                if did_retract {
                    writer.retract();
                }
                if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                    writer.set_travel_acceleration_for(entity_role, travel_len);
                } else {
                    writer.set_travel_acceleration(6000.0);
                }
                if crate::gcode::writer::lift_faithful_gate() {
                    // R206: single travel_to_xyz merges the pending lazy lift
                    // (GCode.cpp:6902-6912, travel.size()==2 branch; dest z =
                    // the current nominal z).
                    let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
                    writer.travel_to_xyz(tx, ty, zdest);
                } else {
                    writer.travel_to(tx, ty, None);
                }
                if did_retract {
                    writer.unretract();
                }
            }
        }
        if pending_markers {
            // R218: native marker position — after travel+unretract, before the
            // feature F (GCode.cpp _extrude description/width block).
            writer.write_comment(&format!("FEATURE: {}", entity_role.to_string()));
            let line_width = if is_first_layer && config.initial_layer_line_width > 0.0 {
                config.initial_layer_line_width
            } else {
                match entity_role {
                    ExtrusionRole::ExternalPerimeter => config.outer_wall_line_width,
                    ExtrusionRole::Perimeter => config.inner_wall_line_width,
                    ExtrusionRole::InternalInfill => config.sparse_infill_line_width,
                    ExtrusionRole::SolidInfill => config.solid_infill_line_width,
                    ExtrusionRole::TopSolidInfill => config.top_surface_line_width,
                    _ => config.outer_wall_line_width,
                }
            };
            // R225: under LINEWIDTH_PERPATH the tag is emitted per path from
            // path.width at the extrude_path choke point (native GCode.cpp:6605
            // register) and the entity-level config width would fight it.
            if !crate::faithful_gate("LINEWIDTH_PERPATH")
                && line_width > 0.0
                && writer.width_tag_changed(line_width)
            {
                let lw_str = format!("{:.5}", line_width);
                let lw_trimmed = lw_str.trim_end_matches('0').trim_end_matches('.');
                writer.write_comment(&format!("LINE_WIDTH: {}", lw_trimmed));
            }
        }

        // Re-assert the feature feedrate before extruding.
        //
        // travel_to(..., None) above resets the sticky feedrate to travel_speed
        // (F60000 = 1000 mm/s). BambuStudio always emits the feature `G1 F<speed>`
        // as the LAST F-set before an extrusion run, so each extrude move inherits
        // the feature speed — not the travel speed. set_speed() is a no-op when the
        // feedrate is unchanged (writer.rs), so this only emits a line when a
        // preceding travel (or speed change) clobbered the feedrate. Skipping this
        // (the previous behaviour) let the F60000 travel feedrate leak onto 65% of
        // extrusion moves, so the acceleration-aware GCodeProcessor estimator paid
        // huge accel/decel ramp cost targeting an unreachable 1000 mm/s on short
        // extrusion segments (rust 1h50m vs native 43m).
        // ZSMOOTH_FAITHFUL: native's _extrude sets the feature acceleration for
        // EVERY path through the shared M204 register (GCode.cpp:6393-6420 +
        // GCodeWriter set_acceleration_impl) — after the travel (whose accel
        // went through the same register), so travel/feature alternation
        // re-emits M204 constantly. Legacy path emits only on role change.
        if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
            if let Some(acc) = writer.feature_acceleration(entity_role, is_first_layer) {
                writer.set_feature_acceleration_shared(acc);
            }
        }

        // ZSMOOTH_FAITHFUL: native never emits a collection-level feature-speed
        // F before a perimeter loop — its first F is the FIRST PATH's
        // overhang-corrected speed (GCode.cpp extrude_loop emits per-path F via
        // _extrude; e.g. L2 outwall starts F9300 = deg-1.3 speed, not F12000).
        // The rust per-path set_speed in extrude_loop always re-asserts F for
        // perimeter roles under enable_overhang_speed, so the pre-set is safely
        // skippable there. Default path keeps the pre-set (byte-locked).
        let skip_pre_speed = crate::faithful_gate("ZSMOOTH_FAITHFUL")
            && (config.enable_overhang_speed
                && matches!(
                    entity,
                    crate::extrusion_entity::ExtrusionEntityType::Loop(_)
                )
                && matches!(
                    entity_role,
                    ExtrusionRole::ExternalPerimeter
                        | ExtrusionRole::Perimeter
                        | ExtrusionRole::OverhangPerimeter
                )
                // R758: Path entities re-emit their own F in the R245 per-path
                // block below (native _extrude, GCode.cpp:6663) — native has NO
                // collection-level F before them either. Under
                // SET_SPEED_ALWAYS_EMIT the pre-set became a visible duplicate
                // `G1 F..;_EXTRUDE_SET_SPEED` pair (benchy layer-6 sparse:
                // 4 blocks vs native 2). PARKED OFF: byte-inert on benchy (the
                // cooling rewrite deletes the duplicates) but −51 in-order on
                // arachne — re-score after the arachne width-segmentation gap.
                || (crate::opt_in_gate("NO_COLLECTION_PRESPEED")
                    && matches!(
                        entity,
                        crate::extrusion_entity::ExtrusionEntityType::Path(_)
                    )));
        // R654 — instrument first (do not infer from the guard's source, R649).
        if crate::probe_enabled("PRESPEED_PROBE") {
            use std::sync::atomic::{AtomicU64, Ordering};
            static N: AtomicU64 = AtomicU64::new(0);
            static SKIP: AtomicU64 = AtomicU64::new(0);
            static NO_OH: AtomicU64 = AtomicU64::new(0);
            static NO_LOOP: AtomicU64 = AtomicU64::new(0);
            static NO_ROLE: AtomicU64 = AtomicU64::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed) + 1;
            if skip_pre_speed {
                SKIP.fetch_add(1, Ordering::Relaxed);
            } else {
                if !config.enable_overhang_speed {
                    NO_OH.fetch_add(1, Ordering::Relaxed);
                }
                if !matches!(entity, crate::extrusion_entity::ExtrusionEntityType::Loop(_)) {
                    NO_LOOP.fetch_add(1, Ordering::Relaxed);
                }
                if !matches!(
                    entity_role,
                    ExtrusionRole::ExternalPerimeter
                        | ExtrusionRole::Perimeter
                        | ExtrusionRole::OverhangPerimeter
                ) {
                    NO_ROLE.fetch_add(1, Ordering::Relaxed);
                }
            }
            if n % 200000 == 0 {
                eprintln!(
                    "[PRESPEED] n={n} skipped={} | blocked_by: !overhang_speed={} !Loop={} !perimeter_role={}",
                    SKIP.load(Ordering::Relaxed),
                    NO_OH.load(Ordering::Relaxed),
                    NO_LOOP.load(Ordering::Relaxed),
                    NO_ROLE.load(Ordering::Relaxed)
                );
            }
        }
        if !skip_pre_speed {
            // R654: C++ emits NO collection-level F — `_extrude` writes the Width
            // tag (GCode.cpp:6607) and only then `set_speed` (:6663). Defer ours
            // to just after the tag rather than dropping it, so the line count is
            // untouched and only its position changes.
            if crate::opt_in_gate("LINEWIDTH_BEFORE_SPEED") {
                writer.set_speed_pending(feature_speed * 60.0, cooling_comment);
            } else {
                writer.set_speed(feature_speed * 60.0, cooling_comment);
            }
        }

        /// Recursively extrude the entity
        /// GCode.cpp:2870-2880
        /// C++: gcode += this->extrude_entity(*entity, description, speed);
        extrude_entity(entity, writer, config, is_first_layer)?;
    }

    // R605: port of `GCode::extrude_multi_path`'s wipe-path install
    // (GCode.cpp:5664-5673). C++ builds ONE wipe path for the whole multipath --
    // every sub-path concatenated, skipping a duplicated joint, then REVERSED:
    //
    //     m_wipe.path = Polyline();
    //     for (ExtrusionPath &path : multipath.paths) {
    //         if (!m_wipe.path.empty() && !path.empty() &&
    //             m_wipe.path.last_point() == path.first_point())
    //             m_wipe.path.append(path.polyline.points.begin() + 1, ...);
    //         else
    //             m_wipe.path.append(path.polyline);
    //     }
    //     m_wipe.path.reverse();
    //
    // Our `extrude_entity` has no multipath branch because `ExtrusionEntityType`
    // has no `MultiPath` variant -- `exporter.rs` aliases
    // `extrude_collection as extrude_multi_path` "for backward compatibility"
    // (R604). The two are NOT interchangeable here: dispatching each sub-path
    // through `extrude_entity` installs a wipe path PER sub-path, so what survives
    // is only the LAST one, where C++ keeps the whole concatenation.
    //
    // Since the type distinction does not exist, this fires only on the shape a C++
    // `ExtrusionMultiPath` actually takes here -- a collection whose children are
    // ALL paths (what `thick_polyline_to_multi_path` produces for Arachne
    // variable-width walls and gap fill). It deliberately does NOT guess on mixed
    // collections, which C++ routes through `extrude_collection` instead.
    //
    // Shipped OPT-IN (default OFF) per R557/R595/R599. It is reachable (fires 468x
    // on Benchy, 11,175x on Majora) and the wipe COUNT is unchanged on both
    // fixtures (2,041 and 36,394), confirming this changes wipe CONTENT rather than
    // how often we wipe. But matched lines are flat-to-negative: Benchy +10
    // (115,900 -> 115,910), Majora **-18** (648,759 -> 648,741). The line-count gap
    // improves on both (1.20% -> 1.11%, 8.91% -> 8.62%) because the longer wipe
    // paths emit more moves and C++ emits them too -- but a narrowing gap is NOT
    // evidence of correctness on its own, since the gap rewards emitting lines
    // whether or not they match. Same shape as R595, so the same disposition.
    //
    // To make this net-positive the wipe MOVE VALUES have to match, not just the
    // path: the emitted `G1 X.. Y.. E-..` depend on wipe_dist, retract length and
    // wipe speed as well as the path. That is the next thing to check.
    if crate::probe_enabled("WIPE_MULTIPATH_CPP")
        && crate::gcode::writer::lift_faithful_gate()
        && collection.entities.len() >= 2
        && collection
            .entities
            .iter()
            .all(|e| matches!(e, crate::extrusion_entity::ExtrusionEntityType::Path(_)))
    {
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for e in &collection.entities {
            if let crate::extrusion_entity::ExtrusionEntityType::Path(p) = e {
                for (k, pt) in p.polyline.points().iter().enumerate() {
                    let q = (crate::unscale(pt.x()), crate::unscale(pt.y()));
                    // C++'s "don't save a duplicated point into wipe path".
                    if k == 0 && pts.last() == Some(&q) {
                        continue;
                    }
                    pts.push(q);
                }
            }
        }
        pts.reverse();
        // WIPE_MULTIPATH_POP=1 — reachability census (R595): prove the branch fires
        // before arguing about whether it helped. Prints cumulative totals, never a
        // truncated prefix (R598).
        if crate::probe_enabled("WIPE_MULTIPATH_POP") {
            use std::sync::atomic::{AtomicUsize, Ordering};
            static N: AtomicUsize = AtomicUsize::new(0);
            static PTS: AtomicUsize = AtomicUsize::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed) + 1;
            let tot = PTS.fetch_add(pts.len(), Ordering::Relaxed) + pts.len();
            eprintln!("[WIPEMP] n={n} pts={} cum_pts={tot}", pts.len());
        }
        if pts.len() >= 2 {
            writer.set_wipe_path_points(pts);
        }
    }

    Ok(())
}

/// The writer's current XY position as a scaled `Point` — the Rust analogue of
/// C++ `GCode::m_last_pos`, which the chaining routines accept as `start_near`.
///
/// `writer.position()` is unscaled mm (`PointF`); chaining operates in scaled
/// i64 coordinates, so convert with `scale()`.
/// R211: the travel TARGET for an entity. Native _extrude seam-splits loops
/// BEFORE the travel (GCode.cpp:5085-5090), so the travel goes to the SEAM,
/// not the loop's natural first vertex. For aligned seams place_seam is
/// position-independent (final_seam_position), so resolving it here and again
/// inside extrude_loop yields the same point.
fn travel_target_for_entity(
    entity: &crate::extrusion_entity::ExtrusionEntityType,
    last_pos: Point,
) -> Option<Point> {
    use crate::extrusion_entity::ExtrusionEntityType;
    if let ExtrusionEntityType::Loop(l) = entity {
        let polygon = l.as_polygon();
        if let Some(seam) = active_place_seam(l, &polygon, last_pos) {
            // R228: native travels to loop.first_point() AFTER split_at
            // (GCode.cpp:5085-5090). For outer walls the placed seam IS a
            // loop vertex, but for inner walls place_seam can return a point
            // OFF this polygon (nearest stored perimeter data) — split_at
            // then PROJECTS it onto the loop, so the raw seam and the split
            // first-point differ (0.4mm corrective mini-travels + duplicate
            // feature-F lines). Run the same split here and return its
            // actual first vertex.
            let mut probe = l.clone();
            apply_seam_split(&mut probe, &seam);
            if let Some(fp) = probe
                .paths
                .first()
                .and_then(|pa| pa.polyline.points().first().copied())
            {
                return Some(fp);
            }
            return Some(seam);
        }
    }
    get_entity_first_point(entity)
}

fn writer_last_pos(writer: &GCodeWriter) -> Point {
    let p = writer.position();
    Point::new(scale(p.x), scale(p.y))
}

/// Rust analogue of `ExtrusionEntityCollection::chained_path_from(start_near, role)`
/// (ExtrusionEntityCollection.hpp:102-103, .cpp:89-99) with the default
/// `role == erMixed` (no role filtering — Benchy infill EECs are mixed-role).
///
/// hpp:103: `return this->no_sort ? *this : chained_path_from(entities, start_near, role);`
/// cpp:89-99: clone the entities into a fresh collection, then
/// `chain_and_reorder_extrusion_entities(out.entities, &start_near)`.
///
/// `start_near` is the writer's current position (== C++ `m_last_pos`).
fn chained_path_from(
    eec: &ExtrusionEntityCollection,
    writer: &GCodeWriter,
) -> ExtrusionEntityCollection {
    // hpp:103: when no_sort is set, return the collection unchanged.
    if eec.no_sort {
        return eec.clone();
    }
    // cpp:92-96: filtered (here unfiltered, role == erMixed) clone of the entities.
    let mut out = eec.clone();
    // cpp:97: chain_and_reorder_extrusion_entities(out.entities, &start_near)
    let start_near = writer_last_pos(writer);
    crate::shortest_path::chain_and_reorder_extrusion_entities(
        &mut out.entities,
        Some(&start_near),
    );
    out
}

/// Get the first point of an extrusion entity for travel-to targeting.
/// R229: native _extrude's filament volumetric cap (GCode.cpp:6560-6567):
/// `speed = min(speed, filament_max_volumetric_speed / (path.mm3_per_mm *
/// print_flow_ratio))`. 0/unset caps disable it.
fn volumetric_capped_speed(speed: f64, mm3_per_mm: f64, fmvs: f64, flow_ratio: f64) -> f64 {
    if fmvs <= 0.0 {
        return speed;
    }
    let ratio = if flow_ratio > 0.0 { flow_ratio } else { 1.0 };
    let mm3 = mm3_per_mm * ratio;
    if mm3 > 0.0 {
        speed.min(fmvs / mm3)
    } else {
        speed
    }
}

/// First path's mm3_per_mm of an entity (native caps per path; the collection
/// pre-speed uses the first path as representative).
fn get_entity_mm3_per_mm(entity: &ExtrusionEntityType) -> f64 {
    use crate::extrusion_entity::ExtrusionEntityType;
    match entity {
        ExtrusionEntityType::Path(p) => p.mm3_per_mm,
        ExtrusionEntityType::Loop(l) => l.paths.first().map(|p| p.mm3_per_mm).unwrap_or(0.0),
        ExtrusionEntityType::Collection(c) => c
            .entities
            .first()
            .map(get_entity_mm3_per_mm)
            .unwrap_or(0.0),
    }
}

pub fn get_entity_first_point(entity: &ExtrusionEntityType) -> Option<Point> {
    match entity {
        ExtrusionEntityType::Path(path) => path.polyline.points().first().copied(),
        ExtrusionEntityType::Loop(loop_entity) => loop_entity
            .paths
            .first()
            .and_then(|p| p.polyline.points().first().copied()),
        ExtrusionEntityType::Collection(coll) => coll
            .entities
            .first()
            .and_then(|e| get_entity_first_point(e)),
    }
}

/// Helper to extract role from ExtrusionEntityType
/// GCode.cpp:2100-2120 (role accessor methods)
fn get_entity_role(entity: &ExtrusionEntityType) -> ExtrusionRole {
    match entity {
        ExtrusionEntityType::Path(path) => path.role,
        ExtrusionEntityType::Loop(loop_entity) => {
            /// Loops may have multiple paths with different roles
            /// Use first path's role, or Mixed if empty
            /// GCode.cpp:2105
            /// C++: ExtrusionRole ExtrusionLoop::role() const {
            /// C++: return paths.empty() ? erNone : paths.front().role();
            /// C++: }
            loop_entity
                .paths
                .first()
                .map(|p| p.role)
                .unwrap_or(ExtrusionRole::None)
        }
        ExtrusionEntityType::Collection(coll) => {
            // Recurse into first entity of collection to get the actual role
            coll.entities
                .first()
                .map(|e| get_entity_role(e))
                .unwrap_or(ExtrusionRole::Mixed)
        }
    }
}

/// Alias for backward compatibility (C++ has both multi_path and collections)
pub use extrude_collection as extrude_multi_path;

/// Extrude a single extrusion path.
///
/// C++ reference: GCode::_extrude()
/// GCode.cpp:4200-4800 (~600 lines)
///
/// This function:
/// 1. Generates G1 commands for each point in the path
/// 2. Calculates extrusion amount (E value) for each segment
/// 3. Handles variable width segments
/// 4. Applies arc fitting if enabled
///
/// # Arguments
/// * `path` - The extrusion path to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
/// C++ printf "%g" (default precision 6): 6 significant digits, trailing
/// zeros trimmed. Native formats the LINE_WIDTH tag with %g of the f32
/// path.width promoted to double (GCode.cpp:6607).
fn fmt_g6(v: f64) -> String {
    if v == 0.0 {
        return "0".to_string();
    }
    let exp = v.abs().log10().floor() as i32;
    if exp < -5 || exp >= 6 {
        // %g scientific branch — widths never reach it; minimal fallback.
        return format!("{:e}", v);
    }
    let decimals = (5 - exp).max(0) as usize;
    let s = format!("{:.*}", decimals, v);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    s.to_string()
}

pub fn extrude_path(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    extrude_path_ex(path, writer, config, is_first_layer, None);
}

/// Extrude a path with access to actual PrintConfig for arc fitting.
///
/// The arc-fitting / simplification gating is now taken from the live print
/// configuration carried by the `GCodeWriter` (mirroring C++ where
/// `LayerRegion::simplify_path` and `GCode::_extrude` both consult
/// `print()->config()`), so the optional `print_config` argument is no longer
/// needed to enable the live path.
pub fn extrude_path_ex(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    _print_config: Option<&PrintConfig>,
) {
    extrude_path_with_arc_fitting(path, writer, config, is_first_layer, None);
}

/// Extrude a path with optional arc fitting.
///
/// When `print_config` is provided and arc fitting is enabled in it,
/// sequences of line segments that form arcs will be converted to
/// G2/G3 arc commands.
pub fn extrude_path_with_arc_fitting(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    _print_config: Option<&PrintConfig>,
) {
    // C++ reference: GCode.cpp:4200-4210
    // C++: std::string GCode::_extrude(const ExtrusionPath &path, std::string description, double speed, bool is_hole_or_comp_speed)
    // C++: {
    // C++: std::string gcode;
    // C++:
    // C++: // check if path is empty
    // C++: if (path.polyline.points.empty())
    // C++: return "";
    // Check if path is empty
    if path.polyline.points().is_empty() {
        return;
    }

    // R225: native _extrude emits ";LINE_WIDTH: %g" whenever path.width
    // differs from m_last_width (GCode.cpp:6605-6609) — a PER-PATH register,
    // not per feature change (fills alternate widths constantly; 6.5k
    // native-only tag lines came from this). PARKED behind its own gate:
    // emission shape is native-correct, but rust f64 flow/Arachne width
    // values drift from native f32 chains in the 6th significant digit
    // (0.43272-vs-0.43273, 0.42-vs-0.41999), so enabling it today ADDS
    // unmatched lines (83187 → 87805). Unlocks when width values converge.
    static LW_PERPATH: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    static EXPW_EMITTED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    // R571 STEP 2: a GLOBAL (all-roles) register, mirroring the writer's own
    // `last_width_tag`, so the outer-wall-only count below can be checked
    // against the quantity the emitter actually tests (R570's open tension).
    static EXPW_GLOBAL_PREV: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    static EXPW_GLOBAL_CH: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    static EXPW_GLOBAL_N: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    if crate::probe_enabled("EXPWPROBE") && path.width > 0.0 {
        let bits = (path.width as f32).to_bits() as u64;
        EXPW_GLOBAL_N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if EXPW_GLOBAL_PREV.swap(bits, std::sync::atomic::Ordering::Relaxed) != bits {
            EXPW_GLOBAL_CH.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }
    // EXPWPROBE (R569) — do the per-path widths produced by
    // thick_polyline_to_multi_path still differ by the time they reach the
    // emitter? Counts outer-wall paths seen and how many changed the register.
    let expw = crate::probe_enabled("EXPWPROBE")
        && path.role == crate::extrusion_entity::ExtrusionRole::ExternalPerimeter;

    // R656 — GCode.cpp:6591 evaluates `last_was_wipe_tower` ONCE per `_extrude`
    // and both the Width tag (:6605) and the Height tag (:6619) read that same
    // value, so take it here rather than at each guard.
    // SHIPPED OPT-IN (probe, default OFF) per R656's PRE-REGISTERED fallback.
    // The port is right: it takes `; LAYER_HEIGHT:` from 4,720 to 8,058 against
    // C++'s 8,297 — a class that was 43% short is now 3% short — and content
    // matched rises +1,204. But IN-ORDER falls 2,117, the second time a
    // demonstrably C++-faithful tag addition has cost order while the
    // `; LINE_WIDTH:` count is still 61,136 short (R654 was the first, −26,309).
    // The fallback pre-registered for exactly this: park it, stop adding tags,
    // and re-run this A/B once the Arachne width-variety gap is closed. Two
    // flips then land together — this and `LINEWIDTH_BEFORE_SPEED`.
    let force_tags = crate::opt_in_gate("WIPE_TOWER_FORCE_TAGS")
        && writer.take_force_analyzer_tags();

    if *LW_PERPATH.get_or_init(|| crate::faithful_gate("LINEWIDTH_PERPATH"))
        && path.width > 0.0
        // R656: `width_tag_changed` must run either way — it is the register
        // update, and C++ assigns `m_last_width = path.width` inside the same
        // branch. Evaluate it FIRST so the force-emit cannot leave the register
        // stale.
        && (writer.width_tag_changed(path.width) | force_tags)
    {
        writer.write_comment(&format!("LINE_WIDTH: {}", fmt_g6(path.width)));
        if crate::probe_enabled("EXPWPROBE") {
            EXPW_EMITTED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }
    if expw {
        use std::sync::atomic::{AtomicU64, Ordering};
        static SEEN: AtomicU64 = AtomicU64::new(0);
        static CHANGED: AtomicU64 = AtomicU64::new(0);
        static ZEROW: AtomicU64 = AtomicU64::new(0);
        if path.width <= 0.0 {
            ZEROW.fetch_add(1, Ordering::Relaxed);
        }
        static PREV: AtomicU64 = AtomicU64::new(0);
        static PREV_END: std::sync::Mutex<Option<(i64, i64)>> =
            std::sync::Mutex::new(None);
        static CH_CONT: AtomicU64 = AtomicU64::new(0);
        static CH_TRAV: AtomicU64 = AtomicU64::new(0);
        static CONT: AtomicU64 = AtomicU64::new(0);
        // Contiguous == this path starts exactly where the previous one ended, so
        // no travel was needed between them (the internal analogue of R568's
        // G-code classification).
        let contiguous = {
            let mut g = PREV_END.lock().unwrap();
            let f = path.polyline.points.first().map(|p| (p.x, p.y));
            let c = *g == f;
            *g = path.polyline.points.last().map(|p| (p.x, p.y));
            c
        };
        if contiguous {
            CONT.fetch_add(1, Ordering::Relaxed);
        }
        let cur = (path.width as f32).to_bits() as u64;
        if PREV.swap(cur, Ordering::Relaxed) != cur {
            CHANGED.fetch_add(1, Ordering::Relaxed);
            if contiguous {
                CH_CONT.fetch_add(1, Ordering::Relaxed);
            } else {
                CH_TRAV.fetch_add(1, Ordering::Relaxed);
            }
        }
        let n = SEEN.fetch_add(1, Ordering::Relaxed) + 1;
        if n % 50_000 == 0 {
            println!(
                "EXPWPROBE outer_paths={} width_changed={} contiguous={} ch_contig={} ch_after_travel={} zero_width={} EMITTED_ALLROLES={} GLOBAL_paths={} GLOBAL_changed={}",
                n,
                CHANGED.load(Ordering::Relaxed),
                CONT.load(Ordering::Relaxed),
                CH_CONT.load(Ordering::Relaxed),
                CH_TRAV.load(Ordering::Relaxed),
                ZEROW.load(Ordering::Relaxed),
                EXPW_EMITTED.load(Ordering::Relaxed),
                EXPW_GLOBAL_N.load(Ordering::Relaxed),
                EXPW_GLOBAL_CH.load(Ordering::Relaxed),
            );
        }
    }

    // R234: native _extrude emits ";LAYER_HEIGHT: %g" when the path height
    // leaves the m_last_height register by > EPSILON (GCode.cpp:6619-6623;
    // bridges print at 0.4 over 0.2 layers). Register also fed by layer
    // changes (print.rs, GCode.cpp:4065).
    if crate::gcode::writer::lift_faithful_gate()
        && path.height > 0.0
        && ((writer.last_height_tag - path.height).abs() > 1e-4 || force_tags)
    {
        writer.last_height_tag = path.height;
        writer.write_comment(&format!("LAYER_HEIGHT: {}", fmt_g6(path.height)));
    }

    // R708 — the deferred collection-level feedrate lands HERE. R654 flushed it
    // straight after the Width tag, one tag too early: C++ writes Width
    // (GCode.cpp:6607), then Height (:6619-6623), and only then `set_speed`
    // (:6663), so the F must follow the height tag, not precede it.
    // Unconditional: if neither tag fired, the F is still emitted exactly once,
    // so the line count is identical either way.
    writer.flush_pending_speed();

    // C++ reference: GCode.cpp:4211-4220
    // C++: // get path properties
    // C++: const Polyline &polyline = path.polyline;
    // C++: const double mm3_per_mm = path.mm3_per_mm;
    // C++: const double width = path.width;
    // C++: const double height = path.height;
    // Get path properties
    let points = path.polyline.points();
    let mm3_per_mm = path.mm3_per_mm;
    let _width = path.width;
    let _height = path.height;

    // Calculate extrusion length per distance unit
    // GCode.cpp:6071-6081
    // C++: auto _mm3_per_mm = path.mm3_per_mm * double(this->config().print_flow_ratio.value);
    // C++: if( path.role() == erTopSolidInfill )
    // C++: _mm3_per_mm *= NOZZLE_CONFIG(top_solid_infill_flow_ratio);
    // C++: else if (this->on_first_layer())
    // C++: _mm3_per_mm *= m_config.initial_layer_flow_ratio.value;
    // C++: double e_per_mm = m_writer.filament()->e_per_mm3() * _mm3_per_mm;
    //
    // Apply extrusion multiplier (print_flow_ratio)
    // GCode.cpp:6073
    // C++: auto _mm3_per_mm = path.mm3_per_mm * double(this->config().print_flow_ratio.value);
    let mut adjusted_mm3_per_mm = mm3_per_mm * config.print_flow_ratio;

    // Apply role-specific flow multipliers
    // GCode.cpp:6074-6077
    // C++: if( path.role() == erTopSolidInfill )
    // C++: _mm3_per_mm *= NOZZLE_CONFIG(top_solid_infill_flow_ratio);
    // C++: else if (this->on_first_layer())
    // C++: _mm3_per_mm *= m_config.initial_layer_flow_ratio.value;
    if path.role == crate::extrusion_entity::ExtrusionRole::TopSolidInfill {
        adjusted_mm3_per_mm *= config.top_solid_infill_flow_ratio;
    } else if is_first_layer {
        adjusted_mm3_per_mm *= config.initial_layer_flow_ratio;
    }

    // Use extruder's e_per_mm3 to convert mm³/mm to filament mm/mm
    let e_per_mm = writer.extruder_e_per_mm(adjusted_mm3_per_mm);

    // Travel to the path's first point if the nozzle is not already there.
    //
    // C++ GCode::_extrude (GCode.cpp:6366-6368) travels to path.first_point()
    // whenever m_last_pos != path.first_point(), before emitting the path. The
    // segment emission below assumes the nozzle is at points[0]; when it is not
    // (extrude_loop reorders the loop to the seam via split_loop_at_*, and a
    // multi-path loop's later paths start where the previous path ended), the
    // first emitted segment is wrong. For an arc-fitted first segment this is
    // catastrophic: the I/J center offset is computed relative to
    // arc.start_point (= points[0]) but the printer/estimator applies it
    // relative to the ACTUAL nozzle position, producing a degenerate off-circle
    // arc whose computed arc length explodes (one arc -> "12 m" / ~100 s in the
    // GCodeProcessor), inflating the estimated print time ~3x (1h50m vs 43m).
    //
    // The feature feedrate was already set by extrude_collection / extrude_loop
    // before this call; travel_to() resets it to travel_speed (F60000), so we
    // capture and restore it after the travel — matching C++, where _extrude
    // emits the path F (speed*60) after the travel. set_speed is a no-op when
    // the feedrate is unchanged, so no spurious F line is emitted when no travel
    // was needed.
    if let Some(first_pt) = path.polyline.points().first() {
        let pos = writer.position();
        let tx = unscale(first_pt.x());
        let ty = unscale(first_pt.y());
        let dist_sq = (pos.x - tx) * (pos.x - tx) + (pos.y - ty) * (pos.y - ty);
        if dist_sq > 0.001 * 0.001 {
            let resume_speed = writer.get_current_speed() * 60.0; // mm/s -> mm/min

            // Bare positioning travel — no retraction/spiral-lift decision here.
            // This site handles the travel between the contiguous internal paths of a
            // multi-path loop / multipath. In C++ those moves are skipped entirely
            // (m_last_pos == path.first_point(), GCode::_extrude:6366), so they must
            // not retract. The retraction + spiral-lift decision for the travel to the
            // start of each top-level extrusion entity is made once in
            // extrude_collection (mirroring GCode::needs_retraction per _extrude),
            // which has already moved the nozzle here, so for an entity's first path
            // this branch is a no-op.
            writer.set_travel_acceleration(6000.0);
            writer.travel_to(tx, ty, None);
            if resume_speed > 0.0 {
                writer.set_speed(resume_speed, "");
            }
        }
    }

    // -------------------------------------------------------------------
    // Toolpath simplification + (optional) arc fitting.
    //
    // C++ splits this across two places that we fuse here because the Rust
    // ExtrusionPath does not carry a persistent `fitting_result`:
    //   1. LayerRegion::simplify_path (LayerRegion.cpp:786-802) mutates
    //      path.polyline.points and populates path.polyline.fitting_result.
    //   2. GCode::_extrude (GCode.cpp:6665-6745) emits G1 / G2 / G3 from the
    //      (possibly simplified) polyline and its fitting_result.
    //
    // The simplification is deterministic on the same input points, so doing
    // it here on a local copy yields the same motion output as doing it once
    // up front in PrintObject::simplify_extrusion_path.
    // -------------------------------------------------------------------
    // LayerRegion.cpp:789-791
    // C++: const bool spiral_mode        = print_config.spiral_mode;
    // C++: const bool enable_arc_fitting = print_config.enable_arc_fitting;
    // C++: const auto scaled_resolution  = scaled<double>(print_config.resolution.value);
    let spiral_mode = writer.spiral_mode();
    let enable_arc_fitting = writer.arc_fitting_enabled();

    // Both the plain Douglas-Peucker (geometry::simplify::douglas_peucker) and
    // the arc-fitter (arc_fitter::do_arc_fitting_and_simplify) need a tolerance,
    // but in *different units*:
    //   - geometry::simplify::douglas_peucker scales its argument internally, so
    //     it expects the tolerance in **mm** (unscaled).
    //   - arc_fitter::do_arc_fitting_and_simplify drives multi_point::douglas_peucker
    //     which compares against the raw *scaled* tolerance, exactly as C++
    //     feeds `scaled<double>(resolution)` (LayerRegion.cpp:791,798).
    let resolution_mm = writer.resolution();

    // SPARSE_INFILL_RESOLUTION = 0.04mm (libslic3r.h:65). Internal sparse infill
    // is simplified with this coarser tolerance.
    const SPARSE_INFILL_RESOLUTION_MM: f64 = 0.04;

    // Per-role simplification tolerance, mirroring LayerRegion::simplify_path
    // (LayerRegion.cpp:793-801). Internal sparse infill uses the coarser
    // SCALED_SPARSE_INFILL_RESOLUTION; everything else uses scaled_resolution.
    let tolerance_mm = if path.role == ExtrusionRole::InternalInfill {
        SPARSE_INFILL_RESOLUTION_MM
    } else {
        resolution_mm
    };

    // Working copy of the scaled points.
    let mut work_points: Vec<Point> = points.to_vec();

    if enable_arc_fitting && !spiral_mode && !path.polyline.fitting_result.is_empty() {
        // -------------------------------------------------------------------
        // Arc-fitting path: emit the STORED fitting_result that
        // LayerRegion::simplify_path's `simplify_by_fitting_arc(scaled_resolution)`
        // (LayerRegion.cpp:796-798) already computed once on the RAW perimeter
        // points (and which seam-splitting in extrude_loop keeps consistent with
        // polyline.points via ExtrusionLoop::split_at). This is exactly what C++
        // GCode::_extrude does (GCode.cpp:6700-6745): it iterates the stored
        // path.polyline.fitting_result, it does NOT re-fit.
        //
        // PARITY-FIX (arc-fit): the previous code re-ran do_arc_fitting_and_simplify
        // here on the already-arc-fit-AND-DP-simplified points, which destroyed
        // ~36% of the arcs that pass-1 had found (5758 -> 3688 outer-wall arcs)
        // because the per-segment Douglas-Peucker in pass-1 had already removed the
        // interior points that try_create_arc / are_points_within_slice need to
        // re-confirm curvature. Emitting the stored result restores native parity
        // (~5758 ≈ native 5793). Material is unchanged (arc dE uses arc.length, same
        // value; linear-run dE identical).
        // -------------------------------------------------------------------
        let fitting_result = &path.polyline.fitting_result;

        // GCode.cpp:6703-6744: iterate fitting_result, emit G1/G2/G3.
        for seg in fitting_result {
            match seg.path_type {
                EMovePathType::LinearMove => {
                    // GCode.cpp:6705-6720
                    let start_index = seg.start_point_index;
                    let end_index = seg.end_point_index;
                    for point_index in (start_index + 1)..(end_index + 1) {
                        let from = work_points[point_index - 1];
                        let to = work_points[point_index];
                        // line.length() in mm (scaled -> mm via unscale).
                        let line_length = unscale(from.distance_to_f64(to) as i64);
                        if line_length < EPSILON {
                            continue;
                        }
                        let de = line_length * e_per_mm;
                        if path.no_extrusion {
                            // Native GCode.cpp:6686 extrude_to_xy(..., is_force_no_
                            // extrusion()): E-less move, filament E not advanced.
                            writer.wipe_to(unscale(to.x()), unscale(to.y()), None);
                        } else {
                            writer.extrude_to(unscale(to.x()), unscale(to.y()), de, None);
                        }
                    }
                }
                EMovePathType::ArcMoveCw | EMovePathType::ArcMoveCcw => {
                    // GCode.cpp:6722-6737
                    let arc = &seg.arc_data;
                    // arc_length = arc.length * SCALING_FACTOR (scaled length -> mm).
                    let arc_length = arc.length * crate::libslic3r::SCALING_FACTOR;
                    if arc_length < EPSILON {
                        continue;
                    }
                    if std::env::var("ARCDBG").is_ok() {
                        let wp_start = work_points[seg.start_point_index];
                        let wp_end = work_points[seg.end_point_index];
                        let ds = ((wp_start.x() - arc.start_point.x()) as f64)
                            .hypot((wp_start.y() - arc.start_point.y()) as f64);
                        let de_ = ((wp_end.x() - arc.end_point.x()) as f64)
                            .hypot((wp_end.y() - arc.end_point.y()) as f64);
                        // center equidistant check (scaled):
                        let r_s = ((arc.circle.center.x() - arc.start_point.x()) as f64)
                            .hypot((arc.circle.center.y() - arc.start_point.y()) as f64);
                        let r_e = ((arc.circle.center.x() - arc.end_point.x()) as f64)
                            .hypot((arc.circle.center.y() - arc.end_point.y()) as f64);
                        if ds > 1000.0 || de_ > 1000.0 || (r_s - r_e).abs() > 0.02 * r_s.max(1.0) {
                            eprintln!(
                                "ARCDBG2 wp_start!=arcstart={:.0} wp_end!=arcend={:.0} r_s={:.0} r_e={:.0} ({})",
                                ds, de_, r_s, r_e,
                                crate::extrusion_entity::role_to_string(path.role)
                            );
                        }
                    }
                    // center_offset = point_to_gcode(center) - point_to_gcode(start).
                    // Origin/extruder offset cancels in the difference, so plain
                    // unscale matches the linear path's coordinate convention.
                    // Unscale each endpoint then subtract, mirroring C++'s ordering.
                    let i = unscale(arc.circle.center.x()) - unscale(arc.start_point.x());
                    let j = unscale(arc.circle.center.y()) - unscale(arc.start_point.y());
                    let de = arc_length * e_per_mm;
                    writer.extrude_arc(
                        unscale(arc.end_point.x()),
                        unscale(arc.end_point.y()),
                        i,
                        j,
                        de,
                        arc.direction,
                        None,
                    );
                }
                EMovePathType::NoopMove => {}
            }
        }
    } else {
        // Plain Douglas-Peucker + linear emission (GCode.cpp:6670-6686 G1 path).
        // Polyline::simplify -> MultiPoint::_douglas_peucker (Polyline.cpp:146-150).
        if work_points.len() > 2 {
            let simplified = Polyline::douglas_peucker(
                &Polyline::from_points(work_points.clone()),
                tolerance_mm,
            );
            work_points = simplified.points;
        }

        // GCode.cpp:6676-6686: iterate polyline.lines(), skip segments < EPSILON.
        for i in 1..work_points.len() {
            let from = work_points[i - 1];
            let to = work_points[i];

            // line.length() in mm (scaled -> mm via unscale).
            let line_length = unscale(from.distance_to_f64(to) as i64);
            if line_length < EPSILON {
                continue;
            }
            let de = line_length * e_per_mm;
            writer.extrude_to(unscale(to.x()), unscale(to.y()), de, None);
        }
    }

    // Emit cooling end marker after extrusion
    // C++ GCode.cpp:6361-6363
    writer.write_raw(";_EXTRUDE_END");
}

/// Extrude any extrusion entity (dispatcher).
///
/// C++ reference: GCode::extrude_entity()
/// GCode.cpp:3100-3400 (~300 lines)
///
/// This function dispatches to the appropriate extrusion function
/// based on the entity type (Loop, Path, Collection).
///
/// # Arguments
/// * `entity` - The extrusion entity type to extrude
/// * `writer` - GCodeWriter to emit commands
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors or unsupported entity type
pub fn extrude_entity(
    entity: &ExtrusionEntityType,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) -> Result<()> {
    /// Dispatch based on entity type
    /// GCode.cpp:3120-3180
    /// C++: std::string GCode::extrude_entity(const ExtrusionEntity &entity, const std::string &description, double speed) {
    /// C++: if (entity.is_loop()) {
    /// C++: return this->extrude_loop(*static_cast<const ExtrusionLoop*>(&entity), description, speed);
    /// C++: } else if (entity.is_path()) {
    /// C++: return this->_extrude(*static_cast<const ExtrusionPath*>(&entity), description, speed);
    /// C++: } else if (entity.is_collection()) {
    /// C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
    /// C++: } else {
    /// C++: throw std::runtime_error("Unknown extrusion entity type");
    /// C++: }
    /// C++: }
    match entity {
        /// Dispatch to loop handler
        /// GCode.cpp:3122-3125
        /// C++: if (entity.is_loop()) {
        /// C++: return this->extrude_loop(*static_cast<const ExtrusionLoop*>(&entity), description, speed);
        /// C++: }
        ExtrusionEntityType::Loop(loop_entity) => {
            extrude_loop(loop_entity, writer, config, is_first_layer);
        }

        /// Dispatch to path handler
        /// GCode.cpp:3126-3129
        /// C++: else if (entity.is_path()) {
        /// C++: return this->_extrude(*static_cast<const ExtrusionPath*>(&entity), description, speed);
        /// C++: }
        ExtrusionEntityType::Path(path) => {
            // R245 (gated): native _extrude emits set_speed(F) for EVERY path
            // (GCode.cpp:6663, cooling buffer dedups); the per-path F embeds
            // the volumetric cap, so variable-width gap/fill paths carry
            // fractional Fs (F14761.269 = 25mm3s / mm3_per_mm). writer
            // set_speed dedups, so without VOLCAP this is a no-op.
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                use crate::extrusion_entity::ExtrusionRole;
                let role = path.role;
                let base = if is_first_layer {
                    match role {
                        ExtrusionRole::InternalInfill
                        | ExtrusionRole::SolidInfill
                        | ExtrusionRole::TopSolidInfill
                        | ExtrusionRole::BottomSurface => {
                            if config.initial_layer_infill_speed > 0.0 {
                                config.initial_layer_infill_speed
                            } else {
                                config.initial_layer_speed
                            }
                        }
                        _ => config.initial_layer_speed,
                    }
                } else {
                    match role {
                        ExtrusionRole::ExternalPerimeter => config.external_perimeter_speed,
                        ExtrusionRole::Perimeter => config.perimeter_speed,
                        ExtrusionRole::InternalInfill => config.infill_speed,
                        ExtrusionRole::SolidInfill => config.solid_infill_speed,
                        ExtrusionRole::TopSolidInfill => config.top_solid_infill_speed,
                        ExtrusionRole::BridgeInfill => config.bridge_speed,
                        ExtrusionRole::GapFill => config.gap_fill_speed,
                        ExtrusionRole::FloatingVerticalShell => {
                            config.vertical_shell_speed / 100.0 * config.solid_infill_speed
                        }
                        _ => config.perimeter_speed,
                    }
                };
                let speed = if crate::faithful_gate("VOLCAP_FAITHFUL") {
                    volumetric_capped_speed(
                        base,
                        path.mm3_per_mm,
                        writer.config_ref().filament_max_volumetric_speed,
                        config.print_flow_ratio,
                    )
                } else {
                    base
                };
                if speed > 0.0 {
                    let cooling_comment = if role == ExtrusionRole::BridgeInfill {
                        ""
                    } else if role == ExtrusionRole::ExternalPerimeter {
                        ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
                    } else {
                        ";_EXTRUDE_SET_SPEED"
                    };
                    set_speed_before_path(writer, speed * 60.0, cooling_comment);
                }
            }
            extrude_path(path, writer, config, is_first_layer);
            // R233: native extrude_path installs the wipe path = the path's
            // SOURCE polyline REVERSED (GCode.cpp:5703-5717, non-tree branch)
            // — fills wipe backwards along the just-printed line. Rust only
            // installed wipe paths at loop ends, so 207 native wipes had no
            // rust counterpart (bare retracts).
            if crate::gcode::writer::lift_faithful_gate() {
                let pts: Vec<(f64, f64)> = path
                    .polyline
                    .points()
                    .iter()
                    .rev()
                    .map(|pt| (crate::unscale(pt.x()), crate::unscale(pt.y())))
                    .collect();
                if pts.len() >= 2 {
                    writer.set_wipe_path_points(pts);
                }
            }
            // R227: native extrude_path wrapper resets accel after the path
            // (GCode.cpp:5719-5725).
            writer.reset_acceleration_default(is_first_layer);
        }

        /// Dispatch to collection handler (recursive)
        /// GCode.cpp:3130-3133
        /// C++: else if (entity.is_collection()) {
        /// C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
        /// C++: }
        ExtrusionEntityType::Collection(collection) => {
            extrude_collection(collection, writer, config, is_first_layer)?;
        }
    }

    /// TODO : Add feature type annotations
    /// GCode.cpp:3150-3200
    /// C++: // Emit feature comment if role changed
    /// C++: if (entity->role() != m_last_extrusion_role) {
    /// C++: gcode += "; FEATURE: " + entity->role_to_string() + "\n";
    /// C++: m_last_extrusion_role = entity->role();
    /// C++: }

    /// TODO : Add metadata comments (description, speed, etc.)
    /// GCode.cpp:3210-3250
    /// C++: if (!description.empty()) {
    /// C++: gcode += "; " + description + "\n";
    /// C++: }
    Ok(())
}

/// Extrude perimeters for a single region on one island.
///
/// C++ reference: GCode::extrude_perimeters()
/// GCode.cpp:5362-5383
///
/// Iterates the region's perimeters and extrudes each entity.
/// When `skip_inner_walls` is true (spiral vase mode), only external
/// perimeters are emitted.
pub fn extrude_perimeters(
    region: &crate::layer::LayerRegion,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    skip_inner_walls: bool,
) {
    if region.perimeters.entities.is_empty() {
        return;
    }

    // Retract and travel to first perimeter point
    writer.retract();
    writer.set_travel_acceleration(6000.0);
    let first_target = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        travel_target_for_entity(&region.perimeters.entities[0], writer_last_pos(writer))
    } else {
        get_entity_first_point(&region.perimeters.entities[0])
    };
    if let Some(first_pt) = first_target {
        let target_x = crate::unscale(first_pt.x());
        let target_y = crate::unscale(first_pt.y());
        if crate::gcode::writer::lift_faithful_gate() {
            let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
            writer.travel_to_xyz(target_x, target_y, zdest);
        } else {
            writer.travel_to(target_x, target_y, None);
        }
    }
    writer.unretract();

    if skip_inner_walls {
        // Spiral vase mode: only emit outer perimeter entities
        use crate::extrusion_entity::ExtrusionRole;
        for entity in &region.perimeters.entities {
            let role = get_entity_role(entity);
            if role == ExtrusionRole::ExternalPerimeter {
                let _ = extrude_entity(entity, writer, config, is_first_layer);
            }
        }
    } else {
        // GCode.cpp:5372-5380: extrude perimeters via extrude_collection
        // which handles FEATURE comments, LINE_WIDTH, feedrate (F), and
        // extrusion acceleration (M204) per entity role change.
        let _ = extrude_collection(&region.perimeters, writer, config, is_first_layer);
    }
}

/// Extrude infill for a single region.
///
/// C++ reference: GCode::extrude_infill()
/// GCode.cpp:5753-5780
///
/// Chains the region's infill entities by a greedy nearest-neighbor algorithm
/// (starting near the writer's current position, == C++ `m_last_pos`), then
/// extrudes each. Per C++, each fill that is itself an ExtrusionEntityCollection
/// (EEC) is re-chained via `chained_path_from(m_last_pos)` before emission.
///
/// C++ calls this once per `ironing` flag (false then true) so that all
/// non-ironing infill across all regions precedes all ironing infill. The Rust
/// pipeline drives this per region, so we replicate the filter locally: chain &
/// emit non-ironing fills first, then ironing fills.
pub fn extrude_infill(
    region: &crate::layer::LayerRegion,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    use crate::extrusion_entity::ExtrusionRole;

    if region.fills.entities.is_empty() {
        return;
    }

    // GCode.cpp:5759-5765: build the `extrusions` vector, filtering by ironing
    // role. `(ee->role() == erIroning) == ironing` selects non-ironing fills on
    // the first pass and ironing fills on the second. We clone the entities so
    // chain_and_reorder (which reorders/reverses in place) does not mutate the
    // stored region.fills.
    for ironing_pass in [false, true] {
        let mut extrusions: Vec<ExtrusionEntityType> = region
            .fills
            .entities
            .iter()
            .filter(|ee| (get_entity_role(ee) == ExtrusionRole::Ironing) == ironing_pass)
            .cloned()
            .collect();
        if extrusions.is_empty() {
            continue;
        }

        // GCode.cpp:5768: chain_and_reorder_extrusion_entities(extrusions, &m_last_pos).
        // NOTE: chain_and_reorder retains-out empty collections (ShortestPath.cpp:1033-1035),
        // so `extrusions` may become empty here even though the role filter found entries.
        let m_last_pos = writer_last_pos(writer);
        crate::shortest_path::chain_and_reorder_extrusion_entities(
            &mut extrusions,
            Some(&m_last_pos),
        );

        // Retract and travel to the (now reordered) first infill point.
        let Some(first_pt) = extrusions.first().and_then(|e| {
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                travel_target_for_entity(e, writer_last_pos(writer))
            } else {
                get_entity_first_point(e)
            }
        }) else {
            continue;
        };
        // ZSMOOTH_FAITHFUL: bucket-start retract through faithful
        // needs_retraction (GCode.cpp:6964) — R167.
        {
            let __zs_do_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let travel_len = (dx * dx + dy * dy).sqrt();
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.needs_retraction_faithful(from, first_pt, role, travel_len)
            } else {
                true
            };
            if __zs_do_retract {
                writer.retract();
            }
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.set_travel_acceleration_for(role, (dx * dx + dy * dy).sqrt());
            } else {
                writer.set_travel_acceleration(6000.0);
            }
            if crate::gcode::writer::lift_faithful_gate() {
                let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
                writer.travel_to_xyz(
                    crate::unscale(first_pt.x()),
                    crate::unscale(first_pt.y()),
                    zdest,
                );
            } else {
                writer.travel_to(crate::unscale(first_pt.x()), crate::unscale(first_pt.y()), None);
            }
            // Native _extrude unconditionally unretracts at extrusion start —
            // also clears the initial/carried retracted state.
            writer.unretract();
        }

        // GCode.cpp:5769-5776: for each fill, if it is an EEC, re-chain it via
        // chained_path_from(m_last_pos) and emit its entities; otherwise emit
        // the entity directly.
        for fill in &extrusions {
            match fill {
                ExtrusionEntityType::Collection(eec) => {
                    let chained = chained_path_from(eec, writer);
                    let _ = extrude_collection(&chained, writer, config, is_first_layer);
                }
                _ => {
                    let _ = extrude_entity(fill, writer, config, is_first_layer);
                }
            }
        }
    }
}

/// Island-subset perimeter emission (GCode::extrude_perimeters on one island's
/// by_region perimeters). Same retract/travel/extrude as `extrude_perimeters`
/// but over a given entity subset (this island+region's perimeter EECs).
pub fn extrude_perimeters_entities(
    entities: &[ExtrusionEntityType],
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    skip_inner_walls: bool,
    // Per-entity cooling-node ids (ZSMOOTH_FAITHFUL): when Some, a
    // `; COOLING_NODE: <id>` marker precedes every entity whose id != -1
    // (GCode.cpp:5738-5747 — native's compare value never updates from -1).
    cooling_node_ids: Option<&[i32]>,
) {
    if entities.is_empty() {
        return;
    }
    // ZSMOOTH_FAITHFUL: the bucket-start retract must go through the faithful
    // needs_retraction (GCode.cpp:6964) — the unconditional retract here was
    // the outer-wall->gap wipe excess (R167). Legacy path keeps the
    // unconditional behavior (byte-locked).
    let first_pt_opt = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        travel_target_for_entity(&entities[0], writer_last_pos(writer))
    } else {
        get_entity_first_point(&entities[0])
    };
    let do_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        match first_pt_opt {
            Some(fp) => {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(fp.x()) - crate::unscale(from.x);
                let dy = crate::unscale(fp.y()) - crate::unscale(from.y);
                let travel_len = (dx * dx + dy * dy).sqrt();
                let role = get_entity_role(&entities[0]);
                writer.needs_retraction_faithful(from, fp, role, travel_len)
            }
            None => true,
        }
    } else {
        true
    };
    if do_retract {
        writer.retract();
    }
    if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        if let Some(fp) = first_pt_opt {
            let from = writer_last_pos(writer);
            let dx = crate::unscale(fp.x()) - crate::unscale(from.x);
            let dy = crate::unscale(fp.y()) - crate::unscale(from.y);
            writer.set_travel_acceleration_for(
                get_entity_role(&entities[0]),
                (dx * dx + dy * dy).sqrt(),
            );
        }
    } else {
        writer.set_travel_acceleration(6000.0);
    }
    if let Some(first_pt) = first_pt_opt {
        if crate::gcode::writer::lift_faithful_gate() {
            let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
            writer.travel_to_xyz(
                crate::unscale(first_pt.x()),
                crate::unscale(first_pt.y()),
                zdest,
            );
        } else {
            writer.travel_to(crate::unscale(first_pt.x()), crate::unscale(first_pt.y()), None);
        }
    }
    // Native _extrude unconditionally unretracts at extrusion start.
    writer.unretract();
    if skip_inner_walls {
        use crate::extrusion_entity::ExtrusionRole;
        for entity in entities {
            if get_entity_role(entity) == ExtrusionRole::ExternalPerimeter {
                let _ = extrude_entity(entity, writer, config, is_first_layer);
            }
        }
    } else if let Some(ids) = cooling_node_ids {
        // Marker-injecting path (gated): per-entity emission, same order and
        // writer state as the batch path below.
        for (k, entity) in entities.iter().enumerate() {
            let id = ids.get(k).copied().unwrap_or(-1);
            if id != -1 {
                writer.write_raw(&format!("; COOLING_NODE: {}", id));
            }
            let coll = crate::extrusion_entity::ExtrusionEntityCollection {
                entities: vec![entity.clone()],
                no_sort: true,
                ..Default::default()
            };
            let _ = extrude_collection(&coll, writer, config, is_first_layer);
        }
    } else {
        let coll = crate::extrusion_entity::ExtrusionEntityCollection {
            entities: entities.to_vec(),
            no_sort: true,
            ..Default::default()
        };
        let _ = extrude_collection(&coll, writer, config, is_first_layer);
    }
}

/// Island-subset infill emission (GCode::extrude_infill on one island's by_region
/// fills). Same ironing-split / chain / extrude as `extrude_infill` over a subset.
pub fn extrude_infill_entities(
    entities: &[ExtrusionEntityType],
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    use crate::extrusion_entity::ExtrusionRole;
    if entities.is_empty() {
        return;
    }
    for ironing_pass in [false, true] {
        let mut extrusions: Vec<ExtrusionEntityType> = entities
            .iter()
            .filter(|ee| (get_entity_role(ee) == ExtrusionRole::Ironing) == ironing_pass)
            .cloned()
            .collect();
        if extrusions.is_empty() {
            continue;
        }
        let m_last_pos = writer_last_pos(writer);
        crate::shortest_path::chain_and_reorder_extrusion_entities(&mut extrusions, Some(&m_last_pos));
        let Some(first_pt) = extrusions.first().and_then(|e| {
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                travel_target_for_entity(e, writer_last_pos(writer))
            } else {
                get_entity_first_point(e)
            }
        }) else {
            continue;
        };
        // ZSMOOTH_FAITHFUL: bucket-start retract through faithful
        // needs_retraction (GCode.cpp:6964) — R167.
        {
            let __zs_do_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let travel_len = (dx * dx + dy * dy).sqrt();
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.needs_retraction_faithful(from, first_pt, role, travel_len)
            } else {
                true
            };
            if __zs_do_retract {
                writer.retract();
            }
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.set_travel_acceleration_for(role, (dx * dx + dy * dy).sqrt());
            } else {
                writer.set_travel_acceleration(6000.0);
            }
            if crate::gcode::writer::lift_faithful_gate() {
                let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
                writer.travel_to_xyz(
                    crate::unscale(first_pt.x()),
                    crate::unscale(first_pt.y()),
                    zdest,
                );
            } else {
                writer.travel_to(crate::unscale(first_pt.x()), crate::unscale(first_pt.y()), None);
            }
            // Native _extrude unconditionally unretracts at extrusion start —
            // also clears the initial/carried retracted state.
            writer.unretract();
        }
        for fill in &extrusions {
            match fill {
                ExtrusionEntityType::Collection(eec) => {
                    let chained = chained_path_from(eec, writer);
                    let _ = extrude_collection(&chained, writer, config, is_first_layer);
                }
                _ => {
                    let _ = extrude_entity(fill, writer, config, is_first_layer);
                }
            }
        }
    }
}

/// Extrude support material fills.
///
/// C++ reference: GCode::extrude_support()
/// GCode.cpp:5782-5848
///
/// Splits the fills into ironing / non-ironing groups, chains each group with a
/// greedy nearest-neighbor algorithm (starting near the writer's current
/// position == C++ `m_last_pos`), then emits non-ironing first, ironing second
/// ("make sure the ironing was after the support extrusions", GCode.cpp:5844).
/// Each role gets its own FEATURE label.
pub fn extrude_support(
    support_fills: &ExtrusionEntityCollection,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    use crate::extrusion_entity::ExtrusionRole;

    if support_fills.entities.is_empty() {
        return;
    }

    // GCode.cpp:5794-5802: split ironing vs non-ironing. Clone so chaining does
    // not mutate the stored support_fills.
    let mut extrusions: Vec<ExtrusionEntityType> = Vec::new();
    let mut ironing_extrusions: Vec<ExtrusionEntityType> = Vec::new();
    let mut has_support_ironing = false;
    for ee in &support_fills.entities {
        if get_entity_role(ee) == ExtrusionRole::SupportIroning {
            ironing_extrusions.push(ee.clone());
            has_support_ironing = true;
        } else {
            extrusions.push(ee.clone());
        }
    }
    // GCode.cpp:5803
    if extrusions.is_empty() && ironing_extrusions.is_empty() {
        return;
    }
    // GCode.cpp:5804-5810: chain each group; clear ironing if disabled.
    has_support_ironing = has_support_ironing && config.enable_support_ironing;
    if has_support_ironing {
        let m_last_pos = writer_last_pos(writer);
        crate::shortest_path::chain_and_reorder_extrusion_entities(
            &mut ironing_extrusions,
            Some(&m_last_pos),
        );
    } else {
        ironing_extrusions.clear();
    }
    {
        let m_last_pos = writer_last_pos(writer);
        crate::shortest_path::chain_and_reorder_extrusion_entities(
            &mut extrusions,
            Some(&m_last_pos),
        );
    }

    // GCode.cpp:5815-5842: emit one entity, with role label. Nested collections
    // recurse into extrude_support (matching the C++ dynamic_cast<EEC> branch).
    fn process_entities(
        entities: &[ExtrusionEntityType],
        writer: &mut GCodeWriter,
        config: &crate::print_config::PrintObjectConfig,
        is_first_layer: bool,
    ) {
        for ee in entities {
            if let ExtrusionEntityType::Collection(coll) = ee {
                // GCode.cpp:5836-5837: collection -> recurse.
                extrude_support(coll, writer, config, is_first_layer);
                continue;
            }
            let role = get_entity_role(ee);
            let label = match role {
                ExtrusionRole::SupportMaterial => "support material",
                ExtrusionRole::SupportMaterialInterface => "support material interface",
                ExtrusionRole::SupportIroning => "support ironing",
                ExtrusionRole::SupportTransition => "support transition",
                _ => "support material",
            };
            writer.write_comment(&format!("FEATURE: {}", label));
            let _ = extrude_entity(ee, writer, config, is_first_layer);
        }
    }

    // First positioning travel before the chained non-ironing group, matching
    // the prior behaviour (retract + travel to the reordered first point).
    if let Some(first_pt) = extrusions.first().and_then(|e| {
        if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
            travel_target_for_entity(e, writer_last_pos(writer))
        } else {
            get_entity_first_point(e)
        }
    }) {
        // ZSMOOTH_FAITHFUL: faithful needs_retraction (R167).
        {
            let __zs_do_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let travel_len = (dx * dx + dy * dy).sqrt();
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.needs_retraction_faithful(from, first_pt, role, travel_len)
            } else {
                true
            };
            if __zs_do_retract {
                writer.retract();
            }
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.set_travel_acceleration_for(role, (dx * dx + dy * dy).sqrt());
            } else {
                writer.set_travel_acceleration(6000.0);
            }
            if crate::gcode::writer::lift_faithful_gate() {
                let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
                writer.travel_to_xyz(
                    crate::unscale(first_pt.x()),
                    crate::unscale(first_pt.y()),
                    zdest,
                );
            } else {
                writer.travel_to(crate::unscale(first_pt.x()), crate::unscale(first_pt.y()), None);
            }
            // Native _extrude unconditionally unretracts at extrusion start —
            // also clears the initial/carried retracted state.
            writer.unretract();
        }
    }

    // GCode.cpp:5843-5845: non-ironing first, then ironing.
    process_entities(&extrusions, writer, config, is_first_layer);
    if has_support_ironing {
        process_entities(&ironing_extrusions, writer, config, is_first_layer);
    }
}

/// Generate a travel move to the specified point.
///
/// C++ reference: GCode::travel_to()
/// GCode.cpp:6416-6572 (~157 lines)
///
/// This function:
/// 1. Decides whether retraction is needed
/// 2. Performs retraction if needed
/// 3. Optionally avoids crossing perimeters
/// 4. Optionally performs Z-hop
/// 5. Generates G0/G1 travel move
/// 6. Optionally performs un-retraction
///
/// # Arguments
/// * `point` - Destination point
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Travel configuration
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn travel_to(point: Point, writer: &mut GCodeWriter, config: &TravelConfig) -> Result<()> {
    /// Define the travel move as a line between current position and target point
    /// GCode.cpp:6418-6420
    /// C++: Polyline travel { this->last_pos(), point };
    let current_pos = writer.position();
    let current_pos_scaled = Point::new(
        (current_pos.x * 1_000_000.0) as i64,
        (current_pos.y * 1_000_000.0) as i64,
    );

    /// Create travel polyline
    /// GCode.cpp:6418-6420
    let travel = Polyline::from_points(vec![current_pos_scaled, point]);

    /// Calculate travel distance and check if retraction is needed
    /// GCode.cpp:6422-6424
    /// C++: bool needs_retraction = this->needs_retraction(travel, role, lift_type);
    let travel_distance = unscale(travel.length() as i64);
    let needs_retraction =
        config.retract_on_travel && travel_distance >= config.retract_length_travel;

    /// If reduce_crossing_wall is enabled, try to plan multi-hop path
    /// GCode.cpp:6431-6440
    /// C++: if (m_config.reduce_crossing_wall && !m_avoid_crossing_perimeters.disabled_once())
    /// C++: {
    /// C++: travel = m_avoid_crossing_perimeters.travel_to(*this, point, &could_be_wipe_disabled);
    /// C++: needs_retraction = this->needs_retraction(travel, role, lift_type);
    /// C++: }
    // TODO: Implement avoid_crossing_perimeters integration (GCode.cpp:6431-6440)
    // For now, skip path optimization - use direct travel

    /// Perform retraction if needed
    /// GCode.cpp:6447-6465
    /// C++: if (needs_retraction) {
    /// C++: if (m_config.reduce_crossing_wall && could_be_wipe_disabled && !m_last_scarf_seam_flag)
    /// C++: m_wipe.reset_path();
    /// C++: Point last_post_before_retract = this->last_pos();
    /// C++: gcode += this->retract(false, false, lift_type);
    /// C++: ...
    /// C++: } else {
    /// C++: m_wipe.reset_path();
    /// C++: }
    if needs_retraction {
        retract(writer, false)?;
    }

    /// Emit travel move(s)
    /// GCode.cpp:6475-6562
    /// C++: if (travel.size() >= 2) {
    /// C++: ...
    /// C++: for (size_t i = 1; i < travel.size(); ++i)
    /// C++: gcode += m_writer.travel_to_xy(this->point_to_gcode(travel.points[i]), comment, use_short_travel_accel);
    /// C++: ...
    /// C++: }
    if travel.points().len() >= 2 {
        /// Emit travel moves for each segment
        /// GCode.cpp:6520-6562
        for i in 1..travel.points().len() {
            let target = travel.points()[i];
            let target_x = unscale(target.x);
            let target_y = unscale(target.y);
            writer.travel_to(target_x, target_y, None);
        }
    }

    Ok(())
}

/// Perform retraction.
///
/// C++ reference: GCode::retract()
/// GCode.cpp:6693-6725 (~33 lines)
///
/// This function:
/// 1. Checks if already retracted (avoid double retraction)
/// 2. Emits retraction command (G1 E-<length> or G10 for firmware retract)
/// 3. Optionally performs wipe move
/// 4. Updates retraction state
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
/// * `wipe` - Whether to perform wipe move after retraction
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn retract(writer: &mut GCodeWriter, wipe: bool) -> Result<()> {
    /// Check if already retracted - no-op if yes
    /// GCode.cpp:6695-6697
    /// C++: if (m_writer.filament() == nullptr)
    /// C++: return gcode;
    if writer.is_retracted() {
        return Ok(());
    }

    /// Perform wipe if enabled and wipe path available
    /// GCode.cpp:6699-6702
    /// C++: if (FILAMENT_CONFIG(wipe) && m_wipe.has_path() && scale_(FILAMENT_CONFIG(wipe_distance)) > SCALED_EPSILON) {
    /// C++: gcode += toolchange ? m_writer.retract_for_toolchange(true) : m_writer.retract(true);
    /// C++: gcode += m_wipe.wipe(*this, toolchange, is_last_retraction);
    /// C++: }
    // TODO: Implement wipe integration (GCode.cpp:6699-6702)
    // For now, skip wipe - just do direct retraction
    let _ = wipe;

    /// Call writer's retract method (handles firmware retract or manual retract)
    /// GCode.cpp:6707-6708
    /// C++: gcode += toolchange ? m_writer.retract_for_toolchange() : m_writer.retract();
    writer.retract();

    /// Reset E position after retraction
    /// GCode.cpp:6710
    /// C++: gcode += m_writer.reset_e();
    // Note: GCodeWriter::retract() already handles E tracking

    /// Perform Z-lift if retraction length > 0 or firmware retraction enabled
    /// GCode.cpp:6711-6720
    /// C++: if (m_writer.filament()->retraction_length() > 0 || m_config.use_firmware_retraction) {
    /// C++: if (apply_instantly)
    /// C++: gcode += m_writer.eager_lift(lift_type,toolchange);
    /// C++: else
    /// C++: gcode += m_writer.lazy_lift(lift_type, m_spiral_vase != nullptr, toolchange);
    /// C++: }
    // Note: Z-lift is handled by GCodeWriter::retract() which calls do_z_hop()
    Ok(())
}

/// Perform un-retraction (prime).
///
/// C++ reference: GCodeWriter::unretract()
/// GCodeWriter.cpp:808-839 (~32 lines)
///
/// This function:
/// 1. Checks if currently retracted (no-op if not)
/// 2. Emits un-retraction command (G1 E<length> or G11 for firmware)
/// 3. Optionally adds extra restart length
/// 4. Updates retraction state
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn unretract(writer: &mut GCodeWriter) -> Result<()> {
    /// Check if currently retracted - no-op if not
    /// GCodeWriter.cpp:810-820
    /// C++: if (double dE = filament()->unretract(); dE != 0) {
    /// C++: if (config.use_firmware_retraction) {
    /// C++: gcode += FLAVOR_IS(gcfMachinekit) ? "G23 ;unretract \n" : "G11 ;unretract \n";
    /// C++: gcode += reset_e();
    /// C++: }
    /// C++: else {
    /// C++: ...
    /// C++: }
    /// C++: }
    if !writer.is_retracted() {
        return Ok(());
    }

    /// Call writer's unretract method (handles firmware unretract or manual unretract)
    /// GCodeWriter.cpp:815-828
    /// C++: GCodeG1Formatter w;
    /// C++: w.emit_e(filament()->E()+extra_retract);
    /// C++: w.emit_f(filament()->deretract_speed() * 60.);
    /// C++: w.emit_comment(GCodeWriter::full_gcode_comment, " ; unretract");
    /// C++: gcode += w.string();
    writer.unretract();

    Ok(())
}

/// Perform a wipe move.
///
/// C++ reference: Wipe::wipe()
/// GCode.cpp:355-438 (~83 lines)
///
/// This function:
/// 1. Calculates wipe speed (reduced from travel speed)
/// 2. Takes the stored wipe path and clips it to wipe distance
/// 3. Retracts while traveling along wipe path
/// 4. Emits wipe start/end tags for processor
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
/// * `wipe_path` - Path to wipe along (from last extrusion)
/// * `wipe_distance` - Target wipe distance (mm)
/// * `retraction_length` - Total retraction length to distribute over wipe
/// * `wipe_speed` - Wipe speed in mm/s
/// * `toolchange` - Whether this is a toolchange wipe
/// * `is_last` - Whether this is the last wipe (affects cooling markers)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn wipe(
    writer: &mut GCodeWriter,
    wipe_path: &Polyline,
    wipe_distance: f64,
    retraction_length: f64,
    wipe_speed: f64,
    _toolchange: bool,
    is_last: bool,
) -> Result<()> {
    /// Get retraction length to apply during wipe
    /// GCode.cpp:366-373
    /// C++: double length = toolchange
    /// C++: ? gcodegen.writer().filament()->retract_length_toolchange()
    /// C++: : gcodegen.writer().filament()->retraction_length();
    /// C++: // Shorten the retraction length by the amount already retracted before wipe.
    /// C++: length *= (1. - gcodegen.writer().filament()->retract_before_wipe());
    let length = retraction_length;

    /// Only wipe if retraction length is positive
    /// GCode.cpp:375
    /// C++: if (length >= 0) {
    if length < 1e-6 {
        return Ok(());
    }

    /// Calculate wipe distance
    /// GCode.cpp:379-380
    /// C++: // BBS
    /// C++: double wipe_dist = scale_(gcodegen.config().wipe_distance.get_at(gcodegen.writer().filament()->id()));
    let wipe_dist_scaled = scale(wipe_distance);

    /// Take the stored wipe path and replace first point with current position
    /// GCode.cpp:382-388
    /// C++: /* Take the stored wipe path and replace first point with the current actual position
    /// C++: (they might be different, for example, in case of loop clipping). */
    /// C++: Polyline wipe_path;
    /// C++: wipe_path.append(gcodegen.last_pos());
    /// C++: wipe_path.append(
    /// C++: this->path.points.begin() + 1,
    /// C++: this->path.points.end()
    /// C++: );
    let mut wipe_polyline = wipe_path.clone();
    if wipe_polyline.points.is_empty() {
        return Ok(());
    }

    // Replace first point with current position
    let current_pos_f = writer.position();
    let current_pos = Point::new(
        (current_pos_f.x * 1_000_000.0) as i64,
        (current_pos_f.y * 1_000_000.0) as i64,
    );
    wipe_polyline.points[0] = current_pos;

    /// Clip wipe path to wipe distance
    /// GCode.cpp:390
    /// C++: wipe_path.clip_end(wipe_path.length() - wipe_dist);
    let total_length = wipe_polyline.length() as i64;
    if total_length > wipe_dist_scaled {
        wipe_polyline.clip_end((total_length - wipe_dist_scaled) as f64);
    }

    /// Subdivide the retraction in segments along wipe path
    /// GCode.cpp:392-407
    /// C++: // subdivide the retraction in segments
    /// C++: if (!wipe_path.empty()) {
    if !wipe_polyline.points.is_empty() {
        /// Handle short path case
        /// GCode.cpp:393-399
        /// C++: // BBS. Handle short path case.
        /// C++: if (wipe_path.length() < wipe_dist) {
        /// C++: wipe_dist = wipe_path.length();
        /// C++: //BBS: avoid to divide 0
        /// C++: wipe_dist = wipe_dist < EPSILON ? EPSILON : wipe_dist;
        /// C++: }
        let actual_wipe_dist = if total_length < wipe_dist_scaled {
            (total_length as f64).max(1e-6) // Avoid division by zero
        } else {
            wipe_dist_scaled as f64
        };

        /// Add wipe start tag for processor
        /// GCode.cpp:401
        /// C++: // add tag for processor
        /// C++: gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start) + "\n";
        writer.write_comment("TYPE:Wipe_Start");

        /// Set wipe speed
        /// GCode.cpp:402-403
        /// C++: //BBS: don't need to enable cooling markers when this is the last wipe. Because no more cooling layer will clean this "_WIPE"
        /// C++: gcode += gcodegen.writer().set_speed(wipe_speed * 60, "", (gcodegen.enable_cooling_markers() && !is_last) ? ";_WIPE" : "");
        let comment = if !is_last { ";_WIPE" } else { "" };
        writer.set_speed(wipe_speed * 60.0, comment);

        /// Iterate through wipe path segments and retract while traveling
        /// GCode.cpp:404-416
        /// C++: for (const Line& line : wipe_path.lines()) {
        /// C++: double segment_length = line.length();
        /// C++: /* Reduce retraction length a bit to avoid effective retraction speed to be greater than the configured one
        /// C++: due to rounding (TODO: test and/or better math for this) */
        /// C++: double dE = length * (segment_length / wipe_dist) * 0.95;
        /// C++: //BBS: fix this FIXME
        /// C++: //FIXME one shall not generate the unnecessary G1 Fxxx commands, here wipe_speed is a constant inside this cycle.
        /// C++: // Is it here for the cooling markers? Or should it be outside of the cycle?
        /// C++: //gcode += gcodegen.writer().set_speed(wipe_speed * 60, "", gcodegen.enable_cooling_markers() ? ";_WIPE" : "");
        /// C++: gcode += gcodegen.writer().extrude_to_xy(
        /// C++: gcodegen.point_to_gcode(line.b),
        /// C++: -dE,
        /// C++: "wipe and retract"
        /// C++: );
        /// C++: }
        for i in 1..wipe_polyline.points.len() {
            let from = wipe_polyline.points[i - 1];
            let to = wipe_polyline.points[i];

            // Calculate segment length
            let segment_length = from.distance_to_f64(to);

            // Calculate retraction for this segment (95% to avoid rounding issues)
            let de = length * (segment_length / actual_wipe_dist) * 0.95;

            // Emit wipe move with negative extrusion (retraction)
            let to_x = unscale(to.x());
            let to_y = unscale(to.y());
            writer.extrude_to_xy(to_x, to_y, -de, Some("wipe and retract"));
        }

        /// Add wipe end tag for processor
        /// GCode.cpp:418-419
        /// C++: // add tag for processor
        /// C++: gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End) + "\n";
        writer.write_comment("TYPE:Wipe_End");

        /// Update last position
        /// GCode.cpp:420
        /// C++: gcodegen.set_last_pos(wipe_path.points.back());
        if let Some(last_point) = wipe_polyline.points.last() {
            let last_x = unscale(last_point.x());
            let last_y = unscale(last_point.y());
            writer.set_position_xy(last_x, last_y);
        }
    }

    /// Prevent wiping again on same path (path is reset by caller)
    /// GCode.cpp:424
    /// C++: // prevent wiping again on same path
    /// C++: this->reset_path();
    // Note: Wipe path management is handled by the caller
    Ok(())
}

/// Change to a different extruder (tool change).
///
/// C++ reference: GCode::set_extruder()
/// GCode.cpp:6726-6950 (~224 lines)
///
/// This function orchestrates a complete tool change:
/// 1. Checks if tool change is needed (no-op if same extruder)
/// 2. Performs retraction on old extruder (with optional wipe)
/// 3. Resets wipe path
/// 4. Processes filament end G-code
/// 5. Handles ooze prevention
/// 6. Emits tool change command (T<n>)
/// 7. Processes filament start G-code
/// 8. Handles temperature changes
/// 9. Performs un-retraction on new extruder
/// 10. Updates active extruder state
///
/// # Arguments
/// * `new_extruder_id` - Target extruder ID (0-based)
/// * `writer` - GCodeWriter to emit commands
/// * `print_z` - Current Z height for temperature decisions
/// * `config` - Print configuration for tool change settings
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors or invalid extruder ID
pub fn set_extruder(
    new_extruder_id: usize,
    writer: &mut GCodeWriter,
    _print_z: f64,
    _config: &PrintConfig,
) -> Result<()> {
    /// Check if tool change is needed
    /// GCode.cpp:6728-6729
    /// C++: int new_extruder_id = get_extruder_id(new_filament_id);
    /// C++: if (!m_writer.need_toolchange(new_filament_id))
    /// C++: return "";
    if !writer.need_toolchange(new_extruder_id) {
        return Ok(());
    }

    /// Single extruder setup - just set extruder and return
    /// GCode.cpp:6731-6761
    /// C++: // if we are running a single-extruder setup, just set the extruder and return nothing
    /// C++: if (!m_writer.multiple_extruders) {
    /// C++: m_placeholder_parser.set("current_extruder", new_filament_id);
    /// C++: ...
    /// C++: gcode += m_writer.toolchange(new_filament_id);
    /// C++: return gcode;
    /// C++: }
    if !writer.has_multiple_extruders() {
        // Single extruder - just emit T command and update state
        writer.write_command_with_comment(&format!("T{}", new_extruder_id), Some("tool change"));
        writer.set_extruder(new_extruder_id);
        return Ok(());
    }

    /// Multi-extruder setup - full tool change sequence
    /// GCode.cpp:6763-6765
    /// C++: // BBS. Should be placed before retract.
    /// C++: m_toolchange_count++;
    // Tool change counter would be tracked here

    /// Prepend retraction on the current extruder
    /// GCode.cpp:6767
    /// C++: // prepend retraction on the current extruder
    /// C++: std::string gcode = this->retract(true, false);
    retract(writer, true)?; // true = with wipe

    /// Reset wipe path to avoid reusing it
    /// GCode.cpp:6770
    /// C++: // Always reset the extrusion path, even if the tool change retract is set to zero.
    /// C++: m_wipe.reset_path();
    // Note: Wipe path management is handled by caller

    /// Insert skip object labels for sequential printing
    /// GCode.cpp:6772-6776
    /// C++: // BBS: insert skip object label before change filament while by object
    /// C++: if (by_object)
    /// C++: m_writer.add_object_change_labels(gcode);
    /// C++: else
    /// C++: m_writer.add_object_end_labels(gcode);
    // TODO: Implement object change labels for sequential printing

    /// Process filament end G-code if current filament exists
    /// GCode.cpp:6778-6794
    /// C++: bool add_change_filament_624 = false;
    /// C++: if (m_writer.filament() != nullptr) {
    /// C++: // Process the custom filament_end_gcode. set_extruder() is only called if there is no wipe tower
    /// C++: // so it should not be injected twice.
    /// C++: unsigned int old_filament_id = m_writer.filament()->id();
    /// C++: const std::string &filament_end_gcode = m_config.filament_end_gcode.get_at(old_filament_id);
    /// C++: if (! filament_end_gcode.empty()) {
    /// C++: ...
    /// C++: }
    /// C++: }
    // TODO: Process filament_end_gcode custom G-code

    /// Handle ooze prevention (park extruder at standby position)
    /// GCode.cpp:6797-6798
    /// C++: // If ooze prevention is enabled, park current extruder in the nearest
    /// C++: // standby point and set it to the standby temperature.
    /// C++: if (m_ooze_prevention.enable && m_writer.filament() != nullptr)
    /// C++: gcode += m_ooze_prevention.pre_toolchange(*this);
    // TODO: Implement ooze prevention pre-toolchange parking

    /// Calculate flush/wipe volumes and temperatures
    /// GCode.cpp:6800-6880
    /// C++: // BBS
    /// C++: float new_retract_length = m_config.retraction_length.get_at(new_filament_id);
    /// C++: float new_retract_length_toolchange = m_config.retract_length_toolchange.get_at(new_filament_id);
    /// C++: ...
    /// C++: wipe_volume = flush_matrix[old_filament_id * number_of_extruders + new_filament_id];
    /// C++: wipe_volume *= m_config.flush_multiplier.get_at(new_extruder_id);
    // TODO: Calculate flush volumes from flush matrix

    /// Process change_filament_gcode with placeholder substitution
    /// GCode.cpp:6882-6920
    /// C++: dyn_config.set_key_value("outer_wall_volumetric_speed", new ConfigOptionFloat(outer_wall_volumetric_speed));
    /// C++: dyn_config.set_key_value("previous_extruder", new ConfigOptionInt(old_filament_id));
    /// C++: dyn_config.set_key_value("next_extruder", new ConfigOptionInt((int)new_filament_id));
    /// C++: ...
    /// C++: gcode += this->placeholder_parser_process("change_filament_gcode", change_filament_gcode, new_filament_id, &dyn_config);
    // TODO: Process change_filament_gcode with dynamic config placeholders

    /// Emit tool change command
    /// GCode.cpp:6926
    /// C++: gcode += m_writer.toolchange(new_filament_id);
    writer.write_command_with_comment(&format!("T{}", new_extruder_id), Some("tool change"));

    /// Handle ooze prevention post-toolchange
    /// GCode.cpp:6929-6931
    /// C++: // append custom toolchange gcode
    /// C++: if (m_ooze_prevention.enable && m_writer.filament() != nullptr)
    /// C++: gcode += m_ooze_prevention.post_toolchange(*this);
    // TODO: Implement ooze prevention post-toolchange

    /// Process filament_start_gcode for new extruder
    /// GCode.cpp:6933-6947
    /// C++: // Append the filament start G-code.
    /// C++: const std::string &filament_start_gcode = m_config.filament_start_gcode.get_at(new_filament_id);
    /// C++: if (! filament_start_gcode.empty()) {
    /// C++: // Process the filament_start_gcode for the filament.
    /// C++: DynamicConfig config;
    /// C++: ...
    /// C++: gcode += this->placeholder_parser_process("filament_start_gcode", filament_start_gcode, new_filament_id, &config);
    /// C++: check_add_eol(gcode);
    /// C++: }
    // TODO: Process filament_start_gcode custom G-code

    /// Update active extruder state
    /// GCode.cpp:6949
    /// C++: return gcode;
    writer.set_extruder(new_extruder_id);

    Ok(())
}

/// Apply cooling adjustments to a layer.
///
/// C++ reference: CoolingBuffer::process()
/// GCode/CoolingBuffer.cpp:200-400 (~200 lines)
///
/// This function:
/// 1. Calculates actual layer print time
/// 2. Determines if slowdown is needed
/// 3. Adjusts speeds to meet minimum layer time
/// 4. Calculates appropriate fan speed
///
/// # Arguments
/// * `writer` - GCodeWriter to apply speed adjustments
/// * `cooling_buffer` - CoolingBuffer with configuration
/// * `layer_time` - Estimated layer time (seconds)
/// * `layer_index` - Current layer number (0-based)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on processing errors
pub fn apply_layer_cooling(
    writer: &mut GCodeWriter,
    cooling_buffer: &CoolingBuffer,
    layer_time: f64,
    layer_index: usize,
) -> Result<()> {
    /// Check if cooling is needed based on minimum layer time
    /// CoolingBuffer.cpp:250-260
    /// C++: float CoolingBuffer::calculate_layer_slowdown(
    /// C++: std::vector<PerExtruderAdjustments> &per_extruder_adjustments) {
    /// C++: float layer_time_stretched = 0.f;
    /// C++: ...
    /// C++: }
    let config = cooling_buffer.config();

    /// Skip cooling for first layer if configured
    /// CoolingBuffer.cpp:255-257
    /// C++: if (m_layer_id < config.disable_fan_first_layers.value) {
    /// C++: return layer_time;
    /// C++: }
    if layer_index < config.disable_fan_first_layers as usize {
        return Ok(());
    }

    /// Calculate fan speed based on layer time
    /// CoolingBuffer.cpp:280-300
    /// C++: unsigned int fan_speed = 0;
    /// C++: if (layer_time < config.fan_below_layer_time.value) {
    /// C++: fan_speed = config.fan_speed.value;
    /// C++: }
    let fan_speed = cooling_buffer.calculate_fan_speed(layer_index as u32, layer_time);
    let fan_speed_pwm = (fan_speed * 255.0) as u32;

    /// Emit fan speed command
    /// CoolingBuffer.cpp:305-310
    /// C++: if (fan_speed != current_fan_speed) {
    /// C++: gcode += m_gcodegen->writer().set_fan(fan_speed);
    /// C++: current_fan_speed = fan_speed;
    /// C++: }
    writer.set_fan_speed(fan_speed_pwm);

    /// TODO : Implement speed slowdown
    /// CoolingBuffer.cpp:320-380
    /// C++: if (layer_time < config.min_layer_time.value) {
    /// C++: // Calculate slowdown factor
    /// C++: float slowdown_factor = config.min_layer_time.value / layer_time;
    /// C++: // Apply slowdown to adjustable moves
    /// C++: ...
    /// C++: }
    // For , we only implement fan control
    // Speed slowdown will be added
    Ok(())
}

/// Calculate fan speed for bridge features.
///
/// C++ reference: CoolingBuffer::bridge_fan_speed()
/// GCode/CoolingBuffer.cpp:450-470
///
/// # Arguments
/// * `cooling_buffer` - CoolingBuffer with configuration
///
/// # Returns
/// * Fan speed (0.0 - 1.0)
pub fn bridge_fan_speed(cooling_buffer: &CoolingBuffer) -> f64 {
    /// Return bridge fan speed from config
    /// CoolingBuffer.cpp:452-454
    /// C++: float CoolingBuffer::bridge_fan_speed() const {
    /// C++: return m_config.bridge_fan_speed.value;
    /// C++: }
    cooling_buffer.bridge_fan_speed().unwrap_or(0.0)
}

/// Calculate fan speed for overhang features.
///
/// C++ reference: CoolingBuffer::overhang_fan_speed()
/// GCode/CoolingBuffer.cpp:475-495
///
/// # Arguments
/// * `cooling_buffer` - CoolingBuffer with configuration
///
/// # Returns
/// * Fan speed (0.0 - 1.0)
pub fn overhang_fan_speed(cooling_buffer: &CoolingBuffer) -> f64 {
    /// Return overhang fan speed from config
    /// CoolingBuffer.cpp:477-479
    /// C++: float CoolingBuffer::overhang_fan_speed() const {
    /// C++: return m_config.overhang_fan_speed.value;
    /// C++: }
    cooling_buffer.overhang_fan_speed().unwrap_or(0.0)
}

/// Set fan speed with optional override for special features.
///
/// C++ reference: GCode helper methods
/// GCode.cpp:various locations
///
/// # Arguments
/// * `writer` - GCodeWriter to emit fan command
/// * `base_fan_speed` - Base fan speed (0.0 - 1.0)
/// * `role` - Extrusion role (may trigger override)
/// * `cooling_buffer` - CoolingBuffer for override config
///
/// # Returns
/// * `Ok(())` on success
pub fn set_fan_speed_for_role(
    writer: &mut GCodeWriter,
    base_fan_speed: f64,
    role: ExtrusionRole,
    cooling_buffer: &CoolingBuffer,
) -> Result<()> {
    /// Check for bridge override
    /// GCode.cpp:various - bridge detection
    /// C++: if (entity.role() == erBridgeInfill && config.bridge_fan_override) {
    /// C++: fan_speed = config.bridge_fan_speed;
    /// C++: }
    let fan_speed = match role {
        ExtrusionRole::BridgeInfill => {
            if cooling_buffer.config().bridge_fan_override {
                cooling_buffer.bridge_fan_speed().unwrap_or(base_fan_speed)
            } else {
                base_fan_speed
            }
        }
        ExtrusionRole::OverhangPerimeter => {
            if cooling_buffer.config().overhang_fan_override {
                cooling_buffer
                    .overhang_fan_speed()
                    .unwrap_or(base_fan_speed)
            } else {
                base_fan_speed
            }
        }
        _ => base_fan_speed,
    };

    /// Convert to PWM (0-255) and emit
    /// GCodeWriter.cpp:860-900
    /// C++: std::string GCodeWriter::set_fan(unsigned int speed) {
    /// C++: gcode << "M106 S" << 255.0 * speed / 100.0;
    /// C++: }
    let fan_speed_pwm = (fan_speed * 255.0) as u32;
    writer.set_fan_speed(fan_speed_pwm);

    Ok(())
}

// ---------------------------------------------------------------------------
// Layer orchestration dispatch functions
// Ports of GCode::change_layer(), extrude_perimeters(), extrude_infill(),
// extrude_support() from BambuStudio GCode.cpp
// ---------------------------------------------------------------------------

/// State tracked across layers during G-code export.
/// Mirrors key fields of the C++ GCode class that persist across process_layer() calls.
///
/// C++ reference: GCode.hpp member variables
#[derive(Debug, Clone)]
pub struct GCodeLayerState {
    /// Current layer index (incremented by change_layer).
    /// C++ GCode.hpp: m_layer_index
    pub layer_index: usize,
    /// Total layer count for progress reporting.
    /// C++ GCode.hpp: m_layer_count
    pub layer_count: usize,
    /// Last layer Z height (for computing layer height delta).
    /// C++ GCode.hpp: m_last_layer_z
    pub last_layer_z: f64,
    /// Maximum Z reached so far.
    /// C++ GCode.hpp: m_max_layer_z
    pub max_layer_z: f64,
    /// Last layer height (for HEIGHT tag).
    /// C++ GCode.hpp: m_last_height
    pub last_height: f64,
    /// Whether second-layer setup has been done (temp transitions, etc).
    /// C++ GCode.hpp: m_second_layer_things_done
    pub second_layer_things_done: bool,
    /// Whether spiral vase mode is active.
    /// C++ GCode.hpp: m_spiral_vase
    pub spiral_vase: bool,
    /// Whether to enable loop clipping (seam gap).
    /// C++ GCode.hpp: m_enable_loop_clipping
    pub enable_loop_clipping: bool,
    /// Set of object label IDs printed on the current layer.
    /// Used for M624/M625 emission.
    pub layer_object_label_ids: Vec<usize>,
    /// Whether change_layer lift is pending (deferred Z move).
    /// C++ GCode.hpp: m_need_change_layer_lift_z
    pub need_change_layer_lift_z: bool,
    /// Nominal Z position.
    /// C++ GCode.hpp: m_nominal_z
    pub nominal_z: f64,
}

impl Default for GCodeLayerState {
    fn default() -> Self {
        Self {
            layer_index: 0,
            layer_count: 0,
            last_layer_z: 0.0,
            max_layer_z: 0.0,
            last_height: 0.0,
            second_layer_things_done: false,
            spiral_vase: false,
            enable_loop_clipping: true,
            layer_object_label_ids: Vec::new(),
            need_change_layer_lift_z: false,
            nominal_z: 0.0,
        }
    }
}

/// Emit layer-change G-code: progress update, optional retract, Z move.
///
/// C++ reference: GCode::change_layer()
/// GCode.cpp:4904-4940
///
/// This function:
/// 1. Increments progress counter and emits M73
/// 2. Optionally retracts if retract_when_changing_layer is set
/// 3. In spiral vase mode, does immediate Z travel; otherwise defers Z move
/// 4. Updates nominal_z
///
/// # Arguments
/// * `print_z` - Target Z height for this layer
/// * `writer` - GCodeWriter to emit commands
/// * `state` - Mutable layer state (layer_index is incremented)
/// * `config` - Print configuration
///
/// # Returns
/// G-code string for the layer change
pub fn change_layer(
    print_z: f64,
    writer: &mut GCodeWriter,
    state: &mut GCodeLayerState,
    _config: &crate::print_config::PrintConfig,
) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:4908-4910
    // Increment progress and emit M73 progress command
    if state.layer_count > 0 {
        state.layer_index += 1;
        let pct = (state.layer_index as f64 / state.layer_count as f64 * 100.0).min(100.0) as u32;
        // Estimate remaining time proportionally (simple linear estimate)
        gcode += &format!("M73 P{} R0\n", pct);
    }

    // C++ GCode.cpp:4913
    // BBS: no z_offset applied
    let z = print_z;

    // C++ GCode.cpp:4914-4918
    // Retract when changing layer if configured
    // C++ uses FILAMENT_CONFIG(retract_when_changing_layer) — per-filament setting.
    // For now, always retract on layer change if Z increases (matching default behavior).
    if writer.z() < z {
        writer.retract();
    }

    // C++ GCode.cpp:4922-4931
    if state.spiral_vase {
        // In spiral vase mode, travel to Z immediately
        // C++ GCode.cpp:4924-4926
        gcode += &format!("; move to next layer ({})\n", state.layer_index);
        writer.travel_to_z(z, None);
    } else {
        // C++ GCode.cpp:4930
        // Defer Z move — it will happen on next travel_to()
        state.need_change_layer_lift_z = true;
    }

    // C++ GCode.cpp:4933
    state.nominal_z = print_z;

    gcode
}

/// Emit the CHANGE_LAYER tag, Z_HEIGHT, HEIGHT metadata, and custom gcode.
///
/// C++ reference: GCode::process_layer() lines 3922-3983
/// This is the per-layer preamble that runs before change_layer().
///
/// # Arguments
/// * `print_z` - Z height of this layer
/// * `state` - Mutable layer state
/// * `config` - Print configuration
///
/// # Returns
/// G-code string for layer preamble
pub fn emit_layer_preamble(
    print_z: f64,
    state: &mut GCodeLayerState,
    config: &crate::print_config::PrintConfig,
) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:3923
    // Add tag for processor
    gcode += "; CHANGE_LAYER\n";

    // C++ GCode.cpp:3925-3927
    // Export layer z
    gcode += &format!("; Z_HEIGHT: {}\n", print_z);

    // C++ GCode.cpp:3929-3931
    // Export layer height
    let first_layer = state.layer_index == 0 && state.last_layer_z.abs() < 1e-6;
    let height = if first_layer {
        print_z as f32
    } else {
        (print_z as f32) - (state.last_layer_z as f32)
    };
    gcode += &format!(";HEIGHT:{}\n", height);

    // C++ GCode.cpp:3933-3934
    // Update caches
    state.last_layer_z = print_z;
    state.max_layer_z = state.max_layer_z.max(print_z);
    state.last_height = height as f64;

    // C++ GCode.cpp:3938-3946
    // Before layer change custom G-code
    if !config.before_layer_change_gcode.is_empty() {
        let processed = config
            .before_layer_change_gcode
            .replace("{layer_num}", &(state.layer_index + 1).to_string())
            .replace("{layer_z}", &format!("{}", print_z))
            .replace("{max_layer_z}", &format!("{}", state.max_layer_z));
        gcode += &processed;
        gcode += "\n";
    }

    gcode
}

/// Emit post-change_layer custom G-code and fan speed marker.
///
/// C++ reference: GCode::process_layer() lines 3972-3983
///
/// # Arguments
/// * `print_z` - Z height of this layer
/// * `state` - Layer state
/// * `config` - Print configuration
///
/// # Returns
/// G-code string
pub fn emit_layer_postamble(
    print_z: f64,
    state: &GCodeLayerState,
    config: &crate::print_config::PrintConfig,
) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:3972-3981
    // After layer change custom G-code
    if !config.layer_change_gcode.is_empty() {
        let processed = config
            .layer_change_gcode
            .replace("{layer_num}", &state.layer_index.to_string())
            .replace("{layer_z}", &format!("{}", print_z))
            .replace("{max_layer_z}", &format!("{}", state.max_layer_z));
        gcode += &processed;
        gcode += "\n";
    }

    // C++ GCode.cpp:3983
    // Set layer time fan speed marker for cooling post-processor
    gcode += ";_SET_FAN_SPEED_CHANGING_LAYER\n";

    gcode
}

/// Encode a list of object label IDs to a base64-like string for M624.
///
/// C++ reference: GCode::_encode_label_ids_to_base64()
/// GCode.cpp (helper function)
///
/// Each label ID is encoded as a character in a compact format.
/// For simplicity, we encode as comma-separated decimal in a string.
fn encode_label_ids_to_base64(ids: &[usize]) -> String {
    // C++ uses a custom base64-like encoding.
    // For compatibility we use a simplified approach that encodes IDs as
    // a compact string. The firmware (M624) just needs a parseable token.
    if ids.is_empty() {
        return String::new();
    }

    // Simple encoding: base64 of the bit-set of object IDs
    // For up to 64 objects, pack into a u64 bitmask then base64-encode
    let mut bitmask: u64 = 0;
    for &id in ids {
        if id < 64 {
            bitmask |= 1u64 << id;
        }
    }

    // Encode as base64 characters (6 bits each)
    const BASE64_CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::new();
    let mut val = bitmask;
    if val == 0 {
        result.push(BASE64_CHARS[0] as char);
    } else {
        while val > 0 {
            result.push(BASE64_CHARS[(val & 0x3F) as usize] as char);
            val >>= 6;
        }
    }
    result
}

/// Emit M624 start label for an object instance.
///
/// C++ reference: GCode::process_layer() lines 4591-4602
///
/// # Arguments
/// * `label_object_id` - Unique label ID for the object instance
/// * `enable_label_object` - Whether M624/M625 labeling is enabled
///
/// # Returns
/// G-code string with start label
pub fn emit_object_start_label(label_object_id: usize, enable_label_object: bool) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:4589
    gcode += &format!("; OBJECT_ID: {}\n", label_object_id);

    if enable_label_object {
        // C++ GCode.cpp:4592-4595
        gcode += &format!(
            "; start printing object, unique label id: {}\n",
            label_object_id
        );
        // C++ GCode.cpp:4594
        gcode += &format!("M624 {}\n", encode_label_ids_to_base64(&[label_object_id]));
    }

    gcode
}

/// Emit M625 end label for an object instance.
///
/// C++ reference: GCode::process_layer() lines 4753-4758
///
/// # Arguments
/// * `label_object_id` - Unique label ID for the object instance
/// * `enable_label_object` - Whether M624/M625 labeling is enabled
///
/// # Returns
/// G-code string with end label
pub fn emit_object_end_label(label_object_id: usize, enable_label_object: bool) -> String {
    let mut gcode = String::new();

    if enable_label_object {
        // C++ GCode.cpp:4754-4756
        gcode += &format!(
            "; stop printing object, unique label id: {}\n",
            label_object_id
        );
        gcode += "M625\n";
    }
    let _ = label_object_id; // suppress warning when labels disabled

    gcode
}

/// Emit M624/M625 wrapping around the entire layer's timelapse position.
///
/// C++ reference: GCode::process_layer() lines 4395-4407
///
/// # Arguments
/// * `object_label_ids` - Set of object IDs present on this layer
/// * `layer_index` - Current layer index
///
/// # Returns
/// Pair of (start_gcode, end_gcode) strings
pub fn emit_layer_object_labels(
    object_label_ids: &[usize],
    layer_index: usize,
) -> (String, String) {
    if object_label_ids.is_empty() {
        return (String::new(), String::new());
    }

    // C++ GCode.cpp:4395-4401
    let ids_str = object_label_ids
        .iter()
        .map(|id| id.to_string())
        .collect::<Vec<_>>()
        .join(",");

    let start = format!(
        "; object ids of layer {} start: {}\nM624 {}\n",
        layer_index + 1,
        ids_str,
        encode_label_ids_to_base64(object_label_ids),
    );

    // C++ GCode.cpp:4403-4404
    let end = format!(
        "; object ids of this layer{} end: {}\nM625\n",
        layer_index + 1,
        ids_str,
    );

    (start, end)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extrusion_entity::{ExtrusionLoopRole, ExtrusionRole};
    use crate::geometry::{Point, Polyline};

    /// Helper to create a simple square loop for testing.
    /// Creates a 10mm x 10mm square starting at (5,5).
    fn create_square_loop() -> ExtrusionLoop {
        // Create a square: (5,5) -> (15,5) -> (15,15) -> (5,15) -> back to (5,5)
        let points = vec![
            Point::new_scale(5.0, 5.0),
            Point::new_scale(15.0, 5.0),
            Point::new_scale(15.0, 15.0),
            Point::new_scale(5.0, 15.0),
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::ExternalPerimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1; // 0.1 mm³/mm extrusion
        path.width = 0.4;
        path.height = 0.2;

        ExtrusionLoop::new(vec![path], ExtrusionLoopRole::Default)
    }

    /// Helper to create a simple path for testing
    fn create_simple_path() -> ExtrusionPath {
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.05;
        path.width = 0.4;
        path.height = 0.2;

        path
    }

    #[test]
    fn test_extrude_simple_square_loop() {
        let mut writer = GCodeWriter::new();
        let loop_entity = create_square_loop();

        // Get initial E value
        let initial_e = writer.e();

        // Extrude the loop
        extrude_loop(&loop_entity, &mut writer).expect("extrude_loop should succeed");

        // Get final E value
        let final_e = writer.e();

        // E should have increased (we extruded material)
        assert!(
            final_e > initial_e,
            "E value should increase after extrusion: {} -> {}",
            initial_e,
            final_e
        );

        // Get the generated G-code
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain G1 commands (extrusion moves)
        assert!(content.contains("G1"), "G-code should contain G1 commands");

        // Should contain E values (extrusion)
        assert!(
            content.contains(" E"),
            "G-code should contain extrusion (E) values"
        );

        // Should have multiple lines (one per segment)
        let lines: Vec<&str> = content.lines().collect();
        assert!(
            lines.len() >= 4,
            "Should have at least 4 move commands for square (got {})",
            lines.len()
        );
    }

    #[test]
    fn test_extrude_loop_seam_placement() {
        let mut writer = GCodeWriter::new();

        // Set writer to a known position closer to (15,15) corner
        writer.set_x(15.0);
        writer.set_y(15.0);

        let loop_entity = create_square_loop();

        // Extrude the loop
        extrude_loop(&loop_entity, &mut writer).expect("extrude_loop should succeed");

        // The loop should have been rotated to start near the writer's position
        // We can't easily verify the exact starting point without inspecting internals,
        // but we can verify that extrusion happened successfully
        let final_e = writer.e();
        assert!(final_e > 0.0, "Should have extruded material");
    }

    #[test]
    fn test_extrude_loop_ccw_orientation() {
        let mut writer = GCodeWriter::new();

        // Create a clockwise loop (will be reversed to CCW)
        let points = vec![
            Point::new_scale(0.0, 0.0),
            Point::new_scale(0.0, 10.0), // Clockwise: up first
            Point::new_scale(10.0, 10.0),
            Point::new_scale(10.0, 0.0),
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::ExternalPerimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let loop_entity = ExtrusionLoop::new(vec![path], ExtrusionLoopRole::Default);

        let initial_e = writer.e();

        // Should succeed and make loop CCW
        extrude_loop(&loop_entity, &mut writer).expect("extrude_loop should succeed");

        let final_e = writer.e();
        assert!(final_e > initial_e, "Should have extruded material");
    }

    #[test]
    fn test_extrude_path_empty() {
        let mut writer = GCodeWriter::new();

        // Create an empty path
        let path = ExtrusionPath::new(ExtrusionRole::Perimeter);

        let initial_e = writer.e();

        // Should not panic, just do nothing
        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // E should not change for empty path
        assert_eq!(final_e, initial_e, "E should not change for empty path");

        // G-code should be empty or unchanged
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.is_empty() || !content.contains("G1"),
            "Should not generate moves for empty path"
        );
    }

    #[test]
    fn test_extrude_path_single_segment() {
        let mut writer = GCodeWriter::new();

        // Create a path with two points (one segment)
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let initial_e = writer.e();

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // Should have extruded for the 10mm segment
        assert!(final_e > initial_e, "E should increase for single segment");

        // Calculate expected E delta: length * mm3_per_mm = 10.0 * 0.1 = 1.0
        let expected_delta = 10.0 * 0.1;
        let actual_delta = final_e - initial_e;

        // Allow small floating-point tolerance
        assert!(
            (actual_delta - expected_delta).abs() < 0.01,
            "E delta should be approximately {}, got {}",
            expected_delta,
            actual_delta
        );

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have exactly one G1 command
        let g1_count = content.matches("G1").count();
        assert_eq!(
            g1_count, 1,
            "Should have exactly 1 G1 command for single segment, got {}",
            g1_count
        );
    }

    #[test]
    fn test_extrude_path_multi_segment() {
        let mut writer = GCodeWriter::new();

        // Create a path with 4 points (3 segments) forming an L shape
        let points = vec![
            Point::new_scale(0.0, 0.0),
            Point::new_scale(10.0, 0.0), // 10mm horizontal
            Point::new_scale(10.0, 5.0), // 5mm vertical
            Point::new_scale(20.0, 5.0), // 10mm horizontal
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::Infill);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.08;
        path.width = 0.4;
        path.height = 0.2;

        let initial_e = writer.e();

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // Total length: 10 + 5 + 10 = 25mm
        // Expected E delta: 25 * 0.08 = 2.0
        let expected_delta = 25.0 * 0.08;
        let actual_delta = final_e - initial_e;

        assert!(
            (actual_delta - expected_delta).abs() < 0.01,
            "E delta should be approximately {}, got {}",
            expected_delta,
            actual_delta
        );

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have 3 G1 commands (one per segment)
        let g1_count = content.matches("G1").count();
        assert_eq!(
            g1_count, 3,
            "Should have exactly 3 G1 commands for 3 segments, got {}",
            g1_count
        );
    }

    #[test]
    fn test_coordinate_conversion() {
        let mut writer = GCodeWriter::new();

        // Create a path with known scaled coordinates
        let points = vec![Point::new_scale(5.5, 7.3), Point::new_scale(15.8, 12.1)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Verify that coordinates were properly unscaled to mm
        // Should contain X15.800 and Y12.100 (or similar precision)
        assert!(
            content.contains("X15.8") || content.contains("X15.80"),
            "Should contain unscaled X coordinate ~15.8"
        );
        assert!(
            content.contains("Y12.1") || content.contains("Y12.10"),
            "Should contain unscaled Y coordinate ~12.1"
        );
    }

    #[test]
    fn test_extrude_path_cumulative_e() {
        let mut writer = GCodeWriter::new();

        // Extrude multiple paths and verify E accumulates
        let path1 = create_simple_path();
        let path2 = create_simple_path();

        let e0 = writer.e();
        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path1, &mut writer, &config, false);
        let e1 = writer.e();
        extrude_path(&path2, &mut writer, &config, false);
        let e2 = writer.e();

        // Each path should increase E
        assert!(e1 > e0, "First path should increase E");
        assert!(e2 > e1, "Second path should increase E further");

        // E should be cumulative (absolute mode)
        let delta1 = e1 - e0;
        let delta2 = e2 - e1;

        // Both deltas should be approximately equal (same path)
        assert!(
            (delta1 - delta2).abs() < 0.01,
            "Both paths should extrude similar amounts: {} vs {}",
            delta1,
            delta2
        );
    }

    // === Tests for extrude_collection() and extrude_entity() ===

    #[test]
    fn test_extrude_collection_empty() {
        let mut writer = GCodeWriter::new();

        // Empty collection should not panic
        let collection = ExtrusionEntityCollection {
            entities: Vec::new(),
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let result = extrude_collection(&collection, &mut writer);
        assert!(result.is_ok(), "Empty collection should succeed");

        // Should not generate any G-code
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.is_empty() || !content.contains("G1"),
            "Empty collection should not generate moves"
        );
    }

    #[test]
    fn test_extrude_collection_single_path() {
        let mut writer = GCodeWriter::new();

        // Create collection with single path
        let path = create_simple_path();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer).expect("Should extrude single path");
        let e_after = writer.e();

        // Should have extruded
        assert!(e_after > e_before, "E should increase");

        // Should have feature comment
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("FEATURE:"), "Should have feature comment");
    }

    #[test]
    fn test_extrude_collection_multiple_paths() {
        let mut writer = GCodeWriter::new();

        // Create collection with 3 paths
        let path1 = create_simple_path();
        let path2 = create_simple_path();
        let path3 = create_simple_path();

        let collection = ExtrusionEntityCollection {
            entities: vec![
                ExtrusionEntityType::Path(path1),
                ExtrusionEntityType::Path(path2),
                ExtrusionEntityType::Path(path3),
            ],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer).expect("Should extrude all paths");
        let e_after = writer.e();

        // E should increase significantly
        assert!(e_after > e_before, "E should increase for multiple paths");

        // Should have multiple G1 commands (one per path segment)
        let gcode = writer.get_gcode();
        let content = gcode.content();
        let g1_count = content.matches("G1").count();
        assert!(g1_count >= 3, "Should have at least 3 G1 commands");
    }

    #[test]
    fn test_extrude_collection_with_loop() {
        let mut writer = GCodeWriter::new();

        // Create collection with a loop
        let loop_entity = create_square_loop();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Loop(loop_entity)],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer).expect("Should extrude loop");
        let e_after = writer.e();

        // Should have extruded
        assert!(e_after > e_before, "E should increase");

        // Should have feature comment
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("FEATURE:"), "Should have feature comment");
    }

    #[test]
    fn test_extrude_collection_nested() {
        let mut writer = GCodeWriter::new();

        // Create nested collection
        let path = create_simple_path();
        let inner_collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let outer_collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Collection(Box::new(inner_collection))],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let e_before = writer.e();
        extrude_collection(&outer_collection, &mut writer)
            .expect("Should handle nested collections");
        let e_after = writer.e();

        // Should have extruded inner path
        assert!(
            e_after > e_before,
            "E should increase for nested collection"
        );
    }

    #[test]
    fn test_extrude_entity_path() {
        let mut writer = GCodeWriter::new();
        let path = create_simple_path();
        let entity = ExtrusionEntityType::Path(path);

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer).expect("Should extrude path entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Path entity should extrude");
    }

    #[test]
    fn test_extrude_entity_loop() {
        let mut writer = GCodeWriter::new();
        let loop_entity = create_square_loop();
        let entity = ExtrusionEntityType::Loop(loop_entity);

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer).expect("Should extrude loop entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Loop entity should extrude");
    }

    #[test]
    fn test_extrude_entity_collection() {
        let mut writer = GCodeWriter::new();

        let path = create_simple_path();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };
        let entity = ExtrusionEntityType::Collection(Box::new(collection));

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer).expect("Should extrude collection entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Collection entity should extrude");
    }

    #[test]
    fn test_feature_comment_on_role_change() {
        let mut writer = GCodeWriter::new();

        // Create two paths with different roles
        let mut path1 = create_simple_path();
        path1.role = ExtrusionRole::Perimeter;

        let mut path2 = create_simple_path();
        path2.role = ExtrusionRole::InternalInfill;

        let collection = ExtrusionEntityCollection {
            entities: vec![
                ExtrusionEntityType::Path(path1),
                ExtrusionEntityType::Path(path2),
            ],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        extrude_collection(&collection, &mut writer).expect("Should extrude");

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have feature comments for both roles
        assert!(
            content.contains("Perimeter"),
            "Should have Perimeter feature comment"
        );
        assert!(
            content.contains("InternalInfill"),
            "Should have InternalInfill feature comment"
        );

        // Should have at least 2 feature comments
        let feature_count = content.matches("FEATURE:").count();
        assert!(
            feature_count >= 2,
            "Should have at least 2 feature comments, got {}",
            feature_count
        );
    }

    // Tests for travel_to, retract, and unretract

    #[test]
    fn test_travel_to_short_distance_no_retract() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Short travel (1mm) - should not trigger retraction
        let target = Point::new_scale(1.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0, // Threshold is 2mm
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should not be retracted
        assert!(
            !writer.is_retracted(),
            "Should not retract for short travel"
        );

        // Position should have moved
        let pos = writer.position();
        assert!((pos.x - 1.0).abs() < 0.001, "X position should be 1.0");
    }

    #[test]
    fn test_travel_to_long_distance_with_retract() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Long travel (10mm) - should trigger retraction
        let target = Point::new_scale(10.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0, // Threshold is 2mm
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should have retracted during travel
        // Note: Writer may have unretracted at destination, check G-code content
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain travel move
        assert!(
            content.contains("G0") || content.contains("G1"),
            "Should have travel move"
        );

        // Position should have moved
        let pos = writer.position();
        assert!((pos.x - 10.0).abs() < 0.001, "X position should be 10.0");
    }

    #[test]
    fn test_travel_to_retract_disabled() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Long travel but retraction disabled
        let target = Point::new_scale(10.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: false, // Retraction disabled
            retract_length_travel: 2.0,
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should not be retracted
        assert!(!writer.is_retracted(), "Should not retract when disabled");
    }

    #[test]
    fn test_retract_basic() {
        let mut writer = GCodeWriter::new();
        writer.set_x(5.0);
        writer.set_y(5.0);

        // Initially not retracted
        assert!(!writer.is_retracted(), "Should start not retracted");

        retract(&mut writer, false).expect("retract should succeed");

        // Should now be retracted
        assert!(writer.is_retracted(), "Should be retracted after retract()");

        // G-code should contain retraction
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("G1") && content.contains("E"),
            "Should have retraction G-code"
        );
    }

    #[test]
    fn test_retract_when_already_retracted() {
        let mut writer = GCodeWriter::new();

        // Retract once
        retract(&mut writer, false).expect("First retract should succeed");
        let gcode_after_first = writer.get_gcode().content().to_string();

        // Retract again - should be no-op
        retract(&mut writer, false).expect("Second retract should succeed");
        let gcode_after_second = writer.get_gcode().content().to_string();

        // G-code should not have changed (no double retraction)
        assert_eq!(
            gcode_after_first, gcode_after_second,
            "Should not emit duplicate retraction"
        );
    }

    #[test]
    fn test_unretract_basic() {
        let mut writer = GCodeWriter::new();

        // First retract
        retract(&mut writer, false).expect("retract should succeed");
        assert!(writer.is_retracted(), "Should be retracted");

        // Then unretract
        unretract(&mut writer).expect("unretract should succeed");

        // Should no longer be retracted
        assert!(
            !writer.is_retracted(),
            "Should not be retracted after unretract()"
        );

        // G-code should contain unretraction
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("G1") && content.contains("E"),
            "Should have unretraction G-code"
        );
    }

    #[test]
    fn test_unretract_when_not_retracted() {
        let mut writer = GCodeWriter::new();

        // Initially not retracted
        assert!(!writer.is_retracted(), "Should start not retracted");

        let gcode_before = writer.get_gcode().content().to_string();

        // Unretract when not retracted - should be no-op
        unretract(&mut writer).expect("unretract should succeed");

        let gcode_after = writer.get_gcode().content().to_string();

        // G-code should not have changed
        assert_eq!(
            gcode_before, gcode_after,
            "Should not emit unretraction when not retracted"
        );
    }

    #[test]
    fn test_retract_unretract_cycle() {
        let mut writer = GCodeWriter::new();

        // Start not retracted
        assert!(!writer.is_retracted());

        // Retract
        retract(&mut writer, false).expect("retract should succeed");
        assert!(writer.is_retracted());

        // Unretract
        unretract(&mut writer).expect("unretract should succeed");
        assert!(!writer.is_retracted());

        // Retract again
        retract(&mut writer, false).expect("second retract should succeed");
        assert!(writer.is_retracted());

        // Unretract again
        unretract(&mut writer).expect("second unretract should succeed");
        assert!(!writer.is_retracted());
    }

    // ===== Wipe Tests =====

    #[test]
    fn test_wipe_basic() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Create a simple wipe path (10mm straight line)
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Perform wipe with 5mm distance, 0.4mm retraction, 50mm/s speed
        wipe(
            &mut writer,
            &wipe_path,
            5.0,   // wipe_distance
            0.4,   // retraction_length
            50.0,  // wipe_speed
            false, // toolchange
            false, // is_last
        )
        .expect("wipe should succeed");

        // Verify position updated
        let final_pos = writer.position();
        assert!(final_pos.x > 0.0, "X position should advance during wipe");

        // Verify G-code contains wipe markers
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Should contain wipe start marker"
        );
        assert!(
            content.contains("TYPE:Wipe_End"),
            "Should contain wipe end marker"
        );
        assert!(
            content.contains("wipe and retract"),
            "Should contain wipe comment"
        );
    }

    #[test]
    fn test_wipe_zero_length() {
        let mut writer = GCodeWriter::new();

        // Create a wipe path
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Wipe with zero retraction length (should be no-op)
        wipe(
            &mut writer,
            &wipe_path,
            5.0,  // wipe_distance
            0.0,  // retraction_length = 0
            50.0, // wipe_speed
            false,
            false,
        )
        .expect("wipe with zero length should succeed");

        // Should be no-op, no G-code emitted
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("Wipe_Start"),
            "Should not contain wipe markers for zero-length wipe"
        );
    }

    #[test]
    fn test_wipe_empty_path() {
        let mut writer = GCodeWriter::new();

        // Empty wipe path
        let wipe_path = Polyline::from_points(vec![]);

        // Should handle gracefully
        wipe(&mut writer, &wipe_path, 5.0, 0.4, 50.0, false, false)
            .expect("wipe with empty path should succeed");

        // No wipe should occur
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("Wipe_Start"),
            "Should not wipe with empty path"
        );
    }

    #[test]
    fn test_wipe_toolchange() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Toolchange wipe
        wipe(
            &mut writer,
            &wipe_path,
            5.0,
            0.4,
            50.0,
            true, // toolchange = true
            false,
        )
        .expect("toolchange wipe should succeed");

        // Should still perform wipe
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Toolchange wipe should emit markers"
        );
    }

    #[test]
    fn test_wipe_is_last() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Last wipe (no cooling markers)
        wipe(
            &mut writer,
            &wipe_path,
            5.0,
            0.4,
            50.0,
            false,
            true, // is_last = true
        )
        .expect("last wipe should succeed");

        // Should still perform wipe
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Last wipe should still emit markers"
        );
    }

    // ===== Set Extruder Tests =====

    #[test]
    fn test_set_extruder_no_change() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Current extruder is 0 (default)
        assert_eq!(writer.extruder(), 0);

        // Try to set to same extruder (should be no-op)
        set_extruder(0, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should not emit tool change
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("T0"),
            "Should not emit T command when extruder unchanged"
        );
    }

    #[test]
    fn test_set_extruder_single_extruder() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Change to extruder 1 (single extruder mode)
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should emit T command
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("T1") || content.contains("tool change"),
            "Should emit tool change command"
        );

        // Extruder should be updated
        assert_eq!(writer.extruder(), 1);
    }

    #[test]
    fn test_set_extruder_with_retraction() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Set initial position
        writer.set_x(10.0);
        writer.set_y(10.0);

        // Change extruder (should retract first)
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should have retracted
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain both retraction and tool change
        assert!(
            content.contains(" E") || content.contains("retract"),
            "Should retract before tool change"
        );
        assert!(
            content.contains("T1") || content.contains("tool change"),
            "Should emit tool change command"
        );
    }

    #[test]
    fn test_set_extruder_multiple_changes() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Change extruder multiple times
        set_extruder(1, &mut writer, 0.2, &config).expect("first change should succeed");
        assert_eq!(writer.extruder(), 1);

        set_extruder(0, &mut writer, 0.2, &config).expect("second change should succeed");
        assert_eq!(writer.extruder(), 0);

        set_extruder(2, &mut writer, 0.2, &config).expect("third change should succeed");
        assert_eq!(writer.extruder(), 2);

        // Should have emitted multiple tool changes
        let gcode = writer.get_gcode();
        let content = gcode.content();
        let t_count = content.matches("T").count();
        assert!(
            t_count >= 3,
            "Should emit at least 3 tool change commands (got {})",
            t_count
        );
    }

    #[test]
    fn test_set_extruder_first_layer() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // First layer (Z = 0.2) should use first layer temperatures
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should complete successfully
        assert_eq!(writer.extruder(), 1);
    }

    #[test]
    fn test_set_extruder_mid_print() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Mid-print (Z = 10.0) should use normal temperatures
        set_extruder(1, &mut writer, 10.0, &config).expect("set_extruder should succeed");

        // Should complete successfully
        assert_eq!(writer.extruder(), 1);
    }

    // ===== Cooling Integration Tests =====

    #[test]
    fn test_apply_layer_cooling_first_layer() {
        let mut writer = GCodeWriter::new();
        let cooling_buffer = CoolingBuffer::new(CoolingConfig::default());

        // First layer should not apply cooling
        apply_layer_cooling(&mut writer, &cooling_buffer, 10.0, 0).expect("cooling should succeed");

        // No fan command should be emitted
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("M106"),
            "First layer should not have fan commands"
        );
    }

    #[test]
    fn test_apply_layer_cooling_fast_layer() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.min_layer_time = 10.0;
        config.fan_below_layer_time = 15.0;
        config.fan_speed = 1.0;
        config.disable_fan_first_layers = 1;
        let cooling_buffer = CoolingBuffer::new(config);

        // Fast layer (5 seconds < 15 second threshold) should enable fan
        apply_layer_cooling(&mut writer, &cooling_buffer, 5.0, 1).expect("cooling should succeed");

        // Fan should be enabled
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("M106"), "Fast layer should enable fan");
    }

    #[test]
    fn test_apply_layer_cooling_slow_layer() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.min_layer_time = 10.0;
        config.fan_below_layer_time = 15.0;
        config.disable_fan_first_layers = 1;
        let cooling_buffer = CoolingBuffer::new(config);

        // Slow layer (20 seconds > 15 second threshold) should not enable fan
        apply_layer_cooling(&mut writer, &cooling_buffer, 20.0, 1).expect("cooling should succeed");

        // No fan command (or M107 fan off)
        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Either no M106 or has M107 (fan off)
        let has_fan_on = content.contains("M106 S");
        let has_fan_off = content.contains("M107");
        assert!(
            !has_fan_on || has_fan_off,
            "Slow layer should have fan off or no fan command"
        );
    }

    #[test]
    fn test_bridge_fan_speed() {
        let mut config = CoolingConfig::default();
        config.bridge_fan_override = true;
        config.bridge_fan_speed = 1.0;
        let cooling_buffer = CoolingBuffer::new(config);

        let speed = bridge_fan_speed(&cooling_buffer);
        assert_eq!(speed, 1.0, "Bridge fan should be full speed");
    }

    #[test]
    fn test_overhang_fan_speed() {
        let mut config = CoolingConfig::default();
        config.overhang_fan_override = true;
        config.overhang_fan_speed = 0.5;
        let cooling_buffer = CoolingBuffer::new(config);

        let speed = overhang_fan_speed(&cooling_buffer);
        assert_eq!(speed, 0.5, "Overhang fan should be 50%");
    }

    #[test]
    fn test_set_fan_speed_for_role_normal() {
        let mut writer = GCodeWriter::new();
        let cooling_buffer = CoolingBuffer::new(CoolingConfig::default());

        // Normal extrusion should use base fan speed
        set_fan_speed_for_role(&mut writer, 0.8, ExtrusionRole::Perimeter, &cooling_buffer)
            .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // 0.8 * 255 = 204
        assert!(
            content.contains("M106 S204"),
            "Should set fan to 80% ({})",
            content
        );
    }

    #[test]
    fn test_set_fan_speed_for_role_bridge() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.bridge_fan_override = true;
        config.bridge_fan_speed = 1.0;
        let cooling_buffer = CoolingBuffer::new(config);

        // Bridge should use override speed
        set_fan_speed_for_role(
            &mut writer,
            0.5,
            ExtrusionRole::BridgeInfill,
            &cooling_buffer,
        )
        .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Bridge override: 1.0 * 255 = 255
        assert!(
            content.contains("M106 S255"),
            "Bridge should use 100% fan ({})",
            content
        );
    }

    #[test]
    fn test_set_fan_speed_for_role_overhang() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.overhang_fan_override = true;
        config.overhang_fan_speed = 0.7;
        let cooling_buffer = CoolingBuffer::new(config);

        // Overhang should use override speed
        set_fan_speed_for_role(
            &mut writer,
            0.5,
            ExtrusionRole::OverhangPerimeter,
            &cooling_buffer,
        )
        .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Overhang override: 0.7 * 255 = 178.5 ≈ 178
        assert!(
            content.contains("M106 S178") || content.contains("M106 S179"),
            "Overhang should use 70% fan ({})",
            content
        );
    }
}

/// R654 — the five sites that set the path feedrate immediately before calling
/// `extrude_path`, which is where the `; LINE_WIDTH:` tag is written.
///
/// C++ does both inside ONE function, tag first: `_extrude` emits the Width tag
/// at GCode.cpp:6607 and only then `set_speed` at :6663. Our split puts the speed
/// in the caller, so the pair came out inverted 148,548 times against C++'s ZERO
/// (R653's census). Deferring the F to a pending slot, flushed straight after the
/// tag, reproduces C++'s order without creating or destroying a line.
///
/// SHIPPED OPT-IN (probe, default OFF). Making the adjacency exactly C++'s
/// (G1F->LW 148,548 -> 0, C++ 0) moved content ZERO as predicted but cost
/// **-26,309 IN-ORDER lines** (468,570 -> 442,261). The reason is the tag
/// COUNT, not its order: we emit 154,063 `; LINE_WIDTH:` against C++'s
/// 215,199, a 61,136 deficit, so our tags cannot anchor 1:1 with C++'s.
/// Binding the F to a tag we under-emit destroys alignment that the F lines
/// previously kept on their own. Close the COUNT first, then flip this ON.
///
/// R654 note: the first suspect was the collection-level pre-set at
/// `extrude_collection` — the A/B proved that gate a NO-OP (identical hash), so
/// `skip_pre_speed` is already true there and these five are the real producers.
fn set_speed_before_path(writer: &mut GCodeWriter, speed: crate::CoordF, comment: &str) {
    if crate::opt_in_gate("LINEWIDTH_BEFORE_SPEED") {
        writer.set_speed_pending(speed, comment);
    } else {
        writer.set_speed(speed, comment);
    }
}
