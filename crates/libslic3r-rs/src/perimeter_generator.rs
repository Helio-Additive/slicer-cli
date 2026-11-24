//! Perimeter generation - Direct port from BambuStudio
//!
//! PerimeterGenerator.cpp
//!
//! This is a LINE-BY-LINE translation of BambuStudio's perimeter generation.
//! NO improvements, NO optimizations, EXACT algorithm only.

use crate::{
    arachne::utils::extrusion_line::{ExtrusionLine as ArachneExtrusionLine, VariableWidthLines},
    arachne::wall_tool_paths::{WallToolPaths, WallToolPathsParams},
    clipper_utils::{
        difference, grow, intersection, offset2, offset2_clib, offset_expolygons, opening, shrink,
        shrink_clib, union_ex, union_polygons_ex, OffsetJoinType,
    },
    extrusion_entity::{
        ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole,
        ExtrusionPath, ExtrusionRole,
    },
    geometry::{ExPolygons, ThickPolyline, ThickPolylines},
    Coord, ExPolygon, Flow, Point, Polygon, Polyline, SCALING_FACTOR,
};
use std::f64::consts::PI;

// FIDELITY-NOTE(F1): all offset/offset2/union/intersection/difference calls below route
// through clipper_utils, which uses the `geo` crate (geo-clipper, fixed scale 1000) rather
// than C++ ClipperLib at coord_t integer precision. This is a cross-cutting geometry-precision
// approximation; per the audit policy it is NOT re-routed per file.
// FIDELITY-NOTE(F2): C++ truncates several intermediate inset/spacing values to coord_t
// (int32) via `coord_t(...)`. Here those values are kept as f64 mm because the offset
// primitives take mm and snap at the geo-clipper scale (F1); local int32 truncation would
// be meaningless against the F1 approximation.

/// R722 — exact-integer fingerprint of one stage of the fill-boundary chain,
/// mirroring the C++ `fbis_dump` injected at the same four statements
/// (PerimeterGenerator.cpp:1395-1428). Vertex count and coordinate sums are
/// exact, so the first stage at which the engines diverge is unambiguous —
/// area and bbox tolerances hid this for nine rounds (R712/R715/R720/R721).
fn noprobe_dump(tag: &str, lid: usize, v: &[ExPolygon]) {
    if !crate::probe_enabled("NOPROBE") {
        return;
    }
    for e in v {
        let mut iv: i64 = e.contour.points.len() as i64;
        let mut ix: i64 = 0;
        let mut iy: i64 = 0;
        for pt in &e.contour.points {
            ix += pt.x();
            iy += pt.y();
        }
        for h in &e.holes {
            iv += h.points.len() as i64;
            for pt in &h.points {
                ix += pt.x();
                iy += pt.y();
            }
        }
        eprintln!(
            "{} lid={} holes={} iv={} ix={} iy={}",
            tag,
            lid,
            e.holes.len(),
            iv,
            ix,
            iy
        );
    }
}

fn fbis_dump(stage: &str, lid: usize, v: &[ExPolygon]) {
    if !crate::probe_enabled("FBIS") {
        return;
    }
    let mut nv = 0usize;
    let mut sx: i64 = 0;
    let mut sy: i64 = 0;
    let mut area = 0.0f64;
    for ex in v {
        area += ex.area();
        nv += ex.contour.points.len();
        for p in ex.contour.points.iter() {
            sx = sx.wrapping_add(p.x() as i64);
            sy = sy.wrapping_add(p.y() as i64);
        }
        for h in ex.holes.iter() {
            nv += h.points.len();
            for p in h.points.iter() {
                sx = sx.wrapping_add(p.x() as i64);
                sy = sy.wrapping_add(p.y() as i64);
            }
        }
    }
    eprintln!(
        "[FBIS] stage={} lid={} n={} v={} sx={} sy={} area={:.0}",
        stage,
        lid,
        v.len(),
        nv,
        sx,
        sy,
        area
    );
    // `FBISL=<layer>`: every point of that layer's stages, `[FBISP] <stage> c|h x y`.
    if std::env::var("FBISL").ok().and_then(|l| l.parse::<usize>().ok()) == Some(lid) {
        for ex in v {
            for p in ex.contour.points.iter() {
                eprintln!("[FBISP] {} c {} {}", stage, p.x(), p.y());
            }
            for h in ex.holes.iter() {
                for p in h.points.iter() {
                    eprintln!("[FBISP] {} h {} {}", stage, p.x(), p.y());
                }
            }
        }
    }
}

/// Overlap tolerance for perimeter insets
/// libslic3r.h:72
/// C++: static constexpr double INSET_OVERLAP_TOLERANCE = 0.4;
const INSET_OVERLAP_TOLERANCE: f64 = 0.4;

/// Overlap tolerance for smaller external perimeter insets
/// PerimeterGenerator.cpp:28
/// C++: static constexpr double SMALLER_EXT_INSET_OVERLAP_TOLERANCE = 0.22;
const SMALLER_EXT_INSET_OVERLAP_TOLERANCE: f64 = 0.22;

/// Narrow loop length threshold in mm
/// PerimeterGenerator.cpp:19
/// C++: static const double narrow_loop_length_threshold = 10;
const NARROW_LOOP_LENGTH_THRESHOLD: f64 = 10.0;

/// Safety limit to prevent infinite loops
const MAX_PERIMETER_ITERATIONS: usize = 1000;

/// Gate for the faithful only_one_wall_top + infill-boundary-inset path
/// (PerimeterGenerator.cpp:925-926, 1116-1183, 1378-1413).
/// `TOP_FILLS=0` forces the legacy divergent path, any other value forces the
/// faithful path; unset uses the compiled default.
fn top_fills_gate() -> bool {
    // Faithful path is now the default: it matches C++ where
    // top_one_wall_type == TopOneWallType::Alltop activates the only_one_wall_top
    // top_fills handling (PerimeterGenerator.cpp:1116-1183), which the
    // surface-classification pipeline relies on so rim Top surfaces land inside
    // fill_expolygons and survive slices_to_fill_surfaces_clipped. `TOP_FILLS=0`
    // still forces the legacy divergent path.
    const DEFAULT_ON: bool = true;
    match std::env::var("TOP_FILLS") {
        Ok(v) => v != "0",
        Err(_) => DEFAULT_ON,
    }
}

/// Wall generator mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum WallGeneratorMode {
    Classic,
    Arachne,
}

impl Default for WallGeneratorMode {
    fn default() -> Self {
        WallGeneratorMode::Classic
    }
}

/// Configuration for perimeter generation
/// Corresponds to members of C++ PerimeterGenerator class
#[derive(Debug, Clone)]
pub struct PerimeterConfig {
    /// Number of perimeters to generate
    pub perimeter_count: usize,

    /// Extrusion widths
    pub perimeter_extrusion_width: f64,
    pub external_perimeter_extrusion_width: f64,
    pub smaller_external_perimeter_width: f64,

    /// Spacing between perimeters
    pub perimeter_spacing: f64,
    pub external_perimeter_spacing: f64,
    pub smaller_external_perimeter_spacing: f64,
    pub external_to_internal_spacing: f64,

    /// Layer height
    pub layer_height: f64,

    /// Flow objects
    pub perimeter_flow: Flow,
    pub ext_perimeter_flow: Flow,
    pub smaller_ext_perimeter_flow: Flow,

    /// Clipper join type
    pub join_type: OffsetJoinType,

    /// Gap fill threshold
    pub gap_fill_threshold: f64,

    /// Sparse infill density as a fraction (0.0–1.0).
    /// C++ PerimeterGenerator reads `this->config->sparse_infill_density.value`
    /// (a percent) only for the `== 0` comparison at PerimeterGenerator.cpp:1185;
    /// the Rust PrintRegionConfig stores the same option (JSON key
    /// `sparse_infill_density`) as the fraction `fill_density`, and the zero
    /// check is equivalent for percent vs fraction.
    pub sparse_infill_density: f64,

    /// Detect thin walls
    pub detect_thin_wall: bool,

    // --- Arachne wall parameters (PerimeterGenerator.cpp:1536-1553) -----------
    // C++ reads these off `object_config` / `print_config` at the point of use;
    // our PerimeterGenerator has no config back-pointer, so `LayerRegion` copies
    // the raw option values across and the arithmetic below stays where C++ has
    // it. The four size options are PERCENTAGES of the smallest nozzle.
    /// PerimeterGenerator.cpp:1536 `min_nozzle_diameter`
    pub min_nozzle_diameter: f64,
    /// `object_config->min_feature_size` (percent)
    pub arachne_min_feature_size: f64,
    /// `object_config->min_bead_width` (percent)
    pub arachne_min_bead_width: f64,
    /// `object_config->wall_transition_length` (percent)
    pub arachne_wall_transition_length: f64,
    /// `object_config->wall_transition_angle` (DEGREES, no scaling)
    pub wall_transition_angle: f64,
    /// `object_config->wall_transition_filter_deviation` (percent)
    pub wall_transition_filter_deviation: f64,
    /// `object_config->wall_distribution_count`
    pub wall_distribution_count: u32,

    /// Surface simplification resolution
    pub surface_simplify_resolution: f64,

    /// Arc fitting enabled
    pub arc_fitting_enabled: bool,

    /// Wall generator mode (Classic or Arachne)
    pub wall_generator_mode: WallGeneratorMode,

    /// Fuzzy skin mode
    pub fuzzy_skin_mode: crate::region_config::FuzzySkinMode,
    /// Fuzzy skin thickness (mm)
    pub fuzzy_skin_thickness: f64,
    /// Fuzzy skin point distance (mm)
    pub fuzzy_skin_point_distance: f64,

    /// Wall sequence (inner/outer ordering).
    /// BambuStudio: `wall_sequence` in PrintRegionConfig.
    pub wall_sequence: crate::print_config::WallSequence,

    /// Whether to detect overhang walls
    /// PerimeterGenerator.cpp:343: detect_overhang_wall
    pub detect_overhang_wall: bool,

    /// Lower layer slices for overhang detection
    /// PerimeterGenerator.cpp: lower_slices
    pub lower_slices: Option<Vec<ExPolygon>>,

    /// Upper layer slices for top-surface detection (only_one_wall_top / top_fills).
    /// PerimeterGenerator.cpp: upper_slices
    pub upper_slices: Option<Vec<ExPolygon>>,

    /// Whether top_one_wall_type == Alltop (BambuStudio default; config key only_one_wall_top).
    /// Gates the top_fills detection at PerimeterGenerator.cpp:1118.
    pub top_one_wall: bool,

    /// Min width of top areas, percent of perimeter line width.
    /// PerimeterConfig top_area_threshold (PrintConfig.cpp:1288, default 200).
    pub top_area_threshold: f64,

    /// Solid infill flow spacing in mm.
    /// PerimeterGenerator.cpp:874 — coord_t solid_infill_spacing = this->solid_infill_flow.scaled_spacing();
    pub solid_infill_spacing: f64,

    /// Sparse infill line width in mm.
    /// PerimeterGenerator.cpp:1167 — double infill_spacing_unscaled = this->config->sparse_infill_line_width.value;
    pub sparse_infill_line_width: f64,

    /// Infill/wall overlap as a fraction (config `infill_wall_overlap`, percent in C++).
    /// PerimeterGenerator.cpp:1392 — infill_wall_overlap.get_abs_value(...).
    pub infill_wall_overlap: f64,

    /// Layer ID (0-based)
    pub layer_id: usize,

    /// PrintConfig z_direction_outwall_speed_continuous — gates outer-wall
    /// NodeContour/LoopNode collection for cross-layer speed smoothing
    /// (PerimeterGenerator.cpp:1074-1100, 1281-1317).
    pub z_direction_outwall_speed_continuous: bool,

    /// Number of raft layers
    pub raft_layers: usize,

    /// Overhang flow (used for fully unsupported segments)
    pub overhang_flow: Option<Flow>,

    // --- What process_classic of the pin reads and the legacy body did not (R921) ---
    /// `config->precise_outer_wall` (PerimeterGenerator.cpp:872)
    pub precise_outer_wall: bool,
    /// `config->alternate_extra_wall` (PerimeterGenerator.cpp:935)
    pub alternate_extra_wall: bool,
    /// `object_config->top_one_wall_type` (PerimeterGenerator.cpp:938, 1131)
    pub top_one_wall_type: crate::print_config::TopOneWallType,
    /// `object_config->only_one_wall_first_layer` (PerimeterGenerator.cpp:938)
    pub only_one_wall_first_layer: bool,
    /// `starts_at_object_base` (PerimeterGenerator.hpp:40, LayerRegion.cpp:148, 191)
    pub starts_at_object_base: bool,
    /// `object_config->brim_type == BrimType::btOuterOnly && object_config->brim_width.value > 0`
    /// (PerimeterGenerator.cpp:1275-1276)
    pub brim_outer_only: bool,
    /// `config->gap_infill_speed.get_at(...)` (PerimeterGenerator.cpp:895)
    pub gap_infill_speed: f64,
    /// `config->filter_out_gap_fill.value` (PerimeterGenerator.cpp:1378), mm
    pub filter_out_gap_fill: f64,
    /// `m_spiral_vase` (PerimeterGenerator.hpp:128)
    pub spiral_vase: bool,
    /// `solid_infill_flow` (PerimeterGenerator.hpp:45)
    pub solid_infill_flow: Option<Flow>,
    /// `config->infill_wall_overlap.value`, the percent (PerimeterGenerator.cpp:1411)
    pub infill_wall_overlap_percent: f64,
    /// `is_enable_overhang_speed(perimeter_generator)` (PerimeterGenerator.cpp:268-276)
    pub enable_overhang_speed: bool,

    // --- What process_arachne and process_no_bridge of the pin read beside the above (R922) ---
    /// `config->outer_wall_line_width` (PerimeterGenerator.cpp:652): the option itself, not the width of
    /// the external perimeter flow (which is the first layer's width on the first layer).
    pub outer_wall_line_width: f64,
    /// `config->counterbore_hole_bridging` (PerimeterGenerator.cpp:1956, 1987, 2041)
    pub counterbore_hole_bridging: crate::print_config::enums::CounterboreHoleBridgingOption,
    /// `config->bridge_angle.value` (PerimeterGenerator.cpp:1982), degrees
    pub bridge_angle: f64,
    /// `slice_z` (PerimeterGenerator.hpp:41, LayerRegion.cpp:193)
    pub slice_z: f64,
}

impl Default for PerimeterConfig {
    fn default() -> Self {
        Self {
            perimeter_count: 2,
            perimeter_extrusion_width: 0.4,
            external_perimeter_extrusion_width: 0.4,
            smaller_external_perimeter_width: 0.4,
            perimeter_spacing: 0.4,
            external_perimeter_spacing: 0.4,
            smaller_external_perimeter_spacing: 0.4,
            external_to_internal_spacing: 0.4,
            layer_height: 0.2,
            perimeter_flow: Flow::new(0.0, 0.0, 0.0).unwrap(),
            ext_perimeter_flow: Flow::new(0.0, 0.0, 0.0).unwrap(),
            smaller_ext_perimeter_flow: Flow::new(0.0, 0.0, 0.0).unwrap(),
            join_type: OffsetJoinType::Miter,
            gap_fill_threshold: 0.0,
            // PrintConfig.cpp: sparse_infill_density default 20% (fraction 0.2).
            sparse_infill_density: 0.2,
            detect_thin_wall: false,
            // C++ option defaults: min_feature_size 25%, min_bead_width 85%,
            // wall_transition_length 100%, wall_transition_angle 10 deg,
            // wall_transition_filter_deviation 25%, wall_distribution_count 1.
            min_nozzle_diameter: 0.4,
            arachne_min_feature_size: 25.0,
            arachne_min_bead_width: 85.0,
            arachne_wall_transition_length: 100.0,
            wall_transition_angle: 10.0,
            wall_transition_filter_deviation: 25.0,
            wall_distribution_count: 1,
            surface_simplify_resolution: 0.01,
            arc_fitting_enabled: false,
            wall_generator_mode: WallGeneratorMode::Classic,
            fuzzy_skin_mode: crate::region_config::FuzzySkinMode::None,
            fuzzy_skin_thickness: 0.3,
            fuzzy_skin_point_distance: 0.8,
            wall_sequence: crate::print_config::WallSequence::InnerOuter,
            detect_overhang_wall: false,
            lower_slices: None,
            upper_slices: None,
            top_one_wall: true,
            top_area_threshold: 200.0,
            solid_infill_spacing: 0.4,
            sparse_infill_line_width: 0.4,
            infill_wall_overlap: 0.15,
            layer_id: 0,
            z_direction_outwall_speed_continuous: false,
            raft_layers: 0,
            overhang_flow: None,
            // PrintConfig.cpp defaults.
            precise_outer_wall: false,
            alternate_extra_wall: false,
            top_one_wall_type: crate::print_config::TopOneWallType::AllTop,
            only_one_wall_first_layer: false,
            starts_at_object_base: false,
            brim_outer_only: false,
            gap_infill_speed: 30.,
            filter_out_gap_fill: 0.,
            spiral_vase: false,
            solid_infill_flow: None,
            infill_wall_overlap_percent: 15.,
            enable_overhang_speed: true,
            outer_wall_line_width: 0.4,
            counterbore_hole_bridging: crate::print_config::enums::CounterboreHoleBridgingOption::ChbNone,
            bridge_angle: 0.,
            slice_z: 0.,
        }
    }
}

/// A single perimeter loop
#[derive(Debug, Clone)]
pub struct PerimeterLoop {
    /// The polygon defining this perimeter
    pub polygon: Polygon,

    /// Is this a contour (outer boundary) or hole?
    pub is_contour: bool,

    /// Is this an external perimeter (depth == 0)?
    pub is_external: bool,

    /// Perimeter index (0 = external, 1+ = internal)
    pub perimeter_index: usize,

    /// Extrusion width for this perimeter
    pub extrusion_width: f64,

    /// Is this using smaller width?
    pub is_smaller_width: bool,

    /// Flow object
    pub flow: Option<Flow>,

    /// Child loops (nested perimeters)
    /// PerimeterGenerator.cpp:51
    /// C++: std::vector<PerimeterGeneratorLoop> children;
    pub children: Vec<PerimeterLoop>,
}

impl PerimeterLoop {
    /// PerimeterGenerator.cpp:47
    /// C++: PerimeterGeneratorLoop(const Polygon &polygon, unsigned short depth, bool is_contour, bool is_small_width_perimeter = false, ...)
    pub fn new(
        polygon: Polygon,
        perimeter_index: usize,
        is_contour: bool,
        is_smaller_width: bool,
        extrusion_width: f64,
    ) -> Self {
        Self {
            polygon,
            is_contour,
            is_external: perimeter_index == 0,
            perimeter_index,
            extrusion_width,
            is_smaller_width,
            flow: None,
            children: Vec::new(),
        }
    }

    /// PerimeterGenerator.cpp:1830-1839
    /// C++: bool PerimeterGeneratorLoop::is_internal_contour() const {
    /// C++:     // An internal contour is a contour containing no other contours
    /// C++:     if (! this->is_contour)
    /// C++:         return false;
    /// C++:     for (const PerimeterGeneratorLoop &loop : this->children)
    /// C++:         if (loop.is_contour)
    /// C++:             return false;
    /// C++:     return true;
    /// C++: }
    pub fn is_internal_contour(&self) -> bool {
        // An internal contour is a contour containing no other contours
        if !self.is_contour {
            return false;
        }
        for loop_item in &self.children {
            if loop_item.is_contour {
                return false;
            }
        }
        true
    }
}

/// R253: captured per-surface inputs for re-running the classic fill-boundary
/// tail after the gap-fill footprint trim (GAPTRIM_PRETAIL).
#[derive(Debug, Clone, Default)]
pub struct ClassicTailInput {
    pub last: ExPolygons,
    pub top_fills: ExPolygons,
    pub fill_clip: ExPolygons,
    pub loop_number: usize,
}

/// Result of perimeter generation
#[derive(Debug, Clone)]
pub struct PerimeterResult {
    /// Generated extrusion entities (loops and paths)
    /// PerimeterGenerator.cpp:150 - writes to ExtrusionEntityCollection* loops
    pub entities: crate::extrusion_entity::ExtrusionEntityCollection,

    /// Remaining infill area
    pub infill_area: ExPolygons,

    /// No-overlap infill area (fill_no_overlap). PerimeterGenerator.cpp:1415-1430
    /// — the infill region computed WITHOUT the infill_peri_overlap grow, used by
    /// FillMonotonicLineWGapFill (top surface) to avoid overflow / over-extrusion.
    pub no_overlap_area: ExPolygons,

    /// Gap fill areas
    pub gap_fills: ExPolygons,

    /// R253 (GAPTRIM_PRETAIL): per-surface inputs to the classic fill-boundary
    /// tail, captured BEFORE the tail ran. Native subtracts the gap-fill
    /// footprint from `last` before the tail (PG.cpp:1373); the layer-level
    /// gap block re-runs the tail on the trimmed `last` using these.
    pub tail_inputs: Vec<ClassicTailInput>,

    /// Per-surface gap-fill areas in surface order (R173): native runs the
    /// gap collapse + medial axis PER SURFACE (PG.cpp:1327 inside the surface
    /// loop) — collapsing the merged set lets nearby islands' gaps interact
    /// through the ±max/2 offsets (L0: 23-vs-48 polylines from identical
    /// inputs). Consumed by the gated layer-level gap block.
    pub gap_fills_per_surface: Vec<ExPolygons>,

    /// The grown top-band (offset_ex(top_infill_exp, infill_peri_overlap),
    /// PerimeterGenerator.cpp:1411) that was unioned into infill_area. Passed
    /// separately so the layer-level gap-footprint trim can restore it: native
    /// subtracts the gap footprint from `last` BEFORE the fill-boundary block
    /// (PG.cpp:1373), so the top_fills-derived band is never trimmed.
    pub top_band: ExPolygons,

    /// Outer-wall LoopNodes for z-direction speed continuity
    /// (PerimeterGenerator.cpp:1281-1317, native pushes into Layer::loop_nodes;
    /// here returned per-generate() call and merged by the Layer). node_id is
    /// LOCAL to this result — the Layer rebases on append.
    pub loop_nodes: Vec<crate::layer::LoopNode>,

    /// `gap_fill` of the pin's generator (PerimeterGenerator.hpp:54, PerimeterGenerator.cpp:1393): the gap
    /// fill extrusions. Filled by [`PerimeterGenerator::process_classic`] only; the legacy body hands
    /// the areas over in `gap_fills` and the layer makes the extrusions.
    pub gap_fill: crate::extrusion_entity::ExtrusionEntityCollection,
    /// The result is the whole of the pin's outputs (`loops`, `gap_fill`, `fill_surfaces`,
    /// `fill_no_overlap`, `loop_nodes`): nothing is left for the layer to do.
    pub pin: bool,
}

impl PerimeterResult {
    pub fn new() -> Self {
        Self {
            entities: crate::extrusion_entity::ExtrusionEntityCollection::new(),
            infill_area: Vec::new(),
            no_overlap_area: Vec::new(),
            gap_fills: Vec::new(),
            gap_fills_per_surface: Vec::new(),
            tail_inputs: Vec::new(),
            top_band: Vec::new(),
            loop_nodes: Vec::new(),
            gap_fill: crate::extrusion_entity::ExtrusionEntityCollection::new(),
            pin: false,
        }
    }
}

/// Perimeter generator
pub struct PerimeterGenerator {
    config: PerimeterConfig,
    /// PerimeterGenerator.hpp:129-133 `double m_ext_mm3_per_mm; double m_mm3_per_mm; double
    /// m_mm3_per_mm_overhang; double m_ext_mm3_per_mm_smaller_width;`: -1 from the constructor
    /// (PerimeterGenerator.hpp:103), set by process_classic and process_arachne.
    m_ext_mm3_per_mm: std::cell::Cell<f64>,
    m_mm3_per_mm: std::cell::Cell<f64>,
    m_mm3_per_mm_overhang: std::cell::Cell<f64>,
    //BBS
    m_ext_mm3_per_mm_smaller_width: std::cell::Cell<f64>,
}

impl PerimeterGenerator {
    pub fn new(config: PerimeterConfig) -> Self {
        Self {
            config,
            m_ext_mm3_per_mm: std::cell::Cell::new(-1.),
            m_mm3_per_mm: std::cell::Cell::new(-1.),
            m_mm3_per_mm_overhang: std::cell::Cell::new(-1.),
            m_ext_mm3_per_mm_smaller_width: std::cell::Cell::new(-1.),
        }
    }

    /// PerimeterGenerator.hpp:113 `double ext_mm3_per_mm() const { return m_ext_mm3_per_mm; }`
    pub fn ext_mm3_per_mm(&self) -> f64 {
        self.m_ext_mm3_per_mm.get()
    }
    /// PerimeterGenerator.hpp:114 `double mm3_per_mm() const { return m_mm3_per_mm; }`
    pub fn mm3_per_mm(&self) -> f64 {
        self.m_mm3_per_mm.get()
    }
    /// PerimeterGenerator.hpp:115 `double mm3_per_mm_overhang() const { return m_mm3_per_mm_overhang; }`
    pub fn mm3_per_mm_overhang(&self) -> f64 {
        self.m_mm3_per_mm_overhang.get()
    }
    //BBS
    /// PerimeterGenerator.hpp:117 `double smaller_width_ext_mm3_per_mm() const { return m_ext_mm3_per_mm_smaller_width; }`
    pub fn smaller_width_ext_mm3_per_mm(&self) -> f64 {
        self.m_ext_mm3_per_mm_smaller_width.get()
    }

    /// PerimeterGenerator.hpp:118 `Polygons lower_slices_polygons() const { return m_lower_slices_polygons; }`:
    /// the member process_arachne sets (PerimeterGenerator.cpp:1509-1515) where there are lower slices
    /// and detect_overhang_wall is on, `offset(*this->lower_slices, float(scale_(+nozzle_diameter / 2)))`,
    /// empty otherwise. It is computed here where it is read.
    pub fn lower_slices_polygons(&self) -> Vec<Polygon> {
        use crate::clipper_utils::OffsetJoinType;
        let Some(lower) = self.config.lower_slices.as_ref() else {
            return Vec::new();
        };
        if !self.config.detect_overhang_wall {
            return Vec::new();
        }
        let nozzle = self.config.perimeter_flow.nozzle_diameter();
        // R803c — native (PerimeterGenerator.cpp:1514):
        //   m_lower_slices_polygons = offset(*lower_slices, float(scale_(+nozzle/2)))
        // is ClipperLib raw_offset + union over ALL rings (contours and
        // holes, orientation-signed), jtMiter/ML3. The geo `grow` route
        // re-gridded the boundary and joined corners differently, so the
        // signed distances behind every arachne overhang degree — and
        // therefore the split positions of the speed pieces — drifted by
        // up to ~0.9mm on straight stretches (layer-8 benchy-arachne).
        if crate::faithful_gate("ARACHNE_LOWER_GROW_CLIB") {
            let rings = expolygons_to_polygons(lower);
            let delta = (crate::scaled_f(0.5 * nozzle) as f32) as f64;
            crate::clipper_utils::offset_polygons_clib(&rings, delta, OffsetJoinType::Miter)
        } else {
            expolygons_to_polygons(&crate::clipper_utils::grow(lower, 0.5 * nozzle, OffsetJoinType::Miter))
        }
    }

    /// The members process_classic (PerimeterGenerator.cpp:862, 867, 878, 903) and process_arachne
    /// (:1492, 1497, 1503; it leaves the smaller width at -1) assign from the flows. `Flow::mm3_per_mm()`
    /// throws for a flow without a positive section at the pin; such a member stays -1 here.
    fn set_mm3_per_mm_members(&self, classic: bool) {
        // other perimeters
        self.m_mm3_per_mm.set(self.config.perimeter_flow.mm3_per_mm().unwrap_or(-1.));
        // external perimeters
        self.m_ext_mm3_per_mm.set(self.config.ext_perimeter_flow.mm3_per_mm().unwrap_or(-1.));
        // overhang perimeters
        self.m_mm3_per_mm_overhang.set(self.config.overhang_flow.as_ref().and_then(|f| f.mm3_per_mm().ok()).unwrap_or(-1.));
        if classic {
            // BBS: this flow is for smaller external perimeter for small area
            self.m_ext_mm3_per_mm_smaller_width.set(self.config.smaller_ext_perimeter_flow.mm3_per_mm().unwrap_or(-1.));
        }
    }

    /// Generate perimeters for given slices
    /// PerimeterGenerator.cpp:847-1428 (process_classic)
    /// C++ PerimeterGenerator::process_classic() iterates over each surface separately.
    /// This function processes all surfaces by calling generate_classic_one per surface.
    pub fn generate(&self, slices: &[ExPolygon]) -> PerimeterResult {
        let mut result = PerimeterResult::new();

        if slices.is_empty() || self.config.perimeter_count == 0 {
            result.infill_area = slices.to_vec();
            return result;
        }

        // Use Arachne if enabled
        if self.config.wall_generator_mode == WallGeneratorMode::Arachne {
            return self.process_arachne_legacy(slices);
        }
        self.process_classic_legacy(slices, result)
    }

    /// The body `process_classic` had up to R920 (the arm of `PG_CLASSIC_PIN=0`): the surfaces in the
    /// order of chain_expolygons, each through `generate_classic_one`, in millimetres and over the
    /// C++ ClipperLib shims; the gap fill extrusions are left to the layer. The port of the pin's
    /// function is [`PerimeterGenerator::process_classic`].
    pub fn process_classic_legacy(&self, slices: &[ExPolygon], mut result: PerimeterResult) -> PerimeterResult {
        self.set_mm3_per_mm_members(true);
        // C++ process_classic() loops: for (const Surface &surface : this->slices->surfaces)
        // Each surface is processed independently with its own last/contours/holes state.
        // R209: native REORDERS the surfaces first ("reorder the surface to
        // reduce the travel time", PerimeterGenerator.cpp:915-921):
        // chain_expolygons = KD-greedy chain over bbox centers. Without it the
        // island emission order diverges and 332 extra Outer->Outer retractions
        // appear (loop-to-loop hops exceed retraction_minimum_travel).
        let order: Vec<usize> = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
            crate::shortest_path::chain_expolygons(&slices.to_vec())
        } else {
            (0..slices.len()).collect()
        };

        // R710 — R709 verified this chain by READING both sides, which cannot
        // catch a faithful call fed a different input. Print the bbox centers
        // `chain_expolygons` keys on and the order it returned, to compare
        // against the same probe in C++ (`PerimeterGenerator.cpp:919`, PGSO).
        if crate::probe_enabled("PGSO") {
            let c: Vec<String> = slices
                .iter()
                .map(|e| {
                    let b = e.bounding_box();
                    format!("{},{}", b.center().x, b.center().y)
                })
                .collect();
            let o: Vec<String> = order.iter().map(|i| i.to_string()).collect();
            eprintln!(
                "[PGSO] layer={} n={} c={} order={}",
                self.config.layer_id,
                slices.len(),
                c.join(";"),
                o.join(",")
            );
        }
        if crate::probe_enabled("PAPROBE") {
            eprintln!(
                "PACALL lid={} nsurf={} mode=classic",
                self.config.layer_id,
                slices.len()
            );
            for surface in slices.iter() {
                let mut iv: i64 = surface.contour.points.len() as i64;
                let mut ix: i64 = 0;
                let mut iy: i64 = 0;
                for pt in &surface.contour.points {
                    ix += pt.x();
                    iy += pt.y();
                }
                for h in &surface.holes {
                    iv += h.points.len() as i64;
                    for pt in &h.points {
                        ix += pt.x();
                        iy += pt.y();
                    }
                }
                eprintln!(
                    "PAPROBE lid={} holes={} iv={} ix={} iy={}",
                    self.config.layer_id,
                    surface.holes.len(),
                    iv,
                    ix,
                    iy
                );
            }
        }
        for &slice_idx in &order {
            let slice = &slices[slice_idx];
            let surface_result = self.generate_classic_one(slice);
            // z-continuity metadata: rebase per-surface node/entity indices onto
            // the merged result (LESSON R141: per-surface merges silently drop
            // new result fields — loop_id is an entity index LOCAL to the
            // surface, node_id LOCAL to the surface's node vec).
            let entity_base = result.entities.entities.len();
            let node_base = result.loop_nodes.len();
            for mut n in surface_result.loop_nodes {
                n.node_id += node_base;
                n.loop_id += entity_base;
                result.loop_nodes.push(n);
            }
            result
                .entities
                .entities
                .extend(surface_result.entities.entities);
            result.infill_area.extend(surface_result.infill_area);
            result.no_overlap_area.extend(surface_result.no_overlap_area);
            if !surface_result.gap_fills.is_empty() {
                result
                    .gap_fills_per_surface
                    .push(surface_result.gap_fills.clone());
            }
            result.gap_fills.extend(surface_result.gap_fills);
            result.top_band.extend(surface_result.top_band);
            result.tail_inputs.extend(surface_result.tail_inputs);
        }
        result.entities.loop_node_range = (0, result.loop_nodes.len());

        result
    }

    /// Generate classic perimeters for a single ExPolygon (surface).
    /// Matches C++ process_classic() inner loop body.
    fn generate_classic_one(&self, slice: &ExPolygon) -> PerimeterResult {
        let mut result = PerimeterResult::new();

        /// PerimeterGenerator.cpp:914
        /// C++: double surface_simplify_resolution = (print_config->enable_arc_fitting &&
        ///          this->config->fuzzy_skin == FuzzySkinType::None) ? 0.2 * m_scaled_resolution : m_scaled_resolution;
        // The 0.2x reduction only applies when arc fitting is enabled AND fuzzy skin is None.
        let surface_simplify_resolution = if self.config.arc_fitting_enabled
            && self.config.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None
        {
            0.2 * self.config.surface_simplify_resolution
        } else {
            self.config.surface_simplify_resolution
        };

        /// PerimeterGenerator.cpp:945
        /// C++: ExPolygons last = union_ex(surface.expolygon.simplify_p(surface_simplify_resolution));
        // Faithful port: Douglas-Peucker simplify the contour + holes (ExPolygon::simplify_p),
        // then union_ex the resulting Polygons back into ExPolygons. `union_polygons_ex` is the
        // Rust equivalent of C++ `union_ex(const Polygons&)`.
        // R100: faithful C++ `last = union_ex(surface.expolygon.simplify_p(res))`.
        // Default path routes the post-DP `simplify_polygons` + `union_ex` through
        // geo-clipper (`simplify_p` -> `union_polygons_ex`), which runs at
        // GEO_CLIPPER_SCALE=1000 (1 micron) and quantizes every wall-input vertex
        // to a 100-unit grid (rust coords 99.4% on-grid vs native 3.6%), diverging
        // all downstream wall geometry, seams and arc fitting from native's 10nm
        // ClipperLib. Under F1_UNION (the gated byte-match path) mirror the R91
        // slice-path chain instead: DP at full resolution, then
        // `ClipperLib::SimplifyPolygons` (StrictlySimple) + `union_ex` via the
        // vertex-exact vendored ClipperLib (clipper-z-sys @ i32/1e5). fill_type=1
        // (pftNonZero) matches C++ simplify_polygons.
        // R122: native runs simplify_p at the SCALED resolution `0.2·m_scaled_resolution`
        // (m_scaled_resolution = scaled<double>(resolution) = resolution/0.00001 ≈ 240 scaled
        // units); rust's `surface_simplify_resolution` is the UNSCALED value (0.0024) — a 1e5×
        // too-small DP tolerance. The default geo path's rounded-projection DP accidentally
        // masks this (it rounds near-collinear projections to 0, removing points the tiny
        // tolerance never would); the faithful double-distance DP does not, so it needs the
        // correctly-scaled tolerance to match native. Gated: default path keeps the (buggy but
        // byte-locked) unscaled value.
        let simplify_tol_scaled = if self.config.arc_fitting_enabled
            && self.config.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None
        {
            0.2 * (self.config.surface_simplify_resolution / 0.00001_f64)
        } else {
            self.config.surface_simplify_resolution / 0.00001_f64
        };
        let mut last = if crate::faithful_gate("F1_UNION") {
            // R122: faithful DP — native pure-double point-to-segment distance AND the
            // native SCALED tolerance (simplify_tol_scaled). Fixes L33/L34/L54 (+sweep).
            let pp = slice.simplify_p_dp_rings_faithful(simplify_tol_scaled);
            let simplified = crate::clipper_utils::simplify_polygons_clib(&pp, 1);
            crate::clipper_utils::union_ex_clib(&simplified, 1)
        } else {
            union_polygons_ex(&slice.simplify_p(surface_simplify_resolution))
        };

        /// PerimeterGenerator.cpp:920
        /// C++: int loop_number = this->config->wall_loops + surface.extra_perimeters - 1;
        /// perimeter_count == wall_loops (a count), so loop_number is 0-based max depth index
        let mut loop_number = self.config.perimeter_count.saturating_sub(1);

        // PerimeterGenerator.cpp:925-926
        // C++: if (loop_number > 0 && ((this->object_config->top_one_wall_type != TopOneWallType::None
        // C++:     && this->upper_slices == nullptr) || (this->object_config->only_one_wall_first_layer && layer_id == 0)))
        // C++:     loop_number = 0;
        // BBS: set the topmost (no upper layer) and bottom most layer to be one wall.
        // (only_one_wall_first_layer defaults to false and is not threaded yet.)
        if loop_number > 0
            && self.config.top_one_wall
            && top_fills_gate()
            && self.config.upper_slices.is_none()
        {
            loop_number = 0;
        }

        /// Calculate spacing values
        /// PerimeterGenerator.cpp:880-888
        let ext_perimeter_width = self.config.external_perimeter_extrusion_width;
        let perimeter_width = self.config.perimeter_extrusion_width;
        let ext_perimeter_spacing = self.config.external_perimeter_spacing;
        let perimeter_spacing = self.config.perimeter_spacing;
        let ext_perimeter_spacing2 = self.config.external_to_internal_spacing;

        // R101: native computes wall OFFSET deltas from Flow::scaled_width() =
        // coord_t(scale_(m_width)) where the flow width is stored as FLOAT and scale_
        // TRUNCATES toward zero: 0.42 -> f32(0.41999998) -> 41999.998 -> 41999 (NOT
        // 42000). Rust carried the width at f64 and its scale() ROUNDS -> 42000, so
        // every wall offset delta was ~0.5 unit off, amplified to tens of units at
        // miter joins (the R100 residual; outer-wall canonical-hash 0%). `nsc`
        // reproduces native's f32-truncated scaled magnitude (as mm) for the offset
        // DELTA only — the extrusion width / flow-E keeps the raw value (native uses
        // mm3_per_mm, not scaled_width, for E), so this is geometry-only. Gated
        // F1_UNION; default path byte-unchanged. (The residual material shift is a
        // pre-existing rust E-per-length gap this correct geometry exposes — a
        // separate lever; see PARITY_STATUS.) Spacings/inner-wall offset2 deltas are
        // NOT yet ported (inner-wall geometry stays at status quo) — follow-up round.
        let nsc = |w_mm: f64| -> f64 {
            if crate::faithful_gate("F1_UNION") {
                crate::unscale(((w_mm as f32) as f64 * crate::SCALING_FACTOR).trunc() as Coord)
            } else {
                w_mm
            }
        };

        /// PerimeterGenerator.cpp:882
        /// C++: coord_t min_spacing = coord_t(perimeter_spacing * (1 - INSET_OVERLAP_TOLERANCE));
        let min_spacing = perimeter_spacing * (1.0 - INSET_OVERLAP_TOLERANCE);

        /// PerimeterGenerator.cpp:883
        /// C++: coord_t ext_min_spacing = coord_t(ext_perimeter_spacing * (1 - INSET_OVERLAP_TOLERANCE));
        let ext_min_spacing = ext_perimeter_spacing * (1.0 - INSET_OVERLAP_TOLERANCE);

        /// PerimeterGenerator.cpp:884
        /// C++: bool has_gap_fill = this->config->gap_infill_speed.get_at(...) > 0;
        let has_gap_fill = self.config.gap_fill_threshold > 0.0;

        /// PerimeterGenerator.cpp:887
        /// C++: coord_t ext_min_spacing_smaller = coord_t(ext_perimeter_spacing * (1 - SMALLER_EXT_INSET_OVERLAP_TOLERANCE));
        let ext_min_spacing_smaller =
            ext_perimeter_spacing * (1.0 - SMALLER_EXT_INSET_OVERLAP_TOLERANCE);

        /// PerimeterGenerator.cpp:888-890
        /// C++: this->smaller_ext_perimeter_flow = this->smaller_ext_perimeter_flow.with_width(...)
        let ext_perimeter_smaller_width =
            ext_perimeter_width - 0.5 * SMALLER_EXT_INSET_OVERLAP_TOLERANCE * ext_perimeter_spacing;

        /// PerimeterGenerator.cpp:949-950
        /// C++: std::vector<PerimeterGeneratorLoops> contours(loop_number+1);
        /// C++: std::vector<PerimeterGeneratorLoops> holes(loop_number+1);
        let mut contours: Vec<Vec<PerimeterLoop>> = vec![Vec::new(); loop_number + 1];
        let mut holes: Vec<Vec<PerimeterLoop>> = vec![Vec::new(); loop_number + 1];

        /// PerimeterGenerator.cpp:951
        /// C++: ThickPolylines thin_walls;
        let mut thin_walls: ThickPolylines = Vec::new();
        // PerimeterGenerator.cpp:951 — outer-wall contours collected at depth
        // i==0 for z-direction speed continuity (z_direction_outwall_speed_continuous).
        let mut outwall_paths: Vec<crate::layer::NodeContour> = Vec::new();

        /// PerimeterGenerator.cpp:952
        /// C++: ExPolygons gaps;
        let mut gaps: ExPolygons = Vec::new();

        // PerimeterGenerator.cpp:949-950 — top_fills/fill_clip (only_one_wall_top), merged into
        // the infill area at the end so fill_expolygons covers the top skin.
        let mut top_fills: ExPolygons = Vec::new();
        let mut fill_clip: ExPolygons = Vec::new();

        /// PerimeterGenerator.cpp:954
        /// C++: for (int i = 0;; ++ i) {
        let mut i = 0;
        loop {
            if i >= MAX_PERIMETER_ITERATIONS {
                break;
            }

            /// PerimeterGenerator.cpp:956-957
            /// C++: ExPolygons offsets;
            /// C++: ExPolygons offsets_with_smaller_width;
            let mut offsets: ExPolygons = Vec::new();
            let mut offsets_with_smaller_width: ExPolygons = Vec::new();

            /// PerimeterGenerator.cpp:958
            /// C++: if (i == 0) {
            if i == 0 {
                /// PerimeterGenerator.cpp:960
                /// C++: if (this->config->detect_thin_wall) {
                if self.config.detect_thin_wall {
                    // PerimeterGenerator.cpp:963-965
                    // C++: offsets = offset2_ex(last, -float(ext_perimeter_width / 2. + ext_min_spacing / 2. - 1), +float(ext_min_spacing / 2. - 1));
                    // NOTE: the `- 1` here is 1 *scaled* coord unit (= 1/SCALING_FACTOR mm),
                    // NOT ClipperSafetyOffset (which is 10). 1 unit = 0.00001 mm.
                    const ONE_SCALED_MM: f64 = 1.0 / SCALING_FACTOR;
                    // ClipperSafetyOffset = 10 scaled units = 0.0001mm (used at line 975).
                    const CLIPPER_SAFETY_OFFSET: f64 = 0.0001;
                    // R278 (F1_UNION): native-exact scaled-int deltas through the
                    // vendored ClipperLib — W_e/S_e are coord_t (trunc(v/1e-5),
                    // scale_ DIVIDES), ext_min_spacing = coord_t(S_e*(1-0.4)), all
                    // delta exprs in double then f32 at the offset boundary (the
                    // clib shims reproduce the f32; PG.cpp:886-887, 963-975).
                    let f1 = crate::faithful_gate("F1_UNION");
                    let w_e_c = (self.config.ext_perimeter_flow.width() / 0.00001).trunc();
                    let s_e_c = (self.config.ext_perimeter_flow.spacing() / 0.00001).trunc();
                    let ext_min_c = (s_e_c * (1.0 - INSET_OVERLAP_TOLERANCE)).trunc();
                    offsets = if f1 {
                        crate::clipper_utils::offset2_ex_clib(
                            &last,
                            -(w_e_c / 2.0 + ext_min_c / 2.0 - 1.0) / SCALING_FACTOR,
                            (ext_min_c / 2.0 - 1.0) / SCALING_FACTOR,
                            self.config.join_type,
                        )
                    } else {
                        offset2(
                            &last,
                            ext_perimeter_width / 2.0 + ext_min_spacing / 2.0 - ONE_SCALED_MM,
                            ext_min_spacing / 2.0 - ONE_SCALED_MM,
                            self.config.join_type,
                        )
                    };

                    // PerimeterGenerator.cpp:966-975
                    // Thin wall detection using medial axis
                    // C++: coord_t min_width = coord_t(scale_(this->ext_perimeter_flow.nozzle_diameter() / 3));
                    let min_width = self.config.ext_perimeter_flow.nozzle_diameter() / 3.0;

                    // C++: ExPolygons expp = opening_ex(diff_ex(last, offset(offsets, float(ext_perimeter_width / 2.) + ClipperSafetyOffset)), float(min_width / 2.));
                    let (diff_region, expp);
                    if f1 {
                        // Native delta: float(W_e/2.) + ClipperSafetyOffset(10.f) — f32
                        // arithmetic; min_width coord_t = trunc((nozzle_f32/3)f64/1e-5).
                        let grow_d = (((w_e_c / 2.0) as f32 + 10.0f32) as f64) / SCALING_FACTOR;
                        let offset_outward = crate::clipper_utils::grow_clib(
                            &offsets,
                            grow_d,
                            self.config.join_type,
                        );
                        diff_region =
                            crate::clipper_utils::difference_clib(&last, &offset_outward);
                        let nozzle = self.config.ext_perimeter_flow.nozzle_diameter();
                        let min_w_c = ((nozzle as f32 / 3.0) as f64 / 0.00001).trunc();
                        let open_d = (min_w_c / 2.0) / SCALING_FACTOR;
                        expp = crate::clipper_utils::offset2_ex_clib(
                            &diff_region,
                            -open_d,
                            open_d,
                            self.config.join_type,
                        );
                    } else {
                        let offset_outward = grow(
                            &offsets,
                            ext_perimeter_width / 2.0 + CLIPPER_SAFETY_OFFSET,
                            self.config.join_type,
                        );
                        diff_region = difference(&last, &offset_outward);
                        expp = opening(&diff_region, min_width / 2.0, self.config.join_type);
                    }
                    let _ = &diff_region;

                    // C++: for (ExPolygon &ex : expp)
                    // C++:     ex.medial_axis(min_width, ext_perimeter_width + ext_perimeter_spacing2, &thin_walls);
                    for ex in expp {
                        let max_width = ext_perimeter_width + ext_perimeter_spacing2;
                        if crate::faithful_gate("MEDIALAXIS_NATIVE") {
                            // Native: min = coord_t(scale_(nozzle/3)) — nozzle/3 in
                            // f32, double division by 1e-5, trunc (PG.cpp:972); max =
                            // ext_perimeter_width + ext_perimeter_spacing2, both
                            // coord_t: scaled_width() = trunc(width/1e-5) (double),
                            // spacing2 = scaled<coord_t>(0.5f*(sp_e+sp_p)) — ALL-f32
                            // arithmetic including the /1e-5f division (PG.cpp:868).
                            let nozzle = self.config.ext_perimeter_flow.nozzle_diameter();
                            let min_sc = ((nozzle as f32 / 3.0) as f64 / 0.00001).trunc();
                            let ext_w_sc =
                                (self.config.ext_perimeter_flow.width() / 0.00001).trunc();
                            let sp2_sc = ((0.5f32
                                * (self.config.ext_perimeter_flow.spacing() as f32
                                    + self.config.perimeter_flow.spacing() as f32))
                                / 1e-5f32)
                                .trunc() as f64;
                            ex.medial_axis_scaled(min_sc, ext_w_sc + sp2_sc, &mut thin_walls);
                        } else {
                            ex.medial_axis(min_width, max_width, &mut thin_walls);
                        }
                    }
                } else {
                    /// PerimeterGenerator.cpp:977-978
                    /// C++: coord_t ext_perimeter_smaller_width = this->smaller_ext_perimeter_flow.scaled_width();
                    /// C++: for (const ExPolygon& expolygon : last) {
                    for expolygon in &last {
                        /// PerimeterGenerator.cpp:980-982
                        /// C++: ExPolygons expolys;
                        /// C++: expolys.push_back(expolygon);
                        let expolys = vec![expolygon.clone()];

                        /// PerimeterGenerator.cpp:983-985
                        /// C++: ExPolygons offset_result = offset2_ex(expolys, -float(ext_perimeter_width / 2. + ext_min_spacing_smaller / 2.), +float(ext_min_spacing_smaller / 2.));
                        let offset_result = if crate::faithful_gate("F1_UNION") {
                            // R278: native-exact — W_e coord_t, ext_min_spacing_smaller =
                            // coord_t(S_e*(1-0.22)); deltas double->f32 (PG.cpp:891, 983-985).
                            let w_e_c =
                                (self.config.ext_perimeter_flow.width() / 0.00001).trunc();
                            let s_e_c =
                                (self.config.ext_perimeter_flow.spacing() / 0.00001).trunc();
                            let ext_min_sm_c =
                                (s_e_c * (1.0 - SMALLER_EXT_INSET_OVERLAP_TOLERANCE)).trunc();
                            crate::clipper_utils::offset2_ex_clib(
                                &expolys,
                                -(w_e_c / 2.0 + ext_min_sm_c / 2.0) / SCALING_FACTOR,
                                (ext_min_sm_c / 2.0) / SCALING_FACTOR,
                                self.config.join_type,
                            )
                        } else {
                            offset2(
                                &expolys,
                                ext_perimeter_width / 2.0 + ext_min_spacing_smaller / 2.0,
                                ext_min_spacing_smaller / 2.0,
                                self.config.join_type,
                            )
                        };

                        /// PerimeterGenerator.cpp:986-987
                        /// C++: if (offset_result.empty() && expolygon.area() < (double)(ext_perimeter_width + ext_min_spacing_smaller) * scale_(narrow_loop_length_threshold)) {
                        let area_threshold = (ext_perimeter_width + ext_min_spacing_smaller)
                            * NARROW_LOOP_LENGTH_THRESHOLD
                            * SCALING_FACTOR
                            * SCALING_FACTOR;

                        if offset_result.is_empty() && expolygon.area().abs() < area_threshold {
                            /// PerimeterGenerator.cpp:989
                            /// C++: ExPolygons temp_result = offset_ex(expolygon, -float(ext_perimeter_smaller_width / 2.));
                            // PARITY (perim-offset): outer-wall contour offset. Route through the
                            // vertex-EXACT vendored ClipperLib (shrink_clib) instead of geo-clipper,
                            // which emits ~1.1-1.2% extra sub-|delta| vertices on every large slice
                            // (proven in M1: geo 1312/1330 vs clib 1296/1316). offset_ex == a single
                            // ClipperOffset at jtMiter, miterLimit 3.0 — clib_offset_expolygon_paths
                            // replicates ClipperUtils.cpp offset_expolygon_inner exactly. Area-invariant.
                            let temp_result = shrink_clib(
                                &[expolygon.clone()],
                                ext_perimeter_smaller_width / 2.0,
                                self.config.join_type,
                            );
                            offsets_with_smaller_width.extend(temp_result);
                        } else {
                            /// PerimeterGenerator.cpp:993
                            /// C++: ExPolygons temp_result = offset_ex(expolygon, -float(ext_perimeter_width / 2.));
                            // PARITY (perim-offset): outer-wall contour offset — see note above; this
                            // is the dominant outer-wall over-segmentation source (+2687 vs native).
                            // R101: delta = native float(scaled_width/2.); extrusion width kept raw.
                            let temp_result = shrink_clib(
                                &[expolygon.clone()],
                                nsc(ext_perimeter_width) / 2.0,
                                self.config.join_type,
                            );
                            offsets.extend(temp_result);
                        }
                    }
                }

                if std::env::var("PERIDBG").is_ok() {
                    let mut h: u64 = 1469598103934665603;
                    let mut np = 0usize;
                    for ex in &offsets {
                        for p in ex.contour.points() {
                            h ^= p.x as u64;
                            h = h.wrapping_mul(1099511628211);
                            h ^= p.y as u64;
                            h = h.wrapping_mul(1099511628211);
                            np += 1;
                        }
                    }
                    eprintln!(
                        "PERIDBG-R layer={} nex={} npts={} h={:x} nsw={}",
                        self.config.layer_id,
                        offsets.len(),
                        np,
                        h,
                        offsets_with_smaller_width.len()
                    );
                }
                // TODO: Port spiral vase mode (lines 997-1008)
            } else {
                /// PerimeterGenerator.cpp:1012
                /// C++: coord_t distance = (i == 1) ? ext_perimeter_spacing2 : perimeter_spacing;
                let distance = if i == 1 {
                    ext_perimeter_spacing2
                } else {
                    perimeter_spacing
                };

                // PerimeterGenerator.cpp:1026-1028
                // C++: offsets = offset2_ex(last, -float(distance + min_spacing / 2. - 1.), float(min_spacing / 2. - 1.));
                // The `- 1.` is 1 *scaled* coord unit (= 1/SCALING_FACTOR mm), not ClipperSafetyOffset.
                const ONE_SCALED_MM: f64 = 1.0 / SCALING_FACTOR;
                // PARITY (perim-offset): this is the SUCCESSIVE inner-wall offset2 — the
                // operation whose geo-clipper miter densification compounds over iterations
                // (~1.20x inner-wall vertex density, 0.396 vs native 0.329). Route it through
                // the vertex-EXACT vendored ClipperLib (offset2_clib) so inner-wall vertex
                // density byte-matches native. Area-invariant (material stays at parity). The
                // outer wall (i==0 above) is also routed through ClipperLib (shrink_clib).
                // R104 probe: route the inner offset through the faithful offset2_ex_clib
                // (cz_offset2_ex) with the integer-coord_t delta (matches native exactly),
                // so its stepA (between-passes) can be compared to native's. Gated F1_UNION.
                offsets = if crate::faithful_gate("F1_UNION") {
                    // R277: native scale_() DIVIDES by SCALING_FACTOR=1e-5 (not *1e5 —
                    // 1e-5 is inexact, trunc can differ by 1); and min_spacing/2. is a
                    // DOUBLE division (keeps the .5) — the previous integer half was a
                    // half-unit delta error on odd min_spacing (the ±1-unit boundary
                    // drift class, GAPDBG R276).
                    let per_sp_c = (perimeter_spacing / 0.00001).trunc();
                    let ext_sp2_c = (ext_perimeter_spacing2 / 0.00001).trunc();
                    let distance_c = if i == 1 { ext_sp2_c } else { per_sp_c };
                    let min_spacing_c = (per_sp_c * (1.0 - INSET_OVERLAP_TOLERANCE)).trunc();
                    let half_min = min_spacing_c / 2.0;
                    let d1_mm = -(distance_c + half_min - 1.0) / SCALING_FACTOR;
                    let d2_mm = (half_min - 1.0) / SCALING_FACTOR;
                    crate::clipper_utils::offset2_ex_clib(&last, d1_mm, d2_mm, self.config.join_type)
                } else {
                    offset2_clib(
                        &last,
                        distance + min_spacing / 2.0 - ONE_SCALED_MM,
                        min_spacing / 2.0 - ONE_SCALED_MM,
                        self.config.join_type,
                    )
                };

                /// PerimeterGenerator.cpp:1030-1035
                /// C++: if (has_gap_fill) append(gaps, diff_ex(offset(last, - float(0.5 * distance)), offset(offsets, float(0.5 * distance + 10))));
                if has_gap_fill {
                    // The `+ 10` is ClipperSafetyOffset = 10 scaled units = 0.0001 mm.
                    // R108: these gap-detection offsets/difference ran through geo-clipper
                    // (shrink/grow @ GEO_CLIPPER_SCALE=1000, difference @1µm) — the R100
                    // gridding class. MEASURED (GAPDBG): the inputs `last`/`offsets` arrive
                    // full-resolution (0% on the 1µm grid, R104/R105-faithful) but the geo
                    // ops snapped every detected-gap vertex to the grid (100% on-grid),
                    // fragmenting thin gap regions into ~13x too many on-grid slivers. That
                    // drove gap-infill's 17% diff share (R107): gap toolpath vertices matched
                    // native only 2.3% (XY multiset). Under F1_UNION route the same
                    // shrink/grow/difference through the vertex-exact vendored ClipperLib
                    // (shrink_clib/grow_clib/difference_clib @ i32/1e5, reconstructing via
                    // union_ex_clib) — gap output on-grid 100%→0%, XY multiset match
                    // 2.3%→46.9%, total gated diff 243105→188922 (−22%). Same mm deltas
                    // (geo delta is unscaled mm; the clib shims take mm and scale @1e5).
                    // Default path keeps geo (difference_clib reconstructs via geo union off
                    // F1_UNION — R106), byte-unchanged.
                    let detected_gaps = if crate::faithful_gate("F1_UNION") {
                        // R277: native halves the COORD_T distance — float(0.5*distance)
                        // with distance an integer (PG.cpp:1030-1035). Halving the raw mm
                        // value drifts the delta by up to 0.5 units when trunc(mm/1e-5)
                        // loses a fraction (±1-unit boundary drift class, GAPDBG R276).
                        let dist_c = (distance / 0.00001).trunc();
                        let gap_outer = crate::clipper_utils::shrink_clib(
                            &last,
                            (0.5 * dist_c) / SCALING_FACTOR,
                            self.config.join_type,
                        );
                        let gap_inner = crate::clipper_utils::grow_clib(
                            &offsets,
                            (0.5 * dist_c + 10.0) / SCALING_FACTOR,
                            self.config.join_type,
                        );
                        crate::clipper_utils::difference_clib(&gap_outer, &gap_inner)
                    } else {
                        let gap_outer = shrink(&last, 0.5 * distance, self.config.join_type);
                        let gap_inner =
                            grow(&offsets, 0.5 * distance + 0.0001, self.config.join_type);
                        difference(&gap_outer, &gap_inner)
                    };
                    gaps.extend(detected_gaps);
                }
            }

            if crate::probe_enabled("LOOPI") {
                noprobe_dump(&format!("LOOPI{}", i), self.config.layer_id, &offsets);
                if !offsets_with_smaller_width.is_empty() {
                    noprobe_dump(
                        &format!("LOOPS{}", i),
                        self.config.layer_id,
                        &offsets_with_smaller_width,
                    );
                }
            }
            /// PerimeterGenerator.cpp:1037-1044
            /// C++: if (offsets.empty() && offsets_with_smaller_width.empty()) {
            /// C++:     loop_number = i - 1;
            /// C++:     last.clear();
            /// C++:     break;
            /// C++: } else if (i > loop_number) {
            /// C++:     break;
            /// C++: }
            if offsets.is_empty() && offsets_with_smaller_width.is_empty() {
                loop_number = i.saturating_sub(1);
                last.clear();
                break;
            } else if i > loop_number {
                break;
            }

            /// PerimeterGenerator.cpp:1045-1058
            /// C++: for (const ExPolygon& expolygon : offsets) {
            /// C++:     contours[i].emplace_back(PerimeterGeneratorLoop(expolygon.contour, i, true, false, ...));
            /// C++:     if (!expolygon.holes.empty()) {
            /// C++:         holes[i].reserve(holes[i].size() + expolygon.holes.size());
            /// C++:         for (const Polygon &hole : expolygon.holes)
            /// C++:             holes[i].emplace_back(hole, i, false, false, ...);
            /// C++:     }
            /// C++: }
            for expolygon in &offsets {
                let loop_item = PerimeterLoop::new(
                    expolygon.contour.clone(),
                    i,
                    true,  // is_contour
                    false, // is_smaller_width
                    if i == 0 {
                        ext_perimeter_width
                    } else {
                        perimeter_width
                    },
                );
                contours[i].push(loop_item);

                if !expolygon.holes.is_empty() {
                    for hole in &expolygon.holes {
                        let hole_loop = PerimeterLoop::new(
                            hole.clone(),
                            i,
                            false, // is_contour (it's a hole)
                            false, // is_smaller_width
                            if i == 0 {
                                ext_perimeter_width
                            } else {
                                perimeter_width
                            },
                        );
                        holes[i].push(hole_loop);
                    }
                }
            }

            /// PerimeterGenerator.cpp:1079-1093
            /// C++: if (i == 0) {
            /// C++:     for (const ExPolygon& expolygon : offsets_with_smaller_width) {
            /// C++:         contours[i].emplace_back(PerimeterGeneratorLoop(expolygon.contour, i, true, true, ...));
            /// C++:         if (!expolygon.holes.empty()) {
            /// C++:             holes[i].reserve(holes[i].size() + expolygon.holes.size());
            /// C++:             for (const Polygon& hole : expolygon.holes)
            /// C++:                 holes[i].emplace_back(PerimeterGeneratorLoop(hole, i, false, true, ...));
            /// C++:         }
            /// C++:     }
            /// C++: }
            if i == 0 {
                // PerimeterGenerator.cpp:1074-1100 — store the outer-wall
                // contours for z-direction speed continuity. Order matters
                // (native: thin_walls, offsets_with_smaller_width, offsets).
                // to_polylines(ExPolygons) closes each contour/hole by
                // re-appending its first point (ExPolygon.hpp:231-247).
                if self.config.z_direction_outwall_speed_continuous {
                    for polyline in thin_walls.iter() {
                        outwall_paths.push(crate::layer::NodeContour {
                            is_loop: false,
                            pts: polyline.points.clone(),
                            widths: polyline
                                .widths
                                .iter()
                                .map(|w| crate::clipper_utils::scale_faithful(*w))
                                .collect(),
                        });
                    }
                    let smaller_w = crate::clipper_utils::scale_faithful(
                        self.config.smaller_ext_perimeter_flow.width(),
                    );
                    for expolygon in &offsets_with_smaller_width {
                        for poly in std::iter::once(&expolygon.contour).chain(expolygon.holes.iter())
                        {
                            let mut pts = poly.points.clone();
                            if let Some(first) = pts.first().copied() {
                                pts.push(first);
                            }
                            outwall_paths.push(crate::layer::NodeContour {
                                is_loop: true,
                                pts,
                                widths: vec![smaller_w],
                            });
                        }
                    }
                    let ext_w = crate::clipper_utils::scale_faithful(
                        self.config.ext_perimeter_flow.width(),
                    );
                    for expolygon in &offsets {
                        for poly in std::iter::once(&expolygon.contour).chain(expolygon.holes.iter())
                        {
                            let mut pts = poly.points.clone();
                            if let Some(first) = pts.first().copied() {
                                pts.push(first);
                            }
                            outwall_paths.push(crate::layer::NodeContour {
                                is_loop: true,
                                pts,
                                widths: vec![ext_w],
                            });
                        }
                    }
                }
                for expolygon in &offsets_with_smaller_width {
                    let loop_item = PerimeterLoop::new(
                        expolygon.contour.clone(),
                        i,
                        true, // is_contour
                        true, // is_smaller_width
                        ext_perimeter_smaller_width,
                    );
                    contours[i].push(loop_item);

                    if !expolygon.holes.is_empty() {
                        for hole in &expolygon.holes {
                            let hole_loop = PerimeterLoop::new(
                                hole.clone(),
                                i,
                                false, // is_contour (it's a hole)
                                true,  // is_smaller_width
                                ext_perimeter_smaller_width,
                            );
                            holes[i].push(hole_loop);
                        }
                    }
                }
            }

            // PerimeterGenerator.cpp:1096
            // C++: last = std::move(offsets);
            last = offsets;

            // PerimeterGenerator.cpp:1116-1183 — only_one_wall_top (TopOneWallType::Alltop)
            // top/not-top split + top_fills. BBS: refer to superslicer.
            // C++: if (i == 0 && i != loop_number && this->object_config->top_one_wall_type ==
            //          TopOneWallType::Alltop && this->upper_slices != NULL) {
            if i == 0
                && i != loop_number
                && self.config.top_one_wall
                && top_fills_gate()
                && self.config.upper_slices.is_some()
            {
                let upper = self.config.upper_slices.as_ref().unwrap();
                // R104: un-grid the last-chain in this only_one_wall_top block (see note
                // below) via the vertex-exact ClipperLib. Gated F1_UNION; default byte-unchanged.
                let f1_top = crate::faithful_gate("F1_UNION");

                // PerimeterGenerator.cpp:1121-1126
                // C++: coord_t offset_top_surface = scale_(1.5 * (wall_loops == 0 ? 0. :
                // C++:     unscaled(ext_perimeter_width + perimeter_spacing * (wall_loops - 1))));
                // C++: if (offset_top_surface > 0.9 * (wall_loops <= 1 ? 0. : (perimeter_spacing * (wall_loops - 1))))
                // C++:     offset_top_surface -= coord_t(0.9 * (...));
                // C++: else offset_top_surface = 0;
                let wl = self.config.perimeter_count as f64;
                let mut offset_top_surface = if self.config.perimeter_count == 0 {
                    0.0
                } else {
                    1.5 * (ext_perimeter_width + perimeter_spacing * (wl - 1.0))
                };
                let reduction = if self.config.perimeter_count <= 1 {
                    0.0
                } else {
                    0.9 * perimeter_spacing * (wl - 1.0)
                };
                if offset_top_surface > reduction {
                    offset_top_surface -= reduction;
                } else {
                    offset_top_surface = 0.0;
                }

                // PerimeterGenerator.cpp:1128
                // C++: double min_width_top_surface = (top_area_threshold / 100) *
                // C++:     std::max(ext_perimeter_spacing / 2.0, perimeter_width / 2.0);
                let min_width_top_surface = (self.config.top_area_threshold / 100.0)
                    * (ext_perimeter_spacing / 2.0).max(perimeter_width / 2.0);
                // R120: native computes the top-chain offset deltas in SCALED coord_t —
                // the spacings are `scaled_width()/scaled_spacing()` = coord_t(scale_(f32))
                // (= scale_faithful), min_width/offset_top/inner_off are built from those,
                // and `offset(paths, float delta)` f32-casts. Reproduce the exact scaled
                // values for the gated clib ops; `offset_delta_mm_from_scaled_f32` converts
                // each to the mm delta `offset_expolygons_clib` (×1e5 internally) needs.
                // NOTE eps/2 is INTEGER division in native (`double(ext_perimeter_spacing/2)`,
                // coord_t/int → 18853 not 18853.5).
                let eps_s = crate::clipper_utils::scale_faithful(ext_perimeter_spacing);
                let pw_s = crate::clipper_utils::scale_faithful(perimeter_width);
                let epw_s = crate::clipper_utils::scale_faithful(ext_perimeter_width);
                let psp_s = crate::clipper_utils::scale_faithful(perimeter_spacing);
                let wli = self.config.perimeter_count as i64;
                let min_width_scaled = (self.config.top_area_threshold / 100.0)
                    * (eps_s as f64 / 2.0).max(pw_s as f64 / 2.0);
                let mut offset_top_scaled: f64 = if self.config.perimeter_count == 0 {
                    0.0
                } else {
                    // scale_(1.5 * unscaled(epw + psp*(wl-1))): unscaled = *0.00001, scale_ = /0.00001
                    let sum_s = epw_s + psp_s * (wli - 1);
                    ((1.5 * (sum_s as f64 * 0.00001)) / 0.00001).trunc()
                };
                let reduction_s = if self.config.perimeter_count <= 1 {
                    0.0
                } else {
                    0.9 * (psp_s * (wli - 1)) as f64
                };
                if offset_top_scaled > reduction_s {
                    offset_top_scaled -= reduction_s.trunc(); // native coord_t(0.9*...) truncates
                } else {
                    offset_top_scaled = 0.0;
                }
                let inner_off_scaled = offset_top_scaled + min_width_scaled - (eps_s / 2) as f64;

                // PerimeterGenerator.cpp:1131-1136
                // C++: BoundingBox last_box = get_extents(last);
                // C++: Polygons upper_polygons_series_clipped =
                // C++:     ClipperUtils::clip_clipper_polygons_with_subject_bbox(*this->upper_slices, last_box);
                // C++: upper_polygons_series_clipped = offset(upper_polygons_series_clipped, min_width_top_surface);
                //
                // The bbox clip is NOT just a perf optimization: because the offset is applied
                // AFTER the clip, clipping `upper` to this island's `last` bbox first means the
                // offset of the (truncated) upper edge does NOT bleed across `last`'s rim — so a
                // thin top band survives `diff(last, upper)`. Growing the FULL upper (the prior
                // Rust shortcut) covers `last` entirely → top_fills empty → rim Top/Bridge
                // surfaces clipped away. (Pinned via C++-vs-Rust runtime dumps on Benchy L80.)
                let last_box = crate::geometry::get_extents(&last);
                let upper_clipped_polys = crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(
                    upper,
                    &last_box,
                    false,
                );
                // R120: native `offset(clip(upper, last_box), float(min_width_top_surface))`
                // (PerimeterGenerator.cpp:1131). The default geo `offset_polygons` grids the
                // bbox-clipped upper to 1µm (R100/R115 class); under f1_top route through the
                // vertex-exact ClipperLib (union_ex_clib the clipped Polygons → ExPolygons,
                // then offset_expolygons_clib) with the scale_faithful min_width delta so the
                // upper edge (and the top_polygons diff that consumes it) stays full-res.
                let upper_polygons_series_clipped = if f1_top {
                    let upper_ex = crate::clipper_utils::union_ex_clib(&upper_clipped_polys, 1);
                    let mw_mm = crate::clipper_utils::offset_delta_mm_from_scaled_f32(min_width_scaled);
                    crate::clipper_utils::offset_expolygons_clib(&upper_ex, mw_mm, OffsetJoinType::Miter)
                } else {
                    crate::clipper_utils::offset_polygons(&upper_clipped_polys, min_width_top_surface, OffsetJoinType::Miter)
                };

                // PerimeterGenerator.cpp:1139
                // C++: fill_clip = offset_ex(last, -double(ext_perimeter_spacing));
                // R105: feeds temp_gap → last; un-grid under f1_top.
                fill_clip = if crate::faithful_gate("TOPFILL_CLIP_FAITHFUL") {
                    // -double(coord_t ext_perimeter_spacing) through the float
                    // parameter of offset_ex, on the scaled value itself.
                    crate::clipper_utils::offset_expolygons_clib_scaled(
                        &last,
                        -((eps_s as f64) as f32) as f64,
                        OffsetJoinType::Miter,
                    )
                } else if f1_top {
                    crate::clipper_utils::offset_expolygons_clib(&last, -ext_perimeter_spacing, OffsetJoinType::Miter)
                } else {
                    offset_expolygons(&last, -ext_perimeter_spacing, OffsetJoinType::Miter)
                };

                // R104: this only_one_wall_top block recomputes `last` (line 942,
                // last = intersection(inner_polygons, last)) via geo-clipper, gridding
                // it to 1µm BEFORE it feeds the inner-wall offset chain (i≥1) AND the
                // infill-area computation. Native uses ClipperLib (10nm), so rust's inner
                // input diverges (inner-offset input 99.8% on-100-grid vs native 5.2%) →
                // inner walls 0%, and the classification jitter. The whole last-chain
                // (upper offset / top_polygons / inner_polygons / the final intersection+
                // union) is routed through the vertex-exact ClipperLib under f1_top.

                // PerimeterGenerator.cpp:1144
                // C++: ExPolygons top_polygons = diff_ex(last, upper_polygons_series_clipped, ApplySafetyOffset::Yes);
                // R120: this diff is ApplySafetyOffset::Yes — route through difference_clib_safety
                // (R116 shim), not the No-safety difference_clib.
                let mut top_polygons = if f1_top {
                    crate::clipper_utils::difference_clib_safety(&last, &upper_polygons_series_clipped)
                } else {
                    difference(&last, &upper_polygons_series_clipped)
                };

                fbis_dump("t_last", self.config.layer_id, &last);
                fbis_dump("t_upper", self.config.layer_id, &upper_polygons_series_clipped);
                fbis_dump("t_top1", self.config.layer_id, &top_polygons);
                // PerimeterGenerator.cpp:1146
                // C++: ExPolygons temp_gap = diff_ex(top_polygons, fill_clip);
                // R105: temp_gap is unioned into `last` (has_gap_fill) → un-grid under f1_top.
                let temp_gap = if f1_top {
                    crate::clipper_utils::difference_clib(&top_polygons, &fill_clip)
                } else {
                    difference(&top_polygons, &fill_clip)
                };

                // PerimeterGenerator.cpp:1147-1149
                // C++: ExPolygons inner_polygons = diff_ex(last,
                // C++:     offset_ex(top_polygons, offset_top_surface + min_width_top_surface - double(ext_perimeter_spacing / 2)),
                // C++:     ApplySafetyOffset::Yes);
                let inner_off_delta =
                    offset_top_surface + min_width_top_surface - ext_perimeter_spacing / 2.0;
                // R120: this diff is ApplySafetyOffset::Yes; the inner offset delta is the
                // scale_faithful `inner_off_scaled` (offset_top + min_width − eps/2, eps/2
                // integer-divided), fed to offset_expolygons_clib via the f32-cast mm helper.
                let mut inner_polygons = if f1_top {
                    let ioff_mm =
                        crate::clipper_utils::offset_delta_mm_from_scaled_f32(inner_off_scaled);
                    crate::clipper_utils::difference_clib_safety(
                        &last,
                        &crate::clipper_utils::offset_expolygons_clib(
                            &top_polygons,
                            ioff_mm,
                            OffsetJoinType::Miter,
                        ),
                    )
                } else {
                    difference(
                        &last,
                        &offset_expolygons(&top_polygons, inner_off_delta, OffsetJoinType::Miter),
                    )
                };

                // PerimeterGenerator.cpp:1150-1161
                // C++: if (this->lower_slices != NULL) {
                // C++:     Polygons lower_polygons_series_clipped = ...(*this->lower_slices, last_box);
                // C++:     double bridge_offset = std::max(double(ext_perimeter_spacing), (double(perimeter_width)));
                // C++:     bridge_checker = offset_ex(diff_ex(last, lower_polygons_series_clipped, ApplySafetyOffset::Yes), 1.5 * bridge_offset);
                // C++:     if (!bridge_checker.empty() && !intersection_ex(bridge_checker, inner_polygons).empty())
                // C++:         inner_polygons = union_ex(inner_polygons, bridge_checker);
                // C++: }
                // BBS: if the bridge has a connection with the non-top area it belongs to
                // the non-top area, otherwise it stays top to get a better surface.
                if let Some(lower) = self.config.lower_slices.as_ref() {
                    let bridge_offset = ext_perimeter_spacing.max(perimeter_width);
                    // R105: on bridge/overhang layers this re-grids inner_polygons (which
                    // feeds `last`); un-grid the bridge_checker + the merge union under
                    // f1_top. The emptiness check stays geo (boolean, does not touch last).
                    let bridge_checker = if f1_top {
                        // R118: native `offset_ex(diff_ex(last, clip(lower_slices, last_box),
                        // ApplySafetyOffset::Yes), 1.5*bridge_offset)` (PerimeterGenerator.cpp:1172-1175).
                        // Now FAITHFUL — three coupled fixes landed together (R113-R117):
                        //   (a) bbox-clip lower to last_box (was unclipped);
                        //   (b) ::Yes safety difference via difference_clib_safety (R116 shim,
                        //       oracle byte-exact to native diff_last_lower);
                        //   (c) scale_-faithful bridge_offset via scale_faithful (R117: f32-cast
                        //       divide-truncate → 44999 not 45000 → 1.5·44999 = 67498.5).
                        // R116 churned ONLY because (c) lacked the f32 cast; with it, the
                        // bridge closes L1/L5/L7/L9/L10/L18/L19 byte-exact and the intersection
                        // that consumes inner_polygons→last is faithful (R118 phase-1 proof).
                        let lc = crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(
                            lower, &last_box, false,
                        );
                        let lc_ex: Vec<crate::ExPolygon> =
                            lc.iter().map(|p| crate::ExPolygon::new(p.clone())).collect();
                        let bc_diff = crate::clipper_utils::difference_clib_safety(&last, &lc_ex);
                        let bo_scaled = crate::clipper_utils::scale_faithful(ext_perimeter_spacing)
                            .max(crate::clipper_utils::scale_faithful(perimeter_width))
                            as f64;
                        let bridge_delta_mm =
                            crate::clipper_utils::offset_delta_mm_from_scaled_f32(1.5 * bo_scaled);
                        crate::clipper_utils::offset_expolygons_clib(
                            &bc_diff,
                            bridge_delta_mm,
                            OffsetJoinType::Miter,
                        )
                    } else {
                        offset_expolygons(
                            &difference(&last, lower),
                            1.5 * bridge_offset,
                            OffsetJoinType::Miter,
                        )
                    };
                    if !bridge_checker.is_empty()
                        && !intersection(&bridge_checker, &inner_polygons).is_empty()
                    {
                        if f1_top {
                            let mut rings = crate::geometry::to_polygons(&inner_polygons);
                            rings.extend(crate::geometry::to_polygons(&bridge_checker));
                            inner_polygons = crate::clipper_utils::union_ex_clib(&rings, 1);
                        } else {
                            let mut merged = inner_polygons;
                            merged.extend(bridge_checker);
                            inner_polygons = union_ex(&merged);
                        }
                    }
                }

                // PerimeterGenerator.cpp:1162-1163
                // C++: top_polygons = diff_ex(fill_clip, inner_polygons, ApplySafetyOffset::Yes);
                // TOPFILL_CLIP_FAITHFUL (R814): the safety-offset difference on
                // the ClipperLib grid, as written above; =0 keeps the 1 um
                // difference without safety offset.
                let topfill_clip_faithful = crate::faithful_gate("TOPFILL_CLIP_FAITHFUL");
                fbis_dump("t_inner", self.config.layer_id, &inner_polygons);
                fbis_dump("t_clip1", self.config.layer_id, &fill_clip);
                top_polygons = if topfill_clip_faithful {
                    crate::clipper_utils::difference_clib_safety(&fill_clip, &inner_polygons)
                } else {
                    difference(&fill_clip, &inner_polygons)
                };

                fbis_dump("t_top2", self.config.layer_id, &top_polygons);
                // PerimeterGenerator.cpp:1164-1165
                // C++: top_fills = union_ex(top_fills, top_polygons);
                let mut merged = std::mem::take(&mut top_fills);
                merged.extend(top_polygons);
                // Native union_ex runs ClipperLib @1e5; geo grids top_fills @1um
                // (feeds top_infill_exp -> the fill boundary). Gated full-res.
                top_fills = if crate::faithful_gate("TOPFILL_FAITHFUL") {
                    crate::clipper_utils::union_ex_clib(
                        &crate::geometry::to_polygons(&merged),
                        1,
                    )
                } else {
                    union_ex(&merged)
                };

                // PerimeterGenerator.cpp:1166-1168
                // C++: double infill_spacing_unscaled = this->config->sparse_infill_line_width.value;
                // C++: fill_clip = offset_ex(last, double(ext_perimeter_spacing / 2) - scale_(infill_spacing_unscaled / 2));
                // Native offset_ex @1e5; geo grids fill_clip (168-vs-177pts, the
                // R140 residual driver). Gated full-res.
                fill_clip = if topfill_clip_faithful {
                    // double(coord_t / 2) - scale_(width / 2), then the float
                    // parameter of offset_ex (PerimeterGenerator.cpp:1187).
                    let delta = (eps_s / 2) as f64
                        - (self.config.sparse_infill_line_width / 2.0)
                            / crate::libslic3r::SCALING_FACTOR;
                    crate::clipper_utils::offset_expolygons_clib_scaled(
                        &last,
                        (delta as f32) as f64,
                        OffsetJoinType::Miter,
                    )
                } else if crate::faithful_gate("TOPFILL_FAITHFUL") {
                    crate::clipper_utils::offset_expolygons_clib(
                        &last,
                        ext_perimeter_spacing / 2.0 - self.config.sparse_infill_line_width / 2.0,
                        OffsetJoinType::Miter,
                    )
                } else {
                    offset_expolygons(
                        &last,
                        ext_perimeter_spacing / 2.0 - self.config.sparse_infill_line_width / 2.0,
                        OffsetJoinType::Miter,
                    )
                };

                // PerimeterGenerator.cpp:1169
                // C++: last = intersection_ex(inner_polygons, last);
                // R119: faithful single-op ctIntersection shim. The prior `A ∩ B =
                // A − (A − B)` double-difference (R118) added a ~0.001µm-off near-
                // collinear vertex on the L2/L3/L4/L6/L8/L16 band (the second
                // ctDifference re-processed the intermediate region) — the +1-pt
                // residual localized by the R119 stage-split oracle (inputs
                // byte-identical, native intersection_ex 147pt vs A−(A−B) 148pt).
                // `intersection_clib` = cz_intersection_closed + the same union_ex_clib
                // reconstruction difference_clib uses, matching native's
                // PolyTreeToExPolygons(clipper_do_polytree(ctIntersection,...)).
                if crate::probe_enabled("TSPROBE") {
                    noprobe_dump("TS_PRE", self.config.layer_id, &last);
                }
                last = if f1_top {
                    crate::clipper_utils::intersection_clib(&inner_polygons, &last)
                } else {
                    intersection(&inner_polygons, &last)
                };
                if crate::probe_enabled("TSPROBE") {
                    noprobe_dump("TS_POST", self.config.layer_id, &last);
                }

                // PerimeterGenerator.cpp:1170-1171
                // C++: if (has_gap_fill) last = union_ex(last, temp_gap);
                if has_gap_fill {
                    let mut merged = last;
                    merged.extend(temp_gap);
                    last = if f1_top {
                        crate::clipper_utils::union_ex_clib(&crate::geometry::to_polygons(&merged), 1)
                    } else {
                        union_ex(&merged)
                    };
                }
                if crate::probe_enabled("TSPROBE") {
                    noprobe_dump("TS_UNION", self.config.layer_id, &last);
                }

                // TOPDBG (diagnostics only, env-gated): dump the perimeter-derived
                // top region pieces for the TOPDBG_DUMP layer.
                crate::debug::topdbg::dump_expolygons(
                    self.config.layer_id,
                    "b_perimeter_top_fills",
                    &top_fills,
                );
            }

            /// PerimeterGenerator.cpp:1185-1189
            /// C++: if (i == loop_number && (! has_gap_fill || this->config->sparse_infill_density.value == 0)) {
            /// C++:     // The last run of this loop is executed to collect gaps for gap fill.
            /// C++:     // As the gap fill is either disabled or not
            /// C++:     break;
            /// C++: }
            // sparse_infill_density is threaded from PrintRegionConfig::fill_density (a
            // fraction; the == 0 check is equivalent to C++'s percent == 0).
            if i == loop_number && (!has_gap_fill || self.config.sparse_infill_density == 0.0) {
                break;
            }

            i += 1;
        }

        // PerimeterGenerator.cpp:1188-1219
        // C++: Nest holes — holes first into parent holes, then into innermost containing contour.
        // C++: for (int d = 0; d <= loop_number; ++ d) {
        // C++:     PerimeterGeneratorLoops &holes_d = holes[d];
        // C++:     for (int i = 0; i < holes_d.size(); ++ i) {
        // C++:         // find the hole loop that contains this one, if any
        // C++:         for (int t = d + 1; t <= loop_number; ++ t) { ... goto NEXT_LOOP; }
        // C++:         // if no hole, find innermost containing contour (t from loop_number down to 0)
        // C++:         for (int t = loop_number; t >= 0; -- t) { ... goto NEXT_LOOP; }
        // C++:         NEXT_LOOP: ;
        for d in 0..=loop_number {
            let holes_at_d = holes[d].clone();
            'next_hole: for hole in holes_at_d {
                // PerimeterGenerator.cpp:1194-1204
                // C++: find the hole loop that contains this hole (parent hole search)
                // C++: for (int t = d + 1; t <= loop_number; ++ t)
                for t in (d + 1)..=loop_number {
                    for hole_idx in 0..holes[t].len() {
                        if holes[t][hole_idx]
                            .polygon
                            .contains(&hole.polygon.first_point())
                        {
                            holes[t][hole_idx].children.push(hole);
                            continue 'next_hole; // C++: goto NEXT_LOOP
                        }
                    }
                }
                // PerimeterGenerator.cpp:1206-1216
                // C++: if no hole contains this hole, find innermost containing contour
                // C++: for (int t = loop_number; t >= 0; -- t)  ← starts from innermost!
                for t in (0..=loop_number).rev() {
                    for contour_idx in 0..contours[t].len() {
                        if contours[t][contour_idx]
                            .polygon
                            .contains(&hole.polygon.first_point())
                        {
                            contours[t][contour_idx].children.push(hole);
                            continue 'next_hole; // C++: goto NEXT_LOOP
                        }
                    }
                }
                // no parent found — hole is dropped
            }
        }

        // PerimeterGenerator.cpp:1221-1239
        // C++: Nest contours into parent contours at higher depths
        // C++: for (int d = loop_number; d >= 1; -- d) {
        // C++ uses exact first-match: iterate t from d-1 down to 0, first containing parent wins.
        for d in (1..=loop_number).rev() {
            let contours_at_d = contours[d].clone();
            'next_contour: for contour in contours_at_d {
                // PerimeterGenerator.cpp:1224-1238
                // C++: for (int t = d - 1; t >= 0; -- t)
                //          for candidate in contours[t]:
                //              if candidate.contains(loop.first_point): candidate.children.push(loop); goto next_loop;
                for t in (0..d).rev() {
                    for contour_idx in 0..contours[t].len() {
                        if contours[t][contour_idx]
                            .polygon
                            .contains(&contour.polygon.first_point())
                        {
                            contours[t][contour_idx].children.push(contour);
                            continue 'next_contour; // C++: goto next_loop
                        }
                    }
                }
                // No parent found — loop is unparented (same as C++ falling through without match)
            }
        }

        // PerimeterGenerator.cpp:1242-1420
        // C++: ExtrusionEntityCollection entities = traverse_loops(perimeter_generator, contours[0], thin_walls);
        // Convert top-level contours (depth 0) to extrusion entities
        result.entities = traverse_loops_legacy(
            &contours[0],
            &mut thin_walls,
            self.config.layer_height,
            &self.config.perimeter_flow,
            &self.config.ext_perimeter_flow,
            &self.config,
        );

        // Apply wall ordering after traverse_loops, matching C++ PerimeterGenerator.cpp:1246-1273.
        // traverse_loops naturally emits children (inner walls) before parent (outer wall)
        // per polygon, giving the InnerOuter base order.
        use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
        use crate::print_config::WallSequence;
        match self.config.wall_sequence {
            WallSequence::OuterInner => {
                // C++: entities.reverse()
                result.entities.entities.reverse();
            }
            WallSequence::InnerOuter => {
                // C++: no change — traverse_loops order is already inner-first
            }
            WallSequence::InnerOuterInner => {
                // C++ PerimeterGenerator.cpp:1255-1273: move elrSecondPerimeter loops to
                // just after their corresponding ExternalPerimeter loop.
                if result.entities.entities.len() > 1 {
                    let mut reordered: Vec<crate::extrusion_entity::ExtrusionEntityType> =
                        Vec::new();
                    let mut second_wall_buf: Vec<crate::extrusion_entity::ExtrusionEntityType> =
                        Vec::new();
                    for entity in result.entities.entities.drain(..) {
                        let is_second_peri = match &entity {
                            ExtrusionEntityType::Loop(l) => l.loop_role.contains(
                                crate::extrusion_entity::ExtrusionLoopRole::SECOND_PERIMETER,
                            ),
                            _ => false,
                        };
                        let is_external = match &entity {
                            ExtrusionEntityType::Loop(l) => l
                                .paths
                                .first()
                                .map(|p| p.role == ExtrusionRole::ExternalPerimeter)
                                .unwrap_or(false),
                            _ => false,
                        };
                        if is_second_peri {
                            second_wall_buf.push(entity);
                        } else {
                            reordered.push(entity);
                            if is_external && !second_wall_buf.is_empty() {
                                reordered.extend(second_wall_buf.drain(..));
                            }
                        }
                    }
                    result.entities.entities = reordered;
                }
            }
        }

        // PerimeterGenerator.cpp:1280-1318 — add LoopNodes for the outer walls
        // (z-direction speed continuity). One node per outwall path; for the
        // multi-path case each non-inner entity's first point is matched onto
        // an outwall contour (Point::is_in_lines) and the node records the
        // entity index as loop_id.
        if !outwall_paths.is_empty() && self.config.layer_id > 0 {
            use crate::extrusion_entity::ExtrusionRole;
            let scaled_eps = crate::libslic3r::SCALED_EPSILON as crate::Coord;
            let base = result.loop_nodes.len();
            if outwall_paths.len() == 1 {
                let mut bb = crate::geometry::BoundingBox::from_points(&outwall_paths[0].pts);
                bb.expand(scaled_eps);
                let mut node = crate::layer::LoopNode::new(base, 0, bb);
                node.node_contour = outwall_paths[0].clone();
                result.loop_nodes.push(node);
            } else {
                let mut matched = vec![false; outwall_paths.len()];
                for entity_idx in 0..result.entities.entities.len() {
                    // skip inner wall (C++: role() == erPerimeter)
                    if entity_role(&result.entities.entities[entity_idx])
                        == Some(ExtrusionRole::Perimeter)
                    {
                        continue;
                    }
                    let first_point =
                        match entity_first_point(&result.entities.entities[entity_idx]) {
                            Some(p) => p,
                            None => continue,
                        };
                    for lines_idx in 0..outwall_paths.len() {
                        if matched[lines_idx] {
                            continue;
                        }
                        if point_is_in_lines(&first_point, &outwall_paths[lines_idx].pts) {
                            matched[lines_idx] = true;
                            let mut bb = crate::geometry::BoundingBox::from_points(
                                &outwall_paths[lines_idx].pts,
                            );
                            bb.expand(scaled_eps);
                            let mut node = crate::layer::LoopNode::new(
                                result.loop_nodes.len(),
                                entity_idx,
                                bb,
                            );
                            node.node_contour = outwall_paths[lines_idx].clone();
                            result.loop_nodes.push(node);
                            break;
                        }
                    }
                }
            }
            result.entities.loop_node_range = (base, result.loop_nodes.len());
        }

        if std::env::var("PERIDBG").is_ok() {
            use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
            for ee in &result.entities.entities {
                if let ExtrusionEntityType::Loop(el) = ee {
                    if el.role() != ExtrusionRole::ExternalPerimeter {
                        continue;
                    }
                    let mut h: u64 = 1469598103934665603;
                    let mut np = 0usize;
                    for pa in &el.paths {
                        for pt in pa.polyline.points() {
                            h ^= pt.x as u64;
                            h = h.wrapping_mul(1099511628211);
                            h ^= pt.y as u64;
                            h = h.wrapping_mul(1099511628211);
                            np += 1;
                        }
                    }
                    eprintln!(
                        "PERIENT-R layer={} npaths={} npts={} h={:x}",
                        self.config.layer_id,
                        el.paths.len(),
                        np,
                        h
                    );
                }
            }
        }

        // Apply fuzzy skin as post-processing on perimeter entities
        if self.config.fuzzy_skin_mode != crate::region_config::FuzzySkinMode::None {
            let fs_config = crate::fuzzy_skin::FuzzySkinConfig {
                thickness: self.config.fuzzy_skin_thickness,
                point_distance: self.config.fuzzy_skin_point_distance,
                mode: self.config.fuzzy_skin_mode,
            };
            for entity in &mut result.entities.entities {
                apply_fuzzy_skin_to_entity(entity, &fs_config, self.config.fuzzy_skin_mode);
            }
        }

        // PerimeterGenerator.cpp:1378-1413 — infill boundary inset + top_fills merge.
        if top_fills_gate() {
            if crate::faithful_gate("GAPTRIM_PRETAIL") {
                result.tail_inputs.push(ClassicTailInput {
                    last: last.clone(),
                    top_fills: top_fills.clone(),
                    fill_clip: fill_clip.clone(),
                    loop_number,
                });
            }
            let (infill, no_overlap, top_band) =
                self.classic_fill_boundary_tail(&last, &top_fills, &fill_clip, loop_number);
            result.top_band = top_band;
            result.infill_area = infill;
            result.no_overlap_area = no_overlap;
        } else {
            // Legacy divergent path (default until the faithful gauges land): the raw
            // innermost-perimeter region without the C++ 1378-1406 inset.
            result.infill_area = last;
        }

        // PerimeterGenerator.cpp:952
        // C++: gaps collected during generation
        result.gap_fills = gaps;

        result
    }

    /// R253: the classic fill-boundary tail (PerimeterGenerator.cpp:1378-1430)
    /// as a reusable method — (infill_area, no_overlap_area, top_band). The
    /// layer-level gap block re-runs it on the gap-trimmed `last` (native
    /// order: PG.cpp:1373 trims BEFORE this tail).
    pub fn classic_fill_boundary_tail(
        &self,
        last: &[ExPolygon],
        top_fills: &[ExPolygon],
        fill_clip: &[ExPolygon],
        loop_number: usize,
    ) -> (ExPolygons, ExPolygons, ExPolygons) {
        let ext_perimeter_spacing = self.config.external_perimeter_spacing;
        let perimeter_spacing = self.config.perimeter_spacing;
        let mut top_band_out: ExPolygons = Vec::new();
        {
            let mut inset = if loop_number == 0 {
                ext_perimeter_spacing / 2.0
            } else {
                perimeter_spacing / 2.0
            };

            // PerimeterGenerator.cpp:1389-1394
            // C++: coord_t infill_peri_overlap = 0;
            // C++: if (inset > 0) {
            // C++:     infill_peri_overlap = coord_t(scale_(this->config->infill_wall_overlap.get_abs_value(
            // C++:         unscale<double>(inset + solid_infill_spacing / 2))));
            // C++:     inset -= infill_peri_overlap;
            // C++: }
            let mut infill_peri_overlap = 0.0;
            if inset > 0.0 {
                infill_peri_overlap = self.config.infill_wall_overlap
                    * (inset + self.config.solid_infill_spacing / 2.0);
                inset -= infill_peri_overlap;
            }

            // R722 SHIPPED ON. The FBIS bisect proved this is the exact point at
            // which the fill boundary diverges: `last` and `not_filled_exp` are
            // bit-identical to C++ on 35/35 aligned records, while `infill_exp`
            // was bit-identical on 0/35. Computing the offset2_ex deltas the way
            // native does — on truncated coord_t integers, not mm doubles —
            // takes `infill_exp` to 35/35 and `polyWithoutOverlap` to 33/35.
            // Scored: benchy classic +146 in-order, cube +2, arachne 0, majora 0
            // (both of those take the Arachne wall path, which does not run this
            // tail). FILLBOUND_QUANT=0 restores the old mm-double arithmetic.
            //
            // R285 (TOPFILL_FAITHFUL): native computes the whole tail on coord_t
            // ints (PG.cpp:1383-1430): inset = spacing/2 INT div; overlap =
            // trunc(ratio * ((inset + S_si/2_int) * 1e-5) / 1e-5); min_pis =
            // trunc(S_si*0.6). Division domains are load-bearing: min_pis/2. is
            // DOUBLE (keeps .5) in the offset2 deltas but min_pis/2 is INT in
            // the no-overlap branch + condition; the top offsets are double()
            // of ints with NO f32.
            let ext_sp_c = (ext_perimeter_spacing / 0.00001).trunc();
            let per_sp_c = (perimeter_spacing / 0.00001).trunc();
            let s_si_c = (self.config.solid_infill_spacing / 0.00001).trunc();
            let mut inset_c = if loop_number == 0 {
                (ext_sp_c / 2.0).trunc()
            } else {
                (per_sp_c / 2.0).trunc()
            };
            let mut overlap_c = 0.0f64;
            if inset_c > 0.0 {
                let v_unscaled = (inset_c + (s_si_c / 2.0).trunc()) * 0.00001;
                overlap_c = ((self.config.infill_wall_overlap * v_unscaled) / 0.00001).trunc();
                inset_c -= overlap_c;
            }
            let min_pis_c = (s_si_c * (1.0 - INSET_OVERLAP_TOLERANCE)).trunc();

            // PerimeterGenerator.cpp:1395-1399
            // C++: Polygons pp;
            // C++: for (ExPolygon &ex : last) ex.simplify_p(m_scaled_resolution, &pp);
            // C++: ExPolygons not_filled_exp = union_ex(pp);
            // R722 — bisect the fill-boundary chain at EXACT coordinate
            // resolution. Mirrors the C++ FBIS probe injected at the same four
            // statements (PerimeterGenerator.cpp:1395-1428).
            fbis_dump("last", self.config.layer_id, last);
            noprobe_dump("LASTPROBE", self.config.layer_id, last);

            let mut pp: Vec<Polygon> = Vec::new();
            // Native `ex.simplify_p(m_scaled_resolution, &pp)` — the tolerance is
            // SCALED units and the DP distance is the pure-double perpendicular
            // (R122 class: the legacy call passes the UNSCALED mm tolerance into a
            // rounded-projection DP — near no-op simplify, keeps extra points; L23
            // fill_expoly 29pts vs native 24). Gated faithful.
            let tf_simplify = crate::faithful_gate("TOPFILL_FAITHFUL");
            for ex in last {
                if tf_simplify {
                    let rings = ex.simplify_p_dp_rings_faithful(
                        self.config.surface_simplify_resolution / 0.00001,
                    );
                    // R814 TAIL_SIMPLIFY_POLYGONS (faithful ON): ExPolygon::simplify_p
                    // ends in `return simplify_polygons(pp)` (ExPolygon.cpp), the
                    // ClipperLib SimplifyPolygons pass that resolves the
                    // self-intersections Douglas-Peucker leaves. generate_classic_one
                    // has had it since R100; this call site had only the rings.
                    if crate::faithful_gate("TAIL_SIMPLIFY_POLYGONS") {
                        pp.extend(crate::clipper_utils::simplify_polygons_clib(&rings, 1));
                    } else {
                        pp.extend(rings);
                    }
                } else {
                    ex.simplify_p_into(self.config.surface_simplify_resolution, &mut pp);
                }
            }
            // Native union_ex(pp) runs ClipperLib @1e5; geo union grids to 1um and
            // everything downstream (infill_area, no_overlap_area, the top-surface
            // raster clip) inherits it (R100 class). Gated full-res.
            let f1_infill = crate::faithful_gate("TOPFILL_FAITHFUL");
            let not_filled_exp = if f1_infill {
                crate::clipper_utils::union_ex_clib(&pp, 1)
            } else {
                union_polygons_ex(&pp)
            };
            fbis_dump("notfilled", self.config.layer_id, &not_filled_exp);
            noprobe_dump("NFPROBE", self.config.layer_id, &not_filled_exp);
            // PerimeterGenerator.cpp:1400-1406
            // C++: coord_t min_perimeter_infill_spacing = coord_t(solid_infill_spacing * (1. - INSET_OVERLAP_TOLERANCE));
            // C++: ExPolygons infill_exp = offset2_ex(not_filled_exp,
            // C++:     float(-inset - min_perimeter_infill_spacing / 2.),
            // C++:     float(min_perimeter_infill_spacing / 2.));
            let min_perimeter_infill_spacing =
                self.config.solid_infill_spacing * (1.0 - INSET_OVERLAP_TOLERANCE);
            let mut infill_exp = if f1_infill {
                if crate::faithful_gate("FILLBOUND_QUANT") {
                    crate::clipper_utils::offset2_ex_clib(
                        &not_filled_exp,
                        -(inset_c + min_pis_c / 2.0) / SCALING_FACTOR,
                        (min_pis_c / 2.0) / SCALING_FACTOR,
                        self.config.join_type,
                    )
                } else {
                    crate::clipper_utils::offset2_ex_clib(
                        &not_filled_exp,
                        -(inset + min_perimeter_infill_spacing / 2.0),
                        min_perimeter_infill_spacing / 2.0,
                        self.config.join_type,
                    )
                }
            } else {
                offset2(
                    &not_filled_exp,
                    inset + min_perimeter_infill_spacing / 2.0,
                    min_perimeter_infill_spacing / 2.0,
                    self.config.join_type,
                )
            };

            fbis_dump("infillraw", self.config.layer_id, &infill_exp);
            fbis_dump("topfills", self.config.layer_id, &top_fills);
            fbis_dump("fillclip", self.config.layer_id, &fill_clip);

            // PerimeterGenerator.cpp:1407-1413
            // C++: ExPolygons top_infill_exp = intersection_ex(fill_clip, offset_ex(top_fills, double(ext_perimeter_spacing / 2)));
            // C++: if (!top_fills.empty()) {
            // C++:     infill_exp = union_ex(infill_exp, offset_ex(top_infill_exp, double(infill_peri_overlap)));
            // C++: }
            let top_infill_exp = if f1_infill {
                crate::clipper_utils::intersection_clib(
                    &fill_clip,
                    &if crate::faithful_gate("FILLBOUND_QUANT") {
                        crate::clipper_utils::offset_expolygons_clib_scaled(
                            &top_fills,
                            (ext_sp_c / 2.0).trunc(),
                            OffsetJoinType::Miter,
                        )
                    } else {
                        crate::clipper_utils::offset_expolygons_clib(
                            &top_fills,
                            ext_perimeter_spacing / 2.0,
                            OffsetJoinType::Miter,
                        )
                    },
                )
            } else {
                intersection(
                    &fill_clip,
                    &offset_expolygons(
                        &top_fills,
                        ext_perimeter_spacing / 2.0,
                        OffsetJoinType::Miter,
                    ),
                )
            };
            if !top_fills.is_empty() {
                if f1_infill {
                    let grown = if crate::faithful_gate("FILLBOUND_QUANT") {
                        crate::clipper_utils::offset_expolygons_clib_scaled(
                            &top_infill_exp,
                            overlap_c,
                            OffsetJoinType::Miter,
                        )
                    } else {
                        crate::clipper_utils::offset_expolygons_clib(
                            &top_infill_exp,
                            infill_peri_overlap,
                            OffsetJoinType::Miter,
                        )
                    };
                    top_band_out = grown.clone();
                    let mut merged: Vec<Polygon> = crate::geometry::to_polygons(&infill_exp);
                    merged.extend(crate::geometry::to_polygons(&grown));
                    infill_exp = crate::clipper_utils::union_ex_clib(&merged, 1);
                } else {
                    let mut merged = infill_exp;
                    merged.extend(offset_expolygons(
                        &top_infill_exp,
                        infill_peri_overlap,
                        OffsetJoinType::Miter,
                    ));
                    infill_exp = union_ex(&merged);
                }
            }
            fbis_dump("topinfill", self.config.layer_id, &top_infill_exp);
            fbis_dump("final", self.config.layer_id, &infill_exp);
            crate::stage_dump::dump("tail_infill", self.config.layer_id, &infill_exp);

            // R712 — R711 proved the fill PATTERN is faithful (angle, spacing,
            // grid phase and total extruded length all match C++) while only
            // 2.7-42% of fill VERTICES match, which means every line is clipped
            // against a different boundary. This prints that boundary at the
            // moment it is committed, against C++ `PerimeterGenerator.cpp`
            // (`this->fill_surfaces->append(infill_exp, stInternal)`, FILLB).
            // Count/area apart from bbox: a missing region and a wrong inset
            // need opposite fixes.
            if crate::probe_enabled("FILLB") {
                let mut area = 0.0f64;
                let bb: Vec<String> = infill_exp
                    .iter()
                    .map(|e| {
                        area += e.area();
                        let b = e.bounding_box();
                        format!("{},{},{},{}", b.min.x, b.min.y, b.max.x, b.max.y)
                    })
                    .collect();
                eprintln!(
                    "[FILLB] layer={} n={} bb={} area={:.0}",
                    self.config.layer_id,
                    infill_exp.len(),
                    bb.join(";"),
                    area
                );
            }

            let infill_out = infill_exp;

            // PerimeterGenerator.cpp:1415-1430 — BBS: get the no-overlap infill
            // expolygons. Same not_filled_exp, but WITHOUT the infill_peri_overlap
            // grow (so the band the top-surface monotonic lines occupy is exact).
            //   if (min_perimeter_infill_spacing/2 > infill_peri_overlap)
            //       polyWithoutOverlap = offset2_ex(not_filled_exp,
            //           -inset - min_perimeter_infill_spacing/2,
            //            min_perimeter_infill_spacing/2 - infill_peri_overlap);
            //   else
            //       polyWithoutOverlap = offset_ex(not_filled_exp, -inset - infill_peri_overlap);
            //   if (!top_fills.empty()) polyWithoutOverlap = union_ex(polyWithoutOverlap, top_infill_exp);
            // Native offset2_ex/offset_ex run ClipperLib @1e5; the geo variants
            // grid the no-overlap contour to 1um, which the top-surface
            // MonotonicLine raster clips against (R100 class). Gated full-res.
            let f1 = crate::faithful_gate("TOPFILL_FAITHFUL");
            // Native condition: min_pis/2 (INT div) > overlap (PG.cpp:1418).
            let no_cond = if f1 && crate::faithful_gate("FILLBOUND_QUANT") {
                (min_pis_c / 2.0).trunc() > overlap_c
            } else {
                min_perimeter_infill_spacing / 2.0 > infill_peri_overlap
            };
            let mut poly_without_overlap = if no_cond {
                if f1 {
                    if crate::faithful_gate("FILLBOUND_QUANT") {
                        crate::clipper_utils::offset2_ex_clib(
                            &not_filled_exp,
                            -(inset_c + min_pis_c / 2.0) / SCALING_FACTOR,
                            ((min_pis_c / 2.0).trunc() - overlap_c) / SCALING_FACTOR,
                            self.config.join_type,
                        )
                    } else {
                        crate::clipper_utils::offset2_ex_clib(
                            &not_filled_exp,
                            -(inset + min_perimeter_infill_spacing / 2.0),
                            min_perimeter_infill_spacing / 2.0 - infill_peri_overlap,
                            self.config.join_type,
                        )
                    }
                } else {
                    offset2(
                        &not_filled_exp,
                        inset + min_perimeter_infill_spacing / 2.0,
                        min_perimeter_infill_spacing / 2.0 - infill_peri_overlap,
                        self.config.join_type,
                    )
                }
            } else if f1 {
                if crate::faithful_gate("FILLBOUND_QUANT") {
                    crate::clipper_utils::offset_expolygons_clib_scaled(
                        &not_filled_exp,
                        -(inset_c + overlap_c),
                        self.config.join_type,
                    )
                } else {
                    crate::clipper_utils::offset_expolygons_clib(
                        &not_filled_exp,
                        -(inset + infill_peri_overlap),
                        self.config.join_type,
                    )
                }
            } else {
                offset_expolygons(
                    &not_filled_exp,
                    -(inset + infill_peri_overlap),
                    self.config.join_type,
                )
            };
            if !top_fills.is_empty() {
                poly_without_overlap = if f1 {
                    let mut merged: Vec<crate::geometry::Polygon> =
                        crate::geometry::to_polygons(&poly_without_overlap);
                    merged.extend(crate::geometry::to_polygons(&top_infill_exp));
                    crate::clipper_utils::union_ex_clib(&merged, 1)
                } else {
                    let mut merged = poly_without_overlap;
                    merged.extend(top_infill_exp);
                    union_ex(&merged)
                };
            }
            crate::stage_dump::dump("tail_nooverlap", self.config.layer_id, &poly_without_overlap);
            fbis_dump("nooverlap", self.config.layer_id, &poly_without_overlap);
            if crate::probe_enabled("NOPROBE") {
                for e in poly_without_overlap.iter() {
                    let mut iv: i64 = e.contour.points.len() as i64;
                    let mut ix: i64 = 0;
                    let mut iy: i64 = 0;
                    for pt in &e.contour.points {
                        ix += pt.x();
                        iy += pt.y();
                    }
                    for h in &e.holes {
                        iv += h.points.len() as i64;
                        for pt in &h.points {
                            ix += pt.x();
                            iy += pt.y();
                        }
                    }
                    eprintln!(
                        "NOPROBE lid={} holes={} iv={} ix={} iy={}",
                        self.config.layer_id,
                        e.holes.len(),
                        iv,
                        ix,
                        iy
                    );
                }
            }

            // R715 — `fill_no_overlap` is what `FillConcentricInternal` actually
            // iterates (`no_overlap_expolygons`), NOT the `fill_surfaces` region
            // R712 probed. Print it against C++ `PerimeterGenerator.cpp`'s
            // `this->fill_no_overlap->insert(...)` (FILLNOV).
            if crate::probe_enabled("FILLNOV") {
                let mut area = 0.0f64;
                let bb: Vec<String> = poly_without_overlap
                    .iter()
                    .map(|e| {
                        area += e.area();
                        let b = e.bounding_box();
                        format!("{},{},{},{}", b.min.x, b.min.y, b.max.x, b.max.y)
                    })
                    .collect();
                eprintln!(
                    "[FILLNOV] layer={} n={} bb={} area={:.0}",
                    self.config.layer_id,
                    poly_without_overlap.len(),
                    bb.join(";"),
                    area
                );
            }

            return (infill_out, poly_without_overlap, top_band_out);
        }
    }
}

impl Default for PerimeterGenerator {
    fn default() -> Self {
        Self::new(PerimeterConfig::default())
    }
}

/// Convert ThickPolylines to ExtrusionPaths with variable width
/// VariableWidth.cpp:214-230
/// C++: void variable_width(const ThickPolylines& polylines, ExtrusionRole role, const Flow& flow, std::vector<ExtrusionEntity*>& out)
// Entity-level accessors for the LoopNode block (PG.cpp:1291-1303) — the C++
// calls `ee->role()` / `ee->first_point()` through the ExtrusionEntity vtable.
fn entity_role(
    ee: &crate::extrusion_entity::ExtrusionEntityType,
) -> Option<crate::extrusion_entity::ExtrusionRole> {
    use crate::extrusion_entity::ExtrusionEntityType;
    match ee {
        ExtrusionEntityType::Path(p) => Some(p.role),
        ExtrusionEntityType::MultiPath(m) => Some(m.role()),
        ExtrusionEntityType::Loop(l) => Some(l.role()),
        // Collection role is "mixed" in C++ — never equal to erPerimeter.
        ExtrusionEntityType::Collection(_) => None,
    }
}

fn entity_first_point(ee: &crate::extrusion_entity::ExtrusionEntityType) -> Option<Point> {
    use crate::extrusion_entity::ExtrusionEntityType;
    match ee {
        ExtrusionEntityType::Path(p) => {
            if p.polyline.points().is_empty() {
                None
            } else {
                Some(p.first_point())
            }
        }
        ExtrusionEntityType::Loop(l) => {
            if l.paths.is_empty() || l.paths[0].polyline.points().is_empty() {
                None
            } else {
                Some(l.first_point())
            }
        }
        ExtrusionEntityType::MultiPath(m) => {
            if m.paths.is_empty() || m.paths[0].polyline.points().is_empty() {
                None
            } else {
                Some(m.first_point())
            }
        }
        ExtrusionEntityType::Collection(c) => c.first_point(),
    }
}

// Point::is_in_lines (Point.cpp:183-223) — true when the point lies on any
// segment of the polyline `pts` (endpoint hit, exact vertical/horizontal hit,
// or within SCALED_EPSILON of the segment).
fn point_is_in_lines(check: &Point, pts: &[Point]) -> bool {
    const SCALED_EPS: f64 = 10.0; // libslic3r.h SCALED_EPSILON
    for pt_idx in 1..pts.len() {
        let pt = pts[pt_idx];
        let prev_pt = pts[pt_idx - 1];

        // on the endpoints
        if (check.x == pt.x && check.y == pt.y) || (check.x == prev_pt.x && check.y == prev_pt.y) {
            return true;
        }

        let in_x_range = !((check.x > pt.x) == (check.x > prev_pt.x));
        let in_y_range = !((check.y > pt.y) == (check.y > prev_pt.y));

        // on vert line
        if pt.x == prev_pt.x {
            if in_y_range && pt.x == check.x {
                return true;
            }
            continue;
        }

        // on hori line
        if pt.y == prev_pt.y {
            if in_x_range && pt.y == check.y {
                return true;
            }
            continue;
        }

        // not right range
        if !in_x_range || !in_y_range {
            continue;
        }

        // check if in line — C++ Line::distance_to (double, segment-clamped)
        let (ax, ay) = (prev_pt.x as f64, prev_pt.y as f64);
        let (bx, by) = (pt.x as f64, pt.y as f64);
        let (px, py) = (check.x as f64, check.y as f64);
        let (dx, dy) = (bx - ax, by - ay);
        let l2 = dx * dx + dy * dy;
        let dist = if l2 == 0.0 {
            ((px - ax).powi(2) + (py - ay).powi(2)).sqrt()
        } else {
            let t = ((px - ax) * dx + (py - ay) * dy) / l2;
            let t = t.clamp(0.0, 1.0);
            let (cx, cy) = (ax + t * dx, ay + t * dy);
            ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
        };
        if dist.abs() < SCALED_EPS {
            return true;
        }
    }
    false
}

pub(crate) fn convert_thin_walls_to_extrusion_paths(
    thick_polylines: &ThickPolylines,
    role: crate::extrusion_entity::ExtrusionRole,
    flow: &Flow,
) -> Vec<crate::extrusion_entity::ExtrusionPath> {
    

    let mut result = Vec::new();

    /// VariableWidth.cpp:217
    /// C++: const float tolerance = float(scale_(0.05));
    /// C++ ThickPolyline.width is in SCALED units, so it uses scale_(0.05). The crate's
    /// ThickPolyline.widths are stored in MM (see geometry/medial_axis.rs:367, which
    /// unscales each width before `tp.push`). The width-delta threshold is therefore
    /// expressed directly in mm here (0.05 mm), NOT scaled.
    let tolerance: f64 = 0.05;

    /// VariableWidth.cpp:218-228
    /// C++: for (const ThickPolyline& p : polylines) {
    for thick_polyline in thick_polylines {
        let paths = thick_polyline_to_extrusion_paths(thick_polyline, role, flow, tolerance);
        result.extend(paths);
    }

    result
}

/// Convert a single ThickPolyline to ExtrusionPaths
/// VariableWidth.cpp:132-211 (thick_polyline_to_extrusion_paths_2)
fn thick_polyline_to_extrusion_paths(
    thick_polyline: &ThickPolyline,
    role: crate::extrusion_entity::ExtrusionRole,
    flow: &Flow,
    tolerance: f64,
) -> Vec<crate::extrusion_entity::ExtrusionPath> {
    use crate::extrusion_entity::ExtrusionPath;
    use crate::geometry::ThickLine;
    use crate::libslic3r::SCALED_EPSILON;

    let mut paths = Vec::new();
    // VariableWidth.cpp:105 — ThickLines lines = thick_polyline.thicklines();
    // Mutable: the segment-splitting step below erases/inserts lines in place.
    let mut lines = thick_polyline.thicklines();

    // VariableWidth.cpp:107-108
    // C++: size_t start_index = 0; double max_width, min_width;
    let mut start_index: usize = 0;
    let mut max_width: f64 = 0.0;
    let mut min_width: f64 = 0.0;

    /// VariableWidth.cpp:110-189
    /// C++: for (int i = 0; i < (int)lines.size(); ++i)
    let mut i: i64 = 0;
    while (i as usize) < lines.len() {
        let line = lines[i as usize].clone();

        // VariableWidth.cpp:113-116
        // C++: if (i == 0) { max_width = line.a_width; min_width = line.a_width; }
        if i == 0 {
            max_width = line.a_width;
            min_width = line.a_width;
        }

        // VariableWidth.cpp:118-119
        // C++: const coordf_t line_len = line.length(); if (line_len < SCALED_EPSILON) continue;
        let line_len = line.length();
        if line_len < SCALED_EPSILON {
            i += 1;
            continue;
        }

        // VariableWidth.cpp:121
        // C++: double thickness_delta = std::max(fabs(max_width - line.b_width), fabs(min_width - line.b_width));
        let mut thickness_delta =
            (max_width - line.b_width).abs().max((min_width - line.b_width).abs());

        /// VariableWidth.cpp:123
        /// C++: if (thickness_delta > tolerance)
        if thickness_delta > tolerance {
            // VariableWidth.cpp:124-142
            // C++: 1 generate path from start_index to i (not included)
            if start_index != i as usize {
                let mut path = ExtrusionPath::new(role);
                let mut length = 0.0;
                let mut sum = 0.0;

                for idx in start_index..(i as usize) {
                    let l = lines[idx].length();
                    length += l;
                    sum += l * 0.5 * (lines[idx].a_width + lines[idx].b_width);
                    path.polyline.points.push(lines[idx].a);
                }
                path.polyline.points.push(lines[i as usize].a);

                if length > SCALED_EPSILON {
                    // VariableWidth.cpp:135-136 — w = sum/length; flow.with_width(unscale(w) + ...).
                    // C++ widths are SCALED so it unscales `w`. Here the crate's ThickPolyline
                    // widths are already in MM (geometry/medial_axis.rs:367), and `length` is the
                    // scaled segment length which cancels in `sum/length`, so `w` is ALREADY mm.
                    // Re-dividing by SCALING_FACTOR (the prior bug) collapsed widths to ~0,
                    // making gap-fill extrude ~100x too little.
                    let w_mm = sum / length;
                    let new_width = if crate::faithful_gate("UNSCALE_FLOAT") {
                // `unscale<float>(w)` of the scaled width (VariableWidth.cpp:136, :203).
                crate::variable_width::spacing_to_width_scaled(w_mm * crate::SCALING_FACTOR, flow.height())
            } else {
                crate::variable_width::spacing_to_width(w_mm, flow.height())
            };
                    if let Ok(new_flow) = flow.with_width(new_width) {
                        path.mm3_per_mm = new_flow.mm3_per_mm().unwrap_or(0.0);
                        path.width = new_flow.width();
                        path.height = new_flow.height();
                        paths.push(path);
                    }
                }
            }

            // VariableWidth.cpp:144-146
            start_index = i as usize;
            max_width = line.a_width;
            min_width = line.a_width;

            // VariableWidth.cpp:148-182
            // C++: 2 handle the i-th segment — subdivide if internal width delta is large.
            thickness_delta = (line.a_width - line.b_width).abs();
            if thickness_delta > tolerance {
                // C++: segments = (unsigned int)ceil(thickness_delta / tolerance);
                let segments = (thickness_delta / tolerance).ceil() as usize;
                let seg_len = line_len / segments as f64;

                let mut pp: Vec<Point> = Vec::new();
                let mut width: Vec<f64> = Vec::new();

                // C++: pp.push_back(line.a); width.push_back(line.a_width);
                pp.push(line.a);
                width.push(line.a_width);

                let dx = (line.b.x - line.a.x) as f64;
                let dy = (line.b.y - line.a.y) as f64;
                let dlen = (dx * dx + dy * dy).sqrt();
                let (nx, ny) = if dlen > 0.0 { (dx / dlen, dy / dlen) } else { (0.0, 0.0) };

                for j in 1..segments {
                    // C++: pp.push_back((line.a + (line.b - line.a).normalized() * (j*seg_len)).cast<coord_t>());
                    let off = j as f64 * seg_len;
                    let px = line.a.x as f64 + nx * off;
                    let py = line.a.y as f64 + ny * off;
                    pp.push(Point::new(px as Coord, py as Coord));

                    // C++: w = line.a_width + (j*seg_len) * (line.b_width - line.a_width) / line_len;
                    let w = line.a_width + off * (line.b_width - line.a_width) / line_len;
                    width.push(w);
                    width.push(w);
                }

                // C++: pp.push_back(line.b); width.push_back(line.b_width);
                pp.push(line.b);
                width.push(line.b_width);

                // C++: lines.erase(lines.begin() + i);
                lines.remove(i as usize);
                // C++: for (j=0; j<segments; ++j) insert new_line at i+j
                for j in 0..segments {
                    // C++: ThickLine new_line(pp[j], pp[j+1]); new_line.a_width=...; new_line.b_width=...;
                    let new_line = ThickLine::new(pp[j], pp[j + 1], width[2 * j], width[2 * j + 1]);
                    lines.insert(i as usize + j, new_line);
                }
                // C++: --i; continue; — the for-loop's ++i then re-lands on the
                // first newly-inserted segment. Equivalent here: decrement, then
                // fall through to the `i += 1` at the bottom of the loop.
                i -= 1;
            }
        } else {
            // VariableWidth.cpp:185-188
            // C++: just update the max and min width and continue
            max_width = max_width.max(line.a_width.max(line.b_width));
            min_width = min_width.min(line.a_width.min(line.b_width));
        }

        i += 1;
    }

    /// VariableWidth.cpp:190-209 — handle the remaining segment
    let final_size = lines.len();
    if start_index < final_size {
        let mut path = ExtrusionPath::new(role);
        let mut length = 0.0;
        let mut sum = 0.0;

        for idx in start_index..final_size {
            let l = lines[idx].length();
            length += l;
            sum += l * (lines[idx].a_width + lines[idx].b_width) * 0.5;
            path.polyline.points.push(lines[idx].a);
        }
        path.polyline.points.push(lines[final_size - 1].b);

        if length > SCALED_EPSILON {
            // VariableWidth.cpp:202-203 — w already in MM (crate ThickPolyline widths are
            // mm; the scaled `length` cancels in sum/length). See the matching note above.
            let w_mm = sum / length;
            let new_width = if crate::faithful_gate("UNSCALE_FLOAT") {
                // `unscale<float>(w)` of the scaled width (VariableWidth.cpp:136, :203).
                crate::variable_width::spacing_to_width_scaled(w_mm * crate::SCALING_FACTOR, flow.height())
            } else {
                crate::variable_width::spacing_to_width(w_mm, flow.height())
            };
            if let Ok(new_flow) = flow.with_width(new_width) {
                path.mm3_per_mm = new_flow.mm3_per_mm().unwrap_or(0.0);
                path.width = new_flow.width();
                path.height = new_flow.height();
                paths.push(path);
            }
        }
    }

    paths
}

/// Reverse an `ExtrusionEntityType` in place — the analogue of C++
/// `ExtrusionEntity::reverse()` dispatched on the runtime type. Used to honor
/// the reversal flag returned by `chain_extrusion_entities` for thin walls
/// (PerimeterGenerator.cpp:487-488). Loops intentionally do nothing
/// (ExtrusionEntityCollection.cpp:67-75 semantics: reversing a loop is a no-op).
fn entity_reverse_inplace(entity: &mut ExtrusionEntityType) {
    match entity {
        ExtrusionEntityType::Path(p) => p.reverse(),
        ExtrusionEntityType::MultiPath(m) => m.reverse(),
        ExtrusionEntityType::Loop(_) => {}
        ExtrusionEntityType::Collection(c) => c.reverse(),
    }
}

/// PerimeterGenerator.cpp:241-267 — detect_bridge_wall.
/// Routes the 100%-overhang remain polylines into overhang/bridge walls:
/// - if the straight line first->last is shorter than the polyline (i.e. the
///   wall is curved), degree = overhang_sampling_number - 1 (5).
/// - else (straight) it is a bridge wall, degree = overhang_sampling_number (6).
fn detect_bridge_wall(
    paths: &mut Vec<crate::extrusion_entity::ExtrusionPath>,
    remain_polines: &[Polyline],
    role: crate::extrusion_entity::ExtrusionRole,
    mm3_per_mm: f64,
    width: f32,
    height: f32,
) {
    use crate::geometry::Line;
    let n = crate::overhang_detector::OVERHANG_SAMPLING_NUMBER as f64;
    for poly in remain_polines {
        // PerimeterGenerator.cpp:245-246 — Line line(poly.first_point(), poly.last_point());
        let line = Line::new(poly.first_point(), poly.last_point());
        // PerimeterGenerator.cpp:246 — if (line.length() < poly.length())
        let degree = if line.length() < poly.length() {
            // curved overhang wall
            n - 1.0
        } else {
            // bridge wall
            n
        };
        crate::overhang_detector::extrusion_paths_append(
            paths,
            vec![poly.clone()],
            degree,
            0,
            role,
            mm3_per_mm,
            width,
            height,
        );
    }
}

/// Convert perimeter loops to extrusion entities with proper ordering
/// PerimeterGenerator.cpp:280-503
/// C++: static ExtrusionEntityCollection traverse_loops(const PerimeterGenerator &perimeter_generator, const PerimeterGeneratorLoops &loops, ThickPolylines &thin_walls)
fn traverse_loops_legacy(
    loops: &[PerimeterLoop],
    thin_walls: &mut ThickPolylines,
    layer_height: f64,
    perimeter_flow: &Flow,
    ext_perimeter_flow: &Flow,
    config: &PerimeterConfig,
) -> crate::extrusion_entity::ExtrusionEntityCollection {
    use crate::extrusion_entity::{
        CustomizeFlag, ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop,
        ExtrusionLoopRole, ExtrusionPath, ExtrusionRole,
    };

    /// PerimeterGenerator.cpp:283
    /// C++: ExtrusionEntityCollection coll;
    let mut coll = ExtrusionEntityCollection::new();

    /// PerimeterGenerator.cpp:284-461
    /// C++: for (const PerimeterGeneratorLoop &loop : loops) {
    for loop_item in loops.iter() {
        /// PerimeterGenerator.cpp:285-287
        /// C++: bool is_external = loop.is_external();
        /// C++: bool is_small_width = loop.is_smaller_width_perimeter;
        let is_external = loop_item.is_external;
        let _is_small_width = loop_item.is_smaller_width;

        /// PerimeterGenerator.cpp:288
        /// C++: CustomizeFlag flag = loop.need_circle_compensation ? CustomizeFlag::cfCircleCompensation : CustomizeFlag::cfNone;
        let flag = CustomizeFlag::None; // TODO: Port circle compensation

        /// PerimeterGenerator.cpp:290-302
        /// C++: ExtrusionRole role;
        /// C++: ExtrusionLoopRole loop_role;
        /// C++: role = is_external ? erExternalPerimeter : erPerimeter;
        let role = if is_external {
            ExtrusionRole::ExternalPerimeter
        } else {
            ExtrusionRole::Perimeter
        };

        /// PerimeterGenerator.cpp:293-302
        /// C++: if (loop.is_internal_contour()) {
        /// C++:     loop_role = elrContourInternalPerimeter;
        /// C++: } else {
        /// C++:     loop_role = loop.is_contour? elrDefault: elrPerimeterHole;
        /// C++: }
        let mut loop_role = if loop_item.is_internal_contour() {
            ExtrusionLoopRole::CONTOUR_INTERNAL_PERIMETER
        } else if loop_item.is_contour {
            ExtrusionLoopRole::DEFAULT
        } else {
            ExtrusionLoopRole::PERIMETER_HOLE
        };

        /// PerimeterGenerator.cpp:304-308
        /// C++: if( loop.depth == 1 ) {
        /// C++:     if (loop_role == elrDefault)
        /// C++:         loop_role = elrSecondPerimeter;
        /// C++:     else
        /// C++:         loop_role = loop_role | elrSecondPerimeter;
        /// C++: }
        if loop_item.perimeter_index == 1 {
            if loop_role == ExtrusionLoopRole::DEFAULT {
                loop_role = ExtrusionLoopRole::SECOND_PERIMETER;
            } else {
                loop_role = loop_role | ExtrusionLoopRole::SECOND_PERIMETER;
            }
        }

        /// PerimeterGenerator.cpp:310
        /// C++: ExtrusionPaths paths;
        let mut paths: Vec<ExtrusionPath> = Vec::new();

        /// PerimeterGenerator.cpp:312-340
        /// C++: const std::vector<Polygons> *lower_polygons_series;
        /// C++: const std::pair<double, double> *overhang_dist_boundary;
        /// C++: double extrusion_mm3_per_mm;
        /// C++: double extrusion_width;
        let flow = if is_external {
            ext_perimeter_flow
        } else {
            perimeter_flow
        };

        let extrusion_mm3_per_mm = flow.mm3_per_mm().unwrap_or(0.0);
        let extrusion_width = flow.width();

        /// PerimeterGenerator.cpp:342-344
        /// C++: const Polygon polygon = apply_fuzzy_skin(loop.polygon, *(perimeter_generator.config), ...);
        // Fuzzy skin is applied as a post-processing step on the perimeter entities
        // in PerimeterGenerator::generate() after traverse_loops returns.
        let polygon = &loop_item.polygon;

        /// PerimeterGenerator.cpp:346-450 — Overhang detection.
        ///
        /// Splits the loop into per-segment overhang-graded `ExtrusionPath`s and sets
        /// `overhang_degree`, so the gcode exporter can modulate speed per segment.
        ///
        /// The `m_lower_polygons_series` / `m_external_lower_polygons_series` thresholds
        /// and the `overhang_dist_boundary` pair are computed inline from the loop's flow
        /// (generate_lower_polygons_series() / dist_boundary(), see
        /// PerimeterGenerator.cpp:229-239,1841-1864). The nozzle diameter for the wall
        /// filament is read off the loop flow (built with that nozzle).
        ///
        /// DIVERGENCE (geometry-preserving substitute for the clipper split):
        /// C++ clips the loop against the grown lower-slice series via
        /// `intersection_pl_2`/`diff_pl_2` to separate zero-degree (inside lower_front),
        /// middle-overhang (between lower_front and lower_back), and 100%-overhang
        /// (outside lower_back) regions, then calls `detect_overhang_degree` on the
        /// middle region only. The crate's coarse sampling `intersection_pl`/`diff_pl`
        /// (F1) distort polyline length (≈0.3% filament loss) and the Clipper2 open-path
        /// `*_pl_2` drop most of a closed loop, so neither is usable for byte-stable
        /// filament. Instead we classify each ORIGINAL loop segment by its midpoint's
        /// membership in `lower_front` / `lower_back` (point-in-polygon, no geometry
        /// change), group contiguous segments of the same class, and split the loop at
        /// those class boundaries — preserving every original vertex (hence E). The
        /// middle-band sub-polylines are then graded by `detect_overhang_degree`
        /// exactly as in C++; inside→degree 0, outside→max degree. (Crucially this does
        /// NOT grade the whole loop: feeding fully-supported segments to the distancer
        /// would mis-classify deep-interior points as high overhang, since the distancer
        /// measures distance to the support BOUNDARY, not inside/outside.)
        ///
        /// DIVERGENCE (bridge classification): C++ routes the 100%-overhang region through
        /// `detect_bridge_wall` (re-flowed with the overhang flow, degree 5/6). Here the
        /// fully-unsupported sub-segments stay at `role` with overhang_degree = max
        /// (overhang_sampling_number-1 = 5) and the original flow, so filament is unchanged;
        /// straight-vs-curved bridge discrimination (degree 6 → bridge_speed) is not modeled.
        // R536 probe (OHSPLITPROBE=1): count each sub-condition of the overhang-split
        // gate, so a loop that stays single-path names its own cause.
        if crate::probe_enabled("OHSPLITPROBE") {
            use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
            static SEEN: AtomicUsize = AtomicUsize::new(0);
            static OK_DETECT: AtomicUsize = AtomicUsize::new(0);
            static OK_LAYER: AtomicUsize = AtomicUsize::new(0);
            static HAS_LOWER: AtomicUsize = AtomicUsize::new(0);
            static LOWER_NONEMPTY: AtomicUsize = AtomicUsize::new(0);
            let n = SEEN.fetch_add(1, Relaxed) + 1;
            if config.detect_overhang_wall {
                OK_DETECT.fetch_add(1, Relaxed);
            }
            if config.layer_id > config.raft_layers {
                OK_LAYER.fetch_add(1, Relaxed);
            }
            if let Some(ref lo) = config.lower_slices {
                HAS_LOWER.fetch_add(1, Relaxed);
                if !lo.is_empty() {
                    LOWER_NONEMPTY.fetch_add(1, Relaxed);
                }
            }
            if n % 1_000 == 0 || n == 1 {
                eprintln!(
                    "[OHSPLITPROBE] loops={n} detect={} layer_id>raft={} lower_some={} lower_nonempty={}",
                    OK_DETECT.load(Relaxed),
                    OK_LAYER.load(Relaxed),
                    HAS_LOWER.load(Relaxed),
                    LOWER_NONEMPTY.load(Relaxed),
                );
            }
        }
        let did_overhang_split = if config.detect_overhang_wall
            && config.layer_id > config.raft_layers
        {
            if let Some(ref lower) = config.lower_slices {
                if !lower.is_empty() {
                    // PerimeterGenerator.cpp:1841-1864 — generate_lower_polygons_series(width)
                    // offsets = [start + 0.5*(end-start)/(N-1), end]; start=-0.5*width; end=0.5*nozzle.
                    let nozzle_diameter = flow.nozzle_diameter();
                    let start_offset = -0.5 * extrusion_width;
                    let end_offset = 0.5 * nozzle_diameter;
                    let n = crate::overhang_detector::OVERHANG_SAMPLING_NUMBER as f64;
                    let off_front = start_offset + 0.5 * (end_offset - start_offset) / (n - 1.0);
                    // lower_polygons_series.front() = less-grown (off_front), .back() = more-grown (end_offset).
                    // R102c: native generate_lower_polygons_series grows the lower slice via
                    // ClipperLib offset at coord_t (10nm). Rust used geo-clipper (offset_expolygons
                    // @ GEO_CLIPPER_SCALE=1000 = 1 micron), so these overhang BOUNDARIES are gridded
                    // to 100 units. The outer perimeter is then split at those boundaries (vertices
                    // inserted where the wall crosses the overhang region) — gridded boundaries →
                    // gridded/shifted inserted vertices → outer CONTOUR diverges from native (holes
                    // rarely overhang, so they match ~2x better). Same class as R100. Under F1_UNION
                    // route the grows through the vertex-exact vendored ClipperLib (offset_expolygons_clib
                    // @ i32/1e5). Default path byte-unchanged.
                    let faithful_raw: Option<(Vec<Polygon>, Vec<Polygon>)> = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                        let width_f32 = extrusion_width as f32;
                        let nozzle_f32 = nozzle_diameter as f32;
                        let start_f32 = -0.5f32 * width_f32;
                        let end_f32 = 0.5f32 * nozzle_f32;
                        let n_f32 = crate::overhang_detector::OVERHANG_SAMPLING_NUMBER as f32;
                        let front_f32 = start_f32 + 0.5f32 * (end_f32 - start_f32) / (n_f32 - 1.0);
                        let native_delta = |v: f32| ((v as f64 / 0.00001) as f32) as f64;
                        // expolygons_offset (ClipperUtils.cpp:537-547): shrink →
                        // raw concat; grow with >1 collected → ONE flat NonZero union.
                        let front_raw = crate::clipper_utils::offset_expolygons_clib_raw_scaled(
                            lower, native_delta(front_f32), OffsetJoinType::Miter);
                        let (back_raw, back_collected) =
                            crate::clipper_utils::offset_expolygons_clib_raw_scaled_counted(
                                lower, native_delta(end_f32), OffsetJoinType::Miter);
                        let back_polys = if back_collected > 1 && end_f32 > 0.0 {
                            crate::clipper_utils::union_flat_clib(&back_raw)
                        } else {
                            back_raw
                        };
                        Some((front_raw, back_polys))
                    } else {
                        None
                    };
                    let (lower_front_ex, lower_back_ex) = if faithful_raw.is_some() {
                        (Vec::new(), Vec::new())
                    } else if crate::faithful_gate("F1_UNION") {
                        (
                            crate::clipper_utils::offset_expolygons_clib(lower, off_front, OffsetJoinType::Miter),
                            crate::clipper_utils::offset_expolygons_clib(lower, end_offset, OffsetJoinType::Miter),
                        )
                    } else {
                        (
                            offset_expolygons(lower, off_front, OffsetJoinType::Miter),
                            offset_expolygons(lower, end_offset, OffsetJoinType::Miter),
                        )
                    };

                    if std::env::var("PERIDBG").is_ok() {
                        let fnv = |ps: &[ExPolygon]| -> (u64, usize) {
                            let mut h: u64 = 1469598103934665603;
                            let mut n = 0usize;
                            for ex in ps {
                                for pt in ex.contour.points() {
                                    h ^= pt.x as u64; h = h.wrapping_mul(1099511628211);
                                    h ^= pt.y as u64; h = h.wrapping_mul(1099511628211);
                                }
                                n += 1;
                                for hole in &ex.holes {
                                    for pt in hole.points() {
                                        h ^= pt.x as u64; h = h.wrapping_mul(1099511628211);
                                        h ^= pt.y as u64; h = h.wrapping_mul(1099511628211);
                                    }
                                    n += 1;
                                }
                            }
                            (h, n)
                        };
                        let fnv_p = |ps: &[Polygon]| -> (u64, usize) {
                            let mut h: u64 = 1469598103934665603;
                            let mut n = 0usize;
                            for pg in ps {
                                for pt in pg.points() {
                                    h ^= pt.x as u64; h = h.wrapping_mul(1099511628211);
                                    h ^= pt.y as u64; h = h.wrapping_mul(1099511628211);
                                }
                                n += 1;
                            }
                            (h, n)
                        };
                        let ((fh, fnn), (bh, bnn)) = match &faithful_raw {
                            Some((f, b)) => (fnv_p(f), fnv_p(b)),
                            None => (fnv(&lower_front_ex), fnv(&lower_back_ex)),
                        };
                        eprintln!(
                            "PERILOW-R layer={} fh={:x} bh={:x} fn={} bn={}",
                            config.layer_id, fh, bh, fnn, bnn
                        );
                    }
                    // PerimeterGenerator.cpp:229-239 — dist_boundary(width)
                    // first = 0; second = scale_(end_offset) - scale_(off_front)
                    // C++ `scale_(val)` macro == `val / SCALING_FACTOR_CPP` where
                    // SCALING_FACTOR_CPP = 0.00001, i.e. `val * 100000` (a *double*
                    // upscale to scaled coords). The crate-root `SCALING_FACTOR`
                    // constant is the RECIPROCAL (100_000.0), so the scaled value is
                    // `val * SCALING_FACTOR`, NOT `val / SCALING_FACTOR`. The prior
                    // `v / SCALING_FACTOR` collapsed upper_bound to ~3.7e-6, so
                    // get_mapped_degree divided by ~0 and graded every middle-band
                    // perimeter as MAX_OVERHANG_DEGREE (10 mm/s) — flooding the outer
                    // wall with overhang speed and ~3x-ing the print-time estimate.
                    let scale_d = |v: f64| v * SCALING_FACTOR;
                    let lower_bound = 0.0_f64;
                    let upper_bound = scale_d(end_offset) - scale_d(off_front);

                    // lower_polygons_series.front()/back() as flat Polygons (holes included),
                    // matching C++ `std::vector<Polygons>` entries.
                    let (lower_front_polys, lower_back_polys): (Vec<Polygon>, Vec<Polygon>) =
                        match faithful_raw {
                            Some((f, b)) => (f, b),
                            None => (
                                crate::geometry::to_polygons(&lower_front_ex),
                                crate::geometry::to_polygons(&lower_back_ex),
                            ),
                        };

                    if lower_front_polys.is_empty() {
                        false
                    } else {
                        // Faithful classic overhang flow — PerimeterGenerator.cpp:374-432.
                        // intersection_pl_2/diff_pl_2 = the Clipper2Utils.cpp open-path clips
                        // (clipper2_utils.rs recovers solution_open, unlike clipper_utils' safe wrapper).
                        use crate::clipper2_utils::{diff_pl_2, intersection_pl_2};
                        use crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_polygons;

                        // PerimeterGenerator.cpp:376-377 — BoundingBox bbox(polygon.points); bbox.offset(SCALED_EPSILON);
                        // BoundingBoxBase::offset(delta): min -= delta, max += delta.
                        let mut bbox = crate::geometry::BoundingBox::from_points(polygon.points());
                        let se = crate::libslic3r::SCALED_EPSILON as i64;
                        bbox.min = Point::new(bbox.min.x() - se, bbox.min.y() - se);
                        bbox.max = Point::new(bbox.max.x() + se, bbox.max.y() + se);

                        // PerimeterGenerator.cpp:382 — clip lower_series.back() to bbox.
                        let lower_back_clipped = clip_clipper_polygons_with_subject_bbox_polygons(
                            &lower_back_polys,
                            &bbox,
                        );

                        // PerimeterGenerator.cpp:384 — inside = intersection_pl_2([to_polyline(polygon)], back_clipped)
                        let poly_pl = crate::geometry::polygon::to_polyline(polygon);
                        let inside_polines = intersection_pl_2(
                            std::slice::from_ref(&poly_pl),
                            &lower_back_clipped,
                        );
                        if std::env::var("SPLITDBG").is_ok() && config.layer_id == 1 {
                            let fnv_pl = |pls: &[crate::geometry::Polyline]| -> (u64, usize) {
                                let mut h: u64 = 1469598103934665603; let mut np = 0usize;
                                for pl in pls { for pt in pl.points() { h ^= pt.x as u64; h = h.wrapping_mul(1099511628211); h ^= pt.y as u64; h = h.wrapping_mul(1099511628211); np += 1; } }
                                (h, np)
                            };
                            let fnv_pg = |pgs: &[Polygon]| -> (u64, usize) {
                                let mut h: u64 = 1469598103934665603; let mut np = 0usize;
                                for pg in pgs { for pt in pg.points() { h ^= pt.x as u64; h = h.wrapping_mul(1099511628211); h ^= pt.y as u64; h = h.wrapping_mul(1099511628211); np += 1; } }
                                (h, np)
                            };
                            let mut hp: u64 = 1469598103934665603;
                            for pt in polygon.points() { hp ^= pt.x as u64; hp = hp.wrapping_mul(1099511628211); hp ^= pt.y as u64; hp = hp.wrapping_mul(1099511628211); }
                            let (hc, nc) = fnv_pg(&lower_back_clipped);
                            let (hi, ni) = fnv_pl(&inside_polines);
                            eprintln!(
                                "SPLITDBG-R layer=1 poly={:x} bbox={},{}/{},{} clip={:x}({}) inside={:x}({})",
                                hp, bbox.min.x(), bbox.min.y(), bbox.max.x(), bbox.max.y(), hc, nc, hi, ni
                            );
                        }
                        // PerimeterGenerator.cpp:387 — remain = diff_pl_2([to_polyline(polygon)], back_clipped)
                        let remain_polines =
                            diff_pl_2(std::slice::from_ref(&poly_pl), &lower_back_clipped);


                        // detect_overhang_speed is always true here (classic Benchy path;
                        // is_enable_overhang_speed && fuzzy_skin_allows_overhang_slowdown).
                        // PerimeterGenerator.cpp:403 — clip lower_series.front() to bbox.
                        let lower_front_clipped =
                            clip_clipper_polygons_with_subject_bbox_polygons(
                                &lower_front_polys,
                                &bbox,
                            );
                        // PerimeterGenerator.cpp:405 — middle = diff_pl_2(inside, front_clipped)
                        let middle_overhang_polines =
                            diff_pl_2(&inside_polines, &lower_front_clipped);
                        // PerimeterGenerator.cpp:407 — zero = intersection_pl_2(inside, front_clipped)
                        let zero_degree_polines =
                            intersection_pl_2(&inside_polines, &lower_front_clipped);
                        // PerimeterGenerator.cpp:408-401 — append zero-degree paths.
                        if !zero_degree_polines.is_empty() {
                            crate::overhang_detector::extrusion_paths_append(
                                &mut paths,
                                zero_degree_polines,
                                0.0,
                                0,
                                role,
                                extrusion_mm3_per_mm,
                                extrusion_width as f32,
                                layer_height as f32,
                            );
                        }
                        // PerimeterGenerator.cpp:394-404 — detect middle-line overhang.
                        if !middle_overhang_polines.is_empty() {
                            crate::overhang_detector::detect_overhang_degree(
                                lower_front_polys.clone(),
                                role,
                                extrusion_mm3_per_mm,
                                extrusion_width,
                                layer_height,
                                middle_overhang_polines,
                                lower_bound,
                                upper_bound,
                                &mut paths,
                            );
                        }

                        if std::env::var("SPLITDBG").is_ok() && config.layer_id == 1 {
                            let mut h: u64 = 1469598103934665603;
                            let mut np = 0usize;
                            for pa in paths.iter() {
                                for pt in pa.polyline.points() {
                                    h ^= pt.x as u64; h = h.wrapping_mul(1099511628211);
                                    h ^= pt.y as u64; h = h.wrapping_mul(1099511628211);
                                    np += 1;
                                }
                            }
                            eprintln!("SPLITDBG2-R postdeg paths={} pts={} h={:x}", paths.len(), np, h);
                        }

                        // PerimeterGenerator.cpp:411-432 — 100%-overhang -> detect_bridge_wall.
                        // No-support branch (Benchy has no support): role=erOverhangPerimeter,
                        // overhang_flow. (The zero-z-support branch keeps `role`/normal flow.)
                        if !remain_polines.is_empty() {
                            if let Some(ovh_flow) = config.overhang_flow.as_ref() {
                                detect_bridge_wall(
                                    &mut paths,
                                    &remain_polines,
                                    ExtrusionRole::OverhangPerimeter,
                                    ovh_flow.mm3_per_mm().unwrap_or(extrusion_mm3_per_mm),
                                    ovh_flow.width() as f32,
                                    ovh_flow.height() as f32,
                                );
                            } else {
                                // Fallback (overhang_flow not wired): keep original flow.
                                detect_bridge_wall(
                                    &mut paths,
                                    &remain_polines,
                                    role,
                                    extrusion_mm3_per_mm,
                                    extrusion_width as f32,
                                    layer_height as f32,
                                );
                            }
                        }

                        // PerimeterGenerator.cpp:434 — if (paths.empty()) continue;
                        if paths.is_empty() {
                            continue;
                        }

                        // PerimeterGenerator.cpp:439 — chain_and_reorder_extrusion_paths(paths, &paths.front().first_point());
                        let start = paths[0].polyline.first_point();
                        crate::shortest_path::chain_and_reorder_extrusion_paths(
                            &mut paths,
                            Some(&start),
                        );
                        true
                    }
                } else {
                    false
                }
            } else {
                false
            }
        } else {
            false
        };

        /// PerimeterGenerator.cpp:440-449 — non-overhang fallback (the `else` branch).
        /// C++: ExtrusionPath path(role); path.polyline = polygon.split_at_first_point();
        /// C++: path.overhang_degree = 0; path.curve_degree = 0;
        /// Faithful: the perimeter generator does NOT place the seam here — it
        /// splits the loop at its first point. Seam placement is a later (gcode)
        /// stage in BambuStudio.
        if !did_overhang_split {
            let mut path = ExtrusionPath::new(role);
            path.polyline = polygon.split_at_first_point();
            path.overhang_degree = 0.0;
            path.curve_degree = 0;
            path.mm3_per_mm = extrusion_mm3_per_mm;
            path.width = extrusion_width;
            path.height = layer_height;
            paths.push(path);
        }

        /// PerimeterGenerator.cpp:456-459
        /// C++: for (ExtrusionPath& path : paths) {
        /// C++:     path.set_customize_flag(flag);
        /// C++: }
        for path in &mut paths {
            path.set_customize_flag(flag);
        }

        /// PerimeterGenerator.cpp:461
        /// C++: coll.append(ExtrusionLoop(std::move(paths), loop_role, flag));
        let eloop = ExtrusionLoop::new_with_flag(paths, loop_role, flag);
        coll.append(ExtrusionEntityType::Loop(eloop));
    }

    /// Append thin walls to the collection
    /// PerimeterGenerator.cpp:464-470
    /// C++: Point zero_point(0, 0);
    let zero_point = Point::new(0, 0);

    /// C++: if (! thin_walls.empty()) {
    if !thin_walls.is_empty() {
        /// PerimeterGenerator.cpp:465-467
        /// C++: BoundingBox bbox;
        /// C++: for (auto &entity : coll.entities) { bbox.merge(entity->as_polyline().bounding_box()); }
        // TODO: Compute bbox from coll.entities and find zero_point
        // For now, use default zero_point

        /// PerimeterGenerator.cpp:468
        /// C++: variable_width(thin_walls, erExternalPerimeter, perimeter_generator.ext_perimeter_flow, coll.entities);
        let gap_fill_paths = convert_thin_walls_to_extrusion_paths(
            thin_walls,
            ExtrusionRole::ExternalPerimeter,
            ext_perimeter_flow,
        );

        /// VariableWidth.cpp:223-228
        /// C++: if (!paths.empty()) {
        /// C++:     if (paths.front().first_point() == paths.back().last_point())
        /// C++:         out.emplace_back(new ExtrusionLoop(std::move(paths)));
        /// C++:     else {
        /// C++:         for (ExtrusionPath& path : paths)
        /// C++:             out.emplace_back(new ExtrusionPath(std::move(path)));
        /// C++:     }
        /// C++: }
        for path in gap_fill_paths {
            if !path.polyline.points.is_empty() {
                let first = path.polyline.first_point();
                let last = path.polyline.last_point();

                if first == last {
                    let eloop = ExtrusionLoop::new(vec![path], ExtrusionLoopRole::DEFAULT);
                    coll.append(ExtrusionEntityType::Loop(eloop));
                } else {
                    coll.append(ExtrusionEntityType::Path(path));
                }
            }
        }

        /// PerimeterGenerator.cpp:469
        /// C++: thin_walls.clear();
        thin_walls.clear();
    }

    /// Traverse children and build the final collection.
    /// PerimeterGenerator.cpp:478:
    ///   std::vector<std::pair<size_t, bool>> chain = chain_extrusion_entities(coll.entities, &zero_point);
    ///
    /// FAITHFULNESS FIX: this previously used a bespoke `greedy_chain_indices`
    /// that only inspected each entity's first_point and never reversed. C++
    /// `chain_extrusion_entities` (ShortestPath.cpp:1003) runs
    /// `chain_segments_greedy_constrained_reversals`, which considers BOTH the
    /// first and last endpoint of every remaining segment and chains from the
    /// previously-placed endpoint, allowing reversals for reversible entities.
    /// For closed loops first==last so the reversal is a no-op (the chain code
    /// already clears `segment.second` for loops), but the *order* in which the
    /// loop groups are visited differs from the first-point-only heuristic —
    /// which is the perimeter contour-group ordering that drives inter-loop
    /// travel. Use the faithful port so the emission order matches native.
    let chain: Vec<(usize, bool)> =
        crate::shortest_path::chain_extrusion_entities(&coll.entities, Some(&zero_point));

    // R710 — same reason as PGSO: the call is faithful, the INPUT may not be.
    // Print the entity first points this chain keys on, the seed, and the
    // resulting order, against C++ `PerimeterGenerator.cpp:478` (PGCH).
    // Must run BEFORE the drain below, which empties `coll.entities`.
    if crate::probe_enabled("PGCH") {
        let fp: Vec<String> = coll
            .entities
            .iter()
            .map(|e| {
                let p = crate::shortest_path::entity_first_point(e);
                format!("{},{}", p.x, p.y)
            })
            .collect();
        let ch: Vec<String> = chain.iter().map(|(i, _)| i.to_string()).collect();
        eprintln!(
            "[PGCH] layer={} n={} zp={},{} fp={} chain={}",
            config.layer_id,
            coll.entities.len(),
            zero_point.x,
            zero_point.y,
            fp.join(";"),
            ch.join(",")
        );
    }

    // Move entities out of coll for indexed access.
    let mut entities: Vec<Option<ExtrusionEntityType>> =
        coll.entities.drain(..).map(Some).collect();

    let mut out = ExtrusionEntityCollection::new();

    /// PerimeterGenerator.cpp:480-505
    /// C++: for (const std::pair<size_t, bool> &idx : chain) {
    for (orig_idx, reverse) in chain {
        /// PerimeterGenerator.cpp:481
        /// C++: assert(coll.entities[idx.first] != nullptr);
        let mut entity = entities[orig_idx].take().unwrap();

        /// PerimeterGenerator.cpp:482-489
        /// C++: if (idx.first >= loops.size()) {
        /// C++:     // This is a thin wall.
        /// C++:     out.entities.emplace_back(coll.entities[idx.first]);
        /// C++:     if (idx.second) out.entities.back()->reverse();
        /// C++: } else {
        if orig_idx >= loops.len() {
            // PerimeterGenerator.cpp:487-488: thin wall — honor the chain's reversal flag.
            if reverse {
                entity_reverse_inplace(&mut entity);
            }
            out.append(entity);
        } else {
            /// PerimeterGenerator.cpp:484-501
            /// C++: const PerimeterGeneratorLoop &loop = loops[idx.first];
            /// C++: ExtrusionEntityCollection children = traverse_loops(perimeter_generator, loop.children, thin_walls);
            let loop_item = &loops[orig_idx];
            let mut child_thin_walls = ThickPolylines::new();
            let children = traverse_loops_legacy(
                &loop_item.children,
                &mut child_thin_walls,
                layer_height,
                perimeter_flow,
                ext_perimeter_flow,
                config,
            );

            /// C++: assert(thin_walls.empty());
            assert!(
                child_thin_walls.is_empty(),
                "Child thin_walls should be empty"
            );

            if let ExtrusionEntityType::Loop(mut eloop) = entity {
                /// PerimeterGenerator.cpp:491-500
                /// C++: if (loop.is_contour) {
                /// C++:     eloop->make_counter_clockwise();
                /// C++:     out.append(std::move(children.entities));
                /// C++:     out.entities.emplace_back(eloop);
                /// C++: } else {
                /// C++:     eloop->make_clockwise();
                /// C++:     out.entities.emplace_back(eloop);
                /// C++:     out.append(std::move(children.entities));
                /// C++: }
                if loop_item.is_contour {
                    eloop.make_counter_clockwise();
                    for child_entity in children.entities {
                        out.append(child_entity);
                    }
                    out.append(ExtrusionEntityType::Loop(eloop));
                } else {
                    eloop.make_clockwise();
                    out.append(ExtrusionEntityType::Loop(eloop));
                    for child_entity in children.entities {
                        out.append(child_entity);
                    }
                }
            }
        }
    }

    /// PerimeterGenerator.cpp:503
    /// C++: return out;
    out
}

impl PerimeterGenerator {
    /// Generate perimeters using the faithful Arachne port.
    ///
    /// Mirrors `PerimeterGenerator::process_arachne()`
    /// (PerimeterGenerator.cpp:1470-1803). This drives the faithful
    /// `Arachne::WallToolPaths` (wall_tool_paths.rs) instead of the previous
    /// divergent simplified generator. The wall-maker backend
    /// (`SkeletalTrapezoidation`) is not yet a working VD port, so
    /// `WallToolPaths` currently emits empty toolpaths and the inner contour
    /// falls back to the input outline; the control flow below remains a
    /// line-by-line translation of the C++ so it produces correct output once
    /// the wall-maker lands.
    /// PerimeterGenerator.cpp:1806-1828 — bool PerimeterGenerator::should_enable_top_one_wall(
    ///     const ExPolygons& original_expolys, ExPolygons& top)
    ///
    /// R684. NOT the same computation as the classic path's only_one_wall_top block
    /// (`generate_classic_one`, mirroring PerimeterGenerator.cpp:1116-1183): that one
    /// also carries an `offset_top_surface` term which this function has no
    /// counterpart for, so the two cannot share code (R671→R672 — check the callee,
    /// do not assume a sibling path computes the same thing).
    ///
    /// UNITS: C++ builds `min_width_top_surface` from `scaled_spacing()`/`scaled_width()`
    /// and feeds it to `offset_ex`, which takes scaled coords. Our `offset_expolygons`
    /// takes MILLIMETRES (see `opening_ex`'s callers), so the widths are taken from
    /// `Flow::spacing()`/`Flow::width()` in mm instead of their `scaled_*` twins.
    /// `ExPolygon::area()` returns SCALED squared units, so C++'s `scale_(1)*scale_(1)`
    /// (1 mm²) is `SCALING_FACTOR²` here — the `lib.rs` constant, 100_000 (R679: two
    /// reciprocal constants of that name are in scope).
    fn should_enable_top_one_wall_legacy(&self, original_expolys: &[ExPolygon], top: &mut ExPolygons) -> bool {
        // C++ :1808-1809
        let perimeter_width = self.config.perimeter_flow.width();
        let ext_perimeter_spacing = self.config.ext_perimeter_flow.spacing();

        // C++ :1811-1815
        let area_of = |v: &[ExPolygon]| -> f64 { v.iter().map(|e| e.area()).sum() };

        // C++ :1818 — BBS: filter small area and extend top surface a bit to hide the wall line
        let min_width_top_surface = (self.config.top_area_threshold / 100.0)
            * (ext_perimeter_spacing / 2.0).max(perimeter_width / 2.0);
        // PerimeterGenerator.cpp:1853-1864 — `perimeter_width` and
        // `ext_perimeter_spacing` are coord_t (`scaled_width()`,
        // `scaled_spacing()`), `min_width_top_surface` a double in scaled units,
        // and both `offset_ex` calls take it as a float and compute at coord_t.
        let exact = crate::faithful_gate("ARACHNE_TOP_ONE_WALL_CLIB");
        let perimeter_width_s = self.config.perimeter_flow.scaled_width() as f64;
        let min_width_top_surface_s = (self.config.top_area_threshold / 100.0)
            * (self.config.ext_perimeter_flow.scaled_spacing() as f64 / 2.0)
                .max(perimeter_width_s / 2.0);
        // C++ :1819
        let shrunk_top = if exact {
            crate::clipper_utils::offset_expolygons_clib_scaled(
                top,
                -(min_width_top_surface_s as f32) as f64,
                crate::clipper_utils::OffsetJoinType::Miter,
            )
        } else {
            crate::clipper_utils::offset_expolygons(
                top,
                -min_width_top_surface,
                crate::clipper_utils::OffsetJoinType::Miter,
            )
        };
        // C++ :1820-1821
        let shrunk_area = area_of(&shrunk_top);
        let original_area = area_of(original_expolys);

        // C++ :1823 — `scale_(1) * scale_(1)` is one square millimetre in scaled units.
        let one_mm2 = crate::SCALING_FACTOR * crate::SCALING_FACTOR;
        if shrunk_area / (original_area + crate::libslic3r::EPSILON) < 0.1
            || original_area < one_mm2
        {
            // C++ :1824
            top.clear();
        } else {
            // C++ :1826
            *top = if exact {
                crate::clipper_utils::offset_expolygons_clib_scaled(
                    &shrunk_top,
                    ((min_width_top_surface_s + perimeter_width_s) as f32) as f64,
                    crate::clipper_utils::OffsetJoinType::Miter,
                )
            } else {
                crate::clipper_utils::offset_expolygons(
                    &shrunk_top,
                    min_width_top_surface + perimeter_width,
                    crate::clipper_utils::OffsetJoinType::Miter,
                )
            };
        }
        // C++ :1827
        !top.is_empty()
    }

    /// The body `process_arachne` had up to R921 (the arm of `PG_ARACHNE_PIN=0`): millimetres, the C++
    /// ClipperLib shims and some twenty gates of its own. The port of the pin's function is
    /// [`PerimeterGenerator::process_arachne`].
    pub fn process_arachne_legacy(&self, slices: &[ExPolygon]) -> PerimeterResult {
        self.set_mm3_per_mm_members(false);
        let mut result = PerimeterResult::new();
        // R803e — see the LoopNode block in the per-line loop below.
        let arachne_nodes_on = self.config.z_direction_outwall_speed_continuous
            && crate::faithful_gate("ARACHNE_LOOP_NODES");
        let arachne_node_first = result.loop_nodes.len();

        if slices.is_empty() {
            result.infill_area = slices.to_vec();
            return result;
        }

        // PerimeterGenerator.cpp:1474-1481
        let perimeter_spacing: Coord = self.config.perimeter_flow.scaled_spacing();
        let ext_perimeter_width: Coord = self.config.ext_perimeter_flow.scaled_width();
        let ext_perimeter_spacing: Coord = self.config.ext_perimeter_flow.scaled_spacing();
        let ext_perimeter_spacing2: Coord = crate::scaled(
            0.5 * (self.config.ext_perimeter_flow.spacing() + self.config.perimeter_flow.spacing()),
        );

        // PerimeterGenerator.cpp:1505  for (const Surface& surface : this->slices->surfaces)
        let mut entities = ExtrusionEntityCollection::new();
        let mut infill_contour: ExPolygons = Vec::new();
        // PerimeterGenerator.cpp:1460 — the parallel `fill_no_overlap` accumulation.
        let mut no_overlap_contour: ExPolygons = Vec::new();

        // PerimeterGenerator.cpp:1499-1500
        // C++: double surface_simplify_resolution = (enable_arc_fitting && fuzzy_skin == None)
        //          ? 0.2 * m_scaled_resolution : m_scaled_resolution;
        //
        // R672 — UNITS. `m_scaled_resolution` is `scaled<double>(print_config.resolution)`,
        // so for 0.0125 mm the C++ tolerance is 0.2 * 1250 = 250 SCALED units. Our
        // `config.surface_simplify_resolution` holds the UNSCALED millimetre value
        // (layer.rs sets it to `print_config.resolution`), so passing it straight to
        // `simplify_p` is a 1e5x-too-small tolerance. R122 found exactly this in the
        // CLASSIC path and fixed it there (:525-531, gated `F1_UNION`); the arachne path
        // kept the unscaled value. Gated separately so the A/B is attributable.
        // R672 — DEFAULT OFF (`probe_enabled`, not `faithful_gate`). The A/B showed the
        // scaled value COLLAPSES every contour: POLYPROBE's "0 input outline" goes from
        // 26,782 polys / 1,223,769 points to 0/0. That is the signature of `simplify_p`
        // taking an UNSCALED millimetre tolerance, so 250 is a 250 mm tolerance and the
        // pre-existing unscaled value was right FOR THIS FUNCTION. Kept as an opt-in
        // switch because the collapse is itself a measurement (see the round notes).
        // R739 STATUS: **BROKEN, not merely unhelpful — do not re-score it.**
        // Turning this on collapses the arachne fixture from 37,349 in-order
        // matched lines to 9,312 and the slice from 1.79 s to 0.387 s: the output
        // is largely GONE, not differently ordered. The R672/R673 premise (that
        // the resolution belongs in scaled units) needs re-deriving from
        // PerimeterGenerator.cpp before this is touched again.
        let surface_simplify_resolution = if crate::opt_in_gate("ARACHNE_SIMPLIFY_SCALED") {
            let scaled = self.config.surface_simplify_resolution / 0.00001_f64;
            if self.config.arc_fitting_enabled
                && self.config.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None
            {
                0.2 * scaled
            } else {
                scaled
            }
        } else if self.config.arc_fitting_enabled
            && self.config.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None
        {
            0.2 * self.config.surface_simplify_resolution
        } else {
            self.config.surface_simplify_resolution
        };

        // R677 — SURFPROBE's absolute totals are revisit-inflated (R676): 26,000
        // surface visits over 656 layers is ~40 per layer against ~2 contours per
        // layer at bracket A. This counts the CALLS and the DISTINCT layer_ids so
        // the inflation factor is a measured number rather than an assumption.
        // calls / distinct_layers, divided by the (layer, region) pairs-per-layer
        // that SLICEPTS bracket C prints, IS the revisit factor.
        if crate::probe_enabled("SURFPROBE") {
            use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
            use std::sync::Mutex;
            static ARACHNE_CALLS: AtomicU64 = AtomicU64::new(0);
            static LAYERS_SEEN: Mutex<Option<std::collections::HashSet<usize>>> = Mutex::new(None);
            let n = ARACHNE_CALLS.fetch_add(1, Relaxed) + 1;
            let mut g = LAYERS_SEEN.lock().unwrap();
            let set = g.get_or_insert_with(std::collections::HashSet::new);
            set.insert(self.config.layer_id);
            if n % 2_000 == 0 {
                eprintln!(
                    "[SURFPROBE] generate_arachne calls={n} distinct_layer_ids={} \
                     surfaces_this_call={} calls_per_layer={:.3}",
                    set.len(),
                    slices.len(),
                    n as f64 / set.len().max(1) as f64,
                );
            }
        }
        if crate::probe_enabled("PAPROBE") {
            eprintln!(
                "PACALL lid={} nsurf={} mode=arachne",
                self.config.layer_id,
                slices.len()
            );
        }
        for surface in slices.iter() {
            if crate::probe_enabled("PAPROBE") {
                let mut iv: i64 = surface.contour.points.len() as i64;
                let mut ix: i64 = 0;
                let mut iy: i64 = 0;
                for pt in &surface.contour.points {
                    ix += pt.x();
                    iy += pt.y();
                }
                for h in &surface.holes {
                    iv += h.points.len() as i64;
                    for pt in &h.points {
                        ix += pt.x();
                        iy += pt.y();
                    }
                }
                eprintln!(
                    "PAPROBE lid={} holes={} iv={} ix={} iy={}",
                    self.config.layer_id,
                    surface.holes.len(),
                    iv,
                    ix,
                    iy
                );
            }
            // R674 — one hop BACK from R670. R670's POLYPROBE "0 input outline" is
            // `WallToolPaths::outline`, i.e. ALREADY past this function's `simplify_p`
            // and `offset_ex`. This counts `surface` as `generate_arachne` receives it,
            // which is the true input, and matches C++'s `surface.expolygon` at
            // PerimeterGenerator.cpp:1511 before its `simplify_p`. Points-per-contour
            // (R670's rule: per-call mixes populations, per-contour does not).
            if crate::probe_enabled("SURFPROBE") {
                use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                static CALLS: AtomicU64 = AtomicU64::new(0);
                static CONTOURS: AtomicU64 = AtomicU64::new(0);
                static POINTS: AtomicU64 = AtomicU64::new(0);
                let mut c = 1u64;
                let mut pts = surface.contour.points.len() as u64;
                for h in &surface.holes {
                    c += 1;
                    pts += h.points.len() as u64;
                }
                CONTOURS.fetch_add(c, Relaxed);
                POINTS.fetch_add(pts, Relaxed);
                let n = CALLS.fetch_add(1, Relaxed) + 1;
                if n % 2_000 == 0 {
                    let (co, po) = (CONTOURS.load(Relaxed), POINTS.load(Relaxed));
                    eprintln!(
                        "[SURFPROBE] surfaces={n} contours={co} points={po} points_per_contour={:.3}",
                        po as f64 / co.max(1) as f64,
                    );
                }
            }
            // PerimeterGenerator.cpp:1507  loop_number = wall_loops + extra_perimeters - 1
            let loop_number: i32 = self.config.perimeter_count as i32 - 1;

            // PerimeterGenerator.cpp:1511-1512  offset for the outer wall.
            // C++: ExPolygons last = offset_ex(surface.expolygon.simplify_p(surface_simplify_resolution),
            //          apply_precise_outer_wall ? -(ext_perimeter_width - ext_perimeter_spacing)
            //                                   : -(ext_perimeter_width/2. - ext_perimeter_spacing/2.));
            // (precise_outer_wall not modelled here -> use the simple inset.)
            // R554: the two C++ perimeter paths differ here and the difference is
            // deliberate. `process_classic` (PerimeterGenerator.cpp:945) reads
            // `union_ex(surface.expolygon.simplify_p(res))`, which our
            // `generate_classic_one` mirrors; `process_arachne` (:1511) reads
            // `offset_ex(surface.expolygon.simplify_p(res), -...)` with NO union
            // -- `offset_ex` reconstructs the ExPolygons itself, so a prior union
            // only risks merging contours. We had inherited the classic form here.
            // R696 — `simplify_p` carries TWO documented F1 approximations, and the
            // retention it produces is 0.9225 against C++'s 0.8230 (R695). The
            // tolerance is NOT the cause: `surface_simplify_resolution` is
            // `print_config.resolution` (0.012 mm) and both engines take the 0.2x
            // arc-fitting branch, so both intend 0.0024 mm, and
            // `geometry::simplify::douglas_peucker` re-scales mm internally
            // (geometry/mod.rs:107 re-exports THAT one, not the already-scaled
            // `multi_point::douglas_peucker`). What differs is the algorithm:
            //   1. the DP distance is a ROUNDED projection (`Line::distance_to_squared`)
            //      where C++ uses pure double — `douglas_peucker_faithful` (R122);
            //   2. the post-DP step is `simplify_polygons_clipper`, a geo-clipper
            //      union standing in for ClipperLib `SimplifyPolygons` with
            //      StrictlySimple(true) — "the KEY difference from a plain ctUnion:
            //      different vertex retention" (clipper_utils.rs, R91).
            // Both faithful halves already exist; this routes through them so the
            // retention can be measured against C++'s 0.8230.
            let simplify_faithful = |ex: &crate::geometry::ExPolygon| -> Vec<crate::geometry::Polygon> {
                let tol_scaled = surface_simplify_resolution * crate::SCALING_FACTOR;
                let rings = ex.simplify_p_dp_rings_faithful(tol_scaled);
                crate::clipper_utils::simplify_polygons_clib(&rings, 1)
            };
            let simplified: crate::geometry::ExPolygons =
                if crate::faithful_gate("ARACHNE_NO_PRE_UNION") {
                    // PerimeterGenerator.cpp:1511 offset_ex(surface.expolygon.simplify_p(res), ...)
                    vec![]
                } else {
                    union_polygons_ex(&surface.simplify_p(surface_simplify_resolution))
                };
            // PerimeterGenerator.cpp:1506-1508 — `-float(ext_perimeter_width / 2 -
            // ext_perimeter_spacing / 2)`: both halves are coord_t divisions, so
            // an odd width or spacing loses its half unit. ARACHNE_INSET_INT=0
            // halves in double.
            let inset = if crate::faithful_gate("ARACHNE_INSET_INT") {
                -(((ext_perimeter_width / 2 - ext_perimeter_spacing / 2) as f32) as f64)
            } else {
                -(ext_perimeter_width as f64 / 2.0 - ext_perimeter_spacing as f64 / 2.0)
            };
            if crate::probe_enabled("AWDELTA") {
                use std::sync::Mutex;
                static SEEN: Mutex<Vec<(i64, i64, i64)>> = Mutex::new(Vec::new());
                let key = (ext_perimeter_width, ext_perimeter_spacing, inset as i64);
                if let Ok(mut v) = SEEN.lock() {
                    if !v.contains(&key) {
                        v.push(key);
                        eprintln!("AWDELTA ext_w={} ext_sp={} inset={:.3}", ext_perimeter_width, ext_perimeter_spacing, inset);
                    }
                }
            }
            let last = if crate::faithful_gate("ARACHNE_NO_PRE_UNION") {
                // PerimeterGenerator.cpp:1511 — offset the raw simplify_p polygons.
                let simplified_polys = if crate::faithful_gate("ARACHNE_SIMPLIFY_FAITHFUL") {
                    simplify_faithful(surface)
                } else {
                    surface.simplify_p(surface_simplify_resolution)
                };
                if crate::faithful_gate("ARACHNE_OFFSET_CLIB") {
                    // R775 — the vertex-exact `offset_ex(Polygons, float delta)`
                    // (per-path raw_offset + polytree shrink reconstruction). The
                    // geo route re-gridded every boundary at 1µm — AWIN measured
                    // ~0.7µm/pt drift on EVERY WallToolPaths input, which then
                    // shifted every arachne junction. delta f32-rounded like the
                    // C++ float parameter.
                    crate::clipper_utils::offset_ex_polygons_clib(
                        &simplified_polys,
                        (inset as f32) as f64,
                        self.config.join_type,
                    )
                } else {
                    crate::clipper_utils::offset_polygons(
                        &simplified_polys,
                        inset / crate::SCALING_FACTOR,
                        self.config.join_type,
                    )
                }
            } else {
                offset_expolygons(
                    &simplified,
                    inset / crate::SCALING_FACTOR,
                    self.config.join_type,
                )
            };

            if crate::probe_enabled("LASTPROBE") {
                let sp: usize = surface.contour.points.len()
                    + surface.holes.iter().map(|h| h.points.len()).sum::<usize>();
                lastprobe("A surface.expolygon", 1, surface.holes.len(), sp);
                let lc = last.len();
                let lh: usize = last.iter().map(|e| e.holes.len()).sum();
                let lp: usize = last
                    .iter()
                    .map(|e| {
                        e.contour.points.len()
                            + e.holes.iter().map(|h| h.points.len()).sum::<usize>()
                    })
                    .sum();
                lastprobe("C last (ExPolygons)", lc, lh, lp);
            }
            // PerimeterGenerator.cpp:1518-1522  Polygons last_p = to_polygons(last);
            let last_p: crate::geometry::Polygons = expolygons_to_polygons(&last);
            // R803 AWDUMP needs the WTP input after `last_p` is moved into the generator.
            let last_p_dump: Option<crate::geometry::Polygons> =
                if std::env::var("AWDUMP_LID").is_ok() { Some(last_p.clone()) } else { None };
            if crate::probe_enabled("LASTPROBE") {
                let pp: usize = last_p.iter().map(|p| p.points.len()).sum();
                lastprobe("D last_p (Polygons)", last_p.len(), 0, pp);
            }

            let mut total_perimeters: Vec<VariableWidthLines> = Vec::new();
            let surface_infill: ExPolygons;

            if loop_number >= 0 {
                // PerimeterGenerator.cpp:1528-1534 — the top-one-wall decision.
                //
                // R683/R684: the ARACHNE path carried only the `loop_number == 0`
                // disjunct; the other two, and the whole `seperate_wall_generation`
                // branch (C++ :1565-1613), were never wired in here. The feature IS
                // implemented for the CLASSIC path (`generate_classic_one`, the
                // only_one_wall_top block), which is why every config key and
                // `upper_slices` were already present — Majora is
                // `wall_generator = arachne`, so all of it was dead for this fixture.
                //
                // C++ :1528 generate_one_wall_by_first_layer = only_one_wall_first_layer && layer_id == 0
                // `only_one_wall_first_layer` is not threaded into PerimeterConfig
                // (see the note at the classic site) and is 0 in every fixture config,
                // so it is modelled as false rather than guessed at.
                let generate_one_wall_by_first_layer = false;
                // C++ :1529 generate_one_wall_by_top_most = top_one_wall_type != None && upper_slices == nullptr
                // C++ :1530 generate_one_wall_by_top      = top_one_wall_type == Alltop && upper_slices != nullptr
                // `self.config.top_one_wall` is the boolean collapse of
                // `top_one_wall_type == Alltop`. The C++ enum has three values
                // (None/Alltop/Topmost, PrintConfig.hpp:234-239), so `!= None` and
                // `== Alltop` coincide only while the value is Alltop or None — which
                // it is for every fixture here. A Topmost config would need the full
                // enum threaded through.
                let generate_one_wall_by_top_most =
                    self.config.top_one_wall && self.config.upper_slices.is_none();
                let generate_one_wall_by_top =
                    self.config.top_one_wall && self.config.upper_slices.is_some();
                // PerimeterGenerator.cpp:1532  is_one_wall
                // R774 — flipped opt-in -> faithful default ON: with the gate off,
                // rust made 2,000 WallToolPaths calls vs native 2,719 (AWIN census)
                // because the per-surface SPECULATIVE one-wall pass (cpp:1896-1921,
                // 733 ic=1 calls on benchy-arachne) never ran for Alltop configs.
                // ON matches the call census exactly (2,722v2,719) and scores up on
                // BOTH fixtures (benchy-arachne +770 in-order, Majora +80).
                let is_one_wall = if crate::faithful_gate("ARACHNE_TOP_ONE_WALL") {
                    loop_number == 0
                        || generate_one_wall_by_first_layer
                        || generate_one_wall_by_top_most
                } else {
                    loop_number == 0
                };
                // PerimeterGenerator.cpp:1534
                let mut seperate_wall_generation = crate::faithful_gate("ARACHNE_TOP_ONE_WALL")
                    && !is_one_wall
                    && generate_one_wall_by_top;

                // PerimeterGenerator.cpp:1537-1553  WallToolPathsParams input_params.
                let mut input_params = WallToolPathsParams::default();
                if crate::faithful_gate("ARACHNE_WTP_PARAMS") {
                    // PerimeterGenerator.cpp:1536 double min_nozzle_diameter = *std::min_element(...);
                    let min_nozzle_diameter = self.config.min_nozzle_diameter;
                    // PerimeterGenerator.cpp:1540 input_params.min_feature_size = v * 0.01 * min_nozzle_diameter;
                    input_params.min_feature_size =
                        (self.config.arachne_min_feature_size * 0.01 * min_nozzle_diameter) as f32;
                    // PerimeterGenerator.cpp:1543 input_params.min_bead_width = v * 0.01 * min_nozzle_diameter;
                    input_params.min_bead_width =
                        (self.config.arachne_min_bead_width * 0.01 * min_nozzle_diameter) as f32;
                    // PerimeterGenerator.cpp:1546 input_params.wall_transition_filter_deviation = v * 0.01 * min_nozzle_diameter;
                    input_params.wall_transition_filter_deviation =
                        (self.config.wall_transition_filter_deviation * 0.01 * min_nozzle_diameter)
                            as f32;
                    // PerimeterGenerator.cpp:1549 input_params.wall_transition_length = v * 0.01 * min_nozzle_diameter;
                    input_params.wall_transition_length =
                        (self.config.arachne_wall_transition_length * 0.01 * min_nozzle_diameter)
                            as f32;
                    // PerimeterGenerator.cpp:1551 input_params.wall_transition_angle = object_config->wall_transition_angle.value;
                    input_params.wall_transition_angle = self.config.wall_transition_angle as f32;
                    // PerimeterGenerator.cpp:1552 input_params.wall_distribution_count = object_config->wall_distribution_count.value;
                    input_params.wall_distribution_count =
                        self.config.wall_distribution_count as i32;
                }

                // PerimeterGenerator.cpp:1560  coord_t wall_0_inset = 0;
                let wall_0_inset: Coord = 0;
                let layer_height = self.config.layer_height;

                // WTPCALL (R580) — every WallToolPaths construction is one
                // `generateToolpaths` invocation. C++ makes 1.592x more of them
                // (41,188 vs 25,876, R577), the larger of the two upstream terms
                // feeding the whole 3.23x tag chain. Bucket by layer to see
                // whether the gap is uniform or concentrated in a band.
                if crate::probe_enabled("WTPCALL") {
                    use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                    use std::sync::Mutex;
                    static CALLS: AtomicU64 = AtomicU64::new(0);
                    static ONEWALL: AtomicU64 = AtomicU64::new(0);
                    static POLYS: AtomicU64 = AtomicU64::new(0);
                    static PERLAYER: Mutex<Vec<u32>> = Mutex::new(Vec::new());
                    if is_one_wall {
                        ONEWALL.fetch_add(1, Relaxed);
                    }
                    POLYS.fetch_add(last_p.len() as u64, Relaxed);
                    let n = CALLS.fetch_add(1, Relaxed) + 1;
                    if let Ok(mut v) = PERLAYER.lock() {
                        let li = self.config.layer_id;
                        if v.len() <= li {
                            v.resize(li + 1, 0);
                        }
                        v[li] += 1;
                        if n % 2_000 == 0 {
                            let nz: Vec<u32> = v.iter().copied().filter(|&x| x > 0).collect();
                            let layers = nz.len();
                            let mx = nz.iter().copied().max().unwrap_or(0);
                            let mut srt = nz.clone();
                            srt.sort_unstable();
                            let med = if srt.is_empty() { 0 } else { srt[srt.len() / 2] };
                            eprintln!(
                                "[WTPCALL] calls={} onewall={} polys={} layers_touched={} per_layer_median={} max={}",
                                n, ONEWALL.load(Relaxed), POLYS.load(Relaxed), layers, med, mx,
                            );
                        }
                    }
                }

                // PerimeterGenerator.cpp:1556-1558 — only valid when the wall
                // generation is separated.
                let mut first_perimeters: Vec<VariableWidthLines> = Vec::new();
                let mut infill_contour_by_one_wall: ExPolygons = Vec::new();
                let mut top_expolys_by_one_wall: ExPolygons = Vec::new();

                // PerimeterGenerator.cpp:1565-1588 — the PROBE pass. Build one wall,
                // take its inner contour, and diff it against the upper and lower
                // slices to isolate the areas that are top-but-not-bottom. C++ clips
                // the upper/lower polygons to the infill bbox first
                // (`clip_clipper_polygons_with_subject_bbox`, :1578/:1583); that is a
                // pure speed optimisation on the clip operand and does not change the
                // difference, so it is omitted rather than approximated.
                if seperate_wall_generation {
                    let top_one_wall_clib = crate::faithful_gate("ARACHNE_TOP_ONE_WALL_CLIB");
                    // C++ :1566
                    let mut one_wall_paths = WallToolPaths::new(
                        last_p.clone(),
                        ext_perimeter_spacing,
                        perimeter_spacing,
                        1,
                        wall_0_inset,
                        layer_height,
                        input_params,
                    );
                    // C++ :1570-1571
                    first_perimeters = one_wall_paths.get_tool_paths().clone();
                    infill_contour_by_one_wall =
                        union_polygons_ex(one_wall_paths.get_inner_contour());

                    if top_one_wall_clib {
                        // PerimeterGenerator.cpp:1618-1634 — the upper and lower
                        // slices are clipped to the bounding box of the one-wall
                        // infill contour first (`infill_bbox.offset(EPSILON)` adds
                        // nothing to a coord_t box), and the three `diff_ex` run at
                        // coord_t.
                        let infill_bbox =
                            crate::geometry::get_extents(&infill_contour_by_one_wall);
                        let clipped = |slices: Option<&ExPolygons>| -> crate::geometry::Polygons {
                            slices
                                .map(|s| {
                                    crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(
                                        s,
                                        &infill_bbox,
                                        false,
                                    )
                                })
                                .unwrap_or_default()
                        };
                        let upper_polygons_clipped = clipped(self.config.upper_slices.as_ref());
                        top_expolys_by_one_wall = crate::clipper_utils::diff_ex_polygons_polygons(
                            &expolygons_to_polygons(&infill_contour_by_one_wall),
                            &upper_polygons_clipped,
                            crate::clipper_utils::ApplySafetyOffset::No,
                        );
                        let lower_polygons_clipped = clipped(self.config.lower_slices.as_ref());
                        let bottom_expolys = crate::clipper_utils::diff_ex_polygons_polygons(
                            &expolygons_to_polygons(&top_expolys_by_one_wall),
                            &lower_polygons_clipped,
                            crate::clipper_utils::ApplySafetyOffset::No,
                        );
                        top_expolys_by_one_wall = crate::clipper_utils::diff_ex_polygons_polygons(
                            &expolygons_to_polygons(&top_expolys_by_one_wall),
                            &expolygons_to_polygons(&bottom_expolys),
                            crate::clipper_utils::ApplySafetyOffset::No,
                        );
                    } else {
                    // C++ :1576-1579  top = infill_contour_by_one_wall - upper_slices
                    top_expolys_by_one_wall = match self.config.upper_slices.as_ref() {
                        Some(upper) => {
                            crate::clipper_utils::difference(&infill_contour_by_one_wall, upper)
                        }
                        None => infill_contour_by_one_wall.clone(),
                    };
                    // C++ :1581-1584  bottom = top - lower_slices
                    let bottom_expolys = match self.config.lower_slices.as_ref() {
                        Some(lower) => {
                            crate::clipper_utils::difference(&top_expolys_by_one_wall, lower)
                        }
                        None => top_expolys_by_one_wall.clone(),
                    };
                    // C++ :1586  top = top - bottom
                    top_expolys_by_one_wall = crate::clipper_utils::difference(
                        &top_expolys_by_one_wall,
                        &bottom_expolys,
                    );
                    }
                    // C++ :1587
                    let before = seperate_wall_generation;
                    seperate_wall_generation =
                        self.should_enable_top_one_wall_legacy(&last, &mut top_expolys_by_one_wall);
                    // R684 census: how often the probe pass SURVIVES. If the survival
                    // rate is ~0 the extra construction is a discarded probe and the
                    // call-count term is downstream-inert.
                    if crate::probe_enabled("TOPONEWALL") {
                        use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                        static PROBED: AtomicU64 = AtomicU64::new(0);
                        static SURVIVED: AtomicU64 = AtomicU64::new(0);
                        static TOPEMPTY: AtomicU64 = AtomicU64::new(0);
                        let _ = before;
                        let n = PROBED.fetch_add(1, Relaxed) + 1;
                        if seperate_wall_generation {
                            SURVIVED.fetch_add(1, Relaxed);
                        }
                        if top_expolys_by_one_wall.is_empty() {
                            TOPEMPTY.fetch_add(1, Relaxed);
                        }
                        if n % 2_000 == 0 {
                            let s = SURVIVED.load(Relaxed);
                            eprintln!(
                                "[TOPONEWALL] probed={n} survived={s} ({:.4}) top_empty_after={}",
                                s as f64 / n as f64,
                                TOPEMPTY.load(Relaxed)
                            );
                        }
                    }
                }

                if seperate_wall_generation {
                    // PerimeterGenerator.cpp:1591-1613 — only generate one wall around
                    // top areas; keep the first generated wall and deal with the rest.
                    // C++ :1594-1595
                    total_perimeters = first_perimeters;
                    surface_infill = if loop_number > 0 {
                        // C++ :1598-1599
                        let top_one_wall_clib = crate::faithful_gate("ARACHNE_TOP_ONE_WALL_CLIB");
                        // PerimeterGenerator.cpp:1643
                        let last2 = if top_one_wall_clib {
                            crate::clipper_utils::diff_ex_polygons_polygons(
                                &expolygons_to_polygons(&infill_contour_by_one_wall),
                                &expolygons_to_polygons(&top_expolys_by_one_wall),
                                crate::clipper_utils::ApplySafetyOffset::No,
                            )
                        } else {
                            crate::clipper_utils::difference(
                                &infill_contour_by_one_wall,
                                &top_expolys_by_one_wall,
                            )
                        };
                        let last_p2: crate::geometry::Polygons = expolygons_to_polygons(&last2);
                        // C++ :1600  NOTE bead_width_0 is perimeter_spacing here, not
                        // ext_perimeter_spacing — the outer wall is already placed.
                        let mut paths_new = WallToolPaths::new(
                            last_p2,
                            perimeter_spacing,
                            perimeter_spacing,
                            loop_number as usize,
                            wall_0_inset,
                            layer_height,
                            input_params,
                        );
                        // C++ :1601-1609  append with inset_idx shifted by one.
                        let new_perimeters = paths_new.get_tool_paths().clone();
                        for mut perimeters in new_perimeters {
                            if !perimeters.is_empty() {
                                for p in perimeters.iter_mut() {
                                    p.inset_idx += 1;
                                }
                                total_perimeters.push(perimeters);
                            }
                        }
                        // C++ :1610-1611
                        let mut acc = union_polygons_ex(paths_new.get_inner_contour());
                        acc.extend(top_expolys_by_one_wall.iter().cloned());
                        if top_one_wall_clib {
                            // PerimeterGenerator.cpp:1655-1656: `union_ex(a, b)` is
                            // one union of both lists (ClipperUtils.cpp:825-831),
                            // `intersection_ex` at coord_t.
                            let acc = crate::clipper_utils::union_ex_clib(
                                &expolygons_to_polygons(&acc),
                                1,
                            );
                            crate::clipper_utils::intersection_ex_polygons_polygons(
                                &expolygons_to_polygons(&acc),
                                &expolygons_to_polygons(&infill_contour_by_one_wall),
                                crate::clipper_utils::ApplySafetyOffset::No,
                            )
                        } else {
                            let acc = crate::clipper_utils::union_ex(&acc);
                            crate::clipper_utils::intersection(&acc, &infill_contour_by_one_wall)
                        }
                    } else if crate::faithful_gate("ARACHNE_TOP_ONE_WALL_CLIB") {
                        // PerimeterGenerator.cpp:1640
                        crate::clipper_utils::union_ex_clib(
                            &expolygons_to_polygons(&infill_contour_by_one_wall),
                            1,
                        )
                    } else {
                        crate::clipper_utils::union_ex(&infill_contour_by_one_wall)
                    };
                } else if is_one_wall {
                    // PerimeterGenerator.cpp:1617-1621  plan wall width as one wall
                    let mut one_wall_paths = WallToolPaths::new(
                        last_p,
                        ext_perimeter_spacing,
                        perimeter_spacing,
                        1,
                        wall_0_inset,
                        layer_height,
                        input_params,
                    );
                    total_perimeters = one_wall_paths.get_tool_paths().clone();
                    surface_infill = union_polygons_ex(one_wall_paths.get_inner_contour());
                } else {
                    // PerimeterGenerator.cpp:1625-1629  plan wall width as normal
                    let mut normal_paths = WallToolPaths::new(
                        last_p,
                        ext_perimeter_spacing,
                        perimeter_spacing,
                        (loop_number + 1) as usize,
                        wall_0_inset,
                        layer_height,
                        input_params,
                    );
                    total_perimeters = normal_paths.get_tool_paths().clone();
                    surface_infill = union_polygons_ex(normal_paths.get_inner_contour());
                }
                // R772 — per-call wall-output checksum (mirrors the cpp AWALL
                // probe): with byte-exact perimeter INPUTS (726/726), the first
                // AWALL divergence pins the surface where the arachne generator
                // itself forks.
                if crate::probe_enabled("AWALL") {
                    let (mut nl, mut nj, mut jx, mut jy, mut jw) = (0i64, 0i64, 0i64, 0i64, 0i64);
                    for lines in &total_perimeters {
                        for line in lines {
                            nl += 1;
                            for j in &line.junctions {
                                nj += 1;
                                jx = jx.wrapping_add(j.p.x);
                                jy = jy.wrapping_add(j.p.y);
                                jw = jw.wrapping_add(j.w);
                            }
                        }
                    }
                    let (mut sv, mut sx2, mut sy2) = (0i64, 0i64, 0i64);
                    sv += surface.contour.points.len() as i64;
                    for pt in &surface.contour.points {
                        sx2 = sx2.wrapping_add(pt.x());
                        sy2 = sy2.wrapping_add(pt.y());
                    }
                    for h in &surface.holes {
                        sv += h.points.len() as i64;
                        for pt in &h.points {
                            sx2 = sx2.wrapping_add(pt.x());
                            sy2 = sy2.wrapping_add(pt.y());
                        }
                    }
                    eprintln!(
                        "AWALL lid={} surf={}/{}/{} nl={} nj={} jx={} jy={} jw={}",
                        self.config.layer_id, sv, sx2, sy2, nl, nj, jx, jy, jw
                    );
                }
                // R803 — AWDUMP_LID=<lid>: full junction dump of every wall line the
                // arachne generator returned for that layer, plus the WTP input
                // polygons (mirrors the cpp probe of the same name). Built into one
                // String and printed once so parallel layers cannot interleave.
                if let Ok(want) = std::env::var("AWDUMP_LID") {
                    if want == self.config.layer_id.to_string() {
                        use std::fmt::Write as _;
                        let mut o = String::new();
                        let last_p = last_p_dump.as_deref().unwrap_or(&[]);
                        let _ = writeln!(o, "AWDUMP lid={} npoly={} nlines={}", self.config.layer_id, last_p.len(), total_perimeters.len());
                        for (i, poly) in last_p.iter().enumerate() {
                            let _ = write!(o, "AWDIN {} n={}", i, poly.points.len());
                            for pt in &poly.points { let _ = write!(o, " {},{}", pt.x(), pt.y()); }
                            o.push('\n');
                        }
                        for (li, lines) in total_perimeters.iter().enumerate() {
                            for (lj, line) in lines.iter().enumerate() {
                                let _ = write!(o, "AWDL {}/{} inset={} odd={} closed={} n={}", li, lj, line.inset_idx, line.is_odd, line.is_closed, line.junctions.len());
                                for j in &line.junctions { let _ = write!(o, " {},{},{}", j.p.x, j.p.y, j.w); }
                                o.push('\n');
                            }
                        }
                        eprint!("{}", o);
                    }
                }
                // R659 — distinct junction widths produced for THIS layer, so the
                // 40 FLAT layers R658 isolated in the gcode can be correlated with
                // what the beading actually returned. `layer_id` is in scope here
                // (the WTPCALL probe above uses it); `WallToolPaths::generate` is
                // not, which is why the census sits at the call site.
                if crate::probe_enabled("WTPLAYER") {
                    use std::collections::BTreeSet;
                    use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                    use std::sync::Mutex;
                    static PERLAYER: Mutex<Vec<BTreeSet<u64>>> = Mutex::new(Vec::new());
                    static N: AtomicU64 = AtomicU64::new(0);
                    let li = self.config.layer_id;
                    if let Ok(mut v) = PERLAYER.lock() {
                        if v.len() <= li {
                            v.resize(li + 1, BTreeSet::new());
                        }
                        for lines in &total_perimeters {
                            for line in lines {
                                for j in &line.junctions {
                                    v[li].insert((j.w as f32).to_bits() as u64);
                                }
                            }
                        }
                        let n = N.fetch_add(1, Relaxed) + 1;
                        if n % 4000 == 0 {
                            // FLAT layers from R658 vs two healthy controls.
                            let flat = [2usize, 3, 4, 5, 33, 66, 69, 70, 75, 76, 77, 80];
                            let ctrl = [150usize, 300, 450];
                            let g = |i: usize| v.get(i).map(|s| s.len()).unwrap_or(0);
                            eprintln!(
                                "[WTPLAYER] n={n} FLAT{:?}={:?} CTRL{:?}={:?}",
                                flat,
                                flat.iter().map(|&i| g(i)).collect::<Vec<_>>(),
                                ctrl,
                                ctrl.iter().map(|&i| g(i)).collect::<Vec<_>>()
                            );
                        }
                    }
                }
            } else {
                // PerimeterGenerator.cpp:1634  infill_contour = last;
                surface_infill = last;
            }

            // PerimeterGenerator.cpp:1654-1667  wall ordering direction.
            let mut start_perimeter: i32 = total_perimeters.len() as i32 - 1;
            let mut end_perimeter: i32 = -1;
            let mut direction: i32 = -1;

            let is_outer_wall_first = self.config.wall_sequence
                == crate::print_config::WallSequence::OuterInner
                || self.config.wall_sequence == crate::print_config::WallSequence::InnerOuterInner;
            if is_outer_wall_first {
                start_perimeter = 0;
                end_perimeter = total_perimeters.len() as i32;
                direction = 1;
            }

            // PerimeterGenerator.cpp:1669-1675  collect all_extrusions in order.
            let mut all_extrusions: Vec<ArachneExtrusionLine> = Vec::new();
            let mut perimeter_idx = start_perimeter;
            while perimeter_idx != end_perimeter {
                if let Some(perim) = total_perimeters.get(perimeter_idx as usize) {
                    if !perim.is_empty() {
                        for wall in perim.iter() {
                            all_extrusions.push(wall.clone());
                        }
                    }
                }
                perimeter_idx += direction;
            }

            // PerimeterGenerator.cpp:1677-1689  region-order constraints.
            let extrusion_refs: Vec<&ArachneExtrusionLine> = all_extrusions.iter().collect();
            let extrusions_constrains =
                WallToolPaths::get_region_order(&extrusion_refs, is_outer_wall_first);
            let mut blocked: Vec<usize> = vec![0; all_extrusions.len()];
            let mut blocking: Vec<Vec<usize>> = vec![Vec::new(); all_extrusions.len()];
            for (before, after) in extrusions_constrains.into_iter() {
                blocked[after] += 1;
                blocking[before].push(after);
            }

            // PerimeterGenerator.cpp:1691-1746  greedy nearest-neighbour topo order.
            let mut processed: Vec<bool> = vec![false; all_extrusions.len()];
            let mut current_position: Point = if all_extrusions.is_empty() {
                Point::new(0, 0)
            } else {
                all_extrusions[0].junctions[0].p
            };
            let mut ordered_extrusions: Vec<ArachneExtrusionLine> =
                Vec::with_capacity(all_extrusions.len());

            while ordered_extrusions.len() < all_extrusions.len() {
                let mut best_candidate: usize = 0;
                let mut best_distance_sqr: f64 = f64::MAX;
                let mut is_best_closed: bool = false;

                let mut available_candidates: Vec<usize> = Vec::new();
                for candidate in 0..all_extrusions.len() {
                    if processed[candidate] || blocked[candidate] != 0 {
                        continue;
                    }
                    available_candidates.push(candidate);
                }
                // is_closed false sorts before true.
                available_candidates
                    .sort_by(|&a, &b| all_extrusions[a].is_closed.cmp(&all_extrusions[b].is_closed));

                for candidate_path_idx in available_candidates.into_iter() {
                    let path = &all_extrusions[candidate_path_idx];
                    if path.junctions.is_empty() {
                        if best_distance_sqr == f64::MAX {
                            best_candidate = candidate_path_idx;
                            is_best_closed = path.is_closed;
                        }
                        continue;
                    }
                    let candidate_position = path.junctions[0].p;
                    let distance_sqr = (current_position - candidate_position).length();
                    if distance_sqr < best_distance_sqr {
                        if path.is_closed
                            || (!path.is_closed && best_distance_sqr != f64::MAX)
                            || (!path.is_closed && !is_best_closed)
                        {
                            best_candidate = candidate_path_idx;
                            best_distance_sqr = distance_sqr;
                            is_best_closed = path.is_closed;
                        }
                    }
                }

                if all_extrusions.is_empty() {
                    break;
                }
                let best_path = all_extrusions[best_candidate].clone();
                processed[best_candidate] = true;
                for &unlocked_idx in blocking[best_candidate].iter() {
                    blocked[unlocked_idx] -= 1;
                }
                if !best_path.junctions.is_empty() {
                    current_position = if best_path.is_closed {
                        best_path.junctions[0].p
                    } else {
                        best_path.junctions.last().unwrap().p
                    };
                }
                ordered_extrusions.push(best_path);
            }

            // PerimeterGenerator.cpp:1748-1790 — BBS: adjust wall generate seq for
            // InnerOuterInner. Re-order each (outer, first_internal, second_internal)
            // triplet so the second internal wall is printed first, then outer, then
            // first internal (a rotation of the three).
            if self.config.wall_sequence == crate::print_config::WallSequence::InnerOuterInner {
                // 3 walls minimum needed to do inner outer inner ordering
                if ordered_extrusions.len() > 2 {
                    let mut position: usize = 0;
                    loop {
                        if position >= ordered_extrusions.len() {
                            break;
                        }
                        let mut outer: i64 = -1;
                        let mut first_internal: i64 = -1;
                        let mut second_internal: i64 = -1;
                        let mut arr_i: usize = position;
                        while arr_i < ordered_extrusions.len() {
                            match ordered_extrusions[arr_i].inset_idx {
                                0 => {
                                    // external perimeter
                                    if outer == -1 {
                                        outer = arr_i as i64;
                                    }
                                }
                                1 => {
                                    // first internal wall
                                    if first_internal == -1 && arr_i as i64 > outer && outer != -1 {
                                        first_internal = arr_i as i64;
                                    }
                                }
                                2 => {
                                    // second internal wall
                                    if second_internal == -1
                                        && arr_i as i64 > first_internal
                                        && outer != -1
                                    {
                                        second_internal = arr_i as i64;
                                    }
                                }
                                _ => {}
                            }
                            if outer > -1 && first_internal > -1 && second_internal > -1 {
                                break; // found all three perimeters to re-order
                            }
                            arr_i += 1;
                        }
                        if outer > -1 && first_internal > -1 && second_internal > -1 {
                            // C++: rotate the triplet (temp = second; second = first; first = outer; outer = temp)
                            let (o, fi, si) =
                                (outer as usize, first_internal as usize, second_internal as usize);
                            ordered_extrusions.swap(si, fi);
                            ordered_extrusions.swap(fi, o);
                        } else {
                            break; // no more candidates to re-order
                        }
                        // arr_i points at the last index inspected (the break point)
                        position = arr_i + 1;
                    }
                }
            }

            // PerimeterGenerator.cpp:1792  traverse_extrusions -> append to loops.
            // R803e — PerimeterGenerator.cpp:684-707/937: with
            // z_direction_outwall_speed_continuous, native records ONE LoopNode per
            // outer ExtrusionLine (contour = junction points + widths, loop_id = the
            // entity index it is about to get) and brackets the collection's
            // loop_node_range. Rust only did this on the classic path, so arachne
            // layers had NO cooling nodes: no `; COOLING_NODE:` tags, and the
            // cross-layer outer-wall speed smoother in the GCodeEditor never ran.
            for ext in ordered_extrusions.iter() {
                let role = if ext.inset_idx == 0 {
                    ExtrusionRole::ExternalPerimeter
                } else {
                    ExtrusionRole::Perimeter
                };
                if arachne_nodes_on && ext.inset_idx == 0 {
                    let pts: Vec<crate::Point> = ext.junctions.iter().map(|j| j.p).collect();
                    let widths: Vec<crate::Coord> = ext.junctions.iter().map(|j| j.w).collect();
                    let bb = crate::geometry::BoundingBox::from_points(&pts);
                    // native: bbox.offset(outer_wall_line_width / 2) — a mm value on a
                    // scaled box, truncated to 0 by the coord_t conversion.
                    let mut node = crate::layer::LoopNode::new(
                        result.loop_nodes.len(),
                        entities.entities.len(),
                        bb,
                    );
                    node.node_contour = crate::layer::NodeContour { pts, widths, is_loop: ext.is_closed };
                    result.loop_nodes.push(node);
                }
                let ext_paths = self.arachne_line_to_extrusion_paths(ext, role);
                // R660 — AWIDTH. TPMPPROBE showed `thick_polyline_to_multi_path`
                // receives an ALREADY-FLAT ThickPolyline on 97.6% of calls, so the
                // collapse is not inside it. That probe is per-call, though, and
                // per-call flatness is compatible with plenty of per-LAYER variety
                // (different lines, different widths). This census answers the
                // per-layer question at the only site where `layer_id` is in scope
                // AND the ExtrusionPaths exist: distinct junction `w` going IN
                // against distinct `path.width` coming OUT, outer wall only, so it
                // is directly comparable to both WTPLAYER and the gcode.
                if crate::probe_enabled("AWIDTH") && role == ExtrusionRole::ExternalPerimeter {
                    use std::collections::BTreeSet;
                    use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                    use std::sync::Mutex;
                    static IN: Mutex<Vec<BTreeSet<u64>>> = Mutex::new(Vec::new());
                    static OUT: Mutex<Vec<BTreeSet<u64>>> = Mutex::new(Vec::new());
                    static NPATH: Mutex<Vec<u64>> = Mutex::new(Vec::new());
                    static N: AtomicU64 = AtomicU64::new(0);
                    let li = self.config.layer_id;
                    if let (Ok(mut vin), Ok(mut vout), Ok(mut vnp)) =
                        (IN.lock(), OUT.lock(), NPATH.lock())
                    {
                        if vin.len() <= li {
                            vin.resize(li + 1, BTreeSet::new());
                            vout.resize(li + 1, BTreeSet::new());
                            vnp.resize(li + 1, 0u64);
                        }
                        for j in &ext.junctions {
                            vin[li].insert((j.w as f32).to_bits() as u64);
                        }
                        for p in &ext_paths {
                            vout[li].insert((p.width as f32).to_bits() as u64);
                        }
                        vnp[li] += ext_paths.len() as u64;
                        let n = N.fetch_add(1, Relaxed) + 1;
                        // R660 — 250, not 4000: the R654 rule. At 4000 the last print
                        // landed 4k calls before the end and layers 300/450 still read
                        // zero, which is indistinguishable from "no variety there".
                        if n % 250 == 0 {
                            let flat = [2usize, 3, 4, 5, 33, 66, 69, 70, 75, 76, 77, 80];
                            let ctrl = [150usize, 300, 450];
                            let gi = |i: usize| vin.get(i).map(|s| s.len()).unwrap_or(0);
                            let go = |i: usize| vout.get(i).map(|s| s.len()).unwrap_or(0);
                            let gp = |i: usize| *vnp.get(i).unwrap_or(&0);
                            let tot_in: usize = vin.iter().map(|s| s.len()).sum();
                            let tot_out: usize = vout.iter().map(|s| s.len()).sum();
                            eprintln!(
                                "[AWIDTH] n={n} sum_in={tot_in} sum_out={tot_out}\n  FLAT  in={:?}\n  FLAT out={:?}\n  FLAT npath={:?}\n  CTRL  in={:?}\n  CTRL out={:?}\n  CTRL npath={:?}",
                                flat.iter().map(|&i| gi(i)).collect::<Vec<_>>(),
                                flat.iter().map(|&i| go(i)).collect::<Vec<_>>(),
                                flat.iter().map(|&i| gp(i)).collect::<Vec<_>>(),
                                ctrl.iter().map(|&i| gi(i)).collect::<Vec<_>>(),
                                ctrl.iter().map(|&i| go(i)).collect::<Vec<_>>(),
                                ctrl.iter().map(|&i| gp(i)).collect::<Vec<_>>(),
                            );
                        }
                    }
                }
                if !ext_paths.is_empty() {
                    if ext.is_closed {
                        // PerimeterGenerator.cpp:780 — elrDefault if contour, else elrPerimeterHole.
                        let is_contour = ext.is_contour();
                        let loop_role = if is_contour {
                            ExtrusionLoopRole::DEFAULT
                        } else {
                            ExtrusionLoopRole::PERIMETER_HOLE
                        };
                        // May be several paths after overhang splitting; the loop is
                        // the ordered chain of non-overhang + overhang segments.
                        let mut eloop = ExtrusionLoop::new(ext_paths, loop_role);
                        // PerimeterGenerator.cpp:781-785 — restore loop orientation:
                        // CCW for contours, CW for holes.
                        if is_contour {
                            eloop.make_counter_clockwise();
                        } else {
                            eloop.make_clockwise();
                        }
                        entities.append(ExtrusionEntityType::Loop(eloop));
                    } else if crate::faithful_gate("ARACHNE_MULTIPATH") {
                        // PerimeterGenerator.cpp:807-823 — the paths of an open
                        // extrusion form one ExtrusionMultiPath; a new one starts
                        // where two consecutive paths do not meet. =0 appends every
                        // path on its own, as the port did.
                        let mut it = ext_paths.into_iter();
                        if let Some(first) = it.next() {
                            let mut multi_path = crate::extrusion_entity::ExtrusionMultiPath::new();
                            multi_path.paths.push(first);
                            for path in it {
                                if multi_path.paths.last().unwrap().last_point() != path.first_point() {
                                    entities.append(ExtrusionEntityType::MultiPath(std::mem::replace(
                                        &mut multi_path,
                                        crate::extrusion_entity::ExtrusionMultiPath::new(),
                                    )));
                                }
                                multi_path.paths.push(path);
                            }
                            entities.append(ExtrusionEntityType::MultiPath(multi_path));
                        }
                    } else {
                        for path in ext_paths {
                            entities.append(ExtrusionEntityType::Path(path));
                        }
                    }
                }
            }

            // PerimeterGenerator.cpp:1795  const coord_t spacing = (total_perimeters.size()==1)
            //                                  ? ext_perimeter_spacing2 : perimeter_spacing;
            let spacing = if total_perimeters.len() == 1 {
                ext_perimeter_spacing2
            } else {
                perimeter_spacing
            };

            // PerimeterGenerator.cpp:1798  min_perimeter_infill_spacing = solid_infill_spacing * (1 - INSET_OVERLAP_TOLERANCE)
            let min_perimeter_infill_spacing =
                self.config.solid_infill_spacing * (1.0 - INSET_OVERLAP_TOLERANCE);

            // PerimeterGenerator.cpp:1800  add_infill_contour_for_arachne(infill_contour=surface_infill,
            //     loops=loop_number, ext_perimeter_spacing, perimeter_spacing, min_perimeter_infill_spacing,
            //     spacing, is_inner_part=false). Faithful port of the body (1436-1466).
            let (infill_pieces, no_overlap_pieces) = Self::add_infill_contour_for_arachne_legacy(
                surface_infill,
                loop_number,
                ext_perimeter_spacing as f64 / crate::SCALING_FACTOR,
                perimeter_spacing as f64 / crate::SCALING_FACTOR,
                min_perimeter_infill_spacing,
                spacing as f64 / crate::SCALING_FACTOR,
                false,
                self.config.infill_wall_overlap,
                self.config.surface_simplify_resolution,
                self.config.join_type,
                // coord_t(solid_infill_spacing * (1. - INSET_OVERLAP_TOLERANCE)),
                // on the coord_t solid_infill_flow.scaled_spacing().
                ((crate::clipper_utils::scale_faithful(self.config.solid_infill_spacing) as f64)
                    * (1.0 - INSET_OVERLAP_TOLERANCE)) as Coord,
            );
            infill_contour.extend(infill_pieces);
            no_overlap_contour.extend(no_overlap_pieces);
        }

        result.entities = entities;
        if arachne_nodes_on {
            result.entities.loop_node_range = (arachne_node_first, result.loop_nodes.len());
        }
        result.infill_area = infill_contour;
        result.no_overlap_area = no_overlap_contour;
        result
    }

    /// PerimeterGenerator.cpp:1436-1466
    /// C++: void PerimeterGenerator::add_infill_contour_for_arachne(ExPolygons infill_contour, int loops,
    ///          coord_t ext_perimeter_spacing, coord_t perimeter_spacing, coord_t min_perimeter_infill_spacing,
    ///          coord_t spacing, bool is_inner_part)
    /// Returns `(fill_surfaces, fill_no_overlap)` — C++ appends the first to
    /// `fill_surfaces` with stInternal (PerimeterGenerator.cpp:1458) and the second to
    /// `fill_no_overlap` (:1460). They are the same offset2_ex of the same
    /// `union_ex(inner_pp)`; only the outward step differs — `insert + min_pis/2` for
    /// the fill surface (grown by the infill_wall_overlap band so infill runs into the
    /// wall) versus a bare `min_pis/2` for the no-overlap copy. R474: the second was
    /// never modelled, so `LayerRegion::fill_no_overlap_expolygons` came out empty on
    /// every Arachne region, and the Concentric / FloatingConcentric fillers -- which
    /// fill `no_overlap_expolygons` and nothing else -- fell back to the overlap-grown
    /// expolygon.
    /// Spacing args are in mm here (the crate's offset/offset2 take mm).
    #[allow(clippy::too_many_arguments)]
    fn add_infill_contour_for_arachne_legacy(
        mut infill_contour: ExPolygons,
        loops: i32,
        ext_perimeter_spacing: f64,
        perimeter_spacing: f64,
        min_perimeter_infill_spacing: f64,
        spacing: f64,
        is_inner_part: bool,
        infill_wall_overlap: f64,
        surface_simplify_resolution: f64,
        join_type: OffsetJoinType,
        min_perimeter_infill_spacing_scaled: Coord,
    ) -> (ExPolygons, ExPolygons) {
        // ARACHNE_TAIL_SCALED (on): PerimeterGenerator.cpp:1463-1484 in coord_t.
        // The arguments are scaled integers there: `spacing / 2.` and
        // `min_perimeter_infill_spacing / 2.` are halves of integers passed
        // through `float`, `insert` is truncated after the overlap percentage,
        // each expolygon is simplified on its own (ExPolygon::simplify_p ends in
        // simplify_polygons) and the rings are united once. =0 computes the
        // deltas in millimetres and simplifies the collected rings a second time.
        if crate::faithful_gate("ARACHNE_TAIL_SCALED") && crate::faithful_gate("TOPFILL_FAITHFUL") {
            let sf = crate::SCALING_FACTOR;
            let to_coord = |mm: f64| -> Coord { (mm * sf).round() as Coord };
            let spacing_c = to_coord(spacing);
            let min_pis_c = min_perimeter_infill_spacing_scaled;
            if crate::clipper_utils::offset_expolygons_clib_scaled(
                &infill_contour,
                -((spacing_c as f64 / 2.0) as f32) as f64,
                OffsetJoinType::Miter,
            )
            .is_empty()
            {
                infill_contour.clear();
            }
            let mut insert_c: Coord = if loops < 0 { 0 } else { to_coord(ext_perimeter_spacing) };
            if is_inner_part || loops > 0 {
                insert_c = to_coord(perimeter_spacing);
            }
            // coord_t(scale_(infill_wall_overlap.get_abs_value(unscale<double>(insert))))
            let insert_c: Coord = ((infill_wall_overlap
                * (insert_c as f64 * crate::libslic3r::SCALING_FACTOR))
                / crate::libslic3r::SCALING_FACTOR) as Coord;
            let tol_scaled = surface_simplify_resolution * sf;
            let mut inner_pp: Vec<Polygon> = Vec::new();
            for ex in &infill_contour {
                let rings = ex.simplify_p_dp_rings_faithful(tol_scaled);
                inner_pp.extend(crate::clipper_utils::simplify_polygons_clib(&rings, 1));
            }
            if inner_pp.is_empty() {
                return (vec![], vec![]);
            }
            let inner_union = crate::clipper_utils::union_ex_clib(&inner_pp, 1);
            let d1 = ((-(min_pis_c as f64) / 2.0) as f32) as f64;
            let d2 = ((insert_c as f64 + min_pis_c as f64 / 2.0) as f32) as f64;
            let d3 = ((min_pis_c as f64 / 2.0) as f32) as f64;
            let mm = crate::libslic3r::SCALING_FACTOR;
            let fill = crate::clipper_utils::offset2_ex_clib(
                &inner_union,
                d1 * mm,
                d2 * mm,
                OffsetJoinType::Miter,
            );
            let no_overlap = crate::clipper_utils::offset2_ex_clib(
                &inner_union,
                d1 * mm,
                d3 * mm,
                OffsetJoinType::Miter,
            );
            return (fill, no_overlap);
        }
        // C++: if (offset_ex(infill_contour, -float(spacing / 2.)).empty()) infill_contour.clear();
        if shrink(&infill_contour, spacing / 2.0, join_type).is_empty() {
            infill_contour.clear();
        }

        // C++: coord_t insert = (loops < 0) ? 0 : ext_perimeter_spacing;
        //      if (is_inner_part || loops > 0) insert = perimeter_spacing;
        let mut insert = if loops < 0 { 0.0 } else { ext_perimeter_spacing };
        if is_inner_part || loops > 0 {
            insert = perimeter_spacing;
        }

        // C++: insert = scale_(infill_wall_overlap.get_abs_value(unscale<double>(insert)));
        // get_abs_value with a fraction: insert_mm * infill_wall_overlap.
        insert = infill_wall_overlap * insert;

        // C++: Polygons inner_pp; for (ExPolygon &ex : infill_contour) ex.simplify_p(m_scaled_resolution, &inner_pp);
        // R250 (gated): run the whole chain at ClipperLib precision — the geo
        // route re-grids at scale-1000 and produced the RAGGED fill_expolygons
        // (L71: rust 2 hole-y blobs n=272 vs native 4 clean pieces n<=69, the
        // root of the entire fill-geometry divergence, R249). Faithful:
        // simplify_p = DP rings + SimplifyPolygons, then union_ex, then
        // offset2_ex(-min_pis/2, insert+min_pis/2) with default jtMiter.
        if crate::faithful_gate("TOPFILL_FAITHFUL") {
            let tol_scaled = surface_simplify_resolution * crate::SCALING_FACTOR;
            let mut inner_pp: Vec<Polygon> = Vec::new();
            for ex in &infill_contour {
                // R814 TAIL_SIMPLIFY_POLYGONS: simplify_p ends in simplify_polygons.
                let rings = ex.simplify_p_dp_rings_faithful(tol_scaled);
                if crate::faithful_gate("TAIL_SIMPLIFY_POLYGONS") {
                    inner_pp.extend(crate::clipper_utils::simplify_polygons_clib(&rings, 1));
                } else {
                    inner_pp.extend(rings);
                }
            }
            if inner_pp.is_empty() {
                return (vec![], vec![]);
            }
            let simplified = crate::clipper_utils::simplify_polygons_clib(&inner_pp, 1);
            let inner_union = crate::clipper_utils::union_ex_clib(&simplified, 1);
            let fill = crate::clipper_utils::offset2_ex_clib(
                &inner_union,
                -(min_perimeter_infill_spacing / 2.0),
                insert + min_perimeter_infill_spacing / 2.0,
                OffsetJoinType::Miter,
            );
            // PerimeterGenerator.cpp:1460 — same offset2_ex, outward step WITHOUT
            // `insert`.
            let no_overlap = if crate::faithful_gate("ARACHNE_FILL_NO_OVERLAP") {
                crate::clipper_utils::offset2_ex_clib(
                    &inner_union,
                    -(min_perimeter_infill_spacing / 2.0),
                    min_perimeter_infill_spacing / 2.0,
                    OffsetJoinType::Miter,
                )
            } else {
                Vec::new()
            };
            return (fill, no_overlap);
        }
        let mut inner_pp: Vec<Polygon> = Vec::new();
        for ex in &infill_contour {
            ex.simplify_p_into(surface_simplify_resolution, &mut inner_pp);
        }
        let inner_union = union_polygons_ex(&inner_pp);

        // C++: this->fill_surfaces->append(offset2_ex(union_ex(inner_pp),
        //          -min_perimeter_infill_spacing/2., insert + min_perimeter_infill_spacing/2.), stInternal);
        let fill = offset2(
            &inner_union,
            min_perimeter_infill_spacing / 2.0,
            insert + min_perimeter_infill_spacing / 2.0,
            join_type,
        );
        // C++: append(*this->fill_no_overlap, offset2_ex(union_ex(inner_pp),
        //          -min_perimeter_infill_spacing/2., +min_perimeter_infill_spacing/2.));
        let no_overlap = if crate::faithful_gate("ARACHNE_FILL_NO_OVERLAP") {
            offset2(
                &inner_union,
                min_perimeter_infill_spacing / 2.0,
                min_perimeter_infill_spacing / 2.0,
                join_type,
            )
        } else {
            Vec::new()
        };
        (fill, no_overlap)
    }

    /// Convert a faithful Arachne `ExtrusionLine` to our `ExtrusionPath`.
    fn arachne_line_to_extrusion_paths(
        &self,
        line: &ArachneExtrusionLine,
        role: ExtrusionRole,
    ) -> Vec<ExtrusionPath> {
        if line.junctions.is_empty() {
            return Vec::new();
        }

        let avg_width: f64 = line
            .junctions
            .iter()
            .map(|j| crate::unscale(j.w))
            .sum::<f64>()
            / line.junctions.len() as f64;

        // R443/R444: an Arachne junction width `w` is a SPACING, not an extrusion
        // width — `WallToolPaths` is seeded with ext_perimeter_spacing /
        // perimeter_spacing (PerimeterGenerator.cpp:1566), so the beads come back in
        // that same convention. C++ converts back before building the flow
        // (VariableWidth.cpp:66,136,203):
        //     flow.with_width(unscale(w) + flow.height() * (1 - 0.25*PI))
        // Using `w` directly as a width under-extrudes by exactly that term: for
        // Majora (w 0.33562, h 0.3) it yielded 0.336 instead of 0.400, i.e. wall
        // E/mm 0.805x C++ and object-only material 0.878x. This is the same
        // spacing-vs-width convention that caused R413's +14% overshoot, in reverse.
        let avg_width = avg_width + self.config.layer_height * (1.0 - 0.25 * std::f64::consts::PI);

        // Get flow for this width
        let flow = match Flow::new(
            avg_width,
            self.config.layer_height,
            self.config.perimeter_extrusion_width,
        ) {
            Ok(f) => f,
            Err(_) => return Vec::new(),
        };
        let mm3_per_mm = flow.mm3_per_mm().unwrap_or(0.0);

        // Overhang classification (PerimeterGenerator.cpp:690-727). Split each
        // CLOSED wall loop into non-overhang (role) + overhang (erOverhangPerimeter)
        // segments via a Z-clipper intersection/difference against the grown lower
        // slices, then re-chain. The segments keep the loop's avg-width flow, so the
        // total deposited material is unchanged (v1) — only the feature role and the
        // overhang degree (speed) differ; a faithful per-segment variable-width pass
        // is deferred (R414: the variable-width builder double-applies the
        // spacing→width conversion). v1 covers Majora's closed outer/inner walls;
        // open extrusions keep the single-path form. Benchy/Cube are CLASSIC and
        // never reach here.
        // R538 probe (ARACHPROBE=1): C++'s guard at PerimeterGenerator.cpp:667 is
        // `detect_overhang_wall && layer_id > raft_layers` — it has NO `is_closed`
        // test. Count how many lines our extra condition excludes, split by inset,
        // before deciding whether that divergence matters.
        if crate::probe_enabled("ARACHPROBE") {
            use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
            static SEEN_EXT: AtomicUsize = AtomicUsize::new(0);
            static OPEN_EXT: AtomicUsize = AtomicUsize::new(0);
            static SEEN_INT: AtomicUsize = AtomicUsize::new(0);
            static OPEN_INT: AtomicUsize = AtomicUsize::new(0);
            let ext = line.inset_idx == 0;
            if ext {
                SEEN_EXT.fetch_add(1, Relaxed);
                if !line.is_closed {
                    OPEN_EXT.fetch_add(1, Relaxed);
                }
            } else {
                SEEN_INT.fetch_add(1, Relaxed);
                if !line.is_closed {
                    OPEN_INT.fetch_add(1, Relaxed);
                }
            }
            let tot = SEEN_EXT.load(Relaxed) + SEEN_INT.load(Relaxed);
            if tot % 5_000 == 0 {
                eprintln!(
                    "[ARACHPROBE] external: {} lines, {} OPEN ({:.1}%)  |  internal: {} lines, {} OPEN ({:.1}%)",
                    SEEN_EXT.load(Relaxed),
                    OPEN_EXT.load(Relaxed),
                    100.0 * OPEN_EXT.load(Relaxed) as f64 / SEEN_EXT.load(Relaxed).max(1) as f64,
                    SEEN_INT.load(Relaxed),
                    OPEN_INT.load(Relaxed),
                    100.0 * OPEN_INT.load(Relaxed) as f64 / SEEN_INT.load(Relaxed).max(1) as f64,
                );
            }
        }
        // ARACHNE_OVERHANG_OPEN (on unless =0): the guard of
        // PerimeterGenerator.cpp:667-668 has no `is_closed` test, so an open
        // extrusion is split against the lower slices as a closed one is. =0
        // keeps open extrusions out, as one supported path.
        let overhang_open = crate::faithful_gate("ARACHNE_OVERHANG_OPEN");
        if self.config.detect_overhang_wall && (line.is_closed || overhang_open) {
            if let Some(lower) = self.config.lower_slices.as_ref() {
                if !lower.is_empty() {
                    use crate::clipper_utils::OffsetJoinType;
                    use crate::clipper_z::{clip_extrusion, ClipType};
                    use crate::clipper_z_utils::{ZPath, ZPaths};

                    // Subject Z-path from the junction centreline (cpp:692-694).
                    let subject: ZPath =
                        line.junctions.iter().map(|j| (j.p.x(), j.p.y(), j.w)).collect();

                    // Grow the lower slices by half a nozzle: anything within that of
                    // the lower slice is "supported" (cpp process_arachne:26).
                    let nozzle = self.config.perimeter_flow.nozzle_diameter();
                    // R803c — native (PerimeterGenerator.cpp:1781):
                    //   m_lower_slices_polygons = offset(*lower_slices, float(scale_(+nozzle/2)))
                    // is ClipperLib raw_offset + union over ALL rings (contours and
                    // holes, orientation-signed), jtMiter/ML3. The geo `grow` route
                    // re-gridded the boundary and joined corners differently, so the
                    // signed distances behind every arachne overhang degree — and
                    // therefore the split positions of the speed pieces — drifted by
                    // up to ~0.9mm on straight stretches (layer-8 benchy-arachne).
                    // PerimeterGenerator.cpp:673 `Polygons lower_layer_polys = perimeter_generator.lower_slices_polygons();`
                    let grown_rings: Vec<Polygon> = self.lower_slices_polygons();
                    // ARACHNE_CLIP_BBOX (on): PerimeterGenerator.cpp:675-692 clips each
                    // grown lower polygon to the bounding box of the extrusion's
                    // junctions (`clip_clipper_polygon_with_subject_bbox(.., true)`,
                    // empties dropped) and uses the result both as the clip paths
                    // and as the distancer's polygons. The box is not inflated: the
                    // `inflated()` at :684 returns a copy that is discarded. =0 clips
                    // against the whole grown rings.
                    let grown_rings: Vec<Polygon> = if crate::faithful_gate("ARACHNE_CLIP_BBOX") {
                        let mut extrusion_path_bbox = crate::geometry::BoundingBox::new();
                        for j in &line.junctions {
                            extrusion_path_bbox.merge_point(Point::new(j.p.x(), j.p.y()));
                        }
                        grown_rings
                            .iter()
                            .filter_map(|ring| {
                                let new_poly = crate::clipper_utils::clip_clipper_polygon_with_subject_bbox(
                                    ring,
                                    &extrusion_path_bbox,
                                    true,
                                );
                                if new_poly.points.is_empty() {
                                    None
                                } else {
                                    Some(new_poly)
                                }
                            })
                            .collect()
                    } else {
                        grown_rings
                    };
                    let mut clip: ZPaths = Vec::new();
                    for ring in &grown_rings {
                        clip.push(ring.points.iter().map(|p| (p.x(), p.y(), 0i64)).collect());
                    }

                    // Build an avg-width ExtrusionPath from a split Z-path.
                    let mk = |zpath: &ZPath, r: ExtrusionRole, degree: f64| -> Option<ExtrusionPath> {
                        if zpath.len() < 2 {
                            return None;
                        }
                        let pts: Vec<Point> = zpath.iter().map(|&(x, y, _)| Point::new(x, y)).collect();
                        let mut p = ExtrusionPath::new(r);
                        p.polyline = Polyline::from_points(pts);
                        p.mm3_per_mm = mm3_per_mm;
                        p.width = avg_width;
                        p.height = self.config.layer_height;
                        p.overhang_degree = degree;
                        Some(p)
                    };

                    // R541 — the VARIABLE-WIDTH builder.
                    //
                    // C++ builds every arachne wall path through
                    // `thick_polyline_to_multi_path` (Arachne/utils/ExtrusionLine.cpp:301-310),
                    // taking the width of each segment from the ThickPolyline's per-junction
                    // `z` value; the `flow` argument supplies only role/height. `mk()` above
                    // instead stamps ONE avg width per loop, so we emitted 2,873 `; LINE_WIDTH:`
                    // changes on Majora's outer wall against C++'s 62,582 (R540).
                    //
                    // R414 recorded this as blocked because "the variable-width builder
                    // double-applies the Arachne spacing->width conversion". Re-derived on the
                    // real code (R504/R540): `variable_width.rs::thick_polyline_to_multi_path`
                    // applies `spacing_to_width` EXACTLY ONCE (:198/:311/:423) and is a faithful
                    // port of VariableWidth.cpp:66/136/203, f32 fidelity gate included. The
                    // double-apply is a CALLER hazard — it happens only if the caller hands it
                    // an already-converted width. So we pass the RAW junction widths (the
                    // ZPath `z`, still in Arachne's spacing convention) and let the builder do
                    // the single conversion.
                    //
                    // `extrusion_paths_append_zpaths` and `detect_bridge_wall_arachne` were
                    // both already ported (R412) and carry a note saying "R413 wires it" —
                    // R413 never did, and they have been dead code since.
                    // R541 probe (ARACHPROBE=1): the builder is faithful, so if the emitted
                    // widths barely vary the INPUT junction widths must be near-constant.
                    // Measure the per-loop spread of the Arachne junction `w` directly.
                    if crate::probe_enabled("ARACHPROBE") {
                        use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
                        static LOOPS: AtomicUsize = AtomicUsize::new(0);
                        static FLAT: AtomicUsize = AtomicUsize::new(0);
                        static SPREAD_UM: AtomicUsize = AtomicUsize::new(0);
                        static JUNCS: AtomicUsize = AtomicUsize::new(0);
                        static DISTINCT: AtomicUsize = AtomicUsize::new(0);
                        // R567: the ORDERING-sensitive quantity. R566 established that
                        // the output gap is a RATE — C++ changes width 2.73x more often
                        // per extruded move (0.0368 vs 0.1003). Distinct-value counts
                        // (R558, 1.24x) and spread (R549, MATCHED) are both blind to
                        // ordering, so neither can speak to a rate. This counts how often
                        // CONSECUTIVE junctions differ, which is the direct internal
                        // analogue: transitions = n-1 per loop, changes = #{i : w[i] != w[i-1]}.
                        // Tracked separately over ALL loops and over VARYING loops (mx != mn)
                        // so the varying sub-population can be reported without building a
                        // decomposition whose split criterion is the outcome (R566).
                        static TRANSITIONS: AtomicUsize = AtomicUsize::new(0);
                        static CHANGES: AtomicUsize = AtomicUsize::new(0);
                        static V_LOOPS: AtomicUsize = AtomicUsize::new(0);
                        static V_TRANS: AtomicUsize = AtomicUsize::new(0);
                        static V_CHANGES: AtomicUsize = AtomicUsize::new(0);
                        // R567: OUTER WALL only (inset_idx == 0). The 2.73x output rate
                        // is measured over outer-wall feature blocks, so an all-loops
                        // internal rate is a scope mismatch (R507) — the exact error class
                        // that cost R559-R566. This bucket makes the comparison sound.
                        static O_LOOPS: AtomicUsize = AtomicUsize::new(0);
                        static O_TRANS: AtomicUsize = AtomicUsize::new(0);
                        static O_CHANGES: AtomicUsize = AtomicUsize::new(0);
                        let ws: Vec<i64> = line.junctions.iter().map(|j| j.w).collect();
                        if !ws.is_empty() {
                            let mn = *ws.iter().min().unwrap();
                            let mx = *ws.iter().max().unwrap();
                            let mut u: Vec<i64> = ws.clone();
                            u.sort_unstable();
                            u.dedup();
                            let n = LOOPS.fetch_add(1, Relaxed) + 1;
                            JUNCS.fetch_add(ws.len(), Relaxed);
                            DISTINCT.fetch_add(u.len(), Relaxed);
                            let changes = ws.windows(2).filter(|p| p[0] != p[1]).count();
                            let trans = ws.len().saturating_sub(1);
                            TRANSITIONS.fetch_add(trans, Relaxed);
                            CHANGES.fetch_add(changes, Relaxed);
                            // spread in microns (crate scale is 1e5 units/mm)
                            SPREAD_UM.fetch_add(((mx - mn) as f64 / 100.0) as usize, Relaxed);
                            if mx == mn {
                                FLAT.fetch_add(1, Relaxed);
                            } else {
                                V_LOOPS.fetch_add(1, Relaxed);
                                V_TRANS.fetch_add(trans, Relaxed);
                                V_CHANGES.fetch_add(changes, Relaxed);
                            }
                            if line.inset_idx == 0 {
                                O_LOOPS.fetch_add(1, Relaxed);
                                O_TRANS.fetch_add(trans, Relaxed);
                                O_CHANGES.fetch_add(changes, Relaxed);
                            }
                            if n % 5_000 == 0 {
                                let (tr, ch) = (TRANSITIONS.load(Relaxed), CHANGES.load(Relaxed));
                                let (vt, vc) = (V_TRANS.load(Relaxed), V_CHANGES.load(Relaxed));
                                eprintln!(
                                    "[ARACHWIDTH] loops={n} flat(min==max)={} ({:.1}%) mean_spread={:.1}um \
                                     juncs/loop={:.1} distinct_w/loop={:.2} | \
                                     CHANGE_RATE all={:.4} ({}/{}) varying={:.4} ({}/{}) v_loops={} \
                                     OUTER={:.4} ({}/{}) o_loops={}",
                                    FLAT.load(Relaxed),
                                    100.0 * FLAT.load(Relaxed) as f64 / n as f64,
                                    SPREAD_UM.load(Relaxed) as f64 / n as f64,
                                    JUNCS.load(Relaxed) as f64 / n as f64,
                                    DISTINCT.load(Relaxed) as f64 / n as f64,
                                    ch as f64 / tr.max(1) as f64, ch, tr,
                                    vc as f64 / vt.max(1) as f64, vc, vt,
                                    V_LOOPS.load(Relaxed),
                                    O_CHANGES.load(Relaxed) as f64
                                        / O_TRANS.load(Relaxed).max(1) as f64,
                                    O_CHANGES.load(Relaxed),
                                    O_TRANS.load(Relaxed),
                                    O_LOOPS.load(Relaxed),
                                );
                            }
                        }
                    }
                    let variable_width = crate::faithful_gate("ARACHNE_VARIABLE_WIDTH");
                    let vw_flow = flow.clone();
                    let mut push_zpath =
                        |paths: &mut Vec<ExtrusionPath>, zp: &ZPath, r: ExtrusionRole, degree: f64| {
                            if variable_width {
                                crate::arachne::utils::extrusion_line::extrusion_paths_append_zpaths(
                                    paths,
                                    &vec![zp.clone()],
                                    r,
                                    &vw_flow,
                                    degree,
                                );
                            } else if let Some(p) = mk(zp, r, degree) {
                                paths.push(p);
                            }
                        };

                    let mut paths: Vec<ExtrusionPath> = Vec::new();
                    // Non-overhang = intersection with the grown lower slices.
                    //
                    // R537 — PerimeterGenerator.cpp:703-716. C++ picks between two
                    // treatments of this supported part:
                    //   if (is_enable_overhang_speed(pg) && fuzzy_skin_allows_overhang_slowdown(pg))
                    //       paths = detect_overhang_degree(flow, role, lower_layer_polys,
                    //                                      clip_paths, subject_path, nozzle_diameter);
                    //   else
                    //       plain ctIntersection -> one path, degree 0
                    // We had only the `else`, which left `overhang_degree = 0` on every
                    // supported segment. With no degree differences there are no speed
                    // discontinuities, so the smoothing pass in the exporter (gated on
                    // `paths.len() > 1`) never fired: 80% of Majora's outer wall carried
                    // a single feedrate against C++'s 49,568 distinct ones (R535/R536).
                    // C++'s guard resolves TRUE for our fixtures, so it is treated as
                    // constant here exactly as the classic path does at :2614:
                    // `is_enable_overhang_speed` (PerimeterGenerator.cpp:269-277) reads
                    // `override_process_overhang_speed[filament]` — all '0' on Majora —
                    // and therefore falls back to `enable_overhang_speed`, which is
                    // ['1','1']. `fuzzy_skin_allows_overhang_slowdown` (:279-283) holds
                    // because neither fixture enables fuzzy skin.
                    // SPLITPROBE (R575) counters for the two splitting branches below.
                    let mut n_supported = 0usize;
                    let mut n_overhang = 0usize;
                    // REDPROBE (R583) — does the clip destroy width changes?
                    // R582: both engines discard POINTS at the same rate (0.962x)
                    // but we retain only 9.0% of assembly width-changes against
                    // C++'s 17.8% (1.972x). Count changes IN (the subject ZPath,
                    // built 1:1 from junctions) versus OUT (the clipped ZPaths).
                    let mut red_pts_out = 0usize;
                    let mut red_ch_out = 0usize;
                    if crate::faithful_gate("ARACHNE_OVERHANG_DEGREE") {
                        // C++ passes `lower_slices_polygons()` — which is
                        // `offset(*lower_slices, scale_(+nozzle_diameter/2))`
                        // (PerimeterGenerator.cpp:1495) — as BOTH the clip paths and the
                        // distancer reference. So the distancer measures to the GROWN
                        // outline, and `detect_overhang_degree`'s `offset_width` term
                        // (+nozzle/2) converts that back to a distance from the raw lower
                        // slice. Handing it the ungrown polys double-counts the nozzle
                        // offset: it inflated every degree and over-split the outer wall
                        // 2.3x against C++ before this was corrected.
                        let lower_polys = grown_rings.clone();
                        OVHD_ON.with(|c| c.set(std::env::var("OVHD_LID").map(|w| w == self.config.layer_id.to_string()).unwrap_or(false) && line.inset_idx == 0));
                        for (zp, degree) in crate::overhang_detector::detect_overhang_degree_arachne(
                            &lower_polys,
                            &clip,
                            &subject,
                            nozzle,
                        ) {
                            n_supported += 1;
                            red_pts_out += zp.len();
                            red_ch_out += (1..zp.len()).filter(|&k| zp[k].2 != zp[k - 1].2).count();
                            push_zpath(&mut paths, &zp, role, degree);
                        }
                    } else {
                        for zp in clip_extrusion(&subject, &clip, ClipType::Intersection).iter() {
                            n_supported += 1;
                            red_pts_out += zp.len();
                            red_ch_out += (1..zp.len()).filter(|&k| zp[k].2 != zp[k - 1].2).count();
                            push_zpath(&mut paths, zp, role, 0.0);
                        }
                    }
                    // Overhang = difference (cpp:721); classify bridge (straight) vs
                    // curved-overhang like detect_brigde_wall_arachne (cpp:604-626).
                    let n = crate::overhang_detector::OVERHANG_SAMPLING_NUMBER as f64;
                    for zp in clip_extrusion(&subject, &clip, ClipType::Difference).iter() {
                        if zp.len() < 2 {
                            continue;
                        }
                        let (fx, fy, _) = zp[0];
                        let (lx, ly, _) = *zp.last().unwrap();
                        let line_len =
                            (((lx - fx) as f64).powi(2) + ((ly - fy) as f64).powi(2)).sqrt();
                        let mut poly_len = 0.0_f64;
                        for w in zp.windows(2) {
                            let (ax, ay, _) = w[0];
                            let (bx, by, _) = w[1];
                            poly_len +=
                                (((bx - ax) as f64).powi(2) + ((by - ay) as f64).powi(2)).sqrt();
                        }
                        let degree = if line_len < poly_len { n - 1.0 } else { n };
                        // R539 probe (ARACHPROBE=1): C++ emits 718 curved (degree 5) to
                        // 418 straight (degree 6) on Majora; ours is inverted. A ZPath
                        // with only two points tests as straight by construction, so
                        // count the point-count distribution of what we classify.
                        if crate::probe_enabled("ARACHPROBE") {
                            use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
                            static N2: AtomicUsize = AtomicUsize::new(0);
                            static N3_5: AtomicUsize = AtomicUsize::new(0);
                            static N6P: AtomicUsize = AtomicUsize::new(0);
                            static CURVED: AtomicUsize = AtomicUsize::new(0);
                            static STRAIGHT: AtomicUsize = AtomicUsize::new(0);
                            match zp.len() {
                                2 => N2.fetch_add(1, Relaxed),
                                3..=5 => N3_5.fetch_add(1, Relaxed),
                                _ => N6P.fetch_add(1, Relaxed),
                            };
                            if line_len < poly_len {
                                CURVED.fetch_add(1, Relaxed);
                            } else {
                                STRAIGHT.fetch_add(1, Relaxed);
                            }
                            let tot = CURVED.load(Relaxed) + STRAIGHT.load(Relaxed);
                            if tot % 100 == 0 {
                                eprintln!(
                                    "[ARACHBRIDGE] overhang zpaths={tot} pts=2:{} pts=3-5:{} pts>=6:{} | curved(deg5)={} straight(deg6)={}",
                                    N2.load(Relaxed),
                                    N3_5.load(Relaxed),
                                    N6P.load(Relaxed),
                                    CURVED.load(Relaxed),
                                    STRAIGHT.load(Relaxed),
                                );
                            }
                        }
                        n_overhang += 1;
                        red_pts_out += zp.len();
                        red_ch_out += (1..zp.len()).filter(|&k| zp[k].2 != zp[k - 1].2).count();
                        push_zpath(&mut paths, zp, ExtrusionRole::OverhangPerimeter, degree);
                    }

                    // SPLITPROBE (R575) — how many pieces does ONE assembled ExtrusionLine
                    // become at this site? R574 measured 6.37 builder calls per assembled
                    // outer line vs C++'s 3.39 (1.88x), aggregated over every stage between
                    // generateToolpaths and extrusion_paths_append. This attributes the
                    // pieces to the two branches here. Outer wall only, to match R574.
                    if crate::probe_enabled("REDPROBE") && role == ExtrusionRole::ExternalPerimeter {
                        use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                        static LINES: AtomicU64 = AtomicU64::new(0);
                        static PIN: AtomicU64 = AtomicU64::new(0);
                        static POUT: AtomicU64 = AtomicU64::new(0);
                        static CIN: AtomicU64 = AtomicU64::new(0);
                        static COUT: AtomicU64 = AtomicU64::new(0);
                        let ch_in = (1..subject.len())
                            .filter(|&k| subject[k].2 != subject[k - 1].2)
                            .count();
                        PIN.fetch_add(subject.len() as u64, Relaxed);
                        POUT.fetch_add(red_pts_out as u64, Relaxed);
                        CIN.fetch_add(ch_in as u64, Relaxed);
                        COUT.fetch_add(red_ch_out as u64, Relaxed);
                        let n = LINES.fetch_add(1, Relaxed) + 1;
                        if n % 2_000 == 0 {
                            let (pi, po) = (PIN.load(Relaxed), POUT.load(Relaxed));
                            let (ci, co) = (CIN.load(Relaxed), COUT.load(Relaxed));
                            eprintln!(
                                "[REDPROBE] lines={} pts_in={} pts_out={} ({:.4}) ch_in={} ch_out={} ({:.4})",
                                n, pi, po, po as f64 / pi.max(1) as f64,
                                ci, co, co as f64 / ci.max(1) as f64,
                            );
                        }
                    }

                    if crate::probe_enabled("SPLITPROBE") && role == ExtrusionRole::ExternalPerimeter
                    {
                        use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                        static LINES: AtomicU64 = AtomicU64::new(0);
                        static PIECES: AtomicU64 = AtomicU64::new(0);
                        static SUPP: AtomicU64 = AtomicU64::new(0);
                        static OVER: AtomicU64 = AtomicU64::new(0);
                        static JUNCS: AtomicU64 = AtomicU64::new(0);
                        LINES.fetch_add(1, Relaxed);
                        PIECES.fetch_add(paths.len() as u64, Relaxed);
                        SUPP.fetch_add(n_supported as u64, Relaxed);
                        OVER.fetch_add(n_overhang as u64, Relaxed);
                        JUNCS.fetch_add(line.junctions.len() as u64, Relaxed);
                        let l = LINES.load(Relaxed);
                        if l % 2_000 == 0 {
                            eprintln!(
                                "[SPLITPROBE] lines={} juncs={} pieces={} supported={} overhang={}",
                                l,
                                JUNCS.load(Relaxed),
                                PIECES.load(Relaxed),
                                SUPP.load(Relaxed),
                                OVER.load(Relaxed),
                            );
                        }
                    }

                    if !paths.is_empty() {
                        // Re-chain the split segments into one continuous loop (cpp:763).
                        let mut start = paths[0].polyline.first_point();
                        if !line.is_closed {
                            // PerimeterGenerator.cpp:728-757 — for an open extrusion
                            // the start is an end of the chain (a point that occurs
                            // once), a non-overhang one if there is one. C++ walks a
                            // `std::unordered_map<Point, PointInfo, PointHash>`, so
                            // which end wins is the container's iteration order.
                            let mut point_occurrence = LibcxxUnorderedPointMap::new();
                            for path in &paths {
                                let is_overhang = path.role == ExtrusionRole::OverhangPerimeter;
                                for pt in [path.polyline.first_point(), path.polyline.last_point()] {
                                    let info = point_occurrence.entry(pt);
                                    info.0 += 1;
                                    if is_overhang {
                                        info.1 = true;
                                    }
                                }
                            }
                            for (pt, (occurrence, is_overhang)) in point_occurrence.in_order() {
                                if occurrence == 1 {
                                    start = pt;
                                    if !is_overhang {
                                        break;
                                    }
                                }
                            }
                        }
                        crate::shortest_path::chain_and_reorder_extrusion_paths(
                            &mut paths,
                            Some(&start),
                        );
                        // PerimeterGenerator.cpp:836-838 — after chaining, native
                        // lowpass-filters the per-piece overhang degrees
                        // (smooth_overhang_level) whenever overhang speed is on.
                        // R803: rust had NO twin, so every short degree flip
                        // reached the writer as its own speed change: arachne
                        // benchy outer walls carried +7,936 speed-only `G1 F`
                        // lines (20,032 v 12,096).
                        if crate::faithful_gate("ARACHNE_OVERHANG_DEGREE")
                            && crate::faithful_gate("ARACHNE_SMOOTH_OVERHANG")
                        {
                            smooth_overhang_level(&mut paths);
                        }
                        if let Ok(want) = std::env::var("OVHDEG_LID") {
                            if want == self.config.layer_id.to_string() {
                                use std::fmt::Write as _;
                                let mut o = String::new();
                                for (i, p) in paths.iter().enumerate() {
                                    let r = match p.role { ExtrusionRole::ExternalPerimeter => 'E', ExtrusionRole::Perimeter => 'P', ExtrusionRole::OverhangPerimeter => 'O', _ => '?' };
                                    let fp = p.polyline.first_point();
                                    let _ = writeln!(o, "OVHDEG lid={} line={} i={} role={} deg={} degf={:.4} len={:.1} x={} y={} w={:.4}", self.config.layer_id, line.inset_idx, i, r, p.overhang_degree as i32, p.overhang_degree, p.length(), fp.x, fp.y, p.width);
                                }
                                eprint!("{}", o);
                            }
                        }
                        return paths;
                    }
                    // else fall through to the single-path form.
                }
            }
        }

        // Single-path form (no overhang split).
        //
        // R558 — this was the LAST place an Arachne loop's per-junction widths
        // were averaged away. C++'s matching branch (PerimeterGenerator.cpp:772)
        // is a single call:
        //     extrusion_paths_append(paths, *extrusion, role, flow);
        // i.e. the `ExtrusionLine` overload (ExtrusionLine.cpp:301-305), which is
        // `to_thick_polyline(extrusion)` -> `thick_polyline_to_multi_path(...)` and
        // therefore emits ONE ExtrusionPath PER WIDTH CHANGE along the loop. We
        // instead built ONE path carrying `avg_width`, so every non-overhang loop
        // left the generator with a single constant width.
        //
        // R541 fixed exactly this for the OVERHANG-SPLIT branch above (via
        // `extrusion_paths_append_zpaths`) and lifted Majora's outer-wall
        // `; LINE_WIDTH:` count 2,873 -> 17,270. But loops that never reach the
        // split — the common case — still fell through to here, which is why the
        // count stalled at 17,270 against C++'s 62,582 (1.19 vs 4.21 tags per
        // outer-wall feature-block) while the polyline structure itself already
        // matched C++ to within 10% (36.1 vs 42.0 extrude moves per block, 3.55
        // vs 3.76 moves per run). The gap was never geometry; it was this average.
        //
        // `extrusion_paths_append_line` is the faithful port of that C++ overload.
        // It was written in R412 with a note saying "R413 wires it" — R413 never
        // did, and it has been dead code (its definition the only occurrence in
        // the crate) ever since, exactly as `extrusion_paths_append_zpaths` was
        // until R541.
        //
        // The gate covers the WIDTH SPLIT ONLY. The `points.reverse()` below is a
        // Rust-only artifact with no C++ counterpart, but it fires only for OPEN
        // lines, so it is preserved verbatim here rather than bundled into this
        // experiment (R500).
        // R558 LINEPROBE — census of the FALL-THROUGH population specifically
        // (ARACHPROBE only samples loops that reach the overhang split, which is
        // the other branch entirely). Reports how much width variation the input
        // junctions actually carry and how many paths the builder makes of them.
        if crate::probe_enabled("LINEPROBE") {
            use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
            static LOOPS: AtomicUsize = AtomicUsize::new(0);
            static FLAT: AtomicUsize = AtomicUsize::new(0);
            static JUNCS: AtomicUsize = AtomicUsize::new(0);
            static DISTINCT: AtomicUsize = AtomicUsize::new(0);
            static SPREAD_UM: AtomicUsize = AtomicUsize::new(0);
            static PATHS: AtomicUsize = AtomicUsize::new(0);
            let ws: Vec<i64> = line.junctions.iter().map(|j| j.w).collect();
            let (mn, mx) = (
                *ws.iter().min().unwrap_or(&0),
                *ws.iter().max().unwrap_or(&0),
            );
            let mut u = ws.clone();
            u.sort_unstable();
            u.dedup();
            let n = LOOPS.fetch_add(1, Relaxed) + 1;
            JUNCS.fetch_add(ws.len(), Relaxed);
            DISTINCT.fetch_add(u.len(), Relaxed);
            SPREAD_UM.fetch_add(((mx - mn) as f64 / 100.0) as usize, Relaxed);
            if mx == mn {
                FLAT.fetch_add(1, Relaxed);
            }
            let mut probe_paths: Vec<ExtrusionPath> = Vec::new();
            crate::arachne::utils::extrusion_line::extrusion_paths_append_line(
                &mut probe_paths,
                line,
                role,
                &flow,
                0.0,
            );
            PATHS.fetch_add(probe_paths.len(), Relaxed);
            let mut wu: Vec<i64> = probe_paths
                .iter()
                .map(|p| (p.width * 1e6) as i64)
                .collect();
            wu.dedup();
            if n == 1 || n % 2_000 == 0 {
                eprintln!(
                    "[LINEPROBE] fallthrough_loops={n} flat(min==max)={} ({:.1}%) \
                     juncs/loop={:.2} distinct_w/loop={:.3} mean_spread={:.1}um \
                     paths/loop={:.3}",
                    FLAT.load(Relaxed),
                    100.0 * FLAT.load(Relaxed) as f64 / n as f64,
                    JUNCS.load(Relaxed) as f64 / n as f64,
                    DISTINCT.load(Relaxed) as f64 / n as f64,
                    SPREAD_UM.load(Relaxed) as f64 / n as f64,
                    PATHS.load(Relaxed) as f64 / n as f64,
                );
            }
        }
        if crate::faithful_gate("ARACHNE_LINE_VARIABLE_WIDTH") {
            let mut paths: Vec<ExtrusionPath> = Vec::new();
            // Preserve the open-line reversal by reversing the junctions, so the
            // resulting polyline is point-for-point what the averaged form built.
            let reversed;
            let src = if !line.is_closed && line.junctions.len() > 1 {
                let mut l = line.clone();
                l.junctions.reverse();
                reversed = l;
                &reversed
            } else {
                line
            };
            // PerimeterGenerator.cpp:772 / ExtrusionLine.cpp:301-305, overhang = 0.
            crate::arachne::utils::extrusion_line::extrusion_paths_append_line(
                &mut paths, src, role, &flow, 0.0,
            );
            if !paths.is_empty() {
                return paths;
            }
            // thick_polyline_to_multi_path can return nothing for a degenerate
            // (zero-length) loop; fall through to the averaged form rather than
            // dropping the extrusion entirely.
        }
        let mut points: Vec<Point> = line.junctions.iter().map(|j| j.p).collect();
        if !line.is_closed && points.len() > 1 {
            points.reverse();
        }
        let mut path = ExtrusionPath::new(role);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = mm3_per_mm;
        path.width = avg_width;
        path.height = self.config.layer_height;
        vec![path]
    }
}

/// A libc++ `std::unordered_map<Point, PointInfo, PointHash>` that is only
/// inserted into, with its iteration order (PerimeterGenerator.cpp:739-755).
/// The value is `(occurrence, is_overhang)`. The container is the one
/// `region_expansion.rs` emulates for `double` keys; `PointHash`
/// (Point.hpp:368-372) is `coord_t((89 * 31 + int64_t(x)) * 31 + y)`.
struct LibcxxUnorderedPointMap {
    /// (hash, key, value, next)
    nodes: Vec<(u64, Point, (usize, bool), Option<usize>)>,
    /// First node of the list (`__p1_.first().__next_`).
    head: Option<usize>,
    /// Per bucket: the node before its first node. `Some(None)` is the
    /// before-begin anchor, `None` an empty bucket.
    buckets: Vec<Option<Option<usize>>>,
}

impl LibcxxUnorderedPointMap {
    fn new() -> Self {
        Self { nodes: Vec::new(), head: None, buckets: Vec::new() }
    }

    fn hash(key: &Point) -> u64 {
        ((89i64 * 31).wrapping_add(key.x()).wrapping_mul(31).wrapping_add(key.y())) as u64
    }

    /// `std::__constrain_hash`
    fn constrain(h: u64, bc: usize) -> usize {
        let bc64 = bc as u64;
        if bc64 & (bc64 - 1) == 0 {
            (h & (bc64 - 1)) as usize
        } else if h < bc64 {
            h as usize
        } else {
            (h % bc64) as usize
        }
    }

    fn next_of(&self, pred: Option<usize>) -> Option<usize> {
        match pred {
            None => self.head,
            Some(i) => self.nodes[i].3,
        }
    }

    fn set_next(&mut self, pred: Option<usize>, next: Option<usize>) {
        match pred {
            None => self.head = next,
            Some(i) => self.nodes[i].3 = next,
        }
    }

    /// `std::__next_prime`: the smallest prime not below `n`.
    fn next_prime(mut n: usize) -> usize {
        fn is_prime(n: usize) -> bool {
            if n < 2 {
                return false;
            }
            let mut d = 2usize;
            while d * d <= n {
                if n % d == 0 {
                    return false;
                }
                d += 1;
            }
            true
        }
        while !is_prime(n) {
            n += 1;
        }
        n
    }

    /// `__hash_table::__rehash<true>` on the growing path, with `__do_rehash`.
    fn rehash(&mut self, mut n: usize) {
        if n == 1 {
            n = 2;
        } else if n & (n - 1) != 0 {
            n = Self::next_prime(n);
        }
        if n <= self.buckets.len() {
            return;
        }
        self.buckets = vec![None; n];
        let mut pp: Option<usize> = None; // before-begin
        let Some(first) = self.head else { return };
        let mut phash = Self::constrain(self.nodes[first].0, n);
        self.buckets[phash] = Some(pp);
        pp = Some(first);
        let mut cp = self.nodes[first].3;
        while let Some(c) = cp {
            let chash = Self::constrain(self.nodes[c].0, n);
            if chash == phash {
                pp = Some(c);
            } else if self.buckets[chash].is_none() {
                self.buckets[chash] = Some(pp);
                pp = Some(c);
                phash = chash;
            } else {
                // Unlink `c` and put it behind its bucket's predecessor.
                let after = self.nodes[c].3;
                self.set_next(pp, after);
                let bucket_pred = self.buckets[chash].unwrap();
                let bucket_first = self.next_of(bucket_pred);
                self.nodes[c].3 = bucket_first;
                self.set_next(bucket_pred, Some(c));
            }
            cp = self.next_of(pp);
        }
    }

    /// `operator[]`: the value under `key`, default-constructed if new.
    fn entry(&mut self, key: Point) -> &mut (usize, bool) {
        let hash = Self::hash(&key);
        let mut bc = self.buckets.len();
        let mut chash = 0usize;
        if bc != 0 {
            chash = Self::constrain(hash, bc);
            if let Some(pred) = self.buckets[chash] {
                let mut nd = self.next_of(pred);
                while let Some(i) = nd {
                    let (h, k, _, next) = self.nodes[i];
                    if !(h == hash || Self::constrain(h, bc) == chash) {
                        break;
                    }
                    if h == hash && k == key {
                        return &mut self.nodes[i].2;
                    }
                    nd = next;
                }
            }
        }
        let size = self.nodes.len();
        if (size + 1) as f32 > bc as f32 * 1.0f32 || bc == 0 {
            let is_power2 = bc > 2 && (bc & (bc - 1)) == 0;
            let want = (2 * bc + usize::from(!is_power2))
                .max(((size + 1) as f32 / 1.0f32).ceil() as usize);
            self.rehash(want);
            bc = self.buckets.len();
            chash = Self::constrain(hash, bc);
        }
        let new_idx = self.nodes.len();
        self.nodes.push((hash, key, (0, false), None));
        match self.buckets[chash] {
            None => {
                // Empty bucket: to the front of the list.
                self.nodes[new_idx].3 = self.head;
                self.head = Some(new_idx);
                self.buckets[chash] = Some(None);
                if let Some(next) = self.nodes[new_idx].3 {
                    let next_bucket = Self::constrain(self.nodes[next].0, bc);
                    self.buckets[next_bucket] = Some(Some(new_idx));
                }
            }
            Some(pred) => {
                let after = self.next_of(pred);
                self.nodes[new_idx].3 = after;
                self.set_next(pred, Some(new_idx));
            }
        }
        &mut self.nodes[new_idx].2
    }

    fn in_order(&self) -> Vec<(Point, (usize, bool))> {
        let mut out = Vec::with_capacity(self.nodes.len());
        let mut cur = self.head;
        while let Some(i) = cur {
            out.push((self.nodes[i].1, self.nodes[i].2));
            cur = self.nodes[i].3;
        }
        out
    }
}

/// Flatten `ExPolygons` into `Polygons` (contour followed by holes), matching
/// `to_polygons(const ExPolygons&)` in ClipperUtils.
fn expolygons_to_polygons(ex: &[ExPolygon]) -> crate::geometry::Polygons {
    let mut out: crate::geometry::Polygons = Vec::new();
    for e in ex.iter() {
        out.push(e.contour.clone());
        for h in e.holes.iter() {
            out.push(h.clone());
        }
    }
    out
}

/// Apply fuzzy skin to an extrusion entity by perturbing its polyline points.
fn apply_fuzzy_skin_to_entity(
    entity: &mut crate::extrusion_entity::ExtrusionEntityType,
    config: &crate::fuzzy_skin::FuzzySkinConfig,
    mode: crate::region_config::FuzzySkinMode,
) {
    use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};

    match entity {
        ExtrusionEntityType::Path(path) => {
            let should_apply = match mode {
                crate::region_config::FuzzySkinMode::All => true,
                crate::region_config::FuzzySkinMode::External => {
                    path.role == ExtrusionRole::ExternalPerimeter
                }
                _ => false,
            };
            if should_apply && path.polyline.points.len() >= 3 {
                // Create a polygon from the polyline for fuzzy skin processing
                let polygon = crate::geometry::Polygon::from_points(path.polyline.points.clone());
                let fuzzied = crate::fuzzy_skin::apply_fuzzy_skin_polygon_adapter(
                    &polygon, config, 1, 0,
                    true, // layer_idx=1 to enable fuzzy (0 is skipped for adhesion)
                );
                path.polyline.points = fuzzied.points().to_vec();
            }
        }
        ExtrusionEntityType::Loop(loop_entity) => {
            for lpath in &mut loop_entity.paths {
                let should_apply = match mode {
                    crate::region_config::FuzzySkinMode::All => true,
                    crate::region_config::FuzzySkinMode::External => {
                        lpath.role == ExtrusionRole::ExternalPerimeter
                    }
                    _ => false,
                };
                if should_apply && lpath.polyline.points.len() >= 3 {
                    let polygon =
                        crate::geometry::Polygon::from_points(lpath.polyline.points.clone());
                    let fuzzied = crate::fuzzy_skin::apply_fuzzy_skin_polygon_adapter(
                        &polygon, config, 1, 0,
                        true, // layer_idx=1 to enable fuzzy (0 is skipped for adhesion)
                    );
                    lpath.polyline.points = fuzzied.points().to_vec();
                }
            }
        }
        // An Arachne extrusion gets its fuzzy skin before it becomes paths
        // (PerimeterGenerator.cpp:661-662); nothing to do on the multi-path.
        ExtrusionEntityType::MultiPath(_) => {}
        ExtrusionEntityType::Collection(coll) => {
            for sub_entity in &mut coll.entities {
                apply_fuzzy_skin_to_entity(sub_entity, config, mode);
            }
        }
    }
}

/// R553: contour/hole census of `last` / `last_p` where `PerimeterGenerator`
/// hands the region to Arachne, mirroring `[CPP-LASTPROBE]`. Contours and holes
/// are counted separately because a union that merges contours preserves area
/// while reducing the count -- the signature R552 observed.
fn lastprobe(stage: &str, contours: usize, holes: usize, points: usize) {
    use std::collections::BTreeMap;
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    use std::sync::Mutex;
    #[derive(Default, Clone, Copy)]
    struct Acc {
        calls: usize,
        contours: usize,
        holes: usize,
        points: usize,
    }
    static ACC: Mutex<Option<BTreeMap<String, Acc>>> = Mutex::new(None);
    static ROUNDS: AtomicUsize = AtomicUsize::new(0);
    let Ok(mut g) = ACC.lock() else { return };
    let m = g.get_or_insert_with(BTreeMap::new);
    let a = m.entry(stage.to_string()).or_default();
    a.calls += 1;
    a.contours += contours;
    a.holes += holes;
    a.points += points;
    if stage.starts_with('D') && ROUNDS.fetch_add(1, Relaxed) % 4_000 == 3_999 {
        eprintln!("[LASTPROBE] ---- cumulative ----");
        for (k, v) in m.iter() {
            eprintln!(
                "  {k:<26} calls={:7} contours={:9} holes={:9} points={:10}",
                v.calls, v.contours, v.holes, v.points
            );
        }
    }
}


/// PerimeterGenerator.cpp:527-608 — `static void smooth_overhang_level(ExtrusionPaths&)`.
///
/// For every run of consecutive wall pieces (Perimeter / ExternalPerimeter) that
/// share an integer overhang degree and are together shorter than 0.8 mm, replace
/// the run's degree by the length-weighted average degree over a 6.5 mm window
/// (the run plus its neighbours on both sides, wrapping around the loop, stopping
/// at an OverhangPerimeter piece). Degrees are read and written as C++ `int`
/// (truncation), exactly like `get_overhang_degree()` / `set_overhang_degree(int)`.
pub fn smooth_overhang_level(paths: &mut [ExtrusionPath]) {
    let threshold_length: f64 = crate::scaled_f(0.8);
    let filter_range: f64 = crate::scaled_f(6.5);

    // 0. save old overhang series first which is input of filter
    let path_num = paths.len() as i64;
    if path_num < 2 {
        // don't need to do filtering if only has one path in vector
        return;
    }
    // PerimeterGenerator.cpp:526 `paths[i].get_overhang_degree()`: 0 for a role that is neither a
    // perimeter nor a support (the paths of the live path are perimeters).
    let old: Vec<i32> = paths.iter().map(|p| p.get_overhang_degree()).collect();
    let is_wall = |r: ExtrusionRole| {
        r == ExtrusionRole::Perimeter || r == ExtrusionRole::ExternalPerimeter
    };

    let mut i: i64 = 0;
    while i < path_num {
        if !is_wall(paths[i as usize].role) {
            i += 1;
            continue;
        }
        let current_length = paths[i as usize].length();
        let current_overhang_degree = old[i as usize];
        let mut total_lens = current_length;
        let mut pt = i + 1;
        while pt < path_num {
            let p = &paths[pt as usize];
            if p.get_overhang_degree() != current_overhang_degree || !is_wall(p.role) {
                break;
            }
            total_lens += p.length();
            pt += 1;
        }

        if total_lens < threshold_length {
            let mut left_total_length = (filter_range - total_lens) / 2.0;
            let mut right_total_length = left_total_length;
            let mut neighbor_path: Vec<(f64, i32)> = Vec::new();

            let mut j = i - 1;
            let mut guard = 0i64;
            while left_total_length > 0.0 {
                let index = if j < 0 { path_num - 1 } else { j };
                if paths[index as usize].role == ExtrusionRole::OverhangPerimeter {
                    break;
                }
                let temp_length = paths[index as usize].length();
                if temp_length > left_total_length {
                    neighbor_path.push((left_total_length, old[index as usize]));
                } else {
                    neighbor_path.push((temp_length, old[index as usize]));
                }
                left_total_length -= temp_length;
                j = index;
                j -= 1;
                guard += 1;
                if guard > 4 * path_num + 8 {
                    break; // all-zero-length loop: native would spin here
                }
            }

            j = pt;
            guard = 0;
            while right_total_length > 0.0 {
                let index = j % path_num;
                if paths[index as usize].role == ExtrusionRole::OverhangPerimeter {
                    break;
                }
                let temp_length = paths[index as usize].length();
                if temp_length > right_total_length {
                    neighbor_path.push((right_total_length, old[index as usize]));
                } else {
                    neighbor_path.push((temp_length, old[index as usize]));
                }
                right_total_length -= temp_length;
                j += 1;
                guard += 1;
                if guard > 4 * path_num + 8 {
                    break;
                }
            }

            // PG_SMOOTH_FMA (on unless =0): PerimeterGenerator.cpp:583 `sum += (it->first * it->second)`
            // and :587 `(total_lens * current_overhang_degree + sum)` are each one expression, which
            // the pin's compiler contracts to a fused multiply-add. =0 rounds the products first.
            let fma = crate::faithful_gate("PG_SMOOTH_FMA");
            let mut sum = 0.0_f64;
            let mut length_sum = 0.0_f64;
            for (l, d) in &neighbor_path {
                if fma {
                    sum = l.mul_add(*d as f64, sum);
                } else {
                    sum += l * (*d as f64);
                }
                length_sum += l;
            }
            let average_overhang = if fma {
                total_lens.mul_add(current_overhang_degree as f64, sum) / (length_sum + total_lens)
            } else {
                (total_lens * current_overhang_degree as f64 + sum) / (length_sum + total_lens)
            };
            for idx in i..pt {
                // set_overhang_degree(int): clamp to [0, 10] on perimeter roles.
                paths[idx as usize].set_overhang_degree(average_overhang as i32);
            }
        }

        i = pt;
    }
}

thread_local! {
    /// R803 OVHD — per-sample overhang distance probe flag, read by
    /// overhang_detector::detect_overhang_degree_arachne.
    pub(crate) static OVHD_ON: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

// ---------------------------------------------------------------------------------------------
// R919: functions of PerimeterGenerator.cpp under their own names, line by line. The live path above
// carries the same steps inline (it has no config back-pointers); nothing live calls these yet. The
// members of the C++ `PerimeterGenerator` a method reads (`print_config`, `config`, `lower_slices`,
// `multi_nozzle_diameter_mode`) are its leading arguments.
// Native parity: tests/perimeter_generator_native_parity.rs.
// ---------------------------------------------------------------------------------------------

/// libslic3r.h:58 `static constexpr double SCALING_FACTOR = 0.00001;`: `scale_(v)` divides by it. (The
/// `SCALING_FACTOR` of the crate root, which the live path above uses, is its inverse.)
const SCALING_FACTOR_PIN: f64 = crate::libslic3r::SCALING_FACTOR;

fn pg_cfg_int(config: &crate::print_config::config_def::DynamicPrintConfig, key: &str) -> i32 {
    use crate::config::ConfigOptionResolver;
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or_else(|| panic!("option {key}"))
}

/// PerimeterGenerator.cpp:147-230 `static void lowpass_filter_by_paths_overhang_degree(ExtrusionPaths& paths)`
pub fn lowpass_filter_by_paths_overhang_degree(paths: &mut Vec<ExtrusionPath>) {
    // scale_(6.5), scale_(1.2)
    let filter_range: f64 = 6.5 / SCALING_FACTOR_PIN;
    let threshold_length: f64 = 1.2 / SCALING_FACTOR_PIN;

    //0.save old overhang series first which is input of filter
    let path_num: i32 = paths.len() as i32;
    if path_num < 2 {
        //don't need to do filting if only has one path in vector
        return;
    }
    let mut old_overhang_series: Vec<i32> = Vec::with_capacity(path_num as usize);
    for i in 0..path_num {
        old_overhang_series.push(paths[i as usize].get_overhang_degree());
    }

    //1.lowpass filter
    for i in 0..path_num {
        let current_length: f64 = paths[i as usize].length();
        let current_overhang_degree: i32 = old_overhang_series[i as usize];
        if current_length < threshold_length
            && (paths[i as usize].role == ExtrusionRole::Perimeter || paths[i as usize].role == ExtrusionRole::ExternalPerimeter)
        {
            let mut left_total_length: f64 = (filter_range - current_length) / 2.;
            let mut right_total_length: f64 = left_total_length;

            let mut temp_length: f64;
            let mut j: i32 = i - 1;
            let mut index: i32;
            let mut neighbor_path: Vec<(f64, i32)> = Vec::new();
            while left_total_length > 0. {
                index = if j < 0 { path_num - 1 } else { j };
                if paths[index as usize].role == ExtrusionRole::OverhangPerimeter {
                    break;
                }
                temp_length = paths[index as usize].length();
                if temp_length > left_total_length {
                    neighbor_path.push((left_total_length, old_overhang_series[index as usize]));
                } else {
                    neighbor_path.push((temp_length, old_overhang_series[index as usize]));
                }
                left_total_length -= temp_length;
                j = index;
                j -= 1;
            }

            j = i + 1;
            while right_total_length > 0. {
                index = j % path_num;
                if paths[index as usize].role == ExtrusionRole::OverhangPerimeter {
                    break;
                }
                temp_length = paths[index as usize].length();
                if temp_length > right_total_length {
                    neighbor_path.push((right_total_length, old_overhang_series[index as usize]));
                } else {
                    neighbor_path.push((temp_length, old_overhang_series[index as usize]));
                }
                right_total_length -= temp_length;
                j += 1;
            }

            let mut sum: f64 = 0.;
            let mut length_sum: f64 = 0.;
            for it in &neighbor_path {
                // sum += (it->first * it->second): one expression, fused
                sum = it.0.mul_add(it.1 as f64, sum);
                length_sum += it.0;
            }

            // (double)(current_length * current_overhang_degree + sum) / (length_sum + current_length)
            let average_overhang: f64 = current_length.mul_add(current_overhang_degree as f64, sum) / (length_sum + current_length);
            paths[i as usize].set_overhang_degree(average_overhang as i32);
        }
    }

    //2.merge path if have same overhang degree. from back to front to avoid data copy
    let mut last_overhang: i32 = paths[0].get_overhang_degree();
    let mut it: usize = 1;
    while it != paths.len() {
        if last_overhang == paths[it].get_overhang_degree() {
            //BBS: don't need to append duplicated points, remove the last point
            if paths[it - 1].polyline.last_point() == paths[it].polyline.first_point() {
                paths[it - 1].polyline.points.pop();
            }
            let src = paths.remove(it);
            paths[it - 1].polyline.append(&src.polyline);
        } else {
            last_overhang = paths[it].get_overhang_degree();
            it += 1;
        }
    }
}

/// PerimeterGenerator.cpp:272-280 `static bool is_enable_overhang_speed(const PerimeterGenerator&
/// perimeter_generator)`. `print` is the Print of `perimeter_generator.print_config`, `config` is
/// `perimeter_generator.config`.
pub fn is_enable_overhang_speed(print: &crate::print::faithful::Print, config: &crate::print_config::config_def::DynamicPrintConfig) -> bool {
    use crate::config::{ConfigOptionBools, ConfigOptionBoolsNullable, ConfigOptionResolver};
    // ConfigOptionVector::get_at: the first value past the end.
    fn bools_get_at(config: &crate::print_config::config_def::DynamicPrintConfig, key: &str, i: i32) -> bool {
        let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
        let values: Vec<bool> = if let Some(o) = opt.as_any().downcast_ref::<ConfigOptionBools>() {
            o.values.iter().map(|v| *v != 0).collect()
        } else if let Some(o) = opt.as_any().downcast_ref::<ConfigOptionBoolsNullable>() {
            o.values.iter().map(|v| *v != 0).collect()
        } else {
            panic!("option {key} is not a vector of booleans")
        };
        if i >= 0 && (i as usize) < values.len() {
            values[i as usize]
        } else {
            values[0]
        }
    }
    let filament_idx: i32 = pg_cfg_int(config, "wall_filament") - 1;
    let config_idx: i32 = print.get_process_config_idx(filament_idx as u32) as i32;
    let use_filament_overhang_speed: bool = bools_get_at(print.config(), "override_process_overhang_speed", filament_idx);

    if use_filament_overhang_speed {
        bools_get_at(print.config(), "filament_enable_overhang_speed", filament_idx)
    } else {
        bools_get_at(config, "enable_overhang_speed", config_idx)
    }
}

/// PerimeterGenerator.cpp:282-286 `static bool fuzzy_skin_allows_overhang_slowdown(const PerimeterGenerator
/// &pg)`. `config` is `pg.config`, `perimeter_regions_empty` is `pg.perimeter_regions->empty()`.
pub fn fuzzy_skin_allows_overhang_slowdown(config: &crate::print_config::config_def::DynamicPrintConfig, perimeter_regions_empty: bool) -> bool {
    // PrintConfig.hpp `enum class FuzzySkinType { None, External, All, AllWalls, Disabled_fuzzy }`
    const FUZZY_SKIN_TYPE_NONE: i32 = 0;
    const FUZZY_SKIN_TYPE_DISABLED_FUZZY: i32 = 4;
    let fs: i32 = pg_cfg_int(config, "fuzzy_skin");
    fs == FUZZY_SKIN_TYPE_DISABLED_FUZZY || (fs == FUZZY_SKIN_TYPE_NONE && perimeter_regions_empty)
}

/// PerimeterGenerator.cpp:835-847 `static Polygons to_polygons_with_flag(const ExPolygon& src, const bool
/// contour_flag, const std::vector<int>& holes_flag, std::vector<int>& flags_out)`
pub fn to_polygons_with_flag(src: &ExPolygon, contour_flag: bool, holes_flag: &[i32], flags_out: &mut Vec<i32>) -> crate::geometry::Polygons {
    let mut polygons: crate::geometry::Polygons = Vec::with_capacity(src.num_contours());
    polygons.push(src.contour.clone());
    polygons.extend(src.holes.iter().cloned());
    flags_out.reserve(holes_flag.len() + 1);
    if contour_flag == true {
        flags_out.push(0);
    }
    for idx in 0..holes_flag.len() {
        flags_out.push(holes_flag[idx] + 1);
    }
    polygons
}

impl PerimeterLoop {
    /// PerimeterGenerator.cpp:58 `bool PerimeterGeneratorLoop::is_external() const { return this->depth == 0; }`:
    /// External perimeter. It may be CCW or CW oriented (outer contour or hole contour).
    pub fn is_external(&self) -> bool {
        self.perimeter_index == 0
    }
}

/// PerimeterGenerator.cpp:40-62 `class PerimeterGeneratorLoop`: the live path names it [`PerimeterLoop`].
pub type PerimeterGeneratorLoop = PerimeterLoop;

impl PerimeterGenerator {
    /// PerimeterGenerator.cpp:232-242 `std::pair<double, double> PerimeterGenerator::dist_boundary(double width)`
    pub fn dist_boundary(
        print_config: &crate::print_config::config_def::DynamicPrintConfig,
        config: &crate::print_config::config_def::DynamicPrintConfig,
        width: f64,
    ) -> (f64, f64) {
        let nozzle_diameter: f32 = crate::print::faithful::nozzle_diameter_for_filament(print_config, pg_cfg_int(config, "wall_filament") as u32);
        let start_offset: f32 = (-0.5 * width) as f32;
        let end_offset: f32 = (0.5 * nozzle_diameter as f64) as f32;
        // scale_(start_offset + 0.5 * (end_offset - start_offset) / (overhang_sampling_number - 1))
        let degree_0: f64 = (start_offset as f64
            + 0.5 * (end_offset - start_offset) as f64 / (crate::overhang_detector::OVERHANG_SAMPLING_NUMBER - 1) as f64)
            / SCALING_FACTOR_PIN;
        (0., end_offset as f64 / SCALING_FACTOR_PIN - degree_0)
    }

    /// PerimeterGenerator.cpp:849-857 `int PerimeterGenerator::effective_wall_loops() const`
    pub fn effective_wall_loops(
        print_config: &crate::print_config::config_def::DynamicPrintConfig,
        config: &crate::print_config::config_def::DynamicPrintConfig,
        multi_nozzle_diameter_mode: bool,
    ) -> i32 {
        let preset_value: i32 = pg_cfg_int(config, "wall_loops");
        if !multi_nozzle_diameter_mode {
            return preset_value;
        }
        let nozzle_diameter: f64 =
            crate::print::faithful::nozzle_diameter_for_filament(print_config, pg_cfg_int(config, "wall_filament") as u32) as f64;
        crate::print_config::config_def::multi_nozzle_wall_loops(nozzle_diameter, preset_value)
    }

    /// PerimeterGenerator.cpp:1886-1909 `std::vector<Polygons> PerimeterGenerator::generate_lower_polygons_series(float width)`
    pub fn generate_lower_polygons_series(
        print_config: &crate::print_config::config_def::DynamicPrintConfig,
        config: &crate::print_config::config_def::DynamicPrintConfig,
        lower_slices: Option<&ExPolygons>,
        width: f32,
    ) -> Vec<crate::geometry::Polygons> {
        use crate::clipper as cl;
        use crate::clipper_utils as cu;
        let nozzle_diameter: f32 = crate::print::faithful::nozzle_diameter_for_filament(print_config, pg_cfg_int(config, "wall_filament") as u32);
        let start_offset: f32 = (-0.5 * width as f64) as f32;
        let end_offset: f32 = (0.5 * nozzle_diameter as f64) as f32;

        debug_assert!(crate::overhang_detector::OVERHANG_SAMPLING_NUMBER >= 3);
        // generate offsets
        let mut offset_series: Vec<f32> = Vec::with_capacity(2);

        offset_series.push(
            (start_offset as f64 + 0.5 * (end_offset - start_offset) as f64 / (crate::overhang_detector::OVERHANG_SAMPLING_NUMBER - 1) as f64) as f32,
        );
        offset_series.push(end_offset);
        let mut lower_polygons_series: Vec<crate::geometry::Polygons> = Vec::new();
        let Some(lower_slices) = lower_slices else {
            return lower_polygons_series;
        };

        // offset expolygon to generate series of polygons
        for i in 0..offset_series.len() {
            // offset(*this->lower_slices, float(scale_(offset_series[i]))): ClipperUtils.cpp
            // `offset(const ExPolygons &, const float delta)` = to_polygons(expolygons_offset(...))
            let delta: f32 = (offset_series[i] as f64 / SCALING_FACTOR_PIN) as f32;
            let rings: Vec<(cl::Path, cl::Paths)> = lower_slices
                .iter()
                .map(|e| {
                    let path = |pts: &[Point]| -> cl::Path { pts.iter().map(|p| cl::IntPoint::new(p.x as i32, p.y as i32)).collect() };
                    (path(&e.contour.points), e.holes.iter().map(|h| path(&h.points)).collect())
                })
                .collect();
            lower_polygons_series.push(cu::clib_polygons_from_paths(&cu::expolygons_offset(&rings, delta as f64, cl::JoinType::jtMiter, 3.)));
        }
        lower_polygons_series
    }
}

/// PerimeterGenerator.hpp:15-28 `struct PerimeterRegion`
#[derive(Clone)]
pub struct PerimeterRegion {
    /// `const PrintRegion *region`
    pub region: crate::print::faithful::PrintRegionPtr,
    pub expolygons: ExPolygons,
    pub bbox: crate::geometry::BoundingBox,
}

/// PerimeterGenerator.hpp:30 `using PerimeterRegions = std::vector<PerimeterRegion>`
pub type PerimeterRegions = Vec<PerimeterRegion>;

impl PerimeterRegion {
    /// PerimeterGenerator.cpp:1911-1915 `PerimeterRegion::PerimeterRegion(const LayerRegion &layer_region)`.
    /// `region` is `&layer_region.region()`.
    pub fn new(region: crate::print::faithful::PrintRegionPtr, layer_region: &crate::layer::LayerRegion) -> Self {
        let expolygons: ExPolygons = layer_region.slices.surfaces.iter().map(|s| s.expolygon.clone()).collect();
        // ExPolygon.cpp:495-504 `get_extents(const ExPolygons &expolygons)`: the boxes of the contours
        // merged. The box of a contour (BoundingBox.hpp:88-102, `construct` without the boundary) is
        // defined only when it has an area, and `merge` leaves out one that is not defined.
        let mut bbox = crate::geometry::BoundingBox::new();
        for e in &expolygons {
            if !e.contour.points.is_empty() {
                let mut contour_bbox = crate::geometry::BoundingBox::new();
                crate::geometry::BoundingBox::construct(&mut contour_bbox, &e.contour.points, false);
                bbox.merge(&contour_bbox);
            }
        }
        PerimeterRegion { region, expolygons, bbox }
    }

    /// PerimeterGenerator.cpp:1917-1927 `bool PerimeterRegion::has_compatible_perimeter_regions(const
    /// PrintRegionConfig &config, const PrintRegionConfig &other_config)`.
    ///
    /// If there is any incompatibility, we don't need to create separate LayerRegions.
    /// Because it is enough to split perimeters by PerimeterRegions.
    pub fn has_compatible_perimeter_regions(
        config: &crate::print_config::config_def::DynamicPrintConfig,
        other_config: &crate::print_config::config_def::DynamicPrintConfig,
    ) -> bool {
        use crate::config::ConfigOptionResolver;
        let eq = |key: &str| -> bool {
            let l = config.option(key).unwrap_or_else(|| panic!("option {key}"));
            let r = other_config.option(key).unwrap_or_else(|| panic!("option {key}"));
            l.operator_eq(r).unwrap_or(false)
        };
        eq("fuzzy_skin")
            && eq("fuzzy_skin_thickness")
            && eq("fuzzy_skin_point_distance")
            && eq("fuzzy_skin_first_layer")
            && eq("fuzzy_skin_noise_type")
            && eq("fuzzy_skin_scale")
            && eq("fuzzy_skin_octaves")
            && eq("fuzzy_skin_persistence")
            && eq("fuzzy_skin_mode")
    }

    /// PerimeterGenerator.cpp:1929-1951 `void PerimeterRegion::merge_compatible_perimeter_regions(PerimeterRegions
    /// &perimeter_regions)`
    pub fn merge_compatible_perimeter_regions(perimeter_regions: &mut PerimeterRegions) {
        use crate::clipper as cl;
        use crate::clipper_utils as cu;
        if perimeter_regions.len() <= 1 {
            return;
        }

        let mut perimeter_regions_merged: PerimeterRegions = Vec::new();
        let mut it_curr_region: usize = 0;
        while it_curr_region != perimeter_regions.len() {
            let mut current_merge: PerimeterRegion = perimeter_regions[it_curr_region].clone();
            let mut it_next_region: usize = it_curr_region + 1;
            while it_next_region != perimeter_regions.len()
                && Self::has_compatible_perimeter_regions(
                    perimeter_regions[it_next_region].region.borrow().config(),
                    perimeter_regions[it_curr_region].region.borrow().config(),
                )
            {
                let moved = std::mem::take(&mut perimeter_regions[it_next_region].expolygons);
                current_merge.expolygons.extend(moved);
                current_merge.bbox.merge(&perimeter_regions[it_next_region].bbox);
                it_next_region += 1;
            }

            if it_next_region - it_curr_region > 1 {
                // union_ex(const ExPolygons &subject)
                let mut subject = cl::Paths::new();
                for e in &current_merge.expolygons {
                    let path = |pts: &[Point]| -> cl::Path { pts.iter().map(|p| cl::IntPoint::new(p.x as i32, p.y as i32)).collect() };
                    subject.push(path(&e.contour.points));
                    for h in &e.holes {
                        subject.push(path(&h.points));
                    }
                }
                current_merge.expolygons = cu::poly_tree_to_ex_polygons(&cu::clipper_do_polytree(
                    cl::ClipType::ctUnion,
                    &subject,
                    &cl::Paths::new(),
                    cl::PolyFillType::pftNonZero,
                ));
            }

            perimeter_regions_merged.push(current_merge);
            it_curr_region = it_next_region;
        }

        *perimeter_regions = perimeter_regions_merged;
    }
}

/// PerimeterGenerator.cpp:597-619 `static void detect_brigde_wall_arachne(const PerimeterGenerator
/// &perimeter_generator, ExtrusionPaths &paths, const ClipperLib_Z::Paths &path_overhang, const ExtrusionRole
/// role, const Flow &flow)`. The C++ does not read `perimeter_generator`; it is left out. A path of
/// `path_overhang` has at least two points (the C++ reads the second one unchecked).
pub fn detect_brigde_wall_arachne(paths: &mut Vec<ExtrusionPath>, path_overhang: &crate::clipper_z_utils::ZPaths, role: ExtrusionRole, flow: &Flow) {
    use crate::arachne::utils::extrusion_line::{extrusion_path_append, to_thick_polyline_z};
    for path in path_overhang {
        // check if the line is straight line, which mean if the wall is bridge
        let thick_polyline = to_thick_polyline_z(path);

        let line = crate::geometry::Line::new(thick_polyline.points[0], thick_polyline.points[thick_polyline.points.len() - 1]);
        if line.length() < thick_polyline.length() {
            extrusion_path_append(paths, &thick_polyline, role, flow, (crate::overhang_detector::OVERHANG_SAMPLING_NUMBER - 1) as f64);
            continue;
        }

        extrusion_path_append(paths, &thick_polyline, role, flow, crate::overhang_detector::OVERHANG_SAMPLING_NUMBER as f64);
    }
}

fn pg_cfg_bool(config: &crate::print_config::config_def::DynamicPrintConfig, key: &str) -> bool {
    use crate::config::ConfigOptionResolver;
    config.option(key).and_then(|o| o.get_bool().ok()).unwrap_or_else(|| panic!("option {key}"))
}

/// PerimeterGenerator.cpp:504-511 `struct PerimeterGeneratorArachneExtrusion`
pub struct PerimeterGeneratorArachneExtrusion<'a> {
    /// `Arachne::ExtrusionLine* extrusion = nullptr;`
    pub extrusion: &'a mut ArachneExtrusionLine,
    /// Indicates if closed ExtrusionLine is a contour or a hole. Used it only when ExtrusionLine is a closed loop.
    pub is_contour: bool,
    /// Should this extrusion be fuzzyfied on path generation?
    pub fuzzify: bool,
}

/// The members of the C++ `PerimeterGenerator` (PerimeterGenerator.hpp:32-134) that `traverse_extrusions`
/// reads through its `perimeter_generator`, the options of its configs as the values the function reads
/// of them. `loop_nodes`, which it writes, is an argument of its own.
pub struct PerimeterGeneratorArachneMembers {
    /// `print_config->z_direction_outwall_speed_continuous` (PerimeterGenerator.cpp:625, 640)
    pub z_direction_outwall_speed_continuous: bool,
    /// `nozzle_diameter_for_filament(*print_config, config->wall_filament)` (PerimeterGenerator.cpp:650, 673)
    pub wall_nozzle_diameter: f32,
    /// `config->outer_wall_line_width` (PerimeterGenerator.cpp:652)
    pub outer_wall_line_width: f64,
    /// `config->detect_overhang_wall` (PerimeterGenerator.cpp:668)
    pub detect_overhang_wall: bool,
    /// `object_config->raft_layers` (PerimeterGenerator.cpp:668)
    pub raft_layers: i32,
    /// `is_enable_overhang_speed(perimeter_generator) && fuzzy_skin_allows_overhang_slowdown(perimeter_generator)`
    /// (PerimeterGenerator.cpp:268-283, 702)
    pub overhang_speed: bool,
    pub layer_id: i32,
    pub starts_at_object_base: bool,
    pub slice_z: f64,
    pub perimeter_flow: Flow,
    pub ext_perimeter_flow: Flow,
    pub overhang_flow: Flow,
    pub preset_nozzle_diameter: f32,
    pub multi_nozzle_diameter_mode: bool,
    /// `lower_slices_polygons()`, the member process_arachne sets.
    pub lower_slices_polygons: Vec<Polygon>,
}

impl PerimeterGeneratorArachneMembers {
    /// The values of the options out of the configs of the pin's generator: `print.config()` is its
    /// `print_config`, `config` its PrintRegionConfig, `object_config` its PrintObjectConfig,
    /// `perimeter_regions_empty` is `perimeter_regions->empty()`.
    #[allow(clippy::too_many_arguments)]
    pub fn from_configs(
        print: &crate::print::faithful::Print,
        config: &crate::print_config::config_def::DynamicPrintConfig,
        object_config: &crate::print_config::config_def::DynamicPrintConfig,
        perimeter_regions_empty: bool,
        layer_id: i32,
        starts_at_object_base: bool,
        slice_z: f64,
        perimeter_flow: Flow,
        ext_perimeter_flow: Flow,
        overhang_flow: Flow,
        preset_nozzle_diameter: f32,
        multi_nozzle_diameter_mode: bool,
        lower_slices_polygons: Vec<Polygon>,
    ) -> Self {
        use crate::config::{ConfigOptionFloat, ConfigOptionResolverTyped};
        let print_config = print.config();
        Self {
            z_direction_outwall_speed_continuous: pg_cfg_bool(print_config, "z_direction_outwall_speed_continuous"),
            wall_nozzle_diameter: crate::print::faithful::nozzle_diameter_for_filament(print_config, pg_cfg_int(config, "wall_filament") as u32),
            outer_wall_line_width: config.option_typed::<ConfigOptionFloat>("outer_wall_line_width").unwrap_or_else(|| panic!("option outer_wall_line_width")).value,
            detect_overhang_wall: pg_cfg_bool(config, "detect_overhang_wall"),
            raft_layers: pg_cfg_int(object_config, "raft_layers"),
            overhang_speed: is_enable_overhang_speed(print, config) && fuzzy_skin_allows_overhang_slowdown(config, perimeter_regions_empty),
            layer_id,
            starts_at_object_base,
            slice_z,
            perimeter_flow,
            ext_perimeter_flow,
            overhang_flow,
            preset_nozzle_diameter,
            multi_nozzle_diameter_mode,
            lower_slices_polygons,
        }
    }
}

/// PerimeterGenerator.cpp:621-832 `static ExtrusionEntityCollection traverse_extrusions(const PerimeterGenerator&
/// perimeter_generator, std::vector<PerimeterGeneratorArachneExtrusion>& pg_extrusions)`.
///
/// `apply_fuzzy_skin(extrusion, layer_id, inset_idx, is_contour, slice_z)` is FuzzySkin.cpp's
/// `apply_fuzzy_skin(extrusion, *config, *perimeter_regions, layer_id, inset_idx, is_contour, slice_z)`
/// with the config and the perimeter regions bound: its port (fuzzy_skin.rs) reads the typed region
/// config of the live path, not the PrintRegionConfig this function reads.
///
/// extrusion_entity.rs' ExtrusionMultiPath has no customize flag of its own (the C++ sets
/// cfCircleCompensation on the multi path and on its paths); the paths carry it.
pub fn traverse_extrusions(
    perimeter_generator: &PerimeterGeneratorArachneMembers,
    loop_nodes: &mut Vec<crate::layer::LoopNode>,
    pg_extrusions: &mut [PerimeterGeneratorArachneExtrusion],
    apply_fuzzy_skin: &dyn Fn(&ArachneExtrusionLine, usize, usize, bool, f64) -> ArachneExtrusionLine,
) -> crate::extrusion_entity::ExtrusionEntityCollection {
    use crate::arachne::utils::extrusion_line::{extrusion_paths_append_line, extrusion_paths_append_zpaths};
    use crate::clipper_z::{clip_extrusion, ClipType};
    use crate::clipper_z_utils::{ZPath, ZPaths};
    use crate::config::ConfigOptionFloat;
    use crate::extrusion_entity::{CustomizeFlag, ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionMultiPath};

    let z_direction_outwall_speed_continuous = perimeter_generator.z_direction_outwall_speed_continuous;

    let mut extrusion_coll = ExtrusionEntityCollection::new();
    if z_direction_outwall_speed_continuous {
        extrusion_coll.loop_node_range.0 = loop_nodes.len();
    }

    for pg_extrusion in pg_extrusions.iter_mut() {
        let pg_extrusion_is_contour = pg_extrusion.is_contour;
        let extrusion: &mut ArachneExtrusionLine = &mut *pg_extrusion.extrusion;
        if extrusion.junctions.is_empty() {
            continue;
        }
        //get extrusion date
        let is_external: bool = extrusion.inset_idx == 0;
        let role: ExtrusionRole = if is_external { ExtrusionRole::ExternalPerimeter } else { ExtrusionRole::Perimeter };

        if is_external && z_direction_outwall_speed_continuous {
            let mut node_contour = crate::layer::NodeContour::default();
            node_contour.is_loop = extrusion.is_closed;
            for i in 0..extrusion.junctions.len() {
                node_contour.pts.push(extrusion.junctions[i].p);
                node_contour.widths.push(extrusion.junctions[i].w);
            }
            // get_extents(node.node_contour.pts): BoundingBox(points), defined where it has an area
            let mut bbox = crate::geometry::BoundingBox::new();
            crate::geometry::BoundingBox::construct(&mut bbox, &node_contour.pts, false);
            let nozzle_diameter: f32 = perimeter_generator.wall_nozzle_diameter;
            // node.bbox.offset(width / 2): BoundingBox.hpp `PointClass v(delta, delta); min -= v; max += v;`
            // The width is in millimetres and Point(double, double) rounds it with lrint.
            let outer_wall_line_width = ConfigOptionFloat { value: perimeter_generator.outer_wall_line_width };
            let delta: f64 = Flow::extrusion_width_for_nozzle(
                &outer_wall_line_width,
                perimeter_generator.preset_nozzle_diameter,
                nozzle_diameter,
                perimeter_generator.multi_nozzle_diameter_mode,
            ) / 2.;
            let v: Coord = delta.round_ties_even() as Coord;
            bbox.min.x -= v;
            bbox.min.y -= v;
            bbox.max.x += v;
            bbox.max.y += v;
            let mut node = crate::layer::LoopNode::new(loop_nodes.len(), extrusion_coll.entities.len(), bbox);
            node.node_contour = node_contour;
            loop_nodes.push(node);
        }

        // Apply fuzzy skin if it is enabled for at least some part of the ExtrusionLine.
        *extrusion = apply_fuzzy_skin(
            extrusion,
            perimeter_generator.layer_id as usize,
            extrusion.inset_idx,
            !extrusion.is_closed || pg_extrusion_is_contour,
            perimeter_generator.slice_z,
        );

        let mut paths: Vec<ExtrusionPath> = Vec::new();
        // detect overhanging/bridging perimeters

        if perimeter_generator.detect_overhang_wall
            && perimeter_generator.layer_id > perimeter_generator.raft_layers
            && !perimeter_generator.starts_at_object_base
        {
            // `ClipperLib_Z::Path extrusion_path` is filled and not read.
            let nozzle_diameter: f64 = perimeter_generator.wall_nozzle_diameter as f64;
            let mut lower_layer_polys: Vec<Polygon> = perimeter_generator.lower_slices_polygons.clone();

            let mut extrusion_path_bbox = crate::geometry::BoundingBox::new();
            for ej in &extrusion.junctions {
                extrusion_path_bbox.merge_point(Point::new(ej.p.x(), ej.p.y()));
            }
            // extrusion_path_bbox.inflated(max_extrusion_width+scale_(nozzle_diameter)) returns a copy
            // that is dropped: the box stays as it is.

            let mut new_lower_polys: Vec<Polygon> = Vec::new();
            for idx in 0..lower_layer_polys.len() {
                let new_poly = crate::clipper_utils::clip_clipper_polygon_with_subject_bbox(&lower_layer_polys[idx], &extrusion_path_bbox, true);
                if !new_poly.points.is_empty() {
                    new_lower_polys.push(new_poly);
                }
            }

            lower_layer_polys = new_lower_polys;

            let mut subject_path: ZPath = Vec::new();
            for ej in &extrusion.junctions {
                subject_path.push((ej.p.x(), ej.p.y(), ej.w));
            }

            let mut clip_paths: ZPaths = Vec::new();
            for poly in &lower_layer_polys {
                clip_paths.push(Vec::new());
                for p in &poly.points {
                    clip_paths.last_mut().unwrap().push((p.x(), p.y(), 0));
                }
            }

            let overhang_speed = perimeter_generator.overhang_speed;
            let flow: &Flow = if is_external { &perimeter_generator.ext_perimeter_flow } else { &perimeter_generator.perimeter_flow };
            if overhang_speed {
                // OverhangDetector.cpp:168-300 `detect_overhang_degree(flow, role, lower_polys, clip_paths,
                // extrusion_path, nozzle_diameter)`: its port hands back the pieces with their degrees,
                // which the C++ appends with `extrusion_paths_append(ret_path_list, { prev_line }, role,
                // flow, target_degree)`.
                for (zp, degree) in crate::overhang_detector::detect_overhang_degree_arachne(&lower_layer_polys, &clip_paths, &subject_path, nozzle_diameter) {
                    extrusion_paths_append_zpaths(&mut paths, &vec![zp], role, flow, degree);
                }
            } else {
                let mut temp_paths: Vec<ExtrusionPath> = Vec::new();
                let path_non_overhang: ZPaths = clip_extrusion(&subject_path, &clip_paths, ClipType::Intersection);
                // get non-overhang paths by intersecting this loop with the grown lower slices
                extrusion_paths_append_zpaths(&mut temp_paths, &path_non_overhang, role, flow, 0.);
                paths = temp_paths;
            }
            // get overhang paths by checking what parts of this loop fall
            // outside the grown lower slices (thus where the distance between
            // the loop centerline and original lower slices is >= half nozzle diameter
            // detect if the overhang perimeter is bridge
            let path_overhang: ZPaths = clip_extrusion(&subject_path, &clip_paths, ClipType::Difference);
            // R936 probe ARACHOVH=<layer_id> (diagnostic only): the junction bbox, the clipped
            // lower polygons and the overhang pieces of each wall line of that layer.
            if crate::probe_enabled("ARACHOVH")
                && std::env::var("ARACHOVH").ok().and_then(|v| v.parse::<i64>().ok()) == Some(perimeter_generator.layer_id as i64)
            {
                let ov: Vec<(usize, (i64, i64), (i64, i64))> = path_overhang
                    .iter()
                    .filter(|z| !z.is_empty())
                    .map(|z| (z.len(), (z[0].0, z[0].1), (z[z.len() - 1].0, z[z.len() - 1].1)))
                    .collect();
                eprintln!(
                    "ARACHOVH lid={} inset={} closed={} n={} first=({},{}) bbox=({},{})-({},{}) clip_polys={} clip_pts={:?} overhang={:?}",
                    perimeter_generator.layer_id, extrusion.inset_idx, extrusion.is_closed, subject_path.len(),
                    subject_path.first().map(|p| p.0).unwrap_or(0), subject_path.first().map(|p| p.1).unwrap_or(0),
                    extrusion_path_bbox.min.x, extrusion_path_bbox.min.y, extrusion_path_bbox.max.x, extrusion_path_bbox.max.y,
                    clip_paths.len(), clip_paths.iter().map(|c| c.len()).collect::<Vec<_>>(), ov
                );
            }
            // Zero-gap support must not force normal wall role/flow: unsupported walls still need
            // erOverhangPerimeter + overhang_flow. See GitHub #11462.
            detect_brigde_wall_arachne(&mut paths, &path_overhang, ExtrusionRole::OverhangPerimeter, &perimeter_generator.overhang_flow);
            // Reapply the nearest point search for starting point.
            // We allow polyline reversal because Clipper may have randomly reversed polylines during clipping.
            // Arachne sometimes creates extrusion with zero-length (just two same endpoints);
            if !paths.is_empty() {
                let mut start_point: Point = paths[0].polyline.first_point();
                if !extrusion.is_closed {
                    // Especially for open extrusion, we need to select a starting point that is at the start
                    // or the end of the extrusions to make one continuous line. Also, we prefer a non-overhang
                    // starting point.
                    // `std::unordered_map<Point, PointInfo, PointHash> point_occurrence`, walked in the
                    // order of libc++.
                    let mut point_occurrence = LibcxxUnorderedPointMap::new();
                    for path in &paths {
                        point_occurrence.entry(path.polyline.first_point()).0 += 1;
                        point_occurrence.entry(path.polyline.last_point()).0 += 1;
                        if path.role == ExtrusionRole::OverhangPerimeter {
                            point_occurrence.entry(path.polyline.first_point()).1 = true;
                            point_occurrence.entry(path.polyline.last_point()).1 = true;
                        }
                    }

                    // Prefer non-overhang point as a starting point.
                    for (pt, (occurrence, is_overhang)) in point_occurrence.in_order() {
                        if occurrence == 1 {
                            start_point = pt;
                            if !is_overhang {
                                start_point = pt;
                                break;
                            }
                        }
                    }
                }

                crate::shortest_path::chain_and_reorder_extrusion_paths(&mut paths, Some(&start_point));

                if overhang_speed {
                    // BBS: filter the speed
                    smooth_overhang_level(&mut paths);
                }
            }
        } else {
            let flow: &Flow = if is_external { &perimeter_generator.ext_perimeter_flow } else { &perimeter_generator.perimeter_flow };
            extrusion_paths_append_line(&mut paths, extrusion, role, flow, 0.);
        }
        // R939 TINYPROBE=<max length> (diagnostic only): short Arachne lines of layers 0-2 and the
        // paths they became (worker B's R938 finding: tiny inner-wall pieces the pin prints).
        if crate::probe_enabled("TINYPROBE") && perimeter_generator.layer_id <= 2 {
            let mut len = 0.0_f64;
            for w in extrusion.junctions.windows(2) {
                len += ((w[1].p.x - w[0].p.x) as f64).hypot((w[1].p.y - w[0].p.y) as f64);
            }
            if len < std::env::var("TINYPROBE").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(3000.0) {
                let js: Vec<String> = extrusion.junctions.iter().map(|j| format!("{},{},{}", j.p.x, j.p.y, j.w)).collect();
                let ps: Vec<String> = paths.iter().map(|p| format!("n={} w={:.5} len={:.1}", p.polyline.points.len(), p.width, p.length())).collect();
                eprintln!("TINYPROBE lid={} inset={} closed={} odd={} len={:.1} j=[{}] paths=[{}]", perimeter_generator.layer_id, extrusion.inset_idx, extrusion.is_closed, extrusion.is_odd, len, js.join(" "), ps.join("; "));
            }
        }

        // `shouldApplyHoleCompensation(const double threshold = 0.8)`
        let apply_hole_compensation: bool = extrusion.should_apply_hole_compensation(0.8);

        // Append paths to collection.
        if !paths.is_empty() {
            if extrusion.is_closed {
                let mut extrusion_loop =
                    ExtrusionLoop::new(paths, if extrusion.is_contour() { ExtrusionLoopRole::DEFAULT } else { ExtrusionLoopRole::PERIMETER_HOLE });
                // Restore the orientation of the extrusion loop.
                if pg_extrusion_is_contour {
                    extrusion_loop.make_counter_clockwise();
                } else {
                    extrusion_loop.make_clockwise();
                }

                if apply_hole_compensation {
                    for path in &mut extrusion_loop.paths {
                        path.set_customize_flag(CustomizeFlag::CircleCompensation);
                    }
                    extrusion_loop.set_customize_flag(CustomizeFlag::CircleCompensation);
                }
                extrusion_coll.append(ExtrusionEntityType::Loop(extrusion_loop));
            } else {
                // Because we are processing one ExtrusionLine all ExtrusionPaths should form one connected path.
                // But there is possibility that due to numerical issue there is poss
                let mut multi_path = ExtrusionMultiPath::new();
                let mut it_path = paths.into_iter();
                multi_path.paths.push(it_path.next().unwrap());

                for path in it_path {
                    if multi_path.paths.last().unwrap().polyline.last_point() != path.polyline.first_point() {
                        extrusion_coll.append(ExtrusionEntityType::MultiPath(std::mem::replace(&mut multi_path, ExtrusionMultiPath::new())));
                    }
                    multi_path.paths.push(path);
                }

                if apply_hole_compensation {
                    for path in &mut multi_path.paths {
                        path.set_customize_flag(CustomizeFlag::CircleCompensation);
                    }
                }
                extrusion_coll.append(ExtrusionEntityType::MultiPath(multi_path));
            }
        }
    }
    if z_direction_outwall_speed_continuous {
        extrusion_coll.loop_node_range.1 = loop_nodes.len();
    }

    extrusion_coll
}

// ---------------------------------------------------------------------------------------------
// PerimeterGenerator::process_classic as the pin has it (R921)
// ---------------------------------------------------------------------------------------------

/// The ClipperUtils.cpp overloads process_classic calls, with the default join type and miter limit
/// (ClipperUtils.hpp:31, 37). `delta` is the `const float` of the C++.
mod pgcu {
    use crate::clipper::{ClipType, PolyFillType};
    use crate::clipper_utils as u;
    use crate::clipper_utils::ApplySafetyOffset;
    use crate::geometry::{ExPolygon, ExPolygons, Polygon, Polygons};
    use crate::support::support_common::{cu, cu_expolygon};

    /// ClipperUtils.cpp:813 `union_ex(const Polygons &subject, pftNonZero)`
    pub fn union_ex(subject: &[Polygon]) -> ExPolygons {
        cu::union_ex(subject)
    }
    /// ClipperUtils.cpp:825-831 `union_ex(const ExPolygons &poly1, const ExPolygons &poly2, bool = false)`
    pub fn union_ex_2(poly1: &[ExPolygon], poly2: &[ExPolygon]) -> ExPolygons {
        cu_expolygon::union_ex_2(poly1, poly2)
    }
    /// ClipperUtils.cpp:581-584 `offset2_ex(const ExPolygons &, float delta1, float delta2)`
    pub fn offset2_ex(expolygons: &[ExPolygon], delta1: f32, delta2: f32) -> ExPolygons {
        cu::offset2_ex(expolygons, delta1, delta2, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
    }
    /// ClipperUtils.hpp:428-429 `opening_ex(const ExPolygons &, float delta)`: `offset2_ex(polygons, -delta, delta)`
    pub fn opening_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
        offset2_ex(expolygons, -delta, delta)
    }
    /// ClipperUtils.cpp:561 `offset(const ExPolygons &, float delta)`
    pub fn offset(expolygons: &[ExPolygon], delta: f32) -> Polygons {
        cu::offset_expolygons(expolygons, delta, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
    }
    /// ClipperUtils.cpp:413 `offset(const Polygons &, float delta)`
    pub fn offset_polygons(polygons: &[Polygon], delta: f32) -> Polygons {
        cu::offset(polygons, delta, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
    }
    /// ClipperUtils.cpp:567-569 `offset_ex(const ExPolygon &, float delta)`
    pub fn offset_ex_expolygon(expolygon: &ExPolygon, delta: f32) -> ExPolygons {
        cu_expolygon::offset_ex(expolygon, delta, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
    }
    /// ClipperUtils.cpp:570 `offset_ex(const ExPolygons &, float delta)`
    pub fn offset_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
        cu::offset_ex_expolygons(expolygons, delta, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
    }
    /// ClipperUtils.cpp:755 `diff_ex(const ExPolygons &, const ExPolygons &, ApplySafetyOffset)`
    pub fn diff_ex(subject: &[ExPolygon], clip: &[ExPolygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
        cu::diff_ex_ee(subject, clip, do_safety_offset)
    }
    /// ClipperUtils.cpp:753 `diff_ex(const ExPolygons &, const Polygons &, ApplySafetyOffset)`
    pub fn diff_ex_ep(subject: &[ExPolygon], clip: &[Polygon], do_safety_offset: ApplySafetyOffset) -> ExPolygons {
        cu::diff_ex_ep(subject, clip, do_safety_offset)
    }
    /// ClipperUtils.cpp:741 `diff_ex(const Polygons &, const Polygons &, ApplySafetyOffset::No)`
    pub fn diff_ex_pp(subject: &[Polygon], clip: &[Polygon]) -> ExPolygons {
        cu::diff_ex(subject, clip, ApplySafetyOffset::No)
    }
    /// ClipperUtils.cpp:802 `intersection_ex(const ExPolygons &, const ExPolygons &, ApplySafetyOffset::No)`
    pub fn intersection_ex(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctIntersection, &cu::expolygons_provider(subject), &cu::expolygons_provider(clip), ApplySafetyOffset::No, PolyFillType::pftNonZero)
    }
    /// ExPolygon.cpp:231-251 `Polygons ExPolygon::simplify_p(double tolerance) const`: the rings after
    /// `MultiPoint::_douglas_peucker` (closed by their first point) through `simplify_polygons`
    /// (ClipperUtils.cpp:1026-1041, `ClipperLib::SimplifyPolygons(pftNonZero)`).
    pub fn simplify_p(expolygon: &ExPolygon, tolerance: f64) -> Polygons {
        let pp = expolygon.simplify_p_dp_rings_faithful(tolerance);
        cu::to_polygons(&crate::clipper::simplify_polygons(&cu::polygons_provider(&pp), PolyFillType::pftNonZero))
    }
}

/// libslic3r.h:81 `scale_(val)`: `val / SCALING_FACTOR`, a double.
#[inline]
fn pg_scale_(val: f64) -> f64 {
    val / SCALING_FACTOR_PIN
}

/// Point.hpp:536-541 `scaled<coord_t>(float v)`: `coord_t(v / float(SCALING_FACTOR))`, a division of floats.
#[inline]
fn pg_scaled_float(v: f32) -> Coord {
    (v / (SCALING_FACTOR_PIN as f32)) as Coord
}

impl PerimeterGenerator {
    /// PerimeterGenerator.cpp:859-1452 `void PerimeterGenerator::process_classic()`. `surfaces` is
    /// `this->slices->surfaces`; the result carries what the C++ writes: `entities` is `loops` (the
    /// collections of the islands one after the other), `gap_fill`, `infill_area` the expolygons of
    /// `fill_surfaces`, `no_overlap_area` is `fill_no_overlap`, `loop_nodes`.
    ///
    /// Not carried by this crate's inputs, with the value the port takes: the multi nozzle diameter mode
    /// is off (`effective_wall_loops()` is `wall_loops`, `Flow::extrusion_width_for_nozzle` hands the
    /// width back, Flow.cpp:100-102); a Surface has no circle compensation (`counter_circle_compensation`
    /// false, `holes_circle_compensation` empty, PerimeterGenerator.cpp:941-946); there are no perimeter
    /// regions.
    pub fn process_classic(&self, surfaces: &[crate::surface::Surface]) -> PerimeterResult {
        use crate::clipper_utils::ApplySafetyOffset;
        use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
        use crate::print_config::{TopOneWallType, WallSequence};
        use pgcu::*;

        let cfg = &self.config;
        let mut result = PerimeterResult::new();
        result.pin = true;

        // other perimeters
        self.m_mm3_per_mm.set(cfg.perimeter_flow.mm3_per_mm().unwrap_or(-1.));
        let perimeter_width: Coord = cfg.perimeter_flow.scaled_width();
        let perimeter_spacing: Coord = cfg.perimeter_flow.scaled_spacing();

        // external perimeters
        self.m_ext_mm3_per_mm.set(cfg.ext_perimeter_flow.mm3_per_mm().unwrap_or(-1.));
        let ext_perimeter_width: Coord = cfg.ext_perimeter_flow.scaled_width();
        let ext_perimeter_spacing: Coord = cfg.ext_perimeter_flow.scaled_spacing();
        // Orca: ignore precise_outer_wall if wall_sequence is not InnerOuter
        let ext_perimeter_spacing2: Coord = if cfg.precise_outer_wall && cfg.wall_sequence == WallSequence::InnerOuter {
            pg_scaled_float(0.5f32 * (cfg.ext_perimeter_flow.width() as f32 + cfg.perimeter_flow.width() as f32))
        } else {
            pg_scaled_float(0.5f32 * (cfg.ext_perimeter_flow.spacing() as f32 + cfg.perimeter_flow.spacing() as f32))
        };

        // overhang perimeters
        self.m_mm3_per_mm_overhang.set(cfg.overhang_flow.as_ref().and_then(|f| f.mm3_per_mm().ok()).unwrap_or(-1.));

        // solid infill
        let solid_infill_flow = cfg.solid_infill_flow.as_ref().unwrap_or(&cfg.perimeter_flow);
        let solid_infill_spacing: Coord = solid_infill_flow.scaled_spacing();

        // Calculate the minimum required spacing between two adjacent traces.
        let min_spacing: Coord = (perimeter_spacing as f64 * (1. - INSET_OVERLAP_TOLERANCE)) as Coord;
        let ext_min_spacing: Coord = (ext_perimeter_spacing as f64 * (1. - INSET_OVERLAP_TOLERANCE)) as Coord;
        let has_gap_fill = cfg.gap_infill_speed > 0.;

        // BBS: this flow is for smaller external perimeter for small area
        let ext_min_spacing_smaller: Coord = (ext_perimeter_spacing as f64 * (1. - SMALLER_EXT_INSET_OVERLAP_TOLERANCE)) as Coord;
        // `with_width(SCALING_FACTOR * (ext_perimeter_width - 0.5 * SMALLER_EXT_INSET_OVERLAP_TOLERANCE *
        // ext_perimeter_spacing))`: the product is fused into the subtraction, the width is a float.
        let smaller_width = SCALING_FACTOR_PIN * (-(0.5 * SMALLER_EXT_INSET_OVERLAP_TOLERANCE)).mul_add(ext_perimeter_spacing as f64, ext_perimeter_width as f64);
        let smaller_ext_perimeter_flow = cfg.ext_perimeter_flow.with_width((smaller_width as f32) as f64).unwrap_or_else(|_| cfg.ext_perimeter_flow.clone());
        self.m_ext_mm3_per_mm_smaller_width.set(smaller_ext_perimeter_flow.mm3_per_mm().unwrap_or(-1.));

        // prepare grown lower layer slices for overhang detection
        let nozzle_diameter = cfg.perimeter_flow.nozzle_diameter() as f32;
        let lower_slices = cfg.lower_slices.as_ref();
        let m_lower_polygons_series = pg_generate_lower_polygons_series(lower_slices, cfg.perimeter_flow.width() as f32, nozzle_diameter);
        let m_lower_overhang_dist_boundary = pg_dist_boundary((cfg.perimeter_flow.width() as f32) as f64, nozzle_diameter);
        let (m_external_lower_polygons_series, m_external_overhang_dist_boundary) = if ext_perimeter_width == perimeter_width {
            (m_lower_polygons_series.clone(), m_lower_overhang_dist_boundary)
        } else {
            (
                pg_generate_lower_polygons_series(lower_slices, cfg.ext_perimeter_flow.width() as f32, nozzle_diameter),
                pg_dist_boundary((cfg.ext_perimeter_flow.width() as f32) as f64, nozzle_diameter),
            )
        };
        let members = PerimeterGeneratorClassicMembers {
            config: cfg,
            m_smaller_external_lower_polygons_series: pg_generate_lower_polygons_series(lower_slices, smaller_ext_perimeter_flow.width() as f32, nozzle_diameter),
            m_smaller_external_overhang_dist_boundary: pg_dist_boundary((smaller_ext_perimeter_flow.width() as f32) as f64, nozzle_diameter),
            smaller_ext_perimeter_flow: &smaller_ext_perimeter_flow,
            m_lower_polygons_series,
            m_external_lower_polygons_series,
            m_lower_overhang_dist_boundary,
            m_external_overhang_dist_boundary,
            m_ext_mm3_per_mm: self.m_ext_mm3_per_mm.get(),
            m_mm3_per_mm: self.m_mm3_per_mm.get(),
            m_mm3_per_mm_overhang: self.m_mm3_per_mm_overhang.get(),
            m_ext_mm3_per_mm_smaller_width: self.m_ext_mm3_per_mm_smaller_width.get(),
        };

        let mut all_surfaces: Vec<crate::surface::Surface> = surfaces.to_vec();
        self.process_no_bridge(&mut all_surfaces, perimeter_spacing, ext_perimeter_width, &mut result);

        // PerimeterGenerator.hpp:97 `m_scaled_resolution`
        let m_scaled_resolution = pg_scale_(if cfg.surface_simplify_resolution > crate::libslic3r::EPSILON { cfg.surface_simplify_resolution } else { crate::libslic3r::EPSILON });
        // BBS: don't simplify too much which influence arc fitting when export gcode if arc_fitting is enabled
        let surface_simplify_resolution =
            if cfg.arc_fitting_enabled && cfg.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None { 0.2 * m_scaled_resolution } else { m_scaled_resolution };
        //BBS: reorder the surface to reduce the travel time
        let surface_exp: ExPolygons = all_surfaces.iter().map(|s| s.expolygon.clone()).collect();
        let surface_order = crate::shortest_path::chain_expolygons(&surface_exp);
        for order_idx in 0..surface_order.len() {
            let surface = &all_surfaces[surface_order[order_idx]];
            // detect how many perimeters must be generated for this island
            let wall_loops: i32 = cfg.perimeter_count as i32;
            let mut loop_number: i32 = wall_loops + surface.extra_perimeters as i32 - 1; // 0-indexed loops
            if cfg.alternate_extra_wall && cfg.layer_id % 2 == 1 && !cfg.spiral_vase {
                loop_number += 1;
            }
            //BBS: set the topmost and bottom most layer to be one wall
            if loop_number > 0
                && ((cfg.top_one_wall_type != TopOneWallType::None && cfg.upper_slices.is_none()) || (cfg.only_one_wall_first_layer && cfg.starts_at_object_base))
            {
                loop_number = 0;
            }

            let mut last: ExPolygons = union_ex(&simplify_p(&surface.expolygon, surface_simplify_resolution));
            let mut gaps: ExPolygons = Vec::new();
            let mut top_fills: ExPolygons = Vec::new();
            let mut fill_clip: ExPolygons = Vec::new();
            let mut outwall_paths: Vec<crate::layer::NodeContour> = Vec::new();
            if loop_number >= 0 {
                // In case no perimeters are to be generated, loop_number will equal to -1.
                let mut contours: Vec<Vec<PerimeterLoop>> = vec![Vec::new(); loop_number as usize + 1]; // depth => loops
                let mut holes: Vec<Vec<PerimeterLoop>> = vec![Vec::new(); loop_number as usize + 1]; // depth => loops
                // In the pin's layout: `widths` is two per segment, scaled ([`pg_thick_polyline`]).
                let mut thin_walls: ThickPolylines = Vec::new();
                // we loop one time more than needed in order to find gaps after the last perimeter was applied
                let mut i: i32 = 0;
                loop {
                    // outer loop is 0
                    // Calculate next onion shell of perimeters.
                    let mut offsets: ExPolygons = Vec::new();
                    let mut offsets_with_smaller_width: ExPolygons = Vec::new();
                    if i == 0 {
                        // look for thin walls
                        if cfg.detect_thin_wall {
                            // the minimum thickness of a single loop is:
                            // ext_width/2 + ext_spacing/2 + spacing/2 + width/2
                            offsets = offset2_ex(
                                &last,
                                -((ext_perimeter_width as f64 / 2. + ext_min_spacing as f64 / 2. - 1.) as f32),
                                (ext_min_spacing as f64 / 2. - 1.) as f32,
                            );
                            // the following offset2 ensures almost nothing in @thin_walls is narrower than $min_width
                            // (actually, something larger than that still may exist due to mitering or other causes)
                            let min_width: Coord = pg_scale_((cfg.ext_perimeter_flow.nozzle_diameter() as f32 / 3.) as f64) as Coord;
                            let expp = opening_ex(
                                // medial axis requires non-overlapping geometry
                                &diff_ex_ep(&last, &offset(&offsets, (ext_perimeter_width as f64 / 2.) as f32 + 10f32), ApplySafetyOffset::No),
                                (min_width as f64 / 2.) as f32,
                            );
                            // the maximum thickness of our thin wall area is equal to the minimum thickness of a single loop
                            for ex in &expp {
                                let mut pp: Vec<crate::geometry::MedialAxisThickPolyline> = Vec::new();
                                ex.medial_axis_thick_polylines(min_width as f64, (ext_perimeter_width + ext_perimeter_spacing2) as f64, &mut pp);
                                for pl in &pp {
                                    if let Some(tp) = pg_thick_polyline(pl) {
                                        thin_walls.push(tp);
                                    }
                                }
                            }
                        } else {
                            let ext_perimeter_smaller_width: Coord = smaller_ext_perimeter_flow.scaled_width();
                            for expolygon in &last {
                                // BBS: judge whether it's narrow but not too long island which is hard to place two line
                                let expolys = vec![expolygon.clone()];
                                let offset_result = offset2_ex(
                                    &expolys,
                                    -((ext_perimeter_width as f64 / 2. + ext_min_spacing_smaller as f64 / 2.) as f32),
                                    (ext_min_spacing_smaller as f64 / 2.) as f32,
                                );
                                if offset_result.is_empty()
                                    && expolygon.area() < (ext_perimeter_width + ext_min_spacing_smaller) as f64 * pg_scale_(NARROW_LOOP_LENGTH_THRESHOLD)
                                {
                                    // BBS: for narrow external loop, use smaller line width
                                    offsets_with_smaller_width.extend(offset_ex_expolygon(expolygon, -((ext_perimeter_smaller_width as f64 / 2.) as f32)));
                                } else {
                                    //BBS: for not narrow loop, use normal external perimeter line width
                                    offsets.extend(offset_ex_expolygon(expolygon, -((ext_perimeter_width as f64 / 2.) as f32)));
                                }
                            }
                        }
                        if cfg.spiral_vase && (offsets.len() > 1 || offsets_with_smaller_width.len() > 1) {
                            // Remove all but the largest area polygon.
                            crate::geometry::keep_largest_contour_only(&mut offsets);
                            //BBS
                            if offsets.is_empty() {
                                //BBS: only have small width loop, then keep the largest in spiral vase mode
                                crate::geometry::keep_largest_contour_only(&mut offsets_with_smaller_width);
                            } else {
                                //BBS: have large area, clean the small width loop
                                offsets_with_smaller_width.clear();
                            }
                        }
                    } else {
                        //FIXME Is this offset correct if the line width of the inner perimeters differs
                        // from the line width of the infill?
                        let distance: Coord = if i == 1 { ext_perimeter_spacing2 } else { perimeter_spacing };
                        //BBS: For internal perimeter, we should "enable" thin wall strategy in which offset2 is used to
                        // remove too closed line, so that gap fill can be used for such internal narrow area in following
                        // handling.
                        offsets = offset2_ex(&last, -((distance as f64 + min_spacing as f64 / 2. - 1.) as f32), (min_spacing as f64 / 2. - 1.) as f32);
                        // look for gaps
                        if has_gap_fill {
                            // not using safety offset here would "detect" very narrow gaps
                            // (but still long enough to escape the area threshold) that gap fill
                            // won't be able to fill but we'd still remove from infill area
                            gaps.extend(diff_ex_pp(&offset(&last, -((0.5 * distance as f64) as f32)), &offset(&offsets, (0.5 * distance as f64 + 10.) as f32))); // safety offset
                        }
                    }
                    if offsets.is_empty() && offsets_with_smaller_width.is_empty() {
                        // Store the number of loops actually generated.
                        loop_number = i - 1;
                        // No region left to be filled in.
                        last.clear();
                        break;
                    } else if i > loop_number {
                        // If i > loop_number, we were looking just for gaps.
                        break;
                    }
                    {
                        let depth = i as usize;
                        let width = if i == 0 { cfg.ext_perimeter_flow.width() } else { cfg.perimeter_flow.width() };
                        for expolygon in &offsets {
                            // Outer contour may overlap with an inner contour,
                            // inner contour may overlap with another inner contour,
                            // outer contour may overlap with itself.
                            contours[depth].push(PerimeterLoop::new(expolygon.contour.clone(), depth, true, false, width));
                            for hole in &expolygon.holes {
                                holes[depth].push(PerimeterLoop::new(hole.clone(), depth, false, false, width));
                            }
                        }

                        //BBS: save perimeter loop which use smaller width
                        if i == 0 {
                            //store outer wall
                            if cfg.z_direction_outwall_speed_continuous {
                                // not loop
                                for polyline in thin_walls.iter() {
                                    outwall_paths.push(crate::layer::NodeContour {
                                        is_loop: false,
                                        pts: polyline.points.clone(),
                                        widths: polyline.widths.iter().map(|w| *w as Coord).collect(),
                                    });
                                }
                                // loop: `to_polylines(const ExPolygons &)` closes every ring by its first point
                                // (ExPolygon.hpp:231-247).
                                let mut rings = |expolygons: &ExPolygons, width: Coord| {
                                    for expolygon in expolygons {
                                        for poly in std::iter::once(&expolygon.contour).chain(expolygon.holes.iter()) {
                                            let mut pts = poly.points.clone();
                                            if let Some(first) = pts.first().copied() {
                                                pts.push(first);
                                            }
                                            outwall_paths.push(crate::layer::NodeContour { is_loop: true, pts, widths: vec![width] });
                                        }
                                    }
                                };
                                rings(&offsets_with_smaller_width, pg_scale_((smaller_ext_perimeter_flow.width() as f32) as f64) as Coord);
                                rings(&offsets, pg_scale_((cfg.ext_perimeter_flow.width() as f32) as f64) as Coord);
                            }

                            let smaller = smaller_ext_perimeter_flow.width();
                            for expolygon in &offsets_with_smaller_width {
                                contours[depth].push(PerimeterLoop::new(expolygon.contour.clone(), depth, true, true, smaller));
                                for hole in &expolygon.holes {
                                    holes[depth].push(PerimeterLoop::new(hole.clone(), depth, false, true, smaller));
                                }
                            }
                        }
                    }

                    last = offsets;

                    //BBS: refer to superslicer
                    //store surface for top infill if only_one_wall_top
                    if i == 0 && i != loop_number && cfg.top_one_wall_type == TopOneWallType::AllTop && cfg.upper_slices.is_some() {
                        //split the polygons with top/not_top
                        //get the offset from solid surface anchor
                        let mut offset_top_surface: Coord = pg_scale_(
                            1.5 * (if wall_loops == 0 { 0. } else { (ext_perimeter_width + perimeter_spacing * (wall_loops as Coord - 1)) as f64 * SCALING_FACTOR_PIN }),
                        ) as Coord;
                        // if possible, try to not push the extra perimeters inside the sparse infill
                        let inside = 0.9 * (if wall_loops <= 1 { 0. } else { (perimeter_spacing * (wall_loops as Coord - 1)) as f64 });
                        if offset_top_surface as f64 > inside {
                            offset_top_surface -= inside as Coord;
                        } else {
                            offset_top_surface = 0;
                        }
                        //don't takes into account too thin areas
                        let min_width_top_surface: f64 = (cfg.top_area_threshold / 100.) * f64::max(ext_perimeter_spacing as f64 / 2.0, perimeter_width as f64 / 2.0);

                        //BBS: get boungding box of last
                        // `last_box.offset(SCALED_EPSILON)`: a Point of doubles, rounded (BoundingBox.hpp:42-47).
                        let mut last_box = crate::geometry::get_extents(&last);
                        let eps = pg_scale_(crate::libslic3r::EPSILON).round_ties_even() as Coord;
                        last_box.min = Point::new(last_box.min.x - eps, last_box.min.y - eps);
                        last_box.max = Point::new(last_box.max.x + eps, last_box.max.y + eps);

                        // BBS: get the Polygons upper the polygon this layer
                        let mut upper_polygons_series_clipped =
                            crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(cfg.upper_slices.as_ref().unwrap(), &last_box, false);
                        upper_polygons_series_clipped = offset_polygons(&upper_polygons_series_clipped, min_width_top_surface as f32);

                        //set the clip to a virtual "second perimeter"
                        fill_clip = offset_ex(&last, -(ext_perimeter_spacing as f64) as f32);
                        // get the real top surface
                        let mut top_polygons = diff_ex_ep(&last, &upper_polygons_series_clipped, ApplySafetyOffset::Yes);
                        //get the not-top surface, from the "real top" but enlarged by external_infill_margin (and the min_width_top_surface we removed a bit before)
                        let temp_gap = diff_ex(&top_polygons, &fill_clip, ApplySafetyOffset::No);
                        let mut inner_polygons = diff_ex(
                            &last,
                            &offset_ex(&top_polygons, (offset_top_surface as f64 + min_width_top_surface - (ext_perimeter_spacing / 2) as f64) as f32),
                            ApplySafetyOffset::Yes,
                        );
                        // BBS: check whether surface be bridge or not
                        if let Some(lower_slices) = cfg.lower_slices.as_ref() {
                            // BBS: get the Polygons below the polygon this layer
                            let lower_polygons_series_clipped = crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(lower_slices, &last_box, false);

                            let bridge_offset = f64::max(ext_perimeter_spacing as f64, perimeter_width as f64);
                            let bridge_checker = offset_ex(&diff_ex_ep(&last, &lower_polygons_series_clipped, ApplySafetyOffset::Yes), (1.5 * bridge_offset) as f32);

                            // BBS: Check which piece the bridge belongs to. If the bridge has a connection with the non-top area, it should belong to the non-top area, otherwise it should belong to the top area to get a better surface.
                            if !bridge_checker.is_empty() && !intersection_ex(&bridge_checker, &inner_polygons).is_empty() {
                                inner_polygons = union_ex_2(&inner_polygons, &bridge_checker);
                            }
                        }
                        // get the enlarged top surface, by using inner_polygons instead of upper_slices, and clip it for it to be exactly the polygons to fill.
                        top_polygons = diff_ex(&fill_clip, &inner_polygons, ApplySafetyOffset::Yes);
                        // increase by half peri the inner space to fill the frontier between last and stored.
                        top_fills = union_ex_2(&top_fills, &top_polygons);
                        //set the clip to the external wall but go back inside by infill_extrusion_width/2 to be sure the extrusion won't go outside even with a 100% overlap.
                        let infill_spacing_unscaled: f64 = cfg.sparse_infill_line_width;
                        fill_clip = offset_ex(&last, ((ext_perimeter_spacing / 2) as f64 - pg_scale_(infill_spacing_unscaled / 2.)) as f32);
                        last = intersection_ex(&inner_polygons, &last);
                        if has_gap_fill {
                            last = union_ex_2(&last, &temp_gap);
                        }
                    }

                    if i == loop_number && (!has_gap_fill || cfg.sparse_infill_density == 0.) {
                        // The last run of this loop is executed to collect gaps for gap fill.
                        // As the gap fill is either disabled or not
                        break;
                    }
                    i += 1;
                }

                // nest loops: holes first
                for d in 0..=(loop_number.max(-1) + 1) as usize {
                    if d as i32 > loop_number {
                        break;
                    }
                    // loop through all holes having depth == d
                    let holes_d = std::mem::take(&mut holes[d]);
                    let mut kept: Vec<PerimeterLoop> = Vec::new();
                    'next_loop: for hole in holes_d {
                        // find the hole loop that contains this one, if any
                        for t in (d + 1)..=(loop_number as usize) {
                            for j in 0..holes[t].len() {
                                if holes[t][j].polygon.contains(&hole.polygon.first_point()) {
                                    holes[t][j].children.push(hole);
                                    continue 'next_loop;
                                }
                            }
                        }
                        // if no hole contains this hole, find the contour loop that contains it
                        for t in (0..=(loop_number as usize)).rev() {
                            for j in 0..contours[t].len() {
                                if contours[t][j].polygon.contains(&hole.polygon.first_point()) {
                                    contours[t][j].children.push(hole);
                                    continue 'next_loop;
                                }
                            }
                        }
                        kept.push(hole);
                    }
                    holes[d] = kept;
                }
                // nest contour loops
                let mut d = loop_number;
                while d >= 1 {
                    let du = d as usize;
                    // loop through all contours having depth == d
                    let contours_d = std::mem::take(&mut contours[du]);
                    let mut kept: Vec<PerimeterLoop> = Vec::new();
                    'next_contour: for contour in contours_d {
                        // find the contour loop that contains it
                        for t in (0..du).rev() {
                            for j in 0..contours[t].len() {
                                if contours[t][j].polygon.contains(&contour.polygon.first_point()) {
                                    contours[t][j].children.push(contour);
                                    continue 'next_contour;
                                }
                            }
                        }
                        kept.push(contour);
                    }
                    contours[du] = kept;
                    d -= 1;
                }
                // at this point, all loops should be in contours[0]
                let mut entities = if contours.is_empty() {
                    crate::extrusion_entity::ExtrusionEntityCollection::new()
                } else {
                    traverse_loops(&members, &contours[0], &mut thin_walls)
                };
                // The fuzzy skin of the pin is put on the polygon of a loop before its overhangs are looked
                // for (`apply_fuzzy_skin`, PerimeterGenerator.cpp:348-349, FuzzySkin.cpp, whose noise cannot
                // be repeated); this arm fuzzes the finished walls, as the body before it does.
                if cfg.fuzzy_skin_mode != crate::region_config::FuzzySkinMode::None {
                    let fs_config = crate::fuzzy_skin::FuzzySkinConfig {
                        thickness: cfg.fuzzy_skin_thickness,
                        point_distance: cfg.fuzzy_skin_point_distance,
                        mode: cfg.fuzzy_skin_mode,
                    };
                    for entity in &mut entities.entities {
                        apply_fuzzy_skin_to_entity(entity, &fs_config, cfg.fuzzy_skin_mode);
                    }
                }
                // if brim will be printed, reverse the order of perimeters so that
                // we continue inwards after having finished the brim
                let is_outer_wall_first = cfg.wall_sequence == WallSequence::OuterInner;
                if is_outer_wall_first
                    //BBS: always print outer wall first when there indeed has brim.
                    || (cfg.starts_at_object_base && cfg.brim_outer_only)
                {
                    entities.reverse();
                }
                //BBS. adjust wall generate seq
                else if cfg.wall_sequence == WallSequence::InnerOuterInner {
                    if entities.entities.len() > 1 {
                        let mut entities_reorder: Vec<ExtrusionEntityType> = Vec::new();
                        let mut entities_second_wall: Vec<ExtrusionEntityType> = Vec::new();
                        for entity in std::mem::take(&mut entities.entities) {
                            // `static_cast<ExtrusionLoop *>` of every entity: a thin wall is no loop; its
                            // bytes are not read as a loop role here.
                            let second = matches!(&entity, ExtrusionEntityType::Loop(l) if l.loop_role.contains(crate::extrusion_entity::ExtrusionLoopRole::SECOND_PERIMETER));
                            if second {
                                entities_second_wall.push(entity);
                            } else {
                                let external = entity_role(&entity) == Some(ExtrusionRole::ExternalPerimeter);
                                entities_reorder.push(entity);
                                if external && !entities_second_wall.is_empty() {
                                    entities_reorder.append(&mut entities_second_wall);
                                }
                            }
                        }
                        entities.entities = entities_reorder;
                    }
                }

                //BBS: add node for loops
                if !outwall_paths.is_empty() && cfg.layer_id > 0 {
                    let node_of = |node_id: usize, loop_id: usize, contour: &crate::layer::NodeContour| {
                        // `node.bbox = get_extents(node.node_contour.pts); node.bbox.offset(SCALED_EPSILON);`
                        let mut bbox = crate::geometry::BoundingBox::from_points(&contour.pts);
                        let eps = pg_scale_(crate::libslic3r::EPSILON).round_ties_even() as Coord;
                        bbox.min = Point::new(bbox.min.x - eps, bbox.min.y - eps);
                        bbox.max = Point::new(bbox.max.x + eps, bbox.max.y + eps);
                        let mut node = crate::layer::LoopNode::new(node_id, loop_id, bbox);
                        node.node_contour = contour.clone();
                        node
                    };
                    entities.loop_node_range.0 = result.loop_nodes.len();
                    if outwall_paths.len() == 1 {
                        result.loop_nodes.push(node_of(result.loop_nodes.len(), 0, &outwall_paths[0]));
                    } else {
                        let mut matched = vec![false; outwall_paths.len()];
                        for entity_idx in 0..entities.entities.len() {
                            //skip inner wall
                            if entity_role(&entities.entities[entity_idx]) == Some(ExtrusionRole::Perimeter) {
                                continue;
                            }
                            let Some(first_point) = entity_first_point(&entities.entities[entity_idx]) else { continue };
                            for lines_idx in 0..outwall_paths.len() {
                                if matched[lines_idx] {
                                    continue;
                                }
                                if point_is_in_lines(&first_point, &outwall_paths[lines_idx].pts) {
                                    matched[lines_idx] = true;
                                    result.loop_nodes.push(node_of(result.loop_nodes.len(), entity_idx, &outwall_paths[lines_idx]));
                                    break;
                                }
                            }
                        }
                    }
                    entities.loop_node_range.1 = result.loop_nodes.len();
                }

                // append perimeters for this slice as a collection
                if !entities.entities.is_empty() {
                    result.entities.entities.push(ExtrusionEntityType::Collection(Box::new(entities)));
                }
            } // for each loop of an island

            // fill gaps
            if !gaps.is_empty() {
                // collapse
                let min: f64 = 0.2 * perimeter_width as f64 * (1. - INSET_OVERLAP_TOLERANCE);
                let max: f64 = 2. * perimeter_spacing as f64;
                let mut gaps_ex = diff_ex(
                    //FIXME offset2 would be enough and cheaper.
                    &opening_ex(&gaps, (min / 2.) as f32),
                    &offset2_ex(&gaps, -((max / 2.) as f32), (max / 2. + 10.) as f32),
                    ApplySafetyOffset::No,
                );
                let mut polylines: ThickPolylines = Vec::new();
                for ex in gaps_ex.iter_mut() {
                    //BBS: Use DP simplify to avoid duplicated points and accelerate medial-axis calculation as well.
                    ex.contour.douglas_peucker(surface_simplify_resolution);
                    for hole in ex.holes.iter_mut() {
                        hole.douglas_peucker(surface_simplify_resolution);
                    }
                    let mut pp: Vec<crate::geometry::MedialAxisThickPolyline> = Vec::new();
                    ex.medial_axis_thick_polylines(min, max, &mut pp);
                    for pl in &pp {
                        // OrcaSlicer: filter out tiny gap fills
                        if pg_polyline_length(&pl.points) < pg_scale_(cfg.filter_out_gap_fill) {
                            continue;
                        }
                        if let Some(tp) = pg_thick_polyline(pl) {
                            polylines.push(tp);
                        }
                    }
                }

                if !polylines.is_empty() {
                    let mut gap_fill = crate::extrusion_entity::ExtrusionEntityCollection::new();
                    pg_variable_width(&polylines, ExtrusionRole::GapFill, solid_infill_flow, &mut gap_fill.entities);
                    /*  Make sure we don't infill narrow parts that are already gap-filled
                        (we only consider this surface's gaps to reduce the diff() complexity).
                        Growing actual extrusions ensures that gaps not filled by medial axis
                        are not subtracted from fill surfaces (they might be too short gaps
                        that medial axis skips but infill might join with other infill regions
                        and use zigzag).  */
                    let mut covered: Vec<Polygon> = Vec::new();
                    for entity in &gap_fill.entities {
                        match entity {
                            ExtrusionEntityType::Path(path) => path.polygons_covered_by_width(&mut covered, 10.0),
                            ExtrusionEntityType::Loop(l) => l.polygons_covered_by_width(&mut covered, 10.0),
                            ExtrusionEntityType::MultiPath(m) => m.polygons_covered_by_width(&mut covered, 10.0),
                            ExtrusionEntityType::Collection(_) => {}
                        }
                    }
                    last = diff_ex_ep(&last, &covered, ApplySafetyOffset::No);
                    result.gap_fill.entities.append(&mut gap_fill.entities);
                }
            }

            // create one more offset to be used as boundary for fill
            // we offset by half the perimeter spacing (to get to the actual infill boundary)
            // and then we offset back and forth by half the infill spacing to only consider the
            // non-collapsing regions
            let mut inset: Coord = if loop_number < 0 {
                0
            } else if loop_number == 0 {
                // one loop
                ext_perimeter_spacing / 2
            } else {
                // two or more loops?
                perimeter_spacing / 2
            };
            // only apply infill overlap if we actually have one perimeter
            let mut infill_peri_overlap: Coord = 0;
            if inset > 0 {
                // `config->infill_wall_overlap.get_abs_value(unscale<double>(inset + solid_infill_spacing / 2))`:
                // `ratio_over * value / 100` (Config.hpp:1216)
                let ratio_over = (inset + solid_infill_spacing / 2) as f64 * SCALING_FACTOR_PIN;
                infill_peri_overlap = pg_scale_(ratio_over * cfg.infill_wall_overlap_percent / 100.) as Coord;
                inset -= infill_peri_overlap;
            }
            // simplify infill contours according to resolution
            let mut pp: Vec<Polygon> = Vec::new();
            for ex in &last {
                pp.extend(simplify_p(ex, m_scaled_resolution));
            }
            let not_filled_exp = union_ex(&pp);
            // collapse too narrow infill areas
            let min_perimeter_infill_spacing: Coord = (solid_infill_spacing as f64 * (1. - INSET_OVERLAP_TOLERANCE)) as Coord;

            let mut infill_exp = offset2_ex(
                &not_filled_exp,
                (-(inset as f64) - min_perimeter_infill_spacing as f64 / 2.) as f32,
                (min_perimeter_infill_spacing as f64 / 2.) as f32,
            );
            // append infill areas to fill_surfaces
            //if any top_fills, grow them by ext_perimeter_spacing/2 to have the real un-anchored fill
            let top_infill_exp = intersection_ex(&fill_clip, &offset_ex(&top_fills, (ext_perimeter_spacing / 2) as f64 as f32));
            if !top_fills.is_empty() {
                infill_exp = union_ex_2(&infill_exp, &offset_ex(&top_infill_exp, infill_peri_overlap as f64 as f32));
            }
            result.infill_area.extend(infill_exp);

            // BBS: get the no-overlap infill expolygons
            {
                let mut poly_without_overlap = if min_perimeter_infill_spacing / 2 > infill_peri_overlap {
                    offset2_ex(
                        &not_filled_exp,
                        (-(inset as f64) - min_perimeter_infill_spacing as f64 / 2.) as f32,
                        (min_perimeter_infill_spacing / 2 - infill_peri_overlap) as f32,
                    )
                } else {
                    offset_ex(&not_filled_exp, (-inset - infill_peri_overlap) as f64 as f32)
                };
                if !top_fills.is_empty() {
                    poly_without_overlap = union_ex_2(&poly_without_overlap, &top_infill_exp);
                }
                result.no_overlap_area.extend(poly_without_overlap);
            }
        } // for each island
        result
    }
}

/// A polyline of the medial axis as a `ThickPolyline` in the pin's layout (Polyline.hpp:245-285): `widths` is
/// the C++ `width`, two per segment, in scaled units (not the one per point in millimetres of the legacy
/// body); `edge_widths` holds the same pairs, which `thicklines()` reads first.
fn pg_thick_polyline(pl: &crate::geometry::MedialAxisThickPolyline) -> Option<ThickPolyline> {
    if pl.points.len() < 2 || pl.width.is_empty() {
        return None;
    }
    let mut tp = ThickPolyline::with_capacity(pl.points.len());
    tp.points = pl.points.clone();
    tp.widths = pl.width.clone();
    tp.endpoints = [pl.endpoints.0, pl.endpoints.1];
    tp.edge_widths = pl.width.clone();
    Some(tp)
}

/// Polyline.cpp `Polyline::length()`: the sum of the lengths of the segments, each the norm of the
/// difference of its points as doubles.
fn pg_polyline_length(points: &[Point]) -> f64 {
    let mut l = 0.;
    for w in points.windows(2) {
        let dx = (w[1].x - w[0].x) as f64;
        let dy = (w[1].y - w[0].y) as f64;
        l += dx.mul_add(dx, dy * dy).sqrt();
    }
    l
}

/// VariableWidth.cpp:214-231 `variable_width(polylines, role, flow, out)` ([`crate::variable_width::variable_width`])
/// over thick polylines in the pin's layout ([`pg_thick_polyline`]).
fn pg_variable_width(polylines: &ThickPolylines, role: ExtrusionRole, flow: &Flow, out: &mut Vec<ExtrusionEntityType>) {
    crate::variable_width::variable_width(polylines, role, flow, out);
}

/// PerimeterGenerator.cpp:1886-1909 `PerimeterGenerator::generate_lower_polygons_series(float width)` with
/// the nozzle diameter of the wall filament handed in (`nozzle_diameter_for_filament(*print_config,
/// config->wall_filament)`, a float there): the arithmetic of
/// [`PerimeterGenerator::generate_lower_polygons_series`].
fn pg_generate_lower_polygons_series(lower_slices: Option<&ExPolygons>, width: f32, nozzle_diameter: f32) -> Vec<crate::geometry::Polygons> {
    use crate::support::support_common::cu;
    let start_offset: f32 = (-0.5 * width as f64) as f32;
    let end_offset: f32 = (0.5 * nozzle_diameter as f64) as f32;
    // generate offsets
    let offset_series: [f32; 2] =
        [(start_offset as f64 + 0.5 * (end_offset - start_offset) as f64 / (crate::overhang_detector::OVERHANG_SAMPLING_NUMBER - 1) as f64) as f32, end_offset];
    let mut lower_polygons_series: Vec<crate::geometry::Polygons> = Vec::new();
    let Some(lower_slices) = lower_slices else {
        return lower_polygons_series;
    };
    // offset expolygon to generate series of polygons
    for o in offset_series {
        lower_polygons_series.push(cu::offset_expolygons(lower_slices, (o as f64 / SCALING_FACTOR_PIN) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
    }
    lower_polygons_series
}

/// PerimeterGenerator.cpp:232-242 `PerimeterGenerator::dist_boundary(double width)` with the nozzle diameter
/// of the wall filament handed in: the arithmetic of [`PerimeterGenerator::dist_boundary`].
fn pg_dist_boundary(width: f64, nozzle_diameter: f32) -> (f64, f64) {
    let start_offset: f32 = (-0.5 * width) as f32;
    let end_offset: f32 = (0.5 * nozzle_diameter as f64) as f32;
    let degree_0: f64 =
        (start_offset as f64 + 0.5 * (end_offset - start_offset) as f64 / (crate::overhang_detector::OVERHANG_SAMPLING_NUMBER - 1) as f64) / SCALING_FACTOR_PIN;
    (0., end_offset as f64 / SCALING_FACTOR_PIN - degree_0)
}

/// The members of the PerimeterGenerator `traverse_loops` reads (PerimeterGenerator.cpp:293-501,
/// PerimeterGenerator.hpp:60-66, 130-134), as process_classic sets them (:862-916).
struct PerimeterGeneratorClassicMembers<'a> {
    config: &'a PerimeterConfig,
    smaller_ext_perimeter_flow: &'a Flow,
    m_lower_polygons_series: Vec<crate::geometry::Polygons>,
    m_external_lower_polygons_series: Vec<crate::geometry::Polygons>,
    m_smaller_external_lower_polygons_series: Vec<crate::geometry::Polygons>,
    m_lower_overhang_dist_boundary: (f64, f64),
    m_external_overhang_dist_boundary: (f64, f64),
    m_smaller_external_overhang_dist_boundary: (f64, f64),
    m_ext_mm3_per_mm: f64,
    m_mm3_per_mm: f64,
    m_mm3_per_mm_overhang: f64,
    m_ext_mm3_per_mm_smaller_width: f64,
}

/// PerimeterGenerator.cpp:293-501 `static ExtrusionEntityCollection traverse_loops(const PerimeterGenerator
/// &perimeter_generator, const PerimeterGeneratorLoops &loops, ThickPolylines &thin_walls)`.
///
/// Not carried by this crate's inputs: the circle compensation of a loop (`need_circle_compensation`, the
/// flag stays `cfNone`), the perimeter regions (none), and `apply_fuzzy_skin` (:348-349), which hands the
/// polygon back where the fuzzy skin is off.
fn traverse_loops(
    perimeter_generator: &PerimeterGeneratorClassicMembers,
    loops: &[PerimeterLoop],
    thin_walls: &mut ThickPolylines,
) -> ExtrusionEntityCollection {
    use crate::clipper2_utils::{diff_pl_2, intersection_pl_2};
    use crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_polygons;
    use crate::extrusion_entity::CustomizeFlag;
    let cfg = perimeter_generator.config;
    let overhang_flow = cfg.overhang_flow.as_ref().unwrap_or(&cfg.perimeter_flow);
    let empty: crate::geometry::Polygons = Vec::new();
    // `bbox.offset(SCALED_EPSILON)`: a Point of doubles, rounded (BoundingBox.hpp:42-47).
    let eps = pg_scale_(crate::libslic3r::EPSILON).round_ties_even() as Coord;

    // loops is an arrayref of ::Loop objects
    // turn each one into an ExtrusionLoop object
    let mut coll = ExtrusionEntityCollection::new();
    for loop_ in loops {
        let is_external = loop_.is_external();
        let is_small_width = loop_.is_smaller_width;
        let flag = CustomizeFlag::None;

        let role = if is_external { ExtrusionRole::ExternalPerimeter } else { ExtrusionRole::Perimeter };
        let mut loop_role = if loop_.is_internal_contour() {
            // Note that we set loop role to ContourInternalPerimeter
            // also when loop is both internal and external (i.e.
            // there's only one contour loop).
            ExtrusionLoopRole::CONTOUR_INTERNAL_PERIMETER
        } else if loop_.is_contour {
            ExtrusionLoopRole::DEFAULT
        } else {
            ExtrusionLoopRole::PERIMETER_HOLE
        };

        if loop_.perimeter_index == 1 {
            if loop_role == ExtrusionLoopRole::DEFAULT {
                loop_role = ExtrusionLoopRole::SECOND_PERIMETER;
            } else {
                loop_role = loop_role | ExtrusionLoopRole::SECOND_PERIMETER;
            }
        }

        // detect overhanging/bridging perimeters
        let mut paths: Vec<ExtrusionPath> = Vec::new();

        // BBS: get lower polygons series, width, mm3_per_mm
        let (lower_polygons_series, overhang_dist_boundary, extrusion_mm3_per_mm, extrusion_width): (&Vec<crate::geometry::Polygons>, &(f64, f64), f64, f64) =
            if is_external {
                if is_small_width {
                    //BBS: smaller width external perimeter
                    (
                        &perimeter_generator.m_smaller_external_lower_polygons_series,
                        &perimeter_generator.m_smaller_external_overhang_dist_boundary,
                        perimeter_generator.m_ext_mm3_per_mm_smaller_width,
                        perimeter_generator.smaller_ext_perimeter_flow.width(),
                    )
                } else {
                    //BBS: normal external perimeter
                    (
                        &perimeter_generator.m_external_lower_polygons_series,
                        &perimeter_generator.m_external_overhang_dist_boundary,
                        perimeter_generator.m_ext_mm3_per_mm,
                        cfg.ext_perimeter_flow.width(),
                    )
                }
            } else {
                //BBS: normal perimeter
                (&perimeter_generator.m_lower_polygons_series, &perimeter_generator.m_lower_overhang_dist_boundary, perimeter_generator.m_mm3_per_mm, cfg.perimeter_flow.width())
            };

        // Apply fuzzy skin if it is enabled for at least some part of the polygon.
        let polygon = &loop_.polygon;

        if cfg.detect_overhang_wall && cfg.layer_id > cfg.raft_layers && !cfg.starts_at_object_base {
            // get non 100% overhang paths by intersecting this loop with the grown lower slices
            // prepare grown lower layer slices for overhang detection
            let mut bbox = crate::geometry::BoundingBox::from_points(&polygon.points);
            bbox.min = Point::new(bbox.min.x - eps, bbox.min.y - eps);
            bbox.max = Point::new(bbox.max.x + eps, bbox.max.y + eps);

            //BBS: don't calculate overhang degree when enable fuzzy skin. It's unmeaning
            // (Without lower slices the series is empty and the pin reads the back of an empty vector;
            // no layer reaches this with `layer_id > raft_layers`.)
            let lower_polygons_series_clipped = clip_clipper_polygons_with_subject_bbox_polygons(lower_polygons_series.last().unwrap_or(&empty), &bbox);

            let polyline = crate::geometry::polygon::to_polyline(polygon);
            let inside_polines = intersection_pl_2(std::slice::from_ref(&polyline), &lower_polygons_series_clipped);

            let remain_polines = diff_pl_2(std::slice::from_ref(&polyline), &lower_polygons_series_clipped);

            // `is_enable_overhang_speed(perimeter_generator) && fuzzy_skin_allows_overhang_slowdown(perimeter_generator)`
            let detect_overhang_speed = cfg.enable_overhang_speed && cfg.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None;

            if !detect_overhang_speed {
                if !inside_polines.is_empty() {
                    crate::overhang_detector::extrusion_paths_append(&mut paths, inside_polines, 0., 0, role, extrusion_mm3_per_mm, extrusion_width as f32, cfg.layer_height as f32);
                }
            } else {
                let lower_polygons_series_clipped = clip_clipper_polygons_with_subject_bbox_polygons(lower_polygons_series.first().unwrap_or(&empty), &bbox);

                let middle_overhang_polyines = diff_pl_2(&inside_polines, &lower_polygons_series_clipped);
                //BBS: add zero_degree_path
                let zero_degree_polines = intersection_pl_2(&inside_polines, &lower_polygons_series_clipped);
                if !zero_degree_polines.is_empty() {
                    crate::overhang_detector::extrusion_paths_append(&mut paths, zero_degree_polines, 0., 0, role, extrusion_mm3_per_mm, extrusion_width as f32, cfg.layer_height as f32);
                }
                //BBS: detect middle line overhang
                if !middle_overhang_polyines.is_empty() {
                    crate::overhang_detector::detect_overhang_degree(
                        lower_polygons_series.first().unwrap_or(&empty).clone(),
                        role,
                        extrusion_mm3_per_mm,
                        extrusion_width,
                        cfg.layer_height,
                        middle_overhang_polyines,
                        overhang_dist_boundary.0,
                        overhang_dist_boundary.1,
                        &mut paths,
                    );
                }
            }

            // get 100% overhang paths by checking what parts of this loop fall
            // outside the grown lower slices (thus where the distance between
            // the loop centerline and original lower slices is >= half nozzle diameter
            if !remain_polines.is_empty() {
                // Detect overhang perimeter / bridge. Zero-gap support must not force normal wall role/flow here:
                // unsupported walls still need erOverhangPerimeter + overhang_flow. See GitHub #11462.
                detect_bridge_wall(
                    &mut paths,
                    &remain_polines,
                    ExtrusionRole::OverhangPerimeter,
                    perimeter_generator.m_mm3_per_mm_overhang,
                    overhang_flow.width() as f32,
                    overhang_flow.height() as f32,
                );
            }

            if paths.is_empty() {
                continue;
            }

            // Reapply the nearest point search for starting point.
            // We allow polyline reversal because Clipper may have randomly reversed polylines during clipping.
            let start = paths[0].first_point();
            crate::shortest_path::chain_and_reorder_extrusion_paths(&mut paths, Some(&start));
        } else {
            let mut path = ExtrusionPath::new(role);
            //BBS.
            path.polyline = polygon.split_at_first_point();
            path.overhang_degree = 0.;
            path.curve_degree = 0;
            path.mm3_per_mm = extrusion_mm3_per_mm;
            path.width = extrusion_width;
            path.height = cfg.layer_height;
            paths.push(path);
        }

        for path in paths.iter_mut() {
            path.set_customize_flag(flag);
        }

        coll.entities.push(ExtrusionEntityType::Loop(ExtrusionLoop::new_with_flag(paths, loop_role, flag)));
    }

    // Append thin walls to the nearest-neighbor search (only for first iteration)
    let mut zero_point = Point::new(0, 0);

    if !thin_walls.is_empty() {
        // `bbox.merge(entity->as_polyline().bounding_box())` of every loop; a box of nothing has both corners
        // at the origin (BoundingBox.hpp:21).
        let mut bbox = crate::geometry::BoundingBox::new();
        for entity in &coll.entities {
            if let ExtrusionEntityType::Loop(l) = entity {
                for path in &l.paths {
                    for p in &path.polyline.points {
                        bbox.merge_point(*p);
                    }
                }
            }
        }
        let (bbox_min, bbox_max) = if bbox.is_defined() { (bbox.min, bbox.max) } else { (Point::new(0, 0), Point::new(0, 0)) };
        for thin_wall in thin_walls.iter() {
            // find the corner of bbox that's farthest from the thin wall
            let first = thin_wall.points[0];
            let squared_norm = |c: &Point| {
                let dx = c.x as f64 - first.x as f64;
                let dy = c.y as f64 - first.y as f64;
                dx.mul_add(dx, dy * dy)
            };
            let mut corner_far = bbox_min;
            if squared_norm(&corner_far) < squared_norm(&bbox_max) {
                corner_far = bbox_max;
            }
            zero_point = corner_far;
        }
        pg_variable_width(thin_walls, ExtrusionRole::ExternalPerimeter, &cfg.ext_perimeter_flow, &mut coll.entities);
        thin_walls.clear();
    }

    // Traverse children and build the final collection.
    let chain: Vec<(usize, bool)> = crate::shortest_path::chain_extrusion_entities(&coll.entities, Some(&zero_point));
    let mut entities: Vec<Option<ExtrusionEntityType>> = coll.entities.drain(..).map(Some).collect();
    let mut out = ExtrusionEntityCollection::new();
    for (idx, reversed) in chain {
        let mut entity = entities[idx].take().unwrap();
        if idx >= loops.len() {
            // This is a thin wall.
            if reversed {
                entity_reverse_inplace(&mut entity);
            }
            out.entities.push(entity);
        } else {
            let loop_ = &loops[idx];
            debug_assert!(thin_walls.is_empty());
            let children = traverse_loops(perimeter_generator, &loop_.children, thin_walls);
            match entity {
                ExtrusionEntityType::Loop(mut eloop) => {
                    if loop_.is_contour {
                        eloop.make_counter_clockwise();
                        out.entities.extend(children.entities);
                        out.entities.push(ExtrusionEntityType::Loop(eloop));
                    } else {
                        eloop.make_clockwise();
                        out.entities.push(ExtrusionEntityType::Loop(eloop));
                        out.entities.extend(children.entities);
                    }
                }
                // A thin wall where a loop was skipped for having no path: the pin casts it to a loop.
                other => {
                    out.entities.extend(children.entities);
                    out.entities.push(other);
                }
            }
        }
    }
    out
}

// ---------------------------------------------------------------------------------------------
// PerimeterGenerator::process_arachne as the pin has it (R922)
// ---------------------------------------------------------------------------------------------

/// The ClipperUtils.cpp overloads process_arachne and its helpers call beside those of [`pgcu`], with
/// the default join type and miter limit where the C++ names none.
mod pgcu_arachne {
    use crate::clipper::JoinType;
    use crate::geometry::{ExPolygon, ExPolygons, Polygon, Polygons};
    use crate::support::support_common::cu;

    /// ClipperUtils.cpp:415 `offset_ex(const Polygons &, float delta)`
    pub fn offset_ex_polygons(polygons: &[Polygon], delta: f32) -> ExPolygons {
        cu::offset_ex(polygons, delta, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
    }
    /// ClipperUtils.cpp:815 `union_ex(const ExPolygons &)`
    pub fn union_ex_expolygons(subject: &[ExPolygon]) -> ExPolygons {
        cu::union_ex_expolygons(subject)
    }
    /// ClipperUtils.cpp:570 `offset_ex(const ExPolygons &, float delta, JoinType, double miterLimit)`
    pub fn offset_ex_jt(expolygons: &[ExPolygon], delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        cu::offset_ex_expolygons(expolygons, delta, join_type, miter_limit)
    }
    /// ClipperUtils.cpp:581-584 `offset2_ex(const ExPolygons &, float delta1, float delta2, JoinType, double miterLimit)`
    pub fn offset2_ex_jt(expolygons: &[ExPolygon], delta1: f32, delta2: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        cu::offset2_ex(expolygons, delta1, delta2, join_type, miter_limit)
    }
    /// ClipperUtils.cpp:410 `offset(const Polygon &, float delta)`
    pub fn offset_polygon(polygon: &Polygon, delta: f32) -> Polygons {
        cu::offset_polygon(polygon, delta, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
    }
}

impl PerimeterGenerator {
    /// PerimeterGenerator.hpp:97 `m_scaled_resolution(scaled<double>(print_config->resolution.value > EPSILON ?
    /// print_config->resolution.value : EPSILON))`
    fn m_scaled_resolution(&self) -> f64 {
        let resolution = self.config.surface_simplify_resolution;
        pg_scale_(if resolution > crate::libslic3r::EPSILON { resolution } else { crate::libslic3r::EPSILON })
    }

    /// R943 probe (PA_L=a-b, diagnostic only, never changes output): process_arachne's tail in the
    /// format of the pin's twin (target/parity-work/d/pa/patch.py over PerimeterGenerator.cpp:
    /// pa_s0 the surface, pa_last / pa_ic before traverse_extrusions (1837), pa_a0 / pa_ipp / pa_u /
    /// pa_fs / pa_no in add_infill_contour_for_arachne (1463-1484)); PAP=<tag>:<lidx> prints points.
    fn pa_probe(tag: &str, l: usize, pp: &[crate::geometry::Polygon], x: f64) {
        let Ok(v) = std::env::var("PA_L") else { return };
        let mut it = v.split('-');
        let a = it.next().and_then(|t| t.parse::<usize>().ok());
        let b = it.next().and_then(|t| t.parse::<usize>().ok()).or(a);
        match (a, b) {
            (Some(a), Some(b)) if l >= a && l <= b => {}
            _ => return,
        }
        let (mut np, mut sx, mut sy) = (0usize, 0i64, 0i64);
        for p in pp {
            for q in p.points() {
                np += 1;
                sx += q.x;
                sy += q.y;
            }
        }
        eprintln!("BOI {} L={} n={} np={} sx={} sy={} x={:.9}", tag, l, pp.len(), np, sx, sy, x);
        if std::env::var("PAP").ok().as_deref() == Some(format!("{}:{}", tag, l).as_str()) {
            for (i, p) in pp.iter().enumerate() {
                for q in p.points() {
                    eprintln!("PAP {} {} {}", i, q.x, q.y);
                }
            }
        }
    }

    /// PerimeterGenerator.cpp:1489-1848 `void PerimeterGenerator::process_arachne()`. `surfaces` is
    /// `this->slices->surfaces`; the result carries what the C++ writes: `entities` is `loops` (the
    /// collection of every island that has walls, one after the other), `infill_area` the expolygons of
    /// `fill_surfaces`, `no_overlap_area` is `fill_no_overlap`, `loop_nodes`.
    ///
    /// Not carried by this crate's inputs, with the value the port takes: the multi nozzle diameter mode
    /// is off (`effective_wall_loops()` is `wall_loops`, the island is `simplify_p`'d, :1537-1543, the
    /// smallest nozzle stays the one of `nozzle_diameter`, :1577-1581); a Surface has no circle
    /// compensation (`counter_circle_compensation` false, `holes_circle_compensation` empty, :1557: the
    /// walls are told to compensate no polygon); there are no perimeter regions.
    pub fn process_arachne(&self, surfaces: &[crate::surface::Surface]) -> PerimeterResult {
        use crate::clipper_utils::ApplySafetyOffset;
        use crate::print_config::{TopOneWallType, WallSequence};
        use pgcu::*;
        use pgcu_arachne::*;

        let cfg = &self.config;
        let mut result = PerimeterResult::new();
        result.pin = true;

        // other perimeters
        self.m_mm3_per_mm.set(cfg.perimeter_flow.mm3_per_mm().unwrap_or(-1.));
        // `coord_t perimeter_width = this->perimeter_flow.scaled_width();` is not read.
        let perimeter_spacing: Coord = cfg.perimeter_flow.scaled_spacing();

        // external perimeters
        self.m_ext_mm3_per_mm.set(cfg.ext_perimeter_flow.mm3_per_mm().unwrap_or(-1.));
        let ext_perimeter_width: Coord = cfg.ext_perimeter_flow.scaled_width();
        let ext_perimeter_spacing: Coord = cfg.ext_perimeter_flow.scaled_spacing();
        let ext_perimeter_spacing2: Coord = pg_scaled_float(0.5f32 * (cfg.ext_perimeter_flow.spacing() as f32 + cfg.perimeter_flow.spacing() as f32));

        // overhang perimeters
        self.m_mm3_per_mm_overhang.set(cfg.overhang_flow.as_ref().and_then(|f| f.mm3_per_mm().ok()).unwrap_or(-1.));

        // solid infill
        let solid_infill_flow = cfg.solid_infill_flow.as_ref().unwrap_or(&cfg.perimeter_flow);
        let solid_infill_spacing: Coord = solid_infill_flow.scaled_spacing();

        // `nozzle_diameter_for_filament(*print_config, config->wall_filament)`, a float: the nozzle of the
        // wall flow.
        let wall_nozzle_diameter: f32 = cfg.perimeter_flow.nozzle_diameter() as f32;

        // prepare grown lower layer slices for overhang detection
        let mut m_lower_slices_polygons: crate::geometry::Polygons = Vec::new();
        if let Some(lower_slices) = cfg.lower_slices.as_ref() {
            if cfg.detect_overhang_wall {
                // We consider overhang any part where the entire nozzle diameter is not supported by the
                // lower layer, so we take lower slices and offset them by half the nozzle diameter used
                // in the current layer
                let nozzle_diameter: f64 = wall_nozzle_diameter as f64;
                m_lower_slices_polygons = offset(lower_slices, pg_scale_(nozzle_diameter / 2.) as f32);
            }
        }

        let mut all_surfaces: Vec<crate::surface::Surface> = surfaces.to_vec();
        self.process_no_bridge(&mut all_surfaces, perimeter_spacing, ext_perimeter_width, &mut result);

        let m_scaled_resolution = self.m_scaled_resolution();
        // BBS: don't simplify too much which influence arc fitting when export gcode if arc_fitting is enabled
        let surface_simplify_resolution =
            if cfg.arc_fitting_enabled && cfg.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None { 0.2 * m_scaled_resolution } else { m_scaled_resolution };

        // The members `traverse_extrusions` reads through its `perimeter_generator`.
        let members = PerimeterGeneratorArachneMembers {
            z_direction_outwall_speed_continuous: cfg.z_direction_outwall_speed_continuous,
            wall_nozzle_diameter,
            outer_wall_line_width: cfg.outer_wall_line_width,
            detect_overhang_wall: cfg.detect_overhang_wall,
            raft_layers: cfg.raft_layers as i32,
            // `is_enable_overhang_speed(perimeter_generator) && fuzzy_skin_allows_overhang_slowdown(perimeter_generator)`
            // (PerimeterGenerator.cpp:272-286): the fuzzy skin of this crate's region has no "disabled".
            overhang_speed: cfg.enable_overhang_speed && cfg.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None,
            layer_id: cfg.layer_id as i32,
            starts_at_object_base: cfg.starts_at_object_base,
            slice_z: cfg.slice_z,
            perimeter_flow: cfg.perimeter_flow.clone(),
            ext_perimeter_flow: cfg.ext_perimeter_flow.clone(),
            overhang_flow: cfg.overhang_flow.clone().unwrap_or_else(|| cfg.perimeter_flow.clone()),
            preset_nozzle_diameter: wall_nozzle_diameter,
            multi_nozzle_diameter_mode: false,
            lower_slices_polygons: m_lower_slices_polygons,
        };
        // FuzzySkin.cpp:236-272 `apply_fuzzy_skin(extrusion, *config, *perimeter_regions, layer_id, inset_idx,
        // is_contour, slice_z)` with the region's fuzzy skin and no perimeter regions.
        let fs_config = crate::fuzzy_skin::FuzzySkinConfig { thickness: cfg.fuzzy_skin_thickness, point_distance: cfg.fuzzy_skin_point_distance, mode: cfg.fuzzy_skin_mode };
        let apply_fuzzy_skin = |extrusion: &ArachneExtrusionLine, layer_id: usize, inset_idx: usize, is_contour: bool, _slice_z: f64| -> ArachneExtrusionLine {
            if cfg.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None {
                extrusion.clone()
            } else {
                crate::fuzzy_skin::apply_fuzzy_skin_extrusion_adapter(extrusion, &fs_config, layer_id, inset_idx, is_contour)
            }
        };

        // we need to process each island separately because we might have different
        // extra perimeters for each one

        let apply_precise_outer_wall = cfg.precise_outer_wall && cfg.wall_sequence == WallSequence::InnerOuter;
        for surface in &all_surfaces {
            // detect how many perimeters must be generated for this island
            let mut loop_number: i32 = cfg.perimeter_count as i32 + surface.extra_perimeters as i32 - 1; // 0-indexed loops
            if cfg.alternate_extra_wall && cfg.layer_id % 2 == 1 && !cfg.spiral_vase {
                loop_number += 1;
            }

            let mut apply_circle_compensation = true;
            let perimeter_offset: f32 =
                if apply_precise_outer_wall { -((ext_perimeter_width - ext_perimeter_spacing) as f32) } else { -((ext_perimeter_width / 2 - ext_perimeter_spacing / 2) as f32) };
            Self::pa_probe("pa_s0", cfg.layer_id, &expolygons_to_polygons(std::slice::from_ref(&surface.expolygon)), surface_simplify_resolution as f64);
            if let Ok(v) = std::env::var("PA_L") {
                let mut it = v.split('-');
                let a = it.next().and_then(|t| t.parse::<usize>().ok());
                let b = it.next().and_then(|t| t.parse::<usize>().ok()).or(a);
                let on = matches!((a, b), (Some(a), Some(b)) if cfg.layer_id >= a && cfg.layer_id <= b);
                crate::arachne::wall_tool_paths::PA_LAYER.with(|c| c.set(if on { cfg.layer_id as i64 } else { -1 }));
            }
            let mut last: ExPolygons = offset_ex_polygons(&simplify_p(&surface.expolygon, surface_simplify_resolution), perimeter_offset);

            if !last.is_empty() {
                let new_size: i32 = last.iter().fold(0i32, |previous, expoly| previous + expoly.num_contours() as i32);
                if last.len() != 1 || new_size as usize != surface.expolygon.num_contours() {
                    apply_circle_compensation = false;
                }
            } else {
                apply_circle_compensation = false;
            }

            let mut circle_poly_indices: Vec<i32> = Vec::new();
            let mut last_p: crate::geometry::Polygons =
                if apply_circle_compensation { to_polygons_with_flag(&last[0], false, &[], &mut circle_poly_indices) } else { expolygons_to_polygons(&last) };

            let mut total_perimeters: Vec<VariableWidthLines> = Vec::new();
            let mut infill_contour: ExPolygons = Vec::new();

            if loop_number >= 0 {
                let generate_one_wall_by_first_layer = cfg.only_one_wall_first_layer && cfg.starts_at_object_base;
                let generate_one_wall_by_top_most = cfg.top_one_wall_type != TopOneWallType::None && cfg.upper_slices.is_none();
                let generate_one_wall_by_top = cfg.top_one_wall_type == TopOneWallType::AllTop && cfg.upper_slices.is_some();

                let is_one_wall = loop_number == 0 || generate_one_wall_by_first_layer || generate_one_wall_by_top_most;
                // whether to seperate the generatation of wall into two parts,first generate outer wall,then generate the remaining wall
                let mut seperate_wall_generation = !is_one_wall && generate_one_wall_by_top;

                let min_nozzle_diameter: f64 = cfg.min_nozzle_diameter;
                // `Arachne::WallToolPathsParams input_params;` has no initialisers (WallToolPaths.hpp:23-32): a
                // size whose option is zero keeps an indeterminate value at the pin, zero here.
                let mut input_params = WallToolPathsParams {
                    min_bead_width: 0.,
                    min_feature_size: 0.,
                    wall_transition_length: 0.,
                    wall_transition_angle: 0.,
                    wall_transition_filter_deviation: 0.,
                    wall_distribution_count: 0,
                };
                {
                    if cfg.arachne_min_feature_size != 0. {
                        input_params.min_feature_size = (cfg.arachne_min_feature_size * 0.01 * min_nozzle_diameter) as f32;
                    }

                    if cfg.arachne_min_bead_width != 0. {
                        input_params.min_bead_width = (cfg.arachne_min_bead_width * 0.01 * min_nozzle_diameter) as f32;
                    }

                    if cfg.wall_transition_filter_deviation != 0. {
                        input_params.wall_transition_filter_deviation = (cfg.wall_transition_filter_deviation * 0.01 * min_nozzle_diameter) as f32;
                    }

                    if cfg.arachne_wall_transition_length != 0. {
                        input_params.wall_transition_length = (cfg.arachne_wall_transition_length * 0.01 * min_nozzle_diameter) as f32;
                    }

                    input_params.wall_transition_angle = cfg.wall_transition_angle as f32;
                    input_params.wall_distribution_count = cfg.wall_distribution_count as i32;
                }

                // these variables are only valid if need to seperate wall generation
                let mut top_expolys_by_one_wall: ExPolygons = Vec::new();
                let mut first_perimeters: Vec<VariableWidthLines> = Vec::new();
                let mut infill_contour_by_one_wall: ExPolygons = Vec::new();

                let mut wall_0_inset: Coord = 0;
                if apply_precise_outer_wall {
                    wall_0_inset = -(ext_perimeter_width / 2 - ext_perimeter_spacing / 2);
                }

                // do detail check whether to enable one wall
                if seperate_wall_generation {
                    let mut one_wall_paths = WallToolPaths::new(last_p.clone(), ext_perimeter_spacing, perimeter_spacing, 1, wall_0_inset, cfg.layer_height, input_params);
                    if apply_circle_compensation {
                        one_wall_paths.enable_hole_compensation(true, circle_poly_indices.clone());
                    }

                    first_perimeters = one_wall_paths.get_tool_paths().clone();
                    infill_contour_by_one_wall = union_ex(one_wall_paths.get_inner_contour());

                    // `infill_bbox.offset(EPSILON)`: a Point of doubles, rounded to nothing (BoundingBox.hpp:42-47).
                    let infill_bbox = crate::geometry::get_extents(&infill_contour_by_one_wall);

                    let mut upper_polygons_clipped: crate::geometry::Polygons = Vec::new();
                    if let Some(upper_slices) = cfg.upper_slices.as_ref() {
                        upper_polygons_clipped = crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(upper_slices, &infill_bbox, false);
                    }
                    top_expolys_by_one_wall = diff_ex_ep(&infill_contour_by_one_wall, &upper_polygons_clipped, ApplySafetyOffset::No);

                    let mut lower_polygons_clipped: crate::geometry::Polygons = Vec::new();
                    if let Some(lower_slices) = cfg.lower_slices.as_ref() {
                        lower_polygons_clipped = crate::clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygons(lower_slices, &infill_bbox, false);
                    }
                    let bottom_expolys = diff_ex_ep(&top_expolys_by_one_wall, &lower_polygons_clipped, ApplySafetyOffset::No);

                    top_expolys_by_one_wall = diff_ex(&top_expolys_by_one_wall, &bottom_expolys, ApplySafetyOffset::No);
                    seperate_wall_generation = self.should_enable_top_one_wall(&last, &mut top_expolys_by_one_wall);
                }

                if seperate_wall_generation {
                    // only generate one wall around top areas
                    // keep the first generated wall
                    total_perimeters = first_perimeters;
                    infill_contour = union_ex_expolygons(&infill_contour_by_one_wall);
                    // deal with remaining walls to be generated
                    if loop_number > 0 {
                        last = diff_ex(&infill_contour_by_one_wall, &top_expolys_by_one_wall, ApplySafetyOffset::No);
                        last_p = expolygons_to_polygons(&last); // disable contour compensation in remaining walls
                        let mut paths_new = WallToolPaths::new(last_p.clone(), perimeter_spacing, perimeter_spacing, loop_number as usize, wall_0_inset, cfg.layer_height, input_params);
                        let new_perimeters: Vec<VariableWidthLines> = paths_new.get_tool_paths().clone();
                        for mut perimeters in new_perimeters {
                            if !perimeters.is_empty() {
                                for p in perimeters.iter_mut() {
                                    p.inset_idx += 1;
                                }
                                total_perimeters.push(perimeters);
                            }
                        }
                        infill_contour = union_ex_2(&union_ex(paths_new.get_inner_contour()), &top_expolys_by_one_wall);
                        infill_contour = intersection_ex(&infill_contour, &infill_contour_by_one_wall);
                    }
                } else {
                    if is_one_wall {
                        // plan wall width as one wall
                        let mut one_wall_paths = WallToolPaths::new(last_p.clone(), ext_perimeter_spacing, perimeter_spacing, 1, wall_0_inset, cfg.layer_height, input_params);
                        if apply_circle_compensation {
                            one_wall_paths.enable_hole_compensation(true, circle_poly_indices.clone());
                        }
                        total_perimeters = one_wall_paths.get_tool_paths().clone();
                        Self::pa_probe("pa_gic", cfg.layer_id, &one_wall_paths.get_inner_contour(), 0.);
                        infill_contour = union_ex(one_wall_paths.get_inner_contour());
                    } else {
                        // plan wall width as noraml
                        let mut normal_paths =
                            WallToolPaths::new(last_p.clone(), ext_perimeter_spacing, perimeter_spacing, (loop_number + 1) as usize, wall_0_inset, cfg.layer_height, input_params);
                        if apply_circle_compensation {
                            normal_paths.enable_hole_compensation(true, circle_poly_indices.clone());
                        }
                        total_perimeters = normal_paths.get_tool_paths().clone();
                        Self::pa_probe("pa_gic", cfg.layer_id, &normal_paths.get_inner_contour(), 0.);
                        infill_contour = union_ex(normal_paths.get_inner_contour());
                    }
                }
            } else {
                infill_contour = last.clone();
            }

            // All closed ExtrusionLine should have the same the first and the last point.
            // But in rare cases, Arachne produce ExtrusionLine marked as closed but without
            // equal the first and the last point.
            debug_assert!(total_perimeters.iter().all(|perimeter| perimeter
                .iter()
                .all(|el| !(el.is_closed && el.junctions.first().map(|j| j.p) != el.junctions.last().map(|j| j.p)))));

            let mut start_perimeter: i32 = total_perimeters.len() as i32 - 1;
            let mut end_perimeter: i32 = -1;
            let mut direction: i32 = -1;

            let mut is_outer_wall_first = cfg.wall_sequence == WallSequence::OuterInner || cfg.wall_sequence == WallSequence::InnerOuterInner;
            if cfg.starts_at_object_base {
                is_outer_wall_first = cfg.wall_sequence == WallSequence::OuterInner;
            }
            if is_outer_wall_first {
                start_perimeter = 0;
                end_perimeter = total_perimeters.len() as i32;
                direction = 1;
            }

            // `std::vector<Arachne::ExtrusionLine*> all_extrusions`: the lines themselves here, in the same
            // order; an extrusion is named by its index.
            let mut all_extrusions: Vec<ArachneExtrusionLine> = Vec::new();
            let mut perimeter_idx = start_perimeter;
            while perimeter_idx != end_perimeter {
                if !total_perimeters[perimeter_idx as usize].is_empty() {
                    for wall in &total_perimeters[perimeter_idx as usize] {
                        all_extrusions.push(wall.clone());
                    }
                }
                perimeter_idx += direction;
            }

            // Find topological order with constraints from extrusions_constrains.
            let mut blocked: Vec<usize> = vec![0; all_extrusions.len()]; // Value indicating how many extrusions it is blocking (preceding extrusions) an extrusion.
            let mut blocking: Vec<Vec<usize>> = vec![Vec::new(); all_extrusions.len()]; // Each extrusion contains a vector of extrusions that are blocked by this extrusion.

            // `map_extrusion_to_idx` is the index itself. The constraints come out of an unordered set; what
            // they are walked for (counting, and the lists whose members are all decremented) does not
            // depend on its order.
            let extrusions_constrains = {
                let refs: Vec<&ArachneExtrusionLine> = all_extrusions.iter().collect();
                WallToolPaths::get_region_order(&refs, is_outer_wall_first)
            };
            for (before, after) in extrusions_constrains {
                blocked[after] += 1;
                blocking[before].push(after);
            }

            let mut processed: Vec<bool> = vec![false; all_extrusions.len()]; // Indicate that the extrusion was already processed.
            // Some starting position. (`all_extrusions.front()->junctions.front().p`: the C++ reads the
            // first junction unchecked.)
            let mut current_position: Point = all_extrusions.first().and_then(|e| e.junctions.first()).map(|j| j.p).unwrap_or_else(|| Point::new(0, 0));
            // To store our result in: `{ best_path, best_path->is_contour(), false }` per entry.
            let mut ordered_extrusions: Vec<usize> = Vec::with_capacity(all_extrusions.len());

            while ordered_extrusions.len() < all_extrusions.len() {
                let mut best_candidate: usize = 0;
                let mut best_distance_sqr: f64 = f64::MAX;
                let mut is_best_closed = false;

                let mut available_candidates: Vec<usize> = Vec::new();
                for candidate in 0..all_extrusions.len() {
                    if processed[candidate] || blocked[candidate] != 0 {
                        continue; // Not a valid candidate.
                    }
                    available_candidates.push(candidate);
                }

                crate::clipper::libcxx_sort(&mut available_candidates, |a_idx: &usize, b_idx: &usize| -> bool {
                    !all_extrusions[*a_idx].is_closed & all_extrusions[*b_idx].is_closed
                });

                for &candidate_path_idx in &available_candidates {
                    let path = &all_extrusions[candidate_path_idx];

                    if path.junctions.is_empty() {
                        // No vertices in the path. Can't find the start position then or really plan it in. Put that at the end.
                        if best_distance_sqr == f64::MAX {
                            best_candidate = candidate_path_idx;
                            is_best_closed = path.is_closed;
                        }
                        continue;
                    }

                    let candidate_position: Point = path.junctions[0].p;
                    // `(current_position - candidate_position).cast<double>().norm()`
                    let dx = (current_position.x - candidate_position.x) as f64;
                    let dy = (current_position.y - candidate_position.y) as f64;
                    let distance_sqr: f64 = (dx * dx + dy * dy).sqrt();
                    if distance_sqr < best_distance_sqr {
                        // Closer than the best candidate so far.
                        if path.is_closed || (!path.is_closed && best_distance_sqr != f64::MAX) || (!path.is_closed && !is_best_closed) {
                            best_candidate = candidate_path_idx;
                            best_distance_sqr = distance_sqr;
                            is_best_closed = path.is_closed;
                        }
                    }
                }

                // Without a candidate (constraints in a circle) `best_candidate` is still 0: the C++ takes
                // extrusion 0 once more, and so does this.
                let best_path = &all_extrusions[best_candidate];
                ordered_extrusions.push(best_candidate);
                processed[best_candidate] = true;
                for &unlocked_idx in &blocking[best_candidate] {
                    blocked[unlocked_idx] = blocked[unlocked_idx].wrapping_sub(1);
                }

                if !best_path.junctions.is_empty() {
                    //If all paths were empty, the best path is still empty. We don't upate the current position then.
                    if best_path.is_closed {
                        current_position = best_path.junctions[0].p; //We end where we started.
                    } else {
                        current_position = best_path.junctions[best_path.junctions.len() - 1].p; //Pick the other end from where we started.
                    }
                }
            }

            // BBS. adjust wall generate seq
            if cfg.wall_sequence == WallSequence::InnerOuterInner {
                if ordered_extrusions.len() > 2 {
                    // 3 walls minimum needed to do inner outer inner ordering
                    let mut position: i32 = 0; // index to run the re-ordering for multiple external perimeters in a single island.
                    // run the re-ordering for all wall loops in the same island
                    while (position as usize) < ordered_extrusions.len() {
                        // initialise all index values to -1
                        let mut outer: i32 = -1;
                        let mut first_internal: i32 = -1;
                        let mut second_internal: i32 = -1;
                        // run through the walls to get the index values that need re-ordering until the first one for each
                        // is found. Start at "position" index to enable the for loop to iterate for multiple external
                        // perimeters in a single island
                        let mut arr_i: i32 = position; // index to run through the walls
                        while (arr_i as usize) < ordered_extrusions.len() {
                            match all_extrusions[ordered_extrusions[arr_i as usize]].inset_idx {
                                0 => {
                                    // external perimeter
                                    if outer == -1 {
                                        outer = arr_i;
                                    }
                                }
                                1 => {
                                    // first internal wall
                                    if first_internal == -1 && arr_i > outer && outer != -1 {
                                        first_internal = arr_i;
                                    }
                                }
                                2 => {
                                    // second internal wall
                                    if second_internal == -1 && arr_i > first_internal && outer != -1 {
                                        second_internal = arr_i;
                                    }
                                }
                                _ => {}
                            }
                            if outer > -1 && first_internal > -1 && second_internal > -1 {
                                break; // found all three perimeters to re-order
                            }
                            arr_i += 1;
                        }
                        if outer > -1 && first_internal > -1 && second_internal > -1 {
                            // found perimeters to re-order?
                            let temp = ordered_extrusions[second_internal as usize];
                            ordered_extrusions[second_internal as usize] = ordered_extrusions[first_internal as usize];
                            ordered_extrusions[first_internal as usize] = ordered_extrusions[outer as usize];
                            ordered_extrusions[outer as usize] = temp;
                        } else {
                            break; // did not find any more candidates to re-order, so stop the while loop early
                        }
                        // go to the next perimeter to continue scanning for external walls in the same island
                        position = arr_i + 1;
                    }
                }
            }

            {
                // The extrusions in their order: an extrusion the order names twice (see above) is taken
                // twice as it was made, where the C++ would hand its second visit the fuzzified line.
                let mut ordered_lines: Vec<ArachneExtrusionLine> = ordered_extrusions.iter().map(|&idx| all_extrusions[idx].clone()).collect();
                let is_contour: Vec<bool> = ordered_lines.iter().map(|line| line.is_contour()).collect();
                let mut pg_extrusions: Vec<PerimeterGeneratorArachneExtrusion> = ordered_lines
                    .iter_mut()
                    .zip(is_contour)
                    .map(|(extrusion, is_contour)| PerimeterGeneratorArachneExtrusion { extrusion, is_contour, fuzzify: false })
                    .collect();
                Self::pa_probe("pa_last", cfg.layer_id, &expolygons_to_polygons(&last), loop_number as f64);
                Self::pa_probe("pa_ic", cfg.layer_id, &expolygons_to_polygons(&infill_contour), total_perimeters.len() as f64);
                let extrusion_coll = traverse_extrusions(&members, &mut result.loop_nodes, &mut pg_extrusions, &apply_fuzzy_skin);
                if !extrusion_coll.entities.is_empty() {
                    result.entities.entities.push(ExtrusionEntityType::Collection(Box::new(extrusion_coll)));
                }
            }

            let spacing: Coord = if total_perimeters.len() == 1 { ext_perimeter_spacing2 } else { perimeter_spacing };

            // collapse too narrow infill areas
            let min_perimeter_infill_spacing: Coord = (solid_infill_spacing as f64 * (1. - INSET_OVERLAP_TOLERANCE)) as Coord;
            // append infill areas to fill_surfaces
            self.add_infill_contour_for_arachne(infill_contour, loop_number, ext_perimeter_spacing, perimeter_spacing, min_perimeter_infill_spacing, spacing, false, &mut result);
        }
        result
    }

    //BBS:
    /// PerimeterGenerator.cpp:1455-1485 `void PerimeterGenerator::add_infill_contour_for_arachne(ExPolygons
    /// infill_contour, int loops, coord_t ext_perimeter_spacing, coord_t perimeter_spacing, coord_t
    /// min_perimeter_infill_spacing, coord_t spacing, bool is_inner_part)`. `result.infill_area` is
    /// `fill_surfaces` (the surfaces are stInternal), `result.no_overlap_area` is `fill_no_overlap`.
    #[allow(clippy::too_many_arguments)]
    fn add_infill_contour_for_arachne(
        &self,
        mut infill_contour: ExPolygons,
        loops: i32,
        ext_perimeter_spacing: Coord,
        perimeter_spacing: Coord,
        min_perimeter_infill_spacing: Coord,
        spacing: Coord,
        is_inner_part: bool,
        result: &mut PerimeterResult,
    ) {
        use pgcu::*;
        if offset_ex(&infill_contour, -((spacing as f64 / 2.) as f32)).is_empty() {
            infill_contour.clear(); // Infill region is too small, so let's filter it out.
        }

        // create one more offset to be used as boundary for fill
        // we offset by half the perimeter spacing (to get to the actual infill boundary)
        // and then we offset back and forth by half the infill spacing to only consider the
        // non-collapsing regions
        let mut insert: Coord = if loops < 0 { 0 } else { ext_perimeter_spacing };
        if is_inner_part || loops > 0 {
            insert = perimeter_spacing;
        }

        // `config->infill_wall_overlap.get_abs_value(unscale<double>(insert))`: `ratio_over * value / 100`
        // (Config.hpp:1216)
        insert = pg_scale_(insert as f64 * SCALING_FACTOR_PIN * self.config.infill_wall_overlap_percent / 100.) as Coord;

        let m_scaled_resolution = self.m_scaled_resolution();
        let mut inner_pp: crate::geometry::Polygons = Vec::new();
        for ex in &infill_contour {
            inner_pp.extend(simplify_p(ex, m_scaled_resolution));
        }
        if std::env::var("PA_L").is_ok() {
            let l = self.config.layer_id;
            Self::pa_probe("pa_a0", l, &expolygons_to_polygons(&infill_contour), insert as f64);
            Self::pa_probe("pa_ipp", l, &inner_pp, m_scaled_resolution as f64);
            Self::pa_probe("pa_u", l, &expolygons_to_polygons(&union_ex(&inner_pp)), min_perimeter_infill_spacing as f64);
            let fs = offset2_ex(&union_ex(&inner_pp), (-(min_perimeter_infill_spacing as f64) / 2.) as f32, (insert as f64 + min_perimeter_infill_spacing as f64 / 2.) as f32);
            Self::pa_probe("pa_fs", l, &expolygons_to_polygons(&fs), spacing as f64);
            let no = offset2_ex(&union_ex(&inner_pp), (-(min_perimeter_infill_spacing as f64) / 2.) as f32, (min_perimeter_infill_spacing as f64 / 2.) as f32);
            Self::pa_probe("pa_no", l, &expolygons_to_polygons(&no), 0.);
        }

        result.infill_area.extend(offset2_ex(
            &union_ex(&inner_pp),
            (-(min_perimeter_infill_spacing as f64) / 2.) as f32,
            (insert as f64 + min_perimeter_infill_spacing as f64 / 2.) as f32,
        ));

        result
            .no_overlap_area
            .extend(offset2_ex(&union_ex(&inner_pp), (-(min_perimeter_infill_spacing as f64) / 2.) as f32, (min_perimeter_infill_spacing as f64 / 2.) as f32));
    }

    // expand the top expoly and determine whether to enable top one wall feature
    /// PerimeterGenerator.cpp:1851-1873 `bool PerimeterGenerator::should_enable_top_one_wall(const ExPolygons&
    /// original_expolys, ExPolygons& top)`
    fn should_enable_top_one_wall(&self, original_expolys: &[ExPolygon], top: &mut ExPolygons) -> bool {
        use pgcu::*;
        let perimeter_width: Coord = self.config.perimeter_flow.scaled_width();
        let ext_perimeter_spacing: Coord = self.config.ext_perimeter_flow.scaled_spacing();

        let get_expolygs_area = |expolys: &[ExPolygon]| -> f64 { expolys.iter().fold(0f64, |val, expoly| val + expoly.area()) };

        //BBS: filter small area and extend top surface a bit to hide the wall line
        let min_width_top_surface: f64 = (self.config.top_area_threshold / 100.) * f64::max(ext_perimeter_spacing as f64 / 2.0, perimeter_width as f64 / 2.0);
        let shrunk_top = offset_ex(top, (-min_width_top_surface) as f32);
        let shrunk_area: f64 = get_expolygs_area(&shrunk_top);
        let original_area: f64 = get_expolygs_area(original_expolys);

        if shrunk_area / (original_area + crate::libslic3r::EPSILON) < 0.1 || original_area < pg_scale_(1.) * pg_scale_(1.) {
            top.clear();
        } else {
            *top = offset_ex(&shrunk_top, (min_width_top_surface + perimeter_width as f64) as f32);
        }
        !top.is_empty()
    }
    // Counterbore hole bridging: sacrificial bridge layer algorithm ported from OrcaSlicer/SuperSlicer
    /// PerimeterGenerator.cpp:1953-2115 `void PerimeterGenerator::process_no_bridge(Surfaces& all_surfaces,
    /// coord_t perimeter_spacing, coord_t ext_perimeter_width)`. `result.infill_area` is `fill_surfaces`
    /// (the surfaces are stInternal).
    ///
    /// The C++ keeps `Surface* surface = &all_surfaces[surface_idx]` over insertions into the vector; this
    /// reads `all_surfaces[surface_idx]`, which is what the pointer names wherever it is still valid.
    fn process_no_bridge(&self, all_surfaces: &mut Vec<crate::surface::Surface>, perimeter_spacing: Coord, ext_perimeter_width: Coord, result: &mut PerimeterResult) {
        use crate::clipper::JoinType;
        use crate::clipper_utils::ApplySafetyOffset;
        use crate::print_config::enums::CounterboreHoleBridgingOption::{ChbBridges, ChbFilled, ChbNone};
        use pgcu::*;
        use pgcu_arachne::*;
        let cfg = &self.config;
        //store surface for bridge infill to avoid unsupported perimeters (but the first one, this one is always good)
        let Some(lower_slices) = cfg.lower_slices.as_ref() else { return };
        if cfg.counterbore_hole_bridging != ChbNone && !lower_slices.is_empty() {
            // libslic3r.h:76 `BRIDGE_INFILL_MARGIN = 1`
            let bridged_infill_margin: f64 = pg_scale_(1.);

            let mut surface_idx: usize = 0;
            while surface_idx < all_surfaces.len() {
                let mut last: ExPolygons = vec![all_surfaces[surface_idx].expolygon.clone()];
                //compute our unsupported surface
                let unsupported: ExPolygons = diff_ex(&last, lower_slices, ApplySafetyOffset::Yes);
                if !unsupported.is_empty() {
                    //remove small overhangs
                    let mut unsupported_filtered: ExPolygons = offset2_ex(&unsupported, (-perimeter_spacing) as f64 as f32, perimeter_spacing as f64 as f32);
                    if !unsupported_filtered.is_empty() {
                        //extract only the useful part of the lower layer. The safety offset is really needed here.
                        let support: ExPolygons = diff_ex(&last, &unsupported, ApplySafetyOffset::Yes);
                        if !unsupported.is_empty() {
                            //only consider the part that can be bridged (really, by the bridge algorithm)
                            //first, separate into islands (ie, each ExPlolygon)
                            //only consider the bottom layer that intersect unsupported, to be sure it's only on our island.
                            let lower_island: &ExPolygons = &support;
                            //a detector per island
                            let mut bridgeable: ExPolygons = Vec::new();
                            for unsupported_ep in &unsupported_filtered {
                                let mut detector = crate::bridge_detector::BridgeDetector::new(unsupported_ep.clone(), lower_island, perimeter_spacing);
                                // Geometry.hpp:295 `deg2rad(angle)`: `PI * angle / 180.`
                                if detector.detect_angle(PI * cfg.bridge_angle / 180.0) {
                                    bridgeable.extend(union_ex(&detector.coverage(-1., true)));
                                }
                            }
                            if !bridgeable.is_empty() {
                                //check if we get everything or just the bridgeable area
                                if cfg.counterbore_hole_bridging == ChbFilled {
                                    //we bridge everything, even the not-bridgeable bits
                                    let mut i: usize = 0;
                                    while i < unsupported_filtered.len() {
                                        let poly_unsupp: &ExPolygon = &unsupported_filtered[i];
                                        // Polygon.cpp:137-152 `Polygon::simplify(double tolerance)`: the ring after
                                        // Douglas-Peucker through `simplify_polygons`, as `ExPolygon::simplify_p` of
                                        // an expolygon without holes.
                                        let contour_simplified: crate::geometry::Polygons =
                                            simplify_p(&ExPolygon { contour: poly_unsupp.contour.clone(), holes: Vec::new() }, perimeter_spacing as f64);
                                        let mut poly_unsupp_bigger: ExPolygon = poly_unsupp.clone();
                                        let contour_bigger: crate::geometry::Polygons = offset_polygon(&poly_unsupp_bigger.contour, bridged_infill_margin as f32);
                                        if contour_bigger.len() == 1 {
                                            poly_unsupp_bigger.contour = contour_bigger[0].clone();
                                        }

                                        //check convex, has some bridge, not overhang
                                        if contour_simplified.len() == 1
                                            && contour_bigger.len() == 1
                                            && contour_simplified[0].concave_points(0.).is_empty()
                                            && !intersection_ex(&bridgeable, std::slice::from_ref(poly_unsupp)).is_empty()
                                            && diff_ex(
                                                std::slice::from_ref(&poly_unsupp_bigger),
                                                &union_ex_2(&last, &offset_ex(&bridgeable, (bridged_infill_margin + (perimeter_spacing / 2) as f64) as f32)),
                                                ApplySafetyOffset::Yes,
                                            )
                                            .is_empty()
                                        {
                                            //ok, keep it
                                            i += 1;
                                        } else {
                                            unsupported_filtered.remove(i);
                                        }
                                    }
                                    unsupported_filtered = intersection_ex(
                                        &last,
                                        &offset2_ex(&unsupported_filtered, (-perimeter_spacing / 2) as f64 as f32, (bridged_infill_margin + (perimeter_spacing / 2) as f64) as f32),
                                    );
                                    for expol_idx in 0..unsupported_filtered.len() {
                                        unsupported_filtered[expol_idx].holes.clear();

                                        //detect inside volume
                                        let mut surface_idx_other: usize = 0;
                                        while surface_idx_other < all_surfaces.len() {
                                            if surface_idx == surface_idx_other {
                                                surface_idx_other = surface_idx_other.wrapping_add(1);
                                                continue;
                                            }
                                            if !intersection_ex(std::slice::from_ref(&unsupported_filtered[expol_idx]), std::slice::from_ref(&all_surfaces[surface_idx_other].expolygon))
                                                .is_empty()
                                            {
                                                //this means that other_surf was inside an expol holes
                                                //as we removed them, we need to add a new one
                                                let new_poly: ExPolygons = offset2_ex(
                                                    std::slice::from_ref(&all_surfaces[surface_idx_other].expolygon),
                                                    (-bridged_infill_margin - perimeter_spacing as f64) as f32,
                                                    perimeter_spacing as f64 as f32,
                                                );
                                                if new_poly.len() == 1 {
                                                    all_surfaces[surface_idx_other].expolygon = new_poly[0].clone();
                                                    let mut hole = new_poly[0].contour.clone();
                                                    hole.make_clockwise();
                                                    unsupported_filtered[expol_idx].holes.push(hole);
                                                } else {
                                                    for idx in 0..new_poly.len() {
                                                        let mut new_surf = all_surfaces[surface_idx_other].clone();
                                                        new_surf.expolygon = new_poly[idx].clone();
                                                        all_surfaces.push(new_surf);
                                                        let mut hole = new_poly[idx].contour.clone();
                                                        hole.make_clockwise();
                                                        unsupported_filtered[expol_idx].holes.push(hole);
                                                    }
                                                    all_surfaces.remove(surface_idx_other);
                                                    if surface_idx_other < surface_idx {
                                                        surface_idx -= 1;
                                                    }
                                                    surface_idx_other = surface_idx_other.wrapping_sub(1);
                                                }
                                            }
                                            surface_idx_other = surface_idx_other.wrapping_add(1);
                                        }
                                    }
                                } else if cfg.counterbore_hole_bridging == ChbBridges {
                                    //simplify to avoid most of artefacts from printing lines.
                                    let mut bridgeable_simplified: ExPolygons = Vec::new();
                                    for poly in &bridgeable {
                                        // ExPolygon.cpp:253-261 `ExPolygon::simplify(tolerance, &expolygons)`:
                                        // `union_ex(this->simplify_p(tolerance))` appended.
                                        bridgeable_simplified.extend(union_ex(&simplify_p(poly, perimeter_spacing as f64)));
                                    }
                                    bridgeable_simplified = offset2_ex(&bridgeable_simplified, (-ext_perimeter_width) as f32, ext_perimeter_width as f32);

                                    let mut unbridgeable: ExPolygons = unsupported_filtered.clone();
                                    for expol in unbridgeable.iter_mut() {
                                        expol.holes.clear();
                                    }
                                    unbridgeable = diff_ex(&unbridgeable, &bridgeable_simplified, ApplySafetyOffset::No);
                                    unbridgeable = offset2_ex(&unbridgeable, (-ext_perimeter_width * 2) as f32, (ext_perimeter_width * 2) as f32);
                                    let mut bridges_temp: ExPolygons = offset2_ex(
                                        &crate::clipper_utils::_clipper_ex(
                                            crate::clipper::ClipType::ctIntersection,
                                            &crate::support::support_common::cu::expolygons_provider(&last),
                                            &crate::support::support_common::cu::expolygons_provider(&diff_ex(&unsupported_filtered, &unbridgeable, ApplySafetyOffset::No)),
                                            ApplySafetyOffset::Yes,
                                            crate::clipper::PolyFillType::pftNonZero,
                                        ),
                                        (-ext_perimeter_width / 4) as f32,
                                        (ext_perimeter_width / 4) as f32,
                                    );
                                    //remove the overhangs section from the surface polygons
                                    let reference: ExPolygons = last.clone();
                                    last = diff_ex(&last, &unsupported_filtered, ApplySafetyOffset::No);
                                    let mut offset_to_do: f64 = bridged_infill_margin;
                                    // `bool first = true;` is written and not read.
                                    unbridgeable = diff_ex(&unbridgeable, &offset_ex(&bridges_temp, ext_perimeter_width as f32), ApplySafetyOffset::No);
                                    while offset_to_do > ext_perimeter_width as f64 * 1.5 {
                                        unbridgeable = offset2_ex_jt(
                                            &unbridgeable,
                                            (-ext_perimeter_width / 4) as f32,
                                            (ext_perimeter_width as f64 * 2.25) as f32,
                                            JoinType::jtSquare,
                                            crate::support::support_common::cu::DEFAULT_MITER_LIMIT,
                                        );
                                        bridges_temp = diff_ex(&bridges_temp, &unbridgeable, ApplySafetyOffset::No);
                                        bridges_temp = offset_ex_jt(&bridges_temp, ext_perimeter_width as f32, JoinType::jtMiter, 6.);
                                        unbridgeable = diff_ex(&unbridgeable, &offset_ex(&bridges_temp, ext_perimeter_width as f32), ApplySafetyOffset::No);
                                        offset_to_do -= ext_perimeter_width as f64;
                                    }
                                    unbridgeable = offset_ex_jt(
                                        &unbridgeable,
                                        (ext_perimeter_width as f64 + offset_to_do) as f32,
                                        JoinType::jtSquare,
                                        crate::support::support_common::cu::DEFAULT_MITER_LIMIT,
                                    );
                                    bridges_temp = diff_ex(&bridges_temp, &unbridgeable, ApplySafetyOffset::No);
                                    unsupported_filtered = offset_ex(&bridges_temp, offset_to_do as f32);
                                    unsupported_filtered = intersection_ex(&unsupported_filtered, &reference);
                                } else {
                                    unsupported_filtered.clear();
                                }
                            } else {
                                unsupported_filtered.clear();
                            }
                        }

                        if !unsupported_filtered.is_empty() {
                            //add this directly to the infill list.
                            // this will avoid to throw wrong offsets into a good polygons
                            result.infill_area.extend(unsupported_filtered.iter().cloned());

                            // store the results
                            last = diff_ex(&last, &unsupported_filtered, ApplySafetyOffset::Yes);
                            //remove "thin air" polygons (note: it assumes that all polygons below will be extruded)
                            let mut i: i32 = 0;
                            while (i as usize) < last.len() {
                                if intersection_ex(&support, std::slice::from_ref(&last[i as usize])).is_empty() {
                                    result.infill_area.push(last[i as usize].clone());
                                    last.remove(i as usize);
                                    i -= 1;
                                }
                                i += 1;
                            }
                        }
                    }
                }
                if last.is_empty() {
                    all_surfaces.remove(surface_idx);
                    surface_idx = surface_idx.wrapping_sub(1);
                } else {
                    all_surfaces[surface_idx].expolygon = last[0].clone();
                    for idx in 1..last.len() {
                        // `Surface(const Surface &other, const ExPolygon &_expolygon)`
                        let mut s = all_surfaces[surface_idx].clone();
                        s.expolygon = last[idx].clone();
                        all_surfaces.push(s);
                    }
                }
                surface_idx = surface_idx.wrapping_add(1);
            }
        }
    }
}
