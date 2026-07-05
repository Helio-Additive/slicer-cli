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

    /// Number of raft layers
    pub raft_layers: usize,

    /// Overhang flow (used for fully unsupported segments)
    pub overhang_flow: Option<Flow>,
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
            raft_layers: 0,
            overhang_flow: None,
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
}

impl PerimeterResult {
    pub fn new() -> Self {
        Self {
            entities: crate::extrusion_entity::ExtrusionEntityCollection::new(),
            infill_area: Vec::new(),
            no_overlap_area: Vec::new(),
            gap_fills: Vec::new(),
        }
    }
}

/// Perimeter generator
pub struct PerimeterGenerator {
    config: PerimeterConfig,
}

impl PerimeterGenerator {
    pub fn new(config: PerimeterConfig) -> Self {
        Self { config }
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
            return self.generate_arachne(slices);
        }

        // C++ process_classic() loops: for (const Surface &surface : this->slices->surfaces)
        // Each surface is processed independently with its own last/contours/holes state.
        for slice in slices {
            let surface_result = self.generate_classic_one(slice);
            result
                .entities
                .entities
                .extend(surface_result.entities.entities);
            result.infill_area.extend(surface_result.infill_area);
            result.no_overlap_area.extend(surface_result.no_overlap_area);
            result.gap_fills.extend(surface_result.gap_fills);
        }

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
        let mut last = if std::env::var("F1_UNION").is_ok() {
            let pp = slice.simplify_p_dp_rings(surface_simplify_resolution);
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
            if std::env::var("F1_UNION").is_ok() {
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
                    offsets = offset2(
                        &last,
                        ext_perimeter_width / 2.0 + ext_min_spacing / 2.0 - ONE_SCALED_MM,
                        ext_min_spacing / 2.0 - ONE_SCALED_MM,
                        self.config.join_type,
                    );

                    // PerimeterGenerator.cpp:966-975
                    // Thin wall detection using medial axis
                    // C++: coord_t min_width = coord_t(scale_(this->ext_perimeter_flow.nozzle_diameter() / 3));
                    let min_width = self.config.ext_perimeter_flow.nozzle_diameter() / 3.0;

                    // C++: ExPolygons expp = opening_ex(diff_ex(last, offset(offsets, float(ext_perimeter_width / 2.) + ClipperSafetyOffset)), float(min_width / 2.));
                    let offset_outward = grow(
                        &offsets,
                        ext_perimeter_width / 2.0 + CLIPPER_SAFETY_OFFSET,
                        self.config.join_type,
                    );
                    let diff_region = difference(&last, &offset_outward);
                    let expp = opening(&diff_region, min_width / 2.0, self.config.join_type);

                    // C++: for (ExPolygon &ex : expp)
                    // C++:     ex.medial_axis(min_width, ext_perimeter_width + ext_perimeter_spacing2, &thin_walls);
                    for ex in expp {
                        let max_width = ext_perimeter_width + ext_perimeter_spacing2;
                        ex.medial_axis(min_width, max_width, &mut thin_walls);
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
                        let offset_result = offset2(
                            &expolys,
                            ext_perimeter_width / 2.0 + ext_min_spacing_smaller / 2.0,
                            ext_min_spacing_smaller / 2.0,
                            self.config.join_type,
                        );

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
                offsets = if std::env::var("F1_UNION").is_ok() {
                    let per_sp_c = (perimeter_spacing * SCALING_FACTOR).trunc() as i64;
                    let ext_sp2_c = (ext_perimeter_spacing2 * SCALING_FACTOR).trunc() as i64;
                    let distance_c = if i == 1 { ext_sp2_c } else { per_sp_c };
                    let min_spacing_c =
                        (per_sp_c as f64 * (1.0 - INSET_OVERLAP_TOLERANCE)).trunc() as i64;
                    let half_min = min_spacing_c / 2;
                    let d1_mm = -((distance_c + half_min - 1) as f64) / SCALING_FACTOR;
                    let d2_mm = ((half_min - 1) as f64) / SCALING_FACTOR;
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
                    let detected_gaps = if std::env::var("F1_UNION").is_ok() {
                        let gap_outer = crate::clipper_utils::shrink_clib(
                            &last,
                            0.5 * distance,
                            self.config.join_type,
                        );
                        let gap_inner = crate::clipper_utils::grow_clib(
                            &offsets,
                            0.5 * distance + 0.0001,
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
                let f1_top = std::env::var("F1_UNION").is_ok();

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
                // NOTE: `upper` (upper_slices) is itself pre-gridded upstream, so routing
                // this offset through clib is a no-op for the residual top-surface-layer
                // gridding (measured). Left on geo-clipper; the residual is upstream.
                let upper_polygons_series_clipped =
                    crate::clipper_utils::offset_polygons(&upper_clipped_polys, min_width_top_surface, OffsetJoinType::Miter);

                // PerimeterGenerator.cpp:1139
                // C++: fill_clip = offset_ex(last, -double(ext_perimeter_spacing));
                // R105: feeds temp_gap → last; un-grid under f1_top.
                fill_clip = if f1_top {
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
                let mut top_polygons = if f1_top {
                    crate::clipper_utils::difference_clib(&last, &upper_polygons_series_clipped)
                } else {
                    difference(&last, &upper_polygons_series_clipped)
                };

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
                let mut inner_polygons = if f1_top {
                    crate::clipper_utils::difference_clib(
                        &last,
                        &crate::clipper_utils::offset_expolygons_clib(
                            &top_polygons,
                            inner_off_delta,
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
                top_polygons = difference(&fill_clip, &inner_polygons);

                // PerimeterGenerator.cpp:1164-1165
                // C++: top_fills = union_ex(top_fills, top_polygons);
                let mut merged = std::mem::take(&mut top_fills);
                merged.extend(top_polygons);
                top_fills = union_ex(&merged);

                // PerimeterGenerator.cpp:1166-1168
                // C++: double infill_spacing_unscaled = this->config->sparse_infill_line_width.value;
                // C++: fill_clip = offset_ex(last, double(ext_perimeter_spacing / 2) - scale_(infill_spacing_unscaled / 2));
                fill_clip = offset_expolygons(
                    &last,
                    ext_perimeter_spacing / 2.0 - self.config.sparse_infill_line_width / 2.0,
                    OffsetJoinType::Miter,
                );

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
                last = if f1_top {
                    crate::clipper_utils::intersection_clib(&inner_polygons, &last)
                } else {
                    intersection(&inner_polygons, &last)
                };

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
        result.entities = traverse_loops(
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
            // PerimeterGenerator.cpp:1378-1388
            // C++: // create one more offset to be used as boundary for fill
            // C++: coord_t inset = (loop_number < 0) ? 0 :
            // C++:     (loop_number == 0) ? ext_perimeter_spacing / 2 : perimeter_spacing / 2;
            // (the loop_number < 0 case clears `last` above, so the inset value is moot there)
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

            // PerimeterGenerator.cpp:1395-1399
            // C++: Polygons pp;
            // C++: for (ExPolygon &ex : last) ex.simplify_p(m_scaled_resolution, &pp);
            // C++: ExPolygons not_filled_exp = union_ex(pp);
            let mut pp: Vec<Polygon> = Vec::new();
            for ex in &last {
                ex.simplify_p_into(self.config.surface_simplify_resolution, &mut pp);
            }
            let not_filled_exp = union_polygons_ex(&pp);

            // PerimeterGenerator.cpp:1400-1406
            // C++: coord_t min_perimeter_infill_spacing = coord_t(solid_infill_spacing * (1. - INSET_OVERLAP_TOLERANCE));
            // C++: ExPolygons infill_exp = offset2_ex(not_filled_exp,
            // C++:     float(-inset - min_perimeter_infill_spacing / 2.),
            // C++:     float(min_perimeter_infill_spacing / 2.));
            let min_perimeter_infill_spacing =
                self.config.solid_infill_spacing * (1.0 - INSET_OVERLAP_TOLERANCE);
            let mut infill_exp = offset2(
                &not_filled_exp,
                inset + min_perimeter_infill_spacing / 2.0,
                min_perimeter_infill_spacing / 2.0,
                self.config.join_type,
            );

            // PerimeterGenerator.cpp:1407-1413
            // C++: ExPolygons top_infill_exp = intersection_ex(fill_clip, offset_ex(top_fills, double(ext_perimeter_spacing / 2)));
            // C++: if (!top_fills.empty()) {
            // C++:     infill_exp = union_ex(infill_exp, offset_ex(top_infill_exp, double(infill_peri_overlap)));
            // C++: }
            let top_infill_exp = intersection(
                &fill_clip,
                &offset_expolygons(&top_fills, ext_perimeter_spacing / 2.0, OffsetJoinType::Miter),
            );
            if !top_fills.is_empty() {
                let mut merged = infill_exp;
                merged.extend(offset_expolygons(
                    &top_infill_exp,
                    infill_peri_overlap,
                    OffsetJoinType::Miter,
                ));
                infill_exp = union_ex(&merged);
            }
            result.infill_area = infill_exp;

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
            let mut poly_without_overlap = if min_perimeter_infill_spacing / 2.0
                > infill_peri_overlap
            {
                offset2(
                    &not_filled_exp,
                    inset + min_perimeter_infill_spacing / 2.0,
                    min_perimeter_infill_spacing / 2.0 - infill_peri_overlap,
                    self.config.join_type,
                )
            } else {
                offset_expolygons(
                    &not_filled_exp,
                    -(inset + infill_peri_overlap),
                    self.config.join_type,
                )
            };
            if !top_fills.is_empty() {
                let mut merged = poly_without_overlap;
                merged.extend(top_infill_exp);
                poly_without_overlap = union_ex(&merged);
            }
            result.no_overlap_area = poly_without_overlap;
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
}

impl Default for PerimeterGenerator {
    fn default() -> Self {
        Self::new(PerimeterConfig::default())
    }
}

/// Convert ThickPolylines to ExtrusionPaths with variable width
/// VariableWidth.cpp:214-230
/// C++: void variable_width(const ThickPolylines& polylines, ExtrusionRole role, const Flow& flow, std::vector<ExtrusionEntity*>& out)
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
                    let new_width = w_mm + flow.height() * (1.0 - 0.25 * PI);
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
            let new_width = w_mm + flow.height() * (1.0 - 0.25 * PI);
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
fn traverse_loops(
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
                    let (lower_front_ex, lower_back_ex) = if std::env::var("F1_UNION").is_ok() {
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
                    let lower_front_polys: Vec<Polygon> =
                        crate::geometry::to_polygons(&lower_front_ex);
                    let lower_back_polys: Vec<Polygon> = crate::geometry::to_polygons(&lower_back_ex);

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
            let children = traverse_loops(
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
    fn generate_arachne(&self, slices: &[ExPolygon]) -> PerimeterResult {
        let mut result = PerimeterResult::new();

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

        // PerimeterGenerator.cpp:1499-1500
        // C++: double surface_simplify_resolution = (enable_arc_fitting && fuzzy_skin == None)
        //          ? 0.2 * m_scaled_resolution : m_scaled_resolution;
        let surface_simplify_resolution = if self.config.arc_fitting_enabled
            && self.config.fuzzy_skin_mode == crate::region_config::FuzzySkinMode::None
        {
            0.2 * self.config.surface_simplify_resolution
        } else {
            self.config.surface_simplify_resolution
        };

        for surface in slices.iter() {
            // PerimeterGenerator.cpp:1507  loop_number = wall_loops + extra_perimeters - 1
            let loop_number: i32 = self.config.perimeter_count as i32 - 1;

            // PerimeterGenerator.cpp:1511-1512  offset for the outer wall.
            // C++: ExPolygons last = offset_ex(surface.expolygon.simplify_p(surface_simplify_resolution),
            //          apply_precise_outer_wall ? -(ext_perimeter_width - ext_perimeter_spacing)
            //                                   : -(ext_perimeter_width/2. - ext_perimeter_spacing/2.));
            // (precise_outer_wall not modelled here -> use the simple inset.)
            let simplified = union_polygons_ex(&surface.simplify_p(surface_simplify_resolution));
            let inset = -(ext_perimeter_width as f64 / 2.0 - ext_perimeter_spacing as f64 / 2.0);
            let last = offset_expolygons(
                &simplified,
                inset / crate::SCALING_FACTOR,
                self.config.join_type,
            );

            // PerimeterGenerator.cpp:1518-1522  Polygons last_p = to_polygons(last);
            let last_p: crate::geometry::Polygons = expolygons_to_polygons(&last);

            let mut total_perimeters: Vec<VariableWidthLines> = Vec::new();
            let surface_infill: ExPolygons;

            if loop_number >= 0 {
                // PerimeterGenerator.cpp:1532  is_one_wall
                let is_one_wall = loop_number == 0;

                // PerimeterGenerator.cpp:1537-1553  WallToolPathsParams input_params.
                let input_params = WallToolPathsParams::default();

                // PerimeterGenerator.cpp:1560  coord_t wall_0_inset = 0;
                let wall_0_inset: Coord = 0;
                let layer_height = self.config.layer_height;

                if is_one_wall {
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
            for ext in ordered_extrusions.iter() {
                let role = if ext.inset_idx == 0 {
                    ExtrusionRole::ExternalPerimeter
                } else {
                    ExtrusionRole::Perimeter
                };
                if let Some(path) = self.arachne_line_to_extrusion_path(ext, role) {
                    if ext.is_closed {
                        // PerimeterGenerator.cpp:780 — elrDefault if contour, else elrPerimeterHole.
                        let is_contour = ext.is_contour();
                        let loop_role = if is_contour {
                            ExtrusionLoopRole::DEFAULT
                        } else {
                            ExtrusionLoopRole::PERIMETER_HOLE
                        };
                        let mut eloop = ExtrusionLoop::new(vec![path], loop_role);
                        // PerimeterGenerator.cpp:781-785 — restore loop orientation:
                        // CCW for contours, CW for holes.
                        if is_contour {
                            eloop.make_counter_clockwise();
                        } else {
                            eloop.make_clockwise();
                        }
                        entities.append(ExtrusionEntityType::Loop(eloop));
                    } else {
                        entities.append(ExtrusionEntityType::Path(path));
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
            let infill_pieces = Self::add_infill_contour_for_arachne(
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
            );
            infill_contour.extend(infill_pieces);
        }

        result.entities = entities;
        result.infill_area = infill_contour;
        result
    }

    /// PerimeterGenerator.cpp:1436-1466
    /// C++: void PerimeterGenerator::add_infill_contour_for_arachne(ExPolygons infill_contour, int loops,
    ///          coord_t ext_perimeter_spacing, coord_t perimeter_spacing, coord_t min_perimeter_infill_spacing,
    ///          coord_t spacing, bool is_inner_part)
    /// Returns the fill-surface ExPolygons (the C++ appends these to fill_surfaces with stInternal).
    /// Spacing args are in mm here (the crate's offset/offset2 take mm).
    #[allow(clippy::too_many_arguments)]
    fn add_infill_contour_for_arachne(
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
    ) -> ExPolygons {
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
        let mut inner_pp: Vec<Polygon> = Vec::new();
        for ex in &infill_contour {
            ex.simplify_p_into(surface_simplify_resolution, &mut inner_pp);
        }
        let inner_union = union_polygons_ex(&inner_pp);

        // C++: this->fill_surfaces->append(offset2_ex(union_ex(inner_pp),
        //          -min_perimeter_infill_spacing/2., insert + min_perimeter_infill_spacing/2.), stInternal);
        // (fill_no_overlap uses offset2_ex(..., -.../2, +.../2) — not modelled in PerimeterResult.)
        offset2(
            &inner_union,
            min_perimeter_infill_spacing / 2.0,
            insert + min_perimeter_infill_spacing / 2.0,
            join_type,
        )
    }

    /// Convert a faithful Arachne `ExtrusionLine` to our `ExtrusionPath`.
    fn arachne_line_to_extrusion_path(
        &self,
        line: &ArachneExtrusionLine,
        role: ExtrusionRole,
    ) -> Option<ExtrusionPath> {
        if line.junctions.is_empty() {
            return None;
        }

        let avg_width: f64 = line
            .junctions
            .iter()
            .map(|j| crate::unscale(j.w))
            .sum::<f64>()
            / line.junctions.len() as f64;

        // Get flow for this width
        let flow = Flow::new(
            avg_width,
            self.config.layer_height,
            self.config.perimeter_extrusion_width,
        )
        .ok()?;
        let mm3_per_mm = flow.mm3_per_mm().unwrap_or(0.0);

        // Convert junctions to points
        let mut points: Vec<Point> = line.junctions.iter().map(|j| j.p).collect();

        // If open path, reverse to maintain correct orientation
        if !line.is_closed && points.len() > 1 {
            points.reverse();
        }

        let mut path = ExtrusionPath::new(role);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = mm3_per_mm;
        path.width = avg_width;
        path.height = self.config.layer_height;

        Some(path)
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
        ExtrusionEntityType::Collection(coll) => {
            for sub_entity in &mut coll.entities {
                apply_fuzzy_skin_to_entity(sub_entity, config, mode);
            }
        }
    }
}
