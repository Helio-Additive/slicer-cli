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
        difference, grow, intersection, offset2, offset_expolygons, opening, shrink, union_ex,
        union_polygons_ex, OffsetJoinType,
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

    /// Gap fill areas
    pub gap_fills: ExPolygons,
}

impl PerimeterResult {
    pub fn new() -> Self {
        Self {
            entities: crate::extrusion_entity::ExtrusionEntityCollection::new(),
            infill_area: Vec::new(),
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
        let mut last = union_polygons_ex(&slice.simplify_p(surface_simplify_resolution));

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
                        let mut expolys = vec![expolygon.clone()];

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
                            let temp_result = shrink(
                                &[expolygon.clone()],
                                ext_perimeter_smaller_width / 2.0,
                                self.config.join_type,
                            );
                            offsets_with_smaller_width.extend(temp_result);
                        } else {
                            /// PerimeterGenerator.cpp:993
                            /// C++: ExPolygons temp_result = offset_ex(expolygon, -float(ext_perimeter_width / 2.));
                            let temp_result = shrink(
                                &[expolygon.clone()],
                                ext_perimeter_width / 2.0,
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
                offsets = offset2(
                    &last,
                    distance + min_spacing / 2.0 - ONE_SCALED_MM,
                    min_spacing / 2.0 - ONE_SCALED_MM,
                    self.config.join_type,
                );

                /// PerimeterGenerator.cpp:1030-1035
                /// C++: if (has_gap_fill) append(gaps, diff_ex(offset(last, - float(0.5 * distance)), offset(offsets, float(0.5 * distance + 10))));
                if has_gap_fill {
                    let gap_outer = shrink(&last, 0.5 * distance, self.config.join_type);
                    // The `+ 10` is ClipperSafetyOffset = 10 scaled units = 0.0001 mm.
                    let gap_inner = grow(&offsets, 0.5 * distance + 0.0001, self.config.join_type);
                    let detected_gaps = difference(&gap_outer, &gap_inner);
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
                // C++: Polygons upper_polygons_series_clipped =
                // C++:     ClipperUtils::clip_clipper_polygons_with_subject_bbox(*this->upper_slices, last_box);
                // C++: upper_polygons_series_clipped = offset(upper_polygons_series_clipped, min_width_top_surface);
                // (the bbox clip is a performance optimization; the offset is what matters)
                let upper_polygons_series_clipped =
                    grow(upper, min_width_top_surface, OffsetJoinType::Miter);

                // PerimeterGenerator.cpp:1139
                // C++: fill_clip = offset_ex(last, -double(ext_perimeter_spacing));
                fill_clip = offset_expolygons(&last, -ext_perimeter_spacing, OffsetJoinType::Miter);

                // PerimeterGenerator.cpp:1144
                // C++: ExPolygons top_polygons = diff_ex(last, upper_polygons_series_clipped, ApplySafetyOffset::Yes);
                let mut top_polygons = difference(&last, &upper_polygons_series_clipped);

                // PerimeterGenerator.cpp:1146
                // C++: ExPolygons temp_gap = diff_ex(top_polygons, fill_clip);
                let temp_gap = difference(&top_polygons, &fill_clip);

                // PerimeterGenerator.cpp:1147-1149
                // C++: ExPolygons inner_polygons = diff_ex(last,
                // C++:     offset_ex(top_polygons, offset_top_surface + min_width_top_surface - double(ext_perimeter_spacing / 2)),
                // C++:     ApplySafetyOffset::Yes);
                let mut inner_polygons = difference(
                    &last,
                    &offset_expolygons(
                        &top_polygons,
                        offset_top_surface + min_width_top_surface - ext_perimeter_spacing / 2.0,
                        OffsetJoinType::Miter,
                    ),
                );

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
                    let bridge_checker = offset_expolygons(
                        &difference(&last, lower),
                        1.5 * bridge_offset,
                        OffsetJoinType::Miter,
                    );
                    if !bridge_checker.is_empty()
                        && !intersection(&bridge_checker, &inner_polygons).is_empty()
                    {
                        let mut merged = inner_polygons;
                        merged.extend(bridge_checker);
                        inner_polygons = union_ex(&merged);
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
                last = intersection(&inner_polygons, &last);

                // PerimeterGenerator.cpp:1170-1171
                // C++: if (has_gap_fill) last = union_ex(last, temp_gap);
                if has_gap_fill {
                    let mut merged = last;
                    merged.extend(temp_gap);
                    last = union_ex(&merged);
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
            if !top_fills.is_empty() {
                let top_infill_exp = intersection(
                    &fill_clip,
                    &offset_expolygons(&top_fills, ext_perimeter_spacing / 2.0, OffsetJoinType::Miter),
                );
                let mut merged = infill_exp;
                merged.extend(offset_expolygons(
                    &top_infill_exp,
                    infill_peri_overlap,
                    OffsetJoinType::Miter,
                ));
                infill_exp = union_ex(&merged);
            }
            result.infill_area = infill_exp;
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
    use crate::extrusion_entity::ExtrusionPath;

    let mut result = Vec::new();

    /// VariableWidth.cpp:217
    /// C++: const float tolerance = float(scale_(0.05));
    let tolerance = (0.05 * SCALING_FACTOR as f64) as i64;

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
    tolerance: i64,
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
        if thickness_delta > tolerance as f64 {
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
                    let w = sum / length;
                    let w_mm = w / SCALING_FACTOR as f64;
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
            if thickness_delta > tolerance as f64 {
                // C++: segments = (unsigned int)ceil(thickness_delta / tolerance);
                let segments = (thickness_delta / tolerance as f64).ceil() as usize;
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
            let w = sum / length;
            let w_mm = w / SCALING_FACTOR as f64;
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

/// Greedy nearest-neighbor chain ordering for extrusion entities.
/// Port of chain_extrusion_entities() from ShortestPath.cpp:1001-1015.
/// Returns indices into `points` ordered by proximity: start from start_near,
/// then each successor is the closest unvisited entity to the previous.
/// For ExtrusionLoops first_point() == last_point(), so reversal is a no-op.
fn greedy_chain_indices(points: &[Point], start_near: Point) -> Vec<usize> {
    let n = points.len();
    if n == 0 {
        return vec![];
    }
    let mut visited = vec![false; n];
    let mut chain = Vec::with_capacity(n);
    let mut current = start_near;
    for _ in 0..n {
        let best = (0..n)
            .filter(|&i| !visited[i])
            .min_by_key(|&i| {
                let dx = (points[i].x - current.x) as i64;
                let dy = (points[i].y - current.y) as i64;
                dx * dx + dy * dy
            })
            .unwrap();
        visited[best] = true;
        chain.push(best);
        current = points[best];
    }
    chain
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

        /// PerimeterGenerator.cpp:346-432 — Overhang detection.
        /// DIVERGENCE (non-faithful substitute): C++ clips the loop polyline against the
        /// grown lower-slice series (m_lower_polygons_series / m_external_lower_polygons_series)
        /// via intersection_pl_2 / diff_pl_2, then calls detect_overhang_degree() and
        /// detect_bridge_wall() to split the loop into graded overhang/bridge ExtrusionPaths.
        /// Those inputs (the per-width lower_polygons_series and overhang_dist_boundary pairs
        /// produced by generate_lower_polygons_series()/dist_boundary()) are NOT computed in
        /// this flat-config adapter, so a coarse "single role" heuristic is used instead:
        /// if less than 90% of the external loop overlaps the grown lower slices, the whole loop
        /// is marked erOverhangPerimeter. This is a known parity gap, not the C++ algorithm.
        let effective_role =
            if is_external && config.detect_overhang_wall && config.layer_id > config.raft_layers {
                if let Some(ref lower) = config.lower_slices {
                    if !lower.is_empty() {
                        let half_width = extrusion_width / 2.0;
                        let supported = crate::clipper_utils::offset_expolygons(
                            lower,
                            half_width,
                            crate::clipper_utils::OffsetJoinType::Miter,
                        );
                        let polyline = polygon.split_at_first_point();
                        let supported_segments =
                            crate::clipper_utils::intersection_pl(&[polyline.clone()], &supported);
                        let total_len: f64 = polyline
                            .points
                            .iter()
                            .zip(polyline.points.iter().skip(1))
                            .map(|(a, b)| {
                                let dx = (b.x - a.x) as f64;
                                let dy = (b.y - a.y) as f64;
                                (dx * dx + dy * dy).sqrt()
                            })
                            .sum();
                        let supported_len: f64 = supported_segments
                            .iter()
                            .flat_map(|s| s.points.iter().zip(s.points.iter().skip(1)))
                            .map(|(a, b)| {
                                let dx = (b.x - a.x) as f64;
                                let dy = (b.y - a.y) as f64;
                                (dx * dx + dy * dy).sqrt()
                            })
                            .sum();
                        if total_len > 0.0 && supported_len / total_len < 0.9 {
                            ExtrusionRole::OverhangPerimeter
                        } else {
                            role
                        }
                    } else {
                        role
                    }
                } else {
                    role
                }
            } else {
                role
            };

        /// PerimeterGenerator.cpp:441-449
        /// C++: ExtrusionPath path(role);
        /// C++: path.polyline = polygon.split_at_first_point();
        /// C++: path.overhang_degree = 0; path.curve_degree = 0;
        /// Faithful: the perimeter generator does NOT place the seam here — it
        /// splits the loop at its first point. Seam placement is a later
        /// (gcode) stage in BambuStudio. (Previously this used a SeamPlacer,
        /// which diverged from the C++ control flow.)
        let mut path = ExtrusionPath::new(effective_role);
        path.polyline = polygon.split_at_first_point();
        path.overhang_degree = 0;
        path.curve_degree = 0;
        path.mm3_per_mm = extrusion_mm3_per_mm;
        path.width = extrusion_width;
        path.height = layer_height;
        paths.push(path);

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
    let mut zero_point = Point::new(0, 0);

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
    /// PerimeterGenerator.cpp:472-502
    /// C++: std::vector<std::pair<size_t, bool>> chain = chain_extrusion_entities(coll.entities, &zero_point);
    /// C++: ExtrusionEntityCollection out;
    // Collect first_points for all entities to feed into greedy NN chain.
    let first_points: Vec<Point> = coll
        .entities
        .iter()
        .map(|e| match e {
            ExtrusionEntityType::Loop(l) => l.first_point(),
            ExtrusionEntityType::Path(p) => p.first_point(),
            ExtrusionEntityType::Collection(c) => c.first_point().unwrap_or(Point::new(0, 0)),
        })
        .collect();
    let chain = greedy_chain_indices(&first_points, zero_point);

    // Move entities out of coll for indexed access.
    let mut entities: Vec<Option<ExtrusionEntityType>> =
        coll.entities.drain(..).map(Some).collect();

    let mut out = ExtrusionEntityCollection::new();

    /// PerimeterGenerator.cpp:474-502
    /// C++: for (const std::pair<size_t, bool> &idx : chain) {
    for orig_idx in chain {
        /// PerimeterGenerator.cpp:475
        /// C++: assert(coll.entities[idx.first] != nullptr);
        let entity = entities[orig_idx].take().unwrap();

        /// PerimeterGenerator.cpp:476-483
        /// C++: if (idx.first >= loops.size()) {
        /// C++:     // This is a thin wall.
        /// C++:     out.entities.emplace_back(coll.entities[idx.first]);
        /// C++:     // if (idx.second) out.entities.back()->reverse();
        /// C++: } else {
        if orig_idx >= loops.len() {
            // This is a thin wall — append as-is (reversal not needed for loops/open paths here)
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
            let mut surface_infill: ExPolygons;

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
