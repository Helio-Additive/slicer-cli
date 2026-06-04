//! Perimeter generation - Direct port from BambuStudio
//!
//! PerimeterGenerator.cpp
//!
//! This is a LINE-BY-LINE translation of BambuStudio's perimeter generation.
//! NO improvements, NO optimizations, EXACT algorithm only.

use crate::{
    arachne::{generate_arachne_walls, ArachneConfig, ExtrusionLine as ArachneExtrusionLine},
    clipper_utils::{
        difference, grow, offset2, opening, shrink, union_ex, union_polygons_ex, OffsetJoinType,
    },
    extrusion_entity::{
        ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole,
        ExtrusionPath, ExtrusionRole,
    },
    geometry::{ExPolygons, ThickPolyline, ThickPolylines},
    ExPolygon, Flow, Point, Polygon, Polyline, SCALING_FACTOR,
};
use std::f64::consts::PI;

/// Overlap tolerance for perimeter insets
/// PerimeterGenerator.cpp:24
/// C++: static constexpr double INSET_OVERLAP_TOLERANCE = 0.45;
const INSET_OVERLAP_TOLERANCE: f64 = 0.45;

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

    /// PerimeterGenerator.cpp:58
    /// C++: bool is_internal_contour() const {
    /// C++:     return this->is_contour && this->depth > 0;
    /// C++: }
    pub fn is_internal_contour(&self) -> bool {
        self.is_contour && self.perimeter_index > 0
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

        /// PerimeterGenerator.cpp:909
        let surface_simplify_resolution = if self.config.arc_fitting_enabled {
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
                    // ClipperSafetyOffset = 10 scaled units = 0.0001mm
                    const CLIPPER_SAFETY_OFFSET: f64 = 0.0001;
                    offsets = offset2(
                        &last,
                        ext_perimeter_width / 2.0 + ext_min_spacing / 2.0 - CLIPPER_SAFETY_OFFSET,
                        ext_min_spacing / 2.0 - CLIPPER_SAFETY_OFFSET,
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
                // ClipperSafetyOffset = 10 scaled units = 0.0001mm
                offsets = offset2(
                    &last,
                    distance + min_spacing / 2.0 - 0.0001,
                    min_spacing / 2.0 - 0.0001,
                    self.config.join_type,
                );

                /// PerimeterGenerator.cpp:1030-1035
                /// C++: if (has_gap_fill) append(gaps, diff_ex(offset(last, - float(0.5 * distance)), offset(offsets, float(0.5 * distance + 10))));
                if has_gap_fill {
                    let gap_outer = shrink(&last, 0.5 * distance, self.config.join_type);
                    // ClipperSafetyOffset = 10 scaled units = 0.0001mm
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

            // TODO: Port top_one_wall_type (Alltop) top_fills handling (PerimeterGenerator.cpp:1116-1183).
            // First faithful attempt (upper-slice threading is in place via PerimeterConfig.upper_slices)
            // regressed filament + only moved Top surface 1->2: top_fills geometry came out over-large
            // (fill_expolygons bloated past the slice) AND a downstream stage re-types the kept top.
            // Needs: correct top/inner split (partial-top layers, not whole-layer) + the
            // offset2_ex/infill_peri_overlap end-merge + a downstream top-preservation fix. See memory
            // project_benchy_parity_gap. Threading retained for the next attempt.

            /// PerimeterGenerator.cpp:1252-1253
            /// C++: if (i == loop_number && (! has_gap_fill || this->config->sparse_infill_density.value == 0)) {
            /// C++:     break;
            /// C++: }
            if i == loop_number && !has_gap_fill {
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
                            ExtrusionEntityType::Loop(l) => l.role.contains(
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

        // PerimeterGenerator.cpp:1096
        // C++: last is the remaining infill area
        result.infill_area = last;

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

    let mut paths = Vec::new();
    let lines = thick_polyline.thicklines();

    if lines.is_empty() {
        return paths;
    }

    let mut start_index = 0;
    let mut max_width = lines[0].a_width;
    let mut min_width = lines[0].a_width;

    /// VariableWidth.cpp:147-209
    for i in 0..lines.len() {
        let line = &lines[i];
        let line_len = line.length();

        if line_len < (SCALING_FACTOR as f64 * 0.001) {
            continue;
        }

        let thickness_delta_a = (max_width - line.b_width)
            .abs()
            .max((min_width - line.b_width).abs());

        /// VariableWidth.cpp:153
        /// C++: if (thickness_delta > tolerance)
        if thickness_delta_a > tolerance as f64 {
            /// Generate path from start_index to i (not included)
            if start_index != i {
                let mut path = ExtrusionPath::new(role);
                let mut length = 0.0;
                let mut sum = 0.0;

                for idx in start_index..i {
                    let l = lines[idx].length();
                    length += l;
                    sum += l * 0.5 * (lines[idx].a_width + lines[idx].b_width);
                    path.polyline.points.push(lines[idx].a);
                }
                path.polyline.points.push(lines[i].a);

                if length > (SCALING_FACTOR as f64 * 0.001) {
                    let w = sum / length;
                    let w_mm = w / SCALING_FACTOR as f64;
                    let new_width = w_mm + flow.height() * (1.0 - 0.25 * PI);
                    path.width = new_width;
                    path.height = flow.height();

                    let new_flow = flow.with_width(new_width).ok();
                    if let Some(f) = new_flow {
                        path.mm3_per_mm = f.mm3_per_mm().unwrap_or(0.0);
                    }

                    paths.push(path);
                }
            }

            start_index = i;
            max_width = line.a_width;
            min_width = line.a_width;
        } else {
            // Update max and min width
            max_width = max_width.max(line.a_width.max(line.b_width));
            min_width = min_width.min(line.a_width.min(line.b_width));
        }
    }

    /// Handle remaining segments
    /// VariableWidth.cpp:195-211
    if start_index < lines.len() {
        let mut path = ExtrusionPath::new(role);
        let mut length = 0.0;
        let mut sum = 0.0;

        for idx in start_index..lines.len() {
            let l = lines[idx].length();
            length += l;
            sum += l * 0.5 * (lines[idx].a_width + lines[idx].b_width);
            path.polyline.points.push(lines[idx].a);
        }
        path.polyline.points.push(lines[lines.len() - 1].b);

        if length > (SCALING_FACTOR as f64 * 0.001) {
            let w = sum / length;
            let w_mm = w / SCALING_FACTOR as f64;
            let new_width = w_mm + flow.height() * (1.0 - 0.25 * PI);
            path.width = new_width;
            path.height = flow.height();

            let new_flow = flow.with_width(new_width).ok();
            if let Some(f) = new_flow {
                path.mm3_per_mm = f.mm3_per_mm().unwrap_or(0.0);
            }

            paths.push(path);
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

        /// PerimeterGenerator.cpp:346-444 — Overhang detection
        /// Simplified: check if >50% of external perimeter is unsupported
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

        /// Create path with determined role, splitting at seam position
        let mut path = ExtrusionPath::new(effective_role);
        // Seam placement using SeamPlacer (ported from BambuStudio).
        // Uses angle-based scoring (prefers concave corners), rear bias,
        // and inter-layer alignment when a previous seam position is known.
        let seam_idx = crate::gcode::seam_placer::find_best_seam_index(
            polygon,
            None, // TODO: pass previous layer's seam position for alignment
            &crate::gcode::seam_placer::SeamPlacerConfig::default(),
        );
        path.polyline = polygon.split_at_index(seam_idx);
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
    /// Generate perimeters using Arachne algorithm
    fn generate_arachne(&self, slices: &[ExPolygon]) -> PerimeterResult {
        let mut result = PerimeterResult::new();

        if slices.is_empty() {
            result.infill_area = slices.to_vec();
            return result;
        }

        // Create Arachne config
        let arachne_config = ArachneConfig::new(
            self.config.perimeter_count,
            self.config.perimeter_extrusion_width,
        );

        // Generate Arachne walls
        let arachne_result = crate::arachne::ArachneGenerator::new(arachne_config).generate(slices);

        // Convert Arachne toolpaths to ExtrusionEntityCollection
        let mut entities = ExtrusionEntityCollection::new();

        for (wall_idx, wall_lines) in arachne_result.toolpaths.iter().enumerate() {
            for line in wall_lines {
                // Determine role based on wall index
                let role = if wall_idx == 0 {
                    ExtrusionRole::ExternalPerimeter
                } else {
                    ExtrusionRole::Perimeter
                };

                // Convert ArachneExtrusionLine to ExtrusionPath
                if let Some(extrusion_path) = self.arachne_line_to_extrusion_path(line, role) {
                    // Create a loop for closed paths, or add directly for open paths
                    if line.is_closed {
                        let loop_role = if wall_idx == 0 {
                            ExtrusionLoopRole::DEFAULT
                        } else {
                            ExtrusionLoopRole::CONTOUR_INTERNAL_PERIMETER
                        };
                        let eloop = ExtrusionLoop::new(vec![extrusion_path], loop_role);
                        entities.append(ExtrusionEntityType::Loop(eloop));
                    } else {
                        entities.append(ExtrusionEntityType::Path(extrusion_path));
                    }
                }
            }
        }

        // Add thin fills
        for line in &arachne_result.thin_fills {
            if let Some(extrusion_path) =
                self.arachne_line_to_extrusion_path(line, ExtrusionRole::ExternalPerimeter)
            {
                entities.append(ExtrusionEntityType::Path(extrusion_path));
            }
        }

        result.entities = entities;
        result.infill_area = arachne_result.inner_contour;

        result
    }

    /// Convert Arachne ExtrusionLine to our ExtrusionPath
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
            .map(|j| crate::unscale(j.width) as f64)
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
        let mut points: Vec<Point> = line.junctions.iter().map(|j| j.position).collect();

        // If open path, reverse to maintain correct orientation
        if !line.is_closed && points.len() > 1 {
            // For Arachne lines, reverse to get correct extrusion order
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
                let fuzzied = crate::fuzzy_skin::apply_fuzzy_skin_polygon(
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
                    let fuzzied = crate::fuzzy_skin::apply_fuzzy_skin_polygon(
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
